#!/usr/bin/env python3
"""PC-side PS live server for FastBEV result display.

This file models the serial FastBEV pipeline boundary on the PC side:

  board/PS result ready -> PC receive/cache -> web API -> browser Canvas

It supports three sources without changing the browser:

* loop: local result/parameter files;
* push: board previews plus local result/parameter files;
* carla: same-frame CARLA JPEGs and real board detections forwarded by Gateway.
"""

from __future__ import annotations

import argparse
import base64
import binascii
import json
import math
import re
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import unquote, urlparse

import interactive_viewer as viewer
import carla_nuscenes_geometry


DEPLOY_DIR = Path(__file__).resolve().parents[2]
STATIC_DIR = Path(__file__).resolve().parent / "web_viewer"
MAX_RESULT_BYTES = 2 * 1024 * 1024
MAX_CAMERA_PARAMS_BYTES = 256 * 1024
MAX_LIVE_IMAGE_BYTES = 2 * 1024 * 1024
MAX_PUSH_BODY_BYTES = 16 * 1024 * 1024
CARLA_CAMERAS = carla_nuscenes_geometry.build_camera_infos()
CARLA_SCORE_THRESHOLD = 0.185


def parse_frame_list(value: str) -> list[str]:
    frames = [item.strip() for item in value.split(",") if item.strip()]
    if not frames:
        raise ValueError("empty frame list")
    normalized = []
    for frame_id in frames:
        if not re.fullmatch(r"\d{1,4}", frame_id):
            raise ValueError(f"frame id must be 1-4 digits: {frame_id}")
        normalized.append(frame_id.zfill(4))
    return normalized


class LivePipelineState:
    """Small in-memory state for loop simulation and future board pushes."""

    def __init__(self, loop_frames: list[str], period_ms: int) -> None:
        self.loop_frames = loop_frames
        self.period_s = max(period_ms, 1) / 1000.0
        self.started_s = time.monotonic()
        self.lock = threading.Lock()
        self.push_frame_id: str | None = None
        self.push_sequence = 0
        self.push_received_epoch_ms: float | None = None
        self.push_images: dict[str, bytes] = {}
        self.carla_frame: dict[str, Any] | None = None

    def loop_frame(self) -> tuple[str, dict[str, Any]]:
        seq = int((time.monotonic() - self.started_s) / self.period_s)
        frame_id = self.loop_frames[seq % len(self.loop_frames)]
        meta = {
            "mode": "loop",
            "sequence": seq,
            "frames": self.loop_frames,
            "period_ms": round(self.period_s * 1000.0),
        }
        return frame_id, meta

    def push_frame(self, frame_id: str, images: dict[str, bytes] | None = None) -> dict[str, Any]:
        with self.lock:
            self.push_frame_id = frame_id
            self.push_sequence += 1
            self.push_received_epoch_ms = time.time() * 1000.0
            self.push_images = dict(images or {})
            return {
                "mode": "push",
                "sequence": self.push_sequence,
                "frame_id": frame_id,
                "push_received_epoch_ms": self.push_received_epoch_ms,
            }

    def live_image(self, frame_id: str, view: str) -> bytes | None:
        with self.lock:
            if frame_id != self.push_frame_id:
                return None
            return self.push_images.get(view)

    def latest_push_frame(self) -> tuple[str | None, dict[str, Any]]:
        with self.lock:
            if self.push_frame_id is None:
                return None, {"mode": "push", "sequence": self.push_sequence}
            return self.push_frame_id, {
                "mode": "push",
                "sequence": self.push_sequence,
                "frame_id": self.push_frame_id,
                "push_received_epoch_ms": self.push_received_epoch_ms,
            }

    def push_carla_frame(
        self,
        frame_id: str,
        images: dict[str, bytes],
        detections: list[dict[str, Any]],
        metadata: dict[str, Any],
    ) -> dict[str, Any]:
        with self.lock:
            self.push_frame_id = frame_id
            self.push_sequence += 1
            self.push_received_epoch_ms = time.time() * 1000.0
            self.push_images = dict(images)
            live = {
                "mode": "carla",
                "sequence": self.push_sequence,
                "frame_id": frame_id,
                "push_received_epoch_ms": self.push_received_epoch_ms,
                **metadata,
            }
            self.carla_frame = build_carla_frame(frame_id, detections, live)
            return dict(live)

    def latest_carla_frame(self) -> dict[str, Any] | None:
        with self.lock:
            if self.carla_frame is None:
                return None
            return json.loads(json.dumps(self.carla_frame))


def finite_number(value: Any, field: str) -> float:
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"object field is not finite: {field}")
    return number


def carla_detection(det_id: int, obj: dict[str, Any]) -> dict[str, Any]:
    x = finite_number(obj["x"], "x")
    y = finite_number(obj["y"], "y")
    z = finite_number(obj["z"], "z")
    w = finite_number(obj.get("dx", obj.get("w")), "dx")
    l = finite_number(obj.get("dy", obj.get("l")), "dy")
    h = finite_number(obj.get("dz", obj.get("h")), "dz")
    yaw = finite_number(obj["yaw"], "yaw")
    class_id = int(obj["class_id"])
    score = finite_number(obj["score"], "score")
    raw = f"{x} {y} {z} {w} {l} {h} {yaw} {class_id} {score}"
    # FastBEV uses a row-vector yaw convention while the shared viewer corner
    # helper uses column vectors. Negating yaw reproduces the authoritative
    # LiDARInstance3DBoxes corners used by the CARLA server overlay.
    detection = viewer.Detection(x, y, z, w, l, h, -yaw, class_id, score, raw)
    geometry = viewer.geometry_for_detection(det_id, detection, CARLA_CAMERAS)
    geometry["box"]["yaw"] = yaw
    geometry["raw"] = raw
    return geometry


def build_carla_frame(
    frame_id: str,
    detections: list[dict[str, Any]],
    live: dict[str, Any],
) -> dict[str, Any]:
    sequence = live["sequence"]
    # Enforce the CARLA display threshold in the backend. The browser slider
    # can further reduce this set, but cannot reveal discarded detections.
    filtered_detections = [
        obj
        for obj in detections
        if finite_number(obj.get("score", 0.0), "score") > CARLA_SCORE_THRESHOLD
    ]
    return {
        "frame_id": frame_id,
        "image_size": [viewer.OUT_W, viewer.OUT_H],
        "layout": {
            "width": viewer.OUT_W,
            "height": viewer.OUT_H,
            "bev": {"x": viewer.BEV_X, "y": viewer.BEV_Y, "size": viewer.K_CANVAS},
            "camera_tile": {"width": viewer.CAM_TILE_W, "height": viewer.CAM_TILE_H},
            "views": viewer.VIEWS,
        },
        "camera_images": [
            {
                "view": view,
                "url": f"/api/image/{frame_id}/{view}?live_seq={sequence}",
            }
            for view in viewer.VIEWS
        ],
        "score_threshold_default": CARLA_SCORE_THRESHOLD,
        "detections": [
            carla_detection(i, obj) for i, obj in enumerate(filtered_detections)
        ],
        "live": live,
    }


def frame_files_exist(output_root: Path, frame_id: str) -> bool:
    result, params = viewer.frame_paths(output_root, frame_id)
    return result.exists() and params.exists()


def write_uploaded_frame(output_root: Path, frame_id: str, result_text: str, camera_params_text: str) -> None:
    if len(result_text.encode("utf-8")) > MAX_RESULT_BYTES:
        raise ValueError("result_text exceeds upload limit")
    if len(camera_params_text.encode("utf-8")) > MAX_CAMERA_PARAMS_BYTES:
        raise ValueError("camera_params_text exceeds upload limit")

    result_path, params_path = viewer.frame_paths(output_root, frame_id)
    result_path.parent.mkdir(parents=True, exist_ok=True)
    params_path.parent.mkdir(parents=True, exist_ok=True)
    result_path.write_text(result_text, encoding="utf-8")
    params_path.write_text(camera_params_text, encoding="utf-8")


def build_live_frame(server: ThreadingHTTPServer) -> dict[str, Any]:
    output_root: Path = server.output_root  # type: ignore[attr-defined]
    state: LivePipelineState = server.live_state  # type: ignore[attr-defined]
    mode: str = server.live_mode  # type: ignore[attr-defined]

    if mode == "carla":
        data = state.latest_carla_frame()
        if data is not None:
            return data
        frame_id, loop_meta = state.loop_frame()
        data = viewer.build_frame(output_root, frame_id)
        data["live"] = {"mode": "carla_waiting", "fallback": loop_meta}
        return data
    if mode == "push":
        frame_id, meta = state.latest_push_frame()
        if frame_id is None:
            frame_id, loop_meta = state.loop_frame()
            meta = {"mode": "push_waiting", "fallback": loop_meta}
    else:
        frame_id, meta = state.loop_frame()

    data = viewer.build_frame(output_root, frame_id)
    if mode == "push" and "sequence" in meta:
        for image in data["camera_images"]:
            image["url"] = f"{image['url']}?live_seq={meta['sequence']}"
    data["live"] = meta
    return data


class PcPsLiveHandler(BaseHTTPRequestHandler):
    server_version = "FastBEVPcPsLive/0.1"

    def send_json(self, obj: Any, status: int = 200) -> None:
        body = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def send_file(self, path: Path, content_type: str | None = None) -> None:
        if not path.exists() or not path.is_file():
            self.send_json({"error": f"not found: {path.name}"}, 404)
            return
        body = path.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", content_type or viewer.mimetypes.guess_type(path.name)[0] or "application/octet-stream")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def send_bytes(self, body: bytes, content_type: str) -> None:
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        path = unquote(parsed.path)
        output_root: Path = self.server.output_root  # type: ignore[attr-defined]
        deploy_root: Path = self.server.deploy_root  # type: ignore[attr-defined]

        try:
            if path == "/api/frames":
                self.send_json({"frames": viewer.scan_frames(output_root)})
                return
            if path == "/api/live":
                self.send_json(build_live_frame(self.server))  # type: ignore[arg-type]
                return
            if path == "/api/status":
                state: LivePipelineState = self.server.live_state  # type: ignore[attr-defined]
                self.send_json({
                    "mode": self.server.live_mode,  # type: ignore[attr-defined]
                    "loop_frames": state.loop_frames,
                    "period_ms": round(state.period_s * 1000.0),
                    "push": state.latest_push_frame()[1],
                })
                return
            if path.startswith("/api/frame/"):
                frame_id = path.rsplit("/", 1)[-1]
                if not re.fullmatch(r"\d{4}", frame_id):
                    self.send_json({"error": "frame id must be 4 digits"}, 400)
                    return
                self.send_json(viewer.build_frame(output_root, frame_id))
                return
            if path.startswith("/api/image/"):
                parts = path.strip("/").split("/")
                if len(parts) != 4:
                    self.send_json({"error": "expected /api/image/<frame>/<view>"}, 400)
                    return
                _, _, frame_id, view = parts
                frame_pattern = (
                    r"\d{1,20}" if self.server.live_mode == "carla" else r"\d{4}"
                )  # type: ignore[attr-defined]
                if not re.fullmatch(frame_pattern, frame_id) or view not in viewer.VIEWS:
                    self.send_json({"error": "invalid image request"}, 400)
                    return
                if self.server.live_mode in {"push", "carla"}:  # type: ignore[attr-defined]
                    state: LivePipelineState = self.server.live_state  # type: ignore[attr-defined]
                    image_bytes = state.live_image(frame_id, view)
                    if image_bytes is None:
                        self.send_json({"error": f"board image unavailable: {frame_id}/{view}"}, 404)
                        return
                    self.send_bytes(image_bytes, "image/jpeg")
                    return
                _, params = viewer.frame_paths(output_root, frame_id)
                cams = viewer.load_cameras(params)
                if view not in cams:
                    self.send_json({"error": f"missing camera {view}"}, 404)
                    return
                image_path = viewer.resolve_image_path(deploy_root, cams[view].img_path)
                self.send_file(image_path)
                return
            if path == "/" or path == "/index.html":
                self.send_file(STATIC_DIR / "index.html", "text/html; charset=utf-8")
                return
            if path in {"/app.js", "/style.css"}:
                self.send_file(STATIC_DIR / path.lstrip("/"))
                return
            if path == "/favicon.ico":
                self.send_response(204)
                self.end_headers()
                return
            self.send_json({"error": "not found"}, 404)
        except Exception as exc:
            self.send_json({"error": str(exc)}, 500)

    def do_POST(self) -> None:
        parsed = urlparse(self.path)
        path = unquote(parsed.path)
        output_root: Path = self.server.output_root  # type: ignore[attr-defined]
        state: LivePipelineState = self.server.live_state  # type: ignore[attr-defined]

        try:
            if path not in {"/api/push_frame", "/api/push_carla_frame"}:
                self.send_json({"error": "not found"}, 404)
                return
            length = int(self.headers.get("Content-Length", "0"))
            if length < 0 or length > MAX_PUSH_BODY_BYTES:
                self.send_json({"error": "request body exceeds upload limit"}, 413)
                return
            payload = json.loads(self.rfile.read(length).decode("utf-8")) if length else {}
            frame_id = str(payload.get("frame_id", ""))
            if not re.fullmatch(r"\d{1,20}", frame_id):
                self.send_json({"error": "frame_id must be numeric"}, 400)
                return
            if path == "/api/push_carla_frame":
                objects = payload.get("objects")
                image_payload = payload.get("images")
                if not isinstance(objects, list) or not isinstance(image_payload, dict):
                    self.send_json({"error": "objects must be a list and images must be an object"}, 400)
                    return
                images = self.decode_live_images(image_payload)
                metadata = {
                    "capture_ts_ns": int(payload.get("capture_ts_ns", 0)),
                    "inference_ms": finite_number(payload.get("inference_ms", 0.0), "inference_ms"),
                    "gateway_roundtrip_ms": finite_number(
                        payload.get("gateway_roundtrip_ms", 0.0), "gateway_roundtrip_ms"
                    ),
                    "source": str(payload.get("source", "unknown")),
                    "object_count": len(objects),
                }
                meta = state.push_carla_frame(frame_id, images, objects, metadata)
                self.send_json({"ok": True, "live": meta})
                return
            if not re.fullmatch(r"\d{4}", frame_id):
                self.send_json({"error": "frame_id must be 4 digits for file push mode"}, 400)
                return
            result_text = payload.get("result_text")
            camera_params_text = payload.get("camera_params_text")
            if result_text is not None or camera_params_text is not None:
                if not isinstance(result_text, str) or not isinstance(camera_params_text, str):
                    self.send_json({"error": "result_text and camera_params_text must both be strings"}, 400)
                    return
                write_uploaded_frame(output_root, frame_id, result_text, camera_params_text)
            if not frame_files_exist(output_root, frame_id):
                self.send_json({"error": f"missing result/parameter for frame {frame_id}"}, 404)
                return
            image_payload = payload.get("images")
            images: dict[str, bytes] | None = None
            if image_payload is not None:
                if not isinstance(image_payload, dict):
                    self.send_json({"error": "images must be an object"}, 400)
                    return
                images = self.decode_live_images(image_payload)
            elif payload.get("source") == "fastbev_pipeline_real":
                self.send_json({"error": "real pipeline push requires six board images"}, 400)
                return
            meta = state.push_frame(frame_id, images)
            self.send_json({"ok": True, "live": meta})
        except Exception as exc:
            self.send_json({"error": str(exc)}, 500)

    def decode_live_images(self, image_payload: dict[str, Any]) -> dict[str, bytes]:
        images: dict[str, bytes] = {}
        for view in viewer.VIEWS:
            encoded = image_payload.get(view)
            if not isinstance(encoded, str):
                raise ValueError(f"missing live image: {view}")
            try:
                image_bytes = base64.b64decode(encoded.encode("ascii"), validate=True)
            except (UnicodeEncodeError, ValueError, binascii.Error) as exc:
                raise ValueError(f"invalid live image {view}: {exc}") from exc
            if not image_bytes or len(image_bytes) > MAX_LIVE_IMAGE_BYTES:
                raise ValueError(f"live image size invalid: {view}")
            if not image_bytes.startswith(b"\xff\xd8") or not image_bytes.endswith(b"\xff\xd9"):
                raise ValueError(f"invalid JPEG markers: {view}")
            images[view] = image_bytes
        return images

    def log_message(self, fmt: str, *args: Any) -> None:
        viewer.log_line(f"[pc-ps-live] {self.address_string()} - {fmt % args}")


def main() -> int:
    parser = argparse.ArgumentParser(description="PC PS FastBEV live server, loop demo frames 0-4")
    parser.add_argument("--root", default=str(DEPLOY_DIR / "io" / "output"), help="output root containing result/parameter")
    parser.add_argument("--deploy-root", default=None, help="deploy directory used to resolve relative image paths")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8092)
    parser.add_argument(
        "--mode", choices=["loop", "push", "carla"], default="loop",
        help="local loop, file push, or CARLA Gateway live frames",
    )
    parser.add_argument("--loop-frames", default="0001,0002,0003,0004,0005", help="0-4 demo frames; current files are named 0001-0005")
    parser.add_argument("--live-period-ms", type=int, default=100)
    args = parser.parse_args()

    output_root = Path(args.root).resolve()
    if not output_root.exists():
        parser.error(f"output root does not exist: {output_root}")
    deploy_root = Path(args.deploy_root).resolve() if args.deploy_root else viewer.deploy_root_from_output(output_root)
    loop_frames = parse_frame_list(args.loop_frames)
    missing = [frame_id for frame_id in loop_frames if not frame_files_exist(output_root, frame_id)]
    if missing:
        parser.error("missing result/parameter for loop frame(s): " + ", ".join(missing))

    server = ThreadingHTTPServer((args.host, args.port), PcPsLiveHandler)
    server.output_root = output_root  # type: ignore[attr-defined]
    server.deploy_root = deploy_root  # type: ignore[attr-defined]
    server.live_state = LivePipelineState(loop_frames, args.live_period_ms)  # type: ignore[attr-defined]
    server.live_mode = args.mode  # type: ignore[attr-defined]

    viewer.log_line(f"[pc-ps-live] output root: {output_root}")
    viewer.log_line(f"[pc-ps-live] deploy root: {deploy_root}")
    viewer.log_line(f"[pc-ps-live] mode: {args.mode}")
    viewer.log_line(f"[pc-ps-live] loop frames 0-4: {','.join(loop_frames)} @ {args.live_period_ms} ms")
    viewer.log_line(f"[pc-ps-live] open: http://{args.host}:{args.port}/?live=1")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        viewer.log_line("[pc-ps-live] stopped")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
