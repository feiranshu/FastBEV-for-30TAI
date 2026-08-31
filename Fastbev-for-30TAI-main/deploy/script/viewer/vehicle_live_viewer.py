#!/usr/bin/env python3
"""Vehicle live interactive viewer backend.

PC capture code pushes one synchronized six-camera frame and board detections
to /api/push_vehicle_frame. The browser polls /api/live and fetches preview
JPEGs from /api/live_image/<frame>/<camera>.jpg.
"""

from __future__ import annotations

import argparse
import base64
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import unquote, urlparse

import vehicle_viewer_geometry as geometry


DEPLOY_DIR = Path(__file__).resolve().parents[2]
STATIC_DIR = Path(__file__).resolve().parent / "web_viewer_vehicle"
MAX_PUSH_BODY_BYTES = 32 * 1024 * 1024
MAX_IMAGE_BYTES = 2 * 1024 * 1024


class VehicleLiveState:
    def __init__(self, camera_params: Path, score_threshold: float) -> None:
        self.camera_params = camera_params
        self.score_threshold = score_threshold
        self.lock = threading.Lock()
        self.sequence = 0
        self.frame: dict[str, Any] | None = None
        self.images: dict[str, bytes] = {}
        self.push_received_epoch_ms: float | None = None

    def push(self, payload: dict[str, Any]) -> dict[str, Any]:
        frame_id = str(payload.get("frame_id", "0000")).zfill(4)
        objects = payload.get("objects") or []
        images_payload = payload.get("images") or {}
        if not isinstance(objects, list):
            raise ValueError("objects must be a list")
        if not isinstance(images_payload, dict):
            raise ValueError("images must be an object")

        decoded_images: dict[str, bytes] = {}
        for view in geometry.DISPLAY_CAMERA_ORDER:
            text = images_payload.get(view)
            if text is None:
                continue
            image = base64.b64decode(text, validate=True)
            if len(image) > MAX_IMAGE_BYTES:
                raise ValueError(f"{view} image too large")
            decoded_images[view] = image

        with self.lock:
            self.sequence += 1
            self.push_received_epoch_ms = time.time() * 1000.0
            live = {
                "mode": "vehicle",
                "sequence": self.sequence,
                "frame_id": frame_id,
                "push_received_epoch_ms": self.push_received_epoch_ms,
                "source": payload.get("source", "vehicle-real-edge"),
                "inference_ms": payload.get("inference_ms"),
                "edge_roundtrip_ms": payload.get("edge_roundtrip_ms"),
                "input_capture_ts_ns": payload.get("input_capture_ts_ns"),
                "finished_ts_ns": payload.get("finished_ts_ns"),
                "timing": payload.get("timing") or {},
            }
            self.images = decoded_images
            self.frame = geometry.build_frame(
                frame_id,
                objects,
                self.camera_params,
                live=live,
                score_threshold=self.score_threshold,
            )
            return dict(live)

    def latest(self) -> dict[str, Any]:
        with self.lock:
            if self.frame is None:
                return {
                    "frame_id": "0000",
                    "image_size": [geometry.LAYOUT["width"], geometry.LAYOUT["height"]],
                    "layout": geometry.LAYOUT,
                    "score_threshold_default": self.score_threshold,
                    "camera_images": [],
                    "detections": [],
                    "live": {"mode": "vehicle_waiting", "sequence": self.sequence},
                }
            return json.loads(json.dumps(self.frame))

    def image(self, frame_id: str, view: str) -> bytes | None:
        with self.lock:
            if self.frame is None or str(self.frame.get("frame_id")) != frame_id:
                return None
            return self.images.get(view)


def json_response(handler: BaseHTTPRequestHandler, data: Any, status: int = 200) -> None:
    body = json.dumps(data, ensure_ascii=False).encode("utf-8")
    handler.send_response(status)
    handler.send_header("Content-Type", "application/json; charset=utf-8")
    handler.send_header("Content-Length", str(len(body)))
    handler.send_header("Cache-Control", "no-store")
    handler.end_headers()
    handler.wfile.write(body)


def bytes_response(handler: BaseHTTPRequestHandler, body: bytes, content_type: str) -> None:
    handler.send_response(200)
    handler.send_header("Content-Type", content_type)
    handler.send_header("Content-Length", str(len(body)))
    handler.send_header("Cache-Control", "no-store")
    handler.end_headers()
    handler.wfile.write(body)


def make_handler(state: VehicleLiveState) -> type[BaseHTTPRequestHandler]:
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt: str, *args: Any) -> None:
            print(f"[HTTP] {self.address_string()} - {fmt % args}")

        def do_GET(self) -> None:
            parsed = urlparse(self.path)
            path = unquote(parsed.path)
            try:
                if path == "/" or path == "/index.html":
                    bytes_response(self, (STATIC_DIR / "index.html").read_bytes(),
                                   "text/html; charset=utf-8")
                    return
                if path == "/app.js":
                    bytes_response(self, (STATIC_DIR / "app.js").read_bytes(),
                                   "application/javascript; charset=utf-8")
                    return
                if path == "/style.css":
                    bytes_response(self, (STATIC_DIR / "style.css").read_bytes(),
                                   "text/css; charset=utf-8")
                    return
                if path == "/api/live":
                    json_response(self, state.latest())
                    return
                if path == "/api/frames":
                    json_response(self, {"frames": []})
                    return
                if path.startswith("/api/frame/"):
                    json_response(self, state.latest())
                    return
                if path.startswith("/api/live_image/") and path.endswith(".jpg"):
                    parts = path.split("/")
                    if len(parts) != 5:
                        raise ValueError("bad live image path")
                    frame_id = parts[3]
                    view = parts[4][:-4]
                    image = state.image(frame_id, view)
                    if image is None:
                        json_response(self, {"error": "image not found"}, 404)
                    else:
                        bytes_response(self, image, "image/jpeg")
                    return
                json_response(self, {"error": "not found"}, 404)
            except Exception as exc:
                json_response(self, {"error": str(exc)}, 500)

        def do_POST(self) -> None:
            parsed = urlparse(self.path)
            path = unquote(parsed.path)
            try:
                if path != "/api/push_vehicle_frame":
                    json_response(self, {"error": "not found"}, 404)
                    return
                length = int(self.headers.get("Content-Length", "0"))
                if length <= 0 or length > MAX_PUSH_BODY_BYTES:
                    json_response(self, {"error": "invalid body length"}, 413)
                    return
                payload = json.loads(self.rfile.read(length).decode("utf-8"))
                live = state.push(payload)
                json_response(self, {"ok": True, "live": live})
            except Exception as exc:
                json_response(self, {"ok": False, "error": str(exc)}, 400)

    return Handler


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Vehicle live interactive viewer")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8093)
    parser.add_argument(
        "--camera-params",
        default=str(DEPLOY_DIR / "io" / "input" / "vehicle_camera_params.txt"),
        help="Vehicle camera_params.txt used for browser-side geometry",
    )
    parser.add_argument("--score-threshold", type=float, default=0.6)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    camera_params = Path(args.camera_params)
    if not camera_params.is_file():
        raise FileNotFoundError(f"camera params not found: {camera_params}")
    if not STATIC_DIR.is_dir():
        raise FileNotFoundError(f"static viewer dir not found: {STATIC_DIR}")

    state = VehicleLiveState(camera_params, args.score_threshold)
    server = ThreadingHTTPServer((args.host, args.port), make_handler(state))
    print(f"[VehicleViewer] http://{args.host}:{args.port}/?live=1")
    print(f"[VehicleViewer] camera_params={camera_params}")
    server.serve_forever()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
