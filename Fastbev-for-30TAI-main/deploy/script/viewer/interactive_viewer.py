#!/usr/bin/env python3
"""FastBEV interactive result viewer.

Serves a small static UI plus JSON geometry computed from:
  output/result/result_XXXX.txt
  output/parameter/camera_params_XXXX.txt
  input/data/images/*.jpg

The browser draws the six camera images, BEV canvas, and result overlays itself.
No per-frame PNG generation is required.
"""

from __future__ import annotations

import argparse
import json
import math
import mimetypes
import re
import sys
import time
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import parse_qs, unquote, urlparse


DEPLOY_DIR = Path(__file__).resolve().parents[2]
STATIC_DIR = Path(__file__).resolve().parent / "web_viewer"

K_CANVAS = 1000
K_SHOW_RANGE = 50.0
K_SCORE_THRESH = 0.2
K_IMG_W = 1600
K_IMG_H = 900
K_SCALE = 4
OUT_W = 1200
OUT_H = 1450
CAM_TILE_W = K_IMG_W // K_SCALE
CAM_TILE_H = K_IMG_H // K_SCALE
BEV_X = (K_IMG_W * 3 // K_SCALE - K_CANVAS) // 2
BEV_Y = K_IMG_H // K_SCALE
BOTTOM_Y = OUT_H - CAM_TILE_H

VIEWS = [
    "CAM_FRONT_LEFT",
    "CAM_FRONT",
    "CAM_FRONT_RIGHT",
    "CAM_BACK_LEFT",
    "CAM_BACK",
    "CAM_BACK_RIGHT",
]


def log_line(message: str) -> None:
    try:
        if sys.stdout:
            sys.stdout.write(message + "\n")
            sys.stdout.flush()
    except Exception:
        pass

EDGES_3D = [
    (0, 1), (1, 2), (2, 3), (3, 0),
    (4, 5), (5, 6), (6, 7), (7, 4),
    (0, 4), (1, 5), (2, 6), (3, 7),
]


@dataclass
class CameraInfo:
    name: str
    img_path: str
    extrinsic: list[list[float]]
    intrinsic: list[list[float]]
    post_aug_inv: list[list[float]]


@dataclass
class Detection:
    x: float
    y: float
    z: float
    w: float
    l: float
    h: float
    yaw: float
    cls: int
    score: float
    raw: str


def parse_floats(line: str, n: int) -> list[float]:
    vals = [float(x) for x in line.split()]
    if len(vals) < n:
        raise ValueError(f"expected {n} floats, got {len(vals)} in: {line}")
    return vals[:n]


def mat33_inv(m: list[list[float]]) -> list[list[float]]:
    a, b, c = m[0]
    d, e, f = m[1]
    g, h, i = m[2]
    det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)
    if abs(det) < 1e-12:
        raise ValueError("singular 3x3 matrix")
    inv_det = 1.0 / det
    return [
        [(e * i - f * h) * inv_det, (c * h - b * i) * inv_det, (b * f - c * e) * inv_det],
        [(f * g - d * i) * inv_det, (a * i - c * g) * inv_det, (c * d - a * f) * inv_det],
        [(d * h - e * g) * inv_det, (b * g - a * h) * inv_det, (a * e - b * d) * inv_det],
    ]


def mat_vec(m: list[list[float]], v: list[float]) -> list[float]:
    return [sum(row[i] * v[i] for i in range(len(v))) for row in m]


def next_data_line(lines: list[str], index: int) -> tuple[str, int]:
    while index < len(lines):
        line = lines[index].split("#", 1)[0].strip()
        index += 1
        if line:
            return line, index
    raise ValueError("unexpected end of camera params")


def load_cameras(path: Path) -> dict[str, CameraInfo]:
    lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    line, idx = next_data_line(lines, 0)
    count = int(line)
    out: dict[str, CameraInfo] = {}

    for _ in range(count):
        name, idx = next_data_line(lines, idx)
        img_path, idx = next_data_line(lines, idx)

        line, idx = next_data_line(lines, idx)
        e = parse_floats(line, 16)
        extrinsic = [e[r * 4:(r + 1) * 4] for r in range(4)]

        line, idx = next_data_line(lines, idx)
        k = parse_floats(line, 9)
        intrinsic = [k[r * 3:(r + 1) * 3] for r in range(3)]

        line, idx = next_data_line(lines, idx)
        pr = parse_floats(line, 9)
        post_rot = [pr[r * 3:(r + 1) * 3] for r in range(3)]

        line, idx = next_data_line(lines, idx)
        pt = parse_floats(line, 3)

        post_aug = [
            [post_rot[0][0], post_rot[0][1], pt[0]],
            [post_rot[1][0], post_rot[1][1], pt[1]],
            [0.0, 0.0, 1.0],
        ]
        out[name] = CameraInfo(name, img_path, extrinsic, intrinsic, mat33_inv(post_aug))
    return out


def load_detections(path: Path) -> list[Detection]:
    detections: list[Detection] = []
    for raw in path.read_text(encoding="utf-8", errors="replace").splitlines():
        raw = raw.strip()
        if not raw:
            continue
        vals = raw.split()
        if len(vals) < 9:
            continue
        x, y, z, w, l, h, yaw = [float(v) for v in vals[:7]]
        # Stable rows are 9 columns. SA inserts vx/vy before class and score.
        class_index = 9 if len(vals) >= 11 else 7
        detections.append(Detection(
            x, y, z, w, l, h, yaw,
            int(float(vals[class_index])), float(vals[class_index + 1]), raw,
        ))
    return detections


def compute_corners(d: Detection) -> list[list[float]]:
    signs = [
        (-1, -1, 0), (-1, -1, 1), (-1, 1, 1), (-1, 1, 0),
        (1, -1, 0), (1, -1, 1), (1, 1, 1), (1, 1, 0),
    ]
    cy = math.cos(d.yaw)
    sy = math.sin(d.yaw)
    corners: list[list[float]] = []
    for sx, sy_sign, sz in signs:
        lx = sx * d.w * 0.5
        ly = sy_sign * d.l * 0.5
        lz = sz * d.h
        rx = lx * cy - ly * sy
        ry = lx * sy + ly * cy
        corners.append([d.x + rx, d.y + ry, d.z + lz])
    return corners


def project_point(pt: list[float], cam: CameraInfo) -> tuple[float, float, bool]:
    px, py, pz = pt
    e = cam.extrinsic
    cx = e[0][0] * px + e[0][1] * py + e[0][2] * pz + e[0][3]
    cy = e[1][0] * px + e[1][1] * py + e[1][2] * pz + e[1][3]
    cz = e[2][0] * px + e[2][1] * py + e[2][2] * pz + e[2][3]
    valid = cz > 0.5
    if abs(cz) < 1e-6:
        cz = -1e-6 if cz < 0 else 1e-6

    norm = [cx / cz, cy / cz, 1.0]
    img = mat_vec(cam.intrinsic, norm)
    out = mat_vec(cam.post_aug_inv, img)
    return out[0], out[1], valid


def bev_point(wx: float, wy_flipped: float, rounded: bool = True) -> list[float]:
    x = (wx + K_SHOW_RANGE) / K_SHOW_RANGE / 2.0 * K_CANVAS
    y = (wy_flipped + K_SHOW_RANGE) / K_SHOW_RANGE / 2.0 * K_CANVAS
    if rounded:
        x = round(x)
        y = round(y)
    else:
        x = math.trunc(x)
        y = math.trunc(y)
    return [BEV_X + x, BEV_Y + y]


def camera_tile_transform(view_index: int, x: float, y: float) -> list[float]:
    col = view_index % 3
    if view_index < 3:
        return [col * CAM_TILE_W + x / K_SCALE, y / K_SCALE]
    return [col * CAM_TILE_W + CAM_TILE_W - x / K_SCALE, BOTTOM_Y + y / K_SCALE]


def bbox(points: list[list[float]], pad: float = 6.0) -> list[float]:
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    return [min(xs) - pad, min(ys) - pad, max(xs) + pad, max(ys) + pad]


def geometry_for_detection(det_id: int, d: Detection, cams: dict[str, CameraInfo]) -> dict[str, Any]:
    corners = compute_corners(d)
    bottom = [0, 3, 7, 4]
    bev_poly = [[round(v, 3) for v in bev_point(corners[i][0], -corners[i][1], True)] for i in bottom]
    center = bev_point(sum(corners[i][0] for i in bottom) / 4.0, -sum(corners[i][1] for i in bottom) / 4.0, False)
    head = bev_point((corners[0][0] + corners[4][0]) * 0.5, (-corners[0][1] - corners[4][1]) * 0.5, False)

    camera_views = []
    visible_cameras: list[str] = []
    for view_index, name in enumerate(VIEWS):
        cam = cams.get(name)
        if not cam:
            continue
        raw_points = [project_point(c, cam) for c in corners]
        valid = [p[2] for p in raw_points]
        canvas_points = [camera_tile_transform(view_index, p[0], p[1]) for p in raw_points]
        edges = []
        for a, b in EDGES_3D:
            p0 = raw_points[a]
            p1 = raw_points[b]
            p0_in_image = 0 <= p0[0] < K_IMG_W and 0 <= p0[1] < K_IMG_H
            p1_in_image = 0 <= p1[0] < K_IMG_W and 0 <= p1[1] < K_IMG_H
            if valid[a] and valid[b] and p0_in_image and p1_in_image:
                edges.append([canvas_points[a], canvas_points[b]])
        if edges:
            visible_cameras.append(name)
            pts = [p for edge in edges for p in edge]
            camera_views.append({
                "camera": name,
                "edges": edges,
                "bbox": bbox(pts, 8.0),
            })

    return {
        "id": det_id,
        "class_id": d.cls,
        "score": d.score,
        "distance": math.sqrt(d.x * d.x + d.y * d.y),
        "box": {
            "x": d.x, "y": d.y, "z": d.z,
            "w": d.w, "l": d.l, "h": d.h,
            "yaw": d.yaw,
        },
        "raw": d.raw,
        "bev": {
            "polygon": bev_poly,
            "center": [round(center[0], 3), round(center[1], 3)],
            "heading": [[round(center[0], 3), round(center[1], 3)], [round(head[0], 3), round(head[1], 3)]],
            "bbox": bbox(bev_poly, 5.0),
        },
        "camera_views": camera_views,
        "visible_cameras": visible_cameras,
    }


def png_size(path: Path) -> list[int] | None:
    with path.open("rb") as f:
        header = f.read(24)
    if len(header) >= 24 and header[:8] == b"\x89PNG\r\n\x1a\n":
        return [int.from_bytes(header[16:20], "big"), int.from_bytes(header[20:24], "big")]
    return None


def deploy_root_from_output(output_root: Path) -> Path:
    # Expected layout: deploy/io/output/{result,parameter}
    if output_root.name == "output" and output_root.parent.name == "io":
        return output_root.parent.parent
    return DEPLOY_DIR


def resolve_image_path(deploy_root: Path, img_path: str) -> Path:
    path = Path(img_path)
    if path.is_absolute():
        return path
    return (deploy_root / path).resolve()


def frame_paths(root: Path, frame_id: str) -> tuple[Path, Path]:
    return (
        root / "result" / f"result_{frame_id}.txt",
        root / "parameter" / f"camera_params_{frame_id}.txt",
    )


def scan_frames(root: Path) -> list[dict[str, Any]]:
    pattern = re.compile(r"result_(\d{4})\.txt$")
    frames = []
    for result in sorted((root / "result").glob("result_*.txt")):
        m = pattern.match(result.name)
        if not m:
            continue
        frame_id = m.group(1)
        _, params = frame_paths(root, frame_id)
        complete = params.exists()
        frames.append({
            "id": frame_id,
            "complete": complete,
            "result": result.name if result.exists() else None,
            "parameter": params.name if params.exists() else None,
        })
    return frames


def build_frame(root: Path, frame_id: str) -> dict[str, Any]:
    result, params = frame_paths(root, frame_id)
    missing = [str(p) for p in (result, params) if not p.exists()]
    if missing:
        raise FileNotFoundError("missing frame files: " + ", ".join(missing))

    cams = load_cameras(params)
    detections = load_detections(result)
    camera_images = []
    for view in VIEWS:
        cam = cams.get(view)
        if cam:
            camera_images.append({"view": view, "url": f"/api/image/{frame_id}/{view}"})
    return {
        "frame_id": frame_id,
        "image_size": [OUT_W, OUT_H],
        "layout": {
            "width": OUT_W,
            "height": OUT_H,
            "bev": {"x": BEV_X, "y": BEV_Y, "size": K_CANVAS},
            "camera_tile": {"width": CAM_TILE_W, "height": CAM_TILE_H},
            "views": VIEWS,
        },
        "camera_images": camera_images,
        "score_threshold_default": K_SCORE_THRESH,
        "detections": [geometry_for_detection(i, d, cams) for i, d in enumerate(detections)],
    }


def parse_live_frames(value: str) -> list[str]:
    frames = [item.strip() for item in value.split(",") if item.strip()]
    bad = [item for item in frames if not re.fullmatch(r"\d{4}", item)]
    if bad:
        raise ValueError("live frame ids must be 4 digits: " + ", ".join(bad))
    if not frames:
        raise ValueError("at least one live frame is required")
    return frames


def build_live_frame(server: ThreadingHTTPServer) -> dict[str, Any]:
    root: Path = server.output_root  # type: ignore[attr-defined]
    live_frames: list[str] = server.live_frames  # type: ignore[attr-defined]
    period_s: float = server.live_period_s  # type: ignore[attr-defined]
    started_s: float = server.live_started_s  # type: ignore[attr-defined]
    seq = int((time.monotonic() - started_s) / period_s)
    frame_id = live_frames[seq % len(live_frames)]
    data = build_frame(root, frame_id)
    data["live"] = {
        "enabled": True,
        "sequence": seq,
        "frames": live_frames,
        "period_ms": round(period_s * 1000.0),
    }
    return data


class ViewerHandler(BaseHTTPRequestHandler):
    server_version = "FastBEVInteractiveViewer/0.1"

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
        self.send_header("Content-Type", content_type or mimetypes.guess_type(path.name)[0] or "application/octet-stream")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        path = unquote(parsed.path)
        root: Path = self.server.output_root  # type: ignore[attr-defined]
        deploy_root: Path = self.server.deploy_root  # type: ignore[attr-defined]

        try:
            if path == "/api/frames":
                self.send_json({"frames": scan_frames(root)})
                return
            if path == "/api/live":
                self.send_json(build_live_frame(self.server))  # type: ignore[arg-type]
                return
            if path.startswith("/api/frame/"):
                frame_id = path.rsplit("/", 1)[-1]
                if not re.fullmatch(r"\d{4}", frame_id):
                    self.send_json({"error": "frame id must be 4 digits"}, 400)
                    return
                self.send_json(build_frame(root, frame_id))
                return
            if path.startswith("/api/image/"):
                parts = path.strip("/").split("/")
                if len(parts) != 4:
                    self.send_json({"error": "expected /api/image/<frame>/<view>"}, 400)
                    return
                _, _, frame_id, view = parts
                if not re.fullmatch(r"\d{4}", frame_id) or view not in VIEWS:
                    self.send_json({"error": "invalid image request"}, 400)
                    return
                _, params = frame_paths(root, frame_id)
                cams = load_cameras(params)
                if view not in cams:
                    self.send_json({"error": f"missing camera {view}"}, 404)
                    return
                image_path = resolve_image_path(deploy_root, cams[view].img_path)
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

    def log_message(self, fmt: str, *args: Any) -> None:
        log_line(f"[viewer] {self.address_string()} - {fmt % args}")


def main() -> int:
    parser = argparse.ArgumentParser(description="FastBEV interactive output viewer")
    parser.add_argument("--root", default=str(DEPLOY_DIR / "io" / "output"), help="output root containing result/parameter")
    parser.add_argument("--deploy-root", default=None, help="deploy directory used to resolve relative image paths")
    parser.add_argument("--live-frames", default="0001,0002,0003,0004", help="comma-separated frame ids for /api/live simulation")
    parser.add_argument("--live-period-ms", type=int, default=100, help="/api/live simulated frame period")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8080)
    args = parser.parse_args()

    output_root = Path(args.root).resolve()
    if not output_root.exists():
        parser.error(f"output root does not exist: {output_root}")
    deploy_root = Path(args.deploy_root).resolve() if args.deploy_root else deploy_root_from_output(output_root)
    live_frames = parse_live_frames(args.live_frames)
    live_period_s = max(args.live_period_ms, 1) / 1000.0

    server = ThreadingHTTPServer((args.host, args.port), ViewerHandler)
    server.output_root = output_root  # type: ignore[attr-defined]
    server.deploy_root = deploy_root  # type: ignore[attr-defined]
    server.live_frames = live_frames  # type: ignore[attr-defined]
    server.live_period_s = live_period_s  # type: ignore[attr-defined]
    server.live_started_s = time.monotonic()  # type: ignore[attr-defined]
    log_line(f"[viewer] output root: {output_root}")
    log_line(f"[viewer] deploy root: {deploy_root}")
    log_line(f"[viewer] live simulation: {','.join(live_frames)} @ {round(live_period_s * 1000)} ms")
    log_line(f"[viewer] open: http://{args.host}:{args.port}/")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        log_line("[viewer] stopped")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
