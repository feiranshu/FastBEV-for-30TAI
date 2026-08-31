#!/usr/bin/env python3
"""Send Vehicle six-camera Raw BGR frames to the board and push results to Viewer.

The reusable boundary is send_vehicle_frame(): callers provide six JPEG/PNG
image bytes from the PC UVC rolling buffers, this script decodes them to
640x480 BGR uint8, sends Raw BGR to the board, waits for one result, then
pushes the same preview images plus detections to the Vehicle Viewer.
"""

from __future__ import annotations

import argparse
import base64
import json
import socket
import struct
import subprocess
import time
import zlib
from pathlib import Path
from typing import Any
from urllib import error, request


CAMERA_ORDER = [
    "CAM_FRONT",
    "CAM_FRONT_RIGHT",
    "CAM_FRONT_LEFT",
    "CAM_BACK",
    "CAM_BACK_LEFT",
    "CAM_BACK_RIGHT",
]

DEFAULT_IMAGE_FILES = {
    "CAM_FRONT": "0-FRONT.jpg",
    "CAM_FRONT_RIGHT": "1-FRONT_RIGHT.jpg",
    "CAM_FRONT_LEFT": "2-FRONT_LEFT.jpg",
    "CAM_BACK": "3-BACK.jpg",
    "CAM_BACK_LEFT": "4-BACK_LEFT.jpg",
    "CAM_BACK_RIGHT": "5-BACK_RIGHT.jpg",
}

MAGIC = b"VEH1"
VERSION = 1
MSG_BATCH = 1
MSG_RESULT = 2
HEADER_STRUCT = struct.Struct(">4sHHQQII")
ENTRY_STRUCT = struct.Struct(">16sHHHHII")
WIDTH = 640
HEIGHT = 480
CHANNELS = 3
RAW_BYTES = WIDTH * HEIGHT * CHANNELS
MAX_PACKET_BYTES = 64 * 1024 * 1024
APP_DIR = Path(__file__).resolve().parents[3]
CONTROL_FFMPEG = (
    APP_DIR / "task" / "小车操作流程" / "new" / "运行程序" /
    "runtime" / "ffmpeg" / "bin" / "ffmpeg.exe"
)
DEFAULT_FFMPEG = str(CONTROL_FFMPEG) if CONTROL_FFMPEG.is_file() else "ffmpeg"


def now_ns() -> int:
    return time.time_ns()


def decode_to_bgr(image_bytes: bytes, view: str, ffmpeg: str) -> bytes:
    if len(image_bytes) == RAW_BYTES:
        return image_bytes
    cmd = [
        ffmpeg,
        "-hide_banner",
        "-loglevel",
        "error",
        "-i",
        "pipe:0",
        "-f",
        "rawvideo",
        "-pix_fmt",
        "bgr24",
        "pipe:1",
    ]
    proc = subprocess.run(
        cmd,
        input=image_bytes,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    if proc.returncode != 0:
        message = proc.stderr.decode("utf-8", errors="replace").strip()
        raise RuntimeError(f"ffmpeg failed to decode {view}: {message}")
    if len(proc.stdout) != RAW_BYTES:
        raise ValueError(f"{view} decoded bytes={len(proc.stdout)}, expected {RAW_BYTES}")
    return proc.stdout


def pack_vehicle_packet(frame_id: int, capture_ts_ns: int,
                        preview_images: dict[str, bytes], ffmpeg: str) -> bytes:
    body = bytearray()
    for view in CAMERA_ORDER:
        if view not in preview_images:
            raise ValueError(f"missing camera image: {view}")
        raw = decode_to_bgr(preview_images[view], view, ffmpeg)
        crc = zlib.crc32(raw) & 0xFFFFFFFF
        name = view.encode("ascii")
        if len(name) > 15:
            raise ValueError(f"camera name too long: {view}")
        body += ENTRY_STRUCT.pack(
            name.ljust(16, b"\0"),
            WIDTH,
            HEIGHT,
            CHANNELS,
            0,
            len(raw),
            crc,
        )
        body += raw
    body_bytes = bytes(body)
    header = HEADER_STRUCT.pack(
        MAGIC,
        VERSION,
        MSG_BATCH,
        frame_id,
        capture_ts_ns,
        len(CAMERA_ORDER),
        len(body_bytes),
    )
    packet = header + body_bytes
    if len(packet) > MAX_PACKET_BYTES:
        raise ValueError(f"packet too large: {len(packet)}")
    return struct.pack(">I", len(packet)) + packet


def recv_exact(sock: socket.socket, size: int) -> bytes:
    chunks = bytearray()
    while len(chunks) < size:
        chunk = sock.recv(size - len(chunks))
        if not chunk:
            raise ConnectionError("socket closed while receiving result")
        chunks += chunk
    return bytes(chunks)


def recv_vehicle_result(sock: socket.socket) -> dict[str, Any]:
    packet_size = struct.unpack(">I", recv_exact(sock, 4))[0]
    if packet_size < HEADER_STRUCT.size or packet_size > MAX_PACKET_BYTES:
        raise ValueError(f"invalid result packet size: {packet_size}")
    packet = recv_exact(sock, packet_size)
    magic, version, msg_type, frame_id, finished_ts_ns, item_count, payload_size = (
        HEADER_STRUCT.unpack(packet[:HEADER_STRUCT.size])
    )
    if magic != MAGIC:
        raise ValueError(f"bad result magic: {magic!r}")
    if version != VERSION:
        raise ValueError(f"unsupported result version: {version}")
    if msg_type != MSG_RESULT:
        raise ValueError(f"unexpected result message type: {msg_type}")
    payload = packet[HEADER_STRUCT.size:]
    if len(payload) != payload_size:
        raise ValueError("result payload size mismatch")
    result = json.loads(payload.decode("utf-8"))
    if int(result.get("frame_id", frame_id)) != frame_id:
        raise ValueError("result frame_id mismatch")
    if int(item_count) != len(result.get("objects", [])):
        raise ValueError("result object count mismatch")
    result["finished_ts_ns"] = result.get("finished_ts_ns", finished_ts_ns)
    return result


def push_to_viewer(viewer_url: str, result: dict[str, Any],
                   preview_images: dict[str, bytes], edge_roundtrip_ms: float) -> None:
    payload = {
        "frame_id": str(int(result["frame_id"])).zfill(4),
        "source": result.get("source", "vehicle-real-edge"),
        "inference_ms": result.get("inference_ms"),
        "edge_roundtrip_ms": edge_roundtrip_ms,
        "input_capture_ts_ns": result.get("input_capture_ts_ns"),
        "finished_ts_ns": result.get("finished_ts_ns"),
        "timing": result.get("timing") or {},
        "objects": result.get("objects") or [],
        "images": {
            view: base64.b64encode(preview_images[view]).decode("ascii")
            for view in CAMERA_ORDER
        },
    }
    body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
    req = request.Request(
        viewer_url.rstrip("/") + "/api/push_vehicle_frame",
        data=body,
        headers={"Content-Type": "application/json; charset=utf-8"},
        method="POST",
    )
    try:
        with request.urlopen(req, timeout=10.0) as response:
            if response.status >= 400:
                raise RuntimeError(f"viewer push failed: HTTP {response.status}")
    except error.HTTPError as exc:
        detail = exc.read().decode("utf-8", errors="replace").strip()
        raise RuntimeError(f"viewer push failed: HTTP {exc.code}: {detail}") from exc


def send_vehicle_frame(edge_host: str, edge_port: int, viewer_url: str,
                       frame_id: int, preview_images: dict[str, bytes],
                       ffmpeg: str = DEFAULT_FFMPEG) -> dict[str, Any]:
    capture_ts_ns = now_ns()
    packet = pack_vehicle_packet(frame_id, capture_ts_ns, preview_images, ffmpeg)
    with socket.create_connection((edge_host, edge_port), timeout=10.0) as sock:
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        started = time.perf_counter()
        sock.sendall(packet)
        result = recv_vehicle_result(sock)
        edge_roundtrip_ms = (time.perf_counter() - started) * 1000.0
    push_to_viewer(viewer_url, result, preview_images, edge_roundtrip_ms)
    return result


def load_sample_images(sample_dir: Path) -> dict[str, bytes]:
    images: dict[str, bytes] = {}
    for view, filename in DEFAULT_IMAGE_FILES.items():
        path = sample_dir / filename
        if not path.is_file():
            raise FileNotFoundError(f"sample image not found: {path}")
        images[view] = path.read_bytes()
    return images


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Vehicle Raw BGR board sender")
    parser.add_argument("--edge-host", required=True)
    parser.add_argument("--edge-port", type=int, default=5200)
    parser.add_argument("--viewer", default="http://127.0.0.1:8093")
    parser.add_argument(
        "--sample-dir",
        default=str(Path(__file__).resolve().parents[2] / "io" / "input" / "vehicle_sample"),
        help="Temporary source for one six-camera frame. Live UVC code can call send_vehicle_frame().",
    )
    parser.add_argument("--frame-id", type=int, default=1)
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--interval-ms", type=int, default=1000)
    parser.add_argument("--ffmpeg", default=DEFAULT_FFMPEG)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.repeat <= 0:
        raise ValueError("--repeat must be positive")
    sample_dir = Path(args.sample_dir)
    for i in range(args.repeat):
        frame_id = args.frame_id + i
        images = load_sample_images(sample_dir)
        result = send_vehicle_frame(args.edge_host, args.edge_port, args.viewer,
                                    frame_id, images, args.ffmpeg)
        print(
            f"[VehicleLive] frame={frame_id:04d} objects={len(result.get('objects', []))} "
            f"inference_ms={float(result.get('inference_ms') or 0.0):.2f}"
        )
        if i + 1 < args.repeat:
            time.sleep(max(args.interval_ms, 0) / 1000.0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
