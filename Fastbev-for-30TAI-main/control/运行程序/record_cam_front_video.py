#!/usr/bin/env python3
"""Record the configured CAM_FRONT DirectShow source to one video file."""

from __future__ import annotations

import argparse
import json
import subprocess
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from probe_cameras import ffmpeg_path


ROOT = Path(__file__).resolve().parent


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def camera_source(config: dict[str, Any], name: str) -> str:
    for item in config["cameras"]:
        if item.get("enabled", True) and item.get("name") == name:
            source = str(item.get("source", ""))
            if source.startswith("@device_pnp_"):
                return source
            raise ValueError(f"{name} is not configured with a DirectShow PnP source.")
    raise ValueError(f"Camera not found or disabled in config: {name}")


def build_command(
    source: str,
    output: Path,
    width: int,
    height: int,
    fps: int,
    duration_s: float | None,
) -> list[str]:
    command = [
        str(ffmpeg_path()),
        "-hide_banner",
        "-loglevel",
        "warning",
        "-f",
        "dshow",
        "-rtbufsize",
        "128M",
        "-vcodec",
        "mjpeg",
        "-video_size",
        f"{width}x{height}",
        "-framerate",
        str(fps),
        "-i",
        f"video={source}",
        "-an",
        "-c:v",
        "copy",
    ]
    if duration_s is not None:
        command.extend(["-t", f"{duration_s:.3f}"])
    command.extend(["-y", str(output)])
    return command


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera-config", type=Path, default=ROOT / "config" / "cameras.json")
    parser.add_argument("--output-root", type=Path, default=ROOT.parent / "采集数据" / "front_video")
    parser.add_argument("--duration-s", type=float, help="record this many seconds, otherwise record until Ctrl+C")
    parser.add_argument("--camera-name", default="CAM_FRONT")
    args = parser.parse_args()

    config = read_json(args.camera_config)
    source = camera_source(config, args.camera_name)
    width = int(config.get("width", 640))
    height = int(config.get("height", 480))
    fps = int(config.get("fps", 15))

    session_dir = args.output_root / f"session_{datetime.now():%Y%m%d_%H%M%S}"
    session_dir.mkdir(parents=True, exist_ok=False)
    output = session_dir / f"{args.camera_name}.mkv"
    command = build_command(source, output, width, height, fps, args.duration_s)

    metadata = {
        "started_at": now_iso(),
        "camera_name": args.camera_name,
        "source": source,
        "width": width,
        "height": height,
        "fps": fps,
        "duration_s": args.duration_s,
        "output": str(output),
        "container": "matroska",
        "video_codec": "mjpeg copy",
    }
    metadata_path = session_dir / "recording.json"
    metadata_path.write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    print(f"Recording {args.camera_name} to: {output}")
    print("Stop with Ctrl+C." if args.duration_s is None else f"Recording for {args.duration_s:.3f} s.")
    try:
        completed = subprocess.run(command)
    except KeyboardInterrupt:
        print("Stopping recording...")
        return 130
    metadata["finished_at"] = now_iso()
    metadata["returncode"] = completed.returncode
    metadata_path.write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    if completed.returncode != 0:
        print(
            "FFmpeg recording failed. If the six-camera web dashboard is running, "
            "close it first because CAM_FRONT cannot be opened by two processes at once."
        )
        return completed.returncode
    print(f"Saved: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
