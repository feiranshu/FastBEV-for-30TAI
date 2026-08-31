#!/usr/bin/env python3
"""Capture one still image from a discovered DirectShow PnP source."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
from datetime import datetime
from pathlib import Path

from probe_cameras import ffmpeg_path


ROOT = Path(__file__).resolve().parent


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("devices_json", type=Path)
    parser.add_argument("index", type=int)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=5)
    args = parser.parse_args()
    sources = json.loads(args.devices_json.read_text(encoding="utf-8"))["sources"]
    if not 0 <= args.index < len(sources):
        raise ValueError(f"Index must be between 0 and {len(sources) - 1}.")
    output_dir = ROOT / "checks" / f"camera_preview_{datetime.now():%Y%m%d_%H%M%S}"
    output_dir.mkdir(parents=True, exist_ok=False)
    output = output_dir / f"source_{args.index:02d}.jpg"
    command = [
        str(ffmpeg_path()), "-hide_banner", "-loglevel", "warning", "-f", "dshow",
        "-vcodec", "mjpeg",
        "-video_size", f"{args.width}x{args.height}", "-framerate", str(args.fps),
        "-i", f"video={sources[args.index]}", "-frames:v", "1", "-q:v", "2", "-y", str(output),
    ]
    subprocess.run(command, check=True)
    print(output)
    os.startfile(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
