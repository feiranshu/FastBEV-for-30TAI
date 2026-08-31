#!/usr/bin/env python3
"""Discover unique FFmpeg DirectShow sources for identical USB cameras."""

from __future__ import annotations

import argparse
import json
import re
import subprocess
from datetime import datetime
from pathlib import Path


ROOT = Path(__file__).resolve().parent
SOFTWARE_SOURCE_PREFIX = "@device_sw_"


def ffmpeg_path() -> Path:
    path = ROOT / "runtime" / "ffmpeg" / "bin" / "ffmpeg.exe"
    if not path.exists():
        raise FileNotFoundError(f"FFmpeg not found: {path}. Run setup_ffmpeg.cmd first.")
    return path


def discover_devices(vid_pid: str | None = "vid_1bcf&pid_2cc8") -> list[dict[str, str]]:
    command = [str(ffmpeg_path()), "-hide_banner", "-list_devices", "true", "-f", "dshow", "-i", "dummy"]
    result = subprocess.run(command, capture_output=True, text=True, encoding="utf-8", errors="replace")
    text = result.stderr + "\n" + result.stdout
    pairs = re.findall(
        r'\] "([^"]+)" \((?:video|none)\)\s*\n\[[^\n]+\]\s+Alternative name "([^"]+)"',
        text,
    )
    devices = [
        {"name": name, "source": source}
        for name, source in pairs
        if not source.startswith(SOFTWARE_SOURCE_PREFIX)
    ]
    if vid_pid:
        wanted = vid_pid.lower()
        devices = [device for device in devices if wanted in device["source"].lower()]
    return sorted(devices, key=lambda item: (item["name"], item["source"]))


def discover_sources(vid_pid: str | None = "vid_1bcf&pid_2cc8") -> list[str]:
    return [device["source"] for device in discover_devices(vid_pid)]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--vid-pid", default="vid_1bcf&pid_2cc8", help="case-insensitive USB VID/PID filter")
    parser.add_argument("--all", action="store_true", help="list all non-virtual DirectShow video devices")
    args = parser.parse_args()
    devices = discover_devices(None if args.all else args.vid_pid)
    sources = [device["source"] for device in devices]
    payload = {"filter": None if args.all else args.vid_pid, "devices": devices, "sources": sources}
    output_dir = ROOT / "checks"
    output_dir.mkdir(exist_ok=True)
    output = output_dir / f"camera_devices_{datetime.now():%Y%m%d_%H%M%S}.json"
    output.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    if not sources:
        raise RuntimeError("No matching camera sources found. Check that the seven LRCP cameras are connected.")
    print(f"Found {len(sources)} matching cameras:")
    for index, device in enumerate(devices):
        print(f"  [{index}] {device['name']}")
        print(f"      {device['source']}")
    print(f"\nSaved: {output}")
    print("Preview each: preview_camera.cmd <index> <saved-json-path>")
    print("Then map the six vehicle-facing cameras: configure_cameras.cmd <saved-json-path>")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
