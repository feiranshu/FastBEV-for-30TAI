#!/usr/bin/env python3
"""Start the PC-side Vehicle live viewer, then optionally send sample frames."""

from __future__ import annotations

import argparse
from pathlib import Path
import subprocess
import sys
import time


APP_DIR = Path(__file__).resolve().parents[1]
DEFAULT_PYTHON = APP_DIR / "deploy" / "deps" / "venv" / "Scripts" / "python.exe"
VIEWER_SCRIPT = APP_DIR / "deploy" / "script" / "viewer" / "vehicle_live_viewer.py"
SENDER_SCRIPT = APP_DIR / "deploy" / "script" / "vehicle" / "run_bev_capture_vehicle_live.py"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Start Vehicle PC viewer flow")
    parser.add_argument("--python", default=str(DEFAULT_PYTHON))
    parser.add_argument("--viewer-host", default="127.0.0.1")
    parser.add_argument("--viewer-port", type=int, default=8093)
    parser.add_argument("--edge-host", default=None)
    parser.add_argument("--edge-port", type=int, default=5200)
    parser.add_argument("--sample-once", action="store_true",
                        help="After starting viewer, send one sample frame to the board.")
    parser.add_argument("--sample-dir", default=None)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    python = Path(args.python)
    if not python.is_file():
        raise FileNotFoundError(f"python not found: {python}")

    viewer_cmd = [
        str(python),
        str(VIEWER_SCRIPT),
        "--host",
        args.viewer_host,
        "--port",
        str(args.viewer_port),
    ]
    print("[VehiclePC] starting viewer:")
    print(" ".join(viewer_cmd))
    viewer = subprocess.Popen(viewer_cmd, cwd=str(APP_DIR))
    print(f"[VehiclePC] viewer URL: http://{args.viewer_host}:{args.viewer_port}/?live=1")

    if args.sample_once:
        if not args.edge_host:
            raise ValueError("--sample-once requires --edge-host")
        time.sleep(1.0)
        sender_cmd = [
            str(python),
            str(SENDER_SCRIPT),
            "--edge-host",
            args.edge_host,
            "--edge-port",
            str(args.edge_port),
            "--viewer",
            f"http://{args.viewer_host}:{args.viewer_port}",
        ]
        if args.sample_dir:
            sender_cmd += ["--sample-dir", args.sample_dir]
        print("[VehiclePC] sending one sample frame:")
        print(" ".join(sender_cmd))
        subprocess.check_call(sender_cmd, cwd=str(APP_DIR))
        print("[VehiclePC] sample sent; viewer keeps running. Press Ctrl+C to stop.")

    try:
        return viewer.wait()
    except KeyboardInterrupt:
        viewer.terminate()
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
