#!/usr/bin/env python3
"""Start the PC interactive Viewer for board-pushed FastBEV results."""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


APP_DIR = Path(__file__).resolve().parents[1]
DEPLOY_DIR = APP_DIR / "deploy"
VIEWER = DEPLOY_DIR / "script" / "viewer" / "pc_ps_live_pipeline_server.py"


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Start the PC Viewer for board result pushes."
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8092)
    parser.add_argument("--mode", choices=["loop", "push", "carla"], default="push")
    parser.add_argument("--root", default=str(DEPLOY_DIR / "io" / "output"))
    parser.add_argument("--deploy-root", default=str(DEPLOY_DIR))
    parser.add_argument("--loop-frames", default="0001,0002,0003,0004,0005")
    parser.add_argument("--live-period-ms", type=int, default=100)
    args = parser.parse_args()

    if not VIEWER.is_file():
        parser.error(f"viewer script not found: {VIEWER}")

    cmd = [
        sys.executable,
        str(VIEWER),
        "--host",
        args.host,
        "--port",
        str(args.port),
        "--mode",
        args.mode,
        "--root",
        args.root,
        "--deploy-root",
        args.deploy_root,
        "--loop-frames",
        args.loop_frames,
        "--live-period-ms",
        str(args.live_period_ms),
    ]

    print("[start_board_viewer] " + " ".join(cmd), flush=True)
    print(f"[start_board_viewer] open http://{args.host}:{args.port}/?live=1", flush=True)
    return subprocess.run(cmd, cwd=str(APP_DIR)).returncode


if __name__ == "__main__":
    raise SystemExit(main())
