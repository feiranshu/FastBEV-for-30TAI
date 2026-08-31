#!/usr/bin/env python3
"""Start the PC-side CARLA Viewer plus Gateway data flow."""

from __future__ import annotations

import argparse
import subprocess
import sys
import time
from pathlib import Path


APP_DIR = Path(__file__).resolve().parents[1]
DEPLOY_DIR = APP_DIR / "deploy"
VIEWER = DEPLOY_DIR / "script" / "viewer" / "pc_ps_live_pipeline_server.py"
GATEWAY = DEPLOY_DIR / "script" / "gateway" / "gateway.py"


def terminate(process: subprocess.Popen[object], name: str) -> None:
    if process.poll() is not None:
        return
    print(f"[start_carla_pc_flow] stopping {name} pid={process.pid}", flush=True)
    process.terminate()
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=5)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Start local CARLA Viewer mode and the PC Gateway."
    )
    parser.add_argument("--server", default="ws://10.134.143.120:8080/gateway")
    parser.add_argument("--edge-host", default="192.168.125.166")
    parser.add_argument("--edge-port", type=int, default=5200)
    parser.add_argument("--edge-timeout", type=float, default=10.0)
    parser.add_argument("--reconnect-delay", type=float, default=1.0)
    parser.add_argument("--viewer-host", default="127.0.0.1")
    parser.add_argument("--viewer-port", type=int, default=8092)
    parser.add_argument("--viewer-timeout", type=float, default=5.0)
    parser.add_argument("--dump-first-batch", default="")
    parser.add_argument(
        "--no-viewer",
        action="store_true",
        help="do not start the local Viewer; Gateway still pushes to the configured URL",
    )
    args = parser.parse_args()

    if not VIEWER.is_file():
        parser.error(f"viewer script not found: {VIEWER}")
    if not GATEWAY.is_file():
        parser.error(f"gateway script not found: {GATEWAY}")

    viewer_url = f"http://{args.viewer_host}:{args.viewer_port}/api/push_carla_frame"
    processes: list[tuple[str, subprocess.Popen[object]]] = []

    try:
        if not args.no_viewer:
            viewer_cmd = [
                sys.executable,
                str(VIEWER),
                "--host",
                args.viewer_host,
                "--port",
                str(args.viewer_port),
                "--mode",
                "carla",
            ]
            print("[start_carla_pc_flow] viewer: " + " ".join(viewer_cmd), flush=True)
            viewer_proc = subprocess.Popen(viewer_cmd, cwd=str(APP_DIR))
            processes.append(("viewer", viewer_proc))
            time.sleep(1.0)
            if viewer_proc.poll() is not None:
                return viewer_proc.returncode or 1
            print(
                f"[start_carla_pc_flow] open http://{args.viewer_host}:{args.viewer_port}/?live=1",
                flush=True,
            )

        gateway_cmd = [
            sys.executable,
            str(GATEWAY),
            "--server",
            args.server,
            "--edge-host",
            args.edge_host,
            "--edge-port",
            str(args.edge_port),
            "--edge-timeout",
            str(args.edge_timeout),
            "--reconnect-delay",
            str(args.reconnect_delay),
            "--viewer-push-url",
            viewer_url,
            "--viewer-timeout",
            str(args.viewer_timeout),
        ]
        if args.dump_first_batch:
            gateway_cmd.extend(["--dump-first-batch", args.dump_first_batch])

        print("[start_carla_pc_flow] gateway: " + " ".join(gateway_cmd), flush=True)
        gateway_proc = subprocess.Popen(gateway_cmd, cwd=str(APP_DIR))
        processes.append(("gateway", gateway_proc))
        return gateway_proc.wait()
    except KeyboardInterrupt:
        print("\n[start_carla_pc_flow] interrupted", flush=True)
        return 130
    finally:
        for name, process in reversed(processes):
            terminate(process, name)


if __name__ == "__main__":
    raise SystemExit(main())
