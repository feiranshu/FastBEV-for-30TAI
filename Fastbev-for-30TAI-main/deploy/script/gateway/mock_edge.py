import argparse
import itertools
import math
import random
import socket
import time
from pathlib import Path

from protocol import ProtocolError, pack_result, parse_result_txt, recv_packet, send_packet, unpack_batch


EXPECTED_CAMERAS = [
    "CAM_FRONT", "CAM_FRONT_RIGHT", "CAM_BACK_RIGHT",
    "CAM_BACK", "CAM_BACK_LEFT", "CAM_FRONT_LEFT",
]
ROOT = Path(__file__).resolve().parent


def load_results(paths):
    result_sets = []
    for path in paths:
        objects = parse_result_txt(path.read_text(encoding="utf-8"))
        result_sets.append(objects)
        print(f"[LOAD] {path.name}: {len(objects)} objects", flush=True)
    return result_sets


def serve_connection(client, peer, result_cycle, inference_ms, jitter_ms):
    print(f"[CLIENT] connected: {peer[0]}:{peer[1]}", flush=True)
    with client:
        client.settimeout(30.0)
        while True:
            packet = recv_packet(client)
            batch = unpack_batch(packet, verify_crc=True)
            names = [item["name"] for item in batch["cameras"]]
            if names != EXPECTED_CAMERAS:
                raise ProtocolError(f"camera order mismatch: {names}")
            frame_id = int(batch["frame_id"])
            print(
                f"[INFER] frame={frame_id} cameras={len(names)} packet={len(packet)/1048576:.2f} MiB",
                flush=True,
            )
            started = time.perf_counter()
            requested_ms = max(0.0, inference_ms + random.uniform(-jitter_ms, jitter_ms))
            time.sleep(requested_ms / 1000.0)
            objects = next(result_cycle)
            actual_ms = (time.perf_counter() - started) * 1000.0
            result = pack_result(
                frame_id=frame_id,
                finished_ts_ns=time.time_ns(),
                inference_ms=actual_ms,
                objects=objects,
                source="mock-edge",
                input_capture_ts_ns=batch["timestamp_ns"],
                extra={"requested_mock_ms": round(requested_ms, 2)},
            )
            send_packet(client, result)
            print(f"[RESULT] frame={frame_id} objects={len(objects)} inference={actual_ms:.1f} ms", flush=True)


def main():
    parser = argparse.ArgumentParser(description="Mock 2-FPS Fast-BEV development board")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", default=5200, type=int)
    parser.add_argument("--inference-ms", default=500.0, type=float)
    parser.add_argument("--jitter-ms", default=120.0, type=float)
    parser.add_argument("--synthetic-boxes", action="store_true")
    parser.add_argument(
        "--results", nargs="+", type=Path,
        default=[ROOT / "result_0001.txt", ROOT / "result_0002.txt"],
    )
    args = parser.parse_args()
    if args.synthetic_boxes:
        result_sets = [[
            {"x": 0.0, "y": 13.1, "z": -1.60, "dx": 4.4, "dy": 1.8,
             "dz": 1.6, "yaw": -math.pi / 2, "class_id": 0, "score": 0.96},
            {"x": -4.0, "y": 23.1, "z": -1.55, "dx": 4.8, "dy": 2.0,
             "dz": 1.9, "yaw": -math.pi / 2, "class_id": 1, "score": 0.91},
            {"x": 3.0, "y": 9.1, "z": -1.60, "dx": 0.8, "dy": 0.8,
             "dz": 1.75, "yaw": -math.pi / 2, "class_id": 7, "score": 0.88},
        ]]
        print("[LOAD] synthetic calibration boxes: 3 objects", flush=True)
    else:
        result_sets = load_results(args.results)
    result_cycle = itertools.cycle(result_sets)

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((args.host, args.port))
        server.listen(4)
        print(f"[LISTEN] mock edge on {args.host}:{args.port}", flush=True)
        while True:
            client, peer = server.accept()
            try:
                serve_connection(
                    client, peer, result_cycle, args.inference_ms, args.jitter_ms
                )
            except (OSError, ConnectionError, ProtocolError) as exc:
                print(f"[CLIENT] disconnected: {exc}", flush=True)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped.")
