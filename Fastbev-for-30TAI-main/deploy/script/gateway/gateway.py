import argparse
import asyncio
import base64
import json
import socket
import time
from pathlib import Path

import aiohttp

from protocol import (
    ProtocolError, pack_result, recv_packet, send_packet, unpack_batch, unpack_result,
)


EXPECTED_CAMERAS = [
    "CAM_FRONT", "CAM_FRONT_RIGHT", "CAM_BACK_RIGHT",
    "CAM_BACK", "CAM_BACK_LEFT", "CAM_FRONT_LEFT",
]


class EdgeConnection:
    def __init__(self, host, port, timeout):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.sock = None

    def close(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
        self.sock = None

    def _connect(self):
        self.close()
        self.sock = socket.create_connection((self.host, self.port), timeout=self.timeout)
        self.sock.settimeout(self.timeout)
        print(f"[EDGE] connected to {self.host}:{self.port}", flush=True)

    def exchange(self, batch_packet):
        last_error = None
        for attempt in range(2):
            try:
                if self.sock is None:
                    self._connect()
                send_packet(self.sock, batch_packet)
                return recv_packet(self.sock)
            except (OSError, ConnectionError, ProtocolError) as exc:
                last_error = exc
                print(f"[EDGE] exchange failed (attempt {attempt + 1}/2): {exc}", flush=True)
                self.close()
                if attempt == 0:
                    time.sleep(0.5)
        raise ConnectionError(f"edge unavailable: {last_error}")


def validate_batch(packet):
    batch = unpack_batch(packet, verify_crc=True)
    names = [camera["name"] for camera in batch["cameras"]]
    if names != EXPECTED_CAMERAS:
        raise ProtocolError(f"camera order mismatch: {names}")
    for camera in batch["cameras"]:
        jpeg = camera["jpeg"]
        if len(jpeg) < 4 or not jpeg.startswith(b"\xff\xd8") or not jpeg.endswith(b"\xff\xd9"):
            raise ProtocolError(f"invalid JPEG markers: {camera['name']}")
    return batch


def dump_batch(batch, destination):
    destination.mkdir(parents=True, exist_ok=True)
    for camera in batch["cameras"]:
        (destination / f"{camera['name']}.jpg").write_bytes(camera["jpeg"])
    manifest = {
        "frame_id": batch["frame_id"],
        "capture_timestamp_ns": batch["timestamp_ns"],
        "cameras": [
            {"name": item["name"], "bytes": item["size"], "crc32": f"{item['crc32']:08x}"}
            for item in batch["cameras"]
        ],
    }
    (destination / "batch.json").write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False), encoding="utf-8"
    )


async def run_once(args, edge):
    timeout = aiohttp.ClientTimeout(total=None, sock_connect=10, sock_read=None)
    async with aiohttp.ClientSession(timeout=timeout) as session:
        async with session.ws_connect(
            args.server,
            heartbeat=10.0,
            max_msg_size=64 * 1024 * 1024,
            compress=0,
        ) as ws:
            print(f"[SERVER] gateway connected: {args.server}", flush=True)
            last_frame_id = -1
            dumped = False
            while True:
                await ws.send_json({"type": "ready", "last_frame_id": last_frame_id})
                message = await ws.receive()
                if message.type == aiohttp.WSMsgType.BINARY:
                    received_at = time.perf_counter()
                    batch = validate_batch(message.data)
                    frame_id = int(batch["frame_id"])
                    total_mib = len(message.data) / 1048576.0
                    print(
                        f"[RX] frame={frame_id} cameras=6 packet={total_mib:.2f} MiB CRC=OK",
                        flush=True,
                    )
                    if args.dump_first_batch and not dumped:
                        dump_batch(batch, Path(args.dump_first_batch))
                        dumped = True
                        print(f"[DUMP] first batch saved to {args.dump_first_batch}", flush=True)

                    loop = asyncio.get_running_loop()
                    result_packet = await loop.run_in_executor(None, edge.exchange, message.data)
                    result = unpack_result(result_packet)
                    if int(result["frame_id"]) != frame_id:
                        raise ProtocolError(
                            f"result frame mismatch: result={result['frame_id']} batch={frame_id}"
                        )
                    elapsed_ms = (time.perf_counter() - received_at) * 1000.0
                    reserved = {
                        "frame_id", "finished_ts_ns", "inference_ms", "source",
                        "objects", "timestamp_ns", "input_capture_ts_ns",
                    }
                    extra = {
                        key: value for key, value in result.items() if key not in reserved
                    }
                    extra.update({
                        "gateway_roundtrip_ms": round(elapsed_ms, 2),
                        "gateway_received_wall_ns": time.time_ns(),
                        "batch_packet_bytes": len(message.data),
                    })
                    forwarded_packet = pack_result(
                        frame_id=frame_id,
                        finished_ts_ns=result.get("finished_ts_ns", result["timestamp_ns"]),
                        inference_ms=result["inference_ms"],
                        objects=result["objects"],
                        source=result.get("source", "edge"),
                        input_capture_ts_ns=result.get(
                            "input_capture_ts_ns", batch["timestamp_ns"]
                        ),
                        extra=extra,
                    )
                    await ws.send_bytes(forwarded_packet)
                    if args.viewer_push_url:
                        viewer_started = time.perf_counter()
                        viewer_payload = {
                            "frame_id": str(frame_id),
                            "capture_ts_ns": int(batch["timestamp_ns"]),
                            "inference_ms": float(result["inference_ms"]),
                            "gateway_roundtrip_ms": round(elapsed_ms, 2),
                            "source": result.get("source", "edge"),
                            "objects": result["objects"],
                            "images": {
                                camera["name"]: base64.b64encode(camera["jpeg"]).decode("ascii")
                                for camera in batch["cameras"]
                            },
                        }
                        try:
                            async with session.post(
                                args.viewer_push_url,
                                json=viewer_payload,
                                timeout=aiohttp.ClientTimeout(total=args.viewer_timeout),
                            ) as response:
                                response_text = await response.text()
                                if response.status != 200:
                                    raise RuntimeError(
                                        f"HTTP {response.status}: {response_text[:200]}"
                                    )
                            viewer_ms = (time.perf_counter() - viewer_started) * 1000.0
                            print(
                                f"[VIEWER] frame={frame_id} HTTP 200 push={viewer_ms:.1f} ms",
                                flush=True,
                            )
                        except Exception as exc:
                            # The local viewer is optional; never interrupt the
                            # real CARLA-to-board inference chain if it is down.
                            print(f"[VIEWER] frame={frame_id} skipped: {exc}", flush=True)
                    print(
                        f"[TX] frame={frame_id} objects={len(result['objects'])} "
                        f"inference={result['inference_ms']:.1f} ms roundtrip={elapsed_ms:.1f} ms",
                        flush=True,
                    )
                    last_frame_id = frame_id
                elif message.type == aiohttp.WSMsgType.TEXT:
                    document = json.loads(message.data)
                    if document.get("type") == "no_data":
                        await asyncio.sleep(0.02)
                    elif document.get("type") == "error":
                        raise ProtocolError(document.get("message", "server protocol error"))
                elif message.type in (
                    aiohttp.WSMsgType.CLOSE,
                    aiohttp.WSMsgType.CLOSED,
                    aiohttp.WSMsgType.ERROR,
                ):
                    raise ConnectionError(f"server websocket closed: {message.type}")


async def main_async(args):
    edge = EdgeConnection(args.edge_host, args.edge_port, args.edge_timeout)
    try:
        while True:
            try:
                await run_once(args, edge)
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                edge.close()
                print(f"[RECONNECT] {exc}; retry in {args.reconnect_delay:.1f}s", flush=True)
                await asyncio.sleep(args.reconnect_delay)
    finally:
        edge.close()


def main():
    parser = argparse.ArgumentParser(description="CARLA server <-> development-board gateway")
    parser.add_argument("--server", default="ws://10.134.143.120:8080/gateway")
    parser.add_argument("--edge-host", default="127.0.0.1")
    parser.add_argument("--edge-port", default=5200, type=int)
    parser.add_argument("--edge-timeout", default=10.0, type=float)
    parser.add_argument("--reconnect-delay", default=1.0, type=float)
    parser.add_argument("--dump-first-batch", default="")
    parser.add_argument(
        "--viewer-push-url",
        default="http://127.0.0.1:8092/api/push_carla_frame",
        help="optional local interactive Viewer endpoint; pass an empty string to disable",
    )
    parser.add_argument("--viewer-timeout", default=5.0, type=float)
    args = parser.parse_args()
    try:
        asyncio.run(main_async(args))
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
