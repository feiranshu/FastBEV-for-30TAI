import json
import socket
import struct
import zlib


MAGIC = b"BEV1"
VERSION = 1
MSG_BATCH = 1
MSG_RESULT = 2
MAX_PACKET_BYTES = 64 * 1024 * 1024

HEADER = struct.Struct("!4sHHQQII")
ENTRY = struct.Struct("!16sII")
LENGTH = struct.Struct("!I")


class ProtocolError(RuntimeError):
    pass


def _pack_message(message_type, frame_id, timestamp_ns, item_count, payload):
    if len(payload) > MAX_PACKET_BYTES:
        raise ProtocolError(f"payload too large: {len(payload)}")
    return HEADER.pack(
        MAGIC, VERSION, message_type, int(frame_id), int(timestamp_ns),
        int(item_count), len(payload),
    ) + payload


def unpack_header(packet):
    if len(packet) < HEADER.size:
        raise ProtocolError("packet shorter than header")
    magic, version, message_type, frame_id, timestamp_ns, item_count, payload_size = HEADER.unpack_from(packet)
    if magic != MAGIC:
        raise ProtocolError(f"bad magic: {magic!r}")
    if version != VERSION:
        raise ProtocolError(f"unsupported version: {version}")
    if payload_size != len(packet) - HEADER.size:
        raise ProtocolError(f"payload size mismatch: header={payload_size}, actual={len(packet)-HEADER.size}")
    return {
        "message_type": message_type,
        "frame_id": frame_id,
        "timestamp_ns": timestamp_ns,
        "item_count": item_count,
        "payload_size": payload_size,
    }


def pack_batch(frame_id, capture_ts_ns, camera_order, jpeg_by_camera):
    parts = []
    for camera_name in camera_order:
        jpeg = jpeg_by_camera[camera_name]
        encoded_name = camera_name.encode("ascii")
        if len(encoded_name) > 16:
            raise ProtocolError(f"camera name too long: {camera_name}")
        name_field = encoded_name.ljust(16, b"\0")
        parts.append(ENTRY.pack(name_field, len(jpeg), zlib.crc32(jpeg) & 0xFFFFFFFF))
        parts.append(jpeg)
    return _pack_message(MSG_BATCH, frame_id, capture_ts_ns, len(camera_order), b"".join(parts))


def unpack_batch(packet, verify_crc=True):
    header = unpack_header(packet)
    if header["message_type"] != MSG_BATCH:
        raise ProtocolError(f"expected batch, got type={header['message_type']}")
    payload = memoryview(packet)[HEADER.size:]
    offset = 0
    cameras = []
    for _ in range(header["item_count"]):
        if offset + ENTRY.size > len(payload):
            raise ProtocolError("truncated camera entry")
        name_raw, jpeg_size, expected_crc = ENTRY.unpack_from(payload, offset)
        offset += ENTRY.size
        if offset + jpeg_size > len(payload):
            raise ProtocolError("truncated JPEG payload")
        jpeg = bytes(payload[offset:offset + jpeg_size])
        offset += jpeg_size
        actual_crc = zlib.crc32(jpeg) & 0xFFFFFFFF
        if verify_crc and actual_crc != expected_crc:
            raise ProtocolError(f"CRC mismatch for {name_raw!r}: {actual_crc:08x}!={expected_crc:08x}")
        cameras.append({
            "name": name_raw.rstrip(b"\0").decode("ascii"),
            "jpeg": jpeg,
            "size": jpeg_size,
            "crc32": expected_crc,
        })
    if offset != len(payload):
        raise ProtocolError(f"unexpected trailing bytes: {len(payload)-offset}")
    header["cameras"] = cameras
    return header


def pack_result(
    frame_id, finished_ts_ns, inference_ms, objects, source="mock-edge",
    input_capture_ts_ns=None, extra=None,
):
    document = {
        "frame_id": int(frame_id),
        "finished_ts_ns": int(finished_ts_ns),
        "inference_ms": float(inference_ms),
        "source": str(source),
        "objects": objects,
    }
    if input_capture_ts_ns is not None:
        document["input_capture_ts_ns"] = int(input_capture_ts_ns)
    if extra:
        for key, value in dict(extra).items():
            if key not in document:
                document[key] = value
    payload = json.dumps(document, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
    return _pack_message(MSG_RESULT, frame_id, finished_ts_ns, len(objects), payload)


def unpack_result(packet):
    header = unpack_header(packet)
    if header["message_type"] != MSG_RESULT:
        raise ProtocolError(f"expected result, got type={header['message_type']}")
    document = json.loads(packet[HEADER.size:].decode("utf-8"))
    if int(document["frame_id"]) != int(header["frame_id"]):
        raise ProtocolError("result frame_id differs between envelope and JSON")
    if len(document.get("objects", [])) != header["item_count"]:
        raise ProtocolError("result object count mismatch")
    document["timestamp_ns"] = header["timestamp_ns"]
    return document


def parse_result_txt(text):
    objects = []
    for line_number, line in enumerate(text.splitlines(), 1):
        line = line.strip()
        if not line:
            continue
        fields = line.split()
        if len(fields) != 9:
            raise ProtocolError(f"result line {line_number}: expected 9 fields, got {len(fields)}")
        x, y, z, dx, dy, dz, yaw = map(float, fields[:7])
        class_id = int(fields[7])
        score = float(fields[8])
        objects.append({
            "x": x, "y": y, "z": z,
            "dx": dx, "dy": dy, "dz": dz,
            "yaw": yaw, "class_id": class_id, "score": score,
        })
    return objects


def recv_exact(sock, size):
    chunks = []
    remaining = size
    while remaining:
        chunk = sock.recv(remaining)
        if not chunk:
            raise ConnectionError("socket closed while receiving packet")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def send_packet(sock, packet):
    if len(packet) > MAX_PACKET_BYTES:
        raise ProtocolError(f"packet too large: {len(packet)}")
    sock.sendall(LENGTH.pack(len(packet)) + packet)


def recv_packet(sock):
    packet_size = LENGTH.unpack(recv_exact(sock, LENGTH.size))[0]
    if packet_size < HEADER.size or packet_size > MAX_PACKET_BYTES:
        raise ProtocolError(f"invalid packet size: {packet_size}")
    return recv_exact(sock, packet_size)
