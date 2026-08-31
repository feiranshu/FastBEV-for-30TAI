#!/usr/bin/env python3
"""Verify the compiled Part1 -> FPGA -> Part3/Conv256 interface contract."""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path


def storage_dtype(dtype: dict) -> str:
    value = dtype["element_dtype"]
    return value if isinstance(value, str) else value["storage_dtype"]


def check_raw(root: Path, stem: str) -> dict:
    metadata = json.loads((root / f"{stem}.json").read_text(encoding="utf-8"))
    raw = root / f"{stem}.raw"
    if raw.stat().st_size != metadata["params_bytes"]:
        raise ValueError(f"{raw}: byte count does not match JSON")
    digest = hashlib.md5(raw.read_bytes()).hexdigest()
    if digest != metadata["params_md5"]:
        raise ValueError(f"{raw}: MD5 does not match JSON")
    return metadata


def read_varint(data: bytes, pos: int) -> tuple[int, int]:
    value = shift = 0
    while True:
        byte = data[pos]
        pos += 1
        value |= (byte & 0x7F) << shift
        if byte < 0x80:
            return value, pos
        shift += 7


def proto_fields(data: bytes):
    pos = 0
    while pos < len(data):
        key, pos = read_varint(data, pos)
        number, wire = key >> 3, key & 7
        if wire == 0:
            value, pos = read_varint(data, pos)
        elif wire == 1:
            value, pos = data[pos : pos + 8], pos + 8
        elif wire == 2:
            size, pos = read_varint(data, pos)
            value, pos = data[pos : pos + size], pos + size
        elif wire == 5:
            value, pos = data[pos : pos + 4], pos + 4
        else:
            raise ValueError(f"unsupported protobuf wire type {wire}")
        yield number, wire, value


def attribute_ints(data: bytes) -> tuple[str, list[int]]:
    name = ""
    values: list[int] = []
    for number, wire, value in proto_fields(data):
        if number == 1:
            name = value.decode()
        elif number == 8:
            if wire == 0:
                values.append(value)
            else:
                pos = 0
                while pos < len(value):
                    item, pos = read_varint(value, pos)
                    values.append(item)
    return name, values


def first_onnx_nodes(path: Path) -> list[tuple[str, dict[str, list[int]]]]:
    model = path.read_bytes()
    graph = next(value for number, _, value in proto_fields(model) if number == 7)
    result = []
    for number, _, node in proto_fields(graph):
        if number != 1:
            continue
        op_type = ""
        attrs: dict[str, list[int]] = {}
        for field, _, value in proto_fields(node):
            if field == 4:
                op_type = value.decode()
            elif field == 5:
                name, values = attribute_ints(value)
                attrs[name] = values
        result.append((op_type, attrs))
        if len(result) == 3:
            break
    return result


def main() -> int:
    root = Path(os.environ["FASTBEV_DATA_ROOT"]).resolve()
    p1 = check_raw(root / "part1", "fastbev_part1_vehicle_fp16_ZG")
    p3 = check_raw(root / "part3", "fastbev_part3_vehicle_fp16_ZG")

    p1_output = next(
        value
        for op in p1["ops"]
        if op["_type_key"].endswith("Output")
        for value in op["inputs"]
    )
    p1_type = p1_output["dtype"]
    if p1_type["shape"] != [6, 120, 160, 64]:
        raise ValueError(f"bad Part1 output shape: {p1_type['shape']}")
    if p1_type["layout"] != "@layout(NHWC)" or storage_dtype(p1_type) != "@fp(32)":
        raise ValueError(f"bad Part1 output type/layout: {p1_type}")

    reshape = next(op for op in p3["ops"] if op.get("op_id") == 254)
    conv = next(op for op in p3["ops"] if op.get("op_id") == 256)
    target = reshape["outputs"][0]
    target_type = target["dtype"]
    if target["v_id"] != 362 or conv["inputs"][0]["v_id"] != 362:
        raise ValueError("Part3 Conv256 is not connected to value 362")
    if target_type["shape"] != [1, 16, 200, 200, 16]:
        raise ValueError(f"bad Conv256 input shape: {target_type['shape']}")
    if target_type["layout"] != "@layout(*C**c16)" or storage_dtype(target_type) != "@fp(16)":
        raise ValueError(f"bad Conv256 input type/layout: {target_type}")

    nodes = first_onnx_nodes(root / "part3" / "fastbev_part3.onnx")
    if nodes[0] != ("Transpose", {"perm": [0, 2, 3, 4, 1]}):
        raise ValueError(f"unexpected Part3 first transpose: {nodes[0]}")
    if nodes[1][0] != "Reshape":
        raise ValueError(f"unexpected Part3 second node: {nodes[1]}")
    if nodes[2] != ("Transpose", {"perm": [0, 3, 1, 2]}):
        raise ValueError(f"unexpected Part3 second transpose: {nodes[2]}")

    output_bytes = 1 * 16 * 200 * 200 * 16 * 2
    print("NETWORK_INTERFACE_PASS")
    print("part1_output=FP32 NHWC [6,120,160,64], bytes=29491200")
    print("part3_conv256_input=value362 FP16 [1,16,200,200,16], bytes=20480000")
    print("mapping=global_channel=z*64+c; global_cblk16=z*4+cblk16")
    print(f"raw_md5_part1={p1['params_md5']} raw_md5_part3={p3['params_md5']}")
    if output_bytes != 20_480_000:
        raise AssertionError(output_bytes)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
