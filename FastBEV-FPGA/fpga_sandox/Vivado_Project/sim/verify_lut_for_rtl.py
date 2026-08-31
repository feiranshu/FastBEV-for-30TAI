#!/usr/bin/env python3
"""Verify the hardware LUT contract consumed by rtl/lut_engine_fp32.v."""

from __future__ import annotations

import hashlib
import os
import struct
from pathlib import Path


EXPECTED_HASH = "84187065e7fa9b7c2dd7aecf7992770bfa4f21b809696f14f9e0ee610a566383"


def main() -> int:
    root = Path(os.environ["FASTBEV_DATA_ROOT"]).resolve()
    path = root / "part2_lut" / "fastbev_lut_table.bin"
    data = path.read_bytes()
    digest = hashlib.sha256(data).hexdigest()
    if len(data) != 160_000 * 8:
        raise ValueError(f"bad LUT size: {len(data)}")
    if digest != EXPECTED_HASH:
        raise ValueError(f"bad LUT hash: {digest}")

    valid = 0
    for index, (cam, u, v, pad) in enumerate(struct.iter_unpack("<hhhh", data)):
        if pad != 0 or not -1 <= cam < 6:
            raise ValueError(f"bad entry {index}: {(cam, u, v, pad)}")
        if cam < 0:
            if (u, v) != (0, 0):
                raise ValueError(f"bad invalid entry {index}: {(cam, u, v, pad)}")
        else:
            valid += 1
            if not 0 <= u < 160 or not 0 <= v < 120:
                raise ValueError(f"out-of-range entry {index}: {(cam, u, v, pad)}")
    if valid != 156_125:
        raise ValueError(f"bad valid count: {valid}")

    print("LUT_RTL_CONTRACT_PASS")
    print(f"entries=160000 valid={valid} bytes={len(data)}")
    print(f"sha256={digest}")
    print("format=little-endian int16[4]: cam_id,u,v,pad; order=ZYX")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
