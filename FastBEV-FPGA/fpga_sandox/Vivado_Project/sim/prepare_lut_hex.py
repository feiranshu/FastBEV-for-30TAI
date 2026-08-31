#!/usr/bin/env python3
"""Emit the verified 64-bit-lane LUT image used by the full RTL regression."""

from __future__ import annotations

import hashlib
import os
import struct
from pathlib import Path


EXPECTED_SHA256 = "84187065e7fa9b7c2dd7aecf7992770bfa4f21b809696f14f9e0ee610a566383"


def main() -> int:
    root = Path(os.environ["FASTBEV_DATA_ROOT"]).resolve()
    source = root / "part2_lut" / "fastbev_lut_table.bin"
    target = Path(os.environ["FASTBEV_SIM_BUILD"]).resolve() / "fastbev_lut_table.hex"
    payload = source.read_bytes()
    if len(payload) != 160_000 * 8:
        raise ValueError(f"bad LUT size: {len(payload)}")
    if hashlib.sha256(payload).hexdigest() != EXPECTED_SHA256:
        raise ValueError("LUT SHA256 mismatch")
    target.parent.mkdir(parents=True, exist_ok=True)
    with target.open("w", encoding="ascii", newline="\n") as stream:
        for (word,) in struct.iter_unpack("<Q", payload):
            stream.write(f"{word:016x}\n")
    print(f"LUT_HEX_PASS entries=160000 path={target}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
