#!/usr/bin/env python3
"""Map six semantic BEV camera names to discovered DirectShow sources."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parent
NAMES = (
    "CAM_FRONT",
    "CAM_FRONT_LEFT",
    "CAM_FRONT_RIGHT",
    "CAM_BACK_LEFT",
    "CAM_BACK",
    "CAM_BACK_RIGHT",
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("devices_json", type=Path)
    parser.add_argument("--config", type=Path, default=ROOT / "config" / "cameras.json")
    args = parser.parse_args()
    sources = json.loads(args.devices_json.read_text(encoding="utf-8"))["sources"]
    if len(sources) < 6:
        raise ValueError("At least six external camera sources are required.")
    for index, source in enumerate(sources):
        print(f"[{index}] {source}")
    print("\nUse previews to identify each camera. Exclude the TOP/overhead source.")
    used: set[int] = set()
    chosen: dict[str, int] = {}
    for name in NAMES:
        while True:
            try:
                index = int(input(f"Source index for {name}: ").strip())
            except ValueError:
                print("Enter an integer.")
                continue
            if not 0 <= index < len(sources):
                print("Index is outside the list.")
            elif index in used:
                print("That source is already assigned.")
            else:
                chosen[name] = index
                used.add(index)
                break
    config = json.loads(args.config.read_text(encoding="utf-8"))
    config["cameras"] = [
        {"name": name, "source": sources[chosen[name]], "enabled": True}
        for name in NAMES
    ]
    args.config.write_text(json.dumps(config, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"Saved six-camera mapping: {args.config}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
