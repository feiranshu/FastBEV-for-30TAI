#!/usr/bin/env python3
"""Clear EMM42 stall-protection flags after an explicit lifted-wheel confirmation."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from emm42_modbus import Emm42Modbus


ROOT = Path(__file__).resolve().parent


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "config" / "chassis.json")
    parser.add_argument("--serial-port")
    args = parser.parse_args()
    config = json.loads(args.config.read_text(encoding="utf-8"))
    port = args.serial_port or config["serial_port"]
    confirmation = input(
        "Raise all four wheels and keep the power switch within reach. "
        "Type CLEAR to clear stall flags: "
    ).strip()
    if confirmation != "CLEAR":
        print("Cancelled; nothing was written.")
        return 2
    with Emm42Modbus(port, int(config["baudrate"])) as bus:
        for address in config["addresses"]:
            bus.read_driver_parameters(address)
        before = bus.healthcheck(config["addresses"])
        affected = [status.address for status in before if status.motion_fault]
        for address in affected:
            bus.immediate_stop(address)
            time.sleep(0.1)
            bus.clear_stall_protection(address)
            time.sleep(0.2)
        after = bus.healthcheck(config["addresses"])
    print("Before:", {status.address: f"0x{status.raw:04X}" for status in before})
    print("After: ", {status.address: f"0x{status.raw:04X}" for status in after})
    remaining = [status.address for status in after if status.motion_fault]
    if remaining:
        print(f"FAILED: stall flag remains set on motor(s): {remaining}")
        print("Power off the chassis and inspect the affected motor/wheel before retrying.")
        return 3
    print("Stall flags cleared. Keep the wheels raised for the first motion test.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
