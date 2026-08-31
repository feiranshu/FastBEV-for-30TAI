#!/usr/bin/env python3
"""Read-only connection check for the four EMM42 drivers."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from emm42_modbus import Emm42Modbus


ROOT = Path(__file__).resolve().parent


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "config" / "chassis.json")
    parser.add_argument("--serial-port", help="Overrides config serial_port, e.g. COM3")
    args = parser.parse_args()
    config = json.loads(args.config.read_text(encoding="utf-8"))
    port = args.serial_port or config["serial_port"]
    with Emm42Modbus(port, int(config["baudrate"])) as bus:
        statuses = bus.healthcheck(config["addresses"])
        parameters = [bus.read_driver_parameters(address) for address in config["addresses"]]
    print("Connected to", port)
    replies = {0: "None", 1: "Receive", 2: "Reached", 3: "Both", 4: "Other"}
    for status, params in zip(statuses, parameters):
        print(
            f"  motor {status.address}: status=0x{status.raw:04X}, "
            f"enabled={status.enabled}, reached={status.reached}, "
            f"stall={status.stalled}, stall_protection={status.stall_protection_triggered}, "
            f"serial_mode={params.serial_port_mode}, baud_code={params.baudrate_code}, "
            f"checksum={params.checksum_mode}, reply={replies.get(params.control_reply_mode, params.control_reply_mode)}"
        )
        if params.serial_port_mode != 2 or params.baudrate_code != 5 or params.checksum_mode != 3:
            raise RuntimeError(f"motor {status.address} has incompatible communication parameters")
    disabled = [str(status.address) for status in statuses if not status.enabled]
    faulted = [str(status.address) for status in statuses if status.motion_fault]
    if disabled or faulted:
        if disabled:
            print(f"BLOCKED: motor(s) {', '.join(disabled)} are not enabled.")
        if faulted:
            print(f"BLOCKED: motor(s) {', '.join(faulted)} report a stall/protection fault.")
            print("Raise all wheels, run clear_stall_flags.cmd, then run this check again.")
        print("Read-only check completed. No motion command was sent.")
        return 2
    print("Read-only check passed. No motion command was sent.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
