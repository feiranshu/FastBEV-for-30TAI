#!/usr/bin/env python3
"""Read PC900 wheel/pedal input without opening cameras, COM ports, or motors."""

from __future__ import annotations

import argparse
import time
from pathlib import Path

from pc900_input import PC900Input


ROOT = Path(__file__).resolve().parent


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pc900-config", type=Path, default=ROOT / "config" / "pc900.json")
    parser.add_argument("--seconds", type=float, default=0.0)
    args = parser.parse_args()

    pc900 = PC900Input(args.pc900_config)
    pc900.open()
    deadline = time.monotonic() + args.seconds if args.seconds > 0 else None
    print("INPUT ONLY: no serial port, no cameras, and no vehicle motion.")
    print("Button 8 toggles PC900 arm in the web dashboard. Button 4 captures in the dashboard.")
    print("Ctrl+C exits.")
    try:
        while deadline is None or time.monotonic() < deadline:
            try:
                sample = pc900.poll()
            except Exception as exc:
                print(f"\n[PC900] input error: {exc}")
                pc900.close()
                while deadline is None or time.monotonic() < deadline:
                    time.sleep(1.0)
                    try:
                        pc900 = PC900Input(args.pc900_config)
                        pc900.open()
                    except Exception as reopen_exc:
                        print(f"[PC900] reconnecting: {reopen_exc}")
                        continue
                    print("[PC900] reconnected")
                    break
                continue
            axes = " ".join(f"a{index}={value:+.3f}" for index, value in enumerate(sample.axes))
            print(
                "\r"
                f"throttle={sample.throttle:.2f} brake={sample.brake:.2f} "
                f"steering={sample.steering:<6} buttons={sample.buttons} "
                f"neutral={sample.neutral_for_arm} {axes}    ",
                end="",
                flush=True,
            )
            time.sleep(1.0 / pc900.poll_hz())
        print()
        return 0
    except KeyboardInterrupt:
        print()
        return 0
    finally:
        pc900.close()


if __name__ == "__main__":
    raise SystemExit(main())
