"""Resolve the chassis serial port, including automatic CH340 detection."""

from __future__ import annotations

from serial.tools import list_ports


CH340_USB_IDS = {
    (0x1A86, 0x7523),
    (0x1A86, 0x5523),
}


def _is_ch340(port: object) -> bool:
    vid = getattr(port, "vid", None)
    pid = getattr(port, "pid", None)
    if (vid, pid) in CH340_USB_IDS:
        return True
    text = " ".join(
        str(getattr(port, field, "") or "")
        for field in ("description", "manufacturer", "hwid")
    ).upper()
    return "CH340" in text or "CH341" in text


def resolve_serial_port(requested: str | None) -> str:
    """Return an explicit COM port or find the single attached CH340 device."""
    value = str(requested or "AUTO").strip()
    if value.upper() != "AUTO":
        return value

    matches = [port for port in list_ports.comports() if _is_ch340(port)]
    if len(matches) == 1:
        return str(matches[0].device)
    if not matches:
        raise RuntimeError(
            "No CH340/CH341 serial device was detected. Check the driver, USB cable, "
            "Device Manager, and chassis power."
        )
    devices = ", ".join(str(port.device) for port in matches)
    raise RuntimeError(
        f"Multiple CH340/CH341 devices were detected ({devices}). "
        "Set SERIAL_PORT in the launcher to the chassis COM port."
    )
