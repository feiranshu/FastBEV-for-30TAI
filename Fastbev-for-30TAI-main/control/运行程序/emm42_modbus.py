"""Minimal Modbus-RTU transport for the EMM42 V5.0 motor driver.

Only the documented commands required by this project are implemented:
read status, speed mode, synchronized trigger, and immediate stop.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Iterable

import serial


class ModbusError(RuntimeError):
    """A driver did not return a valid Modbus-RTU response."""


def crc16_modbus(payload: bytes) -> bytes:
    """Return Modbus CRC16 in RTU wire order: low byte followed by high byte."""
    crc = 0xFFFF
    for byte in payload:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc.to_bytes(2, "little")


def frame(payload: bytes) -> bytes:
    return payload + crc16_modbus(payload)


def _u16(value: int) -> bytes:
    if not 0 <= value <= 0xFFFF:
        raise ValueError(f"16-bit value out of range: {value}")
    return value.to_bytes(2, "big")


@dataclass(frozen=True)
class MotorStatus:
    address: int
    raw: int

    @property
    def enabled(self) -> bool:
        return bool(self.raw & 0x0001)

    @property
    def reached(self) -> bool:
        return bool(self.raw & 0x0002)

    @property
    def stalled(self) -> bool:
        # EMM V5.0 Rev1.3 section 6.3.4: stall flag = status & 0x04.
        return bool(self.raw & 0x0004)

    @property
    def stall_protection_triggered(self) -> bool:
        # EMM V5.0 Rev1.3 section 6.3.4: protection flag = status & 0x08.
        return bool(self.raw & 0x0008)

    @property
    def motion_fault(self) -> bool:
        return self.stalled or self.stall_protection_triggered


@dataclass(frozen=True)
class DriverParameters:
    address: int
    serial_port_mode: int
    baudrate_code: int
    checksum_mode: int
    control_reply_mode: int
    stall_protection: int


class Emm42Modbus:
    """Single RS485 bus that controls one or more EMM42 drivers."""

    def __init__(self, port: str, baudrate: int = 115200, timeout_s: float = 0.20):
        self.port = port
        self.timeout_s = timeout_s
        self.control_reply_modes: dict[int, int] = {}
        self.serial = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout_s,
            write_timeout=timeout_s,
        )

    def close(self) -> None:
        if self.serial.is_open:
            self.serial.close()

    def __enter__(self) -> "Emm42Modbus":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()

    def _request(self, request: bytes, response_length: int) -> bytes:
        self.serial.reset_input_buffer()
        self.serial.write(request)
        self.serial.flush()
        response = self.serial.read(response_length)
        if len(response) != response_length:
            raise ModbusError(
                f"{self.port}: request {request.hex(' ')} expected {response_length} response bytes, "
                f"got {len(response)} ({response.hex(' ') or 'no response'})"
            )
        if crc16_modbus(response[:-2]) != response[-2:]:
            raise ModbusError(f"{self.port}: response CRC mismatch: {response.hex(' ')}")
        return response

    def _send_without_response(self, request: bytes) -> None:
        self.serial.reset_input_buffer()
        self.serial.write(request)
        self.serial.flush()
        # Keep a conservative RTU bus gap before the next request.
        time.sleep(0.010)

    def write_registers(self, address: int, start: int, values: Iterable[int]) -> None:
        registers = list(values)
        if not registers:
            raise ValueError("at least one register is required")
        payload = bytes((address, 0x10)) + _u16(start) + _u16(len(registers))
        payload += bytes((len(registers) * 2,)) + b"".join(_u16(v) for v in registers)
        request = frame(payload)
        # EMM42 menu item "Control command reply": 0=None, 1=Receive,
        # 2=Reached, 3=Both, 4=Other. Velocity/stop commands acknowledge
        # receipt only in modes 1, 3, and 4. Read commands always reply.
        reply_mode = self.control_reply_modes.get(address, 1)
        if reply_mode not in (1, 3, 4):
            self._send_without_response(request)
            return
        response = self._request(request, 8)
        expected_prefix = bytes((address, 0x10)) + _u16(start)
        returned_count = int.from_bytes(response[4:6], "big")
        # The vendor's Rev. 2023 Modbus manual documents 0x0002 in the F6
        # response even though the request contains three registers. Accept
        # both that documented value and the standard Modbus echo (0x0003).
        valid_count = returned_count == len(registers) or (start == 0x00F6 and returned_count == 2)
        if response[:4] != expected_prefix or not valid_count:
            raise ModbusError(f"unexpected write response: {response.hex(' ')}")

    def write_registers_broadcast(self, start: int, values: Iterable[int]) -> None:
        """Write to Modbus address 0. Broadcast requests do not have a response."""
        registers = list(values)
        payload = bytes((0, 0x10)) + _u16(start) + _u16(len(registers))
        payload += bytes((len(registers) * 2,)) + b"".join(_u16(v) for v in registers)
        self.serial.reset_input_buffer()
        self.serial.write(frame(payload))
        self.serial.flush()

    def write_single_register(self, address: int, register: int, value: int) -> None:
        payload = bytes((address, 0x06)) + _u16(register) + _u16(value)
        request = frame(payload)
        reply_mode = self.control_reply_modes.get(address, 1)
        if reply_mode not in (1, 3, 4):
            self._send_without_response(request)
            return
        response = self._request(request, 8)
        if response != request:
            raise ModbusError(f"unexpected single-register response: {response.hex(' ')}")

    def read_input_registers(self, address: int, start: int, count: int) -> list[int]:
        payload = bytes((address, 0x04)) + _u16(start) + _u16(count)
        response = self._request(frame(payload), 5 + count * 2)
        if response[0] != address or response[1] != 0x04 or response[2] != count * 2:
            raise ModbusError(f"unexpected read response: {response.hex(' ')}")
        return [int.from_bytes(response[3 + i * 2 : 5 + i * 2], "big") for i in range(count)]

    def read_status(self, address: int) -> MotorStatus:
        return MotorStatus(address=address, raw=self.read_input_registers(address, 0x003A, 1)[0])

    def read_driver_parameters(self, address: int) -> DriverParameters:
        registers = self.read_input_registers(address, 0x0042, 15)
        parameters = DriverParameters(
            address=address,
            serial_port_mode=registers[2] >> 8,
            baudrate_code=registers[8] >> 8,
            checksum_mode=registers[9] & 0xFF,
            control_reply_mode=registers[10] >> 8,
            stall_protection=registers[10] & 0xFF,
        )
        self.control_reply_modes[address] = parameters.control_reply_mode
        return parameters

    def healthcheck(self, addresses: Iterable[int]) -> list[MotorStatus]:
        return [self.read_status(address) for address in addresses]

    def stage_speed(self, address: int, direction: int, rpm: int, acceleration: int) -> None:
        if direction not in (0, 1):
            raise ValueError("direction must be 0 (CW) or 1 (CCW)")
        if not 0 <= rpm <= 3000:
            raise ValueError("speed must be between 0 and 3000 RPM")
        if not 0 <= acceleration <= 255:
            raise ValueError("acceleration must be between 0 and 255")
        self.write_registers(
            address,
            0x00F6,
            [(direction << 8) | acceleration, rpm, 0x0100],
        )

    def trigger_staged_motion(self, settle_s: float = 0.2) -> None:
        # Address 0 is a documented broadcast address. It begins all staged motors together.
        self.write_registers_broadcast(0x00FF, [0x6600])
        time.sleep(settle_s)

    def immediate_stop(self, address: int) -> None:
        self.write_registers(address, 0x00FE, [0x9800])

    def clear_stall_protection(self, address: int) -> None:
        self.write_single_register(address, 0x000E, 0x0001)

    def immediate_stop_all(self, addresses: Iterable[int]) -> None:
        errors: list[Exception] = []
        for address in addresses:
            try:
                self.immediate_stop(address)
            except Exception as exc:  # Safety action: continue stopping remaining motors.
                errors.append(exc)
        if errors:
            raise ModbusError("; ".join(str(item) for item in errors))
