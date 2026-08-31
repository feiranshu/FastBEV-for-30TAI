"""PC900 steering wheel and pedal input helpers."""

from __future__ import annotations

import json
import os
import warnings
from dataclasses import dataclass
from pathlib import Path
from typing import Any


os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
warnings.filterwarnings("ignore", message="pkg_resources is deprecated as an API.*")


def read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def activation(value: float, released: float, pressed: float) -> float:
    span = pressed - released
    if abs(span) < 0.15:
        return 0.0
    return max(0.0, min(1.0, (value - released) / span))


def steering_strengths(value: float, spec: dict[str, Any]) -> tuple[float, float]:
    center = float(spec["center"])

    def toward(extreme: float) -> float:
        span = extreme - center
        if abs(span) < 0.15:
            return 0.0
        return max(0.0, min(1.0, (value - center) / span))

    return toward(float(spec["left"])), toward(float(spec["right"]))


@dataclass(frozen=True)
class PC900Sample:
    axes: list[float]
    throttle: float
    brake: float
    left: float
    right: float
    steering: str
    buttons: list[int]
    rising_buttons: list[int]
    throttle_engaged: bool
    high_speed: bool
    brake_active: bool
    neutral_for_arm: bool

    def as_dict(self, linear_mps: float = 0.0, angular_radps: float = 0.0, armed: bool = False) -> dict[str, Any]:
        return {
            "armed": armed,
            "axes": self.axes,
            "throttle": self.throttle,
            "brake": self.brake,
            "steering": self.steering,
            "buttons": self.buttons,
            "linear_mps": linear_mps,
            "angular_radps": angular_radps,
            "throttle_engaged": self.throttle_engaged,
            "high_speed": self.high_speed,
            "brake_active": self.brake_active,
            "neutral_for_arm": self.neutral_for_arm,
        }


class PC900Input:
    def __init__(self, config_path: Path):
        self.config_path = config_path
        self.config = read_json(config_path)
        self.pygame: Any = None
        self.joystick: Any = None
        self.previous_buttons: set[int] = set()
        self.steer_key: str | None = None
        self.throttle_engaged = False

    def open(self) -> None:
        try:
            import pygame  # type: ignore[import-not-found]
        except ModuleNotFoundError as exc:
            raise RuntimeError("pygame is not installed. Run setup.cmd or install pygame>=2.6.1.") from exc

        pygame.init()
        pygame.joystick.init()
        count = pygame.joystick.get_count()
        if count != 1:
            raise RuntimeError(f"必须且只能连接一个游戏控制器，当前检测到 {count} 个。")
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        actual_guid = str(joystick.get_guid()).lower()
        try:
            guid_bytes = bytes.fromhex(actual_guid)
            actual_vid = int.from_bytes(guid_bytes[4:6], "little")
            actual_pid = int.from_bytes(guid_bytes[8:10], "little")
        except (ValueError, IndexError):
            actual_vid = actual_pid = -1
        expected_vid = int(str(self.config["usb_vid"]), 16)
        expected_pid = int(str(self.config["usb_pid"]), 16)
        if (actual_vid, actual_pid) != (expected_vid, expected_pid):
            raise RuntimeError(
                "检测到的唯一游戏控制器不是已标定的 PC900："
                f"期望 VID:PID={expected_vid:04X}:{expected_pid:04X}，"
                f"实际={actual_vid:04X}:{actual_pid:04X}（GUID {actual_guid}）。"
            )
        expected_axes = int(self.config["expected_axis_count"])
        minimum_buttons = int(self.config["minimum_button_count"])
        if joystick.get_numaxes() != expected_axes:
            raise RuntimeError(
                f"PC900 轴数量变化：期望 {expected_axes}，实际 {joystick.get_numaxes()}。"
            )
        if joystick.get_numbuttons() < minimum_buttons:
            raise RuntimeError(
                f"PC900 按钮数量不足：至少 {minimum_buttons}，实际 {joystick.get_numbuttons()}。"
            )
        pygame.event.pump()
        self.pygame = pygame
        self.joystick = joystick

    def close(self) -> None:
        if self.pygame is not None:
            self.pygame.quit()
        self.pygame = None
        self.joystick = None

    def poll(self) -> PC900Sample:
        if self.pygame is None or self.joystick is None:
            raise RuntimeError("PC900 is not open.")
        removed_events: list[Any] = []
        for event in self.pygame.event.get():
            if event.type == self.pygame.JOYDEVICEREMOVED:
                removed_events.append(event)
        if removed_events:
            raise RuntimeError("方向盘输入链路重置；小车已停车并重新锁定。")
        if not self.joystick.get_init():
            raise RuntimeError("方向盘输入设备已失效；小车已停车并重新锁定。")

        try:
            axes = [self.joystick.get_axis(index) for index in range(self.joystick.get_numaxes())]
            buttons = {
                index + 1
                for index in range(self.joystick.get_numbuttons())
                if self.joystick.get_button(index)
            }
        except Exception as exc:
            raise RuntimeError("方向盘输入读取失败；小车已停车并重新锁定。") from exc
        rising = buttons - self.previous_buttons
        self.previous_buttons = buttons

        spec = self.config["axes"]
        steering = spec["steering"]
        throttle_spec = spec["throttle"]
        brake_spec = spec["brake"]
        left, right = steering_strengths(axes[int(steering["index"])], steering)
        throttle = activation(
            axes[int(throttle_spec["index"])],
            float(throttle_spec["released"]),
            float(throttle_spec["pressed"]),
        )
        brake = activation(
            axes[int(brake_spec["index"])],
            float(brake_spec["released"]),
            float(brake_spec["pressed"]),
        )

        if self.steer_key == "LEFT":
            if left < float(steering["release_threshold"]):
                self.steer_key = "RIGHT" if right > float(steering["engage_threshold"]) else None
        elif self.steer_key == "RIGHT":
            if right < float(steering["release_threshold"]):
                self.steer_key = "LEFT" if left > float(steering["engage_threshold"]) else None
        elif left > float(steering["engage_threshold"]):
            self.steer_key = "LEFT"
        elif right > float(steering["engage_threshold"]):
            self.steer_key = "RIGHT"

        brake_active = brake > float(brake_spec["stop_threshold"])
        if brake_active:
            throttle = 0.0
            self.throttle_engaged = False
        elif self.throttle_engaged:
            if throttle < float(throttle_spec["release_threshold"]):
                self.throttle_engaged = False
        elif throttle > float(throttle_spec["engage_threshold"]):
            self.throttle_engaged = True

        high_speed = throttle >= float(throttle_spec["high_speed_threshold"])
        neutral_for_arm = throttle <= 0.08 and brake <= 0.08 and max(left, right) <= 0.15
        return PC900Sample(
            axes=axes,
            throttle=throttle,
            brake=brake,
            left=left,
            right=right,
            steering=self.steer_key or "CENTER",
            buttons=sorted(buttons),
            rising_buttons=sorted(rising),
            throttle_engaged=self.throttle_engaged,
            high_speed=high_speed,
            brake_active=brake_active,
            neutral_for_arm=neutral_for_arm,
        )

    def button(self, name: str) -> int:
        return int(self.config["buttons"][name])

    def poll_hz(self) -> float:
        return max(10.0, float(self.config.get("poll_hz", 50)))

    def watchdog_timeout_s(self) -> float:
        return float(self.config.get("watchdog_timeout_s", 2.0))


def pc900_motion(sample: PC900Sample, chassis_config: dict[str, Any], armed: bool) -> tuple[float, float]:
    if not armed or not sample.throttle_engaged or sample.brake_active:
        return 0.0, 0.0
    linear = float(chassis_config["normal_speed_mps"] if sample.high_speed else chassis_config["initial_speed_mps"])
    turn_rate = float(chassis_config["turn_rate_radps"])
    if sample.steering == "LEFT":
        return linear, turn_rate
    if sample.steering == "RIGHT":
        return linear, -turn_rate
    return linear, 0.0
