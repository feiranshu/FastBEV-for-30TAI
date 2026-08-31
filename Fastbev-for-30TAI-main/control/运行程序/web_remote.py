#!/usr/bin/env python3
"""Local web dashboard for EMM42 driving and six-view snapshots."""

from __future__ import annotations

import argparse
import json
import mimetypes
import shutil
import sys
import threading
import time
import webbrowser
from datetime import datetime
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import quote, unquote, urlparse

from pc900_input import PC900Input, PC900Sample, pc900_motion
from run_bev_capture import Chassis, SixCameraSnapshotRig, load_specs, now_iso, read_json
from serial_port import resolve_serial_port


ROOT = Path(__file__).resolve().parent
APP_ROOT = ROOT.parents[3]
WEB_ROOT = ROOT / "web"
CAMERA_NAMES = (
    "CAM_FRONT",
    "CAM_FRONT_LEFT",
    "CAM_FRONT_RIGHT",
    "CAM_BACK_LEFT",
    "CAM_BACK",
    "CAM_BACK_RIGHT",
)
DRIVE_KEYS = frozenset(("W", "A", "S", "D"))


class DashboardApp:
    def __init__(
        self,
        chassis_path: Path,
        camera_path: Path,
        output_root: Path,
        serial_port: str | None,
        watchdog_s: float,
        ui_only: bool,
        pc900_path: Path,
        pc900_enabled: bool,
        vehicle_live_enabled: bool,
        vehicle_edge_host: str,
        vehicle_edge_port: int,
        vehicle_viewer: str,
        vehicle_ffmpeg: str | None,
        vehicle_interval_ms: int,
    ):
        self.chassis_path = chassis_path
        self.camera_path = camera_path
        self.output_root = output_root
        self.watchdog_s = watchdog_s
        self.ui_only = ui_only
        self.pc900_path = pc900_path
        self.pc900_enabled = pc900_enabled
        self.pc900_config = read_json(pc900_path)
        self.pc900_reconnect_sleep_s = max(
            0.05, float(self.pc900_config.get("reconnect_interval_s", 0.2))
        )
        self.vehicle_live_enabled = vehicle_live_enabled
        self.vehicle_edge_host = vehicle_edge_host
        self.vehicle_edge_port = vehicle_edge_port
        self.vehicle_viewer = vehicle_viewer
        self.vehicle_ffmpeg = vehicle_ffmpeg
        self.vehicle_interval_ms = max(0, vehicle_interval_ms)
        self.chassis_config = read_json(chassis_path)
        self.camera_config = read_json(camera_path)
        if serial_port:
            self.chassis_config["serial_port"] = serial_port
        self.specs = load_specs(self.camera_config, validate_sources=not ui_only)
        if tuple(spec.name for spec in self.specs) != CAMERA_NAMES:
            raise ValueError("The camera config must use the frozen six semantic camera names/order.")

        self.state_lock = threading.RLock()
        self.motor_lock = threading.Lock()
        self.capture_lock = threading.Lock()
        self.event_lock = threading.Lock()
        self.running = threading.Event()
        self.ready = False
        self.armed = False
        self.control_source = "web"
        self.desired_keys: set[str] = set()
        self.last_heartbeat = 0.0
        self.pc900_last_input = 0.0
        self.last_applied: tuple[float, float] | None = None
        self.frame_number = 0
        self.latest_capture: dict[str, Any] | None = None
        self.error: str | None = None
        self.stop_reason = "等待系统初始化"

        self.chassis: Chassis | None = None
        self.rig: SixCameraSnapshotRig | None = None
        self.pc900: PC900Input | None = None
        self.control_thread: threading.Thread | None = None
        self.pc900_thread: threading.Thread | None = None
        self.vehicle_live_thread: threading.Thread | None = None
        self.events: Any = None
        self.session_dir: Path | None = None
        self.session_file: Path | None = None
        self.session_data: dict[str, Any] = {}
        self.motor_statuses: list[int] = []
        self.pc900_present = False
        self.pc900_error: str | None = None
        self.pc900_restore_pending = False
        self.pc900_state: dict[str, Any] = {
            "armed": False,
            "throttle": 0.0,
            "brake": 0.0,
            "steering": "CENTER",
            "buttons": [],
            "linear_mps": 0.0,
            "angular_radps": 0.0,
        }
        self.vehicle_live_state: dict[str, Any] = {
            "enabled": self.vehicle_live_enabled,
            "edge": f"{self.vehicle_edge_host}:{self.vehicle_edge_port}",
            "viewer": self.vehicle_viewer,
            "frames_sent": 0,
            "last_frame_id": None,
            "last_objects": None,
            "last_inference_ms": None,
            "last_roundtrip_ms": None,
            "last_sync_spread_ms": None,
            "error": None,
        }
        self.vehicle_live_last_sent_ns = 0

    def start(self) -> None:
        if self.ui_only:
            with self.state_lock:
                self.ready = True
                self.stop_reason = "界面预览模式：不会连接小车或摄像头"
            self.running.set()
            if self.pc900_enabled:
                self._start_pc900(optional=True)
            return

        self.chassis = Chassis(self.chassis_config, True)
        try:
            self.motor_statuses = self.chassis.open()
            self.session_dir = self.output_root / f"session_{datetime.now():%Y%m%d_%H%M%S}"
            self.session_dir.mkdir(parents=True, exist_ok=False)
            snapshot_dir = self.session_dir / "config_snapshot"
            snapshot_dir.mkdir()
            shutil.copy2(self.camera_path, snapshot_dir / "cameras.json")
            shutil.copy2(self.chassis_path, snapshot_dir / "chassis.json")
            self.rig = SixCameraSnapshotRig(self.session_dir, self.specs, self.camera_config)
            self.rig.start()
            self.events = (self.session_dir / "control_events.jsonl").open(
                "w", encoding="utf-8", buffering=1
            )
            self.session_data = {
                "started_at": now_iso(),
                "mode": "web_remote_six_view_snapshot",
                "serial_port": self.chassis_config["serial_port"],
                "motor_status_registers": self.motor_statuses,
                "camera_names": list(CAMERA_NAMES),
                "capture_while_moving": True,
                "camera_capture_profile": self.rig.capture_profile(),
                "pc900_enabled": self.pc900_enabled,
                "watchdog_timeout_s": self.watchdog_s,
            }
            self.session_file = self.session_dir / "session.json"
            self._write_session()
            with self.state_lock:
                self.ready = True
                self.stop_reason = "系统就绪，请点击开始运行"
            self.running.set()
            if self.pc900_enabled:
                self._start_pc900(optional=True)
            self.control_thread = threading.Thread(
                target=self._control_loop, name="chassis-watchdog", daemon=True
            )
            self.control_thread.start()
            if self.vehicle_live_enabled:
                self._start_vehicle_live()
        except Exception:
            if self.rig is not None:
                self.rig.stop()
            if self.chassis is not None:
                self.chassis.close()
            raise

    def _start_vehicle_live(self) -> None:
        if self.ui_only or self.rig is None:
            return
        self.vehicle_live_thread = threading.Thread(
            target=self._vehicle_live_loop, name="vehicle-live", daemon=True
        )
        self.vehicle_live_thread.start()

    def _vehicle_sender(self):
        vehicle_script_dir = APP_ROOT / "deploy" / "script" / "vehicle"
        if str(vehicle_script_dir) not in sys.path:
            sys.path.insert(0, str(vehicle_script_dir))
        from run_bev_capture_vehicle_live import DEFAULT_FFMPEG, send_vehicle_frame

        return send_vehicle_frame, DEFAULT_FFMPEG

    def _select_vehicle_live_frames(self) -> tuple[dict[str, bytes], float] | None:
        assert self.rig is not None
        with self.rig.condition:
            if self.rig.errors:
                detail = "; ".join(f"{name}: {error}" for name, error in self.rig.errors.items())
                raise RuntimeError(f"Camera stream failed: {detail}")
            candidates = {spec.name: list(self.rig.buffers[spec.name]) for spec in self.rig.specs}
            result = self.rig._select_time_paired_frames(candidates, time.monotonic_ns())
        if result is None:
            return None
        frames, spread_ns, _center_ns = result
        newest_ns = max(values[2] for values in frames.values())
        if newest_ns <= self.vehicle_live_last_sent_ns:
            return None
        self.vehicle_live_last_sent_ns = newest_ns
        return {name: values[0] for name, values in frames.items()}, spread_ns / 1_000_000

    def _vehicle_live_loop(self) -> None:
        try:
            send_vehicle_frame, default_ffmpeg = self._vehicle_sender()
        except Exception as exc:
            with self.state_lock:
                self.vehicle_live_state["error"] = f"Vehicle sender import failed: {exc}"
            return

        frame_id = 1
        idle_sleep_s = 0.02
        while self.running.is_set():
            try:
                selected = self._select_vehicle_live_frames()
                if selected is None:
                    time.sleep(idle_sleep_s)
                    continue
                preview_images, spread_ms = selected
                started = time.perf_counter()
                result = send_vehicle_frame(
                    self.vehicle_edge_host,
                    self.vehicle_edge_port,
                    self.vehicle_viewer,
                    frame_id,
                    preview_images,
                    self.vehicle_ffmpeg or default_ffmpeg,
                )
                roundtrip_ms = (time.perf_counter() - started) * 1000.0
                objects = result.get("objects") or []
                with self.state_lock:
                    self.vehicle_live_state.update(
                        {
                            "enabled": True,
                            "frames_sent": self.vehicle_live_state["frames_sent"] + 1,
                            "last_frame_id": f"{frame_id:04d}",
                            "last_objects": len(objects),
                            "last_inference_ms": result.get("inference_ms"),
                            "last_roundtrip_ms": roundtrip_ms,
                            "last_sync_spread_ms": spread_ms,
                            "error": None,
                        }
                    )
                self._log(
                    {
                        "event": "vehicle_live_frame",
                        "frame_id": frame_id,
                        "objects": len(objects),
                        "roundtrip_ms": roundtrip_ms,
                        "sync_spread_ms": spread_ms,
                    }
                )
                print(
                    f"[VehicleLive] frame={frame_id:04d} objects={len(objects)} "
                    f"roundtrip={roundtrip_ms:.1f} ms sync={spread_ms:.1f} ms"
                )
                frame_id += 1
                if self.vehicle_interval_ms > 0:
                    time.sleep(self.vehicle_interval_ms / 1000.0)
            except Exception as exc:
                with self.state_lock:
                    self.vehicle_live_state["error"] = str(exc)
                self._log({"event": "vehicle_live_error", "reason": str(exc)})
                print(f"[VehicleLive] error: {exc}")
                time.sleep(1.0)

    def _start_pc900(self, optional: bool) -> None:
        if self.pc900_thread is not None and self.pc900_thread.is_alive():
            return
        pc900: PC900Input | None = None
        try:
            pc900 = PC900Input(self.pc900_path)
            pc900.open()
            sample = pc900.poll()
            target = pc900_motion(sample, self.chassis_config, False)
        except Exception as exc:
            with self.state_lock:
                self.pc900 = None
                self.pc900_present = False
                self.pc900_error = str(exc)
            if not optional:
                raise
        else:
            with self.state_lock:
                self.pc900 = pc900
                self.pc900_present = True
                self.pc900_error = None
                self.pc900_last_input = time.monotonic()
                self.pc900_state = sample.as_dict(target[0], target[1], False)
        self.pc900_thread = threading.Thread(
            target=self._pc900_loop, name="pc900-input", daemon=True
        )
        self.pc900_thread.start()

    def _write_session(self) -> None:
        if self.session_file is not None:
            self.session_file.write_text(
                json.dumps(self.session_data, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )

    def _log(self, payload: dict[str, Any]) -> None:
        payload = {"timestamp_ns": time.time_ns(), **payload}
        if self.events is not None:
            with self.event_lock:
                self.events.write(json.dumps(payload, ensure_ascii=False) + "\n")

    def arm(self) -> dict[str, Any]:
        with self.state_lock:
            if not self.ready:
                raise RuntimeError("系统尚未就绪")
            if self.control_source == "pc900":
                raise PermissionError("PC900 方向盘控制中，网页键盘已锁定。")
            self.armed = True
            self.control_source = "web"
            self.pc900_restore_pending = False
            self.desired_keys.clear()
            self.last_heartbeat = time.monotonic()
            self.stop_reason = "遥控已启用"
            self.error = None
        self._log({"event": "arm"})
        return self.status()

    def activate_pc900(self) -> dict[str, Any]:
        stop_web_motion = False
        with self.state_lock:
            if not self.ready:
                raise RuntimeError("系统尚未就绪")
            if not self.pc900_present:
                raise PermissionError(f"PC900 未就绪：{self.pc900_error or '未连接'}")
            if not bool(self.pc900_state.get("neutral_for_arm")):
                self.stop_reason = "PC900 解锁被拒绝：方向回中并松开油门、刹车"
                self._log({"event": "pc900_arm_refused", "source": "web_button"})
                raise PermissionError(self.stop_reason)
            stop_web_motion = self.armed and self.control_source == "web"
            self.armed = True
            self.control_source = "pc900"
            self.pc900_restore_pending = False
            self.desired_keys.clear()
            self.stop_reason = "PC900 方向盘已启用"
            self.error = None
            self.pc900_state["armed"] = True
        if stop_web_motion:
            self._safe_stop()
        self._log({"event": "pc900_arm", "source": "web_button"})
        return self.status()

    def disarm(self, reason: str = "用户结束运行") -> dict[str, Any]:
        with self.state_lock:
            was_armed = self.armed
            self.armed = False
            self.control_source = "web"
            self.pc900_restore_pending = False
            self.desired_keys.clear()
            self.stop_reason = reason
        self._safe_stop()
        if was_armed:
            self._log({"event": "disarm", "reason": reason})
        return self.status()

    def heartbeat(self) -> dict[str, Any]:
        with self.state_lock:
            if self.armed and self.control_source == "web":
                self.last_heartbeat = time.monotonic()
        return self.status()

    def set_input(self, keys: list[str]) -> dict[str, Any]:
        normalized = {str(key).upper() for key in keys}
        if not normalized <= DRIVE_KEYS:
            raise ValueError("Only W/A/S/D are valid drive keys.")
        with self.state_lock:
            if not self.armed:
                raise PermissionError("请先点击开始运行")
            if self.control_source != "web":
                raise PermissionError("PC900 方向盘控制中，网页方向键已锁定。")
            self.desired_keys = normalized
            self.last_heartbeat = time.monotonic()
        return self.status()

    def _desired_motion(self, keys: set[str]) -> tuple[float, float]:
        linear = (int("W" in keys) - int("S" in keys)) * float(
            self.chassis_config["normal_speed_mps"]
        )
        angular = (int("A" in keys) - int("D" in keys)) * float(
            self.chassis_config["turn_rate_radps"]
        )
        return linear, angular

    def _safe_stop(self) -> None:
        if self.ui_only or self.chassis is None:
            self.last_applied = (0.0, 0.0)
            return
        try:
            with self.motor_lock:
                self.chassis.stop()
        finally:
            self.last_applied = (0.0, 0.0)

    def _control_loop(self) -> None:
        while self.running.is_set():
            camera_failure = (
                None if self.ui_only or self.rig is None else self.rig.failure_message()
            )
            if camera_failure:
                with self.state_lock:
                    first_report = self.error != camera_failure
                    self.ready = False
                    self.armed = False
                    self.desired_keys.clear()
                    self.stop_reason = "摄像头流中断，已自动停车并锁定"
                    self.error = camera_failure
                self._safe_stop()
                if first_report:
                    self._log({"event": "camera_stream_failure", "reason": camera_failure})
                time.sleep(0.1)
                continue
            with self.state_lock:
                armed = self.armed
                control_source = self.control_source
                keys = set(self.desired_keys)
                heartbeat_age = time.monotonic() - self.last_heartbeat
                if armed and control_source == "web" and heartbeat_age > self.watchdog_s:
                    self.armed = False
                    self.control_source = "web"
                    self.desired_keys.clear()
                    self.stop_reason = "网页心跳超时，已自动停车并锁定"
                    armed = False
                    keys.clear()
                    self._log({"event": "watchdog_stop", "heartbeat_age_s": heartbeat_age})
            if control_source == "pc900":
                time.sleep(0.025)
                continue
            target = self._desired_motion(keys) if armed and control_source == "web" else (0.0, 0.0)
            if target != self.last_applied:
                try:
                    if target == (0.0, 0.0):
                        self._safe_stop()
                    else:
                        assert self.chassis is not None
                        with self.motor_lock:
                            commands = self.chassis.command(*target)
                        self.last_applied = target
                        self._log(
                            {
                                "event": "motion",
                                "keys": sorted(keys),
                                "linear_mps": target[0],
                                "angular_radps": target[1],
                                "motor_commands": commands,
                            }
                        )
                except Exception as exc:
                    with self.state_lock:
                        self.error = str(exc)
                        self.armed = False
                        self.desired_keys.clear()
                        self.stop_reason = "底盘控制异常，已锁定"
                    try:
                        self._safe_stop()
                    except Exception:
                        pass
            time.sleep(0.025)

    def _capture_from_pc900(self) -> None:
        try:
            self.capture(require_armed_source="pc900")
        except Exception as exc:
            self._log({"event": "pc900_snapshot_rejected", "reason": str(exc)})

    def _pc900_loop(self) -> None:
        reconnect_sleep_s = self.pc900_reconnect_sleep_s
        while self.running.is_set():
            if self.pc900 is None:
                try:
                    pc900 = PC900Input(self.pc900_path)
                    pc900.open()
                    sample = pc900.poll()
                    target = pc900_motion(sample, self.chassis_config, False)
                except Exception as exc:
                    with self.state_lock:
                        self.pc900_present = False
                        self.pc900_error = str(exc)
                        self.pc900_state["armed"] = False
                    time.sleep(reconnect_sleep_s)
                    continue
                with self.state_lock:
                    self.pc900 = pc900
                    self.pc900_present = True
                    self.pc900_error = None
                    self.pc900_last_input = time.monotonic()
                    restore = self.pc900_restore_pending and bool(sample.neutral_for_arm)
                    if restore:
                        self.armed = True
                        self.control_source = "pc900"
                        self.stop_reason = "PC900 已重连并自动恢复控制"
                        self.error = None
                        self.pc900_restore_pending = False
                    self.pc900_state = sample.as_dict(target[0], target[1], restore)
                    if self.pc900_restore_pending and not restore:
                        self.stop_reason = "PC900 已重连：方向回中并松开油门、刹车后自动恢复"
                self._log({"event": "pc900_reconnected"})
            try:
                pc900 = self.pc900
                assert pc900 is not None
                period = 1.0 / pc900.poll_hz()
                sample = pc900.poll()
                self._handle_pc900_sample(sample)
            except Exception as exc:
                pc900_to_close = self.pc900
                if pc900_to_close is not None:
                    try:
                        pc900_to_close.close()
                    except Exception:
                        pass
                with self.state_lock:
                    self.pc900 = None
                    self.pc900_present = False
                    self.pc900_error = str(exc)
                    was_pc900 = self.control_source == "pc900"
                    self.armed = False if was_pc900 else self.armed
                    self.control_source = "web"
                    self.pc900_restore_pending = self.pc900_restore_pending or was_pc900
                    self.pc900_state["armed"] = False
                    self.stop_reason = "PC900 输入故障，已停车并锁定" if was_pc900 else self.stop_reason
                if was_pc900:
                    self._safe_stop()
                    self._log({"event": "pc900_failure", "reason": str(exc)})
                time.sleep(reconnect_sleep_s)
                continue
            time.sleep(period)

    def _handle_pc900_sample(self, sample: PC900Sample) -> None:
        now = time.monotonic()
        start_button = self.pc900.button("start_toggle") if self.pc900 else 8
        capture_button = self.pc900.button("capture") if self.pc900 else 4
        capture_requested = False
        apply_motion = False
        with self.state_lock:
            self.pc900_last_input = now
            was_pc900_active = self.armed and self.control_source == "pc900"
            if start_button in sample.rising_buttons:
                if self.armed and self.control_source == "pc900":
                    self.armed = False
                    self.control_source = "web"
                    self.stop_reason = "PC900 Button 8 锁定"
                    self.desired_keys.clear()
                    self._log({"event": "pc900_lock"})
                else:
                    if sample.neutral_for_arm:
                        self.armed = True
                        self.control_source = "pc900"
                        self.stop_reason = "PC900 方向盘已启用"
                        self.error = None
                        self.desired_keys.clear()
                        self._log({"event": "pc900_arm"})
                    else:
                        self.stop_reason = "PC900 解锁被拒绝：方向回中并松开油门、刹车"
                        self._log({"event": "pc900_arm_refused"})
            pc900_armed = self.armed and self.control_source == "pc900"
            if pc900_armed and capture_button in sample.rising_buttons:
                capture_requested = True
            target = pc900_motion(sample, self.chassis_config, pc900_armed)
            apply_motion = was_pc900_active or pc900_armed
            self.pc900_state = sample.as_dict(target[0], target[1], pc900_armed)
            timeout = self.pc900.watchdog_timeout_s() if self.pc900 else 2.0
            if pc900_armed and now - self.pc900_last_input > timeout:
                self.armed = False
                self.control_source = "web"
                self.stop_reason = "PC900 输入心跳超时，已自动停车并锁定"
                target = (0.0, 0.0)
                apply_motion = True
                self._log({"event": "pc900_watchdog_stop"})
        if capture_requested:
            threading.Thread(target=self._capture_from_pc900, name="pc900-capture", daemon=True).start()
        if apply_motion and target != self.last_applied:
            try:
                if target == (0.0, 0.0):
                    self._safe_stop()
                elif self.ui_only:
                    self.last_applied = target
                else:
                    assert self.chassis is not None
                    with self.motor_lock:
                        commands = self.chassis.command(*target)
                    self.last_applied = target
                    self._log(
                        {
                            "event": "pc900_motion",
                            "linear_mps": target[0],
                            "angular_radps": target[1],
                            "motor_commands": commands,
                        }
                    )
            except Exception as exc:
                with self.state_lock:
                    self.error = str(exc)
                    self.armed = False
                    self.control_source = "web"
                    self.stop_reason = "底盘控制异常，已锁定"
                self._safe_stop()

    def capture(self, require_armed_source: str | None = None) -> dict[str, Any]:
        request_utc_ns = time.time_ns()
        with self.state_lock:
            if not self.armed:
                raise PermissionError("请先点击开始运行")
            if require_armed_source is not None and self.control_source != require_armed_source:
                raise PermissionError("当前控制源无权触发采样。")
        if not self.capture_lock.acquire(blocking=False):
            raise RuntimeError("上一帧六路采样尚未完成")
        try:
            next_number = self.frame_number + 1
            if self.ui_only:
                time.sleep(0.12)
                frame_name = f"frame_{next_number:06d}"
                completed_utc_ns = time.time_ns()
                metadata = {
                    "frame": frame_name,
                    "saved_at": now_iso(),
                    "arrival_spread_ms": 42.6,
                    "max_sync_spread_ms": 75.0,
                    "software_sync_passed": True,
                    "active_camera_fps": 15,
                    "active_input_mode": "mjpeg",
                    "trigger_to_pc_ms": 120.0,
                    "request_to_pc_ms": (completed_utc_ns - request_utc_ns) / 1_000_000,
                    "image_urls": {
                        name: f"/preview/{quote(name)}.svg?frame={next_number}" for name in CAMERA_NAMES
                    },
                }
            else:
                assert self.rig is not None and self.session_dir is not None
                _, metadata = self.rig.capture(next_number)
                completed_utc_ns = time.time_ns()
                metadata["request_received_utc_ns"] = request_utc_ns
                metadata["response_ready_utc_ns"] = completed_utc_ns
                metadata["request_to_pc_ms"] = (completed_utc_ns - request_utc_ns) / 1_000_000
                metadata["image_urls"] = {
                    name: "/data/" + "/".join(quote(part) for part in rel.split("/"))
                    for name, rel in metadata["camera_files"].items()
                }
                metadata_file = self.session_dir / "metadata" / f"{metadata['frame']}.json"
                metadata_file.write_text(
                    json.dumps(metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
                )
            self.frame_number = next_number
            with self.state_lock:
                self.latest_capture = metadata
            self._log(
                {
                    "event": "snapshot",
                    "frame": metadata["frame"],
                    "request_to_pc_ms": metadata["request_to_pc_ms"],
                    "arrival_spread_ms": metadata["arrival_spread_ms"],
                    "captured_while_motion_command_active": self.last_applied not in (None, (0.0, 0.0)),
                }
            )
            return metadata
        except (RuntimeError, TimeoutError) as exc:
            self._log({"event": "snapshot_rejected", "reason": str(exc)})
            raise
        finally:
            self.capture_lock.release()

    def status(self) -> dict[str, Any]:
        with self.state_lock:
            heartbeat_age_ms = (
                (time.monotonic() - self.last_heartbeat) * 1000 if self.armed else None
            )
            return {
                "ready": self.ready,
                "armed": self.armed,
                "keys": sorted(self.desired_keys),
                "moving": self.last_applied not in (None, (0.0, 0.0)),
                "control_source": self.control_source,
                "capturing": self.capture_lock.locked(),
                "frame_count": self.frame_number,
                "latest_capture": self.latest_capture,
                "session_dir": str(self.session_dir) if self.session_dir else "界面预览，不写采集数据",
                "motor_statuses": self.motor_statuses,
                "heartbeat_age_ms": heartbeat_age_ms,
                "watchdog_ms": int(self.watchdog_s * 1000),
                "message": self.stop_reason,
                "error": self.error,
                "ui_only": self.ui_only,
                "pc900_present": self.pc900_present,
                "pc900_error": self.pc900_error,
                "pc900_restore_pending": self.pc900_restore_pending,
                "pc900_state": self.pc900_state,
                "vehicle_live": self.vehicle_live_state,
                "camera_live_count": (
                    6 if self.ui_only else (self.rig.live_stream_count() if self.rig else 0)
                ),
                "camera_capture_profile": (
                    {
                        "active_fps": 15,
                        "input_mode": "mjpeg",
                        "max_sync_spread_ms": 75.0,
                    }
                    if self.ui_only or self.rig is None
                    else self.rig.capture_profile()
                ),
            }

    def close(self) -> None:
        self.running.clear()
        try:
            self.disarm("服务已关闭")
        except Exception:
            pass
        if self.control_thread is not None:
            self.control_thread.join(timeout=3.0)
        if self.pc900_thread is not None:
            self.pc900_thread.join(timeout=2.5)
        if self.vehicle_live_thread is not None:
            self.vehicle_live_thread.join(timeout=2.5)
        if self.pc900 is not None:
            self.pc900.close()
            self.pc900 = None
        if self.rig is not None:
            self.rig.stop()
        if self.events is not None:
            self.events.close()
            self.events = None
        if self.chassis is not None:
            self.chassis.close()
        self.session_data.update(
            {"finished_at": now_iso(), "six_view_frame_count": self.frame_number}
        )
        self._write_session()


class DashboardHandler(BaseHTTPRequestHandler):
    server_version = "CarDashboard/1.0"

    @property
    def app(self) -> DashboardApp:
        return self.server.app  # type: ignore[attr-defined]

    def log_message(self, fmt: str, *args: Any) -> None:
        print(f"[HTTP] {self.address_string()} {fmt % args}")

    def _send_bytes(self, data: bytes, content_type: str, status: int = 200) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store, max-age=0")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        self.wfile.write(data)

    def _json(self, payload: dict[str, Any], status: int = 200) -> None:
        self._send_bytes(
            json.dumps(payload, ensure_ascii=False).encode("utf-8"),
            "application/json; charset=utf-8",
            status,
        )

    def _read_json(self) -> dict[str, Any]:
        size = int(self.headers.get("Content-Length", "0"))
        if size > 65536:
            raise ValueError("Request is too large.")
        if size == 0:
            return {}
        return json.loads(self.rfile.read(size).decode("utf-8"))

    def do_GET(self) -> None:
        path = urlparse(self.path).path
        if path == "/":
            self._send_bytes((WEB_ROOT / "index.html").read_bytes(), "text/html; charset=utf-8")
            return
        if path == "/api/status":
            self._json(self.app.status())
            return
        if path == "/live/front.mjpg":
            self._serve_front_live_stream()
            return
        if path.startswith("/data/"):
            self._serve_session_file(path[len("/data/") :])
            return
        if path.startswith("/preview/") and self.app.ui_only:
            name = unquote(Path(path).stem)
            self._send_bytes(self._preview_svg(name), "image/svg+xml; charset=utf-8")
            return
        self._json({"error": "not found"}, HTTPStatus.NOT_FOUND)

    def _serve_session_file(self, relative_url: str) -> None:
        if self.app.session_dir is None:
            self._json({"error": "no session"}, HTTPStatus.NOT_FOUND)
            return
        session = self.app.session_dir.resolve()
        target = (session / Path(unquote(relative_url))).resolve()
        if session not in target.parents or not target.is_file():
            self._json({"error": "file not found"}, HTTPStatus.NOT_FOUND)
            return
        mime = mimetypes.guess_type(target.name)[0] or "application/octet-stream"
        self._send_bytes(target.read_bytes(), mime)

    def _serve_front_live_stream(self) -> None:
        boundary = "frontframe"
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={boundary}")
        self.send_header("Cache-Control", "no-store, max-age=0")
        self.send_header("Pragma", "no-cache")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()

        last_monotonic_ns = 0
        interval_s = 1.0 / max(1, int(self.app.camera_config.get("fps", 15)))
        while self.app.running.is_set() or self.app.ui_only:
            if self.app.ui_only:
                frame = self._preview_svg("CAM_FRONT")
                monotonic_ns = time.monotonic_ns()
                content_type = "image/svg+xml"
            elif self.app.rig is None:
                time.sleep(0.1)
                continue
            else:
                latest = self.app.rig.latest_jpeg("CAM_FRONT")
                if latest is None:
                    time.sleep(0.02)
                    continue
                frame, _, monotonic_ns = latest
                content_type = "image/jpeg"
            if monotonic_ns == last_monotonic_ns:
                time.sleep(0.01)
                continue
            last_monotonic_ns = monotonic_ns
            try:
                self.wfile.write(
                    f"--{boundary}\r\n"
                    f"Content-Type: {content_type}\r\n"
                    f"Content-Length: {len(frame)}\r\n\r\n"
                    .encode("ascii")
                )
                self.wfile.write(frame)
                self.wfile.write(b"\r\n")
                self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                break
            time.sleep(interval_s)

    def _preview_svg(self, name: str) -> bytes:
        safe_name = name if name in CAMERA_NAMES else "CAMERA"
        hue = (sum(ord(ch) for ch in safe_name) * 7) % 360
        svg = f"""<svg xmlns='http://www.w3.org/2000/svg' width='640' height='480' viewBox='0 0 640 480'>
<defs><linearGradient id='g' x1='0' y1='0' x2='1' y2='1'><stop stop-color='hsl({hue} 75% 18%)'/><stop offset='1' stop-color='#07101d'/></linearGradient></defs>
<rect width='640' height='480' fill='url(#g)'/><path d='M0 355 L180 235 L300 315 L430 170 L640 340 V480 H0Z' fill='#0ee7d455' stroke='#67f9ee' stroke-width='3'/>
<circle cx='540' cy='92' r='42' fill='#f7c95d' opacity='.9'/><text x='32' y='62' fill='#dffeff' font-size='27' font-family='monospace'>{safe_name}</text>
<text x='32' y='440' fill='#9cb4ca' font-size='19' font-family='sans-serif'>UI PREVIEW · NO HARDWARE</text></svg>"""
        return svg.encode("utf-8")

    def do_POST(self) -> None:
        path = urlparse(self.path).path
        try:
            body = self._read_json()
            if path == "/api/arm":
                result = self.app.arm()
            elif path in ("/api/disarm", "/api/stop"):
                result = self.app.disarm(str(body.get("reason", "用户停车")))
            elif path == "/api/pc900/activate":
                result = self.app.activate_pc900()
            elif path == "/api/heartbeat":
                result = self.app.heartbeat()
            elif path == "/api/input":
                result = self.app.set_input(list(body.get("keys", [])))
            elif path == "/api/capture":
                result = self.app.capture()
            else:
                self._json({"error": "not found"}, HTTPStatus.NOT_FOUND)
                return
            self._json(result)
        except PermissionError as exc:
            self._json({"error": str(exc)}, HTTPStatus.CONFLICT)
        except (ValueError, RuntimeError, TimeoutError) as exc:
            self._json({"error": str(exc)}, HTTPStatus.BAD_REQUEST)
        except Exception as exc:
            self._json({"error": f"internal error: {exc}"}, HTTPStatus.INTERNAL_SERVER_ERROR)


class DashboardServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address: tuple[str, int], app: DashboardApp):
        super().__init__(address, DashboardHandler)
        self.app = app


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--chassis-config", type=Path, default=ROOT / "config" / "chassis.json")
    parser.add_argument("--camera-config", type=Path, default=ROOT / "config" / "cameras.json")
    parser.add_argument("--output-root", type=Path, default=ROOT.parent / "采集数据")
    parser.add_argument(
        "--serial-port",
        default="AUTO",
        help="chassis COM port, or AUTO to detect the single attached CH340",
    )
    parser.add_argument("--bind", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--watchdog-s", type=float, default=1.0)
    parser.add_argument("--execute", action="store_true", help="allow real chassis/camera initialization")
    parser.add_argument("--ui-only", action="store_true", help="safe UI preview without the chassis serial port or cameras")
    parser.add_argument("--pc900-config", type=Path, default=ROOT / "config" / "pc900.json")
    parser.add_argument("--pc900", dest="pc900_enabled", action="store_true", default=True, help="try to enable the PC900 wheel/pedal controller")
    parser.add_argument("--no-pc900", dest="pc900_enabled", action="store_false", help="disable PC900 support for this run")
    parser.add_argument("--vehicle-live", action="store_true", help="stream synchronized six-camera frames to the Vehicle board service")
    parser.add_argument("--vehicle-edge-host", default="192.168.125.166")
    parser.add_argument("--vehicle-edge-port", type=int, default=5200)
    parser.add_argument("--vehicle-viewer", default="http://127.0.0.1:8093")
    parser.add_argument("--vehicle-ffmpeg", help="optional ffmpeg.exe path for Vehicle Raw BGR decoding")
    parser.add_argument("--vehicle-interval-ms", type=int, default=0, help="extra delay after each Vehicle inference result")
    parser.add_argument("--open-browser", action="store_true")
    args = parser.parse_args()
    if not args.execute and not args.ui_only:
        raise RuntimeError("Refusing hardware startup without --execute; use --ui-only for a safe preview.")
    if args.execute and not args.ui_only:
        args.serial_port = resolve_serial_port(args.serial_port)
        print(f"Using chassis serial port: {args.serial_port}")
    app = DashboardApp(
        args.chassis_config,
        args.camera_config,
        args.output_root,
        args.serial_port,
        args.watchdog_s,
        args.ui_only,
        args.pc900_config,
        args.pc900_enabled,
        args.vehicle_live,
        args.vehicle_edge_host,
        args.vehicle_edge_port,
        args.vehicle_viewer,
        args.vehicle_ffmpeg,
        args.vehicle_interval_ms,
    )
    server: DashboardServer | None = None
    try:
        print("Initializing chassis and six cameras..." if not args.ui_only else "Starting safe UI preview...")
        app.start()
        server = DashboardServer((args.bind, args.port), app)
        host = "127.0.0.1" if args.bind in ("0.0.0.0", "::") else args.bind
        url = f"http://{host}:{args.port}/"
        print(json.dumps({"status": "ready", "url": url, "session": app.status()["session_dir"]}, ensure_ascii=False))
        print("Close with Ctrl+C. Browser heartbeat loss automatically stops and locks the chassis.")
        if args.open_browser:
            timer = threading.Timer(0.35, webbrowser.open, args=(url,))
            timer.daemon = True
            timer.start()
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        if server is not None:
            server.server_close()
        app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
