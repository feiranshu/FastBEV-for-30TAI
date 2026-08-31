#!/usr/bin/env python3
"""Keyboard-drive the EMM42 chassis and take six-view DirectShow snapshots.

The program begins stationary. Motor movement needs --allow-motion. Each R press
saves exactly one new image from each of six surround cameras. Releasing
W/A/S/D stops the motors; SPACE and ESC stop now.
"""

from __future__ import annotations

import argparse
import ctypes
import json
import math
import shutil
import subprocess
import threading
import time
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from emm42_modbus import Emm42Modbus
from probe_cameras import discover_sources, ffmpeg_path


ROOT = Path(__file__).resolve().parent
VK = {"W": 0x57, "A": 0x41, "S": 0x53, "D": 0x44, "R": 0x52, "SPACE": 0x20, "ESC": 0x1B}


def now_ns() -> int:
    return time.time_ns()


def now_iso() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def key_down(key: str) -> bool:
    return bool(ctypes.windll.user32.GetAsyncKeyState(VK[key]) & 0x8000)


def edge_pressed(key: str, previous: dict[str, bool]) -> bool:
    current = key_down(key)
    pressed = current and not previous.get(key, False)
    previous[key] = current
    return pressed


@dataclass(frozen=True)
class CameraSpec:
    name: str
    source: str


def load_specs(config: dict[str, Any], validate_sources: bool = True) -> list[CameraSpec]:
    specs = [
        CameraSpec(str(item["name"]), str(item.get("source", "")))
        for item in config["cameras"]
        if item.get("enabled", True)
    ]
    if len(specs) != 6 or len({spec.name for spec in specs}) != 6:
        raise ValueError("config/cameras.json must have exactly six uniquely named surround cameras.")
    if any(not spec.source.startswith("@device_pnp_") for spec in specs):
        raise ValueError("Cameras are unconfigured. Run probe_cameras.cmd and configure_cameras.cmd first.")
    if len({spec.source for spec in specs}) != 6:
        raise ValueError("Each semantic camera must use a distinct DirectShow source.")
    excluded_top = str(config.get("excluded_top_source", ""))
    if excluded_top and any(spec.source == excluded_top for spec in specs):
        raise ValueError("The known TOP camera is present in the six-camera list; it must remain excluded.")
    if validate_sources:
        available = set(discover_sources(None))
        missing = [spec for spec in specs if spec.source not in available]
        if missing:
            missing_lines = "\n".join(f"  {spec.name}: {spec.source}" for spec in missing)
            raise RuntimeError(
                "Configured six-camera sources are not all present.\n"
                f"Found {len(available)} non-virtual DirectShow camera source(s), but 6 configured sources are required.\n"
                "Missing configured source(s):\n"
                f"{missing_lines}\n"
                "Reconnect the missing USB cameras or run probe_cameras.cmd --all, preview_camera.cmd, "
                "and configure_cameras.cmd after all six vehicle-facing cameras are visible."
            )
    return specs


class SixCameraSnapshotRig:
    """Keep all cameras open and save a time-paired six-view frame per trigger.

    The six UVC devices are free-running cameras. A short rolling buffer lets us
    select the six JPEGs closest to one another around the user's trigger instead
    of blindly taking the next frame produced by every independent stream.
    """

    def __init__(self, session_dir: Path, specs: list[CameraSpec], config: dict[str, Any]):
        self.session_dir, self.specs, self.config = session_dir, specs, config
        self.processes: dict[str, subprocess.Popen[bytes]] = {}
        self.logs: dict[str, Any] = {}
        self.readers: dict[str, threading.Thread] = {}
        self.latest: dict[str, tuple[bytes, int, int]] = {}
        self.buffer_frames = max(4, int(config.get("buffer_frames", 16)))
        self.buffers: dict[str, deque[tuple[bytes, int, int]]] = {
            spec.name: deque(maxlen=self.buffer_frames) for spec in specs
        }
        self.last_saved_monotonic_ns: dict[str, int] = {spec.name: 0 for spec in specs}
        self.errors: dict[str, str] = {}
        self.condition = threading.Condition()
        self.stopping = False
        self.active_fps: int | None = None
        self.active_input_mode = "auto"
        self.writer_pool = ThreadPoolExecutor(
            max_workers=len(specs), thread_name_prefix="six-camera-writer"
        )

    def start(self) -> None:
        logs_dir = self.session_dir / "logs"
        images_dir = self.session_dir / "images"
        logs_dir.mkdir(exist_ok=True)
        (self.session_dir / "metadata").mkdir(exist_ok=True)
        for spec in self.specs:
            (images_dir / spec.name).mkdir(parents=True, exist_ok=True)

        configured_fps = int(self.config["fps"])
        fps_attempts = [configured_fps]
        for value in self.config.get("fallback_fps", []):
            value = int(value)
            if value > 0 and value not in fps_attempts:
                fps_attempts.append(value)
        prefer_mjpeg = bool(self.config.get("prefer_input_mjpeg", True))
        attempts = (
            [(fps, True) for fps in fps_attempts]
            + [(fps, False) for fps in fps_attempts]
            if prefer_mjpeg
            else [(fps, False) for fps in fps_attempts]
        )
        failures: list[str] = []
        for attempt_number, (fps, force_mjpeg) in enumerate(attempts, start=1):
            self._reset_stream_state()
            try:
                self._start_streams(logs_dir, attempt_number, fps, force_mjpeg)
                self._wait_for_buffer_depth(
                    min(3, self.buffer_frames),
                    timeout=float(self.config.get("camera_start_timeout_s", 12.0)),
                )
                self.active_fps = fps
                self.active_input_mode = "mjpeg" if force_mjpeg else "auto"
                return
            except Exception as exc:
                mode = "mjpeg" if force_mjpeg else "auto"
                failures.append(f"{fps} FPS/{mode}: {exc}")
                self._shutdown_streams()
                if attempt_number < len(attempts):
                    time.sleep(float(self.config.get("camera_retry_delay_s", 1.0)))
        raise RuntimeError("Unable to open all six cameras. " + " | ".join(failures))

    def _reset_stream_state(self) -> None:
        self.processes.clear()
        self.logs.clear()
        self.readers.clear()
        self.latest.clear()
        self.buffers = {
            spec.name: deque(maxlen=self.buffer_frames) for spec in self.specs
        }
        self.errors.clear()
        self.stopping = False

    def _start_streams(
        self, logs_dir: Path, attempt_number: int, fps: int, force_mjpeg: bool
    ) -> None:
        for spec in self.specs:
            mode = "mjpeg" if force_mjpeg else "auto"
            log = (logs_dir / f"ffmpeg_attempt{attempt_number}_{fps}fps_{mode}_{spec.name}.log").open(
                "wb", buffering=0
            )
            command = [
                str(ffmpeg_path()), "-hide_banner", "-loglevel", "warning", "-f", "dshow",
                "-thread_queue_size", "32", "-rtbufsize", "128M",
                "-video_size", f"{int(self.config['width'])}x{int(self.config['height'])}",
                "-framerate", str(fps),
            ]
            if force_mjpeg:
                command.extend(("-vcodec", "mjpeg"))
            command.extend(["-i", f"video={spec.source}", "-an"])
            if force_mjpeg:
                # DirectShow devices can repeat PTS/DTS; replace packet timestamps
                # while copying the already-compressed JPEG frames.
                command.extend([
                    "-c:v", "copy",
                    "-bsf:v", "setts=pts=N:dts=N:duration=1",
                ])
            else:
                command.extend([
                    "-vf", f"setpts=N/({fps}*TB)",
                    "-fps_mode", "vfr", "-c:v", "mjpeg", "-q:v", "2",
                ])
            command.extend(["-f", "image2pipe", "pipe:1"])
            process = subprocess.Popen(
                command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=log, bufsize=0
            )
            self.processes[spec.name] = process
            self.logs[spec.name] = log
            reader = threading.Thread(target=self._read_jpegs, args=(spec.name, process), daemon=True)
            self.readers[spec.name] = reader
            reader.start()
            time.sleep(float(self.config.get("camera_open_stagger_ms", 75.0)) / 1000.0)

    def _wait_for_buffer_depth(self, depth: int, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        with self.condition:
            while True:
                if self.errors:
                    detail = "; ".join(f"{name}: {error}" for name, error in self.errors.items())
                    raise RuntimeError(f"Camera stream failed: {detail}. Inspect session logs.")
                missing = [
                    spec.name for spec in self.specs if len(self.buffers[spec.name]) < depth
                ]
                if not missing:
                    return
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(f"Timed out warming camera buffers: {', '.join(missing)}")
                self.condition.wait(min(remaining, 0.25))

    def _read_jpegs(self, name: str, process: subprocess.Popen[bytes]) -> None:
        stream = process.stdout
        if stream is None:
            return
        buffer = bytearray()
        try:
            while True:
                chunk = stream.read(65536)
                if not chunk:
                    break
                buffer.extend(chunk)
                while True:
                    start = buffer.find(b"\xff\xd8")
                    if start < 0:
                        if len(buffer) > 1:
                            del buffer[:-1]
                        break
                    if start:
                        del buffer[:start]
                    end = buffer.find(b"\xff\xd9", 2)
                    if end < 0:
                        break
                    jpeg = bytes(buffer[: end + 2])
                    del buffer[: end + 2]
                    with self.condition:
                        frame = (jpeg, time.time_ns(), time.monotonic_ns())
                        self.latest[name] = frame
                        self.buffers[name].append(frame)
                        self.condition.notify_all()
        except Exception as exc:
            with self.condition:
                self.errors[name] = str(exc)
                self.condition.notify_all()
        finally:
            with self.condition:
                if not self.stopping and name not in self.errors:
                    exit_code = process.poll()
                    if exit_code is None:
                        try:
                            exit_code = process.wait(timeout=0.25)
                        except subprocess.TimeoutExpired:
                            pass
                    self.errors[name] = f"FFmpeg stream ended (exit code {exit_code})."
                self.condition.notify_all()

    @staticmethod
    def _select_time_paired_frames(
        candidates: dict[str, list[tuple[bytes, int, int]]], trigger_monotonic_ns: int
    ) -> tuple[dict[str, tuple[bytes, int, int]], int, int] | None:
        """Choose a near-trigger set with the smallest cross-camera time span."""

        if not candidates or any(not frames for frames in candidates.values()):
            return None
        anchors = sorted({frame[2] for frames in candidates.values() for frame in frames})
        best: tuple[tuple[int, int, int], dict[str, tuple[bytes, int, int]], int, int] | None = None
        for anchor in anchors:
            selected = {
                name: min(frames, key=lambda frame: (abs(frame[2] - anchor), frame[2]))
                for name, frames in candidates.items()
            }
            times = [frame[2] for frame in selected.values()]
            spread_ns = max(times) - min(times)
            center_ns = (max(times) + min(times)) // 2
            score = (spread_ns, abs(center_ns - trigger_monotonic_ns), max(times))
            if best is None or score < best[0]:
                best = (score, selected, spread_ns, center_ns)
        if best is None:
            return None
        return best[1], best[2], best[3]

    def _collect_synchronized(
        self, trigger_monotonic_ns: int, timeout: float
    ) -> tuple[dict[str, tuple[bytes, int, int]], int, int]:
        pretrigger_ns = int(float(self.config.get("sync_pretrigger_ms", 160.0)) * 1_000_000)
        limit_ns = int(float(self.config.get("max_sync_spread_ms", 75.0)) * 1_000_000)
        settle_ns = int(float(self.config.get("sync_settle_ms", 15.0)) * 1_000_000)
        lower_bound = trigger_monotonic_ns - pretrigger_ns
        deadline_ns = trigger_monotonic_ns + int(timeout * 1_000_000_000)
        best_result: tuple[dict[str, tuple[bytes, int, int]], int, int] | None = None
        candidates: dict[str, list[tuple[bytes, int, int]]] = {}
        with self.condition:
            while True:
                if self.errors:
                    detail = "; ".join(f"{name}: {error}" for name, error in self.errors.items())
                    raise RuntimeError(f"Camera stream failed: {detail}. Inspect session logs.")
                candidates = {
                    spec.name: [
                        frame for frame in self.buffers[spec.name]
                        if frame[2] >= lower_bound
                        and frame[2] > self.last_saved_monotonic_ns[spec.name]
                    ]
                    for spec in self.specs
                }
                result = self._select_time_paired_frames(candidates, trigger_monotonic_ns)
                if result is not None and (best_result is None or result[1] < best_result[1]):
                    best_result = result
                now_monotonic_ns = time.monotonic_ns()
                if result is not None and result[1] <= limit_ns and now_monotonic_ns >= trigger_monotonic_ns + settle_ns:
                    return result
                remaining_ns = deadline_ns - now_monotonic_ns
                if remaining_ns <= 0:
                    break
                self.condition.wait(min(remaining_ns / 1_000_000_000, 0.05))

        if best_result is None:
            missing = [name for name, frames in candidates.items() if not frames]
            raise TimeoutError(
                "六路软件同步超时：未在时限内收到新帧：" + ", ".join(missing)
            )
        best_ms = best_result[1] / 1_000_000
        limit_ms = limit_ns / 1_000_000
        raise RuntimeError(
            f"六路软件同步拒绝（software synchronization rejected）："
            f"最佳跨度 {best_ms:.1f} ms 超过 {limit_ms:.1f} ms 上限，本次未保存图片。"
        )

    def capture(self, number: int, timeout: float = 5.0) -> tuple[Path, dict[str, Any]]:
        trigger_utc_ns = time.time_ns()
        trigger_monotonic_ns = time.monotonic_ns()
        sync_wait_s = min(timeout, float(self.config.get("sync_wait_ms", 120.0)) / 1000.0)
        frames, spread_ns, center_monotonic_ns = self._collect_synchronized(
            trigger_monotonic_ns, sync_wait_s
        )
        selected_utc_ns = {name: values[1] for name, values in frames.items()}
        selection_completed_utc_ns = time.time_ns()
        images_root = self.session_dir / "images"
        metadata_root = self.session_dir / "metadata"
        frame_name = f"frame_{number:06d}"
        partial_dir = self.session_dir / f".{frame_name}.partial"
        targets = {spec.name: images_root / spec.name / f"{frame_name}.jpg" for spec in self.specs}
        if partial_dir.exists() or any(path.exists() for path in targets.values()):
            raise FileExistsError(f"Snapshot target already exists: {frame_name}")
        partial_dir.mkdir()
        moved: list[Path] = []
        try:
            futures = [
                self.writer_pool.submit(
                    (partial_dir / f"{spec.name}.jpg").write_bytes,
                    frames[spec.name][0],
                )
                for spec in self.specs
            ]
            for future in futures:
                future.result()
            for spec in self.specs:
                target = targets[spec.name]
                (partial_dir / f"{spec.name}.jpg").replace(target)
                moved.append(target)
            partial_dir.rmdir()
        except Exception:
            for target in moved:
                if target.exists():
                    target.unlink()
            for item in partial_dir.glob("*"):
                item.unlink()
            if partial_dir.exists():
                partial_dir.rmdir()
            raise
        self.last_saved_monotonic_ns.update(
            {name: values[2] for name, values in frames.items()}
        )
        metadata = {
            "frame": frame_name,
            "trigger_utc_ns": trigger_utc_ns,
            "saved_at": now_iso(),
            "camera_files": {spec.name: f"images/{spec.name}/{frame_name}.jpg" for spec in self.specs},
            "frame_arrival_utc_ns": selected_utc_ns,
            "arrival_spread_ms": spread_ns / 1_000_000,
            "capture_center_offset_ms": (center_monotonic_ns - trigger_monotonic_ns) / 1_000_000,
            "pairing_wait_ms": (selection_completed_utc_ns - trigger_utc_ns) / 1_000_000,
            "max_sync_spread_ms": float(self.config.get("max_sync_spread_ms", 75.0)),
            "software_sync_passed": True,
            "active_camera_fps": self.active_fps,
            "active_input_mode": self.active_input_mode,
            "synchronization": (
                "Strict software time-pairing of continuously buffered USB frames. "
                "Hardware-synchronized exposure is not implied."
            ),
        }
        completed_utc_ns = time.time_ns()
        metadata["completed_utc_ns"] = completed_utc_ns
        metadata["trigger_to_pc_ms"] = (completed_utc_ns - trigger_utc_ns) / 1_000_000
        (metadata_root / f"{frame_name}.json").write_text(
            json.dumps(metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        return images_root, metadata

    def capture_profile(self) -> dict[str, Any]:
        return {
            "active_fps": self.active_fps,
            "input_mode": self.active_input_mode,
            "buffer_frames": self.buffer_frames,
            "max_sync_spread_ms": float(self.config.get("max_sync_spread_ms", 75.0)),
            "sync_pretrigger_ms": float(self.config.get("sync_pretrigger_ms", 160.0)),
            "sync_wait_ms": float(self.config.get("sync_wait_ms", 120.0)),
        }

    def failure_message(self) -> str | None:
        with self.condition:
            if not self.errors:
                return None
            detail = "; ".join(f"{name}: {error}" for name, error in self.errors.items())
            return f"六路摄像头流故障：{detail}"

    def latest_jpeg(self, name: str) -> tuple[bytes, int, int] | None:
        with self.condition:
            return self.latest.get(name)

    def live_stream_count(self) -> int:
        return sum(process.poll() is None for process in self.processes.values())

    def stop(self) -> None:
        self._shutdown_streams()
        self.writer_pool.shutdown(wait=True, cancel_futures=False)

    def _shutdown_streams(self) -> None:
        self.stopping = True
        for process in self.processes.values():
            if process.poll() is None and process.stdin is not None:
                try:
                    process.stdin.write(b"q\n")
                    process.stdin.flush()
                except OSError:
                    pass
        for process in self.processes.values():
            try:
                process.wait(timeout=8.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2.0)
        for reader in self.readers.values():
            reader.join(timeout=2.0)
        for log in self.logs.values():
            log.close()


class Chassis:
    def __init__(self, config: dict[str, Any], allow_motion: bool):
        self.config, self.allow_motion = config, allow_motion
        self.bus: Emm42Modbus | None = None
        self.last_signature: tuple[tuple[int, int, int], ...] | None = None
        self.moving = False
        self.driver_parameters: list[Any] = []

    def open(self) -> list[int]:
        self.bus = Emm42Modbus(str(self.config["serial_port"]), int(self.config["baudrate"]))
        statuses = self.bus.healthcheck(self.config["addresses"])
        self.driver_parameters = [self.bus.read_driver_parameters(address) for address in self.config["addresses"]]
        for params in self.driver_parameters:
            if params.serial_port_mode != 2 or params.baudrate_code != 5 or params.checksum_mode != 3:
                raise RuntimeError(
                    f"motor {params.address} communication settings are incompatible: "
                    f"serial_mode={params.serial_port_mode}, baud_code={params.baudrate_code}, "
                    f"checksum={params.checksum_mode}"
                )
        disabled = [str(status.address) for status in statuses if not status.enabled]
        faulted = [str(status.address) for status in statuses if status.motion_fault]
        if disabled or faulted:
            details = []
            if disabled:
                details.append(f"not enabled: {', '.join(disabled)}")
            if faulted:
                details.append(f"stall/protection fault: {', '.join(faulted)}")
            raise RuntimeError(
                "Refusing to arm motion (" + "; ".join(details) + "). "
                "Raise all wheels and resolve the driver state first."
            )
        return [item.raw for item in statuses]

    def _motor(self, address: int, speed_mps: float, circumference: float) -> tuple[int, int]:
        rpm = min(int(round(abs(speed_mps) * 60.0 / circumference)), int(self.config["max_motor_rpm"]))
        forward = int(self.config["forward_directions"][str(address)])
        return (forward if speed_mps >= 0 else 1 - forward), rpm

    def command(self, linear_mps: float, angular_radps: float) -> dict[int, tuple[int, int]]:
        width, circumference = float(self.config["effective_track_width_m"]), math.pi * float(self.config["wheel_diameter_m"])
        sides = {"left": linear_mps - angular_radps * width / 2.0, "right": linear_mps + angular_radps * width / 2.0}
        commands = {int(address): self._motor(int(address), sides[side], circumference) for side, addresses in (("left", self.config["left_addresses"]), ("right", self.config["right_addresses"])) for address in addresses}
        signature = tuple(sorted((address, direction, rpm) for address, (direction, rpm) in commands.items()))
        if signature == self.last_signature:
            return commands
        if self.allow_motion:
            if self.bus is None:
                raise RuntimeError("Motor bus is not open.")
            for address, (direction, rpm) in commands.items():
                self.bus.stage_speed(address, direction, rpm, int(self.config["acceleration"]))
                time.sleep(float(self.config.get("stage_settle_s", 0.1)))
            self.bus.trigger_staged_motion(float(self.config.get("trigger_settle_s", 0.2)))
            self.moving = any(rpm for _, rpm in commands.values())
        self.last_signature = signature
        return commands

    def stop(self) -> None:
        if self.allow_motion and self.bus is not None and self.moving:
            self.bus.immediate_stop_all(self.config["addresses"])
        self.moving, self.last_signature = False, None

    def close(self) -> None:
        try:
            self.stop()
        finally:
            if self.bus is not None:
                self.bus.close()
                self.bus = None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--chassis-config", type=Path, default=ROOT / "config" / "chassis.json")
    parser.add_argument("--camera-config", type=Path, default=ROOT / "config" / "cameras.json")
    parser.add_argument("--output-root", type=Path, default=ROOT.parent / "采集数据")
    parser.add_argument("--serial-port")
    parser.add_argument("--allow-motion", action="store_true")
    parser.add_argument("--cameras-only", action="store_true")
    parser.add_argument("--snapshot-once", action="store_true", help="save one six-view frame and exit without opening the chassis serial port")
    args = parser.parse_args()
    chassis_config, camera_config = read_json(args.chassis_config), read_json(args.camera_config)
    if args.serial_port:
        chassis_config["serial_port"] = args.serial_port
    specs = load_specs(camera_config)
    ffmpeg_path()
    camera_only = args.cameras_only or args.snapshot_once
    chassis = Chassis(chassis_config, args.allow_motion and not camera_only)
    statuses = [] if camera_only else chassis.open()
    if not camera_only:
        reply_names = {0: "None", 1: "Receive", 2: "Reached", 3: "Both", 4: "Other"}
        for address, status, params in zip(chassis_config["addresses"], statuses, chassis.driver_parameters):
            print(f"Motor {address}: status=0x{status:04X}, control reply={reply_names.get(params.control_reply_mode, params.control_reply_mode)}")
    session_dir = args.output_root / f"session_{datetime.now():%Y%m%d_%H%M%S}"
    session_dir.mkdir(parents=True, exist_ok=False)
    snapshot_dir = session_dir / "config_snapshot"
    snapshot_dir.mkdir()
    shutil.copy2(args.camera_config, snapshot_dir / "cameras.json")
    shutil.copy2(args.chassis_config, snapshot_dir / "chassis.json")
    session_data = {
        "started_at": now_iso(),
        "serial_port": chassis_config["serial_port"],
        "motor_status_registers": statuses,
        "motion_armed": chassis.allow_motion,
        "camera_names": [spec.name for spec in specs],
        "capture_mode": "one R press = one identically numbered JPEG in each of six semantic camera folders",
    }
    session_file = session_dir / "session.json"
    session_file.write_text(json.dumps(session_data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"Session: {session_dir}")
    print("Opening six surround cameras...")
    print("Keys: W/S drive, A/D turn, R one six-view snapshot, SPACE stop, ESC exit.")
    print("MOTION ARMED: first test must have all wheels raised." if chassis.allow_motion else "CAMERAS ONLY: chassis serial port is not opened.")
    keys: dict[str, bool] = {}
    previous: tuple[float, float] | None = None
    number = 0
    rig = SixCameraSnapshotRig(session_dir, specs, camera_config)
    events = None
    try:
        rig.start()
        session_data["camera_capture_profile"] = rig.capture_profile()
        session_file.write_text(json.dumps(session_data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        events = (session_dir / "control_events.jsonl").open("w", encoding="utf-8", buffering=1)
        print("CAMERAS READY: press R once to save exactly six images.")
        if args.snapshot_once:
            _, metadata = rig.capture(1)
            number = 1
            print(f"SAVED frame_000001: 6 images, arrival spread {metadata['arrival_spread_ms']:.1f} ms")
            return 0
        while True:
            if edge_pressed("ESC", keys):
                break
            if edge_pressed("SPACE", keys):
                chassis.stop()
            if edge_pressed("R", keys):
                try:
                    _, metadata = rig.capture(number + 1)
                except (RuntimeError, TimeoutError) as exc:
                    events.write(json.dumps({"event": "snapshot_rejected", "timestamp_ns": now_ns(), "reason": str(exc)}) + "\n")
                    print(f"REJECTED snapshot: {exc}")
                else:
                    number += 1
                    events.write(json.dumps({"event": "snapshot", "timestamp_ns": now_ns(), "frame": metadata["frame"], "arrival_spread_ms": metadata["arrival_spread_ms"]}) + "\n")
                    print(f"SAVED {metadata['frame']}: 6 images, arrival spread {metadata['arrival_spread_ms']:.1f} ms")
            w, s, a, d = (key_down(name) for name in ("W", "S", "A", "D"))
            linear, angular = (int(w) - int(s)) * float(chassis_config["normal_speed_mps"]), (int(a) - int(d)) * float(chassis_config["turn_rate_radps"])
            if not (w or s or a or d):
                chassis.stop()
            elif (linear, angular) != previous:
                commands = chassis.command(linear, angular)
                events.write(json.dumps({"event": "motion", "timestamp_ns": now_ns(), "linear_mps": linear, "angular_radps": angular, "motor_commands": commands}) + "\n")
            previous = (linear, angular)
            time.sleep(0.015)
    finally:
        chassis.stop()
        rig.stop()
        if events is not None:
            events.close()
        chassis.close()
        session_data.update({"finished_at": now_iso(), "six_view_frame_count": number})
        session_file.write_text(json.dumps(session_data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
