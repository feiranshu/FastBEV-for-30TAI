import argparse
import asyncio
import json
import logging
import math
import queue
import random
import signal
import threading
import time
from collections import deque
from fractions import Fraction
from pathlib import Path

import av
import carla
import numpy as np
from aiohttp import web
from aiortc import (
    RTCConfiguration,
    RTCPeerConnection,
    RTCRtpSender,
    RTCSessionDescription,
    VideoStreamTrack,
)

from detection_overlay import DetectionOverlay, FramePoseHistory
from nuscenes_rig import LatestBatchCache, NuScenesCameraRig
from protocol import MSG_RESULT, ProtocolError, unpack_header, unpack_result


LOG = logging.getLogger("live-drive")
ROOT = Path(__file__).resolve().parent
PCS = set()
VIDEO_CLOCK = 90000
VIDEO_FPS = 20
VIDEO_STEP = VIDEO_CLOCK // VIDEO_FPS
CAMERA_FOV = 85.0
DISPLAY_WIDTH = 960
DISPLAY_HEIGHT = 540
TRAFFIC_SETTLE_TICKS = 20


CAMERA_VIEWS = [
    {
        "name": "LONG LOW CHASE",
        "transform": carla.Transform(
            carla.Location(x=-5.8, y=0.0, z=2.15),
            carla.Rotation(pitch=-2.0, yaw=0.0, roll=0.0),
        ),
    },
    {
        "name": "RACE CHASE",
        "transform": carla.Transform(
            carla.Location(x=-4.8, y=0.0, z=2.35),
            carla.Rotation(pitch=-10.0, yaw=0.0, roll=0.0),
        ),
    },
    {
        "name": "MANUAL CONTROL",
        "transform": carla.Transform(
            carla.Location(x=-5.0, y=0.0, z=3.0),
            carla.Rotation(pitch=-20.0, yaw=0.0, roll=0.0),
        ),
    },
]


class LatestVideoFrame:
    def __init__(self, overlay):
        self.overlay = overlay
        self._condition = threading.Condition()
        self._sequence = 0
        self._bgr = None
        self._carla_frame = None
        self._timestamp = None

    def callback(self, image):
        bgra = np.frombuffer(image.raw_data, dtype=np.uint8)
        bgra = bgra.reshape(image.height, image.width, 4)
        bgr = np.ascontiguousarray(bgra[:, :, :3])
        try:
            self.overlay.draw(bgr, image.transform)
        except Exception:
            LOG.exception("Detection overlay failed for camera frame %s", image.frame)
        with self._condition:
            self._sequence += 1
            self._bgr = bgr
            self._carla_frame = image.frame
            self._timestamp = image.timestamp
            self._condition.notify_all()

    def wait_after(self, last_sequence, timeout=1.0):
        deadline = time.monotonic() + timeout
        with self._condition:
            while self._sequence <= last_sequence or self._bgr is None:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._condition.wait(remaining)
            return (
                self._sequence,
                self._bgr,
                self._carla_frame,
                self._timestamp,
            )


class BrowserControl:
    def __init__(self):
        self._lock = threading.Lock()
        self._keys = {"w": False, "a": False, "s": False, "d": False, "space": False}
        self._last_message = 0.0
        self._sequence = -1
        self._reverse = False
        self._view_index = 0
        self._requested_view_index = 0

    def update_json(self, message):
        try:
            data = json.loads(message)
        except (TypeError, json.JSONDecodeError):
            return
        if data.get("type") != "control":
            return
        sequence = int(data.get("seq", -1))
        with self._lock:
            if sequence <= self._sequence:
                return
            self._sequence = sequence
            keys = data.get("keys", {})
            for key in self._keys:
                self._keys[key] = bool(keys.get(key, False))
            self._reverse = bool(data.get("reverse", False))
            self._requested_view_index = int(data.get("view_index", 0)) % len(CAMERA_VIEWS)
            self._last_message = time.monotonic()

    def snapshot(self):
        with self._lock:
            return {
                "keys": dict(self._keys),
                "reverse": self._reverse,
                "view_index": self._requested_view_index,
                "age": time.monotonic() - self._last_message if self._last_message else math.inf,
            }


class ResultStore:
    def __init__(self, overlay):
        self.overlay = overlay
        self._lock = threading.Lock()
        self._gateway_connected = False
        self._gateway_peer = None
        self._last_result = None
        self._received_results = 0
        self._updated_monotonic = None
        self._last_overlay_accepted = None
        self._last_overlay_reason = None

    def set_gateway(self, connected, peer=None):
        with self._lock:
            self._gateway_connected = bool(connected)
            self._gateway_peer = peer if connected else None

    def update(self, result):
        accepted, reason = self.overlay.publish_result(result)
        with self._lock:
            self._last_result = dict(result)
            self._received_results += 1
            self._updated_monotonic = time.monotonic()
            self._last_overlay_accepted = accepted
            self._last_overlay_reason = reason

    def status(self):
        with self._lock:
            result = self._last_result or {}
            return {
                "gateway_connected": self._gateway_connected,
                "gateway_peer": self._gateway_peer,
                "received_results": self._received_results,
                "result_frame_id": result.get("frame_id"),
                "inference_ms": result.get("inference_ms"),
                "object_count": len(result.get("objects", [])),
                "source": result.get("source"),
                "gateway_roundtrip_ms": result.get("gateway_roundtrip_ms"),
                "input_capture_ts_ns": result.get("input_capture_ts_ns"),
                "overlay_accepted": self._last_overlay_accepted,
                "overlay_reason": self._last_overlay_reason,
                "result_age_seconds": (
                    None if self._updated_monotonic is None
                    else round(time.monotonic() - self._updated_monotonic, 3)
                ),
                "overlay": self.overlay.status(),
            }


class CarlaVideoTrack(VideoStreamTrack):
    kind = "video"

    def __init__(self, source):
        super().__init__()
        self.source = source
        self.sequence = 0
        self.pts = 0
        self.last_bgr = np.zeros((DISPLAY_HEIGHT, DISPLAY_WIDTH, 3), dtype=np.uint8)

    async def recv(self):
        loop = asyncio.get_running_loop()
        item = await loop.run_in_executor(None, self.source.wait_after, self.sequence, 1.0)
        if item is not None:
            self.sequence, self.last_bgr, _, _ = item
        else:
            await asyncio.sleep(1.0 / VIDEO_FPS)

        frame = av.VideoFrame.from_ndarray(self.last_bgr, format="bgr24")
        frame.pts = self.pts
        frame.time_base = Fraction(1, VIDEO_CLOCK)
        self.pts += VIDEO_STEP
        return frame


class CarlaSimulation:
    def __init__(
        self, host, port, town, source, controls, batch_cache, pose_history,
        traffic_vehicles=20, traffic_walkers=0, freeze_traffic=False,
        max_speed_kmh=30.0,
    ):
        self.host = host
        self.port = port
        self.town = town
        self.source = source
        self.controls = controls
        self.batch_cache = batch_cache
        self.pose_history = pose_history
        self.traffic_vehicles = max(0, int(traffic_vehicles))
        self.traffic_walkers = max(0, int(traffic_walkers))
        self.freeze_traffic = bool(freeze_traffic)
        self.max_speed_kmh = float(max_speed_kmh)
        self.ready = threading.Event()
        self.stopped = threading.Event()
        self.thread = None
        self.error = None
        self.client = None
        self.world = None
        self.original_settings = None
        self.traffic_manager = None
        self.ego = None
        self.camera = None
        self.camera_rig = None
        self.actor_ids = []
        self.walker_controller_ids = []
        self.frozen_vehicle_ids = []
        self.frozen_walker_ids = []
        self.spawned_vehicles = 0
        self.spawned_walkers = 0
        self.current_steer = 0.0
        self.current_view = 0
        self.speed_kmh = 0.0
        self._metrics_lock = threading.Lock()
        self._tick_ms = deque(maxlen=160)
        self._tick_wall_times = deque(maxlen=160)

    def start(self):
        self.thread = threading.Thread(target=self._run, name="carla-simulation", daemon=True)
        self.thread.start()
        if not self.ready.wait(timeout=90):
            raise RuntimeError("CARLA simulation did not become ready in 90 seconds")
        if self.error is not None:
            raise RuntimeError(f"CARLA simulation failed: {self.error}")

    def stop(self):
        self.stopped.set()
        if self.thread is not None:
            self.thread.join(timeout=15)

    def _spawn_traffic(self, library, spawn_points, tm_port):
        random.seed(20260713)
        candidates = [
            bp for bp in library.filter("vehicle.*")
            if bp.has_attribute("number_of_wheels")
            and int(bp.get_attribute("number_of_wheels")) == 4
        ]
        batch = []
        for spawn_point in spawn_points[1:1 + self.traffic_vehicles]:
            bp = random.choice(candidates)
            if bp.has_attribute("color"):
                bp.set_attribute("color", random.choice(bp.get_attribute("color").recommended_values))
            spawn = carla.command.SpawnActor(bp, spawn_point)
            if not self.freeze_traffic:
                spawn = spawn.then(
                    carla.command.SetAutopilot(
                        carla.command.FutureActor, True, tm_port
                    )
                )
            batch.append(spawn)

        results = self.client.apply_batch_sync(batch, True) if batch else []
        vehicle_ids = []
        for result in results:
            if not result.error:
                self.actor_ids.append(result.actor_id)
                vehicle_ids.append(result.actor_id)

        self.spawned_vehicles = len(vehicle_ids)
        if self.freeze_traffic and vehicle_ids:
            self.frozen_vehicle_ids.extend(vehicle_ids)

    def _spawn_walkers(self, library):
        random.seed(20260714)
        candidates = list(library.filter("walker.pedestrian.*"))
        batch = []
        walker_speeds = []

        for _ in range(self.traffic_walkers):
            location = self.world.get_random_location_from_navigation()
            if location is None:
                continue
            bp = random.choice(candidates)
            if bp.has_attribute("is_invincible"):
                bp.set_attribute("is_invincible", "false")
            speed = 1.4
            if bp.has_attribute("speed"):
                values = bp.get_attribute("speed").recommended_values
                if len(values) > 1:
                    speed = float(values[1])
            batch.append(carla.command.SpawnActor(bp, carla.Transform(location)))
            walker_speeds.append(speed)

        results = self.client.apply_batch_sync(batch, True) if batch else []
        walker_ids = []
        valid_speeds = []
        for result, speed in zip(results, walker_speeds):
            if not result.error:
                self.actor_ids.append(result.actor_id)
                walker_ids.append(result.actor_id)
                valid_speeds.append(speed)

        self.spawned_walkers = len(walker_ids)
        if not walker_ids:
            return

        if self.freeze_traffic:
            self.frozen_walker_ids.extend(walker_ids)
            return

        controller_bp = library.find("controller.ai.walker")
        controller_batch = [
            carla.command.SpawnActor(controller_bp, carla.Transform(), walker_id)
            for walker_id in walker_ids
        ]
        controller_results = self.client.apply_batch_sync(controller_batch, True)
        controller_ids = []
        controller_speeds = []
        for result, speed in zip(controller_results, valid_speeds):
            if not result.error:
                controller_ids.append(result.actor_id)
                controller_speeds.append(speed)
        self.walker_controller_ids.extend(controller_ids)

        for controller, speed in zip(
            self.world.get_actors(controller_ids), controller_speeds
        ):
            controller.start()
            destination = self.world.get_random_location_from_navigation()
            if destination is not None:
                controller.go_to_location(destination)
            controller.set_max_speed(speed)

    def _settle_and_freeze_traffic(self):
        actor_ids = self.frozen_vehicle_ids + self.frozen_walker_ids
        if not self.freeze_traffic or not actor_ids:
            return

        LOG.info(
            "Settling %d frozen traffic actors for %d ticks",
            len(actor_ids), TRAFFIC_SETTLE_TICKS,
        )
        for _ in range(TRAFFIC_SETTLE_TICKS):
            self.world.tick()

        zero_velocity = carla.Vector3D(0.0, 0.0, 0.0)
        frozen = 0
        for actor in self.world.get_actors(actor_ids):
            try:
                actor.set_target_velocity(zero_velocity)
                actor.set_target_angular_velocity(zero_velocity)
            except RuntimeError as exc:
                LOG.warning("Could not clear traffic actor %s velocity: %s", actor.id, exc)
            try:
                actor.set_simulate_physics(False)
                frozen += 1
            except RuntimeError as exc:
                LOG.warning("Could not freeze traffic actor %s: %s", actor.id, exc)
        LOG.info("Traffic settled on ground and frozen: %d/%d", frozen, len(actor_ids))

    def _apply_browser_control(self):
        state = self.controls.snapshot()
        keys = state["keys"]

        if state["view_index"] != self.current_view:
            self.current_view = state["view_index"]
            view = CAMERA_VIEWS[self.current_view]
            self.camera.set_transform(view["transform"])
            LOG.info("Camera view: %s", view["name"])

        velocity = self.ego.get_velocity()
        speed_kmh = 3.6 * math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
        self.speed_kmh = speed_kmh

        # A lost browser connection must not leave throttle or steering active.
        if state["age"] > 0.5:
            desired_steer = 0.0
            throttle = 0.0
            brake = 0.6
            hand_brake = False
        else:
            desired_steer = (-1.0 if keys["a"] else 0.0) + (1.0 if keys["d"] else 0.0)
            steer_limit = max(0.28, 0.70 - max(0.0, speed_kmh - 20.0) * 0.012)
            desired_steer *= steer_limit
            if keys["w"]:
                taper_start = max(5.0, self.max_speed_kmh - 8.0)
                throttle_scale = np.clip(
                    (self.max_speed_kmh - speed_kmh) /
                    max(1.0, self.max_speed_kmh - taper_start), 0.0, 1.0
                )
                throttle = 0.68 * float(throttle_scale)
            else:
                throttle = 0.0
            brake = 1.0 if keys["s"] else 0.0
            if speed_kmh > self.max_speed_kmh + 1.0 and not keys["s"]:
                throttle = 0.0
                brake = min(0.30, 0.08 + 0.04 * (speed_kmh - self.max_speed_kmh))
            hand_brake = keys["space"]

        steer_step = 0.075
        delta = max(-steer_step, min(steer_step, desired_steer - self.current_steer))
        self.current_steer += delta
        if desired_steer == 0.0:
            self.current_steer *= 0.72
        if abs(self.current_steer) < 0.005:
            self.current_steer = 0.0

        self.ego.apply_control(
            carla.VehicleControl(
                throttle=float(throttle),
                steer=float(self.current_steer),
                brake=float(brake),
                hand_brake=bool(hand_brake),
                reverse=bool(state["reverse"]),
                manual_gear_shift=False,
            )
        )

    def _record_tick(self, frame_id, tick_elapsed_ms):
        velocity = self.ego.get_velocity()
        speed_kmh = 3.6 * math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
        self.speed_kmh = speed_kmh
        actor_to_world = np.asarray(self.ego.get_transform().get_matrix(), dtype=np.float64)
        self.pose_history.add(frame_id, actor_to_world, speed_kmh)
        now = time.monotonic()
        with self._metrics_lock:
            self._tick_ms.append(float(tick_elapsed_ms))
            self._tick_wall_times.append(now)

    def performance_status(self):
        with self._metrics_lock:
            tick_ms = list(self._tick_ms)
            wall_times = list(self._tick_wall_times)
        sim_fps = None
        if len(wall_times) >= 2 and wall_times[-1] > wall_times[0]:
            sim_fps = (len(wall_times) - 1) / (wall_times[-1] - wall_times[0])
        return {
            "display_resolution": f"{DISPLAY_WIDTH}x{DISPLAY_HEIGHT}",
            "target_fps": VIDEO_FPS,
            "sim_fps": None if sim_fps is None else round(sim_fps, 2),
            "tick_ms_mean": None if not tick_ms else round(float(np.mean(tick_ms)), 2),
            "tick_ms_p95": None if not tick_ms else round(float(np.percentile(tick_ms, 95)), 2),
            "speed_kmh": round(self.speed_kmh, 1),
            "max_speed_kmh": self.max_speed_kmh,
            "traffic_vehicles_requested": self.traffic_vehicles,
            "traffic_vehicles_spawned": self.spawned_vehicles,
            "traffic_walkers_requested": self.traffic_walkers,
            "traffic_walkers_spawned": self.spawned_walkers,
            "traffic_frozen": self.freeze_traffic,
        }

    def _run(self):
        try:
            self.client = carla.Client(self.host, self.port)
            self.client.set_timeout(60.0)
            current_world = self.client.get_world()
            current_map = current_world.get_map().name.rsplit("/", 1)[-1]
            requested_map = self.town.rsplit("/", 1)[-1]
            if current_map == requested_map:
                self.world = current_world
                LOG.info("Reusing current CARLA world: %s", current_world.get_map().name)
            else:
                LOG.info("Loading CARLA world: %s -> %s", current_map, self.town)
                self.world = self.client.load_world(self.town)
            self.original_settings = self.world.get_settings()

            settings = self.world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 0.05
            settings.no_rendering_mode = False
            self.world.apply_settings(settings)

            tm_port = 8000
            self.traffic_manager = self.client.get_trafficmanager(tm_port)
            self.traffic_manager.set_synchronous_mode(True)
            self.traffic_manager.set_random_device_seed(20260713)
            self.traffic_manager.global_percentage_speed_difference(10.0)
            self.world.set_weather(carla.WeatherParameters.ClearNoon)

            library = self.world.get_blueprint_library()
            spawn_points = self.world.get_map().get_spawn_points()
            ego_bp = library.find("vehicle.lincoln.mkz_2017")
            ego_bp.set_attribute("role_name", "hero")
            self.ego = self.world.try_spawn_actor(ego_bp, spawn_points[0])
            if self.ego is None:
                raise RuntimeError("Could not spawn ego vehicle")
            self.actor_ids.append(self.ego.id)
            self.ego.set_autopilot(False, tm_port)
            self._spawn_traffic(library, spawn_points, tm_port)
            self._spawn_walkers(library)
            self._settle_and_freeze_traffic()
            LOG.info(
                "Traffic ready: vehicles=%d/%d walkers=%d/%d frozen=%s",
                self.spawned_vehicles, self.traffic_vehicles,
                self.spawned_walkers, self.traffic_walkers,
                self.freeze_traffic,
            )

            camera_bp = library.find("sensor.camera.rgb")
            camera_bp.set_attribute("image_size_x", str(DISPLAY_WIDTH))
            camera_bp.set_attribute("image_size_y", str(DISPLAY_HEIGHT))
            camera_bp.set_attribute("fov", str(CAMERA_FOV))
            camera_bp.set_attribute("sensor_tick", "0.05")
            camera_bp.set_attribute("gamma", "2.2")
            if camera_bp.has_attribute("motion_blur_intensity"):
                camera_bp.set_attribute("motion_blur_intensity", "0.0")

            self.camera = self.world.spawn_actor(
                camera_bp,
                CAMERA_VIEWS[0]["transform"],
                attach_to=self.ego,
                attachment_type=carla.AttachmentType.SpringArmGhost,
            )
            self.camera.listen(self.source.callback)

            # The algorithm cameras share this process and therefore the same
            # synchronous world clock as the display camera and controls.
            self.camera_rig = NuScenesCameraRig(
                self.client, self.world, self.ego, self.batch_cache
            )
            self.camera_rig.start()

            for _ in range(5):
                started = time.perf_counter()
                frame_id = self.world.tick()
                self._record_tick(frame_id, (time.perf_counter() - started) * 1000.0)

            LOG.info("CARLA ready: town=%s, ego=%s, camera=%s", self.town, self.ego.id, CAMERA_VIEWS[0]["name"])
            self.ready.set()

            next_wall_time = time.monotonic()
            while not self.stopped.is_set():
                self._apply_browser_control()
                started = time.perf_counter()
                frame_id = self.world.tick()
                self._record_tick(frame_id, (time.perf_counter() - started) * 1000.0)
                next_wall_time += 0.05
                delay = next_wall_time - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
                elif delay < -0.5:
                    next_wall_time = time.monotonic()

        except Exception as exc:
            LOG.exception("Simulation thread failed")
            self.error = exc
            self.ready.set()
        finally:
            try:
                if self.camera_rig is not None:
                    self.camera_rig.stop()
            except Exception:
                pass
            try:
                if self.camera is not None:
                    self.camera.stop()
                    self.camera.destroy()
            except Exception:
                pass
            try:
                if self.world is not None and self.walker_controller_ids:
                    for controller in self.world.get_actors(self.walker_controller_ids):
                        controller.stop()
                    commands = [
                        carla.command.DestroyActor(actor_id)
                        for actor_id in self.walker_controller_ids
                    ]
                    self.client.apply_batch_sync(commands, True)
            except Exception:
                pass
            try:
                if self.client is not None and self.actor_ids:
                    commands = [carla.command.DestroyActor(actor_id) for actor_id in self.actor_ids]
                    self.client.apply_batch_sync(commands, True)
            except Exception:
                pass
            try:
                if self.traffic_manager is not None:
                    self.traffic_manager.set_synchronous_mode(False)
                if self.world is not None and self.original_settings is not None:
                    self.world.apply_settings(self.original_settings)
            except Exception:
                pass


async def index(request):
    return web.FileResponse(ROOT / "index.html")


async def health(request):
    simulation = request.app["simulation"]
    return web.json_response({
        "ok": simulation.error is None and simulation.ready.is_set(),
        "camera_view": CAMERA_VIEWS[simulation.current_view]["name"],
        "performance": simulation.performance_status(),
        "pose_history": simulation.pose_history.status(),
        "six_camera": (
            None if simulation.camera_rig is None else simulation.camera_rig.status()
        ),
    })


async def status(request):
    simulation = request.app["simulation"]
    return web.json_response({
        "ok": simulation.error is None and simulation.ready.is_set(),
        "camera_view": CAMERA_VIEWS[simulation.current_view]["name"],
        "performance": simulation.performance_status(),
        "pose_history": simulation.pose_history.status(),
        "batch": request.app["batch_cache"].status(),
        "six_camera": (
            None if simulation.camera_rig is None else simulation.camera_rig.status()
        ),
        "edge": request.app["result_store"].status(),
    })


async def gateway(request):
    ws = web.WebSocketResponse(
        heartbeat=10.0,
        max_msg_size=64 * 1024 * 1024,
        compress=False,
    )
    await ws.prepare(request)
    peer = request.remote
    batch_cache = request.app["batch_cache"]
    result_store = request.app["result_store"]
    result_store.set_gateway(True, peer)
    LOG.info("Gateway connected: %s", peer)

    try:
        async for message in ws:
            if message.type == web.WSMsgType.TEXT:
                try:
                    command = json.loads(message.data)
                except json.JSONDecodeError:
                    await ws.send_json({"type": "error", "message": "invalid JSON"})
                    continue
                if command.get("type") != "ready":
                    await ws.send_json({"type": "error", "message": "expected ready"})
                    continue
                last_frame = int(command.get("last_frame_id", -1))
                loop = asyncio.get_running_loop()
                item = await loop.run_in_executor(None, batch_cache.wait_newer, last_frame, 2.0)
                if item is None:
                    await ws.send_json({"type": "no_data", "last_frame_id": last_frame})
                else:
                    frame_id, packet, metadata = item
                    await ws.send_bytes(packet)
                    LOG.info(
                        "Batch sent: frame=%s bytes=%s encode_ms=%s",
                        frame_id, len(packet), metadata.get("encode_ms"),
                    )
            elif message.type == web.WSMsgType.BINARY:
                try:
                    header = unpack_header(message.data)
                    if header["message_type"] != MSG_RESULT:
                        raise ProtocolError(f"expected RESULT, got {header['message_type']}")
                    result = unpack_result(message.data)
                    result_store.update(result)
                    LOG.info(
                        "Result received: frame=%s objects=%s inference_ms=%.1f source=%s",
                        result["frame_id"], len(result["objects"]),
                        result["inference_ms"], result.get("source"),
                    )
                except Exception as exc:
                    LOG.warning("Invalid gateway result: %s", exc)
                    await ws.send_json({"type": "error", "message": str(exc)})
            elif message.type == web.WSMsgType.ERROR:
                LOG.warning("Gateway websocket error: %s", ws.exception())
                break
    finally:
        result_store.set_gateway(False)
        LOG.info("Gateway disconnected: %s", peer)
    return ws


async def offer(request):
    params = await request.json()
    offer_description = RTCSessionDescription(sdp=params["sdp"], type=params["type"])
    pc = RTCPeerConnection(RTCConfiguration(iceServers=[]))
    PCS.add(pc)
    LOG.info("Peer created; total=%d", len(PCS))

    controls = request.app["controls"]
    source = request.app["source"]

    @pc.on("datachannel")
    def on_datachannel(channel):
        LOG.info("Data channel: %s", channel.label)

        @channel.on("message")
        def on_message(message):
            if isinstance(message, str):
                controls.update_json(message)

    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
        LOG.info("Peer connection state: %s", pc.connectionState)
        if pc.connectionState in ("failed", "closed"):
            await pc.close()
            PCS.discard(pc)

    await pc.setRemoteDescription(offer_description)
    sender = pc.addTrack(CarlaVideoTrack(source))

    # Prefer H.264 when the local PyAV build exposes it, retain other codecs
    # as fallback so negotiation still succeeds on different browsers.
    try:
        transceiver = next(item for item in pc.getTransceivers() if item.sender == sender)
        codecs = RTCRtpSender.getCapabilities("video").codecs
        h264 = [codec for codec in codecs if codec.mimeType.lower() == "video/h264"]
        others = [codec for codec in codecs if codec.mimeType.lower() != "video/h264"]
        if h264:
            transceiver.setCodecPreferences(h264 + others)
    except Exception:
        LOG.exception("Could not set H.264 preference; using default codec order")

    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)
    return web.json_response({"sdp": pc.localDescription.sdp, "type": pc.localDescription.type})


async def on_shutdown(app):
    await asyncio.gather(*(pc.close() for pc in list(PCS)), return_exceptions=True)
    PCS.clear()
    app["simulation"].stop()


def create_app(simulation, source, controls, batch_cache, result_store):
    app = web.Application(client_max_size=2 * 1024 * 1024)
    app["simulation"] = simulation
    app["source"] = source
    app["controls"] = controls
    app["batch_cache"] = batch_cache
    app["result_store"] = result_store
    app.router.add_get("/", index)
    app.router.add_get("/health", health)
    app.router.add_get("/status", status)
    app.router.add_get("/gateway", gateway)
    app.router.add_post("/offer", offer)
    app.on_shutdown.append(on_shutdown)
    return app


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--carla-host", default="127.0.0.1")
    parser.add_argument("--carla-port", default=2000, type=int)
    parser.add_argument("--town", default="Town03")
    parser.add_argument("--http-host", default="0.0.0.0")
    parser.add_argument("--http-port", default=8080, type=int)
    parser.add_argument("--traffic-vehicles", default=20, type=int)
    parser.add_argument("--traffic-walkers", default=0, type=int)
    parser.add_argument(
        "--freeze-traffic", action="store_true",
        help="spawn traffic vehicles and walkers as stationary actors",
    )
    parser.add_argument("--max-speed-kmh", default=30.0, type=float)
    parser.add_argument(
        "--box-visible-seconds", default=0.45, type=float,
        help=(
            "deprecated compatibility option; accepted boxes remain visible "
            "until the next accepted result"
        ),
    )
    parser.add_argument("--max-result-age-seconds", default=3.0, type=float)
    parser.add_argument("--min-score", default=0.185, type=float)
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    controls = BrowserControl()
    batch_cache = LatestBatchCache()
    pose_history = FramePoseHistory()
    overlay = DetectionOverlay(
        pose_history,
        DISPLAY_WIDTH,
        DISPLAY_HEIGHT,
        CAMERA_FOV,
        visible_seconds=args.box_visible_seconds,
        max_result_age_seconds=args.max_result_age_seconds,
        min_score=args.min_score,
    )
    source = LatestVideoFrame(overlay)
    result_store = ResultStore(overlay)
    simulation = CarlaSimulation(
        args.carla_host, args.carla_port, args.town,
        source, controls, batch_cache, pose_history,
        traffic_vehicles=args.traffic_vehicles,
        traffic_walkers=args.traffic_walkers,
        freeze_traffic=args.freeze_traffic,
        max_speed_kmh=args.max_speed_kmh,
    )
    simulation.start()

    app = create_app(simulation, source, controls, batch_cache, result_store)
    web.run_app(app, host=args.http_host, port=args.http_port, access_log=LOG)


if __name__ == "__main__":
    main()
