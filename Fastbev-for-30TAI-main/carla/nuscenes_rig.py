import math
import queue
import threading
import time
from concurrent.futures import ThreadPoolExecutor

import carla
import cv2
import numpy as np

from protocol import pack_batch


cv2.setNumThreads(1)

TARGET_W, TARGET_H = 1600, 900
RAW_W, RAW_H = 1728, 1028
JPEG_QUALITY = 90
REAR_AXLE_IN_CARLA_ACTOR = np.array([-1.4, 0.0, 0.25], dtype=np.float64)

CAMERAS = {
    "CAM_FRONT": {
        "translation": [1.70079118954, 0.0159456324149, 1.51095763913],
        "rotation": [0.4998015430569128, -0.5030316162024876, 0.4997798114386805, -0.49737083824542755],
        "K": [[1266.417203046554, 0.0, 816.2670197447984], [0.0, 1266.417203046554, 491.50706579294757], [0.0, 0.0, 1.0]],
    },
    "CAM_FRONT_RIGHT": {
        "translation": [1.5508477543, -0.493404796419, 1.49574800619],
        "rotation": [0.2060347966337182, -0.2026940577919598, 0.6824507824531167, -0.6713610884174485],
        "K": [[1260.8474446004698, 0.0, 807.968244525554], [0.0, 1260.8474446004698, 495.3344268742088], [0.0, 0.0, 1.0]],
    },
    "CAM_BACK_RIGHT": {
        "translation": [1.0148780988, -0.480568219723, 1.56239545128],
        "rotation": [0.12280980120078765, -0.132400842670559, -0.7004305821388234, 0.690496031265798],
        "K": [[1259.5137405846733, 0.0, 807.2529053838625], [0.0, 1259.5137405846733, 501.19579884916527], [0.0, 0.0, 1.0]],
    },
    "CAM_BACK": {
        "translation": [0.0283260309358, 0.00345136761476, 1.57910346144],
        "rotation": [0.5037872666382278, -0.49740249788611096, -0.4941850223835201, 0.5045496097725578],
        "K": [[809.2209905677063, 0.0, 829.2196003259838], [0.0, 809.2209905677063, 481.77842384512485], [0.0, 0.0, 1.0]],
    },
    "CAM_BACK_LEFT": {
        "translation": [1.03569100218, 0.484795032713, 1.59097014818],
        "rotation": [0.6924185592174665, -0.7031619420114925, -0.11648342771943819, 0.11203317912370753],
        "K": [[1256.7414812095406, 0.0, 792.1125740759628], [0.0, 1256.7414812095406, 492.7757465151356], [0.0, 0.0, 1.0]],
    },
    "CAM_FRONT_LEFT": {
        "translation": [1.52387798135, 0.494631336551, 1.50932822144],
        "rotation": [0.6757265034669446, -0.6736266522251881, 0.21214015046209478, -0.21122827103904068],
        "K": [[1272.5979470598488, 0.0, 826.6154927353808], [0.0, 1272.5979470598488, 479.75165386361925], [0.0, 0.0, 1.0]],
    },
}

CAMERA_ORDER = [
    "CAM_FRONT", "CAM_FRONT_RIGHT", "CAM_BACK_RIGHT",
    "CAM_BACK", "CAM_BACK_LEFT", "CAM_FRONT_LEFT",
]


def quaternion_matrix(q):
    w, x, y, z = np.asarray(q, dtype=np.float64)
    w, x, y, z = np.array([w, x, y, z]) / np.linalg.norm([w, x, y, z])
    return np.array([
        [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
        [2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
        [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y],
    ])


def nuscenes_to_carla_transform(translation, quaternion):
    local_basis = np.array([[0, 1, 0], [0, 0, -1], [1, 0, 0]], dtype=np.float64)
    nusc_to_carla_ego = np.diag([1.0, -1.0, 1.0])
    rotation = nusc_to_carla_ego @ quaternion_matrix(quaternion) @ local_basis
    pitch = math.asin(float(np.clip(rotation[2, 0], -1.0, 1.0)))
    yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    roll = math.atan2(-rotation[2, 1], rotation[2, 2])
    t = np.asarray(translation, dtype=np.float64)
    location = REAR_AXLE_IN_CARLA_ACTOR + np.array([t[0], -t[1], t[2]])
    return carla.Transform(
        carla.Location(x=float(location[0]), y=float(location[1]), z=float(location[2])),
        carla.Rotation(pitch=math.degrees(pitch), yaw=math.degrees(yaw), roll=math.degrees(roll)),
    )


class LatestBatchCache:
    def __init__(self):
        self._condition = threading.Condition()
        self._frame_id = -1
        self._packet = None
        self._metadata = {}

    def publish(self, frame_id, packet, metadata):
        with self._condition:
            if frame_id <= self._frame_id:
                return
            self._frame_id = frame_id
            self._packet = packet
            self._metadata = dict(metadata)
            self._condition.notify_all()

    def wait_newer(self, last_frame_id, timeout=2.0):
        deadline = time.monotonic() + timeout
        with self._condition:
            while self._packet is None or self._frame_id <= last_frame_id:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._condition.wait(remaining)
            return self._frame_id, self._packet, dict(self._metadata)

    def status(self):
        with self._condition:
            return {"frame_id": self._frame_id, **self._metadata}


class SixCameraSynchronizer:
    def __init__(self, camera_names):
        self.camera_names = tuple(camera_names)
        self._lock = threading.Lock()
        self._pending = {}
        self._complete = queue.Queue(maxsize=1)
        self.received = {name: 0 for name in camera_names}
        self.dropped_incomplete = 0
        self.dropped_complete = 0

    def callback_for(self, camera_name):
        def callback(image):
            completed = None
            with self._lock:
                self.received[camera_name] += 1
                bucket = self._pending.setdefault(image.frame, {})
                bucket[camera_name] = image
                if len(bucket) == len(self.camera_names):
                    completed = (
                        image.frame,
                        time.time_ns(),
                        {name: bucket[name] for name in self.camera_names},
                    )
                    del self._pending[image.frame]
                    for old_frame in [frame for frame in self._pending if frame < image.frame]:
                        del self._pending[old_frame]
                        self.dropped_incomplete += 1
                while len(self._pending) > 20:
                    del self._pending[min(self._pending)]
                    self.dropped_incomplete += 1
            if completed is not None:
                try:
                    self._complete.put_nowait(completed)
                except queue.Full:
                    try:
                        self._complete.get_nowait()
                        self.dropped_complete += 1
                    except queue.Empty:
                        pass
                    self._complete.put_nowait(completed)
        return callback

    def get(self, timeout=0.5):
        return self._complete.get(timeout=timeout)

    def status(self):
        with self._lock:
            return {
                "received": dict(self.received),
                "pending_frames": len(self._pending),
                "dropped_incomplete": self.dropped_incomplete,
                "dropped_complete": self.dropped_complete,
            }


class NuScenesCameraRig:
    def __init__(self, client, world, ego, batch_cache):
        self.client = client
        self.world = world
        self.ego = ego
        self.batch_cache = batch_cache
        self.synchronizer = SixCameraSynchronizer(CAMERA_ORDER)
        self.sensors = []
        self.stop_event = threading.Event()
        self.worker = None
        self.encode_pool = None
        self.encoded_batches = 0
        self.last_encode_ms = None
        self.last_batch_bytes = None

    def start(self):
        library = self.world.get_blueprint_library()
        commands = []
        for name in CAMERA_ORDER:
            spec = CAMERAS[name]
            fx = float(spec["K"][0][0])
            raw_fov = math.degrees(2.0 * math.atan(RAW_W / (2.0 * fx)))
            bp = library.find("sensor.camera.rgb")
            bp.set_attribute("image_size_x", str(RAW_W))
            bp.set_attribute("image_size_y", str(RAW_H))
            bp.set_attribute("fov", f"{raw_fov:.10f}")
            bp.set_attribute("sensor_tick", "0.5")
            bp.set_attribute("gamma", "2.2")
            transform = nuscenes_to_carla_transform(spec["translation"], spec["rotation"])
            commands.append(carla.command.SpawnActor(bp, transform, self.ego.id))

        results = self.client.apply_batch_sync(commands, True)
        for name, result in zip(CAMERA_ORDER, results):
            if result.error:
                raise RuntimeError(f"failed to spawn {name}: {result.error}")
            sensor = self.world.get_actor(result.actor_id)
            if sensor is None:
                raise RuntimeError(f"missing sensor actor for {name}")
            self.sensors.append(sensor)
            sensor.listen(self.synchronizer.callback_for(name))

        self.encode_pool = ThreadPoolExecutor(
            max_workers=len(CAMERA_ORDER), thread_name_prefix="jpeg"
        )
        self.worker = threading.Thread(target=self._encode_loop, name="six-camera-jpeg", daemon=True)
        self.worker.start()

    @staticmethod
    def _image_to_cropped_bgr(image, intrinsic):
        bgra = np.frombuffer(image.raw_data, dtype=np.uint8).reshape(image.height, image.width, 4)
        cx, cy = float(intrinsic[0][2]), float(intrinsic[1][2])
        x0 = int(round(RAW_W / 2.0 - cx))
        y0 = int(round(RAW_H / 2.0 - cy))
        crop = bgra[y0:y0 + TARGET_H, x0:x0 + TARGET_W, :3]
        if crop.shape[:2] != (TARGET_H, TARGET_W):
            raise RuntimeError(f"bad camera crop: {crop.shape}")
        return np.ascontiguousarray(crop)

    def _encode_loop(self):
        while not self.stop_event.is_set():
            try:
                frame_id, capture_ts_ns, images = self.synchronizer.get(timeout=0.5)
            except queue.Empty:
                continue
            started = time.perf_counter()
            jpeg_by_camera = {}
            try:
                futures = {}
                for name in CAMERA_ORDER:
                    bgr = self._image_to_cropped_bgr(images[name], CAMERAS[name]["K"])
                    futures[name] = self.encode_pool.submit(
                        cv2.imencode,
                        ".jpg",
                        bgr,
                        [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY],
                    )
                for name in CAMERA_ORDER:
                    ok, encoded = futures[name].result()
                    if not ok:
                        raise RuntimeError(f"JPEG encode failed: {name}")
                    jpeg_by_camera[name] = encoded.tobytes()
                packet = pack_batch(frame_id, capture_ts_ns, CAMERA_ORDER, jpeg_by_camera)
                encode_ms = (time.perf_counter() - started) * 1000.0
                self.encoded_batches += 1
                self.last_encode_ms = encode_ms
                self.last_batch_bytes = len(packet)
                self.batch_cache.publish(frame_id, packet, {
                    "capture_ts_ns": capture_ts_ns,
                    "encode_ms": round(encode_ms, 2),
                    "bytes": len(packet),
                    "encoded_batches": self.encoded_batches,
                })
            except Exception as exc:
                print(f"[ERROR] six-camera encode frame={frame_id}: {exc}", flush=True)

    def status(self):
        return {
            "encoded_batches": self.encoded_batches,
            "last_encode_ms": self.last_encode_ms,
            "last_batch_bytes": self.last_batch_bytes,
            "sync": self.synchronizer.status(),
        }

    def stop(self):
        self.stop_event.set()
        if self.worker is not None:
            self.worker.join(timeout=3)
        if self.encode_pool is not None:
            self.encode_pool.shutdown(wait=False)
        for sensor in self.sensors:
            try:
                sensor.stop()
                sensor.destroy()
            except Exception:
                pass
        self.sensors.clear()
