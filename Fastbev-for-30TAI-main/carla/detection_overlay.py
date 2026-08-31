import math
import threading
import time
from collections import OrderedDict

import cv2
import numpy as np


# Representative nuScenes calibrated_sensor record for LIDAR_TOP. The six
# camera records in nuscenes_rig.py come from the same calibration set.
LIDAR2EGO_TRANSLATION = np.array([0.943713, 0.0, 1.84023], dtype=np.float64)
LIDAR2EGO_QUATERNION = np.array([
    0.7077955119163518,
    -0.006492242056004365,
    0.010646214713995808,
    -0.7063073142877817,
], dtype=np.float64)

# CARLA actor origin -> representative nuScenes ego origin (rear axle).
REAR_AXLE_IN_CARLA_ACTOR = np.array([-1.4, 0.0, 0.25], dtype=np.float64)
NUSC_EGO_TO_CARLA_ACTOR = np.diag([1.0, -1.0, 1.0])

CLASS_NAMES = (
    "car", "truck", "trailer", "bus", "construction_vehicle",
    "bicycle", "motorcycle", "pedestrian", "traffic_cone", "barrier",
)
CLASS_COLORS_BGR = (
    (40, 220, 80), (20, 170, 255), (60, 120, 255), (255, 170, 20),
    (60, 220, 220), (255, 100, 180), (220, 80, 220), (40, 80, 255),
    (0, 210, 255), (180, 180, 180),
)
BOX_EDGES = (
    (0, 1), (1, 2), (2, 3), (3, 0),
    (4, 5), (5, 6), (6, 7), (7, 4),
    (0, 4), (1, 5), (2, 6), (3, 7),
)


def quaternion_matrix(q):
    w, x, y, z = np.asarray(q, dtype=np.float64)
    norm = np.linalg.norm([w, x, y, z])
    if norm == 0:
        raise ValueError("zero quaternion")
    w, x, y, z = np.array([w, x, y, z]) / norm
    return np.array([
        [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
        [2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
        [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y],
    ], dtype=np.float64)


LIDAR2EGO_ROTATION = quaternion_matrix(LIDAR2EGO_QUATERNION)


def fastbev_box_corners_lidar(obj):
    """Reproduce Fast-BEV's old LiDARInstance3DBoxes.corners convention.

    Input z is the bottom center. Positive yaw uses the row-vector rotation
    matrix implemented by rotation_3d_in_axis(axis=2) in Fast-BEV.
    """
    x, y, z = float(obj["x"]), float(obj["y"]), float(obj["z"])
    dx, dy, dz = float(obj["dx"]), float(obj["dy"]), float(obj["dz"])
    yaw = float(obj["yaw"])
    if not all(math.isfinite(v) for v in (x, y, z, dx, dy, dz, yaw)):
        raise ValueError("non-finite box value")
    if not (0.03 <= dx <= 30.0 and 0.03 <= dy <= 30.0 and 0.03 <= dz <= 15.0):
        raise ValueError(f"invalid box dimensions: {(dx, dy, dz)}")

    corners = np.array([
        [-dx/2, -dy/2, 0.0], [-dx/2, dy/2, 0.0],
        [dx/2, dy/2, 0.0], [dx/2, -dy/2, 0.0],
        [-dx/2, -dy/2, dz], [-dx/2, dy/2, dz],
        [dx/2, dy/2, dz], [dx/2, -dy/2, dz],
    ], dtype=np.float64)
    c, s = math.cos(yaw), math.sin(yaw)
    rotation_row = np.array([
        [c, -s, 0.0],
        [s, c, 0.0],
        [0.0, 0.0, 1.0],
    ], dtype=np.float64)
    corners = corners @ rotation_row
    corners += np.array([x, y, z], dtype=np.float64)
    return corners


def lidar_corners_to_world(corners_lidar, actor_to_world):
    corners_ego = (LIDAR2EGO_ROTATION @ corners_lidar.T).T + LIDAR2EGO_TRANSLATION
    corners_actor = (NUSC_EGO_TO_CARLA_ACTOR @ corners_ego.T).T
    corners_actor += REAR_AXLE_IN_CARLA_ACTOR
    homogeneous = np.concatenate(
        [corners_actor, np.ones((len(corners_actor), 1), dtype=np.float64)], axis=1
    )
    return (np.asarray(actor_to_world, dtype=np.float64) @ homogeneous.T).T[:, :3]


class FramePoseHistory:
    def __init__(self, capacity=600):
        self.capacity = capacity
        self._lock = threading.Lock()
        self._poses = OrderedDict()
        self._latest_frame = -1

    def add(self, frame_id, actor_to_world, speed_kmh):
        record = {
            "frame_id": int(frame_id),
            "actor_to_world": np.asarray(actor_to_world, dtype=np.float64).copy(),
            "recorded_monotonic": time.monotonic(),
            "recorded_wall_ns": time.time_ns(),
            "speed_kmh": float(speed_kmh),
        }
        with self._lock:
            self._poses[int(frame_id)] = record
            self._poses.move_to_end(int(frame_id))
            self._latest_frame = int(frame_id)
            while len(self._poses) > self.capacity:
                self._poses.popitem(last=False)

    def get(self, frame_id):
        with self._lock:
            record = self._poses.get(int(frame_id))
            if record is None:
                return None
            result = dict(record)
            result["actor_to_world"] = record["actor_to_world"].copy()
            return result

    def status(self):
        with self._lock:
            return {"latest_frame": self._latest_frame, "stored_poses": len(self._poses)}


class DetectionOverlay:
    def __init__(
        self,
        pose_history,
        image_width,
        image_height,
        fov_degrees,
        visible_seconds=0.45,
        max_result_age_seconds=3.0,
        min_score=0.185,
        max_boxes=60,
    ):
        self.pose_history = pose_history
        self.image_width = int(image_width)
        self.image_height = int(image_height)
        self.fov_degrees = float(fov_degrees)
        # Kept for command-line compatibility. Accepted detections now remain
        # visible until the next accepted result replaces them.
        self.visible_seconds = float(visible_seconds)
        self.max_result_age_seconds = float(max_result_age_seconds)
        self.min_score = float(min_score)
        self.max_boxes = int(max_boxes)
        self._lock = threading.Lock()
        self._boxes = []
        self._frame_id = None
        self._capture_age_ms = None
        self._accepted_results = 0
        self._dropped_no_pose = 0
        self._dropped_stale = 0
        self._invalid_boxes = 0
        self._last_reason = "waiting"

        focal = self.image_width / (
            2.0 * math.tan(math.radians(self.fov_degrees) / 2.0)
        )
        self._intrinsic = np.array([
            [focal, 0.0, self.image_width / 2.0],
            [0.0, focal, self.image_height / 2.0],
            [0.0, 0.0, 1.0],
        ], dtype=np.float64)

    def publish_result(self, result):
        now = time.monotonic()
        frame_id = int(result["frame_id"])
        pose = self.pose_history.get(frame_id)
        if pose is None:
            with self._lock:
                self._dropped_no_pose += 1
                self._last_reason = f"no pose for frame {frame_id}"
            return False, "no_pose"

        capture_age = now - pose["recorded_monotonic"]
        if capture_age > self.max_result_age_seconds:
            with self._lock:
                self._dropped_stale += 1
                self._last_reason = f"stale {capture_age:.3f}s"
            return False, "stale"

        candidates = sorted(
            result.get("objects", []), key=lambda item: float(item.get("score", 0.0)),
            reverse=True,
        )
        boxes = []
        invalid = 0
        for obj in candidates:
            score = float(obj.get("score", 0.0))
            if score <= self.min_score:
                continue
            try:
                corners_lidar = fastbev_box_corners_lidar(obj)
                corners_world = lidar_corners_to_world(
                    corners_lidar, pose["actor_to_world"]
                )
            except (KeyError, TypeError, ValueError):
                invalid += 1
                continue
            class_id = int(obj.get("class_id", -1))
            label = CLASS_NAMES[class_id] if 0 <= class_id < len(CLASS_NAMES) else f"class_{class_id}"
            color = CLASS_COLORS_BGR[class_id] if 0 <= class_id < len(CLASS_COLORS_BGR) else (255, 255, 255)
            boxes.append({
                "corners_world": corners_world,
                "label": label,
                "score": score,
                "color": color,
            })
            if len(boxes) >= self.max_boxes:
                break

        with self._lock:
            self._boxes = boxes
            self._frame_id = frame_id
            self._capture_age_ms = capture_age * 1000.0
            self._accepted_results += 1
            self._invalid_boxes += invalid
            self._last_reason = "visible" if boxes else "empty"
        return True, "accepted"

    def _snapshot(self):
        with self._lock:
            if self._frame_id is None:
                return None
            return {
                "boxes": list(self._boxes),
                "frame_id": self._frame_id,
                "capture_age_ms": self._capture_age_ms,
            }

    def draw(self, bgr, camera_transform):
        snapshot = self._snapshot()
        if snapshot is None or not snapshot["boxes"]:
            return bgr
        world_to_camera = np.asarray(
            camera_transform.get_inverse_matrix(), dtype=np.float64
        )
        height, width = bgr.shape[:2]
        canvas = (0, 0, width, height)

        for box in snapshot["boxes"]:
            corners = box["corners_world"]
            homogeneous = np.concatenate(
                [corners, np.ones((8, 1), dtype=np.float64)], axis=1
            )
            ue_camera = (world_to_camera @ homogeneous.T).T[:, :3]
            camera_xyz = np.stack(
                [ue_camera[:, 1], -ue_camera[:, 2], ue_camera[:, 0]], axis=1
            )
            depth = camera_xyz[:, 2]
            projected_h = (self._intrinsic @ camera_xyz.T).T
            projected = projected_h[:, :2] / np.maximum(
                projected_h[:, 2:3], 1e-6
            )
            color = tuple(int(v) for v in box["color"])
            drawn_points = []
            for start, end in BOX_EDGES:
                if depth[start] <= 0.20 or depth[end] <= 0.20:
                    continue
                p1 = tuple(np.rint(projected[start]).astype(int))
                p2 = tuple(np.rint(projected[end]).astype(int))
                ok, clipped1, clipped2 = cv2.clipLine(canvas, p1, p2)
                if ok:
                    cv2.line(bgr, clipped1, clipped2, color, 2, cv2.LINE_AA)
                    drawn_points.extend((clipped1, clipped2))
            if drawn_points:
                label_x = max(2, min(point[0] for point in drawn_points))
                label_y = max(18, min(point[1] for point in drawn_points) - 4)
                text = f"{box['label']} {box['score']:.2f}"
                (tw, th), _ = cv2.getTextSize(
                    text, cv2.FONT_HERSHEY_SIMPLEX, 0.48, 1
                )
                cv2.rectangle(
                    bgr, (label_x, label_y - th - 5),
                    (min(width - 1, label_x + tw + 6), label_y + 2), color, -1,
                )
                cv2.putText(
                    bgr, text, (label_x + 3, label_y - 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (10, 10, 10), 1, cv2.LINE_AA,
                )

        banner = (
            f"FAST-BEV frame {snapshot['frame_id']}  "
            f"age {snapshot['capture_age_ms']:.0f} ms"
        )
        cv2.rectangle(bgr, (8, height - 31), (310, height - 7), (12, 20, 30), -1)
        cv2.putText(
            bgr, banner, (14, height - 14), cv2.FONT_HERSHEY_SIMPLEX,
            0.48, (80, 235, 150), 1, cv2.LINE_AA,
        )
        return bgr

    def status(self):
        with self._lock:
            return {
                "result_frame_id": self._frame_id,
                "box_count": len(self._boxes),
                "visible": bool(self._boxes),
                "visible_remaining_ms": None,
                "capture_age_ms": None if self._capture_age_ms is None else round(self._capture_age_ms, 1),
                "visible_seconds": None,
                "retention_mode": "until_next_result",
                "max_result_age_seconds": self.max_result_age_seconds,
                "accepted_results": self._accepted_results,
                "dropped_no_pose": self._dropped_no_pose,
                "dropped_stale": self._dropped_stale,
                "invalid_boxes": self._invalid_boxes,
                "last_reason": self._last_reason,
            }
