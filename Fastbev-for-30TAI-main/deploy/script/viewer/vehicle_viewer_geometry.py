#!/usr/bin/env python3
"""Vehicle live viewer geometry.

This mirrors deploy/src/visualize_vehicle.cpp for browser rendering:
640x480 six-camera tiles, 72 m BEV range, bottom-z boxes, and the
sandbox Vehicle camera order.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any


DISPLAY_CAMERA_ORDER = [
    "CAM_FRONT_LEFT",
    "CAM_FRONT",
    "CAM_FRONT_RIGHT",
    "CAM_BACK_LEFT",
    "CAM_BACK",
    "CAM_BACK_RIGHT",
]

WIRE_CAMERA_ORDER = [
    "CAM_FRONT",
    "CAM_FRONT_RIGHT",
    "CAM_FRONT_LEFT",
    "CAM_BACK",
    "CAM_BACK_LEFT",
    "CAM_BACK_RIGHT",
]

CAMERA_ORDER = DISPLAY_CAMERA_ORDER

IMAGE_W = 640
IMAGE_H = 480
TILE_W = 640
TILE_H = 480
BEV_SIZE = 960
BEV_RANGE_M = 72.0
WORLD_SCALE = 24.0
LAYOUT = {
    "width": 1920,
    "height": 1920,
    "camera_tile": {"width": TILE_W, "height": TILE_H},
    "bev": {"x": (TILE_W * 3 - BEV_SIZE) // 2, "y": TILE_H,
            "size": BEV_SIZE, "range_m": BEV_RANGE_M, "world_scale": WORLD_SCALE},
    "views": DISPLAY_CAMERA_ORDER,
}

EDGES_3D = [
    (0, 1), (1, 2), (2, 3), (3, 0),
    (4, 5), (5, 6), (6, 7), (7, 4),
    (0, 4), (1, 5), (2, 6), (3, 7),
]
EDGES_BEV = [(0, 1), (1, 2), (2, 3), (3, 0)]
BEV_BOTTOM = [0, 3, 7, 4]
CORNER_SIGNS = [
    (-1, -1, 0), (-1, -1, 1), (-1, 1, 1), (-1, 1, 0),
    (1, -1, 0), (1, -1, 1), (1, 1, 1), (1, 1, 0),
]


@dataclass
class CameraInfo:
    name: str
    image_path: str
    extrinsic: list[list[float]]
    intrinsic: list[list[float]]
    post_aug_inv: list[list[float]]


def _parse_floats(line: str, n: int) -> list[float]:
    vals = [float(x) for x in line.split()]
    if len(vals) < n:
        raise ValueError(f"expected {n} floats, got {len(vals)}")
    return vals[:n]


def _mat33_inv(m: list[list[float]]) -> list[list[float]]:
    a, b, c = m[0]
    d, e, f = m[1]
    g, h, i = m[2]
    det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)
    if abs(det) < 1e-12:
        raise ValueError("singular post augmentation matrix")
    inv_det = 1.0 / det
    return [
        [(e * i - f * h) * inv_det, (c * h - b * i) * inv_det, (b * f - c * e) * inv_det],
        [(f * g - d * i) * inv_det, (a * i - c * g) * inv_det, (c * d - a * f) * inv_det],
        [(d * h - e * g) * inv_det, (b * g - a * h) * inv_det, (a * e - b * d) * inv_det],
    ]


def load_camera_params(path: str | Path) -> dict[str, CameraInfo]:
    lines = []
    for raw_line in Path(path).read_text(encoding="utf-8").splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if line:
            lines.append(line)
    if not lines:
        raise ValueError(f"empty camera params file: {path}")
    count = int(lines[0])
    offset = 1
    cameras: dict[str, CameraInfo] = {}
    for _ in range(count):
        name = lines[offset]
        image_path = lines[offset + 1]
        ext = _parse_floats(lines[offset + 2], 16)
        intr = _parse_floats(lines[offset + 3], 9)
        post_rot = _parse_floats(lines[offset + 4], 9)
        post_tran = _parse_floats(lines[offset + 5], 3)
        offset += 6
        extrinsic = [ext[0:4], ext[4:8], ext[8:12], ext[12:16]]
        intrinsic = [intr[0:3], intr[3:6], intr[6:9]]
        post_aug = [
            [post_rot[0], post_rot[1], post_tran[0]],
            [post_rot[3], post_rot[4], post_tran[1]],
            [0.0, 0.0, 1.0],
        ]
        cameras[name] = CameraInfo(
            name=name,
            image_path=image_path,
            extrinsic=extrinsic,
            intrinsic=intrinsic,
            post_aug_inv=_mat33_inv(post_aug),
        )
    return cameras


def _mat44_mul_point(m: list[list[float]], p: tuple[float, float, float]) -> tuple[float, float, float]:
    x, y, z = p
    return (
        m[0][0] * x + m[0][1] * y + m[0][2] * z + m[0][3],
        m[1][0] * x + m[1][1] * y + m[1][2] * z + m[1][3],
        m[2][0] * x + m[2][1] * y + m[2][2] * z + m[2][3],
    )


def _mat33_mul_point(m: list[list[float]], p: tuple[float, float, float]) -> tuple[float, float, float]:
    x, y, z = p
    return (
        m[0][0] * x + m[0][1] * y + m[0][2] * z,
        m[1][0] * x + m[1][1] * y + m[1][2] * z,
        m[2][0] * x + m[2][1] * y + m[2][2] * z,
    )


def box_corners(obj: dict[str, Any]) -> list[tuple[float, float, float]]:
    x = float(obj.get("x", 0.0))
    y = float(obj.get("y", 0.0))
    z = float(obj.get("z", 0.0))
    length = float(obj.get("length", obj.get("dx", obj.get("l", 0.0))))
    width = float(obj.get("width", obj.get("dy", obj.get("w", 0.0))))
    height = float(obj.get("height", obj.get("dz", obj.get("h", 0.0))))
    yaw = float(obj.get("yaw", 0.0))
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    corners: list[tuple[float, float, float]] = []
    for sx, sy_sign, sz in CORNER_SIGNS:
        lx = sx * length * 0.5
        ly = sy_sign * width * 0.5
        lz = sz * height
        rx = lx * cy - ly * sy
        ry = lx * sy + ly * cy
        corners.append((x + rx, y + ry, z + lz))
    return corners


def project_corners(corners: list[tuple[float, float, float]], cam: CameraInfo) -> tuple[list[list[float]], list[bool]]:
    pts: list[list[float]] = []
    valid: list[bool] = []
    for corner in corners:
        cx, cy, cz = _mat44_mul_point(cam.extrinsic, corner)
        valid.append(cz > 0.05)
        if abs(cz) < 1e-6:
            cz = -1e-6 if cz < 0 else 1e-6
        ix, iy, iz = _mat33_mul_point(cam.intrinsic, (cx / cz, cy / cz, 1.0))
        ox, oy, _ = _mat33_mul_point(cam.post_aug_inv, (ix, iy, iz))
        pts.append([ox * (TILE_W / IMAGE_W), oy * (TILE_H / IMAGE_H)])
    return pts, valid


def bev_point(x: float, y: float) -> list[float]:
    bev = LAYOUT["bev"]
    cx = bev["x"] + (-y + BEV_RANGE_M) / (2.0 * BEV_RANGE_M) * bev["size"]
    cy = bev["y"] + (-x + BEV_RANGE_M) / (2.0 * BEV_RANGE_M) * bev["size"]
    return [cx, cy]


def _camera_offset(view: str) -> tuple[float, float]:
    idx = DISPLAY_CAMERA_ORDER.index(view)
    return (idx % 3) * TILE_W, 0.0 if idx < 3 else LAYOUT["height"] - TILE_H


def _point_in_tile(p: list[float]) -> bool:
    return 0.0 <= p[0] < TILE_W and 0.0 <= p[1] < TILE_H


def _segment_intersects_tile(a: list[float], b: list[float]) -> bool:
    if _point_in_tile(a) or _point_in_tile(b):
        return True
    min_x = min(a[0], b[0])
    max_x = max(a[0], b[0])
    min_y = min(a[1], b[1])
    max_y = max(a[1], b[1])
    return max_x >= 0.0 and min_x < TILE_W and max_y >= 0.0 and min_y < TILE_H


def _build_camera_view(view: str, corners: list[tuple[float, float, float]], cam: CameraInfo) -> dict[str, Any] | None:
    pts, valid = project_corners(corners, cam)
    if not any(valid):
        return None
    ox, oy = _camera_offset(view)
    idx = DISPLAY_CAMERA_ORDER.index(view)
    local: list[list[float]] = []
    shifted: list[list[float]] = []
    for px, py in pts:
        if idx >= 3:
            px = TILE_W - 1 - px
        local.append([px, py])
        shifted.append([px + ox, py + oy])
    if not any(valid[i] and _point_in_tile(local[i]) for i in range(len(local))):
        return None
    edges = []
    xs = []
    ys = []
    for a, b in EDGES_3D:
        if valid[a] and valid[b] and _segment_intersects_tile(local[a], local[b]):
            edges.append([shifted[a], shifted[b]])
            xs.extend([shifted[a][0], shifted[b][0]])
            ys.extend([shifted[a][1], shifted[b][1]])
    if not edges:
        return None
    clipped_bbox = [
        max(ox, min(ox + TILE_W - 1, min(xs))),
        max(oy, min(oy + TILE_H - 1, min(ys))),
        max(ox, min(ox + TILE_W - 1, max(xs))),
        max(oy, min(oy + TILE_H - 1, max(ys))),
    ]
    return {
        "view": view,
        "edges": edges,
        "bbox": [min(xs), min(ys), max(xs), max(ys)],
        "clipped_bbox": clipped_bbox,
    }


def build_frame(frame_id: str, objects: list[dict[str, Any]], camera_params: str | Path,
                live: dict[str, Any] | None = None, score_threshold: float = 0.6) -> dict[str, Any]:
    cameras = load_camera_params(camera_params)
    detections = []
    for idx, obj in enumerate(objects):
        x = float(obj.get("x", 0.0))
        y = float(obj.get("y", 0.0))
        class_id = int(obj.get("class_id", obj.get("cls", 0)))
        score = float(obj.get("score", 0.0))
        length = float(obj.get("length", obj.get("dx", obj.get("l", 0.0))))
        width = float(obj.get("width", obj.get("dy", obj.get("w", 0.0))))
        height = float(obj.get("height", obj.get("dz", obj.get("h", 0.0))))
        yaw = float(obj.get("yaw", 0.0))
        z = float(obj.get("z", 0.0))

        corners = box_corners(obj)
        bev_poly = [bev_point(corners[i][0], corners[i][1]) for i in BEV_BOTTOM]
        center = bev_point(x, y)
        head_x = (corners[0][0] + corners[4][0]) * 0.5
        head_y = (corners[0][1] + corners[4][1]) * 0.5
        heading = [
            center,
            bev_point(head_x, head_y),
        ]
        camera_views = []
        for view in DISPLAY_CAMERA_ORDER:
            if view in cameras:
                view_geom = _build_camera_view(view, corners, cameras[view])
                if view_geom is not None:
                    camera_views.append(view_geom)
        detections.append({
            "id": idx,
            "class_id": class_id,
            "score": score,
            # Vehicle model coordinates are centimeters; keep this value in cm.
            "distance": math.hypot(x, y),
            "x": x,
            "y": y,
            "z": z,
            "w": width,
            "l": length,
            "h": height,
            "yaw": yaw,
            "box": {
                "x": x,
                "y": y,
                "z": z,
                "w": width,
                "l": length,
                "h": height,
                "yaw": yaw,
            },
            "visible_cameras": [view["view"] for view in camera_views],
            "raw": f"{x:.6g} {y:.6g} {z:.6g} "
                   f"{length:.6g} {width:.6g} {height:.6g} {yaw:.6g} {class_id} {score:.6g}",
            "bev": {
                "center": center,
                "polygon": bev_poly,
                "heading": heading,
                "bbox": [
                    min(p[0] for p in bev_poly),
                    min(p[1] for p in bev_poly),
                    max(p[0] for p in bev_poly),
                    max(p[1] for p in bev_poly),
                ],
            },
            "camera_views": camera_views,
        })

    return {
        "frame_id": frame_id,
        "image_size": [LAYOUT["width"], LAYOUT["height"]],
        "layout": LAYOUT,
        "score_threshold_default": score_threshold,
        "camera_images": [
            {"view": view, "url": f"/api/live_image/{frame_id}/{view}.jpg"}
            for view in DISPLAY_CAMERA_ORDER
        ],
        "detections": detections,
        "live": live or {"mode": "vehicle_waiting", "sequence": 0},
    }
