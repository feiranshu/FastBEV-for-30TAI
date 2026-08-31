#!/usr/bin/env python3
"""PC-side FastBEV visualization.

Reads the exported camera parameter files and result text files under
deploy/io/output, then renders the same six-camera + BEV composite used by
deploy/src/visualize.cpp. Run from anywhere; paths are resolved relative to
the deploy directory by default.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Dict, Iterable, List, Tuple

import cv2
import numpy as np


VIEWS = [
    "CAM_FRONT_LEFT",
    "CAM_FRONT",
    "CAM_FRONT_RIGHT",
    "CAM_BACK_LEFT",
    "CAM_BACK",
    "CAM_BACK_RIGHT",
]

CORNER_SIGNS = np.array(
    [
        [-1.0, -1.0, 0.0],
        [-1.0, -1.0, 1.0],
        [-1.0, 1.0, 1.0],
        [-1.0, 1.0, 0.0],
        [1.0, -1.0, 0.0],
        [1.0, -1.0, 1.0],
        [1.0, 1.0, 1.0],
        [1.0, 1.0, 0.0],
    ],
    dtype=np.float32,
)

EDGES_3D = [
    (0, 1),
    (1, 2),
    (2, 3),
    (3, 0),
    (4, 5),
    (5, 6),
    (6, 7),
    (7, 4),
    (0, 4),
    (1, 5),
    (2, 6),
    (3, 7),
]
EDGES_BEV = [(0, 1), (1, 2), (2, 3), (3, 0)]
BOX_COLOR = (0, 255, 255)  # BGR yellow, matching visualize.cpp.


def _real_lines(path: Path) -> Iterable[str]:
    with path.open("r", encoding="utf-8") as f:
        for raw in f:
            line = raw.split("#", 1)[0].strip()
            if line:
                yield line


def _floats(line: str, n: int) -> np.ndarray:
    values = [float(x) for x in line.split()]
    if len(values) < n:
        raise ValueError(f"expected {n} floats, got {len(values)} in: {line}")
    return np.array(values[:n], dtype=np.float32)


def load_cameras(path: Path, deploy_dir: Path) -> Dict[str, dict]:
    lines = iter(_real_lines(path))
    count = int(next(lines))
    cameras: Dict[str, dict] = {}
    for _ in range(count):
        name = next(lines)
        img_path_raw = next(lines)
        img_path = Path(img_path_raw)
        if not img_path.is_absolute():
            img_path = (deploy_dir / img_path).resolve()

        extrinsic = _floats(next(lines), 16).reshape(4, 4)
        intrinsic = _floats(next(lines), 9).reshape(3, 3)
        post_rot = _floats(next(lines), 9).reshape(3, 3)
        post_tran = _floats(next(lines), 3)

        post_aug = np.eye(3, dtype=np.float32)
        post_aug[0, 0] = post_rot[0, 0]
        post_aug[0, 1] = post_rot[0, 1]
        post_aug[0, 2] = post_tran[0]
        post_aug[1, 0] = post_rot[1, 0]
        post_aug[1, 1] = post_rot[1, 1]
        post_aug[1, 2] = post_tran[1]

        cameras[name] = {
            "img_path": img_path,
            "extrinsic": extrinsic,
            "intrinsic": intrinsic,
            "post_aug_inv": np.linalg.inv(post_aug),
        }
    return cameras


def load_detections(path: Path) -> np.ndarray:
    rows: List[List[float]] = []
    with path.open("r", encoding="utf-8") as f:
        for raw in f:
            parts = raw.split()
            if len(parts) >= 9:
                rows.append([float(x) for x in parts[:9]])
    if not rows:
        return np.zeros((0, 9), dtype=np.float32)
    return np.array(rows, dtype=np.float32)


def compute_corners(det: np.ndarray) -> np.ndarray:
    x, y, z, w, length, h, yaw = det[:7]
    local = np.empty((8, 3), dtype=np.float32)
    local[:, 0] = CORNER_SIGNS[:, 0] * w * 0.5
    local[:, 1] = CORNER_SIGNS[:, 1] * length * 0.5
    local[:, 2] = CORNER_SIGNS[:, 2] * h

    cy = math.cos(float(yaw))
    sy = math.sin(float(yaw))
    out = np.empty_like(local)
    out[:, 0] = local[:, 0] * cy - local[:, 1] * sy + x
    out[:, 1] = local[:, 0] * sy + local[:, 1] * cy + y
    out[:, 2] = local[:, 2] + z
    return out


def project(points: np.ndarray, camera: dict) -> Tuple[np.ndarray, np.ndarray]:
    if points.size == 0:
        return np.zeros((0, 2), dtype=np.float32), np.zeros((0,), dtype=bool)

    ones = np.ones((points.shape[0], 1), dtype=np.float32)
    homo = np.concatenate([points, ones], axis=1)
    cam = homo @ camera["extrinsic"].T

    z = cam[:, 2].copy()
    valid = z > 0.5
    z[np.abs(z) < 1e-6] = np.where(z[np.abs(z) < 1e-6] < 0, -1e-6, 1e-6)
    norm = np.stack([cam[:, 0] / z, cam[:, 1] / z, np.ones_like(z)], axis=1)
    img = norm @ camera["intrinsic"].T
    out = img @ camera["post_aug_inv"].T
    return out[:, :2], valid


def depth_to_color(depth: float) -> Tuple[int, int, int]:
    gray = max(0.0, min((depth + 2.5) / 3.0, 1.0))
    level = 200.0
    palette = np.array(
        [
            [level, 0.0, level],
            [level, 0.0, 0.0],
            [level, level, 0.0],
            [0.0, level, 0.0],
            [0.0, level, level],
            [0.0, 0.0, level],
        ],
        dtype=np.float32,
    )
    if gray >= 1.0:
        rgb = palette[-1]
    else:
        rank = int(math.floor(gray * 5))
        diff = (gray - rank / 5.0) * 5.0
        rgb = palette[rank] + (palette[rank + 1] - palette[rank]) * diff
    return int(rgb[2]), int(rgb[1]), int(rgb[0])


def world_to_canvas_round(wx: float, wy: float, canvas: int, show_range: float) -> Tuple[int, int]:
    return (
        round((wx + show_range) / show_range / 2.0 * canvas),
        round((wy + show_range) / show_range / 2.0 * canvas),
    )


def world_to_canvas_trunc(wx: float, wy: float, canvas: int, show_range: float) -> Tuple[int, int]:
    return (
        int((wx + show_range) / show_range / 2.0 * canvas),
        int((wy + show_range) / show_range / 2.0 * canvas),
    )


def render_frame(camera_params: Path, result_path: Path, out_path: Path, deploy_dir: Path) -> None:
    canvas_size = 1000
    show_range = 50.0
    scale_factor = 4
    score_thresh = 0.2
    img_w = 1600
    img_h = 900

    cameras = load_cameras(camera_params, deploy_dir)
    detections = load_detections(result_path)
    corners = np.concatenate([compute_corners(det) for det in detections], axis=0) if len(detections) else np.zeros((0, 3), dtype=np.float32)

    rendered = []
    for view_name in VIEWS:
        if view_name not in cameras:
            raise KeyError(f"missing camera {view_name} in {camera_params}")
        cam = cameras[view_name]
        image = cv2.imread(str(cam["img_path"]), cv2.IMREAD_COLOR)
        if image is None:
            print(f"[warn] cannot open {cam['img_path']} -- substituting blank frame")
            image = np.zeros((img_h, img_w, 3), dtype=np.uint8)
        if image.shape[1] != img_w or image.shape[0] != img_h:
            image = cv2.resize(image, (img_w, img_h), interpolation=cv2.INTER_LINEAR)

        pixels, valid = project(corners, cam)
        for box_idx in range(len(detections)):
            base = box_idx * 8
            for a, b in EDGES_3D:
                i0 = base + a
                i1 = base + b
                if valid[i0] and valid[i1]:
                    p0 = tuple(np.rint(pixels[i0]).astype(int))
                    p1 = tuple(np.rint(pixels[i1]).astype(int))
                    cv2.line(image, p0, p1, BOX_COLOR, scale_factor, lineType=cv2.LINE_8)
        rendered.append(image)

    bev = np.zeros((canvas_size, canvas_size, 3), dtype=np.uint8)
    centre = int((0.0 + show_range) / show_range / 2.0 * canvas_size)
    cv2.circle(bev, (centre, centre), 1, (255, 255, 255), 0)
    for radius in range(10, 100, 10):
        rc = int(radius / show_range / 2.0 * canvas_size)
        cv2.circle(bev, (centre, centre), rc, depth_to_color(radius), 1)

    order = np.argsort(detections[:, 8]) if len(detections) else []
    for idx in order:
        det = detections[int(idx)]
        score = float(det[8])
        if score < score_thresh:
            continue
        intensity = min(score * 2.0, 1.0)
        color = tuple(int(c * intensity) for c in BOX_COLOR)
        c8 = compute_corners(det)
        bottom_idx = [0, 3, 7, 4]
        bottom = []
        cx_w = 0.0
        cy_w = 0.0
        for corner_idx in bottom_idx:
            wx = float(c8[corner_idx, 0])
            wy = float(-c8[corner_idx, 1])
            bottom.append(world_to_canvas_round(wx, wy, canvas_size, show_range))
            cx_w += wx
            cy_w += wy
        cx_w /= 4.0
        cy_w /= 4.0
        center_pt = world_to_canvas_trunc(cx_w, cy_w, canvas_size, show_range)
        head_pt = world_to_canvas_trunc(
            float((c8[0, 0] + c8[4, 0]) * 0.5),
            float((-c8[0, 1] + -c8[4, 1]) * 0.5),
            canvas_size,
            show_range,
        )
        for a, b in EDGES_BEV:
            cv2.line(bev, bottom[a], bottom[b], color, 1, lineType=cv2.LINE_8)
        cv2.line(bev, center_pt, head_pt, color, 1, lineType=cv2.LINE_8)

    full_h = img_h * 2 + canvas_size * scale_factor
    full_w = img_w * 3
    big = np.zeros((full_h, full_w, 3), dtype=np.uint8)
    for k in range(3):
        big[0:img_h, k * img_w : (k + 1) * img_w] = rendered[k]
    for k in range(3):
        flipped = cv2.flip(rendered[3 + k], 1)
        big[full_h - img_h : full_h, k * img_w : (k + 1) * img_w] = flipped

    out_w = img_w // scale_factor * 3
    out_h = img_h // scale_factor * 2 + canvas_size
    resized = cv2.resize(big, (out_w, out_h), interpolation=cv2.INTER_LINEAR)
    w_begin = (img_w * 3 // scale_factor - canvas_size) // 2
    y_begin = img_h // scale_factor
    resized[y_begin : y_begin + canvas_size, w_begin : w_begin + canvas_size] = bev

    out_path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(out_path), resized):
        raise RuntimeError(f"failed to write {out_path}")
    print(f"saved {out_path} ({out_w}x{out_h}), detections={len(detections)}")


def frame_id(path: Path) -> str:
    stem = path.stem
    return stem.rsplit("_", 1)[-1]


def main() -> int:
    script_path = Path(__file__).resolve()
    deploy_dir = script_path.parents[2]
    parser = argparse.ArgumentParser(description="Render FastBEV PC-side visualization PNGs.")
    parser.add_argument("--deploy-dir", type=Path, default=deploy_dir)
    parser.add_argument("--params-dir", type=Path, default=None)
    parser.add_argument("--results-dir", type=Path, default=None)
    parser.add_argument("--out-dir", type=Path, default=None)
    parser.add_argument("--frames", nargs="*", default=None, help="Frame ids such as 0001 0002. Defaults to first frame.")
    parser.add_argument("--limit", type=int, default=1, help="Render first N sorted frames when --frames is omitted.")
    args = parser.parse_args()

    deploy_dir = args.deploy_dir.resolve()
    params_dir = args.params_dir or (deploy_dir / "io" / "output" / "parameter")
    results_dir = args.results_dir or (deploy_dir / "io" / "output" / "result")
    out_dir = args.out_dir or (deploy_dir / "io" / "output" / "pc_visualize")

    params = sorted(params_dir.glob("camera_params_*.txt"))
    if args.frames:
        wanted = set(args.frames)
        params = [p for p in params if frame_id(p) in wanted]
    else:
        params = params[: max(args.limit, 0)]
    if not params:
        raise FileNotFoundError(f"no camera_params_*.txt files selected in {params_dir}")

    for param_path in params:
        fid = frame_id(param_path)
        result_path = results_dir / f"result_{fid}.txt"
        if not result_path.exists():
            print(f"[warn] missing {result_path}, skipping frame {fid}")
            continue
        render_frame(param_path, result_path, out_dir / f"vis_{fid}.png", deploy_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
