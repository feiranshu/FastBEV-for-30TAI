#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FastBEV nuScenes dataset preprocessing + lidar2img injection
============================================================

This single script merges the original two-step workflow:
  1) preprocess_nuscenes.py            -> generate dataset_info.json and copy images
  2) inject_lidar2img_nuscenes.py      -> inject precise lidar2img matrices

Typical usage:
    python preprocess_nuscenes_merged.py \
        --dataroot ../nuscenes \
        --output   ../fastbev_data

Output:
    fastbev_data/
    ├── dataset_info.json      # includes camera calibration, affine params and lidar2img
    └── images/                # flat image directory named by sample_data_token

Optional modes:
    # Run unit tests only
    python preprocess_nuscenes_merged.py --test

    # Only generate JSON, do not copy images
    python preprocess_nuscenes_merged.py --dataroot ../nuscenes --output ../fastbev_data --no-copy

    # Inject lidar2img into an existing dataset_info.json, compatible with the old script
    python preprocess_nuscenes_merged.py \
        --dataroot ../nuscenes \
        --json     ../fastbev_data/dataset_info.json \
        --out      ../fastbev_data/dataset_info.json

Dependencies:
    numpy
"""

from __future__ import annotations

import argparse
import bisect
import io
import json
import math
import os
import pickle
import shutil
import sys
import types
import zipfile
from collections import defaultdict
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional

import numpy as np


# ─────────────────────────────────────────────────────────────────────────────
# Constants
# ─────────────────────────────────────────────────────────────────────────────

CAMERAS = [
    'CAM_FRONT',
    'CAM_FRONT_RIGHT',
    'CAM_FRONT_LEFT',
    'CAM_BACK',
    'CAM_BACK_LEFT',
    'CAM_BACK_RIGHT',
]

LIDAR_CHANNEL = 'LIDAR_TOP'
INDEX_CHANNELS = CAMERAS + [LIDAR_CHANNEL]

DEFAULT_HISTORY_STEPS = [1, 3, 5]       # FastBEV f4 default: previous 1/3/5 camera sweeps
MAX_TS_DIFF_US = 150_000                # cross-sensor timestamp tolerance: 150 ms

# BEV feature grid used by FastBEV temporal alignment.
BEV_X_MIN = -51.2
BEV_Y_MIN = -51.2
BEV_DX = 0.512
BEV_DY = 0.512
BEV_WIDTH = 200
BEV_HEIGHT = 200

# Index-affine identity, order: [a, b, tx, c, d, ty].
IDENTITY_AFFINE = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0]

# CUDA-FastBEV image augmentation baked into lidar2img.
POST_AUG = np.array([
    [0.44, 0.0,   0.0],
    [0.0,  0.44, -70.0],
    [0.0,  0.0,   1.0],
], dtype=np.float64)


# ─────────────────────────────────────────────────────────────────────────────
# JSON / metadata helpers
# ─────────────────────────────────────────────────────────────────────────────

def load_json(path: Path) -> Any:
    with path.open('r', encoding='utf-8') as f:
        return json.load(f)


def write_json(path: Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', encoding='utf-8') as f:
        json.dump(data, f, ensure_ascii=False, indent=2)


def build_map(records: Iterable[dict], key: str = 'token') -> Dict[str, dict]:
    return {r[key]: r for r in records}


def resolve_version_dir(dataroot: Path, version: Optional[str] = None) -> Path:
    """Return the nuScenes metadata directory."""
    if version:
        vdir = dataroot / version
        if not vdir.exists():
            raise FileNotFoundError(f'未找到 {vdir}')
        return vdir

    for name in ('v1.0-mini', 'v1.0-trainval', 'v1.0-test', '.'):
        candidate = dataroot / name
        if (candidate / 'sample.json').is_file() and (candidate / 'sample_data.json').is_file():
            return candidate

    raise FileNotFoundError(
        f'在 {dataroot} 下找不到 nuScenes 元数据目录，已尝试 v1.0-mini/v1.0-trainval/v1.0-test/.')


def load_nuscenes_metadata(dataroot: Path, version: Optional[str] = None) -> dict:
    vdir = resolve_version_dir(dataroot, version)
    print(f'加载 nuScenes 元数据: {vdir}')

    tables = {
        'samples': load_json(vdir / 'sample.json'),
        'sample_data': load_json(vdir / 'sample_data.json'),
        'ego_pose': load_json(vdir / 'ego_pose.json'),
        'calibrated_sensor': load_json(vdir / 'calibrated_sensor.json'),
        'sensor': load_json(vdir / 'sensor.json'),
        'scene': load_json(vdir / 'scene.json'),
    }
    maps = {
        'sample_map': build_map(tables['samples']),
        'sd_map': build_map(tables['sample_data']),
        'ego_map': build_map(tables['ego_pose']),
        'cal_map': build_map(tables['calibrated_sensor']),
        'sensor_map': build_map(tables['sensor']),
        'scene_map': build_map(tables['scene']),
    }
    return {'version_dir': vdir, **tables, **maps}


def _scene_samples(scene: dict, sample_map: Dict[str, dict]) -> Iterable[dict]:
    """Yield all samples in a scene in temporal order."""
    tok = scene['first_sample_token']
    while tok:
        sample = sample_map[tok]
        yield sample
        tok = sample.get('next') or ''


# ─────────────────────────────────────────────────────────────────────────────
# Affine transform helpers
# ─────────────────────────────────────────────────────────────────────────────

def quat_to_yaw(quat: List[float]) -> float:
    """Extract yaw from nuScenes quaternion [w, x, y, z].

    Kept for compatibility with older notes/tests. Temporal affine now uses the
    full 4x4 pose path below rather than yaw-only SE(2).
    """
    w, x, y, z = quat
    norm = math.sqrt(w*w + x*x + y*y + z*z)
    w, x, y, z = w/norm, x/norm, y/norm, z/norm
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


# ─────────────────────────────────────────────────────────────────────────────
# lidar2img helpers
# ─────────────────────────────────────────────────────────────────────────────

def quat_to_rotmat(q: List[float]) -> np.ndarray:
    """nuScenes quaternion [w, x, y, z] -> 3x3 rotation matrix."""
    w, x, y, z = q
    n = (w*w + x*x + y*y + z*z) ** 0.5
    w, x, y, z = w/n, x/n, y/n, z/n
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y)],
        [2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y)],
    ], dtype=np.float64)


def make_transform(rotation: List[float], translation: List[float]) -> np.ndarray:
    """rotation + translation -> 4x4 homogeneous transform."""
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = quat_to_rotmat(rotation)
    T[:3, 3] = translation
    return T


def extract_physical_affine(T_hist_from_cur: np.ndarray) -> List[float]:
    """Extract physical BEV affine from a full 4x4 current->history transform.

    Return order: [a, b, c, d, tx, ty]
      x_hist = a * x_cur + b * y_cur + tx
      y_hist = c * x_cur + d * y_cur + ty
    """
    return [
        float(T_hist_from_cur[0, 0]),
        float(T_hist_from_cur[0, 1]),
        float(T_hist_from_cur[1, 0]),
        float(T_hist_from_cur[1, 1]),
        float(T_hist_from_cur[0, 3]),
        float(T_hist_from_cur[1, 3]),
    ]


def physical_to_index_affine(
    physical: List[float],
    x_min: float = BEV_X_MIN,
    y_min: float = BEV_Y_MIN,
    dx: float = BEV_DX,
    dy: float = BEV_DY,
) -> List[float]:
    """Convert physical affine [a,b,c,d,tx,ty] to index affine [a,b,c,d,tx,ty].

    Physical:
      x_hist = a*x_cur + b*y_cur + tx
      y_hist = c*x_cur + d*y_cur + ty

    Index:
      u = a_idx*j + b_idx*i + tx_idx
      v = c_idx*j + d_idx*i + ty_idx

    where j is the BEV column index and i is the BEV row index.
    """
    a, b, c, d, tx, ty = physical

    a_idx = a
    b_idx = b * dy / dx
    tx_idx = (a * x_min + b * y_min + tx - x_min) / dx

    c_idx = c * dx / dy
    d_idx = d
    ty_idx = (c * x_min + d * y_min + ty - y_min) / dy

    return [
        float(a_idx),
        float(b_idx),
        float(c_idx),
        float(d_idx),
        float(tx_idx),
        float(ty_idx),
    ]


def compute_affine_params(cur_trans, cur_rot, hist_trans, hist_rot) -> List[float]:
    """Compute current->history BEV index affine using full 4x4 ego poses.

    Return order: [a, b, tx, c, d, ty]  (row-major 2x3 layout)
      u = a*j + b*i + tx
      v = c*j + d*i + ty

    This is ready for FPGA/CUDA inverse-warp sampling on BEV feature indices.
    """
    T_cur = make_transform(cur_rot, cur_trans)
    T_hist = make_transform(hist_rot, hist_trans)
    T_hist_from_cur = np.linalg.inv(T_hist) @ T_cur
    physical = extract_physical_affine(T_hist_from_cur)
    a, b, c, d, tx, ty = physical_to_index_affine(physical)
    return [a, b, tx, c, d, ty]


def compute_lidar2img(
    lidar_cs_record: dict,
    lidar_ego_record: dict,
    cam_cs_record: dict,
    cam_ego_record: dict,
) -> np.ndarray:
    """
    Compute one camera's 4x4 lidar2img matrix.

    Chain:
      lidar -> ego(lidar_ts) -> global -> ego(cam_ts) -> camera -> image -> augmented
    """
    T_lidar2ego = make_transform(lidar_cs_record['rotation'], lidar_cs_record['translation'])
    T_ego2global_lidar = make_transform(lidar_ego_record['rotation'], lidar_ego_record['translation'])

    T_ego2global_cam = make_transform(cam_ego_record['rotation'], cam_ego_record['translation'])
    T_global2ego_cam = np.linalg.inv(T_ego2global_cam)

    T_cam2ego = make_transform(cam_cs_record['rotation'], cam_cs_record['translation'])
    T_ego2cam = np.linalg.inv(T_cam2ego)

    T_lidar2cam = T_ego2cam @ T_global2ego_cam @ T_ego2global_lidar @ T_lidar2ego

    K = np.array(cam_cs_record['camera_intrinsic'], dtype=np.float64)
    E_raw = K @ T_lidar2cam[:3, :]
    E_top3 = POST_AUG @ E_raw

    E = np.zeros((4, 4), dtype=np.float64)
    E[:3, :] = E_top3
    E[3, 3] = 1.0
    return E


def compute_lidar2img_from_sd(lidar_sd: dict, cam_sd: dict, cal_map: Dict[str, dict], ego_map: Dict[str, dict]) -> List[List[float]]:
    lidar_cs = cal_map[lidar_sd['calibrated_sensor_token']]
    lidar_ego = ego_map[lidar_sd['ego_pose_token']]
    cam_cs = cal_map[cam_sd['calibrated_sensor_token']]
    cam_ego = ego_map[cam_sd['ego_pose_token']]
    return compute_lidar2img(lidar_cs, lidar_ego, cam_cs, cam_ego).tolist()


# ─────────────────────────────────────────────────────────────────────────────
# Index builders
# ─────────────────────────────────────────────────────────────────────────────

def build_channel_indexes(meta: dict):
    cal_map = meta['cal_map']
    sensor_map = meta['sensor_map']
    sd_list = meta['sample_data']

    cal_to_channel = {
        cs['token']: sensor_map[cs['sensor_token']]['channel']
        for cs in cal_map.values()
    }

    ch_sds: Dict[str, List[dict]] = defaultdict(list)
    for sd in sd_list:
        ch = cal_to_channel.get(sd['calibrated_sensor_token'], '')
        if ch in INDEX_CHANNELS:
            ch_sds[ch].append(sd)

    for ch in INDEX_CHANNELS:
        ch_sds[ch].sort(key=lambda s: s['timestamp'])

    ch_ts = {ch: [s['timestamp'] for s in ch_sds[ch]] for ch in INDEX_CHANNELS}

    # sample_token -> {channel: key-frame sample_data record}, includes cameras and LIDAR_TOP.
    sample_key_data: Dict[str, Dict[str, dict]] = defaultdict(dict)
    for sd in sd_list:
        if not sd.get('is_key_frame', False):
            continue
        ch = cal_to_channel.get(sd['calibrated_sensor_token'], '')
        if ch in INDEX_CHANNELS:
            sample_key_data[sd['sample_token']][ch] = sd

    return cal_to_channel, ch_sds, ch_ts, sample_key_data


def nearest_sd(channel: str, target_ts: int, ch_sds: Dict[str, List[dict]], ch_ts: Dict[str, List[int]],
               before_ts: Optional[int] = None, max_diff_us: int = MAX_TS_DIFF_US) -> Optional[dict]:
    """Find the nearest sample_data in one channel, optionally restricted to timestamps before before_ts."""
    ts_list = ch_ts.get(channel, [])
    sds = ch_sds.get(channel, [])
    if not ts_list:
        return None

    if before_ts is not None:
        hi_idx = bisect.bisect_left(ts_list, before_ts)
        if hi_idx == 0:
            return None
        ts_list = ts_list[:hi_idx]
        sds = sds[:hi_idx]

    idx = bisect.bisect_left(ts_list, target_ts)
    candidates = []
    if idx < len(sds):
        candidates.append(sds[idx])
    if idx > 0:
        candidates.append(sds[idx - 1])
    if not candidates:
        return None

    best = min(candidates, key=lambda s: abs(s['timestamp'] - target_ts))
    if abs(best['timestamp'] - target_ts) > max_diff_us:
        return None
    return best


def _pad_frame(t_idx: int, timestamp: int, ego: dict, cam_block: dict) -> dict:
    """Pad missing history with current frame and identity affine."""
    return {
        'frame_index': t_idx,
        'timestamp': timestamp,
        'is_key_frame': True,
        'ego_pose': ego,
        'affine_params': IDENTITY_AFFINE,
        'cameras': cam_block,
    }


def parse_history_steps(value: str) -> List[int]:
    try:
        steps = [int(x.strip()) for x in value.split(',') if x.strip()]
    except ValueError as exc:
        raise argparse.ArgumentTypeError('history steps 必须是逗号分隔的正整数，例如 1,3,5') from exc
    if not steps or any(s <= 0 for s in steps):
        raise argparse.ArgumentTypeError('history steps 必须是正整数，例如 1,3,5')
    if sorted(set(steps)) != steps:
        raise argparse.ArgumentTypeError('history steps 必须升序且不能重复，例如 1,3,5')
    return steps


# ─────────────────────────────────────────────────────────────────────────────
# Main preprocessing flow
# ─────────────────────────────────────────────────────────────────────────────

def preprocess(
    dataroot: str,
    output: str,
    copy_images: bool = True,
    version: Optional[str] = None,
    history_steps: Optional[List[int]] = None,
    inject_lidar2img: bool = True,
    max_ts_diff_us: int = MAX_TS_DIFF_US,
    verify_pth: Optional[str] = None,
) -> dict:
    dataroot_path = Path(dataroot)
    outdir = Path(output)
    img_dir = outdir / 'images'
    img_dir.mkdir(parents=True, exist_ok=True)

    history_steps = history_steps or DEFAULT_HISTORY_STEPS

    print('[ 1/5 ] 加载 nuScenes 元数据...')
    meta = load_nuscenes_metadata(dataroot_path, version)
    sample_map = meta['sample_map']
    sd_map = meta['sd_map']
    ego_map = meta['ego_map']
    cal_map = meta['cal_map']
    scene_list = meta['scene']
    scene_map = meta['scene_map']

    print('[ 2/5 ] 建立索引结构...')
    _cal_to_channel, ch_sds, ch_ts, sample_key_data = build_channel_indexes(meta)

    copied = set()
    warn_missing_lidar = 0
    warn_missing_lidar_hist = 0

    def copy_image(sd_record: dict) -> str:
        token = sd_record['token']
        ext = Path(sd_record['filename']).suffix or '.jpg'
        rel = f'images/{token}{ext}'
        if copy_images and token not in copied:
            src = dataroot_path / sd_record['filename']
            dst = outdir / rel
            if src.exists():
                shutil.copy2(src, dst)
                copied.add(token)
            else:
                print(f'  [警告] 图像文件不存在: {src}')
        return rel

    def ego_from_sd(sd_record: dict) -> dict:
        ep = ego_map[sd_record['ego_pose_token']]
        return {'translation': ep['translation'], 'rotation': ep['rotation']}

    def build_cam_block(ch_sd_dict: Dict[str, dict], lidar_sd: Optional[dict]) -> dict:
        block = {}
        for ch in CAMERAS:
            sd = ch_sd_dict.get(ch)
            if sd is None:
                continue
            cal = cal_map[sd['calibrated_sensor_token']]
            item = {
                'image_path': copy_image(sd),
                'sample_data_token': sd['token'],
                'timestamp': sd['timestamp'],
                'calibration': {
                    'translation': cal['translation'],
                    'rotation': cal['rotation'],
                    'camera_intrinsic': cal['camera_intrinsic'],
                },
            }
            if inject_lidar2img and lidar_sd is not None:
                item['lidar2img'] = compute_lidar2img_from_sd(lidar_sd, sd, cal_map, ego_map)
            block[ch] = item
        return block

    def find_history_lidar(ref_ts: int, cur_ts: int, fallback_sample_token: str, current_lidar_sd: Optional[dict]) -> Optional[dict]:
        # Prefer a lidar sweep close to the historical camera timestamp; fallback to the key-frame LIDAR_TOP
        # associated with that sample, then to the current frame's lidar.
        hist_lidar = nearest_sd(
            LIDAR_CHANNEL, ref_ts, ch_sds, ch_ts, before_ts=cur_ts, max_diff_us=max_ts_diff_us)
        if hist_lidar is not None:
            return hist_lidar
        key_lidar = sample_key_data.get(fallback_sample_token, {}).get(LIDAR_CHANNEL)
        return key_lidar or current_lidar_sd

    print('[ 3/5 ] 处理样本、历史帧与 lidar2img...')
    all_records = []
    skipped = 0
    total = sum(len(list(_scene_samples(scene_map[sc['token']], sample_map))) for sc in scene_list)

    for scene in scene_list:
        ordered = list(_scene_samples(scene, sample_map))

        for si, sample in enumerate(ordered):
            sample_token = sample['token']
            key_data = sample_key_data.get(sample_token, {})
            cur_cams = {ch: key_data[ch] for ch in CAMERAS if ch in key_data}

            if len(cur_cams) < len(CAMERAS):
                skipped += 1
                continue

            front_sd = cur_cams['CAM_FRONT']
            cur_ego = ego_from_sd(front_sd)
            cur_t = cur_ego['translation']
            cur_r = cur_ego['rotation']
            cur_ts = sample['timestamp']

            cur_lidar_sd = key_data.get(LIDAR_CHANNEL)
            if inject_lidar2img and cur_lidar_sd is None:
                warn_missing_lidar += 1

            cur_cam_block = build_cam_block(cur_cams, cur_lidar_sd)

            # Build the CAM_FRONT prev-chain once, then sample the configured steps (default 1/3/5).
            prev_by_step: Dict[int, dict] = {}
            walk_sd = front_sd
            for step in range(1, max(history_steps) + 1):
                prev_tok = walk_sd.get('prev', '')
                if not prev_tok or prev_tok not in sd_map:
                    break
                walk_sd = sd_map[prev_tok]
                prev_by_step[step] = walk_sd

            temporal = []
            for t_idx, step in enumerate(history_steps):
                hist_front_ref = prev_by_step.get(step)
                if hist_front_ref is None:
                    temporal.append(_pad_frame(t_idx, cur_ts, cur_ego, cur_cam_block))
                    continue

                ref_ts = hist_front_ref['timestamp']

                hist_ch_sds = {}
                for ch in CAMERAS:
                    hist_sd = nearest_sd(ch, ref_ts, ch_sds, ch_ts, before_ts=cur_ts, max_diff_us=max_ts_diff_us)
                    if hist_sd is not None:
                        hist_ch_sds[ch] = hist_sd

                if 'CAM_FRONT' not in hist_ch_sds:
                    temporal.append(_pad_frame(t_idx, cur_ts, cur_ego, cur_cam_block))
                    continue

                hist_ego = ego_from_sd(hist_ch_sds['CAM_FRONT'])
                params = compute_affine_params(cur_t, cur_r, hist_ego['translation'], hist_ego['rotation'])

                hist_lidar_sd = find_history_lidar(ref_ts, cur_ts, hist_front_ref.get('sample_token', ''), cur_lidar_sd)
                if inject_lidar2img and hist_lidar_sd is None:
                    warn_missing_lidar_hist += 1

                temporal.append({
                    'frame_index': t_idx,
                    'history_step': step,
                    'timestamp': ref_ts,
                    'is_key_frame': hist_front_ref.get('is_key_frame', False),
                    'ego_pose': hist_ego,
                    'affine_params': params,
                    'cameras': build_cam_block(hist_ch_sds, hist_lidar_sd),
                })

            all_records.append({
                'index': len(all_records),
                'sample_token': sample_token,
                'scene_token': scene['token'],
                'scene_name': scene['name'],
                'timestamp': cur_ts,
                'is_first_in_scene': si == 0,
                'ego_pose': cur_ego,
                'cameras': cur_cam_block,
                'temporal_frames': temporal,
            })

            n = len(all_records)
            if n % 100 == 0:
                print(f'  已处理 {n} / {total - skipped} 帧...')

    print('[ 4/5 ] 写出索引文件...')
    dataset_info = {
        'version': '1.0',
        'source': f'nuScenes {meta["version_dir"].name}',
        'num_samples': len(all_records),
        'num_temporal_frames': len(history_steps),
        'history_steps': history_steps,
        'cameras': CAMERAS,
        'lidar2img': {
            'enabled': bool(inject_lidar2img),
            'lidar_channel': LIDAR_CHANNEL,
            'post_aug': POST_AUG.tolist(),
            'description': '4x4 matrix: lidar -> ego(lidar_ts) -> global -> ego(cam_ts) -> camera -> image -> augmented image',
        },
        'bev_params': {
            'x_range': [BEV_X_MIN, BEV_X_MIN + BEV_WIDTH * BEV_DX],
            'y_range': [BEV_Y_MIN, BEV_Y_MIN + BEV_HEIGHT * BEV_DY],
            'width': BEV_WIDTH,
            'height': BEV_HEIGHT,
            'resolution_m_per_pixel': BEV_DX,
        },
        'affine_params_doc': {
            'description': 'Index affine for inverse warp: u = a*j + b*i + tx ; v = c*j + d*i + ty',
            'order': ['a', 'b', 'tx', 'c', 'd', 'ty'],
            'units': 'index/grid coordinates; computed from full 4x4 ego poses and physical->index conversion',
            'source_physical_transform': 'T_hist_from_cur = inv(T_G_Ehist) @ T_G_Ecur',
        },
        'samples': all_records,
    }

    if verify_pth:
        _verify_against_pth(dataset_info, verify_pth)

    idx_path = outdir / 'dataset_info.json'
    write_json(idx_path, dataset_info)

    print('[ 5/5 ] 完成！')
    print(f'  样本总数       : {len(all_records)}')
    if skipped:
        print(f'  跳过(数据缺失) : {skipped}')
    if inject_lidar2img and warn_missing_lidar:
        print(f'  当前帧缺 LIDAR : {warn_missing_lidar}，对应帧未写入 lidar2img')
    if inject_lidar2img and warn_missing_lidar_hist:
        print(f'  历史帧缺 LIDAR : {warn_missing_lidar_hist}，对应帧未写入 lidar2img')
    print(f'  索引文件       : {idx_path}')
    print(f'  图像目录       : {img_dir}')
    if copy_images:
        imgs = list(img_dir.glob('*'))
        size_gb = sum(f.stat().st_size for f in imgs) / 1e9
        print(f'  图像文件数     : {len(imgs)}')
        print(f'  图像总大小     : {size_gb:.2f} GB')

    return dataset_info


# ─────────────────────────────────────────────────────────────────────────────
# Inject-only compatibility mode
# ─────────────────────────────────────────────────────────────────────────────

def inject_lidar2img_into_json(
    dataroot: str,
    json_path: str,
    out_path: Optional[str] = None,
    version: Optional[str] = None,
    verify_pth: Optional[str] = None,
) -> dict:
    """Compatibility mode for the old inject_lidar2img_nuscenes.py workflow."""
    dataroot_path = Path(dataroot)
    meta = load_nuscenes_metadata(dataroot_path, version)
    cal_map = meta['cal_map']
    ego_map = meta['ego_map']
    sd_map = meta['sd_map']
    _cal_to_channel, _ch_sds, _ch_ts, sample_key_data = build_channel_indexes(meta)

    print(f'加载 JSON: {json_path}')
    ds = load_json(Path(json_path))
    num_samples = len(ds.get('samples', []))
    print(f'共 {num_samples} 帧')

    print('计算并注入 lidar2img...')
    for i, sample_entry in enumerate(ds.get('samples', [])):
        sample_token = sample_entry.get('sample_token', '')
        key_data = sample_key_data.get(sample_token, {})
        lidar_sd = key_data.get(LIDAR_CHANNEL)
        if lidar_sd is None:
            print(f'  警告: 帧 {i} 缺少 LIDAR_TOP 数据，跳过当前帧 lidar2img', file=sys.stderr)
        else:
            for cam_name in CAMERAS:
                cam_info = sample_entry.get('cameras', {}).get(cam_name)
                cam_sd = key_data.get(cam_name)
                if cam_info is not None and cam_sd is not None:
                    cam_info['lidar2img'] = compute_lidar2img_from_sd(lidar_sd, cam_sd, cal_map, ego_map)

        for tf in sample_entry.get('temporal_frames', []):
            for cam_name in CAMERAS:
                cam_info = tf.get('cameras', {}).get(cam_name)
                if cam_info is None:
                    continue
                cam_sd_token = cam_info.get('sample_data_token')
                if not cam_sd_token or cam_sd_token not in sd_map:
                    continue
                cam_sd = sd_map[cam_sd_token]

                # Prefer a LIDAR_TOP keyframe tied to this sample_data's sample_token, matching the old script.
                hist_sample_token = cam_sd.get('sample_token', '')
                hist_lidar_sd = sample_key_data.get(hist_sample_token, {}).get(LIDAR_CHANNEL) or lidar_sd
                if hist_lidar_sd is None:
                    continue
                cam_info['lidar2img'] = compute_lidar2img_from_sd(hist_lidar_sd, cam_sd, cal_map, ego_map)

        if (i + 1) % 50 == 0 or i == num_samples - 1:
            print(f'  已处理 {i + 1}/{num_samples} 帧')

    ds['lidar2img'] = {
        'enabled': True,
        'lidar_channel': LIDAR_CHANNEL,
        'post_aug': POST_AUG.tolist(),
        'description': '4x4 matrix injected by preprocess_nuscenes_merged.py',
    }

    if verify_pth:
        _verify_against_pth(ds, verify_pth)

    final_out = Path(out_path) if out_path else Path(json_path)
    write_json(final_out, ds)
    print(f'写出: {final_out}')
    print('完成。')
    return ds


# ─────────────────────────────────────────────────────────────────────────────
# Tests and optional .pth verification
# ─────────────────────────────────────────────────────────────────────────────

def verify_affine_unit_test() -> bool:
    cur_trans = [410.78, 1179.47, 0.0]
    cur_rot = [0.5732, -0.0016, 0.0139, -0.8193]
    hist_trans = [413.39, 1176.44, 0.0]
    hist_rot = [0.5720, -0.0018, 0.0141, -0.8201]

    expected = {
        'a': 0.9999958266,
        'b': -0.0028884028,
        'c': 0.0028884365,
        'd': 0.9999956835,
        'tx': -3.5018117246,
        'ty': -7.1138773737,
    }

    linear_labels = {'a', 'b', 'c', 'd'}
    tol_linear = 1e-6
    tol_offset = 1e-5

    params = compute_affine_params(cur_trans, cur_rot, hist_trans, hist_rot)
    labels = ['a', 'b', 'tx', 'c', 'd', 'ty']
    all_ok = True
    for i, label in enumerate(labels):
        diff = abs(params[i] - expected[label])
        tol = tol_linear if label in linear_labels else tol_offset
        ok = diff < tol
        mark = '[OK]' if ok else '[FAIL]'
        print(f'  {mark} {label}: 计算={params[i]:.7f}  期望={expected[label]:.7f}  差={diff:.8f}  容差={tol}')
        all_ok = all_ok and ok
    return all_ok


def run_unit_tests() -> None:
    print('─── 单元测试：compute_affine_params（full 4x4 + index affine）───')
    ok = verify_affine_unit_test()
    if not ok:
        print('  存在失败项，请检查实现 [FAIL]')
        sys.exit(1)
    print('  所有测试通过 [OK]\n')

    print('─── 单元测试：index 仿射点验证 ───')
    p = compute_affine_params(
        [410.78, 1179.47, 0.0],
        [0.5732, -0.0016, 0.0139, -0.8193],
        [413.39, 1176.44, 0.0],
        [0.5720, -0.0018, 0.0141, -0.8201],
    )
    i, j = 5.0, 10.0
    u = p[0]*j + p[1]*i + p[2]
    v = p[3]*j + p[4]*i + p[5]
    print(f'  (i=5.0, j=10.0) → (u={u:.4f}, v={v:.4f})')
    print('  期望               → (6.4837, -2.0850)')
    assert abs(u - 6.4837) < 0.001 and abs(v + 2.0850) < 0.001, 'index 点变换结果偏差过大'
    print('  通过 [OK]\n')

    print('─── 单元测试：静止车辆 identity ───')
    trans = [100.0, 200.0, 0.0]
    rot = [0.7071, 0.0, 0.0, 0.7071]
    p_id = compute_affine_params(trans, rot, trans, rot)
    expected_id = IDENTITY_AFFINE
    for i, (v, e) in enumerate(zip(p_id, expected_id)):
        assert abs(v - e) < 1e-5, f'identity 参数 [{i}] 偏差: {v} vs {e}'
    print(f'  参数: {[round(v, 6) for v in p_id]}')
    print('  通过 [OK]\n')

    print('─── 单元测试：lidar2img 矩阵形状 ───')
    lidar_cs = {'rotation': [1, 0, 0, 0], 'translation': [0, 0, 0]}
    lidar_ego = {'rotation': [1, 0, 0, 0], 'translation': [0, 0, 0]}
    cam_cs = {
        'rotation': [1, 0, 0, 0],
        'translation': [0, 0, 0],
        'camera_intrinsic': [[1000, 0, 800], [0, 1000, 450], [0, 0, 1]],
    }
    cam_ego = {'rotation': [1, 0, 0, 0], 'translation': [0, 0, 0]}
    E = compute_lidar2img(lidar_cs, lidar_ego, cam_cs, cam_ego)
    assert E.shape == (4, 4) and abs(E[3, 3] - 1.0) < 1e-12
    print('  通过 [OK]\n')


def _verify_against_pth(ds: dict, pth_path: str) -> None:
    """Compare computed lidar2img in JSON against one FastBEV .pth sample, if available."""
    print(f'\n=== 验证：与 {pth_path} 比较 ===')

    class _G:
        def __init__(self, *args, **kwargs):
            pass
        def __setstate__(self, state):
            if isinstance(state, dict):
                self.__dict__.update(state)
            else:
                self._state = state

    stubs = [
        'torch', 'torch.storage', 'torch._utils', 'mmcv', 'mmcv.parallel',
        'mmcv.parallel.data_container', 'mmdet3d', 'mmdet3d.core',
        'mmdet3d.core.bbox', 'mmdet3d.core.bbox.structures',
        'mmdet3d.core.bbox.structures.lidar_box3d',
        'mmdet3d.core.points', 'mmdet3d.core.points.lidar_points',
        'mmdet3d.core.points.base_points',
    ]
    for module_name in stubs:
        if module_name not in sys.modules:
            sys.modules[module_name] = types.ModuleType(module_name)
    for cls_name in ('FloatStorage', 'HalfStorage', 'DoubleStorage', 'LongStorage',
                     'IntStorage', 'BoolStorage', 'ByteStorage'):
        setattr(sys.modules['torch'], cls_name, type(cls_name, (_G,), {}))
    sys.modules['torch.storage']._UntypedStorage = _G
    sys.modules['torch.storage']._TypedStorage = _G
    sys.modules['torch._utils']._rebuild_tensor_v2 = lambda *args, **kwargs: ('TENSOR_PLACEHOLDER',)

    class DC:
        def __init__(self, data=None, **kwargs):
            self._data = data
        def __setstate__(self, state):
            if isinstance(state, dict):
                self.__dict__.update(state)
                if '_data' not in self.__dict__:
                    self._data = state.get('data')
            else:
                self._data = state
        @property
        def data(self):
            return self._data

    sys.modules['mmcv.parallel.data_container'].DataContainer = DC
    sys.modules['mmcv.parallel'].DataContainer = DC

    try:
        with zipfile.ZipFile(pth_path) as zf:
            with zf.open('archive/data.pkl') as f:
                raw = f.read()
    except Exception as exc:
        print(f'  无法读取 .pth: {exc}', file=sys.stderr)
        return

    class _Up(pickle.Unpickler):
        def persistent_load(self, pid):
            return ('STORAGE_REF', pid)
        def find_class(self, mod, name):
            try:
                return super().find_class(mod, name)
            except Exception:
                cls = type(name, (_G,), {})
                if mod not in sys.modules:
                    sys.modules[mod] = types.ModuleType(mod)
                setattr(sys.modules[mod], name, cls)
                return cls

    data = _Up(io.BytesIO(raw)).load()
    inner = data['img_metas'].data[0][0]
    pth_exts = inner['lidar2img']['extrinsic']
    img_infos = inner['img_info']

    pth_cam_front_filename = img_infos[0]['filename']
    try:
        pth_ts = int(pth_cam_front_filename.split('__')[-1].replace('.jpg', ''))
    except ValueError:
        print('  无法解析 .pth 时间戳，取消验证', file=sys.stderr)
        return

    matched_idx = -1
    for idx, sample in enumerate(ds.get('samples', [])):
        cam_front = sample.get('cameras', {}).get('CAM_FRONT', {})
        if cam_front.get('timestamp') == pth_ts:
            matched_idx = idx
            break

    if matched_idx < 0:
        best_diff = float('inf')
        for idx, sample in enumerate(ds.get('samples', [])):
            cam_front = sample.get('cameras', {}).get('CAM_FRONT', {})
            ts = cam_front.get('timestamp')
            if ts is None:
                continue
            diff = abs(ts - pth_ts)
            if diff < best_diff:
                best_diff, matched_idx = diff, idx
        print(f'  未找到精确匹配（ts={pth_ts}），使用最近帧 sample[{matched_idx}]（diff={best_diff} us）')
    else:
        print(f'  .pth 对应 sample[{matched_idx}]  场景={ds["samples"][matched_idx].get("scene_name", "")}')

    if matched_idx < 0:
        print('  JSON 中没有可比较样本，取消验证', file=sys.stderr)
        return

    matched_sample = ds['samples'][matched_idx]
    inv_post_aug = np.linalg.inv(POST_AUG)

    print(f'  {"相机":20s}  {"矩阵max|diff|":>16}  {"投影偏差(20m前方)":>18}  状态')
    print(f'  {"-"*20}  {"-"*16}  {"-"*18}  ----')
    all_ok = True
    for j, cam_name in enumerate(CAMERAS):
        cam_info = matched_sample.get('cameras', {}).get(cam_name)
        if not cam_info or 'lidar2img' not in cam_info:
            print(f'  {cam_name:20s}: JSON 中无 lidar2img，跳过')
            all_ok = False
            continue

        my_E = np.array(cam_info['lidar2img'])
        gt_E = np.array(pth_exts[j]).reshape(4, 4)
        mat_diff = np.abs(my_E - gt_E).max()

        test_pt = np.array([0, 20, 0, 1])
        p_my = my_E @ test_pt
        p_gt = gt_E @ test_pt
        if abs(p_my[2]) > 0.01 and abs(p_gt[2]) > 0.01:
            pix_my = inv_post_aug @ np.array([p_my[0] / p_my[2], p_my[1] / p_my[2], 1])
            pix_gt = inv_post_aug @ np.array([p_gt[0] / p_gt[2], p_gt[1] / p_gt[2], 1])
            px_diff = float(np.sqrt((pix_my[0] - pix_gt[0])**2 + (pix_my[1] - pix_gt[1])**2))
            px_str = f'{px_diff:.4f}px'
        else:
            px_diff = float('inf')
            px_str = '(测试点不可见)'

        ok = mat_diff < 0.1 and px_diff < 1.0
        all_ok = all_ok and ok
        status = '[OK]' if ok else '[FAIL]'
        print(f'  {cam_name:20s}  {mat_diff:>16.6f}  {px_str:>18}  {status}')

    if all_ok:
        print('  结论：全部通过，nuScenes 计算结果与 .pth GT 精确吻合。')
    else:
        print('  结论：存在偏差，请检查 nuScenes 数据路径、版本、history/camera 顺序或 post_aug。')


# ─────────────────────────────────────────────────────────────────────────────
# CLI
# ─────────────────────────────────────────────────────────────────────────────

def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(
        description='FastBEV nuScenes 预处理：生成 dataset_info.json、拷贝图像并注入 lidar2img',
        formatter_class=argparse.RawTextHelpFormatter,
    )
    ap.add_argument('--dataroot', required=False, help='nuScenes 数据集根目录（包含 v1.0-mini/ 等子目录）')
    ap.add_argument('--version', default=None, help='元数据版本目录名，例如 v1.0-mini；默认自动查找')
    ap.add_argument('--output', default='./fastbev_data', help='预处理输出目录（默认: ./fastbev_data）')
    ap.add_argument('--no-copy', action='store_true', help='不拷贝图像，只生成索引 JSON')
    ap.add_argument('--no-lidar2img', action='store_true', help='不计算/写入 lidar2img（仅保留原始预处理结果）')
    ap.add_argument('--history-steps', type=parse_history_steps, default=DEFAULT_HISTORY_STEPS,
                    help='历史帧沿 CAM_FRONT prev 链的步数，默认: 1,3,5')
    ap.add_argument('--max-ts-diff-us', type=int, default=MAX_TS_DIFF_US,
                    help=f'跨相机/雷达时间戳匹配容差，单位 us，默认: {MAX_TS_DIFF_US}')
    ap.add_argument('--verify-pth', default=None, help='可选：FastBEV .pth 文件路径，用于比较第一个匹配帧的 lidar2img')
    ap.add_argument('--test', action='store_true', help='仅运行单元测试，不处理数据集')

    # Compatibility with inject_lidar2img_nuscenes.py.
    ap.add_argument('--json', default=None,
                    help='兼容旧注入脚本：指定已有 dataset_info.json 时进入 inject-only 模式')
    ap.add_argument('--out', default=None,
                    help='inject-only 模式输出 JSON 路径；默认覆盖 --json')

    args = ap.parse_args(argv)

    if args.test:
        run_unit_tests()
        return 0

    if not args.dataroot:
        ap.error('必须指定 --dataroot，或使用 --test 模式')

    if args.json:
        inject_lidar2img_into_json(
            dataroot=args.dataroot,
            json_path=args.json,
            out_path=args.out,
            version=args.version,
            verify_pth=args.verify_pth,
        )
    else:
        preprocess(
            dataroot=args.dataroot,
            output=args.output,
            copy_images=not args.no_copy,
            version=args.version,
            history_steps=args.history_steps,
            inject_lidar2img=not args.no_lidar2img,
            max_ts_diff_us=args.max_ts_diff_us,
            verify_pth=args.verify_pth,
        )

    return 0


if __name__ == '__main__':
    sys.exit(main())
