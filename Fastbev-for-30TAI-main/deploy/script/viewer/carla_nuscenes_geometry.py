#!/usr/bin/env python3
"""Fixed CARLA six-camera geometry matching task/live_drive_demo.

The CARLA rig reuses one nuScenes calibrated-sensor set. FastBEV detections are
in LIDAR_TOP coordinates, so each projection is:

    lidar -> ego -> camera -> cropped 1600x900 image

The crop in nuscenes_rig.py preserves the principal point from the nuScenes
camera matrix, so no additional post augmentation is required.
"""

from __future__ import annotations

import math

import interactive_viewer as viewer


LIDAR2EGO_TRANSLATION = [0.943713, 0.0, 1.84023]
LIDAR2EGO_QUATERNION = [
    0.7077955119163518,
    -0.006492242056004365,
    0.010646214713995808,
    -0.7063073142877817,
]

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


def quaternion_matrix(q: list[float]) -> list[list[float]]:
    norm = math.sqrt(sum(value * value for value in q))
    if norm == 0.0:
        raise ValueError("zero quaternion")
    w, x, y, z = [value / norm for value in q]
    return [
        [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
        [2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
        [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y],
    ]


def transpose33(matrix: list[list[float]]) -> list[list[float]]:
    return [[matrix[col][row] for col in range(3)] for row in range(3)]


def matmul33(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    return [
        [sum(a[row][k] * b[k][col] for k in range(3)) for col in range(3)]
        for row in range(3)
    ]


def matvec3(matrix: list[list[float]], vector: list[float]) -> list[float]:
    return [sum(matrix[row][k] * vector[k] for k in range(3)) for row in range(3)]


def build_camera_infos() -> dict[str, viewer.CameraInfo]:
    lidar_rotation = quaternion_matrix(LIDAR2EGO_QUATERNION)
    cameras: dict[str, viewer.CameraInfo] = {}
    for name, spec in CAMERAS.items():
        camera_to_ego = quaternion_matrix(spec["rotation"])
        ego_to_camera = transpose33(camera_to_ego)
        lidar_to_camera = matmul33(ego_to_camera, lidar_rotation)
        delta = [
            LIDAR2EGO_TRANSLATION[i] - spec["translation"][i]
            for i in range(3)
        ]
        translation = matvec3(ego_to_camera, delta)
        extrinsic = [
            [*lidar_to_camera[row], translation[row]]
            for row in range(3)
        ]
        extrinsic.append([0.0, 0.0, 0.0, 1.0])
        cameras[name] = viewer.CameraInfo(
            name=name,
            img_path="",
            extrinsic=extrinsic,
            intrinsic=spec["K"],
            post_aug_inv=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        )
    return cameras

