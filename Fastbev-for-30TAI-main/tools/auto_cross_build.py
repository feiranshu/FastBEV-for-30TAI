#!/usr/bin/env python3
"""
Cross-build the retained FastBEV deploy executables for aarch64.

This helper intentionally does not delete the build directory. The repository
policy forbids bulk recursive deletion; CMake is allowed to reuse the existing
build tree for incremental builds.
"""

from __future__ import annotations

import multiprocessing
import os
from pathlib import Path
import shutil
import subprocess
import sys


SCRIPT_DIR = Path(__file__).resolve().parent
APP_DIR = SCRIPT_DIR.parent
DEPLOY_DIR = APP_DIR / "deploy"
BUILD_DIR = DEPLOY_DIR / "build"
TARGET_CHIP = "ZG"

EXECUTABLES = [
    "fastbev_pipeline_audio",
    "fastbev_pipeline_async",
    "fastbev_pipeline_SA",
    "fastbev_pipeline_carla",
    "fastbev_pipeline_carla_fp16",
    "fastbev_pipeline_carla_new",
    "fastbev_pipeline_carla_new_modular",
    "fastbev_pipeline_vehicle",
    "fastbev_pipeline_vehicle_live",
    "fastbev_pipeline_matrixvt",
    "fastbev_pipeline_matrixvt_native",
    "pointpillars",
    "visualize",
    "visualize_vehicle",
]


def run_cmd(cmd: list[str], cwd: Path, desc: str) -> None:
    print(f"\n[Step] {desc}")
    print("  command:", " ".join(cmd))
    print("  cwd:", cwd)
    print("-" * 60)
    result = subprocess.run(cmd, cwd=str(cwd))
    print("-" * 60)
    if result.returncode != 0:
        print(f"[Error] command failed, exit code: {result.returncode}")
        sys.exit(result.returncode)


def main() -> int:
    print("=" * 60)
    print(f"  FastBEV cross build (aarch64, TARGET_CHIP={TARGET_CHIP})")
    print("=" * 60)

    if not DEPLOY_DIR.is_dir():
        print(f"[Error] deploy dir not found: {DEPLOY_DIR}")
        return 1

    BUILD_DIR.mkdir(parents=True, exist_ok=True)

    run_cmd(
        ["cmake", "-S", ".", "-B", "build", f"-DTARGET_CHIP={TARGET_CHIP}"],
        cwd=DEPLOY_DIR,
        desc="Configure CMake",
    )

    jobs = multiprocessing.cpu_count()
    run_cmd(
        ["cmake", "--build", "build", f"-j{jobs}"],
        cwd=DEPLOY_DIR,
        desc=f"Build retained targets (j{jobs})",
    )

    print(f"\n[Step] Copy executables to {DEPLOY_DIR}")
    for exe in EXECUTABLES:
        src = BUILD_DIR / exe
        dst = DEPLOY_DIR / exe
        if not src.is_file():
            print(f"  [Warning] missing build artifact: {src}")
            continue
        shutil.copy2(src, dst)
        os.chmod(dst, 0o777)
        print(f"  {exe} -> {dst} [OK]")

    print("\n" + "=" * 60)
    print("  Cross build complete")
    print("  Board run examples:")
    print("    cd app/deploy")
    print("    ./fastbev_pipeline_audio ./config/fastbev_nuscenes.yaml")
    print("    ./fastbev_pipeline_async ./config/fastbev_nuscenes.yaml")
    print("    ./fastbev_pipeline_SA ./config/fastbev_nuscenes.yaml")
    print("    ./fastbev_pipeline_carla ./config/fastbev_carla.yaml --host 0.0.0.0 --port 5200")
    print("    ./fastbev_pipeline_carla_fp16 ./config/fastbev_carla_fp16.yaml --host 0.0.0.0 --port 5200")
    print("    ./fastbev_pipeline_carla_new ./config/fastbev_carla_new.yaml --host 0.0.0.0 --port 5200 --source fastbev-real-edge")
    print("    ./fastbev_pipeline_carla_new_modular ./config/fastbev_carla_new_modular.yaml --host 0.0.0.0 --port 5200 --control-port 5201 --source fastbev-real-edge")
    print("    ./fastbev_pipeline_vehicle ./config/fastbev_vehicle_fp16.yaml")
    print("    ./fastbev_pipeline_vehicle_live ./config/fastbev_vehicle_live.yaml --host 0.0.0.0 --port 5200 --source vehicle-real-edge")
    print("    ./fastbev_pipeline_matrixvt ./config/matrixvt_nuscenes.yaml")
    print("    ./fastbev_pipeline_matrixvt_native ./config/matrixvt_nuscenes_native.yaml")
    print("    ./pointpillars")
    print("=" * 60)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
