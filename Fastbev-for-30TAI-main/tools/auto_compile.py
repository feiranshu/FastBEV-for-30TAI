#!/usr/bin/env python3
"""
Compile and optionally run FastBEV neural-network model configs.

Examples:
  python tools/auto_compile.py --mode compile
  python tools/auto_compile.py --mode run --dataset nuscenes
  python tools/auto_compile.py --mode all --dataset carla
  python tools/auto_compile.py --mode compile --dataset vehicle
  python tools/auto_compile.py --mode compile --dataset nuscenes --model matrixvt
"""

import argparse
import os
import subprocess
import sys


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
APP_DIR = os.path.dirname(SCRIPT_DIR)
COMPILE_DIR = os.path.join(APP_DIR, "compile")


MODEL_SETS = {
    "nuscenes": [
        {
            "name": "NuScenes Extractor (Part1)",
            "config": os.path.join("config", "extractor", "nuscenes", "fastbev_part1_nuscenes.toml"),
        },
        {
            "name": "NuScenes Decoder (Part3)",
            "config": os.path.join("config", "decoder", "nuscenes", "fastbev_part3_nuscenes.toml"),
        },
    ],
    "carla": [
        {
            "name": "CARLA Extractor (Part1)",
            "config": os.path.join("config", "extractor", "carla", "fastbev_part1_carla.toml"),
        },
        {
            "name": "CARLA Decoder (Part3)",
            "config": os.path.join("config", "decoder", "carla", "fastbev_part3_carla.toml"),
        },
    ],
    "vehicle": [
        {
            "name": "Vehicle FP16 Extractor (Part1)",
            "config": os.path.join("config", "extractor", "vehicle_fp16", "fastbev_part1_vehicle_fp16.toml"),
        },
        {
            "name": "Vehicle FP16 Decoder (Part3)",
            "config": os.path.join("config", "decoder", "vehicle_fp16", "fastbev_part3_vehicle_fp16.toml"),
        },
    ],
}

MATRIXVT_MODEL_SETS = {
    "nuscenes": [
        {
            "name": "MatrixVT NuScenes FP16",
            "config": os.path.join("config", "matrixvt", "nuscenes", "matrixvt_nuscenes_fp16.toml"),
        },
    ],
}


def run_icraft(action, config_path, model_name):
    full_path = os.path.join(COMPILE_DIR, config_path)
    if not os.path.isfile(full_path):
        print(f"[Error] config not found: {full_path}")
        return False

    cmd = ["icraft", action, config_path]
    print(f"\n[{action.upper()}] {model_name}")
    print(f"  command: {' '.join(cmd)}")
    print(f"  cwd: {COMPILE_DIR}")
    print("-" * 60)

    ret = subprocess.run(cmd, cwd=COMPILE_DIR)
    print("-" * 60)

    if ret.returncode == 0:
        print(f"[OK] {model_name} {action} succeeded")
        return True

    print(f"[FAIL] {model_name} {action} failed, exit code: {ret.returncode}")
    return False


def main():
    parser = argparse.ArgumentParser(description="FastBEV model compile/run helper")
    parser.add_argument(
        "--mode",
        choices=["compile", "run", "all"],
        default="all",
        help="compile only, run only, or both. Default: all",
    )
    parser.add_argument(
        "--dataset",
        choices=["nuscenes", "carla", "vehicle", "all"],
        default="nuscenes",
        help="model set to use. Default: nuscenes",
    )
    parser.add_argument(
        "--model",
        choices=["fastbev", "matrixvt"],
        default="fastbev",
        help="model family to compile/run. Default: fastbev",
    )
    args = parser.parse_args()

    print("=" * 60)
    print(f"  model helper  mode={args.mode} dataset={args.dataset} model={args.model}")
    print("=" * 60)

    if not os.path.isdir(COMPILE_DIR):
        print(f"[Error] compile dir not found: {COMPILE_DIR}")
        sys.exit(1)

    actions = []
    if args.mode in ("compile", "all"):
        actions.append("compile")
    if args.mode in ("run", "all"):
        actions.append("run")

    if args.model == "matrixvt":
        if args.dataset in ("carla", "vehicle"):
            print("[Error] MatrixVT config is currently available only for dataset=nuscenes")
            sys.exit(1)
        selected_datasets = ["nuscenes"]
        model_sets = MATRIXVT_MODEL_SETS
    else:
        selected_datasets = ["nuscenes", "carla", "vehicle"] if args.dataset == "all" else [args.dataset]
        model_sets = MODEL_SETS

    success_count = 0
    fail_count = 0

    for action in actions:
        for dataset_name in selected_datasets:
            for model in model_sets[dataset_name]:
                ok = run_icraft(action, model["config"], model["name"])
                if ok:
                    success_count += 1
                else:
                    fail_count += 1

    print("\n" + "=" * 60)
    print(f"  done: {success_count} succeeded, {fail_count} failed")
    print("=" * 60)

    if fail_count > 0:
        sys.exit(1)


if __name__ == "__main__":
    main()
