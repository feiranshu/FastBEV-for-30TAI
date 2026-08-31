#!/usr/bin/env python3
"""
auto_preprocess.py  —  一键数据集预处理
在 app/tools/ 目录下运行，自动调用 dataset/preprocess_nuscenes_merged.py
"""

import os
import sys
import subprocess

# ── 路径定义（相对于 app/tools/） ──
SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
APP_DIR     = os.path.dirname(SCRIPT_DIR)                          # app/
DATASET_DIR = os.path.join(APP_DIR, "dataset")                     # app/dataset/
PREPROCESS  = os.path.join(DATASET_DIR, "preprocess_nuscenes_merged.py")
DATAROOT    = os.path.join(DATASET_DIR, "nuscenes")                # app/dataset/nuscenes/
OUTPUT_DIR  = os.path.join(APP_DIR, "deploy", "io", "input", "data")  # app/deploy/io/input/data


def main():
    print("=" * 60)
    print("  FastBEV 数据集预处理")
    print("=" * 60)

    # 检查预处理脚本是否存在
    if not os.path.isfile(PREPROCESS):
        print(f"[Error] 预处理脚本不存在: {PREPROCESS}")
        sys.exit(1)

    # 检查数据集目录是否存在
    if not os.path.isdir(DATAROOT):
        print(f"[Error] 数据集目录不存在: {DATAROOT}")
        sys.exit(1)

    # 创建输出目录
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    print(f"[Info] 数据集路径 : {DATAROOT}")
    print(f"[Info] 输出路径   : {OUTPUT_DIR}")
    print()

    # 执行预处理（在 dataset/ 目录下运行）
    cmd = [
        sys.executable,
        PREPROCESS,
        "--dataroot", DATAROOT,
        "--output",   OUTPUT_DIR
    ]
    print(f"[Run] {' '.join(cmd)}")
    print("-" * 60)

    ret = subprocess.run(cmd, cwd=DATASET_DIR)
    print("-" * 60)

    if ret.returncode == 0:
        print("[Done] 数据集预处理完成")
    else:
        print(f"[Error] 预处理失败，退出码: {ret.returncode}")
        sys.exit(ret.returncode)


if __name__ == "__main__":
    main()
