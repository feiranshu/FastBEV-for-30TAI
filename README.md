# FastBEV for 30TAI

面向 30TAI/FPAI 平台的 FastBEV 异构部署、延迟优化与实体验证工程。

本仓库以 FastBEV 为主线，完整保留从模型编译、板端运行、FPGA Part2 加速到 CARLA 与六路相机实车联调的软硬件工程。MatrixVT、PointPillars 和 TCP 用于主流方案对照及系统能力验证，不是本项目的主要优化对象。

## 项目亮点

- **FastBEV 全链路部署**：实现六路相机输入与 `Part1 NPU → Part2 PL → Part3 NPU` 异构推理链路。
- **FPGA Part2 加速**：完成 FP32/INT8 量化、LUT 2D-to-3D 映射、四帧 Group4、SA 仿射对齐、缓存与 DDR 访问优化。
- **延迟优化**：Pro v2.0 板端 `Part2 LUT+concat` 记录为 `385.17 ms`，相较 Pro v1.0 的 `1067.32 ms` 下降约 `63.9%`，约为原版本的 `2.77×` 速度。
- **实体系统落地**：支持六路 UVC 相机小车、开发板、PC Viewer、LED/音频预警及 CARLA 闭环联调。
- **工程可复核**：保留 RTL、testbench、约束、实现报告、运行日志、bitstream、BOOT 镜像、板端程序和技术文档。

## 系统链路

```text
六路相机 / NuScenes / CARLA
            │
            ▼
     FastBEV Part1（NPU）
            │
            ▼
  Part2 LUT / Group4 / SA（FPGA）
            │
            ▼
     FastBEV Part3（NPU）
            │
            ├─ 3D 检测与可视化
            ├─ LED / 音频预警
            └─ CARLA / 实车控制链路
```

## 仓库结构

```text
FastBEV-for-30TAI/
├─ Fastbev-for-30TAI-main/   软件工程
│  ├─ compile/               模型编译配置与日志
│  ├─ dataset/               数据预处理
│  ├─ deploy/                板端 C++、配置、Viewer 与 Gateway
│  ├─ carla/                 CARLA 服务端程序
│  ├─ control/               实体小车控制程序
│  ├─ docs/                  软件部署与接口文档
│  └─ tools/                 编译、交叉构建和演示辅助工具
└─ FastBEV-FPGA/             硬件工程
   ├─ fpga_flash_1.0/        当前帧 Part2 基线
   ├─ fpga_flash_2.0/        当前帧延迟与预警优化版
   ├─ fpga_pro_v1.0/         四帧 Group4 + SA 基线
   └─ fpga_pro_v2.0/         四帧 Group4 + SA 性能优化版
```

详细说明：

- [软件工程说明](Fastbev-for-30TAI-main/README.md)
- [FPGA 工程说明](FastBEV-FPGA/README.md)

## FastBEV 硬件版本

| 工程 | 主要内容 | 定位 |
|---|---|---|
| `fpga_flash_1.0` | 量化、LUT 映射、64→256 通道复制 | 当前帧 Part2 基线 |
| `fpga_flash_2.0` | LUT cache、请求流水、性能计数、三级 LED 预警 | 当前帧优化与预警版本 |
| `fpga_pro_v1.0` | 四帧 history、Group4、SA 仿射对齐 | 时序融合基线 |
| `fpga_pro_v2.0` | SA DDA、16-lane 插值、cache、outstanding、ping-pong | 时序融合性能优化版本 |

四个工程面向 `xc7z030ffg676-2`，原始实现环境为 Vivado 2018.3。各版本的 RTL、EDIF、XDC、bitstream 和 BOOT 镜像必须成套使用，不应跨目录混用。

## 软件能力

- FastBEV NuScenes、CARLA 与 Vehicle 六路相机版本。
- PC/开发板之间的 TCP、Gateway、网页 Viewer 与离线可视化。
- CARLA 高频控制流与低频感知流双链路联调。
- 实体小车六路 UVC 采集、Raw BGR 传输和实时检测。
- LED 与音频分级预警。
- MatrixVT、PointPillars 和 TCP 对照链路。

具体编译命令、运行参数、数据依赖和端口说明请查看[软件工程 README](Fastbev-for-30TAI-main/README.md)。

## 获取工程

仓库使用 Git LFS 管理 EDF、bitstream、BOOT 镜像、模型、二进制程序和技术文档。克隆前请先安装 Git LFS：

```bash
git lfs install
git clone https://github.com/feiranshu/FastBEV-for-30TAI.git
cd FastBEV-for-30TAI
git lfs pull
```

大型数据集、量化校准集和第三方运行依赖未直接纳入普通 Git 对象，下载与放置位置见[软件工程说明](Fastbev-for-30TAI-main/README.md#2-数据集与依赖文件下载)。

## 复现与核验建议

1. 从软件工程的 `compile/config/` 核验模型编译配置。
2. 从 `deploy/src/`、`deploy/config/` 核验板端推理和联调链路。
3. 从硬件各版本的 RTL、testbench 与约束目录核验实现机制。
4. 从 `Verification_Results/`、`latency_opt_reports/` 核验综合、实现、时序和回归结果。
5. 烧录前核对对应版本 README 中的镜像文件名、字节数和 SHA-256。

## 说明

本仓库记录比赛项目的工程实现与验证材料。硬件结果依赖指定器件、工具版本、厂商封装和板级环境；性能数据应以对应版本的原始日志、测试输入与计时范围为准。
