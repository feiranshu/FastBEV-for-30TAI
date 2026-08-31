# FastBEV 工程交接说明

## 1. 当前工程目标

本工程是在复旦微诸葛 / FMQL30TAI / ZG330AIU 开发板上部署 FastBEV 目标检测算法。当前已经完成 NuScenes 离线数据、CARLA 仿真数据和开发板端 NPU/PL 联调，下一阶段将投入真实小车场景：接入实际摄像头、采集新数据集、训练新网络，并把已经导出的 ONNX 模型重新编译部署到开发板。

当前主数据通路为：

```text
六路图像
-> CPU/Parser 前处理
-> Part1 Extractor NPU
-> Part2 FPGA/PL LUT 与特征重排
-> Part3 Decoder NPU
-> 后处理/NMS
-> 本地 visualize / PC Viewer / CARLA BEV1 / LED 与音频告警
```

正式命名中统一使用 `CARLA/carla`。历史对话中出现过 `calar/clar`，均指 CARLA。

## 2. 当前工程结构

关键目录如下：

```text
compile/                 神经网络编译配置、浮点模型、校准集和测试输入
dataset/                 NuScenes 数据集预处理脚本和本地数据目录
deploy/                  板端部署工程、C++ 源码、运行配置、模型和 PC 侧脚本
deploy/src/              板端 C++ pipeline、后处理、告警、可视化源码
deploy/config/           板端运行 YAML
deploy/imodel/           已编译 ZG 模型，按 extractor/decoder 和数据集拆分
deploy/script/           PC Viewer、Gateway、离线可视化脚本
carla/                   CARLA 服务器端网页后端
docs/                    运行指南、服务器指南、PC Viewer 指南和版本说明
tools/                   根目录自动化入口：编译、预处理、交叉编译和一键启动脚本
hardware/                当前工程相关硬件资料和位流参考
task/                    临时任务资料、旧 RTL、实验模型和调试记录
old/                     已归档的旧 CARLA 模型、配置或日志
BOOT/                    不同实验位流文件
```

大文件如 `dataset`、`compile/qtset`、`compile/test`、`deploy/deps` 通常通过网盘同步，不建议直接提交到 GitHub。

## 3. 当前保留的运行版本

当前 `deploy/CMakeLists.txt` 中保留以下可执行目标：

| 程序 | 用途 | 位流要求 |
| --- | --- | --- |
| `fastbev_pipeline_audio` | NuScenes 推理 + PC 网页交互式 Viewer + LED/音频告警 | `sound demo` |
| `fastbev_pipeline_async` | NuScenes 流水化推理 + 板端本地 `visualize` 输出 PNG | `sound demo` |
| `fastbev_pipeline_SA` | NuScenes Group4/SA 推理 + 板端本地 `visualize` 输出 PNG | `SA_v2` |
| `fastbev_pipeline_carla` | CARLA BEV1 实时服务，INT8 Decoder 路径 | `sound demo` |
| `fastbev_pipeline_carla_fp16` | CARLA FP16 Decoder 验证版，旧 Part2 走 PL-PS-PL | 旧 FP32 Part2 位流 |
| `fastbev_pipeline_carla_new` | CARLA FP16 Decoder Conv 直连候选版，新 Part2 直接写 `view(12)/v244` | `0x20260721` FP16 Part2 位流 |
| `visualize` | NuScenes 本地 PNG 可视化工具 | 无独立位流要求 |

NuScenes 网页交互主版本使用：

```bash
./fastbev_pipeline_audio ./config/fastbev_nuscenes.yaml
```

CARLA 新 FP16 直连候选版本使用：

```bash
./fastbev_pipeline_carla_new ./config/fastbev_carla_new.yaml \
  --host 0.0.0.0 \
  --port 5200 \
  --source fastbev-real-edge
```

PC Viewer、CARLA Server 和 Gateway 的详细启动方法见：

```text
docs/pc_viewer_guidance.md
docs/calar_guidance.md
README.md
```

## 4. 模型与编译策略

当前模型按数据集拆分：

```text
compile/fmodel/extractor/nuscenes/
compile/fmodel/extractor/carla/
compile/fmodel/decoder/nuscenes/
compile/fmodel/decoder/carla/

deploy/imodel/extractor/nuscenes/
deploy/imodel/extractor/carla/
deploy/imodel/decoder/nuscenes/
deploy/imodel/decoder/carla/
deploy/imodel/decoder/carla_fp16/
```

Part1 当前公开输入为 FP32 NHWC `[6,256,704,3]`。运行时送入原始 BGR `0~255` FP32，Parser 内部完成：

```text
BGR -> RGB -> (pixel - mean) / std
```

Part1 仍是 INT8 编译，虽然输出公开 Tensor 是 FP32 `[6,64,176,64]`，内部量化仍会影响 FEAT2D 质量。校准集必须和运行时输入语义一致，即 BGR FP32 `0~255`、NHWC、相同 resize/crop。

Part3 在 CARLA 上采用 FP16 后准确度明显提升，说明 Decoder INT8 量化是一个重要误差源。当前仍存在偶发假阳性，需要继续定位来源：Part1 INT8 校准、CARLA 域偏移、Part2 RTL 排布、相机几何或后处理策略都有可能。

CARLA FP16 新直连版本的目标是让 Part2 直接输出 Decoder Conv 所需的 FP16 NCHWc16 数据，直接写入 Decoder `view(12)` 的 `v244` Runtime PLDDR，避免旧版 PS 读回、CPU 四次复制和重排。

下一阶段已有新数据集训练导出的 ONNX。建议不要覆盖 NuScenes/CARLA 现有目录，优先新增真实小车数据集对应的模型目录、TOML、校准集和运行 YAML。

## 5. 已完成进展

- NuScenes 离线数据预处理、Part1/Part2/Part3、后处理和本地 `visualize` 已跑通。
- NuScenes PC 网页交互式 Viewer 已接入，开发板可向 PC 推送图像和检测结果。
- LED 与 USB 声卡告警已接入 NuScenes `fastbev_pipeline_audio`。
- CARLA Server、PC Gateway、PC Viewer 与开发板 BEV1 服务已联调通过。
- CARLA 内置检测框显示逻辑已改为保持到下一次检测结果到达再刷新。
- CARLA 后处理已做定制：丢弃 `bicycle(5)`，对 `pedestrian(7)` 和 `motorcycle(6)` 做同类中心距离去重，对车辆类、motorcycle 和 pedestrian 做跨类别 IoU 抑制。
- CARLA Part3 FP16 版本精度相对 INT8 有明显改善。
- aTrust/VPN 导致 Windows 到开发板 `5200` 路由错误的问题已有处理办法，见 `docs/calar_guidance.md` 末尾的 `/32` 路由修复流程。

## 6. 当前问题与风险

- CARLA 场景仍有偶发高分假阳性，需要固定问题帧，对比 `INT8 Part1 + FP16 Part3` 与浮点/FP16 参考输出。
- Part1 仍为 INT8，校准集数量和覆盖范围可能不足，尤其在新城市、新天气、真实摄像头输入下风险更高。
- CARLA FP16 新直连路径需要继续确认 RTL 输出与 Decoder Conv 前置 `cast/reshape/transpose` 等算子完全等价。
- CARLA Simulator 在高画质、GPU 资源紧张或端口残留时可能启动失败或卡住，需先检查 `2000/2001/8080` 端口和 `CarlaUE4` 进程。
- 真实摄像头接入后，最大风险不是代码能否跑，而是输入分布是否和训练/校准一致：相机顺序、时间同步、内参、外参、畸变、图像格式、曝光、resize/crop 都会影响结果。
- 告警方位逻辑当前按 FastBEV/NuScenes 坐标理解：`Y+` 为前方，`X+` 为右方。真实小车坐标系必须重新确认。

## 7. 下一阶段：真实小车摄像头接入

建议按以下顺序推进：

1. 明确真实小车的六路摄像头定义：相机顺序、分辨率、帧率、同步方式、内参、外参和坐标系。
2. 建立真实摄像头采集输入路径，先只保存六路图像和时间戳，不立即接入 NPU。
3. 用新数据集导出的 ONNX 在 PC/服务器侧跑浮点验证，确认类别、坐标系、后处理阈值和 NMS 逻辑。
4. 为新数据集建立独立编译目录和配置，不覆盖现有 NuScenes/CARLA：

   ```text
   compile/fmodel/extractor/<new_dataset>/
   compile/fmodel/decoder/<new_dataset>/
   compile/qtset/extractor/<new_dataset>/
   compile/qtset/decoder/<new_dataset>/
   deploy/imodel/extractor/<new_dataset>/
   deploy/imodel/decoder/<new_dataset>/
   deploy/config/fastbev_<new_dataset>.yaml
   ```

5. 生成与真实运行完全一致的 Part1 校准集：BGR FP32 `0~255`、NHWC `[6,256,704,3]`、同样 resize/crop 和相机顺序。
6. 先编译并验证 Part1 输出，再接 Part2，再接 Part3，最后接 Viewer 和告警。
7. 最后才接入小车实时运行，先低速、短距离、固定场景验证，记录每帧输入、结果、告警和延时。

## 8. 新对话优先阅读清单

新对话接手时建议优先阅读：

```text
README.md
AGEND.md
docs/pc_viewer_guidance.md
docs/calar_guidance.md
deploy/CMakeLists.txt
deploy/config/fastbev_nuscenes.yaml
deploy/config/fastbev_carla_new.yaml
compile/config/extractor/carla/fastbev_part1_carla.toml
compile/config/decoder/carla/fastbev_part3_carla_fp16.toml
deploy/src/fastbev_pipeline.cpp
deploy/src/fastbev_pipeline_carla_new.cpp
deploy/src/alert_manager.cpp
deploy/src/nms.cpp
```

新阶段如果要接入真实摄像头，优先关注输入链路和坐标系，不要先改 Part2 或 NMS。模型能否稳定工作，首先取决于真实摄像头数据是否和训练、校准、运行时前处理保持一致。
