# DriveBench on FPAI - 运行时工程

## 1. 工程介绍

本工程是面向 FPAI 开发板的多 BEV 感知算法与自动驾驶算法运行时工程。当前仓库重点覆盖板端推理、PC/服务器数据流、交互式可视化和 CARLA/实车联调。

BEV 感知算法的核心思路是将多传感器输入映射到 BEV（Bird's-Eye View，鸟瞰图）空间，在统一的俯视坐标系中完成 3D 目标检测。本工程目前包含：

- `FastBEV`：六路相机图像输入，分为 Part1 NPU、Part2 PL、Part3 NPU 三段执行。
- `FastBEV-SA`：在 FastBEV 基础上加入历史帧/SA 相关逻辑，用于 NuScenes Group4 版本验证。
- `MatrixVT`：单模型 BEV 网络，主要用于 NuScenes 侧的单网络验证。
- `PointPillars`：点云输入的 BEV 检测网络，用于 LiDAR 感知链路验证。

自动驾驶部分目前包含两类方向：

- `TCP`：端到端驾驶网络，直接从图像特征预测控制相关输出，适合 CARLA 闭环驾驶验证。
- `CARLA Modular`：服务器端提供高频控制流和低频感知流，开发板侧同时处理感知结果和控制指令，用于 Radar / Route / AEB / Pure Pursuit / PID 的组合式自动驾驶联调。

## 2. 数据集与依赖文件下载

大型依赖、数据集、校准集和输入数据不建议直接提交到 Git。请按需从网盘下载后放到对应目录。

| 内容 | 用途 | 下载链接 | 放置路径 |
| --- | --- | --- | --- |
| `deps` | 交叉编译和运行依赖，例如 OpenCV、FFmpeg、iCraft SDK、Python venv 等 | [百度网盘](https://pan.baidu.com/s/1LFA1AcdyHQtmj4I4Eh23Sg?pwd=dyeo)，提取码 `dyeo` | `deploy/deps/` |
| `nuscenes` | NuScenes 原始数据集和相关验证数据 | [百度网盘](https://pan.baidu.com/s/1NAJleETa7yJ7CLNxlb5W_w?pwd=dyeo)，提取码 `dyeo` | `dataset/nuscenes/` |
| `qtset` | INT8/PTQ 量化校准集 | [百度网盘](https://pan.baidu.com/s/1ovga4m9P4RMl6EiIQyk22g?pwd=dyeo)，提取码 `dyeo` | `compile/qtset/` |
| `input` | 板端运行输入，包括 NuScenes 输入、Vehicle sample、PointPillars 输入等 | [百度网盘](https://pan.baidu.com/s/1KQf0tBI3LTWUHfRC-Papnw?pwd=dyeo)，提取码 `dyeo` | `deploy/io/input/` |

建议解压后保持目录名不变。若路径发生变化，需要同步修改 `deploy/config/*.yaml` 和 `compile/config/**/*.toml`。

## 3. 工程结构

```text
app/
├─ compile/                         # iCraft 网络编译目录
│  ├─ config/                       # TOML 编译配置
│  │  ├─ extractor/                 # FastBEV Part1 编译配置
│  │  ├─ decoder/                   # FastBEV Part3 编译配置
│  │  ├─ matrixvt/                  # MatrixVT 编译配置
│  │  ├─ pfe/                       # PointPillars PFE 编译配置
│  │  ├─ bev_heads/                 # PointPillars BEV heads 编译配置
│  │  └─ tcp/                       # TCP 编译配置
│  ├─ fmodel/                       # 浮点模型
│  │  ├─ extractor/{nuscenes,carla,vehicle}/
│  │  ├─ decoder/{nuscenes,carla,vehicle}/
│  │  ├─ matrixvt/nuscenes/
│  │  ├─ pfe/
│  │  ├─ bev_heads/
│  │  └─ tcp/
│  ├─ logs/                         # 编译日志
│  └─ qtset/                        # 校准集，大文件从网盘下载
│
├─ dataset/                         # 数据集和预处理脚本
│  ├─ nuscenes/
│  └─ preprocess_nuscenes_merged.py
│
├─ carla/                           # CARLA 服务器端程序
│
├─ deploy/                          # 板端部署工程
│  ├─ CMakeLists.txt                # 板端 C++ 构建入口
│  ├─ config/                       # 运行 YAML
│  ├─ deps/                         # 交叉编译和运行依赖，大文件从网盘下载
│  ├─ imodel/                       # iCraft 编译后的 ZG 模型
│  ├─ io/
│  │  ├─ input/
│  │  │  ├─ data/                   # NuScenes 输入帧
│  │  │  ├─ pointpillars/           # PointPillars 点云输入和 manifest
│  │  │  └─ vehicle_sample/         # Vehicle 单帧六路图片
│  │  └─ output/
│  │     ├─ result/                 # 检测结果 txt
│  │     ├─ png/                    # PNG 可视化输出
│  │     ├─ parameter/              # 相机参数输出
│  │     ├─ pointpillars/           # PointPillars CSV 输出
│  │     └─ video/                  # 视频输出
│  ├─ script/
│  │  ├─ viewer/                    # PC 交互式 Viewer
│  │  ├─ gateway/                   # PC/服务器/开发板中转脚本
│  │  ├─ vehicle/                   # 实车六路图像采集与 Raw BGR 发送
│  │  └─ visualize/                 # PC 离线可视化工具
│  └─ src/
│     ├─ fastbev_pipeline_audio.cpp             # NuScenes + 网页 Viewer + LED/音频告警
│     ├─ fastbev_pipeline_SA.cpp                # NuScenes SA/Group4 + 本地 visualize
│     ├─ fastbev_pipeline_carla_new.cpp         # CARLA FP16 Part2 直连 Decoder
│     ├─ fastbev_pipeline_carla_new_modular.cpp # CARLA modular 双流自动驾驶
│     ├─ fastbev_pipeline_vehicle_live.cpp      # 实车 Raw BGR 实时推理
│     ├─ fastbev_pipeline_matrixvt_native.cpp   # MatrixVT 原生后处理/可视化验证
│     ├─ pointpillars.cpp                       # PointPillars 点云推理
│     ├─ tcp.cpp                                # TCP 自动驾驶网络推理
│     ├─ visualize.cpp                          # NuScenes/CARLA/MatrixVT 通用可视化
│     └─ visualize_vehicle.cpp                  # Vehicle 专用可视化
│
├─ docs/
│  ├─ API_REFERENCE.md
│  ├─ calar_guidance.md
│  ├─ pc_viewer_guidance.md
│  └─ README_TCP_CARLA_ZHUGE_STARTUP.md
│
├─ tools/
│  ├─ auto_compile.py             # PC 端 iCraft 编译入口
│  ├─ auto_cross_build.py         # Ubuntu 交叉编译和产物复制
│  ├─ auto_preprocess.py          # 数据预处理辅助入口
│  ├─ auto_generate_video.py      # PNG 转视频
│  ├─ start_board_viewer.py       # 一键启动 NuScenes/板端结果 Viewer
│  ├─ start_carla_pc_flow.py      # 一键启动 CARLA PC 侧 Gateway/Viewer
│  └─ start_vehicle_pc_flow.py    # 一键启动 Vehicle PC 侧 Viewer/采集流
```

## 4. 环境依赖

### Windows PC

- Windows 11。
- iCraft 编译工具链，可在 PowerShell 中执行 `icraft compile`。
- Python 3，推荐使用 `deploy/deps/venv/`。
- 用于 PC Viewer、Gateway、Vehicle UVC 采集和 CARLA 中转。

### Ubuntu 交叉编译环境

推荐使用共享目录，例如：

```bash
cd /mnt/hgfs/ubuntu_share/FASTBEV/app/deploy
```

常用依赖：

```bash
sudo apt update
sudo apt install -y cmake build-essential git pkg-config
```

交叉编译依赖库位于：

```text
deploy/deps/
```

### 开发板

- Ubuntu 20.04 arm64。
- ZG330AIU / iCraft Runtime。
- 对应 BOOT 位流需要与运行版本匹配。
- 若使用音频告警，需要 USB 声卡和 `aplay`。
- 若使用 PC 网页链路，需要保证 Windows 到开发板 `5200/5201` TCP 可达。

## 5. 编译与运行

### 5.1 数据预处理

NuScenes 数据预处理：

```powershell
python dataset\preprocess_nuscenes_merged.py
```

具体参数按脚本内部说明和当前数据路径调整。

### 5.2 网络编译

推荐使用统一入口：

```powershell
python tools\auto_compile.py --mode compile --dataset nuscenes --model fastbev
python tools\auto_compile.py --mode compile --dataset carla --model fastbev
python tools\auto_compile.py --mode compile --dataset vehicle --model fastbev
python tools\auto_compile.py --mode compile --dataset nuscenes --model matrixvt
```

也可以在 `compile/` 目录下直接编译 TOML。当前工程中主要 TOML 入口如下：

```powershell
cd compile

icraft compile config\extractor\nuscenes\fastbev_part1_nuscenes.toml
icraft compile config\decoder\nuscenes\fastbev_part3_nuscenes.toml

icraft compile config\extractor\carla\fastbev_part1_carla.toml
icraft compile config\decoder\carla\fastbev_part3_carla.toml
icraft compile config\decoder\carla\fastbev_part3_carla_fp16.toml

icraft compile config\extractor\vehicle_fp16\fastbev_part1_vehicle_fp16.toml
icraft compile config\decoder\vehicle_fp16\fastbev_part3_vehicle_fp16.toml

icraft compile config\matrixvt\nuscenes\matrixvt_nuscenes_fp16.toml

icraft compile config\pfe\pfe.toml
icraft compile config\bev_heads\bev_heads.toml

icraft compile config\tcp\tcp.toml
```

### 5.3 板端交叉编译

在 Ubuntu 共享目录中执行：

```bash
cd /mnt/hgfs/ubuntu_share/FASTBEV/app/deploy
rm -f build/CMakeCache.txt
cmake -S . -B build -DTARGET_CHIP=ZG
cmake --build build -j$(nproc)
```

也可以只编译某个目标：

```bash
cmake --build build -j$(nproc) --target fastbev_pipeline_audio
cmake --build build -j$(nproc) --target fastbev_pipeline_carla_new
cmake --build build -j$(nproc) --target fastbev_pipeline_vehicle_live
cmake --build build -j$(nproc) --target pointpillars
cmake --build build -j$(nproc) --target tcp
```

可执行文件会复制到 `deploy/` 根目录，之后再同步到开发板。

### 5.4 NuScenes FastBEV 版本

#### 网页交互版本

PC 端启动 Viewer：

```powershell
cd D:\github\Fastbev-for-30TAI

.\deploy\deps\venv\Scripts\python.exe tools\start_board_viewer.py `
  --host 127.0.0.1 `
  --port 8092 `
  --mode push
```

浏览器打开：

```text
http://127.0.0.1:8092/?live=1
```

开发板启动：

```bash
./fastbev_pipeline_audio ./config/fastbev_nuscenes.yaml
```

该版本用于 NuScenes 全流程演示，检测结果推送到 PC 网页，同时启用 LED/音频告警。

#### 本地 PNG 可视化版本

```bash
./fastbev_pipeline_async ./config/fastbev_nuscenes.yaml
```

输出目录：

```text
io/output/result/
io/output/png/
io/output/parameter/
```

#### SA / Group4 版本

```bash
./fastbev_pipeline_SA ./config/fastbev_nuscenes.yaml
```

该版本使用 SA/Group4 逻辑，同样通过本地 `visualize` 生成 PNG。

### 5.5 CARLA FastBEV 版本

普通 CARLA 服务：

```bash
./fastbev_pipeline_carla ./config/fastbev_carla.yaml \
  --host 0.0.0.0 \
  --port 5200 \
  --source fastbev-real-edge
```

CARLA FP16 Part2 直连 Decoder 版本：

```bash
./fastbev_pipeline_carla_new ./config/fastbev_carla_new.yaml \
  --host 0.0.0.0 \
  --port 5200 \
  --source fastbev-real-edge
```

Windows 单流 Gateway 示例：

```powershell
.\deploy\deps\venv\Scripts\python.exe .\deploy\script\gateway\gateway.py `
  --server ws://10.134.143.120:8080/gateway `
  --edge-host 192.168.125.166 `
  --edge-port 5200
```

CARLA 服务器启动、天气、地图、端口释放和 aTrust/VPN 路由问题见：

```text
docs/calar_guidance.md
```

### 5.6 CARLA New Modular 自动驾驶版本

#### 5.6.1 开发板启动双端口服务

```bash
./fastbev_pipeline_carla_new_modular ./config/fastbev_carla_new_modular.yaml \
  --host 0.0.0.0 \
  --port 5200 \
  --control-port 5201 \
  --source fastbev-real-edge
```

开发板应看到：

```text
[Listen] 0.0.0.0:5200
[ControlListen] 0.0.0.0:5201
```

#### 5.6.2 服务器端关闭旧 CARLA 进程

启动最终版 CARLA 前，先关闭旧的 simulator 和旧网页后端，释放 `2000/2001/8080`：

```bash
cd /data/$USER/carla_bev_demo
source /opt/anaconda3/etc/profile.d/conda.sh
conda activate /data/$USER/carla_bev_demo/env

pkill -f 'modular_live_drive_server|live_drive_server.py|live_drive_demo/live_drive_server.py' || true
pkill -f 'CarlaUE4-Linux-Shipping|CarlaUE4.sh' || true

ss -lntp | grep -E ':(2000|2001|8080)\b' || echo "CARLA server ports released"
```

如果仍能看到 `2000/2001/8080` 被监听，说明旧进程没有完全退出，需要先处理对应 PID。

#### 5.6.3 服务器端启动 CARLA Simulator

最终版使用 `Epic` 画质和 GPU 3：

```bash
cd /data/$USER/carla_bev_demo
source /opt/anaconda3/etc/profile.d/conda.sh
conda activate /data/$USER/carla_bev_demo/env
mkdir -p logs run

cd /data/$USER/carla_bev_demo/carla
nohup env CUDA_VISIBLE_DEVICES=3 \
  ./CarlaUE4.sh \
  -RenderOffScreen \
  -nosound \
  -quality-level=Epic \
  -carla-rpc-port=2000 \
  > /data/$USER/carla_bev_demo/logs/carla.log 2>&1 &

echo $! > /data/$USER/carla_bev_demo/run/carla.pid
echo "CARLA PID: $(cat /data/$USER/carla_bev_demo/run/carla.pid)"
```

确认 simulator 端口：

```bash
ss -lntp | grep -E ':(2000|2001)\b'
```

确认 CARLA API。该命令不带 `sleep`；如果 CARLA 还没初始化完成，会超时，等待终端端口稳定后重新执行即可：

```bash
cd /data/$USER/carla_bev_demo
source /opt/anaconda3/etc/profile.d/conda.sh
conda activate /data/$USER/carla_bev_demo/env

python - <<'PY'
import carla

client = carla.Client("127.0.0.1", 2000)
client.set_timeout(30.0)
world = client.get_world()

print("CARLA API: OK")
print("map:", world.get_map().name)
print("frame:", world.get_snapshot().frame)
PY
```

如果这里报 `AttributeError: module 'carla' has no attribute 'Client'`，通常是当前目录或 `PYTHONPATH` 里有错误的 `carla` 包遮蔽了 CARLA PythonAPI，需要确认已激活 `/data/$USER/carla_bev_demo/env`。

#### 5.6.4 服务器端启动最终版 modular 后端

确认 CARLA API 可用后，在服务器端启动循迹版后端：

```bash
cd /data/$USER/carla_bev_demo/carla
source /opt/anaconda3/etc/profile.d/conda.sh
conda activate /data/$USER/carla_bev_demo/env

python modular_live_drive_server_avoid_modified.py \
  --carla-host 127.0.0.1 \
  --town Town04_Opt \
  --http-port 8080 \
  --ego-spawn-index 36 \
  --route-csv /data/wbyin/carla_bev_demo/carla/routes/route_avoid_trim20m.csv \
  --route-raw-csv /data/wbyin/carla_bev_demo/carla/routes/route_raw_avoid_trim20m.csv \
  --remove-parked-vehicles \
  --setting-json /data/wbyin/carla_bev_demo/carla/settings/setting_66.json \
  --auto-route-speed-kmh 16 \
  --car-threshold-m 56 \
  --bus-threshold-m 20 \
  --box-position-offset-m 0.5 \
  --box-yaw-offset-deg 5
```

另开一个服务器终端确认 `8080` 和网页状态：

```bash
cd /data/$USER/carla_bev_demo
source /opt/anaconda3/etc/profile.d/conda.sh
conda activate /data/$USER/carla_bev_demo/env

ss -lntp | grep ':8080'
curl -s http://127.0.0.1:8080/status | python -m json.tool
```

#### 5.6.5 Windows 端启动双流 Gateway

```powershell
cd D:\github\Fastbev-for-30TAI

.\deploy\deps\venv\Scripts\python.exe .\deploy\script\gateway\modular_gateway.py `
  --server-perception ws://10.134.143.120:8080/gateway/perception `
  --server-control ws://10.134.143.120:8080/gateway/control `
  --edge-host 192.168.125.166 `
  --edge-port 5200 `
  --control-port 5201
```

#### 5.6.6 浏览器打开 CARLA modular 页面

```text
http://10.134.143.120:8080
```

确认 HUD 中低频 FastBEV 和高频 Radar 都在线后，按 `M` 进入 AUTO。

常用可替换项：

- `CUDA_VISIBLE_DEVICES=3`：服务器 GPU 编号。
- `10.134.143.120`：服务器 IP，Windows Gateway 和浏览器访问时需要替换。
- `192.168.125.166`：开发板 IP。
- `--route-csv` / `--route-raw-csv`：最终循迹路线文件。
- `--setting-json`：场景配置文件。
- `--auto-route-speed-kmh 16`：自动循迹目标速度。
- `--car-threshold-m 56`、`--bus-threshold-m 20`：CARLA modular 后端使用的车辆/公交车距离阈值。

更多 TCP/CARLA 自动驾驶启动细节见：

```text
docs/README_TCP_CARLA_ZHUGE_STARTUP.md
```

### 5.7 Vehicle 实车版本


#### Raw BGR 实时版本

开发板启动：

```bash
./fastbev_pipeline_vehicle_live ./config/fastbev_vehicle_live.yaml \
  --host 0.0.0.0 \
  --port 5200 \
  --source vehicle-real-edge
```

PC 端启动 Vehicle Viewer：

```powershell
.\deploy\deps\venv\Scripts\python.exe .\deploy\script\viewer\vehicle_live_viewer.py `
  --host 127.0.0.1 `
  --port 8093
```

PC 端用样例或小车六路 UVC 图像发送到开发板：

```powershell
.\deploy\deps\venv\Scripts\python.exe .\deploy\script\vehicle\run_bev_capture_vehicle_live.py `
  --edge-host 192.168.125.166 `
  --edge-port 5200 `
  --viewer http://127.0.0.1:8093
```

浏览器打开：

```text
http://127.0.0.1:8093
```

Vehicle 实时链路中，PC 负责 UVC/MJPEG 解码并发送 `640x480 Raw BGR`，开发板不再做 JPEG 解码。

### 5.8 MatrixVT 版本

MatrixVT native 后处理/可视化验证：

```bash
./fastbev_pipeline_matrixvt_native ./config/matrixvt_nuscenes_native.yaml
```

### 5.9 PointPillars 版本

81 帧 manifest 批量运行：

```bash
./pointpillars \
  --manifest ./io/input/pointpillars/manifest.csv \
  --csv-dir ./io/output/pointpillars/csv
```

批量模式每帧输出一个 CSV：

```text
io/output/pointpillars/csv/detections_0001_<sample_token>.csv
...
io/output/pointpillars/csv/detections_0081_<sample_token>.csv
```

### 5.10 TCP 自动驾驶网络

开发板启动示例：

```bash
./tcp \
  --device 'axi://zg330aiu?npu=0x40000000&dma=0x80000000' \
  --bind 0.0.0.0 \
  --port 5200
```

TCP 的服务器、CARLA 和 Gateway 联调请优先阅读：

```text
docs/README_TCP_CARLA_ZHUGE_STARTUP.md
```

## 6. 常用端口

| 端口 | 位置 | 用途 |
| --- | --- | --- |
| `5200` | 开发板 | 感知推理服务，CARLA/Vehicle/Point-to-PC 主链路常用 |
| `5201` | 开发板 | CARLA modular 高频 control 服务 |
| `8080` | 服务器 | CARLA Web 后端 |
| `8092` | PC | NuScenes/CARLA PC Viewer |
| `8093` | PC | Vehicle PC Viewer |
| `8765` | PC | 小车控制系统本地 Web Remote |
