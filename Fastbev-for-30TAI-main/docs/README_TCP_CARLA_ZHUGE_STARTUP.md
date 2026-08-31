# TCP CARLA + PC Gateway + ZhuGe NPU 启动 README

本文档记录当前已经联调通过的启动流程：

```text
CARLA 服务器
    ↓ WebSocket :8080
Windows PC Gateway
    ↓ TCP :5200
诸葛开发板 / Icraft ZG330
    ↓
控制结果原路返回
```

当前地址：

```text
服务器 IP：10.134.143.120
服务器 WebSocket：8080
服务器 Dashboard：8081
开发板 IP：192.168.125.166
开发板 TCP：5200
```

## 1. 启动顺序

```text
1. 开发板启动 tcp
2. 服务器启动 CARLA
3. 服务器启动 Leaderboard + TCP agent
4. 验证服务器 8080
5. PC 验证 8080 和 5200
6. PC 启动 gateway.py
7. 浏览器打开 Dashboard
```

## 2. 开发板：启动 Icraft runtime

```bash
cd /mnt/hgfs/ubuntu_share/FASTBEV/app/deploy

ls -lh ./imodel/tcp/tcp_ZG.json
ls -lh ./imodel/tcp/tcp_ZG.raw

./tcp \
  --device 'axi://zg330aiu?npu=0x40000000&dma=0x80000000' \
  --bind 0.0.0.0 \
  --port 5200
```

正常应看到：

```text
[Init] Icraft session applied; rgb=FP32 NHWC [1,256,900,3], state=FP32 [1,9]
[BOARD] listening on 0.0.0.0:5200
```

另开开发板终端：

```bash
ss -lntp | grep 5200
```

应看到 `0.0.0.0:5200`。

## 3. 服务器终端 A：启动 CARLA

```bash
conda activate tcp_carla09101

export CARLA_ROOT=/data/wbyin/carla_09101
export CARLA_EGG=$CARLA_ROOT/PythonAPI/carla/dist/carla-0.9.10-py3.7-linux-x86_64.egg
export PYTHONPATH=$CARLA_ROOT/PythonAPI:$CARLA_ROOT/PythonAPI/carla:$CARLA_EGG:${PYTHONPATH:-}

cd /data/wbyin/tcp_carla_onnx_rpc_demo

bash server_carla/run_carla_server.sh
```

保持终端运行。

### 验证 CARLA

另开服务器终端：

```bash
conda activate tcp_carla09101

export CARLA_ROOT=/data/wbyin/carla_09101
export CARLA_EGG=$CARLA_ROOT/PythonAPI/carla/dist/carla-0.9.10-py3.7-linux-x86_64.egg
export PYTHONPATH=$CARLA_ROOT/PythonAPI:$CARLA_ROOT/PythonAPI/carla:$CARLA_EGG:${PYTHONPATH:-}

cd /data/wbyin/tcp_carla_onnx_rpc_demo

python server_carla/check_carla_api.py \
  --host 127.0.0.1 \
  --port 2000
```

正常：

```text
carla distribution: carla 0.9.10
connected: Town03
settings: WorldSettings(...)
```

## 4. 服务器终端 B：启动 Leaderboard + TCP agent

```bash
conda activate tcp_carla09101

export CARLA_ROOT=/data/wbyin/carla_09101
export CARLA_EGG=$CARLA_ROOT/PythonAPI/carla/dist/carla-0.9.10-py3.7-linux-x86_64.egg
export PYTHONPATH=$CARLA_ROOT/PythonAPI:$CARLA_ROOT/PythonAPI/carla:$CARLA_EGG:${PYTHONPATH:-}

export TCP_ROOT=/data/wbyin/TCP_onnx/TCP
export DEMO_ROOT=/data/wbyin/tcp_carla_onnx_rpc_demo

cd $DEMO_ROOT

unset ROUTE_SCENARIO

bash server_carla/run_evaluation_rpc.sh
```

注意：当前版本不要使用：

```bash
export ROUTE_SCENARIO=0
```

否则当前 Leaderboard fork 会出现：

```text
ZeroDivisionError: float division by zero
```

正常情况下 agent 会启动：

```text
ws://0.0.0.0:8080/gateway
http://0.0.0.0:8080/status
http://0.0.0.0:8081/
```

## 5. 服务器终端 C：验证 8080 / 8081

```bash
ss -lntp | grep -E '8080|8081'
```

再执行：

```bash
curl http://127.0.0.1:8080/status
```

应该返回 JSON，不能出现：

```text
Connection refused
```

## 6. Windows PC：验证网络

```powershell
Test-NetConnection 10.134.143.120 -Port 8080
Test-NetConnection 192.168.125.166 -Port 5200
```

两条都应看到：

```text
TcpTestSucceeded : True
```

## 7. Windows PC：启动 Gateway

```powershell
cd D:\github\Fastbev-for-30TAI\task\tcp_carla_onnx_rpc_demo\pc_gateway

python gateway.py `
  --server ws://10.134.143.120:8080/gateway `
  --board-host 192.168.125.166 `
  --board-port 5200
```

第一次联调建议抓第一帧：

```powershell
python gateway.py `
  --server ws://10.134.143.120:8080/gateway `
  --board-host 192.168.125.166 `
  --board-port 5200 `
  --dump-first-frame .\first_frame
```

会生成：

```text
first_frame/
├── rgb.jpg
└── frame.json
```

## 8. 正常日志

PC Gateway：

```text
[SERVER] connected: ws://10.134.143.120:8080/gateway
[BOARD] connected to 192.168.125.166:5200
[RX] frame=... CRC=OK
[TX] frame=... steer=... throttle=... brake=...
```

开发板：

```text
[BOARD] PC gateway connected: ...
[FRAME] id=... jpeg=... control=(...) decode=... pre=... icraft=... post=... total=... ms
```

看到持续 `[FRAME]` 基本说明：

```text
CARLA
  ↓
JPEG + state
  ↓
PC Gateway
  ↓
TCP1 parse
  ↓
JPEG decode
  ↓
BGR -> RGB
  ↓
ImageNet normalize
  ↓
rgb FP32 NHWC [1,256,900,3]
state FP32 [1,9]
  ↓
Icraft / ZG330
  ↓
pred_wp / mu / sigma
  ↓
Beta + PID + fusion
  ↓
steer / throttle / brake
  ↓
PC
  ↓
CARLA
```

## 9. 模型输入输出契约

```text
输入：
rgb   float32 [1,256,900,3] NHWC RGB
state float32 [1,9]

输出：
pred_wp        float32 [1,4,2]
mu_branches    float32 [1,2]
sigma_branches float32 [1,2]
```

不要把 RGB 输入改成 NCHW。

## 10. 查看画面

### Dashboard

Windows 浏览器打开：

```text
http://10.134.143.120:8081
```

### 查看实际送给开发板的图像

使用 `--dump-first-frame` 后打开：

```text
first_frame\rgb.jpg
```

应为 `900 x 256`。

## 11. 常用状态检查

服务器 CARLA：

```bash
python server_carla/check_carla_api.py --host 127.0.0.1 --port 2000
```

服务器 WebSocket：

```bash
ss -lntp | grep 8080
curl http://127.0.0.1:8080/status
```

开发板：

```bash
ss -lntp | grep 5200
```

Windows：

```powershell
Test-NetConnection 10.134.143.120 -Port 8080
Test-NetConnection 192.168.125.166 -Port 5200
```

## 12. 停止 Leaderboard / TCP agent

如果 `Ctrl+C` 后没有完全退出：

```bash
ps -ef | grep -E "leaderboard_evaluator|run_evaluation_rpc|tcp_onnx_rpc_agent" | grep -v grep
```

杀 evaluator：

```bash
pkill -f leaderboard_evaluator.py
pkill -f run_evaluation_rpc.sh
```

检查端口：

```bash
ss -lntp | grep -E "8080|8081"
```

如果仍被占用：

```bash
sudo fuser -k 8080/tcp
sudo fuser -k 8081/tcp
```

## 13. 停止 CARLA

如果只停评测，不停 CARLA：

```bash
pkill -f leaderboard_evaluator.py
```

如果整个仿真全部停止：

```bash
pkill -f leaderboard_evaluator.py
pkill -f run_evaluation_rpc.sh
pkill -f CarlaUE4
pkill -f run_carla_server.sh
```

## 14. 常见问题

### PC 出现 getaddrinfo failed

不要写：

```text
ws://服务器IP:8080/gateway
```

要写真实地址：

```text
ws://10.134.143.120:8080/gateway
```

### PC 显示“远程计算机拒绝网络连接”

服务器检查：

```bash
ss -lntp | grep 8080
curl http://127.0.0.1:8080/status
```

如果没有监听，说明 Leaderboard / agent 没启动成功。

### Leaderboard 出现 ZeroDivisionError

如果日志里有：

```text
routeScenario='0'
ZeroDivisionError: float division by zero
```

执行：

```bash
unset ROUTE_SCENARIO
bash server_carla/run_evaluation_rpc.sh
```

### Gateway 反复 connected / disconnected

按顺序检查：

```text
1. 服务器 8080 是否稳定监听
2. 开发板 5200 是否稳定监听
3. 开发板是否出现 Icraft inference failed
4. PC 是否出现 CRC 错误
5. 是否有单帧 inference 超时
```

### 服务器出现 RPC FAIL / timed out

检查同一个 `frame_id` 是否能在：

```text
服务器
PC Gateway
开发板
```

三端对应起来，同时检查开发板 `[FRAME]` 是否持续产生。

## 15. 最终检查表

```text
[开发板]
5200 LISTEN
Icraft Session applied
持续出现 [FRAME]

[服务器]
CARLA 2000 正常
8080 LISTEN
8081 LISTEN
/gateway connected

[PC]
SERVER connected
BOARD connected
CRC=OK
持续出现 TX

[CARLA]
车辆开始运动
```

以上全部满足，即表示服务器 -> PC -> 开发板 NPU -> PC -> CARLA 闭环正常。
