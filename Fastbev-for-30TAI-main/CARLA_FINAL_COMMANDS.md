# 最终版 CARLA Modular 启动命令

本文记录最终版 CARLA modular 自动驾驶链路的关闭旧进程、启动、API 确认、Gateway 和浏览器访问命令。正文统一使用正确名称 `CARLA`。

当前默认链路：

```text
CARLA Simulator 2000/2001
-> modular_live_drive_server_avoid_modified.py 8080
-> Windows modular_gateway.py
-> 开发板 fastbev_pipeline_carla_new_modular 5200/5201
```

## 1. 开发板启动双端口服务

在开发板 `deploy` 目录运行：

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

确认端口：

```bash
ss -lntp | grep -E ':(5200|5201)\b'
```

## 2. 服务器关闭旧 CARLA 进程

在服务器执行，释放 `2000/2001/8080`：

```bash
cd /data/$USER/carla_bev_demo
source /opt/anaconda3/etc/profile.d/conda.sh
conda activate /data/$USER/carla_bev_demo/env

pkill -f 'modular_live_drive_server|live_drive_server.py|live_drive_demo/live_drive_server.py' || true
pkill -f 'CarlaUE4-Linux-Shipping|CarlaUE4.sh' || true

ss -lntp | grep -E ':(2000|2001|8080)\b' || echo "CARLA server ports released"
```

如果仍能看到 `2000/2001/8080` 被监听，说明旧进程没有完全退出，需要先处理对应 PID。

## 3. 服务器启动 CARLA Simulator

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

如果启动失败，查看日志：

```bash
tail -n 120 /data/$USER/carla_bev_demo/logs/carla.log
```

## 4. 服务器确认 CARLA API

不使用 `sleep`。如果 CARLA 尚未初始化完成，该命令会超时，稍后重新执行即可：

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

如果报：

```text
AttributeError: module 'carla' has no attribute 'Client'
```

通常是当前目录或 `PYTHONPATH` 里有错误的 `carla` 包遮蔽了 CARLA PythonAPI。先确认已激活：

```bash
conda activate /data/$USER/carla_bev_demo/env
```

## 5. 服务器启动最终版 Modular 后端

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

## 6. 服务器确认网页后端

另开一个服务器终端执行：

```bash
cd /data/$USER/carla_bev_demo
source /opt/anaconda3/etc/profile.d/conda.sh
conda activate /data/$USER/carla_bev_demo/env

ss -lntp | grep ':8080'
curl -s http://127.0.0.1:8080/status | python -m json.tool
```

## 7. Windows 启动双流 Gateway

在 Windows 项目根目录运行：

```powershell
cd D:\github\Fastbev-for-30TAI

.\deploy\deps\venv\Scripts\python.exe .\deploy\script\gateway\modular_gateway.py `
  --server-perception ws://10.134.143.120:8080/gateway/perception `
  --server-control ws://10.134.143.120:8080/gateway/control `
  --edge-host 192.168.125.166 `
  --edge-port 5200 `
  --control-port 5201
```

期望看到：

```text
[LOW] connected ...
[HIGH] connected ...
```

## 8. 浏览器打开 CARLA Modular 页面

```text
http://10.134.143.120:8080
```

确认 HUD 中低频 FastBEV 和高频 Radar 都在线后，按 `M` 进入 AUTO。

## 9. 常用可替换项

- `CUDA_VISIBLE_DEVICES=3`：服务器 GPU 编号。
- `10.134.143.120`：服务器 IP，Windows Gateway 和浏览器访问时需要替换。
- `192.168.125.166`：开发板 IP。
- `--route-csv` / `--route-raw-csv`：最终循迹路线文件。
- `--setting-json`：场景配置文件。
- `--auto-route-speed-kmh 16`：自动循迹目标速度。
- `--car-threshold-m 56`：车辆阈值。
- `--bus-threshold-m 20`：公交车阈值。
- `--box-position-offset-m 0.5`：检测框位置修正。
- `--box-yaw-offset-deg 5`：检测框朝向修正。

## 10. 快速排查

开发板是否监听：

```bash
ss -lntp | grep -E ':(5200|5201)\b'
```

服务器 CARLA 是否监听：

```bash
ss -lntp | grep -E ':(2000|2001)\b'
```

服务器后端是否监听：

```bash
ss -lntp | grep ':8080'
curl -s http://127.0.0.1:8080/status | python -m json.tool
```

Windows 到开发板 TCP 是否可达：

```powershell
Test-NetConnection 192.168.125.166 -Port 5200
Test-NetConnection 192.168.125.166 -Port 5201
```

Windows 到服务器是否可达：

```powershell
Test-NetConnection 10.134.143.120 -Port 8080
```
