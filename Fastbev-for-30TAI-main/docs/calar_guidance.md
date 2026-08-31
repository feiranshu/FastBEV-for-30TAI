# CARLA 服务器端运行指南

本文记录 CARLA 服务器端的启动、检查和关闭命令。文件名沿用项目约定写作 `calar_guidance`，正文统一使用正确名称 `CARLA`。

## 1. 启动 CARLA Simulator

服务器端先启动 CARLA 仿真。以下命令使用当前联调环境的默认路径和参数：

```bash
cd /data/$USER/carla_bev_demo
source /opt/anaconda3/etc/profile.d/conda.sh
conda activate /data/$USER/carla_bev_demo/env
mkdir -p logs run

cd /data/$USER/carla_bev_demo/carla
nohup ./CarlaUE4.sh /Game/Carla/Maps/Town10HD_Opt \
  -RenderOffScreen \
  -nosound \
  -quality-level=Epic \
  -carla-rpc-port=2000 \
  '-ini:[/Script/Engine.RendererSettings]:r.GraphicsAdapter=3' \
  > /data/$USER/carla_bev_demo/logs/carla_town10.log 2>&1 &
echo $! > /data/$USER/carla_bev_demo/run/carla.pid
```

可替换项：

- `/data/$USER/carla_bev_demo`：CARLA 联调工程在服务器上的实际路径。
- `/data/$USER/carla_bev_demo/carla`：包含 `CarlaUE4.sh` 的 CARLA Simulator 安装目录，可按实际安装位置替换；它不是本仓库的 `carla/` 脚本目录。
- `/Game/Carla/Maps/Town10HD_Opt`：仿真地图，可替换为服务器已安装的 CARLA 地图。
- `-quality-level=Epic`：画质等级，可替换为 `Low`、`Epic` 等；画质越高，服务器 GPU 压力和图像编码压力越大。
- `-carla-rpc-port=2000`：CARLA RPC 端口；如果改这里，下面 `--carla-port` 必须同步修改。
- `r.GraphicsAdapter=3`：服务器使用的 GPU 编号，按 `nvidia-smi` 中的 GPU 序号调整。

## 2. 确认 CARLA API 可用

```bash
cd /data/$USER/carla_bev_demo
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

可替换项：

- `127.0.0.1`：CARLA Simulator 所在主机；同机运行时保持 `127.0.0.1`。
- `2000`：CARLA RPC 端口，必须与 `-carla-rpc-port` 一致。

## 3. 启动 CARLA 网页后端

确认 CARLA API 可用后，在服务器项目根目录启动网页后端：

```bash
cd /data/$USER/carla_bev_demo
python -u carla/live_drive_server.py \
  --carla-host 127.0.0.1 \
  --carla-port 2000 \
  --town Town10HD_Opt \
  --http-host 0.0.0.0 \
  --http-port 8080 \
  --traffic-vehicles 20 \
  --traffic-walkers 20 \
  --freeze-traffic \
  --max-speed-kmh 20 \
  --max-result-age-seconds 3.0 \
  --min-score 0.185
```


可替换项：

- `/data/$USER/carla_bev_demo`：CARLA 联调工程在服务器上的实际路径。
- `--carla-host 127.0.0.1`：CARLA Simulator 所在主机；同机运行时保持 `127.0.0.1`。
- `--carla-port 2000`：必须与 CARLA Simulator 的 `-carla-rpc-port` 一致。
- `--town Town10HD_Opt`：必须与当前 CARLA 地图匹配。
- `--http-host 0.0.0.0`：允许外部 PC 访问服务器网页；本机调试可改为 `127.0.0.1`。
- `--http-port 8080`：CARLA 网页和 Gateway WebSocket 端口；如果修改，PC 端 `--server ws://...:8080/gateway` 也要同步修改。
- `--traffic-vehicles 20`、`--traffic-walkers 20`：生成车辆和行人数量。
- `--freeze-traffic`：冻结交通参与者；删除该参数后车辆/行人会运动。
- `--max-speed-kmh 30`：自车最高速度。
- `--min-score 0.185`：服务器叠加显示的最低 score 阈值。

CARLA 内置检测框会保持显示，直到下一次被接受的检测结果到达并整体刷新。
旧参数 `--box-visible-seconds` 仅为兼容历史启动命令而保留，当前不再控制框的显示时长。

## 4. 确认网页后端 JSON 状态

```bash
curl -s http://127.0.0.1:8080/status | python -m json.tool
```

可替换项：

- `127.0.0.1:8080`：网页后端地址；端口必须与 `--http-port` 一致。服务器本机检查用 `127.0.0.1`，PC 浏览器访问时用服务器 IP。

PC 浏览器访问 CARLA 控制页面：

```text
http://10.134.143.120:8080/
```

可替换项：`10.134.143.120:8080` 必须与 CARLA 服务器 IP 和 `--http-port` 一致。

## 5. 关闭 CARLA 服务器端进程和端口

```bash
cd /data/$USER/carla_bev_demo

# 关闭 live_drive_server.py，占用 8080/gateway 的服务器后端
pkill -f 'carla/live_drive_server.py|live_drive_demo/live_drive_server.py' || true

# 关闭 CARLA Simulator，占用 2000/2001
pkill -f 'CarlaUE4-Linux-Shipping|CarlaUE4.sh' || true

# 确认服务器端口已释放
ss -lntp | grep -E ':(2000|2001|8080)\b' || echo "CARLA server ports released"
```

可替换项：

- `2000/2001`：CARLA Simulator 端口，需与 `-carla-rpc-port` 对应。
- `8080`：CARLA 网页后端端口，需与 `--http-port` 对应。

## 6. aTrust 连接后开发板停在 Listen 的处理

典型现象：

- 开发板已显示 `[Listen] 0.0.0.0:5200`，但没有出现 `[Client]` 和推理结果。
- 强制从开发板网卡地址 Ping 可以成功：

  ```powershell
  ping -S 192.168.125.100 192.168.125.166
  ```

- 普通 TCP 检查失败，而且错误输出中的 `SourceAddress` 是 aTrust 虚拟地址，而不是 `192.168.125.100`：

  ```powershell
  Test-NetConnection 192.168.125.166 -Port 5200
  ```

使用下面的命令检查实际路由：

```powershell
Find-NetRoute -RemoteIPAddress 192.168.125.166 |
Format-List InterfaceIndex,InterfaceAlias,IPAddress,DestinationPrefix,NextHop
```

如果结果走向 aTrust 虚拟网卡，例如使用 `192.168.125.128/25` 路由和 `10.230.*` 源地址，说明 aTrust 的更具体路由覆盖了开发板物理网卡的 `/24` 直连路由。`192.168.125.166` 位于 `192.168.125.128/25` 范围内，因此 Windows 会优先选择该 `/25` 路由。

保持 aTrust 已连接，在**管理员 PowerShell**中执行：

```powershell
$boardIf = (
  Get-NetIPAddress -AddressFamily IPv4 |
  Where-Object IPAddress -eq "192.168.125.100"
).InterfaceIndex

$boardIf

New-NetRoute `
  -DestinationPrefix "192.168.125.166/32" `
  -InterfaceIndex $boardIf `
  -NextHop 0.0.0.0 `
  -RouteMetric 1 `
  -PolicyStore ActiveStore
```

当前联调环境中，`192.168.125.100` 对应的物理网卡索引曾为 `22`；应始终以上面命令实时查询到的 `$boardIf` 为准，不要固定使用历史索引。

添加后验证：

```powershell
Find-NetRoute -RemoteIPAddress 192.168.125.166 |
Format-List InterfaceIndex,InterfaceAlias,IPAddress,DestinationPrefix,NextHop

Test-NetConnection 192.168.125.166 -Port 5200
```

正确结果应满足：

```text
IPAddress          : 192.168.125.100
DestinationPrefix  : 192.168.125.166/32
NextHop            : 0.0.0.0
SourceAddress      : 192.168.125.100
TcpTestSucceeded   : True
```

TCP 验证通过后，重新启动 PC Gateway：

```powershell
.\deploy\deps\venv\Scripts\python.exe tools\start_carla_pc_flow.py `
  --server ws://10.134.143.120:8080/gateway `
  --edge-host 192.168.125.166 `
  --edge-port 5200
```

`Test-NetConnection` 会短暂连接一次开发板的 5200 端口后断开，因此开发板可能短暂打印一次 `[Client]`，属于正常现象。

该 `/32` 路由存放在 `ActiveStore`。aTrust 重连、网卡复位或 Windows 重启后可能消失；再次出现开发板停在 `[Listen]` 时，应首先重新执行 `Find-NetRoute`，确认流量仍通过 `192.168.125.100` 的物理网卡发送。



PC TAI GPU