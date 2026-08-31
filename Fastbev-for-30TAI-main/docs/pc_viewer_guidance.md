# PC Viewer 运行指南

本文记录 PC 端交互式 Viewer 和 Gateway 的启动、访问和关闭命令。

## 1. NuScenes 板端结果 Viewer

用于 `fastbev_pipeline_audio`：开发板推送 NuScenes 结果到 PC，PC 网页实时展示检测框，并配合 LED/音频告警版本演示。

在项目根目录运行：

```powershell
.\deploy\deps\venv\Scripts\python.exe tools\start_board_viewer.py `
  --host 127.0.0.1 `
  --port 8092 `
  --mode push
```

浏览器打开：

```text
http://127.0.0.1:8092/?live=1
```

可替换项：

- `.\deploy\deps\venv\Scripts\python.exe`：PC 端 Python 虚拟环境路径。
- `--host 127.0.0.1`：PC Viewer 监听地址；本机浏览器访问保持 `127.0.0.1`。
- `--port 8092`：PC Viewer 端口；如果修改，浏览器地址也要同步修改。
- `--mode push`：等待开发板推送结果；本地循环预览可改为 `loop`，CARLA 数据流使用 `carla`。

## 2. CARLA Viewer + Gateway

用于 `fastbev_pipeline_carla`：PC 从 CARLA 服务器接收六路 JPEG，经 Gateway 转发给开发板 `5200`，再把开发板结果推回 PC Viewer 和 CARLA 服务器。

在项目根目录运行：

```powershell
.\deploy\deps\venv\Scripts\python.exe tools\start_carla_pc_flow.py `
  --server ws://10.134.143.120:8080/gateway `
  --edge-host 192.168.125.166 `
  --edge-port 5200 `
  --viewer-host 127.0.0.1 `
  --viewer-port 8092
```

PC Viewer 浏览器打开：

```text
http://127.0.0.1:8092/?live=1
```

CARLA 浏览器控制页面打开：

```text
http://10.134.143.120:8080/
```

可替换项：

- `.\deploy\deps\venv\Scripts\python.exe`：PC 端 Python 虚拟环境路径。
- `--server ws://10.134.143.120:8080/gateway`：CARLA 服务器 WebSocket 地址；其中 `10.134.143.120` 替换为服务器 IP，`8080` 必须与服务器 `--http-port` 一致。
- `--edge-host 192.168.125.166`：开发板 IP。
- `--edge-port 5200`：开发板 BEV1 服务端口，必须与开发板 `fastbev_pipeline_carla --port` 一致。
- `--viewer-host 127.0.0.1`、`--viewer-port 8092`：PC 本地交互式 Viewer 地址和端口。

## 3. 关闭 PC Viewer + Gateway

```powershell
Get-NetTCPConnection -State Listen -LocalPort 8092 -ErrorAction SilentlyContinue |
  Select-Object -ExpandProperty OwningProcess -Unique |
  ForEach-Object { Stop-Process -Id $_ -Force }

Get-CimInstance Win32_Process |
  Where-Object {
    $_.Name -match 'python' -and
    $_.CommandLine -match 'start_carla_pc_flow|start_board_viewer|pc_ps_live_pipeline_server|gateway.py'
  } |
  ForEach-Object { Stop-Process -Id $_.ProcessId -Force }
```

可替换项：

- `8092`：PC Viewer 端口，需与 `--viewer-port` 或 `start_board_viewer.py --port` 对应。

## 4. 直接运行底层脚本

通常优先使用 `tools/start_board_viewer.py` 和 `tools/start_carla_pc_flow.py`。需要单独调试时，可直接运行底层脚本：

```powershell
.\deploy\deps\venv\Scripts\python.exe deploy\script\viewer\pc_ps_live_pipeline_server.py --host 127.0.0.1 --port 8092 --mode push
```

```powershell
.\deploy\deps\venv\Scripts\python.exe deploy\script\gateway\gateway.py --server ws://10.134.143.120:8080/gateway --edge-host 192.168.125.166 --edge-port 5200
```

```powershell
.\deploy\deps\venv\Scripts\python.exe deploy\script\gateway\mock_edge.py --host 127.0.0.1 --port 5200 --inference-ms 500 --jitter-ms 120 --synthetic-boxes
```
