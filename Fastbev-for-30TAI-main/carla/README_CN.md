# CARLA WebRTC + 六路 nuScenes 相机实时驾驶服务

## 服务器端安装

```bash
cd /data/wbyin/carla_bev_demo
source /opt/anaconda3/etc/profile.d/conda.sh
conda activate /data/wbyin/carla_bev_demo/env
python -m pip install -r live_drive_demo/requirements-live.txt
```

## 启动

CARLA Server 必须已监听 2000/2001。只能由本程序推进同步仿真时钟，不要同时运行旧的
`nuscenes_rig_capture*.py` 或其他调用 `world.tick()` 的客户端。

```bash
python live_drive_demo/live_drive_server.py --http-port 8080
```

Windows 浏览器打开 `http://10.134.143.120:8080/`。

服务启动后会同时完成三件事：

- 以 20 Hz 推进 CARLA，并向浏览器发送第三人称 WebRTC 视频；
- 以 2 Hz 采集严格同一 CARLA frame 的六路 nuScenes 相机并编码为 1600×900 JPEG；
- 仅缓存最新的完整六相机二进制批次，等待 Windows 网关按需取走。

状态检查：

```bash
curl -s http://127.0.0.1:8080/status | python -m json.tool
```

正常时 `batch.frame_id` 与 `six_camera.encoded_batches` 会持续增长。

## 控制

- W：加速
- S：制动
- A/D：转向
- Space：手刹
- Q：切换前进/倒车
- C：切换第三人称视角

控制消息以 30 Hz 发送。超过 500 ms 未收到控制状态，服务器会自动松开油门并制动。

默认的 `LONG LOW CHASE` 位于车辆后方 5.8 m、高 2.15 m，俯角 -2°，FOV 85°，
用于比旧 LOW CHASE 显示更远的前方道路。
