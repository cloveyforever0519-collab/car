# official_carla_demo

这是当前交付主链路：官方 CARLA pygame 画面 + 旧前端接口 + AI/AIGO/Manual + 后视镜 MJPEG + 车辆遥测 + 传感器摘要。

它不再使用旧 RTSP、ffplay、center-display、mediamtx、delivery-watchdog 视频链路。

## 前端接口

前端接口保持旧约定：

- `POST /command`
- `POST /view`
- `POST /route`
- `GET /health`
- `GET /telemetry`
- `GET /sensors`

示例：

```json
{
  "sendstate": "START",
  "scene": "Town04",
  "sky": "Sunny",
  "sunshinetime": "Noon",
  "drive_mode": "AIGO",
  "loadingtransportation": "0",
  "vehiclemodel": "Tesla Model 3",
  "camera_view": "follow"
}
```

`loadingtransportation` 当前约定：

- `"0"`：加载交通
- `"1"`：不加载交通

## 视角

- `follow`、`rear`、`third`：第三人称跟车视角
- `driver`、`first`、`cockpit`：驾驶员第一视角
- 后视镜不是主画面 `rear`，它在 `side_camera_streams.rear_left/rear_right`

## 关键端口

- HTTP API：`8765`
- 手动/AIGO 控制入口：UDP `5001`
- 外部车辆数据：UDP `5000/5002/5003`
- 传感器摘要：UDP `5010`
- AIGO 内部遥测：UDP `127.0.0.1:5500`
- 手动桥内部遥测：UDP `127.0.0.1:5501`
- 左右后视镜：HTTP MJPEG `8771`

不要监听或占用 `5001`，它是控制口。

## 快速启动

```bash
cd ~/Carla_Project
chmod +x official_carla_demo/*.sh official_carla_demo/runtime/*.sh official_carla_demo/runtime/wait_for_carla.py
./official_carla_demo/use_official_backend.sh
```

强制重启 CARLA：

```bash
cd ~/Carla_Project
CARLA_FORCE_RESTART=1 ./official_carla_demo/use_official_backend.sh
```

停止当前 demo：

```bash
./official_carla_demo/stop_demo.sh
```

## 开机自启

```bash
cd ~/Carla_Project
chmod +x official_carla_demo/*.sh official_carla_demo/runtime/*.sh official_carla_demo/runtime/wait_for_carla.py
./official_carla_demo/install_autostart.sh
systemctl --user restart carla-engine.service
systemctl --user restart carla-backend.service
```

检查：

```bash
systemctl --user status carla-engine.service --no-pager -l
systemctl --user status carla-backend.service --no-pager -l
curl -sS http://127.0.0.1:8765/health | python3 -m json.tool
```

取消自启：

```bash
cd ~/Carla_Project
./official_carla_demo/uninstall_autostart.sh
```

## 传感器

车辆生成后默认创建：

- 64 线 LiDAR
- 前向 Radar
- GNSS
- IMU
- 4 路 Ultrasonic/Obstacle

传感器输出走：

- `GET /sensors`
- `GET /telemetry` 里的 `sensor_data`
- UDP `5010`

默认限流：

```bash
OFFICIAL_ENABLE_SENSORS=1
OFFICIAL_SENSOR_UDP_PORTS=5010
OFFICIAL_SENSOR_SUMMARY_HZ=5
OFFICIAL_SENSOR_UDP_HZ=5
OFFICIAL_SENSOR_SAMPLE_LIMIT=2048
OFFICIAL_SENSOR_LIDAR_MAX_PPS=300000
OFFICIAL_SENSOR_LIDAR_MAX_HZ=5
```

临时关闭传感器：

```bash
OFFICIAL_ENABLE_SENSORS=0 ./official_carla_demo/use_official_backend.sh
```

监听传感器 UDP：

```bash
python3 official_carla_demo/listen_sensor_udp.py
```

## 后视镜

本机：

```bash
xdg-open http://127.0.0.1:8771/rear_left.mjpg
xdg-open http://127.0.0.1:8771/rear_right.mjpg
curl -sS http://127.0.0.1:8771/health | python3 -m json.tool
```

外部后视镜设备：

```text
http://192.168.110.106:8771/rear_left.mjpg
http://192.168.110.107:8771/rear_right.mjpg
```

## 常用测试

```bash
./official_carla_demo/send_ai_town02.sh
./official_carla_demo/send_aigo_town02.sh
./official_carla_demo/send_manual_town02.sh
./official_carla_demo/view_follow.sh
./official_carla_demo/view_driver.sh
./official_carla_demo/test_sensor_local.sh
```

Windows 远程调试参考：

```powershell
.\official_carla_demo\test_sensor_remote.ps1
```

## Frontend Sensor And Route Shortcuts

`loadingsensor` uses the same convention as `loadingtransportation`:

- `"0"`: mount all sensors
- `"1"`: do not mount sensors; `sensor_summary` still returns the same fields with zero values

Route shortcut is isolated from normal `/command`. It always launches AI route
mode with Sunny/Noon, Tesla Model 3, third-person view, and no traffic:

```bash
curl -sS -X POST http://127.0.0.1:8765/route \
  -H "Content-Type: application/json" \
  -d '{"scene":"Town02","segment":"AB","loadingsensor":"1"}' \
  | python3 -m json.tool
```

Valid route segments are `AB`, `BC`, and `CA`. Route mode uses CARLA BasicAgent
destination navigation; it does not fall back to random TrafficManager driving.
