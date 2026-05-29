# Native CARLA Demo Controller

这个目录是新方案的可搬运包。它不走 RTSP/ffplay 视频流，直接使用 CARLA 原生 Unreal 窗口显示画面，并用 spectator 跟随车辆。

## 要搬哪些内容

把整个 `native_carla_demo` 文件夹搬到目标机器的项目根目录：

```bash
$HOME/Carla_Project/native_carla_demo
```

目标项目根目录仍然需要保留这些旧资产：

- `output/`：车辆 JSON 配置。
- `can_tcp_bridge_vcu.py`：硬件台架桥接。
- `vdanyi.py`、`vjiansu.py`、`vjiasu.py`、`vshexing.py`、`vshuangyi.py`：AIGO 算法脚本。
- `can/智能座舱CAN协议-NJ0515.dbc`：硬件桥 DBC。
- `delivery/` 和 systemd user services：继续负责启动 CARLA 和 backend。

## 本地 z 机器测试

```bash
cd ~/Carla_Project
chmod +x native_carla_demo/*.sh
./native_carla_demo/use_native_backend.sh
```

发 CARLA 原生 AI：

```bash
./native_carla_demo/send_ai_town02.sh
```

发 AIGO：

```bash
./native_carla_demo/send_aigo_town02.sh
```

发硬件手动：

```bash
./native_carla_demo/send_manual_town02.sh
```

查状态：

```bash
./native_carla_demo/health.sh
```

停 demo：

```bash
./native_carla_demo/stop_demo.sh
```

## 关键变化

- 显示：CARLA 原生窗口，不再经过 camera sensor -> ffmpeg -> RTSP -> player。
- 前端：继续使用 `POST /command`、`POST /view`、`GET /health`。
- AIGO：继续收 `127.0.0.1:5000` 遥测，发 `127.0.0.1:5001` 控制。
- 硬件：继续使用 `can_tcp_bridge_vcu.py`，控制端口仍是 `5001`。
- AI：使用 CARLA Traffic Manager 原生自动驾驶。
