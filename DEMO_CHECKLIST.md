# Demo Checklist

## 1. 演示机环境

- 项目目录: `~/Carla_Project`
- CARLA 目录: `~/CARLA_0.9.15`
- conda 环境: `carla_vcu`

## 2. 启动顺序

1. 打开终端
2. 进入 CARLA 目录并启动 Carla 引擎
3. 新开一个终端
4. 进入 `~/Carla_Project`
5. 执行:

```bash
bash start_demo.sh
```

## 3. 页面访问

- Streamlit 调试页默认端口: `8501`
- 后端接口:
  - 指令入口: `POST /command`
  - 健康检查: `GET /health`

## 4. 安卓大屏六项字段

- `scene`
- `sky`
- `sunshinetime`
- `drive_mode`
- `loadingtransportation`
- `vehiclemodel`

说明:

- `loadingtransportation = "0"` 表示加载全量交通参与者
- `loadingtransportation = "1"` 表示不加载交通参与者

## 5. 展示前自检

访问健康检查接口，确认返回:

- `api = running`
- `carla_connected = true`

推荐命令:

```bash
curl http://127.0.0.1:8765/health
```

## 6. 现场兜底

如果安卓大屏现场下发异常，可用本机直接发送一条标准指令:

```bash
curl -X POST http://127.0.0.1:8765/command \
  -H "Content-Type: application/json" \
  -d '{"scene":"Town01","sky":"Light Rain","sunshinetime":"Sunset","drive_mode":"Manual","loadingtransportation":"0","vehiclemodel":"Lincoln MKZ"}'
```

## 7. 日志位置

- 算法脚本日志目录: `./logs/`

## 8. 页面新增状态区

Streamlit 页面里新增了:

- “最近一次安卓大屏指令 / 后端执行结果”

可直接查看:

- 最近一次解码后的六项指令
- 最近一次后端执行结果

