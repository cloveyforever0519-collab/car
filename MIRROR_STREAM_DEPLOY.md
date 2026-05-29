# CARLA 双后视镜 RTSP 视频流部署说明

## 输出地址

- 左后视镜：`rtsp://192.168.110.100:8554/carla_rear_left`
- 右后视镜：`rtsp://192.168.110.100:8554/carla_rear_right`

视频参数默认：`1920x1080`、`30fps`、`H264`。

## 依赖

Ubuntu 主机需要：

```bash
ffmpeg -version
ffmpeg -encoders | grep -E "h264_nvenc|libx264"
```

还需要一个 RTSP 服务。推荐 MediaMTX：

```bash
mkdir -p ~/Carla_Project/bin
# 将 mediamtx 可执行文件放到 ~/Carla_Project/bin/mediamtx
chmod +x ~/Carla_Project/bin/mediamtx
```

如果系统 PATH 里已经有 `mediamtx`，也可以不用放到 `bin/`。

## 启动

```bash
cd ~/Carla_Project
chmod +x delivery/start_mirror_stream.sh
systemctl --user daemon-reload
systemctl --user enable carla-video.service
systemctl --user restart carla-video.service
systemctl --user status carla-video.service --no-pager
```

## 验证

在 192.168.110.103 或同网段机器上用 VLC/ffplay 拉流：

```bash
ffplay -rtsp_transport tcp rtsp://192.168.110.100:8554/carla_rear_left
ffplay -rtsp_transport tcp rtsp://192.168.110.100:8554/carla_rear_right
```

在 CARLA 主机上看状态：

```bash
cat ~/Carla_Project/logs/mirror_stream_status.json
journalctl --user -u carla-video.service -n 120 --no-pager
tail -f ~/Carla_Project/logs/mirror_carla_rear_left.ffmpeg.log
tail -f ~/Carla_Project/logs/mirror_carla_rear_right.ffmpeg.log
```

## 配置

修改 `delivery/mirror_stream.env`：

- `MIRROR_WIDTH` / `MIRROR_HEIGHT`
- `MIRROR_FPS`
- `MIRROR_BITRATE`
- `MIRROR_ENCODER`: `auto` / `h264_nvenc` / `libx264`
- `MIRROR_HFLIP`: 后视镜水平翻转，默认 `0`
