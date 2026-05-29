# Local Comparison Test

This folder is meant to be copied to:

```bash
/home/z/Carla_Project
```

It assumes CARLA is installed at:

```bash
/home/z/Workspace/carla_hil_project
```

The conda environment is:

```bash
carla_vcu
```

## First Check

```bash
cd /home/z/Carla_Project
chmod +x local_manual_test.sh delivery/*.sh delivery/wait_for_carla.py
conda activate carla_vcu
python3 -m py_compile main_gui.py carla_mirror_stream.py vdanyi.py vshexing.py vshuangyi.py
```

## Start Without Video First

This isolates CARLA + backend + map/vehicle deployment from video load.

```bash
cd /home/z/Carla_Project
START_VIDEO=0 START_CENTER_DISPLAY=0 START_WATCHDOG=0 bash local_manual_test.sh start
bash local_manual_test.sh smoke
```

## Start Full Local Stack

```bash
cd /home/z/Carla_Project
bash local_manual_test.sh stop
START_VIDEO=1 START_CENTER_DISPLAY=1 START_WATCHDOG=1 bash local_manual_test.sh start
bash local_manual_test.sh smoke
```

## Status / Stop

```bash
bash local_manual_test.sh status
bash local_manual_test.sh stop
```

The script uses user systemd services for a close comparison with the demo host,
but keeps every unit disabled so nothing starts on boot.

## If RTSP Video Fails

`carla-video.service` needs `mediamtx`. Put the binary here:

```bash
/home/z/Carla_Project/bin/mediamtx
```

or install `mediamtx` into `PATH`.
