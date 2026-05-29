# CARLA Delivery Autostart

This directory contains the delivery autostart and watchdog stack.

## Services

- `carla-engine.service`: starts CARLA first.
- `carla-backend.service`: waits for CARLA, then starts `main_gui.py`.
- `delivery-watchdog.timer`: checks health periodically and restarts failed parts.

## Install

```bash
cd ~/Carla_Project
bash delivery/install_user_services.sh
```

Start immediately:

```bash
systemctl --user start carla-engine.service
systemctl --user start carla-backend.service
systemctl --user start delivery-watchdog.timer
```

Check:

```bash
bash delivery/status.sh
curl http://192.168.110.100:8765/health
```

## Important

CARLA is a graphical Unreal application. The most stable delivery setup is:

1. Ubuntu boots into kernel `6.8.0-110-generic`.
2. User `zhang` auto-logs into the desktop.
3. User systemd services start CARLA and backend.

Use an Xorg session when possible. If the login screen lets you choose between
Wayland and Xorg, choose Ubuntu on Xorg. CARLA/Unreal behaves more predictably
with `DISPLAY=:0` under Xorg for unattended delivery.

If you use `loginctl enable-linger`, services may start before the graphical
display exists. Keep desktop auto-login enabled for the CARLA host.

The fixed delivery IP should remain configured on the wired connection:

```bash
ip -br addr
nmcli con show "有线连接 1" | grep ipv4.addresses
```

## Logs

```bash
journalctl --user -u carla-engine.service -f
journalctl --user -u carla-backend.service -f
journalctl --user -u delivery-watchdog.service -f
```

Project logs are under:

```bash
~/Carla_Project/logs/delivery
```
