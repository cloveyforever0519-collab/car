#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""HTTP gateway that launches CARLA official-example based demo clients."""

from __future__ import annotations

import glob
import json
import os
import signal
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, Optional


APP_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = Path(os.environ.get("CARLA_PROJECT_ROOT", APP_DIR.parent)).resolve()
LOG_DIR = Path(os.environ.get("OFFICIAL_DEMO_LOG_DIR", PROJECT_ROOT / "logs"))
STATUS_FILE = Path(os.environ.get("OFFICIAL_GATEWAY_STATUS_FILE", LOG_DIR / "official_gateway_status.json"))
UDP_STATUS_FILE = Path(os.environ.get("OFFICIAL_UDP_STATUS_FILE", LOG_DIR / "official_udp_status.json"))
VIEW_COMMAND_FILE = Path(os.environ.get("OFFICIAL_VIEW_COMMAND_FILE", LOG_DIR / "official_view_command.json"))

API_HOST = os.environ.get("CARLA_API_HOST", "0.0.0.0")
API_PORT = int(os.environ.get("CARLA_API_PORT", os.environ.get("API_PORT", "8765")))
CARLA_HOST = os.environ.get("CARLA_HOST", "127.0.0.1")
CARLA_PORT = int(os.environ.get("CARLA_PORT", "2000"))
DISPLAY_RES = os.environ.get("OFFICIAL_DEMO_RES", "1920x1080")
COMMAND_COOLDOWN_SEC = float(os.environ.get("OFFICIAL_COMMAND_COOLDOWN_SEC", "25.0"))
TERMINATE_TIMEOUT_SEC = float(os.environ.get("OFFICIAL_TERMINATE_TIMEOUT_SEC", "20.0"))
RELAUNCH_GAP_SEC = float(os.environ.get("OFFICIAL_RELAUNCH_GAP_SEC", "4.0"))
ALGO_TELEMETRY_PORT = int(os.environ.get("OFFICIAL_ALGO_TELEMETRY_PORT", "5500"))
MANUAL_TELEMETRY_PORT = int(os.environ.get("OFFICIAL_MANUAL_TELEMETRY_PORT", "5501"))
SENSOR_UDP_PORTS = os.environ.get("OFFICIAL_SENSOR_UDP_PORTS", "5010")
CAMERA_STREAM_URL = os.environ.get("OFFICIAL_CAMERA_STREAM_URL", "official-pygame://camera")
SIDE_STREAM_PORT = int(os.environ.get("OFFICIAL_SIDE_STREAM_PORT", "8771"))
SIDE_RTSP_HOST = os.environ.get("OFFICIAL_SIDE_RTSP_HOST", os.environ.get("MIRROR_RTSP_PUBLIC_HOST", "192.168.110.100"))
SIDE_RTSP_PORT = int(os.environ.get("OFFICIAL_SIDE_RTSP_PORT", os.environ.get("MIRROR_RTSP_PORT", "8554")))
SIDE_CAMERA_STREAMS = {
    "rear_left": os.environ.get("OFFICIAL_REAR_LEFT_STREAM_URL", f"rtsp://{SIDE_RTSP_HOST}:{SIDE_RTSP_PORT}/carla_rear_left"),
    "rear_right": os.environ.get("OFFICIAL_REAR_RIGHT_STREAM_URL", f"rtsp://{SIDE_RTSP_HOST}:{SIDE_RTSP_PORT}/carla_rear_right"),
    "birdview": os.environ.get("OFFICIAL_BIRDVIEW_STREAM_URL", f"rtsp://{SIDE_RTSP_HOST}:{SIDE_RTSP_PORT}/carla_birdview"),
}


def env_flag(name: str, default: str = "0") -> bool:
    return os.environ.get(name, default).strip().lower() in {"1", "true", "yes", "on"}


ALLOW_AIGO_TRAFFIC = env_flag("OFFICIAL_ALLOW_AIGO_TRAFFIC", "0")

FRONTEND_VEHICLES = (
    "Dodge Charger",
    "Lincoln MKZ",
    "Tesla Model 3",
    "Audi e-tron",
    "Jeep Wrangler",
    "Tesla Cybertruck",
    "Fuso Rosa",
    "Mercedes Sprinter",
    "Volkswagen T2",
    "Carlacola Truck",
    "European HGV",
    "Firetruck",
)

FRONTEND_SCENES = {"Town01", "Town02", "Town03", "Town04", "Town05", "TrainingGround"}
FRONTEND_WEATHERS = {"Sunny", "Cloudy", "Light Rain", "Heavy Rainstorm", "Fog/Dense Fog", "Clear"}
FRONTEND_TIMES = {"Noon", "Sunset", "Late Night"}
VIEW_LABELS = {
    "driver": "驾驶员第一视角",
    "follow": "第三人称跟车视角",
}


def add_carla_paths() -> None:
    py_tag = f"py{sys.version_info.major}.{sys.version_info.minor}"
    candidates = [
        os.environ.get("CARLA_PYTHONAPI"),
        "/home/z/Workspace/carla_hil_project/PythonAPI/carla",
        "/home/zhang/Workspace/carla_hil_project/PythonAPI/carla",
        str(PROJECT_ROOT / "PythonAPI" / "carla"),
    ]
    for base in candidates:
        if not base:
            continue
        base_path = Path(base)
        if base_path.exists() and str(base_path) not in sys.path:
            sys.path.append(str(base_path))
        for egg in glob.glob(str(base_path / "dist" / f"carla-*{py_tag}*.egg")):
            if egg not in sys.path:
                sys.path.insert(0, egg)
                return


try:
    import carla  # type: ignore
except ImportError:
    add_carla_paths()
    try:
        import carla  # type: ignore
    except Exception:
        carla = None  # type: ignore


SCENE_ALIASES = {
    "Town01/Urban City District": "Town01",
    "Town02/Low-Density Suburban Area": "Town02",
    "Town03/High-Density Residential Zone": "Town03",
    "Town04/High-Speed Expressway": "Town04",
    "Town05/Performance Proving Ground": "Town05",
    "Town04Forest": "TrainingGround",
    "训练场": "TrainingGround",
}

MODE_LABELS = {
    "AI": "官方 pygame 画面 + CARLA Traffic Manager",
    "AIGO": "官方 pygame 画面 + AIGO UDP 控制",
    "Manual": "官方 pygame 画面 + 硬件 UDP 控制",
}

ROUTE_SEGMENTS = ("AB", "BC", "CA")


def first_value(body: Dict[str, Any], *keys: str, default: Any = None) -> Any:
    for key in keys:
        if key in body and body.get(key) is not None:
            return body.get(key)
    return default


def frontend_zero_means_on(value: Any, default: str = "1") -> bool:
    text = str(value if value is not None else default).strip().lower()
    return text in {"0", "true", "yes", "y", "load", "loaded", "on", "enable", "enabled", "已加载", "加载"}


def empty_sensor_summary(enabled: bool = False) -> Dict[str, Any]:
    return {
        "enabled": enabled,
        "lidar_points": 0,
        "radar_targets": 0,
        "nearest_obstacle_m": 0.0,
        "imu_accel_norm_m_s2": 0.0,
        "heading_deg": 0.0,
        "gnss": {
            "latitude": 0.0,
            "longitude": 0.0,
            "altitude_m": 0.0,
        },
        "updated_at": None,
    }


def sensor_frontend_summary(sensor_data: Dict[str, Any]) -> Dict[str, Any]:
    if not sensor_data:
        return empty_sensor_summary(False)
    summary = sensor_data.get("frontend_summary")
    if isinstance(summary, dict):
        return summary
    return empty_sensor_summary(bool(sensor_data.get("enabled")))


def normalize_scene(value: Any) -> str:
    text = str(value or "Town02").strip()
    return SCENE_ALIASES.get(text, text if text in FRONTEND_SCENES else "Town02")


def normalize_vehicle(value: Any) -> str:
    text = str(value or "Lincoln MKZ").strip()
    return text if text in FRONTEND_VEHICLES else "Lincoln MKZ"


def normalize_weather(value: Any) -> str:
    text = str(value or "Sunny").strip()
    return text if text in FRONTEND_WEATHERS else "Sunny"


def normalize_time_of_day(value: Any) -> str:
    text = str(value or "Noon").strip()
    return text if text in FRONTEND_TIMES else "Noon"


def camera_view_label(view: Any) -> str:
    return "Driver first-person view" if normalize_view(view) == "driver" else "Third-person chase view"


def normalize_mode(value: Any) -> str:
    text = str(value or "AI").strip()
    lower = text.lower()
    if "aigo" in lower or "algo" in lower or "算法" in text or "自动驾驶" in text or "域控" in text:
        return "AIGO"
    if "manual" in lower or "hil" in lower or "手动" in text or "硬件" in text:
        return "Manual"
    return "AI"


def normalize_view(value: Any) -> str:
    raw = str(value or "follow").strip()
    text = raw.lower().replace("-", "_")
    if "driver" in text or "first" in text or "cockpit" in text or "驾驶" in raw or "第一" in raw or "座舱" in raw:
        return "driver"
    if "follow" in text or "third" in text or "chase" in text or "rear" in text or "back" in text:
        return "follow"
    if "第三" in raw or "跟车" in raw or "后" in raw or "外部" in raw:
        return "follow"
    return "follow"


def normalize_route_segment(value: Any) -> str:
    text = str(value or "").strip().upper().replace("-", "").replace("_", "").replace(" ", "")
    aliases = {
        "A2B": "AB",
        "ATOB": "AB",
        "B2C": "BC",
        "BTOC": "BC",
        "C2A": "CA",
        "CTOA": "CA",
    }
    text = aliases.get(text, text)
    return text if text in ROUTE_SEGMENTS else ""


def decode_route_command(body: Dict[str, Any]) -> Dict[str, Any]:
    segment = normalize_route_segment(
        first_value(body, "segment", "route_segment", "routeSegment", "route", "Route", default="")
    )
    return {
        "scene": normalize_scene(first_value(body, "scene", "Scene", default="Town02")),
        "weather": "Sunny",
        "time_of_day": "Noon",
        "mode": "AI",
        "vehicle": "Tesla Model 3",
        "camera_view": normalize_view(first_value(body, "camera_view", "view", "View", "视角", "camera", default="follow")),
        "load_traffic": False,
        "load_sensors": frontend_zero_means_on(
            first_value(body, "loadingsensor", "loadingSensors", "load_sensors", "sensor", default="1")
        ),
        "traffic_forced_off": False,
        "route_segment": segment,
        "route_shortcut": True,
    }


def decode_command(body: Dict[str, Any]) -> Dict[str, Any]:
    traffic_value = str(first_value(body, "loadingtransportation", "Traffic Load", default="1")).strip()
    mode = normalize_mode(first_value(body, "drive_mode", "Drive Mode", default="AI"))
    load_traffic = traffic_value.lower() in {"0", "true", "yes", "y", "load", "loaded", "on", "已加载", "加载"}
    load_sensors = frontend_zero_means_on(
        first_value(
            body,
            "loadingsensor",
            "loadingSensors",
            "load_sensors",
            "sensorload",
            "Sensor Load",
            "sensors",
            "sensor",
            default="1",
        )
    )
    route_segment = ""
    traffic_forced_off = False
    if mode == "AIGO" and load_traffic and not ALLOW_AIGO_TRAFFIC:
        load_traffic = False
        traffic_forced_off = True
    return {
        "scene": normalize_scene(first_value(body, "scene", "Scene", default="Town02")),
        "weather": normalize_weather(first_value(body, "sky", "Weather Condition", default="Sunny")),
        "time_of_day": normalize_time_of_day(first_value(body, "sunshinetime", "Sunshine Time", default="Noon")),
        "mode": mode,
        "vehicle": normalize_vehicle(first_value(body, "vehiclemodel", "Vehicle Model", default="Lincoln MKZ")),
        "camera_view": normalize_view(first_value(body, "camera_view", "Camera View", "view", "View", "视角", "camera", default="follow")),
        "load_traffic": load_traffic,
        "load_sensors": load_sensors,
        "traffic_forced_off": traffic_forced_off,
        "route_segment": route_segment,
    }


def read_json(path: Path) -> Optional[Dict[str, Any]]:
    try:
        if path.exists():
            return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return None
    return None


def write_json(path: Path, data: Dict[str, Any]) -> None:
    LOG_DIR.mkdir(exist_ok=True)
    tmp = path.with_name(f"{path.name}.{os.getpid()}.{threading.get_ident()}.{time.time_ns()}.tmp")
    tmp.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
    os.replace(tmp, path)


def write_view_command(view: str) -> Dict[str, Any]:
    view = normalize_view(view)
    now = time.time()
    previous = read_json(VIEW_COMMAND_FILE) or {}
    revision = int(previous.get("revision", 0) or 0) + 1
    payload = {
        "camera_view": view,
        "revision": revision,
        "updated_at": now,
        "updated_at_text": time.strftime("%Y-%m-%d %H:%M:%S"),
    }
    write_json(VIEW_COMMAND_FILE, payload)

    status = read_json(UDP_STATUS_FILE) or {}
    status.update({
        "camera_view_requested": view,
        "camera_view_command_revision": revision,
        "camera_view_command_updated_at": now,
    })
    write_json(UDP_STATUS_FILE, status)
    return payload


def terminate(proc: Optional[subprocess.Popen]) -> None:
    if not proc:
        return
    try:
        if proc.poll() is not None:
            return
        if os.name != "nt":
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            except Exception:
                proc.terminate()
        else:
            proc.terminate()
        try:
            proc.wait(timeout=TERMINATE_TIMEOUT_SEC)
        except Exception:
            if os.name != "nt":
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except Exception:
                    proc.kill()
            else:
                proc.kill()
            try:
                proc.wait(timeout=3.0)
            except Exception:
                pass
    except Exception:
        pass


def process_cmdline(pid: int) -> str:
    if os.name == "nt":
        return ""
    try:
        raw = Path(f"/proc/{int(pid)}/cmdline").read_bytes()
        return raw.replace(b"\x00", b" ").decode("utf-8", errors="ignore")
    except Exception:
        return ""


def terminate_known_child(pid: Any) -> None:
    try:
        pid_int = int(pid)
    except Exception:
        return
    if pid_int <= 1:
        return
    cmdline = process_cmdline(pid_int)
    allowed_tokens = (
        "official_udp_vehicle_client.py",
        "aigo_port_wrapper.py",
        "can_tcp_bridge_vcu.py",
        "can_wheel_bridge_vcu.py",
        "vdanyi.py",
        "vjiansu.py",
        "vjiasu.py",
        "vshexing.py",
        "vshuangyi.py",
    )
    if os.name != "nt" and not any(token in cmdline for token in allowed_tokens):
        return
    try:
        if os.name != "nt":
            try:
                os.killpg(os.getpgid(pid_int), signal.SIGTERM)
            except Exception:
                os.kill(pid_int, signal.SIGTERM)
            deadline = time.time() + max(3.0, min(TERMINATE_TIMEOUT_SEC, 10.0))
            while time.time() < deadline:
                try:
                    os.kill(pid_int, 0)
                    time.sleep(0.05)
                except OSError:
                    return
            try:
                os.killpg(os.getpgid(pid_int), signal.SIGKILL)
            except Exception:
                os.kill(pid_int, signal.SIGKILL)
    except Exception:
        pass


def cleanup_status_children() -> None:
    status = read_json(UDP_STATUS_FILE) or {}
    terminate_known_child(status.get("udp_pid"))
    terminate_known_child(status.get("algo_pid"))
    terminate_known_child(status.get("bridge_pid"))


class Runtime:
    def __init__(self) -> None:
        self.lock = threading.RLock()
        self.proc: Optional[subprocess.Popen] = None
        self.log_file = None
        self.log_path: Optional[Path] = None
        self.mode = "idle"
        self.command: Dict[str, Any] = {}
        self.target_ip = "127.0.0.1"
        self.last_result: Dict[str, Any] = {"ok": False, "msg": "idle"}
        self.started_at = 0.0

    def stop(self) -> None:
        with self.lock:
            terminate(self.proc)
            cleanup_status_children()
            self.proc = None
            if self.log_file:
                try:
                    self.log_file.close()
                except Exception:
                    pass
            self.log_file = None
            self.log_path = None
            self.mode = "idle"
            write_json(UDP_STATUS_FILE, {
                "running": False,
                "mode": self.mode,
                "vehicle_alive": False,
                "udp_pid": None,
                "speed_kmh": 0.0,
                "updated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            })
            self.last_result = {"ok": True, "msg": "stopped"}

    def start(self, decoded: Dict[str, Any], target_ip: str) -> Dict[str, Any]:
        with self.lock:
            now = time.time()
            running = bool(self.proc and self.proc.poll() is None)
            udp_status = read_json(UDP_STATUS_FILE) or {}
            warming_up = running and now - self.started_at < COMMAND_COOLDOWN_SEC
            vehicle_not_ready = running and not bool(udp_status.get("vehicle_alive")) and now - self.started_at < 45.0
            route_switch = bool(decoded.get("route_segment")) and decoded.get("route_segment") != self.command.get("route_segment")
            if route_switch:
                warming_up = False
                vehicle_not_ready = False
            if warming_up or vehicle_not_ready:
                write_view_command(decoded["camera_view"])
                self.last_result = {
                    "ok": True,
                    "msg": "current demo still warming up; START ignored",
                    "pid": self.proc.pid if self.proc else None,
                    "mode": self.mode,
                }
                return self.last_result
            if running and decoded == self.command:
                write_view_command(decoded["camera_view"])
                self.last_result = {
                    "ok": True,
                    "msg": "same demo already running",
                    "pid": self.proc.pid if self.proc else None,
                    "mode": self.mode,
                }
                return self.last_result

            was_running = running
            self.stop()
            if was_running and RELAUNCH_GAP_SEC > 0:
                time.sleep(RELAUNCH_GAP_SEC)
            LOG_DIR.mkdir(exist_ok=True)
            write_json(UDP_STATUS_FILE, {
                "running": False,
                "mode": decoded["mode"],
                "scene": decoded["scene"],
                "vehicle_alive": False,
                "speed_kmh": 0.0,
                "camera_view": decoded["camera_view"],
                "frontend_telemetry": {},
                "legacy_telemetry": {},
                "sensor_data": {
                    "enabled": bool(decoded.get("load_sensors")),
                    "frontend_summary": empty_sensor_summary(bool(decoded.get("load_sensors"))),
                },
                "updated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            })
            write_view_command(decoded["camera_view"])
            env = os.environ.copy()
            env["CARLA_PROJECT_ROOT"] = str(PROJECT_ROOT)
            env["OFFICIAL_UDP_STATUS_FILE"] = str(UDP_STATUS_FILE)
            env["OFFICIAL_VIEW_COMMAND_FILE"] = str(VIEW_COMMAND_FILE)
            env["OFFICIAL_ENABLE_SENSORS"] = "1" if decoded.get("load_sensors") else "0"
            sensor_mode_disable = "0" if decoded.get("load_sensors") else "1"
            env["OFFICIAL_AIGO_DISABLE_SENSORS"] = sensor_mode_disable
            env["OFFICIAL_AI_BASELINE_DISABLE_SENSORS"] = sensor_mode_disable
            env.setdefault("SDL_VIDEODRIVER", "x11")

            cmd = [
                sys.executable,
                str(APP_DIR / "official_udp_vehicle_client.py"),
                "--host",
                CARLA_HOST,
                "--port",
                str(CARLA_PORT),
                "--scene",
                decoded["scene"],
                "--vehicle",
                decoded["vehicle"],
                "--mode",
                decoded["mode"],
                "--weather",
                decoded["weather"],
                "--time-of-day",
                decoded["time_of_day"],
                "--camera-view",
                decoded["camera_view"],
                "--res",
                DISPLAY_RES,
                "--target-ip",
                target_ip,
                "--algo-telemetry-port",
                str(ALGO_TELEMETRY_PORT),
                "--manual-telemetry-port",
                str(MANUAL_TELEMETRY_PORT),
            ]
            if decoded.get("route_segment"):
                cmd.extend(["--route-segment", str(decoded.get("route_segment", ""))])
            if decoded["mode"] == "Manual":
                cmd.append("--start-bridge")
            if decoded["load_traffic"]:
                cmd.append("--load-traffic")
            cwd = PROJECT_ROOT

            log_path = LOG_DIR / f"official_demo_{decoded['mode'].lower()}_{int(time.time())}.log"
            log_file = open(log_path, "a", encoding="utf-8")
            kwargs: Dict[str, Any] = {
                "cwd": str(cwd),
                "stdout": log_file,
                "stderr": subprocess.STDOUT,
                "env": env,
            }
            if os.name != "nt":
                kwargs["preexec_fn"] = os.setsid
            proc = subprocess.Popen(cmd, **kwargs)
            self.proc = proc
            self.log_file = log_file
            self.log_path = log_path
            self.mode = decoded["mode"]
            self.command = dict(decoded)
            self.target_ip = target_ip
            self.started_at = time.time()
            status = read_json(UDP_STATUS_FILE) or {}
            status.update({
                "running": True,
                "mode": decoded["mode"],
                "scene": decoded["scene"],
                "vehicle_alive": False,
                "udp_pid": proc.pid,
                "camera_view_requested": decoded["camera_view"],
                "updated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            })
            write_json(UDP_STATUS_FILE, status)
            self.last_result = {
                "ok": True,
                "msg": "official demo launched",
                "pid": proc.pid,
                "mode": decoded["mode"],
                "target_ip": target_ip,
                "load_traffic": decoded["load_traffic"],
                "load_sensors": decoded.get("load_sensors", False),
                "traffic_forced_off": decoded.get("traffic_forced_off", False),
                "route_segment": decoded.get("route_segment", ""),
            }
            return self.last_result

    def snapshot(self) -> Dict[str, Any]:
        with self.lock:
            running = bool(self.proc and self.proc.poll() is None)
            udp_status = read_json(UDP_STATUS_FILE)
            telemetry = (udp_status or {}).get("frontend_telemetry", {}) if udp_status else {}
            sensor_data = (udp_status or {}).get("sensor_data", {"enabled": False}) if udp_status else {"enabled": False}
            sensor_summary = sensor_frontend_summary(sensor_data)
            health = {
                "ok": True,
                "backend": "official_demo_gateway",
                "running": running,
                "pid": self.proc.pid if running and self.proc else None,
                "mode": self.mode,
                "drive_mode": MODE_LABELS.get(self.mode, self.mode),
                "last_command": self.command,
                "target_ip": self.target_ip,
                "last_result": self.last_result,
                "log_path": str(self.log_path) if self.log_path else None,
                "camera_view": self.command.get("camera_view", "follow"),
                "camera_view_label": "驾驶员第一视角" if self.command.get("camera_view") == "driver" else "第三人称跟车视角",
                "camera_stream_url": CAMERA_STREAM_URL,
                "camera_streams": {
                    "active": CAMERA_STREAM_URL,
                    "follow": CAMERA_STREAM_URL,
                    "driver": CAMERA_STREAM_URL,
                    "rear": CAMERA_STREAM_URL,
                },
                "side_camera_streams": SIDE_CAMERA_STREAMS,
                "vehicle_alive": False,
                "world": None,
                "paused": False,
                "telemetry": telemetry,
                "sensor_data": sensor_data,
                "sensor_summary": sensor_summary,
                "route": (udp_status or {}).get("route") if udp_status else None,
                "diagnostics": {"actor_id": None, "speed_kmh": 0.0, "control": None},
                "udp_client": udp_status,
            }
            if udp_status:
                health["vehicle_alive"] = bool(udp_status.get("vehicle_alive"))
                health["world"] = udp_status.get("world")
                health["camera_view"] = udp_status.get("camera_view", health["camera_view"])
                health["camera_view_label"] = "驾驶员第一视角" if health["camera_view"] == "driver" else "第三人称跟车视角"
                health["diagnostics"] = {
                    "actor_id": udp_status.get("actor_id"),
                    "speed_kmh": udp_status.get("speed_kmh", 0.0),
                    "control": udp_status.get("last_control"),
                    "telemetry_ports": udp_status.get("telemetry_ports"),
                    "sensor_udp_ports": udp_status.get("sensor_udp_ports"),
                    "sensor_tx_count": udp_status.get("sensor_tx_count"),
                    "algo_telemetry_port": udp_status.get("algo_telemetry_port"),
                    "manual_telemetry_port": udp_status.get("manual_telemetry_port"),
                }
            return health


runtime = Runtime()


class Handler(BaseHTTPRequestHandler):
    server_version = "OfficialCarlaDemoGateway/1.0"

    def log_message(self, fmt: str, *args: Any) -> None:
        print(f"[official-gateway] {self.client_address[0]} {fmt % args}", flush=True)

    def send_json(self, status: int, payload: Dict[str, Any]) -> None:
        data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.send_header("Access-Control-Allow-Methods", "GET,POST,OPTIONS")
        self.end_headers()
        self.wfile.write(data)

    def read_body(self) -> Dict[str, Any]:
        length = int(self.headers.get("Content-Length", "0") or "0")
        raw = self.rfile.read(length) if length else b"{}"
        return json.loads(raw.decode("utf-8")) if raw else {}

    def do_OPTIONS(self) -> None:
        self.send_json(200, {"ok": True})

    def do_GET(self) -> None:
        path = self.path.split("?", 1)[0]
        if path in {"/", "/health"}:
            snap = runtime.snapshot()
            write_json(STATUS_FILE, snap)
            self.send_json(200, snap)
        elif path == "/telemetry":
            snap = runtime.snapshot()
            self.send_json(200, {
                "ok": True,
                "target_ip": snap.get("target_ip"),
                "drive_mode": snap.get("drive_mode"),
                "camera_view": snap.get("camera_view"),
                "camera_view_label": snap.get("camera_view_label"),
                "camera_stream_url": snap.get("camera_stream_url"),
                "camera_streams": snap.get("camera_streams"),
                "side_camera_streams": snap.get("side_camera_streams"),
                "paused": False,
                "vehicle_alive": snap.get("vehicle_alive"),
                "telemetry": snap.get("telemetry", {}),
                "sensor_data": snap.get("sensor_data", {"enabled": False}),
                "sensor_summary": snap.get("sensor_summary", empty_sensor_summary(False)),
                "route": snap.get("route"),
                "diagnostics": snap.get("diagnostics", {}),
            })
        elif path == "/sensors":
            snap = runtime.snapshot()
            self.send_json(200, {
                "ok": True,
                "target_ip": snap.get("target_ip"),
                "running": snap.get("running"),
                "vehicle_alive": snap.get("vehicle_alive"),
                "world": snap.get("world"),
                "camera_view": snap.get("camera_view"),
                "sensor_data": snap.get("sensor_data", {"enabled": False}),
                "sensor_summary": snap.get("sensor_summary", empty_sensor_summary(False)),
                "route": snap.get("route"),
                "diagnostics": snap.get("diagnostics", {}),
            })
        elif path in {"/route", "/routes"}:
            snap = runtime.snapshot()
            self.send_json(200, {
                "ok": True,
                "endpoint": "/route",
                "segments": list(ROUTE_SEGMENTS),
                "defaults": {
                    "scene": "Town02",
                    "weather": "Sunny",
                    "time_of_day": "Noon",
                    "mode": "AI",
                    "vehicle": "Tesla Model 3",
                    "camera_view": "follow",
                    "load_traffic": False,
                    "load_sensors": False,
                },
                "example": {"segment": "AB", "scene": "Town02", "loadingsensor": "1"},
                "route": snap.get("route"),
            })
        else:
            self.send_json(404, {"ok": False, "msg": "endpoint not found"})

    def do_POST(self) -> None:
        path = self.path.split("?", 1)[0]
        if path not in {"/command", "/view", "/route"}:
            self.send_json(404, {"ok": False, "msg": "endpoint not found"})
            return
        try:
            body = self.read_body()
        except Exception as exc:
            self.send_json(400, {"ok": False, "msg": f"invalid json: {exc}"})
            return

        if path == "/view":
            view_keys = ("camera_view", "view", "View", "视角")
            if not any(key in body for key in view_keys):
                snap = runtime.snapshot()
                snap.update({
                    "ok": False,
                    "msg": "missing camera_view/view; ignored to avoid accidental mouse-click view changes",
                })
                self.send_json(400, snap)
                return
            view = normalize_view(first_value(body, "camera_view", "view", "View", "视角", default="follow"))
            command = write_view_command(view)
            with runtime.lock:
                runtime.command["camera_view"] = view
            snap = runtime.snapshot()
            snap.update({
                "ok": True,
                "msg": f"view switched to {view}",
                "camera_view": view,
                "camera_view_label": "驾驶员第一视角" if view == "driver" else "第三人称跟车视角",
                "camera_view_command": command,
            })
            self.send_json(200, snap)
            return

        sendstate = str(first_value(body, "sendstate", default="START")).upper()
        if sendstate in {"STOP", "END", "QUIT"}:
            runtime.stop()
            snap = runtime.snapshot()
            write_json(STATUS_FILE, snap)
            self.send_json(200, snap)
            return

        decoded = decode_route_command(body) if path == "/route" else decode_command(body)
        if path == "/route" and not decoded.get("route_segment"):
            self.send_json(400, {"ok": False, "msg": "route segment must be one of AB, BC, CA"})
            return
        target_ip = self.client_address[0] if self.client_address[0] != "127.0.0.1" else "127.0.0.1"
        try:
            result = runtime.start(decoded, target_ip)
            snap = runtime.snapshot()
            write_json(STATUS_FILE, snap)
            self.send_json(202, {
                "ok": True,
                "msg": result["msg"],
                "target_ip": target_ip,
                "camera_view": decoded["camera_view"],
                "camera_view_label": "驾驶员第一视角" if decoded["camera_view"] == "driver" else "第三人称跟车视角",
                "camera_stream_url": CAMERA_STREAM_URL,
                "camera_streams": {
                    "active": CAMERA_STREAM_URL,
                    "follow": CAMERA_STREAM_URL,
                    "driver": CAMERA_STREAM_URL,
                    "rear": CAMERA_STREAM_URL,
                },
                "side_camera_streams": SIDE_CAMERA_STREAMS,
                "vehicle_alive": snap["vehicle_alive"],
                "telemetry": snap.get("telemetry", {}),
                "sensor_data": snap.get("sensor_data", {"enabled": False}),
                "sensor_summary": snap.get("sensor_summary", empty_sensor_summary(False)),
                "route": snap.get("route"),
                "last_result": result,
            })
        except Exception as exc:
            self.send_json(500, {"ok": False, "msg": str(exc)})


class Server(ThreadingHTTPServer):
    allow_reuse_address = True


def main() -> None:
    os.chdir(PROJECT_ROOT)
    LOG_DIR.mkdir(exist_ok=True)
    cleanup_status_children()
    server = Server((API_HOST, API_PORT), Handler)
    print(f"[official-gateway] listening on http://{API_HOST}:{API_PORT}", flush=True)
    try:
        server.serve_forever()
    finally:
        runtime.stop()
        server.server_close()


if __name__ == "__main__":
    main()
