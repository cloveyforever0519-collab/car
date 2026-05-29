#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Native CARLA demo controller.

This file is intentionally standalone: it keeps the frontend HTTP API and the
existing UDP hardware/algorithm contracts, but uses CARLA's native Unreal
viewport plus spectator transforms for display. No RTSP, ffmpeg, or ffplay is
used in the main demo path.
"""

from __future__ import annotations

import json
import math
import os
import random
import signal
import socket
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple

try:
    import carla  # type: ignore
except Exception as exc:  # pragma: no cover - target machine owns CARLA install
    carla = None  # type: ignore
    CARLA_IMPORT_ERROR = exc
else:
    CARLA_IMPORT_ERROR = None


APP_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = Path(os.environ.get("CARLA_PROJECT_ROOT", APP_DIR.parent)).resolve()
ROOT = PROJECT_ROOT
LOG_DIR = Path(os.environ.get("NATIVE_DEMO_LOG_DIR", PROJECT_ROOT / "logs"))
STATUS_FILE = Path(os.environ.get("NATIVE_DEMO_STATUS_FILE", LOG_DIR / "native_demo_status.json"))

API_HOST = os.environ.get("CARLA_API_HOST", "0.0.0.0")
API_PORT = int(os.environ.get("CARLA_API_PORT", "8765"))
CARLA_HOST = os.environ.get("CARLA_HOST", "127.0.0.1")
CARLA_PORT = int(os.environ.get("CARLA_PORT", "2000"))
CARLA_TIMEOUT = float(os.environ.get("CARLA_CONNECT_TIMEOUT", "10.0"))
CARLA_DEPLOY_TIMEOUT = float(os.environ.get("CARLA_DEPLOY_TIMEOUT", "60.0"))

TM_PORT = int(os.environ.get("CARLA_TM_PORT", "8010"))
CONTROL_PORT = int(os.environ.get("CARLA_CONTROL_PORT", "5001"))
TELEMETRY_PORTS = tuple(
    int(x.strip())
    for x in os.environ.get("CARLA_TELEMETRY_PORTS", "5000,5002,5003").split(",")
    if x.strip()
)
LOCAL_TELEMETRY_ADDR = ("127.0.0.1", int(os.environ.get("CARLA_LOCAL_TELEMETRY_PORT", "5000")))

MANUAL_CONTROL_TIMEOUT_SEC = float(os.environ.get("MANUAL_CONTROL_TIMEOUT_SEC", "0.7"))
MANUAL_BRIDGE_SCRIPT = os.environ.get("MANUAL_BRIDGE_SCRIPT", "can_tcp_bridge_vcu.py")
MANUAL_BRIDGE_PYTHON = os.environ.get("MANUAL_BRIDGE_PYTHON", sys.executable)
MANUAL_BRIDGE_STATUS_FILE = Path(os.environ.get("MANUAL_BRIDGE_STATUS_FILE", LOG_DIR / "manual_bridge_status.json"))

RUNTIME_DT_SEC = float(os.environ.get("NATIVE_DEMO_DT_SEC", "0.02"))
SPECTATOR_HZ = float(os.environ.get("NATIVE_DEMO_SPECTATOR_HZ", "60"))
STATUS_HZ = float(os.environ.get("NATIVE_DEMO_STATUS_HZ", "2"))
AI_FALLBACK_AFTER_SEC = float(os.environ.get("AI_FALLBACK_AFTER_SEC", "3.0"))
AI_FALLBACK_THROTTLE = float(os.environ.get("AI_FALLBACK_THROTTLE", "0.45"))
TRAFFIC_COUNT = int(os.environ.get("NATIVE_DEMO_TRAFFIC_COUNT", "18"))

VEHICLE_DIR = ROOT / "output"


SCENE_MAP = {
    "Town01/Urban City District": "Town01",
    "Town01": "Town01",
    "Town02/Low-Density Suburban Area": "Town02",
    "Town02": "Town02",
    "Town03/High-Density Residential Zone": "Town03",
    "Town03": "Town03",
    "Town04/High-Speed Expressway": "Town04",
    "Town04": "Town04",
    "Town05/Performance Proving Ground": "Town05",
    "Town05": "Town05",
    "TrainingGround": "TrainingGround",
    "Town04Forest": "TrainingGround",
    "Town04Forest/Town04 Forest Road": "TrainingGround",
    "训练场": "TrainingGround",
}

WEATHER_MAP = {
    "Sunny": "晴天",
    "Clear": "晴天",
    "Cloudy": "多云",
    "Light Rain": "小雨",
    "Heavy Rainstorm": "暴雨",
    "Fog/Dense Fog": "大雾",
}

TIME_MAP = {
    "Noon": "正午",
    "Sunset": "夕阳",
    "Late Night": "深夜",
}

MODE_LABELS = {
    "Manual": "硬件在环手动模式",
    "AI": "CARLA 原生 AI 巡航模式",
    "AIGO": "自动驾驶域控算法模式",
}

VEHICLE_MAP = {
    "None": None,
    "No Vehicle": None,
    "Environment Only": None,
    "Dodge Charger": "sedan_dodge_charger.json",
    "Lincoln MKZ": "sedan_lincoln_mkz.json",
    "Tesla Model 3": "sedan_tesla_model3.json",
    "Audi e-tron": "suv_audi_etron.json",
    "Jeep Wrangler": "suv_jeep_wrangler.json",
    "Tesla Cybertruck": "suv_tesla_cyber.json",
    "Fuso Rosa": "bus_fuso_rosa.json",
    "Mercedes Sprinter": "van_mercedes_sprinter.json",
    "Volkswagen T2": "van_volkswagen_t2.json",
    "Carlacola Truck": "truck_carlacola.json",
    "European HGV": "truck_european_hgv.json",
    "Firetruck": "truck_firetruck.json",
}

VEHICLE_GEOMETRY_SPECS = {
    "Dodge Charger": {"Overall": "5.10x1.90x1.46", "Wheelbase": "3.05", "Tirebase": "1.62", "Tireradius": "0.364"},
    "Lincoln MKZ": {"Overall": "4.93x1.86x1.48", "Wheelbase": "2.85", "Tirebase": "1.58", "Tireradius": "0.334"},
    "Tesla Model 3": {"Overall": "4.69x1.85x1.44", "Wheelbase": "2.88", "Tirebase": "1.58", "Tireradius": "0.334"},
    "Audi e-tron": {"Overall": "4.90x1.93x1.63", "Wheelbase": "2.93", "Tirebase": "1.65", "Tireradius": "0.381"},
    "Jeep Wrangler": {"Overall": "4.78x1.87x1.86", "Wheelbase": "3.01", "Tirebase": "1.60", "Tireradius": "0.415"},
    "Tesla Cybertruck": {"Overall": "5.88x2.03x1.90", "Wheelbase": "3.81", "Tirebase": "1.75", "Tireradius": "0.439"},
    "Fuso Rosa": {"Overall": "6.99x2.01x2.73", "Wheelbase": "3.99", "Tirebase": "1.70", "Tireradius": "0.387"},
    "Mercedes Sprinter": {"Overall": "5.93x2.02x2.68", "Wheelbase": "3.66", "Tirebase": "1.73", "Tireradius": "0.356"},
    "Volkswagen T2": {"Overall": "4.50x1.72x1.94", "Wheelbase": "2.40", "Tirebase": "1.38", "Tireradius": "0.326"},
    "Carlacola Truck": {"Overall": "8.50x2.50x3.80", "Wheelbase": "5.20", "Tirebase": "2.05", "Tireradius": "0.521"},
    "European HGV": {"Overall": "6.00x2.50x3.90", "Wheelbase": "3.80", "Tirebase": "2.05", "Tireradius": "0.506"},
    "Firetruck": {"Overall": "10.50x2.55x3.60", "Wheelbase": "5.80", "Tirebase": "2.10", "Tireradius": "0.537"},
}

SCENARIO_DATABASE = {
    "Town01": {"pos": (-2.0, 8.0, 2.0, 90.0), "script": "vshuangyi.py", "task": "DLC 双移线紧急避险"},
    "Town02": {"pos": (3.0, 109.5, 2.0, 0.0), "script": "vdanyi.py", "task": "单移线避障测试"},
    "Town03": {"pos": (-42.0, 204.0, 2.0, 0.0), "script": "vjiansu.py", "task": "动态速度廓线减速"},
    "Town04": {"pos": (9.0, 237.0, 2.0, -90.0), "script": "vshexing.py", "task": "长距离蛇行绕桩"},
    "Town05": {"pos": (206.6, 110.0, 2.0, -90.0), "script": "vjiasu.py", "task": "加速性能测试"},
    "TrainingGround": {
        "pos": (9.0, 237.0, 2.0, -90.0),
        "script": "vshexing.py",
        "task": "训练场",
        "runtime_map": "Town04",
        "prefer_opt": True,
        "unload_buildings": True,
    },
}

CAMERA_VIEW_ALIASES = {
    "follow": "follow",
    "third": "follow",
    "third_person": "follow",
    "third-person": "follow",
    "driver": "driver",
    "first": "driver",
    "first_person": "driver",
    "cockpit": "driver",
    "rear": "rear",
    "back": "rear",
    "top": "top",
}


def clamp(value: Any, low: float, high: float, default: float = 0.0) -> float:
    try:
        x = float(value)
    except Exception:
        x = default
    return max(low, min(high, x))


def first_value(body: Dict[str, Any], *keys: str, default: Any = None) -> Any:
    for key in keys:
        if key in body and body.get(key) is not None:
            return body.get(key)
    return default


def normalize_camera_view(value: Any, default: str = "follow") -> str:
    raw = str(value if value is not None else default).strip()
    if not raw:
        return default
    lowered = raw.lower().replace("-", "_").strip()
    if lowered in CAMERA_VIEW_ALIASES:
        return CAMERA_VIEW_ALIASES[lowered]
    if "driver" in lowered or "first" in lowered or "cockpit" in lowered or "驾驶" in raw or "第一" in raw:
        return "driver"
    if "rear" in lowered or "back" in lowered or "后" in raw:
        return "rear"
    if "top" in lowered or "俯" in raw:
        return "top"
    return "follow"


def camera_view_label(view: str) -> str:
    return {
        "driver": "驾驶员第一视角",
        "rear": "后视角",
        "top": "俯视角",
        "follow": "第三人称跟车视角",
    }.get(normalize_camera_view(view), "第三人称跟车视角")


def camera_stream_url(view: str) -> str:
    return f"native://carla/spectator/{normalize_camera_view(view)}"


def camera_streams() -> Dict[str, str]:
    return {
        "active": "native://carla/spectator",
        "follow": camera_stream_url("follow"),
        "driver": camera_stream_url("driver"),
        "rear": camera_stream_url("rear"),
        "top": camera_stream_url("top"),
    }


def decode_mode(value: Any) -> str:
    text = str(value if value is not None else "AI").strip()
    lower = text.lower()
    if "aigo" in lower or "algo" in lower or "算法" in text or "域控" in text or "自动驾驶" in text:
        return "AIGO"
    if "manual" in lower or "hil" in lower or "手动" in text or "硬件在环" in text:
        return "Manual"
    return "AI"


def is_ai_mode(mode: str) -> bool:
    return mode == "AI"


def is_manual_mode(mode: str) -> bool:
    return mode == "Manual"


def is_algo_mode(mode: str) -> bool:
    return mode == "AIGO"


def vehicle_geometry_params(vehicle_model: str) -> Dict[str, float]:
    geometry = VEHICLE_GEOMETRY_SPECS.get(vehicle_model, VEHICLE_GEOMETRY_SPECS["Lincoln MKZ"])
    wheelbase = float(geometry.get("Wheelbase", 2.88) or 2.88)
    track = float(geometry.get("Tirebase", 1.58) or 1.58)
    tire_radius = float(geometry.get("Tireradius", 0.35) or 0.35)
    return {
        "wheelbase": wheelbase,
        "track": track,
        "tire_radius": tire_radius,
        "a": round(wheelbase * 0.517, 3),
        "b": round(wheelbase * 0.483, 3),
    }


def load_vehicle_config(vehicle_model: str) -> Tuple[Optional[Dict[str, Any]], Optional[Path], str]:
    file_name = VEHICLE_MAP.get(vehicle_model, VEHICLE_MAP["Lincoln MKZ"])
    if file_name is None:
        return None, None, vehicle_model
    path = VEHICLE_DIR / file_name
    try:
        return json.loads(path.read_text(encoding="utf-8")), path, vehicle_model
    except Exception:
        fallback = VEHICLE_DIR / VEHICLE_MAP["Lincoln MKZ"]
        return json.loads(fallback.read_text(encoding="utf-8")), fallback, "Lincoln MKZ"


def build_ui_params(vehicle_model: str, vehicle_config: Optional[Dict[str, Any]]) -> Dict[str, float]:
    geom = vehicle_geometry_params(vehicle_model)
    mass_props = (vehicle_config or {}).get("weight_and_mass_properties") or {}
    mech_props = (vehicle_config or {}).get("chassis_and_mechanical_systems") or {}
    steering_props = mech_props.get("steering_system") or {}
    mass = float(mass_props.get("curb_weight_kg", 1800.0) or 1800.0)
    return {
        "mass": mass,
        "cf": -110000.0 * (mass / 1500.0),
        "cr": -95000.0 * (mass / 1500.0),
        "wheelbase": geom["wheelbase"],
        "track": geom["track"],
        "tire_radius": geom["tire_radius"],
        "a": geom["a"],
        "b": geom["b"],
        "steer": float(steering_props.get("wheel_max_angle_deg", 40.0) or 40.0),
    }


def safe_actor_alive(actor: Any) -> bool:
    try:
        return bool(actor and actor.is_alive)
    except Exception:
        return False


def terminate_process_tree(proc: Optional[subprocess.Popen], timeout: float = 2.0) -> None:
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
            proc.wait(timeout=timeout)
        except Exception:
            proc.kill()
    except Exception:
        pass


def set_vehicle_autopilot(vehicle: Any, enabled: bool) -> None:
    try:
        vehicle.set_autopilot(enabled, TM_PORT)
    except TypeError:
        vehicle.set_autopilot(enabled)
    except Exception as exc:
        print(f"[native-demo] set_autopilot({enabled}) warning: {exc}", flush=True)


def read_json_file(path: Path) -> Optional[Dict[str, Any]]:
    try:
        if path.exists():
            return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return None
    return None


def write_status_file(payload: Dict[str, Any]) -> None:
    try:
        LOG_DIR.mkdir(exist_ok=True)
        tmp_path = STATUS_FILE.with_suffix(".tmp")
        tmp_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
        tmp_path.replace(STATUS_FILE)
    except Exception:
        pass


def find_existing_bridge_pids() -> List[int]:
    if os.name == "nt":
        return []
    try:
        out = subprocess.check_output(
            ["pgrep", "-af", str(MANUAL_BRIDGE_SCRIPT)],
            text=True,
            stderr=subprocess.DEVNULL,
            timeout=1.0,
        )
    except Exception:
        return []
    pids = []
    for line in out.splitlines():
        parts = line.strip().split(maxsplit=1)
        if not parts:
            continue
        try:
            pid = int(parts[0])
        except ValueError:
            continue
        if pid != os.getpid():
            pids.append(pid)
    return pids


def stop_external_bridge_pids() -> None:
    for pid in find_existing_bridge_pids():
        try:
            os.kill(pid, signal.SIGTERM)
        except Exception:
            continue
    if os.name != "nt":
        time.sleep(0.2)
        for pid in find_existing_bridge_pids():
            try:
                os.kill(pid, signal.SIGKILL)
            except Exception:
                pass


class DemoState:
    def __init__(self) -> None:
        self.lock = threading.RLock()
        self.client = None
        self.world = None
        self.vehicle = None
        self.traffic_actors: List[Any] = []
        self.runtime_stop = threading.Event()
        self.runtime_thread: Optional[threading.Thread] = None
        self.deploy_lock = threading.Lock()

        self.algo_process: Optional[subprocess.Popen] = None
        self.algo_log_file = None
        self.algo_log_path: Optional[Path] = None
        self.manual_bridge_process: Optional[subprocess.Popen] = None
        self.manual_bridge_log_file = None
        self.manual_bridge_log_path: Optional[Path] = None

        self.target_ip = "127.0.0.1"
        self.current_world_name: Optional[str] = None
        self.requested_scene = "Town02"
        self.runtime_map = "Town02"
        self.vehicle_model = "Lincoln MKZ"
        self.vehicle_config: Optional[Dict[str, Any]] = None
        self.ui_params = build_ui_params(self.vehicle_model, None)
        self.drive_mode = "AI"
        self.camera_view = normalize_camera_view(os.environ.get("CAMERA_VIEW_DEFAULT", "follow"))
        self.camera_view_revision = 0

        self.carla_connected = False
        self.carla_status_message = "not connected"
        self.deployment_active = False
        self.deployment_step = "idle"
        self.last_command: Dict[str, Any] = {}
        self.decoded_command: Dict[str, Any] = {}
        self.last_result: Dict[str, Any] = {"ok": False, "msg": "idle"}
        self.last_error: Optional[str] = None

        self.last_control: Optional[Dict[str, Any]] = None
        self.last_control_time = 0.0
        self.control_rx_count = 0
        self.telemetry_tx_count = 0
        self.last_telemetry: Dict[str, Any] = {}
        self.last_speed_kmh = 0.0

    def set_step(self, step: str, msg: str = "") -> None:
        with self.lock:
            self.deployment_step = step
            if msg:
                self.carla_status_message = msg
            print(f"[native-demo] {step}: {msg}", flush=True)

    def set_camera_view(self, view: Any) -> str:
        normalized = normalize_camera_view(view, self.camera_view)
        with self.lock:
            if normalized != self.camera_view:
                self.camera_view_revision += 1
            self.camera_view = normalized
        return normalized

    def snapshot(self) -> Dict[str, Any]:
        with self.lock:
            actor_id = None
            if safe_actor_alive(self.vehicle):
                try:
                    actor_id = self.vehicle.id
                except Exception:
                    actor_id = None
            manual_bridge_status = read_json_file(MANUAL_BRIDGE_STATUS_FILE)
            runtime_running = bool(self.runtime_thread and self.runtime_thread.is_alive())
            return {
                "ok": True,
                "backend": "native_demo_controller",
                "carla_connected": self.carla_connected,
                "carla_status": self.carla_status_message,
                "deployment_active": self.deployment_active,
                "deployment_step": self.deployment_step,
                "world": self.current_world_name,
                "requested_scene": self.requested_scene,
                "runtime_map": self.runtime_map,
                "vehicle_alive": safe_actor_alive(self.vehicle),
                "drive_mode": MODE_LABELS.get(self.drive_mode, self.drive_mode),
                "drive_mode_key": self.drive_mode,
                "camera_view": self.camera_view,
                "camera_view_label": camera_view_label(self.camera_view),
                "camera_stream_url": camera_stream_url(self.camera_view),
                "camera_streams": camera_streams(),
                "camera_stream_reload_key": f"{self.camera_view}:{self.camera_view_revision}",
                "active_view_url": camera_stream_url(self.camera_view),
                "last_command": self.last_command,
                "decoded_command": self.decoded_command,
                "last_result": self.last_result,
                "last_error": self.last_error,
                "target_ip": self.target_ip,
                "runtime_running": runtime_running,
                "telemetry_tx_count": self.telemetry_tx_count,
                "manual_bridge": {
                    "running": bool(self.manual_bridge_process and self.manual_bridge_process.poll() is None),
                    "pid": self.manual_bridge_process.pid if self.manual_bridge_process and self.manual_bridge_process.poll() is None else None,
                    "external_pids": find_existing_bridge_pids(),
                    "log_path": str(self.manual_bridge_log_path) if self.manual_bridge_log_path else None,
                    "last_control_age_sec": round(time.time() - self.last_control_time, 3) if self.last_control_time else None,
                    "last_control": self.last_control,
                    "bridge_file_status": manual_bridge_status,
                },
                "diagnostics": self.diagnostics(actor_id),
            }

    def diagnostics(self, actor_id: Optional[int] = None) -> Dict[str, Any]:
        vehicle = self.vehicle
        if not safe_actor_alive(vehicle):
            return {
                "actor_id": actor_id,
                "speed_kmh": 0.0,
                "control": None,
                "autopilot_expected": is_ai_mode(self.drive_mode),
                "last_control": self.last_control,
            }
        try:
            v = vehicle.get_velocity()
            speed_kmh = round(math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z) * 3.6, 3)
            ctrl = vehicle.get_control()
            actor_id = vehicle.id if actor_id is None else actor_id
            return {
                "actor_id": actor_id,
                "speed_kmh": speed_kmh,
                "control": {
                    "throttle": round(float(ctrl.throttle), 3),
                    "steer": round(float(ctrl.steer), 4),
                    "brake": round(float(ctrl.brake), 3),
                    "reverse": bool(ctrl.reverse),
                    "hand_brake": bool(ctrl.hand_brake),
                    "manual_gear_shift": bool(ctrl.manual_gear_shift),
                    "gear": int(ctrl.gear),
                },
                "autopilot_expected": is_ai_mode(self.drive_mode),
                "last_control": self.last_control,
            }
        except Exception as exc:
            return {
                "actor_id": actor_id,
                "speed_kmh": self.last_speed_kmh,
                "control": None,
                "autopilot_expected": is_ai_mode(self.drive_mode),
                "last_control": self.last_control,
                "error": str(exc),
            }


state = DemoState()


def ensure_carla_connection(timeout: float = CARLA_TIMEOUT) -> Tuple[bool, str]:
    if carla is None:
        return False, f"CARLA Python API import failed: {CARLA_IMPORT_ERROR}"
    try:
        if state.client is not None and state.world is not None:
            try:
                state.client.get_world()
                state.carla_connected = True
                return True, "CARLA already connected"
            except Exception:
                state.client = None
                state.world = None

        client = carla.Client(CARLA_HOST, CARLA_PORT)
        client.set_timeout(timeout)
        world = client.get_world()
        apply_async_world_settings(world)
        state.client = client
        state.world = world
        state.carla_connected = True
        return True, f"connected to CARLA {CARLA_HOST}:{CARLA_PORT}"
    except Exception as exc:
        state.carla_connected = False
        state.carla_status_message = f"CARLA connect failed: {exc}"
        return False, state.carla_status_message


def apply_async_world_settings(world: Any) -> None:
    try:
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = float(os.environ.get("OFFICIAL_FIXED_DELTA_SECONDS", "0.02"))
        if hasattr(settings, "substepping"):
            settings.substepping = True
        if hasattr(settings, "max_substep_delta_time"):
            settings.max_substep_delta_time = float(os.environ.get("OFFICIAL_MAX_SUBSTEP_DELTA_TIME", "0.005"))
        if hasattr(settings, "max_substeps"):
            settings.max_substeps = int(os.environ.get("OFFICIAL_MAX_SUBSTEPS", "16"))
        world.apply_settings(settings)
    except Exception as exc:
        print(f"[native-demo] world settings warning: {exc}", flush=True)


def resolve_runtime_map(client: Any, scene_name: str) -> str:
    scene_cfg = SCENARIO_DATABASE.get(scene_name, {})
    base_map = scene_cfg.get("runtime_map", scene_name)
    if not scene_cfg.get("prefer_opt"):
        return base_map
    opt_map = f"{base_map}_Opt"
    try:
        available = [m.split("/")[-1] for m in client.get_available_maps()]
        if opt_map in available:
            return opt_map
    except Exception:
        pass
    return base_map


def activate_scene_variant(world: Any, scene_name: str) -> None:
    scene_cfg = SCENARIO_DATABASE.get(scene_name, {})
    if not scene_cfg.get("unload_buildings"):
        return
    try:
        world.unload_map_layer(carla.MapLayer.Buildings)
    except Exception as exc:
        print(f"[native-demo] map layer warning: {exc}", flush=True)


def apply_custom_weather(world: Any, weather_text: str, time_text: str) -> None:
    w = carla.WeatherParameters()
    w.cloudiness = 0.0
    w.precipitation = 0.0
    w.precipitation_deposits = 0.0
    w.wind_intensity = 0.0
    w.fog_density = 0.0
    w.fog_distance = 0.0

    if "多云" in weather_text:
        w.cloudiness = 80.0
    elif "小雨" in weather_text:
        w.cloudiness = 80.0
        w.precipitation = 30.0
        w.precipitation_deposits = 30.0
    elif "暴雨" in weather_text:
        w.cloudiness = 100.0
        w.precipitation = 90.0
        w.precipitation_deposits = 90.0
        w.wind_intensity = 80.0
    elif "大雾" in weather_text:
        w.cloudiness = 50.0
        w.fog_density = 50.0
        w.fog_distance = 10.0

    if "正午" in time_text:
        w.sun_altitude_angle = 75.0
        w.sun_azimuth_angle = 180.0
    elif "夕阳" in time_text:
        w.sun_altitude_angle = 5.0
        w.sun_azimuth_angle = 180.0
    elif "深夜" in time_text:
        w.sun_altitude_angle = -90.0
        w.sun_azimuth_angle = 0.0

    world.set_weather(w)


def build_spawn_transform(scene_name: str) -> Any:
    cfg = SCENARIO_DATABASE.get(scene_name)
    if not cfg:
        cfg = SCENARIO_DATABASE["Town02"]
    x, y, z, yaw = cfg["pos"]
    return carla.Transform(carla.Location(x=x, y=y, z=z), carla.Rotation(pitch=0.0, yaw=yaw, roll=0.0))


def cleanup_existing_ego_actors(world: Any) -> None:
    try:
        actors = world.get_actors().filter("vehicle.*")
    except Exception:
        return
    for actor in actors:
        try:
            role = actor.attributes.get("role_name", "")
            if role in {"hero", "ego", "manual", "native_demo_ego"}:
                actor.destroy()
        except Exception:
            pass


def cleanup_runtime(stop_bridge: bool = True) -> None:
    state.runtime_stop.set()
    if state.runtime_thread and state.runtime_thread.is_alive():
        try:
            state.runtime_thread.join(timeout=2.0)
        except Exception:
            pass
    state.runtime_thread = None
    state.runtime_stop = threading.Event()

    terminate_process_tree(state.algo_process)
    state.algo_process = None
    if state.algo_log_file:
        try:
            state.algo_log_file.close()
        except Exception:
            pass
    state.algo_log_file = None
    state.algo_log_path = None

    if stop_bridge:
        terminate_process_tree(state.manual_bridge_process)
        state.manual_bridge_process = None
        if state.manual_bridge_log_file:
            try:
                state.manual_bridge_log_file.close()
            except Exception:
                pass
        state.manual_bridge_log_file = None
        state.manual_bridge_log_path = None

    for actor in list(state.traffic_actors):
        try:
            if safe_actor_alive(actor):
                actor.destroy()
        except Exception:
            pass
    state.traffic_actors = []

    try:
        if safe_actor_alive(state.vehicle):
            set_vehicle_autopilot(state.vehicle, False)
            state.vehicle.destroy()
    except Exception:
        pass
    state.vehicle = None
    state.last_control = None
    state.last_control_time = 0.0
    state.control_rx_count = 0


def find_vehicle_blueprint(bp_lib: Any, vehicle_config: Optional[Dict[str, Any]], vehicle_model: str) -> Any:
    bp_id = ((vehicle_config or {}).get("vehicle_metadata") or {}).get("blueprint_id")
    if bp_id:
        try:
            return bp_lib.find(bp_id)
        except Exception:
            pass
    filters = {
        "Lincoln MKZ": ["vehicle.lincoln.mkz_2017", "*lincoln*", "*mkz*"],
        "Tesla Model 3": ["vehicle.tesla.model3", "*model3*", "*tesla*"],
        "Dodge Charger": ["*charger*"],
        "Audi e-tron": ["*etron*", "*audi*"],
        "Jeep Wrangler": ["*jeep*", "*wrangler*"],
        "Tesla Cybertruck": ["*cybertruck*"],
        "Fuso Rosa": ["*fuso*", "*rosa*"],
        "Mercedes Sprinter": ["*sprinter*"],
        "Volkswagen T2": ["*volkswagen*", "*t2*"],
        "Carlacola Truck": ["*carlacola*"],
        "European HGV": ["*european_hgv*", "*hgv*"],
        "Firetruck": ["*firetruck*"],
    }
    for pattern in filters.get(vehicle_model, filters["Lincoln MKZ"]):
        try:
            matches = bp_lib.filter(pattern)
            if matches:
                return matches[0]
        except Exception:
            continue
    matches = bp_lib.filter("vehicle.*")
    if not matches:
        raise RuntimeError("no vehicle blueprint available")
    return matches[0]


def spawn_vehicle(world: Any, vehicle_config: Optional[Dict[str, Any]], vehicle_model: str, scene_name: str) -> Any:
    bp_lib = world.get_blueprint_library()
    bp = find_vehicle_blueprint(bp_lib, vehicle_config, vehicle_model)
    if bp.has_attribute("role_name"):
        bp.set_attribute("role_name", "hero")
    if bp.has_attribute("color"):
        recommended = bp.get_attribute("color").recommended_values
        if recommended:
            bp.set_attribute("color", recommended[0])

    spawn_tf = build_spawn_transform(scene_name)
    vehicle = None
    for dz in (0.0, 0.3, 0.8, 1.2):
        tf = carla.Transform(
            carla.Location(
                x=spawn_tf.location.x,
                y=spawn_tf.location.y,
                z=spawn_tf.location.z + dz,
            ),
            spawn_tf.rotation,
        )
        try:
            vehicle = world.try_spawn_actor(bp, tf)
        except Exception:
            vehicle = None
        if vehicle is not None:
            break
        try:
            world.wait_for_tick()
        except Exception:
            time.sleep(0.1)
    if vehicle is None:
        raise RuntimeError(f"failed to spawn ego vehicle from blueprint {bp.id}")

    try:
        vehicle.set_simulate_physics(True)
        vehicle.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0, brake=0.0, hand_brake=False, reverse=False, manual_gear_shift=False, gear=1))
    except Exception as exc:
        print(f"[native-demo] vehicle init warning: {exc}", flush=True)
    return vehicle


def spawn_traffic(world: Any, client: Any, count: int, ego_location: Any) -> List[Any]:
    if count <= 0:
        return []
    try:
        tm = client.get_trafficmanager(TM_PORT)
        tm.set_synchronous_mode(False)
        tm.global_percentage_speed_difference(10.0)
    except Exception:
        tm = None

    bp_lib = world.get_blueprint_library()
    blueprints = list(bp_lib.filter("vehicle.*"))
    spawn_points = list(world.get_map().get_spawn_points())
    random.shuffle(spawn_points)
    actors = []
    for sp in spawn_points:
        if len(actors) >= count:
            break
        try:
            if sp.location.distance(ego_location) < 25.0:
                continue
        except Exception:
            pass
        bp = random.choice(blueprints)
        if bp.has_attribute("role_name"):
            bp.set_attribute("role_name", "normal_vehicle")
        if bp.has_attribute("color"):
            values = bp.get_attribute("color").recommended_values
            if values:
                bp.set_attribute("color", random.choice(values))
        try:
            actor = world.try_spawn_actor(bp, sp)
            if actor:
                actors.append(actor)
                set_vehicle_autopilot(actor, True)
        except Exception:
            continue
    return actors


def start_manual_bridge() -> None:
    if find_existing_bridge_pids():
        return
    script_path = Path(MANUAL_BRIDGE_SCRIPT)
    if not script_path.is_absolute():
        script_path = ROOT / script_path
    if not script_path.exists():
        state.last_error = f"manual bridge not found: {script_path}"
        print(f"[native-demo] {state.last_error}", flush=True)
        return
    try:
        LOG_DIR.mkdir(exist_ok=True)
        log_path = LOG_DIR / f"native_manual_bridge_{int(time.time())}.log"
        log_file = open(log_path, "a", encoding="utf-8")
        kwargs: Dict[str, Any] = {
            "cwd": str(ROOT),
            "stdout": log_file,
            "stderr": subprocess.STDOUT,
        }
        if os.name != "nt":
            kwargs["preexec_fn"] = os.setsid
        proc = subprocess.Popen([MANUAL_BRIDGE_PYTHON, str(script_path)], **kwargs)
        state.manual_bridge_process = proc
        state.manual_bridge_log_file = log_file
        state.manual_bridge_log_path = log_path
        print(f"[native-demo] manual bridge started: pid={proc.pid}, log={log_path}", flush=True)
    except Exception as exc:
        state.last_error = f"manual bridge start failed: {exc}"
        print(f"[native-demo] {state.last_error}", flush=True)


def start_algorithm(scene_name: str) -> None:
    cfg = SCENARIO_DATABASE.get(scene_name, SCENARIO_DATABASE["Town02"])
    script_name = cfg.get("script")
    if not script_name:
        return
    script_path = ROOT / script_name
    if not script_path.exists():
        state.last_error = f"algorithm script not found: {script_path}"
        print(f"[native-demo] {state.last_error}", flush=True)
        return
    try:
        LOG_DIR.mkdir(exist_ok=True)
        log_path = LOG_DIR / f"native_algo_{script_path.stem}_{int(time.time())}.log"
        log_file = open(log_path, "a", encoding="utf-8")
        kwargs: Dict[str, Any] = {
            "cwd": str(ROOT),
            "stdout": log_file,
            "stderr": subprocess.STDOUT,
        }
        if os.name != "nt":
            kwargs["preexec_fn"] = os.setsid
        proc = subprocess.Popen([sys.executable, str(script_path)], **kwargs)
        state.algo_process = proc
        state.algo_log_file = log_file
        state.algo_log_path = log_path
        print(f"[native-demo] algorithm started: {script_name}, pid={proc.pid}, log={log_path}", flush=True)
    except Exception as exc:
        state.last_error = f"algorithm start failed: {exc}"
        print(f"[native-demo] {state.last_error}", flush=True)


def setup_ai(vehicle: Any, client: Any) -> None:
    try:
        tm = client.get_trafficmanager(TM_PORT)
        tm.set_synchronous_mode(False)
        tm.ignore_lights_percentage(vehicle, 0.0)
        tm.ignore_signs_percentage(vehicle, 0.0)
        tm.vehicle_percentage_speed_difference(vehicle, -20.0)
    except Exception as exc:
        print(f"[native-demo] Traffic Manager setup warning: {exc}", flush=True)
    set_vehicle_autopilot(vehicle, True)


def control_from_json(data: Dict[str, Any]) -> Dict[str, Any]:
    reverse = bool(data.get("reverse", False))
    return {
        "throttle": clamp(data.get("throttle", 0.0), 0.0, 1.0),
        "steer": clamp(data.get("steer", 0.0), -1.0, 1.0),
        "brake": clamp(data.get("brake", 0.0), 0.0, 1.0),
        "reverse": reverse,
        "hand_brake": bool(data.get("hand_brake", False)),
    }


def apply_vehicle_control(vehicle: Any, command: Dict[str, Any]) -> None:
    reverse = bool(command.get("reverse", False))
    gear = -1 if reverse else 1
    vehicle.apply_control(
        carla.VehicleControl(
            throttle=float(command.get("throttle", 0.0)),
            steer=float(command.get("steer", 0.0)),
            brake=float(command.get("brake", 0.0)),
            reverse=reverse,
            hand_brake=bool(command.get("hand_brake", False)),
            manual_gear_shift=False,
            gear=gear,
        )
    )


def build_telemetry(vehicle: Any, ui: Dict[str, float]) -> Dict[str, Any]:
    t = vehicle.get_transform()
    v = vehicle.get_velocity()
    a = vehicle.get_acceleration()
    w = vehicle.get_angular_velocity()
    ctrl = vehicle.get_control()
    speed_ms = math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
    state.last_speed_kmh = speed_ms * 3.6

    try:
        steer_fl = vehicle.get_wheel_steer_angle(carla.VehicleWheelLocation.FL_Wheel)
        steer_fr = vehicle.get_wheel_steer_angle(carla.VehicleWheelLocation.FR_Wheel)
    except Exception:
        steer_fl = steer_fr = float(ctrl.steer) * float(ui.get("steer", 40.0))

    wheel_radius = max(0.05, float(ui.get("tire_radius", 0.35)))
    base_rpm = (speed_ms / wheel_radius) * (60.0 / (2 * math.pi))
    wheel_rpm = [base_rpm, base_rpm, base_rpm, base_rpm]
    cf = float(ui.get("cf", -110000.0))
    cr = float(ui.get("cr", -95000.0))

    return {
        "1_刚体运动学 (Rigid Body Kinematics)": {
            "1_全局绝对坐标_XYZ_米": [round(t.location.x, 3), round(t.location.y, 3), round(t.location.z, 3)],
            "2_姿态角_俯仰_偏航_滚转_度": [round(t.rotation.pitch, 3), round(t.rotation.yaw, 3), round(t.rotation.roll, 3)],
            "3_线速度矢量_XYZ_米每秒": [round(v.x, 3), round(v.y, 3), round(v.z, 3)],
            "4_线加速度_XYZ_米每平方秒": [round(a.x, 3), round(a.y, 3), round(a.z, 3)],
            "5_角速度_XYZ_度每秒": [round(w.x, 3), round(w.y, 3), round(w.z, 3)],
        },
        "2_轮端与底盘动态 (Wheel Dynamics)": {
            "6_四轮独立转速_RPM_左前_右前_左后_右后": [round(x, 1) for x in wheel_rpm],
            "7_悬架实时压缩量_毫米_左前_右前_左后_右后": [0.0, 0.0, 0.0, 0.0],
            "8_前轮真实阿克曼转向角_度": [round(steer_fl, 2), round(steer_fr, 2)],
        },
        "3_驾驶控制反读 (Control State)": {
            "9_实际油门开度_0至1": round(float(ctrl.throttle), 3),
            "10_实际刹车力度_0至1": round(float(ctrl.brake), 3),
            "11_方向盘转角_负1至1": round(float(ctrl.steer), 3),
            "12_当前机械档位": int(ctrl.gear),
            "13_手刹激活状态": bool(ctrl.hand_brake),
            "14_倒车挂档状态": bool(ctrl.reverse),
        },
        "5_环境与交通真值 (Environment Truth)": {
            "19_当前路段法定限速_公里每小时": round(vehicle.get_speed_limit(), 1),
            "20_前方红绿灯当前状态": str(vehicle.get_traffic_light_state()),
            "21_是否处于红绿灯管制区": bool(vehicle.is_at_traffic_light()),
            "22_车辆灯光激活状态_位掩码": str(vehicle.get_light_state()),
        },
        "6_动态车辆参数": {
            "整备质量": float(ui.get("mass", 1800.0)),
            "前轮侧偏刚度_Cf": cf,
            "后轮侧偏刚度_Cr": cr,
            "轮距_L": float(ui.get("wheelbase", 2.88)),
            "轴距_L": float(ui.get("wheelbase", 2.88)),
            "a": float(ui.get("a", 1.49)),
            "b": float(ui.get("b", 1.39)),
            "Cf": cf,
            "Cr": cr,
            "最大前轮转角_deg": float(ui.get("steer", 40.0)),
        },
    }


def local_to_world(transform: Any, dx: float, dy: float, dz: float) -> Any:
    pitch = math.radians(transform.rotation.pitch)
    yaw = math.radians(transform.rotation.yaw)
    roll = math.radians(transform.rotation.roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    cr, sr = math.cos(roll), math.sin(roll)

    fwd = (cy * cp, sy * cp, sp)
    right = (-cy * sp * sr - sy * cr, -sy * sp * sr + cy * cr, cp * sr)
    up = (-cy * sp * cr + sy * sr, -sy * sp * cr - cy * sr, cp * cr)

    return carla.Location(
        x=transform.location.x + fwd[0] * dx + right[0] * dy + up[0] * dz,
        y=transform.location.y + fwd[1] * dx + right[1] * dy + up[1] * dz,
        z=transform.location.z + fwd[2] * dx + right[2] * dy + up[2] * dz,
    )


def update_spectator(world: Any, vehicle: Any, view: str) -> None:
    if not safe_actor_alive(vehicle):
        return
    spectator = world.get_spectator()
    tf = vehicle.get_transform()
    view = normalize_camera_view(view)

    if view == "driver":
        loc = local_to_world(tf, 0.55, -0.35, 1.25)
        rot = carla.Rotation(pitch=tf.rotation.pitch - 2.0, yaw=tf.rotation.yaw, roll=0.0)
    elif view == "rear":
        loc = local_to_world(tf, -1.6, 0.0, 1.45)
        rot = carla.Rotation(pitch=-4.0, yaw=tf.rotation.yaw + 180.0, roll=0.0)
    elif view == "top":
        loc = local_to_world(tf, -2.0, 0.0, 18.0)
        rot = carla.Rotation(pitch=-80.0, yaw=tf.rotation.yaw, roll=0.0)
    else:
        loc = local_to_world(tf, -7.0, 0.0, 3.0)
        rot = carla.Rotation(pitch=-15.0, yaw=tf.rotation.yaw, roll=0.0)

    spectator.set_transform(carla.Transform(loc, rot))


def make_control_socket() -> socket.socket:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    if hasattr(socket, "SO_REUSEPORT"):
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except Exception:
            pass
    sock.bind(("0.0.0.0", CONTROL_PORT))
    sock.setblocking(False)
    return sock


def runtime_loop(vehicle: Any, world: Any, client: Any, stop_event: threading.Event) -> None:
    telemetry_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    control_sock: Optional[socket.socket] = None
    try:
        control_sock = make_control_socket()
        print(f"[native-demo] UDP control listening on 0.0.0.0:{CONTROL_PORT}", flush=True)
    except Exception as exc:
        state.last_error = f"UDP {CONTROL_PORT} bind failed: {exc}"
        print(f"[native-demo] {state.last_error}", flush=True)

    last_mode = None
    mode_started_at = time.time()
    ai_fallback = False
    last_spectator = 0.0
    last_status = 0.0
    spectator_interval = 1.0 / max(1.0, SPECTATOR_HZ)
    status_interval = 1.0 / max(0.2, STATUS_HZ)

    try:
        while not stop_event.is_set():
            started = time.time()
            if not safe_actor_alive(vehicle):
                state.last_error = "ego vehicle is not alive"
                break

            mode = state.drive_mode
            if mode != last_mode:
                mode_started_at = time.time()
                ai_fallback = False
                try:
                    vehicle.disable_constant_velocity()
                except Exception:
                    pass
                if is_ai_mode(mode):
                    setup_ai(vehicle, client)
                else:
                    set_vehicle_autopilot(vehicle, False)
                last_mode = mode

            if is_ai_mode(mode):
                try:
                    v = vehicle.get_velocity()
                    speed = math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z) * 3.6
                except Exception:
                    speed = 0.0
                if not ai_fallback and time.time() - mode_started_at >= AI_FALLBACK_AFTER_SEC and speed < 1.0:
                    ai_fallback = True
                    try:
                        set_vehicle_autopilot(vehicle, False)
                    except Exception:
                        pass
                    print("[native-demo] AI fallback throttle enabled because TM did not move ego", flush=True)
                if ai_fallback:
                    apply_vehicle_control(vehicle, {"throttle": AI_FALLBACK_THROTTLE, "steer": 0.0, "brake": 0.0, "reverse": False, "hand_brake": False})

            latest_control = None
            if control_sock is not None and not is_ai_mode(mode):
                try:
                    packet, _ = control_sock.recvfrom(4096)
                    while True:
                        latest_control = packet
                        try:
                            packet, _ = control_sock.recvfrom(4096)
                        except BlockingIOError:
                            break
                except BlockingIOError:
                    pass
                except Exception as exc:
                    state.last_error = f"UDP control receive failed: {exc}"

            if latest_control is not None and not is_ai_mode(mode):
                try:
                    data = json.loads(latest_control.decode("utf-8"))
                    command = control_from_json(data)
                    apply_vehicle_control(vehicle, command)
                    with state.lock:
                        state.last_control = {
                            "throttle": round(command["throttle"], 3),
                            "steer": round(command["steer"], 4),
                            "brake": round(command["brake"], 3),
                            "reverse": bool(command["reverse"]),
                            "hand_brake": bool(command["hand_brake"]),
                        }
                        state.last_control_time = time.time()
                        state.control_rx_count += 1
                except Exception as exc:
                    state.last_error = f"UDP control parse failed: {exc}"

            if is_manual_mode(mode):
                stale = time.time() - state.last_control_time if state.last_control_time else None
                if stale is None or stale > MANUAL_CONTROL_TIMEOUT_SEC:
                    try:
                        apply_vehicle_control(vehicle, {"throttle": 0.0, "steer": 0.0, "brake": 1.0, "reverse": False, "hand_brake": False})
                    except Exception:
                        pass

            try:
                telemetry = build_telemetry(vehicle, state.ui_params)
                payload = json.dumps(telemetry, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
                targets = {(state.target_ip, port) for port in TELEMETRY_PORTS}
                targets.add(LOCAL_TELEMETRY_ADDR)
                for addr in targets:
                    try:
                        telemetry_sock.sendto(payload, addr)
                    except Exception:
                        pass
                with state.lock:
                    state.last_telemetry = telemetry
                    state.telemetry_tx_count += 1
            except Exception as exc:
                state.last_error = f"telemetry build/send failed: {exc}"

            now = time.time()
            if now - last_spectator >= spectator_interval:
                try:
                    update_spectator(world, vehicle, state.camera_view)
                except Exception as exc:
                    state.last_error = f"spectator update failed: {exc}"
                last_spectator = now

            if now - last_status >= status_interval:
                write_status_file(state.snapshot())
                last_status = now

            elapsed = time.time() - started
            sleep_for = RUNTIME_DT_SEC - elapsed
            if sleep_for > 0:
                time.sleep(sleep_for)
    finally:
        try:
            telemetry_sock.close()
        except Exception:
            pass
        if control_sock is not None:
            try:
                control_sock.close()
            except Exception:
                pass
        write_status_file(state.snapshot())
        print("[native-demo] runtime loop stopped", flush=True)


def should_load_traffic(value: Any) -> bool:
    text = str(value).strip().lower()
    return text in {"0", "true", "yes", "y", "on", "有", "加载", "traffic"}


def decode_command(body: Dict[str, Any], client: Any = None) -> Dict[str, Any]:
    scene_key = first_value(body, "scene", "Scene", default="Town02")
    scene_name = SCENE_MAP.get(str(scene_key), str(scene_key))
    if scene_name not in SCENARIO_DATABASE:
        scene_name = "Town02"

    weather_key = first_value(body, "sky", "Weather Condition", default="Sunny")
    weather_text = WEATHER_MAP.get(str(weather_key), str(weather_key) if str(weather_key) in WEATHER_MAP.values() else "晴天")
    time_key = first_value(body, "sunshinetime", "Sunshine Time", default="Noon")
    time_text = TIME_MAP.get(str(time_key), str(time_key) if str(time_key) in TIME_MAP.values() else "正午")
    drive_mode = decode_mode(first_value(body, "drive_mode", "Drive Mode", default="AI"))
    vehicle_model = str(first_value(body, "vehiclemodel", "Vehicle Model", default="Lincoln MKZ")).strip() or "Lincoln MKZ"
    if vehicle_model not in VEHICLE_MAP:
        vehicle_model = "Lincoln MKZ"
    traffic_key = first_value(body, "loadingtransportation", "Traffic Load", default="1")
    camera_view = normalize_camera_view(first_value(body, "camera_view", "Camera View", "view", "View", "视角", "camera", default=state.camera_view))
    runtime_map = resolve_runtime_map(client, scene_name) if client else scene_name

    return {
        "scene": scene_name,
        "runtime_map": runtime_map,
        "sky": weather_text,
        "sunshinetime": time_text,
        "drive_mode": drive_mode,
        "drive_mode_label": MODE_LABELS.get(drive_mode, drive_mode),
        "loadingtransportation": "0" if should_load_traffic(traffic_key) else "1",
        "load_traffic": should_load_traffic(traffic_key),
        "vehiclemodel": vehicle_model,
        "camera_view": camera_view,
        "camera_view_label": camera_view_label(camera_view),
        "task": SCENARIO_DATABASE.get(scene_name, {}).get("task"),
    }


def deploy_command(body: Dict[str, Any]) -> None:
    if not state.deploy_lock.acquire(blocking=False):
        return
    try:
        with state.lock:
            state.deployment_active = True
            state.last_error = None
            state.last_result = {"ok": True, "msg": "deployment running"}
        state.set_step("connect_carla", f"connecting to {CARLA_HOST}:{CARLA_PORT}")
        ok, msg = ensure_carla_connection(CARLA_TIMEOUT)
        if not ok:
            with state.lock:
                state.last_result = {"ok": False, "msg": msg}
                state.last_error = msg
            return

        client = state.client
        client.set_timeout(CARLA_DEPLOY_TIMEOUT)
        decoded = decode_command(body, client=client)
        target_ip = body.get("_client_ip") or "127.0.0.1"

        with state.lock:
            state.last_command = dict(body)
            state.decoded_command = dict(decoded)
            state.target_ip = target_ip
            state.requested_scene = decoded["scene"]
            state.runtime_map = decoded["runtime_map"]
            state.vehicle_model = decoded["vehiclemodel"]
            state.drive_mode = decoded["drive_mode"]
            state.set_camera_view(decoded["camera_view"])

        state.set_step("cleanup_runtime", "stopping previous actors and child processes")
        cleanup_runtime(stop_bridge=not is_manual_mode(decoded["drive_mode"]))
        if not is_manual_mode(decoded["drive_mode"]):
            stop_external_bridge_pids()

        state.set_step("load_world", f"loading {decoded['runtime_map']}")
        current = ""
        try:
            current = client.get_world().get_map().name.split("/")[-1]
        except Exception:
            current = ""
        if current == decoded["runtime_map"]:
            world = client.get_world()
        else:
            world = client.load_world(decoded["runtime_map"])
        apply_async_world_settings(world)
        state.world = world
        state.current_world_name = decoded["scene"] if decoded["scene"] == decoded["runtime_map"] else f"{decoded['scene']} ({decoded['runtime_map']})"
        activate_scene_variant(world, decoded["scene"])
        cleanup_existing_ego_actors(world)

        state.set_step("apply_weather", f"{decoded['sky']} / {decoded['sunshinetime']}")
        apply_custom_weather(world, decoded["sky"], decoded["sunshinetime"])

        vehicle_config, vehicle_path, vehicle_model = load_vehicle_config(decoded["vehiclemodel"])
        ui_params = build_ui_params(vehicle_model, vehicle_config)
        with state.lock:
            state.vehicle_config = vehicle_config
            state.vehicle_model = vehicle_model
            state.ui_params = ui_params

        if vehicle_config is None:
            with state.lock:
                state.vehicle = None
                state.last_result = {"ok": True, "msg": "world loaded without vehicle"}
            return

        state.set_step("spawn_vehicle", f"{vehicle_model} from {vehicle_path.name if vehicle_path else 'default'}")
        vehicle = spawn_vehicle(world, vehicle_config, vehicle_model, decoded["scene"])
        with state.lock:
            state.vehicle = vehicle

        if decoded["load_traffic"]:
            state.set_step("spawn_traffic", f"spawning up to {TRAFFIC_COUNT} native autopilot vehicles")
            state.traffic_actors = spawn_traffic(world, client, TRAFFIC_COUNT, vehicle.get_location())

        state.set_step("start_runtime", f"mode={decoded['drive_mode_label']}, view={decoded['camera_view_label']}")
        state.runtime_stop = threading.Event()
        thread = threading.Thread(target=runtime_loop, args=(vehicle, world, client, state.runtime_stop), daemon=True)
        state.runtime_thread = thread
        thread.start()

        time.sleep(0.2)
        if is_manual_mode(decoded["drive_mode"]):
            start_manual_bridge()
        elif is_algo_mode(decoded["drive_mode"]):
            start_algorithm(decoded["scene"])

        with state.lock:
            state.last_result = {
                "ok": True,
                "msg": "native CARLA demo ready",
                "actor_id": vehicle.id,
                "mode": decoded["drive_mode"],
                "view": decoded["camera_view"],
            }
            state.carla_status_message = "native demo ready"
    except Exception as exc:
        with state.lock:
            state.last_error = str(exc)
            state.last_result = {"ok": False, "msg": str(exc)}
        print(f"[native-demo] deployment failed: {exc}", flush=True)
    finally:
        try:
            if state.client is not None:
                state.client.set_timeout(CARLA_TIMEOUT)
        except Exception:
            pass
        with state.lock:
            state.deployment_active = False
        write_status_file(state.snapshot())
        state.deploy_lock.release()


def stop_demo() -> Dict[str, Any]:
    cleanup_runtime(stop_bridge=True)
    with state.lock:
        state.deployment_active = False
        state.deployment_step = "stopped"
        state.last_result = {"ok": True, "msg": "native demo stopped"}
    write_status_file(state.snapshot())
    return state.snapshot()


class Handler(BaseHTTPRequestHandler):
    server_version = "NativeCarlaDemo/1.0"

    def log_message(self, fmt: str, *args: Any) -> None:
        print(f"[native-demo-http] {self.client_address[0]} {fmt % args}", flush=True)

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

    def read_body_json(self) -> Dict[str, Any]:
        length = int(self.headers.get("Content-Length", "0") or "0")
        raw = self.rfile.read(length) if length > 0 else b"{}"
        if not raw:
            return {}
        return json.loads(raw.decode("utf-8"))

    def do_OPTIONS(self) -> None:
        self.send_json(200, {"ok": True})

    def do_GET(self) -> None:
        if self.path.split("?", 1)[0] in {"/", "/health"}:
            snap = state.snapshot()
            write_status_file(snap)
            self.send_json(200, snap)
            return
        self.send_json(404, {"ok": False, "msg": "endpoint not found"})

    def do_POST(self) -> None:
        path = self.path.split("?", 1)[0]
        if path not in {"/command", "/view"}:
            self.send_json(404, {"ok": False, "msg": "endpoint not found"})
            return
        try:
            body = self.read_body_json()
        except Exception as exc:
            self.send_json(400, {"ok": False, "msg": f"invalid json: {exc}"})
            return

        if path == "/view":
            view = state.set_camera_view(first_value(body, "camera_view", "Camera View", "view", "View", "视角", "camera", default=state.camera_view))
            try:
                if state.world is not None and safe_actor_alive(state.vehicle):
                    update_spectator(state.world, state.vehicle, view)
            except Exception:
                pass
            snap = state.snapshot()
            self.send_json(200, {
                "ok": True,
                "msg": f"camera view switched to {camera_view_label(view)}",
                "camera_view": view,
                "camera_view_label": camera_view_label(view),
                "camera_stream_url": camera_stream_url(view),
                "camera_streams": camera_streams(),
                "camera_stream_reload_key": snap["camera_stream_reload_key"],
                "vehicle_alive": snap["vehicle_alive"],
                "target_ip": snap["target_ip"],
            })
            return

        sendstate = str(first_value(body, "sendstate", "Send State", default="START")).strip().upper()
        if sendstate in {"STOP", "END", "QUIT", "CLOSE"}:
            snap = stop_demo()
            self.send_json(200, snap)
            return

        body["_client_ip"] = self.client_address[0]
        view = state.set_camera_view(first_value(body, "camera_view", "Camera View", "view", "View", "视角", "camera", default=state.camera_view))
        if state.deploy_lock.locked():
            snap = state.snapshot()
            self.send_json(202, {
                "ok": True,
                "msg": "deployment already running",
                "last_result": snap["last_result"],
                "vehicle_alive": snap["vehicle_alive"],
                "camera_view": view,
                "camera_stream_url": camera_stream_url(view),
                "camera_streams": camera_streams(),
                "telemetry": state.last_telemetry,
            })
            return

        with state.lock:
            state.last_result = {"ok": True, "msg": "command accepted, deployment running"}
            state.deployment_step = "queued"
        threading.Thread(target=deploy_command, args=(dict(body),), daemon=True).start()
        snap = state.snapshot()
        self.send_json(202, {
            "ok": True,
            "msg": "command accepted, deployment running",
            "target_ip": body["_client_ip"],
            "camera_view": view,
            "camera_view_label": camera_view_label(view),
            "camera_stream_url": camera_stream_url(view),
            "camera_streams": camera_streams(),
            "camera_stream_reload_key": snap["camera_stream_reload_key"],
            "vehicle_alive": snap["vehicle_alive"],
            "telemetry": state.last_telemetry,
        })


class ReusableThreadingHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True


def start_server() -> ThreadingHTTPServer:
    server = ReusableThreadingHTTPServer((API_HOST, API_PORT), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    print(f"[native-demo] HTTP API listening on http://{API_HOST}:{API_PORT}", flush=True)
    return server


def main() -> None:
    os.chdir(PROJECT_ROOT)
    LOG_DIR.mkdir(exist_ok=True)
    server = start_server()
    ok, msg = ensure_carla_connection(timeout=2.0)
    print(f"[native-demo] startup CARLA probe: {msg}", flush=True)
    write_status_file(state.snapshot())
    try:
        while True:
            time.sleep(3600)
    except KeyboardInterrupt:
        pass
    finally:
        cleanup_runtime(stop_bridge=True)
        server.shutdown()
        server.server_close()


if __name__ == "__main__":
    main()
