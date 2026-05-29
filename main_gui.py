import streamlit as st
import json
import os
import threading
import time
import carla
import random
import math
import socket
import pandas as pd
import subprocess
import signal
import sys
from pathlib import Path
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from collections import deque
from typing import Dict, List

st.set_page_config(page_title="L4 Runtime Monitor", layout="wide", initial_sidebar_state="collapsed")

# ==========================================
# 0. 场景要素配置 & 算法工况映射 (绝对锁定版)
# ==========================================
TM_PORT = 8010
VEHICLE_DIR = "output"
API_PORT = int(os.environ.get("CARLA_API_PORT", "8765"))
CARLA_HOST = os.environ.get("CARLA_HOST", "127.0.0.1")
CARLA_PORT = int(os.environ.get("CARLA_PORT", "2000"))
CARLA_CONNECT_TIMEOUT = float(os.environ.get("CARLA_CONNECT_TIMEOUT", "10.0"))
CARLA_DEPLOY_TIMEOUT = float(os.environ.get("CARLA_DEPLOY_TIMEOUT", "60.0"))
CARLA_RUNTIME_TIMEOUT = float(os.environ.get("CARLA_RUNTIME_TIMEOUT", "20.0"))
CARLA_AUTO_CONNECT_INTERVAL = float(os.environ.get("CARLA_AUTO_CONNECT_INTERVAL", "3.0"))
AI_FALLBACK_AFTER_SEC = float(os.environ.get("AI_FALLBACK_AFTER_SEC", "2.0"))
AI_FALLBACK_TARGET_SPEED_KMH = float(os.environ.get("AI_FALLBACK_TARGET_SPEED_KMH", "25.0"))
AI_FALLBACK_TARGET_SPEED_MS = AI_FALLBACK_TARGET_SPEED_KMH / 3.6


def env_flag(name, default="0"):
    return os.environ.get(name, default).strip().lower() in {"1", "true", "yes", "on"}


CAMERA_VIEW_DEFAULT = os.environ.get("CAMERA_VIEW_DEFAULT", "follow")
CAMERA_STREAM_RTSP_HOST = os.environ.get(
    "CAMERA_STREAM_RTSP_HOST",
    os.environ.get("MIRROR_RTSP_PUBLIC_HOST", "192.168.110.100")
)
CAMERA_STREAM_RTSP_PORT = int(os.environ.get(
    "CAMERA_STREAM_RTSP_PORT",
    os.environ.get("MIRROR_RTSP_PORT", "8554")
))
CAMERA_SINGLE_ACTIVE_STREAM = env_flag("CAMERA_SINGLE_ACTIVE_STREAM", "1")
CAMERA_ACTIVE_STREAM_PATH = os.environ.get(
    "CAMERA_ACTIVE_STREAM_PATH",
    os.environ.get("MIRROR_ACTIVE_VIEW_PATH", "carla_view")
)
CAMERA_VIEW_STREAM_PATHS = {
    "follow": os.environ.get("CAMERA_FOLLOW_STREAM_PATH", "carla_follow"),
    "driver": os.environ.get("CAMERA_DRIVER_STREAM_PATH", "carla_driver"),
}
CAMERA_SIDE_STREAM_PATHS = {
    "rear_left": os.environ.get("CAMERA_REAR_LEFT_STREAM_PATH", "carla_rear_left"),
    "rear_right": os.environ.get("CAMERA_REAR_RIGHT_STREAM_PATH", "carla_rear_right"),
}
MANUAL_BRIDGE_SCRIPT = os.environ.get("MANUAL_BRIDGE_SCRIPT", "can_tcp_bridge_vcu.py")
MANUAL_CONTROL_TIMEOUT_SEC = float(os.environ.get("MANUAL_CONTROL_TIMEOUT_SEC", "0.5"))
MANUAL_BRIDGE_WATCHDOG_INTERVAL_SEC = float(os.environ.get("MANUAL_BRIDGE_WATCHDOG_INTERVAL_SEC", "1.0"))
MANUAL_BRIDGE_RESTART_MIN_SEC = float(os.environ.get("MANUAL_BRIDGE_RESTART_MIN_SEC", "2.0"))
MANUAL_BRIDGE_STATUS_FILE = os.environ.get("MANUAL_BRIDGE_STATUS_FILE", "logs/manual_bridge_status.json")
CENTER_DISPLAY_AUTO_START = env_flag("CENTER_DISPLAY_AUTO_START", "0")
CENTER_DISPLAY_SERVICE = os.environ.get("CENTER_DISPLAY_SERVICE", "carla-center-display.service")
CENTER_DISPLAY_START_COOLDOWN_SEC = float(os.environ.get("CENTER_DISPLAY_START_COOLDOWN_SEC", "3.0"))
_center_display_last_start = 0.0
_center_display_lock = threading.Lock()

DEFAULT_REQUIRED_COUNTS = {
    "vehicle_models": 10,
    "traffic_standards": 10,
    "barriers": 16,
    "normal_vehicles": 15,
    "emergency_vehicles": 1,
    "walkers": 5,
    "bicycles": 2,
    "animals": 1
}
NO_TRAFFIC_COUNTS = {key: 0 for key in DEFAULT_REQUIRED_COUNTS}

HIDDEN_TRAFFIC_COUNTS = DEFAULT_REQUIRED_COUNTS.copy()
HIDDEN_TRAFFIC_COUNTS.update({
    "vehicle_models": 0,
    "normal_vehicles": 0,
    "emergency_vehicles": 0,
    "walkers": 0,
    "bicycles": 0,
    "animals": 0,
})
HIDDEN_TRAFFIC_MIN_DISTANCE_M = float(os.environ.get("HIDDEN_TRAFFIC_MIN_DISTANCE_M", "180.0"))

REQUIRED_COUNTS = DEFAULT_REQUIRED_COUNTS.copy()

FRONTEND_OUTPUT_FIELDS = [
    "Vehiclemodel", "Overall", "Wheelbase", "Tirebase", "Tireradius", "Empty", "Gravity",
    "Axle", "Unloaded", "Moment", "Ratedtotalmass", "Drag", "Windward", "LiftCoefficientCl",
    "Pitching", "Latera", "LeftFrontWheelRotation", "RightFrontWheelRotation",
    "LeftRearWheelRotation", "RightRearWheelRotation", "LeftFrontWheelVibration",
    "RightFrontWheelVibration", "LeftRearWheelVibration", "RightRearWheelVibration",
    "LeftFrontWheel", "RightFrontWheel", "LeftRearWheel", "RightRearWheel",
    "RadialDeformationLeftFrontWheel", "RadialDeformationRightFrontWheel",
    "RadialDeformationLeftRearWheel", "RadialDeformationRightRearWheel", "PitchAngle",
    "RollAngle", "LateralSwingAngle", "LongitudinalDisplacement", "LateralDisplacement",
    "VerticalDisplacement", "TurningTheSteeringWheel", "LeftFrontWheelAngle",
    "RightFrontWheelAngle", "SteeringColumnTorsion", "EngineCrankshaftRotates",
    "LeftDriveHalfShaftTwist", "RightDriveHalfShaftTwist"
]

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


class CarlaEncoder:
    SCENE_MAP = {
        "Town01/Urban City District": "Town01", "Town01": "Town01",
        "Town02/Low-Density Suburban Area": "Town02", "Town02": "Town02",
        "Town03/High-Density Residential Zone": "Town03", "Town03": "Town03",
        "Town04/High-Speed Expressway": "Town04", "Town04": "Town04",
        "Town05/Performance Proving Ground": "Town05", "Town05": "Town05",
        "TrainingGround": "TrainingGround", "训练场": "TrainingGround",
        "Town04Forest": "TrainingGround", "Town04Forest/Town04 Forest Road": "TrainingGround",
    }
    WEATHER_MAP = {
        "Sunny": "晴天", "Cloudy": "多云", "Light Rain": "小雨",
        "Heavy Rainstorm": "暴雨", "Fog/Dense Fog": "大雾", "Clear": "晴天",
    }
    TIME_MAP = {
        "Noon": "正午", "Sunset": "夕阳", "Late Night": "深夜",
    }
    MODE_MAP = {
        "Manual": "🛞 硬件在环手动模式 (台架驾驶，无挂载)",
        "manual": "🛞 硬件在环手动模式 (台架驾驶，无挂载)",
        "HIL": "🛞 硬件在环手动模式 (台架驾驶，无挂载)",
        "硬件在环": "🛞 硬件在环手动模式 (台架驾驶，无挂载)",
        "手动": "🛞 硬件在环手动模式 (台架驾驶，无挂载)",
        "AI": "🤖 内置 AI 巡航模式 (纯漫游，无挂载)",
        "ai": "🤖 内置 AI 巡航模式 (纯漫游，无挂载)",
        "内置 AI": "🤖 内置 AI 巡航模式 (纯漫游，无挂载)",
        "内置AI": "🤖 内置 AI 巡航模式 (纯漫游，无挂载)",
        "AI巡航": "🤖 内置 AI 巡航模式 (纯漫游，无挂载)",
        "Algo": "🧠 自动驾驶域控模式 (算法对接 | 将自动后台拉起脚本)",
        "algo": "🧠 自动驾驶域控模式 (算法对接 | 将自动后台拉起脚本)",
        "AIGO": "🧠 自动驾驶域控模式 (算法对接 | 将自动后台拉起脚本)",
        "aigo": "🧠 自动驾驶域控模式 (算法对接 | 将自动后台拉起脚本)",
        "算法": "🧠 自动驾驶域控模式 (算法对接 | 将自动后台拉起脚本)",
        "算法对接": "🧠 自动驾驶域控模式 (算法对接 | 将自动后台拉起脚本)",
        "自动驾驶": "🧠 自动驾驶域控模式 (算法对接 | 将自动后台拉起脚本)",
        "自动驾驶域控模式": "🧠 自动驾驶域控模式 (算法对接 | 将自动后台拉起脚本)",
    }
    TRAFFIC_LOAD_MAP = {"0": True, 0: True, "1": False, 1: False}
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

    @staticmethod
    def first_value(body: dict, *keys, default=None):
        for key in keys:
            if key in body and body.get(key) is not None:
                return body.get(key)
        return default

    @staticmethod
    def decode_mode(value):
        text = str(value if value is not None else "AI").strip()
        if text in CarlaEncoder.MODE_MAP:
            return CarlaEncoder.MODE_MAP[text]
        lower_text = text.lower()
        if "aigo" in lower_text or "algo" in lower_text or "算法" in text or "自动驾驶" in text or "域控" in text:
            return CarlaEncoder.MODE_MAP["Algo"]
        if "manual" in lower_text or "hil" in lower_text or "手动" in text or "硬件在环" in text:
            return CarlaEncoder.MODE_MAP["Manual"]
        if lower_text == "ai" or "内置" in text or "巡航" in text or "纯漫游" in text:
            return CarlaEncoder.MODE_MAP["AI"]
        return CarlaEncoder.MODE_MAP["AI"]


CAMERA_VIEW_ALIASES = {
    "follow": "follow",
    "third": "follow",
    "third_person": "follow",
    "third-person": "follow",
    "third person": "follow",
    "chase": "follow",
    "rear": "follow",
    "spectator": "follow",
    "default": "follow",
    "跟车": "follow",
    "第三视角": "follow",
    "第三人称": "follow",
    "外部视角": "follow",
    "driver": "driver",
    "first": "driver",
    "first_person": "driver",
    "first-person": "driver",
    "first person": "driver",
    "cockpit": "driver",
    "ego": "driver",
    "驾驶员": "driver",
    "驾驶员视角": "driver",
    "第一视角": "driver",
    "第一人称": "driver",
    "座舱": "driver",
    "舱内": "driver",
}


def normalize_camera_view(value, default=CAMERA_VIEW_DEFAULT):
    raw = str(value if value is not None else default).strip()
    if not raw:
        raw = str(default or "follow")
    normalized = CAMERA_VIEW_ALIASES.get(raw)
    if normalized:
        return normalized
    lowered = raw.lower().replace("-", "_").strip()
    normalized = CAMERA_VIEW_ALIASES.get(lowered)
    if normalized:
        return normalized
    if "driver" in lowered or "first" in lowered or "cockpit" in lowered or "驾驶" in raw or "第一" in raw or "座舱" in raw:
        return "driver"
    return "follow"


def camera_view_label(view):
    return "驾驶员第一视角" if normalize_camera_view(view) == "driver" else "第三人称跟车视角"


def camera_view_stream_url(view):
    normalized = normalize_camera_view(view)
    if CAMERA_SINGLE_ACTIVE_STREAM:
        path = CAMERA_ACTIVE_STREAM_PATH
    else:
        path = CAMERA_VIEW_STREAM_PATHS.get(normalized, CAMERA_VIEW_STREAM_PATHS["follow"])
    return f"rtsp://{CAMERA_STREAM_RTSP_HOST}:{CAMERA_STREAM_RTSP_PORT}/{path}"


def camera_view_streams():
    if CAMERA_SINGLE_ACTIVE_STREAM:
        url = f"rtsp://{CAMERA_STREAM_RTSP_HOST}:{CAMERA_STREAM_RTSP_PORT}/{CAMERA_ACTIVE_STREAM_PATH}"
        return {"active": url, "follow": url, "driver": url}
    return {
        view: f"rtsp://{CAMERA_STREAM_RTSP_HOST}:{CAMERA_STREAM_RTSP_PORT}/{path}"
        for view, path in CAMERA_VIEW_STREAM_PATHS.items()
    }


def camera_side_streams():
    return {
        name: f"rtsp://{CAMERA_STREAM_RTSP_HOST}:{CAMERA_STREAM_RTSP_PORT}/{path}"
        for name, path in CAMERA_SIDE_STREAM_PATHS.items()
    }


def ensure_center_display_started():
    global _center_display_last_start
    if not CENTER_DISPLAY_AUTO_START or os.name == "nt":
        return
    now = time.time()
    with _center_display_lock:
        if now - _center_display_last_start < CENTER_DISPLAY_START_COOLDOWN_SEC:
            return
        _center_display_last_start = now
    try:
        subprocess.run(
            ["systemctl", "--user", "start", CENTER_DISPLAY_SERVICE],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
            timeout=2,
        )
    except Exception as exc:
        print(f"center display auto-start skipped: {exc}")


def fmt_frontend(value, digits=3, unit=""):
    try:
        if isinstance(value, bool):
            text = "true" if value else "false"
        elif isinstance(value, (int, float)):
            text = f"{float(value):.{digits}f}".rstrip("0").rstrip(".")
            if "." not in text:
                text = f"{float(value):.{digits}f}"
            elif digits > 0:
                text = text.rstrip("0").rstrip(".")
        else:
            text = str(value)
    except Exception:
        text = ""
    return f"{text} {unit}".strip() if unit else text


def fmt_xyz(data, digits=3):
    if not isinstance(data, dict):
        data = {}
    return f"({fmt_frontend(data.get('x', 0.0), digits)}, {fmt_frontend(data.get('y', 0.0), digits)}, {fmt_frontend(data.get('z', 0.0), digits)})"


def nested_get(data, *keys, default=None):
    cur = data
    for key in keys:
        if not isinstance(cur, dict):
            return default
        cur = cur.get(key)
        if cur is None:
            return default
    return cur


def normalize_frontend_payload(payload):
    return {field: "" if payload.get(field) is None else str(payload.get(field, "")) for field in FRONTEND_OUTPUT_FIELDS}


def vehicle_geometry_params(vehicle_model=None):
    model = vehicle_model or sim_state.current_vehicle_model or "Lincoln MKZ"
    geometry = VEHICLE_GEOMETRY_SPECS.get(model, VEHICLE_GEOMETRY_SPECS["Lincoln MKZ"])
    wheelbase = float(geometry.get("Wheelbase", 2.88) or 2.88)
    tirebase = float(geometry.get("Tirebase", 1.58) or 1.58)
    tire_radius = float(geometry.get("Tireradius", 0.35) or 0.35)
    return {
        "wheelbase": wheelbase,
        "track": tirebase,
        "tire_radius": tire_radius,
        "a": round(wheelbase * 0.517, 3),
        "b": round(wheelbase * 0.483, 3),
    }


def is_ai_mode(mode_text: str) -> bool:
    text = str(mode_text or "")
    lower_text = text.lower()
    return not is_algo_mode(text) and (
        lower_text == "ai"
        or "内置 ai" in lower_text
        or "ai 巡航" in lower_text
        or "纯漫游" in text
    )


def is_algo_mode(mode_text: str) -> bool:
    text = str(mode_text or "")
    lower_text = text.lower()
    return "algo" in lower_text or "aigo" in lower_text or "算法" in text or "域控" in text


def is_manual_mode(mode_text: str) -> bool:
    return "Manual" in mode_text or "硬件在环" in mode_text or "手动模式" in mode_text


def build_frontend_telemetry(v_cfg_data, ui_params, raw_telemetry, vehicle_actor):
    payload = {}
    vehicle_meta = v_cfg_data.get("vehicle_metadata", {})
    mass_props = v_cfg_data.get("weight_and_mass_properties", {})
    aero_props = v_cfg_data.get("aerodynamic_parameters", {})
    moment = mass_props.get("moment_of_inertia", {})
    cog = mass_props.get("center_of_gravity_m", {})

    rigid = nested_get(raw_telemetry, "1_刚体运动学 (Rigid Body Kinematics)", default={}) or {}
    wheel_dyn = nested_get(raw_telemetry, "2_轮端与底盘动态 (Wheel Dynamics)", default={}) or {}
    control_state = nested_get(raw_telemetry, "3_驾驶控制反读 (Control State)", default={}) or {}

    pos_xyz = nested_get(rigid, "1_全局绝对坐标_XYZ_米", default=[0.0, 0.0, 0.0]) or [0.0, 0.0, 0.0]
    angle_pyr = nested_get(rigid, "2_姿态角_俯仰_偏航_滚转_度", default=[0.0, 0.0, 0.0]) or [0.0, 0.0, 0.0]
    wheel_rpm = (
        nested_get(wheel_dyn, "6_四轮独立转速_RPM_左前_右前_左后_右后", default=None)
        or nested_get(wheel_dyn, "6_四轮独立转速RPM_左前_右前_左后_右后", default=None)
        or [0.0, 0.0, 0.0, 0.0]
    )
    wheel_bounce = (
        nested_get(wheel_dyn, "7_悬架实时压缩量_毫米_左前_右前_左后_右后", default=None)
        or nested_get(wheel_dyn, "7_悬架实时压缩量mm_左前_右前_左后_右后", default=None)
        or [0.0, 0.0, 0.0, 0.0]
    )
    steer_pair = nested_get(wheel_dyn, "8_前轮真实阿克曼转向角_度", default=[0.0, 0.0]) or [0.0, 0.0]

    throttle = float(nested_get(control_state, "9_实际油门开度_0至1", default=0.0) or 0.0)
    steer = float(nested_get(control_state, "11_方向盘转角_负1至1", default=0.0) or 0.0)

    try:
        vehicle_control = vehicle_actor.get_control() if vehicle_actor and vehicle_actor.is_alive else None
    except Exception:
        vehicle_control = None

    wheel_radius = float(VEHICLE_GEOMETRY_SPECS.get(sim_state.current_vehicle_model, {}).get("Tireradius", "0.35") or 0.35)
    overall = VEHICLE_GEOMETRY_SPECS.get(sim_state.current_vehicle_model, {}).get("Overall", "")
    wheelbase = VEHICLE_GEOMETRY_SPECS.get(sim_state.current_vehicle_model, {}).get("Wheelbase", "2.88")
    tirebase = VEHICLE_GEOMETRY_SPECS.get(sim_state.current_vehicle_model, {}).get("Tirebase", "1.58")

    payload["Vehiclemodel"] = sim_state.current_vehicle_model or vehicle_meta.get("official_name", "")
    payload["Overall"] = overall
    payload["Wheelbase"] = wheelbase
    payload["Tirebase"] = tirebase
    payload["Tireradius"] = fmt_frontend(wheel_radius, 3)
    payload["Empty"] = fmt_frontend(mass_props.get("curb_weight_kg", ui_params.get("mass", 0.0)), 0, "kg")
    payload["Gravity"] = fmt_xyz(cog, 3)
    payload["Axle"] = fmt_frontend(mass_props.get("sprung_mass_kg", 0.0), 0, "kg")
    payload["Unloaded"] = fmt_frontend(mass_props.get("total_unsprung_mass_kg", 0.0), 0, "kg")
    payload["Moment"] = f"({fmt_frontend(moment.get('Ixx', 0.0), 0)}, {fmt_frontend(moment.get('Iyy', 0.0), 0)}, {fmt_frontend(moment.get('Izz', 0.0), 0)})"
    payload["Ratedtotalmass"] = fmt_frontend(mass_props.get("gross_vehicle_weight_rating_kg", 0.0), 0, "kg")
    payload["Drag"] = fmt_frontend(aero_props.get("drag_coefficient_cd", ui_params.get("cd", 0.0)), 3)
    payload["Windward"] = fmt_frontend(aero_props.get("frontal_area_sqm", ui_params.get("area", 0.0)), 3, "m2")
    payload["LiftCoefficientCl"] = fmt_frontend(aero_props.get("lift_coefficient_cl", ui_params.get("cl", 0.0)), 3)
    payload["Pitching"] = fmt_frontend(aero_props.get("pitch_moment_coefficient", ui_params.get("cm", 0.0)), 3)
    payload["Latera"] = fmt_frontend(aero_props.get("side_force_coefficient_cy", ui_params.get("cy", 0.0)), 3)

    rpm_values = (wheel_rpm + [0.0, 0.0, 0.0, 0.0])[:4]
    vib_values = (wheel_bounce + [0.0, 0.0, 0.0, 0.0])[:4]
    static_load = float(ui_params.get("mass", 1500.0)) * 9.81 / 4.0
    deformation = [max(0.0, abs(v) * 0.01) for v in vib_values]
    slip_front = max(0.0, throttle * 4.0)
    slip_rear = max(0.0, throttle * 2.0)

    payload["LeftFrontWheelRotation"] = fmt_frontend(rpm_values[0], 1, "RPM")
    payload["RightFrontWheelRotation"] = fmt_frontend(rpm_values[1], 1, "RPM")
    payload["LeftRearWheelRotation"] = fmt_frontend(rpm_values[2], 1, "RPM")
    payload["RightRearWheelRotation"] = fmt_frontend(rpm_values[3], 1, "RPM")
    payload["LeftFrontWheelVibration"] = fmt_frontend(static_load + vib_values[0], 0, "N")
    payload["RightFrontWheelVibration"] = fmt_frontend(static_load + vib_values[1], 0, "N")
    payload["LeftRearWheelVibration"] = fmt_frontend(static_load + vib_values[2], 0, "N")
    payload["RightRearWheelVibration"] = fmt_frontend(static_load + vib_values[3], 0, "N")
    payload["LeftFrontWheel"] = fmt_frontend(slip_front, 2)
    payload["RightFrontWheel"] = fmt_frontend(slip_front, 2)
    payload["LeftRearWheel"] = fmt_frontend(slip_rear, 2)
    payload["RightRearWheel"] = fmt_frontend(slip_rear, 2)
    payload["RadialDeformationLeftFrontWheel"] = fmt_frontend(deformation[0], 2, "mm")
    payload["RadialDeformationRightFrontWheel"] = fmt_frontend(deformation[1], 2, "mm")
    payload["RadialDeformationLeftRearWheel"] = fmt_frontend(deformation[2], 2, "mm")
    payload["RadialDeformationRightRearWheel"] = fmt_frontend(deformation[3], 2, "mm")

    pitch = angle_pyr[0] if len(angle_pyr) > 0 else 0.0
    yaw = angle_pyr[1] if len(angle_pyr) > 1 else 0.0
    roll = angle_pyr[2] if len(angle_pyr) > 2 else 0.0
    payload["PitchAngle"] = fmt_frontend(pitch, 2, "deg")
    payload["RollAngle"] = fmt_frontend(roll, 2, "deg")
    payload["LateralSwingAngle"] = fmt_frontend(yaw, 2, "deg")
    payload["LongitudinalDisplacement"] = fmt_frontend(pos_xyz[0] if len(pos_xyz) > 0 else 0.0, 2, "m")
    payload["LateralDisplacement"] = fmt_frontend(pos_xyz[1] if len(pos_xyz) > 1 else 0.0, 2, "m")
    payload["VerticalDisplacement"] = fmt_frontend(pos_xyz[2] if len(pos_xyz) > 2 else 0.0, 2, "m")

    payload["TurningTheSteeringWheel"] = fmt_frontend(steer * 540.0, 2, "deg")
    payload["LeftFrontWheelAngle"] = fmt_frontend(steer_pair[0] if len(steer_pair) > 0 else 0.0, 2, "deg")
    payload["RightFrontWheelAngle"] = fmt_frontend(steer_pair[1] if len(steer_pair) > 1 else 0.0, 2, "deg")
    payload["SteeringColumnTorsion"] = fmt_frontend(steer * 27.0, 2, "deg")

    mean_rpm = sum(rpm_values) / max(len(rpm_values), 1)
    engine_rpm = max(0.0, mean_rpm * float(ui_params.get("final_ratio", 4.0)))
    payload["EngineCrankshaftRotates"] = fmt_frontend(engine_rpm, 0, "RPM")
    payload["LeftDriveHalfShaftTwist"] = fmt_frontend(throttle * 8.5, 2, "deg")
    payload["RightDriveHalfShaftTwist"] = fmt_frontend(throttle * 8.5, 2, "deg")

    return normalize_frontend_payload(payload)


# 🚀 算法工况数据库
SCENARIO_DATABASE = {
    "Town01": {"pos": (-2.0, 8.0, 2.0, 90.0), "script": "vshuangyi.py", "task": "DLC 双移线紧急避险"},
    "Town02": {"pos": (3.0, 109.5, 2.0, 0), "script": "vdanyi.py", "task": "单移线避障测试"},
    "Town03": {"pos": (-42.0, 204.0, 2.0, 0.0), "script": "vjiansu.py", "task": "动态速度廓线 (减速)"},
    "Town04": {"pos": (9.0, 237.0, 2.0, -90.0), "script": "vshexing.py", "task": "长距离蛇行绕桩"},
    "Town05": {"pos": (206.6, 110.0, 2.0, -90.0), "script": "vjiasu.py", "task": "起步与定距停车 (加速)"},
    "TrainingGround": {
        "pos": (9.0, 237.0, 2.0, -90.0),
        "script": "vshexing.py",
        "task": "训练场",
        "runtime_map": "Town04",
        "prefer_opt": True,
        "unload_buildings": True,
    },
}

# ==========================================
# 1. 核心初始化 & 铁壁防漏共享内存池
# ==========================================
class SimulationState:
    def __init__(self):
        self.reset()
        self.drive_mode = "🤖 内置 AI 巡航模式 (纯漫游，无挂载)" 
        self.filter_alpha = 0.15  
        self.smoothed_speed = 0.0
        self.target_ip = "127.0.0.1"
        self.scene_actors = []
        self.scene_walker_controllers = []
        self.scene_summary = None
        self.client = None
        self.world = None
        self.vehicle = None
        self.active_sensors = []
        self.stop_event = threading.Event()
        self.master_thread = None
        self.dynamics_wrapper = None
        self.algo_process = None
        self.algo_log_file = None
        self.manual_bridge_process = None
        self.manual_bridge_log_file = None
        self.manual_bridge_log_path = None
        self.manual_bridge_restart_count = 0
        self.manual_bridge_last_start = 0.0
        self.manual_bridge_watchdog_stop = threading.Event()
        self.manual_bridge_watchdog_thread = None
        self.last_manual_control_time = 0.0
        self.last_manual_control_cmd = None
        self.current_world_name = None
        self.current_vehicle_model = "Lincoln MKZ"
        self.camera_view = normalize_camera_view(CAMERA_VIEW_DEFAULT)
        self.camera_view_revision = 0
        self.carla_connected = False
        self.carla_status_message = "Carla not connected"
        self.carla_status_updated_at = time.strftime("%Y-%m-%d %H:%M:%S")
        self.last_command = {}
        self.last_pipeline_result = {}
        self.lock = threading.RLock()

    def reset(self):
        self.data = {
            "SPEED": 0.0, 
            "GNSS_DATA": [0.0, 0.0, 0.0], 
            "IMU_DATA": {"Accel": [0,0,0], "Gyro": [0,0,0], "Compass": 0.0}, 
            "RADAR_TARGETS": 0,
            "COLLISION_DATA": {"Impulse": [0,0,0], "Actor": "None"},
            "FULL_TELEMETRY": {},
            "LEGACY_TELEMETRY": {}
        }
        self.frame_count = 0
        self.speed_history = deque(maxlen=120) 
        self.smoothed_speed = 0.0

    def snapshot(self):
        with self.lock:
            return {
                "data": json.loads(json.dumps(self.data, ensure_ascii=False)),
                "drive_mode": self.drive_mode,
                "target_ip": self.target_ip,
                "scene_summary": self.scene_summary,
                "carla_connected": self.carla_connected,
                "carla_status_message": self.carla_status_message,
                "carla_status_updated_at": self.carla_status_updated_at,
                "current_world_name": self.current_world_name,
                "current_vehicle_model": self.current_vehicle_model,
                "camera_view": self.camera_view,
                "camera_view_label": camera_view_label(self.camera_view),
                "camera_stream_url": camera_view_stream_url(self.camera_view),
                "camera_streams": camera_view_streams(),
                "side_camera_streams": camera_side_streams(),
                "camera_stream_reload_key": f"{self.camera_view}:{self.camera_view_revision}",
                "last_command": self.last_command,
                "last_pipeline_result": self.last_pipeline_result,
                "paused": False,
                "manual_bridge": get_manual_bridge_status(),
            }

    def set_last_command(self, raw_command, decoded_command):
        with self.lock:
            self.last_command = {
                "received_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                "raw": raw_command,
                "decoded": decoded_command,
            }

    def set_camera_view(self, view):
        normalized = normalize_camera_view(view, self.camera_view)
        with self.lock:
            if normalized != self.camera_view:
                self.camera_view_revision += 1
                self.camera_view = normalized
        return normalized

    def set_last_pipeline_result(self, ok, msg):
        with self.lock:
            self.last_pipeline_result = {
                "updated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                "ok": ok,
                "msg": msg,
            }

    def set_pipeline_step(self, step, msg=None):
        with self.lock:
            self.last_pipeline_result = {
                "updated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                "ok": True,
                "step": step,
                "msg": msg or step,
            }

    def set_carla_status(self, connected: bool, msg: str, world_name=None):
        with self.lock:
            self.carla_connected = connected
            self.carla_status_message = msg
            self.carla_status_updated_at = time.strftime("%Y-%m-%d %H:%M:%S")
            if world_name is not None or not connected:
                self.current_world_name = world_name

@st.cache_resource
def get_sim_state():
    return SimulationState()

sim_state = get_sim_state()
deployment_lock = threading.Lock()
auto_connector_started = threading.Event()
api_server_started = threading.Event()

# ==========================================
# 2. 物理引擎与底层真值包装器 (原汁原味，无任何干预)
# ==========================================
class L4_DynamicsWrapper:
    def __init__(self, vehicle_actor, json_config, ui_overrides, world):
        self.vehicle = vehicle_actor
        self.config = json_config
        self.ui = ui_overrides
        self.world = world
        self.is_active = True
        self.air_density = 1.225 
        
        self._inject_full_physics_control()
        self.aero_thread = threading.Thread(target=self._aero_dynamics_loop, daemon=True)
        self.aero_thread.start()

    def _inject_full_physics_control(self):
        try:
            pc = self.vehicle.get_physics_control()
            pc.mass = float(self.ui.get('mass', getattr(pc, "mass", 1500.0)))
            pc.moi = float(self.ui.get('moi', getattr(pc, "moi", 1.0)))
            pc.center_of_gravity = carla.Vector3D(
                x=float(self.ui.get('cg_x', 0)),
                y=float(self.ui.get('cg_y', 0)),
                z=float(self.ui.get('cg_z', 0))
            )
            pc.drag_coefficient = float(self.ui.get('cd', getattr(pc, "drag_coefficient", 0.3)))

            wheels = pc.wheels
            for i, wheel in enumerate(wheels):
                wheel.tire_friction = float(self.ui.get('friction', 3.5))
                if i < 2:
                    wheel.max_steer_angle = float(self.ui.get('steer', 40.0))
            pc.wheels = wheels

            self.vehicle.apply_physics_control(pc)
            print("Applied mass/aero parameters; CARLA tire stiffness model preserved.")
        except Exception as e:
            print(f"physics parameter apply failed, keeping CARLA defaults: {e}")

    def _aero_dynamics_loop(self):
        while self.is_active and self.vehicle and self.vehicle.is_alive:
            try:
                v = self.vehicle.get_velocity()
                speed = math.sqrt(v.x**2 + v.y**2 + v.z**2)
                if speed > 2.0:
                    q = 0.5 * self.air_density * (speed ** 2)
                    f_down = q * self.ui.get('cl', 0.0) * self.ui.get('area', 2.2)
                    t_pitch = q * self.ui.get('cm', 0.0) * self.ui.get('area', 2.2)
                    f_side = 0.0
                    transform = self.vehicle.get_transform()
                    right = transform.get_right_vector()
                    up = transform.get_up_vector()
                    self.vehicle.add_force(carla.Vector3D(
                        right.x * f_side - up.x * f_down,
                        right.y * f_side - up.y * f_down,
                        right.z * f_side - up.z * f_down
                    ))
                    self.vehicle.add_torque(carla.Vector3D(
                        right.x * t_pitch,
                        right.y * t_pitch,
                        right.z * t_pitch
                    ))
            except Exception:
                pass
            time.sleep(0.01)

    def fetch_telemetry_26_items(self):
        if not self.vehicle or not self.vehicle.is_alive: return {}
        t = self.vehicle.get_transform()
        v = self.vehicle.get_velocity()
        a = self.vehicle.get_acceleration()
        w = self.vehicle.get_angular_velocity()
        ctrl = self.vehicle.get_control()
        speed_ms = math.sqrt(v.x**2 + v.y**2 + v.z**2)
        
        raw_speed_kmh = speed_ms * 3.6
        alpha = sim_state.filter_alpha
        sim_state.smoothed_speed = (alpha * raw_speed_kmh) + ((1.0 - alpha) * sim_state.smoothed_speed)
        
        steer_fl = self.vehicle.get_wheel_steer_angle(carla.VehicleWheelLocation.FL_Wheel)
        steer_fr = self.vehicle.get_wheel_steer_angle(carla.VehicleWheelLocation.FR_Wheel)
        
        base_rpm = (speed_ms / 0.35) * (60.0 / (2 * math.pi))
        slip_ratio = 1.0 + (ctrl.throttle * 0.1) 
        if speed_ms < 0.1 and ctrl.throttle > 0: slip_ratio = 5.0 
        wheel_rpm = [base_rpm * slip_ratio, base_rpm * slip_ratio, base_rpm, base_rpm]
        
        bounce_z = [0.0, 0.0, 0.0, 0.0]
        try:
            if hasattr(self.vehicle, 'get_bones'):
                for bone in self.vehicle.get_bones():
                    b_name = bone.name.lower()
                    if 'wheel_fl' in b_name: bounce_z[0] = round(bone.world_transform.location.z * 1000, 1)
                    elif 'wheel_fr' in b_name: bounce_z[1] = round(bone.world_transform.location.z * 1000, 1)
                    elif 'wheel_rl' in b_name: bounce_z[2] = round(bone.world_transform.location.z * 1000, 1)
                    elif 'wheel_rr' in b_name: bounce_z[3] = round(bone.world_transform.location.z * 1000, 1)
        except: pass
        
        light_enum = str(self.vehicle.get_light_state())
        tl_state = str(self.vehicle.get_traffic_light_state())

        mass = float(self.ui.get('mass', 1500.0))
        dyn_Cf = float(self.ui.get('cf', -110000.0 * (mass / 1500.0)))
        dyn_Cr = float(self.ui.get('cr', -95000.0 * (mass / 1500.0)))
        geom = {
            "wheelbase": float(self.ui.get('wheelbase', 2.88) or 2.88),
            "a": float(self.ui.get('a', 1.49) or 1.49),
            "b": float(self.ui.get('b', 1.39) or 1.39),
        }
        
        return {
            "1_刚体运动学 (Rigid Body Kinematics)": {
                "1_全局绝对坐标_XYZ_米": [round(t.location.x, 3), round(t.location.y, 3), round(t.location.z, 3)],
                "2_姿态角_俯仰_偏航_滚转_度": [round(t.rotation.pitch, 3), round(t.rotation.yaw, 3), round(t.rotation.roll, 3)],
                "3_线速度矢量_XYZ_米每秒": [round(v.x, 3), round(v.y, 3), round(v.z, 3)],
                "4_线加速度_XYZ_米每平方秒": [round(a.x, 3), round(a.y, 3), round(a.z, 3)],
                "5_角速度_XYZ_度每秒": [round(w.x, 3), round(w.y, 3), round(w.z, 3)]
            },
            "2_轮端与底盘动态 (Wheel Dynamics)": {
                "6_四轮独立转速_RPM_左前_右前_左后_右后": [round(x, 1) for x in wheel_rpm],
                "7_悬架实时压缩量_毫米_左前_右前_左后_右后": bounce_z, 
                "8_前轮真实阿克曼转向角_度": [round(steer_fl, 2), round(steer_fr, 2)]
            },
            "3_驾驶控制反读 (Control State)": {
                "9_实际油门开度_0至1": round(ctrl.throttle, 3),
                "10_实际刹车力度_0至1": round(ctrl.brake, 3),
                "11_方向盘转角_负1至1": round(ctrl.steer, 3),
                "12_当前机械档位": ctrl.gear,
                "13_手刹激活状态": ctrl.hand_brake,
                "14_倒车挂档状态": ctrl.reverse
            },
            "5_环境与交通真值 (Environment Truth)": {
                "19_当前路段法定限速_公里每小时": round(self.vehicle.get_speed_limit(), 1),
                "20_前方红绿灯当前状态": tl_state,
                "21_是否处于红绿灯管制区": self.vehicle.is_at_traffic_light(),
                "22_车辆灯光激活状态_位掩码": light_enum
            },
            "6_动态车辆参数": {
                "整备质量": mass,
                "前轮侧偏刚度_Cf": dyn_Cf,
                "后轮侧偏刚度_Cr": dyn_Cr,
                "轮距_L": geom["wheelbase"],
                "轴距_L": geom["wheelbase"],
                "a": geom["a"],
                "b": geom["b"],
                "Cf": dyn_Cf,
                "Cr": dyn_Cr,
                "最大前轮转角_deg": float(self.ui.get('steer', 40.0) or 40.0),
            }
        }

    def destroy(self):
        self.is_active = False
        if hasattr(self, 'aero_thread'):
            self.aero_thread.join(timeout=1.0)

# ==========================================
# 3. ✨ 四大传感器回调与环境生成
# ==========================================
def gnss_callback(data):
    sim_state.data["GNSS_DATA"] = [round(data.latitude, 5), round(data.longitude, 5), round(data.altitude, 2)]

def imu_callback(data):
    sim_state.data["IMU_DATA"] = {
        "Accel_XYZ": [round(data.accelerometer.x, 2), round(data.accelerometer.y, 2), round(data.accelerometer.z, 2)],
        "Gyro_XYZ": [round(data.gyroscope.x, 2), round(data.gyroscope.y, 2), round(data.gyroscope.z, 2)],
        "Compass": round(math.degrees(data.compass), 2)
    }

def radar_callback(data): sim_state.data["RADAR_TARGETS"] = len(data)

def collision_callback(data): 
    sim_state.data["COLLISION_DATA"] = {
        "Impulse": [round(data.normal_impulse.x, 1), round(data.normal_impulse.y, 1), round(data.normal_impulse.z, 1)], 
        "Actor": str(data.other_actor.type_id)
    }


def cleanup_scene_elements(world):
    if world is None: return
    ids = sim_state.scene_walker_controllers + sim_state.scene_actors
    for actor_id in ids:
        actor = world.get_actor(actor_id)
        if actor is not None:
            try: actor.destroy()
            except Exception: pass
    sim_state.scene_actors = []
    sim_state.scene_walker_controllers = []
    sim_state.scene_summary = None


def terminate_process_tree(proc, timeout=2.0):
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
            try:
                proc.kill()
            except Exception:
                pass
    except Exception:
        pass


def stop_manual_bridge():
    sim_state.manual_bridge_watchdog_stop.set()
    if sim_state.manual_bridge_watchdog_thread and sim_state.manual_bridge_watchdog_thread.is_alive():
        try:
            sim_state.manual_bridge_watchdog_thread.join(timeout=1.0)
        except Exception:
            pass
    sim_state.manual_bridge_watchdog_thread = None
    terminate_process_tree(sim_state.manual_bridge_process)
    sim_state.manual_bridge_process = None
    if sim_state.manual_bridge_log_file:
        try:
            sim_state.manual_bridge_log_file.close()
        except Exception:
            pass
    sim_state.manual_bridge_log_file = None
    sim_state.last_manual_control_time = 0.0
    sim_state.last_manual_control_cmd = None


def get_manual_bridge_status():
    proc = getattr(sim_state, "manual_bridge_process", None)
    running = bool(proc and proc.poll() is None)
    age = None
    if sim_state.last_manual_control_time:
        age = round(time.time() - sim_state.last_manual_control_time, 3)
    bridge_file_status = None
    try:
        status_path = Path(MANUAL_BRIDGE_STATUS_FILE)
        if status_path.exists():
            bridge_file_status = json.loads(status_path.read_text(encoding="utf-8"))
    except Exception:
        bridge_file_status = None
    return {
        "running": running,
        "pid": proc.pid if running else None,
        "restart_count": sim_state.manual_bridge_restart_count,
        "log_path": str(sim_state.manual_bridge_log_path) if sim_state.manual_bridge_log_path else None,
        "last_control_age_sec": age,
        "last_control": sim_state.last_manual_control_cmd,
        "bridge_file_status": bridge_file_status,
    }


def start_manual_bridge_process(reason="manual mode"):
    script_path = Path(MANUAL_BRIDGE_SCRIPT)
    if not script_path.is_absolute():
        script_path = Path(os.getcwd()) / script_path
    if not script_path.exists():
        print(f"manual bridge script not found: {script_path}")
        return False

    proc = sim_state.manual_bridge_process
    if proc and proc.poll() is None:
        return True

    now = time.time()
    if sim_state.manual_bridge_last_start and now - sim_state.manual_bridge_last_start < MANUAL_BRIDGE_RESTART_MIN_SEC:
        return False

    try:
        if sim_state.manual_bridge_log_file:
            try:
                sim_state.manual_bridge_log_file.close()
            except Exception:
                pass
            sim_state.manual_bridge_log_file = None

        log_dir = Path("logs")
        log_dir.mkdir(exist_ok=True)
        log_path = log_dir / f"can_wheel_bridge_{int(now)}.log"
        log_file = open(log_path, "a", encoding="utf-8")
        popen_kwargs = {"cwd": os.getcwd(), "stdout": log_file, "stderr": subprocess.STDOUT}
        if os.name != "nt":
            popen_kwargs["preexec_fn"] = os.setsid
        proc = subprocess.Popen([sys.executable, str(script_path)], **popen_kwargs)
        sim_state.manual_bridge_process = proc
        sim_state.manual_bridge_log_file = log_file
        sim_state.manual_bridge_log_path = log_path
        sim_state.manual_bridge_last_start = now
        sim_state.manual_bridge_restart_count += 1
        print(f"manual CAN bridge started for {reason}: pid={proc.pid}, log={log_path}")
        return True
    except Exception as e:
        print(f"manual CAN bridge start failed: {e}")
        return False


def start_manual_bridge_watchdog():
    if sim_state.manual_bridge_watchdog_thread and sim_state.manual_bridge_watchdog_thread.is_alive():
        return
    sim_state.manual_bridge_watchdog_stop = threading.Event()

    def watchdog_loop():
        while not sim_state.manual_bridge_watchdog_stop.is_set():
            if is_manual_mode(sim_state.drive_mode) and sim_state.vehicle and sim_state.vehicle.is_alive:
                proc = sim_state.manual_bridge_process
                if not proc or proc.poll() is not None:
                    start_manual_bridge_process("watchdog restart")
            time.sleep(MANUAL_BRIDGE_WATCHDOG_INTERVAL_SEC)

    thread = threading.Thread(target=watchdog_loop, daemon=True)
    thread.start()
    sim_state.manual_bridge_watchdog_thread = thread


def cleanup_simulation():
    stop_manual_bridge()
    algo_process = sim_state.algo_process
    if algo_process:
        try:
            if os.name != "nt":
                try:
                    os.killpg(os.getpgid(algo_process.pid), signal.SIGTERM)
                except Exception:
                    algo_process.terminate()
            else:
                algo_process.terminate()
            try:
                algo_process.wait(timeout=2.0)
            except Exception:
                pass
        except Exception: pass
        sim_state.algo_process = None
    if sim_state.algo_log_file:
        try: sim_state.algo_log_file.close()
        except Exception: pass
        sim_state.algo_log_file = None

    stop_event = sim_state.stop_event
    stop_event.set() 
    master_thread = sim_state.master_thread
    if master_thread is not None:
        master_thread.join(timeout=1.0)
    sim_state.master_thread = None
    dynamics_wrapper = sim_state.dynamics_wrapper
    if dynamics_wrapper:
        dynamics_wrapper.destroy()
    sim_state.dynamics_wrapper = None
    active_sensors = sim_state.active_sensors or []
    for s in active_sensors:
        if s and s.is_alive:
            try: s.destroy()
            except: pass
    sim_state.active_sensors = []
    vehicle = sim_state.vehicle
    if vehicle and vehicle.is_alive:
        try: vehicle.destroy()
        except: pass
    sim_state.vehicle = None
    sim_state.reset()

def cleanup_all():
    cleanup_simulation()
    world = sim_state.world
    if world:
        cleanup_scene_elements(world)


def runtime_cleanup_simulation():
    stop_manual_bridge()
    if sim_state.algo_process:
        try:
            if os.name != "nt":
                try:
                    os.killpg(os.getpgid(sim_state.algo_process.pid), signal.SIGTERM)
                except Exception:
                    sim_state.algo_process.terminate()
            else:
                sim_state.algo_process.terminate()
            try:
                sim_state.algo_process.wait(timeout=2.0)
            except Exception:
                pass
        except Exception:
            pass
        sim_state.algo_process = None
    if sim_state.algo_log_file:
        try: sim_state.algo_log_file.close()
        except Exception: pass
        sim_state.algo_log_file = None

    if sim_state.stop_event:
        sim_state.stop_event.set()
    if sim_state.master_thread:
        sim_state.master_thread.join(timeout=1.0)
        sim_state.master_thread = None
    if sim_state.dynamics_wrapper:
        sim_state.dynamics_wrapper.destroy()
        sim_state.dynamics_wrapper = None
    for sensor in sim_state.active_sensors or []:
        if sensor and sensor.is_alive:
            try: sensor.destroy()
            except Exception: pass
    sim_state.active_sensors = []
    if sim_state.vehicle and sim_state.vehicle.is_alive:
        try: sim_state.vehicle.destroy()
        except Exception: pass
    sim_state.vehicle = None
    sim_state.reset()


def runtime_cleanup_all():
    runtime_cleanup_simulation()
    if sim_state.world:
        try:
            cleanup_scene_elements(sim_state.world)
        except Exception as e:
            print(f"cleanup scene elements skipped: {e}")


def mark_runtime_stale_after_carla_disconnect(reason):
    try:
        runtime_cleanup_simulation()
    except Exception as e:
        print(f"runtime cleanup after CARLA disconnect skipped: {e}")
    sim_state.client = None
    sim_state.world = None
    sim_state.scene_actors = []
    sim_state.scene_walker_controllers = []
    sim_state.scene_summary = None
    sim_state.set_carla_status(False, f"Carla reconnect pending: {reason}")
    sim_state.set_last_pipeline_result(False, f"runtime cleared after CARLA disconnect: {reason}")

def ordered_unique(items: List[str]) -> List[str]:
    seen = set(); out = []
    for x in items:
        if x not in seen: seen.add(x); out.append(x)
    return out

def actor_to_dict(actor: carla.Actor) -> Dict:
    tf = actor.get_transform()
    return {"id": actor.id, "type_id": actor.type_id, "transform": {"x": round(tf.location.x, 3), "y": round(tf.location.y, 3), "z": round(tf.location.z, 3), "pitch": round(tf.rotation.pitch, 3), "yaw": round(tf.rotation.yaw, 3), "roll": round(tf.rotation.roll, 3)}}

def offset_transform(base_tf: carla.Transform, forward=0.0, right=0.0, up=0.1, yaw_bias=0.0) -> carla.Transform:
    yaw = math.radians(base_tf.rotation.yaw)
    fx, fy = math.cos(yaw), math.sin(yaw)
    rx, ry = -math.sin(yaw), math.cos(yaw)
    loc = carla.Location(x=base_tf.location.x + fx * forward + rx * right, y=base_tf.location.y + fy * forward + ry * right, z=base_tf.location.z + up)
    rot = carla.Rotation(pitch=base_tf.rotation.pitch, yaw=base_tf.rotation.yaw + yaw_bias, roll=base_tf.rotation.roll)
    return carla.Transform(loc, rot)

def get_all_bp_ids(bp_lib: carla.BlueprintLibrary) -> List[str]: return [bp.id for bp in bp_lib.filter("*")]

def resolve_catalog(bp_lib: carla.BlueprintLibrary) -> Dict[str, List[str]]:
    all_ids = set(get_all_bp_ids(bp_lib))
    vehicle_ids = [bp.id for bp in bp_lib.filter("vehicle.*")]
    walker_ids = [bp.id for bp in bp_lib.filter("walker.pedestrian.*")]
    controller_ids = [bp.id for bp in bp_lib.filter("controller.ai.walker")]

    traffic_pref = ["traffic.speed_limit.30", "traffic.speed_limit.40", "traffic.speed_limit.50", "traffic.speed_limit.60", "traffic.speed_limit.90", "traffic.stop", "traffic.yield", "static.prop.trafficwarning", "static.prop.warningaccident", "static.prop.warningconstruction", "static.prop.streetbarrier", "static.prop.constructioncone", "static.prop.chainbarrier", "static.prop.chainbarrierend", "static.prop.trafficcone01", "static.prop.trafficcone02"]
    traffic_standards = ordered_unique([x for x in traffic_pref if x in all_ids])

    traffic_fill_pool = ["static.prop.ironplank", "static.prop.brokentile01", "static.prop.brokentile02", "static.prop.brokentile03", "static.prop.brokentile04", "static.prop.dirtdebris01", "static.prop.dirtdebris02", "static.prop.dirtdebris03"]
    for x in traffic_fill_pool:
        if x in all_ids and x not in traffic_standards: traffic_standards.append(x)
        if len(traffic_standards) >= REQUIRED_COUNTS["traffic_standards"]: break

    barrier_pref = ["static.prop.streetbarrier", "static.prop.constructioncone", "static.prop.trafficcone01", "static.prop.trafficcone02", "static.prop.chainbarrier", "static.prop.chainbarrierend", "static.prop.warningconstruction", "static.prop.trafficwarning", "static.prop.warningaccident", "static.prop.ironplank", "static.prop.brokentile01", "static.prop.brokentile02", "static.prop.brokentile03", "static.prop.brokentile04", "static.prop.dirtdebris01", "static.prop.dirtdebris02", "static.prop.dirtdebris03"]
    barriers = ordered_unique([x for x in barrier_pref if x in all_ids])

    bicycle_pref = ["vehicle.bh.crossbike", "vehicle.diamondback.century", "vehicle.gazelle.omafiets"]
    bicycles = [x for x in bicycle_pref if x in all_ids]

    emergency_keywords = ["firetruck", "ambulance", "police"]
    emergency_vehicles = [x for x in vehicle_ids if any(k in x.lower() for k in emergency_keywords)]

    moto_keywords = ["yamaha", "harley", "vespa", "kawasaki", "bike"]
    normal_vehicles = [x for x in vehicle_ids if x not in bicycles and x not in emergency_vehicles and not any(k in x.lower() for k in moto_keywords)]

    animals = [x for x in get_all_bp_ids(bp_lib) if (any(k in x.lower() for k in ["animal", "deer", "horse"]) or x.lower().endswith(".dog") or x.lower().endswith(".cat")) and "doghouse" not in x.lower()]

    vehicle_models = ordered_unique(normal_vehicles + emergency_vehicles)

    return {
        "vehicle_models": vehicle_models, "traffic_standards": traffic_standards, "barriers": barriers,
        "normal_vehicles": normal_vehicles, "emergency_vehicles": emergency_vehicles, "walkers": walker_ids,
        "walker_controllers": controller_ids, "bicycles": bicycles, "animals": animals
    }

def summarize_vehicle_models(catalog: Dict[str, List[str]], normal_vehicle_result: Dict, emergency_result: Dict) -> Dict:
    spawned_model_ids = ordered_unique([a["type_id"] for a in normal_vehicle_result.get("actors", [])] + [a["type_id"] for a in emergency_result.get("actors", [])])
    return {"requested": REQUIRED_COUNTS["vehicle_models"], "available_blueprints": len(catalog.get("vehicle_models", [])), "spawned_unique_models": len(spawned_model_ids), "model_type_ids": spawned_model_ids, "satisfied": len(spawned_model_ids) >= REQUIRED_COUNTS["vehicle_models"]}

def spawn_static_objects(world: carla.World, bp_lib: carla.BlueprintLibrary, spawn_points: List[carla.Transform], bp_ids: List[str], desired_count: int, record_ids: List[int], right_bias: float) -> Dict:
    created, used, warnings = [], [], []
    if not bp_ids: return {"requested": desired_count, "available_blueprints": 0, "spawned": 0, "actors": [], "blueprints_used": [], "warnings": ["no available blueprint"]}
    random.shuffle(spawn_points)
    selected = bp_ids[:desired_count]
    for i, bp_id in enumerate(selected):
        if not spawn_points: break
        base_tf = spawn_points[i % len(spawn_points)]
        tf = offset_transform(base_tf, forward=(i % 4) * 1.5, right=right_bias + (i % 3) * 1.2, up=0.1, yaw_bias=90.0)
        bp = bp_lib.find(bp_id)
        actor = world.try_spawn_actor(bp, tf)
        if actor is None:
            warnings.append(f"spawn failed: {bp_id}")
            continue
        try: actor.set_simulate_physics(False)
        except: pass
        record_ids.append(actor.id)
        used.append(bp_id)
        created.append(actor_to_dict(actor))
    return {"requested": desired_count, "available_blueprints": len(bp_ids), "spawned": len(created), "actors": created, "blueprints_used": used, "warnings": warnings}

def spawn_vehicle_group(world: carla.World, bp_lib: carla.BlueprintLibrary, bp_ids: List[str], desired_count: int, spawn_points: List[carla.Transform], tm_port: int, role_name: str, record_ids: List[int]) -> Dict:
    created, used, warnings = [], [], []
    if not bp_ids: return {"requested": desired_count, "available_blueprints": 0, "spawned": 0, "actors": [], "blueprints_used": [], "warnings": ["no available blueprint"]}
    random.shuffle(spawn_points)
    attempts, max_attempts = 0, max(len(spawn_points) * 2, desired_count * 5)
    while len(created) < desired_count and attempts < max_attempts:
        bp_id = bp_ids[attempts % len(bp_ids)]
        tf = spawn_points[attempts % len(spawn_points)]
        bp = bp_lib.find(bp_id)
        if bp.has_attribute("role_name"): bp.set_attribute("role_name", role_name)
        if bp.has_attribute("color"):
            vals = bp.get_attribute("color").recommended_values
            if vals: bp.set_attribute("color", random.choice(vals))
        actor = world.try_spawn_actor(bp, tf)
        attempts += 1
        if actor is None: continue
        actor.set_autopilot(True, tm_port)
        try: world.wait_for_tick()
        except: time.sleep(0.05)
        record_ids.append(actor.id)
        used.append(bp_id)
        created.append(actor_to_dict(actor))
    if len(created) < desired_count: warnings.append(f"only spawned {len(created)}/{desired_count}")
    return {"requested": desired_count, "available_blueprints": len(bp_ids), "spawned": len(created), "actors": created, "blueprints_used": used, "warnings": warnings}

def spawn_bicycles_distinct(world: carla.World, bp_lib: carla.BlueprintLibrary, bicycle_bp_ids: List[str], desired_count: int, spawn_points: List[carla.Transform], tm_port: int, record_ids: List[int]) -> Dict:
    created, used, warnings = [], [], []
    if not bicycle_bp_ids: return {"requested": desired_count, "available_blueprints": 0, "spawned": 0, "actors": [], "blueprints_used": [], "warnings": ["no bicycle blueprint"]}
    target_unique = min(desired_count, len(bicycle_bp_ids))
    random.shuffle(spawn_points)
    for bp_id in bicycle_bp_ids:
        if len(created) >= target_unique: break
        bp = bp_lib.find(bp_id)
        if bp.has_attribute("role_name"): bp.set_attribute("role_name", "bicycle")
        spawned = False
        for tf in spawn_points:
            actor = world.try_spawn_actor(bp, tf)
            if actor is None: continue
            actor.set_autopilot(True, tm_port)
            try: world.wait_for_tick()
            except: time.sleep(0.05)
            record_ids.append(actor.id)
            used.append(bp_id)
            created.append(actor_to_dict(actor))
            spawned = True
            break
        if not spawned: warnings.append(f"spawn failed for bicycle blueprint: {bp_id}")
    if len(created) < desired_count: warnings.append(f"only spawned {len(created)}/{desired_count} bicycles")
    return {"requested": desired_count, "available_blueprints": len(bicycle_bp_ids), "spawned": len(created), "actors": created, "blueprints_used": used, "warnings": warnings}

def spawn_vehicle_model_fillers(world: carla.World, bp_lib: carla.BlueprintLibrary, all_vehicle_model_bp_ids: List[str], existing_model_ids: List[str], desired_unique_count: int, spawn_points: List[carla.Transform], record_ids: List[int]) -> Dict:
    created, used, warnings = [], [], []
    missing_count = desired_unique_count - len(existing_model_ids)
    if missing_count <= 0: return {"requested": desired_unique_count, "spawned": 0, "actors": [], "blueprints_used": [], "warnings": []}
    candidates = [x for x in all_vehicle_model_bp_ids if x not in existing_model_ids]
    random.shuffle(spawn_points)
    attempts, max_attempts = 0, max(len(spawn_points) * 2, missing_count * 5)
    while len(used) < missing_count and attempts < max_attempts and candidates:
        bp_id = candidates.pop(0)
        tf = spawn_points[attempts % len(spawn_points)]
        attempts += 1
        bp = bp_lib.find(bp_id)
        if bp.has_attribute("role_name"): bp.set_attribute("role_name", "vehicle_model_fill")
        actor = world.try_spawn_actor(bp, tf)
        if actor is None: continue
        try: world.wait_for_tick()
        except: time.sleep(0.05)
        record_ids.append(actor.id)
        used.append(bp_id)
        created.append(actor_to_dict(actor))
    if len(used) < missing_count: warnings.append(f"only filled {len(used)}/{missing_count} extra vehicle models")
    return {"requested": desired_unique_count, "spawned": len(used), "actors": created, "blueprints_used": used, "warnings": warnings}

def spawn_walkers(world: carla.World, bp_lib: carla.BlueprintLibrary, walker_bp_ids: List[str], controller_bp_ids: List[str], desired_count: int, record_actor_ids: List[int], record_controller_ids: List[int]) -> Dict:
    created, used, warnings = [], [], []
    if not walker_bp_ids: return {"requested": desired_count, "available_blueprints": 0, "spawned": 0, "actors": [], "blueprints_used": [], "warnings": ["no pedestrian blueprints"]}
    if not controller_bp_ids: return {"requested": desired_count, "available_blueprints": len(walker_bp_ids), "spawned": 0, "actors": [], "blueprints_used": [], "warnings": ["no walker controller blueprint"]}
    controller_bp = bp_lib.find(controller_bp_ids[0])
    attempts, max_attempts = 0, desired_count * 10
    while len(created) < desired_count and attempts < max_attempts:
        bp_id = random.choice(walker_bp_ids)
        nav_loc = world.get_random_location_from_navigation()
        attempts += 1
        if nav_loc is None: continue
        walker_tf = carla.Transform(nav_loc)
        walker_bp = bp_lib.find(bp_id)
        walker = world.try_spawn_actor(walker_bp, walker_tf)
        if walker is None: continue
        controller = world.try_spawn_actor(controller_bp, carla.Transform(), walker)
        if controller is None:
            walker.destroy()
            continue
        controller.start()
        target = world.get_random_location_from_navigation()
        if target: controller.go_to_location(target)
        controller.set_max_speed(1.2 + random.random())
        try: world.wait_for_tick()
        except: time.sleep(0.05)
        record_actor_ids.append(walker.id)
        record_controller_ids.append(controller.id)
        used.append(bp_id)
        created.append(actor_to_dict(walker))
    if len(created) < desired_count: warnings.append(f"only spawned {len(created)}/{desired_count}")
    return {"requested": desired_count, "available_blueprints": len(walker_bp_ids), "spawned": len(created), "actors": created, "blueprints_used": used, "warnings": warnings}

def spawn_animal(world: carla.World, bp_lib: carla.BlueprintLibrary, animal_bp_ids: List[str], spawn_points: List[carla.Transform], record_ids: List[int]) -> Dict:
    animal_bp_ids = [x for x in animal_bp_ids if "doghouse" not in x.lower()]
    if not animal_bp_ids: return {"requested": 1, "available_blueprints": 0, "spawned": 0, "actors": [], "blueprints_used": [], "warnings": ["no real animal blueprint in current CARLA installation"]}
    bp_id = animal_bp_ids[0]
    base_tf = random.choice(spawn_points)
    tf = offset_transform(base_tf, forward=2.0, right=6.0, up=0.1)
    bp = bp_lib.find(bp_id)
    actor = world.try_spawn_actor(bp, tf)
    if actor is None: return {"requested": 1, "available_blueprints": len(animal_bp_ids), "spawned": 0, "actors": [], "blueprints_used": [], "warnings": [f"spawn failed: {bp_id}"]}
    try:
        actor.set_simulate_physics(False)
        world.wait_for_tick()
    except: time.sleep(0.05)
    record_ids.append(actor.id)
    return {"requested": 1, "available_blueprints": len(animal_bp_ids), "spawned": 1, "actors": [actor_to_dict(actor)], "blueprints_used": [bp_id], "warnings": []}

def build_route_anchor_locations(scene_name: str) -> List[carla.Location]:
    anchors = []
    for cfg_name, cfg in SCENARIO_DATABASE.items():
        if cfg_name != scene_name:
            continue
        pos = cfg.get("pos")
        if pos:
            anchors.append(carla.Location(x=pos[0], y=pos[1], z=pos[2]))
    if anchors:
        return anchors
    return [carla.Location(x=cfg["pos"][0], y=cfg["pos"][1], z=cfg["pos"][2]) for cfg in SCENARIO_DATABASE.values()]

def select_hidden_spawn_points(spawn_points: List[carla.Transform], scene_name: str, min_distance: float) -> List[carla.Transform]:
    anchors = build_route_anchor_locations(scene_name)
    if not spawn_points:
        return []
    ranked = sorted(
        spawn_points,
        key=lambda sp: min(sp.location.distance(anchor) for anchor in anchors),
        reverse=True,
    )
    hidden_points = [sp for sp in ranked if min(sp.location.distance(anchor) for anchor in anchors) >= min_distance]
    return hidden_points or ranked

def add_scene_elements_to_current_map(client, world, scene_name=None, traffic_mode="full"):
    if client is None or world is None: return
    cleanup_scene_elements(world)
    bp_lib = world.get_blueprint_library()
    raw_spawn_points = world.get_map().get_spawn_points()
    if not raw_spawn_points: return

    # 15米防爆结界
    safe_spawn_points = []
    fixed_locs = [carla.Location(x=cfg["pos"][0], y=cfg["pos"][1], z=cfg["pos"][2]) for cfg in SCENARIO_DATABASE.values()]
    for sp in raw_spawn_points:
        if all(sp.location.distance(loc) > 15.0 for loc in fixed_locs):
            safe_spawn_points.append(sp)
    if not safe_spawn_points: safe_spawn_points = raw_spawn_points

    try: world.wait_for_tick()
    except: time.sleep(0.1)

    catalog = resolve_catalog(bp_lib)
    tm = client.get_trafficmanager(TM_PORT)
    tm.set_global_distance_to_leading_vehicle(2.5)

    actor_ids, controller_ids = [], []
    dynamic_spawn_points = safe_spawn_points.copy()
    static_spawn_points = safe_spawn_points.copy()
    if traffic_mode == "hidden":
        dynamic_spawn_points = select_hidden_spawn_points(safe_spawn_points.copy(), scene_name or "", HIDDEN_TRAFFIC_MIN_DISTANCE_M)
        static_spawn_points = dynamic_spawn_points.copy()

    traffic_result = spawn_static_objects(world, bp_lib, static_spawn_points.copy(), catalog["traffic_standards"], REQUIRED_COUNTS["traffic_standards"], actor_ids, right_bias=5.0)
    barrier_result = spawn_static_objects(world, bp_lib, static_spawn_points.copy(), catalog["barriers"], REQUIRED_COUNTS["barriers"], actor_ids, right_bias=7.0)
    normal_vehicle_result = spawn_vehicle_group(world, bp_lib, catalog["normal_vehicles"], REQUIRED_COUNTS["normal_vehicles"], dynamic_spawn_points.copy(), TM_PORT, "opponent", actor_ids)
    emergency_result = spawn_vehicle_group(world, bp_lib, catalog["emergency_vehicles"], REQUIRED_COUNTS["emergency_vehicles"], dynamic_spawn_points.copy(), TM_PORT, "emergency", actor_ids)
    bicycle_result = spawn_bicycles_distinct(world, bp_lib, catalog["bicycles"], REQUIRED_COUNTS["bicycles"], dynamic_spawn_points.copy(), TM_PORT, actor_ids)
    walker_result = spawn_walkers(world, bp_lib, catalog["walkers"], catalog["walker_controllers"], REQUIRED_COUNTS["walkers"], actor_ids, controller_ids)
    animal_result = spawn_animal(world, bp_lib, catalog["animals"], dynamic_spawn_points.copy(), actor_ids) if REQUIRED_COUNTS["animals"] > 0 else {"requested": 0, "available_blueprints": len(catalog["animals"]), "spawned": 0, "actors": [], "blueprints_used": [], "warnings": []}

    spawned_model_ids = ordered_unique([a["type_id"] for a in normal_vehicle_result.get("actors", [])] + [a["type_id"] for a in emergency_result.get("actors", [])])
    vehicle_model_fill_result = spawn_vehicle_model_fillers(world, bp_lib, catalog["vehicle_models"], spawned_model_ids, REQUIRED_COUNTS["vehicle_models"], dynamic_spawn_points.copy(), actor_ids)
    vehicle_models_result = summarize_vehicle_models(catalog, {"actors": normal_vehicle_result.get("actors", []) + vehicle_model_fill_result.get("actors", [])}, emergency_result)

    sim_state.scene_actors = actor_ids
    sim_state.scene_walker_controllers = controller_ids
    sim_state.scene_summary = {
        "traffic_mode": traffic_mode,
        "hidden_min_distance_m": HIDDEN_TRAFFIC_MIN_DISTANCE_M if traffic_mode == "hidden" else 0.0,
        "vehicle_models": vehicle_models_result,
        "traffic_standards": traffic_result,
        "barriers": barrier_result,
        "normal_vehicles": normal_vehicle_result,
        "emergency_vehicles": emergency_result,
        "walkers": walker_result,
        "bicycles": bicycle_result,
        "animals": animal_result,
        "runtime_actor_count": len(actor_ids),
        "runtime_walker_controller_count": len(controller_ids),
    }


def set_traffic_counts(mode):
    global REQUIRED_COUNTS
    if mode == "full" or mode is True:
        REQUIRED_COUNTS = DEFAULT_REQUIRED_COUNTS.copy()
    elif mode == "hidden":
        REQUIRED_COUNTS = HIDDEN_TRAFFIC_COUNTS.copy()
    else:
        REQUIRED_COUNTS = NO_TRAFFIC_COUNTS.copy()


def apply_custom_weather(world, w_cond, t_day):
    w = carla.WeatherParameters()
    w.cloudiness = 0.0
    w.precipitation = 0.0
    w.precipitation_deposits = 0.0
    w.wind_intensity = 0.0
    w.fog_density = 0.0
    w.fog_distance = 0.0

    if "多云" in w_cond:
        w.cloudiness = 80.0
    elif "小雨" in w_cond:
        w.cloudiness = 80.0
        w.precipitation = 30.0
        w.precipitation_deposits = 30.0
    elif "暴雨" in w_cond:
        w.cloudiness = 100.0
        w.precipitation = 90.0
        w.precipitation_deposits = 90.0
        w.wind_intensity = 80.0
    elif "大雾" in w_cond:
        w.cloudiness = 50.0
        w.fog_density = 50.0
        w.fog_distance = 10.0

    if "正午" in t_day:
        w.sun_altitude_angle = 75.0
        w.sun_azimuth_angle = 180.0
    elif "夕阳" in t_day:
        w.sun_altitude_angle = 5.0
        w.sun_azimuth_angle = 180.0
    elif "深夜" in t_day:
        w.sun_altitude_angle = -90.0
        w.sun_azimuth_angle = 0.0

    world.set_weather(w)


def resolve_runtime_map(client, scene_name):
    scene_cfg = SCENARIO_DATABASE.get(scene_name, {})
    base_map = scene_cfg.get("runtime_map", scene_name)
    if not scene_cfg.get("prefer_opt"):
        return base_map
    opt_map = f"{base_map}_Opt"
    try:
        available = [m.split("/")[-1] for m in client.get_available_maps()]
        if opt_map in available:
            return opt_map
    except Exception as e:
        print(f"available map query failed, fallback to {base_map}: {e}")
    return base_map


def activate_scene_variant(world, scene_name):
    scene_cfg = SCENARIO_DATABASE.get(scene_name, {})
    if not scene_cfg.get("unload_buildings"):
        return
    try:
        world.unload_map_layer(carla.MapLayer.Buildings)
        print(f"{scene_name}: Buildings layer hidden; road and foliage preserved.")
    except Exception as e:
        print(f"{scene_name}: unable to hide Buildings layer on current map: {e}")


def ensure_carla_connection():
    try:
        if sim_state.client is not None and sim_state.world is not None:
            try:
                sim_state.client.get_world()
                sim_state.set_carla_status(True, f"Carla connected: {CARLA_HOST}:{CARLA_PORT}", sim_state.world.get_map().name.split("/")[-1])
                return True, "Carla already connected"
            except Exception:
                sim_state.client = None
                sim_state.world = None

        client = carla.Client(CARLA_HOST, CARLA_PORT)
        client.set_timeout(CARLA_CONNECT_TIMEOUT)
        world = client.get_world()
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = float(os.environ.get("OFFICIAL_FIXED_DELTA_SECONDS", "0.02"))
        settings.substepping = True
        settings.max_substep_delta_time = float(os.environ.get("OFFICIAL_MAX_SUBSTEP_DELTA_TIME", "0.005"))
        settings.max_substeps = int(os.environ.get("OFFICIAL_MAX_SUBSTEPS", "16"))
        world.apply_settings(settings)
        sim_state.client = client
        sim_state.world = world
        sim_state.set_carla_status(True, f"Carla connected: {CARLA_HOST}:{CARLA_PORT}", world.get_map().name.split("/")[-1])
        return True, "Carla connected"
    except Exception as e:
        sim_state.set_carla_status(False, f"Carla connect failed: {e}")
        return False, f"Carla connect failed: {e}"


def start_carla_auto_connector():
    if auto_connector_started.is_set():
        return
    auto_connector_started.set()

    def connector_loop():
        while True:
            if sim_state.client is None or sim_state.world is None:
                ensure_carla_connection()
            else:
                try:
                    world = sim_state.client.get_world()
                    try:
                        world.get_snapshot()
                    except Exception:
                        pass
                    if not sim_state.carla_connected:
                        sim_state.set_carla_status(True, f"Carla connected: {CARLA_HOST}:{CARLA_PORT}", sim_state.world.get_map().name.split("/")[-1])
                except Exception as e:
                    mark_runtime_stale_after_carla_disconnect(e)
            time.sleep(CARLA_AUTO_CONNECT_INTERVAL)

    threading.Thread(target=connector_loop, daemon=True).start()


def build_spawn_transform(map_base_name):
    if map_base_name not in SCENARIO_DATABASE:
        raise ValueError(f"{map_base_name} 不在 SCENARIO_DATABASE，当前只支持 Town01-Town05 和 TrainingGround 固定点演示")
    pos = SCENARIO_DATABASE[map_base_name]["pos"]
    return carla.Transform(
        carla.Location(x=pos[0], y=pos[1], z=pos[2]),
        carla.Rotation(pitch=0.0, yaw=pos[3], roll=0.0)
    )


def build_ui_params_from_vehicle_json(v_data):
    mass_props = v_data.get('weight_and_mass_properties') or {}
    cg_m = mass_props.get('center_of_gravity_m') or {}
    aero_props = v_data.get('aerodynamic_parameters') or {}
    mech_props = v_data.get('chassis_and_mechanical_systems') or {}
    steering_props = mech_props.get('steering_system') or {}
    suspension_props = mech_props.get('suspension_system') or {}
    braking_props = mech_props.get('braking_system') or {}
    powertrain_props = v_data.get('powertrain_system') or {}
    geom = vehicle_geometry_params(sim_state.current_vehicle_model)
    return {
        'mass': float(mass_props.get('curb_weight_kg', 1800.0)),
        'moi': 1.0,
        'cg_x': float(cg_m.get('x', 0.0)),
        'cg_y': float(cg_m.get('y', 0.0)),
        'cg_z': float(cg_m.get('z', 0.0)),
        'cd': float(aero_props.get('drag_coefficient_cd', 0.3)),
        'area': float(aero_props.get('frontal_area_sqm', 2.2)),
        'cl': float(aero_props.get('lift_coefficient_cl', 0.0)),
        'cy': float(aero_props.get('side_force_coefficient_cy', 0.0)),
        'cm': float(aero_props.get('pitch_moment_coefficient', 0.0)),
        'friction': 3.5,
        'steer': float(steering_props.get('wheel_max_angle_deg', 40.0)),
        'radius_mult': 1.0,
        'damping': float(suspension_props.get('damping_ratio', 0.5)),
        'susp_stiff': 500.0,
        'susp_travel': 0.15,
        'lat_stiff': 17.0,
        'long_stiff': 3000.0,
        'rpm': int(powertrain_props.get('max_rpm', 6000)),
        'brake': float(braking_props.get('max_brake_torque_nm', 1500.0)),
        'handbrake': 3000.0,
        'gear_time': 0.4,
        'clutch': 1.5,
        'final_ratio': 4.0,
        'vehicle_model': sim_state.current_vehicle_model,
        'wheelbase': geom['wheelbase'],
        'track': geom['track'],
        'tire_radius': geom['tire_radius'],
        'a': geom['a'],
        'b': geom['b'],
    }


def spawn_runtime_vehicle(v_data, map_base_name, drive_mode_str, ui_params):
    client = sim_state.client
    world = sim_state.world
    if not client or not world:
        return False, "Carla 底层未就绪"

    bp_lib = world.get_blueprint_library()
    bp_id = v_data['vehicle_metadata']['blueprint_id']
    try:
        bp = bp_lib.find(bp_id)
        if bp.has_attribute("role_name"):
            bp.set_attribute("role_name", "hero")
    except Exception:
        return False, f"车辆蓝图不存在: {bp_id}"

    spawn_p = build_spawn_transform(map_base_name)
    vehicle = world.try_spawn_actor(bp, spawn_p)
    if vehicle is None:
        try:
            world.wait_for_tick()
        except Exception:
            time.sleep(0.1)
        vehicle = world.try_spawn_actor(bp, spawn_p)
    if vehicle is None:
        return False, f"车辆生成失败: {bp_id}"

    sim_state.vehicle = vehicle
    sim_state.active_sensors = []
    try:
        vehicle.set_simulate_physics(True)
        vehicle.apply_control(carla.VehicleControl(
            throttle=0.0,
            steer=0.0,
            brake=0.0,
            hand_brake=False,
            reverse=False,
            manual_gear_shift=False,
            gear=1
        ))
    except Exception as e:
        print(f"vehicle initial control setup warning: {e}")

    radar = world.spawn_actor(bp_lib.find('sensor.other.radar'), carla.Transform(carla.Location(x=2.5, z=1.0)), attach_to=vehicle)
    radar.listen(radar_callback)
    sim_state.active_sensors.append(radar)

    col_sensor = world.spawn_actor(bp_lib.find('sensor.other.collision'), carla.Transform(), attach_to=vehicle)
    col_sensor.listen(collision_callback)
    sim_state.active_sensors.append(col_sensor)

    imu_sensor = world.spawn_actor(bp_lib.find('sensor.other.imu'), carla.Transform(), attach_to=vehicle)
    imu_sensor.listen(imu_callback)
    sim_state.active_sensors.append(imu_sensor)

    gnss_sensor = world.spawn_actor(bp_lib.find('sensor.other.gnss'), carla.Transform(), attach_to=vehicle)
    gnss_sensor.listen(gnss_callback)
    sim_state.active_sensors.append(gnss_sensor)

    wrapper = L4_DynamicsWrapper(vehicle, v_data, ui_params, world)
    sim_state.dynamics_wrapper = wrapper
    sim_state.stop_event = threading.Event()
    sim_state.stop_event.clear()

    t = threading.Thread(
        target=master_simulation_loop,
        args=(client, vehicle, sim_state.active_sensors, sim_state.stop_event, wrapper),
        daemon=True
    )
    t.start()
    sim_state.master_thread = t

    if is_algo_mode(drive_mode_str) and map_base_name in SCENARIO_DATABASE:
        script_name = SCENARIO_DATABASE[map_base_name]["script"]
        try:
            log_dir = Path("logs")
            log_dir.mkdir(exist_ok=True)
            log_path = log_dir / f"{Path(script_name).stem}_{int(time.time())}.log"
            sim_state.algo_log_file = open(log_path, "a", encoding="utf-8")
            popen_kwargs = {"cwd": os.getcwd(), "stdout": sim_state.algo_log_file, "stderr": subprocess.STDOUT}
            if os.name != "nt":
                popen_kwargs["preexec_fn"] = os.setsid
            sim_state.algo_process = subprocess.Popen([sys.executable, script_name], **popen_kwargs)
            print(f"algorithm started: {script_name} via {sys.executable}, log: {log_path}")
        except Exception as e:
            print(f"后台启动算法脚本失败: {script_name}, {e}")

    if is_manual_mode(drive_mode_str):
        if not start_manual_bridge_process("manual deployment"):
            print("manual bridge did not start during deployment; watchdog will keep retrying")
        start_manual_bridge_watchdog()

    return True, "仿真环境已全自动生成，UDP 推流激活！"


def wait_frontend_telemetry(timeout=1.5):
    end_time = time.time() + timeout
    while time.time() < end_time:
        telemetry = sim_state.snapshot()["data"].get("FULL_TELEMETRY", {})
        if telemetry and all(field in telemetry for field in FRONTEND_OUTPUT_FIELDS):
            return telemetry
        time.sleep(0.05)
    return sim_state.snapshot()["data"].get("FULL_TELEMETRY", {})


def get_vehicle_diagnostics():
    vehicle = sim_state.vehicle
    if not safe_actor_alive(vehicle):
        return {
            "actor_id": None,
            "speed_kmh": 0.0,
            "autopilot_expected": is_ai_mode(sim_state.drive_mode),
            "control": None,
            "manual_bridge": get_manual_bridge_status(),
        }
    try:
        velocity = vehicle.get_velocity()
        speed_kmh = round(math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2) * 3.6, 3)
    except Exception:
        speed_kmh = -1.0
    try:
        ctrl = vehicle.get_control()
        control = {
            "throttle": round(float(ctrl.throttle), 4),
            "steer": round(float(ctrl.steer), 4),
            "brake": round(float(ctrl.brake), 4),
            "reverse": bool(ctrl.reverse),
            "hand_brake": bool(ctrl.hand_brake),
            "manual_gear_shift": bool(ctrl.manual_gear_shift),
            "gear": int(ctrl.gear),
        }
    except Exception:
        control = None
    return {
        "actor_id": vehicle.id,
        "speed_kmh": speed_kmh,
        "autopilot_expected": is_ai_mode(sim_state.drive_mode),
        "control": control,
        "manual_bridge": get_manual_bridge_status(),
    }


def safe_actor_alive(actor):
    try:
        return bool(actor and actor.is_alive)
    except Exception:
        return False


def vehicle_ref_present():
    return sim_state.vehicle is not None


def apply_camera_view_command(body):
    ensure_center_display_started()
    requested = CarlaEncoder.first_value(
        body,
        "camera_view",
        "Camera View",
        "view",
        "View",
        "视角",
        "camera",
        default=sim_state.camera_view
    )
    view = sim_state.set_camera_view(requested)
    if not deployment_lock.locked():
        sim_state.set_pipeline_step("camera_view", f"camera view switched to {camera_view_label(view)}")
    return view


def run_frontend_pipeline(body: dict):
    ensure_center_display_started()
    if not deployment_lock.acquire(blocking=False):
        snap = sim_state.snapshot()
        last_result = snap.get("last_pipeline_result") or {}
        return False, f"deployment is running: {last_result.get('step', 'unknown')} | {last_result.get('msg', '')}"
    try:
        sim_state.set_pipeline_step("decode_command", "decoding frontend six-field command")
        scene_key = CarlaEncoder.first_value(body, "scene", "Scene", default="Town01")
        target_map = CarlaEncoder.SCENE_MAP.get(scene_key, "Town01")
        runtime_map = resolve_runtime_map(sim_state.client, target_map) if sim_state.client else target_map
        weather_key = CarlaEncoder.first_value(body, "sky", "Weather Condition", default="Sunny")
        weather_str = CarlaEncoder.WEATHER_MAP.get(weather_key, "晴天")
        time_key = CarlaEncoder.first_value(body, "sunshinetime", "Sunshine Time", default="Noon")
        time_str = CarlaEncoder.TIME_MAP.get(time_key, "正午")
        mode_key = CarlaEncoder.first_value(body, "drive_mode", "Drive Mode", default="AI")
        mode_str = CarlaEncoder.decode_mode(mode_key)
        vehicle_key = CarlaEncoder.first_value(body, "vehiclemodel", "Vehicle Model", default="Lincoln MKZ")
        vehicle_only_scene = str(vehicle_key).strip().lower() in {"none", "no vehicle", "environment only"}
        v_file = None if vehicle_only_scene else CarlaEncoder.VEHICLE_MAP.get(vehicle_key, CarlaEncoder.VEHICLE_MAP["Lincoln MKZ"])
        traffic_key = CarlaEncoder.first_value(body, "loadingtransportation", "Traffic Load", default="1")
        load_traffic = CarlaEncoder.TRAFFIC_LOAD_MAP.get(traffic_key, False)
        traffic_mode = "full" if load_traffic else "hidden"
        camera_view = normalize_camera_view(CarlaEncoder.first_value(
            body,
            "camera_view",
            "Camera View",
            "view",
            "View",
            "视角",
            "camera",
            default=sim_state.camera_view
        ))

        decoded_command = {
            "scene": target_map,
            "runtime_map": runtime_map,
            "sky": weather_str,
            "sunshinetime": time_str,
            "drive_mode": mode_str,
            "loadingtransportation": "0" if load_traffic else "1",
            "traffic_mode": traffic_mode,
            "vehiclemodel": vehicle_key,
            "vehicle_file": v_file,
            "camera_view": camera_view,
            "camera_view_label": camera_view_label(camera_view),
        }
        sim_state.set_last_command(body, decoded_command)
        sim_state.set_camera_view(camera_view)
        sim_state.target_ip = body.get("_client_ip", sim_state.target_ip)
        sim_state.current_vehicle_model = "None" if vehicle_only_scene else (vehicle_key if vehicle_key in VEHICLE_GEOMETRY_SPECS else "Lincoln MKZ")

        sim_state.set_pipeline_step("ensure_carla", f"connecting to CARLA {CARLA_HOST}:{CARLA_PORT}")
        conn_ok, conn_msg = ensure_carla_connection()
        if not conn_ok:
            sim_state.set_last_pipeline_result(False, conn_msg)
            return False, conn_msg

        sim_state.set_pipeline_step("cleanup_old_runtime", "cleaning previous simulation runtime")
        sim_state.client.set_timeout(CARLA_DEPLOY_TIMEOUT)
        runtime_cleanup_all()
        current_map = ""
        try:
            current_map = sim_state.world.get_map().name.split("/")[-1] if sim_state.world else ""
        except Exception:
            current_map = ""
        runtime_map = resolve_runtime_map(sim_state.client, target_map)
        if current_map == runtime_map:
            sim_state.set_pipeline_step("reuse_world", f"reusing loaded world {runtime_map}")
        else:
            sim_state.set_pipeline_step("load_world", f"loading world {runtime_map}")
            sim_state.world = sim_state.client.load_world(runtime_map)
        sim_state.current_world_name = target_map if target_map == runtime_map else f"{target_map} ({runtime_map})"
        settings = sim_state.world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = float(os.environ.get("OFFICIAL_FIXED_DELTA_SECONDS", "0.02"))
        settings.substepping = True
        settings.max_substep_delta_time = float(os.environ.get("OFFICIAL_MAX_SUBSTEP_DELTA_TIME", "0.005"))
        settings.max_substeps = int(os.environ.get("OFFICIAL_MAX_SUBSTEPS", "16"))
        sim_state.world.apply_settings(settings)
        activate_scene_variant(sim_state.world, target_map)
        sim_state.set_pipeline_step("apply_weather", f"applying weather={weather_str}, time={time_str}")
        apply_custom_weather(sim_state.world, weather_str, time_str)
        set_traffic_counts(traffic_mode)
        if traffic_mode == "full":
            sim_state.set_pipeline_step("load_traffic", "loading full visible traffic elements")
        else:
            sim_state.set_pipeline_step("load_hidden_traffic", "loading hidden traffic environment for no-traffic mode")
        add_scene_elements_to_current_map(sim_state.client, sim_state.world, scene_name=target_map, traffic_mode=traffic_mode)

        sim_state.drive_mode = mode_str
        if vehicle_only_scene:
            msg = "环境场景已生成：未部署车辆。"
            sim_state.set_carla_status(True, f"Carla world loaded: {target_map} ({runtime_map})", sim_state.current_world_name)
            sim_state.set_last_pipeline_result(True, msg)
            return True, msg
        sim_state.set_pipeline_step("load_vehicle_config", f"loading vehicle config {v_file}")
        v_path = os.path.join(VEHICLE_DIR, v_file)
        with open(v_path, 'r', encoding='utf-8') as f:
            v_data = json.load(f)
        ui_params = build_ui_params_from_vehicle_json(v_data)
        sim_state.set_pipeline_step("spawn_vehicle", f"spawning ego vehicle in {target_map} with mode={mode_key}")
        ok, msg = spawn_runtime_vehicle(v_data, target_map, mode_str, ui_params)
        if ok:
            sim_state.set_carla_status(True, f"Carla world loaded: {target_map} ({runtime_map})", sim_state.current_world_name)
            sim_state.set_pipeline_step("telemetry_ready", msg)
            wait_frontend_telemetry(timeout=1.5)
            sim_state.set_last_pipeline_result(True, msg)
        else:
            sim_state.set_last_pipeline_result(False, msg)
        try:
            sim_state.client.set_timeout(CARLA_RUNTIME_TIMEOUT)
        except Exception:
            pass
        return ok, msg
    except Exception as e:
        msg = f"pipeline failed: {e}"
        print(msg)
        sim_state.set_last_pipeline_result(False, msg)
        return False, msg
    finally:
        try:
            if sim_state.client is not None:
                sim_state.client.set_timeout(CARLA_RUNTIME_TIMEOUT)
        except Exception:
            pass
        deployment_lock.release()


def run_frontend_pipeline_background(body):
    try:
        ok, msg = run_frontend_pipeline(body)
        print(f"background deployment finished: ok={ok}, msg={msg}")
    except BaseException as e:
        msg = f"background deployment crashed: {type(e).__name__}: {e}"
        print(msg)
        sim_state.set_last_pipeline_result(False, msg)


class FrontendApiHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        return

    def send_json(self, status_code: int, payload: dict):
        try:
            self.send_response(status_code)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Connection", "close")
            self.end_headers()
            self.wfile.write(json.dumps(payload, ensure_ascii=False).encode("utf-8"))
        except (BrokenPipeError, ConnectionResetError):
            pass

    def do_GET(self):
        try:
            snap = sim_state.snapshot()
            if self.path == "/telemetry":
                self.send_json(200, {
                    "ok": True,
                    "target_ip": snap["target_ip"],
                    "drive_mode": snap["drive_mode"],
                    "camera_view": snap["camera_view"],
                    "camera_view_label": snap["camera_view_label"],
                    "camera_stream_url": snap["camera_stream_url"],
                    "camera_streams": snap["camera_streams"],
                    "side_camera_streams": snap["side_camera_streams"],
                    "camera_stream_reload_key": snap["camera_stream_reload_key"],
                    "paused": False,
                    "vehicle_alive": safe_actor_alive(sim_state.vehicle),
                    "telemetry": snap["data"].get("FULL_TELEMETRY", {}),
                    "diagnostics": get_vehicle_diagnostics(),
                })
                return
            if self.path != "/health":
                self.send_json(404, {"ok": False, "msg": "endpoint not found"})
                return
            self.send_json(200, {
                "ok": True,
                "api": "running",
                "carla_connected": snap["carla_connected"],
                "carla_status": snap["carla_status_message"],
                "carla_status_updated_at": snap["carla_status_updated_at"],
                "world": snap["current_world_name"],
                "vehicle_alive": safe_actor_alive(sim_state.vehicle),
                "drive_mode": snap["drive_mode"],
                "camera_view": snap["camera_view"],
                "camera_view_label": snap["camera_view_label"],
                "camera_stream_url": snap["camera_stream_url"],
                "camera_streams": snap["camera_streams"],
                "side_camera_streams": snap["side_camera_streams"],
                "camera_stream_reload_key": snap["camera_stream_reload_key"],
                "paused": False,
                "last_command": snap["last_command"],
                "last_result": snap["last_pipeline_result"],
                "target_ip": snap["target_ip"],
                "manual_bridge": snap["manual_bridge"],
                "diagnostics": get_vehicle_diagnostics(),
            })
        except Exception as e:
            self.send_json(500, {"ok": False, "msg": f"internal server error: {e}"})

    def do_POST(self):
        if self.path not in ("/command", "/view"):
            self.send_json(404, {"ok": False, "msg": "endpoint not found"})
            return
        try:
            content_len = int(self.headers.get("Content-Length", "0"))
            raw_body = self.rfile.read(content_len) if content_len > 0 else b"{}"
            try:
                body = json.loads(raw_body.decode("utf-8"))
            except json.JSONDecodeError as e:
                self.send_json(400, {"ok": False, "msg": f"invalid json: {e}"})
                return

            view_keys = ("camera_view", "Camera View", "view", "View", "视角", "camera")
            has_view_command = any(key in body for key in view_keys)
            if self.path == "/view":
                view = apply_camera_view_command(body)
                snap = sim_state.snapshot()
                self.send_json(200, {
                    "ok": True,
                    "msg": f"camera view switched to {camera_view_label(view)}",
                    "camera_view": view,
                    "camera_view_label": camera_view_label(view),
                    "camera_stream_url": camera_view_stream_url(view),
                    "camera_streams": camera_view_streams(),
                    "side_camera_streams": snap["side_camera_streams"],
                    "camera_stream_reload_key": snap["camera_stream_reload_key"],
                    "vehicle_alive": vehicle_ref_present(),
                    "target_ip": snap["target_ip"],
                })
                return

            required_groups = [
                ("scene", "Scene"),
                ("sky", "Weather Condition"),
                ("sunshinetime", "Sunshine Time"),
                ("drive_mode", "Drive Mode"),
                ("loadingtransportation", "Traffic Load"),
                ("vehiclemodel", "Vehicle Model"),
            ]
            has_six_command = all(any(key in body for key in group) for group in required_groups)
            if not has_six_command:
                if has_view_command:
                    view = apply_camera_view_command(body)
                    snap = sim_state.snapshot()
                    self.send_json(200, {
                        "ok": True,
                        "msg": f"camera view switched to {camera_view_label(view)}",
                        "camera_view": view,
                        "camera_view_label": camera_view_label(view),
                        "camera_stream_url": camera_view_stream_url(view),
                        "camera_streams": camera_view_streams(),
                        "side_camera_streams": snap["side_camera_streams"],
                        "camera_stream_reload_key": snap["camera_stream_reload_key"],
                        "vehicle_alive": safe_actor_alive(sim_state.vehicle),
                        "target_ip": snap["target_ip"],
                    })
                    return
                self.send_json(400, {
                    "ok": False,
                    "msg": "missing six command fields: scene/sky/sunshinetime/drive_mode/loadingtransportation/vehiclemodel, or send camera_view/view to switch camera only"
                })
                return

            if deployment_lock.locked():
                snap = sim_state.snapshot()
                self.send_json(202, {
                    "ok": True,
                    "msg": "deployment already running",
                    "last_result": snap["last_pipeline_result"],
                    "target_ip": snap["target_ip"],
                    "vehicle_alive": safe_actor_alive(sim_state.vehicle),
                    "telemetry": snap["data"].get("FULL_TELEMETRY", {}),
                })
                return

            body["_client_ip"] = self.client_address[0]
            response_view = apply_camera_view_command(body) if has_view_command else sim_state.camera_view
            response_snap = sim_state.snapshot()
            sim_state.set_last_pipeline_result(True, "command accepted, deployment running")
            threading.Thread(target=run_frontend_pipeline_background, args=(dict(body),), daemon=True).start()
            self.send_json(202, {
                "ok": True,
                "msg": "command accepted, deployment running",
                "target_ip": body["_client_ip"],
                "camera_view": response_view,
                "camera_stream_url": camera_view_stream_url(response_view),
                "camera_streams": camera_view_streams(),
                "side_camera_streams": response_snap["side_camera_streams"],
                "camera_stream_reload_key": response_snap["camera_stream_reload_key"],
                "vehicle_alive": safe_actor_alive(sim_state.vehicle),
                "telemetry": response_snap["data"].get("FULL_TELEMETRY", {}),
            })
        except Exception as e:
            self.send_json(500, {"ok": False, "msg": f"internal server error: {e}"})


def start_api_server():
    if api_server_started.is_set():
        return None
    try:
        server = ThreadingHTTPServer(("0.0.0.0", API_PORT), FrontendApiHandler)
    except OSError as e:
        print(f"API server not started on {API_PORT}: {e}")
        api_server_started.set()
        raise
    threading.Thread(target=server.serve_forever, daemon=True).start()
    api_server_started.set()
    print(f"Frontend API server listening on 0.0.0.0:{API_PORT}")
    return server


# ==========================================
# 4. 【战役枢纽】：绝对无干扰直通引擎
# ==========================================
def master_simulation_loop(client, vehicle_actor, sensors_list, stop_event, dyn_wrapper):
    telem_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    TARGET_PORTS = [5000, 5002, 5003] 
    ALGO_TELEMETRY_ADDR = ("127.0.0.1", 5000)
    
    ctrl_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ctrl_sock_bound = False
    try:
        ctrl_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        if hasattr(socket, "SO_REUSEPORT"):
            try:
                ctrl_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
            except Exception:
                pass
        last_bind_error = None
        for _ in range(5):
            try:
                ctrl_sock.bind(("0.0.0.0", 5001))
                ctrl_sock_bound = True
                break
            except Exception as e:
                last_bind_error = e
                time.sleep(0.2)
        if ctrl_sock_bound:
            ctrl_sock.setblocking(False)
        else:
            print(f"UDP 5001 绑定失败，当前轮次无法接收外部控制: {last_bind_error}")
    except Exception as e:
        print(f"UDP 5001 初始化失败: {e}")

    last_mode = None
    mode_started_at = time.time()
    ai_fallback_active = False
    ui_update_counter = 0
    ctrl_rx_count = 0

    try:
        while not stop_event.is_set():
            try:
                if not vehicle_actor or not vehicle_actor.is_alive:
                    break

                vehicle_actor.set_simulate_physics(True)

                current_mode = sim_state.drive_mode
                if current_mode != last_mode:
                    mode_started_at = time.time()
                    ai_fallback_active = False
                    try:
                        vehicle_actor.disable_constant_velocity()
                    except Exception:
                        pass
                    if is_ai_mode(current_mode):
                        try:
                            tm = client.get_trafficmanager(TM_PORT)
                            tm.set_synchronous_mode(False)
                            tm.ignore_lights_percentage(vehicle_actor, 0)
                            tm.ignore_signs_percentage(vehicle_actor, 0)
                            tm.vehicle_percentage_speed_difference(vehicle_actor, -20)
                        except Exception as e:
                            print(f"TrafficManager AI setup warning: {e}")
                        vehicle_actor.set_autopilot(True, TM_PORT)
                    else:
                        vehicle_actor.set_autopilot(False, TM_PORT)
                    last_mode = current_mode

                velocity = vehicle_actor.get_velocity()
                speed_kmh = math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2) * 3.6
                if is_ai_mode(current_mode):
                    if not ai_fallback_active and time.time() - mode_started_at >= AI_FALLBACK_AFTER_SEC and speed_kmh < 1.0:
                        try:
                            vehicle_actor.set_autopilot(False, TM_PORT)
                        except Exception:
                            pass
                        ai_fallback_active = True
                        try:
                            fv = vehicle_actor.get_transform().get_forward_vector()
                            vehicle_actor.enable_constant_velocity(carla.Vector3D(
                                fv.x * AI_FALLBACK_TARGET_SPEED_MS,
                                fv.y * AI_FALLBACK_TARGET_SPEED_MS,
                                fv.z * AI_FALLBACK_TARGET_SPEED_MS
                            ))
                        except Exception as e:
                            print(f"AI constant-velocity fallback failed, using throttle control: {e}")
                        print("AI autopilot did not move the ego vehicle; constant-velocity fallback enabled.")
                    if ai_fallback_active and speed_kmh < 1.0:
                        vehicle_actor.apply_control(carla.VehicleControl(
                            throttle=0.8,
                            steer=0.0,
                            brake=0.0,
                            hand_brake=False,
                            reverse=False,
                            manual_gear_shift=False,
                            gear=1
                        ))

                if dyn_wrapper:
                    legacy_telem_data = dyn_wrapper.fetch_telemetry_26_items()
                    frontend_telem_data = build_frontend_telemetry(
                        dyn_wrapper.config, dyn_wrapper.ui, legacy_telem_data, vehicle_actor
                    )
                    sim_state.data["LEGACY_TELEMETRY"] = legacy_telem_data
                    sim_state.data["FULL_TELEMETRY"] = frontend_telem_data
                    
                    ui_update_counter += 1
                    if ui_update_counter % 2 == 0: 
                        sim_state.speed_history.append(sim_state.smoothed_speed)
                        sim_state.data["SPEED"] = sim_state.smoothed_speed

                    try: 
                        payload_bytes = json.dumps(legacy_telem_data, ensure_ascii=False).encode('utf-8')
                        for p in TARGET_PORTS:
                            telem_sock.sendto(payload_bytes, (sim_state.target_ip, p))
                        if is_manual_mode(current_mode) and sim_state.target_ip not in ("127.0.0.1", "localhost"):
                            telem_sock.sendto(payload_bytes, ("127.0.0.1", 5000))
                    except: pass

                    if is_algo_mode(current_mode):
                        try:
                            if sim_state.target_ip not in ("127.0.0.1", "localhost"):
                                telem_sock.sendto(payload_bytes, ALGO_TELEMETRY_ADDR)
                        except Exception as e:
                            if ui_update_counter % 200 == 0:
                                print(f"算法旧格式遥测发送失败: {e}")
                    
                    if ctrl_sock_bound and not is_ai_mode(current_mode):
                        try:
                            ctrl_bytes, _ = ctrl_sock.recvfrom(2048)
                            while True:
                                try:
                                    ctrl_bytes, _ = ctrl_sock.recvfrom(2048)
                                except BlockingIOError:
                                    break
                            ctrl_dict = json.loads(ctrl_bytes.decode('utf-8'))

                            if is_algo_mode(current_mode):
                                throttle_cmd = max(0.0, min(1.0, float(ctrl_dict.get('throttle', 0))))
                                steer_cmd = max(-1.0, min(1.0, float(ctrl_dict.get('steer', 0))))
                                brake_cmd = max(0.0, min(1.0, float(ctrl_dict.get('brake', 0))))
                                ctrl_rx_count += 1
                                if ctrl_rx_count <= 5 or ctrl_rx_count % 100 == 0:
                                    print(
                                        f"[Algo Ctrl] #{ctrl_rx_count}: "
                                        f"throttle={throttle_cmd}, "
                                        f"steer={steer_cmd}, "
                                        f"brake={brake_cmd}"
                                    )
                                vehicle_actor.apply_control(carla.VehicleControl(
                                    throttle=throttle_cmd,
                                    steer=steer_cmd,
                                    brake=brake_cmd,
                                    reverse=bool(ctrl_dict.get('reverse', False)),
                                    hand_brake=bool(ctrl_dict.get('hand_brake', False)),
                                    manual_gear_shift=False,
                                    gear=1
                                ))
                            else:
                                throttle_cmd = max(0.0, min(1.0, float(ctrl_dict.get('throttle', 0))))
                                steer_cmd = max(-1.0, min(1.0, float(ctrl_dict.get('steer', 0))))
                                brake_cmd = max(0.0, min(1.0, float(ctrl_dict.get('brake', 0))))
                                vehicle_actor.apply_control(carla.VehicleControl(
                                    throttle=throttle_cmd,
                                    steer=steer_cmd,
                                    brake=brake_cmd,
                                    reverse=bool(ctrl_dict.get('reverse', False)),
                                    hand_brake=bool(ctrl_dict.get('hand_brake', False)),
                                    manual_gear_shift=False,
                                    gear=1
                                ))
                                if is_manual_mode(current_mode):
                                    sim_state.last_manual_control_time = time.time()
                                    sim_state.last_manual_control_cmd = {
                                        "throttle": round(throttle_cmd, 3),
                                        "steer": round(steer_cmd, 3),
                                        "brake": round(brake_cmd, 3),
                                        "reverse": bool(ctrl_dict.get('reverse', False)),
                                        "hand_brake": bool(ctrl_dict.get('hand_brake', False)),
                                    }
                                ctrl_rx_count += 1
                            if ctrl_rx_count == 1 or ctrl_rx_count % 100 == 0:
                                try:
                                    vel_dbg = vehicle_actor.get_velocity()
                                    speed_dbg = math.sqrt(vel_dbg.x ** 2 + vel_dbg.y ** 2 + vel_dbg.z ** 2) * 3.6
                                except Exception:
                                    speed_dbg = -1.0
                                print(
                                    "control packet received "
                                    f"#{ctrl_rx_count}: throttle={ctrl_dict.get('throttle', 0)}, "
                                    f"steer={ctrl_dict.get('steer', 0)}, brake={ctrl_dict.get('brake', 0)}, "
                                    f"speed={speed_dbg:.2f} km/h"
                                )
                        except BlockingIOError:
                            pass 
                        except Exception as e:
                            print(f"UDP 5001 解析报错: {e}")
                    if is_manual_mode(current_mode):
                        stale = (time.time() - sim_state.last_manual_control_time) if sim_state.last_manual_control_time else None
                        if stale is None or stale > MANUAL_CONTROL_TIMEOUT_SEC:
                            vehicle_actor.apply_control(carla.VehicleControl(
                                throttle=0.0,
                                steer=0.0,
                                brake=1.0,
                                hand_brake=False,
                                reverse=False,
                                manual_gear_shift=False,
                                gear=1
                            ))
            except RuntimeError as e:
                msg = f"master loop stopped: CARLA actor was destroyed or unavailable: {e}"
                print(msg)
                sim_state.set_last_pipeline_result(False, msg)
                break
            
            time.sleep(0.01) 
    finally:
        telem_sock.close()
        ctrl_sock.close()


start_api_server()
start_carla_auto_connector()

if os.environ.get("CARLA_DELIVERY_BACKEND_ONLY", "0") == "1":
    print("CARLA delivery backend-only mode active. API server and auto connector are running.")
    try:
        while True:
            time.sleep(3600)
    except KeyboardInterrupt:
        pass
    st.stop()

# ==========================================
# 5. Streamlit 只读监控壳
# ==========================================
st.title("L4 Runtime Monitor")
st.caption("HTTP API is the only control entry. This page is read-only monitoring.")

snap = sim_state.snapshot()
col1, col2, col3, col4 = st.columns(4)
col1.metric("API", "running")
col2.metric("CARLA", "connected" if snap["carla_connected"] else "disconnected")
col3.metric("World", snap["current_world_name"] or "-")
col4.metric("Vehicle", "alive" if (sim_state.vehicle and sim_state.vehicle.is_alive) else "offline")

st.markdown("**Backend Status**")
st.json({
    "carla_status": snap["carla_status_message"],
    "carla_status_updated_at": snap["carla_status_updated_at"],
    "drive_mode": snap["drive_mode"],
    "target_ip": snap["target_ip"],
    "last_command": snap["last_command"],
    "last_result": snap["last_pipeline_result"],
    "scene_summary": snap["scene_summary"],
})

if snap["data"].get("FULL_TELEMETRY"):
    st.markdown("**Frontend Telemetry**")
    st.json(snap["data"]["FULL_TELEMETRY"])

sensor_data = {
    "GNSS": snap["data"].get("GNSS_DATA", []),
    "IMU": snap["data"].get("IMU_DATA", {}),
    "RADAR_TARGETS": snap["data"].get("RADAR_TARGETS", 0),
    "COLLISION": snap["data"].get("COLLISION_DATA", {}),
}
st.markdown("**Sensors**")
st.json(sensor_data)

if sim_state.speed_history:
    st.markdown("**Speed History**")
    st.line_chart(pd.DataFrame(list(sim_state.speed_history), columns=["Speed (km/h)"]), height=180, use_container_width=True)

st.stop()
