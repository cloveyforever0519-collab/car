#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CARLA pygame display client for HIL/AIGO.

This follows the official examples approach: a CARLA RGB camera sensor feeds a
pygame window directly. It keeps the old UDP contract:
  - external telemetry out: target client IP:5000, 5002, 5003
  - AIGO private telemetry out: 127.0.0.1:5500 by default
  - Manual bridge private telemetry out: 127.0.0.1:5501 by default
  - control in:    0.0.0.0:5001
"""

from __future__ import annotations

import argparse
import glob
import hashlib
import io
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
from typing import Any, Dict, List, Optional, Tuple

try:
    import numpy as np
    import pygame
except Exception as exc:
    raise RuntimeError("pygame and numpy are required in the carla_vcu environment") from exc

try:
    from PIL import Image
except Exception:
    Image = None


APP_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = Path(os.environ.get("CARLA_PROJECT_ROOT", APP_DIR.parent)).resolve()
LOG_DIR = Path(os.environ.get("OFFICIAL_DEMO_LOG_DIR", PROJECT_ROOT / "logs"))
STATUS_FILE = Path(os.environ.get("OFFICIAL_UDP_STATUS_FILE", LOG_DIR / "official_udp_status.json"))
VIEW_COMMAND_FILE = Path(os.environ.get("OFFICIAL_VIEW_COMMAND_FILE", LOG_DIR / "official_view_command.json"))


def env_flag(name: str, default: str = "0") -> bool:
    return os.environ.get(name, default).strip().lower() in {"1", "true", "yes", "on"}


def env_float(name: str, default: str, lo: Optional[float] = None, hi: Optional[float] = None) -> float:
    try:
        value = float(os.environ.get(name, default))
    except Exception:
        value = float(default)
    if lo is not None:
        value = max(lo, value)
    if hi is not None:
        value = min(hi, value)
    return value


def add_carla_paths() -> None:
    py_tag = f"py{sys.version_info.major}.{sys.version_info.minor}"
    candidates = [
        os.environ.get("CARLA_PYTHONAPI"),
        os.environ.get("CARLA_EXAMPLES_DIR"),
        str(PROJECT_ROOT / "PythonAPI"),
        str(PROJECT_ROOT / "PythonAPI" / "carla"),
        str(Path.home() / "CARLA_0.9.15" / "PythonAPI"),
        str(Path.home() / "CARLA_0.9.15" / "PythonAPI" / "carla"),
        str(Path.home() / "CARLA_0.9.15" / "PythonAPI" / "examples"),
        "/home/z/Workspace/carla_hil_project/PythonAPI",
        "/home/z/Workspace/carla_hil_project/PythonAPI/carla",
        "/home/z/Workspace/carla_hil_project/PythonAPI/examples",
        "/home/zhang/Workspace/carla_hil_project/PythonAPI",
        "/home/zhang/Workspace/carla_hil_project/PythonAPI/carla",
        "/home/zhang/Workspace/carla_hil_project/PythonAPI/examples",
        "/home/z/CARLA_0.9.15/PythonAPI",
        "/home/z/CARLA_0.9.15/PythonAPI/carla",
        "/home/z/CARLA_0.9.15/PythonAPI/examples",
        "/home/zhang/CARLA_0.9.15/PythonAPI",
        "/home/zhang/CARLA_0.9.15/PythonAPI/carla",
        "/home/zhang/CARLA_0.9.15/PythonAPI/examples",
    ]
    for base in candidates:
        if not base:
            continue
        base_path = Path(base)
        if not base_path.exists():
            continue
        if str(base_path) not in sys.path:
            sys.path.append(str(base_path))
        for egg in glob.glob(str(base_path / "dist" / f"carla-*{py_tag}*.egg")):
            if egg not in sys.path:
                sys.path.insert(0, egg)


try:
    import carla  # type: ignore  # noqa: E402
except ImportError:
    add_carla_paths()
    import carla  # type: ignore  # noqa: E402

add_carla_paths()
try:
    from agents.navigation.basic_agent import BasicAgent  # type: ignore  # noqa: E402
except Exception:
    BasicAgent = None  # type: ignore
try:
    from agents.navigation.global_route_planner import GlobalRoutePlanner  # type: ignore  # noqa: E402
except Exception:
    GlobalRoutePlanner = None  # type: ignore


SCENARIOS = {
    "Town01": {"pos": (-2.0, 8.0, 2.0, 90.0), "script": "vshuangyi.py"},
    "Town02": {"pos": (3.0, 109.5, 2.0, 0.0), "script": "vdanyi.py"},
    "Town03": {"pos": (-42.0, 204.0, 2.0, 0.0), "script": "vjiansu.py"},
    "Town04": {"pos": (9.0, 237.0, 2.0, -90.0), "script": "vshexing.py"},
    "Town05": {"pos": (206.6, 110.0, 2.0, -90.0), "script": "vjiasu.py"},
    "TrainingGround": {"pos": (9.0, 237.0, 2.0, -90.0), "script": "vshexing.py", "runtime_map": "Town04"},
}

SCENE_ALIASES = {
    "Town01/Urban City District": "Town01",
    "Town02/Low-Density Suburban Area": "Town02",
    "Town03/High-Density Residential Zone": "Town03",
    "Town04/High-Speed Expressway": "Town04",
    "Town05/Performance Proving Ground": "Town05",
    "Town04Forest": "TrainingGround",
    "训练场": "TrainingGround",
}

VEHICLE_BLUEPRINTS = {
    "Dodge Charger": "vehicle.dodge.charger_2020",
    "Lincoln MKZ": "vehicle.lincoln.mkz_2017",
    "Tesla Model 3": "vehicle.tesla.model3",
    "Audi e-tron": "vehicle.audi.etron",
    "Jeep Wrangler": "vehicle.jeep.wrangler_rubicon",
    "Tesla Cybertruck": "vehicle.tesla.cybertruck",
    "Fuso Rosa": "vehicle.mitsubishi.fusorosa",
    "Mercedes Sprinter": "vehicle.mercedes.sprinter",
    "Volkswagen T2": "vehicle.volkswagen.t2_2021",
    "Carlacola Truck": "vehicle.carlamotors.carlacola",
    "European HGV": "vehicle.carlamotors.european_hgv",
    "Firetruck": "vehicle.carlamotors.firetruck",
}

VEHICLE_FILTERS = {
    "Dodge Charger": ["vehicle.dodge.charger_2020", "vehicle.dodge.charger_police_2020", "*charger*"],
    "Lincoln MKZ": ["vehicle.lincoln.mkz_2017", "vehicle.lincoln.mkz_2020", "*lincoln*", "*mkz*"],
    "Tesla Model 3": ["vehicle.tesla.model3", "*model3*"],
    "Audi e-tron": ["vehicle.audi.etron", "*etron*", "*audi*"],
    "Jeep Wrangler": ["vehicle.jeep.wrangler_rubicon", "*wrangler*", "*jeep*"],
    "Tesla Cybertruck": ["vehicle.tesla.cybertruck", "*cybertruck*"],
    "Fuso Rosa": ["vehicle.mitsubishi.fusorosa", "*fusorosa*", "*fuso*"],
    "Mercedes Sprinter": ["vehicle.mercedes.sprinter", "*sprinter*"],
    "Volkswagen T2": ["vehicle.volkswagen.t2_2021", "vehicle.volkswagen.t2", "*volkswagen*", "*t2*"],
    "Carlacola Truck": ["vehicle.carlamotors.carlacola", "*carlacola*"],
    "European HGV": ["vehicle.carlamotors.european_hgv", "*european_hgv*", "*hgv*"],
    "Firetruck": ["vehicle.carlamotors.firetruck", "*firetruck*"],
}

GEOMETRY = {
    "Dodge Charger": {"wheelbase": 3.05, "track": 1.62, "tire_radius": 0.364, "mass": 1800.0, "max_steer_deg": 40.0},
    "Lincoln MKZ": {"wheelbase": 2.85, "track": 1.58, "tire_radius": 0.334, "mass": 1965.0, "max_steer_deg": 35.0},
    "Tesla Model 3": {"wheelbase": 2.88, "track": 1.58, "tire_radius": 0.334, "mass": 1800.0, "max_steer_deg": 38.0},
    "Audi e-tron": {"wheelbase": 2.93, "track": 1.65, "tire_radius": 0.345, "mass": 2565.0, "max_steer_deg": 38.0},
    "Jeep Wrangler": {"wheelbase": 3.01, "track": 1.60, "tire_radius": 0.390, "mass": 2000.0, "max_steer_deg": 40.0},
    "Tesla Cybertruck": {"wheelbase": 3.81, "track": 1.75, "tire_radius": 0.430, "mass": 3000.0, "max_steer_deg": 40.0},
    "Fuso Rosa": {"wheelbase": 3.99, "track": 1.70, "tire_radius": 0.390, "mass": 3950.0, "max_steer_deg": 40.0},
    "Mercedes Sprinter": {"wheelbase": 3.66, "track": 1.73, "tire_radius": 0.390, "mass": 3000.0, "max_steer_deg": 40.0},
    "Volkswagen T2": {"wheelbase": 2.40, "track": 1.38, "tire_radius": 0.320, "mass": 1450.0, "max_steer_deg": 40.0},
    "Carlacola Truck": {"wheelbase": 5.20, "track": 2.05, "tire_radius": 0.520, "mass": 12500.0, "max_steer_deg": 40.0},
    "European HGV": {"wheelbase": 3.80, "track": 2.05, "tire_radius": 0.520, "mass": 12500.0, "max_steer_deg": 40.0},
    "Firetruck": {"wheelbase": 5.80, "track": 2.10, "tire_radius": 0.540, "mass": 16500.0, "max_steer_deg": 40.0},
}

TRAFFIC_COUNT = int(os.environ.get("OFFICIAL_DEMO_TRAFFIC_COUNT", "18"))
TRAFFIC_MANAGER_PORT = int(os.environ.get("OFFICIAL_TRAFFIC_MANAGER_PORT", "8000"))
AI_USE_MAP_SPAWN = env_flag("OFFICIAL_AI_USE_MAP_SPAWN", "0")
AI_IGNORE_LIGHTS_PERCENT = env_float("OFFICIAL_AI_IGNORE_LIGHTS_PERCENT", "0", 0.0, 100.0)
AI_IGNORE_SIGNS_PERCENT = env_float("OFFICIAL_AI_IGNORE_SIGNS_PERCENT", "0", 0.0, 100.0)
AI_SPEED_DIFFERENCE_PERCENT = env_float("OFFICIAL_AI_SPEED_DIFFERENCE_PERCENT", "-20.0", -100.0, 100.0)
AI_FALLBACK_ENABLED = env_flag("OFFICIAL_AI_FALLBACK_ENABLED", "0")
AI_FALLBACK_AFTER_SEC = env_float("OFFICIAL_AI_FALLBACK_AFTER_SEC", "8.0", 0.0, None)
AI_FALLBACK_TARGET_SPEED_KMH = env_float("OFFICIAL_AI_FALLBACK_TARGET_SPEED_KMH", "25.0", 0.0, None)
AI_FALLBACK_TARGET_SPEED_MS = AI_FALLBACK_TARGET_SPEED_KMH / 3.6
ROUTE_TARGET_SPEED_KMH = env_float("OFFICIAL_ROUTE_TARGET_SPEED_KMH", "35.0", 1.0, 120.0)
ROUTE_POINT_TARGET_DISTANCE_M = env_float("OFFICIAL_ROUTE_POINT_TARGET_DISTANCE_M", "200.0", 30.0, 2000.0)
ROUTE_POINT_MIN_DISTANCE_M = env_float("OFFICIAL_ROUTE_POINT_MIN_DISTANCE_M", "120.0", 10.0, None)
ROUTE_POINT_MAX_DISTANCE_M = env_float("OFFICIAL_ROUTE_POINT_MAX_DISTANCE_M", "320.0", 20.0, None)
ROUTE_STEER_LIMIT = env_float("OFFICIAL_ROUTE_STEER_LIMIT", "0.18", 0.05, 1.0)
ROUTE_STEER_RATE_LIMIT = env_float("OFFICIAL_ROUTE_STEER_RATE_LIMIT", "0.45", 0.1, 10.0)
ROUTE_TURN_THROTTLE_STEER = env_float("OFFICIAL_ROUTE_TURN_THROTTLE_STEER", "0.08", 0.0, 1.0)
ROUTE_TURN_BRAKE_STEER = env_float("OFFICIAL_ROUTE_TURN_BRAKE_STEER", "0.14", 0.0, 1.0)
ROUTE_TURN_MAX_BRAKE = env_float("OFFICIAL_ROUTE_TURN_MAX_BRAKE", "0.35", 0.0, 1.0)
ROUTE_LATERAL_KP = env_float("OFFICIAL_ROUTE_LATERAL_KP", "0.55", 0.05, 3.0)
ROUTE_LATERAL_KD = env_float("OFFICIAL_ROUTE_LATERAL_KD", "0.08", 0.0, 1.0)
ROUTE_LATERAL_KI = env_float("OFFICIAL_ROUTE_LATERAL_KI", "0.0", 0.0, 1.0)
ROUTE_LOCAL_SAMPLING_RADIUS = env_float("OFFICIAL_ROUTE_LOCAL_SAMPLING_RADIUS", "4.0", 1.0, 20.0)
ROUTE_LOCAL_BASE_MIN_DISTANCE = env_float("OFFICIAL_ROUTE_LOCAL_BASE_MIN_DISTANCE", "6.0", 1.0, 30.0)
ROUTE_AVOID_POINT_RADIUS_M = env_float("OFFICIAL_ROUTE_AVOID_POINT_RADIUS_M", "35.0", 5.0, 200.0)
ROUTE_SHARED_ROUTE_CELL_SIZE_M = env_float("OFFICIAL_ROUTE_SHARED_ROUTE_CELL_SIZE_M", "8.0", 1.0, 50.0)
ROUTE_MAX_SHARED_ROUTE_CELLS = int(env_float("OFFICIAL_ROUTE_MAX_SHARED_ROUTE_CELLS", "8", 0.0, 100.0))
ROUTE_CANDIDATE_LIMIT = int(env_float("OFFICIAL_ROUTE_CANDIDATE_LIMIT", "45", 10.0, 200.0))
ROUTE_SAMPLING_RESOLUTION = env_float("OFFICIAL_ROUTE_SAMPLING_RESOLUTION", "2.0", 0.5, 10.0)
ROUTE_TURN_PENALTY_THRESHOLD_DEG = env_float("OFFICIAL_ROUTE_TURN_PENALTY_THRESHOLD_DEG", "28.0", 1.0, 180.0)
ROUTE_TURN_PENALTY_WEIGHT = env_float("OFFICIAL_ROUTE_TURN_PENALTY_WEIGHT", "8.0", 0.0, 100.0)
ROUTE_TM_ENABLED = env_flag("OFFICIAL_ROUTE_TM_ENABLED", "1")
ROUTE_TM_SEED = int(env_float("OFFICIAL_ROUTE_TM_SEED", "42", 0.0, 1000000.0))
ROUTE_TM_SPEED_DIFFERENCE_PERCENT = env_float("OFFICIAL_ROUTE_TM_SPEED_DIFFERENCE_PERCENT", "35.0", -100.0, 100.0)
ROUTE_TM_DISTANCE_TO_LEADING_M = env_float("OFFICIAL_ROUTE_TM_DISTANCE_TO_LEADING_M", "2.5", 0.0, 50.0)
ROUTE_ARRIVAL_DISTANCE_M = env_float("OFFICIAL_ROUTE_ARRIVAL_DISTANCE_M", "8.0", 1.0, 50.0)
FIXED_DELTA_SECONDS = env_float("OFFICIAL_FIXED_DELTA_SECONDS", "0.02", 0.001, None)
PHYSICS_SUBSTEP_DELTA = env_float("OFFICIAL_MAX_SUBSTEP_DELTA_TIME", "0.005", 0.001, None)
PHYSICS_MAX_SUBSTEPS = int(os.environ.get("OFFICIAL_MAX_SUBSTEPS", "16"))
CAMERA_SENSOR_TICK = env_float("OFFICIAL_CAMERA_SENSOR_TICK", "0.02", 0.0, None)
DISPLAY_LOOP_HZ = int(os.environ.get("OFFICIAL_DISPLAY_LOOP_HZ", "60"))
MAX_UDP_PAYLOAD_BYTES = int(os.environ.get("OFFICIAL_MAX_UDP_PAYLOAD_BYTES", "60000"))
SHUTDOWN_EVENT = threading.Event()
DEFAULT_TELEMETRY_PORTS = os.environ.get("OFFICIAL_TELEMETRY_PORTS", "5000,5002,5003")
DEFAULT_ALGO_TELEMETRY_PORT = int(os.environ.get("OFFICIAL_ALGO_TELEMETRY_PORT", "5500"))
DEFAULT_MANUAL_TELEMETRY_PORT = int(os.environ.get("OFFICIAL_MANUAL_TELEMETRY_PORT", "5501"))
ENABLE_SENSOR_SUITE = env_flag("OFFICIAL_ENABLE_SENSORS", "1")
DISABLE_AIGO_SENSORS = env_flag("OFFICIAL_AIGO_DISABLE_SENSORS", "1")
DISABLE_AI_SENSORS = env_flag("OFFICIAL_AI_BASELINE_DISABLE_SENSORS", "1")
SENSOR_UDP_PORTS = os.environ.get("OFFICIAL_SENSOR_UDP_PORTS", "5010")
SENSOR_SUMMARY_HZ = float(os.environ.get("OFFICIAL_SENSOR_SUMMARY_HZ", "5"))
SENSOR_UDP_HZ = float(os.environ.get("OFFICIAL_SENSOR_UDP_HZ", "5"))
SENSOR_LIDAR_MAX_PPS = int(float(os.environ.get("OFFICIAL_SENSOR_LIDAR_MAX_PPS", "300000")))
SENSOR_LIDAR_MAX_HZ = float(os.environ.get("OFFICIAL_SENSOR_LIDAR_MAX_HZ", "5"))
SIDE_STREAM_HOST = os.environ.get("OFFICIAL_SIDE_STREAM_HOST", "0.0.0.0")
SIDE_STREAM_PORT = int(os.environ.get("OFFICIAL_SIDE_STREAM_PORT", "8771"))
SIDE_STREAM_FPS = float(os.environ.get("OFFICIAL_SIDE_STREAM_FPS", "15.0"))
SIDE_CAMERA_WIDTH = int(os.environ.get("OFFICIAL_SIDE_CAMERA_WIDTH", "960"))
SIDE_CAMERA_HEIGHT = int(os.environ.get("OFFICIAL_SIDE_CAMERA_HEIGHT", "540"))
SIDE_CAMERA_FOV = float(os.environ.get("OFFICIAL_SIDE_CAMERA_FOV", "82.0"))
SIDE_MIRRORS_ALWAYS_ON = env_flag("OFFICIAL_SIDE_MIRRORS_ALWAYS_ON", "1")
MAX_STEER_CACHE: Dict[int, float] = {}

CAMERA_PROFILES = {
    "Dodge Charger": {
        "follow": {"x": -11.0, "y": 0.0, "z": 5.30, "pitch": -10.2, "fov": 98.0},
        "driver": {"x": 0.32, "y": -0.40, "z": 1.22, "pitch": -2.0, "fov": 93.0},
    },
    "Lincoln MKZ": {
        "follow": {"x": -10.7, "y": 0.0, "z": 5.20, "pitch": -10.2, "fov": 98.0},
        "driver": {"x": 0.38, "y": -0.38, "z": 1.32, "pitch": -2.0, "fov": 93.0},
    },
    "Tesla Model 3": {
        "follow": {"x": -10.4, "y": 0.0, "z": 5.05, "pitch": -10.0, "fov": 98.0},
        "driver": {"x": 0.34, "y": -0.38, "z": 1.28, "pitch": -2.0, "fov": 93.0},
    },
    "Audi e-tron": {
        "follow": {"x": -12.0, "y": 0.0, "z": 5.85, "pitch": -9.8, "fov": 99.0},
        "driver": {"x": 0.55, "y": -0.42, "z": 1.30, "pitch": -2.0, "fov": 93.0},
    },
    "Jeep Wrangler": {
        "follow": {"x": -12.2, "y": 0.0, "z": 6.10, "pitch": -9.8, "fov": 99.0},
        "driver": {"x": -0.08, "y": -0.34, "z": 1.50, "pitch": -2.5, "fov": 93.0},
    },
    "Tesla Cybertruck": {
        "follow": {"x": -13.8, "y": 0.0, "z": 6.70, "pitch": -9.6, "fov": 100.0},
        "driver": {"x": 0.55, "y": -0.45, "z": 1.96, "pitch": -2.5, "fov": 94.0},
    },
    "Fuso Rosa": {
        "follow": {"x": -15.0, "y": 0.0, "z": 7.20, "pitch": -9.2, "fov": 100.0},
        "driver": {"x": 2.95, "y": -0.55, "z": 2.82, "pitch": -4.0, "fov": 92.0},
    },
    "Mercedes Sprinter": {
        "follow": {"x": -13.8, "y": 0.0, "z": 6.75, "pitch": -9.4, "fov": 99.0},
        "driver": {"x": 1.05, "y": -0.48, "z": 1.88, "pitch": -3.0, "fov": 93.0},
    },
    "Volkswagen T2": {
        "follow": {"x": -11.2, "y": 0.0, "z": 5.65, "pitch": -9.8, "fov": 98.0},
        "driver": {"x": 1.20, "y": -0.40, "z": 1.62, "pitch": -3.0, "fov": 93.0},
    },
    "Carlacola Truck": {
        "follow": {"x": -18.5, "y": 0.0, "z": 8.85, "pitch": -8.8, "fov": 101.0},
        "driver": {"x": 1.80, "y": -0.42, "z": 1.80, "pitch": -6.0, "fov": 92.0},
    },
    "European HGV": {
        "follow": {"x": -17.2, "y": 0.0, "z": 8.60, "pitch": -8.8, "fov": 101.0},
        "driver": {"x": 2.85, "y": -0.62, "z": 2.65, "pitch": -6.0, "fov": 92.0},
    },
    "Firetruck": {
        "follow": {"x": -20.0, "y": 0.0, "z": 9.35, "pitch": -8.6, "fov": 102.0},
        "driver": {"x": 2.90, "y": -0.62, "z": 2.60, "pitch": -6.0, "fov": 92.0},
    },
}

FRONTEND_VEHICLES = tuple(VEHICLE_BLUEPRINTS.keys())
_profile_mismatch = set(FRONTEND_VEHICLES) ^ set(CAMERA_PROFILES.keys())
if _profile_mismatch:
    raise RuntimeError(f"camera profile mismatch for frontend vehicles: {sorted(_profile_mismatch)}")

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
    "LeftDriveHalfShaftTwist", "RightDriveHalfShaftTwist",
]

FRONTEND_GEOMETRY = {
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


def clamp(value: Any, lo: float, hi: float, default: float = 0.0) -> float:
    try:
        x = float(value)
    except Exception:
        x = default
    return max(lo, min(hi, x))


def normalize_scene(scene: str) -> str:
    scene = str(scene or "Town02").strip()
    return SCENE_ALIASES.get(scene, scene if scene in SCENARIOS else "Town02")


def normalize_view(view: str) -> str:
    text = str(view or "follow").strip().lower().replace("-", "_")
    if text in {"driver", "first", "first_person", "first person", "cockpit", "ego"}:
        return "driver"
    if text in {"follow", "third", "third_person", "third person", "chase", "rear", "back", "spectator", "default"}:
        return "follow"
    if any(token in str(view) for token in ("驾驶", "第一", "座舱")):
        return "driver"
    return "follow"


def runtime_map(scene: str) -> str:
    return SCENARIOS[scene].get("runtime_map", scene)


def spawn_transform(scene: str) -> carla.Transform:
    x, y, z, yaw = SCENARIOS[scene]["pos"]
    return carla.Transform(carla.Location(x=x, y=y, z=z), carla.Rotation(yaw=yaw))


def apply_weather(world: carla.World, weather: str, time_of_day: str) -> None:
    w = carla.WeatherParameters()
    weather_text = str(weather or "Sunny")
    if weather_text in {"Cloudy", "多云"}:
        w.cloudiness = 80.0
    elif weather_text in {"Light Rain", "小雨"}:
        w.cloudiness = 80.0
        w.precipitation = 30.0
        w.precipitation_deposits = 30.0
    elif weather_text in {"Heavy Rainstorm", "暴雨"}:
        w.cloudiness = 100.0
        w.precipitation = 90.0
        w.precipitation_deposits = 90.0
        w.wind_intensity = 80.0
    elif weather_text in {"Fog/Dense Fog", "大雾"}:
        w.cloudiness = 50.0
        w.fog_density = 50.0
        w.fog_distance = 10.0

    time_text = str(time_of_day or "Noon")
    if time_text in {"Sunset", "夕阳"}:
        w.sun_altitude_angle = 5.0
    elif time_text in {"Late Night", "深夜"}:
        w.sun_altitude_angle = -90.0
    else:
        w.sun_altitude_angle = 75.0
    world.set_weather(w)


def set_async(world: carla.World) -> None:
    settings = world.get_settings()
    settings.synchronous_mode = False
    settings.fixed_delta_seconds = FIXED_DELTA_SECONDS
    if hasattr(settings, "substepping"):
        settings.substepping = True
    if hasattr(settings, "max_substep_delta_time"):
        settings.max_substep_delta_time = PHYSICS_SUBSTEP_DELTA
    if hasattr(settings, "max_substeps"):
        settings.max_substeps = PHYSICS_MAX_SUBSTEPS
    world.apply_settings(settings)


def destroy_actor(actor: Any) -> None:
    try:
        if actor and actor.is_alive:
            try:
                if hasattr(actor, "stop"):
                    actor.stop()
            except Exception:
                pass
            try:
                if str(getattr(actor, "type_id", "")).startswith("vehicle."):
                    try:
                        actor.set_autopilot(False, TRAFFIC_MANAGER_PORT)
                    except TypeError:
                        actor.set_autopilot(False)
                    try:
                        actor.disable_constant_velocity()
                    except Exception:
                        pass
            except Exception:
                pass
            actor.destroy()
    except Exception:
        pass


def terminate_process(proc: Optional[subprocess.Popen]) -> None:
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
            proc.wait(timeout=10.0)
        except Exception:
            if os.name != "nt":
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except Exception:
                    proc.kill()
            else:
                proc.kill()
    except Exception:
        pass


def write_status(payload: Dict[str, Any]) -> None:
    try:
        LOG_DIR.mkdir(exist_ok=True)
        tmp = STATUS_FILE.with_suffix(".tmp")
        tmp.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
        tmp.replace(STATUS_FILE)
    except Exception:
        pass


def parse_port_list(value: Any) -> List[int]:
    ports: List[int] = []
    for item in str(value or "").split(","):
        item = item.strip()
        if not item:
            continue
        try:
            port = int(item)
        except ValueError:
            continue
        if 0 < port < 65536 and port not in ports:
            ports.append(port)
    return ports


def safe_udp_send(sock: socket.socket, payload: bytes, target: Tuple[str, int], label: str) -> bool:
    if len(payload) > MAX_UDP_PAYLOAD_BYTES:
        print(
            f"{label} udp payload dropped: {len(payload)} bytes exceeds {MAX_UDP_PAYLOAD_BYTES}",
            flush=True,
        )
        return False
    try:
        sock.sendto(payload, target)
        return True
    except Exception as exc:
        print(f"{label} udp send failed to {target}: {exc}", flush=True)
        return False


def fmt_frontend(value: Any, digits: int = 2, suffix: str = "") -> str:
    try:
        text = f"{float(value):.{digits}f}"
        if "." in text:
            text = text.rstrip("0").rstrip(".")
    except Exception:
        text = str(value if value is not None else "")
    return f"{text} {suffix}".strip() if suffix else text


def nested_find(mapping: Dict[str, Any], token: str, default: Any = None) -> Any:
    for key, value in (mapping or {}).items():
        if token in str(key):
            return value
    return default


def normalize_frontend_payload(payload: Dict[str, Any]) -> Dict[str, str]:
    return {field: "" if payload.get(field) is None else str(payload.get(field, "")) for field in FRONTEND_OUTPUT_FIELDS}


class CameraDisplay:
    def __init__(self, world: carla.World, vehicle: carla.Actor, size: Tuple[int, int], view: str, vehicle_model: str) -> None:
        self.world = world
        self.vehicle = vehicle
        self.width, self.height = size
        self.view = normalize_view(view)
        self.vehicle_model = vehicle_model if vehicle_model in CAMERA_PROFILES else "Lincoln MKZ"
        self.sensor: Optional[carla.Sensor] = None
        self.frame_lock = threading.Lock()
        self.frame: Optional[np.ndarray] = None
        self.spawn()

    def profile(self, view: str) -> Dict[str, float]:
        profile = CAMERA_PROFILES.get(self.vehicle_model, CAMERA_PROFILES["Lincoln MKZ"])[view]
        extent = self.vehicle.bounding_box.extent
        if view == "follow":
            x = min(float(profile["x"]), -(float(extent.x) + 6.8))
            z = max(float(profile["z"]), float(extent.z) + 2.80)
            pitch = clamp(profile["pitch"], -11.5, -8.0, -9.6)
            fov = clamp(profile["fov"], 96.0, 104.0, 99.0)
            return {"x": x, "y": float(profile.get("y", 0.0)), "z": z, "pitch": pitch, "fov": fov}
        x_hi = max(float(extent.x) - 0.25, 0.60)
        x = clamp(profile["x"], -0.20, x_hi, 0.45)
        z = clamp(profile["z"], 0.85, max(5.00, float(extent.z) + 1.25), 1.30)
        return {
            "x": x,
            "y": float(profile.get("y", -0.38)),
            "z": z,
            "pitch": clamp(profile["pitch"], -8.0, 0.0, -2.0),
            "fov": clamp(profile["fov"], 90.0, 96.0, 93.0),
        }

    def camera_mount(self) -> Tuple[carla.Transform, Any, float]:
        if self.view == "driver":
            p = self.profile("driver")
            tf = carla.Transform(
                carla.Location(x=p["x"], y=p["y"], z=p["z"]),
                carla.Rotation(pitch=p["pitch"]),
            )
            return tf, carla.AttachmentType.Rigid, p["fov"]
        attach = getattr(carla.AttachmentType, "SpringArmGhost", carla.AttachmentType.Rigid)
        p = self.profile("follow")
        tf = carla.Transform(
            carla.Location(x=p["x"], y=p["y"], z=p["z"]),
            carla.Rotation(pitch=p["pitch"]),
        )
        return tf, attach, p["fov"]

    def spawn(self) -> None:
        self.destroy()
        bp = self.world.get_blueprint_library().find("sensor.camera.rgb")
        bp.set_attribute("image_size_x", str(self.width))
        bp.set_attribute("image_size_y", str(self.height))
        tf, attach, fov = self.camera_mount()
        bp.set_attribute("fov", str(fov))
        if bp.has_attribute("sensor_tick"):
            bp.set_attribute("sensor_tick", str(max(0.0, CAMERA_SENSOR_TICK)))
        self.sensor = self.world.spawn_actor(bp, tf, attach_to=self.vehicle, attachment_type=attach)
        self.sensor.listen(self._on_image)

    def _on_image(self, image: Any) -> None:
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = array.reshape((image.height, image.width, 4))
        array = array[:, :, :3][:, :, ::-1]
        with self.frame_lock:
            self.frame = array

    def set_view(self, view: str) -> None:
        view = normalize_view(view)
        if view != self.view:
            self.view = view
            self.spawn()

    def render(self, display: pygame.Surface) -> None:
        with self.frame_lock:
            frame = None if self.frame is None else self.frame.copy()
        if frame is None:
            display.fill((0, 0, 0))
            return
        surface = pygame.surfarray.make_surface(frame.swapaxes(0, 1))
        display.blit(surface, (0, 0))

    def destroy(self) -> None:
        if self.sensor is not None:
            try:
                self.sensor.stop()
            except Exception:
                pass
            destroy_actor(self.sensor)
        self.sensor = None


class SideMirrorCamera:
    def __init__(self, world: carla.World, vehicle: carla.Actor, name: str) -> None:
        self.world = world
        self.vehicle = vehicle
        self.name = name
        self.sensor: Optional[carla.Sensor] = None
        self.lock = threading.Condition()
        self.frame_id = 0
        self.frame_bytes: Optional[bytes] = None
        self.content_type = "image/jpeg"
        self.closed = False
        self.active_clients = 0

    def camera_mount(self) -> carla.Transform:
        extent = self.vehicle.bounding_box.extent
        side = -1.0 if self.name == "rear_left" else 1.0
        x = clamp(float(extent.x) * 0.32, 0.65, 2.80, 1.00)
        y = side * (float(extent.y) + 0.22)
        z = max(1.15, min(float(extent.z) + 0.70, float(extent.z) * 1.05 + 0.35))
        yaw = -145.0 if self.name == "rear_left" else 145.0
        return carla.Transform(
            carla.Location(x=x, y=y, z=z),
            carla.Rotation(pitch=-5.0, yaw=yaw),
        )

    def spawn(self) -> bool:
        if self.closed or self.sensor is not None:
            return self.sensor is not None
        bp = self.world.get_blueprint_library().find("sensor.camera.rgb")
        bp.set_attribute("image_size_x", str(SIDE_CAMERA_WIDTH))
        bp.set_attribute("image_size_y", str(SIDE_CAMERA_HEIGHT))
        bp.set_attribute("fov", str(SIDE_CAMERA_FOV))
        if bp.has_attribute("sensor_tick"):
            bp.set_attribute("sensor_tick", str(1.0 / max(1.0, SIDE_STREAM_FPS)))
        self.sensor = self.world.spawn_actor(
            bp,
            self.camera_mount(),
            attach_to=self.vehicle,
            attachment_type=carla.AttachmentType.Rigid,
        )
        self.sensor.listen(self._on_image)
        return True

    def _encode_image(self, rgb: np.ndarray) -> Tuple[bytes, str]:
        if Image is not None:
            out = io.BytesIO()
            Image.fromarray(rgb).save(out, format="JPEG", quality=72, optimize=True)
            return out.getvalue(), "image/jpeg"
        surface = pygame.surfarray.make_surface(rgb.swapaxes(0, 1))
        out = io.BytesIO()
        try:
            pygame.image.save(surface, out, "jpg")
            return out.getvalue(), "image/jpeg"
        except Exception:
            out = io.BytesIO()
            pygame.image.save(surface, out, "png")
            return out.getvalue(), "image/png"

    def _on_image(self, image: Any) -> None:
        if self.closed or (self.active_clients <= 0 and not SIDE_MIRRORS_ALWAYS_ON):
            return
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = array.reshape((image.height, image.width, 4))
        rgb = array[:, :, :3][:, :, ::-1]
        try:
            payload, content_type = self._encode_image(rgb)
        except Exception:
            return
        with self.lock:
            self.frame_id += 1
            self.frame_bytes = payload
            self.content_type = content_type
            self.lock.notify_all()

    def wait_frame(self, last_seen: int, timeout: float = 1.0) -> Tuple[int, Optional[bytes], str]:
        with self.lock:
            if self.frame_id == last_seen and not self.closed:
                self.lock.wait(timeout=timeout)
            if self.closed or self.frame_id == last_seen or self.frame_bytes is None:
                return last_seen, None, self.content_type
            return self.frame_id, self.frame_bytes, self.content_type

    def add_client(self) -> bool:
        with self.lock:
            self.active_clients += 1
        try:
            return self.spawn()
        except Exception:
            with self.lock:
                self.active_clients = max(0, self.active_clients - 1)
                if self.active_clients == 0:
                    self.frame_bytes = None
            return False

    def remove_client(self) -> None:
        should_destroy = False
        with self.lock:
            self.active_clients = max(0, self.active_clients - 1)
            if self.active_clients == 0 and not SIDE_MIRRORS_ALWAYS_ON:
                self.frame_bytes = None
                should_destroy = True
        if should_destroy:
            self.stop_sensor()

    def snapshot(self) -> Dict[str, Any]:
        tf = self.camera_mount()
        return {
            "frame_id": self.frame_id,
            "active_clients": self.active_clients,
            "width": SIDE_CAMERA_WIDTH,
            "height": SIDE_CAMERA_HEIGHT,
            "fps": SIDE_STREAM_FPS,
            "transform": {
                "x": round(tf.location.x, 3),
                "y": round(tf.location.y, 3),
                "z": round(tf.location.z, 3),
                "pitch": round(tf.rotation.pitch, 3),
                "yaw": round(tf.rotation.yaw, 3),
            },
        }

    def destroy(self) -> None:
        self.closed = True
        with self.lock:
            self.active_clients = 0
            self.frame_bytes = None
            self.lock.notify_all()
        self.stop_sensor()

    def stop_sensor(self) -> None:
        if self.sensor is not None:
            try:
                self.sensor.stop()
            except Exception:
                pass
            destroy_actor(self.sensor)
        self.sensor = None


class SideMirrorHandler(BaseHTTPRequestHandler):
    mirrors: Dict[str, SideMirrorCamera] = {}

    def log_message(self, _fmt: str, *_args: Any) -> None:
        return

    def do_GET(self) -> None:
        try:
            self.connection.settimeout(2.0)
        except Exception:
            pass
        path = self.path.split("?", 1)[0].strip("/")
        aliases = {
            "rear_left": "rear_left",
            "rear_left.mjpg": "rear_left",
            "left": "rear_left",
            "left.mjpg": "rear_left",
            "rear_right": "rear_right",
            "rear_right.mjpg": "rear_right",
            "right": "rear_right",
            "right.mjpg": "rear_right",
        }
        if path == "health":
            payload = {
                name: mirror.snapshot()
                for name, mirror in self.mirrors.items()
            }
            data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)
            return
        name = aliases.get(path)
        if not name or name not in self.mirrors:
            self.send_error(404, "side mirror stream not found")
            return

        last_seen = 0
        mirror = self.mirrors[name]
        if not mirror.add_client():
            self.send_error(503, "side mirror sensor unavailable")
            return
        self.send_response(200)
        self.send_header("Age", "0")
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Pragma", "no-cache")
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        try:
            while not SHUTDOWN_EVENT.is_set() and not mirror.closed:
                last_seen, frame, content_type = mirror.wait_frame(last_seen, timeout=1.0)
                if frame is None:
                    continue
                self.wfile.write(b"--frame\r\n")
                self.wfile.write(f"Content-Type: {content_type}\r\n".encode("ascii"))
                self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode("ascii"))
                self.wfile.write(frame)
                self.wfile.write(b"\r\n")
                self.wfile.flush()
        except Exception:
            return
        finally:
            mirror.remove_client()


class SideMirrorHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True


class SideMirrorStreams:
    def __init__(self, world: carla.World, vehicle: carla.Actor) -> None:
        self.mirrors = {
            "rear_left": SideMirrorCamera(world, vehicle, "rear_left"),
            "rear_right": SideMirrorCamera(world, vehicle, "rear_right"),
        }
        self.server: Optional[SideMirrorHTTPServer] = None
        self.thread: Optional[threading.Thread] = None

    def start(self) -> None:
        SideMirrorHandler.mirrors = self.mirrors
        if SIDE_MIRRORS_ALWAYS_ON:
            for mirror in self.mirrors.values():
                mirror.spawn()
        self.server = SideMirrorHTTPServer((SIDE_STREAM_HOST, SIDE_STREAM_PORT), SideMirrorHandler)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def snapshot(self) -> Dict[str, Any]:
        return {
            "port": SIDE_STREAM_PORT,
            "rear_left": self.mirrors["rear_left"].snapshot(),
            "rear_right": self.mirrors["rear_right"].snapshot(),
        }

    def destroy(self) -> None:
        if self.server is not None:
            try:
                self.server.shutdown()
                self.server.server_close()
            except Exception:
                pass
        self.server = None
        SideMirrorHandler.mirrors = {}
        for mirror in self.mirrors.values():
            mirror.destroy()


def build_sensor_frontend_summary(sensor_data: Dict[str, Any]) -> Dict[str, Any]:
    if not sensor_data or not sensor_data.get("enabled"):
        return {
            "enabled": False,
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
            "updated_at": sensor_data.get("updated_at") if sensor_data else None,
        }
    lidar = sensor_data.get("lidar") or {}
    radar = sensor_data.get("radar") or {}
    imu = sensor_data.get("imu") or {}
    gnss = sensor_data.get("gnss") or {}
    ultrasonic = sensor_data.get("ultrasonic") or {}

    distances: List[float] = []
    radar_nearest = radar.get("nearest_depth_m")
    lidar_nearest = lidar.get("range_min_m")
    for value in (radar_nearest, lidar_nearest):
        try:
            if value is not None:
                distances.append(float(value))
        except Exception:
            pass
    for item in (ultrasonic.get("distances") or {}).values():
        try:
            value = item.get("distance_m")
            if value is not None:
                distances.append(float(value))
        except Exception:
            pass

    accel = imu.get("accelerometer_m_s2") or []
    try:
        accel_norm = round(math.sqrt(sum(float(v) * float(v) for v in accel[:3])), 3)
    except Exception:
        accel_norm = None
    try:
        compass_deg = round(math.degrees(float(imu.get("compass_rad"))), 2)
    except Exception:
        compass_deg = None

    return {
        "enabled": True,
        "lidar_points": lidar.get("point_count"),
        "radar_targets": radar.get("detection_count"),
        "nearest_obstacle_m": round(min(distances), 3) if distances else None,
        "imu_accel_norm_m_s2": accel_norm,
        "heading_deg": compass_deg,
        "gnss": {
            "latitude": gnss.get("latitude"),
            "longitude": gnss.get("longitude"),
            "altitude_m": gnss.get("altitude_m"),
        },
        "updated_at": sensor_data.get("updated_at"),
    }


class SensorSuite:
    def __init__(self, world: carla.World, vehicle: carla.Actor, vehicle_model: str, mode: str) -> None:
        self.world = world
        self.vehicle = vehicle
        self.vehicle_model = vehicle_model
        self.mode = mode
        self.enabled = self._should_enable()
        self.started_at = time.time()
        self.updated_at = self.started_at
        self.actors: List[carla.Actor] = []
        self.lock = threading.Lock()
        self.data: Dict[str, Any] = {
            "enabled": self.enabled,
            "vehicle_model": vehicle_model,
            "started_at": self.started_at,
            "updated_at": self.updated_at,
            "sensors": {},
            "last_error": None,
            "summary_hz": SENSOR_SUMMARY_HZ,
            "physics_fixed_delta_seconds": FIXED_DELTA_SECONDS,
        }
        self.sensor_configs = self._load_sensor_configs()
        self.udp_ports = parse_port_list(SENSOR_UDP_PORTS)
        self.udp_sock: Optional[socket.socket] = None
        self.tx_count = 0
        self.last_udp_tx = 0.0
        if self.udp_ports:
            self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def _load_sensor_configs(self) -> Dict[str, Dict[str, Any]]:
        sensors_dir = PROJECT_ROOT / "sensors"
        configs: Dict[str, Dict[str, Any]] = {}
        for key, filename in {
            "lidar": "lidar_64_main_roof.json",
            "radar": "radar_mmw_front_long.json",
            "gnss_imu": "gnss_imu_combined.json",
            "ultrasonic": "ultrasonic_array_surround.json",
        }.items():
            path = sensors_dir / filename
            try:
                configs[key] = json.loads(path.read_text(encoding="utf-8"))
            except Exception as exc:
                configs[key] = {}
                self._record_error(f"sensor config {filename}: {exc}")
        return configs

    @staticmethod
    def _sensor_tick_from_hz(hz: Any, fallback_hz: float) -> float:
        try:
            requested = 1.0 / max(0.1, float(hz))
        except Exception:
            requested = 1.0 / max(0.1, fallback_hz)
        return max(requested, FIXED_DELTA_SECONDS)

    def _should_enable(self) -> bool:
        if not ENABLE_SENSOR_SUITE:
            return False
        if self.mode == "AIGO" and DISABLE_AIGO_SENSORS:
            return False
        if self.mode == "AI" and DISABLE_AI_SENSORS:
            return False
        return True

    def _mark_sensor(self, sensor_id: str, ok: bool, blueprint: str, detail: Optional[str] = None) -> None:
        with self.lock:
            self.data.setdefault("sensors", {})[sensor_id] = {
                "ok": ok,
                "blueprint": blueprint,
                "detail": detail,
                "updated_at": time.time(),
            }
            self.data["updated_at"] = time.time()

    def _record_error(self, message: str) -> None:
        with self.lock:
            self.data["last_error"] = message
            self.data["updated_at"] = time.time()
        print(f"sensor suite warning: {message}", flush=True)

    def start(self) -> None:
        if not self.enabled:
            with self.lock:
                self.data["reason"] = f"disabled for mode {self.mode}"
            return
        bp_lib = self.world.get_blueprint_library()
        self._spawn_lidar(bp_lib)
        self._spawn_radar(bp_lib)
        self._spawn_gnss(bp_lib)
        self._spawn_imu(bp_lib)
        self._spawn_ultrasonic(bp_lib)

    def _spawn_actor(self, sensor_id: str, bp: carla.ActorBlueprint, tf: carla.Transform, callback: Any) -> None:
        try:
            actor = self.world.spawn_actor(bp, tf, attach_to=self.vehicle)
            actor.listen(callback)
            self.actors.append(actor)
            self._mark_sensor(sensor_id, True, bp.id)
        except Exception as exc:
            self._mark_sensor(sensor_id, False, getattr(bp, "id", "unknown"), str(exc))
            self._record_error(f"{sensor_id}: {exc}")

    def _spawn_lidar(self, bp_lib: carla.BlueprintLibrary) -> None:
        cfg = self.sensor_configs.get("lidar", {})
        hw = cfg.get("hardware_specifications", {}) or {}
        vfov = cfg.get("vertical_fov_distribution", {}) or {}
        noise = cfg.get("noise_and_physics_model", {}) or {}
        bp = bp_lib.find("sensor.lidar.ray_cast")
        pps = min(max(1000, int(hw.get("points_per_second", SENSOR_LIDAR_MAX_PPS) or SENSOR_LIDAR_MAX_PPS)), SENSOR_LIDAR_MAX_PPS)
        hz = min(max(1.0, float(hw.get("rotation_frequency_hz", SENSOR_LIDAR_MAX_HZ) or SENSOR_LIDAR_MAX_HZ)), SENSOR_LIDAR_MAX_HZ)
        for name, value in {
            "channels": str(int(hw.get("channels", 64) or 64)),
            "points_per_second": str(pps),
            "rotation_frequency": str(hz),
            "range": str(float(hw.get("range_m", 80.0) or 80.0)),
            "upper_fov": str(float(vfov.get("upper_fov_deg", 15.0) or 15.0)),
            "lower_fov": str(float(vfov.get("lower_fov_deg", -25.0) or -25.0)),
            "noise_stddev": str(float(noise.get("noise_stddev_m", 0.0) or 0.0)),
            "dropoff_general_rate": str(float(noise.get("dropoff_general_rate", 0.0) or 0.0)),
            "sensor_tick": str(self._sensor_tick_from_hz(hz, SENSOR_LIDAR_MAX_HZ)),
        }.items():
            if bp.has_attribute(name):
                bp.set_attribute(name, value)
        tf = carla.Transform(carla.Location(x=0.0, y=0.0, z=2.35))
        self._spawn_actor("lidar_64_main_roof", bp, tf, self._on_lidar)

    def _spawn_radar(self, bp_lib: carla.BlueprintLibrary) -> None:
        cfg = self.sensor_configs.get("radar", {})
        hw = cfg.get("hardware_specifications", {}) or {}
        bp = bp_lib.find("sensor.other.radar")
        for name, value in {
            "horizontal_fov": str(float(hw.get("horizontal_fov_deg", 30.0) or 30.0)),
            "vertical_fov": str(float(hw.get("vertical_fov_deg", 12.0) or 12.0)),
            "range": str(float(hw.get("range_m", 80.0) or 80.0)),
            "sensor_tick": str(max(float(hw.get("sensor_tick_sec", 0.1) or 0.1), FIXED_DELTA_SECONDS)),
        }.items():
            if bp.has_attribute(name):
                bp.set_attribute(name, value)
        tf = carla.Transform(carla.Location(x=2.5, y=0.0, z=1.0), carla.Rotation(pitch=0.0))
        self._spawn_actor("radar_mmw_front", bp, tf, self._on_radar)

    def _spawn_gnss(self, bp_lib: carla.BlueprintLibrary) -> None:
        cfg = self.sensor_configs.get("gnss_imu", {})
        spec = cfg.get("gnss_specifications", {}) or {}
        bp = bp_lib.find("sensor.other.gnss")
        if bp.has_attribute("sensor_tick"):
            bp.set_attribute("sensor_tick", str(self._sensor_tick_from_hz(spec.get("update_rate_hz", 10.0), 10.0)))
        self._spawn_actor("gnss", bp, carla.Transform(), self._on_gnss)

    def _spawn_imu(self, bp_lib: carla.BlueprintLibrary) -> None:
        cfg = self.sensor_configs.get("gnss_imu", {})
        spec = cfg.get("imu_specifications", {}) or {}
        bp = bp_lib.find("sensor.other.imu")
        if bp.has_attribute("sensor_tick"):
            bp.set_attribute("sensor_tick", str(self._sensor_tick_from_hz(spec.get("update_rate_hz", 100.0), 100.0)))
        self._spawn_actor("imu", bp, carla.Transform(), self._on_imu)

    def _spawn_ultrasonic(self, bp_lib: carla.BlueprintLibrary) -> None:
        cfg = self.sensor_configs.get("ultrasonic", {})
        hw = cfg.get("hardware_specifications", {}) or {}
        bp = bp_lib.find("sensor.other.obstacle")
        for name, value in {
            "distance": str(float(hw.get("range_m", 5.0) or 5.0)),
            "hit_radius": "0.35",
            "sensor_tick": str(max(float(hw.get("sensor_tick_sec", 0.1) or 0.1), FIXED_DELTA_SECONDS)),
        }.items():
            if bp.has_attribute(name):
                bp.set_attribute(name, value)
        if bp.has_attribute("only_dynamics"):
            bp.set_attribute("only_dynamics", "false")
        mounts = {
            "fl": carla.Transform(carla.Location(x=2.4, y=-0.65, z=0.55), carla.Rotation(yaw=0.0)),
            "fr": carla.Transform(carla.Location(x=2.4, y=0.65, z=0.55), carla.Rotation(yaw=0.0)),
            "rl": carla.Transform(carla.Location(x=-2.2, y=-0.65, z=0.55), carla.Rotation(yaw=180.0)),
            "rr": carla.Transform(carla.Location(x=-2.2, y=0.65, z=0.55), carla.Rotation(yaw=180.0)),
        }
        with self.lock:
            self.data["ultrasonic"] = {
                "sensor_id": "ultrasonic_array",
                "type": "sensor.other.obstacle",
                "distances": {
                    key: {"id": key, "distance_m": None, "other_actor_id": None, "updated_at": time.time()}
                    for key in mounts
                },
                "updated_at": time.time(),
            }
        for key, tf in mounts.items():
            self._spawn_actor(f"ultrasonic_{key}", bp, tf, lambda event, sensor_key=key: self._on_obstacle(sensor_key, event))

    def _on_lidar(self, data: Any) -> None:
        try:
            points = np.frombuffer(data.raw_data, dtype=np.float32).reshape((-1, 4))
            summary = {
                "sensor_id": "lidar_64_main_roof",
                "type": "sensor.lidar.ray_cast",
                "frame": int(data.frame),
                "point_count": int(points.shape[0]),
                "range_min_m": round(float(np.linalg.norm(points[:, :3], axis=1).min()), 3) if points.size else None,
                "range_max_m": round(float(np.linalg.norm(points[:, :3], axis=1).max()), 3) if points.size else None,
                "updated_at": time.time(),
            }
            with self.lock:
                self.data["lidar"] = summary
                self.data["updated_at"] = summary["updated_at"]
        except Exception as exc:
            self._record_error(f"lidar callback: {exc}")

    def _on_radar(self, data: Any) -> None:
        try:
            detections = list(data)
            nearest = min((float(d.depth) for d in detections), default=None)
            fastest = max((abs(float(d.velocity)) for d in detections), default=None)
            summary = {
                "sensor_id": "radar_mmw_front",
                "type": "sensor.other.radar",
                "frame": int(data.frame),
                "detection_count": len(detections),
                "nearest_depth_m": round(nearest, 3) if nearest is not None else None,
                "max_abs_velocity_m_s": round(fastest, 3) if fastest is not None else None,
                "updated_at": time.time(),
            }
            with self.lock:
                self.data["radar"] = summary
                self.data["updated_at"] = summary["updated_at"]
        except Exception as exc:
            self._record_error(f"radar callback: {exc}")

    def _on_gnss(self, data: Any) -> None:
        summary = {
            "sensor_id": "gnss",
            "type": "sensor.other.gnss",
            "frame": int(data.frame),
            "latitude": round(float(data.latitude), 7),
            "longitude": round(float(data.longitude), 7),
            "altitude_m": round(float(data.altitude), 3),
            "updated_at": time.time(),
        }
        with self.lock:
            self.data["gnss"] = summary
            self.data["updated_at"] = summary["updated_at"]

    def _on_imu(self, data: Any) -> None:
        summary = {
            "sensor_id": "imu",
            "type": "sensor.other.imu",
            "frame": int(data.frame),
            "accelerometer_m_s2": [
                round(float(data.accelerometer.x), 4),
                round(float(data.accelerometer.y), 4),
                round(float(data.accelerometer.z), 4),
            ],
            "gyroscope_rad_s": [
                round(float(data.gyroscope.x), 5),
                round(float(data.gyroscope.y), 5),
                round(float(data.gyroscope.z), 5),
            ],
            "compass_rad": round(float(data.compass), 5),
            "updated_at": time.time(),
        }
        with self.lock:
            self.data["imu"] = summary
            self.data["updated_at"] = summary["updated_at"]

    def _on_obstacle(self, key: str, event: Any) -> None:
        with self.lock:
            ultra = self.data.setdefault(
                "ultrasonic",
                {"sensor_id": "ultrasonic_array", "type": "sensor.other.obstacle", "distances": {}},
            )
            ultra.setdefault("distances", {})[key] = {
                "id": key,
                "distance_m": round(float(getattr(event, "distance", 0.0)), 3),
                "other_actor_id": getattr(getattr(event, "other_actor", None), "id", None),
                "updated_at": time.time(),
            }
            ultra["updated_at"] = time.time()
            self.data["updated_at"] = ultra["updated_at"]

    def snapshot(self) -> Dict[str, Any]:
        with self.lock:
            snap = json.loads(json.dumps(self.data, ensure_ascii=False))
        try:
            snap["actor_count"] = len(self.world.get_actors())
        except Exception:
            snap["actor_count"] = None
        snap["sensor_udp_ports"] = self.udp_ports
        snap["sensor_tx_count"] = self.tx_count
        snap["frontend_summary"] = build_sensor_frontend_summary(snap)
        return snap

    def maybe_send_udp(self) -> None:
        if not self.enabled or not self.udp_sock or not self.udp_ports:
            return
        now = time.time()
        if now - self.last_udp_tx < 1.0 / max(0.1, SENSOR_UDP_HZ):
            return
        payload = json.dumps(self.snapshot(), ensure_ascii=False, separators=(",", ":")).encode("utf-8")
        for port in self.udp_ports:
            if not safe_udp_send(self.udp_sock, payload, ("127.0.0.1", port), "sensor"):
                self._record_error(f"sensor udp {port}: payload_len={len(payload)}")
        self.tx_count += 1
        self.last_udp_tx = now

    def destroy(self) -> None:
        for actor in self.actors:
            destroy_actor(actor)
        self.actors = []
        if self.udp_sock is not None:
            try:
                self.udp_sock.close()
            except Exception:
                pass
            self.udp_sock = None


def bind_control_socket(port: int) -> socket.socket:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    if hasattr(socket, "SO_REUSEPORT"):
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except Exception:
            pass
    sock.bind(("0.0.0.0", port))
    sock.setblocking(False)
    return sock


def configure_ai(vehicle: carla.Actor, client: carla.Client) -> None:
    try:
        try:
            vehicle.disable_constant_velocity()
        except Exception:
            pass
        tm = client.get_trafficmanager(TRAFFIC_MANAGER_PORT)
        tm.set_synchronous_mode(False)
        tm.ignore_lights_percentage(vehicle, AI_IGNORE_LIGHTS_PERCENT)
        tm.ignore_signs_percentage(vehicle, AI_IGNORE_SIGNS_PERCENT)
        tm.vehicle_percentage_speed_difference(vehicle, AI_SPEED_DIFFERENCE_PERCENT)
        vehicle.set_autopilot(True, TRAFFIC_MANAGER_PORT)
    except TypeError:
        vehicle.set_autopilot(True)
    except Exception as exc:
        print(f"AI setup failed: {exc}", flush=True)


def blocked_by_red_light(vehicle: carla.Actor) -> bool:
    try:
        if not vehicle.is_at_traffic_light():
            return False
        light = vehicle.get_traffic_light()
        if light is None:
            return False
        return light.get_state() == carla.TrafficLightState.Red
    except Exception:
        return False


def spawn_traffic(world: carla.World, client: carla.Client, ego: carla.Actor, count: int = TRAFFIC_COUNT) -> List[carla.Actor]:
    if count <= 0:
        return []
    try:
        tm = client.get_trafficmanager(TRAFFIC_MANAGER_PORT)
        tm.set_synchronous_mode(False)
        tm.global_percentage_speed_difference(10.0)
    except Exception:
        pass

    bp_lib = world.get_blueprint_library()
    blueprints = list(bp_lib.filter("vehicle.*"))
    spawn_points = list(world.get_map().get_spawn_points())
    random.shuffle(spawn_points)
    actors: List[carla.Actor] = []
    ego_loc = ego.get_location()
    for sp in spawn_points:
        if len(actors) >= count:
            break
        try:
            if sp.location.distance(ego_loc) < 25.0:
                continue
        except Exception:
            pass
        bp = random.choice(blueprints)
        if bp.has_attribute("role_name"):
            bp.set_attribute("role_name", "normal_vehicle")
        if bp.has_attribute("color"):
            colors = bp.get_attribute("color").recommended_values
            if colors:
                bp.set_attribute("color", random.choice(colors))
        try:
            actor = world.try_spawn_actor(bp, sp)
            if actor:
                actors.append(actor)
                try:
                    actor.set_autopilot(True, TRAFFIC_MANAGER_PORT)
                except TypeError:
                    actor.set_autopilot(True)
        except Exception:
            continue
    return actors


def vehicle_max_steer_deg(vehicle: carla.Actor, vehicle_model: str) -> float:
    geom = GEOMETRY.get(vehicle_model, GEOMETRY["Lincoln MKZ"])
    fallback = float(geom.get("max_steer_deg", 40.0))
    actor_id = int(getattr(vehicle, "id", -1) or -1)
    if actor_id in MAX_STEER_CACHE:
        return MAX_STEER_CACHE[actor_id]
    try:
        physics = vehicle.get_physics_control()
        angles = [
            float(getattr(wheel, "max_steer_angle", 0.0) or 0.0)
            for wheel in getattr(physics, "wheels", [])[:2]
        ]
        angles = [angle for angle in angles if angle > 0.0]
        if angles:
            max_angle = max(angles)
            if actor_id >= 0:
                MAX_STEER_CACHE[actor_id] = max_angle
            return max_angle
    except Exception:
        pass
    if actor_id >= 0:
        MAX_STEER_CACHE[actor_id] = fallback
    return fallback


def build_telemetry(vehicle: carla.Actor, vehicle_model: str) -> Dict[str, Any]:
    tf = vehicle.get_transform()
    vel = vehicle.get_velocity()
    acc = vehicle.get_acceleration()
    ang = vehicle.get_angular_velocity()
    ctrl = vehicle.get_control()
    geom = GEOMETRY.get(vehicle_model, GEOMETRY["Lincoln MKZ"])
    max_steer_deg = vehicle_max_steer_deg(vehicle, vehicle_model)
    speed_ms = math.sqrt(vel.x * vel.x + vel.y * vel.y + vel.z * vel.z)
    rpm = speed_ms / max(geom["tire_radius"], 0.05) * 60.0 / (2.0 * math.pi)
    mass = geom["mass"]
    cf = -110000.0 * mass / 1500.0
    cr = -95000.0 * mass / 1500.0
    return {
        "1_刚体运动学 (Rigid Body Kinematics)": {
            "1_全局绝对坐标_XYZ_米": [round(tf.location.x, 3), round(tf.location.y, 3), round(tf.location.z, 3)],
            "2_姿态角_俯仰_偏航_滚转_度": [round(tf.rotation.pitch, 3), round(tf.rotation.yaw, 3), round(tf.rotation.roll, 3)],
            "3_线速度矢量_XYZ_米每秒": [round(vel.x, 3), round(vel.y, 3), round(vel.z, 3)],
            "4_线加速度_XYZ_米每平方秒": [round(acc.x, 3), round(acc.y, 3), round(acc.z, 3)],
            "5_角速度_XYZ_度每秒": [round(ang.x, 3), round(ang.y, 3), round(ang.z, 3)],
        },
        "2_轮端与底盘动态 (Wheel Dynamics)": {
            "6_四轮独立转速_RPM_左前_右前_左后_右后": [round(rpm, 1), round(rpm, 1), round(rpm, 1), round(rpm, 1)],
            "7_悬架实时压缩量_毫米_左前_右前_左后_右后": [0.0, 0.0, 0.0, 0.0],
            "8_前轮真实阿克曼转向角_度": [round(float(ctrl.steer) * max_steer_deg, 2), round(float(ctrl.steer) * max_steer_deg, 2)],
        },
        "3_驾驶控制反读 (Control State)": {
            "9_实际油门开度_0至1": round(float(ctrl.throttle), 3),
            "10_实际刹车力度_0至1": round(float(ctrl.brake), 3),
            "11_方向盘转角_负1至1": round(float(ctrl.steer), 3),
            "12_当前机械档位": int(ctrl.gear),
            "13_手刹激活状态": bool(ctrl.hand_brake),
            "14_倒车挂档状态": bool(ctrl.reverse),
        },
        "6_动态车辆参数": {
            "整备质量": mass,
            "前轮侧偏刚度_Cf": cf,
            "后轮侧偏刚度_Cr": cr,
            "轮距_L": geom["track"],
            "轴距_L": geom["wheelbase"],
            "a": round(geom["wheelbase"] * 0.517, 3),
            "b": round(geom["wheelbase"] * 0.483, 3),
            "Cf": cf,
            "Cr": cr,
            "最大前轮转角_deg": max_steer_deg,
            "max_steer_deg": max_steer_deg,
        },
    }


def build_frontend_telemetry(legacy: Dict[str, Any], vehicle: carla.Actor, vehicle_model: str) -> Dict[str, str]:
    tf = vehicle.get_transform()
    ctrl = vehicle.get_control()
    geom = GEOMETRY.get(vehicle_model, GEOMETRY["Lincoln MKZ"])
    max_steer_deg = vehicle_max_steer_deg(vehicle, vehicle_model)
    spec = FRONTEND_GEOMETRY.get(vehicle_model, FRONTEND_GEOMETRY["Lincoln MKZ"])
    rigid = nested_find(legacy, "刚体运动学", {}) or {}
    wheel_dyn = nested_find(legacy, "轮端与底盘动态", {}) or {}
    control_state = nested_find(legacy, "驾驶控制反读", {}) or {}
    dyn = nested_find(legacy, "动态车辆参数", {}) or {}

    pos_xyz = nested_find(rigid, "全局绝对坐标", [tf.location.x, tf.location.y, tf.location.z]) or [0.0, 0.0, 0.0]
    angle_pyr = nested_find(rigid, "姿态角", [tf.rotation.pitch, tf.rotation.yaw, tf.rotation.roll]) or [0.0, 0.0, 0.0]
    wheel_rpm = nested_find(wheel_dyn, "四轮独立转速", [0.0, 0.0, 0.0, 0.0]) or [0.0, 0.0, 0.0, 0.0]
    wheel_bounce = nested_find(wheel_dyn, "悬架实时压缩量", [0.0, 0.0, 0.0, 0.0]) or [0.0, 0.0, 0.0, 0.0]
    steer_pair = nested_find(wheel_dyn, "前轮真实阿克曼", [float(ctrl.steer) * max_steer_deg, float(ctrl.steer) * max_steer_deg]) or [0.0, 0.0]
    throttle = float(nested_find(control_state, "实际油门", float(ctrl.throttle)) or 0.0)
    steer = float(nested_find(control_state, "方向盘转角", float(ctrl.steer)) or 0.0)

    mass = float(dyn.get("整备质量", geom["mass"]) or geom["mass"])
    wheel_rpm = (list(wheel_rpm) + [0.0, 0.0, 0.0, 0.0])[:4]
    wheel_bounce = (list(wheel_bounce) + [0.0, 0.0, 0.0, 0.0])[:4]
    steer_pair = (list(steer_pair) + [0.0, 0.0])[:2]
    pos_xyz = (list(pos_xyz) + [0.0, 0.0, 0.0])[:3]
    angle_pyr = (list(angle_pyr) + [0.0, 0.0, 0.0])[:3]
    static_load = mass * 9.81 / 4.0
    deformation = [max(0.0, abs(float(v)) * 0.01) for v in wheel_bounce]
    slip_front = max(0.0, throttle * 4.0)
    slip_rear = max(0.0, throttle * 2.0)

    payload: Dict[str, Any] = {
        "Vehiclemodel": vehicle_model,
        "Overall": spec["Overall"],
        "Wheelbase": spec["Wheelbase"],
        "Tirebase": spec["Tirebase"],
        "Tireradius": spec["Tireradius"],
        "Empty": fmt_frontend(mass, 0, "kg"),
        "Gravity": "(-0.05, 0, 0.28)",
        "Axle": fmt_frontend(mass * 0.87, 0, "kg"),
        "Unloaded": fmt_frontend(mass * 0.13, 0, "kg"),
        "Moment": f"({fmt_frontend(mass * 0.35, 0)}, {fmt_frontend(mass * 1.65, 0)}, {fmt_frontend(mass * 1.75, 0)})",
        "Ratedtotalmass": fmt_frontend(mass * 1.23, 0, "kg"),
        "Drag": "0.3",
        "Windward": fmt_frontend(max(2.1, geom["track"] * 1.4), 2, "m2"),
        "LiftCoefficientCl": "0.08",
        "Pitching": "0.015",
        "Latera": "0.5",
        "LeftFrontWheelRotation": fmt_frontend(wheel_rpm[0], 1, "RPM"),
        "RightFrontWheelRotation": fmt_frontend(wheel_rpm[1], 1, "RPM"),
        "LeftRearWheelRotation": fmt_frontend(wheel_rpm[2], 1, "RPM"),
        "RightRearWheelRotation": fmt_frontend(wheel_rpm[3], 1, "RPM"),
        "LeftFrontWheelVibration": fmt_frontend(static_load + float(wheel_bounce[0]), 0, "N"),
        "RightFrontWheelVibration": fmt_frontend(static_load + float(wheel_bounce[1]), 0, "N"),
        "LeftRearWheelVibration": fmt_frontend(static_load + float(wheel_bounce[2]), 0, "N"),
        "RightRearWheelVibration": fmt_frontend(static_load + float(wheel_bounce[3]), 0, "N"),
        "LeftFrontWheel": fmt_frontend(slip_front, 2),
        "RightFrontWheel": fmt_frontend(slip_front, 2),
        "LeftRearWheel": fmt_frontend(slip_rear, 2),
        "RightRearWheel": fmt_frontend(slip_rear, 2),
        "RadialDeformationLeftFrontWheel": fmt_frontend(deformation[0], 2, "mm"),
        "RadialDeformationRightFrontWheel": fmt_frontend(deformation[1], 2, "mm"),
        "RadialDeformationLeftRearWheel": fmt_frontend(deformation[2], 2, "mm"),
        "RadialDeformationRightRearWheel": fmt_frontend(deformation[3], 2, "mm"),
        "PitchAngle": fmt_frontend(angle_pyr[0], 2, "deg"),
        "RollAngle": fmt_frontend(angle_pyr[2], 2, "deg"),
        "LateralSwingAngle": fmt_frontend(angle_pyr[1], 2, "deg"),
        "LongitudinalDisplacement": fmt_frontend(pos_xyz[0], 2, "m"),
        "LateralDisplacement": fmt_frontend(pos_xyz[1], 2, "m"),
        "VerticalDisplacement": fmt_frontend(pos_xyz[2], 2, "m"),
        "TurningTheSteeringWheel": fmt_frontend(steer * 540.0, 2, "deg"),
        "LeftFrontWheelAngle": fmt_frontend(steer_pair[0], 2, "deg"),
        "RightFrontWheelAngle": fmt_frontend(steer_pair[1], 2, "deg"),
        "SteeringColumnTorsion": fmt_frontend(abs(steer) * 2.2, 2, "deg"),
        "EngineCrankshaftRotates": fmt_frontend(900.0 + abs(throttle) * 3600.0, 0, "RPM"),
        "LeftDriveHalfShaftTwist": fmt_frontend(abs(throttle) * 1.1, 2, "deg"),
        "RightDriveHalfShaftTwist": fmt_frontend(abs(throttle) * 1.1, 2, "deg"),
    }
    return normalize_frontend_payload(payload)


def apply_udp_control(vehicle: carla.Actor, packet: bytes) -> Optional[Dict[str, Any]]:
    data = json.loads(packet.decode("utf-8"))
    reverse = bool(data.get("reverse", False))
    command = {
        "throttle": clamp(data.get("throttle", 0.0), 0.0, 1.0),
        "steer": clamp(data.get("steer", 0.0), -1.0, 1.0),
        "brake": clamp(data.get("brake", 0.0), 0.0, 1.0),
        "reverse": reverse,
        "hand_brake": bool(data.get("hand_brake", False)),
    }
    vehicle.apply_control(
        carla.VehicleControl(
            throttle=command["throttle"],
            steer=command["steer"],
            brake=command["brake"],
            reverse=reverse,
            hand_brake=command["hand_brake"],
            manual_gear_shift=False,
            gear=-1 if reverse else 1,
        )
    )
    return command


ROUTE_LEGS = {
    "AB": ("A", "B"),
    "BC": ("B", "C"),
    "CA": ("C", "A"),
}


def normalize_route_segment(value: Any) -> str:
    text = str(value or "").strip().upper().replace("-", "").replace("_", "").replace(" ", "")
    if text in {"", "0", "NONE", "OFF", "FALSE", "NO"}:
        return ""
    if text in {"AB", "A2B", "ATOB"}:
        return "AB"
    if text in {"BC", "B2C", "BTOC"}:
        return "BC"
    if text in {"CA", "C2A", "CTOA"}:
        return "CA"
    if text in {"LOOP", "ABC", "ABCA", "CYCLE", "ALL"}:
        return "LOOP"
    return ""


def project_to_road_transform(world: carla.World, transform: carla.Transform) -> carla.Transform:
    try:
        waypoint = world.get_map().get_waypoint(
            transform.location,
            project_to_road=True,
            lane_type=carla.LaneType.Driving,
        )
        if waypoint:
            road_tf = waypoint.transform
            road_tf.location.z = max(float(road_tf.location.z), float(transform.location.z))
            return road_tf
    except Exception:
        pass
    return transform


def route_transform_dict(transform: carla.Transform) -> Dict[str, float]:
    return {
        "x": round(float(transform.location.x), 3),
        "y": round(float(transform.location.y), 3),
        "z": round(float(transform.location.z), 3),
        "yaw": round(float(transform.rotation.yaw), 3),
    }


def smooth_route_control(control: carla.VehicleControl, previous_steer: float, dt: float) -> Tuple[carla.VehicleControl, float]:
    raw_steer = clamp(getattr(control, "steer", 0.0), -1.0, 1.0)
    limited = clamp(raw_steer, -ROUTE_STEER_LIMIT, ROUTE_STEER_LIMIT)
    max_delta = ROUTE_STEER_RATE_LIMIT * max(0.01, dt)
    steer = clamp(limited, previous_steer - max_delta, previous_steer + max_delta)
    control.steer = steer
    abs_steer = abs(steer)
    if abs_steer >= ROUTE_TURN_THROTTLE_STEER:
        scale = max(0.0, 1.0 - abs_steer / max(ROUTE_STEER_LIMIT, 0.01))
        control.throttle = min(float(getattr(control, "throttle", 0.0)), 0.25 * scale)
    if abs_steer >= ROUTE_TURN_BRAKE_STEER:
        denom = max(ROUTE_STEER_LIMIT - ROUTE_TURN_BRAKE_STEER, 0.01)
        brake = ROUTE_TURN_MAX_BRAKE * min(1.0, (abs_steer - ROUTE_TURN_BRAKE_STEER) / denom)
        control.brake = max(float(getattr(control, "brake", 0.0)), brake)
    return control, steer


def route_trace(planner: Any, start: carla.Transform, end: carla.Transform) -> List[Any]:
    if planner is None:
        return []
    try:
        return list(planner.trace_route(start.location, end.location))
    except Exception:
        return []


def route_waypoints(trace: List[Any]) -> List[Any]:
    waypoints = []
    for item in trace:
        try:
            waypoint = item[0]
        except Exception:
            waypoint = item
        if waypoint is not None:
            waypoints.append(waypoint)
    return waypoints


def route_length_m(trace: List[Any]) -> float:
    waypoints = route_waypoints(trace)
    total = 0.0
    for prev, curr in zip(waypoints, waypoints[1:]):
        try:
            total += float(prev.transform.location.distance(curr.transform.location))
        except Exception:
            pass
    return total


def angle_delta_deg(a: float, b: float) -> float:
    return abs((b - a + 180.0) % 360.0 - 180.0)


def route_turn_penalty(trace: List[Any]) -> float:
    waypoints = route_waypoints(trace)
    if len(waypoints) < 3:
        return 0.0
    penalty = 0.0
    last_yaw = float(waypoints[0].transform.rotation.yaw)
    for waypoint in waypoints[1:]:
        yaw = float(waypoint.transform.rotation.yaw)
        delta = angle_delta_deg(last_yaw, yaw)
        if delta > ROUTE_TURN_PENALTY_THRESHOLD_DEG:
            penalty += (delta - ROUTE_TURN_PENALTY_THRESHOLD_DEG) * ROUTE_TURN_PENALTY_WEIGHT
        last_yaw = yaw
    return penalty


def route_cells(trace: List[Any]) -> set:
    size = max(1.0, ROUTE_SHARED_ROUTE_CELL_SIZE_M)
    cells = set()
    for waypoint in route_waypoints(trace):
        loc = waypoint.transform.location
        cells.add((round(float(loc.x) / size), round(float(loc.y) / size)))
    return cells


def trace_to_locations(trace: List[Any]) -> List[carla.Location]:
    locations = []
    last_key = None
    for waypoint in route_waypoints(trace):
        loc = waypoint.transform.location
        key = (round(float(loc.x), 2), round(float(loc.y), 2), round(float(loc.z), 2))
        if key == last_key:
            continue
        locations.append(carla.Location(float(loc.x), float(loc.y), float(loc.z)))
        last_key = key
    return locations


def route_path_summary(locations: List[carla.Location]) -> Dict[str, Any]:
    items = [
        (round(float(loc.x), 2), round(float(loc.y), 2), round(float(loc.z), 2))
        for loc in locations
    ]
    raw = json.dumps(items, separators=(",", ":")).encode("utf-8")
    return {
        "hash": hashlib.sha1(raw).hexdigest()[:16],
        "waypoint_count": len(items),
    }


def trace_passes_near(trace: List[Any], transform: carla.Transform, radius_m: float) -> bool:
    loc = transform.location
    for waypoint in route_waypoints(trace):
        try:
            if waypoint.transform.location.distance(loc) <= radius_m:
                return True
        except Exception:
            pass
    return False


def trace_has_repeated_lane(trace: List[Any], max_repeat: int = 8) -> bool:
    seen = set()
    reentries: Dict[Tuple[int, int, int], int] = {}
    last_key = None
    for waypoint in route_waypoints(trace):
        try:
            key = (int(waypoint.road_id), int(waypoint.section_id), int(waypoint.lane_id))
        except Exception:
            continue
        if key == last_key:
            continue
        if key in seen:
            reentries[key] = reentries.get(key, 0) + 1
            if reentries[key] > max_repeat:
                return True
        seen.add(key)
        last_key = key
    return False


def soften_route_agent(agent: Any) -> None:
    local_planner = getattr(agent, "_local_planner", None)
    if local_planner is None:
        return
    try:
        local_planner.set_speed(ROUTE_TARGET_SPEED_KMH)
    except Exception:
        pass
    try:
        local_planner._sampling_radius = ROUTE_LOCAL_SAMPLING_RADIUS
        local_planner._base_min_distance = ROUTE_LOCAL_BASE_MIN_DISTANCE
    except Exception:
        pass
    try:
        local_planner._args_lateral_dict = {
            "K_P": ROUTE_LATERAL_KP,
            "K_D": ROUTE_LATERAL_KD,
            "K_I": ROUTE_LATERAL_KI,
            "dt": FIXED_DELTA_SECONDS,
        }
    except Exception:
        pass
    try:
        controller = getattr(local_planner, "_vehicle_controller", None)
        if controller is not None:
            controller.max_steering = ROUTE_STEER_LIMIT
            lat_controller = getattr(controller, "_lat_controller", None)
            if lat_controller is not None:
                lat_controller._k_p = ROUTE_LATERAL_KP
                lat_controller._k_d = ROUTE_LATERAL_KD
                lat_controller._k_i = ROUTE_LATERAL_KI
                lat_controller._dt = FIXED_DELTA_SECONDS
    except Exception:
        pass


def configure_route_tm(vehicle: carla.Actor, client: carla.Client, route_plan: Dict[str, Any]) -> Tuple[bool, str]:
    if not ROUTE_TM_ENABLED:
        return False, "Traffic Manager route mode disabled by OFFICIAL_ROUTE_TM_ENABLED"
    first_leg = "".join(route_plan["legs"][0])
    path = list((route_plan.get("path_locations") or {}).get(first_leg) or [])
    if len(path) < 2:
        return False, f"route path {first_leg} has too few waypoints"
    try:
        tm = client.get_trafficmanager(TRAFFIC_MANAGER_PORT)
        try:
            tm.set_synchronous_mode(False)
        except Exception:
            pass
        try:
            tm.set_random_device_seed(ROUTE_TM_SEED)
        except Exception:
            pass
        try:
            tm.auto_lane_change(vehicle, False)
        except Exception:
            pass
        try:
            tm.distance_to_leading_vehicle(vehicle, ROUTE_TM_DISTANCE_TO_LEADING_M)
        except Exception:
            pass
        try:
            tm.ignore_lights_percentage(vehicle, AI_IGNORE_LIGHTS_PERCENT)
            tm.ignore_signs_percentage(vehicle, AI_IGNORE_SIGNS_PERCENT)
        except Exception:
            pass
        try:
            tm.vehicle_percentage_speed_difference(vehicle, ROUTE_TM_SPEED_DIFFERENCE_PERCENT)
        except Exception:
            pass
        if hasattr(tm, "set_path"):
            tm.set_path(vehicle, path)
            vehicle.set_autopilot(True, TRAFFIC_MANAGER_PORT)
            return True, f"Traffic Manager set_path active for {first_leg}"
        return False, "Traffic Manager set_path is unavailable in this CARLA Python API"
    except Exception as exc:
        return False, f"Traffic Manager route setup failed: {exc}"


def build_route_plan(world: carla.World, scene: str, route_segment: str) -> Optional[Dict[str, Any]]:
    mode = normalize_route_segment(route_segment)
    if not mode:
        return None
    base_tf = project_to_road_transform(world, spawn_transform(scene))
    try:
        candidates = [project_to_road_transform(world, tf) for tf in world.get_map().get_spawn_points()]
    except Exception:
        candidates = []
    candidates = [tf for tf in candidates if tf.location.distance(base_tf.location) > 5.0]
    if not candidates:
        return None

    target = ROUTE_POINT_TARGET_DISTANCE_M
    min_d = min(ROUTE_POINT_MIN_DISTANCE_M, ROUTE_POINT_MAX_DISTANCE_M)
    max_d = max(ROUTE_POINT_MIN_DISTANCE_M, ROUTE_POINT_MAX_DISTANCE_M)
    planner = None
    if GlobalRoutePlanner is not None:
        try:
            planner = GlobalRoutePlanner(world.get_map(), ROUTE_SAMPLING_RESOLUTION)
        except Exception:
            planner = None

    def dist(a: carla.Transform, b: carla.Transform) -> float:
        return float(a.location.distance(b.location))

    def score_from(origin: carla.Transform, candidate: carla.Transform) -> float:
        d = dist(origin, candidate)
        penalty = 0.0 if min_d <= d <= max_d else min(abs(d - min_d), abs(d - max_d)) * 2.0
        return abs(d - target) + penalty

    ranked = sorted(candidates, key=lambda tf: score_from(base_tf, tf))[:ROUTE_CANDIDATE_LIMIT]

    def simple_pick_next(origin: carla.Transform, pool: List[carla.Transform], used: List[carla.Transform]) -> Optional[carla.Transform]:
        usable = [
            tf for tf in pool
            if all(tf.location.distance(item.location) > 20.0 for item in used)
        ]
        if not usable:
            usable = pool
        return min(usable, key=lambda tf: score_from(origin, tf)) if usable else None

    def candidate_path_score(start: carla.Transform, end: carla.Transform, avoid: Optional[carla.Transform]) -> Tuple[float, List[Any], set]:
        trace = route_trace(planner, start, end)
        if not trace:
            return (999999.0 + score_from(start, end), trace, set())
        length = route_length_m(trace)
        cells = route_cells(trace)
        near_avoid = avoid is not None and trace_passes_near(trace, avoid, ROUTE_AVOID_POINT_RADIUS_M)
        repeated_lane = trace_has_repeated_lane(trace)
        length_penalty = abs(length - target)
        range_penalty = 0.0 if min_d <= length <= max_d else min(abs(length - min_d), abs(length - max_d)) * 2.0
        avoid_penalty = 10000.0 if near_avoid else 0.0
        repeat_penalty = 250.0 if repeated_lane else 0.0
        return (length_penalty + range_penalty + avoid_penalty + repeat_penalty + route_turn_penalty(trace), trace, cells)

    point_b = None
    point_c = None
    traces: Dict[str, List[Any]] = {}
    shared_cells = {"AB_BC": None, "BC_CA": None, "CA_AB": None}
    if planner is not None:
        best_score = None
        c_ranked = ranked[:]
        for cand_b in ranked:
            if cand_b.location.distance(base_tf.location) <= 20.0:
                continue
            ab_score, ab_trace, ab_cells = candidate_path_score(base_tf, cand_b, None)
            if not ab_trace:
                continue
            for cand_c in c_ranked:
                if cand_c.location.distance(base_tf.location) <= 20.0 or cand_c.location.distance(cand_b.location) <= 20.0:
                    continue
                bc_score, bc_trace, bc_cells = candidate_path_score(cand_b, cand_c, base_tf)
                ca_score, ca_trace, ca_cells = candidate_path_score(cand_c, base_tf, cand_b)
                if not bc_trace or not ca_trace:
                    continue
                ab_near_c = trace_passes_near(ab_trace, cand_c, ROUTE_AVOID_POINT_RADIUS_M)
                shared_ab_bc = len(ab_cells & bc_cells)
                shared_bc_ca = len(bc_cells & ca_cells)
                shared_ca_ab = len(ca_cells & ab_cells)
                shared_total = shared_ab_bc + shared_bc_ca + shared_ca_ab
                shared_penalty = max(0, shared_ab_bc - ROUTE_MAX_SHARED_ROUTE_CELLS) * 35.0
                shared_penalty += max(0, shared_bc_ca - ROUTE_MAX_SHARED_ROUTE_CELLS) * 35.0
                shared_penalty += max(0, shared_ca_ab - ROUTE_MAX_SHARED_ROUTE_CELLS) * 35.0
                avoid_penalty = 10000.0 if ab_near_c else 0.0
                balance_penalty = 0.2 * abs(route_length_m(ab_trace) - route_length_m(bc_trace))
                balance_penalty += 0.2 * abs(route_length_m(bc_trace) - route_length_m(ca_trace))
                score = ab_score + bc_score + ca_score + shared_penalty + avoid_penalty + balance_penalty + shared_total
                if best_score is None or score < best_score:
                    best_score = score
                    point_b = cand_b
                    point_c = cand_c
                    traces = {"AB": ab_trace, "BC": bc_trace, "CA": ca_trace}
                    shared_cells = {
                        "AB_BC": shared_ab_bc,
                        "BC_CA": shared_bc_ca,
                        "CA_AB": shared_ca_ab,
                    }
        if point_b is None or point_c is None:
            print("route planner could not find non-overlapping A/B/C points; using distance-based fallback", flush=True)

    if point_b is None:
        point_b = simple_pick_next(base_tf, candidates, [base_tf])
    if point_b is None:
        return None
    if point_c is None:
        c_pool = [tf for tf in candidates if tf.location.distance(point_b.location) > 5.0]
        if not c_pool:
            return None
        c_candidates = [
            tf for tf in c_pool
            if all(tf.location.distance(item.location) > 20.0 for item in [base_tf, point_b])
        ] or c_pool
        point_c = min(
            c_candidates,
            key=lambda tf: (
                score_from(point_b, tf)
                + score_from(tf, base_tf)
                + 0.1 * abs(dist(point_b, tf) - dist(tf, base_tf))
            ),
        )
    if point_c is None:
        return None
    if planner is not None and not traces:
        traces = {
            "AB": route_trace(planner, base_tf, point_b),
            "BC": route_trace(planner, point_b, point_c),
            "CA": route_trace(planner, point_c, base_tf),
        }
    points = {"A": base_tf, "B": point_b, "C": point_c}
    if mode == "LOOP":
        legs = [("A", "B"), ("B", "C"), ("C", "A")]
    else:
        legs = [ROUTE_LEGS[mode]]
    trace_lengths = {key: round(route_length_m(value), 1) for key, value in traces.items()} if traces else {}
    path_locations: Dict[str, List[carla.Location]] = {
        key: trace_to_locations(value) for key, value in traces.items()
    } if traces else {}
    path_summary = {
        key: route_path_summary(value) for key, value in path_locations.items()
    }
    return {
        "enabled": True,
        "mode": mode,
        "loop": mode == "LOOP",
        "legs": legs,
        "points": points,
        "point_summary": {name: route_transform_dict(tf) for name, tf in points.items()},
        "distances_m": {
            "AB": round(dist(points["A"], points["B"]), 1),
            "BC": round(dist(points["B"], points["C"]), 1),
            "CA": round(dist(points["C"], points["A"]), 1),
        },
        "route_lengths_m": trace_lengths,
        "shared_route_cells": shared_cells,
        "path_locations": path_locations,
        "path_summary": path_summary,
    }


def spawn_vehicle(
    world: carla.World,
    vehicle_model: str,
    scene: str,
    use_map_spawn: bool = False,
    preferred_transform: Optional[carla.Transform] = None,
    require_preferred: bool = False,
) -> carla.Actor:
    bp_lib = world.get_blueprint_library()
    bp = None
    for pattern in VEHICLE_FILTERS.get(vehicle_model, VEHICLE_FILTERS["Lincoln MKZ"]):
        try:
            if "*" in pattern:
                matches = list(bp_lib.filter(pattern))
                if matches:
                    bp = matches[0]
                    break
            else:
                bp = bp_lib.find(pattern)
                break
        except Exception:
            continue
    if bp is None:
        fallback = bp_lib.filter("vehicle.lincoln.mkz*")
        bp = fallback[0] if fallback else bp_lib.filter("vehicle.*")[0]
    if bp.has_attribute("role_name"):
        bp.set_attribute("role_name", "hero")

    transforms: List[carla.Transform] = []
    if preferred_transform is not None:
        transforms.append(preferred_transform)
    elif use_map_spawn:
        try:
            transforms.extend(list(world.get_map().get_spawn_points()))
        except Exception as exc:
            print(f"map spawn points unavailable, using scenario spawn: {exc}", flush=True)
    if not require_preferred:
        transforms.append(spawn_transform(scene))

    for transform in transforms:
        for dz in (0.0, 0.5, 1.0, 1.5, 2.0):
            tf = carla.Transform(
                carla.Location(transform.location.x, transform.location.y, transform.location.z + dz),
                transform.rotation,
            )
            actor = world.try_spawn_actor(bp, tf)
            if actor:
                actor.set_simulate_physics(True)
                return actor
            time.sleep(0.05)
    raise RuntimeError(f"failed to spawn vehicle: {vehicle_model}")


def actual_vehicle_model(requested: str, actor: carla.Actor) -> str:
    requested = requested if requested in CAMERA_PROFILES else "Lincoln MKZ"
    actor_type = str(getattr(actor, "type_id", "")).lower()
    for name, blueprint in VEHICLE_BLUEPRINTS.items():
        if actor_type == blueprint.lower():
            return name
    for name, patterns in VEHICLE_FILTERS.items():
        for pattern in patterns:
            token = pattern.lower().replace("*", "")
            if token and token in actor_type:
                return name
    return requested


def cleanup_hero(world: carla.World) -> None:
    try:
        for actor in world.get_actors().filter("vehicle.*"):
            if actor.attributes.get("role_name") in {"hero", "ego", "manual"}:
                destroy_actor(actor)
    except Exception:
        pass


def start_child(
    script: Path,
    env: Dict[str, str],
    wrapper: Optional[Path] = None,
) -> Tuple[Optional[subprocess.Popen], Optional[Any], Optional[Path]]:
    if not script.exists():
        return None, None, None
    LOG_DIR.mkdir(exist_ok=True)
    log_path = LOG_DIR / f"{script.stem}_{int(time.time())}.log"
    log_file = open(log_path, "a", encoding="utf-8")
    kwargs: Dict[str, Any] = {
        "cwd": str(PROJECT_ROOT),
        "stdout": log_file,
        "stderr": subprocess.STDOUT,
        "env": env,
    }
    if os.name != "nt":
        kwargs["preexec_fn"] = os.setsid
    cmd = [sys.executable, str(wrapper), str(script)] if wrapper else [sys.executable, str(script)]
    proc = subprocess.Popen(cmd, **kwargs)
    return proc, log_file, log_path


def read_view_command(last_seen: Any) -> Tuple[Any, Optional[str]]:
    try:
        mtime = VIEW_COMMAND_FILE.stat().st_mtime
        data = json.loads(VIEW_COMMAND_FILE.read_text(encoding="utf-8"))
        marker = data.get("revision")
        if marker is None:
            marker = data.get("updated_at")
        if marker is None:
            marker = mtime
        marker = str(marker)
        if marker == last_seen:
            return last_seen, None
        return marker, normalize_view(data.get("camera_view", "follow"))
    except Exception:
        return last_seen, None


def side_mirrors_enabled(mode: str) -> bool:
    if env_flag("OFFICIAL_DISABLE_SIDE_MIRRORS", "0"):
        return False
    if mode == "AIGO" and env_flag("OFFICIAL_AIGO_DISABLE_SIDE_MIRRORS", "0"):
        return False
    if mode == "AI" and env_flag("OFFICIAL_AI_BASELINE_DISABLE_SIDE_MIRRORS", "0"):
        return False
    return True


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default=os.environ.get("CARLA_HOST", "127.0.0.1"))
    parser.add_argument("--port", type=int, default=int(os.environ.get("CARLA_PORT", "2000")))
    parser.add_argument("--scene", default="Town02")
    parser.add_argument("--vehicle", default="Lincoln MKZ")
    parser.add_argument("--mode", choices=["Manual", "AIGO", "AI"], default="AIGO")
    parser.add_argument("--weather", default="Sunny")
    parser.add_argument("--time-of-day", default="Noon")
    parser.add_argument("--camera-view", default="follow")
    parser.add_argument("--res", default="1920x1080")
    parser.add_argument("--target-ip", default="127.0.0.1")
    parser.add_argument("--control-port", type=int, default=5001)
    parser.add_argument("--telemetry-ports", default=DEFAULT_TELEMETRY_PORTS)
    parser.add_argument("--algo-telemetry-port", type=int, default=DEFAULT_ALGO_TELEMETRY_PORT)
    parser.add_argument("--manual-telemetry-port", type=int, default=DEFAULT_MANUAL_TELEMETRY_PORT)
    parser.add_argument("--start-bridge", action="store_true")
    parser.add_argument("--show-overlay", action="store_true")
    parser.add_argument("--no-load-world", action="store_true")
    parser.add_argument("--load-traffic", action="store_true")
    parser.add_argument("--route-segment", default="")
    return parser.parse_args()


def handle_shutdown(signum: int, _frame: Any) -> None:
    print(f"shutdown requested by signal {signum}", flush=True)
    SHUTDOWN_EVENT.set()


def resolve_display_size(res: str) -> Tuple[int, int]:
    text = str(res or "auto").strip().lower()
    if text in {"auto", "fullscreen", "screen"}:
        info = pygame.display.Info()
        width = int(getattr(info, "current_w", 0) or 1920)
        height = int(getattr(info, "current_h", 0) or 1080)
        return width, height
    try:
        width, height = [int(x) for x in text.split("x", 1)]
        return width, height
    except Exception:
        return 1920, 1080


def main() -> None:
    args = parse_args()
    for sig in (signal.SIGTERM, signal.SIGINT):
        try:
            signal.signal(sig, handle_shutdown)
        except Exception:
            pass

    scene = normalize_scene(args.scene)
    telemetry_ports = parse_port_list(args.telemetry_ports) or parse_port_list(DEFAULT_TELEMETRY_PORTS)
    external_targets = [(args.target_ip, port) for port in telemetry_ports]
    local_targets = [("127.0.0.1", port) for port in telemetry_ports if port != args.algo_telemetry_port]
    if args.target_ip in {"127.0.0.1", "localhost"}:
        external_targets = local_targets
    algo_target = ("127.0.0.1", args.algo_telemetry_port)
    manual_bridge_target = ("127.0.0.1", args.manual_telemetry_port)

    os.chdir(PROJECT_ROOT)
    LOG_DIR.mkdir(exist_ok=True)
    pygame.init()
    pygame.font.init()
    width, height = resolve_display_size(args.res)
    display_flags = pygame.HWSURFACE | pygame.DOUBLEBUF
    if os.environ.get("OFFICIAL_DEMO_FULLSCREEN", "1") == "1":
        display_flags |= pygame.FULLSCREEN
    display = pygame.display.set_mode((width, height), display_flags)
    pygame.display.set_caption(f"CARLA Official Demo - {args.mode}")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("monospace", 18)

    client = carla.Client(args.host, args.port)
    client.set_timeout(float(os.environ.get("OFFICIAL_CARLA_CLIENT_TIMEOUT", "60.0")))
    target_world = runtime_map(scene)
    current_world = client.get_world()
    current_name = current_world.get_map().name.split("/")[-1]
    if args.no_load_world or current_name == target_world:
        world = current_world
    else:
        world = client.load_world(target_world)
    set_async(world)
    apply_weather(world, args.weather, args.time_of_day)
    cleanup_hero(world)

    route_plan = build_route_plan(world, scene, args.route_segment) if args.mode == "AI" else None
    route_start_requested = bool(normalize_route_segment(args.route_segment))
    route_start_transform = None
    if route_plan:
        start_label = route_plan["legs"][0][0]
        route_start_transform = route_plan["points"][start_label]
    use_map_spawn = args.mode == "AI" and AI_USE_MAP_SPAWN and route_plan is None
    if route_start_transform is not None:
        spawn_strategy = f"route_point_{route_plan['legs'][0][0]}"
    else:
        spawn_strategy = "map_spawn_points" if use_map_spawn else "scenario_transform"
    vehicle = spawn_vehicle(
        world,
        args.vehicle,
        scene,
        use_map_spawn=use_map_spawn,
        preferred_transform=route_start_transform,
        require_preferred=route_start_transform is not None,
    )
    camera_vehicle_model = actual_vehicle_model(args.vehicle, vehicle)
    route_agent = None
    route_leg_index = 0
    route_active_leg = None
    route_target_label = None
    route_arrived = False
    route_error = None
    route_controller = None
    route_tm_message = None
    route_last_steer = 0.0
    route_last_control_time = time.time()
    if args.mode == "AI" and route_plan:
        tm_ok, tm_message = configure_route_tm(vehicle, client, route_plan)
        route_tm_message = tm_message
        if tm_ok:
            route_controller = "traffic_manager_path"
            route_active_leg = "".join(route_plan["legs"][0])
            route_target_label = route_plan["legs"][0][1]
            print(tm_message, flush=True)
        elif BasicAgent is None:
            route_error = f"{tm_message}; CARLA BasicAgent is unavailable"
            print(route_error, flush=True)
        else:
            try:
                try:
                    vehicle.set_autopilot(False, TRAFFIC_MANAGER_PORT)
                except Exception:
                    pass
                try:
                    route_agent = BasicAgent(vehicle, target_speed=ROUTE_TARGET_SPEED_KMH)
                except TypeError:
                    route_agent = BasicAgent(vehicle)
                soften_route_agent(route_agent)
                route_controller = "basic_agent_fallback"
                route_active_leg = "".join(route_plan["legs"][0])
                route_target_label = route_plan["legs"][0][1]
                route_agent.set_destination(route_plan["points"][route_target_label].location)
                route_error = tm_message
            except Exception as exc:
                route_error = f"route agent setup failed: {exc}"
                print(route_error, flush=True)
                route_agent = None
    elif args.mode == "AI":
        configure_ai(vehicle, client)
    traffic_actors: List[carla.Actor] = []
    if args.load_traffic:
        traffic_actors = spawn_traffic(world, client, vehicle)
    camera = CameraDisplay(world, vehicle, (width, height), args.camera_view, camera_vehicle_model)
    side_mirrors: Optional[SideMirrorStreams] = None
    if side_mirrors_enabled(args.mode):
        try:
            side_mirrors = SideMirrorStreams(world, vehicle)
            side_mirrors.start()
        except Exception as exc:
            print(f"side mirror stream disabled: {exc}", flush=True)
            if side_mirrors is not None:
                side_mirrors.destroy()
            side_mirrors = None
    else:
        print(f"side mirror stream disabled for {args.mode} baseline path", flush=True)
    sensor_suite = SensorSuite(world, vehicle, camera_vehicle_model, args.mode)
    sensor_suite.start()
    ctrl_sock = bind_control_socket(args.control_port) if args.mode in {"AIGO", "Manual"} else None
    telem_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    env = os.environ.copy()
    algo_proc = bridge_proc = None
    algo_log = bridge_log = None
    algo_path = bridge_path = None

    if args.mode == "AIGO":
        script = PROJECT_ROOT / SCENARIOS[scene]["script"]
        env["AIGO_TELEMETRY_PORT"] = str(args.algo_telemetry_port)
        algo_proc, algo_log, algo_path = start_child(script, env, wrapper=APP_DIR / "aigo_port_wrapper.py")
    elif args.mode == "Manual" and args.start_bridge:
        script = PROJECT_ROOT / os.environ.get("MANUAL_BRIDGE_SCRIPT", "can_tcp_bridge_vcu.py")
        env["CARLA_MANUAL_TELEM_PORT"] = str(args.manual_telemetry_port)
        env["CARLA_MANUAL_CTRL_PORT"] = str(args.control_port)
        bridge_proc, bridge_log, bridge_path = start_child(script, env)

    running = True
    last_control_time = 0.0
    last_control = None
    control_rx = 0
    telemetry_tx = 0
    last_status = 0.0
    mode_started_at = time.time()
    ai_fallback_active = False
    last_snapshot_sim_time: Optional[float] = None
    last_timing_wall_time: Optional[float] = None
    try:
        data = json.loads(VIEW_COMMAND_FILE.read_text(encoding="utf-8"))
        marker = data.get("revision")
        if marker is None:
            marker = data.get("updated_at")
        last_view_marker = str(marker if marker is not None else VIEW_COMMAND_FILE.stat().st_mtime)
    except Exception:
        last_view_marker = ""

    try:
        while running and not SHUTDOWN_EVENT.is_set():
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                    SHUTDOWN_EVENT.set()
                elif event.type == pygame.KEYUP:
                    if event.key in (pygame.K_ESCAPE, pygame.K_q):
                        running = False
                        SHUTDOWN_EVENT.set()
                    elif event.key == pygame.K_1:
                        camera.set_view("follow")
                    elif event.key == pygame.K_2:
                        camera.set_view("driver")
                    elif event.key == pygame.K_3:
                        camera.set_view("follow")

            last_view_marker, view_command = read_view_command(last_view_marker)
            if view_command:
                camera.set_view(view_command)

            latest_packet = None
            if ctrl_sock is not None:
                try:
                    packet, _ = ctrl_sock.recvfrom(4096)
                    while True:
                        latest_packet = packet
                        try:
                            packet, _ = ctrl_sock.recvfrom(4096)
                        except BlockingIOError:
                            break
                except BlockingIOError:
                    pass

            if latest_packet is not None:
                try:
                    last_control = apply_udp_control(vehicle, latest_packet)
                    last_control_time = time.time()
                    control_rx += 1
                except Exception as exc:
                    print(f"control parse failed: {exc}", flush=True)

            if args.mode == "Manual":
                stale = time.time() - last_control_time if last_control_time else 999.0
                if stale > 0.7:
                    vehicle.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0))

            loop_now = time.time()
            speed = math.sqrt(
                vehicle.get_velocity().x ** 2 + vehicle.get_velocity().y ** 2 + vehicle.get_velocity().z ** 2
            ) * 3.6
            if route_controller == "traffic_manager_path" and route_plan and not route_arrived:
                try:
                    target_tf = route_plan["points"].get(route_target_label) if route_target_label else None
                    if target_tf and vehicle.get_location().distance(target_tf.location) <= ROUTE_ARRIVAL_DISTANCE_M:
                        route_arrived = True
                        route_active_leg = None
                        route_target_label = None
                        try:
                            vehicle.set_autopilot(False, TRAFFIC_MANAGER_PORT)
                        except Exception:
                            pass
                        vehicle.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0))
                except Exception as exc:
                    route_error = f"Traffic Manager route monitor failed: {exc}"
            if args.mode == "AI" and route_agent is not None:
                try:
                    if not route_arrived:
                        control = route_agent.run_step()
                        control.manual_gear_shift = False
                        control, route_last_steer = smooth_route_control(
                            control,
                            route_last_steer,
                            loop_now - route_last_control_time,
                        )
                        route_last_control_time = loop_now
                        vehicle.apply_control(control)
                        if route_agent.done():
                            route_leg_index += 1
                            if route_plan and route_plan.get("loop"):
                                route_leg_index %= len(route_plan["legs"])
                            if route_plan and route_leg_index < len(route_plan["legs"]):
                                route_active_leg = "".join(route_plan["legs"][route_leg_index])
                                route_target_label = route_plan["legs"][route_leg_index][1]
                                route_agent.set_destination(route_plan["points"][route_target_label].location)
                            else:
                                route_arrived = True
                                route_active_leg = None
                                route_target_label = None
                                vehicle.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0))
                    else:
                        vehicle.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0))
                except Exception as exc:
                    route_error = f"route agent failed: {exc}"
                    print(route_error, flush=True)
                    route_agent = None
                    vehicle.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0))

            if args.mode == "AI" and route_start_requested and route_agent is None and route_controller != "traffic_manager_path":
                vehicle.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0))

            if args.mode == "AI" and route_agent is None and not route_start_requested:
                can_fallback = AI_FALLBACK_ENABLED and not blocked_by_red_light(vehicle)
                if can_fallback and not ai_fallback_active and loop_now - mode_started_at >= AI_FALLBACK_AFTER_SEC and speed < 1.0:
                    try:
                        vehicle.set_autopilot(False, TRAFFIC_MANAGER_PORT)
                    except Exception:
                        pass
                    ai_fallback_active = True
                    try:
                        fv = vehicle.get_transform().get_forward_vector()
                        vehicle.enable_constant_velocity(
                            carla.Vector3D(
                                fv.x * AI_FALLBACK_TARGET_SPEED_MS,
                                fv.y * AI_FALLBACK_TARGET_SPEED_MS,
                                fv.z * AI_FALLBACK_TARGET_SPEED_MS,
                            )
                        )
                    except Exception as exc:
                        print(f"AI constant-velocity fallback failed, using throttle control: {exc}", flush=True)
                    print("AI autopilot did not move the ego vehicle; constant-velocity fallback enabled.", flush=True)
                if ai_fallback_active and speed < 1.0:
                    vehicle.apply_control(
                        carla.VehicleControl(
                            throttle=0.8,
                            steer=0.0,
                            brake=0.0,
                            hand_brake=False,
                            reverse=False,
                            manual_gear_shift=False,
                            gear=1,
                        )
                    )

            route_status = None
            if route_plan:
                route_status = {
                    "enabled": True,
                    "mode": route_plan.get("mode"),
                    "loop": route_plan.get("loop"),
                    "active_leg": route_active_leg,
                    "target": route_target_label,
                    "arrived": route_arrived,
                    "controller": route_controller,
                    "traffic_manager_message": route_tm_message,
                    "target_speed_kmh": ROUTE_TARGET_SPEED_KMH,
                    "tm_seed": ROUTE_TM_SEED,
                    "steer_limit": ROUTE_STEER_LIMIT,
                    "steer_rate_limit_per_sec": ROUTE_STEER_RATE_LIMIT,
                    "points": route_plan.get("point_summary"),
                    "distances_m": route_plan.get("distances_m"),
                    "route_lengths_m": route_plan.get("route_lengths_m"),
                    "shared_route_cells": route_plan.get("shared_route_cells"),
                    "path_summary": route_plan.get("path_summary"),
                    "error": route_error,
                }
            elif route_start_requested:
                route_status = {
                    "enabled": True,
                    "mode": normalize_route_segment(args.route_segment),
                    "loop": False,
                    "active_leg": None,
                    "target": None,
                    "arrived": False,
                    "controller": route_controller,
                    "traffic_manager_message": route_tm_message,
                    "target_speed_kmh": ROUTE_TARGET_SPEED_KMH,
                    "tm_seed": ROUTE_TM_SEED,
                    "steer_limit": ROUTE_STEER_LIMIT,
                    "steer_rate_limit_per_sec": ROUTE_STEER_RATE_LIMIT,
                    "points": None,
                    "distances_m": None,
                    "route_lengths_m": None,
                    "shared_route_cells": None,
                    "path_summary": None,
                    "error": route_error or "route plan could not be built from current map spawn points",
                }

            telemetry = build_telemetry(vehicle, camera_vehicle_model)
            frontend_telemetry = build_frontend_telemetry(telemetry, vehicle, camera_vehicle_model)
            payload = json.dumps(telemetry, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
            udp_targets = list(dict.fromkeys(external_targets))
            if args.mode in {"AIGO", "Manual"}:
                udp_targets.extend(local_targets)
            if args.mode == "AIGO":
                udp_targets.append(algo_target)
            elif args.mode == "Manual":
                udp_targets.append(manual_bridge_target)
            for target in dict.fromkeys(udp_targets):
                safe_udp_send(telem_sock, payload, target, "telemetry")
            telemetry_tx += 1
            sensor_suite.maybe_send_udp()

            camera.render(display)
            if args.show_overlay:
                label = f"{args.mode} | {scene} | {camera.view} | speed {speed:5.1f} km/h | ctrl {control_rx}"
                display.blit(font.render(label, True, (255, 255, 255)), (12, 12))
            pygame.display.flip()

            now = time.time()
            if now - last_status > 0.5:
                timing: Dict[str, Any] = {
                    "wall_time": now,
                    "wall_dt": round(now - last_timing_wall_time, 6) if last_timing_wall_time is not None else None,
                    "traffic_manager_port": TRAFFIC_MANAGER_PORT,
                    "fixed_delta_seconds": FIXED_DELTA_SECONDS,
                    "max_substep_delta_time": PHYSICS_SUBSTEP_DELTA,
                    "max_substeps": PHYSICS_MAX_SUBSTEPS,
                    "camera_sensor_tick": CAMERA_SENSOR_TICK,
                    "display_loop_hz": DISPLAY_LOOP_HZ,
                }
                try:
                    snapshot = world.get_snapshot()
                    timestamp = snapshot.timestamp
                    sim_time = float(getattr(timestamp, "elapsed_seconds", 0.0) or 0.0)
                    settings = world.get_settings()
                    timing.update(
                        {
                            "frame": int(getattr(snapshot, "frame", 0) or 0),
                            "sim_time": sim_time,
                            "sim_dt": round(sim_time - last_snapshot_sim_time, 6)
                            if last_snapshot_sim_time is not None
                            else None,
                            "platform_time": float(getattr(timestamp, "platform_timestamp", 0.0) or 0.0),
                            "synchronous_mode": bool(getattr(settings, "synchronous_mode", False)),
                            "fixed_delta_current": getattr(settings, "fixed_delta_seconds", None),
                        }
                    )
                    last_snapshot_sim_time = sim_time
                except Exception as exc:
                    timing["error"] = str(exc)
                last_timing_wall_time = now
                write_status(
                    {
                        "running": True,
                        "mode": args.mode,
                        "scene": scene,
                        "world": world.get_map().name.split("/")[-1],
                        "vehicle_alive": bool(vehicle and vehicle.is_alive),
                        "vehicle_model": camera_vehicle_model,
                        "vehicle_type_id": getattr(vehicle, "type_id", None),
                        "actor_id": vehicle.id,
                        "speed_kmh": round(speed, 3),
                        "spawn_strategy": spawn_strategy,
                        "route": route_status,
                        "traffic_manager_port": TRAFFIC_MANAGER_PORT if args.mode == "AI" or args.load_traffic else None,
                        "ai_obey_lights": AI_IGNORE_LIGHTS_PERCENT <= 0.0,
                        "ai_ignore_lights_percent": AI_IGNORE_LIGHTS_PERCENT,
                        "ai_ignore_signs_percent": AI_IGNORE_SIGNS_PERCENT,
                        "ai_fallback_active": ai_fallback_active if args.mode == "AI" else None,
                        "camera_view": camera.view,
                        "camera_profile": camera.profile(camera.view),
                        "control_rx_count": control_rx,
                        "telemetry_tx_count": telemetry_tx,
                        "telemetry_ports": telemetry_ports,
                        "algo_telemetry_port": args.algo_telemetry_port if args.mode == "AIGO" else None,
                        "manual_telemetry_port": args.manual_telemetry_port if args.mode == "Manual" else None,
                        "last_control": last_control,
                        "last_control_age_sec": round(now - last_control_time, 3) if last_control_time else None,
                        "legacy_telemetry": telemetry,
                        "frontend_telemetry": frontend_telemetry,
                        "algo_pid": algo_proc.pid if algo_proc and algo_proc.poll() is None else None,
                        "algo_log": str(algo_path) if algo_path else None,
                        "bridge_pid": bridge_proc.pid if bridge_proc and bridge_proc.poll() is None else None,
                        "bridge_log": str(bridge_path) if bridge_path else None,
                        "traffic_count": len(traffic_actors),
                        "side_mirror_streams": side_mirrors.snapshot() if side_mirrors is not None else None,
                        "sensor_data": sensor_suite.snapshot(),
                        "sensor_udp_ports": sensor_suite.udp_ports,
                        "sensor_tx_count": sensor_suite.tx_count,
                        "timing": timing,
                        "updated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                    }
                )
                last_status = now

            clock.tick(max(1, DISPLAY_LOOP_HZ))
    finally:
        write_status({"running": False, "mode": args.mode, "updated_at": time.strftime("%Y-%m-%d %H:%M:%S")})
        terminate_process(algo_proc)
        terminate_process(bridge_proc)
        if algo_log:
            algo_log.close()
        if bridge_log:
            bridge_log.close()
        camera.destroy()
        if side_mirrors is not None:
            side_mirrors.destroy()
        sensor_suite.destroy()
        for actor in traffic_actors:
            destroy_actor(actor)
        try:
            MAX_STEER_CACHE.pop(int(getattr(vehicle, "id", -1) or -1), None)
        except Exception:
            pass
        destroy_actor(vehicle)
        if ctrl_sock is not None:
            ctrl_sock.close()
        telem_sock.close()
        pygame.quit()


if __name__ == "__main__":
    main()
