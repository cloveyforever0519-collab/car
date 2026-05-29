#!/usr/bin/env python3
import argparse
import json
import random
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path


SCENES = [
    "Town01",
    "Town02",
    "Town03",
    "Town04",
    "Town05",
    "TrainingGround",
]

WEATHERS = [
    "Sunny",
    "Cloudy",
    "Light Rain",
    "Heavy Rainstorm",
    "Fog/Dense Fog",
    "Clear",
]

SUNSHINE_TIMES = [
    "Noon",
    "Sunset",
    "Late Night",
]

DRIVE_MODES = [
    "AI",
    "AIGO",
    "Manual",
]

VEHICLES = [
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
]

CAMERA_VIEWS = [
    "follow",
    "driver",
]

TRAFFIC_LOADS = [
    "0",  # full visible traffic in the backend mapping
    "1",  # hidden/no visible traffic in the backend mapping
]


def http_json(method, url, payload=None, timeout=10.0):
    data = None
    headers = {"Accept": "application/json"}
    if payload is not None:
        data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        headers["Content-Type"] = "application/json"
    req = urllib.request.Request(url, data=data, headers=headers, method=method)
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        body = resp.read().decode("utf-8", errors="replace")
        if not body:
            return {}
        return json.loads(body)


def load_stream_status(project_dir):
    path = project_dir / "logs" / "mirror_stream_status.json"
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception as exc:
        return {"last_error": f"stream status unavailable: {exc}"}


def random_command(index):
    return {
        "sendstate": "START",
        "scene": random.choice(SCENES),
        "sky": random.choice(WEATHERS),
        "sunshinetime": random.choice(SUNSHINE_TIMES),
        "drive_mode": random.choice(DRIVE_MODES),
        "loadingtransportation": random.choice(TRAFFIC_LOADS),
        "vehiclemodel": random.choice(VEHICLES),
        "camera_view": random.choice(CAMERA_VIEWS),
        "stress_index": index,
    }


def summarize_health(health):
    diagnostics = health.get("diagnostics") or {}
    speed = diagnostics.get("speed_kmh")
    if isinstance(speed, (int, float)):
        speed_text = f"{speed:.1f}km/h"
    else:
        speed_text = "n/a"
    return (
        f"api={health.get('api')} "
        f"indicator={health.get('carla_backend_indicator')} "
        f"connected={health.get('carla_connected')} "
        f"vehicle={health.get('vehicle_alive')} "
        f"world={health.get('world')} "
        f"view={health.get('camera_view')} "
        f"speed={speed_text}"
    )


def summarize_streams(streams):
    stream_map = streams.get("streams") or {}
    parts = []
    for name in ("carla_view", "carla_rear_left", "carla_rear_right"):
        item = stream_map.get(name) or {}
        if item:
            parts.append(f"{name}=pub:{item.get('publishing')} frames:{item.get('frames_out')}")
        else:
            parts.append(f"{name}=missing")
    err = streams.get("last_error")
    if err:
        parts.append(f"last_error={err}")
    return " ".join(parts)


def sleep_with_status(api_base, project_dir, seconds, poll_sec):
    deadline = time.time() + seconds
    while time.time() < deadline:
        remaining = max(0, int(deadline - time.time()))
        try:
            health = http_json("GET", f"{api_base}/health", timeout=5.0)
            health_text = summarize_health(health)
        except Exception as exc:
            health_text = f"health_error={exc}"
        stream_text = summarize_streams(load_stream_status(project_dir))
        print(f"[wait {remaining:>3}s] {health_text} | {stream_text}", flush=True)
        time.sleep(min(poll_sec, max(0.0, deadline - time.time())))


def main():
    parser = argparse.ArgumentParser(description="Random CARLA frontend command stress test.")
    parser.add_argument("--api", default="http://127.0.0.1:8765", help="backend API base URL")
    parser.add_argument("--count", type=int, default=10, help="number of commands to send")
    parser.add_argument("--interval", type=float, default=60.0, help="seconds between commands")
    parser.add_argument("--poll", type=float, default=10.0, help="status poll seconds while waiting")
    parser.add_argument("--project-dir", default=".", help="Carla_Project directory")
    parser.add_argument("--seed", type=int, default=None, help="random seed for reproducible runs")
    parser.add_argument("--dry-run", action="store_true", help="print commands without sending")
    args = parser.parse_args()

    if args.seed is not None:
        random.seed(args.seed)

    api_base = args.api.rstrip("/")
    project_dir = Path(args.project_dir).resolve()

    print(f"stress api={api_base} count={args.count} interval={args.interval}s project={project_dir}")
    try:
        health = http_json("GET", f"{api_base}/health", timeout=5.0)
        print(f"initial health: {summarize_health(health)}")
    except Exception as exc:
        print(f"initial health failed: {exc}")
        if not args.dry_run:
            return 2

    for index in range(1, args.count + 1):
        cmd = random_command(index)
        print("=" * 90)
        print(f"[{index}/{args.count}] command:")
        print(json.dumps(cmd, ensure_ascii=False, indent=2))

        if not args.dry_run:
            try:
                response = http_json("POST", f"{api_base}/command", payload=cmd, timeout=15.0)
                print("response:")
                print(json.dumps(response, ensure_ascii=False, indent=2))
            except (urllib.error.URLError, TimeoutError, json.JSONDecodeError) as exc:
                print(f"post failed: {exc}")
                return 3

            try:
                view_response = http_json(
                    "POST",
                    f"{api_base}/view",
                    payload={"camera_view": cmd["camera_view"]},
                    timeout=8.0,
                )
                print(
                    "view refresh: "
                    f"ok={view_response.get('ok')} "
                    f"view={view_response.get('camera_view')} "
                    f"url={view_response.get('camera_stream_url')}"
                )
            except Exception as exc:
                print(f"view refresh failed: {exc}")

        if index < args.count:
            sleep_with_status(api_base, project_dir, args.interval, args.poll)

    print("=" * 90)
    try:
        health = http_json("GET", f"{api_base}/health", timeout=5.0)
        print(f"final health: {summarize_health(health)}")
    except Exception as exc:
        print(f"final health failed: {exc}")
    print(f"final streams: {summarize_streams(load_stream_status(project_dir))}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
