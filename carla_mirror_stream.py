import json
import os
import queue
import subprocess
import threading
import time
import urllib.request
from dataclasses import dataclass
from pathlib import Path

import carla


CARLA_HOST = os.environ.get("CARLA_HOST", "127.0.0.1")
CARLA_PORT = int(os.environ.get("CARLA_PORT", "2000"))
CARLA_TIMEOUT = float(os.environ.get("MIRROR_CARLA_TIMEOUT", "5.0"))

HEALTH_URL = os.environ.get("MIRROR_HEALTH_URL", "http://127.0.0.1:8765/health")
STATUS_FILE = Path(os.environ.get("MIRROR_STATUS_FILE", "logs/mirror_stream_status.json"))

RTSP_HOST = os.environ.get("MIRROR_RTSP_HOST", "127.0.0.1")
RTSP_PORT = int(os.environ.get("MIRROR_RTSP_PORT", "8554"))
RTSP_PUBLIC_HOST = os.environ.get("MIRROR_RTSP_PUBLIC_HOST", "192.168.110.100")

WIDTH = int(os.environ.get("MIRROR_WIDTH", "1280"))
HEIGHT = int(os.environ.get("MIRROR_HEIGHT", "720"))
FPS = float(os.environ.get("MIRROR_FPS", "20"))
BITRATE = os.environ.get("MIRROR_BITRATE", "5000k")
BUFSIZE = os.environ.get("MIRROR_BUFSIZE", BITRATE)
FFMPEG_BIN = os.environ.get("MIRROR_FFMPEG_BIN", "ffmpeg")
ENCODER = os.environ.get("MIRROR_ENCODER", "auto").strip().lower()
HFLIP = os.environ.get("MIRROR_HFLIP", "0").strip().lower() in {"1", "true", "yes", "on"}
ENABLE_VIEW_STREAMS = os.environ.get("MIRROR_ENABLE_VIEW_STREAMS", "0").strip().lower() not in {"0", "false", "no", "off"}
ENABLE_REAR_STREAMS = os.environ.get("MIRROR_ENABLE_REAR_STREAMS", "1").strip().lower() not in {"0", "false", "no", "off"}
ENABLE_BIRDVIEW_STREAM = os.environ.get("MIRROR_ENABLE_BIRDVIEW_STREAM", "1").strip().lower() not in {"0", "false", "no", "off"}
SINGLE_ACTIVE_VIEW = os.environ.get("MIRROR_SINGLE_ACTIVE_VIEW", "0").strip().lower() not in {"0", "false", "no", "off"}
ACTIVE_VIEW_PATH = os.environ.get("MIRROR_ACTIVE_VIEW_PATH", "carla_view")
BIRDVIEW_PATH = os.environ.get("MIRROR_BIRDVIEW_PATH", "carla_birdview")
BIRDVIEW_FOV = float(os.environ.get("MIRROR_BIRDVIEW_FOV", "96.0"))
QUEUE_SIZE = max(1, int(os.environ.get("MIRROR_QUEUE_SIZE", "1")))
STALE_FRAME_SEC = max(0.0, float(os.environ.get("MIRROR_STALE_FRAME_SEC", "0.35")))

RECONNECT_SEC = float(os.environ.get("MIRROR_RECONNECT_SEC", "2.0"))
EGO_SCAN_SEC = float(os.environ.get("MIRROR_EGO_SCAN_SEC", "1.0"))

VIEW_PROFILES = {
    "follow": {
        "profile": "follow",
        "location": {"x": -7.0, "y": 0.0, "z": 3.0},
        "rotation": {"pitch": -15.0, "yaw": 0.0, "roll": 0.0},
        "fov": 90.0,
        "attachment_type": "SpringArmGhost",
    },
    "driver": {
        "profile": "driver",
        "location": {"x": 0.55, "y": -0.38, "z": 1.25},
        "rotation": {"pitch": -2.0, "yaw": 0.0, "roll": 0.0},
        "fov": 95.0,
        "attachment_type": "Rigid",
    },
}

VIEW_MOUNTS = {
    "carla_follow": {
        **VIEW_PROFILES["follow"],
        "path": "carla_follow",
        "view": "follow",
    },
    "carla_driver": {
        **VIEW_PROFILES["driver"],
        "path": "carla_driver",
        "view": "driver",
    },
}

REAR_MOUNTS = {
    "carla_rear_left": {
        "path": "carla_rear_left",
        "location": {"x": -0.55, "y": -0.95, "z": 1.35},
        "rotation": {"pitch": -6.0, "yaw": -145.0, "roll": 0.0},
        "fov": 75.0,
        "attachment_type": "Rigid",
    },
    "carla_rear_right": {
        "path": "carla_rear_right",
        "location": {"x": -0.55, "y": 0.95, "z": 1.35},
        "rotation": {"pitch": -6.0, "yaw": 145.0, "roll": 0.0},
        "fov": 75.0,
        "attachment_type": "Rigid",
    },
}

BIRDVIEW_MOUNTS = {
    "carla_birdview": {
        "path": BIRDVIEW_PATH,
        "location": {"x": 0.0, "y": 0.0, "z": 8.0},
        "rotation": {"pitch": -90.0, "yaw": 0.0, "roll": 0.0},
        "fov": BIRDVIEW_FOV,
        "attachment_type": "Rigid",
        "view": "birdview",
        "fit": "contain",
    },
}

MOUNTS = {}
if ENABLE_VIEW_STREAMS:
    if SINGLE_ACTIVE_VIEW:
        MOUNTS["carla_view"] = {**VIEW_PROFILES["follow"], "path": ACTIVE_VIEW_PATH, "view": "follow"}
    else:
        MOUNTS.update(VIEW_MOUNTS)
if ENABLE_REAR_STREAMS:
    MOUNTS.update(REAR_MOUNTS)
if ENABLE_BIRDVIEW_STREAM:
    MOUNTS.update(BIRDVIEW_MOUNTS)
if not MOUNTS:
    MOUNTS["carla_view"] = {**VIEW_PROFILES["follow"], "path": ACTIVE_VIEW_PATH, "view": "follow"}


def normalize_camera_view(value):
    raw = str(value or "follow").strip().lower().replace("-", "_")
    if raw in {"driver", "first", "first_person", "cockpit", "ego"} or "driver" in raw or "first" in raw:
        return "driver"
    return "follow"


def view_public_url():
    if SINGLE_ACTIVE_VIEW:
        return f"rtsp://{RTSP_PUBLIC_HOST}:{RTSP_PORT}/{ACTIVE_VIEW_PATH}"
    return None


def now_text():
    return time.strftime("%Y-%m-%d %H:%M:%S")


class Status:
    def __init__(self):
        self.lock = threading.Lock()
        self.data = {
            "running": True,
            "updated_at": now_text(),
            "carla_connected": False,
            "ego_actor_id": None,
            "width": WIDTH,
            "height": HEIGHT,
            "fps": FPS,
            "codec": "H264",
            "rtsp_port": RTSP_PORT,
            "urls": {
                name: f"rtsp://{RTSP_PUBLIC_HOST}:{RTSP_PORT}/{cfg['path']}"
                for name, cfg in MOUNTS.items()
            },
            "center_view_url": view_public_url(),
            "side_view_urls": {
                name: f"rtsp://{RTSP_PUBLIC_HOST}:{RTSP_PORT}/{cfg['path']}"
                for name, cfg in REAR_MOUNTS.items()
                if name in MOUNTS
            },
            "birdview_url": (
                f"rtsp://{RTSP_PUBLIC_HOST}:{RTSP_PORT}/{BIRDVIEW_PATH}"
                if "carla_birdview" in MOUNTS
                else None
            ),
            "view_mode": "single_active" if SINGLE_ACTIVE_VIEW else "parallel",
            "active_view": "follow",
            "active_view_url": view_public_url(),
            "view_urls": (
                {"active": view_public_url(), "follow": view_public_url(), "driver": view_public_url()}
                if SINGLE_ACTIVE_VIEW else
                {
                    cfg["view"]: f"rtsp://{RTSP_PUBLIC_HOST}:{RTSP_PORT}/{cfg['path']}"
                    for cfg in MOUNTS.values()
                    if "view" in cfg
                }
            ),
            "streams": {},
            "last_error": None,
        }

    def update(self, **kwargs):
        with self.lock:
            self.data.update(kwargs)
            self.data["updated_at"] = now_text()
            self._write_locked()

    def update_stream(self, name, **kwargs):
        with self.lock:
            stream = self.data.setdefault("streams", {}).setdefault(name, {})
            stream.update(kwargs)
            stream["updated_at"] = now_text()
            self.data["updated_at"] = now_text()
            self._write_locked()

    def _write_locked(self):
        try:
            STATUS_FILE.parent.mkdir(parents=True, exist_ok=True)
            tmp = STATUS_FILE.with_suffix(".tmp")
            tmp.write_text(json.dumps(self.data, ensure_ascii=False, indent=2), encoding="utf-8")
            tmp.replace(STATUS_FILE)
        except Exception:
            pass


status = Status()


def ffmpeg_supports_encoder(name):
    try:
        result = subprocess.run(
            [FFMPEG_BIN, "-hide_banner", "-encoders"],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=5,
            check=False,
        )
        return name in result.stdout
    except Exception:
        return False


def choose_encoder():
    if ENCODER and ENCODER != "auto":
        return ENCODER
    if ffmpeg_supports_encoder("h264_nvenc"):
        return "h264_nvenc"
    if ffmpeg_supports_encoder("libx264"):
        return "libx264"
    return "h264"


def build_ffmpeg_cmd(rtsp_url, encoder):
    filters = []
    if HFLIP:
        filters.append("hflip")
    filters.append("format=yuv420p")

    cmd = [
        FFMPEG_BIN,
        "-hide_banner",
        "-loglevel",
        "warning",
        "-fflags",
        "nobuffer",
        "-f",
        "rawvideo",
        "-pix_fmt",
        "bgra",
        "-video_size",
        f"{WIDTH}x{HEIGHT}",
        "-framerate",
        str(FPS),
        "-i",
        "pipe:0",
        "-an",
        "-vf",
        ",".join(filters),
    ]

    if encoder == "h264_nvenc":
        cmd += [
            "-c:v",
            "h264_nvenc",
            "-preset",
            "p4",
            "-tune",
            "ll",
            "-rc",
            "cbr",
            "-b:v",
            BITRATE,
            "-maxrate",
            BITRATE,
            "-bufsize",
            BUFSIZE,
            "-g",
            str(int(FPS)),
            "-bf",
            "0",
        ]
    elif encoder == "libx264":
        cmd += [
            "-c:v",
            "libx264",
            "-preset",
            "veryfast",
            "-tune",
            "zerolatency",
            "-profile:v",
            "high",
            "-b:v",
            BITRATE,
            "-maxrate",
            BITRATE,
            "-bufsize",
            BUFSIZE,
            "-g",
            str(int(FPS)),
            "-keyint_min",
            str(int(FPS)),
            "-sc_threshold",
            "0",
        ]
    else:
        cmd += ["-c:v", encoder, "-b:v", BITRATE, "-g", str(int(FPS))]

    cmd += [
        "-pix_fmt",
        "yuv420p",
        "-f",
        "rtsp",
        "-rtsp_transport",
        "tcp",
        rtsp_url,
    ]
    return cmd


@dataclass
class RtspPublisher:
    name: str
    path: str
    encoder: str

    def __post_init__(self):
        self.frame_queue = queue.Queue(maxsize=QUEUE_SIZE)
        self.stop_event = threading.Event()
        self.proc = None
        self.log_file = None
        self.thread = threading.Thread(target=self._writer_loop, daemon=True)
        self.frames_in = 0
        self.frames_out = 0
        self.last_frame_at = None
        self.last_publish_at = None
        self.rtsp_url = f"rtsp://{RTSP_HOST}:{RTSP_PORT}/{self.path}"
        self.public_url = f"rtsp://{RTSP_PUBLIC_HOST}:{RTSP_PORT}/{self.path}"

    def start(self):
        self.thread.start()

    def stop(self):
        self.stop_event.set()
        self._stop_process()

    def deactivate(self):
        while True:
            try:
                self.frame_queue.get_nowait()
            except queue.Empty:
                break
        self.last_frame_at = None
        self.last_publish_at = None
        self._stop_process()
        status.update_stream(
            self.name,
            publishing=False,
            ffmpeg_pid=None,
            camera_attached=False,
            last_error=None,
        )

    def enqueue(self, image):
        try:
            frame = (time.time(), bytes(image.raw_data))
        except Exception as exc:
            status.update_stream(self.name, last_error=f"raw frame copy failed: {exc}")
            return

        self.frames_in += 1
        self.last_frame_at = time.time()
        # Keep latency bounded: old frames are never useful for mirrors.
        while True:
            try:
                self.frame_queue.put_nowait(frame)
                break
            except queue.Full:
                try:
                    self.frame_queue.get_nowait()
                except queue.Empty:
                    break

    def _start_process(self):
        self._stop_process()
        log_dir = Path("logs")
        log_dir.mkdir(exist_ok=True)
        self.log_file = open(log_dir / f"mirror_{self.name}.ffmpeg.log", "a", encoding="utf-8")
        cmd = build_ffmpeg_cmd(self.rtsp_url, self.encoder)
        self.log_file.write(f"\n[{now_text()}] starting: {' '.join(cmd)}\n")
        self.log_file.flush()
        self.proc = subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            stdout=self.log_file,
            stderr=self.log_file,
            bufsize=0,
        )
        status.update_stream(
            self.name,
            publishing=True,
            encoder=self.encoder,
            rtsp_url=self.public_url,
            ffmpeg_pid=self.proc.pid,
            last_error=None,
        )

    def _stop_process(self):
        if self.proc is not None:
            try:
                if self.proc.stdin:
                    self.proc.stdin.close()
            except Exception:
                pass
            try:
                self.proc.terminate()
                self.proc.wait(timeout=2)
            except Exception:
                try:
                    self.proc.kill()
                except Exception:
                    pass
            self.proc = None
        if self.log_file is not None:
            try:
                self.log_file.close()
            except Exception:
                pass
            self.log_file = None

    def _writer_loop(self):
        next_start = 0.0
        while not self.stop_event.is_set():
            try:
                frame_at, frame = self.frame_queue.get(timeout=0.5)
            except queue.Empty:
                continue

            if STALE_FRAME_SEC and time.time() - frame_at > STALE_FRAME_SEC:
                continue

            now = time.time()
            if self.proc is None or self.proc.poll() is not None:
                if now < next_start:
                    continue
                try:
                    self._start_process()
                except Exception as exc:
                    status.update_stream(self.name, publishing=False, last_error=str(exc))
                    next_start = time.time() + RECONNECT_SEC
                    continue

            try:
                self.proc.stdin.write(frame)
                self.frames_out += 1
                self.last_publish_at = time.time()
                if self.frames_out % max(int(FPS), 1) == 0:
                    status.update_stream(
                        self.name,
                        publishing=True,
                        frames_in=self.frames_in,
                        frames_out=self.frames_out,
                        last_frame_age_sec=round(time.time() - self.last_frame_at, 3)
                        if self.last_frame_at
                        else None,
                    )
            except Exception as exc:
                status.update_stream(self.name, publishing=False, last_error=str(exc))
                self._stop_process()
                next_start = time.time() + RECONNECT_SEC


def read_backend_actor_id():
    try:
        with urllib.request.urlopen(HEALTH_URL, timeout=1.0) as resp:
            data = json.loads(resp.read().decode("utf-8"))
    except Exception:
        return None
    diagnostics = data.get("diagnostics") or {}
    actor_id = diagnostics.get("actor_id")
    if actor_id is None:
        return None
    try:
        return int(actor_id)
    except Exception:
        return None


def read_backend_camera_view(default="follow"):
    try:
        with urllib.request.urlopen(HEALTH_URL, timeout=1.0) as resp:
            data = json.loads(resp.read().decode("utf-8"))
    except Exception:
        return normalize_camera_view(default)
    return normalize_camera_view(data.get("camera_view", default))


def find_ego_vehicle(world):
    actor_id = read_backend_actor_id()
    if actor_id is not None:
        try:
            actor = world.get_actor(actor_id)
        except Exception:
            actor = None
        if actor is not None and actor.is_alive and actor.type_id.startswith("vehicle."):
            return actor

    candidates = []
    actors = world.get_actors().filter("vehicle.*")
    for actor in actors:
        try:
            role_name = actor.attributes.get("role_name", "")
        except Exception:
            role_name = ""
        if role_name in {"normal_vehicle", "emergency_vehicle", "vehicle_model_fill", "bicycle"}:
            continue
        candidates.append(actor)

    if len(candidates) == 1:
        return candidates[0]
    for actor in candidates:
        if actor.attributes.get("role_name", "") in {"hero", "ego", "manual"}:
            return actor
    return None


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def vehicle_extents(vehicle):
    try:
        extent = vehicle.bounding_box.extent
        return (
            max(float(extent.x), 0.1),
            max(float(extent.y), 0.1),
            max(float(extent.z), 0.1),
        )
    except Exception:
        return 2.35, 0.95, 0.75


def adaptive_view_mount(vehicle, view):
    profile = dict(VIEW_PROFILES[normalize_camera_view(view)])
    length_half, width_half, height_half = vehicle_extents(vehicle)

    if normalize_camera_view(view) == "follow":
        distance = clamp(length_half * 2.75 + 2.1, 7.2, 11.8)
        height = clamp(height_half * 2.25 + 1.55, 3.0, 4.8)
        pitch = clamp(-2.7 - length_half * 0.55, -5.8, -3.8)
        fov = clamp(82.0 + width_half * 2.6, 84.0, 90.0)
        profile.update({
            "location": {"x": -distance, "y": 0.0, "z": height},
            "rotation": {"pitch": pitch, "yaw": 0.0, "roll": 0.0},
            "fov": fov,
            "attachment_type": "SpringArmGhost",
        })
    else:
        x = clamp(length_half * 0.18, 0.45, 1.15)
        y = -clamp(width_half * 0.38, 0.32, 0.72)
        z = clamp(height_half * 1.15 + 0.35, 1.15, 2.2)
        profile.update({
            "location": {"x": x, "y": y, "z": z},
            "rotation": {"pitch": -2.0, "yaw": 0.0, "roll": 0.0},
            "fov": 95.0,
            "attachment_type": "Rigid",
        })
    profile["view"] = normalize_camera_view(view)
    return profile


def adaptive_rear_mount(vehicle, name, cfg):
    profile = dict(cfg)
    length_half, width_half, height_half = vehicle_extents(vehicle)
    side = -1.0 if name == "carla_rear_left" else 1.0
    profile.update({
        "location": {
            "x": clamp(length_half * 0.32, 0.65, 2.80),
            "y": side * (width_half + 0.22),
            "z": max(1.15, min(height_half + 0.70, height_half * 1.05 + 0.35)),
        },
        "rotation": {
            "pitch": -5.0,
            "yaw": -145.0 if name == "carla_rear_left" else 145.0,
            "roll": 0.0,
        },
    })
    return profile


def adaptive_birdview_mount(vehicle, cfg):
    profile = dict(cfg)
    length_half, width_half, height_half = vehicle_extents(vehicle)
    # Fit the whole vehicle with some context, while keeping the car central.
    height = clamp(max(length_half * 3.1, width_half * 4.6, height_half + 5.5), 6.8, 17.5)
    fov = clamp(BIRDVIEW_FOV + width_half * 1.4, 92.0, 108.0)
    profile.update({
        "location": {"x": 0.0, "y": 0.0, "z": height},
        "rotation": {"pitch": -90.0, "yaw": 0.0, "roll": 0.0},
        "fov": fov,
        "attachment_type": "Rigid",
        "view": "birdview",
        "fit": "contain",
    })
    return profile


def make_transform(cfg):
    loc = cfg["location"]
    rot = cfg["rotation"]
    return carla.Transform(
        carla.Location(x=float(loc["x"]), y=float(loc["y"]), z=float(loc["z"])),
        carla.Rotation(
            pitch=float(rot["pitch"]),
            yaw=float(rot["yaw"]),
            roll=float(rot.get("roll", 0.0)),
        ),
    )


def spawn_cameras(world, vehicle, publishers, mounts=None):
    mounts = mounts or MOUNTS
    bp_lib = world.get_blueprint_library()
    camera_bp = bp_lib.find("sensor.camera.rgb")
    camera_bp.set_attribute("image_size_x", str(WIDTH))
    camera_bp.set_attribute("image_size_y", str(HEIGHT))
    if camera_bp.has_attribute("sensor_tick"):
        camera_bp.set_attribute("sensor_tick", str(round(1.0 / max(FPS, 0.1), 6)))

    sensors = []
    for name, cfg in mounts.items():
        if name in REAR_MOUNTS:
            cfg = adaptive_rear_mount(vehicle, name, cfg)
        elif name in BIRDVIEW_MOUNTS:
            cfg = adaptive_birdview_mount(vehicle, cfg)
        camera_bp.set_attribute("fov", str(float(cfg["fov"])))
        attachment_type = getattr(
            carla.AttachmentType,
            cfg.get("attachment_type", "Rigid"),
            carla.AttachmentType.Rigid,
        )
        sensor = world.spawn_actor(
            camera_bp,
            make_transform(cfg),
            attach_to=vehicle,
            attachment_type=attachment_type,
        )
        publisher = publishers[name]
        sensor.listen(lambda image, pub=publisher: pub.enqueue(image))
        sensors.append(sensor)
        status.update_stream(
            name,
            sensor_id=sensor.id,
            camera_attached=True,
            mount=cfg,
            attachment_type=cfg.get("attachment_type", "Rigid"),
            rtsp_url=publisher.public_url,
            width=WIDTH,
            height=HEIGHT,
            fps=FPS,
        )
    return sensors


def destroy_sensors(sensors):
    for sensor in sensors:
        try:
            sensor.stop()
        except Exception:
            pass
        try:
            if sensor.is_alive:
                sensor.destroy()
        except Exception:
            pass


def deactivate_publishers(publishers, names=None):
    selected = names if names is not None else list(publishers.keys())
    for name in selected:
        publisher = publishers.get(name)
        if publisher is not None:
            publisher.deactivate()


def main():
    encoder = choose_encoder()
    publishers = {
        name: RtspPublisher(name=name, path=cfg["path"], encoder=encoder)
        for name, cfg in MOUNTS.items()
    }
    for publisher in publishers.values():
        publisher.start()

    client = None
    world = None
    current_actor_id = None
    current_view = None
    active_view_sensors = []
    rear_sensors = []
    birdview_sensors = []

    print(
        f"mirror stream service starting: {WIDTH}x{HEIGHT}@{FPS} H264, "
        f"encoder={encoder}, rtsp=:{RTSP_PORT}",
        flush=True,
    )
    for name, publisher in publishers.items():
        print(f"{name}: {publisher.public_url}", flush=True)

    try:
        while True:
            try:
                if client is None:
                    client = carla.Client(CARLA_HOST, CARLA_PORT)
                    client.set_timeout(CARLA_TIMEOUT)
                world = client.get_world()
                status.update(carla_connected=True, last_error=None)
            except Exception as exc:
                status.update(carla_connected=False, ego_actor_id=None, last_error=str(exc))
                destroy_sensors(active_view_sensors)
                destroy_sensors(rear_sensors)
                destroy_sensors(birdview_sensors)
                active_view_sensors = []
                rear_sensors = []
                birdview_sensors = []
                deactivate_publishers(publishers)
                current_actor_id = None
                current_view = None
                client = None
                time.sleep(RECONNECT_SEC)
                continue

            try:
                vehicle = find_ego_vehicle(world)
            except Exception as exc:
                status.update(
                    carla_connected=False,
                    ego_actor_id=None,
                    last_error=f"CARLA actor scan failed: {exc}",
                )
                destroy_sensors(active_view_sensors)
                destroy_sensors(rear_sensors)
                destroy_sensors(birdview_sensors)
                active_view_sensors = []
                rear_sensors = []
                birdview_sensors = []
                deactivate_publishers(publishers)
                current_actor_id = None
                current_view = None
                client = None
                time.sleep(RECONNECT_SEC)
                continue
            if vehicle is None:
                if active_view_sensors or rear_sensors:
                    destroy_sensors(active_view_sensors)
                    destroy_sensors(rear_sensors)
                    destroy_sensors(birdview_sensors)
                    active_view_sensors = []
                    rear_sensors = []
                    birdview_sensors = []
                    deactivate_publishers(publishers)
                    current_actor_id = None
                    current_view = None
                status.update(ego_actor_id=None, last_error="waiting for ego vehicle")
                time.sleep(EGO_SCAN_SEC)
                continue

            requested_view = read_backend_camera_view(current_view or "follow")

            if SINGLE_ACTIVE_VIEW:
                actor_changed = current_actor_id != vehicle.id
                if actor_changed:
                    destroy_sensors(active_view_sensors)
                    destroy_sensors(rear_sensors)
                    destroy_sensors(birdview_sensors)
                    active_view_sensors = []
                    rear_sensors = []
                    birdview_sensors = []
                    deactivate_publishers(publishers, ["carla_view", *REAR_MOUNTS.keys()])

                active_view_changed = current_view != requested_view
                if actor_changed or not active_view_sensors or active_view_changed:
                    destroy_sensors(active_view_sensors)
                    active_view_sensors = []
                    deactivate_publishers(publishers, ["carla_view"])
                    try:
                        active_mount = {
                            "carla_view": {
                                **adaptive_view_mount(vehicle, requested_view),
                                "path": ACTIVE_VIEW_PATH,
                            }
                        }
                        active_view_sensors = spawn_cameras(world, vehicle, publishers, active_mount)
                        current_view = requested_view
                        current_actor_id = vehicle.id
                        status.update(
                            ego_actor_id=vehicle.id,
                            active_view=current_view,
                            active_view_url=view_public_url(),
                            last_error=None,
                        )
                        print(
                            f"center camera view attached to ego actor {vehicle.id}: {current_view}",
                            flush=True,
                        )
                    except Exception as exc:
                        destroy_sensors(active_view_sensors)
                        active_view_sensors = []
                        current_view = None
                        status.update(ego_actor_id=vehicle.id, last_error=f"center camera attach failed: {exc}")
                        time.sleep(RECONNECT_SEC)
                        continue

                rear_mounts = {name: cfg for name, cfg in REAR_MOUNTS.items() if name in publishers}
                if rear_mounts and (actor_changed or not rear_sensors):
                    destroy_sensors(rear_sensors)
                    rear_sensors = []
                    deactivate_publishers(publishers, rear_mounts.keys())
                    try:
                        rear_sensors = spawn_cameras(world, vehicle, publishers, rear_mounts)
                        current_actor_id = vehicle.id
                        status.update(ego_actor_id=vehicle.id, last_error=None)
                        print(f"side mirror cameras attached to ego actor {vehicle.id}", flush=True)
                    except Exception as exc:
                        destroy_sensors(rear_sensors)
                        rear_sensors = []
                        status.update(ego_actor_id=vehicle.id, last_error=f"side mirror attach failed: {exc}")
                        time.sleep(RECONNECT_SEC)
                        continue

                birdview_mounts = {name: cfg for name, cfg in BIRDVIEW_MOUNTS.items() if name in publishers}
                if birdview_mounts and (actor_changed or not birdview_sensors):
                    destroy_sensors(birdview_sensors)
                    birdview_sensors = []
                    deactivate_publishers(publishers, birdview_mounts.keys())
                    try:
                        birdview_sensors = spawn_cameras(world, vehicle, publishers, birdview_mounts)
                        current_actor_id = vehicle.id
                        status.update(ego_actor_id=vehicle.id, last_error=None)
                        print(f"birdview camera attached to ego actor {vehicle.id}", flush=True)
                    except Exception as exc:
                        destroy_sensors(birdview_sensors)
                        birdview_sensors = []
                        status.update(ego_actor_id=vehicle.id, last_error=f"birdview attach failed: {exc}")
                        time.sleep(RECONNECT_SEC)
                        continue
            else:
                all_sensors = active_view_sensors + rear_sensors + birdview_sensors
                if current_actor_id != vehicle.id or not all_sensors:
                    destroy_sensors(all_sensors)
                    active_view_sensors = []
                    rear_sensors = []
                    birdview_sensors = []
                    try:
                        active_view_sensors = spawn_cameras(world, vehicle, publishers)
                        current_view = requested_view
                        status.update(ego_actor_id=vehicle.id, active_view=current_view, last_error=None)
                        print(f"mirror cameras attached to ego actor {vehicle.id}", flush=True)
                        current_actor_id = vehicle.id
                    except Exception as exc:
                        destroy_sensors(active_view_sensors)
                        destroy_sensors(rear_sensors)
                        destroy_sensors(birdview_sensors)
                        active_view_sensors = []
                        rear_sensors = []
                        birdview_sensors = []
                        current_actor_id = None
                        current_view = None
                        status.update(ego_actor_id=vehicle.id, last_error=f"camera attach failed: {exc}")
                        time.sleep(RECONNECT_SEC)
                        continue

            time.sleep(EGO_SCAN_SEC)
    finally:
        destroy_sensors(active_view_sensors)
        destroy_sensors(rear_sensors)
        destroy_sensors(birdview_sensors)
        for publisher in publishers.values():
            publisher.stop()
        status.update(running=False)


if __name__ == "__main__":
    main()
