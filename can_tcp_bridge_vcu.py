import json
import math
import os
import socket
import struct
import sys
import time
from pathlib import Path

import cantools


UDP_IP = os.environ.get("CARLA_MANUAL_UDP_IP", "127.0.0.1")
UDP_PORT_TX = int(os.environ.get("CARLA_MANUAL_CTRL_PORT", "5001"))
UDP_PORT_RX = int(os.environ.get("CARLA_MANUAL_TELEM_PORT", "5000"))

TCP_CAN_HOST = os.environ.get("CARLA_TCP_CAN_HOST", "192.168.110.112")
TCP_CAN_PORT = int(os.environ.get("CARLA_TCP_CAN_PORT", "4001"))
TCP_CONNECT_TIMEOUT_SEC = float(os.environ.get("CARLA_TCP_CAN_CONNECT_TIMEOUT_SEC", "3.0"))
TCP_RECONNECT_SEC = float(os.environ.get("CARLA_TCP_CAN_RECONNECT_SEC", "3.0"))

PRINT_INTERVAL_SEC = float(os.environ.get("CARLA_BRIDGE_PRINT_INTERVAL_SEC", "0.25"))
STATUS_FILE = Path(os.environ.get("MANUAL_BRIDGE_STATUS_FILE", "logs/manual_bridge_status.json"))
SEND_STATE = os.environ.get("CARLA_CAN_SEND_STATE", "1").strip().lower() not in ("0", "false", "no", "off")
RAW_DEBUG = os.environ.get("CARLA_CAN_RAW_DEBUG", "0").strip().lower() in ("1", "true", "yes", "on")

CONTROL_MESSAGE = os.environ.get("CARLA_CAN_CONTROL_MESSAGE", "Cockpit_Control")
STATE_MESSAGE = os.environ.get("CARLA_CAN_STATE_MESSAGE", "Carla_EV_state")

STEER_MAX_DEG = float(os.environ.get("CARLA_CAN_STEER_MAX_DEG", "540.0"))
STATE_GEAR_NEUTRAL = int(os.environ.get("CARLA_CAN_STATE_GEAR_NEUTRAL", "1"), 0)
STATE_GEAR_DRIVE = int(os.environ.get("CARLA_CAN_STATE_GEAR_DRIVE", "2"), 0)
STATE_GEAR_REVERSE = int(os.environ.get("CARLA_CAN_STATE_GEAR_REVERSE", "4"), 0)


def parse_int_set(value):
    result = set()
    for item in str(value or "").replace(";", ",").split(","):
        item = item.strip()
        if not item:
            continue
        result.add(int(item, 0))
    return result


# NJ0515 DBC uses 4=R, 2=D, 1=N, 0=P. Keep this configurable for cockpit
# firmware variants, but do not treat P(0) as reverse by default.
REVERSE_GEARS = parse_int_set(os.environ.get("CARLA_CAN_REVERSE_GEARS", "4"))
DRIVE_GEARS = parse_int_set(os.environ.get("CARLA_CAN_DRIVE_GEARS", "2,3"))

sim_state = {
    "speed_ms": 0.0,
    "steer_angle_deg": 0.0,
    "gear": STATE_GEAR_NEUTRAL,
    "eps_torque_nm": 0.0,
}

bridge_status = {
    "pid": os.getpid(),
    "running": True,
    "mode": "tcp-can",
    "dbc_path": None,
    "tcp_host": TCP_CAN_HOST,
    "tcp_port": TCP_CAN_PORT,
    "tcp_connected": False,
    "can_connected": False,
    "can_tx_ok": False,
    "can_rx_count": 0,
    "control_rx_count": 0,
    "can_tx_count": 0,
    "udp_ctrl_tx_count": 0,
    "state_tx_enabled": SEND_STATE,
    "raw_debug": RAW_DEBUG,
    "heartbeat_count": 0,
    "rx_resync_count": 0,
    "rx_decode_fail_count": 0,
    "tx_fail_count": 0,
    "last_tcp_connected_at": None,
    "last_tcp_disconnected_at": None,
    "last_can_rx_at": None,
    "last_can_tx_ok_at": None,
    "last_udp_ctrl_at": None,
    "last_heartbeat_at": None,
    "last_frame_id": None,
    "last_frame_hex": None,
    "last_control_frame_hex": None,
    "last_state_frame_hex": None,
    "last_state_values": None,
    "last_control": None,
    "last_decoded_control": None,
    "last_error": None,
    "updated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
}
_last_status_write = 0.0


def now_text():
    return time.strftime("%Y-%m-%d %H:%M:%S")


def write_status(force=False, **updates):
    global _last_status_write
    bridge_status.update(updates)
    bridge_status["updated_at"] = now_text()
    current = time.time()
    if not force and current - _last_status_write < 0.2:
        return
    _last_status_write = current
    try:
        STATUS_FILE.parent.mkdir(parents=True, exist_ok=True)
        tmp = STATUS_FILE.with_suffix(".tmp")
        tmp.write_text(json.dumps(bridge_status, ensure_ascii=False, indent=2), encoding="utf-8")
        tmp.replace(STATUS_FILE)
    except Exception:
        pass


def find_by_token(data, token, default=None):
    if not isinstance(data, dict):
        return default
    return next((value for key, value in data.items() if token in str(key)), default)


def dbc_candidates():
    candidates = []
    env_path = os.environ.get("CARLA_CAN_DBC", "")
    if env_path:
        candidates.append(env_path)
    candidates.append(str(Path("can") / "智能座舱CAN协议-NJ0515.dbc"))
    candidates.append("智能座舱CAN协议-NJ0515.dbc")
    candidates.extend(str(path) for path in Path.cwd().glob("*NJ0515.dbc"))
    candidates.extend(str(path) for path in Path.cwd().glob("*NJ0423.dbc"))
    candidates.append("vcu_protocol.dbc")

    ordered = []
    seen = set()
    for candidate in candidates:
        if candidate and candidate not in seen:
            seen.add(candidate)
            ordered.append(candidate)
    return ordered


def clean_dbc_text(text):
    clean_lines = []
    skip_vector_msg = False
    skip_comment = False
    for line in text.splitlines(True):
        stripped = line.strip()
        if skip_comment:
            if stripped.endswith('";') or stripped == ";":
                skip_comment = False
            continue

        if stripped.startswith("BO_ 3221225472 "):
            skip_vector_msg = True
            continue
        if skip_vector_msg and stripped.startswith("SG_ "):
            continue
        skip_vector_msg = False

        if stripped.startswith("CM_"):
            if not stripped.endswith('";'):
                skip_comment = True
            continue

        if stripped.startswith(("VAL_", "BA_", "SIG_", "VAL_TABLE_", "CAT_", "SGTYPE_")):
            continue
        clean_lines.append(line.encode("ascii", errors="ignore").decode("ascii", errors="ignore"))
    return "".join(clean_lines)


def load_database():
    last_error = None
    for candidate in dbc_candidates():
        path = Path(candidate)
        if not path.exists():
            continue
        for encoding in ("utf-8", "gbk"):
            try:
                text = path.read_text(encoding=encoding, errors="ignore")
                try:
                    db = cantools.database.load_string(text, strict=False)
                except Exception:
                    db = cantools.database.load_string(clean_dbc_text(text), strict=False)
                db.get_message_by_name(CONTROL_MESSAGE)
                db.get_message_by_name(STATE_MESSAGE)
                write_status(force=True, dbc_path=str(path), last_error=None)
                print(f"[tcp-can] DBC loaded: {path} ({len(db.messages)} messages)", flush=True)
                return db, path
            except Exception as exc:
                last_error = exc
        print(f"[tcp-can] DBC load failed for {path}: {last_error}", flush=True)
    raise RuntimeError(f"no usable DBC found, last_error={last_error}")


def bind_udp_rx():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    if hasattr(socket, "SO_REUSEPORT"):
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except Exception:
            pass
    sock.bind((UDP_IP, UDP_PORT_RX))
    sock.setblocking(False)
    return sock


def connect_tcp_can():
    while True:
        try:
            print(f"[tcp-can] connecting to GCAN-212 {TCP_CAN_HOST}:{TCP_CAN_PORT} ...", flush=True)
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(TCP_CONNECT_TIMEOUT_SEC)
            sock.connect((TCP_CAN_HOST, TCP_CAN_PORT))
            sock.settimeout(0.02)
            write_status(
                force=True,
                tcp_connected=True,
                can_connected=True,
                last_tcp_connected_at=time.time(),
                last_error=None,
            )
            print("[tcp-can] connected, waiting for CAN frames", flush=True)
            return sock
        except Exception as exc:
            write_status(
                force=True,
                tcp_connected=False,
                can_connected=False,
                last_error=f"TCP connect failed: {exc}",
            )
            print(f"[tcp-can] connect failed: {exc}; retrying in {TCP_RECONNECT_SEC}s", flush=True)
            time.sleep(TCP_RECONNECT_SEC)


def parse_telemetry(packet):
    telemetry = json.loads(packet.decode("utf-8"))
    kinematics = find_by_token(telemetry, "1_", {})
    control_state = find_by_token(telemetry, "3_", {})

    velocity = find_by_token(kinematics, "3_", [0.0, 0.0, 0.0])
    if not isinstance(velocity, list) or len(velocity) < 3:
        velocity = [0.0, 0.0, 0.0]

    speed_ms = math.sqrt(float(velocity[0]) ** 2 + float(velocity[1]) ** 2 + float(velocity[2]) ** 2)
    reverse = bool(find_by_token(control_state, "14_", False))
    steer_ratio = float(find_by_token(control_state, "11_", 0.0) or 0.0)
    speed_factor = min(1.0, speed_ms / 10.0)

    sim_state["speed_ms"] = speed_ms
    sim_state["gear"] = STATE_GEAR_REVERSE if reverse else STATE_GEAR_DRIVE
    sim_state["steer_angle_deg"] = max(-STEER_MAX_DEG, min(STEER_MAX_DEG, steer_ratio * STEER_MAX_DEG))
    sim_state["eps_torque_nm"] = max(-12.0, min(12.0, steer_ratio * 8.0 * speed_factor))


def pack_gcan_frame(msg_def, payload):
    frame_info = len(payload) & 0x0F
    if getattr(msg_def, "is_extended_frame", False):
        frame_info |= 0x80
    return struct.pack(">B I 8s", frame_info, int(msg_def.frame_id), payload.ljust(8, b"\x00"))


def pack_state_frame(db):
    msg = db.get_message_by_name(STATE_MESSAGE)
    data = {signal.name: 0 for signal in msg.signals}
    data.update(
        {
            "Carla_Gear": int(sim_state["gear"]),
            "Carla_EV_Speed": max(0.0, min(100.0, float(sim_state["speed_ms"]))),
            "Carla_Steer_Angle": max(-STEER_MAX_DEG, min(STEER_MAX_DEG, float(sim_state["steer_angle_deg"]))),
            "Carla_EPS_Torque": max(-12.0, min(12.0, float(sim_state["eps_torque_nm"]))),
        }
    )
    encoded = msg.encode(data, strict=False)
    return pack_gcan_frame(msg, encoded)


def drain_udp_and_send_state(sock_rx, tcp_sock, db, last_state_tx):
    latest = None
    while True:
        try:
            latest, _ = sock_rx.recvfrom(8192)
        except BlockingIOError:
            break

    if latest is not None:
        try:
            parse_telemetry(latest)
        except Exception as exc:
            write_status(last_error=f"telemetry parse warning: {exc}")

    now = time.time()
    if now - last_state_tx < 0.02:
        return last_state_tx
    if not SEND_STATE:
        write_status(can_tx_ok=False, state_tx_enabled=False)
        return now

    try:
        state_frame = pack_state_frame(db)
        tcp_sock.sendall(state_frame)
        write_status(
            can_tx_ok=True,
            state_tx_enabled=True,
            can_tx_count=bridge_status["can_tx_count"] + 1,
            last_can_tx_ok_at=now,
            last_state_frame_hex=state_frame.hex(" "),
            last_state_values={
                "gear": sim_state["gear"],
                "speed_ms": round(sim_state["speed_ms"], 4),
                "steer_angle_deg": round(sim_state["steer_angle_deg"], 3),
                "eps_torque_nm": round(sim_state["eps_torque_nm"], 3),
            },
            last_error=None,
        )
    except Exception as exc:
        write_status(
            force=True,
            can_tx_ok=False,
            tx_fail_count=bridge_status["tx_fail_count"] + 1,
            last_error=f"CAN state send failed: {exc}",
        )
        raise
    return now


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def decode_control_payload(db, frame_id, payload):
    decoded = db.decode_message(frame_id, payload, decode_choices=False)

    throttle_pct = float(decoded.get("Cockpit_ACC", 0) or 0)
    brake_pct = float(decoded.get("Cockpit_Beak", 0) or 0)
    steer_deg = float(decoded.get("Cockpit_EPS_Angle", 0) or 0)
    raw_gear = int(decoded.get("Cockpit_Gear", 0) or 0)
    key_break = int(decoded.get("Cockpit_Key_Break", 0) or 0)
    key_stop = int(decoded.get("Cockpit_Key_Stop", 0) or 0)
    key_d = int(decoded.get("Cockpit_Key_D", 0) or 0)
    key_r = int(decoded.get("Cockpit_Key_R", 0) or 0)
    key_park = int(decoded.get("Cockpit_Key_Park", 0) or 0)
    key_horn = int(decoded.get("Cockpit_Key_Horn", 0) or 0)
    key_hazard = int(decoded.get("Cockpit_Key_Hazard", 0) or 0)
    sw_auto = int(decoded.get("Cockpit_SW_Auto", 0) or 0)

    park_requested = bool(key_park and not key_r)
    reverse = bool(key_r) or (raw_gear in REVERSE_GEARS and not key_d and not park_requested)
    drive = bool(key_d) or raw_gear in DRIVE_GEARS

    throttle = clamp(throttle_pct / 100.0, 0.0, 1.0)
    brake = clamp(brake_pct / 100.0, 0.0, 1.0)
    steer = clamp(steer_deg / STEER_MAX_DEG, -1.0, 1.0)
    forced_stop = bool(key_break or key_stop or park_requested)

    if forced_stop:
        throttle = 0.0
        brake = 1.0
    elif not reverse and not drive:
        throttle = 0.0

    command = {
        "steer": steer,
        "throttle": throttle,
        "brake": brake,
        "reverse": reverse,
        "hand_brake": forced_stop,
    }
    debug = {
        "steer_deg": steer_deg,
        "throttle_pct": throttle_pct,
        "brake_pct": brake_pct,
        "raw_gear": raw_gear,
        "key_d": key_d,
        "key_r": key_r,
        "key_park": key_park,
        "key_horn": key_horn,
        "key_hazard": key_hazard,
        "key_break": key_break,
        "key_stop": key_stop,
        "sw_auto": sw_auto,
        "reverse": reverse,
        "drive": drive,
    }
    return command, debug, decoded


def parse_tcp_frames(buffer, db, sock_tx, last_print, hz_state):
    control_def = db.get_message_by_name(CONTROL_MESSAGE)
    control_frame_id = int(control_def.frame_id)

    while len(buffer) >= 13:
        first = buffer[0]
        if first == 0xAA:
            buffer = buffer[13:]
            write_status(
                heartbeat_count=bridge_status["heartbeat_count"] + 1,
                last_heartbeat_at=time.time(),
            )
            continue

        dlc = first & 0x0F
        if dlc > 8:
            buffer = buffer[1:]
            write_status(rx_resync_count=bridge_status["rx_resync_count"] + 1)
            continue

        frame = buffer[:13]
        buffer = buffer[13:]
        frame_id = struct.unpack(">I", frame[1:5])[0]
        payload = frame[5 : 5 + dlc]
        frame_hex = frame.hex(" ")
        now = time.time()

        write_status(
            can_rx_count=bridge_status["can_rx_count"] + 1,
            last_can_rx_at=now,
            last_frame_id=frame_id,
            last_frame_hex=frame_hex,
        )

        if frame_id != control_frame_id:
            continue

        try:
            command, debug, decoded = decode_control_payload(db, frame_id, payload)
        except Exception as exc:
            write_status(
                rx_decode_fail_count=bridge_status["rx_decode_fail_count"] + 1,
                last_error=f"decode frame 0x{frame_id:X} failed: {exc}",
            )
            continue

        sock_tx.sendto(json.dumps(command, separators=(",", ":")).encode("utf-8"), (UDP_IP, UDP_PORT_TX))

        if hz_state["last"] > 0:
            dt = now - hz_state["last"]
            if dt > 0:
                inst_hz = 1.0 / dt
                hz_state["hz"] = inst_hz if hz_state["hz"] <= 0 else hz_state["hz"] * 0.9 + inst_hz * 0.1
        hz_state["last"] = now

        write_status(
            control_rx_count=bridge_status["control_rx_count"] + 1,
            udp_ctrl_tx_count=bridge_status["udp_ctrl_tx_count"] + 1,
            last_udp_ctrl_at=now,
            last_control_frame_hex=frame_hex,
            last_control=command,
            last_decoded_control=debug,
            last_error=None,
        )

        if now - last_print >= PRINT_INTERVAL_SEC:
            gear_text = "R" if debug["reverse"] else ("D" if debug["drive"] else "N/P")
            stop_text = "STOP" if command["hand_brake"] else "OK"
            print(
                f"[tcp-can] steer={debug['steer_deg']:>6.1f}deg "
                f"thr={debug['throttle_pct']:>5.1f}% "
                f"brk={debug['brake_pct']:>5.1f}% "
                f"gear={gear_text}(raw={debug['raw_gear']}) "
                f"horn={debug['key_horn']} haz={debug['key_hazard']} "
                f"auto={debug['sw_auto']} {stop_text} hz={hz_state['hz']:>5.1f}",
                flush=True,
            )
            last_print = now

    return buffer, last_print


def run_bridge():
    db, dbc_path = load_database()
    sock_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_rx = bind_udp_rx()

    print(
        f"[tcp-can] bridge online, dbc={dbc_path}, "
        f"tcp={TCP_CAN_HOST}:{TCP_CAN_PORT}, "
        f"udp_rx={UDP_IP}:{UDP_PORT_RX}, udp_tx={UDP_IP}:{UDP_PORT_TX}, "
        f"reverse_gears={sorted(REVERSE_GEARS)}, drive_gears={sorted(DRIVE_GEARS)}",
        flush=True,
    )

    try:
        while True:
            tcp_sock = connect_tcp_can()
            buffer = b""
            last_state_tx = 0.0
            last_print = 0.0
            hz_state = {"last": 0.0, "hz": 0.0}
            try:
                while True:
                    last_state_tx = drain_udp_and_send_state(sock_rx, tcp_sock, db, last_state_tx)
                    try:
                        chunk = tcp_sock.recv(4096)
                    except socket.timeout:
                        continue
                    if not chunk:
                        raise ConnectionError("remote closed")
                    buffer += chunk
                    buffer, last_print = parse_tcp_frames(buffer, db, sock_tx, last_print, hz_state)
            except Exception as exc:
                try:
                    tcp_sock.close()
                except Exception:
                    pass
                write_status(
                    force=True,
                    tcp_connected=False,
                    can_connected=False,
                    can_tx_ok=False,
                    last_tcp_disconnected_at=time.time(),
                    last_error=f"TCP disconnected: {exc}",
                )
                print(f"[tcp-can] disconnected: {exc}; reconnecting in {TCP_RECONNECT_SEC}s", flush=True)
                time.sleep(TCP_RECONNECT_SEC)
    finally:
        write_status(force=True, running=False, tcp_connected=False, can_connected=False)
        try:
            sock_tx.close()
            sock_rx.close()
        except Exception:
            pass


if __name__ == "__main__":
    if "--check" in sys.argv:
        db, dbc_path = load_database()
        control = db.get_message_by_name(CONTROL_MESSAGE)
        state = db.get_message_by_name(STATE_MESSAGE)
        print(f"[tcp-can] check ok: dbc={dbc_path}", flush=True)
        print(f"[tcp-can] control={CONTROL_MESSAGE} frame_id={control.frame_id} dlc={control.length}", flush=True)
        print(f"[tcp-can] state={STATE_MESSAGE} frame_id={state.frame_id} dlc={state.length}", flush=True)
        write_status(force=True, running=False, tcp_connected=False, can_connected=False, last_error=None)
        sys.exit(0)

    try:
        run_bridge()
    except KeyboardInterrupt:
        print("\n[tcp-can] stopped", flush=True)
