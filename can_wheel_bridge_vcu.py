import json
import math
import os
import socket
import time
from pathlib import Path

import can
import cantools


UDP_IP = os.environ.get("CARLA_MANUAL_UDP_IP", "127.0.0.1")
UDP_PORT_TX = int(os.environ.get("CARLA_MANUAL_CTRL_PORT", "5001"))
UDP_PORT_RX = int(os.environ.get("CARLA_MANUAL_TELEM_PORT", "5000"))

CAN_BUSTYPE = os.environ.get("CARLA_CAN_BUSTYPE", "socketcan")
CAN_CHANNEL = os.environ.get("CARLA_CAN_CHANNEL", "can0")
CAN_BITRATE = int(os.environ.get("CARLA_CAN_BITRATE", "500000"))
CAN_RECONNECT_SEC = float(os.environ.get("CARLA_CAN_RECONNECT_SEC", "2.0"))
PRINT_INTERVAL_SEC = float(os.environ.get("CARLA_BRIDGE_PRINT_INTERVAL_SEC", "0.25"))
STATUS_FILE = Path(os.environ.get("MANUAL_BRIDGE_STATUS_FILE", "logs/manual_bridge_status.json"))

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
    "dbc_path": None,
    "can_connected": False,
    "can_tx_ok": False,
    "last_can_tx_ok_at": None,
    "last_can_rx_at": None,
    "last_udp_ctrl_at": None,
    "last_error": None,
    "tx_fail_count": 0,
    "updated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
}


def find_first(data, keys, default=None):
    for key in keys:
        if isinstance(data, dict) and key in data:
            return data[key]
    return default


def find_by_token(data, token, default=None):
    if not isinstance(data, dict):
        return default
    return next((value for key, value in data.items() if token in str(key)), default)


def dbc_candidates():
    candidates = []
    env_path = os.environ.get("CARLA_CAN_DBC", "")
    if env_path:
        candidates.append(env_path)
    candidates.extend(str(path) for path in Path.cwd().glob("can/NJ0515.dbc"))
    candidates.extend(str(path) for path in Path.cwd().glob("*NJ0515.dbc"))
    candidates.extend(str(path) for path in Path.cwd().glob("can/*NJ0515*.dbc"))
    candidates.extend(str(path) for path in Path.cwd().glob("*NJ0423.dbc"))
    candidates.extend(str(path) for path in Path.cwd().glob("can/*NJ0423*.dbc"))
    candidates.append("vcu_protocol.dbc")
    seen = set()
    ordered = []
    for candidate in candidates:
        if candidate and candidate not in seen:
            seen.add(candidate)
            ordered.append(candidate)
    return ordered


def write_status(**updates):
    bridge_status.update(updates)
    bridge_status["updated_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
    try:
        STATUS_FILE.parent.mkdir(exist_ok=True)
        tmp = STATUS_FILE.with_suffix(".tmp")
        tmp.write_text(json.dumps(bridge_status, ensure_ascii=False, indent=2), encoding="utf-8")
        tmp.replace(STATUS_FILE)
    except Exception:
        pass


def clean_dbc_text(text):
    clean_lines = []
    for line in text.splitlines(True):
        stripped = line.strip()
        if stripped.startswith(("CM_", "VAL_", "BA_", "SIG_", "VAL_TABLE_")):
            continue
        clean_lines.append(line.encode("ascii", errors="ignore").decode("ascii", errors="ignore"))
    return "".join(clean_lines)


def load_database():
    last_error = None
    for candidate in dbc_candidates():
        if not candidate:
            continue
        path = Path(candidate)
        if not path.exists():
            continue
        try:
            text = path.read_text(encoding="utf-8", errors="ignore")
            try:
                db = cantools.database.load_string(text)
            except Exception:
                db = cantools.database.load_string(clean_dbc_text(text))
            db.get_message_by_name(CONTROL_MESSAGE)
            db.get_message_by_name(STATE_MESSAGE)
            write_status(dbc_path=str(path), last_error=None)
            print(f"[bridge] DBC loaded: {path} ({len(db.messages)} messages)", flush=True)
            return db, path
        except Exception as exc:
            last_error = exc
            print(f"[bridge] DBC load failed for {path}: {exc}", flush=True)
    raise RuntimeError(f"no usable DBC found, last_error={last_error}")


def open_can_bus():
    return can.interface.Bus(bustype=CAN_BUSTYPE, channel=CAN_CHANNEL, bitrate=CAN_BITRATE)


def shutdown_bus(bus):
    if bus is None:
        return
    try:
        bus.shutdown()
    except Exception:
        pass
    write_status(can_connected=False)


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


def parse_telemetry(packet):
    telemetry = json.loads(packet.decode("utf-8"))
    kinematics = find_by_token(telemetry, "1_", {})
    control_state = find_by_token(telemetry, "3_", {})

    velocity = find_by_token(kinematics, "3_", [0.0, 0.0, 0.0])
    if not isinstance(velocity, list) or len(velocity) < 3:
        velocity = [0.0, 0.0, 0.0]

    sim_state["speed_ms"] = math.sqrt(float(velocity[0]) ** 2 + float(velocity[1]) ** 2 + float(velocity[2]) ** 2)
    reverse = bool(find_by_token(control_state, "14_", False))
    sim_state["gear"] = STATE_GEAR_REVERSE if reverse else STATE_GEAR_DRIVE
    steer_ratio = float(find_by_token(control_state, "11_", 0.0) or 0.0)
    sim_state["steer_angle_deg"] = max(-STEER_MAX_DEG, min(STEER_MAX_DEG, steer_ratio * STEER_MAX_DEG))
    speed_factor = min(1.0, sim_state["speed_ms"] / 10.0)
    sim_state["eps_torque_nm"] = max(-12.0, min(12.0, steer_ratio * 8.0 * speed_factor))


def pack_state_message(db):
    msg = db.get_message_by_name(STATE_MESSAGE)
    data = {
        "Carla_Gear": int(sim_state["gear"]),
        "Carla_EV_Speed": max(0.0, min(100.0, float(sim_state["speed_ms"]))),
        "Carla_Steer_Angle": max(-STEER_MAX_DEG, min(STEER_MAX_DEG, float(sim_state["steer_angle_deg"]))),
        "Carla_EPS_Torque": max(-12.0, min(12.0, float(sim_state["eps_torque_nm"]))),
    }
    return msg, msg.encode(data)


def send_state_can(db, bus):
    if bus is None:
        return
    msg, encoded = pack_state_message(db)
    bus.send(can.Message(arbitration_id=msg.frame_id, data=encoded, is_extended_id=False))


def drain_udp(sock_rx, db, bus, last_state_tx):
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
            print(f"[bridge] telemetry parse warning: {exc}", flush=True)

    now = time.time()
    if now - last_state_tx >= 0.02:
        try:
            send_state_can(db, bus)
            write_status(can_tx_ok=True, last_can_tx_ok_at=now, last_error=None)
        except Exception as exc:
            write_status(can_tx_ok=False, tx_fail_count=bridge_status["tx_fail_count"] + 1, last_error=str(exc))
            print(f"[bridge] CAN state send failed: {exc}", flush=True)
        return now
    return last_state_tx


def decode_control_message(db, can_msg):
    control_def = db.get_message_by_name(CONTROL_MESSAGE)
    if can_msg.arbitration_id != control_def.frame_id:
        return None
    decoded = db.decode_message(can_msg.arbitration_id, can_msg.data, decode_choices=False)

    throttle_pct = float(decoded.get("Cockpit_ACC", 0) or 0)
    brake_pct = float(decoded.get("Cockpit_Beak", 0) or 0)
    steer_deg = float(decoded.get("Cockpit_EPS_Angle", 0) or 0)
    gear = int(decoded.get("Cockpit_Gear", 0) or 0)
    key_break = int(decoded.get("Cockpit_Key_Break", 0) or 0)
    key_stop = int(decoded.get("Cockpit_Key_Stop", 0) or 0)
    key_d = int(decoded.get("Cockpit_Key_D", 0) or 0)
    key_r = int(decoded.get("Cockpit_Key_R", 0) or 0)
    key_park = int(decoded.get("Cockpit_Key_Park", 0) or 0)

    park_requested = bool(key_park and not key_r)
    reverse = bool(key_r) or (gear in REVERSE_GEARS and not key_d and not park_requested)
    drive = bool(key_d) or gear in DRIVE_GEARS
    forced_stop = bool(key_break or key_stop or park_requested)

    throttle = max(0.0, min(1.0, throttle_pct / 100.0))
    brake = max(0.0, min(1.0, brake_pct / 100.0))
    steer = max(-1.0, min(1.0, steer_deg / STEER_MAX_DEG))
    if forced_stop:
        throttle = 0.0
        brake = 1.0
    elif not reverse and not drive:
        throttle = 0.0

    return {
        "throttle": throttle,
        "steer": steer,
        "brake": brake,
        "reverse": reverse,
        "hand_brake": forced_stop,
        "_debug": {
            "steer_deg": steer_deg,
            "throttle_pct": throttle_pct,
            "brake_pct": brake_pct,
            "gear": gear,
            "drive": drive,
            "reverse": reverse,
            "e_stop": key_stop,
            "key_break": key_break,
            "key_d": key_d,
            "key_r": key_r,
            "key_park": key_park,
            "xbw": int(decoded.get("Cockpit_Key_XbW", 0) or 0),
        },
    }


def run_bridge():
    db, dbc_path = load_database()
    sock_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_rx = bind_udp_rx()
    bus = None
    next_can_attempt = 0.0
    last_state_tx = 0.0
    last_print = 0.0
    last_no_can_print = 0.0

    print(
        f"[bridge] manual CAN bridge online, dbc={dbc_path}, "
        f"udp_rx={UDP_IP}:{UDP_PORT_RX}, udp_tx={UDP_IP}:{UDP_PORT_TX}, "
        f"can={CAN_BUSTYPE}:{CAN_CHANNEL}@{CAN_BITRATE}",
        flush=True,
    )

    try:
        while True:
            now = time.time()
            if bus is None and now >= next_can_attempt:
                try:
                    bus = open_can_bus()
                    write_status(can_connected=True, last_error=None)
                    print("[bridge] CAN connected", flush=True)
                except Exception as exc:
                    write_status(can_connected=False, last_error=str(exc))
                    if now - last_no_can_print >= 5.0:
                        print(f"[bridge] CAN unavailable, retrying: {exc}", flush=True)
                        last_no_can_print = now
                    next_can_attempt = now + CAN_RECONNECT_SEC

            last_state_tx = drain_udp(sock_rx, db, bus, last_state_tx)

            if bus is not None:
                try:
                    can_msg = bus.recv(timeout=0.005)
                except Exception as exc:
                    write_status(can_connected=False, last_error=str(exc))
                    print(f"[bridge] CAN receive failed, reconnecting: {exc}", flush=True)
                    shutdown_bus(bus)
                    bus = None
                    next_can_attempt = time.time() + CAN_RECONNECT_SEC
                    continue

                if can_msg is not None:
                    write_status(last_can_rx_at=time.time())
                    command = decode_control_message(db, can_msg)
                    if command is not None:
                        debug = command.pop("_debug")
                        sock_tx.sendto(json.dumps(command).encode("utf-8"), (UDP_IP, UDP_PORT_TX))
                        now = time.time()
                        write_status(last_udp_ctrl_at=now)
                        if now - last_print >= PRINT_INTERVAL_SEC:
                            gear_text = "R" if debug["reverse"] else ("D" if debug["drive"] else "N/P")
                            mode_text = "XbW" if debug["xbw"] else "Manual"
                            stop_text = "E-STOP" if debug["e_stop"] else "OK"
                            print(
                                f"[bridge] steer={debug['steer_deg']:>6.1f}deg "
                                f"thr={debug['throttle_pct']:>5.1f}% "
                                f"brk={debug['brake_pct']:>5.1f}% "
                                f"gear={gear_text} mode={mode_text} {stop_text}",
                                flush=True,
                            )
                            last_print = now
            else:
                time.sleep(0.02)
    finally:
        shutdown_bus(bus)
        sock_tx.close()
        sock_rx.close()


if __name__ == "__main__":
    try:
        run_bridge()
    except KeyboardInterrupt:
        print("\n[bridge] stopped", flush=True)
