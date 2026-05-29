import argparse
import math
import random
import sys
import threading
import time

import carla


VIEW_ALIASES = {
    "1": "follow",
    "follow": "follow",
    "third": "follow",
    "third_person": "follow",
    "第三视角": "follow",
    "2": "driver",
    "driver": "driver",
    "first": "driver",
    "first_person": "driver",
    "第一视角": "driver",
    "3": "rear",
    "rear": "rear",
    "back": "rear",
    "mirror": "rear",
    "后视": "rear",
    "后视镜": "rear",
    "倒车": "rear",
}


def normalize_view(value):
    raw = str(value or "follow").strip()
    if raw in VIEW_ALIASES:
        return VIEW_ALIASES[raw]
    lowered = raw.lower().replace("-", "_").strip()
    if lowered in VIEW_ALIASES:
        return VIEW_ALIASES[lowered]
    if "rear" in lowered or "back" in lowered or "mirror" in lowered or "后视" in raw or "倒车" in raw:
        return "rear"
    if "driver" in lowered or "first" in lowered or "驾驶" in raw or "第一" in raw:
        return "driver"
    return "follow"


def lerp(a, b, alpha):
    return float(a) + (float(b) - float(a)) * alpha


def lerp_angle(a, b, alpha):
    delta = (float(b) - float(a) + 180.0) % 360.0 - 180.0
    return float(a) + delta * alpha


def smooth_transform(previous, target, alpha):
    if previous is None:
        return target
    return carla.Transform(
        carla.Location(
            x=lerp(previous.location.x, target.location.x, alpha),
            y=lerp(previous.location.y, target.location.y, alpha),
            z=lerp(previous.location.z, target.location.z, alpha),
        ),
        carla.Rotation(
            pitch=lerp_angle(previous.rotation.pitch, target.rotation.pitch, alpha),
            yaw=lerp_angle(previous.rotation.yaw, target.rotation.yaw, alpha),
            roll=0.0,
        ),
    )


def offset_from_vehicle(vehicle_transform, forward=0.0, right=0.0, up=0.0):
    yaw = math.radians(vehicle_transform.rotation.yaw)
    forward_x = math.cos(yaw)
    forward_y = math.sin(yaw)
    right_x = -math.sin(yaw)
    right_y = math.cos(yaw)
    base = vehicle_transform.location
    return carla.Location(
        x=base.x + forward_x * forward + right_x * right,
        y=base.y + forward_y * forward + right_y * right,
        z=base.z + up,
    )


def target_transform(vehicle_transform, view):
    view = normalize_view(view)
    yaw = vehicle_transform.rotation.yaw
    if view == "driver":
        return carla.Transform(
            offset_from_vehicle(vehicle_transform, forward=0.85, right=-0.25, up=1.35),
            carla.Rotation(pitch=-3.0, yaw=yaw, roll=0.0),
        )
    if view == "rear":
        return carla.Transform(
            offset_from_vehicle(vehicle_transform, forward=-1.0, right=0.0, up=1.45),
            carla.Rotation(pitch=-5.0, yaw=yaw + 180.0, roll=0.0),
        )
    return carla.Transform(
        offset_from_vehicle(vehicle_transform, forward=-7.5, right=0.0, up=3.0),
        carla.Rotation(pitch=-13.0, yaw=yaw, roll=0.0),
    )


def smoothing_alpha(view):
    view = normalize_view(view)
    if view == "follow":
        return 0.22
    return 0.55


def find_existing_vehicle(world):
    vehicles = list(world.get_actors().filter("vehicle.*"))
    for actor in vehicles:
        if actor.attributes.get("role_name") == "hero":
            return actor
    return vehicles[0] if vehicles else None


def choose_vehicle_blueprint(world):
    library = world.get_blueprint_library()
    candidates = list(library.filter("vehicle.lincoln.mkz*"))
    if not candidates:
        candidates = list(library.filter("vehicle.tesla.model3"))
    if not candidates:
        candidates = [bp for bp in library.filter("vehicle.*") if int(bp.get_attribute("number_of_wheels")) == 4]
    if not candidates:
        raise RuntimeError("no four-wheel vehicle blueprint found")
    blueprint = random.choice(candidates)
    if blueprint.has_attribute("role_name"):
        blueprint.set_attribute("role_name", "hero")
    return blueprint


def spawn_vehicle(world):
    spawn_points = world.get_map().get_spawn_points()
    random.shuffle(spawn_points)
    blueprint = choose_vehicle_blueprint(world)
    for spawn_point in spawn_points:
        vehicle = world.try_spawn_actor(blueprint, spawn_point)
        if vehicle is not None:
            return vehicle
    raise RuntimeError("failed to spawn test vehicle")


def input_loop(state):
    print("视角键: 1/follow=第三视角, 2/driver=第一视角, 3/rear=后视镜, q=退出")
    while not state["stop"]:
        try:
            text = input("> ").strip()
        except EOFError:
            state["stop"] = True
            return
        if text.lower() in {"q", "quit", "exit"}:
            state["stop"] = True
            return
        state["view"] = normalize_view(text)
        print(f"switched view -> {state['view']}")


def main():
    parser = argparse.ArgumentParser(description="Local CARLA spectator view smoke test.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--town", default="", help="Optional world name, for example Town02.")
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--view", default="follow", choices=["follow", "driver", "rear"])
    parser.add_argument("--no-spawn", action="store_true", help="Use an existing vehicle instead of spawning one.")
    parser.add_argument("--autopilot", action="store_true", help="Enable autopilot on the test vehicle.")
    args = parser.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    world = client.get_world()
    if args.town:
        print(f"loading world {args.town} ...")
        world = client.load_world(args.town)
        time.sleep(1.0)

    spawned_here = False
    vehicle = find_existing_vehicle(world) if args.no_spawn else None
    if vehicle is None:
        vehicle = spawn_vehicle(world)
        spawned_here = True
    print(f"vehicle id={vehicle.id}, type={vehicle.type_id}, spawned_here={spawned_here}")

    if args.autopilot:
        try:
            vehicle.set_autopilot(True)
            print("autopilot enabled")
        except Exception as exc:
            print(f"autopilot enable failed: {exc}")

    state = {"view": normalize_view(args.view), "stop": False}
    threading.Thread(target=input_loop, args=(state,), daemon=True).start()

    previous = None
    previous_view = None
    try:
        while not state["stop"]:
            view = normalize_view(state["view"])
            if previous_view != view:
                previous = None
                previous_view = view
            target = target_transform(vehicle.get_transform(), view)
            previous = smooth_transform(previous, target, smoothing_alpha(view))
            world.get_spectator().set_transform(previous)
            time.sleep(0.03)
    except KeyboardInterrupt:
        pass
    finally:
        if spawned_here:
            try:
                vehicle.destroy()
                print("test vehicle destroyed")
            except Exception as exc:
                print(f"vehicle destroy skipped: {exc}")


if __name__ == "__main__":
    sys.exit(main())
