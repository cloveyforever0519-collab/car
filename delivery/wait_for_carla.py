#!/usr/bin/env python3
import argparse
import sys
import time


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--client-timeout", type=float, default=5.0)
    args = parser.parse_args()

    try:
        import carla
    except Exception as exc:
        print(f"carla Python module unavailable: {exc}", file=sys.stderr)
        return 2

    deadline = time.time() + args.timeout
    last_error = None
    while time.time() < deadline:
        try:
            client = carla.Client(args.host, args.port)
            client.set_timeout(args.client_timeout)
            world = client.get_world()
            map_name = world.get_map().name.split("/")[-1]
            print(f"CARLA ready: {args.host}:{args.port}, world={map_name}")
            return 0
        except Exception as exc:
            last_error = exc
            time.sleep(2.0)

    print(
        f"CARLA not ready after {args.timeout:.0f}s at {args.host}:{args.port}: {last_error}",
        file=sys.stderr,
    )
    return 1


if __name__ == "__main__":
    raise SystemExit(main())

