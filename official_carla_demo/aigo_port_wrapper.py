#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Run legacy AIGO scripts with an isolated telemetry port.

The old scenario scripts bind 127.0.0.1:5000 internally. The front-end contract
also uses UDP 5000 for telemetry, so a local data listener can steal packets from
the algorithm. This wrapper redirects only that local bind to a private port.
"""

from __future__ import annotations

import os
import runpy
import socket as _socket
import sys
from pathlib import Path
from typing import Any


PRIVATE_TELEMETRY_PORT = int(os.environ.get("AIGO_TELEMETRY_PORT", "5500"))
LEGACY_TELEMETRY_ADDR = ("127.0.0.1", 5000)


class SocketProxy:
    def __init__(self, *args: Any, **kwargs: Any) -> None:
        self._sock = _ORIGINAL_SOCKET(*args, **kwargs)

    def bind(self, address: Any) -> Any:
        host, port = address
        if host == LEGACY_TELEMETRY_ADDR[0] and int(port) == LEGACY_TELEMETRY_ADDR[1]:
            address = (host, PRIVATE_TELEMETRY_PORT)
        return self._sock.bind(address)

    def __enter__(self) -> "SocketProxy":
        return self

    def __exit__(self, *args: Any) -> None:
        self._sock.close()

    def __getattr__(self, name: str) -> Any:
        return getattr(self._sock, name)


_ORIGINAL_SOCKET = _socket.socket


def main() -> None:
    if len(sys.argv) < 2:
        raise SystemExit("usage: aigo_port_wrapper.py <legacy_script.py>")
    script = Path(sys.argv[1]).resolve()
    if not script.exists():
        raise SystemExit(f"AIGO script not found: {script}")
    print(f"[aigo-wrapper] {script.name}: 127.0.0.1:5000 -> 127.0.0.1:{PRIVATE_TELEMETRY_PORT}", flush=True)
    _socket.socket = SocketProxy  # type: ignore[assignment]
    sys.argv = [str(script)]
    runpy.run_path(str(script), run_name="__main__")


if __name__ == "__main__":
    main()
