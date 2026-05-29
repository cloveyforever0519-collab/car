#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import socket


def main() -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", 5010))
    print("listening sensor UDP 5010...")
    while True:
        data, addr = sock.recvfrom(65535)
        print("from", addr)
        try:
            print(json.dumps(json.loads(data.decode("utf-8")), ensure_ascii=False, indent=2)[:5000])
        except Exception:
            print(data[:5000])


if __name__ == "__main__":
    main()
