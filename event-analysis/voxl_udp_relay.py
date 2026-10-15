#!/usr/bin/env python3
"""Temporary bench relay from VOXL's onboard bridge to laptop MAVROS."""

import selectors
import socket


def main():
    laptop = "192.168.100.1"
    peer = None
    counts = {"onboard": 0, "laptop": 0}
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as local, \
            socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as network, \
            selectors.DefaultSelector() as selector:
        local.bind(("127.0.0.1", 14551))
        network.bind(("0.0.0.0", 14601))
        selector.register(local, selectors.EVENT_READ, "onboard")
        selector.register(network, selectors.EVENT_READ, "laptop")
        print("Relay: onboard 127.0.0.1:14551 <-> laptop 192.168.100.1:14545; "
              "network listen port 14601", flush=True)
        while True:
            for key, _ in selector.select():
                data, source = key.fileobj.recvfrom(65535)
                if key.data == "onboard":
                    if source[0] != "127.0.0.1":
                        continue
                    if source != peer:
                        peer = source
                        print("Onboard peer:", peer, flush=True)
                    network.sendto(data, (laptop, 14545))
                else:
                    if source[0] != laptop or peer is None:
                        continue
                    local.sendto(data, peer)
                counts[key.data] += 1
                if counts[key.data] == 1:
                    print("First packet forwarded from", key.data,
                          "bytes:", len(data), flush=True)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Relay stopped.", flush=True)
