#!/usr/bin/env python3
"""rnode_ifac.py — set the interface access code (IFAC) of a node's LoRa interface over USB,
through the Provisioning subsystem.

    rnode_ifac.py <port> get                 # current IFAC network name (the passphrase is never read back)
    rnode_ifac.py <port> set <file.json>     # {"network_name": "...", "passphrase": "..."}
    rnode_ifac.py <port> off                 # disable IFAC

Equivalent to `network_name` / `passphrase` on a Python RNS interface: every node sharing the
LoRa channel must use the same values. The passphrase is read from a file, never from the command
line (shell history, process list). `set` and `off` commit the fields, then reboot the node: they
take effect at boot.

Wire protocol: KISS frames CMD_PROVISION_REQ (0x86) / CMD_PROVISION_RSP (0x87) carrying MsgPack
[op, seq, payload] envelopes (see microReticulum docs/provisioning_client_guide.md). Fields:
namespace "General" (100), 8 = IFAC network name, 9 = IFAC passphrase (secret).
Requires pyserial and RNS (for its bundled umsgpack) or the msgpack package.
"""
import json
import sys
import time

import serial

try:
    import RNS.vendor.umsgpack as msgpack
except ImportError:
    import msgpack

FEND, FESC, TFEND, TFESC = 0xC0, 0xDB, 0xDC, 0xDD
CMD_PROVISION_REQ, CMD_PROVISION_RSP = 0x86, 0x87
OP_GET_STATE, OP_SET_STATE, OP_COMMIT, OP_REBOOT, OP_ERROR = 4, 5, 6, 9, 101
NS_GENERAL, F_NETNAME, F_PASSPHRASE = 100, 8, 9
MAX_LEN = 63  # 64-byte buffers on the firmware side


def escape(data):
    out = bytearray()
    for b in data:
        if b == FEND:
            out += bytes([FESC, TFEND])
        elif b == FESC:
            out += bytes([FESC, TFESC])
        else:
            out.append(b)
    return bytes(out)


class Node:
    def __init__(self, port):
        self.serial = serial.Serial(port, 115200, timeout=0.2)
        time.sleep(2)
        self.serial.reset_input_buffer()
        self.seq = 0
        self.buffer = bytearray()

    def frames(self):
        # Log text and KISS frames share the port: keep complete frames only.
        self.buffer += self.serial.read(4096)
        while True:
            start = self.buffer.find(bytes([FEND]))
            if start < 0:
                self.buffer.clear()
                return
            end = self.buffer.find(bytes([FEND]), start + 1)
            if end < 0:
                del self.buffer[:start]
                return
            raw, self.buffer = bytes(self.buffer[start + 1:end]), self.buffer[end:]
            frame, esc = bytearray(), False
            for b in raw:
                if esc:
                    frame.append(FEND if b == TFEND else FESC if b == TFESC else b)
                    esc = False
                elif b == FESC:
                    esc = True
                else:
                    frame.append(b)
            if frame:
                yield bytes(frame)

    def request(self, op, payload=None, timeout=8.0, expect_reply=True):
        self.seq += 1
        body = msgpack.packb([op, self.seq, payload])
        self.serial.write(bytes([FEND, CMD_PROVISION_REQ]) + escape(body) + bytes([FEND]))
        if not expect_reply:
            return None
        deadline = time.time() + timeout
        while time.time() < deadline:
            for frame in self.frames():
                if frame[0] != CMD_PROVISION_RSP:
                    continue
                reply = msgpack.unpackb(frame[1:])
                if reply[1] != self.seq:
                    continue
                if reply[0] == OP_ERROR:
                    raise SystemExit(f"error from node: {reply[2] if len(reply) > 2 else reply}")
                return reply[2] if len(reply) > 2 else None
        raise SystemExit("error: no reply from the node (still booting? Provisioning starts once RNS is up)")


def read_secret_file(path):
    with open(path) as f:
        data = json.load(f)
    netname, passphrase = data.get("network_name") or "", data.get("passphrase") or ""
    if not netname and not passphrase:
        raise SystemExit("error: the file has neither network_name nor passphrase")
    if len(netname.encode()) > MAX_LEN or len(passphrase.encode()) > MAX_LEN:
        raise SystemExit(f"error: network_name and passphrase are limited to {MAX_LEN} bytes")
    return netname, passphrase


def apply(node, netname, passphrase):
    reply = node.request(OP_SET_STATE, {3: {NS_GENERAL: {F_NETNAME: netname, F_PASSPHRASE: passphrase}}})
    if reply.get(3):
        raise SystemExit(f"error: fields rejected {reply[3]} — firmware without IFAC fields?")
    reply = node.request(OP_COMMIT, {1: [NS_GENERAL]})
    print(f"committed ({reply.get(1)} field(s)), rebooting the node...")
    node.request(OP_REBOOT, None, expect_reply=False)


def main(argv):
    if len(argv) < 2:
        raise SystemExit(__doc__)
    node = Node(argv[0])
    command = argv[1]
    if command == "get":
        values = node.request(OP_GET_STATE, {1: [NS_GENERAL]}).get(1, {}).get(NS_GENERAL, {})
        if F_NETNAME not in values:
            raise SystemExit("error: no IFAC field — firmware without IFAC support?")
        name = values[F_NETNAME]
        print(f"IFAC network name: {name!r}" if name else "IFAC: no network name set")
    elif command == "set" and len(argv) == 3:
        apply(node, *read_secret_file(argv[2]))
    elif command == "off":
        apply(node, "", "")
    else:
        raise SystemExit(__doc__)


if __name__ == "__main__":
    main(sys.argv[1:])
