#!/usr/bin/env python3
"""MASN V1.2: traced SX1262 oscillator bring-up (no RF emission).

Usage: python masn/masn_osc_test.py [/dev/ttyACM0] [1v8|3v3|none|all]

Sends KISS CMD_DETECT (0x08) subcommands 0x68/0x69/0x6A handled in
MASN_Diagnostics.h and decodes the reply. Each snapshot is
STATUS/MODE/ERRORS; mode 2=STDBY_RC, 3=STDBY_XOSC, 4=FS, 6=TX.
"""
import sys
import time

import serial

OPS = {"1v8": 0x68, "3v3": 0x69, "none": 0x6A}
MODES = {0: "?", 1: "RFU", 2: "STDBY_RC", 3: "STDBY_XOSC", 4: "FS", 5: "RX", 6: "TX"}
ERRBITS = ["RC64K_CALIB", "RC13M_CALIB", "PLL_CALIB", "ADC_CALIB",
           "IMG_CALIB", "XOSC_START", "PLL_LOCK", "?", "PA_RAMP"]


def decode_err(hexstr):
    try:
        val = int(hexstr.rstrip("!"), 16)
    except ValueError:
        return hexstr
    names = [n for i, n in enumerate(ERRBITS) if val & (1 << i)]
    return "OK" if not names else "+".join(names)


def run(ser, name):
    op = OPS[name]
    ser.reset_input_buffer()
    ser.write(bytes([0xC0, 0x08, op, 0xC0]))
    ser.flush()
    data = bytearray()
    end = time.monotonic() + 5
    reply = None
    while time.monotonic() < end and reply is None:
        data.extend(ser.read(256))
        for frame in data.split(b"\xC0"):
            if len(frame) > 1 and frame[0] == op and frame[1:4] == b"V=1":
                reply = frame[1:].decode("ascii", "replace")
    if reply is None:
        print(f"[{name}] NO REPLY. Raw: {data.hex(' ')}")
        return
    print(f"\n=== TCXO {name.upper()} ===")
    print(reply)
    for field in reply.split(";"):
        key, _, val = field.partition("=")
        parts = val.split("/")
        if len(parts) == 3 and key in ("POR", "CLR", "CAL", "XOSC", "FS", "END"):
            st, mode, err = parts
            m = int(mode) if mode.isdigit() else 255
            print(f"  {key:5s} status={st} mode={MODES.get(m, m)} errors={decode_err(err)}")
        elif key == "WR":
            print(f"  WR    SPI write: {'OK' if val == '3444' else 'FAILED'} (read {val})")
        elif key == "RST":
            print(f"  RST   reset via P0.09: {'OK' if val.startswith('1424') else 'NO RESET'} ({val})")


def main():
    port = "/dev/ttyACM0"
    which = "all"
    for arg in sys.argv[1:]:
        if arg.startswith("/dev/"):
            port = arg
        else:
            which = arg.lower()
    names = list(OPS) if which == "all" else [which]
    with serial.Serial(port, 115200, timeout=0.2) as ser:
        time.sleep(1)
        for name in names:
            run(ser, name)
            time.sleep(0.5)


if __name__ == "__main__":
    main()
