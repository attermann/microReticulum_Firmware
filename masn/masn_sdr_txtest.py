#!/usr/bin/env python3
"""MASN V1.2: verify real RF emission with an RTL-SDR (rtl_sdr + numpy).

Board must be in RNode mode (masn_setup.py rnode) with TX power <= 7 dBm on
869.618 MHz (the diagnostic TX commands refuse anything else).

  python masn/masn_sdr_txtest.py [--port /dev/ttyACM0] [--gain 20]

Records 8 s of IQ 200 kHz below the channel (avoids the SDR DC spike), fires
a 600 ms CW carrier (diag 0x63) at t=2 s and one LoRa packet (diag 0x65) at
t=4.5 s, then prints the in-channel power excess over time.
Expected: ~600 ms of narrow carrier at +0.0 kHz offset, then a short burst.
"""
import argparse
import os
import subprocess
import sys
import tempfile
import time

import numpy as np
import serial

FC_CHAN = 869618000
FS = 1024000
OFFSET = 200e3
DUR = 8


def capture(port, gain, path):
    s = serial.Serial(port, 115200, timeout=0.3)
    time.sleep(1)
    s.reset_input_buffer()
    log = open(path + ".log", "w")
    p = subprocess.Popen(["rtl_sdr", "-f", str(int(FC_CHAN - OFFSET)), "-s", str(FS),
                          "-g", str(gain), "-n", str(FS * DUR), path], stderr=log)
    time.sleep(1.5)                       # rtl_sdr start-up
    t0 = time.monotonic()
    for op, at in ((0x63, 2.0), (0x65, 4.5)):
        while time.monotonic() - t0 < at:
            time.sleep(0.001)
        s.write(bytes([0xC0, 0x08, op, 0xC0]))
        s.flush()
    p.wait()
    data, quiet_until = b"", time.monotonic() + 1.5
    while time.monotonic() < quiet_until:
        data += s.read(4096)
    for frame in data.split(b"\xc0"):
        if frame[:1] in (b"\x63", b"\x65"):
            print(f"diag 0x{frame[0]:02x}: {frame[1:].decode(errors='replace')}")


def analyse(path):
    raw = np.fromfile(path, dtype=np.uint8).astype(np.float32) - 127.5
    iq = raw[0::2] + 1j * raw[1::2]
    n = 1024
    blocks = len(iq) // n
    spec = np.abs(np.fft.fft(iq[:blocks * n].reshape(blocks, n) * np.hanning(n), axis=1)) ** 2
    spec = np.fft.fftshift(spec, axes=1)
    f = (np.arange(n) - n // 2) * FS / n
    chan = (f > OFFSET - 35e3) & (f < OFFSET + 35e3)
    guard = (np.abs(f) > 300e3) & (np.abs(f) < 450e3)
    delta = 10 * np.log10(spec[:, chan].mean(1)) - 10 * np.log10(spec[:, guard].mean(1))
    delta -= np.median(delta)
    t = np.arange(blocks) * n / FS
    print(" t(s)  excess_dB  peak_offset_kHz")
    for i in range(0, blocks, 50):
        j = i + int(np.argmax(delta[i:i + 50]))
        if delta[j] > 6:
            peak = f[chan][np.argmax(spec[j, chan])] / 1e3 - OFFSET / 1e3
            print(f"{t[i]:5.2f}  {delta[j]:8.1f}  {peak:+8.2f}")
    active = t[delta > 6]
    print(f"active blocks: {len(active)}"
          + (f" ({active[0]:.2f}s .. {active[-1]:.2f}s)" if len(active) else " - NO EMISSION SEEN"))


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default="/dev/ttyACM0")
    p.add_argument("--gain", type=int, default=20)
    a = p.parse_args()
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "cap.u8")
        capture(a.port, a.gain, path)
        analyse(path)


if __name__ == "__main__":
    sys.exit(main())
