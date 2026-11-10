#!/usr/bin/env python3
"""MASN: LoRa receive monitor (RNode mode). Configures the radio and prints
every packet with RSSI/SNR and its decoded Reticulum header.
Usage: python masn/masn_rxmon.py [/dev/ttyACM0] [seconds]"""
import struct, sys, time
import serial
port = next((a for a in sys.argv[1:] if a.startswith("/dev/")), "/dev/ttyACM0")
dur = next((int(a) for a in sys.argv[1:] if a.isdigit()), 300)
FEND, FESC, TFEND, TFESC = 0xC0, 0xDB, 0xDC, 0xDD
PTYPE = {0: "DATA", 1: "ANNOUNCE", 2: "LINKREQ", 3: "PROOF"}
DTYPE = {0: "SINGLE", 1: "GROUP", 2: "PLAIN", 3: "LINK"}
esc = lambda b: b.replace(b"\xdb", b"\xdb\xdd").replace(b"\xc0", b"\xdb\xdc")

def rns_header(p):
    if len(p) < 19: return "corto"
    f = p[0]
    htype, ctxflag, transport = (f >> 6) & 1, (f >> 5) & 1, (f >> 4) & 1
    dtype, ptype = (f >> 2) & 3, f & 3
    hops = p[1]
    if htype == 1 and len(p) >= 35:
        tid, dst, ctx = p[2:18].hex(), p[18:34].hex(), p[34]
        via = f" via={tid}"
    else:
        dst, ctx, via = p[2:18].hex(), p[18], ""
    return (f"{PTYPE[ptype]:8s} dst={dst} {DTYPE[dtype]} hops={hops} "
            f"ctx={ctx:02x}{' TRANSPORT' if transport else ''}{via}")

s = serial.Serial(port, 115200, timeout=0.2); time.sleep(1); s.reset_input_buffer()
def cmd(c, p): s.write(bytes([FEND, c]) + esc(p) + bytes([FEND])); s.flush(); time.sleep(0.3)
cmd(0x01, struct.pack(">I", 869618000)); cmd(0x02, struct.pack(">I", 62500))
cmd(0x03, bytes([7])); cmd(0x04, bytes([5])); cmd(0x05, bytes([5])); cmd(0x06, bytes([1]))
print(f"Listening on 869.618 MHz BW62.5k SF5 CR5 for {dur}s...", flush=True)
buf, inframe, escp, cmdb = bytearray(), False, False, None
rssi = snr = None; n = 0; t0 = time.monotonic()
while time.monotonic() - t0 < dur:
    for b in s.read(512):
        if b == FEND:
            if inframe and cmdb is not None:
                if cmdb == 0x23 and buf: rssi = buf[0] - 157
                elif cmdb == 0x24 and buf: snr = struct.unpack("b", bytes([buf[0]]))[0] / 4
                elif cmdb == 0x00 and buf:
                    n += 1
                    print(f"[{time.strftime('%H:%M:%S')}] #{n} {len(buf)}B RSSI={rssi} SNR={snr} "
                          f"{rns_header(bytes(buf))}", flush=True)
            inframe, buf, cmdb, escp = True, bytearray(), None, False
        elif inframe:
            if cmdb is None: cmdb = b; continue
            if b == FESC: escp = True; continue
            if escp: b = FEND if b == TFEND else FESC if b == TFESC else b; escp = False
            buf.append(b)
print(f"Done: {n} packets in {dur}s")
