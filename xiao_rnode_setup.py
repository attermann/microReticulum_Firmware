#!/usr/bin/env python3
"""
XIAO nRF52840 + Wio-SX1262 — RNode Setup Wizard

Just run it:  python xiao_rnode_setup.py

It will auto-detect your device, ask you a few questions,
and handle the entire setup: build, flash, provision,
radio config, rnodeconf patch, and verification.
"""

import serial
import time
import sys
import hashlib
import os
import glob
import subprocess
import random
import datetime
import struct

# ── Constants ────────────────────────────────────────

FEND, FESC, TFEND, TFESC = 0xC0, 0xDB, 0xDC, 0xDD

CMD_RESET, CMD_RESET_BYTE   = 0x55, 0xF8
CMD_ROM_READ, CMD_ROM_WRITE  = 0x51, 0x52
CMD_FW_HASH, CMD_CONF_SAVE   = 0x58, 0x53
CMD_UNLOCK_ROM               = 0x59
ROM_UNLOCK_BYTE              = 0xF8
CMD_FREQUENCY, CMD_BANDWIDTH = 0x01, 0x02
CMD_TXPOWER, CMD_SF, CMD_CR  = 0x03, 0x04, 0x05

ADDR_PRODUCT, ADDR_MODEL, ADDR_HW_REV = 0x00, 0x01, 0x02
ADDR_SERIAL, ADDR_MADE, ADDR_CHKSUM   = 0x03, 0x07, 0x0B
ADDR_INFO_LOCK = 0x9B
CHECKSUMMED_SIZE = 0x0B

PRODUCT_XIAO, MODEL_433, MODEL_868 = 0x11, 0x11, 0x12
INFO_LOCK_BYTE = 0x73

FW_BIN_PATHS = [
    ".pio/build/xiao_nrf52840/rnode_firmware_xiao_nrf52840.bin",
    ".pio/build/xiao_nrf52840_lowpower/rnode_firmware_xiao_nrf52840.bin",
]

# ── KISS helpers ─────────────────────────────────────

def kiss_escape(data):
    r = bytearray()
    for b in data:
        if b == FEND: r.extend([FESC, TFEND])
        elif b == FESC: r.extend([FESC, TFESC])
        else: r.append(b)
    return bytes(r)

def kiss_unescape(data):
    r, esc = bytearray(), False
    for b in data:
        if esc:
            r.append(FEND if b == TFEND else (FESC if b == TFESC else b))
            esc = False
        elif b == FESC: esc = True
        else: r.append(b)
    return bytes(r)

def kiss_frame(cmd, data=b''):
    f = bytearray([FEND, cmd])
    f.extend(kiss_escape(data))
    f.append(FEND)
    return bytes(f)

def read_kiss_frame(ser, timeout=3.0):
    start, buf, in_frame = time.time(), bytearray(), False
    while (time.time() - start) < timeout:
        if ser.in_waiting > 0:
            b = ser.read(1)[0]
            if b == FEND:
                if in_frame and buf:
                    return buf[0], kiss_unescape(bytes(buf[1:]))
                in_frame, buf = True, bytearray()
            elif in_frame:
                buf.append(b)
        else:
            time.sleep(0.01)
    return None, None

def read_eeprom(ser):
    ser.reset_input_buffer()
    ser.write(kiss_frame(CMD_ROM_READ))
    cmd, data = read_kiss_frame(ser, 3.0)
    return data if cmd == CMD_ROM_READ else None

# ── UI helpers ───────────────────────────────────────

def ask(prompt, default=None):
    if default is not None:
        raw = input(f"  {prompt} [{default}]: ").strip()
        return raw if raw else str(default)
    return input(f"  {prompt}: ").strip()

def ask_yes(prompt, default=True):
    suffix = "Y/n" if default else "y/N"
    raw = input(f"  {prompt} [{suffix}]: ").strip().lower()
    if raw == "": return default
    return raw in ("y", "yes")

def ask_pick(prompt, options, default=None):
    """options = [(value, label), ...]"""
    print(f"\n  {prompt}")
    for i, (val, label) in enumerate(options):
        marker = " ← default" if val == default else ""
        print(f"    {i+1}) {label}{marker}")
    while True:
        raw = input(f"  Choice: ").strip()
        if raw == "" and default is not None: return default
        try:
            idx = int(raw)
            if 1 <= idx <= len(options): return options[idx-1][0]
        except ValueError:
            pass
        print("  Invalid, try again.")

def wait_for_port(port, timeout=20):
    print(f"  Waiting for device on {port}...", end='', flush=True)
    start = time.time()
    time.sleep(1.5)
    while (time.time() - start) < timeout:
        if os.path.exists(port) or glob.glob(port):
            try:
                t = serial.Serial(port, 115200, timeout=1); t.close()
                print(" OK"); return True
            except (serial.SerialException, OSError): pass
        time.sleep(0.5)
        print(".", end='', flush=True)
    print(" timeout!"); return False

def find_fw_bin():
    for p in FW_BIN_PATHS:
        if os.path.exists(p): return p
    return None

def find_ports():
    ports = []
    for pat in ["/dev/cu.usbmodem*", "/dev/ttyACM*", "/dev/ttyUSB*"]:
        ports.extend(glob.glob(pat))
    return sorted(set(ports))

def header(n, text):
    print(f"\n{'━'*55}")
    print(f"  Step {n}: {text}")
    print(f"{'━'*55}")

def open_serial(port):
    ser = serial.Serial(port, 115200, timeout=2)
    time.sleep(2.0)
    ser.reset_input_buffer()
    return ser

# ── Main wizard ──────────────────────────────────────

def main():
    print()
    print("╔═══════════════════════════════════════════════════════╗")
    print("║   XIAO nRF52840 + Wio-SX1262 — RNode Setup Wizard    ║")
    print("╚═══════════════════════════════════════════════════════╝")
    print()

    # ─── Port ────────────────────────────────────
    header(1, "Find your device")

    ports = find_ports()
    if not ports:
        print("  No serial devices detected.")
        port = ask("Enter serial port manually")
    elif len(ports) == 1:
        port = ports[0]
        print(f"  Found: {port}")
    else:
        port = ask_pick("Multiple ports found:",
                        [(p, p) for p in ports], ports[0])
    print(f"  → Using {port}")

    # ─── Band ────────────────────────────────────
    header(2, "Frequency band")

    band_key = ask_pick("Which band does your Wio-SX1262 use?", [
        ("868", "868 MHz band  (779–928 MHz, EU/Asia)"),
        ("433", "433 MHz band  (430–510 MHz)"),
    ], "868")

    if band_key == "868":
        model = MODEL_868
        freq_min, freq_max, freq_default = 779000000, 928000000, 869525000
        band_label = "779–928 MHz"
    else:
        model = MODEL_433
        freq_min, freq_max, freq_default = 430000000, 510000000, 433775000
        band_label = "430–510 MHz"
    print(f"  → {band_label}")

    # ─── Build & flash ───────────────────────────
    header(3, "Build & flash firmware")

    have_pio = os.path.exists("platformio.ini")
    if have_pio:
        print("  Found platformio.ini in current directory.")
        if ask_yes("Build and flash firmware now?"):
            env = "xiao_nrf52840"
            print("\n  Building...", flush=True)
            subprocess.run(["pio", "run", "-e", env, "-t", "clean"],
                           capture_output=True)
            r = subprocess.run(["pio", "run", "-e", env],
                               capture_output=True, text=True)
            if r.returncode != 0:
                print("  ✗ Build failed!")
                for l in r.stderr.split('\n')[-8:]:
                    if l.strip(): print(f"    {l.strip()}")
                sys.exit(1)
            print("  ✓ Build OK")

            print("  Flashing...", flush=True)
            r = subprocess.run(["pio", "run", "-e", env, "-t", "upload"],
                               capture_output=True, text=True)
            if r.returncode != 0:
                print("  ✗ Flash failed!")
                for l in r.stderr.split('\n')[-8:]:
                    if l.strip(): print(f"    {l.strip()}")
                sys.exit(1)
            print("  ✓ Flash OK\n")

            time.sleep(3.0)
            if not wait_for_port(port):
                print("  Device not found after flash!")
                sys.exit(1)
            time.sleep(3.0)
        else:
            print("  Skipped (using firmware already on device)")
    else:
        print("  No platformio.ini found — assuming firmware already loaded.")
        print("  (Run this from the firmware project root to enable building)")

    # ─── Provision ───────────────────────────────
    header(4, "Provision device identity")

    print(f"  Connecting to {port}...")
    ser = open_serial(port)

    eeprom = read_eeprom(ser)
    already_ok = False
    locked = False

    if eeprom is not None:
        lock = eeprom[ADDR_INFO_LOCK] if len(eeprom) > ADDR_INFO_LOCK else 0x00
        locked = (lock == INFO_LOCK_BYTE)
        print(f"  Current: product=0x{eeprom[0]:02X} model=0x{eeprom[1]:02X} "
              f"{'LOCKED' if locked else 'unlocked'}")

        if locked and eeprom[0] == PRODUCT_XIAO and eeprom[1] == model:
            print("  ✓ Already provisioned correctly!")
            if not ask_yes("Re-provision with new serial number?", False):
                already_ok = True
    else:
        print("  Empty/fresh EEPROM")

    if not already_ok:
        # Unlock if locked
        if locked:
            print("  Unlocking (device will reboot)...")
            ser.write(kiss_frame(CMD_UNLOCK_ROM, bytes([ROM_UNLOCK_BYTE])))
            time.sleep(0.5)
            ser.close()
            if not wait_for_port(port):
                print("  ERROR: Device didn't come back!"); sys.exit(1)
            time.sleep(3.0)
            ser = open_serial(port)
            eeprom = read_eeprom(ser)
            if eeprom and eeprom[ADDR_INFO_LOCK] == INFO_LOCK_BYTE:
                print("  ERROR: Still locked!"); sys.exit(1)
            print("  ✓ Unlocked")

        # Generate identity
        sn = random.randint(1, 0xFFFFFFFE)
        sn_bytes = sn.to_bytes(4, 'big')
        made = (datetime.datetime.now() - datetime.datetime(1970, 1, 1)).days
        made_bytes = made.to_bytes(4, 'big')

        data = bytearray([PRODUCT_XIAO, model, 0x01])
        data.extend(sn_bytes)
        data.extend(made_bytes)
        chk = hashlib.md5(data[:CHECKSUMMED_SIZE]).digest()

        print(f"  Serial:   {sn:08X}")
        print(f"  Checksum: {chk.hex()}")

        writes = [(ADDR_PRODUCT, PRODUCT_XIAO), (ADDR_MODEL, model),
                  (ADDR_HW_REV, 0x01)]
        for i, b in enumerate(sn_bytes):  writes.append((ADDR_SERIAL+i, b))
        for i, b in enumerate(made_bytes): writes.append((ADDR_MADE+i, b))
        for i, b in enumerate(chk):       writes.append((ADDR_CHKSUM+i, b))
        writes.append((ADDR_INFO_LOCK, INFO_LOCK_BYTE))

        print(f"  Writing {len(writes)} bytes...", end='', flush=True)
        for a, v in writes:
            ser.write(kiss_frame(CMD_ROM_WRITE, bytes([a, v])))
            time.sleep(0.05)
        time.sleep(1.0)
        ser.write(kiss_frame(CMD_CONF_SAVE))
        time.sleep(1.0)
        print(" done")

        # Verify
        ser.reset_input_buffer()
        eeprom = read_eeprom(ser)
        if eeprom:
            bad = [(a,v) for a,v in writes if a < len(eeprom) and eeprom[a] != v]
            if not bad:
                print(f"  ✓ Verified OK")
            else:
                print(f"  ✗ {len(bad)} mismatches!")
                for a, v in bad:
                    print(f"    0x{a:02X}: want 0x{v:02X} got 0x{eeprom[a]:02X}")
                ser.close(); sys.exit(1)
        else:
            print("  Could not verify (proceeding)")

        ser.close()
    else:
        ser.close()

    # ─── Radio ───────────────────────────────────
    header(5, "Configure radio")

    freq = int(ask(f"Frequency in Hz ({freq_min/1e6:.0f}–{freq_max/1e6:.0f} MHz)",
                   freq_default))

    bw = ask_pick("Bandwidth:", [
        (7800,"7.8 KHz"), (10400,"10.4 KHz"), (15600,"15.6 KHz"),
        (20800,"20.8 KHz"), (31250,"31.25 KHz"), (41700,"41.7 KHz"),
        (62500,"62.5 KHz"), (125000,"125 KHz"), (250000,"250 KHz"),
        (500000,"500 KHz"),
    ], 125000)

    sf = ask_pick("Spreading factor:", [(s, f"SF{s}") for s in range(6,13)], 7)
    cr = ask_pick("Coding rate:", [(c, f"4/{c}") for c in [5,6,7,8]], 5)
    txp = int(ask("TX power in dBm (0–22)", 14))
    txp = max(0, min(22, txp))

    print(f"\n  ┌──────────────────────────────┐")
    print(f"  │ Freq : {freq/1e6:>9.3f} MHz        │")
    print(f"  │ BW   : {bw/1e3:>9.1f} KHz        │")
    print(f"  │ SF   : {sf:>9}            │")
    print(f"  │ CR   : 4/{cr:<8}            │")
    print(f"  │ TXP  : {txp:>9} dBm        │")
    print(f"  └──────────────────────────────┘")

    if not wait_for_port(port, 10):
        print("  Port not available!"); sys.exit(1)
    time.sleep(2.0)

    # Try rnodeconf, fall back to direct KISS
    r = subprocess.run(["rnodeconf", port, "--tnc",
                        "--freq", str(freq), "--bw", str(bw),
                        "--sf", str(sf), "--cr", str(cr), "--txp", str(txp)],
                       capture_output=True, text=True)
    if r.returncode == 0:
        print("  ✓ Radio configured")
    else:
        print("  rnodeconf failed, configuring directly...")
        ser = open_serial(port)
        ser.write(kiss_frame(CMD_FREQUENCY, struct.pack('>I', freq)))
        time.sleep(0.1)
        ser.write(kiss_frame(CMD_BANDWIDTH, struct.pack('>I', bw)))
        time.sleep(0.1)
        ser.write(kiss_frame(CMD_TXPOWER, bytes([txp])))
        time.sleep(0.1)
        ser.write(kiss_frame(CMD_SF, bytes([sf])))
        time.sleep(0.1)
        ser.write(kiss_frame(CMD_CR, bytes([cr])))
        time.sleep(0.1)
        ser.write(kiss_frame(CMD_CONF_SAVE))
        time.sleep(1.0)
        ser.close()
        print("  ✓ Radio configured (direct)")

    # ─── Firmware hash ───────────────────────────
    header(6, "Firmware hash")

    fw = find_fw_bin()
    if fw:
        h = hashlib.sha256(open(fw,'rb').read()).hexdigest()
        print(f"  {fw}")
        print(f"  SHA-256: {h}")
        if wait_for_port(port, 10):
            time.sleep(2.0)
            ser = open_serial(port)
            ser.write(kiss_frame(CMD_FW_HASH, bytes.fromhex(h)))
            time.sleep(0.5)
            ser.write(kiss_frame(CMD_RESET, bytes([CMD_RESET_BYTE])))
            time.sleep(0.5)
            ser.close()
            print("  ✓ Hash set")
    else:
        print("  No binary found, skipping")

    # ─── Patch rnodeconf ─────────────────────────
    header(7, "Patch rnodeconf")

    try:
        r = subprocess.run(
            [sys.executable, "-c",
             "from RNS.Utilities import rnodeconf; print(rnodeconf.__file__)"],
            capture_output=True, text=True, timeout=10)
        rncpath = r.stdout.strip() if r.returncode == 0 else None
    except Exception:
        rncpath = None

    if not rncpath:
        print("  rnodeconf not installed, skipping")
    else:
        src = open(rncpath).read()
        if "PRODUCT_XIAO_NRF52840" in src:
            print("  ✓ Already patched")
        else:
            lines, out, ok = src.split('\n'), [], False
            for line in lines:
                out.append(line)
                if 'PRODUCT_RAK4631' in line and '0x10' in line and '=' in line:
                    ind = line[:len(line)-len(line.lstrip())]
                    out.append(f"{ind}PRODUCT_XIAO_NRF52840 = 0x11")
                    ok = True
                if 'ROM.PRODUCT_RAK4631' in line and '"RAK4631"' in line:
                    ind = line[:len(line)-len(line.lstrip())]
                    out.append(f'{ind}ROM.PRODUCT_XIAO_NRF52840: '
                               f'"Seeed XIAO nRF52840 Wio-SX1262",')
                    ok = True
            if ok:
                open(rncpath,'w').write('\n'.join(out))
                print("  ✓ Patched")
            else:
                print("  Could not find insertion points — patch manually")

    # ─── Verify ──────────────────────────────────
    header(8, "Verify")

    time.sleep(3.0)
    if not wait_for_port(port, 15):
        print(f"  Device not found. Verify manually:")
        print(f"    rnodeconf {port} -i")
    else:
        time.sleep(3.0)
        r = subprocess.run(["rnodeconf", port, "-i"],
                           capture_output=True, text=True)
        skip = ['WARNING','NOT verifiable','privacy','signing key',
                'Autogenerated','Proceed','firmware downloaded','github.com',
                'reflash','recommended','autoinstall','Opening serial',
                'Device connected']
        for line in r.stdout.split('\n'):
            s = line.strip()
            if not s or any(w in s for w in skip): continue
            if s.startswith('[') and ']' in s:
                s = s[s.index(']')+1:].strip()
            if s: print(f"  {s}")

        if r.returncode == 0:
            print("\n  ✓ All good!")
        elif 'KeyError' in (r.stderr or ''):
            print("\n  rnodeconf patch may need a restart. Try:")
            print(f"    rnodeconf {port} -i")

    # ─── Done ────────────────────────────────────
    print(f"\n{'━'*55}")
    print(f"  ✓ SETUP COMPLETE — your RNode is ready!")
    print(f"{'━'*55}")
    print(f"  rnodeconf {port} -i          — device info")
    print(f"  rnodeconf {port} --tnc ...   — reconfigure radio")
    print()


if __name__ == "__main__":
    main()
