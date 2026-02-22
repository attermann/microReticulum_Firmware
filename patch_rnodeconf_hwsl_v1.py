#!/usr/bin/env python3
"""
Patch rnodeconf to recognize and autoinstall the Heltec Wireless Stick Lite V1.

8 patches:
  1-3  ROM constants, products dict, models dict
  4-7  --autoinstall menu entry [17], range check, device+band selection
  8    Intercept BEFORE ensure_firmware_file() for HWSL_V1 →
       self-contained PIO compile → flash → provision → hash → verify → exit

Run:  python patch_rnodeconf_hwsl_v1.py          # auto-finds rnodeconf
      python patch_rnodeconf_hwsl_v1.py <path>    # explicit path
"""

import sys, os, subprocess, shutil

MARKER = "PRODUCT_HWSL_V1"


def find_rnodeconf(explicit=None):
    if explicit and os.path.isfile(explicit):
        return explicit
    try:
        r = subprocess.run(
            [sys.executable, "-c",
             "from RNS.Utilities import rnodeconf; print(rnodeconf.__file__)"],
            capture_output=True, text=True, timeout=10)
        if r.returncode == 0 and r.stdout.strip():
            return r.stdout.strip()
    except Exception:
        pass
    for root in [".", ".."]:
        for p in ["venv", ".venv"]:
            candidate = os.path.join(root, p, "lib")
            if os.path.isdir(candidate):
                for dirpath, _, filenames in os.walk(candidate):
                    if "rnodeconf.py" in filenames:
                        return os.path.join(dirpath, "rnodeconf.py")
    return None


def replace_once(content, old, new, label):
    if old in content:
        content = content.replace(old, new, 1)
        print(f"  ✓ {label}")
        return content, True
    print(f"  ✗ {label} — pattern not found")
    return content, False


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else None
    rnc = find_rnodeconf(path)
    if not rnc:
        print("ERROR: Cannot find rnodeconf.py")
        print("  pip install rns")
        sys.exit(1)

    print(f"Found rnodeconf: {rnc}")

    with open(rnc, "r") as f:
        content = f.read()

    if MARKER in content:
        print(f"  ✓ Already patched ({MARKER} found)")
        sys.exit(0)

    bak = rnc + ".bak_hwsl"
    if not os.path.exists(bak):
        shutil.copy2(rnc, bak)
        print(f"  Backup → {bak}")

    ok = 0
    total = 8

    # ── 1. ROM constants ──────────────────────────────
    content, hit = replace_once(content,
        "    PRODUCT_H32_V4      = 0xC3\n    MODEL_C8",
        "    PRODUCT_H32_V4      = 0xC3\n"
        "    PRODUCT_HWSL_V1     = 0xC5\n"
        "    MODEL_CB            = 0xCB\n"
        "    MODEL_CC            = 0xCC\n\n"
        "    MODEL_C8",
        "1/8  ROM constants")
    if hit: ok += 1

    # ── 2. products dict ──────────────────────────────
    content, hit = replace_once(content,
        'ROM.PRODUCT_H32_V4: "Heltec LoRa32 v4",',
        'ROM.PRODUCT_H32_V4: "Heltec LoRa32 v4",\n'
        '    ROM.PRODUCT_HWSL_V1: "Heltec Wireless Stick Lite V1",',
        "2/8  products dict")
    if hit: ok += 1

    # ── 3. models dict ────────────────────────────────
    # Use real firmware zip name (needed for display in "Installer Ready")
    # but we intercept before ensure_firmware_file ever runs
    c8 = '    0xC8: [860000000, 930000000, 28, "850 - 950 MHz", "rnode_firmware_heltec32v4pa.zip", "SX1262"],'
    content, hit = replace_once(content, c8,
        c8 + '\n'
        '    0xCB: [420000000, 520000000, 17, "420 - 520 MHz", "rnode_firmware_heltec_wsl_v1.zip", "SX1278"],\n'
        '    0xCC: [850000000, 950000000, 17, "850 - 950 MHz", "rnode_firmware_heltec_wsl_v1.zip", "SX1276"],',
        "3/8  models dict (0xCB, 0xCC)")
    if hit: ok += 1

    # ── 4. autoinstall menu [17] ──────────────────────
    content, hit = replace_once(content,
        '            print("[16] Seeed XIAO ESP32S3 Wio-SX1262")',
        '            print("[16] Seeed XIAO ESP32S3 Wio-SX1262")\n'
        '            print("[17] Heltec Wireless Stick Lite V1")',
        "4/8  menu entry [17]")
    if hit: ok += 1

    # ── 5. range check ────────────────────────────────
    content, hit = replace_once(content,
        "if c_dev < 1 or c_dev > 16:",
        "if c_dev < 1 or c_dev > 17:",
        "5/8  range check (16→17)")
    if hit: ok += 1

    # ── 6. device selection (c_dev == 17) ─────────────
    xiao_marker = 'SeeedStudio XIAO esp32s3 wio RNode Installer'
    if xiao_marker in content:
        xiao_pos = content.index(xiao_marker)
        except_pos = content.index('            except Exception as e:', xiao_pos)
        pre = content[:except_pos]
        inp_pos = pre.rfind('                    input()')
        if inp_pos > xiao_pos:
            pt = content.index('\n', inp_pos) + 1
            blk = (
                '                elif c_dev == 17:\n'
                '                    selected_product = ROM.PRODUCT_HWSL_V1\n'
                '                    clear()\n'
                '                    print("")\n'
                '                    print("---------------------------------------------------------------------------")\n'
                '                    print("              Heltec Wireless Stick Lite V1 RNode Installer")\n'
                '                    print("")\n'
                '                    print("Important! Using RNode firmware on Heltec devices should currently be")\n'
                '                    print("considered experimental. It is not intended for production or critical use.")\n'
                '                    print("")\n'
                '                    print("The currently supplied firmware is provided AS-IS as a courtesy to those")\n'
                '                    print("who would like to experiment with it. Hit enter to continue.")\n'
                '                    print("---------------------------------------------------------------------------")\n'
                '                    input()\n'
            )
            content = content[:pt] + blk + content[pt:]
            ok += 1
            print("  ✓ 6/8  device selection (c_dev == 17)")
        else:
            print("  ✗ 6/8  insertion point not found")
    else:
        print("  ✗ 6/8  XIAO marker not found")

    # ── 7. band/model selection ───────────────────────
    xm = 'elif selected_product == ROM.PRODUCT_XIAO_S3:'
    if xm in content:
        mp = content.index(xm)
        ep = content.index('except Exception as e:', mp)
        xp = content.index('exit()', ep)
        eol = content.index('\n', xp) + 1
        blk = (
            '\n'
            '            elif selected_product == ROM.PRODUCT_HWSL_V1:\n'
            '                selected_mcu = ROM.MCU_ESP32\n'
            '                print("\\nWhat band is this Heltec Wireless Stick Lite V1 for?\\n")\n'
            '                print("[1] 433 MHz")\n'
            '                print("[2] 868 MHz")\n'
            '                print("[3] 915 MHz")\n'
            '                print("\\n? ", end="")\n'
            '                try:\n'
            '                    c_model = int(input())\n'
            '                    if c_model < 1 or c_model > 3:\n'
            '                        raise ValueError()\n'
            '                    elif c_model == 1:\n'
            '                        selected_model = ROM.MODEL_CB\n'
            '                        selected_platform = ROM.PLATFORM_ESP32\n'
            '                    elif c_model > 1:\n'
            '                        selected_model = ROM.MODEL_CC\n'
            '                        selected_platform = ROM.PLATFORM_ESP32\n'
            '                except Exception as e:\n'
            '                    print("That band does not exist, exiting now.")\n'
            '                    exit()\n'
        )
        content = content[:eol] + blk + content[eol:]
        ok += 1
        print("  ✓ 7/8  band/model selection")
    else:
        print("  ✗ 7/8  XIAO S3 model block not found")

    # ── 8. Intercept BEFORE ensure_firmware_file ──────
    # This is the critical fix. The normal flow is:
    #   args.flash = True
    #   ensure_firmware_file(fw_filename)  ← downloads from GitHub
    #   rnode.disconnect()
    #
    # For HWSL_V1 we hijack right before ensure_firmware_file:
    #   compile with PIO → flash → provision → hash → verify → exit
    # Then graceful_exit() so we never enter the download pipeline.
    ensure_block = (
        '            try:\n'
        '                RNS.log("Checking firmware file availability...")\n'
        '                ensure_firmware_file(fw_filename)'
    )
    hwsl_intercept = (
        '            if selected_product == ROM.PRODUCT_HWSL_V1:\n'
        '                import shutil as _sh, subprocess as _sp, time as _t, hashlib as _hl\n'
        '                _pio = _sh.which("pio") or _sh.which("platformio")\n'
        '                _port = selected_port.device\n'
        '                if not (_pio and os.path.isfile("platformio.ini")):\n'
        '                    print("")\n'
        '                    print("No pre-built firmware exists for the Heltec Wireless Stick Lite V1.")\n'
        '                    print("Run from the firmware directory with PlatformIO installed:")\n'
        '                    print("")\n'
        '                    if not _pio: print("  pip install platformio")\n'
        '                    print("  python wsl_v1_setup.py           # all-in-one wizard")\n'
        '                    print("  # or manually:")\n'
        '                    print("  pio run -e heltec_wsl_v1 -t upload")\n'
        '                    print("  rnodeconf " + _port + " --rom --product c5 --model cc --hwrev 1")\n'
        '                    print("")\n'
        '                    graceful_exit()\n'
        '                # ── Compile ──\n'
        '                print("\\nCompiling firmware with PlatformIO...\\n")\n'
        '                if _sp.call([_pio, "run", "-e", "heltec_wsl_v1"]) != 0:\n'
        '                    RNS.log("Build failed"); graceful_exit()\n'
        '                # ── Flash ──\n'
        '                print("\\nFlashing firmware to " + _port + "...\\n")\n'
        '                rnode.disconnect()\n'
        '                if _sp.call([_pio, "run", "-e", "heltec_wsl_v1", "-t", "upload",\n'
        '                             "--upload-port", _port]) != 0:\n'
        '                    RNS.log("Flash failed"); graceful_exit()\n'
        '                print("")\n'
        '                print("━" * 60)\n'
        '                print("  Firmware flashed. Please PRESS THE RESET BUTTON on the")\n'
        '                print("  board now, then wait a few seconds for it to boot.")\n'
        '                print("  Hit enter when ready.")\n'
        '                print("━" * 60)\n'
        '                input()\n'
        '                _t.sleep(3)\n'
        '                # ── Provision EEPROM ──\n'
        '                _model_hex = "cb" if selected_model == ROM.MODEL_CB else "cc"\n'
        '                RNS.log("Provisioning EEPROM...")\n'
        '                _sp.call(["rnodeconf", _port, "--rom", "--product", "c5",\n'
        '                          "--model", _model_hex, "--hwrev", "1"])\n'
        '                _t.sleep(3)\n'
        '                # ── Firmware hash ──\n'
        '                _bin = ".pio/build/heltec_wsl_v1/rnode_firmware_heltec_wsl_v1.bin"\n'
        '                if os.path.isfile(_bin):\n'
        '                    _fwd = open(_bin, "rb").read()\n'
        '                    _calc = _hl.sha256(_fwd[0:-32]).digest()\n'
        '                    _part = _fwd[-32:]\n'
        '                    if _calc == _part:\n'
        '                        RNS.log("Setting firmware hash...")\n'
        '                        _sp.call(["rnodeconf", _port, "--firmware-hash", _part.hex()])\n'
        '                    else:\n'
        '                        RNS.log("Firmware hash embed mismatch, skipping")\n'
        '                else:\n'
        '                    RNS.log("Firmware binary not found at " + _bin + ", skipping hash")\n'
        '                # ── Verify ──\n'
        '                _t.sleep(2)\n'
        '                print("")\n'
        '                print("━" * 60)\n'
        '                print("  Verifying device...")\n'
        '                print("━" * 60)\n'
        '                _sp.call(["rnodeconf", _port, "-i"])\n'
        '                print("")\n'
        '                print("━" * 60)\n'
        '                print("  ✓ Heltec Wireless Stick Lite V1 setup complete!")\n'
        '                print("")\n'
        '                print("  To enable standalone transport mode:")\n'
        '                print("    rnodeconf " + _port + " --tnc --freq 869525000 --bw 125000 --sf 7 --cr 5 --txp 14")\n'
        '                print("━" * 60)\n'
        '                graceful_exit()\n'
        '\n'
        '            try:\n'
        '                RNS.log("Checking firmware file availability...")\n'
        '                ensure_firmware_file(fw_filename)'
    )
    content, hit = replace_once(content, ensure_block, hwsl_intercept,
        "8/8  intercept before ensure_firmware_file → PIO compile+flash+provision")
    if hit: ok += 1

    # ── Write ─────────────────────────────────────────
    with open(rnc, "w") as f:
        f.write(content)

    print(f"\n{'━'*60}")
    print(f"  {ok}/{total} applied")
    if ok == total:
        print("  ✓ All patches applied!")
        print()
        print("  Usage (from the firmware directory):")
        print("    rnodeconf <port> --autoinstall")
        print("      → [17] → band → PIO compile → flash → provision → done")
        print("    rnodeconf <port> -i")
    elif ok > 0:
        print(f"  ⚠ {total-ok} failed — check warnings above")
    else:
        shutil.copy2(bak, rnc)
        print("  ✗ Restored from backup")
    print(f"{'━'*60}")
    return ok == total


if __name__ == "__main__":
    sys.exit(0 if main() else 1)
