# XIAO nRF52840 Firmware Fix Report

## Root Cause: Firmware won't boot / provision fails

Six bugs were found, three of them critical (firmware cannot function at all).

---

## CRITICAL BUG #1 — EEPROM_SIZE causes negative memory offsets

**File:** `Boards.h`  
**Impact:** `device_init()` always fails → `hw_ready = false` → radio never starts

The XIAO had `EEPROM_SIZE = 200` with `EEPROM_RESERVED = 200`. The firmware stores a 64-byte device signature and 32-byte firmware hash **below** the reserved EEPROM region. The offsets are computed as:

```
DEV_SIG_OFFSET    = EEPROM_SIZE - EEPROM_RESERVED - 64 = 200 - 200 - 64 = -64   ← NEGATIVE!
DEV_FWHASH_OFFSET = EEPROM_SIZE - EEPROM_RESERVED - 64 - 32 = -96               ← NEGATIVE!
```

When the firmware calls `eeprom_read()` at these negative offsets, it gets garbage data. The firmware hash comparison fails, so `fw_signature_validated = false` and `device_init()` returns false.

**Fix:** Changed `EEPROM_SIZE` from `200` to `296` (matching RAK4631). The layout becomes:

| Byte Range | Content |
|---|---|
| 0–31 | Firmware hash target (32 bytes) |
| 32–95 | Device signature (64 bytes) |
| 96–295 | Regular EEPROM (product info, config, etc.) |

---

## CRITICAL BUG #2 — Provision script writes wrong INFO_LOCK_BYTE

**File:** `provision_xiao.py`  
**Impact:** `eeprom_lock_set()` returns false → "Error, eeprom lock not set"

The provision script wrote `0x01` for `ADDR_INFO_LOCK`, but the firmware checks for `INFO_LOCK_BYTE = 0x73`.

**Fix:** Changed to write `0x73`.

---

## CRITICAL BUG #3 — extra_script.py doesn't recognize Seeed platform

**File:** `extra_script.py`  
**Impact:** `--specs=nano.specs` not removed → C++ exceptions break → link/runtime failures. Firmware hash not set after upload.

The XIAO uses `platform = https://github.com/Seeed-Studio/platform-seeedboards.git` but the script only checked for `platform == "nordicnrf52"`.

**Fix:** Added `is_nrf52_platform()` helper that matches both `"nordicnrf52"` and any URL containing `"platform-seeedboards"`.

---

## IMPORTANT BUG #4 — Serial wait blocks headless boot

**File:** `RNode_Firmware.ino`  
**Impact:** Device hangs forever on boot when no USB is connected (fatal for solar-powered transport node)

The XIAO was not excluded from `while (!Serial)` in `setup()`.

**Fix:** Added `BOARD_MODEL != BOARD_XIAO_NRF52840` to the exclusion condition.

---

## IMPORTANT BUG #5 — Hardcoded firmware hash in provision script

**File:** `provision_xiao.py`  
**Impact:** Firmware hash mismatch even if EEPROM_SIZE was correct

The old script had a hardcoded SHA-256 hash that couldn't match the actual compiled firmware.

**Fix:** Provision script now computes the hash from the actual `.bin` file at provision time.

---

## MODERATE BUG #6 — platformio.ini has ESP32-only settings for XIAO

**File:** `platformio.ini`  
**Impact:** `board_build.partitions = no_ota.csv` is meaningless for nRF52 (ignored, but confusing). Missing `lib_archive = no` (needed for Bluefruit BLE library). Missing `-DRNS_USE_ALLOCATOR=1 -DRNS_USE_TLSF=1` (needed for microReticulum on nRF52). Missing `build_unflags = -fno-exceptions`.

**Fix:** Removed ESP32-only settings, added all required nRF52/XIAO flags.

---

## Files Changed

| File | Change |
|---|---|
| `Boards.h` | `EEPROM_SIZE 200` → `296` |
| `RNode_Firmware.ino` | Added XIAO to serial-wait exclusion |
| `extra_script.py` | Added Seeed platform detection |
| `provision_xiao.py` | Fixed lock byte, dynamic firmware hash |
| `flash_xiao_nrf52840.sh` | Updated provisioning flow |
| `platformio.ini` | Fixed nRF52 build config for XIAO |

## Build & Flash Procedure

```bash
# 1. Build
pio run -e xiao_nrf52840
#    (or xiao_nrf52840_lowpower for solar operation)

# 2. Double-tap reset on XIAO to enter bootloader

# 3. Flash + provision
./flash_xiao_nrf52840.sh /dev/ttyACM0 868
#    (or use provision_xiao.py directly after pio upload)

# 4. Verify
rnodeconf /dev/ttyACM0 -i
```

## Energy Efficiency Notes (Solar Transport Node)

- Use `xiao_nrf52840_lowpower` environment for solar operation
- The `while (!Serial)` fix means the device boots immediately without USB
- LowPower.h implements three modes: PERFORMANCE (~10mA), BALANCED (~1.5mA), LOW_POWER (~500µA)
- BLE advertising interval is automatically slowed in low-power modes
- FreeRTOS tickless idle is leveraged during `delay()` in the main loop
