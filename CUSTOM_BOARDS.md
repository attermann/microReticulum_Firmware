# Custom Board Support
## Seeed XIAO nRF52840 + Wio-SX1262 / Heltec Wireless Stick Lite V1

| Board | MCU | Radio | Use Case |
|-------|-----|-------|----------|
| **Seeed XIAO nRF52840 + Wio-SX1262** | nRF52840 | SX1262 | Low power solar/battery node |
| **Heltec Wireless Stick Lite V1** | ESP32-PICO-D4 | SX1276 | Compact always-on USB-powered node |

---

## Quick Start

### Prerequisites

Install [uv](https://docs.astral.sh/uv/) for Python dependency management:

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

Then clone the repo and sync all dependencies (PlatformIO, `rnodeconf`, `adafruit-nrfutil`):

```bash
git clone <repo-url> && cd microreticulum-firmware
uv sync
```

---

## Board 1: Seeed XIAO nRF52840 + Wio-SX1262

A plug-and-play kit — the Wio-SX1262 expansion board plugs directly onto the XIAO headers, no soldering needed. Connect an 868/915 MHz antenna to the SMA/U.FL connector before powering on.

### All-in-One Setup

The `xiao_rnode_setup.py` script handles everything — building firmware, flashing, EEPROM provisioning, `rnodeconf` patching, and interactive radio configuration:

```bash
uv run python xiao_rnode_setup.py
```

The wizard will walk you through each step and auto-detect your serial port.

### Manual Build & Flash

```bash
# Build firmware
uv run pio run -e xiao_nrf52840

# Flash (double-tap reset button on XIAO if upload fails)
uv run pio run -e xiao_nrf52840 --target upload
```

Or use the flash script:

```bash
./flash_xiao_nrf52840.sh /dev/ttyACM0 868
```

### Manual Provisioning

If you didn't use the all-in-one setup script, provision the EEPROM separately:

```bash
uv run python provision_xiao.py /dev/ttyACM0 868
```

### Power Modes

The XIAO build includes three power tiers for solar/battery operation:

| Mode | Average Current | Battery Life (500 mAh) | Build Target |
|------|-----------------|------------------------|--------------|
| **Performance** | ~10 mA | ~50 hours | `xiao_nrf52840` |
| **Balanced** | ~1.5 mA | ~2 weeks | `xiao_nrf52840` (runtime default) |
| **Low Power** | ~500 µA | ~1 month | `xiao_nrf52840_lowpower` |

Balanced mode is the default for transport nodes. For maximum power savings, build with the low power variant:

```bash
uv run pio run -e xiao_nrf52840_lowpower
```

When using duty cycle modes, transmitting nodes should use longer preambles (18+ symbols for balanced, 32+ for low power) to ensure reliable reception.

---

## Board 2: Heltec Wireless Stick Lite V1

An all-in-one ESP32 + SX1276 board — just connect an antenna and a Micro-USB cable.

### Build & Flash

```bash
./flash_wsl_v1.sh
```

Or use `rnodeconf --autoinstall` (select [17]):

```bash
# Patch rnodeconf first (one-time)
uv run python patch_rnodeconf_hwsl_v1.py

# Then autoinstall — compiles, flashes, and provisions in one step
cd <firmware-directory>
uv run rnodeconf /dev/ttyUSB0 --autoinstall
# → select [17] Heltec Wireless Stick Lite V1
# → select band (433/868/915)
# → PlatformIO compiles + flashes + provisions automatically
```

Or manually:

```bash
uv run pio run -e heltec_wsl_v1 --target upload
```

### Provisioning

First patch rnodeconf to recognize the board (one-time):

```bash
uv run python patch_rnodeconf_hwsl_v1.py
```

Then provision (use `cb` for 433 MHz, `cc` for 868/915 MHz):

```bash
uv run rnodeconf /dev/ttyUSB0 --rom --product c5 --model cc --hwrev 1
uv run rnodeconf /dev/ttyUSB0 --firmware-hash <hash>
uv run rnodeconf /dev/ttyUSB0 --tnc --freq 869525000 --bw 125000 --sf 7 --cr 5 --txp 14
```

Or use the interactive installer:

```bash
uv run rnodeconf /dev/ttyUSB0 --autoinstall    # select [17] Heltec Wireless Stick Lite V1
```

---

## Project Scripts

| Script | Purpose |
|--------|---------|
| `xiao_rnode_setup.py` | All-in-one XIAO setup wizard (build, flash, provision, configure) |
| `flash_xiao_nrf52840.sh` | Build & flash XIAO firmware with optional provisioning |
| `flash_wsl_v1.sh` | Build & flash Heltec WSL V1 firmware |
| `patch_rnodeconf_hwsl_v1.py` | Patch rnodeconf to recognize HWSL V1 (product, models, autoinstall) |
| `provision_xiao.py` | Direct EEPROM provisioning for XIAO (KISS protocol) |

All Python scripts should be run via `uv run` to use the managed dependencies. If you prefer not to use `uv`, install dependencies manually with `pip install platformio rns adafruit-nrfutil`.

---

## Files Modified (vs upstream)

| File | Change |
|------|--------|
| `Boards.h` | Added `BOARD_XIAO_NRF52840` and `BOARD_HWSL_V1` definitions |
| `sx126x.h` / `sx126x.cpp` | SX1262 RX duty cycle for power optimization |
| `LowPower.h` / `LowPower.cpp` | New power management module (XIAO only) |
| `Bluetooth.h` | Power-optimized BLE advertising intervals |
| `Config.h` | Low power configuration constants |
| `RNode_Firmware.ino` | Event-driven loop with sleep support |
| `platformio.ini` | Added `xiao_nrf52840`, `xiao_nrf52840_lowpower`, and `heltec_wsl_v1` build environments |
| `pyproject.toml` | Python dependencies for `uv sync` |

---

## Troubleshooting

**XIAO won't enter bootloader:** Double-tap the tiny reset button quickly. A USB drive should appear.

**"eeprom hardware config invalid" after flash:** Run `provision_xiao.py` or the setup wizard — the EEPROM needs to be written before the device will operate.

**Heltec not detected on serial:** Try `/dev/ttyUSB0` (Linux) or `/dev/cu.SLAB_USBtoUART` (macOS). You may need the CP2102 USB driver.

**Missing packets with duty cycling:** Increase preamble length on transmitting nodes. Try balanced mode before low power.

---

## License

GPLv3 — same as the upstream microReticulum firmware.
