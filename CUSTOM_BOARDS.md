# Custom Board Support
## Seeed XIAO nRF52840 + Wio-SX1262 / Heltec Wireless Stick Lite V1 / TTGO LoRa32 V1

| Board | MCU | Radio | Use Case |
|-------|-----|-------|----------|
| **Seeed XIAO nRF52840 + Wio-SX1262** | nRF52840 | SX1262 | Low power solar/battery node |
| **Heltec Wireless Stick Lite V1** | ESP32-PICO-D4 | SX1276 | Compact always-on USB-powered node |
| **TTGO LoRa32 V1** | ESP32 | SX1276 | General purpose node with OLED display |

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

### GPIO Control (XIAO nRF52840)

The XIAO nRF52840 + Wio-SX1262 kit has very few free GPIO pins because most are consumed by the radio module. Only three digital pins are available for GPIO control:

| Pin | Arduino Name | nRF52840 GPIO | Direction |
|-----|-------------|---------------|-----------|
| D0  | 0           | P0.02         | I/O       |
| D6  | 6           | P1.11         | I/O       |
| D7  | 7           | P1.12         | I/O       |

**Pins D1–D5, D8–D10 are used by the SX1262 radio and must not be reassigned.**

**Step 1 — Edit the pin table.** Open `GPIO_Control.h` and replace the default `gpio_pins[]` array with:

```cpp
static GPIOPin gpio_pins[] = {
    { 0, true, false, INPUT, "D0" },
    { 6, true, false, INPUT, "D6" },
    { 7, true, false, INPUT, "D7" },
};
```

**Step 2 — Add the build flag.** In `platformio.ini`, add `-DHAS_GPIO_CONTROL` to the `xiao_nrf52840` environment's `build_flags`:

```ini
[env:xiao_nrf52840]
build_flags =
	${env.build_flags}
	-fexceptions
	-DBOARD_MODEL=BOARD_XIAO_NRF52840
	-DHAS_GPIO_CONTROL
	-DRNS_USE_ALLOCATOR=1
	-DRNS_USE_TLSF=1
	-Wl,--allow-multiple-definition
```

**Step 3 — Build and flash.**

```bash
rm -rf .pio/build
uv run pio run -e xiao_nrf52840 --target upload
```

After boot, the LXMF address will appear in the serial output. Send commands from Sideband using **opportunistic** delivery.

> **Note:** With only 3 GPIO pins, the XIAO is best suited for simple sensor inputs or relay triggers. For projects needing more I/O, use the TTGO LoRa32 v1 or Heltec WSL V1.

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

### GPIO Control (Heltec WSL V1)

The Heltec WSL V1 has a good selection of free GPIO pins since there is no display.

**Pins used by the radio (do not reassign):** 5 (SCK), 14 (Reset), 18 (CS), 19 (MISO), 25 (LED), 26 (DIO), 27 (MOSI).

| Pin | Direction | Notes |
|-----|-----------|-------|
| 12  | I/O       | |
| 13  | I/O       | |
| 17  | I/O       | |
| 23  | I/O       | |
| 32  | I/O       | |
| 33  | I/O       | |
| 34  | Input only | no internal pull-up |
| 35  | Input only | no internal pull-up |
| 36  | Input only | no internal pull-up |
| 39  | Input only | no internal pull-up |

**Step 1 — Apply the FileSystem.cpp patch.** All ESP32-based boards require the patched `FileSystem.cpp` from this repo. See [FileSystem.cpp Patch](#filesystemcpp-patch-esp32-boards) for details on what it fixes.

**Step 2 — Edit the pin table.** Open `GPIO_Control.h` and replace the default `gpio_pins[]` array with:

```cpp
static GPIOPin gpio_pins[] = {
    { 12, true,  false, INPUT, "GP12" },
    { 13, true,  false, INPUT, "GP13" },
    { 17, true,  false, INPUT, "GP17" },
    { 23, true,  false, INPUT, "GP23" },
    { 32, true,  false, INPUT, "GP32" },
    { 33, true,  false, INPUT, "GP33" },
    { 34, false, false, INPUT, "GP34 input-only" },
    { 35, false, false, INPUT, "GP35 input-only" },
    { 36, false, false, INPUT, "GP36 input-only" },
    { 39, false, false, INPUT, "GP39 input-only" },
};
```

**Step 3 — Enable RNS and GPIO control in platformio.ini.** The default `heltec_wsl_v1` environment has RNS disabled. Enable it along with GPIO control:

```ini
[env:heltec_wsl_v1]
platform = espressif32
board = heltec_wireless_stick_lite
custom_variant = heltec_wsl_v1
board_build.partitions = no_ota.csv
board_build.filesystem = littlefs
build_flags =
	-Wall
	-Wno-missing-field-initializers
	-Wno-format
	-I.
	-DBOARD_MODEL=BOARD_HWSL_V1
	-DHAS_RNS
	-DRNS_USE_FS
	-DHAS_GPIO_CONTROL
	-DNDEBUG
lib_deps =
	${env.lib_deps}
	https://github.com/attermann/microReticulum.git
```

> **Note:** RNS was previously disabled on this board due to a heap corruption crash. This was caused by the double-free bug in `FileSystem.cpp`, which the patched version fixes. With the patch applied, RNS runs stable.

**Step 4 — Build, flash, and provision.**

```bash
rm -rf .pio/build

# Upload LittleFS filesystem (creates the /cache/ partition format)
uv run pio run -e heltec_wsl_v1 -t uploadfs --upload-port /dev/ttyUSB0

# Build and flash firmware
uv run pio run -e heltec_wsl_v1 -t upload --upload-port /dev/ttyUSB0

# Wait for boot, then set firmware hash
sleep 2
uv run rnodeconf --firmware-hash $(shasum -a 256 \
  .pio/build/heltec_wsl_v1/*.bin | cut -d' ' -f1) /dev/ttyUSB0
```

After boot, the LXMF address will appear in the serial output. Send commands from Sideband using **opportunistic** delivery.

---

## Board 3: TTGO LoRa32 V1

An ESP32 board with built-in SX1276 radio and 0.96" OLED display. Widely available and well-suited for GPIO control with the most free pins of the three boards.

### GPIO Control (TTGO LoRa32 V1)

**Pins used by the radio and display (do not reassign):** 2 (LED), 4 (OLED SDA), 5 (SCK), 14 (Reset), 15 (OLED SCL), 16 (OLED RST), 18 (CS), 19 (MISO), 26 (DIO), 27 (MOSI).

| Pin | Direction | Notes |
|-----|-----------|-------|
| 13  | I/O       | |
| 17  | I/O       | |
| 23  | I/O       | |
| 25  | I/O       | |
| 32  | I/O       | |
| 33  | I/O       | |
| 34  | Input only | no internal pull-up |
| 35  | Input only | no internal pull-up |
| 36  | Input only | no internal pull-up |
| 39  | Input only | no internal pull-up |

**Step 1 — Apply the FileSystem.cpp patch.** All ESP32-based boards require the patched `FileSystem.cpp` from this repo. See [FileSystem.cpp Patch](#filesystemcpp-patch-esp32-boards) for details on what it fixes.

**Step 2 — Edit the pin table (optional).** The default `gpio_pins[]` array in `GPIO_Control.h` is already configured for the TTGO LoRa32 v1 — no changes needed unless you want to add or remove pins.

**Step 3 — The platformio.ini environment.** The `ttgo-lora32-v1` environment already has GPIO control enabled:

```ini
[env:ttgo-lora32-v1]
platform = espressif32
board = ttgo-lora32-v1
custom_variant = lora32v10
board_build.partitions = no_ota.csv
board_build.filesystem = littlefs
build_flags =
	${env.build_flags}
	-DBOARD_MODEL=BOARD_LORA32_V1_0
	-DHAS_GPIO_CONTROL
	-DNDEBUG
	-DARDUINO_LOOP_STACK_SIZE=16384
lib_deps =
	${env.lib_deps}
	adafruit/Adafruit SSD1306@^2.5.9
	XPowersLib@^0.2.1
	https://github.com/attermann/microReticulum.git
```

**Step 4 — Build, flash, and provision.**

```bash
rm -rf .pio/build

# Upload LittleFS filesystem (creates the /cache/ partition format)
uv run pio run -e ttgo-lora32-v1 -t uploadfs --upload-port /dev/cu.usbserial-0001

# Build and flash firmware
uv run pio run -e ttgo-lora32-v1 -t upload --upload-port /dev/cu.usbserial-0001

# Wait for boot, then set firmware hash
sleep 2
uv run rnodeconf --firmware-hash $(shasum -a 256 \
  .pio/build/ttgo-lora32-v1/rnode_firmware_lora32v10.bin | cut -d' ' -f1) \
  /dev/cu.usbserial-0001
```

After boot, the LXMF address will appear in the serial output. Send commands from Sideband using **opportunistic** delivery.

---

## GPIO Control — General Information

GPIO control lets you set and read hardware pins on the RNode via LXMF messages sent from [Sideband](https://github.com/markqvist/Sideband). It works over any Reticulum transport — LoRa, serial, TCP, or mesh.

### How It Works

The firmware runs a minimal LXMF endpoint alongside the RNode TNC. When a message arrives, `GPIO_Control.h` parses the text command, performs the GPIO operation, and sends a reply with the result. The node announces its LXMF address on boot, so Sideband can discover it automatically.

### Delivery Mode

Always use **opportunistic** delivery in Sideband. Direct delivery requires a full Reticulum Link handshake which is too resource-intensive for microcontrollers over LoRa.

### Commands

All commands are case-insensitive.

| Command | Example | Description |
|---------|---------|-------------|
| `SET <pin> HIGH\|LOW` | `SET 13 HIGH` | Set output pin high or low (also accepts `ON`/`OFF`/`1`/`0`) |
| `GET <pin>` | `GET 32` | Read current pin state |
| `MODE <pin> IN\|OUT` | `MODE 12 OUT` | Set pin direction |
| `PINS` | `PINS` | List all available GPIO pins |
| `STATUS` | `STATUS` | Report states of all configured pins |
| `HELP` | `HELP` | Show command list |

### Required Files

| File | Purpose |
|------|---------|
| `GPIO_Control.h` | Command parser and pin control logic |
| `LXMF_Minimal.h` | Lightweight LXMF send/receive |
| `FileSystem.cpp` | Patched filesystem layer (ESP32 only — see below) |

### FileSystem.cpp Patch (ESP32 boards)

All ESP32-based boards (TTGO LoRa32 v1, Heltec WSL V1, and others) require the patched `FileSystem.cpp` from this repo. The upstream version has four bugs that cause crashes and silent data loss:

1. **Double-free in `open_file()`** — the ESP32 code path creates two `unique_ptr<FileStream>` wrappers around the same `File*` pointer. The second `delete` corrupts the heap, causing `CORRUPT HEAP: Bad head` crashes during `Transport::start()`.

2. **Missing `/cache/` directory** — microReticulum stores identity and path caches in `/cache/<hash>`, but LittleFS won't create files when the parent directory doesn't exist. The patch creates `/cache/` at boot.

3. **`create=true` parameter** — ESP32 Arduino Core 3.x defaults `FS.open()` to `create=false`, which prevents file creation inside subdirectories even when the parent exists.

4. **Filename length limit** — ESP32 LittleFS is compiled with `CONFIG_LITTLEFS_OBJ_NAME_LEN=64` (63 usable characters), but microReticulum uses 64-character SHA256 hex hashes as cache filenames. The patch truncates basenames to 32 characters (128 bits — astronomically collision-proof).

The XIAO nRF52840 uses InternalFS (not LittleFS) and has a different `open_file()` code path, so these bugs do not apply.

If you haven't already replaced `FileSystem.cpp`, copy the patched version from this repo into your project root before building.

---

## Project Scripts

| Script | Purpose |
|--------|---------|
| `xiao_rnode_setup.py` | All-in-one XIAO setup wizard (build, flash, provision, configure) |
| `flash_xiao_nrf52840.sh` | Build & flash XIAO firmware with optional provisioning |
| `flash_wsl_v1.sh` | Build & flash Heltec WSL V1 firmware |
| `flash_ttgo_v1.sh` | Build & flash TTGO LoRa32 v1 with GPIO control |
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
| `RNode_Firmware.ino` | Event-driven loop with sleep support, heap diagnostics |
| `platformio.ini` | Added `xiao_nrf52840`, `xiao_nrf52840_lowpower`, `heltec_wsl_v1`, and `ttgo-lora32-v1` build environments |
| `pyproject.toml` | Python dependencies for `uv sync` |
| `FileSystem.cpp` | Fixed double-free, cache directory, `create=true`, filename truncation |
| `GPIO_Control.h` | New — LXMF-based GPIO control |
| `LXMF_Minimal.h` | New — lightweight LXMF message handling |

---

## Troubleshooting

**XIAO won't enter bootloader:** Double-tap the tiny reset button quickly. A USB drive should appear.

**"eeprom hardware config invalid" after flash:** Run `provision_xiao.py` or the setup wizard — the EEPROM needs to be written before the device will operate.

**Heltec not detected on serial:** Try `/dev/ttyUSB0` (Linux) or `/dev/cu.SLAB_USBtoUART` (macOS). You may need the CP2102 USB driver.

**TTGO not detected on serial:** Try `/dev/cu.usbserial-0001` (macOS) or `/dev/ttyUSB0` (Linux). You may need the CH9102/CP2102 USB driver.

**Missing packets with duty cycling:** Increase preamble length on transmitting nodes. Try balanced mode before low power.

**"CORRUPT HEAP" crash on ESP32 during boot:** You are using the unpatched `FileSystem.cpp`. Replace it with the patched version from this repo.

**Cache write errors (`write_file: failed to open output file /cache/...`):** Either the `/cache/` directory doesn't exist or cache filenames exceed the 63-character LittleFS limit. Both are fixed by the patched `FileSystem.cpp`.

**"sender identity not known (not announced?)":** The sending device hasn't announced itself recently. Make sure Sideband has announced, then retry.

**Opportunistic messages work but direct delivery fails:** Direct delivery requires a Reticulum Link handshake that is too resource-intensive for microcontrollers. Always use opportunistic delivery in Sideband.

---

## License

GPLv3 — same as the upstream microReticulum firmware.
