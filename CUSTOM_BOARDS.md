# Custom Board Support
## Seeed XIAO nRF52840 / XIAO ESP32-S3 / Heltec Wireless Stick Lite V1 / TTGO LoRa32 V1

| Board | MCU | Radio | Use Case |
|-------|-----|-------|----------|
| **Seeed XIAO nRF52840 + Wio-SX1262** | nRF52840 | SX1262 | Low power solar/battery node |
| **Seeed XIAO ESP32-S3 + Wio-SX1262** | ESP32-S3 | SX1262 | Solar LoRa-only transport node (WiFi/BLE disabled) |
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

### LXMF Remote Management (XIAO nRF52840)

The XIAO nRF52840 firmware includes an LXMF endpoint for remote node management. The node announces itself on boot and can be messaged from Sideband or MeshChat. See [LXMF Remote Control](#lxmf-remote-control--general-information) below for the full command list.

**Battery monitoring:** The firmware reads battery voltage via the onboard ADC and detects charging state using the BQ25101 charger IC's `/CHG` pin (P0.17) combined with the nRF52840 USB power register. Three states are reported: discharging, charging, and charged.

The `xiao_nrf52840` environment has LXMF management enabled by default (`-DHAS_GPIO_CONTROL` in `platformio.ini`). The node name is `RTransport-XXXX` where XXXX is a random 4-digit number generated at compile time.

After boot, the LXMF address appears in the serial output. Send commands from Sideband or MeshChat using **opportunistic** delivery.

---

## Board 2: Seeed XIAO ESP32-S3 + Wio-SX1262

The same Wio-SX1262 carrier as Board 1, but with the ESP32-S3 variant of the XIAO module — better suited as a solar-powered LoRa-only transport router. WiFi and BLE are disabled in the build; only LoRa is active. The radio uses an SX1262 via B2B connector.

### Hardware Notes

- **Hardware:** XIAO ESP32-S3 plugs into the Wio-SX1262 expansion board (B2B connector — no soldering).
- **BOOT button is tiny.** The Wio carrier hides the XIAO module's BOOT button (also a tiny pad on top of the XIAO). To enter bootloader manually: hold the XIAO's BOOT pad while plugging in USB. The Wio carrier's user button is **not** BOOT.
- **Once flashed**, subsequent reflashes can use the firmware's `CMD_BOOTLOADER` KISS command to trigger download mode in software — the flash script attempts this first before falling back to `esptool usb_reset`, then to the physical BOOT pad.
- **LED is active-low** (GPIO 21). Firmware drives it dark in idle, brief blink on TX/RX activity — minimises battery drain.
- **TCXO is present** on the Wio-SX1262 and is automatically driven by the SX1262 from DIO3. No external TCXO enable pin needed.

### Build & Flash

```bash
./flash_xiao_esp32s3.sh
```

The script builds, attempts software entry to bootloader, falls back to `esptool` reset, flashes, then patches `rnodeconf` and provisions the EEPROM (ROM signature, model, firmware hash) and configures the radio defaults (868.800 MHz, BW 125 kHz, SF 8, CR 5, TX 14 dBm). All flags overridable — see `--help`.

Or manually:

```bash
uv run pio run -e xiao_esp32s3 -t upload
```

### Power Optimisation for Solar Operation

The `xiao_esp32s3` build is tuned for solar/battery deployment:

- **WiFi disabled**: never initialised. Library not even pulled in (`HAS_CONSOLE=false` short-circuits `Console.h`).
- **BLE disabled**: `HAS_BLE=false`. `esp_bt_controller_mem_release(ESP_BT_MODE_BLE)` is called right after `Serial.begin()` in `setup()`, releasing ~30 KB of BT-reserved RAM back to the general heap.
- **LED inverted** to active-low so the user LED is dark in idle (only blinks on activity).
- **Loop-task stack raised** to 16 KB (`-DARDUINO_LOOP_STACK_SIZE=16384`) to give RNS recursion headroom; the framework default of 8 KB is tight for transport-mode workloads.

Net effect: at idle the radio is the only RF section drawing power; the LED is dark; ~30 KB more free heap available for RNS routing.

### LXMF Remote Management (XIAO ESP32-S3)

The same LXMF endpoint as the XIAO nRF52840 — see [LXMF Remote Control](#lxmf-remote-control--general-information) below for the command list. Battery globals on this board read 0 because `HAS_PMU=false` and no battery-sensing branch exists for this board in `Power.h`, so the `BATTERY` command will reply `Battery: not available`. `ANNOUNCE`, `DUTY`, and `HELP` all work normally.

---

## Board 3: Heltec Wireless Stick Lite V1

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

### Pin Reference

**Used by the radio (do not reassign):** 5 (SCK), 14 (Reset), 18 (CS), 19 (MISO), 25 (LED), 26 (DIO), 27 (MOSI).

Free GPIOs (for forks that want to re-add pin control): 12, 13, 17, 23, 32, 33, 34 (input-only), 35 (input-only), 36 (input-only), 39 (input-only).

### Build & Flash

**Step 1 — Apply the FileSystem.cpp patch.** All ESP32-based boards require the patched `FileSystem.cpp` from this repo. See [FileSystem.cpp Patch](#filesystemcpp-patch-esp32-boards) for details on what it fixes.

**Step 2 — Enable RNS and the LXMF endpoint in platformio.ini.** The default `heltec_wsl_v1` environment has RNS disabled. Enable it along with the LXMF command endpoint (`HAS_GPIO_CONTROL` is the historical flag name; it now gates only the LXMF text-command endpoint, not any pin toggling):

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

**Step 3 — Build, flash, and provision.**

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

After boot, the LXMF address will appear in the serial output. Send commands from Sideband using **opportunistic** delivery — see [LXMF Remote Control](#lxmf-remote-control--general-information) for the command list.

---

## Board 4: TTGO LoRa32 V1

An ESP32 board with built-in SX1276 radio and 0.96" OLED display. Widely available, USB-powered, well-suited as a general-purpose RNS transport node with a visible status display.

### Pin Reference

**Used by the radio and display (do not reassign):** 2 (LED), 4 (OLED SDA), 5 (SCK), 14 (Reset), 15 (OLED SCL), 16 (OLED RST), 18 (CS), 19 (MISO), 26 (DIO), 27 (MOSI).

Free GPIOs (for forks that want to re-add pin control): 13, 17, 23, 25, 32, 33, 34 (input-only), 35 (input-only), 36 (input-only), 39 (input-only).

### Build & Flash

**Step 1 — Apply the FileSystem.cpp patch.** All ESP32-based boards require the patched `FileSystem.cpp` from this repo. See [FileSystem.cpp Patch](#filesystemcpp-patch-esp32-boards) for details on what it fixes.

**Step 2 — Verify the platformio.ini environment.** The `ttgo-lora32-v1` environment already enables the LXMF command endpoint and raises the loop-task stack to 16 KB (`HAS_GPIO_CONTROL` is the historical flag name; it now gates only the LXMF text-command endpoint, not any pin toggling):

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

**Step 3 — Build, flash, and provision.**

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

After boot, the LXMF address will appear in the serial output. Send commands from Sideband using **opportunistic** delivery — see [LXMF Remote Control](#lxmf-remote-control--general-information) for the command list.

---

## Board 5: Heltec Mesh Node T114 (nRF52840 + SX1262)

PlatformIO environment: `heltec_t114`. Tested on the 868 MHz T114 without display.

### Build & Flash

```bash
uv run pio run -e heltec_t114
# Enter the bootloader (or double-tap RESET): a port named
# usb-Heltec_AutoMation_HT-n5262_<serial>-if00 appears
stty -F /dev/serial/by-id/usb-Heltec_HT-n5262_<serial>-if00 1200
adafruit-nrfutil dfu serial -pkg .pio/build/heltec_t114/rnode_firmware_heltec_t114.zip \
  -p /dev/serial/by-id/usb-Heltec_AutoMation_HT-n5262_<serial>-if00 -b 115200 --singlebank
```

`pio run -t upload` only works when starting from the application port. It fails
if the board is already in the bootloader, or if another program keeps the serial
port open.

### Provisioning

The T114 is provisioned as a RAK4631 (product 0x10, model 0x12 for 868/915 MHz,
hwrev 1), because the nRF52 code path only accepts products 0x10, 0x11 and 0xF0.
After every flash, set the firmware hash:

```bash
HASH=$(unzip -p .pio/build/heltec_t114/rnode_firmware_heltec_t114.zip rnode_firmware_heltec_t114.bin | sha256sum | cut -d' ' -f1)
rnodeconf /dev/serial/by-id/usb-Heltec_HT-n5262_<serial>-if00 --firmware-hash $HASH
```

TNC mode example: `rnodeconf <port> --tnc --freq 869525000 --bw 250000 --sf 8 --cr 5 --txp 20`

### Preamble length (important)

Until October 2026 this firmware used an 18-symbol preamble at SF8/250 kHz
(`LORA_PREAMBLE_TARGET_MS 15` minus `LORA_PREAMBLE_SYMBOLS_HW`, clamped to
`LORA_PREAMBLE_SYMBOLS_MIN 18`). Current official RNode firmware (tested: 1.86
on a Heltec V4) transmits 24 symbols. With the receiver set to 18, the SX1262
detected the preamble but never decoded a header: the T114 transmitted normally
but never received anything from current RNodes (no replies, no delivery proofs).
Raising `LORA_PREAMBLE_SYMBOLS_MIN` to 24 fixed it. The opposite direction
(receiver 24, transmitter 18) works, so older nodes are still received.

### Battery sensing

`HAS_PMU` is enabled for the T114 and `Power.h` reads the battery divider on
P0.04 (AIN2), switching it on through `ADC_CTRL` (P0.06) only while sampling.
Voltage = raw × 3.0 V / 4096 × 4.916 (values from the Meshtastic T114 variant),
averaged over 5 samples taken once per second. The percentage follows an
approximate LiPo resting curve, so it reads high while charging and low under
load. With USB present the state is `charging`, or `charged` at 4.15 V and
above; below 2.5 V the board reports no battery. Query it remotely with the
`BATTERY` or `STATUS` LXMF commands.

## LXMF Remote Control — General Information

The firmware runs a minimal LXMF endpoint alongside the RNode TNC, allowing remote management via [Sideband](https://github.com/markqvist/Sideband) or MeshChat. When a message arrives, it parses the text command, performs the operation, and sends a reply. The node announces its LXMF address on boot, so apps can discover it automatically.

### Delivery Mode

Always use **opportunistic** delivery in Sideband. Direct delivery requires a full Reticulum Link handshake which is too resource-intensive for microcontrollers over LoRa.

### Commands

The same command set is supported on every board that has `HAS_GPIO_CONTROL` enabled (XIAO nRF52840, XIAO ESP32-S3, Heltec WSL V1, TTGO LoRa32 V1). Commands are case-insensitive; short aliases are noted in parentheses.

| Command | Example | Description |
|---------|---------|-------------|
| `STATUS` (`STATO`) | `STATUS` | One-line summary: battery, uptime, packets received/sent since boot, current RSSI (noise floor when the channel is idle). Example: `Batt 3.92V 70% discharging \| up 2d 4h 13m \| rx 1532 tx 1490 \| rssi -105 dBm` |
| `BATTERY` (`BAT`) | `BATTERY` | Battery voltage, percent, and charging state. Replies `Battery: not available` on boards without a battery-sensing branch (e.g. XIAO ESP32-S3). |
| `ANNOUNCE [min]` (`ANN`) | `ANNOUNCE 30` | Show or set both the LXMF endpoint and transport-probe announce intervals (1–1440 min). Session-only — does not persist across reboots. |
| `DUTY [ON [pct] \| OFF]` (`DC`) | `DUTY ON 1`, `DUTY OFF`, `DUTY` | Enable or disable the EU duty-cycle airtime limit, or report current state. `DUTY ON` enables with 1% long-term default (EU 868 MHz baseline); supply a percentage to override. `DUTY OFF` removes both short-term and long-term enforcement. Session-only. **Caveat:** running with `OFF` in EU regulatory territory is on you. |
| `HELP` (`?`) | `HELP` | Show command list |

Earlier revisions also exposed `SET`/`GET`/`MODE`/`PINS`/`STATUS` for direct GPIO pin toggling. Those were removed in commit `4a0df60` because they were too easy to misuse remotely — every transport node now exposes the same query/control surface. The `STATUS` name was later reused for the read-only summary above.

### Required Files

| File | Purpose |
|------|---------|
| `GPIO_Control.h` | LXMF text-command parser (STATUS, BATTERY, ANNOUNCE, DUTY, HELP) — name is historical; no GPIO pin toggling code remains |
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
| `xiao_rnode_setup.py` | All-in-one XIAO nRF52840 setup wizard (build, flash, provision, configure) |
| `flash_xiao_nrf52840.sh` | Build & flash XIAO nRF52840 firmware with optional provisioning |
| `flash_xiao_esp32s3.sh` | Build & flash XIAO ESP32-S3 firmware; software-triggered bootloader entry + EEPROM provisioning |
| `flash_wsl_v1.sh` | Build & flash Heltec WSL V1 firmware |
| `flash_ttgo_v1.sh` | Build & flash TTGO LoRa32 v1 with GPIO control |
| `patch_rnodeconf_hwsl_v1.py` | Patch rnodeconf to recognize HWSL V1 (product, models, autoinstall) |
| `provision_xiao.py` | Direct EEPROM provisioning for XIAO (KISS protocol) |

All Python scripts should be run via `uv run` to use the managed dependencies. If you prefer not to use `uv`, install dependencies manually with `pip install platformio rns adafruit-nrfutil`.

---

## Files Modified (vs upstream)

| File | Change |
|------|--------|
| `Boards.h` | Added `BOARD_XIAO_NRF52840`, `BOARD_XIAO_ESP32S3`, and `BOARD_HWSL_V1` definitions. XIAO ESP32-S3 block disables WiFi/BLE/PMU/display (LoRa-only solar profile). |
| `sx126x.h` / `sx126x.cpp` | SX1262 RX duty cycle for power optimization |
| `LowPower.h` / `LowPower.cpp` | New power management module (XIAO nRF52840 only) |
| `Bluetooth.h` | Power-optimized BLE advertising intervals (nRF52840). On XIAO ESP32-S3 the entire header is elided because `HAS_BLUETOOTH=false` and `HAS_BLE=false`. |
| `Config.h` | Low power configuration constants |
| `Device.h` | `device_init()` `bt_ready` precondition gated by `#if HAS_BLUETOOTH \|\| HAS_BLE` so boards with BT disabled (e.g. XIAO ESP32-S3 solar profile) still pass hardware validation. |
| `Utilities.h` | XIAO ESP32-S3 user LED (GPIO 21) declared active-low; LED is explicitly turned off after `pinMode` so it boots dark. |
| `RNode_Firmware.ino` | Event-driven loop with sleep support, heap diagnostics. For XIAO ESP32-S3: `esp_bt_controller_mem_release(ESP_BT_MODE_BLE)` after `Serial.begin()` returns ~30 KB to the heap; board name added to the boot banner. |
| `platformio.ini` | Added `xiao_nrf52840`, `xiao_nrf52840_lowpower`, `xiao_esp32s3`, `heltec_wsl_v1`, and `ttgo-lora32-v1` build environments. XIAO ESP32-S3 env raises loop-task stack to 16 KB (`-DARDUINO_LOOP_STACK_SIZE=16384`) for RNS recursion headroom. |
| `pyproject.toml` | Python dependencies for `uv sync` |
| `FileSystem.cpp` | Fixed double-free, cache directory, `create=true`, filename truncation, nRF52840 FS corruption guard |
| `GPIO_Control.h` | LXMF text-command endpoint — `BATTERY`, `ANNOUNCE`, `DUTY`, `HELP`. (No GPIO pin toggling code; that was removed in commit `4a0df60`. File name is historical.) |
| `LXMF_Minimal.h` | Lightweight LXMF send/receive. Sideband Telemeter battery payload builder and auto-attach-to-every-reply removed — replies are now plain text only. |
| `Power.h` | Fixed XIAO nRF52840 battery voltage divider and BQ25101 charging detection |

---

## Troubleshooting

**XIAO nRF52840 won't enter bootloader:** Double-tap the tiny reset button quickly. A USB mass-storage drive should appear.

**XIAO ESP32-S3 won't enter bootloader:** The Wio-SX1262 carrier hides the XIAO's tiny BOOT pad — the carrier's user button is **not** BOOT. Hold the XIAO module's BOOT pad while plugging in USB. Once a firmware build with `CMD_BOOTLOADER` (KISS 0x5A 0xF8) is flashed, subsequent reflashes can be triggered in software via `flash_xiao_esp32s3.sh` without touching the pad.

**XIAO ESP32-S3 boots but `[RNode] Op mode: Normal` and `RNS is inoperable because hardware is not ready!`:** Either the EEPROM provisioning is missing (run the flash script which provisions automatically), or you're on an old build where `device_init()` required `bt_ready=true` even when BLE was disabled. Fixed in `Device.h`: the `bt_ready` precondition is now gated by `#if HAS_BLUETOOTH || HAS_BLE`.

**XIAO ESP32-S3 LED stays on:** Old behavior — the LED was treated as active-high (it's actually active-low). Reflash with the current firmware; the LED is now dark in idle and only blinks on TX/RX activity.

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
