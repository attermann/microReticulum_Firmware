# microReticulum_Firmware

Fork of RNode_Firmware with integration of the [microReticulum](https://github.com/attermann/microReticulum) Network Stack to implement a completeley self-contained standalone Reticulum node.

## Installation

This firmware can be easily installed on devices in the same way as RNode using the new `fw-url` switch to `rnodeconf` which allows firmware images to be pulled from an alternate repository. RNS may need to be updated to the latest version to use this new switch.

The latest version of this firmware can be installed in the usual RNode way with the following command:
```
rnodeconf --autoinstall --fw-url https://github.com/attermann/microReticulum_Firmware/releases/
```

NOTE: If re-installing a new build of the same version installed previously, be sure to clear the rnodeconf cache first to force it to download the very latest.
```
rnodeconf --clear-cache
```

## Enabling Transport Mode

By default this firmware will operate just like any other RNode firmware allowing it to be used as just a radio by RNS installed on an attached machine.

To enable `Transport Mode` using the RNS embedded on the device, the device must be switched to TNC mode using a command like the following:
```
rnodeconf --tnc --freq 915000000 --bw 125000 --sf 8 --cr 5 --txp 17 /dev/ttyACM0
```
When in `Transport Mode`, the device will display "TRANSPORT" across the top of the AirTime panel of the display to indicate that the embedded RNS is active and routing packets.

Note that at the present time, when in TNC mode this firmware does not operate like a regular RNode does when in TNC mode due to logging from the embedded RNS that is output on the serial port. This can clobber KISS communication from the attached machine so do not attempt to attach another RNS to the device while in this mode. On the plus side, there is extensive logging available on the serial port to observe the embedded RNS in action and to aid in troubleshooting.

## Supported Hardware

### Seeed XIAO nRF52840 + Wio-SX1262

The [Seeed XIAO nRF52840](https://wiki.seeedstudio.com/XIAO_BLE/) paired with the [Wio-SX1262 expansion board](https://wiki.seeedstudio.com/wio_sx1262_with_xiao_nrf52840_kit/) is a compact, low-power LoRa node well suited for deployment as a standalone Reticulum transport/repeater.

**Pin mapping (do not modify — consumed by radio):**

| Function | Arduino pin | nRF52840 GPIO |
|----------|-------------|---------------|
| RF switch (RXEN) | D5 | P0.05 |
| Radio reset | D2 | P0.28 |
| SPI CS | D4 | P0.04 |
| SPI SCK | D8 | P1.13 |
| SPI MOSI | D10 | P1.15 |
| SPI MISO | D9 | P1.14 |
| BUSY | D3 | P0.29 |
| DIO1 / IRQ | D1 | P0.03 |

Free GPIO for external use: D0, D6, D7.

**PlatformIO environments:**

| Environment | Description |
|-------------|-------------|
| `xiao_nrf52840` | Standard build — continuous RX, ~10 mA |
| `xiao_nrf52840_lowpower` | Low-power build — 7 ms RX / 50 ms sleep, ~500 µA (requires ≥32 symbol preamble on transmitters) |

**Build and flash:**

The Seeed platform is downloaded automatically on the first build.
```
pio run -e xiao_nrf52840
```

Flashing uses the nRF52 UF2 DFU bootloader. Double-tap the reset button on the XIAO to enter bootloader mode (the LED pulses slowly), then upload:
```
pio run -e xiao_nrf52840 -t upload
```
The post-upload script automatically provisions the EEPROM and writes the firmware hash via `rnodeconf`.

**Configuration:**

For standalone TNC / transport mode:
```
rnodeconf /dev/ttyACM0 --tnc --freq 869525000 --bw 125000 --sf 8 --cr 5 --txp 14
```

For host-controlled mode (attached RNS / meshchat):
```
rnodeconf /dev/ttyACM0 --normal
```

Verify with:
```
rnodeconf /dev/ttyACM0 --info
```

The onboard red LED indicates radio state: briefly lit on RX/TX activity in HOST mode; lit while the channel is sensed busy (CSMA) in TNC mode.

**Flash storage and RNS persistence:**

The nRF52840 has 1 MB of internal flash, but the Adafruit nRF52 BSP hard-codes the LittleFS filesystem region (`InternalFileSystem`) to only 28 KB. This severely limits how many RNS paths and announces microReticulum can persist across reboots.

This fork works around the limitation with a pre-build patcher in `extra_script.py`. At every build, `patch_internalfs_for_xiao()` locates `InternalFileSystem.cpp` inside the PlatformIO package cache and rewrites two `#define`s to enlarge the region to 128 KB (32 × 4096-byte pages):

```
LFS_FLASH_ADDR:       0xED000 → 0xD4000
LFS_FLASH_TOTAL_SIZE: 7 pages → 32 pages  (28 KB → 128 KB)
```

The resulting flash layout (with S140 v7.3.0 SoftDevice):

| Region | Address range | Size |
|---|---|---|
| MBR | `0x00000`–`0x01000` | 4 KB |
| SoftDevice S140 | `0x01000`–`0x27000` | 152 KB |
| Application | `0x27000`–`0xD4000` | 692 KB |
| LittleFS (InternalFS) | `0xD4000`–`0xF4000` | **128 KB** |
| Bootloader | `0xF4000`–`0x100000` | 48 KB |

The patch is idempotent — it skips cleanly on subsequent builds once applied, backs up the original file to `InternalFileSystem.cpp.xiao.orig`, and raises an error if the BSP changes in a way that breaks the regex so the failure is explicit rather than silent. The region size is configurable via `-DXIAO_INTERNALFS_PAGES=N` (supported values: 7, 16, 32, 64).

With 128 KB available, the path table cap is raised to 100 entries (`-DRNS_PATH_TABLE_MAX=100`) — the nRF52840 boots using ~42 KB of its 237 KB RAM, leaving ample headroom. The firmware prints the mounted filesystem size at INFO level on boot:

```
Total flash: 131072 bytes
Free flash:  130048 bytes
```

Connect a serial monitor after flashing to confirm the new region is in use.

**Notes:**
- The Seeed bootloader does not write the image size at the Adafruit nRF52 `IMG_SIZE_START` address, so `VALIDATE_FIRMWARE` is disabled for this board to prevent the firmware hash check from failing.
- No dedicated `rnodeconf` product/model ID exists for the XIAO nRF52840 yet; the RAK4631 bytes (`0x10`/`0x12`) are reused during provisioning.
- PlatformIO's USB HWID auto-detection crashes on the Seeed platform (the XIAO enumerates as `0x2886:0x0045`, which is absent from the board JSON's hwid list). `upload_port = /dev/ttyACM0` is pinned in the env to bypass this; the board's `use_1200bps_touch` setting still triggers DFU automatically on upload.

## Build Dependencies

Build environment is configured for use in [VSCode](https://code.visualstudio.com/) and [PlatformIO](https://platformio.org/).

## Building from Source

Building and uploading to hardware is simple through the VSCode PlatformIO IDE
- Install VSCode and PlatformIO
- Clone this repo
- Launch PlatformIO and load repo
- In PlatformIO, select the environment for intended board
- Build, Upload, and Monitor to observe application logging

Uploading to devices requires access to the `rnodeconf` utility included in the official [Reticulum](https://github.com/markqvist/Reticulum) distribution to update the device firmware hash. Without this step the device will report invalid firmware and will fail to fully initialize.

Instructions for command line builds and packaging for firmware distribution.

## Build Options

- `-DHAS_RNS` Used to enable the microReticulum RNS stack and transport node.
- `-DUDP_TRANSPORT` Used to enable WiFi connection (when configured through `rnodeconf` as an additional transport medium (currently hard-coded to use port 4242).
- `-DXIAO_INTERNALFS_PAGES=N` *(XIAO nRF52840 only)* Sets the number of 4096-byte pages reserved for the LittleFS region. Supported values: `7` (28 KB, BSP default), `16` (64 KB), `32` (128 KB, default in this fork), `64` (256 KB). Larger values trade application flash for persistence capacity.
- `-DURTN_PATH_TABLE_MAX_RECS=N` Sets the maximum number of RNS path table entries held in RAM and persisted to flash (default: `100`). Passed directly to `RNS::Transport::path_table_maxsize()` at startup.

## PlatformIO Command Line

Clean all environments (boards):
```
pio run -t clean
```

Full Clean (including libdeps) all environments (boards):
```
pio run -t fullclean
```

Build a single environment (board):
```
pio run -e ttgo-t-beam
pio run -e wiscore_rak4631
pio run -e xiao_nrf52840
```

Build and upload a single environment (board):
```
pio run -e ttgo-t-beam -t upload
pio run -e wiscore_rak4631 -t upload
pio run -e xiao_nrf52840 -t upload
```

Build and package a single environment (board):
```
pio run -e ttgo-t-beam -t package
pio run -e wiscore_rak4631 -t package
pio run -e xiao_nrf52840 -t package
```

Build all environments (boards):
```
pio run
```

Build and package all environments (boards):
```
pio run -t package
```

Write version info:
  python release_hashes.py > Release/release.json

## Firmware Release

New firmware release procedure:

  1. Ensure that microReticulum repo is updated for build (and package versioning is incremented if changed)

  2. Shutdown microReticulum_Firmware project in IDE (if open)

  3. Clean build directory
     ```
     pio run -t fullclean
     ```

  4. Clean release directory
     ```
     rm Release/release.json
     ```

  5. Build new releases
     ```
     pio run -t package
     ```

  6. Upload all files (except README.md and esptool) to github release

## Roadmap

- [ ] Extend KISS interface to support config/control of the integrated microReticulum stack
- [ ] Add interface for easy customization of firmware
- [ ] Option 2: use XIAO nRF52840 onboard 2 MB QSPI flash for even larger RNS persistence (path table, announce cache)
- [x] Enlarge XIAO nRF52840 InternalFS from 28 KB to 128 KB via pre-build BSP patcher; raise path table cap to 100 entries
- [x] Add power management and sleep states to extend battery runtime (XIAO nRF52840: PERFORMANCE / BALANCED / LOW_POWER modes)
- [x] Add build targets for NRF52 boards
- [x] Add build target for Seeed XIAO nRF52840 + Wio-SX1262

Please open an Issue if you have trouble building ior using the API, and feel free to start a new Discussion for anything else.

