# MASN V1.2 (HT-RA62): autonomous solar Reticulum node

*[Versión en español](README.es.md)*

This adds support for the **MASN** board (*Mesh Autonomous Solar Node*). The board can run as a standalone **[Reticulum](https://reticulum.network/) transport node**: it receives, validates, routes and rebroadcasts packets over LoRa with no host attached. It can also run as a regular RNode for a host-side Reticulum instance.

## What MASN is

MASN is an open PCB for building a self-powered solar LoRa node. It was designed by Daniel P. Costas, came out of meetups at Bricolabs (A Coruña), and is based on the fakeTec design. It was originally made for Meshtastic.

- Article and build guide (Spanish): <https://danielpcostas.dev/es/masn-nodo-solar-meshtastic-que-cualquiera-puede-montar/>
- Hardware (Gerbers, STLs): <https://github.com/danielcharrua/masn-meshtastic-autonomous-solar-node>, licensed CERN OHL-S v2

This firmware targets **V1.2 with the Heltec HT-RA62 module**. The Waveshare Core1262 variant and MASN V2.0 have not been tested.

## Components

| Part | Device | Used by this firmware |
|---|---|---|
| MCU | nice!nano (nRF52840) | Runs the firmware and the microReticulum stack |
| LoRa radio | Heltec HT-RA62 (SX1262, 868 MHz), 1.8 V TCXO | LoRa interface |
| Solar charger | CN3791 MPPT | Not controlled by firmware |
| Battery | 3.7 V LiPo with BMS (e.g. 4400 mAh) | — |
| Solar panel | 5–6 V | — |
| Current/voltage monitor | INA3221 (I2C 0x40) | Battery, solar and node current on the NomadNet page and `masn_setup.py power` |
| Environment sensor | BME280 (I2C 0x76) | Detected, **not used yet** |
| Antenna | 868 MHz, UFL→SMA pigtail | — |

### Pinout (verified on hardware)

| Signal | nRF52840 | Notes |
|---|---|---|
| SX1262 NSS (CS) | P1.13 (45) | |
| SCK / MOSI / MISO | P1.11 (43) / P1.15 (47) / P0.02 (2) | SPIM2 |
| RESET | P0.09 (9) | NFC pin, works as GPIO |
| **BUSY** | **P0.29 (29)** | **Not P0.20** |
| DIO1 (IRQ) | P0.10 (10) | NFC pin, works as GPIO |
| RXEN | P0.17 (17) | TX side switched by SX1262 DIO2 |
| **External VCC** | **P0.13 (13)** | Powers the HT-RA62. **Must be driven HIGH** |
| TCXO | SX1262 DIO3, 1.8 V | |

This matches Meshtastic's `nrf52_promicro_diy_tcxo` variant ("NRF52 Pro-micro DIY").

## Build and flash

You need [PlatformIO](https://platformio.org/) and `rnodeconf` (Python package `rns`). The nice!nano needs bootloader 0.8 or newer, the same as for Meshtastic.

```sh
pio run -e masn-ht-ra62                                    # build
pio run -e masn-ht-ra62 -t upload --upload-port /dev/ttyACM0
```

`upload` does three things: flashes over USB DFU, provisions the ROM (product/model `0x53`), and writes the firmware hash. Without the hash the radio stays disabled.

- `rnodeconf` does not know model `0x53` yet, so the hash is written directly over KISS. On an already provisioned board, `rnodeconf --rom` prints a harmless `KeyError: 83`.
- Provisioning a blank, never-provisioned board has **not been verified** yet.
- If you flash by other means, write the hash afterwards with `python masn/masn_setup.py hash`.

After flashing, run `python masn/masn_setup.py` (see below).

> If the hash does not change after a code change, PlatformIO reused a stale `.zip`. Delete it with `rm .pio/build/masn-ht-ra62/*.{hex,zip,bin}` and rebuild.

## Configuration: `masn_setup.py`

`masn/masn_setup.py` is the configuration program for a flashed MASN. It talks to the board over USB using the RNode KISS protocol and the firmware's Provisioning subsystem, so it needs no other tool. `rnodeconf --tnc` and `--firmware-hash` abort on the MASN model (`0x53`); this program replaces them.

```sh
pip install pyserial rns lxmf          # lxmf is only needed for map visibility
python masn/masn_setup.py              # interactive wizard
```

The wizard finds the nice!nano on USB, waits for Reticulum to start on the board, and asks, step by step, offering the current value as the default (Enter keeps it, `-` clears an optional text):

1. **Radio.** Shows the defaults (869.618 MHz, 62.5 kHz, SF5, CR4/5, 7 dBm) and asks whether to change them. They must match the rest of your network. It warns about EU868 limits: 500 mW e.r.p. at a 10 % duty cycle in 869.40–869.65 MHz, and usually 25 mW at 1 % elsewhere.
2. **Mode.** Standalone Reticulum transport node (the configuration is saved and the board starts as a node at power-up, with no USB needed) or RNode for a host computer.
3. **Name and NomadNet page.** The node name, and whether to publish a NomadNet page. If yes, it asks for a one-line description and an operator contact.
4. **Map visibility ([rmap.world](https://rmap.world)).** If yes, it asks for the location as one line, `latitude, longitude` in decimal degrees with a dot (it can be pasted from a map, e.g. `42.8125, -1.6458`), then the altitude, whether to also show the location on the NomadNet page, an optional operator LXMF address, and the announce interval (default 6 h; rmap flags nodes announcing more than 3 times per hour). The NomadNet site has its own identity (`NOMADNET_OWN_IDENTITY`), separate from the node identity, so rmap shows one marker per node (the LoRa interface) and lists the site only under *Services*. rmap removes nodes that have not announced for 7 days.
5. **Summary and confirmation.** It writes everything, reboots the board, checks that it comes back, and prints the node's NomadNet address and transport identity. It saves the answers to a profile (`masn-<id>.json`); `--profile file.json` reuses them as defaults, for example to set up several nodes.

Sub-commands for scripting:

| Command | What it does |
|---|---|
| `status` | Board status (diagnostic `0x60`): `ONLINE`, `HW`, frequency, power, pins |
| `show` | Stored node configuration (name, page, location, discovery) and the node's addresses |
| `hash` | Writes the hash of `.pio/build/masn-ht-ra62/*.bin` (only if you flashed without `pio … -t upload`) |
| `tnc` / `rnode` | Switch mode, with radio options `--freq --bw --sf --cr --txp` |
| `power` | Battery, solar panel and node current from the INA3221 (diagnostic `0x6C`) |

Common options: `--port` (default: auto-detect the nice!nano) and `--profile`. The interface language follows `LANG` (Spanish or English).

To watch the node working, open the port as a serial console (`pio device monitor -b 115200` or `screen /dev/ttyACM0 115200`). Look for `Valid announce for …` and `Rebroadcasting announce …`. Close the console before running `masn_setup.py` again, because only one program can use the port at a time.

### What the node publishes

- **NomadNet page** (`nomadnetwork.node`): the name, description, contact, location (if enabled), radio parameters and uptime, followed by the firmware's statistics pages. The site is announced at boot and then on every announce interval.
- **Interface discovery** (`rnstransport.discovery.interface`, only if visible): the same announce `rnsd` sends for an interface with `discoverable = yes`. It carries the name, location, radio parameters and the proof-of-work stamp maps require. The stamp is computed by `masn_setup.py` on the computer (it would take minutes on the nRF52) and stored on the board, which re-sends it 2 minutes after boot and then on every interval. A LoRa-only node reaches the map only if some transport node bridges your LoRa network to the Internet.

## Manual configuration

You do not need `masn_setup.py`. There are two ways to set things up by hand.

### Option A: standalone node, with raw KISS commands

The board takes standard RNode KISS frames over USB at 115200 baud: `C0 <command> <data> C0`, where `C0` and `DB` in the data are escaped as `DB DC` and `DB DD`. Mind two details:

- Every command needs at least one data byte. `CMD_CONF_SAVE` is sent as `53 00`; without the `00` the firmware ignores it.
- Keep reading (or flushing) the port while it is open. If the host leaves it open without reading, the USB buffer fills, the firmware blocks writing its log, and the watchdog reboots the board after 60 s.

| Command | Data | Meaning |
|---|---|---|
| `01` | 4 bytes, big endian | Frequency in Hz |
| `02` | 4 bytes, big endian | Bandwidth in Hz |
| `03` | 1 byte | TX power in dBm |
| `04` | 1 byte | Spreading factor |
| `05` | 1 byte | Coding rate (5–8) |
| `06` | `01` | Radio on |
| `53` | `00` | Save the radio configuration: the board boots as a transport node |
| `54` | `00` | Delete it: the board boots as an RNode |
| `55` | `F8` | Reboot |
| `86` | msgpack | Provisioning request (`[op, seq, payload]`; SetState = 5, Commit = 6) |

Name, NomadNet page and location are Provisioning fields of namespace `100`:

| Id | Field | Type |
|---|---|---|
| 4 | NomadNet Enabled | bool |
| 5 | NomadNet Name | text, ≤ 63 |
| 20 | Site Description | text, ≤ 160 |
| 21 | Site Contact | text, ≤ 64 |
| 22 / 23 / 24 | Latitude / Longitude / Altitude | float |
| 25 | Show Location (on the NomadNet page) | bool |
| 26 | Discovery Enabled | bool |
| 27 | Discovery Data (announce with its stamp, see `build_discovery_app_data()` in `masn_setup.py`) | bytes |
| 28 | Announce Interval (minutes, 30–1440) | int |

Example (Python with `pyserial` and `rns`), tested on hardware:

```python
import serial, struct, time
from RNS.vendor import umsgpack

s = serial.Serial("/dev/ttyACM0", 115200, timeout=0.1)

def kiss(cmd, data):
    data = data.replace(b"\xdb", b"\xdb\xdd").replace(b"\xc0", b"\xdb\xdc")
    s.write(b"\xc0" + bytes([cmd]) + data + b"\xc0")
    time.sleep(1)
    s.reset_input_buffer()              # keep the port drained

# 1. Name and NomadNet page: Provisioning SetState + Commit (namespace 100)
kiss(0x86, umsgpack.packb([5, 1, {3: {100: {5: "My MASN node",
                                            20: "Solar node on the hill",
                                            21: "LXMF: <your address>"}}}]))
kiss(0x86, umsgpack.packb([6, 2, {}]))

# 2. Radio, save as standalone node, reboot
kiss(0x01, struct.pack(">I", 869618000))   # frequency (Hz)
kiss(0x02, struct.pack(">I", 62500))       # bandwidth (Hz)
kiss(0x03, bytes([7]))                     # TX power (dBm)
kiss(0x04, bytes([5]))                     # spreading factor
kiss(0x05, bytes([5]))                     # coding rate
kiss(0x06, b"\x01")                        # radio on
kiss(0x53, b"\x00")                        # save: boot as transport node
kiss(0x55, b"\xf8")                        # reboot
```

Provisioning answers only once Reticulum has started on the board, up to about a minute after boot. The RNode Console web app (`Release/console.html`, see [docs/Provisioning.md](../docs/Provisioning.md)) speaks the same protocol and shows the same fields under *Transport Config → General*; it has not been tested with the MASN.

### Option B: as an RNode for Reticulum on a computer

Put the board in RNode mode (`masn_setup.py rnode`, or command `54 00` + reboot) and add it to `~/.reticulum/config` like any RNode. Tested with `rnsd` 1.5.7:

```ini
[interfaces]
  [[MASN]]
    type = RNodeInterface
    enabled = yes
    port = /dev/ttyACM0
    frequency = 869618000
    bandwidth = 62500
    txpower = 7
    spreadingfactor = 5
    codingrate = 5
```

The computer then does the routing (`enable_transport = yes` under `[reticulum]`). If that computer is also on the Internet, it is the simplest way onto [rmap.world](https://rmap.world): add to the interface `discoverable = yes`, `discovery_name = …`, `latitude = …`, `longitude = …`, `height = …` and `announce_interval = 360`, and install `lxmf` (`pip install lxmf`). This last part (`discoverable`) is from the RNS and rmap documentation and has not been tested with the MASN.

## Verification tools

| Script | Purpose |
|---|---|
| `masn_osc_test.py` | Step-by-step SX1262 bring-up: SPI write, reset, TCXO, STDBY_XOSC and PLL. No RF emitted. |
| `masn_sdr_txtest.py` | Sends a CW carrier and a LoRa packet and measures them with an RTL-SDR (needs `rtl_sdr` and `numpy`). |
| `masn_rxmon.py` | Receive monitor: RSSI, SNR and the Reticulum header of every packet. |

They need the board in RNode mode (`masn_setup.py rnode`). The commands that transmit only run at 869.618 MHz and ≤ 7 dBm.

Expected `masn_osc_test.py 1v8` result: `XOSC … mode=STDBY_XOSC errors=OK` and `FS … mode=FS errors=OK`. The `XOSC_START` shown at `POR`/`END` is the power-on error from before the TCXO is configured, and is normal.

The firmware diagnostic suite (`MASN_Diagnostics.h`) answers `CMD_DETECT` (`0x08`) sub-commands `0x60`–`0x6F`:

| Code | What it does |
|---|---|
| `0x60` | Configuration |
| `0x61` / `0x62` | SX1262 status, IRQ, errors and counters; `0x62` also gives the radio watchdog counters (`RECOV`, `RECOV_PT`, `RECOV_AGO`) and the uptime (`UP`, seconds) |
| `0x63` / `0x64` | 600 ms CW (`0x64` with RXEN low) |
| `0x65` | LoRa test packet |
| `0x66` | Clear errors and IRQ |
| `0x67` | RF registers |
| `0x68`–`0x6A` | Oscillator trace (1.8 V TCXO / 3.3 V TCXO / crystal) |
| `0x6B` | Find the GPIO that follows BUSY |
| `0x6C` | Supply rails, I2C scan and raw INA3221 channels |
| `0x6D` | State of the identity/configuration backup |
| `0x6E` | **Formats the filesystem and reboots**, to test the restore from the backup |
| `0x6F` | Resets the SX1262 through its reset pin, to test the radio watchdog |

## Problems solved while porting

In case you hit the same symptoms on another board:

1. **BUSY on the wrong pin (P0.20 instead of P0.29).** Commands were sent while the SX1262 was busy and the chip dropped them silently. The TCXO appeared not to start and TX failed intermittently.
2. **P0.13 left undriven.** The VCC MOSFET only half-conducted: enough for RX, but the SX1262 browned out (full power-on reset) as soon as the PA turned on. It is driven in `sx126x::preInit()`. Putting it in the variant's `initVariant()` does not work, because that symbol is never linked.
3. **TCXO:** DIO3 must be configured before calibration.
4. **Memory:** the RNS pool needs 128 KB (`RNS_HEAP_POOL_BUFFER_SIZE=131072`). With 64 KB, RNS starts with `bad_alloc` and drops every packet (`No identity!`).
5. **microReticulum deadlock (a known library issue, not specific to this board):** serving a NomadNet page over a *Link* makes the *Resource* retries block `Transport::jobs()`, and the watchdog reboots the board after 60 s. A fix has been submitted to the microReticulum repository.

## Known limitations

- **Paths are kept in RAM.** Paths, known destinations and the packet hashlist are not written to flash (`RNS_PERSIST_*=0`), because that meant a flash write for almost every received announce. After a reboot the node relearns them from announces. The path table holds 25 destinations (`URTN_PATH_TABLE_MAX_RECS`).
- **Power cuts and the filesystem.** A power cut while LittleFS is writing can leave it unmountable; the Adafruit core then formats it, which used to erase the provisioning, the configuration and the node identity. The firmware now keeps a copy of those files in two flash pages outside the filesystem (`MASN_Backup.h`, refreshed 90 s after boot and every 10 minutes when something changed) and restores them at boot. Changes made in the last 10 minutes before a cut can still be lost.
- **Radio watchdog.** After several days outdoors a node was found deaf: the SX1262 sat in standby and nothing put it back into RX. The firmware now checks the chip every 5 s and re-runs the radio setup if it lost its configuration (packet type no longer LoRa, i.e. the chip was reset) or stays in standby for 15 s. `RECOV_PT` in diagnostic `0x62` records which case it was (0 = chip reset, 1 = left in standby). The root cause (most likely a supply glitch on the radio) is not confirmed.
- **Provisioning changes can be lost on reboot (intermittent).** A provisioning commit followed shortly by a radio configuration save and a reboot has been seen to come back with the previous values. `masn_setup.py` writes the radio first, commits last, and re-reads and re-writes the configuration after the reboot until it sticks. If you configure by hand, check with `masn_setup.py show` after rebooting.
- **A USB host that keeps the port open without reading it reboots the node.** The firmware logs over USB; once the buffer is full it blocks, and the 60 s watchdog resets the board. Read or close the port; `masn_setup.py` does both. A node with nothing attached to USB is not affected.
- Provisioning a blank board, MASN V2.0 and the Core1262 variant are not verified.

## To do

- Read the environment sensor (BME280) and publish battery and environment as telemetry.
- Test MASN V2.0 and the Core1262 variant.
- Add model `0x53` to `rnodeconf`, so that `--autoinstall` works.
- Measure power draw in node mode and tune power management.
