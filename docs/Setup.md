# Setup Guide — WiFi and TCP client on microReticulum_Firmware

How to take a freshly flashed board (tested on a Heltec WiFi LoRa 32 V4) to a
working Reticulum transport node that joins your WiFi and dials out to a
Reticulum TCP server. Radio parameters and TNC mode are covered because the
TCP interface only forwards traffic once the board is in TNC mode.

Everything here is done over the USB cable. Two tools are involved:

| Tool | What it configures | Where it stores it |
|---|---|---|
| `rnodeconf` (from the `rns` Python package, **2.5.0 or newer**) | Firmware hash, WiFi mode/SSID/PSK, radio parameters, TNC mode | EEPROM (legacy RNode opcodes) |
| RNode Console (`webconsole/index.html`) | Everything rnodeconf can, plus the Provisioning fields: **TCP Mode / Host / Port**, interface modes, NomadNet | Provisioning store on the board's filesystem |

The TCP target can only be set with the Console (or baked in at build time).
Nothing else speaks the Provisioning protocol yet.

## 0. Prerequisites

- Firmware built with `-DTCP_TRANSPORT` (the `heltec_wifi_lora_32_V4` env has it).
- `rns` 1.5.x or newer on the host, so that `rnodeconf` knows the Heltec V4
  model. Check with:

  ```bash
  rnodeconf --version      # 2.5.0 or newer
  ```

  Older rnodeconf (2.4.0 and earlier) has no entry for model `0xC8`. The
  symptoms are `KeyError: 200` during the flash script's post-upload step and
  `[init] Error, device init failed` on every boot. If you must keep an older
  `rns` system-wide, put a newer one in a venv and use that `rnodeconf`.
- Chrome or Edge for the Console. It needs the browser's Web Serial API;
  Safari does not have it. Brave works if Web Serial is allowed for the page.

## 1. Flash

```bash
pio run -e heltec_wifi_lora_32_V4 -t upload --upload-port /dev/cu.usbmodemXXXX
```

The flash script's post-upload step provisions the EEPROM and writes the
firmware hash through `rnodeconf`. With rnodeconf 2.5.0+ this just works.
Confirm on the next boot's serial output:

```
[init] hw_ready: 1
```

If it says `hw_ready: 0` / `device init failed`, write the hash by hand:

```bash
H=$(python3 -c "import hashlib;print(hashlib.sha256(open('.pio/build/heltec_wifi_lora_32_V4/rnode_firmware_heltec32v4pa.bin','rb').read()[:-32]).hexdigest())")
rnodeconf --firmware-hash "$H" /dev/cu.usbmodemXXXX
```

`hw_ready` matters: the firmware refuses to enter TNC mode without it.

## 2. WiFi (station mode)

Either tool works. Settings persist in EEPROM and survive reflashing.

**rnodeconf**

```bash
rnodeconf --wifi STATION --ssid 'YourNetwork' --psk 'YourPassword' /dev/cu.usbmodemXXXX
```

`--wifi` accepts `OFF`, `STATION`, or `AP`. Optional: `--channel`, `--ip`, `--nm`
for a static address; `--ip NONE` returns to DHCP.

**Console:** Node Config tab → WiFi → set Mode = Station, SSID, PSK → **Save WiFi**.
The SSID and PSK fields are write-only on the wire, so they read back blank.

The board applies WiFi immediately and reconnects on its own after reboots.

## 3. TCP client target (Console)

1. Open `webconsole/index.html` in Chrome/Edge. The page comes up on the
   **Settings** tab; only Settings and Node Status are shown until you connect.
2. Top bar: Transport = **Serial**, click **Connect**, pick the board's port in
   the browser's picker. Opening the port resets the board; leave
   `auto-reconnect` checked and click Connect again if the first attempt
   times out while it boots.
3. After connecting, the **Node Config**, **Transport Config** and **Logs**
   tabs appear. Open **Transport Config**.
4. In the left sidenav pick **RNode Network Config**. Set:
   - **TCP Mode** — `client` (dial out) or `server` (listen for inbound peers).
   - **TCP Host** — hostname or IP of the Reticulum `TCPServerInterface`
     (client mode only; leave empty to keep the interface idle).
   - **TCP Port** — the server's port; 4242 is the Reticulum convention.
5. Click **Save namespace**. This stages a draft on the board.
6. Click **Commit all** in the toolbar at the top of the tab. These fields are
   reboot-required, so a banner appears: click **Reboot now**.
7. Optional, applies live without reboot: sidenav **RNode General Config** →
   **TCP Interface Mode** (`gateway` default, or `full`, `boundary`, …).

Verify after the reboot on the serial log or the Console's **Logs** tab:

```
Registering TCP Interface...
TCPInterface: connecting to host (1.2.3.4):4242
TCPInterface: connected to host:4242
```

Live counters are under Transport Config → **RNode General Metrics** →
Interfaces → TCPInterface (`tcp_peers`, `tcp_rx_frames`, `tcp_tx_frames`).

### Baking a default into the build instead

A build can carry a default target, useful for fleets or bench boards. It is
still overridable from the Console:

```bash
PLATFORMIO_BUILD_FLAGS='-DTCP_HOST=\"rns.example.org\" -DTCP_PORT=4242' \
  pio run -e heltec_wifi_lora_32_V4 -t upload --upload-port /dev/cu.usbmodemXXXX
```

## 4. Radio parameters and TNC mode

The TCP interface is registered as soon as WiFi is on, but Reticulum
**transport** (forwarding between LoRa and TCP) is only enabled when the
board boots in TNC mode. TNC mode needs `hw_ready` (step 1) and a saved radio
configuration.

**rnodeconf**

```bash
rnodeconf -T --freq 914900000 --bw 125000 --sf 7 --cr 5 /dev/cu.usbmodemXXXX
```

rnodeconf prompts for TX power when the requested value is above 17 dBm;
type it at the prompt. The Heltec V4 accepts up to 28 dBm (its PA ceiling);
anything higher is clamped.

**Console:** Node Config tab → Radio → set the operating mode selector to
**TNC** and the radio fields → **Save Radio** → reboot from the banner.

Confirm on boot:

```
[init] op_mode: 18            (18 = TNC)
RNS transport mode is ENABLED
```

## 5. Server side

The board's client expects a plain Reticulum `TCPServerInterface`, for
example in the server's `~/.reticulum/config`:

```
[[TCP Server]]
  type = TCPServerInterface
  enabled = yes
  listen_ip = 0.0.0.0
  listen_port = 4242
```

Framing is the HDLC-style framing Python RNS uses, so no special server is
needed. An empty two-flag keepalive frame is sent every 30 s by the board.

## Troubleshooting

- **"Selected device is not an RNode" / detect fails in a web tool.** Usually
  `hw_ready: 0` from a missing firmware hash (step 1), or another program
  holding the serial port. Only one process can own the port; close other
  Console tabs, monitors, and `rnodeconf` before connecting.
- **`TCPInterface: DNS failed for …`** — the hostname does not resolve. Check
  it from the host with `dig`. Typos in a build-time `TCP_HOST` are the
  common cause.
- **Interface never registers.** WiFi mode is Off. Set Station or AP (step 2);
  the interface is created only when WiFi is enabled.
- **Connects but transport disabled.** The board is not in TNC mode (step 4).
- **Reconnect timing.** After a server restart the board retries at 10 s,
  then doubles the interval up to 2 min. It caches the resolved IP and falls
  back to DNS only when the cached address fails.
