# Provisioning Guide — RNode Console

This guide walks through using the **RNode Console** (the single-page web app shipped as `Release/console.html`) to provision and manage microReticulum nodes — both nodes sitting in front of you and nodes several LoRa hops away. It assumes you have already flashed the firmware and have a working node; see the main [README](../README.md) for installation.

> **Audience.** Operators of single nodes and small fleets. If you want the protocol-level view of the Provisioning subsystem, read `Provisioning.h` / `Provisioning.cpp` and the upstream [microReticulum Provisioning headers](https://github.com/attermann/microReticulum).

## Contents

- [Concepts](#concepts)
- [Opening the Console](#opening-the-console)
- [Local provisioning](#local-provisioning)
- [Remote provisioning (over RNS / LoRa)](#remote-provisioning-over-rns--lora)
- [Transports in detail](#transports-in-detail)
  - [Serial (USB)](#serial-usb)
  - [Bluetooth (BLE)](#bluetooth-ble)
  - [WebSocket (WiFi)](#websocket-wifi)
  - [RNS via MeshChatX](#rns-via-meshchatx)
- [Console tabs in detail](#console-tabs-in-detail)
- [Caveats and pitfalls](#caveats-and-pitfalls)
  - [RNS over LoRa](#rns-over-lora)
  - [Security and exposure](#security-and-exposure)
  - [WiFi and AP mode](#wifi-and-ap-mode)
  - [Reboot-required fields](#reboot-required-fields)
- [Troubleshooting](#troubleshooting)

## Concepts

The Console manipulates a node through **three layers**:

1. **Legacy RNode KISS opcodes** — the original `rnodeconf` protocol (radio params, WiFi, Bluetooth, display, raw EEPROM). Surfaced as the **Node Config** tab. Most of these are *set-only* on the wire (no read counterpart), so the Console marks them with a `set-only` badge.
2. **Provisioning subsystem** — a typed, namespaced configuration engine running inside the embedded microReticulum stack. Each field has a declared type, flags (`LIVE_APPLY`, `REBOOT_REQUIRED`, `READ_ONLY`, `WRITE_ONLY`, `SECRET`), and a setter/getter. Drafts are staged with `SetState` and applied with `Commit` (or thrown away with `Discard`). This is the **Transport Config** tab.
3. **Live metrics** — read-only fields exposed through the same Provisioning namespace tree (radio link, channel utilisation, RNS destination hashes, WiFi info). These drive the **Node Status** tab together with KISS `CMD_STAT_*` frames.

A node's identity in the RNS sense is a long-lived Ed25519 keypair generated on first boot. The Console never asks you to manage that directly — it reads identity hashes through the Metrics namespace and uses them when establishing remote links.

## Opening the Console

The Console is a static HTML file. Open `Release/console.html` from disk (drag-and-drop into Chrome / Edge / latest Firefox), serve it from any static web server, or — when the device is in console mode — load it directly from the node's HTTP endpoint at `http://10.0.0.1/`.

The Console only depends on the browser's Web Serial, Web Bluetooth, and WebSocket APIs. Chrome and Chromium-derived Edge are the supported browsers together with the latest versions of Firefox; older Firefox versions as well as Safari lack Web Serial / Web Bluetooth and can only use the WebSocket and RNS transports.

> NOTE: Chrome's "Local Network Access Checks" feature may need to be temporarily disabled at `chrome://flags/#local-network-access-check` to connect to a node on the LAN, depending on your Chrome version.

## Local provisioning

"Local" means you have a direct wire / radio link to the node: USB, BLE, or the same LAN. The recommended bring-up sequence for a fresh node:

1. **Connect over Serial** (USB cable to the node).
2. In the topbar, select `Transport: Serial` and click **Connect**. Pick the port for the device when your browser prompts.
3. Open the **Node Config** tab.
   - **Device Info** sub-tab: confirm board, platform, MCU, and firmware version.
   - **Radio** sub-tab: set frequency, bandwidth, spreading factor, coding rate, TX power. These are committed to EEPROM and only take effect on the *next* reboot when the device is in boot-into-TNC mode.
   - **WiFi** sub-tab (ESP32 boards only): set SSID / PSK / mode if you plan to connect over WebSocket later. These fields are write-only.
   - **Bluetooth** sub-tab: pair / unpair, set PIN if you plan to use the BLE transport.
   - **Display** sub-tab: intensity, blanking, rotation, NeoPixel intensity.
4. Open the **Transport Config** tab.
   - **General** namespace: turn on `kiss_framed_logs` if you want log frames in KISS format. Set `nomadnet_name` and toggle `nomadnet_enabled` if NomadNet stats pages are wanted.
   - **Reticulum / Transport** namespaces (provided by microReticulum itself): identity, transport-enabled state, remote management allow-list, etc.
   - **Interfaces** sub-namespaces: pick the LoRa / UDP interface modes (`gateway`, `full`, `point-to-point`, `access-point`, `roaming`, `boundary`).
5. Click **Save** on each namespace that has edits. The toolbar shows a draft indicator. If any field had the `REBOOT_REQUIRED` flag, a banner appears at the top of the Console: click **Reboot now**.
6. After the reboot, the Console auto-reconnects (if "auto-reconnect" is checked) and you can confirm the new state on **Node Status**.

For everyday operations (looking at link quality, kicking the node, changing the NomadNet site name), Serial is overkill — switch to **Bluetooth** or **WebSocket** to skip the cable.

## Remote provisioning (over RNS / LoRa)

The same Provisioning protocol travels over a Reticulum Link, so any node reachable on your mesh can be configured from a workstation that has Reticulum installed.

### How it works

The firmware registers a destination on the aspect `rnstransport.remote.management`. When `RNS::Reticulum::remote_management_enabled(true)` is set (the default on this firmware), incoming Links to that destination route Provisioning frames into the Provisioner, and replies come back over the same Link. The microReticulum stack handles path discovery, Link establishment, and identification.

The RNode Console does **not** itself speak RNS — your browser can't open a Reticulum Link directly. Instead it tunnels Provisioning frames through a **MeshChatX** instance running on your workstation, which acts as the RNS endpoint.

### Setup

1. Install and run [MeshChatX](https://meshchatx.com/) on your workstation. Make sure it has at least one RNS interface configured that can reach the target node (typically a `TCPClientInterface` to your local rnsd, plus whatever LoRa / packet-radio interfaces sit between you and the node).
2. MeshChatX exposes a WebSocket bus (default `ws://localhost:8000/ws` — confirm the path in your MeshChatX install) with a generic `rns.link.*` API. The Console uses this bus to ask MeshChatX to open a Link, send frames over it, and receive replies.
3. **Obtain the destination hash of the remote node.** When a node announces, its `rnstransport.remote.management` destination shows up in Reticulum's path table. Grab the 16-byte hex hash with `rnsd` / `rnpath` / `rnstatus`, or read it off the local Console's **Transport Config → Metrics → Destinations → mgmt_destination** field.
4. In the Console topbar, select `Transport: RNS (via MeshChatX)`. Fill in:
   - **WebSocket URL** — your local MeshChatX endpoint (e.g. `ws://localhost:8000/ws`).
   - **Destination hash** — the 32-hex-character hash from step 3.
   - **Aspect** — usually `rnstransport.remote.management` (default).
   - **authenticate** — check this to identify on the Link with MeshChatX's local identity. Required if the node restricts management to an allow-list.
5. Click **Connect**. The status pill walks through the phases: *WS connecting → WS verifying → Link connecting → Finding path → Establishing link → Identifying → Connected*. This commonly takes 10–30 seconds over LoRa, much longer than the snappy Serial / BLE / WebSocket paths — the Console raises the per-request timeout when the RNS transport is in use to accommodate Resource transfers.

> **Note on MeshChatX.** MeshChatX is used here as a thin RNS-to-WebSocket bridge: it gives the browser something to talk to that knows how to open Reticulum Links. Any future tool implementing the same `rns.link.*` WebSocket protocol could be substituted. The Console caches its Link in MeshChatX, but a clean disconnect (`Disconnect` button, tab close, refresh) tells MeshChatX to evict the cache so the next session opens a fresh Link.

### Operating a remote node

Once connected over RNS, almost everything works the same as locally:

- **Node Status** — live metrics stream back over the Link.
- **Transport Config** — schema, drafts, commits, factory reset, and reboot all work.
- **Node Config** — works, but every legacy KISS exchange is a Link round-trip. Set-only fields are still set-only (the protocol limitation is in the firmware, not the transport).

Things that *don't* work over RNS:

- **Logs tab** — the Console hides it. The RNS transport only forwards `CMD_PROVISION_RSP` frames; KISS `CMD_LOG` frames are not multiplexed over the Link.
- **Raw EEPROM hex view** — this uses `CMD_ROM_READ`, which is a legacy opcode and so works over RNS, but the operation will be very slow and the protocol is brittle to packet loss. Prefer staying on Serial for EEPROM operations.

## Transports in detail

### Serial (USB)

- **Wire format:** standard RNode KISS over a USB CDC virtual serial port (`/dev/ttyACM*` on Linux/macOS, `COMx` on Windows).
- **Browser API:** Web Serial. Your browser might prompt for permission the first time.
- **Latency:** sub-millisecond. Best transport for first-boot provisioning and EEPROM work.
- **Caveat:** after a physical USB disconnect/reconnect, Your browser can leave the `SerialPort` JS object in a half-bound state; the Console works around this but you may see "Failed to open serial port" once — clicking **Connect** again resolves it.

### Bluetooth (BLE)

- **Wire format:** KISS framed as binary writes to a Nordic-UART-like characteristic on the BLE GATT service exposed by the firmware.
- **Browser API:** Web Bluetooth. Chrome's permission UI asks you to pick the device.
- **Use:** ESP32 boards with the BLE stack enabled, or nRF52 boards (T-Echo, T114, RAK4631). Set a BLE PIN under Node Config → Bluetooth for paired access.
- **Caveat:** BLE links drop more often than Serial. The Console exposes "auto-reconnect" in the topbar to handle this transparently.

### WebSocket (WiFi)

- **Wire format:** KISS frames as binary WebSocket messages over the device's KISS-over-WebSocket server.
- **Endpoint:** `ws://<node-ip>:81` on ESP32 builds, `ws://<host>:8080` on `native` builds.
- **Use:**
  - Embedded: device must be in WiFi STA or AP mode. In **console mode** (quick-reboot the device twice; the second reboot starts a WiFi AP named after the device) the Console is served at `http://10.0.0.1/` and connects to itself.
  - Native: the daemon starts the KISS-over-WebSocket server on port 8080 automatically. Loopback-only by default; set `kiss_ws_public = true` in `rnoded.conf` to bind on `0.0.0.0`.
- **Caveat — Chrome Local Network Access Checks:**

  > NOTE: To access a node on the LAN, Chrome Local Network Access Checks must be temporarily disabled.

  **What it is.** Chrome (and other Chromium-derived browsers — Edge, Brave, Arc, etc.) gates requests that cross from a "public" origin to a "private" address space. The browser considers any page loaded from the internet, or from a `file://` URL, or from a server outside [RFC 1918](https://datatracker.ietf.org/doc/html/rfc1918) / link-local space, to be public. Any IP in `10.0.0.0/8`, `172.16.0.0/12`, `192.168.0.0/16`, `169.254.0.0/16`, or `127.0.0.0/8` is considered private. When the page tries to open a connection from a public origin to a private one — including a `new WebSocket("ws://192.168.x.y:81")` from a Console you opened over the internet or from your disk — Chrome blocks it with a network error and a console message about *Private Network Access* or *Local Network Access*. This is the rollout of the *Private Network Access* spec (formerly "CORS-RFC1918"), recently rebranded to *Local Network Access* in newer Chrome versions.

  **When you'll hit it.**
  - Opening `Release/console.html` from disk and pointing it at `ws://<node-lan-ip>:81` (embedded) or `ws://<host-lan-ip>:8080` (native).
  - Loading the Console from a hosted https/http origin on the internet and trying to reach a node on your LAN.
  - You will **not** hit it when the device is in console mode and serving the Console itself from `http://10.0.0.1/`, because in that case both the page and the target are on the same private origin.

  **How to disable the check.** Open `chrome://flags/#local-network-access-check` in Chrome (or `chrome://flags/#block-insecure-private-network-requests` on slightly older Chrome builds), set it to **Disabled**, and relaunch the browser. The setting is per-profile and persists across restarts, so disable it deliberately and re-enable it when you're done. On Edge the same flag lives at `edge://flags/#local-network-access-check`. Firefox and Safari do not currently enforce Private/Local Network Access, but they also lack Web Serial and Web Bluetooth, so you'd only be using them for the WebSocket / RNS transports anyway.

  **Why we don't paper over it.** The "right" long-term fix is for the firmware's WebSocket server to participate in the Private Network Access preflight handshake (responding to the `Access-Control-Request-Private-Network: true` preflight with the matching `Access-Control-Allow-Private-Network: true` header). That requires a real HTTP layer in front of the WebSocket upgrade, which the embedded ESP32 web server doesn't fully expose. Until that lands, the Chrome flag is the supported workaround.

  **Security note.** Disabling this check loosens a browser-wide protection against cross-site attacks against private network devices (a malicious public website asking your browser to reconfigure your router, etc.). Only disable it when you actually need it, and re-enable it afterwards.

### RNS via MeshChatX

- **Wire format:** Provisioning frames carried over a Reticulum Link to the node's `rnstransport.remote.management` destination, brokered by a MeshChatX instance reachable from the browser by WebSocket.
- **Use:** any node reachable through your RNS mesh (LoRa, packet radio, TCP, UDP, … all transparent at the Link layer).
- **Latency:** depends on the underlying RNS path. On a multi-hop LoRa mesh, expect 5–30 seconds per request and tens of seconds for schema / EEPROM reads. The Console adjusts timeouts automatically.

## Console tabs in detail

### Node Status

Always available. Shows:

- **Device** — firmware version, schema version, board / platform / MCU, "needs reboot" flag.
- **Radio link** — last RSSI, last SNR, current RSSI, noise floor, interference RSSI.
- **Channel** — short and long airtime, channel utilisation.
- **Counters** — RX / TX packet counts.
- **PHY parameters** — symbol time, symbol rate, preamble symbols and time, CSMA slot, DIFS.
- **CSMA** — contention window band, min, max.
- **Power** — battery percentage and state (unknown / discharging / charging / charged), temperature.
- **Danger zone** — Reboot, Factory reset.

### Node Config

Visible only when the firmware responds to a `CMD_BOARD` probe (so it's hidden on RNS connections where the probe times out). Sub-tabs:

- **Device Info** — board, platform, MCU, firmware hash, device hash, signature.
- **Radio** — frequency, bandwidth, SF, coding rate, TX power, interference avoidance. Most fields are set-only; the panel combines an op-mode selector (HOST / TNC) with the radio params. Radio params only persist when the device is configured to boot into TNC.
- **Bluetooth** — pair / unpair, PIN, mode toggles.
- **WiFi** — mode (Off / STA / AP), channel, SSID, PSK, static IP, netmask. All set-only.
- **Display** — intensity, blanking, rotation, I2C address, NeoPixel intensity.
- **EEPROM** — raw hex dump with markers; danger-zone operations (`CMD_CONF_DELETE`, `CMD_UNLOCK_ROM`).

### Transport Config

Visible only when the firmware advertises any Provisioning namespace with id < 100 (i.e. a built-in RNS namespace, not just the General app namespace). Shows the namespace tree from the schema; left sidenav lists root namespaces, right pane renders the fields for the selected one with sub-section headings for nested namespaces. Each panel has a toolbar with **Save**, **Revert**, **Commit-all** (or **Discard-all**). Reboot-required edits raise a persistent banner at the top of the Console until you reboot.

### Logs

Visible on Serial / BLE / WebSocket connections. Streams KISS `CMD_LOG` frames from the device. Use the toolbar to clear, pause / resume follow, and download. Hidden on RNS connections.

## Caveats and pitfalls

### RNS over LoRa

This is by far the highest-leverage feature — and the easiest to underestimate.

- **Bandwidth.** A typical LoRa configuration (`SF8 BW125`) gives you on the order of a few kbps shared across the entire mesh. A schema fetch from a fresh connection can transfer several KB; over LoRa, that means **tens of seconds**. The Console handles this by raising per-request timeouts on the RNS transport, but plan for it.
- **Per-Link establishment cost.** Opening a Link to a node you haven't talked to recently kicks off path discovery and a Link handshake. Both are several packet exchanges, and on a busy / congested channel each exchange may itself need retries. Allow 10–30 seconds for `Establishing link`; a few minutes is not unusual on long, lossy paths.
- **No streaming.** Logs are not delivered over the RNS transport — only Provisioning request/response frames. If you need to debug a node's boot sequence, use Serial.
- **MeshChatX caches Links.** MeshChatX keeps the Link open across requests so subsequent calls don't pay the establishment cost again. The Console asks MeshChatX to evict that cache on `Disconnect`, page unload, and on `link_closed` events from RNS, but a stale cache can occasionally hand you a dead Link — the symptom is a fast "no_active_link" error on the first request after reconnecting. Disconnect and reconnect to force a fresh Link.
- **Don't commit radio params over RNS.** Frequency / bandwidth / SF / CR / TXP are `REBOOT_REQUIRED`. The reboot happens immediately and the new params will be in effect — but if you got the parameters wrong, the node is now on a channel you can't reach. **Never** change radio params on a remote node you can't physically recover.
- **Same warning applies to WiFi and interface mode changes.** Putting an embedded node's only reachable LoRa interface into `gateway` mode when you expected `full` can render the node unreachable. Use `point-to-point` interface modes with care.
- **Allow-list enforcement.** The firmware can be configured to only accept Provisioning requests from identified peers on an allow-list. If you set this up, make sure the local identity used by your MeshChatX instance is on that list before you disconnect — otherwise you've locked yourself out.

### Security and exposure

- The KISS protocol on Serial / BLE / WebSocket has **no authentication and no encryption**. Anyone who can talk to the wire can read and write configuration.
- The KISS-over-WebSocket endpoint on `native` and the KISS-over-TCP endpoint on `native` (port 7633) default to **loopback only**. Setting `kiss_ws_public = true` or `kiss_tcp_public = true` in `rnoded.conf` binds them on `0.0.0.0`, with no auth — only do this on a trusted network.
- The RNS transport authenticates Link peers cryptographically (Reticulum Identity) and supports an allow-list via `RNS::Transport::remote_management_allowed()`. Configure this if your management destination is reachable over public mesh paths.
- The `SECRET` field flag means a field's value is never returned in `GET_STATE` responses. PSKs and similar belong here. The Console displays "(secret — not exposed)" for these on read; you can still write a new value.

### WiFi and AP mode

- The embedded web console (HTTP on port 80, KISS-over-WebSocket on port 81) starts only when the device is in **console mode**. Activation: quick-reboot the device twice in quick succession (the first reboot stores a marker in the LoRa modem; the second sees the marker and enables the AP).
- In normal operation, WiFi is whatever you configured under Node Config → WiFi (Off / Station / AP). Station mode connects to your network; AP mode opens an access point named after the device.
- Switching WiFi mode is reboot-required. Get the SSID / PSK right before saving, especially if the node is remote.

### Reboot-required fields

Fields flagged `REBOOT_REQUIRED` in the schema (badge: `↻ reboot`) only take effect after a device reboot. Edits to those fields raise a banner at the top of the Console:

> *Reboot recommended to apply committed changes.*

Click **Reboot now** to apply. If "auto-reconnect" is checked in the topbar, the Console reconnects after the device comes back up and you keep your view of the node.

## Troubleshooting

- **"Failed to open serial port"** after a USB reset. Click **Connect** again — the Console works around a Chrome Web Serial quirk where the `SerialPort` JS object stays half-bound after re-enumeration.
- **Topbar pill stuck on "Connecting…"** over RNS. The status pill shows the phase and elapsed seconds. If it stalls on `Finding path`, the node is unreachable on your mesh — confirm with `rnpath` / `rnstatus`. If it stalls on `Establishing link`, the path is congested or one hop is dropping packets; wait or try again later.
- **"Send rejected: ReadOnly"** when committing. The field is marked `READ_ONLY` (a metric, not a setting). The Console should not have let you draft it — this usually indicates a schema/firmware mismatch. Reconnect to refetch the schema.
- **Reboot banner won't clear.** A reboot-required commit landed but the device didn't actually reboot. Click **Reboot now**, or use Node Status → Danger zone → Reboot.
- **Console works over Serial but not WebSocket.** Confirm the device is on WiFi (Node Config → WiFi → mode), check that you can ping the node's IP, and verify Chrome's Local Network Access flag (see [Opening the Console](#opening-the-console)).
- **RNS transport disconnects immediately.** MeshChatX may not be running, or its WebSocket URL is wrong. Open a browser tab to MeshChatX's web UI to confirm it's up, and check the path component of the WebSocket URL.
