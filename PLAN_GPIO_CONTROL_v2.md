# LXMF GPIO Control — Autonomous ESP32 Node
## TTGO LoRa 32 v1 + microReticulum

### Updated: 2026-02-22 — Bridge eliminated, fully embedded LXMF

---

## Architecture

```
┌─────────────────┐         LoRa          ┌──────────────────────┐
│   Sideband      │ ◄──────────────────── │  TTGO LoRa 32 v1     │
│   (phone/PC)    │ ────────────────────► │                      │
│                 │    LXMF single-packet  │  ┌─────────────────┐ │
│  Send: SET 13 H │                       │  │ LXMF_Minimal.h  │ │
│  Recv: OK GPIO  │                       │  │  parse/pack msg  │ │
│         13:HIGH │                       │  └────────┬────────┘ │
└─────────────────┘                       │           │          │
                                          │  ┌────────▼────────┐ │
                                          │  │ GPIO_Control.h  │ │
                                          │  │  command parser  │ │
                                          │  │  pin operations  │ │
                                          │  └────────┬────────┘ │
                                          │           │          │
                                          │     GPIO 13,17,23,  │
                                          │     25,32,33,34-39  │
                                          └──────────────────────┘
```

**Key change:** No bridge script needed. The ESP32 IS the LXMF endpoint.
Sideband sends a text message → LoRa → ESP32 parses LXMF → executes GPIO → 
packs LXMF reply → LoRa → Sideband receives response.

### How it works

1. **ESP32 boots** → creates RNS Identity → creates Destination("lxmf", "delivery")
2. **ESP32 announces** → Sideband discovers it as a messaging endpoint
3. **User sends text** from Sideband (e.g., "SET 13 HIGH")
4. **Sideband sends LXMF** as single-packet (opportunistic) — fits easily in one LoRa packet
5. **ESP32 receives packet** → callback fires → LXMF_Minimal parses msgpack payload
6. **GPIO_Control** extracts content text → parses command → executes GPIO operation
7. **ESP32 builds reply** → packs LXMF message → sends as single packet back
8. **Sideband receives** the reply text (e.g., "OK GPIO 13: HIGH")

### Why single-packet LXMF works here

LXMF has two delivery modes:
- **Link-based (DIRECT):** Requires Link + Channel support — NOT in microReticulum yet
- **Single-packet (OPPORTUNISTIC):** Just a regular RNS packet — WORKS NOW

GPIO commands are short text ("SET 13 HIGH" = 12 bytes). Max content for encrypted
single-packet LXMF is **295 bytes**. We have plenty of room.

---

## LXMF Wire Format (for reference)

### Packed message (full)
```
destination_hash  [16 bytes]
source_hash       [16 bytes]
signature         [64 bytes]  — Ed25519
packed_payload    [variable]  — msgpack([timestamp, title, content, fields])
```

### Opportunistic packet data (what arrives at packet callback)
```
source_hash       [16 bytes]  — sender's destination hash
signature         [64 bytes]  — Ed25519 signature
packed_payload    [variable]  — msgpack array
```

The destination_hash is implicit (it's our own destination).
Router prepends it when calling `lxmf_delivery()`.

### Payload structure (msgpack array)
```
[
  timestamp,    // float64 — UNIX epoch seconds
  title,        // bytes   — empty for GPIO commands
  content,      // bytes   — the command text ("SET 13 HIGH")
  fields,       // map|nil — unused
  stamp         // bytes   — optional 5th element (anti-spam, can be ignored)
]
```

### Destination aspect
```
App name: "lxmf"
Aspect:   "delivery"
Full:     "lxmf.delivery"
```

This is what Sideband scans for in announces.

---

## Files

### New files (add to firmware directory)

| File | Purpose |
|------|---------|
| `LXMF_Minimal.h` | Embedded LXMF: parse incoming, build outgoing single-packet messages |
| `GPIO_Control.h` | GPIO pin management + command parser + LXMF message handler |
| `test_lxmf_gpio.py` | Python test client (sends LXMF from laptop to verify) |

### Modified files

| File | Change |
|------|--------|
| `RNode_Firmware.ino` | Include GPIO_Control.h, init after RNS, call loop |
| `platformio.ini` | Add `-DHAS_GPIO_CONTROL` to ttgo-lora32-v1 env |

### Build scripts (from previous session)

| File | Purpose |
|------|---------|
| `flash_ttgo_gpio.sh` | Build + flash + provision + TNC config |

---

## GPIO Pin Map — TTGO LoRa 32 v1

### Safe bidirectional (output + input)
| Pin | Notes |
|-----|-------|
| 13  | General purpose, safe |
| 17  | TX2 (safe if UART2 unused) |
| 23  | General purpose, safe |
| 25  | DAC1, general purpose |
| 32  | ADC1_CH4, general purpose |
| 33  | ADC1_CH5 (check DIO1 on some boards) |

### Input only
| Pin | Notes |
|-----|-------|
| 34  | ADC1_CH6 |
| 35  | ADC1_CH7 |
| 36  | VP (sensor) |
| 39  | VN (sensor) |

### Reserved (DO NOT expose)
| Pin | Used by |
|-----|---------|
| 0   | Boot mode |
| 2   | Onboard LED |
| 4   | OLED SDA |
| 5   | SPI SS (LoRa) |
| 12  | Boot (pulldown sensitive) |
| 14  | LoRa RST |
| 15  | OLED SCL |
| 16  | OLED RST |
| 18  | SPI SCK |
| 19  | SPI MISO |
| 26  | LoRa IRQ |
| 27  | SPI MOSI |

---

## Command Protocol

### Commands (case-insensitive)
```
SET <pin> HIGH|LOW|ON|OFF|1|0   Set output pin state
GET <pin>                        Read pin state
MODE <pin> IN|OUT                Set pin direction
STATUS                           Report all configured pins
PINS                             List available GPIO pins
HELP                             Show command list
```

### Response format
```
OK GPIO <pin>: HIGH|LOW          Pin state changed/read
OK MODE <pin>: IN|OUT            Pin direction set
ERR: <message>                   Error with description
STATUS: 13=HIGH 17=LOW ...       All pin states
```

### Auto-configure behavior
- `SET` auto-configures pin as OUTPUT if not already set
- `GET` works on any pin regardless of mode
- `MODE` must be called explicitly for INPUT pins

---

## Build & Deploy

### Prerequisites
- PlatformIO with ESP32 platform
- TTGO LoRa 32 v1 connected via USB

### Quick start
```bash
# Build and flash
pio run -e ttgo-lora32-v1 -t upload --upload-port /dev/cu.usbserial-0001

# Monitor serial output
tio /dev/cu.usbserial-0001

# Provision RNode
rnodeconf /dev/cu.usbserial-0001 --rom --product B2 --model BB --hwrev 1

# Set TNC mode (868 MHz EU)
rnodeconf /dev/cu.usbserial-0001 --tnc --freq 869525000 --bw 250000 --sf 7 --cr 5 --txp 14
```

### What to expect on serial
```
[RNS] Reticulum starting...
[RNS] Interface ready
[GPIO] Initializing GPIO control...
[LXMF] Delivery destination ready: a1b2c3d4e5f6a1b2c3d4e5f6a1b2c3d4
[GPIO] Ready!
[GPIO] LXMF address: a1b2c3d4e5f6a1b2c3d4e5f6a1b2c3d4
[GPIO] Send commands from Sideband to this address
[LXMF] Announced delivery destination
```

### Connect from Sideband
1. Copy the LXMF address from serial output
2. Open Sideband → New Conversation → paste address
3. Wait for announce (or trigger manually)
4. Send: `HELP`
5. Receive command list
6. Send: `SET 13 HIGH`
7. Receive: `OK GPIO 13: HIGH`

### Test with Python client (alternative)
```bash
pip install rns lxmf
python3 test_lxmf_gpio.py a1b2c3d4e5f6a1b2c3d4e5f6a1b2c3d4 HELP
python3 test_lxmf_gpio.py a1b2c3d4e5f6a1b2c3d4e5f6a1b2c3d4 "SET 13 HIGH"
python3 test_lxmf_gpio.py a1b2c3d4e5f6a1b2c3d4e5f6a1b2c3d4   # interactive
```

---

## Memory Budget (TTGO LoRa 32 v1)

| Component | Estimate |
|-----------|----------|
| Base RNode firmware | ~60 KB |
| microReticulum (Identity, Destination, Packet, Transport) | ~80-120 KB |
| LXMF_Minimal (msgpack parse/build, send/receive) | ~3 KB |
| GPIO_Control (command parser, pin management) | ~2 KB |
| OLED display buffer | ~1 KB |
| **Total** | **~150-190 KB** |
| **Free heap (of 520 KB)** | **~330+ KB** |

Much more headroom than the Heltec WSL v1 (320 KB total).

---

## Known Limitations & Future Work

### Current limitations
- **No signature verification on receive** — we trust the RNS encryption layer.
  Full LXMF signature validation requires knowing the sender's Identity, which
  may not always be cached. For GPIO control behind your own LoRa network this
  is acceptable.
- **Timestamp is approximate** — ESP32 uses `millis()` for reply timestamps.
  Without NTP or RTC, timestamps are relative to boot. Sideband doesn't care
  much about reply timestamps for display purposes.
- **No stamp support** — LXMF anti-spam stamps are ignored. Not needed for
  private networks.
- **Reply requires sender's Identity** — the ESP32 must have seen the sender's
  announce to encrypt the reply. Sideband announces automatically, but there
  may be a delay on first contact.

### Future improvements
- **OLED display** — show last command, response, and signal strength on screen
- **Persistent pin state** — save GPIO config to LittleFS, restore on boot
- **Pin aliases** — "SET RELAY1 ON" instead of "SET 13 HIGH"
- **Watchdog** — auto-reset pins if no command received within timeout
- **Sensor reading** — periodic ADC readings sent as unsolicited LXMF messages
- **Full LXMF** — when microReticulum gets Link support, upgrade to full LXMF
  with delivery receipts, longer messages, and Resources

---

## Risk Assessment

| Risk | Impact | Mitigation |
|------|--------|------------|
| microReticulum Destination callback doesn't fire | High | Fallback: use Transport-level receive_packet | 
| MsgPack library API mismatch | Medium | Check hideakitai/MsgPack examples, adjust deserialize calls |
| Reply encryption fails (sender Identity unknown) | Medium | Log warning, skip reply. Sender must announce first |
| Heap corruption on boot | Low | TTGO has 520KB SRAM. Monitor ESP.getFreeHeap() |
| OLED conflicts with GPIO | Low | Strict pin reservation in whitelist |
| LoRa packet too large for LXMF reply | Very Low | Max content 295 bytes, our replies are <100 bytes |
