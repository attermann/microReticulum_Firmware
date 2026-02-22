#!/bin/bash
#
# Flash script for TTGO LoRa32 v1 — microReticulum + GPIO Control
#
# Usage: ./flash_ttgo_v1.sh [PORT]
#   PORT: Serial port (default: /dev/cu.usbserial-0001)
#

set -e

PORT="${1:-/dev/cu.usbserial-0001}"
ENV="ttgo-lora32-v1"
BIN=".pio/build/$ENV/rnode_firmware_lora32v10.bin"

echo ""
echo "=== TTGO LoRa32 v1 Flash Script ==="
echo "  Port: $PORT"
echo "  Env:  $ENV"
echo ""

# 1. Erase everything clean
echo "=== Step 1/6: Erasing flash ==="
pio run -e "$ENV" -t erase --upload-port "$PORT"
echo ""

# 2. Upload filesystem image (formats LittleFS partition)
echo "=== Step 2/6: Uploading LittleFS filesystem ==="
pio run -e "$ENV" -t uploadfs --upload-port "$PORT"
echo ""

# 3. Clean build & upload firmware
echo "=== Step 3/6: Clean build & uploading firmware ==="
pio run -e "$ENV" -t clean
pio run -e "$ENV" -t upload --upload-port "$PORT"
echo ""

# 4. Provision ROM
echo "=== Step 4/6: Provisioning EEPROM ==="
echo ""
echo "  >>> Press RESET on the board now, then press Enter here <<<"
read -r
rnodeconf "$PORT" --rom --product B2 --model BB --hwrev 1
echo ""

# 5. Set firmware hash
echo "=== Step 5/6: Setting firmware hash ==="
sleep 2
HASH=$(shasum -a 256 "$BIN" | cut -d' ' -f1)
echo "  Hash: $HASH"
rnodeconf --firmware-hash "$HASH" "$PORT"
echo ""

# 6. Enable TNC mode
echo "=== Step 6/6: Enabling TNC mode ==="
sleep 2
rnodeconf "$PORT" --tnc --freq 869525000 --bw 250000 --sf 7 --cr 5 --txp 14
echo ""

echo "=== Done! ==="
echo "  Monitor with: tio $PORT"
echo ""
