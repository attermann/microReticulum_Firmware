#!/bin/bash
#
# Flash TTGO LoRa 32 v1 — GPIO Control Firmware
# Erases flash, builds ttgo-lora32-v1, uploads, provisions, enables TNC
#
# Usage: ./flash_ttgo_gpio.sh [--no-tnc]
#

set -e

PORT="/dev/cu.usbserial-3"
PRODUCT="B2"
MODEL="BB"
HWREV="1"
FREQ="869525000"
BW="250000"
ENV="ttgo-lora32-v1"
SKIP_TNC=false

for arg in "$@"; do
    case "$arg" in
        --no-tnc) SKIP_TNC=true ;;
    esac
done

echo ""
echo "=========================================="
echo "  TTGO LoRa 32 v1 — GPIO Control Flash"
echo "=========================================="
echo "  Port:    $PORT"
echo "  Product: 0x$PRODUCT  Model: 0x$MODEL"
echo "  Env:     $ENV"
echo "=========================================="
echo ""

# Check port
if [ ! -e "$PORT" ]; then
    echo "ERROR: $PORT not found"
    echo "Available ports:"
    ls /dev/cu.usbserial* 2>/dev/null || echo "  (none)"
    exit 1
fi

# Step 1: Erase
echo "=== Step 1: Erasing flash ==="
esptool.py --port "$PORT" erase_flash
echo ""
echo "  Press RESET on the board, then press Enter..."
read -r
sleep 2

# Step 2: Build
echo "=== Step 2: Building firmware (ttgo-lora32-v1 + GPIO control) ==="
pio run -e "$ENV"
echo ""

# Step 3: Upload
echo "=== Step 3: Uploading firmware ==="
pio run -e "$ENV" -t upload --upload-port "$PORT"
echo ""
echo "  Waiting for boot..."
sleep 5

# Step 4: Provision
echo "=== Step 4: Provisioning EEPROM ==="
echo "  Product: 0x$PRODUCT  Model: 0x$MODEL  HW Rev: $HWREV"
rnodeconf "$PORT" --rom --product "$PRODUCT" --model "$MODEL" --hwrev "$HWREV"
sleep 3

# Step 5: TNC mode
if [ "$SKIP_TNC" = false ]; then
    echo ""
    echo "=== Step 5: Enabling TNC mode ==="
    echo "  Freq: $FREQ Hz  BW: $BW Hz  SF: 7  CR: 5  TXP: 14 dBm"
    rnodeconf "$PORT" --tnc --freq "$FREQ" --bw "$BW" --sf 7 --cr 5 --txp 14
    sleep 2
fi

# Step 6: Verify
echo ""
echo "=== Step 6: Verifying ==="
rnodeconf "$PORT" -i

echo ""
echo "=========================================="
echo "  Flash complete!"
echo "=========================================="
echo ""
echo "  Monitor serial output with:"
echo "    tio $PORT"
echo ""
echo "  Look for:"
echo "    [RNode] Board: LORA32_V1_0"
echo "    [GPIO] LXMF address: <hex>"
echo "    [GPIO] Ready!"
echo ""
