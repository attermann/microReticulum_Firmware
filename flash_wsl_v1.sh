#!/bin/bash
#
# Flash script for Heltec Wireless Stick Lite V1 - microReticulum Transport Node Firmware
#
# Usage: ./flash_wsl_v1.sh [PORT] [FREQ_BAND]
#   PORT: Serial port (default: /dev/cu.usbserial-0001)
#   FREQ_BAND: 433 or 868 (default: 868)
#

set -e

PORT="${1:-/dev/cu.usbserial-0001}"
FREQ_BAND="${2:-868}"

# Model codes
if [ "$FREQ_BAND" == "433" ]; then
    MODEL="cb"
    FREQ="433000000"
    echo "=== Configuring for 433 MHz band ==="
else
    MODEL="cc"
    FREQ="868000000"
    echo "=== Configuring for 868/915 MHz band ==="
fi

PRODUCT="c5"
HWREV="1"

echo ""
echo "Port: $PORT"
echo "Product: 0x$PRODUCT (HWSL_V1)"
echo "Model: 0x$MODEL"
echo ""

# Step 1: Build firmware
echo "=== Step 1: Building firmware ==="
if command -v pio &> /dev/null; then
    pio run -e heltec_wsl_v1
    FIRMWARE_BIN=".pio/build/heltec_wsl_v1/firmware.bin"
else
    echo "ERROR: PlatformIO not found. Install with: pip install platformio"
    exit 1
fi

# Step 2: Upload firmware
echo ""
echo "=== Step 2: Uploading firmware ==="
pio run -e heltec_wsl_v1 -t upload --upload-port "$PORT"

sleep 2

# Step 3: Provision EEPROM
echo ""
echo "=== Step 3: Provisioning EEPROM ==="
rnodeconf "$PORT" --rom --product "$PRODUCT" --model "$MODEL" --hwrev "$HWREV"

sleep 2

# Step 4: Set firmware hash
echo ""
echo "=== Step 4: Setting firmware hash ==="
if [ -f "./partition_hashes" ]; then
    HASH=$(./partition_hashes "$FIRMWARE_BIN")
elif [ -f "../partition_hashes" ]; then
    HASH=$(../partition_hashes "$FIRMWARE_BIN")
else
    echo "WARNING: partition_hashes script not found, skipping hash"
    HASH=""
fi

if [ -n "$HASH" ]; then
    rnodeconf "$PORT" --firmware-hash "$HASH"
fi

sleep 2

# Step 5: Enable TNC/Transport mode
echo ""
echo "=== Step 5: Enabling TNC mode (standalone transport) ==="
rnodeconf "$PORT" --tnc --freq "$FREQ" --bw 125000 --sf 7 --cr 5 --txp 14

sleep 2

# Step 6: Verify
echo ""
echo "=== Step 6: Verifying device ==="
rnodeconf "$PORT" -i

echo ""
echo "=== Done! ==="
echo "Your Heltec Wireless Stick Lite V1 is now a standalone Reticulum transport node."
echo ""
echo "To return to normal (host-controlled) mode:"
echo "  rnodeconf $PORT -N"
echo ""
