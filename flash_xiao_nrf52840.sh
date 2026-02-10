#!/bin/bash
# Flash script for XIAO nRF52840 + Wio-SX1262 microReticulum firmware
# Usage: ./flash_xiao_nrf52840.sh [PORT] [FREQ_BAND]
# FREQ_BAND: 433, 868, or 915 (default: 868)

set -e

PORT="${1:-/dev/ttyACM0}"
FREQ_BAND="${2:-868}"
FIRMWARE_DIR=".pio/build/xiao_nrf52840"
FIRMWARE_NAME="rnode_firmware_xiao_nrf52840"

# Product code for XIAO nRF52840 (hex, matching ROM.h PRODUCT_XIAO_NRF52840 = 0x11)
PRODUCT="11"

# Select model based on frequency band
# MODEL_11 = 0x11 (433/915 MHz), MODEL_12 = 0x12 (868 MHz)
if [ "$FREQ_BAND" == "433" ] || [ "$FREQ_BAND" == "915" ]; then
    MODEL="11"
    if [ "$FREQ_BAND" == "433" ]; then
        FREQ="433000000"
    else
        FREQ="915000000"
    fi
    echo "=== Configuring for $FREQ_BAND MHz band ==="
else
    MODEL="12"
    FREQ="869525000"
    echo "=== Configuring for 868 MHz band ==="
fi

echo "=== XIAO nRF52840 + Wio-SX1262 microReticulum Flash Script ==="
echo "Port: $PORT"
echo "Frequency band: $FREQ_BAND MHz"
echo "Product: 0x$PRODUCT, Model: 0x$MODEL"
echo ""

# Check if pio is available
if ! command -v pio &> /dev/null; then
    echo "ERROR: PlatformIO (pio) not found. Please install it first:"
    echo "  pip install platformio"
    exit 1
fi

# Check if rnodeconf is available
if ! command -v rnodeconf &> /dev/null; then
    echo "WARNING: rnodeconf not found. Will use provision_xiao.py instead."
    echo "  Install Reticulum for rnodeconf: pip install rns"
    SKIP_RNODECONF=1
fi

# Build firmware
echo "=== Building firmware ==="
pio run -e xiao_nrf52840

# Check for firmware output
ZIP_FILE="$FIRMWARE_DIR/${FIRMWARE_NAME}.zip"
HEX_FILE="$FIRMWARE_DIR/${FIRMWARE_NAME}.hex"

if [ ! -f "$ZIP_FILE" ] && [ ! -f "$HEX_FILE" ]; then
    echo "ERROR: Firmware build failed - no output files found"
    echo "Expected: $ZIP_FILE or $HEX_FILE"
    exit 1
fi

echo ""
echo "=== Firmware built successfully ==="
ls -la "$FIRMWARE_DIR/${FIRMWARE_NAME}"* 2>/dev/null || true

# Extract .bin from .zip for firmware hash calculation
BIN_FILE="$FIRMWARE_DIR/${FIRMWARE_NAME}.bin"
if [ -f "$ZIP_FILE" ] && [ ! -f "$BIN_FILE" ]; then
    echo "Extracting .bin from DFU package for hash..."
    unzip -o "$ZIP_FILE" "${FIRMWARE_NAME}.bin" -d "$FIRMWARE_DIR" 2>/dev/null || true
fi

# Calculate firmware hash from the .bin file
if [ -f "$BIN_FILE" ]; then
    FIRMWARE_HASH=$(sha256sum "$BIN_FILE" | cut -d' ' -f1)
    echo "Firmware hash (bin): $FIRMWARE_HASH"
elif [ -f "$HEX_FILE" ]; then
    FIRMWARE_HASH=$(sha256sum "$HEX_FILE" | cut -d' ' -f1)
    echo "Firmware hash (hex): $FIRMWARE_HASH"
fi

echo ""
echo "=== Ready to flash ==="
echo "Please double-tap the reset button on the XIAO to enter bootloader mode."
echo "A USB drive named 'XIAO-SENSE' or similar should appear."
echo ""
read -p "Press Enter when the device is in bootloader mode..."

# Try DFU upload methods in order of preference
if [ -f "$ZIP_FILE" ]; then
    echo "=== Uploading via DFU package ==="
    
    if command -v adafruit-nrfutil &> /dev/null; then
        echo "Using adafruit-nrfutil for DFU upload..."
        adafruit-nrfutil dfu serial --package "$ZIP_FILE" -p "$PORT" -b 115200 --singlebank
    else
        echo "adafruit-nrfutil not found. Trying pio upload..."
        pio run -e xiao_nrf52840 --target upload --upload-port "$PORT"
    fi
else
    echo "=== Uploading via PlatformIO ==="
    pio run -e xiao_nrf52840 --target upload --upload-port "$PORT"
fi

# Wait for device to reboot
echo ""
echo "Waiting for device to reboot..."
SERIAL_PORT=""
for i in {1..30}; do
    # macOS uses cu.usbmodem*, Linux uses ttyACM*
    for p in /dev/cu.usbmodem* /dev/ttyACM*; do
        if [ -e "$p" ]; then
            SERIAL_PORT="$p"
            break 2
        fi
    done
    echo "  Waiting... ($i/30)"
    sleep 1
done

if [ -z "$SERIAL_PORT" ]; then
    echo ""
    echo "WARNING: Device port not found after reboot."
    echo "The device may need more time, or be on a different port."
    echo ""
    echo "To provision manually:"
    echo "  python provision_xiao.py <PORT> $FREQ_BAND $BIN_FILE"
    echo ""
    echo "Or with rnodeconf:"
    echo "  rnodeconf <PORT> --rom --product $PRODUCT --model $MODEL --hwrev 1"
    echo "  rnodeconf <PORT> --firmware-hash $FIRMWARE_HASH"
    echo "  rnodeconf <PORT> --tnc --freq $FREQ --bw 125000 --sf 7 --cr 5 --txp 14"
    exit 0
fi

echo "Device is back online at $SERIAL_PORT"
sleep 2  # Give firmware time to initialize

# Provision the device
echo ""
echo "=== Provisioning device ==="

# Prefer provision_xiao.py since it handles the XIAO-specific EEPROM layout correctly
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROVISION_SCRIPT="$SCRIPT_DIR/provision_xiao.py"

if [ -f "$PROVISION_SCRIPT" ]; then
    echo "Using provision_xiao.py..."
    python3 "$PROVISION_SCRIPT" "$SERIAL_PORT" "$FREQ_BAND" "$BIN_FILE"
elif [ -z "$SKIP_RNODECONF" ]; then
    echo "Using rnodeconf..."
    echo "Writing product code: $PRODUCT, model: $MODEL"
    rnodeconf "$SERIAL_PORT" --rom --product "$PRODUCT" --model "$MODEL" --hwrev 1

    echo ""
    echo "=== Setting firmware hash ==="
    rnodeconf "$SERIAL_PORT" --firmware-hash "$FIRMWARE_HASH"

    echo ""
    echo "=== Enabling TNC mode (standalone transport) ==="
    rnodeconf "$SERIAL_PORT" --tnc --freq "$FREQ" --bw 125000 --sf 7 --cr 5 --txp 14
else
    echo "ERROR: Neither provision_xiao.py nor rnodeconf available."
    echo "Install rns (pip install rns) or ensure provision_xiao.py is alongside this script."
    exit 1
fi

echo ""
echo "=== Verifying configuration ==="
if [ -z "$SKIP_RNODECONF" ]; then
    rnodeconf "$SERIAL_PORT" -i
fi

echo ""
echo "=========================================="
echo "  Flash & provision complete!"
echo "=========================================="
echo ""
echo "The device is now configured as a standalone"
echo "Reticulum transport node on $FREQ_BAND MHz."
echo ""
echo "To return to normal RNode mode:"
echo "  rnodeconf $SERIAL_PORT -N"
echo ""
echo "To connect via BLE, pair with the device"
echo "named 'RNode XXXX'"
echo ""
