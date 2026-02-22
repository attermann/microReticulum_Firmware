#!/bin/bash
#
# Flash script for Heltec Wireless Stick Lite V1 — RNode Firmware
#
# Usage: ./flash_wsl_v1.sh [PORT] [FREQ_BAND] [--no-tnc] [--erase]
#   PORT:       Serial port (default: /dev/cu.usbserial-0001)
#   FREQ_BAND:  433, 868, or 915 (default: 868)
#   --no-tnc:   Skip TNC configuration (leave in normal/host-controlled mode)
#   --erase:    Erase entire flash before uploading (clean install)
#
# Examples:
#   ./flash_wsl_v1.sh                                    # Default port, 868 MHz, TNC enabled
#   ./flash_wsl_v1.sh /dev/ttyUSB0 915                   # Custom port, 915 MHz
#   ./flash_wsl_v1.sh /dev/cu.usbserial-0001 868 --erase # Clean flash first
#

set -e

# --- Parse arguments ---
PORT="/dev/cu.usbserial-0001"
FREQ_BAND="868"
SKIP_TNC=false
DO_ERASE=false

for arg in "$@"; do
    case "$arg" in
        --no-tnc)  SKIP_TNC=true ;;
        --erase)   DO_ERASE=true ;;
        /dev/*)    PORT="$arg" ;;
        433|868|915) FREQ_BAND="$arg" ;;
    esac
done

# --- Band configuration ---
# Product 0xC5 = HWSL_V1
# Model 0xCB = 433 MHz (SX1278) | Model 0xCC = 868/915 MHz (SX1276)
PRODUCT="c5"
HWREV="1"

case "$FREQ_BAND" in
    433)
        MODEL="cb"
        FREQ="433775000"
        BW="125000"
        echo "=== Configuring for 433 MHz band (SX1278) ==="
        ;;
    868)
        MODEL="cc"
        FREQ="869525000"
        BW="250000"
        echo "=== Configuring for 868 MHz EU ISM band (SX1276) ==="
        ;;
    915)
        MODEL="cc"
        FREQ="915000000"
        BW="250000"
        echo "=== Configuring for 915 MHz US ISM band (SX1276) ==="
        ;;
    *)
        echo "ERROR: Invalid frequency band '$FREQ_BAND'. Use 433, 868, or 915."
        exit 1
        ;;
esac

ENV="heltec_wsl_v1"

echo ""
echo "  Port:    $PORT"
echo "  Product: 0x$PRODUCT (HWSL_V1)"
echo "  Model:   0x$MODEL"
echo "  Band:    $FREQ_BAND MHz"
echo ""

# --- Check prerequisites ---
if ! command -v pio &> /dev/null; then
    echo "ERROR: PlatformIO not found. Install with: pip install platformio"
    exit 1
fi

if ! command -v rnodeconf &> /dev/null; then
    echo "ERROR: rnodeconf not found. Install with: pip install rns"
    exit 1
fi

# --- Step 1: Build firmware ---
echo "=== Step 1: Building firmware ==="
pio run -e "$ENV"
echo ""

# --- Step 2: Erase flash (optional) ---
if [ "$DO_ERASE" = true ]; then
    echo "=== Step 2: Erasing flash (clean install) ==="
    pio run -e "$ENV" -t erase --upload-port "$PORT"
    echo ""
    echo "  Press RESET on the board, then press Enter to continue..."
    read -r
fi

# --- Step 3: Upload firmware ---
# PIO's post_upload script automatically sets the firmware hash via rnodeconf
echo "=== Step 3: Uploading firmware + setting firmware hash ==="
pio run -e "$ENV" -t upload --upload-port "$PORT"

echo ""
echo "  Waiting for device to boot..."
sleep 5

# --- Step 4: Provision EEPROM ---
echo "=== Step 4: Provisioning EEPROM ==="
rnodeconf "$PORT" --rom --product "$PRODUCT" --model "$MODEL" --hwrev "$HWREV"

sleep 3

# --- Step 5: Enable TNC mode ---
if [ "$SKIP_TNC" = false ]; then
    echo ""
    echo "=== Step 5: Enabling TNC mode ==="
    rnodeconf "$PORT" --tnc --freq "$FREQ" --bw "$BW" --sf 7 --cr 5 --txp 14
    sleep 2
else
    echo ""
    echo "=== Step 5: Skipped (--no-tnc) — device left in normal mode ==="
fi

# --- Step 6: Verify ---
echo ""
echo "=== Step 6: Verifying device ==="
rnodeconf "$PORT" -i

echo ""
echo "=== Done! ==="
if [ "$SKIP_TNC" = false ]; then
    echo "  Device is now an RNode transport node on $FREQ_BAND MHz."
    echo ""
    echo "  Add to ~/.reticulum/config:"
    echo ""
    echo "    [[WSL]]"
    echo "      type = RNodeInterface"
    echo "      interface_enabled = true"
    echo "      port = $PORT"
    echo "      frequency = $FREQ"
    echo "      bandwidth = $BW"
    echo "      txpower = 14"
    echo "      spreadingfactor = 7"
    echo "      codingrate = 5"
    echo ""
    echo "  To switch to normal (host-controlled) mode:"
    echo "    rnodeconf $PORT --normal"
else
    echo "  Device provisioned in normal mode."
    echo "  To enable TNC mode:"
    echo "    rnodeconf $PORT --tnc --freq $FREQ --bw $BW --sf 7 --cr 5 --txp 14"
fi
echo ""
