#!/usr/bin/env bash
# Flash microReticulum firmware onto Seeed XIAO nRF52840 + Wio-SX1262
# Usage: ./flash_xiao.sh [serial_port]
# If serial_port is omitted the script will try to auto-detect it.

set -euo pipefail

REPO="slack-t/microReticulum_Firmware"
ASSET="rnode_firmware_xiao_nrf52840.zip"
TMPDIR="$(mktemp -d)"
trap 'rm -rf "$TMPDIR"' EXIT

# ── colour helpers ──────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'
info()  { echo -e "${GREEN}[*]${NC} $*"; }
warn()  { echo -e "${YELLOW}[!]${NC} $*"; }
die()   { echo -e "${RED}[✗]${NC} $*" >&2; exit 1; }

# ── dependency check ────────────────────────────────────────────────────────
for cmd in adafruit-nrfutil rnodeconf curl; do
  command -v "$cmd" &>/dev/null || die "Required tool not found: $cmd"
done

# ── serial port ─────────────────────────────────────────────────────────────
if [[ $# -ge 1 ]]; then
  PORT="$1"
else
  # Auto-detect: prefer /dev/ttyACM0, fall back to any ttyACM/ttyUSB
  PORT=""
  for candidate in /dev/ttyACM0 /dev/ttyACM1 /dev/ttyUSB0 /dev/ttyUSB1; do
    if [[ -e "$candidate" ]]; then PORT="$candidate"; break; fi
  done
  [[ -n "$PORT" ]] || die "Could not auto-detect serial port. Pass it explicitly: $0 /dev/ttyACM0"
  info "Auto-detected serial port: $PORT"
fi

[[ -e "$PORT" ]] || die "Serial port $PORT not found"

# ── download firmware ────────────────────────────────────────────────────────
info "Fetching latest release info from github.com/$REPO ..."
RELEASE_JSON=$(curl -fsSL "https://api.github.com/repos/$REPO/releases/latest")
DOWNLOAD_URL=$(echo "$RELEASE_JSON" | grep -o "\"browser_download_url\": \"[^\"]*${ASSET}\"" | cut -d'"' -f4)
TAG=$(echo "$RELEASE_JSON" | grep -o '"tag_name": "[^"]*"' | cut -d'"' -f4)

[[ -n "$DOWNLOAD_URL" ]] || die "Could not find $ASSET in latest release"
info "Downloading firmware $TAG ..."
curl -fsSL -o "$TMPDIR/$ASSET" "$DOWNLOAD_URL"
info "Downloaded: $(du -h "$TMPDIR/$ASSET" | cut -f1)"

# ── DFU flash ────────────────────────────────────────────────────────────────
echo
warn "Put the XIAO in DFU bootloader mode:"
warn "  → Double-tap the RESET button — the red LED should pulse slowly."
warn "  → The board will re-enumerate; if $PORT disappears briefly, wait for it."
echo
read -rp "Press ENTER when the LED is pulsing and $PORT is available..."

[[ -e "$PORT" ]] || die "$PORT not found after reset — check the port and try again"

info "Flashing firmware via DFU ..."
adafruit-nrfutil --verbose dfu serial \
  --package "$TMPDIR/$ASSET" \
  -p "$PORT" -b 115200

info "Flash complete. Waiting for device to reboot ..."
sleep 6

# ── EEPROM provisioning ──────────────────────────────────────────────────────
[[ -e "$PORT" ]] || { sleep 4; }
[[ -e "$PORT" ]] || die "$PORT still not present after reboot — check USB connection"

info "Provisioning EEPROM (product=0x10 model=0x12) ..."
rnodeconf --product 10 --model 12 --hwrev 1 --rom "$PORT"

info "Writing firmware hash ..."
# Extract hash from the release metadata, or compute it locally
HASH_JSON=$(curl -fsSL "https://raw.githubusercontent.com/$REPO/refs/tags/$TAG/Release/release.json" 2>/dev/null || true)
FW_HASH=$(echo "$HASH_JSON" | grep -o '"hash": "[^"]*"' | cut -d'"' -f4)

if [[ -n "$FW_HASH" ]]; then
  rnodeconf --firmware-hash "$FW_HASH" "$PORT"
else
  warn "Could not fetch hash from release metadata — computing locally ..."
  cd "$TMPDIR"
  unzip -o "$ASSET" "$(basename "$ASSET" .zip).bin" &>/dev/null || \
    unzip -o "$ASSET" *.bin &>/dev/null
  BIN="$(ls "$TMPDIR"/*.bin | head -1)"
  FW_HASH=$(python3 -c "
import hashlib, sys
data = open(sys.argv[1],'rb').read()
print(hashlib.sha256(data).hexdigest())
" "$BIN")
  rnodeconf --firmware-hash "$FW_HASH" "$PORT"
fi

# ── done ─────────────────────────────────────────────────────────────────────
echo
info "Firmware installed successfully."
echo
echo "  To configure as standalone transport node (TNC mode):"
echo "    rnodeconf $PORT --tnc --freq 869525000 --bw 125000 --sf 8 --cr 5 --txp 14"
echo
echo "  To configure as host-controlled modem (normal mode):"
echo "    rnodeconf $PORT --normal"
echo
echo "  Verify with:"
echo "    rnodeconf $PORT --info"
