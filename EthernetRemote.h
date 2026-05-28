// Copyright (C) 2024, Mark Qvist

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include <RAK13800_W5100S.h>
#include <SPI.h>

#define ETH_TCP_PORT           7633
#define ETH_READ_TIMEOUT_MS    6500
#define ETH_CONNECT_TIMEOUT_MS 15000
#define ETH_MAC_CHECK_MS       5000
#define ETH_RETRY_MS           30000
#define ETH_MAINTAIN_MS        1000

uint32_t eth_last_read      = 0;
uint32_t eth_connect_time   = 0;
uint32_t eth_last_mac_check = 0;
uint32_t eth_last_retry     = 0;
uint32_t eth_last_maintain  = 0;
bool     eth_initialized    = false;
bool     eth_session_started = false;
bool     eth_hw_detected    = false; // W5100S confirmed present at least once
bool     eth_hw_absent      = false; // W5100S confirmed absent; skip all retries

// The board's default SPI (NRF_SPIM3) is on the IO slot pins (P0.03/P0.29/P0.30),
// which are exactly the W5100S pins.  The SX1262 modem uses spiModem (NRF_SPIM2)
// on the Core slot pins (P1.11–P1.13) — different SPIM instance, no conflict.

EthernetServer eth_listener(ETH_TCP_PORT);
EthernetClient eth_connection;

uint8_t eth_mac[6];

extern void host_disconnected();
extern volatile uint8_t queue_height; // pending TX packets (RNode_Firmware.ino)
extern bool dcd;                       // carrier detect / RX in progress (Config.h)

static void eth_get_mac(uint8_t *mac) {
  // Derive a stable, unique MAC from the nRF52840 FICR device address
  // (the same 64-bit value the BLE stack uses for its public address).
  uint32_t addr0 = NRF_FICR->DEVICEADDR[0];
  uint32_t addr1 = NRF_FICR->DEVICEADDR[1];
  mac[0] = (addr1 >>  8) & 0xFF;
  mac[1] =  addr1        & 0xFF;
  mac[2] = (addr0 >> 24) & 0xFF;
  mac[3] = (addr0 >> 16) & 0xFF;
  mac[4] = (addr0 >>  8) & 0xFF;
  mac[5] =  addr0        & 0xFF;
  mac[0] &= 0xFE; // unicast
  mac[0] |= 0x02; // locally administered
}

bool eth_host_is_connected() { return (bool)eth_connection; }

static void eth_close_all() {
  if (eth_connection) { eth_connection.stop(); }
  EthernetClient stale = eth_listener.available();
  while (stale) { stale.stop(); stale = eth_listener.available(); }
}

static void eth_disconnect() {
  eth_session_started = false;
  eth_close_all();
  host_disconnected();
}

static void eth_check_timeout() {
  if (!eth_session_started) {
    if (millis() - eth_connect_time >= ETH_CONNECT_TIMEOUT_MS) { eth_disconnect(); }
  } else {
    if (millis() - eth_last_read   >= ETH_READ_TIMEOUT_MS)     { eth_disconnect(); }
  }
}

bool eth_remote_available() {
  if (!eth_initialized) { return false; }
  if (eth_connection) {
    if (eth_connection.connected()) {
      if (eth_connection.available()) { eth_last_read = millis(); eth_session_started = true; return true; }
      else { eth_check_timeout(); return false; }
    } else {
      eth_disconnect();
      return false;
    }
  } else {
    EthernetClient client = eth_listener.available();
    if (!client) { return false; }
    eth_connection      = client;
    eth_connect_time    = millis();
    eth_session_started = false;
    cable_state         = CABLE_STATE_CONNECTED;
    return eth_connection.available();
  }
}

uint8_t eth_remote_read() {
  if (eth_connection && eth_connection.available()) { return eth_connection.read(); }
  if (eth_connection) { eth_disconnect(); }
  return 0xC0; // FEND — safe frame delimiter, acts as a no-op
}

void eth_remote_write(uint8_t byte) {
  if (eth_connection) { eth_connection.write(byte); }
}

static void eth_hw_reset() {
  pinMode(ETH_RST_PIN, OUTPUT);
  digitalWrite(ETH_RST_PIN, LOW);  delay(100);
  digitalWrite(ETH_RST_PIN, HIGH); delay(100);
}

static bool eth_start() {
  if (eth_hw_absent) { return false; }
  SPI.begin();
  Ethernet.init(SPI, ETH_CS_PIN);
  // If hardware was seen before, check link before attempting DHCP — avoids
  // the full 8s timeout every 30s when the cable is simply unplugged.
  if (eth_hw_detected && Ethernet.linkStatus() != LinkON) { return false; }
  int status = Ethernet.begin(eth_mac, 3000, 1500);
  if (status == 0) {
    if (Ethernet.hardwareStatus() == EthernetNoHardware) { eth_hw_absent = true; }
    else                                                  { eth_hw_detected = true; }
    return false;
  }
  eth_hw_detected = true;
  eth_listener.begin();
  return true;
}

// Detect W5100S brownout: PoE power instability can reset the chip while the
// MCU keeps running, reverting all registers (including the MAC) to defaults.
static void eth_check_chip() {
  uint8_t current_mac[6];
  Ethernet.MACAddress(current_mac);
  if (memcmp(current_mac, eth_mac, 6) != 0) {
    if (eth_connection) { eth_disconnect(); }
    eth_hw_reset();
    // Clear eth_hw_detected so eth_start() skips the linkStatus gate — PHY
    // needs time to auto-negotiate after a hardware reset and would return
    // LinkOFF too soon, causing DHCP to be skipped.
    eth_hw_detected = false;
    if (eth_start()) {
      eth_last_mac_check = millis();
    } else {
      eth_initialized = false;
      eth_last_retry  = millis();
    }
  }
}

void update_eth() {
  if (!eth_initialized) {
    if (millis() - eth_last_retry >= ETH_RETRY_MS) {
      eth_last_retry = millis();
      if (eth_start()) { eth_initialized = true; eth_last_mac_check = millis(); }
    }
    return;
  }

  // Renew the DHCP lease, but only during a radio-quiet gap (no carrier, no queued
  // TX) so the blocking call can't interrupt an in-flight packet. checkLease()
  // advances by elapsed time, so a renewal that comes due while busy just fires at
  // the next quiet gap; the W5100S holds the IP until then. A normal renewal is
  // sub-second — the 3 s begin() timeout only bites an unresponsive server.
  if (!dcd && queue_height == 0 && millis() - eth_last_maintain >= ETH_MAINTAIN_MS) {
    eth_last_maintain = millis();
    Ethernet.maintain();
  }

  if (millis() - eth_last_mac_check >= ETH_MAC_CHECK_MS) {
    eth_last_mac_check = millis();
    eth_check_chip();
  }
}

bool eth_init() {
  eth_get_mac(eth_mac);
  eth_hw_reset();

  if (!eth_start()) {
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
      Serial.write("ETH: W5100S not found\r\n");
    } else if (Ethernet.linkStatus() == LinkOFF) {
      Serial.write("ETH: cable not connected, will retry\r\n");
    } else {
      Serial.write("ETH: DHCP failed, will retry\r\n");
    }
    eth_last_retry = millis();
    return false;
  }

  eth_initialized    = true;
  eth_last_mac_check = millis();
  return true;
}
