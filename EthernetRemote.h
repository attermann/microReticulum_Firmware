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

// KISS-over-TCP host transport on a WIZnet W5100S (RAK13800 WisBlock
// Ethernet module). Mirrors the WiFi remote in Remote.h: one host at a
// time on TCP port 7633, idle connections are dropped, and losing the host
// stops the radio via host_disconnected().
//
// Bus layout on the RAK4631: the W5100S sits on the board-default SPI
// (SPIM3, IO-slot pins) behind the shared IO-slot chip select, while the
// SX1262 uses spiModem (SPIM2) on the core-slot pins. The two buses never
// contend, so the modem ISR is unaffected by Ethernet traffic.
//
// The module is detected at runtime so a single firmware image serves
// boards with and without it. Every W5100S library call that goes through
// W5100Class::init() re-runs the 560 ms chip probe until a chip has been
// found, so once the module is known to be absent nothing here touches the
// library again.

#include <RAK13800_W5100S.h>
#include <SPI.h>

#define ETH_TCP_PORT           7633
#define ETH_READ_TIMEOUT_MS    6500   // same idle window as the WiFi remote (WR_READ_TIMEOUT_MS)
#define ETH_DHCP_TIMEOUT_MS    3000   // Ethernet.begin() blocks the main loop for at most this long
#define ETH_DHCP_RESPONSE_MS   1500
#define ETH_RETRY_MS           30000  // between DHCP attempts while the link is down or unconfigured
#define ETH_MAINTAIN_MS        1000   // DHCP lease bookkeeping cadence
#define ETH_CHIP_CHECK_MS      5000   // brownout detection cadence
#define ETH_ACCEPT_POLL_MS     20     // listener poll cadence while no host is attached
#define ETH_RX_POLL_MS         2      // socket poll cadence while the local RX buffer is empty
#define ETH_TX_FLUSH_MS        20     // flush partial output that is not FEND-terminated
#define ETH_RX_BUF_SIZE        512
#define ETH_TX_BUF_SIZE        1024   // holds a fully escaped maximum-length KISS frame
#define ETH_RAIL_SETTLE_MS     100    // 3V3_S switch-on to first reset pulse
#define ETH_PROBE_ATTEMPTS     3      // failed chip probes before the module is declared absent

uint32_t eth_last_read        = 0;
uint32_t eth_last_poll        = 0;
uint32_t eth_last_retry       = 0;
uint32_t eth_last_maintain    = 0;
uint32_t eth_last_chip_check  = 0;
bool     eth_initialized      = false;
bool     eth_hw_detected      = false; // W5100S answered at least once
bool     eth_hw_absent        = false; // W5100S confirmed absent; never touch the library again
uint8_t  eth_probe_failures   = 0;

// Bytes read from the socket in bulk and handed out one at a time by
// eth_remote_read(). Refilled only when empty, so a plain linear buffer
// is enough.
static uint8_t  eth_rx_buf[ETH_RX_BUF_SIZE];
static uint16_t eth_rx_pos = 0;
static uint16_t eth_rx_len = 0;

// Outbound bytes are coalesced into one TCP segment per KISS frame. The
// W5100S issues a SEND command per write() call and waits for the peer's
// ACK, so writing byte-by-byte would cost one round trip per byte.
static uint8_t  eth_tx_buf[ETH_TX_BUF_SIZE];
static uint16_t eth_tx_len  = 0;
static uint32_t eth_tx_last = 0;
static uint32_t eth_tx_dropped_frames = 0;

EthernetServer eth_listener(ETH_TCP_PORT);
EthernetClient eth_connection;

uint8_t eth_mac[6];

extern void host_disconnected();
extern volatile uint8_t queue_height;  // pending TX packets, RNode_Firmware.ino
extern bool spi_ss_in_use;             // shared IO-slot CS already claimed by an external flash

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

// Radio-quiet gap: no carrier and nothing queued for transmission. The
// blocking library calls (DHCP, chip re-init) are only allowed here so they
// cannot interrupt a packet in flight.
static bool eth_radio_quiet() { return !dcd && queue_height == 0; }

bool eth_host_is_connected() { return (bool)eth_connection; }

// True while bytes read from the socket are still waiting in the RX buffer.
bool eth_remote_pending() { return eth_rx_len > 0; }

static void eth_flush_tx() {
  if (eth_tx_len == 0) { return; }
  if (eth_connection) {
    // socketSend() spins until the chip has room for the whole write. A
    // peer that stopped reading would stall the radio loop, so drop the
    // frame instead; the idle timeout will retire the connection.
    if (eth_connection.availableForWrite() >= (int)eth_tx_len) { eth_connection.write(eth_tx_buf, eth_tx_len); }
    else                                                        { eth_tx_dropped_frames++; }
  }
  eth_tx_len = 0;
}

static void eth_close_all() {
  eth_tx_len = 0;
  eth_rx_pos = 0;
  eth_rx_len = 0;
  if (eth_connection) { eth_connection.stop(); }
  EthernetClient stale = eth_listener.available();
  while (stale) { stale.stop(); stale = eth_listener.available(); }
}

static void eth_disconnect() {
  eth_close_all();
  host_disconnected();
}

// Pull whatever the socket holds (up to the buffer size) in one transfer.
static bool eth_fill_rx() {
  int avail = eth_connection.available();
  if (avail <= 0) { return false; }
  if (avail > ETH_RX_BUF_SIZE) { avail = ETH_RX_BUF_SIZE; }
  int n = eth_connection.read(eth_rx_buf, avail);
  if (n <= 0) { return false; }
  eth_rx_pos    = 0;
  eth_rx_len    = n;
  eth_last_read = millis();
  return true;
}

bool eth_remote_available() {
  if (!eth_initialized) { return false; }
  if (eth_rx_len > 0)   { return true; }

  uint32_t now = millis();
  if (eth_connection) {
    if (now - eth_last_poll < ETH_RX_POLL_MS) { return false; }
    eth_last_poll = now;
    // connected() stays true for a half-closed socket that still holds
    // unread data, so the tail of a closing stream is not lost.
    if (!eth_connection.connected()) { eth_disconnect(); return false; }
    if (eth_fill_rx()) { return true; }
    if (now - eth_last_read >= ETH_READ_TIMEOUT_MS) { eth_disconnect(); }
    return false;
  }

  if (now - eth_last_poll < ETH_ACCEPT_POLL_MS) { return false; }
  eth_last_poll = now;
  // available() only hands out a client once it has sent something, so a
  // freshly accepted connection always carries data.
  EthernetClient client = eth_listener.available();
  if (!client) { return false; }
  eth_connection = client;
  eth_last_read  = now;
  cable_state    = CABLE_STATE_CONNECTED;
  return eth_fill_rx();
}

uint8_t eth_remote_read() {
  if (eth_rx_len > 0) {
    uint8_t byte = eth_rx_buf[eth_rx_pos++];
    eth_rx_len--;
    return byte;
  }
  if (eth_connection) { eth_disconnect(); }
  return FEND; // frame delimiter, harmless to the KISS parser
}

void eth_remote_write(uint8_t byte) {
  if (!eth_connection) { eth_tx_len = 0; return; }
  eth_tx_buf[eth_tx_len++] = byte;
  eth_tx_last = millis();
  // A KISS frame is FEND ... FEND: flush when the closing delimiter lands
  // (the buffer then holds more than the opening one) or when full.
  if ((byte == FEND && eth_tx_len > 1) || eth_tx_len >= ETH_TX_BUF_SIZE) { eth_flush_tx(); }
}

void eth_power_on() {
  pinMode(ETH_PWR_PIN, OUTPUT);
  digitalWrite(ETH_PWR_PIN, HIGH);
}

static void eth_hw_reset() {
  pinMode(ETH_RST_PIN, OUTPUT);
  digitalWrite(ETH_RST_PIN, LOW);  delay(100);
  digitalWrite(ETH_RST_PIN, HIGH); delay(100);
}

static bool eth_start() {
  if (eth_hw_absent) { return false; }
  Ethernet.init(SPI, ETH_CS_PIN);
  // Once the chip is known to exist, skip DHCP while the cable is out;
  // otherwise every retry would block for the full DHCP timeout.
  if (eth_hw_detected && Ethernet.linkStatus() != LinkON) { return false; }
  int status = Ethernet.begin(eth_mac, ETH_DHCP_TIMEOUT_MS, ETH_DHCP_RESPONSE_MS);
  if (status == 0) {
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
      // A chip that was slow to come up must not be written off for good:
      // give it a few tries (30 s apart, from update_eth) before giving up.
      if (++eth_probe_failures >= ETH_PROBE_ATTEMPTS) { eth_hw_absent = true; }
    } else {
      eth_hw_detected = true;
    }
    return false;
  }
  eth_hw_detected = true;
  eth_listener.begin();
  IPAddress ip = Ethernet.localIP();
  printf("[ETH] link up, %u.%u.%u.%u:%u\n", ip[0], ip[1], ip[2], ip[3], ETH_TCP_PORT);
  return true;
}

// PoE power dips can reset the W5100S while the MCU keeps running, which
// reverts every register (including the MAC) to defaults. Re-run the whole
// bring-up when that happens.
static void eth_check_chip() {
  uint8_t current_mac[6];
  Ethernet.MACAddress(current_mac);
  if (memcmp(current_mac, eth_mac, 6) == 0) { return; }
  printf("[ETH] W5100S lost its configuration, reinitializing\n");
  if (eth_connection) { eth_disconnect(); }
  eth_hw_reset();
  // The PHY needs time to negotiate after a reset and would report LinkOFF
  // too early; clearing the flag makes eth_start() skip the link gate once.
  eth_hw_detected = false;
  if (eth_start()) {
    eth_last_chip_check = millis();
  } else {
    eth_initialized = false;
    eth_last_retry  = millis();
  }
}

void update_eth() {
  if (eth_hw_absent) { return; }
  uint32_t now = millis();

  if (!eth_initialized) {
    if (now - eth_last_retry >= ETH_RETRY_MS && eth_radio_quiet()) {
      eth_last_retry = now;
      if (!eth_hw_detected) { eth_hw_reset(); } // still probing: start each attempt from a clean chip
      if (eth_start()) { eth_initialized = true; eth_last_chip_check = now; }
    }
    return;
  }

  if (eth_tx_len > 0 && now - eth_tx_last >= ETH_TX_FLUSH_MS) { eth_flush_tx(); }

  if (eth_radio_quiet()) {
    // checkLease() counts elapsed time, so a renewal that falls due while
    // the radio is busy simply fires at the next quiet gap.
    if (now - eth_last_maintain >= ETH_MAINTAIN_MS) {
      eth_last_maintain = now;
      Ethernet.maintain();
    }
    if (now - eth_last_chip_check >= ETH_CHIP_CHECK_MS) {
      eth_last_chip_check = now;
      eth_check_chip();
    }
  }
}

// Called once at the very end of setup(). The RAK15001 probe earlier in
// setup() drives the same chip select, so the chip is reset here before
// first use. The rail itself is switched on at the top of setup(): on PoE
// the switch drops the port unless the board keeps drawing its minimum
// "maintain power signature" current, and only the W5100S gets a bare
// nRF52 with the radio off above that threshold.
bool eth_init() {
  if (spi_ss_in_use) {
    printf("[ETH] IO-slot chip select is used by external flash, Ethernet disabled\n");
    eth_hw_absent = true;
    return false;
  }

  eth_get_mac(eth_mac);
  eth_power_on(); // already on since the top of setup(); harmless to repeat
  delay(ETH_RAIL_SETTLE_MS);
  eth_hw_reset();

  if (!eth_start()) {
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
      printf("[ETH] no W5100S module found, will probe again\n");
    } else if (Ethernet.linkStatus() == LinkOFF) {
      printf("[ETH] cable not connected, will retry\n");
    } else {
      printf("[ETH] DHCP failed, will retry\n");
    }
    eth_last_retry = millis();
    return false;
  }

  eth_initialized     = true;
  eth_last_chip_check = millis();
  return true;
}
