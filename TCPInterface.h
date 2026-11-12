// Copyright (C) 2026, Chad Attermann and Gene Michael Lauria

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

#pragma once

// TCPInterface — carries Reticulum over TCP/WiFi using the HDLC-style
// framing spoken by Python RNS TCPServerInterface/TCPClientInterface.
//
// Ported from the rns_gateway MeshCore firmware (examples/rns_gateway/
// TcpInterface.h), itself derived from RTNode-HeltecV4. Two behaviours from
// that lineage are load-bearing and must not be "cleaned up":
//
//   _FIXED_MTU = true   Transport only clamps link MTU on forwarded
//                       LINKREQUEST packets for interfaces that declare a
//                       fixed MTU. Without it clamping is skipped entirely.
//   Oversized frames    A frame that overruns the rx buffer is dropped whole,
//                       never delivered truncated. A truncated frame handed
//                       to Transport as if complete corrupts resource
//                       segments and stalls the transfer silently.
//
// Client mode (default) dials out to an rnsd TCPServerInterface with DNS
// caching and exponential backoff. Server mode listens for inbound clients.
// Both share the same per-slot deframer; client mode simply uses slot 0.
//
// Lifecycle follows the UDP interface: the object is created and registered
// in RNode_Firmware.ino under TCP_TRANSPORT, Remote.h calls start()/stop()
// from wifi_remote_start(), and update_wifi() services loop().

#include <microReticulum.h>
#include "Provisioning.h"   // TCP_MODE_CLIENT / TCP_MODE_SERVER

#include <WiFi.h>
#include <lwip/sockets.h>   // SO_LINGER — force RST to free lwIP PCBs immediately

// Compile-time defaults; both are overridable at runtime through Provisioning.
// TCP_HOST is empty by default so the client stays idle until provisioned.
#ifndef TCP_PORT
  #define TCP_PORT 4242
#endif
#ifndef TCP_HOST
  #define TCP_HOST ""
#endif
#ifndef TCP_MAX_CLIENTS
  // Server mode only. Each slot costs a TCP_HW_MTU rx buffer plus lwIP's
  // per-PCB send/receive buffers.
  #define TCP_MAX_CLIENTS 4
#endif
#define TCP_HW_MTU            1064
#define TCP_CONNECT_TIMEOUT   6000    // ms — well inside WDT_TIMEOUT
#define TCP_WRITE_TIMEOUT     2000    // ms
// Idle reaper for peers that vanished without a FIN. Must outlast a healthy
// peer's longest silence: some clients (Columba on iOS) send nothing when
// idle. 10 min reclaims ghosts while leaving quiet-but-live peers alone.
#define TCP_READ_TIMEOUT      600000  // ms
#define TCP_RECONNECT_MIN     10000   // ms — initial reconnect interval
#define TCP_RECONNECT_MAX     120000  // ms — max backoff
#define TCP_KEEPALIVE_INTERVAL 30000  // ms — empty HDLC frames keep the link alive

// HDLC-like framing (matches Python RNS TCP interfaces)
#ifndef HDLC_FLAG
  #define HDLC_FLAG     0x7E
#endif
#ifndef HDLC_ESC
  #define HDLC_ESC      0x7D
#endif
#ifndef HDLC_ESC_MASK
  #define HDLC_ESC_MASK 0x20
#endif

extern bool wifi_initialized;

// Runtime configuration, persisted through Provisioning (PROV_NS_NETWORK).
// Read by start(), not the constructor, so values loaded by
// init_provisioning() after the object exists still take effect.
uint8_t  tcp_mode = TCP_MODE_CLIENT;
char     tcp_host[64] = TCP_HOST;
uint16_t tcp_port = TCP_PORT;

class TCPInterface : public RNS::InterfaceImpl {
public:
	TCPInterface(const char *name) : RNS::InterfaceImpl(name) {
		_IN = true;
		_OUT = true;
		_HW_MTU = TCP_HW_MTU;
		_FIXED_MTU = true;
		// Realistic bitrate so Transport prefers this path over LoRa when
		// both reach a destination; announce_cap keeps announce flooding
		// in check.
		_bitrate = 10000000;
		_announce_cap = 2.0;
		for (int i = 0; i < TCP_MAX_CLIENTS; i++) {
			_reset_slot(i);
		}
	}
	TCPInterface() : TCPInterface("TCPInterface") {}
	virtual ~TCPInterface() {
		stop();
		_name = "deleted";
	}

	// Cumulative frames across the TCP boundary. A peer's packet can arrive
	// here and then die silently inside Transport; these counters are the
	// only record that it existed.
	uint32_t rx_frames() const { return _rx_frames; }
	uint32_t tx_frames() const { return _tx_frames; }
	int peer_count() const { return _num_clients; }
	bool connected() const { return _num_clients > 0; }
	bool started() const { return _started; }
	uint8_t mode_config() const { return _mode; }

	virtual bool start() {
		if (_started) return true;
		_mode = tcp_mode;
		_port = tcp_port;
		strncpy(_target_host, tcp_host, sizeof(_target_host) - 1);
		_target_host[sizeof(_target_host) - 1] = '\0';

		if (_mode == TCP_MODE_SERVER) {
			_server = new WiFiServer(_port, TCP_MAX_CLIENTS);
			_server->begin();
			_server->setNoDelay(true);
			INFOF("TCPInterface: server listening on port %u (max %d peers)", _port, TCP_MAX_CLIENTS);
			_started = true;
		} else {
			if (_target_host[0] == '\0') {
				INFO("TCPInterface: no target host configured, client mode idle");
			}
			_started = true;
			_connect_client();
		}
		_online = _started;
		return _started;
	}

	virtual void stop() {
		for (int i = 0; i < TCP_MAX_CLIENTS; i++) {
			if (_clients[i].active) _cleanup_client(i, "stopped");
		}
		if (_server) {
			_server->end();
			delete _server;
			_server = nullptr;
		}
		_started = false;
		_online = false;
		_num_clients = 0;
	}

	virtual void loop() {
		if (!_started) return;

		if (_mode == TCP_MODE_SERVER && _server) {
			WiFiClient newClient = _server->available();
			if (newClient) _accept_client(newClient);
		}

		if (_mode == TCP_MODE_CLIENT && _num_clients == 0 && _target_host[0] != '\0') {
			uint32_t now = millis();
			if (now - _last_reconnect >= _reconnect_interval) {
				if (WiFi.status() == WL_CONNECTED) {
					_connect_client();
				} else {
					// WiFi down — skip the TCP attempt, just re-arm
					_last_reconnect = now;
				}
			}
		}

		if (_num_clients > 0) {
			uint32_t now = millis();
			if (now - _last_keepalive >= TCP_KEEPALIVE_INTERVAL) {
				_last_keepalive = now;
				uint8_t ka[] = { HDLC_FLAG, HDLC_FLAG };
				for (int i = 0; i < TCP_MAX_CLIENTS; i++) {
					if (_clients[i].active && _clients[i].client.connected()) {
						_clients[i].client.write(ka, 2);
					}
				}
			}
		}

		for (int i = 0; i < TCP_MAX_CLIENTS; i++) {
			if (!_clients[i].active) continue;

			if (!_clients[i].client.connected()) {
				_cleanup_client(i, "disconnected");
				continue;
			}
			if (_clients[i].last_activity > 0 &&
			    (millis() - _clients[i].last_activity) > TCP_READ_TIMEOUT) {
				_cleanup_client(i, "read timeout");
				continue;
			}
			while (_clients[i].client.available()) {
				uint8_t byte = _clients[i].client.read();
				_clients[i].last_activity = millis();
				_hdlc_deframe(i, byte);
			}
		}
	}

	virtual inline std::string toString() const {
		return "TCPInterface[" + _name + "/" +
		       (_mode == TCP_MODE_SERVER ? ":" + std::to_string(_port)
		                                 : std::string(_target_host) + ":" + std::to_string(_port)) + "]";
	}

protected:
	virtual void handle_incoming(const RNS::Bytes& data) {
    TRACEF("TCPInterface.handle_incoming: (%u bytes) data: %s", data.size(), data.toHex().c_str());
    TRACE("TCPInterface.handle_incoming: sending packet to rns...");
    try {
      InterfaceImpl::handle_incoming(data);
    }
    catch (const std::bad_alloc&) {
      ERROR("TCPInterface::handle_incoming: bad_alloc - out of memory");
    }
    catch (std::exception& e) {
      ERRORF("TCPInterface::handle_incoming: %s", e.what());
    }
  }

	virtual bool send_outgoing(const RNS::Bytes& data) {
    bool success = false;
    try {
      if (_started && _num_clients > 0 && wifi_initialized) {
        TRACEF("TCPInterface.send_outgoing: (%u bytes) data: %s", data.size(), data.toHex().c_str());

        // HDLC frame the data into the member buffer (worst case every byte
        // escaped + 2 flags) rather than 2 KB of loop-task stack under
        // Transport's call chain.
        uint8_t* frame_buf = _frame_buf;
        uint16_t flen = 0;
        frame_buf[flen++] = HDLC_FLAG;
        for (size_t i = 0; i < data.size(); i++) {
          uint8_t b = data.data()[i];
          if (b == HDLC_FLAG || b == HDLC_ESC) {
            frame_buf[flen++] = HDLC_ESC;
            frame_buf[flen++] = b ^ HDLC_ESC_MASK;
          } else {
            frame_buf[flen++] = b;
          }
          if (flen >= sizeof(_frame_buf) - 4) break;
        }
        frame_buf[flen++] = HDLC_FLAG;

        // Send to every peer EXCEPT the one that delivered this packet.
        // Transport forwarding a packet received from peer N back to peer N
        // floods TCP buffers and stalls resource transfers.
        for (int i = 0; i < TCP_MAX_CLIENTS; i++) {
          if (i == _last_rx_client_idx) continue;
          if (_clients[i].active && _clients[i].client.connected()) {
            size_t written = _clients[i].client.write(frame_buf, flen);
            if (written == 0) {
              _cleanup_client(i, "write failed");
            } else {
              success = true;
              if (written < flen) {
                WARNINGF("TCPInterface: partial write to peer %d: %u/%u bytes", i, (unsigned)written, (unsigned)flen);
              }
            }
          }
        }
        yield();
        if (success) _tx_frames++;
      }
      // Perform post-send housekeeping
      InterfaceImpl::handle_outgoing(data);
    }
    catch (const std::bad_alloc&) {
      ERROR("TCPInterface::send_outgoing: bad_alloc - out of memory");
      success = false;
    }
    catch (std::exception& e) {
      ERRORF("TCPInterface::send_outgoing: %s", e.what());
      success = false;
    }
    return success;
  }

private:
	struct Peer {
		WiFiClient client;
		uint32_t   last_activity;
		bool       active;
		// HDLC deframe state
		bool       in_frame;
		bool       escape;
		bool       truncated;
		uint8_t    rxbuf[TCP_HW_MTU];
		uint16_t   rxlen;
	};

	void _reset_slot(int idx) {
		Peer& c = _clients[idx];
		c.active = false;
		c.in_frame = false;
		c.escape = false;
		c.truncated = false;
		c.rxlen = 0;
		c.last_activity = 0;
	}

	static void _force_rst(WiFiClient& client) {
		// SO_LINGER with timeout 0 forces RST instead of FIN, skipping
		// TIME_WAIT and immediately freeing the lwIP PCB and its buffers.
		int fd = client.fd();
		if (fd >= 0) {
			struct linger lin;
			lin.l_onoff = 1;
			lin.l_linger = 0;
			setsockopt(fd, SOL_SOCKET, SO_LINGER, &lin, sizeof(lin));
		}
	}

	void _cleanup_client(int idx, const char* reason) {
		Peer& c = _clients[idx];
		if (!c.active) return;
		_force_rst(c.client);
		c.client.stop();
		c.client = WiFiClient();  // release residual shared state
		_reset_slot(idx);
		_num_clients--;
		INFOF("TCPInterface: peer %d %s (free heap %u)", idx, reason, ESP.getFreeHeap());
	}

	void _hdlc_deframe(int idx, uint8_t byte) {
		Peer& c = _clients[idx];

		if (byte == HDLC_FLAG) {
			if (c.in_frame && c.rxlen > 0) {
				if (c.truncated) {
					// Drop the whole frame; never deliver a truncated head.
					WARNINGF("TCPInterface: dropped oversized frame from peer %d (>%d bytes)", idx, TCP_HW_MTU);
				} else {
					// _last_rx_client_idx lets send_outgoing() skip echoing this
					// packet to its sender. The chain handle_incoming → Transport
					// → send_outgoing is synchronous, so the scoped set/clear is safe.
					RNS::Bytes data(c.rxbuf, c.rxlen);
					_rx_frames++;
					_last_rx_client_idx = idx;
					handle_incoming(data);
					_last_rx_client_idx = -1;
				}
			}
			c.in_frame = true;
			c.escape = false;
			c.truncated = false;
			c.rxlen = 0;
		} else if (c.in_frame) {
			if (c.escape) {
				byte ^= HDLC_ESC_MASK;
				c.escape = false;
				if (c.rxlen < TCP_HW_MTU) c.rxbuf[c.rxlen++] = byte; else c.truncated = true;
			} else if (byte == HDLC_ESC) {
				c.escape = true;
			} else {
				if (c.rxlen < TCP_HW_MTU) c.rxbuf[c.rxlen++] = byte; else c.truncated = true;
			}
		}
	}

	void _bind_slot(int idx, WiFiClient& client) {
		// Defensive: release any residual lwIP resources in this slot first.
		_force_rst(_clients[idx].client);
		_clients[idx].client.stop();
		_clients[idx].client = WiFiClient();

		_clients[idx].client = client;
		_clients[idx].client.setNoDelay(true);
		_clients[idx].client.setTimeout(TCP_WRITE_TIMEOUT / 1000);
		_reset_slot(idx);
		_clients[idx].active = true;
		_clients[idx].last_activity = millis();
		_num_clients++;
	}

	void _accept_client(WiFiClient& newClient) {
		for (int i = 0; i < TCP_MAX_CLIENTS; i++) {
			if (!_clients[i].active) {
				_bind_slot(i, newClient);
				INFOF("TCPInterface: peer %d connected from %s", i, _clients[i].client.remoteIP().toString().c_str());
				return;
			}
		}
		WARNING("TCPInterface: max peers reached, rejecting connection");
		newClient.stop();
	}

	void _connect_client() {
		if (_target_host[0] == '\0') return;

		WiFiClient client;
		client.setTimeout(TCP_CONNECT_TIMEOUT / 1000);
		bool connected = false;

		// Cached IP first — avoids a DNS lookup on every reconnect.
		if (_resolved_ip != (uint32_t)0) {
			INFOF("TCPInterface: connecting to %s:%u (cached IP)", _target_host, _port);
			connected = client.connect(_resolved_ip, _port);
			if (!connected) _resolved_ip = (uint32_t)0;
		}
		if (!connected) {
			IPAddress resolved;
			if (WiFi.hostByName(_target_host, resolved)) {
				_resolved_ip = resolved;
				INFOF("TCPInterface: connecting to %s (%s):%u", _target_host, resolved.toString().c_str(), _port);
				connected = client.connect(resolved, _port);
			} else {
				WARNINGF("TCPInterface: DNS failed for %s", _target_host);
			}
		}

		if (connected) {
			_bind_slot(0, client);
			_consecutive_failures = 0;
			_reconnect_interval = TCP_RECONNECT_MIN;
			INFOF("TCPInterface: connected to %s:%u", _target_host, _port);
		} else {
			_consecutive_failures++;
			// Exponential backoff: 10s -> 20s -> 40s -> 80s -> 120s (max)
			_reconnect_interval = _reconnect_interval * 2;
			if (_reconnect_interval > TCP_RECONNECT_MAX) _reconnect_interval = TCP_RECONNECT_MAX;
			WARNINGF("TCPInterface: connect to %s:%u failed (attempt %u, retry in %us)",
			         _target_host, _port, _consecutive_failures, _reconnect_interval / 1000);
		}
		_last_reconnect = millis();
	}

	uint8_t     _mode = TCP_MODE_CLIENT;
	uint16_t    _port = TCP_PORT;
	char        _target_host[64] = "";
	WiFiServer* _server = nullptr;
	Peer        _clients[TCP_MAX_CLIENTS];
	uint8_t     _frame_buf[TCP_HW_MTU * 2 + 4];
	int         _num_clients = 0;
	uint32_t    _last_reconnect = 0;
	uint32_t    _last_keepalive = 0;
	uint32_t    _reconnect_interval = TCP_RECONNECT_MIN;
	IPAddress   _resolved_ip = IPAddress((uint32_t)0);
	uint16_t    _consecutive_failures = 0;
	bool        _started = false;
	int         _last_rx_client_idx = -1;
	uint32_t    _rx_frames = 0;
	uint32_t    _tx_frames = 0;
};

// Raw pointer to the live impl, set by RNode_Firmware.ino when the interface
// is created. RNS::Interface hides the impl, and Provisioning.cpp / Pages.h
// only see a forward declaration, so they read counters through these.
TCPInterface* tcp_impl = nullptr;
int      tcp_peer_count() { return tcp_impl ? tcp_impl->peer_count() : 0; }
uint32_t tcp_rx_frames()  { return tcp_impl ? tcp_impl->rx_frames()  : 0; }
uint32_t tcp_tx_frames()  { return tcp_impl ? tcp_impl->tx_frames()  : 0; }
