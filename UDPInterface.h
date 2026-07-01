// Copyright (C) 2026, Chad Attermann

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

#include <microReticulum.h>

#include <WiFi.h>
#include <WiFiUdp.h>

#define UDP_LOCAL_HOST "0.0.0.0"
#define UDP_REMOTE_HOST "255.255.255.255"
#define UDP_PORT 4242

//#include "Remote.h"
extern WiFiUDP udp;
extern bool wifi_initialized;

uint16_t udp_port = UDP_PORT;

class UDPInterface : public RNS::InterfaceImpl {
public:
	UDPInterface(const char *name) : RNS::InterfaceImpl(name) {
		_IN = true;
		_OUT = true;
		_HW_MTU = 1064;
	}
	UDPInterface() : UDPInterface("UDPInterface") {}
	virtual ~UDPInterface() {
		_name = "deleted";
	}
protected:
	virtual void handle_incoming(const RNS::Bytes& data) {
    TRACEF("UDPInterface.handle_incoming: (%u bytes) data: %s", data.size(), data.toHex().c_str());
    TRACE("UDPInterface.handle_incoming: sending packet to rns...");
    try {
      InterfaceImpl::handle_incoming(data);
    }
    catch (const std::bad_alloc&) {
      ERROR("UDPInterface::handle_incoming: bad_alloc - out of memory");
    }
    catch (std::exception& e) {
      ERRORF("UDPInterface::handle_incoming: %s", e.what());
    }
  }
	virtual bool send_outgoing(const RNS::Bytes& data) {
    bool success = true;
    try {
      //if (udp.availableForWrite()) {
      //wl_status_t wifi_status = WiFi.status();
      //if (wifi_status == WL_CONNECTED) {
      if (wifi_initialized) {
        TRACEF("UDPInterface.send_outgoing: (%u bytes) data: %s", data.size(), data.toHex().c_str());
        if (udp.beginPacket(UDP_REMOTE_HOST, udp_port) != 0) {
          size_t wrote = udp.write(data.data(), data.size());
          udp.endPacket();
          if (wrote != data.size()) {
            WARNINGF("Failed to send %u packet over UDPInterface", wrote);
            success = false;
          }
        }
      }
      // Perform post-send housekeeping
      InterfaceImpl::handle_outgoing(data);
    }
    catch (const std::bad_alloc&) {
      ERROR("UDPInterface::send_outgoing: bad_alloc - out of memory");
      success = false;
    }
    catch (std::exception& e) {
      ERRORF("UDPInterface::send_outgoing: %s", e.what());
      success = false;
    }
    return success;
  }
};
