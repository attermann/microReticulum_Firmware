#include <Reticulum.h>
#include <Interface.h>
#include <Log.h>
#include <Bytes.h>

#include <WiFi.h>
#include <WiFiUdp.h>

#define UDP_LOCAL_HOST "0.0.0.0"
#define UDP_REMOTE_HOST "255.255.255.255"
#define UDP_PORT 4242

//#include "Remote.h"
extern WiFiUDP udp;
extern bool wifi_initialized;

#if defined(HAS_RNS) && defined(UDP_TRANSPORT)
// CBA UDP interface
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
        if (udp.beginPacket(UDP_REMOTE_HOST, UDP_PORT) != 0) {
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
#endif