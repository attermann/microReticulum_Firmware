#pragma once

#include <string>
#include <cstdint>

#ifdef ARDUINO
#include <WiFi.h>
#include <IPv6Address.h>
#else
#include <netinet/in.h>
#include <arpa/inet.h>
#endif

// Lightweight peer info holder for AutoInterface
// Not a full InterfaceImpl - just holds address and timing info
struct AutoInterfacePeer {
    // IPv6 address storage
#ifdef ARDUINO
    IPv6Address address;  // Must use IPv6Address, not IPAddress (which is IPv4-only!)
#else
    struct in6_addr address;
#endif
    uint16_t data_port;
    double last_heard;      // Timestamp of last activity
    double last_outbound;   // Timestamp of last reverse peering sent
    bool is_local;          // True if this is our own announcement (to ignore)

    AutoInterfacePeer() : data_port(0), last_heard(0), last_outbound(0), is_local(false) {
#ifndef ARDUINO
        memset(&address, 0, sizeof(address));
#endif
    }

#ifdef ARDUINO
    AutoInterfacePeer(const IPv6Address& addr, uint16_t port, double time, bool local = false)
        : address(addr), data_port(port), last_heard(time), last_outbound(0), is_local(local) {}
#else
    AutoInterfacePeer(const struct in6_addr& addr, uint16_t port, double time, bool local = false)
        : address(addr), data_port(port), last_heard(time), last_outbound(0), is_local(local) {}
#endif

    // Get string representation of address for logging
    std::string address_string() const {
#ifdef ARDUINO
        // ESP32 IPAddress.toString() is IPv4-only, need manual IPv6 formatting
        char buf[64];
        // Read 16 bytes from IPAddress
        uint8_t addr[16];
        for (int i = 0; i < 16; i++) {
            addr[i] = address[i];
        }
        snprintf(buf, sizeof(buf), "%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x",
                 addr[0], addr[1], addr[2], addr[3], addr[4], addr[5], addr[6], addr[7],
                 addr[8], addr[9], addr[10], addr[11], addr[12], addr[13], addr[14], addr[15]);
        return std::string(buf);
#else
        char buf[INET6_ADDRSTRLEN];
        inet_ntop(AF_INET6, &address, buf, sizeof(buf));
        return std::string(buf);
#endif
    }

    // Check if two peers have the same address
#ifdef ARDUINO
    bool same_address(const IPv6Address& other) const {
        // Use IPv6Address == operator (properly compares all 16 bytes)
        return address == other;
    }
#else
    bool same_address(const struct in6_addr& other) const {
        return memcmp(&address, &other, sizeof(address)) == 0;
    }
#endif
};
