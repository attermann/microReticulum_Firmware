#include "AutoInterface.h"
#include "Log.h"
#include "Utilities/OS.h"

#include <cstring>
#include <algorithm>

#ifdef ARDUINO
#include <Esp.h>  // For ESP.getMaxAllocHeap()
// ESP32 lwIP headers for raw socket support
#include <lwip/sockets.h>
#include <lwip/netdb.h>
#include <lwip/mld6.h>
#include <lwip/netif.h>
#include <errno.h>
#else
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>
#include <net/if.h>
#include <ifaddrs.h>
#include <fcntl.h>
#endif

using namespace RNS;

// Helper: Convert IPv6 address bytes to compressed string format (RFC 5952)
// This matches Python's inet_ntop output
static std::string ipv6_to_compressed_string(const uint8_t* addr) {
    // Build 8 groups of 16-bit values
    uint16_t groups[8];
    for (int i = 0; i < 8; i++) {
        groups[i] = (addr[i*2] << 8) | addr[i*2+1];
    }

    // Find longest run of zeros for :: compression
    int best_start = -1, best_len = 0;
    int cur_start = -1, cur_len = 0;
    for (int i = 0; i < 8; i++) {
        if (groups[i] == 0) {
            if (cur_start < 0) cur_start = i;
            cur_len++;
        } else {
            if (cur_len > best_len && cur_len > 1) {
                best_start = cur_start;
                best_len = cur_len;
            }
            cur_start = -1;
            cur_len = 0;
        }
    }
    if (cur_len > best_len && cur_len > 1) {
        best_start = cur_start;
        best_len = cur_len;
    }

    // Build string
    std::string result;
    for (int i = 0; i < 8; i++) {
        char buf[8];
        if (best_start >= 0 && i >= best_start && i < best_start + best_len) {
            if (i == best_start) result += "::";
            continue;
        }
        if (!result.empty() && result.back() != ':') result += ":";
        snprintf(buf, sizeof(buf), "%x", groups[i]);
        result += buf;
    }
    // Handle trailing ::
    if (best_start >= 0 && best_start + best_len == 8 && result.size() == 2) {
        // Just "::" for all zeros
    }
    return result;
}

AutoInterface::AutoInterface(const char* name) : InterfaceImpl(name) {
    _IN = true;
    _OUT = true;
    _bitrate = BITRATE_GUESS;
    _HW_MTU = HW_MTU;
    memset(&_multicast_address, 0, sizeof(_multicast_address));
    memset(&_link_local_address, 0, sizeof(_link_local_address));
}

AutoInterface::~AutoInterface() {
    stop();
}

bool AutoInterface::start() {
    _online = false;

    INFO("AutoInterface: Starting with group_id: " + _group_id);
    INFO("AutoInterface: Discovery port: " + std::to_string(_discovery_port));
    INFO("AutoInterface: Data port: " + std::to_string(_data_port));

#ifdef ARDUINO
    // ESP32 implementation using WiFiUDP

    // Get link-local address for our interface
    if (!get_link_local_address()) {
        ERROR("AutoInterface: Could not get link-local IPv6 address");
        return false;
    }

    // Calculate multicast address from group_id hash
    calculate_multicast_address();

    // Calculate our discovery token
    calculate_discovery_token();

    // Set up discovery socket (multicast receive)
    if (!setup_discovery_socket()) {
        ERROR("AutoInterface: Could not set up discovery socket");
        return false;
    }

    // Set up unicast discovery socket (reverse peering receive)
    if (!setup_unicast_discovery_socket()) {
        // Non-fatal - reverse peering won't work but multicast discovery will
        WARNING("AutoInterface: Could not set up unicast discovery socket (reverse peering disabled)");
    }

    // Set up data socket (unicast send/receive)
    if (!setup_data_socket()) {
        // Data socket failure is non-fatal - we can still discover peers
        WARNING("AutoInterface: Could not set up data socket (discovery-only mode)");
        _data_socket_ok = false;
    } else {
        _data_socket_ok = true;
    }

    _online = true;
    INFO("AutoInterface: Started successfully (data_socket=" + std::string(_data_socket_ok ? "yes" : "no") +
         ", unicast_discovery=" + std::string(_unicast_discovery_socket >= 0 ? "yes" : "no") + ")");
    INFO("AutoInterface: Multicast address: " + _multicast_address_str);
    INFO("AutoInterface: Link-local address: " + _link_local_address_str);
    INFO("AutoInterface: Discovery token: " + _discovery_token.toHex());

    return true;
#else
    // Get link-local address for our interface
    if (!get_link_local_address()) {
        ERROR("AutoInterface: Could not get link-local IPv6 address");
        return false;
    }

    // Calculate multicast address from group_id hash
    calculate_multicast_address();

    // Calculate our discovery token
    calculate_discovery_token();

    // Set up discovery socket (multicast receive)
    if (!setup_discovery_socket()) {
        ERROR("AutoInterface: Could not set up discovery socket");
        return false;
    }

    // Set up unicast discovery socket (reverse peering receive)
    if (!setup_unicast_discovery_socket()) {
        // Non-fatal - reverse peering won't work but multicast discovery will
        WARNING("AutoInterface: Could not set up unicast discovery socket (reverse peering disabled)");
    }

    // Set up data socket (unicast send/receive)
    if (!setup_data_socket()) {
        // Data socket failure is non-fatal - we can still discover peers
        // This happens when Python RNS is already bound to the same address:port
        WARNING("AutoInterface: Could not set up data socket (discovery-only mode)");
        WARNING("AutoInterface: Another RNS instance may be using this address");
    }

    _online = true;
    INFO("AutoInterface: Started successfully (data_socket=" +
         std::string(_data_socket >= 0 ? "yes" : "no") +
         ", unicast_discovery=" + std::string(_unicast_discovery_socket >= 0 ? "yes" : "no") + ")");
    INFO("AutoInterface: Multicast address: " + std::string(inet_ntop(AF_INET6, &_multicast_address,
        (char*)_buffer.writable(INET6_ADDRSTRLEN), INET6_ADDRSTRLEN)));
    INFO("AutoInterface: Link-local address: " + _link_local_address_str);
    INFO("AutoInterface: Discovery token: " + _discovery_token.toHex());

    return true;
#endif
}

void AutoInterface::stop() {
#ifdef ARDUINO
    // ESP32 cleanup - raw sockets for discovery, unicast discovery, and data
    if (_discovery_socket > -1) {
        close(_discovery_socket);
        _discovery_socket = -1;
    }
    if (_unicast_discovery_socket > -1) {
        close(_unicast_discovery_socket);
        _unicast_discovery_socket = -1;
    }
    if (_data_socket > -1) {
        close(_data_socket);
        _data_socket = -1;
    }
    _data_socket_ok = false;
#else
    if (_discovery_socket > -1) {
        close(_discovery_socket);
        _discovery_socket = -1;
    }
    if (_unicast_discovery_socket > -1) {
        close(_unicast_discovery_socket);
        _unicast_discovery_socket = -1;
    }
    if (_data_socket > -1) {
        close(_data_socket);
        _data_socket = -1;
    }
#endif
    _online = false;
    _peers.clear();
}

void AutoInterface::loop() {
    if (!_online) return;

    double now = Utilities::OS::time();

    // Send periodic discovery announce
    if (now - _last_announce >= ANNOUNCE_INTERVAL) {
#ifdef ARDUINO
        // Skip announce if memory is critically low - prevents fragmentation
        // Threshold lowered to 8KB since announces are small (32 byte token)
        uint32_t max_block = ESP.getMaxAllocHeap();
        if (max_block < 8000) {
            WARNING("AutoInterface: Skipping announce - low memory (max_block=" + std::to_string(max_block) + ")");
            _last_announce = now;  // Still update timer to avoid tight loop
        } else {
            send_announce();
            _last_announce = now;
        }
#else
        send_announce();
        _last_announce = now;
#endif
    }

    // Process incoming discovery packets (multicast)
    process_discovery();

    // Process incoming unicast discovery packets (reverse peering)
    process_unicast_discovery();

    // Send reverse peering to known peers
    send_reverse_peering();

    // Process incoming data packets
    process_data();

    // Check multicast echo timeout
    check_echo_timeout();

    // Expire stale peers
    expire_stale_peers();

    // Expire old deque entries
    expire_deque_entries();

    // Periodic peer job (every 4 seconds) - check for address changes
    if (now - _last_peer_job >= PEER_JOB_INTERVAL) {
        check_link_local_address();
        _last_peer_job = now;
    }
}

void AutoInterface::send_outgoing(const Bytes& data) {
    DEBUG(toString() + ".send_outgoing: data: " + data.toHex());

    if (!_online) return;

#ifdef ARDUINO
    // ESP32: Send to all known peers via unicast using persistent raw IPv6 socket
    // (WiFiUDP doesn't support IPv6)
    if (_data_socket < 0) {
        WARNING("AutoInterface: Data socket not ready, cannot send");
        return;
    }

    for (const auto& peer : _peers) {
        if (peer.is_local) continue;  // Don't send to ourselves

        sockaddr_in6 peer_addr = {};
        peer_addr.sin6_family = AF_INET6;
        peer_addr.sin6_port = htons(_data_port);
        peer_addr.sin6_scope_id = _if_index;

        // Copy IPv6 address from peer (IPv6Address stores 16 bytes)
        for (int i = 0; i < 16; i++) {
            ((uint8_t*)&peer_addr.sin6_addr)[i] = peer.address[i];
        }

        ssize_t sent = sendto(_data_socket, data.data(), data.size(), 0,
                              (struct sockaddr*)&peer_addr, sizeof(peer_addr));
        if (sent < 0) {
            WARNING("AutoInterface: Failed to send to peer " + peer.address_string() +
                    " errno=" + std::to_string(errno));
        } else {
            INFO("AutoInterface: Sent " + std::to_string(sent) + " bytes to " + peer.address_string() +
                 " port " + std::to_string(_data_port));
        }
    }

    // Perform post-send housekeeping
    handle_outgoing(data);
#else
    // POSIX: Send to all known peers via unicast
    for (const auto& peer : _peers) {
        if (peer.is_local) continue;  // Don't send to ourselves

        struct sockaddr_in6 peer_addr;
        memset(&peer_addr, 0, sizeof(peer_addr));
        peer_addr.sin6_family = AF_INET6;
        peer_addr.sin6_port = htons(_data_port);
        peer_addr.sin6_addr = peer.address;
        peer_addr.sin6_scope_id = _if_index;

        ssize_t sent = sendto(_data_socket, data.data(), data.size(), 0,
                              (struct sockaddr*)&peer_addr, sizeof(peer_addr));
        if (sent < 0) {
            WARNING("AutoInterface: Failed to send to peer " + peer.address_string() +
                    ": " + std::string(strerror(errno)));
        } else {
            TRACE("AutoInterface: Sent " + std::to_string(sent) + " bytes to " + peer.address_string());
        }
    }

    // Perform post-send housekeeping
    InterfaceImpl::handle_outgoing(data);
#endif
}

// ============================================================================
// Platform-specific: get_link_local_address()
// ============================================================================

#ifdef ARDUINO

bool AutoInterface::get_link_local_address() {
    // ESP32: Get link-local IPv6 from WiFi
    if (WiFi.status() != WL_CONNECTED) {
        ERROR("AutoInterface: WiFi not connected");
        return false;
    }

    // Enable IPv6 and wait for link-local address
    WiFi.enableIpV6();
    DEBUG("AutoInterface: IPv6 enabled, waiting for link-local address...");

    // Give time for SLAAC to assign link-local address
    for (int i = 0; i < 100; i++) {  // Increased timeout to 10 seconds
        IPv6Address lladdr = WiFi.localIPv6();

        // Debug: print what we're getting
        if (i % 10 == 0) {
            DEBUG("AutoInterface: Attempt " + std::to_string(i) + " - IPv6: " +
                  std::string(lladdr.toString().c_str()));
        }

        // Check if we got a valid address (not all zeros)
        // IPv6Address stores bytes in network order
        if (lladdr[0] != 0 || lladdr[1] != 0) {
            // Store as in6_addr - copy from IPv6Address internal storage
            // IPv6Address operator[] returns bytes in network order
            uint8_t addr_bytes[16];
            for (int j = 0; j < 16; j++) {
                addr_bytes[j] = lladdr[j];
                ((uint8_t*)&_link_local_address)[j] = lladdr[j];
            }

            // Store the address string in COMPRESSED format to match Python's inet_ntop
            _link_local_address_str = ipv6_to_compressed_string(addr_bytes);

            // Also store as IPAddress for easier ESP32 use
            _link_local_ip = lladdr;

            INFO("AutoInterface: Found IPv6 address " + _link_local_address_str);

            // Check if it's link-local (fe80::/10)
            if (lladdr[0] == 0xfe && (lladdr[1] & 0xc0) == 0x80) {
                INFO("AutoInterface: Confirmed link-local address");
                return true;
            } else {
                // Got an address, but it's not link-local - might be global
                // Still use it for now
                WARNING("AutoInterface: Got non-link-local IPv6: " + _link_local_address_str);
                return true;
            }
        }
        delay(100);
    }

    ERROR("AutoInterface: No IPv6 address after timeout");
    return false;
}

void AutoInterface::check_link_local_address() {
    // ESP32: Check if link-local address changed
    if (WiFi.status() != WL_CONNECTED) {
        WARNING("AutoInterface: WiFi disconnected during address check");
        return;
    }

    IPv6Address current_ip = WiFi.localIPv6();

    // Check for valid address (not all zeros)
    if (current_ip[0] == 0 && current_ip[1] == 0) {
        WARNING("AutoInterface: Lost IPv6 address");
        return;
    }

    // Compare with stored address
    if (current_ip == _link_local_ip) {
        return;  // No change
    }

    // Address changed!
    std::string old_addr_str = _link_local_address_str;

    // Update stored addresses
    _link_local_ip = current_ip;
    for (int i = 0; i < 16; i++) {
        ((uint8_t*)&_link_local_address)[i] = current_ip[i];
    }

    // Get new address string in compressed format
    uint8_t addr_bytes[16];
    for (int i = 0; i < 16; i++) {
        addr_bytes[i] = current_ip[i];
    }
    _link_local_address_str = ipv6_to_compressed_string(addr_bytes);

    WARNING("AutoInterface: Link-local address changed from " + old_addr_str + " to " + _link_local_address_str);

    // Close and rebind data socket
    if (_data_socket > -1) {
        close(_data_socket);
        _data_socket = -1;
    }
    if (!setup_data_socket()) {
        WARNING("AutoInterface: Failed to rebind data socket after address change");
        _data_socket_ok = false;
    } else {
        _data_socket_ok = true;
        INFO("AutoInterface: Data socket rebound to new address");
    }

    // Close and rebind unicast discovery socket
    if (_unicast_discovery_socket > -1) {
        close(_unicast_discovery_socket);
        _unicast_discovery_socket = -1;
    }
    if (!setup_unicast_discovery_socket()) {
        WARNING("AutoInterface: Failed to rebind unicast discovery socket after address change");
    } else {
        INFO("AutoInterface: Unicast discovery socket rebound to new address");
    }

    // Recalculate discovery token (critical - token includes address)
    calculate_discovery_token();
    INFO("AutoInterface: Discovery token recalculated: " + _discovery_token.toHex());

    // Signal change to Transport layer
    _carrier_changed = true;
}

#else  // POSIX/Linux

bool AutoInterface::get_link_local_address() {
    struct ifaddrs* ifaddr;
    if (getifaddrs(&ifaddr) == -1) {
        ERROR("AutoInterface: getifaddrs failed: " + std::string(strerror(errno)));
        return false;
    }

    bool found = false;
    for (struct ifaddrs* ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr == nullptr) continue;
        if (ifa->ifa_addr->sa_family != AF_INET6) continue;

        // Skip loopback
        if (strcmp(ifa->ifa_name, "lo") == 0) continue;

        // If interface name specified, match it
        if (!_ifname.empty() && _ifname != ifa->ifa_name) continue;

        struct sockaddr_in6* addr6 = (struct sockaddr_in6*)ifa->ifa_addr;

        // Check for link-local address (fe80::/10)
        if (IN6_IS_ADDR_LINKLOCAL(&addr6->sin6_addr)) {
            _link_local_address = addr6->sin6_addr;
            _if_index = if_nametoindex(ifa->ifa_name);
            _ifname = ifa->ifa_name;

            // Convert to string for token generation
            char buf[INET6_ADDRSTRLEN];
            inet_ntop(AF_INET6, &addr6->sin6_addr, buf, sizeof(buf));
            _link_local_address_str = buf;

            INFO("AutoInterface: Found link-local address " + _link_local_address_str +
                 " on interface " + _ifname);
            found = true;
            break;
        }
    }

    freeifaddrs(ifaddr);
    return found;
}

void AutoInterface::check_link_local_address() {
    // POSIX: Check if link-local address changed
    struct in6_addr old_addr = _link_local_address;
    std::string old_addr_str = _link_local_address_str;

    // Temporarily clear to force refresh
    memset(&_link_local_address, 0, sizeof(_link_local_address));

    if (!get_link_local_address()) {
        // Lost address entirely - restore old address
        WARNING("AutoInterface: Lost link-local address during check");
        _link_local_address = old_addr;
        _link_local_address_str = old_addr_str;
        return;
    }

    // Check if address changed
    if (memcmp(&old_addr, &_link_local_address, sizeof(old_addr)) == 0) {
        return;  // No change
    }

    // Address changed!
    WARNING("AutoInterface: Link-local address changed from " + old_addr_str + " to " + _link_local_address_str);

    // Close and rebind data socket
    if (_data_socket > -1) {
        close(_data_socket);
        _data_socket = -1;
    }
    if (!setup_data_socket()) {
        WARNING("AutoInterface: Failed to rebind data socket after address change");
    } else {
        INFO("AutoInterface: Data socket rebound to new address");
    }

    // Close and rebind unicast discovery socket
    if (_unicast_discovery_socket > -1) {
        close(_unicast_discovery_socket);
        _unicast_discovery_socket = -1;
    }
    if (!setup_unicast_discovery_socket()) {
        WARNING("AutoInterface: Failed to rebind unicast discovery socket after address change");
    } else {
        INFO("AutoInterface: Unicast discovery socket rebound to new address");
    }

    // Recalculate discovery token (critical - token includes address)
    calculate_discovery_token();
    INFO("AutoInterface: Discovery token recalculated: " + _discovery_token.toHex());

    // Signal change to Transport layer
    _carrier_changed = true;
}

#endif  // ARDUINO

void AutoInterface::calculate_multicast_address() {
    // Python: group_hash = RNS.Identity.full_hash(self.group_id)
    Bytes group_id_bytes((const uint8_t*)_group_id.c_str(), _group_id.length());
    Bytes group_hash = Identity::full_hash(group_id_bytes);

    // Build multicast address: ff12:0:XXXX:XXXX:XXXX:XXXX:XXXX:XXXX
    // ff = multicast prefix
    // 1 = temporary address type (MULTICAST_TEMPORARY_ADDRESS_TYPE)
    // 2 = link scope (SCOPE_LINK)
    // The remaining 112 bits come from the group hash

    // Python format from AutoInterface.py lines 195-205:
    //   gt  = "0"                                      # literal 0, NOT from hash!
    //   gt += ":"+"{:02x}".format(g[3]+(g[2]<<8))      # hash bytes 2-3
    //   gt += ":"+"{:02x}".format(g[5]+(g[4]<<8))      # hash bytes 4-5
    //   gt += ":"+"{:02x}".format(g[7]+(g[6]<<8))      # hash bytes 6-7
    //   gt += ":"+"{:02x}".format(g[9]+(g[8]<<8))      # hash bytes 8-9
    //   gt += ":"+"{:02x}".format(g[11]+(g[10]<<8))    # hash bytes 10-11
    //   gt += ":"+"{:02x}".format(g[13]+(g[12]<<8))    # hash bytes 12-13
    //   mcast_discovery_address = "ff12:" + gt
    //
    // Result: ff12:0:XXXX:XXXX:XXXX:XXXX:XXXX:XXXX (8 groups)

    uint8_t addr[16];
    addr[0] = 0xff;
    addr[1] = 0x12;  // 1=temporary, 2=link scope

    const uint8_t* g = group_hash.data();

    // Group 1: Python uses literal "0", NOT hash bytes 0-1!
    addr[2] = 0x00;
    addr[3] = 0x00;

    // Group 2: g[3]+(g[2]<<8) - starts at hash byte 2
    addr[4] = g[2];
    addr[5] = g[3];

    // Group 3: g[5]+(g[4]<<8)
    addr[6] = g[4];
    addr[7] = g[5];

    // Group 4: g[7]+(g[6]<<8)
    addr[8] = g[6];
    addr[9] = g[7];

    // Group 5: g[9]+(g[8]<<8)
    addr[10] = g[8];
    addr[11] = g[9];

    // Group 6: g[11]+(g[10]<<8)
    addr[12] = g[10];
    addr[13] = g[11];

    // Group 7: g[13]+(g[12]<<8)
    addr[14] = g[12];
    addr[15] = g[13];

    memcpy(&_multicast_address, addr, 16);
    _multicast_address_bytes = Bytes(addr, 16);

    // Convert to string for logging
    char buf[INET6_ADDRSTRLEN];
    inet_ntop(AF_INET6, &_multicast_address, buf, sizeof(buf));
    _multicast_address_str = buf;

#ifdef ARDUINO
    // Also store as IPv6Address for ESP32 use
    _multicast_ip = IPv6Address(addr);
#endif
}

// ============================================================================
// Platform-independent: calculate_discovery_token()
// ============================================================================

void AutoInterface::calculate_discovery_token() {
    // Python: discovery_token = RNS.Identity.full_hash(self.group_id+link_local_address.encode("utf-8"))
    // Python sends the FULL 32-byte hash (not truncated)
    Bytes combined;
    combined.append((const uint8_t*)_group_id.c_str(), _group_id.length());
    combined.append((const uint8_t*)_link_local_address_str.c_str(), _link_local_address_str.length());

    Bytes full_hash = Identity::full_hash(combined);
    // Use full TOKEN_SIZE (32 bytes) to match Python RNS
    _discovery_token = Bytes(full_hash.data(), TOKEN_SIZE);
    TRACE("AutoInterface: Discovery token input: " + combined.toHex());
    TRACE("AutoInterface: Discovery token: " + _discovery_token.toHex());
}

// ============================================================================
// Platform-specific: Socket setup
// ============================================================================

#ifdef ARDUINO

bool AutoInterface::setup_discovery_socket() {
    // ESP32: Use raw lwIP socket for IPv6 multicast (WiFiUDP.beginMulticast() only supports IPv4)

    // Create IPv6 UDP socket
    _discovery_socket = socket(AF_INET6, SOCK_DGRAM, IPPROTO_UDP);
    if (_discovery_socket < 0) {
        ERROR("AutoInterface: Failed to create discovery socket (errno=" + std::to_string(errno) + ")");
        return false;
    }

    // Allow address reuse
    int reuse = 1;
    setsockopt(_discovery_socket, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    // Bind to discovery port on in6addr_any (ESP32 lwIP doesn't support binding to multicast)
    sockaddr_in6 bind_addr = {};
    bind_addr.sin6_family = AF_INET6;
    bind_addr.sin6_port = htons(_discovery_port);
    bind_addr.sin6_addr = in6addr_any;  // Receive from any source

    if (bind(_discovery_socket, (sockaddr*)&bind_addr, sizeof(bind_addr)) < 0) {
        ERROR("AutoInterface: Failed to bind discovery socket (errno=" + std::to_string(errno) + ")");
        close(_discovery_socket);
        _discovery_socket = -1;
        return false;
    }

    // Set non-blocking
    int flags = fcntl(_discovery_socket, F_GETFL, 0);
    fcntl(_discovery_socket, F_SETFL, flags | O_NONBLOCK);

    // Find the station netif (WiFi interface) to get interface index
    netif* nif = netif_list;
    while (nif != nullptr) {
        if (nif->name[0] == 's' && nif->name[1] == 't') {  // "st" = station
            break;
        }
        nif = nif->next;
    }

    if (nif != nullptr) {
        // Get interface index for multicast (needed for send and receive)
        _if_index = netif_get_index(nif);
        INFO("AutoInterface: Using interface index " + std::to_string(_if_index) + " for multicast");

        // Join the IPv6 multicast group using standard socket API
        // This properly links the socket to receive multicast packets
        ipv6_mreq mreq;
        memcpy(&mreq.ipv6mr_multiaddr, &_multicast_address, sizeof(_multicast_address));
        mreq.ipv6mr_interface = _if_index;

        if (setsockopt(_discovery_socket, IPPROTO_IPV6, IPV6_JOIN_GROUP, &mreq, sizeof(mreq)) < 0) {
            WARNING("AutoInterface: Failed to join multicast group via setsockopt (errno=" +
                    std::to_string(errno) + "), trying mld6 API");
            // Fallback to lwIP mld6 API
            ip6_addr_t mcast_addr;
            memcpy(&mcast_addr.addr, &_multicast_address, sizeof(_multicast_address));
            err_t err = mld6_joingroup_netif(nif, &mcast_addr);
            if (err == ERR_OK) {
                INFO("AutoInterface: Joined IPv6 multicast group via mld6 API: " + _multicast_address_str);
            } else {
                WARNING("AutoInterface: mld6_joingroup failed (err=" + std::to_string(err) +
                        ") - discovery may not work");
            }
        } else {
            INFO("AutoInterface: Joined IPv6 multicast group via setsockopt: " + _multicast_address_str);
        }

        // Set multicast interface for outgoing packets (critical for multicast to reach other hosts!)
        if (setsockopt(_discovery_socket, IPPROTO_IPV6, IPV6_MULTICAST_IF,
                       &_if_index, sizeof(_if_index)) < 0) {
            WARNING("AutoInterface: Failed to set IPV6_MULTICAST_IF (errno=" + std::to_string(errno) + ")");
        } else {
            DEBUG("AutoInterface: Set IPV6_MULTICAST_IF to interface " + std::to_string(_if_index));
        }
    } else {
        WARNING("AutoInterface: Could not find station netif for multicast join");
    }

    INFO("AutoInterface: Discovery socket listening on port " + std::to_string(_discovery_port));
    return true;
}

bool AutoInterface::setup_data_socket() {
    // ESP32: Use raw IPv6 socket for data port (WiFiUDP doesn't support IPv6)
    _data_socket = socket(AF_INET6, SOCK_DGRAM, IPPROTO_UDP);
    if (_data_socket < 0) {
        ERROR("AutoInterface: Failed to create data socket (errno=" + std::to_string(errno) + ")");
        return false;
    }

    // Allow address reuse
    int reuse = 1;
    setsockopt(_data_socket, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    // Note: _if_index should already be set by setup_discovery_socket()
    // Fallback in case discovery socket wasn't set up first
    if (_if_index == 0) {
        netif* nif = netif_list;
        while (nif != nullptr) {
            if (nif->name[0] == 's' && nif->name[1] == 't') {  // "st" = station
                _if_index = netif_get_index(nif);
                break;
            }
            nif = nif->next;
        }
        INFO("AutoInterface: Using interface index " + std::to_string(_if_index) + " for data socket (fallback)");
    }

    // Bind to our link-local address and data port (helps with routing)
    sockaddr_in6 bind_addr = {};
    bind_addr.sin6_family = AF_INET6;
    bind_addr.sin6_port = htons(_data_port);
    memcpy(&bind_addr.sin6_addr, &_link_local_address, sizeof(_link_local_address));
    bind_addr.sin6_scope_id = _if_index;

    if (bind(_data_socket, (sockaddr*)&bind_addr, sizeof(bind_addr)) < 0) {
        WARNING("AutoInterface: Failed to bind to link-local (errno=" + std::to_string(errno) +
                "), trying any address");
        // Fallback to any address
        bind_addr.sin6_addr = in6addr_any;
        bind_addr.sin6_scope_id = 0;
        if (bind(_data_socket, (sockaddr*)&bind_addr, sizeof(bind_addr)) < 0) {
            ERROR("AutoInterface: Failed to bind data socket (errno=" + std::to_string(errno) + ")");
            close(_data_socket);
            _data_socket = -1;
            return false;
        }
    }

    // Set non-blocking
    int flags = fcntl(_data_socket, F_GETFL, 0);
    fcntl(_data_socket, F_SETFL, flags | O_NONBLOCK);

    INFO("AutoInterface: Data socket listening on port " + std::to_string(_data_port));
    return true;
}

bool AutoInterface::setup_unicast_discovery_socket() {
    // ESP32: Create socket for receiving unicast discovery (reverse peering)
    _unicast_discovery_socket = socket(AF_INET6, SOCK_DGRAM, IPPROTO_UDP);
    if (_unicast_discovery_socket < 0) {
        ERROR("AutoInterface: Failed to create unicast discovery socket (errno=" + std::to_string(errno) + ")");
        return false;
    }

    // Allow address reuse
    int reuse = 1;
    setsockopt(_unicast_discovery_socket, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    // Bind to our link-local address and unicast discovery port
    sockaddr_in6 bind_addr = {};
    bind_addr.sin6_family = AF_INET6;
    bind_addr.sin6_port = htons(_unicast_discovery_port);
    memcpy(&bind_addr.sin6_addr, &_link_local_address, sizeof(_link_local_address));
    bind_addr.sin6_scope_id = _if_index;

    if (bind(_unicast_discovery_socket, (sockaddr*)&bind_addr, sizeof(bind_addr)) < 0) {
        WARNING("AutoInterface: Failed to bind unicast discovery to link-local (errno=" + std::to_string(errno) +
                "), trying any address");
        // Fallback to any address
        bind_addr.sin6_addr = in6addr_any;
        bind_addr.sin6_scope_id = 0;
        if (bind(_unicast_discovery_socket, (sockaddr*)&bind_addr, sizeof(bind_addr)) < 0) {
            ERROR("AutoInterface: Failed to bind unicast discovery socket (errno=" + std::to_string(errno) + ")");
            close(_unicast_discovery_socket);
            _unicast_discovery_socket = -1;
            return false;
        }
    }

    // Set non-blocking
    int flags = fcntl(_unicast_discovery_socket, F_GETFL, 0);
    fcntl(_unicast_discovery_socket, F_SETFL, flags | O_NONBLOCK);

    INFO("AutoInterface: Unicast discovery socket listening on port " + std::to_string(_unicast_discovery_port));
    return true;
}

bool AutoInterface::join_multicast_group() {
    // ESP32: Multicast join handled by beginMulticast()
    INFO("AutoInterface: Joined multicast group " + _multicast_address_str);
    return true;
}

void AutoInterface::send_announce() {
    // ESP32: Send discovery token to multicast address using raw socket
    if (_discovery_socket < 0) {
        WARNING("AutoInterface: Discovery socket not initialized");
        return;
    }

    sockaddr_in6 dest_addr = {};
    dest_addr.sin6_family = AF_INET6;
    dest_addr.sin6_port = htons(_discovery_port);
    memcpy(&dest_addr.sin6_addr, &_multicast_address, sizeof(_multicast_address));
    dest_addr.sin6_scope_id = _if_index;  // Specify WiFi interface for link-local multicast

    ssize_t sent = sendto(_discovery_socket, _discovery_token.data(), _discovery_token.size(), 0,
                          (struct sockaddr*)&dest_addr, sizeof(dest_addr));
    if (sent > 0) {
        DEBUG("AutoInterface: Sent discovery announce (" + std::to_string(sent) + " bytes) to " + _multicast_address_str);
    } else {
        WARNING("AutoInterface: Failed to send discovery announce (errno=" + std::to_string(errno) + ")");
    }
}

void AutoInterface::process_discovery() {
    // ESP32: Use raw socket recvfrom for IPv6 multicast
    if (_discovery_socket < 0) return;

    uint8_t recv_buffer[128];
    sockaddr_in6 src_addr;
    socklen_t src_len = sizeof(src_addr);

    ssize_t len = recvfrom(_discovery_socket, recv_buffer, sizeof(recv_buffer), 0,
                           (struct sockaddr*)&src_addr, &src_len);

    // Hot path - no logging to avoid heap allocation on every packet

    while (len > 0) {
        // Convert source address to COMPRESSED string format (match Python)
        std::string src_str = ipv6_to_compressed_string((const uint8_t*)&src_addr.sin6_addr);

        // Verify the peering hash (full TOKEN_SIZE = 32 bytes)
        Bytes combined;
        combined.append((const uint8_t*)_group_id.c_str(), _group_id.length());
        combined.append((const uint8_t*)src_str.c_str(), src_str.length());
        Bytes expected_hash = Identity::full_hash(combined);

        // Compare received token with expected (full TOKEN_SIZE = 32 bytes)
        if (len >= (ssize_t)TOKEN_SIZE && memcmp(recv_buffer, expected_hash.data(), TOKEN_SIZE) == 0) {
            // Valid peer - use IPv6Address (IPAddress is IPv4-only!)
            IPv6Address remoteIP((const uint8_t*)&src_addr.sin6_addr);
            add_or_refresh_peer(remoteIP, Utilities::OS::time());
        }

        // Try to receive more
        src_len = sizeof(src_addr);
        len = recvfrom(_discovery_socket, recv_buffer, sizeof(recv_buffer), 0,
                       (sockaddr*)&src_addr, &src_len);
    }
}

void AutoInterface::process_data() {
    // ESP32: Use raw socket for IPv6 data reception
    if (_data_socket < 0) return;

    uint8_t recv_buffer[Type::Reticulum::MTU + 64];
    sockaddr_in6 src_addr;
    socklen_t src_len = sizeof(src_addr);

    ssize_t len = recvfrom(_data_socket, recv_buffer, sizeof(recv_buffer), 0,
                           (struct sockaddr*)&src_addr, &src_len);

    while (len > 0) {
        _buffer.clear();
        _buffer.append(recv_buffer, len);

        // Check for duplicates
        if (is_duplicate(_buffer)) {
            TRACE("AutoInterface: Dropping duplicate packet");
            src_len = sizeof(src_addr);
            len = recvfrom(_data_socket, recv_buffer, sizeof(recv_buffer), 0,
                           (sockaddr*)&src_addr, &src_len);
            continue;
        }

        add_to_deque(_buffer);

        // Convert source address to string for logging
        std::string src_str = ipv6_to_compressed_string((const uint8_t*)&src_addr.sin6_addr);
        DEBUG("AutoInterface: Received data from " + src_str + " (" + std::to_string(len) + " bytes)");

        // Pass to transport
        InterfaceImpl::handle_incoming(_buffer);

        // Try to receive more
        src_len = sizeof(src_addr);
        len = recvfrom(_data_socket, recv_buffer, sizeof(recv_buffer), 0,
                       (sockaddr*)&src_addr, &src_len);
    }
}

void AutoInterface::process_unicast_discovery() {
    // ESP32: Process incoming unicast discovery packets (reverse peering)
    if (_unicast_discovery_socket < 0) return;

    uint8_t recv_buffer[128];
    sockaddr_in6 src_addr;
    socklen_t src_len = sizeof(src_addr);

    ssize_t len = recvfrom(_unicast_discovery_socket, recv_buffer, sizeof(recv_buffer), 0,
                           (struct sockaddr*)&src_addr, &src_len);

    while (len > 0) {
        // Convert source address to COMPRESSED string format (match Python)
        std::string src_str = ipv6_to_compressed_string((const uint8_t*)&src_addr.sin6_addr);

        // Verify the peering hash (full TOKEN_SIZE = 32 bytes)
        Bytes combined;
        combined.append((const uint8_t*)_group_id.c_str(), _group_id.length());
        combined.append((const uint8_t*)src_str.c_str(), src_str.length());
        Bytes expected_hash = Identity::full_hash(combined);

        // Compare received token with expected (full TOKEN_SIZE = 32 bytes)
        if (len >= (ssize_t)TOKEN_SIZE && memcmp(recv_buffer, expected_hash.data(), TOKEN_SIZE) == 0) {
            // Valid peer via unicast discovery (reverse peering)
            IPv6Address remoteIP((const uint8_t*)&src_addr.sin6_addr);
            DEBUG("AutoInterface: Received unicast discovery from " + src_str);
            add_or_refresh_peer(remoteIP, Utilities::OS::time());
        }

        // Try to receive more
        src_len = sizeof(src_addr);
        len = recvfrom(_unicast_discovery_socket, recv_buffer, sizeof(recv_buffer), 0,
                       (sockaddr*)&src_addr, &src_len);
    }
}

void AutoInterface::reverse_announce(AutoInterfacePeer& peer) {
    // ESP32: Send our discovery token directly to a peer's unicast discovery port
    // This allows peer to discover us even if multicast is not working

    // Create temporary socket for sending
    int sock = socket(AF_INET6, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        WARNING("AutoInterface: Failed to create reverse announce socket (errno=" + std::to_string(errno) + ")");
        return;
    }

    // Build destination address
    sockaddr_in6 dest_addr = {};
    dest_addr.sin6_family = AF_INET6;
    dest_addr.sin6_port = htons(_unicast_discovery_port);
    dest_addr.sin6_scope_id = _if_index;

    // Copy peer's IPv6 address
    for (int i = 0; i < 16; i++) {
        ((uint8_t*)&dest_addr.sin6_addr)[i] = peer.address[i];
    }

    // Send discovery token
    ssize_t sent = sendto(sock, _discovery_token.data(), _discovery_token.size(), 0,
                          (struct sockaddr*)&dest_addr, sizeof(dest_addr));

    close(sock);

    if (sent > 0) {
        TRACE("AutoInterface: Sent reverse announce to " + peer.address_string());
    } else {
        WARNING("AutoInterface: Failed to send reverse announce to " + peer.address_string() +
                " (errno=" + std::to_string(errno) + ")");
    }
}

void AutoInterface::send_reverse_peering() {
    // ESP32: Periodically send reverse peering to known peers
    // This maintains peer connections even when multicast is unreliable
    double now = Utilities::OS::time();

    for (auto& peer : _peers) {
        // Skip local peers (our own announcements)
        if (peer.is_local) continue;

        // Check if it's time to send reverse peering to this peer
        if (now > peer.last_outbound + REVERSE_PEERING_INTERVAL) {
            reverse_announce(peer);
            peer.last_outbound = now;
        }
    }
}

#else  // POSIX/Linux

bool AutoInterface::setup_discovery_socket() {
    // Create IPv6 UDP socket
    _discovery_socket = socket(AF_INET6, SOCK_DGRAM, 0);
    if (_discovery_socket < 0) {
        ERROR("AutoInterface: Could not create discovery socket: " + std::string(strerror(errno)));
        return false;
    }

    // Enable address reuse
    int reuse = 1;
    setsockopt(_discovery_socket, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
#ifdef SO_REUSEPORT
    setsockopt(_discovery_socket, SOL_SOCKET, SO_REUSEPORT, &reuse, sizeof(reuse));
#endif

    // Set multicast interface
    setsockopt(_discovery_socket, IPPROTO_IPV6, IPV6_MULTICAST_IF, &_if_index, sizeof(_if_index));

    // Join multicast group
    if (!join_multicast_group()) {
        close(_discovery_socket);
        _discovery_socket = -1;
        return false;
    }

    // Bind to discovery port on multicast address
    struct sockaddr_in6 bind_addr;
    memset(&bind_addr, 0, sizeof(bind_addr));
    bind_addr.sin6_family = AF_INET6;
    bind_addr.sin6_port = htons(_discovery_port);
    bind_addr.sin6_addr = _multicast_address;
    bind_addr.sin6_scope_id = _if_index;

    if (bind(_discovery_socket, (struct sockaddr*)&bind_addr, sizeof(bind_addr)) < 0) {
        ERROR("AutoInterface: Could not bind discovery socket: " + std::string(strerror(errno)));
        close(_discovery_socket);
        _discovery_socket = -1;
        return false;
    }

    // Make socket non-blocking
    int flags = 1;
    ioctl(_discovery_socket, FIONBIO, &flags);

    INFO("AutoInterface: Discovery socket bound to port " + std::to_string(_discovery_port));
    return true;
}

bool AutoInterface::setup_data_socket() {
    // Create IPv6 UDP socket for data
    _data_socket = socket(AF_INET6, SOCK_DGRAM, 0);
    if (_data_socket < 0) {
        ERROR("AutoInterface: Could not create data socket: " + std::string(strerror(errno)));
        return false;
    }

    // Enable address reuse
    int reuse = 1;
    setsockopt(_data_socket, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
#ifdef SO_REUSEPORT
    setsockopt(_data_socket, SOL_SOCKET, SO_REUSEPORT, &reuse, sizeof(reuse));
#endif

    // Bind to data port on link-local address
    struct sockaddr_in6 bind_addr;
    memset(&bind_addr, 0, sizeof(bind_addr));
    bind_addr.sin6_family = AF_INET6;
    bind_addr.sin6_port = htons(_data_port);
    bind_addr.sin6_addr = _link_local_address;
    bind_addr.sin6_scope_id = _if_index;

    if (bind(_data_socket, (struct sockaddr*)&bind_addr, sizeof(bind_addr)) < 0) {
        ERROR("AutoInterface: Could not bind data socket: " + std::string(strerror(errno)));
        close(_data_socket);
        _data_socket = -1;
        return false;
    }

    // Make socket non-blocking
    int flags = 1;
    ioctl(_data_socket, FIONBIO, &flags);

    INFO("AutoInterface: Data socket bound to port " + std::to_string(_data_port));
    return true;
}

bool AutoInterface::join_multicast_group() {
    struct ipv6_mreq mreq;
    memcpy(&mreq.ipv6mr_multiaddr, &_multicast_address, sizeof(_multicast_address));
    mreq.ipv6mr_interface = _if_index;

    if (setsockopt(_discovery_socket, IPPROTO_IPV6, IPV6_JOIN_GROUP, &mreq, sizeof(mreq)) < 0) {
        ERROR("AutoInterface: Could not join multicast group: " + std::string(strerror(errno)));
        return false;
    }

    INFO("AutoInterface: Joined multicast group " + _multicast_address_str);
    return true;
}

void AutoInterface::send_announce() {
    if (_discovery_socket < 0) return;

    // Send discovery token to multicast address
    struct sockaddr_in6 mcast_addr;
    memset(&mcast_addr, 0, sizeof(mcast_addr));
    mcast_addr.sin6_family = AF_INET6;
    mcast_addr.sin6_port = htons(_discovery_port);
    mcast_addr.sin6_addr = _multicast_address;
    mcast_addr.sin6_scope_id = _if_index;

    ssize_t sent = sendto(_discovery_socket, _discovery_token.data(), _discovery_token.size(), 0,
                          (struct sockaddr*)&mcast_addr, sizeof(mcast_addr));
    if (sent < 0) {
        WARNING("AutoInterface: Failed to send discovery announce: " + std::string(strerror(errno)));
    } else {
        TRACE("AutoInterface: Sent discovery announce (" + std::to_string(sent) + " bytes)");
    }
}

void AutoInterface::process_discovery() {
    if (_discovery_socket < 0) return;

    uint8_t recv_buffer[1024];
    struct sockaddr_in6 src_addr;
    socklen_t addr_len = sizeof(src_addr);

    while (true) {
        ssize_t len = recvfrom(_discovery_socket, recv_buffer, sizeof(recv_buffer), 0,
                               (struct sockaddr*)&src_addr, &addr_len);
        if (len <= 0) break;

        // Get source address string
        char src_str[INET6_ADDRSTRLEN];
        inet_ntop(AF_INET6, &src_addr.sin6_addr, src_str, sizeof(src_str));

        DEBUG("AutoInterface: Received discovery packet from " + std::string(src_str) +
              " (" + std::to_string(len) + " bytes)");

        // Verify the peering hash
        Bytes combined;
        combined.append((const uint8_t*)_group_id.c_str(), _group_id.length());
        combined.append((const uint8_t*)src_str, strlen(src_str));
        Bytes expected_hash = Identity::full_hash(combined);

        // Compare received hash with expected
        if (len >= 32 && memcmp(recv_buffer, expected_hash.data(), 32) == 0) {
            // Valid peer
            add_or_refresh_peer(src_addr.sin6_addr, RNS::Utilities::OS::time());
        } else {
            DEBUG("AutoInterface: Invalid discovery hash from " + std::string(src_str));
        }
    }
}

void AutoInterface::process_data() {
    if (_data_socket < 0) return;

    struct sockaddr_in6 src_addr;
    socklen_t addr_len = sizeof(src_addr);

    while (true) {
        _buffer.clear();
        ssize_t len = recvfrom(_data_socket, _buffer.writable(Type::Reticulum::MTU),
                               Type::Reticulum::MTU, 0,
                               (struct sockaddr*)&src_addr, &addr_len);
        if (len <= 0) break;

        _buffer.resize(len);

        // Check for duplicates (multi-interface deduplication)
        if (is_duplicate(_buffer)) {
            TRACE("AutoInterface: Dropping duplicate packet");
            continue;
        }

        add_to_deque(_buffer);

        char src_str[INET6_ADDRSTRLEN];
        inet_ntop(AF_INET6, &src_addr.sin6_addr, src_str, sizeof(src_str));
        DEBUG("AutoInterface: Received data from " + std::string(src_str) +
              " (" + std::to_string(len) + " bytes)");

        // Pass to transport
        InterfaceImpl::handle_incoming(_buffer);
    }
}

bool AutoInterface::setup_unicast_discovery_socket() {
    // POSIX: Create socket for receiving unicast discovery (reverse peering)
    _unicast_discovery_socket = socket(AF_INET6, SOCK_DGRAM, 0);
    if (_unicast_discovery_socket < 0) {
        ERROR("AutoInterface: Could not create unicast discovery socket: " + std::string(strerror(errno)));
        return false;
    }

    // Enable address reuse
    int reuse = 1;
    setsockopt(_unicast_discovery_socket, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
#ifdef SO_REUSEPORT
    setsockopt(_unicast_discovery_socket, SOL_SOCKET, SO_REUSEPORT, &reuse, sizeof(reuse));
#endif

    // Bind to unicast discovery port on link-local address
    struct sockaddr_in6 bind_addr;
    memset(&bind_addr, 0, sizeof(bind_addr));
    bind_addr.sin6_family = AF_INET6;
    bind_addr.sin6_port = htons(_unicast_discovery_port);
    bind_addr.sin6_addr = _link_local_address;
    bind_addr.sin6_scope_id = _if_index;

    if (bind(_unicast_discovery_socket, (struct sockaddr*)&bind_addr, sizeof(bind_addr)) < 0) {
        ERROR("AutoInterface: Could not bind unicast discovery socket: " + std::string(strerror(errno)));
        close(_unicast_discovery_socket);
        _unicast_discovery_socket = -1;
        return false;
    }

    // Make socket non-blocking
    int flags = 1;
    ioctl(_unicast_discovery_socket, FIONBIO, &flags);

    INFO("AutoInterface: Unicast discovery socket bound to port " + std::to_string(_unicast_discovery_port));
    return true;
}

void AutoInterface::process_unicast_discovery() {
    // POSIX: Process incoming unicast discovery packets (reverse peering)
    if (_unicast_discovery_socket < 0) return;

    uint8_t recv_buffer[128];
    struct sockaddr_in6 src_addr;
    socklen_t addr_len = sizeof(src_addr);

    while (true) {
        ssize_t len = recvfrom(_unicast_discovery_socket, recv_buffer, sizeof(recv_buffer), 0,
                               (struct sockaddr*)&src_addr, &addr_len);
        if (len <= 0) break;

        // Get source address string
        char src_str[INET6_ADDRSTRLEN];
        inet_ntop(AF_INET6, &src_addr.sin6_addr, src_str, sizeof(src_str));

        DEBUG("AutoInterface: Received unicast discovery from " + std::string(src_str) +
              " (" + std::to_string(len) + " bytes)");

        // Verify the peering hash
        Bytes combined;
        combined.append((const uint8_t*)_group_id.c_str(), _group_id.length());
        combined.append((const uint8_t*)src_str, strlen(src_str));
        Bytes expected_hash = Identity::full_hash(combined);

        // Compare received hash with expected
        if (len >= 32 && memcmp(recv_buffer, expected_hash.data(), 32) == 0) {
            // Valid peer via unicast discovery (reverse peering)
            add_or_refresh_peer(src_addr.sin6_addr, RNS::Utilities::OS::time());
        } else {
            DEBUG("AutoInterface: Invalid unicast discovery hash from " + std::string(src_str));
        }
    }
}

void AutoInterface::reverse_announce(AutoInterfacePeer& peer) {
    // POSIX: Send our discovery token directly to a peer's unicast discovery port
    // This allows peer to discover us even if multicast is not working

    // Create temporary socket for sending
    int sock = socket(AF_INET6, SOCK_DGRAM, 0);
    if (sock < 0) {
        WARNING("AutoInterface: Failed to create reverse announce socket: " + std::string(strerror(errno)));
        return;
    }

    // Build destination address
    struct sockaddr_in6 dest_addr;
    memset(&dest_addr, 0, sizeof(dest_addr));
    dest_addr.sin6_family = AF_INET6;
    dest_addr.sin6_port = htons(_unicast_discovery_port);
    dest_addr.sin6_addr = peer.address;
    dest_addr.sin6_scope_id = _if_index;

    // Send discovery token
    ssize_t sent = sendto(sock, _discovery_token.data(), _discovery_token.size(), 0,
                          (struct sockaddr*)&dest_addr, sizeof(dest_addr));

    close(sock);

    if (sent > 0) {
        TRACE("AutoInterface: Sent reverse announce to " + peer.address_string());
    } else {
        WARNING("AutoInterface: Failed to send reverse announce to " + peer.address_string() +
                ": " + std::string(strerror(errno)));
    }
}

void AutoInterface::send_reverse_peering() {
    // POSIX: Periodically send reverse peering to known peers
    // This maintains peer connections even when multicast is unreliable
    double now = RNS::Utilities::OS::time();

    for (auto& peer : _peers) {
        // Skip local peers (our own announcements)
        if (peer.is_local) continue;

        // Check if it's time to send reverse peering to this peer
        if (now > peer.last_outbound + REVERSE_PEERING_INTERVAL) {
            reverse_announce(peer);
            peer.last_outbound = now;
        }
    }
}

#endif  // ARDUINO

// ============================================================================
// Platform-specific: Peer management
// ============================================================================

#ifdef ARDUINO

void AutoInterface::add_or_refresh_peer(const IPv6Address& addr, double timestamp) {
    // Check if this is our own address (IPv6Address == properly compares all 16 bytes)
    if (addr == _link_local_ip) {
        // Update echo timestamp
        _last_multicast_echo = timestamp;

        // Track initial echo received
        if (!_initial_echo_received) {
            _initial_echo_received = true;
            INFO("AutoInterface: Initial multicast echo received - multicast is working");
        }

        DEBUG("AutoInterface: Received own multicast echo - ignoring");
        return;
    }

    // Check if peer already exists
    for (auto& peer : _peers) {
        if (peer.same_address(addr)) {
            peer.last_heard = timestamp;
            TRACE("AutoInterface: Refreshed peer " + peer.address_string());
            return;
        }
    }

    // Add new peer
    AutoInterfacePeer new_peer(addr, _data_port, timestamp);
    _peers.push_back(new_peer);

    INFO("AutoInterface: Added new peer " + new_peer.address_string());
}

#else  // POSIX

void AutoInterface::add_or_refresh_peer(const struct in6_addr& addr, double timestamp) {
    // Check if this is our own address
    if (memcmp(&addr, &_link_local_address, sizeof(addr)) == 0) {
        // Update echo timestamp
        _last_multicast_echo = timestamp;

        // Track initial echo received
        if (!_initial_echo_received) {
            _initial_echo_received = true;
            INFO("AutoInterface: Initial multicast echo received - multicast is working");
        }

        DEBUG("AutoInterface: Received own multicast echo - ignoring");
        return;
    }

    // Check if peer already exists
    for (auto& peer : _peers) {
        if (peer.same_address(addr)) {
            peer.last_heard = timestamp;
            TRACE("AutoInterface: Refreshed peer " + peer.address_string());
            return;
        }
    }

    // Add new peer
    AutoInterfacePeer new_peer(addr, _data_port, timestamp);
    _peers.push_back(new_peer);

    INFO("AutoInterface: Added new peer " + new_peer.address_string());
}

#endif  // ARDUINO

// ============================================================================
// Platform-independent: Echo Timeout Checking
// ============================================================================

void AutoInterface::check_echo_timeout() {
    double now = Utilities::OS::time();

    // Only check if we've started announcing
    if (_last_announce == 0) {
        return;  // Haven't sent first announce yet
    }

    // Calculate time since last echo
    double echo_age = now - _last_multicast_echo;
    bool timed_out = (echo_age > MCAST_ECHO_TIMEOUT);

    // Detect timeout state transitions
    if (timed_out != _timed_out) {
        _timed_out = timed_out;
        _carrier_changed = true;

        if (!timed_out) {
            WARNING("AutoInterface: Carrier recovered on interface");
        } else {
            WARNING("AutoInterface: Multicast echo timeout for interface. Carrier lost.");
        }
    }

    // One-time firewall diagnostic (after grace period)
    double startup_grace = ANNOUNCE_INTERVAL * 3.0;  // ~5 seconds

    if (!_initial_echo_received &&
        (now - _last_announce) > startup_grace &&
        !_firewall_warning_logged) {
        ERROR("AutoInterface: No multicast echoes received. "
              "The networking hardware or a firewall may be blocking multicast traffic.");
        _firewall_warning_logged = true;
    }
}

// ============================================================================
// Platform-independent: Deduplication
// ============================================================================

void AutoInterface::expire_stale_peers() {
    double now = Utilities::OS::time();

    _peers.erase(
        std::remove_if(_peers.begin(), _peers.end(),
            [this, now](const AutoInterfacePeer& peer) {
                if (now - peer.last_heard > PEERING_TIMEOUT) {
                    INFO("AutoInterface: Removed stale peer " + peer.address_string());
                    return true;
                }
                return false;
            }),
        _peers.end());
}

bool AutoInterface::is_duplicate(const Bytes& packet) {
    Bytes packet_hash = Identity::full_hash(packet);

    for (const auto& entry : _packet_deque) {
        if (entry.hash == packet_hash) {
            return true;
        }
    }
    return false;
}

void AutoInterface::add_to_deque(const Bytes& packet) {
    DequeEntry entry;
    entry.hash = Identity::full_hash(packet);
    entry.timestamp = Utilities::OS::time();

    _packet_deque.push_back(entry);

    // Limit deque size
    while (_packet_deque.size() > DEQUE_SIZE) {
        _packet_deque.pop_front();
    }
}

void AutoInterface::expire_deque_entries() {
    double now = Utilities::OS::time();

    while (!_packet_deque.empty() && now - _packet_deque.front().timestamp > DEQUE_TTL) {
        _packet_deque.pop_front();
    }
}
