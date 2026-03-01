#pragma once

#include "Interface.h"
#include "Bytes.h"
#include "AutoInterfacePeer.h"

#ifdef ARDUINO
#include <WiFi.h>
#include <IPv6Address.h>
#include <lwip/netdb.h>
#include <lwip/sockets.h>
#else
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#endif

#include <vector>
#include <deque>
#include <string>
#include <cstdint>

// AutoInterface - automatic peer discovery via IPv6 multicast
// Matches Python RNS AutoInterface behavior for interoperability
class AutoInterface : public RNS::InterfaceImpl {

public:
    // Protocol constants (match Python RNS)
    static constexpr uint16_t DEFAULT_DISCOVERY_PORT = 29716;
    static constexpr uint16_t DEFAULT_DATA_PORT = 42671;
    static constexpr const char* DEFAULT_GROUP_ID = "reticulum";
    static constexpr double PEERING_TIMEOUT = 22.0;      // seconds (matches Python RNS)
    static constexpr double ANNOUNCE_INTERVAL = 1.6;     // seconds (matches Python RNS)
    static constexpr double MCAST_ECHO_TIMEOUT = 6.5;    // seconds (matches Python RNS)
    static constexpr double REVERSE_PEERING_INTERVAL = ANNOUNCE_INTERVAL * 3.25;  // ~5.2 seconds
    static constexpr double PEER_JOB_INTERVAL = 4.0;  // seconds (matches Python RNS)
    static constexpr size_t DEQUE_SIZE = 48;                 // packet dedup window
    static constexpr double DEQUE_TTL = 0.75;            // seconds
    static constexpr uint32_t BITRATE_GUESS = 10 * 1000 * 1000;
    static constexpr uint16_t HW_MTU = 1196;

    // Discovery token is full_hash(group_id + link_local_address) = 32 bytes
    // Python RNS sends and expects the full 32-byte hash (HASHLENGTH//8 = 256//8 = 32)
    static constexpr size_t TOKEN_SIZE = 32;

public:
    AutoInterface(const char* name = "AutoInterface");
    ~AutoInterface() override;

    // Configuration (call before start())
    void set_group_id(const std::string& group_id) { _group_id = group_id; }
    void set_discovery_port(uint16_t port) { _discovery_port = port; }
    void set_data_port(uint16_t port) { _data_port = port; }
    void set_interface_name(const std::string& ifname) { _ifname = ifname; }

    // InterfaceImpl overrides
    bool start() override;
    void stop() override;
    void loop() override;

    inline std::string toString() const override {
        return "AutoInterface[" + _name + "/" + _group_id + "]";
    }

    // Getters for testing
    const RNS::Bytes& get_discovery_token() const { return _discovery_token; }
    const RNS::Bytes& get_multicast_address() const { return _multicast_address_bytes; }
    size_t peer_count() const { return _peers.size(); }

    // Carrier state tracking (matches Python RNS)
    bool carrier_changed() {
        bool changed = _carrier_changed;
        _carrier_changed = false;  // Clear flag on read
        return changed;
    }
    void clear_carrier_changed() { _carrier_changed = false; }
    bool is_timed_out() const { return _timed_out; }

protected:
    void send_outgoing(const RNS::Bytes& data) override;

private:
    // Discovery and addressing
    void calculate_multicast_address();
    void calculate_discovery_token();
    bool get_link_local_address();

    // Socket operations
    bool setup_discovery_socket();
    bool setup_unicast_discovery_socket();
    bool setup_data_socket();
    bool join_multicast_group();

    // Main loop operations
    void send_announce();
    void process_discovery();
    void process_unicast_discovery();
    void send_reverse_peering();
    void reverse_announce(AutoInterfacePeer& peer);
    void process_data();
    void check_echo_timeout();
    void check_link_local_address();

    // Peer management
#ifdef ARDUINO
    void add_or_refresh_peer(const IPv6Address& addr, double timestamp);
#else
    void add_or_refresh_peer(const struct in6_addr& addr, double timestamp);
#endif
    void expire_stale_peers();

    // Deduplication
    bool is_duplicate(const RNS::Bytes& packet);
    void add_to_deque(const RNS::Bytes& packet);
    void expire_deque_entries();

    // Configuration
    std::string _group_id = DEFAULT_GROUP_ID;
    uint16_t _discovery_port = DEFAULT_DISCOVERY_PORT;
    uint16_t _unicast_discovery_port = DEFAULT_DISCOVERY_PORT + 1;  // 29717
    uint16_t _data_port = DEFAULT_DATA_PORT;
    std::string _ifname;  // Network interface name (e.g., "eth0", "wlan0")

    // Computed values
    RNS::Bytes _discovery_token;          // 16 bytes
    RNS::Bytes _multicast_address_bytes;  // 16 bytes (IPv6)
    struct in6_addr _multicast_address;
    struct in6_addr _link_local_address;
    std::string _link_local_address_str;
    std::string _multicast_address_str;   // For logging
    bool _data_socket_ok = false;         // Data socket initialized successfully
#ifdef ARDUINO
    IPv6Address _link_local_ip;           // ESP32: link-local as IPv6Address
    IPv6Address _multicast_ip;            // ESP32: multicast as IPv6Address
#endif

    // Sockets
#ifdef ARDUINO
    int _discovery_socket = -1;  // Raw socket for IPv6 multicast discovery
    int _unicast_discovery_socket = -1;  // Raw socket for unicast discovery (reverse peering)
    int _data_socket = -1;       // Raw socket for IPv6 unicast data (WiFiUDP doesn't support IPv6)
    unsigned int _if_index = 0;  // Interface index for scope_id
#else
    int _discovery_socket = -1;
    int _unicast_discovery_socket = -1;  // Socket for unicast discovery (reverse peering)
    int _data_socket = -1;
    unsigned int _if_index = 0;  // Interface index for multicast
#endif

    // Peers and state
    std::vector<AutoInterfacePeer> _peers;
    double _last_announce = 0;
    double _last_peer_job = 0;  // Timestamp of last peer job check

    // Echo tracking (matches Python RNS multicast_echoes / initial_echoes)
    double _last_multicast_echo = 0.0;       // Timestamp of last own echo received
    bool _initial_echo_received = false;      // True once first echo received
    bool _timed_out = false;                  // Current timeout state
    bool _carrier_changed = false;            // Flag for Transport layer notification
    bool _firewall_warning_logged = false;    // Track firewall warning (log once)

    // Deduplication: pairs of (packet_hash, timestamp)
    struct DequeEntry {
        RNS::Bytes hash;
        double timestamp;
    };
    std::deque<DequeEntry> _packet_deque;

    // Receive buffer
    RNS::Bytes _buffer;
};
