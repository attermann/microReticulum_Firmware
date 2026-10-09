// GPIO_Control.h — LXMF-based remote control for microReticulum transport nodes
// Receives text commands via LXMF messages, returns status.
//
// Commands (case-insensitive):
//   STATUS (STATO)       — Battery, uptime, packet counters, current RSSI
//   BATTERY              — Report battery voltage, percent, and charging state
//   ANNOUNCE [min]       — Show or set announce interval (1-1440 min)
//   HELP                 — Show command list

#ifndef GPIO_CONTROL_H
#define GPIO_CONTROL_H

#ifdef HAS_RNS

#include "LXMF_Minimal.h"
#include <Arduino.h>
#include <string>
#include <vector>
#include <algorithm>

// Battery globals (defined in Config.h, populated by Power.h)
extern bool    battery_ready;
extern float   battery_voltage;
extern float   battery_percent;
extern uint8_t battery_state;
extern bool    battery_installed;
extern bool    external_power;

// Radio/modem globals (Config.h) and packet counters (RNode_Firmware.ino)
extern int      current_rssi;
extern uint32_t radio_rx_packets;
extern uint32_t radio_tx_packets;

// ─── Helpers ────────────────────────────────────────────────────

// Uptime that survives the 49.7-day millis() rollover.
// uptime_tick() is called every loop by GPIOControl::loop().
static uint32_t up_last_ms = 0;
static uint32_t up_wraps   = 0;
static uint64_t uptime_tick() {
    uint32_t now = millis();
    if (now < up_last_ms) up_wraps++;
    up_last_ms = now;
    return ((uint64_t)up_wraps << 32) | now;
}

static const char* battery_state_str() {
    if (battery_state == 0x01) return "discharging";
    if (battery_state == 0x02) return "charging";
    if (battery_state == 0x03) return "charged";
    return "unknown";
}

static std::string str_upper(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), ::toupper);
    return s;
}

static std::string str_trim(const std::string& s) {
    size_t a = s.find_first_not_of(" \t\r\n");
    size_t b = s.find_last_not_of(" \t\r\n");
    if (a == std::string::npos) return "";
    return s.substr(a, b - a + 1);
}

static std::vector<std::string> tokenize(const std::string& s) {
    std::vector<std::string> t;
    std::string tok;
    for (char c : s) {
        if (c == ' ' || c == '\t') {
            if (!tok.empty()) { t.push_back(tok); tok.clear(); }
        } else { tok += c; }
    }
    if (!tok.empty()) t.push_back(tok);
    return t;
}

// Global LXMF pointer for use in ANNOUNCE command (set during GPIOControl::init)
static LXMFMinimal* g_lxmf = nullptr;

// ─── Command Handler ────────────────────────────────────────────

static std::string handle_command(const std::string& raw_command) {
    std::string cmd = str_trim(raw_command);
    if (cmd.empty()) return "ERR: Empty command";

    auto tokens = tokenize(str_upper(cmd));
    if (tokens.empty()) return "ERR: Empty command";
    std::string verb = tokens[0];

    // ── HELP ──
    if (verb == "HELP" || verb == "?") {
        return "Commands:\n"
               "STATUS (STATO)\n"
               "BATTERY\n"
               "ANNOUNCE [min]\n"
               "DUTY [ON [pct] | OFF]\n"
               "HELP";
    }

    // ── STATUS ──
    if (verb == "STATUS" || verb == "STATO") {
        char bat[48];
        if (!battery_ready) {
            snprintf(bat, sizeof(bat), "Batt n/a");
        } else if (!battery_installed) {
            snprintf(bat, sizeof(bat), "Batt none%s", external_power ? " (USB)" : "");
        } else {
            int mv = (int)(battery_voltage * 1000.0f + 0.5f);
            snprintf(bat, sizeof(bat), "Batt %d.%02dV %d%% %s",
                     mv / 1000, (mv % 1000) / 10, (int)(battery_percent + 0.5f),
                     battery_state_str());
        }

        uint64_t secs = uptime_tick() / 1000;
        unsigned long d = (unsigned long)(secs / 86400);
        unsigned int  h = (unsigned int)((secs % 86400) / 3600);
        unsigned int  m = (unsigned int)((secs % 3600) / 60);

        char rssi[24];
        if (current_rssi <= -292) snprintf(rssi, sizeof(rssi), "rssi n/a");
        else snprintf(rssi, sizeof(rssi), "rssi %d dBm", current_rssi);

        char buf[160];
        snprintf(buf, sizeof(buf), "%s | up %lud %uh %um | rx %lu tx %lu | %s",
                 bat, d, h, m,
                 (unsigned long)radio_rx_packets, (unsigned long)radio_tx_packets,
                 rssi);
        return std::string(buf);
    }

    // ── BATTERY ──
    if (verb == "BATTERY" || verb == "BAT") {
        if (!battery_ready) return "Battery: not available";
        if (!battery_installed) return external_power ? "Battery: none (USB powered)"
                                                      : "Battery: none";
        // integer formatting: float printf may be disabled in newlib-nano
        int mv = (int)(battery_voltage * 1000.0f + 0.5f);
        char buf[64];
        snprintf(buf, sizeof(buf), "Battery: %d.%02dV  %d%%  %s",
                 mv / 1000, (mv % 1000) / 10, (int)(battery_percent + 0.5f),
                 battery_state_str());
        return std::string(buf);
    }

    // ── ANNOUNCE ──
    if (verb == "ANNOUNCE" || verb == "ANN") {
        extern unsigned long transport_announce_interval;
        if (tokens.size() < 2) {
            // Report current intervals
            unsigned long lxmf_min = 0;
            unsigned long transport_min = transport_announce_interval / 60000;
            if (g_lxmf) {
                lxmf_min = g_lxmf->get_announce_interval() / 60000;
            }
            char buf[80];
            snprintf(buf, sizeof(buf), "Announce: LXMF %lu min, Transport %lu min",
                     lxmf_min, transport_min);
            return std::string(buf);
        }
        int minutes = 0;
        try { minutes = std::stoi(tokens[1]); }
        catch (...) { return "ERR: Bad number: " + tokens[1]; }
        if (minutes < 1 || minutes > 1440) return "ERR: Range 1-1440 min";
        unsigned long ms = (unsigned long)minutes * 60000UL;
        if (g_lxmf) {
            g_lxmf->set_announce_interval(ms);
        }
        transport_announce_interval = ms;
        char buf[48];
        snprintf(buf, sizeof(buf), "OK Announce interval: %d min", minutes);
        return std::string(buf);
    }

    // ── DUTY (EU duty-cycle airtime limit) ──
    if (verb == "DUTY" || verb == "DC") {
        extern float st_airtime_limit;
        extern float lt_airtime_limit;
        extern bool  airtime_lock;

        if (tokens.size() < 2) {
            bool on = (lt_airtime_limit > 0.0f) || (st_airtime_limit > 0.0f);
            char buf[96];
            snprintf(buf, sizeof(buf),
                     "Duty: %s  LT %.2f%%  ST %.2f%%%s",
                     on ? "ON" : "OFF",
                     lt_airtime_limit * 100.0f,
                     st_airtime_limit * 100.0f,
                     airtime_lock ? "  [LOCKED]" : "");
            return std::string(buf);
        }

        const std::string& arg = tokens[1];
        if (arg == "OFF" || arg == "0") {
            st_airtime_limit = 0.0f;
            lt_airtime_limit = 0.0f;
            return "OK Duty: OFF (no airtime limit)";
        }
        if (arg == "ON" || arg == "1") {
            float pct = 1.0f;  // EU 868 MHz regulatory default
            if (tokens.size() >= 3) {
                try { pct = std::stof(tokens[2]); }
                catch (...) { return "ERR: Bad percentage: " + tokens[2]; }
                if (pct <= 0.0f || pct >= 100.0f) return "ERR: Percent must be 0 < x < 100";
            }
            lt_airtime_limit = pct / 100.0f;
            char buf[64];
            snprintf(buf, sizeof(buf), "OK Duty: ON  LT %.2f%%", pct);
            return std::string(buf);
        }
        return "ERR: DUTY [ON [pct] | OFF]";
    }

    return "ERR: Unknown: " + verb + " (try HELP)";
}

// ─── Node Control Manager ───────────────────────────────────────

class GPIOControl {
public:
    static GPIOControl* _instance;

    GPIOControl() : _initialized(false) {}

    bool init(RNS::Identity& identity, const char* display_name = "LXMF Node") {
        Serial.println("[GPIO] Initializing...");
        _instance = this;

        if (!_lxmf.init(identity, display_name)) {
            Serial.println("[GPIO] LXMF init failed");
            return false;
        }

        // Set global LXMF pointer for command handlers
        g_lxmf = &_lxmf;

        // Register message handler
        _lxmf.set_message_handler(GPIOControl::on_lxmf_message);

        _initialized = true;

        Serial.println("[GPIO] Ready!");
        Serial.print("[GPIO] LXMF address: ");
        Serial.println(_lxmf.get_address().c_str());

        return true;
    }

    void loop() {
        uptime_tick();  // keep the rollover counter current
        if (!_initialized) return;
        _lxmf.loop();
    }

    std::string get_address() { return _lxmf.get_address(); }
    LXMFMinimal& lxmf() { return _lxmf; }

private:
    static std::string on_lxmf_message(const uint8_t* source_hash, const std::string& content) {
        if (!_instance) return "ERR: not initialized";

        Serial.print("[GPIO] Command: ");
        Serial.println(content.c_str());

        std::string response = handle_command(content);
        Serial.print("[GPIO] Response: ");
        Serial.println(response.c_str());

        return response;
    }

    LXMFMinimal _lxmf;
    bool        _initialized;
};

// Static instance pointer
GPIOControl* GPIOControl::_instance = nullptr;

#endif  // HAS_RNS
#endif  // GPIO_CONTROL_H
