// Copyright (C) 2026, Chad Attermann

#include "config.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>

namespace native_config {

Config g_config;

namespace {

void trim(std::string& s) {
    auto issp = [](unsigned char c) { return std::isspace(c); };
    while (!s.empty() && issp(s.front())) s.erase(s.begin());
    while (!s.empty() && issp(s.back()))  s.pop_back();
}

int parse_int(const std::string& v, int fallback) {
    try { return std::stoi(v, nullptr, 0); } catch (...) { return fallback; }
}

uint32_t parse_u32(const std::string& v, uint32_t fallback) {
    try { return static_cast<uint32_t>(std::stoul(v, nullptr, 0)); }
    catch (...) { return fallback; }
}

float parse_float(const std::string& v, float fallback) {
    try { return std::stof(v); } catch (...) { return fallback; }
}

// Case-insensitive truthy parser. Accepts 1 / true / yes / on (and their
// uppercase / mixed-case variants) as true; 0 / false / no / off as false.
// Unrecognized values return the fallback so a typo doesn't silently flip
// a sensitive flag.
bool parse_bool(const std::string& v, bool fallback) {
    std::string lower = v;
    for (auto& c : lower) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    if (lower == "1" || lower == "true"  || lower == "yes" || lower == "on")  return true;
    if (lower == "0" || lower == "false" || lower == "no"  || lower == "off") return false;
    return fallback;
}

// Comma-separated pin list with optional ":low" / ":high" suffix per entry.
//   "22,25:low,33" -> [(22,true),(25,false),(33,true)]
// Entries that parse to a negative pin are silently dropped (mirrors the
// -1-is-disabled convention used elsewhere in this config).
void parse_pin_list(const std::string& v, std::vector<std::pair<int,bool>>& out) {
    out.clear();
    size_t start = 0;
    while (start <= v.size()) {
        size_t end = v.find(',', start);
        std::string tok = v.substr(start, end == std::string::npos ? std::string::npos : end - start);
        trim(tok);
        if (!tok.empty()) {
            bool active_high = true;
            auto colon = tok.find(':');
            std::string num_part = tok;
            if (colon != std::string::npos) {
                num_part = tok.substr(0, colon);
                std::string suffix = tok.substr(colon + 1);
                trim(num_part); trim(suffix);
                for (auto& c : suffix) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
                if (suffix == "low")  active_high = false;
                // anything else (including "high" or a typo) keeps the default
            }
            int pin = parse_int(num_part, -1);
            if (pin >= 0) out.emplace_back(pin, active_high);
        }
        if (end == std::string::npos) break;
        start = end + 1;
    }
}

} // namespace

bool load(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) {
        std::fprintf(stderr, "[config] %s not found; using defaults\n", path.c_str());
        return true;
    }

    std::string line;
    int lineno = 0;
    while (std::getline(f, line)) {
        ++lineno;
        // Strip trailing comment
        auto hash = line.find('#');
        if (hash != std::string::npos) line.erase(hash);
        trim(line);
        if (line.empty()) continue;

        auto eq = line.find('=');
        if (eq == std::string::npos) {
            std::fprintf(stderr, "[config] %s:%d: missing '='\n", path.c_str(), lineno);
            continue;
        }
        std::string k = line.substr(0, eq);
        std::string v = line.substr(eq + 1);
        trim(k); trim(v);

        if      (k == "data_dir")       g_config.data_dir = v;
        else if (k == "spi_dev")        g_config.spi_dev = v;
        else if (k == "gpio_chip")      g_config.gpio_chip = v;
        else if (k == "spi_speed_hz")   g_config.spi_speed_hz = parse_u32(v, g_config.spi_speed_hz);
        else if (k == "pin_cs")         g_config.pin_cs = parse_int(v, -1);
        else if (k == "pin_reset")      g_config.pin_reset = parse_int(v, -1);
        else if (k == "pin_busy")       g_config.pin_busy = parse_int(v, -1);
        else if (k == "pin_dio")        g_config.pin_dio = parse_int(v, -1);
        else if (k == "pin_rxen")       g_config.pin_rxen = parse_int(v, -1);
        else if (k == "pin_txen")       g_config.pin_txen = parse_int(v, -1);
        else if (k == "pin_tcxo_enable") g_config.pin_tcxo_enable = parse_int(v, -1);
        else if (k == "pin_sclk")       g_config.pin_sclk = parse_int(v, -1);
        else if (k == "pin_mosi")       g_config.pin_mosi = parse_int(v, -1);
        else if (k == "pin_miso")       g_config.pin_miso = parse_int(v, -1);
        else if (k == "pin_led_rx")     g_config.pin_led_rx = parse_int(v, -1);
        else if (k == "pin_led_tx")     g_config.pin_led_tx = parse_int(v, -1);
        else if (k == "radio_enable_pins") parse_pin_list(v, g_config.radio_enable_pins);
        else if (k == "dio3_tcxo_voltage") {
            // Mirrors meshtasticd: float volts wins; if absent/zero, fall
            // back to a boolean interpretation where "true" => 1.8 V.
            float fv = parse_float(v, 0.0f);
            if (fv > 0.0f) {
                g_config.dio3_tcxo_voltage = fv;
            } else if (parse_bool(v, false)) {
                g_config.dio3_tcxo_voltage = 1.8f;
            } else {
                g_config.dio3_tcxo_voltage = 0.0f;
            }
        }
        else if (k == "dio2_as_rf_switch") g_config.dio2_as_rf_switch = parse_bool(v, g_config.dio2_as_rf_switch);
        else if (k == "lora_freq_hz")   g_config.lora_freq_hz = parse_u32(v, g_config.lora_freq_hz);
        else if (k == "lora_bw_hz")     g_config.lora_bw_hz = parse_u32(v, g_config.lora_bw_hz);
        else if (k == "lora_sf")        g_config.lora_sf = static_cast<uint8_t>(parse_int(v, g_config.lora_sf));
        else if (k == "lora_cr")        g_config.lora_cr = static_cast<uint8_t>(parse_int(v, g_config.lora_cr));
        else if (k == "lora_txp")       g_config.lora_txp = static_cast<int8_t>(parse_int(v, g_config.lora_txp));
        else if (k == "modem") {
            // Accept either symbolic names or a numeric value matching Modem.h.
            if      (v == "SX1262") g_config.modem = 0x03;
            else if (v == "SX1276") g_config.modem = 0x01;
            else if (v == "SX1278") g_config.modem = 0x02;
            else if (v == "SX1280") g_config.modem = 0x04;
            else                    g_config.modem = static_cast<uint8_t>(parse_int(v, g_config.modem));
        }
        else if (k == "kiss_tcp_port")   g_config.kiss_tcp_port   = static_cast<uint16_t>(parse_int(v, g_config.kiss_tcp_port));
        else if (k == "kiss_tcp_public") g_config.kiss_tcp_public = parse_bool(v, g_config.kiss_tcp_public);
        else if (k == "kiss_ws_public")  g_config.kiss_ws_public  = parse_bool(v, g_config.kiss_ws_public);
        else if (k == "reboot_on_tx_failure") g_config.reboot_on_tx_failure = parse_bool(v, g_config.reboot_on_tx_failure);
        else {
            std::fprintf(stderr, "[config] %s:%d: unknown key '%s'\n",
                         path.c_str(), lineno, k.c_str());
        }
    }
    return true;
}

} // namespace native_config
