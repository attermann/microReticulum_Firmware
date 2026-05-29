// Copyright (C) 2026, microReticulum_Firmware contributors

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
        else {
            std::fprintf(stderr, "[config] %s:%d: unknown key '%s'\n",
                         path.c_str(), lineno, k.c_str());
        }
    }
    return true;
}

} // namespace native_config
