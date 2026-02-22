// GPIO_Control.h — LXMF-based GPIO control for microReticulum nodes
// Receives text commands via LXMF messages, controls GPIO pins, returns status.
//
// Commands (case-insensitive):
//   SET <pin> HIGH|LOW|ON|OFF|1|0  — Set output pin
//   GET <pin>                       — Read pin state
//   MODE <pin> IN|OUT               — Set pin direction
//   STATUS                          — Report all configured pin states
//   PINS                            — List available GPIO pins
//   HELP                            — Show command list

#ifndef GPIO_CONTROL_H
#define GPIO_CONTROL_H

#ifdef HAS_RNS

#include "LXMF_Minimal.h"
#include <Arduino.h>
#include <string>
#include <vector>
#include <algorithm>

// ─── Pin Configuration ──────────────────────────────────────────
// TTGO LoRa 32 v1 safe GPIO pins

struct GPIOPin {
    uint8_t number;
    bool    output_capable;
    bool    configured;
    uint8_t mode;
    const char* description;
};

static GPIOPin gpio_pins[] = {
    { 13, true,  false, INPUT, "GP13" },
    { 17, true,  false, INPUT, "GP17" },
    { 23, true,  false, INPUT, "GP23" },
    { 25, true,  false, INPUT, "GP25" },
    { 32, true,  false, INPUT, "GP32" },
    { 33, true,  false, INPUT, "GP33" },
    { 34, false, false, INPUT, "GP34 input-only" },
    { 35, false, false, INPUT, "GP35 input-only" },
    { 36, false, false, INPUT, "GP36 input-only" },
    { 39, false, false, INPUT, "GP39 input-only" },
};
static const int NUM_GPIO_PINS = sizeof(gpio_pins) / sizeof(gpio_pins[0]);

// ─── Helpers ────────────────────────────────────────────────────

static GPIOPin* find_pin(int pin_num) {
    for (int i = 0; i < NUM_GPIO_PINS; i++) {
        if (gpio_pins[i].number == pin_num) return &gpio_pins[i];
    }
    return nullptr;
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

// ─── Command Handler ────────────────────────────────────────────

static std::string handle_gpio_command(const std::string& raw_command) {
    std::string cmd = str_trim(raw_command);
    if (cmd.empty()) return "ERR: Empty command";

    auto tokens = tokenize(str_upper(cmd));
    if (tokens.empty()) return "ERR: Empty command";
    std::string verb = tokens[0];

    // ── HELP ──
    if (verb == "HELP" || verb == "?") {
        return "Commands:\n"
               "SET <pin> HIGH|LOW\n"
               "GET <pin>\n"
               "MODE <pin> IN|OUT\n"
               "STATUS\n"
               "PINS\n"
               "HELP";
    }

    // ── PINS ──
    if (verb == "PINS") {
        std::string r = "Pins:\n";
        for (int i = 0; i < NUM_GPIO_PINS; i++) {
            r += std::to_string(gpio_pins[i].number);
            r += gpio_pins[i].output_capable ? " I/O" : " IN";
            if (gpio_pins[i].configured) {
                r += gpio_pins[i].mode == OUTPUT ? " [OUT]" : " [IN]";
            }
            r += "\n";
        }
        return r;
    }

    // ── STATUS ──
    if (verb == "STATUS") {
        std::string r = "STATUS:";
        bool any = false;
        for (int i = 0; i < NUM_GPIO_PINS; i++) {
            if (gpio_pins[i].configured) {
                int v = digitalRead(gpio_pins[i].number);
                r += " " + std::to_string(gpio_pins[i].number) + "=" + (v == HIGH ? "HIGH" : "LOW");
                any = true;
            }
        }
        if (!any) r += " (no pins configured)";
        return r;
    }

    // ── Commands requiring pin number ──
    if (tokens.size() < 2) return "ERR: Missing pin number";

    int pin_num = -1;
    try { pin_num = std::stoi(tokens[1]); }
    catch (...) { return "ERR: Bad pin: " + tokens[1]; }

    GPIOPin* pin = find_pin(pin_num);
    if (!pin) return "ERR: Pin " + std::to_string(pin_num) + " unavailable";

    // ── GET ──
    if (verb == "GET") {
        int v = digitalRead(pin->number);
        return "OK GPIO " + std::to_string(pin->number) + ": " + (v == HIGH ? "HIGH" : "LOW");
    }

    // ── MODE ──
    if (verb == "MODE") {
        if (tokens.size() < 3) return "ERR: MODE needs IN or OUT";
        std::string dir = tokens[2];
        if (dir == "OUT" || dir == "OUTPUT") {
            if (!pin->output_capable) return "ERR: Pin " + std::to_string(pin->number) + " input-only";
            pinMode(pin->number, OUTPUT);
            pin->mode = OUTPUT; pin->configured = true;
            return "OK MODE " + std::to_string(pin->number) + ": OUT";
        }
        if (dir == "IN" || dir == "INPUT") {
            pinMode(pin->number, INPUT);
            pin->mode = INPUT; pin->configured = true;
            return "OK MODE " + std::to_string(pin->number) + ": IN";
        }
        return "ERR: Bad direction: " + dir;
    }

    // ── SET ──
    if (verb == "SET") {
        if (tokens.size() < 3) return "ERR: SET needs HIGH or LOW";
        if (!pin->output_capable) return "ERR: Pin " + std::to_string(pin->number) + " input-only";

        // Auto-configure as output
        if (!pin->configured || pin->mode != OUTPUT) {
            pinMode(pin->number, OUTPUT);
            pin->mode = OUTPUT; pin->configured = true;
        }

        std::string val = tokens[2];
        if (val == "HIGH" || val == "1" || val == "ON") {
            digitalWrite(pin->number, HIGH);
            return "OK GPIO " + std::to_string(pin->number) + ": HIGH";
        }
        if (val == "LOW" || val == "0" || val == "OFF") {
            digitalWrite(pin->number, LOW);
            return "OK GPIO " + std::to_string(pin->number) + ": LOW";
        }
        return "ERR: Bad value: " + val;
    }

    return "ERR: Unknown: " + verb + " (try HELP)";
}

// ─── GPIO Control Manager ───────────────────────────────────────

class GPIOControl {
public:
    // Static instance for the message handler callback
    static GPIOControl* _instance;

    GPIOControl() : _initialized(false) {}

    bool init(RNS::Identity& identity, const char* display_name = "GPIO Node") {
        Serial.println("[GPIO] Initializing...");
        _instance = this;

        if (!_lxmf.init(identity, display_name)) {
            Serial.println("[GPIO] LXMF init failed");
            return false;
        }

        // Register message handler (plain function pointer)
        _lxmf.set_message_handler(GPIOControl::on_lxmf_message);

        _initialized = true;

        Serial.println("[GPIO] Ready!");
        Serial.print("[GPIO] LXMF address: ");
        Serial.println(_lxmf.get_address().c_str());

        return true;
    }

    void loop() {
        if (!_initialized) return;
        _lxmf.loop();
    }

    std::string get_address() { return _lxmf.get_address(); }
    LXMFMinimal& lxmf() { return _lxmf; }

private:
    // Static callback: parse command, return reply string
    static std::string on_lxmf_message(const uint8_t* source_hash, const std::string& content) {
        if (!_instance) return "ERR: not initialized";

        Serial.print("[GPIO] Command: ");
        Serial.println(content.c_str());

        std::string response = handle_gpio_command(content);
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
