// Copyright (C) 2026, microReticulum_Firmware contributors

#include "Radio.h"

#include "config.h"
#include "reboot.h"
#include "../LoRaRadio.h"

#include <Arduino.h>     // pinMode / digitalWrite / millis / HIGH / LOW / OUTPUT

#include <cstdio>

// Pin globals defined in native/PinMap.cpp.
extern int pin_reset;

// LoRa pointer + radio_online flag live in the main .ino TU (via
// Utilities.h on the native target). Declared extern here so the
// watchdog can poll them without dragging in Utilities.h / Config.h
// (both contain definitions that would conflict at link time).
extern ILoRaRadio* LoRa;
extern bool        radio_online;

// startRadio() / stopRadio() are defined in RNode_Firmware.ino.
extern bool startRadio();
extern void stopRadio();

namespace native_radio {

volatile std::sig_atomic_t g_shutdown_requested = 0;

namespace {

uint32_t last_probe_ms       = 0;
uint32_t consecutive_failures = 0;

void pulse_nreset(uint32_t low_ms, uint32_t high_settle_ms) {
    if (pin_reset < 0) return;
    pinMode(pin_reset, OUTPUT);
    digitalWrite(pin_reset, LOW);
    delay(low_ms);
    digitalWrite(pin_reset, HIGH);
    delay(high_settle_ms);
}

} // namespace

void clean_reset_at_boot() {
    if (pin_reset < 0) {
        // No reset pin configured (e.g. cross_platform / macOS sim, or a
        // board that ties NRESET directly to host MCU reset). Boot-time
        // reset is silently skipped — the modem driver's own reset() in
        // begin() still runs.
        return;
    }
    std::fprintf(stderr, "[radio] clean reset at boot (NRESET low 50ms, settle 20ms)\n");
    pulse_nreset(50, 20);
}

void watchdog_tick() {
    const auto& cfg = native_config::g_config;
    if (!cfg.radio_watchdog_enabled) return;
    if (!radio_online) return;
    if (LoRa == nullptr) return;

    uint32_t now = millis();
    if (now - last_probe_ms < cfg.radio_watchdog_interval_ms) return;
    last_probe_ms = now;

    if (LoRa->isResponding()) {
        consecutive_failures = 0;
        return;
    }

    consecutive_failures++;
    std::fprintf(stderr,
                 "[radio-watchdog] chip not responding (attempt %u/%u) — recovering\n",
                 consecutive_failures, cfg.radio_max_recovery_attempts);

    if (consecutive_failures > cfg.radio_max_recovery_attempts) {
        std::fprintf(stderr, "[radio-watchdog] giving up — escalating to hard_reset\n");
        consecutive_failures = 0;
        native_request_reboot();
        return;
    }

    stopRadio();
    pulse_nreset(50, 20);
    if (!startRadio()) {
        std::fprintf(stderr, "[radio-watchdog] startRadio() failed after reset\n");
    }
}

void on_shutdown_signal() {
    std::fprintf(stderr, "[radio] shutdown signal — parking chip in reset\n");
    if (radio_online) stopRadio();
    if (pin_reset >= 0) {
        pinMode(pin_reset, OUTPUT);
        digitalWrite(pin_reset, LOW);
    }
}

} // namespace native_radio
