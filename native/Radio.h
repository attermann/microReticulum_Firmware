// Copyright (C) 2026, microReticulum_Firmware contributors
//
// Native-daemon radio recovery: boot-time clean reset of the LoRa modem,
// runtime watchdog that detects a wedged chip and runs a soft-recovery
// cycle, and a clean-shutdown helper that parks the chip in reset on
// SIGINT/SIGTERM.
//
// All three entry points are no-ops when the boot-time reset is run on a
// platform without a configured reset pin, and `watchdog_tick()` is gated
// by `radio_watchdog_enabled` in rnoded.conf (default off). The shutdown
// flag is set from an async signal handler in native/main.cpp and polled
// from loop() in RNode_Firmware.ino.

#ifndef NATIVE_RADIO_H
#define NATIVE_RADIO_H

#include <csignal>

namespace native_radio {

// Drive pin_reset LOW for ~50 ms then HIGH and settle, restoring the
// SX126x to a known POR state before any SPI traffic. Safe to call
// multiple times; safe when pin_reset == -1 (returns immediately).
void clean_reset_at_boot();

// Periodic health probe. Rate-limited internally by
// radio_watchdog_interval_ms; no-op when the watchdog is disabled, the
// radio is offline, or the interval hasn't elapsed. Runs the recovery
// state machine on probe mismatch.
void watchdog_tick();

// Called from loop() once g_shutdown_requested is observed set. Stops
// the radio, drives NRESET LOW (chip parked in reset), and returns;
// caller is expected to exit immediately after.
void on_shutdown_signal();

// Set from the SIGINT/SIGTERM/SIGHUP handler installed by
// native/main.cpp; polled from loop().
extern volatile std::sig_atomic_t g_shutdown_requested;

} // namespace native_radio

#endif // NATIVE_RADIO_H
