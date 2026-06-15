// Copyright (C) 2026, microReticulum_Firmware contributors
//
// Public declarations for native_pinmap. The implementation lives in
// PinMap.cpp; the matching pin_* extern globals are declared in Boards.h
// under the MCU_NATIVE branch.

#ifndef NATIVE_PINMAP_H
#define NATIVE_PINMAP_H

namespace native_pinmap {

void apply();
void bind_linux_gpios();
void release_linux_gpios();
void seed_eeprom_if_unprovisioned();

// Drive each pin in g_config.radio_enable_pins to its active level (HIGH
// by default; LOW for entries that were tagged ":low" in rnoded.conf).
// Called from startRadio() just before LoRa->begin() so external rails
// (LDOs, TCXO supply, PA bias) are stable before the SX126x probes them.
void assert_radio_enable_pins();

// Drive each pin in g_config.radio_enable_pins to its inactive level.
// Called from startRadio()'s failure path and from stopRadio() after
// LoRa->end() so SPI cleanup finishes while rails are still up.
void deassert_radio_enable_pins();

} // namespace native_pinmap

#endif // NATIVE_PINMAP_H
