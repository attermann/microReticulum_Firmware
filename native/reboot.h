// Copyright (C) 2026, Chad Attermann
//
// Deferred-reboot mechanism for the native daemon. Mirrors embedded
// firmware's hard_reset() semantics: a KISS-driven config write (e.g.
// drot_conf_save, dia_conf_save, eeprom_erase) or an explicit CMD_RESET
// signals that the daemon should re-initialize from the updated EEPROM.
//
// On ESP32/NRF52 this is `ESP.restart()` / `NVIC_SystemReset()` — a hard
// reset of the MCU. On native, we mirror meshtasticd's pattern: schedule
// the reboot via a flag, let the current loop tick finish (so any pending
// KISS ACK can flush), then tear down host-side resources and call
// Portduino's `reboot()`, which execv()s argv[0]. The process image is
// replaced; portduinoSetup() and setup() run fresh.

#ifndef NATIVE_REBOOT_H
#define NATIVE_REBOOT_H

namespace native_reboot {

// Set the deferred-reboot flag. Idempotent; safe to call from any
// hard_reset() call site.
void request();

// True if a reboot has been requested but not yet performed. Checked at
// the top of the firmware's loop().
bool pending();

// Tear down host-side resources (KISS-TCP listener + client, libgpiod
// handles, SPI) and execv() argv[0]. Does not return.
[[noreturn]] void perform();

} // namespace native_reboot

// File-scope shims used by Utilities.h::hard_reset() and the .ino loop()
// to avoid pulling this header (under native/) into the global include
// path of the rest of the firmware. Each just delegates to the namespaced
// API above.
void native_request_reboot();
bool native_reboot_pending();
[[noreturn]] void native_reboot_perform();

#endif // NATIVE_REBOOT_H
