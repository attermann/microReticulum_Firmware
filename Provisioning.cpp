// Copyright (C) 2026, Chad Attermann

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

#include "Provisioning.h"

#ifdef HAS_PROVISIONING

// KISS framing constants. We don't include "Framing.h" because it defines
// the parser's module-state globals (IN_FRAME, ESCAPE, command, frame_len)
// at file scope without extern guards — pulling it into a second TU
// produces ODR clashes. The wire-format values below are protocol
// constants and must match Framing.h's definitions.
#define FEND              0xC0
#define CMD_LOG           0x80
#define CMD_PROVISION_RSP 0x87

#include <Provisioning/Provisioning.h>

#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// External hooks into the rest of the firmware.
//
// serial_write / escaped_serial_write are defined inline in Utilities.h
// (compiled in the RNode_Firmware.ino TU). Forward-declaring them here
// avoids pulling Utilities.h — which is not include-guarded and contains
// file-scope globals — into a second translation unit.
//
// Radio config knobs and op_mode live in Config.h's global namespace and
// are only referenced by the (currently commented-out) radio namespace
// registration below. Pulling Config.h is enough since they're declared
// there at file scope.
// ---------------------------------------------------------------------------
extern void serial_write(uint8_t byte);
extern void escaped_serial_write(uint8_t byte);

// ---------------------------------------------------------------------------
// Public globals
// ---------------------------------------------------------------------------
bool log_kiss_enabled = true;
bool provisioning_started = false;
RNS::Bytes provision_rx_buf;

// ---------------------------------------------------------------------------
// Provisioning namespace + field IDs.
//
// Namespace IDs 1-2 are RNS built-ins (Reticulum, Transport); 100-199
// are the official app range. PROV_NS_RADIO and its field IDs are kept
// here as a reference for the (currently disabled) radio namespace —
// EEPROM (driven by rnodeconf) remains the source of truth for radio
// configuration. See register_provisioning_namespaces() below.
// ---------------------------------------------------------------------------
#define PROV_NS_RADIO         100
#define PROV_NS_GENERAL       101

// NOTE: **NEVER** change these values once they are in production. Only additions can be made.
#define PROV_RADIO_OP_MODE      1
#define PROV_RADIO_FREQ         2
#define PROV_RADIO_BW           3
#define PROV_RADIO_SF           4
#define PROV_RADIO_CR           5
#define PROV_RADIO_TXP          6
#define PROV_RADIO_IMPLICIT     7

#define PROV_GENERAL_KISS_LOG   1

// ---------------------------------------------------------------------------
// Register Provisioning namespaces. Called from init_provisioning()
// before Manager::begin().
//
// The "radio" namespace registration is kept here purely as reference —
// EEPROM is currently the source of truth for radio configuration and a
// future revival of Provisioning-backed radio config will need its own
// migration strategy. See git history around the original Provisioning
// integration for the prior wiring.
// ---------------------------------------------------------------------------
static void register_provisioning_namespaces() {
  using namespace RNS::Provisioning;

  // ----- radio namespace (DISABLED) -----
  //
  // Manager::instance()
  //   .register_namespace("radio", PROV_NS_RADIO)
  //     .field_enum("op_mode", PROV_RADIO_OP_MODE, FF_REBOOT_REQUIRED,
  //                (int64_t)MODE_HOST,
  //                std::vector<int64_t>{ (int64_t)MODE_HOST, (int64_t)MODE_TNC },
  //                std::vector<std::string>{ "host", "tnc" },
  //                [](const Value& v) { op_mode = (uint8_t)v.as_int(); return true; })
  //     .field_int("frequency", PROV_RADIO_FREQ, FF_REBOOT_REQUIRED,
  //                (int64_t)0, (int64_t)100000000, (int64_t)1000000000,
  //                [](const Value& v) { lora_freq = (uint32_t)v.as_int(); return true; })
  //     .field_int("bandwidth", PROV_RADIO_BW, FF_REBOOT_REQUIRED,
  //                (int64_t)0, (int64_t)7800, (int64_t)500000,
  //                [](const Value& v) { lora_bw = (uint32_t)v.as_int(); return true; })
  //     .field_int("sf", PROV_RADIO_SF, FF_REBOOT_REQUIRED,
  //                (int64_t)0, (int64_t)5, (int64_t)12,
  //                [](const Value& v) { lora_sf = (int)v.as_int(); return true; })
  //     .field_int("cr", PROV_RADIO_CR, FF_REBOOT_REQUIRED,
  //                (int64_t)5, (int64_t)5, (int64_t)8,
  //                [](const Value& v) { lora_cr = (int)v.as_int(); return true; })
  //     .field_int("txp", PROV_RADIO_TXP, FF_REBOOT_REQUIRED,
  //                (int64_t)0xFF, (int64_t)-9, (int64_t)22,
  //                [](const Value& v) { lora_txp = (int)v.as_int(); return true; })
  //     .field_int("implicit_l", PROV_RADIO_IMPLICIT, FF_REBOOT_REQUIRED,
  //                (int64_t)0, (int64_t)0, (int64_t)255,
  //                [](const Value& v) { implicit_l = (uint8_t)v.as_int(); return true; })
  //     .end();

  // ----- general namespace -----
  Manager::instance()
    .register_namespace("General", PROV_NS_GENERAL)
      .field_bool("kiss_enabled", PROV_GENERAL_KISS_LOG, FF_LIVE_APPLY, true,
                 [](const Value& v) { log_kiss_enabled = v.as_bool(); return true; })
      .end();
}

// ---------------------------------------------------------------------------
// Bring the Provisioning subsystem up. Loads any persisted MsgPack files
// under /config (built-in Reticulum / Transport namespaces auto-register
// inside begin(); our general namespace is registered above). The
// on_reboot_requested callback is wired up but intentionally a no-op —
// the host orchestrates reboots via CMD_RESET.
// ---------------------------------------------------------------------------
void init_provisioning() {
  RNS::Provisioning::Manager::instance().on_reboot_requested([]() {
    // Host orchestrates reboot via CMD_RESET. Manager::needs_reboot()
    // remains queryable via GetInfo for callers that want to surface
    // pending-reboot state.
  });
  register_provisioning_namespaces();
  RNS::Provisioning::Manager::instance().begin("/config");
  provisioning_started = true;
}

// ---------------------------------------------------------------------------
// Request / response over KISS
// ---------------------------------------------------------------------------
void on_provision_request(const RNS::Bytes& req) {
  if (!provisioning_started) return;
  RNS::Bytes response = RNS::Provisioning::Manager::instance().handle_message(req);
  kiss_indicate_provision_response(response);
}

void kiss_indicate_provision_response(const RNS::Bytes& payload) {
  serial_write(FEND);
  serial_write(CMD_PROVISION_RSP);
  const uint8_t* data = payload.data();
  size_t n = payload.size();
  for (size_t i = 0; i < n; ++i) escaped_serial_write(data[i]);
  serial_write(FEND);
}

// ---------------------------------------------------------------------------
// Log line over KISS — invoked by on_log() in RNode_Firmware.ino when
// log_kiss_enabled is true.
// ---------------------------------------------------------------------------
void kiss_emit_log(const char* line, size_t len) {
  serial_write(FEND);
  serial_write(CMD_LOG);
  for (size_t i = 0; i < len; ++i) escaped_serial_write((uint8_t)line[i]);
  serial_write(FEND);
}

#endif // HAS_PROVISIONING
