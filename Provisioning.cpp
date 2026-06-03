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

//#include "Config.h"

#ifdef HAS_PROVISIONING

// KISS framing constants. We don't include "Framing.h" because it defines
// the parser's module-state globals (IN_FRAME, ESCAPE, command, frame_len)
// at file scope without extern guards — pulling it into a second TU
// produces ODR clashes. The wire-format values below are protocol
// constants and must match Framing.h's definitions.
#define FEND              0xC0
#define CMD_LOG           0x80
#define CMD_PROVISION_RSP 0x87

#include <Interface.h>
#include <Provisioning/Provisioning.h>

#include <string>
#include <vector>

// lora_interface is always declared in RNode_Firmware.ino (constructed
// with RNS::Type::NONE), even when LORA_TRANSPORT is not defined. Its
// operator bool() returns true only after setup() assigns it a real
// LoRaInterface implementation — so a single runtime check works in
// both compile configurations.
extern RNS::Interface lora_interface;
extern int lora_sf;
extern int lora_cr;
extern int lora_txp;
extern uint32_t lora_bw;
extern uint32_t lora_freq;
extern uint32_t lora_bitrate;
extern int noise_floor;
extern int current_rssi;
extern int last_rssi;
extern uint8_t last_rssi_raw;
extern uint8_t last_snr_raw;
#if defined(UDP_TRANSPORT)
// udp_interface is only declared in RNode_Firmware.ino under UDP_TRANSPORT,
// so this extern (and its dependents) must stay behind the same guard.
extern RNS::Interface udp_interface;
extern IPAddress wr_device_ip;
extern uint16_t udp_port;
extern uint8_t wifi_mode;
extern char wr_ssid[];
#endif
extern bool kiss_framed_logs;

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
bool provisioning_started = false;
RNS::Bytes provision_rx_buf;

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

  // ----- general namespace -----
  auto general = Manager::instance()
    .register_namespace("General", PROV_NS_GENERAL)
      .field_bool("kiss_framed_logs", PROV_GENERAL_KISS_LOG, FF_LIVE_APPLY, true,
        [](const Value& v) { kiss_framed_logs = v.as_bool(); return true; },
        []() { return kiss_framed_logs; });

#ifdef URTN_STATS_PAGES
    general
      .field_bool("enable_nomadnet", PROV_GENERAL_NOMADNET, FF_REBOOT_REQUIRED, true);
#endif

#if defined(LORA_TRANSPORT)
  if (lora_interface) {
    general
      .field_enum(
          "lora_interface_mode", PROV_GENERAL_LORA_MODE, FF_LIVE_APPLY,
          (fint_t)RNS::Type::Interface::MODE_GATEWAY,
          /* values   */ {
            RNS::Type::Interface::MODE_GATEWAY,
            RNS::Type::Interface::MODE_FULL,
            RNS::Type::Interface::MODE_POINT_TO_POINT,
            RNS::Type::Interface::MODE_ACCESS_POINT,
            RNS::Type::Interface::MODE_ROAMING,
            RNS::Type::Interface::MODE_BOUNDARY,
          },
          /* labels   */ {
            "gateway",
            "full",
            "point-to-point",
            "access-point",
            "roaming",
            "boundary" },
          /* setter   */ [](const Value& v) {
            lora_interface.mode(static_cast<RNS::Type::Interface::modes>(v.as_int())); return true;
          },
          /* getter   */ []() {
            return static_cast<fint_t>(lora_interface.mode());
          }
      );
  }
#endif

#if defined(UDP_TRANSPORT)
  if (udp_interface) {
    general
      .field_enum(
          "udp_interface_mode", PROV_GENERAL_UDP_MODE, FF_LIVE_APPLY,
          (fint_t)RNS::Type::Interface::MODE_GATEWAY,
          /* values   */ {
            RNS::Type::Interface::MODE_GATEWAY,
            RNS::Type::Interface::MODE_FULL,
            RNS::Type::Interface::MODE_POINT_TO_POINT,
            RNS::Type::Interface::MODE_ACCESS_POINT,
            RNS::Type::Interface::MODE_ROAMING,
            RNS::Type::Interface::MODE_BOUNDARY,
          },
          /* labels   */ {
            "gateway",
            "full",
            "point-to-point",
            "access-point",
            "roaming",
            "boundary" },
          /* setter   */ [](const Value& v) {
            udp_interface.mode(static_cast<RNS::Type::Interface::modes>(v.as_int())); return true;
          },
          /* getter   */ []() {
            return static_cast<fint_t>(udp_interface.mode());
          }
      );
  }
#endif

  general
    .end();   // close "General"

  // ----- Metrics namespace -----
  //
  // The Metrics > Interfaces parent chain is opened unconditionally; the
  // per-interface child namespaces are added only when the corresponding
  // interface object reports it has a live implementation (operator bool
  // on RNS::Interface). Compile-time guards remain only where they need
  // to — UDP's externs aren't declared without UDP_TRANSPORT.
  auto metrics_ifaces = Manager::instance()
      .namespace_("Metrics", PROV_NS_METRICS)
        .namespace_("Interfaces", PROV_NS_METRICS_IFACE);

#if defined(LORA_TRANSPORT)
  if (lora_interface) {
    metrics_ifaces
      //.namespace_("LoRa", PROV_NS_IFACE_LORA)
      .namespace_(lora_interface.name().c_str(), PROV_NS_IFACE_LORA)
        .metric_int("frequency", PROV_METRICS_LORA_FREQ, []() { return lora_freq; })
        .metric_int("bandwidth", PROV_METRICS_LORA_BW, []() { return lora_bw; })
        .metric_int("spreading_factor", PROV_METRICS_LORA_SF, []() { return lora_sf; })
        .metric_int("coding_rate", PROV_METRICS_LORA_CR, []() { return lora_cr; })
        .metric_int("tx_power", PROV_METRICS_LORA_TXP, []() { return lora_txp; })
        //.metric_int("current_rssi", PROV_METRICS_LORA_CRSSI, []() { return last_rssi+rssi_offset; })
        .metric_int("current_rssi", PROV_METRICS_LORA_CRSSI, []() { return current_rssi; })
        .metric_int("noise_floor", PROV_METRICS_LORA_NF, []() { return noise_floor; })
        .metric_int("last_rssi", PROV_METRICS_LORA_LRSSI, []() { return last_rssi+157; })
        .metric_int("last_snr", PROV_METRICS_LORA_LSNR, []() { return last_snr_raw; })
        .end();
  }
#endif

#if defined(UDP_TRANSPORT)
  if (udp_interface) {
    metrics_ifaces
      //.namespace_("UDP", PROV_NS_IFACE_UDP)
      .namespace_(udp_interface.name().c_str(), PROV_NS_IFACE_LORA)
        .metric_string("ip_addr", PROV_METRICS_UDP_ADDR, []() { return wr_device_ip.toString().c_str(); })
        .metric_int("udp_port", PROV_METRICS_UDP_PORT, []() { return udp_port; })
        .metric_string("wifi_ssid", PROV_METRICS_WIFI_SSID, []() { return wr_ssid; })
        .end();
  }
#endif

  metrics_ifaces
      .end()  // close "Interfaces"
    .end();   // close "Metrics"

  // ----- radio namespace (DISABLED) -----
  //
  // Manager::instance()
  //   .register_namespace("radio", PROV_NS_RADIO)
  //     .field_enum("op_mode", PROV_RADIO_OP_MODE, FF_REBOOT_REQUIRED,
  //                (fint_t)MODE_HOST,
  //                std::vector<fint_t>{ (fint_t)MODE_HOST, (fint_t)MODE_TNC },
  //                std::vector<std::string>{ "host", "tnc" },
  //                [](const Value& v) { op_mode = (uint8_t)v.as_int(); return true; })
  //     .field_int("frequency", PROV_RADIO_FREQ, FF_REBOOT_REQUIRED,
  //                (fint_t)0, (fint_t)100000000, (fint_t)1000000000,
  //                [](const Value& v) { lora_freq = (uint32_t)v.as_int(); return true; })
  //     .field_int("bandwidth", PROV_RADIO_BW, FF_REBOOT_REQUIRED,
  //                (fint_t)0, (fint_t)7800, (fint_t)500000,
  //                [](const Value& v) { lora_bw = (uint32_t)v.as_int(); return true; })
  //     .field_int("sf", PROV_RADIO_SF, FF_REBOOT_REQUIRED,
  //                (fint_t)0, (fint_t)5, (fint_t)12,
  //                [](const Value& v) { lora_sf = (int)v.as_int(); return true; })
  //     .field_int("cr", PROV_RADIO_CR, FF_REBOOT_REQUIRED,
  //                (fint_t)5, (fint_t)5, (fint_t)8,
  //                [](const Value& v) { lora_cr = (int)v.as_int(); return true; })
  //     .field_int("txp", PROV_RADIO_TXP, FF_REBOOT_REQUIRED,
  //                (fint_t)0xFF, (fint_t)-9, (fint_t)22,
  //                [](const Value& v) { lora_txp = (int)v.as_int(); return true; })
  //     .field_int("implicit_l", PROV_RADIO_IMPLICIT, FF_REBOOT_REQUIRED,
  //                (fint_t)0, (fint_t)0, (fint_t)255,
  //                [](const Value& v) { implicit_l = (uint8_t)v.as_int(); return true; })
  //     .end();
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

#endif // HAS_PROVISIONING
