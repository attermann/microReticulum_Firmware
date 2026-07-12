// Copyright (C) 2026, Chad Attermann

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

#ifdef HAS_PROVISIONING

#include "Provisioning.h"

//#include "Config.h"

#ifdef HAS_GPIO
#include "GPIO.h"
#endif

#include <microReticulum/Log.h>

// KISS framing constants. We don't include "Framing.h" because it defines
// the parser's module-state globals (IN_FRAME, ESCAPE, command, frame_len)
// at file scope without extern guards — pulling it into a second TU
// produces ODR clashes. The wire-format values below are protocol
// constants and must match Framing.h's definitions.
#define FEND              0xC0
#define CMD_LOG           0x80
#define CMD_PROVISION_RSP 0x87

#include <microReticulum/Transport.h>
#include <microReticulum/Reticulum.h>
#include <microReticulum/Interface.h>
#include <microReticulum/Identity.h>
#include <microReticulum/Destination.h>
#include <microReticulum/Provisioning/Provisioning.h>

#include <string>
#include <vector>

// lora_interface is always declared in RNode_Firmware.ino (constructed
// with RNS::Type::NONE), even when LORA_TRANSPORT is not defined. Its
// operator bool() returns true only after setup() assigns it a real
// LoRaInterface implementation — so a single runtime check works in
// both compile configurations.
#if defined(LORA_TRANSPORT)
extern RNS::Interface lora_interface;
extern uint32_t lora_freq;
extern uint32_t lora_bw;
extern int lora_sf;
extern int lora_cr;
extern int lora_txp;
extern uint32_t lora_bitrate;
extern uint8_t implicit_l;
extern int noise_floor;
extern int current_rssi;
extern int last_rssi;
extern uint8_t last_snr_raw;
extern float st_airtime_limit;
extern float lt_airtime_limit;
#endif
#if defined(UDP_TRANSPORT)
// udp_interface is only declared in RNode_Firmware.ino under UDP_TRANSPORT,
// so this extern (and its dependents) must stay behind the same guard.
extern RNS::Interface udp_interface;
extern IPAddress wr_device_ip;
extern uint16_t udp_port;
extern uint8_t wifi_mode;
extern char wr_ssid[33];
#endif
extern bool kiss_framed_logs;
extern bool nomadnet_enabled;
extern RNS::Destination nomadnet_destination;
extern char nomadnet_name[64];
extern float battery_voltage;
extern float battery_percent;
extern uint8_t battery_state;
extern void hard_reset(void);
extern void eeprom_conf_save();

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
// before Provisioner::begin().
//
// The "radio" namespace registration is kept here purely as reference —
// EEPROM is currently the source of truth for radio configuration and a
// future revival of Provisioning-backed radio config will need its own
// migration strategy. See git history around the original Provisioning
// integration for the prior wiring.
// ---------------------------------------------------------------------------
static void register_provisioning_namespaces() {
  using namespace RNS::Provisioning;

  // ----- General namespace -----
  auto general = Provisioner::instance()
    .register_namespace("RNode General Config", PROV_NS_GENERAL)
      .field_bool("Kiss Framed Logs", PROV_GENERAL_KISS_LOG, FF_LIVE_APPLY, kiss_framed_logs,
        [](const Value& v) { kiss_framed_logs = v.as_bool(); return true; },
        []() { return kiss_framed_logs; });

#ifdef URTN_STATS_PAGES
    general
      .field_bool("NomadNet Enabled", PROV_GENERAL_NOMADNET_ENABLE, FF_REBOOT_REQUIRED, nomadnet_enabled,
        [](const Value& v) { nomadnet_enabled = v.as_bool(); return true; },
        []() { return nomadnet_enabled; })
      .field_string("NomadNet Name", PROV_GENERAL_NOMADNET_NAME, FF_REBOOT_REQUIRED, nomadnet_name, sizeof(nomadnet_name)-1,
        [](const Value& v) { strncpy(nomadnet_name, v.as_string().c_str(), sizeof(nomadnet_name)); return true; },
        []() { return nomadnet_name; });
#endif

#ifdef HAS_GPIO
    general
/*
      .field_bool("Relay Enabled", PROV_GENERAL_GPIO0, FF_LIVE_APPLY, false,
        [](const Value& v) { v.as_bool() ? GPIO::setState(GPIO::GPIO0, GPIO::STATE_HIGH) : GPIO::setState(GPIO::GPIO0, GPIO::STATE_LOW); return true; },
        []() { return GPIO::isHigh(GPIO::GPIO0); })
      .metric_bool("Device Ready", PROV_GENERAL_GPIO1,
        []() { return GPIO::isHigh(GPIO::GPIO1); });
*/
      .field_enum("GPIO0", PROV_GENERAL_GPIO0, FF_LIVE_APPLY, GPIO::DISP_INPUT,
          {
            GPIO::DISP_INPUT,
            GPIO::DISP_INPUT_LOW,
            GPIO::DISP_INPUT_HIGH,
            GPIO::DISP_OUTPUT_LOW,
            GPIO::DISP_OUTPUT_HIGH,
          },
          {
            "INPUT",
            "INPUT_LOW",
            "INPUT_HIGH",
            "OUTPUT_LOW",
            "OUTPUT_HIGH",
          },
          [](const Value& v) {
            GPIO::setDisposition(GPIO::GPIO0, static_cast<GPIO::Disposition>(v.as_int())); return true;
          },
          []() {
            return static_cast<fint_t>(GPIO::getDisposition(GPIO::GPIO0));
          }
        )
      .field_enum("GPIO1", PROV_GENERAL_GPIO1, FF_LIVE_APPLY, GPIO::DISP_INPUT,
          {
            GPIO::DISP_INPUT,
            GPIO::DISP_INPUT_LOW,
            GPIO::DISP_INPUT_HIGH,
            GPIO::DISP_OUTPUT_LOW,
            GPIO::DISP_OUTPUT_HIGH,
          },
          {
            "INPUT",
            "INPUT_LOW",
            "INPUT_HIGH",
            "OUTPUT_LOW",
            "OUTPUT_HIGH",
          },
          [](const Value& v) {
            GPIO::setDisposition(GPIO::GPIO1, static_cast<GPIO::Disposition>(v.as_int())); return true;
          },
          []() {
            return static_cast<fint_t>(GPIO::getDisposition(GPIO::GPIO1));
          }
        );
#endif

#if defined(LORA_TRANSPORT)
  if (lora_interface) {
    general
      .field_enum(
          "LoRa Interface Mode", PROV_GENERAL_LORA_MODE, FF_LIVE_APPLY, static_cast<fint_t>(lora_interface.mode()),
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
          "UDP Interface Mode", PROV_GENERAL_UDP_MODE, FF_LIVE_APPLY, static_cast<fint_t>(udp_interface.mode()),
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
  auto metrics = Provisioner::instance().register_namespace("RNode General Metrics", PROV_NS_METRICS);

  metrics.register_namespace("Device", PROV_NS_METRICS_DEV)
    //.metric_string("transport_identity", PROV_METRICS_DEV_VER, []() { return std::to_string(MAJ_VERS) + "." + std::to_string(MIN_VERS); })
    .metric_float("Battery Voltage", PROV_METRICS_DEV_BATV, []() { return battery_voltage; })
    .metric_float("Battery Percent", PROV_METRICS_DEV_BATP, []() { return battery_percent; })
/*
    .metric_string("Battery State", PROV_METRICS_DEV_BATS, []() {
      switch (battery_state) {
        case BATTERY_STATE_CHARGING:
          return "CHARGING";
        case BATTERY_STATE_CHARGED:
          return "CHARGED";
        case BATTERY_STATE_DISCHARGING:
          return "DISCHARGING";
        case BATTERY_STATE_UNKNOWN:
          return "UNKNOWN";
        return "";
      }
    })
*/
    .end();

  metrics.register_namespace("Addresses", PROV_NS_METRICS_ADDRS)
    .metric_bytes("Transport Identity", PROV_METRICS_TRANS_ID, []() { return RNS::Transport::identity() ? RNS::Transport::identity().hash() : RNS::Bytes{}; })
    .metric_bytes("Probe Destination", PROV_METRICS_PROBE_DST, []() { return RNS::Transport::probe_destination() ? RNS::Transport::probe_destination().hash() : RNS::Bytes{}; })
    .metric_bytes("Mgmt Destination", PROV_METRICS_MGMT_DST, []() { return RNS::Transport::remote_management_destination() ? RNS::Transport::remote_management_destination().hash() : RNS::Bytes{}; })
    .metric_bytes("NomadNet Destination", PROV_METRICS_NOMAD_DST, []() { return nomadnet_destination ? nomadnet_destination.hash() : RNS::Bytes{}; })
    .end();

  auto metrics_ifaces = metrics.register_namespace("Interfaces", PROV_NS_METRICS_IFACE);
#if defined(LORA_TRANSPORT)
  if (lora_interface) {
    metrics_ifaces
      //.register_namespace("LoRa", PROV_NS_IFACE_LORA)
      .register_namespace(lora_interface.name().c_str(), PROV_NS_IFACE_LORA)
        .metric_int("Frequency", PROV_METRICS_LORA_FREQ, []() { return lora_freq; })
        .metric_int("Bandwidth", PROV_METRICS_LORA_BW, []() { return lora_bw; })
        .metric_int("Spreading Factor", PROV_METRICS_LORA_SF, []() { return lora_sf; })
        .metric_int("Coding Rate", PROV_METRICS_LORA_CR, []() { return lora_cr; })
        .metric_int("TX Power", PROV_METRICS_LORA_TXP, []() { return lora_txp; })
        //.metric_int("Current RSSI", PROV_METRICS_LORA_CRSSI, []() { return last_rssi+rssi_offset; })
        .metric_int("Current RSSI", PROV_METRICS_LORA_CRSSI, []() { return current_rssi; })
        .metric_int("Noise Floor", PROV_METRICS_LORA_NF, []() { return noise_floor; })
        .metric_int("Last RSSI", PROV_METRICS_LORA_LRSSI, []() { return last_rssi; })
        .metric_int("Last SNR", PROV_METRICS_LORA_LSNR, []() { return last_snr_raw; })
        .metric_float("ST Airtime Limit", PROV_METRICS_LORA_STAL, []() { return st_airtime_limit; })
        .metric_float("LT Airtime Limit", PROV_METRICS_LORA_LTAL, []() { return lt_airtime_limit; })
        .end();
  }
#endif
#if defined(UDP_TRANSPORT)
  if (udp_interface) {
    metrics_ifaces
      //.register_namespace("UDP", PROV_NS_IFACE_UDP)
      .register_namespace(udp_interface.name().c_str(), PROV_NS_IFACE_UDP)
        .metric_string("ip_addr", PROV_METRICS_UDP_ADDR, []() { return wr_device_ip.toString().c_str(); })
        .metric_int("udp_port", PROV_METRICS_UDP_PORT, []() { return udp_port; })
        .metric_string("wifi_ssid", PROV_METRICS_WIFI_SSID, []() { return wr_ssid; })
        .end();
  }
#endif
  metrics_ifaces.end(); // close "Interfaces"

  metrics.end();        // close "Metrics"

#if defined(LORA_TRANSPORT)
  // ----- Radio namespace (DISABLED) -----
  //
  Provisioner::instance()
    .register_namespace("RNode Radio Config", PROV_NS_RADIO)
      //.field_enum("op_mode", PROV_RADIO_OP_MODE, FF_REBOOT_REQUIRED,
      //           (fint_t)MODE_HOST,
      //           std::vector<fint_t>{ (fint_t)MODE_HOST, (fint_t)MODE_TNC },
      //           std::vector<std::string>{ "host", "tnc" },
      //           [](const Value& v) { op_mode = (uint8_t)v.as_int(); return true; })
      .field_int("Frequency", PROV_RADIO_FREQ, FF_REBOOT_REQUIRED,
        (fint_t)lora_freq, (fint_t)100000000, (fint_t)1000000000,
        [](const Value& v) { lora_freq = (uint32_t)v.as_int(); return true; })
      .field_int("Bandwidth", PROV_RADIO_BW, FF_REBOOT_REQUIRED,
        (fint_t)lora_bw, (fint_t)7800, (fint_t)500000,
        [](const Value& v) { lora_bw = (uint32_t)v.as_int(); return true; })
      .field_int("Spreading Factor", PROV_RADIO_SF, FF_REBOOT_REQUIRED,
        (fint_t)lora_sf, (fint_t)5, (fint_t)12,
        [](const Value& v) { lora_sf = (int)v.as_int(); return true; })
      .field_int("Coding Rate", PROV_RADIO_CR, FF_REBOOT_REQUIRED,
        (fint_t)lora_cr, (fint_t)5, (fint_t)8,
        [](const Value& v) { lora_cr = (int)v.as_int(); return true; })
      .field_int("TX Power", PROV_RADIO_TXP, FF_REBOOT_REQUIRED,
        (fint_t)lora_txp, (fint_t)-9, (fint_t)22,
        [](const Value& v) { lora_txp = (int)v.as_int(); return true; })
      .field_int("Implicit Length", PROV_RADIO_IMPLICIT, FF_REBOOT_REQUIRED,
        (fint_t)implicit_l, (fint_t)0, (fint_t)255,
        [](const Value& v) { implicit_l = (uint8_t)v.as_int(); return true; })
      .field_float("ST Airtime Limit", PROV_RADIO_STAL, FF_LIVE_APPLY,
        (ffloat_t)st_airtime_limit, (ffloat_t)0, (ffloat_t)1.0,
        [](const Value& v) { st_airtime_limit = (float)v.as_float(); return true; })
      .field_float("LT Airtime Limit", PROV_RADIO_LTAL, FF_LIVE_APPLY,
        (ffloat_t)lt_airtime_limit, (ffloat_t)0, (ffloat_t)1.0,
        [](const Value& v) { lt_airtime_limit = (float)v.as_float(); return true; })
      .on_commit([](Namespace& ns) {
        //TRACE("[provision] Radio commit\n");
        Value v;
        bool dirty = false;
        if (ns.draft(PROV_RADIO_FREQ, v)) {
          lora_freq = (uint32_t)v.as_int();
          //ns.clear_draft(PROV_RADIO_FREQ);
          dirty = true;
        }
        if (ns.draft(PROV_RADIO_BW, v)) {
          lora_bw = (uint32_t)v.as_int();
          //ns.clear_draft(PROV_RADIO_BW);
          dirty = true;
        }
        if (ns.draft(PROV_RADIO_SF, v)) {
          lora_sf = (uint32_t)v.as_int();
          //ns.clear_draft(PROV_RADIO_SF);
          dirty = true;
        }
        if (ns.draft(PROV_RADIO_CR, v)) {
          lora_cr = (uint32_t)v.as_int();
          //ns.clear_draft(PROV_RADIO_CR);
          dirty = true;
        }
        if (ns.draft(PROV_RADIO_TXP, v)) {
          lora_txp = (uint32_t)v.as_int();
          //ns.clear_draft(PROV_RADIO_TXP);
          dirty = true;
        }
        if (dirty) {
          //TRACE("[provision] Writing eeprom\n");
          eeprom_conf_save();
        }
      })
      .end();
#endif

#if defined(UDP_TRANSPORT)
  //if (wifi_mode != WR_WIFI_OFF && udp_interface) {
    Provisioner::instance()
      .register_namespace("RNode Network Config", PROV_NS_NETWORK)
        .field_string("IP Address", PROV_NET_IP, FF_REBOOT_REQUIRED,
          wr_device_ip.toString().c_str(), 15,
          [](const Value& v) { /*wr_device_ip = v.as_string();*/ return true; })
        .field_int("UDP Port", PROV_NET_PORT, FF_REBOOT_REQUIRED,
          (fint_t)udp_port, (fint_t)1024, (fint_t)65535,
          [](const Value& v) { udp_port = (uint32_t)v.as_int(); return true; })
        .field_string("WiFi SSID", PROV_NET_SSID, FF_REBOOT_REQUIRED,
          wr_ssid, 32,
          [](const Value& v) { strncpy(wr_ssid, v.as_string().c_str(), sizeof(wr_ssid)); return true; })
        .field_string("WiFi Mode", PROV_NET_MODE, FF_REBOOT_REQUIRED,
          std::to_string(wifi_mode).c_str(), 0,
          [](const Value& v) { return true; })
      .end();
  //}
#endif

}

// ---------------------------------------------------------------------------
// Bring the Provisioning subsystem up. Loads any persisted MsgPack files
// under /config (built-in Reticulum / Transport namespaces auto-register
// inside begin(); our general namespace is registered above). The
// on_reboot_required callback is wired up but intentionally a no-op —
// the host orchestrates reboots via CMD_RESET.
// ---------------------------------------------------------------------------
void init_provisioning() {
  RNS::Provisioning::Provisioner::instance().on_factory_reset([]() {
    // Not currently implemented
  });
  RNS::Provisioning::Provisioner::instance().on_reboot_required([]() {
    // Host orchestrates reboot via CMD_RESET. Provisioner::needs_reboot()
    // remains queryable via GetInfo for callers that want to surface
    // pending-reboot state.
  });
  RNS::Provisioning::Provisioner::instance().on_reboot([]() {
    hard_reset();
  });
  register_provisioning_namespaces();
  RNS::Provisioning::Provisioner::instance().begin();
  provisioning_started = true;
}

// ---------------------------------------------------------------------------
// Request / response over KISS
// ---------------------------------------------------------------------------
void on_provision_request(const RNS::Bytes& req) {
  if (!provisioning_started) return;
  RNS::Bytes response = RNS::Provisioning::Provisioner::instance().handle_message(req);
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
