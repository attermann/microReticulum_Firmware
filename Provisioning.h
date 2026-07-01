// Copyright (C) 2026, Chad Attermann

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.


#pragma once

#ifdef HAS_PROVISIONING

#include <microReticulum/Bytes.h>

#include <stddef.h>
#include <stdint.h>

// Per-platform cap on a single inbound CMD_PROVISION_REQ payload. Sized
// to admit the largest plausible host request (SetState across all radio
// + general fields) without giving up too much RAM on tight nRF52 builds.
#if MCU_VARIANT == MCU_NRF52
  #define PROVISION_RX_BUF_MAX 512
#else
  #define PROVISION_RX_BUF_MAX 2048
#endif

// ---------------------------------------------------------------------------
// Provisioning namespace + field IDs.
//
// Namespace IDs 1-2 are RNS built-ins (Reticulum, Transport); 100-199
// are the official app range. PROV_NS_RADIO and its field IDs are kept
// here as a reference for the (currently disabled) radio namespace —
// EEPROM (driven by rnodeconf) remains the source of truth for radio
// configuration. See register_provisioning_namespaces() below.
//
// NOTE: **NEVER** change these values once they are in production. Only additions can be made.
// ---------------------------------------------------------------------------
#define PROV_NS_GENERAL         100
#define PROV_NS_RADIO           101
#define PROV_NS_NETWORK         102
#define PROV_NS_METRICS         103
#define PROV_NS_METRICS_IFACE   104
#define PROV_NS_IFACE_LORA      105
#define PROV_NS_IFACE_UDP       106
#define PROV_NS_METRICS_ADDRS   107
#define PROV_NS_METRICS_DEV     108

#define PROV_GENERAL_KISS_LOG        1
#define PROV_GENERAL_LORA_MODE       2
#define PROV_GENERAL_UDP_MODE        3
#define PROV_GENERAL_NOMADNET_ENABLE 4
#define PROV_GENERAL_NOMADNET_NAME   5

#define PROV_METRICS_TRANS_ID   1
#define PROV_METRICS_PROBE_DST  2
#define PROV_METRICS_MGMT_DST   3
#define PROV_METRICS_NOMAD_DST  4

#define PROV_METRICS_DEV_VER    1
#define PROV_METRICS_DEV_BATV   2
#define PROV_METRICS_DEV_BATP   3
#define PROV_METRICS_DEV_BATS   4

#define PROV_METRICS_LORA_FREQ  1
#define PROV_METRICS_LORA_BW    2
#define PROV_METRICS_LORA_SF    3
#define PROV_METRICS_LORA_CR    4
#define PROV_METRICS_LORA_TXP   5
#define PROV_METRICS_LORA_CRSSI 6  
#define PROV_METRICS_LORA_NF    7
#define PROV_METRICS_LORA_LRSSI 8
#define PROV_METRICS_LORA_LSNR  9
#define PROV_METRICS_LORA_STAL  10
#define PROV_METRICS_LORA_LTAL  11

#define PROV_METRICS_UDP_ADDR   1
#define PROV_METRICS_UDP_PORT   2
#define PROV_METRICS_WIFI_SSID  3

#define PROV_RADIO_OP_MODE      1
#define PROV_RADIO_FREQ         2
#define PROV_RADIO_BW           3
#define PROV_RADIO_SF           4
#define PROV_RADIO_CR           5
#define PROV_RADIO_TXP          6
#define PROV_RADIO_IMPLICIT     7
#define PROV_RADIO_STAL         8
#define PROV_RADIO_LTAL         9

#define PROV_NET_IP             1
#define PROV_NET_PORT           2
#define PROV_NET_SSID           3
#define PROV_NET_MODE           4

// Set true once Provisioning::Provisioner::begin() has run.
extern bool provisioning_started;

// Buffer for an in-flight CMD_PROVISION_REQ frame. Bytes are un-escaped
// into here by the serial_callback() byte-accumulator branch and handed
// to on_provision_request() at frame-end.
extern RNS::Bytes provision_rx_buf;

// Bring the Provisioning subsystem up. Must be called after the
// filesystem has been registered with RNS::Utilities::OS so storage
// reads can resolve.
void init_provisioning();

// Dispatch one un-escaped MsgPack envelope to the Provisioning Provisioner
// and emit the framed MsgPack response back over KISS.
void on_provision_request(const RNS::Bytes& req);

// Emit a CMD_PROVISION_RSP KISS frame carrying the given payload bytes.
void kiss_indicate_provision_response(const RNS::Bytes& payload);

#endif // HAS_PROVISIONING
