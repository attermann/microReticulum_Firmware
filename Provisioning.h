// Copyright (C) 2026, Chad Attermann

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

#ifndef RNODE_PROVISIONING_H
#define RNODE_PROVISIONING_H

#ifdef HAS_PROVISIONING

#include <Bytes.h>

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

// True while the on_log() callback should wrap log lines in a CMD_LOG
// KISS frame instead of writing them as plain text to Serial. Backed by
// the Provisioning general.kiss_enabled field; defaults to true so a
// KISS-aware host sees structured logs from the first boot.
extern bool log_kiss_enabled;

// Set true once Provisioning::Manager::begin() has run.
extern bool provisioning_started;

// Buffer for an in-flight CMD_PROVISION_REQ frame. Bytes are un-escaped
// into here by the serial_callback() byte-accumulator branch and handed
// to on_provision_request() at frame-end.
extern RNS::Bytes provision_rx_buf;

// Bring the Provisioning subsystem up. Must be called after the
// filesystem has been registered with RNS::Utilities::OS so storage
// reads can resolve.
void init_provisioning();

// Dispatch one un-escaped MsgPack envelope to the Provisioning Manager
// and emit the framed MsgPack response back over KISS.
void on_provision_request(const RNS::Bytes& req);

// Emit a CMD_PROVISION_RSP KISS frame carrying the given payload bytes.
void kiss_indicate_provision_response(const RNS::Bytes& payload);

// Emit a CMD_LOG KISS frame carrying one log line (timestamp + level +
// message), already composed by the caller.
void kiss_emit_log(const char* line, size_t len);

#endif // HAS_PROVISIONING

#endif // RNODE_PROVISIONING_H
