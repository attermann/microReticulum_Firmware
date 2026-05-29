// Native-target KISS-over-TCP transport. A single-client TCP server bound
// to 127.0.0.1, replacing the embedded firmware's USB-serial KISS channel
// on Portduino-backed builds.
//
// Lifecycle mirrors Remote.h's WiFiServer pattern: one listening socket,
// at most one active client, idle-timeout-driven reconnect. Second clients
// are rejected immediately by accepting + close()ing them on the spot.
//
// Logs (RNS log callback, printf redirect) DO NOT pass through this
// channel. They stay on Portduino's Serial (typically stdout). This keeps
// the KISS byte stream clean of ASCII log content.

#ifndef NATIVE_TCP_HOST_INTERFACE_H
#define NATIVE_TCP_HOST_INTERFACE_H

#include <cstdint>

namespace native_kiss_tcp {

// Create the listening socket. Returns false if bind/listen failed; the
// daemon should keep running anyway (no host connection just means no
// external control, like an embedded board with no USB cable attached).
//
// `bind_public`: when false, bind only on 127.0.0.1 (loopback, the safe
// default — no network exposure). When true, bind on 0.0.0.0 so remote
// hosts on the same network can connect. Public binding has no auth or
// encryption, so opt in only on trusted networks.
bool init(uint16_t port, bool bind_public);

// Per-loop housekeeping. Should be called from the firmware's main loop
// before draining bytes. Accepts any pending connection (or rejects it if
// one is already active), and refills the internal RX buffer from the
// kernel socket.
void poll_accept();

// Idle / dead-peer sweep. Drops the active client if it has gone silent
// past the idle window. Safe to call from a periodic tick.
void check_active();

// True if a client is currently attached and the connection looks healthy.
bool is_connected();

// True if at least one byte is currently available to read. Calling
// poll_accept() refills the buffer; available() does not block.
bool available();

// Non-blocking read of one byte from the staging buffer. Returns 0xC0
// (KISS FEND) if no data is available — same fallback as Remote.h so an
// accidental drain of an empty buffer yields a frame boundary rather
// than spurious data.
uint8_t read();

// Best-effort single-byte send to the active client. Drops the byte if
// no client is attached or the send fails (treats partial / EAGAIN as
// a fatal disconnect, since the host can't recover a half-sent frame).
void write(uint8_t byte);

} // namespace native_kiss_tcp

#endif
