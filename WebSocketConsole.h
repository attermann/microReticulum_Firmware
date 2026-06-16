// WebSocketConsole — KISS↔WebSocket bridge for the embedded web console.
//
// Sits between WebSocketServer (RFC 6455 frames) and the firmware's KISS
// I/O. One WebSocket binary frame == one KISS frame on the wire, including
// the leading and trailing FEND markers.
//
// Outbound (firmware → browser):
//   * Hook into Utilities.h::serial_write() via ws_console::on_serial_write(b).
//   * We buffer bytes between KISS frame boundaries (FEND) and emit one
//     binary WebSocket frame per complete KISS frame.
//
// Inbound (browser → firmware):
//   * Each binary WS frame is treated as one KISS frame and pushed into
//     serialFIFO; serial_poll() in RNode_Firmware.ino then parses it.
//   * Text frames are not currently used (KISS is binary). They're
//     accepted by WebSocketServer but ignored here.
//
// Single-client by design — matches Remote.h and TCPHostInterface.h.

#ifndef WEBSOCKET_CONSOLE_H
#define WEBSOCKET_CONSOLE_H

#if defined(ENABLE_WEBSOCKETS) && __has_include(<WiFi.h>)

#include <cstdint>

namespace ws_console {

// Initialize on the given TCP port (typically 81 alongside HTTP on 80).
// bind_public is honored only on native (Portduino) — false binds the
// listening socket to 127.0.0.1, true to 0.0.0.0. ESP32 always binds
// 0.0.0.0 regardless (the console is served over WiFi AP).
// Safe to call multiple times — second and subsequent calls no-op.
void init(uint16_t port, bool bind_public = false);

// Drive the underlying WebSocketServer. Call once per main-loop tick.
// Non-blocking.
void service();

// Outbound tap from serial_write(). Buffers bytes between KISS FEND
// markers and flushes a complete KISS frame as a single WS binary frame
// when a client is attached. Bytes are dropped on the floor when no
// client is attached — same behavior as the existing wifi/native TCP
// transports.
void on_serial_write(uint8_t byte);

// True when a client is connected and the WS handshake is complete.
bool client_attached();

} // namespace ws_console

#endif // ENABLE_WEBSOCKETS && __has_include(<WiFi.h>)
#endif // WEBSOCKET_CONSOLE_H
