// SHA-1 (RFC 3174) used by the WebSocket handshake to derive the
// Sec-WebSocket-Accept response from the client's Sec-WebSocket-Key.
// Portduino doesn't ship sha1, and attermann/Crypto only has SHA-2/3 —
// SHA-1 is deprecated for new uses but RFC 6455 mandates it here, so
// we vendor a self-contained implementation.

#ifndef WS_SHIMS_SHA1_H
#define WS_SHIMS_SHA1_H

#include <cstddef>
#include <cstdint>

namespace ws_proto {

// 20-byte digest of `data[0..len)` into `out[0..19]`.
void sha1(const uint8_t* data, size_t len, uint8_t out[20]);

} // namespace ws_proto

#endif
