// base64 encoder used by the WebSocket handshake (Sec-WebSocket-Accept).
// Portduino doesn't ship a base64.h; the ESP32 Arduino framework does, but
// keeping our own keeps the WS code portable across both targets without
// platform #ifdefs.

#ifndef WS_SHIMS_BASE64_H
#define WS_SHIMS_BASE64_H

#include <cstddef>
#include <cstdint>

namespace ws_proto {

// Encode `len` bytes from `data`. Writes ((len+2)/3)*4 characters into
// `out` (NUL not written, caller sizes the buffer). Returns the number
// of characters written.
size_t base64_encode(const uint8_t* data, size_t len, char* out);

} // namespace ws_proto

#endif
