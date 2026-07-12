#ifdef ENABLE_WEBSOCKETS

#include "base64.h"

namespace {

constexpr char kAlphabet[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

} // namespace

namespace ws_proto {

size_t base64_encode(const uint8_t* data, size_t len, char* out) {
    char* p = out;
    size_t i = 0;
    while (i + 3 <= len) {
        uint32_t v = (uint32_t(data[i]) << 16)
                   | (uint32_t(data[i + 1]) << 8)
                   |  uint32_t(data[i + 2]);
        *p++ = kAlphabet[(v >> 18) & 0x3f];
        *p++ = kAlphabet[(v >> 12) & 0x3f];
        *p++ = kAlphabet[(v >> 6)  & 0x3f];
        *p++ = kAlphabet[ v        & 0x3f];
        i += 3;
    }
    if (i < len) {
        uint32_t v = uint32_t(data[i]) << 16;
        if (i + 1 < len) v |= uint32_t(data[i + 1]) << 8;
        *p++ = kAlphabet[(v >> 18) & 0x3f];
        *p++ = kAlphabet[(v >> 12) & 0x3f];
        *p++ = (i + 1 < len) ? kAlphabet[(v >> 6) & 0x3f] : '=';
        *p++ = '=';
    }
    return static_cast<size_t>(p - out);
}

} // namespace ws_proto

#endif // ENABLE_WEBSOCKETS
