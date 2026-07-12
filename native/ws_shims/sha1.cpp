#ifdef ENABLE_WEBSOCKETS

#include "sha1.h"

#include <cstring>

namespace {

inline uint32_t rotl(uint32_t v, int n) { return (v << n) | (v >> (32 - n)); }

struct Ctx {
    uint32_t h[5];
    uint64_t len_bits;
    uint8_t  buf[64];
    size_t   buf_len;
};

void init(Ctx& c) {
    c.h[0] = 0x67452301; c.h[1] = 0xEFCDAB89; c.h[2] = 0x98BADCFE;
    c.h[3] = 0x10325476; c.h[4] = 0xC3D2E1F0;
    c.len_bits = 0;
    c.buf_len = 0;
}

void block(Ctx& c, const uint8_t* in) {
    uint32_t w[80];
    for (int i = 0; i < 16; ++i) {
        w[i] = (uint32_t(in[i*4    ]) << 24)
             | (uint32_t(in[i*4 + 1]) << 16)
             | (uint32_t(in[i*4 + 2]) <<  8)
             |  uint32_t(in[i*4 + 3]);
    }
    for (int i = 16; i < 80; ++i) {
        w[i] = rotl(w[i-3] ^ w[i-8] ^ w[i-14] ^ w[i-16], 1);
    }
    uint32_t a = c.h[0], b = c.h[1], cc = c.h[2], d = c.h[3], e = c.h[4];
    for (int i = 0; i < 80; ++i) {
        uint32_t f, k;
        if      (i < 20) { f = (b & cc) | (~b & d);              k = 0x5A827999; }
        else if (i < 40) { f =  b ^ cc ^ d;                       k = 0x6ED9EBA1; }
        else if (i < 60) { f = (b & cc) | (b & d) | (cc & d);     k = 0x8F1BBCDC; }
        else             { f =  b ^ cc ^ d;                       k = 0xCA62C1D6; }
        uint32_t t = rotl(a, 5) + f + e + k + w[i];
        e = d; d = cc; cc = rotl(b, 30); b = a; a = t;
    }
    c.h[0] += a; c.h[1] += b; c.h[2] += cc; c.h[3] += d; c.h[4] += e;
}

void update(Ctx& c, const uint8_t* data, size_t len) {
    c.len_bits += uint64_t(len) * 8;
    while (len > 0) {
        size_t take = 64 - c.buf_len;
        if (take > len) take = len;
        std::memcpy(c.buf + c.buf_len, data, take);
        c.buf_len += take;
        data       += take;
        len        -= take;
        if (c.buf_len == 64) {
            block(c, c.buf);
            c.buf_len = 0;
        }
    }
}

void finalize(Ctx& c, uint8_t out[20]) {
    c.buf[c.buf_len++] = 0x80;
    if (c.buf_len > 56) {
        std::memset(c.buf + c.buf_len, 0, 64 - c.buf_len);
        block(c, c.buf);
        c.buf_len = 0;
    }
    std::memset(c.buf + c.buf_len, 0, 56 - c.buf_len);
    const uint64_t bits = c.len_bits;
    for (int i = 0; i < 8; ++i) {
        c.buf[56 + i] = uint8_t(bits >> (56 - i * 8));
    }
    block(c, c.buf);
    for (int i = 0; i < 5; ++i) {
        out[i*4    ] = uint8_t(c.h[i] >> 24);
        out[i*4 + 1] = uint8_t(c.h[i] >> 16);
        out[i*4 + 2] = uint8_t(c.h[i] >>  8);
        out[i*4 + 3] = uint8_t(c.h[i]      );
    }
}

} // namespace

namespace ws_proto {

void sha1(const uint8_t* data, size_t len, uint8_t out[20]) {
    Ctx c;
    init(c);
    update(c, data, len);
    finalize(c, out);
}

} // namespace ws_proto

#endif // ENABLE_WEBSOCKETS
