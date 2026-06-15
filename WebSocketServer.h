// Cross-platform RFC 6455 WebSocket server, hand-rolled to sit on top of
// stock Arduino `WiFiServer`/`WiFiClient`. Works as-is on ESP32 (Arduino
// framework) and on native Portduino — no library shims, no preprocessor
// platform forks inside this file.
//
// Scope: server-only, single client at a time, RFC 6455 message
// fragmentation supported, no SSL, payload capped at PAYLOAD_CAP bytes.
// That's enough for KISS frames (LoRa MTU is sub-300 bytes) and matches
// the firmware's existing single-host model (Remote.h / TCPHostInterface
// both reject second connections).
//
// Reads are done exclusively via `client_.read()` (single byte). This is
// deliberate: Portduino's `WiFiClient::available()` does a destructive
// 1-byte peek into a private member that `WiFiClient::read(buf,size)`
// then ignores, dropping the first byte of every burst. The single-byte
// `read()` path consumes the stash correctly, so by staying on it we
// avoid the bug on Portduino without a WiFiClient subclass workaround.
// Per-byte overhead is fine for KISS data rates.

#ifndef WEBSOCKET_SERVER_H
#define WEBSOCKET_SERVER_H

// Two-level gate: ENABLE_WEBSOCKETS is the user-facing build flag set in
// platformio.ini; the __has_include guard auto-disables the feature on
// platforms whose Arduino framework doesn't ship <WiFi.h> (e.g. nRF52).
// This lets us set the flag once in [env:embedded] without breaking
// non-WiFi targets.
#if defined(ENABLE_WEBSOCKETS) && __has_include(<WiFi.h>)

#include <WiFi.h>
#include <cstdint>
#include <cstddef>

class WebSocketServer {
public:
    static constexpr size_t REQ_BUF_CAP    = 1024;  // handshake request
    static constexpr size_t PAYLOAD_CAP    = 2048;  // per-message accumulator
    static constexpr size_t CTRL_PAYLOAD_CAP = 125; // RFC 6455 §5.5
    static constexpr uint16_t CLOSE_NORMAL = 1000;
    static constexpr uint16_t CLOSE_PROTO  = 1002;
    static constexpr uint16_t CLOSE_TOOBIG = 1009;

    explicit WebSocketServer(uint16_t port) : server_(port), port_(port) {}

    void begin();
    void service();  // call once per main-loop tick

    bool connected() const { return state_ == State::OPEN; }

    // Send a complete frame (FIN=1, no masking — server side).
    // Returns false if not currently OPEN or if the write call failed.
    bool send_text  (const char* text);
    bool send_binary(const uint8_t* data, size_t len);

    // Inbound message hook. Invoked once per complete data message
    // (after any fragmentation has been reassembled). `is_text`
    // distinguishes opcode 0x1 (text) vs 0x2 (binary).
    using MessageCb = void (*)(const uint8_t* data, size_t len,
                               bool is_text, void* ctx);
    void on_message(MessageCb cb, void* ctx = nullptr) {
        on_message_     = cb;
        on_message_ctx_ = ctx;
    }

private:
    enum class State : uint8_t {
        WAITING,       // no client; accept loop active
        HANDSHAKING,   // client attached; parsing HTTP upgrade
        OPEN,          // handshake complete; framing live
        CLOSING,       // close handshake in progress; teardown next tick
    };

    // Frame-parser cursor. We advance one byte at a time, so the state
    // tells us what the next byte is.
    enum class RxPhase : uint8_t {
        HEAD_0,        // FIN + reserved + opcode
        HEAD_1,        // MASK + 7-bit length
        EXT_LEN,       // 2 or 8 bytes of extended length
        MASK,          // 4 bytes of mask key
        PAYLOAD,       // payload bytes (unmasked on the fly)
    };

    void drive_accept();
    void drive_handshake();
    void drive_frame();

    bool finish_handshake();   // parse req_buf_, write 101 or 400
    void dispatch_frame();     // dispatch the just-completed frame
    bool send_frame(uint8_t opcode, const uint8_t* data, size_t len);
    void send_close(uint16_t code);
    void teardown();
    void reset_frame_rx();     // frame-scoped state, NOT message-scoped

    WiFiServer server_;
    uint16_t   port_;
    WiFiClient client_;
    State      state_         = State::WAITING;
    MessageCb  on_message_    = nullptr;
    void*      on_message_ctx_= nullptr;

    // Handshake buffer
    uint8_t req_buf_[REQ_BUF_CAP];
    size_t  req_len_ = 0;

    // ---- Per-frame state (reset between frames) ----
    RxPhase  rx_phase_         = RxPhase::HEAD_0;
    uint8_t  rx_opcode_        = 0;       // raw opcode of the in-flight frame
    bool     rx_fin_           = false;
    bool     rx_is_ctrl_       = false;
    bool     rx_masked_        = false;
    uint8_t  rx_ext_remaining_ = 0;
    uint8_t  rx_mask_pos_      = 0;
    uint8_t  rx_mask_[4]       = {0};
    uint32_t rx_payload_pos_   = 0;       // within current frame's payload
    uint32_t rx_payload_total_ = 0;

    // ---- Per-message state (persists across fragments) ----
    // 0 = no message in progress; 0x1/0x2 = TEXT/BIN message being built.
    uint8_t  msg_opcode_       = 0;
    uint32_t msg_len_          = 0;       // bytes accumulated so far
    uint8_t  payload_[PAYLOAD_CAP];

    // Control-frame payload buffer. Per RFC 6455 §5.5, control frames
    // may not be fragmented and carry ≤125-byte payloads. They can be
    // interleaved with data fragments, so they need their own buffer.
    uint8_t  ctrl_buf_[CTRL_PAYLOAD_CAP];
};

#endif // ENABLE_WEBSOCKETS && __has_include(<WiFi.h>)
#endif // WEBSOCKET_SERVER_H
