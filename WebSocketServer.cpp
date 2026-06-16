#if defined(ENABLE_WEBSOCKETS) && __has_include(<WiFi.h>)

#include "WebSocketServer.h"
#include "native/ws_shims/base64.h"
#include "native/ws_shims/sha1.h"

#include <Arduino.h>
#include <cstdio>
#include <cstring>

#if defined(PORTDUINO)
  #include <arpa/inet.h>
  #include <errno.h>
  #include <fcntl.h>
  #include <netinet/in.h>
  #include <signal.h>
  #include <sys/socket.h>
  #include <unistd.h>
#endif

namespace {

constexpr char kWsGuid[] = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";

// ASCII case-insensitive prefix compare — used for header name lookup.
bool ieq_prefix(const uint8_t* hay, size_t hay_len,
                const char* needle) {
    size_t i = 0;
    while (needle[i]) {
        if (i >= hay_len) return false;
        uint8_t a = hay[i];
        uint8_t b = static_cast<uint8_t>(needle[i]);
        if (a >= 'A' && a <= 'Z') a |= 0x20;
        if (b >= 'A' && b <= 'Z') b |= 0x20;
        if (a != b) return false;
        ++i;
    }
    return true;
}

// Find header line `name:` (case-insensitive) in `buf[0..len)`. Returns a
// pointer to the start of the value (after the colon, leading SP trimmed)
// and writes the value length into *out_len. Returns nullptr if absent.
const uint8_t* find_header(const uint8_t* buf, size_t len,
                           const char* name,
                           size_t* out_len) {
    const size_t name_len = std::strlen(name);
    size_t line_start = 0;
    for (size_t i = 0; i + 1 < len; ++i) {
        if (buf[i] == '\r' && buf[i + 1] == '\n') {
            const size_t line_len = i - line_start;
            if (line_len > name_len + 1
                && ieq_prefix(buf + line_start, line_len, name)
                && buf[line_start + name_len] == ':') {
                size_t v0 = line_start + name_len + 1;
                while (v0 < i && (buf[v0] == ' ' || buf[v0] == '\t')) ++v0;
                size_t v1 = i;
                while (v1 > v0 && (buf[v1-1] == ' ' || buf[v1-1] == '\t')) --v1;
                *out_len = v1 - v0;
                return buf + v0;
            }
            line_start = i + 2;
            ++i;  // skip \n
        }
    }
    return nullptr;
}

} // namespace

WebSocketServer::~WebSocketServer() {
#if defined(PORTDUINO)
    if (listen_fd_ >= 0) {
        ::close(listen_fd_);
        listen_fd_ = -1;
    }
#endif
}

void WebSocketServer::begin() {
#if defined(PORTDUINO)
    // Native (Portduino): we bypass WiFiServer entirely because Portduino's
    // WiFiServer::begin() hardcodes INADDR_ANY, leaving no way to bind on
    // loopback only. Open and bind our own listening socket so we honor
    // bind_public_.
    signal(SIGPIPE, SIG_IGN);  // a disconnecting WS peer shouldn't kill us

    listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd_ < 0) {
        std::fprintf(stderr, "[ws] socket(): %s\n", std::strerror(errno));
        return;
    }
    int yes = 1;
    setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    const uint32_t bind_addr = bind_public_ ? INADDR_ANY : INADDR_LOOPBACK;
    const char* bind_label   = bind_public_ ? "0.0.0.0"  : "127.0.0.1";

    sockaddr_in addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(bind_addr);
    addr.sin_port        = htons(port_);
    if (::bind(listen_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::fprintf(stderr, "[ws] bind(%s:%u): %s\n",
                     bind_label, port_, std::strerror(errno));
        ::close(listen_fd_);
        listen_fd_ = -1;
        return;
    }
    if (::listen(listen_fd_, 1) < 0) {
        std::fprintf(stderr, "[ws] listen(): %s\n", std::strerror(errno));
        ::close(listen_fd_);
        listen_fd_ = -1;
        return;
    }
    int flags = fcntl(listen_fd_, F_GETFL, 0);
    fcntl(listen_fd_, F_SETFL, flags | O_NONBLOCK);
    std::fprintf(stderr, "[ws] listening on %s:%u%s\n",
                 bind_label, port_,
                 bind_public_ ? " (PUBLIC — no auth)" : "");
#else
    // ESP32: stock Arduino WiFiServer. Always binds 0.0.0.0 (intended —
    // the console is served over the device's WiFi AP). bind_public_ is
    // ignored.
    server_.begin();
#endif
}

void WebSocketServer::service() {
    // Tear down a half-dead client first.
    if (state_ != State::WAITING && !client_.connected()) {
        teardown();
    }
    switch (state_) {
        case State::WAITING:     drive_accept();    break;
        case State::HANDSHAKING: drive_handshake(); break;
        case State::OPEN:        drive_frame();     break;
        case State::CLOSING:     teardown();        break;
    }
}

void WebSocketServer::drive_accept() {
#if defined(PORTDUINO)
    if (listen_fd_ < 0) return;
    sockaddr_in cli;
    socklen_t   clen = sizeof(cli);
    int s = ::accept(listen_fd_, reinterpret_cast<sockaddr*>(&cli), &clen);
    if (s < 0) {
        // EAGAIN/EWOULDBLOCK on a non-blocking listener = no pending
        // client right now; ignore. Other errno values are unexpected
        // but non-fatal — we'll try again next tick.
        return;
    }
    int flags = fcntl(s, F_GETFL, 0);
    fcntl(s, F_SETFL, flags | O_NONBLOCK);
    client_  = WiFiClient(s);
    state_   = State::HANDSHAKING;
    req_len_ = 0;
#else
    WiFiClient pending = server_.available();
    if (!pending.connected()) return;
    client_  = pending;
    state_   = State::HANDSHAKING;
    req_len_ = 0;
#endif
}

void WebSocketServer::drive_handshake() {
    while (true) {
        int b = client_.read();
        if (b < 0) return;  // no more data right now
        if (req_len_ >= REQ_BUF_CAP) {
            const char* resp = "HTTP/1.1 400 Bad Request\r\n\r\n";
            client_.write(reinterpret_cast<const uint8_t*>(resp),
                          std::strlen(resp));
            teardown();
            return;
        }
        req_buf_[req_len_++] = static_cast<uint8_t>(b);

        if (req_len_ >= 4
            && req_buf_[req_len_ - 4] == '\r'
            && req_buf_[req_len_ - 3] == '\n'
            && req_buf_[req_len_ - 2] == '\r'
            && req_buf_[req_len_ - 1] == '\n') {
            if (!finish_handshake()) teardown();
            return;
        }
    }
}

bool WebSocketServer::finish_handshake() {
    size_t key_len = 0;
    const uint8_t* key = find_header(req_buf_, req_len_,
                                     "Sec-WebSocket-Key", &key_len);
    if (!key || key_len == 0 || key_len > 64) {
        const char* resp =
            "HTTP/1.1 400 Bad Request\r\n"
            "Content-Length: 0\r\n\r\n";
        client_.write(reinterpret_cast<const uint8_t*>(resp),
                      std::strlen(resp));
        return false;
    }

    // accept = base64( SHA1( key || GUID ) )
    uint8_t concat[64 + 36];
    std::memcpy(concat, key, key_len);
    std::memcpy(concat + key_len, kWsGuid, 36);

    uint8_t digest[20];
    ws_proto::sha1(concat, key_len + 36, digest);

    char accept[32];
    size_t accept_len = ws_proto::base64_encode(digest, 20, accept);

    char resp[256];
    int n = std::snprintf(resp, sizeof(resp),
        "HTTP/1.1 101 Switching Protocols\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        "Sec-WebSocket-Accept: %.*s\r\n"
        "\r\n",
        static_cast<int>(accept_len), accept);
    if (n <= 0 || n >= static_cast<int>(sizeof(resp))) return false;
    client_.write(reinterpret_cast<const uint8_t*>(resp),
                  static_cast<size_t>(n));

    state_       = State::OPEN;
    msg_opcode_  = 0;
    msg_len_     = 0;
    reset_frame_rx();
    return true;
}

void WebSocketServer::reset_frame_rx() {
    rx_phase_         = RxPhase::HEAD_0;
    rx_opcode_        = 0;
    rx_fin_           = false;
    rx_is_ctrl_       = false;
    rx_masked_        = false;
    rx_ext_remaining_ = 0;
    rx_mask_pos_      = 0;
    rx_payload_pos_   = 0;
    rx_payload_total_ = 0;
}

void WebSocketServer::drive_frame() {
    while (true) {
        int b = client_.read();
        if (b < 0) return;
        const uint8_t byte = static_cast<uint8_t>(b);

        switch (rx_phase_) {
            case RxPhase::HEAD_0: {
                rx_fin_      = (byte & 0x80) != 0;
                rx_opcode_   =  byte & 0x0f;
                rx_is_ctrl_  = (rx_opcode_ & 0x8) != 0;

                if (rx_is_ctrl_) {
                    // Control frames must NOT be fragmented.
                    if (!rx_fin_) { send_close(CLOSE_PROTO); return; }
                } else {
                    // Data frame: TEXT / BIN / CONTINUATION
                    if (rx_opcode_ == 0x0) {
                        // Continuation — must have a message in progress.
                        if (msg_opcode_ == 0) {
                            send_close(CLOSE_PROTO); return;
                        }
                    } else if (rx_opcode_ == 0x1 || rx_opcode_ == 0x2) {
                        // New message — previous one must be finished.
                        if (msg_opcode_ != 0) {
                            send_close(CLOSE_PROTO); return;
                        }
                        msg_opcode_ = rx_opcode_;
                        msg_len_    = 0;
                    } else {
                        // Reserved data opcode.
                        send_close(CLOSE_PROTO); return;
                    }
                }
                rx_phase_ = RxPhase::HEAD_1;
                break;
            }

            case RxPhase::HEAD_1: {
                rx_masked_     = (byte & 0x80) != 0;
                const uint8_t l7 = byte & 0x7f;
                // RFC 6455 §5.3: client→server frames MUST be masked.
                if (!rx_masked_) { send_close(CLOSE_PROTO); return; }
                // Control frames cap at 125-byte payloads.
                if (rx_is_ctrl_ && l7 >= 126) {
                    send_close(CLOSE_PROTO); return;
                }
                if (l7 < 126) {
                    rx_payload_total_ = l7;
                    rx_ext_remaining_ = 0;
                    rx_phase_ = RxPhase::MASK;
                } else if (l7 == 126) {
                    rx_payload_total_ = 0;
                    rx_ext_remaining_ = 2;
                    rx_phase_ = RxPhase::EXT_LEN;
                } else {
                    // 64-bit length — not accepted in this server.
                    send_close(CLOSE_TOOBIG); return;
                }
                break;
            }

            case RxPhase::EXT_LEN:
                rx_payload_total_ = (rx_payload_total_ << 8) | byte;
                if (--rx_ext_remaining_ == 0) {
                    // Ensure the *accumulated* message wouldn't overflow.
                    if (!rx_is_ctrl_
                        && msg_len_ + rx_payload_total_ > PAYLOAD_CAP) {
                        send_close(CLOSE_TOOBIG); return;
                    }
                    rx_phase_ = RxPhase::MASK;
                }
                break;

            case RxPhase::MASK:
                rx_mask_[rx_mask_pos_++] = byte;
                if (rx_mask_pos_ == 4) {
                    rx_mask_pos_ = 0;
                    if (rx_payload_total_ == 0) {
                        dispatch_frame();
                        if (state_ != State::OPEN) return;
                        reset_frame_rx();
                    } else {
                        rx_phase_ = RxPhase::PAYLOAD;
                    }
                }
                break;

            case RxPhase::PAYLOAD: {
                const uint8_t unmasked = byte ^ rx_mask_[rx_payload_pos_ & 3];
                if (rx_is_ctrl_) {
                    if (rx_payload_pos_ < CTRL_PAYLOAD_CAP) {
                        ctrl_buf_[rx_payload_pos_] = unmasked;
                    }
                } else if (msg_len_ < PAYLOAD_CAP) {
                    payload_[msg_len_++] = unmasked;
                }
                ++rx_payload_pos_;
                if (rx_payload_pos_ >= rx_payload_total_) {
                    dispatch_frame();
                    if (state_ != State::OPEN) return;
                    reset_frame_rx();
                }
                break;
            }
        }
    }
}

void WebSocketServer::dispatch_frame() {
    if (rx_is_ctrl_) {
        switch (rx_opcode_) {
            case 0x8:  // CLOSE
                send_close(CLOSE_NORMAL);
                return;
            case 0x9:  // PING — echo as PONG
                send_frame(0xA, ctrl_buf_, rx_payload_total_);
                break;
            case 0xA:  // PONG — ignore
                break;
            default:
                send_close(CLOSE_PROTO);
                return;
        }
    } else if (rx_fin_) {
        // Final fragment of a data message — deliver.
        if (on_message_) {
            on_message_(payload_, msg_len_, msg_opcode_ == 0x1,
                        on_message_ctx_);
        }
        msg_opcode_ = 0;
        msg_len_    = 0;
    }
    // Non-final data fragment: payload is already accumulated, wait for
    // more.
}

bool WebSocketServer::send_frame(uint8_t opcode,
                                 const uint8_t* data, size_t len) {
    if (!client_.connected()) return false;
    uint8_t hdr[4];
    size_t  hdr_len = 2;
    hdr[0] = 0x80 | (opcode & 0x0f);  // FIN=1, no masking server-side
    if (len < 126) {
        hdr[1] = static_cast<uint8_t>(len);
    } else if (len <= 0xFFFF) {
        hdr[1] = 126;
        hdr[2] = static_cast<uint8_t>(len >> 8);
        hdr[3] = static_cast<uint8_t>(len);
        hdr_len = 4;
    } else {
        return false;  // not supported in this server
    }
    if (client_.write(hdr, hdr_len) != hdr_len) return false;
    if (len > 0 && client_.write(data, len) != len) return false;
    return true;
}

bool WebSocketServer::send_text(const char* text) {
    return send_frame(0x1, reinterpret_cast<const uint8_t*>(text),
                      std::strlen(text));
}

bool WebSocketServer::send_binary(const uint8_t* data, size_t len) {
    return send_frame(0x2, data, len);
}

void WebSocketServer::send_close(uint16_t code) {
    uint8_t body[2] = { uint8_t(code >> 8), uint8_t(code) };
    send_frame(0x8, body, 2);
    state_ = State::CLOSING;
}

void WebSocketServer::teardown() {
    if (client_) client_.stop();
    state_      = State::WAITING;
    req_len_    = 0;
    msg_opcode_ = 0;
    msg_len_    = 0;
    reset_frame_rx();
}

#endif // ENABLE_WEBSOCKETS && __has_include(<WiFi.h>)
