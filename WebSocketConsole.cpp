#if defined(ENABLE_WEBSOCKETS) && __has_include(<WiFi.h>)

#include "WebSocketConsole.h"
#include "WebSocketServer.h"

#include <cstdint>
#include <cstddef>

// KISS frame boundary. Defined inline here rather than including Framing.h
// because that header defines several globals at file scope (command,
// IN_FRAME, frame_len, ESCAPE) — including it from this second TU would
// cause multiple-definition link errors. The byte value is the KISS
// standard and isn't going to change.
constexpr uint8_t FEND = 0xC0;

// Provided by RNode_Firmware.ino — pushes one byte into serialFIFO when
// space is available, drops it on the floor otherwise. Mirrors the
// behavior of the other inbound paths in buffer_serial().
extern "C" void serial_fifo_push(uint8_t byte);

namespace {

// Max single KISS frame we'll emit over WS. Sized to comfortably cover
// the LoRa MTU plus KISS escaping overhead. If a single frame exceeds
// this, the buffer wraps via flush + restart on the next FEND — the
// outbound stream stays self-synchronizing because FEND is unambiguous.
constexpr size_t TX_BUF_CAP = 1024;

WebSocketServer* g_server = nullptr;
uint8_t          g_tx_buf[TX_BUF_CAP];
size_t           g_tx_len = 0;
bool             g_collecting = false;  // true between an opening FEND and
                                        // the next FEND that completes a
                                        // non-empty frame

void on_ws_message(const uint8_t* data, size_t len, bool /*is_text*/,
                   void* /*ctx*/) {
    // Hand each byte of the WS frame to the KISS parser. The browser is
    // expected to send one full KISS frame per binary WS frame, but the
    // serial_poll() KISS parser is byte-stream-driven — it doesn't care
    // about our WS frame boundaries.
    for (size_t i = 0; i < len; ++i) {
        serial_fifo_push(data[i]);
    }
}

void flush_outbound_frame() {
    if (g_tx_len == 0) return;
    if (g_server && g_server->connected()) {
        g_server->send_binary(g_tx_buf, g_tx_len);
    }
    g_tx_len = 0;
}

} // namespace

namespace ws_console {

void init(uint16_t port) {
    if (g_server) return;
    g_server = new WebSocketServer(port);
    g_server->on_message(&on_ws_message);
    g_server->begin();
}

void service() {
    if (g_server) g_server->service();
    // Drop any outbound buffer if the client went away mid-frame — the
    // next FEND from serial_write() will start a fresh frame.
    if (!client_attached() && g_tx_len > 0) {
        g_tx_len = 0;
        g_collecting = false;
    }
}

void on_serial_write(uint8_t byte) {
    if (!g_server) return;
    // We don't drop bytes when no client is attached; we just keep the
    // buffer empty by never appending. Cheap and avoids the cost of
    // formatting frames that go nowhere.
    if (!g_server->connected()) {
        g_tx_len     = 0;
        g_collecting = false;
        return;
    }

    if (byte == FEND) {
        if (g_collecting && g_tx_len > 1) {
            // Frame end: append the closing FEND and emit.
            if (g_tx_len < TX_BUF_CAP) g_tx_buf[g_tx_len++] = byte;
            flush_outbound_frame();
            g_collecting = false;
        } else {
            // Frame start (or a run of idle FENDs). Reset the buffer
            // with a single leading FEND and wait for the first data
            // byte.
            g_tx_buf[0]  = FEND;
            g_tx_len     = 1;
            g_collecting = true;
        }
        return;
    }

    if (!g_collecting) {
        // Mid-stream byte with no preceding FEND — shouldn't happen
        // under correct KISS, but be lenient: drop it.
        return;
    }
    if (g_tx_len < TX_BUF_CAP) {
        g_tx_buf[g_tx_len++] = byte;
    } else {
        // Frame overflows our buffer. Drop the in-flight frame; the
        // next FEND will start fresh. This is a guard, not a normal
        // path — LoRa frames don't approach 1 KiB.
        g_tx_len     = 0;
        g_collecting = false;
    }
}

bool client_attached() {
    return g_server && g_server->connected();
}

} // namespace ws_console

#endif // ENABLE_WEBSOCKETS && __has_include(<WiFi.h>)
