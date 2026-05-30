// Copyright (C) 2026, microReticulum_Firmware contributors

#include "TCPHostInterface.h"

#include <arpa/inet.h>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

namespace native_kiss_tcp {

namespace {

// KISS frame delimiter — used as the "no data" sentinel from read(), so
// a host that races us into a drained buffer sees a frame boundary rather
// than garbage. Matches Remote.h::wifi_remote_read().
constexpr uint8_t KISS_FEND = 0xC0;

// 15 s mirrors WR_READ_TIMEOUT_MS in Remote.h. If we see no inbound bytes
// for this long AND no client is generating any traffic, we tear down the
// socket so a fresh client can take over without waiting for the kernel's
// keepalive timer (which defaults to ~2 hours on Linux).
constexpr int IDLE_TIMEOUT_MS = 15000;

// Local 256-byte staging buffer between the kernel socket and the
// per-byte FIFO consumer in buffer_serial(). One recv() per fill cycle
// instead of one syscall per byte.
constexpr size_t RX_BUF_SIZE = 256;

int listen_fd = -1;
int client_fd = -1;
uint8_t rx_buf[RX_BUF_SIZE];
size_t rx_head = 0;
size_t rx_tail = 0;
uint32_t last_activity_ms = 0;

// monotonic_ms — Arduino's millis() is available since we link against
// Portduino, but we don't want to pull in Arduino.h here. clock_gettime
// with CLOCK_MONOTONIC is the POSIX equivalent and matches what we need
// for timeouts.
uint32_t now_ms() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint32_t>(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

void set_nonblocking(int fd) {
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags < 0) return;
    (void)fcntl(fd, F_SETFL, flags | O_NONBLOCK);
}

// Enable TCP keepalive on the accepted socket with aggressive timing so
// the kernel reaps half-open peers within ~10 s of them going silent,
// rather than the platform default (~2 hours on Linux, ~2 hours on
// macOS). The named knobs differ between Linux and macOS — on Linux
// the idle-before-first-probe is TCP_KEEPIDLE, on macOS it's TCP_KEEPALIVE.
void apply_keepalive(int fd) {
    int yes = 1;
    setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &yes, sizeof(yes));

    int idle_s = 5;
    int interval_s = 2;
    int count = 3;
#ifdef __APPLE__
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPALIVE, &idle_s, sizeof(idle_s));
#else
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &idle_s, sizeof(idle_s));
#endif
#ifdef TCP_KEEPINTVL
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &interval_s, sizeof(interval_s));
#endif
#ifdef TCP_KEEPCNT
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &count, sizeof(count));
#endif
}

void close_client() {
    if (client_fd >= 0) {
        ::close(client_fd);
        client_fd = -1;
    }
    rx_head = rx_tail = 0;
}

// Drain whatever the kernel has queued on the client socket into the
// staging buffer. Treats clean close (recv == 0) and hard errors as
// "client gone", so the next poll_accept() will pick up a new one.
void fill_rx_buffer() {
    if (client_fd < 0) return;

    // Reset the buffer if it's empty so we always read into the front
    // (avoids needing a ring).
    if (rx_head == rx_tail) {
        rx_head = rx_tail = 0;
    }

    if (rx_tail >= RX_BUF_SIZE) return; // full, consumer must drain first

    ssize_t n = ::recv(client_fd, rx_buf + rx_tail, RX_BUF_SIZE - rx_tail, 0);
    if (n > 0) {
        rx_tail += static_cast<size_t>(n);
        last_activity_ms = now_ms();
    } else if (n == 0) {
        // Peer closed cleanly.
        std::fprintf(stderr, "[kiss-tcp] client disconnected\n");
        close_client();
    } else if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
        std::fprintf(stderr, "[kiss-tcp] recv error: %s — dropping client\n",
                     std::strerror(errno));
        close_client();
    }
}

} // namespace

bool init(uint16_t port, bool bind_public) {
    listen_fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd < 0) {
        std::fprintf(stderr, "[kiss-tcp] socket(): %s\n", std::strerror(errno));
        return false;
    }

    int yes = 1;
    setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    // Loopback by default; bind_public flips to 0.0.0.0 so remote hosts
    // on the same network can connect. No auth/encryption either way —
    // public binding is an opt-in for trusted networks.
    const uint32_t bind_addr = bind_public ? INADDR_ANY : INADDR_LOOPBACK;
    const char* bind_label   = bind_public ? "0.0.0.0"  : "127.0.0.1";

    sockaddr_in addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(bind_addr);
    addr.sin_port = htons(port);

    if (::bind(listen_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::fprintf(stderr, "[kiss-tcp] bind(%s:%u): %s\n",
                     bind_label, port, std::strerror(errno));
        ::close(listen_fd);
        listen_fd = -1;
        return false;
    }

    if (::listen(listen_fd, 1) < 0) {
        std::fprintf(stderr, "[kiss-tcp] listen(): %s\n", std::strerror(errno));
        ::close(listen_fd);
        listen_fd = -1;
        return false;
    }

    set_nonblocking(listen_fd);
    std::fprintf(stderr, "[kiss-tcp] listening on %s:%u%s\n",
                 bind_label, port, bind_public ? " (PUBLIC — no auth)" : "");
    return true;
}

void poll_accept() {
    if (listen_fd < 0) return;

    // Drain pending accepts. Either we adopt one (no client yet) or we
    // immediately close any extras (one already attached).
    while (true) {
        sockaddr_in peer;
        socklen_t plen = sizeof(peer);
        int fd = ::accept(listen_fd, reinterpret_cast<sockaddr*>(&peer), &plen);
        if (fd < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
                std::fprintf(stderr, "[kiss-tcp] accept(): %s\n",
                             std::strerror(errno));
            }
            break;
        }

        if (client_fd >= 0) {
            // Already attached. Reject the second client immediately.
            std::fprintf(stderr,
                "[kiss-tcp] rejecting second client (one already attached)\n");
            ::close(fd);
            continue;
        }

        set_nonblocking(fd);
        apply_keepalive(fd);
        // Nagle off — KISS frames are small and latency-sensitive.
        int yes = 1;
        setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));

        client_fd = fd;
        rx_head = rx_tail = 0;
        last_activity_ms = now_ms();
        std::fprintf(stderr, "[kiss-tcp] client connected\n");
    }

    // Pull whatever the kernel has buffered for the active client.
    fill_rx_buffer();
}

void check_active() {
    if (client_fd < 0) return;
    if (now_ms() - last_activity_ms >= static_cast<uint32_t>(IDLE_TIMEOUT_MS)) {
        std::fprintf(stderr, "[kiss-tcp] client idle %d ms — closing\n",
                     IDLE_TIMEOUT_MS);
        close_client();
    }
}

bool is_connected() {
    return client_fd >= 0;
}

bool available() {
    return client_fd >= 0 && rx_head < rx_tail;
}

uint8_t read() {
    if (rx_head < rx_tail) {
        return rx_buf[rx_head++];
    }
    return KISS_FEND;
}

void write(uint8_t byte) {
    if (client_fd < 0) return;
    ssize_t n = ::send(client_fd, &byte, 1, 0);
    if (n == 1) {
        last_activity_ms = now_ms();
        return;
    }
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
        // The kernel's send buffer is full — for our low byte rate this
        // shouldn't happen with a healthy client. Treat it as the client
        // failing to drain and drop them so we don't hang on subsequent
        // writes.
        std::fprintf(stderr, "[kiss-tcp] send would block — dropping client\n");
    } else {
        std::fprintf(stderr, "[kiss-tcp] send error: %s — dropping client\n",
                     std::strerror(errno));
    }
    close_client();
}

void shutdown() {
    close_client();
    if (listen_fd >= 0) {
        ::close(listen_fd);
        listen_fd = -1;
    }
}

} // namespace native_kiss_tcp
