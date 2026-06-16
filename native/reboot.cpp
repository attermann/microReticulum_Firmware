// Copyright (C) 2026, microReticulum_Firmware contributors
//
// See reboot.h for the deferred-reboot rationale. Cleanup sequence mirrors
// meshtasticd's Power::reboot() (src/Power.cpp:711-737 in their tree):
// notify, close API server, release peripheral handles, SPI.end(), reboot().
// Our daemon doesn't have a notification observer pattern yet, so we skip
// that step.
//
// Re-exec note: we deliberately bypass Portduino's `::reboot()`. That
// function does `execv(progArgv[0], progArgv)` with the raw argv[0] the
// kernel handed main() — when the daemon is launched via PATH (e.g.
// systemd's `rnoded` or a bare command-name invocation) argv[0] is the
// bare name, and execv() does NOT consult PATH, so it fails with ENOENT
// and Portduino then exit()s. We resolve our own absolute path from the
// kernel (/proc/self/exe on Linux, _NSGetExecutablePath on macOS) and
// reconstruct argv from the kernel too (/proc/self/cmdline on Linux,
// KERN_PROCARGS2 on macOS) so flags like --fsroot survive the re-exec.

#include "reboot.h"

#include "EEPROMShim.h"
#include "TCPHostInterface.h"
#if defined(ENABLE_WEBSOCKETS) && __has_include(<WiFi.h>)
#include "../WebSocketConsole.h"
#endif

#include <Arduino.h>   // extern HardwareSPI SPI;

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <limits.h>     // PATH_MAX
#include <string>
#include <unistd.h>
#include <vector>

#if defined(__APPLE__)
#include <mach-o/dyld.h>
#include <sys/sysctl.h>
#endif

namespace native_pinmap {
    // Defined in PinMap.cpp. No-op on macOS, releases libgpiod handles
    // on Linux by rebinding each previously-bound pin to a SimGPIOPin
    // (which triggers ~LinuxGPIOPin → gpiod_line_release).
    void release_linux_gpios();
}

// Captured in native/main.cpp::portduinoSetup() before the chdir(data_dir).
// We need to restore this before execv() so the child's load() can resolve
// `rnoded.conf` against the same cwd as the original invocation.
extern std::string g_launch_cwd;

namespace native_reboot {

namespace {

bool reboot_flag = false;

#if defined(__linux__)

std::string resolve_exe_path() {
    char buf[PATH_MAX];
    ssize_t n = ::readlink("/proc/self/exe", buf, sizeof(buf) - 1);
    if (n <= 0) return {};
    buf[n] = '\0';
    return std::string(buf);
}

std::vector<std::string> resolve_argv() {
    std::vector<std::string> out;
    std::FILE* f = std::fopen("/proc/self/cmdline", "rb");
    if (!f) return out;
    std::string cur;
    int c;
    while ((c = std::fgetc(f)) != EOF) {
        if (c == 0) {
            out.push_back(std::move(cur));
            cur.clear();
        } else {
            cur.push_back(static_cast<char>(c));
        }
    }
    std::fclose(f);
    if (!cur.empty()) out.push_back(std::move(cur));
    return out;
}

#elif defined(__APPLE__)

std::string resolve_exe_path() {
    char buf[PATH_MAX];
    uint32_t size = sizeof(buf);
    if (_NSGetExecutablePath(buf, &size) != 0) return {};
    // _NSGetExecutablePath returns a path that may include `..` or symlinks;
    // canonicalize so the child sees the same exe regardless of how the
    // parent was launched.
    char real[PATH_MAX];
    if (::realpath(buf, real)) return std::string(real);
    return std::string(buf);
}

std::vector<std::string> resolve_argv() {
    // KERN_PROCARGS2 layout (Darwin):
    //   int   argc
    //   char  exe_path[]    (NUL-terminated, may be NUL-padded for alignment)
    //   char  argv[0][]     (NUL-terminated)
    //   ...
    //   char  argv[argc-1][]
    //   char  envp[0][] ... (we stop after argc strings)
    std::vector<std::string> out;
    int mib[3] = { CTL_KERN, KERN_PROCARGS2, ::getpid() };
    size_t size = 0;
    if (::sysctl(mib, 3, nullptr, &size, nullptr, 0) != 0 || size == 0) {
        return out;
    }
    std::vector<char> buf(size);
    if (::sysctl(mib, 3, buf.data(), &size, nullptr, 0) != 0) {
        return out;
    }
    if (size < sizeof(int)) return out;
    int argc = 0;
    std::memcpy(&argc, buf.data(), sizeof(argc));
    if (argc <= 0) return out;

    const char* p   = buf.data() + sizeof(int);
    const char* end = buf.data() + size;
    while (p < end && *p != '\0') ++p;        // skip exe string
    while (p < end && *p == '\0') ++p;        // skip alignment NULs
    for (int i = 0; i < argc && p < end; ++i) {
        const char* s = p;
        while (p < end && *p != '\0') ++p;
        out.emplace_back(s, static_cast<size_t>(p - s));
        if (p < end) ++p;                     // step over NUL
    }
    return out;
}

#else

// Unknown POSIX host — try /proc as a last resort, fall back to empty.
std::string resolve_exe_path() {
    char buf[PATH_MAX];
    ssize_t n = ::readlink("/proc/self/exe", buf, sizeof(buf) - 1);
    if (n <= 0) return {};
    buf[n] = '\0';
    return std::string(buf);
}

std::vector<std::string> resolve_argv() {
    return {};
}

#endif

// Close inherited spidev fds before execv. Surgical replacement for a
// blanket "close every fd above stderr" sweep: blanket closing turned out
// to break libgpiod re-binding in the child (likely because we closed
// something Portduino still needed during shutdown / inside its own
// destructors). Here we walk /proc/self/fd (Linux) / /dev/fd (macOS) and
// only close fds whose target path is /dev/spidev*.
//
// Why this matters: Portduino's LinuxHardwareSPI::end() calls spiChip.reset()
// but a static SPI_map<string, shared_ptr> still holds the LinuxSPIChip ref,
// so its PosixFile base never destructs. PosixFile opens without O_CLOEXEC,
// so the spidev fd is inherited by the re-exec'd child, which then races
// the kernel spidev driver with its own fresh open — the SX126x reads back
// 0x00 across all SPI ioctls. Explicitly closing the spidev fd before
// execv breaks the race.
//
// Other potentially-leaked fds (libgpiod chips not closed by ~LinuxGPIOPin,
// AsyncUDP sockets, etc.) we leave alone here. libgpiod chip fds in
// particular are closed by release_linux_gpios() destroying every bound
// LinuxGPIOPin, which is called right before this. If a different fd
// turns out to also need cleanup later, add it to the prefix match.
void close_inherited_spidev_fds() {
#if defined(__linux__)
    const char* fd_dir = "/proc/self/fd";
#elif defined(__APPLE__)
    const char* fd_dir = "/dev/fd";
#else
    return;
#endif
#if defined(__linux__) || defined(__APPLE__)
    DIR* d = ::opendir(fd_dir);
    if (!d) return;
    const int dfd = ::dirfd(d);
    struct dirent* e;
    while ((e = ::readdir(d)) != nullptr) {
        if (e->d_name[0] < '0' || e->d_name[0] > '9') continue;
        const int fd = std::atoi(e->d_name);
        if (fd <= 2 || fd == dfd) continue;
        char fdpath[64];
        std::snprintf(fdpath, sizeof(fdpath), "%s/%s", fd_dir, e->d_name);
        char target[PATH_MAX];
        const ssize_t n = ::readlink(fdpath, target, sizeof(target) - 1);
        if (n <= 0) continue;
        target[n] = '\0';
        if (std::strncmp(target, "/dev/spidev", 11) == 0) {
            std::fprintf(stderr, "[reboot] closing inherited %s (fd=%d)\n",
                         target, fd);
            ::close(fd);
        }
    }
    ::closedir(d);
#endif
}

} // namespace

void request() {
    if (!reboot_flag) {
        std::fprintf(stderr,
            "[reboot] requested — will re-exec on next loop tick\n");
    }
    reboot_flag = true;
}

bool pending() {
    return reboot_flag;
}

[[noreturn]] void perform() {
    std::fprintf(stderr, "[reboot] tearing down and re-execing daemon\n");

    // Defensive flush. Most config-save paths in Utilities.h call
    // EEPROM.commit() before hard_reset(), but a missed call site would
    // otherwise lose the property write on re-exec.
    EEPROM.commit();

    // Close the WebSocket listener so the re-exec'd process can re-bind
    // port 8080. FD_CLOEXEC on the listen fd would handle it too — this
    // is the explicit-teardown half of belt and suspenders.
    #if defined(ENABLE_WEBSOCKETS) && __has_include(<WiFi.h>)
    ws_console::shutdown();
    #endif

    // Close the KISS-over-TCP listener + active client so the re-exec'd
    // process can re-bind the same port without waiting for TIME_WAIT.
    native_kiss_tcp::shutdown();

    // Release libgpiod chip/line handles. Without this, the re-exec'd
    // process fails to re-acquire the same lines (EBUSY).
    native_pinmap::release_linux_gpios();

    // Let Portduino tear down the SimSPIChip (cross_platform) or the
    // /dev/spidev binding (Linux). On Linux this is not actually enough —
    // LinuxHardwareSPI holds the spidev fd in a static SPI_map shared_ptr
    // that end() doesn't clear, and PosixFile opens without O_CLOEXEC — the
    // close_inherited_spidev_fds() sweep below is what actually closes the fd.
    SPI.end();

    // Close any inherited /dev/spidev* fd so the child doesn't race the
    // kernel spidev driver on re-open. See close_inherited_spidev_fds()
    // for the full rationale.
    close_inherited_spidev_fds();

    // Restore the original launch cwd. portduinoSetup() chdir'd to data_dir
    // after loading config; without this restore the child's load() looks
    // for rnoded.conf in data_dir, doesn't find it, and silently falls back
    // to defaults — wrong pin map, radio doesn't respond.
    if (!g_launch_cwd.empty()) {
        if (::chdir(g_launch_cwd.c_str()) != 0) {
            std::fprintf(stderr, "[reboot] chdir(%s) failed: %s — child may "
                         "not find rnoded.conf\n",
                         g_launch_cwd.c_str(), std::strerror(errno));
        }
    }

    // Resolve our own absolute path from the kernel — see file-top comment.
    std::string exe = resolve_exe_path();
    if (exe.empty()) {
        std::fprintf(stderr,
            "[reboot] could not resolve own exe path — abort\n");
        std::abort();
    }

    // Reconstruct argv so --fsroot etc. survive the re-exec. If the kernel
    // gives us nothing (rare), fall back to a single-arg exec — lossy but
    // better than dying.
    std::vector<std::string> argv_strs = resolve_argv();
    if (argv_strs.empty()) {
        argv_strs.emplace_back(exe);
    } else {
        // Ensure argv[0] agrees with the resolved path. Conventional and
        // makes ps output less confusing across the re-exec.
        argv_strs[0] = exe;
    }

    // std::string::data() doesn't return non-const char* until C++17; use
    // &s[0] for C++14 compatibility (non-const since C++11, including for
    // empty strings where it points at the NUL terminator).
    std::vector<char*> argv;
    argv.reserve(argv_strs.size() + 1);
    for (auto& s : argv_strs) argv.push_back(&s[0]);
    argv.push_back(nullptr);

    std::fprintf(stderr, "[reboot] re-exec %s (argc=%zu)\n",
                 exe.c_str(), argv_strs.size());
    ::execv(exe.c_str(), argv.data());

    std::fprintf(stderr, "[reboot] execv(%s) failed: %s — abort\n",
                 exe.c_str(), std::strerror(errno));
    std::abort();
}

} // namespace native_reboot

void native_request_reboot() {
    native_reboot::request();
}

bool native_reboot_pending() {
    return native_reboot::pending();
}

[[noreturn]] void native_reboot_perform() {
    native_reboot::perform();
}
