// Copyright (C) 2026, microReticulum_Firmware contributors
//
// Portduino owns `main()` (see cores/portduino/main.cpp in the framework).
// It defines `portduinoSetup()` as a weak symbol; overriding it here lets
// us run config loading + EEPROM init before the Arduino sketch's setup()
// is invoked.
//
// Sequence per Portduino's main:
//   portduinoCustomInit()  -> empty (weak)
//   argp_parse(...)         -> --fsroot etc.
//   mkdir($fsroot)          -> VFS root directory
//   portduinoVFS->mountpoint(fsroot)
//   gpioInit()
//   portduinoSetup()        <- this file
//   setup()                 <- Arduino sketch (RNode_Firmware.ino)
//   while (1) loop();

#include "config.h"
#include "EEPROMShim.h"
#include "Radio.h"
#include "TCPHostInterface.h"
#if defined(ENABLE_WEBSOCKETS) && __has_include(<WiFi.h>)
#include "../WebSocketConsole.h"
#endif

#include <Arduino.h>  // brings in `extern HardwareSPI SPI;` declaration

#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits.h>     // PATH_MAX
#include <string>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>

// Captured at portduinoSetup() entry, before we chdir() to data_dir.
// native/reboot.cpp reads this to restore the launch directory before
// execv() so the re-exec'd child can resolve `rnoded.conf` against the
// same cwd as the original invocation. Without this, the child runs with
// defaults (wrong pin map → radio dead).
std::string g_launch_cwd;

// strchrnul is a glibc extension that argp-standalone (on macOS, via
// Homebrew) expects to find in libc. Apple's libc doesn't provide it,
// so we define it here. POSIX-correct equivalent: return a pointer to
// the first occurrence of c, or to the trailing NUL if c isn't found.
#ifdef __APPLE__
extern "C" char* strchrnul(const char* s, int c) {
    while (*s && *s != static_cast<char>(c)) ++s;
    return const_cast<char*>(s);
}
#endif

// Boards.h provides EEPROM_SIZE.
#include "../Boards.h"

namespace native_pinmap {
    void apply();
    void bind_linux_gpios();   // no-op on macOS / cross_platform
    void seed_eeprom_if_unprovisioned();
}

// Defined in Utilities.h under the MODEM_RUNTIME branch. The native LoRa
// factory in RNode_Firmware.ino setup() reads this to instantiate the
// right concrete driver. Must be assigned before setup() runs.
extern uint8_t current_modem;

// Forward declarations matching Config.h. We can't include Config.h
// directly here because it contains file-scope variable definitions
// (e.g. `uint8_t op_mode = MODE_HOST;`) that would cause multiple-
// definition link errors when included from a second TU.
extern uint8_t op_mode;
#define FIRMWARE_MODE_TNC 0x12   // mirrors MODE_TNC in Config.h

// Async-signal-safe handler: just records the intent. The main loop polls
// native_radio::g_shutdown_requested and calls on_shutdown_signal() from a
// safe context, then exits. SIGKILL is uncatchable — boot-time
// clean_reset_at_boot() covers that case on the next launch.
static void shutdown_signal_handler(int /*sig*/) {
    native_radio::g_shutdown_requested = 1;
}

static void install_signal_handlers() {
    struct sigaction sa{};
    sa.sa_handler = shutdown_signal_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = SA_RESTART;
    sigaction(SIGINT,  &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);
    sigaction(SIGHUP,  &sa, nullptr);
}

// Portduino calls this before invoking the Arduino sketch's setup().
// The symbol replaces Portduino's weak default (which has C++ linkage,
// no extern "C") — match its signature exactly so the linker picks ours.
void portduinoSetup() {
    install_signal_handlers();

    // 0) Capture the launch directory before any chdir() so the deferred-
    //    reboot path (native/reboot.cpp) can restore it before execv(). The
    //    child resolves `rnoded.conf` (or $MR_CONFIG when relative) against
    //    cwd; without this, after the parent's step 3 chdir() to data_dir
    //    the child can't find the config and silently falls back to
    //    defaults.
    {
        char buf[PATH_MAX];
        if (::getcwd(buf, sizeof(buf))) {
            g_launch_cwd = buf;
        }
    }

    // 1) Load config from the default path (or a path supplied via env var).
    const char* cfg_env = std::getenv("MR_CONFIG");
    std::string cfg_path = cfg_env ? cfg_env : "rnoded.conf";
    native_config::load(cfg_path);

    // 2) Optional --data-dir style override via env var.
    const char* data_env = std::getenv("MR_DATA_DIR");
    if (data_env && *data_env) {
        native_config::g_config.data_dir = data_env;
    }

    // 3) chdir to data dir so PosixFileSystem (microStore) and the EEPROM
    //    image are scoped there. Portduino's own VFS root (--fsroot) is a
    //    separate concern; we only care about cwd for our own persistence.
    mkdir(native_config::g_config.data_dir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    if (chdir(native_config::g_config.data_dir.c_str()) != 0) {
        std::fprintf(stderr, "[portduinoSetup] chdir(%s): %s\n",
                     native_config::g_config.data_dir.c_str(),
                     std::strerror(errno));
        // Continue anyway — relative paths will resolve to wherever
        // Portduino started us.
    }

    // 4) Banner — make the mode obvious in the log.
    #if !defined(LORA_TRANSPORT)
        // macOS launches headless: no Reticulum interfaces registered
        // (LORA_TRANSPORT is unflagged from [env:native-macos]).
        // Future UDPInterface support will land separately.
        std::fprintf(stderr, "[native] no LoRa interface (LORA_TRANSPORT undefined)\n");
    #elif defined(PORTDUINO_LINUX_HARDWARE)
        // Linux native: real /dev/spidev + libgpiod backing.
        std::fprintf(stderr, "[native] Linux hardware mode (spi=%s gpio=%s)\n",
                     native_config::g_config.spi_dev.c_str(),
                     native_config::g_config.gpio_chip.c_str());
    #endif

    // 5) Pin map. apply() copies g_config.pin_* into the extern int pin_*
    //    globals consulted by the modem driver. bind_linux_gpios() then
    //    registers libgpiod-backed pins with Portduino (no-op on macOS).
    native_pinmap::apply();
    native_pinmap::bind_linux_gpios();

    // 5.5) Extended NRESET pulse before any SPI traffic. Recovers a chip
    //      that was wedged by a prior daemon crash (SIGKILL mid-opcode,
    //      BUSY stuck high, etc.) before the modem driver's own 10 ms
    //      reset() runs inside LoRa->begin(). No-op when pin_reset == -1.
    native_radio::clean_reset_at_boot();

    // 5a) Surface the configured modem family to setup() so the native LoRa
    //     factory can instantiate the right driver. Must precede setup().
    current_modem = native_config::g_config.modem;
    const char* modem_name;
    switch (current_modem) {
        case 0x01: modem_name = "SX1276"; break;
        case 0x02: modem_name = "SX1278"; break;
        case 0x03: modem_name = "SX1262"; break;
        case 0x04: modem_name = "SX1280"; break;
        default:   modem_name = "UNKNOWN"; break;
    }
    std::fprintf(stderr, "[native] modem = %s (0x%02X)\n", modem_name, current_modem);

    // 5b) KISS-over-TCP host transport. The embedded firmware's USB-serial
    //     KISS channel is replaced on native by a localhost TCP server.
    //     A failure to bind isn't fatal — the daemon keeps running like
    //     an embedded RNode with no USB cable plugged in. kiss_tcp_public
    //     opts into binding on 0.0.0.0; defaults to loopback.
    native_kiss_tcp::init(native_config::g_config.kiss_tcp_port,
                          native_config::g_config.kiss_tcp_public);

    #if defined(ENABLE_WEBSOCKETS) && __has_include(<WiFi.h>)
    // KISS-over-WebSocket on port 8080. Loopback-only by default;
    // kiss_ws_public=true in rnoded.conf opts into binding 0.0.0.0 the
    // same way kiss_tcp_public does for the raw KISS TCP server.
    ws_console::init(8080, native_config::g_config.kiss_ws_public);
    #endif

    // 6) Bind a SimSPIChip as a safety net. With LORA_TRANSPORT removed,
    //    the modem driver no longer initiates SPI activity, but Portduino's
    //    HardwareSPI::transfer() will assert(spiChip) if anything else
    //    (a stray indirect call, a thread, etc.) reaches the SPI subsystem.
    //    SPI.begin() creates a SimSPIChip on cross_platform / non-Linux.
    SPI.begin();

    // 7) EEPROM image (file-backed shim — see native/EEPROMShim.h).
    EEPROM.begin(EEPROM_SIZE);
    native_pinmap::seed_eeprom_if_unprovisioned();

    // 8) Force TNC mode on the native daemon. The embedded firmware flips
    //    op_mode to MODE_TNC only when `hw_ready && eeprom_have_conf()`,
    //    which requires real radio hardware. On the native daemon — where
    //    we may run without a connected modem (or where the modem driver
    //    is fully bypassed on macOS via LORA_TRANSPORT) — we mark the
    //    daemon as TNC explicitly so the Reticulum Transport runs in
    //    enabled mode rather than the host-protocol fallback.
    op_mode = FIRMWARE_MODE_TNC;
}
