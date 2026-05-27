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

#include <Arduino.h>  // brings in `extern HardwareSPI SPI;` declaration

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <unistd.h>

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
    void seed_eeprom_if_unprovisioned();
}

// Portduino calls this before invoking the Arduino sketch's setup().
// The symbol replaces Portduino's weak default (which has C++ linkage,
// no extern "C") — match its signature exactly so the linker picks ours.
void portduinoSetup() {
    // 1) Load config from the default path (or a path supplied via env var).
    const char* cfg_env = std::getenv("MR_CONFIG");
    std::string cfg_path = cfg_env ? cfg_env : "microreticulum.conf";
    native_config::load(cfg_path);

    // 2) Optional --data-dir style override via env var.
    const char* data_env = std::getenv("MR_DATA_DIR");
    if (data_env && *data_env) {
        native_config::g_config.data_dir = data_env;
    }

    // 3) chdir to data dir so PosixFileSystem (microStore) and the EEPROM
    //    image are scoped there. Portduino's own VFS root (--fsroot) is a
    //    separate concern; we only care about cwd for our own persistence.
    if (chdir(native_config::g_config.data_dir.c_str()) != 0) {
        std::fprintf(stderr, "[portduinoSetup] chdir(%s): %s\n",
                     native_config::g_config.data_dir.c_str(),
                     std::strerror(errno));
        // Continue anyway — relative paths will resolve to wherever
        // Portduino started us.
    }

    // 4) Banner. macOS launches headless — no Reticulum interfaces
    //    registered (LORA_TRANSPORT is removed from build_flags for this
    //    env). Future UDPInterface support will land separately.
    #if !defined(LORA_TRANSPORT)
        std::fprintf(stderr, "[native] no LoRa interface (LORA_TRANSPORT undefined)\n");
    #endif

    // 5) Pin map. Still applied for consistency; the values are unused
    //    by anything on native-macos because no driver touches SPI/GPIO.
    native_pinmap::apply();

    // 6) Bind a SimSPIChip as a safety net. With LORA_TRANSPORT removed,
    //    the modem driver no longer initiates SPI activity, but Portduino's
    //    HardwareSPI::transfer() will assert(spiChip) if anything else
    //    (a stray indirect call, a thread, etc.) reaches the SPI subsystem.
    //    SPI.begin() creates a SimSPIChip on cross_platform / non-Linux.
    SPI.begin();

    // 7) EEPROM image (file-backed shim — see native/EEPROMShim.h).
    EEPROM.begin(EEPROM_SIZE);
    native_pinmap::seed_eeprom_if_unprovisioned();
}
