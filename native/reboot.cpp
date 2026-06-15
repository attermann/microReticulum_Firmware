// Copyright (C) 2026, microReticulum_Firmware contributors
//
// See reboot.h for the deferred-reboot rationale. Cleanup sequence mirrors
// meshtasticd's Power::reboot() (src/Power.cpp:711-737 in their tree):
// notify, close API server, release peripheral handles, SPI.end(), reboot().
// Our daemon doesn't have a notification observer pattern yet, so we skip
// that step.

#include "reboot.h"

#include "EEPROMShim.h"
#include "TCPHostInterface.h"

#include <Arduino.h>   // void reboot(); extern HardwareSPI SPI;

#include <cstdio>
#include <cstdlib>

namespace native_pinmap {
    // Defined in PinMap.cpp. No-op on macOS, releases libgpiod handles
    // on Linux by rebinding each previously-bound pin to a SimGPIOPin
    // (which triggers ~LinuxGPIOPin → gpiod_line_release).
    void release_linux_gpios();
}

namespace native_reboot {

namespace {
bool reboot_flag = false;
}

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

    // Close the KISS-over-TCP listener + active client so the re-exec'd
    // process can re-bind the same port without waiting for TIME_WAIT.
    native_kiss_tcp::shutdown();

    // Release libgpiod chip/line handles. Without this, the re-exec'd
    // process fails to re-acquire the same lines (EBUSY).
    native_pinmap::release_linux_gpios();

    // Let Portduino tear down the SimSPIChip (cross_platform) or the
    // /dev/spidev binding (Linux).
    SPI.end();

    std::fprintf(stderr, "[reboot] re-exec...\n");
    ::reboot();  // Portduino's reboot() — execv(argv[0], argv).

    // Portduino's reboot() calls exit(EXIT_FAILURE) on execv failure, so
    // this should be unreachable. [[noreturn]] still requires we don't
    // fall through, so terminate explicitly.
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
