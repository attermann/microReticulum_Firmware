// Copyright (C) 2026, microReticulum_Firmware contributors
//
// Native Linux entrypoint. Replaces the implicit Arduino setup()/loop()
// scheduler with a process-style main(). The firmware's existing setup()
// and loop() from RNode_Firmware.ino are reused unchanged once Portduino
// has been initialized and the pin map populated from the config file.

#include "config.h"
#include "EEPROMShim.h"

#include <atomic>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <unistd.h>

// Boards.h provides EEPROM_SIZE; pull it in via the existing chain.
#include "../Boards.h"

// The Arduino sketch entrypoints (RNode_Firmware.ino is compiled as .cpp on
// native — see CMakeLists.txt — so these symbols are link-time visible).
void setup();
void loop();

// Portduino's framework initializer. The exact signature depends on the
// vendored Portduino release; the call below matches Meshtastic's
// framework-portduino as of mid-2025. Verify against the actual headers
// when building.
//   - HAL needs to know which /dev/spidev to back SPI.{begin,transfer,...}
//   - libgpiod chip path for digitalRead/Write/pinMode + attachInterrupt
// If the API has drifted, expect to adapt this call site only.
extern "C" {
    int  portduinoSetup(int argc, char** argv);    // returns 0 on success
    void portduinoConfigureSPI(const char* dev, uint32_t speed_hz);
    void portduinoConfigureGPIO(const char* chip_path);
}

namespace native_pinmap {
    void apply();
    void seed_eeprom_if_unprovisioned();
}

namespace {

std::atomic<bool> g_running{true};

void on_signal(int /*sig*/) {
    g_running.store(false, std::memory_order_relaxed);
}

struct Cli {
    std::string config_path = "microreticulum.conf";
    std::string data_dir;  // empty -> use config's data_dir
};

Cli parse_cli(int argc, char** argv) {
    Cli cli;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&]() -> const char* {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "missing value for %s\n", a.c_str());
                std::exit(2);
            }
            return argv[++i];
        };
        if      (a == "--config")    cli.config_path = next();
        else if (a == "--data-dir")  cli.data_dir = next();
        else if (a == "-h" || a == "--help") {
            std::printf(
                "Usage: microreticulum [options]\n"
                "  --config PATH     key=value config file (default: microreticulum.conf)\n"
                "  --data-dir PATH   override data_dir from config\n");
            std::exit(0);
        } else {
            std::fprintf(stderr, "unknown arg: %s\n", a.c_str());
            std::exit(2);
        }
    }
    return cli;
}

} // namespace

int main(int argc, char** argv) {
    Cli cli = parse_cli(argc, argv);

    // 1) Load config.
    native_config::load(cli.config_path);
    if (!cli.data_dir.empty()) native_config::g_config.data_dir = cli.data_dir;

    // 2) chdir into the data dir so PosixFileSystem (microStore) and the
    //    EEPROM image are scoped there.
    if (chdir(native_config::g_config.data_dir.c_str()) != 0) {
        std::fprintf(stderr, "[main] cannot chdir to %s: %s\n",
                     native_config::g_config.data_dir.c_str(), std::strerror(errno));
        return 1;
    }

    // 3) Initialize Portduino with the chosen SPI device and GPIO chip.
    //    Order matters: SPI/GPIO config must precede setPins()/begin().
    portduinoConfigureSPI(native_config::g_config.spi_dev.c_str(),
                          native_config::g_config.spi_speed_hz);
    portduinoConfigureGPIO(native_config::g_config.gpio_chip.c_str());
    if (portduinoSetup(argc, argv) != 0) {
        std::fprintf(stderr, "[main] portduinoSetup failed\n");
        return 1;
    }

    // 4) Pin map and EEPROM image.
    native_pinmap::apply();
    EEPROM.begin(EEPROM_SIZE);
    native_pinmap::seed_eeprom_if_unprovisioned();

    // 5) Install signal handlers for clean shutdown.
    std::signal(SIGINT,  on_signal);
    std::signal(SIGTERM, on_signal);

    // 6) Run the firmware sketch.
    setup();
    while (g_running.load(std::memory_order_relaxed)) {
        loop();
    }

    // 7) Flush in-memory EEPROM image so config changes made at runtime
    //    survive across restarts.
    EEPROM.commit();
    std::fprintf(stderr, "[main] shutdown complete\n");
    return 0;
}
