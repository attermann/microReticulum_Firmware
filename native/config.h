// Copyright (C) 2026, microReticulum_Firmware contributors
//
// Native-build runtime configuration. Loaded once at startup from a key=value
// text file (default: ./rnoded.conf). All values have sensible
// defaults so the daemon is runnable with an empty config file for testing.

#ifndef NATIVE_CONFIG_H
#define NATIVE_CONFIG_H

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace native_config {

struct Config {
    // Storage
    std::string data_dir = ".";

    // SPI / GPIO devices
    std::string spi_dev   = "/dev/spidev0.0";
    std::string gpio_chip = "/dev/gpiochip0";
    uint32_t    spi_speed_hz = 8000000;

    // Modem pin map. Defaults are deliberately small positive integers
    // so they fall inside Portduino's simulated GPIO range (NUM_GPIOS=64
    // on cross_platform builds). On native Linux, override with real
    // BCM/libgpiod line offsets via the config file.
    int pin_cs           = 0;
    int pin_reset        = 1;
    int pin_busy         = 2;
    int pin_dio          = 3;
    int pin_rxen         = 4;
    int pin_txen         = 5;
    int pin_tcxo_enable  = -1;  // -1 = disabled
    int pin_sclk         = 6;
    int pin_mosi         = 7;
    int pin_miso         = 8;
    int pin_led_rx       = 9;
    int pin_led_tx       = 10;

    // Additional GPIOs to drive to an active level for the duration the
    // radio is in use (asserted before LoRa->begin(), de-asserted after
    // LoRa->end()). Use for external LDO enables, antenna-switch power,
    // PA bias, etc. Each entry is (pin, active_high): default active is
    // HIGH; in rnoded.conf, append ":low" to a pin to invert.
    //   radio_enable_pins = 22,25:low,33
    std::vector<std::pair<int,bool>> radio_enable_pins;

    // SX1262-only. Override the per-board compile-time TCXO voltage byte
    // written to the chip's DIO3-TCXO-control register. Float volts;
    // 0.0f = unset (keep compile-time default). Accepts the meshtasticd
    // YAML conventions: a float like 1.8, or the literal "true" (alias
    // for 1.8 V) / "false" (no override).
    float dio3_tcxo_voltage = 0.0f;

    // SX1262-only. When true, configure the chip's DIO2 line as the RF
    // switch driver (the SX1262 toggles the antenna switch itself from
    // its TX/RX state, no host GPIO needed). Default false on native;
    // embedded targets keep their per-board compile-time #define.
    bool dio2_as_rf_switch = false;

    // LoRa radio settings. These get written into the EEPROM image at
    // startup so the existing eeprom_conf_load() path picks them up.
    uint32_t lora_freq_hz = 915000000;
    uint32_t lora_bw_hz   = 125000;
    uint8_t  lora_sf      = 8;
    uint8_t  lora_cr      = 5;
    int8_t   lora_txp     = 17;

    // Modem family selected at runtime. Values match the constants in
    // Modem.h: SX1262 (0x03), SX1276 (0x01), SX1278 (0x02), SX1280 (0x04).
    // Default 0x03 mirrors what was the compile-time default for the
    // `native` env before runtime selection landed.
    uint8_t  modem        = 0x03;

    // When true, a TX failure (LoRa->endPacket() returning 0, i.e. the
    // modem didn't report TX_DONE within the driver's timeout) triggers
    // hard_reset() — on native that means re-execing the daemon. The
    // default is false because a re-exec is expensive on native (releases
    // libgpiod handles, closes the KISS-TCP socket, etc.) and one bad
    // packet doesn't justify killing the process. When false, the firmware
    // logs the error to KISS and returns the modem to RX so subsequent
    // packets can still flow. Set to true if your deployment really needs
    // the embedded behavior of "reboot on any TX timeout".
    bool reboot_on_tx_failure = false;

    // KISS-over-TCP host transport. The native daemon listens on this
    // localhost port for a single client (rnodeconf, an RNS KISSInterface,
    // etc.). 7633 matches the ESP32 WiFi-remote convention in Remote.h.
    uint16_t kiss_tcp_port   = 7633;

    // Bind the KISS TCP server on all interfaces (0.0.0.0) instead of just
    // 127.0.0.1. Off by default — exposing this socket to the network has
    // no auth/encryption, so the daemon should only be reachable across a
    // network you trust. Turning this on is the explicit way to opt in.
    bool     kiss_tcp_public = false;
};

extern Config g_config;

// Parse a key=value file. Lines beginning with '#' are comments. Missing
// keys keep their default values. Returns true on success (file read
// without parse errors); a missing file is treated as "use defaults"
// and returns true.
bool load(const std::string& path);

} // namespace native_config

#endif // NATIVE_CONFIG_H
