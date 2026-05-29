// Copyright (C) 2026, microReticulum_Firmware contributors
//
// Native-build runtime configuration. Loaded once at startup from a key=value
// text file (default: ./microreticulum.conf). All values have sensible
// defaults so the daemon is runnable with an empty config file for testing.

#ifndef NATIVE_CONFIG_H
#define NATIVE_CONFIG_H

#include <cstdint>
#include <string>

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
};

extern Config g_config;

// Parse a key=value file. Lines beginning with '#' are comments. Missing
// keys keep their default values. Returns true on success (file read
// without parse errors); a missing file is treated as "use defaults"
// and returns true.
bool load(const std::string& path);

} // namespace native_config

#endif // NATIVE_CONFIG_H
