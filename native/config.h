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

    // Modem pin map. -1 disables a pin. CS/RESET are required; the rest
    // are conditional on the modem variant.
    int pin_cs           = -1;
    int pin_reset        = -1;
    int pin_busy         = -1;
    int pin_dio          = -1;
    int pin_rxen         = -1;
    int pin_txen         = -1;
    int pin_tcxo_enable  = -1;
    int pin_sclk         = -1;
    int pin_mosi         = -1;
    int pin_miso         = -1;
    int pin_led_rx       = -1;
    int pin_led_tx       = -1;

    // LoRa radio settings. These get written into the EEPROM image at
    // startup so the existing eeprom_conf_load() path picks them up.
    uint32_t lora_freq_hz = 915000000;
    uint32_t lora_bw_hz   = 125000;
    uint8_t  lora_sf      = 8;
    uint8_t  lora_cr      = 5;
    int8_t   lora_txp     = 17;
};

extern Config g_config;

// Parse a key=value file. Lines beginning with '#' are comments. Missing
// keys keep their default values. Returns true on success (file read
// without parse errors); a missing file is treated as "use defaults"
// and returns true.
bool load(const std::string& path);

} // namespace native_config

#endif // NATIVE_CONFIG_H
