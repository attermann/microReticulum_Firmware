// Copyright (C) 2026, microReticulum_Firmware contributors
//
// Definitions for the runtime-configurable pin globals declared as
// `extern int pin_*` in Boards.h under MCU_VARIANT == MCU_NATIVE.
// Populated from native_config::g_config before LoRa->begin() runs.

#include "config.h"
#include "EEPROMShim.h"
#include "../ROM.h"   // CONF_OK_BYTE and ADDR_* offsets
#include <cstdio>

#ifdef PORTDUINO_LINUX_HARDWARE
#include <PortduinoGPIO.h>
#include <linux/gpio/LinuxGPIOPin.h>
#include <string>
#endif

// --- Pin globals (single definition) ---
int pin_cs           = -1;
int pin_reset        = -1;
int pin_dio          = -1;
int pin_busy         = -1;
int pin_rxen         = -1;
int pin_txen         = -1;
int pin_led_rx       = -1;
int pin_led_tx       = -1;
int pin_tcxo_enable  = -1;
int pin_sclk         = -1;
int pin_mosi         = -1;
int pin_miso         = -1;

// --- EEPROM shim instance (single definition) ---
EEPROMClass EEPROM;

namespace native_pinmap {

// Copy pin numbers from the loaded config into the globals the modem
// drivers consult via LoRa->setPins(). Call once after native_config::load()
// and before setup().
void apply() {
    const auto& c = native_config::g_config;
    pin_cs          = c.pin_cs;
    pin_reset       = c.pin_reset;
    pin_dio         = c.pin_dio;
    pin_busy        = c.pin_busy;
    pin_rxen        = c.pin_rxen;
    pin_txen        = c.pin_txen;
    pin_led_rx      = c.pin_led_rx;
    pin_led_tx      = c.pin_led_tx;
    pin_tcxo_enable = c.pin_tcxo_enable;
    pin_sclk        = c.pin_sclk;
    pin_mosi        = c.pin_mosi;
    pin_miso        = c.pin_miso;
}

// Bind Portduino's logical pin numbers to real libgpiod-backed lines.
// Identity-mapping: the value of `pin_*` in microreticulum.conf is treated
// as both the Portduino logical pin number AND the gpiochip line offset
// (BCM number on a Raspberry Pi). No-op on macOS / cross_platform — the
// `#ifdef PORTDUINO_LINUX_HARDWARE` body is empty there.
//
// Excludes the SPI bus pins (CS, SCLK, MOSI, MISO). Those lines are owned
// by the `spidev` kernel driver via the Pi's device tree; requesting them
// through libgpiod from userspace would conflict.
void bind_linux_gpios() {
#ifdef PORTDUINO_LINUX_HARDWARE
    // gpio_chip from config is a /dev path like "/dev/gpiochip0"; Portduino
    // wants just the trailing chip label ("gpiochip0").
    std::string chipLabel = native_config::g_config.gpio_chip;
    const auto slash = chipLabel.rfind('/');
    if (slash != std::string::npos) chipLabel = chipLabel.substr(slash + 1);

    auto bind = [&](int pin, const char* name) {
        if (pin < 0) return;  // -1 = disabled in config
        gpioBind(new LinuxGPIOPin(pin, chipLabel.c_str(), pin, name));
    };

    const auto& c = native_config::g_config;
    bind(c.pin_reset,       "RESET");
    bind(c.pin_busy,        "BUSY");
    bind(c.pin_dio,         "DIO1");
    bind(c.pin_rxen,        "RXEN");
    bind(c.pin_txen,        "TXEN");
    bind(c.pin_tcxo_enable, "TCXO_EN");
    bind(c.pin_led_rx,      "LED_RX");
    bind(c.pin_led_tx,      "LED_TX");
    // CS / SCLK / MOSI / MISO intentionally left as Portduino sim pins —
    // spidev owns them. The modem driver's digitalWrite(_ss, ...) goes to
    // the sim path, but spidev toggles CS itself per transaction.

    std::fprintf(stderr, "[pinmap] bound Linux GPIOs on %s\n",
                 chipLabel.c_str());
#endif
}

// Seed the EEPROM image with the LoRa radio settings from config so the
// existing eeprom_conf_load() path in Utilities.h picks them up during
// setup(). Only runs if the EEPROM image lacks the CONF_OK_BYTE sentinel
// (first boot or after explicit reset).
void seed_eeprom_if_unprovisioned() {
    if (EEPROM.read(ADDR_CONF_OK) == CONF_OK_BYTE) return;

    const auto& c = native_config::g_config;
    EEPROM.write(ADDR_CONF_SF,  c.lora_sf);
    EEPROM.write(ADDR_CONF_CR,  c.lora_cr);
    EEPROM.write(ADDR_CONF_TXP, static_cast<uint8_t>(c.lora_txp));

    EEPROM.write(ADDR_CONF_FREQ + 0, (c.lora_freq_hz >> 24) & 0xFF);
    EEPROM.write(ADDR_CONF_FREQ + 1, (c.lora_freq_hz >> 16) & 0xFF);
    EEPROM.write(ADDR_CONF_FREQ + 2, (c.lora_freq_hz >>  8) & 0xFF);
    EEPROM.write(ADDR_CONF_FREQ + 3, (c.lora_freq_hz      ) & 0xFF);

    EEPROM.write(ADDR_CONF_BW + 0, (c.lora_bw_hz >> 24) & 0xFF);
    EEPROM.write(ADDR_CONF_BW + 1, (c.lora_bw_hz >> 16) & 0xFF);
    EEPROM.write(ADDR_CONF_BW + 2, (c.lora_bw_hz >>  8) & 0xFF);
    EEPROM.write(ADDR_CONF_BW + 3, (c.lora_bw_hz      ) & 0xFF);

    EEPROM.write(ADDR_CONF_OK, CONF_OK_BYTE);
    EEPROM.commit();
    std::fprintf(stderr, "[pinmap] seeded EEPROM image from config\n");
}

} // namespace native_pinmap
