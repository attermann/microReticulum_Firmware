// Copyright (C) 2026, microReticulum_Firmware contributors
//
// Definitions for the runtime-configurable pin globals declared as
// `extern int pin_*` in Boards.h under MCU_VARIANT == MCU_NATIVE.
// Populated from native_config::g_config before LoRa->begin() runs.

#include "config.h"
#include "EEPROMShim.h"
#include "../Boards.h"   // PRODUCT_NATIVE_LINUX / MODEL_60 / BOARD_NATIVE_LINUX constants
#include "../ROM.h"      // CONF_OK_BYTE, INFO_LOCK_BYTE, ADDR_* offsets
#include <cstdio>

#ifdef PORTDUINO_LINUX_HARDWARE
#include <PortduinoGPIO.h>
#include <linux/gpio/LinuxGPIOPin.h>
#include <gpiod.h>
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
    // Portduino's LinuxGPIOPin::getLine() (LinuxGPIOPin.cpp:225-249)
    // iterates /dev/gpiochip* and matches by gpiod_chip_label() — the
    // SoC-specific identifier (e.g. "pinctrl-bcm2711" on a Pi, "gpio1"
    // on Rockchip), NOT the device filename. So we can't just strip
    // "/dev/" off the user's gpio_chip path and pass that as the label.
    // Open the chip ourselves to resolve its real label, then hand that
    // to LinuxGPIOPin.
    const char* chipPath = native_config::g_config.gpio_chip.c_str();
    struct gpiod_chip* probe = gpiod_chip_open(chipPath);
    if (!probe) {
        std::fprintf(stderr,
            "[pinmap] could not open %s: %s — GPIO binding skipped\n",
            chipPath, std::strerror(errno));
        return;
    }
    const std::string chipLabel = gpiod_chip_label(probe);
    gpiod_chip_close(probe);

    // Pattern adopted from meshtasticd (PortduinoGlue.cpp::initGPIOPin):
    // construct the pin, mark it silent BEFORE binding, then hand to
    // gpioBind. Portduino's base GPIOPin::writePin/refreshState/setPinMode
    // all log unconditionally via the unfiltered Portduino log(); the only
    // way to quiet them is the per-pin silent flag. Once silenced, normal
    // SPI activity (CS toggles) and Portduino's ISR-poll thread no longer
    // drown the output — the modem driver's own Serial logs come through.
    //
    // SCLK / MOSI / MISO are always claimed by the spidev driver via the
    // pinctrl device tree node, so we never bind those.
    //
    // CS gets a silent SimGPIOPin rather than a LinuxGPIOPin: spidev owns
    // the SPI controller's hardware CS automatically across each
    // SPI_IOC_MESSAGE ioctl, so the modem driver's digitalWrite(_ss, ...)
    // is intentionally a no-op at the OS level. Binding a SimGPIOPin (vs.
    // letting it default to "Unbound") just lets us flip its silent flag.
    auto bind_linux = [&](int pin, const char* name) {
        if (pin < 0) return;  // -1 = disabled in config
        try {
            GPIOPin* p = new LinuxGPIOPin(pin, chipLabel.c_str(), pin, name);
            p->setSilent();
            gpioBind(p);
        } catch (const std::exception& e) {
            std::fprintf(stderr,
                "[pinmap] %s (line %d on %s) NOT bound: %s — leaving as "
                "Portduino sim (typically because another driver owns the line)\n",
                name, pin, chipLabel.c_str(), e.what());
        }
    };
    auto bind_sim_silent = [&](int pin, const char* name) {
        if (pin < 0) return;
        GPIOPin* p = new SimGPIOPin(pin, name);
        p->setSilent();
        gpioBind(p);
    };

    const auto& c = native_config::g_config;
    bind_sim_silent(c.pin_cs,    "CS");      // spidev owns the real CS
    bind_linux(c.pin_reset,      "RESET");
    bind_linux(c.pin_busy,       "BUSY");
    bind_linux(c.pin_dio,        "DIO1");
    bind_linux(c.pin_rxen,       "RXEN");
    bind_linux(c.pin_txen,       "TXEN");
    bind_linux(c.pin_tcxo_enable,"TCXO_EN");
    bind_linux(c.pin_led_rx,     "LED_RX");
    bind_linux(c.pin_led_tx,     "LED_TX");

    std::fprintf(stderr, "[pinmap] bound Linux GPIOs on %s (label=%s)\n",
                 chipPath, chipLabel.c_str());
#endif
}

// Seed the EEPROM image with the LoRa radio settings from config so the
// existing eeprom_conf_load() path in Utilities.h picks them up during
// setup(). Only runs if the EEPROM image lacks the CONF_OK_BYTE sentinel
// (first boot or after explicit reset).
void seed_eeprom_if_unprovisioned() {
    // The firmware reads logical addresses via Config.h's
    //   #define eeprom_addr(a) (a + EEPROM_OFFSET)
    // so we must apply the same translation when writing, or our bytes
    // land 824 bytes (EEPROM_SIZE - EEPROM_RESERVED) off from where
    // validate_status() and eeprom_conf_load() look for them. Local
    // helper avoids repeating the offset on every write.
    auto write_at = [](int addr, uint8_t val) {
        EEPROM.write(addr + EEPROM_OFFSET, val);
    };
    auto read_at = [](int addr) -> uint8_t {
        return EEPROM.read(addr + EEPROM_OFFSET);
    };

    if (read_at(ADDR_CONF_OK) == CONF_OK_BYTE) return;

    // --- Device-identity provisioning ---
    // Write enough of the rnodeconf-style provisioning header that
    // eeprom_lock_set(), eeprom_product_valid(), eeprom_model_valid(),
    // and eeprom_hwrev_valid() in Utilities.h all return true on native
    // builds. The MD5 checksum check (eeprom_checksum_valid()) is gated
    // out via -DDISABLE_FIRMWARE_CHECKSUM in the native PlatformIO envs,
    // so we deliberately don't compute / write ADDR_CHKSUM here.
    write_at(ADDR_PRODUCT,   PRODUCT_NATIVE_LINUX);   // 0x60
    write_at(ADDR_MODEL,     MODEL_60);               // 0x60
    write_at(ADDR_HW_REV,    1);                      // any non-0x00/0xFF
    write_at(ADDR_INFO_LOCK, INFO_LOCK_BYTE);         // 0x73

    // --- LoRa radio configuration ---
    const auto& c = native_config::g_config;
    write_at(ADDR_CONF_SF,  c.lora_sf);
    write_at(ADDR_CONF_CR,  c.lora_cr);
    write_at(ADDR_CONF_TXP, static_cast<uint8_t>(c.lora_txp));

    write_at(ADDR_CONF_FREQ + 0, (c.lora_freq_hz >> 24) & 0xFF);
    write_at(ADDR_CONF_FREQ + 1, (c.lora_freq_hz >> 16) & 0xFF);
    write_at(ADDR_CONF_FREQ + 2, (c.lora_freq_hz >>  8) & 0xFF);
    write_at(ADDR_CONF_FREQ + 3, (c.lora_freq_hz      ) & 0xFF);

    write_at(ADDR_CONF_BW + 0, (c.lora_bw_hz >> 24) & 0xFF);
    write_at(ADDR_CONF_BW + 1, (c.lora_bw_hz >> 16) & 0xFF);
    write_at(ADDR_CONF_BW + 2, (c.lora_bw_hz >>  8) & 0xFF);
    write_at(ADDR_CONF_BW + 3, (c.lora_bw_hz      ) & 0xFF);

    write_at(ADDR_CONF_OK, CONF_OK_BYTE);
    EEPROM.commit();
    std::fprintf(stderr, "[pinmap] seeded EEPROM image from config\n");
}

} // namespace native_pinmap
