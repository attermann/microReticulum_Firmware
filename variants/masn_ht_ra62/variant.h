#pragma once

#ifndef _VARIANT_MASN_HT_RA62_
#define _VARIANT_MASN_HT_RA62_

#include <Arduino.h>

#define VARIANT_MCK (64000000ul)
#define USE_LFXO

#define PINS_COUNT (48)
#define NUM_DIGITAL_PINS (48)
#define NUM_ANALOG_INPUTS (6)
#define NUM_ANALOG_OUTPUTS (0)

// No status LED is configured yet
#define LED_BUILTIN (-1)

// Compatibility with Adafruit Bluefruit
#define LED_BLUE (-1)
#define LED_STATE_ON HIGH

// UART1 pins are not assigned yet
#define PIN_SERIAL1_RX (0xff)
#define PIN_SERIAL1_TX (0xff)

// MASN V1 HT-RA62 SPI
#define SPI_INTERFACES_COUNT 1
#define PIN_SPI_MISO (2)
#define PIN_SPI_MOSI (47)
#define PIN_SPI_SCK  (43)

static const uint8_t SS = 45;
static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK = PIN_SPI_SCK;

// I2C: INA3221 (0x40) and BME280 (0x76), verified on hardware
#define WIRE_INTERFACES_COUNT 1
#define PIN_WIRE_SDA (36)  // P1.04 (schematic V1.2)
#define PIN_WIRE_SCL (11)  // P0.11
// Unused analog pins
#define PIN_A0 (0xff)
#define PIN_A1 (0xff)
#define PIN_A2 (0xff)
#define PIN_A3 (0xff)
#define PIN_A4 (0xff)
#define PIN_A5 (0xff)

#endif
