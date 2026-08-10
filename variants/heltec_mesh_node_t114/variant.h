/*
 Copyright (c) 2014-2015 Arduino LLC.  All right reserved.
 Copyright (c) 2016 Sandeep Mistry All right reserved.
 Copyright (c) 2018, Adafruit Industries (adafruit.com)

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 See the GNU Lesser General Public License for more details.
 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef _VARIANT_HELTEC_NRF_
#define _VARIANT_HELTEC_NRF_

/** Master clock frequency */
#define VARIANT_MCK (64000000ul)

#define USE_LFXO // Board uses 32 kHz crystal for LF clock

#include "WVariant.h"

#ifdef __cplusplus
extern "C" {
#endif

#define HELTEC_MESH_NODE_T114

/* -----------------------------------------------------------------------
 * Pin count
 * --------------------------------------------------------------------- */
#define PINS_COUNT        (48)
#define NUM_DIGITAL_PINS  (48)
#define NUM_ANALOG_INPUTS  (1)
#define NUM_ANALOG_OUTPUTS (0)

/* -----------------------------------------------------------------------
 * LEDs  (active-low)
 * P1.03 = green LED
 * --------------------------------------------------------------------- */
#define PIN_LED1      (32 + 3)
#define LED_BLUE      PIN_LED1   // alias expected by some BSP code
#define LED_GREEN     PIN_LED1
#define LED_STATE_ON  0          // LOW = lit

/* -----------------------------------------------------------------------
 * NeoPixels
 * P0.14 — 2× WS2812B RGB LEDs
 * --------------------------------------------------------------------- */
#define HAS_NEOPIXEL
#define NEOPIXEL_COUNT 2
#define NEOPIXEL_DATA  14
#define NEOPIXEL_TYPE  (NEO_GRB + NEO_KHZ800)

/* -----------------------------------------------------------------------
 * Button
 * P1.10
 * --------------------------------------------------------------------- */
#define PIN_BUTTON1 (32 + 10)

/* -----------------------------------------------------------------------
 * I²C
 * Bus 0: P0.26 SDA / P0.27 SCL  — RTC footprint (unpopulated on V1)
 * Bus 1: P0.16 SDA / P0.13 SCL  — header pins, general use
 * --------------------------------------------------------------------- */
#define WIRE_INTERFACES_COUNT 2

#define PIN_WIRE_SDA  (0 + 26)
#define PIN_WIRE_SCL  (0 + 27)

#define PIN_WIRE1_SDA (0 + 16)
#define PIN_WIRE1_SCL (0 + 13)

/* -----------------------------------------------------------------------
 * QSPI Flash  —  MX25R1635F (2 MB)
 * --------------------------------------------------------------------- */
#define PIN_QSPI_SCK  (32 + 14)
#define PIN_QSPI_CS   (32 + 15)
#define PIN_QSPI_IO0  (32 + 12)   // MOSI / DI
#define PIN_QSPI_IO1  (32 + 13)   // MISO / DO
#define PIN_QSPI_IO2  (0  +  7)   // WP   (unused in 2-bit mode)
#define PIN_QSPI_IO3  (0  +  5)   // HOLD (unused in 2-bit mode)

#define EXTERNAL_FLASH_DEVICES MX25R1635F
#define EXTERNAL_FLASH_USE_QSPI

/* -----------------------------------------------------------------------
 * LoRa radio — SX1262  (SPI0)
 * CS   P0.24
 * DIO1 P0.20
 * BUSY P0.17
 * RST  P0.25
 * DIO2 used internally as RF switch (do not drive from CPU)
 * DIO3 drives the TCXO at 1.8 V
 * --------------------------------------------------------------------- */
#define USE_SX1262
#define SX126X_CS            (0 + 24)
#define LORA_CS              (0 + 24)
#define SX126X_DIO1          (0 + 20)
#define SX126X_BUSY          (0 + 17)
#define SX126X_RESET         (0 + 25)
#define SX126X_DIO2_AS_RF_SWITCH
#define SX126X_DIO3_TCXO_VOLTAGE 1.8

/* -----------------------------------------------------------------------
 * Serial1 — required by the BSP's Uart.cpp regardless of whether it is used.
 * These are the physical UART pins on the board; leave unconnected if unused.
 * P1.07 = TX (toward board), P1.05 = RX (toward board)
 * --------------------------------------------------------------------- */
#define PIN_SERIAL1_RX (32 + 7)
#define PIN_SERIAL1_TX (32 + 5)

/* -----------------------------------------------------------------------
 * SPI0 — LoRa
 * MISO P0.23 / MOSI P0.22 / SCK P0.19
 * --------------------------------------------------------------------- */
#define SPI_INTERFACES_COUNT 1
#define PIN_SPI_MISO (0 + 23)
#define PIN_SPI_MOSI (0 + 22)
#define PIN_SPI_SCK  (0 + 19)

/* -----------------------------------------------------------------------
 * Battery ADC
 * P0.04 = AIN2 via resistor divider (100k / 390k → ×4.916)
 * ADC_CTRL (P0.06) must be driven HIGH to enable the divider.
 * --------------------------------------------------------------------- */
#define ADC_CTRL              6
#define ADC_CTRL_ENABLED      HIGH
#define BATTERY_PIN           4
#define ADC_RESOLUTION        14
#define BATTERY_SENSE_RESOLUTION_BITS 12
#define BATTERY_SENSE_RESOLUTION      4096.0
#undef  AREF_VOLTAGE
#define AREF_VOLTAGE          3.0
#define VBAT_AR_INTERNAL      AR_INTERNAL_3_0
#define ADC_MULTIPLIER        (4.916F)

#ifdef __cplusplus
}
#endif

#endif /* _VARIANT_HELTEC_NRF_ */
