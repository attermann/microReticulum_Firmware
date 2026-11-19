/*
 * variant.h - Seeed Tracker T1000-E (nRF52840 + LR1110)
 *
 * Adapted from the MeshCore t1000-e variant for the Adafruit nRF52 Arduino
 * core used by this firmware. Pin map verified against the Seeed schematic:
 * https://github.com/Seeed-Studio/Seeed-Tracker-T1000-E-for-LoRaWAN-dev-board
 */

#pragma once

#ifndef _VARIANT_T1000E_
#define _VARIANT_T1000E_

/** Master clock frequency */
#define VARIANT_MCK (64000000ul)

// 32.768 kHz low-frequency clock source: crystal oscillator.
#define USE_LFXO
// #define USE_LFRC

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/
#include <Arduino.h>

#ifdef __cplusplus
extern "C" {
#endif

// Number of pins defined in the pin-description array.
#define PINS_COUNT           (48)
#define NUM_DIGITAL_PINS     (48)
#define NUM_ANALOG_INPUTS    (6)
#define NUM_ANALOG_OUTPUTS   (0)

/*----------------------------------------------------------------------------
 *        Power
 *----------------------------------------------------------------------------*/

#define NRF_APM                        // detect USB power

#define BATTERY_PIN          (2)       // P0.02 / AIN0
#define BATTERY_IMMUTABLE
#define ADC_MULTIPLIER       (2.0F)
#define AREF_VOLTAGE         (3.0)

#define EXT_CHRG_DETECT      (35)      // P1.03
#define EXT_PWR_DETECT       (5)       // P0.05

#define ADC_RESOLUTION       (14)

/*----------------------------------------------------------------------------
 *        LEDs
 *----------------------------------------------------------------------------*/

#define LED_BUILTIN          (-1)
#define LED_BLUE             (-1)
#define LED_GREEN            (24)      // P0.24
#define LED_PIN              LED_GREEN

#define LED_STATE_ON         HIGH

/*----------------------------------------------------------------------------
 *        Buttons
 *----------------------------------------------------------------------------*/

#define PIN_BUTTON1          (6)       // P0.06
#define BUTTON_PIN           PIN_BUTTON1

/*----------------------------------------------------------------------------
 *        Serial
 *----------------------------------------------------------------------------*/

#define PIN_SERIAL1_RX       (14)      // P0.14 (GPS)
#define PIN_SERIAL1_TX       (13)      // P0.13

#define PIN_SERIAL2_RX       (17)      // P0.17
#define PIN_SERIAL2_TX       (16)      // P0.16

/*----------------------------------------------------------------------------
 *        Wire (I2C)
 *----------------------------------------------------------------------------*/

#define HAS_WIRE             (1)
#define WIRE_INTERFACES_COUNT (1)

#define PIN_WIRE_SDA         (26)      // P0.26
#define PIN_WIRE_SCL         (27)      // P0.27

/*----------------------------------------------------------------------------
 *        SPI (shared with the LR1110)
 *----------------------------------------------------------------------------*/

#define SPI_INTERFACES_COUNT (1)

#define PIN_SPI_MISO         (40)      // P1.08
#define PIN_SPI_MOSI         (41)      // P1.09
#define PIN_SPI_SCK          (11)      // P0.11
#define PIN_SPI_NSS          (12)      // P0.12

static const uint8_t SS  = PIN_SPI_NSS;
static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK  = PIN_SPI_SCK;

/*----------------------------------------------------------------------------
 *        LR1110
 *----------------------------------------------------------------------------*/

#define LORA_DIO_1           (33)      // P1.01 (IRQ)
#define LORA_NSS             (PIN_SPI_NSS)
#define LORA_RESET           (42)      // P1.10
#define LORA_BUSY            (7)       // P0.07
#define LORA_SCLK            (PIN_SPI_SCK)
#define LORA_MISO            (PIN_SPI_MISO)
#define LORA_MOSI            (PIN_SPI_MOSI)

// TCXO supplied via DIO3 at 1.6 V; on-chip RF switch driven by DIO5..DIO8.
#define LR11X0_DIO_AS_RF_SWITCH   true
#define LR11X0_DIO3_TCXO_VOLTAGE  1.6

/*----------------------------------------------------------------------------
 *        GPS
 *----------------------------------------------------------------------------*/

#define HAS_GPS              1
#define GPS_RX_PIN           PIN_SERIAL1_RX
#define GPS_TX_PIN           PIN_SERIAL1_TX

#define GPS_EN               (43)      // P1.11
#define GPS_RESET            (47)      // P1.15

#define GPS_VRTC_EN          (8)       // P0.08
#define GPS_SLEEP_INT        (44)      // P1.12
#define GPS_RTC_INT          (15)      // P0.15
#define GPS_RESETB           (46)      // P1.14

/*----------------------------------------------------------------------------
 *        Sensors / power rails
 *----------------------------------------------------------------------------*/

#define PIN_3V3_EN           (38)      // P1.06 — power to sensors / 3V3 rail
#define PIN_3V3_ACC_EN       (39)      // P1.07
#define SENSOR_EN            (4)       // P0.04
#define TEMP_SENSOR          (31)      // P0.31 / AIN7
#define LUX_SENSOR           (29)      // P0.29 / AIN5

/*----------------------------------------------------------------------------
 *        Buzzer
 *----------------------------------------------------------------------------*/

#define BUZZER_EN            (37)      // P1.05
#define BUZZER_PIN           (25)      // P0.25

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#endif
