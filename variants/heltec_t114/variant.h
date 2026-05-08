/*
  T114 variant for the Adafruit nRF52 Arduino core.
  Pin definitions follow Heltec's HT-n5262/Mesh Node T114 Rev 2.0 mapping.
*/

#ifndef _VARIANT_HELTEC_T114_LOCAL_
#define _VARIANT_HELTEC_T114_LOCAL_

#define VARIANT_MCK (64000000ul)
#define USE_LFXO

#include "WVariant.h"

#ifdef __cplusplus
extern "C" {
#endif

#define HARD_VERSION_ADDR (0xED000 + 7 * 4096 - 16 - 1)
#define HT_LICENSE_ADDR (0xED000 + 7 * 4096 - 16)
#define HT_LICENSE_ADDR_BASE (0xED000 + 6 * 4096)

#define PINS_COUNT           (48)
#define NUM_DIGITAL_PINS     (48)
#define NUM_ANALOG_INPUTS    (6) // Valid nRF52 ADC pins on this board
#define NUM_ANALOG_OUTPUTS   (0)

// ------------------------------------------------------------------
// LEDs & NeoPixel
// ------------------------------------------------------------------
#define PIN_LED1             (35) // LED_Green
#define PIN_NEOPIXEL         (14) // RGB
#define NEOPIXEL_NUM         2
#define LED_BUILTIN          PIN_LED1
#define LED_GREEN            PIN_LED1
#define LED_BLUE             PIN_LED1
#define LED_STATE_ON         1

// ------------------------------------------------------------------
// Buttons
// ------------------------------------------------------------------
#define PIN_BUTTON1          (42) // Button_User (P1.10 = 42)

// ------------------------------------------------------------------
// Analog Pins (Hardwired to nRF52 ADC channels)
// ------------------------------------------------------------------
#define PIN_A0               (5)
#define PIN_A1               (28)
#define PIN_A2               (29)
#define PIN_A3               (30)
#define PIN_A4               (31)
#define PIN_A5               (4)  // BAT_ADC

static const uint8_t A0 = PIN_A0;
static const uint8_t A1 = PIN_A1;
static const uint8_t A2 = PIN_A2;
static const uint8_t A3 = PIN_A3;
static const uint8_t A4 = PIN_A4;
static const uint8_t A5 = PIN_A5;
#define ADC_RESOLUTION       14

// ------------------------------------------------------------------
// Power & Battery Control
// ------------------------------------------------------------------
#define PIN_VEXT_CTL         (21) // VextCtrl
#define VEXT_ENABLE          1
#define PIN_BAT_ADC          (4)  // BAT_ADC
#define PIN_BAT_ADC_CTL      (6)  // ADC_Ctrl (Enables battery divider)
#define BAT_AMPLIFY          4.9

// ------------------------------------------------------------------
// UART Interfaces
// ------------------------------------------------------------------
#define PIN_SERIAL1_RX       (9)  // UART1_RX
#define PIN_SERIAL1_TX       (10) // UART1_TX

// GNSS (Serial 2)
#define PIN_SERIAL2_RX       (39) // RX_GPS
#define PIN_SERIAL2_TX       (37) // TX_GPS
#define PIN_GPS_PPS          (36) // PPS
#define PIN_GPS_RESET        (38) // RST_GPS
#define PIN_GPS_WAKE         (34) // WAKE_UP

// ------------------------------------------------------------------
// SPI Interfaces
// ------------------------------------------------------------------
#define SPI_INTERFACES_COUNT 2
#define SPI_32MHZ_INTERFACE  1

// SPI 0: LoRa (SX1262)
#define PIN_SPI_MISO         (23) // LoRa_MISO
#define PIN_SPI_MOSI         (22) // LoRa_MOSI
#define PIN_SPI_SCK          (19) // LoRa_SCK
#define PIN_SPI_CS           (24) // LoRa_NSS
#define LORA_DIO1            (20) // DIO1
#define LORA_BUSY            (17) // LoRa_BUSY
#define LORA_RESET           (25) // LoRa_NRSET
static const uint8_t SS   = PIN_SPI_CS;
static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK  = PIN_SPI_SCK;

// SPI 1: TFT Display
#define PIN_SPI1_MOSI        (41) // TFT_SDA
#define PIN_SPI1_SCK         (40) // TFT_SCL
#define PIN_SPI1_MISO        (43) // Display is write-only; must still be a valid pin for SPIClass.
static const uint8_t MOSI1 = PIN_SPI1_MOSI;
static const uint8_t SCK1  = PIN_SPI1_SCK;

// ------------------------------------------------------------------
// TFT Display Control Pins
// ------------------------------------------------------------------
#define PIN_TFT_CS           (11) // TFTCS
#define PIN_TFT_RST          (2)  // TFT_RST
#define PIN_TFT_DC           (12) // TFT_RS
#define PIN_TFT_VDD_CTL      (3)  // TFT_VDD_EN, active low
#define PIN_TFT_LEDA_CTL     (15) // TFT_LED_EN

// ------------------------------------------------------------------
// I2C Interface (Wire)
// Required for Adafruit_BusIO / GFX compilation
// ------------------------------------------------------------------
#define WIRE_INTERFACES_COUNT 1
#define PIN_WIRE_SDA         (13) 
#define PIN_WIRE_SCL         (16)

#ifdef __cplusplus
}
#endif

#endif
