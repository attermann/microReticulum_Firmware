#pragma once

#ifndef _VARIANT_RAK3401_
#define _VARIANT_RAK3401_

#define RAK4630

/** Master clock frequency */
#define VARIANT_MCK (64000000ul)

#define USE_LFXO // Board uses 32khz crystal for LF
// define USE_LFRC    // Board uses RC for LF

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/
#include <Arduino.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

	/*
	 * WisBlock Base GPIO definitions
	 */
	static const uint8_t WB_IO1 = 17;	   // SLOT_A SLOT_B
	static const uint8_t WB_IO2 = 34;	   // SLOT_A SLOT_B
	static const uint8_t WB_IO3 = 21;	   // SLOT_C
	static const uint8_t WB_IO4 = 4;	   // SLOT_C
	static const uint8_t WB_IO5 = 9;	   // SLOT_D
	static const uint8_t WB_IO6 = 10;	   // SLOT_D
	static const uint8_t WB_SW1 = 33;	   // IO_SLOT
	static const uint8_t WB_A0 = 5;	   // IO_SLOT
	static const uint8_t WB_A1 = 31;	   // IO_SLOT
	static const uint8_t WB_I2C1_SDA = 13; // SENSOR_SLOT IO_SLOT
	static const uint8_t WB_I2C1_SCL = 14; // SENSOR_SLOT IO_SLOT
	static const uint8_t WB_I2C2_SDA = 24; // IO_SLOT
	static const uint8_t WB_I2C2_SCL = 25; // IO_SLOT
	static const uint8_t WB_SPI_CS = 26;   // IO_SLOT
	static const uint8_t WB_SPI_CLK = 3;   // IO_SLOT
	static const uint8_t WB_SPI_MISO = 29; // IO_SLOT
	static const uint8_t WB_SPI_MOSI = 30; // IO_SLOT

// Number of pins defined in PinDescription array
#define PINS_COUNT (48)
#define NUM_DIGITAL_PINS (48)
#define NUM_ANALOG_INPUTS (6)
#define NUM_ANALOG_OUTPUTS (0)

// LEDs
#define PIN_LED1 (35)
#define PIN_LED2 (36)

#define LED_BUILTIN PIN_LED1
#define LED_CONN PIN_LED2

#define LED_GREEN PIN_LED1
#define LED_BLUE PIN_LED2

#define LED_STATE_ON 1 // State when LED is litted

/*
 * Analog pins
 */
#define PIN_A0 (5)	//(3)
#define PIN_A1 (31) //(4)
#define PIN_A2 (28)
#define PIN_A3 (29)
#define PIN_A4 (30)
#define PIN_A5 (31)
#define PIN_A6 (0xff)
#define PIN_A7 (0xff)

	static const uint8_t A0 = PIN_A0;
	static const uint8_t A1 = PIN_A1;
	static const uint8_t A2 = PIN_A2;
	static const uint8_t A3 = PIN_A3;
	static const uint8_t A4 = PIN_A4;
	static const uint8_t A5 = PIN_A5;
	static const uint8_t A6 = PIN_A6;
	static const uint8_t A7 = PIN_A7;
#define ADC_RESOLUTION 14

// Other pins
#define PIN_AREF (2)
// NFC pins intentionally left undefined — P0.09 and P0.10 are used by
// the RAK13302 SX1262 (BUSY and DIO) and must not be claimed by the
// NFC peripheral.  The Adafruit BSP skips NFCT init when these are absent.

	static const uint8_t AREF = PIN_AREF;

/*
 * Serial interfaces
 */
// TXD1 RXD1 on Base Board
#define PIN_SERIAL1_RX (15)
#define PIN_SERIAL1_TX (16)

// Connected to Jlink CDC
#define PIN_SERIAL2_RX (8)
#define PIN_SERIAL2_TX (6)

/*
 * SPI Interfaces
 *
 * Primary SPI (SPI0/SPIM2) on P1 — matches stock rak4630 BSP layout.
 * Secondary SPI (SPI1/SPIM3) on P0 — RAK13302 LoRa bus pins.
 * spiModem in Config.h explicitly constructs on NRF_SPIM2 with P0 pins
 * from Boards.h, overriding the BSP's SPI0 pin assignment at runtime.
 */
#define SPI_INTERFACES_COUNT 2

#define PIN_SPI_MISO (45)
#define PIN_SPI_MOSI (44)
#define PIN_SPI_SCK (43)

#define PIN_SPI1_MISO (29)
#define PIN_SPI1_MOSI (30)
#define PIN_SPI1_SCK (3)

	static const uint8_t SS = 42;
	static const uint8_t MOSI = PIN_SPI_MOSI;
	static const uint8_t MISO = PIN_SPI_MISO;
	static const uint8_t SCK = PIN_SPI_SCK;

/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 1

#define PIN_WIRE_SDA (13)
#define PIN_WIRE_SCL (14)

// QSPI Pins
#define PIN_QSPI_SCK 3
#define PIN_QSPI_CS 26
#define PIN_QSPI_IO0 30
#define PIN_QSPI_IO1 29
#define PIN_QSPI_IO2 28
#define PIN_QSPI_IO3 2

// On-board QSPI Flash
#define EXTERNAL_FLASH_DEVICES IS25LP080D
#define EXTERNAL_FLASH_USE_QSPI

// RAK13302 SX1262 + SKY66122 FEM control
// SKY66122 CSD and CPS are tied together on the RAK13302 PCB, routed to IO3 (P0.21).
// Setting HIGH enables the FEM (LNA active for RX; PA path available for TX).
// TX/RX switching (CTX) is driven by SX1262 DIO2 via SetDIO2AsRfSwitchCtrl — no GPIO needed.
#define SX126X_POWER_EN (21)  // P0.21 = WB_IO3 -> SKY66122 CSD+CPS (FEM enable)

// 5V boost + 3.3V switched peripheral rail (powers RAK13302 FEM).
// Must be held HIGH during radio operation — do not toggle for power saving.
#define PIN_3V3_EN (34)

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#endif
