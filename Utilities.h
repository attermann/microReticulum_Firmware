// Copyright (C) 2023, Mark Qvist

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include "Config.h"

#if HAS_EEPROM
    #include <EEPROM.h>
#elif PLATFORM == PLATFORM_NRF52
    #include <Adafruit_LittleFS.h>
    #include <InternalFileSystem.h>
    using namespace Adafruit_LittleFS_Namespace;
    #define EEPROM_FILE_LEGACY   "eeprom"
    #define EEPROM_FILE_ROM      "eeprom_rom"
    #define EEPROM_FILE_CONF     "eeprom_conf"
    #define EEPROM_FILE_DEFAULTS "eeprom_defaults"
    int written_bytes_rom = 0;
    int written_bytes_conf = 0;
    File file_rom(InternalFS);
    File file_conf(InternalFS);
    File file_defaults(InternalFS);

    // Raw flash page for EEPROM backup (one 4KB page before LittleFS at 0xED000)
    // Used for auto-recovery when LittleFS is corrupted by power loss
    #define EEPROM_BACKUP_ADDR 0xEC000

    // Raw flash backup functions (flash_nrf5x is compiled as part of InternalFileSystem)
    extern "C" {
        void flash_nrf5x_flush(void);
        bool flash_nrf5x_erase(uint32_t addr);
        int  flash_nrf5x_write(uint32_t dst, void const* src, uint32_t len);
    }

    #define EEPROM_BACKUP_MAGIC_0 'R'
    #define EEPROM_BACKUP_MAGIC_1 'N'
    #define EEPROM_BACKUP_MAGIC_2 'B'
    #define EEPROM_BACKUP_MAGIC_3 'K'
    #define EEPROM_BACKUP_HEADER_SIZE 8
    #define IDENTITY_KEY_SIZE 64
    // v1 layout: [magic 4B][version 1B][reserved 3B][ROM][CONF][checksum 2B]
    // v2 layout: [magic 4B][version 1B][flags 1B][reserved 2B][ROM][CONF]
    //            [ID_MAGIC 2B "ID"][transport_len 1B][transport_id 64B]
    //            [lxmf_len 1B][lxmf_id 64B][checksum 2B]
    #define BACKUP_FLAG_HAS_IDENTITY 0x01
    #define IDENTITY_SECTION_MAGIC_0 'I'
    #define IDENTITY_SECTION_MAGIC_1 'D'
    #define IDENTITY_SECTION_SIZE (2 + 1 + IDENTITY_KEY_SIZE + 1 + IDENTITY_KEY_SIZE) // 132 bytes

    bool eeprom_backup_valid() {
        const uint8_t* p = (const uint8_t*)EEPROM_BACKUP_ADDR;
        return (p[0] == EEPROM_BACKUP_MAGIC_0 && p[1] == EEPROM_BACKUP_MAGIC_1 &&
                p[2] == EEPROM_BACKUP_MAGIC_2 && p[3] == EEPROM_BACKUP_MAGIC_3);
    }

    // Compute total backup size based on version/flags at the given raw flash pointer
    static uint16_t backup_total_size(const uint8_t* p) {
        uint16_t total = EEPROM_BACKUP_HEADER_SIZE + 2 * EEPROM_SIZE;
        if (p[4] >= 0x02 && (p[5] & BACKUP_FLAG_HAS_IDENTITY)) {
            total += IDENTITY_SECTION_SIZE;
        }
        return total;
    }

    static bool backup_checksum_valid(const uint8_t* p) {
        uint16_t total = backup_total_size(p);
        uint16_t sum = 0;
        for (uint16_t i = 0; i < total; i++) sum += p[i];
        uint16_t stored = p[total] | (p[total + 1] << 8);
        return (sum == stored);
    }

    bool eeprom_read_backup(uint8_t* rom_out, uint8_t* conf_out) {
        const uint8_t* p = (const uint8_t*)EEPROM_BACKUP_ADDR;
        if (!eeprom_backup_valid()) return false;
        if (!backup_checksum_valid(p)) {
            Serial.println("[EEPROM] Backup checksum mismatch");
            return false;
        }
        memcpy(rom_out, p + EEPROM_BACKUP_HEADER_SIZE, EEPROM_SIZE);
        memcpy(conf_out, p + EEPROM_BACKUP_HEADER_SIZE + EEPROM_SIZE, EEPROM_SIZE);
        return true;
    }

    // Read identity keys from raw flash backup (v2+)
    // Returns bitmask: bit 0 = transport valid, bit 1 = lxmf valid
    uint8_t identity_read_backup(uint8_t* transport_out, uint8_t* lxmf_out) {
        const uint8_t* p = (const uint8_t*)EEPROM_BACKUP_ADDR;
        if (!eeprom_backup_valid()) return 0;
        if (p[4] < 0x02 || !(p[5] & BACKUP_FLAG_HAS_IDENTITY)) return 0;
        if (!backup_checksum_valid(p)) {
            Serial.println("[IDENTITY] Backup checksum mismatch");
            return 0;
        }

        uint16_t id_offset = EEPROM_BACKUP_HEADER_SIZE + 2 * EEPROM_SIZE;
        if (p[id_offset] != IDENTITY_SECTION_MAGIC_0 || p[id_offset + 1] != IDENTITY_SECTION_MAGIC_1) {
            Serial.println("[IDENTITY] Identity section magic mismatch");
            return 0;
        }

        uint8_t result = 0;
        uint8_t transport_len = p[id_offset + 2];
        if (transport_len == IDENTITY_KEY_SIZE && transport_out) {
            memcpy(transport_out, p + id_offset + 3, IDENTITY_KEY_SIZE);
            result |= 0x01;
        }
        uint8_t lxmf_len = p[id_offset + 3 + IDENTITY_KEY_SIZE];
        if (lxmf_len == IDENTITY_KEY_SIZE && lxmf_out) {
            memcpy(lxmf_out, p + id_offset + 3 + IDENTITY_KEY_SIZE + 1, IDENTITY_KEY_SIZE);
            result |= 0x02;
        }
        return result;
    }

    // Write full backup page: EEPROM data + identity keys (v2 format)
    // If transport_id or lxmf_id is NULL, preserves existing identity from flash
    void write_full_backup(const uint8_t* transport_id, uint8_t transport_len,
                           const uint8_t* lxmf_id, uint8_t lxmf_len) {
        uint16_t total = EEPROM_BACKUP_HEADER_SIZE + 2 * EEPROM_SIZE + IDENTITY_SECTION_SIZE;
        uint8_t buf[total + 2]; // +2 for checksum
        memset(buf, 0xFF, sizeof(buf));

        // Header (v2)
        buf[0] = EEPROM_BACKUP_MAGIC_0;
        buf[1] = EEPROM_BACKUP_MAGIC_1;
        buf[2] = EEPROM_BACKUP_MAGIC_2;
        buf[3] = EEPROM_BACKUP_MAGIC_3;
        buf[4] = 0x02; // version 2
        buf[5] = BACKUP_FLAG_HAS_IDENTITY;
        buf[6] = 0x00;
        buf[7] = 0x00;

        // Read current ROM/CONF data from EEPROM files
        file_rom.seek(0);
        file_rom.read(buf + EEPROM_BACKUP_HEADER_SIZE, EEPROM_SIZE);
        file_conf.seek(0);
        file_conf.read(buf + EEPROM_BACKUP_HEADER_SIZE + EEPROM_SIZE, EEPROM_SIZE);

        // Identity section
        uint16_t id_offset = EEPROM_BACKUP_HEADER_SIZE + 2 * EEPROM_SIZE;
        buf[id_offset] = IDENTITY_SECTION_MAGIC_0;
        buf[id_offset + 1] = IDENTITY_SECTION_MAGIC_1;

        // If caller didn't provide an identity, try to preserve from existing backup
        uint8_t existing_transport[IDENTITY_KEY_SIZE];
        uint8_t existing_lxmf[IDENTITY_KEY_SIZE];
        uint8_t existing_valid = identity_read_backup(existing_transport, existing_lxmf);

        if (transport_id && transport_len == IDENTITY_KEY_SIZE) {
            buf[id_offset + 2] = IDENTITY_KEY_SIZE;
            memcpy(buf + id_offset + 3, transport_id, IDENTITY_KEY_SIZE);
        } else if (existing_valid & 0x01) {
            buf[id_offset + 2] = IDENTITY_KEY_SIZE;
            memcpy(buf + id_offset + 3, existing_transport, IDENTITY_KEY_SIZE);
        } else {
            buf[id_offset + 2] = 0; // no transport identity available
        }

        if (lxmf_id && lxmf_len == IDENTITY_KEY_SIZE) {
            buf[id_offset + 3 + IDENTITY_KEY_SIZE] = IDENTITY_KEY_SIZE;
            memcpy(buf + id_offset + 3 + IDENTITY_KEY_SIZE + 1, lxmf_id, IDENTITY_KEY_SIZE);
        } else if (existing_valid & 0x02) {
            buf[id_offset + 3 + IDENTITY_KEY_SIZE] = IDENTITY_KEY_SIZE;
            memcpy(buf + id_offset + 3 + IDENTITY_KEY_SIZE + 1, existing_lxmf, IDENTITY_KEY_SIZE);
        } else {
            buf[id_offset + 3 + IDENTITY_KEY_SIZE] = 0; // no lxmf identity available
        }

        // Checksum over entire payload
        uint16_t sum = 0;
        for (uint16_t i = 0; i < total; i++) sum += buf[i];
        buf[total] = sum & 0xFF;
        buf[total + 1] = (sum >> 8) & 0xFF;

        // Erase page and write atomically
        flash_nrf5x_erase(EEPROM_BACKUP_ADDR);
        flash_nrf5x_write(EEPROM_BACKUP_ADDR, buf, total + 2);
        flash_nrf5x_flush();
    }

    // Legacy wrapper: write EEPROM backup, preserving any existing identity data
    void eeprom_write_backup() {
        write_full_backup(NULL, 0, NULL, 0);
        Serial.println("[EEPROM] Backup written to raw flash");
    }
#endif
#include <stddef.h>

#if MODEM == SX1262
#include "sx126x.h"
sx126x *LoRa = &sx126x_modem;
#elif MODEM == SX1276 || MODEM == SX1278
#include "sx127x.h"
sx127x *LoRa = &sx127x_modem;
#elif MODEM == SX1280
#include "sx128x.h"
sx128x *LoRa = &sx128x_modem;
#endif

#include "ROM.h"
#include "Framing.h"
#include "MD5.h"

#if !HAS_EEPROM && MCU_VARIANT == MCU_NRF52
uint8_t eeprom_read(uint32_t mapped_addr);
#endif

#if HAS_DISPLAY == true
  #include "Display.h"
#endif

#if HAS_BLUETOOTH == true || HAS_BLE == true
	void kiss_indicate_btpin();
  #include "Bluetooth.h"
#endif

#if HAS_PMU == true
  #include "Power.h"
#endif

#if HAS_INPUT == true
	#include "Input.h"
#endif

#if MCU_VARIANT == MCU_ESP32 || MCU_VARIANT == MCU_NRF52
	#include "Device.h"
#endif
#if MCU_VARIANT == MCU_ESP32
  #if defined(IS_ESP32S3)
    //https://github.com/espressif/esp-idf/issues/8855
    #include "hal/wdt_hal.h"
  #else
	  #include "soc/rtc_wdt.h"
	#endif
  #include "soc/rtc_cntl_reg.h"
  #define ISR_VECT IRAM_ATTR
#else
  #define ISR_VECT
#endif

#if MCU_VARIANT == MCU_1284P || MCU_VARIANT == MCU_2560
	#include <avr/wdt.h>
	#include <util/atomic.h>
#endif

uint8_t boot_vector = 0x00;

#if MCU_VARIANT == MCU_1284P || MCU_VARIANT == MCU_2560
	uint8_t OPTIBOOT_MCUSR __attribute__ ((section(".noinit")));
	void resetFlagsInit(void) __attribute__ ((naked)) __attribute__ ((used)) __attribute__ ((section (".init0")));
	void resetFlagsInit(void) {
	    __asm__ __volatile__ ("sts %0, r2\n" : "=m" (OPTIBOOT_MCUSR) :);
	}
#elif MCU_VARIANT == MCU_ESP32
	// TODO: Get ESP32 boot flags
#elif MCU_VARIANT == MCU_NRF52
	// TODO: Get NRF52 boot flags
#endif

#ifdef HAS_RNS
#include <Reticulum.h>
extern RNS::Reticulum reticulum;
#endif

#if HAS_NP == true
	#include <Adafruit_NeoPixel.h>
	#define NUMPIXELS 1
	#define NP_M 0.15
	Adafruit_NeoPixel pixels(NUMPIXELS, pin_np, NEO_GRB + NEO_KHZ800);

	uint8_t npr = 0;
  uint8_t npg = 0;
  uint8_t npb = 0;
  bool pixels_started = false;
  void npset(uint8_t r, uint8_t g, uint8_t b) {
  	if (pixels_started != true) {
  		pixels.begin();
  		pixels_started = true;
  	}

  	if (r != npr || g != npg || b != npb) {
  		npr = r; npg = g; npb = b;
  		pixels.setPixelColor(0, pixels.Color(npr*NP_M, npg*NP_M, npb*NP_M));
  		pixels.show();
  	}
  }

  void boot_seq() {
  	uint8_t rs[] = { 0x00, 0x00, 0x00 };
  	uint8_t gs[] = { 0x10, 0x08, 0x00 };
  	uint8_t bs[] = { 0x00, 0x08, 0x10 };
  	for (int i = 0; i < 1*sizeof(rs); i++) {
	  	npset(rs[i%sizeof(rs)], gs[i%sizeof(gs)], bs[i%sizeof(bs)]);
	  	delay(33);
	  	npset(0x00, 0x00, 0x00);
	  	delay(66);
  	}
  }
#else
  void boot_seq() { }
#endif

#if MCU_VARIANT == MCU_1284P || MCU_VARIANT == MCU_2560
	void led_rx_on()  { digitalWrite(pin_led_rx, HIGH); }
	void led_rx_off() {	digitalWrite(pin_led_rx, LOW); }
	void led_tx_on()  { digitalWrite(pin_led_tx, HIGH); }
	void led_tx_off() { digitalWrite(pin_led_tx, LOW); }
#elif MCU_VARIANT == MCU_ESP32
	#if HAS_NP == true
		void led_rx_on()  { npset(0, 0, 0xFF); }
		void led_rx_off() {	npset(0, 0, 0); }
		void led_tx_on()  { npset(0xFF, 0x50, 0x00); }
		void led_tx_off() { npset(0, 0, 0); }
	#elif BOARD_MODEL == BOARD_RNODE_NG_20
		void led_rx_on()  { digitalWrite(pin_led_rx, HIGH); }
		void led_rx_off() {	digitalWrite(pin_led_rx, LOW); }
		void led_tx_on()  { digitalWrite(pin_led_tx, HIGH); }
		void led_tx_off() { digitalWrite(pin_led_tx, LOW); }
	#elif BOARD_MODEL == BOARD_RNODE_NG_21
		void led_rx_on()  { digitalWrite(pin_led_rx, HIGH); }
		void led_rx_off() {	digitalWrite(pin_led_rx, LOW); }
		void led_tx_on()  { digitalWrite(pin_led_tx, HIGH); }
		void led_tx_off() { digitalWrite(pin_led_tx, LOW); }
	#elif BOARD_MODEL == BOARD_RNODE_NG_22
		void led_rx_on()  { digitalWrite(pin_led_rx, HIGH); }
		void led_rx_off() {	digitalWrite(pin_led_rx, LOW); }
		void led_tx_on()  { digitalWrite(pin_led_tx, HIGH); }
		void led_tx_off() { digitalWrite(pin_led_tx, LOW); }
	#elif BOARD_MODEL == BOARD_TBEAM
		void led_rx_on()  { digitalWrite(pin_led_rx, HIGH); }
		void led_rx_off() {	digitalWrite(pin_led_rx, LOW); }
		void led_tx_on()  { digitalWrite(pin_led_tx, LOW); }
		void led_tx_off() { digitalWrite(pin_led_tx, HIGH); }
	#elif BOARD_MODEL == BOARD_LORA32_V1_0
		#if defined(EXTERNAL_LEDS)
			void led_rx_on()  { digitalWrite(pin_led_rx, HIGH); }
			void led_rx_off() {	digitalWrite(pin_led_rx, LOW); }
			void led_tx_on()  { digitalWrite(pin_led_tx, HIGH); }
			void led_tx_off() { digitalWrite(pin_led_tx, LOW); }
		#else
			void led_rx_on()  { digitalWrite(pin_led_rx, HIGH); }
			void led_rx_off() {	digitalWrite(pin_led_rx, LOW); }
			void led_tx_on()  { digitalWrite(pin_led_tx, HIGH); }
			void led_tx_off() { digitalWrite(pin_led_tx, LOW); }
		#endif
	#elif BOARD_MODEL == BOARD_LORA32_V2_0
		#if defined(EXTERNAL_LEDS)
			void led_rx_on()  { digitalWrite(pin_led_rx, HIGH); }
			void led_rx_off() {	digitalWrite(pin_led_rx, LOW); }
			void led_tx_on()  { digitalWrite(pin_led_tx, HIGH); }
			void led_tx_off() { digitalWrite(pin_led_tx, LOW); }
		#else
			void led_rx_on()  { digitalWrite(pin_led_rx, LOW); }
			void led_rx_off() {	digitalWrite(pin_led_rx, HIGH); }
			void led_tx_on()  { digitalWrite(pin_led_tx, LOW); }
			void led_tx_off() { digitalWrite(pin_led_tx, HIGH); }
		#endif
	#elif BOARD_MODEL == BOARD_HELTEC32_V2
		#if defined(EXTERNAL_LEDS)
			void led_rx_on()  { digitalWrite(pin_led_rx, HIGH); }
			void led_rx_off() {	digitalWrite(pin_led_rx, LOW); }
			void led_tx_on()  { digitalWrite(pin_led_tx, HIGH); }
			void led_tx_off() { digitalWrite(pin_led_tx, LOW); }
		#else
			void led_rx_on()  { digitalWrite(pin_led_rx, HIGH); }
			void led_rx_off() {	digitalWrite(pin_led_rx, LOW); }
			void led_tx_on()  { digitalWrite(pin_led_tx, HIGH); }
			void led_tx_off() { digitalWrite(pin_led_tx, LOW); }
		#endif
	#elif BOARD_MODEL == BOARD_HELTEC32_V3
			void led_rx_on()  { digitalWrite(pin_led_rx, HIGH); }
			void led_rx_off() {	digitalWrite(pin_led_rx, LOW); }
			void led_tx_on()  { digitalWrite(pin_led_tx, HIGH); }
			void led_tx_off() { digitalWrite(pin_led_tx, LOW); }
	#elif BOARD_MODEL == BOARD_HWSL_V1
			void led_rx_on()  { digitalWrite(pin_led_rx, HIGH); }
			void led_rx_off() {	digitalWrite(pin_led_rx, LOW); }
			void led_tx_on()  { digitalWrite(pin_led_tx, HIGH); }
			void led_tx_off() { digitalWrite(pin_led_tx, LOW); }
	#elif BOARD_MODEL == BOARD_XIAO_ESP32S3
			// XIAO ESP32-S3 user LED (GPIO 21) is active LOW
			void led_rx_on()  { digitalWrite(pin_led_rx, LOW); }
			void led_rx_off() {	digitalWrite(pin_led_rx, HIGH); }
			void led_tx_on()  { digitalWrite(pin_led_tx, LOW); }
			void led_tx_off() { digitalWrite(pin_led_tx, HIGH); }
	#elif BOARD_MODEL == BOARD_LORA32_V2_1
		void led_rx_on()  { digitalWrite(pin_led_rx, HIGH); }
		void led_rx_off() {	digitalWrite(pin_led_rx, LOW); }
		void led_tx_on()  { digitalWrite(pin_led_tx, HIGH); }
		void led_tx_off() { digitalWrite(pin_led_tx, LOW); }
	#elif BOARD_MODEL == BOARD_HUZZAH32
		void led_rx_on()  { digitalWrite(pin_led_rx, HIGH); }
		void led_rx_off() {	digitalWrite(pin_led_rx, LOW); }
		void led_tx_on()  { digitalWrite(pin_led_tx, HIGH); }
		void led_tx_off() { digitalWrite(pin_led_tx, LOW); }
	#elif BOARD_MODEL == BOARD_GENERIC_ESP32
		void led_rx_on()  { digitalWrite(pin_led_rx, HIGH); }
		void led_rx_off() {	digitalWrite(pin_led_rx, LOW); }
		void led_tx_on()  { digitalWrite(pin_led_tx, HIGH); }
		void led_tx_off() { digitalWrite(pin_led_tx, LOW); }
	#endif
#elif MCU_VARIANT == MCU_NRF52
    #if BOARD_MODEL == BOARD_RAK4631 || BOARD_MODEL == BOARD_HELTEC_T114
		void led_rx_on()  { digitalWrite(pin_led_rx, HIGH); }
		void led_rx_off() {	digitalWrite(pin_led_rx, LOW); }
		void led_tx_on()  { digitalWrite(pin_led_tx, HIGH); }
		void led_tx_off() { digitalWrite(pin_led_tx, LOW); }
    #elif BOARD_MODEL == BOARD_XIAO_NRF52840
		// XIAO nRF52840 LEDs are active LOW
		void led_rx_on()  { digitalWrite(pin_led_rx, LOW); }
		void led_rx_off() {	digitalWrite(pin_led_rx, HIGH); }
		void led_tx_on()  { digitalWrite(pin_led_tx, LOW); }
		void led_tx_off() { digitalWrite(pin_led_tx, HIGH); }
    #endif
#endif

void hard_reset(void) {
	#if MCU_VARIANT == MCU_1284P || MCU_VARIANT == MCU_2560
		wdt_enable(WDTO_15MS);
		while(true) {
			led_tx_on(); led_rx_off();
		}
	#elif MCU_VARIANT == MCU_ESP32
		ESP.restart();
	#elif MCU_VARIANT == MCU_NRF52
        NVIC_SystemReset();
	#endif
}

void enter_bootloader(void) {
	#if MCU_VARIANT == MCU_ESP32
		#if defined(CONFIG_IDF_TARGET_ESP32S3)
			// Force next boot into ROM download mode regardless of GPIO0 strap.
			// Needed for XIAO ESP32-S3 + Wio-SX1262 kit where the BOOT button
			// is physically obstructed by the carrier board.
			// RTC_CNTL_OPTION1_REG / FORCE_DOWNLOAD_BOOT exists only on S3.
			REG_WRITE(RTC_CNTL_OPTION1_REG, RTC_CNTL_FORCE_DOWNLOAD_BOOT);
		#endif
		esp_restart();
	#else
		hard_reset();
	#endif
}

// LED Indication: Error
void led_indicate_error(int cycles) {
	#if HAS_NP == true
		bool forever = (cycles == 0) ? true : false;
		cycles = forever ? 1 : cycles;
		while(cycles > 0) {
			npset(0xFF, 0x00, 0x00);
			delay(100);
			npset(0xFF, 0x50, 0x00);
			delay(100);
			if (!forever) cycles--;
		}
		npset(0,0,0);
	#else
		bool forever = (cycles == 0) ? true : false;
		cycles = forever ? 1 : cycles;
		while(cycles > 0) {
	        digitalWrite(pin_led_rx, HIGH);
	        digitalWrite(pin_led_tx, LOW);
	        delay(100);
	        digitalWrite(pin_led_rx, LOW);
	        digitalWrite(pin_led_tx, HIGH);
	        delay(100);
	        if (!forever) cycles--;
	    }
	    led_rx_off();
	    led_tx_off();
	#endif
}

// LED Indication: Airtime Lock
void led_indicate_airtime_lock() {
	#if HAS_NP == true
		npset(32,0,2);
	#endif
}

// LED Indication: Boot Error
void led_indicate_boot_error() {
	#if HAS_NP == true
		while(true) {
			npset(0xFF, 0xFF, 0xFF);
		}
	#else
		while (true) {
		    led_tx_on();
		    led_rx_off();
		    delay(10);
		    led_rx_on();
		    led_tx_off();
		    delay(5);
		}
	#endif
}

// LED Indication: Warning
void led_indicate_warning(int cycles) {
	#if HAS_NP == true
		bool forever = (cycles == 0) ? true : false;
		cycles = forever ? 1 : cycles;
		while(cycles > 0) {
			npset(0xFF, 0x50, 0x00);
			delay(100);
			npset(0x00, 0x00, 0x00);
			delay(100);
			if (!forever) cycles--;
		}
		npset(0,0,0);
	#else
		bool forever = (cycles == 0) ? true : false;
		cycles = forever ? 1 : cycles;
		digitalWrite(pin_led_tx, HIGH);
		while(cycles > 0) {
      led_tx_off();
      delay(100);
      led_tx_on();
      delay(100);
      if (!forever) cycles--;
    }
    led_tx_off();
	#endif
}

// LED Indication: Info
#if MCU_VARIANT == MCU_1284P || MCU_VARIANT == MCU_2560
	void led_indicate_info(int cycles) {
		bool forever = (cycles == 0) ? true : false;
		cycles = forever ? 1 : cycles;
		while(cycles > 0) {
	    led_rx_off();
	    delay(100);
	    led_rx_on();
	    delay(100);
	    if (!forever) cycles--;
	  }
	  led_rx_off();
	}
#elif MCU_VARIANT == MCU_ESP32 || MCU_VARIANT == MCU_NRF52
	#if HAS_NP == true
		void led_indicate_info(int cycles) {
			bool forever = (cycles == 0) ? true : false;
			cycles = forever ? 1 : cycles;
			while(cycles > 0) {
		    npset(0x00, 0x00, 0xFF);
  			delay(100);
  			npset(0x00, 0x00, 0x00);
  			delay(100);
  			if (!forever) cycles--;
		  }
		  npset(0,0,0);
		}
	#elif BOARD_MODEL == BOARD_LORA32_V2_1
		void led_indicate_info(int cycles) {
			bool forever = (cycles == 0) ? true : false;
			cycles = forever ? 1 : cycles;
			while(cycles > 0) {
		    led_rx_off();
		    delay(100);
		    led_rx_on();
		    delay(100);
		    if (!forever) cycles--;
		  }
		  led_rx_off();
		}
	#elif BOARD_MODEL == BOARD_LORA32_V2_0
		void led_indicate_info(int cycles) {
			bool forever = (cycles == 0) ? true : false;
			cycles = forever ? 1 : cycles;
			while(cycles > 0) {
		    led_rx_off();
		    delay(100);
		    led_rx_on();
		    delay(100);
		    if (!forever) cycles--;
		  }
		  led_rx_off();
		}
	#else
		void led_indicate_info(int cycles) {
			bool forever = (cycles == 0) ? true : false;
			cycles = forever ? 1 : cycles;
			while(cycles > 0) {
		    led_tx_off();
		    delay(100);
		    led_tx_on();
		    delay(100);
		    if (!forever) cycles--;
		  }
		  led_tx_off();
		}
	#endif
#endif


unsigned long led_standby_ticks = 0;
#if MCU_VARIANT == MCU_1284P || MCU_VARIANT == MCU_2560
	uint8_t led_standby_min = 1;
	uint8_t led_standby_max = 40;
	unsigned long led_standby_wait = 11000;

#elif MCU_VARIANT == MCU_ESP32

	#if HAS_NP == true
		int led_standby_lng = 100;
		int led_standby_cut = 200;
		int led_standby_min = 0;
		int led_standby_max = 375+led_standby_lng;
		int led_notready_min = 0;
		int led_notready_max = led_standby_max;
		int led_notready_value = led_notready_min;
		int8_t  led_notready_direction = 0;
		unsigned long led_notready_ticks = 0;
		unsigned long led_standby_wait = 350;
		unsigned long led_console_wait = 1;
		unsigned long led_notready_wait = 200;
	
	#else
		uint8_t led_standby_min = 200;
		uint8_t led_standby_max = 255;
		uint8_t led_notready_min = 0;
		uint8_t led_notready_max = 255;
		uint8_t led_notready_value = led_notready_min;
		int8_t  led_notready_direction = 0;
		unsigned long led_notready_ticks = 0;
		unsigned long led_standby_wait = 1768;
		unsigned long led_notready_wait = 150;
	#endif

#elif MCU_VARIANT == MCU_NRF52
		uint8_t led_standby_min = 200;
		uint8_t led_standby_max = 255;
		uint8_t led_notready_min = 0;
		uint8_t led_notready_max = 255;
		uint8_t led_notready_value = led_notready_min;
		int8_t  led_notready_direction = 0;
		unsigned long led_notready_ticks = 0;
		unsigned long led_standby_wait = 1768;
		unsigned long led_notready_wait = 150;
#endif

unsigned long led_standby_value = led_standby_min;
int8_t  led_standby_direction = 0;

#if MCU_VARIANT == MCU_1284P || MCU_VARIANT == MCU_2560
	void led_indicate_standby() {
		led_standby_ticks++;
		if (led_standby_ticks > led_standby_wait) {
			led_standby_ticks = 0;
			if (led_standby_value <= led_standby_min) {
				led_standby_direction = 1;
			} else if (led_standby_value >= led_standby_max) {
				led_standby_direction = -1;
			}
			led_standby_value += led_standby_direction;
			analogWrite(pin_led_rx, led_standby_value);
			led_tx_off();
		}
	}

#elif MCU_VARIANT == MCU_ESP32 || MCU_VARIANT == MCU_NRF52
	#if HAS_NP == true
		void led_indicate_standby() {
			led_standby_ticks++;

			if (led_standby_ticks > led_standby_wait) {
				led_standby_ticks = 0;
				
				if (led_standby_value <= led_standby_min) {
					led_standby_direction = 1;
				} else if (led_standby_value >= led_standby_max) {
					led_standby_direction = -1;
				}

				uint8_t led_standby_intensity;
				led_standby_value += led_standby_direction;
				int led_standby_ti = led_standby_value - led_standby_lng;

				if (led_standby_ti < 0) {
					led_standby_intensity = 0;
				} else if (led_standby_ti > led_standby_cut) {
					led_standby_intensity = led_standby_cut;
				} else {
					led_standby_intensity = led_standby_ti;
				}
  			npset(0x00, 0x00, led_standby_intensity);
			}
		}

		void led_indicate_console() {
			npset(0x60, 0x00, 0x60);
			// led_standby_ticks++;

			// if (led_standby_ticks > led_console_wait) {
			// 	led_standby_ticks = 0;
				
			// 	if (led_standby_value <= led_standby_min) {
			// 		led_standby_direction = 1;
			// 	} else if (led_standby_value >= led_standby_max) {
			// 		led_standby_direction = -1;
			// 	}

			// 	uint8_t led_standby_intensity;
			// 	led_standby_value += led_standby_direction;
			// 	int led_standby_ti = led_standby_value - led_standby_lng;

			// 	if (led_standby_ti < 0) {
			// 		led_standby_intensity = 0;
			// 	} else if (led_standby_ti > led_standby_cut) {
			// 		led_standby_intensity = led_standby_cut;
			// 	} else {
			// 		led_standby_intensity = led_standby_ti;
			// 	}
  	// 		npset(led_standby_intensity, 0x00, led_standby_intensity);
			// }
		}

	#else
		void led_indicate_standby() {
			led_standby_ticks++;
			if (led_standby_ticks > led_standby_wait) {
				led_standby_ticks = 0;
				if (led_standby_value <= led_standby_min) {
					led_standby_direction = 1;
				} else if (led_standby_value >= led_standby_max) {
					led_standby_direction = -1;
				}
				led_standby_value += led_standby_direction;
				if (led_standby_value > 253) {
					led_tx_on();
				} else {
					led_tx_off();
				}
				#if BOARD_MODEL == BOARD_LORA32_V2_1
					#if defined(EXTERNAL_LEDS)
						led_rx_off();
					#endif
				#elif BOARD_MODEL == BOARD_LORA32_V2_0
					#if defined(EXTERNAL_LEDS)
						led_rx_off();
					#endif
				#else
					led_rx_off();
				#endif
			}
		}

		void led_indicate_console() {
			led_indicate_standby();
		}
  #endif
#endif

#if MCU_VARIANT == MCU_1284P || MCU_VARIANT == MCU_2560
	void led_indicate_not_ready() {
		led_standby_ticks++;
		if (led_standby_ticks > led_standby_wait) {
			led_standby_ticks = 0;
			if (led_standby_value <= led_standby_min) {
				led_standby_direction = 1;
			} else if (led_standby_value >= led_standby_max) {
				led_standby_direction = -1;
			}
			led_standby_value += led_standby_direction;
			analogWrite(pin_led_tx, led_standby_value);
			led_rx_off();
		}
	}
#elif MCU_VARIANT == MCU_ESP32 || MCU_VARIANT == MCU_NRF52
	#if HAS_NP == true
    void led_indicate_not_ready() {
    	led_standby_ticks++;

			if (led_standby_ticks > led_notready_wait) {
				led_standby_ticks = 0;
				
				if (led_standby_value <= led_standby_min) {
					led_standby_direction = 1;
				} else if (led_standby_value >= led_standby_max) {
					led_standby_direction = -1;
				}

				uint8_t led_standby_intensity;
				led_standby_value += led_standby_direction;
				int led_standby_ti = led_standby_value - led_standby_lng;

				if (led_standby_ti < 0) {
					led_standby_intensity = 0;
				} else if (led_standby_ti > led_standby_cut) {
					led_standby_intensity = led_standby_cut;
				} else {
					led_standby_intensity = led_standby_ti;
				}

  			npset(led_standby_intensity, 0x00, 0x00);
			}
		}
	#else
		void led_indicate_not_ready() {
			led_notready_ticks++;
			if (led_notready_ticks > led_notready_wait) {
				led_notready_ticks = 0;
				if (led_notready_value <= led_notready_min) {
					led_notready_direction = 1;
				} else if (led_notready_value >= led_notready_max) {
					led_notready_direction = -1;
				}
				led_notready_value += led_notready_direction;
				if (led_notready_value > 128) {
					led_tx_on();
				} else {
					led_tx_off();
				}
				#if BOARD_MODEL == BOARD_LORA32_V2_1
					#if defined(EXTERNAL_LEDS)
						led_rx_off();
					#endif
				#elif BOARD_MODEL == BOARD_LORA32_V2_0
					#if defined(EXTERNAL_LEDS)
						led_rx_off();
					#endif
				#else
					led_rx_off();
				#endif
			}
		}
	#endif
#endif

void serial_write(uint8_t byte) {
	#if HAS_BLUETOOTH || HAS_BLE == true
		if (bt_state != BT_STATE_CONNECTED) {
			Serial.write(byte);
		} else {
			SerialBT.write(byte);
		}
	#else
		Serial.write(byte);
	#endif
}

void escaped_serial_write(uint8_t byte) {
	if (byte == FEND) { serial_write(FESC); byte = TFEND; }
    if (byte == FESC) { serial_write(FESC); byte = TFESC; }
    serial_write(byte);
}

void kiss_indicate_reset() {
	serial_write(FEND);
	serial_write(CMD_RESET);
	serial_write(CMD_RESET_BYTE);
	serial_write(FEND);
}

void kiss_indicate_error(uint8_t error_code) {
	serial_write(FEND);
	serial_write(CMD_ERROR);
	serial_write(error_code);
	serial_write(FEND);
}

void kiss_indicate_radiostate() {
	serial_write(FEND);
	serial_write(CMD_RADIO_STATE);
	serial_write(radio_online);
	serial_write(FEND);
}

void kiss_indicate_stat_rx() {
	serial_write(FEND);
	serial_write(CMD_STAT_RX);
	escaped_serial_write(stat_rx>>24);
	escaped_serial_write(stat_rx>>16);
	escaped_serial_write(stat_rx>>8);
	escaped_serial_write(stat_rx);
	serial_write(FEND);
}

void kiss_indicate_stat_tx() {
	serial_write(FEND);
	serial_write(CMD_STAT_TX);
	escaped_serial_write(stat_tx>>24);
	escaped_serial_write(stat_tx>>16);
	escaped_serial_write(stat_tx>>8);
	escaped_serial_write(stat_tx);
	serial_write(FEND);
}

void kiss_indicate_stat_rssi() {
    uint8_t packet_rssi_val = (uint8_t)(last_rssi+rssi_offset);
	serial_write(FEND);
	serial_write(CMD_STAT_RSSI);
	escaped_serial_write(packet_rssi_val);
	serial_write(FEND);
}

void kiss_indicate_stat_snr() {
	serial_write(FEND);
	serial_write(CMD_STAT_SNR);
	escaped_serial_write(last_snr_raw);
	serial_write(FEND);
}

void kiss_indicate_radio_lock() {
	serial_write(FEND);
	serial_write(CMD_RADIO_LOCK);
	serial_write(radio_locked);
	serial_write(FEND);
}

void kiss_indicate_spreadingfactor() {
	serial_write(FEND);
	serial_write(CMD_SF);
	serial_write((uint8_t)lora_sf);
	serial_write(FEND);
}

void kiss_indicate_codingrate() {
	serial_write(FEND);
	serial_write(CMD_CR);
	serial_write((uint8_t)lora_cr);
	serial_write(FEND);
}

void kiss_indicate_implicit_length() {
	serial_write(FEND);
	serial_write(CMD_IMPLICIT);
	serial_write(implicit_l);
	serial_write(FEND);
}

void kiss_indicate_txpower() {
	serial_write(FEND);
	serial_write(CMD_TXPOWER);
	serial_write((uint8_t)lora_txp);
	serial_write(FEND);
}

void kiss_indicate_bandwidth() {
	serial_write(FEND);
	serial_write(CMD_BANDWIDTH);
	escaped_serial_write(lora_bw>>24);
	escaped_serial_write(lora_bw>>16);
	escaped_serial_write(lora_bw>>8);
	escaped_serial_write(lora_bw);
	serial_write(FEND);
}

void kiss_indicate_frequency() {
	serial_write(FEND);
	serial_write(CMD_FREQUENCY);
	escaped_serial_write(lora_freq>>24);
	escaped_serial_write(lora_freq>>16);
	escaped_serial_write(lora_freq>>8);
	escaped_serial_write(lora_freq);
	serial_write(FEND);
}

void kiss_indicate_st_alock() {
	uint16_t at = (uint16_t)(st_airtime_limit*100*100);
	serial_write(FEND);
	serial_write(CMD_ST_ALOCK);
	escaped_serial_write(at>>8);
	escaped_serial_write(at);
	serial_write(FEND);
}

void kiss_indicate_lt_alock() {
	uint16_t at = (uint16_t)(lt_airtime_limit*100*100);
	serial_write(FEND);
	serial_write(CMD_LT_ALOCK);
	escaped_serial_write(at>>8);
	escaped_serial_write(at);
	serial_write(FEND);
}

void kiss_indicate_channel_stats() {
	#if MCU_VARIANT == MCU_ESP32
		uint16_t ats = (uint16_t)(airtime*100*100);
		uint16_t atl = (uint16_t)(longterm_airtime*100*100);
		uint16_t cls = (uint16_t)(total_channel_util*100*100);
		uint16_t cll = (uint16_t)(longterm_channel_util*100*100);
		serial_write(FEND);
		serial_write(CMD_STAT_CHTM);
		escaped_serial_write(ats>>8);
		escaped_serial_write(ats);
		escaped_serial_write(atl>>8);
		escaped_serial_write(atl);
		escaped_serial_write(cls>>8);
		escaped_serial_write(cls);
		escaped_serial_write(cll>>8);
		escaped_serial_write(cll);
		serial_write(FEND);
	#endif
}

void kiss_indicate_phy_stats() {
	#if MCU_VARIANT == MCU_ESP32
		uint16_t lst = (uint16_t)(lora_symbol_time_ms*1000);
		uint16_t lsr = (uint16_t)(lora_symbol_rate);
		uint16_t prs = (uint16_t)(lora_preamble_symbols+4);
		uint16_t prt = (uint16_t)((lora_preamble_symbols+4)*lora_symbol_time_ms);
		uint16_t cst = (uint16_t)(csma_slot_ms);
		serial_write(FEND);
		serial_write(CMD_STAT_PHYPRM);
		escaped_serial_write(lst>>8);
		escaped_serial_write(lst);
		escaped_serial_write(lsr>>8);
		escaped_serial_write(lsr);
		escaped_serial_write(prs>>8);
		escaped_serial_write(prs);
		escaped_serial_write(prt>>8);
		escaped_serial_write(prt);
		escaped_serial_write(cst>>8);
		escaped_serial_write(cst);
		serial_write(FEND);
	#endif
}

void kiss_indicate_battery() {
	#if MCU_VARIANT == MCU_ESP32
		serial_write(FEND);
		serial_write(CMD_STAT_BAT);
		escaped_serial_write(battery_state);
		escaped_serial_write((uint8_t)int(battery_percent));
		serial_write(FEND);
	#endif
}

void kiss_indicate_btpin() {
	#if HAS_BLUETOOTH || HAS_BLE == true
		serial_write(FEND);
		serial_write(CMD_BT_PIN);
		escaped_serial_write(bt_ssp_pin>>24);
		escaped_serial_write(bt_ssp_pin>>16);
		escaped_serial_write(bt_ssp_pin>>8);
		escaped_serial_write(bt_ssp_pin);
		serial_write(FEND);
	#endif
}

void kiss_indicate_random(uint8_t byte) {
	serial_write(FEND);
	serial_write(CMD_RANDOM);
	serial_write(byte);
	serial_write(FEND);
}

void kiss_indicate_fbstate() {
	serial_write(FEND);
	serial_write(CMD_FB_EXT);
	#if HAS_DISPLAY
		if (disp_ext_fb) {
			serial_write(0x01);
		} else {
			serial_write(0x00);
		}
	#else
		serial_write(0xFF);
	#endif
	serial_write(FEND);
}

#if MCU_VARIANT == MCU_ESP32 || MCU_VARIANT == MCU_NRF52
	void kiss_indicate_device_hash() {
	  serial_write(FEND);
	  serial_write(CMD_DEV_HASH);
	  for (int i = 0; i < DEV_HASH_LEN; i++) {
	    uint8_t byte = dev_hash[i];
	 		escaped_serial_write(byte);
	  }
	  serial_write(FEND);
	}

	void kiss_indicate_target_fw_hash() {
	  serial_write(FEND);
	  serial_write(CMD_HASHES);
	  serial_write(0x01);
	  for (int i = 0; i < DEV_HASH_LEN; i++) {
	    uint8_t byte = dev_firmware_hash_target[i];
	 		escaped_serial_write(byte);
	  }
	  serial_write(FEND);
	}

	void kiss_indicate_fw_hash() {
	  serial_write(FEND);
	  serial_write(CMD_HASHES);
	  serial_write(0x02);
	  for (int i = 0; i < DEV_HASH_LEN; i++) {
	    uint8_t byte = dev_firmware_hash[i];
	 		escaped_serial_write(byte);
	  }
	  serial_write(FEND);
	}

	void kiss_indicate_bootloader_hash() {
	  serial_write(FEND);
	  serial_write(CMD_HASHES);
	  serial_write(0x03);
	  for (int i = 0; i < DEV_HASH_LEN; i++) {
	    uint8_t byte = dev_bootloader_hash[i];
	 		escaped_serial_write(byte);
	  }
	  serial_write(FEND);
	}

	void kiss_indicate_partition_table_hash() {
	  serial_write(FEND);
	  serial_write(CMD_HASHES);
	  serial_write(0x04);
	  for (int i = 0; i < DEV_HASH_LEN; i++) {
	    uint8_t byte = dev_partition_table_hash[i];
	 		escaped_serial_write(byte);
	  }
	  serial_write(FEND);
	}
#endif

void kiss_indicate_fb() {
	serial_write(FEND);
	serial_write(CMD_FB_READ);
	#if HAS_DISPLAY
		for (int i = 0; i < 512; i++) {
			uint8_t byte = fb[i];
			escaped_serial_write(byte);
		}
	#else
		serial_write(0xFF);
	#endif
	serial_write(FEND);
}

void kiss_indicate_ready() {
	serial_write(FEND);
	serial_write(CMD_READY);
	serial_write(0x01);
	serial_write(FEND);
}

void kiss_indicate_not_ready() {
	serial_write(FEND);
	serial_write(CMD_READY);
	serial_write(0x00);
	serial_write(FEND);
}

void kiss_indicate_promisc() {
	serial_write(FEND);
	serial_write(CMD_PROMISC);
	if (promisc) {
		serial_write(0x01);
	} else {
		serial_write(0x00);
	}
	serial_write(FEND);
}

void kiss_indicate_detect() {
	serial_write(FEND);
	serial_write(CMD_DETECT);
	serial_write(DETECT_RESP);
	serial_write(FEND);
}

void kiss_indicate_version() {
	serial_write(FEND);
	serial_write(CMD_FW_VERSION);
	serial_write(MAJ_VERS);
	serial_write(MIN_VERS);
	serial_write(FEND);
}

void kiss_indicate_platform() {
	serial_write(FEND);
	serial_write(CMD_PLATFORM);
	serial_write(PLATFORM);
	serial_write(FEND);
}

void kiss_indicate_board() {
	serial_write(FEND);
	serial_write(CMD_BOARD);
	serial_write(BOARD_MODEL);
	serial_write(FEND);
}

void kiss_indicate_mcu() {
	serial_write(FEND);
	serial_write(CMD_MCU);
	serial_write(MCU_VARIANT);
	serial_write(FEND);
}

inline bool isSplitPacket(uint8_t header) {
	return (header & FLAG_SPLIT);
}

inline uint8_t packetSequence(uint8_t header) {
	return header >> 4;
}

void setPreamble() {
	if (radio_online) LoRa->setPreambleLength(lora_preamble_symbols);
	kiss_indicate_phy_stats();
}

void updateBitrate() {
	#if MCU_VARIANT == MCU_ESP32 || MCU_VARIANT == MCU_NRF52
		if (radio_online) {
			lora_symbol_rate = (float)lora_bw/(float)(pow(2, lora_sf));
			lora_symbol_time_ms = (1.0/lora_symbol_rate)*1000.0;
			lora_bitrate = (uint32_t)(lora_sf * ( (4.0/(float)lora_cr) / ((float)(pow(2, lora_sf))/((float)lora_bw/1000.0)) ) * 1000.0);
			lora_us_per_byte = 1000000.0/((float)lora_bitrate/8.0);
			// csma_slot_ms = lora_symbol_time_ms*10;
			float target_preamble_symbols = (LORA_PREAMBLE_TARGET_MS/lora_symbol_time_ms)-LORA_PREAMBLE_SYMBOLS_HW;
			if (target_preamble_symbols < LORA_PREAMBLE_SYMBOLS_MIN) {
				target_preamble_symbols = LORA_PREAMBLE_SYMBOLS_MIN;
			} else {
				target_preamble_symbols = ceil(target_preamble_symbols);
			}
			lora_preamble_symbols = (long)target_preamble_symbols;
			setPreamble();
		} else {
			lora_bitrate = 0;
		}
	#endif
}

void setSpreadingFactor() {
	if (radio_online) LoRa->setSpreadingFactor(lora_sf);
	updateBitrate();
}

void setCodingRate() {
	if (radio_online) LoRa->setCodingRate4(lora_cr);
	updateBitrate();
}

void set_implicit_length(uint8_t len) {
	implicit_l = len;
	if (implicit_l != 0) {
		implicit = true;
	} else {
		implicit = false;
	}
}

int getTxPower() {
	uint8_t txp = LoRa->getTxPower();
	return (int)txp;
}

void setTXPower() {
	if (radio_online) {
		if (model == MODEL_11) LoRa->setTxPower(lora_txp, PA_OUTPUT_RFO_PIN);
		if (model == MODEL_12) LoRa->setTxPower(lora_txp, PA_OUTPUT_RFO_PIN);

		if (model == MODEL_A1) LoRa->setTxPower(lora_txp, PA_OUTPUT_PA_BOOST_PIN);
		if (model == MODEL_A2) LoRa->setTxPower(lora_txp, PA_OUTPUT_PA_BOOST_PIN);
		if (model == MODEL_A3) LoRa->setTxPower(lora_txp, PA_OUTPUT_RFO_PIN);
		if (model == MODEL_A4) LoRa->setTxPower(lora_txp, PA_OUTPUT_RFO_PIN);
		if (model == MODEL_A6) LoRa->setTxPower(lora_txp, PA_OUTPUT_PA_BOOST_PIN);
		if (model == MODEL_A7) LoRa->setTxPower(lora_txp, PA_OUTPUT_PA_BOOST_PIN);
		if (model == MODEL_A8) LoRa->setTxPower(lora_txp, PA_OUTPUT_PA_BOOST_PIN);
		if (model == MODEL_A9) LoRa->setTxPower(lora_txp, PA_OUTPUT_PA_BOOST_PIN);

		if (model == MODEL_B3) LoRa->setTxPower(lora_txp, PA_OUTPUT_PA_BOOST_PIN);
		if (model == MODEL_B4) LoRa->setTxPower(lora_txp, PA_OUTPUT_PA_BOOST_PIN);
		if (model == MODEL_B8) LoRa->setTxPower(lora_txp, PA_OUTPUT_PA_BOOST_PIN);
		if (model == MODEL_B9) LoRa->setTxPower(lora_txp, PA_OUTPUT_PA_BOOST_PIN);

		if (model == MODEL_C4) LoRa->setTxPower(lora_txp, PA_OUTPUT_PA_BOOST_PIN);
		if (model == MODEL_C9) LoRa->setTxPower(lora_txp, PA_OUTPUT_PA_BOOST_PIN);

		if (model == MODEL_CB) LoRa->setTxPower(lora_txp, PA_OUTPUT_PA_BOOST_PIN);
		if (model == MODEL_CC) LoRa->setTxPower(lora_txp, PA_OUTPUT_PA_BOOST_PIN);

		if (model == MODEL_E4) LoRa->setTxPower(lora_txp, PA_OUTPUT_PA_BOOST_PIN);
		if (model == MODEL_E9) LoRa->setTxPower(lora_txp, PA_OUTPUT_PA_BOOST_PIN);
		if (model == MODEL_E3) LoRa->setTxPower(lora_txp, PA_OUTPUT_PA_BOOST_PIN);
		if (model == MODEL_E8) LoRa->setTxPower(lora_txp, PA_OUTPUT_PA_BOOST_PIN);

		if (model == MODEL_FE) LoRa->setTxPower(lora_txp, PA_OUTPUT_PA_BOOST_PIN);
		if (model == MODEL_FF) LoRa->setTxPower(lora_txp, PA_OUTPUT_RFO_PIN);
	}
}


void getBandwidth() {
	if (radio_online) {
			lora_bw = LoRa->getSignalBandwidth();
	}
	updateBitrate();
}

void setBandwidth() {
	if (radio_online) {
		LoRa->setSignalBandwidth(lora_bw);
		getBandwidth();
	}
}

void getFrequency() {
	if (radio_online) {
		lora_freq = LoRa->getFrequency();
	}
}

void setFrequency() {
	if (radio_online) {
		LoRa->setFrequency(lora_freq);
		getFrequency();
	}
}

uint8_t getRandom() {
	if (radio_online) {
		return LoRa->random();
	} else {
		return 0x00;
	}
}

void promisc_enable() {
	promisc = true;
}

void promisc_disable() {
	promisc = false;
}

#if !HAS_EEPROM && MCU_VARIANT == MCU_NRF52
    // Returns true if the logical address falls in the config region
    bool addr_is_conf(int mapped_addr) {
        int logical = mapped_addr - eeprom_addr(0);
        return (logical >= ADDR_CONF_SF && logical <= ADDR_CONF_DADR);
    }

    // Open or create a file with initial data, returns true on success
    bool eeprom_open_or_create(File &f, const char* name, uint8_t* init_data) {
        if (InternalFS.exists(name)) {
            if (f.open(name, FILE_O_WRITE)) {
                if (f.size() == EEPROM_SIZE) return true;
                f.close();
                InternalFS.remove(name);
            }
        }
        if (!f.open(name, FILE_O_WRITE)) return false;
        if (f.write(init_data, EEPROM_SIZE) < EEPROM_SIZE) {
            f.close();
            return false;
        }
        f.flush();
        return true;
    }

    // Restore eeprom_conf from eeprom_defaults
    bool eeprom_restore_conf_from_defaults() {
        uint8_t buf[EEPROM_SIZE];
        file_defaults.seek(0);
        if (file_defaults.read(buf, EEPROM_SIZE) != EEPROM_SIZE) {
            // Defaults unreadable, use empty
            memset(buf, 0, EEPROM_SIZE);
        }
        if (InternalFS.exists(EEPROM_FILE_CONF)) {
            file_conf.close();
            InternalFS.remove(EEPROM_FILE_CONF);
        }
        return eeprom_open_or_create(file_conf, EEPROM_FILE_CONF, buf);
    }

    // Open all EEPROM-backed files. Returns false on any failure; caller is
    // responsible for closing handles before retrying (see eeprom_begin).
    // Extracted from eeprom_begin so the whole open path can be retried after
    // an InternalFS.format() when the FS is corrupt past auto-recovery.
    bool eeprom_open_files() {
        // Migration: legacy single "eeprom" file exists
        if (InternalFS.exists(EEPROM_FILE_LEGACY)) {
            File file_legacy(InternalFS);
            if (!file_legacy.open(EEPROM_FILE_LEGACY, FILE_O_READ)) return false;

            uint8_t buffer[EEPROM_SIZE];
            memset(buffer, 0, EEPROM_SIZE);
            file_legacy.read(buffer, EEPROM_SIZE);
            file_legacy.close();

            if (!eeprom_open_or_create(file_rom, EEPROM_FILE_ROM, buffer)) return false;
            if (!eeprom_open_or_create(file_conf, EEPROM_FILE_CONF, buffer)) return false;
            if (!eeprom_open_or_create(file_defaults, EEPROM_FILE_DEFAULTS, buffer)) return false;

            // Only remove legacy after all three are safely written
            InternalFS.remove(EEPROM_FILE_LEGACY);
            Serial.println("[EEPROM] Migrated legacy file to split files");
            return true;
        }

        // Check if files are missing (e.g. after LittleFS auto-format due to corruption)
        bool rom_missing = !InternalFS.exists(EEPROM_FILE_ROM);
        bool conf_missing = !InternalFS.exists(EEPROM_FILE_CONF);

        if ((rom_missing || conf_missing) && eeprom_backup_valid()) {
            // Files lost (likely due to LittleFS corruption + auto-format)
            // Restore from raw flash backup
            uint8_t rom_data[EEPROM_SIZE];
            uint8_t conf_data[EEPROM_SIZE];

            if (eeprom_read_backup(rom_data, conf_data)) {
                Serial.println("[EEPROM] Restoring from raw flash backup");

                if (!eeprom_open_or_create(file_rom, EEPROM_FILE_ROM, rom_data)) return false;
                if (!eeprom_open_or_create(file_conf, EEPROM_FILE_CONF, conf_data)) return false;
                if (!eeprom_open_or_create(file_defaults, EEPROM_FILE_DEFAULTS, conf_data)) return false;

                Serial.println("[EEPROM] Recovery from backup complete");
                return true;
            } else {
                Serial.println("[EEPROM] Backup read failed, starting fresh");
            }
        }

        // Normal boot: open split files
        uint8_t empty[EEPROM_SIZE] = {0};

        // ROM file
        if (InternalFS.exists(EEPROM_FILE_ROM)) {
            if (!file_rom.open(EEPROM_FILE_ROM, FILE_O_WRITE)) return false;
            if (file_rom.size() != EEPROM_SIZE) {
                file_rom.close();
                InternalFS.remove(EEPROM_FILE_ROM);
                if (!eeprom_open_or_create(file_rom, EEPROM_FILE_ROM, empty)) return false;
            }
        } else {
            if (!eeprom_open_or_create(file_rom, EEPROM_FILE_ROM, empty)) return false;
        }

        // Defaults file
        if (InternalFS.exists(EEPROM_FILE_DEFAULTS)) {
            if (!file_defaults.open(EEPROM_FILE_DEFAULTS, FILE_O_WRITE)) return false;
            if (file_defaults.size() != EEPROM_SIZE) {
                file_defaults.close();
                InternalFS.remove(EEPROM_FILE_DEFAULTS);
                if (!eeprom_open_or_create(file_defaults, EEPROM_FILE_DEFAULTS, empty)) return false;
            }
        } else {
            if (!eeprom_open_or_create(file_defaults, EEPROM_FILE_DEFAULTS, empty)) return false;
        }

        // Config file - restore from defaults if missing or corrupt
        if (InternalFS.exists(EEPROM_FILE_CONF)) {
            if (!file_conf.open(EEPROM_FILE_CONF, FILE_O_WRITE)) {
                return eeprom_restore_conf_from_defaults();
            }
            if (file_conf.size() != EEPROM_SIZE) {
                Serial.println("[EEPROM] Config file corrupt, restoring from defaults");
                return eeprom_restore_conf_from_defaults();
            }
        } else {
            Serial.println("[EEPROM] Config file missing, restoring from defaults");
            return eeprom_restore_conf_from_defaults();
        }

        return true;
    }

    bool eeprom_begin() {
        // Mount, with format-and-retry if a brownout-corrupted superblock
        // prevents mount entirely.
        if (!InternalFS.begin()) {
            Serial.println("[EEPROM] InternalFS mount failed, forcing format..."); Serial.flush();
            InternalFS.format();
            if (!InternalFS.begin()) {
                Serial.println("[EEPROM] InternalFS unrecoverable after format"); Serial.flush();
                return false;
            }
            Serial.println("[EEPROM] InternalFS recovered after format"); Serial.flush();
        }

        // Try the normal open path. If it fails — typically because LFS
        // returned LFS_ERR_CORRUPT mid-traversal on a metadata block that's
        // damaged below the superblock level — close any partial handles,
        // format, and retry. Without this, the partially-open handles get
        // touched by validate_status()/eeprom_read() and trip LFS again.
        if (!eeprom_open_files()) {
            Serial.println("[EEPROM] File operations failed (corrupt FS?), forcing format..."); Serial.flush();
            file_rom.close();
            file_conf.close();
            file_defaults.close();
            InternalFS.format();
            if (!InternalFS.begin()) {
                Serial.println("[EEPROM] Re-mount failed after format"); Serial.flush();
                return false;
            }
            if (!eeprom_open_files()) {
                // Make damn sure no half-open handles survive on the failure path.
                file_rom.close();
                file_conf.close();
                file_defaults.close();
                Serial.println("[EEPROM] File operations still failing after format"); Serial.flush();
                return false;
            }
            Serial.println("[EEPROM] Recovered files after format"); Serial.flush();
        }

        return true;
    }

    uint8_t eeprom_read(uint32_t mapped_addr) {
        uint8_t byte = 0xFF;
        if (addr_is_conf(mapped_addr)) {
            file_conf.seek(mapped_addr);
            file_conf.read(&byte, 1);
        } else {
            file_rom.seek(mapped_addr);
            file_rom.read(&byte, 1);
        }
        return byte;
    }
#endif

bool eeprom_info_locked() {
    #if HAS_EEPROM
	    uint8_t lock_byte = EEPROM.read(eeprom_addr(ADDR_INFO_LOCK));
    #elif MCU_VARIANT == MCU_NRF52
        uint8_t lock_byte = eeprom_read(eeprom_addr(ADDR_INFO_LOCK));
    #endif
	if (lock_byte == INFO_LOCK_BYTE) {
		return true;
	} else {
		return false;
	}
}

void eeprom_dump_info() {
	for (int addr = ADDR_PRODUCT; addr <= ADDR_INFO_LOCK; addr++) {
        #if HAS_EEPROM
            uint8_t byte = EEPROM.read(eeprom_addr(addr));
        #elif MCU_VARIANT == MCU_NRF52
            uint8_t byte = eeprom_read(eeprom_addr(addr));
        #endif
		escaped_serial_write(byte);
	}
}

void eeprom_dump_config() {
	for (int addr = ADDR_CONF_SF; addr <= ADDR_CONF_OK; addr++) {
        #if HAS_EEPROM
            uint8_t byte = EEPROM.read(eeprom_addr(addr));
        #elif MCU_VARIANT == MCU_NRF52
            uint8_t byte = eeprom_read(eeprom_addr(addr));
        #endif
		escaped_serial_write(byte);
	}
}

void eeprom_dump_all() {
	for (int addr = 0; addr < EEPROM_RESERVED; addr++) {
        #if HAS_EEPROM
            uint8_t byte = EEPROM.read(eeprom_addr(addr));
        #elif MCU_VARIANT == MCU_NRF52
            uint8_t byte = eeprom_read(eeprom_addr(addr));
        #endif
		escaped_serial_write(byte);
	}
}

void kiss_dump_eeprom() {
	serial_write(FEND);
	serial_write(CMD_ROM_READ);
	eeprom_dump_all();
	serial_write(FEND);
}

#if !HAS_EEPROM && MCU_VARIANT == MCU_NRF52
void eeprom_flush_rom() {
    file_rom.close();
    file_rom.open(EEPROM_FILE_ROM, FILE_O_WRITE);
    written_bytes_rom = 0;
    eeprom_write_backup();
}

void eeprom_flush_conf() {
    file_conf.close();
    file_conf.open(EEPROM_FILE_CONF, FILE_O_WRITE);
    written_bytes_conf = 0;
    eeprom_write_backup();
}

void eeprom_flush() {
    eeprom_flush_rom();
    eeprom_flush_conf();
}
#endif

void eeprom_update(int mapped_addr, uint8_t byte) {
	#if MCU_VARIANT == MCU_1284P || MCU_VARIANT == MCU_2560
		EEPROM.update(mapped_addr, byte);
	#elif MCU_VARIANT == MCU_ESP32
		if (EEPROM.read(mapped_addr) != byte) {
			EEPROM.write(mapped_addr, byte);
			EEPROM.commit();
		}
    #elif !HAS_EEPROM && MCU_VARIANT == MCU_NRF52
        bool is_conf = addr_is_conf(mapped_addr);
        File &target = is_conf ? file_conf : file_rom;
        int &written = is_conf ? written_bytes_conf : written_bytes_rom;
        const char* fname = is_conf ? EEPROM_FILE_CONF : EEPROM_FILE_ROM;

        uint8_t read_byte;
        target.seek(mapped_addr);
        target.read(&read_byte, 1);
        target.seek(mapped_addr);
        if (read_byte != byte) {
            target.write(byte);
        }
        written++;

        int logical = mapped_addr - eeprom_addr(0);
        if (logical == ADDR_INFO_LOCK) {
            eeprom_flush_rom();
        }
        else if (logical == ADDR_CONF_OK) {
            eeprom_flush_conf();
        }

        if (written >= 4) {
            target.close();
            target.open(fname, FILE_O_WRITE);
            written = 0;
        }
	#endif
}

void eeprom_write(uint8_t addr, uint8_t byte) {
	if (!eeprom_info_locked() && addr >= 0 && addr < EEPROM_RESERVED) {
		eeprom_update(eeprom_addr(addr), byte);
	} else {
		kiss_indicate_error(ERROR_EEPROM_LOCKED);
	}
}

void eeprom_erase() {
	for (int addr = 0; addr < EEPROM_RESERVED; addr++) {
		eeprom_update(eeprom_addr(addr), 0xFF);
	}
	#if !HAS_EEPROM && MCU_VARIANT == MCU_NRF52
		eeprom_flush();
		// Also clear defaults
		file_defaults.close();
		InternalFS.remove(EEPROM_FILE_DEFAULTS);
		uint8_t empty[EEPROM_SIZE] = {0};
		eeprom_open_or_create(file_defaults, EEPROM_FILE_DEFAULTS, empty);
	#endif
	#ifdef HAS_RNS
		reticulum.clear_caches();
	#endif
	hard_reset();
}

bool eeprom_lock_set() {
    #if HAS_EEPROM
	    if (EEPROM.read(eeprom_addr(ADDR_INFO_LOCK)) == INFO_LOCK_BYTE) {
    #elif MCU_VARIANT == MCU_NRF52
        if (eeprom_read(eeprom_addr(ADDR_INFO_LOCK)) == INFO_LOCK_BYTE) {
    #endif
		return true;
	} else {
		return false;
	}
}

bool eeprom_product_valid() {
    #if HAS_EEPROM
	    uint8_t rval = EEPROM.read(eeprom_addr(ADDR_PRODUCT));
    #elif MCU_VARIANT == MCU_NRF52
	    uint8_t rval = eeprom_read(eeprom_addr(ADDR_PRODUCT));
    #endif

	#if PLATFORM == PLATFORM_AVR
	if (rval == PRODUCT_RNODE || rval == PRODUCT_HMBRW) {
	#elif PLATFORM == PLATFORM_ESP32
	if (rval == PRODUCT_RNODE || rval == BOARD_RNODE_NG_20 || rval == BOARD_RNODE_NG_21 || rval == PRODUCT_HMBRW || rval == PRODUCT_TBEAM || rval == PRODUCT_T32_10 || rval == PRODUCT_T32_20 || rval == PRODUCT_T32_21 || rval == PRODUCT_H32_V2 || rval == PRODUCT_H32_V3 || rval == PRODUCT_HWSL_V1 || rval == PRODUCT_XIAO_ESP32S3) {
	#elif PLATFORM == PLATFORM_NRF52
	if (rval == PRODUCT_RAK4631 || rval == PRODUCT_XIAO_NRF52840 || rval == PRODUCT_HMBRW) {
	#else
	if (false) {
	#endif
		return true;
	} else {
		return false;
	}
}

bool eeprom_model_valid() {
    #if HAS_EEPROM
        model = EEPROM.read(eeprom_addr(ADDR_MODEL));
    #elif MCU_VARIANT == MCU_NRF52
        model = eeprom_read(eeprom_addr(ADDR_MODEL));
    #endif
	#if BOARD_MODEL == BOARD_RNODE
	if (model == MODEL_A4 || model == MODEL_A9 || model == MODEL_FF || model == MODEL_FE) {
	#elif BOARD_MODEL == BOARD_RNODE_NG_20
	if (model == MODEL_A3 || model == MODEL_A8) {
	#elif BOARD_MODEL == BOARD_RNODE_NG_21
	if (model == MODEL_A2 || model == MODEL_A7) {
	#elif BOARD_MODEL == BOARD_RNODE_NG_22
	if (model == MODEL_A1 || model == MODEL_A6) {
	#elif BOARD_MODEL == BOARD_HMBRW
	if (model == MODEL_FF || model == MODEL_FE) {
	#elif BOARD_MODEL == BOARD_TBEAM
	if (model == MODEL_E4 || model == MODEL_E9 || model == MODEL_E3 || model == MODEL_E8) {
	#elif BOARD_MODEL == BOARD_LORA32_V1_0
	if (model == MODEL_BA || model == MODEL_BB) {
	#elif BOARD_MODEL == BOARD_LORA32_V2_0
	if (model == MODEL_B3 || model == MODEL_B8) {
	#elif BOARD_MODEL == BOARD_LORA32_V2_1
	if (model == MODEL_B4 || model == MODEL_B9) {
	#elif BOARD_MODEL == BOARD_HELTEC32_V2
	if (model == MODEL_C4 || model == MODEL_C9) {
	#elif BOARD_MODEL == BOARD_HELTEC32_V3
	if (model == MODEL_C5 || model == MODEL_CA) {
	#elif BOARD_MODEL == BOARD_HWSL_V1
	if (model == MODEL_CB || model == MODEL_CC) {
    #elif BOARD_MODEL == BOARD_RAK4631 || BOARD_MODEL == BOARD_HELTEC_T114
    if (model == MODEL_11 || model == MODEL_12) {
    #elif BOARD_MODEL == BOARD_XIAO_NRF52840
    if (model == MODEL_11 || model == MODEL_12) {
	#elif BOARD_MODEL == BOARD_XIAO_ESP32S3
    if (model == MODEL_13) {
	#elif BOARD_MODEL == BOARD_HUZZAH32
	if (model == MODEL_FF) {
	#elif BOARD_MODEL == BOARD_GENERIC_ESP32
	if (model == MODEL_FF || model == MODEL_FE) {
	#else
	if (false) {
	#endif
		return true;
	} else {
		return false;
	}
}

bool eeprom_hwrev_valid() {
    #if HAS_EEPROM
        hwrev = EEPROM.read(eeprom_addr(ADDR_HW_REV));
    #elif MCU_VARIANT == MCU_NRF52
        hwrev = eeprom_read(eeprom_addr(ADDR_HW_REV));
    #endif
	if (hwrev != 0x00 && hwrev != 0xFF) {
		return true;
	} else {
		return false;
	}
}

bool eeprom_checksum_valid() {
	char *data = (char*)malloc(CHECKSUMMED_SIZE);
	for (uint8_t  i = 0; i < CHECKSUMMED_SIZE; i++) {
        #if HAS_EEPROM
            char byte = EEPROM.read(eeprom_addr(i));
        #elif MCU_VARIANT == MCU_NRF52
            char byte = eeprom_read(eeprom_addr(i));
        #endif
		data[i] = byte;
	}
	
	unsigned char *hash = MD5::make_hash(data, CHECKSUMMED_SIZE);
	bool checksum_valid = true;
	for (uint8_t i = 0; i < 16; i++) {
        #if HAS_EEPROM
            uint8_t stored_chk_byte = EEPROM.read(eeprom_addr(ADDR_CHKSUM+i));
        #elif MCU_VARIANT == MCU_NRF52
            uint8_t stored_chk_byte = eeprom_read(eeprom_addr(ADDR_CHKSUM+i));
        #endif
		uint8_t calced_chk_byte = (uint8_t)hash[i];
		if (stored_chk_byte != calced_chk_byte) {
			checksum_valid = false;
		}
	}

	free(hash);
	free(data);
	return checksum_valid;
}

void bt_conf_save(bool is_enabled) {
	if (is_enabled) {
		eeprom_update(eeprom_addr(ADDR_CONF_BT), BT_ENABLE_BYTE);
        #if !HAS_EEPROM && MCU_VARIANT == MCU_NRF52
            eeprom_flush_conf();
        #endif
	} else {
		eeprom_update(eeprom_addr(ADDR_CONF_BT), 0x00);
        #if !HAS_EEPROM && MCU_VARIANT == MCU_NRF52
            eeprom_flush_conf();
        #endif
	}
}

void di_conf_save(uint8_t dint) {
	eeprom_update(eeprom_addr(ADDR_CONF_DINT), dint);
	#if !HAS_EEPROM && MCU_VARIANT == MCU_NRF52
		eeprom_flush_conf();
	#endif
}

void da_conf_save(uint8_t dadr) {
	eeprom_update(eeprom_addr(ADDR_CONF_DADR), dadr);
	#if !HAS_EEPROM && MCU_VARIANT == MCU_NRF52
		eeprom_flush_conf();
	#endif
}

bool eeprom_have_conf() {
    #if HAS_EEPROM
	    if (EEPROM.read(eeprom_addr(ADDR_CONF_OK)) == CONF_OK_BYTE) {
    #elif MCU_VARIANT == MCU_NRF52
        if (eeprom_read(eeprom_addr(ADDR_CONF_OK)) == CONF_OK_BYTE) {
    #endif
		return true;
	} else {
		return false;
	}
}

void eeprom_conf_load() {
	if (eeprom_have_conf()) {
        #if HAS_EEPROM
            lora_sf = EEPROM.read(eeprom_addr(ADDR_CONF_SF));
            lora_cr = EEPROM.read(eeprom_addr(ADDR_CONF_CR));
            lora_txp = EEPROM.read(eeprom_addr(ADDR_CONF_TXP));
            lora_freq = (uint32_t)EEPROM.read(eeprom_addr(ADDR_CONF_FREQ)+0x00) << 24 | (uint32_t)EEPROM.read(eeprom_addr(ADDR_CONF_FREQ)+0x01) << 16 | (uint32_t)EEPROM.read(eeprom_addr(ADDR_CONF_FREQ)+0x02) << 8 | (uint32_t)EEPROM.read(eeprom_addr(ADDR_CONF_FREQ)+0x03);
            lora_bw = (uint32_t)EEPROM.read(eeprom_addr(ADDR_CONF_BW)+0x00) << 24 | (uint32_t)EEPROM.read(eeprom_addr(ADDR_CONF_BW)+0x01) << 16 | (uint32_t)EEPROM.read(eeprom_addr(ADDR_CONF_BW)+0x02) << 8 | (uint32_t)EEPROM.read(eeprom_addr(ADDR_CONF_BW)+0x03);
        #elif MCU_VARIANT == MCU_NRF52
            lora_sf = eeprom_read(eeprom_addr(ADDR_CONF_SF));
            lora_cr = eeprom_read(eeprom_addr(ADDR_CONF_CR));
            lora_txp = eeprom_read(eeprom_addr(ADDR_CONF_TXP));
            lora_freq = (uint32_t)eeprom_read(eeprom_addr(ADDR_CONF_FREQ)+0x00) << 24 | (uint32_t)eeprom_read(eeprom_addr(ADDR_CONF_FREQ)+0x01) << 16 | (uint32_t)eeprom_read(eeprom_addr(ADDR_CONF_FREQ)+0x02) << 8 | (uint32_t)eeprom_read(eeprom_addr(ADDR_CONF_FREQ)+0x03);
            lora_bw = (uint32_t)eeprom_read(eeprom_addr(ADDR_CONF_BW)+0x00) << 24 | (uint32_t)eeprom_read(eeprom_addr(ADDR_CONF_BW)+0x01) << 16 | (uint32_t)eeprom_read(eeprom_addr(ADDR_CONF_BW)+0x02) << 8 | (uint32_t)eeprom_read(eeprom_addr(ADDR_CONF_BW)+0x03);
        #endif
	}
}

void eeprom_conf_save() {
	if (hw_ready && radio_online) {
		eeprom_update(eeprom_addr(ADDR_CONF_SF), lora_sf);
		eeprom_update(eeprom_addr(ADDR_CONF_CR), lora_cr);
		eeprom_update(eeprom_addr(ADDR_CONF_TXP), lora_txp);

		eeprom_update(eeprom_addr(ADDR_CONF_BW)+0x00, lora_bw>>24);
		eeprom_update(eeprom_addr(ADDR_CONF_BW)+0x01, lora_bw>>16);
		eeprom_update(eeprom_addr(ADDR_CONF_BW)+0x02, lora_bw>>8);
		eeprom_update(eeprom_addr(ADDR_CONF_BW)+0x03, lora_bw);

		eeprom_update(eeprom_addr(ADDR_CONF_FREQ)+0x00, lora_freq>>24);
		eeprom_update(eeprom_addr(ADDR_CONF_FREQ)+0x01, lora_freq>>16);
		eeprom_update(eeprom_addr(ADDR_CONF_FREQ)+0x02, lora_freq>>8);
		eeprom_update(eeprom_addr(ADDR_CONF_FREQ)+0x03, lora_freq);

		eeprom_update(eeprom_addr(ADDR_CONF_OK), CONF_OK_BYTE);

		#if !HAS_EEPROM && MCU_VARIANT == MCU_NRF52
			eeprom_flush_conf();
			// Snapshot config as defaults (for power-loss recovery)
			uint8_t conf_buf[EEPROM_SIZE];
			file_conf.seek(0);
			file_conf.read(conf_buf, EEPROM_SIZE);
			file_defaults.seek(0);
			file_defaults.write(conf_buf, EEPROM_SIZE);
			file_defaults.close();
			file_defaults.open(EEPROM_FILE_DEFAULTS, FILE_O_WRITE);
		#endif

		led_indicate_info(10);
	} else {
		led_indicate_warning(10);
	}
}

void eeprom_conf_delete() {
	eeprom_update(eeprom_addr(ADDR_CONF_OK), 0x00);
}

void unlock_rom() {
	led_indicate_error(50);
	eeprom_erase();
}

void init_channel_stats() {
	#if MCU_VARIANT == MCU_ESP32
		for (uint16_t ai = 0; ai < DCD_SAMPLES; ai++) { util_samples[ai] = false; }
		for (uint16_t ai = 0; ai < AIRTIME_BINS; ai++) { airtime_bins[ai] = 0; }
		for (uint16_t ai = 0; ai < AIRTIME_BINS; ai++) { longterm_bins[ai] = 0.0; }
		local_channel_util = 0.0;
		total_channel_util = 0.0;
		airtime = 0.0;
		longterm_airtime = 0.0;
	#endif
}

typedef struct FIFOBuffer
{
  unsigned char *begin;
  unsigned char *end;
  unsigned char * volatile head;
  unsigned char * volatile tail;
} FIFOBuffer;

inline bool fifo_isempty(const FIFOBuffer *f) {
  return f->head == f->tail;
}

inline bool fifo_isfull(const FIFOBuffer *f) {
  return ((f->head == f->begin) && (f->tail == f->end)) || (f->tail == f->head - 1);
}

inline void fifo_push(FIFOBuffer *f, unsigned char c) {
  *(f->tail) = c;
  
  if (f->tail == f->end) {
    f->tail = f->begin;
  } else {
    f->tail++;
  }
}

inline unsigned char fifo_pop(FIFOBuffer *f) {
  if(f->head == f->end) {
    f->head = f->begin;
    return *(f->end);
  } else {
    return *(f->head++);
  }
}

inline void fifo_flush(FIFOBuffer *f) {
  f->head = f->tail;
}

#if MCU_VARIANT != MCU_ESP32 && MCU_VARIANT != MCU_NRF52
	static inline bool fifo_isempty_locked(const FIFOBuffer *f) {
	  bool result;
	  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
	    result = fifo_isempty(f);
	  }
	  return result;
	}

	static inline bool fifo_isfull_locked(const FIFOBuffer *f) {
	  bool result;
	  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
	    result = fifo_isfull(f);
	  }
	  return result;
	}

	static inline void fifo_push_locked(FIFOBuffer *f, unsigned char c) {
	  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
	    fifo_push(f, c);
	  }
	}
#endif

/*
static inline unsigned char fifo_pop_locked(FIFOBuffer *f) {
  unsigned char c;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    c = fifo_pop(f);
  }
  return c;
}
*/

inline void fifo_init(FIFOBuffer *f, unsigned char *buffer, size_t size) {
  f->head = f->tail = f->begin = buffer;
  f->end = buffer + size;
}

inline size_t fifo_len(FIFOBuffer *f) {
  return f->end - f->begin;
}

typedef struct FIFOBuffer16
{
  uint16_t *begin;
  uint16_t *end;
  uint16_t * volatile head;
  uint16_t * volatile tail;
} FIFOBuffer16;

inline bool fifo16_isempty(const FIFOBuffer16 *f) {
  return f->head == f->tail;
}

inline bool fifo16_isfull(const FIFOBuffer16 *f) {
  return ((f->head == f->begin) && (f->tail == f->end)) || (f->tail == f->head - 1);
}

inline void fifo16_push(FIFOBuffer16 *f, uint16_t c) {
  *(f->tail) = c;

  if (f->tail == f->end) {
    f->tail = f->begin;
  } else {
    f->tail++;
  }
}

inline uint16_t fifo16_pop(FIFOBuffer16 *f) {
  if(f->head == f->end) {
    f->head = f->begin;
    return *(f->end);
  } else {
    return *(f->head++);
  }
}

inline void fifo16_flush(FIFOBuffer16 *f) {
  f->head = f->tail;
}

#if MCU_VARIANT != MCU_ESP32 && MCU_VARIANT != MCU_NRF52
	static inline bool fifo16_isempty_locked(const FIFOBuffer16 *f) {
	  bool result;
	  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
	    result = fifo16_isempty(f);
	  }

	  return result;
	}
#endif

/*
static inline bool fifo16_isfull_locked(const FIFOBuffer16 *f) {
  bool result;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    result = fifo16_isfull(f);
  }
  return result;
}


static inline void fifo16_push_locked(FIFOBuffer16 *f, uint16_t c) {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    fifo16_push(f, c);
  }
}

static inline size_t fifo16_pop_locked(FIFOBuffer16 *f) {
  size_t c;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    c = fifo16_pop(f);
  }
  return c;
}
*/

inline void fifo16_init(FIFOBuffer16 *f, uint16_t *buffer, uint16_t size) {
  f->head = f->tail = f->begin = buffer;
  f->end = buffer + size;
}

inline uint16_t fifo16_len(FIFOBuffer16 *f) {
  return (f->end - f->begin);
}
