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

// CBA Reticulum includes must come before local to avoid collision with local defines
#ifdef HAS_RNS
#include <Transport.h>
#include <Reticulum.h>
#include <Interface.h>
#include <Log.h>
#include <Bytes.h>
#include <queue>
#endif

// GPIO Control via LXMF messaging
#ifdef HAS_GPIO_CONTROL
#include "GPIO_Control.h"
GPIOControl gpio_control;
#endif

#include <Arduino.h>
#include <SPI.h>
#include "Utilities.h"

// CBA FileSystem
#if defined(RNS_USE_FS)
#include "FileSystem.h"
#else
#include "NoopFileSystem.h"
#endif

// CBA SD
#if HAS_SDCARD
#include <SD.h>
SPIClass SDSPI(HSPI);
#endif

#if MCU_VARIANT == MCU_ESP32
  #include <esp_task_wdt.h>
#endif

// Low Power Management
#include "LowPower.h"

// WDT timeout
#define WDT_TIMEOUT 60  // seconds

FIFOBuffer serialFIFO;
uint8_t serialBuffer[CONFIG_UART_BUFFER_SIZE+1];

FIFOBuffer16 packet_starts;
uint16_t packet_starts_buf[CONFIG_QUEUE_MAX_LENGTH+1];

FIFOBuffer16 packet_lengths;
uint16_t packet_lengths_buf[CONFIG_QUEUE_MAX_LENGTH+1];

uint8_t packet_queue[CONFIG_QUEUE_SIZE];

volatile uint8_t queue_height = 0;
volatile uint16_t queued_bytes = 0;
volatile uint16_t queue_cursor = 0;
volatile uint16_t current_packet_start = 0;
volatile bool serial_buffering = false;
#if HAS_BLUETOOTH || HAS_BLE == true
  bool bt_init_ran = false;
#endif

#if HAS_CONSOLE
  #include "Console.h"
#endif

char sbuf[128];

#if MCU_VARIANT == MCU_ESP32 || MCU_VARIANT == MCU_NRF52
  bool packet_ready = false;
#endif

#ifdef HAS_RNS
// CBA LoRa interface
class LoRaInterface : public RNS::InterfaceImpl {
public:
	LoRaInterface() : RNS::InterfaceImpl("LoRaInterface") {
		_IN = true;
		_OUT = true;
		_HW_MTU = 508;
	}
	LoRaInterface(const char *name) : RNS::InterfaceImpl(name) {
		_IN = true;
		_OUT = true;
		_HW_MTU = 508;
	}
	virtual ~LoRaInterface() {
		_name = "deleted";
	}
protected:
	virtual void send_outgoing(const RNS::Bytes& data) {
    // CBA NOTE header will be addded later by transmit function
    TRACEF("LoRaInterface.send_outgoing: (%u bytes) data: %s", data.size(), data.toHex().c_str());
    TRACE("LoRaInterface.send_outgoing: adding packet to outgoing queue...");
    for (size_t i = 0; i < data.size(); i++) {
        if (queue_height < CONFIG_QUEUE_MAX_LENGTH && queued_bytes < CONFIG_QUEUE_SIZE) {
            queued_bytes++;
            packet_queue[queue_cursor++] = data.data()[i];
            if (queue_cursor == CONFIG_QUEUE_SIZE) queue_cursor = 0;
        }
    }
    if (!fifo16_isfull(&packet_starts) && queued_bytes < CONFIG_QUEUE_SIZE) {
        uint16_t s = current_packet_start;
        int16_t e = queue_cursor-1; if (e == -1) e = CONFIG_QUEUE_SIZE-1;
        uint16_t l;

        if (s != e) {
            l = (s < e) ? e - s + 1 : CONFIG_QUEUE_SIZE - s + e + 1;
        } else {
            l = 1;
        }

        if (l >= MIN_L) {
            queue_height++;

            fifo16_push(&packet_starts, s);
            fifo16_push(&packet_lengths, l);

            current_packet_start = queue_cursor;
        }

    }
    // Perform post-send housekeeping
    InterfaceImpl::handle_outgoing(data);
  }
};

// CBA logger callback
void on_log(const char* msg, RNS::LogLevel level) {
  // Only write RNS logs to Serial when no KISS client is connected.
  // KISS protocol and ASCII log text share the same UART — interleaving
  // them corrupts KISS framing and prevents rnodeconf from communicating.
  if (cable_state != CABLE_STATE_CONNECTED) {
    String line = RNS::getTimeString() + String(" [") + RNS::getLevelName(level) + "] " + msg + "\n";
    Serial.print(line);
    Serial.flush();
  }

#ifdef HAS_SDCARD
	File file = SD.open("/logfile.txt", FILE_APPEND);
	if (file) {
    file.write((uint8_t*)line.c_str(), line.length());
    file.close();
  }
#endif  // HAS_SDCARD
}

// CBA receive packet callback
void on_receive_packet(const RNS::Bytes& raw, const RNS::Interface& interface) {
#ifdef HAS_SDCARD
  TRACE("Logging receive packet to SD");
  String line = RNS::getTimeString() + String(" recv: ") + String(raw.toHex().c_str()) + "\n";
	File file = SD.open("/tracefile.txt", FILE_APPEND);
	if (file) {
    file.write((uint8_t*)line.c_str(), line.length());
    file.close();
  }
	RNS::Packet packet({RNS::Type::NONE}, raw);
	if (packet.unpack()) {
    String line = RNS::getTimeString() + String(" recv: ") + String(packet.dumpString().c_str()) + "\n";
    File file = SD.open("/tracedetails.txt", FILE_APPEND);
    if (file) {
      file.write((uint8_t*)line.c_str(), line.length());
      file.close();
    }
	}
#endif  // HAS_SDCARD
}

// CBA transmit packet callback
void on_transmit_packet(const RNS::Bytes& raw, const RNS::Interface& interface) {
#ifdef HAS_SDCARD
  TRACE("Logging transmit packet to SD");
  String line = RNS::getTimeString() + String(" send: ") + String(raw.toHex().c_str()) + "\n";
	File file = SD.open("/tracefile.txt", FILE_APPEND);
	if (file) {
    file.write((uint8_t*)line.c_str(), line.length());
    file.close();
  }
	RNS::Packet packet({RNS::Type::NONE}, raw);
	if (packet.unpack()) {
    String line = RNS::getTimeString() + String(" send: ") + String(packet.dumpString().c_str()) + "\n";
    File file = SD.open("/tracedetails.txt", FILE_APPEND);
    if (file) {
      file.write((uint8_t*)line.c_str(), line.length());
      file.close();
    }
	}
#endif  // HAS_SDCARD
}

// CBA RNS
RNS::Reticulum reticulum(RNS::Type::NONE);
RNS::Interface lora_interface(RNS::Type::NONE);
RNS::FileSystem filesystem(RNS::Type::NONE);
#endif  // HAS_RNS

void setup() {

  // Initialise serial communication
  memset(serialBuffer, 0, sizeof(serialBuffer));
  fifo_init(&serialFIFO, serialBuffer, CONFIG_UART_BUFFER_SIZE);

  #if MCU_VARIANT == MCU_ESP32
    Serial.setRxBufferSize(CONFIG_UART_BUFFER_SIZE);
  #endif
  Serial.begin(serial_baudrate);

  // CBA Safely wait for serial initialization
  while (!Serial) {
    if (millis() > 2000) {
      break;
    }
    delay(10);
  }
  // CBA Test
  delay(2000);

  Serial.println("");
  Serial.println("[RNode] Boot starting...");
  Serial.print("[RNode] Board: ");
  #if BOARD_MODEL == BOARD_RNODE
    Serial.print("RNODE");
  #elif BOARD_MODEL == BOARD_HMBRW
    Serial.print("HMBRW");
  #elif BOARD_MODEL == BOARD_TBEAM
    Serial.print("TBEAM");
  #elif BOARD_MODEL == BOARD_HUZZAH32
    Serial.print("HUZZAH32");
  #elif BOARD_MODEL == BOARD_GENERIC_ESP32
    Serial.print("GENERIC_ESP32");
  #elif BOARD_MODEL == BOARD_LORA32_V1_0
    Serial.print("LORA32_V1_0");
  #elif BOARD_MODEL == BOARD_LORA32_V2_0
    Serial.print("LORA32_V2_0");
  #elif BOARD_MODEL == BOARD_LORA32_V2_1
    Serial.print("LORA32_V2_1");
  #elif BOARD_MODEL == BOARD_HELTEC32_V2
    Serial.print("HELTEC32_V2");
  #elif BOARD_MODEL == BOARD_HELTEC32_V3
    Serial.print("HELTEC32_V3");
  #elif BOARD_MODEL == BOARD_HWSL_V1
    Serial.print("HWSL_V1");
  #elif BOARD_MODEL == BOARD_RNODE_NG_20
    Serial.print("RNODE_NG_20");
  #elif BOARD_MODEL == BOARD_RNODE_NG_21
    Serial.print("RNODE_NG_21");
  #elif BOARD_MODEL == BOARD_RNODE_NG_22
    Serial.print("RNODE_NG_22");
  #elif BOARD_MODEL == BOARD_RAK4631
    Serial.print("RAK4631");
  #elif BOARD_MODEL == BOARD_XIAO_NRF52840
    Serial.print("XIAO_NRF52840");
  #else
    Serial.print("UNKNOWN");
  #endif
  Serial.print(", Baud: ");
  Serial.println(serial_baudrate);

  // Configure WDT
  #if MCU_VARIANT == MCU_ESP32
    esp_task_wdt_init(WDT_TIMEOUT, true); // enable panic so ESP32 restarts
    esp_task_wdt_add(NULL);               // add current thread to WDT watch
  #elif MCU_VARIANT == MCU_NRF52
    NRF_WDT->CONFIG         = 0x01;           // Configure WDT to run when CPU is asleep
    NRF_WDT->CRV            = WDT_TIMEOUT * 32768 + 1; // set timeout
    NRF_WDT->RREN           = 0x01;           // Enable the RR[0] reload register
    NRF_WDT->TASKS_START    = 1;              // Start WDT
  #endif

  #if MCU_VARIANT == MCU_ESP32
    boot_seq();
    EEPROM.begin(EEPROM_SIZE);
    Serial.println("[RNode] EEPROM initialised");
  #endif

  #if MCU_VARIANT == MCU_NRF52
    if (!eeprom_begin()) {
        Serial.write("EEPROM initialisation failed.\r\n");
    }
  #endif

  // Seed the PRNG for CSMA R-value selection
  # if MCU_VARIANT == MCU_ESP32
    // On ESP32, get the seed value from the
    // hardware RNG
    int seed_val = (int)esp_random();
  #else
    // Otherwise, get a pseudo-random seed
    // value from an unconnected analog pin
    int seed_val = analogRead(0);
  #endif
  randomSeed(seed_val);

  #if BOARD_MODEL != BOARD_RAK4631 && BOARD_MODEL != BOARD_RNODE_NG_22 && BOARD_MODEL != BOARD_XIAO_NRF52840
  // Some boards need to wait until the hardware UART is set up before booting
  // the full firmware. In the case of the RAK4631 and XIAO nRF52840, the line
  // below will wait until a serial connection is actually established with a
  // master. Thus, it is disabled on these platforms.
    while (!Serial);
  #endif

  serial_interrupt_init();

  // Configure input and output pins
  #if HAS_INPUT
    input_init();
  #endif

  #if HAS_NP == false
    pinMode(pin_led_rx, OUTPUT);
    pinMode(pin_led_tx, OUTPUT);
  #endif

  #if HAS_TCXO == true
    if (pin_tcxo_enable != -1) {
        pinMode(pin_tcxo_enable, OUTPUT);
        digitalWrite(pin_tcxo_enable, HIGH);
    }
  #endif

  // Initialise buffers
  memset(pbuf, 0, sizeof(pbuf));
  memset(cmdbuf, 0, sizeof(cmdbuf));
  
  memset(packet_queue, 0, sizeof(packet_queue));

  memset(packet_starts_buf, 0, sizeof(packet_starts_buf));
  fifo16_init(&packet_starts, packet_starts_buf, CONFIG_QUEUE_MAX_LENGTH);
  
  memset(packet_lengths_buf, 0, sizeof(packet_starts_buf));
  fifo16_init(&packet_lengths, packet_lengths_buf, CONFIG_QUEUE_MAX_LENGTH);

  // Set chip select, reset and interrupt
  // pins for the LoRa module
  #if MODEM == SX1276 || MODEM == SX1278
  LoRa->setPins(pin_cs, pin_reset, pin_dio, pin_busy);
  #elif MODEM == SX1262
  LoRa->setPins(pin_cs, pin_reset, pin_dio, pin_busy, pin_rxen);
  #elif MODEM == SX1280
  LoRa->setPins(pin_cs, pin_reset, pin_dio, pin_busy, pin_rxen, pin_txen);
  #endif
  
  #if MCU_VARIANT == MCU_ESP32 || MCU_VARIANT == MCU_NRF52
    init_channel_stats();

    // Check installed transceiver chip and
    // probe boot parameters.
    if (LoRa->preInit()) {
      modem_installed = true;
      Serial.println("[RNode] Modem detected");
      uint32_t lfr = LoRa->getFrequency();
      if (lfr == 0) {
        // Normal boot
      } else if (lfr == M_FRQ_R) {
        // Quick reboot
        #if HAS_CONSOLE
          if (rtc_get_reset_reason(0) == POWERON_RESET) {
            console_active = true;
          }
        #endif
      } else {
        // Unknown boot
      }
      LoRa->setFrequency(M_FRQ_S);
    } else {
      modem_installed = false;
      Serial.println("[RNode] ERROR: Modem not detected!");
    }
  #else
    // Older variants only came with SX1276/78 chips,
    // so assume that to be the case for now.
    modem_installed = true;
  #endif

  #if HAS_DISPLAY
    #if HAS_EEPROM
    if (EEPROM.read(eeprom_addr(ADDR_CONF_DSET)) != CONF_OK_BYTE) {
    #elif MCU_VARIANT == MCU_NRF52
    if (eeprom_read(eeprom_addr(ADDR_CONF_DSET)) != CONF_OK_BYTE) {
    #endif
      eeprom_update(eeprom_addr(ADDR_CONF_DSET), CONF_OK_BYTE);
      eeprom_update(eeprom_addr(ADDR_CONF_DINT), 0xFF);
    }
    disp_ready = display_init();
    update_display();
  #endif

  #if MCU_VARIANT == MCU_ESP32 || MCU_VARIANT == MCU_NRF52
    #if HAS_PMU == true
      pmu_ready = init_pmu();
    #endif

    #if HAS_BLUETOOTH || HAS_BLE == true
      bt_init();
      bt_init_ran = true;
      Serial.println("[RNode] Bluetooth initialised");
    #endif

    if (console_active) {
      #if HAS_CONSOLE
        console_start();
      #else
        Serial.println("[RNode] Console requested but disabled, sending KISS reset");
        kiss_indicate_reset();
      #endif
    } else {
      Serial.println("[RNode] Entering TNC/KISS mode");
      kiss_indicate_reset();
    }
  #endif

  // Validate board health, EEPROM and config
  validate_status();
  Serial.print("[RNode] Op mode: ");
  Serial.println(op_mode == MODE_TNC ? "TNC" : "Normal");
  if (op_mode == MODE_TNC) {
    Serial.print("[RNode] Freq: "); Serial.print(lora_freq); Serial.println(" Hz");
    Serial.print("[RNode] BW: "); Serial.print(lora_bw); Serial.println(" Hz");
    Serial.print("[RNode] SF: "); Serial.print(lora_sf);
    Serial.print(" CR: "); Serial.print(lora_cr);
    Serial.print(" TXP: "); Serial.print(lora_txp); Serial.println(" dBm");
  }

  if (op_mode != MODE_TNC) LoRa->setFrequency(0);

  // CBA SD
#ifdef HAS_SDCARD
  pinMode(SDCARD_MISO, INPUT_PULLUP);
  SDSPI.begin(SDCARD_SCLK, SDCARD_MISO, SDCARD_MOSI, SDCARD_CS);
  if (!SD.begin(SDCARD_CS, SDSPI)) {
      Serial.println("setupSDCard FAIL");
  } else {
      uint32_t cardSize = SD.cardSize() / (1024 * 1024);
      Serial.print("setupSDCard PASS . SIZE = ");
      Serial.print(cardSize / 1024.0);
      Serial.println(" GB");
      SD.remove("/logfile");
      SD.remove("/logfile.txt");
      SD.remove("/tracefile");
      SD.remove("/tracedetails");
      SD.remove("/tracefile.txt");
      SD.remove("/tracedetails.txt");
      Serial.println("DIR: /");
      File root = SD.open("/");
      File file = root.openNextFile();
      while(file){
          Serial.print("  FILE: ");
          Serial.println(file.name());
          file = root.openNextFile();
      }
  }
  delay(3000);
#endif

#ifdef HAS_RNS
  try {
    Serial.printf("[HEAP] Before filesystem init: %u bytes free\r\n", ESP.getFreeHeap());
    // CBA Init filesystem
#if defined(RNS_USE_FS)
    filesystem = new FileSystem();
    ((FileSystem*)filesystem.get())->init();
#else
    filesystem = new NoopFileSystem();
    ((FileSystem*)filesystem.get())->init();
#endif

    HEAD("Registering filesystem...", RNS::LOG_TRACE);
    RNS::Utilities::OS::register_filesystem(filesystem);
    Serial.printf("[HEAP] After filesystem init: %u bytes free\r\n", ESP.getFreeHeap());

#ifndef NDEBUG
    //filesystem.remove_directory("/cache");
    //filesystem.remove_file("/destination_table");
    //filesystem.reformat();
    TRACE("Listing filesystem...");
#if defined(RNS_USE_FS)
    //FileSystem::listDir("/");
#endif
    TRACE("Finished listing");
    //TRACE("Dumping filesystem...");
    //FileSystem::dumpDir("/");
    //TRACE("Finished dumping");
    //reticulum.clear_caches();

    // CBA DEBUG
/*
    std::list<std::string> files = filesystem.list_directory("/cache");
    for (auto& file : files) {
      Serial.print("  FILE: ");
      Serial.println(file.c_str());
      //RNS::Bytes content = filesystem.read_file(file.c_str());
      //DEBUG(std::string("FILE: ") + file);
      //DEBUG(content.toString());
      }
*/
    TRACE("FILE: destination_table");
    RNS::Bytes content;
    if (filesystem.read_file("/destination_table", content) > 0) {
      TRACE(content.toString() + "\r\n");
    }
#endif  // NDEBUG

    // CBA Start RNS
    if (hw_ready) {
      RNS::setLogCallback(&on_log);
      RNS::Transport::set_receive_packet_callback(on_receive_packet);
      RNS::Transport::set_transmit_packet_callback(on_transmit_packet);

      Serial.write("Starting RNS...\r\n");
      Serial.printf("[HEAP] Before RNS init: %u bytes free\r\n", ESP.getFreeHeap());
      RNS::loglevel(RNS::LOG_WARNING);
      //RNS::loglevel(RNS::LOG_MEM);

      HEAD("Registering LoRA Interface...", RNS::LOG_TRACE);
      lora_interface = new LoRaInterface();
      lora_interface.mode(RNS::Type::Interface::MODE_GATEWAY);
      RNS::Transport::register_interface(lora_interface);
      Serial.printf("[HEAP] After LoRa interface: %u bytes free\r\n", ESP.getFreeHeap());

      HEAD("Creating Reticulum instance...", RNS::LOG_TRACE);
      Serial.printf("[HEAP] Before Reticulum(): %u bytes free\r\n", ESP.getFreeHeap());
      reticulum = RNS::Reticulum();
      Serial.printf("[HEAP] After Reticulum(): %u bytes free\r\n", ESP.getFreeHeap());
      reticulum.transport_enabled(op_mode == MODE_TNC);
      reticulum.probe_destination_enabled(true);
      Serial.printf("[HEAP] Before start(): %u bytes free\r\n", ESP.getFreeHeap());
      reticulum.start();
      Serial.printf("[HEAP] After start(): %u bytes free\r\n", ESP.getFreeHeap());

      // CBA load/create local destination for admin node
/*
      RNS::Identity identity = {RNS::Type::NONE};
      std::string local_identity_path = RNS::Reticulum::_storagepath + "/local_identity";
      if (RNS::Utilities::OS::file_exists(local_identity_path.c_str())) {
        identity = RNS::Identity::from_file(local_identity_path.c_str());
      }
      if (!identity) {
        RNS::verbose("No valid local identity in storage, creating...");
        identity = RNS::Identity();
        identity.to_file(local_identity_path.c_str());
      }
      else {
        RNS::verbose("Loaded local identity from storage");
      }
      RNS::Destination destination(identity, RNS::Type::Destination::IN, RNS::Type::Destination::SINGLE, "rnstransport", "local");
*/
      RNS::Destination destination(RNS::Transport::identity(), RNS::Type::Destination::IN, RNS::Type::Destination::SINGLE, "rnstransport", "local");

      HEAD("RNS is READY!", RNS::LOG_TRACE);

      // GPIO Control: Initialize LXMF endpoint for GPIO commands
      #ifdef HAS_GPIO_CONTROL
      {
          // Load or create persistent identity for LXMF GPIO endpoint
          RNS::Identity gpio_identity = {RNS::Type::NONE};
          std::string gpio_id_path = std::string(RNS::Reticulum::_storagepath) + "/gpio_identity";
          if (RNS::Utilities::OS::file_exists(gpio_id_path.c_str())) {
              gpio_identity = RNS::Identity::from_file(gpio_id_path.c_str());
              Serial.println("[GPIO] Loaded identity from storage");
          }
          if (!gpio_identity) {
              Serial.println("[GPIO] Creating new identity...");
              gpio_identity = RNS::Identity();
              gpio_identity.to_file(gpio_id_path.c_str());
              Serial.println("[GPIO] New identity saved");
          }
          gpio_control.init(gpio_identity, "GPIO Node");
      }
      #endif
      if (op_mode == MODE_TNC) {
        HEAD("RNS transport mode is ENABLED", RNS::LOG_TRACE);
        TRACE(std::string("Frequency: " + std::to_string(lora_freq)) + " Hz");
        TRACE(std::string("Bandwidth: " + std::to_string(lora_bw)) + " Hz");
        TRACE(std::string("TX Power: " + std::to_string(lora_txp)) + " dBm");
        TRACE(std::string("Spreading Factor: " + std::to_string(lora_sf)));
        TRACE(std::string("Coding Rate: " + std::to_string(lora_sf)));
      }
      else {
        HEAD("RNS transport mode is DISABLED", RNS::LOG_INFO);
        HEAD("Configure TNC mode with radio configuration to enable RNS transport", RNS::LOG_INFO);
      }
      //RNS::loglevel(RNS::LOG_NONE);
    }
    else {
      HEAD("RNS is inoperable because hardware is not ready!", RNS::LOG_ERROR);
      HEAD("Check firmware signature and eeprom provisioning", RNS::LOG_ERROR);
      // CBA Clear cached files just in case cached files are responsible for failure
  		//reticulum.clear_caches();
    }
  }
  catch (std::exception& e) {
    ERROR("RNS startup failed: " + std::string(e.what()));
  }
#endif  // HAS_RNS

  Serial.println("[RNode] Boot complete");
}

void lora_receive() {
  #if defined(HAS_LOWPOWER) && HAS_LOWPOWER == true && MODEM == SX1262
    // Use RX duty cycle mode if enabled for power saving
    const PowerConfig* config = lowpower_get_config();
    if (config != NULL && config->use_rx_duty_cycle && low_power_initialized) {
      // Configure RX duty cycle based on current LoRa parameters
      LoRa->setRxDutyCycleParams(lora_sf, lora_bw, lora_preamble_symbols);
      LoRa->startRxDutyCycle();
      return;
    }
  #endif
  
  // Standard continuous receive mode
  if (!implicit) {
    LoRa->receive();
  } else {
    LoRa->receive(implicit_l);
  }
}

inline void kiss_write_packet() {

#ifdef HAS_RNS
  // CBA send packet received over LoRa to RNS in addition to connected client
  // CBA RESERVE
  //RNS::Bytes data();
  RNS::Bytes data(512);
  for (uint16_t i = 0; i < read_len; i++) {
    data << pbuf[i];
  }
  lora_interface.handle_incoming(data);
#endif

  serial_write(FEND);
  serial_write(CMD_DATA);
  for (uint16_t i = 0; i < read_len; i++) {
    uint8_t byte = pbuf[i];
    if (byte == FEND) { serial_write(FESC); byte = TFEND; }
    if (byte == FESC) { serial_write(FESC); byte = TFESC; }
    serial_write(byte);
  }
  serial_write(FEND);
  read_len = 0;
  #if MCU_VARIANT == MCU_ESP32 || MCU_VARIANT == MCU_NRF52
    packet_ready = false;
  #endif
}

inline void getPacketData(uint16_t len) {
  while (len-- && read_len < MTU) {
    pbuf[read_len++] = LoRa->read();
  }
}

void ISR_VECT receive_callback(int packet_size) {
  if (!promisc) {
    // The standard operating mode allows large
    // packets with a payload up to 500 bytes,
    // by combining two raw LoRa packets.
    // We read the 1-byte header and extract
    // packet sequence number and split flags
    uint8_t header   = LoRa->read(); packet_size--;
    uint8_t sequence = packetSequence(header);
    bool    ready    = false;

    if (isSplitPacket(header) && seq == SEQ_UNSET) {
      // This is the first part of a split
      // packet, so we set the seq variable
      // and add the data to the buffer
      read_len = 0;
      seq = sequence;

      #if MCU_VARIANT != MCU_ESP32 && MCU_VARIANT != MCU_NRF52
        last_rssi = LoRa->packetRssi();
        last_snr_raw = LoRa->packetSnrRaw();
      #endif

      getPacketData(packet_size);

    } else if (isSplitPacket(header) && seq == sequence) {
      // This is the second part of a split
      // packet, so we add it to the buffer
      // and set the ready flag.
      

      #if MCU_VARIANT != MCU_ESP32 && MCU_VARIANT != MCU_NRF52
        last_rssi = (last_rssi+LoRa->packetRssi())/2;
        last_snr_raw = (last_snr_raw+LoRa->packetSnrRaw())/2;
      #endif

      getPacketData(packet_size);

      seq = SEQ_UNSET;
      ready = true;

    } else if (isSplitPacket(header) && seq != sequence) {
      // This split packet does not carry the
      // same sequence id, so we must assume
      // that we are seeing the first part of
      // a new split packet.
      read_len = 0;
      seq = sequence;

      #if MCU_VARIANT != MCU_ESP32 && MCU_VARIANT != MCU_NRF52
        last_rssi = LoRa->packetRssi();
        last_snr_raw = LoRa->packetSnrRaw();
      #endif

      getPacketData(packet_size);

    } else if (!isSplitPacket(header)) {
      // This is not a split packet, so we
      // just read it and set the ready
      // flag to true.

      if (seq != SEQ_UNSET) {
        // If we already had part of a split
        // packet in the buffer, we clear it.
        read_len = 0;
        seq = SEQ_UNSET;
      }

      #if MCU_VARIANT != MCU_ESP32 && MCU_VARIANT != MCU_NRF52
        last_rssi = LoRa->packetRssi();
        last_snr_raw = LoRa->packetSnrRaw();
      #endif

      getPacketData(packet_size);
      ready = true;
    }

    if (ready) {
      #if MCU_VARIANT != MCU_ESP32 && MCU_VARIANT != MCU_NRF52
        // We first signal the RSSI of the
        // recieved packet to the host.
        kiss_indicate_stat_rssi();
        kiss_indicate_stat_snr();

        // And then write the entire packet
        kiss_write_packet();
      #else
        packet_ready = true;
      #endif
    }  
  } else {
    // In promiscuous mode, raw packets are
    // output directly to the host
    read_len = 0;

    #if MCU_VARIANT != MCU_ESP32 && MCU_VARIANT != MCU_NRF52
      last_rssi = LoRa->packetRssi();
      last_snr_raw = LoRa->packetSnrRaw();
      getPacketData(packet_size);

      // We first signal the RSSI of the
      // recieved packet to the host.
      kiss_indicate_stat_rssi();
      kiss_indicate_stat_snr();

      // And then write the entire packet
      kiss_write_packet();

    #else
      getPacketData(packet_size);
      packet_ready = true;
    #endif
  }
}

bool startRadio() {
  update_radio_lock();
  if (!radio_online && !console_active) {
    if (!radio_locked && hw_ready) {
      if (!LoRa->begin(lora_freq)) {
        // The radio could not be started.
        // Indicate this failure over both the
        // serial port and with the onboard LEDs
        radio_error = true;
        Serial.println("[RNode] ERROR: Radio init failed!");
        kiss_indicate_error(ERROR_INITRADIO);
        led_indicate_error(0);
        return false;
      } else {
        radio_online = true;
        Serial.println("[RNode] Radio online");

        init_channel_stats();

        setTXPower();
        setBandwidth();
        setSpreadingFactor();
        setCodingRate();
        getFrequency();

        LoRa->enableCrc();

        LoRa->onReceive(receive_callback);

        // Initialize low power mode (after radio is configured)
        #if defined(HAS_LOWPOWER) && HAS_LOWPOWER == true
          lowpower_init();
          // Set balanced mode by default for transport nodes
          if (op_mode == MODE_TNC) {
            lowpower_set_mode(POWER_MODE_BALANCED);
          }
        #endif

        lora_receive();

        // Flash an info pattern to indicate
        // that the radio is now on
        kiss_indicate_radiostate();
        led_indicate_info(3);
        return true;
      }

    } else {
      // Flash a warning pattern to indicate
      // that the radio was locked, and thus
      // not started
      radio_online = false;
      kiss_indicate_radiostate();
      led_indicate_warning(3);
      return false;
    }
  } else {
    // If radio is already on, we silently
    // ignore the request.
    kiss_indicate_radiostate();
    return true;
  }
}

void stopRadio() {
  LoRa->end();
  radio_online = false;
}

void update_radio_lock() {
  if (lora_freq != 0 && lora_bw != 0 && lora_txp != 0xFF && lora_sf != 0) {
    radio_locked = false;
  } else {
    radio_locked = true;
  }
}

bool queueFull() {
  return (queue_height >= CONFIG_QUEUE_MAX_LENGTH || queued_bytes >= CONFIG_QUEUE_SIZE);
}

volatile bool queue_flushing = false;
void flushQueue(void) {
  if (!queue_flushing) {
    queue_flushing = true;

    led_tx_on();
    uint16_t processed = 0;

    #if MCU_VARIANT == MCU_ESP32 || MCU_VARIANT == MCU_NRF52
    while (!fifo16_isempty(&packet_starts)) {
    #else
    while (!fifo16_isempty_locked(&packet_starts)) {
    #endif

      uint16_t start = fifo16_pop(&packet_starts);
      uint16_t length = fifo16_pop(&packet_lengths);

      if (length >= MIN_L && length <= MTU) {
        for (uint16_t i = 0; i < length; i++) {
          uint16_t pos = (start+i)%CONFIG_QUEUE_SIZE;
          tbuf[i] = packet_queue[pos];
        }

        transmit(length);
        processed++;
      }
    }

    lora_receive();
    led_tx_off();
    post_tx_yield_timeout = millis()+(lora_post_tx_yield_slots*csma_slot_ms);
  }

  queue_height = 0;
  queued_bytes = 0;
  #if MCU_VARIANT == MCU_ESP32 || MCU_VARIANT == MCU_NRF52
    update_airtime();
  #endif
  queue_flushing = false;
}

#if MCU_VARIANT == MCU_ESP32 || MCU_VARIANT == MCU_NRF52
  #define _e 2.71828183
  #define _S 10.0
  float csma_slope(float u) { return (pow(_e,_S*u-_S/2.0))/(pow(_e,_S*u-_S/2.0)+1.0); }
  void update_csma_p() {
      csma_p = (uint8_t)((1.0-(csma_p_min+(csma_p_max-csma_p_min)*csma_slope(airtime)))*255.0);
}
#endif

#define PHY_HEADER_LORA_SYMBOLS 8
void add_airtime(uint16_t written) {
  #if MCU_VARIANT == MCU_ESP32 || MCU_VARIANT == MCU_NRF52
    float packet_cost_ms = 0.0;
    float payload_cost_ms = ((float)written * lora_us_per_byte)/1000.0;
    packet_cost_ms += payload_cost_ms;
    packet_cost_ms += (lora_preamble_symbols+4.25)*lora_symbol_time_ms;
    packet_cost_ms += PHY_HEADER_LORA_SYMBOLS * lora_symbol_time_ms;
    uint16_t cb = current_airtime_bin();
    uint16_t nb = cb+1; if (nb == AIRTIME_BINS) { nb = 0; }
    airtime_bins[cb] += packet_cost_ms;
    airtime_bins[nb] = 0;
  #endif
}

void update_airtime() {
  #if MCU_VARIANT == MCU_ESP32 || MCU_VARIANT == MCU_NRF52
    uint16_t cb = current_airtime_bin();
    uint16_t pb = cb-1; if (cb-1 < 0) { pb = AIRTIME_BINS-1; }
    uint16_t nb = cb+1; if (nb == AIRTIME_BINS) { nb = 0; }
    airtime_bins[nb] = 0;
    airtime = (float)(airtime_bins[cb]+airtime_bins[pb])/(2.0*AIRTIME_BINLEN_MS);

    uint32_t longterm_airtime_sum = 0;
    for (uint16_t bin = 0; bin < AIRTIME_BINS; bin++) {
      longterm_airtime_sum += airtime_bins[bin];
    }
    longterm_airtime = (float)longterm_airtime_sum/(float)AIRTIME_LONGTERM_MS;

    float longterm_channel_util_sum = 0.0;
    for (uint16_t bin = 0; bin < AIRTIME_BINS; bin++) {
      longterm_channel_util_sum += longterm_bins[bin];
    }
    longterm_channel_util = (float)longterm_channel_util_sum/(float)AIRTIME_BINS;

    #if MCU_VARIANT == MCU_ESP32 || MCU_VARIANT == MCU_NRF52
      update_csma_p();
    #endif
    kiss_indicate_channel_stats();
  #endif
}

void transmit(uint16_t size) {
  if (radio_online) {
    if (!promisc) {
      uint16_t  written = 0;
      uint8_t header  = random(256) & 0xF0;

      if (size > SINGLE_MTU - HEADER_L) {
        header = header | FLAG_SPLIT;
      }

      LoRa->beginPacket();
      LoRa->write(header); written++;

      for (uint16_t i=0; i < size; i++) {
        LoRa->write(tbuf[i]);

        written++;

        if (written == 255) {
          LoRa->endPacket(); add_airtime(written);
          LoRa->beginPacket();
          LoRa->write(header);
          written = 1;
        }
      }

      LoRa->endPacket(); add_airtime(written);
    } else {
      // In promiscuous mode, we only send out
      // plain raw LoRa packets with a maximum
      // payload of 255 bytes
      led_tx_on();
      uint16_t  written = 0;
      
      // Cap packets at 255 bytes
      if (size > SINGLE_MTU) {
        size = SINGLE_MTU;
      }

      // If implicit header mode has been set,
      // set packet length to payload data length
      if (!implicit) {
        LoRa->beginPacket();
      } else {
        LoRa->beginPacket(size);
      }

      for (uint16_t i=0; i < size; i++) {
        LoRa->write(tbuf[i]);

        written++;
      }
      LoRa->endPacket(); add_airtime(written);
    }
  } else {
    kiss_indicate_error(ERROR_TXFAILED);
    led_indicate_error(5);
  }
}

void serialCallback(uint8_t sbyte) {
  if (IN_FRAME && sbyte == FEND && command == CMD_DATA) {
    IN_FRAME = false;

    if (!fifo16_isfull(&packet_starts) && queued_bytes < CONFIG_QUEUE_SIZE) {
        uint16_t s = current_packet_start;
        int16_t e = queue_cursor-1; if (e == -1) e = CONFIG_QUEUE_SIZE-1;
        uint16_t l;

        if (s != e) {
            l = (s < e) ? e - s + 1 : CONFIG_QUEUE_SIZE - s + e + 1;
        } else {
            l = 1;
        }

        if (l >= MIN_L) {
            queue_height++;

            fifo16_push(&packet_starts, s);
            fifo16_push(&packet_lengths, l);

            current_packet_start = queue_cursor;
        }

    }

  } else if (sbyte == FEND) {
    IN_FRAME = true;
    command = CMD_UNKNOWN;
    frame_len = 0;
  } else if (IN_FRAME && frame_len < MTU) {
    // Have a look at the command byte first
    if (frame_len == 0 && command == CMD_UNKNOWN) {
        command = sbyte;
    } else if (command == CMD_DATA) {
        if (bt_state != BT_STATE_CONNECTED) cable_state = CABLE_STATE_CONNECTED;
        if (sbyte == FESC) {
            ESCAPE = true;
        } else {
            if (ESCAPE) {
                if (sbyte == TFEND) sbyte = FEND;
                if (sbyte == TFESC) sbyte = FESC;
                ESCAPE = false;
            }
            if (queue_height < CONFIG_QUEUE_MAX_LENGTH && queued_bytes < CONFIG_QUEUE_SIZE) {
              queued_bytes++;
              packet_queue[queue_cursor++] = sbyte;
              if (queue_cursor == CONFIG_QUEUE_SIZE) queue_cursor = 0;
            }
        }
    } else if (command == CMD_FREQUENCY) {
      if (sbyte == FESC) {
            ESCAPE = true;
        } else {
            if (ESCAPE) {
                if (sbyte == TFEND) sbyte = FEND;
                if (sbyte == TFESC) sbyte = FESC;
                ESCAPE = false;
            }
            if (frame_len < CMD_L) cmdbuf[frame_len++] = sbyte;
        }

        if (frame_len == 4) {
          uint32_t freq = (uint32_t)cmdbuf[0] << 24 | (uint32_t)cmdbuf[1] << 16 | (uint32_t)cmdbuf[2] << 8 | (uint32_t)cmdbuf[3];

          if (freq == 0) {
            kiss_indicate_frequency();
          } else {
            lora_freq = freq;
            if (op_mode == MODE_HOST) setFrequency();
            kiss_indicate_frequency();
          }
        }
    } else if (command == CMD_BANDWIDTH) {
      if (sbyte == FESC) {
            ESCAPE = true;
        } else {
            if (ESCAPE) {
                if (sbyte == TFEND) sbyte = FEND;
                if (sbyte == TFESC) sbyte = FESC;
                ESCAPE = false;
            }
            if (frame_len < CMD_L) cmdbuf[frame_len++] = sbyte;
        }

        if (frame_len == 4) {
          uint32_t bw = (uint32_t)cmdbuf[0] << 24 | (uint32_t)cmdbuf[1] << 16 | (uint32_t)cmdbuf[2] << 8 | (uint32_t)cmdbuf[3];

          if (bw == 0) {
            kiss_indicate_bandwidth();
          } else {
            lora_bw = bw;
            if (op_mode == MODE_HOST) setBandwidth();
            kiss_indicate_bandwidth();
          }
        }
    } else if (command == CMD_TXPOWER) {
      if (sbyte == 0xFF) {
        kiss_indicate_txpower();
      } else {
        int txp = sbyte;
        #if MODEM == SX1262
          if (txp > 22) txp = 22;
        #else
          if (txp > 17) txp = 17;
        #endif

        lora_txp = txp;
        if (op_mode == MODE_HOST) setTXPower();
        kiss_indicate_txpower();
      }
    } else if (command == CMD_SF) {
      if (sbyte == 0xFF) {
        kiss_indicate_spreadingfactor();
      } else {
        int sf = sbyte;
        if (sf < 5) sf = 5;
        if (sf > 12) sf = 12;

        lora_sf = sf;
        if (op_mode == MODE_HOST) setSpreadingFactor();
        kiss_indicate_spreadingfactor();
      }
    } else if (command == CMD_CR) {
      if (sbyte == 0xFF) {
        kiss_indicate_codingrate();
      } else {
        int cr = sbyte;
        if (cr < 5) cr = 5;
        if (cr > 8) cr = 8;

        lora_cr = cr;
        if (op_mode == MODE_HOST) setCodingRate();
        kiss_indicate_codingrate();
      }
    } else if (command == CMD_IMPLICIT) {
      set_implicit_length(sbyte);
      kiss_indicate_implicit_length();
    } else if (command == CMD_LEAVE) {
      if (sbyte == 0xFF) {
        cable_state   = CABLE_STATE_DISCONNECTED;
        current_rssi  = -292;
        last_rssi     = -292;
        last_rssi_raw = 0x00;
        last_snr_raw  = 0x80;
      }
    } else if (command == CMD_RADIO_STATE) {
      if (bt_state != BT_STATE_CONNECTED) cable_state = CABLE_STATE_CONNECTED;
      if (sbyte == 0xFF) {
        kiss_indicate_radiostate();
      } else if (sbyte == 0x00) {
        stopRadio();
        kiss_indicate_radiostate();
      } else if (sbyte == 0x01) {
        startRadio();
        kiss_indicate_radiostate();
      }
    } else if (command == CMD_ST_ALOCK) {
      if (sbyte == FESC) {
            ESCAPE = true;
        } else {
            if (ESCAPE) {
                if (sbyte == TFEND) sbyte = FEND;
                if (sbyte == TFESC) sbyte = FESC;
                ESCAPE = false;
            }
            if (frame_len < CMD_L) cmdbuf[frame_len++] = sbyte;
        }

        if (frame_len == 2) {
          uint16_t at = (uint16_t)cmdbuf[0] << 8 | (uint16_t)cmdbuf[1];

          if (at == 0) {
            st_airtime_limit = 0.0;
          } else {
            st_airtime_limit = (float)at/(100.0*100.0);
            if (st_airtime_limit >= 1.0) { st_airtime_limit = 0.0; }
          }
          kiss_indicate_st_alock();
        }
    } else if (command == CMD_LT_ALOCK) {
      if (sbyte == FESC) {
            ESCAPE = true;
        } else {
            if (ESCAPE) {
                if (sbyte == TFEND) sbyte = FEND;
                if (sbyte == TFESC) sbyte = FESC;
                ESCAPE = false;
            }
            if (frame_len < CMD_L) cmdbuf[frame_len++] = sbyte;
        }

        if (frame_len == 2) {
          uint16_t at = (uint16_t)cmdbuf[0] << 8 | (uint16_t)cmdbuf[1];

          if (at == 0) {
            lt_airtime_limit = 0.0;
          } else {
            lt_airtime_limit = (float)at/(100.0*100.0);
            if (lt_airtime_limit >= 1.0) { lt_airtime_limit = 0.0; }
          }
          kiss_indicate_lt_alock();
        }
    } else if (command == CMD_STAT_RX) {
      kiss_indicate_stat_rx();
    } else if (command == CMD_STAT_TX) {
      kiss_indicate_stat_tx();
    } else if (command == CMD_STAT_RSSI) {
      kiss_indicate_stat_rssi();
    } else if (command == CMD_RADIO_LOCK) {
      update_radio_lock();
      kiss_indicate_radio_lock();
    } else if (command == CMD_BLINK) {
      led_indicate_info(sbyte);
    } else if (command == CMD_RANDOM) {
      kiss_indicate_random(getRandom());
    } else if (command == CMD_DETECT) {
      if (sbyte == DETECT_REQ) {
        if (bt_state != BT_STATE_CONNECTED) cable_state = CABLE_STATE_CONNECTED;
        kiss_indicate_detect();
      }
    } else if (command == CMD_PROMISC) {
      if (sbyte == 0x01) {
        promisc_enable();
      } else if (sbyte == 0x00) {
        promisc_disable();
      }
      kiss_indicate_promisc();
    } else if (command == CMD_READY) {
      if (!queueFull()) {
        kiss_indicate_ready();
      } else {
        kiss_indicate_not_ready();
      }
    } else if (command == CMD_UNLOCK_ROM) {
      if (sbyte == ROM_UNLOCK_BYTE) {
        unlock_rom();
      }
    } else if (command == CMD_RESET) {
      if (sbyte == CMD_RESET_BYTE) {
        hard_reset();
      }
    } else if (command == CMD_ROM_READ) {
      kiss_dump_eeprom();
    } else if (command == CMD_ROM_WRITE) {
      if (sbyte == FESC) {
            ESCAPE = true;
        } else {
            if (ESCAPE) {
                if (sbyte == TFEND) sbyte = FEND;
                if (sbyte == TFESC) sbyte = FESC;
                ESCAPE = false;
            }
            if (frame_len < CMD_L) cmdbuf[frame_len++] = sbyte;
        }

        if (frame_len == 2) {
          eeprom_write(cmdbuf[0], cmdbuf[1]);
        }
    } else if (command == CMD_FW_VERSION) {
      kiss_indicate_version();
    } else if (command == CMD_PLATFORM) {
      kiss_indicate_platform();
    } else if (command == CMD_MCU) {
      kiss_indicate_mcu();
    } else if (command == CMD_BOARD) {
      kiss_indicate_board();
    } else if (command == CMD_CONF_SAVE) {
      eeprom_conf_save();
    } else if (command == CMD_CONF_DELETE) {
      eeprom_conf_delete();
    } else if (command == CMD_FB_EXT) {
      #if HAS_DISPLAY == true
        if (sbyte == 0xFF) {
          kiss_indicate_fbstate();
        } else if (sbyte == 0x00) {
          ext_fb_disable();
          kiss_indicate_fbstate();
        } else if (sbyte == 0x01) {
          ext_fb_enable();
          kiss_indicate_fbstate();
        }
      #endif
    } else if (command == CMD_FB_WRITE) {
      if (sbyte == FESC) {
            ESCAPE = true;
        } else {
            if (ESCAPE) {
                if (sbyte == TFEND) sbyte = FEND;
                if (sbyte == TFESC) sbyte = FESC;
                ESCAPE = false;
            }
            if (frame_len < CMD_L) cmdbuf[frame_len++] = sbyte;
        }
        #if HAS_DISPLAY
          if (frame_len == 9) {
            uint8_t line = cmdbuf[0];
            if (line > 63) line = 63;
            int fb_o = line*8; 
            memcpy(fb+fb_o, cmdbuf+1, 8);
          }
        #endif
    } else if (command == CMD_FB_READ) {
      if (sbyte != 0x00) {
        kiss_indicate_fb();
      }
    } else if (command == CMD_DEV_HASH) {
      #if MCU_VARIANT == MCU_ESP32 || MCU_VARIANT == MCU_NRF52
        if (sbyte != 0x00) {
          kiss_indicate_device_hash();
        }
      #endif
    } else if (command == CMD_DEV_SIG) {
      #if MCU_VARIANT == MCU_ESP32 || MCU_VARIANT == MCU_NRF52
        if (sbyte == FESC) {
              ESCAPE = true;
          } else {
              if (ESCAPE) {
                  if (sbyte == TFEND) sbyte = FEND;
                  if (sbyte == TFESC) sbyte = FESC;
                  ESCAPE = false;
              }
              if (frame_len < CMD_L) cmdbuf[frame_len++] = sbyte;
          }

          if (frame_len == DEV_SIG_LEN) {
            memcpy(dev_sig, cmdbuf, DEV_SIG_LEN);
            device_save_signature();
          }
      #endif
    } else if (command == CMD_FW_UPD) {
      if (sbyte == 0x01) {
        firmware_update_mode = true;
      } else {
        firmware_update_mode = false;
      }
    } else if (command == CMD_HASHES) {
      #if MCU_VARIANT == MCU_ESP32 || MCU_VARIANT == MCU_NRF52
        if (sbyte == 0x01) {
          kiss_indicate_target_fw_hash();
        } else if (sbyte == 0x02) {
          kiss_indicate_fw_hash();
        } else if (sbyte == 0x03) {
          kiss_indicate_bootloader_hash();
        } else if (sbyte == 0x04) {
          kiss_indicate_partition_table_hash();
        }
      #endif
    } else if (command == CMD_FW_HASH) {
      #if MCU_VARIANT == MCU_ESP32 || MCU_VARIANT == MCU_NRF52
        if (sbyte == FESC) {
              ESCAPE = true;
          } else {
              if (ESCAPE) {
                  if (sbyte == TFEND) sbyte = FEND;
                  if (sbyte == TFESC) sbyte = FESC;
                  ESCAPE = false;
              }
              if (frame_len < CMD_L) cmdbuf[frame_len++] = sbyte;
          }

          if (frame_len == DEV_HASH_LEN) {
            memcpy(dev_firmware_hash_target, cmdbuf, DEV_HASH_LEN);
            device_save_firmware_hash();
          }
      #endif
    } else if (command == CMD_BT_CTRL) {
      #if HAS_BLUETOOTH || HAS_BLE
        if (sbyte == 0x00) {
          bt_stop();
          bt_conf_save(false);
        } else if (sbyte == 0x01) {
          bt_start();
          bt_conf_save(true);
        } else if (sbyte == 0x02) {
          bt_enable_pairing();
        }
      #endif
    } else if (command == CMD_DISP_INT) {
      #if HAS_DISPLAY
        if (sbyte == FESC) {
            ESCAPE = true;
        } else {
            if (ESCAPE) {
                if (sbyte == TFEND) sbyte = FEND;
                if (sbyte == TFESC) sbyte = FESC;
                ESCAPE = false;
            }
            display_intensity = sbyte;
            di_conf_save(display_intensity);
        }

      #endif
    } else if (command == CMD_DISP_ADDR) {
      #if HAS_DISPLAY
        if (sbyte == FESC) {
            ESCAPE = true;
        } else {
            if (ESCAPE) {
                if (sbyte == TFEND) sbyte = FEND;
                if (sbyte == TFESC) sbyte = FESC;
                ESCAPE = false;
            }
            display_addr = sbyte;
            da_conf_save(display_addr);
        }

      #endif
    }
  }
}

#if MCU_VARIANT == MCU_ESP32
  portMUX_TYPE update_lock = portMUX_INITIALIZER_UNLOCKED;
#endif

void updateModemStatus() {
  #if MCU_VARIANT == MCU_ESP32
    portENTER_CRITICAL(&update_lock);
  #elif MCU_VARIANT == MCU_NRF52
    portENTER_CRITICAL();
  #endif

  uint8_t status = LoRa->modemStatus();
  current_rssi = LoRa->currentRssi();
  last_status_update = millis();

  #if MCU_VARIANT == MCU_ESP32
    portEXIT_CRITICAL(&update_lock);
  #elif MCU_VARIANT == MCU_NRF52
    portEXIT_CRITICAL();
  #endif

  if ((status & SIG_DETECT) == SIG_DETECT) { stat_signal_detected = true; } else { stat_signal_detected = false; }
  if ((status & SIG_SYNCED) == SIG_SYNCED) { stat_signal_synced = true; } else { stat_signal_synced = false; }
  if ((status & RX_ONGOING) == RX_ONGOING) { stat_rx_ongoing = true; } else { stat_rx_ongoing = false; }

  // if (stat_signal_detected || stat_signal_synced || stat_rx_ongoing) {
  if (stat_signal_detected || stat_signal_synced) {
    if (stat_rx_ongoing) {
      if (dcd_count < dcd_threshold) {
        dcd_count++;
      } else {
        last_dcd = last_status_update;
        dcd_led = true;
        dcd = true;
      }
    }
  } else {
    #define DCD_LED_STEP_D 3
    if (dcd_count == 0) {
      dcd_led = false;
    } else if (dcd_count > DCD_LED_STEP_D) {
      dcd_count -= DCD_LED_STEP_D;
    } else {
      dcd_count = 0;
    }

    if (last_status_update > last_dcd+csma_slot_ms) {
      dcd = false;
      dcd_led = false;
      dcd_count = 0;
    }
  }

  if (dcd_led) {
    led_rx_on();
  } else {
    if (airtime_lock) {
      led_indicate_airtime_lock();
    } else {
      led_rx_off();
    }
  }
}

void checkModemStatus() {
  if (millis()-last_status_update >= status_interval_ms) {
    updateModemStatus();

    #if MCU_VARIANT == MCU_ESP32 || MCU_VARIANT == MCU_NRF52
      util_samples[dcd_sample] = dcd;
      dcd_sample = (dcd_sample+1)%DCD_SAMPLES;
      if (dcd_sample % UTIL_UPDATE_INTERVAL == 0) {
        int util_count = 0;
        for (int ui = 0; ui < DCD_SAMPLES; ui++) {
          if (util_samples[ui]) util_count++;
        }
        local_channel_util = (float)util_count / (float)DCD_SAMPLES;
        total_channel_util = local_channel_util + airtime;
        if (total_channel_util > 1.0) total_channel_util = 1.0;

        int16_t cb = current_airtime_bin();
        uint16_t nb = cb+1; if (nb == AIRTIME_BINS) { nb = 0; }
        if (total_channel_util > longterm_bins[cb]) longterm_bins[cb] = total_channel_util;
        longterm_bins[nb] = 0.0;

        update_airtime();
      }
    #endif
  }
}

void validate_status() {
  #if MCU_VARIANT == MCU_1284P
      uint8_t boot_flags = OPTIBOOT_MCUSR;
      uint8_t F_POR = PORF;
      uint8_t F_BOR = BORF;
      uint8_t F_WDR = WDRF;
  #elif MCU_VARIANT == MCU_2560
      uint8_t boot_flags = OPTIBOOT_MCUSR;
      if (boot_flags == 0x00) boot_flags = 0x03;
      uint8_t F_POR = PORF;
      uint8_t F_BOR = BORF;
      uint8_t F_WDR = WDRF;
  #elif MCU_VARIANT == MCU_ESP32
      // TODO: Get ESP32 boot flags
      uint8_t boot_flags = 0x02;
      uint8_t F_POR = 0x00;
      uint8_t F_BOR = 0x00;
      uint8_t F_WDR = 0x01;
  #elif MCU_VARIANT == MCU_NRF52
      // TODO: Get NRF52 boot flags
      uint8_t boot_flags = 0x02;
      uint8_t F_POR = 0x00;
      uint8_t F_BOR = 0x00;
      uint8_t F_WDR = 0x01;
  #endif

  if (hw_ready || device_init_done) {
    hw_ready = false;
    Serial.write("Error, invalid hardware check state\r\n");
    #if HAS_DISPLAY
      if (disp_ready) {
        device_init_done = true;
        update_display();
      }
    #endif
    led_indicate_boot_error();
  }

  if (boot_flags & (1<<F_POR)) {
    boot_vector = START_FROM_POWERON;
  } else if (boot_flags & (1<<F_BOR)) {
    boot_vector = START_FROM_BROWNOUT;
  } else if (boot_flags & (1<<F_WDR)) {
    boot_vector = START_FROM_BOOTLOADER;
  } else {
      Serial.write("Error, indeterminate boot vector\r\n");
      #if HAS_DISPLAY
        if (disp_ready) {
          device_init_done = true;
          update_display();
        }
      #endif
      led_indicate_boot_error();
  }

  if (boot_vector == START_FROM_BOOTLOADER || boot_vector == START_FROM_POWERON) {
    if (eeprom_lock_set()) {
      if (eeprom_product_valid() && eeprom_model_valid() && eeprom_hwrev_valid()) {
        if (eeprom_checksum_valid()) {
          eeprom_ok = true;
          if (modem_installed) {
            #if PLATFORM == PLATFORM_ESP32 || PLATFORM == PLATFORM_NRF52
              if (device_init()) {
                hw_ready = true;
              } else {
                hw_ready = false;
              }
            #else
              hw_ready = true;
            #endif
          } else {
            hw_ready = false;
            Serial.write("No valid radio module found\r\n");
            #if HAS_DISPLAY
              if (disp_ready) {
                device_init_done = true;
                update_display();
              }
            #endif
          }
          
          if (hw_ready && eeprom_have_conf()) {
            eeprom_conf_load();
            op_mode = MODE_TNC;
            startRadio();
          }
        } else {
          hw_ready = false;
          Serial.write("Error, eeprom checksum invalid\r\n");
          #if HAS_DISPLAY
            if (disp_ready) {
              device_init_done = true;
              update_display();
            }
          #endif
        }
      } else {
        hw_ready = false;
        Serial.write("Error, eeprom hardware config invalid\r\n");
        #if HAS_DISPLAY
          if (disp_ready) {
            device_init_done = true;
            update_display();
          }
        #endif
      }
    } else {
      hw_ready = false;
      Serial.write("Error, eeprom lock not set\r\n");
      #if HAS_DISPLAY
        if (disp_ready) {
          device_init_done = true;
          update_display();
        }
      #endif
    }
  } else {
    hw_ready = false;
    Serial.write("Error, incorrect boot vector\r\n");
    #if HAS_DISPLAY
      if (disp_ready) {
        device_init_done = true;
        update_display();
      }
    #endif
    led_indicate_boot_error();
  }
}

void loop() {

#ifdef HAS_RNS
  // CBA
  if (reticulum) {
	  reticulum.loop();
  }
#endif

  // GPIO Control: periodic announces, message handling
  #ifdef HAS_GPIO_CONTROL
  gpio_control.loop();
  #endif

  if (radio_online) {
    #if MCU_VARIANT == MCU_ESP32
      if (packet_ready) {
        portENTER_CRITICAL(&update_lock);
        last_rssi = LoRa->packetRssi();
        last_snr_raw = LoRa->packetSnrRaw();
        portEXIT_CRITICAL(&update_lock);
        kiss_indicate_stat_rssi();
        kiss_indicate_stat_snr();
        kiss_write_packet();
      }

      airtime_lock = false;
      if (st_airtime_limit != 0.0 && airtime >= st_airtime_limit) airtime_lock = true;
      if (lt_airtime_limit != 0.0 && longterm_airtime >= lt_airtime_limit) airtime_lock = true;

    #elif MCU_VARIANT == MCU_NRF52
      if (packet_ready) {
        portENTER_CRITICAL();
        last_rssi = LoRa->packetRssi();
        last_snr_raw = LoRa->packetSnrRaw();
        portEXIT_CRITICAL();
        kiss_indicate_stat_rssi();
        kiss_indicate_stat_snr();
        kiss_write_packet();
      }

      airtime_lock = false;
      if (st_airtime_limit != 0.0 && airtime >= st_airtime_limit) airtime_lock = true;
      if (lt_airtime_limit != 0.0 && longterm_airtime >= lt_airtime_limit) airtime_lock = true;
    #endif

    checkModemStatus();
    if (!airtime_lock) {
      if (queue_height > 0) {
        #if MCU_VARIANT == MCU_ESP32 || MCU_VARIANT == MCU_NRF52
          long check_time = millis();
          if (check_time > post_tx_yield_timeout) {
            if (dcd_waiting && (check_time >= dcd_wait_until)) { dcd_waiting = false; }
            if (!dcd_waiting) {
              for (uint8_t dcd_i = 0; dcd_i < dcd_threshold*2; dcd_i++) {
                delay(STATUS_INTERVAL_MS); updateModemStatus();
              }

              if (!dcd) {
                uint8_t csma_r = (uint8_t)random(256);
                if (csma_p >= csma_r) {
                  flushQueue();
                } else {
                  dcd_waiting = true;
                  dcd_wait_until = millis()+csma_slot_ms;
                }
              }
            }
          }
          
        #else
          if (!dcd_waiting) updateModemStatus();

          if (!dcd && !dcd_led) {
            if (dcd_waiting) delay(lora_rx_turnaround_ms);

            updateModemStatus();

            if (!dcd) {
              dcd_waiting = false;
              flushQueue();
            }

          } else {
            dcd_waiting = true;
          }
        #endif
      }
    }
  
  } else {
    if (hw_ready) {
      if (console_active) {
        #if HAS_CONSOLE
          console_loop();
        #endif
      } else {
        led_indicate_standby();
      }
    } else {

      led_indicate_not_ready();
      stopRadio();
    }
  }

  #if MCU_VARIANT == MCU_ESP32 || MCU_VARIANT == MCU_NRF52
      buffer_serial();
      if (!fifo_isempty(&serialFIFO)) serial_poll();
  #else
    if (!fifo_isempty_locked(&serialFIFO)) serial_poll();
  #endif

  #if HAS_DISPLAY
    if (disp_ready) update_display();
  #endif

  #if HAS_PMU
    if (pmu_ready) update_pmu();
  #endif

  #if HAS_BLUETOOTH || HAS_BLE == true
    if (!console_active && bt_ready) update_bt();
  #endif

  #if HAS_INPUT
    input_read();
  #endif

  // Low Power Sleep - Allow MCU to enter low power mode between loop iterations
  // This is critical for power saving on battery-powered devices
  #if defined(HAS_LOWPOWER) && HAS_LOWPOWER == true && MCU_VARIANT == MCU_NRF52
    // Only sleep if radio is online and we're not actively processing
    if (radio_online && !packet_ready && queue_height == 0 && !serial_buffering) {
      const PowerConfig* config = lowpower_get_config();
      if (config != NULL && config->main_loop_sleep_ms > 0) {
        // FreeRTOS delay() enables tickless idle for power saving
        delay(config->main_loop_sleep_ms);
      }
    }
  #endif

  // Feed WDT
#if MCU_VARIANT == MCU_ESP32
  esp_task_wdt_reset();
#elif MCU_VARIANT == MCU_NRF52
  NRF_WDT->RR[0] = WDT_RR_RR_Reload;
#endif
}

void sleep_now() {
  #if HAS_SLEEP == true
    #if BOARD_MODEL == BOARD_RNODE_NG_22
      display_intensity = 0;
      update_display(true);
    #endif
    #if PIN_DISP_SLEEP >= 0
      pinMode(PIN_DISP_SLEEP, OUTPUT);
      digitalWrite(PIN_DISP_SLEEP, DISP_SLEEP_LEVEL);
    #endif
    
    #if MCU_VARIANT == MCU_ESP32
      esp_sleep_enable_ext0_wakeup(PIN_WAKEUP, WAKEUP_LEVEL);
      esp_deep_sleep_start();
    #elif MCU_VARIANT == MCU_NRF52
      // For nRF52, we use System OFF mode for deep sleep
      // Configure wake on DIO1 (radio interrupt)
      // Note: This will reset the device on wake
      #if defined(PIN_WAKEUP)
        nrf_gpio_cfg_sense_input(g_ADigitalPinMap[PIN_WAKEUP], 
                                 NRF_GPIO_PIN_PULLDOWN, 
                                 NRF_GPIO_PIN_SENSE_HIGH);
      #endif
      // Put radio to sleep first
      if (radio_online) {
        LoRa->sleep();
      }
      // Enter System OFF (deep sleep)
      sd_power_system_off();
    #endif
  #endif
}

void button_event(uint8_t event, unsigned long duration) {
  if (duration > 2000) {
    sleep_now();
  }
}

volatile bool serial_polling = false;
void serial_poll() {
  serial_polling = true;

  #if MCU_VARIANT != MCU_ESP32 && MCU_VARIANT != MCU_NRF52
  while (!fifo_isempty_locked(&serialFIFO)) {
  #else
  while (!fifo_isempty(&serialFIFO)) {
  #endif
    char sbyte = fifo_pop(&serialFIFO);
    serialCallback(sbyte);
  }

  serial_polling = false;
}

#if MCU_VARIANT != MCU_ESP32
  #define MAX_CYCLES 20
#else
  #define MAX_CYCLES 10
#endif
void buffer_serial() {
  if (!serial_buffering) {
    serial_buffering = true;

    uint8_t c = 0;

    #if HAS_BLUETOOTH || HAS_BLE == true
    while (
      c < MAX_CYCLES &&
      ( (bt_state != BT_STATE_CONNECTED && Serial.available()) || (bt_state == BT_STATE_CONNECTED && SerialBT.available()) )
      )
    #else
    while (c < MAX_CYCLES && Serial.available())
    #endif
    {
      c++;

      #if MCU_VARIANT != MCU_ESP32 && MCU_VARIANT != MCU_NRF52
        if (!fifo_isfull_locked(&serialFIFO)) {
          fifo_push_locked(&serialFIFO, Serial.read());
        }
      #elif HAS_BLUETOOTH || HAS_BLE == true
        if (bt_state == BT_STATE_CONNECTED) {
          if (!fifo_isfull(&serialFIFO)) {
            fifo_push(&serialFIFO, SerialBT.read());
          }
        } else {
          if (!fifo_isfull(&serialFIFO)) {
            fifo_push(&serialFIFO, Serial.read());
          }
        }
      #else
        if (!fifo_isfull(&serialFIFO)) {
          fifo_push(&serialFIFO, Serial.read());
        }
      #endif
    }

    serial_buffering = false;
  }
}

void serial_interrupt_init() {
  #if MCU_VARIANT == MCU_1284P
      TCCR3A = 0;
      TCCR3B = _BV(CS10) |
               _BV(WGM33)|
               _BV(WGM32);

      // Buffer incoming frames every 1ms
      ICR3 = 16000;

      TIMSK3 = _BV(ICIE3);

  #elif MCU_VARIANT == MCU_2560
      // TODO: This should probably be updated for
      // atmega2560 support. Might be source of
      // reported issues from snh.
      TCCR3A = 0;
      TCCR3B = _BV(CS10) |
               _BV(WGM33)|
               _BV(WGM32);

      // Buffer incoming frames every 1ms
      ICR3 = 16000;

      TIMSK3 = _BV(ICIE3);

  #elif MCU_VARIANT == MCU_ESP32
      // No interrupt-based polling on ESP32
  #endif

}

#if MCU_VARIANT == MCU_1284P || MCU_VARIANT == MCU_2560
  ISR(TIMER3_CAPT_vect) {
    buffer_serial();
  }
#endif
