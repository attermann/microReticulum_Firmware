// Copyright Sandeep Mistry, Mark Qvist and Jacob Eva.
// Licensed under the MIT license.

#ifndef SX126X_H
#define SX126X_H

#include <Arduino.h>
#include <SPI.h>
#include "Modem.h"
#include "LoRaRadio.h"

#define LORA_DEFAULT_SS_PIN    10
#define LORA_DEFAULT_RESET_PIN 9
#define LORA_DEFAULT_DIO0_PIN  2
#define LORA_DEFAULT_RXEN_PIN  -1
#define LORA_DEFAULT_TXEN_PIN  -1
#define LORA_DEFAULT_BUSY_PIN  -1
#define LORA_MODEM_TIMEOUT_MS 20E3

#define PA_OUTPUT_RFO_PIN      0
#define PA_OUTPUT_PA_BOOST_PIN 1

#define RSSI_OFFSET 157

class sx126x : public ILoRaRadio {
public:
  sx126x();

  int begin(uint32_t frequency) override;
  void end() override;

  int beginPacket(int implicitHeader = false) override;
  int endPacket() override;

  int parsePacket(int size = 0);
  int packetRssi() override;
  int packetRssi(uint8_t pkt_snr_raw) override;
  int currentRssi() override;
  uint8_t packetRssiRaw();
  uint8_t currentRssiRaw();
  uint8_t packetSnrRaw() override;
  float packetSnr();
  long packetFrequencyError();

  // from Print
  virtual size_t write(uint8_t byte);
  virtual size_t write(const uint8_t *buffer, size_t size);

  // from Stream
  virtual int available();
  virtual int read();
  virtual int peek();
  virtual void flush();

  void onReceive(void(*callback)(int)) override;

  void receive(int size = 0) override;
  void standby();
  void sleep();
  void reset(void) override;

  bool preInit() override;
  bool isResponding() override;
  uint8_t getTxPower() override;
  void setTxPower(int level, int outputPin = PA_OUTPUT_PA_BOOST_PIN) override;
  uint32_t getFrequency() override;
  void setFrequency(uint32_t frequency) override;
  void setSpreadingFactor(int sf) override;
  uint32_t getSignalBandwidth() override;
  void setSignalBandwidth(uint32_t sbw) override;
  void setCodingRate4(int denominator) override;
  void setPreambleLength(long preamble_symbols) override;
  void setSyncWord(uint16_t sw);
  bool dcd() override;
  void enableCrc() override;
  void disableCrc();
  void enableTCXO();
  void disableTCXO();

  // Runtime overrides used by the native target. setTcxoVoltage() sets the
  // byte written to the DIO3-TCXO-control opcode; 0xFF (default) defers to
  // the per-board compile-time #if BOARD_MODEL chain in enableTCXO().
  // setDio2AsRfSwitch() forces the DIO2-as-RF-switch opcode on/off
  // independently of the compile-time DIO2_AS_RF_SWITCH macro.
  void setTcxoVoltage(uint8_t mode_byte) override;
  void setDio2AsRfSwitch(bool enable) override;

  void rxAntEnable();
  void loraMode();
  void waitOnBusy();
  void executeOpcode(uint8_t opcode, uint8_t *buffer, uint8_t size);
  void executeOpcodeRead(uint8_t opcode, uint8_t *buffer, uint8_t size);
  void writeBuffer(const uint8_t* buffer, size_t size);
  void readBuffer(uint8_t* buffer, size_t size);
  void setPacketParams(long preamble_symbols, uint8_t headermode, uint8_t payload_length, uint8_t crc);

  void setModulationParams(uint8_t sf, uint8_t bw, uint8_t cr, int ldro);

  // deprecated
  void crc() { enableCrc(); }
  void noCrc() { disableCrc(); }

  byte random();

  void setPins(int ss = LORA_DEFAULT_SS_PIN, int reset = LORA_DEFAULT_RESET_PIN, int dio0 = LORA_DEFAULT_DIO0_PIN, int busy = LORA_DEFAULT_BUSY_PIN, int rxen = LORA_DEFAULT_RXEN_PIN);
  void setSPIFrequency(uint32_t frequency);

  void dumpRegisters(Stream& out);
  void handleDio0IfPending() override;

private:
  void explicitHeaderMode();
  void implicitHeaderMode();

  void handleDio0Rise();

  uint8_t readRegister(uint16_t address);
  void writeRegister(uint16_t address, uint8_t value);
  uint8_t singleTransfer(uint8_t opcode, uint16_t address, uint8_t value);

  static void onDio0Rise();

  void handleLowDataRate();
  void optimizeModemSensitivity();

  void calibrate(void);
  void calibrate_image(long frequency);

private:
  SPISettings _spiSettings;
  int _ss;
  int _reset;
  int _dio0;
  int _rxen;
  int _busy;
  long _frequency;
  int _txp;
  uint8_t _sf;
  uint8_t _bw;
  uint8_t _cr;
  uint8_t _ldro;
  int _packetIndex;
  int _preambleLength;
  int _implicitHeaderMode;
  int _payloadLength;
  int _crcMode;
  int _fifo_tx_addr_ptr;
  int _fifo_rx_addr_ptr;
  uint8_t _packet[255];
  bool _preinit_done;
  volatile bool _dio0_pending;
  void (*_onReceive)(int);

  // Runtime override state. _tcxoVoltageOverride defaults to 0xFF (= "no
  // override; use per-board default"); 0x00 is a valid mode byte so we
  // can't reuse it as a sentinel. _dio2_as_rf_switch_override pairs with
  // _dio2_as_rf_switch_set because both true and false are valid values.
  uint8_t _tcxoVoltageOverride;
  bool _dio2_as_rf_switch_override;
  bool _dio2_as_rf_switch_set;
};

extern sx126x sx126x_modem;

#endif
