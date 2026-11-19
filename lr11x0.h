// lr11x0.h - LoRa driver for the Semtech LR11x0 family (LR1110/LR1120/LR1121).
//
// This driver implements the same ILoRaRadio interface as sx126x/sx127x/sx128x
// so it slots into the existing RNode firmware radio-dispatch machinery. The
// SPI command set and frame layout follow the LR11x0 datasheet (commands are
// 16-bit; reads use a two-phase transaction: send command, then clock out a
// status byte followed by the payload).
//
// Although the LR11x0 is a distinct silicon family from the SX126x, its LoRa
// packet engine is conceptually identical (modulation params, packet params,
// DIO-driven IRQ), so the public API mirrors sx126x one-for-one.
//
// Ported from the RadioLib LR11x0 reference (jgromes/RadioLib) and the
// MeshCore CustomLR1110 wrapper (which drives the Seeed T1000-E).
//
// Licensed under the MIT license.

#ifndef LR11X0_H
#define LR11X0_H

#include <Arduino.h>
#include <SPI.h>
#include "Modem.h"
#include "LoRaRadio.h"

#define LORA_DEFAULT_SS_PIN    10
#define LORA_DEFAULT_RESET_PIN 9
#define LORA_DEFAULT_DIO0_PIN  2
#define LORA_DEFAULT_BUSY_PIN  -1
#define LORA_MODEM_TIMEOUT_MS 20E3

// PA output pin selectors (kept for ILoRaRadio setTxPower() signature
// compatibility; the LR11x0 has an internal high-power PA selected via
// setPaConfig rather than a pin).
#define PA_OUTPUT_RFO_PIN      0
#define PA_OUTPUT_PA_BOOST_PIN 1

// RSSI scaling on the LR11x0: register value v maps to -v/2 dBm.
#define RSSI_OFFSET 157

class lr11x0 : public ILoRaRadio {
public:
  lr11x0();

  int begin(uint32_t frequency) override;
  void end() override;

  int beginPacket(int implicitHeader = false) override;
  int endPacket() override;

  int packetRssi() override;
  int packetRssi(uint8_t pkt_snr_raw) override;
  int currentRssi() override;
  uint8_t packetSnrRaw() override;
  float packetSnr();

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
  uint8_t getTxPower() override;
  void setTxPower(int level, int outputPin = PA_OUTPUT_PA_BOOST_PIN) override;
  uint32_t getFrequency() override;
  void setFrequency(uint32_t frequency) override;
  void setSpreadingFactor(int sf) override;
  uint32_t getSignalBandwidth() override;
  void setSignalBandwidth(uint32_t sbw) override;
  void setCodingRate4(int denominator) override;
  void setPreambleLength(long preamble_symbols) override;
  bool dcd() override;
  void enableCrc() override;
  void disableCrc();

  // Runtime SX126x-style knobs. The native target calls these on the
  // polymorphic LoRa pointer; on the LR11x0 they are no-ops (TCXO voltage
  // and RF-switch routing are configured at begin() from board macros).
  void setTcxoVoltage(uint8_t mode_byte) override { (void)mode_byte; }
  void setDio2AsRfSwitch(bool enable) override { (void)enable; }

  void waitOnBusy();
  void handleDio0IfPending() override;

  void setPins(int ss = LORA_DEFAULT_SS_PIN, int reset = LORA_DEFAULT_RESET_PIN, int dio0 = LORA_DEFAULT_DIO0_PIN, int busy = LORA_DEFAULT_BUSY_PIN);
  void setSPIFrequency(uint32_t frequency);

  byte random();

private:
  // Low-level SPI primitives implementing the LR11x0 frame protocol.
  //   writeCommand(): single transaction — [cmd_hi, cmd_lo, data...]
  //   readCommand():  two transactions — phase 1 sends [cmd, request...],
  //                   phase 2 clocks out [status_byte, data...].
  void writeCommand(uint16_t cmd, const uint8_t *data, uint8_t len);
  void readCommand(uint16_t cmd, const uint8_t *req, uint8_t reqlen, uint8_t *out, uint8_t outlen);
  // Poll the always-available status frame: returns [stat1, stat2, irq(4 BE)].
  void readStatusFrame(uint8_t *out6);

  uint16_t getIrqStatus();
  void setDioIrqParams(uint32_t irq, uint32_t dioMask);
  void clearIrq(uint32_t irq);

  void setPacketType(uint8_t type);
  void setModulationParams(uint8_t sf, uint8_t bw, uint8_t cr, uint8_t ldro);
  void setPacketParams(uint16_t preambleLen, uint8_t hdrType, uint8_t payloadLen, uint8_t crcType);
  void setRfFrequency(uint32_t frf);
  void setPaConfig(uint8_t paSel, uint8_t regPaSupply, uint8_t paDutyCycle, uint8_t paHpSel);
  void setTxParams(int8_t pwr, uint8_t ramp);
  void setTcxoMode(uint8_t tune, uint32_t delay);
  void setRegMode(uint8_t mode);
  void setStandby(uint8_t mode);
  void calibrate(uint8_t params);
  void calibrateImageRejection(uint32_t frequency);
  void setRxBoostedGainMode(bool en);
  void enableTcxo();
  void configRfSwitch();

  void explicitHeaderMode();
  void implicitHeaderMode();
  void handleLowDataRate();

  void handleDio0Rise();
  static void onDio0Rise();

private:
  SPISettings _spiSettings;
  int _ss;
  int _reset;
  int _dio0;     // LR11x0 IRQ pin (DIO1 on the chip)
  int _busy;
  long _frequency;
  int _txp;
  uint8_t _sf;
  uint8_t _bw;     // LR11x0 bandwidth code
  uint8_t _cr;     // raw LR11x0 coding-rate code
  uint8_t _ldro;
  int _packetIndex;
  int _preambleLength;
  int _implicitHeaderMode;
  int _payloadLength;
  int _crcMode;
  uint8_t _rxBufOffset;   // LR11x0 RX buffer start offset (moves per packet)
  uint8_t _packet[255];
  uint8_t _txBuf[255];    // LoRa TX is staged here (the firmware writes byte-by-byte)
  uint8_t _txLen;
  bool _preinit_done;
  volatile bool _dio0_pending;
  void (*_onReceive)(int);
};

extern lr11x0 lr11x0_modem;

#endif
