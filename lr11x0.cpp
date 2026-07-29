// lr11x0.cpp - LoRa driver for the Semtech LR11x0 family (LR1110/LR1120/LR1121).
//
// Implements the ILoRaRadio interface against the LR11x0 SPI command set.
// See lr11x0.h for the protocol overview. The public API mirrors sx126x
// one-for-one so the firmware's radio-dispatch code can treat it uniformly.
//
// Ported from the RadioLib LR11x0 reference (jgromes/RadioLib) and the
// MeshCore CustomLR1110 wrapper (Seeed T1000-E).
//
// Licensed under the MIT license.

#include "Boards.h"

#if MODEM == LR1110
#include "lr11x0.h"
#include <math.h>

#if MCU_VARIANT == MCU_ESP32
  #include "driver/gpio.h"
  #define ISR_VECT IRAM_ATTR
#else
  #define ISR_VECT
#endif

// LR11x0 SPI commands (16-bit, MSB first).
#define LR11X0_CMD_NOP                0x0000
#define LR11X0_CMD_GET_STATUS         0x0100
#define LR11X0_CMD_GET_VERSION        0x0101
#define LR11X0_CMD_CLEAR_ERRORS       0x010E
#define LR11X0_CMD_CALIBRATE          0x010F
#define LR11X0_CMD_SET_REG_MODE       0x0110
#define LR11X0_CMD_CALIB_IMAGE        0x0111
#define LR11X0_CMD_SET_DIO_AS_RF_SWITCH 0x0112
#define LR11X0_CMD_SET_DIO_IRQ_PARAMS  0x0113
#define LR11X0_CMD_CLEAR_IRQ          0x0114
#define LR11X0_CMD_SET_TCXO_MODE      0x0117
#define LR11X0_CMD_SET_SLEEP          0x011B
#define LR11X0_CMD_SET_STANDBY        0x011C
#define LR11X0_CMD_GET_RX_BUFFER_STATUS 0x0203
#define LR11X0_CMD_GET_PACKET_STATUS  0x0204
#define LR11X0_CMD_GET_RSSI_INST      0x0205
#define LR11X0_CMD_SET_RX             0x0209
#define LR11X0_CMD_SET_TX             0x020A
#define LR11X0_CMD_SET_RF_FREQUENCY   0x020B
#define LR11X0_CMD_SET_RX_TX_FALLBACK_MODE 0x0213
#define LR11X0_CMD_SET_PACKET_TYPE    0x020E
#define LR11X0_CMD_SET_MODULATION_PARAMS 0x020F
#define LR11X0_CMD_SET_PACKET_PARAMS  0x0210
#define LR11X0_CMD_SET_TX_PARAMS      0x0211
#define LR11X0_CMD_SET_PA_CONFIG      0x0215
#define LR11X0_CMD_SET_RX_BOOSTED     0x0227
#define LR11X0_CMD_SET_LORA_SYNC_WORD 0x022B
#define LR11X0_CMD_WRITE_BUFFER       0x0109
#define LR11X0_CMD_READ_BUFFER        0x010A

// Packet types.
#define LR11X0_PACKET_TYPE_LORA       0x02

// LoRa bandwidth codes.
#define LR11X0_LORA_BW_62_5           0x03
#define LR11X0_LORA_BW_125_0          0x04
#define LR11X0_LORA_BW_250_0          0x05
#define LR11X0_LORA_BW_500_0          0x06

// Coding rate (short interleaver).
#define LR11X0_LORA_CR_4_5            0x01
#define LR11X0_LORA_CR_4_6            0x02
#define LR11X0_LORA_CR_4_7            0x03
#define LR11X0_LORA_CR_4_8            0x04

// Low data rate optimization.
#define LR11X0_LORA_LDRO_DISABLED     0x00
#define LR11X0_LORA_LDRO_ENABLED      0x01

// Header / CRC.
#define LR11X0_LORA_HEADER_EXPLICIT   0x00
#define LR11X0_LORA_HEADER_IMPLICIT   0x01
#define LR11X0_LORA_CRC_DISABLED      0x00
#define LR11X0_LORA_CRC_ENABLED       0x01

// Sync word.
#define LR11X0_LORA_SYNC_WORD_PRIVATE 0x12

// Misc.
#define LR11X0_PACKET_TYPE_LORA_VAL   0x02
#define LR11X0_STDBY_RC               0x00
#define LR11X0_REG_MODE_LDO           0x00
#define LR11X0_FALLBACK_STBY_RC       0x01
#define LR11X0_PA_RAMP_48U            0x02
#define LR11X0_CALIBRATE_ALL          0x3F
#define LR11X0_TCXO_1_6V              0x00

// IRQ bits.
#define LR11X0_IRQ_TX_DONE            (1UL << 2)
#define LR11X0_IRQ_RX_DONE            (1UL << 3)
#define LR11X0_IRQ_PREAMBLE_DETECTED  (1UL << 4)
#define LR11X0_IRQ_SYNC_WORD_HEADER_VALID (1UL << 5)
#define LR11X0_IRQ_HEADER_ERR         (1UL << 6)
#define LR11X0_IRQ_CRC_ERR            (1UL << 7)
#define LR11X0_IRQ_TIMEOUT            (1UL << 10)
#define LR11X0_IRQ_ALL                0x1BF80FFCUL

// Status register: GET_VERSION returns [hw, device, fwMajor, fwMinor].
#define LR11X0_DEVICE_LR1110          0x01

// SPI / PLL constants. Frequency is rfFreq = freq_Hz (the LR11x0 takes the
// frequency directly in Hz — unlike the SX126x which divides by FREQ_STEP).
// (RadioLib: setRfFrequency((uint32_t)(freq*1000000.0f)).)
#define LR11X0_MAX_PKT_LENGTH         255
#define LR11X0_RX_TIMEOUT_INF         0xFFFFFFUL
#define LR11X0_TX_TIMEOUT_NONE        0x000000UL

#if defined(NRF52840_XXAA)
  extern SPIClass spiModem;
  #define SPI spiModem
#endif

extern SPIClass SPI;

#define MAX_PKT_LENGTH 255

// Preamble/header timing shared with the firmware's CSMA/airtime logic.
extern long lora_preamble_time_ms;
extern long lora_header_time_ms;
extern bool lora_low_datarate;

lr11x0::lr11x0() :
  _spiSettings(8E6, MSBFIRST, SPI_MODE0),
  _ss(LORA_DEFAULT_SS_PIN),
  _reset(LORA_DEFAULT_RESET_PIN),
  _dio0(LORA_DEFAULT_DIO0_PIN),
  _busy(LORA_DEFAULT_BUSY_PIN),
  _frequency(0),
  _txp(0),
  _sf(0x07),
  _bw(LR11X0_LORA_BW_125_0),
  _cr(LR11X0_LORA_CR_4_5),
  _ldro(0x00),
  _packetIndex(0),
  _preambleLength(18),
  _implicitHeaderMode(0),
  _payloadLength(255),
  _crcMode(1),
  _rxBufOffset(0),
  _packet{0},
  _txLen(0),
  _preinit_done(false),
  _dio0_pending(false),
  _onReceive(NULL)
{ setTimeout(0); }

// ---- Low-level SPI primitives ------------------------------------------------

void lr11x0::waitOnBusy() {
  if (_busy == -1) { return; }
  unsigned long start = millis();
  #if MCU_VARIANT == MCU_ESP32
    while (gpio_get_level((gpio_num_t)_busy) == HIGH) {
  #else
    while (digitalRead(_busy) == HIGH) {
  #endif
      if (millis() >= (start + 100)) { break; }
    }
}

// Write command: single transaction [cmd_hi, cmd_lo, data...].
void lr11x0::writeCommand(uint16_t cmd, const uint8_t *data, uint8_t len) {
  waitOnBusy();
  digitalWrite(_ss, LOW);
  SPI.beginTransaction(_spiSettings);
  SPI.transfer(cmd >> 8);
  SPI.transfer(cmd & 0xFF);
  for (uint8_t i = 0; i < len; i++) { SPI.transfer(data[i]); }
  SPI.endTransaction();
  digitalWrite(_ss, HIGH);
}

// Read command: two transactions.
//   phase 1: [cmd_hi, cmd_lo, request...]
//   phase 2: clock out [status_byte, data...]  (status discarded)
void lr11x0::readCommand(uint16_t cmd, const uint8_t *req, uint8_t reqlen, uint8_t *out, uint8_t outlen) {
  waitOnBusy();
  digitalWrite(_ss, LOW);
  SPI.beginTransaction(_spiSettings);
  SPI.transfer(cmd >> 8);
  SPI.transfer(cmd & 0xFF);
  for (uint8_t i = 0; i < reqlen; i++) { SPI.transfer(req[i]); }
  SPI.endTransaction();
  digitalWrite(_ss, HIGH);

  waitOnBusy();

  digitalWrite(_ss, LOW);
  SPI.beginTransaction(_spiSettings);
  SPI.transfer(0x00);  // status byte slot
  for (uint8_t i = 0; i < outlen; i++) { out[i] = SPI.transfer(0x00); }
  SPI.endTransaction();
  digitalWrite(_ss, HIGH);
}

// Poll the always-available status frame: 6 bytes [stat1, stat2, irq(4 BE)].
// No command byte is sent — the chip streams status while NSS is low.
void lr11x0::readStatusFrame(uint8_t *out6) {
  waitOnBusy();
  digitalWrite(_ss, LOW);
  SPI.beginTransaction(_spiSettings);
  for (uint8_t i = 0; i < 6; i++) { out6[i] = SPI.transfer(0x00); }
  SPI.endTransaction();
  digitalWrite(_ss, HIGH);
}

uint16_t lr11x0::getIrqStatus() {
  uint8_t buf[6];
  readStatusFrame(buf);
  // The status frame is [stat1, stat2, irq_b3, irq_b2, irq_b1, irq_b0]
  // (IRQ big-endian). Every IRQ bit the firmware acts on lives in the low
  // 16 bits (bits 2..11), so return just those: byte 4 (15..8) | byte 5 (7..0).
  return (uint16_t)((((uint16_t)buf[4] << 8) | (uint16_t)buf[5]));
}

void lr11x0::setDioIrqParams(uint32_t irq, uint32_t dioMask) {
  uint8_t buf[8] = {
    (uint8_t)(irq >> 24), (uint8_t)(irq >> 16), (uint8_t)(irq >> 8), (uint8_t)(irq),
    (uint8_t)(dioMask >> 24), (uint8_t)(dioMask >> 16), (uint8_t)(dioMask >> 8), (uint8_t)(dioMask),
  };
  writeCommand(LR11X0_CMD_SET_DIO_IRQ_PARAMS, buf, 8);
}

void lr11x0::clearIrq(uint32_t irq) {
  uint8_t buf[4] = { (uint8_t)(irq >> 24), (uint8_t)(irq >> 16), (uint8_t)(irq >> 8), (uint8_t)(irq) };
  writeCommand(LR11X0_CMD_CLEAR_IRQ, buf, 4);
}

void lr11x0::setPacketType(uint8_t type) {
  writeCommand(LR11X0_CMD_SET_PACKET_TYPE, &type, 1);
}

void lr11x0::setModulationParams(uint8_t sf, uint8_t bw, uint8_t cr, uint8_t ldro) {
  uint8_t buf[4] = { sf, bw, cr, ldro };
  writeCommand(LR11X0_CMD_SET_MODULATION_PARAMS, buf, 4);
}

void lr11x0::setPacketParams(uint16_t preambleLen, uint8_t hdrType, uint8_t payloadLen, uint8_t crcType) {
  uint8_t buf[6] = {
    (uint8_t)(preambleLen >> 8), (uint8_t)(preambleLen & 0xFF),
    hdrType, payloadLen, crcType, 0x00  // standard IQ (no inversion)
  };
  writeCommand(LR11X0_CMD_SET_PACKET_PARAMS, buf, 6);
}

void lr11x0::setRfFrequency(uint32_t frf) {
  uint8_t buf[4] = { (uint8_t)(frf >> 24), (uint8_t)(frf >> 16), (uint8_t)(frf >> 8), (uint8_t)(frf) };
  writeCommand(LR11X0_CMD_SET_RF_FREQUENCY, buf, 4);
}

void lr11x0::setPaConfig(uint8_t paSel, uint8_t regPaSupply, uint8_t paDutyCycle, uint8_t paHpSel) {
  uint8_t buf[4] = { paSel, regPaSupply, paDutyCycle, paHpSel };
  writeCommand(LR11X0_CMD_SET_PA_CONFIG, buf, 4);
}

void lr11x0::setTxParams(int8_t pwr, uint8_t ramp) {
  uint8_t buf[2] = { (uint8_t)pwr, ramp };
  writeCommand(LR11X0_CMD_SET_TX_PARAMS, buf, 2);
}

void lr11x0::setTcxoMode(uint8_t tune, uint32_t delay) {
  uint8_t buf[4] = { tune, (uint8_t)(delay >> 16), (uint8_t)(delay >> 8), (uint8_t)(delay) };
  writeCommand(LR11X0_CMD_SET_TCXO_MODE, buf, 4);
}

void lr11x0::setRegMode(uint8_t mode) {
  writeCommand(LR11X0_CMD_SET_REG_MODE, &mode, 1);
}

void lr11x0::setStandby(uint8_t mode) {
  writeCommand(LR11X0_CMD_SET_STANDBY, &mode, 1);
}

void lr11x0::calibrate(uint8_t params) {
  writeCommand(LR11X0_CMD_CALIBRATE, &params, 1);
}

void lr11x0::calibrateImageRejection(uint32_t frequency) {
  // LR11x0 takes two bytes: freqMin and freqMax in units of ~4 MHz.
  // RadioLib: floor((freqMin-1)/4), ceil((freqMax+1)/4) with the band ±band MHz.
  float mhz = frequency / 1.0e6f;
  uint8_t lo = (uint8_t)floor((mhz - 5.0f) / 4.0f);
  uint8_t hi = (uint8_t)ceil((mhz + 5.0f) / 4.0f);
  uint8_t buf[2] = { lo, hi };
  writeCommand(LR11X0_CMD_CALIB_IMAGE, buf, 2);
}

void lr11x0::setRxBoostedGainMode(bool en) {
  uint8_t buf[1] = { (uint8_t)en };
  writeCommand(LR11X0_CMD_SET_RX_BOOSTED, buf, 1);
}

// ---- Init --------------------------------------------------------------------

bool lr11x0::preInit() {
  pinMode(_ss, OUTPUT);
  digitalWrite(_ss, HIGH);

  #if defined(NRF52840_XXAA)
    SPI.setPins(pin_miso, pin_sclk, pin_mosi);
    SPI.begin();
  #else
    SPI.begin();
  #endif

  // Reset the chip so it lands in a known state.
  reset();

  // Read the version and confirm an LR1110 is present.
  uint8_t ver[4] = { 0 };
  readCommand(LR11X0_CMD_GET_VERSION, NULL, 0, ver, 4);
  // ver = [hw, device, fwMajor, fwMinor]; device 0x01 == LR1110.
  if (ver[1] != LR11X0_DEVICE_LR1110) {
    return false;
  }

  _preinit_done = true;
  return true;
}

void lr11x0::enableTcxo() {
  #if defined(LR11X0_TCXO_TUNE)
    // TCXO wakeup delay register unit is ~30.52 us (RadioLib: delay/30.52).
    uint32_t delayUs = LR11X0_TCXO_DELAY_US;
    uint32_t delayVal = (uint32_t)((float)delayUs / 30.52f);
    if (delayVal == 0) { delayVal = 1; }
    setTcxoMode(LR11X0_TCXO_TUNE, delayVal);
  #endif
}

void lr11x0::configRfSwitch() {
  #if defined(LR11X0_RFSWITCH_ENABLE)
    // The board (Boards.h) supplies the 8-byte SET_DIO_AS_RF_SWITCH payload:
    // enable bitmask followed by 7 per-mode DIO drive configs
    // (stby, rx, tx, tx_hp, tx_hf, gnss, wifi).
    uint8_t buf[8] = {
      LR11X0_RFSWITCH_ENABLE,
      LR11X0_RFSW_STBY, LR11X0_RFSW_RX, LR11X0_RFSW_TX,
      LR11X0_RFSW_TX_HP, LR11X0_RFSW_TX_HF, LR11X0_RFSW_GNSS, LR11X0_RFSW_WIFI
    };
    writeCommand(LR11X0_CMD_SET_DIO_AS_RF_SWITCH, buf, 8);
  #endif
}

int lr11x0::begin(uint32_t frequency) {
  if (_busy != -1) { pinMode(_busy, INPUT); }
  if (!_preinit_done) { if (!preInit()) { return false; } }

  // Clear any pending errors and IRQs from a prior session.
  writeCommand(LR11X0_CMD_CLEAR_ERRORS, NULL, 0);
  clearIrq(LR11X0_IRQ_ALL);

  // Standby (RC) is required before most configuration commands.
  setStandby(LR11X0_STDBY_RC);

  // Regulator mode: LDO (matches the RadioLib/MeshCore default path).
  setRegMode(LR11X0_REG_MODE_LDO);

  // Enable TCXO (DIO3) if the board defines a voltage.
  enableTcxo();

  // RX/Tx fallback to STDBY_RC after each transaction.
  uint8_t fb = LR11X0_FALLBACK_STBY_RC;
  writeCommand(LR11X0_CMD_SET_RX_TX_FALLBACK_MODE, &fb, 1);

  // Route the on-chip RF switch (DIO5..DIO8) if the board has one.
  configRfSwitch();

  // Select LoRa packet type.
  setPacketType(LR11X0_PACKET_TYPE_LORA);

  // Calibrate all blocks, then image rejection for the operating band.
  calibrate(LR11X0_CALIBRATE_ALL);
  delay(5);
  waitOnBusy();
  calibrateImageRejection(frequency);

  // Private LoRa network sync word (0x12) — matches RNode/SX126x default.
  uint8_t sw = LR11X0_LORA_SYNC_WORD_PRIVATE;
  writeCommand(LR11X0_CMD_SET_LORA_SYNC_WORD, &sw, 1);

  setFrequency(frequency);
  setTxPower(2);
  enableCrc();

  // RX boosted gain improves sensitivity; the T1000-E front end expects it.
  #if defined(LR11X0_RX_BOOSTED)
    setRxBoostedGainMode(LR11X0_RX_BOOSTED);
  #endif

  setModulationParams(_sf, _bw, _cr, _ldro);
  setPacketParams(_preambleLength, _implicitHeaderMode, _payloadLength, _crcMode);

  return 1;
}

void lr11x0::end() { sleep(); SPI.end(); _preinit_done = false; }

void lr11x0::reset(void) {
  if (_reset != -1) {
    pinMode(_reset, OUTPUT);
    digitalWrite(_reset, LOW);
    delay(10);
    digitalWrite(_reset, HIGH);
    // The LR1110 typically needs ~273 ms after NRESET release.
    delay(300);
  }
}

void lr11x0::standby() { setStandby(LR11X0_STDBY_RC); }

void lr11x0::sleep() {
  // retainConfig=1 (bit0), sleepTime=0. Keep config so a wake NSS toggle
  // restores the radio without a full re-init.
  uint8_t buf[5] = { 0x01, 0x00, 0x00, 0x00, 0x00 };
  writeCommand(LR11X0_CMD_SET_SLEEP, buf, 5);
}

// ---- PHY parameter setters ---------------------------------------------------

void lr11x0::setFrequency(uint32_t frequency) {
  _frequency = frequency;
  setRfFrequency(frequency);  // LR11x0 takes the frequency directly in Hz.
}

uint32_t lr11x0::getFrequency() { return _frequency; }

void lr11x0::setSpreadingFactor(int sf) {
  if (sf < 5) { sf = 5; }
  else if (sf > 12) { sf = 12; }
  _sf = sf;
  handleLowDataRate();
  setModulationParams(_sf, _bw, _cr, _ldro);
}

uint32_t lr11x0::getSignalBandwidth() {
  switch (_bw) {
    case LR11X0_LORA_BW_62_5:  return 62500;
    case LR11X0_LORA_BW_125_0: return 125000;
    case LR11X0_LORA_BW_250_0: return 250000;
    case LR11X0_LORA_BW_500_0: return 500000;
  }
  return 0;
}

void lr11x0::handleLowDataRate() {
  // Enable LDRO when the symbol time exceeds 16 ms (RadioLib rule).
  if (long((1UL << _sf) / (getSignalBandwidth() / 1000)) > 16) {
    _ldro = LR11X0_LORA_LDRO_ENABLED; lora_low_datarate = true;
  } else {
    _ldro = LR11X0_LORA_LDRO_DISABLED; lora_low_datarate = false;
  }
}

void lr11x0::setSignalBandwidth(uint32_t sbw) {
  if      (sbw <= 62500)  { _bw = LR11X0_LORA_BW_62_5; }
  else if (sbw <= 125000) { _bw = LR11X0_LORA_BW_125_0; }
  else if (sbw <= 250000) { _bw = LR11X0_LORA_BW_250_0; }
  else                    { _bw = LR11X0_LORA_BW_500_0; }
  handleLowDataRate();
  setModulationParams(_sf, _bw, _cr, _ldro);
}

void lr11x0::setCodingRate4(int denominator) {
  if (denominator < 5) { denominator = 5; }
  else if (denominator > 8) { denominator = 8; }
  // LR11x0 short-interleaver codes: 4/5=0x01 .. 4/8=0x04.
  _cr = (uint8_t)(denominator - 4);
  setModulationParams(_sf, _bw, _cr, _ldro);
}

void lr11x0::setPreambleLength(long preamble_symbols) {
  _preambleLength = preamble_symbols;
  setPacketParams(_preambleLength, _implicitHeaderMode, _payloadLength, _crcMode);
}

void lr11x0::enableCrc()  { _crcMode = 1; setPacketParams(_preambleLength, _implicitHeaderMode, _payloadLength, LR11X0_LORA_CRC_ENABLED); }
void lr11x0::disableCrc() { _crcMode = 0; setPacketParams(_preambleLength, _implicitHeaderMode, _payloadLength, LR11X0_LORA_CRC_DISABLED); }
void lr11x0::explicitHeaderMode() { _implicitHeaderMode = LR11X0_LORA_HEADER_EXPLICIT; setPacketParams(_preambleLength, _implicitHeaderMode, _payloadLength, _crcMode); }
void lr11x0::implicitHeaderMode() { _implicitHeaderMode = LR11X0_LORA_HEADER_IMPLICIT; setPacketParams(_preambleLength, _implicitHeaderMode, _payloadLength, _crcMode); }

void lr11x0::setTxPower(int level, int outputPin) {
  (void)outputPin;  // PA selection is internal on the LR11x0.

  if (level > 22) { level = 22; }
  else if (level < -9) { level = -9; }

  // High-power PA for >14 dBm, low-power PA otherwise (RadioLib LR1110 rule).
  bool useHp = (level > 14);
  setPaConfig((uint8_t)useHp, (uint8_t)useHp, 0x04, 0x07);
  setTxParams((int8_t)level, LR11X0_PA_RAMP_48U);
  _txp = level;
}

uint8_t lr11x0::getTxPower() { return _txp; }

void lr11x0::setPins(int ss, int reset, int dio0, int busy) {
  _ss = ss;
  _reset = reset;
  _dio0 = dio0;
  _busy = busy;
}

void lr11x0::setSPIFrequency(uint32_t frequency) { _spiSettings = SPISettings(frequency, MSBFIRST, SPI_MODE0); }

byte lr11x0::random() {
  // GET_RSSI_INST doubles as a quick SPI round-trip; for true entropy the
  // LR11x0 has a dedicated RNG opcode, but the firmware only uses this for
  // a per-packet random header nibble where any varying byte suffices.
  uint8_t b = 0;
  uint8_t irq[6];
  readStatusFrame(irq);
  b = irq[5] ^ irq[2] ^ (uint8_t)micros();
  return b;
}

// ---- TX ----------------------------------------------------------------------

int lr11x0::beginPacket(int implicitHeader) {
  standby();
  if (implicitHeader) { implicitHeaderMode(); }
  else { explicitHeaderMode(); }

  _payloadLength = 0;
  _txLen = 0;
  return 1;
}

size_t lr11x0::write(uint8_t byte) { return write(&byte, sizeof(byte)); }

size_t lr11x0::write(const uint8_t *buffer, size_t size) {
  // The firmware writes packets a byte at a time; the LR11x0 WRITE_BUFFER
  // command has no auto-increment address exposed via this driver's API, so
  // stage the whole payload here and flush it in endPacket().
  for (size_t i = 0; i < size; i++) {
    if (_payloadLength >= MAX_PKT_LENGTH) { break; }
    _txBuf[_payloadLength++] = buffer[i];
  }
  _txLen = _payloadLength;
  return size;
}

int lr11x0::endPacket() {
  // Commit the staged payload length, then write the whole buffer in one go.
  setPacketParams(_preambleLength, _implicitHeaderMode, _txLen, _crcMode);

  waitOnBusy();
  digitalWrite(_ss, LOW);
  SPI.beginTransaction(_spiSettings);
  SPI.transfer(LR11X0_CMD_WRITE_BUFFER >> 8);
  SPI.transfer(LR11X0_CMD_WRITE_BUFFER & 0xFF);
  for (uint8_t i = 0; i < _txLen; i++) { SPI.transfer(_txBuf[i]); }
  SPI.endTransaction();
  digitalWrite(_ss, HIGH);

  setDioIrqParams(LR11X0_IRQ_TX_DONE | LR11X0_IRQ_TIMEOUT,
                  LR11X0_IRQ_TX_DONE | LR11X0_IRQ_TIMEOUT);
  clearIrq(LR11X0_IRQ_ALL);

  // Single TX mode with no timeout (the firmware polls the IRQ).
  uint8_t timeout[3] = { 0, 0, 0 };
  writeCommand(LR11X0_CMD_SET_TX, timeout, 3);

  // Wait for BUSY to drop (PA ramp-up done), then poll TX_DONE.
  waitOnBusy();
  uint32_t w_timeout = millis() + LORA_MODEM_TIMEOUT_MS;
  bool timed_out = true;
  while (millis() < w_timeout) {
    if (getIrqStatus() & LR11X0_IRQ_TX_DONE) { timed_out = false; break; }
    yield();
  }
  clearIrq(LR11X0_IRQ_ALL);
  standby();
  return timed_out ? 0 : 1;
}

// ---- RX ----------------------------------------------------------------------

void lr11x0::onReceive(void(*callback)(int)) {
  _onReceive = callback;
  if (callback) {
    pinMode(_dio0, INPUT);
    // Route RX_DONE (+ timeout) to the IRQ/DIO1 pin.
    setDioIrqParams(LR11X0_IRQ_RX_DONE | LR11X0_IRQ_TIMEOUT,
                    LR11X0_IRQ_RX_DONE | LR11X0_IRQ_TIMEOUT);
    #if MCU_VARIANT != MCU_ESP32 && MCU_VARIANT != MCU_NRF52 && MCU_VARIANT != MCU_NATIVE
      #ifdef SPI_HAS_NOTUSINGINTERRUPT
        SPI.usingInterrupt(digitalPinToInterrupt(_dio0));
      #endif
    #endif
    attachInterrupt(digitalPinToInterrupt(_dio0), lr11x0::onDio0Rise, RISING);
  } else {
    detachInterrupt(digitalPinToInterrupt(_dio0));
    #if MCU_VARIANT != MCU_ESP32 && MCU_VARIANT != MCU_NRF52 && MCU_VARIANT != MCU_NATIVE
      #ifdef SPI_HAS_NOTUSINGINTERRUPT
        SPI.notUsingInterrupt(digitalPinToInterrupt(_dio0));
      #endif
    #endif
  }
}

void lr11x0::receive(int size) {
  if (size > 0) {
    implicitHeaderMode();
    _payloadLength = size;
    setPacketParams(_preambleLength, _implicitHeaderMode, (uint8_t)_payloadLength, _crcMode);
  } else {
    explicitHeaderMode();
  }

  setDioIrqParams(LR11X0_IRQ_RX_DONE | LR11X0_IRQ_TIMEOUT,
                  LR11X0_IRQ_RX_DONE | LR11X0_IRQ_TIMEOUT);
  clearIrq(LR11X0_IRQ_ALL);

  // Continuous RX (infinite timeout).
  uint8_t timeout[3] = { 0xFF, 0xFF, 0xFF };
  writeCommand(LR11X0_CMD_SET_RX, timeout, 3);
}

int ISR_VECT lr11x0::available() {
  uint8_t buf[2] = { 0 };
  readCommand(LR11X0_CMD_GET_RX_BUFFER_STATUS, NULL, 0, buf, 2);
  _rxBufOffset = buf[1];
  return buf[0] - _packetIndex;
}

int ISR_VECT lr11x0::read() {
  if (!available()) { return -1; }
  if (_packetIndex == 0) {
    uint8_t status[2] = { 0 };
    readCommand(LR11X0_CMD_GET_RX_BUFFER_STATUS, NULL, 0, status, 2);
    int pktLen = status[0];
    _rxBufOffset = status[1];
    // READ_BUFFER request: [offset, length], then read `length` payload bytes.
    uint8_t req[2] = { _rxBufOffset, (uint8_t)pktLen };
    readCommand(LR11X0_CMD_READ_BUFFER, req, 2, _packet, (uint8_t)pktLen);
  }
  uint8_t byte = _packet[_packetIndex];
  _packetIndex++;
  return byte;
}

int lr11x0::peek() {
  if (!available()) { return -1; }
  if (_packetIndex == 0) {
    uint8_t status[2] = { 0 };
    readCommand(LR11X0_CMD_GET_RX_BUFFER_STATUS, NULL, 0, status, 2);
    int pktLen = status[0];
    _rxBufOffset = status[1];
    uint8_t req[2] = { _rxBufOffset, (uint8_t)pktLen };
    readCommand(LR11X0_CMD_READ_BUFFER, req, 2, _packet, (uint8_t)pktLen);
  }
  return _packet[_packetIndex];
}

void lr11x0::flush() { }

// ---- Signal metrics ----------------------------------------------------------

int ISR_VECT lr11x0::currentRssi() {
  uint8_t b = 0;
  readCommand(LR11X0_CMD_GET_RSSI_INST, NULL, 0, &b, 1);
  return -(int)b / 2;
}

int ISR_VECT lr11x0::packetRssi() {
  uint8_t buf[3] = { 0 };
  readCommand(LR11X0_CMD_GET_PACKET_STATUS, NULL, 0, buf, 3);
  return -(int)buf[0] / 2;
}

int ISR_VECT lr11x0::packetRssi(uint8_t pkt_snr_raw) {
  (void)pkt_snr_raw;
  return packetRssi();
}

uint8_t ISR_VECT lr11x0::packetSnrRaw() {
  uint8_t buf[3] = { 0 };
  readCommand(LR11X0_CMD_GET_PACKET_STATUS, NULL, 0, buf, 3);
  return buf[1];
}

float lr11x0::packetSnr() {
  uint8_t buf[3] = { 0 };
  readCommand(LR11X0_CMD_GET_PACKET_STATUS, NULL, 0, buf, 3);
  return (float)((int8_t)buf[1]) * 0.25;
}

// ---- DCD / carrier detect ----------------------------------------------------

bool lr11x0::dcd() {
  // The LR11x0 exposes preamble-detect and header-valid as IRQ bits, so DCD
  // mirrors the sx126x logic using those flags.
  uint16_t irq = getIrqStatus();
  uint32_t now = millis();

  bool carrier_detected = false;

  if ((irq & LR11X0_IRQ_SYNC_WORD_HEADER_VALID) != 0) { carrier_detected = true; }
  if ((irq & LR11X0_IRQ_PREAMBLE_DETECTED) != 0) {
    carrier_detected = true;
    // A preamble-detect latch without a following header-valid within the
    // expected window is a false alarm — clear it so DCD can drop.
    static unsigned long preamble_detected_at = 0;
    if (preamble_detected_at == 0) { preamble_detected_at = now; }
    if (now - preamble_detected_at > lora_preamble_time_ms + lora_header_time_ms) {
      preamble_detected_at = 0;
      clearIrq(LR11X0_IRQ_PREAMBLE_DETECTED);
    }
  }
  return carrier_detected;
}

// ---- DIO1 interrupt ----------------------------------------------------------

void lr11x0::handleDio0IfPending() {
  if (_dio0_pending) {
    _dio0_pending = false;
    handleDio0Rise();
  }
}

void ISR_VECT lr11x0::handleDio0Rise() {
  uint16_t irq = getIrqStatus();
  clearIrq(LR11X0_IRQ_ALL);

  // Only deliver a packet on RX_DONE with no CRC error.
  if ((irq & LR11X0_IRQ_RX_DONE) &&
      ((irq & LR11X0_IRQ_CRC_ERR) == 0) &&
      ((irq & LR11X0_IRQ_HEADER_ERR) == 0)) {
    _packetIndex = 0;
    uint8_t status[2] = { 0 };
    readCommand(LR11X0_CMD_GET_RX_BUFFER_STATUS, NULL, 0, status, 2);
    int packetLength = status[0];
    if (_onReceive) { _onReceive(packetLength); }
  } else if (irq & LR11X0_IRQ_HEADER_ERR) {
    // LR1110 erratum: a corrupted header can shift subsequent packets; a
    // standby() returns the radio to a known-good state (MeshCore workaround).
    standby();
  }

  // Re-arm continuous RX. The LR11x0 drops back to its Rx/Tx fallback mode
  // (STDBY_RC) after RX_DONE, so unlike the SX126x it does not auto-restart
  // the receiver. Issuing SET_RX again here keeps it listening for the next
  // packet, matching the firmware's expectation of an always-on RX.
  uint8_t timeout[3] = { 0xFF, 0xFF, 0xFF };
  writeCommand(LR11X0_CMD_SET_RX, timeout, 3);
}

void ISR_VECT lr11x0::onDio0Rise() {
  #if MCU_VARIANT == MCU_ESP32 || MCU_VARIANT == MCU_NRF52 || MCU_VARIANT == MCU_NATIVE
    lr11x0_modem._dio0_pending = true;
  #else
    lr11x0_modem.handleDio0Rise();
  #endif
}

lr11x0 lr11x0_modem;

#endif
