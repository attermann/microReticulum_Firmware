// MASN V1.2 / HT-RA62: diagnostic suite, KISS CMD_DETECT subcommands 0x60..0x6C.
// GPL-3.0-or-later, intended for integration in microReticulum_Firmware.
// Only compiled for BOARD_MASN_HTRA62; included after lora_receive() is defined.
#ifndef MASN_DIAGNOSTICS_H
#define MASN_DIAGNOSTICS_H

#if BOARD_MODEL == BOARD_MASN_HTRA62

extern SPIClass spiModem;             // The same bus used in sx126x.cpp.
#include "MASN_Power.h"
static volatile uint32_t masn_diag_rx_callbacks = 0;
static volatile uint32_t masn_diag_rx_bytes = 0;
static uint32_t masn_diag_tx_ok = 0;
static uint32_t masn_diag_tx_fail = 0;

void ISR_VECT receive_callback(int packet_size);  // RNode_Firmware.ino

// After a hardware reset of the SX1262 every setting is lost, including the
// IRQ mask (without it neither TX_DONE nor RX_DONE is ever flagged).
// Re-run the same sequence as startRadio().
static bool masn_diag_restore_radio() {
  bool ok = LoRa->begin(lora_freq);
  setTXPower();
  setBandwidth();
  setSpreadingFactor();
  setCodingRate();
  LoRa->enableCrc();
  LoRa->onReceive(receive_callback);
  lora_receive();
  return ok;
}

static void masn_diag_reply(uint8_t op, const char *message) {
  serial_write(FEND);
  serial_write(op);
  while (*message) {
    // Only ASCII data in these messages, therefore no KISS escaping needed.
    serial_write((uint8_t)*message++);
  }
  serial_write(FEND);
}

// Unlike the old ad-hoc patches, all reads use the *modem* SPI instance,
// a single continuous transaction, a bounded BUSY wait and a correct CS
// assertion order. Return false rather than issuing SPI while BUSY is HIGH.
static bool masn_diag_spi(uint8_t *data, size_t count) {
  if (!radio_online || count == 0 || count > 8) return false;
  const uint32_t start = millis();
  while (digitalRead(pin_busy) == HIGH) {
    if ((uint32_t)(millis() - start) >= 100U) return false;
    delay(1);
  }
  spiModem.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(pin_cs, LOW);
  spiModem.transfer(data, count);
  digitalWrite(pin_cs, HIGH);
  spiModem.endTransaction();
  return true;
}

static bool masn_diag_status(uint8_t &status) {
  uint8_t d[2] = {0xC0, 0x00}; // Semtech GetStatus, then a dummy byte.
  if (!masn_diag_spi(d, sizeof(d))) return false;
  status = d[1];
  return true;
}

static bool masn_diag_reg(uint16_t reg, uint8_t &value) {
  uint8_t d[5] = {0x1D, (uint8_t)(reg >> 8), (uint8_t)reg, 0, 0};
  if (!masn_diag_spi(d, sizeof(d))) return false;
  value = d[4];
  return true;
}

static bool masn_diag_word(uint8_t opcode, uint16_t &result) {
  uint8_t d[4] = {opcode, 0x00, 0x00, 0x00}; // dummy/status + 2 data bytes
  if (!masn_diag_spi(d, sizeof(d))) return false;
  result = ((uint16_t)d[2] << 8) | d[3];
  return true;
}

static bool masn_diag_tx_allowed() {
  // Explicit limits prevent accidentally transmitting on another channel
  // or at the previous diagnostic's higher-power configuration.
  return radio_online && hw_ready && !radio_locked &&
         LoRa->getFrequency() == 869618000UL &&
         LoRa->getTxPower() <= 7;
}

static void masn_diag_cw(uint8_t op, bool disable_rxen) {
  if (!masn_diag_tx_allowed()) {
    masn_diag_reply(op, "ERR=NOT_READY_OR_RF_LIMIT;REQ=869618000Hz,TXP<=7dBm,ONLINE=1");
    return;
  }
  const int saved_rxen = pin_rxen >= 0 ? digitalRead(pin_rxen) : -1;
  uint8_t pre = 0xFF, during1 = 0xFF, during2 = 0xFF, post = 0xFF;
  bool pre_ok, during1_ok, during2_ok, post_ok;
  LoRa->standby();
  pre_ok = masn_diag_status(pre);
  if (disable_rxen && pin_rxen >= 0) digitalWrite(pin_rxen, LOW);
  uint8_t dummy = 0;
  LoRa->executeOpcode(0xD1, &dummy, 0); // SX1262 SetTxContinuousWave
  delay(150);
  during1_ok = masn_diag_status(during1);
  int busy = digitalRead(pin_busy);
  delay(250);
  during2_ok = masn_diag_status(during2);
  delay(200);                         // total CW attempt 600 ms
  LoRa->standby();                    // always terminate before reporting
  if (saved_rxen >= 0) digitalWrite(pin_rxen, saved_rxen);
  lora_receive();                     // restore standard firmware receive path
  post_ok = masn_diag_status(post);
  char out[192];
  snprintf(out, sizeof(out),
           "V=1;CW=600;RXLOW=%d;PRE=%02X;TX1=%02X;TX2=%02X;POST=%02X;"
           "TXMODE1=%u;TXMODE2=%u;BUSY=%d;SPI=%d%d%d%d",
           disable_rxen ? 1 : 0, (unsigned)pre, (unsigned)during1,
           (unsigned)during2, (unsigned)post,
           during1_ok ? (unsigned)((during1 >> 4) & 7) : 255U,
           during2_ok ? (unsigned)((during2 >> 4) & 7) : 255U,
           busy, (int)pre_ok, (int)during1_ok, (int)during2_ok, (int)post_ok);
  masn_diag_reply(op, out);
}

// ---------------------------------------------------------------------------
// 0x68..0x6A: traced oscillator bring-up (no RF emission).
//   0x68 = DIO3 TCXO 1.8 V, 0x69 = DIO3 TCXO 3.3 V, 0x6A = no DIO3 (XTAL).
// Sequence (Semtech DS_SX1261-2 13.3.6 / 9.2):
//   write SYNC=3444 + readback (proves SPI writes) -> hardware reset ->
//   SYNC back to 1424? (proves RESET pin works) -> status/errors ->
//   ClearDeviceErrors -> [SetDIO3AsTCXOCtrl(V, 10 ms)] -> Calibrate(0x7F) ->
//   SetStandby(STDBY_XOSC) -> SetRfFrequency + SetFs (PLL lock, no PA).
// Every step reports GetStatus + GetDeviceErrors. The radio is fully
// re-initialised with the normal firmware path afterwards.
// ---------------------------------------------------------------------------
static bool masn_diag_wait_busy(uint32_t ms, uint32_t &waited) {
  const uint32_t start = millis();
  while (digitalRead(pin_busy) == HIGH) {
    if ((uint32_t)(millis() - start) >= ms) { waited = ms; return false; }
    delay(1);
  }
  waited = (uint32_t)(millis() - start);
  return true;
}

static bool masn_diag_cmd(const uint8_t *data, size_t count) {
  uint8_t tmp[8];
  if (count > sizeof(tmp)) return false;
  memcpy(tmp, data, count);
  return masn_diag_spi(tmp, count);
}

static bool masn_diag_wreg(uint16_t reg, uint8_t value) {
  const uint8_t d[4] = {0x0D, (uint8_t)(reg >> 8), (uint8_t)reg, value};
  return masn_diag_cmd(d, sizeof(d));
}

// Appends ";<tag>=<status>/<mode>/<errors>" to out.
static void masn_diag_snap(char *out, size_t size, const char *tag) {
  uint8_t st = 0xFF;
  uint16_t err = 0xFFFF;
  bool a = masn_diag_status(st);
  bool b = masn_diag_word(0x17, err);
  size_t len = strlen(out);
  snprintf(out + len, size - len, ";%s=%02X/%u/%04X%s", tag, (unsigned)st,
           a ? (unsigned)((st >> 4) & 7) : 255U, (unsigned)err,
           (a && b) ? "" : "!");
}

static void masn_diag_osc_trace(uint8_t op) {
  char out[240];
  if (!radio_online) { masn_diag_reply(op, "ERR=RADIO_OFF"); return; }

  const int tcxo_mode = (op == 0x68) ? 0x02 : (op == 0x69) ? 0x07 : -1;
  snprintf(out, sizeof(out), "V=1;TCXO=%s",
           tcxo_mode == 0x02 ? "1V8" : tcxo_mode == 0x07 ? "3V3" : "NONE");
  size_t len;

  // 1. SPI write test with a non-default value.
  uint8_t s0 = 0, s1 = 0;
  LoRa->standby();
  masn_diag_wreg(0x0740, 0x34);
  masn_diag_wreg(0x0741, 0x44);
  masn_diag_reg(0x0740, s0);
  masn_diag_reg(0x0741, s1);
  len = strlen(out);
  snprintf(out + len, sizeof(out) - len, ";WR=%02X%02X", s0, s1);

  // 2. Hardware reset through P0.09 (NFC1 pin unless NFCPINS is cleared).
  uint32_t busy_ms = 0;
  digitalWrite(pin_reset, LOW);
  delay(2);
  bool busy_during_reset = digitalRead(pin_busy) == HIGH;
  digitalWrite(pin_reset, HIGH);
  delay(1);
  bool rst_ok = masn_diag_wait_busy(200, busy_ms);
  delay(30);  // fixed margin: do not trust BUSY until it is validated
  masn_diag_reg(0x0740, s0);
  masn_diag_reg(0x0741, s1);
  len = strlen(out);
  snprintf(out + len, sizeof(out) - len, ";RST=%02X%02X/B%d/%lums%s",
           s0, s1, (int)busy_during_reset, (unsigned long)busy_ms,
           rst_ok ? "" : "!");
  masn_diag_snap(out, sizeof(out), "POR");

  // 3. Clear errors.
  const uint8_t clr[3] = {0x07, 0x00, 0x00};
  masn_diag_cmd(clr, sizeof(clr));
  delay(5);
  masn_diag_snap(out, sizeof(out), "CLR");

  // 4. Optional TCXO control, 10 ms start-up timeout (0x000280 * 15.625 us).
  if (tcxo_mode >= 0) {
    const uint8_t tcxo[5] = {0x97, (uint8_t)tcxo_mode, 0x00, 0x02, 0x80};
    masn_diag_cmd(tcxo, sizeof(tcxo));
    delay(5);
  }

  // 5. Full calibration (runs on the selected oscillator).
  const uint8_t cal[2] = {0x89, 0x7F};
  masn_diag_cmd(cal, sizeof(cal));
  delay(1);
  masn_diag_wait_busy(100, busy_ms);
  delay(50);  // fixed margin covering TCXO timeout + calibration
  len = strlen(out);
  snprintf(out + len, sizeof(out) - len, ";CALB=%lums", (unsigned long)busy_ms);
  masn_diag_snap(out, sizeof(out), "CAL");

  // 6. STDBY_XOSC: expected mode 3 if the oscillator really runs.
  const uint8_t xosc[2] = {0x80, 0x01};
  masn_diag_cmd(xosc, sizeof(xosc));
  delay(30);
  masn_diag_snap(out, sizeof(out), "XOSC");

  // 7. SetPacketType(LoRa) + SetRfFrequency + SetFs: expected mode 4 (FS),
  //    i.e. PLL locked on the oscillator. The PA stays off.
  const uint8_t ptype[2] = {0x8A, 0x01};
  masn_diag_cmd(ptype, sizeof(ptype));
  delay(5);
  const uint32_t frf = (uint32_t)(((uint64_t)869618000UL << 25) / 32000000UL);
  const uint8_t freq[5] = {0x86, (uint8_t)(frf >> 24), (uint8_t)(frf >> 16),
                           (uint8_t)(frf >> 8), (uint8_t)frf};
  masn_diag_cmd(freq, sizeof(freq));
  delay(5);
  const uint8_t fs[1] = {0xC1};
  masn_diag_cmd(fs, sizeof(fs));
  delay(20);
  masn_diag_snap(out, sizeof(out), "FS");

  // 8. Back to STDBY_RC and full normal re-initialisation.
  const uint8_t rc[2] = {0x80, 0x00};
  masn_diag_cmd(rc, sizeof(rc));
  bool reinit = masn_diag_restore_radio();
  len = strlen(out);
  snprintf(out + len, sizeof(out) - len, ";REINIT=%d", (int)reinit);
  masn_diag_snap(out, sizeof(out), "END");
  masn_diag_reply(op, out);
}

// 0x6B: find the real BUSY line. BUSY is HIGH while NRESET is held LOW and
// LOW once the SX1262 is idle. Samples candidate GPIOs (inputs, no pull) in
// both states; reports pins whose level follows reset. Pins in use by SPI,
// RESET, RXEN, nice!nano VCC switch (P0.13), LED (P0.15), XTAL32 and USB
// are excluded so nothing gets disturbed.
static void masn_diag_busy_scan(uint8_t op) {
  static const uint8_t cand[] = {3, 4, 5, 6, 7, 8, 10, 11, 12, 20, 22, 24,
                                 25, 26, 28, 29, 30, 31, 32, 33, 34, 35,
                                 36, 37, 38, 39, 40, 41, 42, 44, 46};
  char out[240];
  uint32_t in_reset = 0, idle = 0, unstable = 0;  // bit i -> cand[i]
  for (size_t i = 0; i < sizeof(cand); i++) {
    if (cand[i] != pin_busy && cand[i] != pin_dio) pinMode(cand[i], INPUT);
  }
  for (int rep = 0; rep < 3; rep++) {
    digitalWrite(pin_reset, LOW);
    delay(3);
    uint32_t a = 0, b = 0;
    for (size_t i = 0; i < sizeof(cand); i++) if (digitalRead(cand[i])) a |= 1UL << i;
    digitalWrite(pin_reset, HIGH);
    delay(60);
    for (size_t i = 0; i < sizeof(cand); i++) if (digitalRead(cand[i])) b |= 1UL << i;
    if (rep == 0) { in_reset = a; idle = b; }
    else { unstable |= (a ^ in_reset) | (b ^ idle); }
  }
  snprintf(out, sizeof(out), "V=1;BUSYPIN=%d;FOLLOWS=", pin_busy);
  size_t len = strlen(out);
  bool any = false;
  for (size_t i = 0; i < sizeof(cand); i++) {
    uint32_t m = 1UL << i;
    if ((in_reset & m) && !(idle & m) && !(unstable & m)) {
      len = strlen(out);
      snprintf(out + len, sizeof(out) - len, "%s%u", any ? "," : "", cand[i]);
      any = true;
    }
  }
  len = strlen(out);
  snprintf(out + len, sizeof(out) - len, "%s;RST=%08lX;IDLE=%08lX;NOISY=%08lX",
           any ? "" : "NONE", (unsigned long)in_reset, (unsigned long)idle,
           (unsigned long)unstable);
  // Chip was reset: restore the normal radio configuration.
  bool reinit = masn_diag_restore_radio();
  len = strlen(out);
  snprintf(out + len, sizeof(out) - len, ";REINIT=%d", (int)reinit);
  masn_diag_reply(op, out);
}

// 0x6C: power probe. Reports the nRF52840 supply rails (VDDH is the
// battery/regulator input on the nice!nano, VDD the 3.3 V rail), every
// device that answers on the I2C bus, and the three INA3221 channels if
// one is found (bus voltage in mV, shunt voltage in uV).
static void masn_diag_power(uint8_t op) {
  char out[224];
  uint32_t vddh = masn_read_vddh_mv();
  uint32_t vdd = 0;
  analogReference(AR_INTERNAL);
  analogReadResolution(12);
  vdd = analogReadVDD() * 3600UL / 4095UL;
  analogReference(AR_DEFAULT);
  analogReadResolution(10);
  int len = snprintf(out, sizeof(out), "VDDH=%lu;VDD=%lu;I2C=", (unsigned long)vddh, (unsigned long)vdd);

  masn_wire_begin();
  uint8_t ina = 0;
  bool any = false;
  for (uint8_t a = 0x08; a < 0x78 && len < (int)sizeof(out) - 8; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      len += snprintf(out + len, sizeof(out) - len, "%s%02X", any ? "," : "", a);
      any = true;
      if (!ina && a >= 0x40 && a <= 0x43) {
        bool ok = true;
        if (masn_ina_reg(a, 0xFE, ok) == 0x5449 && ok) ina = a;  // INA3221 manufacturer ID "TI"
      }
    }
  }
  if (!any) len += snprintf(out + len, sizeof(out) - len, "none");
  if (ina) {
    len += snprintf(out + len, sizeof(out) - len, ";INA=%02X", ina);
    for (uint8_t ch = 0; ch < 3; ch++) {
      bool ok = true;
      int16_t shunt = masn_ina_reg(ina, 1 + ch * 2, ok);
      int16_t bus = masn_ina_reg(ina, 2 + ch * 2, ok);
      // LSBs: shunt 40 uV, bus 8 mV; the low 3 bits are unused.
      len += snprintf(out + len, sizeof(out) - len, ";CH%u=%d,%ld", ch + 1,
                      ok ? (int)(bus >> 3) * 8 : -1, ok ? (long)(shunt >> 3) * 40L : -1L);
    }
  }
  masn_diag_reply(op, out);
}

static void masn_diag_command(uint8_t op) {
  char out[224];
  if (op >= 0x68 && op <= 0x6A) { masn_diag_osc_trace(op); return; }
  if (op == 0x6B) { masn_diag_busy_scan(op); return; }
  if (op == 0x6C) { masn_diag_power(op); return; }
  if (op == 0x60) {
    snprintf(out, sizeof(out),
             "V=1;BOARD=MASN_HTRA62;ONLINE=%d;HW=%d;LOCK=%d;FREQ=%lu;"
             "TXP=%u;CS=%d;SCK=%d;MOSI=%d;MISO=%d;BUSYPIN=%d;DIO1=%d;RXENPIN=%d",
             (int)radio_online, (int)hw_ready, (int)radio_locked,
             (unsigned long)LoRa->getFrequency(), (unsigned)LoRa->getTxPower(),
             pin_cs, pin_sclk, pin_mosi, pin_miso, pin_busy, pin_dio, pin_rxen);
    masn_diag_reply(op, out);
  } else if (op == 0x61) {
    if (!radio_online) { masn_diag_reply(op, "ERR=RADIO_OFF"); return; }
    uint8_t st = 0xFF, sw0 = 0xFF, sw1 = 0xFF, ocp = 0xFF;
    bool a = masn_diag_status(st), b = masn_diag_reg(0x0740, sw0);
    bool c = masn_diag_reg(0x0741, sw1), d = masn_diag_reg(0x08E7, ocp);
    snprintf(out, sizeof(out),
             "SPI=%d%d%d%d;STATUS=%02X;MODE=%u;SYNC=%02X%02X;OCP=%02X;BUSY=%d",
             (int)a, (int)b, (int)c, (int)d, (unsigned)st,
             a ? (unsigned)((st >> 4) & 7) : 255U,
             (unsigned)sw0, (unsigned)sw1, (unsigned)ocp, digitalRead(pin_busy));
    masn_diag_reply(op, out);
  } else if (op == 0x62) {
    if (!radio_online) { masn_diag_reply(op, "ERR=RADIO_OFF"); return; }
    uint8_t st = 0xFF;
    uint16_t irq = 0xFFFF, err = 0xFFFF;
    bool a = masn_diag_status(st), b = masn_diag_word(0x12, irq);
    bool c = masn_diag_word(0x17, err);
    snprintf(out, sizeof(out),
             "SPI=%d%d%d;STATUS=%02X;MODE=%u;IRQ=%04X;ERRORS=%04X;BUSY=%d;"
             "DIO1=%d;RXEN=%d;RXCB=%lu;RXBYTES=%lu;TXOK=%lu;TXFAIL=%lu",
             (int)a, (int)b, (int)c, (unsigned)st,
             a ? (unsigned)((st >> 4) & 7) : 255U,
             (unsigned)irq, (unsigned)err, digitalRead(pin_busy),
             digitalRead(pin_dio), pin_rxen >= 0 ? digitalRead(pin_rxen) : -1,
             (unsigned long)masn_diag_rx_callbacks,
             (unsigned long)masn_diag_rx_bytes,
             (unsigned long)masn_diag_tx_ok, (unsigned long)masn_diag_tx_fail);
    masn_diag_reply(op, out);
  } else if (op == 0x63) {
    masn_diag_cw(op, false);
  } else if (op == 0x64) {
    masn_diag_cw(op, true);
  } else if (op == 0x65) {
    if (!masn_diag_tx_allowed()) {
      masn_diag_reply(op, "ERR=NOT_READY_OR_RF_LIMIT;REQ=869618000Hz,TXP<=7dBm,ONLINE=1");
      return;
    }
    static const uint8_t payload[] = "MASN-RF-TEST-V1";
    LoRa->beginPacket();
    LoRa->write(payload, sizeof(payload) - 1);
    int ok = LoRa->endPacket();
    if (ok) masn_diag_tx_ok++;
    else masn_diag_tx_fail++;
    lora_receive();
    uint8_t status = 0xFF;
    bool spi_ok = masn_diag_status(status);
    snprintf(out, sizeof(out), "TXDONE=%d;STATUS=%02X;SPI=%d;TXOK=%lu;TXFAIL=%lu",
             ok ? 1 : 0, (unsigned)status, (int)spi_ok,
             (unsigned long)masn_diag_tx_ok, (unsigned long)masn_diag_tx_fail);
    masn_diag_reply(op, out);
  } else if (op == 0x66) {
    if (!radio_online) { masn_diag_reply(op, "ERR=RADIO_OFF"); return; }
    LoRa->standby();
    uint8_t clear_err[2] = {0, 0};
    uint8_t clear_irq[2] = {0xFF, 0xFF};
    LoRa->executeOpcode(0x07, clear_err, 2);   // ClearDeviceErrors
    LoRa->executeOpcode(0x02, clear_irq, 2);   // ClearIrqStatus
    masn_diag_rx_callbacks = 0;
    masn_diag_rx_bytes = 0;
    masn_diag_tx_ok = 0;
    masn_diag_tx_fail = 0;
    lora_receive();
    uint16_t err = 0xFFFF;
    bool ok = masn_diag_word(0x17, err);
    snprintf(out, sizeof(out), "CLEARED=1;ERRORS=%04X;SPI=%d", (unsigned)err, (int)ok);
    masn_diag_reply(op, out);
  } else if (op == 0x67) {
    // Read-only RF configuration. Compare only with trusted HT-RA62 sources.
    if (!radio_online) { masn_diag_reply(op, "ERR=RADIO_OFF"); return; }
    uint8_t ocp = 0xFF, clamp = 0xFF, sens = 0xFF;
    bool a = masn_diag_reg(0x08E7, ocp);
    bool b = masn_diag_reg(0x08D8, clamp);
    bool c = masn_diag_reg(0x0889, sens);
    snprintf(out, sizeof(out), "SPI=%d%d%d;OCP=%02X;TXCLAMP=%02X;SENS=%02X;RXEN=%d",
             (int)a, (int)b, (int)c, (unsigned)ocp, (unsigned)clamp,
             (unsigned)sens, pin_rxen >= 0 ? digitalRead(pin_rxen) : -1);
    masn_diag_reply(op, out);
  }
}
#endif // BOARD_MODEL == BOARD_MASN_HTRA62
#endif // MASN_DIAGNOSTICS_H
