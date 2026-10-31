// MASN V1.2: battery, solar and load readings from the on-board INA3221.
// GPL-3.0-or-later, intended for integration in microReticulum_Firmware.
//
// Schematic masn-ht-ra62 rev 1.2 (I2C on SDA P1.04 / SCL P0.11):
//   CH1  BATTERY_POSITIVE -> NICENANO_BATTERY   node consumption
//   CH2  MPPT_CHARGER_BAT -> BATTERY_POSITIVE   CN3791 charge current
//   CH3  SOLAR_PANEL      -> MPPT_CHARGER_VIN   solar panel input
// Bus voltage is measured on IN- (the load side) of each channel. The usual
// INA3221 breakout boards carry 0.1 ohm shunts. CH1 goes negative while USB
// is plugged in: the nice!nano charger then feeds the battery backwards.
#ifndef MASN_POWER_H
#define MASN_POWER_H

#if BOARD_MODEL == BOARD_MASN_HTRA62

#include <Arduino.h>
#include <Wire.h>

#define MASN_INA3221_ADDR 0x40
#define MASN_SHUNT_MOHM   100

struct masn_power_t {
  bool ok;            // INA3221 answered
  int battery_mv;     // CH2 bus voltage (battery terminal)
  int battery_pct;    // rough LiPo state of charge from the voltage
  int load_ma;        // CH1: node consumption (negative = USB charging)
  int charge_ma;      // CH2: current from the solar charger into the battery
  int solar_mv;       // CH3 bus voltage (charger input)
  int solar_ma;       // CH3: current from the panel
  uint32_t vddh_mv;   // nRF52840 VDDH (battery, or USB when plugged)
};

static bool masn_wire_started = false;

static void masn_wire_begin() {
  if (!masn_wire_started) {
    Wire.begin();
    Wire.setClock(100000);
    masn_wire_started = true;
  }
}

static int16_t masn_ina_reg(uint8_t addr, uint8_t reg, bool &ok) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(addr, (uint8_t)2) != 2) { ok = false; return 0; }
  uint16_t hi = Wire.read(), lo = Wire.read();
  return (int16_t)((hi << 8) | lo);
}

// Bus voltage in mV (LSB 8 mV) and shunt voltage in uV (LSB 40 uV).
static bool masn_ina_channel(uint8_t ch, int &bus_mv, long &shunt_uv) {
  bool ok = true;
  int16_t shunt = masn_ina_reg(MASN_INA3221_ADDR, 1 + ch * 2, ok);
  int16_t bus = masn_ina_reg(MASN_INA3221_ADDR, 2 + ch * 2, ok);
  bus_mv = (bus >> 3) * 8;
  shunt_uv = (long)(shunt >> 3) * 40L;
  return ok;
}

static uint32_t masn_read_vddh_mv() {
  analogReference(AR_INTERNAL);       // 0..3.6 V
  analogReadResolution(12);
  delay(1);
  uint32_t mv = analogReadVDDHDIV5() * 5UL * 3600UL / 4095UL;
  analogReference(AR_DEFAULT);
  analogReadResolution(10);
  return mv;
}

static int masn_lipo_pct(int mv) {
  static const int16_t table[][2] = {
    {4200, 100}, {4100, 90}, {4000, 79}, {3900, 66}, {3800, 52},
    {3700, 32}, {3600, 15}, {3500, 6}, {3300, 0},
  };
  if (mv >= table[0][0]) return 100;
  for (unsigned i = 1; i < sizeof(table) / sizeof(table[0]); i++) {
    if (mv >= table[i][0]) {
      int v0 = table[i][0], v1 = table[i - 1][0], p0 = table[i][1], p1 = table[i - 1][1];
      return p0 + (mv - v0) * (p1 - p0) / (v1 - v0);
    }
  }
  return 0;
}

static masn_power_t masn_power_read() {
  masn_power_t p = {};
  masn_wire_begin();
  p.vddh_mv = masn_read_vddh_mv();
  int v; long uv;
  p.ok = masn_ina_channel(0, v, uv);
  p.load_ma = (int)(uv / MASN_SHUNT_MOHM);
  p.ok = masn_ina_channel(1, p.battery_mv, uv) && p.ok;
  p.charge_ma = (int)(uv / MASN_SHUNT_MOHM);
  p.ok = masn_ina_channel(2, p.solar_mv, uv) && p.ok;
  p.solar_ma = (int)(uv / MASN_SHUNT_MOHM);
  p.battery_pct = masn_lipo_pct(p.battery_mv);
  return p;
}

#endif
#endif
