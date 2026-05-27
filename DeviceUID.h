// Hardware-rooted device unique ID.
//
// Derived at boot from factory-burned identifiers — no provisioning required:
//   ESP32: BT MAC (efuse), via esp_read_mac(mac, ESP_MAC_BT)
//   nRF52: FICR DEVICEADDR, with the 0xC0 BLE static-random marker applied
//          so the value matches the address advertised over BLE.
//
// 6 bytes / 48 bits — the smallest size that preserves the full unique range
// of both sources. Independent of ADDR_SERIAL and rnodeconf provisioning.

#ifndef DEVICEUID_H
#define DEVICEUID_H

#include <stdint.h>
#include <stdio.h>
#include "Boards.h"

#if MCU_VARIANT == MCU_ESP32
  #if __has_include("esp_mac.h")
    #include "esp_mac.h"
  #endif
  #include "esp_system.h"
#endif

#define DEVICE_UID_LEN 6
#define DEVICE_UID_STR_LEN 13  // 12 hex chars + NUL

uint8_t device_uid[DEVICE_UID_LEN] = {0};
char    device_uid_str[DEVICE_UID_STR_LEN] = {0};

void device_uid_init() {
#if MCU_VARIANT == MCU_ESP32
  esp_read_mac(device_uid, ESP_MAC_BT);

#elif MCU_VARIANT == MCU_NRF52
  uint32_t lo = NRF_FICR->DEVICEADDR[0];
  uint32_t hi = NRF_FICR->DEVICEADDR[1];
  device_uid[0] = (lo      ) & 0xFF;
  device_uid[1] = (lo >>  8) & 0xFF;
  device_uid[2] = (lo >> 16) & 0xFF;
  device_uid[3] = (lo >> 24) & 0xFF;
  device_uid[4] = (hi      ) & 0xFF;
  device_uid[5] = ((hi >> 8) & 0xFF) | 0xC0;
#endif

  snprintf(device_uid_str, DEVICE_UID_STR_LEN,
           "%02X%02X%02X%02X%02X%02X",
           device_uid[0], device_uid[1], device_uid[2],
           device_uid[3], device_uid[4], device_uid[5]);
}

#endif  // DEVICEUID_H
