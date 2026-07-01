// Hardware-rooted device unique ID.
//
// Derived at boot from factory-burned identifiers — no provisioning required:
//   ESP32:  BT MAC (efuse), via esp_read_mac(mac, ESP_MAC_BT). Already a
//           public link-layer identifier, used directly.
//   nRF52:  FICR DEVICEADDR, with the 0xC0 BLE static-random marker applied
//           so the value matches the address advertised over BLE.
//   NATIVE: SHA-256(salt || stable_host_identifier)[0..5]. Source is, in
//           priority order: /etc/machine-id, /var/lib/dbus/machine-id,
//           /proc/cpuinfo Serial line, then hostname. The hash hides the
//           underlying identifier — those sources are not normally
//           broadcast by the OS, so exposing them verbatim in a radio
//           announce would leak host identity. Different threat model
//           from the BLE-rooted IDs above, hence the extra step.
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
#elif MCU_VARIANT == MCU_NATIVE
  #include <string.h>
  #include <unistd.h>
  #include <SHA256.h>
#endif

#define DEVICE_UID_LEN 6
#define DEVICE_UID_STR_LEN 13  // 12 hex chars + NUL

uint8_t device_uid[DEVICE_UID_LEN] = {0};
char    device_uid_str[DEVICE_UID_STR_LEN] = {0};

#if MCU_VARIANT == MCU_NATIVE
// Return number of bytes written to `out`, or 0 if the source isn't usable.
// Helpers are file-scope static — DeviceUID.h is included from exactly one
// translation unit (RNode_Firmware.ino) so no ODR concerns.

static size_t deviceuid_read_file_trimmed(const char* path, uint8_t* out, size_t cap) {
  FILE* f = fopen(path, "r");
  if (!f) return 0;
  size_t n = fread(out, 1, cap, f);
  fclose(f);
  while (n > 0 && (out[n-1] == '\n' || out[n-1] == '\r' ||
                   out[n-1] == ' '  || out[n-1] == '\t')) n--;
  return n;
}

static size_t deviceuid_read_cpuinfo_serial(uint8_t* out, size_t cap) {
  FILE* f = fopen("/proc/cpuinfo", "r");
  if (!f) return 0;
  char line[256];
  size_t result = 0;
  while (fgets(line, sizeof(line), f)) {
    if (strncmp(line, "Serial", 6) != 0) continue;
    char* p = strchr(line, ':');
    if (!p) continue;
    p++;
    while (*p == ' ' || *p == '\t') p++;
    size_t hex_len = 0;
    while (p[hex_len] &&
           ((p[hex_len] >= '0' && p[hex_len] <= '9') ||
            (p[hex_len] >= 'a' && p[hex_len] <= 'f') ||
            (p[hex_len] >= 'A' && p[hex_len] <= 'F'))) hex_len++;
    // Many SoC kernels emit a placeholder Serial of all zeros — treat that
    // as "no usable serial" and let the next fallback run.
    if (hex_len < 8) break;
    bool all_zero = true;
    for (size_t i = 0; i < hex_len; i++) {
      if (p[i] != '0') { all_zero = false; break; }
    }
    if (all_zero) break;
    if (hex_len > cap) hex_len = cap;
    memcpy(out, p, hex_len);
    result = hex_len;
    break;
  }
  fclose(f);
  return result;
}

static size_t deviceuid_read_hostname(uint8_t* out, size_t cap) {
  char host[256];
  if (gethostname(host, sizeof(host)) != 0) return 0;
  host[sizeof(host)-1] = '\0';
  size_t n = strlen(host);
  if (n == 0) return 0;
  if (n > cap) n = cap;
  memcpy(out, host, n);
  return n;
}
#endif  // MCU_VARIANT == MCU_NATIVE

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

#elif MCU_VARIANT == MCU_NATIVE
  uint8_t raw[256];
  size_t n =                 deviceuid_read_file_trimmed("/etc/machine-id",          raw, sizeof(raw));
  if (n == 0) n =            deviceuid_read_file_trimmed("/var/lib/dbus/machine-id", raw, sizeof(raw));
  if (n == 0) n =            deviceuid_read_cpuinfo_serial(                          raw, sizeof(raw));
  if (n == 0) n =            deviceuid_read_hostname(                                raw, sizeof(raw));

  if (n == 0) {
    fprintf(stderr, "[DeviceUID] no usable identifier source; UID will be 000000000000\n");
  } else {
    // SHA-256(salt || raw), take first 6 bytes. Salt is hashed including its
    // terminating NUL so a hypothetical raw input of "microReticulum-device-uid"
    // can't collide with the salted form.
    static const char salt[] = "microReticulum-device-uid";
    SHA256 sha;
    sha.update(salt, sizeof(salt));  // sizeof(salt) includes the trailing '\0'
    sha.update(raw, n);
    uint8_t digest[32];
    sha.finalize(digest, sizeof(digest));
    memcpy(device_uid, digest, DEVICE_UID_LEN);
  }
#endif

  snprintf(device_uid_str, DEVICE_UID_STR_LEN,
           "%02X%02X%02X%02X%02X%02X",
           device_uid[0], device_uid[1], device_uid[2],
           device_uid[3], device_uid[4], device_uid[5]);
}

#endif  // DEVICEUID_H
