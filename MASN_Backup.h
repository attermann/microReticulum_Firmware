// MASN V1.2: keep a copy of the node's identity and configuration outside
// the LittleFS filesystem.
// GPL-3.0-or-later, intended for integration in microReticulum_Firmware.
//
// If the power fails while LittleFS is writing (the Adafruit flash layer
// erases and rewrites a whole 4 KB page for every block), the filesystem
// may no longer mount. InternalFileSystem::begin() then erases and formats
// the whole region, losing the emulated EEPROM (provisioning, firmware hash,
// radio config), the provisioning namespaces and the transport identity, so
// the node comes back unconfigured, radio off and with a new address.
//
// This module keeps those files in two raw flash pages just below the
// filesystem (A/B with a sequence number and CRC, so a power cut while
// saving still leaves the previous copy). The copy is refreshed only when
// the files change. At boot, if the EEPROM file is missing (the filesystem
// was formatted), the files are written back before anything reads them.
//
// The two pages are reserved by lowering upload.maximum_size in
// boards/masn_ht_ra62.json by 8 KB. The image is larger than half of the
// application area, so the bootloader updates it in single-bank mode and
// does not touch these pages.
#ifndef MASN_BACKUP_H
#define MASN_BACKUP_H

#if BOARD_MODEL == BOARD_MASN_HTRA62 && !HAS_EEPROM && MCU_VARIANT == MCU_NRF52

#include <InternalFileSystem.h>
#include "flash/flash_nrf5x.h"

#define MASN_BK_PAGE_A     0xEB000UL
#define MASN_BK_PAGE_B     0xEC000UL
#define MASN_BK_PAGE_SIZE  4096
#define MASN_BK_MAGIC      0x314B424DUL  // "MBK1"

struct masn_bk_header_t {
  uint32_t magic;
  uint32_t seq;
  uint32_t crc;      // CRC32 of the payload
  uint16_t len;      // payload length
  uint16_t count;    // number of files
};

// Payload: for each file, path length (1 byte), path, data length
// (2 bytes, little endian), data.
static uint8_t masn_bk_buf[MASN_BK_PAGE_SIZE];

static uint32_t masn_bk_crc32(const uint8_t *data, size_t len) {
  uint32_t crc = 0xFFFFFFFFUL;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) crc = (crc >> 1) ^ (0xEDB88320UL & (0UL - (crc & 1)));
  }
  return ~crc;
}

// Reads the page at addr into masn_bk_buf; true if it holds a valid copy.
static bool masn_bk_load(uint32_t addr, masn_bk_header_t &h) {
  flash_nrf5x_read(masn_bk_buf, addr, MASN_BK_PAGE_SIZE);
  memcpy(&h, masn_bk_buf, sizeof(h));
  if (h.magic != MASN_BK_MAGIC || h.len > MASN_BK_PAGE_SIZE - sizeof(h)) return false;
  return masn_bk_crc32(masn_bk_buf + sizeof(h), h.len) == h.crc;
}

// Address of the newest valid copy (0 if none) and its sequence number.
static uint32_t masn_bk_latest(uint32_t &seq) {
  masn_bk_header_t a, b;
  bool va = masn_bk_load(MASN_BK_PAGE_A, a);
  bool vb = masn_bk_load(MASN_BK_PAGE_B, b);
  if (va && (!vb || (int32_t)(a.seq - b.seq) > 0)) { seq = a.seq; return MASN_BK_PAGE_A; }
  if (vb) { seq = b.seq; return MASN_BK_PAGE_B; }
  seq = 0;
  return 0;
}

static bool masn_bk_add(uint8_t *payload, size_t &pos, uint16_t &count, const char *path,
                        Adafruit_LittleFS_Namespace::File &f) {
  const size_t plen = strlen(path);
  const uint32_t size = f.size();
  const size_t room = MASN_BK_PAGE_SIZE - sizeof(masn_bk_header_t);
  if (size > 0xFFFF || pos + 1 + plen + 2 + size > room) return false;
  payload[pos++] = (uint8_t)plen;
  memcpy(payload + pos, path, plen); pos += plen;
  payload[pos++] = size & 0xFF;
  payload[pos++] = size >> 8;
  f.seek(0);
  if (f.read(payload + pos, size) != (int)size) return false;
  pos += size;
  count++;
  return true;
}

// Builds the payload in masn_bk_buf. eeprom_file is the EEPROM file the
// firmware keeps open; the other files are opened read-only.
static bool masn_bk_build(Adafruit_LittleFS_Namespace::File &eeprom_file, masn_bk_header_t &h) {
  uint8_t *payload = masn_bk_buf + sizeof(masn_bk_header_t);
  size_t pos = 0;
  uint16_t count = 0;
  memset(masn_bk_buf, 0xFF, sizeof(masn_bk_buf));
  if (!masn_bk_add(payload, pos, count, EEPROM_FILE, eeprom_file)) return false;

  char path[32];
  for (int i = -1; i <= 9; i++) {
    if (i < 0) strcpy(path, "./transport_identity");
    else snprintf(path, sizeof(path), "./config/ns%d.msgpack", 100 + i);
    Adafruit_LittleFS_Namespace::File f(InternalFS);
    if (!f.open(path, Adafruit_LittleFS_Namespace::FILE_O_READ)) continue;
    bool ok = masn_bk_add(payload, pos, count, path, f);
    f.close();
    if (!ok) return false;
  }
  h.magic = MASN_BK_MAGIC;
  h.len = (uint16_t)pos;
  h.count = count;
  h.crc = masn_bk_crc32(payload, pos);
  return true;
}

// Saves a new copy if the files differ from the newest one. Only a
// provisioned node with an identity is saved, so a half-configured state
// never replaces a good copy. Returns true if a page was written.
static bool masn_backup_save(Adafruit_LittleFS_Namespace::File &eeprom_file) {
  uint8_t lock = 0;
  eeprom_file.seek(eeprom_addr(ADDR_INFO_LOCK));
  eeprom_file.read(&lock, 1);
  if (lock != INFO_LOCK_BYTE || !InternalFS.exists("./transport_identity")) return false;

  uint32_t seq;
  uint32_t latest = masn_bk_latest(seq);
  masn_bk_header_t old_h = {};
  uint32_t old_crc = 0; uint16_t old_len = 0;
  if (latest && masn_bk_load(latest, old_h)) { old_crc = old_h.crc; old_len = old_h.len; }

  masn_bk_header_t h;
  if (!masn_bk_build(eeprom_file, h)) return false;
  if (latest && h.crc == old_crc && h.len == old_len) return false;

  h.seq = seq + 1;
  memcpy(masn_bk_buf, &h, sizeof(h));
  const uint32_t target = (latest == MASN_BK_PAGE_A) ? MASN_BK_PAGE_B : MASN_BK_PAGE_A;
  flash_nrf5x_write(target, masn_bk_buf, MASN_BK_PAGE_SIZE);
  flash_nrf5x_flush();
  printf("[backup] identity and configuration saved (copy %lu, %u files, %u bytes)\r\n",
         (unsigned long)h.seq, h.count, h.len);
  return true;
}

// Called after InternalFS.begin() when the EEPROM file is missing. Writes
// every file of the newest copy back. Returns true if something was restored.
static bool masn_backup_restore() {
  uint32_t seq;
  uint32_t latest = masn_bk_latest(seq);
  masn_bk_header_t h;
  if (!latest || !masn_bk_load(latest, h)) return false;

  const uint8_t *p = masn_bk_buf + sizeof(h);
  const uint8_t *end = p + h.len;
  InternalFS.mkdir("./config");
  uint16_t restored = 0;
  for (uint16_t i = 0; i < h.count && p < end; i++) {
    char path[64];
    uint8_t plen = *p++;
    if (plen >= sizeof(path) || p + plen + 2 > end) break;
    memcpy(path, p, plen); path[plen] = 0; p += plen;
    uint16_t size = p[0] | (p[1] << 8); p += 2;
    if (p + size > end) break;
    if (InternalFS.exists(path)) InternalFS.remove(path);
    Adafruit_LittleFS_Namespace::File f(InternalFS);
    if (f.open(path, Adafruit_LittleFS_Namespace::FILE_O_WRITE)) {
      f.write(p, size);
      f.close();
      restored++;
    }
    p += size;
  }
  printf("[backup] filesystem was empty: restored %u files from copy %lu\r\n",
         restored, (unsigned long)h.seq);
  return restored > 0;
}

// Used when the EEPROM is wiped on purpose, so the copy does not bring the
// old configuration back on the next boot.
static void masn_backup_erase() {
  flash_nrf5x_erase(MASN_BK_PAGE_A);
  flash_nrf5x_erase(MASN_BK_PAGE_B);
}

#endif
#endif
