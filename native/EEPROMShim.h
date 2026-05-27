// Copyright (C) 2026, microReticulum_Firmware contributors
//
// File-backed EEPROM shim for the native Linux build. Stands in for the
// Arduino <EEPROM.h> library so the existing config-load/save paths in
// Utilities.h (eeprom_conf_load, eeprom_conf_save, eeprom_info_locked,
// etc.) work without per-platform forks.
//
// Storage model: an in-memory byte array of EEPROM_SIZE, flushed to
// ./eeprom (relative to CWD, set by --data-dir before setup() runs)
// when EEPROM.commit() is called or on graceful shutdown.

#ifndef NATIVE_EEPROM_SHIM_H
#define NATIVE_EEPROM_SHIM_H

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

class EEPROMClass {
public:
    EEPROMClass() : path_("eeprom") {}

    // Arduino API surface used by Utilities.h
    void begin(std::size_t size) {
        buf_.assign(size, 0xFF);
        FILE* f = std::fopen(path_.c_str(), "rb");
        if (f) {
            std::fread(buf_.data(), 1, size, f);
            std::fclose(f);
        }
    }

    uint8_t read(int addr) const {
        if (addr < 0 || static_cast<std::size_t>(addr) >= buf_.size()) return 0xFF;
        return buf_[addr];
    }

    void write(int addr, uint8_t val) {
        if (addr < 0 || static_cast<std::size_t>(addr) >= buf_.size()) return;
        if (buf_[addr] != val) {
            buf_[addr] = val;
            dirty_ = true;
        }
    }

    // Arduino's EEPROM.update() writes only if value changed (avoiding
    // flash wear on real hardware); our write() already short-circuits.
    void update(int addr, uint8_t val) { write(addr, val); }

    bool commit() {
        if (!dirty_) return true;
        FILE* f = std::fopen(path_.c_str(), "wb");
        if (!f) return false;
        std::size_t wrote = std::fwrite(buf_.data(), 1, buf_.size(), f);
        std::fclose(f);
        dirty_ = (wrote != buf_.size());
        return !dirty_;
    }

    // Set before begin() if the default ./eeprom path is unwanted.
    void set_path(const std::string& p) { path_ = p; }

private:
    std::vector<uint8_t> buf_;
    std::string path_;
    bool dirty_ = false;
};

extern EEPROMClass EEPROM;

#endif // NATIVE_EEPROM_SHIM_H
