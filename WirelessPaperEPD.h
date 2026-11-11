// Copyright (C) 2026, microReticulum contributors

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

// E-paper glue for the Heltec Wireless Paper.
//
// Heltec shipped the Wireless Paper with two different 2.13" 250x122 panels
// that need different controller drivers:
//
//   * LCMEN2R13EFC1 (Fitipower JD79656) on the V1.1 boards
//   * E0213A367 (Solomon Systech SSD1682) on V1.1.1 and later boards
//
// Neither driver exists in the upstream GxEPD2 library; both live in the
// Meshtastic fork, which is what the heltec-wireless-paper environment pulls
// in. The panel is identified at boot the same way Meshtastic does it: hold
// the controller in reset and sample the BUSY line, which idles LOW on the
// Fitipower controller and HIGH on the Solomon Systech one. Only the matching
// GxEPD2_BW instance is then allocated, and every call the firmware makes on
// the display is forwarded to it.

#ifndef WIRELESS_PAPER_EPD_H
#define WIRELESS_PAPER_EPD_H

#include <Arduino.h>
#include <SPI.h>
#include <GxEPD2_BW.h>

class WirelessPaperEPD {
public:
  enum Panel : uint8_t {
    PANEL_UNKNOWN = 0,
    PANEL_LCMEN2R13EFC1,
    PANEL_E0213A367
  };

  typedef GxEPD2_BW<GxEPD2_213_FC1, GxEPD2_213_FC1::HEIGHT> FC1Display;
  typedef GxEPD2_BW<GxEPD2_213_E0213A367, GxEPD2_213_E0213A367::HEIGHT> A367Display;

  WirelessPaperEPD(int16_t cs, int16_t dc, int16_t rst, int16_t busy, SPIClass& spi)
    : _cs(cs), _dc(dc), _rst(rst), _busy(busy), _spi(spi) {}

  // Identify the fitted panel from the idle level of its BUSY line while the
  // controller is held in reset. Safe to call before any SPI traffic.
  Panel detect() {
    pinMode(_rst, OUTPUT);
    digitalWrite(_rst, LOW);
    delay(10);
    pinMode(_busy, INPUT);
    int busy_level = digitalRead(_busy);
    pinMode(_rst, INPUT);
    return (busy_level == LOW) ? PANEL_LCMEN2R13EFC1 : PANEL_E0213A367;
  }

  // Detect the panel and allocate the matching driver. Returns false if the
  // driver could not be allocated.
  bool begin() {
    if (_gfx != nullptr) return true;
    _panel = detect();
    if (_panel == PANEL_LCMEN2R13EFC1) {
      _fc1 = new FC1Display(GxEPD2_213_FC1(_cs, _dc, _rst, _busy, _spi));
      _gfx = _fc1;
    } else {
      _a367 = new A367Display(GxEPD2_213_E0213A367(_cs, _dc, _rst, _busy, _spi));
      _gfx = _a367;
    }
    return _gfx != nullptr;
  }

  bool ready() const { return _gfx != nullptr; }
  Panel panel() const { return _panel; }
  const char* panel_name() const {
    switch (_panel) {
      case PANEL_LCMEN2R13EFC1: return "LCMEN2R13EFC1";
      case PANEL_E0213A367:     return "E0213A367";
      default:                  return "unknown";
    }
  }

  // GxEPD2_BW specific calls
  void init(uint32_t serial_diag_bitrate = 0) {
    if (_fc1) _fc1->init(serial_diag_bitrate);
    else if (_a367) _a367->init(serial_diag_bitrate);
  }
  void setFullWindow() {
    if (_fc1) _fc1->setFullWindow();
    else if (_a367) _a367->setFullWindow();
  }
  void setPartialWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
    if (_fc1) _fc1->setPartialWindow(x, y, w, h);
    else if (_a367) _a367->setPartialWindow(x, y, w, h);
  }
  // partial_update_mode = true requests a fast refresh, false a full refresh.
  void display(bool partial_update_mode = false) {
    if (_fc1) _fc1->display(partial_update_mode);
    else if (_a367) _a367->display(partial_update_mode);
  }
  void powerOff() {
    if (_fc1) _fc1->powerOff();
    else if (_a367) _a367->powerOff();
  }
  void hibernate() {
    if (_fc1) _fc1->hibernate();
    else if (_a367) _a367->hibernate();
  }
  bool isBusy() {
    if (_fc1) return _fc1->epd2.isBusy();
    if (_a367) return _a367->epd2.isBusy();
    return false;
  }

  // Adafruit_GFX calls used by Display.h
  void setRotation(uint8_t r) { if (_gfx) _gfx->setRotation(r); }
  uint8_t getRotation() const { return _gfx ? _gfx->getRotation() : 0; }
  int16_t width() const { return _gfx ? _gfx->width() : 0; }
  int16_t height() const { return _gfx ? _gfx->height() : 0; }
  void cp437(bool x = true) { if (_gfx) _gfx->cp437(x); }
  void fillScreen(uint16_t color) { if (_gfx) _gfx->fillScreen(color); }
  void drawPixel(int16_t x, int16_t y, uint16_t color) { if (_gfx) _gfx->drawPixel(x, y, color); }
  void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color) {
    if (_gfx) _gfx->drawLine(x0, y0, x1, y1, color);
  }
  void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
    if (_gfx) _gfx->fillRect(x, y, w, h, color);
  }
  void drawBitmap(int16_t x, int16_t y, const uint8_t* bitmap, int16_t w, int16_t h, uint16_t color, uint16_t bg) {
    if (_gfx) _gfx->drawBitmap(x, y, bitmap, w, h, color, bg);
  }

private:
  int16_t _cs, _dc, _rst, _busy;
  SPIClass& _spi;
  Panel _panel = PANEL_UNKNOWN;
  FC1Display* _fc1 = nullptr;
  A367Display* _a367 = nullptr;
  Adafruit_GFX* _gfx = nullptr;
};

#endif
