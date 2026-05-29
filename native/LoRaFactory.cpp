// Copyright (C) 2026, microReticulum_Firmware contributors

#include "LoRaFactory.h"

#include "../Modem.h"
#include "../sx126x.h"
#include "../sx127x.h"
#include "../sx128x.h"
#include "../LoRaRadio.h"

#include <cstdio>

// Pin globals defined in PinMap.cpp; populated by native_pinmap::apply().
extern int pin_cs;
extern int pin_reset;
extern int pin_dio;
extern int pin_busy;
extern int pin_rxen;
extern int pin_txen;

namespace native_lora {

ILoRaRadio* create_radio(uint8_t modem_id) {
    switch (modem_id) {
        case SX1262:
            sx126x_modem.setPins(pin_cs, pin_reset, pin_dio, pin_busy, pin_rxen);
            return &sx126x_modem;
        case SX1276:
        case SX1278:
            sx127x_modem.setPins(pin_cs, pin_reset, pin_dio, pin_busy);
            return &sx127x_modem;
        case SX1280:
            sx128x_modem.setPins(pin_cs, pin_reset, pin_dio, pin_busy, pin_rxen, pin_txen);
            return &sx128x_modem;
        default:
            std::fprintf(stderr, "[lora-factory] unknown modem id 0x%02X\n", modem_id);
            return nullptr;
    }
}

} // namespace native_lora
