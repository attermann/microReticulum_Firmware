// Native-build LoRa driver factory. Runtime-selects one of the three
// concrete modem drivers (sx126x / sx127x / sx128x) based on the modem id
// loaded from rnoded.conf, calls its driver-native setPins() with
// the correct arity using the global pin_* values already populated by
// native_pinmap::apply(), and returns the polymorphic ILoRaRadio* the
// firmware then drives via virtual dispatch.

#ifndef NATIVE_LORA_FACTORY_H
#define NATIVE_LORA_FACTORY_H

#include <cstdint>

class ILoRaRadio;

namespace native_lora {

// modem_id values match the constants in ../Modem.h (SX1262, SX1276,
// SX1278, SX1280). Returns nullptr if the id is unrecognized.
ILoRaRadio* create_radio(uint8_t modem_id);

} // namespace native_lora

#endif
