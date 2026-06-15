// Abstract LoRa radio interface used by the polymorphic `LoRa` pointer.
//
// On embedded targets the pointer is typed concretely (sx126x*, sx127x*,
// sx128x*) so virtual dispatch is bypassed at the call site; the interface
// is still present in the vtable but only one driver compiles.
//
// On native (-DMODEM=MODEM_RUNTIME) all three drivers compile in and the
// `LoRa` pointer is typed as `ILoRaRadio*` so the runtime-selected driver
// can be dispatched through this interface.
//
// Only methods that are called through `LoRa->` (across the firmware) are
// declared here. Driver-internal methods stay private to each concrete
// class. Method parameter types are normalized to widths that fit every
// driver family (e.g. uint32_t for frequency and bandwidth).

#ifndef LORA_RADIO_H
#define LORA_RADIO_H

#include <Arduino.h>
#include <stdint.h>

class ILoRaRadio : public Stream {
public:
  virtual ~ILoRaRadio() {}

  // Lifecycle
  virtual bool preInit() = 0;
  virtual int  begin(uint32_t frequency) = 0;
  virtual void end() = 0;

  // RX
  virtual void receive(int size = 0) = 0;
  virtual void onReceive(void(*callback)(int)) = 0;
  virtual void handleDio0IfPending() = 0;

  // TX
  virtual int beginPacket(int implicitHeader = false) = 0;
  virtual int endPacket() = 0;

  // Per-packet metrics
  virtual int     packetRssi() = 0;
  virtual int     packetRssi(uint8_t pkt_snr_raw) = 0;
  virtual uint8_t packetSnrRaw() = 0;
  virtual int     currentRssi() = 0;
  virtual bool    dcd() = 0;

  // PHY parameters
  virtual uint32_t getFrequency() = 0;
  virtual void     setFrequency(uint32_t frequency) = 0;
  virtual uint32_t getSignalBandwidth() = 0;
  virtual void     setSignalBandwidth(uint32_t sbw) = 0;
  virtual void     setSpreadingFactor(int sf) = 0;
  virtual void     setCodingRate4(int denominator) = 0;
  virtual void     setPreambleLength(long preamble_symbols) = 0;
  virtual void     enableCrc() = 0;

  // Power
  virtual uint8_t getTxPower() = 0;
  virtual void    setTxPower(int level, int outputPin) = 0;

  // Optional runtime SX126x knobs (no-op on other modems). Used by the
  // native target to override per-board compile-time defaults from
  // rnoded.conf without a downcast at the call site.
  virtual void    setTcxoVoltage(uint8_t /*mode_byte*/) {}
  virtual void    setDio2AsRfSwitch(bool /*enable*/) {}
};

#endif
