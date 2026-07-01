#define SX1276 0x01
#define SX1278 0x02
#define SX1262 0x03
#define SX1280 0x04

// Sentinel for native targets where the modem family is chosen at runtime
// from rnoded.conf. Drivers and MODEM-gated call sites use this to
// enable a parallel runtime-dispatch branch alongside the compile-time arms.
#define MODEM_RUNTIME 0xFF
