/*
  WVariant.h - Seeed Tracker T1000-E (nRF52840 + LR1110)
  Standard Adafruit nRF52 variant glue.
*/

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <nrf.h>
#include <nrf_soc.h>
#include <nrf_sdm.h>
#include <nrf_gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

extern const uint32_t g_ADigitalPinMap[];

#ifdef __cplusplus
} // extern "C"
#endif
