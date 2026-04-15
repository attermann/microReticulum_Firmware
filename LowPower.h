// Copyright (C) 2024
// Low Power Management for XIAO nRF52840 + Wio-SX1262
// Part of microReticulum firmware

#ifndef LOWPOWER_H
#define LOWPOWER_H

#include <Arduino.h>
#include "Boards.h"

#if defined(HAS_LOWPOWER) && HAS_LOWPOWER == true

// Power modes
#define POWER_MODE_PERFORMANCE  0   // Maximum responsiveness, continuous RX
#define POWER_MODE_BALANCED     1   // RX duty cycle, moderate power saving
#define POWER_MODE_LOW_POWER    2   // Aggressive RX duty cycle, maximum power saving

// Default power mode
#ifndef DEFAULT_POWER_MODE
  #define DEFAULT_POWER_MODE POWER_MODE_BALANCED
#endif

// Power configuration structure
struct PowerConfig {
    uint32_t rx_duty_rx_us;       // RX window in duty cycle (microseconds)
    uint32_t rx_duty_sleep_us;    // Sleep window in duty cycle (microseconds)
    uint32_t main_loop_sleep_ms;  // MCU sleep between loops (milliseconds)
    uint16_t ble_adv_interval_ms; // BLE advertising interval (milliseconds)
    int8_t   tx_power_dbm;        // TX power in dBm
    bool     use_rx_duty_cycle;   // Enable RX duty cycling
};

// Power configurations for each mode
// These are calculated for SF7/125kHz, 18-symbol preamble
// Symbol time @ SF7/125kHz = 1.024ms
static const PowerConfig POWER_CONFIGS[] = {
    // PERFORMANCE: Continuous RX, fast response, ~10mA
    {
        .rx_duty_rx_us = 0,           // Not used (continuous RX)
        .rx_duty_sleep_us = 0,        // Not used
        .main_loop_sleep_ms = 1,      // Minimal sleep
        .ble_adv_interval_ms = 100,   // Fast advertising
        .tx_power_dbm = 14,           // Higher TX power
        .use_rx_duty_cycle = false
    },
    
    // BALANCED: RX duty cycle, ~1.5mA average
    // RX window: 7ms (2 symbols + 5ms TCXO warmup)
    // Sleep window: 14ms (~14 symbols)
    {
        .rx_duty_rx_us = 7000,        // 7ms RX window
        .rx_duty_sleep_us = 14000,    // 14ms sleep window
        .main_loop_sleep_ms = 50,     // 50ms MCU sleep when idle
        .ble_adv_interval_ms = 1000,  // 1 second advertising
        .tx_power_dbm = 10,           // Moderate TX power
        .use_rx_duty_cycle = true
    },
    
    // LOW_POWER: Aggressive duty cycle, ~500µA average
    // RX window: 7ms
    // Sleep window: 50ms (requires 32+ symbol preamble on TX side)
    {
        .rx_duty_rx_us = 7000,        // 7ms RX window
        .rx_duty_sleep_us = 50000,    // 50ms sleep window
        .main_loop_sleep_ms = 100,    // 100ms MCU sleep when idle
        .ble_adv_interval_ms = 2000,  // 2 second advertising
        .tx_power_dbm = 7,            // Lower TX power
        .use_rx_duty_cycle = true
    }
};

// Global power state
extern uint8_t power_mode;
extern bool low_power_initialized;

// Statistics
extern uint32_t lp_packets_received;
extern uint32_t lp_packets_transmitted;
extern uint32_t lp_sleep_time_ms;
extern uint32_t lp_awake_time_ms;

// Function declarations
void lowpower_init();
void lowpower_set_mode(uint8_t mode);
uint8_t lowpower_get_mode();
const PowerConfig* lowpower_get_config();
void lowpower_enter_sleep(uint32_t ms);
void lowpower_disable_unused_peripherals();
void lowpower_apply_radio_config();
void lowpower_apply_ble_config();

// Utility functions
float lowpower_get_duty_cycle();
float lowpower_get_estimated_current_ma();

#endif // HAS_LOWPOWER

// Stub functions for when low power is not available
#if !defined(HAS_LOWPOWER) || HAS_LOWPOWER != true
  #define lowpower_init() do {} while(0)
  #define lowpower_set_mode(m) do {} while(0)
  #define lowpower_get_mode() 0
  #define lowpower_enter_sleep(ms) delay(ms)
  #define lowpower_disable_unused_peripherals() do {} while(0)
  #define lowpower_apply_radio_config() do {} while(0)
  #define lowpower_apply_ble_config() do {} while(0)
#endif

#endif // LOWPOWER_H
