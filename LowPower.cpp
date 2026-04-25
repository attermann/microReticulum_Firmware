// Copyright (C) 2024
// Low Power Management Implementation for XIAO nRF52840 + Wio-SX1262
// Part of microReticulum firmware

#include "Boards.h"
#include "LowPower.h"

#if defined(HAS_LOWPOWER) && HAS_LOWPOWER == true

#include <Arduino.h>

#if MCU_VARIANT == MCU_NRF52
  #include <nrf_power.h>
  #include <nrf_gpio.h>
  // SoftDevice power management
  extern "C" {
    #include "nrf_sdm.h"
    #include "nrf_soc.h"
  }
#endif

// Forward declarations for external components
#if MODEM == SX1262
  #include "sx126x.h"
  extern sx126x* LoRa;
#endif

#if HAS_BLE == true
  #include <bluefruit.h>
#endif

// Global state
uint8_t power_mode = DEFAULT_POWER_MODE;
bool low_power_initialized = false;

// Statistics
uint32_t lp_packets_received = 0;
uint32_t lp_packets_transmitted = 0;
uint32_t lp_sleep_time_ms = 0;
uint32_t lp_awake_time_ms = 0;

// Internal state
static uint32_t last_mode_change = 0;
static uint32_t sleep_start_time = 0;

void lowpower_init() {
    if (low_power_initialized) return;
    
    #if MCU_VARIANT == MCU_NRF52
        // Enable DC-DC converter for better efficiency
        // This is done via SoftDevice if BLE is enabled
        #if HAS_BLE == true
            // DC-DC is configured by Bluefruit
        #else
            NRF_POWER->DCDCEN = 1;
        #endif
        
        // Set low power mode
        // Note: With SoftDevice, use sd_power_mode_set()
        #if HAS_BLE == true
            sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
        #endif
        
        // Disable unused peripherals
        lowpower_disable_unused_peripherals();
    #endif
    
    low_power_initialized = true;
    last_mode_change = millis();
    
    // Apply initial power mode configuration
    lowpower_set_mode(power_mode);
}

void lowpower_disable_unused_peripherals() {
    #if MCU_VARIANT == MCU_NRF52
        // Disable UART if console is not active
        #if !HAS_CONSOLE
            // Don't disable UART0 as it may be used for serial communication
            // NRF_UARTE0->ENABLE = 0;
        #endif
        
        // Disable ADC (SAADC)
        NRF_SAADC->ENABLE = 0;
        
        // Disable PWM instances if no display/LEDs using PWM
        #if !HAS_DISPLAY
            NRF_PWM0->ENABLE = 0;
            NRF_PWM1->ENABLE = 0;
            NRF_PWM2->ENABLE = 0;
        #endif
        
        // Disable I2C if no PMU or display
        #if !HAS_PMU && !HAS_DISPLAY
            NRF_TWIM0->ENABLE = 0;
            NRF_TWIM1->ENABLE = 0;
        #endif
        
        // Disable NFC
        NRF_NFCT->TASKS_DISABLE = 1;
        
        // Disable unused GPIOTE channels
        // Note: Keep GPIOTE enabled for radio interrupts
        
        // Disable comparators
        NRF_COMP->ENABLE = 0;
        NRF_LPCOMP->ENABLE = 0;
    #endif
}

void lowpower_set_mode(uint8_t mode) {
    if (mode > POWER_MODE_LOW_POWER) {
        mode = POWER_MODE_BALANCED;
    }
    
    power_mode = mode;
    last_mode_change = millis();
    
    // Apply radio configuration
    lowpower_apply_radio_config();
    
    // Apply BLE configuration
    lowpower_apply_ble_config();
}

uint8_t lowpower_get_mode() {
    return power_mode;
}

const PowerConfig* lowpower_get_config() {
    return &POWER_CONFIGS[power_mode];
}

void lowpower_apply_radio_config() {
    #if MODEM == SX1262
        if (LoRa == NULL) return;
        
        const PowerConfig* config = lowpower_get_config();
        
        // Set TX power
        LoRa->setTxPower(config->tx_power_dbm);
        
        // Configure RX mode based on power mode
        if (config->use_rx_duty_cycle) {
            // Set up RX duty cycle parameters
            LoRa->setRxDutyCycle(config->rx_duty_rx_us, config->rx_duty_sleep_us);
        }
    #endif
}

void lowpower_apply_ble_config() {
    #if HAS_BLE == true && MCU_VARIANT == MCU_NRF52
        const PowerConfig* config = lowpower_get_config();
        
        // Only adjust if BLE is already started
        if (Bluefruit.Advertising.isRunning()) {
            // Convert ms to 0.625ms units for BLE
            uint16_t adv_interval = (config->ble_adv_interval_ms * 1000) / 625;
            
            // Stop advertising temporarily
            Bluefruit.Advertising.stop();
            
            // Set new interval (min and max)
            Bluefruit.Advertising.setInterval(adv_interval, adv_interval * 2);
            
            // Adjust TX power based on power mode
            if (power_mode == POWER_MODE_LOW_POWER) {
                Bluefruit.setTxPower(-4);  // Low TX power
            } else if (power_mode == POWER_MODE_BALANCED) {
                Bluefruit.setTxPower(0);   // Medium TX power
            } else {
                Bluefruit.setTxPower(4);   // Normal TX power
            }
            
            // Restart advertising
            Bluefruit.Advertising.start(0);
        }
    #endif
}

void lowpower_enter_sleep(uint32_t ms) {
    if (ms == 0) return;
    
    sleep_start_time = millis();
    
    #if MCU_VARIANT == MCU_NRF52
        // FreeRTOS tickless idle handles low power automatically
        // Just call delay() and the system will sleep efficiently
        // The nRF52 Arduino core uses FreeRTOS with tickless idle enabled
        delay(ms);
    #else
        delay(ms);
    #endif
    
    // Update statistics
    uint32_t actual_sleep = millis() - sleep_start_time;
    lp_sleep_time_ms += actual_sleep;
}

float lowpower_get_duty_cycle() {
    const PowerConfig* config = lowpower_get_config();
    
    if (!config->use_rx_duty_cycle || config->rx_duty_rx_us == 0) {
        return 1.0f;  // 100% duty cycle (always on)
    }
    
    uint32_t total_period = config->rx_duty_rx_us + config->rx_duty_sleep_us;
    return (float)config->rx_duty_rx_us / (float)total_period;
}

float lowpower_get_estimated_current_ma() {
    const PowerConfig* config = lowpower_get_config();
    
    // Current consumption estimates (mA)
    const float MCU_ACTIVE_MA = 3.0f;
    const float MCU_SLEEP_MA = 0.003f;  // 3µA in System ON idle
    const float RADIO_RX_MA = 4.6f;     // SX1262 RX boosted mode
    const float RADIO_SLEEP_MA = 0.0006f; // 0.6µA warm sleep
    const float BLE_ADV_MA = 0.015f;    // Average during slow advertising
    
    float duty_cycle = lowpower_get_duty_cycle();
    
    // Calculate radio current
    float radio_current;
    if (config->use_rx_duty_cycle) {
        radio_current = (RADIO_RX_MA * duty_cycle) + (RADIO_SLEEP_MA * (1.0f - duty_cycle));
    } else {
        radio_current = RADIO_RX_MA;
    }
    
    // Calculate MCU current (simplified - assumes mostly sleeping)
    float mcu_duty = 0.01f;  // ~1% active time for processing
    float mcu_current = (MCU_ACTIVE_MA * mcu_duty) + (MCU_SLEEP_MA * (1.0f - mcu_duty));
    
    // Calculate BLE current (simplified)
    float ble_current = BLE_ADV_MA * (100.0f / config->ble_adv_interval_ms);
    
    return radio_current + mcu_current + ble_current;
}

#endif // HAS_LOWPOWER
