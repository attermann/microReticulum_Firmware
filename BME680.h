// Copyright (C) 2026, Chad Attermann
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

#pragma once

#ifdef HAS_BME

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BME680.h>
#include <microReticulum/Log.h>

namespace BME680 {

    inline bool bme_installed = false;
    inline Adafruit_BME680 bme;

    inline void init() {
        RNS::trace("Looking for BME680 environmental sensor...");
        
#ifdef WB_IO2
        RNS::trace("WB_IO2 is defined, pulling HIGH to power sensors...");
        // Power on sensors on WisBlock baseboards
        pinMode(WB_IO2, OUTPUT);
        digitalWrite(WB_IO2, HIGH);
        delay(100); // Give the sensor some time to power up
#else
        RNS::trace("WB_IO2 is NOT defined.");
#endif

        // Start I2C
        Wire.begin();
        
        // Attempt to initialize the sensor on the default I2C address (0x77 or 0x76)
        if (bme.begin(0x76) || bme.begin(0x77) || bme.begin()) {
            bme_installed = true;
            
            // Set up oversampling and filter initialization
            bme.setTemperatureOversampling(BME680_OS_8X);
            bme.setHumidityOversampling(BME680_OS_2X);
            bme.setPressureOversampling(BME680_OS_4X);
            bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
            bme.setGasHeater(320, 150); // 320*C for 150 ms
            
            RNS::trace("Initialized BME680 sensor successfully");
        } else {
            RNS::trace("BME680 sensor not found (gracefully ignoring)");
        }
    }

}

#endif
