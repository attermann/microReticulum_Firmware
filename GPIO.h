// Copyright (C) 2026, Chad Attermann

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.


#pragma once

#ifdef HAS_GPIO

#include <Arduino.h>

// ============================
// Pin definitions (EDIT THESE)
// ============================

#define GPIO0_PIN   20   // UART1_TX (P0.20)
#define GPIO1_PIN   19   // UART1_RX (P0.19)

#define GPIO_PIN_COUNT 2

namespace GPIO {

    enum {
        GPIO0 = 0,
        GPIO1 = 1,
    };
    typedef uint8_t Index;

    enum {
        MODE_INPUT = INPUT,
        MODE_OUTPUT = OUTPUT,
        MODE_UNKNOWN = 255,
    };
    typedef uint8_t Mode;

    enum {
        STATE_LOW = LOW,
        STATE_HIGH = HIGH,
        STATE_UNKNOWN = 255,
    };
    typedef uint8_t State;

    enum Disposition : uint8_t {
        DISP_UNKNOWN,
        DISP_INPUT,
        DISP_INPUT_LOW,
        DISP_INPUT_HIGH,
        DISP_OUTPUT_LOW,
        DISP_OUTPUT_HIGH,
    };
    //typedef uint8_t Disposition;

    inline uint32_t _pins[GPIO_PIN_COUNT];
    inline Mode _modes[GPIO_PIN_COUNT];

    // ============================
    // GPIO control abstraction
    // ============================

    inline void init()
    {
        // Set GPIO pins
        _pins[0] = GPIO0_PIN;
        _pins[1] = GPIO1_PIN;

        // Start floating (high impedance)
        pinMode(_pins[GPIO0], INPUT);
        _modes[GPIO0] = MODE_INPUT;
        pinMode(_pins[GPIO1], INPUT);
        _modes[GPIO1] = MODE_INPUT;
    }

    inline bool setMode(Index index, Mode mode)
    {
        if (index > sizeof(_pins)) return false;
        pinMode(_pins[index], mode);
        _modes[index] = mode;
        return true;
    }

    inline Mode getMode(Index index)
    {
        if (index > sizeof(_pins)) return MODE_UNKNOWN;
        return _modes[index];
    }
    inline bool isInput(Index index) { return getMode(index) == MODE_INPUT; }
    inline bool isOutput(Index index) { return getMode(index) == MODE_OUTPUT; }

    inline bool setState(Index index, State state)
    {
        if (index > sizeof(_pins)) return false;
        //setMode(index, MODE_OUTPUT);
        digitalWrite(_pins[index], state);
        return true;
    }

    inline State getState(Index index)
    {
        if (index > sizeof(_pins)) return STATE_UNKNOWN;
        return (State)digitalRead(_pins[index]);
    }
    inline bool isLow(Index index) { return getState(index) == STATE_LOW; }
    inline bool isHigh(Index index) { return getState(index) == STATE_HIGH; }

    inline bool setDisposition(Index index, Disposition disp)
    {
        if (index > sizeof(_pins)) return false;
        switch (disp) {
        case DISP_INPUT:
            setMode(index, MODE_INPUT);
            return true;
        case DISP_INPUT_LOW:
            setState(index, STATE_LOW);
            setMode(index, MODE_INPUT);
            return true;
        case DISP_INPUT_HIGH:
            setState(index, STATE_HIGH);
            setMode(index, MODE_INPUT);
            return true;
        case DISP_OUTPUT_LOW:
            setMode(index, MODE_OUTPUT);
            setState(index, STATE_LOW);
            return true;
        case DISP_OUTPUT_HIGH:
            setMode(index, MODE_OUTPUT);
            setState(index, STATE_HIGH);
            return true;
        default:
            return false;
        }
    }

    inline Disposition getDisposition(Index index)
    {
        if (index > sizeof(_pins)) return DISP_UNKNOWN;
        if (getMode(index) == MODE_INPUT && getState(index) == STATE_LOW) return DISP_INPUT_LOW;
        if (getMode(index) == MODE_INPUT && getState(index) == STATE_HIGH) return DISP_INPUT_HIGH;
        if (getMode(index) == MODE_OUTPUT && getState(index) == STATE_LOW) return DISP_OUTPUT_LOW;
        if (getMode(index) == MODE_OUTPUT && getState(index) == STATE_HIGH) return DISP_OUTPUT_HIGH;
        return DISP_UNKNOWN;
    }

}

#endif
