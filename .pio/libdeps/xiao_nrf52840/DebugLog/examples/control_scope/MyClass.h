#pragma once
#ifndef MYCLASS_H
#define MYCLASS_H

#include <Arduino.h>
#include <DebugLog.h>

// You can control the scope depending on your own flag
#ifdef MYCLASS_ENABLE_DEBUGLOG
#include <DebugLogEnable.h>
#else
#include <DebugLogDisable.h>
#endif  // MYCLASS_ENABLE_DEBUGLOG

class MyClass {
public:
    MyClass() {
        LOG_INFO("MyClass created");
    }
    ~MyClass() {
        LOG_INFO("MyClass destroyed");
    }
    void hello() {
        // This is always visible
        PRINTLN("hello", "world");
    }
};

// Finally you need to restore default DebugLog status (release or debug)
#include "DebugLogRestoreState.h"

#endif  // MYCLASS_H
