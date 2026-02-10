// Uncommenting DEBUGLOG_DISABLE_LOG disables ASSERT and all log (Release Mode)
// PRINT and PRINTLN are always valid even in Release Mode
// #define DEBUGLOG_DISABLE_LOG

// You can also set default log level by defining macro (default: INFO)
// #define DEBUGLOG_DEFAULT_LOG_LEVEL_TRACE

#include <DebugLog.h>

void setup() {
    Serial.begin(115200);
    delay(2000);

    // The default log_leval is DebugLogLevel::LVL_INFO
    // 0: NONE, 1: ERROR, 2: WARN, 3: INFO, 4: DEBUG, 5: TRACE
    PRINTLN("current log level is", (int)LOG_GET_LEVEL());

    LOG_ERROR("this is error log");
    LOG_WARN("this is warn log");
    LOG_INFO("this is info log");
    LOG_DEBUG("this is debug log");  // won't be printed
    LOG_TRACE("this is trace log");  // won't be printed

    // Let's try to change the log_level and see what happens
    for (size_t i = 0; i < 6; ++i) {
        switch (i) {
            case 0: LOG_SET_LEVEL(DebugLogLevel::LVL_TRACE); break;
            case 1: LOG_SET_LEVEL(DebugLogLevel::LVL_DEBUG); break;
            case 2: LOG_SET_LEVEL(DebugLogLevel::LVL_INFO); break;
            case 3: LOG_SET_LEVEL(DebugLogLevel::LVL_WARN); break;
            case 4: LOG_SET_LEVEL(DebugLogLevel::LVL_ERROR); break;
            default: LOG_SET_LEVEL(DebugLogLevel::LVL_NONE); break;
        }

        // PRINT and PRINTLN is not affected by the log_level (always visible)
        PRINTLN("change log level to", (int)LOG_GET_LEVEL());

        // LOG_XXXX can be controlled by log_level
        LOG_ERROR("this is error log");
        LOG_WARN("this is warn log");
        LOG_INFO("this is info log");
        LOG_DEBUG("this is debug log");
        LOG_TRACE("this is trace log");
    }
}

void loop() {
}
