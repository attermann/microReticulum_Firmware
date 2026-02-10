// Uncommenting DEBUGLOG_DISABLE_LOG disables ASSERT and all log (Release Mode)
// PRINT and PRINTLN are always valid even in Release Mode
// #define DEBUGLOG_DISABLE_LOG

// You can also set default log level by defining macro (default: INFO)
// #define DEBUGLOG_DEFAULT_LOG_LEVEL_TRACE

#include <DebugLog.h>

void setup() {
    Serial.begin(115200);
    delay(2000);

    // PRINT and PRINTLN is not affected by log_level (always visible)
    PRINT("DebugLog", "can print variable args: ");
    PRINTLN(1, 2.2, "three", "=> like this");

    // You can change log_leval by following macro
    // LOG_SET_LEVEL(DebugLogLevel::LVL_TRACE);

    // The default log_leval is DebugLogLevel::LVL_INFO
    // 0: NONE, 1: ERROR, 2: WARN, 3: INFO, 4: DEBUG, 5: TRACE
    PRINTLN("current log level is", (int)LOG_GET_LEVEL());

    // The default log_leval is DebugLogLevel::LVL_INFO
    LOG_ERROR("this is error log");
    LOG_WARN("this is warn log");
    LOG_INFO("this is info log");
    LOG_DEBUG("this is debug log");  // won't be printed
    LOG_TRACE("this is trace log");  // won't be printed

    // Log array
    float arr[3] {1.1, 2.2, 3.3};
    PRINTLN("Array can be also printed like this", LOG_AS_ARR(arr, 3));

#if ARX_HAVE_LIBSTDCPLUSPLUS >= 201103L  // Have libstdc++11
    // Log containers
    std::vector<int> vs {1, 2, 3};
    std::deque<float> ds {1.1, 2.2, 3.3};
    std::map<String, int> ms {{"one", 1}, {"two", 2}, {"three", 3}};
    PRINTLN("Containers can also be printed like", vs, ds, ms);
#endif

    delay(1000);

    // You can also use assert
    // If assertion failed, Serial endlessly prints message
    int x = 1;
    // ASSERT(x != 1);
    // You can also use assert with messages by ASSERTM macro
    ASSERTM(x != 1, "This always fails");
}

void loop() {
    PRINTLN("If DEBUGLOG_DISABLE_LOG is commented out (assert is enabled), does not come here");
    delay(1000);
}
