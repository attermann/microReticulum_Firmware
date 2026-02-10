#include <DebugLog.h>

// You can define your own flag to control the scope of DebugLog
// In this example, you can control DebugLog only inside of "MyClass.h" depending on this flag
// see also "MyClass.h"
#define MYCLASS_ENABLE_DEBUGLOG
#include "MyClass.h"

void setup() {
    Serial.begin(115200);
    delay(2000);

    LOG_INFO("Create MyClass");

    MyClass myclass;
    myclass.hello();

    LOG_INFO("MyClass is going to be destroyed");
}

void loop() {
}
