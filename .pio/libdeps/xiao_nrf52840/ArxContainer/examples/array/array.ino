#include <ArxContainer.h>

// initialize with initializer_list
std::array<int, 3> arr {1, 2, 3};

void setup() {
    Serial.begin(115200);
    delay(2000);

    // index access
    for (size_t i = 0; i < arr.size(); ++i) {
        Serial.print(arr[i]);
        Serial.print(" ");
    }
    Serial.println();

    // fill
    arr.fill(123);

    // range-based access
    for (const auto& a : arr) {
        Serial.print(a);
        Serial.print(" ");
    }
    Serial.println();
}

void loop() {
}
