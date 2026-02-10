#include <ArxContainer.h>

// initialize with initializer_list
std::map<String, int> mp {{"one", 1}, {"two", 2}, {"four", 4}};

void setup() {
    Serial.begin(115200);
    delay(2000);

    // add contents
    String three = "three";
    mp.insert(std::make_pair(three, 3));
    mp["five"] = 5;

    // range based access
    Serial.print({"{ "});
    for (const auto& m : mp) {
        Serial.print("{");
        Serial.print(m.first);
        Serial.print(",");
        Serial.print(m.second);
        Serial.print("},");
    }
    Serial.println({" }"});

    // key access
    Serial.print("one   = ");
    Serial.println(mp["one"]);
    Serial.print("two   = ");
    Serial.println(mp["two"]);
    Serial.print("three = ");
    Serial.println(mp["three"]);
    Serial.print("four  = ");
    Serial.println(mp["four"]);
    Serial.print("five  = ");
    Serial.println(mp["five"]);

    // const ref access
    const auto& mp_ro = mp;
    Serial.print("const map one   = ");
    Serial.println(mp_ro.at("one"));
    Serial.print("const map four  = ");
    Serial.println(mp_ro.at("four"));
}

void loop() {
}
