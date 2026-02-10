#include <ArxTypeTraits.h>

int add(int x) { return x + 1; }

void setup()
{
    Serial.begin(115200);
    delay(2000);


    // std::function<float(int, float, String)>
    auto f1 = [](int i, float f, String s) -> float
    {
        return (float)i * f * s.toFloat();
    };
    Serial.print("calcurated return value from args : ");
    Serial.println(f1(1, 1.1, "1.5")); // 1.65

    Serial.print("reference captured values : ");
    int i0 {0}, i1 {1}, i2 {2};
    auto f2 = [&]()
    {
        Serial.print(i0); Serial.print(", ");
        Serial.print(i1); Serial.print(", ");
        Serial.print(i2); Serial.println();
    };
    f2();

    Serial.print("copy captured and modified values : ");
    auto f3 = [=](const int i) mutable
    {
        i0 += i; i1 += i; i2 += i;
        Serial.print(i0); Serial.print(", ");
        Serial.print(i1); Serial.print(", ");
        Serial.print(i2); Serial.println();
    };
    f3(10);

    auto f4 = add;
    Serial.print("function pointer return value : ");
    Serial.println(f4(1));
}

void loop()
{
}
