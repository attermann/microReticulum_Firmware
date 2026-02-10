#include <ArxTypeTraits.h>

void setup()
{
    Serial.begin(115200);
    delay(2000);


    auto func = [](int i, float f, String s) -> float
    {
        return (float)i * f * s.toFloat();
    };

    auto t = std::make_tuple((int)1, (float)1.1f, String("1.5"));

    Serial.print(std::apply(func, t)); // 1.65
}

void loop()
{
}
