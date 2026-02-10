#include <ArxTypeTraits.h>

template <class T>
auto f(T)
-> typename std::enable_if<std::is_integral<T>::value>::type
{
    Serial.println("T is integral");
}

template <class T>
auto f(T)
-> typename std::enable_if<std::is_floating_point<T>::value>::type
{
    Serial.println("T is floating point");
}

template <class T>
auto f(T)
-> typename std::enable_if<!std::is_arithmetic<T>::value>::type
{
    Serial.println("T is not arithmetic");
}


void setup()
{
    Serial.begin(115200);
    delay(2000);

    f(1);
    f(1.1);
    f("1.11");

    // T is integral
    // T is floating point
    // T is not arithmetic
}

void loop()
{
}
