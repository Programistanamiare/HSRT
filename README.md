
# Library for Handshake serial communication




## Authors

- [@Programistanamiare](https://www.github.com/Programistanamiare)


## Usage/Examples

# Hardware Handshake Receive / Transmit (HW_HSRT) EXEMPLE

```cpp
#include <hsrt.hpp>

HW_HSRT serial{Serial1, 19, 18};

void setup()
{
    Serial.begin(9600);
    serial.init(9600);
}

void loop()
{
    if (serial.isPeerRead())
    {
        uint8_t buffer[5];
        serial.read(buffer, 5);
        Serial.write(buffer, 5);
    }
}

```
