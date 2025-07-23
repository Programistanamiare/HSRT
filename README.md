
# Library for Handshake serial communication

## Authors

- [@Programistanamiare](https://www.github.com/Programistanamiare)

## API Reference

### HW_HSRT Class (Hardware driver handshake serial communication)

#### Constructor

```cpp
  HW_HSRT::HW_HSRT(HardwareSerial& driver, uint8_t rxPin, uint8_t txPin);
```
| Parameter | Type | Description |
| :--- | :--- | :--- |
| `driver` | `HardwareSerial&` | Reference to hardware serial driver (ex. Serial1) |
| `rxPin` | `uint8_t` | Pin connected to RX serial driver |
| `txPin` | `uint8_t` | Pin connected to TX serial driver |

---

#### Initialize 

```cpp
  void HW_HSRT::init(const unsigned long baudrate = 9600ul);
```

| Parameter | Type     | Description                |
| :-------- | :------- | :------------------------- |
| `baudrate` | `unsigned long` | Communcation speed |

---

#### Checking the sender does not want to establish communication by setting a flag.

```cpp
  bool HW_HSRT::isPeerReady(void);
```

| Retval | If |   
| :----- | :------------- |
|  `true` | Sender set communication flag  |
| `false` | Sender did not set the communication flag |

---

#### Write data to the recipient.

```cpp
  int HW_HSRT::write(const uint8_t* buffer, size_t size, const unsigned long timeout = 200ul);
```

| Parameter | Type | Description |
| :--- | :--- | :--- |
| `buffer` | `uint8_t*` | Buffer with data to send |
| `size`  | `size_t` | Buffer size |
| `timeout` | `unsigned long` | Waittime for handshake connection to the recipient |

| Retval | If |   
| :----- | :------------- |
|  `0` | Communication was successfully established and data was sent |
| `-1` | The receiving device should set the communication flag first, which means that it should handle the reading of this data first and then send its data |
| `-2` | Data buffer points to nullptr |
| `-3` | The waiting time for connection to the receiving device has expired |

---

#### Read data from the sender.

```cpp
  int HW_HSRT::read(uint8_t* buffer, size_t size);
```

| Parameter | Type | Description |
| :--- | :--- | :--- |
| `buffer` | `uint8_t*` | Buffer to readed data. |
| `size` | `size_t` | Buffer size. |

| Retval | If |
| :--- | :--- |
| `>= 0` | number of bytes read |
| `-1` | sender did not set the communication flag |
| `-2` | data buffer points to nullptr |

---

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
---
---

### SW_HSRT Class (Software driver handshake serial communication)

#### Constructor

```cpp
  HW_HSRT::HW_HSRT(uint8_t rxPin, uint8_t txPin);
```
| Parameter | Type | Description |
| :--- | :--- | :--- |
| `rxPin` | `uint8_t` | Pin connected to RX serial driver |
| `txPin` | `uint8_t` | Pin connected to TX serial driver |

---

#### Initialize 

```cpp
  void SW_HSRT::init(const unsigned long baudrate = 9600ul);
```

| Parameter | Type | Description |
| :--- | :--- | :--- |
| `baudrate` | `unsigned long` | Communcation speed |

---

#### Checking the sender does not want to establish communication by setting a flag.

```cpp
  bool SW_HSRT::isPeerReady(void);
```

| Retval | If |   
| :--- | :--- |
|  `true` | Sender set communication flag  |
| `false` | Sender did not set the communication flag |

---

#### Write data to the recipient.

```cpp
  int SW_HSRT::write(const uint8_t* buffer, size_t size, const unsigned long timeout = 200ul);
```

| Parameter | Type | Description |
| :--- | :--- | :--- |
| `buffer` | `uint8_t*` | Buffer with data to send |
| `size`  | `size_t` | Buffer size |
| `timeout` | `unsigned long` | Waittime for handshake connection to the recipient |

| Retval | If |   
| :--- | :--- |
|  `0` | Communication was successfully established and data was sent |
| `-1` | The receiving device should set the communication flag first, which means that it should handle the reading of this data first and then send its data |
| `-2` | Data buffer points to nullptr |
| `-3` | The waiting time for connection to the receiving device has expired |

---

#### Read data from the sender.

```cpp
  int SW_HSRT::read(uint8_t* buffer, size_t size);
```

| Parameter | Type | Description |
| :--- | :--- | :--- |
| `buffer` | `uint8_t*` | Buffer to readed data. |
| `size` | `size_t` | Buffer size. |

| Retval | If |
| :--- | :--- |
| `>= 0` | number of bytes read |
| `-1` | sender did not set the communication flag |
| `-2` | data buffer points to nullptr |

---
## Usage/Examples

# Software Handshake Receive / Transmit (SW_HSRT) EXEMPLE

```cpp
#include <hsrt.hpp>

SW_HSRT serial{2, 3};

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
