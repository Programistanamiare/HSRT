#include "hsrt.hpp"


HW_HSRT::HW_HSRT(HardwareSerial& serial, uint8_t rx, uint8_t tx)
: rxPin{rx},
  txPin{tx},
  baudrate{9600},
  waitByteTime{0.f},
  serial{serial}
{}


void HW_HSRT::init(uint16_t baudrate)
{
  this->baudrate = baudrate;
  serial.end();
  pinMode(txPin, OUTPUT);
  digitalWrite(txPin, LOW);
  pinMode(rxPin, INPUT);
  waitByteTime = (10.f / baudrate) * 1000.f; 
}

bool HW_HSRT::isPeerReady(void) const
{
  return HIGH == digitalRead(rxPin);
}

int HW_HSRT::write(uint8_t* buffer, uint16_t size, unsigned long timeout)
{
  if (nullptr == buffer) return -2; // sprawdzenie czy wskaznik do bufora danych nie jest pusty
  if (isPeerReady()) return -1; // sprawdzenie czy podlaczone urzadzenie nie chce nadawac
  digitalWrite(txPin, HIGH); // wyslanie sygnalu chęci nadawania
  unsigned long t0 = millis(); 
  while(!isPeerReady()) // dopoki urzadzenie odbiorcze nie odpowie
  {
    if (millis() - t0 > timeout) // jesli minal czas oczekiwania na odpowiedz
    {
      digitalWrite(txPin, LOW); // odwolanie sygnału chęci nadawania
      return -3;
    }
  }
  serial.begin(baudrate); // uruchomienie portu szeregowego
  serial.write(buffer, size); // wyslanie danych z bufora
  serial.flush(); // odczekanie na wyslanie wszystkich danych z bufora
  serial.end(); // wylaczenie portu szeregowego
  digitalWrite(txPin, LOW); // zresetowanie sygnalu chęci nadawania
  delay(2); // nie ruszaj tego to musi kurwa być!!!
  return 0;
}

int HW_HSRT::read(uint8_t* buffer, uint16_t size)
{
  if (nullptr == buffer) return -2; // sprawdznie czy wskaznik do bufora danych nie jest pusty
  if (!isPeerReady()) return -1; // sprawdzenie czy podlaczone urzadzenie chce nadawac
  serial.begin(baudrate); // wlaczenie portu szeregowego
  digitalWrite(txPin, HIGH); // wyslanie sygnalu odpowiedzi gotowości do odbioru danych
  delay(waitByteTime * size + 1); // odczekanie na wyslanie przez nadawcę danych
  size_t bytesRead = 0;
  while (serial.available() > 0)
  {
    buffer[bytesRead++] = serial.read();
  }
  serial.end(); // wylaczenie portu szeregowego
  digitalWrite(txPin, LOW); // zresetowanie sygnału gotowości odbioru
  return bytesRead;
}

#ifndef __AVR_ATmega2560__

SW_HSRT::SW_HSRT(uint8_t rx, uint8_t tx)
: rxPin{rx},
  txPin{tx},
  baudrate{9600},
  waitByteTime{0.f},
  serial{rx, tx}
{}


void SW_HSRT::init(uint16_t baudrate)
{
  this->baudrate = baudrate;
  serial.end();
  pinMode(txPin, OUTPUT);
  digitalWrite(txPin, LOW);
  pinMode(rxPin, INPUT);
  waitByteTime = (10.f / baudrate) * 1000.f;
  serial.setTimeout(10);
}

bool SW_HSRT::isPeerReady(void) const
{
  return HIGH == digitalRead(rxPin);
}

int SW_HSRT::write(uint8_t* buffer, uint16_t size, unsigned long timeout)
{
  if (nullptr == buffer) return -2; // sprawdzenie czy wskaznik do bufora danych nie jest pusty
  if (isPeerReady()) return -1; // sprawdzenie czy podlaczone urzadzenie nie chce nadawac
  digitalWrite(txPin, HIGH); // wyslanie sygnalu chęci nadawania
  unsigned long t0 = millis(); 
  while(!isPeerReady()) // dopoki urzadzenie odbiorcze nie odpowie
  {
    if (millis() - t0 > timeout) // jesli minal czas oczekiwania na odpowiedz
    {
      digitalWrite(txPin, LOW); // odwolanie sygnału chęci nadawania
      return -3;
    }
  }
  serial.begin(baudrate); // uruchomienie portu szeregowego
  serial.write(buffer, size); // wyslanie danych z bufora
  serial.end(); // wylaczenie portu szeregowego
  digitalWrite(txPin, LOW); // zresetowanie sygnalu chęci nadawania
  delay(2); // nie ruszaj tego to musi kurwa być!!!
  return 0;
}

int SW_HSRT::read(uint8_t* buffer, uint16_t size)
{
  if (nullptr == buffer) return -2; // sprawdznie czy wskaznik do bufora danych nie jest pusty
  if (!isPeerReady()) return -1; // sprawdzenie czy podlaczone urzadzenie chce nadawac
  serial.begin(baudrate); // wlaczenie portu szeregowego
  digitalWrite(txPin, HIGH); // wyslanie sygnalu odpowiedzi gotowości do odbioru danych
  interrupts();
  size_t bytesRead = serial.readBytes(buffer, size); // odczytanie danych do bufora
  digitalWrite(txPin, LOW); // zresetowanie sygnału gotowości odbioru
  serial.end(); // wylaczenie portu szeregowego
  return bytesRead;
}

#endif
