#pragma once

#include <Arduino.h>
#include <SoftwareSerial.h>

class HW_HSRT
{
public:
  HW_HSRT(HardwareSerial& serial, uint8_t rx, uint8_t tx);
  /**
   * Inicjalizacja obiektu.
   * @param baudrate szybkosc komunikacji
   * @retval None
   */
  void init(uint16_t baudrate);
  /**
   * Odczyt danych.
   * @param buffer bufor pamięci, do którego zostaną zapisane odczytane dane
   * @param size rozmiar bufora
   * @retval -2 wskaznik do bufora danych jest pust
   * @retval -1 urządzenie nadawcze nie ma wysłało sygnału chęci komunikacji
   * @retval >= 0 ilość odebranych bajtów danych.
   */
  int read(uint8_t* buffer, uint16_t size);
  /**
   * Wysyłanie danych.
   * @param buffer bufor pamięci, z którego zostaną wysłane dane
   * @param size ilość danych do wysłania w bajtach
   * @param timeout czas, po którym zakończy się oczekiwanie na odpowiedź od urządzenia odbiorczego
   * @retval -3 upłynął czas oczekiwania na odpowiedź od urządzenia odbiorczego
   * @retval -2 wskaznik do bufora danych jest pust
   * @retval -1 urządzenie odbiorcze nadało sygnał chęci przesyłu danych (należy obsłużyć ich odczyt, a później wysłać swoje dane)
   * @retval 0 pomyślnie wysłano dane.
   */
  int write(uint8_t* buffer, uint16_t size, unsigned long timeout = 200UL);
  bool isPeerReady(void) const;

private:
  uint8_t rxPin;
  uint8_t txPin;
  uint16_t baudrate;
  float waitByteTime;
  HardwareSerial& serial;
};

#ifndef __AVR_ATmega2560__

class SW_HSRT
{
public:
  SW_HSRT(uint8_t rx, uint8_t tx);
  /**
   * Inicjalizacja obiektu.
   * @param baudrate szybkosc komunikacji
   * @retval None
   */
  void init(uint16_t baudrate);
  /**
   * Odczyt danych.
   * @param buffer bufor pamięci, do którego zostaną zapisane odczytane dane
   * @param size rozmiar bufora
   * @retval -2 wskaznik do bufora danych jest pust
   * @retval -1 urządzenie nadawcze nie ma wysłało sygnału chęci komunikacji
   * @retval >= 0 ilość odebranych bajtów danych.
   */
  int read(uint8_t* buffer, uint16_t size);
  /**
   * Wysyłanie danych.
   * @param buffer bufor pamięci, z którego zostaną wysłane dane
   * @param size ilość danych do wysłania w bajtach
   * @param timeout czas, po którym zakończy się oczekiwanie na odpowiedź od urządzenia odbiorczego
   * @retval -3 upłynął czas oczekiwania na odpowiedź od urządzenia odbiorczego
   * @retval -2 wskaznik do bufora danych jest pust
   * @retval -1 urządzenie odbiorcze nadało sygnał chęci przesyłu danych (należy obsłużyć ich odczyt, a później wysłać swoje dane)
   * @retval 0 pomyślnie wysłano dane.
   */
  int write(uint8_t* buffer, uint16_t size, unsigned long timeout = 200UL);
  bool isPeerReady(void) const;

private:
  uint8_t rxPin;
  uint8_t txPin;
  uint16_t baudrate;
  float waitByteTime;
  SoftwareSerial serial;
};

#endif
