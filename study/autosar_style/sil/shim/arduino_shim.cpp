// ============================================================
// 가짜 Arduino API 구현. 원본 sketch_aug.ino 의 핀 번호를 SIL 코어에 잇는다.
//
//   D2/D3 엔코더 A/B, D7 조향 DIR, D9 조향 PWM, D10 구동 PWM,
//   D11 구동 DIR, A0 조향 POT, D8 Pixhawk (PCINT0)
// ============================================================

#include "../sim_core.h"
#include "Arduino.h"
#include "Wire.h"


uint8_t PINB = 0, PCIFR = 0, PCMSK0 = 0, PCICR = 0, TCCR1B = 0;

HardwareSerial Serial;
TwoWire Wire;

static int pinLevel[20] = {0};
static void (*encoderHandler)(void) = nullptr;


void pinMode(uint8_t, uint8_t) {}

void digitalWrite(uint8_t pin, uint8_t value)
{
  pinLevel[pin] = value ? 1 : 0;
  if (pin == 7)  sim::steerDirLevel = pinLevel[pin];
  if (pin == 11) sim::driveDirLevel = pinLevel[pin];
}

int digitalRead(uint8_t pin)
{
  return pinLevel[pin];
}

int analogRead(uint8_t pin)
{
  return pin == A0 ? sim::potAdc() : 0;
}

void analogWrite(uint8_t pin, int value)
{
  if (pin == 9)  sim::steerPwm = value;
  if (pin == 10) sim::drivePwm = value;
}

unsigned long millis() { return sim::nowMs; }
unsigned long micros() { return sim::micros(); }

void attachInterrupt(uint8_t interruptNum, void (*handler)(void), int)
{
  if (interruptNum == 0) encoderHandler = handler;
}

long map(long x, long inMin, long inMax, long outMin, long outMax)
{
  return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}


// 엔코더 A상 CHANGE 한 번. 원본 ISR 은 A == B 면 감소, 다르면 증가로 센다.
void Backend_EncoderEdge(int direction)
{
  pinLevel[2] = !pinLevel[2];
  pinLevel[3] = direction > 0 ? !pinLevel[2] : pinLevel[2];
  if (encoderHandler) encoderHandler();
}


// D8 상승 · 하강 엣지를 지정 시각에 일으켜 원본 ISR(PCINT0_vect) 을 부른다.
void Backend_PixhawkPulse(unsigned long riseUs, unsigned long fallUs)
{
  sim::setMicrosOverride(true, riseUs);
  PINB |= _BV(PB0);
  PCINT0_vect();
  sim::setMicrosOverride(true, fallUs);
  PINB &= ~_BV(PB0);
  PCINT0_vect();
  sim::setMicrosOverride(false, 0);
}


// ---- Serial ----
void HardwareSerial::begin(unsigned long) {}
int  HardwareSerial::available() { return sim::serialAvailable(); }
int  HardwareSerial::read() { return sim::serialRead(); }

void HardwareSerial::print(const char *text)  { sim::serialWrite(text); }
void HardwareSerial::print(char c)            { sim::serialWrite(std::string(1, c)); }
void HardwareSerial::print(int value)         { sim::serialWriteLong(value); }
void HardwareSerial::print(unsigned int value){ sim::serialWriteULong(value); }
void HardwareSerial::print(long value)        { sim::serialWriteLong(value); }
void HardwareSerial::print(unsigned long value) { sim::serialWriteULong(value); }
void HardwareSerial::print(double value, int digits) { sim::serialWriteFloat(value, digits); }

void HardwareSerial::println()                  { sim::serialWrite("\r\n"); }
void HardwareSerial::println(const char *text)  { print(text);  println(); }
void HardwareSerial::println(int value)         { print(value); println(); }
void HardwareSerial::println(unsigned int value){ print(value); println(); }
void HardwareSerial::println(long value)        { print(value); println(); }
void HardwareSerial::println(unsigned long value) { print(value); println(); }
void HardwareSerial::println(double value, int digits) { print(value, digits); println(); }


// ---- Wire ----
static uint8_t i2cBuffer[32];
static size_t  i2cLength = 0;
static size_t  i2cIndex  = 0;

void TwoWire::begin() {}

uint8_t TwoWire::requestFrom(int, int quantity)
{
  i2cLength = sim::buildRemoteReport(i2cBuffer);
  if (i2cLength > (size_t)quantity) i2cLength = (size_t)quantity;
  i2cIndex = 0;
  return (uint8_t)i2cLength;
}

int TwoWire::available() { return (int)(i2cLength - i2cIndex); }
int TwoWire::read() { return i2cIndex < i2cLength ? i2cBuffer[i2cIndex++] : -1; }
