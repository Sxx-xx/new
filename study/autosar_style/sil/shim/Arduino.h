// ============================================================
// PC용 가짜 Arduino.h
//
// 원본 sketch_aug.ino 를 한 글자도 고치지 않고 g++ 로 빌드하기 위한 것이다.
// 원본이 쓰는 API 만 흉내 낸다. 값은 SIL 코어(sim_core)로 연결된다.
// 매크로 정의는 AVR 코어 1.8.x 의 Arduino.h 와 같다.
// ============================================================

#ifndef SIL_FAKE_ARDUINO_H
#define SIL_FAKE_ARDUINO_H

#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>

typedef uint8_t byte;

#define HIGH 0x1
#define LOW  0x0

#define INPUT        0x0
#define OUTPUT       0x1
#define INPUT_PULLUP 0x2

#define CHANGE 1

#define A0 14

#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#define abs(x) ((x)>0?(x):-(x))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

#define _BV(bit) (1 << (bit))
#define digitalPinToInterrupt(p) ((p) == 2 ? 0 : ((p) == 3 ? 1 : -1))

// ---- AVR 레지스터 (원본이 건드리는 것만) ----
extern uint8_t PINB, PCIFR, PCMSK0, PCICR, TCCR1B;
#define PB0    0
#define PCIF0  0
#define PCINT0 0
#define PCIE0  0

// ISR(PCINT0_vect) 는 이름이 PCINT0_vect 인 일반 함수가 된다. 코어가 직접 부른다.
#define ISR(vector) void vector(void)
void PCINT0_vect(void);

void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t value);
int  digitalRead(uint8_t pin);
int  analogRead(uint8_t pin);
void analogWrite(uint8_t pin, int value);
unsigned long millis();
unsigned long micros();
void attachInterrupt(uint8_t interruptNum, void (*handler)(void), int mode);
inline void noInterrupts() {}
inline void interrupts() {}
long map(long x, long inMin, long inMax, long outMin, long outMax);
inline bool isDigit(int c) { return isdigit(c) != 0; }


class HardwareSerial
{
public:
  void begin(unsigned long baud);
  int  available();
  int  read();

  void print(const char *text);
  void print(char c);
  void print(int value);
  void print(unsigned int value);
  void print(long value);
  void print(unsigned long value);
  void print(double value, int digits = 2);

  void println();
  void println(const char *text);
  void println(int value);
  void println(unsigned int value);
  void println(long value);
  void println(unsigned long value);
  void println(double value, int digits = 2);
};

extern HardwareSerial Serial;

#endif
