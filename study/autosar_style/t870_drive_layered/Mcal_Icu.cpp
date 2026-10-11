#include <Arduino.h>
#include "Mcal_Icu.h"
#include "Mcal_Cfg.h"


// Pixhawk 펄스 유효 범위와 단절 판정. 캡처 드라이버의 설정이다.
static const unsigned int  PIXHAWK_PWM_VALID_MIN  = 800;
static const unsigned int  PIXHAWK_PWM_VALID_MAX  = 2200;
static const unsigned long PIXHAWK_PWM_TIMEOUT_US = 150000UL;

static volatile long encoderCount = 0;

static volatile unsigned long pixhawkRiseUs       = 0;
static volatile unsigned long pixhawkLastUpdateUs = 0;
static volatile unsigned int  pixhawkPulseUs      = 0;


// ============================================================
// 엔코더 ISR   (A상 CHANGE, 2체배)
//
// CHANGE 라 A의 상승/하강 양쪽에서 들어온다.
// A == B 인지로 방향을 판별하면 두 엣지 모두에서 일관된다.
// ============================================================
static void encoderISR()
{
  bool a = digitalRead(MCAL_PIN_ENCODER_A);
  bool b = digitalRead(MCAL_PIN_ENCODER_B);

  if (a == b)
  {
    encoderCount--;
  }
  else
  {
    encoderCount++;
  }
}


// D8 = PB0 = PCINT0. 상승 시각을 저장하고 하강 시 HIGH 펄스폭을 만든다.
ISR(PCINT0_vect)
{
  unsigned long nowUs = micros();

  if (PINB & _BV(PB0))
  {
    pixhawkRiseUs = nowUs;
  }
  else
  {
    unsigned long widthUs = nowUs - pixhawkRiseUs;

    if (widthUs >= PIXHAWK_PWM_VALID_MIN &&
        widthUs <= PIXHAWK_PWM_VALID_MAX)
    {
      pixhawkPulseUs      = (unsigned int)widthUs;
      pixhawkLastUpdateUs = nowUs;
    }
  }
}


void Mcal_Icu_InitEncoder()
{
  attachInterrupt(digitalPinToInterrupt(MCAL_PIN_ENCODER_A), encoderISR, CHANGE);
}


void Mcal_Icu_InitPixhawkCapture()
{
  // D8(PB0/PCINT0) 핀 체인지 인터럽트 활성화.
  PCIFR  |= _BV(PCIF0);    // 남아 있는 인터럽트 플래그 제거
  PCMSK0 |= _BV(PCINT0);   // D8만 감시
  PCICR  |= _BV(PCIE0);    // PORT B 핀 체인지 인터럽트 허용
}


long Mcal_Icu_GetEncoderCount()
{
  noInterrupts();
  long count = encoderCount;
  interrupts();
  return count;
}


long Mcal_Icu_GetEncoderCountUnprotected()
{
  return encoderCount;
}


bool Mcal_Icu_ReadPixhawkPwm(unsigned int &pulseUs)
{
  unsigned long lastUpdateUs;

  noInterrupts();
  pulseUs      = pixhawkPulseUs;
  lastUpdateUs = pixhawkLastUpdateUs;
  interrupts();

  if (lastUpdateUs == 0)
  {
    return false;
  }

  return (micros() - lastUpdateUs) <= PIXHAWK_PWM_TIMEOUT_US;
}
