#include <Arduino.h>
#include "Mcal_Pwm.h"
#include "Mcal_Cfg.h"


// ------------------------------------------------------------
// Timer1 PWM 주파수 -> 약 3.9kHz
//
// D9(조향) / D10(구동) 이 같은 Timer1 이라 한 줄로 둘 다 잡힌다.
// 기본값 488Hz 는 모터에서 가청소음이 난다.
// 0x01 은 31.25kHz 인데 MD20A/MDD20A 상한 20kHz 를 넘는다.
// millis() 는 Timer0 이라 영향 없다.
// ------------------------------------------------------------
void Mcal_Pwm_Init()
{
  TCCR1B = (TCCR1B & 0b11111000) | 0x02;
}


void Mcal_Pwm_SetSteerDuty(int duty)
{
  analogWrite(MCAL_PIN_STEER_PWM, duty);
}


void Mcal_Pwm_SetDriveDuty(int duty)
{
  analogWrite(MCAL_PIN_DRIVE_PWM, duty);
}
