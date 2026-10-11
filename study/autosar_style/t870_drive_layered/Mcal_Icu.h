// ============================================================
// MCAL Icu: 엔코더 인터럽트(D2)와 Pixhawk PWM 펄스 캡처(D8)
// ============================================================

#ifndef MCAL_ICU_H
#define MCAL_ICU_H

#include "Std_Types.h"

void Mcal_Icu_InitEncoder();
void Mcal_Icu_InitPixhawkCapture();

// 인터럽트를 막고 읽는다. 제어 계산용.
long Mcal_Icu_GetEncoderCount();

// 원본 printStatus() 처럼 인터럽트를 막지 않고 읽는다. 상태 표시용.
long Mcal_Icu_GetEncoderCountUnprotected();

// 마지막 유효 펄스폭을 pulseUs 에 넣고, 펄스가 아직 살아 있으면 true.
bool Mcal_Icu_ReadPixhawkPwm(unsigned int &pulseUs);

#endif
