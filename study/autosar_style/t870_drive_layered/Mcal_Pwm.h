// ============================================================
// MCAL Pwm: Timer1 (D9 조향, D10 구동)
// ============================================================

#ifndef MCAL_PWM_H
#define MCAL_PWM_H

#include "Std_Types.h"

void Mcal_Pwm_Init();
void Mcal_Pwm_SetSteerDuty(int duty);
void Mcal_Pwm_SetDriveDuty(int duty);

#endif
