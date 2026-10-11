#include <Arduino.h>
#include "Mcal_Port.h"
#include "Mcal_Cfg.h"
#include "Mcal_Icu.h"
#include "Mcal_Pwm.h"


// 원본 setup() 의 핀 설정 순서를 그대로 따른다.
void Mcal_Port_Init()
{
  pinMode(MCAL_PIN_ENCODER_A, INPUT_PULLUP);
  pinMode(MCAL_PIN_ENCODER_B, INPUT_PULLUP);

  Mcal_Icu_InitEncoder();

  pinMode(MCAL_PIN_STEER_DIR, OUTPUT);
  pinMode(MCAL_PIN_STEER_PWM, OUTPUT);
  Mcal_Pwm_SetSteerDuty(0);

  pinMode(MCAL_PIN_DRIVE_DIR, OUTPUT);
  pinMode(MCAL_PIN_DRIVE_PWM, OUTPUT);
  digitalWrite(MCAL_PIN_DRIVE_DIR, MCAL_DRIVE_DIR_FORWARD_LEVEL);
  Mcal_Pwm_SetDriveDuty(0);

  pinMode(MCAL_PIN_PIXHAWK_PWM, INPUT);
}


void Mcal_Port_SetSteerDirection(bool potUp)
{
  digitalWrite(MCAL_PIN_STEER_DIR,
               potUp ? MCAL_STEER_DIR_POT_UP_LEVEL
                     : MCAL_STEER_DIR_POT_DOWN_LEVEL);
}


void Mcal_Port_SetDriveDirection(bool forward)
{
  digitalWrite(MCAL_PIN_DRIVE_DIR,
               forward ? MCAL_DRIVE_DIR_FORWARD_LEVEL
                       : MCAL_DRIVE_DIR_REVERSE_LEVEL);
}
