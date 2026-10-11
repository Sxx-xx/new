// ============================================================
// PC용 MCAL 모의 구현
//
// t870_drive_layered 의 Mcal_*.cpp 일곱 개를 이 파일 하나로 바꾼다.
// 헤더(Mcal_*.h)는 그대로 쓰므로 Mcal 위 계층(Cdd, Bsw, Rte, Swc)은
// 한 줄도 바꾸지 않고 PC 에서 돈다.
// ============================================================

#include "sim_core.h"

#include "Mcal_Port.h"
#include "Mcal_Pwm.h"
#include "Mcal_Adc.h"
#include "Mcal_Icu.h"
#include "Mcal_Gpt.h"
#include "Mcal_I2c.h"
#include "Mcal_Uart.h"


// Mcal_Cfg.h 의 극성과 같다. 구동 HIGH = 전진, 조향 LOW = POT 증가.
static const int DRIVE_FORWARD_LEVEL = 1;
static const int DRIVE_REVERSE_LEVEL = 0;
static const int STEER_POT_UP_LEVEL   = 0;
static const int STEER_POT_DOWN_LEVEL = 1;


// ---- Port ----
void Mcal_Port_Init()
{
  sim::steerPwm = 0;
  sim::driveDirLevel = DRIVE_FORWARD_LEVEL;
  sim::drivePwm = 0;
}

void Mcal_Port_SetSteerDirection(bool potUp)
{
  sim::steerDirLevel = potUp ? STEER_POT_UP_LEVEL : STEER_POT_DOWN_LEVEL;
}

void Mcal_Port_SetDriveDirection(bool forward)
{
  sim::driveDirLevel = forward ? DRIVE_FORWARD_LEVEL : DRIVE_REVERSE_LEVEL;
}


// ---- Pwm ----
void Mcal_Pwm_Init() {}
void Mcal_Pwm_SetSteerDuty(int duty) { sim::steerPwm = duty; }
void Mcal_Pwm_SetDriveDuty(int duty) { sim::drivePwm = duty; }


// ---- Adc ----
int Mcal_Adc_ReadSteerPot() { return sim::potAdc(); }


// ---- Icu ----
// 캡처 로직은 실기판용 Mcal_Icu.cpp 와 같다. 인터럽트 대신 코어가 직접 부른다.
static const unsigned int  PIXHAWK_PWM_VALID_MIN  = 800;
static const unsigned int  PIXHAWK_PWM_VALID_MAX  = 2200;
static const unsigned long PIXHAWK_PWM_TIMEOUT_US = 150000UL;

static long encoderCount = 0;
static unsigned long pixhawkLastUpdateUs = 0;
static unsigned int  pixhawkPulseUs      = 0;

void Backend_EncoderEdge(int direction)
{
  encoderCount += direction;
}

void Backend_PixhawkPulse(unsigned long riseUs, unsigned long fallUs)
{
  unsigned long widthUs = fallUs - riseUs;
  if (widthUs >= PIXHAWK_PWM_VALID_MIN && widthUs <= PIXHAWK_PWM_VALID_MAX)
  {
    pixhawkPulseUs      = (unsigned int)widthUs;
    pixhawkLastUpdateUs = fallUs;
  }
}

void Mcal_Icu_InitEncoder() {}
void Mcal_Icu_InitPixhawkCapture() {}
long Mcal_Icu_GetEncoderCount() { return encoderCount; }
long Mcal_Icu_GetEncoderCountUnprotected() { return encoderCount; }

bool Mcal_Icu_ReadPixhawkPwm(unsigned int &pulseUs)
{
  pulseUs = pixhawkPulseUs;
  if (pixhawkLastUpdateUs == 0)
  {
    return false;
  }
  return (sim::micros() - pixhawkLastUpdateUs) <= PIXHAWK_PWM_TIMEOUT_US;
}


// ---- Gpt ----
unsigned long Mcal_Gpt_GetMs() { return sim::nowMs; }


// ---- I2c ----
static uint8_t i2cBuffer[32];
static size_t  i2cLength = 0;
static size_t  i2cIndex  = 0;

void Mcal_I2c_Init() {}

uint8_t Mcal_I2c_RequestFrom(uint8_t address, uint8_t length)
{
  (void)address;
  i2cLength = sim::buildRemoteReport(i2cBuffer);
  if (i2cLength > length) i2cLength = length;
  i2cIndex = 0;
  return (uint8_t)i2cLength;
}

int Mcal_I2c_Available() { return (int)(i2cLength - i2cIndex); }
int Mcal_I2c_Read() { return i2cIndex < i2cLength ? i2cBuffer[i2cIndex++] : -1; }


// ---- Uart ----
void Mcal_Uart_Init() {}
int  Mcal_Uart_Available() { return sim::serialAvailable(); }
int  Mcal_Uart_Read() { return sim::serialRead(); }
void Mcal_Uart_WriteText(const char *text) { sim::serialWrite(text); }
void Mcal_Uart_WriteLong(long value) { sim::serialWriteLong(value); }
void Mcal_Uart_WriteULong(unsigned long value) { sim::serialWriteULong(value); }
void Mcal_Uart_WriteFloat(float value, uint8_t digits) { sim::serialWriteFloat(value, digits); }
void Mcal_Uart_WriteNewline() { sim::serialWrite("\r\n"); }
