// ============================================================
// T870 구동 펌웨어 - AUTOSAR Classic 계층 흉내 (학습용)
//
// 실차에 올리지 않은 구조 연습이다. 원본은 sketch_aug/sketch_aug.ino.
// 이 파일은 OS 스케줄러 자리다. 초기화 순서와 주기 태스크 호출만 둔다.
// 계층 설명은 같은 폴더의 README.md.
// ============================================================

#include "Mcal_Port.h"
#include "Mcal_Pwm.h"
#include "Mcal_Icu.h"
#include "Mcal_Gpt.h"
#include "Mcal_I2c.h"
#include "Mcal_Uart.h"
#include "Cdd_RemoteRx.h"
#include "Cdd_SerialCmd.h"
#include "Cdd_Telemetry.h"
#include "Bsw_Watchdog.h"
#include "Swc_SpeedControl.h"
#include "Swc_SteeringControl.h"
#include "Swc_StopHold.h"


const unsigned long SPEED_INTERVAL_MS = 100;
const unsigned long PRINT_INTERVAL_MS = 200;

unsigned long previousSpeedTime = 0;
unsigned long previousPrintTime = 0;


void setup()
{
  Mcal_Uart_Init();

  Swc_SteeringControl_Init();
  Swc_SpeedControl_Init();

  Mcal_Port_Init();               // 핀 방향, 엔코더 인터럽트, PWM 0
  Mcal_I2c_Init();                // 보조 RC 보드(0x12) 마스터
  Mcal_Icu_InitPixhawkCapture();  // D8 핀 체인지 인터럽트
  Mcal_Pwm_Init();                // Timer1 3.9kHz

  unsigned long now = Mcal_Gpt_GetMs();

  previousSpeedTime = now;
  previousPrintTime = now;
  Bsw_Watchdog_Init(now);

  Cdd_Telemetry_PrintBanner();
}


void loop()
{
  Cdd_SerialCmd_MainFunction();

  unsigned long now = Mcal_Gpt_GetMs();

  Cdd_RemoteRx_MainFunction(now);
  Bsw_Watchdog_MainFunction(now);

  Swc_SpeedControl_SelectCommand(now);
  Swc_SteeringControl_Run(now);


  // 100 ms 구동 태스크. 순서가 곧 원본 driveControl() 의 순서다.
  if (now - previousSpeedTime >= SPEED_INTERVAL_MS)
  {
    float dt = (now - previousSpeedTime) / 1000.0;
    previousSpeedTime = now;

    Swc_SpeedControl_MeasureSpeed(dt);
    Swc_StopHold_Run(dt);
    Swc_SpeedControl_Run(dt);
  }


  // 200 ms 상태 출력 태스크
  if (now - previousPrintTime >= PRINT_INTERVAL_MS)
  {
    previousPrintTime = now;

    Cdd_Telemetry_MainFunction();
  }
}
