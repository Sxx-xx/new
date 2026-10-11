#include "Rte.h"

#include "Mcal_Gpt.h"
#include "Mcal_Icu.h"
#include "Mcal_Adc.h"
#include "Mcal_Port.h"
#include "Mcal_Pwm.h"
#include "Mcal_Uart.h"
#include "Cdd_RemoteRx.h"
#include "Bsw_Watchdog.h"
#include "Bsw_FaultLatch.h"
#include "Swc_SpeedControl.h"
#include "Swc_SteeringControl.h"
#include "Swc_StopHold.h"


// ============================================================
// Sender-receiver 버퍼
//
// 초기값은 원본 전역 변수의 초기값과 같다. 구동 방향은 원본에서
// DRIVE_FORWARD_DIR 로 시작하므로 true 다.
// ============================================================

static float rteSerialSpeedCmdKmh = 0.0;
static float rteCommandSpeedKmh   = 0.0;
static float rteRampedSpeedKmh    = 0.0;
static float rteActualSpeedKmh    = 0.0;
static long  rteEncoderCountDiff  = 0;

static bool  rteDriveDirForward        = true;
static bool  rteOpenLoopMode           = false;
static bool  rteManualNeutralRequired  = true;
static bool  rteReceiverFailsafeActive = true;
static unsigned long rteModeChangeStopUntilMs = 0;
static bool  rteStopHoldOverride       = false;
static int   rteDrivePwm               = 0;

static bool  rteSteerOpenLoop    = false;
static int   rteSteerTarget      = 0;   // Swc_SteeringControl_Init() 이 중앙값을 쓴다
static bool  rtePixhawkPwmValid  = false;
static unsigned int rtePixhawkPwmUs = 0;


// ---- 보조 RC 수신기 ----
bool     Rte_Read_Remote_Valid()           { return Cdd_RemoteRx_IsValid(); }
bool     Rte_Read_Remote_AutoMode()        { return Cdd_RemoteRx_IsAutoMode(); }
int      Rte_Read_Remote_ThrottleCommand() { return Cdd_RemoteRx_GetThrottleCommand(); }
int      Rte_Read_Remote_SteeringCommand() { return Cdd_RemoteRx_GetSteeringCommand(); }
uint16_t Rte_Read_Remote_ThrottleRaw()     { return Cdd_RemoteRx_GetThrottleRaw(); }
uint16_t Rte_Read_Remote_SteeringRaw()     { return Cdd_RemoteRx_GetSteeringRaw(); }

// ---- 공통 서비스 상태 ----
bool Rte_Read_Watchdog_Enabled()      { return Bsw_Watchdog_IsEnabled(); }
bool Rte_Read_FaultLatch_SteerFault() { return Bsw_FaultLatch_IsSteerFault(); }
bool Rte_Read_FaultLatch_SteerStall() { return Bsw_FaultLatch_IsSteerStall(); }

// ---- SWC 사이의 신호 ----
float Rte_Read_SerialSpeedCmdKmh()             { return rteSerialSpeedCmdKmh; }
void  Rte_Write_SerialSpeedCmdKmh(float value) { rteSerialSpeedCmdKmh = value; }
float Rte_Read_CommandSpeedKmh()               { return rteCommandSpeedKmh; }
void  Rte_Write_CommandSpeedKmh(float value)   { rteCommandSpeedKmh = value; }
float Rte_Read_RampedSpeedKmh()                { return rteRampedSpeedKmh; }
void  Rte_Write_RampedSpeedKmh(float value)    { rteRampedSpeedKmh = value; }
float Rte_Read_ActualSpeedKmh()                { return rteActualSpeedKmh; }
void  Rte_Write_ActualSpeedKmh(float value)    { rteActualSpeedKmh = value; }
long  Rte_Read_EncoderCountDiff()              { return rteEncoderCountDiff; }
void  Rte_Write_EncoderCountDiff(long value)   { rteEncoderCountDiff = value; }

bool  Rte_Read_DriveDirForward()                   { return rteDriveDirForward; }
void  Rte_Write_DriveDirForward(bool value)        { rteDriveDirForward = value; }
bool  Rte_Read_OpenLoopMode()                      { return rteOpenLoopMode; }
void  Rte_Write_OpenLoopMode(bool value)           { rteOpenLoopMode = value; }
bool  Rte_Read_ManualNeutralRequired()             { return rteManualNeutralRequired; }
void  Rte_Write_ManualNeutralRequired(bool value)  { rteManualNeutralRequired = value; }
bool  Rte_Read_ReceiverFailsafeActive()            { return rteReceiverFailsafeActive; }
void  Rte_Write_ReceiverFailsafeActive(bool value) { rteReceiverFailsafeActive = value; }
unsigned long Rte_Read_ModeChangeStopUntilMs()     { return rteModeChangeStopUntilMs; }
void  Rte_Write_ModeChangeStopUntilMs(unsigned long value) { rteModeChangeStopUntilMs = value; }
bool  Rte_Read_StopHoldOverride()                  { return rteStopHoldOverride; }
void  Rte_Write_StopHoldOverride(bool value)       { rteStopHoldOverride = value; }
int   Rte_Read_DrivePwm()                          { return rteDrivePwm; }

bool  Rte_Read_SteerOpenLoop()                     { return rteSteerOpenLoop; }
void  Rte_Write_SteerOpenLoop(bool value)          { rteSteerOpenLoop = value; }
int   Rte_Read_SteerTarget()                       { return rteSteerTarget; }
void  Rte_Write_SteerTarget(int value)             { rteSteerTarget = value; }
bool  Rte_Read_PixhawkPwmValid()                   { return rtePixhawkPwmValid; }
void  Rte_Write_PixhawkPwmValid(bool value)        { rtePixhawkPwmValid = value; }
unsigned int Rte_Read_PixhawkPwmUs()               { return rtePixhawkPwmUs; }
void  Rte_Write_PixhawkPwmUs(unsigned int value)   { rtePixhawkPwmUs = value; }


// ---- 하드웨어 ----
unsigned long Rte_Call_Time_GetMs()   { return Mcal_Gpt_GetMs(); }
long Rte_Call_Encoder_GetCount()      { return Mcal_Icu_GetEncoderCount(); }
int  Rte_Call_SteerPot_Read()         { return Mcal_Adc_ReadSteerPot(); }
bool Rte_Call_Pixhawk_ReadPwm(unsigned int &pulseUs) { return Mcal_Icu_ReadPixhawkPwm(pulseUs); }

void Rte_Call_DriveMotor_SetDirection(bool forward) { Mcal_Port_SetDriveDirection(forward); }

// 원본의 drivePWMValue = x; analogWrite(DRIVE_PWM, x); 쌍이다.
void Rte_Call_DriveMotor_SetPwm(int duty)
{
  rteDrivePwm = duty;
  Mcal_Pwm_SetDriveDuty(duty);
}

void Rte_Call_SteerMotor_SetDirection(bool potUp) { Mcal_Port_SetSteerDirection(potUp); }
void Rte_Call_SteerMotor_SetPwm(int duty)         { Mcal_Pwm_SetSteerDuty(duty); }


// ---- 공통 서비스 ----
void Rte_Call_FaultLatch_SetSteerFault() { Bsw_FaultLatch_SetSteerFault(); }
void Rte_Call_FaultLatch_SetSteerStall() { Bsw_FaultLatch_SetSteerStall(); }
void Rte_Call_Watchdog_Kick(unsigned long now) { Bsw_Watchdog_Kick(now); }
void Rte_Call_Console_WriteText(const char *text)              { Mcal_Uart_WriteText(text); }
void Rte_Call_Console_WriteLong(long value)                    { Mcal_Uart_WriteLong(value); }
void Rte_Call_Console_WriteFloat(float value, uint8_t digits)  { Mcal_Uart_WriteFloat(value, digits); }
void Rte_Call_Console_WriteNewline()                           { Mcal_Uart_WriteNewline(); }


// ---- SWC 사이의 요청 ----
void Rte_Call_SpeedControl_ResetRamp(float rampedSpeedKmh) { Swc_SpeedControl_ResetRamp(rampedSpeedKmh); }
void Rte_Call_SpeedControl_CancelOpenLoop()                { Swc_SpeedControl_CancelOpenLoop(); }
void Rte_Call_StopHold_Release()                           { Swc_StopHold_Release(); }


// ---- 보정 상수 ----
int   Rte_Prm_SteerPotCenter()  { return Swc_SteeringControl_GetPotCenter(); }
int   Rte_Prm_SteerPotSoftMin() { return Swc_SteeringControl_GetPotSoftMin(); }
int   Rte_Prm_SteerPotSoftMax() { return Swc_SteeringControl_GetPotSoftMax(); }
float Rte_Prm_MaxSpeedKmh()     { return Swc_SpeedControl_GetMaxSpeedKmh(); }


// ============================================================
// 이벤트 분배
// ============================================================

// 원본 handleCommand() 의 분기. 명령 하나에 정확히 한 처리 함수가 돈다.
void Rte_Notify_SerialCommand(const Rte_SerialCmdType &cmd)
{
  switch (cmd.kind)
  {
    case RTE_CMD_EMERGENCY_STOP:  Swc_SpeedControl_OnEmergencyStopCmd();             break;
    case RTE_CMD_MISSION_HOLD:    Swc_StopHold_OnHoldCmd();                          break;
    case RTE_CMD_WATCHDOG_TOGGLE: Bsw_Watchdog_Toggle();                             break;
    case RTE_CMD_LATCH_CLEAR:     Bsw_FaultLatch_ClearAll();                         break;
    case RTE_CMD_STEER_OPEN_LOOP: Swc_SteeringControl_OnOpenLoopCmd(cmd.intValue);   break;
    case RTE_CMD_DRIVE_OPEN_LOOP: Swc_SpeedControl_OnOpenLoopCmd(cmd.intValue);      break;
    case RTE_CMD_TARGET_SPEED:    Swc_SpeedControl_OnTargetSpeedCmd(cmd.floatValue); break;
  }
}


// 원본 loop() 의 명령 워치독 만료 블록.
void Rte_Notify_CommandTimeout()
{
  Swc_SpeedControl_OnCommandTimeout();
  Swc_SteeringControl_OnCommandTimeout();
}
