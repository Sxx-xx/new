// ============================================================
// RTE (Runtime Environment)
//
// SWC 는 이 헤더만 include 한다. SWC 가 다른 SWC, BSW, MCAL 에
// 닿는 길은 아래 함수뿐이다.
//
//   Rte_Read_xxx / Rte_Write_xxx : sender-receiver. 값은 Rte.cpp 버퍼에 있다
//   Rte_Call_xxx                 : client-server. 다른 계층의 동작을 요청한다
//   Rte_Prm_xxx                  : 보정 상수 읽기
//   Rte_Notify_xxx               : BSW 가 SWC 의 이벤트 처리 함수를 깨운다
//
// 실제 AUTOSAR 에서는 이 파일을 ARXML 설정으로 생성한다. 여기서는 손으로 쓴다.
// ============================================================

#ifndef RTE_H
#define RTE_H

#include "Std_Types.h"


// ------------------------------------------------------------
// 구동 명령 (Cdd_CanGateway 가 DriveCmd 신호를 바꿔 Rte_Notify_SerialCommand 로 넘긴다)
// 이름은 Swc 를 고치지 않으려고 t870_drive_layered 의 것을 그대로 쓴다.
// ------------------------------------------------------------
enum Rte_SerialCmdKind
{
  RTE_CMD_EMERGENCY_STOP,   // "e"
  RTE_CMD_MISSION_HOLD,     // "h"
  RTE_CMD_WATCHDOG_TOGGLE,  // "b..."
  RTE_CMD_LATCH_CLEAR,      // "r..."
  RTE_CMD_STEER_OPEN_LOOP,  // "s<duty>"
  RTE_CMD_DRIVE_OPEN_LOOP,  // "p<duty>"
  RTE_CMD_TARGET_SPEED      // "<km/h>"
};

struct Rte_SerialCmdType
{
  Rte_SerialCmdKind kind;
  int   intValue;     // s, p 의 atoi 결과 (범위 제한 전)
  float floatValue;   // 목표 속도의 atof 결과 (범위 제한 전)
};


// ------------------------------------------------------------
// Sender-receiver: 보조 RC 수신기 (Cdd_RemoteRx 제공)
// ------------------------------------------------------------
bool     Rte_Read_Remote_Valid();
bool     Rte_Read_Remote_AutoMode();
int      Rte_Read_Remote_ThrottleCommand();
int      Rte_Read_Remote_SteeringCommand();
uint16_t Rte_Read_Remote_ThrottleRaw();
uint16_t Rte_Read_Remote_SteeringRaw();

// ------------------------------------------------------------
// Sender-receiver: 공통 서비스 상태 (Bsw 제공)
// ------------------------------------------------------------
bool Rte_Read_Watchdog_Enabled();
bool Rte_Read_Watchdog_Expired();
bool Rte_Read_FaultLatch_SteerFault();
bool Rte_Read_FaultLatch_SteerStall();
bool Rte_Read_FaultLatch_E2e();

// ------------------------------------------------------------
// Sender-receiver: SWC 사이의 신호 (Rte.cpp 버퍼)
// ------------------------------------------------------------
float Rte_Read_SerialSpeedCmdKmh();               // AUTO 시리얼 목표
void  Rte_Write_SerialSpeedCmdKmh(float value);
float Rte_Read_CommandSpeedKmh();                 // 선택된 AUTO/MANUAL 목표
void  Rte_Write_CommandSpeedKmh(float value);
float Rte_Read_RampedSpeedKmh();                  // 램프를 거친 목표
void  Rte_Write_RampedSpeedKmh(float value);
float Rte_Read_ActualSpeedKmh();                  // 부호 있는 측정 속도
void  Rte_Write_ActualSpeedKmh(float value);
long  Rte_Read_EncoderCountDiff();                // 100 ms 카운트 변화
void  Rte_Write_EncoderCountDiff(long value);

bool  Rte_Read_DriveDirForward();                 // 현재 구동 방향
void  Rte_Write_DriveDirForward(bool value);
bool  Rte_Read_OpenLoopMode();                    // 'p' 구동 개루프
void  Rte_Write_OpenLoopMode(bool value);
bool  Rte_Read_ManualNeutralRequired();
void  Rte_Write_ManualNeutralRequired(bool value);
bool  Rte_Read_ReceiverFailsafeActive();
void  Rte_Write_ReceiverFailsafeActive(bool value);
unsigned long Rte_Read_ModeChangeStopUntilMs();
void  Rte_Write_ModeChangeStopUntilMs(unsigned long value);
bool  Rte_Read_StopHoldOverride();                // 이번 주기 구동 출력을 StopHold 가 정했나
void  Rte_Write_StopHoldOverride(bool value);
int   Rte_Read_DrivePwm();                        // 마지막으로 낸 구동 duty
int   Rte_Read_SteerPwm();                        // 마지막으로 낸 조향 duty

bool  Rte_Read_SteerOpenLoop();                   // 's' 조향 개루프
void  Rte_Write_SteerOpenLoop(bool value);
int   Rte_Read_SteerTarget();
void  Rte_Write_SteerTarget(int value);
bool  Rte_Read_PixhawkPwmValid();
void  Rte_Write_PixhawkPwmValid(bool value);
unsigned int Rte_Read_PixhawkPwmUs();
void  Rte_Write_PixhawkPwmUs(unsigned int value);

// ------------------------------------------------------------
// Client-server: 하드웨어 (MCAL 로 연결)
// ------------------------------------------------------------
unsigned long Rte_Call_Time_GetMs();
long Rte_Call_Encoder_GetCount();
int  Rte_Call_SteerPot_Read();
bool Rte_Call_Pixhawk_ReadPwm(unsigned int &pulseUs);
void Rte_Call_DriveMotor_SetDirection(bool forward);
void Rte_Call_DriveMotor_SetPwm(int duty);
void Rte_Call_SteerMotor_SetDirection(bool potUp);
void Rte_Call_SteerMotor_SetPwm(int duty);

// ------------------------------------------------------------
// Client-server: 공통 서비스 (BSW 로 연결)
// ------------------------------------------------------------
void Rte_Call_FaultLatch_SetSteerFault();
void Rte_Call_FaultLatch_SetSteerStall();
void Rte_Call_Watchdog_Kick(unsigned long now);
void Rte_Call_Console_WriteText(const char *text);
void Rte_Call_Console_WriteLong(long value);
void Rte_Call_Console_WriteFloat(float value, uint8_t digits);
void Rte_Call_Console_WriteNewline();

// ------------------------------------------------------------
// Client-server: SWC 사이의 요청
// ------------------------------------------------------------
void Rte_Call_SpeedControl_ResetRamp(float rampedSpeedKmh);  // 램프 목표 설정 + 적분 0
void Rte_Call_SpeedControl_CancelOpenLoop();
void Rte_Call_StopHold_Release();

// ------------------------------------------------------------
// 보정 상수
// ------------------------------------------------------------
int   Rte_Prm_SteerPotCenter();
int   Rte_Prm_SteerPotSoftMin();
int   Rte_Prm_SteerPotSoftMax();
float Rte_Prm_MaxSpeedKmh();

// ------------------------------------------------------------
// 이벤트: BSW -> SWC
// ------------------------------------------------------------
void Rte_Notify_SerialCommand(const Rte_SerialCmdType &cmd);
void Rte_Notify_CommandTimeout();
void Rte_Notify_ComSafeState();     // E2E 고장. 워치독 처리 + 비상정지 + 래치

#endif
