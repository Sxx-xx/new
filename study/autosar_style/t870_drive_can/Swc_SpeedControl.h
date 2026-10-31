// ============================================================
// SWC SpeedControl: 명령 선택, 속도 측정, 램프, 방향 반전 보호, PI + FF
// ============================================================

#ifndef SWC_SPEED_CONTROL_H
#define SWC_SPEED_CONTROL_H

#include "Std_Types.h"

void Swc_SpeedControl_Init();

// 주기 runnable
void Swc_SpeedControl_SelectCommand(unsigned long now);   // 매 loop
void Swc_SpeedControl_MeasureSpeed(float dt);             // 100 ms, StopHold 앞
void Swc_SpeedControl_Run(float dt);                      // 100 ms, StopHold 뒤

// 이벤트 runnable (Rte_Notify_xxx 가 부른다)
void Swc_SpeedControl_OnEmergencyStopCmd();
void Swc_SpeedControl_OnOpenLoopCmd(int duty);
void Swc_SpeedControl_OnTargetSpeedCmd(float targetKmh);
void Swc_SpeedControl_OnCommandTimeout();

// 서버 연산 (Rte_Call_SpeedControl_xxx 가 부른다)
void Swc_SpeedControl_ResetRamp(float rampedSpeedKmh);
void Swc_SpeedControl_CancelOpenLoop();

float Swc_SpeedControl_GetMaxSpeedKmh();

#endif
