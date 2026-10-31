// ============================================================
// SWC SteeringControl: 포텐셔미터 기반 조향 위치 제어
//
// 목표: AUTO 는 Pixhawk 조향 PWM, MANUAL 은 RC 조향 명령.
// 보호: 센서 폴트 래치, 소프트 리밋, 목표 신호 단절 시 정지, (비활성) 스톨.
// ============================================================

#ifndef SWC_STEERING_CONTROL_H
#define SWC_STEERING_CONTROL_H

#include "Std_Types.h"

void Swc_SteeringControl_Init();
void Swc_SteeringControl_Run(unsigned long now);   // 매 loop

void Swc_SteeringControl_OnOpenLoopCmd(int duty);  // 's' 명령
void Swc_SteeringControl_OnCommandTimeout();

int Swc_SteeringControl_GetPotCenter();
int Swc_SteeringControl_GetPotSoftMin();
int Swc_SteeringControl_GetPotSoftMax();

#endif
