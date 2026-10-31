// ============================================================
// BSW FaultLatch: 조향 센서 폴트 / 스톨 / E2E 고장 래치
//
// 한 번 걸리면 LatchClearReq 로 풀 때까지 유지된다. 무엇을 폴트로 볼지는
// Swc_SteeringControl 과 Com 이 판정하고, 이 모듈은 상태를 보관만 한다.
// ============================================================

#ifndef BSW_FAULT_LATCH_H
#define BSW_FAULT_LATCH_H

#include "Std_Types.h"

void Bsw_FaultLatch_SetSteerFault();
void Bsw_FaultLatch_SetSteerStall();
bool Bsw_FaultLatch_IsSteerFault();
bool Bsw_FaultLatch_IsSteerStall();
void Bsw_FaultLatch_SetE2eFault();
bool Bsw_FaultLatch_IsE2eFault();
void Bsw_FaultLatch_ClearAll();

#endif
