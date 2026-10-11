// ============================================================
// SWC StopHold: 정지 · 홀드 토크 판단
//
// 이 차에는 브레이크가 없다. PWM 0 은 관성주행(coast)이다. 그래서
// "멈춰야 하는" 세 상황에서 제한된 역토크를 걸지 판단한다.
//   1) MANUAL 반대 스로틀 능동 제동 (한 번만)
//   2) 미션 홀드 'H' (오르막 정차 밀림 방지, 주행 중 제동)
//   3) AUTO 반대 명령 능동 제동 (ON/OFF 반복)
// 셋 중 하나가 이번 주기 출력을 정하면 StopHoldOverride 를 세워
// Swc_SpeedControl_Run() 이 PI 를 건너뛰게 한다.
// ============================================================

#ifndef SWC_STOP_HOLD_H
#define SWC_STOP_HOLD_H

#include "Std_Types.h"

void Swc_StopHold_Run(float dt);       // 100 ms, MeasureSpeed 와 SpeedControl_Run 사이

void Swc_StopHold_OnHoldCmd();         // 'h' 명령
void Swc_StopHold_Release();           // 서버 연산: 미션 홀드 해제

#endif
