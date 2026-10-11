// ============================================================
// 구동 SWC 두 개(SpeedControl, StopHold)가 함께 쓰는 보정 상수
//
// 실제 AUTOSAR 라면 파라미터 SWC 를 두고 Rte_Prm 으로 읽는다.
// 여기서는 같은 값을 두 곳에 복사하지 않도록 헤더 하나로 둔다.
// ============================================================

#ifndef SWC_DRIVE_PARAMS_H
#define SWC_DRIVE_PARAMS_H

static const float MAX_SPEED_KMH = 10.0;

// 주행 중 방향 반전 금지 임계. 문서 9.2(4)
static const float REVERSE_GUARD_KMH  = 0.3;
static const float STOP_THRESHOLD_KMH = 0.05;

// MANUAL 스로틀 중립 판정 (재무장 · 제동 해제)
static const int MANUAL_REARM_DEADBAND = 5;

#endif
