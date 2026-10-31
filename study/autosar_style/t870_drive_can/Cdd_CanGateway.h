// ============================================================
// CDD CanGateway: Com 신호 <-> RTE 연결 (Cdd_SerialCmd 를 대신한다)
//
// 시리얼 명령은 한 번 오는 사건이고 DriveCmd 는 20 ms 마다 오는 현재 상태다.
// 이 드라이버가 상태를 사건으로 바꿔 시리얼과 같은 Rte_Notify_SerialCommand()
// 로 넘긴다. SWC 는 명령이 어디서 왔는지 모른다.
//
//   Mode = AUTO              목표 속도가 바뀔 때 RTE_CMD_TARGET_SPEED
//   Mode = MANUAL            목표 0 으로 취급. RC 스위치를 덮어쓰지 않는다
//   Mode = HOLD              매 프레임 RTE_CMD_MISSION_HOLD (500 ms 안에 갱신해야 유지)
//   Mode = ESTOP             바뀌는 순간 RTE_CMD_EMERGENCY_STOP
//   OpenLoopSel = DRIVE      AUTO 일 때 duty 가 바뀌면 RTE_CMD_DRIVE_OPEN_LOOP
//   OpenLoopSel = STEER      duty 가 바뀌면 RTE_CMD_STEER_OPEN_LOOP. 다른 값이 되면 duty 0
//   WatchdogDisable          현재 상태와 다를 때만 RTE_CMD_WATCHDOG_TOGGLE
//   LatchClearReq            상승 에지에서 RTE_CMD_LATCH_CLEAR
//
// E2E 고장 래치가 서 있는 동안은 위 명령을 모두 무시한다. 해제는 정상 프레임의
// LatchClearReq 상승 에지이고, 그 프레임의 목표 속도가 0 이고 개루프가 꺼져 있어야 한다.
// 통신이 돌아왔다고 옛 목표로 저절로 출발하지 않게 하려는 것이다.
// ============================================================

#ifndef CDD_CAN_GATEWAY_H
#define CDD_CAN_GATEWAY_H

#include "Std_Types.h"

void Cdd_CanGateway_Init();      // Mcal_Port_Init() 다음에 부른다
void Cdd_CanGateway_MainFunctionRx(unsigned long now);
void Cdd_CanGateway_MainFunctionTx(unsigned long now);

#endif
