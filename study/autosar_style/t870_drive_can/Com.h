// ============================================================
// Com: 신호 pack/unpack, 주기 송신, 수신 타임아웃
//
// DriveCmd 를 E2E 로 검사해 통과한 프레임의 신호만 위로 넘긴다.
// E2E 실패가 연속 E2E_FAIL_THRESHOLD 프레임이거나 정상 프레임이
// E2E_RX_TIMEOUT_MS 동안 없으면 안전 상태 요청을 세운다.
// 첫 정상 프레임 전에는 타임아웃을 세지 않는다. 명령이 온 적 없으면 차는
// 움직이지 않고, 켤 때마다 고장 해제를 요구하게 되기 때문이다.
// ============================================================

#ifndef COM_H
#define COM_H

#include "Std_Types.h"
#include "Com_Cfg.h"
#include "E2E.h"

static const uint8_t       E2E_FAIL_THRESHOLD = 3;
static const unsigned long E2E_RX_TIMEOUT_MS  = 100;   // DriveCmd 5 주기

void Com_Init();
void Com_RxIndication(uint8_t dlc, const uint8_t *data, unsigned long now);   // CanIf 가 부른다
void Com_MainFunctionRx(unsigned long now);
void Com_MainFunctionTx(unsigned long now);

bool Com_ReceiveDriveCmd(Com_DriveCmdType &cmd);   // 지난 호출 뒤 새 정상 프레임이 있으면 true
bool Com_ConsumeSafeStateRequest();                // 요청을 읽고 내린다

bool Com_IsDriveStatusDue(unsigned long now);      // 100 ms 마다 true
void Com_SendDriveStatus(const Com_DriveStatusType &status);

#endif
