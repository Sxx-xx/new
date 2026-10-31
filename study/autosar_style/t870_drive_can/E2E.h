// ============================================================
// E2E: CRC8 + Alive 카운터 보호
//
// 8 바이트 프레임의 마지막 바이트가 CRC8 이다.
// CRC8 = SAE J1850 (다항식 0x1D, 초기값 0xFF, 최종 XOR 0xFF), 입력은
// 데이터 0~6 바이트 다음에 DataID 한 바이트.
// Alive 는 4 비트이고 프레임마다 1 씩 늘어난다.
//
// 상태 값은 DBC 의 DriveDiag.E2EState 와 같다.
// ============================================================

#ifndef E2E_H
#define E2E_H

#include "Std_Types.h"

enum E2E_StatusType
{
  E2E_STATUS_OK          = 0,
  E2E_STATUS_WRONG_CRC   = 1,   // CRC 불일치 또는 DLC 불일치
  E2E_STATUS_REPEATED    = 2,   // Alive 가 그대로다
  E2E_STATUS_WRONG_SEQ   = 3,   // Alive 가 건너뛰었다
  E2E_STATUS_NO_NEW_DATA = 4    // 프레임을 받은 적 없거나 수신 타임아웃
};

struct E2E_RxStateType
{
  bool    synced;       // Alive 기준값을 잡았나
  uint8_t lastAlive;
  uint8_t crcMissed;    // 마지막 정상 프레임 뒤 CRC 로 버린 프레임 수
};

void    E2E_InitRx(E2E_RxStateType &state);
uint8_t E2E_Crc8(const uint8_t *data, uint8_t length, uint8_t dataId);

// 송신: Alive 를 넣고 CRC8 을 계산해 8 바이트 프레임을 완성한다.
void E2E_Protect(uint8_t *data, uint8_t aliveStartBit, uint8_t alive, uint8_t dataId);

// 수신: CRC 를 먼저 보고, 믿을 수 있는 프레임의 Alive 만 본다.
E2E_StatusType E2E_Check(E2E_RxStateType &state, const uint8_t *data, uint8_t dlc,
                         uint8_t aliveStartBit, uint8_t dataId);

#endif
