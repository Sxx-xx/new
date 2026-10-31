// ============================================================
// CanIf: Mcal_Can 위의 수신 ID 필터와 송신 큐
//
// 수신: 등록된 ID 만 Com 으로 올린다.
// 송신: MCP2515 송신 버퍼가 차 있어도 Com 이 기다리지 않게 큐에 넣는다.
//       큐가 차면 가장 오래된 프레임을 버린다. 상태 프레임은 새것이 더 쓸모 있다.
// ============================================================

#ifndef CAN_IF_H
#define CAN_IF_H

#include "Std_Types.h"

bool CanIf_Init();
void CanIf_MainFunctionRx(unsigned long now);
void CanIf_MainFunctionTx();
void CanIf_Transmit(uint16_t id, const uint8_t *data, uint8_t dlc);

#endif
