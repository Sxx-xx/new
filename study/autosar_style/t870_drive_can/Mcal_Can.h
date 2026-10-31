// ============================================================
// MCAL Can: CAN 컨트롤러 (11 비트 ID 데이터 프레임만 위로 올린다)
//
// 구현은 둘이다. 위 계층은 어느 쪽인지 모른다.
//   Mcal_Can_Mcp2515.cpp      : 실기판. MCP2515 + autowp/arduino-mcp2515
//   ../sil/Mcal_Can_Sim.cpp   : PC. SocketCAN 또는 인메모리 버스
// ============================================================

#ifndef MCAL_CAN_H
#define MCAL_CAN_H

#include "Std_Types.h"

struct Mcal_Can_FrameType
{
  uint16_t id;
  uint8_t  dlc;
  uint8_t  data[8];
};

// Mcal_Port_Init() 다음에 불러야 한다. 이유는 Mcal_Can_Mcp2515.cpp 주석.
bool Mcal_Can_Init();
bool Mcal_Can_Read(Mcal_Can_FrameType &frame);          // 비블로킹. 없으면 false
bool Mcal_Can_Write(const Mcal_Can_FrameType &frame);   // 비블로킹. 송신 버퍼가 차 있으면 false

#endif
