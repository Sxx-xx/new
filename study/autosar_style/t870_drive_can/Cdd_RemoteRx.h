// ============================================================
// CDD RemoteRx: 보조 Arduino RC 수신기 보고서 (I2C 0x12)
//
// 보조 보드가 일반 PWM 리시버 D4/D5/D6을 읽어 주소 0x12로 전달한다.
// 이 드라이버는 보고서를 읽고 헤더 · 체크섬 · 순번을 검사해
// "지금 믿을 수 있는 조종기 상태"만 위로 내보낸다.
// ============================================================

#ifndef CDD_REMOTE_RX_H
#define CDD_REMOTE_RX_H

#include "Std_Types.h"

void Cdd_RemoteRx_MainFunction(unsigned long now);

bool     Cdd_RemoteRx_IsValid();
bool     Cdd_RemoteRx_IsAutoMode();
int      Cdd_RemoteRx_GetThrottleCommand();
int      Cdd_RemoteRx_GetSteeringCommand();
uint16_t Cdd_RemoteRx_GetThrottleRaw();
uint16_t Cdd_RemoteRx_GetSteeringRaw();

#endif
