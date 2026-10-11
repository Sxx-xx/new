// ============================================================
// MCAL Port/Dio: 핀 방향 설정과 DIR 출력
// ============================================================

#ifndef MCAL_PORT_H
#define MCAL_PORT_H

#include "Std_Types.h"

void Mcal_Port_Init();

// 극성(LOW/HIGH)은 Mcal_Cfg.h 가 정한다. 상위 계층은 방향만 말한다.
void Mcal_Port_SetSteerDirection(bool potUp);
void Mcal_Port_SetDriveDirection(bool forward);

#endif
