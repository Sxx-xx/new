// ============================================================
// BSW Watchdog: 명령 워치독
//
// 노트북이 죽거나 USB 가 빠져도 마지막 명령으로 계속 달리는 것을 막는다.
// AUTO 에서 CMD_TIMEOUT_MS 동안 명령 줄이 없으면 Rte_Notify_CommandTimeout()
// 으로 SWC 에 알린다. MANUAL 에서는 살아 있는 수신기 보고서가 워치독을 갱신한다.
// ============================================================

#ifndef BSW_WATCHDOG_H
#define BSW_WATCHDOG_H

#include "Std_Types.h"

void Bsw_Watchdog_Init(unsigned long now);
void Bsw_Watchdog_MainFunction(unsigned long now);
void Bsw_Watchdog_Kick(unsigned long now);
void Bsw_Watchdog_Toggle();
bool Bsw_Watchdog_IsEnabled();

#endif
