#include "Bsw_Watchdog.h"
#include "Cdd_RemoteRx.h"
#include "Mcal_Uart.h"
#include "Rte.h"


// [2026-08-27 지면 부하 단거리 시험]
// 단발 시리얼 명령 후 관찰 시간을 확보하되, 통신이 끊기면 3초 안에 정지한다.
static const unsigned long CMD_TIMEOUT_MS = 3000;

static unsigned long lastCommandTime = 0;
static bool watchdogEnabled = true;
static bool watchdogExpired = false;


void Bsw_Watchdog_Init(unsigned long now)
{
  lastCommandTime = now;
}


void Bsw_Watchdog_MainFunction(unsigned long now)
{
  if (watchdogEnabled && Cdd_RemoteRx_IsAutoMode()
      && (now - lastCommandTime > CMD_TIMEOUT_MS))
  {
    watchdogExpired = true;
    Rte_Notify_CommandTimeout();
  }
}


void Bsw_Watchdog_Kick(unsigned long now)
{
  lastCommandTime = now;
  watchdogExpired = false;
}


void Bsw_Watchdog_Toggle()
{
  watchdogEnabled = !watchdogEnabled;

  Mcal_Uart_WriteText("WATCHDOG ");
  Mcal_Uart_WriteText(watchdogEnabled ? "ON" : "OFF");
  Mcal_Uart_WriteNewline();
}


bool Bsw_Watchdog_IsEnabled()
{
  return watchdogEnabled;
}


bool Bsw_Watchdog_IsExpired()
{
  return watchdogExpired;
}
