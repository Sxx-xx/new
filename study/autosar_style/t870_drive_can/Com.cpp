#include <string.h>

#include "Com.h"
#include "CanIf.h"


static E2E_RxStateType rxState;
static E2E_StatusType  rxStatus = E2E_STATUS_NO_NEW_DATA;

static uint8_t rxLatest[COM_DRIVECMD_DLC];
static bool    rxLatestIsNew = false;
static bool    rxEverOk      = false;
static bool    rxTimedOut    = false;
static unsigned long rxLastOkMs = 0;

static uint8_t failCount = 0;
static bool    safeStateRequest = false;

static uint16_t crcErrorCount    = 0;
static uint16_t aliveJumpCount   = 0;
static uint8_t  aliveRepeatCount = 0;
static uint8_t  timeoutCount     = 0;

static uint8_t statusAlive = 0;
static uint8_t diagAlive   = 0;
static unsigned long statusLastMs = 0;
static unsigned long diagLastMs   = 0;


static void countSaturating16(uint16_t &counter) { if (counter < 0xFFFF) counter++; }
static void countSaturating8(uint8_t &counter)   { if (counter < 0xFF) counter++; }


void Com_Init()
{
  E2E_InitRx(rxState);
}


void Com_RxIndication(uint8_t dlc, const uint8_t *data, unsigned long now)
{
  rxStatus = E2E_Check(rxState, data, dlc, COM_DRIVECMD_ALIVE_START, COM_DRIVECMD_DATA_ID);

  switch (rxStatus)
  {
    case E2E_STATUS_OK:
      memcpy(rxLatest, data, COM_DRIVECMD_DLC);
      rxLatestIsNew = true;
      rxEverOk      = true;
      rxTimedOut    = false;
      rxLastOkMs    = now;
      failCount     = 0;
      return;

    case E2E_STATUS_WRONG_CRC: countSaturating16(crcErrorCount);   break;
    case E2E_STATUS_REPEATED:  countSaturating8(aliveRepeatCount); break;
    case E2E_STATUS_WRONG_SEQ: countSaturating16(aliveJumpCount);  break;
    case E2E_STATUS_NO_NEW_DATA: break;
  }

  if (failCount < E2E_FAIL_THRESHOLD)
  {
    failCount++;
  }
  if (failCount >= E2E_FAIL_THRESHOLD)
  {
    safeStateRequest = true;
  }
}


void Com_MainFunctionRx(unsigned long now)
{
  if (rxEverOk && !rxTimedOut && (now - rxLastOkMs > E2E_RX_TIMEOUT_MS))
  {
    rxTimedOut = true;
    rxStatus = E2E_STATUS_NO_NEW_DATA;
    countSaturating8(timeoutCount);
    safeStateRequest = true;
  }
}


bool Com_ReceiveDriveCmd(Com_DriveCmdType &cmd)
{
  if (!rxLatestIsNew)
  {
    return false;
  }
  rxLatestIsNew = false;
  Com_Unpack_DriveCmd(rxLatest, cmd);
  return true;
}


bool Com_ConsumeSafeStateRequest()
{
  const bool requested = safeStateRequest;
  safeStateRequest = false;
  return requested;
}


bool Com_IsDriveStatusDue(unsigned long now)
{
  if (now - statusLastMs < COM_DRIVESTATUS_CYCLE_MS)
  {
    return false;
  }
  statusLastMs = now;
  return true;
}


void Com_SendDriveStatus(const Com_DriveStatusType &status)
{
  uint8_t data[COM_DRIVESTATUS_DLC] = {0};
  Com_Pack_DriveStatus(status, data);
  E2E_Protect(data, COM_DRIVESTATUS_ALIVE_START, statusAlive++, COM_DRIVESTATUS_DATA_ID);
  CanIf_Transmit(COM_DRIVESTATUS_ID, data, COM_DRIVESTATUS_DLC);
}


void Com_MainFunctionTx(unsigned long now)
{
  if (now - diagLastMs < COM_DRIVEDIAG_CYCLE_MS)
  {
    return;
  }
  diagLastMs = now;

  Com_DriveDiagType diag;
  diag.CrcErrorCount    = crcErrorCount;
  diag.AliveJumpCount   = aliveJumpCount;
  diag.AliveRepeatCount = aliveRepeatCount;
  diag.TimeoutCount     = timeoutCount;
  diag.E2EState         = (uint8_t)rxStatus;
  diag.Alive            = 0;
  diag.CRC8             = 0;

  uint8_t data[COM_DRIVEDIAG_DLC] = {0};
  Com_Pack_DriveDiag(diag, data);
  E2E_Protect(data, COM_DRIVEDIAG_ALIVE_START, diagAlive++, COM_DRIVEDIAG_DATA_ID);
  CanIf_Transmit(COM_DRIVEDIAG_ID, data, COM_DRIVEDIAG_DLC);
}
