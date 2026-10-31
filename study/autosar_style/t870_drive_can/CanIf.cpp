#include <string.h>

#include "CanIf.h"
#include "Com.h"
#include "Com_Cfg.h"
#include "Mcal_Can.h"


// MCP2515 수신 버퍼가 두 개이므로 한 loop 에 그 이상 읽을 일이 없다.
static const uint8_t RX_FRAMES_PER_LOOP = 2;

static const uint8_t TX_QUEUE_LENGTH = 4;

static Mcal_Can_FrameType txQueue[TX_QUEUE_LENGTH];
static uint8_t txHead  = 0;
static uint8_t txCount = 0;


bool CanIf_Init()
{
  return Mcal_Can_Init();
}


void CanIf_MainFunctionRx(unsigned long now)
{
  Mcal_Can_FrameType frame;

  for (uint8_t i = 0; i < RX_FRAMES_PER_LOOP && Mcal_Can_Read(frame); ++i)
  {
    if (frame.id == COM_DRIVECMD_ID)
    {
      Com_RxIndication(frame.dlc, frame.data, now);
    }
  }
}


void CanIf_Transmit(uint16_t id, const uint8_t *data, uint8_t dlc)
{
  if (txCount == TX_QUEUE_LENGTH)
  {
    txHead = (txHead + 1) % TX_QUEUE_LENGTH;
    txCount--;
  }

  Mcal_Can_FrameType &slot = txQueue[(txHead + txCount) % TX_QUEUE_LENGTH];
  slot.id  = id;
  slot.dlc = dlc;
  memcpy(slot.data, data, dlc);
  txCount++;
}


void CanIf_MainFunctionTx()
{
  while (txCount > 0 && Mcal_Can_Write(txQueue[txHead]))
  {
    txHead = (txHead + 1) % TX_QUEUE_LENGTH;
    txCount--;
  }
}
