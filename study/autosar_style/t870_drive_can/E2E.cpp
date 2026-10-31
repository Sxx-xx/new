#include "E2E.h"
#include "Com_Cfg.h"


static const uint8_t E2E_ALIVE_LENGTH = 4;
static const uint8_t E2E_ALIVE_MASK   = 0x0F;
static const uint8_t E2E_FRAME_LENGTH = 8;
static const uint8_t E2E_CRC_BYTE     = 7;


static uint8_t crc8Update(uint8_t crc, uint8_t value)
{
  crc ^= value;
  for (uint8_t bit = 0; bit < 8; ++bit)
  {
    crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x1D) : (uint8_t)(crc << 1);
  }
  return crc;
}


uint8_t E2E_Crc8(const uint8_t *data, uint8_t length, uint8_t dataId)
{
  uint8_t crc = 0xFF;
  for (uint8_t i = 0; i < length; ++i)
  {
    crc = crc8Update(crc, data[i]);
  }
  crc = crc8Update(crc, dataId);
  return crc ^ 0xFF;
}


void E2E_InitRx(E2E_RxStateType &state)
{
  state.synced = false;
  state.lastAlive = 0;
  state.crcMissed = 0;
}


void E2E_Protect(uint8_t *data, uint8_t aliveStartBit, uint8_t alive, uint8_t dataId)
{
  Com_SetBits(data, aliveStartBit, E2E_ALIVE_LENGTH, alive & E2E_ALIVE_MASK);
  data[E2E_CRC_BYTE] = E2E_Crc8(data, E2E_CRC_BYTE, dataId);
}


E2E_StatusType E2E_Check(E2E_RxStateType &state, const uint8_t *data, uint8_t dlc,
                         uint8_t aliveStartBit, uint8_t dataId)
{
  if (dlc != E2E_FRAME_LENGTH
      || data[E2E_CRC_BYTE] != E2E_Crc8(data, E2E_CRC_BYTE, dataId))
  {
    if (state.crcMissed < E2E_ALIVE_MASK - 1)
    {
      state.crcMissed++;
    }
    return E2E_STATUS_WRONG_CRC;
  }

  const uint8_t alive = (uint8_t)Com_GetBits(data, aliveStartBit, E2E_ALIVE_LENGTH);

  if (!state.synced)
  {
    state.synced = true;
    state.lastAlive = alive;
    state.crcMissed = 0;
    return E2E_STATUS_OK;
  }

  const uint8_t delta = (alive - state.lastAlive) & E2E_ALIVE_MASK;

  if (delta == 0)
  {
    return E2E_STATUS_REPEATED;
  }

  // CRC 로 버린 프레임만큼의 건너뜀은 이미 센 고장이다. 한 번 깨진 프레임이
  // 다음 정상 프레임까지 WRONG_SEQ 로 만들어 고장을 두 번 세지 않게 한다.
  const uint8_t allowedDelta = 1 + state.crcMissed;
  state.lastAlive = alive;
  state.crcMissed = 0;

  // 건너뛴 경우에도 기준값은 새로 잡는다. 송신기가 재시작하면 한 번만 틀리고 끝난다.
  return delta <= allowedDelta ? E2E_STATUS_OK : E2E_STATUS_WRONG_SEQ;
}
