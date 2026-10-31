#include "Cdd_RemoteRx.h"
#include "Mcal_I2c.h"
#include "Lib_Math.h"


static const uint8_t REMOTE_I2C_ADDRESS     = 0x12;
static const uint8_t REMOTE_REPORT_MAGIC    = 0xB7;
static const uint8_t REMOTE_REPORT_VERSION  = 1;
static const uint8_t REMOTE_FLAG_AUTO       = 0x01;
static const uint8_t REMOTE_FLAG_MODE_VALID = 0x02;
static const unsigned long REMOTE_POLL_MS   = 20;
// 일반 RC 펄스/I2C 전송의 일시적인 지연은 허용한다. 마지막 정상 보고서가
// 이 시간보다 오래 갱신되지 않을 때만 실제 수신기 단절로 판정한다.
static const unsigned long REMOTE_TIMEOUT_MS = 1000;

struct __attribute__((packed)) RemoteReport
{
  uint8_t  magic;
  uint8_t  version;
  uint8_t  sequence;
  int8_t   throttleCommand;
  int8_t   steeringCommand;
  uint8_t  flags;
  uint8_t  connected;
  uint16_t throttleRaw;
  uint16_t steeringRaw;
  uint8_t  checksum;
};

static bool     remoteControlValid    = false;
static bool     remoteAutoMode        = false;
static int      remoteThrottleCommand = 0;
static int      remoteSteeringCommand = 0;
static uint16_t remoteThrottleRaw     = 0;
static uint16_t remoteSteeringRaw     = 0;
static uint8_t  remoteSequence        = 0;
static bool     remoteSequenceSeen    = false;
static unsigned long lastRemotePollMs   = 0;
static unsigned long lastRemoteUpdateMs = 0;


static uint8_t remoteChecksum(const RemoteReport &value)
{
  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&value);
  uint8_t checksum = 0;
  for (size_t i = 0; i < sizeof(RemoteReport) - 1; ++i)
  {
    checksum ^= bytes[i];
  }
  return checksum;
}


void Cdd_RemoteRx_MainFunction(unsigned long now)
{
  if (now - lastRemotePollMs < REMOTE_POLL_MS)
  {
    if (now - lastRemoteUpdateMs > REMOTE_TIMEOUT_MS)
    {
      remoteControlValid = false;
    }
    return;
  }
  lastRemotePollMs = now;

  RemoteReport next;
  uint8_t *destination = reinterpret_cast<uint8_t *>(&next);
  uint8_t received = Mcal_I2c_RequestFrom(
    REMOTE_I2C_ADDRESS, (uint8_t)sizeof(RemoteReport));

  if (received != sizeof(RemoteReport))
  {
    while (Mcal_I2c_Available()) Mcal_I2c_Read();
    if (now - lastRemoteUpdateMs > REMOTE_TIMEOUT_MS)
    {
      remoteControlValid = false;
    }
    return;
  }

  for (size_t i = 0; i < sizeof(RemoteReport); ++i)
  {
    if (!Mcal_I2c_Available())
    {
      if (now - lastRemoteUpdateMs > REMOTE_TIMEOUT_MS)
      {
        remoteControlValid = false;
      }
      return;
    }
    destination[i] = (uint8_t)Mcal_I2c_Read();
  }

  const bool headerValid =
    next.magic == REMOTE_REPORT_MAGIC
    && next.version == REMOTE_REPORT_VERSION;
  const bool checksumValid = next.checksum == remoteChecksum(next);
  const bool modeValid = (next.flags & REMOTE_FLAG_MODE_VALID) != 0;

  if (!headerValid || !checksumValid || !next.connected || !modeValid)
  {
    // 단일 깨진 I2C 프레임은 무시한다. 즉시 RC:LOST로 바꾸면
    // 주행 중 PWM이 끊기고 MANUAL 중립 래치가 걸릴 수 있다.
    if (now - lastRemoteUpdateMs > REMOTE_TIMEOUT_MS)
    {
      remoteControlValid = false;
    }
    return;
  }

  // 같은 보고서만 반복되면 보조 보드가 멈춘 것으로 보고 timeout 처리한다.
  if (!remoteSequenceSeen || next.sequence != remoteSequence)
  {
    remoteSequenceSeen = true;
    remoteSequence = next.sequence;
    lastRemoteUpdateMs = now;
  }

  remoteAutoMode = (next.flags & REMOTE_FLAG_AUTO) != 0;
  remoteThrottleCommand = (int)Lib_ConstrainL((int)next.throttleCommand, -100, 100);
  remoteSteeringCommand = (int)Lib_ConstrainL((int)next.steeringCommand, -100, 100);
  remoteThrottleRaw = next.throttleRaw;
  remoteSteeringRaw = next.steeringRaw;
  remoteControlValid = (now - lastRemoteUpdateMs <= REMOTE_TIMEOUT_MS);
}


bool     Cdd_RemoteRx_IsValid()            { return remoteControlValid; }
bool     Cdd_RemoteRx_IsAutoMode()         { return remoteAutoMode; }
int      Cdd_RemoteRx_GetThrottleCommand() { return remoteThrottleCommand; }
int      Cdd_RemoteRx_GetSteeringCommand() { return remoteSteeringCommand; }
uint16_t Cdd_RemoteRx_GetThrottleRaw()     { return remoteThrottleRaw; }
uint16_t Cdd_RemoteRx_GetSteeringRaw()     { return remoteSteeringRaw; }
