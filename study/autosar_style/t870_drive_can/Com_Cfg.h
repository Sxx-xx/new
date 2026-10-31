// ============================================================
// 자동 생성 파일. 직접 고치지 말 것.
// 원본: can/t870.dbc, 생성기: can/gen_com_cfg.py
//
// 구조체 멤버는 스케일을 적용하지 않은 raw 값이다.
// 물리값 = raw * FACTOR. 부동소수 오차를 피하려고 raw / RAW_PER_UNIT 으로 나눠 쓴다.
// ============================================================

#ifndef COM_CFG_H
#define COM_CFG_H

#include "Std_Types.h"

// 리틀엔디언(Intel) 비트 필드 읽기 / 쓰기
static inline uint32_t Com_GetBits(const uint8_t *data, uint8_t start, uint8_t length)
{
  uint32_t value = 0;
  for (uint8_t i = 0; i < length; ++i)
  {
    const uint8_t bit = start + i;
    if (data[bit / 8] & (uint8_t)(1U << (bit % 8)))
    {
      value |= ((uint32_t)1) << i;
    }
  }
  return value;
}

static inline int32_t Com_GetSignedBits(const uint8_t *data, uint8_t start, uint8_t length)
{
  const uint32_t value = Com_GetBits(data, start, length);
  const uint32_t signBit = ((uint32_t)1) << (length - 1);
  return (value & signBit) ? (int32_t)value - ((int32_t)signBit * 2) : (int32_t)value;
}

static inline void Com_SetBits(uint8_t *data, uint8_t start, uint8_t length, uint32_t value)
{
  for (uint8_t i = 0; i < length; ++i)
  {
    const uint8_t bit = start + i;
    const uint8_t mask = (uint8_t)(1U << (bit % 8));
    if ((value >> i) & 1U)
    {
      data[bit / 8] |= mask;
    }
    else
    {
      data[bit / 8] &= (uint8_t)~mask;
    }
  }
}

// ---- DriveCmd ----
#define COM_DRIVECMD_ID          0x100
#define COM_DRIVECMD_DLC         8
#define COM_DRIVECMD_CYCLE_MS    20
#define COM_DRIVECMD_DATA_ID     0x10
#define COM_DRIVECMD_TARGETSPEED_KMH_START 0
#define COM_DRIVECMD_TARGETSPEED_KMH_LENGTH 16
#define COM_DRIVECMD_TARGETSPEED_KMH_RAW_PER_UNIT 100.0f
#define COM_DRIVECMD_STEERTARGETPOT_START 16
#define COM_DRIVECMD_STEERTARGETPOT_LENGTH 16
#define COM_DRIVECMD_MODE_START 32
#define COM_DRIVECMD_MODE_LENGTH 2
#define COM_DRIVECMD_OPENLOOPSEL_START 34
#define COM_DRIVECMD_OPENLOOPSEL_LENGTH 2
#define COM_DRIVECMD_ALIVE_START 36
#define COM_DRIVECMD_ALIVE_LENGTH 4
#define COM_DRIVECMD_OPENLOOPDUTY_START 40
#define COM_DRIVECMD_OPENLOOPDUTY_LENGTH 10
#define COM_DRIVECMD_WATCHDOGDISABLE_START 50
#define COM_DRIVECMD_WATCHDOGDISABLE_LENGTH 1
#define COM_DRIVECMD_LATCHCLEARREQ_START 51
#define COM_DRIVECMD_LATCHCLEARREQ_LENGTH 1
#define COM_DRIVECMD_CRC8_START 56
#define COM_DRIVECMD_CRC8_LENGTH 8
#define COM_DRIVECMD_MODE_MANUAL 0
#define COM_DRIVECMD_MODE_AUTO 1
#define COM_DRIVECMD_MODE_HOLD 2
#define COM_DRIVECMD_MODE_ESTOP 3
#define COM_DRIVECMD_OPENLOOPSEL_NONE 0
#define COM_DRIVECMD_OPENLOOPSEL_DRIVE 1
#define COM_DRIVECMD_OPENLOOPSEL_STEER 2

struct Com_DriveCmdType
{
  int16_t TargetSpeed_kmh;
  uint16_t SteerTargetPot;
  uint8_t Mode;
  uint8_t OpenLoopSel;
  uint8_t Alive;
  int16_t OpenLoopDuty;
  bool WatchdogDisable;
  bool LatchClearReq;
  uint8_t CRC8;
};

static inline void Com_Pack_DriveCmd(const Com_DriveCmdType &s, uint8_t *data)
{
  Com_SetBits(data, 0, 16, (uint32_t)(int32_t)s.TargetSpeed_kmh);
  Com_SetBits(data, 16, 16, s.SteerTargetPot);
  Com_SetBits(data, 32, 2, s.Mode);
  Com_SetBits(data, 34, 2, s.OpenLoopSel);
  Com_SetBits(data, 36, 4, s.Alive);
  Com_SetBits(data, 40, 10, (uint32_t)(int32_t)s.OpenLoopDuty);
  Com_SetBits(data, 50, 1, s.WatchdogDisable);
  Com_SetBits(data, 51, 1, s.LatchClearReq);
  Com_SetBits(data, 56, 8, s.CRC8);
}

static inline void Com_Unpack_DriveCmd(const uint8_t *data, Com_DriveCmdType &s)
{
  s.TargetSpeed_kmh = (int16_t)Com_GetSignedBits(data, 0, 16);
  s.SteerTargetPot = (uint16_t)Com_GetBits(data, 16, 16);
  s.Mode = (uint8_t)Com_GetBits(data, 32, 2);
  s.OpenLoopSel = (uint8_t)Com_GetBits(data, 34, 2);
  s.Alive = (uint8_t)Com_GetBits(data, 36, 4);
  s.OpenLoopDuty = (int16_t)Com_GetSignedBits(data, 40, 10);
  s.WatchdogDisable = Com_GetBits(data, 50, 1) != 0;
  s.LatchClearReq = Com_GetBits(data, 51, 1) != 0;
  s.CRC8 = (uint8_t)Com_GetBits(data, 56, 8);
}

// ---- DriveStatus ----
#define COM_DRIVESTATUS_ID          0x200
#define COM_DRIVESTATUS_DLC         8
#define COM_DRIVESTATUS_CYCLE_MS    100
#define COM_DRIVESTATUS_DATA_ID     0x20
#define COM_DRIVESTATUS_MEASUREDSPEED_KMH_START 0
#define COM_DRIVESTATUS_MEASUREDSPEED_KMH_LENGTH 16
#define COM_DRIVESTATUS_MEASUREDSPEED_KMH_RAW_PER_UNIT 100.0f
#define COM_DRIVESTATUS_STEERPOT_START 16
#define COM_DRIVESTATUS_STEERPOT_LENGTH 16
#define COM_DRIVESTATUS_DRIVEPWM_START 32
#define COM_DRIVESTATUS_DRIVEPWM_LENGTH 8
#define COM_DRIVESTATUS_STEERPWM_START 40
#define COM_DRIVESTATUS_STEERPWM_LENGTH 8
#define COM_DRIVESTATUS_FAULTSTEER_START 48
#define COM_DRIVESTATUS_FAULTSTEER_LENGTH 1
#define COM_DRIVESTATUS_FAULTSTALL_START 49
#define COM_DRIVESTATUS_FAULTSTALL_LENGTH 1
#define COM_DRIVESTATUS_FAULTWATCHDOG_START 50
#define COM_DRIVESTATUS_FAULTWATCHDOG_LENGTH 1
#define COM_DRIVESTATUS_FAULTE2E_START 51
#define COM_DRIVESTATUS_FAULTE2E_LENGTH 1
#define COM_DRIVESTATUS_ALIVE_START 52
#define COM_DRIVESTATUS_ALIVE_LENGTH 4
#define COM_DRIVESTATUS_CRC8_START 56
#define COM_DRIVESTATUS_CRC8_LENGTH 8

struct Com_DriveStatusType
{
  int16_t MeasuredSpeed_kmh;
  uint16_t SteerPot;
  uint8_t DrivePwm;
  uint8_t SteerPwm;
  bool FaultSteer;
  bool FaultStall;
  bool FaultWatchdog;
  bool FaultE2E;
  uint8_t Alive;
  uint8_t CRC8;
};

static inline void Com_Pack_DriveStatus(const Com_DriveStatusType &s, uint8_t *data)
{
  Com_SetBits(data, 0, 16, (uint32_t)(int32_t)s.MeasuredSpeed_kmh);
  Com_SetBits(data, 16, 16, s.SteerPot);
  Com_SetBits(data, 32, 8, s.DrivePwm);
  Com_SetBits(data, 40, 8, s.SteerPwm);
  Com_SetBits(data, 48, 1, s.FaultSteer);
  Com_SetBits(data, 49, 1, s.FaultStall);
  Com_SetBits(data, 50, 1, s.FaultWatchdog);
  Com_SetBits(data, 51, 1, s.FaultE2E);
  Com_SetBits(data, 52, 4, s.Alive);
  Com_SetBits(data, 56, 8, s.CRC8);
}

static inline void Com_Unpack_DriveStatus(const uint8_t *data, Com_DriveStatusType &s)
{
  s.MeasuredSpeed_kmh = (int16_t)Com_GetSignedBits(data, 0, 16);
  s.SteerPot = (uint16_t)Com_GetBits(data, 16, 16);
  s.DrivePwm = (uint8_t)Com_GetBits(data, 32, 8);
  s.SteerPwm = (uint8_t)Com_GetBits(data, 40, 8);
  s.FaultSteer = Com_GetBits(data, 48, 1) != 0;
  s.FaultStall = Com_GetBits(data, 49, 1) != 0;
  s.FaultWatchdog = Com_GetBits(data, 50, 1) != 0;
  s.FaultE2E = Com_GetBits(data, 51, 1) != 0;
  s.Alive = (uint8_t)Com_GetBits(data, 52, 4);
  s.CRC8 = (uint8_t)Com_GetBits(data, 56, 8);
}

// ---- DriveDiag ----
#define COM_DRIVEDIAG_ID          0x201
#define COM_DRIVEDIAG_DLC         8
#define COM_DRIVEDIAG_CYCLE_MS    500
#define COM_DRIVEDIAG_DATA_ID     0x21
#define COM_DRIVEDIAG_CRCERRORCOUNT_START 0
#define COM_DRIVEDIAG_CRCERRORCOUNT_LENGTH 16
#define COM_DRIVEDIAG_ALIVEJUMPCOUNT_START 16
#define COM_DRIVEDIAG_ALIVEJUMPCOUNT_LENGTH 16
#define COM_DRIVEDIAG_ALIVEREPEATCOUNT_START 32
#define COM_DRIVEDIAG_ALIVEREPEATCOUNT_LENGTH 8
#define COM_DRIVEDIAG_TIMEOUTCOUNT_START 40
#define COM_DRIVEDIAG_TIMEOUTCOUNT_LENGTH 8
#define COM_DRIVEDIAG_E2ESTATE_START 48
#define COM_DRIVEDIAG_E2ESTATE_LENGTH 3
#define COM_DRIVEDIAG_ALIVE_START 52
#define COM_DRIVEDIAG_ALIVE_LENGTH 4
#define COM_DRIVEDIAG_CRC8_START 56
#define COM_DRIVEDIAG_CRC8_LENGTH 8
#define COM_DRIVEDIAG_E2ESTATE_OK 0
#define COM_DRIVEDIAG_E2ESTATE_WRONG_CRC 1
#define COM_DRIVEDIAG_E2ESTATE_REPEATED 2
#define COM_DRIVEDIAG_E2ESTATE_WRONG_SEQ 3
#define COM_DRIVEDIAG_E2ESTATE_NO_NEW_DATA 4

struct Com_DriveDiagType
{
  uint16_t CrcErrorCount;
  uint16_t AliveJumpCount;
  uint8_t AliveRepeatCount;
  uint8_t TimeoutCount;
  uint8_t E2EState;
  uint8_t Alive;
  uint8_t CRC8;
};

static inline void Com_Pack_DriveDiag(const Com_DriveDiagType &s, uint8_t *data)
{
  Com_SetBits(data, 0, 16, s.CrcErrorCount);
  Com_SetBits(data, 16, 16, s.AliveJumpCount);
  Com_SetBits(data, 32, 8, s.AliveRepeatCount);
  Com_SetBits(data, 40, 8, s.TimeoutCount);
  Com_SetBits(data, 48, 3, s.E2EState);
  Com_SetBits(data, 52, 4, s.Alive);
  Com_SetBits(data, 56, 8, s.CRC8);
}

static inline void Com_Unpack_DriveDiag(const uint8_t *data, Com_DriveDiagType &s)
{
  s.CrcErrorCount = (uint16_t)Com_GetBits(data, 0, 16);
  s.AliveJumpCount = (uint16_t)Com_GetBits(data, 16, 16);
  s.AliveRepeatCount = (uint8_t)Com_GetBits(data, 32, 8);
  s.TimeoutCount = (uint8_t)Com_GetBits(data, 40, 8);
  s.E2EState = (uint8_t)Com_GetBits(data, 48, 3);
  s.Alive = (uint8_t)Com_GetBits(data, 52, 4);
  s.CRC8 = (uint8_t)Com_GetBits(data, 56, 8);
}

#endif
