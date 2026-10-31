#include "Cdd_CanGateway.h"
#include "CanIf.h"
#include "Com.h"
#include "Rte.h"


static const uint8_t MODE_UNKNOWN = 0xFF;   // 다음 프레임의 명령을 반드시 내보낸다

static uint8_t lastMode      = MODE_UNKNOWN;
static uint8_t lastSel       = COM_DRIVECMD_OPENLOOPSEL_NONE;
static int16_t lastTargetRaw = 0;
static int16_t lastDriveDuty = 0;
static int16_t lastSteerDuty = 0;
static bool    lastClearReq  = false;


static void notifyInt(Rte_SerialCmdKind kind, int value)
{
  Rte_SerialCmdType cmd;
  cmd.kind = kind;
  cmd.intValue = value;
  cmd.floatValue = 0.0;
  Rte_Notify_SerialCommand(cmd);
}


static void notifyTarget(int16_t targetRaw)
{
  Rte_SerialCmdType cmd;
  cmd.kind = RTE_CMD_TARGET_SPEED;
  cmd.intValue = 0;
  // raw * 0.01f 로 곱하면 1.5 가 1.5 에 가까운 다른 float 이 될 수 있다.
  cmd.floatValue = (float)targetRaw / COM_DRIVECMD_TARGETSPEED_KMH_RAW_PER_UNIT;
  Rte_Notify_SerialCommand(cmd);
}


static void handleLatchClear(const Com_DriveCmdType &cmd)
{
  const bool risingEdge = cmd.LatchClearReq && !lastClearReq;
  lastClearReq = cmd.LatchClearReq;

  if (!risingEdge)
  {
    return;
  }

  if (Rte_Read_FaultLatch_E2e())
  {
    if (cmd.TargetSpeed_kmh != 0 || cmd.OpenLoopSel != COM_DRIVECMD_OPENLOOPSEL_NONE)
    {
      return;
    }
    lastMode = MODE_UNKNOWN;
    lastSteerDuty = 0;      // 안전 상태 진입이 조향 개루프를 껐다
  }

  notifyInt(RTE_CMD_LATCH_CLEAR, 0);
}


static void applyDriveCommand(const Com_DriveCmdType &cmd)
{
  const bool modeChanged = cmd.Mode != lastMode;

  if (cmd.Mode == COM_DRIVECMD_MODE_ESTOP)
  {
    if (modeChanged)
    {
      notifyInt(RTE_CMD_EMERGENCY_STOP, 0);
    }
  }
  else if (cmd.Mode == COM_DRIVECMD_MODE_HOLD)
  {
    notifyInt(RTE_CMD_MISSION_HOLD, 0);
  }
  else
  {
    const bool autoMode = cmd.Mode == COM_DRIVECMD_MODE_AUTO;
    const bool driveOpen = autoMode && cmd.OpenLoopSel == COM_DRIVECMD_OPENLOOPSEL_DRIVE;
    const int16_t targetRaw = autoMode ? cmd.TargetSpeed_kmh : 0;

    if (driveOpen)
    {
      if (modeChanged || lastSel != COM_DRIVECMD_OPENLOOPSEL_DRIVE
          || cmd.OpenLoopDuty != lastDriveDuty)
      {
        notifyInt(RTE_CMD_DRIVE_OPEN_LOOP, cmd.OpenLoopDuty);
      }
    }
    else if (modeChanged || lastSel == COM_DRIVECMD_OPENLOOPSEL_DRIVE
             || targetRaw != lastTargetRaw)
    {
      notifyTarget(targetRaw);
    }

    lastTargetRaw = targetRaw;
  }

  const int16_t steerDuty =
    cmd.OpenLoopSel == COM_DRIVECMD_OPENLOOPSEL_STEER ? cmd.OpenLoopDuty : 0;
  if (steerDuty != lastSteerDuty)
  {
    notifyInt(RTE_CMD_STEER_OPEN_LOOP, steerDuty);
  }

  lastMode      = cmd.Mode;
  lastSel       = cmd.OpenLoopSel;
  lastDriveDuty = cmd.OpenLoopDuty;
  lastSteerDuty = steerDuty;
}


void Cdd_CanGateway_Init()
{
  Com_Init();

  if (!CanIf_Init())
  {
    Rte_Call_Console_WriteText("CAN INIT FAILED");
    Rte_Call_Console_WriteNewline();
  }
}


void Cdd_CanGateway_MainFunctionRx(unsigned long now)
{
  if (Com_ConsumeSafeStateRequest() && !Rte_Read_FaultLatch_E2e())
  {
    Rte_Notify_ComSafeState();
    Rte_Call_Console_WriteText("E2E SAFE STATE");
    Rte_Call_Console_WriteNewline();
  }

  Com_DriveCmdType cmd;
  if (!Com_ReceiveDriveCmd(cmd))
  {
    return;
  }

  // 시리얼의 "줄이 들어왔다" 와 같은 뜻이다. 기존 3 s 워치독은 뒤에 그대로 있다.
  Rte_Call_Watchdog_Kick(now);

  handleLatchClear(cmd);

  if (Rte_Read_FaultLatch_E2e())
  {
    return;
  }

  if (Rte_Read_Watchdog_Enabled() == cmd.WatchdogDisable)
  {
    notifyInt(RTE_CMD_WATCHDOG_TOGGLE, 0);
  }

  applyDriveCommand(cmd);
}


void Cdd_CanGateway_MainFunctionTx(unsigned long now)
{
  if (Com_IsDriveStatusDue(now))
  {
    const float speedKmh = Rte_Read_ActualSpeedKmh();
    const float raw = speedKmh * COM_DRIVESTATUS_MEASUREDSPEED_KMH_RAW_PER_UNIT;

    Com_DriveStatusType status;
    status.MeasuredSpeed_kmh =
      raw >= 32767.0f ? 32767
      : raw <= -32768.0f ? -32768
      : (int16_t)(raw + (raw >= 0.0f ? 0.5f : -0.5f));
    status.SteerPot      = (uint16_t)Rte_Call_SteerPot_Read();
    status.DrivePwm      = (uint8_t)Rte_Read_DrivePwm();
    status.SteerPwm      = (uint8_t)Rte_Read_SteerPwm();
    status.FaultSteer    = Rte_Read_FaultLatch_SteerFault();
    status.FaultStall    = Rte_Read_FaultLatch_SteerStall();
    status.FaultWatchdog = Rte_Read_Watchdog_Expired();
    status.FaultE2E      = Rte_Read_FaultLatch_E2e();
    status.Alive         = 0;
    status.CRC8          = 0;
    Com_SendDriveStatus(status);
  }

  Com_MainFunctionTx(now);
}
