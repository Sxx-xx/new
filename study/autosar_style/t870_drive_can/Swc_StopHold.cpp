#include <math.h>

#include "Swc_StopHold.h"
#include "Swc_DriveParams.h"
#include "Rte.h"
#include "Lib_Math.h"


// ============================================================
// MANUAL 능동 제동
//
// 주행 방향과 반대 스로틀이 들어오면 짧게 역토크를 걸어 관성 정지보다
// 빠르게 감속한다. 한 번의 스틱 반전에는 한 번만 동작하며, 정지/중립
// 확인 전에는 반복 펄스를 허용하지 않는다.
// ============================================================
static const float MANUAL_BRAKE_TRIGGER_KMH = 0.4;
static const int   MANUAL_BRAKE_PWM = 65;
static const unsigned long MANUAL_BRAKE_MS = 250;

// ============================================================
// AUTO 능동 제동
//
// 시리얼 목표가 지금 굴러가는 방향과 반대로 들어오면 제한된 역토크를
// 짧게 건다. GPS 추종은 목표를 계속 유지하므로 쉬는 시간을 두고 반복한다.
// ON 250 ms / OFF 150 ms 로 끊어 걸어 평균 전류를 낮춘다.
// ============================================================
static const float AUTO_BRAKE_TRIGGER_KMH = 0.4;
static const int   AUTO_BRAKE_PWM = 65;
static const unsigned long AUTO_BRAKE_MS = 250;
static const unsigned long AUTO_BRAKE_GAP_MS = 150;

// ============================================================
// 미션 홀드 (경사 밀림 방지)
//
// 'H' 로 들어오는 미션 정차에서 굴러가는 것이 감지되면 반대 방향으로
// 밀리는 속도에 비례한 duty 를 준다.
//   duty = HOLD_MIN_PWM + HOLD_GAIN * (|속도| - HOLD_DEADBAND_KMH)
//   0.3 km/h -> 30,  0.5 -> 40,  1.0 -> 65,  1.5 이상 -> 90 포화
//
// 불감대 0.3 km/h: 엔코더 1 카운트가 약 0.204 km/h 라 그 아래는 측정이
// 안 되고, 불감대가 없으면 되민 뒤 반대로 또 걸려 진동한다.
//
// 이것은 완화책이지 브레이크가 아니다. 밀림을 0 으로 만들지는 못하고
// 제한할 뿐이다.
// ============================================================
static const float HOLD_DEADBAND_KMH = 0.3;
static const int   HOLD_MIN_PWM      = 30;
static const float HOLD_GAIN         = 50.0;   // km/h 당 duty
static const int   HOLD_MAX_PWM      = 90;

// 주행 중 제동 구간. 저속 곡선은 1.5 km/h 위에서 90 으로 포화되므로
// 8 km/h 로 굴러오는 차를 세우기에 약하다. 그 위에서는 속도에 비례해
// 더 세게, 대신 끊어 건다.
//   duty = 90 + HOLD_BRAKE_GAIN * (|속도| - HOLD_PULSE_ABOVE_KMH)
//   1.5 km/h -> 90,  2 -> 96,  4 -> 117,  8 -> 160 포화
static const float HOLD_PULSE_ABOVE_KMH = 1.5;
static const float HOLD_BRAKE_GAIN      = 11.0;  // km/h 당 duty
static const int   HOLD_BRAKE_MAX_PWM   = 160;
static const unsigned long HOLD_BRAKE_ON_MS  = 250;
static const unsigned long HOLD_BRAKE_OFF_MS = 150;
// 연속 홀드 상한. 스톨 토크를 무한정 물리지 않는다.
static const unsigned long HOLD_MAX_MS = 10000;
// 홀드 자체의 워치독. ROS 는 홀드 중 매 제어주기 H 를 보낸다.
static const unsigned long HOLD_REFRESH_MS = 500;


// ============================================================
// 내부 상태
// ============================================================

static bool manualBrakeActive = false;
static bool manualBrakeLatched = false;
static unsigned long manualBrakeStartMs = 0;

static bool autoBrakeActive = false;
static unsigned long autoBrakeStartMs = 0;
static unsigned long autoBrakeRestUntilMs = 0;

static bool missionHoldActive = false;
static unsigned long missionHoldStartMs = 0;
static unsigned long missionHoldRefreshMs = 0;


// 이번 주기 구동 출력을 StopHold 가 정했다고 표시하고 PWM 을 낸다.
static void overrideOutput(int duty)
{
  Rte_Write_StopHoldOverride(true);
  Rte_Call_DriveMotor_SetPwm(duty);
}


// 목표 방향으로 DIR 을 맞춘다. 굴러가는 방향과 반대면 역토크가 된다.
static void setDriveDirection(bool forward)
{
  Rte_Write_DriveDirForward(forward);
  Rte_Call_DriveMotor_SetDirection(forward);
}


void Swc_StopHold_Run(float dt)
{
  if (dt <= 0.0)
  {
    return;
  }

  Rte_Write_StopHoldOverride(false);

  const float actualSpeedKmh = Rte_Read_ActualSpeedKmh();
  const bool  remoteValid    = Rte_Read_Remote_Valid();
  const bool  remoteAuto     = Rte_Read_Remote_AutoMode();


  // ----------------------------------------------------------
  // 1) MANUAL 반대 스로틀 능동 제동
  // ----------------------------------------------------------
  const float commandSpeedKmh = Rte_Read_CommandSpeedKmh();
  const bool manualDriveAvailable =
    remoteValid && !remoteAuto && !Rte_Read_ManualNeutralRequired();
  const bool manualThrottleNeutral =
    Lib_AbsI(Rte_Read_Remote_ThrottleCommand()) <= MANUAL_REARM_DEADBAND;
  const bool oppositeManualRequest =
    manualDriveAvailable
    && !manualThrottleNeutral
    && fabs(actualSpeedKmh) >= MANUAL_BRAKE_TRIGGER_KMH
    && commandSpeedKmh * actualSpeedKmh < 0.0;

  // 중립으로 돌아왔거나 거의 정지했을 때 다음 제동 동작을 재무장한다.
  if (!manualDriveAvailable || manualThrottleNeutral
      || fabs(actualSpeedKmh) <= REVERSE_GUARD_KMH)
  {
    manualBrakeActive = false;
    manualBrakeLatched = false;
  }

  if (oppositeManualRequest && !manualBrakeLatched)
  {
    manualBrakeActive = true;
    manualBrakeLatched = true;
    manualBrakeStartMs = Rte_Call_Time_GetMs();
    Rte_Call_SpeedControl_ResetRamp(0.0);
  }

  if (manualBrakeActive)
  {
    if (Rte_Call_Time_GetMs() - manualBrakeStartMs < MANUAL_BRAKE_MS
        && oppositeManualRequest)
    {
      // 요청 방향으로 제한된 PWM을 주면 현재 회전에 대한 역토크가 된다.
      setDriveDirection(commandSpeedKmh >= 0.0);
      overrideOutput(MANUAL_BRAKE_PWM);
      return;
    }

    manualBrakeActive = false;
    overrideOutput(0);
    return;
  }


  // ----------------------------------------------------------
  // 2) 미션 홀드
  // ----------------------------------------------------------
  if (missionHoldActive)
  {
    // 안전 게이트. 수신 두절, MANUAL, 모드 전환 정지, 개루프 중에는
    // 토크를 주지 않고 PWM 을 끊는다.
    const bool holdAvailable =
      remoteValid
      && remoteAuto
      && !Rte_Read_ReceiverFailsafeActive()
      && !Rte_Read_OpenLoopMode()
      && (long)(Rte_Read_ModeChangeStopUntilMs() - Rte_Call_Time_GetMs()) <= 0;

    Rte_Call_SpeedControl_ResetRamp(0.0);

    const float drift = fabs(actualSpeedKmh);
    const bool holdExpired =
      (Rte_Call_Time_GetMs() - missionHoldStartMs) > HOLD_MAX_MS;
    // H 가 끊기면 홀드를 놓는다. ROS 나 USB 가 죽은 채로 토크를
    // 물고 있으면 안 된다.
    if ((Rte_Call_Time_GetMs() - missionHoldRefreshMs) > HOLD_REFRESH_MS)
    {
      missionHoldActive = false;
    }

    if (!missionHoldActive || !holdAvailable || holdExpired
        || drift < HOLD_DEADBAND_KMH)
    {
      // 안 밀리고 있거나 홀드할 수 없는 상태다. 관성주행으로 둔다.
      overrideOutput(0);
      return;
    }

    int duty;
    bool pulsed = false;
    if (drift < HOLD_PULSE_ABOVE_KMH)
    {
      // 저속 밀림 방지. 연속으로 약하게 건다.
      duty = HOLD_MIN_PWM + (int)(HOLD_GAIN * (drift - HOLD_DEADBAND_KMH));
      duty = (int)Lib_ConstrainL(duty, HOLD_MIN_PWM, HOLD_MAX_PWM);
    }
    else
    {
      // 주행 중 제동. 속도에 비례해 더 세게, 대신 끊어 건다.
      duty = HOLD_MAX_PWM
        + (int)(HOLD_BRAKE_GAIN * (drift - HOLD_PULSE_ABOVE_KMH));
      duty = (int)Lib_ConstrainL(duty, HOLD_MAX_PWM, HOLD_BRAKE_MAX_PWM);
      pulsed = true;
    }

    if (pulsed)
    {
      // ON/OFF 주기 안에서 쉬는 구간이면 duty 0 으로 둔다.
      const unsigned long period = HOLD_BRAKE_ON_MS + HOLD_BRAKE_OFF_MS;
      if ((Rte_Call_Time_GetMs() - missionHoldStartMs) % period >= HOLD_BRAKE_ON_MS)
      {
        overrideOutput(0);
        return;
      }
    }

    // 굴러가는 반대 방향으로 민다. actualSpeedKmh 가 엔코더 부호를
    // 따르는 값이라 전진 밀림/후진 밀림 양쪽 모두 성립한다.
    setDriveDirection(actualSpeedKmh < 0.0);
    overrideOutput(duty);
    return;
  }


  // ----------------------------------------------------------
  // 3) AUTO 반대 명령 능동 제동
  //
  // follower 가 기어를 바꾸려 할 때 반대 부호 목표를 그대로 보낸다.
  // duty 0 은 관성주행이라 몇 m 를 더 가므로 여기서 제한된 역토크로
  // 세운다. 세워지면 SpeedControl 의 역전 보호가 방향을 바꿔 준다.
  // ----------------------------------------------------------
  const float serialCommandSpeedKmh = Rte_Read_SerialSpeedCmdKmh();
  const bool autoDriveAvailable = remoteAuto && !Rte_Read_OpenLoopMode();
  const bool autoCommandNeutral =
    fabs(serialCommandSpeedKmh) < STOP_THRESHOLD_KMH;
  const bool oppositeAutoRequest =
    autoDriveAvailable
    && !autoCommandNeutral
    && fabs(actualSpeedKmh) >= AUTO_BRAKE_TRIGGER_KMH
    && serialCommandSpeedKmh * actualSpeedKmh < 0.0;

  if (!oppositeAutoRequest)
  {
    autoBrakeActive = false;
    return;
  }

  const unsigned long nowMs = Rte_Call_Time_GetMs();
  if (!autoBrakeActive && nowMs >= autoBrakeRestUntilMs)
  {
    autoBrakeActive = true;
    autoBrakeStartMs = nowMs;
    Rte_Call_SpeedControl_ResetRamp(0.0);
  }
  if (autoBrakeActive)
  {
    if (nowMs - autoBrakeStartMs < AUTO_BRAKE_MS)
    {
      setDriveDirection(serialCommandSpeedKmh >= 0.0);
      overrideOutput(AUTO_BRAKE_PWM);
      return;
    }
    autoBrakeActive = false;
    autoBrakeRestUntilMs = nowMs + AUTO_BRAKE_GAP_MS;
    if (fabs(actualSpeedKmh) <= REVERSE_GUARD_KMH)
    {
      // 섰다. 램프를 건너뛰고 목표 속도에서 바로 출발한다.
      Rte_Call_SpeedControl_ResetRamp(serialCommandSpeedKmh);
      autoBrakeRestUntilMs = 0;
    }
  }
  // 쉬는 구간에서는 duty 0 으로 둔다.
  overrideOutput(0);
}


// 대회 규정 정차(WP39 오르막)처럼 '거기 서 있어야 하는' 정지다.
// E 와 달리 구동을 완전히 놓지 않고, 밀리면 밀리는 반대로 되민다.
void Swc_StopHold_OnHoldCmd()
{
  Rte_Write_SerialSpeedCmdKmh(0.0);

  if (!Rte_Read_Remote_Valid() || Rte_Read_Remote_AutoMode())
  {
    missionHoldRefreshMs = Rte_Call_Time_GetMs();
    if (!missionHoldActive)
    {
      missionHoldActive = true;
      missionHoldStartMs = missionHoldRefreshMs;
      Rte_Call_SpeedControl_ResetRamp(0.0);
      Rte_Call_SpeedControl_CancelOpenLoop();
      Rte_Call_Console_WriteText("MISSION HOLD");
      Rte_Call_Console_WriteNewline();
    }
  }
}


void Swc_StopHold_Release()
{
  missionHoldActive = false;
}
