#include <math.h>

#include "Swc_SpeedControl.h"
#include "Swc_DriveParams.h"
#include "Rte.h"
#include "Lib_Math.h"


// ============================================================
// 엔코더 / 바퀴
// ============================================================

// 실차 확인: 엔코더 초록색(A상)->D2, 파란색(B상)->D3 배선에서
// 전진할 때 카운트가 양수로 증가하므로 +1 이 맞다.
static const int ENCODER_SIGN = 1;

// [2026-08-27 실차 재측정] 바퀴 10회전 = 약 1498 counts.
static const float COUNTS_PER_REV = 149.8;

static const float WHEEL_DIAMETER      = 0.27;                      // m
static const float WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * 3.14159;  // 0.848 m

// 저속 양자화 완화용 이동평균
// 1 km/h 에서 100ms 당 5 counts뿐이라 +-1 count 가 크게 잡힌다.
static const uint8_t SPEED_FILTER_N = 3;


// ============================================================
// 램프 / PI / 피드포워드
// ============================================================

// 속도 램프 (소프트 스타트). km/h per sec
static const float ACCEL_RATE = 0.5;
static const float DECEL_RATE = 0.7;

// 피드포워드. duty = FF_OFFSET + FF_GAIN * |목표 km/h|
// [2026-08-27 지면 부하 실측] PWM 10 -> 0.20 km/h, PWM 15 -> 0.48 km/h
static const float FF_OFFSET = 6.4;
static const float FF_GAIN   = 17.9;

// MANUAL은 기존 실차 확인값을 유지한다. AUTO 5 km/h에서는 피드포워드가
// 대부분의 출력을 담당하고 PI는 오차만 보정하도록 게인을 낮춘다.
static const float MANUAL_KP = 22.0;
static const float MANUAL_KI = 8.0;
static const float AUTO_KP = 20.0;
static const float AUTO_KI = 5.0;
static const float MANUAL_INTEGRAL_LIMIT = 20.0;
static const float AUTO_INTEGRAL_LIMIT = 10.0;

static const int DRIVE_MIN_PWM = 5;
static const int DRIVE_MAX_PWM = 200;

// AUTO/MANUAL/신호단절 전환 직후 강제 정지 시간
static const unsigned long MODE_CHANGE_STOP_MS = 1000;


// ============================================================
// 내부 상태 (다른 SWC 는 보지 않는다)
// ============================================================

static float   integral = 0.0;
static int     openLoopDuty = 0;
static long    previousEncoderCount = 0;
static float   speedBuffer[SPEED_FILTER_N] = {0.0, 0.0, 0.0};
static uint8_t speedBufferIndex = 0;


// 감속 램프와 PI를 우회하는 구동 급정지 공통 함수.
// PWM/DIR 방식 모터드라이버에서 PWM=0으로 즉시 출력을 차단한다.
// 수신 두절/모드 전환/E-stop 은 미션 홀드도 해제한다.
static void emergencyStopDrive()
{
  Rte_Write_CommandSpeedKmh(0.0);
  Rte_Write_RampedSpeedKmh(0.0);
  integral = 0.0;
  Rte_Write_OpenLoopMode(false);
  openLoopDuty = 0;
  Rte_Call_StopHold_Release();
  Rte_Call_DriveMotor_SetPwm(0);
}


void Swc_SpeedControl_Init()
{
  Rte_Write_DriveDirForward(true);
}


// ============================================================
// 명령 선택 (원본 selectControlCommand)
// ============================================================

void Swc_SpeedControl_SelectCommand(unsigned long now)
{
  static bool previousValid = false;
  static bool previousAuto = false;

  const bool remoteValid = Rte_Read_Remote_Valid();
  const bool remoteAuto  = Rte_Read_Remote_AutoMode();

  const bool modeChanged =
    previousValid != remoteValid
    || (remoteValid && previousAuto != remoteAuto);

  if (modeChanged)
  {
    // AUTO/MANUAL/신호단절 전환은 감속 램프를 우회해 즉시 PWM을 끈다.
    emergencyStopDrive();
    Rte_Write_ModeChangeStopUntilMs(now + MODE_CHANGE_STOP_MS);

    // MANUAL 진입 때 스로틀이 이미 당겨져 있으면 갑자기 출발하지 않는다.
    Rte_Write_ManualNeutralRequired(remoteValid && !remoteAuto);
  }

  // 리시버 또는 보조 Arduino 통신이 끊긴 동안에는 매 loop마다
  // 급정지 출력을 재적용한다. 개루프/시리얼 명령도 이를 우회할 수 없다.
  Rte_Write_ReceiverFailsafeActive(!remoteValid);
  if (Rte_Read_ReceiverFailsafeActive())
  {
    emergencyStopDrive();
  }

  if (remoteValid && !remoteAuto
      && Lib_AbsI(Rte_Read_Remote_ThrottleCommand()) <= MANUAL_REARM_DEADBAND)
  {
    Rte_Write_ManualNeutralRequired(false);
  }

  const bool transitionStop =
    (long)(Rte_Read_ModeChangeStopUntilMs() - now) > 0;

  if (!remoteValid || transitionStop
      || (!remoteAuto && Rte_Read_ManualNeutralRequired()))
  {
    Rte_Write_CommandSpeedKmh(0.0);
  }
  else if (remoteAuto)
  {
    Rte_Write_CommandSpeedKmh(Rte_Read_SerialSpeedCmdKmh());
  }
  else
  {
    Rte_Write_CommandSpeedKmh(
      ((float)Rte_Read_Remote_ThrottleCommand() / 100.0) * MAX_SPEED_KMH);
    // A live MANUAL report is itself the command watchdog.
    Rte_Call_Watchdog_Kick(now);
  }

  previousValid = remoteValid;
  previousAuto = remoteAuto;
}


// ============================================================
// 속도 측정 (원본 driveControl 앞부분)
//
// abs() 를 쓰지 않는다. 경사에서 뒤로 밀릴 때 전진 명령 중이면 실제로는
// 후진인데 abs() 를 쓰면 "전진 중"으로 읽어 PI 가 출력을 줄인다.
// ============================================================

void Swc_SpeedControl_MeasureSpeed(float dt)
{
  if (dt <= 0.0)
  {
    return;
  }

  long currentEncoderCount = Rte_Call_Encoder_GetCount();

  long countDifference = currentEncoderCount - previousEncoderCount;
  previousEncoderCount = currentEncoderCount;
  Rte_Write_EncoderCountDiff(countDifference);

  float revolutions = (float)countDifference / COUNTS_PER_REV;
  float distance    = revolutions * WHEEL_CIRCUMFERENCE;
  float rawKmh      = (distance / dt) * 3.6 * ENCODER_SIGN;

  // 이동평균. 문서 9.2(8)
  speedBuffer[speedBufferIndex] = rawKmh;
  speedBufferIndex = (speedBufferIndex + 1) % SPEED_FILTER_N;

  float sum = 0.0;
  for (uint8_t i = 0; i < SPEED_FILTER_N; i++)
  {
    sum += speedBuffer[i];
  }
  Rte_Write_ActualSpeedKmh(sum / SPEED_FILTER_N);
}


// ============================================================
// 개루프 · 램프 · 방향 반전 보호 · PI (원본 driveControl 뒷부분)
//
// Swc_StopHold_Run() 이 이번 주기 출력을 정했으면 여기서는 아무것도 하지 않는다.
// ============================================================

void Swc_SpeedControl_Run(float dt)
{
  if (dt <= 0.0)
  {
    return;
  }

  if (Rte_Read_StopHoldOverride())
  {
    return;
  }

  const float actualSpeedKmh = Rte_Read_ActualSpeedKmh();


  // ---- 개루프 모드 (FF 실측용) ----
  if (Rte_Read_OpenLoopMode())
  {
    Rte_Call_DriveMotor_SetDirection(true);
    Rte_Write_DriveDirForward(true);

    integral = 0.0;

    Rte_Call_DriveMotor_SetPwm(openLoopDuty);
    return;
  }


  // ---- 속도 램프 (소프트 스타트) ----
  // 부호가 반대면 아직 0 을 향해 줄어드는 중이므로 항상 감속률을 쓴다.
  const float commandSpeedKmh = Rte_Read_CommandSpeedKmh();
  float rampedSpeedKmh = Rte_Read_RampedSpeedKmh();

  bool speedingUp = (fabs(commandSpeedKmh) > fabs(rampedSpeedKmh)) &&
                    (commandSpeedKmh * rampedSpeedKmh >= 0.0);

  float step = (speedingUp ? ACCEL_RATE : DECEL_RATE) * dt;

  if (rampedSpeedKmh < commandSpeedKmh)
  {
    rampedSpeedKmh = Lib_MinF(rampedSpeedKmh + step, commandSpeedKmh);
  }
  else if (rampedSpeedKmh > commandSpeedKmh)
  {
    rampedSpeedKmh = Lib_MaxF(rampedSpeedKmh - step, commandSpeedKmh);
  }
  Rte_Write_RampedSpeedKmh(rampedSpeedKmh);


  // ---- 방향 반전 보호. 문서 9.2(4) ----
  // 굴러가는 중에 DIR 을 뒤집으면 역전 제동이 걸려 드라이버와
  // 모터에 큰 전류가 흐른다. 일단 0 으로 감속하고, 선 다음에 반전한다.
  const bool wantForward = (rampedSpeedKmh >= 0.0);

  float effectiveTarget = fabs(rampedSpeedKmh);

  if (wantForward != Rte_Read_DriveDirForward())
  {
    if (fabs(actualSpeedKmh) > REVERSE_GUARD_KMH)
    {
      effectiveTarget = 0.0;      // 아직 구른다. 먼저 세운다.
    }
    else
    {
      Rte_Write_DriveDirForward(wantForward);
      Rte_Call_DriveMotor_SetDirection(wantForward);
      integral = 0.0;
    }
  }


  // ---- 정지 ----
  if (effectiveTarget < STOP_THRESHOLD_KMH)
  {
    integral = 0.0;
    Rte_Call_DriveMotor_SetPwm(0);
    return;
  }


  // ---- PI + 피드포워드 ----
  // 진행 방향 기준 속도로 오차를 잡는다. 전진 명령 중에 뒤로 밀리면
  // speedAlongDir 이 음수가 되어 오차가 커지고, PI 가 더 밟는다.
  float dirSign       = Rte_Read_DriveDirForward() ? 1.0 : -1.0;
  float speedAlongDir = actualSpeedKmh * dirSign;

  float error = effectiveTarget - speedAlongDir;

  float ff = FF_OFFSET + FF_GAIN * effectiveTarget;

  // 조건부 적분 (anti-windup). 문서 9.2(6)
  // 적분을 더했을 때 출력이 포화되면 누적하지 않는다.
  const bool  remoteAuto = Rte_Read_Remote_AutoMode();
  const float activeKp = remoteAuto ? AUTO_KP : MANUAL_KP;
  const float activeKi = remoteAuto ? AUTO_KI : MANUAL_KI;
  const float integralLimit = remoteAuto
    ? AUTO_INTEGRAL_LIMIT : MANUAL_INTEGRAL_LIMIT;
  float trialIntegral = Lib_ConstrainF(
    integral + error * dt, -integralLimit, integralLimit);
  float control = ff + activeKp * error + activeKi * trialIntegral;

  if (control > 0.0 && control < (float)DRIVE_MAX_PWM)
  {
    integral = trialIntegral;
  }
  else
  {
    control = ff + activeKp * error + activeKi * integral;
  }

  // 출력이 0 이하면 duty 0(관성주행)으로 둔다. 문서 9.2(7)
  int duty;
  if (control <= 0.0)
  {
    duty = 0;
  }
  else
  {
    duty = (int)Lib_ConstrainL((int)control, DRIVE_MIN_PWM, DRIVE_MAX_PWM);
  }

  Rte_Call_DriveMotor_SetPwm(duty);
}


// ============================================================
// 시리얼 명령 처리
// ============================================================

// 일반 0 km/h 명령은 DECEL_RATE 램프를 쓰지만 E-stop 은 램프와 PI 적분을
// 우회해 구동 PWM 을 즉시 차단한다. 물리 RC MANUAL 에서는 ROS 노드가 보내는
// E 가 반복되어도 RC 구동 램프와 PI 를 리셋하지 않는다.
void Swc_SpeedControl_OnEmergencyStopCmd()
{
  Rte_Write_SerialSpeedCmdKmh(0.0);

  if (!Rte_Read_Remote_Valid() || Rte_Read_Remote_AutoMode())
  {
    emergencyStopDrive();
    Rte_Call_Console_WriteText("EMERGENCY STOP");
    Rte_Call_Console_WriteNewline();
  }
}


// 구동 개루프 duty 인가 (문서 10.3)
void Swc_SpeedControl_OnOpenLoopCmd(int duty)
{
  Rte_Write_OpenLoopMode(true);
  openLoopDuty = (int)Lib_ConstrainL(duty, 0, DRIVE_MAX_PWM);

  integral = 0.0;
  Rte_Write_CommandSpeedKmh(0.0);
  Rte_Write_SerialSpeedCmdKmh(0.0);
  Rte_Write_RampedSpeedKmh(0.0);

  Rte_Call_Console_WriteText("OPEN LOOP duty = ");
  Rte_Call_Console_WriteLong(openLoopDuty);
  Rte_Call_Console_WriteNewline();
}


void Swc_SpeedControl_OnTargetSpeedCmd(float targetKmh)
{
  float v = Lib_ConstrainF(targetKmh, -MAX_SPEED_KMH, MAX_SPEED_KMH);

  Rte_Write_OpenLoopMode(false);
  // 0 이 아닌 새 목표 속도가 들어오면 미션 홀드는 끝난 것이다.
  // 0 명령으로는 풀지 않는다. ROS 브리지는 구 펌웨어 호환을 위해
  // 홀드 중에도 '0.000' 을 먼저 보내고 H 를 보내기 때문이다.
  if (fabs(v) >= STOP_THRESHOLD_KMH)
  {
    Rte_Call_StopHold_Release();
  }

  // 목표가 실제로 바뀔 때만 적분을 리셋한다. ROS 브리지가 10~20Hz 로
  // 같은 목표를 계속 보낼 때 매번 리셋하면 사실상 P 제어가 된다.
  if (Rte_Read_Remote_AutoMode() && fabs(v - Rte_Read_SerialSpeedCmdKmh()) > 0.01)
  {
    integral = 0.0;
  }

  Rte_Write_SerialSpeedCmdKmh(v);

  Rte_Call_Console_WriteText("TARGET = ");
  Rte_Call_Console_WriteFloat(v, 2);
  Rte_Call_Console_WriteText(" km/h");
  Rte_Call_Console_WriteNewline();
}


void Swc_SpeedControl_OnCommandTimeout()
{
  Rte_Write_SerialSpeedCmdKmh(0.0);
  Rte_Write_OpenLoopMode(false);
}


// ============================================================
// 서버 연산
// ============================================================

void Swc_SpeedControl_ResetRamp(float rampedSpeedKmh)
{
  Rte_Write_RampedSpeedKmh(rampedSpeedKmh);
  integral = 0.0;
}


void Swc_SpeedControl_CancelOpenLoop()
{
  Rte_Write_OpenLoopMode(false);
  openLoopDuty = 0;
}


float Swc_SpeedControl_GetMaxSpeedKmh()
{
  return MAX_SPEED_KMH;
}
