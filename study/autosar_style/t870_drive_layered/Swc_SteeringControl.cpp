#include "Swc_SteeringControl.h"
#include "Rte.h"
#include "Lib_Math.h"


// ============================================================
// Pixhawk 조향 PWM 매핑
//
// FMU PWM OUT 1 = ArduPilot SERVO9 (Ground Steering, function 26)
// 실측/설정 범위: 1100(좌) / 1500(중앙) / 1900(우)
// ============================================================
static const unsigned int PIXHAWK_PWM_MIN_US    = 1100;
static const unsigned int PIXHAWK_PWM_CENTER_US = 1500;
static const unsigned int PIXHAWK_PWM_MAX_US    = 1900;


// ============================================================
// 조향 캘리브레이션 (아두이노 5V 급전 기준)
//
//   좌측 끝            중앙              우측 끝
//     32 ------------- 543 ------------ 1020
//
// POT 값은 좌회전할수록 작아지고, 우회전할수록 커진다.
// ============================================================
static const int POT_AT_RIGHT_LOCK = 1020;
// [2026-09-14 GPS 직진 실측] 556 은 참중앙보다 13 counts 높아 상시 0.49도
// 우타각이 걸려 있었다.
static const int POT_AT_CENTER     = 543;
static const int POT_AT_LEFT_LOCK  = 32;

// 소프트 리밋. 좌우를 바꿔 넣어도 따라오도록 min/max 로 계산한다.
static const int POT_LOCK_MARGIN = 15;

static const int POT_SOFT_MIN =
  ((POT_AT_RIGHT_LOCK < POT_AT_LEFT_LOCK)
     ? POT_AT_RIGHT_LOCK : POT_AT_LEFT_LOCK) + POT_LOCK_MARGIN;

static const int POT_SOFT_MAX =
  ((POT_AT_RIGHT_LOCK > POT_AT_LEFT_LOCK)
     ? POT_AT_RIGHT_LOCK : POT_AT_LEFT_LOCK) - POT_LOCK_MARGIN;

// 고장 판정. 단선 -> ADC 0 근처. 단락(1023)은 정상 우측 끝과 ADC 값만으로
// 구분할 수 없으므로 바깥 방향은 소프트 리밋 1005 에서 막는다.
static const int     STEER_FAULT_LOW   = 20;
static const int     STEER_FAULT_HIGH  = 1023;
static const uint8_t STEER_FAULT_COUNT = 5;      // 연속 5회만 인정 (스파이크 방지)

// 스톨 판정. 실차에서 중앙 부근 정지마찰을 스톨로 오검출해 비활성화했다.
static const bool          ENABLE_STEER_STALL = false;
static const int           STEER_STALL_DELTA = 5;
static const unsigned long STEER_STALL_MS    = 500;


// ============================================================
// 조향 제어
// ============================================================
static const int STEER_DEADBAND = 15;

// Timer1 3.9kHz 기준 값. 옛 490Hz 값(60/150)을 쓰면 리밋 사이클이 난다.
static const int STEER_MIN_PWM = 45;     // 정지마찰 극복 최소 duty
static const int STEER_MAX_PWM = 100;    // 캘리브레이션 중 상한

// 중앙에서 이 거리 안쪽이면 비례 제어, 바깥이면 MAX_PWM.
static const int STEER_SLOW_RANGE = 150;


// ============================================================
// 내부 상태
// ============================================================
static int steerOpenLoopDuty = 0;


static int pixhawkPwmToTargetPot(unsigned int pulseUs)
{
  long limited = Lib_ConstrainL((long)pulseUs,
                                PIXHAWK_PWM_MIN_US,
                                PIXHAWK_PWM_MAX_US);

  if (limited <= PIXHAWK_PWM_CENTER_US)
  {
    return (int)Lib_Map(limited,
                        PIXHAWK_PWM_MIN_US, PIXHAWK_PWM_CENTER_US,
                        POT_SOFT_MIN,       POT_AT_CENTER);
  }

  return (int)Lib_Map(limited,
                      PIXHAWK_PWM_CENTER_US, PIXHAWK_PWM_MAX_US,
                      POT_AT_CENTER,         POT_SOFT_MAX);
}


void Swc_SteeringControl_Init()
{
  // 원본 setup() 처럼 RC 신호가 붙기 전에도 상태 줄이 현재 중앙값을 보이게 한다.
  Rte_Write_SteerTarget(POT_AT_CENTER);
}


void Swc_SteeringControl_Run(unsigned long now)
{
  static uint8_t       faultCount    = 0;
  static int           stallLastPot  = 0;
  static unsigned long stallLastTime = 0;

  int potValue = Rte_Call_SteerPot_Read();


  // ---- 센서 고장 판정. 문서 9.1(1) ----
  // 와이퍼가 끊기면 ADC 가 0 으로 떨어진다. 이 검사가 없으면 조향 모터를
  // MAX_PWM 으로 한쪽 끝까지 밀어붙인 채 영원히 멈추지 않는다.
  if (potValue < STEER_FAULT_LOW || potValue > STEER_FAULT_HIGH)
  {
    if (faultCount < STEER_FAULT_COUNT)
    {
      faultCount++;
    }

    if (faultCount >= STEER_FAULT_COUNT)
    {
      Rte_Call_FaultLatch_SetSteerFault();
    }
  }
  else
  {
    faultCount = 0;
  }


  if (Rte_Read_FaultLatch_SteerFault() || Rte_Read_FaultLatch_SteerStall())
  {
    Rte_Call_SteerMotor_SetPwm(0);

    stallLastPot  = potValue;
    stallLastTime = now;
    return;
  }


  // ---- 조향 개루프 ----
  // 최소 구동 duty 를 찾을 때 쓴다. 스톨 판정은 적용하지 않고,
  // 소프트 리밋과 폴트는 그대로다.
  if (Rte_Read_SteerOpenLoop())
  {
    bool openPotUp = (steerOpenLoopDuty >= 0);
    int  openDuty  = Lib_AbsI(steerOpenLoopDuty);

    if ((openPotUp  && potValue >= POT_SOFT_MAX) ||
        (!openPotUp && potValue <= POT_SOFT_MIN))
    {
      Rte_Call_SteerMotor_SetPwm(0);
    }
    else
    {
      Rte_Call_SteerMotor_SetDirection(openPotUp);
      Rte_Call_SteerMotor_SetPwm(openDuty);
    }

    stallLastPot  = potValue;
    stallLastTime = now;
    return;
  }


  // 리시버 또는 Arduino 간 통신이 끊기면 모드를 추측하지 않고 정지한다.
  if (!Rte_Read_Remote_Valid())
  {
    Rte_Call_SteerMotor_SetPwm(0);

    stallLastPot  = potValue;
    stallLastTime = now;
    return;
  }

  int targetPot;

  if (Rte_Read_Remote_AutoMode())
  {
    // AUTO: Pixhawk 목표 조향각. 유효 PWM이 없거나 150ms 이상 끊기면 정지한다.
    unsigned int pwmUs;
    const bool pwmValid = Rte_Call_Pixhawk_ReadPwm(pwmUs);
    Rte_Write_PixhawkPwmValid(pwmValid);
    Rte_Write_PixhawkPwmUs(pwmUs);

    if (!pwmValid)
    {
      Rte_Call_SteerMotor_SetPwm(0);

      stallLastPot  = potValue;
      stallLastTime = now;
      return;
    }

    targetPot = pixhawkPwmToTargetPot(pwmUs);
  }
  else
  {
    // MANUAL: 보조 Arduino가 정규화한 -100..100 조향값을 POT 목표로 변환한다.
    Rte_Write_PixhawkPwmValid(false);
    const int steeringCommand = Rte_Read_Remote_SteeringCommand();
    if (steeringCommand >= 0)
    {
      targetPot = (int)Lib_Map(steeringCommand, 0, 100,
                               POT_AT_CENTER, POT_SOFT_MAX);
    }
    else
    {
      targetPot = (int)Lib_Map(steeringCommand, -100, 0,
                               POT_SOFT_MIN, POT_AT_CENTER);
    }
  }

  Rte_Write_SteerTarget(targetPot);

  int error    = targetPot - potValue;
  int absError = Lib_AbsI(error);


  // ---- 목표 도달 ----
  if (absError <= STEER_DEADBAND)
  {
    Rte_Call_SteerMotor_SetPwm(0);

    stallLastPot  = potValue;
    stallLastTime = now;
    return;
  }


  // ---- 방향 ----
  bool potUp = (error > 0);


  // ---- 소프트 리밋. 문서 9.1(2) ----
  // "그 방향으로 더 나가려 할 때"만 막는다. 이미 리밋 밖에 있어도
  // 중앙으로 돌아오는 것은 허용해야 한다.
  if ((potUp  && potValue >= POT_SOFT_MAX) ||
      (!potUp && potValue <= POT_SOFT_MIN))
  {
    Rte_Call_SteerMotor_SetPwm(0);

    stallLastPot  = potValue;
    stallLastTime = now;
    return;
  }


  // ---- 비례 제어 ----
  int pwmValue;

  if (absError >= STEER_SLOW_RANGE)
  {
    pwmValue = STEER_MAX_PWM;
  }
  else
  {
    pwmValue = Lib_Map(absError,
                       STEER_DEADBAND, STEER_SLOW_RANGE,
                       STEER_MIN_PWM,  STEER_MAX_PWM);
  }

  pwmValue = (int)Lib_ConstrainL(pwmValue, STEER_MIN_PWM, STEER_MAX_PWM);


  Rte_Call_SteerMotor_SetDirection(potUp);
  Rte_Call_SteerMotor_SetPwm(pwmValue);


  // ---- 스톨 보호. 문서 9.1(3) ----
  // 구동 중인데 pot 이 STALL_MS 동안 STALL_DELTA 만큼도 안 변하면
  // 링키지가 걸렸거나 포텐셔미터가 고장난 것이다. 'r' 로 해제한다.
  if (!ENABLE_STEER_STALL)
  {
    // 기능을 다시 켰을 때 과거 시간값 때문에 즉시 래치되지 않도록 한다.
    stallLastPot  = potValue;
    stallLastTime = now;
  }
  else if (Lib_AbsI(potValue - stallLastPot) >= STEER_STALL_DELTA)
  {
    stallLastPot  = potValue;
    stallLastTime = now;
  }
  else if (now - stallLastTime > STEER_STALL_MS)
  {
    Rte_Call_FaultLatch_SetSteerStall();

    Rte_Call_SteerMotor_SetPwm(0);

    Rte_Call_Console_WriteText("!! STEER STALL LATCHED  ('r' to clear)");
    Rte_Call_Console_WriteNewline();
  }
}


// 조향 개루프 (STEER_MIN_PWM 탐색용). 부호가 방향, 크기가 duty.
void Swc_SteeringControl_OnOpenLoopCmd(int duty)
{
  steerOpenLoopDuty = (int)Lib_ConstrainL(duty, -255, 255);
  Rte_Write_SteerOpenLoop(steerOpenLoopDuty != 0);

  Rte_Call_Console_WriteText("STEER OPEN LOOP duty = ");
  Rte_Call_Console_WriteLong(steerOpenLoopDuty);
  Rte_Call_Console_WriteNewline();
}


void Swc_SteeringControl_OnCommandTimeout()
{
  Rte_Write_SteerOpenLoop(false);
}


int Swc_SteeringControl_GetPotCenter()  { return POT_AT_CENTER; }
int Swc_SteeringControl_GetPotSoftMin() { return POT_SOFT_MIN; }
int Swc_SteeringControl_GetPotSoftMax() { return POT_SOFT_MAX; }
