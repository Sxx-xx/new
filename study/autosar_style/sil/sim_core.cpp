#include <math.h>
#include <deque>

#include "sim_core.h"


namespace sim
{

// ============================================================
// 모의 플랜트 상수
//
// [주의] 아래 값은 전부 그래프를 그리기 위한 가정값이다. 실차에서 잰
// 값이 아니다. 차가 펌웨어 피드포워드(duty = 6.4 + 17.9 * km/h)와 일부러
// 조금 다르게 반응하도록 잡아, PI 가 오차를 메우는 모습이 보이게 했다.
// ============================================================

// 구동: duty -> 정상상태 속도의 1차 지연
static const double DRIVE_GAIN_KMH_PER_DUTY = 0.050;  // 가정값
static const double DRIVE_DEADZONE_DUTY     = 4.0;    // 가정값
static const double DRIVE_TAU_S             = 0.40;   // 가정값. 구동 시 시정수
static const double COAST_TAU_S             = 2.00;   // 가정값. PWM 0 관성주행 감속 시정수
static const double GRAVITY                 = 9.81;

// 엔코더: 펌웨어의 COUNTS_PER_REV, 바퀴 지름과 같다고 가정한다.
static const double ENCODER_COUNTS_PER_REV = 149.8;
static const double WHEEL_CIRCUMFERENCE_M  = 0.27 * 3.14159;

// 조향: duty 가 정지마찰을 넘으면 넘은 만큼에 비례한 속도로 POT 가 움직인다.
static const double STEER_STICTION_DUTY      = 30.0;  // 가정값
static const double STEER_RATE_PER_DUTY      = 12.0;  // 가정값. ADC count/s per duty
static const double STEER_MECH_MIN           = 32.0;  // 원본 캘리브레이션의 기계적 끝
static const double STEER_MECH_MAX           = 1020.0;


unsigned long nowMs = 0;
static bool microsOverrideActive = false;
static unsigned long microsOverrideUs = 0;

int drivePwm = 0;
int driveDirLevel = 0;
int steerPwm = 0;
int steerDirLevel = 0;

double grade = 0.0;
double speedKmh = 0.0;
double potValue = 543.0;
static double encoderAccum = 0.0;

Remote remote = {true, true, true, 0, 0, 1500, 1500, 0};

bool         pixhawkEnabled = true;
unsigned int pixhawkUs = 1500;

static std::deque<char> serialIn;
static std::string serialOut;


unsigned long micros()
{
  return microsOverrideActive ? microsOverrideUs : nowMs * 1000UL;
}


void setMicrosOverride(bool active, unsigned long us)
{
  microsOverrideActive = active;
  microsOverrideUs = us;
}


void plantReset(double initialPot)
{
  speedKmh = 0.0;
  potValue = initialPot;
  encoderAccum = 0.0;
}


void plantStep(double dt)
{
  // ---- 구동 ----
  double accel;   // km/h per s
  if (drivePwm > 0)
  {
    double steady = DRIVE_GAIN_KMH_PER_DUTY * (drivePwm - DRIVE_DEADZONE_DUTY);
    if (steady < 0.0) steady = 0.0;
    if (driveDirLevel == 0) steady = -steady;   // LOW = 후진
    accel = (steady - speedKmh) / DRIVE_TAU_S;
  }
  else
  {
    accel = -speedKmh / COAST_TAU_S;
  }
  accel -= GRAVITY * grade * 3.6;   // 오르막이면 뒤로 민다
  speedKmh += accel * dt;

  encoderAccum += (speedKmh / 3.6) * dt / WHEEL_CIRCUMFERENCE_M * ENCODER_COUNTS_PER_REV;
  while (encoderAccum >= 1.0)
  {
    Backend_EncoderEdge(+1);
    encoderAccum -= 1.0;
  }
  while (encoderAccum <= -1.0)
  {
    Backend_EncoderEdge(-1);
    encoderAccum += 1.0;
  }

  // ---- 조향 ----
  if (steerPwm > STEER_STICTION_DUTY)
  {
    double rate = STEER_RATE_PER_DUTY * (steerPwm - STEER_STICTION_DUTY);
    potValue += (steerDirLevel == 0 ? rate : -rate) * dt;   // LOW = POT 증가
    if (potValue < STEER_MECH_MIN) potValue = STEER_MECH_MIN;
    if (potValue > STEER_MECH_MAX) potValue = STEER_MECH_MAX;
  }
}


int potAdc()
{
  return (int)lround(potValue);
}


// 보조 보드 pwm_receiver_bridge.ino 의 RemoteReport 와 같은 바이트 배치.
size_t buildRemoteReport(uint8_t *b)
{
  if (!remote.boardAlive)
  {
    return 0;
  }

  remote.sequence++;
  const bool modeValid = remote.connected;
  b[0] = 0xB7;
  b[1] = 1;
  b[2] = remote.sequence;
  b[3] = (uint8_t)(int8_t)(remote.connected && !remote.autoMode ? remote.throttle : 0);
  b[4] = (uint8_t)(int8_t)(remote.connected && !remote.autoMode ? remote.steering : 0);
  b[5] = (uint8_t)((modeValid ? 0x02 : 0) | (modeValid && remote.autoMode ? 0x01 : 0));
  b[6] = remote.connected ? 1 : 0;
  b[7] = (uint8_t)(remote.throttleRaw & 0xFF);
  b[8] = (uint8_t)(remote.throttleRaw >> 8);
  b[9] = (uint8_t)(remote.steeringRaw & 0xFF);
  b[10] = (uint8_t)(remote.steeringRaw >> 8);
  uint8_t checksum = 0;
  for (int i = 0; i < 11; ++i) checksum ^= b[i];
  b[11] = checksum;
  return 12;
}


void pushSerialLine(const std::string &line)
{
  for (char c : line) serialIn.push_back(c);
  serialIn.push_back('\n');
}


int serialAvailable()
{
  return (int)serialIn.size();
}


int serialRead()
{
  if (serialIn.empty()) return -1;
  char c = serialIn.front();
  serialIn.pop_front();
  return (unsigned char)c;
}


void serialWrite(const std::string &text)
{
  serialOut += text;
}


void serialWriteLong(long value)
{
  serialOut += std::to_string(value);
}


void serialWriteULong(unsigned long value)
{
  serialOut += std::to_string(value);
}


// Arduino AVR 코어 Print::printFloat() 와 같은 알고리즘.
void serialWriteFloat(double number, int digits)
{
  if (isnan(number)) { serialOut += "nan"; return; }
  if (isinf(number)) { serialOut += "inf"; return; }
  if (number > 4294967040.0 || number < -4294967040.0) { serialOut += "ovf"; return; }

  if (number < 0.0)
  {
    serialOut += '-';
    number = -number;
  }

  double rounding = 0.5;
  for (int i = 0; i < digits; ++i) rounding /= 10.0;
  number += rounding;

  unsigned long intPart = (unsigned long)number;
  double remainder = number - (double)intPart;
  serialOut += std::to_string(intPart);

  if (digits > 0) serialOut += '.';
  while (digits-- > 0)
  {
    remainder *= 10.0;
    unsigned int toPrint = (unsigned int)remainder;
    serialOut += std::to_string(toPrint);
    remainder -= toPrint;
  }
}


const std::string &serialOutput()
{
  return serialOut;
}

}  // namespace sim
