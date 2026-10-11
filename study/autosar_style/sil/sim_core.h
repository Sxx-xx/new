// ============================================================
// SIL 공통 코어: 가상 시간, 모의 플랜트, 가상 RC 수신기, 가상 시리얼
//
// 두 펌웨어 빌드가 같은 코어를 쓴다.
//   sil_layered  : t870_drive_layered (Swc/Rte/Cdd/Bsw) + Mcal_Sim.cpp
//   sil_original : sketch_aug.ino 원본 + shim/ (가짜 Arduino.h, Wire.h)
// 펌웨어 쪽 이름과 겹치지 않도록 전부 namespace sim 안에 둔다.
// ============================================================

#ifndef SIL_SIM_CORE_H
#define SIL_SIM_CORE_H

#include <stdint.h>
#include <stddef.h>
#include <string>

namespace sim
{

// ---- 가상 시간 ----
// loop() 한 번 = 1 ms. 한 loop 안에서는 millis() 가 변하지 않는다.
// 그래서 두 펌웨어가 millis() 를 몇 번 부르든 같은 값을 본다.
extern unsigned long nowMs;
unsigned long micros();
void setMicrosOverride(bool active, unsigned long us);   // ISR 시각 지정용

// ---- 액추에이터 (백엔드가 쓴다) ----
// DIR 은 핀 레벨 그대로 (0=LOW, 1=HIGH). 극성은 원본과 같다:
//   구동 HIGH = 전진, 조향 LOW = POT 증가 방향
extern int drivePwm;
extern int driveDirLevel;
extern int steerPwm;
extern int steerDirLevel;

// ---- 모의 플랜트 ----
extern double grade;          // 경사 (0.1 = 10% 오르막, 전진 방향 기준)
extern double speedKmh;       // 부호 있는 실제 차속
extern double potValue;       // 실제 포텐셔미터 위치 (ADC count)
void plantReset(double initialPot);
void plantStep(double dt);    // 엔코더 엣지는 Backend_EncoderEdge() 로 낸다
int  potAdc();

// ---- 가상 보조 RC 보드 (I2C 0x12) ----
struct Remote
{
  bool     boardAlive;   // false 면 I2C 응답 없음
  bool     connected;    // 수신기 신호 정상
  bool     autoMode;
  int      throttle;     // -100..100
  int      steering;     // -100..100
  uint16_t throttleRaw;
  uint16_t steeringRaw;
  uint8_t  sequence;
};
extern Remote remote;
size_t buildRemoteReport(uint8_t *buffer);   // 응답 바이트 수 (보드가 죽었으면 0)

// ---- 가상 Pixhawk 조향 PWM ----
extern bool         pixhawkEnabled;
extern unsigned int pixhawkUs;

// ---- 가상 시리얼 ----
void pushSerialLine(const std::string &line);   // 끝에 '\n' 을 붙여 넣는다
int  serialAvailable();
int  serialRead();
void serialWrite(const std::string &text);
void serialWriteLong(long value);
void serialWriteULong(unsigned long value);
void serialWriteFloat(double value, int digits);   // Arduino Print::printFloat 와 같은 규칙
const std::string &serialOutput();

}  // namespace sim


// ---- 펌웨어 빌드마다 따로 구현하는 훅 ----
void Backend_EncoderEdge(int direction);                         // +1 전진, -1 후진
void Backend_PixhawkPulse(unsigned long riseUs, unsigned long fallUs);

// ---- 펌웨어 진입점 ----
void setup();
void loop();

#endif
