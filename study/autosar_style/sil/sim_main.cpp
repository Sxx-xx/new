// ============================================================
// SIL 시나리오 실행기
//
//   sil_xxx <scenario> <out.csv> <out_serial.txt>
//   sil_can <scenario> <out.csv> <out_serial.txt> [--bus=...] (Mcal_Can_Sim.h 참고)
//
// sil_can 은 시리얼 입력을 읽지 않는다. 아래 시나리오가 넣는 시리얼 줄은 쌓이기만 하고,
// 명령은 CAN 으로 온다. RC · Pixhawk · 경사 입력과 CSV 형식은 다른 빌드와 같다.
//
// 시나리오
//   speed_step : AUTO, ROS 목표 0 -> 1.5 km/h 계단
//   steering   : AUTO, Pixhawk 조향 1100(좌) -> 1500(중앙) -> 1900(우)
//   watchdog   : AUTO, 1.5 km/h 주행 중 5 s 에 ROS 명령이 끊김
//   mixed      : 동등성 테스트용. 경사, 능동 제동, 미션 홀드, E-stop, 개루프,
//                MANUAL 반전 제동, 수신기 단절, 래치/워치독 명령을 한 번씩 거친다
// ============================================================

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <string>

#include "sim_core.h"

#ifdef SIL_CAN
#include "Mcal_Can_Sim.h"
#endif


struct Scenario
{
  const char *name;
  unsigned long durationMs;
  double grade;
  void (*events)(unsigned long t);
};


// ROS 브리지 흉내. 100 ms 마다 현재 목표를 보낸다.
static bool        rosAlive = false;
static double      rosTargetKmh = 0.0;
static bool        rosHold = false;
static double      steerTargetRef = NAN;   // 그래프용 기준선 (펌웨어 입력이 아님)

static void rosTick(unsigned long t)
{
  if (!rosAlive || t % 100 != 0)
  {
    return;
  }
  char line[32];
  snprintf(line, sizeof(line), "%.3f", rosTargetKmh);
  sim::pushSerialLine(line);
  if (rosHold)
  {
    sim::pushSerialLine("h");
  }
}


static void setAuto()
{
  sim::remote.boardAlive = true;
  sim::remote.connected = true;
  sim::remote.autoMode = true;
  sim::remote.throttle = 0;
  sim::remote.steering = 0;
}


static void setManual(int throttle, int steering)
{
  sim::remote.boardAlive = true;
  sim::remote.connected = true;
  sim::remote.autoMode = false;
  sim::remote.throttle = throttle;
  sim::remote.steering = steering;
  sim::remote.throttleRaw = (uint16_t)(1500 + throttle * 5);
  sim::remote.steeringRaw = (uint16_t)(1500 - steering * 5);
}


// 펌웨어의 Pixhawk -> POT 목표 매핑을 손으로 계산한 기준선 (1100:47, 1500:543, 1900:1005)
static double pixhawkToPotRef(unsigned int us)
{
  if (us <= 1500) return 47.0 + (us - 1100.0) * (543.0 - 47.0) / 400.0;
  return 543.0 + (us - 1500.0) * (1005.0 - 543.0) / 400.0;
}


// ---- (a) 속도 계단 ----
static void eventsSpeedStep(unsigned long t)
{
  if (t == 1)
  {
    setAuto();
    sim::pixhawkUs = 1500;
    rosAlive = true;
    rosTargetKmh = 0.0;
  }
  if (t == 1000) rosTargetKmh = 1.5;
  rosTick(t);
}


// ---- (b) 조향 좌 -> 중앙 -> 우 ----
static void eventsSteering(unsigned long t)
{
  if (t == 1)
  {
    setAuto();
    rosAlive = true;
    rosTargetKmh = 0.0;
  }
  if (t < 3000)      sim::pixhawkUs = 1100;
  else if (t < 6000) sim::pixhawkUs = 1500;
  else               sim::pixhawkUs = 1900;
  steerTargetRef = pixhawkToPotRef(sim::pixhawkUs);
  rosTick(t);
}


// ---- (c) ROS 명령 끊김 -> 워치독 정지 ----
static void eventsWatchdog(unsigned long t)
{
  if (t == 1)
  {
    setAuto();
    sim::pixhawkUs = 1500;
    rosAlive = true;
    rosTargetKmh = 1.5;
  }
  if (t == 5000) rosAlive = false;
  rosTick(t);
}


// ---- (d) 동등성 테스트용 혼합 시나리오 ----
static void eventsMixed(unsigned long t)
{
  if (t == 1)
  {
    setAuto();
    sim::pixhawkUs = 1500;
    rosAlive = true;
    rosTargetKmh = 2.0;
  }
  if (t == 3000)  sim::pixhawkUs = 1300;
  if (t == 5000)  sim::pixhawkUs = 1700;
  if (t == 6000)  rosTargetKmh = -1.0;          // AUTO 반대 명령 능동 제동 후 후진
  if (t == 9000)  { rosTargetKmh = 0.0; rosHold = true; sim::grade = 0.10; }   // 10% 경사에서 미션 홀드
  if (t == 9500)  sim::pixhawkEnabled = false;  // Pixhawk 단절 -> 조향 정지
  if (t == 10500) sim::pixhawkEnabled = true;
  if (t == 12000) { rosHold = false; rosAlive = false; sim::grade = 0.0; sim::pushSerialLine("e"); }
  if (t == 12500) sim::pushSerialLine("p40");
  if (t == 13500) sim::pushSerialLine("0");
  if (t == 13600) sim::pushSerialLine("s60");
  if (t == 14000) sim::pushSerialLine("s-60");
  if (t == 14300) sim::pushSerialLine("s0");
  if (t == 14400) { sim::pushSerialLine("b"); sim::pushSerialLine("xyz"); }
  if (t == 14500) { sim::pushSerialLine("r"); sim::pushSerialLine("b"); }
  if (t == 15000) setManual(60, -60);           // MANUAL 진입. 스로틀이 당겨져 있어 출발 안 함
  if (t == 15500) setManual(0, -60);            // 중립 확인
  if (t == 16000) setManual(60, -60);
  if (t == 18000) setManual(-60, 60);           // 반대 스로틀 -> MANUAL 능동 제동
  if (t == 19000) setManual(0, 60);
  if (t == 19500) sim::remote.connected = false;   // 수신기 단절
  if (t == 20500) sim::remote.boardAlive = false;  // 보조 보드 무응답
  if (t == 21000) setManual(40, 0);             // 복귀. 중립 확인 전에는 정지
  if (t == 21500) setManual(0, 0);
  if (t == 22000) setManual(30, 20);
  if (t == 23000) { setAuto(); rosAlive = true; rosTargetKmh = 3.0; }
  if (t == 29000) { rosTargetKmh = 0.0; rosHold = true; }   // 주행 중 홀드 -> 끊어 거는 제동 구간
  rosTick(t);
}


static const Scenario SCENARIOS[] = {
  {"speed_step", 12000, 0.0,  eventsSpeedStep},
  {"steering",    9000, 0.0,  eventsSteering},
  {"watchdog",   12000, 0.0,  eventsWatchdog},
  {"mixed",      32000, 0.0,  eventsMixed},
};


int main(int argc, char **argv)
{
#ifdef SIL_CAN
  if (argc < 4 || !SimCan_ParseArgs(argc - 4, argv + 4))
#else
  if (argc != 4)
#endif
  {
    fprintf(stderr, "usage: %s <speed_step|steering|watchdog|mixed> <out.csv> <out_serial.txt>\n", argv[0]);
    return 2;
  }

  const Scenario *scenario = nullptr;
  for (const Scenario &s : SCENARIOS)
  {
    if (strcmp(s.name, argv[1]) == 0) scenario = &s;
  }
  if (!scenario)
  {
    fprintf(stderr, "unknown scenario: %s\n", argv[1]);
    return 2;
  }

  FILE *csv = fopen(argv[2], "w");
  if (!csv)
  {
    perror(argv[2]);
    return 1;
  }
  fprintf(csv, "t_ms,ros_alive,ros_target_kmh,remote_mode,speed_kmh,drive_pwm,drive_dir,"
               "pixhawk_us,steer_target_ref,pot,steer_pwm,steer_dir\n");

  sim::grade = scenario->grade;
  sim::plantReset(543.0);
  sim::remote.boardAlive = false;   // 시작 직후에는 수신기 보고가 없다
  sim::nowMs = 0;
  setup();

#ifdef SIL_CAN
  if (!SimCan_WaitForStart())
  {
    return 1;
  }
  const unsigned long durationMs = SimCan_Duration(scenario->durationMs);
#else
  const unsigned long durationMs = scenario->durationMs;
#endif

  for (unsigned long t = 1; t <= durationMs; ++t)
  {
#ifdef SIL_CAN
    SimCan_BeforeLoop(t);
#endif
    sim::nowMs = t;
    scenario->events(t);

    // Pixhawk 펄스: 20 ms 마다. 이번 loop 직전에 끝난 펄스로 만든다.
    if (sim::pixhawkEnabled && t % 20 == 0)
    {
      unsigned long fallUs = t * 1000UL - 100;
      Backend_PixhawkPulse(fallUs - sim::pixhawkUs, fallUs);
    }

    loop();
    sim::plantStep(0.001);

    const char *mode = !sim::remote.boardAlive || !sim::remote.connected ? "LOST"
                     : (sim::remote.autoMode ? "AUTO" : "MANUAL");
    fprintf(csv, "%lu,%d,%.3f,%s,%.6f,%d,%d,%u,%.1f,%.4f,%d,%d\n",
            t, rosAlive ? 1 : 0, rosTargetKmh, mode, sim::speedKmh,
            sim::drivePwm, sim::driveDirLevel,
            sim::pixhawkEnabled ? sim::pixhawkUs : 0, steerTargetRef,
            sim::potValue, sim::steerPwm, sim::steerDirLevel);
  }
  fclose(csv);

#ifdef SIL_CAN
  SimCan_Close();
#endif

  FILE *serial = fopen(argv[3], "w");
  if (!serial)
  {
    perror(argv[3]);
    return 1;
  }
  fwrite(sim::serialOutput().data(), 1, sim::serialOutput().size(), serial);
  fclose(serial);
  return 0;
}
