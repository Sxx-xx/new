// ============================================================
// sil_can 전용: PC 쪽 CAN 버스 설정과 시간 맞추기
//
//   --bus=mem      인메모리 버스. --frames 파일의 프레임을 가상 시각에 넣는다.
//                  가상 시간으로 최대한 빨리 돌고 결과가 매번 같다.
//   --bus=<ifname> SocketCAN (예: vcan0). 첫 프레임을 받으면 시작해 실시간으로 돈다.
//   --tx-log=<경로> 펌웨어가 보낸 프레임 기록 (버스 종류와 무관)
//   --duration-ms=<ms> 시나리오 길이 덮어쓰기
// ============================================================

#ifndef SIL_MCAL_CAN_SIM_H
#define SIL_MCAL_CAN_SIM_H

bool SimCan_ParseArgs(int argc, char **argv);    // 실패하면 false
unsigned long SimCan_Duration(unsigned long scenarioMs);
bool SimCan_WaitForStart();                      // SocketCAN 은 첫 프레임까지 기다린다
void SimCan_BeforeLoop(unsigned long t);         // 해당 시각의 프레임을 넣고 실시간을 맞춘다
void SimCan_Close();

#endif
