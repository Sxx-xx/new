// ============================================================
// CDD SerialCmd: USB 시리얼 명령 파싱 (115200bps, 줄 끝 = Newline)
//
//   1.5    목표 속도 1.5 km/h (전진)
//  -1.0    목표 속도 1.0 km/h (후진)
//   0      정지
//   e      비상정지 (램프 · PI 우회)
//   h      미션 홀드 (경사 밀림 방지 정차)
//   p90    구동 개루프 duty 90
//   s60    조향 개루프. POT 값이 커지는 방향으로 duty 60
//   s-60   조향 개루프. POT 값이 작아지는 방향으로 duty 60
//   s0     조향 개루프 해제
//   b      명령 워치독 on/off 토글 (벤치 테스트용)
//   r      폴트/스톨 래치 해제
//
// 이 드라이버는 문자열을 명령으로 바꾸기만 한다. 명령이 무엇을
// 하는지는 Rte_Notify_SerialCommand() 가 부르는 SWC 가 정한다.
// ============================================================

#ifndef CDD_SERIAL_CMD_H
#define CDD_SERIAL_CMD_H

#include "Std_Types.h"

void Cdd_SerialCmd_MainFunction();

#endif
