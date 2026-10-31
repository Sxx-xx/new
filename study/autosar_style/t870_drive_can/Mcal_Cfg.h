// ============================================================
// MCAL 설정. 핀 배치와 배선 극성은 이 파일에만 있다.
//
// D2  : 구동 엔코더 A상 (INT0)
// D3  : 구동 엔코더 B상
// D4  : 구동 DIR           (layered 의 D11 에서 옮김. D11 은 SPI MOSI)
// D5  : MCP2515 INT        (폴링으로 읽는다. D2/D3 외부 인터럽트는 엔코더가 쓴다)
// D6  : MCP2515 CS
// D7  : 조향 DIR
// D8  : Pixhawk PWM 입력   (PCINT0)
// D9  : 조향 PWM           (Timer1)
// D10 : 구동 PWM           (Timer1). 하드웨어 SS 핀이기도 하다. 출력이므로
//                           SPI 마스터 모드가 유지된다.
// D11 : MCP2515 SI  (SPI MOSI)
// D12 : MCP2515 SO  (SPI MISO). 원본 주석의 모드 스위치는 미구현이라 코드에서 안 쓴다.
// D13 : MCP2515 SCK (SPI SCK)
// A0  : 조향 포텐셔미터
// A4/A5 : I2C (보조 RC 수신기 보드)
// ============================================================

#ifndef MCAL_CFG_H
#define MCAL_CFG_H

#include "Std_Types.h"

#define MCAL_PIN_ENCODER_A   2
#define MCAL_PIN_ENCODER_B   3
#define MCAL_PIN_STEER_DIR   7
#define MCAL_PIN_PIXHAWK_PWM 8
#define MCAL_PIN_STEER_PWM   9
#define MCAL_PIN_DRIVE_PWM   10
#define MCAL_PIN_DRIVE_DIR   4
#define MCAL_PIN_CAN_INT     5
#define MCAL_PIN_CAN_CS      6
#define MCAL_PIN_STEER_POT   A0

// MCP2515 모듈의 크리스털. 모듈 위 캔에 적힌 값(8.000 또는 16.000)에 맞춘다.
// 틀리면 비트레이트가 두 배로 어긋나 통신이 전혀 안 된다.
#define MCAL_CAN_CRYSTAL_MHZ 8

// 조향 DIR. Cytron PWM/DIR 모드에서 LOW = MA 방향, HIGH = MB 방향.
// [2026-08-26 실차 확인]
//   우회전 -> POT 값 증가, MA 방향 (LOW)
//   좌회전 -> POT 값 감소, MB 방향 (HIGH)
#define MCAL_STEER_DIR_POT_UP_LEVEL    LOW
#define MCAL_STEER_DIR_POT_DOWN_LEVEL  HIGH

// 구동 DIR. [2026-08-27 실차 명령 시험] HIGH=전진, LOW=후진.
#define MCAL_DRIVE_DIR_FORWARD_LEVEL   HIGH
#define MCAL_DRIVE_DIR_REVERSE_LEVEL   LOW

#endif
