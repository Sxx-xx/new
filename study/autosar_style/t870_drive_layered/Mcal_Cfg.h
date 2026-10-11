// ============================================================
// MCAL 설정. 핀 배치와 배선 극성은 이 파일에만 있다.
//
// D2  : 구동 엔코더 A상 (INT0)
// D3  : 구동 엔코더 B상
// D7  : 조향 DIR
// D8  : Pixhawk PWM 입력   (PCINT0)
// D9  : 조향 PWM           (Timer1)
// D10 : 구동 PWM           (Timer1)
// D11 : 구동 DIR
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
#define MCAL_PIN_DRIVE_DIR   11
#define MCAL_PIN_STEER_POT   A0

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
