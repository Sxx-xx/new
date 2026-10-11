# t870_drive_layered

> **실차에 올리지 않은 학습용 구조 연습이다.**
> 실제 차량에서 돈 펌웨어는 [`sketch_aug/sketch_aug.ino`](../../../sketch_aug/sketch_aug.ino)이고, 이 폴더는 그것을 AUTOSAR Classic 계층 구조로 나눠 본 것이다.
> 제어 알고리즘, 상수, 동작은 원본과 같게 두고 구조만 바꿨다.

## 계층

```
t870_drive_layered.ino      OS 자리. 초기화 순서와 주기 태스크 호출만
│
├── Swc_SpeedControl        명령 선택, 속도 측정, 램프, 방향 반전 보호, PI + FF
├── Swc_SteeringControl     포텐셔미터 조향 위치 제어, 소프트 리밋, 폴트 판정
├── Swc_StopHold            MANUAL/AUTO 능동 제동, 미션 홀드 토크
│        │  Rte_Read / Rte_Write / Rte_Call 만 호출
├── Rte                     SWC 사이 신호 버퍼, 하위 계층 연결, 명령 이벤트 분배
│        │
├── Cdd_RemoteRx            I2C 수신기 보고서 검사 (헤더 · 체크섬 · 순번 · 타임아웃)
├── Cdd_SerialCmd           시리얼 줄 버퍼, 명령 파싱
├── Cdd_Telemetry           시작 배너, 200 ms 상태 줄
├── Bsw_Watchdog            명령 워치독
├── Bsw_FaultLatch          조향 폴트 / 스톨 래치
│        │
└── Mcal_Port / Pwm / Adc / Icu / Gpt / I2c / Uart, Mcal_Cfg.h
                            Arduino API와 레지스터는 여기서만 호출
```

`Std_Types.h`, `Lib_Math.h`는 모든 계층이 쓰는 공통 타입과 수학 함수다. `Lib_Math.h`는 Arduino의 `map` · `constrain` · `abs` · `min` · `max`를 AVR 코어 1.8.x와 같은 식으로 다시 정의한 것이다. SWC가 Arduino 헤더 없이도 원본과 같은 값을 계산하게 하려는 것이다.

100 ms 구동 태스크는 `MeasureSpeed → StopHold_Run → SpeedControl_Run` 순서로 돈다. 이 순서는 원본 `driveControl()` 안의 순서와 같다. StopHold가 이번 주기 출력을 정하면 `StopHoldOverride` 신호를 세우고, SpeedControl은 PI를 건너뛴다.

## 원본 함수 → 새 계층

| 원본 (`sketch_aug.ino`) | 새 위치 |
| :--- | :--- |
| `setup()` | `t870_drive_layered.ino` `setup()` → `Mcal_Port_Init`, `Mcal_I2c_Init`, `Mcal_Icu_InitPixhawkCapture`, `Mcal_Pwm_Init`, `Cdd_Telemetry_PrintBanner` |
| `loop()` | `t870_drive_layered.ino` `loop()` |
| `encoderISR()` | `Mcal_Icu.cpp` `encoderISR()` |
| `ISR(PCINT0_vect)` | `Mcal_Icu.cpp` |
| `readPixhawkPwm()` | `Mcal_Icu_ReadPixhawkPwm()` |
| `pixhawkPwmToTargetPot()` | `Swc_SteeringControl.cpp` 내부 함수 |
| `remoteChecksum()`, `readRemoteReport()` | `Cdd_RemoteRx_MainFunction()` |
| `emergencyStopDrive()` | `Swc_SpeedControl.cpp` 내부 함수. 미션 홀드 해제는 `Rte_Call_StopHold_Release()` |
| `selectControlCommand()` | `Swc_SpeedControl_SelectCommand()` |
| `loop()`의 명령 워치독 블록 | `Bsw_Watchdog_MainFunction()` → `Rte_Notify_CommandTimeout()` → 각 SWC의 `OnCommandTimeout()` |
| `readSerialCommand()` | `Cdd_SerialCmd_MainFunction()` |
| `handleCommand()` | 파싱은 `Cdd_SerialCmd.cpp` `handleLine()`, 분배는 `Rte_Notify_SerialCommand()`. 처리 함수는 아래 표 참고 |
| `steeringControl()` | `Swc_SteeringControl_Run()` |
| `driveControl()` 속도 측정 | `Swc_SpeedControl_MeasureSpeed()` |
| `driveControl()` MANUAL 제동 · 미션 홀드 · AUTO 제동 | `Swc_StopHold_Run()` |
| `driveControl()` 개루프 · 램프 · 반전 보호 · PI | `Swc_SpeedControl_Run()` |
| `printStatus()` | `Cdd_Telemetry_MainFunction()` |
| 핀 번호, DIR 극성 상수 | `Mcal_Cfg.h` |

`handleCommand()`의 명령별 처리 함수는 다음과 같다.

| 명령 | 처리 |
| :--- | :--- |
| `e` | `Swc_SpeedControl_OnEmergencyStopCmd()` |
| `h` | `Swc_StopHold_OnHoldCmd()` |
| `b` | `Bsw_Watchdog_Toggle()` |
| `r` | `Bsw_FaultLatch_ClearAll()` |
| `s<duty>` | `Swc_SteeringControl_OnOpenLoopCmd()` |
| `p<duty>` | `Swc_SpeedControl_OnOpenLoopCmd()` |
| `<km/h>` | `Swc_SpeedControl_OnTargetSpeedCmd()` |

## 칩이 바뀌면 Mcal만, 기능을 옮기면 Swc만

- **칩이 바뀔 때**: `pinMode` · `analogWrite` · `TCCR1B` · `PCICR` · `Wire` · `Serial` 같은 Arduino API와 레지스터 접근은 모두 `Mcal_*.cpp`에 있다. 핀 번호와 DIR 극성은 `Mcal_Cfg.h`에 있다. Mcal 위 계층은 `Mcal_Pwm_SetDriveDuty(duty)`나 `Mcal_Port_SetDriveDirection(forward)`처럼 "무엇을 할지"만 말한다. 예를 들어 STM32로 옮기면 Timer1 설정, 핀 체인지 인터럽트, ADC 읽기를 STM32 방식으로 다시 쓰는 일은 Mcal 파일 안에서 끝난다. 모터 드라이버의 DIR 극성이 반대인 보드로 바꿀 때도 `Mcal_Cfg.h` 한 줄만 고치면 된다.
- **기능을 옮길 때**: SWC는 `Rte.h` 하나만 include하고, 다른 SWC나 하드웨어에는 RTE 포트로만 닿는다. 그래서 정지 · 홀드 판단을 바꾸거나 다른 SWC로 옮기는 작업은 해당 `Swc_*.cpp`와 `Rte.cpp`의 연결만 고치면 된다. 다른 SWC와 하위 계층은 건드리지 않는다.

## 실제 AUTOSAR와 다른 점

- **RTE 자동 생성이 없다.** 실제로는 ARXML로 포트와 연결을 정의하고 RTE 생성기가 `Rte.c`를 만든다. 여기서는 손으로 썼다.
- **OS가 없다.** 태스크, 우선순위, 선점, 이벤트 기반 runnable 트리거가 없다. `.ino`의 `loop()`가 하나의 슈퍼루프로 OS를 흉내 낸다.
- **COM 스택이 없다.** I2C 보고서와 시리얼 명령은 COM / PduR / 신호 매핑 없이 CDD가 직접 파싱한다. 그래서 SWC를 다른 ECU로 옮기는 일은 이 구조로는 할 수 없다.
- **MCAL이 표준 API가 아니다.** `Dio_WriteChannel`, `Pwm_SetDutyCycle`, `Adc_ReadGroup` 같은 AUTOSAR MCAL 규격 대신 Arduino 함수를 감싼 함수다. ECU 추상화 계층(IoHwAb)도 따로 두지 않았다.
- **진단 · 모드 관리 BSW가 없다.** DEM / DET / EcuM / BswM이 없고, 폴트 래치는 bool 두 개짜리 모듈이다.
- **보정값이 상수다.** `Rte_Prm_*`는 컴파일 타임 상수를 돌려줄 뿐이다. 측정 · 보정 도구(XCP)로 바꿀 수 없다. 두 구동 SWC가 같이 쓰는 상수는 파라미터 SWC 대신 `Swc_DriveParams.h` 헤더로 공유한다.
- **언어와 메모리 배치가 다르다.** C 대신 Arduino C++을 쓰고, MemMap / Compiler 추상화 헤더가 없다.
- **경계를 지키는 데 예외가 있다.** `Cdd_Telemetry`는 상태 줄을 만들려고 `Rte_Read_*`로 SWC 신호를 읽는다. `.ino`는 시간을 얻으려고 `Mcal_Gpt_GetMs()`를 직접 부른다. 실제 구조라면 각각 진단 SWC와 OS 카운터가 맡을 일이다.

## 원본과의 차이

동작은 같게 두었다. 다만 다음은 다르다.

- 원본의 `requestedDrivePWM`은 `drivePWMValue`로 복사되는 것 말고는 읽히지 않아서 없앴다. 출력 duty는 `Rte_Call_DriveMotor_SetPwm()`이 기록하는 `DrivePwm` 하나로 남겼다.
- 함수 호출이 늘어 한 루프의 실행 시간이 조금 길다. 제어 주기(100 ms)나 상태 출력(200 ms) 판정에는 영향이 없는 수준이지만, 실차에서 측정하지는 않았다.
- 시리얼 출력 문자열은 원본과 글자 하나까지 같다. `arduino_check.py`가 이 상태 줄을 그대로 파싱할 수 있다.

## 빌드 결과

`arduino-cli compile -b arduino:avr:uno` (arduino:avr 1.8.8), `--warnings all`에서 경고 없음.

| | 플래시 | RAM (전역) |
| :--- | --: | --: |
| 원본 `sketch_aug` | 13,822 B (42%) | 1,062 B (51%) |
| `t870_drive_layered` | 13,978 B (43%) | 1,061 B (51%) |
| 차이 | +156 B | −1 B |

플래시 증가분은 RTE 접근 함수와 분리된 처리 함수의 호출 오버헤드다. RAM은 원본 전역 변수가 RTE 버퍼와 각 모듈의 static 변수로 자리만 옮겨서 거의 같다.
