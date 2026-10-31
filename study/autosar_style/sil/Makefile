# SIL 빌드 · 실행 · 그래프 · 동등성 테스트
#
#   make          두 실행 파일 빌드
#   make run      시나리오 3개를 계층 버전으로 돌려 out/*.csv 저장
#   make plot     out/*.csv -> images/*.png
#   make test     원본과 계층 버전이 같은 출력을 내는지 비교
#   make can      CAN 버전 실행 파일 sil_can, com_roundtrip 빌드
#   make test-can CAN 버전 테스트 (python-can, cantools 필요. vcan0 이 없으면 그 테스트만 건너뛴다)
#   make plot-can CAN 고장 주입 그래프

CXX      ?= g++
# 두 빌드가 부동소수 연산을 같은 순서로 하도록 축약(FMA)을 끈다.
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -ffp-contract=off
PYTHON   ?= /usr/bin/python3

BUILD    ?= build
LAYERED  ?= ../t870_drive_layered
ORIGINAL ?= ../../../sketch_aug
CAN      ?= ../t870_drive_can

LAYERED_SRCS := \
	$(LAYERED)/Rte.cpp \
	$(LAYERED)/Swc_SpeedControl.cpp \
	$(LAYERED)/Swc_SteeringControl.cpp \
	$(LAYERED)/Swc_StopHold.cpp \
	$(LAYERED)/Cdd_RemoteRx.cpp \
	$(LAYERED)/Cdd_SerialCmd.cpp \
	$(LAYERED)/Cdd_Telemetry.cpp \
	$(LAYERED)/Bsw_Watchdog.cpp \
	$(LAYERED)/Bsw_FaultLatch.cpp

CAN_SRCS := \
	$(CAN)/Rte.cpp \
	$(CAN)/Swc_SpeedControl.cpp \
	$(CAN)/Swc_SteeringControl.cpp \
	$(CAN)/Swc_StopHold.cpp \
	$(CAN)/Cdd_RemoteRx.cpp \
	$(CAN)/Cdd_CanGateway.cpp \
	$(CAN)/Cdd_Telemetry.cpp \
	$(CAN)/Bsw_Watchdog.cpp \
	$(CAN)/Bsw_FaultLatch.cpp \
	$(CAN)/CanIf.cpp \
	$(CAN)/Com.cpp \
	$(CAN)/E2E.cpp

CORE_SRCS := sim_core.cpp sim_main.cpp

all: $(BUILD)/sil_layered $(BUILD)/sil_original

$(BUILD)/sil_layered: $(CORE_SRCS) Mcal_Sim.cpp $(LAYERED_SRCS) $(LAYERED)/t870_drive_layered.ino $(wildcard $(LAYERED)/*.h) sim_core.h
	@mkdir -p $(BUILD)
	$(CXX) $(CXXFLAGS) -I. -I$(LAYERED) -o $@ \
		$(CORE_SRCS) Mcal_Sim.cpp $(LAYERED_SRCS) -x c++ $(LAYERED)/t870_drive_layered.ino

$(BUILD)/sil_original: $(CORE_SRCS) shim/arduino_shim.cpp shim/Arduino.h shim/Wire.h $(ORIGINAL)/sketch_aug.ino sim_core.h
	@mkdir -p $(BUILD)
	$(CXX) $(CXXFLAGS) -Wno-unused-parameter -I. -Ishim -o $@ \
		$(CORE_SRCS) shim/arduino_shim.cpp -x c++ $(ORIGINAL)/sketch_aug.ino

can: $(BUILD)/sil_can $(BUILD)/com_roundtrip

$(BUILD)/sil_can: $(CORE_SRCS) Mcal_Sim.cpp Mcal_Can_Sim.cpp Mcal_Can_Sim.h $(CAN_SRCS) $(CAN)/t870_drive_can.ino $(wildcard $(CAN)/*.h) sim_core.h
	@mkdir -p $(BUILD)
	$(CXX) $(CXXFLAGS) -DSIL_CAN -I. -I$(CAN) -o $@ \
		$(CORE_SRCS) Mcal_Sim.cpp Mcal_Can_Sim.cpp $(CAN_SRCS) -x c++ $(CAN)/t870_drive_can.ino

$(BUILD)/com_roundtrip: com_roundtrip.cpp $(CAN)/E2E.cpp $(CAN)/E2E.h $(CAN)/Com_Cfg.h
	@mkdir -p $(BUILD)
	$(CXX) $(CXXFLAGS) -I$(CAN) -o $@ com_roundtrip.cpp $(CAN)/E2E.cpp

run: $(BUILD)/sil_layered
	@mkdir -p out
	for s in speed_step steering watchdog; do \
		$(BUILD)/sil_layered $$s out/$$s.csv out/$$s.serial.txt || exit 1; \
	done

plot: run
	$(PYTHON) plot_sil.py out images

test: all
	$(PYTHON) -m pytest -q test_equivalence.py

test-can: all can
	$(PYTHON) -m pytest -q test_can.py

plot-can: $(BUILD)/sil_can $(BUILD)/sil_layered
	$(PYTHON) plot_sil.py --can images

clean:
	rm -rf $(BUILD) out

.PHONY: all can run plot test test-can plot-can clean
