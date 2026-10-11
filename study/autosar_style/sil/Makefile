# SIL 빌드 · 실행 · 그래프 · 동등성 테스트
#
#   make          두 실행 파일 빌드
#   make run      시나리오 3개를 계층 버전으로 돌려 out/*.csv 저장
#   make plot     out/*.csv -> images/*.png
#   make test     원본과 계층 버전이 같은 출력을 내는지 비교

CXX      ?= g++
# 두 빌드가 부동소수 연산을 같은 순서로 하도록 축약(FMA)을 끈다.
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -ffp-contract=off
PYTHON   ?= /usr/bin/python3

BUILD    ?= build
LAYERED  ?= ../t870_drive_layered
ORIGINAL ?= ../../../sketch_aug

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

run: $(BUILD)/sil_layered
	@mkdir -p out
	for s in speed_step steering watchdog; do \
		$(BUILD)/sil_layered $$s out/$$s.csv out/$$s.serial.txt || exit 1; \
	done

plot: run
	$(PYTHON) plot_sil.py out images

test: all
	$(PYTHON) -m pytest -q test_equivalence.py

clean:
	rm -rf $(BUILD) out

.PHONY: all run plot test clean
