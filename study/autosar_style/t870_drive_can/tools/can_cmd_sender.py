#!/usr/bin/env python3
"""노트북 쪽 CAN 명령 송신기.

시나리오 파일을 읽어 DriveCmd(0x100)를 20 ms 마다 보내고, DriveStatus(0x200) ·
DriveDiag(0x201)를 받아 CSV 로 남긴다. 고장을 일부러 넣을 수도 있다.

    # 실시간 전송 (vcan0 또는 CANable 의 can0)
    can_cmd_sender.py scenarios/speed_step.txt --bus vcan0 --record status.csv

    # 프레임 목록만 파일로 (sil_can --bus=mem --frames=<파일> 입력)
    can_cmd_sender.py scenarios/speed_step.txt --dump frames.txt

    # 고장 주입: crc | alive_freeze | alive_jump | stop | dlc
    can_cmd_sender.py scenarios/speed_step.txt --bus vcan0 --fault crc --fault-at 5000 --fault-frames 5

시나리오 한 줄은 `<t_ms> 신호=값 ...` 이고 값은 다음 변경까지 유지된다.
    mode=MANUAL|AUTO|HOLD|ESTOP  target=<km/h>  pot=<count>
    sel=NONE|DRIVE|STEER  duty=<-255..255>  wd=0|1  clear=0|1  stop=1
"""

import argparse
import csv
import pathlib
import sys
import time
from dataclasses import dataclass

import cantools

DBC_PATH = pathlib.Path(__file__).resolve().parent.parent / 'can' / 't870.dbc'
FAULT_KINDS = ('crc', 'alive_freeze', 'alive_jump', 'stop', 'dlc')
SHORT_DLC = 7


def load_database():
    return cantools.database.load_file(DBC_PATH)


def crc8(data, data_id):
    """SAE J1850: 다항식 0x1D, 초기값 0xFF, 최종 XOR 0xFF. 입력 끝에 DataID 를 붙인다."""
    crc = 0xFF
    for value in bytes(data) + bytes([data_id]):
        crc ^= value
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1D) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc ^ 0xFF


def data_id(message):
    return message.dbc.attributes['E2E_DataID'].value


def cycle_ms(message):
    return message.dbc.attributes['GenMsgCycleTime'].value


@dataclass
class Fault:
    kind: str
    at_ms: int
    frames: int | None = None     # None 이면 끝까지


@dataclass
class Scenario:
    duration_ms: int
    changes: list                 # [(t_ms, {신호: 문자열 값}), ...] 시각 순


def parse_scenario(path):
    duration_ms = None
    changes = []
    for number, raw in enumerate(pathlib.Path(path).read_text().splitlines(), start=1):
        line = raw.split('#', 1)[0].strip()
        if not line:
            continue
        head, *fields = line.split()
        if head == 'duration':
            duration_ms = int(fields[0])
            continue
        try:
            changes.append((int(head), dict(field.split('=', 1) for field in fields)))
        except ValueError as error:
            raise SystemExit(f'{path}:{number}: {error}')
    if duration_ms is None:
        raise SystemExit(f'{path}: duration 줄이 없다')
    return Scenario(duration_ms, sorted(changes, key=lambda change: change[0]))


def signals_from_state(state):
    """시나리오 상태를 DBC 신호 raw 값으로 바꾼다. 범위를 넘으면 cantools 가 오류를 낸다."""
    return {
        'TargetSpeed_kmh': float(state['target']),
        'SteerTargetPot': int(state['pot']),
        'Mode': state['mode'],
        'OpenLoopSel': state['sel'],
        'OpenLoopDuty': int(state['duty']),
        'WatchdogDisable': int(state['wd']),
        'LatchClearReq': int(state['clear']),
    }


def build_frames(scenario, database, fault=None):
    """(t_ms, id, bytes) 목록을 만든다. 실시간 전송과 인메모리 버스가 같은 목록을 쓴다."""
    message = database.get_message_by_name('DriveCmd')
    period = cycle_ms(message)
    identifier = data_id(message)

    state = {'mode': 'AUTO', 'target': '0', 'pot': '543', 'sel': 'NONE',
             'duty': '0', 'wd': '0', 'clear': '0', 'stop': '0'}
    changes = list(scenario.changes)

    frames = []
    alive = 0
    fault_frames_sent = 0
    for t in range(0, scenario.duration_ms + 1, period):
        while changes and changes[0][0] <= t:
            state.update(changes.pop(0)[1])
        if state['stop'] == '1':
            break

        active = (fault is not None and t >= fault.at_ms
                  and (fault.frames is None or fault_frames_sent < fault.frames))
        kind = fault.kind if active else None
        if active:
            fault_frames_sent += 1
        if kind == 'stop':
            continue

        if kind == 'alive_jump':
            alive = (alive + 2) & 0x0F
        elif kind != 'alive_freeze' and frames:
            alive = (alive + 1) & 0x0F

        signals = signals_from_state(state)
        signals.update(Alive=alive, CRC8=0)
        data = bytearray(message.encode(signals, strict=True))
        data[7] = crc8(data[:7], identifier)

        if kind == 'crc':
            data[7] ^= 0xA5
        if kind == 'dlc':
            data = data[:SHORT_DLC]
        frames.append((t, message.frame_id, bytes(data)))
    return frames


def write_frames(frames, path):
    with open(path, 'w') as file:
        for t, frame_id, data in frames:
            file.write(f'{t} {frame_id:03X} {len(data)} {data.hex().upper()}\n')


class StatusRecorder:
    """받은 상태 프레임을 CRC 검사 결과와 함께 CSV 로 남긴다."""

    def __init__(self, database, path):
        self.database = database
        self.messages = {m.frame_id: m for m in database.messages if m.name != 'DriveCmd'}
        columns = ['t_ms', 'name', 'crc_ok']
        for message in self.messages.values():
            columns += [s.name for s in message.signals if s.name not in columns]
        self.file = open(path, 'w', newline='') if path else None
        self.writer = csv.DictWriter(self.file, fieldnames=columns, restval='') if path else None
        if self.writer:
            self.writer.writeheader()

    def add(self, t_ms, frame_id, data):
        message = self.messages.get(frame_id)
        if message is None or len(data) != 8:
            return
        decoded = message.decode(bytes(data), decode_choices=False)
        crc_ok = crc8(data[:7], data_id(message)) == data[7]
        if self.writer:
            self.writer.writerow({'t_ms': round(t_ms, 1), 'name': message.name,
                                  'crc_ok': int(crc_ok), **decoded})

    def close(self):
        if self.file:
            self.file.close()


def send_live(frames, duration_ms, channel, recorder):
    """절대 시각 기준으로 보내 지연이 쌓이지 않게 한다. 보낸 간격(ms) 목록을 돌려준다."""
    import can

    receive_ids = [{'can_id': frame_id, 'can_mask': 0x7FF} for frame_id in recorder.messages]
    sent_at = []
    with can.Bus(channel=channel, interface='socketcan', can_filters=receive_ids) as bus:
        start = time.perf_counter()

        def elapsed_ms():
            return (time.perf_counter() - start) * 1000.0

        def receive_until(deadline_ms):
            while True:
                remaining = (deadline_ms - elapsed_ms()) / 1000.0
                if remaining <= 0:
                    return
                message = bus.recv(timeout=remaining)
                if message is not None:
                    recorder.add(elapsed_ms(), message.arbitration_id, message.data)

        for t, frame_id, data in frames:
            receive_until(t)
            bus.send(can.Message(arbitration_id=frame_id, data=data, is_extended_id=False))
            sent_at.append(elapsed_ms())
        receive_until(duration_ms)

    return [b - a for a, b in zip(sent_at, sent_at[1:])]


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('scenario')
    parser.add_argument('--bus', help='SocketCAN 인터페이스 이름 (vcan0, can0)')
    parser.add_argument('--dump', help='실시간 전송 대신 프레임 목록을 이 파일에 쓴다')
    parser.add_argument('--record', help='받은 상태 프레임을 이 CSV 에 쓴다')
    parser.add_argument('--fault', choices=FAULT_KINDS)
    parser.add_argument('--fault-at', type=int, default=0, help='고장 시작 시각 (ms)')
    parser.add_argument('--fault-frames', type=int, help='고장 프레임 수. 없으면 끝까지')
    args = parser.parse_args()

    if bool(args.bus) == bool(args.dump):
        parser.error('--bus 와 --dump 중 하나를 고른다')

    database = load_database()
    scenario = parse_scenario(args.scenario)
    fault = Fault(args.fault, args.fault_at, args.fault_frames) if args.fault else None
    frames = build_frames(scenario, database, fault)

    if args.dump:
        write_frames(frames, args.dump)
        return 0

    recorder = StatusRecorder(database, args.record)
    try:
        intervals = send_live(frames, scenario.duration_ms, args.bus, recorder)
    finally:
        recorder.close()
    if intervals:
        print(f'frames={len(frames)} interval_ms min={min(intervals):.2f} '
              f'max={max(intervals):.2f}', file=sys.stderr)
    return 0


if __name__ == '__main__':
    sys.exit(main())
