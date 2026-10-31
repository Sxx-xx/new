"""t870_drive_can 테스트: CAN 명령 경로.

세 종류를 본다.

  - pack/unpack: Com_Cfg.h 와 cantools 가 DBC 범위 경계값에서 같은 바이트를 내는지
  - 동등성: 같은 시나리오에서 시리얼 명령 layered 와 CAN 명령 can 의 출력 비교
  - E2E 고장 주입: CRC, Alive 정지, Alive 점프, DLC, 송신 중단 뒤 안전 상태까지 걸린 시간

버스는 둘이다.

  - --bus=mem: 가상 시간. 프레임이 시리얼 줄과 같은 ms 에 도착하므로 오차 0 을 요구한다.
  - --bus=vcan0: 실시간. python-can 송신기와 SocketCAN 으로 주고받는다. vcan0 이 없으면
    건너뛴다. 허용 오차는 LIVE_* 상수 주석 참고.

    /usr/bin/python3 -m pytest -q test_can.py
"""

import filecmp
import importlib.util
import pathlib
import random
import re
import shutil
import subprocess
import sys
import time

import cantools
import numpy as np
import pytest

SIL_DIR = pathlib.Path(__file__).resolve().parent
CAN_DIR = SIL_DIR.parent / 't870_drive_can'
LAYERED_DIR = SIL_DIR.parent / 't870_drive_layered'
SENDER_PATH = CAN_DIR / 'tools' / 'can_cmd_sender.py'
SCENARIO_DIR = CAN_DIR / 'tools' / 'scenarios'
CAN_INTERFACE = 'vcan0'

spec = importlib.util.spec_from_file_location('can_cmd_sender', SENDER_PATH)
sender = importlib.util.module_from_spec(spec)
sys.modules['can_cmd_sender'] = sender
spec.loader.exec_module(sender)

DATABASE = sender.load_database()

# Com.h 의 값. 프레임 주기 20 ms, 연속 실패 3 프레임, 타임아웃 100 ms.
FRAME_MS = 20
FAIL_THRESHOLD = 3
TIMEOUT_MS = 100

# 고장 프레임 시작 ~ 구동 PWM 0 까지 가상 시간에서 나와야 하는 값 (ms).
# 프레임 고장: 고장 프레임 3 개째가 도착한 시각. 첫 프레임이 0 ms 이므로 (3-1) * 20.
FRAME_FAULT_MS = (FAIL_THRESHOLD - 1) * FRAME_MS
# 송신 중단: 마지막 정상 프레임은 시작 시각 20 ms 전이고, 100 ms 를 넘긴 첫 loop 에 판정한다.
STOP_FAULT_MS = TIMEOUT_MS + 1 - FRAME_MS

FAULT_CASES = [
    ('crc', FRAME_FAULT_MS),
    ('alive_freeze', FRAME_FAULT_MS),
    ('alive_jump', FRAME_FAULT_MS),
    ('dlc', FRAME_FAULT_MS),
    ('stop', STOP_FAULT_MS),
]

# 실시간 버스 허용 오차.
#   프레임 고장은 가상 시간 값 + LIVE_FAULT_MARGIN_MS (송신 지터 + 두 프로세스 시작 시각 차이)
#   FaultE2E 비트는 안전 상태 뒤 DriveStatus 한 주기 + 같은 여유 안에 보여야 한다
LIVE_FAULT_MARGIN_MS = 40
STATUS_PERIOD_MS = 100
# 실시간 동등성: 시간축으로 +-LIVE_SHIFT_MS 안의 layered 값 범위 + 허용 duty 안에 들어가야 한다.
#   100 ms 제어 태스크 한 주기 + 20 ms 프레임 주기 + 지터 30 ms
LIVE_SHIFT_MS = 150
LIVE_PWM_TOLERANCE = 2
# 프레임이 제어 태스크보다 1 ms 만 늦어도 명령이 다음 주기로 밀린다. 미션 홀드의 밀림 제어는
# 플랜트와 맞물린 리밋 사이클이라 이 작은 차이가 위상 차이로 남고, 앞선 명령의 지연도 플랜트
# 상태를 거쳐 뒤의 홀드까지 간다. 그래서 홀드 구간 전체에 허용 duty 를 따로 둔다.
# 초과량 실측 최대 38 duty (test_arrival_delay_differs_only_in_hold_windows).
LIVE_HOLD_WINDOWS_MS = {'mixed': [(9000, 12000), (29000, 32000)]}
LIVE_HOLD_PWM_TOLERANCE = 45

MANUAL_DRIVE_T_MS = 16500   # mixed 에서 RC MANUAL 로 달리는 중인 시각
MANUAL_END_T_MS = 23000     # mixed 에서 AUTO 로 돌아오는 시각


def build(build_dir, can_dir=CAN_DIR):
    subprocess.run(
        ['make', '-s', f'BUILD={build_dir}', f'CAN={can_dir}',
         f'{build_dir}/sil_can', f'{build_dir}/com_roundtrip', f'{build_dir}/sil_layered'],
        cwd=SIL_DIR, check=True)


@pytest.fixture(scope='session')
def build_dir(tmp_path_factory):
    path = tmp_path_factory.mktemp('build')
    build(path)
    return path


def read_csv(path):
    with open(path) as file:
        header = file.readline().strip().split(',')
        rows = [line.strip().split(',') for line in file]
    return {name: [row[i] for row in rows] for i, name in enumerate(header)}


def ints(column):
    return np.array([int(value) for value in column])


def status_lines(path):
    return [line for line in path.read_text().splitlines() if line.startswith('RC:')]


class Run:
    """sil_can 한 번 실행한 결과."""

    def __init__(self, csv, serial, tx_log):
        self.csv_path = csv
        self.serial_path = serial
        self.tx_log = tx_log

    @property
    def data(self):
        return read_csv(self.csv_path)

    @property
    def serial(self):
        return self.serial_path.read_text()

    def frames(self, name):
        """[(t_ms, 신호 dict, crc_ok, alive)]"""
        message = DATABASE.get_message_by_name(name)
        result = []
        for line in self.tx_log.read_text().splitlines():
            t, frame_id, _, data = line.split()
            if int(frame_id, 16) != message.frame_id:
                continue
            raw = bytes.fromhex(data)
            crc_ok = sender.crc8(raw[:7], sender.data_id(message)) == raw[7]
            result.append((int(t), message.decode(raw, decode_choices=False), crc_ok))
        return result


def load_scenario(name, duration_ms=None):
    scenario = sender.parse_scenario(SCENARIO_DIR / f'{name}.txt')
    if duration_ms is not None:
        scenario.duration_ms = duration_ms
    return scenario


def run_mem(binary, tmp_path, sil_scenario, scenario, fault=None, tag='run', duration_ms=None):
    frames_path = tmp_path / f'{tag}.frames'
    sender.write_frames(sender.build_frames(scenario, DATABASE, fault), frames_path)
    csv = tmp_path / f'{tag}.csv'
    serial = tmp_path / f'{tag}.serial.txt'
    tx_log = tmp_path / f'{tag}.tx.txt'
    command = [str(binary), sil_scenario, str(csv), str(serial), '--bus=mem',
               f'--frames={frames_path}', f'--tx-log={tx_log}']
    if duration_ms is not None:
        command.append(f'--duration-ms={duration_ms}')
    subprocess.run(command, check=True)
    return Run(csv, serial, tx_log)


def run_layered(build_dir, tmp_path, sil_scenario):
    csv = tmp_path / f'{sil_scenario}.layered.csv'
    serial = tmp_path / f'{sil_scenario}.layered.serial.txt'
    subprocess.run([str(build_dir / 'sil_layered'), sil_scenario, str(csv), str(serial)], check=True)
    return Run(csv, serial, None)


def first_zero_after(pwm, t_ms, start_ms):
    """start_ms 이후 처음으로 PWM 이 0 이 된 시각까지 걸린 ms. 없으면 None."""
    for t, value in zip(t_ms, pwm):
        if t >= start_ms and value == 0:
            return int(t - start_ms)
    return None


def band_excess(can_values, layered_values, shift=LIVE_SHIFT_MS):
    """샘플마다 can(t) 이 layered 의 [t-shift, t+shift] 값 범위를 벗어난 양."""
    padded = np.pad(layered_values, shift, mode='edge')
    windows = np.lib.stride_tricks.sliding_window_view(padded, 2 * shift + 1)
    low, high = windows.min(axis=1), windows.max(axis=1)
    return np.maximum(np.maximum(low - can_values, can_values - high), 0)


def split_by_hold_windows(excess, scenario):
    """(홀드 구간 밖 최대 초과량, 홀드 구간 안 최대 초과량). t_ms 는 1 부터 시작한다."""
    inside = np.zeros(len(excess), dtype=bool)
    for start, end in LIVE_HOLD_WINDOWS_MS.get(scenario, []):
        inside[start - 1:end] = True
    outside_max = float(excess[~inside].max()) if np.any(~inside) else 0.0
    inside_max = float(excess[inside].max()) if np.any(inside) else 0.0
    return outside_max, inside_max


# ------------------------------------------------------------
# 구조
# ------------------------------------------------------------

@pytest.mark.parametrize('path', sorted(CAN_DIR.glob('Swc_*')), ids=lambda p: p.name)
def test_swc_files_are_unchanged(path):
    assert filecmp.cmp(path, LAYERED_DIR / path.name, shallow=False)


def test_generated_header_matches_dbc():
    subprocess.run([sys.executable, str(CAN_DIR / 'can' / 'gen_com_cfg.py'), '--check'], check=True)


def test_drive_pwm_pin_is_set_before_can_init():
    # Mcal_Can_Mcp2515.cpp 주석 참고. 순서가 바뀌면 실기판에서 D10 이 잠시 HIGH 가 된다.
    source = (CAN_DIR / 't870_drive_can.ino').read_text()
    assert source.index('Mcal_Port_Init();') < source.index('Cdd_CanGateway_Init();')
    mcal = (CAN_DIR / 'Mcal_Can_Mcp2515.cpp').read_text()
    assert 'static MCP2515 instance(' in mcal.split('bool Mcal_Can_Init()')[1]


# Uno 에서 고정이거나 Mcal_Cfg.h 밖에서 이미 쓰는 핀. A0 = 14 이므로 A4, A5 = 18, 19.
RESERVED_PINS = {
    0: 'USB 시리얼 RX', 1: 'USB 시리얼 TX',
    11: 'SPI MOSI', 12: 'SPI MISO', 13: 'SPI SCK',
    15: '배터리 감시 예약(A1)', 18: 'I2C SDA', 19: 'I2C SCL',
}


def read_pin_map():
    pins = {}
    for name, value in re.findall(r'^#define (MCAL_PIN_\w+)\s+(A?\d+)', (CAN_DIR / 'Mcal_Cfg.h').read_text(), re.M):
        pins[name] = 14 + int(value[1:]) if value.startswith('A') else int(value)
    return pins


def test_pin_map_has_no_conflicts():
    pins = read_pin_map()
    assert len(pins) == 11

    assert len(set(pins.values())) == len(pins), pins
    assert not {name: pin for name, pin in pins.items() if pin in RESERVED_PINS}

    # 모드 스위치는 원본 D12 에서 옮겼고, D8 의 PCINT0 벡터(D8~D13)를 같이 쓰지 않는다.
    assert pins['MCAL_PIN_MODE_SWITCH'] == 16
    # CS 는 라이브러리 예제의 D10(구동 PWM)이 아니고, INT 는 엔코더 외부 인터럽트 핀이 아니다.
    assert pins['MCAL_PIN_CAN_CS'] != pins['MCAL_PIN_DRIVE_PWM']
    assert pins['MCAL_PIN_CAN_INT'] not in (pins['MCAL_PIN_ENCODER_A'], pins['MCAL_PIN_ENCODER_B'])
    # INT 는 폴링이다. 외부 인터럽트는 엔코더 하나뿐이어야 한다.
    sources = [path.read_text() for path in CAN_DIR.glob('*.cpp')]
    attached = [line for text in sources for line in text.splitlines() if 'attachInterrupt(' in line]
    assert len(attached) == 1 and 'MCAL_PIN_ENCODER_A' in attached[0]


def test_steer_target_pot_is_reserved_only():
    # DBC 예약 신호다. 생성 헤더 밖에서 읽으면 조향에 연결된 것이다.
    users = [path.name for path in CAN_DIR.glob('*.*')
             if path.suffix in ('.cpp', '.h', '.ino') and 'SteerTargetPot' in path.read_text()]
    assert users == ['Com_Cfg.h']


# ------------------------------------------------------------
# pack / unpack
# ------------------------------------------------------------

class RoundTrip:
    def __init__(self, binary):
        self.process = subprocess.Popen([str(binary)], stdin=subprocess.PIPE,
                                        stdout=subprocess.PIPE, text=True)

    def ask(self, line):
        self.process.stdin.write(line + '\n')
        self.process.stdin.flush()
        return self.process.stdout.readline().strip()

    def close(self):
        self.process.stdin.close()
        self.process.wait()


@pytest.fixture(scope='module')
def round_trip(build_dir):
    tool = RoundTrip(build_dir / 'com_roundtrip')
    yield tool
    tool.close()


def boundary_cases():
    """신호마다 DBC 최소 / 최대 / 0 을, 나머지 신호는 1 또는 0 으로 둔 조합과 전체 최소 / 최대."""
    for message in DATABASE.messages:
        def raw_limits(signal):
            return (round(signal.minimum / signal.scale), round(signal.maximum / signal.scale))

        yield message.name, 'all_min', {s.name: raw_limits(s)[0] for s in message.signals}
        yield message.name, 'all_max', {s.name: raw_limits(s)[1] for s in message.signals}
        for signal in message.signals:
            for label, value in zip(('min', 'max'), raw_limits(signal)):
                values = {s.name: 0 for s in message.signals}
                values[signal.name] = value
                yield message.name, f'{signal.name}_{label}', values


CASES = list(boundary_cases())


@pytest.mark.parametrize('name, label, values', CASES, ids=[f'{c[0]}-{c[1]}' for c in CASES])
def test_pack_unpack_matches_cantools(round_trip, name, label, values):
    message = DATABASE.get_message_by_name(name)
    order = [signal.name for signal in message.signals]
    encoded = message.encode(values, scaling=False, strict=False)

    packed = round_trip.ask(f'pack {name} ' + ' '.join(str(values[s]) for s in order))
    assert packed == encoded.hex().upper()

    unpacked = round_trip.ask(f'unpack {name} {encoded.hex().upper()}').split()
    assert [int(value) for value in unpacked] == [values[s] for s in order]

    decoded = message.decode(encoded, scaling=False, decode_choices=False)
    assert all(decoded[s] == values[s] for s in order)


def test_dbc_physical_limits_keep_their_raw_value():
    message = DATABASE.get_message_by_name('DriveCmd')
    for physical, raw in ((-327.68, -32768), (327.67, 32767), (0.01, 1), (-0.01, -1), (1.5, 150)):
        values = {s.name: 0 for s in message.signals}
        values['TargetSpeed_kmh'] = physical
        decoded = message.decode(message.encode(values, strict=True),
                                 scaling=False, decode_choices=False)
        assert decoded['TargetSpeed_kmh'] == raw


def test_crc8_standard_check_value(round_trip):
    # SAE J1850 검사값: "123456789" -> 0x4B. 마지막 '9' 를 DataID 자리에 넣는다.
    assert int(round_trip.ask(f'crc {b"12345678".hex()} {ord("9")}')) == 0x4B
    assert sender.crc8(b'12345678', ord('9')) == 0x4B


@pytest.mark.parametrize('payload', ['00000000000000', 'FFFFFFFFFFFFFF', '123456789ABCDE'])
@pytest.mark.parametrize('message_name', ['DriveCmd', 'DriveStatus', 'DriveDiag'])
def test_crc8_firmware_matches_sender(round_trip, payload, message_name):
    identifier = sender.data_id(DATABASE.get_message_by_name(message_name))
    assert int(round_trip.ask(f'crc {payload} {identifier}')) == \
        sender.crc8(bytes.fromhex(payload), identifier)


# ------------------------------------------------------------
# 동등성 (가상 시간, 오차 0)
# ------------------------------------------------------------

@pytest.mark.parametrize('scenario', ['speed_step', 'steering', 'mixed'])
def test_can_matches_layered_exactly_on_mem_bus(build_dir, tmp_path, scenario):
    can_run = run_mem(build_dir / 'sil_can', tmp_path, scenario, load_scenario(scenario))
    layered = run_layered(build_dir, tmp_path, scenario)

    assert filecmp.cmp(can_run.csv_path, layered.csv_path, shallow=False)
    assert status_lines(can_run.serial_path) == status_lines(layered.serial_path)
    assert 'E2E' not in can_run.serial


def delay_all(frames):
    return [(t + 1, frame_id, data) for t, frame_id, data in frames]


def delay_one_command(command_ms):
    def apply(frames):
        return [(t + 1 if command_ms <= t < command_ms + FRAME_MS else t, frame_id, data)
                for t, frame_id, data in frames]
    return apply


def delay_randomly(seed):
    def apply(frames):
        generator = random.Random(seed)
        delayed, latest = [], 0
        for t, frame_id, data in frames:
            latest = max(latest, t + generator.choice((0, 1, 2)))   # 프레임 순서는 지킨다
            delayed.append((latest, frame_id, data))
        return delayed
    return apply


@pytest.mark.parametrize('delay', [
    delay_all, delay_one_command(9000), delay_one_command(12000), delay_one_command(29000),
    delay_randomly(0), delay_randomly(1), delay_randomly(2),
], ids=['all_1ms', 'hold_9000', 'estop_12000', 'hold_29000', 'random0', 'random1', 'random2'])
def test_arrival_delay_differs_only_in_hold_windows(build_dir, tmp_path, delay):
    # 실시간 허용 오차의 근거. 지터 없는 가상 시간에서 도착만 1~2 ms 늦춰도 차이가 나고,
    # 그 차이는 미션 홀드 구간 안에만 있어야 한다.
    frames_path = tmp_path / 'delayed.frames'
    sender.write_frames(delay(sender.build_frames(load_scenario('mixed'), DATABASE)), frames_path)
    csv, serial = tmp_path / 'delayed.csv', tmp_path / 'delayed.serial.txt'
    subprocess.run([str(build_dir / 'sil_can'), 'mixed', str(csv), str(serial),
                    '--bus=mem', f'--frames={frames_path}'], check=True)
    layered = run_layered(build_dir, tmp_path, 'mixed')

    delayed = read_csv(csv)
    for column in ('drive_pwm', 'steer_pwm'):
        excess = band_excess(ints(delayed[column]), ints(layered.data[column]))
        outside_max, inside_max = split_by_hold_windows(excess, 'mixed')
        assert outside_max <= LIVE_PWM_TOLERANCE, column
        assert inside_max <= LIVE_HOLD_PWM_TOLERANCE, column


def test_watchdog_scenario_is_expected_to_differ(build_dir, tmp_path):
    # 시리얼은 3 s 워치독 뒤 감속 정지, CAN 은 100 ms 타임아웃 뒤 즉시 정지. 의도된 차이다.
    can_run = run_mem(build_dir / 'sil_can', tmp_path, 'watchdog', load_scenario('watchdog'))
    layered = run_layered(build_dir, tmp_path, 'watchdog')

    t_ms = ints(can_run.data['t_ms'])
    last_frame_ms = 5000 - FRAME_MS
    can_pwm, layered_pwm = ints(can_run.data['drive_pwm']), ints(layered.data['drive_pwm'])

    assert np.array_equal(can_pwm[t_ms <= last_frame_ms], layered_pwm[t_ms <= last_frame_ms])
    assert first_zero_after(can_pwm, t_ms, last_frame_ms) == TIMEOUT_MS + 1
    assert first_zero_after(layered_pwm, t_ms, last_frame_ms) > 3000


def test_status_frames_report_firmware_outputs(build_dir, tmp_path):
    run = run_mem(build_dir / 'sil_can', tmp_path, 'mixed', load_scenario('mixed'))
    data = run.data
    drive, steer = ints(data['drive_pwm']), ints(data['steer_pwm'])

    frames = run.frames('DriveStatus')
    assert len(frames) == 32000 // STATUS_PERIOD_MS

    previous_alive = None
    for t, signals, crc_ok in frames:
        assert crc_ok
        assert signals['DrivePwm'] == drive[t - 1]
        assert signals['SteerPwm'] == steer[t - 1]
        assert abs(signals['SteerPot'] - float(data['pot'][t - 1])) <= 2
        if previous_alive is not None:
            assert signals['Alive'] == (previous_alive + 1) & 0x0F
        previous_alive = signals['Alive']

    diag = run.frames('DriveDiag')
    assert len(diag) == 32000 // 500
    assert all(crc_ok for _, _, crc_ok in diag)
    last = diag[-1][1]
    assert (last['CrcErrorCount'], last['AliveJumpCount'], last['AliveRepeatCount'],
            last['TimeoutCount'], last['E2EState']) == (0, 0, 0, 0, 0)


def test_target_at_dbc_limit_is_clamped_by_swc(build_dir, tmp_path):
    # 최고 속도 제한은 Swc 에 있다. DBC 범위 끝 값이 들어와도 10 km/h 로 잘려야 한다.
    for target, expected in ((327.67, 'TARGET = 10.00 km/h'), (-327.68, 'TARGET = -10.00 km/h')):
        scenario = sender.Scenario(2000, [(0, {'mode': 'AUTO', 'target': str(target)})])
        run = run_mem(build_dir / 'sil_can', tmp_path, 'speed_step', scenario, duration_ms=2000)
        assert expected in run.serial


# ------------------------------------------------------------
# E2E 고장 주입 (가상 시간)
# ------------------------------------------------------------

FAULT_AT_MS = 5000
FAULT_DURATION_MS = 7000


def faulted_run(build_dir, tmp_path, kind, frames=None, tag='fault'):
    # speed_step 은 1 s 부터 1.5 km/h 로 달린다. 5 s 에는 구동 PWM 이 0 보다 크다.
    fault = sender.Fault(kind, FAULT_AT_MS, frames)
    return run_mem(build_dir / 'sil_can', tmp_path, 'speed_step',
                   load_scenario('speed_step', FAULT_DURATION_MS), fault, tag=tag,
                   duration_ms=FAULT_DURATION_MS)


@pytest.mark.parametrize('kind, expected_ms', FAULT_CASES, ids=[c[0] for c in FAULT_CASES])
def test_fault_reaches_safe_state_in_time(build_dir, tmp_path, kind, expected_ms):
    run = faulted_run(build_dir, tmp_path, kind)
    data = run.data
    t_ms, pwm = ints(data['t_ms']), ints(data['drive_pwm'])

    assert np.all(pwm[(t_ms >= 3000) & (t_ms < FAULT_AT_MS)] > 0), '고장 전에 달리고 있어야 한다'
    assert first_zero_after(pwm, t_ms, FAULT_AT_MS) == expected_ms
    assert np.all(pwm[t_ms >= FAULT_AT_MS + expected_ms] == 0), '래치가 풀리기 전에는 0 이어야 한다'

    assert run.serial.count('E2E SAFE STATE') == 1
    assert run.serial.count('EMERGENCY STOP') == 1
    assert 'E2E-FAULT' in run.serial

    status = run.frames('DriveStatus')
    raised = next(t for t, signals, _ in status if signals['FaultE2E'])
    assert raised - (FAULT_AT_MS + expected_ms) <= STATUS_PERIOD_MS
    assert all(not signals['FaultE2E'] for t, signals, _ in status if t < FAULT_AT_MS)


@pytest.mark.parametrize('kind, counter, minimum', [
    ('crc', 'CrcErrorCount', FAIL_THRESHOLD),
    ('dlc', 'CrcErrorCount', FAIL_THRESHOLD),
    ('alive_jump', 'AliveJumpCount', FAIL_THRESHOLD),
    ('alive_freeze', 'AliveRepeatCount', FAIL_THRESHOLD),
    ('stop', 'TimeoutCount', 1),
])
def test_fault_is_counted_in_diag_frame(build_dir, tmp_path, kind, counter, minimum):
    run = faulted_run(build_dir, tmp_path, kind)
    last = run.frames('DriveDiag')[-1][1]
    assert last[counter] >= minimum
    others = {'CrcErrorCount', 'AliveJumpCount', 'AliveRepeatCount'} - {counter}
    assert all(last[name] == 0 for name in others)
    # 끝까지 이어지는 고장이면 정상 프레임도 끊기므로 타임아웃도 한 번 센다.
    assert last['TimeoutCount'] == 1


@pytest.mark.parametrize('kind, frames, counter, expected', [
    ('crc', 1, 'CrcErrorCount', 1),
    # 깨진 프레임 2 개 뒤 정상 프레임은 Alive 가 3 건너뛰지만 고장을 다시 세지 않는다.
    ('crc', 2, 'CrcErrorCount', 2),
    ('alive_jump', 1, 'AliveJumpCount', 1),
    ('alive_jump', 2, 'AliveJumpCount', 2),
    ('alive_freeze', 2, 'AliveRepeatCount', 2),
])
def test_faults_below_threshold_do_not_stop_the_car(build_dir, tmp_path, kind, frames, counter, expected):
    clean = run_mem(build_dir / 'sil_can', tmp_path, 'speed_step',
                    load_scenario('speed_step', FAULT_DURATION_MS), tag='clean',
                    duration_ms=FAULT_DURATION_MS)
    run = faulted_run(build_dir, tmp_path, kind, frames)

    assert filecmp.cmp(run.csv_path, clean.csv_path, shallow=False)
    assert 'E2E SAFE STATE' not in run.serial
    last = run.frames('DriveDiag')[-1][1]
    assert last[counter] == expected
    assert last['AliveJumpCount'] == (expected if counter == 'AliveJumpCount' else 0)


def test_no_frames_at_boot_means_no_motion_and_no_latch(build_dir, tmp_path):
    scenario = sender.Scenario(6000, [(0, {'stop': '1'})])
    run = run_mem(build_dir / 'sil_can', tmp_path, 'speed_step', scenario, duration_ms=6000)
    assert np.all(ints(run.data['drive_pwm']) == 0)
    assert 'E2E' not in run.serial


def test_safe_state_cancels_steer_open_loop(build_dir, tmp_path):
    scenario = sender.Scenario(3000, [(0, {'mode': 'AUTO', 'target': '0'}),
                                      (1000, {'sel': 'STEER', 'duty': '60'}),
                                      (2000, {'stop': '1'})])
    run = run_mem(build_dir / 'sil_can', tmp_path, 'speed_step', scenario, duration_ms=3000)
    lines = status_lines(run.serial_path)
    assert any('STEER-OPEN' in line for line in lines[:10])
    assert 'STEER-OPEN' not in lines[-1]


def test_rc_manual_driving_ignores_laptop_loss(build_dir, tmp_path):
    # mixed 는 16 s 부터 RC MANUAL 로 달린다. 노트북이 죽어도 RC 주행은 그대로여야 한다.
    scenario = load_scenario('mixed')
    clean = run_mem(build_dir / 'sil_can', tmp_path, 'mixed', scenario, tag='clean')
    lost = run_mem(build_dir / 'sil_can', tmp_path, 'mixed', scenario,
                   sender.Fault('stop', MANUAL_DRIVE_T_MS), tag='lost')

    t_ms = ints(clean.data['t_ms'])
    manual = t_ms < MANUAL_END_T_MS
    assert np.array_equal(ints(clean.data['drive_pwm'])[manual], ints(lost.data['drive_pwm'])[manual])
    assert np.any(ints(lost.data['drive_pwm'])[(t_ms > MANUAL_DRIVE_T_MS) & manual] > 0)
    assert 'E2E-FAULT' in lost.serial

    # AUTO 로 돌아와도 노트북 명령이 없으니 달리지 않는다.
    assert np.all(ints(lost.data['drive_pwm'])[t_ms > MANUAL_END_T_MS + 2000] == 0)


def test_latch_clear_requires_zero_target_and_then_recovers(build_dir, tmp_path):
    scenario = load_scenario('recovery')
    run = run_mem(build_dir / 'sil_can', tmp_path, 'speed_step', scenario,
                  sender.Fault('crc', 3000, 5), duration_ms=scenario.duration_ms)
    data = run.data
    t_ms, pwm = ints(data['t_ms']), ints(data['drive_pwm'])
    fault_at = {t: signals['FaultE2E'] for t, signals, _ in run.frames('DriveStatus')}

    # 3 s 고장 뒤 프레임이 정상으로 돌아와도 정지를 유지한다.
    assert np.all(pwm[(t_ms > 3040) & (t_ms < 7000)] == 0)
    assert fault_at[3200] == 1

    # 5 s 해제 요청은 목표가 1.5 로 남아 있어 무시된다.
    assert fault_at[5900] == 1
    assert run.serial.count('LATCH CLEARED') == 1

    # 6.1 s 해제 요청은 목표가 0 이라 받아들여진다. 그래도 7 s 새 목표 전에는 안 움직인다.
    assert fault_at[6200] == 0
    assert np.any(pwm[t_ms > 8000] > 0)


# ------------------------------------------------------------
# 이 테스트들이 차이를 정말 잡는지 (t870_drive_can 사본을 바꿔 본다)
# ------------------------------------------------------------

def mutated_binary(tmp_path, file_name, before, after):
    mutated = tmp_path / 't870_drive_can'
    shutil.copytree(CAN_DIR, mutated)
    source = mutated / file_name
    text = source.read_text()
    assert before in text, f'{before!r} not found in {file_name}'
    source.write_text(text.replace(before, after))
    out = tmp_path / 'build'
    build(out, mutated)
    return out / 'sil_can'


@pytest.mark.parametrize('file_name, before, after', [
    ('Com_Cfg.h', 'TARGETSPEED_KMH_RAW_PER_UNIT 100.0f', 'TARGETSPEED_KMH_RAW_PER_UNIT 101.0f'),
    ('Cdd_CanGateway.cpp', 'notifyInt(RTE_CMD_MISSION_HOLD, 0);', ';'),
    ('Cdd_CanGateway.cpp', 'Rte_Read_Watchdog_Enabled() == cmd.WatchdogDisable', 'false'),
], ids=['target_scale', 'hold_dropped', 'watchdog_dropped'])
def test_equivalence_detects_gateway_change(build_dir, tmp_path, file_name, before, after):
    binary = mutated_binary(tmp_path, file_name, before, after)
    run = run_mem(binary, tmp_path, 'mixed', load_scenario('mixed'))
    layered = run_layered(build_dir, tmp_path, 'mixed')
    assert not (filecmp.cmp(run.csv_path, layered.csv_path, shallow=False)
                and status_lines(run.serial_path) == status_lines(layered.serial_path))


def faulted_run_with(binary, tmp_path, kind, frames=None):
    return run_mem(binary, tmp_path, 'speed_step', load_scenario('speed_step', FAULT_DURATION_MS),
                   sender.Fault(kind, FAULT_AT_MS, frames), duration_ms=FAULT_DURATION_MS)


def test_fault_timing_detects_changed_threshold(tmp_path):
    binary = mutated_binary(tmp_path, 'Com.h', 'E2E_FAIL_THRESHOLD = 3;', 'E2E_FAIL_THRESHOLD = 4;')
    run = faulted_run_with(binary, tmp_path, 'crc')
    data = run.data
    assert first_zero_after(ints(data['drive_pwm']), ints(data['t_ms']), FAULT_AT_MS) != FRAME_FAULT_MS


def test_crc_error_double_count_is_detected(tmp_path):
    binary = mutated_binary(tmp_path, 'E2E.cpp', '1 + state.crcMissed', '1')
    run = faulted_run_with(binary, tmp_path, 'crc', frames=1)
    assert run.frames('DriveDiag')[-1][1]['AliveJumpCount'] == 1


# ------------------------------------------------------------
# 실제 SocketCAN (vcan0). 실시간이라 오래 걸린다.
# ------------------------------------------------------------

needs_vcan = pytest.mark.skipif(
    not pathlib.Path(f'/sys/class/net/{CAN_INTERFACE}').exists(),
    reason=f'{CAN_INTERFACE} 없음: sudo modprobe vcan && sudo ip link add dev vcan0 type vcan '
           '&& sudo ip link set up vcan0')


class LiveRun(Run):
    def __init__(self, csv, serial, tx_log, status_csv):
        super().__init__(csv, serial, tx_log)
        self.status_csv = status_csv


def run_live(binary, tmp_path, sil_scenario, scenario, duration_ms, fault=None, tag='live'):
    scenario_path = tmp_path / f'{tag}.scenario.txt'
    lines = [f'duration {duration_ms}']
    for t, change in scenario.changes:
        lines.append(f'{t} ' + ' '.join(f'{k}={v}' for k, v in change.items()))
    scenario_path.write_text('\n'.join(lines) + '\n')

    csv = tmp_path / f'{tag}.csv'
    serial = tmp_path / f'{tag}.serial.txt'
    tx_log = tmp_path / f'{tag}.tx.txt'
    status_csv = tmp_path / f'{tag}.status.csv'

    ecu = subprocess.Popen([str(binary), sil_scenario, str(csv), str(serial),
                            f'--bus={CAN_INTERFACE}', f'--tx-log={tx_log}',
                            f'--duration-ms={duration_ms}'])
    time.sleep(0.3)   # 소켓을 열 시간. 그 전에 보낸 프레임은 사라진다.

    command = [sys.executable, str(SENDER_PATH), str(scenario_path),
               '--bus', CAN_INTERFACE, '--record', str(status_csv)]
    if fault is not None:
        command += ['--fault', fault.kind, '--fault-at', str(fault.at_ms)]
        if fault.frames is not None:
            command += ['--fault-frames', str(fault.frames)]
    completed = subprocess.run(command, check=True, capture_output=True, text=True)
    print(completed.stderr.strip())
    assert ecu.wait(timeout=duration_ms / 1000 + 20) == 0
    return LiveRun(csv, serial, tx_log, status_csv)


@needs_vcan
@pytest.mark.parametrize('scenario', ['speed_step', 'steering', 'mixed'])
def test_can_matches_layered_on_vcan(build_dir, tmp_path, scenario):
    plan = load_scenario(scenario)
    live = run_live(build_dir / 'sil_can', tmp_path, scenario, plan, plan.duration_ms)
    layered = run_layered(build_dir, tmp_path, scenario)

    assert 'E2E' not in live.serial, '정상 전송에서 E2E 고장이 나면 안 된다'
    for column in ('drive_pwm', 'steer_pwm'):
        excess = band_excess(ints(live.data[column]), ints(layered.data[column]))
        outside_max, inside_max = split_by_hold_windows(excess, scenario)
        print(f'{scenario}.{column}: excess outside hold {outside_max}, inside hold {inside_max}')
        assert outside_max <= LIVE_PWM_TOLERANCE, column
        assert inside_max <= LIVE_HOLD_PWM_TOLERANCE, column


@needs_vcan
@pytest.mark.parametrize('kind, expected_ms', FAULT_CASES, ids=[c[0] for c in FAULT_CASES])
def test_fault_reaches_safe_state_on_vcan(build_dir, tmp_path, kind, expected_ms):
    plan = load_scenario('speed_step')
    live = run_live(build_dir / 'sil_can', tmp_path, 'speed_step', plan, FAULT_DURATION_MS,
                    sender.Fault(kind, FAULT_AT_MS))
    data = live.data
    t_ms, pwm = ints(data['t_ms']), ints(data['drive_pwm'])

    assert np.all(pwm[(t_ms >= 3000) & (t_ms < FAULT_AT_MS - 2 * FRAME_MS)] > 0)
    stopped = first_zero_after(pwm, t_ms, FAULT_AT_MS)
    print(f'{kind}: stopped {stopped} ms after fault')
    assert stopped is not None and stopped <= expected_ms + LIVE_FAULT_MARGIN_MS
    assert np.all(pwm[t_ms >= FAULT_AT_MS + expected_ms + LIVE_FAULT_MARGIN_MS] == 0)

    # 노트북이 실제로 받은 프레임에서 FaultE2E 가 보이는지 본다.
    rows = read_csv(live.status_csv)
    status = [i for i, name in enumerate(rows['name']) if name == 'DriveStatus']
    assert all(rows['crc_ok'][i] == '1' for i in range(len(rows['name'])))
    raised = next(float(rows['t_ms'][i]) for i in status if rows['FaultE2E'][i] == '1')
    assert raised - FAULT_AT_MS <= expected_ms + STATUS_PERIOD_MS + LIVE_FAULT_MARGIN_MS
