"""원본 sketch_aug.ino 와 계층 버전 t870_drive_layered 의 동등성 테스트.

두 펌웨어를 PC 에서 각각 빌드해 같은 모의 플랜트 · 같은 입력으로 닫힌 루프를
돌리고, 다음 두 가지가 바이트 단위로 같은지 본다.

  - 1 ms 마다 기록한 액추에이터 출력과 플랜트 상태 (구동/조향 PWM, DIR, 속도, POT)
  - 펌웨어가 시리얼로 낸 모든 문자열 (명령 응답, 200 ms 상태 줄)

출력이 플랜트를 거쳐 다음 입력이 되므로, 한 주기라도 계산이 다르면 그 뒤
기록 전체가 갈라진다.

테스트가 실제로 차이를 잡아내는지도 확인한다. 계층 버전의 상수 하나를
살짝 바꾼 사본을 빌드해, 원본과 달라지는지 본다.

    /usr/bin/python3 -m pytest -q test_equivalence.py
"""

import filecmp
import pathlib
import shutil
import subprocess

import pytest

SIL_DIR = pathlib.Path(__file__).resolve().parent
LAYERED_DIR = SIL_DIR.parent / 't870_drive_layered'
SCENARIOS = ['speed_step', 'steering', 'watchdog', 'mixed']


def build(build_dir, layered_dir=LAYERED_DIR):
    subprocess.run(
        ['make', '-s', f'BUILD={build_dir}', f'LAYERED={layered_dir}',
         f'{build_dir}/sil_layered', f'{build_dir}/sil_original'],
        cwd=SIL_DIR, check=True)


def run(binary, scenario, out_dir):
    csv = out_dir / f'{scenario}.{binary.name}.csv'
    serial = out_dir / f'{scenario}.{binary.name}.serial.txt'
    subprocess.run([str(binary), scenario, str(csv), str(serial)], check=True)
    return csv, serial


def first_difference(path_a, path_b):
    lines_a = path_a.read_text().splitlines()
    lines_b = path_b.read_text().splitlines()
    for number, (a, b) in enumerate(zip(lines_a, lines_b), start=1):
        if a != b:
            return f'line {number}\n  layered : {a}\n  original: {b}'
    return f'length differs: {len(lines_a)} vs {len(lines_b)} lines'


@pytest.fixture(scope='module')
def binaries(tmp_path_factory):
    build_dir = tmp_path_factory.mktemp('build')
    build(build_dir)
    return build_dir / 'sil_layered', build_dir / 'sil_original'


@pytest.mark.parametrize('scenario', SCENARIOS)
def test_layered_matches_original(binaries, scenario, tmp_path):
    layered, original = binaries
    layered_csv, layered_serial = run(layered, scenario, tmp_path)
    original_csv, original_serial = run(original, scenario, tmp_path)

    assert filecmp.cmp(layered_csv, original_csv, shallow=False), \
        first_difference(layered_csv, original_csv)
    assert filecmp.cmp(layered_serial, original_serial, shallow=False), \
        first_difference(layered_serial, original_serial)


# 계층 버전 상수를 하나씩 바꾼 사본. 이 차이를 못 잡으면 위 테스트는 의미가 없다.
MUTATIONS = [
    ('Swc_StopHold.cpp', 'HOLD_GAIN         = 50.0;', 'HOLD_GAIN         = 50.5;'),
    ('Swc_SpeedControl.cpp', 'AUTO_KI = 5.0;', 'AUTO_KI = 5.1;'),
    ('Swc_SteeringControl.cpp', 'STEER_DEADBAND = 15;', 'STEER_DEADBAND = 16;'),
]


@pytest.mark.parametrize('file_name, before, after', MUTATIONS,
                         ids=[m[0] for m in MUTATIONS])
def test_detects_changed_constant(binaries, tmp_path, file_name, before, after):
    mutated = tmp_path / 't870_drive_layered'
    shutil.copytree(LAYERED_DIR, mutated)
    source = mutated / file_name
    text = source.read_text()
    assert before in text, f'{before!r} not found in {file_name}'
    source.write_text(text.replace(before, after))

    build_dir = tmp_path / 'build'
    build(build_dir, mutated)

    _, original = binaries
    mutated_csv, _ = run(build_dir / 'sil_layered', 'mixed', tmp_path)
    original_csv, _ = run(original, 'mixed', tmp_path)
    assert not filecmp.cmp(mutated_csv, original_csv, shallow=False)
