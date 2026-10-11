"""SIL 결과 CSV 를 그래프 PNG 로 만든다.

    /usr/bin/python3 plot_sil.py <csv 폴더> <png 폴더>

입력: <csv 폴더>/{speed_step,steering,watchdog}.csv 와 .serial.txt
펌웨어 램프 목표(Tgt)는 시리얼 상태 줄에서 읽는다. 상태 줄은 200 ms 마다
나오므로 k 번째 줄의 시각은 200*k ms 다.
"""

import csv
import pathlib
import re
import sys

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

# 기본 팔레트 (dataviz 참조 팔레트 light, 검증 통과)
SURFACE = '#fcfcfb'
TEXT_PRIMARY = '#0b0b0b'
TEXT_SECONDARY = '#52514e'
GRID = '#e4e3df'
SERIES_1 = '#2a78d6'   # 측정값 (차속, POT, PWM)
SERIES_2 = '#eb6834'   # 펌웨어 내부 목표 (램프)
REFERENCE = '#8a8983'  # 입력 · 기준선 (점선)

plt.rcParams.update({
    'font.family': ['NanumGothic', 'DejaVu Sans'],
    'axes.unicode_minus': False,
    'figure.facecolor': SURFACE,
    'axes.facecolor': SURFACE,
    'axes.edgecolor': GRID,
    'axes.labelcolor': TEXT_SECONDARY,
    'axes.titlecolor': TEXT_PRIMARY,
    'axes.titlesize': 11,
    'axes.titleweight': 'bold',
    'axes.grid': True,
    'grid.color': GRID,
    'grid.linewidth': 0.8,
    'xtick.color': TEXT_SECONDARY,
    'ytick.color': TEXT_SECONDARY,
    'legend.frameon': False,
    'legend.labelcolor': TEXT_PRIMARY,
    'lines.linewidth': 2.0,
})

TGT_PATTERN = re.compile(r'Tgt:(-?[\d.]+)')


def load_csv(path):
    with open(path, newline='') as f:
        rows = list(csv.DictReader(f))
    columns = {key: [] for key in rows[0]}
    for row in rows:
        for key, value in row.items():
            columns[key].append(value)
    return columns


def floats(values):
    return [float(v) for v in values]


def load_firmware_target(serial_path):
    times, targets = [], []
    status_index = 0
    for line in serial_path.read_text().splitlines():
        if line.startswith('RC:'):
            status_index += 1
            match = TGT_PATTERN.search(line)
            if match:
                times.append(status_index * 0.2)
                targets.append(float(match.group(1)))
    return times, targets


def finish(fig, axes, title, out_path):
    for ax in axes:
        for side in ('top', 'right'):
            ax.spines[side].set_visible(False)
    axes[-1].set_xlabel('시간 (s)')
    fig.suptitle(title, color=TEXT_PRIMARY, fontsize=13, fontweight='bold', x=0.01, ha='left')
    fig.text(0.01, 0.005, '플랜트 모델은 실측이 아닌 가정값이다. 펌웨어 계산만 원본과 같다.',
             color=TEXT_SECONDARY, fontsize=8, ha='left', va='bottom')
    fig.tight_layout(rect=(0, 0.03, 1, 0.97))
    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def plot_speed(data, serial_path, out_path, title, events=()):
    t = [v / 1000.0 for v in floats(data['t_ms'])]
    ros = [float(r) if alive == '1' else float('nan')
           for r, alive in zip(data['ros_target_kmh'], data['ros_alive'])]
    tgt_t, tgt = load_firmware_target(serial_path)

    fig, (ax_speed, ax_pwm) = plt.subplots(
        2, 1, figsize=(9, 5.6), sharex=True, gridspec_kw={'height_ratios': [3, 2]})
    ax_speed.plot(t, ros, color=REFERENCE, linestyle='--', linewidth=1.5, label='ROS 목표')
    ax_speed.plot(tgt_t, tgt, color=SERIES_2, label='펌웨어 램프 목표 (Tgt)')
    ax_speed.plot(t, floats(data['speed_kmh']), color=SERIES_1, label='차속 (모의 플랜트)')
    ax_speed.set_ylabel('km/h')
    ax_speed.set_title('목표와 차속', loc='left')
    ax_speed.legend(loc='lower right', bbox_to_anchor=(1.0, 1.0), ncol=3)

    ax_pwm.plot(t, floats(data['drive_pwm']), color=SERIES_1)
    ax_pwm.set_ylabel('duty (0~255)')
    ax_pwm.set_title('구동 PWM', loc='left')

    # (x, 이름, 'left' 면 선 오른쪽에, 'right' 면 선 왼쪽에 쓴다)
    # 두 이름이 같은 틈에 들어가므로 높이를 엇갈린다.
    for index, (x, label, align) in enumerate(events):
        for ax in (ax_speed, ax_pwm):
            ax.axvline(x, color=TEXT_SECONDARY, linewidth=1, linestyle=':')
        ax_speed.annotate(label, xy=(x, 0.62 - 0.2 * index), xycoords=('data', 'axes fraction'),
                          xytext=(4 if align == 'left' else -4, 0), textcoords='offset points',
                          color=TEXT_SECONDARY, fontsize=9, ha=align)
    finish(fig, (ax_speed, ax_pwm), title, out_path)


def plot_steering(data, out_path):
    t = [v / 1000.0 for v in floats(data['t_ms'])]
    signed_pwm = [p if d == 0 else -p   # DIR LOW = POT 증가 방향을 + 로 그린다
                  for p, d in zip(map(int, data['steer_pwm']), map(int, data['steer_dir']))]

    fig, (ax_pot, ax_pwm) = plt.subplots(
        2, 1, figsize=(9, 5.6), sharex=True, gridspec_kw={'height_ratios': [3, 2]})
    ax_pot.plot(t, floats(data['steer_target_ref']), color=REFERENCE, linestyle='--',
                linewidth=1.5, label='POT 목표 (Pixhawk PWM 매핑)')
    ax_pot.plot(t, floats(data['pot']), color=SERIES_1, label='POT (모의 플랜트)')
    ax_pot.set_ylabel('ADC count')
    ax_pot.set_title('조향 목표와 포텐셔미터', loc='left')
    ax_pot.legend(loc='lower right', bbox_to_anchor=(1.0, 1.0), ncol=2)
    # 구간 이름은 POT 가 목표에 닿은 뒤의 빈 자리에 둔다.
    for x, label in ((1.0, '좌 1100 us'), (4.0, '중앙 1500 us'), (7.0, '우 1900 us')):
        ax_pot.annotate(label, xy=(x, 0.78), xycoords=('data', 'axes fraction'),
                        color=TEXT_SECONDARY, fontsize=9)

    ax_pwm.plot(t, signed_pwm, color=SERIES_1)
    ax_pwm.axhline(0, color=TEXT_SECONDARY, linewidth=0.8)
    ax_pwm.set_ylabel('duty (+: POT 증가)')
    ax_pwm.set_title('조향 PWM', loc='left')
    finish(fig, (ax_pot, ax_pwm), '(b) 조향 목표 좌 → 중앙 → 우', out_path)


def main():
    in_dir = pathlib.Path(sys.argv[1])
    out_dir = pathlib.Path(sys.argv[2])
    out_dir.mkdir(parents=True, exist_ok=True)

    plot_speed(load_csv(in_dir / 'speed_step.csv'), in_dir / 'speed_step.serial.txt',
               out_dir / 'speed_step.png', '(a) 목표 속도 0 → 1.5 km/h 계단')

    plot_steering(load_csv(in_dir / 'steering.csv'), out_dir / 'steering.png')

    watchdog = load_csv(in_dir / 'watchdog.csv')
    alive = watchdog['ros_alive']
    lost_ms = next(int(t) for t, a in zip(watchdog['t_ms'], alive) if a == '0')
    # 마지막 명령은 끊기기 직전 100 ms 틱. 워치독은 그 뒤 3 s 를 넘기면 만료된다.
    expire_s = (lost_ms - 100 + 3000) / 1000.0
    plot_speed(watchdog, in_dir / 'watchdog.serial.txt', out_dir / 'watchdog.png',
               '(c) ROS 명령 끊김 → 명령 워치독 정지',
               events=((lost_ms / 1000.0, 'ROS 명령 끊김', 'left'),
                       (expire_s, '워치독 만료 (끊긴 뒤 3 s)', 'right')))


if __name__ == '__main__':
    main()
