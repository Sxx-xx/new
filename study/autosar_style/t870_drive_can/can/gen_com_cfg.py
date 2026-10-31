"""DBC 에서 Com_Cfg.h 를 생성한다.

    python3 can/gen_com_cfg.py              # ../Com_Cfg.h 를 다시 쓴다
    python3 can/gen_com_cfg.py --check      # 파일이 DBC 와 같은지만 본다 (다르면 종료 코드 1)

펌웨어 빌드에는 파이썬이 필요 없도록 생성 결과를 저장소에 함께 둔다.
"""

import argparse
import pathlib
import sys

import cantools

CAN_DIR = pathlib.Path(__file__).resolve().parent
DBC_PATH = CAN_DIR / 't870.dbc'
HEADER_PATH = CAN_DIR.parent / 'Com_Cfg.h'

HELPERS = '''\
// 리틀엔디언(Intel) 비트 필드 읽기 / 쓰기
static inline uint32_t Com_GetBits(const uint8_t *data, uint8_t start, uint8_t length)
{
  uint32_t value = 0;
  for (uint8_t i = 0; i < length; ++i)
  {
    const uint8_t bit = start + i;
    if (data[bit / 8] & (uint8_t)(1U << (bit % 8)))
    {
      value |= ((uint32_t)1) << i;
    }
  }
  return value;
}

static inline int32_t Com_GetSignedBits(const uint8_t *data, uint8_t start, uint8_t length)
{
  const uint32_t value = Com_GetBits(data, start, length);
  const uint32_t signBit = ((uint32_t)1) << (length - 1);
  return (value & signBit) ? (int32_t)value - ((int32_t)signBit * 2) : (int32_t)value;
}

static inline void Com_SetBits(uint8_t *data, uint8_t start, uint8_t length, uint32_t value)
{
  for (uint8_t i = 0; i < length; ++i)
  {
    const uint8_t bit = start + i;
    const uint8_t mask = (uint8_t)(1U << (bit % 8));
    if ((value >> i) & 1U)
    {
      data[bit / 8] |= mask;
    }
    else
    {
      data[bit / 8] &= (uint8_t)~mask;
    }
  }
}
'''


def macro(*parts):
    return '_'.join(parts).upper()


def c_type(signal):
    if signal.length == 1:
        return 'bool'
    bits = 8 if signal.length <= 8 else 16
    return f"{'int' if signal.is_signed else 'uint'}{bits}_t"


def attribute(message, name):
    return message.dbc.attributes[name].value


def check_supported(database):
    for message in database.messages:
        if message.length != 8:
            sys.exit(f'{message.name}: 8 바이트 프레임만 지원한다')
        for signal in message.signals:
            if signal.byte_order != 'little_endian':
                sys.exit(f'{message.name}.{signal.name}: 리틀엔디언만 지원한다')
            if signal.length > 16:
                sys.exit(f'{message.name}.{signal.name}: 16 비트까지만 지원한다')
            if signal.offset != 0:
                sys.exit(f'{message.name}.{signal.name}: offset 은 지원하지 않는다')


def generate(database):
    out = [
        '// ============================================================',
        '// 자동 생성 파일. 직접 고치지 말 것.',
        '// 원본: can/t870.dbc, 생성기: can/gen_com_cfg.py',
        '//',
        '// 구조체 멤버는 스케일을 적용하지 않은 raw 값이다.',
        '// 물리값 = raw * FACTOR. 부동소수 오차를 피하려고 raw / RAW_PER_UNIT 으로 나눠 쓴다.',
        '// ============================================================',
        '',
        '#ifndef COM_CFG_H',
        '#define COM_CFG_H',
        '',
        '#include "Std_Types.h"',
        '',
        HELPERS,
    ]

    for message in database.messages:
        prefix = macro('COM', message.name)
        out += [
            f'// ---- {message.name} ----',
            f'#define {prefix}_ID          0x{message.frame_id:03X}',
            f'#define {prefix}_DLC         {message.length}',
            f'#define {prefix}_CYCLE_MS    {attribute(message, "GenMsgCycleTime")}',
            f'#define {prefix}_DATA_ID     0x{attribute(message, "E2E_DataID"):02X}',
        ]
        for signal in message.signals:
            name = macro(prefix, signal.name)
            out.append(f'#define {name}_START {signal.start}')
            out.append(f'#define {name}_LENGTH {signal.length}')
            if signal.scale != 1:
                raw_per_unit = round(1 / signal.scale)
                if abs(raw_per_unit * signal.scale - 1) > 1e-9:
                    sys.exit(f'{message.name}.{signal.name}: FACTOR 의 역수가 정수여야 한다')
                out.append(f'#define {name}_RAW_PER_UNIT {raw_per_unit}.0f')
        for signal in message.signals:
            for label_value, label in sorted(
                    (int(value), str(label)) for value, label in (signal.choices or {}).items()):
                out.append(f'#define {macro(prefix, signal.name, label)} {label_value}')

        struct = f'Com_{message.name}Type'
        out += ['', f'struct {struct}', '{']
        out += [f'  {c_type(s)} {s.name};' for s in message.signals]
        out += ['};', '']

        out += [f'static inline void Com_Pack_{message.name}(const {struct} &s, uint8_t *data)', '{']
        for signal in message.signals:
            value = f'(uint32_t)(int32_t)s.{signal.name}' if signal.is_signed else f's.{signal.name}'
            out.append(f'  Com_SetBits(data, {signal.start}, {signal.length}, {value});')
        out += ['}', '']

        out += [f'static inline void Com_Unpack_{message.name}(const uint8_t *data, {struct} &s)', '{']
        for signal in message.signals:
            if signal.is_signed:
                read = f'Com_GetSignedBits(data, {signal.start}, {signal.length})'
            else:
                read = f'Com_GetBits(data, {signal.start}, {signal.length})'
            if signal.length == 1:
                read += ' != 0'
                out.append(f'  s.{signal.name} = {read};')
            else:
                out.append(f'  s.{signal.name} = ({c_type(signal)}){read};')
        out += ['}', '']

    out += ['#endif', '']
    return '\n'.join(out)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true')
    args = parser.parse_args()

    database = cantools.database.load_file(DBC_PATH)
    check_supported(database)
    text = generate(database)

    if args.check:
        if not HEADER_PATH.exists() or HEADER_PATH.read_text() != text:
            print(f'{HEADER_PATH} 가 DBC 와 다르다. gen_com_cfg.py 를 실행할 것.', file=sys.stderr)
            return 1
        return 0

    HEADER_PATH.write_text(text)
    return 0


if __name__ == '__main__':
    sys.exit(main())
