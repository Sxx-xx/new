// ============================================================
// 수학 라이브러리 (AUTOSAR 의 Mfl/Mfx 라이브러리 자리)
//
// SWC 는 Arduino API 를 부를 수 없으므로 map / constrain / abs /
// min / max 를 여기서 다시 정의한다. 계산식은 Arduino AVR 코어
// 1.8.x 의 WMath.cpp, Arduino.h 매크로와 같다. 결과가 원본과
// 비트 단위로 같아야 하므로 식을 바꾸지 않는다.
// ============================================================

#ifndef LIB_MATH_H
#define LIB_MATH_H

#include "Std_Types.h"

// Arduino map(): (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
static inline long Lib_Map(long x, long inMin, long inMax, long outMin, long outMax)
{
  return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

static inline long Lib_ConstrainL(long amt, long low, long high)
{
  return (amt < low) ? low : ((amt > high) ? high : amt);
}

static inline float Lib_ConstrainF(float amt, float low, float high)
{
  return (amt < low) ? low : ((amt > high) ? high : amt);
}

static inline int Lib_AbsI(int x)
{
  return (x > 0) ? x : -x;
}

static inline float Lib_MinF(float a, float b)
{
  return (a < b) ? a : b;
}

static inline float Lib_MaxF(float a, float b)
{
  return (a > b) ? a : b;
}

#endif
