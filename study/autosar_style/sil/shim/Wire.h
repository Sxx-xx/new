// PC용 가짜 Wire.h. 원본이 쓰는 마스터 수신 API 만 있다.

#ifndef SIL_FAKE_WIRE_H
#define SIL_FAKE_WIRE_H

#include "Arduino.h"

class TwoWire
{
public:
  void    begin();
  uint8_t requestFrom(int address, int quantity);
  int     available();
  int     read();
};

extern TwoWire Wire;

#endif
