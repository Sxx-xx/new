// ============================================================
// Com_Cfg.h 와 E2E_Crc8 를 cantools 와 대조하는 작은 도구 (test_can.py 가 쓴다)
//
//   stdin 한 줄마다 stdout 한 줄:
//     pack <메시지> <raw 값 ...>   -> 16진수 8 바이트
//     unpack <메시지> <16진수>     -> raw 값 (DBC 신호 순서)
//     crc <16진수> <DataID 10진수> -> CRC8 10진수
// ============================================================

#include <stdio.h>
#include <string.h>

#include <iostream>
#include <sstream>
#include <string>

#include "Com_Cfg.h"
#include "E2E.h"


static std::string toHex(const uint8_t *data, size_t length)
{
  std::string out;
  char text[3];
  for (size_t i = 0; i < length; ++i)
  {
    snprintf(text, sizeof(text), "%02X", data[i]);
    out += text;
  }
  return out;
}


static size_t fromHex(const std::string &hex, uint8_t *data)
{
  size_t length = hex.size() / 2;
  for (size_t i = 0; i < length; ++i)
  {
    data[i] = (uint8_t)std::stoi(hex.substr(i * 2, 2), nullptr, 16);
  }
  return length;
}


static std::string packLine(const std::string &name, std::istringstream &in)
{
  uint8_t data[8] = {0};
  long v[10] = {0};
  for (long &value : v) in >> value;

  if (name == "DriveCmd")
  {
    Com_DriveCmdType s;
    s.TargetSpeed_kmh = (int16_t)v[0]; s.SteerTargetPot = (uint16_t)v[1]; s.Mode = (uint8_t)v[2];
    s.OpenLoopSel = (uint8_t)v[3]; s.Alive = (uint8_t)v[4]; s.OpenLoopDuty = (int16_t)v[5];
    s.WatchdogDisable = v[6]; s.LatchClearReq = v[7]; s.CRC8 = (uint8_t)v[8];
    Com_Pack_DriveCmd(s, data);
  }
  else if (name == "DriveStatus")
  {
    Com_DriveStatusType s;
    s.MeasuredSpeed_kmh = (int16_t)v[0]; s.SteerPot = (uint16_t)v[1]; s.DrivePwm = (uint8_t)v[2];
    s.SteerPwm = (uint8_t)v[3]; s.FaultSteer = v[4]; s.FaultStall = v[5]; s.FaultWatchdog = v[6];
    s.FaultE2E = v[7]; s.Alive = (uint8_t)v[8]; s.CRC8 = (uint8_t)v[9];
    Com_Pack_DriveStatus(s, data);
  }
  else
  {
    Com_DriveDiagType s;
    s.CrcErrorCount = (uint16_t)v[0]; s.AliveJumpCount = (uint16_t)v[1];
    s.AliveRepeatCount = (uint8_t)v[2]; s.TimeoutCount = (uint8_t)v[3];
    s.E2EState = (uint8_t)v[4]; s.Alive = (uint8_t)v[5]; s.CRC8 = (uint8_t)v[6];
    Com_Pack_DriveDiag(s, data);
  }
  return toHex(data, sizeof(data));
}


static std::string unpackLine(const std::string &name, const std::string &hex)
{
  uint8_t data[8] = {0};
  fromHex(hex, data);
  std::ostringstream out;

  if (name == "DriveCmd")
  {
    Com_DriveCmdType s;
    Com_Unpack_DriveCmd(data, s);
    out << s.TargetSpeed_kmh << ' ' << s.SteerTargetPot << ' ' << (int)s.Mode << ' '
        << (int)s.OpenLoopSel << ' ' << (int)s.Alive << ' ' << s.OpenLoopDuty << ' '
        << (int)s.WatchdogDisable << ' ' << (int)s.LatchClearReq << ' ' << (int)s.CRC8;
  }
  else if (name == "DriveStatus")
  {
    Com_DriveStatusType s;
    Com_Unpack_DriveStatus(data, s);
    out << s.MeasuredSpeed_kmh << ' ' << s.SteerPot << ' ' << (int)s.DrivePwm << ' '
        << (int)s.SteerPwm << ' ' << (int)s.FaultSteer << ' ' << (int)s.FaultStall << ' '
        << (int)s.FaultWatchdog << ' ' << (int)s.FaultE2E << ' ' << (int)s.Alive << ' ' << (int)s.CRC8;
  }
  else
  {
    Com_DriveDiagType s;
    Com_Unpack_DriveDiag(data, s);
    out << s.CrcErrorCount << ' ' << s.AliveJumpCount << ' ' << (int)s.AliveRepeatCount << ' '
        << (int)s.TimeoutCount << ' ' << (int)s.E2EState << ' ' << (int)s.Alive << ' ' << (int)s.CRC8;
  }
  return out.str();
}


int main()
{
  std::string line;
  while (std::getline(std::cin, line))
  {
    std::istringstream in(line);
    std::string command;
    in >> command;

    if (command == "pack")
    {
      std::string name;
      in >> name;
      std::cout << packLine(name, in) << std::endl;
    }
    else if (command == "unpack")
    {
      std::string name, hex;
      in >> name >> hex;
      std::cout << unpackLine(name, hex) << std::endl;
    }
    else if (command == "crc")
    {
      std::string hex;
      int dataId;
      in >> hex >> dataId;
      uint8_t data[32];
      const size_t length = fromHex(hex, data);
      std::cout << (int)E2E_Crc8(data, (uint8_t)length, (uint8_t)dataId) << std::endl;
    }
  }
  return 0;
}
