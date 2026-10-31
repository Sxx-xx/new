#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>

#include "Mcal_Can.h"
#include "Mcal_Cfg.h"


#if MCAL_CAN_CRYSTAL_MHZ == 8
static const CAN_CLOCK CAN_CRYSTAL = MCP_8MHZ;
#elif MCAL_CAN_CRYSTAL_MHZ == 16
static const CAN_CLOCK CAN_CRYSTAL = MCP_16MHZ;
#else
#error "MCAL_CAN_CRYSTAL_MHZ must be 8 or 16"
#endif


static MCP2515 *controller = nullptr;


// [주의] MCP2515 객체를 전역으로 두면 안 된다.
// 생성자가 SPI.begin() 을 부르고, SPI.begin() 은 SS(D10) 가 아직 입력이면
// HIGH 를 쓴 뒤 출력으로 바꾼다. D10 은 구동 PWM 이므로 전역 생성자에서
// 이 일이 일어나면 setup() 전까지 구동 모터가 duty 100% 를 받는다.
// Mcal_Port_Init() 이 D10 을 출력 LOW 로 잡은 뒤에 여기서 만든다.
bool Mcal_Can_Init()
{
  static MCP2515 instance(MCAL_PIN_CAN_CS);

  pinMode(MCAL_PIN_CAN_INT, INPUT_PULLUP);

  if (instance.reset() != MCP2515::ERROR_OK
      || instance.setBitrate(CAN_500KBPS, CAN_CRYSTAL) != MCP2515::ERROR_OK
      || instance.setNormalMode() != MCP2515::ERROR_OK)
  {
    return false;
  }

  controller = &instance;
  return true;
}


bool Mcal_Can_Read(Mcal_Can_FrameType &frame)
{
  // INT 가 HIGH 면 받을 것이 없다. loop 마다 SPI 를 돌리지 않으려고 먼저 본다.
  if (controller == nullptr || digitalRead(MCAL_PIN_CAN_INT) == HIGH)
  {
    return false;
  }

  struct can_frame raw;
  if (controller->readMessage(&raw) != MCP2515::ERROR_OK)
  {
    // 수신이 아닌 오류 인터럽트로 INT 가 내려간 경우다. 수신 플래그는 건드리지
    // 않도록 clearInterrupts() 대신 오류 플래그만 지운다.
    controller->clearRXnOVRFlags();
    controller->clearERRIF();
    controller->clearMERR();
    return false;
  }

  if (raw.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG))
  {
    return false;
  }

  frame.id  = (uint16_t)(raw.can_id & CAN_SFF_MASK);
  frame.dlc = raw.can_dlc;
  memcpy(frame.data, raw.data, sizeof(frame.data));
  return true;
}


bool Mcal_Can_Write(const Mcal_Can_FrameType &frame)
{
  if (controller == nullptr)
  {
    return false;
  }

  struct can_frame raw;
  raw.can_id  = frame.id;
  raw.can_dlc = frame.dlc;
  memcpy(raw.data, frame.data, sizeof(frame.data));
  return controller->sendMessage(&raw) == MCP2515::ERROR_OK;
}
