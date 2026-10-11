#include "Bsw_FaultLatch.h"
#include "Mcal_Uart.h"


static bool steerFaultLatched = false;
static bool steerStallLatched = false;


void Bsw_FaultLatch_SetSteerFault() { steerFaultLatched = true; }
void Bsw_FaultLatch_SetSteerStall() { steerStallLatched = true; }
bool Bsw_FaultLatch_IsSteerFault()  { return steerFaultLatched; }
bool Bsw_FaultLatch_IsSteerStall()  { return steerStallLatched; }


void Bsw_FaultLatch_ClearAll()
{
  steerFaultLatched = false;
  steerStallLatched = false;

  Mcal_Uart_WriteText("LATCH CLEARED");
  Mcal_Uart_WriteNewline();
}
