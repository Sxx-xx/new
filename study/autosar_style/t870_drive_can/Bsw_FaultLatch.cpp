#include "Bsw_FaultLatch.h"
#include "Mcal_Uart.h"


static bool steerFaultLatched = false;
static bool steerStallLatched = false;
static bool e2eFaultLatched   = false;


void Bsw_FaultLatch_SetSteerFault() { steerFaultLatched = true; }
void Bsw_FaultLatch_SetSteerStall() { steerStallLatched = true; }
bool Bsw_FaultLatch_IsSteerFault()  { return steerFaultLatched; }
bool Bsw_FaultLatch_IsSteerStall()  { return steerStallLatched; }
void Bsw_FaultLatch_SetE2eFault()   { e2eFaultLatched = true; }
bool Bsw_FaultLatch_IsE2eFault()    { return e2eFaultLatched; }


void Bsw_FaultLatch_ClearAll()
{
  steerFaultLatched = false;
  steerStallLatched = false;
  e2eFaultLatched   = false;

  Mcal_Uart_WriteText("LATCH CLEARED");
  Mcal_Uart_WriteNewline();
}
