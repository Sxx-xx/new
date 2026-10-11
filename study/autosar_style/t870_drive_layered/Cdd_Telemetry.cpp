#include "Cdd_Telemetry.h"
#include "Mcal_Uart.h"
#include "Mcal_Adc.h"
#include "Mcal_Icu.h"
#include "Rte.h"


static void writeLine(const char *text)
{
  Mcal_Uart_WriteText(text);
  Mcal_Uart_WriteNewline();
}


void Cdd_Telemetry_PrintBanner()
{
  Mcal_Uart_WriteNewline();
  writeLine("==========================================");
  writeLine("T870 Steering + Drive");
  writeLine("==========================================");
  Mcal_Uart_WriteText("Steer center = ");
  Mcal_Uart_WriteLong(Rte_Prm_SteerPotCenter());
  Mcal_Uart_WriteNewline();
  Mcal_Uart_WriteText("Soft limit   = ");
  Mcal_Uart_WriteLong(Rte_Prm_SteerPotSoftMin());
  Mcal_Uart_WriteText(" ~ ");
  Mcal_Uart_WriteLong(Rte_Prm_SteerPotSoftMax());
  Mcal_Uart_WriteNewline();
  Mcal_Uart_WriteText("Max speed    = ");
  Mcal_Uart_WriteFloat(Rte_Prm_MaxSpeedKmh(), 1);
  writeLine(" km/h");
  Mcal_Uart_WriteNewline();
  writeLine("Serial Monitor line ending must be 'Newline'.");
  writeLine("cmd: <speed> | e(emergency stop) | p<duty> | s<duty> | b | r");
  Mcal_Uart_WriteNewline();
}


void Cdd_Telemetry_MainFunction()
{
  Mcal_Uart_WriteText("RC:");
  if (!Rte_Read_Remote_Valid())
  {
    Mcal_Uart_WriteText("LOST");
  }
  else
  {
    Mcal_Uart_WriteText(Rte_Read_Remote_AutoMode() ? "AUTO" : "MANUAL");
  }
  Mcal_Uart_WriteText(" TH:");
  Mcal_Uart_WriteULong(Rte_Read_Remote_ThrottleRaw());
  Mcal_Uart_WriteText("/");
  Mcal_Uart_WriteLong(Rte_Read_Remote_ThrottleCommand());
  Mcal_Uart_WriteText(" ST:");
  Mcal_Uart_WriteULong(Rte_Read_Remote_SteeringRaw());
  Mcal_Uart_WriteText("/");
  Mcal_Uart_WriteLong(Rte_Read_Remote_SteeringCommand());
  Mcal_Uart_WriteText(" | ");

  Mcal_Uart_WriteText("POT:");
  Mcal_Uart_WriteLong(Mcal_Adc_ReadSteerPot());

  Mcal_Uart_WriteText(" | SteerPWM:");
  if (Rte_Read_PixhawkPwmValid())
  {
    Mcal_Uart_WriteULong(Rte_Read_PixhawkPwmUs());
  }
  else
  {
    Mcal_Uart_WriteText("LOST");
  }

  Mcal_Uart_WriteText(" | SteerTarget:");
  Mcal_Uart_WriteLong(Rte_Read_SteerTarget());

  Mcal_Uart_WriteText(" | Tgt:");
  Mcal_Uart_WriteFloat(Rte_Read_RampedSpeedKmh(), 2);

  Mcal_Uart_WriteText(" | Act:");
  Mcal_Uart_WriteFloat(Rte_Read_ActualSpeedKmh(), 2);

  Mcal_Uart_WriteText(" | C/100ms:");
  Mcal_Uart_WriteLong(Rte_Read_EncoderCountDiff());

  Mcal_Uart_WriteText(" | Cnt:");
  Mcal_Uart_WriteLong(Mcal_Icu_GetEncoderCountUnprotected());

  Mcal_Uart_WriteText(" | PWM:");
  Mcal_Uart_WriteLong(Rte_Read_DrivePwm());

  if (Rte_Read_OpenLoopMode())          Mcal_Uart_WriteText(" | DRIVE-OPEN");
  if (Rte_Read_SteerOpenLoop())         Mcal_Uart_WriteText(" | STEER-OPEN");
  if (!Rte_Read_Watchdog_Enabled())     Mcal_Uart_WriteText(" | WDOG-OFF");
  if (Rte_Read_FaultLatch_SteerFault()) Mcal_Uart_WriteText(" | STEER-FAULT");
  if (Rte_Read_FaultLatch_SteerStall()) Mcal_Uart_WriteText(" | STEER-STALL");

  Mcal_Uart_WriteNewline();
}
