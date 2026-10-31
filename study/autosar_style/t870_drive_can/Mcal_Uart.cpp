#include <Arduino.h>
#include "Mcal_Uart.h"


void Mcal_Uart_Init()
{
  Serial.begin(115200);
}


int Mcal_Uart_Available()
{
  return Serial.available();
}


int Mcal_Uart_Read()
{
  return Serial.read();
}


void Mcal_Uart_WriteText(const char *text)
{
  Serial.print(text);
}


void Mcal_Uart_WriteLong(long value)
{
  Serial.print(value);
}


void Mcal_Uart_WriteULong(unsigned long value)
{
  Serial.print(value);
}


void Mcal_Uart_WriteFloat(float value, uint8_t digits)
{
  Serial.print(value, digits);
}


void Mcal_Uart_WriteNewline()
{
  Serial.println();
}
