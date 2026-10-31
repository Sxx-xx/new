// ============================================================
// MCAL Uart: USB 시리얼 (115200bps). 이 버전에서는 디버그 출력만 쓴다.
// ============================================================

#ifndef MCAL_UART_H
#define MCAL_UART_H

#include "Std_Types.h"

void Mcal_Uart_Init();
int  Mcal_Uart_Available();
int  Mcal_Uart_Read();

void Mcal_Uart_WriteText(const char *text);
void Mcal_Uart_WriteLong(long value);
void Mcal_Uart_WriteULong(unsigned long value);
void Mcal_Uart_WriteFloat(float value, uint8_t digits);
void Mcal_Uart_WriteNewline();

#endif
