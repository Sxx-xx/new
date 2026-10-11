#include <ctype.h>
#include <stdlib.h>

#include "Cdd_SerialCmd.h"
#include "Mcal_Uart.h"
#include "Mcal_Gpt.h"
#include "Bsw_Watchdog.h"
#include "Rte.h"


// [원본 주석] Serial.parseFloat() 는 종결 문자가 없으면 최대 1초
//             블로킹한다. 그동안 조향 제어까지 멈춘다. 그래서 줄 단위
//             비블로킹 버퍼로 받는다.
static char    cmdBuffer[16];
static uint8_t cmdLength = 0;


static void handleLine(char *s)
{
  // 알 수 없는 명령이어도 줄이 들어왔으면 워치독을 갱신한다 (원본과 같다).
  Bsw_Watchdog_Kick(Mcal_Gpt_GetMs());

  Rte_SerialCmdType cmd;
  cmd.intValue   = 0;
  cmd.floatValue = 0.0;

  if ((s[0] == 'e' || s[0] == 'E') && s[1] == '\0')
  {
    cmd.kind = RTE_CMD_EMERGENCY_STOP;
  }
  else if ((s[0] == 'h' || s[0] == 'H') && s[1] == '\0')
  {
    cmd.kind = RTE_CMD_MISSION_HOLD;
  }
  else if (s[0] == 'b' || s[0] == 'B')
  {
    cmd.kind = RTE_CMD_WATCHDOG_TOGGLE;
  }
  else if (s[0] == 'r' || s[0] == 'R')
  {
    cmd.kind = RTE_CMD_LATCH_CLEAR;
  }
  else if (s[0] == 's' || s[0] == 'S')
  {
    cmd.kind = RTE_CMD_STEER_OPEN_LOOP;
    cmd.intValue = atoi(s + 1);
  }
  else if (s[0] == 'p' || s[0] == 'P')
  {
    cmd.kind = RTE_CMD_DRIVE_OPEN_LOOP;
    cmd.intValue = atoi(s + 1);
  }
  else if (isdigit(s[0]) || s[0] == '-' || s[0] == '+' || s[0] == '.')
  {
    cmd.kind = RTE_CMD_TARGET_SPEED;
    cmd.floatValue = atof(s);
  }
  else
  {
    Mcal_Uart_WriteText("? unknown cmd");
    Mcal_Uart_WriteNewline();
    return;
  }

  Rte_Notify_SerialCommand(cmd);
}


void Cdd_SerialCmd_MainFunction()
{
  while (Mcal_Uart_Available() > 0)
  {
    char c = Mcal_Uart_Read();

    if (c == '\n' || c == '\r')
    {
      if (cmdLength > 0)
      {
        cmdBuffer[cmdLength] = '\0';
        handleLine(cmdBuffer);
        cmdLength = 0;
      }
    }
    else if (cmdLength < sizeof(cmdBuffer) - 1)
    {
      cmdBuffer[cmdLength++] = c;
    }
  }
}
