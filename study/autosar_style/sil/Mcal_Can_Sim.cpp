// ============================================================
// PC용 Mcal_Can 구현. t870_drive_can/Mcal_Can_Mcp2515.cpp 를 대신한다.
// ============================================================

#include <errno.h>
#include <fcntl.h>
#include <net/if.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <deque>
#include <string>

#include "sim_core.h"
#include "Mcal_Can.h"
#include "Mcal_Can_Sim.h"


struct TimedFrame
{
  unsigned long tMs;
  Mcal_Can_FrameType frame;
};

static std::string busName;
static std::string framesPath;
static FILE *txLog = nullptr;
static unsigned long durationOverrideMs = 0;

static int canSocket = -1;
static struct timespec startTime;

static std::deque<TimedFrame> pendingFrames;           // mem: 아직 시각이 안 된 프레임
static std::deque<Mcal_Can_FrameType> memRxQueue;      // mem: 펌웨어가 읽을 프레임

static const int START_WAIT_TIMEOUT_MS = 15000;


static bool isMemBus()
{
  return busName == "mem";
}


// 한 줄 형식: <t_ms> <id 16진수> <dlc> <데이터 16진수>
static bool loadFrames(const std::string &path)
{
  FILE *file = fopen(path.c_str(), "r");
  if (!file)
  {
    perror(path.c_str());
    return false;
  }

  unsigned long tMs;
  unsigned int id, dlc;
  char hex[32];
  while (fscanf(file, "%lu %x %u %31s", &tMs, &id, &dlc, hex) == 4)
  {
    TimedFrame item;
    item.tMs = tMs;
    item.frame.id = (uint16_t)id;
    item.frame.dlc = (uint8_t)dlc;
    memset(item.frame.data, 0, sizeof(item.frame.data));
    for (unsigned int i = 0; i < dlc && i < sizeof(item.frame.data); ++i)
    {
      unsigned int byteValue;
      sscanf(hex + i * 2, "%2x", &byteValue);
      item.frame.data[i] = (uint8_t)byteValue;
    }
    pendingFrames.push_back(item);
  }
  fclose(file);
  return true;
}


static bool openSocket(const std::string &name)
{
  canSocket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (canSocket < 0)
  {
    perror("socket(PF_CAN)");
    return false;
  }

  struct ifreq ifr;
  memset(&ifr, 0, sizeof(ifr));
  strncpy(ifr.ifr_name, name.c_str(), IFNAMSIZ - 1);
  if (ioctl(canSocket, SIOCGIFINDEX, &ifr) < 0)
  {
    perror(name.c_str());
    return false;
  }

  struct sockaddr_can addr;
  memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(canSocket, (struct sockaddr *)&addr, sizeof(addr)) < 0)
  {
    perror("bind");
    return false;
  }

  fcntl(canSocket, F_SETFL, O_NONBLOCK);
  return true;
}


bool SimCan_ParseArgs(int argc, char **argv)
{
  busName = "mem";
  for (int i = 0; i < argc; ++i)
  {
    if (strncmp(argv[i], "--bus=", 6) == 0)              busName = argv[i] + 6;
    else if (strncmp(argv[i], "--frames=", 9) == 0)      framesPath = argv[i] + 9;
    else if (strncmp(argv[i], "--duration-ms=", 14) == 0) durationOverrideMs = strtoul(argv[i] + 14, nullptr, 10);
    else if (strncmp(argv[i], "--tx-log=", 9) == 0)
    {
      txLog = fopen(argv[i] + 9, "w");
      if (!txLog)
      {
        perror(argv[i] + 9);
        return false;
      }
    }
    else
    {
      fprintf(stderr, "unknown option: %s\n", argv[i]);
      return false;
    }
  }

  if (isMemBus())
  {
    return framesPath.empty() || loadFrames(framesPath);
  }
  return openSocket(busName);
}


unsigned long SimCan_Duration(unsigned long scenarioMs)
{
  return durationOverrideMs ? durationOverrideMs : scenarioMs;
}


bool SimCan_WaitForStart()
{
  if (!isMemBus())
  {
    // 프레임은 소켓에 남겨 둔다. 펌웨어가 첫 loop 에서 읽는다.
    struct pollfd waiter = {canSocket, POLLIN, 0};
    if (poll(&waiter, 1, START_WAIT_TIMEOUT_MS) <= 0)
    {
      fprintf(stderr, "no CAN frame on %s within %d ms\n", busName.c_str(), START_WAIT_TIMEOUT_MS);
      return false;
    }
    clock_gettime(CLOCK_MONOTONIC, &startTime);
  }
  return true;
}


void SimCan_BeforeLoop(unsigned long t)
{
  if (isMemBus())
  {
    while (!pendingFrames.empty() && pendingFrames.front().tMs <= t)
    {
      memRxQueue.push_back(pendingFrames.front().frame);
      pendingFrames.pop_front();
    }
    return;
  }

  struct timespec deadline = startTime;
  deadline.tv_sec  += (time_t)(t / 1000);
  deadline.tv_nsec += (long)(t % 1000) * 1000000L;
  if (deadline.tv_nsec >= 1000000000L)
  {
    deadline.tv_sec++;
    deadline.tv_nsec -= 1000000000L;
  }
  clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &deadline, nullptr);
}


void SimCan_Close()
{
  if (txLog)
  {
    fclose(txLog);
    txLog = nullptr;
  }
  if (canSocket >= 0)
  {
    close(canSocket);
    canSocket = -1;
  }
}


bool Mcal_Can_Init()
{
  return isMemBus() || canSocket >= 0;
}


bool Mcal_Can_Read(Mcal_Can_FrameType &frame)
{
  if (isMemBus())
  {
    if (memRxQueue.empty())
    {
      return false;
    }
    frame = memRxQueue.front();
    memRxQueue.pop_front();
    return true;
  }

  struct can_frame raw;
  const ssize_t length = read(canSocket, &raw, sizeof(raw));
  if (length != (ssize_t)sizeof(raw)
      || (raw.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG)))
  {
    return false;
  }

  frame.id  = (uint16_t)(raw.can_id & CAN_SFF_MASK);
  frame.dlc = raw.len;
  memcpy(frame.data, raw.data, sizeof(frame.data));
  return true;
}


bool Mcal_Can_Write(const Mcal_Can_FrameType &frame)
{
  if (!isMemBus())
  {
    struct can_frame raw;
    memset(&raw, 0, sizeof(raw));
    raw.can_id = frame.id;
    raw.len = frame.dlc;
    memcpy(raw.data, frame.data, sizeof(raw.data));
    if (write(canSocket, &raw, sizeof(raw)) != (ssize_t)sizeof(raw))
    {
      return false;   // 큐가 차면 CanIf 가 다음 loop 에 다시 보낸다
    }
  }

  if (txLog)
  {
    fprintf(txLog, "%lu %03X %u ", sim::nowMs, frame.id, frame.dlc);
    for (unsigned int i = 0; i < frame.dlc; ++i)
    {
      fprintf(txLog, "%02X", frame.data[i]);
    }
    fputc('\n', txLog);
  }
  return true;
}
