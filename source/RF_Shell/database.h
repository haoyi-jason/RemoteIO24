#ifndef _DATABASE_H
#define _DATABASE_H

#include "hal.h"

#define OFFSET_NVM_BOARD        0
#define OFFSET_NVM_RF           OFFSET_NVM_BOARD + SZ_NVM_BOARD



struct map_u16{
  uint16_t x;
  uint16_t y;
};

struct rf_device_config_s{
  uint16_t selfAddr;
  uint16_t destAddr;
  uint8_t opMode;
  uint16_t txIntervalMs;
  uint16_t timeoutMs;
  uint16_t txPattern;
  uint16_t txMask;
  uint8_t dataRate;
  float frequency;
  uint16_t txPeriod;
  uint8_t txPower;
};


struct nvmParam_s{
  uint16_t crc16;
  uint32_t board_version;
  uint32_t fw_version;
  struct rf_device_config_s device_config;
  struct map_u16 map_a[4];     // for stepper and servo
  struct map_u16 map_b[4];     // for valve
  uint16_t min_pps;
  uint16_t max_pps;
  float pulseDegree;
  
};

extern struct nvmParam_s nvmParam2;

void database_init(void);
#endif