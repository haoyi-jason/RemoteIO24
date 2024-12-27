#ifndef _BC3601_
#define _BC3601_
#include "hal.h"
#include "bc3601_def.h"



typedef enum
{
  BC3601_UNINIT = 0,
  BC3601_STOP = 1,
  BC3601_READY = 2,
}bc3601_state_t;

typedef struct{
  SPIDriver *spip;
  const SPIConfig *spicfg;
  ioportid_t ssport;
  ioportmask_t ssline;
}bc3601_config_t;

struct reg_cmd{
  uint8_t cfg1;
  uint8_t rc1;
  uint8_t irq[3];
  uint8_t io[3];
  uint8_t fifo[2];
  uint8_t pkt[9];
  uint8_t mod[3];
  uint8_t dm[8];
};

struct reg_bk0{
  uint8_t om;
  uint8_t sx[4];
  uint8_t sta1;
  uint8_t rssi[3];
  uint8_t atr[11];
  uint8_t xo[3];
  uint8_t tx2;
};

struct reg_bk1{
  uint8_t agc[5];
  uint8_t fcf[19]; 
};


typedef struct BC3601Driver BC3601Driver;

struct BC3601Driver{
  bc3601_state_t state;
  const bc3601_config_t *config;
  uint8_t txPower;
  uint8_t dataRate;
  uint16_t destAddr;
  float frequency;
  struct reg_cmd reg_cmd;
  struct reg_bk0 reg_bk0;
  struct reg_bk1 reg_bk1;

};

#define CMDO(dev,x)     registerWrite(dev,x,NULL,0)

#define BC3601_SETBANK(dev,x)           cmdStrobe(dev,0x20 | x)
#define BC3601_RESET_CHIP(dev)          cmdStrobe(dev,SOFT_RESET_CMD)
#define BC3601_RESET_TXFIFO(dev)        cmdStrobe(dev,REST_TX_POS_CMD)
#define BC3601_RESET_RXFIFO(dev)        cmdStrobe(dev,REST_RX_POS_CMD)
#define BC3601_DEEP_SLEEP(dev)          cmdStrobe(dev,DEEP_SLEEP_CMD)
#define BC3601_IDLE(dev)                cmdStrobe(dev,IDLE_MODE_CMD)
#define BC3601_LITE_SLEEP(dev)          cmdStrobe(dev,LIGHT_SLEEP_CMD)
#define BC3601_STBY(dev)                cmdStrobe(dev,STANDBY_MODE_CMD)
#define BC3601_TX_MODE(dev)             cmdStrobe(dev,TX_MODE_CMD)
#define BC3601_RX_MODE(dev)             cmdStrobe(dev,RX_MODE_CMD)

#define BC3601_REG_WRITE(dev,x,r)       registerWrite(dev,0x40 | x,r,1)
#define BC3601_REG_READ(dev,x,r)        registerRead(dev,0xC0 | x,r,1)
#define BC3601_REGS_WRITE(dev,x,r,n)       registerWrite(dev,0x40 | x,r,n)
#define BC3601_REGS_READ(dev,x,r,n)        registerRead(dev,0xC0 | x,r,n)
#define BC3601_FIFO_WRITE(dev,r,n)      registerWrite(dev,0x11,r,n)
#define BC3601_FIFO_READ(dev,r,n)       registerRead(dev,0x91,r,n)
#define BC3601_SYNC_WRITE(dev,r,n)      registerWrite(dev,0x10,r,n)
#define BC3601_SYNC_READ(dev,r,n)       registerWrite(dev,0x90,r,n)

#define BC3601_SET_TX_PAYLOAD_WIDTH(dev,x) registerWrite(dev,TX_DATA_LENG_REGS | 0x40,x,1)
#define BC3601_SET_RX_PAYLOAD_WIDTH(dev,x) registerWrite(dev,RX_DATA_LENG_REGS | 0x40,x,1)

#define BC3601_SET_TX_PAYLOAD_SADDR(dev,x) registerWrite(dev,TX_FIFO_SA_REGS | 0x40,x,1)

void bc3601_parameter_initialize(BC3601Driver *dev);
void bc3601WriteRegister(BC3601Driver *dev, uint8_t regAddr, uint8_t *v);
void bc3601ReadRegister(BC3601Driver *dev, uint8_t regAddr, uint8_t *v);

uint8_t registerRead(BC3601Driver *devp, uint8_t regAddr, uint8_t *d,uint8_t n);
uint8_t registerWrite(BC3601Driver *devp, uint8_t regAddr, uint8_t *d, uint16_t n);
uint8_t cmdStrobe(BC3601Driver *devp, uint8_t cmd);
void bc3601_refresh_registers(BC3601Driver *dev);
uint8_t bc3601_readRSSI(BC3601Driver *dev);
#endif