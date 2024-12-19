#include "ch.h"
#include "hal.h"
#include "bc3601_def.h"
#include "bc3601.h"



uint8_t registerRead(BC3601Driver *devp, uint8_t regAddr, uint8_t *d,uint8_t n)
{
  if(devp == NULL) return 0xff;
  
  
//  uint8_t reg = CMD_REG_READ(regAddr);
  uint8_t reg = regAddr;
  spiAcquireBus(devp->config->spip);
  spiStart(devp->config->spip,devp->config->spicfg);
  palClearPad(devp->config->ssport,devp->config->ssline);
  spiSend(devp->config->spip,1,&reg);

  palSetPadMode(GPIOB,15,PAL_MODE_INPUT);
  spiReceive(devp->config->spip,n,d);
  palSetPadMode(GPIOB,15,PAL_MODE_STM32_ALTERNATE_PUSHPULL);

  palSetPad(devp->config->ssport,devp->config->ssline);
  spiStop(devp->config->spip);
  spiReleaseBus(devp->config->spip);

  return 0;
}


uint8_t registerWrite(BC3601Driver *devp, uint8_t regAddr, uint8_t *d, uint16_t n)
{
  if(devp == NULL) return 0xFF;
  
//  uint8_t reg = CMD_REG_WRITE(regAddr);
  uint8_t reg = regAddr;

  spiAcquireBus(devp->config->spip);
  spiStart(devp->config->spip,devp->config->spicfg);
  palClearPad(devp->config->ssport,devp->config->ssline);  
  spiSend(devp->config->spip,1,&reg);
  if(d != NULL)
    spiSend(devp->config->spip,n,d);
  palSetPad(devp->config->ssport,devp->config->ssline);
  spiStop(devp->config->spip);
  spiReleaseBus(devp->config->spip);  
  
  return 0;
}

static uint8_t registerXfer(BC3601Driver *devp,uint8_t *tx, uint16_t txn, uint8_t *rx, uint16_t rxn)
{
  spiAcquireBus(devp->config->spip);
  spiStart(devp->config->spip,devp->config->spicfg);
  palClearPad(devp->config->ssport,devp->config->ssline);
  
  if(tx != NULL){
    spiSend(devp->config->spip,txn,tx);
  }
  if(rx != NULL){
    spiReceive(devp->config->spip,rxn,rx);
  }
  palSetPad(devp->config->ssport,devp->config->ssline);
  spiStop(devp->config->spip);
  spiReleaseBus(devp->config->spip);  
  return 0;
}

uint8_t cmdStrobe(BC3601Driver *devp, uint8_t cmd)
{
  
  spiAcquireBus(devp->config->spip);
  spiStart(devp->config->spip,devp->config->spicfg);
  palClearPad(devp->config->ssport,devp->config->ssline);  
  spiSend(devp->config->spip,1,&cmd);
  palSetPad(devp->config->ssport,devp->config->ssline);
  spiStop(devp->config->spip);
  spiReleaseBus(devp->config->spip);  
  
  return 0;
}



void bc3601WriteRegister(BC3601Driver *dev, uint8_t regAddr, uint8_t *v)
{
  registerWrite(dev,regAddr,v,1);
}

void bc3601ReadRegister(BC3601Driver *dev, uint8_t regAddr, uint8_t *v)
{
  registerRead(dev,regAddr,v,1);
}

void bc3601StrobeCommand(BC3601Driver *dev, uint8_t cmd)
{
  cmdStrobe(dev,cmd);
}

void bc3601_default_mode(BC3601Driver *dev)
{
  BC3601_LITE_SLEEP(dev);  
}


msg_t bc3601ObjectInit(BC3601Driver *dev)
{
  return 0;
  
}


//    GIO Driving
//			<0x00=>	 0.5mA
//			<0x40=>  1mA
//			<0x80=>  5mA
//			<0xC0=>  10mA
#define _GIO_DRIVING_  	(0xC0)

msg_t bc3601Start(BC3601Driver *dev, const bc3601_config_t *config)
{
  dev->config = config;
  
    return 0;

  
}

msg_t bc3601Stop(BC3601Driver *dev)
{
    return 0;

}

msg_t bc3601SetOM(BC3601Driver *dev)
{
  // set to bank 0
  BC3601_SETBANK(dev,0);
  
  uint8_t config = 0x6;
  
  config = 0x03;
  BC3601_REG_WRITE(dev,BC_PKT1,&config);
  
  // read pkt2 and set RX premble
  BC3601_REG_READ(dev,BC_PKT2,&config);
  config &= ~0x0F;
  // rx premble = 4 bytes, synclen = 4-bytes
  config |= (0x01 << 2) | 0x03;
  BC3601_REG_WRITE(dev,BC_PKT2,&config);
    return 0;

}

/* exported function */

void bc3601_refresh_registers(BC3601Driver *dev)
{
  //return;
  uint8_t *ptr;
  BC3601_SETBANK(dev,REGS_BANK0);
  ptr = (uint8_t*)&dev->reg_cmd;
  for(uint8_t i=0;i<0x1f;i++){
    if(i == 0x05) continue;
    BC3601_REG_READ(dev,i,ptr++);
  }
  ptr = (uint8_t*)&dev->reg_bk0;
  for(uint8_t i=0x20;i<0x40;i++){
    if(i == 0x21) continue;
    if(i == 0x27) continue;
    if((i >= 0x36) && (i <=0x3b)) continue;
    BC3601_REG_READ(dev,i,ptr++);
  }
  
  BC3601_SETBANK(dev,REGS_BANK1);
  ptr = (uint8_t*)&dev->reg_bk1;
  for(uint8_t i=0x21;i<0x3f;i++){
    if(i == 0x25) continue;
    if((i >= 0x27) && (i <=0x2b)) continue;

    BC3601_REG_READ(dev,i,ptr++);
  }
  BC3601_SETBANK(dev,REGS_BANK0);
}
