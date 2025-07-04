#include "ch.h"
#include "hal.h"
#include "ad5522_dev.h"
#include "ad5522_def.h"
#include <string.h>

#define DBG_CHECK(x)    (x == 0)

const uint8_t dac_address_list[NOF_DAC_ADDRESS] = {
  DAC_ADDRESS_OFFSET_X,
  DAC_ADDRESS_I_5UA ,
  DAC_ADDRESS_I_20UA,
  DAC_ADDRESS_I_200UA,
  DAC_ADDRESS_I_2MA,
  DAC_ADDRESS_I_EXT,
  DAC_ADDRESS_V,
  DAC_ADDRESS_I_CLL,
  DAC_ADDRESS_V_CLL,
  DAC_ADDRESS_I_CLH,
  DAC_ADDRESS_V_CLH,
  DAC_ADDRESS_I_CPL_5UA,
  DAC_ADDERSS_I_CPL_20UA,
  DAC_ADDRESS_I_CPL_200UA,
  DAC_ADDRESS_I_CPL_2MA,
  DAC_ADDRESS_I_CPL_EXT,
  DAC_ADDRESS_V_CPL,
  DAC_ADDRESS_I_CPH_5UA,
  DAC_ADDRESS_I_CPH_20UA,
  DAC_ADDRESS_I_CPH_200UA,
  DAC_ADDRESS_I_CPH_2MA,
  DAC_ADDRESS_I_CPH_EXT,
  DAC_ADDRESS_V_CPH,
};

static void delay(uint32_t n)
{
  uint32_t cntr = n;
  while(--n){
    __NOP();
  }
}


static void chipSel(AD5522Driver *dev, uint8_t set)
{
  if(DBG_CHECK(dev->config->ssport)) return;
  if(DBG_CHECK(dev->config->sspad)) return;
  
  if(set){
    palClearPad(dev->config->ssport, dev->config->sspad);
    delay(200);
  }
  else
    palSetPad(dev->config->ssport, dev->config->sspad);
}

static void registerRead(AD5522Driver *dev, uint8_t *b)
{
  if(DBG_CHECK(dev->config->devp)) return;
 
  uint8_t n = 4 ;
  spiAcquireBus(dev->config->devp);
  spiStart(dev->config->devp, dev->config->config);
  chipSel(dev,1);
  spiReceive(dev->config->devp,n,b);
  chipSel(dev,0);
  spiStop(dev->config->devp);
  spiReleaseBus(dev->config->devp);
  
}

static void registerWrite(AD5522Driver *dev, uint8_t *b)
{
  if(DBG_CHECK(dev->config->ssport)) return;
  if(DBG_CHECK(dev->config->sspad)) return;
  if(DBG_CHECK(dev->config->devp)) return;
  
  uint8_t n = 4 ;
  spiAcquireBus(dev->config->devp);
  spiStart(dev->config->devp, dev->config->config);
  chipSel(dev,1);
  spiSend(dev->config->devp,n,b);
  chipSel(dev,0);
  spiStop(dev->config->devp);
  spiReleaseBus(dev->config->devp);
  
}

static void registerExchange(AD5522Driver *dev,uint8_t *tx, uint8_t *rx)
{
  if(DBG_CHECK(dev->config->devp)) return;
  
  uint8_t n = 4 ;
  spiAcquireBus(dev->config->devp);
  spiStart(dev->config->devp, dev->config->config);
  chipSel(dev,1);
  spiExchange(dev->config->devp,n,tx,rx);
  chipSel(dev,0);
  spiStop(dev->config->devp);
  spiReleaseBus(dev->config->devp);  
}


static bool dev_busy(AD5522Driver *dev)
{
  if(DBG_CHECK(dev->config->devp)) return false;
  if(DBG_CHECK(dev->config->busyport)) return false;
  if(palReadPad(dev->config->busyport,dev->config->busypad) == PAL_LOW)
    return true;
  return false;
}

void ad5522_reset(AD5522Driver *dev,uint8_t channel)
{
  if(DBG_CHECK(dev->config->devp)) return;
  uint8_t buf_out[4] = {0,0,0,0};
  uint32_t v = 0x0;
  v |= PMU_REG_WR_MASK;
  v |= PMU_REG_SELECT(channel);
  v |= PMU_REG_MODE(PMU_MODE_WR_SYS_REG);
  v |= PMU_PMU_REG_RESET(1);
  //v <<= 3;
  
  buf_out[0] = (v >> 24) & 0xFF;
  buf_out[1] = (v >> 16) & 0xFF;
  buf_out[2] = (v >> 8) & 0xFF;
  buf_out[3] = (v & 0xFF);
  
  registerWrite(dev,buf_out);
  
}

void ad5522_nop(AD5522Driver *dev)
{
  if(DBG_CHECK(dev->config->devp)) return;
  uint8_t buf_out[4] = {0,0xff,0xff,0xff};
  registerWrite(dev,buf_out);
}

static uint32_t ad5522_readFrame(AD5522Driver *dev)
{
  if(DBG_CHECK(dev->config->devp)) return 0;
  uint8_t buf_out[4] = {0x00,0xFF,0xFF,0xFF};
  uint8_t buf_in[4];
  uint32_t result;
  registerExchange(dev,buf_out,buf_in);
  result = (buf_in[0]<<16) | (buf_in[1]<<8) | (buf_in[2]);
  return result;
}
                                               

void ad5522_rd_sysconfig(AD5522Driver *dev)
{
  if(DBG_CHECK(dev->config->devp)) return;
  uint8_t buf_out[4] = {0,0,0,0};
  uint8_t buf_in[4] = {0,0,0,0};
  uint32_t v = 0x0;
  v |= PMU_REG_READ_MASK;
  v |= (PMU_SYSCFG);
  v |= PMU_REG_MODE(PMU_MODE_WR_SYS_REG);
  // <<= 3;
  
  buf_out[0] = (v >> 24) & 0xFF;
  buf_out[1] = (v >> 16) & 0xFF;
  buf_out[2] = (v >> 8) & 0xFF;
  buf_out[3] = (v & 0xFF);
  
  registerWrite(dev,buf_out);
  v = ad5522_readFrame(dev);
  
  dev->sys_config = v;
    
}

void ad5522_wr_sysconfig(AD5522Driver *dev,uint32_t value)
{
  if(DBG_CHECK(dev->config->devp)) return ;
  uint8_t buf_out[4] = {0,0,0,0};
  //uint8_t buf_in[4] = {0,0,0,0};
  uint32_t v = value;
  v |= PMU_REG_WR_MASK;
  v |= (PMU_SYSCFG);
  v |= PMU_REG_MODE(PMU_MODE_WR_SYS_REG);
  //v <<= 3;
  
  buf_out[0] = (v >> 24) & 0xFF;
  buf_out[1] = (v >> 16) & 0xFF;
  buf_out[2] = (v >> 8) & 0xFF;
  buf_out[3] = (v & 0xFF);
  
  registerWrite(dev,buf_out);
  
  ad5522_rd_sysconfig(dev);
}

void ad5522_rd_pmu_register(AD5522Driver *dev, uint8_t pmu_id)
{
  if(DBG_CHECK(dev->config->devp)) return ;
  uint8_t buf_out[4] = {0,0,0,0};
  uint8_t buf_in[4] = {0,0,0,0};
  uint32_t v = 0x0;
  v |= PMU_REG_READ_MASK;
  v |= PMU_REG_SELECT(pmu_id);
  v |= PMU_REG_MODE(PMU_MODE_WR_SYS_REG);
  //v <<= 3;
  
  buf_out[0] = (v >> 24) & 0xFF;
  buf_out[1] = (v >> 16) & 0xFF;
  buf_out[2] = (v >> 8) & 0xFF;
  buf_out[3] = (v & 0xFF);
  
  //ad5522_nop(dev);
  registerWrite(dev,buf_out);

  v = ad5522_readFrame(dev);
  dev->PMU[pmu_id].pmu_reg = v;
  //dev->PMU[pmu_id] = v;
}

void ad5522_wr_pmu_register(AD5522Driver *dev,uint8_t pmu_id, uint32_t value)
{
  if(DBG_CHECK(dev->config->devp)) return ;
  uint8_t buf_out[4] = {0,0,0,0};
  uint32_t v = value;
  v |= PMU_REG_WR_MASK;
  v |= PMU_REG_SELECT(pmu_id);
  v |= PMU_REG_MODE(PMU_MODE_WR_SYS_REG);
  //v <<= 3;
  
  buf_out[0] = (v >> 24) & 0xFF;
  buf_out[1] = (v >> 16) & 0xFF;
  buf_out[2] = (v >> 8) & 0xFF;
  buf_out[3] = (v & 0xFF);
  
  registerWrite(dev,buf_out);
  
  ad5522_rd_pmu_register(dev,pmu_id);
}

void ad5522_rd_pmu_dac_register(AD5522Driver *dev, uint8_t pmu_id,uint8_t dac_address,uint8_t mode)
{
  if(DBG_CHECK(dev->config->devp)) return;
  uint8_t buf_out[4] = {0,0,0,0};
  uint8_t buf_in[4] = {0,0,0,0};
  uint32_t v = 0x0;
  v |= PMU_REG_READ_MASK;
  v |= PMU_REG_SELECT(pmu_id);
  v |= PMU_REG_MODE(mode);
  v |= DAC_ADDRESS(dac_address_list[dac_address]);
  //v <<= 3;
  
  buf_out[0] = (v >> 24) & 0xFF;
  buf_out[1] = (v >> 16) & 0xFF;
  buf_out[2] = (v >> 8) & 0xFF;
  buf_out[3] = (v & 0xFF);
  
  registerWrite(dev,buf_out);
  v = ad5522_readFrame(dev);
  uint16_t value = v & 0xffff;
  switch(mode){
  case 1: dev->PMU[pmu_id].dac[dac_address].m = value;break;
  case 2: dev->PMU[pmu_id].dac[dac_address].c = value;break;
  case 3: dev->PMU[pmu_id].dac[dac_address].x1 = value;break;
  default:break;
  }
}

void ad5522_wr_pmu_dac_register(AD5522Driver *dev,uint8_t pmu_id, uint8_t dac_address,uint8_t mode, uint16_t value)
{
  if(DBG_CHECK(dev->config->devp)) return;
  uint8_t buf_out[4] = {0,0,0,0};
  // check range for cll & clh, page 52
  uint16_t v_write = value;
  switch(dac_address){
  case DAC_ID_I_CLL:
  case DAC_ID_V_CLL:
    if(v_write > 0x7FFF) v_write = 0x7FFF;
    break;
  case DAC_ID_I_CLH:
  case DAC_ID_V_CLH:
    if(v_write < 0x8000) v_write = 0x8000;
    break;
  default:break;
  }


  uint32_t v = 0x0;
  v |= PMU_REG_WR_MASK;
  v |= PMU_REG_SELECT(pmu_id);
  v |= PMU_REG_MODE(mode);
  v |= DAC_ADDRESS(dac_address_list[dac_address]);
  v |= v_write;
  //v <<= 3;
  
  buf_out[0] = (v >> 24) & 0xFF;
  buf_out[1] = (v >> 16) & 0xFF;
  buf_out[2] = (v >> 8) & 0xFF;
  buf_out[3] = (v & 0xFF);
  
  
  registerWrite(dev,buf_out);
  
  ad5522_rd_pmu_dac_register(dev,pmu_id,dac_address, mode);
}

void ad5522_load(AD5522Driver *dev, uint8_t state)
{
  if(state){
    palClearPad(dev->config->loadport,dev->config->loadpad);
  }
  else{
    palSetPad(dev->config->loadport,dev->config->loadpad);
  }
    
}

/* Exported functions */

void ad5522_init(AD5522Driver *dev,const AD5522Config *config )
{
  if(dev == NULL) return;
  if(config == NULL) return;
  
  dev->config = (void*)config;
  
  if(!DBG_CHECK(dev->config->sspad)){
    palSetPadMode(dev->config->ssport, dev->config->sspad,PAL_MODE_OUTPUT_PUSHPULL);
    palSetPad(dev->config->ssport, dev->config->sspad);
  }
  if(!DBG_CHECK(dev->config->rstport)){
    palSetPadMode(dev->config->rstport, dev->config->rstpad,PAL_MODE_OUTPUT_PUSHPULL);
    palSetPad(dev->config->rstport, dev->config->rstpad);
  }
  if(!DBG_CHECK(dev->config->busyport)){
    palSetPadMode(dev->config->busyport, dev->config->busypad,PAL_MODE_INPUT_PULLUP);
  }
  if(!DBG_CHECK(dev->config->loadport)){
    palSetPadMode(dev->config->loadport, dev->config->loadpad,PAL_MODE_OUTPUT_PUSHPULL);
    if(dev->use_sync == 1)
      palSetPad(dev->config->loadport, dev->config->loadpad);
    else
      palClearPad(dev->config->loadport, dev->config->loadpad);
  }
  
  if(dev->grp_reset == 0){
    palClearPad(dev->config->rstport,dev->config->rstpad);
    delay(1000);
    palSetPad(dev->config->rstport,dev->config->rstpad);
  }
  
  for(uint8_t i=0;i< NOF_PMU_CHANNEL;i++){
    ad5522_reset(dev, i);
  }
  
  // write registers
  ad5522_rd_sysconfig(dev);
}

