#include "ch.h"
#include "hal.h"
#include "ad5522_dev.h"
#include "ad5522_def.h"

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

static void chipSel(AD5522Driver *dev, uint8_t set)
{
  if(DBG_CHECK(dev->config->ssport)) return;
  if(DBG_CHECK(dev->config->sspad)) return;
  
  if(set)
    palClearPad(dev->config->ssport, dev->config->sspad);
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
  spiSend(dev->config->devp,4,b);
  chipSel(dev,0);
  spiStop(dev->config->devp);
  spiReleaseBus(dev->config->devp);
  
}

static void registerExchange(AD5522Driver *dev,uint8_t *tx, uint8_t *rx)
{
  if(DBG_CHECK(dev->config->devp)) return;
  
  spiAcquireBus(dev->config->devp);
  spiStart(dev->config->devp, dev->config->config);
  chipSel(dev,1);
  spiExchange(dev->config->devp,4,tx,rx);
  chipSel(dev,0);
  spiStop(dev->config->devp);
  spiReleaseBus(dev->config->devp);  
}


static void delay(uint32_t n)
{
  uint32_t cntr = n;
  while(--n){
    __NOP();
  }
}

static void ad5522_reset(AD5522Driver *dev,uint8_t channel)
{
  uint8_t buf_out[4] = {0,0,0,0};
  uint32_t v = 0x0;
  v |= PMU_REG_WR_MASK;
  v |= PMU_REG_SELECT(channel);
  v |= PMU_REG_MODE(PMU_MODE_WR_SYS_REG);
  v |= PMU_PMU_REG_RESET;
  
  buf_out[3] = (v >> 24) & 0xFF;
  buf_out[2] = (v >> 16) & 0xFF;
  buf_out[1] = (v >> 8) & 0xFF;
  buf_out[0] = (v & 0xFF);
  
  registerWrite(dev,buf_out);
  
}

static void ad5522_rd_sysconfig(AD5522Driver *dev)
{
  uint8_t buf_out[4] = {0,0,0,0};
  uint8_t buf_in[4] = {0,0,0,0};
  uint32_t v = 0x0;
  v |= PMU_REG_READ_MASK;
  v |= (PMU_SYSCFG);
  v |= PMU_REG_MODE(PMU_MODE_WR_SYS_REG);
  
  buf_out[3] = (v >> 24) & 0xFF;
  buf_out[2] = (v >> 16) & 0xFF;
  buf_out[1] = (v >> 8) & 0xFF;
  buf_out[0] = (v & 0xFF);
  
  registerExchange(dev,buf_out,buf_in);
  
  dev->sys_config = (buf_in[3] << 24) | (buf_in[2] << 16) | (buf_in[1] << 8) | (buf_in[0]);
    
}

static void ad5522_wr_sysconfig(AD5522Driver *dev,uint32_t value)
{
  uint8_t buf_out[4] = {0,0,0,0};
  //uint8_t buf_in[4] = {0,0,0,0};
  uint32_t v = 0x0;
  v |= PMU_REG_WR_MASK;
  v |= (PMU_SYSCFG);
  v |= PMU_REG_MODE(PMU_MODE_WR_SYS_REG);
  
  buf_out[3] = (v >> 24) & 0xFF;
  buf_out[2] = (v >> 16) & 0xFF;
  buf_out[1] = (v >> 8) & 0xFF;
  buf_out[0] = (v & 0xFF);
  
  registerWrite(dev,buf_out);
  
  ad5522_rd_sysconfig(dev);
}

static void ad5522_rd_pmu_register(AD5522Driver *dev, uint8_t pmu_id)
{
  uint8_t buf_out[4] = {0,0,0,0};
  uint8_t buf_in[4] = {0,0,0,0};
  uint32_t v = 0x0;
  v |= PMU_REG_READ_MASK;
  v |= PMU_REG_SELECT(pmu_id);
  v |= PMU_REG_MODE(PMU_MODE_WR_SYS_REG);
  
  buf_out[3] = (v >> 24) & 0xFF;
  buf_out[2] = (v >> 16) & 0xFF;
  buf_out[1] = (v >> 8) & 0xFF;
  buf_out[0] = (v & 0xFF);
  
  registerExchange(dev,buf_out,buf_in);
  
  dev->sys_config = (buf_in[3] << 24) | (buf_in[2] << 16) | (buf_in[1] << 8) | (buf_in[0]);
}

static void ad5522_wr_pmu_register(AD5522Driver *dev,uint8_t pmu_id, uint32_t value)
{
  uint8_t buf_out[4] = {0,0,0,0};
  uint32_t v = value;
  v |= PMU_REG_WR_MASK;
  v |= PMU_REG_SELECT(pmu_id);
  v |= PMU_REG_MODE(PMU_MODE_WR_SYS_REG);
  
  buf_out[3] = (v >> 24) & 0xFF;
  buf_out[2] = (v >> 16) & 0xFF;
  buf_out[1] = (v >> 8) & 0xFF;
  buf_out[0] = (v & 0xFF);
  
  registerWrite(dev,buf_out);
  
  ad5522_rd_pmu_register(dev,pmu_id);
}

static void ad5522_rd_pmu_adc_register(AD5522Driver *dev, uint8_t pmu_id,uint8_t adc_address,uint8_t mode)
{
  uint8_t buf_out[4] = {0,0,0,0};
  uint8_t buf_in[4] = {0,0,0,0};
  uint32_t v = 0x0;
  v |= PMU_REG_READ_MASK;
  v |= PMU_REG_SELECT(pmu_id);
  v |= PMU_REG_MODE(mode);
  
  buf_out[3] = (v >> 24) & 0xFF;
  buf_out[2] = (v >> 16) & 0xFF;
  buf_out[1] = (v >> 8) & 0xFF;
  buf_out[0] = (v & 0xFF);
  
  registerExchange(dev,buf_out,buf_in);
  
  switch(mode){
  case 1: dev->dac_registers[adc_address].m = (buf_in[3] << 24) | (buf_in[2] << 16) | (buf_in[1] << 8) | (buf_in[0]);break;
  case 2: dev->dac_registers[adc_address].c = (buf_in[3] << 24) | (buf_in[2] << 16) | (buf_in[1] << 8) | (buf_in[0]);break;
  case 3: dev->dac_registers[adc_address].x1 = (buf_in[3] << 24) | (buf_in[2] << 16) | (buf_in[1] << 8) | (buf_in[0]);break;
  default:break;
  }
}

static void ad5522_wr_pmu_adc_register(AD5522Driver *dev,uint8_t pmu_id, uint8_t adc_address,uint8_t mode, uint16_t value)
{
  uint8_t buf_out[4] = {0,0,0,0};
  uint32_t v = value;
  v |= PMU_REG_WR_MASK;
  v |= PMU_REG_SELECT(pmu_id);
  v |= PMU_REG_MODE(mode);
  
  buf_out[3] = (v >> 24) & 0xFF;
  buf_out[2] = (v >> 16) & 0xFF;
  buf_out[1] = (v >> 8) & 0xFF;
  buf_out[0] = (v & 0xFF);
  
  registerWrite(dev,buf_out);
  
  ad5522_rd_pmu_adc_register(dev,pmu_id,adc_address, mode);
}


/* Exported functions */

void ad5522_init(AD5522Driver *dev,const AD5522Config *config )
{
  if(dev == NULL) return;
  if(config == NULL) return;
  
  dev->config = (void*)config;
  
  // reset
  for(uint8_t i=0;i< NOF_PMU_CHANNEL;i++){
    ad5522_reset(dev, PMU_CHANNEL_0 + i);
  }
  
  // write registers
  ad5522_rd_sysconfig(dev);
  for(uint8_t p=1;p<5;p++){
    for(uint8_t i=0;i<NOF_DAC_ADDRESS;i++){
      ad5522_wr_pmu_adc_register(dev,p,i,PMU_MODE_WR_DAC_GAIN,dev->dac_registers[i].m);
      ad5522_wr_pmu_adc_register(dev,p,i,PMU_MODE_WR_DAC_OFFSET,dev->dac_registers[i].c);
      ad5522_wr_pmu_adc_register(dev,p,i,PMU_MODE_WR_DAC_X1,dev->dac_registers[i].x1);
    }
  }
  
}

