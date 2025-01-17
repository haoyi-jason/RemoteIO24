#include "ch.h"
#include "hal.h"
#include "task_ppmu.h"
#include "ad5522_dev.h"
#include "ad5522_def.h"
#include "nvm_config.h"
#include "../drivers/nvm.h"

#define ASSERT_CHANNEL(x)       (x < TOTAL_PMU_CHANNEL)
#define ASSERT(id,pmu) ((id < NOF_AD5522) && (pmu < NOF_PMU_CHANNEL))
#define ASSERT_DAC(id,pmu,dac) ((id < NOF_AD5522) && (pmu_id < NOF_PMU_CHANNEL) && (dac < NOF_DAC_ADDRESS))


static const SPIConfig spicfg = {
  false,
  NULL,
  NULL,
  NULL,
  SPI_CR1_BR_2  | SPI_CR1_CPHA
};

static struct nvmParam_s{
  uint16_t checksum;
  _dac_reg_grp_t dac_registers[NOF_AD5522*NOF_PMU_CHANNEL*NOF_DAC_ADDRESS];
}nvmParam;

static struct{
  thread_t *self;
  thread_t *mainThread;
  thread_reference_t reference;
  AD5522Driver ad5522[NOF_AD5522];
}runTime;


static AD5522Config ad5522_config[NOF_AD5522] = {
  {
    &SPID1,
    &spicfg,
    GPIOA,
    4,
    GPIOA,
    0,
    GPIOA,
    2,
    GPIOA,
    3
  },
//  {
//    &SPID1,
//    &spicfg,
//    GPIOA,
//    4,
//    GPIOA,
//    0,
//    GPIOA,
//    2,
//    GPIOA,
//    3
//  }
};

static void load_nvm_default()
{
  for(uint8_t i=0;i<TOTAL_PMU_CHANNEL;i++){
    for(uint8_t j=0;j<NOF_DAC_ADDRESS;j++){
      nvmParam.dac_registers[i*NOF_DAC_ADDRESS + j].m = 65535;
      nvmParam.dac_registers[i*NOF_DAC_ADDRESS + j].c = 32768;
    }
  }
}

static void save_nvm()
{
  uint16_t checksum = 0x0;
  uint16_t sz = (sizeof(struct nvmParam_s));
  uint8_t *ptr = (uint8_t*)&nvmParam;
  ptr++;
  for(uint16_t i=1;i<sz;i++){
    checksum += *ptr++;
  }
  nvmParam.checksum = checksum;
  nvm_flash_write(OFFSET_NVM_PPMU,(uint8_t*)&nvmParam,sizeof(struct nvmParam_s));
}

static void load_nvm()
{
  uint16_t sz = (sizeof(struct nvmParam_s));
  nvm_flash_read(OFFSET_NVM_PPMU,(uint8_t*)&nvmParam,sz);

  uint16_t checksum = 0x0;
  uint8_t *ptr = (uint8_t*)&nvmParam;
  ptr++;
  for(uint16_t i=1;i<sz;i++){
    checksum += *ptr++;
  }
  
  if(checksum != nvmParam.checksum){
    load_nvm_default();
    save_nvm();
  }
  
}

static THD_WORKING_AREA(waPMU,1024);
static THD_FUNCTION(procPMU ,p)
{
  load_nvm();
  
  for(uint8_t i=0;i<NOF_AD5522;i++){
//    memcpy((void*)runTime.ad5522[i].dac_registers, nvmParam.dac_registers[i*NOF_DAC_ADDRESS], sizeof(_dac_reg_grp_t));
    for(uint8_t j=0;j<NOF_PMU_CHANNEL;j++){
      runTime.ad5522[i].PMU[j].dac = &nvmParam.dac_registers[i*j*NOF_DAC_ADDRESS];
    }
    ad5522_init(&runTime.ad5522[i],&ad5522_config[i]);
  }
  
  
  chThdResume(&runTime.reference,MSG_OK);
  bool bStop = false;
  while(!bStop){
    
    
    chThdSleepMilliseconds(50);
    if(chThdShouldTerminateX()){
      bStop = true;
    }
  }
}

void pmu_init()
{
//  runTime.mainThread = chRegFindThreadByName("Main");
//  runTime.self = chThdCreateStatic(waPMU,sizeof(waPMU),NORMALPRIO,procPMU,NULL);
//  chThdSuspend(&runTime.reference);
  load_nvm();
  for(uint8_t i=0;i<NOF_AD5522;i++){
    for(uint8_t j=0;j<NOF_PMU_CHANNEL;j++){
      runTime.ad5522[i].PMU[j].dac = &nvmParam.dac_registers[i*j*NOF_DAC_ADDRESS];
      runTime.ad5522[i].PMU[j].enable = 1;
      runTime.ad5522[i].PMU[j].fin = 1;
      runTime.ad5522[i].PMU[j].force = 1; // FIMV
      runTime.ad5522[i].PMU[j].crange = 3;
    }
    ad5522_init(&runTime.ad5522[i],&ad5522_config[i]);
  }
  
  for(uint8_t i=0;i<TOTAL_PMU_CHANNEL;i++){
    pmu_reg_flush(i);
  }

  // test output 
  for(uint8_t i=0;i<128;i++){
    pmu_set_dac_output(0,DAC_ID_I_2MA,i<<9);
    pmu_set_dac_output(1,DAC_ID_I_2MA,i<<9);
    pmu_set_dac_output(2,DAC_ID_I_2MA,i<<9);
    pmu_set_dac_output(3,DAC_ID_I_2MA,i<<9);
    chThdSleepMilliseconds(500);
  }
}

void pmu_save_state()
{
  save_nvm();
}
//! DAC registers helper function
void pmu_set_dac_output(uint8_t channel, uint8_t reg, uint16_t value)
{
  if(!ASSERT_CHANNEL(channel)) return;
  uint8_t id = channel / NOF_CHANNEL_PER_PMU;
  uint8_t pmu_id = channel % NOF_CHANNEL_PER_PMU;
  ad5522_wr_pmu_dac_register(&runTime.ad5522[id], pmu_id,reg, DAC_REGISTER_X1,value);
}

void pmu_set_dac_gain(uint8_t channel, uint8_t reg, uint16_t value)
{
  if(!ASSERT_CHANNEL(channel)) return;
  uint8_t id = channel / NOF_CHANNEL_PER_PMU;
  uint8_t pmu_id = channel % NOF_CHANNEL_PER_PMU;
  ad5522_wr_pmu_dac_register(&runTime.ad5522[id], pmu_id,reg, DAC_REGISTER_M,value);
}

void pmu_set_dac_offset(uint8_t channel, uint8_t reg, uint16_t value)
{
  if(!ASSERT_CHANNEL(channel)) return;
  uint8_t id = channel / NOF_CHANNEL_PER_PMU;
  uint8_t pmu_id = channel % NOF_CHANNEL_PER_PMU;
  ad5522_wr_pmu_dac_register(&runTime.ad5522[id], pmu_id,reg, DAC_REGISTER_C,value);
}


//! write force outout
void pmu_set_output(uint8_t channel, uint16_t value)
{
  if(!ASSERT_CHANNEL(channel)) return;
  uint8_t id = channel / NOF_CHANNEL_PER_PMU;
  uint8_t pmu_id = channel % NOF_CHANNEL_PER_PMU;
  _pmu_config_t *pmu = &runTime.ad5522[id].PMU[pmu_id];
  switch(pmu->force){
  case PMU_PMU_REG_FORCE_FVMI:
    pmu_set_dac_output(channel,DAC_ID_V,value);
    break;
  case PMU_PMU_REG_FORCE_FIMV:
    switch(pmu->crange){
    case PMU_PMU_REG_CRANGE_5_UA:
      pmu_set_dac_output(channel,DAC_ID_I_5UA,value);
      break;
    case PMU_PMU_REG_CRANGE_20_UA:
      pmu_set_dac_output(channel,DAC_ID_I_20UA,value);
      break;
    case PMU_PMU_REG_CRANGE_200_UA:
      pmu_set_dac_output(channel,DAC_ID_I_200UA,value);
      break;
    case PMU_PMU_REG_CRANGE_2_MA:
      pmu_set_dac_output(channel,DAC_ID_I_2MA,value);
      break;
    case PMU_PMU_REG_CRANGE_EXT:
      pmu_set_dac_output(channel,DAC_ID_I_EXT,value);
      break;
    default:break;
    }
    break;
  case PMU_PMU_REG_FORCE_FVHZ:
    break;
  case PMU_PMU_REG_FORCE_FCHZ:
    break;
  default:break;
  }
}

void pmu_set_output_all(uint16_t value)
{
  for(uint8_t i=0;i<NOF_AD5522;i++){
    for(uint8_t j=0; j< NOF_PMU_CHANNEL;j++){
      //pmu_set_output(i,j,value);
    }
  }
}
//! set output mode of FV/FI/HZFV/HZFI
void pmu_set_output_mode(uint8_t channel, uint8_t mode)
{
  if(!ASSERT_CHANNEL(channel)) return;
  uint8_t id = channel / NOF_CHANNEL_PER_PMU;
  uint8_t pmu_id = channel % NOF_CHANNEL_PER_PMU;
  runTime.ad5522[id].PMU[pmu_id].force = (mode);
}

void pmu_set_output_mode_all(uint8_t mode)
{
  for(uint8_t i=0;i<NOF_AD5522;i++){
    for(uint8_t j=0; j< NOF_PMU_CHANNEL;j++){
     // pmu_set_output_mode(i,j,mode);
    }
  }
}

void pmu_set_crange(uint8_t channel, uint8_t value)
{
  if(!ASSERT_CHANNEL(channel)) return;
  uint8_t id = channel / NOF_CHANNEL_PER_PMU;
  uint8_t pmu_id = channel % NOF_CHANNEL_PER_PMU;
  runTime.ad5522[id].PMU[pmu_id].crange = value;
}

void pmu_set_crange_all(uint8_t value)
{
  for(uint8_t i=0;i<NOF_AD5522;i++){
    for(uint8_t j=0; j< NOF_PMU_CHANNEL;j++){
      //pmu_set_crange(i,j,value);
    }
  }
}

void pmu_enable(uint8_t channel, bool enable)
{
  if(!ASSERT_CHANNEL(channel)) return;
  uint8_t id = channel / NOF_CHANNEL_PER_PMU;
  uint8_t pmu_id = channel % NOF_CHANNEL_PER_PMU;
  runTime.ad5522[id].PMU[pmu_id].enable = enable?1:0;    
}

//void pmu_fore_type(uint8_t id, uint8_t pmu_id, uint8_t value)
//{
//  if(!ASSERT(id,pmu_id)) return;
//  runTime.ad5522[id].PMU[pmu_id].force = value;    
//}
//
//void pmu_crange(uint8_t id, uint8_t pmu_id, uint8_t value)
//{
//  if(!ASSERT(id,pmu_id)) return;
//  runTime.ad5522[id].PMU[pmu_id].crange = value;    
//}

void pmu_measout(uint8_t channel, uint8_t value)
{
  if(!ASSERT_CHANNEL(channel)) return;
  uint8_t id = channel / NOF_CHANNEL_PER_PMU;
  uint8_t pmu_id = channel % NOF_CHANNEL_PER_PMU;
  runTime.ad5522[id].PMU[pmu_id].measout = value;    
}

//void pmu_enable_input(uint8_t id, uint8_t pmu_id, uint8_t value)
//{
//  if(!ASSERT(id,pmu_id)) return;
//  runTime.ad5522[id].PMU[pmu_id].force = value;    
//}

void pmu_sys_force(uint8_t channel, uint8_t value)
{
  if(!ASSERT_CHANNEL(channel)) return;
  uint8_t id = channel / NOF_CHANNEL_PER_PMU;
  uint8_t pmu_id = id % NOF_CHANNEL_PER_PMU;
  runTime.ad5522[id].PMU[pmu_id].sf = value;    
}

void pmu_cl(uint8_t channel, uint8_t value)
{
  if(!ASSERT_CHANNEL(channel)) return;
  uint8_t id = channel / NOF_CHANNEL_PER_PMU;
  uint8_t pmu_id = channel % NOF_CHANNEL_PER_PMU;
  runTime.ad5522[id].PMU[pmu_id].clamp_enable = value;    
}

void pmu_cpolh(uint8_t channel, uint8_t value)
{
  if(!ASSERT_CHANNEL(channel)) return;
  uint8_t id = channel / NOF_CHANNEL_PER_PMU;
  uint8_t pmu_id = channel % NOF_CHANNEL_PER_PMU;
  runTime.ad5522[id].PMU[pmu_id].cmp_out = value;    
}

void pmu_clear(uint8_t channel, uint8_t value)
{
  if(!ASSERT_CHANNEL(channel)) return;
  uint8_t id = channel / NOF_CHANNEL_PER_PMU;
  uint8_t pmu_id = channel % NOF_CHANNEL_PER_PMU;
  runTime.ad5522[id].PMU[pmu_id].clear = value;    
}

void pmu_reg_flush(uint8_t channel)
{
  if(!ASSERT_CHANNEL(channel)) return;
  uint8_t id = channel / NOF_CHANNEL_PER_PMU;
  uint8_t pmu_id = channel % NOF_CHANNEL_PER_PMU;
  uint32_t mask = 0x0;
  _pmu_config_t *cfg = &runTime.ad5522[id].PMU[pmu_id];
  mask |= PMU_CHANNEL_SELECT(channel);
  mask |= PMU_PMU_REG_CHANNEL_EN_(cfg->enable);
  mask |= PMU_PMU_REG_FORCE(cfg->force);
  mask |= PMU_PMU_REG_CRANGE(cfg->crange);
  mask |= PMU_PMU_REG_MEAS(cfg->measout);
  mask |= PMU_PMU_REG_FORCE_IN(cfg->fin);
  mask |= PMU_PMU_REG_SYS_FORCE(cfg->sf);
  mask |= PMU_PMU_REG_CL(cfg->clamp_enable);
  mask |= PMU_PMU_REG_CPOLH(cfg->cpolh);
  mask |= PMU_PMU_REG_CMP(cfg->cmp_out);
  mask |= PMU_PMU_REG_RESET(cfg->clear);
  // write register
  ad5522_wr_pmu_register(&runTime.ad5522[id], pmu_id, mask);
  
}

           
           
           
void pmu_load(uint8_t id)
{
  for(uint8_t i=0;i<4;i++){
    pmu_reg_flush(i);
  }
  ad5522_load(&runTime.ad5522[0],1);
}

void pmu_unload(uint8_t id)
{
  ad5522_load(&runTime.ad5522[0],0);
}
