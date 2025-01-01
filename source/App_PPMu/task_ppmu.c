#include "ch.h"
#include "hal.h"
#include "task_ppmu.h"
#include "ad5522_dev.h"
#include "ad5522_def.h"
#include "nvm_config.h"
#include "../drivers/nvm.h"

#define NOF_AD5522      2

static const SPIConfig spicfg = {
  false,
  NULL,
  NULL,
  NULL,
  SPI_CR1_BR_2  
};

static struct nvmParam_s{
  uint16_t checksum;
  _dac_reg_grp_t dac_registers_A[NOF_DAC_ADDRESS];
  _dac_reg_grp_t dac_registers_B[NOF_DAC_ADDRESS];
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
    3,
    GPIOA,
    0,
    GPIOA,
    2,
    GPIOA,
    3
  },
  {
    &SPID1,
    &spicfg,
    GPIOA,
    3,
    GPIOA,
    0,
    GPIOA,
    2,
    GPIOA,
    3
  }
};

static void load_nvm_default()
{
  
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
  runTime.mainThread = chRegFindThreadByName("Main");
  runTime.self = chThdCreateStatic(waPMU,sizeof(waPMU),NORMALPRIO,procPMU,NULL);
  chThdSuspend(&runTime.reference);
}

