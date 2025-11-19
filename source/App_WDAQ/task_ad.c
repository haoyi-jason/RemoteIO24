#include "ch.h"
#include "hal.h"
#include "task_ad.h"
#include "adda/ad7124/ad7124.h"
#include "adda/ad7124/ad7124_defs.h"
#include "numeric/filters/iir.h"
#include "sensor/temperature/thermocouple.h"
#include "sensor/ntc/ntc.h"
#include <math.h>
#include "database.h"
#include "app_defs.h"
#include "expansion/pca9555.h"
#include "numeric/filters/iir.h"

#define NOF_ADC_CHANNEL 8
#define BUFFER_DEPTH    8

#define EV_AD7124_START         EVENT_MASK(1)
#define EV_AD7124_STOP          EVENT_MASK(2)
#define EV_AD7124_CONFIG        EVENT_MASK(3)
#define EV_SEC_TIMEOUT          EVENT_MASK(8)
#define EV_PARAM_SAVE           EVENT_MASK(4)
#define EV_LOAD_DEFAULT           EVENT_MASK(5)
#define EV_AD7124_RDY           EVENT_MASK(6)


static ad7124_setup_t setups[]={
  {.u={ADC_PGA_X1,ADC_REF_REFINT,ADC_BIT_ENABLE,ADC_BIT_ENABLE,ADC_BIT_ENABLE,ADC_BIT_ENABLE,ADC_BURNOUT_OFF,ADC_BIT_ENABLE}},
  {.u={ADC_PGA_X1,ADC_REF_REFINT,ADC_BIT_ENABLE,ADC_BIT_ENABLE,ADC_BIT_ENABLE,ADC_BIT_ENABLE,ADC_BURNOUT_OFF,ADC_BIT_ENABLE}},
  {.u={ADC_PGA_X32,ADC_REF_REFINT,ADC_BIT_ENABLE,ADC_BIT_ENABLE,ADC_BIT_ENABLE,ADC_BIT_ENABLE,ADC_BURNOUT_OFF,ADC_BIT_ENABLE}},
  {.u={ADC_PGA_X128,ADC_REF_REFINT,ADC_BIT_ENABLE,ADC_BIT_ENABLE,ADC_BIT_ENABLE,ADC_BIT_ENABLE,ADC_BURNOUT_OFF,ADC_BIT_ENABLE}},
  {.u={ADC_PGA_X1,ADC_REF_REFINT,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BURNOUT_OFF,ADC_BIT_DISABLE}},
  {.u={ADC_PGA_X1,ADC_REF_REFINT,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BURNOUT_OFF,ADC_BIT_DISABLE}},
  {.u={ADC_PGA_X1,ADC_REF_REFINT,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BURNOUT_OFF,ADC_BIT_DISABLE}},
  {.u={ADC_PGA_X1,ADC_REF_REFINT,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BURNOUT_OFF,ADC_BIT_DISABLE}},
  {.u={ADC_PGA_X1,ADC_REF_REFINT,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BIT_DISABLE,ADC_BURNOUT_OFF,ADC_BIT_ENABLE}},
};

static ad7124_filter_t filters[]={
  {.u={12,ADC_BIT_DISABLE,2,ADC_BIT_DISABLE,ADC_FILER_SINC4}},
  {.u={24,ADC_BIT_DISABLE,2,ADC_BIT_DISABLE,ADC_FILER_SINC4}},
  {.u={24,ADC_BIT_DISABLE,2,ADC_BIT_DISABLE,ADC_FILER_SINC4}},
  {.u={24,ADC_BIT_DISABLE,2,ADC_BIT_ENABLE,ADC_FILER_SINC4}}, // use 50Hz output rate , sinc 4 filter to provide 50/60hz notch filter
  {.u={24,ADC_BIT_DISABLE,0,ADC_BIT_DISABLE,ADC_FILER_SINC4}},
  {.u={24,ADC_BIT_DISABLE,0,ADC_BIT_DISABLE,ADC_FILER_SINC4}},
  {.u={24,ADC_BIT_DISABLE,0,ADC_BIT_DISABLE,ADC_FILER_SINC4}},
  {.u={24,ADC_BIT_DISABLE,0,ADC_BIT_DISABLE,ADC_FILER_SINC4}},
};

static const SPIConfig spicfg = {
  FALSE,
  NULL,
  NULL,//GPIOB,
  NULL,//GPIOB_SPI1_CS,
  SPI_CR1_BR_1 | SPI_CR1_CPOL | SPI_CR1_CPHA
};

static const AD7124Config config ={
  &SPID1,
  &spicfg,
  GPIOA,
  4,
  GPIOA,
  6 // dout/drdy
};

static PCA9555Driver pca9555;
static const I2CConfig i2ccfg = {
  OPMODE_I2C,
  100000,
  STD_DUTY_CYCLE
};

static pca_9555_config_t pca9555_config = {
  &I2CD1,
  &i2ccfg,
  {0x00,0x00}, // output
  {0xFF,0xFF}, // dont care
};

static AD7124Driver ad7124 = {
  0,
  0,
  &config,
  NULL,
  NULL,
  NULL,
  NULL
};

struct _runTime{
  thread_t *self;
  thread_t *main;
  thread_reference_t ref;
  uint8_t curr_channel;
  int32_t curr_rawData;
  uint8_t samples_to_ignore;
  uint8_t samples_to_acquire;
  uint8_t sample_acquired;
  uint8_t buffer_index;
  uint8_t adcReady;
  systime_t lastIdleTime;
  _int_filter_t rawFilter[8];
};

static struct _runTime runTime, *adcRunTime;

#define AD7124_SIGNATURE   0xAC
#define SIGNATURE_OFFSET        0
static void init_config()
{
  // load config, filter, offset, gain code
  uint8_t i,u8;
  uint16_t u16;
  uint32_t u32;
  
  u8 = db_read_df_u8(APP_SIGNATURE+SIGNATURE_OFFSET);
  
  if(u8 != AD7124_SIGNATURE){
    db_enable_save_on_write(false);
    for(i=0;i<8;i++){
      u16 = setups[i].b[0] | (setups[i].b[1] << 8);
      db_write_df_u16(AD7124_SETUP_CCONFIG_1+i,u16);
      u32 = filters[i].b[0] | (filters[i].b[1] <<8) | (filters[i].b[2]<<16);
      db_write_df_u32(AD7124_FILTER_CONFIG_1+i, u32);
    }
    
    for(i=0;i<16;i++){
      db_write_df_f32(ENG_GAIN_CHANNEL_1+i,0);
    }
    db_enable_save_on_write(true);
    db_write_df_u8(APP_SIGNATURE+SIGNATURE_OFFSET, AD7124_SIGNATURE);
    db_save_section(U8);
    db_save_section(U16);
    db_save_section(U32);
    db_save_section(F32);
  }
  else{
    for(i=0;i<8;i++){
      u16 = db_read_df_u16(AD7124_SETUP_CCONFIG_1+i);
      setups[i].b[0] = (uint8_t)(u16 & 0xFF);
      setups[i].b[1] = (uint8_t)(u16 >> 8);
      u32 = db_read_df_u32(AD7124_FILTER_CONFIG_1+i);
      filters[i].b[0] = (uint8_t)(u32 & 0xFF);
      filters[i].b[1] = (uint8_t)((u32 >> 8) & 0xFF);
      filters[i].b[2] = (uint8_t)((u32 >> 16) & 0xFF);
      
    }
  }
  
  
  u8 = db_read_df_u8(APP_CONFIG_ADC_CHANNEL_ENABLE_MASK);
  
  u8 = ~u8; // setting: 1=enable, pca9555: output low to enable power
  uint8_t tx[2] = {u8,u8 & 0x3f};
  pca9555_write(&pca9555,tx,PCA9555_PORT_COUNT);  

  for(uint8_t i=0;i<8;i++){
    ad7124_set_config(&ad7124,i,&setups[i]);
    ad7124_set_filter(&ad7124,i,&filters[i]);
  }  
  
}

static void app_channel_config()
{
  for(uint8_t i=0;i<8;i++){
    ad7124_set_config(&ad7124,i,&setups[i]);
    ad7124_set_filter(&ad7124,i,&filters[i]);
  }  
  
  ad7124_channel_t cfg;
  
  // channel config
  uint8_t ch;
  for(ch = 0;ch < 6; ch++){
    cfg.u.ainm = ch * 2;
    cfg.u.ainp = ch * 2 + 1;
    cfg.u.setup = 3; // 128
    cfg.u.enable = 1;
    ad7124_set_channel(&ad7124,ch,&cfg);
  }
  
}

static THD_WORKING_AREA(waAD7124,2048);
static THD_FUNCTION(procAD7124 ,p)
{
    event_listener_t el_adc;
    ad7124_channel_t chcfg;
    // todo: read settings from database
    int32_t data;
    uint8_t status;
    uint8_t channel;
    uint8_t cntr;
    int8_t i;
    if(pca9555_init(&pca9555,&pca9555_config,  PCA9555_ADDR_MASK(0x1)) == MSG_OK){
      //runTime.hasExpansion = pca9555.sla;
      pca9555.inverted = true;
    }


    if(ad7124_init(&ad7124) != AD7124_OK){
      runTime.adcReady = 0;
    }
    else{
      runTime.adcReady = 1;
      init_config();
      ad7124_enable_all_error(&ad7124);
    }
    
//    ad7124_get_channel(&ad7124,1,&chcfg);
//    chcfg.u.enable = 1;
//    
//    if(ad7124_set_channel(&ad7124,1,&chcfg) != 0){
//      //while(1);
//    }
    app_channel_config();
    
    bool bStop = false;
    bool adcRestart = false;
    uint8_t timeout = 0;
    chEvtRegisterMask(&ad7124.ev_drdy,&el_adc,EV_AD7124_RDY);
    ad7124_start_acquisition(&ad7124,runTime.curr_channel,4,1);
    chThdResume(&runTime.ref, MSG_OK);
    
    for(i=0;i<8;i++){
      runTime.rawFilter[i].stages = 8;
      runTime.rawFilter[i].reset = 1;
    }
            
    while(!bStop){
      eventmask_t evt = chEvtWaitAnyTimeout(ALL_EVENTS,10);
      
      if(evt == 0){
        timeout++;
        if(timeout > 10){
          timeout = 0;
          ad7124_start_acquisition(&ad7124,runTime.curr_channel,4,1);
        }
      }
      else{
        timeout = 0;
        if(evt & EV_AD7124_RDY){
          ad7124_ReadDataEx(&ad7124,&data,&status);
          channel = status & 0x0F;
          //db_write_livedata();
          iir_insert(&runTime.rawFilter[channel],data);
          db_write_ld_i32(LIVE_DATA_CH1_RAW+channel, runTime.rawFilter[channel].last);
          ad7124_enable_interrupt(&ad7124);
        }
        if(evt & EV_AD7124_START){
          runTime.curr_channel = 0 ;
          ad7124_start_acquisition(&ad7124,runTime.curr_channel,4,1);
        }
        
        if(evt & EV_AD7124_STOP){
          ad7124_stop(&ad7124);
          ad7124_disable_interrupt(&ad7124);
          ad7124_endis_channel(&ad7124,runTime.curr_channel,0);
        }      
      }
      
      if(chThdShouldTerminateX()){
        bStop = true;
      }
    }
    
  
}


void Ad7124_task_init()
{
  adcRunTime = &runTime;
  runTime.main = chRegFindThreadByName("Main");;
  //adcNvmParam = &nvmParam;
  runTime.self = chThdCreateStatic(waAD7124,sizeof(waAD7124),NORMALPRIO,procAD7124,NULL);
  chSysLock();
  chThdSuspendS(&runTime.ref);
  chSysUnlock();
}

