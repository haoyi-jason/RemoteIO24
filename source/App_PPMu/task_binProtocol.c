#include "ch.h"
#include "hal.h"
#include "protocol/bin_protocol/bin_protocol.h"
#include "task_binProtocol.h"
#include "task_ppmu.h"
#include "nvm_config.h"

#define PORT    SD1
#define PROTOCOL_SIZE   6
#define NVM_FLAG        0xAA
#define RESPONSE_DELAY_MS       2
// function codes
enum FC_CODE_e{
  FC_ENABLE = 0x10,
  FC_FORCE,
  FC_MODE,
  FC_RANGE,
  FC_MEASOUT,
  FC_CLAMP,
  FC_CPOLH,
  FC_SF,
  FC_COMPEN,
  FC_LPF,
  FC_DUTGND,
  FC_INT10K,
  FC_OUTGAIN,
  FC_DAC_X1 = 0x20,
  FC_DAC_M,
  FC_DAC_C,
  FC_CONTROL = 0x40,
  FC_BOARD_INFO = 0x80,
  FC_ALL_DAC
};

#define PMU_ALL_CHANNEL 0xFF

typedef struct{
  thread_t *shelltp;
  BINProtocolConfig *config;
}_runTime;

static _runTime binProtoRuntime;

void cmd_enable(BaseSequentialStream *chp, uint8_t *data, uint8_t size);
void cmd_force(BaseSequentialStream *chp, uint8_t *data, uint8_t size);
void cmd_mode(BaseSequentialStream *chp, uint8_t *data, uint8_t size);
void cmd_range(BaseSequentialStream *chp, uint8_t *data, uint8_t size);
void cmd_measout(BaseSequentialStream *chp, uint8_t *data, uint8_t size);
void cmd_clamp(BaseSequentialStream *chp, uint8_t *data, uint8_t size);
void cmd_cpolh(BaseSequentialStream *chp, uint8_t *data, uint8_t size);
void cmd_sf(BaseSequentialStream *chp, uint8_t *data, uint8_t size);
void cmd_dac_output(BaseSequentialStream *chp, uint8_t *data, uint8_t size);
void cmd_dac_gain(BaseSequentialStream *chp, uint8_t *data, uint8_t size);
void cmd_dac_offset(BaseSequentialStream *chp, uint8_t *data, uint8_t size);
void cmd_compen(BaseSequentialStream *chp, uint8_t *data, uint8_t size);
void cmd_lpf(BaseSequentialStream *chp, uint8_t *data, uint8_t size);
void cmd_dutgnd(BaseSequentialStream *chp, uint8_t *data, uint8_t size);
void cmd_int10k(BaseSequentialStream *chp, uint8_t *data, uint8_t size);
void cmd_outgain(BaseSequentialStream *chp, uint8_t *data, uint8_t size);
void cmd_control(BaseSequentialStream *chp, uint8_t *data, uint8_t size);

static const BINProtocol_Command bin_commands[] = {
  {FC_ENABLE,cmd_enable},
  {FC_FORCE,cmd_force},
  {FC_MODE,cmd_mode},
  {FC_RANGE,cmd_range},
  {FC_MEASOUT,cmd_measout},
  {FC_CLAMP,cmd_clamp},
  {FC_CPOLH,cmd_cpolh},
  {FC_SF,cmd_sf},
  {FC_COMPEN,cmd_compen},
  {FC_LPF,cmd_lpf},
  {FC_DUTGND,cmd_dutgnd},
  {FC_INT10K,cmd_int10k},
  {FC_OUTGAIN,cmd_outgain},
  {FC_DAC_X1,cmd_dac_output},
  {FC_DAC_M,cmd_dac_gain},
  {FC_DAC_C,cmd_dac_offset},
  {FC_CONTROL,cmd_control},
  {0x00,0x00}
};

static SerialConfig serialCfg={
  9600
};

static BINProtocolConfig bin_config = {
  (BaseSequentialStream*)&PORT,
  0x01,
  bin_commands
};

static struct boardNvmparam_s{
  uint8_t checksum;
  uint8_t flag;
  uint8_t id;
  uint8_t baudrate;
}board_nvm;


static void load_nvm_default()
{
  board_nvm.id = 0x1;
  board_nvm.baudrate = 0; // 9600
  board_nvm.flag = NVM_FLAG;
}

static void save_nvm()
{
  uint8_t checksum = 0;
  uint8_t sz = sizeof(board_nvm);
  uint8_t *ptr = (uint8_t*)&board_nvm;
  ptr++;
  for(uint8_t i=1;i<sz;i++){
    checksum += *ptr++;
  }
  board_nvm.checksum = checksum;
  nvm_flash_write(OFFSET_NVM_BOARD,(uint8_t*)&board_nvm,sz);
}

static void load_nvm()
{
  uint16_t sz = (sizeof(struct boardNvmparam_s));
  nvm_flash_read(OFFSET_NVM_BOARD,(uint8_t*)&board_nvm,sz);

  uint8_t checksum = 0x0;
  uint8_t *ptr = (uint8_t*)&board_nvm;
  ptr++;
  for(uint8_t i=1;i<sz;i++){
    checksum += *ptr++;
  }
  
  if(checksum != board_nvm.checksum || NVM_FLAG != board_nvm.flag){
    load_nvm_default();
    save_nvm();
  }
}

#define SHELL_WA_SIZE   512
static THD_WORKING_AREA(waShell,SHELL_WA_SIZE);

void task_binProtocolInit(BINProtocolConfig *config)
{
  load_nvm();
  // read board's id from dip switch
  uint8_t id = 0;
  if(palReadPad(GPIOB,6) == PAL_LOW)
    id |= 0x01;
  if(palReadPad(GPIOB,8) == PAL_LOW)
    id |= 0x02;
  if(palReadPad(GPIOB,7) == PAL_LOW)
    id |= 0x04;
  if(palReadPad(GPIOB,9) == PAL_LOW)
    id |= 0x08;
  
//  bin_config.filter_id = board_nvm.id;
  if(id == 0)
    bin_config.filter_id = board_nvm.id;
  else
    bin_config.filter_id = id;

  binProtoRuntime.config = &bin_config;
  //board_nvm.baudrate = 4;
  switch(board_nvm.baudrate){
  case 0: serialCfg.speed = 9600;break;
  case 1: serialCfg.speed = 19200;break;
  case 2: serialCfg.speed = 38400;break;
  case 3: serialCfg.speed = 57600;break;
  case 4: serialCfg.speed = 115200;break;
  default: serialCfg.speed = 9600;break;
  }

  sdStart(&PORT,&serialCfg);
  binProtoRuntime.shelltp = chThdCreateStatic(waShell, sizeof(waShell),NORMALPRIO+1,binProtocolProc,(void*)&bin_config);

}

/**
  * @fn         cmd_board
  * @brief      access boards' system register an pmu register, total 3(sysconfig)+4*3 = 15 bytes per chip,
                total 15*4 = 60 bytes per board
*/

void cmd_board(BaseSequentialStream *chp, uint8_t *data, uint8_t size)
{
  if(size ==  1){
    uint8_t buffer[128];
    uint8_t *p = buffer;
    uint8_t sz = 0;
    *p++ = START_PREFIX;
    *p++ = FC_BOARD_INFO;
    *p++ = board_nvm.id;
    *p++ = 0xFF;
    pmu_fill_board_registers(p);

    sz = (uint8_t)(p - buffer -2);
    *p++ = crc8(&buffer[1],sz);
    *p = END_PREFIX;
    chThdSleepMilliseconds(RESPONSE_DELAY_MS);
    streamWrite(chp, buffer, p-buffer+1);
  }
}

/**
  * @fn         cmd_idn
  * @brief      identify board is present or not
  *
  *@param
  *@retval
*/


/**
  * @fn         cmd_enable
  * @brief      Enable/Disable PMU Channel
  *
  *@param       d[0]: channel, 0xFF for all channels
  *@param       d[1]: 0:disable, 1:enable
  *@retval
*/


void cmd_enable(BaseSequentialStream *chp, uint8_t *data, uint8_t size)
{
  if(size == 2){
    if(data[0] == 0xff){
      for(uint8_t i=0;i<TOTAL_PMU_CHANNEL;i++){
        pmu_set_enable(i,data[1]);
      }
    }
    else{
      pmu_set_enable(data[0],data[1]);
    }
  }
  if(size == 1 || size == 2){
    uint8_t buffer[64];
    uint8_t *p = buffer;
    *p++ = START_PREFIX;
    *p++ = FC_ENABLE;
    *p++ = bin_config.filter_id;
    if(data[0] == 0xFF){
      *p++ = TOTAL_PMU_CHANNEL+1;
      *p++ = 0xFF;
      for(uint8_t i=0;i<TOTAL_PMU_CHANNEL;i++){
        *p++ = pmu_get_enable(i);
      }
    }
    else{
      *p++ = 2;
      *p++ = data[0];
      *p++ =  pmu_get_enable(data[0]);
    }    
    *p++ = crc8(&buffer[1],(p-buffer-2));
    *p = END_PREFIX;
    chThdSleepMilliseconds(RESPONSE_DELAY_MS);
    streamWrite(chp, buffer, p-buffer+1);
  }
}


/**
  * @fn         cmd_force
  * @brief      Set force type of PMU channel
  *
  *@param       d[0]: channel
  *@param       d[1]: 0: GND, 1: DAC
  *@retval
*/
void cmd_force(BaseSequentialStream *chp, uint8_t *data, uint8_t size)
{
  if(size == 2){
    if(data[0] == 0xff){
      for(uint8_t i=0;i<TOTAL_PMU_CHANNEL;i++){
        pmu_set_fin(i,data[1]);
      }
    }
    else{
      pmu_set_fin(data[0],data[1]);
    }
  }
  if(size == 1 || size == 2){
    uint8_t buffer[64];
    uint8_t *p = buffer;
    *p++ = START_PREFIX;
    *p++ = FC_FORCE;
    *p++ = bin_config.filter_id;
    if(data[0] == 0xFF){
      *p++ = TOTAL_PMU_CHANNEL+1;
      *p++ = 0xFF;
      for(uint8_t i=0;i<TOTAL_PMU_CHANNEL;i++){
        *p++ = pmu_get_fin(i);
      }
    }
    else{
      *p++ = 2;
      *p++ = data[0];
      *p++ =  pmu_get_fin(data[0]);
    }    
    *p++ = crc8(&buffer[1],(p-buffer-2));
    *p = END_PREFIX;
    chThdSleepMilliseconds(RESPONSE_DELAY_MS);
    streamWrite(chp, buffer, p-buffer+1);
  }
}

/**
  * @fn         cmd_mode
  * @brief      Set the mode of PMU channel
  *
  *@param       data[0]: channel
  *@param       data[1]: 0: FVMI, 1: FIMV, 2: FVHZ, 3: FIHZ
  *@retval
*/
void cmd_mode(BaseSequentialStream *chp, uint8_t *data, uint8_t size)
{
  if(size == 2){
    if(data[0] == 0xff){
      for(uint8_t i=0;i<TOTAL_PMU_CHANNEL;i++){
        pmu_set_output_mode(i,data[1]);
      }
    }
    else{
      pmu_set_output_mode(data[0],data[1]);
    }
  }
  if(size == 1 || size == 2){
    uint8_t buffer[64];
    uint8_t *p = buffer;
    *p++ = START_PREFIX;
    *p++ = FC_MODE;
    *p++ = bin_config.filter_id;
    if(data[0] == 0xFF){
      *p++ = TOTAL_PMU_CHANNEL+1;
      *p++ = 0xFF;
      for(uint8_t i=0;i<TOTAL_PMU_CHANNEL;i++){
        *p++ = pmu_get_output_mode(i);
      }
    }
    else{
      *p++ = 2;
      *p++ = data[0];
      *p++ =  pmu_get_output_mode(data[0]);
    }    
    *p++ = crc8(&buffer[1],(p-buffer-2));
    *p = END_PREFIX;
    chThdSleepMilliseconds(RESPONSE_DELAY_MS);
    streamWrite(chp, buffer, p-buffer+1);
  }
}

/**
  * @fn         cmd_range
  * @brief      Set to output range of PMU channel
  *
  *@param       d[0]: channel
  *@param       d[1]: FV: 0, FI: 0: 5uA, 1: 20uA, 2: 200uA, 3: 2000uA, 4: External
  *@retval
*/
void cmd_range(BaseSequentialStream *chp, uint8_t *data, uint8_t size)
{
  if(size == 2){
    if(data[0] == 0xff){
      for(uint8_t i=0;i<TOTAL_PMU_CHANNEL;i++){
        pmu_set_crange(i,data[1]);
      }
    }
    else{
      pmu_set_crange(data[0],data[1]);
    }
  }
  if(size == 1 || size == 2){
    uint8_t buffer[64];
    uint8_t *p = buffer;
    *p++ = START_PREFIX;
    *p++ = FC_RANGE;
    *p++ = bin_config.filter_id;
    if(data[0] == 0xFF){
      *p++ = TOTAL_PMU_CHANNEL+1;
      *p++ = 0xFF;
      for(uint8_t i=0;i<TOTAL_PMU_CHANNEL;i++){
        *p++ = pmu_get_crange(i);
      }
    }
    else{
      *p++ = 2;
      *p++ = data[0];
      *p++ =  pmu_get_crange(data[0]);
    }    
    *p++ = crc8(&buffer[1],(p-buffer-2));
    *p = END_PREFIX;
    chThdSleepMilliseconds(RESPONSE_DELAY_MS);
    streamWrite(chp, buffer, p-buffer+1);
  }
}

/**
  * @fn         cmd_measout
  * @brief      Set the PMU channel's measurement output
  *
  *@param       d[0]: channel
  *@param       d[1]: 0: Isense, 1:Vsense, 2: T, 3: HZ
  *@retval
*/
void cmd_measout(BaseSequentialStream *chp, uint8_t *data, uint8_t size)
{
  if(size == 2){
    if(data[0] == 0xff){
      for(uint8_t i=0;i<TOTAL_PMU_CHANNEL;i++){
        pmu_set_measout(i,data[1]);
      }
    }
    else{
      pmu_set_measout(data[0],data[1]);
    }
  }
  if(size == 1 || size == 2){
    uint8_t buffer[64];
    uint8_t *p = buffer;
    *p++ = START_PREFIX;
    *p++ = FC_MEASOUT;
    *p++ = bin_config.filter_id;
    if(data[0] == 0xFF){
      *p++ = TOTAL_PMU_CHANNEL+1;
      *p++ = 0xFF;
      for(uint8_t i=0;i<TOTAL_PMU_CHANNEL;i++){
        *p++ = pmu_get_measout(i);
      }
    }
    else{
      *p++ = 2;
      *p++ = data[0];
      *p++ =  pmu_get_measout(data[0]);
    }    
    *p++ = crc8(&buffer[1],(p-buffer-2));
    *p = END_PREFIX;
    chThdSleepMilliseconds(RESPONSE_DELAY_MS);
    streamWrite(chp, buffer, p-buffer+1);
  }
}

/**
  * @fn
  * @brief
  *
  *@param
  *@retval
*/
void cmd_clamp(BaseSequentialStream *chp, uint8_t *data, uint8_t size)
{
  if(size == 2){
    if(data[0] == 0xff){
      for(uint8_t i=0;i<TOTAL_PMU_CHANNEL;i++){
        pmu_set_clamp(i,data[1]);
      }
    }
    else{
      pmu_set_clamp(data[0],data[1]);
    }
  }
  if(size == 1 || size == 2){
    uint8_t buffer[64];
    uint8_t *p = buffer;
    *p++ = START_PREFIX;
    *p++ = FC_CLAMP;
    *p++ = bin_config.filter_id;
    if(data[0] == 0xFF){
      *p++ = TOTAL_PMU_CHANNEL+1;
      *p++ = 0xFF;
      for(uint8_t i=0;i<TOTAL_PMU_CHANNEL;i++){
        *p++ = pmu_get_clamp(i);
      }
    }
    else{
      *p++ = 2;
      *p++ = data[0];
      *p++ =  pmu_get_clamp(data[0]);
    }    
    *p++ = crc8(&buffer[1],(p-buffer-2));
    *p = END_PREFIX;
    chThdSleepMilliseconds(RESPONSE_DELAY_MS);
    streamWrite(chp, buffer, p-buffer+1);
  }
}

/**
  * @fn
  * @brief
  *
  *@param
  *@retval
*/
void cmd_cpolh(BaseSequentialStream *chp, uint8_t *data, uint8_t size)
{
  if(size == 2){
    if(data[0] == 0xff){
      for(uint8_t i=0;i<TOTAL_PMU_CHANNEL;i++){
        pmu_set_cpolh(i,data[1]);
      }
    }
    else{
      pmu_set_cpolh(data[0],data[1]);
    }
  }
  if(size == 1 || size == 2){
    uint8_t buffer[64];
    uint8_t *p = buffer;
    *p++ = START_PREFIX;
    *p++ = FC_CPOLH;
    *p++ = bin_config.filter_id;
    if(data[0] == 0xFF){
      *p++ = TOTAL_PMU_CHANNEL+1;
      *p++ = 0xFF;
      for(uint8_t i=0;i<TOTAL_PMU_CHANNEL;i++){
        *p++ = pmu_get_cpolh(i);
      }
    }
    else{
      *p++ = 2;
      *p++ = data[0];
      *p++ =  pmu_get_cpolh(data[0]);
    }    
    *p++ = crc8(&buffer[1],(p-buffer-2));
    *p = END_PREFIX;
    chThdSleepMilliseconds(RESPONSE_DELAY_MS);
    streamWrite(chp, buffer, p-buffer+1);
  }
}

/**
  * @fn
  * @brief
  *
  *@param
  *@retval
*/
void cmd_sf(BaseSequentialStream *chp, uint8_t *data, uint8_t size)
{
  if(size == 2){
    if(data[0] == 0xff){
      for(uint8_t i=0;i<TOTAL_PMU_CHANNEL;i++){
        pmu_sys_force(i,data[1]);
      }
    }
    else{
      pmu_sys_force(data[0],data[1]);
    }
  }
  if(size == 1 || size == 2){
    uint8_t buffer[64];
    uint8_t *p = buffer;
    *p++ = START_PREFIX;
    *p++ = FC_SF;
    *p++ = bin_config.filter_id;
    if(data[0] == 0xFF){
      *p++ = TOTAL_PMU_CHANNEL+1;
      *p++ = 0xFF;
      for(uint8_t i=0;i<TOTAL_PMU_CHANNEL;i++){
        *p++ = pmu_get_sys_force(i);
      }
    }
    else{
      *p++ = 2;
      *p++ = data[0];
      *p++ =  pmu_get_sys_force(data[0]);
    }    
    *p++ = crc8(&buffer[1],(p-buffer-2));
    *p = END_PREFIX;
    chThdSleepMilliseconds(RESPONSE_DELAY_MS);
    streamWrite(chp, buffer, p-buffer+1);
  }
}
/**
  * @fn
  * @brief
  *
  *@param
  *@retval
*/
void cmd_compen(BaseSequentialStream *chp, uint8_t *data, uint8_t size)
{
  if(size == 2){
    pmu_set_ccomp(data[1]);
  }
  if(size == 1 || size == 2){
    uint8_t buffer[64];
    uint8_t *p = buffer;
    *p++ = START_PREFIX;
    *p++ = FC_COMPEN;
    *p++ = bin_config.filter_id;
    if(data[0] == 0xFF){
      *p++ = TOTAL_PMU_CHANNEL+1;
      *p++ = 0xFF;
      for(uint8_t i=0;i<TOTAL_PMU_CHANNEL;i++){
        *p++ = pmu_get_ccomp();
      }
    }
    else{
      *p++ = 2;
      *p++ = data[0];
      *p++ =  pmu_get_ccomp();
    }    
    *p++ = crc8(&buffer[1],(p-buffer-2));
    *p = END_PREFIX;
    chThdSleepMilliseconds(RESPONSE_DELAY_MS);
    streamWrite(chp, buffer, p-buffer+1);
  }
}
/**
  * @fn
  * @brief
  *
  *@param
  *@retval
*/
void cmd_lpf(BaseSequentialStream *chp, uint8_t *data, uint8_t size)
{
  if(size == 2){
    pmu_set_frange(data[1]);
  }
  if(size == 1 || size == 2){
    uint8_t buffer[64];
    uint8_t *p = buffer;
    *p++ = START_PREFIX;
    *p++ = FC_LPF;
    *p++ = bin_config.filter_id;
    if(data[0] == 0xFF){
      *p++ = TOTAL_PMU_CHANNEL+1;
      *p++ = 0xFF;
      for(uint8_t i=0;i<TOTAL_PMU_CHANNEL;i++){
        *p++ = pmu_get_frange();
      }
    }
    else{
      *p++ = 2;
      *p++ = data[0];
      *p++ =  pmu_get_frange();
    }    
    *p++ = crc8(&buffer[1],(p-buffer-2));
    *p = END_PREFIX;
    chThdSleepMilliseconds(RESPONSE_DELAY_MS);
    streamWrite(chp, buffer, p-buffer+1);
  }
}
/**
  * @fn
  * @brief
  *
  *@param
  *@retval
*/
void cmd_dutgnd(BaseSequentialStream *chp, uint8_t *data, uint8_t size)
{
  if(size == 2){
    if(data[0] == 0xff){
      for(uint8_t i=0;i<NOF_AD5522;i++){
        pmu_set_dutgnd(i,data[1]);
      }
    }
    else{
      pmu_set_dutgnd(data[0],data[1]);
    }
  }
  if(size == 1 || size == 2){
    uint8_t buffer[64];
    uint8_t *p = buffer;
    *p++ = START_PREFIX;
    *p++ = FC_DUTGND;
    *p++ = bin_config.filter_id;
    if(data[0] == 0xFF){
      *p++ = NOF_AD5522+1;
      *p++ = 0xFF;
      for(uint8_t i=0;i<NOF_AD5522;i++){
        *p++ = pmu_get_dutgnd(i);
      }
    }
    else{
      *p++ = 2;
      *p++ = data[0];
      *p++ =  pmu_get_dutgnd(data[0]);
    }    
    *p++ = crc8(&buffer[1],(p-buffer-2));
    *p = END_PREFIX;
    chThdSleepMilliseconds(RESPONSE_DELAY_MS);
    streamWrite(chp, buffer, p-buffer+1);
  }
}
/**
  * @fn
  * @brief
  *
  *@param
  *@retval
*/
void cmd_int10k(BaseSequentialStream *chp, uint8_t *data, uint8_t size)
{
  if(size == 2){
    if(data[0] == 0xff){
      for(uint8_t i=0;i<NOF_AD5522;i++){
        pmu_set_int10k(i,data[1]);
      }
    }
    else{
      pmu_set_int10k(data[0],data[1]);
    }
  }
  if(size == 1 || size == 2){
    uint8_t buffer[64];
    uint8_t *p = buffer;
    *p++ = START_PREFIX;
    *p++ = FC_COMPEN;
    *p++ = bin_config.filter_id;
    if(data[0] == 0xFF){
      *p++ = NOF_AD5522+1;
      *p++ = 0xFF;
      for(uint8_t i=0;i<NOF_AD5522;i++){
        *p++ = pmu_get_int10k(i);
      }
    }
    else{
      *p++ = 2;
      *p++ = data[0];
      *p++ =  pmu_get_int10k(data[0]);
    }    
    *p++ = crc8(&buffer[1],(p-buffer-2));
    *p = END_PREFIX;
    chThdSleepMilliseconds(RESPONSE_DELAY_MS);
    streamWrite(chp, buffer, p-buffer+1);
  }
}
/**
  * @fn
  * @brief
  *
  *@param
  *@retval
*/
void cmd_outgain(BaseSequentialStream *chp, uint8_t *data, uint8_t size)
{
  if(size == 2){
    if(data[0] == 0xff){
      for(uint8_t i=0;i<NOF_AD5522;i++){
        pmu_set_outgain(i,data[1]);
      }
    }
    else{
      pmu_set_outgain(data[0],data[1]);
    }
  }
  if(size == 1 || size == 2){
    uint8_t buffer[64];
    uint8_t *p = buffer;
    *p++ = START_PREFIX;
    *p++ = FC_COMPEN;
    *p++ = bin_config.filter_id;
    if(data[0] == 0xFF){
      *p++ = NOF_AD5522+1;
      *p++ = 0xFF;
      for(uint8_t i=0;i<NOF_AD5522;i++){
        *p++ = pmu_get_outgain(i);
      }
    }
    else{
      *p++ = 2;
      *p++ = data[0];
      *p++ =  pmu_get_outgain(data[0]);
    }    
    *p++ = crc8(&buffer[1],(p-buffer-2));
    *p = END_PREFIX;
    chThdSleepMilliseconds(RESPONSE_DELAY_MS);
    streamWrite(chp, buffer, p-buffer+1);
  }
}
/**
  * @fn
  * @brief
  *
  *@param
  *@retval
*/
void cmd_dac_output(BaseSequentialStream *chp, uint8_t *data, uint8_t size)
{
  uint8_t ch = data[0];
  uint8_t r = data[1];
  uint8_t nofCh = (size - 2)/2;
  uint16_t value;
  for(uint8_t i=0;i<nofCh;i++){
    value = (data[i*2+2]) | (data[i*2+3] << 8); // c# is be
    pmu_set_dac_output(ch+i,data[1],value);
  }
//  if(size == 4){
//    pmu_set_dac_output(ch,data[1],value);
//  }
  // prepare response
  uint8_t buffer[128];
  uint8_t *p = buffer;
  *p++ = START_PREFIX;
  *p++ = FC_DAC_X1;
  *p++ = bin_config.filter_id;
  if(ch == 0xFF){
    *p++ = TOTAL_PMU_CHANNEL*2+2;
    *p++ = 0xFF;
    *p++ = r; // dac's id
    for(uint8_t i=0;i<TOTAL_PMU_CHANNEL;i++){
      value = pmu_get_dac_output_cached(i,r);
      *p++ = (value & 0xff);
      *p++ = (value >> 8);
    }
  }
  else{
    *p++ = 4;
    *p++ = ch;
    *p++ = r; // dac's id
    value = pmu_get_dac_output_cached(ch,r);
    *p++ = (value & 0xff);
    *p++ = (value >> 8);
  }    
  *p++ = crc8(&buffer[1],(p-buffer-2));
  *p = END_PREFIX;
  chThdSleepMilliseconds(RESPONSE_DELAY_MS);
  streamWrite(chp, buffer, p-buffer+1);
  
}

/**
  * @fn
  * @brief
  *
  *@param
  *@retval
*/
void cmd_dac_gain(BaseSequentialStream *chp, uint8_t *data, uint8_t size)
{
  uint8_t ch = data[0];
  uint8_t r = data[1];
  uint16_t value = (data[2]) | (data[3] << 8); // c# is be
  if(size == 4){
    pmu_set_dac_gain(ch,r,value);
  }
  // prepare response
  uint8_t buffer[128];
  uint8_t *p = buffer;
  *p++ = START_PREFIX;
  *p++ = FC_DAC_M;
  *p++ = bin_config.filter_id;
  if(ch == 0xFF){
    *p++ = TOTAL_PMU_CHANNEL*2+2;
    *p++ = 0xFF;
    *p++ = r; // dac's id
    for(uint8_t i=0;i<TOTAL_PMU_CHANNEL;i++){
      value = pmu_get_dac_gain_cached(i,r);
      *p++ = (value & 0xff);
      *p++ = (value >> 8);
    }
  }
  else{
    *p++ = 4;
    *p++ = ch;
    *p++ = r; // dac's id
    value = pmu_get_dac_gain_cached(ch,r);
    *p++ = (value & 0xff);
    *p++ = (value >> 8);
  }    
  *p++ = crc8(&buffer[1],(p-buffer-2));
  *p = END_PREFIX;
  chThdSleepMilliseconds(RESPONSE_DELAY_MS);
  streamWrite(chp, buffer, p-buffer+1);
}

/**
  * @fn
  * @brief
  *
  *@param
  *@retval
*/
void cmd_dac_offset(BaseSequentialStream *chp, uint8_t *data, uint8_t size)
{
  uint8_t ch = data[0];
  uint8_t r = data[1];
  uint16_t value = (data[2]) | (data[3] << 8); // c# is be
  if(size == 4){
    pmu_set_dac_offset(ch,r,value);
  }
  // prepare response
  uint8_t buffer[128];
  uint8_t *p = buffer;
  *p++ = START_PREFIX;
  *p++ = FC_DAC_C;
  *p++ = bin_config.filter_id;
  if(ch == 0xFF){
    *p++ = TOTAL_PMU_CHANNEL*2+2;
    *p++ = 0xFF;
    *p++ = r; // dac's id
    for(uint8_t i=0;i<TOTAL_PMU_CHANNEL;i++){
      value = pmu_get_dac_offset_cached(i,r);
      *p++ = (value & 0xff);
      *p++ = (value >> 8);
    }
  }
  else{
    *p++ = 4;
    *p++ = ch;
    *p++ = r; // dac's id
    value = pmu_get_dac_offset_cached(ch,r);
    *p++ = (value & 0xff);
    *p++ = (value >> 8);
  }    
  *p++ = crc8(&buffer[1],(p-buffer-2));
  *p = END_PREFIX;
  chThdSleepMilliseconds(RESPONSE_DELAY_MS);
  streamWrite(chp, buffer, p-buffer+1);
}

/**
  * @fn
  * @brief
  *
  *@param
  *@retval
*/
void cmd_control(BaseSequentialStream *chp, uint8_t *data, uint8_t size)
{
  if(size == 2){
    switch(data[0]){
    case 0x55:
      pmu_reg_flush(data[1]);
      break;
    case 0x53: 
      if(data[1] == 0x01){
        pmu_nvm_save();
      }
      else if(data[1] == 0x02){
        save_nvm();
      }
      break;
    case 0x42:
      board_nvm.baudrate = data[1];
      break;
    case 0x4c:
      if(data[1] == 0x01){
        pmu_nvm_load_default();
      }
      else if(data[1] == 0x02){
        load_nvm_default();
      }
      break;
    case 0x4d:
      pmu_set_init_state(data[1]);
      break;
    case 0x49:
      board_nvm.id = data[1];
      break;
    case 0xAA:
      break;
    default:break;
    }
  }
  // prepare response
  uint8_t buffer[128];
  uint8_t *p = buffer;
  *p++ = START_PREFIX;
  *p++ = FC_CONTROL;
  *p++ = bin_config.filter_id;
  *p++ = 2; // length
  *p++ = data[0];
  switch(data[0]){
  case 0x55:
  case 0x53: 
  case 0x4c:
    *p++ = 0x01;
    break;
  case 0x4d:
    *p++ = pmu_get_init_state();
    break;
  case 0x49:
    *p++ = board_nvm.id;
    break;
  case 0x42:
    *p++ = board_nvm.baudrate;
    break;
  case 0xAA:
    *p++ = bin_config.filter_id;
    break;
  default:break;
  }
  
  *p++ = crc8(&buffer[1],(p-buffer-2));
  *p = END_PREFIX;
  chThdSleepMilliseconds(RESPONSE_DELAY_MS);
  streamWrite(chp, buffer, p-buffer+1);
}