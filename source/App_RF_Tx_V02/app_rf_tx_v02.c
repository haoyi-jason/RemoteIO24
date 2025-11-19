#include "ch.h"
#include "hal.h"
#include "../drivers/bc3601.h"
#include "shell.h"
#include "../rf_shell/shell_cmd_menu.h"
#include "../rf_task/rf_task.h"
#include "../drivers/bc3601.h"
#include "../drivers/dev_bc3601.h"
#include "../drivers/nvm.h"
#include "nvm_config.h"
#include "../rf_shell/database.h"

//#define BOARD_A2
#define BOARD_B2


#define ADC_GRP1_NUM_CHANNELS   2
#define ADC_GRP1_BUF_DEPTH      8

#define LINE_TX_ACT     PAL_LINE(GPIOC,6)
#define LINE_LBT_ACT     PAL_LINE(GPIOC,7)
#ifdef BOARD_A2
#undef VR_POWER_LINE
#define VR_POWER_LINE   PAL_LINE(GPIOB,11)
#endif
#ifdef BOARD_B2
#undef VR_POWER_LINE
#define VR_POWER_LINE   PAL_LINE(GPIOA,3)
#endif
#define VR_POWER_DIS()  palSetLine(VR_POWER_LINE)
#define VR_POWER_EN() palClearLine(VR_POWER_LINE) 
#define NVM_FLAG        0xAA


static const SPIConfig spicfg = {
  false,
  NULL,
  NULL,//GPIOB,
  NULL,//GPIOB_SPI1_CS,
  SPI_CR1_BR_2  
};

static bc3601_config_t config = {
  &SPID2,
  &spicfg,
  GPIOB,
  12
};

static BC3601Driver bc3601;

struct _deviceConfig {
  uint8_t flag;
  uint16_t selfAddr;
  uint16_t destAddr;
  uint8_t opMode;  // 1: rf switch tx, 2: rf switch rx, 3: rf i/o tx, 4: rf i/o rx
  uint8_t txIntervalMs; // tx interval, in ms
  uint16_t txCounts; // number of times to transmit packet
  struct _rx_control_config rx_control;
  uint16_t timeout_ms;
  uint8_t reserved[13]; // reserve total 64-bytes for this struct
};

struct _rfConfig{
  uint8_t dataRate; // 0 to 6, 2/5/10/25/50/125/250K
  uint8_t txPower;
  float frequency; 
  uint8_t reserved[58]; // reserve 64-bytes for this struct
};

struct _nvm{
  struct _deviceConfig deviceConfig;
  struct _rfConfig rfConfig;
  uint16_t tx_dio;
};

//struct _nvm nvmParam, *rfNvmParam;

static struct{
  thread_t *self;
  thread_t *mainThread;
  thread_t *usbtrx;
  thread_reference_t ref;
  uint8_t rssi;
  uint16_t rf_ctrl;
  uint16_t cycles;
  uint8_t stage;
  uint16_t rxLost;
  uint8_t taskFinished;
  uint16_t dio_state;
  uint16_t ain_value;
  uint8_t rfLinkFail;
  uint8_t userMode;
}runTime;

typedef struct {
  uint8_t sz;
  uint8_t data[64];
}_txPacket;

static _txPacket txPacket;

typedef struct{
  uint16_t dio; // bit 0~9 for action, bit 15 for switch
  uint16_t advalue;
  uint16_t srcAddr;
  uint16_t dstAddr;
  uint8_t rsv1;
  uint8_t checksum;
}_rfPacket;

static void load_settings()
{
//  uint16_t nvmSz = sizeof(nvmParam);
//  if(nvmSz > SZ_NVM_CONFIG){
//    // error
//    while(1);
//  }
//  nvm_flash_read(OFFSET_NVM_BOARD,(uint8_t*)&nvmParam,nvmSz);
//  if(nvmParam.deviceConfig.flag != NVM_FLAG){
//    nvmParam.deviceConfig.flag = NVM_FLAG;
//    nvmParam.deviceConfig.selfAddr = 0x04;
//    nvmParam.deviceConfig.destAddr = 0x04;
//    nvmParam.deviceConfig.opMode = 4;
//    nvmParam.deviceConfig.txIntervalMs = 50;
//    nvmParam.deviceConfig.txCounts = 90;
//    
//    nvmParam.rfConfig.dataRate = 2; // 10k
//    nvmParam.rfConfig.frequency = 915.;
//    nvmParam.rfConfig.txPower = 5; // 10 dbm
// 
//    nvmParam.tx_dio = 0xC000; // default tx pattern
//    
//    nvmParam.deviceConfig.rx_control.vr_map[0].angle = 0;
//    nvmParam.deviceConfig.rx_control.vr_map[0].raw = 1200;
//    nvmParam.deviceConfig.rx_control.vr_map[1].angle = 30;
//    nvmParam.deviceConfig.rx_control.vr_map[1].raw = 1600;
//    nvmParam.deviceConfig.rx_control.vr_map[2].angle = 170;
//    nvmParam.deviceConfig.rx_control.vr_map[2].raw = 2600;
//    nvmParam.deviceConfig.rx_control.vr_map[3].angle = 200;
//    nvmParam.deviceConfig.rx_control.vr_map[3].raw = 3000;
//    
//    nvmParam.deviceConfig.rx_control.pulsePerDegree = (320*10)/360.;
//    nvmParam.deviceConfig.rx_control.min_pps = 500;
//    nvmParam.deviceConfig.rx_control.max_pps = 1500;
//    nvm_flash_write(OFFSET_NVM_BOARD,(uint8_t*)&nvmParam,nvmSz);
//  }
}

void rf_save_nvm()
{
  //nvm_flash_write(OFFSET_NVM_BOARD,(uint8_t*)&nvmParam,sizeof(nvmParam));
}

uint8_t checksum(uint8_t *d, uint8_t n)
{
  uint8_t sum = 0;
  for(uint8_t i=0;i<n;i++){
    sum += *d++;
  }
  return sum;
}

//#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(512)
//static THD_WORKING_AREA(waShell,SHELL_WA_SIZE);


static adcsample_t samples[ADC_GRP1_NUM_CHANNELS*ADC_GRP1_BUF_DEPTH];

static void adccallback(ADCDriver *adcp);

static void adcerror(ADCDriver *adcp, adcerror_t err);
static  virtual_timer_t vt_blink;

static void adccallback(ADCDriver *adcp)
{
//  while(1);
}

static void adcerror(ADCDriver *adcp, adcerror_t err)
{
//  while(1);
}

static const ADCConversionGroup adcgrpcfg = {
  TRUE,
  ADC_GRP1_NUM_CHANNELS,
  adccallback,
  adcerror,
  0,
  ADC_CR2_SWSTART,
  0,
  ADC_SMPR2_SMP_AN0(ADC_SAMPLE_239P5) | ADC_SMPR2_SMP_AN1(ADC_SAMPLE_239P5),
  ADC_SQR1_NUM_CH(ADC_GRP1_NUM_CHANNELS),
  0,
#ifdef BOARD_A2
  ADC_SQR3_SQ1_N(ADC_CHANNEL_IN8) | ADC_SQR3_SQ2_N(ADC_CHANNEL_IN9)
#endif
#ifdef BOARD_B2
  ADC_SQR3_SQ1_N(ADC_CHANNEL_IN2) | ADC_SQR3_SQ2_N(ADC_CHANNEL_IN1)
#endif
};


static const SerialConfig ser_cfg = {
  115200
};

typedef struct{
  ioportid_t port;
  ioportmask_t line;
}_gpio_def_t;

#ifdef BOARD_A2
static _gpio_def_t digital_in[] = {
  {GPIOA,15},
  {GPIOA,1},
  {GPIOA,2},
  {GPIOA,3},
  {GPIOA,4},
  {GPIOA,5},
  {GPIOA,6},
  {GPIOA,7},
};

static _gpio_def_t digital_out[] = {
  {GPIOB,11},    // VR power
  {GPIOC,6},    // led
  {GPIOC,7},    // led
};
#endif

#ifdef BOARD_B2
static _gpio_def_t digital_in[] = {
  {GPIOA,6},
  {GPIOA,7},
  {GPIOB,0},
  {GPIOB,1},
  {GPIOB,9},
  {GPIOB,8},
  {GPIOB,7},
  {GPIOB,6},
};

static _gpio_def_t digital_out[] = {
  {GPIOA,3},
  {GPIOC,6},
  {GPIOC,7},  
};
#endif

//static void bc3601_set_addr()
//{
//  uint8_t reg = 0;
//  // set address, 14-bit format
//  reg = nvmParam2.device_config.selfAddr & 0x3F;
//  BC3601_REG_WRITE(&bc3601,HEADER_ADDR0_REGS,&reg); 
//  BC3601_REG_READ(&bc3601,HEADER_ADDR0_REGS,&reg); 
//  reg = (nvmParam2.device_config.selfAddr>>8);
//  BC3601_REG_WRITE(&bc3601,HEADER_ADDR1_REGS,&reg); 
//  BC3601_REG_READ(&bc3601,HEADER_ADDR1_REGS,&reg);
//}

static void blink_cb(void *arg)
{
  chSysLockFromISR();
  palToggleLine(LINE_TX_ACT);
  chVTSetI(&vt_blink,TIME_MS2I(500),blink_cb,NULL);
  chSysUnlockFromISR();
}

static uint16_t read_keys()
{
  uint16_t keys = 0;
  for(uint8_t i=0;i<8;i++){
    if(palReadPad(digital_in[i].port,digital_in[i].line) == PAL_LOW){
      keys |= (1 << i);
    }
  }
  return keys;
}

static void process_ctrl_tx()
{
  uint16_t ch_sum[2] = {0,0};
  for(uint8_t i=0;i<ADC_GRP1_BUF_DEPTH;i++){
    ch_sum[0] += samples[i*2];
    ch_sum[1] += samples[i*2+1];
  }
  
  ch_sum[0] /= ADC_GRP1_BUF_DEPTH;
  ch_sum[1] /= ADC_GRP1_BUF_DEPTH;
  runTime.ain_value = ch_sum[0];
  
  // check if low battery
  /*
    lbt voltage         code
    5.0                 (5-1.2)/10.56*4096 = 1940
    4.8                 (4.8 - 1.2)*10.56/4096 = 1400
  */
  
  if(ch_sum[1] < 1300){
    palSetLine(LINE_LBT_ACT);
  }
  else if(ch_sum[1] > 1400){
    palClearLine(LINE_LBT_ACT);
  }
  runTime.dio_state = read_keys();
}

static void process_switch_tx()
{
  
}

static THD_WORKING_AREA(waRemoteIO,4096);
static THD_FUNCTION(procRemoteIO ,p)
{
  BC3601Driver *dev = &bc3601;
  load_settings();
  runTime.mainThread = chRegFindThreadByName("Main");
  adcStart(&ADCD1,NULL);
  adcStartConversion(&ADCD1,&adcgrpcfg,samples,ADC_GRP1_BUF_DEPTH);  
  ADCD1.adc->CR2 |= 0x0;

  bc3601.txPower = nvmParam2.device_config.txPower;
  bc3601.dataRate = nvmParam2.device_config.dataRate;
  bc3601.frequency = (float)(nvmParam2.device_config.frequency + (nvmParam2.device_config.selfAddr%32)*0.5);
  bc3601.destAddr = nvmParam2.device_config.destAddr;
  
  
  dev_bc3601Init(&bc3601,&config);
  _rfPacket *rfp = (_rfPacket*)(&txPacket.data[0]);
  //rfp.dio = SWITCH_TX_PATTERN;
  rfp->dio = nvmParam2.device_config.txPattern;
  rfp->advalue = 0x0;
  rfp->srcAddr = nvmParam2.device_config.selfAddr;
  rfp->dstAddr = nvmParam2.device_config.destAddr;
  rfp->checksum = checksum((uint8_t*)&rfp,sizeof(_rfPacket)-1);
  uint8_t reg;
  uint8_t pktSz = sizeof(_rfPacket);
  //uint16_t cycles = 0;
  txPacket.sz = pktSz+1;
  //memcpy(txPacket.data,(uint8_t*)&rfp, pktSz);
  bool bStop = false;
  systime_t idleStart = chVTGetSystemTimeX();
  systime_t idleMs = 0;

  chVTObjectInit(&vt_blink);
  chVTSet(&vt_blink,TIME_MS2I(500), blink_cb,NULL);
    
    while(!bStop){
    //cycles++;
    BC3601_RESET_TXFIFO(dev);
    reg = 0;
    // read analog input and i/o state
    if(runTime.dio_state == 0x0){
      systime_t time = chVTGetSystemTimeX();
      idleMs = TIME_I2MS(chVTTimeElapsedSinceX(idleStart));
      //cycles++;
    }
    else{
      idleStart = chVTGetSystemTimeX();
      //cycles = 0;
    }
    if((idleMs > 2000) && (runTime.userMode == 0)){
      bStop = true;
    }
    rfp->dio = runTime.dio_state;
    rfp->advalue = runTime.ain_value;
    rfp->checksum = checksum((uint8_t*)rfp,sizeof(_rfPacket)-1);
    
    BC3601_SET_TX_PAYLOAD_SADDR(dev,&reg);
    //BC3601_SET_TX_PAYLOAD_WIDTH(dev,&pktSz);
    BC3601_FIFO_WRITE(dev,(uint8_t*)&txPacket,txPacket.sz);
    BC3601_TX_MODE(dev);
//    rfp.advalue++;
    chThdSleepMilliseconds(nvmParam2.device_config.txIntervalMs);
    if(chThdShouldTerminateX() && (runTime.userMode == 0)){
      bStop = true;
    }
    process_ctrl_tx();
  }
  BC3601_STBY(dev);
  BC3601_DEEP_SLEEP(dev);
  chVTReset(&vt_blink);
  chEvtSignal(runTime.mainThread,EV_TX_DONE);
  chThdExit(0);
 
}

static void gpio_init()
{
  uint8_t i;
  for(i=0;i<8;i++){
    palSetPadMode(digital_in[i].port, digital_in[i].line, PAL_MODE_INPUT_PULLUP);
  }
  
  palSetPadMode(digital_out[0].port, digital_out[0].line, PAL_MODE_OUTPUT_OPENDRAIN);
  palSetPadMode(digital_out[1].port, digital_out[1].line, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(digital_out[2].port, digital_out[2].line, PAL_MODE_OUTPUT_PUSHPULL);

#ifdef BOARD_A2
  palSetPadMode(GPIOB,0, PAL_MODE_INPUT_ANALOG);
  palSetPadMode(GPIOB,1, PAL_MODE_INPUT_ANALOG);
#endif
#ifdef BOARD_B2
  palSetPadMode(GPIOA,1, PAL_MODE_INPUT_ANALOG);
  palSetPadMode(GPIOA,2, PAL_MODE_INPUT_ANALOG);
#endif
}

//static void console_cb(uint8_t id, uint32_t value)
//{
//  switch(id){
//  case CONFIG_SRC_ID: // id
//    nvmParam.deviceConfig.selfAddr = value;
//    break;
//  case CONFIG_DST_ID:
//    nvmParam.deviceConfig.destAddr = value;
//    break;
//  case CONFIG_FREQUENCY: // frequency
//    nvmParam.rfConfig.frequency = (float)(value / 1000.);
//    break;
//  case CONFIG_RF_POWER: // rf power
//    nvmParam.rfConfig.txPower = value;
//    break;
//  case CONFIG_TX_INTERVAL: // interval
//    nvmParam.deviceConfig.txIntervalMs = value;
//    break;
//  case CONFIG_RX_TIMEOUT: // timeout
//    nvmParam.deviceConfig.timeout_ms = value;
//    break;
//  case 99:
//    rf_save_nvm();
//    break;
//  default:break;
//  }
//}

int main()
{
  bool bStop = false;
  bool control_mode = true;
  //bool setupMode = false;

  halInit();
  chSysInit();
  AFIO->MAPR |= (AFIO_MAP_SWJTAG_CONF_JTAGDISABLE);
  
  palSetPadMode(GPIOC, 9, PAL_MODE_INPUT_PULLUP);
  chThdSleepMilliseconds(100);
  //setupMode = true;
  if((palReadPad(GPIOC,9) == PAL_LOW)){
    //setupMode = true;
    runTime.userMode = 1;
  }
  database_init();
  gpio_init();
  chRegSetThreadName("Main");
  
  VR_POWER_EN();

  //rf_task_init();
  
  // todo : start serial shell via usart1
  sdStart(&SD1,&ser_cfg);
  runTime.userMode = 1;
  //thread_t *shelltp = chThdCreateStatic(waShell,sizeof(waShell),NORMALPRIO,shellThread,(void*)&shell_cfg);
  thread_t *rftp = chThdCreateStatic(waRemoteIO,sizeof(waRemoteIO),NORMALPRIO,procRemoteIO,NULL);
//  chSysLock();
//  chThdSuspendS(&runTime.ref);
//  chSysUnlock();
//  shell_cmd_menu_init((BaseSequentialStream*)&SD1,console_cb);
  
  while(!bStop)
  {
    eventmask_t evt = chEvtWaitAnyTimeout(ALL_EVENTS,TIME_MS2I(10));
    if(evt & EV_TX_DATA_REQUEST){
      if(control_mode){
        process_ctrl_tx();
      }
      else{
        process_switch_tx();
      }
    }
    
    if(evt & EV_TX_DONE){
      // enter sleep mode
      if(runTime.userMode == 0){
        bStop = true;
        //chThdTerminate(shelltp);
        //chThdWait(shelltp);
        //shelltp = NULL;
        palClearLine(LINE_TX_ACT);
      }
    }
    
    if(chThdShouldTerminateX()){
      bStop = true;      
    }
  }
  VR_POWER_DIS();
  // enter sleep mode
  /*
    config exti PA0 falling edge trigger to wakeup 
  */
  EXTI->PR = EXTI_INTEN_LN0;
  EXTI->IMR = EXTI_INTEN_LN0;
  EXTI->EMR = EXTI_EVTEN_LN0;
  EXTI->FTSR = EXTI_FTRSEL_LN0;
  EXTI->RTSR = EXTI_FTRSEL_LN0;
  AFIO->EXTICR[0] &= 0x0FFF;
  AFIO->EXTICR[0] |= AFIO_EXTIC1_EXTINT0_PTA;

  PWR->CR |= PWR_CR_CLWUF | PWR_CR_CLSBF;
  PWR->CR &= ~PWR_CR_PVDS;

  PWR->CR |= (PWR_CR_PDDS); //Standby(0), Sleep(1)
  PWR->CSR |= PWR_CRSTS_WUPEN;
  SCB->SCR |= (SCB_SCR_SLEEPDEEP_Msk);

  chSysDisable();
  __disable_irq();

  __SEV();
  __WFE();  
  __WFE();  

  // wakeup... 
  __enable_irq();
  NVIC_SystemReset(); 

  
  
}


