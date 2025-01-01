#include "ch.h"
#include "hal.h"
#include "stepper.h"
#include "../drivers/bc3601.h"
#include "rf_task.h"
#include "stepper_task.h"
#include "../drivers/usbcdc_task.h"
#include "shell.h"
#include "shell_command.h"
#include "../drivers/usbcfg.h"

#define ADC_GRP1_NUM_CHANNELS   2
#define ADC_GRP1_BUF_DEPTH      8

static adcsample_t samples[ADC_GRP1_NUM_CHANNELS*ADC_GRP1_BUF_DEPTH];

static void adccallback(ADCDriver *adcp);

static void adcerror(ADCDriver *adcp, adcerror_t err);

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
  ADC_SQR3_SQ1_N(ADC_CHANNEL_IN4) | ADC_SQR3_SQ2_N(ADC_CHANNEL_IN1)
};

static const ShellCommand commands[] = {
  //{"write", cmd_write},
  //{"writeX", cmd_writeHex},
  //{"read", cmd_read},
  {"ppr", cmd_ppr},
  {"dest", cmd_addr},
  {"sadr", cmd_selfaddr},
  {"power", cmd_power},
  {"freq", cmd_freq},
  {"rate", cmd_rate},
  {"mode", cmd_mode},
  {"save", cmd_save},
  {"pattern", cmd_default_io_pattern},
  {"polt", cmd_poll_interval},
  {"polc", cmd_poll_count},
  {"idn", cmd_idn},
  {"vrmap", cmd_vrMap},
  {"spdmap", cmd_spdMap},
  {"txsim",cmd_txsim},
  {NULL, NULL}
};

static const ShellConfig shell_cfg = {
  (BaseSequentialStream *)&SDU1,
  commands
};

static void processTX1();
static void processTX2();
static void processRX1();
static void processRX2();

//static BC3601Driver bc3601;
//static const SPIConfig spicfg = {
//  false,
//  NULL,
//  NULL,//GPIOB,
//  NULL,//GPIOB_SPI1_CS,
//  SPI_CR1_BR_2  
//};
//
//static bc3601_config_t config = {
//  &SPID2,
//  &spicfg,
//  GPIOB,
//  12
//};

//struct _deviceConfig{
//  uint8_t flag;
//  uint16_t selfAddr;
//  uint16_t destAddr;
//  uint8_t opMode;  // 1: rf switch tx, 2: rf switch rx
//  uint8_t txIntervalMs; // tx interval, in ms
//  uint16_t txCounts; // number of times to transmit packet
//  uint8_t reserved[55]; // reserve total 64-bytes for this struct
//};
//
//struct _rfConfig{
//  uint8_t dataRate; // 0 to 6, 2/5/10/25/50/125/250K
//  uint8_t txPower;
//  float frequency; 
//  uint8_t reserved[58]; // reserve 64-bytes for this struct
//};
//
//
//struct _nvm{
//  struct _deviceConfig deviceConfig;
//  struct _rfConfig rfConfig;
//  uint16_t tx_dio;
//};

//static struct{
//  thread_t *self;
//  thread_t *usbtrx;
//  thread_reference_t ref;
//  uint8_t rssi;
//  uint16_t rf_ctrl;
//  uint16_t cycles;
//  uint8_t stage;
//  uint16_t rxLost;
//  uint8_t taskFinished;
//}runTime;

//typedef struct {
//  uint8_t sz;
//  uint8_t data[64];
//}_txPacket;
//
//static _txPacket txPacket;
//typedef struct{
//  uint16_t dio; // bit 0~9 for action, bit 15 for switch
//  uint16_t advalue;
//  uint16_t srcAddr;
//  uint16_t dstAddr;
//  uint8_t rsv1;
//  uint8_t checksum;
//}_rfPacket;

//static void load_settings()
//{
//  uint16_t nvmSz = sizeof(nvmParam);
////  if(nvmSz > SZ_NVM_CONFIG){
////    // error
////    while(1);
////  }
//  nvm_flash_read(OFFSET_NVM_BOARD,(uint8_t*)&nvmParam,nvmSz);
//  if(nvmParam.deviceConfig.flag != NVM_FLAG){
//    nvmParam.deviceConfig.flag = NVM_FLAG;
//    nvmParam.deviceConfig.selfAddr = 0x01;
//    nvmParam.deviceConfig.destAddr = 0x01;
//    nvmParam.deviceConfig.opMode = 1;
//    nvmParam.deviceConfig.txIntervalMs = 100;
//    nvmParam.deviceConfig.txCounts = 90;
//    
//    nvmParam.rfConfig.dataRate = 2; // 10k
//    nvmParam.rfConfig.frequency = 915.;
//    nvmParam.rfConfig.txPower = 2; // 10 dbm
// 
//    nvmParam.tx_dio = 0xC000; // default tx pattern
//    
//    nvm_flash_write(OFFSET_NVM_BOARD,(uint8_t*)&nvmParam,nvmSz);
//  }
//}

//uint8_t checksum(uint8_t *d, uint8_t n)
//{
//  uint8_t sum = 0;
//  for(uint8_t i=0;i<n;i++){
//    sum += *d++;
//  }
//  return sum;
//}

//static void set_rf_addr()
//{
//  uint8_t reg = 0;
//  // set address, 14-bit format
//  reg = nvmParam.deviceConfig.selfAddr & 0x3F;
//  BC3601_REG_WRITE(&bc3601,HEADER_ADDR0_REGS,&reg); 
//  BC3601_REG_READ(&bc3601,HEADER_ADDR0_REGS,&reg); 
//  reg = (nvmParam.deviceConfig.selfAddr>>8);
//  BC3601_REG_WRITE(&bc3601,HEADER_ADDR1_REGS,&reg); 
//  BC3601_REG_READ(&bc3601,HEADER_ADDR1_REGS,&reg);
//}












//void receiver_task_init()
//{
//  bc3601.txPower = 1;
//  bc3601.dataRate = 0;
//  bc3601.frequency = 915;
//  bc3601.destAddr = 1;
//  bc3601Start(&bc3601,&config);
//  
//}

typedef struct{
  ioportid_t port;
  ioportmask_t line;
}_dio_def_t;

_dio_def_t digital_in_tx[] = {
  {GPIOA,8},
  {GPIOC,8},
  {GPIOC,6},
  {GPIOB,10},
  {GPIOB,1},
  {GPIOA,7},
  {GPIOA,5},
  {GPIOA,3},
  {GPIOB,3},
  {GPIOB,5}
};

_dio_def_t digital_out_tx[] = {
  {GPIOB,2},
  {GPIOB,11},
};

_dio_def_t digital_out_rx[] = {
  {GPIOB,7},
  {GPIOB,8},
  {GPIOB,5},
  {GPIOB,6},
  {GPIOB,3},
  {GPIOB,4},
  {GPIOA,1},
  {GPIOA,0},
  {GPIOA,3},
  {GPIOA,2},
  {GPIOA,5},
  {GPIOA,10},
  {GPIOA,8},
  {GPIOC,8},
};
_dio_def_t digital_in_rx[] = {
  {GPIOB,1},
  {GPIOB,12},
  {GPIOB,10},
};

static void processRX_DO(uint16_t value)
{
  uint16_t mask;
  for(uint8_t i=0;i<10;i++){
    mask = (1 << i);
    if((mask & value) == 0){
      palClearPad(digital_out_rx[i].port,digital_out_rx[i].line);
    }
    else{
      palSetPad(digital_out_rx[i].port,digital_out_rx[i].line);
    }
  }
}

static uint16_t processDI()
{
  uint16_t ret = 0x0;
  for(uint8_t i=0;i<10;i++){
    if(palReadPad(digital_in_tx[i].port, digital_in_tx[i].line) == PAL_LOW){
      ret |= (1 << i);
    }
  }
  
  return ret;
}

static void processTX1()
{
}
static void processRX1()
{
  
}
static void processTX2()
{
  rf_write_ctrl_state(processDI());
  
  uint16_t ch_sum[2] = {0,0};
  for(uint8_t i=0;i<ADC_GRP1_BUF_DEPTH;i++){
    ch_sum[0] += samples[i*2];
    ch_sum[1] += samples[i*2+1];
  }
  
  ch_sum[0] /= ADC_GRP1_BUF_DEPTH;
  ch_sum[1] /= ADC_GRP1_BUF_DEPTH;
  rf_write_analog_state(ch_sum[0]);
  
//  adcStartConversion(&ADCD1,&adcgrpcfg,samples,ADC_GRP1_BUF_DEPTH);  
}
static void processRX2()
{
  uint16_t dio = read_ctrl_state();
  processRX_DO(dio);
  // process stepper
  uint16_t aio = read_analog_state(0);
  stepper_move_to(aio);
}

static void drdy_handler(void *arg)
{
//  while(1);
  palTogglePad(GPIOB,11);
  chSysDisable();
  NVIC_SystemReset();
  //if(PWR->CR & 
}

int main()
{
  //PWR->CR |= PWR_CR_CLWUF | PWR_CR_CLSBF;
  halInit();
  chSysInit();
  AFIO->MAPR |= (AFIO_MAP_SWJTAG_CONF_JTAGDISABLE);
  
//  PWR->CR |= PWR_CR_CLSBF;
  chRegSetThreadName("Main");
  rf_task_init();
  
  usbcdc_task_init((void*)&shell_cfg);
  
  if(rf_op_mode() == 3){
    // start ADC
    adcStart(&ADCD1,NULL);
    adcStartConversion(&ADCD1,&adcgrpcfg,samples,ADC_GRP1_BUF_DEPTH);  
    ADCD1.adc->CR2 |= 0x0;
    palClearPad(GPIOA,2);
    palClearPad(GPIOB,9);
//    palSetLineCallback(PAL_LINE(GPIOB, 7), drdy_handler,NULL);
//    palEnableLineEvent(PAL_LINE(GPIOB, 7),PAL_EVENT_MODE_FALLING_EDGE);
  }
  
  if(rf_op_mode() == 4){
    stepper_task_init((void*)control_struct());
  }
  
  bool bStop = false;
  while(!bStop){
    eventmask_t evt = chEvtWaitAnyTimeout(ALL_EVENTS, TIME_MS2I(100));
    if(evt & EV_RX_PACKET){
      switch(rf_op_mode()){
      case 1:
        //processTX1();
        break;
      case 2:
        // process RX1();
        break;
      case 3:
        processTX2();
        break;
      case 4:
        processRX2();
        break;
      default:break;
      }    
    }
    if(evt & EV_TX_DATA_REQUEST){
      switch(rf_op_mode()){
      case 1:
        //processTX1();
        break;
      case 2:
        // process RX1();
        break;
      case 3:
        processTX2();
        break;
      case 4:
        //processRX2();
        break;
      default:break;
      }    
    }
    if(evt & EV_TX_DONE){
      rf_task_stop_manual_mode() ; 
      // change output pin to input mode
      palSetPadMode(GPIOB,2,PAL_MODE_INPUT_PULLUP);
      palSetPadMode(GPIOB,11,PAL_MODE_INPUT_PULLUP);
      palSetPadMode(GPIOA,2,PAL_MODE_INPUT);
      palSetPadMode(GPIOB,9,PAL_MODE_INPUT);


//      palSetPad(GPIOA,2);
//      palSetPad(GPIOB,9);
      bStop = true;
    }
  }
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
      __enable_irq();
      NVIC_SystemReset(); 
  
  return 0;
}