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

#define LINE_TX_ACT    PAL_LINE(GPIOB,2)
#define LINE_TX_LBT     PAL_LINE(GPIOB,11)

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
//#ifndef RF_HANDLER
//  {"pattern", cmd_default_io_pattern},
//  {"polt", cmd_poll_interval},
//  {"polc", cmd_poll_count},
//  {"idn", cmd_idn},
//  {"vrmap", cmd_vrMap},
//  {"spdmap", cmd_spdMap},
//  {"txsim",cmd_txsim},
//#endif
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
  {GPIOB,2}, // tx
  {GPIOB,11}, // lbt
};

_dio_def_t digital_out_rx[] = {
  {GPIOB,7},
  {GPIOB,8},
  {GPIOA,9},
  {GPIOB,6},
  {GPIOB,3},
  {GPIOB,4},
  {GPIOC,6},
  {GPIOC,9},
  {GPIOB,12},
  {GPIOB,10},
  {GPIOB,11},
  {GPIOA,10},
  {GPIOA,8},
  {GPIOC,8},
};
_dio_def_t digital_in_rx[] = {
  {GPIOB,1},
  {GPIOB,12},
  {GPIOB,10},
};

static void processTX_DO()
{
  
  
}

static void processRX_DO(uint16_t value)
{
  uint16_t mask;
  if(value != 0x0){
    palSetPad(digital_out_rx[10].port,digital_out_rx[10].line);
  }
  else{
    palClearPad(digital_out_rx[10].port,digital_out_rx[10].line);
  }
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

static void blink_cb(void *arg)
{
 //virtual_timer_t *vt = (virtual_timer_t*)arg;
  chSysLockFromISR();
  palToggleLine(LINE_TX_ACT);
  chVTSetI(&vt_blink,TIME_MS2I(500),blink_cb,NULL);
  chSysUnlockFromISR();
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
  
  // check if low battery
  /*
    lbt voltage         code
    5.0                 (5-1.2)/10.56*4096 = 1940
    4.8                 (4.8 - 1.2)*10.56/4096 = 1400
  */
  
  if(ch_sum[1] < 1300){
    palClearLine(LINE_TX_LBT);
  }
  else if(ch_sum[1] > 1400){
    palSetLine(LINE_TX_LBT);
  }
  
  
//  adcStartConversion(&ADCD1,&adcgrpcfg,samples,ADC_GRP1_BUF_DEPTH);  
}
static void processRX2()
{
  uint16_t dio = read_ctrl_state();
  processRX_DO(dio);
  // process stepper
  if(dio != 0){
    uint16_t aio = read_analog_state(0);
    stepper_move_to(aio);
  }
  else{
    stepMoveHome();
  }
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
    chVTObjectInit(&vt_blink);
    chVTSet(&vt_blink,TIME_MS2I(500), blink_cb,NULL);
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
    if(evt & EV_RX_ERROR){
      if(rf_op_mode() == 4){
        processRX_DO(0x0);
        stepMoveHome();
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

      palClearLine(LINE_TX_ACT);
      if(usb_cdc_active() == 0){
        bStop = true;
        usbcdc_task_stop();
      }
    }
    if(evt & EV_SAVE){
      rf_save_nvm();
    }
  }
  
      chVTReset(&vt_blink);
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