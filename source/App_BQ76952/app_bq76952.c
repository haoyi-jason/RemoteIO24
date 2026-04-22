#include "ch.h"
#include "hal.h"
//#include "../drivers/usbcdc_task.h"
//#include "../drivers/usbcfg.h"
//#include "task_ppmu.h"
//#include "task_binprotocol.h"
#include "version.h"
//#include "database.h"
//#include "app_defs.h"
#include "bq76xx_dev.h"

#define SD_USE_USB      0
#define SD_USE_SD1      1

static struct{
  thread_t *opThread;
  systime_t timeout;
}runTime;

#define LED1_ON()       palClearPad(GPIOB,3)
#define LED1_OFF()       palSetPad(GPIOB,3)
#define LED2_ON()       palClearPad(GPIOB,4)
#define LED2_OFF()       palSetPad(GPIOB,4)
#define LED3_ON()       palClearPad(GPIOB,5)
#define LED3_OFF()       palSetPad(GPIOB,5)

#define RELAY_1_OFF()    palClearPad(GPIOA,1)
#define RELAY_1_ON()    palSetPad(GPIOA,1)
#define RELAY_2_OFF()    palClearPad(GPIOA,0)
#define RELAY_2_ON()    palSetPad(GPIOA,0)
#define RELAY_3_OFF()    palClearPad(GPIOC,9)
#define RELAY_3_ON()    palSetPad(GPIOC,9)

#define IS_5VP_PRESENT()  (palReadPad(GPIOA,8) == PAL_HIGH)
#define IS_SW1_PRESSED()  (palReadPad(GPIOB,8) == PAL_LOW)
#define IS_SW2_PRESSED()  (palReadPad(GPIOB,9) == PAL_LOW)


int main()
{
  halInit();
  chSysInit();
  AFIO->MAPR |= (AFIO_MAP_SWJTAG_CONF_JTAGDISABLE);
  // PB3/4/5 for LED 1/2/3
  palSetPadMode(GPIOB,3,PAL_MODE_OUTPUT_OPENDRAIN);
  palSetPadMode(GPIOB,4,PAL_MODE_OUTPUT_OPENDRAIN);
  palSetPadMode(GPIOB,5,PAL_MODE_OUTPUT_OPENDRAIN);
  
  // PB8/PB9 ACT1/ACT2 button for SPI/IIC interface access
  palSetPadMode(GPIOB,8,PAL_MODE_INPUT_PULLUP);
  palSetPadMode(GPIOB,9,PAL_MODE_INPUT_PULLUP);
  
  // PA1/PA0/PC9 for relay 1/2/3
  palSetPadMode(GPIOA,0,PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(GPIOA,1,PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(GPIOC,9,PAL_MODE_OUTPUT_PUSHPULL);
  RELAY_1_OFF();
  RELAY_2_OFF();
  RELAY_3_OFF();
  
  // PA8 for BQ76952 5VP detection
  palSetPadMode(GPIOA,8,PAL_MODE_INPUT);
  
  // PA 4/5/6/7 for SPI mode
  palSetPadMode(GPIOA,4, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(GPIOA,5, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
  palSetPadMode(GPIOA,6, PAL_MODE_INPUT_PULLUP);
  palSetPadMode(GPIOA,7, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
  
  
  // PB6/7 for IIC mode
  palSetPadMode(GPIOB,6, PAL_MODE_STM32_ALTERNATE_OPENDRAIN);
  palSetPadMode(GPIOB,7, PAL_MODE_STM32_ALTERNATE_OPENDRAIN);
  
  static uint8_t comm_type = 0xff;
  uint8_t stage = 0;
  while(1)
  {
    switch(stage){
    case 0:
      if(IS_SW1_PRESSED()){
        stage = 10;
      }
      if(IS_SW2_PRESSED()){
        stage = 20;
      }
      break;
    case 10:  // check press again
      if(IS_SW1_PRESSED()){
        stage = 11;
        LED1_OFF();
        LED2_OFF();
        LED3_OFF();
      }
      else{
        stage = 0;
      }
      break;
    case 11: // power dut on
      RELAY_3_ON();
      stage = 12;
      break;
    case 12: // check if 5VP present
      if(IS_5VP_PRESENT()){
        stage = 13;
      }
      else{
        RELAY_3_OFF();
        LED3_ON();
        stage = 0;
      }
      break;
    case 13: // switch to SPI mode check
      RELAY_1_ON();
      RELAY_2_ON();
      stage = 14;
      break;
    case 14:
      comm_type = bq76xx_i2c_check();
      if(comm_type == 0x12 || comm_type == 0x11){
        LED2_ON();
        stage = 0;
      }
      else{
        RELAY_1_OFF();
        RELAY_2_OFF();
        stage = 15;
      }
      break;
    case 15:
      comm_type = bq76xx_spi_check();
      if(comm_type == 0x00){
        LED1_ON();
      }
      else{
        LED3_ON();
      }
      RELAY_1_OFF();
      RELAY_2_OFF();
      RELAY_3_OFF();
      stage = 0;
      break;
      
    case 20:  // check press again
      if(IS_SW2_PRESSED()){
        stage = 21;
        LED1_OFF();
        LED2_OFF();
        LED3_OFF();
      }
      else{
        stage = 0;
      }
      break;
    case 21: // set to iic mode
      RELAY_1_OFF();
      RELAY_2_OFF();
      RELAY_3_ON();
      stage = 22;
      break;
    case 22:
      if(IS_5VP_PRESENT()){
        stage = 23;
      }
      else{
        RELAY_3_OFF();
        LED3_ON();
        stage = 0;
      }
      break;
    case 23:
      if(bq76xx_spi_to_iic() == 1){
        stage = 24;
      }
      else{
        LED3_ON();
        stage = 0;
      }
      break;
    case 24:
      RELAY_1_ON();
      RELAY_2_ON();
      stage = 25;
      break;
    case 25:
      comm_type = bq76xx_i2c_check();
      if(comm_type == 0x12 || comm_type == 0x11){
        LED2_ON();
      }
      else{
        LED3_ON();
      }
      stage = 0;
      RELAY_3_OFF();
      RELAY_1_OFF();
      RELAY_2_OFF();
      break;      
    }
      
    
//    if(palReadPad(GPIOA,8) == PAL_LOW){
//      if(comm_type == 0xff){
//        comm_type = bq76xx_spi_check();
//      }
//      else if(comm_type == 0x00){
//        bq76xx_spi_to_iic();        
//      }
//    }
//    
//    if(palReadPad(GPIOA,10) == PAL_LOW){
//      bq76xx_i2c_check();
//    }

    chThdSleepMilliseconds(200);
  }
  

}