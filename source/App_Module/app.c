#include "ch.h"
#include "hal.h"
#include "app.h"
#include "../drivers/bc3601.h"
#include "../drivers/usbcfg.h"
#include "shell.h"
#include "chprintf.h"
#include <string.h>

static void cmd_pulse(BaseSequentialStream *chp, int argc, char *argv[]);
static const ShellCommand commands[] = {
  {"pulse", cmd_pulse},
  {NULL,NULL}
};

static const ShellConfig shell_cfg = {
  (BaseSequentialStream *)&SDU1,
  commands
};

#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(512)
static THD_WORKING_AREA(waShell,SHELL_WA_SIZE);

static const SPIConfig spicfg = {
  NULL,
  NULL,
  NULL,
  SPI_CR1_BR_2 | SPI_CR1_BR_1
};

static bc3601_config_t config = {
  &SPID1,
  &spicfg,
  GPIOA,
  2
};

static BC3601Driver bc3601l;


int app_init(void *arg)
{
    thread_t *shelltp1 = NULL;
  sduObjectInit(&SDU1);
  sduStart(&SDU1, &serusbcfg);
    usbDisconnectBus(serusbcfg.usbp);
  palClearPad(GPIOA,8);
  chThdSleepMilliseconds(50);
  palSetPad(GPIOA,8);
  usbStart(serusbcfg.usbp, &usbcfg);
  usbConnectBus(serusbcfg.usbp);  

  shellInit();
  
      while(1){
      if (SDU1.config->usbp->state == USB_ACTIVE) {
        /* Starting shells.*/
        if (shelltp1 == NULL) {
  //        shelltp1 = chThdCreateFromHeap(NULL, SHELL_WA_SIZE,
  //                                       "shell1", NORMALPRIO + 1,
  //                                       shellThread, (void *)&shell_cfg1);
          shelltp1 = chThdCreateStatic(waShell, sizeof(waShell),NORMALPRIO+1,shellThread,(void*)&shell_cfg);
          
          //remoteio_start_task(&SDU1);
        }

        /* Waiting for an exit event then freeing terminated shells.*/
        chEvtWaitAny(EVENT_MASK(0));
        if (chThdTerminatedX(shelltp1)) {
          chThdRelease(shelltp1);
          shelltp1 = NULL;
          //remoteio_stop_task();
        }
      }
      else {
//        if(txStopped()){
//          break;
//        }
//        else{
          chThdSleepMilliseconds(200);
//        }
      }
      
      
    }
    PWR->CR |= (PWR_CR_PDDS);
    SCB->SCR |= (SCB_SCR_SLEEPDEEP_Msk);
    PWR->CSR &= ~(PWR_CRSTS_WUF);
//    chSysDisable();
    __WFI();
  
}

static void cmd_pulse(BaseSequentialStream *chp, int argc, char *argv[])
{
  
}