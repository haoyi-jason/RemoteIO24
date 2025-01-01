#include "ch.h"
#include "hal.h"
#include "usbcdc_task.h"
#include "../drivers/usbcfg.h"

#include <string.h>
#include "shell.h"
#include "chprintf.h"
#include <stdlib.h>

struct {
  thread_t *self;
  thread_t *mainThread;
  thread_t *shellThread;
}usbcdc_runtime;

#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(512)
static THD_WORKING_AREA(waShell,SHELL_WA_SIZE);

static THD_WORKING_AREA(waUSBTask,1024);
static THD_FUNCTION(procUSBTask,p)
{
  //ShellConfig *shell_cfg = (ShellConfig*)p;
  sduObjectInit(&SDU1);
  sduStart(&SDU1, &serusbcfg);
//  palSetPad(GPIOC,10);
//  palClearPad(GPIOC,10);
//  usbDisconnectBus(serusbcfg.usbp);
  chThdSleepMilliseconds(50);
//  palSetPad(GPIOC,10);
  usbStart(serusbcfg.usbp, &usbcfg);
  usbConnectBus(serusbcfg.usbp);  
  shellInit();
  bool bStop = false;
  while(!bStop){
    if(SDU1.config->usbp->state == USB_ACTIVE){
      if(usbcdc_runtime.shellThread == NULL){
        usbcdc_runtime.shellThread = chThdCreateStatic(waShell,sizeof(waShell),NORMALPRIO,shellThread,p);
        rf_task_start_manual_mode((BaseSequentialStream*)&SDU1);
      }
      chEvtWaitAny(EVENT_MASK(0));
      if(chThdTerminatedX(usbcdc_runtime.shellThread)){
        chThdRelease(usbcdc_runtime.shellThread);
        usbcdc_runtime.shellThread = NULL;
        rf_task_stop_manual_mode();
      }
    }
    chThdSleepMilliseconds(50);
//    else if(usbcdc_runtime.shellThread != NULL){
//      chThdTerminate(usbcdc_runtime.shellThread);
//      chThdWait(usbcdc_runtime.shellThread);
//      usbcdc_runtime.shellThread = NULL;
//      rf_task_stop_manual_mode();
//    }
    
    
  }

}

void usbcdc_task_init(void *arg)
{
  
  usbcdc_runtime.self = chThdCreateStatic(waUSBTask,sizeof(waUSBTask),NORMALPRIO,procUSBTask,arg);
  
  

}