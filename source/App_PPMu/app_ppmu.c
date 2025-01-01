#include "ch.h"
#include "hal.h"
#include "../drivers/usbcdc_task.h"
#include "shell_command.h"
#include "../drivers/usbcfg.h"



static const ShellCommand commands[] = {
  
  {NULL, NULL}
};

static const ShellConfig shell_cfg = {
  (BaseSequentialStream *)&SDU1,
  commands
};

int main()
{
  halInit();
  chSysInit();
  AFIO->MAPR |= (AFIO_MAP_SWJTAG_CONF_JTAGDISABLE);
  
  chRegSetThreadName("Main");
  usbcdc_task_init((void*)&shell_cfg);
  
  while(1){
    
    
    chThdSleepMilliseconds(50);
  }
}