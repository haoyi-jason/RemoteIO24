#include "ch.h"
#include "hal.h"
#include "../drivers/usbcdc_task.h"
#include "shell.h"
#include "shell_command.h"
#include "../drivers/usbcfg.h"
#include "task_ppmu.h"


static const ShellCommand commands[] = {
  {"force",cmd_force},
  {"fm",cmd_force_mode},
  {"fr",cmd_force_range},
  {"clamp",cmd_clamp},
  {"cpoh",cmd_cpoh},
  {"load",cmd_load},
  {"daout",cmd_dac_output},
  //{"dagain",cmd_dac_gain},
  //{"daoffset",cmd_dac_offset},
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
  pmu_init();
  while(1){
    
    
    chThdSleepMilliseconds(50);
  }
}