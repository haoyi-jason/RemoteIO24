#include "ch.h"
#include "hal.h"
#include "../drivers/usbcdc_task.h"
#include "../drivers/usbcfg.h"
#include "task_ppmu.h"
#include "task_binprotocol.h"
#include "version.h"


#define SD_USE_USB      0
#define SD_USE_SD1      0




int main()
{
  halInit();
  chSysInit();
  AFIO->MAPR |= (AFIO_MAP_SWJTAG_CONF_JTAGDISABLE);
  
  chRegSetThreadName("Main");
#if(SD_USE_USB)
  usbcdc_task_init((void*)&shell_cfg);
#else
//  sdStart(&SD1, &serialcfg);
//  sdWrite(&SD1,"HELLO\n",6);
//  chThdCreateStatic(waShell,sizeof(waShell),NORMALPRIO,shellThread,(void*)&shell_cfg);
  task_binProtocolInit(0);
#endif
  pmu_init();
  while(1){
    eventmask_t evt = chEvtWaitAnyTimeout(ALL_EVENTS, TIME_MS2I(50));
    if(evt & EVT_SAVE_PMU_NVM)
    {
      pmu_nvm_save();
    }
    if(evt & EVT_RESTORE_PMU_NVM){
      pmu_nvm_load_default();
    }    
    if(evt & EVT_SAVE_BOARD_NVM)
    {
      board_nvm_save();
    }
    if(evt & EVT_RESTORE_BOARD_NVM){
      board_nvm_load_default();
    }    
    chThdSleepMilliseconds(50);
  }
}