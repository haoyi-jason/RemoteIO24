#include "ch.h"
#include "hal.h"
#include "../drivers/usbcdc_task.h"
#include "../drivers/usbcfg.h"
//#include "task_ppmu.h"
#include "task_binprotocol.h"
#include "version.h"
#include "database.h"
#include "app_defs.h"
#define SD_USE_USB      0
#define SD_USE_SD1      1

static struct{
  thread_t *opThread;
  systime_t timeout;
}runTime;


static THD_WORKING_AREA(waOperation,512);
static THD_FUNCTION(procOperation ,p)
{
  bool bStop = false;
  struct{
    uint16_t packet_id;
    uint16_t start_address;
    union{
      uint8_t b[32];
      int32_t i32[8];
    }u;
  }data;
  data.packet_id = 0;
  data.start_address = LIVE_DATA_CH1_RAW;
  while(!bStop)
  {
    for(uint8_t i=0;i<8;i++){
      data.u.i32[i] = db_read_ld_i32(LIVE_DATA_CH1_RAW+i);
    }
    send_packet((uint8_t*)&data,36);
    data.packet_id++;
    bStop = chThdShouldTerminateX();
    if(bStop){
      
    }
    chThdSleepMilliseconds(100);
  }
}

void start_transfer()
{
  if(!runTime.opThread){
    runTime.opThread = chThdCreateStatic(waOperation,sizeof(waOperation),NORMALPRIO-1,procOperation,NULL);
  }
}

void stop_transfer()
{
  if(runTime.opThread){
    chThdTerminate(runTime.opThread);
    chThdWait(runTime.opThread);
    runTime.opThread = NULL;
    //chVTReset(&runTime.vt);
  }
}

uint8_t is_bounded()
{
  return palReadPad(GPIOA,12);
}

int main()
{
  halInit();
  chSysInit();
  AFIO->MAPR |= (AFIO_MAP_SWJTAG_CONF_JTAGDISABLE);
  
  //chRegSetThreadName("Main");
  J3_POWER_ON();
  J4_POWER_ON();
  database_init();
  task_binProtocolInitw();
  Ad7124_task_init();

  //pmu_init();
  while(1){
    eventmask_t evt = chEvtWaitAnyTimeout(ALL_EVENTS, TIME_MS2I(50));
    if(evt & EVENT_MASK(CMD_START)){
      start_transfer();
    }
    else if(evt & EVENT_MASK(CMD_STOP)){
      stop_transfer();
    }
    else if(evt & EVENT_MASK(CMD_SAVE_NVM)){
      db_save_section(0xff);
    }
    else if(evt & EVENT_MASK(CMD_RESTORE_NVM)){
      
    }
    
    if(is_bounded() == 0){
      stop_transfer();
    }
    chThdSleepMilliseconds(50);
  }
}