#include "ch.h"
#include "hal.h"
#include "usbcfg.h"
#include "shell.h"
#include "chprintf.h"
#include "bc3601.h"
#include "string.h"

#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(2048)
static THD_WORKING_AREA(waShell,SHELL_WA_SIZE);
static const SPIConfig spicfg = {
  NULL,
  NULL,//GPIOB,
  NULL,//GPIOB_SPI1_CS,
  SPI_CR1_BR_2 | SPI_CR1_BR_1  
};

static bc3601_config_t config = {
  &SPID1,
  &spicfg,
  GPIOA,
  2
};


static BC3601Driver bc3601;

/*
  send packet to remote, usage:
  write xx yy
  xx: remote address, 14-bit width except 0
  yy: ascii or hex value to write, hex can start with 0x or $
*/
static void cmd_write(BaseSequentialStream *chp, int argc, char *argv[]) 
{
  if(argc == 2){
    uint16_t addr;
    uint8_t value;
    if(strncmp(argv[0],"0x",2) == 0){
      addr = (uint8_t)strtol(argv[0],NULL,16);
    }
    else{
      addr = (uint8_t)strtol(argv[0],NULL,10);
    }
    remoteio_setSelfAddr(addr);
    remoteio_send(argv[1], strlen(argv[1]));
  }
}

static void cmd_writeHex(BaseSequentialStream *chp, int argc, char *argv[]) 
{
  if(argc == 2){
    uint16_t addr;
    uint8_t value;
    if(strncmp(argv[0],"0x",2) == 0){
      addr = (uint8_t)strtol(argv[0],16);
    }
    else{
      addr = (uint8_t)strtol(argv[0],10);
    }
    remoteio_setDestAddr(addr);
    char str[64];
    char dst[2];
    char *src = argv[2];
    uint8_t sz = 0;
    for(uint8_t i=0;i<strlen(argv[2])/2;i++){
      memcpy(dst,src,2); 
      value = (uint8_t)strtol(dst,16);
      str[i] = value;
      src += 2;
      sz++;
    }
    remoteio_send(str, sz);
  }
}


/*
  read: read register, usage
  read aa,
  aa: register address
*/
static void cmd_read(BaseSequentialStream *chp, int argc, char *argv[]) 
{
  
  if(argc == 2){
    uint8_t addr;
    uint16_t n;
    uint16_t value;
    if(strncmp(argv[0],"0x",2) == 0){
      addr = (uint8_t)strtol(argv[0],16);
    }
    else{
      addr = (uint8_t)strtol(argv[0],10);
    }
        
    if(strncmp(argv[1],"0x",2) == 0){
      n = (uint16_t)strtol(argv[1],16);
    }
    else{
      n = (uint16_t)strtol(argv[1],10);
    }

    // todo read register
    uint8_t tx[2];
    tx[0] = addr;
    uint8_t *rx = chHeapAlloc(NULL,n*2);
    addr = (addr >> 1) << 1;
    
    if(rx != NULL){
      bc3601ReadRegister(&bc3601,addr,rx);
      
      for(uint8_t i=0;i<n;i++){
       //chprintf(chp,"%02x = %04x\r\n",addr+(i<<1),swap_16((uint8_t*)&rx[i*2]));
        //chprintf(chp,"%02x = %04x\r\n",addr+(i<<1),(rx[i]));
      }
    }
    chHeapFree(rx);
    
    
  }
}

static void cmd_addr(BaseSequentialStream *chp, int argc, char *argv[]) 
{
  
  if(argc == 1){
    uint16_t addr;
    if(strncmp(argv[0],"0x",2) == 0){
      addr = (uint16_t)strtol(argv[0],NULL,16);
    }
    else{
      addr = (uint16_t)strtol(argv[0],NULL,10);
    }
    remoteio_setDestAddr(addr);
  }
  else if(argc == 0){
    chprintf(chp,"DEST=%d\n",remoteio_getDestAddr());
  }
}

static void cmd_selfaddr(BaseSequentialStream *chp, int argc, char *argv[]) 
{  
  if(argc == 1){
    uint16_t addr;
    if(strncmp(argv[0],"0x",2) == 0){
      addr = (uint16_t)strtol(argv[0],NULL,16);
    }
    else{
      addr = (uint16_t)strtol(argv[0],NULL,10);
    }
    remoteio_setSelfAddr(addr);
  }
  else if(argc == 0){
    chprintf(chp,"SADR=%d\n",remoteio_getSelfAddr());
  }
}

static void cmd_power(BaseSequentialStream *chp, int argc, char *argv[]) 
{
  if(argc == 1){
    uint8_t value;
    if(strncmp(argv[0],"0x",2) == 0){
      value = (uint8_t)strtol(argv[0],NULL,16);
    }
    else{
      value = (uint8_t)strtol(argv[0],NULL,10);
    }
    remoteio_setPower(value);
  }
  else if(argc == 0){
    chprintf(chp,"POWER=%d\n",remoteio_getPower());
  }
}

static void cmd_freq(BaseSequentialStream *chp, int argc, char *argv[]) 
{
  if(argc == 1){
    float value;
    value = strtof(argv[0],NULL,10);
    remoteio_setFreq(value);
  }
  else if(argc == 0){
    chprintf(chp,"FREQ=%f\n",remteio_getFreq());
  }
}


static void cmd_rate(BaseSequentialStream *chp, int argc, char *argv[]) 
{
  if(argc == 1){
    uint8_t value;
    if(strncmp(argv[0],"0x",2) == 0){
      value = (uint8_t)strtol(argv[0],NULL,16);
    }
    else{
      value = (uint8_t)strtol(argv[0],NULL,10);
    }
    remoteio_setDataRate(value);
  }
  else if(argc == 0){
    chprintf(chp,"RATE=%d\n",getDataRate());
  }
}

static void cmd_mode(BaseSequentialStream *chp, int argc, char *argv[]) 
{
  if(argc == 1){
    uint8_t value;
    if(strncmp(argv[0],"0x",2) == 0){
      value = (uint8_t)strtol(argv[0],NULL,16);
    }
    else{
      value = (uint8_t)strtol(argv[0],NULL,10);
    }
    remoteio_setMode(value);
  }
  else if(argc == 0){
    chprintf(chp,"MODE=%d\n",remoteio_getMode());
  }
}

static void cmd_interval(BaseSequentialStream *chp, int argc, char *argv[]) 
{
  
  if(argc == 1){
    uint16_t addr;
    if(strncmp(argv[0],"0x",2) == 0){
      addr = (uint16_t)strtol(argv[0],NULL,16);
    }
    else{
      addr = (uint16_t)strtol(argv[0],NULL,10);
    }
    remoteio_setDestAddr(addr);
  }
  else if(argc == 0){
    chprintf(chp,"DEST=%d\n",remoteio_getDestAddr());
  }
}

static void cmd_save(BaseSequentialStream *chp, int argc, char *argv[]) 
{
  remoteio_saveParam();
}

static void cmd_default_io_pattern(BaseSequentialStream *chp, int argc, char *argv[]) 
{  
  if(argc == 1){
    uint16_t addr;
    if(strncmp(argv[0],"0x",2) == 0){
      addr = (uint16_t)strtol(argv[0],NULL,16);
    }
    else{
      addr = (uint16_t)strtol(argv[0],NULL,10);
    }
    remoteio_set_io_pattern(addr);
  }
  else if(argc == 0){
    chprintf(chp,"ADDR=0x%x\n",remoteio_get_io_pattern());
  }
}

static void cmd_poll_interval(BaseSequentialStream *chp, int argc, char *argv[]) 
{  
  if(argc == 1){
    uint16_t addr;
    if(strncmp(argv[0],"0x",2) == 0){
      addr = (uint16_t)strtol(argv[0],NULL,16);
    }
    else{
      addr = (uint16_t)strtol(argv[0],NULL,10);
    }
    remoteio_set_pollInterval(addr);
  }
  else if(argc == 0){
    chprintf(chp,"TIME=0x%x ms\n",remoteio_get_pollInterval());
  }
}

static void cmd_poll_count(BaseSequentialStream *chp, int argc, char *argv[]) 
{  
  if(argc == 1){
    uint16_t addr;
    if(strncmp(argv[0],"0x",2) == 0){
      addr = (uint16_t)strtol(argv[0],NULL,16);
    }
    else{
      addr = (uint16_t)strtol(argv[0],NULL,10);
    }
    remoteio_set_pollCount(addr);
  }
  else if(argc == 0){
    chprintf(chp,"COUNT=0x%x\n",remoteio_get_pollCount());
  }
}



static const ShellCommand commands[] = {
  {"write", cmd_write},
  {"writeX", cmd_writeHex},
  {"read", cmd_read},
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
  {NULL, NULL}
};

static const ShellConfig shell_cfg1 = {
  (BaseSequentialStream *)&SDU1,
  commands
};
//
//static SPIConfig spicfg = {
//  false,
//  NULL,
//  GPIOB,
//  12,
//  SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_CPOL | SPI_CR1_CPHA
//};

//static THD_WORKING_AREA(waBlink, 1024);
//static THD_FUNCTION(procBlink, arg) 
//{
//  (void)arg;
//  
//  while(1){
//    palSetPad(GPIOC,2);
//    chThdSleepMilliseconds(500);
//    palClearPad(GPIOC,2);
//    chThdSleepMilliseconds(500);
//  }
//}

int main()
{
  thread_t *shelltp1 = NULL;
  halInit();
  chSysInit();
  
  PWR->CR |= PWR_CR_CLSBF;
  
  sduObjectInit(&SDU1);
  sduStart(&SDU1, &serusbcfg);
  
  
  usbDisconnectBus(serusbcfg.usbp);
  palClearPad(GPIOA,8);
  chThdSleepMilliseconds(50);
  palSetPad(GPIOA,8);
  usbStart(serusbcfg.usbp, &usbcfg);
  usbConnectBus(serusbcfg.usbp);  

  shellInit();
  remoteio_task_init();
//  if(remoteio_task_init()){
//    PWR->CR |= (PWR_CR_PDDS);
//    SCB->SCR |= (SCB_SCR_SLEEPDEEP_Msk);
//    PWR->CSR &= ~(PWR_CRSTS_WUF);
//    chSysDisable();
//    __WFI();
//  }
//  else{
  
    chThdSleepMilliseconds(20);
    while(1){
      if (SDU1.config->usbp->state == USB_ACTIVE) {
        /* Starting shells.*/
        if (shelltp1 == NULL) {
  //        shelltp1 = chThdCreateFromHeap(NULL, SHELL_WA_SIZE,
  //                                       "shell1", NORMALPRIO + 1,
  //                                       shellThread, (void *)&shell_cfg1);
          shelltp1 = chThdCreateStatic(waShell, sizeof(waShell),NORMALPRIO+1,shellThread,(void*)&shell_cfg1);
          
          remoteio_start_task(&SDU1);
        }

        /* Waiting for an exit event then freeing terminated shells.*/
        chEvtWaitAny(EVENT_MASK(0));
        if (chThdTerminatedX(shelltp1)) {
          chThdRelease(shelltp1);
          shelltp1 = NULL;
          remoteio_stop_task();
        }
      }
      else {
        if(txStopped()){
          break;
        }
        else{
          chThdSleepMilliseconds(200);
        }
      }
      
      
    }
    PWR->CR |= (PWR_CR_PDDS);
    SCB->SCR |= (SCB_SCR_SLEEPDEEP_Msk);
    PWR->CSR &= ~(PWR_CRSTS_WUF);
//    chSysDisable();
    __WFI();

//  }
  return 0;
}