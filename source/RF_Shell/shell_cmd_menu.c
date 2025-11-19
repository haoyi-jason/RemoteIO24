#include "hal.h"
#include "ch.h"
#include "chprintf.h"
#include "shell_cmd_menu.h"

static thread_t *self;
static thread_t *main;
static shell_cb cb_func = NULL;

static uint8_t getLine(BaseSequentialStream *chp, char *buffer, uint8_t maxLen)
{
  uint8_t szRead = 0;
  uint8_t *p = buffer;
  char c;
  while(true){
    if(streamRead(chp,(uint8_t *)&c,1) == 0)
      return szRead;
    if(c == '\r' || c == '\n'){
      *p = 0;
      return szRead;
    }
    else{
      *p++ = c;
      szRead++;
    }
  }
  
  return szRead;
}

static void printMenu(BaseSequentialStream *chp)
{
  if(chp == NULL) return;
  chprintf(chp,"\n\n");
  chprintf(chp, "Select Items below\n");
  chprintf(chp, "(1) Set ID\n");
  chprintf(chp, "(2) Set DST ID\n");
  chprintf(chp, "(3) Set RF Frequency\n");
  chprintf(chp, "(4) Set TX Power\n");
  chprintf(chp, "(5) Set TX Interval(ms)\n");
  chprintf(chp, "(6) Set TX Timeout(ms)\n");
  chprintf(chp, "(7) Set RX Map\n");
}

static void printMenu_01(BaseSequentialStream *chp)
{
  chprintf(chp,"Please Enter ID:");  
  char buf[32];
  if(getLine(chp,buf,32) > 0){
    uint32_t id = strtol(buf,NULL,10);
    if(cb_func != NULL)
      cb_func(1,id);
  }
}

static void printMenu_02(BaseSequentialStream *chp)
{
  chprintf(chp,"Please Enter ID:");  
  char buf[32];
  if(getLine(chp,buf,32) > 0){
    uint32_t id = strtol(buf,NULL,10);
    if(cb_func != NULL)
      cb_func(2,id);
  }
}

static void printMenu_03(BaseSequentialStream *chp)
{
  chprintf(chp,"Please Enter Frequency:");  
  char buf[32];
  if(getLine(chp,buf,32) > 0){
    uint32_t value = strtol(buf,NULL,10);
    if(cb_func != NULL)
      cb_func(3,value);
  }
}

static void printMenu_04(BaseSequentialStream *chp)
{
  chprintf(chp,"Please Enter Power (0,1,2,3):");  
  char buf[32];
  if(getLine(chp,buf,32) > 0){
    uint32_t value = strtol(buf,NULL,10);
    if(cb_func != NULL)
      cb_func(4,value);
  }
}

static void printMenu_05(BaseSequentialStream *chp)
{
  chprintf(chp,"Please Enter Interval:");  
  char buf[32];
  if(getLine(chp,buf,32) > 0){
    uint32_t value = strtol(buf,NULL,10);
    if(cb_func != NULL)
      cb_func(5,value);
  }
}

static void printMenu_06(BaseSequentialStream *chp)
{
  chprintf(chp,"Please Enter Time:");  
  char buf[32];
  if(getLine(chp,buf,32) > 0){
    uint32_t value = strtol(buf,NULL,10);
    if(cb_func != NULL)
      cb_func(6,value);
  }
}

static void printMenu_07(BaseSequentialStream *chp)
{
  chprintf(chp,"Please Enter Time:");  
  char buf[32];
  if(getLine(chp,buf,32) > 0){
    uint32_t value = strtol(buf,NULL,10);
    if(cb_func != NULL)
      cb_func(6,value);
  }
}


static THD_WORKING_AREA(waShell,1024);
static THD_FUNCTION(procShellMenu ,p)
{
  BaseSequentialStream *s = (BaseSequentialStream*)p;
  main = chRegFindThreadByName("Main");
  bool bStop = false;
  uint8_t buf[32];
  int8_t currentItem = -1;
  printMenu(p);
  while(!bStop){
    if(getLine(p,buf,32) > 0){
      uint32_t index = strtol(buf,NULL,10);
      switch(index)
      {
      case 1:printMenu_01(p);break;
      case 2:printMenu_02(p);break;
      case 3:printMenu_03(p);break;
      case 4:printMenu_04(p);break;
      case 5:printMenu_05(p);break;
      case 6:printMenu_06(p);break;
      case 7:printMenu_07(p);break;
      default:printMenu(p);break;
      }
    }
    
    if(chThdShouldTerminateX()){
      bStop = true;      
    }
    chThdSleepMilliseconds(10);
  }
}

void shell_cmd_menu_init(BaseSequentialStream *chp, shell_cb cb)
{
//  cb_func = cb;
  cb_func = NULL;
  self = chThdCreateStatic(waShell,sizeof(waShell),NORMALPRIO,procShellMenu,chp);
}

void shell_cmd_menu_stop()
{
  chThdTerminateX(self);
  chThdWait(self);
}