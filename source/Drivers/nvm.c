#include "ch.h"
#include "hal.h"
#include "nvm.h"
//#include "at32_flash.h"
#include "bootConfig.h"
 
static uint8_t pg[PAGE_SIZE];

void nvmFlashReadPage(uint8_t page, uint8_t *d)
{
  flash_Read(FLASH_START_ADDRESS + page*PAGE_SIZE,(uint16_t*)d,PAGE_SIZE/2);
}

void nvmFlashWritePage(uint8_t page, uint8_t *d)
{
  chSysLock();
  flash_Write(FLASH_START_ADDRESS + page * PAGE_SIZE,(uint16_t*)d,PAGE_SIZE/2);
  chSysUnlock();
}

msg_t nvm_flash_read(uint32_t offset, uint8_t *dptr, uint16_t sz)
{
  flash_Read(FLASH_START_ADDRESS + offset, (uint16_t*)dptr, sz/2);
  return MSG_RESET;
}

msg_t nvm_flash_write(uint16_t offset, uint8_t *dptr, uint16_t sz)
{
  //uint8_t *pg = chHeapAlloc(NULL,PAGE_SIZE);
  if(pg == NULL){
    while(1);
  }
  uint8_t pgStart, pgEnd;
  uint32_t addr_end = offset + sz;
  pgStart = PAGE_NUMBER(offset);
  pgEnd = PAGE_NUMBER(addr_end) + 1;
  uint16_t wsz,wof,rsz;
  uint16_t szRemain = sz;
  wof = offset;
  rsz = 0;
  if(pgStart != 0){
    wof -= pgStart * PAGE_SIZE;
  }
  for(uint8_t i = pgStart;i<pgEnd; i++){
      nvmFlashReadPage(i,pg);
      
      if((wof % PAGE_SIZE) == 0){ // align = 0
        wsz = (szRemain > PAGE_SIZE)?(PAGE_SIZE):szRemain;
      }
      else{ // align != 0
        
        wsz = (szRemain > (PAGE_SIZE - wof))?(PAGE_SIZE-wof):szRemain;
      }
      memcpy(&pg[wof],&dptr[rsz],wsz);
      nvmFlashWritePage(i,pg);
      rsz += wsz;
      wof += wsz;
      wof = wof % PAGE_SIZE;
      szRemain -= wsz;
  }
  return MSG_RESET;
}


/*  runtime data exchange */
static struct{
  int32_t adc_raw[8];
  int32_t do_from_ana[2];
  int32_t do_from_com[2];
  int32_t do_from_di[2];
  int32_t do_state[2];
}runTime;

int32_t runtime_get_adc_raw(uint8_t ch)
{
  return runTime.adc_raw[ch];
}

void runtime_set_adc_raw(uint8_t ch, int32_t raw)
{
  runTime.adc_raw[ch] = raw;
}

void runtime_set_do_state(uint8_t ch, int32_t raw)
{
  runTime.do_state[ch] = raw;
}

int32_t runtime_get_do_state(uint8_t ch)
{
  return runTime.do_state[ch];
}


int32_t runtime_get_do_from_ana(uint8_t ch)
{
  return runTime.do_from_ana[ch];
}

int32_t runtime_get_do_from_com(uint8_t ch)
{
  return runTime.do_from_com[ch];
}

int32_t runtime_get_do_from_di(uint8_t ch)
{
  return runTime.do_from_di[ch];
}

void runtime_set_do_from_ana(uint8_t ch, int32_t raw)
{
  runTime.do_from_ana[ch] = raw;
}

void runtime_set_do_from_com(uint8_t ch, int32_t raw)
{
  runTime.do_from_com[ch] = raw;
}

void runtime_set_do_from_di(uint8_t ch, int32_t raw)
{
  runTime.do_from_di[ch] = raw;
}



