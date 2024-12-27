#include "ch.h"
#include "hal.h"
#include "shell_command.h"
#include "rf_task.h"
/*
  send packet to remote, usage:
  write xx yy
  xx: remote address, 14-bit width except 0
  yy: ascii or hex value to write, hex can start with 0x or $
*/
void cmd_write(BaseSequentialStream *chp, int argc, char *argv[]) 
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
    rf_setSelfAddr(addr);
    rf_send((uint8_t*)argv[1], strlen(argv[1]));
  }
}

void cmd_writeHex(BaseSequentialStream *chp, int argc, char *argv[]) 
{
  if(argc == 2){
    uint16_t addr;
    uint8_t value;
    char *endptr;
    if(strncmp(argv[0],"0x",2) == 0){
      addr = (uint8_t)strtol((void*)argv[0],&endptr,16);
    }
    else{
      addr = (uint8_t)strtol(argv[0],&endptr,10);
    }
    rf_setDestAddr(addr);
    char str[64];
    char dst[2];
    char *src = argv[2];
    uint8_t sz = 0;
    for(uint8_t i=0;i<strlen(argv[2])/2;i++){
      memcpy(dst,src,2); 
      value = (uint8_t)strtol(dst,&endptr,16);
      str[i] = value;
      src += 2;
      sz++;
    }
    rf_send((uint8_t*)str, sz);
  }
}


/*
  read: read register, usage
  read aa,
  aa: register address
*/
void cmd_read(BaseSequentialStream *chp, int argc, char *argv[]) 
{
  
//  if(argc == 2){
//    uint8_t addr;
//    uint16_t n;
//    uint16_t value;
//    if(strncmp(argv[0],"0x",2) == 0){
//      addr = (uint8_t)strtol(argv[0],16);
//    }
//    else{
//      addr = (uint8_t)strtol(argv[0],10);
//    }
//        
//    if(strncmp(argv[1],"0x",2) == 0){
//      n = (uint16_t)strtol(argv[1],16);
//    }
//    else{
//      n = (uint16_t)strtol(argv[1],10);
//    }
//
//    // todo read register
//    uint8_t tx[2];
//    tx[0] = addr;
//    uint8_t *rx = chHeapAlloc(NULL,n*2);
//    addr = (addr >> 1) << 1;
//    
//    if(rx != NULL){
//      bc3601ReadRegister(&bc3601,addr,rx);
//      
//      for(uint8_t i=0;i<n;i++){
//       //chprintf(chp,"%02x = %04x\r\n",addr+(i<<1),swap_16((uint8_t*)&rx[i*2]));
//        //chprintf(chp,"%02x = %04x\r\n",addr+(i<<1),(rx[i]));
//      }
//    }
//    chHeapFree(rx);
    
    
//  }
}

void cmd_addr(BaseSequentialStream *chp, int argc, char *argv[]) 
{
  
  if(argc == 1){
    uint16_t addr;
    if(strncmp(argv[0],"0x",2) == 0){
      addr = (uint16_t)strtol(argv[0],NULL,16);
    }
    else{
      addr = (uint16_t)strtol(argv[0],NULL,10);
    }
    rf_setDestAddr(addr);
  }
  else if(argc == 0){
    chprintf(chp,"DEST=%d\n",rf_getDestAddr());
  }
}

void cmd_selfaddr(BaseSequentialStream *chp, int argc, char *argv[]) 
{  
  if(argc == 1){
    uint16_t addr;
    if(strncmp(argv[0],"0x",2) == 0){
      addr = (uint16_t)strtol(argv[0],NULL,16);
    }
    else{
      addr = (uint16_t)strtol(argv[0],NULL,10);
    }
    rf_setSelfAddr(addr);
  }
  else if(argc == 0){
    chprintf(chp,"SADR=%d\n",rf_getSelfAddr());
  }
}

void cmd_power(BaseSequentialStream *chp, int argc, char *argv[]) 
{
  if(argc == 1){
    uint8_t value;
    if(strncmp(argv[0],"0x",2) == 0){
      value = (uint8_t)strtol(argv[0],NULL,16);
    }
    else{
      value = (uint8_t)strtol(argv[0],NULL,10);
    }
    rf_setPower(value);
  }
  else if(argc == 0){
    chprintf(chp,"POWER=%d\n",rf_getPower());
  }
}

void cmd_freq(BaseSequentialStream *chp, int argc, char *argv[]) 
{
  if(argc == 1){
    float value;
    value = strtof(argv[0],NULL);
    rf_setFreq(value);
  }
  else if(argc == 0){
    chprintf(chp,"FREQ=%8.3f\n",rf_getFreq());
  }
}


void cmd_rate(BaseSequentialStream *chp, int argc, char *argv[]) 
{
  
  if(argc == 1){
    uint8_t value;
    if(strncmp(argv[0],"0x",2) == 0){
      value = (uint8_t)strtol(argv[0],NULL,16);
    }
    else{
      value = (uint8_t)strtol(argv[0],NULL,10);
    }
    rf_setDataRate(value);
  }
  else if(argc == 0){
    chprintf(chp,"RATE=%d\n",rf_getDataRate());
  }
}

void cmd_mode(BaseSequentialStream *chp, int argc, char *argv[]) 
{
  if(argc == 1){
    uint8_t value;
    if(strncmp(argv[0],"0x",2) == 0){
      value = (uint8_t)strtol(argv[0],NULL,16);
    }
    else{
      value = (uint8_t)strtol(argv[0],NULL,10);
    }
    rf_setMode(value);
  }
  else if(argc == 0){
    chprintf(chp,"MODE=%d\n",rf_getMode());
  }
}

void cmd_interval(BaseSequentialStream *chp, int argc, char *argv[]) 
{
  
  if(argc == 1){
    uint16_t addr;
    if(strncmp(argv[0],"0x",2) == 0){
      addr = (uint16_t)strtol(argv[0],NULL,16);
    }
    else{
      addr = (uint16_t)strtol(argv[0],NULL,10);
    }
    rf_setDestAddr(addr);
  }
  else if(argc == 0){
    chprintf(chp,"DEST=%d\n",rf_getDestAddr());
  }
}

void cmd_save(BaseSequentialStream *chp, int argc, char *argv[]) 
{
  rf_saveParam();
}

void cmd_default_io_pattern(BaseSequentialStream *chp, int argc, char *argv[]) 
{  
  if(argc == 1){
    uint16_t addr;
    if(strncmp(argv[0],"0x",2) == 0){
      addr = (uint16_t)strtol(argv[0],NULL,16);
    }
    else{
      addr = (uint16_t)strtol(argv[0],NULL,10);
    }
    rf_set_io_pattern(addr);
  }
  else if(argc == 0){
    chprintf(chp,"PATTERN=0x%x\n",rf_get_io_pattern());
  }
}

void cmd_poll_interval(BaseSequentialStream *chp, int argc, char *argv[]) 
{  
  if(argc == 1){
    uint16_t addr;
    if(strncmp(argv[0],"0x",2) == 0){
      addr = (uint16_t)strtol(argv[0],NULL,16);
    }
    else{
      addr = (uint16_t)strtol(argv[0],NULL,10);
    }
    rf_set_pollInterval(addr);
  }
  else if(argc == 0){
    chprintf(chp,"TIME=0x%x ms\n",rf_get_pollInterval());
  }
}

void cmd_poll_count(BaseSequentialStream *chp, int argc, char *argv[]) 
{  
  uint16_t addr;
//      addr = (uint16_t)strtol(argv[0],NULL,10);
  if(argc == 1){
    if(strncmp(argv[0],"0x",2) == 0){
      addr = (uint16_t)strtol(argv[0],NULL,16);
    }
    else{
      addr = (uint16_t)strtol(argv[0],NULL,10);
    }
    rf_set_pollCount(addr);
  }
  else if(argc == 0){
    chprintf(chp,"COUNT=0x%x\n",rf_get_pollCount());
  }
}

void cmd_vrMap(BaseSequentialStream *chp, int argc, char *argv[]) 
{
  struct _rx_control_config *cfg = control_struct();
  char *edptr;
  if(argc == 0){
    chprintf(chp,"ID RAW ANGLE\r\n");
    for(uint8_t i=0;i<4;i++){
      chprintf(chp,"#%2d %4d %4d\r\n",i, cfg->vr_map[i].raw, cfg->vr_map[i].angle); 
    }
  }
  else if(argc == 3){
    uint16_t id, raw, ang;
    id = (uint8_t)strtol(argv[0],&edptr,10);
    raw = (uint16_t)strtol(argv[1],&edptr,10);
    ang = (uint16_t)strtol(argv[2],&edptr,10);
    if(id < 4){
      cfg->vr_map[id].raw = raw;
      cfg->vr_map[id].angle = ang;
    }
  }
}

void cmd_spdMap(BaseSequentialStream *chp, int argc, char *argv[]) 
{
  struct _rx_control_config *cfg = control_struct();
  if(argc == 0){
    chprintf(chp,"ID RAW ANGLE\r\n");
    for(uint8_t i=0;i<4;i++){
      chprintf(chp,"#%2d %4d %4d %4d\r\n",i, cfg->speed_map[i].angle, cfg->speed_map[i].low,cfg->speed_map[i].high); 
    }
  }
  else if(argc == 3){
    uint16_t id,ang,low,high;
    id = (uint8_t)strtol(argv[0],NULL,10);
    ang = (uint16_t)strtol(argv[1],NULL,10);
    low = (uint16_t)strtol(argv[2],NULL,10);
    high = (uint16_t)strtol(argv[3],NULL,10);
    if(id < 4){
      cfg->speed_map[id].angle = ang;
      cfg->speed_map[id].low = low;
      cfg->speed_map[id].high = high;
    }
  }
  
}

void cmd_ppr(BaseSequentialStream *chp, int argc, char *argv[]) 
{
  float value;
  uint16_t ppr;
  uint16_t minp,maxp;
  char *endPtr;
  if(argc == 3){
    ppr = (uint16_t)strtol(argv[0],&endPtr,10);
    minp = (uint16_t)strtol(argv[1],&endPtr,10);
    maxp = (uint16_t)strtol(argv[2],NULL,10);
    value = (float)(ppr/360.);
    rf_set_ppd(value,minp,maxp);
  }
  else if(argc == 0){
    rf_get_ppd(&value,&minp,&maxp);
    ppr = (uint16_t)(value * 360);
    chprintf(chp,"PPR=%d MIN_PPS=%d MAX_PPS=%d\n",ppr,minp,maxp);
  }
}

void cmd_txsim(BaseSequentialStream *chp, int argc, char *argv[]) 
{
  if(argc == 2){
    uint16_t dio, adc_raw;
    if(strncmp(argv[0],"0x",2) == 0){
      dio = (uint16_t)strtol(argv[0],NULL,16);
    }
    else{
      dio = (uint16_t)strtol(argv[0],NULL,10);
    }
    adc_raw = (uint16_t)strtol(argv[1],NULL,10);
    
    rf_write_ctrl_state(dio);
    rf_write_analog_state(adc_raw);
  }
}
void cmd_idn(BaseSequentialStream *chp, int argc, char *argv[])
{
  chprintf(chp,"\r\nBoard Name:         %s",BOARD_NAME);
  chprintf(chp,"\r\nFW Version:         %08x",FW_VERSION);
  chprintf(chp,"\r\nFrequency:          %8.1f",rf_getFreq());
  chprintf(chp,"\r\nDataRate:           %d",rf_getDataRate());
  chprintf(chp,"\r\nPower:              %d",rf_getPower());
  chprintf(chp,"\r\nAddress:            %d",rf_getSelfAddr());
  chprintf(chp,"\r\nAddress2:           %d",rf_getDestAddr());
  chprintf(chp,"\r\nInterval(ms):       %d\r\n",rf_get_pollInterval());
}