#include "ch.h"
#include "hal.h"
#include "shell_command.h"

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
  }
}

