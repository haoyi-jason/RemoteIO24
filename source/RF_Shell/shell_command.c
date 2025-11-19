#include "ch.h"
#include "hal.h"
#include "protocol/bin_protocol/bin_protocol.h"
#include "shell_command.h"
#include <string.h>
#include <stdlib.h>
#include "database.h"

#define PORT    SD1

static shell_cb cb_func = NULL;

enum COMMAND_CODE_e{
  CMD_NONE,
  CMD_RF_CONFIG,
  CMD_BOARD_INFO,
  CMD_SETPPER_CONFIG,
  CMD_RSV_1,
  CMD_MAP_A,
  CMD_MAP_B,
};

typedef struct{
  thread_t *shelltp;
  thread_t *mainThread;
}_runTime;

static _runTime shellRuntime;

void cmd_rf_config(BaseSequentialStram *chp, char *buffer,uint8_t maxLen)
{
  uint8_t buffer[128];
  if(size == 0){
    
  }
  
}

void cmd_model_type(BaseSequentialStram *chp, char *buffer,uint8_t maxLen)
{
  
}

void cmd_tx_v1_config(BaseSequentialStram *chp, char *buffer,uint8_t maxLen)
{
  
}

void cmd_switch_tx_config(BaseSequentialStram *chp, char *buffer,uint8_t maxLen)
{
  
}

void cmd_switch_rx_config(BaseSequentialStram *chp, char *buffer,uint8_t maxLen)
{
  
}
void cmd_rx_v1_config(BaseSequentialStram *chp, char *buffer,uint8_t maxLen)
{
  
}

void cmd_rx_v2_config(BaseSequentialStram *chp, char *buffer,uint8_t maxLen)
{
  
}

static const BINProtocol_Command bin_commands[] = {
  {CMD_RF_CONFIG, cmd_rf_config},
  {CMD_BOARD_INFO,cmd_board_info},
  {CMD_SETPPER_CONFIG,cmd_steooer_config},
  {CMD_MAP_A,cmd_map_a},
  {CMD_MAP_B,cmd_map_b},
  {0x00,0x00}
};

static SerialConfig serialCfg={
  9600
};

static BINProtocolConfig bin_config = {
  (BaseSequentialStream*)&PORT,
  0x01,
  bin_commands
};

#define SHELL_WA_SIZE   1024
static THD_WORKING_AREA(waShell,SHELL_WA_SIZE);

void task_binProtocolInit(BINProtocolConfig *config)
{
  sdStart(&PORT,&serialCfg);
  binProtoRuntime.shelltp = chThdCreateStatic(waShell, sizeof(waShell),NORMALPRIO+1,binProtocolProc,(void*)&bin_config);

  binProtoRuntime.mainThread = chRegFindThreadByName("Main");
}
