#ifndef _SHELL_CMD_MENU_
#define _SHELL_CMD_MENU_

#include "hal.h"
#include "ch.h"

#define SHELL_SET_ID    EVENT_MASK(0)
#define SHELL_SET_ID    EVENT_MASK(0)


enum CONFIG_IDs{
  CONFIG_NONE,
  CONFIG_SRC_ID,
  CONFIG_DST_ID,
  CONFIG_FREQUENCY,
  CONFIG_RF_POWER,
  CONFIG_TX_INTERVAL,
  CONFIG_RX_TIMEOUT,
  CONFIG_SAVE = 99
};

typedef void(*shell_cb)(uint8_t,uint32_t);

void shell_cmd_menu_init(BaseSequentialStream *chp, shell_cb cb);

#endif