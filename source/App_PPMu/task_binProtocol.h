#ifndef _TASK_BINPROTOCOL_
#define _TASK_BINPROTOCOL_

#include "protocol/bin_protocol/bin_protocol.h"

void task_binProtocolInit(BINProtocolConfig *config);

void board_nvm_load_default();
void board_nvm_save();
#endif