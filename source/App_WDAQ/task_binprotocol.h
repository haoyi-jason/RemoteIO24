#ifndef _TASK_BINPROTOCOL_
#define _TASK_BINPROTOCOL_

#include "protocol/bin_protocol/bin_protocol.h"

void task_binProtocolInitw();

void board_nvm_load_default();
void board_nvm_save();
void send_packet(uint8_t *data, uint8_t size);
#endif