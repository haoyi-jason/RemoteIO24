#ifndef _BINARY_PROTOCOL_TASK_
#define _BINARY_PROtoCOL_TASK_

#include "protocol/bin_protocol/bin_protocol.h"

void binaryProtocolInit();
void bp_send_packet(uint8_t *data, uint8_t size);

#endif