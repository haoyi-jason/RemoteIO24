#ifndef _RF_TASK_
#define _RF_TASK_
#include "ch.h"
#include "hal.h"

#define EV_TX           EVENT_MASK(0)
#define EV_SAVE         EVENT_MASK(1)
#define EV_RX_PACKET    EVENT_MASK(2)
#define EV_RX_ERROR     EVENT_MASK(3)
#define EV_RX_LOST      EVENT_MASK(4)
#define EV_TX_DATA_REQUEST      EVENT_MASK(5)
#define EV_TX_DONE      EVENT_MASK(7)


int8_t rf_task_init();
uint16_t read_ctrl_state();
uint16_t read_analog_state(uint8_t channel);
uint8_t rf_op_mode();
void rf_write_dio_state(uint16_t value);
void rf_write_analog_state(uint16_t value);


#endif