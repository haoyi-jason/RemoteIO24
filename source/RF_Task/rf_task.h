#ifndef _RF_TASK_
#define _RF_TASK_
#include "ch.h"
#include "hal.h"

#include "app_defs.h"

#define EV_TX                   EVENT_MASK(CMD_TX)
//#define EV_SAVE               EVENT_MASK(1)
#define EV_RX_PACKET            EVENT_MASK(CMD_RX_PACKET)
#define EV_RX_ERROR             EVENT_MASK(CMD_RX_ERROR)
#define EV_RX_LOST              EVENT_MASK(CMD_RX_LOST)
#define EV_TX_DATA_REQUEST      EVENT_MASK(CMD_TX_DATA_REQUEST)
#define EV_TX_DONE              EVENT_MASK(CMD_TX_DONE)


struct _rx_control_config {
  struct{
    uint16_t raw;
    uint16_t angle;
  }vr_map[4];
  struct{
    int16_t angle;
    int16_t low;
    int16_t high;
  }speed_map[4];
  float pulsePerDegree;
  uint16_t max_pps;
  uint16_t min_pps;
  struct{
    uint16_t raw;
    uint16_t counter;
  }pwm_map[4];
};


int8_t rf_task_init();
uint16_t read_ctrl_state();
uint16_t read_analog_state(uint8_t channel);
uint8_t rf_op_mode();
void rf_write_dio_state(uint16_t value);
void rf_write_analog_state(uint16_t value);

void rf_send(uint8_t *d, uint8_t n);
void rf_setDestAddr(uint16_t addr);
uint16_t rf_getDestAddr();
uint16_t rf_getSelfAddr();
void rf_setSelfAddr(uint16_t addr);
void rf_setPower(uint8_t value);
uint8_t rf_getPower();
void rf_setFreq(float value);
float rf_getFreq();
void rf_setDataRate(uint8_t value);
uint8_t rf_getDataRate();
void rf_setMode(uint8_t mode);
uint8_t rf_getMode();
void rf_saveParam();
void rf_set_io_pattern(uint16_t val);
uint16_t rf_get_io_pattern();
void rf_set_pollInterval(uint16_t val);
uint16_t rf_get_pollInterval();
void rf_set_pollCount(uint16_t val);
uint16_t rf_get_pollCount();
void rf_task_start_manual_mode(BaseSequentialStream *stream);
void rf_task_stop_manual_mode();

struct _rx_control_config *control_struct();

void rf_set_ppd(float value,uint16_t minp, uint16_t maxp);
void rf_get_ppd(float *value,uint16_t *minp, uint16_t *maxp);

void rf_write_ctrl_state(uint16_t value);

void rf_save_nvm();
#endif