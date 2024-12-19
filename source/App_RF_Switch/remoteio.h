#ifndef _REMOTEIO_
#define _REMOTEIO_

#define SWITCH_TX_PATTERN       0x8000  // bit 15

int8_t remoteio_task_init();

void remoteio_send(uint8_t *d, uint8_t n);
void remoteio_setDestAddr(uint16_t addr);
void remoteio_setSelfAddr(uint16_t addr);
void remoteio_setPower(uint8_t value);
void remoteio_setFreq(float value);
void remoteio_setDataRate(uint8_t value);
void remoteio_setMode(uint8_t mode);
void remoteio_saveParam();
void remoteio_start_task(BaseSequentialStream *stream);
void remoteio_stop_task();

uint16_t remoteio_getDestAddr();
uint16_t remoteio_getSelfAddr();
uint8_t remoteio_getPower();
uint16_t remteio_getFreq();
uint8_t getDataRate();
uint8_t remoteio_getMode();

void remoteio_set_io_pattern(uint16_t val);
uint16_t remoteio_get_io_pattern();

void remoteio_set_pollInterval(uint16_t val);
uint16_t remoteio_get_pollInterval();
void remoteio_set_pollCount(uint16_t val);
uint16_t remoteio_get_pollCount();
int8_t txStopped();

#endif
