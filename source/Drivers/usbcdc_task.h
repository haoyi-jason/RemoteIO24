#ifndef _USBCDC_TASK_
#define _USBCDC_TASK_


void usbcdc_task_init(void *arg);
void usbcdc_task_stop();
uint8_t usb_cdc_active();
#endif