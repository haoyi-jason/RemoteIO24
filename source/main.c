#include "ch.h"
#include "hal.h"

#ifdef APP_MODULE
#include "app_module/app.h"
#endif

int main()
{
  halInit();
  chSysInit();
  app_init(0);
  return 1;
}