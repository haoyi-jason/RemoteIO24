#ifndef _AD5522_DEV_
#define _AD5522_DEV_

#include "ad5522_def.h"

typedef struct{
  uint8_t force;
  uint8_t crange;
  uint8_t measout;
  uint8_t fin;
  uint8_t sf;
}_pmu_config_t;

typedef struct{
  uint16_t m;
  uint16_t c;
  uint16_t x1;
}_dac_reg_grp_t;
  
typedef struct{
  SPIDriver *devp;
  const SPIConfig *config;
  ioportid_t    ssport;
  ioportmask_t  sspad;
  ioportid_t    rstport;
  ioportmask_t  rstpad;
  ioportid_t    busyport;
  ioportmask_t  busypad;
  ioportid_t    loadport;
  ioportmask_t  loadpad;
}AD5522Config;

#define _ad5522_data \
  uint8_t id; \
  event_source_t ev_data; \
  uint32_t sys_config; \
  _pmu_config_t PMU[4]; \
  _dac_reg_grp_t dac_registers[NOF_DAC_ADDRESS]

  
typedef struct{
  AD5522Config *config;
  _ad5522_data;
}AD5522Driver;

void ad5522_init(AD5522Driver *dev,const AD5522Config *config );


#endif
