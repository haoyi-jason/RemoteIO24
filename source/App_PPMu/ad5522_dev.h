#ifndef _AD5522_DEV_
#define _AD5522_DEV_

#include "ad5522_def.h"

typedef struct{
  uint16_t m;
  uint16_t c;
  uint16_t x1;
}_dac_reg_grp_t;

typedef struct{
  // register mapping parameters
  uint8_t enable;
  uint8_t force;
  uint8_t crange;
  uint8_t measout;
  uint8_t fin;
  uint8_t sf;
  uint8_t clamp_enable;
  uint8_t cpolh;
  uint8_t cmp_out;
  uint8_t clear;
  _dac_reg_grp_t *dac;
  // private parameters
  
}_pmu_config_t;

  
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
  _pmu_config_t PMU[NOF_PMU_CHANNEL]
  
typedef struct{
  AD5522Config *config;
  _ad5522_data;
}AD5522Driver;

void ad5522_init(AD5522Driver *dev,const AD5522Config *config );
void ad5522_reset(AD5522Driver *dev,uint8_t channel);
void ad5522_rd_sysconfig(AD5522Driver *dev);
void ad5522_wr_sysconfig(AD5522Driver *dev,uint32_t value);
void ad5522_rd_pmu_register(AD5522Driver *dev, uint8_t pmu_id);
void ad5522_wr_pmu_register(AD5522Driver *dev,uint8_t pmu_id, uint32_t value);
void ad5522_rd_pmu_dac_register(AD5522Driver *dev, uint8_t pmu_id,uint8_t dac_address,uint8_t mode);
void ad5522_wr_pmu_dac_register(AD5522Driver *dev,uint8_t pmu_id, uint8_t dac_address,uint8_t mode, uint16_t value);

void ad5522_load(AD5522Driver *dev, uint8_t state);
#endif
