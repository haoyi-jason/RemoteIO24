#ifndef _TASK_PPMU_
#define _TASK_PPMU_

#define NOF_CHANNEL_PER_PMU     4
#define NOF_AD5522                4
#define TOTAL_PMU_CHANNEL       NOF_AD5522 * NOF_CHANNEL_PER_PMU

enum OUTPUT_MODE_e{
  PMU_OUTPUT_FVMI,
  PMU_OUTPUT_FIMV,
  PMU_OUTPUT_HZ
};

void pmu_init();
void pmu_save_state();
void pmu_set_dac_output(uint8_t channel, uint8_t reg, uint16_t value);
uint16_t pmu_get_dac_output(uint8_t channel, uint8_t reg);
uint16_t pmu_get_dac_output_cached(uint8_t channel, uint8_t reg);
void pmu_set_dac_gain(uint8_t channel, uint8_t reg, uint16_t value);
uint16_t pmu_get_dac_gain(uint8_t channel, uint8_t reg);
uint16_t pmu_get_dac_gain_cached(uint8_t channel, uint8_t reg);
void pmu_set_dac_offset(uint8_t channel, uint8_t reg, uint16_t value);
uint16_t pmu_get_dac_offset(uint8_t channel, uint8_t reg);
uint16_t pmu_get_dac_offset_cached(uint8_t channel, uint8_t reg);

void pmu_set_output(uint8_t channel, uint16_t value);

void pmu_set_output_mode(uint8_t channel, uint8_t mode);
uint8_t pmu_get_output_mode(uint8_t channel);

void pmu_set_crange(uint8_t channel, uint8_t value);
uint8_t pmu_get_crange(uint8_t channel);

void pmu_enable(uint8_t channel, bool enable);
uint8_t pmu_get_enable(uint8_t channel);

void pmu_set_measout(uint8_t channel, uint8_t value);
uint8_t pmu_get_measout(uint8_t channel);

void pmu_sys_force(uint8_t channel, uint8_t value);
uint8_t pmu_get_sys_force(uint8_t channel);

void pmu_set_clamp(uint8_t channel, uint8_t value);
uint8_t pmu_get_clamp(uint8_t channel);

void pmu_set_cpolh(uint8_t channel, uint8_t value);
uint8_t pmu_get_cpolh(uint8_t channel);

void pmu_clear(uint8_t channel, uint8_t value);

void pmu_reg_flush(uint8_t channel);

void pmu_load(uint8_t id);
void pmu_unload(uint8_t id);

void pmu_set_enable(uint8_t channel, uint8_t mode);
uint8_t pmu_get_enable(uint8_t channel);

void pmu_set_fin(uint8_t channel, uint8_t mode);
uint8_t pmu_get_fin(uint8_t channel);

void pmu_set_ccomp(uint8_t range);
uint8_t pmu_get_ccomp();
void pmu_set_frange(uint8_t range);
uint8_t pmu_get_frange();

void pmu_nvm_load_default();
void pmu_nvm_save();
void pmu_set_init_state(uint8_t state);
uint8_t pmu_get_init_state();

void pmu_set_dutgnd(uint8_t id, uint8_t state);
uint8_t pmu_get_dutgnd(uint8_t id);

void pmu_set_int10k(uint8_t id, uint8_t state);
uint8_t pmu_get_int10k(uint8_t id);

void pmu_set_outgain(uint8_t id, uint8_t state);
uint8_t pmu_get_outgain(uint8_t id);

uint8_t pmu_fill_board_registers(uint8_t *dptr);
uint8_t pmu_fill_dac_registers(uint8_t channel, uint8_t *dptr);
#endif