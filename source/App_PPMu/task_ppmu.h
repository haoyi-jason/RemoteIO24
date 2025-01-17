#ifndef _TASK_PPMU_
#define _TASK_PPMU_

#define NOF_CHANNEL_PER_PMU     4
#define NOF_AD5522                1
#define TOTAL_PMU_CHANNEL       NOF_AD5522 * NOF_CHANNEL_PER_PMU

enum OUTPUT_MODE_e{
  PMU_OUTPUT_FVMI,
  PMU_OUTPUT_FIMV,
  PMU_OUTPUT_HZ
};

void pmu_init();
void pmu_save_state();
void pmu_set_dac_output(uint8_t channel, uint8_t reg, uint16_t value);
void pmu_set_dac_gain(uint8_t channel, uint8_t reg, uint16_t value);
void pmu_set_dac_offset(uint8_t channel, uint8_t reg, uint16_t value);

void pmu_set_output(uint8_t channel, uint16_t value);
void pmu_set_output_mode(uint8_t channel, uint8_t mode);
void pmu_set_crange(uint8_t channel, uint8_t value);
void pmu_enable(uint8_t channel, bool enable);
void pmu_measout(uint8_t channel, uint8_t value);
void pmu_sys_force(uint8_t channel, uint8_t value);
void pmu_cl(uint8_t channel, uint8_t value);
void pmu_cpolh(uint8_t channel, uint8_t value);
void pmu_clear(uint8_t channel, uint8_t value);
void pmu_reg_flush(uint8_t channel);

void pmu_load(uint8_t id);
void pmu_unload(uint8_t id);
#endif