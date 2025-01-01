#ifndef _AD5522_DEF_
#define _AD5522_DEF_

#define NOF_PMU_CHANNEL 4

enum PMU_ID_e{
  PMU_SYSCFG      =0x0,
  PMU_CHANNEL_0,
  PMU_CHANNEL_1,
  PMU_CHANNEL_2,
  PMU_CHANNEL_3,
};
#define PMU_REG_SELECT(x)   (1 << (23+x))

enum PMU_MODE_e{
  PMU_MODE_WR_SYS_REG,
  PMU_MODE_WR_DAC_GAIN,
  PMU_MODE_WR_DAC_OFFSET,
  PMU_MODE_WR_DAC_X1
};
#define PMU_REG_MODE(x)     (x << 22)
#define PMU_REG_WR_MASK         0
#define PMU_REG_READ_MASK       (1 << 28)

#define CMD_NOP         (0x3 << 22)
#define CMD_WR_SYSCTRL  (0x0)


// system control register
#define PMU_SYS_REG_CGALM_LATCH         (1 << 2)
#define PMU_SYS_REG_TMP_TS_DISABLE      (0x0 << 5)
#define PMU_SYS_REG_TMP_TS_ENABLE       (0x01 << 5)
#define PMU_SYS_REG_TMP_TS_130          (0x0 << 3)
#define PMU_SYS_REG_TMP_TS_120          (0x1 << 3)
#define PMU_SYS_REG_TMP_TS_110          (0x2 << 3)
#define PMU_SYS_REG_TMP_TS_100          (0x3 << 3)

#define PMU_SYS_REG_GAIN_X5             (1 << 6)
#define PMU_SYS_REG_GAIN_X10             (0 << 6)
#define PMU_SYS_REG_GAIN_UP             (1 << 7)
#define PMU_SYS_REG_GAIN_BP             (0 << 7)

#define PMU_SYS_REG_GUARD_EN            ( 1 << 8)
#define PMU_SYS_REG_DIS_INT10K          (1 << 9)
#define PMU_SYS_REG_EN_CLAMP_ALM           (1 << 10)
#define PMU_SYS_REG_EN_GUARD_ALM           (1 << 11)
#define PMU_SYS_REG_EN_DUTGND           (1 << 12)
#define PMU_SYS_REG_EN_CPBIAS           (1 << 13)

#define PMU_SYS_REG_CPOLH_EN(x)         (1 << (x+14))
#define PMU_SYS_REG_CL_EN(x)            (1 << (x + 18))

// PMU registers
#define PMU_PMU_REG_RESET               (1 << 6)
#define PMU_PMU_REG_CMP(x)              (x << 7)
#define PMU_PMU_REG_CMP_VOLT            (1)
#define PMU_PMU_REG_CMP_CURR            (0)
#define PMU_PMU_REG_CPOLH               (1 << 8)
#define PMU_PMU_REG_CL_ENABLE           (1 << 9)
// SS/SF0 undefined
#define PMU_PMU_REG_FORCE_IN            (1 << 12)

#define PMU_PMU_REG_MEAS(x)             (x << 13)
#define PMU_PMU_REG_MEAS_IIN            (0)
#define PMU_PMU_REG_MEAS_VIN            (1)
#define PMU_PMU_REG_MEAS_TMP            (2)
#define PMU_PMU_REG_MEAS_HZ             (3)

#define PMU_PMU_REG_CRANGE(x)           (x << 15)
#define PMU_PMU_REG_CRANGE_5_UA         (0)
#define PMU_PMU_REG_CRANGE_20_UA        (1)
#define PMU_PMU_REG_CRANGE_200_UA       (2)
#define PMU_PMU_REG_CRANGE_2_MA         (3)
#define PMU_PMU_REG_CRANGE_EXT          (4)

#define PMU_PMU_REG_FORCE(x)            (x << 19)
#define PMU_PMU_REG_FORCE_FVCI          (0)
#define PMU_PMU_REG_FORCE_FICV          (1)
#define PMU_PMU_REG_FORCE_FVHZ          (2)
#define PMU_PMU_REG_FORCE_FCHZ          (3)

#define PMU_PMU_REG_CHANNEL_EN          (1 << 21)


// DAC register address
#define DAC_OFFSET_X                    0x0
enum DAC_REGISTER_ADDRESS_e{
  DAC_ADDRESS_OFFSET_X = 0x0,
  DAC_ADDRESS_I_5UA = 0x08,
  DAC_ADDRESS_I_20UA,
  DAC_ADDRESS_I_200UA,
  DAC_ADDRESS_I_2MA,
  DAC_ADDRESS_I_EXT,
  DAC_ADDRESS_V,
  DAC_ADDRESS_I_CLL = 0x14,
  DAC_ADDRESS_V_CLL,
  DAC_ADDRESS_I_CLH = 0x1C,
  DAC_ADDRESS_V_CLH,
  DAC_ADDRESS_I_CPL_5UA = 0x20,
  DAC_ADDERSS_I_CPL_20UA,
  DAC_ADDRESS_I_CPL_200UA,
  DAC_ADDRESS_I_CPL_2MA,
  DAC_ADDRESS_I_CPL_EXT,
  DAC_ADDRESS_V_CPL,
  DAC_ADDRESS_I_CPH_5UA = 0x28,
  DAC_ADDRESS_I_CPH_20UA,
  DAC_ADDRESS_I_CPH_200UA,
  DAC_ADDRESS_I_CPH_2MA,
  DAC_ADDRESS_I_CPH_EXT,
  DAC_ADDRESS_V_CPH,
};
#define NOF_DAC_ADDRESS 23

enum DAC_REGISTER_MODE_e{
  DAC_REGISTER_M = 0x01,
  DAC_REGISTER_C,
  DAC_REGISTER_X1
};



#endif