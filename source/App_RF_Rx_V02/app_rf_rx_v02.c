#include "ch.h"
#include "hal.h"
#include "../drivers/bc3601.h"
#include "shell.h"
#include "../rf_shell/shell_cmd_menu.h"
#include "../rf_task/rf_task.h"
#include "../drivers/bc3601.h"
#include "../drivers/dev_bc3601.h"
#include "../drivers/nvm.h"
#include "nvm_config.h"
#include "stepper_task.h"
#include "stepper.h"
#include "app_defs.h"
#include "database.h"
#include "binaryProtocolTask.h"
//#include "encryption/crc8/crc8.h"

#define ADC_GRP1_NUM_CHANNELS           2
#define ADC_GRP1_NUM_CHANNELS_TX2       5
#define ADC_GRP1_BUF_DEPTH              8

#define LINE_TX_ACT     PAL_LINE(GPIOC,6)
#define LINE_LBT_ACT     PAL_LINE(GPIOC,7)
//#define VR_POWER_LINE   PAL_LINE(GPIOB,11)

#define VR_POWER_DIS()  palSetLine(VR_POWER_LINE)
#define VR_POWER_EN() palClearLine(VR_POWER_LINE) 

#define APP_SIGNATURE_KEY       0xCC
#define APP_SIGNATURE_OFFSET    0

//#define MOTOR_CONTROL   
#define PWM_CONTROL     
#define PWM_FULL_OUTPUT         180
#define PWM_PERIOD_COUNT        5000
#define PWM_REVERSE_POLARITY    false


#define PWM_VALVE_IDLE_OUTPUT     0
#define PWM_VALVE_BASE          125
#define PWM_VALVE_PERIOD        (300 - PWM_VALVE_BASE)
#define PWM_VALVE_FULL          500

#define BOARD_B2

//#define MODE_TX
#define MODE_RX

static void tx_control_loop();
static void rx_control_loop();
//static void tx_switch_loop();
//static void rx_switch_loop();

static PWMConfig pwm_config_2 = {
  250000,
  5000, // 250000/5000 = 50Hz , 0~20 ms, for 0.5ms = 125, 2.5ms = 625
  NULL,
  {
    {PWM_OUTPUT_ACTIVE_HIGH,NULL},
    {PWM_OUTPUT_ACTIVE_HIGH,NULL},
    {PWM_OUTPUT_ACTIVE_HIGH,NULL},
    {PWM_OUTPUT_ACTIVE_HIGH,NULL},
  },
  0,
  0
};
/*
  TIM3/4 PWM control for Valve control, just "PWM"
  80000/500  = 160 hz pwm signal, control duty use only 50%

*/

static PWMConfig pwm_config_3 = {
  80000,
  500, 
  NULL,
  {
    {PWM_OUTPUT_ACTIVE_HIGH,NULL},  // PA6, CH.0
    {PWM_OUTPUT_ACTIVE_HIGH,NULL},  // PA7  CH.1
    {PWM_OUTPUT_ACTIVE_HIGH,NULL},  // PB0  CH.2
    {PWM_OUTPUT_ACTIVE_HIGH,NULL},  // PB1  CH.3  or PC9 for GEAR PWM
  },
  0,
  0
};

static PWMConfig pwm_config_4 = {
  80000,
  500, 
  NULL,
  {
    {PWM_OUTPUT_ACTIVE_HIGH,NULL},      // PB6, CH.7
    {PWM_OUTPUT_ACTIVE_HIGH,NULL},      // PB.7, CH.6
    {PWM_OUTPUT_ACTIVE_HIGH,NULL},      // PB.8, CH.5
    {PWM_OUTPUT_ACTIVE_HIGH,NULL},      // PB.9, CH.4
  },
  0,
  0
};

static PWMConfig pwm_config_8 = {
  10000,
  500, // 10000/5000 = 200Hz 
  NULL,
  {
    {PWM_OUTPUT_ACTIVE_HIGH,NULL},
    {PWM_OUTPUT_ACTIVE_HIGH,NULL},
    {PWM_OUTPUT_ACTIVE_HIGH,NULL},
    {PWM_OUTPUT_ACTIVE_HIGH,NULL},
  },
  0,
  0
};

static const SPIConfig spicfg = {
  false,
  NULL,
  NULL,//GPIOB,
  NULL,//GPIOB_SPI1_CS,
  SPI_CR1_BR_2  
};

static bc3601_config_t config = {
  &SPID2,
  &spicfg,
  GPIOB,
  12
};

static BC3601Driver bc3601;

struct _deviceConfig {
  uint8_t flag;
  uint16_t selfAddr;
  uint16_t destAddr;
  uint8_t opMode;  // 1: rf switch tx, 2: rf switch rx, 3: rf i/o tx, 4: rf i/o rx
  uint8_t txIntervalMs; // tx interval, in ms
  uint16_t txCounts; // number of times to transmit packet
  struct _rx_control_config rx_control;
  uint16_t timeout_ms;
  uint8_t reserved[11]; // reserve total 64-bytes for this struct
};

struct _rfConfig{
  uint8_t dataRate; // 0 to 6, 2/5/10/25/50/125/250K
  uint8_t txPower;
  float frequency; 
  uint8_t reserved[58]; // reserve 64-bytes for this struct
};

struct _nvm{
  struct _deviceConfig deviceConfig;
  struct _rfConfig rfConfig;
  uint16_t tx_dio;
};

//struct _nvm nvmParam, *rfNvmParam;

static struct{
  thread_t *self;
  thread_t *mainThread;
  thread_t *usbtrx;
  thread_t *opThread;
  thread_t *rxThread;
  thread_reference_t ref;
  uint8_t rssi;
  uint16_t rf_ctrl;
  uint16_t cycles;
  uint8_t stage;
  uint16_t rxLost;
  uint8_t taskFinished;
  uint16_t dio_state;
  uint16_t ain_value;
  uint8_t rfLinkFail;
  int8_t pwmDirection[4];
  bool userMode;
  uint8_t opMode;
  uint16_t bat_mv;
  struct _stepper_control_s stepper;
  uint8_t trx_type;
  uint8_t gear_type;
}runTime;

typedef struct {
  uint8_t sz;
  uint8_t data[64];
}_txPacket;

static _txPacket txPacket;
static _txPacket rxPacket;

typedef struct{
  uint16_t srcAddr;
  uint16_t dstAddr;
  uint16_t dio; // bit 0~9 for action, bit 15 for switch
  uint16_t advalue[4];
  uint8_t rsv1;
  uint8_t checksum;
}_rfPacket;


static void load_settings()
{
  uint8_t app_signature = db_read_df_u8(APP_SIGNATURE + APP_SIGNATURE_OFFSET);
  if(app_signature != APP_SIGNATURE_KEY){
      //db_enable_save_on_write(false);
      
      db_write_df_u16(DEVICE_ADDR,0x01);
      db_write_df_u16(DEST_ADDR,0x01);
#ifdef MODE_RX
//      db_write_df_u8(OP_MODE,0x27);
      db_write_df_u8(OP_MODE,OP_MODE_RX | OP_MODE_CONTROL | GEAR_TYPE(GEAR_SERVO) | VALVE_TYPE(VALVE_PWM));
#endif
#ifdef MODE_TX
//      db_write_df_u8(OP_MODE,0x12);
      db_write_df_u8(OP_MODE,OP_MODE_TX | OP_MODE_CONNTROL);
#endif
      db_write_df_u8(TX_INTERVAL_MS,50);
      db_write_df_u16(TIMEOUT_MS,2000);
      //db_write_df_u16(TX_COUNTS,90);
      
      db_write_df_u8(RF_DATA_RATE_CODE, 1);
      db_write_df_f32(RF_BASE_FREQUENCY,915.00);
      db_write_df_u8(RF_TX_POWER,1);
      
      db_write_df_u16(TX_DIO_MASK, 0xC000);
      db_write_df_u16(BLINK_PERIOD_MS,500);
      
      db_write_df_u16(VR_ANGLE_PT_1,0);
      db_write_df_u16(VR_ANGLE_PT_2,30);
      db_write_df_u16(VR_ANGLE_PT_3,170);
      db_write_df_u16(VR_ANGLE_PT_4,200);
      db_write_df_u16(VR_RAW_PT_1,800);
      db_write_df_u16(VR_RAW_PT_2,2600);
      db_write_df_u16(VR_RAW_PT_3,3600);
      db_write_df_u16(VR_RAW_PT_4,4000);
      
      db_write_df_f32(MOTOR_MAP_PPD,(200*10)/360.);
      db_write_df_u16(STEPPER_MIN_PPS, 500);
      db_write_df_u16(STEPPER_MAX_PPS,1500);
      
      db_write_df_u16(PWM_MAP_RAW_PT_1,200);
      db_write_df_u16(PWM_MAP_RAW_PT_2,3300);
      db_write_df_u16(PWM_MAP_RAW_PT_3,3800);
      db_write_df_u16(PWM_MAP_RAW_PT_4,4000);
      db_write_df_u16(PWM_MAP_COUNTER_PT_1,125);
      db_write_df_u16(PWM_MAP_COUNTER_PT_2,250);
      db_write_df_u16(PWM_MAP_COUNTER_PT_3,300);
      db_write_df_u16(PWM_MAP_COUNTER_PT_4,500);

      db_write_df_u16(SERVO_MAP_RAW_PT_1,200);
      db_write_df_u16(SERVO_MAP_RAW_PT_2,3300);
      db_write_df_u16(SERVO_MAP_RAW_PT_3,3800);
      db_write_df_u16(SERVO_MAP_RAW_PT_4,4000);
      db_write_df_u16(SERVO_MAP_COUNTER_PT_1,125);
      db_write_df_u16(SERVO_MAP_COUNTER_PT_2,250);
      db_write_df_u16(SERVO_MAP_COUNTER_PT_3,300);
      db_write_df_u16(SERVO_MAP_COUNTER_PT_4,500);

      db_write_df_u16(TX_LBT_LOW_BOUND,5000);
      db_write_df_u16(TX_LBT_HIGH_BOUND,5100);

      db_write_df_u16(DIO_ON_MASK,0x0200);
      
      //db_enable_save_on_write(true);
      db_write_df_u8(APP_SIGNATURE + APP_SIGNATURE_OFFSET, APP_SIGNATURE_KEY);
      
      db_save_section(U8);
      db_save_section(U16);
      db_save_section(F32);
  }
  
  // test mode for TX Control
  //db_write_df_u8(OP_MODE, OP_MODE_TX | TX_MODE_CONTROL);
  // test mode for TX Switch
  //db_write_df_u8(OP_MODE, OP_MODE_TX | TX_MODE_SWITCH);
  
  // test mode for TX Control, Servo, PWM
  //db_write_df_u8(OP_MODE, OP_MODE_RX | RX_MODE_CONTROL | RX_GEAR_SERVO | RX_VALVE_PWM);

  // test mode for TX Control, Stepper, PWM
  //db_write_df_u8(OP_MODE, OP_MODE_RX | RX_MODE_CONTROL  | RX_VALVE_PWM);

  // test mode for TX Control, Stepper, IO
  //db_write_df_u8(OP_MODE, OP_MODE_RX | RX_MODE_CONTROL);
  
  db_write_df_u8(OP_MODE, OP_MODE_RX | OP_MODE_CONTROL | GEAR_TYPE(GEAR_PWM) | VALVE_TYPE(VALVE_IO));
}

void rf_save_nvm()
{
  //nvm_flash_write(OFFSET_NVM_BOARD,(uint8_t*)&nvmParam,sizeof(nvmParam));
}

uint8_t checksum(uint8_t *d, uint8_t n)
{
  uint8_t sum = 0;
  for(uint8_t i=0;i<n;i++){
    sum += *d++;
  }
  return sum;
}

#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(512)
static THD_WORKING_AREA(waShell,SHELL_WA_SIZE);


static adcsample_t samples[ADC_GRP1_NUM_CHANNELS*ADC_GRP1_BUF_DEPTH];
static adcsample_t samples_tx2[ADC_GRP1_NUM_CHANNELS_TX2*ADC_GRP1_BUF_DEPTH];

static void adccallback(ADCDriver *adcp);

static void adcerror(ADCDriver *adcp, adcerror_t err);
static  virtual_timer_t vt_blink;

static void adccallback(ADCDriver *adcp)
{
//  while(1);
}

static void adcerror(ADCDriver *adcp, adcerror_t err)
{
//  while(1);
}

static const ADCConversionGroup adcgrpcfg = {
  TRUE,
  ADC_GRP1_NUM_CHANNELS,
  adccallback,
  adcerror,
  0,
  ADC_CR2_SWSTART,
  0,
  ADC_SMPR2_SMP_AN0(ADC_SAMPLE_239P5) | ADC_SMPR2_SMP_AN1(ADC_SAMPLE_239P5),
  ADC_SQR1_NUM_CH(ADC_GRP1_NUM_CHANNELS),
  0,
#ifdef BOARD_A2
  ADC_SQR3_SQ1_N(ADC_CHANNEL_IN8) | ADC_SQR3_SQ2_N(ADC_CHANNEL_IN9)
#endif
#ifdef BOARD_B2
  ADC_SQR3_SQ1_N(ADC_CHANNEL_IN2) | ADC_SQR3_SQ2_N(ADC_CHANNEL_IN1)
#endif
};

static const ADCConversionGroup adcgrpcfg_tx2 = {
  TRUE,
  ADC_GRP1_NUM_CHANNELS_TX2,
  adccallback,
  adcerror,
  0,
  ADC_CR2_SWSTART,
  0,
  ADC_SMPR2_SMP_AN0(ADC_SAMPLE_239P5) | ADC_SMPR2_SMP_AN1(ADC_SAMPLE_239P5) |
  ADC_SMPR2_SMP_AN2(ADC_SAMPLE_239P5) | ADC_SMPR2_SMP_AN3(ADC_SAMPLE_239P5) |
  ADC_SMPR2_SMP_AN4(ADC_SAMPLE_239P5),
  ADC_SQR1_NUM_CH(ADC_GRP1_NUM_CHANNELS_TX2),
  0,
#ifdef BOARD_A2
  ADC_SQR3_SQ1_N(ADC_CHANNEL_IN8) | ADC_SQR3_SQ2_N(ADC_CHANNEL_IN9)
#endif
#ifdef BOARD_B2
  ADC_SQR3_SQ1_N(ADC_CHANNEL_IN1) | ADC_SQR3_SQ2_N(ADC_CHANNEL_IN6) |
  ADC_SQR3_SQ1_N(ADC_CHANNEL_IN7) | ADC_SQR3_SQ2_N(ADC_CHANNEL_IN8) |
  ADC_SQR3_SQ1_N(ADC_CHANNEL_IN9) 
#endif
};

static const SerialConfig ser_cfg = {
  115200
};

typedef struct{
  ioportid_t port;
  ioportmask_t line;
}_gpio_def_t;

//static _gpio_def_t digital_in[] = {
//};
#ifdef BOARD_A2
static _gpio_def_t digital_out_rx[] = {
  {GPIOA,15},
  {GPIOA,1},
  {GPIOA,2},
  {GPIOA,3},
  {GPIOA,4},
  {GPIOA,5},
  {GPIOA,6},
  {GPIOA,7},
  {GPIOC,9},
  {GPIOC,9},  
  {GPIOC,9},
};

static _gpio_def_t digital_in_tx[] = {
  {GPIOA,15},
  {GPIOA,1},
  {GPIOA,2},
  {GPIOA,3},
  {GPIOA,4},
  {GPIOA,5},
  {GPIOA,6},
  {GPIOA,7},
  {GPIOC,8}, // aux input
  {GPIOC,9}
};
static _gpio_def_t digital_out_tx[] = {
  {GPIOB,11},    // VR power
  {GPIOC,6},    // led
  {GPIOC,7},    // led
};
#undef VR_POWER_LINE
#define VR_POWER_LINE   PAL_LINE(GPIOB,11)
#endif

#ifdef BOARD_B2
static _gpio_def_t digital_out_rx[] = {
  {GPIOA,6},
  {GPIOA,7},
  {GPIOB,0},
  {GPIOB,1},
  {GPIOB,9},
  {GPIOB,8},
  {GPIOB,7},
  {GPIOB,6},
  {GPIOC,9},
  {GPIOA,2},  
  {GPIOA,3},
};
static _gpio_def_t digital_in_tx[] = {
  {GPIOA,6},
  {GPIOA,7},
  {GPIOB,0},
  {GPIOB,1},
  {GPIOB,9},
  {GPIOB,8},
  {GPIOB,7},
  {GPIOB,6},
  {GPIOC,8}, // aux input
  //{GPIOC,9}
};
static _gpio_def_t digital_out_tx[] = {
  {GPIOA,3},
  {GPIOC,6},
  {GPIOC,7},    
};
static _gpio_def_t digital_in_tx_ana[] = {
  {GPIOB,9},
  {GPIOB,8},
  {GPIOB,7},
  {GPIOB,6},
  {GPIOA,2},
  {GPIOA,3},
  {GPIOC,7},
  {GPIOC,6},
  {GPIOA,11}, // turtle
};
static _gpio_def_t digital_out_tx_ana[] = {
  {GPIOC,9},
  {GPIOC,8},
  {GPIOA,12},    
};

#undef VR_POWER_LINE
#define VR_POWER_LINE   PAL_LINE(GPIOA,3)
#endif

static void bc3601_set_addr()
{
  uint8_t reg = 0;
  // set address, 14-bit format
  reg = db_read_df_u16(DEVICE_ADDR) & 0x3F;
  BC3601_REG_WRITE(&bc3601,HEADER_ADDR0_REGS,&reg); 
  BC3601_REG_READ(&bc3601,HEADER_ADDR0_REGS,&reg); 
  reg = (db_read_df_u16(DEVICE_ADDR)>>8);
  BC3601_REG_WRITE(&bc3601,HEADER_ADDR1_REGS,&reg); 
  BC3601_REG_READ(&bc3601,HEADER_ADDR1_REGS,&reg);
}

static void blink_cb(void *arg)
{
 //virtual_timer_t *vt = (virtual_timer_t*)arg;
  chSysLockFromISR();
  palToggleLine(LINE_TX_ACT);
  chVTSetI(&vt_blink,TIME_MS2I(db_read_df_u16(BLINK_PERIOD_MS)),blink_cb,NULL);
  chSysUnlockFromISR();
}

static uint16_t read_keys(uint8_t tx_mode)
{
  uint16_t keys = 0;
  if(tx_mode == TX_IO){
    for(uint8_t i=0;i<9;i++){
      if(palReadPad(digital_in_tx[i].port,digital_in_tx[i].line) == PAL_LOW){
        keys |= (1 << i);
      }
    }
  }
  else{
    for(uint8_t i=0;i<9;i++){
      if(palReadPad(digital_in_tx_ana[i].port,digital_in_tx_ana[i].line) == PAL_LOW){
        keys |= (1 << i);
      }
    }
  }
  return keys;
}

static void process_ctrl_tx()
{
  uint8_t opMode = db_read_df_u8(OP_MODE);
  float fv = 0;
  
  uint8_t input_type = TYPE_OF_TX(opMode);

  if(input_type == TX_IO){
    uint16_t ch_sum[2] = {0,0};
    for(uint8_t i=0;i<ADC_GRP1_BUF_DEPTH;i++){
      ch_sum[0] += samples[i*2];
      ch_sum[1] += samples[i*2+1];
    }
    
    ch_sum[0] /= ADC_GRP1_BUF_DEPTH;
    ch_sum[1] /= ADC_GRP1_BUF_DEPTH;
    runTime.ain_value = ch_sum[0];
    
    // check if low battery
    /*
      lbt voltage         code
      5.0                 (5-1.2)/10.56*4096 = 1940
      4.8                 (4.8 - 1.2)*10.56/4096 = 1400
    */
    
    runTime.bat_mv = (ch_sum[1]*2*3300)>>12 ;
    runTime.bat_mv += 200;
    db_write_ld_u16(LIVE_DATA_BAT_MV,runTime.bat_mv);
    if(runTime.bat_mv < db_read_df_u16(TX_LBT_LOW_BOUND)){
      palSetLine(LINE_LBT_ACT);
    }
    else if(runTime.bat_mv > db_read_df_u16(TX_LBT_HIGH_BOUND)){
      palClearLine(LINE_LBT_ACT);
    }
    runTime.dio_state = read_keys(input_type);
    db_write_ld_u16(LIVE_DATA_DIO_STATE,runTime.dio_state);
    db_write_ld_u16(LIVE_DATA_AIN_CH1,runTime.ain_value);
    db_write_ld_u16(LIVE_DATA_AIN_CH2,runTime.bat_mv);
  }
  else if(input_type == TX_ADC){
    uint16_t ch_sum[5] = {0,0};
    for(uint8_t i=0;i<ADC_GRP1_BUF_DEPTH;i++){
      ch_sum[0] += samples[i*5];
      ch_sum[1] += samples[i*5+1];
      ch_sum[2] += samples[i*5+2];
      ch_sum[3] += samples[i*5+3];
      ch_sum[4] += samples[i*5+4];
    }
    
    ch_sum[0] /= ADC_GRP1_BUF_DEPTH;
    ch_sum[1] /= ADC_GRP1_BUF_DEPTH;
    ch_sum[2] /= ADC_GRP1_BUF_DEPTH;
    ch_sum[3] /= ADC_GRP1_BUF_DEPTH;
    ch_sum[4] /= ADC_GRP1_BUF_DEPTH;

    runTime.bat_mv = (ch_sum[0]*2*3300)>>12 ;
    runTime.bat_mv += 200;
    db_write_ld_u16(LIVE_DATA_BAT_MV,runTime.bat_mv);
    if(runTime.bat_mv < db_read_df_u16(TX_LBT_LOW_BOUND)){
      palSetLine(LINE_LBT_ACT);
    }
    else if(runTime.bat_mv > db_read_df_u16(TX_LBT_HIGH_BOUND)){
      palClearLine(LINE_LBT_ACT);
    }
    runTime.dio_state = read_keys(input_type);
    db_write_ld_u16(LIVE_DATA_DIO_STATE,runTime.dio_state);
    db_write_ld_u16(LIVE_DATA_AIN_CH1,ch_sum[1]);
    db_write_ld_u16(LIVE_DATA_AIN_CH2,ch_sum[2]);
    db_write_ld_u16(LIVE_DATA_AIN_CH3,ch_sum[3]);
    db_write_ld_u16(LIVE_DATA_AIN_CH4,ch_sum[4]);
  }
}

static void process_switch_tx()
{
  
}

static void process_rxv01(uint16_t value, int16_t adcv)
{
  uint16_t mask;
  pwmcnt_t width = 0;
  pwmcnt_t pwm_servo = 5000; // servo control, reversed
  uint8_t opMode = db_read_df_u8(OP_MODE);
  float fv = 0;
  
  uint8_t gear_type = TYPE_OF_GEAR(opMode);
  uint8_t valve_type = TYPE_OF_VALVE(opMode);
  
  uint16_t dio_mask = db_read_df_u16(DIO_ON_MASK);
  
  if((value & 0xff) == 0x00)
    value &= ~dio_mask;
  else
    value |= dio_mask;
  
//  if(value & 0x100){
//    palSetPad(digital_out_rx[8].port,digital_out_rx[8].line);
//  }
//  else{
//    palClearPad(digital_out_rx[8].port,digital_out_rx[8].line);
//  }
//  
  //value &= 0xff; // reserve active low 8-bit
  
  if(gear_type == GEAR_STEPPER || gear_type == GEAR_STEPPER_DAC){
    if(adcv > 500 &&  value != 0x0){
      //palSetPad(digital_out_rx[9].port,digital_out_rx[9].line);
      //value |= dio_mask;
      stepper_move_to(adcv);
    }
    else{
      //palClearPad(digital_out_rx[9].port,digital_out_rx[9].line);
      //value &= ~dio_mask;
      stepMoveHome();
    }
  }

  if((gear_type == GEAR_SERVO) || (gear_type == GEAR_DAC) || (gear_type == GEAR_PWM) || (gear_type == GEAR_STEPPER_DAC)){
    
    if(value != 0x00){
      value |= dio_mask;
      //palSetPad(digital_out_rx[9].port,digital_out_rx[9].line);
      if(adcv < db_read_df_u16(SERVO_MAP_RAW_PT_1)){
        pwm_servo = db_read_df_u16(SERVO_MAP_COUNTER_PT_1);
      }
      else if(adcv < db_read_df_u16(SERVO_MAP_RAW_PT_2)){
        fv = (db_read_df_u16(SERVO_MAP_COUNTER_PT_2) - db_read_df_u16(SERVO_MAP_COUNTER_PT_1));
        fv /= (db_read_df_u16(SERVO_MAP_RAW_PT_2) - db_read_df_u16(SERVO_MAP_RAW_PT_1));
        fv *= (adcv - db_read_df_u16(SERVO_MAP_RAW_PT_1));
        pwm_servo = (uint16_t)(fv + db_read_df_u16(SERVO_MAP_COUNTER_PT_1));
      }
      else if(adcv < db_read_df_u16(SERVO_MAP_RAW_PT_3)){
        fv = (db_read_df_u16(SERVO_MAP_COUNTER_PT_3) - db_read_df_u16(SERVO_MAP_COUNTER_PT_2));
        fv /= (db_read_df_u16(SERVO_MAP_RAW_PT_3) - db_read_df_u16(SERVO_MAP_RAW_PT_2));
        fv *= (adcv - db_read_df_u16(SERVO_MAP_RAW_PT_2));
        pwm_servo = (uint16_t)(fv + db_read_df_u16(SERVO_MAP_COUNTER_PT_2));
      }
      else if(adcv < db_read_df_u16(SERVO_MAP_RAW_PT_4)){
        fv = (db_read_df_u16(SERVO_MAP_COUNTER_PT_4) - db_read_df_u16(SERVO_MAP_COUNTER_PT_3));
        fv /= (db_read_df_u16(SERVO_MAP_RAW_PT_4) - db_read_df_u16(SERVO_MAP_RAW_PT_3));
        fv *= (adcv - db_read_df_u16(SERVO_MAP_RAW_PT_3));
        pwm_servo = (uint16_t)(fv + db_read_df_u16(SERVO_MAP_COUNTER_PT_3));
      }
      else{
        pwm_servo = db_read_df_u16(SERVO_MAP_COUNTER_PT_4);
      }
    }
    else{
      switch(gear_type){
      case GEAR_SERVO:pwm_servo = 150;break;
      case GEAR_DAC:pwm_servo = 0;break;
      case GEAR_PWM:pwm_servo = 0;break;
      case GEAR_STEPPER_DAC:pwm_servo = 0;break;
      default:break;
      }
//      if(opMode & RX_GEAR_SERVO)
//        pwm_servo = 150;
//      else
//        pwm_servo = 0;
//      palClearPad(digital_out_rx[9].port,digital_out_rx[9].line);
//      pwm_servo = PWM_PERIOD_COUNT;
    }
    if(pwm_servo < 150){      
//      if(opMode & RX_GEAR_SERVO)
      if(gear_type == GEAR_SERVO)
        pwm_servo = 150;
      else
        pwm_servo = 0;
      // set DAC Relay on
      //palClearPad(digital_out_rx[9].port,digital_out_rx[9].line);
      //value &= ~dio_mask;
    }
    else{
//      palSetPad(digital_out_rx[9].port,digital_out_rx[9].line);
      //value |= dio_mask;
    }
    
      switch(TYPE_OF_GEAR(opMode)){
      case GEAR_SERVO:
        pwm_servo = PWM_PERIOD_COUNT - pwm_servo;
        pwmEnableChannel(&PWMD2,3,pwm_servo);
        break;
      case GEAR_DAC:
      case GEAR_STEPPER_DAC:
        pwmEnableChannel(&PWMD2,3,pwm_servo);
        if(pwm_servo < 150)
          value &= ~0x200;
        else
          value |= 0x200;
        ;break;
      case GEAR_PWM:
        pwmEnableChannel(&PWMD3,4,pwm_servo);
        break;
      default:break;
      }
//    if(opMode & RX_GEAR_SERVO){
//      pwm_servo = PWM_PERIOD_COUNT - pwm_servo;
//      pwmEnableChannel(&PWMD2,3,pwm_servo);
//    }
//    else if(opMode & RX_GEAR_DAC){
//      pwmEnableChannel(&PWMD2,3,pwm_servo);
//    }
    
    db_write_ld_u32(LIVE_DATA_SERVO_CONTROL,pwm_servo);
  }

//  if(opMode & RX_VALVE_PWM){
  if(TYPE_OF_VALVE(opMode) == VALVE_PWM){
    if(value != 0x0){
      if(value & 0x100){
        palSetPad(digital_out_rx[8].port,digital_out_rx[8].line);
      }
      else{
        palClearPad(digital_out_rx[8].port,digital_out_rx[8].line);
      }
      if(value & 0x200){
        palSetPad(digital_out_rx[9].port,digital_out_rx[9].line);
      }
      else{
        palClearPad(digital_out_rx[9].port,digital_out_rx[9].line);
      }
    }
    else{
      palClearPad(digital_out_rx[8].port,digital_out_rx[8].line);
      palClearPad(digital_out_rx[9].port,digital_out_rx[9].line);
    }
    int8_t newDirection[4] = {0,0,0,0};
    for(uint8_t i=0;i<4;i++){
      mask = (1 << (i*2));
      if((mask & value) != 0){
        newDirection[i] = 1;
      }
      mask = (1 << (i*2+1));
      if((mask & value) != 0){
        newDirection[i] = -1;
      }
    }
  
    if(adcv < db_read_df_u16(PWM_MAP_RAW_PT_1)){
      width = db_read_df_u16(PWM_MAP_COUNTER_PT_1);
    }
    else if(adcv < db_read_df_u16(PWM_MAP_RAW_PT_2)){
      fv = (db_read_df_u16(PWM_MAP_COUNTER_PT_2) - db_read_df_u16(PWM_MAP_COUNTER_PT_1));
      fv /= (db_read_df_u16(PWM_MAP_RAW_PT_2) - db_read_df_u16(PWM_MAP_RAW_PT_1));
      fv *= (adcv - db_read_df_u16(PWM_MAP_RAW_PT_1));
      width = (uint16_t)(fv + db_read_df_u16(PWM_MAP_COUNTER_PT_1));
    }
    else if(adcv < db_read_df_u16(PWM_MAP_RAW_PT_3)){
      fv = (db_read_df_u16(PWM_MAP_COUNTER_PT_3) - db_read_df_u16(PWM_MAP_COUNTER_PT_2));
      fv /= (db_read_df_u16(PWM_MAP_RAW_PT_3) - db_read_df_u16(PWM_MAP_RAW_PT_2));
      fv *= (adcv - db_read_df_u16(PWM_MAP_RAW_PT_2));
      width = (uint16_t)(fv + db_read_df_u16(PWM_MAP_COUNTER_PT_2));
    }
    else if(adcv < db_read_df_u16(PWM_MAP_RAW_PT_4)){
      fv = (db_read_df_u16(PWM_MAP_COUNTER_PT_4) - db_read_df_u16(PWM_MAP_COUNTER_PT_3));
      fv /= (db_read_df_u16(PWM_MAP_RAW_PT_4) - db_read_df_u16(PWM_MAP_RAW_PT_3));
      fv *= (adcv - db_read_df_u16(PWM_MAP_RAW_PT_3));
      width = (uint16_t)(fv + db_read_df_u16(PWM_MAP_COUNTER_PT_3));
    }
    else{
      width = db_read_df_u16(PWM_MAP_COUNTER_PT_4);
    }
  
    db_write_ld_u32(LIVE_DATA_VALVE_CONTROL,width);

    for(uint8_t i=0;i<4;i++){
  //    if((newDirection[i] != 0) && (newDirection[i] != runTime.pwmDirection[i]) && (runTime.pwmDirection[i] != 0)){
  //      runTime.pwmDirection[i] = 0;
  //    }
  //    else {
  //      runTime.pwmDirection[i] = newDirection[i];
  //    }
      
      if(runTime.pwmDirection[i] == 0 || newDirection[i] == 0){
        runTime.pwmDirection[i] = newDirection[i];
      }
      else if(newDirection[i] != runTime.pwmDirection[i]){
        runTime.pwmDirection[i] = 0;
      }
    }
    
    if(runTime.pwmDirection[0] == 0){
      pwmEnableChannel(&PWMD3,0,PWM_VALVE_IDLE_OUTPUT);
      pwmEnableChannel(&PWMD3,1,PWM_VALVE_IDLE_OUTPUT);
    }
    else if(runTime.pwmDirection[0] == 1){
      pwmEnableChannel(&PWMD3,0,width);
    }
    else if(runTime.pwmDirection[0] == -1){
      pwmEnableChannel(&PWMD3,1,width);
    }
   
    if(runTime.pwmDirection[1] == 0){
      pwmEnableChannel(&PWMD3,2,PWM_VALVE_IDLE_OUTPUT);
      pwmEnableChannel(&PWMD3,3,PWM_VALVE_IDLE_OUTPUT);
    }
    else if(runTime.pwmDirection[1] == 1){
      pwmEnableChannel(&PWMD3,2,width);
    } 
    else if(runTime.pwmDirection[1] == -1){
      pwmEnableChannel(&PWMD3,3,width);
    }
   

    if(runTime.pwmDirection[2] == 0){
      pwmEnableChannel(&PWMD4,3,PWM_VALVE_IDLE_OUTPUT);
      pwmEnableChannel(&PWMD4,2,PWM_VALVE_IDLE_OUTPUT);
    }
    else if(runTime.pwmDirection[2] == 1){
      pwmEnableChannel(&PWMD4,3,width);
    }
    else if(runTime.pwmDirection[2] == -1){
      pwmEnableChannel(&PWMD4,2,width);
    }
   
    if(runTime.pwmDirection[3] == 0){
      pwmEnableChannel(&PWMD4,1,PWM_VALVE_IDLE_OUTPUT);
      pwmEnableChannel(&PWMD4,0,PWM_VALVE_IDLE_OUTPUT);
    }
    else if(runTime.pwmDirection[3] == 1){
      pwmEnableChannel(&PWMD4,1,width);
    } 
    else if(runTime.pwmDirection[3] == -1){
      pwmEnableChannel(&PWMD4,0,width);
    } 
  }
  else{
    for(uint8_t i=0;i<10;i++){
      mask = (1 << i);
      if((mask & value) == 0){
        palClearPad(digital_out_rx[i].port,digital_out_rx[i].line);
      }
      else{
        palSetPad(digital_out_rx[i].port,digital_out_rx[i].line);
      }
    }

  }
  



  
  
}

static void gpio_init()
{
  uint8_t i;
  uint8_t opMode = db_read_df_u8(OP_MODE);
  if((opMode & OP_MODE_MASK) == OP_MODE_TX){ // TX, 0x01 for switch, 0x02 for control
    uint8_t input_type = TYPE_OF_TX(opMode);
    palSetPadMode(GPIOA,0,PAL_MODE_INPUT);
    if(input_type == TX_IO){
      palSetPadMode(GPIOA,1,PAL_MODE_INPUT_ANALOG);
      palSetPadMode(GPIOA,2,PAL_MODE_INPUT_ANALOG);
      for(i=0;i<10;i++){
        palSetPadMode(digital_in_tx[i].port, digital_in_tx[i].line, PAL_MODE_INPUT_PULLUP);
      }
      
      palClearLine(LINE_TX_ACT);
      palClearLine(LINE_LBT_ACT);
      palSetPadMode(digital_out_tx[0].port, digital_out_tx[0].line, PAL_MODE_OUTPUT_OPENDRAIN);
      palSetPadMode(digital_out_tx[1].port, digital_out_tx[1].line, PAL_MODE_OUTPUT_PUSHPULL);
      palSetPadMode(digital_out_tx[2].port, digital_out_tx[2].line, PAL_MODE_OUTPUT_PUSHPULL);
    }
    else{
      palSetPadMode(GPIOA,1,PAL_MODE_INPUT_ANALOG);
      palSetPadMode(GPIOA,2,PAL_MODE_INPUT_ANALOG);
      palSetPadMode(GPIOA,6,PAL_MODE_INPUT_ANALOG);
      palSetPadMode(GPIOA,7,PAL_MODE_INPUT_ANALOG);
      palSetPadMode(GPIOB,0,PAL_MODE_INPUT_ANALOG);
      palSetPadMode(GPIOB,1,PAL_MODE_INPUT_ANALOG);
      
    }
  }
  
  else if((opMode & OP_MODE_MASK) == OP_MODE_RX){ // RX, bit 0: 0: switch, 1: control, bit 1: 0: stepper, 1: servo, bit 2: gpio/PWM
    uint8_t rx_mode = opMode & OP_MODE_SWITCH;
    uint8_t gear_type = TYPE_OF_GEAR(opMode);// & 0x02;
    uint8_t valve_type = TYPE_OF_VALVE(opMode);// & 0x04;
    
    if(rx_mode == OP_MODE_CONTROL){
//      if(((opMode & RX_GEAR_SERVO)== RX_GEAR_SERVO) || ((opMode & RX_GEAR_DAC)== RX_GEAR_DAC)){
      switch(gear_type){
      case GEAR_STEPPER:{
        palSetPadMode(GPIOC, 6, PAL_MODE_OUTPUT_OPENDRAIN);
        palSetPadMode(GPIOC, 7, PAL_MODE_OUTPUT_OPENDRAIN);
        palSetPadMode(GPIOC, 8, PAL_MODE_OUTPUT_OPENDRAIN);
      }break;
      case GEAR_SERVO:{
        // PA3 for PWM servo control
        palSetPadMode(GPIOA, 3, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
        //palSetPadMode(GPIOC, 6, PAL_MODE_STM32_ALTERNATE_PUSHPULL);// TIM8.CH1
      }break;
      case GEAR_DAC:{
        // PA3 for PWM servo control
        palSetPadMode(GPIOA, 3, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
        palSetPadMode(GPIOC, 6, PAL_MODE_STM32_ALTERNATE_PUSHPULL);// TIM8.CH1
      }break;
      case GEAR_PWM:{
        
      }break;
      case GEAR_STEPPER_DAC:{
        // PA3 for PWM servo control
        palSetPadMode(GPIOA, 3, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
        //palSetPadMode(GPIOC, 6, PAL_MODE_STM32_ALTERNATE_PUSHPULL);// TIM8.CH1
        // setpper control pins
        palSetPadMode(GPIOC, 6, PAL_MODE_OUTPUT_OPENDRAIN);
        palSetPadMode(GPIOC, 7, PAL_MODE_OUTPUT_OPENDRAIN);
        palSetPadMode(GPIOC, 8, PAL_MODE_OUTPUT_OPENDRAIN);
        
      }break;
      }
//      if((gear_type== GEAR_SERVO) || (gear_type== GEAR_DAC)){
//        // PA3 for PWM servo control
//        palSetPadMode(GPIOA, 3, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
//        palSetPadMode(GPIOC, 6, PAL_MODE_STM32_ALTERNATE_PUSHPULL);// TIM8.CH1
//      }
////      else if(gear_type == GEAR_PWM){
////        palSetPadMode(GPIOC, 9, PAL_MODE_STM32_ALTERNATE_PUSHPULL);// TIM3.4
////      }
//      else{ // stepper
//        palSetPadMode(GPIOC, 6, PAL_MODE_OUTPUT_OPENDRAIN);
//        palSetPadMode(GPIOC, 7, PAL_MODE_OUTPUT_OPENDRAIN);
//        palSetPadMode(GPIOC, 8, PAL_MODE_OUTPUT_OPENDRAIN);
//      }
      
      if(valve_type == VALVE_PWM){
        palSetPadMode(digital_out_rx[0].port, digital_out_rx[0].line, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
        palSetPadMode(digital_out_rx[1].port, digital_out_rx[1].line, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
        palSetPadMode(digital_out_rx[2].port, digital_out_rx[2].line, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
        palSetPadMode(digital_out_rx[3].port, digital_out_rx[3].line, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
        palSetPadMode(digital_out_rx[4].port, digital_out_rx[4].line, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
        palSetPadMode(digital_out_rx[5].port, digital_out_rx[5].line, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
        palSetPadMode(digital_out_rx[6].port, digital_out_rx[6].line, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
        palSetPadMode(digital_out_rx[7].port, digital_out_rx[7].line, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
        palSetPadMode(digital_out_rx[8].port, digital_out_rx[8].line, PAL_MODE_OUTPUT_PUSHPULL);
        if(gear_type == GEAR_PWM){
          palSetPadMode(GPIOC, 9, PAL_MODE_STM32_ALTERNATE_PUSHPULL);// TIM3.4
        }
        else{
          palSetPadMode(digital_out_rx[9].port, digital_out_rx[9].line, PAL_MODE_OUTPUT_PUSHPULL);
        }
      }
      else{
        palSetPadMode(digital_out_rx[0].port, digital_out_rx[0].line, PAL_MODE_OUTPUT_PUSHPULL);
        palSetPadMode(digital_out_rx[1].port, digital_out_rx[1].line, PAL_MODE_OUTPUT_PUSHPULL);
        palSetPadMode(digital_out_rx[2].port, digital_out_rx[2].line, PAL_MODE_OUTPUT_PUSHPULL);
        palSetPadMode(digital_out_rx[3].port, digital_out_rx[3].line, PAL_MODE_OUTPUT_PUSHPULL);
        palSetPadMode(digital_out_rx[4].port, digital_out_rx[4].line, PAL_MODE_OUTPUT_PUSHPULL);
        palSetPadMode(digital_out_rx[5].port, digital_out_rx[5].line, PAL_MODE_OUTPUT_PUSHPULL);
        palSetPadMode(digital_out_rx[6].port, digital_out_rx[6].line, PAL_MODE_OUTPUT_PUSHPULL);
        palSetPadMode(digital_out_rx[7].port, digital_out_rx[7].line, PAL_MODE_OUTPUT_PUSHPULL);
        palSetPadMode(digital_out_rx[8].port, digital_out_rx[8].line, PAL_MODE_OUTPUT_PUSHPULL);
        palSetPadMode(digital_out_rx[9].port, digital_out_rx[9].line, PAL_MODE_OUTPUT_PUSHPULL);
      }
    }
    else{
      palSetPadMode(GPIOC, 6, PAL_MODE_OUTPUT_OPENDRAIN);
    }
    
    
    
  }

}

static THD_WORKING_AREA(waRemoteIO,512);
static THD_FUNCTION(procRemoteIO ,p)
{
  uint8_t opMode = db_read_df_u8(OP_MODE);
  if((opMode & OP_MODE_MASK) == OP_MODE_TX){
    tx_control_loop();
//    if((runTime.opMode & OP_TYPE_MASK) == OP_TYPE_SWITCH){
//      tx_control_loop();
//    }
//    else if((runTime.opMode & OP_MODE_MASK) == OP_TYPE_CONNTROL){
//      tx_control_loop();
//    }
  }
  else if((opMode & OP_MODE_MASK) == OP_MODE_RX){
    rx_control_loop();      
//    if(runTime.opMode & RX_MODE_CONTROL){
//      rx_control_loop();      
//    }
//    else{
//      rx_control_loop();
//    }
  }
}

static THD_WORKING_AREA(waOperation,512);
static THD_FUNCTION(procOperation ,p)
{
  bool bStop = false;
  struct{
    uint16_t packet_id;
    uint16_t start_address;
    uint8_t buf[64];
  }data;
  data.packet_id = 0;
  data.start_address = LIVE_DATA_RSSI;
  uint8_t *ptr = data.buf;
  while(!bStop)
  {
    runTime.dio_state = db_read_ld_u16(LIVE_DATA_DIO_STATE);
    runTime.ain_value = db_read_ld_u16(LIVE_DATA_AIN_CH1);
    process_rxv01(runTime.dio_state,runTime.ain_value);
    chThdSleepMilliseconds(50);
  }
}

void start_rx_process()
{
  uint8_t opMode = db_read_df_u8(OP_MODE);
  if(!runTime.rxThread){
    uint8_t gear_type = TYPE_OF_GEAR(opMode);
    uint8_t valve_type = TYPE_OF_VALVE(opMode);
    if(valve_type == VALVE_PWM){
      pwmStart(&PWMD3,&pwm_config_3);
      pwmStart(&PWMD4,&pwm_config_4);
      runTime.pwmDirection[0] = 0;
      runTime.pwmDirection[1] = 0;
      runTime.pwmDirection[2] = 0;
      runTime.pwmDirection[3] = 0;
    }
    
    if(gear_type == GEAR_SERVO){
      pwmStart(&PWMD2,&pwm_config_2);
    }
    else if(gear_type == GEAR_DAC){
      pwmStart(&PWMD2,&pwm_config_2);
    }
    else if(gear_type == GEAR_PWM){
      pwmStart(&PWMD3,&pwm_config_3);
    }
    else if(gear_type == GEAR_STEPPER_DAC){
      pwmStart(&PWMD2,&pwm_config_2);
      for(uint8_t i=0;i<4;i++){
        runTime.stepper.stepConv[i].vr = db_read_df_u16(VR_RAW_PT_1 + i);
        runTime.stepper.stepConv[i].ang = db_read_df_u16(VR_ANGLE_PT_1 + i);
        runTime.stepper.angSpeed[i].ang = db_read_df_i16(SPEED_MAP_ANGLE_1+i);
        runTime.stepper.angSpeed[i].speed_low = db_read_df_i16(SPEED_MAP_LOW_1+i);
        runTime.stepper.angSpeed[i].speed_high = db_read_df_i16(SPEED_MAP_HIGH_1+i);
      }
      runTime.stepper.pulsePerDegree = db_read_df_f32(MOTOR_MAP_PPD);
      runTime.stepper.min_pps = db_read_df_u16(STEPPER_MIN_PPS);
      runTime.stepper.max_pps = db_read_df_u16(STEPPER_MAX_PPS);
      stepper_task_init((void*)&runTime.stepper);
    }
    else{
      for(uint8_t i=0;i<4;i++){
        runTime.stepper.stepConv[i].vr = db_read_df_u16(VR_RAW_PT_1 + i);
        runTime.stepper.stepConv[i].ang = db_read_df_u16(VR_ANGLE_PT_1 + i);
        runTime.stepper.angSpeed[i].ang = db_read_df_i16(SPEED_MAP_ANGLE_1+i);
        runTime.stepper.angSpeed[i].speed_low = db_read_df_i16(SPEED_MAP_LOW_1+i);
        runTime.stepper.angSpeed[i].speed_high = db_read_df_i16(SPEED_MAP_HIGH_1+i);
      }
      runTime.stepper.pulsePerDegree = db_read_df_f32(MOTOR_MAP_PPD);
      runTime.stepper.min_pps = db_read_df_u16(STEPPER_MIN_PPS);
      runTime.stepper.max_pps = db_read_df_u16(STEPPER_MAX_PPS);
      stepper_task_init((void*)&runTime.stepper);
    }
    runTime.rxThread = chThdCreateStatic(waOperation,sizeof(waOperation),NORMALPRIO-1,procOperation,NULL);
  }
}

void stop_rx_process()
{
  if(runTime.rxThread){
    uint8_t opMode = db_read_df_u8(OP_MODE);
    chThdTerminate(runTime.opThread);
    chThdWait(runTime.opThread);
    runTime.opThread = NULL;
    //chVTReset(&runTime.vt);
    if(opMode & OP_MODE_RX){
      uint8_t gear_type = TYPE_OF_GEAR(opMode);
      uint8_t valve_type = (TYPE_OF_VALVE(opMode));
                            
      if(valve_type == VALVE_PWM){
        pwmStop(&PWMD3);
        pwmStop(&PWMD4);
      }
      
      if(gear_type == GEAR_SERVO || gear_type == GEAR_DAC ){
        pwmStop(&PWMD2);
      }
      else if(gear_type == GEAR_PWM){
        pwmStop(&PWMD3);
      }
    }
  }
}


static THD_WORKING_AREA(waTransfer,512);
static THD_FUNCTION(procTransfer ,p)
{
  bool bStop = false;
  struct{
    uint16_t packet_id;
    uint16_t start_address;
    uint8_t buf[64];
  }data;
  data.packet_id = 0;
  data.start_address = LIVE_DATA_RSSI;
  uint8_t *ptr = data.buf;
  while(!bStop)
  {
    ptr = data.buf;
    ptr += db_read_livedata(0,0xff,LIVE_DATA_RSSI,ptr);
    ptr += db_read_livedata(0,0xff,LIVE_DATA_DIO_STATE,ptr);
    ptr += db_read_livedata(0,0xff,LIVE_DATA_AIN_CH1,ptr);
    ptr += db_read_livedata(0,0xff,LIVE_DATA_AIN_CH2,ptr);
    ptr += db_read_livedata(0,0xff,LIVE_DATA_AIN_CH3,ptr);
    ptr += db_read_livedata(0,0xff,LIVE_DATA_AIN_CH4,ptr);
    ptr += db_read_livedata(0,0xff,LIVE_DATA_VALVE_CONTROL,ptr);
    ptr += db_read_livedata(0,0xff,LIVE_DATA_SERVO_CONTROL,ptr);
    ptr += db_read_livedata(0,0xff,LIVE_DATA_STEPPER,ptr);
    
    send_packet((uint8_t*)&data,68);
    data.packet_id++;
    bStop = chThdShouldTerminateX();
    if(bStop){
      
    }
    chThdSleepMilliseconds(100);
  }
}

void start_transfer()
{
  if(!runTime.opThread){
    runTime.opThread = chThdCreateStatic(waTransfer,sizeof(waTransfer),NORMALPRIO-1,procTransfer,NULL);
  }
}

void stop_transfer()
{
  if(runTime.opThread){
    chThdTerminate(runTime.opThread);
    chThdWait(runTime.opThread);
    runTime.opThread = NULL;
    //chVTReset(&runTime.vt);
  }
}


int main()
{
  halInit();
  chSysInit();
  AFIO->MAPR |= (AFIO_MAP_SWJTAG_CONF_JTAGDISABLE);
  
  // remap PA2/3 to TIM5
  //sAFIO->MAP4 |= AFIO_MAP4_TIM2_GRMP_11;
  // config PC.9 as input PUD and check if shorted
  palSetPadMode(GPIOC, 9, PAL_MODE_INPUT_PULLUP);

  bool bStop = false;
  if((palReadPad(GPIOC,9) == PAL_LOW)){
    runTime.userMode = true;
  }
  else{
    runTime.userMode = false;
  }

  
  database_init();

  load_settings();
//  uint8_t opMode = db_read_df_u8(OP_MODE);

  // 2 TX mode
//  opMode = OP_MODE_TX | OP_MODE_CONTROL | TX_TYPE(TX_IO); //OK
//  opMode = OP_MODE_TX | OP_MODE_CONTROL | TX_TYPE(TX_ADC);
//  
//  opMode = OP_MODE_RX | OP_MODE_CONTROL | GEAR_TYPE(GEAR_STEPPER) | VALVE_TYPE(VALVE_IO); //OK
//  opMode = OP_MODE_RX | OP_MODE_CONTROL | GEAR_TYPE(GEAR_STEPPER) | VALVE_TYPE(VALVE_PWM);
//
//  opMode = OP_MODE_RX | OP_MODE_CONTROL | GEAR_TYPE(GEAR_SERVO) | VALVE_TYPE(VALVE_IO); // OK
//  opMode = OP_MODE_RX | OP_MODE_CONTROL | GEAR_TYPE(GEAR_DAC) | VALVE_TYPE(VALVE_IO); //OK
//  opMode = OP_MODE_RX | OP_MODE_CONTROL | GEAR_TYPE(GEAR_PWM) | VALVE_TYPE(VALVE_IO);
//  opMode = OP_MODE_RX | OP_MODE_CONTROL | GEAR_TYPE(GEAR_STEPPER_DAC) | VALVE_TYPE(VALVE_IO);//OK
//
//  opMode = OP_MODE_RX | OP_MODE_CONTROL | GEAR_TYPE(GEAR_SERVO) | VALVE_TYPE(VALVE_PWM);//ok
//  opMode = OP_MODE_RX | OP_MODE_CONTROL | GEAR_TYPE(GEAR_DAC) | VALVE_TYPE(VALVE_PWM); //ok
//  opMode = OP_MODE_RX | OP_MODE_CONTROL | GEAR_TYPE(GEAR_PWM) | VALVE_TYPE(VALVE_PWM);
//  opMode = OP_MODE_RX | OP_MODE_CONTROL | GEAR_TYPE(GEAR_STEPPER_DAC) | VALVE_TYPE(VALVE_PWM);//ok

//  db_write_df_u8(OP_MODE,opMode);
  
//  db_write_df_u16(SERVO_MAP_RAW_PT_1,200);
//  db_write_df_u16(SERVO_MAP_RAW_PT_2,3300);
//  db_write_df_u16(SERVO_MAP_RAW_PT_3,3800);
//  db_write_df_u16(SERVO_MAP_RAW_PT_4,4000);
//  db_write_df_u16(SERVO_MAP_COUNTER_PT_1,1000);
//  db_write_df_u16(SERVO_MAP_COUNTER_PT_2,2000);
//  db_write_df_u16(SERVO_MAP_COUNTER_PT_3,3000);
//  db_write_df_u16(SERVO_MAP_COUNTER_PT_4,5000);

  db_write_df_u16(DIO_ON_MASK,0x0100);
  gpio_init();
  binaryProtocolInit();
  //rf_task_init();
    
  //if((runTime.opMode & OP_MODE_TX) || (runTime.opMode & OP_MODE_RX)){
    thread_t *rftp = chThdCreateStatic(waRemoteIO,sizeof(waRemoteIO),NORMALPRIO,procRemoteIO,NULL);
    chSysLock();
    chThdSuspendS(&runTime.ref);
    chSysUnlock();
  //}
   
  systime_t rxFailStart = chVTGetSystemTimeX();
  systime_t rxErrorPeriod = 0;
  if(opMode & OP_MODE_RX){
    start_rx_process();  
  }
  
  if(opMode & OP_MODE_TX){
    VR_POWER_EN();
  }

  while(!bStop)
  {
    eventmask_t evt = chEvtWaitAnyTimeout(ALL_EVENTS,TIME_MS2I(50));
    
     if(evt & EVENT_MASK(CMD_SAVE_NVM)){
        db_save_section(0xff);
     }
    
    if((opMode & OP_MODE_MASK) == OP_MODE_TX){
      process_ctrl_tx();
      if(evt & EV_TX_DONE){
        // enter sleep mode
        if(runTime.userMode == 0){
          bStop = true;
          palClearLine(LINE_TX_ACT);
          palClearLine(LINE_LBT_ACT);
        }
      }
    }
    
    if(opMode & OP_MODE_RX){
      if(evt & EV_RX_PACKET){
        rxFailStart = chVTGetSystemTimeX();
      }
      
      if(evt & EV_RX_ERROR){
        rxErrorPeriod = TIME_I2MS(chVTTimeElapsedSinceX(rxFailStart));
        if(rxErrorPeriod > 1000){
          if(db_read_ld_u8(LIVE_DATA_USER_CONTROL) == 0){
            db_write_ld_u16(LIVE_DATA_DIO_STATE,0x0);
            db_write_ld_u16(LIVE_DATA_AIN_CH1,0x0);
          }
        }
      }
      
      if(evt & EV_RX_LOST){
        rxErrorPeriod = TIME_I2MS(chVTTimeElapsedSinceX(rxFailStart));
        if(rxErrorPeriod > 1000){
          if(db_read_ld_u8(LIVE_DATA_USER_CONTROL) == 0){
            db_write_ld_u16(LIVE_DATA_DIO_STATE,0x0);
            db_write_ld_u16(LIVE_DATA_AIN_CH1,0x0);
          }
        }
      }
      
      if(evt & EVENT_MASK(CMD_START)){
        start_transfer();
      }
      if(evt & EVENT_MASK(CMD_STOP)){
        stop_transfer();
      }
    }
    
    if(chThdShouldTerminateX()){
      bStop = true;      
    }
  }
  
  if(opMode & OP_MODE_RX){
    stop_rx_process();
  }
  
  if(opMode & OP_MODE_TX){
    VR_POWER_DIS();
    // enter sleep mode
    /*
      config exti PA0 falling edge trigger to wakeup 
    */
    EXTI->PR = EXTI_INTEN_LN0;
    EXTI->IMR = EXTI_INTEN_LN0;
    EXTI->EMR = EXTI_EVTEN_LN0;
    EXTI->FTSR = EXTI_FTRSEL_LN0;
    EXTI->RTSR = EXTI_FTRSEL_LN0;
    AFIO->EXTICR[0] &= 0x0FFF;
    AFIO->EXTICR[0] |= AFIO_EXTIC1_EXTINT0_PTA;

    PWR->CR |= PWR_CR_CLWUF | PWR_CR_CLSBF;
    PWR->CR &= ~PWR_CR_PVDS;

    PWR->CR |= (PWR_CR_PDDS); //Standby(0), Sleep(1)
    PWR->CSR |= PWR_CRSTS_WUPEN;
    SCB->SCR |= (SCB_SCR_SLEEPDEEP_Msk);

    chSysDisable();
    __disable_irq();

    __SEV();
    __WFE();  
    __WFE();  

    // wakeup... 
    __enable_irq();
    NVIC_SystemReset(); 
  }
  

  return 0;
  
}
static void tx_switch_loop()
{
  
}

static void tx_control_loop()
{
  uint8_t opMode = db_read_df_u8(OP_MODE);
  float fv = 0;
  uint8_t rx[64];
  uint8_t input_type = TYPE_OF_TX(opMode);

  BC3601Driver *dev = &bc3601;
  //load_settings();
  runTime.mainThread = chRegFindThreadByName("main");
  adcStart(&ADCD1,NULL);
  if(input_type == TX_IO){
    adcStartConversion(&ADCD1,&adcgrpcfg,samples,ADC_GRP1_BUF_DEPTH);  
  }
  else if(input_type == TX_ADC){
    adcStartConversion(&ADCD1,&adcgrpcfg_tx2,samples_tx2,ADC_GRP1_BUF_DEPTH);  
  }
  ADCD1.adc->CR2 |= 0x0;

//  bc3601.txPower = nvmParam2.device_config.txPower;
//  bc3601.dataRate = nvmParam2.device_config.dataRate;
//  bc3601.frequency = (float)(nvmParam2.device_config.frequency + (nvmParam2.device_config.selfAddr%32)*0.5);
//  bc3601.destAddr = nvmParam2.device_config.destAddr;
  bc3601.txPower = db_read_df_u8(RF_TX_POWER);
  bc3601.dataRate = db_read_df_u8(RF_DATA_RATE_CODE);
  bc3601.dataRate = 2; 
  bc3601.frequency = db_read_df_f32(RF_BASE_FREQUENCY)+ (db_read_df_u16(DEVICE_ADDR)%32)*0.5;
  bc3601.destAddr = db_read_df_u16(DEST_ADDR);
  
  
  dev_bc3601Init(&bc3601,&config);
  irq_config(dev,IRQ2_TXCMPIE);
  
  _rfPacket *rfp = (_rfPacket*)(&txPacket.data[0]);
  //rfp.dio = SWITCH_TX_PATTERN;
  rfp->dio = db_read_df_u16(TX_DIO_MASK);
  rfp->advalue[0] = rfp->advalue[1] = rfp->advalue[2] = rfp->advalue[3] = 0x0;
  rfp->srcAddr = db_read_df_u16(DEVICE_ADDR);
  rfp->dstAddr = db_read_df_u16(DEST_ADDR);
  rfp->checksum = checksum((uint8_t*)&rfp,sizeof(_rfPacket)-1);
  uint8_t reg;
  uint8_t pktSz = sizeof(_rfPacket);
  //uint16_t cycles = 0;
  txPacket.sz = pktSz+1;
  //memcpy(txPacket.data,(uint8_t*)&rfp, pktSz);
  bool bStop = false;
  systime_t idleStart = chVTGetSystemTimeX();
  systime_t prevTxStamp = chVTGetSystemTimeX();
  systime_t idleMs = 0;

  chVTObjectInit(&vt_blink);
  chVTSet(&vt_blink,TIME_MS2I(500), blink_cb,NULL);
  
  idleStart = chVTGetSystemTimeX();
  chThdResume(&runTime.ref,MSG_OK);
  runTime.stage = 0;
  while(!bStop){
    systime_t now = chVTGetSystemTimeX();
    idleMs = TIME_I2MS(chVTTimeElapsedSinceX(idleStart));
    if((idleMs > db_read_df_u16(TIMEOUT_MS)) && (runTime.userMode == 0)){
      bStop = true;
    }

    // read analog input and i/o state
    if((opMode & CONTROL_MODE_MASK) == OP_MODE_CONTROL){
      if(db_read_ld_u16(LIVE_DATA_DIO_STATE) == 0x0){
        systime_t time = now;
        idleMs = TIME_I2MS(chVTTimeElapsedSinceX(idleStart));
      }
      else{
        idleStart = now;
      }
    }
    
    
    rfp->dio = db_read_ld_u16(LIVE_DATA_DIO_STATE);
    rfp->advalue[0] = db_read_ld_u16(LIVE_DATA_AIN_CH1);
    rfp->advalue[1] = db_read_ld_u16(LIVE_DATA_AIN_CH2);
    rfp->advalue[2] = db_read_ld_u16(LIVE_DATA_AIN_CH3);
    rfp->advalue[3] = db_read_ld_u16(LIVE_DATA_AIN_CH4);
    rfp->checksum = checksum((uint8_t*)rfp,sizeof(_rfPacket)-1);

    switch(runTime.stage){
    case 0:
      BC3601_RESET_TXFIFO(dev);
      reg = 0;
      irq_config(dev,IRQ2_TXCMPIE);
      BC3601_SET_TX_PAYLOAD_SADDR(dev,&reg);
      //BC3601_SET_TX_PAYLOAD_WIDTH(dev,&pktSz);
      BC3601_FIFO_WRITE(dev,(uint8_t*)&txPacket,txPacket.sz);
      BC3601_TX_MODE(dev);
      runTime.stage = 1;
      prevTxStamp = now;
      break;
    case 1:
      reg = bc3601_irqState(&bc3601);
      if(reg & (IRQ3_TXCMPIF)){
        BC3601_LITE_SLEEP(&bc3601);
//        bc3601_WaitCrystalReady(&bc3601);
        irq_config(dev,IRQ2_RXCMPIE);
        //BC3601_RESET_RXFIFO(dev);
        BC3601_RX_MODE(dev);
        runTime.stage = 3;
      }
      if(TIME_I2MS(chVTTimeElapsedSinceX(prevTxStamp)) >= db_read_df_u8(TX_INTERVAL_MS)){
        runTime.stage = 0;
      }
      break;
    case 2: // switch to rx mode
      irq_config(dev,IRQ2_RXCMPIE);
      BC3601_RESET_RXFIFO(dev);
      BC3601_RX_MODE(dev);
      runTime.stage = 3;
      break;
    default:
      reg = bc3601_irqState(&bc3601);
      if(reg & (IRQ3_RXCMPIF)){
        // read RX data
        BC3601_REG_READ(&bc3601,RX_DATA_LENG_REGS,&reg);    
        if(reg > 0){
          BC3601_FIFO_READ(&bc3601,(void*)&rxPacket.sz,reg);
        }
      }
      if(TIME_I2MS(chVTTimeElapsedSinceX(prevTxStamp)) >= db_read_df_u8(TX_INTERVAL_MS)){
        BC3601_LITE_SLEEP(&bc3601);
        runTime.stage = 0;
      }
    break;
    }
    
    
//    chThdSleepMilliseconds(db_read_df_u8(TX_INTERVAL_MS));
    chThdSleepMilliseconds(5);
    if(chThdShouldTerminateX() && (runTime.userMode == 0)){
      bStop = true;
    }
    //process_ctrl_tx();
  }
  dev_bc3601DeInit(&bc3601);
//  BC3601_STBY(dev);
//  BC3601_DEEP_SLEEP(dev);
  chVTReset(&vt_blink);
  chEvtSignal(runTime.mainThread,EV_TX_DONE);
  chThdExit(0);
 
}

static void rx_control_loop()
{
  BC3601Driver *dev = &bc3601;
  runTime.mainThread = chRegFindThreadByName("main");

  bc3601.txPower = db_read_df_u8(RF_TX_POWER);
  bc3601.dataRate = db_read_df_u8(RF_DATA_RATE_CODE);
  bc3601.dataRate = 2; 
  bc3601.frequency = db_read_df_f32(RF_BASE_FREQUENCY)+ (db_read_df_u16(DEVICE_ADDR)%32)*0.5;
  bc3601.destAddr = db_read_df_u16(DEST_ADDR);
  
  
  dev_bc3601Init(&bc3601,&config);
  _rfPacket *rfp = (_rfPacket*)(&txPacket.data[0]);
  uint8_t reg;
  uint8_t pktSz = sizeof(_rfPacket);
  uint16_t cycles = 0;
  txPacket.sz = pktSz+1;
  //memcpy(txPacket.data,(uint8_t*)&rfp, pktSz);
  bool bStop = false;
  systime_t idleStart = chVTGetSystemTimeX();
  systime_t idleMs = 0;

//  chVTObjectInit(&vt_blink);
//  chVTSet(&vt_blink,TIME_MS2I(500), blink_cb,NULL);
    runTime.stage = 0;
    uint8_t rx[64];
    
    chThdResume(&runTime.ref,MSG_OK);
    while(!bStop){
      systime_t time = chVTGetSystemTimeX();
      
      switch(runTime.stage){
        case 0:
          BC3601_LITE_SLEEP(&bc3601);
          bc3601_WaitCrystalReady(&bc3601);
          BC3601_RESET_RXFIFO(dev);
          BC3601_RX_MODE(dev);
          irq_config(dev,IRQ2_RXCMPIE);
          runTime.stage++;
          runTime.rxLost = 0;
          idleStart = chVTGetSystemTimeX();
          break;
        case 1:
          reg = bc3601_irqState(&bc3601);
          if(reg & (IRQ3_RXCMPIF | IRQ3_RXERRIF)){
            db_write_ld_u8(LIVE_DATA_RSSI,bc3601_readRSSI(&bc3601));
            if(reg & IRQ3_RXERRIF){
              chEvtSignal(runTime.mainThread, EV_RX_ERROR);
              runTime.stage = 0;
            }
            else{
              BC3601_REG_READ(&bc3601,RX_DATA_LENG_REGS,&reg);    
              if(reg > 0){
                BC3601_FIFO_READ(&bc3601,rx,reg);
                //BC3601_RESET_RXFIFO(dev);
                //BC3601_RX_MODE(dev);
                _txPacket *r = (_txPacket*)rx;
                r->sz--;
                if(r->sz == sizeof(_rfPacket)){
                  rfp = (_rfPacket*)r->data;
                  if(checksum((uint8_t*)rfp,sizeof(_rfPacket)-1) == rfp->checksum){
                    if(rfp->dstAddr == db_read_df_u16(DEVICE_ADDR)){
                      runTime.rf_ctrl = rfp->dio;
                      runTime.cycles = 0;
                      runTime.rxLost = 0;
                      db_write_ld_u16(LIVE_DATA_DIO_STATE,rfp->dio);
                      db_write_ld_u16(LIVE_DATA_AIN_CH1,rfp->advalue[0]);
                      chEvtSignal(runTime.mainThread, EV_RX_PACKET);
                      runTime.rfLinkFail = 0;
                    }
                  }
                  else{
                    chEvtSignal(runTime.mainThread, EV_RX_ERROR);      
                  }
                }
                runTime.stage = 0;
              }
              else{
                idleMs = TIME_I2MS(chVTTimeElapsedSinceX(idleStart));
                if(idleMs > 500){
                  chEvtSignal(runTime.mainThread, EV_RX_LOST);
                  runTime.stage = 0;
                  //BC3601_RESET_RXFIFO(dev);
                  //BC3601_RX_MODE(dev);
                }
              }
            }
            //runTime.stage = 0;
          }    
          else{
            idleMs = TIME_I2MS(chVTTimeElapsedSinceX(idleStart));
            if(idleMs > 500){
              chEvtSignal(runTime.mainThread, EV_RX_LOST);
              runTime.stage = 0;
            }
          }
          break;
      }
      chThdSleepMilliseconds(10);
  }
  dev_bc3601DeInit(&bc3601);
  chVTReset(&vt_blink);
  chEvtSignal(runTime.mainThread,EV_TX_DONE);
  chThdExit(0);
}
