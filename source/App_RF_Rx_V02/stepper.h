#ifndef _STEPPER_H
#define _STEPPER_H

#define DIR_POS 1
#define DIR_NEG -1

#define STEP_PULSE_BASE_CLOCK   100000
#define STEP_PULSE_WIDTH_US     200
#define STEP_PULSE_WIDTH_CNTR   STEP_PULSE_WIDTH_US*STEP_PULSE_BASE_CLOCK/1000000

#define STEP_CONTROL_BASE_CLOCK 100000
#define STEP_CONTROL_CYCLE_US   1000
#define STEP_CONTROL_CYCLE_CNTR STEP_CONTROL_CYCLE_US*STEP_CONTROL_BASE_CLOCK/1000000

#define STEP_SPEED_MAX          1000
#define STEP_SPEED_MIN          500
#define STEP_SPEED_MAX_CNTR     STEP_CONTROL_BASE_CLOCK/STEP_SPEED_MIN
#define STEP_SPEED_MIN_CNTR     STEP_CONTROL_BASE_CLOCK/STEP_SPEED_MAX
#define STEP_RELOAD_SIZE        10
#define STEP_ACCEL_CNTR         ((STEP_SPEED_MAX_CNTR - STEP_SPEED_MIN_CNTR)/STEP_RELOAD_SIZE+1)


struct _adc_step_conv_s{
  uint16_t vr;
  uint16_t ang;
};

struct _ang_speed_map_s{
  int16_t ang;
  int16_t speed_low;
  int16_t speed_high;
};

struct _stepper_control_s{
  struct _adc_step_conv_s stepConv[4];
  struct _ang_speed_map_s angSpeed[4];
  float pulsePerDegree;
  uint16_t max_pps;
  uint16_t min_pps;
};

typedef struct{
  void (*stepLow)(void);
  void (*stepHigh)(void);
  void (*dirLow)(void);
  void (*dirHigh)(void);
  void (*stepEn)(void);
  void (*stepDis)(void);
  struct _stepper_control_s stepperCtrl;
//  struct _adc_step_conv_s stepConv[4];
//  struct _ang_speed_map_s angSpeed[3];  
}_stepper_ctrl_t;


void stepper_init(void *arg);
void stepper_move_to(int16_t pos);
void stepper_move(int32_t steps);
void stepMoveHome();
int32_t stepPoll(void);
#endif