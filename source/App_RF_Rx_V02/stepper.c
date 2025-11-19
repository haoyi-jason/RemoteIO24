#include "ch.h"
#include "hal.h"
#include "stepper.h"


// timebase 100 us
#define STEP_PATTEN     0x1 // ____|
#define ACC_STEP_MAX    0x8
#define DCC_STEP_MAX    0x8
#define STEPS_PER_REV   200*10

#define TIM0_CLK_BASE   1000000

enum step_state_e{
  SS_STOPPED,
  SS_ACCEL,
  SS_NORMAL,
  SS_DEACCEL
};

enum pulse_state_e{
  PS_IDLE,
  PS_BLANK,
  PS_HIGH,
  PS_DELAY_HIGH,
  PS_LOW,
  PS_DELAY_LOW
};

struct pps_map_s{
  uint16_t baseReload;
  float slop;
  uint16_t basePos;
} ppsMap[3];

struct runTime_s{
  //int32_t currPos;
  int32_t tarPos;
//  int16_t stepLeft;
//  int16_t stepMoved;
//  int16_t stepToMove;
  int8_t direction;
  int16_t stepState;
  //int16_t stepDelay;
  //int16_t pulseCount;
  //int16_t stepPulse;
  //int16_t accSteps;
  //int16_t daccSteps;
  //int16_t pulseState;
  //int16_t stepBuffer[4];
  //int16_t stepIndex;
  int8_t moveHome;
  float ppd;
  _stepper_ctrl_t *dev;
  float vrSlop[3];
  long curPos;
  
  uint16_t max_speed;
  uint16_t min_speed;
  uint16_t accel;
  uint16_t daccel;
  uint32_t currReload;
  uint32_t minReload;
  uint32_t maxReload;
  int16_t reloadStep;
  //int16_t deltaReload;
  
} step_runtime;


void stepper_init(void *arg)
{
  if(arg == NULL) return;
  _stepper_ctrl_t *dev = (_stepper_ctrl_t*)arg;
  
  //step_runtime.currPos = 0;
  step_runtime.dev = arg;
//  step_runtime.ppd = (float)(STEPS_PER_REV)/360.;
  step_runtime.ppd = dev->stepperCtrl.pulsePerDegree;

  //step_runtime.stepDelay = 100;
  //step_runtime.stepLeft = 0;
  step_runtime.direction = 0;
  step_runtime.stepState = SS_STOPPED;
  //step_runtime.dev->dirLow();
  //step_runtime.dev->stepLow();
  
  
  step_runtime.vrSlop[0] = (dev->stepperCtrl.stepConv[1].ang - dev->stepperCtrl.stepConv[0].ang);
  step_runtime.vrSlop[0] /= (dev->stepperCtrl.stepConv[1].vr - dev->stepperCtrl.stepConv[0].vr);
  step_runtime.vrSlop[1] = (dev->stepperCtrl.stepConv[2].ang - dev->stepperCtrl.stepConv[1].ang);
  step_runtime.vrSlop[1] /= (dev->stepperCtrl.stepConv[2].vr - dev->stepperCtrl.stepConv[1].vr);
  step_runtime.vrSlop[2] = (dev->stepperCtrl.stepConv[3].ang - dev->stepperCtrl.stepConv[2].ang);
  step_runtime.vrSlop[2] /= (dev->stepperCtrl.stepConv[3].vr - dev->stepperCtrl.stepConv[2].vr);
  
  step_runtime.minReload = STEP_CONTROL_BASE_CLOCK/dev->stepperCtrl.max_pps;
  step_runtime.maxReload = STEP_CONTROL_BASE_CLOCK/dev->stepperCtrl.min_pps;
  step_runtime.reloadStep = STEP_RELOAD_SIZE;
  step_runtime.direction = DIR_POS;
  //step_runtime.deltaReload = step_runtime.reloadStep;
}

void stepper_move_to(int16_t pos)
{
  int16_t tmp, steps;
  float deg;
  tmp = pos;
  if(step_runtime.dev == NULL) return; 
  step_runtime.minReload = STEP_CONTROL_BASE_CLOCK/step_runtime.dev->stepperCtrl.max_pps;
  step_runtime.maxReload = STEP_CONTROL_BASE_CLOCK/step_runtime.dev->stepperCtrl.min_pps;
  //step_runtime.reloadStep = STEP_RELOAD_SIZE;
  step_runtime.reloadStep = (step_runtime.maxReload - step_runtime.minReload)/80;
  if(step_runtime.reloadStep < 2) step_runtime.reloadStep = 2;
  if(tmp <= step_runtime.dev->stepperCtrl.stepConv[0].vr){
    deg = step_runtime.dev->stepperCtrl.stepConv[0].ang;
    stepMoveHome();
    return;
  }
  else if((tmp > step_runtime.dev->stepperCtrl.stepConv[0].vr) && (tmp <= step_runtime.dev->stepperCtrl.stepConv[1].vr)){
    deg = step_runtime.vrSlop[0]*(tmp - step_runtime.dev->stepperCtrl.stepConv[0].vr);
  }
  else if((tmp > step_runtime.dev->stepperCtrl.stepConv[1].vr) && (tmp <= step_runtime.dev->stepperCtrl.stepConv[2].vr)){
    deg = step_runtime.vrSlop[1]*(tmp - step_runtime.dev->stepperCtrl.stepConv[1].vr);
    deg += step_runtime.dev->stepperCtrl.stepConv[1].ang;
  }
  else if((tmp > step_runtime.dev->stepperCtrl.stepConv[2].vr) && (tmp <= step_runtime.dev->stepperCtrl.stepConv[3].vr)){
    deg = step_runtime.vrSlop[2]*(tmp - step_runtime.dev->stepperCtrl.stepConv[2].vr);
    deg += step_runtime.dev->stepperCtrl.stepConv[2].ang;
  }
  else{
    deg = step_runtime.dev->stepperCtrl.stepConv[3].ang;
  }
  
  steps = deg * step_runtime.ppd;
  
  tmp = (step_runtime.tarPos > steps)?(step_runtime.tarPos - steps):(steps - step_runtime.tarPos);
  
  if((tmp < 10) && (tmp > 0)) return ;
  
  step_runtime.moveHome = 0;
  step_runtime.tarPos = steps;
}

void stepMoveHome()
{
//  uint8_t i;
//  for(i=0;i<4;i++) step_runtime.stepBuffer[i] = 0;
//  step_runtime.stepIndex = 0;
//  
  if((step_runtime.moveHome == 0) && (step_runtime.tarPos > 0)){
    step_runtime.moveHome = 1;
    step_runtime.tarPos = -100;
  }
  //step_runtime.reloadStep = STEP_RELOAD_SIZE>>1;
  step_runtime.minReload = STEP_CONTROL_BASE_CLOCK/step_runtime.dev->stepperCtrl.max_pps;
  step_runtime.minReload >>= 1;
  
  if(step_runtime.minReload < 20){
    step_runtime.minReload = 20;
  }
}

int32_t stepPoll(void)
{
  if(step_runtime.dev == NULL) return -1; 
  
  int32_t delta_steps;
  bool reverse = false;
  if(step_runtime.stepState != SS_STOPPED){
    if(step_runtime.direction == 1){
      if(step_runtime.tarPos < step_runtime.curPos){
        reverse = true;
      }
    }
    else if(step_runtime.direction == -1){
      if(step_runtime.tarPos > step_runtime.curPos){
        reverse = true;
      }
    }
    
    if(reverse){
      //de-acceleration and reverse
      //step_runtime.deltaReload = step_runtime.reloadStep;
      step_runtime.stepState = SS_DEACCEL;
    }
  }
  switch(step_runtime.stepState){
  case SS_STOPPED:
    if(step_runtime.curPos != step_runtime.tarPos){
      if(step_runtime.dev->stepEn != NULL){
        step_runtime.dev->stepEn();
      }
      if(step_runtime.curPos < step_runtime.tarPos){
        step_runtime.direction = 1;
        step_runtime.dev->dirHigh();
      }
      else{
        step_runtime.direction = -1;
        step_runtime.dev->dirLow();
      }
      step_runtime.stepState = SS_ACCEL;
      step_runtime.currReload = step_runtime.maxReload;
      step_runtime.dev->stepHigh();
      step_runtime.curPos += step_runtime.direction;
      
      //step_runtime.deltaReload = -step_runtime.reloadStep;
    }
    else{
//      if(step_runtime.dev->stepDis != NULL){
//        step_runtime.dev->stepDis();
//      }
      if(step_runtime.curPos == 0){
        step_runtime.currReload = 0xFFFFFFFF;
      }
      else{
        step_runtime.currReload = 0;
      }
    }
    break;
  case SS_ACCEL:
    step_runtime.currReload -= step_runtime.reloadStep;
    if(step_runtime.currReload < step_runtime.minReload){
      step_runtime.currReload = step_runtime.minReload;
      step_runtime.stepState = SS_NORMAL;
    }
    if(step_runtime.tarPos != step_runtime.curPos){
      step_runtime.dev->stepHigh();
      step_runtime.curPos += step_runtime.direction;
    }
    else{ // step less than accel count
      if(step_runtime.stepState == SS_ACCEL){
        step_runtime.stepState = SS_STOPPED;
      }
    }
    break;
  case SS_NORMAL:
    delta_steps = step_runtime.tarPos - step_runtime.curPos;
    if((delta_steps < -STEP_ACCEL_CNTR) || (delta_steps > STEP_ACCEL_CNTR)){
//    if(step_runtime.curPos != step_runtime.tarPos){
      step_runtime.dev->stepHigh();
      step_runtime.curPos += step_runtime.direction;
    }
    else{
      step_runtime.stepState = SS_DEACCEL;
    }
    break;
  case SS_DEACCEL:
    step_runtime.currReload += step_runtime.reloadStep;
    if(step_runtime.currReload > step_runtime.maxReload){
      step_runtime.currReload = step_runtime.maxReload;
      if(reverse){
        step_runtime.direction = -step_runtime.direction;
        step_runtime.stepState = SS_STOPPED;
        break;
      }
    }
    if(step_runtime.curPos != step_runtime.tarPos){
      step_runtime.dev->stepHigh();
      step_runtime.curPos += step_runtime.direction;
    }
    else{
      step_runtime.currReload = 0;
      step_runtime.stepState = SS_STOPPED;
      if(step_runtime.tarPos < 0){
        step_runtime.tarPos = 0;
        step_runtime.curPos = 0;
      }
    }
    
    break;
  }
  
  return step_runtime.currReload;
}