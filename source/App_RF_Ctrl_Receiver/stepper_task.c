#include "ch.h"
#include "hal.h"
#include "stepper_task.h"
#include "stepper.h"
#include <string.h>


#define STEPPER_PUL_LINE        PAL_LINE(GPIOA,10)
#define STEPPER_DIR_LINE        PAL_LINE(GPIOA,8)
#define STEPPER_HFC_LINE        PAL_LINE(GPIOC,8)
#define STEPPER_PUL_REVERSE         1
#define STEPPER_DIR_REVERSE             1

#define PULSE_CTRL_TIMER        &GPTD1
#define STEPPER_POLL_TIMER      &GPTD3

#define EV_UPDATE               EVENT_MASK(0)


static void gpt_callback(GPTDriver *gptp);
static void gpt_callback_poll(GPTDriver *gptp);
static bool stepDone;

static struct _stepper_runtime_s{
  thread_t *self;
}stepperRuntime;

static GPTConfig gptcfg = {
  STEP_PULSE_BASE_CLOCK,  
  gpt_callback
};

static GPTConfig gptcfg_poll = {
  STEP_CONTROL_BASE_CLOCK,      
  gpt_callback_poll
};


static void pulse_active()
{
//  gptStartOneShot(PULSE_CTRL_TIMER,TIME_US2I(100));
  gptStartOneShot(PULSE_CTRL_TIMER,STEP_PULSE_WIDTH_CNTR);
}

static void step_low(void)
{
  if(STEPPER_PUL_REVERSE){
    palSetLine(STEPPER_PUL_LINE);
  }
  else{
    palClearLine(STEPPER_PUL_LINE);
  }
  stepDone = true;
}

static void step_high(void)
{
  if(STEPPER_PUL_REVERSE){
    palClearLine(STEPPER_PUL_LINE);
  }
  else{
    palSetLine(STEPPER_PUL_LINE);
  }
  stepDone = false;
  pulse_active();
}

static void dir_low(void)
{
  if(STEPPER_DIR_REVERSE){
    palSetLine(STEPPER_DIR_LINE);
  }
  else{
    palClearLine(STEPPER_DIR_LINE);
  }
}

static void dir_high(void)
{
  if(STEPPER_DIR_REVERSE){
    palClearLine(STEPPER_DIR_LINE);
  }
  else{
    palSetLine(STEPPER_DIR_LINE);
  }
}

static void hfc_en()
{
  palSetLine(STEPPER_HFC_LINE);
}

static void hfc_dis()
{
  palClearLine(STEPPER_HFC_LINE);
}


_stepper_ctrl_t stepperCtrl = {
  step_low,
  step_high,
  dir_low,
  dir_high,
  NULL,
  NULL,
//  {
//    {100,0},
//    {500,60},
//    {1500,120},
//    {2500,180}
//    
//  },
//  {
//    {30, 0, 200},
//    {120,200,500},
//    {180,500,1000}
//  },
};


static THD_WORKING_AREA(waStepperTask, 512);
static THD_FUNCTION(procStepperTask,p)
{
  chRegSetThreadName("Stepper");
  _stepper_ctrl_t *ctrl = (_stepper_ctrl_t*)p;
  stepper_init(ctrl);
  bool bStop = false;
  int32_t reload;
  step_low();
  dir_high();
  hfc_en();
  
  stepDone = true;
  //gptStartContinuous(STEPPER_POLL_TIMER,TIME_MS2I(1));
  while(!bStop)
  {
    
    bStop = chThdShouldTerminateX();
//    eventmask_t evt = chEvtWaitAnyTimeout(ALL_EVENTS, TIME_MS2I(10));
    if(stepDone){
      reload = stepPoll();
      if(reload > 0 ){
        //pulse_active();
        gptStartOneShot(STEPPER_POLL_TIMER,reload);
        //stepDone = false;
      }
      else{
        gptStartOneShot(STEPPER_POLL_TIMER,1000);
      }
    }
    else{
      gptStartOneShot(STEPPER_POLL_TIMER,1000);
    }
    eventmask_t evt = chEvtWaitAny(ALL_EVENTS);
    
      
  }
}

void stepper_task_init(void *vrmap)
{
  gptInit();
  gptStart(PULSE_CTRL_TIMER, &gptcfg);
  gptStart(STEPPER_POLL_TIMER, &gptcfg_poll);
  memcpy((void*)&stepperCtrl.stepperCtrl,(void*)vrmap,sizeof(struct _stepper_control_s));
  stepperRuntime.self = chThdCreateStatic(waStepperTask,sizeof(waStepperTask),NORMALPRIO+1,procStepperTask,&stepperCtrl);
}

static void gpt_callback(GPTDriver *gptp)
{
  (void)gptp;
  chSysLockFromISR();
  step_low();
  chSysUnlockFromISR();
}

static void gpt_callback_poll(GPTDriver *gptp)
{
  (void)gptp;
  chSysLockFromISR();
  chEvtSignalI(stepperRuntime.self, EV_UPDATE);
  chSysUnlockFromISR();
}