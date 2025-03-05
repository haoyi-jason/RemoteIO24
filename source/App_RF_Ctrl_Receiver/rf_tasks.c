#include "ch.h"
#include "hal.h"
#include "rf_task.h"
#include "../drivers/bc3601.h"
#include "../drivers/dev_bc3601.h"
//#include "usbcfg.h"
#include "../drivers/nvm.h"
#include "nvm_config.h"



#define NVM_FLAG        0xCC



static const SPIConfig spicfg = {
  false,
  NULL,
  NULL,//GPIOB,
  NULL,//GPIOB_SPI1_CS,
  SPI_CR1_BR_2  
};

#ifdef RF_SWITCH
static bc3601_config_t config = {
  &SPID2,
  &spicfg,
  GPIOB,
  12
};
#else
static bc3601_config_t config = {
  &SPID2,
  &spicfg,
  GPIOC,
  7
};
#endif


static BC3601Driver bc3601;



struct _deviceConfig {
  uint8_t flag;
  uint16_t selfAddr;
  uint16_t destAddr;
  uint8_t opMode;  // 1: rf switch tx, 2: rf switch rx, 3: rf i/o tx, 4: rf i/o rx
  uint8_t txIntervalMs; // tx interval, in ms
  uint16_t txCounts; // number of times to transmit packet
  struct _rx_control_config rx_control;
  uint8_t reserved[13]; // reserve total 64-bytes for this struct
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

struct _nvm nvmParam, *rfNvmParam;

static struct{
  thread_t *self;
  thread_t *mainThread;
  thread_t *usbtrx;
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
  uint8_t userMode;
}runTime;

typedef struct {
  uint8_t sz;
  uint8_t data[64];
}_txPacket;

static _txPacket txPacket;

typedef struct{
  uint16_t dio; // bit 0~9 for action, bit 15 for switch
  uint16_t advalue;
  uint16_t srcAddr;
  uint16_t dstAddr;
  uint8_t rsv1;
  uint8_t checksum;
}_rfPacket;

static void load_settings()
{
  uint16_t nvmSz = sizeof(nvmParam);
//  if(nvmSz > SZ_NVM_CONFIG){
//    // error
//    while(1);
//  }
  nvm_flash_read(OFFSET_NVM_BOARD,(uint8_t*)&nvmParam,nvmSz);
  if(nvmParam.deviceConfig.flag != NVM_FLAG){
    nvmParam.deviceConfig.flag = NVM_FLAG;
    nvmParam.deviceConfig.selfAddr = 0x01;
    nvmParam.deviceConfig.destAddr = 0x01;
    nvmParam.deviceConfig.opMode = 4;
    nvmParam.deviceConfig.txIntervalMs = 50;
    nvmParam.deviceConfig.txCounts = 90;
    
    nvmParam.rfConfig.dataRate = 2; // 10k
    nvmParam.rfConfig.frequency = 915.;
    nvmParam.rfConfig.txPower = 5; // 10 dbm
 
    nvmParam.tx_dio = 0xC000; // default tx pattern
    
    nvmParam.deviceConfig.rx_control.vr_map[0].angle = 0;
    nvmParam.deviceConfig.rx_control.vr_map[0].raw = 1200;
    nvmParam.deviceConfig.rx_control.vr_map[1].angle = 30;
    nvmParam.deviceConfig.rx_control.vr_map[1].raw = 1600;
    nvmParam.deviceConfig.rx_control.vr_map[2].angle = 170;
    nvmParam.deviceConfig.rx_control.vr_map[2].raw = 2600;
    nvmParam.deviceConfig.rx_control.vr_map[3].angle = 200;
    nvmParam.deviceConfig.rx_control.vr_map[3].raw = 3000;
    
    nvmParam.deviceConfig.rx_control.pulsePerDegree = (320*10)/360.;
    nvmParam.deviceConfig.rx_control.min_pps = 500;
    nvmParam.deviceConfig.rx_control.max_pps = 1500;
    nvm_flash_write(OFFSET_NVM_BOARD,(uint8_t*)&nvmParam,nvmSz);
  }
}

void rf_save_nvm()
{
  nvm_flash_write(OFFSET_NVM_BOARD,(uint8_t*)&nvmParam,sizeof(nvmParam));
}

uint8_t checksum(uint8_t *d, uint8_t n)
{
  uint8_t sum = 0;
  for(uint8_t i=0;i<n;i++){
    sum += *d++;
  }
  return sum;
}

static void set_rf_addr()
{
  uint8_t reg = 0;
  // set address, 14-bit format
  reg = nvmParam.deviceConfig.selfAddr & 0x3F;
  BC3601_REG_WRITE(&bc3601,HEADER_ADDR0_REGS,&reg); 
  BC3601_REG_READ(&bc3601,HEADER_ADDR0_REGS,&reg); 
  reg = (nvmParam.deviceConfig.selfAddr>>8);
  BC3601_REG_WRITE(&bc3601,HEADER_ADDR1_REGS,&reg); 
  BC3601_REG_READ(&bc3601,HEADER_ADDR1_REGS,&reg);
}

#define STAGE_START 2   //0:tx, 2:rx
static THD_WORKING_AREA(waRemoteIO,4096);
static THD_FUNCTION(procRemoteIO ,p)
{
  BaseSequentialStream *stream = (BaseSequentialStream*)p;
  _rfPacket *rfp = (_rfPacket*)(&txPacket.data[0]);
  BC3601Driver *dev = &bc3601;
  uint8_t rx[64];
  uint8_t reg = 0;
  uint8_t stage = STAGE_START;
  uint8_t wait = 0;
  eventmask_t evt;
  bool txPending = false;
  txPacket.sz = sizeof(_rfPacket)+1;
  rfp->srcAddr = nvmParam.deviceConfig.selfAddr;
  rfp->dstAddr = nvmParam.deviceConfig.destAddr;
  while(1){
    
    evt = chEvtWaitAnyTimeout(ALL_EVENTS,50);
    if(evt & EV_TX){
      txPending = true;
      stage = 0;
    }
    
    if(evt & EV_SAVE){
      nvm_flash_write(OFFSET_NVM_BOARD,(uint8_t*)&nvmParam,sizeof(nvmParam));
    }
    
    switch(stage){
    case 0:
      //  tx
      rfp->dio = runTime.dio_state;
      rfp->advalue = runTime.ain_value;
      rfp->checksum = checksum((uint8_t*)rfp,sizeof(_rfPacket)-1);
//      BC3601_LITE_SLEEP(&bc3601);
//      irq_config(&bc3601,IRQ2_TXCMPIE);
      BC3601_RESET_TXFIFO(dev);
      reg = 0;
      BC3601_SET_TX_PAYLOAD_SADDR(dev,&reg);  
//      BC3601_SET_TX_PAYLOAD_WIDTH(dev,&txPacket.sz);
      BC3601_FIFO_WRITE(dev,(uint8_t*)&txPacket,txPacket.sz);
      BC3601_TX_MODE(dev);
      
     // txPending = false;
      //txPacket.sz = 0;
      
//      bc3601_refresh_registers(&bc3601);
      stage++;
      break;
    case 1: // check irq state
      stage = 0;
//      reg = bc3601_irqState(&bc3601);
//      if(reg & IRQ3_TXCMPIF){
//        
//        BC3601_LITE_SLEEP(&bc3601);
//        stage = 2;
//      }
      break;
    case 2:
//      irq_config(dev,IRQ2_RXERRIE | IRQ2_RXCMPIE);
//      BC3601_SET_RX_PAYLOAD_WIDTH(dev,&reg);
      BC3601_RESET_RXFIFO(dev);
      reg = 4;
      BC3601_SET_RX_PAYLOAD_WIDTH(dev,&reg);
      BC3601_RX_MODE(dev);
      irq_config(dev,IRQ2_RXCMPIE);
//      bc3601_refresh_registers(&bc3601);
      stage++;
      break;
    case 3:
      reg = bc3601_irqState(&bc3601);
      if(reg & (IRQ3_RXCMPIF | IRQ3_RXERRIF)){
        stage = 99;
        BC3601_REG_READ(&bc3601,RX_DATA_LENG_REGS,&reg);
        
        //chprintf((BaseSequentialStream*)&SDU1,"RSSI=%d:",bc3601_readRSSI(&bc3601));
        if(reg > 0){
          BC3601_FIFO_READ(&bc3601,rx,reg);
          _txPacket *r = (_txPacket*)rx;
          for(uint8_t i=0;i<r->sz;i++){
            //chprintf((BaseSequentialStream*)&SDU1,"0x%02X ",r->data[i]);
          }
        }
        //chprintf((BaseSequentialStream*)&SDU1,"\n");
        stage = 99;
      }      
        stage = 0;
//      wait++;
//      if(wait > 100){
//        wait = 0;
//        stage = 99;
//        chprintf((BaseSequentialStream*)&SDU1,"Wait Timeout\n");
//      }
      break;
    default:
      bc3601_refresh_registers(&bc3601);
      BC3601_LITE_SLEEP(&bc3601);
     // stage = 2;
      stage = STAGE_START;
      break;
    }
    //chThdSleepMilliseconds(200);
  }
}

static THD_WORKING_AREA(waSwitchTX,1024);
static THD_FUNCTION(procSwitchTX ,p)
{
  BC3601Driver *dev = &bc3601;
  _rfPacket rfp;
  //rfp.dio = SWITCH_TX_PATTERN;
  rfp.dio = nvmParam.tx_dio;
  rfp.advalue = 0x0;
  rfp.srcAddr = nvmParam.deviceConfig.selfAddr;
  rfp.dstAddr = nvmParam.deviceConfig.destAddr;
  rfp.checksum = checksum((uint8_t*)&rfp,sizeof(rfp)-1);
  uint8_t reg;
  uint8_t pktSz = sizeof(rfp);
  uint8_t cycles = 0;
  txPacket.sz = pktSz+1;
  memcpy(txPacket.data,(uint8_t*)&rfp, pktSz);
  bool bStop = false;
  while((cycles < nvmParam.deviceConfig.txCounts) && !bStop){
    cycles++;
    BC3601_RESET_TXFIFO(dev);
    reg = 0;
    BC3601_SET_TX_PAYLOAD_SADDR(dev,&reg);
    //BC3601_SET_TX_PAYLOAD_WIDTH(dev,&pktSz);
    BC3601_FIFO_WRITE(dev,(uint8_t*)&txPacket,txPacket.sz);
    BC3601_TX_MODE(dev);
//    rfp.advalue++;
    chThdSleepMilliseconds(nvmParam.deviceConfig.txIntervalMs);
    if(chThdShouldTerminateX()){
      bStop = true;
    }
  }
  
  if(!bStop){
//    BC3601_STBY(dev);
//    BC3601_DEEP_SLEEP(dev);
//    runTime.self = NULL;
//    PWR->CR |= (PWR_CR_PDDS);
//    SCB->SCR |= (SCB_SCR_SLEEPDEEP_Msk);
//    PWR->CSR &= ~(PWR_CRSTS_WUF);
//    chSysDisable();
//    __WFI();  
   // runTime.mainThread = NULL;
  }
//  else{          
//  }
    chThdExit(0);
//  }
}

static THD_WORKING_AREA(waSwitchRX,1024);
static THD_FUNCTION(procSwitchRX ,p)
{
  BC3601Driver *dev = &bc3601;
  _rfPacket *rfp;
  uint8_t reg;
  //txPacket.sz = pktSz;
  uint8_t rx[64];
  //uint8_t stage = 0;
  //uint16_t cycles = 0;
  //uint16_t rxLost = 0;
  uint8_t rssi;
  bool bRun = true;
  runTime.rf_ctrl = 0x0;
  runTime.taskFinished = 0;
  while(bRun){
    //rf_ctrl = 0x0;
    if(chThdShouldTerminateX()){
      bRun = false;
    }
    switch(runTime.stage){
    case 0:
      BC3601_LITE_SLEEP(&bc3601);
      BC3601_RESET_RXFIFO(dev);
//      reg = 4;
//      BC3601_SET_RX_PAYLOAD_WIDTH(dev,&reg);
      BC3601_RX_MODE(dev);
      irq_config(dev,IRQ2_RXCMPIE);
      runTime.stage++;
      break;
    case 1:
      reg = bc3601_irqState(&bc3601);
      if(reg & (IRQ3_RXCMPIF | IRQ3_RXERRIF)){
        runTime.rssi = bc3601_readRSSI(&bc3601);       
        BC3601_REG_READ(&bc3601,RX_DATA_LENG_REGS,&reg);       
        if(reg > 0){
          BC3601_FIFO_READ(&bc3601,rx,reg);
          _txPacket *r = (_txPacket*)rx;
          r->sz--;
          if(r->sz == sizeof(_rfPacket)){
            rfp = (_rfPacket*)r->data;
            if(checksum((uint8_t*)rfp,sizeof(_rfPacket)-1) == rfp->checksum){
              if(runTime.taskFinished == 0 && runTime.rf_ctrl == 0){
                runTime.rf_ctrl = rfp->dio;
                runTime.cycles = 0;
                runTime.rxLost = 0;
                chEvtSignal(runTime.mainThread,EV_RX_PACKET);
              }
            }
          }
        }
      }    
      else{ // check sum fail
        runTime.rxLost += nvmParam.deviceConfig.txIntervalMs;
        if(runTime.rxLost > 1000){
          runTime.rf_ctrl = 0x0;
          runTime.taskFinished = 0; // reset task
        }
      }
      if(runTime.rf_ctrl != 0x0){
        runTime.cycles++;
        runTime.cycles++;
        if(runTime.cycles > nvmParam.deviceConfig.txCounts){
          runTime.rf_ctrl = 0x0;
          runTime.taskFinished = 1;
        }
      }
      runTime.stage = 0;
      break;
    case 2:
      runTime.cycles++;
      if(runTime.cycles > nvmParam.deviceConfig.txCounts){
        runTime.rf_ctrl = 0x0;
//        palClearPad(GPIOB,6);
//        palClearPad(GPIOB,7);
        runTime.stage = 0;
        runTime.taskFinished = 1;
      }
      
      break;
    default:
      break;
    }
    // valid input
    if(palReadPad(GPIOA,15) == PAL_LOW){
      //palSetPad(GPIOB,7);
      runTime.rf_ctrl |= 0x8000;
    }
//    else{
//      palClearPad(GPIOB,7);
//    }
    // valid output
    if(runTime.rf_ctrl & 0x4000){
      palSetPad(GPIOB,6);
    }
    else{
      palClearPad(GPIOB,6);
    }
    if(runTime.rf_ctrl & 0x8000){
      palSetPad(GPIOB,7);
    }
    else{
      palClearPad(GPIOB,7);
    }
    eventmask_t evt = chEvtWaitAnyTimeout(ALL_EVENTS,TIME_IMMEDIATE);
    chThdSleepMilliseconds(nvmParam.deviceConfig.txIntervalMs);
  }
  
  chThdExit(0);
}

static THD_WORKING_AREA(waTX2,1024);
static THD_FUNCTION(procTX2 ,p)
{
  BC3601Driver *dev = &bc3601;
  _rfPacket *rfp = (_rfPacket*)(&txPacket.data[0]);
  //rfp.dio = SWITCH_TX_PATTERN;
  rfp->dio = nvmParam.tx_dio;
  rfp->advalue = 0x0;
  rfp->srcAddr = nvmParam.deviceConfig.selfAddr;
  rfp->dstAddr = nvmParam.deviceConfig.destAddr;
  rfp->checksum = checksum((uint8_t*)&rfp,sizeof(_rfPacket)-1);
  uint8_t reg;
  uint8_t pktSz = sizeof(_rfPacket);
  uint16_t cycles = 0;
  txPacket.sz = pktSz+1;
  //memcpy(txPacket.data,(uint8_t*)&rfp, pktSz);
  bool bStop = false;
  while(!bStop){
    //cycles++;
    BC3601_RESET_TXFIFO(dev);
    reg = 0;
    // read analog input and i/o state
    if(runTime.dio_state == 0x0){
      cycles++;
    }
    else{
      cycles = 0;
    }
    if((cycles > nvmParam.deviceConfig.txCounts) && (runTime.userMode == 0)){
      bStop = true;
    }
    rfp->dio = runTime.dio_state;
    rfp->advalue = runTime.ain_value;
    rfp->checksum = checksum((uint8_t*)rfp,sizeof(_rfPacket)-1);
    chEvtSignal(runTime.mainThread,EV_TX_DATA_REQUEST);
    
    BC3601_SET_TX_PAYLOAD_SADDR(dev,&reg);
    //BC3601_SET_TX_PAYLOAD_WIDTH(dev,&pktSz);
    BC3601_FIFO_WRITE(dev,(uint8_t*)&txPacket,txPacket.sz);
    BC3601_TX_MODE(dev);
//    rfp.advalue++;
    chThdSleepMilliseconds(nvmParam.deviceConfig.txIntervalMs);
    if(chThdShouldTerminateX() && (runTime.userMode == 0)){
      bStop = true;
    }
  }
  BC3601_STBY(dev);
  BC3601_DEEP_SLEEP(dev);
  chEvtSignal(runTime.mainThread,EV_TX_DONE);
  chThdExit(0);
  
//  if(!bStop){
//    BC3601_STBY(dev);
//    BC3601_DEEP_SLEEP(dev);
//    runTime.self = NULL;
//    PWR->CR |= (PWR_CR_PDDS);
//    SCB->SCR |= (SCB_SCR_SLEEPDEEP_Msk);
//    PWR->CSR &= ~(PWR_CRSTS_WUF);
//    chSysDisable();
//    __WFI();  
//    runTime.mainThread = NULL;
//  }
//  else{          
//  }
//    chThdExit(0);
//  }
}
static THD_WORKING_AREA(waRX2,1024);
static THD_FUNCTION(procRX2 ,p)
{
  BC3601Driver *dev = &bc3601;
  _rfPacket *rfp;
  uint8_t reg;
  //txPacket.sz = pktSz;
  uint8_t rx[64];
  //uint8_t stage = 0;
  //uint16_t cycles = 0;
  //uint16_t rxLost = 0;
  uint8_t rssi;
  bool bRun = true;
  runTime.rf_ctrl = 0x0;
  runTime.taskFinished = 0;
  runTime.rfLinkFail = 0;
  while(bRun){
    //rf_ctrl = 0x0;
    if(chThdShouldTerminateX()){
      bRun = false;
    }
    switch(runTime.stage){
    case 0:
      BC3601_LITE_SLEEP(&bc3601);
      BC3601_RESET_RXFIFO(dev);
//      reg = 4;
//      BC3601_SET_RX_PAYLOAD_WIDTH(dev,&reg);
      BC3601_RX_MODE(dev);
      irq_config(dev,IRQ2_RXCMPIE);
      runTime.stage++;
      runTime.rxLost = 0;
      break;
    case 1:
      reg = bc3601_irqState(&bc3601);
      if(reg & (IRQ3_RXCMPIF | IRQ3_RXERRIF)){
        runTime.rssi = bc3601_readRSSI(&bc3601);       
        BC3601_REG_READ(&bc3601,RX_DATA_LENG_REGS,&reg);       
        if(reg > 0){
          BC3601_FIFO_READ(&bc3601,rx,reg);
          BC3601_RESET_RXFIFO(dev);
          BC3601_RX_MODE(dev);
          _txPacket *r = (_txPacket*)rx;
          r->sz--;
          if(r->sz == sizeof(_rfPacket)){
            rfp = (_rfPacket*)r->data;
            if(checksum((uint8_t*)rfp,sizeof(_rfPacket)-1) == rfp->checksum){
              //if(runTime.taskFinished == 0 && runTime.rf_ctrl == 0){
                runTime.rf_ctrl = rfp->dio;
                runTime.dio_state = rfp->dio;
                runTime.ain_value = rfp->advalue;
                runTime.cycles = 0;
                runTime.rxLost = 0;
                chEvtSignal(runTime.mainThread, EV_RX_PACKET);
                runTime.rfLinkFail = 0;
              //}
            }
            else{
              chEvtSignal(runTime.mainThread, EV_RX_ERROR);
              
            }
          }
        }
        //runTime.stage = 0;
      }    
      else{
        if(runTime.rxLost < 1000){
          runTime.rxLost += nvmParam.deviceConfig.txIntervalMs;
        }
        if(runTime.rxLost >= 1000 && (runTime.rfLinkFail == 0)){
          chEvtSignal(runTime.mainThread, EV_RX_LOST);
          runTime.rf_ctrl = 0x0;
          runTime.dio_state = 0x0;
          runTime.ain_value = 0x0;
          runTime.rfLinkFail = 1;
//          runTime.taskFinished = 0; // reset task
//          runTime.stage = 0;
        }
      }
      break;
    case 2:
      runTime.cycles++;
      if(runTime.cycles > nvmParam.deviceConfig.txCounts){
        runTime.rf_ctrl = 0x0;
//        palClearPad(GPIOB,6);
//        palClearPad(GPIOB,7);
        runTime.stage = 0;
        runTime.taskFinished = 1;
      }
      
      break;
    default:
      break;
    }
//    chThdSleepMilliseconds(nvmParam.deviceConfig.txIntervalMs);
    chThdSleepMilliseconds(20);
  }
  
  chThdExit(0);
}



void rf_send(uint8_t *d, uint8_t n)
{
  if(txPacket.sz == 0){
    txPacket.sz = n;
    memcpy(txPacket.data,d,n);
    chEvtSignal(runTime.self,EV_TX);
  }
}

void rf_setDestAddr(uint16_t addr)
{  
  nvmParam.deviceConfig.destAddr = addr;
}

uint16_t rf_getDestAddr()
{
  return nvmParam.deviceConfig.destAddr;
}

uint16_t rf_getSelfAddr()
{
  return nvmParam.deviceConfig.selfAddr;
}

void rf_setSelfAddr(uint16_t addr)
{
  nvmParam.deviceConfig.selfAddr = addr;
  set_rf_addr();
}

void rf_setPower(uint8_t value)
{
  uint8_t reg = 0;
  bc3601.txPower = value;
  bc3601_powerConfig(&bc3601,bc3601.txPower);
  nvmParam.rfConfig.txPower = value;
}

uint8_t rf_getPower()
{
  return nvmParam.rfConfig.txPower;
}

void rf_setFreq(float value)
{
  bc3601.frequency = value;
  bc3601_freqConfig(&bc3601,bc3601.frequency);
  nvmParam.rfConfig.frequency = value;
}

float rf_getFreq()
{
  return (nvmParam.rfConfig.frequency);
}

void rf_setDataRate(uint8_t value)
{
  bc3601.dataRate = value;
  bc3601_datarateConfig(&bc3601,bc3601.dataRate);
  nvmParam.rfConfig.dataRate = value;
}

uint8_t rf_getDataRate()
{
  return nvmParam.rfConfig.dataRate;
}

void rf_setMode(uint8_t mode)
{
  nvmParam.deviceConfig.opMode = mode;
}

uint8_t rf_getMode()
{
  return nvmParam.deviceConfig.opMode;
}

void rf_saveParam()
{
  chEvtSignal(runTime.mainThread,EV_SAVE);
}

void rf_set_io_pattern(uint16_t val)
{
  nvmParam.tx_dio = val;
}

uint16_t rf_get_io_pattern()
{
  return nvmParam.tx_dio;
}

void rf_set_pollInterval(uint16_t val)
{
  nvmParam.deviceConfig.txIntervalMs = val;
}

uint16_t rf_get_pollInterval()
{
  return nvmParam.deviceConfig.txIntervalMs;
}

void rf_set_pollCount(uint16_t val)
{
  nvmParam.deviceConfig.txCounts = val;
}

uint16_t rf_get_pollCount()
{
  return nvmParam.deviceConfig.txCounts;
}

// tasks for USB Interface exist
void rf_task_start_manual_mode(BaseSequentialStream *stream)
{
  runTime.userMode = 1;
//  if(runTime.self != NULL){
//    chThdTerminate(runTime.self);
//    chThdWait(runTime.self);
//    runTime.self = NULL;
//  }
//  
//  runTime.self = chThdCreateStatic(waRemoteIO,sizeof(waRemoteIO),NORMALPRIO,procRemoteIO,stream);
}

void rf_task_stop_manual_mode()
{
  runTime.userMode = 0;
//  if(runTime.self != NULL){
//    chThdTerminate(runTime.self);
//    chThdWait(runTime.self);
//    runTime.self = NULL;
//  }
}

void rf_set_ppd(float value,uint16_t minp, uint16_t maxp)
{
  nvmParam.deviceConfig.rx_control.pulsePerDegree = value;
  nvmParam.deviceConfig.rx_control.min_pps = minp;
  nvmParam.deviceConfig.rx_control.max_pps = maxp;
}

void rf_get_ppd(float *value, uint16_t *minp, uint16_t *maxp)
{
  *value = nvmParam.deviceConfig.rx_control.pulsePerDegree;
  *minp = nvmParam.deviceConfig.rx_control.min_pps;
  *maxp = nvmParam.deviceConfig.rx_control.max_pps;
}



int8_t rf_task_init()
{
  //palClearPad(GPIOB,6);
  //palClearPad(GPIOB,7);
  load_settings();
  // rf chip initial
  bc3601.txPower = nvmParam.rfConfig.txPower;
  bc3601.dataRate = nvmParam.rfConfig.dataRate;
  bc3601.frequency = nvmParam.rfConfig.frequency;
  bc3601.destAddr = nvmParam.deviceConfig.destAddr;

  dev_bc3601Init(&bc3601,&config);
  //nvmParam.deviceConfig.opMode = 99;
//  if(palReadPad(GPIOB,0) == PAL_HIGH){ // RX mode
//    nvmParam.deviceConfig.opMode = 2;
//  }
//  else{
//    nvmParam.deviceConfig.opMode = 1;
//  }
//  nvmParam.deviceConfig.opMode = 1; // force tx mode
  runTime.userMode = 0;
  set_rf_addr();
 //nvmParam.deviceConfig.opMode = 3;  // transmitter
 //nvmParam.deviceConfig.opMode = 4;
 
 runTime.mainThread = chRegFindThreadByName("Main");
 
  switch(nvmParam.deviceConfig.opMode){
  case 1:
    runTime.self = chThdCreateStatic(waSwitchTX,sizeof(waSwitchTX),NORMALPRIO,procSwitchTX,NULL);
   // chThdWait(runTime.self);
   // return 1;

    break;
  case 2:
    runTime.self = chThdCreateStatic(waSwitchRX,sizeof(waSwitchRX),NORMALPRIO,procSwitchRX,NULL);
    break;
  case 3:
    runTime.self = chThdCreateStatic(waTX2,sizeof(waTX2),NORMALPRIO,procTX2,NULL);
    break;
  case 4:
    runTime.self = chThdCreateStatic(waRX2,sizeof(waRX2),NORMALPRIO,procRX2,NULL);
    break;
  default:
    runTime.self = chThdCreateStatic(waRemoteIO,sizeof(waRemoteIO),NORMALPRIO,procRemoteIO,NULL);
    break;
  }
  
  //chThdSleepMilliseconds(20);
  //runTime.mainThread = chRegFindThreadByName("Main");

  
  return 0;
//  chSysLock();
//  chThdSuspendS(&runTime.ref);
//  chSysUnlock();
}

msg_t sendPacket(uint8_t sz, uint8_t *data)
{
  if(txPacket.sz == 0){
    txPacket.sz = sz;
    memcpy(txPacket.data,data,sz);
    //schEvtSignal(
    return MSG_OK;
  }
  return MSG_RESET;
}

int8_t txStopped()
{
  return (runTime.self == NULL)?1:0;
}

uint16_t read_ctrl_state()
{
  return runTime.dio_state;
}

void rf_write_ctrl_state(uint16_t value)
{
  runTime.dio_state = value;
}

void rf_write_analog_state(uint16_t value)
{
  runTime.ain_value = value;
}

uint16_t read_analog_state(uint8_t channel)
{
  return runTime.ain_value;
}

uint8_t rf_op_mode()
{
  return nvmParam.deviceConfig.opMode;
}

struct _rx_control_config *control_struct()
{
  return &nvmParam.deviceConfig.rx_control;
}