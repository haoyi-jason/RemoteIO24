#include "ch.h"
#include "hal.h"
#include "vnode_app.h"
#include "adxl355_dev.h"

struct {
  uint8_t state;
  uint8_t *rxPtr;
  uint8_t *txPtr;
  uint8_t *bufEnd;
  uint8_t buffer[1200];
  mutex_t mutex;
  uint16_t rxSz;
}runTime;

static SPIConfig spicfg = {
  false,
  NULL,
  GPIOA,
  4,
  SPI_CR1_BR_2
};

_adxl_interface_t adxlInterface = {
  &SPID1,
  &spicfg_adxl
};

adxl355_config_t adxlConfig = {
  0x0,0x0,0x0,0x0
};


static ADXLDriver adxl = {
  &adxlInterface,
  //&adxlConfig
};

void adxl_int_handler(void *arg)
{
  chSysLockFromISR();
  chEvtBroadcastFlagsI(&appParam.runtime.es_sensor,EV_ADXL_FIFO_FULL);
  chSysUnlockFromISR();
}

static THD_WORKING_AREA(waOperation,1200);
static THD_FUNCTION(procOperation ,p)
{
  cmd_header_t *header;
  size_t sz;
  uint8_t buf[320];
  uint16_t bufSz = 0;
  int32_t data[96];
  uint8_t *p_src,*p_dst;
  uint16_t bsz;
  systime_t t_start;
  uint8_t pktCount = 0;
  static uint8_t adxl_sta;

  node_param_t *node = (node_param_t*)buf;
  adxl355_config_t *adxlcfg = (adxl355_config_t*)buf;
  time_domain_param_t* time = (time_domain_param_t*)buf;
  param_read_ptr(NVM_NODE,buf,16);
  
  uint8_t opMode = node->opMode;
  uint8_t activeSensor = node->activeSensor;
  
  if(node->commType == COMM_USE_SERIAL){
    opMode = OP_VNODE;
  }
  
  resetObject(&m_timeDomain);
 
  bufSz = CMD_STRUCT_SZ;
  appParam.runtime.txBuf.sz = CMD_STRUCT_SZ;
  uint8_t nofSampleToIgnore = 5;
  bool bStop = false;

  event_listener_t elSensor;
  eventflags_t flags;
  //activeSensor = SENSOR_ISM330;
  if(activeSensor == SENSOR_ADXL355){
    chEvtRegisterMask(&appParam.runtime.es_sensor,&elSensor,EV_ADXL_FIFO_FULL );
    // enable interrupt
    palSetLineCallback(LINE_ADXL_INT1,adxl_int_handler,NULL);
    palEnableLineEvent(LINE_ADXL_INT1,PAL_EVENT_MODE_FALLING_EDGE);
    appParam.runtime.ledBlink.ms_on = 500;
    appParam.runtime.ledBlink.ms_off = 500;
    adxl355_get_fifo_size(&adxl,&sz);
    if(sz > 0){
      bStop = false;
    }
    adxl355_cmd_start(&adxl);  
  }
  else if(activeSensor == SENSOR_ISM330){
    chEvtRegisterMask(&appParam.runtime.es_sensor,&elSensor,EV_ISM_FIFO_FULL );
    palSetLineCallback(LINE_ISM_INT,gpioa3_int_handler,NULL);
    palEnableLineEvent(LINE_ISM_INT,PAL_EVENT_MODE_RISING_EDGE);
    appParam.runtime.ledBlink.ms_on = 500;
    appParam.runtime.ledBlink.ms_off = 500;
    ism330_cmd_start_config();
  }
  
  uint8_t packetToIgnore = 5;
  if(appParam.runtime.resetBuffer != NULL){
    appParam.runtime.resetBuffer();
  }
  while(!bStop){
    chEvtWaitAny(ALL_EVENTS);
    flags = chEvtGetAndClearFlags(&elSensor);
    if(flags & EV_ADXL_FIFO_FULL){
      adxl355_get_status(&adxl,&adxl_sta);
      if((adxl_sta & 0x02) == 0x00) continue;
      A0_HI;
      // read fifo data, sz indicate the number of records, not bytes
      adxl355_get_fifo_size(&adxl,&sz);
      fifo_size = sz;
      sz /= 3;
      if(sz){
        switch(opMode){
        case OP_STREAM:
          bsz = sz*9; // read x/y/z combo
          if(bsz > 144) 
            bsz = 144;
          if(bsz == 144){
            adxl355_read_fifo(&adxl,&appParam.runtime.txBuf.buffer[appParam.runtime.txBuf.sz],bsz); // each record has 9-bytes (x/y/z)*3
            if(packetToIgnore){
              packetToIgnore--;
              appParam.runtime.txBuf.sz = CMD_STRUCT_SZ;
            }else{
              appParam.runtime.txBuf.sz += bsz;
            }
            //if(appParam.runtime.txBuf.sz > 270){
              if(appParam.runtime.txBuf.sz > 200){
                header = (cmd_header_t*)appParam.runtime.txBuf.buffer;
                header->magic1 = MAGIC1;
                header->magic2 = MAGIC2;
                header->type = MASK_DATA | appParam.runtime.lbt;
                header->len = appParam.runtime.txBuf.sz;
                header->pid = pktCount++;
                header->chksum = cmd_checksum(appParam.runtime.txBuf.buffer,header->len);
                if(appParam.runtime.writefcn){
                  appParam.runtime.writefcn(appParam.runtime.txBuf.buffer, header->len);
                }
                appParam.runtime.txBuf.sz = CMD_STRUCT_SZ;
              }
          }
          break;
        case OP_VNODE:
        case OP_OLED:
          bsz = sz*9;
          adxl355_read_fifo(&adxl,buf,bsz); // each record has 9-bytes (x/y/z)*3
          bsz = sz*4*3;
          p_src = buf;
          p_dst = (uint8_t*)data;
          p_dst += 3;
          for(uint16_t j=0;j<bsz;j++){
            if((j%4)==3){
              *(p_dst--)=0;
              p_dst = (uint8_t*)data + j + 4;
            }else{
              *(p_dst--)=*(p_src++);
            }
          }
          if(packetToIgnore){
            packetToIgnore--;
          }
          else if(feed_fifo32(&m_timeDomain,(uint8_t*)data,sz)==1){
            // update modbus data
            memcpy((void*)&appParam.runtime.rms,(void*)&m_timeDomain.rms,12);
            //memcpy((void*)&appParam.runtime.peak,(void*)&m_timeDomain.peak,12);
            memcpy((void*)&appParam.runtime.crest,(void*)&m_timeDomain.crest,12);
            memcpy((void*)&appParam.runtime.velocity,(void*)&m_timeDomain.velocity,12);
            appParam.runtime.peak.x = (m_timeDomain.peak.x - m_timeDomain.peakn.x);
            appParam.runtime.peak.y = (m_timeDomain.peak.y - m_timeDomain.peakn.y);
            appParam.runtime.peak.z = (m_timeDomain.peak.z - m_timeDomain.peakn.z);
            resetObject(&m_timeDomain);
            
            // send data via WIFI/BT
            // prepare data transmit
            if(opMode == OP_VNODE){
              header = (cmd_header_t*)buf;
              header->magic1 = MAGIC1;
              header->magic2 = MAGIC2;
              header->type = MASK_DATA;
              header->pid = pktCount++;
              uint8_t *ptr = &buf[CMD_STRUCT_SZ];
              memcpy(ptr,&appParam.runtime.peak,12);
              ptr += 12;
              memcpy(ptr,&appParam.runtime.rms,12);
              ptr += 12;
              memcpy(ptr,&appParam.runtime.crest,12);
              ptr += 12;
              memcpy(ptr,&appParam.runtime.velocity,12);
              ptr += 12;
              header->len = 48 + CMD_STRUCT_SZ;
              header->chksum = cmd_checksum(buf,header->len);
              if(appParam.runtime.writefcn)
                appParam.runtime.writefcn(buf,header->len);
            }
            if(opMode == OP_OLED){
              if(nofSampleToIgnore) nofSampleToIgnore--;
              else{
                appParam.runtime.hisPEAK.x = (appParam.runtime.peak.x > appParam.runtime.hisPEAK.x)?appParam.runtime.peak.x:appParam.runtime.hisPEAK.x;
                appParam.runtime.hisPEAK.y = (appParam.runtime.peak.y > appParam.runtime.hisPEAK.y)?appParam.runtime.peak.y:appParam.runtime.hisPEAK.y;
                appParam.runtime.hisPEAK.z = (appParam.runtime.peak.z > appParam.runtime.hisPEAK.z)?appParam.runtime.peak.z:appParam.runtime.hisPEAK.z;
                appParam.runtime.hisRMS.x = (appParam.runtime.rms.x > appParam.runtime.hisRMS.x)?appParam.runtime.rms.x:appParam.runtime.hisRMS.x;
                appParam.runtime.hisRMS.y = (appParam.runtime.rms.y > appParam.runtime.hisRMS.y)?appParam.runtime.rms.y:appParam.runtime.hisRMS.y;
                appParam.runtime.hisRMS.z = (appParam.runtime.rms.z > appParam.runtime.hisRMS.z)?appParam.runtime.rms.z:appParam.runtime.hisRMS.z;
                chEvtSignal(appParam.runtime.dispThread,0x1);
              }
            }
          }
          break;
        }
      }
      A0_LO;
    }
    if(flags & EV_ISM_FIFO_FULL){
      uint16_t sz;
        uint16_t readSz;
        switch(opMode){
        case OP_STREAM:
          if(ism330_cmd_fifo_read(&appParam.runtime.txBuf.buffer[CMD_STRUCT_SZ],300,&readSz)){
            if(packetToIgnore){
              packetToIgnore--;
            }
            else{
              appParam.runtime.txBuf.sz = readSz+CMD_STRUCT_SZ;
              header = (cmd_header_t*)appParam.runtime.txBuf.buffer;
              header->magic1 = MAGIC1;
              header->magic2 = MAGIC2;
              header->type = MASK_DATA | appParam.runtime.lbt;
              header->len = appParam.runtime.txBuf.sz;
              header->pid = pktCount++;
              header->chksum = cmd_checksum(appParam.runtime.txBuf.buffer,header->len);
              if(appParam.runtime.writefcn){
                appParam.runtime.writefcn(appParam.runtime.txBuf.buffer, header->len);
              }
            }
          }
          break;
        case OP_VNODE:
        case OP_OLED:
          if(ism330_cmd_fifo_read(buf,300,&readSz)){
            sz = readSz/12; // nof records
            if(packetToIgnore){
              packetToIgnore--;
            }
            else if(feed_fifo16_imu(&m_timeDomain,(uint8_t*)buf,sz)==1){
              memcpy((void*)&appParam.runtime.rms,(void*)&m_timeDomain.rms,12);
              memcpy((void*)&appParam.runtime.crest,(void*)&m_timeDomain.crest,12);
              memcpy((void*)&appParam.runtime.velocity,(void*)&m_timeDomain.velocity,12);
              appParam.runtime.peak.x = (m_timeDomain.peak.x - m_timeDomain.peakn.x);
              appParam.runtime.peak.y = (m_timeDomain.peak.y - m_timeDomain.peakn.y);
              appParam.runtime.peak.z = (m_timeDomain.peak.z - m_timeDomain.peakn.z);
              resetObject(&m_timeDomain);
              // send data via WIFI/BT
              header = (cmd_header_t*)buf;
              header->magic1 = MAGIC1;
              header->magic2 = MAGIC2;
              header->type = MASK_DATA;
              header->pid = pktCount++;
              uint8_t *ptr = &buf[CMD_STRUCT_SZ];
              memcpy(ptr,&appParam.runtime.peak,12);
              ptr += 12;
              memcpy(ptr,&appParam.runtime.rms,12);
              ptr += 12;
              memcpy(ptr,&appParam.runtime.crest,12);
              ptr += 12;
              memcpy(ptr,&appParam.runtime.velocity,12);
              ptr += 12;
              header->len = 48 + CMD_STRUCT_SZ;
              header->chksum = cmd_checksum(buf,header->len);
              if(appParam.runtime.writefcn){
                appParam.runtime.writefcn(buf,header->len);
              }
            }
          break;          
        }
      }
      
    }
    bStop = chThdShouldTerminateX();
    if(bStop){
      if(activeSensor == SENSOR_ADXL355){
        palDisableLineEvent(LINE_ADXL_INT1);
        adxl355_cmd_stop(&adxl);  
        // disable interrupt
        appParam.runtime.ledBlink.ms_on = 500;
        appParam.runtime.ledBlink.ms_off = 500;
        chEvtUnregister(&appParam.runtime.es_sensor,&elSensor);
      }
      else if(activeSensor == SENSOR_ISM330){
        palDisableLineEvent(LINE_ISM_INT);
        ism330_cmd_stop_config();
        chEvtUnregister(&appParam.runtime.es_sensor,&elSensor);
      }
      appParam.runtime.ledBlink.ms_on = 1000;
      appParam.runtime.ledBlink.ms_off = 1000;
    }
  }
  chThdExit((msg_t)0);
}

static void startTransfer(void)
{

  if(!appParam.runtime.opThread){
    appParam.runtime.opThread = chThdCreateStatic(waOperation,sizeof(waOperation),NORMALPRIO-1,procOperation,NULL);
  }
}

static void stopTransfer(void)
{
  if(appParam.runtime.opThread){
    chThdTerminate(appParam.runtime.opThread);
    chThdWait(appParam.runtime.opThread);
    appParam.runtime.opThread = NULL;
  }
}

void vnode_app_init()
{
  
    if(adxl355_cmd_init(&adxl) == 0){
      appParam.runtime.sensorReady = 1;
    }
}



