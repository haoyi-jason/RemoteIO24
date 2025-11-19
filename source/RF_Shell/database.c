#include "hal.h"
#include "database.h"
#include "../drivers/nvm.h"
#include "ylib/encryption/crc16/crc16-ccitt.h"
struct nvmParam_s nvmParam2;

static void load_nvm_default()
{
  nvmParam2.board_version = BOARD_ID;
  nvmParam2.fw_version = FW_VERSION;
  nvmParam2.device_config.selfAddr = 0x02;
  nvmParam2.device_config.destAddr = 0x01;
  nvmParam2.device_config.opMode = 0x00;
  nvmParam2.device_config.timeoutMs = 9000;
  nvmParam2.device_config.txIntervalMs = 50;
  nvmParam2.device_config.txMask = 0xFFFF;
  nvmParam2.device_config.txPattern = 0xC000;
  nvmParam2.device_config.txPower = 4;
  nvmParam2.device_config.frequency = 910.00;
  nvmParam2.device_config.dataRate = 1;
  
  nvmParam2.map_a[0].x = 0;
  nvmParam2.map_a[0].y = 800;
  nvmParam2.map_a[1].x = 30;
  nvmParam2.map_a[1].y = 2600;
  nvmParam2.map_a[2].x = 170;
  nvmParam2.map_a[2].y = 3600;
  nvmParam2.map_a[3].x = 200;
  nvmParam2.map_a[3].y = 4000;
  
  nvmParam2.pulseDegree = (320*10)/360.;
  nvmParam2.min_pps = 500;
  nvmParam2.max_pps = 1500;
  
  nvmParam2.map_b[0].x = 200;
  nvmParam2.map_b[0].y = 125;
  nvmParam2.map_b[1].x = 3300;
  nvmParam2.map_b[1].y = 250;
  nvmParam2.map_b[2].x = 3800;
  nvmParam2.map_b[2].y = 300;
  nvmParam2.map_b[3].x = 4000;
  nvmParam2.map_b[3].y = 500;
  
  
}

static void save_nvm()
{
  uint8_t *ptr = (void*)&nvmParam2.crc16;
  ptr += 2; // offset
  uint16_t sz = sizeof(struct nvmParam_s)-2;
  uint16_t crc_cal = crc16_ccitt(ptr,sz,0x0);
  nvmParam2.crc16 = crc_cal;
  nvm_flash_write(OFFSET_NVM_BOARD,(uint8_t*)&nvmParam2,sz);
}

void database_init(void)
{
  uint8_t *ptr = (void*)&nvmParam2.crc16;
  ptr += 2; // offset
  uint16_t sz = sizeof(struct nvmParam_s)-2;
  nvm_flash_read(OFFSET_NVM_BOARD,(uint8_t*)&nvmParam2,sz);
  
  uint16_t crc_cal = crc16_ccitt(ptr,sz,0x0);
  if(nvmParam2.crc16 != crc_cal){
    load_nvm_default();
    save_nvm();
  }
  // comment following line to use stored parameters
  load_nvm_default();  
}