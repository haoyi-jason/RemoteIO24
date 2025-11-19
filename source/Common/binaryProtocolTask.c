#include "ch.h"
#include "hal.h"
#include "protocol/bin_protocol/bin_protocol.h"
#include "binaryProtocolTask.h"

#include "database.h"
#include "app_defs.h"

#define PORT                    SD1
#define PROTOCOL_SIZE           6
#define RESPONSE_DELAY_MS       2
#define BIN_PROTOCOL_SIGNATURE  0xAA
#define SIGNATURE_OFFSET        1


enum FC_CODE_e{
  FC_DATAFLASH = 0x10,
  FC_LIVEDATA = 0x20,
  FC_COMMAND = 0x30,
  FC_DATA = 0x40
};

typedef struct{
  thread_t *shelltp;
  thread_t *mainThread;
  BINProtocolConfig *config;
  uint8_t baudrate;
}_runTime;

static _runTime binProtoRuntime;

void cmd_data_flash(BaseSequentialStream *chp, uint8_t *data, uint8_t size);
void cmd_live_data(BaseSequentialStream *chp, uint8_t *data, uint8_t size);
void cmd_exec(BaseSequentialStream *chp, uint8_t *data, uint8_t size);
void cmd_dummy(BaseSequentialStream *chp, uint8_t *data, uint8_t size);

static const BINProtocol_Command bin_commands[] = {
  {FC_DATAFLASH, cmd_data_flash},
  {FC_LIVEDATA, cmd_live_data},
  {FC_COMMAND, cmd_exec},
  {0xFF,cmd_dummy},
  {0x00,0x00},
};

static SerialConfig serialCfg={
  115200
};

static BINProtocolConfig bin_config = {
  (BaseSequentialStream*)&PORT,
  0x01,
  bin_commands
};

static void init_config()
{
  uint8_t i,u8;
  uint16_t u16;
  uint32_t u32;
  
  u8 = db_read_df_u8(APP_SIGNATURE+SIGNATURE_OFFSET);
  
  if(u8 != BIN_PROTOCOL_SIGNATURE){
    db_write_df_u8(BOARDID,0x01);
    db_write_df_u8(BAUDRATE,4);
    db_write_df_u8(APP_SIGNATURE+SIGNATURE_OFFSET, BIN_PROTOCOL_SIGNATURE);
    db_save_section(U8);
  }
  else{
  }
}

#define SHELL_WA_SIZE   4096
static THD_WORKING_AREA(waShell,SHELL_WA_SIZE);

void binaryProtocolInit()
{
  init_config();
  uint8_t id = db_read_df_u8(BOARDID);

  bin_config.filter_id = id;

  binProtoRuntime.config = &bin_config;
  binProtoRuntime.baudrate = db_read_df_u8(BAUDRATE);
  binProtoRuntime.baudrate = 4; //force to 115200 for test
  switch(binProtoRuntime.baudrate){
  case 0: serialCfg.speed = 9600;break;
  case 1: serialCfg.speed = 19200;break;
  case 2: serialCfg.speed = 38400;break;
  case 3: serialCfg.speed = 57600;break;
  case 4: serialCfg.speed = 115200;break;
  default: serialCfg.speed = 9600;break;
  }
  
  sdStart(&PORT,&serialCfg);
  binProtoRuntime.shelltp = chThdCreateStatic(waShell, sizeof(waShell),NORMALPRIO+1,binProtocolProc,(void*)&bin_config);

  binProtoRuntime.mainThread = chRegFindThreadByName("main");
}

void cmd_dummy(BaseSequentialStream *chp, uint8_t *data, uint8_t size){}

/*
  cmd_data_flash: read/write data flash by address
  Data Sequence:
  b[0]: 0x00: read, 0x80: write, b[6:0] nof registers
  b[2:1]: u16 address include type
  b[6:3]: value


*/
void cmd_data_flash(BaseSequentialStream *chp, uint8_t *data, uint8_t size)
{
  uint8_t *ptr = data;
  uint8_t buffer[128];
  uint8_t *wptr = buffer;
  uint8_t pkt_sz = size;
  uint16_t address = (data[2]<<8 | data[1]);
  uint8_t nof_reg = data[0] & 0x7F;
  uint8_t rw = data[0] & 0x80;
  uint8_t i;
  if(rw == 0x00){ // write 
    int8_t szWrite = 0;
    uint8_t sz_to_write = pkt_sz - 2;
    wptr = &data[3];
    db_enable_save_on_write(false);
    for(i=0;i<nof_reg;i++){
      szWrite = db_write_dataflash(0,0xff,address, wptr);
      sz_to_write -= szWrite;      
      address += szWrite;
      wptr += szWrite;
      if(sz_to_write == 0) break;
    }
    db_enable_save_on_write(true);
  }
  
  wptr = buffer;
  *wptr++ = START_PREFIX;
  *wptr++ = FC_DATAFLASH;
  *wptr++ = bin_config.filter_id;
  *wptr++;  // reserve for packet length
  *wptr++ = data[0]; // rw + nof reg
  *wptr++ = data[1];
  *wptr++ = data[2];
  
  uint8_t sz_to_read = pkt_sz - 2;
  int8_t sz_read = 0;
  address = (data[2]<<8 | data[1]);
  for(i=0;i<nof_reg;i++){
    sz_read = db_read_dataflash(0,0xff,address,wptr);
    address += sz_read;
    wptr+= sz_read;
  }
  buffer[3] = (wptr - &buffer[3]-1);
  *wptr++ = crc8(&buffer[1],(wptr-buffer-2));
  *wptr = END_PREFIX;
  chThdSleepMilliseconds(RESPONSE_DELAY_MS);
  streamWrite(chp, buffer, wptr-buffer+1);
}

void cmd_live_data(BaseSequentialStream *chp, uint8_t *data, uint8_t size)
{
  uint8_t *ptr = data;
  uint8_t buffer[128];
  uint8_t *wptr = buffer;
  uint8_t pkt_sz = size;
  uint16_t address = (data[2]<<8 | data[1]);
  uint8_t nof_reg = data[0] & 0x7F;
  uint8_t rw = data[0] & 0x80;
  uint8_t i;
  if(rw == 0x00){ // write 
    int8_t szWrite = 0;
    uint8_t sz_to_write = pkt_sz - 2;
    wptr = &data[3];
    db_enable_save_on_write(false);
    for(i=0;i<nof_reg;i++){
      szWrite = db_write_livedata(0,0xff,address, wptr);
      sz_to_write -= szWrite;      
      address += szWrite;
      wptr += szWrite;
      if(sz_to_write == 0) break;
    }
    db_enable_save_on_write(true);
  }
  
  wptr = buffer;
  *wptr++ = START_PREFIX;
  *wptr++ = FC_DATAFLASH;
  *wptr++ = bin_config.filter_id;
  *wptr++;  // reserve for packet length
  *wptr++ = data[0]; // rw + nof reg
  *wptr++ = data[1];
  *wptr++ = data[2];
  
  uint8_t sz_to_read = pkt_sz - 2;
  int8_t sz_read = 0;
  for(i=0;i<nof_reg;i++){
    sz_read = db_read_livedata(0,0,address,wptr);
    address += sz_read;
    wptr+= sz_read;
  }
  buffer[3] = (wptr - &buffer[3]-1);
  *wptr++ = crc8(&buffer[1],(wptr-buffer-2));
  *wptr = END_PREFIX;
  chThdSleepMilliseconds(RESPONSE_DELAY_MS);
  streamWrite(chp, buffer, wptr-buffer+1);
}

void cmd_exec(BaseSequentialStream *chp, uint8_t *data, uint8_t size)
{
  uint8_t *ptr = data;
  uint16_t command = (data[1]<<8 | data[0]);
  
  if(binProtoRuntime.mainThread != NULL){
    chEvtSignal(binProtoRuntime.mainThread, EVENT_MASK(command));
  }
}

void send_packet(uint8_t *data, uint8_t size)
{
  uint8_t buffer[128];
  uint8_t *wptr = buffer;
  *wptr++ = START_PREFIX;
  *wptr++ = FC_DATA;
  *wptr++ = bin_config.filter_id;
  *wptr++ = size;  // reserve for packet length
  memcpy(wptr,data,size);
  wptr += size;
  *wptr++ = crc8(&buffer[1],(wptr-buffer-2));
  *wptr = END_PREFIX;
  //chThdSleepMilliseconds(RESPONSE_DELAY_MS);
  streamWrite(&PORT, buffer, wptr-buffer+1);
  
}
