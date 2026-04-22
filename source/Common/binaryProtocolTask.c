#include "ch.h"
#include "hal.h"
#include "binaryProtocolTask.h"

#include "database.h"
#include "app_defs.h"
#include <string.h>

#define PORT                    SD1
#define HOST_STX                0x02
#define HOST_ETX                0x03
#define RESPONSE_DELAY_MS       2
#define BIN_PROTOCOL_SIGNATURE  0xAA
#define SIGNATURE_OFFSET        1
#define HOST_FRAME_MAX          128


enum HOST_STREAM_CMD_e{
  HOST_CMD_STREAM_DATA = 0x40
};

enum HOST_CMD_e{
  HOST_CMD_READ_PARAM  = 0x01,
  HOST_CMD_WRITE_PARAM = 0x02,
  HOST_CMD_READ_LIVE   = 0x03,
  HOST_CMD_WRITE_LIVE  = 0x04,
  HOST_CMD_RESET_MCU   = 0x05,
  HOST_CMD_ACK         = 0xF0,
  HOST_CMD_NAK         = 0xF1
};

typedef struct{
  thread_t *rxThread;
  thread_t *mainThread;
  uint8_t baudrate;
}_runTime;

static _runTime binProtoRuntime;

typedef struct{
  uint8_t buffer[HOST_FRAME_MAX];
  uint8_t count;
  uint8_t expected;
}host_parser_t;

static SerialConfig serialCfg={
  115200
};

static uint8_t crc_xor(const uint8_t *ptr, uint8_t size)
{
  uint8_t crc = 0;
  uint8_t i;
  for(i=0;i<size;i++){
    crc ^= ptr[i];
  }
  return crc;
}

static uint16_t be_u16(const uint8_t *ptr)
{
  return (uint16_t)(((uint16_t)ptr[0] << 8) | ptr[1]);
}

static uint32_t be_u32(const uint8_t *ptr)
{
  return ((uint32_t)ptr[0] << 24) | ((uint32_t)ptr[1] << 16) | ((uint32_t)ptr[2] << 8) | ptr[3];
}

static uint32_t le_bytes_to_u32(const uint8_t *ptr, uint8_t size)
{
  uint32_t v = 0;
  if(size > 0){ v |= (uint32_t)ptr[0]; }
  if(size > 1){ v |= (uint32_t)ptr[1] << 8; }
  if(size > 2){ v |= (uint32_t)ptr[2] << 16; }
  if(size > 3){ v |= (uint32_t)ptr[3] << 24; }
  return v;
}

static void u32_to_le_bytes(uint32_t value, uint8_t *ptr, uint8_t size)
{
  if(size > 0){ ptr[0] = (uint8_t)(value & 0xFF); }
  if(size > 1){ ptr[1] = (uint8_t)((value >> 8) & 0xFF); }
  if(size > 2){ ptr[2] = (uint8_t)((value >> 16) & 0xFF); }
  if(size > 3){ ptr[3] = (uint8_t)((value >> 24) & 0xFF); }
}

static void host_send_frame(uint8_t cmd, const uint8_t *payload, uint8_t payload_len)
{
  uint8_t frame[HOST_FRAME_MAX];
  uint8_t len = (uint8_t)(payload_len + 1);
  uint8_t total = (uint8_t)(len + 4);
  if(total > HOST_FRAME_MAX){
    return;
  }

  frame[0] = HOST_STX;
  frame[1] = len;
  frame[2] = cmd;
  if(payload_len > 0){
    memcpy(&frame[3], payload, payload_len);
  }
  frame[3 + payload_len] = crc_xor(&frame[1], (uint8_t)(len + 1));
  frame[4 + payload_len] = HOST_ETX;

  chThdSleepMilliseconds(RESPONSE_DELAY_MS);
  streamWrite((BaseSequentialStream*)&PORT, frame, total);
}

static void host_send_ack(uint16_t id, uint32_t value)
{
  uint8_t payload[6];
  payload[0] = (uint8_t)(id >> 8);
  payload[1] = (uint8_t)(id & 0xFF);
  payload[2] = (uint8_t)(value >> 24);
  payload[3] = (uint8_t)(value >> 16);
  payload[4] = (uint8_t)(value >> 8);
  payload[5] = (uint8_t)(value & 0xFF);
  host_send_frame(HOST_CMD_ACK, payload, sizeof(payload));
}

static void host_send_nak(uint16_t id)
{
  uint8_t payload[2];
  payload[0] = (uint8_t)(id >> 8);
  payload[1] = (uint8_t)(id & 0xFF);
  host_send_frame(HOST_CMD_NAK, payload, sizeof(payload));
}

static void host_handle_read_param(const uint8_t *data, uint8_t size)
{
  uint8_t raw[4] = {0};
  int8_t n;
  uint16_t id;

  if(size != 2){
    host_send_nak(0);
    return;
  }

  id = be_u16(data);
  n = db_read_dataflash(0, 0xFF, id, raw);
  if(n <= 0){
    host_send_nak(id);
    return;
  }

  host_send_ack(id, le_bytes_to_u32(raw, (uint8_t)n));
}

static void host_handle_write_param(const uint8_t *data, uint8_t size)
{
  uint8_t raw[4] = {0};
  int8_t n;
  uint16_t id;
  uint32_t value;

  if(size != 6){
    host_send_nak(0);
    return;
  }

  id = be_u16(data);
  value = be_u32(&data[2]);
  n = db_read_dataflash(0, 0xFF, id, raw);
  if(n <= 0){
    host_send_nak(id);
    return;
  }

  u32_to_le_bytes(value, raw, (uint8_t)n);
  if(db_write_dataflash(0, 0xFF, id, raw) <= 0){
    host_send_nak(id);
    return;
  }

  host_send_ack(id, value);
}

static void host_handle_read_live(const uint8_t *data, uint8_t size)
{
  uint8_t raw[4] = {0};
  int8_t n;
  uint16_t id;

  if(size != 2){
    host_send_nak(0);
    return;
  }

  id = be_u16(data);
  n = db_read_livedata(0, 0xFF, id, raw);
  if(n <= 0){
    host_send_nak(id);
    return;
  }

  host_send_ack(id, le_bytes_to_u32(raw, (uint8_t)n));
}

static void host_handle_write_live(const uint8_t *data, uint8_t size)
{
  uint8_t raw[4] = {0};
  int8_t n;
  uint16_t id;
  uint32_t value;

  if(size != 6){
    host_send_nak(0);
    return;
  }

  id = be_u16(data);
  value = be_u32(&data[2]);
  n = db_read_livedata(0, 0xFF, id, raw);
  if(n <= 0){
    host_send_nak(id);
    return;
  }

  u32_to_le_bytes(value, raw, (uint8_t)n);
  if(db_write_livedata(0, 0xFF, id, raw) <= 0){
    host_send_nak(id);
    return;
  }

  host_send_ack(id, value);
}

static void host_handle_reset_mcu(const uint8_t *data, uint8_t size)
{
  (void)data;

  if(size != 0){
    host_send_nak(0);
    return;
  }

  host_send_ack(0, 0);
  chThdSleepMilliseconds(20);
  NVIC_SystemReset();
}

static void host_dispatch_frame(const uint8_t *frame)
{
  uint8_t len = frame[1];
  uint8_t cmd = frame[2];
  const uint8_t *data = &frame[3];
  uint8_t data_len = (uint8_t)(len - 1);

  switch(cmd){
  case HOST_CMD_READ_PARAM:
    host_handle_read_param(data, data_len);
    break;
  case HOST_CMD_WRITE_PARAM:
    host_handle_write_param(data, data_len);
    break;
  case HOST_CMD_READ_LIVE:
    host_handle_read_live(data, data_len);
    break;
  case HOST_CMD_WRITE_LIVE:
    host_handle_write_live(data, data_len);
    break;
  case HOST_CMD_RESET_MCU:
    host_handle_reset_mcu(data, data_len);
    break;
  default:
    if(data_len >= 2){
      host_send_nak(be_u16(data));
    }
    else{
      host_send_nak(0);
    }
    break;
  }
}

static void host_parser_reset(host_parser_t *parser)
{
  parser->count = 0;
  parser->expected = 0;
}

static void host_parser_feed(host_parser_t *parser, uint8_t byte)
{
  uint8_t len;
  uint8_t expected_crc;
  uint8_t actual_crc;

  if(parser->count == 0){
    if(byte == HOST_STX){
      parser->buffer[0] = byte;
      parser->count = 1;
    }
    return;
  }

  if(parser->count >= HOST_FRAME_MAX){
    host_parser_reset(parser);
    return;
  }

  parser->buffer[parser->count++] = byte;

  if(parser->count == 2){
    len = parser->buffer[1];
    parser->expected = (uint8_t)(len + 4);
    if((parser->expected < 5) || (parser->expected > HOST_FRAME_MAX)){
      host_parser_reset(parser);
      return;
    }
  }

  if((parser->expected > 0) && (parser->count == parser->expected)){
    len = parser->buffer[1];
    if(parser->buffer[parser->expected - 1] == HOST_ETX){
      expected_crc = parser->buffer[parser->expected - 2];
      actual_crc = crc_xor(&parser->buffer[1], (uint8_t)(len + 1));
      if(expected_crc == actual_crc){
        host_dispatch_frame(parser->buffer);
      }
    }
    host_parser_reset(parser);
  }
}

static void init_config()
{
  uint8_t u8;
  
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

static THD_FUNCTION(procHostProtocol, p)
{
  uint8_t byte;
  host_parser_t parser;
  (void)p;
  host_parser_reset(&parser);

  while(!chThdShouldTerminateX())
  {
    if(streamRead((BaseSequentialStream*)&PORT, &byte, 1) == 1){
      host_parser_feed(&parser, byte);
    }
  }
}

void binaryProtocolInit()
{
  init_config();
  binProtoRuntime.baudrate = db_read_df_u8(BAUDRATE);
  switch(binProtoRuntime.baudrate){
  case 0: serialCfg.speed = 9600;break;
  case 1: serialCfg.speed = 19200;break;
  case 2: serialCfg.speed = 38400;break;
  case 3: serialCfg.speed = 57600;break;
  case 4: serialCfg.speed = 115200;break;
  default: serialCfg.speed = 9600;break;
  }
  
  sdStart(&PORT,&serialCfg);
  binProtoRuntime.rxThread = chThdCreateStatic(waShell, sizeof(waShell),NORMALPRIO+1,procHostProtocol,NULL);

  binProtoRuntime.mainThread = chRegFindThreadByName("main");
}

void send_packet(uint8_t *data, uint8_t size)
{
  host_send_frame(HOST_CMD_STREAM_DATA, data, size);
}

void bp_send_packet(uint8_t *data, uint8_t size)
{
  send_packet(data, size);
}
