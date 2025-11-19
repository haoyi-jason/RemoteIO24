#ifndef _DATABASE_H
#define _DATABASE_H

#include "grididea_global.h"

#define NOF_U8_PARAM            64
#define NOF_I8_PARAM            64
#define NOF_U16_PARAM           64
#define NOF_I16_PARAM           64
#define NOF_U32_PARAM           64
#define NOF_I32_PARAM           64
#define NOF_F32_PARAM            64
#define NOF_F8_PARAM            64


#define NVM_FLAG                                0xAB

#define DATA_FLASH_CRC_SECTION_OFFSET	        0
#define DATA_FLASH_CRC_SECTION_SIZE		0x20

#define DATA_FLASH_TYPE_SECTION_SIZE            0x100        
#define DATA_FLASH_TYPE_SECTION_OFFSET(x)       (x*DATA_FLASH_TYPE_SECTION_SIZE +DATA_FLASH_CRC_SECTION_SIZE)

#define DATA_FLASH_BOARD_SECTION_OFFSET	        (DATA_FLASH_CRC_SECTION_OFFSET+DATA_FLASH_CRC_SECTION_SIZE)
#define DATA_FLASH_BOARD_SIZE			0x80

#define DATA_FLASH_PID_SECTION_OFFSET	        DATA_FLASH_BOARD_SECTION_OFFSET+DATA_FLASH_BOARD_SIZE
#define DATA_FLASH_PID_SIZE			0x80

#define DATA_FLASH_AXIS_SECTION_OFFSET	        DATA_FLASH_PID_SECTION_OFFSET+DATA_FLASH_PID_SIZE
#define DATA_FLASH_AXIS_SIZE			0x100

#define DATA_FLASH_CONFIG_SECTION_OFFSET	DATA_FLASH_AXIS_SECTION_OFFSET+DATA_FLASH_AXIS_SIZE
#define DATA_FLASH_CONFIG_SIZE			0x100


//typedef enum{
/*
  parameter based address by type,
  bit [15:12]: type
  bit [11:0]:  index

*/


//  NOF_PARAM_TYPES
//}ParamTypes;


enum NVM_SECTION_BY_TYPE_e{
  NVM_SECTION_U8,
  NVM_SECTION_U16,
  NVM_SECTION_U32,
  NVM_SECTION_I8,
  NVM_SECTION_I16,
  NVM_SECTION_I32,
  NVM_SECTION_F32,
  NVM_SECTION_STORAGE,
  NVM_NOF_SECTIONS
};


typedef struct{
  uint16_t flag;
  uint16_t sections[15];
}crc_param_t;

typedef struct{
  uint32_t hw_version;
  uint32_t sw_version;
  uint32_t serial;
  uint32_t user_id;
  uint32_t function_code;
  int16_t lock_retry_delay_ms;
  uint8_t lock_retry_count;
  int16_t auto_open_trigger_percent;
  int16_t dual_door_gap_percent;
  int16_t dual_door_gap_free_percent;
  uint16_t lock_active_time_ms;
  uint8_t auto_close_wait_time_sec;
  int16_t door_blocked_percent;
  uint8_t door_blocked_sec;
  uint8_t max_action_time_sec;
  uint32_t cycle_time_ms;
  uint16_t action_delay_ms;
  uint16_t lock_delay_ms;
}board_param_t;

typedef struct{
  struct{
    int16_t userOption;
    uint8_t dirReversed;
  }board_data;
  
}live_data_t;
void *db_read_param(uint32_t index);
void db_write_param(uint32_t index, void *ptr);

uint32_t db_read_u32_param(uint16_t index);
void db_write_u32_param(uint16_t index, uint32_t value);
int32_t db_read_i32_param(uint16_t index);
void db_write_i32_param(uint16_t index, int32_t value);

uint16_t db_read_u16_param(uint16_t index);
void  db_write_u16_param(uint16_t index, uint16_t value);
int16_t db_read_i16_param(uint16_t index);
void db_write_i16_param(uint16_t index, int16_t value);

uint8_t db_read_u8_param(uint16_t index);
void db_write_u8_param(uint16_t index, uint8_t value);
int8_t db_read_i8_param(uint16_t index);
void db_write_i8_param(uint16_t index, int8_t value);

float db_read_single_param(uint16_t index);
void db_write_single_param(uint16_t index, float value);

int8_t db_save_section(uint8_t section);


void* db_read_live_data(uint32_t index);
void  db_write_live_data(uint32_t index, void *dptr);

int8_t db_read_dataflash(uint8_t class, uint8_t type,uint16_t address, uint8_t *dptr);
int8_t db_write_dataflash(uint8_t class, uint8_t type,uint16_t address, uint8_t *dptr);

int8_t db_read_df_i8(uint16_t address);
int16_t db_read_df_i16(uint16_t address);
int32_t db_read_df_i32(uint16_t address);
uint8_t db_read_df_u8(uint16_t address);
uint16_t db_read_df_u16(uint16_t address);
uint32_t db_read_df_u32(uint16_t address);
float db_read_df_f32(uint16_t address);

void db_write_df_i8(uint16_t address, int8_t value);
void db_write_df_i16(uint16_t address, int16_t value);
void db_write_df_i32(uint16_t address, int32_t value);
void db_write_df_u8(uint16_t address, uint8_t value);
void db_write_df_u16(uint16_t address, uint16_t value);
void db_write_df_u32(uint16_t address, uint32_t value);
void db_write_df_f32(uint16_t address, float value);
void db_execute_command(uint8_t section, uint8_t index,uint8_t *value);
void database_init();

int8_t db_read_livedata(uint8_t class, uint8_t type,uint16_t address, uint8_t *dptr);
int8_t db_write_livedata(uint8_t class, uint8_t type,uint16_t address, uint8_t *dptr);

int8_t db_read_ld_i8(uint16_t address);
int16_t db_read_ld_i16(uint16_t address);
int32_t db_read_ld_i32(uint16_t address);
uint8_t db_read_ld_u8(uint16_t address);
uint16_t db_read_ld_u16(uint16_t address);
uint32_t db_read_ld_u32(uint16_t address);
float db_read_ld_f32(uint16_t address);

void db_write_ld_i8(uint16_t address, int8_t value);
void db_write_ld_i16(uint16_t address, int16_t value);
void db_write_ld_i32(uint16_t address, int32_t value);
void db_write_ld_u8(uint16_t address, uint8_t value);
void db_write_ld_u16(uint16_t address, uint16_t value);
void db_write_ld_u32(uint16_t address, uint32_t value);
void db_write_ld_f32(uint16_t address, float value);

void db_enable_save_on_write(bool set);
int8_t db_save_section(uint8_t section);

#endif