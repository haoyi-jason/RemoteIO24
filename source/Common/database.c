#include "ch.h"
#include "hal.h"
#include "database.h"
#include "encryption/crc16/crc16-ccitt.h"
#include <string.h>
#include "at32_flash/nvm.h"
#include "app_defs.h"

#define NVM_READ        nvm_flash_read
#define NVM_WRITE       nvm_flash_write

typedef struct{
    crc_param_t crc;
    struct{
      int32_t i32_param[NOF_I32_PARAM];
      uint32_t u32_param[NOF_U32_PARAM];
      int16_t i16_param[NOF_I16_PARAM];
      uint16_t u16_param[NOF_U16_PARAM];
      int8_t i8_param[NOF_I8_PARAM];
      uint8_t u8_param[NOF_U8_PARAM];
      float f32_param[NOF_F32_PARAM];
    }DataFlash;    
    struct{
      int32_t i32_param[NOF_I32_PARAM];
      uint32_t u32_param[NOF_U32_PARAM];
      int16_t i16_param[NOF_I16_PARAM];
      uint16_t u16_param[NOF_U16_PARAM];
      int8_t i8_param[NOF_I8_PARAM];
      uint8_t u8_param[NOF_U8_PARAM];
      float f32_param[NOF_F32_PARAM];
    }LiveData;    
}_db_runtime_t;

static _db_runtime_t db_runtime;
static live_data_t liveData;
static bool save_on_write = true;

void db_default_section(uint8_t section);

static void load_dataflash_default(uint8_t type)
{
  switch(type)
  {
  case NVM_SECTION_I32:
//    db_runtime.DataFlash.i32_param[PARAM_INDEX(USER_RAW_OFFSET_CHANNEL_1)] = 0;
//    db_runtime.DataFlash.i32_param[PARAM_INDEX(USER_RAW_OFFSET_CHANNEL_2)] = 0;
//    db_runtime.DataFlash.i32_param[PARAM_INDEX(USER_RAW_OFFSET_CHANNEL_3)] = 0;
//    db_runtime.DataFlash.i32_param[PARAM_INDEX(USER_RAW_OFFSET_CHANNEL_4)] = 0;
//    db_runtime.DataFlash.i32_param[PARAM_INDEX(USER_RAW_OFFSET_CHANNEL_5)] = 0;
//    db_runtime.DataFlash.i32_param[PARAM_INDEX(USER_RAW_OFFSET_CHANNEL_6)] = 0;
//    db_runtime.DataFlash.i32_param[PARAM_INDEX(USER_RAW_OFFSET_CHANNEL_7)] = 0;
//    db_runtime.DataFlash.i32_param[PARAM_INDEX(USER_RAW_OFFSET_CHANNEL_8)] = 0;
//    db_runtime.DataFlash.i32_param[PARAM_INDEX(USER_RAW_OFFSET_CHANNEL_9)] = 0;
//    db_runtime.DataFlash.i32_param[PARAM_INDEX(USER_RAW_OFFSET_CHANNEL_10)] = 0;
//    db_runtime.DataFlash.i32_param[PARAM_INDEX(USER_RAW_OFFSET_CHANNEL_11)] = 0;
//    db_runtime.DataFlash.i32_param[PARAM_INDEX(USER_RAW_OFFSET_CHANNEL_12)] = 0;
//    db_runtime.DataFlash.i32_param[PARAM_INDEX(USER_RAW_OFFSET_CHANNEL_13)] = 0;
//    db_runtime.DataFlash.i32_param[PARAM_INDEX(USER_RAW_OFFSET_CHANNEL_14)] = 0;
//    db_runtime.DataFlash.i32_param[PARAM_INDEX(USER_RAW_OFFSET_CHANNEL_15)] = 0;
//    db_runtime.DataFlash.i32_param[PARAM_INDEX(USER_RAW_OFFSET_CHANNEL_16)] = 0;
    break;
  case NVM_SECTION_U32:
//    db_runtime.DataFlash.u32_param[PARAM_INDEX(BOARD_SECTION_HW_VERSION)] = HW_VERSION;
//    db_runtime.DataFlash.u32_param[PARAM_INDEX(BOARD_SECTION_FW_VERSION)] = FW_VERSION;
//    db_runtime.DataFlash.u32_param[PARAM_INDEX(BOARD_SECTION_USERID)] = 0x01;
//    db_runtime.DataFlash.u32_param[PARAM_INDEX(BOARD_SECTION_FUNCTION_CODE)] = 0x0;
//    db_runtime.DataFlash.u32_param[PARAM_INDEX(AD7124_FILTER_CONFIG_1)] = 0x0;
//    db_runtime.DataFlash.u32_param[PARAM_INDEX(AD7124_FILTER_CONFIG_2)] = 0x0;
//    db_runtime.DataFlash.u32_param[PARAM_INDEX(AD7124_FILTER_CONFIG_3)] = 0x0;
//    db_runtime.DataFlash.u32_param[PARAM_INDEX(AD7124_FILTER_CONFIG_4)] = 0x0;
//    db_runtime.DataFlash.u32_param[PARAM_INDEX(AD7124_FILTER_CONFIG_5)] = 0x0;
//    db_runtime.DataFlash.u32_param[PARAM_INDEX(AD7124_FILTER_CONFIG_6)] = 0x0;
//    db_runtime.DataFlash.u32_param[PARAM_INDEX(AD7124_FILTER_CONFIG_7)] = 0x0;
//    db_runtime.DataFlash.u32_param[PARAM_INDEX(AD7124_FILTER_CONFIG_8)] = 0x0;
    break;
  case NVM_SECTION_I16:
//    db_runtime.DataFlash.i16_param[PARAM_INDEX(BOARD_SECTION_AUTO_OPEN_TRIGGER_PERCENT)] = 50;
//    db_runtime.DataFlash.i16_param[PARAM_INDEX(BOARD_SECTION_DUAL_DOOR_GAP_PERCENT)] = 100;
//    db_runtime.DataFlash.i16_param[PARAM_INDEX(BOARD_SECTION_DOOR_FREE_ANGLE)] = 150;
//    db_runtime.DataFlash.i16_param[PARAM_INDEX(BOARD_SECTION_DOOR_BLOCKED_PERCENT)] = 30;  
    break;
  case NVM_SECTION_U16:
//    db_runtime.DataFlash.u16_param[PARAM_INDEX(AD7124_SETUP_CCONFIG_1)] = 0;
//    db_runtime.DataFlash.u16_param[PARAM_INDEX(AD7124_SETUP_CCONFIG_2)] = 0;
//    db_runtime.DataFlash.u16_param[PARAM_INDEX(AD7124_SETUP_CCONFIG_3)] = 0;
//    db_runtime.DataFlash.u16_param[PARAM_INDEX(AD7124_SETUP_CCONFIG_4)] = 0;
//    db_runtime.DataFlash.u16_param[PARAM_INDEX(AD7124_SETUP_CCONFIG_5)] = 0;
//    db_runtime.DataFlash.u16_param[PARAM_INDEX(AD7124_SETUP_CCONFIG_6)] = 0;
//    db_runtime.DataFlash.u16_param[PARAM_INDEX(AD7124_SETUP_CCONFIG_7)] = 0;
//    db_runtime.DataFlash.u16_param[PARAM_INDEX(AD7124_SETUP_CCONFIG_8)] = 0;
//    db_runtime.DataFlash.u16_param[PARAM_INDEX(AD7124_CONTROL_CONFIG_0)] = 0;
//    db_runtime.DataFlash.u16_param[PARAM_INDEX(AD7124_CONTROL_CONFIG_1)] = 0;
//    db_runtime.DataFlash.u16_param[PARAM_INDEX(AD7124_CONTROL_CONFIG_2)] = 0;
//    db_runtime.DataFlash.u16_param[PARAM_INDEX(AD7124_CONTROL_CONFIG_3)] = 0;
//    db_runtime.DataFlash.u16_param[PARAM_INDEX(AD7124_CONTROL_CONFIG_4)] = 0;
//    db_runtime.DataFlash.u16_param[PARAM_INDEX(AD7124_CONTROL_CONFIG_5)] = 0;
//    db_runtime.DataFlash.u16_param[PARAM_INDEX(AD7124_CONTROL_CONFIG_6)] = 0;
//    db_runtime.DataFlash.u16_param[PARAM_INDEX(AD7124_CONTROL_CONFIG_7)] = 0;
    break;
  case NVM_SECTION_I8:
    break;
  case NVM_SECTION_U8:
//    db_runtime.DataFlash.u8_param[PARAM_INDEX(BOARDID)] = 0x01;
//    db_runtime.DataFlash.u8_param[PARAM_INDEX(BAUDRATE)] = 0;
//    db_runtime.DataFlash.u8_param[PARAM_INDEX(APP_CONFIG_ADC_CHANNEL_ENABLE_MASK)] = 0xFF;
//    for(uint8_t i=0;i<16;i++){
//      db_runtime.DataFlash.u8_param[PARAM_INDEX(INPUT_SIGNAL_TYPE_CH_1+i)] = 0x01;
//      db_runtime.DataFlash.u8_param[PARAM_INDEX(OUTPUT_ALARM_TYPE_CH_1+i)] = 0x01;
//    }
    break;
  case NVM_SECTION_F32:
//    db_runtime.DataFlash.f32_param[PARAM_INDEX(ENG_GAIN_CHANNEL_1)] = 0.0;
//    db_runtime.DataFlash.f32_param[PARAM_INDEX(ENG_GAIN_CHANNEL_2)] = 0.0;
//    db_runtime.DataFlash.f32_param[PARAM_INDEX(ENG_GAIN_CHANNEL_3)] = 0.0;
//    db_runtime.DataFlash.f32_param[PARAM_INDEX(ENG_GAIN_CHANNEL_4)] = 0.0;
//    db_runtime.DataFlash.f32_param[PARAM_INDEX(ENG_GAIN_CHANNEL_5)] = 0.0;
//    db_runtime.DataFlash.f32_param[PARAM_INDEX(ENG_GAIN_CHANNEL_6)] = 0.0;
//    db_runtime.DataFlash.f32_param[PARAM_INDEX(ENG_GAIN_CHANNEL_7)] = 0.0;
//    db_runtime.DataFlash.f32_param[PARAM_INDEX(ENG_GAIN_CHANNEL_8)] = 0.0;
//    db_runtime.DataFlash.f32_param[PARAM_INDEX(ENG_GAIN_CHANNEL_9)] = 0.0;
//    db_runtime.DataFlash.f32_param[PARAM_INDEX(ENG_GAIN_CHANNEL_10)] = 0.0;
//    db_runtime.DataFlash.f32_param[PARAM_INDEX(ENG_GAIN_CHANNEL_11)] = 0.0;
//    db_runtime.DataFlash.f32_param[PARAM_INDEX(ENG_GAIN_CHANNEL_12)] = 0.0;
//    db_runtime.DataFlash.f32_param[PARAM_INDEX(ENG_GAIN_CHANNEL_13)] = 0.0;
//    db_runtime.DataFlash.f32_param[PARAM_INDEX(ENG_GAIN_CHANNEL_14)] = 0.0;
//    db_runtime.DataFlash.f32_param[PARAM_INDEX(ENG_GAIN_CHANNEL_15)] = 0.0;
//    db_runtime.DataFlash.f32_param[PARAM_INDEX(ENG_GAIN_CHANNEL_16)] = 0.0;
    break;
  default:break;
  }  
}


static void save_type_param(uint8_t type)
{
  uint32_t sz = 0;
  uint16_t crc_cal = 0;
  uint8_t *ptr = 0x0;
  uint32_t offset = 0;
  switch(type)
  {
  case NVM_SECTION_I32:
    sz=4*NOF_I32_PARAM;
    
    ptr = (void*)db_runtime.DataFlash.i32_param;
    crc_cal = crc16_ccitt((void*)db_runtime.DataFlash.i32_param,sz,0x0);
    break;
  case NVM_SECTION_U32:
    sz=4*NOF_U32_PARAM;
    ptr = (void*)db_runtime.DataFlash.u32_param;
    crc_cal = crc16_ccitt((void*)db_runtime.DataFlash.u32_param,sz,0x0);
    break;
  case NVM_SECTION_I16:
    sz=2*NOF_I32_PARAM;
    ptr = (void*)db_runtime.DataFlash.i16_param;
    crc_cal = crc16_ccitt((void*)db_runtime.DataFlash.i16_param,sz,0x0);
    break;
  case NVM_SECTION_U16:
    sz=2*NOF_U16_PARAM;
    ptr = (void*)db_runtime.DataFlash.u16_param;
    crc_cal = crc16_ccitt((void*)db_runtime.DataFlash.u16_param,sz,0x0);
    break;
  case NVM_SECTION_I8:
    sz=NOF_I8_PARAM;
    ptr = (void*)db_runtime.DataFlash.i8_param;
    crc_cal = crc16_ccitt((void*)db_runtime.DataFlash.i8_param,sz,0x0);
    break;
  case NVM_SECTION_U8:
    sz=NOF_U8_PARAM;
    ptr = (void*)db_runtime.DataFlash.u8_param;
    crc_cal = crc16_ccitt((void*)db_runtime.DataFlash.u8_param,sz,0x0);
    break;
  case NVM_SECTION_F32:
    sz=4*NOF_F32_PARAM;
    ptr = (void*)db_runtime.DataFlash.f32_param;
    crc_cal = crc16_ccitt((void*)db_runtime.DataFlash.f32_param,sz,0x0);
    break;
  default:break;
  }
  
  db_runtime.crc.sections[type] = crc_cal;
  NVM_WRITE(DATA_FLASH_CRC_SECTION_OFFSET,(uint8_t*)&db_runtime.crc,DATA_FLASH_CRC_SECTION_SIZE);
  NVM_WRITE(DATA_FLASH_TYPE_SECTION_OFFSET(type),ptr,DATA_FLASH_TYPE_SECTION_SIZE);

}

static void load_type_param(uint8_t type, uint16_t crc_check)
{
  uint32_t sz = 0;
  uint16_t crc_cal = 0;
  uint8_t *ptr = 0x0;
  //uint32_t offset = 0;
  switch(type)
  {
  case NVM_SECTION_I32:
    sz=4*NOF_I32_PARAM;
    ptr = (void*)db_runtime.DataFlash.i32_param;
    //crc_cal = crc16_ccitt((void*)db_runtime.DataFlash.i32_param,sz,0x0);
    break;
  case NVM_SECTION_U32:
    sz=4*NOF_U32_PARAM;
    ptr = (void*)db_runtime.DataFlash.u32_param;
    //crc_cal = crc16_ccitt((void*)db_runtime.DataFlash.u32_param,sz,0x0);
    break;
  case NVM_SECTION_I16:
    sz=2*NOF_I16_PARAM;
    ptr = (void*)db_runtime.DataFlash.i16_param;
    //crc_cal = crc16_ccitt((void*)db_runtime.DataFlash.i16_param,sz,0x0);
    break;
  case NVM_SECTION_U16:
    sz=2*NOF_U16_PARAM;
    ptr = (void*)db_runtime.DataFlash.u16_param;
    //crc_cal = crc16_ccitt((void*)db_runtime.DataFlash.u16_param,sz,0x0);
    break;
  case NVM_SECTION_I8:
    sz=NOF_I8_PARAM;
    ptr = (void*)db_runtime.DataFlash.i8_param;
    //crc_cal = crc16_ccitt((void*)db_runtime.DataFlash.i8_param,sz,0x0);
    break;
  case NVM_SECTION_U8:
    sz=NOF_U8_PARAM;
    ptr = (void*)db_runtime.DataFlash.u8_param;
    //crc_cal = crc16_ccitt((void*)db_runtime.DataFlash.u8_param,sz,0x0);
    break;
  case NVM_SECTION_F32:
    sz=4*NOF_F32_PARAM;
    ptr = (void*)db_runtime.DataFlash.f32_param;
    //crc_cal = crc16_ccitt((void*)db_runtime.DataFlash.f32_param,sz,0x0);
    break;
  default:break;
  }
  
  NVM_READ(DATA_FLASH_TYPE_SECTION_OFFSET(type),ptr,sz);
  crc_cal = crc16_ccitt(ptr,sz,0x0);
  
  
  if(crc_check != crc_cal){
    load_dataflash_default(type);
    save_type_param(type);
  }
}


void database_init()
{
  NVM_READ(DATA_FLASH_CRC_SECTION_OFFSET,(void*)&db_runtime.crc,DATA_FLASH_CRC_SECTION_SIZE);
  
  uint16_t crc_cal = 0x0;
  if(db_runtime.crc.flag == NVM_FLAG){
    for(uint8_t i=0;i<NVM_NOF_SECTIONS;i++){
      load_type_param(i,db_runtime.crc.sections[i]);
    }
  }
  else{
      db_runtime.crc.flag = NVM_FLAG;
      for(uint8_t i=0;i<NVM_NOF_SECTIONS;i++){
        load_dataflash_default(i);
        save_type_param(i);
      }
  }
  
 
  // other initialization
}

uint32_t db_read_u32_param(uint16_t index)
{
  if(index < NOF_U32_PARAM){
    return db_runtime.DataFlash.u32_param[index];
  }
  return 0;
}

void db_write_u32_param(uint16_t index, uint32_t value)
{
  if(index < NOF_U32_PARAM){
    db_runtime.DataFlash.u32_param[index] = value;
  }
}

int32_t db_read_i32_param(uint16_t index)
{
  if(index < NOF_I32_PARAM){
    return db_runtime.DataFlash.i32_param[index];
  }
  return 0;
}

void db_write_i32_param(uint16_t index, int32_t value)
{
  if(index < NOF_I32_PARAM){
    db_runtime.DataFlash.i32_param[index] = value;
  }
}

uint16_t db_read_u16_param(uint16_t index)
{
  if(index < NOF_U16_PARAM){
    return db_runtime.DataFlash.u16_param[index];
  }
  return 0;
}

void db_write_u16_param(uint16_t index, uint16_t value)
{
  if(index < NOF_U16_PARAM){
    db_runtime.DataFlash.u16_param[index] = value;
  }
}

int16_t db_read_i16_param(uint16_t index)
{
  if(index < NOF_I16_PARAM){
    return db_runtime.DataFlash.i16_param[index];
  }
  return 0;
}

void db_write_i16_param(uint16_t index, int16_t value)
{
  if(index < NOF_I16_PARAM){
    db_runtime.DataFlash.i16_param[index] = value;
  }
}

uint8_t db_read_u8_param(uint16_t index)
{
  if(index < NOF_U8_PARAM){
    return db_runtime.DataFlash.u8_param[index];
  }
  return 0;
}

void db_write_u8_param(uint16_t index, uint8_t value)
{
  if(index < NOF_U8_PARAM){
    db_runtime.DataFlash.u8_param[index] = value;
  }
}

int8_t db_read_i8_param(uint16_t index)
{
  if(index < NOF_I8_PARAM){
    return db_runtime.DataFlash.i8_param[index];
  }
  return 0;
}

void db_write_i8_param(uint16_t index, int8_t value)
{
  if(index < NOF_I8_PARAM){
    db_runtime.DataFlash.i8_param[index] = value;
  }
}

float db_read_single_param(uint16_t index)
{
  if(index < NOF_F32_PARAM){
    return db_runtime.DataFlash.f32_param[index];
  }
  return 0;
}

void db_write_single_param(uint16_t index, float value)
{
  if(index < NOF_F32_PARAM){
    db_runtime.DataFlash.f32_param[index] = value;
  }
}

int8_t db_save_section(uint8_t section)
{
  if(section == 0xff){
    for(uint8_t i=0;i<NVM_NOF_SECTIONS;i++){
      save_type_param(i);
    }
  }
  else{
    save_type_param(section);
  }
  return 0;
}



void *db_read_param(uint32_t index)
{

    return 0x0;
}

void db_write_param(uint32_t index, void *ptr)
{
  
}

int8_t db_read_dataflash(uint8_t class, uint8_t type,uint16_t address, uint8_t *dptr)
{
	int8_t sz = 0;
	uint8_t ptype = type;
	uint8_t *sptr = 0;
	uint16_t index = PARAM_INDEX(address);
	(void)class;
	if(type == 0xff){ // read internal, using hi 4-bit for type
		ptype = (address >> 12);
	}
	switch(ptype){
	case U8: sz = 1; sptr = (uint8_t*)&db_runtime.DataFlash.u8_param[index];break;
	case U16: sz = 2; sptr = (uint8_t*)&db_runtime.DataFlash.u16_param[index];break;
	case U32: sz = 4; sptr = (uint8_t*)&db_runtime.DataFlash.u32_param[index];break;
	case I8: sz = 1; sptr = (uint8_t*)&db_runtime.DataFlash.i8_param[index];break;
	case I16: sz = 2; sptr = (uint8_t*)&db_runtime.DataFlash.i16_param[index];break;
	case I32: sz = 4; sptr = (uint8_t*)&db_runtime.DataFlash.i32_param[index];break;
	case F32: sz = 4; sptr = (uint8_t*)&db_runtime.DataFlash.f32_param[index];break;
	default:break;
	}

	if(sz > 0){
		memcpy(dptr,sptr,sz);
	}
	return sz;
}
int8_t db_write_dataflash(uint8_t class, uint8_t type,uint16_t address, uint8_t *dptr)
{
	int8_t ret = -1;
	int8_t sz = 0;
	uint8_t ptype = type;
	uint8_t *sptr = 0;
	uint16_t index = PARAM_INDEX(address);
	(void)class;
	if(type == 0xff){ // read internal, using hi 4-bit for type
		ptype = (address >> 12);
	}
	switch(ptype){
	case U8: sz = 1; sptr = (uint8_t*)&db_runtime.DataFlash.u8_param[index];break;
	case U16: sz = 2; sptr = (uint8_t*)&db_runtime.DataFlash.u16_param[index];break;
	case U32: sz = 4; sptr = (uint8_t*)&db_runtime.DataFlash.u32_param[index];break;
	case I8: sz = 1; sptr = (uint8_t*)&db_runtime.DataFlash.i8_param[index];break;
	case I16: sz = 2; sptr = (uint8_t*)&db_runtime.DataFlash.i16_param[index];break;
	case I32: sz = 4; sptr = (uint8_t*)&db_runtime.DataFlash.i32_param[index];break;
	case F32: sz = 4; sptr = (uint8_t*)&db_runtime.DataFlash.f32_param[index];break;
	default:break;
	}

	if(sz > 0){
		memcpy(sptr,dptr,sz);
                if(save_on_write)
                  db_save_section(ptype);
	}

	return sz;
}

int8_t db_read_df_i8(uint16_t address)
{
	int8_t ret = 0;
	if(PARAM_TYPE(address) == I8)
	{
		ret = db_runtime.DataFlash.i8_param[PARAM_INDEX(address)];
	}
	else{
		while(1);
	}
	return ret;
}
int16_t db_read_df_i16(uint16_t address)
{
	int16_t ret = 0;
	if(PARAM_TYPE(address) == I16)
	{
		ret = db_runtime.DataFlash.i16_param[PARAM_INDEX(address)];
	}
	else{
		while(1);
	}
	return ret;
}
int32_t db_read_df_i32(uint16_t address)
{
	int32_t ret = 0;
	if(PARAM_TYPE(address) == I32)
	{
		ret = db_runtime.DataFlash.i32_param[PARAM_INDEX(address)];
	}
	else{
		while(1);
	}
	return ret;
}
uint8_t db_read_df_u8(uint16_t address)
{
	uint8_t ret = 0;
	if(PARAM_TYPE(address) == U8)
	{
		ret = db_runtime.DataFlash.u8_param[PARAM_INDEX(address)];
	}
	else{
		while(1);
	}
	return ret;
}
uint16_t db_read_df_u16(uint16_t address)
{
	uint16_t ret = 0;
	if(PARAM_TYPE(address) == U16)
	{
		ret = db_runtime.DataFlash.u16_param[PARAM_INDEX(address)];
	}
	else{
		while(1);
	}
	return ret;
}
uint32_t db_read_df_u32(uint16_t address)
{
	uint32_t ret = 0;
	if(PARAM_TYPE(address) == U32)
	{
		ret = db_runtime.DataFlash.u32_param[PARAM_INDEX(address)];
	}
	else{
		while(1);
	}
	return ret;
}
float db_read_df_f32(uint16_t address)
{
	float ret = 0;
	if(PARAM_TYPE(address) == F32)
	{
		ret = db_runtime.DataFlash.f32_param[PARAM_INDEX(address)];
	}
	else{
		while(1);
	}
	return ret;
}

void db_write_df_i8(uint16_t address, int8_t value)
{
	if(PARAM_TYPE(address) == I8)
	{
		db_runtime.DataFlash.i8_param[PARAM_INDEX(address)] = value;
	}
}
void db_write_df_i16(uint16_t address, int16_t value)
{
	if(PARAM_TYPE(address) == I16)
	{
		db_runtime.DataFlash.i16_param[PARAM_INDEX(address)] = value;
	}
}
void db_write_df_i32(uint16_t address, int32_t value)
{
	if(PARAM_TYPE(address) == I32)
	{
		db_runtime.DataFlash.i32_param[PARAM_INDEX(address)] = value;
	}
}
void db_write_df_u8(uint16_t address, uint8_t value)
{
	if(PARAM_TYPE(address) == U8)
	{
		db_runtime.DataFlash.u8_param[PARAM_INDEX(address)] = value;
	}
}
void db_write_df_u16(uint16_t address, uint16_t value)
{
	if(PARAM_TYPE(address) == U16)
	{
		db_runtime.DataFlash.u16_param[PARAM_INDEX(address)] = value;
	}
}
void db_write_df_u32(uint16_t address, uint32_t value)
{
	if(PARAM_TYPE(address) == U32)
	{
		db_runtime.DataFlash.u32_param[PARAM_INDEX(address)] = value;
	}
}
void db_write_df_f32(uint16_t address, float value)
{
	if(PARAM_TYPE(address) == F32)
	{
		db_runtime.DataFlash.f32_param[PARAM_INDEX(address)] = value;
	}
}

/// LIVE dATA
int8_t db_read_livedata(uint8_t class, uint8_t type,uint16_t address, uint8_t *dptr)
{
	int8_t sz = 0;
	uint8_t ptype = type;
	uint8_t *sptr = 0;
	uint16_t index = PARAM_INDEX(address);
	(void)class;
	if(type == 0xff){ // read internal, using hi 4-bit for type
		ptype = (address >> 12);
	}
	switch(ptype){
	case U8: sz = 1; sptr = ( uint8_t*)&db_runtime.LiveData.u8_param[index];break;
	case U16: sz = 2; sptr = (uint8_t*)&db_runtime.LiveData.u16_param[index];break;
	case U32: sz = 4; sptr = (uint8_t*)&db_runtime.LiveData.u32_param[index];break;
	case I8: sz = 1; sptr =  (uint8_t*)&db_runtime.LiveData.i8_param[index];break;
	case I16: sz = 2; sptr = (uint8_t*)&db_runtime.LiveData.i16_param[index];break;
	case I32: sz = 4; sptr = (uint8_t*)&db_runtime.LiveData.i32_param[index];break;
	case F32: sz = 4; sptr = (uint8_t*)&db_runtime.LiveData.f32_param[index];break;
	default:break;
	}

	if(sz > 0){
		memcpy(dptr,sptr,sz);
	}
	return sz;
}
int8_t db_write_livedata(uint8_t class, uint8_t type,uint16_t address, uint8_t *dptr)
{
	int8_t ret = -1;
	int8_t sz = 0;
	uint8_t ptype = type;
	uint8_t *sptr = 0;
	uint16_t index = PARAM_INDEX(address);
	(void)class;
	if(type == 0xff){ // read internal, using hi 4-bit for type
		ptype = (address >> 12);
	}
	switch(ptype){
	case U8: sz = 1; sptr =  (uint8_t*)&db_runtime.LiveData.u8_param[index];break;
	case U16: sz = 2; sptr = (uint8_t*)&db_runtime.LiveData.u16_param[index];break;
	case U32: sz = 4; sptr = (uint8_t*)&db_runtime.LiveData.u32_param[index];break;
	case I8: sz = 1; sptr =  (uint8_t*)&db_runtime.LiveData.i8_param[index];break;
	case I16: sz = 2; sptr = (uint8_t*)&db_runtime.LiveData.i16_param[index];break;
	case I32: sz = 4; sptr = (uint8_t*)&db_runtime.LiveData.i32_param[index];break;
	case F32: sz = 4; sptr = (uint8_t*)&db_runtime.LiveData.f32_param[index];break;
	default:break;
	}

	if(sz > 0){
		memcpy(sptr,dptr,sz);
                if(save_on_write)
                  db_save_section(ptype);
	}

	return ret;
}

int8_t db_read_ld_i8(uint16_t address)
{
	int8_t ret = 0;
	if(PARAM_TYPE(address) == I8)
	{
		ret = db_runtime.LiveData.i8_param[PARAM_INDEX(address)];
	}
	else{
		while(1);
	}
	return ret;
}
int16_t db_read_ld_i16(uint16_t address)
{
	int16_t ret = 0;
	if(PARAM_TYPE(address) == I16)
	{
		ret = db_runtime.LiveData.i16_param[PARAM_INDEX(address)];
	}
	else{
		while(1);
	}
	return ret;
}
int32_t db_read_ld_i32(uint16_t address)
{
	int32_t ret = 0;
	if(PARAM_TYPE(address) == I32)
	{
		ret = db_runtime.LiveData.i32_param[PARAM_INDEX(address)];
	}
	else{
		while(1);
	}
	return ret;
}
uint8_t db_read_ld_u8(uint16_t address)
{
	uint8_t ret = 0;
	if(PARAM_TYPE(address) == U8)
	{
		ret = db_runtime.LiveData.u8_param[PARAM_INDEX(address)];
	}
	else{
		while(1);
	}
	return ret;
}
uint16_t db_read_ld_u16(uint16_t address)
{
	uint16_t ret = 0;
	if(PARAM_TYPE(address) == U16)
	{
		ret = db_runtime.LiveData.u16_param[PARAM_INDEX(address)];
	}
	else{
		while(1);
	}
	return ret;
}
uint32_t db_read_ld_u32(uint16_t address)
{
	uint32_t ret = 0;
	if(PARAM_TYPE(address) == U32)
	{
		ret = db_runtime.LiveData.u32_param[PARAM_INDEX(address)];
	}
	else{
		while(1);
	}
	return ret;
}
float db_read_ld_f32(uint16_t address)
{
	float ret = 0;
	if(PARAM_TYPE(address) == F32)
	{
		ret = db_runtime.LiveData.f32_param[PARAM_INDEX(address)];
	}
	else{
		while(1);
	}
	return ret;
}

void db_write_ld_i8(uint16_t address, int8_t value)
{
	if(PARAM_TYPE(address) == I8)
	{
		db_runtime.LiveData.i8_param[PARAM_INDEX(address)] = value;
	}
}
void db_write_ld_i16(uint16_t address, int16_t value)
{
	if(PARAM_TYPE(address) == I16)
	{
		db_runtime.LiveData.i16_param[PARAM_INDEX(address)] = value;
	}
}
void db_write_ld_i32(uint16_t address, int32_t value)
{
	if(PARAM_TYPE(address) == I32)
	{
		db_runtime.LiveData.i32_param[PARAM_INDEX(address)] = value;
	}
}
void db_write_ld_u8(uint16_t address, uint8_t value)
{
	if(PARAM_TYPE(address) == U8)
	{
		db_runtime.LiveData.u8_param[PARAM_INDEX(address)] = value;
	}
}
void db_write_ld_u16(uint16_t address, uint16_t value)
{
	if(PARAM_TYPE(address) == U16)
	{
		db_runtime.LiveData.u16_param[PARAM_INDEX(address)] = value;
	}
}
void db_write_ld_u32(uint16_t address, uint32_t value)
{
	if(PARAM_TYPE(address) == U32)
	{
		db_runtime.LiveData.u32_param[PARAM_INDEX(address)] = value;
	}
}
void db_write_ld_f32(uint16_t address, float value)
{
	if(PARAM_TYPE(address) == F32)
	{
		db_runtime.LiveData.f32_param[PARAM_INDEX(address)] = value;
	}
}

void db_enable_save_on_write(bool set)
{
  save_on_write = set;
}

