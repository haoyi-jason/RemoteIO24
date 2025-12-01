#include "ch.h"
#include "hal.h"
#include "bq76xx_dev.h"
#include "encryption/crc8/crc8.h"

#define SPI_DEV         SPID1
#define I2C_DEV         I2CD1
#define CS_SELECT()     palClearPad(GPIOA,4)
#define CS_UNSELECT()   palSetPad(GPIOA,4)

#define REG_WRITE               0x80
#define BUFFER_BASE_ADDR        0x40
#define BUFFER_SIZE             0x20

#define SLAVE_ADDR              0x08

static const SPIConfig spicfg = {
  FALSE,
  NULL,
  NULL,//GPIOB,
  NULL,//GPIOB_SPI1_CS,
  SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0 
};

static const I2CConfig i2ccfg = {
  OPMODE_I2C,
  100000,
  STD_DUTY_CYCLE,
};


static void cs_delay(uint32_t x)
{
  while(--x){
    __NOP();
  }
}

/*** I2C functin ***/
static void bq76xx_i2c_ram_read(uint16_t address, uint8_t *dptr, uint8_t sz)
{
  msg_t ret;
  uint8_t tx[32],rx[32];
  uint8_t crc8 = 0;
  i2cAcquireBus(&I2C_DEV);
  i2cStart(&I2C_DEV,&i2ccfg);
  
  crc8 = crc8_smbus_single_byte((SLAVE_ADDR<<1),0x0);
  
  tx[0] = 0x3E;
  tx[1] = (address & 0xff);
  crc8 = crc8_smbus_table(tx,2,crc8);
  tx[2] = crc8;

  tx[3] = ((address >> 8) & 0xff);
  tx[4] = crc8_smbus_table(tx,3,0x0);
   
  ret = i2cMasterTransmitTimeout(&I2C_DEV,SLAVE_ADDR,tx,5,NULL,0,TIME_MS2I(100));
  tx[0] = 0x40;
  crc8 = crc8_smbus_single_byte((SLAVE_ADDR<<1)+1,0x0);
  crc8 = crc8_smbus_single_byte(tx[0],crc8);
  tx[1] = crc8;
  
  ret = i2cMasterTransmitTimeout(&I2C_DEV,SLAVE_ADDR,tx,1,rx,sz<<1,TIME_MS2I(100));
  
  
  i2cStop(&I2C_DEV);
  i2cReleaseBus(&I2C_DEV);
}

static void bq76xx_i2c_ram_write(uint16_t address, uint8_t *dptr, uint8_t sz)
{
  msg_t ret;
  uint8_t tx[32],rx[32];
  i2cAcquireBus(&I2C_DEV);
  i2cStart(&I2C_DEV,&i2ccfg);
  
  tx[0] = (address & 0xff);
  tx[1] = ((address >> 8) & 0xff);
  for(uint8_t i=0;i<sz;i++){
    tx[i+2] = dptr[i];
  }
  // calculate crc
  tx[sz+2] = crc8_smbus_table(tx,sz+2,0x0);
  ret = i2cMasterTransmitTimeout(&I2C_DEV,SLAVE_ADDR,tx,sz+3,NULL,0,TIME_MS2I(100));
  
  
  i2cStop(&I2C_DEV);
  i2cReleaseBus(&I2C_DEV);
}

static void bq76xx_i2c_subCommand(uint16_t cmd)
{  
  bq76xx_i2c_ram_write(cmd,NULL,0);
}



/*** End of I2C functin ***/

static void bq76xx_protocol_transaction(uint8_t *tx, uint8_t *rx)
{
  tx[2] = crc8_smbus_table(tx,2,0x0);
  bool done = false;
  //uint8_t rx[3];
  spiAcquireBus(&SPI_DEV);
  spiStart(&SPI_DEV,&spicfg);
  uint8_t retry = 0;
  while(!done){
    CS_SELECT();
    cs_delay(0x80);
    spiExchange(&SPI_DEV, 3, tx, rx);
    CS_UNSELECT();
    if(rx[0] == tx[0]){
      done = true;
//      spiExchange(&SPI_DEV, 3, tx, rx); // for read operation
      break;
    }
    retry++;
    if(retry > 50){
      break;
    }
    chThdSleepMicroseconds(100);
  }
  spiStop(&SPI_DEV);
  spiReleaseBus(&SPI_DEV);
}

static void bq76xx_write_address(uint16_t address)
{
  uint8_t tx[3],rx[3];
  tx[0] = 0x3E | REG_WRITE;
  tx[1] = (address & 0xff);
  bq76xx_protocol_transaction(tx,rx);
  tx[0] = 0x3F | REG_WRITE;
  tx[1] = ((address >> 8) & 0xff);
  bq76xx_protocol_transaction(tx,rx);
}

static void bq76xx_write_data(uint8_t *ptr, uint8_t len)
{
  uint8_t tx[3],rx[3];
  if(len < BUFFER_SIZE){
    for(uint8_t i=0;i<len;i++){
      tx[0] = (BUFFER_BASE_ADDR+i) | REG_WRITE;
      tx[1] = ptr[i];
      bq76xx_protocol_transaction(tx,rx);
    }
  }
}

static void bq76xx_write_checksum(uint8_t checksum)
{
  uint8_t tx[3],rx[3];
  tx[0] = (0x60) | REG_WRITE;
  tx[1] = checksum;
  bq76xx_protocol_transaction(tx,rx);
}

static void bq76xx_read_checksum(uint8_t *checksum)
{
  uint8_t tx[3],rx[3];
  tx[0] = (0x60);
  tx[1] = 0xff;
  bq76xx_protocol_transaction(tx,rx);
  *checksum = rx[1];
}

static void bq76xx_write_length(uint8_t len)
{
  uint8_t tx[3],rx[3];
  tx[0] = (0x61) | REG_WRITE;
  tx[1] = len;
  bq76xx_protocol_transaction(tx,rx);
}

static void bq76xx_read_length(uint8_t *len)
{
  uint8_t tx[3],rx[3];
  tx[0] = (0x61) | REG_WRITE;
  tx[1] = 0xff;
  bq76xx_protocol_transaction(tx,rx);
  *len = rx[1];
}

static void bq76xx_read_data(uint8_t *ptr, uint8_t len)
{
  uint8_t tx[3],rx[3];
  if(len < BUFFER_SIZE){
    for(uint8_t i=0;i<len;i++){
      tx[0] = (BUFFER_BASE_ADDR+i);
      tx[1] = 0xff;
      bq76xx_protocol_transaction(tx,rx);
      ptr[i] = rx[1];
    }
  }
}

static void bq76xx_ReadDataFlash(uint16_t reg_addr, uint8_t *dptr, uint8_t sz)
{
  uint8_t tx[3],rx[3];
  uint8_t i;
  bq76xx_write_address(reg_addr);
  bq76xx_read_data(dptr,sz);
}

static void bq76xx_WriteDataFlash(uint16_t reg_addr, uint8_t *dptr, uint8_t sz)
{
  uint8_t tx[3];
  uint8_t i;
  uint8_t checksum = 0;
  
  tx[0] = (reg_addr & 0xff);
  tx[1] = ((reg_addr >> 8) & 0xff);
  checksum = tx[0] + tx[1];
  for(i=0;i<sz;i++){
    checksum += dptr[i];
  }
  checksum = ~checksum;
  bq76xx_write_address(reg_addr);
  bq76xx_write_data(dptr,sz);
  // write checksum to 0x60, length of bytes to 0x61
  bq76xx_write_checksum(checksum);
  bq76xx_write_length(sz+4);
  
  
}

static void bq76xx_direct_command_write(uint8_t command, uint8_t *dptr, uint8_t sz)
{
  uint8_t i;
  uint8_t tx[3],rx[3];
  for(i=0;i<sz;i++){
    tx[0] = (command + i) | REG_WRITE;
    tx[1] = dptr[i];
    bq76xx_protocol_transaction(tx,rx);
  }
}

static void bq76xx_direct_command_read(uint8_t command, uint8_t *dptr, uint8_t sz)
{
  uint8_t i;
  uint8_t tx[3],rx[3];
  for(i=0;i<sz;i++){
    tx[0] = (command + i);
    tx[1] = 0xff;
    bq76xx_protocol_transaction(tx,rx);
    dptr[i] = rx[1];
  }
}

static void bq76xx_subcommand(uint16_t command)
{
  bq76xx_write_address(command);
}

static void bq76xx_subcommand_read(uint16_t command, uint8_t *dptr, uint8_t sz)
{
  bq76xx_write_address(command);
  chThdSleepMilliseconds(100);
  bq76xx_read_data(dptr,sz);
}
#define COMM_TYPE       0x12 // fast IIC w/ CRC
void bq76xx_spi_to_iic()
{
  uint8_t buf[32];
  uint16_t u16 ;
  // read battery status
  bq76xx_direct_command_read(0x12,buf,2);
  u16 = buf[0] | (buf[1]<<8);
  if((buf[1] & 0x03) != 0x01){
    buf[1] &= 0xFC;
    buf[1] |= 0x01;
    bq76xx_direct_command_write(0x12,buf,2);
  }
  
  bq76xx_direct_command_read(0x12,buf,2);
  if((buf[1] & 0x03) != 0x01){
    return;
  }
  
  // enter config update mode
  bq76xx_subcommand(0x0090); // enter config update
  bq76xx_direct_command_read(0x12,buf,2);
  // read first
  bq76xx_ReadDataFlash(0x9239,buf,1);
  if(buf[0] != COMM_TYPE){
     // write register 
    buf[0] = COMM_TYPE;     
    bq76xx_WriteDataFlash(0x9239,buf,1);
    bq76xx_ReadDataFlash(0x9239,buf,1);

    if(buf[0] != COMM_TYPE){
      while(1);
    }
    bq76xx_direct_command_read(0x12,buf,2); // battery status
    if((buf[0] & 0x80) == 0){
      bq76xx_subcommand_read(0x00a0,buf,3); // OTP_WR_CHECK
      if(buf[0] == 0x80){
        bq76xx_subcommand(0x00a1); // otp write
        chThdSleepMilliseconds(100);
        bq76xx_read_data(buf,1); // buf[0] should be 0x80
        
      }
    }
    
  }
  

  bq76xx_subcommand(0x0092); // exit config update
  bq76xx_subcommand(0x29bc); // swap comm mode
  // read back
  //bq76xx_ReadDataFlash(0x9239,buf,1);
  
  
  //bq76xx_subcommand(0x0090); // enter config update
  
  //bq76xx_subcommand(0x0092); // exit config update
  
}

uint8_t bq76xx_spi_check()
{
  uint8_t buf[8];
  // enter config update mode
  //bq76xx_subcommand(0x0090); // enter config update
  bq76xx_ReadDataFlash(0x9239,buf,1);
  //bq76xx_subcommand(0x0092); // exit config update
  return buf[0];
}

uint8_t bq76xx_i2c_check()
{
  uint8_t buf[32];
  bq76xx_i2c_ram_read(0x9239,buf,1);
  
  return 0;
}