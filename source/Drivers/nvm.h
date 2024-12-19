#ifndef _NVM_
#define _NVM_

msg_t nvm_flash_read(uint32_t offset, uint8_t *dptr, uint16_t sz);

msg_t nvm_flash_write(uint16_t offset, uint8_t *dptr, uint16_t sz);
#endif