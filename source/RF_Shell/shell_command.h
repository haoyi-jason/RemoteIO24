#ifndef _SHELL_COMMAND_
#define _SHELL_COMMAND_

typedef void(*shell_cb)(uint8_t,uint32_t);
void shell_cmd_menu_init(BaseSequentialStream *chp, shell_cb cb);

#endif
