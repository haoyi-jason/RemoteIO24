#ifndef _SHELL_COMMAND_
#define _SHELL_COMMAND_

#include "hal.h"

void cmd_write(BaseSequentialStream *chp, int argc, char *argv[]) ;
void cmd_writeHex(BaseSequentialStream *chp, int argc, char *argv[]) ;
void cmd_read(BaseSequentialStream *chp, int argc, char *argv[]);
void cmd_addr(BaseSequentialStream *chp, int argc, char *argv[]);
void cmd_selfaddr(BaseSequentialStream *chp, int argc, char *argv[]) ;
void cmd_power(BaseSequentialStream *chp, int argc, char *argv[]);
void cmd_freq(BaseSequentialStream *chp, int argc, char *argv[]);
void cmd_rate(BaseSequentialStream *chp, int argc, char *argv[]);
void cmd_mode(BaseSequentialStream *chp, int argc, char *argv[]) ;
void cmd_interval(BaseSequentialStream *chp, int argc, char *argv[]);
void cmd_save(BaseSequentialStream *chp, int argc, char *argv[]) ;
void cmd_default_io_pattern(BaseSequentialStream *chp, int argc, char *argv[]);
void cmd_poll_interval(BaseSequentialStream *chp, int argc, char *argv[]) ;
void cmd_poll_count(BaseSequentialStream *chp, int argc, char *argv[]);
void cmd_vrMap(BaseSequentialStream *chp, int argc, char *argv[]) ;
void cmd_spdMap(BaseSequentialStream *chp, int argc, char *argv[]);
void cmd_ppr(BaseSequentialStream *chp, int argc, char *argv[]);
void cmd_txsim(BaseSequentialStream *chp, int argc, char *argv[]);
void cmd_idn(BaseSequentialStream *chp, int argc, char *argv[]);

#endif