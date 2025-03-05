#ifndef _SHELL_COMMAND_
#define _SHELL_COMMAND_

#include "hal.h"

//void cmd_write(BaseSequentialStream *chp, int argc, char *argv[]) ;
void cmd_force(BaseSequentialStream *chp, int argc, char *argv[]) ;
void cmd_force_mode(BaseSequentialStream *chp, int argc, char *argv[]) ;
void cmd_force_range(BaseSequentialStream *chp, int argc, char *argv[]) ;
void cmd_clamp(BaseSequentialStream *chp, int argc, char *argv[]) ;
void cmd_cpoh(BaseSequentialStream *chp, int argc, char *argv[]);
void cmd_load(BaseSequentialStream *chp, int argc, char *argv[]) ;
void cmd_dac_output(BaseSequentialStream *chp, int argc, char *argv[]) ;
void cmd_dac_gain(BaseSequentialStream *chp, int argc, char *argv[]) ;
void cmd_dac_offset(BaseSequentialStream *chp, int argc, char *argv[]) ;
//void cmd_load(BaseSequentialStream *chp, int argc, char *argv[]) ;
void cmd_meas_out(BaseSequentialStream *chp, int argc, char *argv[]);
#endif