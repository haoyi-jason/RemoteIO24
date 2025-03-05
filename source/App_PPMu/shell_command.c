#include "ch.h"
#include "hal.h"
#include "shell_command.h"
#include <string.h>
#include <stdlib.h>
#include "task_ppmu.h"
#include "ad5522_def.h"

/**
  @brief set force output of pmu(s)
  @usage force channel value
  @param pmu_id[in]:    0 for all pmu, 1 to 4 for single pmu
  @param value          16-bit value for output

*/

void cmd_force(BaseSequentialStream *chp, int argc, char *argv[]) 
{
  if(argc == 2){
    uint8_t ch = strtol(argv[0],NULL,10);
    uint16_t val = strtol(argv[1],NULL,10);
    pmu_set_output(ch,val);
  }
}
/**
  @brief set force mode (FVMI/FIMV) of pmu
  @usage fm channel mode
  @param mode: fvmi/fimv/fv/fi        
*/

void cmd_force_mode(BaseSequentialStream *chp, int argc, char *argv[]) 
{
  if(argc == 2){
    uint8_t ch = strtol(argv[0],NULL,10);
    if(strcmp(argv[1],"FVMI") == 0){
      pmu_set_output_mode(ch,PMU_PMU_REG_FORCE_FVMI);
    }
    else if(strcmp(argv[1],"FIMV") == 0){
      pmu_set_output_mode(ch,PMU_PMU_REG_FORCE_FIMV);
    }
  }
}


/**
  @brief set force range, current mode only
  @usage fr channel range
  @param[in] channel: 0 to pmu channels
  @param[in] range: 0/1/2/3/4 for 5/20/200/2000/EXT
*/
void cmd_force_range(BaseSequentialStream *chp, int argc, char *argv[]) 
{
  if(argc == 2){
    uint8_t ch = strtol(argv[0],NULL,10);
    uint8_t range = strtol(argv[1],NULL,10);
    pmu_set_crange(ch,range);
  }
}

/**
  @brief set measurement output
  @usage measout channel type
  @param[in] channel: 0 to pmu channels
  @param[in] range: 0/1/2/3 for Is, Vs, T, HZ
*/
void cmd_meas_out(BaseSequentialStream *chp, int argc, char *argv[]) 
{
  if(argc == 2){
    uint8_t ch = strtol(argv[0],NULL,10);
    uint8_t type = strtol(argv[1],NULL,10);
    pmu_set_measout(ch,type);
  }
}


/**
  @brief set clamp output
  @usage clamp channel on/off
*/
void cmd_clamp(BaseSequentialStream *chp, int argc, char *argv[]) 
{
  if(argc == 2){
    uint8_t ch = strtol(argv[0],NULL,10);
    if(strcmp(argv[1],"on") == 0){
      pmu_cl(ch,1);
    }
    else{
      pmu_cl(ch,0);
    }
  }
}

/**
  @brief set compare output
  @usage cpol channel on/off
*/
void cmd_cpoh(BaseSequentialStream *chp, int argc, char *argv[]) 
{
  if(argc == 2){
    uint8_t ch = strtol(argv[0],NULL,10);
    if(strcmp(argv[1],"on") == 0){
      pmu_cpolh(ch,1);
    }
    else{
      pmu_cpolh(ch,0);
    }
  }
}
            
/**
  @brief set dac output
  @usage daout channel range func value
  @param[in] channel
  @param[in] range i/v/cpl/cph
  @param[in] func, 0/1/2/3/e/l/h for i, 0/l/h for v
  set 0 i 0/1/2/3/e 8000
  set 0 v 1 8000
  set 0 cpl/cph 1/2/3/4/5/e
*/

void cmd_dac_output(BaseSequentialStream *chp, int argc, char *argv[]) 
{
  if(argc == 4){
    uint8_t ch = strtol(argv[0],NULL,10);
    uint16_t value = strtol(argv[3],NULL,10);
    if(strcmp(argv[1],"i") == 0){
      switch(*argv[2]){
      case '0':pmu_set_dac_output(ch,DAC_ID_I_5UA,value);break;
      case '1':pmu_set_dac_output(ch,DAC_ID_I_20UA,value);break;
      case '2':pmu_set_dac_output(ch,DAC_ID_I_200UA,value);break;
      case '3':pmu_set_dac_output(ch,DAC_ID_I_2MA,value);break;
      case 'e':pmu_set_dac_output(ch,DAC_ID_I_EXT,value);break;
      case 'l':pmu_set_dac_output(ch,DAC_ID_I_CLL,value);break;
      case 'h':pmu_set_dac_output(ch,DAC_ID_I_CLH,value);break;
      default:break;
      }
    }
    else if(strcmp(argv[1],"v") == 0){
      switch(*argv[2]){
      case '0':pmu_set_dac_output(ch,DAC_ID_V,value);break;
      case 'l':pmu_set_dac_output(ch,DAC_ID_V_CLL,value);break;
      case 'h':pmu_set_dac_output(ch,DAC_ID_V_CLH,value);break;
      default:break;
      }
      pmu_set_dac_output(ch,DAC_ID_V,value);
    }
    else if(strcmp(argv[1],"cpl") == 0){
      switch(*argv[2]){
      case '0':pmu_set_dac_output(ch,DAC_ID_I_CPL_5UA,value);break;
      case '1':pmu_set_dac_output(ch,DAC_ID_I_CPL_20UA,value);break;
      case '2':pmu_set_dac_output(ch,DAC_ID_I_CPL_200UA,value);break;
      case '3':pmu_set_dac_output(ch,DAC_ID_I_CPL_2MA,value);break;
      case 'e':pmu_set_dac_output(ch,DAC_ID_I_CPL_EXT,value);break;
      case 'v':pmu_set_dac_output(ch,DAC_ID_V_CPL,value);break;
      default:break;
      }
    }
    else if(strcmp(argv[1],"cph") == 0){
      switch(*argv[2]){
      case '0':pmu_set_dac_output(ch,DAC_ID_I_CPH_5UA,value);break;
      case '1':pmu_set_dac_output(ch,DAC_ID_I_CPH_20UA,value);break;
      case '2':pmu_set_dac_output(ch,DAC_ID_I_CPH_200UA,value);break;
      case '3':pmu_set_dac_output(ch,DAC_ID_I_CPH_2MA,value);break;
      case 'e':pmu_set_dac_output(ch,DAC_ID_I_CPH_EXT,value);break;
      case 'v':pmu_set_dac_output(ch,DAC_ID_V_CPH,value);break;
      default:break;
      }
    }
  }
}

/**
  @brief set dac gain
*/
void cmd_dac_gain(BaseSequentialStream *chp, int argc, char *argv[]) 
{
  if(argc == 4){
    uint8_t ch = strtol(argv[0],NULL,10);
    uint16_t value = strtol(argv[3],NULL,10);
    if(strcmp(argv[1],"i") == 0){
      switch(*argv[1]){
      case '0':pmu_set_dac_gain(ch,DAC_ID_I_5UA,value);break;
      case '1':pmu_set_dac_gain(ch,DAC_ID_I_20UA,value);break;
      case '2':pmu_set_dac_gain(ch,DAC_ID_I_200UA,value);break;
      case '3':pmu_set_dac_gain(ch,DAC_ID_I_2MA,value);break;
      case 'e':pmu_set_dac_gain(ch,DAC_ID_I_EXT,value);break;
      case 'l':pmu_set_dac_gain(ch,DAC_ID_I_CLL,value);break;
      case 'h':pmu_set_dac_gain(ch,DAC_ID_I_CLH,value);break;
      default:break;
      }
    }
    else if(strcmp(argv[1],"v") == 0){
      switch(*argv[1]){
      case '0':pmu_set_dac_gain(ch,DAC_ID_V,value);break;
      case 'l':pmu_set_dac_gain(ch,DAC_ID_V_CLL,value);break;
      case 'h':pmu_set_dac_gain(ch,DAC_ID_V_CLH,value);break;
      default:break;
      }
    }
    else if(strcmp(argv[1],"cpl") == 0){
      switch(*argv[1]){
      case '0':pmu_set_dac_gain(ch,DAC_ID_I_CPL_5UA,value);break;
      case '1':pmu_set_dac_gain(ch,DAC_ID_I_CPL_20UA,value);break;
      case '2':pmu_set_dac_gain(ch,DAC_ID_I_CPL_200UA,value);break;
      case '3':pmu_set_dac_gain(ch,DAC_ID_I_CPL_2MA,value);break;
      case 'e':pmu_set_dac_gain(ch,DAC_ID_I_CPL_EXT,value);break;
      case 'v':pmu_set_dac_gain(ch,DAC_ID_V_CPL,value);break;
      default:break;
      }
    }
    else if(strcmp(argv[1],"cph") == 0){
      switch(*argv[1]){
      case '0':pmu_set_dac_gain(ch,DAC_ID_I_CPH_5UA,value);break;
      case '1':pmu_set_dac_gain(ch,DAC_ID_I_CPH_20UA,value);break;
      case '2':pmu_set_dac_gain(ch,DAC_ID_I_CPH_200UA,value);break;
      case '3':pmu_set_dac_gain(ch,DAC_ID_I_CPH_2MA,value);break;
      case 'e':pmu_set_dac_gain(ch,DAC_ID_I_CPH_EXT,value);break;
      case 'v':pmu_set_dac_gain(ch,DAC_ID_V_CPH,value);break;
      default:break;
      }
    }
  }
}

/**
  @brief set dac offset
*/
void cmd_dac_offset(BaseSequentialStream *chp, int argc, char *argv[]) 
{
  if(argc == 4){
    uint8_t ch = strtol(argv[0],NULL,10);
    uint16_t value = strtol(argv[3],NULL,10);
    if(strcmp(argv[1],"i") == 0){
      switch(*argv[1]){
      case '1':pmu_set_dac_offset(ch,DAC_ID_I_5UA,value);break;
      case '2':pmu_set_dac_offset(ch,DAC_ID_I_20UA,value);break;
      case '3':pmu_set_dac_offset(ch,DAC_ID_I_200UA,value);break;
      case '4':pmu_set_dac_offset(ch,DAC_ID_I_2MA,value);break;
      case 'e':pmu_set_dac_offset(ch,DAC_ID_I_EXT,value);break;
      case 'l':pmu_set_dac_offset(ch,DAC_ID_I_CLL,value);break;
      case 'h':pmu_set_dac_offset(ch,DAC_ID_I_CLH,value);break;
      default:break;
      }
    }
    else if(strcmp(argv[1],"v") == 0){
      switch(*argv[1]){
      case '1':pmu_set_dac_offset(ch,DAC_ID_V,value);break;
      case 'l':pmu_set_dac_offset(ch,DAC_ID_V_CLL,value);break;
      case 'h':pmu_set_dac_offset(ch,DAC_ID_V_CLH,value);break;
      default:break;
      }
    }
    else if(strcmp(argv[1],"cpl") == 0){
      switch(*argv[1]){
      case '1':pmu_set_dac_offset(ch,DAC_ID_I_CPL_5UA,value);break;
      case '2':pmu_set_dac_offset(ch,DAC_ID_I_CPL_20UA,value);break;
      case '3':pmu_set_dac_offset(ch,DAC_ID_I_CPL_200UA,value);break;
      case '4':pmu_set_dac_offset(ch,DAC_ID_I_CPL_2MA,value);break;
      case 'e':pmu_set_dac_offset(ch,DAC_ID_I_CPL_EXT,value);break;
      case 'v':pmu_set_dac_offset(ch,DAC_ID_V_CPL,value);break;
      default:break;
      }
    }
    else if(strcmp(argv[1],"cph") == 0){
      switch(*argv[1]){
      case '1':pmu_set_dac_offset(ch,DAC_ID_I_CPH_5UA,value);break;
      case '2':pmu_set_dac_offset(ch,DAC_ID_I_CPH_20UA,value);break;
      case '3':pmu_set_dac_offset(ch,DAC_ID_I_CPH_200UA,value);break;
      case '4':pmu_set_dac_offset(ch,DAC_ID_I_CPH_2MA,value);break;
      case 'e':pmu_set_dac_offset(ch,DAC_ID_I_CPH_EXT,value);break;
      case 'v':pmu_set_dac_offset(ch,DAC_ID_V_CPH,value);break;
      default:break;
      }
    }
  }  
}

