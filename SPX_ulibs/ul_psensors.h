/*
 * ul_psensor.h
 *
 *  Created on: 19 may. 2021
 *      Author: pablo
 */

#ifndef SRC_SPX_ULIBS_UL_PSENSORS_H_
#define SRC_SPX_ULIBS_UL_PSENSORS_H_

#include "spx.h"
//#include "stdbool.h"
//#include "l_abp.h"
//#include "l_tca9546.h"
//#include "l_printf.h"


typedef enum { PSI=0, KGM_CM2 } t_presion_format;
typedef enum { PA=0, PB } t_presion_id;

float psensor_read( t_presion_id pid, t_presion_format formato, bool fdebug );
bool psensor_read_test(void);
void psensor_config_defaults(void);
void psensor_config_print(void);


#endif /* SRC_SPX_ULIBS_UL_PSENSORS_H_ */
