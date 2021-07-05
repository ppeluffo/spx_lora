/*
 * l_psensor.h
 *
 *  Created on: 23 ago. 2019
 *      Author: pablo
 */

#ifndef SRC_SPX_LIBS_L_ABP_H_
#define SRC_SPX_LIBS_L_ABP_H_

#include "frtos-io.h"
#include "stdint.h"
#include "l_i2c.h"
#include "l_printf.h"

//--------------------------------------------------------------------------------
// API START

#define ABP_PMAX_RATED	150.0
// Out_max: 14745 counts ( 90% de 2^14 o 0x3999 )
#define ABP_COUNTS_MAX 	14745
// Out_min:  1638 counts ( 10% de 2^14 o 0x666  )
#define ABP_COUNTS_MIN 	1638

int8_t abp_raw_read( uint8_t sensor_id, char *data );
int8_t abp_raw_read_and_sleep( uint8_t sensor_id, char *data );
bool abp_raw_read_test( uint8_t sensor_id );
void abp_FMR(void);
void abp_PMR(void);

// API END
//--------------------------------------------------------------------------------


#endif /* SRC_SPX_LIBS_L_ABP_H_ */
