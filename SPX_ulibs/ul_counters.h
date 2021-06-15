/*
 * ul_counters.h
 *
 *  Created on: 25 may. 2021
 *      Author: pablo
 */

#include "spx.h"

#ifndef SRC_SPX_ULIBS_UL_COUNTERS_H_
#define SRC_SPX_ULIBS_UL_COUNTERS_H_

bool caudal_read_test(void);
float counter_read_caudal_ltsxs(void);
void counter_init_outofrtos(void);
void counter_init(void);
void counter_print(file_descriptor_t fd, float cnt );
void counter_config_defaults(void);
bool counter_config( char *s_magpp, char *s_pw, char *s_period, char *s_sensing );
uint8_t counters_hash(void);


#endif /* SRC_SPX_ULIBS_UL_COUNTERS_H_ */
