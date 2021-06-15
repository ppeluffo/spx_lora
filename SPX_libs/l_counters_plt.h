/*
 * l_counters.h
 *
 *  Created on: 8 dic. 2018
 *      Author: pablo
 */

#ifndef SRC_SPX_LIBS_L_PLT_COUNTERS_H_
#define SRC_SPX_LIBS_L_PLT_COUNTERS_H_

#include "l_iopines.h"
#include "stdbool.h"
#include <avr/interrupt.h>
//#include "FreeRTOS.h"
//#include "task.h"

// Las entradas de pulsos no solo se configuran para pulso sino tambien
// para generar interrupciones.
//------------------------------------------------------------------------------------
// API publica

typedef enum { RISING_EDGE = 0, FALLING_EDGE } t_sensing_edge;

#define CNT_config_CNT0()	IO_config_PB2()
#define CNT_read_CNT0()		IO_read_PB2()

void COUNTER_init( t_sensing_edge edge );
void COUNTER_disable_interrupt();
void COUNTER_enable_interrupt();

// API end
//------------------------------------------------------------------------------------


#endif /* SRC_SPX_LIBS_L_PLT_COUNTERS_H_ */
