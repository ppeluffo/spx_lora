/*
 * l_counters.c
 *
 *  Created on: 8 dic. 2018
 *      Author: pablo
 */


#include "l_counters_plt.h"

//------------------------------------------------------------------------------------
void COUNTER_init( t_sensing_edge edge )
{

	CNT_config_CNT0();
	// Con optoacoplador. Normalmente en 0. Sensa rising edge
	//PORTB.PIN2CTRL = PORT_OPC_PULLDOWN_gc | PORT_ISC_RISING_gc;	    // Sensa rising edge. Menos consumo con pulldown.
	if ( edge == RISING_EDGE ) {
		PORTB.PIN2CTRL = PORT_OPC_PULLDOWN_gc | PORT_ISC_RISING_gc;		// Sensa rising edge. Menos consumo con pulldown.
	} else {
		PORTB.PIN2CTRL = PORT_OPC_PULLDOWN_gc | PORT_ISC_FALLING_gc;
	}
	COUNTER_enable_interrupt();

	// https://www.freertos.org/FreeRTOS_Support_Forum_Archive/June_2005/freertos_Get_task_handle_1311096.html
	// The task handle is just a pointer to the TCB of the task - but outside of tasks.c the type is hidden as a void*.

}
//------------------------------------------------------------------------------------
void COUNTER_disable_interrupt()
{
	PORTB.INT0MASK = 0x00;
	PORTB.INTCTRL = 0x00;
}
//------------------------------------------------------------------------------------
void COUNTER_enable_interrupt()
{
	PORTB.INT0MASK = PIN2_bm;
	PORTB.INTCTRL = PORT_INT0LVL0_bm;
	PORTB.INTFLAGS = PORT_INT0IF_bm;

}
//------------------------------------------------------------------------------------
