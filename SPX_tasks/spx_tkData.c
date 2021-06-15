/*
 * spx_tkData.c
 *
 *  Created on: 19 may. 2021
 *      Author: pablo
 */

#include "spx.h"

//------------------------------------------------------------------------------------
static void pv_data_init(void);
static void pv_data_read_inputs( u_dataRecord_t *dr, bool f_debug);
static void pv_data_calcular_waiting_ticks( uint32_t *ticks );

// La tarea pasa por el mismo lugar c/5s.
#define WDG_DATA_TIMEOUT	WDG_TO30

#define DF_DATA ( systemVars.debug == DEBUG_DATA )

u_dataRecord_t dataRecd;

//------------------------------------------------------------------------------------
void tkData(void * pvParameters)
{

( void ) pvParameters;
TickType_t xLastWakeTime = 0;
uint32_t waiting_ticks;

	// Espero la notificacion para arrancar
	while ( !startTask )
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

	vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );

	pv_data_init();

	xprintf_P( PSTR("\r\nstarting tkData..\r\n\0"));

	// Initialise the xLastWakeTime variable with the current time.
 	xLastWakeTime = xTaskGetTickCount();

 	// Al arrancar poleo a los 10s
 	waiting_ticks = (uint32_t)(10) * 1000 / portTICK_RATE_MS;

	for( ;; )
	{

		// Paso c/5s plt 30s es suficiente.
		ctl_watchdog_kick(WDG_DATA, WDG_DATA_TIMEOUT);
		//xprintf_P(PSTR("WTicks=%lu\r\n"), waiting_ticks);

		vTaskDelay( ( TickType_t)( waiting_ticks ) );

 		// Espero. Da el tiempo necesario para entrar en tickless.
 		vTaskDelayUntil( &xLastWakeTime, waiting_ticks );

		//pv_data_read_inputs( &dataRecd, false );
		//u_print_dr( fdTERM, &dataRecd, 0 );
		pv_data_calcular_waiting_ticks( &waiting_ticks );

		//u_SEND_SIGNAL( SGN_FRAME_READY );
	}
}
//------------------------------------------------------------------------------------
static void pv_data_init(void)
{


}
//------------------------------------------------------------------------------------
static void pv_data_read_inputs( u_dataRecord_t *dr, bool f_debug)
{
	// Leemos las presiones, caudal y lo dejamos en un datarecord

	// Leo las presiones
	dr->presion = psensor_read( PA, KGM_CM2, DF_DATA );

	// Leo el caudal
	dr->caudal = counter_read_caudal_ltsxs();

}
//------------------------------------------------------------------------------------
static void pv_data_calcular_waiting_ticks( uint32_t *ticks )
{
	// Calculo el tiempo para una nueva espera

	while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 5 ) != pdTRUE )
		taskYIELD();

	// timpo real que voy a dormir esta tarea
	*ticks = (uint32_t)(systemVars.timer_poll) * 1000 / portTICK_RATE_MS;

	xSemaphoreGive( sem_SYSVars );
	return;
}
//------------------------------------------------------------------------------------
void data_read_frame(bool poll_now )
{
	// Funcion publica usada desde cmdMode para forzar una lectura de un frame
	// y visualizarlo en pantalla

	if ( poll_now )
		pv_data_read_inputs( &dataRecd, false );

	u_print_dr( fdTERM, &dataRecd, 0 );

}
//------------------------------------------------------------------------------------

