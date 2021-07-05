/*
 * spx_tkCtl.c
 *
 *  Created on: 4 de oct. de 2017
 *      Author: pablo
 *
 */

#include "spx.h"

//------------------------------------------------------------------------------------
static void pv_ctl_init_system(void);
static void pv_ctl_wink_led(void);
static void pv_ctl_check_wdg(void);
static void pv_ctl_daily_reset(void);
static void pv_ctl_check_terminal_present(void);

static uint16_t watchdog_timers[NRO_WDGS];

static bool f_terminal_connected = false;;

// Timpo que espera la tkControl entre round-a-robin
#define TKCTL_DELAY_S	5

// La tarea pasa por el mismo lugar c/5s.
#define WDG_CTL_TIMEOUT	WDG_TO30

const char string_0[] PROGMEM = "CTL";
const char string_1[] PROGMEM = "CMD";

const char * const wdg_names[] PROGMEM = { string_0, string_1 };

//------------------------------------------------------------------------------------
void tkCtl(void * pvParameters)
{

( void ) pvParameters;

	vTaskDelay( ( TickType_t)( 500 / portTICK_RATE_MS ) );

	pv_ctl_init_system();

	xprintf_P( PSTR("\r\nstarting tkControl..\r\n\0"));

	//lora_sys_sleep(true, 60000 );

	for( ;; )
	{

		// Paso c/5s plt 30s es suficiente.
		ctl_watchdog_kick(WDG_CTL, WDG_CTL_TIMEOUT);

		// Cada 5s controlo el watchdog y los timers.
		pv_ctl_check_wdg();
		pv_ctl_check_terminal_present();
		pv_ctl_wink_led();
		pv_ctl_daily_reset();

		// Para entrar en tickless.
		// Cada 5s hago un chequeo de todo. En particular esto determina el tiempo
		// entre que activo el switch de la terminal y que esta efectivamente responde.
		vTaskDelay( ( TickType_t)( TKCTL_DELAY_S * 1000 / portTICK_RATE_MS ) );
	}
}
//------------------------------------------------------------------------------------
static void pv_ctl_init_system(void)
{

	// Esta funcion corre cuando el RTOS esta inicializado y corriendo lo que nos
	// permite usar las funciones del rtos-io y rtos en las inicializaciones.
	//

uint8_t wdg = 0;

	// Configuro los pines del micro que no se configuran en funciones particulares
	// LEDS:
	IO_config_LED_KA();

	// TERMINAL CTL PIN
	IO_config_TERMCTL_PIN();
	IO_config_TERMCTL_PULLDOWN();

	// Deteccion inicial de la termial conectada o no.
	f_terminal_connected = false;
	if (  IO_read_TERMCTL_PIN() == 1 ) {
		f_terminal_connected = true;
	}

	// BAUD RATE PIN
	IO_config_BAUD_PIN();

	// Al comienzo leo este handle para asi usarlo para leer el estado de los stacks.
	// En la medida que no estoy usando la taskIdle podria deshabilitarla. !!!
	xHandle_idle = xTaskGetIdleTaskHandle();

	//----------------------------------------------------------------------------------------------------------------
	// WATCHDOG
	// Inicializo todos los watchdogs a 30s ( 3 * 5s de loop )
	for ( wdg = 0; wdg < NRO_WDGS; wdg++ ) {
		watchdog_timers[wdg] = (uint16_t)( TKCTL_DELAY_S * 6 );
	}

	// Luego del posible error del bus I2C espero para que se reponga !!!
	vTaskDelay( ( TickType_t)( 100 ) );

	//----------------------------------------------------------------------------------------------------------------
	// SYSTEMVARS:
	// Leo los parametros del la EE y si tengo error, cargo por defecto
	if ( ! u_load_params_from_NVMEE() ) {
		u_load_defaults( NULL );
		u_save_params_in_NVMEE();
		xprintf_P( PSTR("\r\nLoading defaults !!\r\n\0"));
	}

	//----------------------------------------------------------------------------------------------------------------
	// Configuro, inicializo y arranco los timers de contador ( caudalimetro de pulsos )
	counter_init();

	//--------------------------------------------------------------------------------
	// Creo los buffer de trabajo de lora
	u_lora_init();


	// Habilito a arrancar al resto de las tareas
	startTask = true;

}
//------------------------------------------------------------------------------------
static void pv_ctl_check_terminal_present(void)
{
	// Lee el pin de la terminal para ver si hay o no una conectada.
	// Si bien en la IO8 no es necesario desconectar la terminal ya que opera
	// con corriente, por simplicidad uso un solo codigo para ambas arquitecturas.


	//xprintf_P( PSTR("DEBUG terminal start..\r\n\0"));
	if ( IO_read_TERMCTL_PIN() == 1) {
		f_terminal_connected = true;
	} else {
		f_terminal_connected = false;
	}
//	f_terminal_connected = true;
	//xprintf_P( PSTR("DEBUG terminal end..\r\n\0"));
	return;
}
//------------------------------------------------------------------------------------
static void pv_ctl_wink_led(void)
{

	//xprintf_P( PSTR("DEBUG Led..\r\n\0"));
	// SI la terminal esta desconectada salgo.
/*
	if ( ! ctl_terminal_connected() ) {
		// Apago
		IO_set_LED_KA();
		return;
	}
*/
	// Prendo los leds
	IO_clr_LED_KA();

	vTaskDelay( ( TickType_t)( 10 ) );

	// Apago
	IO_set_LED_KA();

}
//------------------------------------------------------------------------------------
static void pv_ctl_check_wdg(void)
{
	// Cada tarea periodicamente reinicia su wdg timer.
	// Esta tarea los decrementa cada 5 segundos.
	// Si alguno llego a 0 es que la tarea se colgo y entonces se reinicia el sistema.

	// Cada ciclo reseteo el wdg para que no expire.
	WDT_Reset();

}
//------------------------------------------------------------------------------------
static void pv_ctl_daily_reset(void)
{
	// Todos los dias debo resetearme para restaturar automaticamente posibles
	// problemas.
	// Se invoca 1 vez por minuto ( 60s ).

static uint32_t ticks_to_reset = 86400 / TKCTL_DELAY_S ; // ticks en 1 dia.

	//xprintf_P( PSTR("DEBUG dailyReset..\r\n\0"));
	while ( --ticks_to_reset > 0 ) {
		return;
	}


	xprintf_P( PSTR("Daily Reset !!\r\n\0") );
	vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );

	CCPWrite( &RST.CTRL, RST_SWRST_bm );   /* Issue a Software Reset to initilize the CPU */


}
//------------------------------------------------------------------------------------
// FUNCIONES PUBLICAS
//------------------------------------------------------------------------------------
void ctl_watchdog_kick(uint8_t taskWdg, uint16_t timeout_in_secs )
{
	// Reinicia el watchdog de la tarea taskwdg con el valor timeout.
	// timeout es uint16_t por lo tanto su maximo valor en segundos es de 65536 ( 18hs )

	while ( xSemaphoreTake( sem_WDGS, ( TickType_t ) 5 ) != pdTRUE )
		taskYIELD();

	watchdog_timers[taskWdg] = timeout_in_secs;

	xSemaphoreGive( sem_WDGS );
}
//------------------------------------------------------------------------------------
bool ctl_terminal_connected(void)
{
	return(f_terminal_connected);
}
//------------------------------------------------------------------------------------
