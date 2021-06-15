	/*
 * ul_counters.c
 *
 *  Created on: 3 may. 2021
 *      Author: pablo
 *
 *  Entradas que tienen optoacoplador y se usan solo para medir caudales
 *  Normalmente con la entrada flotando ( 3.6V ), el transistor del opto esta cortado y la entrada al micro esta en 0V.
 *  Cuando se activa la entrada contra tierra ( 0V), el diodo conduce, el transistor satura y la entrada al micro pasa
 *  a ser 3.6V (1)
 *  Por esto, las entradas del micro ( interrupcion ) las configuramos para sensar RISING_EDGE.
 *  LOW_SPEED: Activa un timer de debounce.
 *  Cuando el timer expira vuelve a leer el pin y si esta en 1 incrementa el contador.
 *  Arranca otro timer de periodo que es el que rearma la interrupcion.
 *
 *
 */

#include "ul_counters.h"
#include "spx.h"

TimerHandle_t pwidth_xTimer, period_xTimer;
StaticTimer_t pwidth_xTimerBuffer, period_xTimerBuffer;


uint32_t p0_tick, p1_tick;

bool f_counter_running;

BaseType_t xHigherPriorityTaskWoken = pdFALSE;

static void pwidth_TimerCallback( TimerHandle_t xTimer );
static void period_TimerCallback( TimerHandle_t xTimer );

#define DF_COUNTERS ( systemVars.debug == DEBUG_COUNTER )

#define TICKS_1MIN ( 60 * configTICK_RATE_HZ )

#define LTSxS_TO_MTS3xH	3.6

static uint32_t caudal_acumulado;

//------------------------------------------------------------------------------------
// CAUDAL
//------------------------------------------------------------------------------------
bool caudal_read_test(void)
{
	// Lee el caudal y lo imprime ( cmdMode )

float caudal;

	caudal = counter_read_caudal_ltsxs( );
	xprintf_P( PSTR("CAUDAL=%.03f(lts/seg.)\r\n\0") , caudal );
	xprintf_P( PSTR("       %.03f(mt3/h)\r\n"), ( caudal * LTSxS_TO_MTS3xH ) );
	return(true);

}
//------------------------------------------------------------------------------------
float counter_read_caudal_ltsxs(void)
{
	// Calcula el caudal en base a la diferencia de tiempos entre ticks.
	// El maximo error son 10ms

float caudal_ltsxs = 0.0;
uint32_t delta_ticks;
uint32_t tick_now = xTaskGetTickCount();

	//xprintf_P(PSTR("DEBUG: p0=%lu\r\n"),p0_tick);
	//xprintf_P(PSTR("DEBUG: p1=%lu\r\n"),p1_tick);

	// Calculo errores por rollover
	if ( p1_tick > p0_tick ) {
		delta_ticks = p1_tick - p0_tick;
	} else {
		// No hubieron ticks o sea que delta = 0
		//xprintf_P(PSTR("ERROR: Caudal rollover\r\n"));
		return (0.0);
	}

	//xprintf_P(PSTR("DEBUG: delta_ticks=%lu\r\n"),delta_ticks);

	// Controlo si llego algun pulso en el ultimo minuto
	if ( ( tick_now - p1_tick ) > TICKS_1MIN ) {
		caudal_ltsxs = 0.0;

	} else {
		// Calculo el caudal instantaneo en lts/seg
		// systemVars.counter_conf.magpp: litros por pulso
		// delta_ticks: ticks en ms.
		caudal_ltsxs = systemVars.counter_conf.magpp /  delta_ticks * configTICK_RATE_HZ;
		//xprintf_P(PSTR("DEBUG: magpp=%.03f\r\n"),systemVars.counter_conf.magpp);
		//xprintf_P(PSTR("DEBUG: caudal=%.03f (lts/s)\r\n"),caudal);
		//xprintf_P(PSTR("DEBUG: caudal=%.03f (mt3/h)\r\n"), ( caudal * LTSxS_TO_MTS3xH ) );
	}

	return(caudal_ltsxs);
}
//------------------------------------------------------------------------------------
// CONTADORES
//------------------------------------------------------------------------------------
void counter_init_outofrtos(void)
{
	// Configura los timers que generan el delay de medida del ancho de pulso
	// y el periodo.
	// Se deben crear antes que las tarea y que arranque el scheduler ( main.c)

	f_counter_running = false;
	COUNTER_disable_interrupt();

	// Counter de debounce de pulsos en linea A
	// Mide el tiempo minimo que el pulso está arriba
	// Mide el pulse_width
	pwidth_xTimer = xTimerCreateStatic ("CWIDTH",
			pdMS_TO_TICKS( 10 ),
			pdFALSE,
			( void * ) 0,
			pwidth_TimerCallback,
			&pwidth_xTimerBuffer
			);

	// Mide el periodo del pulso en la linea A.
	period_xTimer = xTimerCreateStatic ("CPERIOD",
			pdMS_TO_TICKS( 90 ),
			pdFALSE,
			( void * ) 0,
			period_TimerCallback,
			&period_xTimerBuffer
			);

}
//------------------------------------------------------------------------------------
void counter_init(void)
{
	// Configuro los timers con el timeout dado por el tiempo de minimo pulse width.
	// Esto lo debo hacer aqui porque ya lei el systemVars y tengo los valores.
	// Esto arranca el timer por lo que hay que apagarlos

	if ( systemVars.counter_conf.period <= systemVars.counter_conf.pwidth ) {
		systemVars.counter_conf.period = systemVars.counter_conf.pwidth + 10;
		xprintf_P(PSTR("COUNTER ERROR!! C0: periodo debe ser mayor que el ancho\r\n\0"));
	}

	// CNT0 (PA)
	// Pulse-width
	xTimerChangePeriod( pwidth_xTimer, systemVars.counter_conf.pwidth, 10 );
	xTimerStop(pwidth_xTimer, 10);

	// Period
	xTimerChangePeriod( period_xTimer, ( systemVars.counter_conf.period ) , 10 );
	xTimerStop(period_xTimer, 10);

	COUNTER_init( systemVars.counter_conf.sensing_edge);

//	if ( strcmp ( systemVars.counter_conf.name, "X" ) == 0 ) {
//		f_counter_running = false;
//	}

	f_counter_running = true;

	p1_tick =  xTaskGetTickCount();
	p0_tick =  p1_tick;
}
//------------------------------------------------------------------------------------
void counter_config_defaults(void)
{

	// Realiza la configuracion por defecto de los canales contadores.
	// Los valores son en ms.

	systemVars.counter_conf.magpp = 1;
	systemVars.counter_conf.period = 100;
	systemVars.counter_conf.pwidth = 10;
	systemVars.counter_conf.sensing_edge = RISING_EDGE;

}
//------------------------------------------------------------------------------------
bool counter_config( char *s_magpp, char *s_pw, char *s_period, char *s_sensing )
{
	// Configuro un canal contador.
	// channel: id del canal
	// s_param0: string del nombre del canal
	// s_param1: string con el valor del factor magpp.
	//
	// {0..1} dname magPP

bool retS = false;

	// MAGPP
	if ( s_magpp != NULL ) { systemVars.counter_conf.magpp = atof(s_magpp); }

	// PW
	if ( s_pw != NULL ) { systemVars.counter_conf.pwidth = atoi(s_pw); }

	// PERIOD
	if ( s_period != NULL ) { systemVars.counter_conf.period = atoi(s_period); }


	// SENSING ( RISE/FALL )
	if ( strcmp_P( s_sensing, PSTR("RISE\0")) == 0 ) {
		 systemVars.counter_conf.sensing_edge = RISING_EDGE;

	} else if ( strcmp_P( s_sensing , PSTR("FALL\0")) == 0 ) {
		systemVars.counter_conf.sensing_edge = FALLING_EDGE;

	} else {
		xprintf_P(PSTR("ERROR: counters RISE o FALL only!!\r\n\0"));

	}

	retS = true;
	return(retS);

}
//------------------------------------------------------------------------------------
// FUNCIONES PRIVADAS
//------------------------------------------------------------------------------------
static void pwidth_TimerCallback( TimerHandle_t xTimer )
{
	// Funcion de callback de la entrada de contador.
	// Controla el pulse_width
	// Leo la entrada y si esta aun activa, incremento el contador y
	// prendo el timer xTimer1X que termine el debounce.

uint8_t confirm_value = 0;

	if ( systemVars.counter_conf.sensing_edge == RISING_EDGE ) {
		confirm_value = 1;
	}

	// Contabilizo el pulso como valido
	if ( CNT_read_CNT0() == confirm_value ) {
		caudal_acumulado++;
		p0_tick =  p1_tick;
		p1_tick =  xTaskGetTickCount();
		xTimerStart( period_xTimer, 1 );
		if ( systemVars.debug == DEBUG_COUNTER) {
			xprintf_P( PSTR("COUNTERS: DEBUG dtime=%lu ms.\r\n"), (p1_tick - p0_tick) );
		}
		return;
	}

	// No se cumplio el pulse_width minimo. No cuento el pulso y rearmo el sistema
	// para poder volver a interrumpir
	COUNTER_enable_interrupt();

}
//------------------------------------------------------------------------------------
static void period_TimerCallback( TimerHandle_t xTimer )
{
	// Se cumplio es period de la linea A (CNT0)
	// Habilito a volver a interrumpir
	COUNTER_enable_interrupt();

}
//------------------------------------------------------------------------------------
ISR( PORTB_INT0_vect )
{
	// Esta ISR se activa cuando el contador D1 (PB2) genera un flaco se subida.

	if ( !f_counter_running)
		return;

	// Aseguro arrancar el timer
	while ( xTimerStartFromISR( pwidth_xTimer, &xHigherPriorityTaskWoken ) != pdPASS )
		;

	// Deshabilito mas interrupciones. Solo controlo por timers
	COUNTER_disable_interrupt();

}
//------------------------------------------------------------------------------------
