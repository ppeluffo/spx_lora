/*
 * spx_utis.c
 *
 *  Created on: 10 dic. 2018
 *      Author: pablo
 */

#include "spx.h"

#define RTC32_ToscBusy()        !( VBAT.STATUS & VBAT_XOSCRDY_bm )

//------------------------------------------------------------------------------------
void initMCU(void)
{
	// Inicializa los pines del micro

	// PWR_SLEEP
//	IO_config_PWR_SLEEP();
//	IO_set_PWR_SLEEP();

	// Configuro los pines del AUX port
//	IO_config_AUX_PWR();
//	IO_config_XBEE_SLEEP();
//	IO_config_XBEE_RESET();

	// ANALOG: SENSOR VCC CONTROL
	//IO_config_SENS_12V_CTL();

	IO_config_LORA_RESET();
	IO_clr_LORA_RESET();

}
//------------------------------------------------------------------------------------
void RTC32_ToscEnable( bool use1khz );
//------------------------------------------------------------------------------------
void u_configure_systemMainClock(void)
{
/*	Configura el clock principal del sistema
	Inicialmente se arranca en 2Mhz.
	La configuracion del reloj tiene 2 componentes: el clock y el oscilador.
	OSCILADOR:
	Primero debo elejir cual oscilador voy a usar para alimentar los prescalers que me den
	el clock del sistema y esperar a que este este estable.
	CLOCK:
	Elijo cual oscilador ( puedo tener mas de uno prendido ) va a ser la fuente principal
	del closck del sistema.
	Luego configuro el prescaler para derivar los clocks de los perifericos.
	Puedo por ultimo 'lockear' esta configuracion para que no se cambie por error.
	Los registros para configurar el clock son 'protegidos' por lo que los cambio
	utilizando la funcion CCPwrite.

	Para nuestra aplicacion vamos a usar un clock de 32Mhz.
	Como vamos a usar el ADC debemos prestar atencion al clock de perifericos clk_per ya que luego
	el ADC clock derivado del clk_per debe estar entre 100khz y 1.4Mhz ( AVR1300 ).

	Opcionalmente podriamos deshabilitar el oscilador de 2Mhz para ahorrar energia.
*/

#if SYSMAINCLK == 32
	// Habilito el oscilador de 32Mhz
	OSC.CTRL |= OSC_RC32MEN_bm;

	// Espero que este estable
	do {} while ( (OSC.STATUS & OSC_RC32MRDY_bm) == 0 );

	// Seteo el clock para que use el oscilador de 32Mhz.
	// Uso la funcion CCPWrite porque hay que se cuidadoso al tocar estos
	// registros que son protegidos.
	CCPWrite(&CLK.CTRL, CLK_SCLKSEL_RC32M_gc);
	//
	// El prescaler A ( CLK.PSCCTRL ), B y C ( PSBCDIV ) los dejo en 0 de modo que no
	// hago division y con esto tengo un clk_per = clk_sys. ( 32 Mhz ).
	//
#endif

#if SYSMAINCLK == 8
	// Habilito el oscilador de 32Mhz y lo divido por 4
	OSC.CTRL |= OSC_RC32MEN_bm;

	// Espero que este estable
	do {} while ( (OSC.STATUS & OSC_RC32MRDY_bm) == 0 );

	// Seteo el clock para que use el oscilador de 32Mhz.
	// Uso la funcion CCPWrite porque hay que se cuidadoso al tocar estos
	// registros que son protegidos.
	CCPWrite(&CLK.CTRL, CLK_SCLKSEL_RC32M_gc);
	//
	// Pongo el prescaler A por 4 y el B y C en 0.
	CLKSYS_Prescalers_Config( CLK_PSADIV_4_gc, CLK_PSBCDIV_1_1_gc );

	//
#endif

#if SYSMAINCLK == 2
	// Este es el oscilador por defecto por lo cual no tendria porque configurarlo.
	// Habilito el oscilador de 2Mhz
	OSC.CTRL |= OSC_RC2MEN_bm;
	// Espero que este estable
	do {} while ( (OSC.STATUS & OSC_RC2MRDY_bm) == 0 );

	// Seteo el clock para que use el oscilador de 2Mhz.
	// Uso la funcion CCPWrite porque hay que se cuidadoso al tocar estos
	// registros que son protegidos.
	CCPWrite(&CLK.CTRL, CLK_SCLKSEL_RC2M_gc);
	//
	// El prescaler A ( CLK.PSCCTRL ), B y C ( PSBCDIV ) los dejo en 0 de modo que no
	// hago division y con esto tengo un clk_per = clk_sys. ( 2 Mhz ).
	//
#endif

//#ifdef configUSE_TICKLESS_IDLE
	// Para el modo TICKLESS
	// Configuro el RTC con el osc externo de 32Khz
	// Pongo como fuente el xtal externo de 32768 contando a 32Khz.
	//CLK.RTCCTRL = CLK_RTCSRC_TOSC32_gc | CLK_RTCEN_bm;
	//do {} while ( ( RTC.STATUS & RTC_SYNCBUSY_bm ) );

	// Disable RTC interrupt.
	// RTC.INTCTRL = 0x00;
	//
	// Si uso el RTC32, habilito el oscilador para 1ms.

	RTC32_ToscEnable(true);
//#endif

	// Lockeo la configuracion.
	CCPWrite( &CLK.LOCK, CLK_LOCK_bm );

}
//------------------------------------------------------------------------------------
void u_configure_RTC32(void)
{
	// El RTC32 lo utilizo para desperarme en el modo tickless.
	// V-bat needs to be reset, and activated
	VBAT.CTRL |= VBAT_ACCEN_bm;
	// Este registro esta protegido de escritura con CCP.
	CCPWrite(&VBAT.CTRL, VBAT_RESET_bm);

	// Pongo el reloj en 1.024Khz.
	VBAT.CTRL |=  VBAT_XOSCSEL_bm | VBAT_XOSCFDEN_bm ;

	// wait for 200us see AVR1321 Application note page 8
	_delay_us(200);

	// Turn on 32.768kHz crystal oscillator
	VBAT.CTRL |= VBAT_XOSCEN_bm;

	// Wait for stable oscillator
	while(!(VBAT.STATUS & VBAT_XOSCRDY_bm));

	// Disable RTC32 module before setting counter values
	RTC32.CTRL = 0;

	// Wait for sync
	do { } while ( RTC32.SYNCCTRL & RTC32_SYNCBUSY_bm );

	// EL RTC corre a 1024 hz y quiero generar un tick de 10ms,
	RTC32.PER = 1024;
	RTC32.CNT = 0;

	// Interrupt: on Overflow
	RTC32.INTCTRL = RTC32_OVFINTLVL_LO_gc;

	// Enable RTC32 module
	RTC32.CTRL = RTC32_ENABLE_bm;

	/* Wait for sync */
	do { } while ( RTC32.SYNCCTRL & RTC32_SYNCBUSY_bm );
}
//------------------------------------------------------------------------------------
void RTC32_ToscEnable( bool use1khz )
{
	/* Enable 32 kHz XTAL oscillator, with 1 kHz or 1 Hz output. */
	if (use1khz)
		VBAT.CTRL |= ( VBAT_XOSCEN_bm | VBAT_XOSCSEL_bm );
	else
		VBAT.CTRL |= ( VBAT_XOSCEN_bm );

	RTC32.PER = 10;
	RTC32.CNT = 0;

	/* Wait for oscillator to stabilize before returning. */
//	do { } while ( RTC32_ToscBusy() );
}
//------------------------------------------------------------------------------------
uint8_t u_checksum( uint8_t *s, uint16_t size )
{
	/*
	 * Recibe un puntero a una estructura y un tamaño.
	 * Recorre la estructura en forma lineal y calcula el checksum
	 */

uint8_t *p = NULL;
uint8_t checksum = 0;
uint16_t i = 0;

	checksum = 0;
	p = s;
	for ( i = 0; i < size ; i++) {
		checksum += p[i];
	}
	checksum = ~checksum;
	return(checksum);
}
//------------------------------------------------------------------------------------
void u_load_defaults( char *opt )
{
	// Carga la configuracion por defecto.

	systemVars.debug = DEBUG_NONE;
	systemVars.timer_poll = 300;

	psensor_config_defaults();
	counter_config_defaults();
	//lora_config_defaults();
	lora_config_keys_testing();

}
//------------------------------------------------------------------------------------
void u_save_params_in_NVMEE(void)
{
	/*
	 *  Tengo 3 estructuras de variables que debo guardar:
	 *  systemVars, sVarsApp, sVarsComms
	 *  Las guardo en este orden y luego de grabarlas, grabo un byte con el checksum de c/u
	 *
	 *  sVarsComms.checksum
	 *  sVarsComms
	 *  sVarsApp.checksum
	 *  sVarsApp
	 *  systemVars.checksum
	 *  systemVars	ADD 0x00
	 *
	 */


uint8_t checksum = 0;
uint16_t ee_wr_addr;

	// SystemVars.
	// Guardo systemVars en la EE
	ee_wr_addr = 0x00;
	nvm_eeprom_erase_and_write_buffer(ee_wr_addr, &systemVars, sizeof(systemVars));

	ee_wr_addr += sizeof(systemVars);
	checksum = u_checksum( (uint8_t *)&systemVars, sizeof(systemVars) );
	nvm_eeprom_erase_and_write_buffer(ee_wr_addr, &checksum, sizeof(checksum));

	// sVarsComms
	//ee_wr_addr += 1;
	//nvm_eeprom_erase_and_write_buffer(ee_wr_addr, &sVarsComms, sizeof(sVarsComms));

	//ee_wr_addr += sizeof(sVarsComms);
	//checksum = u_checksum( (uint8_t *)&sVarsComms, sizeof(sVarsComms) );
	//nvm_eeprom_erase_and_write_buffer(ee_wr_addr, &checksum, sizeof(checksum));

}
//------------------------------------------------------------------------------------
bool u_load_params_from_NVMEE(void)
{
	/*
	 * Leo el systemVars desde la EE.
	 * Calculo el checksum. Si no coincide es que hubo algun
	 * error por lo que cargo el default.
	 * Hago el proceso inverso de save
	 */

uint8_t stored_checksum;
uint8_t calculated_checksum;
uint16_t ee_rd_addr;

	// systemVars.
	ee_rd_addr = 0x00;
	nvm_eeprom_read_buffer(ee_rd_addr, (char *)&systemVars, sizeof(systemVars));
	calculated_checksum = u_checksum( (uint8_t *)&systemVars, sizeof(systemVars) );
	ee_rd_addr += sizeof(systemVars);
	nvm_eeprom_read_buffer(ee_rd_addr, (char *)&stored_checksum, sizeof(stored_checksum));
	if ( calculated_checksum != stored_checksum ) {
		xprintf_P( PSTR("ERROR: Checksum systemVars failed: calc[0x%0x], sto[0x%0x]\r\n"), calculated_checksum, stored_checksum );
		return(false);
	}

	// sVarsComms
	//ee_rd_addr += 1;
	//nvm_eeprom_read_buffer(ee_rd_addr, (char *)&sVarsComms, sizeof(sVarsComms));
	//calculated_checksum = u_checksum( (uint8_t *)&sVarsComms, sizeof(sVarsComms) );
	//ee_rd_addr += sizeof(sVarsComms);
	//nvm_eeprom_read_buffer(ee_rd_addr, (char *)&stored_checksum, sizeof(stored_checksum));
	//if ( calculated_checksum != stored_checksum ) {
	//	xprintf_P( PSTR("ERROR: Checksum sVarsComms failed: calc[0x%0x], sto[0x%0x]\r\n"), calculated_checksum, stored_checksum );
	//	return(false);
	//}

	return(true);
}
//------------------------------------------------------------------------------------
void u_config_timerpoll ( char *s_timerpoll )
{
	// Configura el tiempo de poleo.
	// Se utiliza desde el modo comando como desde el modo online
	// El tiempo de poleo debe estar entre 15s y 3600s

	//xprintf_P( PSTR("DEBUG_A TPOLL CONFIG: [%s]\r\n\0"), s_timerpoll );

	systemVars.timer_poll = atoi(s_timerpoll);

	if ( systemVars.timer_poll < 15 )
		systemVars.timer_poll = 15;

	if ( systemVars.timer_poll > 3600 )
		systemVars.timer_poll = 300;


	return;
}
//------------------------------------------------------------------------------------
void u_print_dr(file_descriptor_t fd, u_dataRecord_t *dr, uint16_t ctl )
{

	// Muestra en CONSOLA el frame.

	// timeStamp.
	//xfprintf_P(fd, PSTR("CTL:%d;"),ctl );

	xfprintf_P(fd, PSTR(">PA:%.02f;"), dr->presion );

	xfprintf_P(fd, PSTR("Q0:%.02f<"), dr->caudal );

	// TAIL
	// Esto es porque en gprs si mando un cr corto el socket !!!
	if ( fd == fdTERM ) {
		xfprintf_P(fd, PSTR("\r\n\0") );
	}
}
//------------------------------------------------------------------------------------
