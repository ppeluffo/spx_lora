/*
 * drv_uart_spx.c
 *
 *  Created on: 11 jul. 2018
 *      Author: pablo
 */

#include "drv_uart_spx.h"

//----------------------------------------------------------------------------------------
uart_control_t *drv_uart_init( uart_id_t iUART, uint32_t baudrate )
{
	// El puerto del USB es PORTD:
	// TXD pin = high
	// TXD pin output
	// baudrate / frame format
	// Enable TX,RX

uart_control_t *pUart = NULL;

	switch(iUART) {
	case iUART_USARTE:
		// Abro el puerto serial y fijo su velocidad
		drv_uart_usarte_open(baudrate);
		// Inicializo los ringBuffers que manejan el puerto. Son locales al driver.
		rBufferCreateStatic( &uart_usarte.RXringBuffer, &usarte_rxStorage[0], USARTE_RXSTORAGE_SIZE );
		rBufferCreateStatic( &uart_usarte.TXringBuffer, &usarte_txStorage[0], USARTE_TXSTORAGE_SIZE );
		// Asigno el identificador
		uart_usarte.uart_id = iUART_USARTE;
		uart_usarte.usart = &USARTE0;
		// Devuelvo la direccion de uart_usarte para que la asocie al dispositvo USARTE el frtos.
		pUart = (uart_control_t *)&uart_usarte;
		break;
	case iUART_USARTC:
		// Abro el puerto serial y fijo su velocidad
		drv_uart_usartc_open(baudrate);
		// Inicializo los ringBuffers que manejan el puerto. Son locales al driver.
		rBufferCreateStatic( &uart_usartc.RXringBuffer, &usartc_rxStorage[0], USARTC_RXSTORAGE_SIZE );
		rBufferCreateStatic( &uart_usartc.TXringBuffer, &usartc_txStorage[0], USARTC_TXSTORAGE_SIZE );
		// Asigno el identificador
		uart_usartc.uart_id = iUART_USARTC;
		uart_usartc.usart = &USARTC0;
		// Devuelvo la direccion de uart_usarte para que la asocie al dispositvo USARTE el frtos.
		pUart = (uart_control_t *)&uart_usartc;
		break;
	case iUART_USARTF:
		// Abro el puerto serial y fijo su velocidad
		drv_uart_usartf_open(baudrate);
		// Inicializo los ringBuffers que manejan el puerto. Son locales al driver.
		rBufferCreateStatic( &uart_usartf.RXringBuffer, &usartf_rxStorage[0], USARTF_RXSTORAGE_SIZE );
		rBufferCreateStatic( &uart_usartf.TXringBuffer, &usartf_txStorage[0], USARTF_TXSTORAGE_SIZE );
		// Asigno el identificador
		uart_usartf.uart_id = iUART_USARTF;
		uart_usartf.usart = &USARTF0;
		// Devuelvo la direccion de uart_usarte para que la asocie al dispositvo USARTE el frtos.
		pUart = (uart_control_t *)&uart_usartf;
		break;
	case iUART_USARTD:
		// Abro el puerto serial y fijo su velocidad
		drv_uart_usartd_open(baudrate);
		// Inicializo los ringBuffers que manejan el puerto. Son locales al driver.
		rBufferCreateStatic( &uart_usartd.RXringBuffer, &usartd_rxStorage[0], USARTD_RXSTORAGE_SIZE );
		rBufferCreateStatic( &uart_usartd.TXringBuffer, &usartd_txStorage[0], USARTD_TXSTORAGE_SIZE );
		// Asigno el identificador
		uart_usartd.uart_id = iUART_USARTD;
		uart_usartd.usart = &USARTD0;
		// Devuelvo la direccion de uart_usarte para que la asocie al dispositvo USARTD el frtos.
		pUart = (uart_control_t *)&uart_usartd;
		break;
	}

	return(pUart);
}
//----------------------------------------------------------------------------------------
void drv_uart_interruptOn(uart_id_t iUART)
{
	// Habilito la interrupcion TX del UART lo que hace que se ejecute la ISR_TX y
	// esta vaya a la TXqueue y si hay datos los comienze a trasmitir.

uint8_t tempCTRLA = 0;

	switch(iUART) {
	case iUART_USARTE:
		// low level, TXint enabled
		/* Enable DRE interrupt. */
		tempCTRLA = USARTE0.CTRLA;
		tempCTRLA = (tempCTRLA & ~USART_DREINTLVL_gm) | USART_DREINTLVL_LO_gc;
		USARTE0.CTRLA = tempCTRLA;
		break;
	case iUART_USARTC:
		// low level, TXint enabled
		/* Enable DRE interrupt. */
		tempCTRLA = USARTC0.CTRLA;
		tempCTRLA = (tempCTRLA & ~USART_DREINTLVL_gm) | USART_DREINTLVL_LO_gc;
		USARTC0.CTRLA = tempCTRLA;
		break;
	case iUART_USARTF:
		// low level, TXint enabled
		/* Enable DRE interrupt. */
		tempCTRLA = USARTF0.CTRLA;
		tempCTRLA = (tempCTRLA & ~USART_DREINTLVL_gm) | USART_DREINTLVL_LO_gc;
		USARTF0.CTRLA = tempCTRLA;
		break;
	case iUART_USARTD:
		// low level, TXint enabled
		/* Enable DRE interrupt. */
		tempCTRLA = USARTD0.CTRLA;
		tempCTRLA = (tempCTRLA & ~USART_DREINTLVL_gm) | USART_DREINTLVL_LO_gc;
		USARTD0.CTRLA = tempCTRLA;
		break;
	}

}
//----------------------------------------------------------------------------------------
void drv_uart_interruptOff(uart_id_t iUART)
{

uint8_t tempCTRLA = 0;

	switch(iUART) {
	case iUART_USARTE:
		// TXint disabled
		// Espero que no halla nada en el DREG
		tempCTRLA = USARTE0.CTRLA;
		tempCTRLA = (tempCTRLA & ~USART_DREINTLVL_gm) | USART_DREINTLVL_OFF_gc;
		USARTE0.CTRLA = tempCTRLA;
		break;
	case iUART_USARTC:
		// TXint disabled
		// Espero que no halla nada en el DREG
		tempCTRLA = USARTC0.CTRLA;
		tempCTRLA = (tempCTRLA & ~USART_DREINTLVL_gm) | USART_DREINTLVL_OFF_gc;
		USARTC0.CTRLA = tempCTRLA;
		break;
	case iUART_USARTF:
		// TXint disabled
		// Espero que no halla nada en el DREG
		tempCTRLA = USARTF0.CTRLA;
		tempCTRLA = (tempCTRLA & ~USART_DREINTLVL_gm) | USART_DREINTLVL_OFF_gc;
		USARTF0.CTRLA = tempCTRLA;
		break;
	case iUART_USARTD:
		// TXint disabled
		// Espero que no halla nada en el DREG
		tempCTRLA = USARTD0.CTRLA;
		tempCTRLA = (tempCTRLA & ~USART_DREINTLVL_gm) | USART_DREINTLVL_OFF_gc;
		USARTD0.CTRLA = tempCTRLA;
		break;
	}

}
//----------------------------------------------------------------------------------------
void drv_set_baudrate(uint32_t baudRate, uint8_t *baudA, uint8_t *baudB, uint8_t *ctl )
{
#if F_CPU == 32000000

	/* Set Baudrate to 115200 bps:
	 * Use the default I/O clock frequency that is 32 MHz.
	 * Los valores los extraigo de la planilla provista por Atmel
	 * 32Mhz
	 * BSEL = 2094
	 * BSCALE = -7
	 * CLK2X = 0
	 * %error = 0,01%
	 */
	switch(baudRate) {
	case 115200:
		*baudA = (uint8_t) 2094;
		*baudB = ( -7 << USART_BSCALE0_bp)|(2094 >> 8);
		break;
	case 57600:
		*baudA = (uint8_t) 2158;
		*baudB = ( -6 << USART_BSCALE0_bp)|(2158 >> 8);
		break;
	case 9600:
		// 9600
		*baudA = (uint8_t) 3317;
		*baudB = ( -4 << USART_BSCALE0_bp)|(3317 >> 8);
		break;
	}

#endif

#if F_CPU == 8000000
		/* Set Baudrate to 115200 bps:
		 * Use the default I/O clock frequency that is 32 MHz.
		 * Los valores los extraigo de la planilla provista por Atmel
		 * 8Mhz
		 * BSEL = 983
		 * BSCALE = -7
		 * CLK2X = 1
		 * %error = 0,01%
		 */
	*baudA = (uint8_t) 983;
	*baudB = ( -7 << USART_BSCALE0_bp)|(983 >> 8);
		// Habilito CLK2X
	*ctl |= USART_CLK2X_bm;
#endif

#if F_CPU == 2000000
		/* Set Baudrate to 115200 bps:
		 * Use the default I/O clock frequency that is 2 MHz.
		 * Los valores los extraigo de la planilla provista por Atmel
		 * 2Mhz
		 * BSEL = 11
		 * BSCALE = -7
		 * CLK2X = 0
		 * %error = 0,08%
		 */
		*baudA = (uint8_t) 11;
		*baudB = ( -7 << USART_BSCALE0_bp)|(11 >> 8);
#endif
}
//----------------------------------------------------------------------------------------
void drv_uart_enable_tx_int( uart_id_t iUART )
{
	// Habilita la interrrupcion por DRE

	switch(iUART) {
	case iUART_USARTE:
		USARTE0.CTRLA = (USARTE0.CTRLA & ~USART_DREINTLVL_gm) | USART_DREINTLVL_LO_gc;
		break;
	case iUART_USARTC:
		USARTC0.CTRLA = (USARTC0.CTRLA & ~USART_DREINTLVL_gm) | USART_DREINTLVL_LO_gc;
		break;
	case iUART_USARTF:
		USARTF0.CTRLA = (USARTF0.CTRLA & ~USART_DREINTLVL_gm) | USART_DREINTLVL_LO_gc;
		break;
	case iUART_USARTD:
		USARTD0.CTRLA = (USARTD0.CTRLA & ~USART_DREINTLVL_gm) | USART_DREINTLVL_LO_gc;
		break;
	}
}
//----------------------------------------------------------------------------------------
void drv_uart_disable_tx_int( uart_id_t iUART )
{
	// Deshabilita la interrrupcion por DRE

	switch(iUART) {
	case iUART_USARTE:
		USARTE0.CTRLA = (USARTE0.CTRLA & ~USART_DREINTLVL_gm) | USART_DREINTLVL_OFF_gc;
		break;
	case iUART_USARTC:
		USARTC0.CTRLA = (USARTC0.CTRLA & ~USART_DREINTLVL_gm) | USART_DREINTLVL_OFF_gc;
		break;
	case iUART_USARTF:
		USARTF0.CTRLA = (USARTF0.CTRLA & ~USART_DREINTLVL_gm) | USART_DREINTLVL_OFF_gc;
		break;
	case iUART_USARTD:
		USARTD0.CTRLA = (USARTD0.CTRLA & ~USART_DREINTLVL_gm) | USART_DREINTLVL_OFF_gc;
		break;
	}
}
//----------------------------------------------------------------------------------------
void drv_uart_enable_rx_int( uart_id_t iUART )
{
	// Habilita la interrrupcion por RXC

	switch(iUART) {
	case iUART_USARTE:
		USARTE0.CTRLA = (USARTE0.CTRLA & ~USART_RXCINTLVL_gm) | USART_RXCINTLVL_LO_gc;
		break;
	case iUART_USARTC:
		USARTC0.CTRLA = (USARTC0.CTRLA & ~USART_RXCINTLVL_gm) | USART_RXCINTLVL_LO_gc;
		break;
	case iUART_USARTF:
		USARTF0.CTRLA = (USARTF0.CTRLA & ~USART_RXCINTLVL_gm) | USART_RXCINTLVL_LO_gc;
		break;
	case iUART_USARTD:
		USARTD0.CTRLA = (USARTD0.CTRLA & ~USART_RXCINTLVL_gm) | USART_RXCINTLVL_LO_gc;
		break;
	}
}
//----------------------------------------------------------------------------------------
void drv_uart_disable_rx_int( uart_id_t iUART )
{
	// Deshabilita la interrrupcion por RXC

	switch(iUART) {
	case iUART_USARTE:
		USARTE0.CTRLA = (USARTE0.CTRLA & ~USART_RXCINTLVL_gm) | USART_RXCINTLVL_OFF_gc;
		break;
	case iUART_USARTC:
		USARTC0.CTRLA = (USARTC0.CTRLA & ~USART_RXCINTLVL_gm) | USART_RXCINTLVL_OFF_gc;
		break;
	case iUART_USARTF:
		USARTF0.CTRLA = (USARTF0.CTRLA & ~USART_RXCINTLVL_gm) | USART_RXCINTLVL_OFF_gc;
		break;
	case iUART_USARTD:
		USARTD0.CTRLA = (USARTD0.CTRLA & ~USART_RXCINTLVL_gm) | USART_RXCINTLVL_OFF_gc;
		break;
	}
}
//----------------------------------------------------------------------------------------
void drv_uart_enable_tx( uart_id_t iUART )
{
	// Enable USART transmitter

	switch(iUART) {
	case iUART_USARTE:
		USARTE0.CTRLB = (USARTE0.CTRLB |= USART_TXEN_bm);
		break;
	case iUART_USARTC:
		USARTC0.CTRLB = (USARTC0.CTRLB |= USART_TXEN_bm);
		break;
	case iUART_USARTF:
		USARTF0.CTRLB = (USARTF0.CTRLB |= USART_TXEN_bm);
		break;
	case iUART_USARTD:
		USARTD0.CTRLB = (USARTD0.CTRLB |= USART_TXEN_bm);
		break;
	}

}
//----------------------------------------------------------------------------------------
void drv_uart_disable_tx( uart_id_t iUART )
{
	// Disable USART transmitter

	switch(iUART) {
	case iUART_USARTE:
		USARTE0.CTRLB = (USARTE0.CTRLB &= ~USART_TXEN_bm);
		break;
	case iUART_USARTC:
		USARTC0.CTRLB = (USARTC0.CTRLB &= ~USART_TXEN_bm);
		break;
	case iUART_USARTF:
		USARTF0.CTRLB = (USARTF0.CTRLB &= ~USART_TXEN_bm);
		break;
	case iUART_USARTD:
		USARTD0.CTRLB = (USARTD0.CTRLB &= ~USART_TXEN_bm);
		break;
	}
}
//----------------------------------------------------------------------------------------
void drv_uart_enable_rx( uart_id_t iUART )
{
	// Enable USART receiver

	switch(iUART) {
	case iUART_USARTE:
		USARTE0.CTRLB = (USARTE0.CTRLB |= USART_RXEN_bm);
		break;
	case iUART_USARTC:
		USARTC0.CTRLB = (USARTC0.CTRLB |= USART_RXEN_bm);
		break;
	case iUART_USARTF:
		USARTF0.CTRLB = (USARTF0.CTRLB |= USART_RXEN_bm);
		break;
	case iUART_USARTD:
		USARTD0.CTRLB = (USARTD0.CTRLB |= USART_RXEN_bm);
		break;
	}
}
//----------------------------------------------------------------------------------------
void drv_uart_disable_rx( uart_id_t iUART )
{
	// Disable USART receiver

	switch(iUART) {
	case iUART_USARTE:
		USARTE0.CTRLB = (USARTE0.CTRLB &= ~USART_RXEN_bm);
		break;
	case iUART_USARTC:
		USARTC0.CTRLB = (USARTC0.CTRLB &= ~USART_RXEN_bm);
		break;
	case iUART_USARTF:
		USARTF0.CTRLB = (USARTF0.CTRLB &= ~USART_RXEN_bm);
		break;
	case iUART_USARTD:
		USARTD0.CTRLB = (USARTD0.CTRLB &= ~USART_RXEN_bm);
		break;
	}
}
//----------------------------------------------------------------------------------------
// UART USARTE:
//----------------------------------------------------------------------------------------
void drv_uart_usarte_open( uint32_t baudrate )
{
	// El puerto del USARTE es PORTE:
	// TXD pin = high
	// TXD pin output
	// baudrate / frame format
	// Enable TX,RX

uint8_t baudA = 0x00;
uint8_t baudB = 0x00;
uint8_t ctl = 0x00;

	PORTE.DIRSET   = PIN3_bm;	// PD3 (TXD0) as output.
	PORTE.DIRCLR   = PIN2_bm;	// PD2 (RXD0) as input.
	// USARTE0, 8 Data bits, No Parity, 1 Stop bit.
	USARTE0.CTRLC = (uint8_t) USART_CHSIZE_8BIT_gc | USART_PMODE_DISABLED_gc;

	//ctl = USARTE0.CTRLB;
	drv_set_baudrate( baudrate, &baudA, &baudB, &ctl);
	USARTE0.BAUDCTRLA = baudA;
	USARTE0.BAUDCTRLB = baudB;
	USARTE0.CTRLB = ctl;

	// Habilito la TX y RX
	USARTE0.CTRLB |= USART_RXEN_bm;
	USARTE0.CTRLB |= USART_TXEN_bm;

	// Habilito la interrupcion de Recepcion ( low level )
	// low level, RXint enabled
	USARTE0.CTRLA |= _BV(4);	// RXCINTLVL_0 = 1
	USARTE0.CTRLA &= ~(_BV(5));	// RXCINTLVL_1 = 0
	//USARTE0.CTRLA = ( USARTE0.CTRLA & ~USART_RXCINTLVL_gm ) | USART_RXCINTLVL_LO_gc;

	return;
}
//----------------------------------------------------------------------------------------
ISR(USARTE0_DRE_vect)
{

char cChar = ' ';
int8_t res = false;

	res = rBufferPop( &uart_usarte.TXringBuffer, (char *)&cChar );

	if( res == true ) {
		// Send the next character queued for Tx
		USARTE0.DATA = cChar;
	} else {
		// Queue empty, nothing to send.
		drv_uart_interruptOff(uart_usarte.uart_id);
	}
}
//----------------------------------------------------------------------------------------
ISR(USARTE0_RXC_vect)
{

char cChar = ' ';

	cChar = USARTE0.DATA;

	if( rBufferPokeFromISR( &uart_usarte.RXringBuffer, &cChar ) ) {
		taskYIELD();
	}
}
//----------------------------------------------------------------------------------------
// UART USARTC:
//----------------------------------------------------------------------------------------
void drv_uart_usartc_open( uint32_t baudrate )
{
	// El puerto del USARTC es PORTC:
	// TXD pin = high
	// TXD pin output
	// baudrate / frame format
	// Enable TX,RX

uint8_t baudA = 0;
uint8_t baudB = 0;
uint8_t ctl = 0;

	PORTC.DIRSET   = PIN3_bm;	// PD3 (TXD0) as output.
	PORTC.DIRCLR   = PIN2_bm;	// PD2 (RXD0) as input.
	// USARTC0, 8 Data bits, No Parity, 1 Stop bit.
	USARTC0.CTRLC = (uint8_t) USART_CHSIZE_8BIT_gc | USART_PMODE_DISABLED_gc;

	ctl = USARTC0.CTRLB;
	drv_set_baudrate( baudrate, &baudA, &baudB, &ctl);
	USARTC0.BAUDCTRLA = baudA;
	USARTC0.BAUDCTRLB = baudB;
	USARTC0.CTRLB = ctl;

	// Habilito la TX y RX
	USARTC0.CTRLB |= USART_RXEN_bm;
	USARTC0.CTRLB |= USART_TXEN_bm;

	// Habilito la interrupcion de Recepcion ( low level )
	// low level, RXint enabled
	USARTC0.CTRLA |= _BV(4);	// RXCINTLVL_0 = 1
	USARTC0.CTRLA &= ~(_BV(5));	// RXCINTLVL_1 = 0
	//USARTC0.CTRLA = USARTE0.CTRLA | USART_RXCINTLVL_LO_gc;

	return;

}
//----------------------------------------------------------------------------------------
ISR(USARTC0_DRE_vect)
{

char cChar = ' ';
int8_t res = false;

	res = rBufferPop( &uart_usartc.TXringBuffer, (char *)&cChar );

	if( res == true ) {
		// Send the next character queued for Tx
		USARTC0.DATA = cChar;
	} else {
		// Queue empty, nothing to send.
		drv_uart_interruptOff(uart_usartc.uart_id);
	}
}
//----------------------------------------------------------------------------------------
ISR(USARTC0_RXC_vect)
{

char cChar;

	cChar = USARTC0.DATA;

	if( rBufferPokeFromISR( &uart_usartc.RXringBuffer, &cChar ) ) {
		taskYIELD();
	}
}
//----------------------------------------------------------------------------------------
// UART USARTF:
//----------------------------------------------------------------------------------------
void drv_uart_usartf_open( uint32_t baudrate )
{
	// El puerto del USARTF es PORTF:
	// TXD pin = high
	// TXD pin output
	// baudrate / frame format
	// Enable TX,RX
	// Debe operar a 9600 si tiene el BT y a 115200 si es el USB para que se pueda conectar con el celular.

uint8_t baudA, baudB, ctl;

	PORTF.DIRSET   = PIN3_bm;	// PD3 (TXD0) as output.
	PORTF.DIRCLR   = PIN2_bm;	// PD2 (RXD0) as input.
	// USARTF0, 8 Data bits, No Parity, 1 Stop bit.
	USARTF0.CTRLC = (uint8_t) USART_CHSIZE_8BIT_gc | USART_PMODE_DISABLED_gc;

	ctl = USARTF0.CTRLB;
	drv_set_baudrate( baudrate, &baudA, &baudB, &ctl);
	USARTF0.BAUDCTRLA = baudA;
	USARTF0.BAUDCTRLB = baudB;
	USARTF0.CTRLB = ctl;

	// Habilito la TX y RX
	USARTF0.CTRLB |= USART_RXEN_bm;
	USARTF0.CTRLB |= USART_TXEN_bm;

	// Habilito la interrupcion de Recepcion ( low level )
	// low level, RXint enabled
	USARTF0.CTRLA |= _BV(4);	// RXCINTLVL_0 = 1
	USARTF0.CTRLA &= ~(_BV(5));	// RXCINTLVL_1 = 0
	//USARTF0.CTRLA = ( USARTF0.CTRLA & ~USART_RXCINTLVL_gm ) | USART_RXCINTLVL_LO_gc;

	return;
}
//----------------------------------------------------------------------------------------
ISR(USARTF0_DRE_vect)
{

char cChar;
int8_t res = false;

	res = rBufferPop( &uart_usartf.TXringBuffer, (char *)&cChar );

	if( res == true ) {
		// Send the next character queued for Tx
		USARTF0.DATA = cChar;
	} else {
		// Queue empty, nothing to send.
		drv_uart_interruptOff(uart_usartf.uart_id);
	}
}
//----------------------------------------------------------------------------------------
ISR(USARTF0_RXC_vect)
{

char cChar;

	cChar = USARTF0.DATA;

	if( rBufferPokeFromISR( &uart_usartf.RXringBuffer, &cChar ) ) {
		taskYIELD();
	}
}
//----------------------------------------------------------------------------------------
// UART USARTD:
//----------------------------------------------------------------------------------------
void drv_uart_usartd_open( uint32_t baudrate )
{
	// El puerto del USARTD es PORTD:
	// TXD pin = high
	// TXD pin output
	// baudrate / frame format
	// Enable TX,RX
	// Debe operar a 9600 si tiene el BT y a 115200 si es el USB para que se pueda conectar con el celular.

uint8_t baudA, baudB, ctl;

	PORTD.DIRSET   = PIN3_bm;	// PD3 (TXD0) as output.
	PORTD.DIRCLR   = PIN2_bm;	// PD2 (RXD0) as input.
	// USARTD0, 8 Data bits, No Parity, 1 Stop bit.
	USARTD0.CTRLC = (uint8_t) USART_CHSIZE_8BIT_gc | USART_PMODE_DISABLED_gc;

	ctl = USARTD0.CTRLB;
	drv_set_baudrate( baudrate, &baudA, &baudB, &ctl);
	USARTD0.BAUDCTRLA = baudA;
	USARTD0.BAUDCTRLB = baudB;
	USARTD0.CTRLB = ctl;

	// Habilito la TX y RX
	USARTD0.CTRLB |= USART_RXEN_bm;
	USARTD0.CTRLB |= USART_TXEN_bm;

	// Habilito la interrupcion de Recepcion ( low level )
	// low level, RXint enabled
	USARTD0.CTRLA |= _BV(4);	// RXCINTLVL_0 = 1
	USARTD0.CTRLA &= ~(_BV(5));	// RXCINTLVL_1 = 0

	return;
}
//----------------------------------------------------------------------------------------
ISR(USARTD0_DRE_vect)
{

char cChar;
int8_t res = false;

	res = rBufferPop( &uart_usartd.TXringBuffer, (char *)&cChar );

	if( res == true ) {
		// Send the next character queued for Tx
		USARTD0.DATA = cChar;
	} else {
		// Queue empty, nothing to send.
		drv_uart_interruptOff(uart_usartd.uart_id);
	}
}
//----------------------------------------------------------------------------------------
ISR(USARTD0_RXC_vect)
{

char cChar;

	cChar = USARTD0.DATA;

	if( rBufferPokeFromISR( &uart_usartd.RXringBuffer, &cChar ) ) {
		taskYIELD();
	}
}
//----------------------------------------------------------------------------------------


