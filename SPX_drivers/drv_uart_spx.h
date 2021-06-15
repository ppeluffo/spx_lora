/*
 * drv_uart_spx.h
 *
 *  Created on: 8 dic. 2018
 *      Author: pablo
 */

#ifndef SRC_SPX_DRIVERS_DRV_UART_SPX_H_
#define SRC_SPX_DRIVERS_DRV_UART_SPX_H_

#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "FreeRTOS.h"

#include "l_ringBuffer.h"

//-----------------------------------------------------------------------
#define USARTF_RXSTORAGE_SIZE	128
#define USARTF_TXSTORAGE_SIZE	128	// trasmito por poleo. Si uso interrupcion lo subo a 128
uint8_t usartf_rxStorage[USARTF_RXSTORAGE_SIZE];
uint8_t usartf_txStorage[USARTF_TXSTORAGE_SIZE];

#define USARTE_RXSTORAGE_SIZE	8
#define USARTE_TXSTORAGE_SIZE	8	// trasmito por poleo. Si uso interrupcion lo subo a 128
uint8_t usarte_rxStorage[USARTE_RXSTORAGE_SIZE];
uint8_t usarte_txStorage[USARTE_TXSTORAGE_SIZE];

#define USARTC_RXSTORAGE_SIZE	8
#define USARTC_TXSTORAGE_SIZE	8
uint8_t usartc_rxStorage[USARTC_RXSTORAGE_SIZE];
uint8_t usartc_txStorage[USARTC_TXSTORAGE_SIZE];

#define USARTD_RXSTORAGE_SIZE	128
#define USARTD_TXSTORAGE_SIZE	128
uint8_t usartd_rxStorage[USARTD_RXSTORAGE_SIZE];
uint8_t usartd_txStorage[USARTD_TXSTORAGE_SIZE];

// Enumenerador de los puertos fisicos.
typedef enum {
	iUART_USARTF = 0,
	iUART_USARTE,
	iUART_USARTC,
	iUART_USARTD
} uart_id_t;

// Estructura generica de una UART
typedef struct {
	uart_id_t uart_id;			// Identificador de la uart fisico
	ringBuffer_s TXringBuffer;	// ringbuffer de trasmision
	ringBuffer_s RXringBuffer;	// ringbuffer de recepcion.
	USART_t *usart;
} uart_control_t;

// Creo las uart's en memoria.
uart_control_t uart_usartf, uart_usarte, uart_usartc, uart_usartd;

//-----------------------------------------------------------------------
uart_control_t *drv_uart_init( uart_id_t iUART, uint32_t baudrate );
void drv_uart_interruptOn(uart_id_t iUART);
void drv_uart_interruptOff(uart_id_t iUART);

void drv_uart_enable_tx_int( uart_id_t iUART );
void drv_uart_disable_tx_int( uart_id_t iUART );
void drv_uart_enable_rx_int( uart_id_t iUART );
void drv_uart_disable_rx_int( uart_id_t iUART );
void drv_uart_enable_tx( uart_id_t iUART );
void drv_uart_disable_tx( uart_id_t iUART );
void drv_uart_enable_rx( uart_id_t iUART );
void drv_uart_disable_rx( uart_id_t iUART );

void drv_set_baudrate(uint32_t baudRate, uint8_t *baudA, uint8_t *baudB, uint8_t *ctl );

void drv_uart_usarte_open( uint32_t baudrate );
void drv_uart_usartc_open( uint32_t baudrate );
void drv_uart_usartf_open( uint32_t baudrate );
void drv_uart_usartd_open( uint32_t baudrate );
//-----------------------------------------------------------------------


#endif /* SRC_SPX_DRIVERS_DRV_UART_SPX_H_ */
