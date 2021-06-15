/*
 * l_ringBuffer.c
 *
 *  Created on: 11 jul. 2018
 *      Author: pablo
 */


#include "l_ringBuffer.h"

//------------------------------------------------------------------------------------
bool rBufferPoke( ringBuffer_s *rB, char *cChar )
{

	// Inserta un elemento en el ringBuffer.
	// Si el buffer esta lleno, descarto el valor recibido
	// Guardo en el lugar apuntado por head y avanzo.

bool ret = false;

	taskENTER_CRITICAL();

	// Si el buffer esta vacio ajusto los punteros
	if( rB->count == 0) {
		rB->head = rB->tail = 0;
	}

	if ( rB->count < rB->length ) {
		rB->buff[rB->head] = *cChar;
		++rB->count;
		// Avanzo en modo circular
		rB->head = ( rB->head  + 1 ) % ( rB->length );
		ret = true;
    }

	taskEXIT_CRITICAL();
	return(ret);

}
//------------------------------------------------------------------------------------
bool rBufferPokeFromISR( ringBuffer_s *rB, char *cChar )
{

	// Igual a rBufferPoke pero no uso los CRITICAL porque ya estoy
	// dentro de una ISR

bool ret = false;

	// Si el buffer esta vacio ajusto los punteros
	if( rB->count == 0) {
		rB->head = rB->tail = 0;
	}

	if ( rB->count < rB->length ) {
		rB->buff[rB->head] = *cChar;
		++rB->count;
		// Avanzo en modo circular
		rB->head = ( rB->head  + 1 ) % ( rB->length );
		ret = true;
    }

	return(ret);

}
//------------------------------------------------------------------------------------
bool rBufferPop( ringBuffer_s *rB, char *cChar )
{
	// Saco un caracter del ringbuffer y lo retorno

bool ret = false;

	taskENTER_CRITICAL();

	//  Si el buffer esta vacio retorno.
	if( rB->count == 0) {
		rB->head = rB->tail = 0;
		taskEXIT_CRITICAL();
		return(ret);
	}

	*cChar = rB->buff[rB->tail];
	--rB->count;
	// Avanzo en modo circular
	rB->tail = ( rB->tail  + 1 ) % ( rB->length );
	ret = true;

	taskEXIT_CRITICAL();
	return(ret);
}
//------------------------------------------------------------------------------------
bool rBufferPopFromISR( ringBuffer_s *rB, char *cChar )
{

	// Igual a rBufferPop pero no uso los CRITICAL porque ya estoy
	// dentro de una ISR

bool ret = false;

	//  Si el buffer esta vacio retorno.
	if( rB->count == 0) {
		rB->head = rB->tail = 0;
		return(ret);
	}

	*cChar = rB->buff[rB->tail];
	--rB->count;
	// Avanzo en modo circular
	rB->tail = ( rB->tail  + 1 ) % ( rB->length );
	ret = true;

	return(ret);
}
//------------------------------------------------------------------------------------
uint16_t rBufferGetCount( ringBuffer_s *rB )
{
	// Devuelvo cuantos datos hay en el ringbuffer

	return(rB->count);

}
//------------------------------------------------------------------------------------
uint16_t rBufferGetFreeCount( ringBuffer_s *rB )
{

	// Devuelvo cuanto espacio libre tengo

	return(rB->length - rB->count);

}
//------------------------------------------------------------------------------------
bool rBufferReachLowWaterMark( ringBuffer_s *rB)
{

		// Devuelvo true si estoy por debajo del 20% de ocupacion del espacio

bool retS = false;

	if ( rB->count  < (int)(0.2 * rB->length  ))
		retS = true;
	return(retS);

}
//------------------------------------------------------------------------------------
bool rBufferReachHighWaterMark( ringBuffer_s *rB )
{
	// Devuelvo True si estoy por encima del 80% del espacio

bool retS = false;

	if ( rB->count  > (int)(0.8 * rB->length ))
		retS = true;

	return(retS);

}
//------------------------------------------------------------------------------------
void rBufferCreateStatic ( ringBuffer_s *rB, uint8_t *storage_area, uint16_t size  )
{
	// Crea la estructura del ring buffer.
	// Se le pasa el storage_area para no usar malloc

	rB->buff = storage_area;
	rB->head = 0;	// start
	rB->tail = 0;	// end
	rB->count = 0;
	rB->length = size;
}
//------------------------------------------------------------------------------------
void rBufferFlush( ringBuffer_s *rB )
{
	// Inicializa el ring buffer en un contexto de FRTOS

	taskENTER_CRITICAL();

	rB->head = 0;
	rB->tail = 0;
	rB->count = 0;
	memset(rB->buff,'\0', rB->length );

	taskEXIT_CRITICAL();
}
//------------------------------------------------------------------------------------


