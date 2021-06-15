/*
 * l_psensor.c
 *
 *  Created on: 23 ago. 2019
 *      Author: pablo
 *
 *  El sensor mide presion diferencial.
 *  Por lo tanto retorna PRESION de acuerdo a la funcion de transferencia
 *  con la pMax la de la hoja de datos.
 *  De acuerdo a la hoja de datos, la funcion de transferencia es:
 *  p(psi) = (pmax - pmin ) * ( counts - 0.1Max) / ( 0.8Max) + pmin
 *  pmin = 0
 *  Max = 16384 ( 14 bits )
 *  0.1xMax = 1638
 *  0.8xMax = 13107
 *  counts es el valor leido del sensor.
 *  PMAX = 1.0 psi
 *  PMIN = 0 psi
 *
 */

#include "l_abp.h"

//------------------------------------------------------------------------------------
int8_t abp_raw_read( uint8_t sensor_id, char *data )
{

	// En estos sensores no llevan el ciclo Dummy Write. !!
	// Solo se leen 2 caracteres.
	// Se lee del bus I2C en modo raw ( 2 bytes ) que no son interpretados
	// Los 2 sensores estan atras del muxi2c por lo tanto debemos activar la salida
	// adecuada al sensor que queremos leer.

int8_t rcode = -1;
uint8_t times = 1;

	// Activo el canal del muxI2C para el sensor_id
//	if ( ! I2CMUX_enable_channel(sensor_id) ) {
//		return(-1);
//	}

	// Leo en modo raw el sensor

	while ( times-- > 0 ) {

		// Leo 2 bytes del sensor de presion.
		I2C_get_semaphore();
		rcode =  I2C_read_R1( BUSADDR_ABP, data, 0x02 );
		I2C_release_semaphore();

		if ( rcode == -1 ) {
			// Hubo error: trato de reparar el bus y reintentar la operacion
			// Espero 1s que se termine la fuente de ruido.
			vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
			// Reconfiguro los dispositivos I2C del bus que pueden haberse afectado
			xprintf_P(PSTR("ERROR: ABP_raw_read recovering i2c bus (%d)\r\n\0"), times );
			I2C_reinit_devices();
		} else {
			// No hubo error: salgo normalmente
			break;
		}
	}

	return( rcode );
}
//------------------------------------------------------------------------------------
bool abp_raw_read_test( uint8_t sensor_id )
{
	// Wrapper de la funcion abp_raw_read para cmdMode.
	// Sensor Honeywll ABPMANV150PG2A3
	// Invocada desde cmdMode: interpreta los resultados raw del sensor
	// Debe habilitar el canal correspondiente del i2cmux.


bool retS = false;
uint8_t data[2];
uint8_t status;
uint16_t p_counts;

	if (  abp_raw_read( sensor_id, (char *)data ) > 0 ) {
		retS = true;
		xprintf_P(PSTR("ABP_LOW : 0x%02X\r\n"), data[1]);
		xprintf_P(PSTR("ABP_HIGH: 0x%02X\r\n"), data[0]);

		status = data[0] >> 6;
		xprintf_P(PSTR("Status: 0x%02X\r\n"), status);

		p_counts = ((data[0] & 0x3F) << 8) + data[1];
		xprintf_P(PSTR("Pcounts: %05d (0x%04X)\r\n"), p_counts, p_counts );

	} else {
		xprintf_P( PSTR("abp_test_raw_read ERROR\r\n"));
	}

	return(retS);
}
//------------------------------------------------------------------------------------

