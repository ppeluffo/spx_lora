/*
 * frtos-io.c
 *
 *  Created on: 11 jul. 2018
 *      Author: pablo
 */

#include "frtos-io.h"

//#define USARTE_IsTXDataRegisterEmpty() ((USARTE0.STATUS & USART_DREIF_bm) != 0)

#define USART_IsTXDataRegisterEmpty(_usart) (((_usart)->STATUS & USART_DREIF_bm) != 0)
#define USART_IsTXShiftRegisterEmpty(_usart) (((_usart)->STATUS & USART_TXCIF_bm) != 0)

#define USART_PutChar(_usart, _data) ((_usart)->DATA = _data)

#define USARTE_USART	USARTE0
//------------------------------------------------------------------------------------
int frtos_open( file_descriptor_t fd, uint32_t flags)
{
	// Funcion general para abrir el puerto que invoca a una mas
	// especializada para c/periferico.
	// Es la que invoca la aplicacion.
	// Retorna -1 en error o un nro positivo ( fd )

int8_t xRet = -1;

	switch(fd) {
	case fdUSARTE: // @suppress("Symbol is not resolved")
		xRet = frtos_uart_open( &xComUSARTE, fd, iUART_USARTE, flags );
		break;
	case fdUSARTC:
		xRet = frtos_uart_open( &xComUSARTC, fd, iUART_USARTC, flags );
		break;
	case fdUSARTF:
		xRet = frtos_uart_open( &xComUSARTF, fd, iUART_USARTF, flags );
		break;
	case fdUSARTD:
		xRet = frtos_uart_open( &xComUSARTD, fd, iUART_USARTD, flags );
		break;
	case fdI2C:
		xRet = frtos_i2c_open( &xBusI2C, fd, &I2C_xMutexBuffer, flags );
		break;
	default:
		break;
	}

	return(xRet);
}
//------------------------------------------------------------------------------------
int frtos_ioctl( file_descriptor_t fd, uint32_t ulRequest, void *pvValue )
{
int8_t xRet = -1;

	switch(fd) {
	case fdUSARTE:
		xRet = frtos_uart_ioctl( &xComUSARTE, ulRequest, pvValue );
		break;
	case fdUSARTC:
		xRet = frtos_uart_ioctl( &xComUSARTC, ulRequest, pvValue );
		break;
	case fdUSARTF:
		xRet = frtos_uart_ioctl( &xComUSARTF, ulRequest, pvValue );
		break;
	case fdUSARTD:
		xRet = frtos_uart_ioctl( &xComUSARTD, ulRequest, pvValue );
		break;
	case fdI2C:
		xRet = frtos_i2c_ioctl( &xBusI2C, ulRequest, pvValue );
		break;
	default:
		break;
	}

	return(xRet);
}
//------------------------------------------------------------------------------------
int frtos_write( file_descriptor_t fd ,const char *pvBuffer, const uint16_t xBytes )
{
int xRet = -1;

	switch(fd) {
	case fdUSARTE:
		//xRet = frtos_uart_write( &xComUSARTE, pvBuffer, xBytes );
		xRet = frtos_uart_write( &xComUSARTE, pvBuffer, xBytes );
		break;
	case fdUSARTC:
		xRet = frtos_uart_write( &xComUSARTC, pvBuffer, xBytes );
		break;
	case fdUSARTF:
		xRet = frtos_uart_write( &xComUSARTF, pvBuffer, xBytes );
		break;
	case fdUSARTD:
		xRet = frtos_uart_write( &xComUSARTD, pvBuffer, xBytes );
		break;
	case fdI2C:
		xRet = frtos_i2c_write( &xBusI2C, pvBuffer, xBytes );
		break;
	case fdFILE:
//		xRet = frtos_file_write( pvBuffer, xBytes );
		break;
	default:
		break;
	}

	return(xRet);
}
//------------------------------------------------------------------------------------
void frtos_putchar( file_descriptor_t fd ,const char cChar )
{
	// Escribe en el puerto (serial) en modo transparente.

	switch(fd) {
	case fdUSARTE:
		// Espero que este libre para transmitir
		// Transmito.
		USARTE0.DATA = cChar;
		while ( ( USARTE0.STATUS & USART_DREIF_bm) == 0)
			vTaskDelay( ( TickType_t)( 1 ) );
		break;
	case fdUSARTC:
		while ( ( USARTC0.STATUS & USART_DREIF_bm) == 0)
			vTaskDelay( ( TickType_t)( 1 ) );
		USARTC0.DATA = cChar;
		break;
	case fdUSARTF:
		while ( ( USARTF0.STATUS & USART_DREIF_bm) == 0)
			vTaskDelay( ( TickType_t)( 1 ) );
		USARTF0.DATA = cChar;
		break;
	case fdUSARTD:
		while ( ( USARTD0.STATUS & USART_DREIF_bm) == 0)
			vTaskDelay( ( TickType_t)( 1 ) );
		USARTD0.DATA = cChar;
		break;
	case fdI2C:
		break;
	default:
		break;
	}

}
//------------------------------------------------------------------------------------
int frtos_read( file_descriptor_t fd , char *pvBuffer, uint16_t xBytes )
{
int xRet = -1;

	switch(fd) {
	case fdUSARTE:
		xRet = frtos_uart_read( &xComUSARTE, pvBuffer, xBytes );
		break;
	case fdUSARTC:
		xRet = frtos_uart_read( &xComUSARTC, pvBuffer, xBytes );
		break;
	case fdUSARTF:
		xRet = frtos_uart_read( &xComUSARTF, pvBuffer, xBytes );
		break;
	case fdUSARTD:
		xRet = frtos_uart_read( &xComUSARTD, pvBuffer, xBytes );
		break;
	case fdI2C:
		xRet = frtos_i2c_read( &xBusI2C, pvBuffer, xBytes );
		break;
	default:
		break;
	}

	return(xRet);
}
//------------------------------------------------------------------------------------
// FUNCIONES ESPECIFICAS DE UART's
//------------------------------------------------------------------------------------
int frtos_uart_open( periferico_serial_port_t *xCom, file_descriptor_t fd, uart_id_t uart_id, uint32_t flags)
{

	xCom->fd = fd;
	xCom->xBlockTime = (10 / portTICK_RATE_MS );
	// Inicializo la uart del usb (iUART_USB) y la asocio al periferico
	xCom->uart = drv_uart_init( uart_id, flags );

	return(xCom->fd);

}
//------------------------------------------------------------------------------------
int frtos_uart_write( periferico_serial_port_t *xCom, const char *pvBuffer, const uint16_t xBytes )
{
	// Esta funcion debe poner los caracteres apuntados en pvBuffer en la cola de trasmision del
	// puerto serial apuntado por xCom
	// Actua como si fuese rprintfStr.
	// Debe tomar el semaforo antes de trasmitir. Los semaforos los manejamos en la capa FreeRTOS
	// y no en la de los drivers.

char cChar = '\0';
char *p = NULL;
uint16_t bytes2tx = 0;
int wBytes = 0;

	// Controlo no hacer overflow en la cola de trasmision
	bytes2tx = xBytes;

	// Trasmito.
	// Espero que los buffers esten vacios. ( La uart se va limpiando al trasmitir )
	while  ( rBufferGetCount( &xCom->uart->TXringBuffer ) > 0 )
		vTaskDelay( ( TickType_t)( 1 ) );

	// Cargo el buffer en la cola de trasmision.
	p = (char *)pvBuffer;
	while (*p && (bytes2tx-- > 0) ) {

		// Voy cargando la cola de a uno.
		cChar = *p;
		rBufferPoke( &xCom->uart->TXringBuffer, &cChar  );
		//rBufferPoke( &uart_usb.TXringBuffer, &cChar  );
		p++;
		wBytes++;	// Cuento los bytes que voy trasmitiendo

		// Si la cola esta llena, empiezo a trasmitir y espero que se vacie.
		if (  rBufferReachHighWaterMark( &xCom->uart->TXringBuffer ) ) {
			// Habilito a trasmitir para que se vacie
			drv_uart_interruptOn( xCom->uart->uart_id );
			// Y espero que se haga mas lugar.
			while ( ! rBufferReachLowWaterMark( &xCom->uart->TXringBuffer ) )
				vTaskDelay( ( TickType_t)( 1 ) );
		}
	}

	// Luego inicio la trasmision invocando la interrupcion.
	drv_uart_interruptOn( xCom->uart->uart_id );

	// Espero que trasmita todo
	while  ( rBufferGetCount( &xCom->uart->TXringBuffer ) > 0 )
		vTaskDelay( ( TickType_t)( 1 ) );

	return (wBytes);
}
//------------------------------------------------------------------------------------
int frtos_uart_ioctl( periferico_serial_port_t *xCom, uint32_t ulRequest, void *pvValue )
{

int xReturn = 0;

	switch( ulRequest )
	{

		case ioctl_SET_TIMEOUT:
			xCom->xBlockTime = *((uint8_t *)pvValue);
			break;
		case ioctl_UART_CLEAR_RX_BUFFER:
			rBufferFlush(&xCom->uart->RXringBuffer);
			break;
		case ioctl_UART_CLEAR_TX_BUFFER:
			rBufferFlush(&xCom->uart->TXringBuffer);
			break;
		case ioctl_UART_ENABLE_TX_INT:
			drv_uart_enable_tx_int( xCom->uart->uart_id );
			break;
		case ioctl_UART_DISABLE_TX_INT:
			drv_uart_disable_tx_int( xCom->uart->uart_id );
			break;
		case ioctl_UART_ENABLE_RX_INT:
			drv_uart_enable_rx_int( xCom->uart->uart_id );
			break;
		case ioctl_UART_DISABLE_RX_INT:
			drv_uart_disable_rx_int( xCom->uart->uart_id );
			break;
		case ioctl_UART_ENABLE_TX:
			drv_uart_enable_tx( xCom->uart->uart_id );
			break;
		case ioctl_UART_DISABLE_TX:
			drv_uart_disable_tx( xCom->uart->uart_id );
			break;
		case ioctl_UART_ENABLE_RX:
			drv_uart_enable_rx( xCom->uart->uart_id );
			break;
		case ioctl_UART_DISABLE_RX:
			drv_uart_disable_rx( xCom->uart->uart_id );
			break;
		default :
			xReturn = -1;
			break;
	}

	return xReturn;

}
//------------------------------------------------------------------------------------
int frtos_uart_read( periferico_serial_port_t *xCom, char *pvBuffer, uint16_t xBytes )
{
	// Lee caracteres de la cola de recepcion y los deja en el buffer.
	// El timeout lo fijo con ioctl.

int xBytesReceived = 0U;
portTickType xTicksToWait = 0;
xTimeOutType xTimeOut;

	xTicksToWait = 10;
	vTaskSetTimeOutState( &xTimeOut );

	// Are there any more bytes to be received?
	while( xBytesReceived < xBytes )
	{

		if( rBufferPop( &xCom->uart->RXringBuffer, &((char *)pvBuffer)[ xBytesReceived ] ) == true ) {
			xBytesReceived++;
			taskYIELD();
		} else {
			// Espero xTicksToWait antes de volver a chequear
			vTaskDelay( ( TickType_t)( xTicksToWait ) );
		}

		// Time out has expired ?
		if( xTaskCheckForTimeOut( &xTimeOut, &xTicksToWait ) != pdFALSE )
		{
			break;
		}
	}

	return ( xBytesReceived );

}
//------------------------------------------------------------------------------------
// FUNCIONES ESPECIFICAS DEL BUS I2C/TWI
//------------------------------------------------------------------------------------
int frtos_i2c_open( periferico_i2c_port_t *xI2c, file_descriptor_t fd, StaticSemaphore_t *i2c_semph, uint32_t flags)
{
	// Asigno las funciones particulares ed write,read,ioctl
	xI2c->fd = fd;
	xI2c->xBusSemaphore = xSemaphoreCreateMutexStatic( i2c_semph );
	xI2c->xBlockTime = (10 / portTICK_RATE_MS );
	xI2c->i2c_error_code = I2C_OK;
	//
	// Abro e inicializo el puerto I2C solo la primera vez que soy invocado
	drv_I2C_init();

	return(1);
}
//------------------------------------------------------------------------------------
int frtos_i2c_write( periferico_i2c_port_t *xI2c, const char *pvBuffer, const uint16_t xBytes )
{
int xReturn = 0U;


#ifdef DEBUG_I2C
//	xprintf_P( PSTR("FRTOS_I2C_WR: 0x%02x,0x%02x,0x%02x,0x%02x\r\n\0"), &xI2c->devAddress, &xI2c->chipAddressLength, &xI2c->chipAddress, xBytes);
#endif

	if ( ( xReturn = drv_I2C_master_write_R1(xI2c->devAddress, xI2c->chipAddress, xI2c->chipAddressLength,  (char *)pvBuffer, xBytes) ) >= 0 ) {
		xI2c->i2c_error_code = I2C_OK;
	} else {
		// Error de escritura indicado por el driver.
		xI2c->i2c_error_code = I2C_WR_ERROR;
	}

	return(xReturn);

}
//------------------------------------------------------------------------------------
int frtos_i2c_ioctl( periferico_i2c_port_t *xI2c, uint32_t ulRequest, void *pvValue )
{

int xReturn = 0;
uint32_t *p = NULL;

	p = pvValue;

#ifdef DEBUG_I2C
//	xprintf_P( PSTR("FRTOS_I2C_IOCTL: 0x%02x,0x%02x\r\n\0"),(uint8_t)ulRequest, (uint8_t)(*p));
#endif

	switch( ulRequest )
	{
		case ioctl_OBTAIN_BUS_SEMPH:
			// Espero el semaforo en forma persistente.
			while ( xSemaphoreTake(xI2c->xBusSemaphore, ( TickType_t ) 5 ) != pdTRUE )
				taskYIELD();
			break;
			case ioctl_RELEASE_BUS_SEMPH:
				xSemaphoreGive( xI2c->xBusSemaphore );
				break;
			case ioctl_SET_TIMEOUT:
				xI2c->xBlockTime = *p;
				break;
			case ioctl_I2C_SET_DEVADDRESS:
				xI2c->devAddress = (int8_t)(*p);
				break;
			case ioctl_I2C_SET_CHIPADDRESS:
				xI2c->chipAddress = (uint32_t)(*p);
				break;
			case ioctl_I2C_SET_CHIPADDRESSLENGTH:
				xI2c->chipAddressLength = (int8_t)(*p);
				break;
			case ioctl_I2C_GET_LAST_ERROR:
				xReturn = xI2c->i2c_error_code;
				break;
			case ioctl_I2C_SCAN:
				xReturn = drv_I2C_scan_device(xI2c->devAddress );
				break;
			case ioctl_SEND_FMR:
				xReturn = drv_I2C_write_FMR(xI2c->devAddress );
				break;
			case ioctl_SEND_PMR:
				xReturn = drv_I2C_write_PMR(xI2c->devAddress );
				break;
			default :
				xReturn = -1;
				break;
		}

	return xReturn;

}
//------------------------------------------------------------------------------------
int frtos_i2c_read( periferico_i2c_port_t *xI2c, char *pvBuffer, uint16_t xBytes )
{

int xReturn = 0U;

#ifdef DEBUG_I2C
//	xprintf_P( PSTR("FRTOS_I2C_RD: devAddr=0x%02x,xbytes=0x%02x\r\n\0"),xI2c->devAddress, xBytes);
#endif

	if ( ( xReturn = drv_I2C_master_read_R1(xI2c->devAddress, (char *)pvBuffer, xBytes)) >= 0 ) {
		xI2c->i2c_error_code = I2C_OK;
	} else {
		// Error de lectura indicado por el driver.
		xI2c->i2c_error_code = I2C_RD_ERROR;
	}

	return(xReturn);
}
//------------------------------------------------------------------------------------
