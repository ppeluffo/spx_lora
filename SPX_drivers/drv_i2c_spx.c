/*
 * i2c.c
 *
 *  Created on: 18/10/2015
 *      Author: pablo
 *
 *  Modificadas 20/05/2017 para adecuarlas al SP6K basado en xmega256
 *  El bus se trabaja en modo poleado.
 *  La velocidad se fija en 100Khz.
 *  Se utiliza el bus implementado en el puerto E.
 *
 *  Cuando el sistema arranca, el estado del bus es UNKNOWN. Se puede forzar a IDLE escribiendo
 *  los bits correspondientes del STATUS.
 *  Para pasarlo de nuevo a UNKNOWN solo un reset o deshabilitando el TWI.
 *  Para sacarlo de BUSY hay que generar un STOP
 *
 *  Al escribir en ADDR,DATA o STATUS se borran las flags. Esto hay que hacerlo de ultimo
 *  ya que luego de c/operacion, mientras las flags esten prendidas el master mantiene el bus.
 *
 *  CUANDO EL BUS QUEDA MAL NO TENGO FORMA DE SALIR QUE NO SEA APAGANDO Y PRENDIENDO !!!
 */


#include "drv_i2c_spx.h"
#include "l_printf.h"

//------------------------------------------------------------------------------------
// FUNCIONES PRIVADAS
//------------------------------------------------------------------------------------

bool pvI2C_set_bus_idle(void);
bool pvI2C_write_slave_address(const uint8_t devAddress);
bool pvI2C_write_slave_chipAddress( const uint8_t chipAddressLength, const uint16_t chipAddress );
bool pvI2C_write_data( const char txbyte);
bool pvI2C_read_slave( uint8_t response_flag, char *rxByte );

bool pvI2C_waitForComplete(void);
void pvI2C_reset(void);

//------------------------------------------------------------------------------------
void drv_I2C_init(void)
{
uint16_t bitrate_div = 0;
uint16_t bitrateKHz = 100;

	// El pin PE1(SCK) lo pongo como output.( master genera el reloj )
	// No es necesario pero si se tranca el bus, debo clockearlo
	IO_config_SCL();

	// calculate bitrate division
	bitrate_div = ((F_CPU / (2 * ( bitrateKHz * 1000) )) - 5);
	TWIE.MASTER.BAUD = (uint8_t) bitrate_div;
	TWIE.MASTER.CTRLA = 0x00;
	TWIE.MASTER.CTRLA |= ( 1<<TWI_MASTER_ENABLE_bp);	// Enable TWI
	TWIE.MASTER.CTRLB  = 0;
	TWIE.MASTER.STATUS = TWI_MASTER_BUSSTATE_IDLE_gc;

}
//------------------------------------------------------------------------------------
int drv_I2C_master_write_R1 ( const uint8_t devAddress, const uint32_t chipAddress, const uint8_t chipAddressLength, char *pvBuffer, size_t xBytes )
{

	// Durante c/operacion del ciclo WRITE, el status debe ser 0x62.!!!

bool retV = false;
uint8_t i = 0;
int xReturn = -1;

#ifdef DEBUG_I2C
	xprintf_P( PSTR("drv_i2c: MasterWrite: devAddr=0x%02x, length=0x%02x\r\n\0"),devAddress, xBytes );
#endif

	TWIE.MASTER.CTRLA |= ( 1<<TWI_MASTER_ENABLE_bp);	// Enable TWI

	// Fuerzo al bus al estado idle.
	if ( ! pvI2C_set_bus_idle() ) goto i2c_quit;

	// Pass1: Mando un START y el SLAVE_ADDRESS (SLA_W).
	// Reintento porque el slave puede estar ocupado y hasta que termine no va a mandar un ACK.
	if ( ! pvI2C_write_slave_address(devAddress & ~0x01) ) goto i2c_quit;

	// Pass2: Mando la direccion interna del slave donde voy a escribir.
	if ( chipAddressLength > 0 ) {
		if ( ! pvI2C_write_slave_chipAddress( chipAddressLength, chipAddress ) ) goto i2c_quit;
	}

	// Pass3: Mando el buffer de datos. Debo recibir 0x28 (DATA_ACK) en c/u
	if ( xBytes > 0 ) {
		for ( i=0; i < xBytes; i++ ) {
			if ( !pvI2C_write_data( *pvBuffer++ ) ) goto i2c_quit;
		}
		xReturn = i;

	} else {

		xReturn = 0;
	}

	retV = true;
i2c_quit:

	// Pass4) STOP
	TWIE.MASTER.CTRLC = TWI_MASTER_CMD_STOP_gc;

	// En caso de error libero la interface forzando el bus al estado IDLE
	if ( !retV )
		pvI2C_reset();

#ifdef DEBUG_I2C
	xprintf_P( PSTR("drv_i2c: MasterWrite: xReturn=0x%02x\r\n\0"), xReturn );
#endif

	return(xReturn);

}
//------------------------------------------------------------------------------------
int drv_I2C_master_read_R1  ( const uint8_t devAddress, char *pvBuffer, size_t xBytes )
{

bool retV = false;
char rxByte = ' ';
uint8_t i = 0;
int xReturn = -1;

#ifdef DEBUG_I2C
	xprintf_P( PSTR("drv_i2c: MasterRead: devAddr=0x%02x, length=0x%02x\r\n\0"),devAddress, xBytes );
#endif

	if ( xBytes < 1 ) return(false);

	TWIE.MASTER.CTRLA |= ( 1<<TWI_MASTER_ENABLE_bp);	// Enable TWI

	// Pass3: Mando un REPEATED START y el SLAVE_ADDRESS (SLA_R)
	// Lo mando una sola vez ya que estoy en medio del ciclo.

	if ( ! pvI2C_write_slave_address(devAddress | 0x01) ) goto i2c_quit;

	// Pass4: Leo todos los bytes requeridos y respondo a c/u con ACK.
	for ( i=0; i < (xBytes-1); i++ ) {
		if ( ! pvI2C_read_slave(ACK, &rxByte) ) goto i2c_quit;
		*pvBuffer++ = rxByte;
	}
	xReturn = i;

	// Ultimo byte.
	if ( ! pvI2C_read_slave(NACK, &rxByte) ) {
		xReturn = -1;
		goto i2c_quit;
	} else {
		*pvBuffer++ = rxByte;
		xReturn++;
	}

	// I2C read OK.
	retV = true;

i2c_quit:

	// Pass6) STOP
	//TWIE.MASTER.CTRLC = TWI_MASTER_CMD_STOP_gc;

	// En caso de error libero la interface forzando el bus al estado IDLE
	if ( !retV )
		pvI2C_reset();

	return(xReturn);

}
//------------------------------------------------------------------------------------
bool drv_I2C_scan_device( const uint8_t devAddress )
{
	// Pass1: Mando un START y el SLAVE_ADDRESS con 1 ('WR')
	// El start se genera automaticamente al escribir en el reg MASTER.ADDR.
	// Esto tambien resetea todas las flags.
	// La salida correcta es con STATUS = 0x62.
	// El escribir el ADDR borra todas las flags.

bool ret_code = false;

	TWIE.MASTER.CTRLA |= ( 1<<TWI_MASTER_ENABLE_bp);	// Enable TWI

	// Fuerzo al bus al estado idle.
	if ( ! pvI2C_set_bus_idle() ) goto i2c_quit;

	// Pass1: Mando un START y el SLAVE_ADDRESS (SLA_W).
	// Reintento porque el slave puede estar ocupado y hasta que termine no va a mandar un ACK.
	ret_code = pvI2C_write_slave_address(devAddress & ~0x01);

i2c_quit:

	return(ret_code);
}
//------------------------------------------------------------------------------------
// FUNCIONES AUXILIARES PRIVADAS DE I2C
//------------------------------------------------------------------------------------
bool pvI2C_set_bus_idle(void)
{

	// Para comenzar una operacion el bus debe estar en IDLE o OWENED.
	// Intento pasarlo a IDLE hasta 3 veces antes de abortar, esperando 100ms
	// entre c/intento.

uint8_t bus_status = 0;
uint8_t	reintentos = I2C_MAXTRIES;

	while ( reintentos-- > 0 ) {

		// Los bits CLKHOLD y RXACK son solo de read por eso la mascara !!!

		bus_status = TWIE.MASTER.STATUS & 0xCF; 	//& TWI_MASTER_BUSSTATE_gm;

#ifdef DEBUG_I2C
		xprintf_P( PSTR("drv_i2c: SetBusIdle tryes=(%d): status=0x%02x\r\n\0"),reintentos,TWIE.MASTER.STATUS );
#endif

		if (  ( bus_status == TWI_MASTER_BUSSTATE_IDLE_gc ) || ( bus_status == TWI_MASTER_BUSSTATE_OWNER_gc ) ) {
			return(true);

		} else {
			// El status esta indicando errores. Debo limpiarlos antes de usar la interface.
			if ( (bus_status & TWI_MASTER_ARBLOST_bm) != 0 ) {
				TWIE.MASTER.STATUS = bus_status | TWI_MASTER_ARBLOST_bm;
			}
			if ( (bus_status & TWI_MASTER_BUSERR_bm) != 0 ) {
				TWIE.MASTER.STATUS = bus_status | TWI_MASTER_BUSERR_bm;
			}
			if ( (bus_status & TWI_MASTER_WIF_bm) != 0 ) {
				TWIE.MASTER.STATUS = bus_status | TWI_MASTER_WIF_bm;
			}
			if ( (bus_status & TWI_MASTER_RIF_bm) != 0 ) {
				TWIE.MASTER.STATUS = bus_status | TWI_MASTER_RIF_bm;
			}

			TWIE.MASTER.STATUS = bus_status | TWI_MASTER_BUSSTATE_IDLE_gc;	// Pongo el status en 01 ( idle )
			vTaskDelay( ( TickType_t)( 10 / portTICK_RATE_MS ) );
		}
	}

	// No pude pasarlo a IDLE: Error !!!
	xprintf_P( PSTR("drv_i2c: SetBusIdle ERROR!!: status=0x%02x\r\n\0"),TWIE.MASTER.STATUS );
	return(false);
}
//------------------------------------------------------------------------------------
bool pvI2C_write_slave_address(const uint8_t devAddress)
{
	// Pass1: Mando un START y el SLAVE_ADDRESS (SLA_W/R)
	// El start se genera automaticamente al escribir en el reg MASTER.ADDR.
	// Esto tambien resetea todas las flags.
	// La salida correcta es con STATUS = 0x62.
	// El escribir el ADDR borra todas las flags.

char txbyte = devAddress;	// (SLA_W/R) Send slave address
bool ret_code = false;
uint8_t currentStatus = 0;

#ifdef DEBUG_I2C
		xprintf_P( PSTR("drv_i2c: WriteSlaveAddr: IN\r\n\0") );
#endif

	TWIE.MASTER.ADDR = txbyte;
	if ( ! pvI2C_waitForComplete() ) goto i2c_exit;
	currentStatus = TWIE.MASTER.STATUS;

#ifdef DEBUG_I2C
		xprintf_P( PSTR("drv_i2c: WriteSlaveAddr: txbyte=0x%02x, status=0x%02x\r\n\0"), txbyte, currentStatus );
#endif

	// Primero evaluo no tener errores.
	if ( (currentStatus & TWI_MASTER_ARBLOST_bm) != 0 ) {
		xprintf_P( PSTR("drv_i2c: WriteSlaveAddr: ARBLOST: txbyte=0x%02x, status=0x%02x\r\n\0"),txbyte,currentStatus );
		goto i2c_exit;
	}

	if ( (currentStatus & TWI_MASTER_BUSERR_bm) != 0 ) {
		xprintf_P( PSTR("drv_i2c: WriteSlaveAddr: BUSERR: txbyte=0x%02x, status=0x%02x\r\n\0"),txbyte,currentStatus );
		goto i2c_exit;
	}

	// ACK o NACK ?
	if ( (currentStatus & TWI_MASTER_RXACK_bm) != 0 ) {
		// NACK
#ifdef DEBUG_I2C
		xprintf_P( PSTR("drv_i2c: WriteSlaveAddr: NACK: txbyte=0x%02x, status=0x%02x\r\n\0"),txbyte,currentStatus );
#endif
		goto i2c_exit;
	} else {
		// ACK
#ifdef DEBUG_I2C
		xprintf_P( PSTR("drv_i2c: WriteSlaveAddr: ACK: txbyte=0x%02x, status=0x%02x\r\n\0"),txbyte,currentStatus );
#endif
		ret_code = true;
		goto i2c_exit;
	}

i2c_exit:

#ifdef DEBUG_I2C
	if ( !ret_code ) {
		xprintf_P( PSTR("drv_i2c: WriteSlaveAddr: ERROR: txbyte=0x%02x, status=0x%02x\r\n\0"),txbyte,currentStatus );
	}
#endif


#ifdef DEBUG_I2C
		xprintf_P( PSTR("drv_i2c: WriteSlaveAddr: OUT ret_code=0x%02x, status=0x%02x\r\n\0"), ret_code,currentStatus );
#endif

	return(ret_code);
}
//------------------------------------------------------------------------------------
bool pvI2C_write_slave_chipAddress( const uint8_t chipAddressLength, const uint16_t chipAddress )
{
// Mando la direccion interna del slave donde voy a escribir.
// Pueden ser 0, 1 o 2 bytes.
// En las memorias es una direccion de 2 bytes.En el DS1344 o el MCP es de 1 byte
// En los ADC suele ser 0.
// Envio primero el High 8 bit i2c address.
// Luego envio el Low 8 byte i2c address.

char txbyte = ' ';
bool ret_code = false;
uint8_t currentStatus = 0;

#ifdef DEBUG_I2C
		xprintf_P( PSTR("drv_i2c: WriteChipAddr: IN\r\n\0") );
#endif

	// HIGH address
	if ( chipAddressLength == 2 ) {
		txbyte = (chipAddress) >> 8;
		TWIE.MASTER.DATA = txbyte;		// send byte
		if ( ! pvI2C_waitForComplete() )  goto i2c_exit;
		currentStatus = TWIE.MASTER.STATUS;

#ifdef DEBUG_I2C
		xprintf_P( PSTR("drv_i2c: WriteChipAddr: ADDR_H=0x%02x, status=0x%02x\r\n\0"), txbyte, currentStatus );
#endif
		// ACK o NACK ?
		if ( (currentStatus & TWI_MASTER_RXACK_bm) != 0 )  goto i2c_exit;
	}

	// LOW address
	if ( chipAddressLength >= 1 ) {
		txbyte = (chipAddress) & 0x00FF;
		TWIE.MASTER.DATA = txbyte;		// send byte
		if ( ! pvI2C_waitForComplete() )  goto i2c_exit;
		currentStatus = TWIE.MASTER.STATUS;

#ifdef DEBUG_I2C
		xprintf_P( PSTR("drv_i2c: WriteChipAddr: ADDR_L=0x%02x, status=0x%02x\r\n\0"), txbyte, currentStatus );
#endif
		if ( (currentStatus & TWI_MASTER_RXACK_bm) != 0 )  goto i2c_exit;
	}

	ret_code = true;

i2c_exit:

	if ( !ret_code ) {
		xprintf_P( PSTR("drv_i2c: WriteChipAddr: ERROR: txbyte=0x%02x, status=0x%02x\r\n\0"),txbyte,currentStatus );
	}

#ifdef DEBUG_I2C
		xprintf_P( PSTR("drv_i2c: WriteChipAddr: OUT ret_code=0x%02x, status=0x%02x\r\n\0"), ret_code,currentStatus );
#endif
	return(ret_code);
}
//------------------------------------------------------------------------------------
bool pvI2C_write_data( const char txbyte)
{

	// Envia un byte por la interface TWI. Si el resultado es exitoso se prende la
	// flag WIF y recibo un ACK.

bool ret_code = false;
uint8_t currentStatus = 0;

	TWIE.MASTER.DATA = txbyte;		// send byte
	if ( ! pvI2C_waitForComplete() )  goto i2c_exit;
	currentStatus = TWIE.MASTER.STATUS;

#ifdef DEBUG_I2C
	xprintf_P( PSTR("drv_i2c: WriteData: 0x%02x(%c),0x%02x\r\n\0"),txbyte,txbyte,currentStatus );
#endif

	if ( (currentStatus & TWI_MASTER_RXACK_bm) != 0 )  goto i2c_exit;

	ret_code = true;

i2c_exit:
	if ( !ret_code ) {
		xprintf_P( PSTR("drv_i2c: WriteData: ERROR: 0x%02x(%c),0x%02x\r\n\0"),txbyte,txbyte,currentStatus );
	}
	return(ret_code);
}
//------------------------------------------------------------------------------------
bool pvI2C_read_slave( uint8_t response_flag, char *rxByte )
{

bool ret_code = false;
uint8_t currentStatus = 0;

	// Espero 1 byte enviado por el slave
	if ( ! pvI2C_waitForComplete() ) goto i2c_exit;
	currentStatus = TWIE.MASTER.STATUS;

	*rxByte = TWIE.MASTER.DATA;

	if (response_flag == ACK) {
		TWIE.MASTER.CTRLC = TWI_MASTER_CMD_RECVTRANS_gc;	// ACK Mas bytes
	} else {
		TWIE.MASTER.CTRLC = TWI_MASTER_ACKACT_bm | TWI_MASTER_CMD_STOP_gc;	// NACK Ultimo byte
	}

#ifdef DEBUG_I2C
		xprintf_P( PSTR("drv_i2c: ReadSlave: 0x%02x,0x%02x\r\n"),*rxByte,currentStatus );
#endif

	// Primero evaluo no tener errores.
	if ( (currentStatus & TWI_MASTER_ARBLOST_bm) != 0 ) goto i2c_exit;
	if ( (currentStatus & TWI_MASTER_BUSERR_bm) != 0 ) goto i2c_exit;

	ret_code = true;

i2c_exit:

	if ( !ret_code ) {
		xprintf_P( PSTR("drv_i2c: ReadSlave: ERROR: 0x%02x,0x%02x\r\n"), *rxByte, currentStatus );
	}
	return(ret_code);
}
//------------------------------------------------------------------------------------
bool pvI2C_waitForComplete(void)
{

uint8_t ticks_to_wait = 30;		// 3 ticks ( 30ms es el maximo tiempo que espero )

	// wait for i2c interface to complete write operation ( MASTER WRITE INTERRUPT )
	while ( ticks_to_wait-- > 0 ) {
		if ( ( (TWIE.MASTER.STATUS & TWI_MASTER_WIF_bm) != 0 ) || ( (TWIE.MASTER.STATUS & TWI_MASTER_RIF_bm) != 0 ) ) {
			return(true);
		}
		//vTaskDelay( ( TickType_t)( 10 / portTICK_RATE_MS ) );
		vTaskDelay( ( TickType_t)( 1 ) );
	}

	// DEBUG
#ifdef DEBUG_I2C
	xprintf_P( PSTR("drv_i2c: WaitForComplete: TIMEOUT status=0x%02x\r\n\0"), TWIE.MASTER.STATUS );
#endif

	return(false);

}
//------------------------------------------------------------------------------------
void pvI2C_reset(void)
{

uint8_t bus_status = 0;
uint8_t i = 0;

	// https://stackoverflow.com/questions/5497488/failed-twi-transaction-after-sleep-on-xmega
	// There is a common problem on I2C/TWI where the internal state machine gets stuck in an
	// intermediate state if a transaction is not completed fully. The slave then does not respond
	// correctly when addressed on the next transaction. This commonly happens when the master
	// is reset or stops outputting the SCK signal part way through the read or write.
	// A solution is to toggle the SCK line manually 8 or 9 times before starting any data
	// transactions so the that the internal state machines in the slaves are all reset to the
	///start of transfer point and they are all then looking for their address byte

	// La forma de resetear el bus es deshabilitando el TWI
	TWIE.MASTER.CTRLA &= ~( 1<<TWI_MASTER_ENABLE_bp);	// Disable TWI
	vTaskDelay( 5 );

	// Clockeo el SCK varias veces para destrabar a los slaves
	IO_config_SCL();
	for (i=0; i<10;i++) {
		IO_set_SCL();
		vTaskDelay( 1 );
		IO_clr_SCL();
		vTaskDelay( 1 );
	}
	// Lo dejo en reposo alto
	IO_set_SCL();

	TWIE.MASTER.CTRLA |= ( 1<<TWI_MASTER_ENABLE_bp);	// Enable TWI

//	TWIE.MASTER.CTRLC =  TWI_MASTER_CMD_REPSTART_gc;	// Send START

	bus_status = TWIE.MASTER.STATUS;

	// El status esta indicando errores. Debo limpiarlos antes de usar la interface.
	if ( (bus_status & TWI_MASTER_ARBLOST_bm) != 0 ) {
		TWIE.MASTER.STATUS = bus_status | TWI_MASTER_ARBLOST_bm;
	}
	if ( (bus_status & TWI_MASTER_BUSERR_bm) != 0 ) {
		TWIE.MASTER.STATUS = bus_status | TWI_MASTER_BUSERR_bm;
	}
	if ( (bus_status & TWI_MASTER_WIF_bm) != 0 ) {
		TWIE.MASTER.STATUS = bus_status | TWI_MASTER_WIF_bm;
	}
	if ( (bus_status & TWI_MASTER_RIF_bm) != 0 ) {
		TWIE.MASTER.STATUS = bus_status | TWI_MASTER_RIF_bm;
	}

	TWIE.MASTER.STATUS = TWI_MASTER_BUSSTATE_IDLE_gc;	// Pongo el status en 01 ( idle )
	vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

#ifdef DEBUG_I2C
	xprintf_P( PSTR("drv_i2c: I2C_RESET_TWI: 0x%02x\r\n\0"),TWIE.MASTER.STATUS );
#endif
}
//------------------------------------------------------------------------------------



