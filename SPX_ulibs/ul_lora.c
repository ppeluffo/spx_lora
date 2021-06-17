/*
 * ul_lora.c
 *
 *  Created on: 9 jun. 2021
 *      Author: pablo
 */


#include "ul_lora.h"


static const char __attribute__ ((progmem)) HexChars[] = "0123456789ABCDEF";

#define hexchar(x)	pgm_read_byte( HexChars+((x)&0x0f) )
//------------------------------------------------------------------------------------
// FUNCIONES DE MANEJO DE I/O LORA
//------------------------------------------------------------------------------------
void u_lora_init(void)
{
	rBufferCreateStatic( &loraRXrbuffer, &lora_RX_rbstorage[0], LORA_BUFFER_LEN );
	rBufferFlush(&loraRXrbuffer);

	rBufferCreateStatic( &loraPayloadBuffer, &lora_Payload_rbstorage[0], LORA_PAYLOAD_BUFFER_LEN );
	rBufferFlush(&loraPayloadBuffer);
}
//------------------------------------------------------------------------------------
void lora_read_rx_data(void)
{
	// Lee el buffer de la RXint y llena el buffer local

char c;

	while ( frtos_read( fdLORA, &c, 1 ) == 1 ) {
		rBufferPoke( &loraRXrbuffer, &c );
	}
}
//------------------------------------------------------------------------------------
void lora_print_rx_data( void )
{

	// Asumo que le RX buffer está operando en la zona lineal !!!
	// Uso esta funcion para imprimir un buffer largo, mayor al que utiliza xprintf_P. !!!
	xprintf_P( PSTR("%s"), loraRXrbuffer.buff );

}
//------------------------------------------------------------------------------------
void lora_flush_rx_buffer(void)
{
	rBufferFlush(&loraRXrbuffer);
}
//------------------------------------------------------------------------------------
void lora_flush_tx_buffer(void)
{
	memset( loraTXbuffer, '\0', LORA_BUFFER_LEN );
}
//------------------------------------------------------------------------------------
bool lora_payload_poke( uint8_t c)
{
char data;

	data = ( char )c;
	return ( rBufferPoke( &loraPayloadBuffer, &data ));
}
//------------------------------------------------------------------------------------
void lora_rprintf_u04 (uint8_t data )
{
	lora_payload_poke( hexchar(data));
}
//------------------------------------------------------------------------------------
void lora_rprintf_u08(uint8_t data)
{
	lora_rprintf_u04 ( data>>4);
	lora_rprintf_u04 ( data );
}
//------------------------------------------------------------------------------------
void lora_rprintf_u16(uint16_t data)
{
	lora_rprintf_u08 ( data>>8);
	lora_rprintf_u08 ( data );
}
//------------------------------------------------------------------------------------
void lora_rprintf_u32(uint32_t data)
{
	lora_rprintf_u16 ( data>>16);
	lora_rprintf_u16 ( data );

}
//------------------------------------------------------------------------------------
void lora_rprintf_float(float data)
{
union {
	float fdata;
	char cdata[4];
} udata;

	udata.fdata = data;
	lora_rprintf_u08( udata.cdata[3]);
	lora_rprintf_u08( udata.cdata[2]);
	lora_rprintf_u08( udata.cdata[1]);
	lora_rprintf_u08( udata.cdata[0]);

}
//------------------------------------------------------------------------------------
void lora_test_payload( char *s_modo, char *s_value)
{
	lora_flush_payload_buffer();

	if (  strstr_P( s_modo, PSTR("u08")))  {
		lora_rprintf_u08( (uint8_t)atoi(s_value));
		lora_print_payload_buffer();

	} else if (  strstr_P( s_modo, PSTR("u16")))  {
		lora_rprintf_u16(atoi(s_value));
		lora_print_payload_buffer();

	} else if (  strstr_P( s_modo, PSTR("u32")))  {
		lora_rprintf_u32(atol(s_value));
		lora_print_payload_buffer();

	} else if (  strstr_P( s_modo, PSTR("loat")))  {
		lora_rprintf_float(atof(s_value));
		lora_print_payload_buffer();
	}


}
//------------------------------------------------------------------------------------
void lora_flush_payload_buffer(void)
{
	rBufferFlush(&loraPayloadBuffer);
}
//------------------------------------------------------------------------------------
void lora_print_payload_buffer(void)
{
	xprintf_P( PSTR("payload: [%s]\r\n"), loraPayloadBuffer.buff );
}
//------------------------------------------------------------------------------------
void lora_send_cmd( bool f_debug, uint16_t timeout )
{
	// Transmite un mensaje al modulo lora.
	// Puede ser un comando o un frame

	if ( f_debug )
		xprintf_P( PSTR("%s"), loraTXbuffer );

	xfprintf_P( fdLORA, PSTR("%s"), loraTXbuffer );
	vTaskDelay( ( TickType_t)( timeout / portTICK_RATE_MS ) );

}
//------------------------------------------------------------------------------------
void lora_reset(void)
{
	// Hay que poner el pin de reset en 0. (tiene un pull-up)
	// Como pasa por un FET, entonces debo poner el pin del micro en 1
	IO_set_LORA_RESET();
	vTaskDelay( ( TickType_t)( 5000 / portTICK_RATE_MS ) );
	IO_clr_LORA_RESET();
	xprintf_P( PSTR("LORA: Reset HW\r\n"));

}
//------------------------------------------------------------------------------------
// COMANDOS SYS
//------------------------------------------------------------------------------------
bool lora_sys_reset(bool f_debug)
{

uint8_t timeout = 10;

	lora_flush_tx_buffer();
	lora_flush_rx_buffer();
	snprintf_P( loraTXbuffer, sizeof(loraTXbuffer), PSTR("sys reset\r\n") );
	lora_send_cmd ( f_debug, 500);

	while( timeout-->0 ) {
		vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
		lora_read_rx_data();
		if (  strstr_P( ( char*)loraRXrbuffer.buff, PSTR("RN2903")))  {
			lora_print_rx_data();
			return(true);
		}
	}

	// timeout
	return(false);

}
//------------------------------------------------------------------------------------
char *lora_sys_get_ver(bool f_debug)
{
	// Comando inmediato

	lora_flush_tx_buffer();
	lora_flush_rx_buffer();
	snprintf_P( loraTXbuffer, sizeof(loraTXbuffer), PSTR("sys get ver\r\n") );
	lora_send_cmd ( f_debug, 1000);
	lora_read_rx_data();

	if ( f_debug )
		lora_print_rx_data();
	return ( (char *)loraRXrbuffer.buff );

}
//------------------------------------------------------------------------------------
// CONFIGURACION
//------------------------------------------------------------------------------------
void lora_config_defaults(void)
{
	memset( systemVars.lora_conf.deveui, '\0', LORA_KEYS_LENGTH );
	memset( systemVars.lora_conf.appeui, '\0', LORA_KEYS_LENGTH );
	memset( systemVars.lora_conf.appkey, '\0', LORA_KEYS_LENGTH );
	memset( systemVars.lora_conf.devaddr, '\0', LORA_KEYS_LENGTH );
	memset( systemVars.lora_conf.nwkskey, '\0', LORA_KEYS_LENGTH );
	memset( systemVars.lora_conf.appskey, '\0', LORA_KEYS_LENGTH );
	memset( systemVars.lora_conf.join, '\0', LORA_KEYS_LENGTH );

}
//------------------------------------------------------------------------------------
void lora_config_keys_testing(void)
{
	strncpy( systemVars.lora_conf.deveui, "AB01B889F432F30B", LORA_KEYS_LENGTH );
	strncpy( systemVars.lora_conf.appeui, "0000000000000000", LORA_KEYS_LENGTH );
	strncpy( systemVars.lora_conf.appkey, "FC20AAB69F43E7184B33C20CCB1972FE", LORA_KEYS_LENGTH );
	strncpy( systemVars.lora_conf.devaddr, "26031F9D", LORA_KEYS_LENGTH );
	strncpy( systemVars.lora_conf.nwkskey, "01515456354985465474156516384351", LORA_KEYS_LENGTH );
	strncpy( systemVars.lora_conf.appskey, "02626236239623965262565662654541", LORA_KEYS_LENGTH );
	memset( systemVars.lora_conf.join, '\0', LORA_KEYS_LENGTH );

}
//------------------------------------------------------------------------------------
bool lora_config_params( char *s_name, char *s_param )
{

bool retS = false;

	if ( !strcmp_P( strupr(s_name), PSTR("DEVEUI"))) {
		strncpy( systemVars.lora_conf.deveui, s_param , LORA_KEYS_LENGTH );
		retS = true;

	} else if (!strcmp_P( strupr(s_name), PSTR("APPEUI"))) {
		strncpy(systemVars.lora_conf.appeui, s_param , LORA_KEYS_LENGTH );
		retS = true;

	} else if (!strcmp_P( strupr(s_name), PSTR("APPKEY"))) {
		strncpy(systemVars.lora_conf.appkey, s_param , LORA_KEYS_LENGTH );
		retS = true;

	} else if (!strcmp_P( strupr(s_name), PSTR("DEVADDR"))) {
		strncpy(systemVars.lora_conf.devaddr, s_param , LORA_KEYS_LENGTH );
		retS = true;

	} else if (!strcmp_P( strupr(s_name), PSTR("NWKSKEY"))) {
		strncpy(systemVars.lora_conf.nwkskey, s_param , LORA_KEYS_LENGTH );
		retS = true;

	} else if (!strcmp_P( strupr(s_name), PSTR("APPSKEY"))) {
		strncpy(systemVars.lora_conf.appskey, s_param , LORA_KEYS_LENGTH );
		retS = true;

	}  else if (!strcmp_P( strupr(s_name), PSTR("JOIN"))) {
		strncpy(systemVars.lora_conf.join, s_param , LORA_KEYS_LENGTH );
		retS = true;
	}

	return(retS);

}
//------------------------------------------------------------------------------------
// LORA MAC
//------------------------------------------------------------------------------------
bool lora_mac_set_bat( bool f_debug, uint8_t bat_level )
{

bool retS = false;

	if ( bat_level > 100 )
		return(retS);

	lora_flush_tx_buffer();
	lora_flush_rx_buffer();
	snprintf_P( loraTXbuffer, sizeof(loraTXbuffer), PSTR("mac set bat %d\r\n"), bat_level );
	lora_send_cmd ( f_debug, 250);
	lora_read_rx_data();
	if (  strstr_P( ( char*)loraRXrbuffer.buff, PSTR("ok"))) {
		retS = true;
	} else if ( strstr_P( ( char*)loraRXrbuffer.buff, PSTR("invalid_param"))) {
		retS = false;
	}

	if ( f_debug )
		lora_print_rx_data();

	return(retS);

}
//------------------------------------------------------------------------------------
bool lora_mac_set_deveui( bool f_debug )
{
	// Comando instantaneo

bool retS = false;

	lora_flush_tx_buffer();
	lora_flush_rx_buffer();
	snprintf_P( loraTXbuffer, sizeof(loraTXbuffer), PSTR("mac set deveui %s\r\n"), systemVars.lora_conf.deveui );
	lora_send_cmd ( f_debug,1000);
	lora_read_rx_data();
	if (  strstr_P( ( char*)loraRXrbuffer.buff, PSTR("ok"))) {
		retS = true;
	} else if ( strstr_P( ( char*)loraRXrbuffer.buff, PSTR("invalid_param"))) {
		retS = false;
	}

	if ( f_debug )
		lora_print_rx_data();

	return(retS);

}
//------------------------------------------------------------------------------------
char *lora_mac_get_deveui( bool f_debug )
{
	// Comando instantaneo

	lora_flush_tx_buffer();
	lora_flush_rx_buffer();
	snprintf_P( loraTXbuffer, sizeof(loraTXbuffer), PSTR("mac get deveui\r\n"));
	lora_send_cmd ( f_debug,1000);
	lora_read_rx_data();
	if ( f_debug )
		lora_print_rx_data();
	return ( (char *)loraRXrbuffer.buff );

}
//------------------------------------------------------------------------------------
bool lora_mac_set_appeui( bool f_debug )
{
	// Comando instantaneo

bool retS = false;

	lora_flush_tx_buffer();
	lora_flush_rx_buffer();
	snprintf_P( loraTXbuffer, sizeof(loraTXbuffer), PSTR("mac set appeui %s\r\n"), systemVars.lora_conf.appeui );
	lora_send_cmd ( f_debug,1000);
	lora_read_rx_data();
	if (  strstr_P( ( char*)loraRXrbuffer.buff, PSTR("ok"))) {
		retS = true;
	} else if ( strstr_P( ( char*)loraRXrbuffer.buff, PSTR("invalid_param"))) {
		retS = false;
	}

	if ( f_debug )
		lora_print_rx_data();

	return(retS);

}
//------------------------------------------------------------------------------------
char *lora_mac_get_appeui( bool f_debug )
{
	// Comando instantaneo

	lora_flush_tx_buffer();
	lora_flush_rx_buffer();
	snprintf_P( loraTXbuffer, sizeof(loraTXbuffer), PSTR("mac get appeui\r\n"));
	lora_send_cmd ( f_debug,1000);
	lora_read_rx_data();
	if ( f_debug )
		lora_print_rx_data();
	return ( (char *)loraRXrbuffer.buff );

}
//------------------------------------------------------------------------------------
bool lora_mac_set_appkey( bool f_debug )
{
	// Comando instantaneo

bool retS = false;

	lora_flush_tx_buffer();
	lora_flush_rx_buffer();
	snprintf_P( loraTXbuffer, sizeof(loraTXbuffer), PSTR("mac set appkey %s\r\n"), systemVars.lora_conf.appkey );
	lora_send_cmd ( f_debug,1000);
	lora_read_rx_data();
	if (  strstr_P( ( char*)loraRXrbuffer.buff, PSTR("ok"))) {
		retS = true;
	} else if ( strstr_P( ( char*)loraRXrbuffer.buff, PSTR("invalid_param"))) {
		retS = false;
	}

	if ( f_debug )
		lora_print_rx_data();

	return(retS);

}
//------------------------------------------------------------------------------------
bool lora_mac_set_devaddr( bool f_debug )
{
	// Comando instantaneo

bool retS = false;

	lora_flush_tx_buffer();
	lora_flush_rx_buffer();
	snprintf_P( loraTXbuffer, sizeof(loraTXbuffer), PSTR("mac set devaddr %s\r\n"), systemVars.lora_conf.devaddr );
	lora_send_cmd ( f_debug,1000);
	lora_read_rx_data();
	if (  strstr_P( ( char*)loraRXrbuffer.buff, PSTR("ok"))) {
		retS = true;
	} else if ( strstr_P( ( char*)loraRXrbuffer.buff, PSTR("invalid_param"))) {
		retS = false;
	}

	if ( f_debug )
		lora_print_rx_data();

	return(retS);

}
//------------------------------------------------------------------------------------
char *lora_mac_get_devaddr( bool f_debug )
{
	// Comando instantaneo

	lora_flush_tx_buffer();
	lora_flush_rx_buffer();
	snprintf_P( loraTXbuffer, sizeof(loraTXbuffer), PSTR("mac get devaddr\r\n"));
	lora_send_cmd ( f_debug,1000);
	lora_read_rx_data();
	if ( f_debug )
		lora_print_rx_data();
	return ( (char *)loraRXrbuffer.buff );

}
//------------------------------------------------------------------------------------
bool lora_mac_set_nwkskey( bool f_debug )
{
	// Comando instantaneo

bool retS = false;

	lora_flush_tx_buffer();
	lora_flush_rx_buffer();
	snprintf_P( loraTXbuffer, sizeof(loraTXbuffer), PSTR("mac set nwkskey %s\r\n"), systemVars.lora_conf.nwkskey );
	lora_send_cmd ( f_debug,1000);
	lora_read_rx_data();
	if (  strstr_P( ( char*)loraRXrbuffer.buff, PSTR("ok"))) {
		retS = true;
	} else if ( strstr_P( ( char*)loraRXrbuffer.buff, PSTR("invalid_param"))) {
		retS = false;
	}

	if ( f_debug )
		lora_print_rx_data();

	return(retS);

}
//------------------------------------------------------------------------------------
bool lora_mac_set_appskey( bool f_debug )
{
	// Comando instantaneo

bool retS = false;

	lora_flush_tx_buffer();
	lora_flush_rx_buffer();
	snprintf_P( loraTXbuffer, sizeof(loraTXbuffer), PSTR("mac set appskey %s\r\n"), systemVars.lora_conf.appskey );
	lora_send_cmd ( f_debug,1000);
	lora_read_rx_data();
	if (  strstr_P( ( char*)loraRXrbuffer.buff, PSTR("ok"))) {
		retS = true;
	} else if ( strstr_P( ( char*)loraRXrbuffer.buff, PSTR("invalid_param"))) {
		retS = false;
	}

	if ( f_debug )
		lora_print_rx_data();

	return(retS);

}
//------------------------------------------------------------------------------------
bool lora_mac_save( bool f_debug )
{

bool retS = false;

	lora_flush_tx_buffer();
	lora_flush_rx_buffer();
	snprintf_P( loraTXbuffer, sizeof(loraTXbuffer), PSTR("mac save\r\n") );
	lora_send_cmd ( f_debug,1000);
	lora_read_rx_data();
	if (  strstr_P( ( char*)loraRXrbuffer.buff, PSTR("ok")))
		retS = true;

	if ( f_debug )
		lora_print_rx_data();

	return(retS);
}
//------------------------------------------------------------------------------------
bool lora_mac_reset(bool f_debug )
{

bool retS = false;

	lora_flush_tx_buffer();
	lora_flush_rx_buffer();
	snprintf_P( loraTXbuffer, sizeof(loraTXbuffer), PSTR("mac reset\r\n") );
	lora_send_cmd ( f_debug,1000);
	lora_read_rx_data();
	if (  strstr_P( ( char*)loraRXrbuffer.buff, PSTR("ok")))
		retS = true;

	if ( f_debug )
		lora_print_rx_data();

	return(retS);
}
//------------------------------------------------------------------------------------
uint16_t lora_mac_get_status( bool f_debug )
{

uint16_t status_word = 0;

	lora_flush_tx_buffer();
	lora_flush_rx_buffer();
	snprintf_P( loraTXbuffer, sizeof(loraTXbuffer), PSTR("mac get status\r\n") );
	lora_send_cmd ( f_debug,1000);
	lora_read_rx_data();
	status_word = atoi( (char *)loraRXrbuffer.buff );

	if ( f_debug )
		lora_print_rx_data();

	return ( status_word );
}
//------------------------------------------------------------------------------------
uint8_t lora_mac_get_sync( bool f_debug )
{

uint8_t sync_word = 0;

	lora_flush_tx_buffer();
	lora_flush_rx_buffer();
	snprintf_P( loraTXbuffer, sizeof(loraTXbuffer), PSTR("mac get sync\r\n") );
	lora_send_cmd ( f_debug,1000);
	lora_read_rx_data();
	sync_word = atoi( (char *)loraRXrbuffer.buff );

	if ( f_debug )
		lora_print_rx_data();

	return ( sync_word );
}
//------------------------------------------------------------------------------------
uint32_t lora_mac_get_upctr( bool f_debug )
{

uint32_t upctr_word = 0;

	lora_flush_tx_buffer();
	lora_flush_rx_buffer();
	snprintf_P( loraTXbuffer, sizeof(loraTXbuffer), PSTR("mac get upctr\r\n") );
	lora_send_cmd ( f_debug,1000);
	lora_read_rx_data();
	upctr_word = atol( (char *)loraRXrbuffer.buff );

	if ( f_debug )
		lora_print_rx_data();

	return ( upctr_word );
}
//------------------------------------------------------------------------------------
bool lora_mac_set_keys( bool f_debug, lora_join_t joincode )
{

bool retS = false;

	switch (joincode) {
	case OTAA:
		if ( lora_mac_set_deveui(f_debug) ) {
			if ( lora_mac_set_appeui(f_debug) ) {
				if ( lora_mac_set_appkey(f_debug) ) {
					lora_mac_get_deveui(f_debug);
					lora_mac_get_appeui(f_debug);
					retS = true;
				}
			}
		}
		break;

	case ABP:
		if ( lora_mac_set_devaddr(f_debug) ) {
			if ( lora_mac_set_nwkskey(f_debug) ) {
				if ( lora_mac_set_appskey(f_debug) ) {
					lora_mac_get_devaddr(f_debug);
					retS = true;
				}
			}
		}
		break;
	}

	return(retS);
}
//------------------------------------------------------------------------------------
lora_join_exit_codes_t lora_mac_join( bool f_debug, lora_join_t joincode )
{

uint8_t timeout = 15;

	// Transmito el comando
	lora_flush_tx_buffer();
	lora_flush_rx_buffer();
	switch (joincode) {
	case OTAA:
		snprintf_P( loraTXbuffer, sizeof(loraTXbuffer), PSTR("mac join otaa\r\n"));
		break;
	case ABP:
		snprintf_P( loraTXbuffer, sizeof(loraTXbuffer), PSTR("mac join abp\r\n"));
		break;
	default:
		return(JOIN_ERROR);
	}

	// Espero y analizo las respuestas
	// Tiene 2 respuestas: una inmediata que indica si el comando fue aceptado
	// Otra que indica el resultado del join y que puede demorar unos segundos

	// Respuesta Inmediata despues de dar el comando.
	lora_send_cmd ( f_debug,1000);
	lora_read_rx_data();

	if ( strstr_P( ( char*)loraRXrbuffer.buff, PSTR("ok")) ) {
		goto expect_join_response;

	} else if ( strstr_P( ( char*)loraRXrbuffer.buff, PSTR("invalid_param")) ) {
		if ( f_debug ) { lora_print_rx_data(); }
		return( JOIN_INVALID_PARAMS );

	} else if ( strstr_P( ( char*)loraRXrbuffer.buff, PSTR("keys_not_init")) ) {
		if ( f_debug ) { lora_print_rx_data(); }
		return(JOIN_KEYS_NOT_INIT);

	} else if ( strstr_P( ( char*)loraRXrbuffer.buff, PSTR("no_free_ch")) ) {
		if ( f_debug ) { lora_print_rx_data(); }
		return(JOIN_NO_FREE_CH);

	} else if ( strstr_P( ( char*)loraRXrbuffer.buff, PSTR("silent")) ) {
		if ( f_debug ) { lora_print_rx_data(); }
		return(JOIN_SILENT);

	} else if ( strstr_P( ( char*)loraRXrbuffer.buff, PSTR("busy")) ) {
		if ( f_debug ) { lora_print_rx_data(); }
		return(JOIN_BUSY);

	} else if ( strstr_P( ( char*)loraRXrbuffer.buff, PSTR("mac_paused")) ) {
		if ( f_debug ) { lora_print_rx_data(); }
		return(JOIN_MAC_PAUSED);

	} else {
		if ( f_debug ) { lora_print_rx_data(); }
		return(JOIN_ERROR);
	}

expect_join_response:
	// Aqui es donde espero la respuesta al join. Puede demorar algo.

	while (timeout-- > 0 ) {
		vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
		lora_read_rx_data();
		if ( strstr_P( ( char*)loraRXrbuffer.buff, PSTR("denied")) ) {
			if ( f_debug ) { lora_print_rx_data(); }
			return(JOIN_DENIED);
		} else if ( strstr_P( ( char*)loraRXrbuffer.buff, PSTR("accepted")) ) {
			if ( f_debug ) { lora_print_rx_data(); }
			return(JOIN_ACCEPTED);
		}
	}

	// Expiro el timeout
	if ( f_debug ) { lora_print_rx_data(); }
	return(JOIN_ERROR);

}
//------------------------------------------------------------------------------------
lora_join_exit_codes_t lora_connect( bool f_debug, lora_join_t joincode )
{

lora_join_exit_codes_t exit_code = JOIN_ERROR;


	switch (joincode) {
	case OTAA:
		if ( lora_mac_set_keys( f_debug, OTAA) ) {
			exit_code = lora_mac_join(f_debug, OTAA);
		}
		break;

	case ABP:
		if ( lora_mac_set_keys( f_debug, ABP) ) {
			exit_code = lora_mac_join( f_debug, ABP);
		}
		break;
	}

	return(exit_code);
}
//------------------------------------------------------------------------------------
lora_tx_exit_codes_t lora_mac_tx( bool f_debug, uint8_t tx_mode, uint8_t tx_port )
{
	// Cuando transmito, el payload debe estar en hexadecimal !!
	// Si tx_data dice "PABLO\0" debo convertirlo a 0x50 x041 x042 0x4C 0x4F 0x00
	// EL payload se escribe con las funciones lora_rprintf


uint8_t timeout = 15;

	// Transmito el comando
	lora_flush_tx_buffer();
	lora_flush_rx_buffer();
	switch (tx_mode) {
	case CNF:
		snprintf_P( loraTXbuffer, sizeof(loraTXbuffer), PSTR("mac tx cnf %d %s\r\n"), tx_port, loraPayloadBuffer );
		break;
	case UNCNF:
		snprintf_P( loraTXbuffer, sizeof(loraTXbuffer), PSTR("mac tx uncnf %d %s\r\n"), tx_port, loraPayloadBuffer );
		break;
	default:
		return(TX_ERROR);
	}

	// Espero y analizo las respuestas
	// Tiene 2 respuestas: una inmediata que indica si el comando fue aceptado
	// Otra que indica el resultado del join y que puede demorar unos segundos

	// Respuesta Inmediata despues de dar el comando.
	lora_send_cmd ( f_debug,1000);
	lora_read_rx_data();

	if ( strstr_P( ( char*)loraRXrbuffer.buff, PSTR("ok")) ) {
		goto expect_tx_response;

	} else if ( strstr_P( ( char*)loraRXrbuffer.buff, PSTR("invalid_param")) ) {
		if ( f_debug ) { lora_print_rx_data(); }
		return( TX_INVALID_PARAMS );

	} else if ( strstr_P( ( char*)loraRXrbuffer.buff, PSTR("not_joined")) ) {
		if ( f_debug ) { lora_print_rx_data(); }
		return(TX_NOT_JOINED);

	} else if ( strstr_P( ( char*)loraRXrbuffer.buff, PSTR("no_free_ch")) ) {
		if ( f_debug ) { lora_print_rx_data(); }
		return(TX_NO_FREE_CH);

	} else if ( strstr_P( ( char*)loraRXrbuffer.buff, PSTR("silent")) ) {
		if ( f_debug ) { lora_print_rx_data(); }
		return(TX_SILENT);

	} else if ( strstr_P( ( char*)loraRXrbuffer.buff, PSTR("frame_counter_err")) ) {
		if ( f_debug ) { lora_print_rx_data(); }
		return(TX_FRAME_CNT_ERR);

	} else if ( strstr_P( ( char*)loraRXrbuffer.buff, PSTR("busy")) ) {
		if ( f_debug ) { lora_print_rx_data(); }
		return(TX_BUSY);

	} else if ( strstr_P( ( char*)loraRXrbuffer.buff, PSTR("mac_paused")) ) {
		if ( f_debug ) { lora_print_rx_data(); }
		return(TX_MAC_PAUSED);

	} else if ( strstr_P( ( char*)loraRXrbuffer.buff, PSTR("invalid_data_len")) ) {
		if ( f_debug ) { lora_print_rx_data(); }
		return(TX_INVALID_DATA_LEN);

	} else {
		if ( f_debug ) { lora_print_rx_data(); }
		return(TX_ERROR);
	}

expect_tx_response:
	// Aqui es donde espero la respuesta al tx. Puede demorar algo.

	while (timeout-- > 0 ) {
		vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
		lora_read_rx_data();
		if ( strstr_P( ( char*)loraRXrbuffer.buff, PSTR("mac_tx_ok")) ) {
			if ( f_debug ) { lora_print_rx_data(); }
			return(TX_OK);
		} else if ( strstr_P( ( char*)loraRXrbuffer.buff, PSTR("mac_rx")) ) {
			if ( f_debug ) { lora_print_rx_data(); }
			return(TX_OK);
		} else if ( strstr_P( ( char*)loraRXrbuffer.buff, PSTR("mac_err")) ) {
			if ( f_debug ) { lora_print_rx_data(); }
			return(TX_MAC_ERR);
		} else if ( strstr_P( ( char*)loraRXrbuffer.buff, PSTR("invalid_data_len")) ) {
			if ( f_debug ) { lora_print_rx_data(); }
			return(TX_INVALID_DATA_LEN);
		}
	}

	// Expiro el timeout
	if ( f_debug ) { lora_print_rx_data(); }
		return(TX_ERROR);
}
//------------------------------------------------------------------------------------
bool lora_test_tx( char *argv[] )
{

uint8_t i;
char *s_modo;
char *s_value;

	s_modo = argv[2];
	lora_flush_payload_buffer();

	i = 3;
	while ( argv[i] != '\0' ) {

		s_value = argv[i];
		//xprintf_P(PSTR("DEBUG: %s\r\n"), argv[i]);
		//xprintf_P(PSTR("DEBUG: %s\r\n"), s_value );

		if (  strstr_P( s_modo, PSTR("u08")))  {
			lora_rprintf_u08( (uint8_t)atoi(s_value));

		} else if (  strstr_P( s_modo, PSTR("u16")))  {
			lora_rprintf_u16(atoi(s_value));

		} else if (  strstr_P( s_modo, PSTR("u32")))  {
			lora_rprintf_u32(atol(s_value));

		} else if (  strstr_P( s_modo, PSTR("loat")))  {
			lora_rprintf_float(atof(s_value));
		}

		i++;
	}

	lora_mac_tx( true, UNCNF, 1 );

	lora_print_payload_buffer();

	return(true);
}
//------------------------------------------------------------------------------------
// STATUS
//------------------------------------------------------------------------------------
lora_net_status_t lora_net_status ( bool f_debug )
{
	// En el status word, el bit 0 = 1 (net_joined)
	//                             = 0 (net_not_joined)
	// En el status word, el bit 3,2,1 indican el status de la mac. Si es 0 indica que
	// esta en estado idle y la trasmision es posible

uint16_t status;
lora_net_status_t net_status = NOT_JOIN;

	status = lora_mac_get_status( f_debug );

	if ( (status && 0x0001) == 0x00 ) {	// NOT_JOIN
		net_status = NOT_JOIN;
		if (f_debug) xprintf_P(PSTR("not join\r\n"));

	} else if ( ( status && 0x000F) == 0x01 ) {// JOIN_IDLE:
		net_status = JOIN_IDLE;
		if (f_debug) xprintf_P(PSTR("join and idle\r\n"));

	} else if ( ( status && 0x000F) != 0x01 ) {// JOIN_NOT_IDLE:
		net_status = JOIN_NOT_IDLE;
		if (f_debug) xprintf_P(PSTR("join but not idle\r\n"));
	}

	return (net_status);
}
//------------------------------------------------------------------------------------
void lora_getchstatus( bool f_debug, uint8_t channel_id, uint32_t *ch_freq, uint8_t *ch_stat )
{

uint32_t freq;
bool status_on = false;

	// Channel frequency
	lora_flush_tx_buffer();
	lora_flush_rx_buffer();
	snprintf_P( loraTXbuffer, sizeof(loraTXbuffer), PSTR("mac get ch freq %d\r\n"), channel_id );
	lora_send_cmd ( f_debug,500);
	lora_read_rx_data();
	freq = atol((char *)loraRXrbuffer.buff);
	*ch_freq = freq;
	if ( f_debug )
		lora_print_rx_data();

	// Channel status
	lora_flush_tx_buffer();
	lora_flush_rx_buffer();
	snprintf_P( loraTXbuffer, sizeof(loraTXbuffer), PSTR("mac get ch status %d\r\n"), channel_id );
	lora_send_cmd ( f_debug,500);
	lora_read_rx_data();
	if ( strstr_P( ( char*)loraRXrbuffer.buff, PSTR("on")) ) {
		status_on = true;
		*ch_stat = 1;
	} else {
		*ch_stat = 0;
	}

	if ( f_debug )
		lora_print_rx_data();

	if ( f_debug ) {
		if ( status_on ) {
			xprintf_P(PSTR("Channel: %d, status=on, freq=%lu\r\n"), channel_id, freq);
		} else {
			xprintf_P(PSTR("Channel: %d, status=off, freq=%lu\r\n"), channel_id, freq);
		}
	}
}
//------------------------------------------------------------------------------------
bool lora_enable_working_channels( bool f_debug, char *s_0to15, char *s_16to31, char *s_32to47,char *s_48to63, char *s_64to80 )
{
	// Habilita o deshabilita los canales de acuerdo a las mascaras
	// En la mascara high solo considera hasta el canal 71.
	// Si el bit correspondiente esta en 0 lo deshabilita y si esta en 1 lo habilita

uint8_t i;
uint16_t mask;
uint16_t bitmask;
uint8_t bit;
uint8_t channel = 0;

	if ( *s_0to15 == '\0' )
		return (false);

	// Seteo los canales del 0 al 15
	mask = 0x0000;
	bitmask = atoi(s_0to15);
	for ( i=0; i<16; i++ ) {
		mask = (1<<i);
		bit = ( bitmask & mask ) >> i;
		channel = 0 + i;
		if ( bit == 1 ) {
			lora_enable_channel ( f_debug, channel, true );
		} else {
			lora_enable_channel ( f_debug, channel, false );
		}
	}

	// Seteo los canales del 16 al 31
	if ( *s_16to31 == '\0' )
		return (false);

	mask = 0x0000;
	bitmask = atoi(s_16to31);
	for ( i=0; i<16; i++ ) {
		mask = (1<<i);
		bit = ( bitmask & mask ) >> i;
		channel = 16 + i;
		if ( bit == 1 ) {
			lora_enable_channel ( f_debug, channel, true );
		} else {
			lora_enable_channel ( f_debug, channel, false );
		}
	}

	// Seteo los canales del 32 al 47
	if ( *s_32to47 == '\0' )
		return (false);

	mask = 0x0000;
	bitmask = atoi(s_32to47);
	for ( i=0; i<16; i++ ) {
		mask = (1<<i);
		bit = ( bitmask & mask ) >> i;
		channel = 32 + i;
		if ( bit == 1 ) {
			lora_enable_channel ( f_debug, channel, true );
		} else {
			lora_enable_channel ( f_debug, channel, false );
		}
	}

	// Seteo los canales del 48 al 63
	if ( *s_48to63 == '\0' )
		return (false);

	mask = 0x0000;
	bitmask = atoi(s_48to63);
	for ( i=0; i<16; i++ ) {
		mask = (1<<i);
		bit = ( bitmask & mask ) >> i;
		channel = 48 + i;
		if ( bit == 1 ) {
			lora_enable_channel ( f_debug, channel, true );
		} else {
			lora_enable_channel ( f_debug, channel, false );
		}
	}

	// Seteo los canales del 64 al 71
	if ( *s_64to80 == '\0' )
		return (false);

	mask = 0x0000;
	bitmask = atoi(s_64to80);
	for ( i=0; i<16; i++ ) {
		mask = (1<<i);
		bit = ( bitmask & mask ) >> i;
		channel = 64 + i;
		if ( bit == 1 ) {
			lora_enable_channel ( f_debug, channel, true );
		} else {
			lora_enable_channel ( f_debug, channel, false );
		}
	}

	return(true);
}
//------------------------------------------------------------------------------------
bool lora_enable_channel ( bool f_debug, uint8_t channel_id, bool enable )
{
	// Habilita (true) o deshabilita (false) un canal

	lora_flush_tx_buffer();
	lora_flush_rx_buffer();
	if ( enable ) {
		snprintf_P( loraTXbuffer, sizeof(loraTXbuffer), PSTR("mac set ch status %02d on\r\n"), channel_id);
	} else {
		snprintf_P( loraTXbuffer, sizeof(loraTXbuffer), PSTR("mac set ch status %02d off\r\n"), channel_id);
	}
	lora_send_cmd ( f_debug,250);
	if ( f_debug ) {
		lora_read_rx_data();
		lora_print_rx_data();
	}
	return(true);
}
//------------------------------------------------------------------------------------
void lora_enable_working_channels_( bool f_debug )
{
	// Habilita los canales que tiene el gw habilitados.

uint8_t i;

	for ( i=0; i<8; i++) {

		lora_flush_tx_buffer();
		lora_flush_rx_buffer();
		snprintf_P( loraTXbuffer, sizeof(loraTXbuffer), PSTR("mac set ch status %02d off\r\n"), i );
		lora_send_cmd ( f_debug,500);
		lora_read_rx_data();
		lora_print_rx_data();
	}

	for ( i=16; i<65; i++) {

		lora_flush_tx_buffer();
		lora_flush_rx_buffer();
		snprintf_P( loraTXbuffer, sizeof(loraTXbuffer), PSTR("mac set ch status %02d off\r\n"), i );
		lora_send_cmd ( f_debug,500);
		lora_read_rx_data();
		lora_print_rx_data();
	}

	for ( i=66; i<72; i++) {

		lora_flush_tx_buffer();
		lora_flush_rx_buffer();
		snprintf_P( loraTXbuffer, sizeof(loraTXbuffer), PSTR("mac set ch status %d off\r\n"), i );
		lora_send_cmd ( f_debug,500);
		lora_read_rx_data();
		lora_print_rx_data();
	}
}
//------------------------------------------------------------------------------------
void lora_scan_all_channels(void)
{

uint8_t channel;
uint32_t freq;
uint8_t status;

	for ( channel = 0; channel < 72; channel++) {
		lora_getchstatus( false, channel, &freq, &status );
		if ( status == 1 ) {
			xprintf_P(PSTR("D_Channel: %d, status=on, freq=%lu\r\n"), channel, freq);
		} else {
			xprintf_P(PSTR("D_Channel: %d, status=off, freq=%lu\r\n"), channel, freq);
		}
	}
}
//------------------------------------------------------------------------------------
