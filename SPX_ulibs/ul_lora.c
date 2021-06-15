/*
 * ul_lora.c
 *
 *  Created on: 9 jun. 2021
 *      Author: pablo
 */


#include "ul_lora.h"

//------------------------------------------------------------------------------------
void lora_reset(void)
{
	// Hay que poner el pin de reset en 0. (tiene un pull-up)
	// Como pasa por un FET, entonces debo poner el pin del micro en 1
	IO_set_LORA_RESET();
	vTaskDelay( ( TickType_t)( 5000 / portTICK_RATE_MS ) );
	IO_clr_LORA_RESET();

}
//------------------------------------------------------------------------------------
bool lora_sys_reset(void)
{

	lora_flush_TX_buffer();
	snprintf_P( lora_TX_buffer, sizeof(lora_TX_buffer), PSTR("sys reset\r\n") );
	lora_send_cmd(5000);

	lora_print_RX_buffer();
	return(true);

}
//------------------------------------------------------------------------------------
void lora_sys_get_ver(void)
{
	lora_flush_TX_buffer();
	snprintf_P( lora_TX_buffer, sizeof(lora_TX_buffer), PSTR("sys get ver\r\n"));
	lora_send_cmd(1000);
	lora_print_RX_buffer();

}
//------------------------------------------------------------------------------------
void lora_read_rx_data(void)
{
	// Lee el buffer de la RXint y llena el buffer local

char c;
uint8_t ptr;

	ptr = 0;
	while ( frtos_read( fdLORA, &c, 1 ) == 1 ) {
		lora_RX_buffer[ptr++] = c;
	}
}
//------------------------------------------------------------------------------------
void lora_flush_RX_buffer(void)
{
	memset( lora_RX_buffer, '\0', LORA_BUFFER_LEN );
}
//------------------------------------------------------------------------------------
void lora_flush_TX_buffer(void)
{
	memset( lora_TX_buffer, '\0', LORA_BUFFER_LEN );
}
//------------------------------------------------------------------------------------
void lora_print_RX_buffer( void )
{

	// Uso esta funcion para imprimir un buffer largo, mayor al que utiliza xprintf_P. !!!
	xprintf_P( PSTR("%s"), lora_RX_buffer );

}
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
void lora_send_cmd( uint16_t timeout )
{
	// Transmite un mensaje al modulo lora.
	// Puede ser un comando o un frame

	lora_flush_RX_buffer();
	xprintf_P( PSTR("%s"), lora_TX_buffer );
	xfprintf_P( fdLORA, PSTR("%s"), lora_TX_buffer );
	vTaskDelay( ( TickType_t)( timeout / portTICK_RATE_MS ) );
	lora_read_rx_data();
	//lora_print_RX_buffer();
}
//------------------------------------------------------------------------------------
bool lora_mac_deveui( lora_setget_t opcode )
{

bool retS = false;

	lora_flush_TX_buffer();
	switch (opcode) {
	case SET:
		snprintf_P( lora_TX_buffer, sizeof(lora_TX_buffer), PSTR("mac set deveui %s\r\n"), systemVars.lora_conf.deveui );
		lora_send_cmd(500);
		if ( strstr_P( lora_RX_buffer, PSTR("ok")) ) {
			retS = true;
			goto quit;
		}
		break;
	case GET:
		snprintf_P( lora_TX_buffer, sizeof(lora_TX_buffer), PSTR("mac get deveui\r\n"));
		lora_send_cmd(500);
		if ( strstr_P( lora_RX_buffer, PSTR("ok")) ) {
			retS = true;
			goto quit;
		}
		break;

	default:
		retS = false;
	}

quit:
	lora_print_RX_buffer();
	return(retS);

}
//------------------------------------------------------------------------------------
bool lora_mac_appeui( lora_setget_t opcode )
{

bool retS = false;

	lora_flush_TX_buffer();
	switch (opcode) {
	case SET:
		snprintf_P( lora_TX_buffer, sizeof(lora_TX_buffer), PSTR("mac set appeui %s\r\n"), systemVars.lora_conf.appeui );
		lora_send_cmd(500);
		if ( strstr_P( lora_RX_buffer, PSTR("ok")) ) {
			retS = true;
			goto quit;
		}
		break;
	case GET:
		snprintf_P( lora_TX_buffer, sizeof(lora_TX_buffer), PSTR("mac get appeui\r\n"));
		lora_send_cmd(500);
		if ( strstr_P( lora_RX_buffer, PSTR("ok")) ) {
			retS = true;
			goto quit;
		}
		break;

	default:
		retS = false;
		goto quit;
	}

quit:
	lora_print_RX_buffer();
	return(retS);
}
//------------------------------------------------------------------------------------
bool lora_mac_appkey( lora_setget_t opcode )
{

bool retS = false;

	lora_flush_TX_buffer();
	switch (opcode) {
	case SET:
		snprintf_P( lora_TX_buffer, sizeof(lora_TX_buffer), PSTR("mac set appkey %s\r\n"), systemVars.lora_conf.appkey );
		lora_send_cmd(500);
		if ( strstr_P( lora_RX_buffer, PSTR("ok")) ) {
			retS = true;
			goto quit;
		}
		break;

	default:
		retS = false;
		goto quit;
	}

quit:
	lora_print_RX_buffer();
	return(retS);
}
//------------------------------------------------------------------------------------
bool lora_mac_devaddr( lora_setget_t opcode )
{

bool retS = false;

	lora_flush_TX_buffer();
	switch (opcode) {
	case SET:
		snprintf_P( lora_TX_buffer, sizeof(lora_TX_buffer), PSTR("mac set devaddr %s\r\n"), systemVars.lora_conf.devaddr );
		lora_send_cmd(500);
		if ( strstr_P( lora_RX_buffer, PSTR("ok")) ) {
			retS = true;
			goto quit;
		}
		break;
	case GET:
		snprintf_P( lora_TX_buffer, sizeof(lora_TX_buffer), PSTR("mac get devaddr\r\n"));
		lora_send_cmd(500);
		if ( strstr_P( lora_RX_buffer, PSTR("ok")) ) {
			retS = true;
			goto quit;
		}
		break;

	default:
		retS = false;
		goto quit;
	}

quit:
	lora_print_RX_buffer();
	return(retS);
}
//------------------------------------------------------------------------------------
bool lora_mac_nwkskey( lora_setget_t opcode )
{

bool retS = false;

	lora_flush_TX_buffer();
	switch (opcode) {
	case SET:
		snprintf_P( lora_TX_buffer, sizeof(lora_TX_buffer), PSTR("mac set nwkskey %s\r\n"), systemVars.lora_conf.nwkskey );
		lora_send_cmd(500);
		if ( strstr_P( lora_RX_buffer, PSTR("ok")) ) {
			retS = true;
			goto quit;
		}
		break;

	default:
		retS = false;
		goto quit;
	}

quit:
	lora_print_RX_buffer();
	return(retS);
}
//------------------------------------------------------------------------------------
bool lora_mac_appskey( lora_setget_t opcode )
{

bool retS = false;

	lora_flush_TX_buffer();
	switch (opcode) {
	case SET:
		snprintf_P( lora_TX_buffer, sizeof(lora_TX_buffer), PSTR("mac set appskey %s\r\n"), systemVars.lora_conf.appskey );
		lora_send_cmd(500);
		if ( strstr_P( lora_RX_buffer, PSTR("ok")) ) {
			retS = true;
			goto quit;
		}
		break;

	default:
		retS = false;
		goto quit;
	}

quit:
	lora_print_RX_buffer();
	return(retS);
}
//------------------------------------------------------------------------------------
bool lora_mac_save(void)
{
	lora_flush_TX_buffer();
	snprintf_P( lora_TX_buffer, sizeof(lora_TX_buffer), PSTR("mac save\r\n") );
	lora_send_cmd(500 );
	if ( ! strcmp_P( lora_RX_buffer, PSTR("ok")) ) {
		return(true);
	}
	return(false);
}
//------------------------------------------------------------------------------------
bool lora_mac_join( lora_join_t joincode )
{

uint8_t timeout = 10;

	// Transmito el comando
	lora_flush_TX_buffer();
	switch (joincode) {
	case OTAA:
		snprintf_P( lora_TX_buffer, sizeof(lora_TX_buffer), PSTR("mac join otaa\r\n"));
		break;
	case ABP:
		snprintf_P( lora_TX_buffer, sizeof(lora_TX_buffer), PSTR("mac join abp\r\n"));
		break;
	default:
		return(false);
	}

	// Espero y analizo las respuestas
	// Tiene 2 respuestas: una inmediata que indica si el comando fue aceptado
	// Otra que indica el resultado del join y que puede demorar unos segundos
	lora_send_cmd(500);
	if ( strstr_P( lora_RX_buffer, PSTR("ok")) ) {
		goto expect_join_status;

	} else if ( strstr_P( lora_RX_buffer, PSTR("invalid_param")) ) {
		lora_print_RX_buffer();
		return(false);

	} else if ( strstr_P( lora_RX_buffer, PSTR("keys_not_init")) ) {
		lora_print_RX_buffer();
		return(false);

	} else if ( strstr_P( lora_RX_buffer, PSTR("no_free_ch")) ) {
		lora_print_RX_buffer();
		return(false);

	} else if ( strstr_P( lora_RX_buffer, PSTR("silent")) ) {
		lora_print_RX_buffer();
		return(false);

	} else if ( strstr_P( lora_RX_buffer, PSTR("busy")) ) {
		lora_print_RX_buffer();
		return(false);

	} else if ( strstr_P( lora_RX_buffer, PSTR("mac_paused")) ) {
		lora_print_RX_buffer();
		return(false);

	} else {
		lora_print_RX_buffer();
		return(false);
	}

expect_join_status:

	while (timeout-- > 0 ) {
		vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
		if ( strstr_P( lora_RX_buffer, PSTR("denied")) ) {
			lora_print_RX_buffer();
			return(false);
		}
		if ( strstr_P( lora_RX_buffer, PSTR("accepted")) ) {
			lora_print_RX_buffer();
			return(true);
		}
	}

	// Expiro el timeout
	lora_print_RX_buffer();
	return(false);
}
//------------------------------------------------------------------------------------
void lora_mac_reset(void)
{
	lora_flush_TX_buffer();
	snprintf_P( lora_TX_buffer, sizeof(lora_TX_buffer), PSTR("mac reset\r\n") );
	lora_send_cmd(1000 );
	lora_print_RX_buffer();
}
//------------------------------------------------------------------------------------
void lora_mac_get_status(void)
{
	lora_flush_TX_buffer();
	snprintf_P( lora_TX_buffer, sizeof(lora_TX_buffer), PSTR("mac get status\r\n") );
	lora_send_cmd(1000 );
	lora_print_RX_buffer();
}
//------------------------------------------------------------------------------------
void lora_mac_get_sync(void)
{
	lora_flush_TX_buffer();
	snprintf_P( lora_TX_buffer, sizeof(lora_TX_buffer), PSTR("mac get sync\r\n") );
	lora_send_cmd(1000 );
	lora_print_RX_buffer();
}
//------------------------------------------------------------------------------------
void lora_mac_get_upctr(void)
{
	lora_flush_TX_buffer();
	snprintf_P( lora_TX_buffer, sizeof(lora_TX_buffer), PSTR("mac get upctr\r\n") );
	lora_send_cmd(1000 );
	lora_print_RX_buffer();
}
//------------------------------------------------------------------------------------
bool lora_mac_set_keys( lora_join_t joincode )
{

bool retS = false;

	switch (joincode) {
	case OTAA:
		if ( lora_mac_deveui(SET) ) {
			if ( lora_mac_appeui(SET) ) {
				if ( lora_mac_appkey(SET) ) {
					lora_mac_deveui(GET);
					lora_mac_appeui(GET);
				}
			}
		}
		break;

	case ABP:
		if ( lora_mac_devaddr(SET) ) {
			if ( lora_mac_nwkskey(SET) ) {
				if ( lora_mac_appskey(SET) ) {
					lora_mac_devaddr(GET);
				}
			}
		}
		break;
	}

	return(retS);
}
//------------------------------------------------------------------------------------
bool lora_connect( lora_join_t joincode )
{

bool retS = false;

	switch (joincode) {
	case OTAA:
		if ( lora_mac_set_keys(OTAA) ) {
			if ( lora_mac_join(OTAA) ) {
				retS = true;
			}
		}
		break;

	case ABP:
		if ( lora_mac_set_keys(ABP) ) {
			if ( lora_mac_join(ABP) ) {
				retS = true;
			}
		}
		break;
	}

	return(retS);
}
//------------------------------------------------------------------------------------
