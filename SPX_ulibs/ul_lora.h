/*
 * ul_lora.h
 *
 *  Created on: 9 jun. 2021
 *      Author: pablo
 *
 *  Pendiente:
 *  - Como saber si estamos conectados al server
 *
 *  sync word: Sirve para aislar las redes. Un receptor que usa un sync word dada, no va a recibir
 *             paquetes de un receptor que use otra syncword.
 *             Por defecto para redes publicas se usa 0x34
 *
 *   https://market.thingpark.com/
 *
 */

#ifndef SRC_SPX_ULIBS_UL_LORA_H_
#define SRC_SPX_ULIBS_UL_LORA_H_

#include "spx.h"

#define LORA_BUFFER_LEN	128
#define LORA_RX_LINEAL_BUFFER

typedef enum { CNF=0, UNCNF} lora_tx_mode_t;
typedef enum { GET=0, SET } lora_setget_t;
typedef enum { OTAA=0, ABP } lora_join_t;

char lora_RX_buffer[LORA_BUFFER_LEN];
char lora_TX_buffer[LORA_BUFFER_LEN];


void lora_reset(void);
void lora_flush_RX_buffer(void);
void lora_flush_TX_buffer(void);
void lora_print_RX_buffer( void );
bool lora_send(char *msg);
bool lora_expect( const char *rx_str);

void lora_read_rx_data(void);
void lora_config_defaults(void);
void lora_config_keys_testing(void);
bool lora_config_params( char *s_name, char *s_param );
void lora_send_cmd( uint16_t timeout );

bool lora_sys_reset(void);
void lora_sys_get_ver(void);

bool lora_mac_deveui( lora_setget_t opcode );
bool lora_mac_appeui( lora_setget_t opcode );
bool lora_mac_appkey( lora_setget_t opcode );
bool lora_mac_devaddr( lora_setget_t opcode );
bool lora_mac_nwkskey( lora_setget_t opcode );
bool lora_mac_appskey( lora_setget_t opcode );
bool lora_mac_save(void);
bool lora_mac_join( lora_join_t joincode );
void lora_mac_reset(void);
void lora_mac_get_status(void);
void lora_mac_get_sync(void);
void lora_mac_get_upctr(void);
bool lora_mac_set_keys( lora_join_t joincode );

bool lora_connect( lora_join_t joincode );

#define lora_connect_abp() 			lora_connect(ABP)
#define lora_connect_otaa() 		lora_connect(OTAA)
#define lora_mac_set_keys_abp()		lora_mac_set_keys(ABP)
#define lora_mac_set_keys_otaa()	lora_mac_set_keys(OTAA)

#endif /* SRC_SPX_ULIBS_UL_LORA_H_ */
