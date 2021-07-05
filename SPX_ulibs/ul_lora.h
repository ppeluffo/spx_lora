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
 *   PAra configurar los canales como el gW usar lora configallch 65280 0 0 0 2
 */

#ifndef SRC_SPX_ULIBS_UL_LORA_H_
#define SRC_SPX_ULIBS_UL_LORA_H_

#include "spx.h"

#define LORA_BUFFER_LEN			128
#define LORA_PAYLOAD_BUFFER_LEN	32

typedef enum { CNF = 0, UNCNF} lora_tx_mode_t;
typedef enum { OTAA = 0, ABP } lora_join_t;
typedef enum { 	JOIN_ACCEPTED = 0,
	JOIN_INVALID_PARAMS,
	JOIN_KEYS_NOT_INIT,
	JOIN_NO_FREE_CH,
	JOIN_SILENT,
	JOIN_BUSY,
	JOIN_MAC_PAUSED,
	JOIN_DENIED,
	JOIN_ERROR } lora_join_exit_codes_t;

typedef enum { TX_OK = 0,
	TX_INVALID_PARAMS,
	TX_NOT_JOINED,
	TX_NO_FREE_CH,
	TX_SILENT,
	TX_FRAME_CNT_ERR,
	TX_BUSY,
	TX_MAC_PAUSED,
	TX_INVALID_DATA_LEN,
	TX_MAC_ERR,
	TX_ERROR } lora_tx_exit_codes_t;

typedef enum { JOIN_IDLE = 0, JOIN_NOT_IDLE, NOT_JOIN } lora_net_status_t;

typedef enum { LTX_FSM_ENTRY=0,
	LTX_FSM_TRYES,
	LTX_FSM_KEYS,
	LTX_FSM_MAC_JOIN,
	LTX_FSM_KEYS_FAIL,
	LTX_FSM_MAC_JOIN_FAIL,
	LTX_FSM_TX,
	LTX_FSM_TX_FAIL } lora_states_tx_fsm_t;

char loraTXbuffer[LORA_BUFFER_LEN];

ringBuffer_s loraRXrbuffer;
uint8_t lora_RX_rbstorage[LORA_BUFFER_LEN];

ringBuffer_s loraPayloadBuffer;
uint8_t lora_Payload_rbstorage[LORA_PAYLOAD_BUFFER_LEN];

//-------------------------------------------------------------------------------------------
void lora_read_rx_data(void);
void lora_print_rx_data( void );
void lora_flush_rx_buffer(void);
void lora_flush_tx_buffer(void);
void lora_send_cmd( bool f_debug, uint16_t timeout );

void lora_flush_payload_buffer(void);
void lora_print_payload_buffer(void);
void lora_rprintf_u04 (uint8_t data );
void lora_rprintf_u08(uint8_t data);
void lora_rprintf_u16(uint16_t data);
void lora_rprintf_u32(uint32_t data);
void lora_rprintf_float(float data);
void lora_test_payload( char *s_modo, char *s_value);

void lora_reset(void);
bool lora_sys_reset(bool f_debug);
char *lora_sys_get_ver(bool f_debug);
bool lora_sys_sleep(bool f_debug, uint32_t millisec );
bool lora_radio_status( bool f_debug );

void lora_config_defaults(void);
void lora_config_keys_testing(void);
bool lora_config_params( char *s_name, char *s_param );

bool lora_mac_set_bat( bool f_debug, uint8_t bat_level );
bool lora_mac_set_deveui( bool f_debug );
char *lora_mac_get_deveui( bool f_debug );
bool lora_mac_set_appeui( bool f_debug );
char *lora_mac_get_appeui( bool f_debug );
bool lora_mac_set_appkey( bool f_debug );
bool lora_mac_set_devaddr( bool f_debug );
char *lora_mac_get_devaddr( bool f_debug );
bool lora_mac_set_nwkskey( bool f_debug );
bool lora_mac_set_appskey( bool f_debug );
bool lora_mac_save( bool f_debug );
bool lora_mac_reset(bool f_debug );
uint16_t lora_mac_get_status( bool f_debug );
void lora_getchstatus( bool f_debug, uint8_t channel_id, uint32_t *ch_freq, uint8_t *ch_stat );
bool lora_enable_channel ( bool f_debug, uint8_t channel_id, bool enable );
bool lora_enable_working_channels( bool f_debug, char *s_0to15, char *s_16to31, char *s_32to47,char *s_48to63, char *s_64to80 );
void lora_scan_all_channels(void);

void lora_set_ch_off(void);

bool lora_mac_set_dr( bool f_debug, uint8_t dr );
uint8_t lora_mac_get_sync( bool f_debug );
uint32_t lora_mac_get_upctr( bool f_debug );
bool lora_mac_set_keys( bool f_debug, lora_join_t joincode );
lora_join_exit_codes_t  lora_connect( bool f_debug, lora_join_t joincode );
lora_join_exit_codes_t lora_mac_join( bool f_debug, lora_join_t joincode );
lora_net_status_t lora_net_status ( bool f_debug );
lora_tx_exit_codes_t lora_mac_tx( bool f_debug, uint8_t tx_mode, uint8_t tx_port );
bool lora_test_tx( char *argv[]);



#define lora_connect_abp(f_debug) 			lora_connect(f_debug, ABP)
#define lora_connect_otaa(f_debug) 			lora_connect(f_debug, OTAA)
#define lora_mac_set_keys_abp(f_debug)		lora_mac_set_keys(f_debug, ABP)
#define lora_mac_set_keys_otaa(f_debug)		lora_mac_set_keys(f_debug, OTAA)
#define lora_mac_join_abp(f_debug)			lora_mac_join(f_debug, ABP)
#define lora_mac_join_otaa(f_debug)			lora_mac_join(f_debug, OTAA)

#define MAX_TX_ERRORS	3

#endif /* SRC_SPX_ULIBS_UL_LORA_H_ */
