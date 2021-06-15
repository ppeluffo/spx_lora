/*
 * sp5K_tkCmd.c
 *
 *  Created on: 27/12/2013
 *      Author: root
 */

#include "spx.h"

//----------------------------------------------------------------------------------------
// FUNCIONES DE USO PRIVADO
//----------------------------------------------------------------------------------------
static void pv_snprintfP_OK(void );
static void pv_snprintfP_ERR(void);
static void pv_cmd_read_fuses(void);
static void pv_cmd_I2Cscan(bool busscan);
static void pv_cmd_rwLORA(uint8_t cmd_mode );

//----------------------------------------------------------------------------------------
// FUNCIONES DE CMDMODE
//----------------------------------------------------------------------------------------
static void cmdHelpFunction(void);
static void cmdClearScreen(void);
static void cmdResetFunction(void);
static void cmdWriteFunction(void);
static void cmdReadFunction(void);
static void cmdStatusFunction(void);
static void cmdConfigFunction(void);
static void cmdKillFunction(void);
static void cmdLoraFunction(void);

#define WR_CMD 0
#define RD_CMD 1

#define WDG_CMD_TIMEOUT	WDG_TO120

#define LORA_RXBUFFER	128
char txbuffer[LORA_RXBUFFER];

bool lora_loopback;

//------------------------------------------------------------------------------------
void tkCmd(void * pvParameters)
{

uint8_t c = 0;
uint8_t ticks = 0;

( void ) pvParameters;

	// Espero la notificacion para arrancar
	while ( !startTask )
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

	vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );

	FRTOS_CMD_init();

	// Registro los comandos y los mapeo al cmd string.
	FRTOS_CMD_register( "cls\0", cmdClearScreen );
	FRTOS_CMD_register( "reset\0", cmdResetFunction);
	FRTOS_CMD_register( "write\0", cmdWriteFunction);
	FRTOS_CMD_register( "read\0", cmdReadFunction);
	FRTOS_CMD_register( "help\0", cmdHelpFunction );
	FRTOS_CMD_register( "status\0", cmdStatusFunction );
	FRTOS_CMD_register( "config\0", cmdConfigFunction );
	FRTOS_CMD_register( "kill\0", cmdKillFunction );
	FRTOS_CMD_register( "lora\0", cmdLoraFunction );

	// Fijo el timeout del READ
	ticks = 5;
	frtos_ioctl( fdTERM,ioctl_SET_TIMEOUT, &ticks );
	lora_loopback = false;

	xprintf_P( PSTR("starting tkCmd..\r\n") );

	//FRTOS_CMD_regtest();
	// loop
	for( ;; )
	{

		// Con la terminal desconectada paso c/5s plt 30s es suficiente.
		ctl_watchdog_kick(WDG_CMD, WDG_CMD_TIMEOUT);

		// Si no tengo terminal conectada, duermo 25s lo que me permite entrar en tickless.
		while ( ! ctl_terminal_connected() ) {
			ctl_watchdog_kick(WDG_CMD, WDG_CMD_TIMEOUT);
			vTaskDelay( ( TickType_t)( 25000 / portTICK_RATE_MS ) );
		}

		c = '\0';	// Lo borro para que luego del un CR no resetee siempre el timer.
		// el read se bloquea 50ms. lo que genera la espera.
		//while ( CMD_read( (char *)&c, 1 ) == 1 ) {
		while ( frtos_read( fdTERM, (char *)&c, 1 ) == 1 ) {
			FRTOS_CMD_process(c);
		}

	}
}
//------------------------------------------------------------------------------------
static void cmdLoraFunction(void)
{
	// Envia comandos al modulo lora.

uint8_t i,j;
int16_t free_size = sizeof(txbuffer);

	FRTOS_CMD_makeArgv();

	// LORA

	// lora loopback
	if ( strcmp_P( argv[1], PSTR("loopback")) == 0)  {
		if (!strcmp_P( strupr(argv[2]), PSTR("ON"))) {
			lora_loopback = true;
		}	else if (!strcmp_P( strupr(argv[2]), PSTR("OFF"))) {
			lora_loopback = false;
		}
		return;
	}

	// lora setupkeys
	if ( strcmp_P( argv[1], PSTR("macsetkeys")) == 0)  {
		if (!strcmp_P( strupr(argv[2]), PSTR("ABP"))) {
			lora_mac_set_keys_abp();
			return;
		}	else if (!strcmp_P( strupr(argv[2]), PSTR("OTAA"))) {
			lora_mac_set_keys_otaa();
			return;
		}
		return;
	}

	// lora macgetstatus
	if ( strcmp_P( argv[1], PSTR("macgetstatus")) == 0)  {
		lora_mac_get_status();
		return;
	}

	// lora macreset
	if ( strcmp_P( argv[1], PSTR("macreset")) == 0)  {
		lora_mac_reset();
		return;
	}

	// lora sysver
	if ( strcmp_P( argv[1], PSTR("sysgetver")) == 0)  {
		lora_sys_get_ver();
		return;
	}

	// lora sysreset
	if ( strcmp_P( argv[1], PSTR("sysreset")) == 0)  {
		lora_sys_reset();
		return;
	}

	// lora connect
	if ( strcmp_P( argv[1], PSTR("connect")) == 0)  {
		if (!strcmp_P( strupr(argv[2]), PSTR("ABP"))) {
			lora_connect_abp();
			return;
		}	else if (!strcmp_P( strupr(argv[2]), PSTR("OTAA"))) {
			lora_connect_otaa();
			return;
		}
		return;
	}

	// lora read
	if ( strcmp_P( argv[1], PSTR("read")) == 0)  {
		lora_read_rx_data();
		lora_print_RX_buffer();
		return;
	}

	// Comandos no manejados: directo al modulo
	lora_flush_RX_buffer();
	i = 1;
	j = 0;
	while ( argv[i] != NULL ) {
		//xprintf_P(PSTR("DEBUG0: i=%d, j=%d, s=%s\r\n"),i,j,argv[i]);
		j += snprintf_P( &txbuffer[j], free_size, PSTR("%s "), argv[i] );
		free_size = (  sizeof(txbuffer) - j );
		if ( free_size < 0 ) {
			xprintf_P(PSTR("buffer ERROR\r\n"));
			return;
		}
		i++;
	}

	i = strlen(txbuffer);
	txbuffer[--i] = '\0';

	xfprintf_P( fdLORA, PSTR("%s\r\n"),txbuffer );
	xprintf_P( PSTR("sent->%s\r\n"),txbuffer );
//	vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
//	lora_read_rx_data();
//	lora_print_RX_buffer();
	return;
}
//------------------------------------------------------------------------------------
static void cmdStatusFunction(void)
{

	FRTOS_CMD_makeArgv();

	xprintf_P( PSTR("\r\nSpymovil %s %s %s %s \r\n\0"), SPX_HW_MODELO, SPX_FTROS_VERSION, SPX_FW_REV, SPX_FW_DATE);
	xprintf_P( PSTR("Clock %d Mhz, Tick %d Hz\r\n\0"),SYSMAINCLK, configTICK_RATE_HZ );
	xprintf_P( PSTR("IOboard: SPX_lora\r\n\0") );

	// SIGNATURE ID
	xprintf_P( PSTR("uID=%s\r\n\0"), NVMEE_readID() );

	// Last reset cause
	xprintf_P( PSTR("WRST=0x%02X\r\n\0") ,wdg_resetCause );

	// CONFIG
	xprintf_P( PSTR(">Config:\r\n\0"));

	// debug
	switch(systemVars.debug) {
	case DEBUG_NONE:
		xprintf_P( PSTR("  debug: none\r\n\0") );
		break;
	case DEBUG_COUNTER:
		xprintf_P( PSTR("  debug: counter\r\n\0") );
		break;
	case DEBUG_DATA:
		xprintf_P( PSTR("  debug: data\r\n\0") );
		break;
	default:
		xprintf_P( PSTR("  debug: ???\r\n\0") );
		break;
	}

	// Timerpoll
	xprintf_P( PSTR("  timerPoll: %d\r\n"), systemVars.timer_poll );

	// Lora
	xprintf_P( PSTR("  lora deveui: %s\r\n"), systemVars.lora_conf.deveui );
	xprintf_P( PSTR("  lora appeui: %s\r\n"), systemVars.lora_conf.appeui );
	xprintf_P( PSTR("  lora appkey: %s\r\n"), systemVars.lora_conf.appkey );
	xprintf_P( PSTR("  lora devaddr: %s\r\n"), systemVars.lora_conf.devaddr );
	xprintf_P( PSTR("  lora nwkskey: %s\r\n"), systemVars.lora_conf.nwkskey );
	xprintf_P( PSTR("  lora appskey: %s\r\n"), systemVars.lora_conf.appskey );
	xprintf_P( PSTR("  lora join: %s\r\n"), systemVars.lora_conf.join );

	xprintf_P( PSTR(">Channels:\r\n"));

	psensor_config_print();

	xprintf_P( PSTR("  Q0: [magpp=%.03f,pw=%d,T=%d\0"),
			systemVars.counter_conf.magpp,
			systemVars.counter_conf.pwidth,
			systemVars.counter_conf.period );

	if ( systemVars.counter_conf.sensing_edge == RISING_EDGE ) {
		xprintf_P(PSTR("(RISE)]\r\n\0"));
	} else {
		xprintf_P(PSTR("(FALL)]\r\n\0"));
	}

	// FRAME
	xprintf_P( PSTR(">Frame:\r\n  "));
	data_read_frame(false);

}
//-----------------------------------------------------------------------------------
static void cmdResetFunction(void)
{
	// Resetea al micro prendiendo el watchdog

	FRTOS_CMD_makeArgv();

	// LORA
	// reset lora
	if ( strcmp_P( strupr(argv[1]), PSTR("LORA\0")) == 0) {
		lora_reset();
		pv_snprintfP_OK();
		return;
	}

	cmdClearScreen();
	CCPWrite( &RST.CTRL, RST_SWRST_bm );   /* Issue a Software Reset to initilize the CPU */


}
//------------------------------------------------------------------------------------
static void cmdWriteFunction(void)
{

	FRTOS_CMD_makeArgv();

	// LORA
	// write lora cmd
	if ( strcmp_P( strupr(argv[1]), PSTR("LORA\0")) == 0)  {
		pv_cmd_rwLORA(WR_CMD);
		return;
	}

	// NVMEE
	// write nvmee pos string
	if ( strcmp_P( strupr(argv[1]), PSTR("NVMEE\0")) == 0) {
		NVMEE_test_write ( argv[2], argv[3] );
		pv_snprintfP_OK();
		return;
	}

	// CMD NOT FOUND
	xprintf_P( PSTR("ERROR\r\nCMD NOT DEFINED\r\n\0"));
	return;
}
//------------------------------------------------------------------------------------
static void cmdReadFunction(void)
{

	FRTOS_CMD_makeArgv();

	// LORA
	// read lora
	if ( strcmp_P( strupr(argv[1]), PSTR("LORA\0")) == 0)  {
		pv_cmd_rwLORA(RD_CMD);
		return;
	}

	// CAUDAL
	// read caudal
	if ( strcmp_P( strupr(argv[1]), PSTR("CAUDAL\0")) == 0 ) {
		( caudal_read_test() )?  pv_snprintfP_OK() : pv_snprintfP_ERR();
		return;
	}

	// FRAME
	// read frame
	if (!strcmp_P( strupr(argv[1]), PSTR("FRAME\0")) ) {
		data_read_frame(true);
		return;
	}

	// PRESION
	// read presion
	if ( strcmp_P( strupr(argv[1]), PSTR("PRESION\0")) == 0 ) {
		( psensor_read_test() )?  pv_snprintfP_OK() : pv_snprintfP_ERR();
		return;
	}

	// PSENSOR
	// read psensor {0|1}
	if ( strcmp_P( strupr(argv[1]), PSTR("PSENSOR\0")) == 0 ) {
		( abp_raw_read_test( atoi(argv[2])) )?  pv_snprintfP_OK() : pv_snprintfP_ERR();
		return;
	}

	// I2Cscan
	// read i2cscan busaddr
	if (!strcmp_P( strupr(argv[1]), PSTR("I2CSCAN\0")) ) {
		pv_cmd_I2Cscan(false);
		return;
	}

	// I2Cscanbus
	// read i2cscanbus
	if (!strcmp_P( strupr(argv[1]), PSTR("I2CSCANBUS\0")) ) {
		pv_cmd_I2Cscan(true);
		return;
	}

	// FUSES
 	if (!strcmp_P( strupr(argv[1]), PSTR("FUSES\0"))) {
 		pv_cmd_read_fuses();
 		return;
 	}

	// NVMEE
	// read nvmee address length
	if (!strcmp_P( strupr(argv[1]), PSTR("NVMEE\0")) ) {
		NVMEE_test_read ( argv[2], argv[3] );
		return;
	}

	// CMD NOT FOUND
	xprintf_P( PSTR("ERROR\r\nCMD NOT DEFINED\r\n\0"));
	return;

}
//------------------------------------------------------------------------------------
static void cmdClearScreen(void)
{
	// ESC [ 2 J
	xprintf_P( PSTR("\x1B[2J\0"));
}
//------------------------------------------------------------------------------------
static void cmdConfigFunction(void)
{

bool retS = false;

	FRTOS_CMD_makeArgv();

	// Parametros COMUNICACIONES ---------------------------------------------------------------------------------

	// LORA
	// lora {deveui,appeui,appkey,devaddr,nwkskey,appskey,join}
	if (!strcmp_P( strupr(argv[1]), PSTR("LORA\0")) ) {
		retS = lora_config_params( argv[2], argv[3] );
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// TIMERPOLL
	// config timerpoll
	if (!strcmp_P( strupr(argv[1]), PSTR("TIMERPOLL\0")) ) {
		u_config_timerpoll( argv[2] );
		pv_snprintfP_OK();
		return;
	}

	// COUNTER
	// config counter magPP pulseWidth period sensing
	// counter hw {SIMPLE/OPTO)
	if ( strcmp_P( strupr(argv[1]), PSTR("COUNTER\0")) == 0 ) {
		retS = counter_config( argv[2], argv[3], argv[4], argv[5] );
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// DEBUG
	// config debug
	if (!strcmp_P( strupr(argv[1]), PSTR("DEBUG\0"))) {
		if (!strcmp_P( strupr(argv[2]), PSTR("NONE\0"))) {
			systemVars.debug = DEBUG_NONE;
			retS = true;
		} else if (!strcmp_P( strupr(argv[2]), PSTR("COUNTER\0"))) {
			systemVars.debug = DEBUG_COUNTER;
			retS = true;
		} else if (!strcmp_P( strupr(argv[2]), PSTR("DATA\0"))) {
			systemVars.debug = DEBUG_DATA;
			retS = true;
		} else {
			retS = false;
		}
		retS ? pv_snprintfP_OK() : 	pv_snprintfP_ERR();
		return;
	}

	// DEFAULT
	// config default
	if (!strcmp_P( strupr(argv[1]), PSTR("DEFAULT\0"))) {
		u_load_defaults( strupr(argv[2]) );
		pv_snprintfP_OK();
		return;
	}

	// SAVE
	// config save
	if (!strcmp_P( strupr(argv[1]), PSTR("SAVE\0"))) {
		u_save_params_in_NVMEE();
		pv_snprintfP_OK();
		return;
	}

	// CMD NOT FOUND
	xprintf_P( PSTR("ERROR\r\nCMD NOT DEFINED\r\n\0"));
	return;

}
//------------------------------------------------------------------------------------
static void cmdHelpFunction(void)
{

	FRTOS_CMD_makeArgv();

	// HELP LORA
	if (!strcmp_P( strupr(argv[1]), PSTR("LORA"))) {
		xprintf_P( PSTR("-lora\r\n"));
		xprintf_P( PSTR("  connect {join}\r\n"));
		xprintf_P( PSTR("  sysreset,sysgetver,macreset\r\n"));
		xprintf_P( PSTR("  macgetstatus,macsetkeys{join}\r\n"));
		xprintf_P( PSTR("  loopback {on|off}\r\n"));
		return;
	}

	// HELP WRITE
	if (!strcmp_P( strupr(argv[1]), PSTR("WRITE\0"))) {
		xprintf_P( PSTR("-write\r\n\0"));
		xprintf_P( PSTR("  (nvmee) {pos} {string}\r\n\0"));
		return;
	}

	// HELP READ
	else if (!strcmp_P( strupr(argv[1]), PSTR("READ\0"))) {
		xprintf_P( PSTR("-read\r\n\0"));
		xprintf_P( PSTR("  frame, fuses\r\n\0"));
		xprintf_P( PSTR("  psensor,presion\r\n\0"));
		xprintf_P( PSTR("  caudal\r\n"));
		xprintf_P( PSTR("  frame\r\n\0"));
		return;
	}

	// HELP RESET
	else if (!strcmp_P( strupr(argv[1]), PSTR("RESET\0"))) {
		xprintf_P( PSTR("-reset\r\n\0"));
		xprintf_P( PSTR("  lora\r\n\0"));
		return;

	}

	// HELP CONFIG
	else if (!strcmp_P( strupr(argv[1]), PSTR("CONFIG\0"))) {
		xprintf_P( PSTR("-config\r\n\0"));
		xprintf_P( PSTR("  default\r\n\0"));
		xprintf_P( PSTR("  save\r\n\0"));
		xprintf_P( PSTR("  debug {none,counter,comms}\r\n\0"));
		xprintf_P( PSTR("  timerpoll\r\n\0"));
		xprintf_P( PSTR("  counter cname magPP pw(ms) period(ms) edge(RISE/FALL)\r\n\0"));
		xprintf_P( PSTR("  lora {deveui,appeui,appkey,devaddr,nwkskey,appskey,join}\r\n"));
		return;
	}

	// HELP KILL
	else if (!strcmp_P( strupr(argv[1]), PSTR("KILL\0")) ) {
		xprintf_P( PSTR("-kill {data }\r\n\0"));
		return;

	} else {

		// HELP GENERAL
		xprintf_P( PSTR("\r\nSpymovil %s %s %s %s\r\n\0"), SPX_HW_MODELO, SPX_FTROS_VERSION, SPX_FW_REV, SPX_FW_DATE);
		xprintf_P( PSTR("Clock %d Mhz, Tick %d Hz\r\n\0"),SYSMAINCLK, configTICK_RATE_HZ );
		xprintf_P( PSTR("Available commands are:\r\n\0"));
		xprintf_P( PSTR("-cls\r\n\0"));
		xprintf_P( PSTR("-help\r\n\0"));
		xprintf_P( PSTR("-status\r\n\0"));
		xprintf_P( PSTR("-reset...\r\n\0"));
		xprintf_P( PSTR("-kill...\r\n\0"));
		xprintf_P( PSTR("-write...\r\n\0"));
		xprintf_P( PSTR("-read...\r\n\0"));
		xprintf_P( PSTR("-lora...\r\n\0"));
		xprintf_P( PSTR("-config...\r\n\0"));

	}

	xprintf_P( PSTR("\r\n\0"));

}
//------------------------------------------------------------------------------------
static void cmdKillFunction(void)
{

	FRTOS_CMD_makeArgv();

	// KILL DATA
	if (!strcmp_P( strupr(argv[1]), PSTR("DATA\0"))) {
		vTaskSuspend( xHandle_tkData );
		ctl_watchdog_kick(WDG_DATA, 0x8000 );
		pv_snprintfP_OK();
		return;
	}

	pv_snprintfP_ERR();
	return;
}
//------------------------------------------------------------------------------------
static void pv_snprintfP_OK(void )
{
	xprintf_P( PSTR("ok\r\n\0"));
}
//------------------------------------------------------------------------------------
static void pv_snprintfP_ERR(void)
{
	xprintf_P( PSTR("error\r\n\0"));
}
//------------------------------------------------------------------------------------
static void pv_cmd_read_fuses(void)
{
	// Lee los fuses.

uint8_t fuse0 = 0;
uint8_t fuse1 = 0;
uint8_t fuse2 = 0;
uint8_t fuse4 = 0;
uint8_t fuse5 = 0;

	fuse0 = nvm_fuses_read(0x00);	// FUSE0
	xprintf_P( PSTR("FUSE0=0x%x\r\n\0"),fuse0);

	fuse1 = nvm_fuses_read(0x01);	// FUSE1
	xprintf_P( PSTR("FUSE1=0x%x\r\n\0"),fuse1);

	fuse2 = nvm_fuses_read(0x02);	// FUSE2
	xprintf_P( PSTR("FUSE2=0x%x\r\n\0"),fuse2);

	fuse4 = nvm_fuses_read(0x04);	// FUSE4
	xprintf_P( PSTR("FUSE4=0x%x\r\n\0"),fuse4);

	fuse5 = nvm_fuses_read(0x05);	// FUSE5
	xprintf_P( PSTR("FUSE5=0x%x\r\n\0"),fuse5);

	if ( (fuse0 != 0xFF) || ( fuse1 != 0xAA) || (fuse2 != 0xFD) || (fuse4 != 0xF5) || ( fuse5 != 0xD6) ) {
		xprintf_P( PSTR("FUSES ERROR !!!.\r\n\0"));
		xprintf_P( PSTR("Los valores deben ser: FUSE0=0xFF,FUSE1=0xAA,FUSE2=0xFD,FUSE4=0xF5,FUSE5=0xD6\r\n\0"));
		xprintf_P( PSTR("Deben reconfigurarse !!.\r\n\0"));
		pv_snprintfP_ERR();
		return;
	}
	pv_snprintfP_OK();
	return;
}
//------------------------------------------------------------------------------------
static void pv_cmd_I2Cscan(bool busscan)
{

bool retS = false;
uint8_t i2c_address;


	// Scan de una direccion
	if ( busscan == false ) {
		i2c_address = atoi(argv[2]);
		retS = I2C_scan_device(i2c_address);
		if (retS) {
			xprintf_P( PSTR("I2C device found at 0x%02x\r\n\0"), i2c_address );
		} else {
			xprintf_P( PSTR("I2C device NOT found at 0x%02x\r\n\0"), i2c_address );
		}
		return;
	}

	// Scan de todo el bus.00..FF.
	// Solo muestro las direcciones donde encuentro un device.
	for ( i2c_address = 0x00; i2c_address < 0xFF; i2c_address++ ) {
		retS = I2C_scan_device(i2c_address);
		if (retS) {
			xprintf_P( PSTR("I2C device found at 0x%02x\r\n\0"), i2c_address );
		};
	}

}
//------------------------------------------------------------------------------------
static void pv_cmd_rwLORA(uint8_t cmd_mode )
{

uint8_t i,j;
int16_t free_size = sizeof(txbuffer);

	if ( cmd_mode == WR_CMD ) {

		memset(txbuffer,'\0', sizeof(txbuffer) );
		// write lora {cmd}
		i = 2;
		j = 0;
		while ( argv[i] != NULL ) {
			//xprintf_P(PSTR("DEBUG0: i=%d, j=%d, s=%s\r\n"),i,j,argv[i]);
			j += snprintf_P( &txbuffer[j], free_size, PSTR("%s "), argv[i] );
			free_size = (  sizeof(txbuffer) - j );
			if ( free_size < 0 ) {
				xprintf_P(PSTR("buffer ERROR\r\n"));
				return;
			}
			i++;
		}

		//xprintf_P(PSTR("DEBUG0: j=%d\r\n"),j);
		//xprintf_P(PSTR("DEBUG0: s=%d\r\n"), strlen(txbuffer));
		i = strlen(txbuffer);
		txbuffer[--i] = '\0';

		lora_flush_RX_buffer();
		xfprintf_P( fdLORA, PSTR("%s\r\n"),txbuffer );
		xprintf_P( PSTR("sent->%s\r\n"),txbuffer );
		vTaskDelay( ( TickType_t)( 250 / portTICK_RATE_MS ) );
		lora_read_rx_data();
		lora_print_RX_buffer();
		return;
	}

	if ( cmd_mode == RD_CMD ) {
		// read lora
		lora_read_rx_data();
		lora_print_RX_buffer();
		return;
	}

	pv_snprintfP_ERR();
	return;

}
//------------------------------------------------------------------------------------

