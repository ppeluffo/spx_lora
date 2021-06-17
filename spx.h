/*
 * spx.h
 *
 *  Created on: 8 dic. 2018
 *      Author: pablo
 */

#ifndef SRC_SPX_H_
#define SRC_SPX_H_

//------------------------------------------------------------------------------------
// INCLUDES
//------------------------------------------------------------------------------------
#include <avr/io.h>
#include <avr/wdt.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <compat/deprecated.h>
#include <avr/pgmspace.h>
#include <stdarg.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <avr/sleep.h>
#include <ctype.h>
#include "avr_compiler.h"
#include "clksys_driver.h"
#include <inttypes.h>
#include <l_counters_plt.h>
#include "TC_driver.h"
#include "pmic_driver.h"
#include "wdt_driver.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "list.h"
#include "croutine.h"
#include "semphr.h"
#include "timers.h"
#include "limits.h"
#include "portable.h"

#include "frtos-io.h"
#include "FRTOS-CMD.h"

#include "l_iopines.h"
#include "l_i2c.h"
#include "l_iopines.h"
#include "l_nvm.h"
#include "l_printf.h"
#include "l_bytes.h"
#include "l_abp.h"

#include "ul_psensors.h"
#include "ul_counters.h"
#include "ul_lora.h"

//------------------------------------------------------------------------------------
// DEFINES
//------------------------------------------------------------------------------------
#define SPX_FW_REV "1.0.0a"
#define SPX_FW_DATE "@ 20210617"

#define SPX_HW_MODELO "spxR5 HW:xmega256A3B R1.1"
#define SPX_FTROS_VERSION "FW:FRTOS10 Lora."

//#define F_CPU (32000000UL)

//#define SYSMAINCLK 2
//#define SYSMAINCLK 8
#define SYSMAINCLK 32
//
#define CHAR32	32
#define CHAR64	64
#define CHAR128	128
#define CHAR256	256

#define tkCtl_STACK_SIZE		384
#define tkCmd_STACK_SIZE		384
#define tkData_STACK_SIZE		384

StaticTask_t xTask_Ctl_Buffer_Ptr;
StackType_t xTask_Ctl_Buffer [tkCtl_STACK_SIZE];

StaticTask_t xTask_Cmd_Buffer_Ptr;
StackType_t xTask_Cmd_Buffer [tkCmd_STACK_SIZE];

StaticTask_t xTask_Data_Buffer_Ptr;
StackType_t xTask_Data_Buffer [tkData_STACK_SIZE];

#define tkCtl_TASK_PRIORITY	 		( tskIDLE_PRIORITY + 1 )
#define tkCmd_TASK_PRIORITY	 		( tskIDLE_PRIORITY + 1 )
#define tkData_TASK_PRIORITY	 	( tskIDLE_PRIORITY + 1 )

TaskHandle_t xHandle_idle, xHandle_tkCtl, xHandle_tkCmd, xHandle_tkData;

bool startTask;
uint32_t sysTicks;

xSemaphoreHandle sem_SYSVars;
StaticSemaphore_t SYSVARS_xMutexBuffer;
#define MSTOTAKESYSVARSSEMPH ((  TickType_t ) 10 )

xSemaphoreHandle sem_WDGS;
StaticSemaphore_t WDGS_xMutexBuffer;
#define MSTOTAKEWDGSSEMPH ((  TickType_t ) 10 )

xSemaphoreHandle sem_AINPUTS;
StaticSemaphore_t AINPUTS_xMutexBuffer;
#define MSTOTAKEAINPUTSSEMPH ((  TickType_t ) 10 )

void tkCtl(void * pvParameters);
void tkCmd(void * pvParameters);
void tkData(void * pvParameters);

typedef enum { DEBUG_NONE = 0, DEBUG_COUNTER, DEBUG_DATA } t_debug;

#define fdTERM fdUSARTF
#define fdLORA fdUSARTD
//--------------------------------------------------------------------------------------------------
#define PARAMNAME_LENGTH	7
#define LORA_KEYS_LENGTH	38

// Configuracion de canales de contadores
typedef struct {
	float magpp;
	uint16_t pwidth;
	uint16_t period;
	t_sensing_edge sensing_edge;
} counter_conf_t;

// Configuracion de canales analogicos
// Son sensores integrados de 0-10bar
// Los sensores son 2 y las presiones son PA y PB
typedef struct {
	float mmin;
	float mmax;
} psensors_conf_t;

typedef struct {
	char deveui[LORA_KEYS_LENGTH];
	char appeui[LORA_KEYS_LENGTH];
	char appkey[LORA_KEYS_LENGTH];
	char devaddr[LORA_KEYS_LENGTH];
	char nwkskey[LORA_KEYS_LENGTH];
	char appskey[LORA_KEYS_LENGTH];
	char join[LORA_KEYS_LENGTH];
} lora_conf_t;


typedef struct {
	// Variables de trabajo.
	t_debug debug;
	uint16_t timer_poll;
	counter_conf_t counter_conf;	// Estructura con la configuracion de los contadores
	psensors_conf_t psensor_conf;
	lora_conf_t	lora_conf;
	uint8_t checksum;				// El checksum DEBE ser el ultimo byte del systemVars !!!!

} systemVarsType;

systemVarsType systemVars;

typedef struct {
	float presion;
	float caudal;
} u_dataRecord_t;

//--------------------------------------------------------------------------------------------------

// UTILS
void initMCU(void);
void u_configure_systemMainClock(void);
void u_configure_RTC32(void);
uint8_t u_checksum( uint8_t *s, uint16_t size );
void u_load_defaults( char *opt );
void u_save_params_in_NVMEE(void);
bool u_load_params_from_NVMEE(void);
void u_config_timerpoll ( char *s_timerpoll );
void u_print_dr(file_descriptor_t fd, u_dataRecord_t *dr, uint16_t ctl );;
void u_lora_init(void);

// TKCTL
void ctl_watchdog_kick(uint8_t taskWdg, uint16_t timeout_in_secs );
bool ctl_terminal_connected(void);

// TKDATA
void data_read_frame(bool poll_now );

// WATCHDOG
uint8_t wdg_resetCause;

#define WDG_CTL			0
#define WDG_CMD			1
#define WDG_DATA		2

#define NRO_WDGS		3

#define WDG_TO30		30
#define WDG_TO60		60
#define WDG_TO120		120
#define WDG_TO180	 	180
#define WDG_TO300		300
#define WDG_TO600		600
#define WDG_TO900		900

//------------------------------------------------------------------------


#endif /* SRC_SPX_H_ */
