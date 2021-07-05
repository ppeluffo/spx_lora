/*
 * ds18b20.h
 *
 *  Created on: 3 jul. 2021
 *      Author: pablo
 */

#ifndef AVR_LIBS_INCLUDE_1_WIRE_H_
#define AVR_LIBS_INCLUDE_1_WIRE_H_

#include <avr/io.h>
#include "l_iopines.h"

//setup connection

//commands
#define DS18B20_CMD_CONVERTTEMP 0x44
#define DS18B20_CMD_RSCRATCHPAD 0xbe
#define DS18B20_CMD_WSCRATCHPAD 0x4e
#define DS18B20_CMD_CPYSCRATCHPAD 0x48
#define DS18B20_CMD_RECEEPROM 0xb8
#define DS18B20_CMD_RPWRSUPPLY 0xb4
#define DS18B20_CMD_SEARCHROM 0xf0
#define DS18B20_CMD_READROM 0x33
#define DS18B20_CMD_MATCHROM 0x55
#define DS18B20_CMD_SKIPROM 0xcc
#define DS18B20_CMD_ALARMSEARCH 0xec

//stop any interrupt on read
#define DS18B20_STOPINTERRUPTONREAD 1

//functions

#endif /* AVR_LIBS_INCLUDE_1_WIRE_H_ */
