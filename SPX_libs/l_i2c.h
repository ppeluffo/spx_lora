/*
 * l_i2c.h
 *
 *  Created on: 26 de mar. de 2018
 *      Author: pablo
 */

#ifndef SPX_LIBS_L_I2C_H_
#define SPX_LIBS_L_I2C_H_

#include "frtos-io.h"

#include "l_printf.h"
#include "avr/pgmspace.h"

#define BUSADDR_EEPROM_M2402	0xA0
#define BUSADDR_RTC_M79410		0xDE
#define BUSADDR_MCP23018		0x4E
#define BUSADDR_INA_A			0x80
#define BUSADDR_INA_B			0x82
#define BUSADDR_INA_C			0x86
#define BUSADDR_BPS120			0x50
#define BUSADDR_ABP				0x50
#define BUSADDR_ADT7410			0x90
#define BUSADDR_TCA9546			0xE0

bool I2C_scan_device( uint8_t i2c_bus_address );
void I2C_reinit_devices(void);

void I2C_get_semaphore(void);
void I2C_release_semaphore(void);
int8_t I2C_write_R1 ( uint8_t i2c_bus_address, uint32_t chip_address, uint8_t chip_address_length, char *data, uint8_t length );
int8_t I2C_read_R1( uint8_t i2c_bus_address, char *data, uint8_t length );

#endif /* SPX_LIBS_L_I2C_H_ */
