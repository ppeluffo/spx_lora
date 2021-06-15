/*
 * drv_i2c_spx.h
 *
 *  Created on: 8 dic. 2018
 *      Author: pablo
 */

#ifndef SRC_SPX_DRIVERS_DRV_I2C_SPX_H_
#define SRC_SPX_DRIVERS_DRV_I2C_SPX_H_

#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdbool.h>
#include <util/twi.h>
#include "FreeRTOS.h"
#include "task.h"

#include "l_iopines.h"

#define SCL		0
#define SDA		1
#define I2C_MAXTRIES	5

#define ACK 	0
#define NACK 	1

int drv_I2C_master_write_R1 ( const uint8_t devAddress, const uint32_t chipAddress, const uint8_t chipAddressLength, char *pvBuffer, size_t xBytes );
int drv_I2C_master_read_R1  ( const uint8_t devAddress, char *pvBuffer, size_t xBytes );

void drv_I2C_init(void);
bool drv_I2C_scan_device( const uint8_t devAddress );

//#define DEBUG_I2C

#endif /* SRC_SPX_DRIVERS_DRV_I2C_SPX_H_ */
