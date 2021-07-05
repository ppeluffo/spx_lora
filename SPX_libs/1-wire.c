/*
 * ds18b20.c
 *
 *  Created on: 3 jul. 2021
 *      Author: pablo
 */

#include <1-wire.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "l_printf.h"

uint8_t pv_1wire_reset(void);
void pv_1wire_writebit(uint8_t bit);
uint8_t pv_1wire_readbit(void);
uint8_t pv_1wire_readbyte(void);
void pv_1wire_writebyte(uint8_t byte);

//------------------------------------------------------------------------------------
/*
 * get temperature
 */
double one_wire_gettemp(void )
{
uint8_t temperature_l;
uint8_t temperature_h;
double retd = 0;

	// El pin del 1WIRE debe tener un pullup
	PORTA.PIN2CTRL |= PORT_OPC_PULLUP_gc;

#if DS18B20_STOPINTERRUPTONREAD == 1
	cli();
#endif

	pv_1wire_reset(); //reset
	pv_1wire_writebyte(DS18B20_CMD_SKIPROM); //skip ROM
	pv_1wire_writebyte(DS18B20_CMD_CONVERTTEMP); //start temperature conversion

	while(! pv_1wire_readbit() ); //wait until conversion is complete

	pv_1wire_reset(); //reset
	pv_1wire_writebyte(DS18B20_CMD_SKIPROM); //skip ROM
	pv_1wire_writebyte(DS18B20_CMD_RSCRATCHPAD); //read scratchpad

	//read 2 byte from scratchpad
	temperature_l = pv_1wire_readbyte();
	temperature_h = pv_1wire_readbyte();

#if DS18B20_STOPINTERRUPTONREAD == 1
	sei();
#endif

	//convert the 12 bit value obtained
	retd = ( ( temperature_h << 8 ) + temperature_l ) * 0.0625;

	return retd;
}
//------------------------------------------------------------------------------------
void one_wire_read_scratch(void)
{

uint8_t scratch[9];
uint8_t i;

	// El pin del 1WIRE debe tener un pullup
	PORTC.PIN0CTRL |= PORT_OPC_PULLUP_gc;

#if DS18B20_STOPINTERRUPTONREAD == 1
	cli();
#endif


	pv_1wire_reset(); //reset
	pv_1wire_writebyte(DS18B20_CMD_SKIPROM); //skip ROM
	pv_1wire_writebyte(DS18B20_CMD_RSCRATCHPAD); //read scratchpad

	//read scratchpad
	for (i=0; i<9; i++) {
		scratch[i] = pv_1wire_readbyte();
	}


#if DS18B20_STOPINTERRUPTONREAD == 1
	sei();
#endif

	for (i=0; i<9; i++) {
		xprintf_P(PSTR("SCRATCH_%02d=0x%02X\r\n"), i, scratch[i]);
	}

}
//------------------------------------------------------------------------------------
uint8_t pv_1wire_reset(void)
{

uint8_t i;

	//low for 480us
	IO_clr_1WIRE(); 			//low
	IO_config_1WIRE_ASOUTPUT(); //output
	_delay_us(480);

	//release line and wait for 60uS
	IO_config_1WIRE_ASINPUT(); //input
	_delay_us(60);

	//get value and wait 420us
	i = IO_read_ONE_WIRE_PIN();
	_delay_us(420);

	//return the read value, 0=ok, 1=error
	return i;
}
//------------------------------------------------------------------------------------
void pv_1wire_writebit(uint8_t bit)
{

	//low for 1uS
	IO_clr_1WIRE(); 			//low
	IO_config_1WIRE_ASOUTPUT(); //output
	_delay_us(1);

	//if we want to write 1, release the line (if not will keep low)
	if(bit)
		IO_config_1WIRE_ASINPUT(); //input

	//wait 60uS and release the line
	_delay_us(60);
	IO_config_1WIRE_ASINPUT(); //input
}
//------------------------------------------------------------------------------------
uint8_t pv_1wire_readbit(void)
{

uint8_t bit=0;

	//low for 1uS
	IO_clr_1WIRE(); 			//low
	IO_config_1WIRE_ASOUTPUT(); //output
	_delay_us(1);

	//release line and wait for 14uS
	IO_config_1WIRE_ASINPUT(); //input
	_delay_us(14);

	//read the value
	if( IO_read_ONE_WIRE_PIN() == 1 )
		bit=1;

	//wait 45uS and return read value
	_delay_us(45);
	return bit;
}
//------------------------------------------------------------------------------------
void pv_1wire_writebyte(uint8_t byte)
{

uint8_t i=8;

	while(i--){
		pv_1wire_writebit(byte&1);
		byte >>= 1;
	}
}
//------------------------------------------------------------------------------------
uint8_t pv_1wire_readbyte(void)
{

uint8_t i=8, n=0;

while(i--){
		n >>= 1;
		n |= (pv_1wire_readbit()<<7);
	}
	return n;
}
//------------------------------------------------------------------------------------
