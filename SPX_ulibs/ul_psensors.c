/*
 * ul_psensors.c
 *
 *  Created on: 13 may. 2021
 *      Author: pablo
 */


#include "ul_psensors.h"

//------------------------------------------------------------------------------------
float psensor_read( t_presion_id presion_id, t_presion_format formato, bool fdebug )
{
	// pid: { PA, PB }
	// formato: { PSI, KGM_CM2 }
	// fdebug: { true, false }


uint8_t data[2];
uint8_t status;
uint16_t p_counts;
float presion_psi = -1.0;


	// Leo la presion
	if (  abp_raw_read( presion_id, (char *)data ) > 0 ) {
		status = data[0] >> 6;
		p_counts = ((data[0] & 0x3F) << 8) + data[1];
		if ( p_counts < ABP_COUNTS_MIN ) {
			presion_psi = 0.0;
		} else {
			presion_psi = ABP_PMAX_RATED * (p_counts - ABP_COUNTS_MIN ) / ( ABP_COUNTS_MAX - ABP_COUNTS_MIN );
		}

		if (fdebug) {
			xprintf_P(PSTR("read_presion: ABP_LOW =0x%02X\r\n"), data[1]);
			xprintf_P(PSTR("read_presion: ABP_HIGH=0x%02X\r\n"), data[0]);
			xprintf_P(PSTR("read_presion: Status=x%02X\r\n"), status);
			xprintf_P(PSTR("read_presion: pcounts=%05dd (0x%04X)\r\n"), p_counts, p_counts );
			xprintf_P(PSTR("read_presion: Presion: %.03f (psi)\r\n"), presion_psi );
			xprintf_P(PSTR("read_presion: Presion: %.03f (kg/cm2)\r\n"), presion_psi *0.07 );
		}

		switch(formato) {
		case PSI:
			return(presion_psi);
			break;
		case KGM_CM2:
			return (presion_psi * 0.07);
			break;
		default:
			xprintf_P(PSTR("read_presion: ERROR: formato no es correcto\r\n"));
			return(-1.0);
		}
	}
	xprintf_P(PSTR("read_presion: ERROR.\r\n"));
	return(-1.0);

}
//------------------------------------------------------------------------------------
bool psensor_read_test(void )
{

	// Funcion utilizada desde cmdMode para diagnostico de los sensores de presion
	// Pasa como parametro el presion_id { PA , PB } para seleccionar el sensor correcto

t_presion_id presion_id;


	presion_id = PA;
	if ( psensor_read(  presion_id , PSI, true ) < 0 ) {
		return(false);
	}
	return(true);

}
//------------------------------------------------------------------------------------
void psensor_config_defaults(void)
{
	// Realiza la configuracion por defecto de los canales digitales.

	systemVars.psensor_conf.mmin = 0.0;
	systemVars.psensor_conf.mmax = 10.0;

}
//------------------------------------------------------------------------------------
void psensor_config_print(void)
{

	// Imprime en la terminal (status ) la configuracion de los canales de presion.
	xprintf_P( PSTR("  PA: [%.02f-%.02f]\r\n"), systemVars.psensor_conf.mmin, systemVars.psensor_conf.mmax );
}
//------------------------------------------------------------------------------------
