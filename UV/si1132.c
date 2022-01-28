/*
 * si1132.c
 *
 *  Created on: Oct 13, 2021
 *  Last Modified: Jan 26, 2022
 *      Author: Judah Ben-Eliezer, Tennyson Cheng
 *
 *  All code related to the Si1132 i2c interface.
 *	Intended for UV data measurement uses.
 *
 */

//#include <stdio.h>
//#include <string.h>
//#include <stdbool.h>
#include "sl_udelay.h"
#include "em_gpio.h"
#include "em_i2c.h"
#include "i2c.h"
#include "si1132.h"

struct cal_ref_t calref =
{
	FLT_TO_FX20( 4.021290),  // sirpd_adchi_irled
	FLT_TO_FX20(57.528500),  // sirpd_adclo_irled
	FLT_TO_FX20( 2.690010),  // sirpd_adclo_whled
	FLT_TO_FX20( 0.042903),  // vispd_adchi_whled
	FLT_TO_FX20( 0.633435),  // vispd_adclo_whled
	FLT_TO_FX20(23.902900),  // lirpd_adchi_irled
	FLT_TO_FX20(56.889300),  // ledi_65ma
	{0x7B, 0x6B, 0x01, 0x00} // default ucoef
};

bool si1132_init(GPIO_Port_TypeDef power_port, uint8_t power_pin, I2C_TypeDef *i2c, uint8_t slave_addr)
{
	GPIO_PinOutSet(power_port, power_pin);		//power on i2c

	sl_udelay_wait(STARTUP_TIME);				//allow time for startup
												//Note: recommended by the data sheet before any I2C activity

	/* write 0x17 to HW_KEY for proper operation */
	if (i2c_write_single(i2c, DEFAULT_ADDR, HW_KEY, 0x17) != i2cTransferDone) return false;

	/* set device I2C address */
	if (!si1132_write_ram(i2c, DEFAULT_ADDR, I2C_ADDR, slave_addr)) return false;
	if ((si1132_send_command_BUSADDR(i2c, DEFAULT_ADDR, slave_addr) & 0xF0) != NOERROR ) return false;

	/* configure for forced measurement mode */
	if (i2c_write_single(i2c, slave_addr, MEAS_RATE0, 0x00) != i2cTransferDone) return false;
	if (i2c_write_single(i2c, slave_addr, MEAS_RATE1, 0x00) != i2cTransferDone) return false;

	/* enable vis_range and ir_range */
	if (!si1132_write_ram(i2c, slave_addr, ALS_VIS_ADC_MISC, VIS_RANGE) != i2cTransferDone) return false;
	if (!si1132_write_ram(i2c, slave_addr, ALS_IR_ADC_MISC, IR_RANGE) != i2cTransferDone) return false;

	/* calibrate the sensor */
	if (!si1132_calibrate(i2c, slave_addr)) return false;

	/* enable UV detection */
	if (!si1132_write_ram(i2c, slave_addr, CHLIST, EN_UV)) return false;

	return true;
}

bool si1132_calibrate(I2C_TypeDef *i2c, uint8_t slave_addr) {
	//Send command to fetch calibration data
	if ((si1132_send_command(i2c, slave_addr, GET_CAL) & 0xF0) != NOERROR ) return false;

	//Calibration data is located in register addresses 0x22-0x2D
	uint8_t cal[12];
	if (i2c_read_burst(i2c, slave_addr, ALS_VIS_DATA0, cal, 12) != i2cTransferDone) return false;

	//Processing of calibration data for UCOEF values
	uint32_t         long_temp;
	struct operand_t op;
	uint8_t          out_ucoef[4];

	op.op1 = calref.ucoef[0] + ((calref.ucoef[1])<<8);
	op.op2 = vispd_correction(cal);
	long_temp   = fx20_multiply( &op );
	out_ucoef[0] = (long_temp & 0x00ff);
	out_ucoef[1] = (long_temp & 0xff00)>>8;

	op.op1 = calref.ucoef[2] + (calref.ucoef[3]<<8);
	op.op2 = irpd_correction(cal);
	long_temp   = fx20_multiply( &op );
	out_ucoef[2] = (long_temp & 0x00ff);
	out_ucoef[3] = (long_temp & 0xff00)>>8;

	//Write back calibrated UCOEF values back to Si1132
	if (i2c_write_burst(i2c, slave_addr, UCOEF0, out_ucoef, 4) != i2cTransferDone) return false;
	return true;
}

uint8_t si1132_send_command(I2C_TypeDef *i2c, uint8_t slave_addr, uint8_t command)
{
	uint8_t response = 0;

	/* clear response register */
	if (i2c_write_single(i2c, slave_addr, COMMAND, NOP) != i2cTransferDone) return response;
	if (i2c_read_single(i2c, slave_addr, RESPONSE, &response) != i2cTransferDone) return response;

	/* send command to command register */
	if (i2c_write_single(i2c, slave_addr, COMMAND, command) != i2cTransferDone) return response;
	if (i2c_read_single(i2c, slave_addr, RESPONSE, &response) != i2cTransferDone) return response;

	return response;
}

uint8_t si1132_send_command_BUSADDR(I2C_TypeDef *i2c, uint8_t slave_addr, uint8_t slave_addr_new)
{
	uint8_t response = 0;

	/* clear response register */
	if (i2c_write_single(i2c, slave_addr, COMMAND, NOP) != i2cTransferDone) return response;
	if (i2c_read_single(i2c, slave_addr, RESPONSE, &response) != i2cTransferDone) return response;

	/* send BUSADDR to command register */
	if (i2c_write_single(i2c, slave_addr, COMMAND, BUSADDR) != i2cTransferDone) return response;
	if (i2c_read_single(i2c, slave_addr_new, RESPONSE, &response) != i2cTransferDone) return response;

	return response;
}

bool si1132_write_ram(I2C_TypeDef *i2c, uint8_t slave_addr, uint8_t addr, uint8_t data)
{
	/* Write data to PARAM_WR */
	if ((i2c_write_single(i2c, slave_addr, PARAM_WR, data)) != i2cTransferDone) return false;

	uint8_t ram_addr = (addr & 0x1F) | PARAM_SET;

	/* Write PARAM_WR to ram */
	if ((si1132_send_command(i2c, slave_addr, ram_addr) & 0xF0) != NOERROR) return false;
	return true;
}

bool si1132_read_ram(I2C_TypeDef *i2c, uint8_t slave_addr, uint8_t addr, uint8_t buffer)
{
	uint8_t ram_addr = (addr & 0x1F) | PARAM_QUERY;
	if ((si1132_send_command(i2c, slave_addr, ram_addr) & 0xF0) != NOERROR) return false;
	return (i2c_read_single(i2c, slave_addr, PARAM_RD, &buffer) == i2cTransferDone);
}

uint8_t si1132_UV_start_measurement(I2C_TypeDef *i2c, uint8_t slave_addr)
{
	uint8_t status;

	/* send conversion command to I2C */
	if ((status = (si1132_send_command(i2c, slave_addr, ALS_FORCE)) & 0xF0) != NOERROR) return status;

	/* allow time for conversion */
	sl_udelay_wait(CONVERSION_TIME);

	return status;
}

uint8_t si1132_UV_read_measurement(I2C_TypeDef *i2c, uint8_t slave_addr, uint16_t *buffer)
{
	uint8_t data[2], status;

	/* read word into buffer */
	if (i2c_read_burst(i2c, slave_addr, UVINDEX0, data, 2) != i2cTransferDone) return I2C_TRANSFER_ERROR;

	*buffer = (data[1] << 8) | data[0];

	return status;
}

/***************************************************************************//**
 * CODE BELOW HAS BEEN PROVIDED BY SILICON LABS
 * "si114x_function.c"
 *
 * Slightly modified for calibration of Si1132 chips only
 ******************************************************************************/
uint32_t decode(uint32_t input)
{
	int32_t  exponent, exponent_bias9;
	uint32_t mantissa;

	if(input==0) return 0.0;

	exponent_bias9 = (input & 0x0f00) >> 8;
	exponent       = exponent_bias9 - 9;

	mantissa       = input & 0x00ff; // fraction
	mantissa       |=        0x0100; // add in integer

	// representation in 12 bit integer, 20 bit fraction
	mantissa       = mantissa << (12+exponent);
	return mantissa;
}
/***************************************************************************/
uint32_t collect(uint8_t* buffer,
						uint8_t msb_addr,
						uint8_t lsb_addr,
						uint8_t alignment)
{
	uint16_t value;
	uint8_t  msb_ind = msb_addr - 0x22;
	uint8_t  lsb_ind = lsb_addr - 0x22;

	if(alignment == 0)
	{
		value =  buffer[msb_ind]<<4;
		value += buffer[lsb_ind]>>4;
	}
	else
	{
		value =  buffer[msb_ind]<<8;
		value += buffer[lsb_ind];
		value &= 0x0fff;
	}

	if(    ( value == 0x0fff )
		|| ( value == 0x0000 ) ) return FX20_BAD_VALUE;
	else return decode( value );
}
/***************************************************************************/
void shift_left(uint32_t* value_p, int8_t shift)
{
	if(shift > 0)
		*value_p = *value_p<<shift ;
	else
		*value_p = *value_p>>(-shift) ;
}
/***************************************************************************/
int8_t align( uint32_t* value_p, int8_t direction )
{
	int8_t   local_shift, shift ;
	uint32_t mask;

	// Check invalid value_p and *value_p, return without shifting if bad.
//	if( value_p  == NULL )  return 0;
	if( *value_p == 0 )     return 0;

	// Make sure direction is valid
	switch( direction )
	{
		case ALIGN_LEFT:
			local_shift =  1 ;
			mask  = 0x80000000L;
			break;

		case ALIGN_RIGHT:
			local_shift = -1 ;
			mask  = 0x00000001L;
			break;

		default:
			// Invalid direction, return without shifting
			return 0;
	}

	shift = 0;
	while(1)
	{
		if(*value_p & mask ) break;
		shift++;
		shift_left( value_p, local_shift );
	}
	return shift;
}
/***************************************************************************/
void fx20_round(uint32_t *value_p)
{
	int8_t  shift;

	uint32_t mask1  = 0xffff8000;
	uint32_t mask2  = 0xffff0000;
	uint32_t lsb    = 0x00008000;

	shift = align( value_p, ALIGN_LEFT );
	if( ( (*value_p)&mask1 ) == mask1 )
	{
		*value_p = 0x80000000;
		shift -= 1;
	}
	else
	{
		*value_p += lsb;
		*value_p &= mask2;
	}

	shift_left( value_p, -shift );
}
/***************************************************************************/
uint32_t fx20_divide( struct operand_t* operand_p )
{
	int8_t    numerator_sh=0, denominator_sh=0;
	uint32_t  result;
	uint32_t* numerator_p;
	uint32_t* denominator_p;

//	if( operand_p == NULL ) return FX20_BAD_VALUE;

	numerator_p   = &operand_p->op1;
	denominator_p = &operand_p->op2;

	if(   (*numerator_p   == FX20_BAD_VALUE)
	   || (*denominator_p == FX20_BAD_VALUE)
	   || (*denominator_p == 0             ) ) return FX20_BAD_VALUE;

	fx20_round  ( numerator_p   );
	fx20_round  ( denominator_p );
	numerator_sh   = align ( numerator_p,   ALIGN_LEFT  );
	denominator_sh = align ( denominator_p, ALIGN_RIGHT );

	result = *numerator_p / ( (uint16_t)(*denominator_p) );
	shift_left( &result , 20-numerator_sh-denominator_sh );

	return result;
}
/***************************************************************************/
uint32_t fx20_multiply( struct operand_t* operand_p )
{
	uint32_t  result;
	int8_t    val1_sh, val2_sh;
	uint32_t* val1_p;
	uint32_t* val2_p;

//	if( operand_p == NULL ) return FX20_BAD_VALUE;

	val1_p = &(operand_p->op1);
	val2_p = &(operand_p->op2);

	fx20_round( val1_p );
	fx20_round( val2_p );

	val1_sh = align( val1_p, ALIGN_RIGHT );
	val2_sh = align( val2_p, ALIGN_RIGHT );


	result = (uint32_t)( ( (uint32_t)(*val1_p) ) * ( (uint32_t)(*val2_p) ) );
	shift_left( &result, -20+val1_sh+val2_sh );

	return result;
}
/***************************************************************************/
uint32_t vispd_correction(uint8_t *buffer)
{
	struct operand_t op;
	uint32_t         result;

	op.op1 = calref.vispd_adclo_whled;
	op.op2 = VISPD_ADCLO_WHLED;
	result = fx20_divide( &op );

	if( result == FX20_BAD_VALUE ) result = FX20_ONE;

	return result;
}
/***************************************************************************/
uint32_t irpd_correction(uint8_t *buffer)
{
	struct operand_t op;
	uint32_t         result;

	// op.op1 = SIRPD_ADCLO_IRLED_REF; op.op2 = SIRPD_ADCLO_IRLED;
	op.op1 = calref.sirpd_adclo_irled;
	op.op2 = SIRPD_ADCLO_IRLED;
	result = fx20_divide( &op );

	if( result == FX20_BAD_VALUE ) result = FX20_ONE;

	return result;
}
///***************************************************************************/
//static uint32_t adcrange_ratio(uint8_t *buffer)
//{
//	struct operand_t op;
//	uint32_t         result;
//
//	op.op1 = SIRPD_ADCLO_IRLED  ; op.op2 = SIRPD_ADCHI_IRLED  ;
//	result = fx20_divide( &op );
//
//	if( result == FX20_BAD_VALUE ) result = FLT_TO_FX20( 14.5 );
//
//	return result;
//}
///***************************************************************************/
//static uint32_t irsize_ratio(uint8_t *buffer)
//{
//	struct operand_t op;
//	uint32_t         result;
//
//	op.op1 = LIRPD_ADCHI_IRLED  ; op.op2 = SIRPD_ADCHI_IRLED  ;
//
//	result = fx20_divide( &op );
//
//	if( result == FX20_BAD_VALUE ) result = FLT_TO_FX20(  6.0 );
//
//	return  result;
//}
///***************************************************************************/
//static uint32_t ledi_ratio(uint8_t *buffer)
//{
//	struct operand_t op;
//	uint32_t         result;
//
//	// op.op1 = LED_DRV65_REF; op.op2 = LED_DRV65;
//	op.op1 = calref.ledi_65ma;
//	op.op2 = LED_DRV65;
//	result = fx20_divide( &op );
//
//	if( result == FX20_BAD_VALUE ) result = FX20_ONE;
//
//	return result;
//}
