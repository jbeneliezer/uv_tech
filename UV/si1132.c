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

bool si1132_init(GPIO_Port_TypeDef power_port, uint8_t power_pin, I2C_TypeDef *i2c, uint8_t slave_addr)
{
	uint8_t ucoef[] = {0x7B, 0x6B, 0x01, 0x00};

	GPIO_PinOutSet(power_port, power_pin);		//power on i2c

	sl_udelay_wait(STARTUP_TIME);				//allow time for startup
												//Note: recommended by the data sheet before any I2C activity

	/* write 0x17 to HW_KEY for proper operation */
	if (i2c_write_single(i2c, DEFAULT_ADDR, HW_KEY, 0x17) != i2cTransferDone) return false;

	/* set device I2C address */
	if (!si1132_write_ram(i2c, DEFAULT_ADDR, I2C_ADDR, slave_addr)) return false;
	if ((si1132_send_command_BUSADDR(i2c, DEFAULT_ADDR, slave_addr) & 0xF0) != NOERROR ) return false;

	/* enable UV detection */
	if (!si1132_write_ram(i2c, slave_addr, CHLIST, EN_UV)) return false;

	/* configure UCOEF for direct sunlight */
	if (i2c_write_burst(i2c, slave_addr, UCOEF0, ucoef, 4) != i2cTransferDone) return false;

	/* enable vis_range and ir_range */
	if (!si1132_write_ram(i2c, slave_addr, ALS_VIS_ADC_MISC, VIS_RANGE) != i2cTransferDone) return false;
	if (!si1132_write_ram(i2c, slave_addr, ALS_IR_ADC_MISC, IR_RANGE) != i2cTransferDone) return false;

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
