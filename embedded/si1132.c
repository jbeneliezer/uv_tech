/*
 * si1132.c
 *
 *  Created on: Oct 13, 2021
 *      Author: Judah Ben-Eliezer
 *
 *  All code related to the Si1132 i2c interface.
 *
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "sl_udelay.h"
#include "em_gpio.h"
#include "sl_i2cspm_instances.h"
#include "si1132.h"

bool si1132_init(GPIO_Port_TypeDef power_port, uint8_t power_pin,
                 sl_i2cspm_t *i2c, uint8_t slave_id)
{
	uint8_t ucoef[] = {0x7B, 0x6B, 0x01, 0x00};

	GPIO_PinOutSet(power_port, power_pin);	// power on i2c

	sl_udelay_wait(STARTUP_TIME);   // allow time for startup

	/* write 0x17 to HW_KEY for proper operation */
	if (!single_write(i2c, DEFAULT_ADDR, HW_KEY, 0x17)) return false;

	/* set device address */
	if (!write_ram(i2c, DEFAULT_ADDR, I2C_ADDR, slave_id)) return false;
	if (send_command(i2c, DEFAULT_ADDR, BUSADDR).valid != true) return false;

	/* enable uv detection */
	if (!write_ram(i2c, slave_id, CHLIST, EN_UV)) return false;

	/* configure ucoef for direct sunlight */
	if (!burst_write(i2c, slave_id, UCOEF0, ucoef, 4)) return false;

	/* enable vis_range and ir_range */
	if (!single_write(i2c, slave_id, ALS_VIS_ADC_MISC, VIS_RANGE)) return false;
	if (!single_write(i2c, slave_id, ALS_IR_ADC_MISC, IR_RANGE)) return false;

	return true;
}

I2C_TransferReturn_TypeDef single_write(sl_i2cspm_t *i2c, uint8_t slave_id, uint8_t addr,
                  uint8_t data)
{
	uint8_t tranfer_data[] = {addr, data};
	I2C_TransferSeq_TypeDef seq = {slave_id, I2C_FLAG_WRITE,
	        { {tranfer_data, 2}}};

	return I2CSPM_Transfer(i2c, &seq);

}

I2C_TransferReturn_TypeDef burst_write(sl_i2cspm_t *i2c, uint8_t slave_id, uint8_t addr,
                 uint8_t *data, unsigned int num_bytes)
{
	uint8_t transfer_data[num_bytes + 1];
	memcpy(transfer_data, &addr, 1);
	memcpy(transfer_data + sizeof(addr), data, num_bytes);
	I2C_TransferSeq_TypeDef seq = {slave_id, I2C_FLAG_WRITE, { {transfer_data,
	        num_bytes + 1}}};

	return I2CSPM_Transfer(i2c, &seq);
}

I2C_TransferReturn_TypeDef single_read(sl_i2cspm_t *i2c, uint8_t slave_id, uint8_t addr, uint8_t buffer)
{
	I2C_TransferReturn_TypeDef status;
	I2C_TransferSeq_TypeDef addr_seq = {slave_id, I2C_FLAG_WRITE, { {&addr, 1}}};
	I2C_TransferSeq_TypeDef data_seq = {slave_id, I2C_FLAG_READ, { {&buffer, 1}}};

	if ((status = I2CSPM_Transfer(i2c, &addr_seq)) != i2cTransferDone)
	{
		printf("Transfer error: %d\n", status);
		return status;
	}

	return I2CSPM_Transfer(i2c, &data_seq);
}

I2C_TransferReturn_TypeDef burst_read(sl_i2cspm_t *i2c, uint8_t slave_id, uint8_t addr, uint8_t* buffer,
                    unsigned int num_bytes)
{
	I2C_TransferReturn_TypeDef status;
	I2C_TransferSeq_TypeDef addr_seq = {slave_id, I2C_FLAG_WRITE, { {&addr, 1}}};
	I2C_TransferSeq_TypeDef data_seq = {slave_id, I2C_FLAG_READ, { {buffer,
	        num_bytes}}};

	if (Transfer(i2c, &addr_seq) != i2cTransferDone) return status;

	return I2CSPM_Transfer(i2c, &data_seq);
}

bool read_ram(sl_i2cspm_t *i2c, uint8_t slave_id, uint8_t addr, uint8_t buffer)
{
	uint8_t ram_addr = (addr & 0x1F) | PARAM_QUERY;
	if ((send_command(i2c, slave_id, ram_addr) & 0xF0).valid != true) return false;
	return (single_read(i2c, slave_id, PARAM_RD, buffer) == i2cTransferDone);
}

bool write_ram(sl_i2cspm_t *i2c, uint8_t slave_id, uint8_t addr, uint8_t data)
{
	I2C_TransferSeq_TypeDef status;
	/* Write data to PARAM_WR */
	if ((single_write(i2c, slave_id, PARAM_WR, data)) != i2cTransferDone) return false;

	uint8_t ram_addr = (addr & 0x1F) | PARAM_SET;

	/* Write PARAM_WR to ram */
	if ((send_command(i2c, slave_id, ram_addr) & 0xF0).valid != true) return false;
	return true;
}

Response_t send_command(sl_i2cspm_t *i2c, uint8_t slave_id, uint8_t command)
{
	uint8_t  i = 100;
	Response_t response = {false, I2C_TRANSFER_ERROR};

	/* clear response register */
	if (single_write(i2c, slave_id, COMMAND, NOP) != i2cTransferDone) return response;
	if (single_read(i2c, slave_id, RESPONSE, response.e) != i2cTransferDone) return response;

	/* send command to command register until response is detected */
	while (response.e == NOERROR && i-- > 0)
	{
		if (single_write(i2c, slave_id, COMMAND, command) != i2cTransferDone) return response;
		if (single_read(i2c, slave_id, RESPONSE, response.e) != i2cTranferDone) return response;
	}

	return {valid, response.e};
}

uint16_t read_word_aux(sl_i2cspm_t *i2c, uint8_t slave_id, uint16_t buffer)
{
	uint8_t data[2];

	/* send conversion command to i2c */
	if ((send_command(i2c, slave_id, ALS_FORCE) & 0xF0) != 0) return 0xFFFF;

	/* allow time for conversion */
	sl_udelay_wait(CONVERSION_TIME);

	/* read word into buffer */
	if (!burst_read(i2c, slave_id, UVINDEX0, data, 2)) return 0xFFFF;

	return data[0] | (data[1] << 8);
}
