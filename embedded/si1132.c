/*
 * sensor.c
 *
 *  Created on: Oct 13, 2021
 *      Author: Judah Ben-Eliezer
 *
 *  All code related to the Si1132 sensors i2c interface.
 *
 */

#include <stdio.h>
#include <string.h>
#include "sl_udelay.h"
#include "em_gpio.h"
#include "gpiointerrupt.h"
#include "sl_i2cspm_instances.h"
#include "si1132.h"

/* Initializes sensor */
int sensor_init(sl_i2cspm_t *sensor, uint8_t addr, Mode_t mode)
{
	/* enable interrupts */
#if (mode == FORCE)
	GPIOINT_Init();
#endif

	uint8_t init_code = 0x17;
	uint8_t chlist_init = EN_UV;
	uint8_t ucoef[4] = {0x7B, 0x6B, 0x01, 0x00};
	uint8_t vis_range = VIS_RANGE;
	uint8_t ir_range = IR_RANGE;

	sl_udelay_wait(STARTUP_TIME);   // allow time for startup

	/* write 0x17 to HW_KEY for proper operation */
	if(write_byte(sensor, HW_KEY, &init_code) == 0xFF) return -1;

	/* set device address */
	if(write_ram(sensor, I2C_ADDR, &addr) == 0xFF) return -1;
	if(write_ram(sensor, BUSADDR, &addr) == 0xFF) return -1;

	sensor->SADDR = addr;

	/* enable uv detection */
	if(write_ram(sensor, CHLIST, &chlist_init) == 0xFF) return -1;

	/* configure ucoef for direct sunlight */
	if(write_byte(sensor, UCOEF0, &ucoef[0]) == 0xFF) return -1;
	if(write_byte(sensor, UCOEF1, &ucoef[1]) == 0xFF) return -1;
	if(write_byte(sensor, UCOEF2, &ucoef[2]) == 0xFF) return -1;
	if(write_byte(sensor, UCOEF3, &ucoef[3]) == 0xFF) return -1;

	/* enable vis_range and ir_range */
	if(write_byte(sensor, ALS_VIS_ADC_MISC, &vis_range) == 0xFF) return -1;
	if(write_byte(sensor, ALS_IR_ADC_MISC, &ir_range) == 0xFF) return -1;

	/* set mode */
	return (setmode(sensor, mode));
}

/* Configuration for forced/ auto modes */
int setmode(sl_i2cspm_t *sensor, Mode_t m)
{
//	uint16_t rate = (m == AUTO) ? RATE / 31.25 : 0;
//	uint8_t meas[2];
//	memcpy (meas, &rate, 2); // calculate values for meas_h and meas_l: polling rate
//
//	if (write_byte(sensor, MEAS_RATE1, &meas[0]) == 0xFF) return -1;
//	if (write_byte(sensor, MEAS_RATE0, &meas[1]) == 0xFF) return -1;

	uint8_t c = (m == AUTO) ? ALS_AUTO : ALS_FORCE;
	send_command(sensor, c);

	/* set up interrupts */
//#if (m == FORCE)
//	uint8_t int_cfg = INT_OE, als_ie = ALS_IE;
//	/* Si1132 Interrupt Configuration */
//	if (write_byte(sensor, INT_CFG, &int_cfg) == 0xFF) return -1;
//	if (write_byte(sensor, IRQ_ENABLE, &als_ie) == 0xFF) return -1;
//
//	/* Configure INT pin of microcontroller */
//	GPIOINT_CallbackRegister (INT, irq_read);
//	GPIO_IntEnable (1 << 2);
//#endif
	return 0;
}

/* returns 1 byte from i2c register */
uint8_t read_byte(sl_i2cspm_t *sensor, uint8_t addr)
{
	int status;
	uint8_t ret;
	I2C_TransferSeq_TypeDef t = {addr, I2C_FLAG_READ, { {&ret, 1}}};

	if((status = I2CSPM_Transfer(sensor, &t)) != i2cTransferDone)
	{
		printf("Transfer error: %d\n", status);
		return 0xFF;
	}

	return ret;
}

/* writes 1 byte to i2c register */
uint8_t write_byte(sl_i2cspm_t *sensor, uint8_t addr, uint8_t *data)
{
	int status;
	I2C_TransferSeq_TypeDef t = {addr, I2C_FLAG_READ, { {data, 1}}};

	if((status = I2CSPM_Transfer(sensor, &t)) != i2cTransferDone)
	{
		printf("Transfer error: %d\n", status);
		return 0xFF;
	}

	return 0;
}

/* Reads 1 byte from ram */
uint8_t read_ram(sl_i2cspm_t *sensor, uint8_t addr)
{
	uint8_t ram_addr = (addr & 0x1F) | PARAM_QUERY;
	if((send_command(sensor, ram_addr) & 0x80) != 0) return 0xFF;
	return read_byte(sensor, PARAM_RD);
}

/* Writes 1 byte to ram */
uint8_t write_ram(sl_i2cspm_t *sensor, uint8_t addr, uint8_t *data)
{
	/* Write data to PARAM_WR */
	if(write_byte(sensor, PARAM_WR, data) == 0xFF) return 0xFF;

	uint8_t ram_addr = (addr & 0x1F) | PARAM_SET;

	/* Write PARAM_WR to ram */
	if((send_command(sensor, ram_addr) & 0x80) != 0) return 0xFF;
	return read_byte(sensor, PARAM_RD);

}

/* protocol for sending commands to sensor */
uint8_t send_command(sl_i2cspm_t *sensor, uint8_t command)
{
	uint8_t zero = 0x00, response, i = 100;

	/* clear response register */
	if(write_byte(sensor, COMMAND, NOP) == 0xFF) return 0xFF;
	response = read_byte(sensor, RESPONSE);

	if(response != 0x00) return response;

	/* send command to command register until response is detected */
	while(response == 0x00 && i-- > 0)
	{
		if(write_byte(sensor, COMMAND, &command) == 0xFF) return 0xFF;
		response = read_byte(sensor, RESPONSE);
		if(response == 0xFF) return response;
	}

	return response;
}

/* Reads word from UVINDEX1|UVINDEX0 */
uint16_t read_word_aux(sl_i2cspm_t *sensor)
{
	uint16_t ret;

	/* read high byte */
	if((ret = read_byte(sensor, UVINDEX1)) == 0xFF) return 0xFFFF;
	ret <<= 8;

	/* read low byte */
	if((ret |= read_byte(sensor, UVINDEX0)) == 0xFF) return 0xFFFF;

	return ret;
}
