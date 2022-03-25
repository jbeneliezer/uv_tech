/*
 * i2c.c
 *
 *  Created on: Jan 26, 2022
 *      Author: Judah Ben-Eliezer, Tennyson Cheng
 */

#include "em_gpio.h"
#include "em_i2c.h"
#include "em_cmu.h"
#include "i2c.h"

void i2c_init(void) {
	//Enable I2C Clock
	CMU_ClockEnable(cmuClock_I2C0, true);

	//Initialize I2C configuration
	I2C_Init_TypeDef i2cInit = I2C_INIT_DEFAULT;		//base initial config from the default, enabled in master mode
	//i2cInit.freq = I2C_FREQ_FAST_MAX;
	//i2cInit.clhr = i2cClockHLRAsymetric;

	//Both I2C pins configured for open-drain pull up operation
	GPIO_PinModeSet(gpioPortB, 0, gpioModeWiredAndPullUpFilter, 1);		//PB0 = SDA
	GPIO_PinModeSet(gpioPortB, 1, gpioModeWiredAndPullUpFilter, 1);		//PB1 = SCLK

	// Route I2C pins to GPIO
	GPIO->I2CROUTE[0].SDAROUTE = (GPIO->I2CROUTE[0].SDAROUTE & ~_GPIO_I2C_SDAROUTE_MASK)
								  | (gpioPortB << _GPIO_I2C_SDAROUTE_PORT_SHIFT
								  | (0 << _GPIO_I2C_SDAROUTE_PIN_SHIFT));
	GPIO->I2CROUTE[0].SCLROUTE = (GPIO->I2CROUTE[0].SCLROUTE & ~_GPIO_I2C_SCLROUTE_MASK)
								  | (gpioPortB << _GPIO_I2C_SCLROUTE_PORT_SHIFT
								  | (1 << _GPIO_I2C_SCLROUTE_PIN_SHIFT));
	GPIO->I2CROUTE[0].ROUTEEN = GPIO_I2C_ROUTEEN_SDAPEN | GPIO_I2C_ROUTEEN_SCLPEN;

	//Complete I2C initialization
	I2C_Init(I2C0, &i2cInit);							//initialize i2c peripheral
}

I2C_TransferReturn_TypeDef i2c_write_single(I2C_TypeDef *i2c, uint8_t slave_addr, uint8_t reg_addr, uint8_t data)
{
	//Initialize I2C transfer
	I2C_TransferSeq_TypeDef i2cTransfer;
	i2cTransfer.addr = (slave_addr << 1);
	i2cTransfer.flags = I2C_FLAG_WRITE_WRITE;
	i2cTransfer.buf[0].data = &reg_addr;
	i2cTransfer.buf[0].len = 1;
	i2cTransfer.buf[1].data = &data;
	i2cTransfer.buf[1].len = 1;

	//Initialize I2C transfer return status
	I2C_TransferReturn_TypeDef status = I2C_TransferInit(i2c, &i2cTransfer);

	//Start I2C transfer
	while (status == i2cTransferInProgress) {		//loop until i2c transfer is finished
		status = I2C_Transfer(i2c);
	}

	return status;
}

I2C_TransferReturn_TypeDef i2c_write_burst(I2C_TypeDef *i2c, uint8_t slave_addr, uint8_t reg_addr, uint8_t *data, uint16_t num_bytes)
{
	//Initialize I2C transfer
	I2C_TransferSeq_TypeDef i2cTransfer;
	i2cTransfer.addr = (slave_addr << 1);
	i2cTransfer.flags = I2C_FLAG_WRITE_WRITE;
	i2cTransfer.buf[0].data = &reg_addr;
	i2cTransfer.buf[0].len = 1;
	i2cTransfer.buf[1].data = data;
	i2cTransfer.buf[1].len = num_bytes;

	//Initialize I2C transfer return status
	I2C_TransferReturn_TypeDef status = I2C_TransferInit(i2c, &i2cTransfer);

	//Start I2C transfer
	while (status == i2cTransferInProgress) {		//loop until i2c transfer is finished
		status = I2C_Transfer(i2c);
	}

	return status;
}

I2C_TransferReturn_TypeDef i2c_read_single(I2C_TypeDef *i2c, uint8_t slave_addr, uint8_t reg_addr, uint8_t *buffer)
{
	//Initialize I2C transfer
	I2C_TransferSeq_TypeDef i2cTransfer;
	i2cTransfer.addr = (slave_addr << 1);
	i2cTransfer.flags = I2C_FLAG_WRITE_READ;
	i2cTransfer.buf[0].data = &reg_addr;
	i2cTransfer.buf[0].len = 1;
	i2cTransfer.buf[1].data = buffer;
	i2cTransfer.buf[1].len = 1;

	//Initialize I2C transfer return status
	I2C_TransferReturn_TypeDef status = I2C_TransferInit(i2c, &i2cTransfer);

	//Start I2C transfer
	while (status == i2cTransferInProgress) {		//loop until i2c transfer is finished
		status = I2C_Transfer(i2c);
	}

	return status;
}

I2C_TransferReturn_TypeDef i2c_read_burst(I2C_TypeDef *i2c, uint8_t slave_addr, uint8_t reg_addr, uint8_t* buffer, uint16_t num_bytes)
{
	//Initialize I2C transfer
	I2C_TransferSeq_TypeDef i2cTransfer;
	i2cTransfer.addr = (slave_addr << 1);
	i2cTransfer.flags = I2C_FLAG_WRITE_READ;
	i2cTransfer.buf[0].data = &reg_addr;
	i2cTransfer.buf[0].len = 1;
	i2cTransfer.buf[1].data = buffer;
	i2cTransfer.buf[1].len = num_bytes;

	//Initialize I2C transfer return status
	I2C_TransferReturn_TypeDef status = I2C_TransferInit(i2c, &i2cTransfer);

	//Start I2C transfer
	while (status == i2cTransferInProgress) {		//loop until i2c transfer is finished
		status = I2C_Transfer(i2c);
	}

	return status;
}
