/***************************************************************************//**
 * @file
 * @brief Top level application functions
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/
#include "app.h"
#include "em_gpio.h"
#include "em_i2c.h"
#include "sl_udelay.h"
#include "i2c.h"
#include "si1132.h"

/***************************************************************************//**
 * Initialize application.
 ******************************************************************************/
Sensor sensors[MAX_SENSORS] = {{false, gpioPortA, 0, DEFAULT_ADDR},
							   {false, gpioPortA, 4, DEFAULT_ADDR},
							   {false, gpioPortA, 5, DEFAULT_ADDR},
							   {false, gpioPortA, 6, DEFAULT_ADDR}};
uint16_t UV_data[MAX_SENSORS] = {0, 0, 0, 0};

void app_init(void)
{
	//INITIALIZE I2C
	i2c_init();

	//INITIALIZE SENSORS
	for (uint8_t i = 0; i < MAX_SENSORS; i++) {
		if (si1132_init(sensors[i].power_port, sensors[i].power_pin, I2C0, DEFAULT_ADDR+i+1)) {
			sensors[i].addr = DEFAULT_ADDR+i+1;
			sensors[i].active = true;
			UV_data[i] = 0;
		}
		else {
			GPIO_PinOutClear(sensors[i].power_port, sensors[i].power_pin);
			UV_data[i] = -1;
		}
	}
}

/***************************************************************************//**
 * App ticking function.
 ******************************************************************************/
void app_process_action(void)
{
	GPIO_PinOutToggle(gpioPortC, 0);
	sl_udelay_wait(500000);
	GPIO_PinOutToggle(gpioPortC, 0);
	sl_udelay_wait(500000);
	for (uint8_t i = 0; i < MAX_SENSORS; i++) {
		if (sensors[i].active) {
			si1132_UV_start_measurement(I2C0, sensors[i].addr);
			si1132_UV_read_measurement(I2C0, sensors[i].addr, &UV_data[i]);
		}
	}
}
