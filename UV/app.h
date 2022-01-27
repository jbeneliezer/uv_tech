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

#ifndef APP_H
#define APP_H
#include "em_gpio.h"
#include "em_i2c.h"
#define MAX_SENSORS 4

typedef struct Sensor{
	bool active;
	GPIO_Port_TypeDef power_port;
	uint8_t power_pin;
	uint8_t addr;
}Sensor;
extern Sensor sensors[MAX_SENSORS];
extern uint16_t UV_data[MAX_SENSORS];

/***************************************************************************//**
 * Initialize application.
 ******************************************************************************/
void app_init(void);
void i2c_init(void);

/***************************************************************************//**
 * App ticking function.
 ******************************************************************************/
void app_process_action(void);

#endif  // APP_H
