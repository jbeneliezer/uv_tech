/*
 * gpio.c
 *
 *  Created on: Feb 14, 2022
 *      Author: Tennyson Cheng
 */

#include "em_gpio.h"
#include "int_enum.h"
#include "gpio.h"

void gpio_init() {
	GPIO_ExtIntConfig(GPIO_BUTTON_PORT, 			//falling and rising edge interrupts enabled
					  GPIO_BUTTON_PIN,
					  GPIO_BUTTON_PIN,
					  true,
					  true,
					  true);
	NVIC_EnableIRQ(GPIO_ODD_IRQn);					//enable odd pin interrupts
}

void GPIO_ODD_IRQHandler(void)
{
	uint32_t int_pins = GPIO_IntGet();
	GPIO_IntClear(int_pins);

	if (int_pins & (1 << GPIO_BUTTON_PIN)) {
		sl_bt_external_signal(INT_BUTTON);
	}
}
