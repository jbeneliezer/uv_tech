/*
 * gpio.h
 *
 *  Created on: Feb 14, 2022
 *      Author: Tennyson Cheng
 */

#ifndef GPIO_H_
#define GPIO_H_

#include "em_gpio.h"

#define GPIO_BUTTON_PORT gpioPortC
#define GPIO_BUTTON_PIN 1
#define GPIO_LED_PORT gpioPortC
#define GPIO_LED_PIN 0

typedef enum BUTTON_STATE {
	PRESSED = 0,
	RELEASED = 1,
	HELD,					//physical state is the same as PRESSED, but logically different
	DEBOUNCED				//physical state is the same as PRESSED, but logically different
}BUTTON_STATE;

void gpio_init();
void gpio_led_off();
void gpio_led_on();

#endif /* GPIO_H_ */
