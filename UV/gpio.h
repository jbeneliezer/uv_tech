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

typedef enum BUTTON_STATE {
	PRESSED = 0,
	RELEASED = 1,
	HELD,					//physical state is the same as PRESSED, but logically different
	DEBOUNCED				//physical state is the same as PRESSED, but logically different
}BUTTON_STATE;

void gpio_init();

#endif /* GPIO_H_ */
