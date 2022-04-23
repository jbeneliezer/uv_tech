/*
 * gpio.c
 *
 *  Created on: Feb 14, 2022
 *      Author: Tennyson Cheng
 */

#include "sl_bluetooth.h"
#include "em_gpio.h"
#include "int_enum.h"
#include "timer.h"
#include "gpio.h"

static uint8_t button_state;

void gpio_init() {
	button_state = GPIO_PinInGet(GPIO_BUTTON_PORT, 	//store initial button state
								 GPIO_BUTTON_PIN);
	GPIO_ExtIntConfig(GPIO_BUTTON_PORT, 			//falling and rising edge interrupts enabled
					  GPIO_BUTTON_PIN,
					  GPIO_BUTTON_PIN,
					  true,
					  true,
					  true);
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);					//enable even pin interrupts
}

inline void gpio_led_off() {
	GPIO_PinOutClear(GPIO_LED_PORT, GPIO_LED_PIN);
}

inline void gpio_led_on() {
	GPIO_PinOutSet(GPIO_LED_PORT, GPIO_LED_PIN);
}

//NOTE: This IRQ routine assumes that the button state had switched.
//If button was pressed, then in order to enter this IRQ, the button had to have been released.
//If button was released, then in order to enter this IRQ, the button had to have been pressed.
//The implemented debounce state logic will account for bounces.
void GPIO_EVEN_IRQHandler(void)
{
	uint32_t int_pins = GPIO_IntGet();				//clear all pin interrupt flags
	GPIO_IntClear(int_pins);

	if (int_pins & (1 << GPIO_BUTTON_PIN)) {			//check if pin interrupt was from button
		if (button_state == RELEASED) {						//if button was not pressed, debounce first
			NVIC_DisableIRQ(GPIO_ODD_IRQn);						//disable odd pin interrupts
			button_state = PRESSED;								//update button state to PRESSED
			timer_button_debounce_start(&button_state);			//button debounce using sleeptimer
		}
		else if (button_state == DEBOUNCED){				//if button had been debounced using timer
			//NOTE: This conditional is only entered
			//if the button was not held long enough
			//to be considered HELD
			timer_button_hold_stop();							//stop the hold detect timer
			button_state = RELEASED;							//update button state to RELEASED
			sl_bt_external_signal(INT_BUTTON_PRESS);			//signal button press
		}
		else {												//if button was PRESSED or HELD
			button_state = RELEASED;							//update button state to RELEASED
		}
	}
}
