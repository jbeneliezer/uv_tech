/*
 * timer.c
 *
 *  Created on: Feb 14, 2022
 *      Author: Tennyson Cheng
 */

#include "sl_bluetooth.h"
#include "sl_sleeptimer.h"
#include "int_enum.h"
#include "gpio.h"
#include "timer.h"

static uint32_t timer_freq = 32768;
static sl_sleeptimer_timer_handle_t TIMER_SENSOR;
static sl_sleeptimer_timer_handle_t TIMER_BUTTON;
static sl_sleeptimer_timer_handle_t TIMER_LED;
static uint8_t* button_state;

void timer_init_() {
	//STORE SLEEPTIMER CLOCK FREQUENCY FOR SOFT TIMERS
	timer_freq = sl_sleeptimer_get_timer_frequency();
}

sl_status_t timer_sensor_start() {
	return sl_sleeptimer_start_periodic_timer(&TIMER_SENSOR,
											timer_freq,					//1 second timer period
											timer_sensor_callback,
											NULL,
											TIMER_SENSOR_PRIORITY,
											SL_SLEEPTIMER_NO_HIGH_PRECISION_HF_CLOCKS_REQUIRED_FLAG);
}

sl_status_t timer_sensor_stop() {
	return sl_sleeptimer_stop_timer(&TIMER_SENSOR);
}

sl_status_t timer_button_debounce_start(uint8_t* button) {
	button_state = button;												//store pointer to button state
	return sl_sleeptimer_start_timer(&TIMER_BUTTON,
									 timer_freq >> 6,					//15.6 ms timer period
									 timer_button_debounce_callback,
									 NULL,
									 TIMER_BUTTON_PRIORITY,
									 SL_SLEEPTIMER_NO_HIGH_PRECISION_HF_CLOCKS_REQUIRED_FLAG);
}

sl_status_t timer_button_hold_stop() {
	return sl_sleeptimer_stop_timer(&TIMER_BUTTON);
}

sl_status_t timer_led_advertise_start() {
	sl_bt_external_signal(INT_LED_ON);
	return sl_sleeptimer_start_timer(&TIMER_LED,
									 timer_freq >> 4,					//(1/16) second timer period
									 timer_led_on_callback,
									 NULL,
									 TIMER_LED_PRIORITY,
									 SL_SLEEPTIMER_NO_HIGH_PRECISION_HF_CLOCKS_REQUIRED_FLAG);
}

sl_status_t timer_led_advertise_stop() {
	sl_bt_external_signal(INT_LED_OFF);
	return sl_sleeptimer_stop_timer(&TIMER_LED);
}

void timer_sensor_callback(sl_sleeptimer_timer_handle_t *handle, void *data) {
	(void)handle;
	(void)data;
	sl_bt_external_signal(INT_TIMER_SENSOR);
}

void timer_button_debounce_callback(sl_sleeptimer_timer_handle_t *handle, void *data) {
	(void)handle;
	(void)data;
	*button_state = (uint8_t)GPIO_PinInGet(GPIO_BUTTON_PORT,	//get the current state of the button
										   GPIO_BUTTON_PIN);	//can only be either (PRESSED or RELEASED)
	if (*button_state == PRESSED) {								//if the button is still being pressed
		*button_state = DEBOUNCED;									//update button state to DEBOUNCED
		sl_sleeptimer_start_timer(&TIMER_BUTTON,					//start timer to detect for button hold
								  timer_freq << 2,					//4 second timer period
								  timer_button_hold_callback,
								  NULL,
								  TIMER_BUTTON_PRIORITY,
								  SL_SLEEPTIMER_NO_HIGH_PRECISION_HF_CLOCKS_REQUIRED_FLAG);
	}
	uint32_t int_pins = GPIO_IntGet();							//clear all interrupt flags
	GPIO_IntClear(int_pins);
	NVIC_EnableIRQ(GPIO_ODD_IRQn);								//re-enable odd pin interrupts
}

void timer_button_hold_callback(sl_sleeptimer_timer_handle_t *handle, void *data) {
	(void)handle;
	(void)data;
	if (*button_state == DEBOUNCED) {							//if the button had been debounced
		//Note: It shouldn't be possible to enter this callback
		//with a button_state other than DEBOUNCED
		*button_state = HELD;									//update the button state to HELD
		sl_bt_external_signal(INT_BUTTON_HOLD);					//signal button hold
	}
}

void timer_led_on_callback(sl_sleeptimer_timer_handle_t *handle, void *data) {
	(void)handle;
	(void)data;
	sl_bt_external_signal(INT_LED_OFF);
	sl_sleeptimer_start_timer(&TIMER_LED,
							  (timer_freq >> 4) +						//(1/16 +
							  (timer_freq >> 3),						//(1/8) second timer period
							  timer_led_off_callback,
							  NULL,
							  TIMER_LED_PRIORITY,
							  SL_SLEEPTIMER_NO_HIGH_PRECISION_HF_CLOCKS_REQUIRED_FLAG);
}

void timer_led_off_callback(sl_sleeptimer_timer_handle_t *handle, void *data) {
	(void)handle;
	(void)data;
	sl_bt_external_signal(INT_LED_ON);
	sl_sleeptimer_start_timer(&TIMER_LED,
							  timer_freq >> 4,							//(1/16) second timer period
							  timer_led_on_callback,
							  NULL,
							  TIMER_LED_PRIORITY,
							  SL_SLEEPTIMER_NO_HIGH_PRECISION_HF_CLOCKS_REQUIRED_FLAG);
}
