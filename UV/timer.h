/*
 * timer.h
 *
 *  Created on: Feb 14, 2022
 *      Author: Tennyson Cheng
 */

#ifndef TIMER_H_
#define TIMER_H_

#include "sl_sleeptimer.h"

#define TIMER_SENSOR_PRIORITY 0
#define TIMER_BUTTON_PRIORITY 1
#define TIMER_LED_PRIORITY 2

void timer_init_();
sl_status_t timer_sensor_start();
sl_status_t timer_sensor_stop();
sl_status_t timer_button_debounce_start(uint8_t* button);
sl_status_t timer_button_hold_stop();
sl_status_t timer_led_advertise_start();
sl_status_t timer_led_advertise_stop();
void timer_sensor_callback(sl_sleeptimer_timer_handle_t *handle, void *data);
void timer_button_debounce_callback(sl_sleeptimer_timer_handle_t *handle, void *data);
void timer_button_hold_callback(sl_sleeptimer_timer_handle_t *handle, void *data);
void timer_sensor_callback(sl_sleeptimer_timer_handle_t *handle, void *data);
void timer_led_on_callback(sl_sleeptimer_timer_handle_t *handle, void *data);
void timer_led_off_callback(sl_sleeptimer_timer_handle_t *handle, void *data);

#endif /* TIMER_H_ */
