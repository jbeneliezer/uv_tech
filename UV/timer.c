/*
 * timer.c
 *
 *  Created on: Feb 14, 2022
 *      Author: Tennyson Cheng
 */

#include "sl_bluetooth.h"
#include "sl_sleeptimer.h"
#include "int_enum.h"
#include "timer.h"

static uint32_t timer_freq = 32768;
static sl_sleeptimer_timer_handle_t TIMER_SENSOR;
static sl_sleeptimer_timer_handle_t TIMER_BUTTON;

void timer_init_() {
	//STORE SLEEPTIMER CLOCK FREQUENCY FOR SOFT TIMERS
	timer_freq = sl_sleeptimer_get_timer_frequency();
}

sl_status_t timer_sensor_start() {
	return sl_sleeptimer_start_periodic_timer(&TIMER_SENSOR,
											timer_freq,
											timer_sensor_callback,
											NULL,
											TIMER_SENSOR_PRIORITY,
											SL_SLEEPTIMER_NO_HIGH_PRECISION_HF_CLOCKS_REQUIRED_FLAG);
}

sl_status_t timer_sensor_stop() {
	return sl_sleeptimer_stop_timer(&TIMER_SENSOR);
}

void timer_sensor_callback(sl_sleeptimer_timer_handle_t *handle, void *data) {
	(void)handle;
	(void)data;
	sl_bt_external_signal(INT_TIMER_SENSOR);
}
