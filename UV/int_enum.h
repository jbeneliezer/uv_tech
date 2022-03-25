/*
 * int_enum.h
 *
 *  Created on: Feb 14, 2022
 *      Author: Tennyson Cheng
 */

#ifndef INT_ENUM_H_
#define INT_ENUM_H_

typedef enum APP_INT {
	INT_TIMER_SENSOR = 1,
	INT_BUTTON_PRESS,
	INT_BUTTON_HOLD,
	INT_LED_OFF,
	INT_LED_ON
}APP_INT;

#endif /* INT_ENUM_H_ */
