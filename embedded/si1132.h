/*
 * sensor.h
 *
 *  Created on: Oct 13, 2021
 *      Author: Judah Ben-Eliezer
 *
 *  This header file contains the declarations for the macros, types, and functions
 *  needed to initialize and interface with the Si1132 uv sensors.
 *
 */

#ifndef SI1132_H_
#define SI1132_H_

#include "sl_i2cspm.h"
#include "gpiointerrupt.h"
#include "efr32bg22_i2c.h"

/* Modes */
typedef enum
{
	FORCE, AUTO
} Mode_t;

/* Int port and pin */
#define INT gpioPortA
#define INT_PIN 2

/* Clock limits */
#define MAX_FREQUENCY 34000U // max clock frequency
#define MIN_FREQUENCY 95U    // min clock frequency

/* More timing macros */
#define STARTUP_TIME 25000U  // startup time in ms

/* Bit masks for CHLIST */
#define EN_UV (1<<7)
#define EN_AUX (1<<6)
#define EN_ALS_IR (1<<5)
#define EN_ALS_VIS (1<<4)

#define VIS_RANGE (1<<5)
#define IR_RANGE (1<<5)

/* Interrupt Masks */
#define INT_OE (1<<0)
#define ALS_IE (1<<0)

/* Si1132 Addresses */
#define	PART_ID 0x00
#define	REV_ID 0x01
#define	SEQ_ID 0x02
#define	INT_CFG 0x03
#define	IRQ_ENABLE 0x04
#define	HW_KEY 0x07
#define	MEAS_RATE0 0x08
#define	MEAS_RATE1 0x09
#define	UCOEF0 0x13
#define	UCOEF1 0x14
#define	UCOEF2 0x15
#define	UCOEF3 0x16
#define	PARAM_WR 0x17
#define	COMMAND 0x18
#define	RESPONSE 0x20
#define	IRQ_STATUS 0x21
#define	ALS_VIS_DATA0 0x22
#define	ALS_VIS_DATA1 0x23
#define	ALS_IR_DATA0 0x24
#define	ALS_IR_DATA1 0x25
#define	AUX_DATA0 0x2C
#define	UVINDEX0 0x2C
#define	AUX_DATA1 0x2D
#define	UVINDEX1 0x2D
#define	PARAM_RD 0x2E
#define	CHIP_STAT 0x30
#define	ANA_IN_KEYHH 0x3B
#define	ANA_IN_KEYH 0x3C
#define	ANA_IN_KEYL 0x3D
#define	ANA_IN_KEYLL 0x3E

/* Parameter Ram Addresses */
#define	I2C_ADDR 0x00
#define	CHLIST 0x01
#define	ALS_ENCODING 0x06
#define	ALS_IR_ADCMUX 0x0E
#define	AUX_ADCMUX 0x0F
#define	ALS_VIS_ADC_COUNTER 0x10
#define	ALS_VIS_ADC_GAIN 0x11
#define	ALS_VIS_ADC_MISC 0x12
#define	ALS_IR_ADC_COUNTER 0x1D
#define	ALS_IR_ADC_GAIN 0x1E
#define	ALS_IR_ADC_MISC 0x1F

/* Command Register Codes */
#define PARAM_QUERY 0b10000000
#define PARAM_SET 0b10100000
#define	NOP 0b00000
#define	RESET 0b00000001
#define	BUSADDR 0b00000010
#define	GET_CAL 0b00010010
#define	ALS_FORCE 0b00000110
#define	ALS_PAUSE 0b00001010
#define	ALS_AUTO 0b00001110

/* Response Errors */
#define NOERROR 0x0
#define INVALID_SETTINGS 0x80
#define ALS_VIS_ADC_OVERFLOW 0x8C
#define ALS_IR_ADC_OVERFLOW 0x8D
#define AUX_ADC_OVERFLOW = 0x8E

/* Initialization*/
int sensor_init(sl_i2cspm_t*, uint8_t, Mode_t);

/* Set either auto or forced for interrupt driven or polled operation*/
int setmode(sl_i2cspm_t*, Mode_t);

/* Command */
uint8_t send_command(sl_i2cspm_t*, uint8_t);

/* Read byte from  register */
uint8_t read_byte(sl_i2cspm_t*, uint8_t);

/* Write byte to register */
uint8_t write_byte(sl_i2cspm_t*, uint8_t, uint8_t*);

/* Read byte from ram */
uint8_t read_ram(sl_i2cspm_t*, uint8_t);

/* Write byte to ram */
uint8_t write_ram(sl_i2cspm_t*, uint8_t, uint8_t*);

/* Read word from AUX_DATA1|AUX_DATA0 */
uint16_t read_word_aux(sl_i2cspm_t*);

/* Interrupt driven operation */
//void intread(uint8_t);
//GPIOINT_IrqCallbackPtr_t irq_read = &intread;
#endif /* SI1132_H_ */
