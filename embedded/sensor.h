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

#ifndef SENSOR_H_
#define SENSOR_H_

#include "sl_i2cspm.h"
#include "gpiointerrupt.h"
#include "efr32bg22_i2c.h"

#define INT gpioPortA

/* Clock limits */
#define MAX_FREQUENCY 34000U // max clock frequency
#define MIN_FREQUENCY 95U    // min clock frequency

/* More timing macros */
#define STARTUP_TIME 25000U  // startup time in ms
#define RATE 100U            // measurement interval in us, must be at least twice the conversion rate.
                             // for right now I'm setting it at 100 us, typical conversion rate is 285 us.
/* Bit masks for Si1132 */
#define EN_UV (1<<7)
#define VIS_RANGE (1<<5)
#define IR_RANGE (1<<5)

/* Interrupt masks */
#define INT_OE (1<<0)
#define ALS_IE (1<<0)

/* sensor addresses */
typedef enum {
  PART_ID = 0x00,
  REV_ID = 0x01,
  SEQ_ID = 0x02,
  INT_CFG = 0x03,
  IRQ_ENABLE = 0x04,
  HW_KEY = 0x07,
  MEAS_RATE0 = 0x08,
  MEAS_RATE1 = 0x09,
  UCOEF0 = 0x13,
  UCOEF1 = 0x14,
  UCOEF2 = 0x15,
  UCOEF3 = 0x16,
  PARAM_WR = 0x17,
  COMMAND = 0x18,
  RESPONSE = 0x20,
  IRQ_STATUS = 0x21,
  ALS_VIS_DATA0 = 0x22,
  ALS_VIS_DATA1 = 0x23,
  ALS_IR_DATA0 = 0x24,
  ALS_IR_DATA1 = 0x25,
  AUX_DATA0 = 0x2C,
  UVINDEX0 = 0x2C,
  AUX_DATA1 = 0x2D,
  UVINDEX1 = 0x2D,
  PARAM_RD = 0x2E,
  CHIP_STAT = 0x30,
  ANA_IN_KEYHH = 0x3B,
  ANA_IN_KEYH = 0x3C,
  ANA_IN_KEYL = 0x3D,
  ANA_IN_KEYLL = 0x3E
} Si1132_addr_t ;

/* parameter ram addresses */
typedef enum {
  I2C_ADDR = 0x00,
  CHLIST = 0x01,
  ALS_ENCODING = 0x06,
  ALS_IR_ADCMUX = 0x0E,
  AUX_ADCMUX = 0x0F,
  ALS_VIS_ADC_COUNTER = 0x10,
  ALS_VIS_ADC_GAIN = 0x11,
  ALS_VIS_ADC_MISC = 0x12,
  ALS_IR_ADC_COUNTER = 0x1D,
  ALS_IR_ADC_GAIN = 0x1E,
  ALS_IR_ADC_MISC = 0x1F
} Ram_t;

typedef struct {
  enum {
    PARAM_QUERY = 0b100,
    PARAM_SET = 0b101,
    OTHER = 0b000
  } upper;
  enum {
    NOP = 0b00000,
    RESET = 0b00001,
    BUSADDR = 0b00010,
    GET_CAL = 0b10010,
    ALS_FORCE = 0b00110,
    ALS_PAUSE = 0b01010,
    ALS_AUTO = 0b01110
  } lower;
} Command_t;

typedef enum { FORCE, AUTO } Mode_t;

/* Response Errors */
#define NOERROR 0x0
#define INVALID_SETTINGS 0x80
#define ALS_VIS_ADC_OVERFLOW 0x8C
#define ALS_IR_ADC_OVERFLOW 0x8D
#define AUX_ADC_OVERFLOW = 0x8E

/* Use on all Si1132_addr enums */
typedef volatile uint8_t vu8;

/* Initialization*/
int sensor_init(sl_i2cspm_t*);

int setmode(sl_i2cspm_t*, Mode_t);

/* Command */
uint8_t send_command(sl_i2cspm_t*, Command_t*);

/* Polling operation */
double poll(sl_i2cspm_t*);

/* Interrupt driven operation */
void intread(uint8_t);
GPIOINT_IrqCallbackPtr_t irq_read = &intread;

#endif /* SENSOR_H_ */
