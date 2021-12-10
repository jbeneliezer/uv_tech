/*
 * si1132.h
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

/* Modes */
typedef enum
{
	FORCED, AUTO
} Mode_t;

/* Clock limits */
#define MAX_FREQUENCY 34000U 	// max clock frequency
#define MIN_FREQUENCY 95U    	// min clock frequency

/* More timing macros */
#define STARTUP_TIME 25000U  	// startup time (ms)
#define CONVERSION_TIME 300U	// time needed for data generation (us)

#define DEFAULT_ADDR 0x60		// default slave if

/* Bit masks for CHLIST */
#define EN_UV (1<<7)
#define EN_AUX (1<<6)
#define EN_ALS_IR (1<<5)
#define EN_ALS_VIS (1<<4)

/* ADC masks */
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
#define	NOERROR 0x00
#define	INVALID_SETTINGS 0x80
#define	ALS_VIS_ADC_OVERFLOW 0x8C
#define	ALS_IR_ADC_OVERFLOW 0x8D
#define	AUX_ADC_OVERFLOW 0x8E
#define	I2C_TRANSFER_ERROR 0x8F

/***************************************************************************//**
 * @brief
 *   Initializes si1132 slave for UV detection.
 *
 * @param[in] power_port
 * 	 GPIO output port for slave VDD.
 *
 * @param[in] power_pin
 *   GPIO output pin for slave VDD.
 *
 * @param[in] i2c
 *   Pointer to I2C peripheral register block.
 *
 * @param[in] slave_id.
 *   Address to assign to slave.
 ******************************************************************************/
bool si1132_init(GPIO_Port_TypeDef, uint8_t, sl_i2cspm_t*, uint8_t);

/***************************************************************************//**
 * @brief
 *   Writes 1 byte to a register of si1132 slave.
 *
 * @param[in] i2c
 *   Pointer to I2C peripheral register block.
 *
 * @param[in] slave_id.
 *   Address of slave.
 *
 * @param[in] addr
 *   Internal register address to write to.
 *
 * @param[in] data
 *   Byte to write.
 ******************************************************************************/
I2C_TransferReturn_TypeDef single_write(sl_i2cspm_t*, uint8_t, uint8_t, uint8_t);

/***************************************************************************//**
 * @brief
 *   Writes byte array to si1132 slave.
 *
 * @param[in] i2c
 *   Pointer to I2C peripheral register block.
 *
 * @param[in] slave_id.
 *   Address of slave.
 *
 * @param[in] addr
 *   Internal register address to begin write.
 *
 * @param[in] data
 *   Pointer to array of data to write.
 *
 * @param[in] num_bytes
 *   Number of bytes to write.
 ******************************************************************************/
I2C_TransferReturn_TypeDef burst_write(sl_i2cspm_t*, uint8_t, uint8_t, uint8_t*, unsigned int);

/***************************************************************************//**
 * @brief
 *   Reads 1 byte from a register of si1132 slave.
 *
 * @param[in] i2c
 *   Pointer to I2C peripheral register block.
 *
 * @param[in] slave_id.
 *   Address of slave.
 *
 * @param[in] addr
 *   Internal register address to read from.
 *
 * @param[in] buffer
 *   Buffer to read byte into.
 ******************************************************************************/
I2C_TransferReturn_TypeDef single_read(sl_i2cspm_t*, uint8_t, uint8_t, uint8_t);

/***************************************************************************//**
 * @brief
 *   Reads byte array from si1132 slave.
 *
 * @param[in] i2c
 *   Pointer to I2C peripheral register block.
 *
 * @param[in] slave_id.
 *   Address of slave.
 *
 * @param[in] addr
 *   Internal register address to begin read.
 *
 * @param[in] buffer
 *   Buffer to store bytes in.
 *
 * @param[in] num_bytes
 *   Number of bytes to read.
 ******************************************************************************/
I2C_TransferReturn_TypeDef burst_read(sl_i2cspm_t*, uint8_t, uint8_t, uint8_t *buffer, unsigned int);

/***************************************************************************//**
 * @brief
 *   Sends command to si1132 slave.
 *
 * @param[in] i2c
 *   Pointer to I2C peripheral register block.
 *
 * @param[in] slave_id.
 *   Address of slave.
 *
 * @param[in] command
 *   Command to send.
 ******************************************************************************/
uint8_t send_command(sl_i2cspm_t*, uint8_t, uint8_t);

/***************************************************************************//**
 * @brief
 *   Writes 1 byte to ram of si1132 slave.
 *
 * @param[in] i2c
 *   Pointer to I2C peripheral register block.
 *
 * @param[in] slave_id.
 *   Address of slave.
 *
 * @param[in] addr
 *   Ram address to write to.
 *
 * @param[in] data
 *   Byte to write.
 ******************************************************************************/
bool write_ram(sl_i2cspm_t*, uint8_t, uint8_t, uint8_t);

/***************************************************************************//**
 * @brief
 *   Reads 1 byte from ram of si1132 slave.
 *
 * @param[in] i2c
 *   Pointer to I2C peripheral register block.
 *
 * @param[in] slave_id.
 *   Address of slave.
 *
 * @param[in] addr
 *   Ram address to read from.
 *
 * @param[in] buffer
 *   Buffer to read data into.
 ******************************************************************************/
bool read_ram(sl_i2cspm_t*, uint8_t, uint8_t, uint8_t);

/***************************************************************************//**
 * @brief
 *   Reads word from UVINDEX1|UVINDEX0.
 *
 * @param[in] i2c
 *   Pointer to I2C peripheral register block.
 *
 * @param[in] slave_id.
 *   Address of slave.
 *
 * @param[in] buffer.
 *   Buffer to read word into.
 ******************************************************************************/
uint8_t read_word_aux(sl_i2cspm_t*, uint8_t, uint16_t);

#endif /* SI1132_H_ */
