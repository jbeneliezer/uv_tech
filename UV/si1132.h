/*
 * si1132.h
 *
 *  Created on: Oct 13, 2021
 *  Last Modified: Jan 26, 2022
 *      Author: Judah Ben-Eliezer, Tennyson Cheng
 *
 *  This header file contains the declarations for the macros, types, and functions
 *  needed to initialize and interface with the Si1132 UV sensors.
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
#define MAX_FREQUENCY 34000000U 	//max clock frequency (Hz)
#define MIN_FREQUENCY 95000U    	//min clock frequency (Hz)

/* More timing macros */
#define STARTUP_TIME 25000U  		//startup time (us)
#define CONVERSION_TIME 300U		//time needed for uv data to be ready (us)

#define DEFAULT_ADDR 0x60			//default slave i2c address

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

/* Si1132 Register Addresses */
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
 * @param[in] slave_addr.
 *   Address to assign to slave.
 ******************************************************************************/
bool si1132_init(GPIO_Port_TypeDef power_port, uint8_t power_pin, I2C_TypeDef *i2c, uint8_t slave_addr);

bool si1132_calibrate(I2C_TypeDef *i2c, uint8_t slave_addr);

/***************************************************************************//**
 * @brief
 *   Sends command to si1132 slave.
 *   Returns the RESPONSE.
 *
 * @param[in] i2c
 *   Pointer to I2C peripheral register block.
 *
 * @param[in] slave_addr.
 *   Address of slave.
 *
 * @param[in] command
 *   Command to send.
 ******************************************************************************/
uint8_t si1132_send_command(I2C_TypeDef *i2c, uint8_t slave_addr, uint8_t command);

/***************************************************************************//**
 * @brief
 *   Sends BUSADDR command to si1132 slave.
 *   Since the slave address is changed for the final RESPONSE read,
 *   this function is seperated from the normal si1132_send_command.
 *   Returns the RESPONSE.
 *
 * @param[in] i2c
 *   Pointer to I2C peripheral register block.
 *
 * @param[in] slave_addr.
 *   Address of slave.
 *
 * @param[in] slave_addr_new
 *   New address of the slave.
 ******************************************************************************/
uint8_t si1132_send_command_BUSADDR(I2C_TypeDef *i2c, uint8_t slave_addr, uint8_t slave_addr_new);

/***************************************************************************//**
 * @brief
 *   Writes 1 byte to ram of si1132 slave.
 *
 * @param[in] i2c
 *   Pointer to I2C peripheral register block.
 *
 * @param[in] slave_addr.
 *   Address of slave.
 *
 * @param[in] addr
 *   Ram address to write to.
 *
 * @param[in] data
 *   Byte to write.
 ******************************************************************************/
bool si1132_write_ram(I2C_TypeDef *i2c, uint8_t slave_addr, uint8_t addr, uint8_t data);

/***************************************************************************//**
 * @brief
 *   Reads 1 byte from ram of si1132 slave.
 *
 * @param[in] i2c
 *   Pointer to I2C peripheral register block.
 *
 * @param[in] slave_addr.
 *   Address of slave.
 *
 * @param[in] addr
 *   Ram address to read from.
 *
 * @param[in] buffer
 *   Buffer to read data into.
 ******************************************************************************/
bool si1132_read_ram(I2C_TypeDef *i2c, uint8_t slave_addr, uint8_t addr, uint8_t buffer);

/***************************************************************************//**
 * @brief
 *   Sends ALS_FORCE command to start UV measurement in forced conversion mode
 *
 * @param[in] i2c
 *   Pointer to I2C peripheral register block.
 *
 * @param[in] slave_addr.
 *   Address of slave.
 ******************************************************************************/
uint8_t si1132_UV_start_measurement(I2C_TypeDef *i2c, uint8_t slave_addr);

/***************************************************************************//**
 * @brief
 *   Reads word from UVINDEX1|UVINDEX0 and stores into buffer.
 *
 * @param[in] i2c
 *   Pointer to I2C peripheral register block.
 *
 * @param[in] slave_addr.
 *   Address of slave.
 *
 * @param[in] buffer.
 *   Buffer to read word into.
 ******************************************************************************/
uint8_t si1132_UV_read_measurement(I2C_TypeDef *i2c, uint8_t slave_addr, uint16_t *buffer);

/***************************************************************************//**
 * CODE BELOW HAS BEEN PROVIDED BY SILICON LABS
 * "si114x_function.c"
 *
 * Slightly modified for calibration of Si1132 chips only
 ******************************************************************************/
#define FLT_TO_FX20(x)       ((int32_t)((x*1048576)+.5))
#define FX20_ONE             FLT_TO_FX20( 1.000000)
#define FX20_BAD_VALUE       0xffffffff

#define SIRPD_ADCHI_IRLED    (collect(buffer, 0x23, 0x22,  0))
#define SIRPD_ADCLO_IRLED    (collect(buffer, 0x22, 0x25,  1))
#define SIRPD_ADCLO_WHLED    (collect(buffer, 0x24, 0x26,  0))
#define VISPD_ADCHI_WHLED    (collect(buffer, 0x26, 0x27,  1))
#define VISPD_ADCLO_WHLED    (collect(buffer, 0x28, 0x29,  0))
#define LIRPD_ADCHI_IRLED    (collect(buffer, 0x29, 0x2a,  1))
#define LED_DRV65            (collect(buffer, 0x2b, 0x2c,  0))

#define ALIGN_LEFT   1
#define ALIGN_RIGHT -1

struct operand_t
{
  uint32_t op1;
  uint32_t op2;
};
struct cal_ref_t
{
	uint32_t sirpd_adchi_irled;
	uint32_t sirpd_adclo_irled;
	uint32_t sirpd_adclo_whled;
	uint32_t vispd_adchi_whled;
	uint32_t vispd_adclo_whled;
	uint32_t lirpd_adchi_irled;
	uint32_t ledi_65ma;
	uint8_t  ucoef[4];
};

uint32_t decode(uint32_t input);
uint32_t collect(uint8_t* buffer, uint8_t msb_addr, uint8_t lsb_addr, uint8_t alignment);
void shift_left(uint32_t* value_p, int8_t shift);
int8_t align( uint32_t* value_p, int8_t direction );
void fx20_round(uint32_t *value_p);
uint32_t fx20_divide( struct operand_t* operand_p );
uint32_t fx20_multiply( struct operand_t* operand_p );
uint32_t vispd_correction(uint8_t *buffer);
uint32_t irpd_correction(uint8_t *buffer);
#endif /* SI1132_H_ */
