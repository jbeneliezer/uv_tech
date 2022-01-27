/*
 * i2c.h
 *
 *  Created on: Jan 26, 2022
 *      Author: Judah Ben-Eliezer, Tennyson Cheng
 */

#ifndef I2C_H_
#define I2C_H_

/***************************************************************************//**
 * @brief
 *   Initializes I2C0 for fast mode.
 *   Uses PB0 for SDA and PB1 for SCLK.
 ******************************************************************************/
void i2c_init(void);

/***************************************************************************//**
 * @brief
 *   Writes 1 byte to a register of slave.
 *
 * @param[in] i2c
 *   Pointer to I2C peripheral register block.
 *
 * @param[in] slave_addr.
 *   Address of slave.
 *
 * @param[in] reg_addr
 *   Internal register address to write to.
 *
 * @param[in] data
 *   Byte to write.
 ******************************************************************************/
I2C_TransferReturn_TypeDef i2c_write_single(I2C_TypeDef *i2c, uint8_t slave_addr, uint8_t reg_addr, uint8_t data);

/***************************************************************************//**
 * @brief
 *   Writes byte array to slave.
 *
 * @param[in] i2c
 *   Pointer to I2C peripheral register block.
 *
 * @param[in] slave_addr.
 *   Address of slave.
 *
 * @param[in] reg_addr
 *   Internal register address to begin write.
 *
 * @param[in] data
 *   Pointer to array of data to write.
 *
 * @param[in] num_bytes
 *   Number of bytes to write.
 ******************************************************************************/
I2C_TransferReturn_TypeDef i2c_write_burst(I2C_TypeDef *i2c, uint8_t slave_addr, uint8_t reg_addr, uint8_t* data, uint16_t num_bytes);

/***************************************************************************//**
 * @brief
 *   Reads 1 byte from a register of slave.
 *
 * @param[in] i2c
 *   Pointer to I2C peripheral register block.
 *
 * @param[in] slave_addr.
 *   Address of slave.
 *
 * @param[in] reg_addr
 *   Internal register address to read from.
 *
 * @param[in] buffer
 *   Buffer to read byte into.
 ******************************************************************************/
I2C_TransferReturn_TypeDef i2c_read_single(I2C_TypeDef *i2c, uint8_t slave_addr, uint8_t reg_addr, uint8_t *buffer);

/***************************************************************************//**
 * @brief
 *   Reads byte array from slave.
 *
 * @param[in] i2c
 *   Pointer to I2C peripheral register block.
 *
 * @param[in] slave_addr.
 *   Address of slave.
 *
 * @param[in] reg_addr
 *   Internal register address to begin read.
 *
 * @param[in] buffer
 *   Buffer to store bytes in.
 *
 * @param[in] num_bytes
 *   Number of bytes to read.
 ******************************************************************************/
I2C_TransferReturn_TypeDef i2c_read_burst(I2C_TypeDef *i2c, uint8_t slave_addr, uint8_t reg_addr, uint8_t *buffer, uint16_t num_bytes);

#endif /* I2C_H_ */
