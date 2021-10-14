/*
 * sensor.c
 *
 *  Created on: Oct 13, 2021
 *      Author: Judah Ben-Eliezer
 *
 *  All code related to the Si1132 sensors i2c interface.
 *
 */

#include <string.h>
#include "sl_udelay.h"
#include "em_gpio.h"
#include "gpiointerrupt.h"
#include "sl_i2cspm_instances.h"
#include "sensor.h"

/* Initializes sensors.
 * Delays for startup time and writes 0x17 to HW_KEY for proper operation
 */
int sensor_init(sl_i2cspm_t* sensor)
{

  /* enable interrupts */
  GPIOINT_Init();

  I2C_TransferReturn_TypeDef status;
  uint8_t init_code = 0x17;
  uint8_t en_uv = EN_UV;
  uint8_t ucoef[4] = {0x7B, 0x6B, 0x01, 0x00};
  uint8_t vis_range = VIS_RANGE;
  uint8_t ir_range = IR_RANGE;

  I2C_TransferSeq_TypeDef init_seq = {(vu8) HW_KEY, I2C_FLAG_WRITE, {{&init_code, 1}}};
  I2C_TransferSeq_TypeDef uv_enable = {(vu8) CHLIST, I2C_FLAG_WRITE, {{&en_uv, 1}}};
  I2C_TransferSeq_TypeDef ucoef_config[4] =
  {
      {(vu8) UCOEF0, I2C_FLAG_WRITE, {{&ucoef[0], 1}}},
      {(vu8) UCOEF1, I2C_FLAG_WRITE, {{&ucoef[1], 1}}},
      {(vu8) UCOEF2, I2C_FLAG_WRITE, {{&ucoef[2], 1}}},
      {(vu8) UCOEF3, I2C_FLAG_WRITE, {{&ucoef[3], 1}}}
  };
  I2C_TransferSeq_TypeDef vis_range_enable = {(vu8) ALS_VIS_ADC_MISC, I2C_FLAG_WRITE, {{&vis_range, 1}}};
  I2C_TransferSeq_TypeDef ir_range_enable = {(vu8) ALS_IR_ADC_MISC, I2C_FLAG_WRITE, {{&ir_range, 1}}};

  sl_udelay_wait(STARTUP_TIME);   // allow time for startup

  /* write 0x17 to HW_KEY */
  status = I2CSPM_Transfer(sensor, &init_seq);
  if (status < i2cTransferDone) return status;


  /* enable uv detection */
  status = I2CSPM_Transfer(sensor, &uv_enable);
  if (status < i2cTransferDone) return status;

  /* configure ucoef for direct sunlight */
  I2C_TransferSeq_TypeDef *i = ucoef_config;
  while (i <= &ucoef_config[3])
    {
      status = I2CSPM_Transfer(sensor, i);
      if (status < i2cTransferDone) return status;
      ++i;
    }

  /* enable vis_range and ir_range */
  status = I2CSPM_Transfer(sensor, &vis_range_enable);
  if (status < i2cTransferDone) return status;
  status = I2CSPM_Transfer(sensor, &ir_range_enable);
  if (status < i2cTransferDone) return status;

  return 0;
}

int setmode(sl_i2cspm_t* sensor, Mode_t m)
{
  int status;
  uint16_t rate = (m == AUTO) ? RATE/31.25: 0;
  uint8_t meas[2];
  memcpy(meas, &rate, 2);   // calculate values for meas_h and meas_l: polling rate

  I2C_TransferSeq_TypeDef meas_h = {(vu8) MEAS_RATE1, I2C_FLAG_WRITE, {{&meas[0], 1}}};
  I2C_TransferSeq_TypeDef meas_l = {(vu8) MEAS_RATE0, I2C_FLAG_WRITE, {{&meas[1], 1}}};


  /* set poll rate */
  status = I2CSPM_Transfer(sensor, &meas_h);
  if (status < i2cTransferDone) return status;
  status = I2CSPM_Transfer(sensor, &meas_l);
  if (status < i2cTransferDone) return status;


  Command_t c = {OTHER, ((m == AUTO) ? ALS_AUTO: ALS_FORCE)};
  send_command(sensor, &c);

  /* set up interrupts */
  if (m == FORCE)
    {
      /* Si1132 Interrupt Configuration */
      uint8_t int_cfg = INT_OE, als_ie = ALS_IE;
      I2C_TransferSeq_TypeDef config_interrupts = {(vu8) INT_CFG, I2C_FLAG_WRITE, {{&int_cfg, 1}}};
      I2C_TransferSeq_TypeDef enable_interrupts = {(vu8) IRQ_ENABLE, I2C_FLAG_WRITE, {{&als_ie, 1}}};

      status = I2CSPM_Transfer(sensor, &config_interrupts);
      if (status < i2cTransferDone) return status;
      status = I2CSPM_Transfer(sensor, &enable_interrupts);
      if (status < i2cTransferDone) return status;

      /* Configure INT pin of microcontroller */
      GPIOINT_CallbackRegister(INT, irq_read);
      GPIO_IntEnable(1<<2);
    }
  return 0;
}

/* protocol for sending commands to Si1132 sensors */
uint8_t send_command(sl_i2cspm_t* sensor, Command_t* command)
{
  I2C_TransferReturn_TypeDef status;
  uint8_t zero = 0x00, response, i = 100;
  uint8_t cmd = (command->upper << 5) & (command->lower);

  I2C_TransferSeq_TypeDef clear = {(vu8) COMMAND, I2C_FLAG_WRITE, {{&zero, 1}}};
  I2C_TransferSeq_TypeDef read_response = {(vu8) RESPONSE,I2C_FLAG_READ, {{&response, 1}}};
  I2C_TransferSeq_TypeDef send_cmd = {(vu8) COMMAND, I2C_FLAG_READ, {{&cmd, 1}}};

  status = I2CSPM_Transfer(sensor, &clear);
  if(status < i2cTransferDone)
    {
      printf("Transfer error: %d\n", &status);
      return -1;
    }
  status = I2CSPM_Transfer(sensor, &read_response);
  if(status < i2cTransferDone)
    {
      printf("Transfer error: %d\n", &status);
      return -1;
    }
  if (response != 0x00) return response;

  /* send command to command register until response is detected */
  while (response == 0x00 && i-- > 0)
    {
      status = I2CSPM_Transfer(sensor, &send_cmd);
      if(status < i2cTransferDone)
        {
          printf("Transfer error: %d\n", &status);
          return -1;
        }
        status = I2CSPM_Transfer(sensor, &read_response);
      if(status < i2cTransferDone)
        {
          printf("Transfer error: %d\n", &status);
          return -1;
        }
    }

  return response;
}

/*
 * Use poll when in auto mode with interrupts disabled.
 * Call every RATE us
 */
unsigned double poll(sl_i2cspm_t* sensor)
{
  int status;
  uint8_t datah, datal;
  uint16_t ret;
  I2C_TransferSeq_TypeDef dh = {(vu8) AUX_DATA1, I2C_FLAG_WRITE, {{&datah, 1}}};
  I2C_TransferSeq_TypeDef dl = {(vu8) AUX_DATA0, I2C_FLAG_WRITE, {{&datal, 1}}};

  /* read high byte */
  status = I2CSPM_Transfer(sensor, &dh);
  if(status < i2cTransferDone)
    {
      printf("Transfer error: %d\n", &status);
      return -1;
    }

  /* read low byte */
  status = I2CSPM_Transfer(sensor, &dl);
  if(status < i2cTransferDone)
    {
      printf("Transfer error: %d\n", &status);
      return -1;
    }

  ret = datah;
  ret <<= 8;
  ret |= datal;

  return ret/100;
}


void intread( uint8_t intNo)
{

}
