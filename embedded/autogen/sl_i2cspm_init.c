/***************************************************************************//**
 * @file
 * @brief I2C simple poll-based master mode driver instance initialilization
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#include "sl_i2cspm.h"
#include "em_cmu.h"
// Include instance config 
#include "sl_i2cspm_s0_config.h"
#include "sl_i2cspm_s1_config.h"
#include "sl_i2cspm_s2_config.h"
#include "sl_i2cspm_s3_config.h"
#include "sl_i2cspm_s4_config.h"
#include "sl_i2cspm_s5_config.h"
#include "sl_i2cspm_s6_config.h"
#include "sl_i2cspm_s7_config.h"

sl_i2cspm_t *sl_i2cspm_s0 = SL_I2CSPM_S0_PERIPHERAL;
sl_i2cspm_t *sl_i2cspm_s1 = SL_I2CSPM_S1_PERIPHERAL;
sl_i2cspm_t *sl_i2cspm_s2 = SL_I2CSPM_S2_PERIPHERAL;
sl_i2cspm_t *sl_i2cspm_s3 = SL_I2CSPM_S3_PERIPHERAL;
sl_i2cspm_t *sl_i2cspm_s4 = SL_I2CSPM_S4_PERIPHERAL;
sl_i2cspm_t *sl_i2cspm_s5 = SL_I2CSPM_S5_PERIPHERAL;
sl_i2cspm_t *sl_i2cspm_s6 = SL_I2CSPM_S6_PERIPHERAL;
sl_i2cspm_t *sl_i2cspm_s7 = SL_I2CSPM_S7_PERIPHERAL;

#if SL_I2CSPM_S0_SPEED_MODE == 0
#define SL_I2CSPM_S0_HLR i2cClockHLRStandard
#define SL_I2CSPM_S0_MAX_FREQ I2C_FREQ_STANDARD_MAX
#elif SL_I2CSPM_S0_SPEED_MODE == 1
#define SL_I2CSPM_S0_HLR i2cClockHLRAsymetric
#define SL_I2CSPM_S0_MAX_FREQ I2C_FREQ_FAST_MAX
#elif SL_I2CSPM_S0_SPEED_MODE == 2
#define SL_I2CSPM_S0_HLR i2cClockHLRFast
#define SL_I2CSPM_S0_MAX_FREQ I2C_FREQ_FASTPLUS_MAX
#endif

#if SL_I2CSPM_S1_SPEED_MODE == 0
#define SL_I2CSPM_S1_HLR i2cClockHLRStandard
#define SL_I2CSPM_S1_MAX_FREQ I2C_FREQ_STANDARD_MAX
#elif SL_I2CSPM_S1_SPEED_MODE == 1
#define SL_I2CSPM_S1_HLR i2cClockHLRAsymetric
#define SL_I2CSPM_S1_MAX_FREQ I2C_FREQ_FAST_MAX
#elif SL_I2CSPM_S1_SPEED_MODE == 2
#define SL_I2CSPM_S1_HLR i2cClockHLRFast
#define SL_I2CSPM_S1_MAX_FREQ I2C_FREQ_FASTPLUS_MAX
#endif

#if SL_I2CSPM_S2_SPEED_MODE == 0
#define SL_I2CSPM_S2_HLR i2cClockHLRStandard
#define SL_I2CSPM_S2_MAX_FREQ I2C_FREQ_STANDARD_MAX
#elif SL_I2CSPM_S2_SPEED_MODE == 1
#define SL_I2CSPM_S2_HLR i2cClockHLRAsymetric
#define SL_I2CSPM_S2_MAX_FREQ I2C_FREQ_FAST_MAX
#elif SL_I2CSPM_S2_SPEED_MODE == 2
#define SL_I2CSPM_S2_HLR i2cClockHLRFast
#define SL_I2CSPM_S2_MAX_FREQ I2C_FREQ_FASTPLUS_MAX
#endif

#if SL_I2CSPM_S3_SPEED_MODE == 0
#define SL_I2CSPM_S3_HLR i2cClockHLRStandard
#define SL_I2CSPM_S3_MAX_FREQ I2C_FREQ_STANDARD_MAX
#elif SL_I2CSPM_S3_SPEED_MODE == 1
#define SL_I2CSPM_S3_HLR i2cClockHLRAsymetric
#define SL_I2CSPM_S3_MAX_FREQ I2C_FREQ_FAST_MAX
#elif SL_I2CSPM_S3_SPEED_MODE == 2
#define SL_I2CSPM_S3_HLR i2cClockHLRFast
#define SL_I2CSPM_S3_MAX_FREQ I2C_FREQ_FASTPLUS_MAX
#endif

#if SL_I2CSPM_S4_SPEED_MODE == 0
#define SL_I2CSPM_S4_HLR i2cClockHLRStandard
#define SL_I2CSPM_S4_MAX_FREQ I2C_FREQ_STANDARD_MAX
#elif SL_I2CSPM_S4_SPEED_MODE == 1
#define SL_I2CSPM_S4_HLR i2cClockHLRAsymetric
#define SL_I2CSPM_S4_MAX_FREQ I2C_FREQ_FAST_MAX
#elif SL_I2CSPM_S4_SPEED_MODE == 2
#define SL_I2CSPM_S4_HLR i2cClockHLRFast
#define SL_I2CSPM_S4_MAX_FREQ I2C_FREQ_FASTPLUS_MAX
#endif

#if SL_I2CSPM_S5_SPEED_MODE == 0
#define SL_I2CSPM_S5_HLR i2cClockHLRStandard
#define SL_I2CSPM_S5_MAX_FREQ I2C_FREQ_STANDARD_MAX
#elif SL_I2CSPM_S5_SPEED_MODE == 1
#define SL_I2CSPM_S5_HLR i2cClockHLRAsymetric
#define SL_I2CSPM_S5_MAX_FREQ I2C_FREQ_FAST_MAX
#elif SL_I2CSPM_S5_SPEED_MODE == 2
#define SL_I2CSPM_S5_HLR i2cClockHLRFast
#define SL_I2CSPM_S5_MAX_FREQ I2C_FREQ_FASTPLUS_MAX
#endif

#if SL_I2CSPM_S6_SPEED_MODE == 0
#define SL_I2CSPM_S6_HLR i2cClockHLRStandard
#define SL_I2CSPM_S6_MAX_FREQ I2C_FREQ_STANDARD_MAX
#elif SL_I2CSPM_S6_SPEED_MODE == 1
#define SL_I2CSPM_S6_HLR i2cClockHLRAsymetric
#define SL_I2CSPM_S6_MAX_FREQ I2C_FREQ_FAST_MAX
#elif SL_I2CSPM_S6_SPEED_MODE == 2
#define SL_I2CSPM_S6_HLR i2cClockHLRFast
#define SL_I2CSPM_S6_MAX_FREQ I2C_FREQ_FASTPLUS_MAX
#endif

#if SL_I2CSPM_S7_SPEED_MODE == 0
#define SL_I2CSPM_S7_HLR i2cClockHLRStandard
#define SL_I2CSPM_S7_MAX_FREQ I2C_FREQ_STANDARD_MAX
#elif SL_I2CSPM_S7_SPEED_MODE == 1
#define SL_I2CSPM_S7_HLR i2cClockHLRAsymetric
#define SL_I2CSPM_S7_MAX_FREQ I2C_FREQ_FAST_MAX
#elif SL_I2CSPM_S7_SPEED_MODE == 2
#define SL_I2CSPM_S7_HLR i2cClockHLRFast
#define SL_I2CSPM_S7_MAX_FREQ I2C_FREQ_FASTPLUS_MAX
#endif

I2CSPM_Init_TypeDef init_s0 = { 
  .port = SL_I2CSPM_S0_PERIPHERAL,
  .sclPort = SL_I2CSPM_S0_SCL_PORT,
  .sclPin = SL_I2CSPM_S0_SCL_PIN,
  .sdaPort = SL_I2CSPM_S0_SDA_PORT,
  .sdaPin = SL_I2CSPM_S0_SDA_PIN,
  .i2cRefFreq = 0,
  .i2cMaxFreq = SL_I2CSPM_S0_MAX_FREQ,
  .i2cClhr = SL_I2CSPM_S0_HLR
};

I2CSPM_Init_TypeDef init_s1 = { 
  .port = SL_I2CSPM_S1_PERIPHERAL,
  .sclPort = SL_I2CSPM_S1_SCL_PORT,
  .sclPin = SL_I2CSPM_S1_SCL_PIN,
  .sdaPort = SL_I2CSPM_S1_SDA_PORT,
  .sdaPin = SL_I2CSPM_S1_SDA_PIN,
  .i2cRefFreq = 0,
  .i2cMaxFreq = SL_I2CSPM_S1_MAX_FREQ,
  .i2cClhr = SL_I2CSPM_S1_HLR
};

I2CSPM_Init_TypeDef init_s2 = { 
  .port = SL_I2CSPM_S2_PERIPHERAL,
  .sclPort = SL_I2CSPM_S2_SCL_PORT,
  .sclPin = SL_I2CSPM_S2_SCL_PIN,
  .sdaPort = SL_I2CSPM_S2_SDA_PORT,
  .sdaPin = SL_I2CSPM_S2_SDA_PIN,
  .i2cRefFreq = 0,
  .i2cMaxFreq = SL_I2CSPM_S2_MAX_FREQ,
  .i2cClhr = SL_I2CSPM_S2_HLR
};

I2CSPM_Init_TypeDef init_s3 = { 
  .port = SL_I2CSPM_S3_PERIPHERAL,
  .sclPort = SL_I2CSPM_S3_SCL_PORT,
  .sclPin = SL_I2CSPM_S3_SCL_PIN,
  .sdaPort = SL_I2CSPM_S3_SDA_PORT,
  .sdaPin = SL_I2CSPM_S3_SDA_PIN,
  .i2cRefFreq = 0,
  .i2cMaxFreq = SL_I2CSPM_S3_MAX_FREQ,
  .i2cClhr = SL_I2CSPM_S3_HLR
};

I2CSPM_Init_TypeDef init_s4 = { 
  .port = SL_I2CSPM_S4_PERIPHERAL,
  .sclPort = SL_I2CSPM_S4_SCL_PORT,
  .sclPin = SL_I2CSPM_S4_SCL_PIN,
  .sdaPort = SL_I2CSPM_S4_SDA_PORT,
  .sdaPin = SL_I2CSPM_S4_SDA_PIN,
  .i2cRefFreq = 0,
  .i2cMaxFreq = SL_I2CSPM_S4_MAX_FREQ,
  .i2cClhr = SL_I2CSPM_S4_HLR
};

I2CSPM_Init_TypeDef init_s5 = { 
  .port = SL_I2CSPM_S5_PERIPHERAL,
  .sclPort = SL_I2CSPM_S5_SCL_PORT,
  .sclPin = SL_I2CSPM_S5_SCL_PIN,
  .sdaPort = SL_I2CSPM_S5_SDA_PORT,
  .sdaPin = SL_I2CSPM_S5_SDA_PIN,
  .i2cRefFreq = 0,
  .i2cMaxFreq = SL_I2CSPM_S5_MAX_FREQ,
  .i2cClhr = SL_I2CSPM_S5_HLR
};

I2CSPM_Init_TypeDef init_s6 = { 
  .port = SL_I2CSPM_S6_PERIPHERAL,
  .sclPort = SL_I2CSPM_S6_SCL_PORT,
  .sclPin = SL_I2CSPM_S6_SCL_PIN,
  .sdaPort = SL_I2CSPM_S6_SDA_PORT,
  .sdaPin = SL_I2CSPM_S6_SDA_PIN,
  .i2cRefFreq = 0,
  .i2cMaxFreq = SL_I2CSPM_S6_MAX_FREQ,
  .i2cClhr = SL_I2CSPM_S6_HLR
};

I2CSPM_Init_TypeDef init_s7 = { 
  .port = SL_I2CSPM_S7_PERIPHERAL,
  .sclPort = SL_I2CSPM_S7_SCL_PORT,
  .sclPin = SL_I2CSPM_S7_SCL_PIN,
  .sdaPort = SL_I2CSPM_S7_SDA_PORT,
  .sdaPin = SL_I2CSPM_S7_SDA_PIN,
  .i2cRefFreq = 0,
  .i2cMaxFreq = SL_I2CSPM_S7_MAX_FREQ,
  .i2cClhr = SL_I2CSPM_S7_HLR
};

void sl_i2cspm_init_instances(void)
{
  CMU_ClockEnable(cmuClock_GPIO, true);
  I2CSPM_Init(&init_s0);
  I2CSPM_Init(&init_s1);
  I2CSPM_Init(&init_s2);
  I2CSPM_Init(&init_s3);
  I2CSPM_Init(&init_s4);
  I2CSPM_Init(&init_s5);
  I2CSPM_Init(&init_s6);
  I2CSPM_Init(&init_s7);
}
