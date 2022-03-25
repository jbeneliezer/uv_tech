#include "sl_emlib_gpio_simple_init.h"
#include "sl_emlib_gpio_init_BATTERY_MONITOR_config.h"
#include "sl_emlib_gpio_init_BUTTON_config.h"
#include "sl_emlib_gpio_init_I2C_VDD_0_config.h"
#include "sl_emlib_gpio_init_I2C_VDD_1_config.h"
#include "sl_emlib_gpio_init_I2C_VDD_2_config.h"
#include "sl_emlib_gpio_init_I2C_VDD_3_config.h"
#include "sl_emlib_gpio_init_LED_config.h"
#include "em_gpio.h"
#include "em_cmu.h"

void sl_emlib_gpio_simple_init(void)
{
  CMU_ClockEnable(cmuClock_GPIO, true);
  GPIO_PinModeSet(SL_EMLIB_GPIO_INIT_BATTERY_MONITOR_PORT,
                  SL_EMLIB_GPIO_INIT_BATTERY_MONITOR_PIN,
                  SL_EMLIB_GPIO_INIT_BATTERY_MONITOR_MODE,
                  SL_EMLIB_GPIO_INIT_BATTERY_MONITOR_DOUT);

  GPIO_PinModeSet(SL_EMLIB_GPIO_INIT_BUTTON_PORT,
                  SL_EMLIB_GPIO_INIT_BUTTON_PIN,
                  SL_EMLIB_GPIO_INIT_BUTTON_MODE,
                  SL_EMLIB_GPIO_INIT_BUTTON_DOUT);

  GPIO_PinModeSet(SL_EMLIB_GPIO_INIT_I2C_VDD_0_PORT,
                  SL_EMLIB_GPIO_INIT_I2C_VDD_0_PIN,
                  SL_EMLIB_GPIO_INIT_I2C_VDD_0_MODE,
                  SL_EMLIB_GPIO_INIT_I2C_VDD_0_DOUT);

  GPIO_PinModeSet(SL_EMLIB_GPIO_INIT_I2C_VDD_1_PORT,
                  SL_EMLIB_GPIO_INIT_I2C_VDD_1_PIN,
                  SL_EMLIB_GPIO_INIT_I2C_VDD_1_MODE,
                  SL_EMLIB_GPIO_INIT_I2C_VDD_1_DOUT);

  GPIO_PinModeSet(SL_EMLIB_GPIO_INIT_I2C_VDD_2_PORT,
                  SL_EMLIB_GPIO_INIT_I2C_VDD_2_PIN,
                  SL_EMLIB_GPIO_INIT_I2C_VDD_2_MODE,
                  SL_EMLIB_GPIO_INIT_I2C_VDD_2_DOUT);

  GPIO_PinModeSet(SL_EMLIB_GPIO_INIT_I2C_VDD_3_PORT,
                  SL_EMLIB_GPIO_INIT_I2C_VDD_3_PIN,
                  SL_EMLIB_GPIO_INIT_I2C_VDD_3_MODE,
                  SL_EMLIB_GPIO_INIT_I2C_VDD_3_DOUT);

  GPIO_PinModeSet(SL_EMLIB_GPIO_INIT_LED_PORT,
                  SL_EMLIB_GPIO_INIT_LED_PIN,
                  SL_EMLIB_GPIO_INIT_LED_MODE,
                  SL_EMLIB_GPIO_INIT_LED_DOUT);
}
