/***************************************************************************//**
 * @file
 *   gpio.c
 * @author
 *   Frank McDermott
 * @date
 *   11/29/2022
 * @brief
 *   GPIO source file for driving GPIO peripherals
 ******************************************************************************/

//***********************************************************************************
// included header file
//***********************************************************************************
#include "gpio.h"


//***********************************************************************************
// static/private data
//***********************************************************************************


//***********************************************************************************
// static/private functions
//***********************************************************************************


//***********************************************************************************
// function definitions
//***********************************************************************************
/***************************************************************************//**
 * @brief
 *   Driver to open the GPIO peripheral.
 *
 * @details
 *   Enables the GPIO for use with the two onboard LEDs (LED0 & LED1), the
 *   Si7021 and the SHTC3 Temperature & humidity sensors.
 ******************************************************************************/
void gpio_open(void)
{

  // enable clock
  CMU_ClockEnable(cmuClock_GPIO, true);

  // configure Si7021
  GPIO_DriveStrengthSet(SI7021_SENSOR_EN_PORT, SI7021_DRIVE_STRENGTH);
  GPIO_PinModeSet(SI7021_SENSOR_EN_PORT, SI7021_SENSOR_EN_PIN,
                  SI7021_SENSOR_CONFIG, SI7021_DEFAULT_1);
  GPIO_PinModeSet(SI7021_SCL_PORT, SI7021_SCL_PIN, SI7021_WIREDAND, SI7021_DEFAULT_1);
  GPIO_PinModeSet(SI7021_SDA_PORT, SI7021_SDA_PIN, SI7021_WIREDAND, SI7021_DEFAULT_1);

  // configure SHTC3
  GPIO_PinModeSet(SHTC3_SCL_PORT, SHTC3_SCL_PIN, SHTC3_WIREDAND, SHTC3_LINE_DEFAULT);
  GPIO_PinModeSet(SHTC3_SDA_PORT, SHTC3_SDA_PIN, SHTC3_WIREDAND, SHTC3_LINE_DEFAULT);

  // configure LEDs
  GPIO_DriveStrengthSet(LED0_PORT, LED0_DRIVE_STRENGTH);
  GPIO_PinModeSet(LED0_PORT, LED0_PIN, LED0_GPIOMODE, LED0_DEFAULT);
  GPIO_DriveStrengthSet(LED1_PORT, LED1_DRIVE_STRENGTH);
  GPIO_PinModeSet(LED1_PORT, LED1_PIN, LED1_GPIOMODE, LED1_DEFAULT);

  // clear interrupt flags
  GPIO->IFC &= ~(_GPIO_IFC_RESETVALUE);
}


/***************************************************************************//**
 * @brief
 *   Sets up the application-specific peripherals, schedulers, and timers
 *
 * @details
 *   Drives one of the two onboard LEDs on the EFM32 if the relative humidity
 *   measurement threshold is reached.
 ******************************************************************************/
void drive_leds(uint16_t humidity, GPIO_Port_TypeDef led_port, uint8_t led_pin)
{
  // if relative humidity is greater than 30.0%...
  if(humidity >= RH_LED_ON)
  {
      // ... Assert LED1
      GPIO_PinOutSet(led_port, led_pin);
  }
  else
  {
      // De-assert LED1
      GPIO_PinOutClear(led_port, led_pin);
  }
}
