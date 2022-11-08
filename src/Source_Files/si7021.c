/***************************************************************************//**
 * @file
 *   si7021.c
 * @author
 *   Frank McDermott
 * @date
 *   11/06/2022
 * @brief
 *   Si7021 Driver
 ******************************************************************************/

//***********************************************************************************
// included header file
//***********************************************************************************
#include "si7021.h"


//***********************************************************************************
// static/private data
//***********************************************************************************
static volatile uint16_t read_result;
static volatile uint16_t write_value;

//***********************************************************************************
// static/private functions
//***********************************************************************************


//***********************************************************************************
// function definitions
//***********************************************************************************
/***************************************************************************//**
 * @brief
 *  Opens the Si7021 Temperature & Humidity Sensor I2C peripheral
 *
 * @details
 *  Configures application specific I2C protocol and opens the I2C peripheral
 *
 * @param[in] i2c
 *  Desired I2Cn peripheral (either I2C0 or I2C1)
 ******************************************************************************/
void si7021_i2c_open(I2C_TypeDef *i2c)
{
  // instantiate an app specific I2C
  I2C_OPEN_STRUCT app_i2c_open;

  // set a local delay
  uint32_t delay = DELAY80MS;

  // Powerup Time (delay) worst case: From VDD ≥ 1.9 V to ready for a
  // conversion, full temperature range (80ms)
  timer_delay(delay);

  // set app specific frequency
  app_i2c_open.freq = I2C_FREQ;
  app_i2c_open.refFreq = REFFREQ;

  // set app specific low/high clock ratio
  app_i2c_open.clhr = I2C_CLHR_6_3;

  // set opening behavior
  app_i2c_open.enable = true;
  app_i2c_open.master = true;

  // set route locations and enable pins
  app_i2c_open.scl_loc0 = I2C_SCL_ROUTE;
  app_i2c_open.sda_loc0 = I2C_SDA_ROUTE;
  app_i2c_open.scl_pen = I2C_SCL_PEN;
  app_i2c_open.sda_pen = I2C_SDA_PEN;

  // open I2C peripheral
  i2c_open(i2c, &app_i2c_open);
}


/***************************************************************************//**
 * @brief
 *  Sends a read command to the Si7021 over I2C
 *
 * @details
 *  Currently hard coded to read Relative Humidity (No Hold Master Mode)
 *
 * @param[in] i2c
 *  Desired I2Cn peripheral (either I2C0 or I2C1)
 *
 * @param[in] si7021_cb
 *  Callback event to be scheduled after read operation is complete
 ******************************************************************************/
void si7021_i2c_read(I2C_TypeDef *i2c, uint32_t si7021_cb)
{
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();

  // reset read_result
  read_result = RESET_READ_RESULT;

  // exit core critical to allow interrupts
  CORE_EXIT_CRITICAL();


  // start the I2C protocol (READ RH)
  i2c_start(i2c, SI7021_ADDR, SI7021_I2C_WRITE, &read_result, si7021_cb);
}


/***************************************************************************//**
 * @brief
 *  Sends a read write to the Si7021 over I2C
 *
 * @details
 *  NOT YET WORKING
 *
 * @param[in] i2c
 *  Desired I2Cn peripheral (either I2C0 or I2C1)
 *
 * @param[in] si7021_cb
 *  Callback event to be scheduled after write operation is complete
 ******************************************************************************/
void si7021_i2c_write(I2C_TypeDef *i2c, uint32_t si7021_cb)
{
  // start the I2C protocol (W)
  i2c_start(i2c, SI7021_ADDR, SI7021_I2C_READ, &write_value, si7021_cb);

}


/***************************************************************************//**
 * @brief
 *  Converts a Relative Humidity measurement code to a percent humidity
 *  per Si7021-A20 TRM: Section 5.1.1
 *
 * @details
 *  Atomic function due to accessing static variable
 ******************************************************************************/
float si7021_calc_RH(void)
{
  // make atomic by disallowing interrupts
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();

  // convert the stored RH code to percent humidity (Si7021-A20: 5.1.1)
  float rh = ((125 * (float)read_result) / 65536) - 6;

  // exit core critical to allow interrupts
  CORE_EXIT_CRITICAL();

  return rh;
}
