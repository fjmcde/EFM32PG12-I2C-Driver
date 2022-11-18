/*
 * shtc3.c
 *

 */

//***********************************************************************************
// included header file
//***********************************************************************************
#include "shtc3.h"


//***********************************************************************************
// static/private data
//***********************************************************************************
static volatile uint16_t read_result;


//***********************************************************************************
// static/global functions
//***********************************************************************************


//***********************************************************************************
// function definitions
//***********************************************************************************
void shtc3_open(I2C_TypeDef *i2c)
{
  I2C_OPEN_STRUCT app_i2c_open;

  // give the SHTC3 time for VDD to reach the power-up voltage
  timer_delay(SHTC3_PWR_UP_TIME_MAX);

  // set app specific frequency
  app_i2c_open.freq = SHTC3_SCL_CLK_FREQ_FM;
  app_i2c_open.refFreq = SHTC3_REF_FREQ;

  // set app specific clock ratio to 6:3
  app_i2c_open.clhr = i2cClockHLRAsymetric;

  // set opening behavior
  app_i2c_open.master = true;
  app_i2c_open.enable = true;

  // set application specific route locations and enable pins
  app_i2c_open.scl_loc = SHTC3_SCL_ROUTE_LOC;
  app_i2c_open.sda_loc = SHTC3_SDA_ROUTE_LOC;
  app_i2c_open.scl_pen = I2C_ROUTEPEN_SCLPEN;
  app_i2c_open.sda_pen = I2C_ROUTEPEN_SDAPEN;

  // open I2C peripheral
  i2c_open(i2c, &app_i2c_open);
}


float rh_convert(void)
{
  return (100 * ((float)read_result / 65536));
}


float temp_convert(void)
{
  return (175 * ((float)read_result / 65536) - 45);
}
