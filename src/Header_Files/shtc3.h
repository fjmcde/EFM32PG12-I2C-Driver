/*
 * shtc3.h
 *
 *  Created on: Nov 10, 2022
 *      Author: fjmcd
 */

#ifndef SHTC3_H_
#define SHTC3_H_

//***********************************************************************************
// Included Files
//***********************************************************************************
/* System include statements */


/* Silicon Labs include statements */


/* Developer include statements */
#include "i2c.h"
#include "HW_delay.h"


//***********************************************************************************
// defined macros
//***********************************************************************************
#define SHTC3_DEVICE_ADDR         0x70                // Device address for use in I2C
#define SHTC3_PWR_UP_TIME_TYP     180                 // Device typical power-up time (in micro-seconds)
#define SHTC3_PWR_UP_TIME_MAX     240                 // Device maximum power-up time (in micro-seconds)
#define SHTC3_SCL_CLK_FREQ_FM     I2C_FREQ_FAST_MAX   // Frequency of SCL clock in fast-mode (device max is 400kHz)
#define SHTC3_REF_FREQ            0                   // Set to zero to use I2C frequency


//***********************************************************************************
// enums
//***********************************************************************************
typedef enum
{
  /* power and sleep commands */
  sleep                   = 0xB098, /* Sleep */
  wakeup                  = 0x3517, /* Wake-up */
  soft_reset              = 0x805D, /* Software Reset*/
  /* measurement commands */
  read_t_first_nm_cs_en   = 0x7CA2, /* Read temperature first; normal mode; clock stretching enabled */
  read_t_first_lpm_cs_en  = 0x6458, /* Read temperature first; low power mode; clock stretching enabled */
  read_t_first_nm         = 0x7866, /* Read temperature first; normal mode; clock stretching disabled */
  read_t_first_lpm        = 0x609C, /* Read temperature first; low power mode; clock stretching disabled */
  read_rh_first_nm_cs_en  = 0x5C24, /* Read relative humidity first; normal mode; clock stretching enabled */
  read_rh_first_lpm_cs_en = 0x44DE, /* Read relative humidity first; low power mode; clock stretching enabled */
  read_rh_first_nm        = 0x58E0, /* Read relative humidity first; normal mode; clock stretching disabled */
  read_rh_first_lpm       = 0x401A, /* Read relative humidity first; low power mode; clock stretching disabled */
  /* read out commands */
  read_id_reg             = 0xEFC8  /* Read ID register to verify the presence of and communication with the SHTC3 */
}SHTC3_I2C_COMMAND_Typedef;


//***********************************************************************************
// structs
//***********************************************************************************


//***********************************************************************************
// function prototypes
//***********************************************************************************
void shtc3_open(I2C_TypeDef *i2c);
float rh_convert(void);
float temp_convert(void);

#endif
