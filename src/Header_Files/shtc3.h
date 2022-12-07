/***************************************************************************//**
 * @file
 *   shtc3.h
 * @author
 *   Frank McDermott
 * @date
 *   11/29/2022
 * @brief
 *   Header file for SHTC3 device
 ******************************************************************************/

#ifndef SHTC3_H_
#define SHTC3_H_

//***********************************************************************************
// Included Files
//***********************************************************************************
/* System include statements */


/* Silicon Labs include statements */


/* Developer include statements */
#include "HW_delay.h"
#include "i2c.h"

//***********************************************************************************
// defined macros
//***********************************************************************************
/* Device Address */
#define SHTC3_ADDR                0x70                // Device address for use in I2C
/* Delays */
#define SHTC3_PWR_UP_TIME_TYP     180                 // Device typical power-up time (in micro-seconds)
#define SHTC3_PWR_UP_TIME_MAX     240                 // Device maximum power-up time (in micro-seconds)
#define SHTC3_MSR_DELAY_NM_TYP    11                  // Device typical measurement duration (Normal Mode; in milli-seconds)
#define SHTC3_MSR_DELAY_NM_MAX    13                  // Device maximum measurement duration (Normal Mode; in milli-seconds)
#define SHTC3_MSR_DELAY_LPM_TYP   1                   // Device typical measurement duration (Low Power Mode; in milli-seconds)
#define SHTC3_MSR_DELAY_LPM_MAX   1                   // Device maximum measurement duration (Low Power Mode; in milli-seconds)
/* Device Frequencies */
#define SHTC3_SCL_CLK_FREQ_FM     I2C_FREQ_FAST_MAX   // Frequency of SCL clock in fast-mode (device max is 400kHz)
#define SHTC3_REF_FREQ            0                   // Set to zero to use I2C frequency
/* Bit Masks [read_result] */
#define SHTC3_RESET_READ_RESULT   0x00                // Reset read result to zero
#define SHTC3_RESET_WRITE_DATA    0X00
/* Number of bytes I2C should expect */
#define SHTC3_ZERO_BYTES          0                   // expect zero bytes for either read or write
#define SHTC3_TX_2_BYTES          2                   // expect two bytes from a write
#define SHTC3_REQ_6_BYTES         6                   // expect six bytes from a read


//***********************************************************************************
// enums
//***********************************************************************************
/*! Enumerated SHTC3 commands */
typedef enum
{
  /* power and sleep commands */
  sleep                 = 0xB098, /*! Sleep */
  wakeup                = 0x3517, /*! Wake-up */
  softReset             = 0x805D, /*! Software Reset*/
  /* measurement commands */
  readTFirst_NM_CS      = 0x7CA2, /*! Read temperature first; normal mode; clock stretching enabled */
  readTFirst_LPM_CS     = 0x6458, /*! Read temperature first; low power mode; clock stretching enabled */
  readTFirst_NM         = 0x7866, /*! Read temperature first; normal mode; clock stretching disabled */
  readTFirst_LPM        = 0x609C, /*! Read temperature first; low power mode; clock stretching disabled */
  readRHFirst_NM_CS     = 0x5C24, /*! Read relative humidity first; normal mode; clock stretching enabled */
  readRHFirst_LPM_CS    = 0x44DE, /*! Read relative humidity first; low power mode; clock stretching enabled */
  readRHFirst_NM        = 0x58E0, /*! Read relative humidity first; normal mode; clock stretching disabled */
  readRHFirst_LPM       = 0x401A, /*! Read relative humidity first; low power mode; clock stretching disabled */
  /* read out commands */
  read_id_reg           = 0xEFC8  /*! Read ID register to verify the presence of and communication with the SHTC3 */
}SHTC3_CMD_Typedef;


//***********************************************************************************
// structs
//***********************************************************************************


//***********************************************************************************
// function prototypes
//***********************************************************************************
/* Peripheral functions */
void shtc3_open(I2C_TypeDef *i2c);
/* Read/Write functions */
void shtc3_write(I2C_TypeDef *i2c, SHTC3_CMD_Typedef cmd, uint32_t shtc3_cb);
void shtc3_read(I2C_TypeDef *i2c, bool checksum, uint32_t shtc3_cb);
/* Conversion functions */
void shtc3_parse_measurement_data_RH_first(void);
/* Accessor functions */
float shtc3_get_rh(void);
float shtc3_get_temp(void);
/* Modifier functions */
void shtc3_set_rh(float rh);
void shtc3_set_temp(float temp);

#endif
