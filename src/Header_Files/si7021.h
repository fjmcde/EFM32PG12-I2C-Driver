#ifndef SI7021_HG
#define SI7021_HG


//***********************************************************************************
// included files
//***********************************************************************************
// system included files


// Silicon Labs included files


// developer included files
#include "HW_delay.h"
#include "i2c.h"


//***********************************************************************************
// defined macros
//***********************************************************************************
/* Delays for powerup and conversions */
#define SI7021_PU_DELAY_TYP         18        // Typical powerup time delay from VDD ≥ 1.9V, in milliseconds
#define SI7021_PU_DELAY_CONV_MAX    25        // Max powerup time delay for conversion, 25°C, in milliseconds
#define SI7021_PU_DELAY_FULL_MAX    80        // Max powerup time delay conversion, full temperature range, in milliseconds
#define SI7021_PU_DELAY_RESET_TYP   5         // Typical powerup timer delay after soft reset, in milliseconds
#define SI7021_PU_DELAY_RESET_MAX   15        // Maximum powerup time delay after soft reset, in milliseconds
#define SI7021_CONV_DELAY_RH12_TYP  10        // Typical conversion delay for 12-bit RH, in milliseconds
#define SI7021_CONV_DELAY_RH11_TYP  6         // Typical conversion delay for 11-bit RH, in milliseconds
#define SI7021_CONV_DELAY_RH10_TYP  4         // Typical conversion delay for 10-bit RH, in milliseconds
#define SI7021_CONV_DELAY_RH8_TYP   3         // Typical conversion delay for 8-bit RH, in milliseconds
#define SI7021_CONV_DELAY_RH12_MAX  12        // Maximum conversion delay for 12-bit RH, in milliseconds
#define SI7021_CONV_DELAY_RH11_MAX  7         // Maximum conversion delay for 11-bit RH, in milliseconds
#define SI7021_CONV_DELAY_RH10_MAX  5         // Maximum conversion delay for 10-bit RH, in milliseconds
#define SI7021_CONV_DELAY_RH8_MAX   4         // Maximum conversion delay for 8-bit RH, in milliseconds
#define SI7021_CONV_DELAY_T_14_TYP  7         // Typical conversion delay for 14-bit temperature, in milliseconds
#define SI7021_CONV_DELAY_T_13_TYP  4         // Typical conversion delay for 13-bit temperature, in milliseconds
#define SI7021_CONV_DELAY_T_12_TYP  3         // Typical conversion delay for 12-bit temperature, in milliseconds
#define SI7021_CONV_DELAY_T_11_TYP  2         // Typical conversion delay for 11-bit temperature, in milliseconds
#define SI7021_CONV_DELAY_T_14_MAX  11        // Maximum conversion delay for 14-bit temperature, in milliseconds
#define SI7021_CONV_DELAY_T_13_MAX  7         // Maximum conversion delay for 13-bit temperature, in milliseconds
#define SI7021_CONV_DELAY_T_12_MAX  4         // Maximum conversion delay for 12-bit temperature, in milliseconds
#define SI7021_CONV_DELAY_T_11_MAX  3         // Maximum conversion delay for 11-bit temperature, in milliseconds
/* I2C Reference Frequency [refFreq] */
#define SI7021_REFFREQ            0         // Set to zero to use I2C frequency
/* Device specific address */
#define SI7021_ADDR               0x40      // Si7021 peripheral device address
/* Bit Masks [read_result] */
#define SI7021_RESET_READ_RESULT  0x00      // Use when resetting the read_result static variable
/* Bit Masks [write_data] */
#define SI7021_RESET_WRITE_DATA   0x00      // Use when resetting the write_data static variable
/* Number of bytes I2C should expect */
#define SI7021_TX_1_BYTE          1         // number of bytes to expect from a write (transmit single bytes)
#define SI7021_REQ_1_BYTE         1         // expect one byte from a read
#define SI7021_REQ_2_BYTES        2         // expect two bytes from a read
#define SI7021_REQ_3_BYTES        3         // expect three bytes from a read


//***********************************************************************************
// enums
//***********************************************************************************
// enumerated I2C commands (Si7021-A20 DS 5.0, Table 11)
typedef enum
{
  /* Write commands: User & Heater */
  writeReg1           = 0xE6,   /* Write RH/T User Register 1 */
  writeHeaterCtrl     = 0x51,   /* Write Heater Control Register */
  /* Measurement Commands: T & RH */
  measureT_HMM        = 0xE3,   /* Measure Temperature, Hold Master Mode */
  measureT_NHMM       = 0xF3,   /* Measure Temperature, No Hold Master Mode */
  measureRH_HMM       = 0xE5,   /* Measure Relative Humidity, Hold Master Mode */
  measureRH_NHMM      = 0xF5,   /* Measure Relative Humidity, No Hold Master Mode */
  MeasureTFromPrevRH  = 0xE0,   /* Measure Temperature Value from Previous RH Measurement */
  /* Read commands: User & Heater */
  readReg1            = 0xE7,   /* Read RH/T User Register 1 */
  readHeaterCtrl      = 0x11,   /* Read Heater Control Register */
  /* Read commands: Firmware Revision */
  readFWRev0          = 0x84,   /* Read Firmware Revision */
  readFWRev1          = 0xB8,   /* Read Firmware Revision */
  /* Read commands: Electronic ID */
  readIDByte1_0       = 0x0F,   /* Read Electronic ID 1st Byte (Checksum byte required) */
  readIDByte1_1       = 0xFA,   /* Read Electronic ID 1st Byte (Checksum byte required) */
  readIDByte2_0       = 0xC9,   /* Read Electronic ID 2nd Byte (Checksum byte required) */
  readIDByte2_1       = 0xFC,   /* Read Electronic ID 2nd Byte (Checksum byte required) */
  /* Device */
  reset               = 0xFE    /* Reset */
}SI7021_CMD_Typedef;


// enumerated user register 1 control settings
typedef enum
{
  resetReg1            = 0x3A,   /* Reset register: 0b0011 1010 */
  measureResRH12_T14   = 0x00,   /* Measurement Resolution: RH 12-bit; T 14-bit; 0b0xxx xxx0 */
  measureResRH8_T12    = 0x01,   /* Measurement Resolution: RH 12-bit; T 12-bit; 0b0xxx xxx1 */
  measureResRH10_T13   = 0x80,   /* Measurement Resolution: RH 10-bit; T 13-bit; 0b1xxx xxx0 */
  measureResRH11_T11   = 0x81,   /* Measurement Resolution: RH 11-bit; T 11-bit; 0b1xxx xxx1 */
  heaterEn             = 0x04,   /* Enable Heater: 0bxxxx_x1xx */
}SI7021_USER_REG1_CTRL_Typedef;


// enumerated heater control current settings (See: SI7021 DS section 6.1)
typedef enum
{
  ctrlHeaterCurr3     = 0x00,  /* Heater Current 03.09 mA */
  ctrlHeaterCurr9     = 0x01,  /* Heater Current 09.18 mA */
  ctrlHeaterCurr15    = 0x02,  /* Heater Current 15.24 mA */
  ctrlHeaterCurr27    = 0x04,  /* Heater Current 27.39 mA */
  ctrlHeaterCurr51    = 0x08,  /* Heater Current 51.69 mA */
  ctrlHeaterCurr94    = 0x0F,  /* Heater Current 94.20 mA */
}SI7021_HEATER_CTRL_Typedef;


//***********************************************************************************
// structs
//***********************************************************************************


//***********************************************************************************
// function prototypes
//***********************************************************************************
/* Peripheral open function */
void si7021_i2c_open(I2C_TypeDef *i2c,
                     SI7021_CMD_Typedef cmd,
                     SI7021_USER_REG1_CTRL_Typedef ctrl);
/* R/W operation functions */
void si7021_i2c_read(I2C_TypeDef *i2c, SI7021_CMD_Typedef cmd, bool checksum, uint32_t si7021_cb);
void si7021_i2c_write(I2C_TypeDef *i2c, SI7021_CMD_Typedef cmd, uint8_t ctrl, uint32_t si7021_cb);
/* Conversion functions */
void si7021_parse_RH_data(void);
void si7021_parse_temp_data(void);
/* Accessor member functions */
uint8_t si7021_store_user_reg(void);
float si7021_get_rh();
float si7021_get_temp();

#endif
