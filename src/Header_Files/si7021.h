//*******************************************************
// header guards
//*******************************************************
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
/* Delays */
#define DELAY80MS               50        // 80ms delay for [worst case] timer delay (DS Table 2 (cont.) pg 5)
/* I2C Reference Frequency [refFreq] */
#define REFFREQ                 0         // Set to zero to use I2C frequency
/* Device specific address */
#define SI7021_ADDR             0x40      // Si7021 peripheral device address
/* Bit Masks [read_result] */
#define RESET_READ_RESULT       0x00      // Use when resetting the read_result static variable
/* Bit Masks [write_data] */
#define RESET_WRITE_DATA        0x00      // Use when resetting the write_data static variable
/* Number of bytes I2C should expect */
#define SI7021_TX_1_BYTE        1         // number of bytes to expect from a write (transmit single bytes)
#define SI7021_REQ_2_BYTES      2         // number of bytes to expect from a read (Ignore checksum)
#define SI7021_REQ_3_BYTES      3         // number of bytes to expect from a read (Checksum requested)

//***********************************************************************************
// enums
//***********************************************************************************
// enumerated I2C commands (Si7021-A20 DS 5.0, Table 11)
typedef enum
{
  read_id_byte1_0       = 0x0F,   /* Read Electronic ID 1st Byte (Checksum byte required) */
  read_heater_ctrl      = 0x11,   /* Read Heater Control Register */
  write_heater_ctrl     = 0x51,   /* Write Heater Control Register */
  read_fw_rev0          = 0x84,   /* Read Firmware Revision */
  read_fw_rev1          = 0xB8,   /* Read Firmware Revision */
  read_id_byte2_0       = 0xC9,   /* Read Electronic ID 2nd Byte (Checksum byte required) */
  read_T_from_prev_RH   = 0xE0,   /* Read Temperature Value from Previous RH Measurement */
  measure_T_HMM         = 0xE3,   /* Measure Temperature, Hold Master Mode */
  measure_RH_HMM        = 0xE5,   /* Measure Relative Humidity, Hold Master Mode */
  write_reg1            = 0xE6,   /* Write RH/T User Register 1 */
  read_reg1             = 0xE7,   /* Read RH/T User Register 1 */
  measure_T_NHMM        = 0xF3,   /* Measure Temperature, No Hold Master Mode */
  measure_RH_NHMM       = 0xF5,   /* Measure Relative Humidity, No Hold Master Mode */
  read_id_byte1_1       = 0xFA,   /* Read Electronic ID 1st Byte (Checksum byte required) */
  read_id_byte2_1       = 0xFC,   /* Read Electronic ID 2nd Byte (Checksum byte required) */
  reset                 = 0xFE    /* Reset */
}SI7021_CMD_Typedef;


typedef enum
{
  reset_reg1            = 0x3A,   /* Reset register: 0b0011 1010 */
  measure_res_RH12_T14  = 0x00,   /* Measurement Resolution: RH 12-bit; T 14-bit; 0b0xxx xxx0 */
  measure_res_RH8_T12   = 0x01,   /* Measurement Resolution: RH 12-bit; T 12-bit; 0b0xxx xxx1 */
  measure_res_RH10_T13  = 0x80,   /* Measurement Resolution: RH 10-bit; T 13-bit; 0b1xxx xxx0 */
  measure_res_RH11_T11  = 0x81,   /* Measurement Resolution: RH 11-bit; T 11-bit; 0b1xxx xxx1 */
  heater_en = 0x04,               /* Enable Heater: 0bxxxx_x1xx */
}SI7021_USER_REG1_Typedef;


typedef enum
{
  ctrl_heater_curr_3     = 0x00,  /* Heater Current 03.09 mA */
  ctrl_heater_curr_9     = 0x01,  /* Heater Current 09.18 mA */
  ctrl_heater_curr_15    = 0x02,  /* Heater Current 15.24 mA */
  ctrl_heater_curr_27    = 0x04,  /* Heater Current 27.39 mA */
  ctrl_heater_curr_51    = 0x08,  /* Heater Current 51.69 mA */
  ctrl_heater_curr_94    = 0x0F,  /* Heater Current 94.20 mA */
}SI7021_HEATER_CTRL_Typedef;

//***********************************************************************************
// structs
//***********************************************************************************


//***********************************************************************************
// function prototypes
//***********************************************************************************
/* Peripheral open function */
void si7021_i2c_open(I2C_TypeDef *i2c);
/* R/W operation functions */
void si7021_i2c_read(I2C_TypeDef *i2c, SI7021_CMD_Typedef cmd, bool checksum, uint32_t si7021_cb);
void si7021_i2c_write(I2C_TypeDef *i2c, SI7021_CMD_Typedef cmd, uint8_t ctrl, uint32_t si7021_cb);
/* Conversion functions */
float si7021_calc_RH(void);
float si7021_calc_temp(void);
/* Accessor member functions */
uint32_t si7021_read_user_reg(void);

#endif
