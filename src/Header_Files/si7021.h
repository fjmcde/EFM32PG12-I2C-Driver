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
#define DELAY80MS              0X50     // 80ms delay for [worst case] timer delay (DS Table 2 (cont.) pg 5)
#define REFFREQ                0X00     // Set to zero to use I2C frequency
#define SI7021_ADDR            0x40     // Si7021 peripheral device address
#define SI7021_I2C_READ        0X01     // READ BIT = 1; Si7021 TRM 5.1
#define SI7021_I2C_WRITE       0X00     // WRITE BIT = 0; Si7021 TRM 5.1
#define RESET_READ_RESULT      0x00     // Use when resetting the read_result static variable


//***********************************************************************************
// enums
//***********************************************************************************
// enumerated I2C commands (Si7021-A20 TRM 5.0, Table 11)
typedef enum
{
  read_id_byte1_0       = 0x0F, /* Read Electronic ID 1st Byte (Checksum byte required) */
  read_heater_ctrl      = 0x11, /* Read Heater Control Register */
  write_heater_ctrl     = 0x51, /* Write Heater Control Register */
  read_fw_rev0          = 0x84, /* Read Firmware Revision */
  read_fw_rev1          = 0xB8, /* Read Firmware Revision */
  read_id_byte2_0       = 0xC9, /* Read Electronic ID 2nd Byte (Checksum byte required) */
  read_T_from_prev_RH   = 0xE0, /* Read Temperature Value from Previous RH Measurement */
  measure_T_HMM         = 0xE3, /* Measure Temperature, Hold Master Mode */
  measure_RH_HMM        = 0xE5, /* Measure Relative Humidity, Hold Master Mode */
  write_reg1            = 0xE6, /* Write RH/T User Register 1 */
  read_reg1             = 0xE7, /* Read RH/T User Register 1 */
  measure_T_NHMM        = 0xF3, /* Measure Temperature, No Hold Master Mode */
  measure_RH_NHMM       = 0xF5, /* Measure Relative Humidity, No Hold Master Mode */
  read_id_byte1_1       = 0xFA, /* Read Electronic ID 1st Byte (Checksum byte required) */
  read_id_byte2_1       = 0xFC, /* Read Electronic ID 2nd Byte (Checksum byte required) */
  reset                 = 0xFE  /* Reset */
}SI7021_I2C_COMMAND_Typedef;


typedef struct
{


}SI7021_INIT_Typedef;

//***********************************************************************************
// structs
//***********************************************************************************


//***********************************************************************************
// function prototypes
//***********************************************************************************
void si7021_i2c_open(I2C_TypeDef *i2c);
void si7021_i2c_read(I2C_TypeDef *i2c, uint32_t si7021_cb);
void si7021_i2c_write(I2C_TypeDef *i2c, uint32_t si7021_cb);
float si7021_calc_RH(void);

#endif
