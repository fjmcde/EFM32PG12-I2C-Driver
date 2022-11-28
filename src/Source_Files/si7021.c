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
static volatile uint16_t write_data;

//***********************************************************************************
// static/private functions
//***********************************************************************************
static uint8_t si7021_num_bytes(SI7021_CMD_Typedef cmd, bool checksum);


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
  app_i2c_open.scl_loc = I2C_SCL_ROUTE;
  app_i2c_open.sda_loc = I2C_SDA_ROUTE;
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
void si7021_i2c_read(I2C_TypeDef *i2c, SI7021_CMD_Typedef cmd, bool checksum, uint32_t si7021_cb)
{
  // atomic operation
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();

  // reset read_result
  read_result = RESET_READ_RESULT;

  uint8_t bytes_req;

  // determine how many bytes to request
  bytes_req = si7021_num_bytes(cmd, checksum);

  // initialize local I2C state machine for i2c_start()
  volatile I2C_SM_STRUCT i2c_start_sm;
  i2c_start_sm.I2Cn = i2c;
  i2c_start_sm.curr_state = reqRes;
  i2c_start_sm.slave_addr = SI7021_ADDR;
  i2c_start_sm.read_operation = true;
  i2c_start_sm.rxdata = &i2c->RXDATA;
  i2c_start_sm.txdata = &i2c->TXDATA;
  i2c_start_sm.data = &read_result;
  i2c_start_sm.tx_cmd = ((uint8_t) cmd);
  i2c_start_sm.bytes_req = bytes_req;
  i2c_start_sm.num_bytes = bytes_req;
  i2c_start_sm.i2c_cb = si7021_cb;

  // exit core critical to allow interrupts
  CORE_EXIT_CRITICAL();

  // start I2C protocol
  i2c_init_sm(&i2c_start_sm);

  // transmit start
  i2c_tx_req(&i2c_start_sm, i2cWriteBit);
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
void si7021_i2c_write(I2C_TypeDef *i2c, SI7021_CMD_Typedef cmd, uint8_t ctrl, uint32_t si7021_cb)
{
  // atomic operation
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();

  write_data = ctrl;

  // initialize local I2C state machine for i2c_start()
   volatile I2C_SM_STRUCT i2c_start_sm;
   i2c_start_sm.I2Cn = i2c;
   i2c_start_sm.curr_state = reqRes;
   i2c_start_sm.slave_addr = SI7021_ADDR;
   i2c_start_sm.read_operation = false;
   i2c_start_sm.rxdata = &i2c->RXDATA;
   i2c_start_sm.txdata = &i2c->TXDATA;
   i2c_start_sm.data = &write_data;
   i2c_start_sm.tx_cmd = cmd;
   i2c_start_sm.bytes_req = SI7021_TX_1_BYTE;
   i2c_start_sm.num_bytes = SI7021_TX_1_BYTE;
   i2c_start_sm.i2c_cb = si7021_cb;

   // exit core critical to allow interrupts
   CORE_EXIT_CRITICAL();

   // start I2C protocol
   i2c_init_sm(&i2c_start_sm);

   // transmit start
   i2c_tx_req(&i2c_start_sm, i2cWriteBit);
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


float si7021_calc_temp(void)
{
  // make atomic by disallowing interrupts
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();

  // convert stored temperature code to degrees (°C) (SI7021-A20: 5.1.2)
  float temp = ((175.71 * (float)read_result) / 65536) - 46.85;

  // exit core critical to allow interrupts
  CORE_EXIT_CRITICAL();

  return temp;
}


uint32_t si7021_read_user_reg(void)
{
  // make atomic by disallowing interrupts
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();

  uint32_t data = read_result;

  // exit core critical to allow interrupts
  CORE_EXIT_CRITICAL();

  return data;
}


uint8_t si7021_num_bytes(SI7021_CMD_Typedef cmd, bool checksum)
{
  uint8_t bytes_req;

  switch(cmd)
  {
    case readReg1:
      bytes_req = SI7021_REQ_1_BYTE;
      break;
    case measureRH_NHMM:
      bytes_req = SI7021_REQ_2_BYTES;
      break;
    case MeasureTFromPrevRH:
      bytes_req = SI7021_REQ_2_BYTES;
      break;
    default:
      // Not all of the enumerated Si7021 commands have functionality
      // if this default case is reached then the desired command
      // functionality is not yet completed
      EFM_ASSERT(false);
      break;
  }

  if(checksum)
  {
      bytes_req++;
  }

  return bytes_req;
}
