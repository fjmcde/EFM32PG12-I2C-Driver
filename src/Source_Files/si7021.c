/***************************************************************************//**
 * @file
 *   si7021.c
 * @author
 *   Frank McDermott
 * @date
 *   11/29/2022
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
static volatile uint32_t si7021_read_result;
static volatile uint32_t si7021_write_data;
static volatile uint16_t si7021_crc_data;
static volatile float si7021_rh;
static volatile float si7021_temp;
static volatile uint8_t si7021_user_reg_data;

//***********************************************************************************
// static/private functions
//***********************************************************************************
static uint8_t req_bytes(uint8_t cmd);
static void si7021_calc_RH(void);
static void si7021_calc_temp(void);

//***********************************************************************************
// function definitions
//***********************************************************************************


/******************************************************************************
 **************************** PERIPHERAL FUNCTIONS ****************************
 ******************************************************************************/


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
void si7021_i2c_open(I2C_TypeDef *i2c,
                     SI7021_CMD_Typedef cmd,
                     SI7021_USER_REG1_CTRL_Typedef ctrl)
{
  // instantiate an app specific I2C
  I2C_OPEN_STRUCT app_i2c_open;

  // set a local delay
  uint32_t delay = SI7021_PU_DELAY_FULL_MAX;

  // Powerup Time (delay) worst case: From VDD ≥ 1.9 V to ready for a
  // conversion, full temperature range (80ms)
  timer_delay(delay);

  // set app specific frequency
  app_i2c_open.freq = I2C_FREQ;
  app_i2c_open.refFreq = SI7021_REFFREQ;

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

  // timer delay of 1ms (Max required is 240 micro-seconds; DS 3.1)
  timer_delay(80);

  // transmit write to user control register
  si7021_i2c_write(I2C0, cmd, ctrl, SI7021_WRITE_REG_CB);
}


/******************************************************************************
 ************************ PUBLIC READ/WRITE FUNCTIONS *************************
 ******************************************************************************/


/***************************************************************************//**
 * @brief
 *  Starts a read transaction over I2C bus.
 *
 * @details
 *  Initializes an I2C state machine and transmits the initial
 *  request packet.
 *
 * @param[in] i2c
 *  Desired I2Cn peripheral (either I2C0 or I2C1).
 *
 * @param[in] cmd
 *  Enumerated command to transmit later in the state machine.
 *
 * @param[in] si7021_cb
 *  Callback event to be scheduled after read transaction is complete.
 ******************************************************************************/
void si7021_i2c_read(I2C_TypeDef *i2c, SI7021_CMD_Typedef cmd, bool checksum, uint32_t si7021_cb)
{
  // atomic operation
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();

  // reset read_result
  si7021_read_result = SI7021_RESET_READ_RESULT;

  // determine how many bytes to request
  uint8_t bytes = req_bytes(cmd);

  // initialize local I2C state machine
  volatile I2C_SM_STRUCT i2c_start_sm;
  i2c_start_sm.I2Cn = i2c;
  i2c_start_sm.curr_state = reqRes;
  i2c_start_sm.slave_addr = SI7021_ADDR;
  i2c_start_sm.read_operation = true;
  i2c_start_sm.rxdata = &i2c->RXDATA;
  i2c_start_sm.txdata = &i2c->TXDATA;
  i2c_start_sm.data = &si7021_read_result;
  i2c_start_sm.crc_data = &si7021_crc_data;
  i2c_start_sm.checksum = checksum;
  i2c_start_sm.tx_cmd = ((uint8_t)cmd);
  i2c_start_sm.bytes_req = bytes;
  i2c_start_sm.bytes_tx = SI7021_TX_1_BYTE;
  i2c_start_sm.num_bytes = bytes;
  i2c_start_sm.i2c_cb = si7021_cb;
  i2c_start_sm.lock_sm = false;

  // exit core critical to allow interrupts
  CORE_EXIT_CRITICAL();

  // start I2C protocol
  i2c_init_sm(&i2c_start_sm);

  // transmit start
  i2c_tx_req(&i2c_start_sm, i2cWriteBit);
}


/***************************************************************************//**
 * @brief
 *  Starts a write transaction over I2C bus.
 *
 * @details
 *
 *
 * @param[in] i2c
 *  Desired I2Cn peripheral (either I2C0 or I2C1).
 *
 * @param[in] cmd
 *  Enumerated command to transmit later in the state machine.
 *
 * @param[in] ctrl
 *  Data to write to the Si7021's user 1 register.
 *
 * @param[in] si7021_cb
 *  Callback event to be scheduled after write transaction is complete
 ******************************************************************************/
void si7021_i2c_write(I2C_TypeDef *i2c, SI7021_CMD_Typedef cmd, uint8_t ctrl, uint32_t si7021_cb)
{
  // atomic operation
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();

  si7021_write_data = ctrl;

  // initialize local I2C state machine for i2c_start()
  volatile I2C_SM_STRUCT i2c_start_sm;
  i2c_start_sm.I2Cn = i2c;
  i2c_start_sm.curr_state = reqRes;
  i2c_start_sm.slave_addr = SI7021_ADDR;
  i2c_start_sm.read_operation = false;
  i2c_start_sm.rxdata = &i2c->RXDATA;
  i2c_start_sm.txdata = &i2c->TXDATA;
  i2c_start_sm.data = &si7021_write_data;
  i2c_start_sm.tx_cmd = cmd;
  i2c_start_sm.bytes_tx = SI7021_TX_1_BYTE;
  i2c_start_sm.num_bytes = SI7021_TX_1_BYTE;
  i2c_start_sm.i2c_cb = si7021_cb;
  i2c_start_sm.lock_sm = false;

  // exit core critical to allow interrupts
  CORE_EXIT_CRITICAL();

  // start I2C protocol
  i2c_init_sm(&i2c_start_sm);

  // transmit start
  i2c_tx_req(&i2c_start_sm, i2cWriteBit);
}


/******************************************************************************
 ***************************** PRIVATE FUNCTIONS ******************************
 ******************************************************************************/


/***************************************************************************//**
 * @brief
 *  Parses the raw relative humidity measurement code received from the Si7021.
 *
 * @details
 *  Calls private functions to calculate percent relative humidity.
 ******************************************************************************/
void si7021_parse_RH_data(void)
{
  // atomic operation
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();

  si7021_calc_RH();

  // exit core critical to allow interrupts
  CORE_EXIT_CRITICAL();
}


/***************************************************************************//**
 * @brief
 *  Parses the raw temperature measurement code received from the Si7021.
 *
 * @details
 *  Calls private functions to calculate temperature Celsius.
 ******************************************************************************/
void si7021_parse_temp_data(void)
{
  // atomic operation
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();

  si7021_calc_temp();

  // exit core critical to allow interrupts
  CORE_EXIT_CRITICAL();
}




/***************************************************************************//**
 * @brief
 *  Converts a raw relative humidity measurement code to percent humidity
 *  (Si7021-A20 TRM: Section 5.1.1)
 *
 * @details
 *  Stores calculated data in private data member for easy access.
 ******************************************************************************/
void si7021_calc_RH(void)
{
  // convert the stored RH code to percent humidity (Si7021-A20: 5.1.1)
  float rh = ((125 * (((float)si7021_read_result) / 65536)) - 6);

  // update static variable
  si7021_rh = rh;

  // delay to avoid RMW errors
  timer_delay(80);
}


/***************************************************************************//**
 * @brief
 *  Converts a raw temperature measurement code to temperature (Celsius).
 *  (Si7021-A20 TRM: Section 5.1.1)
 *
 * @details
 *  Stores calculated data in private data member for easy access.
 ******************************************************************************/
void si7021_calc_temp(void)
{
  // convert stored temperature code to degrees (°C) (SI7021-A20: 5.1.2)
  float temp = ((175.71 * (((float)si7021_read_result) / 65536)) - 46.85);

  // update static variable
  si7021_temp = temp;

  // delay to avoid RMW errors
  timer_delay(80);
}


/***************************************************************************//**
 * @brief
 *  Stores user register data.
 *
 * @details
 *  Stores results of user register 1 read transaction in private data
 *  member, then returns it for visibility in the application layer.
 ******************************************************************************/
uint8_t si7021_store_user_reg(void)
{
  // make atomic by disallowing interrupts
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();

  si7021_user_reg_data = si7021_read_result;

  // exit core critical to allow interrupts
  CORE_EXIT_CRITICAL();

  return si7021_user_reg_data;
}


/***************************************************************************//**
 * @brief
 *  Determines how many bytes an enumerated command should expect.
 *
 * @details
 *  Determines how many bytes an enumerated command should expect, for use
 *  in I2C state machine initialization.
 *
 *  Note:
 *  When additional command functionality is built, cases will be add
 *  to switch statement.
 *
 *  @param[in] cmd
 *   8-bit command used to determine how many bytes to expect.
 *
 *  @param[out] bytes
 *   Returns number of bytes a command should expect
 ******************************************************************************/
uint8_t req_bytes(uint8_t cmd)
{
  uint8_t bytes;

  switch(cmd)
  {
    case measureT_NHMM:
      bytes = SI7021_REQ_3_BYTES;
      break;
    case measureRH_NHMM:
      bytes = SI7021_REQ_3_BYTES;
      break;
    case MeasureTFromPrevRH:
      bytes = SI7021_REQ_3_BYTES;
      break;
    case readReg1:
      bytes = SI7021_REQ_2_BYTES;
      break;
    default:
      // if the default case is reached, the chosen command
      // has not yet been implemented. EFM_ASSERT for debugging.
      EFM_ASSERT(false);
      break;
  }

  return bytes;
}


/******************************************************************************
 ************************* PUBLIC ACCESSOR FUNCTIONS **************************
 ******************************************************************************/


/***************************************************************************//**
 * @brief
 *  Accessor function for privately stored relative humidity data.
 *
 * @details
 *  Provides the application layer with read access to private data members.
 *
 * @param[out]
 *  Returns relative humidity data.
 ******************************************************************************/
float si7021_get_rh()
{
  // atomic operation
    CORE_DECLARE_IRQ_STATE;
    CORE_ENTER_CRITICAL();

    float rh = si7021_rh;

    // exit core critical to allow interrupts
    CORE_EXIT_CRITICAL();

    return rh;
}


/***************************************************************************//**
 * @brief
 *  Accessor function for privately stored temperature data.
 *
 * @details
 *  Provides the application layer with read access to private data members.
 *
 * @param[out]
 *  Returns temperature data.
 ******************************************************************************/
float si7021_get_temp()
{
  // atomic operation
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();

  float temp = si7021_temp;

  // exit core critical to allow interrupts
  CORE_EXIT_CRITICAL();

  return temp;
}
