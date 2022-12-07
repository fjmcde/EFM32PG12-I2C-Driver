/***************************************************************************//**
 * @file
 *   shtc3.c
 * @author
 *   Frank McDermott
 * @date
 *   11/29/2022
 * @brief
 *   SHTC3 Driver
 ******************************************************************************/

//***********************************************************************************
// included header file
//***********************************************************************************
#include "shtc3.h"


//***********************************************************************************
// static/private data
//***********************************************************************************
static volatile uint32_t shtc3_read_result;
static volatile uint32_t shtc3_write_data;
static volatile uint16_t shtc3_crc_data;
static volatile float shtc3_rh;
static volatile float shtc3_temp;

//***********************************************************************************
// static/global functions
//***********************************************************************************
static bool check_lock(SHTC3_CMD_Typedef cmd);
static uint16_t shtc3_calc_rh(uint16_t data);
static uint16_t shtc3_calc_temp(uint16_t data);

//***********************************************************************************
// function definitions
//***********************************************************************************


/******************************************************************************
 **************************** PERIPHERAL FUNCTIONS ****************************
 ******************************************************************************/

/***************************************************************************//**
 * @brief
 *  Opens SHTC3 peripheral.
 *
 * @details
 *  Opens the I2C peripheral as well as initializes and puts the SHTC3
 *  to sleep.
 *
 * @param[in] i2c
 *  I2C peripheral to use {Can use I2C0 or I2C1).
 ******************************************************************************/
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

  /* TODO: configure HW_delay for micro-second delays */
  // timer delay of 1ms (Max required is 240 micro-seconds; DS 3.1)
  timer_delay(1);

  // transmit sleep command
  shtc3_write(I2C1, sleep, SHTC3_OPEN_CB);
}


/******************************************************************************
 **************************** READ/WRITE FUNCTIONS ****************************
 ******************************************************************************/


/***************************************************************************//**
 * @brief
 *  Starts a write transaction over the I2C bus.
 *
 * @details
 *  initialized an I2C state machine and transmits a write request packet.
 *
 * @param[in] i2c
 *  I2C peripheral to use {Can use I2C0 or I2C1).
 *
 * @param[in] cmd
 *  Enumerated command for transmit to SHTC3.
 *
 * @param[in] shtc3_cb
 *  Callback event to schedule.
 ******************************************************************************/
void shtc3_write(I2C_TypeDef *i2c, SHTC3_CMD_Typedef cmd, uint32_t shtc3_cb)
{
  // atomic operation
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();

  // reset read_result
  shtc3_read_result = SHTC3_RESET_READ_RESULT;

  bool lock = check_lock(cmd);

  // initialize local I2C state machine
  volatile I2C_SM_STRUCT i2c_start_sm;
  i2c_start_sm.I2Cn = i2c;
  i2c_start_sm.curr_state = reqRes;
  i2c_start_sm.slave_addr = SHTC3_ADDR;
  i2c_start_sm.read_operation = false;
  i2c_start_sm.rxdata = &i2c->RXDATA;
  i2c_start_sm.txdata = &i2c->TXDATA;
  i2c_start_sm.data = &shtc3_write_data;
  i2c_start_sm.tx_cmd = ((uint16_t)cmd);
  i2c_start_sm.bytes_req = SHTC3_ZERO_BYTES;
  i2c_start_sm.bytes_tx = SHTC3_TX_2_BYTES;
  i2c_start_sm.num_bytes = SHTC3_TX_2_BYTES;
  i2c_start_sm.i2c_cb = shtc3_cb;
  i2c_start_sm.lock_sm = lock;

  // exit core critical to allow interrupts
  CORE_EXIT_CRITICAL();

  // start I2C protocol
  i2c_init_sm(&i2c_start_sm);

  // transmit start
  i2c_tx_req(&i2c_start_sm, i2cWriteBit);
}


void shtc3_read(I2C_TypeDef *i2c, bool checksum, uint32_t shtc3_cb)
{
  // atomic operation
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();

  // reset read_result
  shtc3_read_result = SHTC3_RESET_READ_RESULT;

  // initialize local I2C state machine
  volatile I2C_SM_STRUCT i2c_start_sm;
  i2c_start_sm.I2Cn = i2c;
  i2c_start_sm.curr_state = dataReq;
  i2c_start_sm.slave_addr = SHTC3_ADDR;
  i2c_start_sm.read_operation = true;
  i2c_start_sm.txdata = &i2c->TXDATA;
  i2c_start_sm.rxdata = &i2c->RXDATA;
  i2c_start_sm.data = &shtc3_read_result;
  i2c_start_sm.crc_data = &shtc3_crc_data;
  i2c_start_sm.checksum = checksum;
  i2c_start_sm.bytes_req = SHTC3_REQ_6_BYTES;
  i2c_start_sm.bytes_tx = SHTC3_ZERO_BYTES;
  i2c_start_sm.num_bytes = SHTC3_REQ_6_BYTES;
  i2c_start_sm.i2c_cb = shtc3_cb;
  i2c_start_sm.lock_sm = false;

  // exit core critical to allow interrupts
  CORE_EXIT_CRITICAL();

  // start I2C protocol
  i2c_init_sm(&i2c_start_sm);

  // Poll for measurement completion
  i2c_tx_req(&i2c_start_sm, i2cReadBit);
}


/******************************************************************************
 **************************** CONVERSION FUNCTIONS ****************************
 ******************************************************************************/


/***************************************************************************//**
 * @brief
 *  Parses the SHTC3 measurement data.
 *
 * @details
 *  The SHTC3 transmits raw measured relative humidity and temperature
 *  codes in a series of four 8-bit packets, for a total of 32-bits of data.
 *  This data is stores in a single unsigned-32-bit integer and must be
 *  split into separate 16-bit values to separate temperature data from RH data.
 *
 *  This private function is used after one of the enumerated "relative humidity
 *  first" commands. The 2-MSBytes are RH data; the 2-LSBytes are temperature data.
 ******************************************************************************/
void shtc3_parse_measurement_data_RH_first(void)
{
  // make atomic by disallowing interrupts
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();

  // manipulate binary shift truncation to split
  // data into MSB (index 1) and LSB (index 0)
  uint16_t split[2];
  split[1] = (((uint32_t)shtc3_read_result) >> 16);
  split[0] = ((((uint32_t)shtc3_read_result) << 16) >> 16);

  // calculate measurements
  float rh = shtc3_calc_rh(split[1]);
  float temp = shtc3_calc_temp(split[0]);

  // modify private variables
  shtc3_set_rh(rh);
  shtc3_set_temp(temp);

  // exit core critical to allow interrupts
  CORE_EXIT_CRITICAL();
}


/******************************************************************************
 ************************* PUBLIC ACCESSOR FUNCTIONS **************************
 ******************************************************************************/


float shtc3_get_rh()
{
  // make atomic by disallowing interrupts
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();

  float rh = shtc3_rh;

  // exit core critical to allow interrupts
  CORE_EXIT_CRITICAL();

  return rh;
}


float shtc3_get_temp()
{
  // make atomic by disallowing interrupts
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();

  float temp = shtc3_temp;

  // exit core critical to allow interrupts
  CORE_EXIT_CRITICAL();

  return temp;
}


/******************************************************************************
 ************************ PUBLIC MODIFIER FUNCTIONS ***************************
 ******************************************************************************/


/***************************************************************************//**
 * @brief
 *  Private function to store percent relative humidity data in private
 *  data member.
 *
 * @details
 *  Stores calculated data in private data member for easy access.
 ******************************************************************************/
void shtc3_set_rh(float rh)
{
  shtc3_rh = rh;

  // delay to avoid RMW errors
  timer_delay(80);
}


/***************************************************************************//**
 * @brief
 *  Private function to store temperature (Celsius) data in private
 *  data member.
 *
 * @details
 *  Stores calculated data in private data member for easy access.
 ******************************************************************************/
void shtc3_set_temp(float temp)
{
  shtc3_temp = temp;

  // delay for RMW errors
  timer_delay(80);
}


/******************************************************************************
 ***************************** PRIVATE FUNCTIONS ******************************
 ******************************************************************************/


/***************************************************************************//**
 * @brief
 *  Converts a raw relative humidity measurement code to percent humidity
 *
 * @details
 *  Stores calculated data in private data member for easy access.
 ******************************************************************************/
uint16_t shtc3_calc_rh(uint16_t data)
{
  // convert raw measurement code to % RH
  float rh =  (100 * (((float)data) / 65536));

  return rh;
}


/***************************************************************************//**
 * @brief
 *  Converts a raw temperature measurement code to temperature (Celsius)
 *
 * @details
 *  Stores calculated data in private data member for easy access.
 ******************************************************************************/
uint16_t shtc3_calc_temp(uint16_t data)
{
  // Convert raw measurement code to temperature (Celsius)
  float temp =  (175 * ((float)(data) / 65536)) - 45;

  return temp;
}



/***************************************************************************//**
 * @brief
 *  Private function which determines whether the I2C state machine requires
 *  locking.
 *
 * @details
 *  Locking refers to whether or not the mStop state, in the state machine,
 *  is allows to reset the bus. Locking is usually reserved for transactions
 *  which require more than one request.
 *
 * @param[in] cmd
 *  Enumerated command to determine whether the I2C state machine requires
 *  locking.
 *
 * @return lock
 *  Returns whether or not a command requires locking the state machine.
 ******************************************************************************/
bool check_lock(SHTC3_CMD_Typedef cmd)
{
  bool lock;

  switch(cmd)
  {
    case sleep:
      lock = false;
      break;
    case wakeup:
      lock = true;
      break;
    case readRHFirst_LPM:
      lock = true;
      break;
    default:
      // if default case is reached, the command has not
      // yet been implemented.
      EFM_ASSERT(false);
      break;
  }

  return lock;
}
