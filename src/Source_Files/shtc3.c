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
static volatile uint32_t read_result;
static volatile uint32_t write_data;
static volatile uint16_t crc_data;
static volatile float shtc3_rh;
static volatile float shtc3_temp;

//***********************************************************************************
// static/global functions
//***********************************************************************************
static bool check_lock(SHTC3_CMD_Typedef cmd);
static float shtc3_calc_rh(uint16_t data);
static float shtc3_calc_temp(uint16_t data);

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

  /* TODO: configure HW_delay for micro-second delays */
  // timer delay of 1ms (Max required is 240 micro-seconds; DS 3.1)
  timer_delay(1);

  // transmit sleep command
  shtc3_write(I2C1, sleep, SHTC3_SLEEP_CB);
}


void shtc3_write(I2C_TypeDef *i2c, SHTC3_CMD_Typedef cmd, uint32_t shtc3_cb)
{
  // atomic operation
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();

  // reset read_result
  read_result = SHTC3_RESET_READ_RESULT;

  bool lock = check_lock(cmd);

  // initialize local I2C state machine
  volatile I2C_SM_STRUCT i2c_start_sm;
  i2c_start_sm.I2Cn = i2c;
  i2c_start_sm.curr_state = reqRes;
  i2c_start_sm.slave_addr = SHTC3_ADDR;
  i2c_start_sm.read_operation = false;
  i2c_start_sm.rxdata = &i2c->RXDATA;
  i2c_start_sm.txdata = &i2c->TXDATA;
  i2c_start_sm.data = &write_data;
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
  read_result = SHTC3_RESET_READ_RESULT;

  // initialize local I2C state machine
  volatile I2C_SM_STRUCT i2c_start_sm;
  i2c_start_sm.I2Cn = i2c;
  i2c_start_sm.curr_state = dataReq;
  i2c_start_sm.slave_addr = SHTC3_ADDR;
  i2c_start_sm.read_operation = true;
  i2c_start_sm.txdata = &i2c->TXDATA;
  i2c_start_sm.rxdata = &i2c->RXDATA;
  i2c_start_sm.data = &read_result;
  i2c_start_sm.crc_data = &crc_data;
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


void shtc3_parse_measurement_data_RH_first(void)
{
  // manipulate binary shift truncation to split
  // data into MSB (index 1) and LSB (index 0)
  uint16_t split[2];
  split[1] = (((uint32_t)read_result) >> 16);
  split[0] = ((((uint32_t)read_result) << 16) >> 16);

  // calculate measurements
  float rh = shtc3_calc_rh(split[1]);
  float temp = shtc3_calc_temp(split[0]);

  // modify private variables
  shtc3_set_rh(rh);
  shtc3_set_temp(temp);
}


float shtc3_calc_rh(uint16_t data)
{
  // make atomic by disallowing interrupts
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();

  float rh =  (100 * (((float)data) / 65536));

  // exit core critical to allow interrupts
  CORE_EXIT_CRITICAL();

  return rh;
}


float shtc3_calc_temp(uint16_t data)
{
  // make atomic by disallowing interrupts
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();

  float temp =  (175 * ((float)(data) / 65536)) - 45;

  // exit core critical to allow interrupts
  CORE_EXIT_CRITICAL();

  return temp;
}


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


void shtc3_set_rh(float rh)
{
  // atomic operation
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();

  shtc3_rh = rh;

  // exit core critical to allow interrupts
  CORE_EXIT_CRITICAL();
}


void shtc3_set_temp(float temp)
{
  // atomic operation
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();

  shtc3_temp = temp;

  // exit core critical to allow interrupts
  CORE_EXIT_CRITICAL();
}


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
