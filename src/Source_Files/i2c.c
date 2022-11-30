/***************************************************************************//**
 * @file
 *   i2c.c
 * @author
 *   Frank McDermott
 * @date
 *   11/29/2022
 * @brief
 *   I2C Protocol driver
 ******************************************************************************/

//***********************************************************************************
// included header file
//***********************************************************************************
#include "i2c.h"


//***********************************************************************************
// static/private data
//***********************************************************************************
static volatile I2C_SM_STRUCT i2c0_sm;
static volatile I2C_SM_STRUCT i2c1_sm;


//***********************************************************************************
// static/private functions
//***********************************************************************************
/* I2C bus functions */
static void i2c_bus_reset(I2C_TypeDef *i2c);
/* Interrupt driven static state machine functions */
static void i2cn_ack_sm(volatile I2C_SM_STRUCT *i2c_sm);
static void i2cn_nack_sm(volatile I2C_SM_STRUCT *i2c_sm);
static void i2cn_rxdata_sm(volatile I2C_SM_STRUCT *i2c_sm);
static void i2cn_mstop_sm(volatile I2C_SM_STRUCT *i2c_sm);
/* static transmission functions */
static void tx_cmd_msb(volatile I2C_SM_STRUCT *i2c_sm);
static uint8_t i2c_split_tx(volatile uint32_t *cmd);
static void i2c_tx_ack(volatile I2C_SM_STRUCT *i2c_sm);
static void i2c_tx_nack(volatile I2C_SM_STRUCT *i2c_sm);
static void i2c_tx_cont(volatile I2C_SM_STRUCT *i2c_sm);
static void i2c_tx_stop(volatile I2C_SM_STRUCT *i2c_sm);
static void i2c_tx_cmd(volatile I2C_SM_STRUCT *i2c_sm, uint32_t tx_cmd);


//***********************************************************************************
// function definitions
//***********************************************************************************

/******************************************************************************
 **************************** PERIPHERAL FUNCTIONS ****************************
 ******************************************************************************/


/***************************************************************************//**
 * @brief
 *  Opens the I2C peripheral.
 *
 * @details
 *  Opens and initialized the requested I2C peripheral and enables the
 *  proper CMU clock.
 *
 * @param[in] i2c
 *  Desired I2Cn peripheral (either I2C0 or I2C1)
 *
 * @param[in] app_i2c_open
 *  All data required to open the I2C peripheral encapsulated in struct
 ******************************************************************************/
void i2c_open(I2C_TypeDef *i2c, I2C_OPEN_STRUCT *app_i2c_open)
{
  // instantiate a local I2C_Init struct
  I2C_Init_TypeDef i2c_init_values;

  // if the address of i2c is equal to the base address of the
  // I2C0 base peripheral ...
  if(i2c == I2C0)
  {
      // ... enable I2C0 clock
      CMU_ClockEnable(cmuClock_I2C0, true);
  }

  // if the address of i2c is equal to the base address of the
  // I2C1 base peripheral ...
  if(i2c == I2C1)
  {
      // ... enable I2C1 clock
      CMU_ClockEnable(cmuClock_I2C1, true);
  }

  // if START interrupt flag not set ...
  if(!(i2c->IF & I2C_IFS_START))
  {
      // .. set the START interrupt flag
      i2c->IFS = I2C_IFS_START;
  }
  // .. else ...
  else
  {
      // clear START flag
      i2c->IFC = I2C_IFC_START;
  }

  // set values for I2C_Init
  i2c_init_values.enable = app_i2c_open->enable;
  i2c_init_values.master = app_i2c_open->master;
  i2c_init_values.freq = app_i2c_open->freq;
  i2c_init_values.refFreq = app_i2c_open->refFreq;
  i2c_init_values.clhr = app_i2c_open->clhr;

  // initialize I2C peripheral
  I2C_Init(i2c, &i2c_init_values);

  // set route location for SDA and SCL
  i2c->ROUTELOC0 |= app_i2c_open->sda_loc;
  i2c->ROUTELOC0 |= app_i2c_open->scl_loc;

  // enable pin route
  i2c->ROUTEPEN |= app_i2c_open->sda_pen;
  i2c->ROUTEPEN |= app_i2c_open->scl_pen;

  // reset the I2C bus
  i2c_bus_reset(i2c);
}


/******************************************************************************
 *********************** STATE MACHINE INIT FUNCTIONS *************************
 ******************************************************************************/


/***************************************************************************//**
 * @brief
 *  Initializes an I2C state machine.
 *
 * @details
 *  Initializes and starts the requested I2C state machine.
 *
 * @param[in] i2c_sm
 *  Pointer to desired I2C state machine, which has previously been
 *  initialized.
 ******************************************************************************/
void i2c_init_sm(volatile I2C_SM_STRUCT *i2c_sm)
{
  // the I2C peripheral cannot cannot go below EM2
  sleep_block_mode(I2C_EM_BLOCK);

  // set busy bit
  i2c_sm->busy = I2C_BUS_BUSY;

  // enable interrupts
  i2c_sm->I2Cn->IEN = I2C_IEN_MASK;

  // if starting the I2C0 peripheral ...
  if(i2c_sm->I2Cn == I2C0)
  {
      // halt until bus is ready
      while(i2c0_sm.busy);

      // make atomic by disallowing interrupts
      CORE_DECLARE_IRQ_STATE;
      CORE_ENTER_CRITICAL();

      // initialize I2C0 state machine
      i2c0_sm = *i2c_sm;

      // exit core critical to allow interrupts
      CORE_EXIT_CRITICAL();
      CORE_EXIT_CRITICAL();

      NVIC_EnableIRQ(I2C0_IRQn);
  }

  // if starting the I2C1 peripheral ...
  if(i2c_sm->I2Cn == I2C1)
  {
      // halt until bus is ready
      while(i2c1_sm.busy);

      // make atomic by disallowing interrupts
      CORE_DECLARE_IRQ_STATE;
      CORE_ENTER_CRITICAL();

      // initialize I2C1 state machine
      i2c1_sm = *i2c_sm;

      // exit core critical to allow interrupts
      CORE_EXIT_CRITICAL();
      CORE_EXIT_CRITICAL();

      NVIC_EnableIRQ(I2C1_IRQn);
  }

  // 80ms timer delay to ensure RWM sync
  timer_delay(I2C_80MS_DELAY);
}


/******************************************************************************
 ****************************** PUBLIC FUNCTIONS ******************************
 ******************************************************************************/


/***************************************************************************//**
 * @brief
 *  Transmits a request packet.
 *
 * @details
 *  A request packet is made up of:
 *  - The 7-bit address of the slave device to request from.
 *  - A single read/write bit. Per the I2C protocol: Read = 1; Write = 0;
 *
 * @param[in] i2c_sm
 *  Pointer to desired I2C state machine, which has previously been
 *  initialized.
 *
 *  @param[in] rw
 *   Enumerated Read or Write bit.
 ******************************************************************************/
void i2c_tx_req(volatile I2C_SM_STRUCT *i2c_sm, I2C_RW_Typedef rw)
{
  // send start bit
  i2c_sm->I2Cn->CMD = I2C_CMD_START;

  // construct 8-bit read/write header packet.
  // 7 MSB = slave device's address
  // LSB   =  read/write bit
  uint32_t req_packet = ((i2c_sm->slave_addr << 1) | rw);

  // transmit header packet
  *i2c_sm->txdata = req_packet;
}



/******************************************************************************
 ****************************** STATIC FUNCTIONS ******************************
 ******************************************************************************/


/***************************************************************************//**
 * @brief
 *  Transmits the MSByte of a 16-bit command.
 *
 * @details
 *  Because the I2CS protocol transmits data in single-byte packets,
 *  a 16-bit command must be split into two packets.
 *
 *  Splits off the MSByte from a 16-bit command and transmits it.
 *
 * @param[in] i2c_sm
 *  Pointer to desired I2C state machine, which has previously been
 *  initialized.
 ******************************************************************************/
static void tx_cmd_msb(volatile I2C_SM_STRUCT *i2c_sm)
{
  // decrement transmit bytes
  i2c_sm->bytes_tx--;

  // split the tx_cmd
  uint8_t tx =  i2c_split_tx(&i2c_sm->tx_cmd);

  // transmit MSB of tx_cmd
  i2c_tx_cmd(i2c_sm, tx);
}


/***************************************************************************//**
 * @brief
 *  Splits a 16-bit command into two 8-bit commands.
 *
 * @details
 *  Makes use of binary truncation during under/overflow to isolate the MSByte
 *  and LSByte of a 16-bit command. The MSByte is returned so that it can be
 *  transmitted immediately after, while the LSByte is stored back in the
 *  register it the command pointed to to be transmitted later on.
 *
 * @param[in] i2c_sm
 *  Pointer to desired I2C state machine, which has previously been
 *  initialized.
 *
 * @param[out] tx
 *  Returns the isolated MSByte
 *
 ******************************************************************************/
static uint8_t i2c_split_tx(volatile uint32_t *cmd)
{
  uint8_t tx[2];

  // manipulate binary shift truncation to split
  // command into MSB (index 0) and LSB (index 1)
  tx[0] = (((uint16_t)*cmd) >> 8);
  tx[1] = ((((uint16_t)*cmd) << 8) >> 8);

  // store the LSB
  *cmd = tx[1];

  // return the MSB
  return tx[0];
}


/***************************************************************************//**
 * @brief
 *  Transmits an ACK to slave device.
 *
 * @details
 *  Sets the ACK bit in the CMD register of the I2C peripheral that the
 *  current state machine is using.
 *
 * @param[in] i2c_sm
 *  Pointer to desired I2C state machine, which has previously been
 *  initialized.
 ******************************************************************************/
void i2c_tx_ack(volatile I2C_SM_STRUCT *i2c_sm)
{
  // set ACK bit in CMD register
  i2c_sm->I2Cn->CMD = I2C_CMD_ACK;
}


/***************************************************************************//**
 * @brief
 *  Transmits an NACK to slave device.
 *
 * @details
 *  Sets the NACK bit in the CMD register of the I2C peripheral that the
 *  current state machine is using.
 *
 * @param[in] i2c_sm
 *  Pointer to desired I2C state machine, which has previously been
 *  initialized.
 ******************************************************************************/
void i2c_tx_nack(volatile I2C_SM_STRUCT *i2c_sm)
{
  // set NACK bit in CMD register
  i2c_sm->I2Cn->CMD = I2C_CMD_NACK;
}


/***************************************************************************//**
 * @brief
 *  Transmits an CONT to slave device.
 *
 * @details
 *  Sets the CONT bit in the CMD register of the I2C peripheral that the
 *  current state machine is using.
 *
 * @param[in] i2c_sm
 *  Pointer to desired I2C state machine, which has previously been
 *  initialized.
 ******************************************************************************/
void i2c_tx_cont(volatile I2C_SM_STRUCT *i2c_sm)
{
  // set CMD CONT register
  i2c_sm->I2Cn->CMD = I2C_CMD_CONT;
}


/***************************************************************************//**
 * @brief
 *  Transmits a STOP to slave device.
 *
 * @details
 *  Sets the STOP bit in the CMD register of the I2C peripheral that the
 *  current state machine is using.
 *
 * @param[in] i2c_sm
 *  Pointer to desired I2C state machine, which has previously been
 *  initialized.
 ******************************************************************************/
void i2c_tx_stop(volatile I2C_SM_STRUCT *i2c_sm)
{
  // set stop bit in I2C CMD register
  i2c_sm->I2Cn->CMD = I2C_CMD_STOP;
}


/***************************************************************************//**
 * @brief
 *  Transmits an 8-bit command to the slave device.
 *
 * @details
 *  Transmits an 8-bit command to the slave device by writing data to the
 *  I2C state machine's txdata register - which points to the I2C peripheral
 *  RXDATA register.
 *
 * @param[in] i2c_sm
 *  Pointer to desired I2C state machine, which has previously been
 *  initialized.
 *
 * @param[in] tx_cmd
 *  Command to transmit over I2C bus.
 ******************************************************************************/
void i2c_tx_cmd(volatile I2C_SM_STRUCT *i2c_sm, uint32_t tx_cmd)
{
  // transmit command via TXDATA
  *i2c_sm->txdata = tx_cmd;
}


/***************************************************************************//**
 * @brief
 *  Resets the I2C Bus
 *
 * @details
 *  A reset is achieved by aborting any current operations on the I2C bus to
 *  for the bus to go idle, saving the state of the IEN register, disabling
 *  all interrupts, clearing all interrupt flags, clearing the transmit
 *  buffer and MSTOP bit, sending a START and STOP command, and finally
 *  restoring the state of the IEN register.
 *
 * @param[in] i2c
 *  Desired I2Cn peripheral (either I2C0 or I2C1)
 ******************************************************************************/
void i2c_bus_reset(I2C_TypeDef *i2c)
{
  // local variable to save the state of the IEN register
  uint32_t ien_state;

  // abort current transmission to make bus go idle (TRM 16.5.2)
  i2c->CMD = I2C_CMD_ABORT;

  // save state of IEN register
  ien_state = i2c->IEN;

  // disable all interrupts (16.5.17)
  i2c->IEN = _I2C_IEN_RESETVALUE;

  // clear IFC register (TRM 16.5.16)
  i2c->IFC = _I2C_IFC_MASK;

  // clear the transmit buffer (16.5.2)
  i2c->CMD = I2C_CMD_CLEARTX;

  // clear MSTOP bit prior to bus reset
  i2c->IFC |= I2C_IFC_MSTOP;

  // bus reset (TRM 16.3.12.2)
  i2c->CMD = (I2C_CMD_START | I2C_CMD_STOP);

  // ensure reset occurred properly
  while(!(i2c->IF & I2C_IF_MSTOP));

  // clear IFC register (TRM 16.5.16)
  // clear any bits that may have been generated by START/STOP
  i2c->IFC = _I2C_IFC_MASK;

  // reset I2C peripheral by setting ABORT bit in CMD register
  i2c->CMD = I2C_CMD_ABORT;

  // restore IEN register
  i2c->IEN = ien_state;
}


/******************************************************************************
 ***************************** INTERRUPT HANDLERS *****************************
 ******************************************************************************/


/***************************************************************************//**
 * @brief
 *  I2C0 peripheral IRQ Handler
 *
 * @details
 *  Handles ACK, NACK, RXDATAV, and MSTOP interrupts for the I2C0 peripheral
 ******************************************************************************/
void I2C0_IRQHandler(void)
{
  // save flags that are both enabled and raised
  uint32_t intflags = (I2C0->IF & I2C0->IEN);

  // lower flags
  I2C0->IFC = intflags;

  // handle ACK
  if(intflags & I2C_IF_ACK)
  {
      i2cn_ack_sm(&i2c0_sm);
  }

  // handle NACK
  if(intflags & I2C_IF_NACK)
  {
      i2cn_nack_sm(&i2c0_sm);
  }

  // handle RXDATAV
  if(intflags & I2C_IF_RXDATAV)
  {
      i2cn_rxdata_sm(&i2c0_sm);
  }

  // handle MSTOP
  if(intflags & I2C_IF_MSTOP)
  {
      i2cn_mstop_sm(&i2c0_sm);
  }
}


/***************************************************************************//**
 * @brief
 *  I2C1 peripheral IRQ Handler
 *
 * @details
 *  Handles ACK, NACK, RXDATAV, and MSTOP interrupts for the I2C1 peripheral
 ******************************************************************************/
void I2C1_IRQHandler(void)
{
  // save flags that are both enabled and raised
  uint32_t intflags = (I2C1->IF & I2C1->IEN);

  // lower flags
  I2C1->IFC = intflags;

  // handle ACK
  if(intflags & I2C_IF_ACK)
  {
    i2cn_ack_sm(&i2c1_sm);
  }

  // handle NACK
  if(intflags & I2C_IF_NACK)
  {
      i2cn_nack_sm(&i2c1_sm);
  }

  // handle RXDATA
  if(intflags & I2C_IF_RXDATAV)
  {
      i2cn_rxdata_sm(&i2c1_sm);
  }

  // handle MSTOP
  if(intflags & I2C_IF_MSTOP)
  {
      i2cn_mstop_sm(&i2c1_sm);
  }
}


/******************************************************************************
 ************************** STATE MACHINE FUNCTIONS ***************************
 ******************************************************************************/


/***************************************************************************//**
 * @brief
 *  I2C ACK state machine
 *
 * @details
 *  State machine function for an ACK interrupt. Handles ACKs for the
 *  Request Resource, Command Transmit, Data Request, and Data Received states.
 *
 * @param[in] i2c_sm
 *  Pointer to desired I2C state machine, which has previously been
 *  initialized.
 ******************************************************************************/
void i2cn_ack_sm(volatile I2C_SM_STRUCT *i2c_sm)
{
  // make atomic by disallowing interrupts
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();

  switch(i2c_sm->curr_state)
  {
    case reqRes:
      // if zero bytes are expected to transmit ...
      if(i2c_sm->bytes_tx == 0)
      {
          // ... and the initial operation was a read ...
          if(i2c_sm->read_operation)
          {
              // SHTC3 READ
              // change states
              i2c_sm->curr_state = dataRx;
              break;
          }
          else
          {
            // a write operation where zero bytes are expected
            // to transmit is a logic error. EFM_ASSERT For debugging.
            EFM_ASSERT(false);
          }
      }
      // ... else if one byte is expected to transmit ...
      else if(i2c_sm->bytes_tx == 1)
      {
          // SI7021 READ OR WRITE
          // transmit command
          i2c_tx_cmd(i2c_sm, i2c_sm->tx_cmd);
      }
      // .. otherwise if two bytes are expected to transmit ...
      else if(i2c_sm->bytes_tx == 2)
      {
          // ... and the initial operation was a write
          if(!(i2c_sm->read_operation))
          {
              // SHTC3 WRITE
              // transmit MSB of tx_cmd
              tx_cmd_msb(i2c_sm);

              // copy LSB of tx_cmd to the data register
              *i2c_sm->data = i2c_sm->tx_cmd;

          }
          else
          {
              // a read with 2 bytes expected to transmit is a
              // logic error. EFM_ASSERT for debugging.
              EFM_ASSERT(false);
          }
      }
      else
      {
          // a read or write expecting more than two bytes to transmit
          // is a logic error. EFM_ASSERT for debugging.
          EFM_ASSERT(false);
      }

      // change state
      i2c_sm->curr_state = commandTx;
      break;


    case commandTx:
      // if initial operation was a read ...
      if(i2c_sm->read_operation)
      {
          // send repeated start command
          i2c_tx_req(i2c_sm, i2cReadBit);

          // change state
          i2c_sm->curr_state = dataReq;
      }
      // ... else ...
      else
      {
        // ... write data to transmit buffer
        i2c_tx_cmd(i2c_sm, *i2c_sm->data);

        // change state
        i2c_sm->curr_state = dataRx;
      }
      break;


    case dataReq:
      // change state
      i2c_sm->curr_state = dataRx;
      break;


    case dataRx:
      // if initial operation was a write ...
      if(!(i2c_sm->read_operation))
      {
          // send stop
          i2c_tx_stop(i2c_sm);

          // change state
          i2c_sm->curr_state = mStop;
      }
      // ... else read operation ...
      else
      {
          // entering this else indicates an error in logic.
          // in the dataRx state a read request will have already been
          // acknowledged and the slave should NOT ACK but instead RXDATAV.
          EFM_ASSERT(false);
      }
      break;


    default:
      // entering this else indicates an error in logic. An ACK has been
      // received in an unexpected state. EFM_ASSERT for debugging.
      EFM_ASSERT(false);
      break;
  }

  // 80ms timer delay for RWM sync
  timer_delay(80);

  // exit core critical to allow interrupts
  CORE_EXIT_CRITICAL();
}


/***************************************************************************//**
 * @brief
 *  I2C NACK state machine
 *
 * @details
 *  State machine function for a NACK interrupt. Handles NACKs for the
 *  Request Resource, Command Transmit, and Data Request states.
 *
 * @param[in] i2c_sm
 *  Pointer to desired I2C state machine, which has previously been
 *  initialized.
 ******************************************************************************/
void i2cn_nack_sm(volatile I2C_SM_STRUCT *i2c_sm)
{
  // make atomic by disallowing interrupts
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();

  switch(i2c_sm->curr_state)
  {
    case reqRes:
      if(i2c_sm->num_bytes > 3)
      {
          // change state;
          i2c_sm->curr_state = mStop;

          // transmit stop
          i2c_tx_stop(i2c_sm);
          break;
      }

      // send repeated start command
      i2c_tx_req(i2c_sm, i2cWriteBit);
      break;


    case commandTx:
      // set CMD CONT register
      i2c_tx_cont(i2c_sm);

      // re-send command
      i2c_tx_cmd(i2c_sm, i2c_sm->tx_cmd);
      break;


    case dataReq:
      if(i2c_sm->read_operation)
      {
          // re-send repeated start command
          i2c_tx_req(i2c_sm, i2cReadBit);
      }
      else
      {
          if(i2c_sm->num_bytes == 2)
          {
              // change state
              i2c_sm->curr_state = commandTx;
          }
          else
          {
              i2c_tx_req(i2c_sm, i2cWriteBit);
          }
      }
      break;


    default:
      // entering this else indicates an error in logic. An ACK has been
      // received in an unexpected state. EFM_ASSERT for debugging.
      EFM_ASSERT(false);
  }

  // 80ms timer delay for RWM sync
  timer_delay(I2C_80MS_DELAY);

  // exit core critical to allow interrupts
  CORE_EXIT_CRITICAL();
}


/***************************************************************************//**
 * @brief
 *  I2C RXDATA state machine
 *
 * @details
 *  State machine function for an RXDATAV interrupt. Handles RXDATAVs for
 *  the Data Request state.
 *
 * @param[in] i2c_sm
 *  Pointer to desired I2C state machine, which has previously been
 *  initialized.
 ******************************************************************************/
void i2cn_rxdata_sm(volatile I2C_SM_STRUCT *i2c_sm)
{
  // make atomic by disallowing interrupts
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();

  switch(i2c_sm->curr_state)
  {
    case dataRx:
      // if initial operation was a read ...
      if(i2c_sm->read_operation)
      {
          // on checksum byte (byte 4 or byte 1) ...
          if(((i2c_sm->num_bytes - 1) % 3) == 0)
          {
              // ... if checksum requested ...
              if(i2c_sm->checksum)
              {
                  // ... store checksum data in crc_data (shifted n bits)
                  *i2c_sm->crc_data |= (*i2c_sm->rxdata << (8 * (i2c_sm->num_bytes - 1) % 2));
              }
              // ... else ignore checksum (byte 4) ...
              else if(i2c_sm->num_bytes == 4)
              {
                  // read rxdata register to clear it (faster than NACK)
                  if(*i2c_sm->rxdata == false){};
              }
          }
          // else on measurement bytes (Byte 6, 5, 3, and 2)
          else
          {
              /* TODO: Come up with more elegant solution
               *       to calculate the shift */
              uint8_t shift;
              switch(i2c_sm->num_bytes)
              {
                case(6):
                  shift = 24;
                  break;
                case(5):
                  shift = 16;
                  break;
                case(3):
                  shift = 8;
                  break;
                case(2):
                  shift = 0;
                  break;
              }

              // store measurement data in read_result (shifted n bits)
              *i2c_sm->data |= (*i2c_sm->rxdata << shift);
          }

          // decrement num_bytes counter
          i2c_sm->num_bytes--;


          // if checksum was requested ...
          if(i2c_sm->checksum)
          {
              // ... NACK AFTER last CRC byte
              if(i2c_sm->num_bytes == 0)
              {
                  // transmit NACK
                  i2c_tx_nack(i2c_sm);
              }
              // ... else NACK after last byte
              else
              {
                  // transmit ACK
                  i2c_tx_ack(i2c_sm);
                  break;
              }

          }
          // if checksum not requested, NACK BEFORE last CRC byte ...
          else if(i2c_sm->num_bytes == 1)
          {
              // transmit NACK
              i2c_tx_nack(i2c_sm);
          }
          // ... else ACK all other bytes
          else
          {
              // transmit ACK
              i2c_tx_ack(i2c_sm);
              break;
          }

          // send stop
          i2c_tx_stop(i2c_sm);

          //change state
          i2c_sm->curr_state = mStop;

      }
      // ... else write ...
      else
      {
          // entering this else indicates an error in logic
          // a write should NOT generate an RXDATAV interrupt
          EFM_ASSERT(false);
      }
      break;


    default:
      // entering this else indicates an error in logic. An ACK has been
      // received in an unexpected state. EFM_ASSERT for debugging.
      EFM_ASSERT(false);
      break;
  }

  // 80ms timer delay for RWM sync
  timer_delay(80);

  // exit core critical to allow interrupts
  CORE_EXIT_CRITICAL();
}


/***************************************************************************//**
 * @brief
 *  I2C MSTOP state machine
 *
 * @details
 *  State machine function for an MSTOP. Handles MSTOPs for the MSTOP state.
 *  Since this is the end of an I2C transaction this function also releases
 *  the bus (unless a state machine has a hold), unblocks EM2, and
 *  schedules callbacks.
 *
 * @param[in] i2c_sm
 *  Pointer to desired I2C state machine, which has previously been
 *  initialized.
 ******************************************************************************/
void i2cn_mstop_sm(volatile I2C_SM_STRUCT *i2c_sm)
{
  // make atomic by disallowing interrupts
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();

  switch(i2c_sm->curr_state)
  {
    case mStop:
      if(!(i2c_sm->lock_sm))
      {
          // reset the I2C bus
          i2c_bus_reset(i2c_sm->I2Cn);
      }

      // clear I2C State Machine busy bit
      i2c_sm->busy = I2C_BUS_READY;

      // schedule device call back
      add_scheduled_event(i2c_sm->i2c_cb);

      // unblock sleep
      sleep_unblock_mode(I2C_EM_BLOCK);
      break;

    default:
      // entering this else indicates an error in logic. An ACK has been
      // received in an unexpected state. EFM_ASSERT for debugging.
      EFM_ASSERT(false);
  }

  // 80ms timer delay for RWM sync
  timer_delay(I2C_80MS_DELAY);

  // exit core critical to allow interrupts
  CORE_EXIT_CRITICAL();
}
