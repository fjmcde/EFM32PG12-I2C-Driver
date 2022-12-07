/***************************************************************************//**
 * @file
 *   i2c.h
 * @author
 *   Frank McDermott
 * @date
 *   11/29/2022
 * @brief
 *   Header file for the I2C protocol
 ******************************************************************************/

#ifndef I2C_HG
#define I2C_HG


//***********************************************************************************
// included files
//***********************************************************************************
// system included files
#include <stdbool.h>

// Silicon Labs included files
#include "em_i2c.h"
#include "em_assert.h"

// developer included files
#include "cmu.h"
#include "sleep_routines.h"
//#include "si7021.h"
#include "app.h"
#include "HW_delay.h"


//***********************************************************************************
// defined macros
//***********************************************************************************
/* I2C route configuration */
#define I2C_SCL_ROUTE         APP_I2C_SCL_ROUTE          // SCL PC11: route location #15 (TRM 6.3 pg 75)
#define I2C_SDA_ROUTE         APP_I2C_SDA_ROUTE          // SDA PC10: route location #15 (TRM 6.3 pg 78)
#define I2C_SCL_PEN           I2C_ROUTEPEN_SCLPEN         // SCL PEN is bit 1 (TRM: 16.5.18)
#define I2C_SDA_PEN           I2C_ROUTEPEN_SDAPEN         // SDA PEN is bit 2 (TRM 16.5.18)
/* I2Cn Clock */
#define I2C_FREQ              I2C_FREQ_FAST_MAX           // Max I2C frequency is 4kHz (EFM32PG12 DS 4.1.20.2 & Si7021-A20 DS Table 3)
#define I2C_CLHR_6_3          i2cClockHLRAsymetric        // IC2 CLHR 6:3 (TRM 16.5.1 & EFM32PG12 HAL I2C_ClockHLR_TypeDef enumeration)
/* I2Cn State Machine Bus Busy [busy] */
#define I2C_BUS_READY         false                       // Clear when bus is available
#define I2C_BUS_BUSY          true                        // Set when bus is busy
/* I2C State Machine transit buffer [txdata] */
#define I2C_ADDR_RW_SHIFT     1                           // Left shift addr before or'ing r/w bit
/* I2C data bytes [data] */
#define SHIFT_MSBYTE          8                           // Left shift a byte in data register to accept another byte as LSB
/* I2C Energy Modes */
#define I2C_EM_BLOCK          EM2                         // I2C Cannot go below EM2
/* I2C Timer Delays */
#define I2C_80MS_DELAY        80                          // 80ms Delay for user with Timer delay to avoid RWM sync issues
/* I2C Interrupt masks [IEN] */
#define I2C_IEN_MASK          0x1E0                       // Enable ACK, NACK, RXDATAV and MSTOP interrupt flags
/* Number of bytes requested [bytes_req] */
#define I2C_BYTES_REQ_READ_2  2
#define I2C_BYTES_REQ_READ_3  3


//***********************************************************************************
// enums
//***********************************************************************************
/*! Enumerated Read/Write bits for use in I2C packet construction */
typedef enum
{
  i2cWriteBit             = 0x00, /*! Write bit for header packet (AN10216-01 I2C Manual) */
  i2cReadBit              = 0x01  /*! Read bit for header packet (AN10216-01 I2C Manual) */
}I2C_RW_Typedef;


/*! Enumerated I2C state machine states */
typedef enum
{
  reqRes,          /*! Request resource: Send 7-bit slave addr + r/w-bit (TRM 16.3.7.6: 0x57) */
  commandTx,       /*! Transmit command to device (TRM 16.3.7.6: 0x97)*/
  dataReq,         /*! Send data request  (TRM 16.3.7.6: 0xD7) */
  dataRx,          /*! Data received (TRM 16.3.7.6)*/
  mStop,           /*! STOP bit sent */
}I2C_STATES_Typedef;

//***********************************************************************************
// structs
//***********************************************************************************
/*! Struct for use in opening the I2C peripheral */
typedef struct
{
  bool                  enable;   /// enable I2C peripheral when init complete
  bool                  master;   /// set to master (true) or Slave (false)
  uint32_t              refFreq;  /// I2C reference clock assumed when configuring bus frequency setup
  uint32_t              freq;     /// max I2C bus frequency
  I2C_ClockHLR_TypeDef  clhr;     /// clock low/high ratio control
  uint32_t              scl_loc;  /// SCL route to GPIO port/pin
  uint32_t              sda_loc;  /// SDA route to GPIO port/pin
  uint32_t              scl_pen;  /// enable SCL pin
  uint32_t              sda_pen;  /// enable SDA pin
}I2C_OPEN_STRUCT;


/*! Struct for managing the I2C state machine. Instantiated as a pair
 of private data members (one for I2C0 and one for I2C1)              */
typedef struct
{
    I2C_TypeDef                  *I2Cn;                   /// pointer to I2C peripheral (I2C0 or I2C1)
    I2C_STATES_Typedef            curr_state;             /// tracks the current state of the state machine
    uint32_t                      slave_addr;             /// pointer to the address of slave device currently being communicated with
    bool                          read_operation;         /// True = Read operation; False = Write operation
    volatile bool                 busy;                   /// True when bus is busy; False when bus is available
    volatile const uint32_t      *rxdata;                 /// pointer to I2C receive buffer address
    volatile uint32_t            *txdata;                 /// pointer to I2C transmit buffer address
    volatile uint32_t            *data;                   /// pointer to static data variable
    volatile uint16_t            *crc_data;               /// pointer to static checksum variable
    bool                          checksum;               /// True = checksum desired; False = ignore checksum
    uint32_t                      tx_cmd;                 /// command to transmit over I2C
    uint8_t                       bytes_req;              /// number of bytes requested
    uint8_t                       bytes_tx;               /// number of bytes to transmit
    uint32_t                      num_bytes;              /// number of bytes remaining
    uint32_t                      i2c_cb;                 /// I2C call back event to request upon completion of I2C operation
    bool                          lock_sm;                /// True = lock the state machine for addition commands; False = unlock; all commands sent
}I2C_SM_STRUCT;


//***********************************************************************************
// function prototypes
//***********************************************************************************
void i2c_open(I2C_TypeDef *i2c, I2C_OPEN_STRUCT *app_i2c_struct);
void i2c_init_sm(volatile I2C_SM_STRUCT *i2c_sm);
void i2c_tx_req(volatile I2C_SM_STRUCT *i2c_sm, I2C_RW_Typedef rw);

#endif
