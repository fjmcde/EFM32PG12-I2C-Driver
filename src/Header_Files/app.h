#ifndef APP_HG
#define APP_HG


//***********************************************************************************
// include files
//***********************************************************************************
// system included files


// silicon Labs included files
#include "em_cmu.h"
#include "em_assert.h"

// developer included files
#include "cmu.h"
#include "gpio.h"
#include "letimer.h"
#include "brd_config.h"
#include "scheduler.h"
#include "sleep_routines.h"
#include "si7021.h"
#include "shtc3.h"


//***********************************************************************************
// defined macros
//***********************************************************************************
// Application specific LETIMER0 Macros
#define PWM_PER               3.0         // PWM period in seconds
#define PWM_ACT_PER           0.25        // PWM active period in seconds
// Application specific Si7021 macros
#define RH_LED_ON             30.0        // Relative humidity threshold to assert LED
// Application specific callback macros
/* LETIMER0 call backs */
#define LETIMER0_UF_CB        0x80        // 0b0000 1000 0000; callback for LETIMER0 Underflow callback
/* Si7021 callbacks */
#define SI7021_HUM_READ_CB    0x40        // 0b0000 0100 0000; callback for humidity read
#define SI7021_TEMP_READ_CB   0x20        // 0b0000 0010 0000; callback for temperature read; from previous RH
#define SI7021_WRITE_REG_CB   0x10        // 0b0000 0001 0000; write to user register callback
#define SI7021_READ_REG_CB    0x08        // 0b0000 0000 1000; read from user register callback
/* SHTC3 callbacks */
#define SHTC3_SLEEP_CB        0X04        // 0b0000 0000 0100; sleep callback
#define SHTC3_WAKEUP_CB       0x02        // 0b0000 0000 0010; wakeup callback
#define SHTC3_MEASUREMENT_CB  0x01        // 0b0000 0000 0001; transmit measurement callback
#define SHTC3_READ_REQ_CB     0x800       // 0b1000 0000 0000; read callback
#define SHTC3_OPEN_CB         0x400       // 0b0100 0000 0000; open callback

//***********************************************************************************
// enums
//***********************************************************************************


//***********************************************************************************
// structs
//***********************************************************************************


//***********************************************************************************
// function prototypes
//***********************************************************************************
void app_peripheral_setup(void);
/* LETIMER0 callback functions */
void scheduled_letimer0_uf_cb(void);
/* SI7021 callback functions */
void scheduled_si7021_hum_read_cb(void);
void scheduled_si7021_temp_read_cb(void);
void scheduled_si7021_write_reg_cb(void);
void scheduled_si7021_read_reg_cb(void);
/* SHTC3 callback functions */
void scheduled_shtc3_sleep_cb(void);
void scheduled_shtc3_wakeup_cb(void);
void scheduled_shtc3_measurement_cb(void);
void scheduled_shtc3_read_req_cb(void);

#endif
