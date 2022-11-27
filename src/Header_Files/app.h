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


//***********************************************************************************
// defined macros
//***********************************************************************************
// Application specific LETIMER0 Macros
#define PWM_PER             3.0                       // PWM period in seconds
#define PWM_ACT_PER         0.25                      // PWM active period in seconds
// Application specific Si7021 macros
#define RH_LED_ON           30.0                      // Comparison value to determine whether or not to ASSERT LED
// Application specific callback macros
/* LETIMER0 call backs */
#define LETIMER0_COMP0_CB   0x00000001                // 0b0000 0001
#define LETIMER0_COMP1_CB   0x00000002                // 0b0000 0010
#define LETIMER0_UF_CB      0x00000004                // 0b0000 0100
/* GPIO call backs */
#define GPIO_ODD_IRQ_CB     0x80                      // 0b1000 0000; unique odd bit for BTN1
#define GPIO_EVEN_IRQ_CB    0x40                      // 0b0100 0000; unique even bit for BTN0
/* Si7021 Call backs */
#define SI7021_HUM_READ_CB  0x20                      // 0b0010 0000; callback for humidity read
#define SI7021_WRITE_REG_CB 0x10                      // 0b0001 0000; write to user register callback
#define SI7021_TEMP_READ_CB 0x08                      // 0b0000 1000; callback for temperature read; from previous RH
#define SI7021_READ_RED_CB  0x04                      // 0b0000 0100; read from user register callback

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
void scheduled_letimer0_comp0_cb(void);
void scheduled_letimer0_comp1_cb(void);
/* GPIO callback functions */
void scheduled_gpio_even_irq_cb(void);
void scheduled_gpio_odd_irq_cb(void);
/* SI7021 callback functions */
void scheduled_si7021_hum_read_cb(void);
void scheduled_si7021_temp_read_cb(void);
void scheduled_si7021_write_reg(void);
void scheduled_si7021_read_reg(void);

#endif
