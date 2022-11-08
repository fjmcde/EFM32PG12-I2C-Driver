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
#define PWM_PER             3.0                       // PWM period in seconds
#define PWM_ACT_PER         0.25                      // PWM active period in seconds
#define LETIMER0_COMP0_CB   0x00000001                // 0b0000 0001
#define LETIMER0_COMP1_CB   0x00000002                // 0b0000 0010
#define LETIMER0_UF_CB      0x00000004                // 0b0000 0100
#define GPIO_ODD_IRQ_CB     0x80                      // 0b1000 0000; unique odd bit for BTN1
#define GPIO_EVEN_IRQ_CB    0x40                      // 0b0100 0000; unique even bit for BTN0
#define SI7021_HUM_READ_CB  0x20                      // 0b0010 0000; unique read bit for Si7021 callback
#define SI7021_WRITE_CB     0x10                      // 0b0001 0000; unique write bit for Si7021 callback
#define RH_LED_ON           30.0                      // Comparison value to determine whether or not to ASSERT LED1

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
void scheduled_letimer0_uf_cb(void);
void scheduled_letimer0_comp0_cb(void);
void scheduled_letimer0_comp1_cb(void);
void scheduled_gpio_even_irq_cb(void);
void scheduled_gpio_odd_irq_cb(void);
void scheduled_si7021_hum_read_cb(void);

#endif
