/***************************************************************************//**
 * @file
 *   letimer.h
 * @author
 *   Frank McDermott
 * @date
 *   11/29/2022
 * @brief
 *   Header file for the LETIMER peripheral
 ******************************************************************************/

#ifndef	LETIMER_HG
#define	LETIMER_HG


//***********************************************************************************
// include files
//***********************************************************************************
// system included files


// Silicon Labs included files
#include "em_letimer.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "em_assert.h"


// developer included files
#include "scheduler.h"
#include "sleep_routines.h"


//***********************************************************************************
// defined macros
//***********************************************************************************
#define LETIMER_HZ		      1000      // utilizing ULFRCO oscillator for LETIMERs
#define LETIMER_EM          EM4       // use ULFRCO, block energy mode 4
#define LETIMER_CNT_RESET   0         // LETIMER counter reset value
#define DEASSERT            0x00      // de-assert PWM idle values
#define COMP0               0x00      // compare0 compare value
#define COMP1               0x01      // compare1 compare value
#define REP0                0x00      // repeat0 set value
#define REP1                0x01      // repeat1 set value
#define REP_PWM_MODE        0x01      // repeat set PWM mode


//***********************************************************************************
// enums
//***********************************************************************************


//***********************************************************************************
// structs
//***********************************************************************************
/*! App specific letimer struct for PWM mode */
typedef struct {
	bool 			  debugRun;			        /// True = keep LETIMER running will halted
	bool 			  enable;				        /// enable the LETIMER upon completion of open
	uint32_t		out_pin_route0;		    /// out 0 route to gpio port/pin
	uint32_t		out_pin_route1;		    /// out 1 route to gpio port/pin
	bool			  out_pin_0_en;		      /// enable out 0 route
	bool			  out_pin_1_en;		      /// enable out 1 route
	float			  period;				        /// period (in seconds)
	float			  active_period;        /// active period (in seconds)
	bool        comp0_irq_enable;     /// enable interrupt on comp0 interrupt
	uint32_t    comp0_cb;             /// comp0 callback register
	bool        comp1_irq_enable;     /// enable interrupt on comp1 interrupt
	uint32_t    comp1_cb;             /// comp1 callback register
	bool        uf_irq_enable;        /// enable interrupt underflow comp0 interrupt
	uint32_t    uf_cb;                /// underflow callback register
} APP_LETIMER_PWM_TypeDef ;


//***********************************************************************************
// function prototypes
//***********************************************************************************
void letimer_pwm_open(LETIMER_TypeDef *letimer, APP_LETIMER_PWM_TypeDef *app_letimer_struct);
void letimer_start(LETIMER_TypeDef *letimer, bool enable);


#endif
