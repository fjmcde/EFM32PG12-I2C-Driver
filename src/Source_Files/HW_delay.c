/***************************************************************************//**
 * @file
 *   HW_delay.c
 * @author
 *   kgraham; edited by Frank McDermott
 * @date
 *   4/19/2020; edited 10/11/2022
 * @brief
 *   Driver to handle hardware delays for use with Si7021 Temperature &
 *   Humidity Sensor
 ******************************************************************************/

//***********************************************************************************
// included header file
//***********************************************************************************
#include "HW_delay.h"


//***********************************************************************************
// static/private data
//***********************************************************************************


//***********************************************************************************
// static/private functions
//***********************************************************************************


//***********************************************************************************
// function definitions
//***********************************************************************************
/***************************************************************************//**
 * @brief
 *  Generates a hardware delay.
 *
 * @details
 *  Initializes and generates a hardware delay using the High-Frequency
 *  Peripheral Clock's frequency.
 *
 * @param[in] ms_delay
 *  Time, in milliseconds, that the delay should last for.
 ******************************************************************************/
void timer_delay(uint32_t ms_delay)
{
  // instantiate local TIMER struct
  TIMER_Init_TypeDef delay_counter_init = TIMER_INIT_DEFAULT;

  // get clock frequency of HFPER CMU clock
	uint32_t timer_clk_freq = CMU_ClockFreqGet(cmuClock_HFPER);

	// calculate a delay
	uint32_t delay_count = ms_delay *(timer_clk_freq/1000) / 1024;

	// enable the TIMER0 CMU clock
	CMU_ClockEnable(cmuClock_TIMER0, true);

	// set init values
	delay_counter_init.oneShot = true;
  delay_counter_init.enable = false;
  delay_counter_init.mode = timerModeDown;
  delay_counter_init.prescale = timerPrescale1024;
  delay_counter_init.debugRun = false;

  // initialize TIMER0
	TIMER_Init(TIMER0, &delay_counter_init);

	// set delay in TIMER0 CNT register
	TIMER0->CNT = delay_count;

	// enable TIMER0
	TIMER_Enable(TIMER0, true);
	while (TIMER0->CNT != 00);

	// disable TIMER0
	TIMER_Enable(TIMER0, false);

	// disable TIMER0 CMU clock
	CMU_ClockEnable(cmuClock_TIMER0, false);
}
