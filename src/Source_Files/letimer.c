/***************************************************************************//**
 * @file
 *   letimer.c
 * @author
 *   Frank McDermott
 * @date
 *   11/29/2022
 * @brief
 *   LETIMER0 driver
 ******************************************************************************/

//***********************************************************************************
// Include files
//***********************************************************************************
#include "letimer.h"


//***********************************************************************************
// static/private data
//***********************************************************************************
static uint32_t scheduled_uf_cb;      // scheduled underflow callback


//***********************************************************************************
// static/private functions
//***********************************************************************************


//***********************************************************************************
// function definitions
//***********************************************************************************
/***************************************************************************//**
 * @brief
 *   Driver to open and set an LETIMER peripheral in PWM mode
 *
 * @details
 * 	 This routine is a low level driver.  The application code calls this function
 * 	 to open one of the LETIMER peripherals for PWM operation to directly drive
 * 	 GPIO output pins of the device and/or create interrupts that can be used as
 * 	 a system "heart beat" or by a scheduler to determine whether any system
 * 	 functions need to be serviced.
 *
 * @note
 *   This function is normally called once to initialize the peripheral and the
 *   function letimer_start() is called to turn-on or turn-off the LETIMER PWM
 *   operation.
 *
 * @param[in] letimer
 *   Pointer to the base peripheral address of the LETIMER peripheral being opened
 *
 * @param[in] app_letimer_struct
 *   Is the STRUCT that the calling routine will use to set the parameters for PWM
 *   operation
 *
 ******************************************************************************/
void letimer_pwm_open(LETIMER_TypeDef *letimer, APP_LETIMER_PWM_TypeDef *app_letimer_struct){
	LETIMER_Init_TypeDef letimer_pwm_values;

	// local variables to track the period of the clock
  unsigned int period_cnt;
  unsigned int period_active_cnt;

	// enable the routed clock to the LETIMER0 peripheral
	CMU_ClockEnable(cmuClock_LETIMER0, true);

	// start letimer
	letimer->CMD = LETIMER_CMD_START;

	// wait for CMD register to sync
	while (letimer->SYNCBUSY);

	// assert that letimer is running
	EFM_ASSERT(letimer->STATUS & LETIMER_STATUS_RUNNING);

  // wait for CMD register to sync
	while(letimer->SYNCBUSY);

	// Reset the Counter to a know value such as 0
	letimer->CNT = LETIMER_CNT_RESET;

	// Initialize letimer for PWM operation
	letimer_pwm_values.bufTop = false;		            // Comp1 will not be used to load comp0, but used to create an on-time/duty cycle
	letimer_pwm_values.comp0Top = true;		            // load comp0 into cnt register when count register underflows enabling continuous looping
	letimer_pwm_values.debugRun = app_letimer_struct->debugRun;
	letimer_pwm_values.enable = app_letimer_struct->enable;
	letimer_pwm_values.out0Pol = DEASSERT;			      // While PWM is not active out, idle is DEASSERTED, 0
	letimer_pwm_values.out1Pol = DEASSERT;			      // While PWM is not active out, idle is DEASSERTED, 0
	letimer_pwm_values.repMode = letimerRepeatFree;	  // Setup letimer for free running for continuous looping
	letimer_pwm_values.ufoa0 = letimerUFOAPwm;		    // Using the HAL documentation, set to PWM mode
	letimer_pwm_values.ufoa1 = letimerUFOAPwm;		    // Using the HAL documentation, set to PWM mode

  // Initialize letimer
	LETIMER_Init(letimer, &letimer_pwm_values);

	// Wait until the CMD register has been synchronized
	while(letimer->SYNCBUSY);

	// set active period and period counter
	period_cnt = app_letimer_struct->period * LETIMER_HZ;
	period_active_cnt = app_letimer_struct->active_period * LETIMER_HZ;

	// set compare registers
	LETIMER_CompareSet(letimer, COMP0, period_cnt);
	LETIMER_CompareSet(letimer, COMP1, period_active_cnt);

	// set repeat mode bits for PWM mode
	LETIMER_RepeatSet(letimer, REP0, REP_PWM_MODE);
	LETIMER_RepeatSet(letimer, REP1, REP_PWM_MODE);

	// set the out route location and pin
	letimer->ROUTELOC0 = app_letimer_struct->out_pin_route0;
	letimer->ROUTEPEN = app_letimer_struct->out_pin_0_en;

	// Clear Interrupt Flags
	letimer->IFC &= ~_LETIMER_IFC_RESETVALUE; // clear all five IFC bits (TRM 20.5.11)

	// Enable Interrupts
  NVIC_EnableIRQ(LETIMER0_IRQn);    // enable NVIC IRQ for LETIMER0

	// enable underflow interrupts
	letimer->IEN |= _LETIMER_IEN_UF_MASK;

	// set schedule underflow callback
	scheduled_uf_cb = app_letimer_struct->uf_cb;

	// if running already ...
  if(letimer->STATUS & LETIMER_STATUS_RUNNING)
  {
      // ... block EM4
      sleep_block_mode(LETIMER_EM);
  }
}



/***************************************************************************//**
 * @brief
 *   Driver to enable the LETIMER
 *
 * @details
 *   Calls LETIMER_Enable and blocks or unblocks sleep mode depending on
 *   weather the call is to enable or disable the LETIMER
 *
 * @note
 *   This function is normally called once to initialize the peripheral and the
 *   function letimer_start() is called to turn-on or turn-off the LETIMER PWM
 *   operation.
 *
 * @param[in] letimer
 *   Pointer to the base address of the LETIMER peripheral
 *
 * @param[in] enable
 *   A bool to enable/disable the LETIMER
 *
 ******************************************************************************/
void letimer_start(LETIMER_TypeDef *letimer, bool enable)
{
  // a call to block letimer from entering EM4 should only be made once,
  // when letimer goes from inactive to active.

  // if letimer is not running and the driver calls to enable it  ...
  if(!(letimer->STATUS & LETIMER_STATUS_RUNNING) && (enable))
  {
    // .. block EM4
    sleep_block_mode(LETIMER_EM);

    LETIMER_Enable(letimer, enable);

    // allow letimer CMD register to sync
    while(letimer->SYNCBUSY);
  }


  // similarly, if letimer is running and the driver calls to disable it ...
  if((letimer->STATUS & LETIMER_STATUS_RUNNING) && !(enable))
  {
    // ... unblock EM4
    sleep_unblock_mode(LETIMER_EM);

    // disable the LETIMER
    LETIMER_Enable(letimer, enable);

    // allow letimer CMD register to sync
    while(letimer->SYNCBUSY);
  }
}

/***************************************************************************//**
 * @brief
 *   Driver to handle all LETIMER0 interrupts
 *
 * @details
 *   Handles various interrupt sources for the LETIMER0 peripheral. Adds
 *   The corresponding event to the event scheduler and asserts the correct
 *   flag has been lowered.
******************************************************************************/
void LETIMER0_IRQHandler(void)
{
  // interrupt flag to store the source interrupt
  uint32_t int_flag;
  int_flag = (LETIMER0->IF) & (LETIMER0->IEN);

  // clear LETIMER0 interrupt flag;
  LETIMER0->IFC = int_flag;

  // handle UF interrupt source
  if(int_flag & LETIMER_IF_UF)
  {
      add_scheduled_event(scheduled_uf_cb);
      // assert to ensure flag is cleared
      EFM_ASSERT(!(LETIMER0->IF & LETIMER_IF_UF));
  }
}
