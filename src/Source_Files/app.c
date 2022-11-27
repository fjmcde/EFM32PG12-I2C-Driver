/***************************************************************************//**
 * @file
 *   app.c
 * @author
 *   Frank McDermott
 * @date
 *   11/06/2022
 * @brief
 *   Driver to configure application-specific peripherals
 ******************************************************************************/

//***********************************************************************************
// included header file
//***********************************************************************************
#include "app.h"


//***********************************************************************************
// defined files
//***********************************************************************************


//***********************************************************************************
// static/private data
//***********************************************************************************


//***********************************************************************************
// static/private functions
//***********************************************************************************
static void app_letimer_pwm_open(float period, float act_period,
                                 uint32_t out0_route, uint32_t out1_route,
                                 bool out0_en, bool out1_en, bool out_en);


//***********************************************************************************
// function definitions
//***********************************************************************************
/***************************************************************************//**
 * @brief
 *   Sets up the application-specific peripherals, schedulers, and timers
 *
 * @details
 *   Opens all application specific peripherals
 ******************************************************************************/
void app_peripheral_setup(void)
{
  cmu_open();
  gpio_open();
  sleep_open();
  scheduler_open();
  app_letimer_pwm_open(PWM_PER, PWM_ACT_PER, PWM_ROUTE_0, PWM_ROUTE_1, false, false, true);
  letimer_start(LETIMER0, true);
  si7021_i2c_open(I2C0);
}


/***************************************************************************//**
 * @brief
 *   Configure LETIMER for PWM mode.
 *
 * @details
 *   Driver which instantiates and opens the LETIMER in PWM mode for the application
 *
 * @param[in] period
 *   Sets the period (in seconds) for the clock
 *
 * @param[in] act_period
 *   Sets the active period (in seconds) for the clock
 *
 * @param[in] out0_route
 *    out0 route to gpio port/pin
 *
 * @param[in] out1_route
 *    out1 route to gpio port/pin
 *
 ******************************************************************************/
void app_letimer_pwm_open(float period, float act_period,
                          uint32_t out0_route, uint32_t out1_route,
                          bool out0_en, bool out1_en, bool out_en)
{
  // instantiate an APP_LETIMER_PWM_TypeDef struct
  APP_LETIMER_PWM_TypeDef letimer_pwm;

  // configure struct
  letimer_pwm.debugRun = false;
  letimer_pwm.enable = out_en;
  letimer_pwm.out_pin_route0 = out0_route;
  letimer_pwm.out_pin_route1 = out1_route;
  letimer_pwm.out_pin_0_en = out0_en;
  letimer_pwm.out_pin_1_en = out1_en;
  letimer_pwm.period = period;
  letimer_pwm.active_period = act_period;
  letimer_pwm.comp0_irq_enable = false;
  letimer_pwm.comp0_cb = LETIMER0_COMP0_CB;
  letimer_pwm.comp1_irq_enable = true;
  letimer_pwm.comp1_cb = LETIMER0_COMP1_CB;
  letimer_pwm.uf_irq_enable = true;
  letimer_pwm.uf_cb = LETIMER0_UF_CB;

  // open letimer for PWM mode
  letimer_pwm_open(LETIMER0, &letimer_pwm);
}


/***************************************************************************//**
 * @brief
 *   Handles the scheduling of the letimer0 underflow call back
 *
 * @details
 *   Removes the underflow call back event from the scheduler then initializes
 *   a read from the Si7021
 ******************************************************************************/
void scheduled_letimer0_uf_cb(void)
{
  // remove LETIMER0 underflow callback even from scheduler
  remove_scheduled_event(LETIMER0_UF_CB);

  // read relative humidity using Si7021
  si7021_i2c_read(I2C0, measure_RH_NHMM, false, SI7021_HUM_READ_CB);
}


/***************************************************************************//**
 * @brief
 *   Handles the scheduling of the letimer0 comp0 call back
 *
 * @note
 *   Currently disabled, as the comp0 interrupt is disabled. The Assert
 *   statement is for debugging. If the assert is reached the configuration
 *   is incorrect.
 *
 * @details
 *   Removes the comp0 call back event from the scheduler then asserts that
 *   the event has been cleared
 ******************************************************************************/
void scheduled_letimer0_comp0_cb(void)
{
  // the LETIMER0 comp0 interrupt is disabled; so if we get here we know that
  // we configured the LETIMER0 incorrectly. FOR DEBUGGING PURPOSES ONLY.
  // REMOVE ASSERT IF INTERRUPT ENABLED.
  EFM_ASSERT(false);

  // added so that we can handle the callback once enabled
  // uncomment next two lines when enabled
  //remove_scheduled_event(LETIMER0_COMP0_CB);
  //EFM_ASSERT(!(get_scheduled_events() & LETIMER0_COMP0_CB));
}


/***************************************************************************//**
 * @brief
 *   Handles the scheduling of the letimer0 comp1 call back
 *
 * @details
 *   Removes the comp1 call back event from the scheduler then
 *   asserts that the event has been cleared
 ******************************************************************************/
void scheduled_letimer0_comp1_cb(void)
{
  remove_scheduled_event(LETIMER0_COMP1_CB);
  EFM_ASSERT(!(get_scheduled_events() & LETIMER0_COMP1_CB));
}


/***************************************************************************//**
 * @brief
 *   Handles the scheduling of the GPIO Odd IRQ (BTN1) call back
 *
 * @details
 *  Removes the triggering event from the scheduler and unblocked the
 *  current energy mode. If the current blocked EM is less than EM4,
 *  increment the current energy mode. Otherwise return EM0 (overflow)
 ******************************************************************************/
void scheduled_gpio_odd_irq_cb(void)
{
  // remove scheduled event from scheduler
  remove_scheduled_event(GPIO_ODD_IRQ_CB);

  // assert to ensure removed
  EFM_ASSERT(!(get_scheduled_events() & GPIO_ODD_IRQ_CB));

  // local variable to store the current blocked energy mode
  uint32_t current_block_em = current_block_energy_mode();

  // unblock current energy mode
  sleep_unblock_mode(current_block_em);

  if(current_block_em < EM4)
  {
    sleep_block_mode(++current_block_em);
  }
  else
  {
      sleep_block_mode(EM0);
  }
}


/***************************************************************************//**
 * @brief
 *   Handles the scheduling of the GPIO Even IRQ (BTN1) call back
 *
 * @details
 *  Removes the triggering event from the scheduler. If the EM is zero,
 *  block EM4 (underflow) otherwise  decrement the current block energy mode.
 ******************************************************************************/
void scheduled_gpio_even_irq_cb(void)
{
  // remove event from scheduler
  remove_scheduled_event(GPIO_EVEN_IRQ_CB);

  // assert to ensure removed
  EFM_ASSERT(!(get_scheduled_events() & GPIO_EVEN_IRQ_CB));

  // local variable to store the current blocked energy mode
  uint32_t current_block_em = current_block_energy_mode();

  // unblock current energy mode
  sleep_unblock_mode(current_block_em);

  // if current block energy mode is > EM0 ...
  if(current_block_em > EM0)
  {
      // ... decrement the current block
      sleep_block_mode(--current_block_em);
  }
  else
  {
      // ... else block EM4
      sleep_block_mode(EM4);
  }
}


/***************************************************************************//**
 * @brief
 *   Handles the scheduling of Si7021 humidity read callback
 *
 * @details
 *  Removes the triggering event from the scheduler and calculates relative
 *  humidity from the stored Si7021's measurement code.
 ******************************************************************************/
void scheduled_si7021_hum_read_cb(void)
{
  // remove event from scheduler
  remove_scheduled_event(SI7021_HUM_READ_CB);

  // asset to ensure removed
  EFM_ASSERT(!(get_scheduled_events() & SI7021_HUM_READ_CB));

  // convert measured value to relative humidity
  float rh = si7021_calc_RH();

  // if relative humidity is greater than 30.0%...
  if(rh >= RH_LED_ON)
  {
      // ... Assert LED1
      GPIO_PinOutSet(LED1_PORT, LED1_PIN);
  }
  else
  {
      // De-assert LED1
      GPIO_PinOutClear(LED1_PORT, LED1_PIN);
  }

  // read temperature from previous previous RH measurement
  si7021_i2c_read(I2C0, read_T_from_prev_RH, false, SI7021_TEMP_READ_CB);
}


void scheduled_si7021_temp_read_cb(void)
{
  remove_scheduled_event(SI7021_TEMP_READ_CB);

  EFM_ASSERT(!(get_scheduled_events() & SI7021_TEMP_READ_CB));

  float temp = si7021_calc_temp();

  printf("Temperature: %3.6f", temp);
}


void scheduled_si7021_write_reg(void)
{
  remove_scheduled_event(SI7021_WRITE_REG_CB);

  EFM_ASSERT(!(get_scheduled_events() & SI7021_WRITE_REG_CB));

  si7021_i2c_read(I2C0, read_reg1, false, SI7021_READ_RED_CB);
}


void scheduled_si7021_read_reg(void)
{
  remove_scheduled_event(SI7021_READ_RED_CB);

  EFM_ASSERT(!(get_scheduled_events() & SI7021_READ_RED_CB));

  uint32_t data = si7021_read_user_reg();

  printf("Data in User Register 1: %lu", data);
}
