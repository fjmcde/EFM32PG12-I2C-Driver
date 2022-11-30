/***************************************************************************//**
 * @file
 *   app.c
 * @author
 *   Frank McDermott
 * @date
 *   11/29/2022
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
static float app_si7021_rh;
static float app_si7021_temp;
static uint8_t app_si7021_user_reg;
static float app_shtc3_rh;
static float app_shtc3_temp;

//***********************************************************************************
// static/private functions
//***********************************************************************************
static void app_letimer_pwm_open(float period, float act_period,
                                 uint32_t out0_route, uint32_t out1_route,
                                 bool out0_en, bool out1_en, bool out_en);


//***********************************************************************************
// function definitions
//***********************************************************************************


/******************************************************************************
 **************************** PERIPHERAL FUNCTIONS ****************************
 ******************************************************************************/


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
  si7021_i2c_open(I2C0, writeReg1, measureResRH8_T12);
  shtc3_open(I2C1);
}


/***************************************************************************//**
 * @brief
 *   Configure LETIMER for PWM mode.
 *
 * @details
 *   Driver which instantiates and opens the LETIMER
 *   peripheral in PWM mode
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
 * @param[in] out0_en
 *    True = enable out0_route; False = disable out0_route
 *
 * @param[in] out1_en
 *    True = enable out1_route; False = disable out1_route
 *
 * @param[in] out_en
 *    True = enable PWM output; False = disable PWM output
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
  letimer_pwm.comp1_irq_enable = false;
  letimer_pwm.uf_irq_enable = true;
  letimer_pwm.uf_cb = LETIMER0_UF_CB;

  // open letimer for PWM mode
  letimer_pwm_open(LETIMER0, &letimer_pwm);
}


/******************************************************************************
 ***************************** CALLBACK FUNCTIONS *****************************
 ******************************************************************************/


/***************************************************************************//**
 * @brief
 *   Handles the scheduling of the LETIMER0 underflow call back
 *
 * @details
 *   When the LETIMER0 underflows, sends a measurement packet to the SI7021
 *   and a wakeup packet to the SHTC3
 ******************************************************************************/
void scheduled_letimer0_uf_cb(void)
{
  // remove LETIMER0 underflow callback even from scheduler
  remove_scheduled_event(LETIMER0_UF_CB);

  // measure relative humidity using Si7021
  si7021_i2c_read(I2C0, measureRH_NHMM, false, SI7021_HUM_READ_CB);

  // wakeup SHTC3
  shtc3_write(I2C1, wakeup, SHTC3_WAKEUP_CB);
}


/***************************************************************************//**
 * @brief
 *   Handles the scheduling of the Si7021 humidity read callback
 *
 * @details
 *   The Si7021 takes a temperature measurement every time it measures
 *   humidity. That means temperature can be read from a previous RH
 *   measurement, without having to perform another full read.
 *
 *   This callback function transmits a read packet in order to read
 *   temperature data from the previous RH measurement that scheduled
 *   this callback.
 ******************************************************************************/
void scheduled_si7021_hum_read_cb(void)
{
  // remove event from scheduler
  remove_scheduled_event(SI7021_HUM_READ_CB);

  si7021_parse_RH_data();

  // read temperature from previous previous RH measurement
  si7021_i2c_read(I2C0, MeasureTFromPrevRH, false, SI7021_TEMP_READ_CB);

  timer_delay(80);

  // parse temperature measurement code
  si7021_parse_temp_data();
}


/***************************************************************************//**
 * @brief
 *   Handles the scheduling of the Si7021 temperature read callback
 *
 * @details
 *   Once relative humidity and temperature measurements have been received,
 *   this callback function will store the converted values in static private
 *   variables in the application layer. Then drives an LED on the EFM32
 *   if the relative humidity threshold is reached.
 ******************************************************************************/
void scheduled_si7021_temp_read_cb(void)
{
  // remove event from scheduler
  remove_scheduled_event(SI7021_TEMP_READ_CB);

  // atomic operation
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();

  // store measurements
  app_si7021_rh = si7021_get_rh();
  app_si7021_temp = si7021_get_temp();

  // exit core critical to allow interrupts
  CORE_EXIT_CRITICAL();

  // drive LED
  drive_leds(app_si7021_rh, LED0_PORT, LED0_PIN);
}


/***************************************************************************//**
 * @brief
 *   Handles the scheduling of the Si7021 write user register callback
 *
 * @details
 *   Following a write to the SI7021's user register to configure measurement
 *   resolution, a read packet is transmitted to retrieve the current settings
 *   of the whole user register.
 ******************************************************************************/
void scheduled_si7021_write_reg_cb(void)
{
  // remove event from scheduler
  remove_scheduled_event(SI7021_WRITE_REG_CB);

  // read from user register
  si7021_i2c_read(I2C0, readReg1, false, SI7021_READ_REG_CB);

  // atomic operation
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();

  // store user register settings
  app_si7021_user_reg = si7021_store_user_reg();

  // exit core critical to allow interrupts
  CORE_EXIT_CRITICAL();
}


/***************************************************************************//**
 * @brief
 *   Handles the scheduling of the Si7021 read register callback
 *
 * @details
 *   When the Si7021 is first opened and initialized, an initial write packet
 *   is transmitted to the User Register 1 to set the measurement resolution.
 *   Once completed a read is transmitted to measure relative humidity
 ******************************************************************************/
void scheduled_si7021_read_reg_cb(void)
{
  // remove event from scheduler
  remove_scheduled_event(SI7021_READ_REG_CB);

  // measure relative humidity using Si7021
  si7021_i2c_read(I2C0, measureRH_NHMM, false, SI7021_HUM_READ_CB);

  timer_delay(80);

  // parse RH measurement code
  si7021_parse_RH_data();
}


/***************************************************************************//**
 * @brief
 *   Handles the scheduling of the SHTC3 open callback
 *
 * @details
 *  Just removes scheduled callback event after the SHTC3 has been opened and put
 *  to sleep.
 ******************************************************************************/
void scheduled_shtc3_open_cb(void)
{
  // remove event from scheduler
  remove_scheduled_event(SHTC3_OPEN_CB);
}


/***************************************************************************//**
 * @brief
 *   Handles the scheduling of the SHTC3 sleep callback
 *
 * @details
 *   Following putting the device to sleep (which is done after a measurement
 *   is taken), the relative humidity and temperature measurements are then
 *   parsed out and the raw values are converted to percent humidity and
 *   temperature Celsius. An LED on the EFM32 is then driven if the
 *   percent humidity threshold is reached.
 ******************************************************************************/
void scheduled_shtc3_sleep_cb(void)
{
  // remove event from scheduler
  remove_scheduled_event(SHTC3_SLEEP_CB);
}


/***************************************************************************//**
 * @brief
 *   Handles the scheduling of the SHTC3 wakeup callback
 *
 * @details
 *   Following the transmission of a wakeup command, a write packet is
 *   transmitted to perform a measurement.
 ******************************************************************************/
void scheduled_shtc3_wakeup_cb(void)
{
  // remove event from scheduler
  remove_scheduled_event(SHTC3_WAKEUP_CB);

  shtc3_write(I2C1, readRHFirst_LPM, SHTC3_MEASUREMENT_CB);
}


/***************************************************************************//**
 * @brief
 *   Handles the scheduling of the SHTC3 measurement callback
 *
 * @details
 *   Following the transmission of a measurement command, a read packet is
 *   transmitted to retrieve the measurement data from the SHTC3
 ******************************************************************************/
void scheduled_shtc3_measurement_cb(void)
{
  // remove event from scheduler
  remove_scheduled_event(SHTC3_MEASUREMENT_CB);

  shtc3_read(I2C1, false, SHTC3_READ_REQ_CB);
}


/***************************************************************************//**
 * @brief
 *   Handles the scheduling of the SHTC3 read request callback
 *
 * @details
 *   Following the a read transaction (which scheduled a sleep callback),
 *   a sleep packet is sent to put the SHTC3 back to sleep. The SHTC3 should
 *   be put to sleep following every transaction.
 ******************************************************************************/
void scheduled_shtc3_read_req_cb(void)
{
  // remove event from scheduler
  remove_scheduled_event(SHTC3_READ_REQ_CB);

  // parse measured data;
  shtc3_parse_measurement_data_RH_first();

  // atomic operation
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();

  app_shtc3_rh = shtc3_get_rh();
  app_shtc3_temp = shtc3_get_temp();

  timer_delay(80);

  // exit core critical to allow interrupts
  CORE_EXIT_CRITICAL();

  drive_leds(app_shtc3_rh, LED1_PORT, LED1_PIN);

  // transmit a sleep command
  shtc3_write(I2C1, sleep, SHTC3_SLEEP_CB);
}
