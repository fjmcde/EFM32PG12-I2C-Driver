/***************************************************************************//**
 * @file
 *   cmu.c
 * @author
 *   Frank McDermott
 * @date
 *   9/18/2022
 * @brief
 *   Driver for enabling the required oscillators and routing the
 *   clock tree to the LETIMER0.
 ******************************************************************************/

//***********************************************************************************
// included header file
//***********************************************************************************
#include "cmu.h"


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
 *   Routes clock tree to LETIMER0 and enables the proper oscillators
 *
 * @details
 *   Enabled:
 *   - High-frequency peripheral clock (HFPER) for use with I2C0 peripheral
 *   - Low-energy clock divided down from HFCLK (CORELE) for use with LETIMER0
 *
 *   Disabled:
 *   - Low Frequency Resistor Capacitor Oscillator (LFRCO)
 *   - Low Frequency Crystal Oscillator (LFXO)
 *
 *   Selected:
 *   - Ultra low-frequency RC oscillator (ULFRCO)
 *
 *   Routed:
 *   - Low-frequency A clock (LFA) to LETIMER
 *
 * @note
 *   No requirement to enable the ULFRCO oscillator.
 *   It is always enabled in EM0-EM4
 *
 ******************************************************************************/
void cmu_open(void){

    // enable High-frequency peripheral clock (AN0004 Table 2.1)
    CMU_ClockEnable(cmuClock_HFPER, true);

    // by default, LFRCO, is enabled; disable the LFRCO oscillator
    CMU_OscillatorEnable(cmuOsc_LFRCO, false, false);

    // disable the Low Frequency Crystal Oscillator, LFXO
    CMU_OscillatorEnable(cmuOsc_LFXO, false, false);

    // route LFA clock to LETIMER0 clock tree
    CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_ULFRCO);

    // enable global low frequency clock
    CMU_ClockEnable(cmuClock_CORELE, true);
}
