/**************************************************************************//**
 * @file
 *   sleep_routines.c
 * @author
 *   Frank McDermott
 * @date
 *   10/4/2022
 ***************************************************************************
 * @section License
 * <b>(C) Copyright 2015 Silicon Labs, http://www.silabs.com</b>
 ***************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
*
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Labs has no
 * obligation to support this Software. Silicon Labs is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Silicon Labs will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
**************************************************************************/

//*******************************************************
// included header file
//*******************************************************
#include "sleep_routines.h"


//*******************************************************
// static/private data
//*******************************************************
static int lowest_energy_mode[MAX_ENERGY_MODES];  // tracks the energy mode blocks for each state


//***********************************************************************************
// static/private functions
//***********************************************************************************


//***********************************************************************************
// function definitions
//***********************************************************************************
/***************************************************************************//**
 * @brief
 *   Driver to initialize state machine
 *
 * @details
 *   Initialize the sleep_routines static array, lowest_energy_mode[],
 *   to all zeroes
******************************************************************************/
void sleep_open(void)
{
  // make atomic by disallowing interrupts
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();

  // reset array
  memset(lowest_energy_mode, EM0, sizeof(lowest_energy_mode));

  // allow interrupts
  CORE_EXIT_CRITICAL();
}


/***************************************************************************//**
 * @brief
 *   Prevents the CPU from entering sleep mode while active
 *
 * @details
 *   Utilized by a peripheral to prevent the CPU from going into
 *   sleep mode while the peripheral is active. It will increment the
 *   associated array element in lowest_energy_mode[] by one.
 *
 * @note
 *   Edits static variable, so must be an atomic operation
 *
 * @param[in] EM
 *   Energy mode to block
 *
******************************************************************************/
void sleep_block_mode(uint32_t EM)
{
  // make atomic by disallowing interrupts
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();

  // increment the energy mode
  lowest_energy_mode[EM]++;

  // allow interrupts
  CORE_EXIT_CRITICAL();

  // assert that the energy mode is less than the maximum energy mode
  EFM_ASSERT(lowest_energy_mode[EM] < MAX_ENERGY_MODES);
}


/***************************************************************************//**
 * @brief
 *   Driver to release an energy mode
 *
 * @details
 *   Utilized to release the processor from going into a sleep mode with
 *   a peripheral that is no longer active. It will decrement the associated
 *   array element in lowest_energy_mode[] by one.
 *
 * @note
 *   Edits static variable, so must be atomic operation.
 *
 * @param[in] EM
 *   Energy mode to release.
 *
******************************************************************************/
void sleep_unblock_mode(uint32_t EM)
{
  // make atomic
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();

  if(lowest_energy_mode[EM] > EM0)
  {
    lowest_energy_mode[EM]--;
  }

  // if called, unblock sleep modes > block sleep modes; NOT GOOD
  EFM_ASSERT(lowest_energy_mode[EM] >= EM0);

  // allow interrupts
  CORE_EXIT_CRITICAL();
}

/***************************************************************************//**
 * @brief
 *   Driver for sleep FSM
 *
 * @note
 *   Must be an atomic operation to disallow interrupts until after the
 *   operation is complete.
 *
 * @details
 *   Function that will enter the appropriate sleep Energy Mode based on the
 *   first non-zero array element in lowest_energy_mode[].
******************************************************************************/
void enter_sleep(void)
{
  // make atomic
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();

  // FSM
  if(lowest_energy_mode[EM0] > EM0){CORE_EXIT_CRITICAL(); return; }
  else if(lowest_energy_mode[EM1] > EM0){ CORE_EXIT_CRITICAL(); return; }
  else if(lowest_energy_mode[EM2] > EM0){ CORE_EXIT_CRITICAL(); EMU_EnterEM1(); return; }
  else if(lowest_energy_mode[EM3] > EM0){ CORE_EXIT_CRITICAL(); EMU_EnterEM2(true); return; }
  else{ CORE_EXIT_CRITICAL(); EMU_EnterEM3(true); return; }
}


/***************************************************************************//**
 * @brief
 *   Driver to find the energy the system cannot enter.
 *
 * @details
 *   Function that returns which energy mode that the current system cannot
 *   enter, the first non-zero array element in lowest_energy_mode[]. Default
 *   blocked energy mode is EM4.
******************************************************************************/
uint32_t current_block_energy_mode(void)
{
  // make atomic by disallowing interrupts
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();

  // must call CORE_EXIT_CRITICAL after accessing/updating static data
  // but before returning. Otherwise
  if(lowest_energy_mode[EM0] != EM0){ CORE_EXIT_CRITICAL(); return EM0; }
  else if(lowest_energy_mode[EM1] != EM0){ CORE_EXIT_CRITICAL(); return EM1; }
  else if(lowest_energy_mode[EM2] != EM0){ CORE_EXIT_CRITICAL(); return EM2; }
  else if(lowest_energy_mode[EM3] != EM0){ CORE_EXIT_CRITICAL(); return EM3; }
  else{ CORE_EXIT_CRITICAL(); return EM4;}
}
