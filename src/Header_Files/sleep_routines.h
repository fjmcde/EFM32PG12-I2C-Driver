/**************************************************************************
 * @file
 *   sleep_routines.h
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
// header guards
//*******************************************************
#ifndef SLEEP_ROUTINES_HG
#define SLEEP_ROUTINES_HG

//*******************************************************
// included files
//*******************************************************
// system included files
#include <string.h>
#include <stdio.h>


// Silicon Labs included files
#include "em_emu.h"
#include "em_core.h"
#include "em_assert.h"


// developer included files


//*******************************************************
// defined macros
//*******************************************************
#define EM0                       ((uint32_t) 0x00)   // energy mode 0
#define EM1                       ((uint32_t) 0x01)   // energy mode 1
#define EM2                       ((uint32_t) 0x02)   // energy mode 2
#define EM3                       ((uint32_t) 0x03)   // energy mode 3
#define EM4                       ((uint32_t) 0x04)   // energy mode 4
#define MAX_ENERGY_MODES          ((uint32_t) 0x05)   // number of energy modes


//*******************************************************
// enums
//*******************************************************


//*******************************************************
// structs
//*******************************************************


//*******************************************************
// function prototypes
//*******************************************************
void sleep_open(void);
void sleep_block_mode(uint32_t EM);
void sleep_unblock_mode(uint32_t EM);
void enter_sleep(void);
uint32_t current_block_energy_mode(void);


#endif
