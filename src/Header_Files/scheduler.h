#ifndef SCHEDULER_HG
#define	SCHEDULER_HG


//***********************************************************************************
// Include files
//***********************************************************************************
// system included files
#include <stdint.h>


// Silicon Labs included files
#include "em_assert.h"
#include "em_core.h"
#include "em_emu.h"

// developer included files


//*******************************************************
// defined macros
//*******************************************************
#define CLEAR_SCHEDULED_EVENTS           (uint32_t)(0x00) // mask to clear all scheduled events


//***********************************************************************************
// enums
//***********************************************************************************


//***********************************************************************************
// structs
//***********************************************************************************


//***********************************************************************************
// function prototypes
//***********************************************************************************
void scheduler_open(void);
void add_scheduled_event(uint32_t event);
void remove_scheduled_event(uint32_t event);
uint32_t get_scheduled_events(void);


#endif
