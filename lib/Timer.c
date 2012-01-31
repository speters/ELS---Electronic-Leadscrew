/* 
    Timer.c -- E-Leadscrew code for an electronic replacement of
    gears used for threading or hobbing on a Lathe or Gear Hobber.

	The Timer.c functions set up a 10mS system clock that can be
	used in state machines to delay further action until a timer
	is done.  Simulates the block/hold features of an RTOS.
	In the application it's use is demonstrated in the Heartbeat 
	code.

    Copyright (C) 2005  John Dammeyer

    This file is part of The Electronic Lead Screw (ELS) Project.

    ELS is free software: you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 2
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.

	Or download here.
	http://www.gnu.org/copyleft/gpl.html

    John Dammeyer
    johnd@autoartisans.com


   	Initial Version: 0.00a

   	Version changes: 

*/
// Timer Routines for delays and Tick/Tock.
// (c) 1996 Automation Artisans Inc.
// Timer.c
//
#include "processor.h"

#include <stddef.h>
#include <ctype.h>

#include "common.h"
#include "config.h"

#include "menu.h"	
#include "globvars.h"

#include "int.h"
#include "timer.h"

// Permanent Variables.
volatile int16 TickCount;		// Number of ticks since we last checked
uint8 Active_timers;	  		// Bit map of active timers; 1 == active
uint8 Done_timers;	  			// Bit map of timers that have completed; 1 == Done.

// ROM mask for each bit position rather than shifting a 1 n times into correct position.
const uint8 TimerMask[8] = { 1,2,4,8,16,32,64,128 };

// Temporary Variables.
uint8 t_Image;		// bit map used for setting and clearing bits
uint8 t_Timer; 		// Timer index variable for loops.
uint8 t_temp;    	// for temporary results to avoid complex expressions.
int16 t_Delay; 		// number of ticks that happened since the last time we checked.
int16 t_currentTimer;


int16 Timers[MAX_TIMERS];		  			// Holds delay for each timer.
int16 TimerReload[MAX_TIMERS];			// Timer reload value for recurring timers.

// Mask holds 0 in bit position of timer that must have bit in Active_timers cleared
// when delay is complete;  has a 1 in position of all timers that must not be stopped.
uint8 TimerResetMask[MAX_TIMERS];  // 

/*
 *  FUNCTION: ServiceRTC
 *
 *  PARAMETERS:	   	None
 *
 *  USES GLOBALS:	TickCount
 *
 *  DESCRIPTION:	Called on each Timer Overflow
 *
 *  RETURNS: 		Nothing
 *
 */
void 
ServiceRTC( void ) {		// Real Time Clock Tick.
	// acknowledge interrupt so we can get interrupted again.
	INTCONbits.T0IF = 0;
	INTCONbits.GIEH = 0;	// Prevent Hi Priority from changing 8 bit operation.
	// Reload counter for correct number of ticks till overflow 
	// and the next 10ms interrupt.
	TMR0H = RTC_DIVISOR>>8;	
	TMR0L = RTC_DIVISOR;
	INTCONbits.GIEH = 1;
	// increment our counter.
	TickCount++;
}

/*
 *  FUNCTION: InitTimerDevice
 *
 *  PARAMETERS:		None
 *
 *  USES GLOBALS:	ActiveTimers, Done_Timers, TickCount
 *
 *  DESCRIPTION:	Sets up timer for 10mS tick.
 *
 *  RETURNS: 		Nothing
 *
 */
void 
InitTimerDevice(void) {
	// Start the real time clock.
	Active_timers = 0;
	Done_timers = 0xff;
	TickCount = 0;

	TMR0H = RTC_DIVISOR>>8;	// Defined in "Int.h"
	TMR0L = RTC_DIVISOR;

	T0CON = 0x87;			// Enabled, 16 bit, internal, don't care, Prescale, 1/256
}

/*
 *  FUNCTION: TimerDevice
 *
 *  PARAMETERS:		None
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:	Called from the mainline loop, checks to see if there has been a 
 *					10mS tick and then if so, checks to see if any outstanding timers
 *					are enabled and if after the TickCount is subtracted, are done.
 *					If so, it flags them as done.
 *
 *  RETURNS: 		Nothing.
 *
 */
void 
TimerDevice( void ) {
	// Test all bits in image of active timers.
	t_Image = Active_timers;
	
	// disable timer interrupts
    INTCONbits.T0IE = 0;
	t_Delay = TickCount;
	TickCount = 0;
    INTCONbits.T0IE = 1;
	// re-enable timer interrupts

	if ( t_Delay != 0 ) { // Loop though only if a tick occurred
		for (t_Timer = 0; t_Timer < MAX_TIMERS; t_Timer++) {
	  		if (t_Image & 1) {						 // if timer is running.
				t_currentTimer = Timers[t_Timer] - t_Delay;
	  						 
				if (t_currentTimer <= 0) {			 // and timer is done.
					Done_timers |= TimerMask[t_Timer];	// Flag this timer done.
					// Reset or leave alone depending on TimerResetMask
					Active_timers &= TimerResetMask[t_Timer]; 
					// reload if it wasn't cleared
					Timers[t_Timer] = TimerReload[t_Timer];
				}
				else
					Timers[t_Timer] = t_currentTimer;
			}
			t_Image >>= 1; // check next timer.
	  	}
	}
}


/*
 *  FUNCTION: TimerDone
 *
 *  PARAMETERS:		t_timer1	-- Timer # 0..7
 *
 *  USES GLOBALS:	Done_timers
 *
 *  DESCRIPTION:	Checks the bit in the Done_timers variable to see if it's set
 *					which means the timer is done.
 *
 *  RETURNS: 		TRUE if timer done, FALSE if not.
 *
 */
int16 
TimerDone( uint8 TimerNumber ) {
	return( (Done_timers & TimerMask[TimerNumber]) != 0 );
}									


/*
 *  FUNCTION: StartTimer
 *
 *  PARAMETERS:		TimerNumber (0..7), DelayTime (10mS ticks)
 *
 *  USES GLOBALS:	Done_timers, Active_timers,
 *					Timers[], TimersResetMask[]
 *
 *  DESCRIPTION:	When supplied with a non-zero delay time start a timer
 *					or if delay is zero, stop the timer.
 *
 *  RETURNS: 		Nothing
 *
 */
void 
StartTimer( uint8 TimerNumber, int16 DelayTime ) {
	t_temp = TimerMask[TimerNumber];
	Done_timers &= ~t_temp;		// Reset Done in case we've changed from Recurring to One-shot
	// DelayTime of means stop timer.
	if (DelayTime) {
		Active_timers |= t_temp;
		// Make a mask that clears our timer.
		TimerResetMask[TimerNumber] = ~t_temp;
		Timers[TimerNumber] = DelayTime;		// Set delay time.
	}
	else {  // Stop timer
		Done_timers |= t_temp;
		t_temp = ~t_temp;
		TimerResetMask[TimerNumber] = t_temp;
		// Clear Timer
		Active_timers &= t_temp;
	}
}


/*
 *  FUNCTION: StartRecurTimer
 *
 *  PARAMETERS:		TimerNumber		-- Timer number (0..7)
 *					DelayTime		-- Delay in 10mS ticks.
 *
 *  USES GLOBALS:	Active_timers, TimerResetMask, Timers, TimerReload
 *
 *  DESCRIPTION:	Like StartTimer, this starts a timer but adds a reload
 *					value that is used when the timer is expired.  This
 *					means that there is no delay between a timer finishing
 *					and the application restarting it and possibily losing 
 *					a tick or two.
 *
 *  RETURNS: 		Nothing.
 *
 */
void 
StartRecurTimer( uint8 TimerNumber, int16 DelayTime ) {
	t_temp = TimerMask[TimerNumber];
	Active_timers |= t_temp;
	// Create mask that won't reset the timer.
	TimerResetMask[TimerNumber] = 0xff;
	Timers[TimerNumber] = DelayTime;		// Set delay time.
	TimerReload[TimerNumber] = DelayTime; // Set reload time.
}


/*
 *  FUNCTION: ArmTimer
 *
 *  PARAMETERS:		t_timer1
 *
 *  USES GLOBALS:	Done_timers
 *
 *  DESCRIPTION:	A re-occuring timer sets the done flag but keeps 
 *					running in the background since it's still an 
 *					Active_timer.
 *					After testing if a tumer is done, Arm the timer again
 *					so that the application which parks on this timer will
 *					can tell when it is done again.
 *
 *  RETURNS: 
 *
 */
void 
ArmTimer( uint8 t_timer1 ) {
	Done_timers &= ~TimerMask[t_timer1];		// Reset Done flag for next time.
}
