/* 
    Timer.h -- E-Leadscrew definitions for an electronic replacement of
    gears used for threading or hobbing on a Lathe or Gear Hobber.

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
 // Timer support defines
#ifndef __TIMER_H
#define __TIMER_H
/*******************************************************************
 *
 *    DESCRIPTION:
 *  The TIMER0 is programmed to create an interrupt every 10ms.  Inside the interrupt
 *  routine a variable named TickCount is incremented on every interrupt.
 *  An optional set of variables used for Time of Day keeping can also be
 *  adjusted.  InitTimer() is called once to inititialize variables and hardware.
 *
 *  Inside the main program loop the Device Driver Timer() needs to be called on a 
 *  regular basis.  The Timer() device driver examines the TickCount variable and 
 *  if non-zero checks the Active_timers flag and decrements the Timers[] variable
 *  for each active timer.
 *
 *  If the Timers[] variable goes to or through zero then the Active_timers bit flag 
 *  for that timer is masked against the Mask[] value which will disable the timer
 *  if the timer was not started as a recurring timer.
 *
 *  Timer Usage:
 *  A timer can be started as a one shot or recurring timer.  The arguments to 
 *  StartTimer() are the Timer # (from 1..8) and the delay in 10ms ticks which is 
 *  an in variable from 0..16367.  Call StartRecTimer() with the same arguments as
 *  StartTimer() to start a recurring timer.
 *
 *  Both types of timers are tested for completion by calling TimerDone() with 
 *  the timer number as the argument.  The function returns a copy of the timer 
 *  done flag (TRUE if the timer is done).  The internal Done flag that 
 *  TimerDone() returns stays set until either a new call to StartTimer() is done 
 *  or the Recurrent timer is re-armed by calling ArmTimer() with the timer number.
 *
 *  The major difference between the two types of timers is that when a recurrent 
 *  timer completes the timer is restarted immediately to ensure that there is no 
 *  extra delay added between timer intervals.  However,  until the call to the 
 *  ArmTimer() function the DoneTimer() call signals that the previous interval is
 *  complete.
 *
 *  To stop a recurring timer so that the Timer() Device driver need not spend
 *  extra time checking if the timer has expired the StartTimer() function is 
 *  called with the timer number and a Time of 0.
 *
 *
 *    AUTHOR:
 *		John Dammeyer
 *		Automation Artisans Inc.
 *		(c) 1996, 1999, 2000
 *
 *    HISTORY:    
 * Initial Version: 0.00 05Nov96
 * Current Version: 
 *   Timer() - changed shift to test first bit 0 then bit 1 etc. rather
 * 				than bit zero and then the value zero thereafter.
 *   Timer() - change mask to compliment of bit position to mask out 
 *   			completed timer rather than all the other timers.
 * Current Version: 0.02 15Nov96
 *   Changed variable names to avoid naming conflict with globals.
 *   Removed Interrupt disable in StartTimer.
 * Current Version: 0.03 03DEC96
 *   Test t_Delay for non-zero before looping through all 8 timer flags
 *   and if zero don't waste timer looking for active timers.
 * Current Version 0.10	17Sep2000
 *	 Milliseconds and other times added for real time clock and precise
 *   times from T1 interrupt to accurately count pulses in a 1 second period.
 * Current Version 0.20 14Dec2000
 *	 Added code for a periodic recurring timer.
 * Current Version 0.30 14FEB08
 *	 Changed timeout to 16 bit int so timer delays can be longer than 2.5 seconds.
 * *
 * *******************************************************************/

void InitTimerDevice(void);

/** ServiceRTC
 * *
 *  FILENAME: ..\include\Timer.h
 *
 *  PARAMETERS:	None
 *
 *  DESCRIPTION: RTC interrupt routine called from Interrupt dispatcher.
 *
 *		Called from the interrupt dispatcher this function reloads the
 *		increments a global a bit variable for every 10ms interrupt.  
 *		It also increments a seconds, minutes and hours variable to be 
 *		able to	maintain a real time clock.
 *
 *  RETURNS: Nothing.
 *
 */
void ServiceRTC(void);		// Real Time Clock Tick.

/*
 ** Timer
 *
 *  FILENAME: ..\include\Timer.h
 *
 *  PARAMETERS: None 
 *
 *  DESCRIPTION: Timer device Driver.
 *
 *		Required in the main processing loop.  This takes the place of 
 *		a lengthy interrupt response routine and disabling interrupts 
 *		whenever any of the timer's 16 bit variables are accessed.
 *		It walks through the Active_timer variable and if the a bit is set 
 *		if subtracts the Tick value incremented in teh TimerRTC.
 *		Once the specific timer falls to zero or less than zero 
 *		the Done_timer Bit Flag for that timer is set.
 *	  !	If application spends any time in a long this function should 
 *		be called periodically.
 *
 *  RETURNS: Nothing.
 *
 */
void TimerDevice( void );			// Handles delays.

/*
 ** StartTimer
 *
 *  FILENAME: ..\include\Timer.h
 *
 *  PARAMETERS:	t_timer -- Timer # between 0..7
 *				t_delay -- delay in 10ms ticks between 0..16367
 *
 *  DESCRIPTION: Start a Timer.
 *
 *		Saves the time delay value into the Timer Array indexed by the timer #
 *		and then sets the correct bit position in the Active_timers variable. 
 *		Usage is just StartTimer( LIST_TIMER, T30MS );
 *	  ! By convention a Timer # is #defined in uppercase in the proj.h file
 *		There are a number of default defines like T_1SEC.
 *
 *  RETURNS: Nothing.
 *
 */
void StartTimer( uint8 TimerNumber, int16 DelayTime );

/*
 ** TimerDone
 *
 *  FILENAME: ..\include\Timer.h
 *
 *  PARAMETERS:	t_timer -- Timer # between 0..7
 *
 *  DESCRIPTION: Checks to see if a timer has completed.
 *
 *		Called to check if the timer is done.  It checks a Done bit
 *		in the Done_timers variable which was set in Timer if counter
 *		reached zero.
 *
 *  RETURNS: TRUE if Timer Done,  FALSE if Timer not done.
 *
 */
int16 TimerDone( uint8 TimerNumber );

/*
 ** StartRecurTimer
 *
 *  FILENAME: ..\include\Timer.h
 *
 *  PARAMETERS:	t_timer -- Timer # between 0..7
 *				t_delay -- delay in 10ms ticks between 0..16367
 *
 *  DESCRIPTION: -- Starts a Timer that is automatically restarted when complete.
 *
 *		Just like StartTimer this routine sets up the Timer number in the 
 *		variable Active_timer.  The difference is that the ResetMask is 
 *		set to all 1's so that the Active_timer flag is not cleared.  The 
 *		timer is reloaded with the initial value and the Done_timer flag 
 *		is set.
 *
 *  RETURNS: Nothing.
 *
 */
void StartRecurTimer( uint8 TimerNumber, int16 DelayTime );

/*
 ** ArmTimer
 *
 *  FILENAME: ..\include\Timer.h
 *
 *  PARAMETERS:	Timer Number
 *
 *  DESCRIPTION: Re-arms a recurring timer.
 *
 *		A recurring timer when expired has the Done_timer bit set to let 
 *		the application know that the timer is done.  However,  the timer
 *		is still running and if this done flag isn't the recurring timeout can
 *		not be passed to the application.  
 *		Some parts of an application may test DoneTimer() return value in 
 *		conjunction with some other events so an explicit method to clear
 *		the Done_timer flag is done with the ArmTimer call.
 *
 *  RETURNS: Nothing.
 *
 */
void ArmTimer( uint8 t_timer1 );


// Real time clock divider for periodic Interrupt.
// With a 11,059,200 Hz osc. Clock is always divided by 4,
//    then 256 prescaler
//    then 108 to get a 100Hz interrupt rate;  10ms period.
// Prescaler setup
#define		RTC_OPTION	((1<<PS0)+(1<<PS1)+(1<<PS2))

// Divisor setup . Ticks till Timer0 overflow.
// 2.4576MHz xtal needs value of 24 for 10ms clock.
#define TXTAL24576	24
// 4.0MHz xtal needs value of 39 for 10ms clock.
#define TXTAL4		39
// 4.9152MHz xtal needs value of 24 for 10ms clock.
#define TXTAL49152	48
// 11,059,200 xtal needs value of 108 for 10ms clock
#define TXTAL110592	108
// 20MHz xtal needs value of 195 for 10ms clock
#define TXTAL20MHZ	195
// 40MHz xtal needs value of 195 for 10ms clock
#define TXTAL40MHZ	0x186


#define T_ZERO			0		// Timer time 0 to stop a timer.
#define T_5MS			1		// Minimum timer resolution 10ms
#define	T_15MS			2		// 20ms instead of 15.
#define	T_20MS			2		// 20ms instead of 15.
#define	T_50MS			5	
#define	T_100MS			10	
#define	T_150MS			15	
#define T_250MS			25		//
#define T_350MS			35		//
#define	T_500MS			50		// 
#define T_900MS			90
#define	T_980MS			98		// Balance of 1 second with 20ms flash.
#define	T_1SEC			100		// 1 Second.
#define T_1500MS		150		// 1.5 Seconds.
#define	T_2SEC			200		// 2 seconds
#define	T_2_5SEC		250		// 2.5 seconds
#define	T_3SEC			300		// 5 seconds
#define	T_4SEC			400		// 5 seconds
#define	T_5SEC			500		// 5 seconds

extern volatile int16 TickCount;

#endif
