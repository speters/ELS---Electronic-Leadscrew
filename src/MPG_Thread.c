// MPG_Thread.c
/* 
    ELeadscrew.c -- Mainline application for an electronic replacement of
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


   	Initial Version: 0.00

   	Version changes:
 
	
*/			 

#include "processor.h"

#include <ctype.h>
#include <stdio.h>

#include "common.h"
#include "config.h"

#include "menu.h"
#include "globvars.h"	

#include "Timer.h"
#include "Key.h"
#include "MotorDriver.h"
#include "MPG_Thread.h"

// *** PRIVATE DEFINITIONS ***

enum MPG_STATES {
	MPG_OFF_STATE,
	MPG_ON_STATE,
	MPG_WAIT_RELEASE_STATE,
	MPG_RELEASED_STATE
};

// *** PRIVATE VARIABLES ***
enum MPG_STATES MPGState = MPG_OFF_STATE;
enum MOTOR_TYPES ActiveMotor = MOTOR_Z;
int32 DistanceToJog;
int8 JogIndex = 0;

// *** PUBLIC VARIABLES ***
int8 NewJogIndex;
float32 JogIncrement[JOG_TABLE_SIZE] = {0.001,0.005,0.010,0.020};	// Default to Imperial

// *** PRIVATE FUNCTIONS ***
void
SetJogIncrements(int8 mode) {
	if (mode) {	// Metric Mode
		JogIncrement[0] = 0.01;	
		JogIncrement[1] = 0.05;	
		JogIncrement[2] = 0.10;	
		JogIncrement[3] = 0.20;	
	}
	else { // Imperial Mode 
		JogIncrement[0] = 0.001;	
		JogIncrement[1] = 0.005;	
		JogIncrement[2] = 0.010;	
		JogIncrement[3] = 0.020;	
	}
}

// *** PUBLIC FUNCTIONS ***
/*
 *  FUNCTION: InitMPG_Thread
 *
 *  PARAMETERS:			mode	-- Metric or Imperial
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:		Sets up initial MPG_Thread.
 *
 *  RETURNS: 			Nothing.
 *
 */
void 
InitMPG_Thread(int8 mode) {
	ActiveMotor = MOTOR_Z;
	OldZEncoderCounter = 0;
	SetJogIncrements(mode);
	JogIndex = 0;				// Start at beginning of JOG Table
	NewJogIndex = JogIndex;		// 
	DistanceToJog = CalculateMotorDistance( JogIncrement[JogIndex], ZDistanceDivisor, mode );
	MPGState = MPG_OFF_STATE;
}

/*
 *  FUNCTION: RestoreCurrentJog
 *
 *  PARAMETERS:			None
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:
 *						This function restores any variables if the CANCEL key is pressed.
 *						On the MPG button press, the NewJogIndex is automatically incremented and 
 *						then each time it's pressed it's incremented again.  But if we cancel out, 
 *						it's left at the incremented value rather than at our current JogIndex.  
 *						So if we press cancel, we restore it.  This keeps the MPG button click 
 *						behavior repeatable.
 *
 *  RETURNS: 			Nothing
 *
 */
void
RestoreCurrentJog(void) {
	NewJogIndex = JogIndex;
}

/*
 *  FUNCTION: ChangeJogDistance
 *
 *  PARAMETERS:		None
 *
 *  USES GLOBALS:	mode	-- Metric or Imperial
 *
 *  DESCRIPTION:
 *					The MPG device driver has possibly updated the NewJogIndex and this function does 
 *					what's needed for the MPG thread to properly move the motor the new jog distance.
 *
 *  RETURNS: 		Nothing
 *
 */
void
ChangeJogDistance(int8 mode) {
	SetJogIncrements(mode);		// Select Metric or Imperial distance table.
	JogIndex = NewJogIndex;		// Update Distance to Jog.
	if (ActiveMotor == MOTOR_Z) 
		DistanceToJog = CalculateMotorDistance( JogIncrement[JogIndex], ZDistanceDivisor, mode );
	else 
		DistanceToJog = CalculateMotorDistance( JogIncrement[JogIndex], XDistanceDivisor, mode );
}

/*
 *  FUNCTION: ChangeJogMotor
 *
 *  PARAMETERS:		mode 	-- Metric or Imperial
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:
 *					Toggle Which motor is run with MPG.  
 *   				Called from KeyThread and Function Button.  May later be ALT and MPG Click.
 *
 *  RETURNS: 		Nothing
 *
 */
void
ChangeJogMotor(int8 mode) {
	if (ActiveMotor == MOTOR_Z) {
		ActiveMotor = MOTOR_X;
	}
	else {
		ActiveMotor = MOTOR_Z;
	}
	NewJogIndex = 0; 	// Start again at smallest increment.
	ChangeJogDistance(mode);
}

/*
	MPG Run Thread:
		Monitors MPG knob and top button and sends out movement messages
		for each click and changes distance moved when top buttton pressed.
*/
/*
 *  FUNCTION: MPG_RunThread
 *
 *  PARAMETERS:		None
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:
 *					Monitors MPG knob and top button and sends out movement messages
 *					for each click and changes distance moved when top buttton pressed.
 *
 *  RETURNS: 		Nothing
 *
 */
void 
MPG_RunThread(void) {

	if (fRunTimeDisplay) {	// MPG Movement only allowed while in RDY state with DRO displayed.

		switch (MPGState) {
		  /*
			Wait for button on top of MPG to be pressed.
		  */	
		  case MPG_OFF_STATE :
			if (bMPGSelect == 0) {
				MPGState = MPG_ON_STATE;
				StartTimer( KEY_TIMER, T_KEY_DEBOUNCE );
			}
			break;

		  /*
			If button still down and not bouncing anymore then change how far we move.
			Only the global array value is changed.  EEROM value needs to be changed 
			directly.
		  */	
		  case MPG_ON_STATE :
			if (TimerDone(KEY_TIMER) && (bMPGSelect == 0) ) {
				SetDisplayMode(MSG_CHANGE_JOG_DISTANCE_MODE);
				NewJogIndex = (NewJogIndex+1) & 0x03;				// Good press so increment current index.
				// ACPT FN4 key will set this value back into JogIndex by calling ChangeJogIndex();
				DEBUGSTR( "New Jog Index %d\n", NewJogIndex);
				MPGState = MPG_WAIT_RELEASE_STATE;
			}
			break;

		  /*
			Wait for button on top of MPG to be released
		  */	
		  case MPG_WAIT_RELEASE_STATE :
			if (bMPGSelect == 1) {
				MPGState = MPG_RELEASED_STATE;
				StartTimer( KEY_TIMER, T_KEY_DEBOUNCE );
			}
			break;

		  /*
			When button has been released and isn't bouncing anymore go wait for
			it to be pressed again.
		  */	
		  case MPG_RELEASED_STATE :
			if ( TimerDone(KEY_TIMER) ) {
				if (bMPGSelect == 1) 
					MPGState = MPG_OFF_STATE;
				else
					MPGState = MPG_WAIT_RELEASE_STATE;
			}
			break;
		}
		/*
			On each encoder click clear the encoder counter and then if we aren't moving
			calculate how far to move and and request the move.
		*/
		switch (ActiveMotor) {
		  case MOTOR_Z :
			// Change in Encoder MPG?
			if (OldZEncoderCounter != ZEncoderCounter) {
				INTCON &= 0x3F;
				OldZEncoderCounter += ZEncoderCounter;		// Accumulate encoder pulses to match markings on knob.
				ZEncoderCounter = 0;
				INTCON |= 0xC0;
				// If we're in the middle of a distance move don't interrupt with a new move. 
				if ( !fZMoveBSY && !fXMoveBSY ) {
					// But if we're not moving,  figure out which way, and use the number of encoder pulses to determine absolute distance.
					if (OldZEncoderCounter < 0) {
						MotorMoveDistance( MOTOR_Z, DistanceToJog * (0-OldZEncoderCounter) , (WORD)GetGlobalVarWord(MOVE_RATE_Z_NDX), MOVE_LEFT,
							SPINDLE_EITHER, /*FALSE,*/ TRUE );
					}
					else if (OldZEncoderCounter > 0) {
						MotorMoveDistance( MOTOR_Z, DistanceToJog * OldZEncoderCounter, (WORD)GetGlobalVarWord(MOVE_RATE_Z_NDX), MOVE_RIGHT,
							SPINDLE_EITHER, /*FALSE,*/ TRUE );
					}
					// Now that we're done our move, clear out the accumulated value.
					OldZEncoderCounter = 0;
				}
			}
			break;

		  case MOTOR_X :
			// Change in Encoder MPG?
			if (OldZEncoderCounter != ZEncoderCounter) {
				INTCON &= 0x3F;
				OldZEncoderCounter += ZEncoderCounter;		// Accumulate encoder pulses to match markings on knob.
				ZEncoderCounter = 0;
				INTCON |= 0xC0;
				// If we're in the middle of a distance move don't interrupt with a new move. 
				if ( !fZMoveBSY && !fXMoveBSY ) {
					// But if we're not moving,  figure out which way, and use the number of encoder pulses to determine absolute distance.
					// Note direction of motor is opposite to Z axis so knob turning matches cross slide handle.
					if (OldZEncoderCounter < 0) {
						MotorMoveDistance( MOTOR_X, DistanceToJog * (0-OldZEncoderCounter) , (WORD)GetGlobalVarWord(MOVE_RATE_X_NDX), 1,
								SPINDLE_EITHER, /*FALSE,*/ TRUE );	// Don't track spindle but watch limit switches.
					}
					else if (OldZEncoderCounter > 0) {
						MotorMoveDistance( MOTOR_X, DistanceToJog * OldZEncoderCounter, (WORD)GetGlobalVarWord(MOVE_RATE_X_NDX), 0,
								SPINDLE_EITHER, /*FALSE,*/ TRUE );	// Don't track spindle but watch limit switches.
					}
					// Now that we're done our move, clear out the accumulated value.
					OldZEncoderCounter = 0;
				}
			}
			break;
		}
	}	
}

