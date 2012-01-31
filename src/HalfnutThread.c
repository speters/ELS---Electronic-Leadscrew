// Electronic Half Nut Thread
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
#include "AtoD.h"
#include "MotorDriver.h"

#include "halfnutThread.h"

// *** PRIVATE DEFINITIONS ***
enum HALFNUT_STATES  {
	HALFNUT_IDLE,
	HALFNUT_TURN_LEFT,
	HALFNUT_SLEW_LEFT,
	HALFNUT_TURN_RIGHT,
	HALFNUT_SLEW_RIGHT,
	HALFNUT_STOP,
	HALFNUT_INIT
};

// *** PRIVATE VARIABLES ***
enum HALFNUT_STATES HalfnutState = HALFNUT_INIT;
uint16 OldAtoDValue, AtoDValue;

// *** PUBLIC VARIABLES ***

// *** PRIVATE FUNCTIONS ***

// *** PUBLIC FUNCTIONS ***

/*
 *  FUNCTION: InitHalfNutDevice
 *
 *  PARAMETERS:		None
 *
 *  USES GLOBALS:	OldAtoDValue, HalfnutState
 *
 *  DESCRIPTION:	Initializes Half Nut device driver and AtoD
 *
 *  RETURNS: 		Nothing
 *
 */
void
InitHalfNutDevice(void) {
	InitAtoD(NULL);
	OldAtoDValue = GetAtoD( 0 ); 
	StartTimer(HALFNUT_TIMER, T_1SEC);
	HalfnutState = HALFNUT_INIT;
}

/*
 *  FUNCTION: HalfNutThread
 *
 *  PARAMETERS:		None
 *
 *  USES GLOBALS:	
 *
 *  DESCRIPTION:	Reads Halfnut pot A/D and uses the value to determine
 *					how to move the carriage.  
 *					1. Halfway point, carriage does nothing. [457..567] (110 counts)
 *					2. Left or right of halfway, carriage jogs at spindle 
 					   related turn speed  [568..657], [357,456]
					3. Less than 357 or greater than 657 spindle slews from 
					   current step 2 jog speed up to maximum slew speed.
					On startup,  Halfnut pot must be at IDLE position before
					state machine starts looking at it for motion.

 *
 *  RETURNS: 		Nothing
 *
 */
void 
HalfNutThread(void) {
  static uint16 startSpeed;
  uint8 ch;
  int32 speed;
  
	AtoDValue = GetAtoD( 0 ); 
	if ((AtoDValue != OldAtoDValue) && TimerDone(HALFNUT_TIMER)) {
		StartTimer(HALFNUT_TIMER, T_50MS);
		OldAtoDValue = AtoDValue;
		switch (HalfnutState) {

		  case HALFNUT_IDLE :
			if ((AtoDValue < 356) && (AtoDValue > 7)) { // Slew left but guard against open circuit
				DEBUGSTR("Slew Left\n");
				HalfnutState = HALFNUT_SLEW_LEFT;
			}
			else if ((AtoDValue >= 367) && (AtoDValue <= 456)) {
				startSpeed = MotorJog(MOTOR_Z, 0, MOVE_LEFT);
				DEBUGSTR( "Turn Left\n");
				HalfnutState = HALFNUT_TURN_LEFT;
			}
			else if ((AtoDValue >= 568) && (AtoDValue < 657)) {
				startSpeed = MotorJog(MOTOR_Z, 0, MOVE_RIGHT);
				DEBUGSTR( "Turn Right\n");
				HalfnutState = HALFNUT_TURN_RIGHT;
			}
			else if (AtoDValue >= 667)  { // Slew right
				DEBUGSTR( "Slew Right\n" );
				HalfnutState = HALFNUT_SLEW_RIGHT;
			}
			break;

		  case HALFNUT_TURN_LEFT :
			if ((AtoDValue >= 367) && (AtoDValue <= 456)) {
				startSpeed = MotorJog(MOTOR_Z, 0, MOVE_LEFT);
			}
			else if (AtoDValue < 356) {
				DEBUGSTR( "Slew Left\n" );
				HalfnutState = HALFNUT_SLEW_LEFT;
			}
			else if (AtoDValue > 456) {
				DEBUGSTR( "Stop\n" );
				HalfnutState = HALFNUT_STOP;
			}
			break;

		  case HALFNUT_SLEW_LEFT : 
			if (AtoDValue >= 356) { // Slewing stopping
				DEBUGSTR( "Stop\n" );
				HalfnutState = HALFNUT_STOP;
			}
			else {	// New Slew Speed so change it.
				speed = ((356 - AtoDValue) * 100) / 356;
				speed = ((long)GetGlobalVarWord(SLEW_RATE_Z_NDX) * speed) / 100L;
				if (speed < startSpeed) 
					speed = startSpeed;
				MotorJog(MOTOR_Z, speed , MOVE_LEFT);
			}
			break;

		  case HALFNUT_TURN_RIGHT :
			if ((AtoDValue >= 568) && (AtoDValue < 657)) {
				MotorJog(MOTOR_Z, 0, MOVE_RIGHT);
			}
			else if (AtoDValue >= 667) {
				DEBUGSTR( "Slew Right\n" );
				HalfnutState = HALFNUT_SLEW_RIGHT;
			}
			else if (AtoDValue < 567) {
				DEBUGSTR( "Stop\n" );
				HalfnutState = HALFNUT_STOP;
			}
			break;

		  case HALFNUT_SLEW_RIGHT :
			if (AtoDValue < 667) { // Slewing stopping
				DEBUGSTR( "Stop\n" );
				HalfnutState = HALFNUT_STOP;
			}
			else {	// New Slew Speed so change it.
				speed = ( (AtoDValue - 667) * 100) / 356;
				speed = (speed * (long)GetGlobalVarWord(SLEW_RATE_Z_NDX)) / 100L; 
				if (speed < startSpeed) 
					speed = startSpeed;
				MotorJog(MOTOR_Z,  speed, MOVE_RIGHT);
			}
			break;

		  case HALFNUT_STOP :	// STOP Z axis before we're allowed to go to a new mode.
			if ((AtoDValue > 456) && (AtoDValue <= 567)) {
				DEBUGSTR( "Idle\n" );
				HalfnutState = HALFNUT_IDLE;
			}
			MotorStop(MOTOR_Z);	
			break;

		  case HALFNUT_INIT :	// Test to see if half nut pot installed and at middle idle position.
			// If not just stay here and don't make system move.
			if ((AtoDValue > 456) && (AtoDValue < 567)) {
				DEBUGSTR( "Idle\n" );
				startSpeed = 0;
				HalfnutState = HALFNUT_IDLE;
			}
			break;
		}
	}
}
