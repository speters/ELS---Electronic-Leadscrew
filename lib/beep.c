/* 	Beep.c --Beeper code for an electronic replacement of
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

// Bring in processor definition and math library
#include "processor.h"

#include <Ctype.h>
#include <stdio.h>
#include <stdlib.h>

#include "Common.h"			// Common constants.
#include "Config.h"			// Project specific definitions

#include "timer.h"

#include "Beep.h"	

// Beeper routines using digital on/off to make beep.
// Beep.c

// GLOBALS
uint8 BeepMsg;

// LOCALS 
enum BeepStates {
	BEEP_START,
	BEEP_OFF,
	BEEP_ON
} BeepState;


uint8 BeepTime;
uint8 BeepRep;


/*
 ** StartBeep()
 *
 *  FILENAME: beep.c
 *
 *  PARAMETERS:  None
 *
 *  DESCRIPTION:	Sets output line to low to turn on beeper.
 *
 *  RETURNS: Nothing
 *
 */
void 
StartBeep(void) {
	bBEEP = 0;
}

/*
 ** StopBeep()
 *
 *  FILENAME: beep.c
 *
 *  PARAMETERS: None
 *
 *  DESCRIPTION: Sets output line hight to turn off beeper.
 *
 *  RETURNS: Nothing
 *
 */
void 
StopBeep(void) {
	bBEEP = 1;
}

/*
 ** InitBeeper()
 *
 *  FILENAME: beep.c
 *
 *  PARAMETERS: None
 *
 *  DESCRIPTION: Turns off Beeper and sets up command variables for Beeper state machine.
 *
 *  RETURNS:
 *
 */
void 
InitBeepDevice(void) {
	StopBeep();
	BeepMsg = 0;
   	BeepState = BEEP_START;
}


/*
 ** Beeper()
 *
 *  FILENAME: beep.c
 *
 *  PARAMETERS:	BeepMsg.  Holds length of beep, [4:0] and number of
 *						  beeps, [7:5]
 *
 *  DESCRIPTION:  When BeepMsg != 0 then it splits the uint8 into the two
 *				  parameters needed and starts the beep sequence.
 *
 *  RETURNS: Nothing.
 *
 */
void 
BeeperDevice() {

    switch (BeepState) {

      case BEEP_START :
	  	if (BeepMsg != 0) {
            // Set how long beeper is on and off.
            BeepTime = (BeepMsg & 0x1F) << 2;  // times 4 for 40ms to 1.24 sec.
            // Set how many times we beep
            BeepRep = (BeepMsg & 0xE0) >> 5;
	        StartBeep();
            StartTimer( BEEP_TIMER, BeepTime );
            BeepMsg = 0;
            BeepState = BEEP_OFF;
        }
        break;

      case BEEP_OFF :
        if (TimerDone( BEEP_TIMER ) ) {
            StopBeep();
            BeepRep--;
            StartTimer( BEEP_TIMER, BeepTime );
            BeepState = BEEP_ON;
        };    
        break;

      case BEEP_ON:
        if (TimerDone( BEEP_TIMER ) ) {
            if ( BeepRep != 0 ) {
	            StartBeep();
	            StartTimer( BEEP_TIMER, BeepTime );
	            BeepState = BEEP_OFF;
            }
            else {
                BeepState = BEEP_START;  // Beeping is done.
            }
        }
        break;

      default:
		BeepState = BEEP_START;
        break;
    }
}
