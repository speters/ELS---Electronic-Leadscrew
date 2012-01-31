/* 	Beep.h -- Beeper Definitions for an electronic replacement of
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

// Beep.h

/*
// Not finding the Enterprise leaves us stranded with the Borg.
#define		BEEP_KEY	174		// Key press feedback.
#define 	BEEP_ATTN	150		// Attention Tone - Something happened 
#define		BEEP_WARN	100		// Warning Tone - Something will happen
#define		BEEP_ALARM	75		// Alarm Tone - Something bad happened.
#define		BEEP_BAD	40		// Start Trek Communicator can't find starship.


#define 	BEEP_PERIOD	128		// 50% duty cycle.
#define 	PWM_BEEP	0x0C	// Enables PWM mode.
*/
#define 	ONE_BEEP	(1<<5)	// One time to beep.
#define 	TWO_BEEPS	(2<<5)	// Two times to beep.
#define		THREE_BEEPS (3<<5)  // Three times to beep.
#define		FOUR_BEEPS  (4<<5)  // Four times to beep.

extern uint8 BeepMsg;
//extern uint8 BeepType;

void InitBeepDevice(void);
void BeeperDevice(void);
