// Electronic Spindle Speed Control Device Thread
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

#include "SpindleSpeed.h"

// *** PRIVATE DEFINITIONS ***

// *** PRIVATE VARIABLES ***

// *** PUBLIC VARIABLES ***
WORD SpindleSpeed;
WORD SpindleSpeedHolding;

// *** PRIVATE FUNCTIONS ***

// *** PUBLIC FUNCTIONS ***

/*
 *  FUNCTION: InitSpindleDevice
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
InitSpindleDevice(void) {
  int i;
	InitAtoD(NULL);
	i = GetAtoD( 0 ); 
	SpindleSpeed = 0;
	SpindleSpeedHolding = 0;
}

/*
 *  FUNCTION: SpindleDevice
 *
 *  PARAMETERS:		None
 *
 *  USES GLOBALS:	
 *
 *  DESCRIPTION:	Reads Halfnut pot A/D and uses the value to set the spindle
 *					step rate.
 *					1. Halfway point, carriage does nothing. [457..567] (110 counts)
 *
 *  RETURNS: 		Nothing
 *
 */
void 
SpindleDevice(void) {
  int32 speed;
  
	speed = GetAtoD( 0 ); // (0..1023)
	speed *= 195;		  // Scale
	speed += 515;		  // Add in offset and round.
	speed /= 10;		  // Undo scaling.
	
	INTCON &= 0x3F;				// Done with interrupts off.
	SpindleSpeed = speed;
	INTCON |= 0xC0;
}


void
SetSpindleSpeed(int32 spd) {
	INTCON &= 0x3F;				// Done with interrupts off.
	SpindleSpeed = spd;
	INTCON |= 0xC0;
}