/* 
    AtoD.c -- E-Leadscrew code for an electronic replacement of
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
// AtoD Support Functions for PIC18F

#include "processor.h"	// Processor specific Definitions.

#include <stdio.h>

#include "Common.h"	// Common constants.

#include "AtoD.h"	// AtoD Specific constants etc.

/*
 *  FUNCTION: InitAtoD
 *
 *  PARAMETERS:			pdata	-- Pointer to data structure for initializations.
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:		Sets up A/D for reading RA0
 *
 *  RETURNS: 
 *
 */
void 
InitAtoD(void * pdata) {
	if (pdata != NULL) {	// Later we'll use the data structure.
	}
	else {
		ADCON2 = RGHT_JUST | FTAD_20 | FOsc_64;	
		ADCON1 = 0x0E;		// All Digital but RA0 is analogue
		ADCON0 = 0x01;		// Enable A/D on port RA0. 
	}
}

/*
 *  FUNCTION: AtoDConvert
 *
 *  PARAMETERS:			AtoD_Channel	-- Channel to convert.
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:		Starts A/D and waits till done.
 *
 *  RETURNS: 
 *
 */
uint16 
AtoDConvert(BYTE AtoD_Channel) {
  uint16 res;
	ADCON0 = AtoD_Channel | 0x01; // Channel and enable
	Delay10TCYx(20);	// 20 * 10Tcy where 10Tcy is 2.0uS
	START_CONVERSION;			// Start conversion.
	_asm 	NOP 
			NOP
			NOP
	_endasm
	while(!DONE_CONVERSION);	// wait till done. (3.2uSec)
	res = ADRESH;
	res <<= 8;
	return( res | ADRESL);
}

/*
	Utility function to get most recent A/D converter value
*/
/*
 *  FUNCTION: GetAtoD
 *
 *  PARAMETERS:			Channel to get.
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:		Calls AtoDConvert to get A/D result
 *
 *  RETURNS: 			Returns channel.
 *
 */
int16 
GetAtoD(uint8 channel) {
	return(AtoDConvert(channel << 2));
}

