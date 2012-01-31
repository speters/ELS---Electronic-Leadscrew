/* 
    Ob_eerom.c -- E-Leadscrew code for an electronic replacement of
    gears used for threading or hobbing on a Lathe or Gear Hobber.

	This code reads from and writes to EEROM as Bytes, Ints  
	and floats.

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
// OnBoard EEROM Utilities.
// 

#include "processor.h"
#include <float.h>
#include <math.h>

#include "common.h"
#include "config.h"
#include "OB_EEROM.h"

#define Onboard_EEROM 1

uint16 EEROMAddress;
uint16 EEROMIndex;

#ifdef Onboard_EEROM 

/*
 *  FUNCTION: Get_ObEEROM_Buffer
 *
 *  PARAMETERS:			addr	-- Where in EEROM
 *						pdata 	-- Pointer to destination buffer
 *					 	len		-- How many bytes
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:		Get data from EEROM into buffer.
 *
 *  RETURNS: 			Nothing
 *
 */
void 
Get_ObEEROM_Buffer(uint16 addr, puint8 pdata, uint8 len) {
	while (len != 0) {
		*pdata++ = Get_ObEEROM_Byte(addr++);
	} 
}

/*
 *  FUNCTION: Get_ObEEROM_Float
 *
 *  PARAMETERS:			addr	-- Where in EEROM
 *						pdata	-- Where in memory
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:		Gets a Float from EEROM into memory
 *
 *  RETURNS: 			Nothing 
 *
 */
void 
Get_ObEEROM_Float(uint16 addr, float32 * pdata) {
  puint8 pd = (puint8)pdata;
	*pd++ = Get_ObEEROM_Byte(addr++);
	*pd++ = Get_ObEEROM_Byte(addr++);
	*pd++ = Get_ObEEROM_Byte(addr++);
	*pd = Get_ObEEROM_Byte(addr);
}		


/*
 *  FUNCTION: Get_ObEEROM_Word
 *
 *  PARAMETERS:			addr	-- Where in EEROM
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:		Gets a Word from EEROM into memory
 *
 *  RETURNS: 			Word 
 *
 */
uint16 
Get_ObEEROM_Word(uint16 addr) {
	EEROMIndex = Get_ObEEROM_Byte(addr);
	EEROMIndex |= ((uint16)Get_ObEEROM_Byte(addr+1) << 8);
	return(EEROMIndex);
}

/*
 *  FUNCTION: Get_ObEEROM_Word
 *
 *  PARAMETERS:			addr	-- Where in EEROM
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:		Gets a Byte from EEROM into memory
 *
 *  RETURNS: 			Byte 
 *
 */
uint8 
Get_ObEEROM_Byte(uint16 addr) {
	INTCON &= 0x3F;
	EEADR = addr;
	EEADRH = addr >> 8;
	EECON1bits.EEPGD = 0;
	EECON1bits.CFGS = 0;
	EECON1bits.RD = 1;
	INTCON |= 0xC0;
	return(EEDATA);
}

/*
 *  FUNCTION: Put_ObEEROM_Byte
 *
 *  PARAMETERS:			addr	-- Where in EEROM
 *						data	-- What
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:		Write byte to EEROM
 *
 *  RETURNS: 			Nothing
 *
 */
void 
Put_ObEEROM_Byte( uint16 addr, uint8 data ) {
	INTCON &= 0x3F;
	EEADR = addr;
	EEADRH = addr >> 8;
	EEDATA = data;
	EECON1bits.CFGS = 0;
	EECON1bits.WREN = 1;
	EECON1bits.EEPGD = 0;
	EECON2 = 0x55;
	EECON2 = 0xAA;
	EECON1bits.WR = 1;
	INTCON |= 0xC0;
	while (!PIR2bits.EEIF)
		;
	PIR2bits.EEIF = 0;
	EECON1bits.WREN = 0;
}

/*
 *  FUNCTION: Put_ObEEROM_Word
 *
 *  PARAMETERS:			addr	-- Where in EEROM
 *						data	-- What
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:		Write Word to EEROM
 *
 *  RETURNS: 			Nothing
 *
 */
void 
Put_ObEEROM_Word( uint16 addr, uint16 data ) {
	Put_ObEEROM_Byte(addr++, (uint16)data );
	Put_ObEEROM_Byte(addr, data >> 8);
}

/*
 *  FUNCTION: Put_ObEEROM_Float
 *
 *  PARAMETERS:			addr	-- Where in EEROM
 *						pdata	-- Pointer to What
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:		Write Float to EEROM
 *
 *  RETURNS: 			Nothing
 *
 */
void 
Put_ObEEROM_Float( uint16 addr, float * pdata ) {
 puint8 pd = (puint8)pdata;
	Put_ObEEROM_Byte(addr++, *pd++ );
	Put_ObEEROM_Byte(addr++, *pd++ );
	Put_ObEEROM_Byte(addr++, *pd++ );
	Put_ObEEROM_Byte(addr, *pd );
}

/*
 *  FUNCTION: Put_ObEEROM_Buffer
 *
 *  PARAMETERS:			addr	-- Where in EEROM
 *						pdata	-- Pointer to What
 *						len		-- How many bytes
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:		Write buffer to EEROM
 *
 *  RETURNS: 			Nothing
 *
 */
void 
Put_ObEEROM_Buffer(uint16 addr, puint8 pdata, uint8 len) {
	while (len != 0) {
		Put_ObEEROM_Byte(addr++, *pdata++);
	} 
}


#endif
