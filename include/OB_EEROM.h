#ifndef __EE18_H
#define __EE18_H
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


extern volatile near uint8	EECON1;
extern volatile near union {
  struct {
    unsigned RD:1;
    unsigned WR:1;
    unsigned WREN:1;
    unsigned WRERR:1;
    unsigned FREE:1;
    unsigned EECON1_5:1;
    unsigned CFGS:1;
    unsigned EEPGD:1;
  };
} EECON1Bits;


//#define Enable_EE_WR_Int   INTCON.EEIE = 1
#define Enable_EE_WR       EECON1Bits.WREN = 1
#define Disable_EE_WR      EECON1Bits.WREN = 0
#define EEProm_Rd          EECON1Bits.RD   = 1
//#define Global_Int_Disable INTCON.GIE  = 0
//#define Global_Int_Enable  INTCON.GIE  = 1
//#define Clr_EE_Int         INTCONBits.EEIF = 0
#define EEProm_WR_Err      EECON1Bits.WRERR
#define Do_EE_Write        _asm BSF 0x08,1,0 _endasm
#define EEProm_Writing     EECON1Bits.WR
//#define EEProm_WR_Int      EECON1.EEIF

extern uint16 EEROMAddress;
#define EEROMAdrHigh (uint8)(EEROMAddress>>8)	
#define EEROMAdrLow  (uint8)EEROMAddress		
extern uint16 EEROMIndex;

void Put_ObEEROM_Byte( uint16 addr, uint8 data);
void Put_ObEEROM_Word( uint16 addr, uint16 data );
void Put_ObEEROM_Float( uint16 addr, float32 * pdata );
uint8 Get_ObEEROM_Byte(uint16 addr);
uint16 Get_ObEEROM_Word(uint16 addr);
void Get_ObEEROM_Float(uint16 addr, float32 * pdata);

void Get_ObEEROM_Buffer(uint16 addr, puint8 pdata, uint8 len); 
void Put_ObEEROM_Buffer( uint16 addr, puint8 pdata, uint8 len );
#endif
