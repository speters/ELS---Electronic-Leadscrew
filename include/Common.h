/* 
    Common.h -- Common definitions for an electronic replacement of
    gears used for threading or hobbing on a Lathe or Gear Hobber.

    Copyright (C) 2005,2006  John Dammeyer

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
// Common.h -- Common definitions and constants used in all files.
#define 	MEM_MODEL	far

#define     BYTE unsigned char
#define 	WORD unsigned int
#define 	ULONG unsigned long
#define 	PBYTE BYTE *
#define 	PWORD WORD *

#define		int8	char
#define 	int16	int
#define		int32	long
#define		float32	float

#define 	uint8	unsigned char
#define		uint16	unsigned int
#define		uint32	unsigned long

#define		pint8	char *
#define 	pint16	int *
#define		pint32	long *
#define		pfloat32 float *

#define 	puint8	unsigned char *
#define		puint16	unsigned int *
#define		puint32	unsigned long *

#define 	TRUE	-1
#define 	FALSE	0

typedef union tag_bits
{
  struct
  {
    unsigned   Bit0:1;
    unsigned   Bit1:1;
    unsigned   Bit2:1;
    unsigned   Bit3:1;
    unsigned   Bit4:1;
    unsigned   Bit5:1;
    unsigned   Bit6:1;
    unsigned   Bit7:1;
  } Bit;
  unsigned char Byte;
} BITS;

typedef union  {
	float f;
	long l;
	WORD w;
	BYTE b;
} PARAMETERS;	

