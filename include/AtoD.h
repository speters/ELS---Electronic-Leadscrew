/* 
    AtoD.h -- AtoD definitions for an electronic replacement of
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
// AtoD.h Definitions file.


/* ---- PRIVATE ---- */
#define Enable_A2D_Int       INTCONbits.ADIE = 1
#define Clr_A2D_Int          ADCON0bits.ADIF = 0
#define Turn_A2D_On          ADCON0bits.ADON = 1
#define Turn_A2D_Off         ADCON0bits.ADON = 0
#define START_CONVERSION     ADCON0bits.GO   = 1
#define DONE_CONVERSION      !ADCON0bits.DONE
#define A2D_Interrupt        PIR1bits.ADIF


#define Ch0         0b00000000
#define Ch1         0b00001000
#define Ch2         0b00010000
#define Ch3         0b00011000
#define Ch4         0b00100000
#define Ch5         0b00101000
#define Ch6         0b00110000
#define Ch7         0b00111000

#define FOsc_4      0b00000100 // fosc/4
#define FOsc_16     0b00000101 // fosc/16
#define FOsc_64     0b00000110 // fosc/64
#define FRCL         0b00000111 // RC osc.
#define FOsc_2      0b00000000 // fosc/2
#define FOsc_8      0b00000001 // fosc/8
#define FOsc_32     0b00000010 // fosc/32
#define FRCH         0b00000011 // RC osc.

#define	FTAD_20		0b00111000 // 20 TAD
#define	FTAD_16		0b00110000 // 16 TAD
#define	FTAD_12		0b00101000 // 12 TAD
#define	FTAD_8		0b00100000 // 8 TAD
#define	FTAD_6		0b00011000 // 6 TAD
#define	FTAD_4		0b00010000 // 4 TAD
#define	FTAD_2		0b00001000 // 2 TAD
#define	FTAD_0		0b00000000 // 0 TAD(1)

#define RGHT_JUST	0b10000000 // Right Justified
	

/* ---- PUBLIC ---- */
#define A_CHAN0		0
#define A_CHAN1		1<<2
#define	A_CHAN2		2<<2
#define A_CHAN3		3<<2
#define A_CHAN4		4<<2

#define MAX_A_D_CHANNELS	1

void InitAtoD(void * pdata);
uint16 AtoDConvert(uint8 AtoD_Channel);
int16 GetAtoD(uint8 channel);

