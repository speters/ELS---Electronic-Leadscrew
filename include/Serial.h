/* 
    Serial.h -- E-Leadscrew definitions for an electronic replacement of
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
// Serial.h -- Definitions and constants.


#define SXTAL24576	3840		// Use BRGH = 0
#define SXTAL4		6250
#define	SXTAL49152	7680
#define	SXTAL110592	17280
// 20MHz/(16*100)
#define	SXTAL20		12500		// Use BRGH = 1
#define	SXTAL40		25000		// Use BRGH = 1


#ifdef BRGH_1

// Values for the SPBRG register with BRGH=1
#define B9600	((SERIAL_XTAL/96)-1)		
#define B56K	(SERIAL_XTAL/576)
#define B115K   (SERIAL_XTAL/1156)		
// Value for TXSTA
#define TXINIT		0x24	// TXEN=1, BRGH=1

#else

#define B9600	((SERIAL_XTAL/96)-1)		
// Value for TXSTA
#define TXINIT		0x20	// TXEN=1, BRGH=0

#endif

// Value for RXSTA
#define RXINIT		0x80	// SPEN=1,  RX disabled.


#define pstrClearLine "            \r"

#define SERIAL_INBUFFER_SIZE  32

#define SERIAL_OUTBUFFER_SIZE  64
#define	SERIAL_OUTBUFFER_MASK	SERIAL_OUTBUFFER_SIZE-1

// Communications bit Flags
extern BITS CommFlags;
#define fNET_FLAG           CommFlags.Bit.Bit2
#define fMSG_DONE_FLAG      CommFlags.Bit.Bit1
#define fECHO_FLAG			CommFlags.Bit.Bit0		// Set if input characters must be echoed.

extern char OutputBuffer[SERIAL_OUTBUFFER_SIZE];

void InitSerial(uint8 baud);				// Sets baud rate etc.
void TxCharDevice(void);
void putchar(int8 ch);
void PutCRLF(void);
void PutString( const rom pint8 ptr );
void PutBuffer(void);
void PutInputLine(void);
uint8 getc(void);
puint8 GetLine(void);
void ungetc(void);
void RxCharDevice(void);
uint8 GetHexNibl(void);
uint8 GetHexByte(void);
void SetEchoFlag(uint8 flg);

void GetHexLong(void);
uint16 GetHexWord(void);
uint16 GetWord(void);
