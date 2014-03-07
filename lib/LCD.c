/* 
    LCD.c --KeyButton code for an electronic replacement of
    gears used for threading or hobbing on a Lathe or Gear Hobber.

	This is the code that would be modified to provide support for
    a larger muxed keypad.

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
#include <math.h>
#include <delays.h>

#include "Common.h"			// Common constants.
#include "Config.h"			// Project specific definitions

#include "Serial.h"

#include "timer.h"
#include "beep.h"

#include "menu.h"
#include "LCD.h"	// Motor Driver Defintions.




/*  lcd.c - 8-Bit LCD Driver                                            */
//
//  Initial Version: 0.00 05Nov96

/* ********************************************************************* 
 *  These routines implement an 8-bit interface to a Hitachi            
 *  LCD module, busy flag used when valid.  The data lines              
 *  are on PORTD, E is on RE2, R/W is on RE0,           
 *  RS is on RE1.  
 *                                                                      
 * ********************************************************************* */

BITS LCDStatus;

uint8 Pos_x;  // @ 
uint8 Pos_y;  // @

uint8 LCDByte;

uint8 LCDDataIndex;
uint8 LCDCounter;
puint8 pLCDBuffer;

enum LCDStates LCDState;

const uint8 LCDInitData[LCD_INIT_BYTES] = {
		0b00111000,   		// set 8-bit interface
		0b00111000,       	// 8 bits, 2 lines, 5x7 dots
        0b00001110,         // display on, cursor on
        0b00000001,         // clear display
        0b00000110,         // set entry mode inc, no shift
        0b10000000          // Address DDRam upper left 
};
										   

/* ********************************************************************* */
/*  Busy                                                                 */
/*  This routine checks the busy flag.                                   */
/*  Returns a 1 when LCD is busy, or a 0 when the LCD is not busy.       */
// Note that constants are hard coded for 2 x N LCD display
// four line displays have odd addressing and require special code.
/* ********************************************************************* */
#define LCD_MAX_X1  19                  // 2 x 20 display.
#define LCD_MAX_X2  0x40+LCD_MAX_X1

int16 LCDBusy(void);   
void LCDSendData(uint8 ch);
void LCDSendBackSpace(void);


/*
 *  FUNCTION: LCDBusy
 *
 *  PARAMETERS:			None
 *
 *  USES GLOBALS:		PORTD, TRISD, LCDStatus
 *
 *  DESCRIPTION:		Busy Status a 1 or 0
 *
 *  RETURNS: 			
 *
 */
int16 
LCDBusy(void) {
    TRISD = 0xff;               // make LCDDATA all inputs
    PORTD = 0xff;
    bLCD_RS = 0;                    // setup LCD to output flags
    Delay10TCYx(1);
    bLCD_RW = 1;
    Delay10TCYx(1);
    bLCD_E = 1;
    Delay10TCYx(1);
    LCDStatus.Byte = LCDDATA;            // Save Rdy status & Cursor position.
    bLCD_E = 0;
    TRISD = 0x00;               // restore LCDDATA to outputs
	return(LCDStatus.Bit.Bit7);
}

/*
 *  FUNCTION: LCDSendData
 *
 *  PARAMETERS:			ch -- Character to send to LCD
 *
 *  USES GLOBALS:	
 *
 *  DESCRIPTION:		Writes data byte to LCD.
 *
 *  RETURNS: 			Nothing.
 *
 */
void 
LCDSendData(uint8 ch) {
	while (LCDBusy())
		;
    LCDDATA = ch & 0x7F;    // load LCDDATA with byte
  	bLCD_RW = 0;            // send character to LCD
    Delay10TCYx(5);
  	bLCD_RS = 1;
    Delay10TCYx(5);
    bLCD_E = 1;
    Delay10TCYx(5);
    bLCD_E = 0;
	Delay10TCYx(200);
	while (LCDBusy())		// Update LCD Cursor position.
		;
}

/*
 *  FUNCTION: LCDSendCmd
 *
 *  PARAMETERS:			ch	-- What to send to Command Register
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:		This routine sends the command in byte to the LCD.
 *
 *  RETURNS: 			Nothing
 *
 */
void 
LCDSendCmd(uint8 ch)
{
	while (LCDBusy())
		;
	LCDDATA = ch;          // load LCDDATA with byte
    bLCD_RW = 0;            // send command byte to LCD
    Delay10TCYx(5);
    bLCD_RS = 0;
    Delay10TCYx(5);
    bLCD_E = 1;
    Delay10TCYx(5);
    bLCD_E = 0;
	Delay10TCYx(200);
	while (LCDBusy())		// Update LCD Cursor position.
		;
}

/*
 *  FUNCTION: LCDSetPosition
 *
 *  PARAMETERS:		X and Y position on LCD screen
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:	Sets the X,Y position for LCD.
 *
 *  RETURNS: 		Nothing
 *
 */
void
LCDSetPosition(uint8 x, uint8 y) {
    if (y==1) 
        LCDByte = 0xC0;   // Command bit and offset to second line.
    else 
        LCDByte = 0x80;   // Just command bit.
    LCDByte += x;
    LCDSendCmd(LCDByte);     // Set DD ram address
 }


/*
 *  FUNCTION: LCDSendBackSpace
 *
 *  PARAMETERS:			None
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:		Backspace LCD cursor one place.
 *
 *  RETURNS: 			Nothing
 *
 */
void 
LCDSendBackSpace(void) {
	LCDStatus.Byte--;
	if (LCDStatus.Byte >= 0x40) {
		LCDSetPosition((LCDStatus.Byte - 0x40),1);
	}
	else {
		if (LCDStatus.Byte > 19) 
			LCDSetPosition(19,0);	
		else if (LCDStatus.Byte == 0xFF) 
			LCDSetPosition(0,0);
		else
			LCDSetPosition(LCDStatus.Byte, 0);
	}
}


/*
 *  FUNCTION: LCDSendChar
 *
 *  PARAMETERS:			ch 	-- character to send.
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:		This routine sends the character in byte to the LCD.
 *
 *  RETURNS: 			Nothing
 *
 */
void 
LCDSendChar(uint8 ch) {
	while (LCDBusy())
		;
	
	if (ch == '\b') {
		LCDSendBackSpace();
		LCDSendData(' ');
		LCDSendBackSpace();
	}		
	else {		 
	    if  (LCDStatus.Byte > LCD_MAX_X2 ) {     // at end of screen
    	    LCDSetPosition(0,0);   // Home Position
	    }     
    	else if ((LCDStatus.Byte > LCD_MAX_X1) && (LCDStatus.Byte < 40)) { // next line
        	LCDSetPosition(0,1);
	    };
		LCDSendData(ch);
	}
}


/*
 *  FUNCTION: PositionCursor
 *
 *  PARAMETERS:		Screen position.
 *
 *  USES GLOBALS:	
 *
 *  DESCRIPTION:	Sets cursor at Menu Index Location.
 *
 *  RETURNS: 
 *
 */
void 
PositionCursor(int  MenuIndex) {
	Pos_y = MenuIndex/DISPLAY_COLUMNS;
	Pos_x = MenuIndex%DISPLAY_COLUMNS;
	LCDSetPosition(Pos_x, Pos_y);
}

/*
 *  FUNCTION: LCDSendBuf
 *
 *  PARAMETERS:			pstr	-- Pointer to ram string to put on LCD.
 *						
 *  USES GLOBALS:
 *
 *  DESCRIPTION:		Puts string to LCD and interprets ESC=RowCol sequence.
 *
 *  RETURNS: 
 *
 */
void 
LCDSendBuf(puint8 pstr) {
	while (*pstr) {  
		while (LCDBusy())
			; 
		LCDByte = *pstr++;
        if (LCDByte) {    // All characters sent?
			// Check for embedded commands
			switch (LCDByte) {
			  case 0x1B : // Cursor positioning ESC=RowCol
				LCDCounter -= 4;
				pstr++;	// Past '='
				Pos_y = *pstr++ - 0x20;
				Pos_x = *pstr++ - 0x20;
				LCDSetPosition(Pos_x, Pos_y); 
				Delay10TCYx(50);
				break;
			  default:
				LCDSendChar(LCDByte);
				break;
			}
        }
	}
}

/*
 *  FUNCTION: LCDSendStr
 *
 *  PARAMETERS:			pstr 	-- Pointer to Rom String
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:		Puts ROM string to LCD interpretting ESC=RowCol
 *
 *  RETURNS: 
 *
 */
void 
LCDSendStr(const rom char * pstr) {
	while (*pstr) {  
		while (LCDBusy())
			; 
		LCDByte = *pstr++;
        if (LCDByte) {    // All characters sent?
			// Check for embedded commands
			switch (LCDByte) {
			  case 0x1B : // Cursor positioning ESC=RowCol
				LCDCounter -= 4;
				pstr++;	// Past '='
				Pos_y = *pstr++ - 0x20;
				Pos_x = *pstr++ - 0x20;
				LCDSetPosition(Pos_x, Pos_y); 
				Delay10TCYx(50);
				break;
			  default:
				LCDSendChar(LCDByte);
				break;
			}
        }
	}
}

/*
 *  FUNCTION: InitLCD
 *
 *  PARAMETERS:		None
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION: 	This routine initializes the LCD module and ports.
 *
 *  RETURNS: 		Nothing
 *
 */
void 
InitLCD(void) {
    for (LCDCounter = 0; LCDCounter < LCD_INIT_BYTES; LCDCounter++) { // Init Bytes.
    	LCDByte = LCDInitData[LCDCounter];
        LCDSendCmd(LCDByte);
		 Delay10KTCYx(50);
	}
    LCDSetPosition(0,0);
	
	LCDSendStr((const rom char *)pstrSignOn);

}


/*
 *  FUNCTION: _user_putc
 *
 *  PARAMETERS:			c	-- Does nothing.
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:		Does nothing for now.
 *
 *  RETURNS: 			Character argument.
 *
 */
int 
_user_putc (int8 c) {
//	if (c=='\n')
//		PutSerial('\r');
//	PutSerial(c);
	return c;
}
