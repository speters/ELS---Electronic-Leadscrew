/* 
    Key.c --KeyButton code for an electronic replacement of
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

#include "Common.h"			// Common constants.
#include "Config.h"			// Project specific definitions

#include "timer.h"
#include "beep.h"

#include "Key.h"	// Motor Driver Defintions.


// Key Switch Device Driver.

/* ------------  Public Data -------------*/
uint16 KeyValue;
int8 KeyNumberIndex;
int8 KeyMaxLength;
uint8 KeyNumberBuffer[MAX_INPUT_BUFFER_SIZE];


/* ------------  Private Declarations and Data -------------*/

enum KeyStates {
	KEY_STARTING,
	KEY_ON,
	KEY_BLINK_ON,
	KEY_BLINK_OFF,
	KEY_OFF
} KeyState;


uint16 CurrentKey;
uint8 KeyTime;
uint8 KeyRep;

/*
 *  FUNCTION: InitKeyDevice
 *
 *  PARAMETERS:		None
 *
 *  USES GLOBALS:		    
 *			KeyState
 *			KeyValue
 *			bKEY_ROW
 *			bKEY_COL
 *
 *  DESCRIPTION:	Initializes key device variables.
 *
 *  RETURNS: 		Nothing
 *
 */
void 
InitKeyDevice(void) {
    KeyState = KEY_STARTING;
	KeyValue = KEY_RELEASED;	// No key pressed yet.
	bKEY_ROW = 0;
	bKEY_COL = 1;
}

#ifdef BOOTLOADER_COMPILE
/*
	CheckEnter:
	Code fragment to check if Enter Key down.  Compiled to assembler which is then
	copied into Bootloader.asm.  This code doesn't really exist in ELS application. 
*/
/*
 *  FUNCTION: CheckEnter
 *
 *  PARAMETERS:			None
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:		Code fragment to check if Enter Key down.  Compiled to assembler 
 *						which is then copied into Bootloader.asm.  This code doesn't 
 *						really exist in ELS application. 
 *
 *  RETURNS: 			Nothing
 *
 */
uint8 
CheckEnter(void) {
	PORTC = 0;			// All PortC outputs low.
	TRISC = 0x90;		// 0..2 out, 3-CLK, 4-MISO, 5-MOSI, 6-TX, 7-RX.
	PORTD = ~0x10;		// Row 4 low.
	TRISD = 0;			// PORT D Outputs
	bKEY_ROW = 1;		// Latch into 74HC374
	bKEY_ROW = 0;		// By Strobing RC1
	TRISD = 0xFF;		// Read column input from 74HC244A.
	bKEY_COL = 0;		// By gating using RC2
	if (PORTDbits.RD3 == 0)	// Test ENTER Key.  
		return 1;		// Enter Key Pressed 
	else 
		return 0;		// Enter Key not pressed.
}
#endif

#define MAX_ROW		5
#define MAX_COL		7

const uint8 Mask[MAX_COL] = {1,  2,  4,  8, 16, 32, 64};
//                       row 0   1   2   3   4   5,  6

/*
	RdKey: Scan through all rows and columns.
	The boot loader does the same thing but looking for only one key.
*/

/*
 *  FUNCTION: RdKey
 *
 *  PARAMETERS:			None
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:		Scan through all rows and columns. The boot loader does 
 *						the same thing but looking for only one key.
 *
 *  RETURNS: 			A value between 0..0x3F for key# and bit 8 set if ALT key was
 *						down.
 *	
 */
uint16
RdKey(void) {
  static uint8 row = 0x00;
  int8 col;
  int16 alt = 0;
  uint8 key = 0xff;
  for (row = 0; row<5; row++) {
	TRISD = 0x00;	// Output Port
	PORTD = ~Mask[row];	// Put a 0 in a row.
	// Allow signal to settle.		
	{_asm nop nop nop _endasm}	bKEY_ROW = 1;		// Latch into 74HC374
	Nop();
	bKEY_ROW = 0;
	Nop();
	TRISD = 0xFF;	// Read column input from 74HC244A.
	bKEY_COL = 0;
	{_asm nop nop nop _endasm}	key = PORTD;	// A low in a column means key pressed on specific row
	bKEY_COL = 1;

	for (col=0; col<MAX_COL; col++) {  // Check each column position to see if a low is that place 
		if ( (key & Mask[col]) == 0 ) {
			key = row * MAX_COL + col;	
			// Check if ALT key and ignore other than setting flag.
			if (key == BUTTON_ALT) 
				alt = KEY_ALT;
			else {
				return((int16)key | alt); // Add in alt key status.
			}
		}
	}	
  } 	
  return((int16)key & 0x3F);
}

/*
 * Scan the keypad or buttons and return port image.
 */
/*
 *  FUNCTION: GetKeys
 *
 *  PARAMETERS:			pAltKey		-- pointer to flag for ALT key down.
 *
 *  USES GLOBALS:		calls RdKey()
 *
 *  DESCRIPTION:		Calls RdKey() to get key # and then returns 0..0x3F
 *						for key number and saves ALT Key status in *pAltKey
 *						Note!  Don't put any serial output debugging in this function
 *						because it's called before the serial port is initialized to
 *						test for DEL key and automatic reload of default parameters.
 *
 *  RETURNS: 			key # from 0..0x3F
 *						*pAltKey holds flag set for ALT Key Down.
 *
 */
uint16
GetKeys(int8 * pAltKey) {
uint16 k;
	k = RdKey();
	*pAltKey = k >> 8;
    return(k & KEY_MASK);
}

/*
 *  FUNCTION: KeyDevice
 *
 *  PARAMETERS:			None
 *
 *  USES GLOBALS:		KeyValue	-- Key pressed value.
 *						BeepMsg		-- Number of beeps and duration.
 *
 *  DESCRIPTION:		State machine to handle Key Debouncing, Key Pressed,
 *						Key Held down and ALT Key plus key down.
 *						Translates Key Scan Code to a key button value.
 *						
 *
 *  RETURNS: 			Sends a BEEP message to Beeper Device. 
 *						KeyValue holds pressed key.
 *
 */
void 
KeyDevice(void) {
  uint16 kdch;
  int8 altkeypressed;
    switch (KeyState) {

    case KEY_STARTING :
	  	if (GetKeys(&altkeypressed) != KEY_MASK) {  // We have a key pressed.
            StartTimer( KEY_TIMER, T_KEY_DEBOUNCE );
			// All key presses must be longer than 50 ms.
            KeyState = KEY_ON;
        }
      	break;

	case KEY_ON :
		if (TimerDone( KEY_TIMER )) {
			if ( (CurrentKey = GetKeys(&altkeypressed)) != KEY_MASK)  {	// Valid Key Pressed

//				printf((MEM_MODEL rom char *)" KeyDevice:CurrentKey %04X\n", CurrentKey);

				BeepMsg = T_KEY_PRESSED | ONE_BEEP;

				KeyState = KEY_BLINK_ON;

				switch ( CurrentKey & KEY_MASK ) {  // Which key(s) pressed?

				case BUTTON_0 :
					KeyValue = KEY_PRESSED | KEY_0;
					break;

				case BUTTON_1 :
					KeyValue = KEY_PRESSED | KEY_1;
					break;

				case BUTTON_2 :
					KeyValue = KEY_PRESSED | KEY_2;
					break;

				case BUTTON_3 :
					KeyValue = KEY_PRESSED | KEY_3;
					break;

				case BUTTON_4 :
					KeyValue = KEY_PRESSED | KEY_4;
					break;

				case BUTTON_5 :
					KeyValue = KEY_PRESSED | KEY_5;
					break;

				case BUTTON_6 :
					KeyValue = KEY_PRESSED | KEY_6;
					break;

				case BUTTON_7 :
					KeyValue = KEY_PRESSED | KEY_7;
					break;

				case BUTTON_8 :
					KeyValue = KEY_PRESSED | KEY_8;
					break;

				case BUTTON_9 :
					KeyValue = KEY_PRESSED | KEY_9;
					break;

				case BUTTON_PERIOD :
					KeyValue = KEY_PRESSED | KEY_PERIOD;	// '.'
					break;

				case BUTTON_DEL :
					KeyValue = KEY_PRESSED | KEY_DEL;	// 
					break;

				case BUTTON_SFN1 :
					KeyValue = KEY_PRESSED | KEY_SFN1;	// DC1
					break;

				case BUTTON_SFN2 :		
					KeyValue = KEY_PRESSED | KEY_SFN2;	// DC2
					break;

				case BUTTON_SFN3 :
					KeyValue = KEY_PRESSED | KEY_SFN3;	// DC3
					break;

				case BUTTON_SFN4 :
					KeyValue = KEY_PRESSED | KEY_SFN4;	// DC4
					break;

				case BUTTON_UP :
					KeyValue = KEY_PRESSED | KEY_UP;	// LF
					break;

				case BUTTON_DOWN :
					KeyValue = KEY_PRESSED | KEY_DOWN;	// FF
					break;

				case BUTTON_ESC :
					KeyValue = KEY_PRESSED | KEY_ESC;	// <ESC>
					break;

				case BUTTON_ENTER :
					KeyValue = KEY_PRESSED | KEY_ENTER;	// <CR>
					break;

				case BUTTON_START :
					KeyValue = KEY_PRESSED | KEY_START;	// SO
					break;

				case BUTTON_STOP :
					KeyValue = KEY_PRESSED | KEY_STOP;	// SI
					break;

				case BUTTON_THREAD :
					KeyValue = KEY_PRESSED | KEY_THREAD;	// FS
					break;

				case BUTTON_TURN :
					KeyValue = KEY_PRESSED | KEY_TURN;	// GS
					break;

				case BUTTON_SET_BEGIN :
					KeyValue = KEY_PRESSED | KEY_SET_BEGIN;  // '('
					break;

				case BUTTON_SET_END :
					KeyValue = KEY_PRESSED | KEY_SET_END;	// ')'
					break;

				case BUTTON_LZFAST :
					KeyValue = KEY_PRESSED | KEY_LFAST;	// '<'
					break;

				case BUTTON_RZFAST :
					KeyValue = KEY_PRESSED | KEY_RFAST;	// '>'
					break;

				case BUTTON_LZJOG :
					KeyValue = KEY_PRESSED | KEY_LZJOG;	// ':'
					break;

				case BUTTON_RZJOG :
					KeyValue = KEY_PRESSED | KEY_RZJOG;	// ';'
					break;

				case BUTTON_XJOG_IN :
					KeyValue = KEY_PRESSED | KEY_XJOG_IN;	// ':'
					break;

				case BUTTON_XJOG_OUT :
					KeyValue = KEY_PRESSED | KEY_XJOG_OUT;	// ';'
					break;

				case BUTTON_SET_ZHOME :
						KeyValue =  KEY_PRESSED | KEY_SET_ZHOME;
						if (altkeypressed)  // Alt key pressed.
							KeyValue |= KEY_ALT;
					break;

				case BUTTON_SET_XHOME :
						KeyValue =  KEY_PRESSED | KEY_SET_XHOME;
						if (altkeypressed) // Alt key pressed.
							KeyValue |= KEY_ALT;
					break;

				case BUTTON_ALT :
						KeyValue =  KEY_PRESSED | KEY_ALT;
					break;

				default :
					BeepMsg = T_BAD_KEY | FOUR_BEEPS;  //  80 ms chirp.
					KeyValue = 0;		// Trash existing key press.
					break;
				};
			}
			else {
				KeyState = KEY_STARTING;   // Noise on Key lines or key down less than
										// 100 ms.
			}
		};
		break;

	case KEY_BLINK_ON :
		KeyState = KEY_BLINK_OFF;
		break;

	case KEY_BLINK_OFF :
		if (TimerDone(KEY_TIMER)) {
			StartTimer( KEY_TIMER, T_KEY_DOWN );
			KeyState = KEY_OFF;
			// If we start timer here then we can test for defective
			// keys.
			// StartTimer( KEY_TIMER, KEY_TIMEOUT ); Timeout would 
			// be 5 minutes or so.  Then we would mask out that key
			// and no longer test for it till a complete power recycle.

		};
		break;

    case KEY_OFF :	// We wait here for key release.
		if (((kdch = GetKeys(&altkeypressed)) != CurrentKey) || TimerDone(KEY_TIMER)) {
			// If keys have changed.
			if (kdch != CurrentKey) {
			 	//  and if all keys off
				if (kdch == 0x3f) { 		
					KeyValue = KEY_RELEASED;		// tell application
					KeyState = KEY_STARTING;				
				}
				else {
					// So we test for new keys if key released or an
					// extra has been pressed down.

					switch (kdch+CurrentKey) {
					  default :		
			        	KeyState = KEY_STARTING;  
						break;
					}
				}
	        } 
	        else {  // Current key now pressed longer than T_KEY_DOWN ms.
				switch ( KeyValue & KEY_MASK ) {

				default :
					KeyValue |= KEY_HELD_DOWN;
					break;
				}
			}
		}
        break;

    default:
        break;
    }  // switch
} // KeyDevice()
