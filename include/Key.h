/* 	Key.h --KeyButton code for an electronic replacement of
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

#define 	T_KEY_DEBOUNCE	5		// 50 ms.
#define		T_KEY_DOWN		50		// 750 ms
#define		T_LED_BLINK		10		// 100ms LED blink.
						//  Total = 550 ms 

#define		T_QUARTER_SECOND	25	// 250 ms
#define		T_HALF_SECOND		50	// 500 ms
#define		T_ONE_SECOND		100	// 1000 ms

// Beep times are 10ms * 4 * the parameter value.
#define 	T_KEY_PRESSED	2		// 4 * 3 =120 ms beep.
#define		T_BAD_KEY		1		// 4 * 1 = 40 ms chirp.
#define 	T_ESC_KEY		2		// 4 * 2 = 80 ms beep.
#define		T_BAD_MENU		25		// 4 * 25 = 1000 ms beep.
#define 	T_MENU_SAVED	3		// 4 * 3 = 120 ms beep
#define 	T_ERROR			31		// 4 * 15 = 600ms beep.

#define		KEY_PRESSED		0x80
#define		KEY_HELD_DOWN 	0x40
#define 	KEY_ALT			0x100	//		0x10	
#define		KEY_RELEASED 	0
#define 	KEY_MASK		0x3f	// Max 64 combinations allowed plus ALT key.

// Key Values.
#define		KEY_0			  0x30	   // 
#define		KEY_1			  0x31	   // 
#define		KEY_2			  0x32	   // 
#define		KEY_3			  0x33	   // 
#define		KEY_4			  0x34	   // 
#define		KEY_5			  0x35	   // 
#define		KEY_6			  0x36	   // 
#define		KEY_7			  0x37	   // 
#define		KEY_8			  0x38	   // 
#define		KEY_9			  0x39	   // 
#define		KEY_PERIOD		  0x2E	   // 
#define		KEY_DEL			  0x1F	   // 
#define		KEY_UP			  0x0A	   // <LF>
#define		KEY_DOWN		  0x0C	   // <FF>
#define		KEY_ESC			  0x1B	   // <ESC>
#define		KEY_ENTER		  0x0D	   // <CR>
#define		KEY_SFN1		  0x11	   // 
#define		KEY_SFN2		  0x12	   // 
#define		KEY_SFN3		  0x13	   // 
#define		KEY_SFN4		  0x14	   // 
#define		KEY_START		  0x0E	   // 
#define		KEY_STOP		  0x0F	   // 
#define		KEY_THREAD		  0x1C	   // 
#define		KEY_TURN		  0x1D	   // 
#define		KEY_SET_BEGIN	  0x28	   // '('
#define		KEY_SET_END		  0x29	   // ')'
#define		KEY_LFAST		  0x3C	   // '<'
#define		KEY_RFAST		  0x3E	   // '>'
#define		KEY_LZJOG		  0x3A	   // ':'
#define		KEY_RZJOG		  0x3B	   // ';'
#define 	KEY_SET_ZHOME	  0x20
#define 	KEY_SET_XHOME	  0x18
#define		KEY_XJOG_IN		  0x16	   // 
#define		KEY_XJOG_OUT	  0x17	   // 
#define 	KEY_SELECT		  0x2A	   // '*'

// SCAN Codes
#define		BUTTON_SFN1							0	   // 
#define		BUTTON_SFN2							1	   // 
#define		BUTTON_SFN3							2	   // 
#define		BUTTON_SFN4							3	   // 
#define		BUTTON_START						4	   // 
#define		BUTTON_STOP							5	   // 
#define 	BUTTON_ALT							6

#define		BUTTON_7							7	   // 
#define		BUTTON_8							8	   // 
#define		BUTTON_9							9	   // 
#define		BUTTON_UP							10	   // 
#define		BUTTON_THREAD						11	   // 
#define		BUTTON_TURN							12	   // 
#define 	BUTTON_SET_ZHOME					13

#define		BUTTON_4							14	   // 
#define		BUTTON_5							15	   // 
#define		BUTTON_6							16	   // 
#define		BUTTON_DOWN							17	   // 
#define		BUTTON_SET_BEGIN					18	   // 
#define		BUTTON_SET_END						19	   // 
#define 	BUTTON_SET_XHOME					20


#define		BUTTON_1							21	   // 
#define		BUTTON_2							22	   // 
#define		BUTTON_3							23	   // 
#define		BUTTON_ESC							24	   // 
#define		BUTTON_LZFAST						25	   // 
#define		BUTTON_RZFAST						26	   // 
#define 	BUTTON_XJOG_IN						27

#define		BUTTON_PERIOD						28	   // 
#define		BUTTON_0							29	   // 
#define		BUTTON_DEL							30	   // 
#define		BUTTON_ENTER						31	   // 
#define		BUTTON_LZJOG						32	   // 
#define		BUTTON_RZJOG						33	   // 
#define		BUTTON_XJOG_OUT						34	   // 


#define MAX_INPUT_BUFFER_SIZE 16
extern uint16 KeyValue;
extern int8 KeyNumberIndex;
extern int8 KeyMaxLength;
extern uint8 KeyNumberBuffer[MAX_INPUT_BUFFER_SIZE];

void InitKeyDevice(void);
uint16 GetKeys(pint8 pAltKey);
void KeyDevice(void);
