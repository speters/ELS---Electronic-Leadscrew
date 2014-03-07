// LCD.H Include file for LCD display
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

enum LCDStates {
	LCD_RDY,
	LCD_OUT,
	LCD_INIT,
	LCD_INIT1,
	LCD_INIT2,
	LCD_INIT3
};

// PWM default constants for display brightness.
#define		DISP_PERIOD	128		// 50% duty cycle 
#define 	PWM_DISP	0x0C	// Enables PWM for display contrast.


#define DISPLAY_SIZE 	40
#define DISPLAY_COLUMNS 20
#define DISPLAY_ROWS	2

#define MENU_SIZE		DISPLAY_SIZE

#define LCD_INIT_BYTES	6



// Each parameter has a value, a minimum, a maximum and 
// an index, (if bit0..3 greater than 0) into the SystemParameters Array.
#define SIZEOF_MENU_PARAM_DATA 4
#define NMBR_PARAMS 4
// And for our LCD Display 0f two lines by 20 chars our menu is 40 chars.

#define SIZEOF_PARAM_DATA  		NMBR_PARAMS*SIZEOF_MENU_PARAM_DATA
#define SIZEOF_MENU_RECORD   	MENU_SIZE+(SIZEOF_PARAM_DATA)

void InitLCD( void );
void LCDSendChar(uint8 ch);
void LCDSendCmd(uint8 ch);
void LCDSetPosition(uint8 x, uint8 y);
void LCDSendBuf(puint8 pstr);
void LCDSendStr(const rom char *pstr);
void CheckLCDDevice(void);
