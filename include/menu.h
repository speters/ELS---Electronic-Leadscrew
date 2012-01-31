// Menu.h 
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


#ifndef __MENU_DEFINED
#define __MENU_DEFINED	1

#define LCD_LINE_LENGTH	20
#define LCD_LINE_COUNT	2
#define LCD_MENU_SIZE	LCD_LINE_COUNT*LCD_LINE_LENGTH

// Position of " ON" or "OFF" on LCD screen.
#define FLAG_XPOS	17

#define MENU_TYPE	0
#define BYTE_TYPE	1
#define INT_TYPE	2
#define WORD_TYPE	3
#define LONG_TYPE	4
#define ULONG_TYPE	5
#define FLOAT_TYPE	6
#define MNEM_TYPE	7
#define LIST_TYPE 	8	// Indexes an array of text strings displayed on second line.
#define FLAG_TYPE	9

// The Menu RAM can hold either a menu displayed on LCD screen

/*
	LCD Menu Navigation Structure.
	MenuBuf holds all the characters for the menu.
	There are two types of menus:

	0 : A menu that has an information line on Line 1
		and up to 4 selections on line 2.  A numeric key
		or arrow keys select which of the 4 menu 
		selections a user can make.  The value of this selection
		is held in the uint8 array Next.  Up to 4 menu #'s can
		exist in this array although normally it's just 2 for 
		a right and left branch in a binary tree.

	1..n : A menu has information on line 1 and an entry field
		on line 2.  Pos and Len define where on line 2 and how 
		many characters can be input or output, high nibble of Len 
		defines how many digits after the decimal point.
		The Type number is now an index into the global 
		variables array.
		A flag bit7 in the Len variable determines whether the 
		updated value should be written to EEROM to be made 
		permanent or only written to the global variables
		array.

	When a menu is retrieved from EEROM the bit7 of len is 
	checked and if set, means that the information in the menu
	was sticky and needs to be put into the global array.
	Correspondingly,  if the value is changed by a user, 
	the value is always written to the global array.

	On startup, the entire menu structure should be parsed to 
	fill the global array. However, we don't need to walk through
    it tree wise,  just index through the array of menu's and if
	Type is non-zero access the parameter and insert into the 
	Global RAM array.
	
	The kind of parameter is used to format the output.

	LIST_TYPE like FLAG_TYPE uses the arrow keys to walk through the list of strings.
		The GIndex points to the location to store the selected menu data.
		Format tells us that it's LIST_TYPE but the [GIndex].Format tells us really what the data is,
		whether it's dependant etc.
		Pos is where on the second line the Text from the list array goes.
		Dependancy holds which List Item is active.  
			Arriving at a LIST_TYPE menu pulls up the string indexed by Dependancy.
			After that the arrow keys move through our list wrapping around and 
			show each successive string.
			An Enter saves the current index into Dependancy and reads the data from the array
			and saves it in the GIndex location.
		

*/
typedef union _MENU_LINKS {
	int32 LongValue;	  		// List of parameters or Indexes.
	float32 FloatValue;
	uint8 Next[4];			// Left and right Menus
  } TMENU_LINKS;


typedef struct tmenu_data{
  uint8 Menubuf[LCD_MENU_SIZE+1];	  	// Our menu text
  int8 GIndex;				// Menu(-1) or leaf(0..n) index (Global memory array index).
  TMENU_LINKS Data, MinValue, MaxValue;
  uint8 Format;				// Type of Data like MENU_TYPE, FLAG_TYPE, LIST_TYPE
  uint8 Pos; 				// Where on second line
  uint8 Len;					// How many characters in low nibble, how many decimals in high nibble
  uint8 Dependancy;			// The Parameter is dependant on this link's parameter.
							// Useful for metric/imperial conversion.
  uint8 UnitsPos;			// From 1..40 where the Units text is placed.
							// 0 When no units used.
							// use dependancy flag or default to " if no dependancy.
  float32 Conversion;
  int8 (*WriteValue)(struct tmenu_data *p);
  int8 (*ReadValue)(struct tmenu_data *p);
} TMENU_DATA;

typedef TMENU_DATA * PTMENU_DATA;

typedef struct tlist_data {
	int8 text[10];
	float32 Data;
} TMENU_LIST;

typedef TMENU_LIST * PTMENU_LIST;

typedef struct tbool_strings {
	 far rom pint8  ptrOn_text;
	 far rom pint8  ptrOff_text;
} TBOOL_STRINGS;

typedef TBOOL_STRINGS * PTBOOL_STRINGS;

extern TMENU_DATA CurrentMenu;
extern signed int8 CurrentMenuNumber, LastMenuNumber;

int8 GetMenu(uint8 menu, PTMENU_DATA pMenu, PBYTE keybuf);

uint8 DisplayFlagText(PTMENU_DATA pMenu, pint8  pStrDest,  const rom TBOOL_STRINGS *src);
void DumpMenu(void);
uint16 SGetWord( pint8 s );
uint32 SGetULong( pint8 s );
int32 SGetLong(int8 *s);
void SuLongToDec(pint8 pDecBuf, int32 decArg, uint8 len, uint8 zeros);
void SLongToDec(pint8 pDecBuf, int32 decArg, uint8 len, uint8 zeros);
#endif
