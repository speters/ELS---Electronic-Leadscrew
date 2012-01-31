// Menu.c
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


// Bring in processor definition and math library
#include "processor.h"

#include <Ctype.h>
#include <stdio.h>
#include <float.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "Common.h"			// Common constants.
#include "Config.h"			// Project specific definitions

#include "menu.h"
#include "menuscript.h"
#include "globvars.h"	

#include "floatio.h"
#include "serial.h"
#include "OB_EEROM.h"		// EEROM Access definitions

#include "LCD.h"

TMENU_DATA CurrentMenu;
signed int8 CurrentMenuNumber, LastMenuNumber;


/* ------------  Public Functions -------------*/

/*
	GetMenu();
	Pull Menu Text from storage and format variable based on
	menu information. 
	Parameters: 
		menu holds menu #
		pMenu points to the data structure that will hold our menu.
		keybuf points the buffer where the entered user data is put.
			keybuf[0] also holds a copy of the flag variable if the menu is a FLAG.
	Returns:
		Length of user input field or zero if no field entry is required.
*/
/*
 *  FUNCTION: GetMenu
 *
 *  PARAMETERS:				
 * 			menu holds menu #
 *			pMenu points to the data structure that will hold our menu.
 *			keybuf points the buffer where the entered user data is put.
 *			keybuf[0] also holds a copy of the flag variable if the menu is a FLAG.
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:
 *			Pull Menu Text from storage and format variable based on
 *			menu information. 
 *
 *  RETURNS: 	
 *			Length of user input field or zero if no field entry is required.
 *
 */
int8 
GetMenu(uint8 menu, PTMENU_DATA pMenu, puint8 keybuf ) {
  uint16 addr;
  int32 data_value;
  uint8 fieldLength = 0;
  uint8 fractions = 0;
  int8 flag_value;
  TMENU_LIST far rom * listptr;
  float32 listmenudata_value;
  rom int8 * sptr;
	DEBUG_MENU("Menu=%d\n", menu);
	// First get default menu information from FLASH into data structure
  	memcpypgm2ram(
		(void *)pMenu, 
		(MEM_MODEL rom void *)(&MenuData[menu]), 
		sizeof(TMENU_DATA)
		);

	// Prepare to display the menu text and parameter information.
	// Format GOTO_XY characters in buffer
	OutputBuffer[0] = 0x1b;
	OutputBuffer[1] = '=';
	OutputBuffer[2] = 0x20;		// Line 0.
	OutputBuffer[3] = 0x20;     // Column 0
	// Put Text into buffer
	sprintf(&OutputBuffer[4], (far rom int8 *)"%s", pMenu->Menubuf);
	DEBUGSTR("%s\n",OutputBuffer);
	// Then display Menu.
	LCDSendBuf((puint8)OutputBuffer);
 
	// Next decide if there's data to display
	// Leaf nodes have a GIndex from 1..254 which indexes the Global variables array to get at the data value.
	if ((pMenu->GIndex != 0) && (pMenu->GIndex != 0xff)){  // Leaf node with data.
		// Set up GOTO XY to point to where data goes.
		OutputBuffer[0] = 0x1b;
		OutputBuffer[1] = '=';
		OutputBuffer[2] = 0x21;		// Line 1.
		OutputBuffer[3] = pMenu->Pos + 0x20;

		// Now go format the data based on what type.
	  	switch(pMenu->Format) {

		  case BYTE_TYPE :	// uint8
			fieldLength = pMenu->Len & 0xf;
			pMenu->Data.LongValue = GlobalVars[pMenu->GIndex].l & 0xFF;
			SuLongToDec(&OutputBuffer[4], pMenu->Data.LongValue, fieldLength, 1);
			strcpy((int8 *)keybuf, (const int8 *)&OutputBuffer[4]);
			break;

		  case INT_TYPE :	// uint16
			fieldLength = pMenu->Len & 0xf;
			pMenu->Data.LongValue = GlobalVars[pMenu->GIndex].l & 0xFFFF;
			SLongToDec(&OutputBuffer[4], pMenu->Data.LongValue, fieldLength, 0);
			strcpy((int8 *)keybuf, (const int8 *)&OutputBuffer[4]);
			break;

		  case WORD_TYPE :	// uint16
			fieldLength = pMenu->Len & 0xf;
			pMenu->Data.LongValue = GlobalVars[pMenu->GIndex].l & 0xFFFF;
			SuLongToDec(&OutputBuffer[4], pMenu->Data.LongValue, fieldLength, 1);
			strcpy((int8 *)keybuf, (const int8 *)&OutputBuffer[4]);
			break;

		  case LONG_TYPE :	// LONG
			fieldLength = pMenu->Len & 0xf;
			pMenu->Data.LongValue = GlobalVars[pMenu->GIndex].l;
			SLongToDec(&OutputBuffer[4], pMenu->Data.LongValue, fieldLength, 0);
			strcpy((int8 *)keybuf, (const int8 *)&OutputBuffer[4]);
			break;

		  case ULONG_TYPE :	// uint32
			fieldLength = pMenu->Len & 0xf;
			pMenu->Data.LongValue = GlobalVars[pMenu->GIndex].l;
			SuLongToDec(&OutputBuffer[4], pMenu->Data.LongValue, fieldLength, 1);
			// Put a copy in our keybutton edit buffer
			strcpy((int8 *)keybuf, (const int8 *)&OutputBuffer[4]);
			break;

		  case FLOAT_TYPE :	// FLOAT
			fieldLength = pMenu->Len & 0xf;
			fractions = (pMenu->Len >> 4) & 0xf;
			pMenu->Data.FloatValue = GlobalVars[pMenu->GIndex].f;
			// Deal with NaN and infinity.
			if (pMenu->Data.LongValue == 0xFFFFFFFF) 
				pMenu->Data.FloatValue = 0.0;


			pMenu->Data.FloatValue *= pMenu->Conversion;
			flag_value = pMenu->ReadValue(pMenu);
			floatToAscii(pMenu->Data.FloatValue, &OutputBuffer[4],fieldLength, fractions);
			strcpy((int8 *)keybuf, (const int8 *)&OutputBuffer[4]);
			OutputBuffer[4+fieldLength] = flag_value;
			OutputBuffer[5+fieldLength] = 0;
			break;

		  case MNEM_TYPE :	// 3 int8 Mnemonic
			break;
		
		  /*
			The List type holds an index into an array of TMENU_LIST records.
			The text field is displayed on the LCD 2nd line from position 0.
			the Data field is printed to the right of that.
			The code below to pull the data from ROM was tried a number of different ways 
			until the generated assembler was the smallest.
		  */	
		  case LIST_TYPE :
			// Tell up/down arrow keys to change GIndex to next/previous value.
			fListFlag = 1;
			// Point to array of labels and data.
			listptr = (TMENU_LIST far rom * )pMenu->Data.LongValue;
			// printf((far rom int8 *)"List=%04X\n", listptr);
			// Then use that to point to our specific entry as index by GIndex.
			listptr = (TMENU_LIST far rom * )&listptr[GlobalVars[pMenu->GIndex].b];
			// printf((far rom int8 *)"ListElement=%04X\n", listptr);
			// Now use this pointer to our ROM'd record to get string from ROM array.
			strcpypgm2ram((int8 *)&OutputBuffer[4], listptr->text);
			// printf((far rom int8 *)"ListCaption=%s\n", (int8 *)&OutputBuffer[4]);
			// Since value is also hard coded in ROM get it from ROM to ram.
			// Set up fields for float32 display conversion.
			fieldLength = pMenu->Len & 0xf;
			fractions = (pMenu->Len >> 4) & 0xf;
			// Display ROM'd value which is already converted to display or entry format.
			floatToAscii(listptr->Data, &OutputBuffer[13],fieldLength, fractions);
			// printf((far rom int8 *)"ListValue=%s\n", (int8 *)&OutputBuffer[13]);
			break;

		}
		DEBUGSTR("%s\n",&OutputBuffer[4]);
		LCDSendBuf((puint8)OutputBuffer);
		return(fieldLength);
	}
	else if (pMenu->GIndex == 0xff) { // Scrolling Flags Menu Type
		fScrollFlags = 1;			// Enable linked list of menus
		fListFlag = 0;				// Cancel List of strings and number pairs.
		// Check if it's a heading MENU or a real flag.
		  /*
			FLAG type menu means that the '.' key toggles the flag while
			the ^,v keys loop through all the flags.  
			The pMenu->Data.LongValue holds the double linked list indexes
			and so isn't used for data.  We access the EEROM flags directly
			instead.
		  */
		if (pMenu->Format==FLAG_TYPE) {
			// Set up to print on/off at correct location.
			OutputBuffer[0] = 0x1b;
			OutputBuffer[1] = '=';
			OutputBuffer[2] = 0x21;		// Line 1.
			OutputBuffer[3] = pMenu->UnitsPos + 0x20;	// Position for display of ON/OFF text

			// Get flag from EEROM and display custom message. 
			*keybuf = DisplayFlagText(pMenu, &OutputBuffer[4], FlagText); // Mirror EEROM value in input buffer.
			// Display Flag value on LCD.
			LCDSendBuf((puint8)OutputBuffer);
			fieldLength = 0; // No printable characters in the field.
			return(0);
		}
	}
	else
		return(0);	// No numeric input required.			
}

/*
 *  FUNCTION: DisplayFlagText
 *
 *  PARAMETERS:			pMenu	-- Pointer to current menu.
 *						pStrDest-- String Buffer pointer
 *						src		-- Text to put into string buffer.
 *  USES GLOBALS:
 *
 *  DESCRIPTION:		Copies one of two strings of text into buffer.
 *
 *  RETURNS: 			Flag value.
 *
 */
uint8 
DisplayFlagText(PTMENU_DATA pMenu, int8 * pStrDest, const rom TBOOL_STRINGS * src) {
	if (GetFlagVar(pMenu)) {
		sprintf(pStrDest, src[pMenu->Dependancy].ptrOn_text);
		return(1);	// Return flag value
	}
	else {
		sprintf(pStrDest, src[pMenu->Dependancy].ptrOff_text );
		return(0);	// Return flag value
	}
}


/*
 *  FUNCTION: DumpMenu
 *
 *  PARAMETERS:		None
 *
 *  USES GLOBALS:	MenuData
 *
 *  DESCRIPTION:	Walks through MenuData Array creating a text dump of
 *					all the parameters.  Useful for diagnostics.
 *
 *  RETURNS: 		Nothing
 *
 */
void
DumpMenu(void) {
  int8 i,j;
  TMENU_DATA d;
	for (i=0; i<MENU_ITEMS; i++) {
		// Get menu.
	  	memcpypgm2ram(
			(void *)&d, 
			(const MEM_MODEL rom void *)&MenuData[i], 
			sizeof(TMENU_DATA)
			);

		// Copy live data but don't copy MENU pointers.
		if (d.Format != MENU_TYPE) { 
			d.Data.LongValue = GlobalVars[d.GIndex].l;
			// Since Unions can only autoinit the first element, we get the floats elsewhere.
			if (d.Format == FLOAT_TYPE) { 
				d.MinValue.FloatValue = GlobalMinimums[d.GIndex];
				d.MaxValue.FloatValue = GlobalMaximums[d.GIndex];
			}
		}
		// Show what's on LCD.

		printf((far rom int8 *)"\nLCD:%02X\n",i);
		printf((far rom int8 *)" --------------------\n|");
		for (j=0;j<20;j++)		 
			putchar(d.Menubuf[j]);

		printf((far rom int8 *)"|\n|");
		for (j=20;j<40;j++)
			putchar(d.Menubuf[j]);
		printf((far rom int8 *)"|\n --------------------\n");
		// Now the type of record.
		switch (d.Format) {
			case  MENU_TYPE :
				printf((far rom int8 *)"MENU: ");
				printf((far rom int8 *)"LCD:%02X, LCD:%02X, LCD:%02X, LCD:%02X\n",
					d.Data.Next[0], d.Data.Next[1], d.Data.Next[2], d.Data.Next[3]); 
			break;
			case  BYTE_TYPE :
				printf((far rom int8 *)"BYTE: %hhu\n", d.Data.Next[0]);
				printf((far rom int8 *)"Min=%hhu, Max=%hhu\n",d.MinValue.Next[0], d.MaxValue.Next[0]);
			break;
			case  INT_TYPE	:
				printf((far rom int8 *)"INT: %d\n", *(uint16 *)&d.Data.Next[0]);
				printf((far rom int8 *)"Min=%d, Max=%d\n",
						*(uint16 *)&d.MinValue.Next[0], *(uint16 *)&d.MaxValue.Next[0]);
			break;
			case  WORD_TYPE	:
				printf((far rom int8 *)"WORD:%u\n", *(uint16 *)&d.Data.Next[0]);
				printf((far rom int8 *)"Min=%u, Max=%u\n",
						*(uint16 *)&d.MinValue.Next[0], *(uint16 *)&d.MaxValue.Next[0]);
			break;
			case  LONG_TYPE	:
				printf((far rom int8 *)"LONG: %ld\n", d.Data.LongValue);
				printf((far rom int8 *)"Min=%ld, Max=%ld\n",
						d.MinValue.LongValue, d.MaxValue.LongValue);
			break;
			case  ULONG_TYPE :
				printf((far rom int8 *)"ULONG:%lu\n", (unsigned int32)d.Data.LongValue);
				printf((far rom int8 *)"Min=%lu, Max=%lu\n",
						(unsigned int32)d.MinValue.LongValue, (unsigned int32)d.MaxValue.LongValue);
			break;
			case  FLOAT_TYPE :

				floatToAscii(d.Data.FloatValue, &OutputBuffer[0],d.Len & 0xF,d.Len>>4);
				printf((far rom int8 *)"FLOAT: %s\n", OutputBuffer);
				floatToAscii(d.MinValue.FloatValue, &OutputBuffer[0],d.Len & 0xF,d.Len>>4);
				printf((far rom int8 *)"Min=%s, ", OutputBuffer);
				floatToAscii(d.MaxValue.FloatValue, &OutputBuffer[0],d.Len & 0xF,d.Len>>4);
				printf((far rom int8 *)"Max=%s\n", OutputBuffer);

			break;
			case  MNEM_TYPE	: 
				printf((far rom int8 *)"MNEMONIC:");
			break;
			case  LIST_TYPE :
				printf((far rom int8 *)"LIST:");
			break;
			case  FLAG_TYPE	:
				printf((far rom int8 *)"FLAG: %d\n", GetFlagVar(&d));
			break;
			default :
				printf((far rom int8 *)"UNKNOWN:");
			break;
		}
		printf((far rom int8 *)"Dependant on: %02X\n",  d.Dependancy );
	}
}


/*
 *  FUNCTION: SGetWord
 *
 *  PARAMETERS:		String Input Buffer
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:	Parse Text to turn into unsigned 16 bit WORD.    
 *
 *  RETURNS: 		16 bit unsigned integer.    
 *
 */
uint16 
SGetWord(int8 *s) {
    uint16 result = 0;
	int8 ch;

	while ((*s == ' ') && *s) // Trash leading blanks.
		s++;
	// Now get number.
	do {
	    ch = *s++; 

		if ( isdigit(ch) ) {
			result *= 10;
			result += ch - '0';
		}
		else if (ch != 0) {
			s--;  // Non digit character.
			return result;
		}
		 
	} while (ch);
	return result;
}

/*
 *  FUNCTION: SGetULong
 *
 *  PARAMETERS:		String Input Buffer
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:	Parse Text to turn into unsigned 32 bit long
 *
 *  RETURNS: 		32 bit unsigned integer.    
 *
 */
uint32 
SGetULong(int8 *s) {
    uint32 result = 0;
	int8 ch;

	while ((*s == ' ') && *s) // Trash leading blanks.
		s++;
	// Now get number.
	do {
	    ch = *s++; 

		if ( isdigit(ch) ) {
			result *= 10;
			result += ch - '0';
		}
		else if (ch != 0) {
			s--;  // Non digit character.
			return result;
		}
		 
	} while (ch);
	return result;
}

/*
 *  FUNCTION: SGetLong
 *
 *  PARAMETERS:		String Input Buffer
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:	Parse Text to turn into signed 32 bit long
 *
 *  RETURNS: 		32 bit signed integer.    
 *
 */
int32 
SGetLong(pint8 s) {
    int32 result = 0;
	int8 ch;
	int8 sign;

	while ((*s == ' ') && *s) // Trash leading blanks.
		s++;
	// Now get sign if there.
	if (*s == '-') {
		sign = 1;	// negative.
		s++;
	}
	else {			// must be a digit or a plus.
		if (*s == '+')
			s++;
		sign = 0;
	}
	// Now get number.
	do {
	    ch = *s++; 

		if ( isdigit(ch) ) {
			result *= 10;
			result += ch - '0';
		}
		else if (ch != 0) {
			s--;  // Non digit character.
			if (sign) 
				return(-result);
			else
				return(result);
		}
		 
	} while (ch);
	if (sign) 
		return( -result );
	else
		return( result );
}

/*
 *  FUNCTION: SEmitSign
 *
 *  PARAMETERS:		pbuf	--	Pointer to buffer.
 *					decArg	--	Integer argument.
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:	Takes abs value of argument
 *					and puts ' ' or '-' to string.
 *
 *  RETURNS: 		Changes argument to positive number.
 *
 */
void 
SEmitSign(pint8 pbuf, pint32 decArg) {
    if (*decArg < 0) {
        *decArg = -*decArg;
        *pbuf = '-';
    }
    else {
        *pbuf = ' ';
    }
}

/*
 *  FUNCTION: SLongToDec
 *
 *  PARAMETERS:		pDecBuf	-- Pointer to output string
 *					decArg	-- Numeric Argument
 *					len		-- Field Length
 *					zeros	-- leading zeros flag.
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:
 *
 *  RETURNS: 
 *
 */
void 
SLongToDec(int8 * pDecBuf, int32 decArg, uint8 len, uint8 zeros) {
	if (decArg < 0) 
		SEmitSign(pDecBuf++, &decArg);
	SuLongToDec(pDecBuf, decArg, len, zeros);	
}

/*
 ** uIntToDec
 *
 *  FILENAME: hexlib.c
 *
 *  PARAMETERS:	len == 0 for no leading zeros,  otherwise non-zero for field size with
 *				leading zeros.
 *				OutputBuffer -- Points to location where ascii result must go.
 *   
 *  DESCRIPTION: Converts DecArg into decimal ascii digits with or without leading 
 *				 zeros within a field size specified by argument len.
 *				 When the number is larger than field width the number is printed
 *				 to full width.  Limit of 7 digits allowed.
 *
 *  RETURNS:	 Null terminated buffer with converted value.
 *
 */
void SuLongToDec(int8 * pDecBuf, int32 decArg, uint8 len, uint8 zeros) {
  uint32 rDivisor;
  uint32 workingValue;
  int8 sz;

	if (decArg == 0) {				// Print the zero
		if (len == 0) 
			*pDecBuf++ = '0';
		else 						// or 'len' zeros.
			for (;len>1;len--) {
				if (zeros) 
					*pDecBuf++ = '0';
				else
					*pDecBuf++ = ' ';
			}
		*pDecBuf++ = '0';	// At least one zero.
		// terminate buffer.
		*pDecBuf = '\0';
		return;
	}

	workingValue = decArg;	// Count how many digits the number might have.
	sz = 0;
	rDivisor = 1;			// If number is between 0 and 10 then 
	while (workingValue > 9) {
		sz++;
		workingValue /= 10;
		rDivisor *= 10;
	}

	if (sz >9) sz = 9;		// Don't allow more than 32 bit integer
	// Figure out how many leading zeros.
	len -= sz+1;
	// Number bigger than field so no leading zeros.
	if ((int8)len<0) len = 0; 
							
    sz = 0;					// Now check size of number and 

	for (;len>0;len--) { 	// Insert leading zeros.
		if (zeros) 
			*pDecBuf++ = '0';
		else
			*pDecBuf++ = ' ';
        sz++;
    }
									// insert digits.
	while (rDivisor != 0) {
		workingValue = decArg / rDivisor;	// Get bcd digit.
		decArg = decArg % rDivisor;	
		workingValue += '0';		// turn into ascii digit.
		if (sz < 10) {				// don't overflow buffer.
		    *pDecBuf++ = workingValue; 	// and put into output buffer.
            sz++;
        }
		rDivisor /= 10;
	}
	*pDecBuf = '\0'; 				// terminate string.
}


