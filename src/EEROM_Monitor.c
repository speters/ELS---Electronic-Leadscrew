/* 
    EEROM_Monitor.c -- E-Leadscrew code for an electronic replacement of
    gears used for threading or hobbing on a Lathe or Gear Hobber.
	EEROM_Monitor allows a user, through the serial port, to modify and
    display EEROM values.

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
#include <float.h>
#include <math.h>

#include "Common.h"		// Common constants.
#include "Config.h"		// Project specific definitions

#include "serial.h"

#ifdef USE_OB_EEROM
#include "OB_EEROM.h"	// EEROM Access definitions
#endif

#ifdef EEROM_MONITOR

#include "EEROM_Monitor.h"

/*
 ** EEROM_Monitor(void)
 *
 *  FILENAME: EEROM_Monitor.c
 *
 *  PARAMETERS: None
 *
 *  DESCRIPTION: Pulls characters from serial port and acts on them to read/write and display Onchip EEROM 
 *
 *  RETURNS: Nothing
 *
 */
void 
EEROM_Monitor(void) {
  int8 ch;
  int16 eerom_cntr;
    if (ch = getc()) {

        switch ( toupper(ch) ) {
        case 'M' : // EMn,b -- Modify memory location.
            EEROMAddress = GetWord();   //
            ch = getc();
            if (ch == ',') { // Correct delimiter
                DecArg = GetHexWord();
                Put_ObEEROM_Byte(EEROMAddress,(BYTE)DecArg);
            }
            while ( getc() ) ;
            break;
        
        case 'P' : // EPn -- Put EEROM location to display
            EEROMAddress = GetWord();   //
            ch = getc();
            DecArg = Get_ObEEROM_Byte(EEROMAdrLow);
            printf((far rom char *)"=%03d",DecArg); // uIntToDec(3);
            PutCRLF();
            while ( getc() ) ;
            break;

		case 'F' : // EFn -- Fill EEROM with n
            DecArg = GetWord();   //
            for ( EEROMAddress = 0; EEROMAddress <= EEROM_SIZE; EEROMAddress++) {
                Put_ObEEROM_Byte(EEROMAddress,(BYTE)DecArg);
			}
			break;

        case 'D' :	// EDx -- Dump EEROM to screen.
            eerom_cntr = GetWord();	// Get Page value.  0 or 1.
			eerom_cntr *= 128;
			DecArg = eerom_cntr;
			eerom_cntr += (EEROM_SIZE-127);
            for ( EEROMAddress = DecArg; EEROMAddress < eerom_cntr; ) {
                DecArg = EEROMAddress;
                printf((far rom char *)"%04d",DecArg); // uIntToDec(4);
                putchar(':');
				putchar(' ');
				for (ch = 0; ch < 10; ch++) {
                    DecArg = Get_ObEEROM_Byte(EEROMAddress);
                    printf((far rom char *)"%02X ",DecArg); // uIntToHex(2);
					EEROMAddress++;
                }
                PutCRLF();
            }
            break;

		case 'R' : // ER -- Report EEROM in format of C nnncfg.h file.
			printf((far rom char *)"#pragma romdata EEDATA\n");
			printf((far rom char *)"const rom unsigned char eememory[] = {\n");

            for ( EEROMAddress = 0; EEROMAddress < EEROM_SIZE; ) {
				putchar(9);		// Tab at front of line.
				for (ch = 0; ch < 4; ch++) {
                    DecArg = Get_ObEEROM_Byte(EEROMAddress);
                    printf((far rom char *)"0x%02X,",DecArg); // uIntToHex(2);
					EEROMAddress++;
                }
				putchar(9);		// Tab at front of comment
				printf((far rom char *)" // %3d(0x%02X)\n",EEROMAddress-4, EEROMAddress-4);
            }
			printf((far rom char *)"};\n#pragma romdata\n");
			break;

		default :
			PutString("eh?\n");
			break;			
		}
	}
}
#endif
