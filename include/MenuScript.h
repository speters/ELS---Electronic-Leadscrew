/* MenuScript Public Functions and Variables and declarations. */
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


#define MENU_ITEMS						87  // Number of entries in Menu Array.
#define NUMBER_OF_MORSE_TAPERS			8
#define NUMBER_OF_JACOB_TAPERS			9
#define NUMBER_OF_ASSORTED_TAPERS		5
#define NUMBER_OF_BOOL_STRINGS			2

#define SIGNON_MENU_INDEX				0
#define RUN_SETUP_MENU_INDEX			1

extern rom TMENU_DATA MenuData[MENU_ITEMS];

extern const rom TMENU_LIST MorseTaperList[NUMBER_OF_MORSE_TAPERS];
extern const rom TMENU_LIST JacobTaperList[NUMBER_OF_JACOB_TAPERS];
extern const rom TMENU_LIST AssortedTaperList[NUMBER_OF_ASSORTED_TAPERS];
extern const rom TBOOL_STRINGS FlagText[NUMBER_OF_BOOL_STRINGS]; 



