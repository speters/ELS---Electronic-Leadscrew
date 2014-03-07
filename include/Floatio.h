/* 
    Floatio.h -- E-Leadscrew definitions for an electronic replacement of
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

	Ref: MATH Toolkit for REAL-TIME Programming by 
			Jack W. Crenshaw.


    John Dammeyer
    johnd@autoartisans.com


   	Initial Version: 0.00a

   	Version changes: 

*/

#define PI              3.14159265358979
#define RADIANS_PER_DEGREE  (PI/180.0)
#define DEGREES_PER_RADIAN  (180.0/PI)

#define TestNaN(f) ( ((((BYTE *)&f)[3] & 0x7f) == 0x7f ) && (((BYTE *)&f)[2] & 0x80) ) 
#define ULONGFloat(f) (*(unsigned long *)&f)

extern const rom float PlusPowerOf10[11];

void AsciiTofloat(pfloat32 Number, char *s);
void floatToAscii(float32 fNumber, char * s, int8 digits, int8 fractions);
