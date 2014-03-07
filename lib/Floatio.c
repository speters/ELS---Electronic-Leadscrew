/* 
    Floatio.c -- E-Leadscrew definitions for an electronic replacement of
    gears used for threading or hobbing on a Lathe or Gear Hobber.
	Functions to read from or write to ascii strings for the purpose of
    floating point conversions.

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
// Hex/Decimal conversion to ascii Library routines.

#include "processor.h"

#include <float.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "common.h"
#include "config.h"
#include "serial.h"
#include "floatio.h"

// Public Functions, Varables and Constants.
#define BUFFER_SIZE		20
#define FLOAT_WIDTH		7

// Private Functions
#define ASCII_TO_FLOAT		1


// Private Variables and constants used by Floating Point functions
char Period[] = ".";
char Plus[] = "+";
char Minus[] = "-";
char Blank[] = " ";
char E[] = "E";
char TheOperator;        // Current operator function key: "+","-","*","/"
char TheEqualSign;       // Equal sign, "="
const rom float32 PlusPowerOf10[11] = {1.0, 1.0E1, 1.0E2, 1.0E3, 1.0E4, 1.0E5, 1.0E6, 1.0E7, 1.0E8, 1.0E9, 1.0E10 };
float32 MinusPowerOf10[11] = {1.0, 1.0E-1, 1.0E-2, 1.0E-3, 1.0E-4, 1.0E-5, 1.0E-6, 1.0E-7, 1.0E-8, 1.0E-9, 1.0E10 };


char Operators[] = {'+','-','*','/','.','=',0};          // Math operators 
char NumericChars[] = {'.','E',0};                       // Numeric characters

int8 CharacterCount = 0;									// Character count


/*
 *  FUNCTION: pow10
 *
 *  PARAMETERS:			x	-- value to raise to a power of 10
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:
 * 						pow10 - Computes 10 raised to the integer power of argument, x, 
 * 						range -10 <= x <= 10, using a table lookup.
 *
 *  RETURNS: 			float32 power'd
 *
 */
float32 
pow10(int16 x) {
	float32 TempValue;
 
	if ((x >= 0) && (x <= 10)) {
   		TempValue = PlusPowerOf10[x];
	}
	else {
   		TempValue = MinusPowerOf10[-x];
	}
	return TempValue;
}

/*
 *  FUNCTION: Member
 *
 *  PARAMETERS:			ch 	-- character
 *						Set	-- Set for testing ch against.
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:		Determines if a character, Ch, is an element of a given set.		
 *
 *  RETURNS: 			Returns TRUE or FALSE.
 *
 */
int8 
Member(char Ch, char * Set) {
	int8 i;
	int8 FoundFlag;

	FoundFlag =  FALSE;
	for (i = 0; i< strlen(Set); i++) {
		if (Ch == Set[i]) {
			FoundFlag = TRUE;
			goto ExitLoop;
		}
	}
ExitLoop:
	return FoundFlag;
}
								
/*
 *  FUNCTION: NormalizeFraction
 *
 *  PARAMETERS:		FractionalPart, 
 *					NumberOfDigits
 *					overflow
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:
 *					Normalizes the fractional part of an MPLAB C18  
 * 					floating point number. Used by printf to output floating point numbers.
 * 					Round up rather than truncate. *
 *
 *  RETURNS: 		Normalized fraction.
 *
 */
long 
NormalizeFraction(float32 FractionalPart, int8 NumberOfDigits, puint8 overflow) {
  int32 TheNormalizedPart;
  int32 scale;
  float32 x;
	*overflow = 0;
	scale = pow10(NumberOfDigits);
	x = (scale * FractionalPart);
   	TheNormalizedPart = ((x * 10.0) + 5) / 10;	// Round.
	if (TheNormalizedPart >= scale) {
		*overflow = 1;
		TheNormalizedPart -= scale;
	}
	return( TheNormalizedPart );  
}

/*
 *  FUNCTION: MantissaToAscii
 *
 *  PARAMETERS:		fNumber	-- positive value float to convert.
 *					Sign	-- sign of the number
 *					*s		-- Where to put the ascii digits
 *					dig 	-- how many digits maximum.
 *					frac	-- how many digits after the decimal place.
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:	Converts the mantissa part of an MPLAB C18 floating point 
 * 					number from binary format to an ASCII string.  Used to output 
 * 					floating point numbers.
 *
 *  RETURNS: 
 *
 */
void 
MantissaToAscii(float32 fNumber, int8 Sign, char * s,  int8 dig, int8 frac ) {
	uint32 IntegralPart;              // Holds the floating point integral part
	uint32 NormalizedPart;            // Holds the floating point normalized fractional part  
	float32 FractionalPart;					     // Holds the floating point fractional part

	int8 n;                                   // Number of digits in the mantissa
	char s1[BUFFER_SIZE];                             // Temporary string
	char s2[BUFFER_SIZE];                             // Temporary string
	int8 j;
	uint8 status;
   
	IntegralPart = (uint32) (fNumber);
	FractionalPart = (float32) (fNumber - (uint32) (fNumber)) ;

	// Convert integral part of the floating point number to ascii
	ultoa(IntegralPart, s1);

	// Determine how many digits are in the Mantissa
	n = strlen(s1)+1; // Include sign bit or blank.
	// Normalize the fractional part of the floating point number  
	NormalizedPart = NormalizeFraction(FractionalPart, frac, &status);  // was 11
	if (status) {	// If fractional part overflowed.
		IntegralPart++;
		// Convert integral part of the floating point number to ascii
		ultoa(IntegralPart, s1);
		// Determine how many digits are in the Mantissa
		n = strlen(s1)+1; // Include sign bit or blank.
	}
	
	s[0] = '\0';	// Start with empty string.
	// How big is the integral part now with the decimal point.
	// Find out by subracting fraction size, the integral size c/w sign and the decimal point.
	if (frac > 0) 
		j = dig-frac-n-1;
	else	// No fractions, no decimal point.
		j = dig-frac-n;

	if (j>0) {	// Space left in front of the sign character?
		strncatpgm2ram(s,(MEM_MODEL rom char *)"           ",j);		// pad with blanks.
		s[j] = '\0';										// Terminate string.
	}	
 	// Now, take care of the mantissa's sign concatenating after the leading blanks.
	if (Sign == -1)	{
		strcat(s, Minus);
	}
	else {
		strcat(s,Blank);
	}

	// Concatenate fractional part to integer part including the decimal point
	strcat(s, s1);
	
	if (frac > 0) 
		strcat(s, Period);
	else {
		return;
	}
	// Convert fractional part of the floating point number to ascii (Number 
	// of digits remaining is (7 - n), including the decimal point. 
	ultoa(NormalizedPart, s2);

	// add leading zeros based on # of float digits minus the ones ahead of the decimal point.
	//j = FloatWidth-(n+1)-strlen(s2);
	j = frac-strlen(s2);		// Calculate number of leading zeros based on normalized fraction.
	n = strlen(s)+j;					// Figure out where termination character goes.
	if (j>0)							// Insert leading zeros.
		strncatpgm2ram(s,(MEM_MODEL rom char *)"000000000",j);
	s[n] = '\0';						// terminate string.
	strcat(s, s2);						// Now add in normalized fraction after leading zeros.
}

/*
 *  FUNCTION: Normalize
 *
 *  PARAMETERS:		Pointer to Value to Normalize
 *					Pointer to exponent.
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:
 *					Normalizes the MPLAB C18 floating point number in exponential 
 * 					notation so that only 1 digit appears to the left of the decimal point. 
 *					The exponent is adjusted accordingly.  
 *					Used to output floating point numbers.
 *
 *  RETURNS: 		Nothing
 *
 */
void 
Normalize(pfloat32 Value, pint16 Exponent) {
  float32 TheNormalizedValue = *Value;
  int16 TheNormalizedExponent  = *Exponent;

	if (TheNormalizedValue > 10.0) {
		while(TheNormalizedValue > 10.0) {
			TheNormalizedValue = TheNormalizedValue / 10.0;
			TheNormalizedExponent++;
		}
	}
	else if ((TheNormalizedValue > 0.0)  && (TheNormalizedValue < 1.0)) {
		while((TheNormalizedValue > 0.0)  && (TheNormalizedValue < 1.0)) {
			TheNormalizedValue = TheNormalizedValue * 10.0;
			TheNormalizedExponent--;
		}
	}
	*Value = TheNormalizedValue;
	*Exponent = TheNormalizedExponent;
}
#ifdef ASCII_TO_FLOAT

float32 temp;

/*
 *  FUNCTION: AsciiTofloat
 *
 *  PARAMETERS:			Pointer to Number to convert
 *						Pointer to string where ascii goes.
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:		Converts a floating point number from
 * 						ASCII format to an MPLAB C18 float in binary format. 
 *						Used by the scanf function to read long floating point numbers.  
 *
 *  RETURNS: 			Nothing
 *
 */
void 
AsciiTofloat(pfloat32 pNumber, char *s) {
 
   	char Ch;							// Character 
	int8 Digit;							// Floating point digit (0..9)
	int8 i;								// Loop index
	int8 FractionalStart;				// Starting location of fraction
    int32 l;
	float32 tfloat;
	int8 digitCount;
	int8 minus;

	CharacterCount = 0;
 	temp = 0.0;   
	Ch = ' ';
	
	i = 0;
	while (*s && (*s == ' ')) // get rid of leading blanks.
		s++;
	
	if (*s == '-') {
		s++;
		minus = 1;
	}
	else
		minus = 0;
	
	digitCount = 0;
	// Count the numeric characters in the Buffer before the decimal point (if any)
 	while (s[CharacterCount] != 0) {
		if ((s[CharacterCount] == '.') || (digitCount >= 7)) {
			goto ExitLoop1;
		}
		else {
			CharacterCount++;
			digitCount++;  
		}
	}
 
ExitLoop1:

	// Convert the Integral Part of an  IEEE Floating Point number
	for (i = 0; i < CharacterCount; i++) {
 		// Get a digit from the floating point work buffer 

		Ch = s[i];    
		Digit = Ch - 48;    // Convert to Binary
 
 		
		if (Member(Ch, Operators) == 0) {
			temp = temp + (float) Digit * pow10(CharacterCount-i-1);
		}
	} 
	// Check terminal character for fractional part of floating point number or for operator (if any)
	Ch = s[CharacterCount];
	if (Ch == '.') {

		FractionalStart = CharacterCount+1;
		CharacterCount = 0;
		i = FractionalStart;
	
		// Count the numeric characters in the Buffer after the decimal point (if any)
 		while ((s[i] != 0) && (digitCount<7)) {
			CharacterCount++;
			digitCount++;  
	 		i++;
		}
 ExitLoop2:
		// CharacterCount = CharacterCount - 1;
		// Convert the Fractional Part of an IEEE Floating Point number (if present)
		for (i = 0; i < CharacterCount; i++) {
			// Get a keystroke from the floating point work buffer 

			Ch = s[FractionalStart+i];    
			Digit = Ch - 48;    // Convert to Binary
 
 			//if (!Member(Ch, Operators)) 
			if ((Member(Ch, Operators) == 0)) {
				tfloat =  pow10(-i-1);
				temp = temp + (float) Digit * tfloat;
			}
		} 
	}
	if (minus)
		temp *= -1.0;
	*pNumber = temp;
}
#endif

/*
 *  FUNCTION: floatToAscii
 *
 *  PARAMETERS:			fNumber				-- Number to convert
 *						s					-- String destination
 *						digits, fractions 	-- Number of places in string.
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION: 		Converts an MPLAB C18 float precision floating point number 
 * 						from binary format to an ASCII string.  
 *						Used to output floating point numbers.
 *
 *  RETURNS: 			Ascii formatted string of float 
 *						Modify fNumber if +/- Infinity or NaN
 *
 */
void 
floatToAscii(float32 fNumber, char * s, int8 digits, int8 fractions) {
  uint32 IntegralPart;              // Holds the floating point integral part
  uint32 NormalizedPart;            // Holds the floating point normalized fractional part 
  uint32 Mantissa;                  // Holds the floating point mantissa used for exponential notation 
  int8 SignOfMantissa;              // Holds the sign of the floating point mantissa
  int8 SignOfExponent;              // Holds the sign of the floating point exponent

  int16 Exponent;                   // Holds the floating point exponent 
  float32 fNumberWidth;				// Floating point number of digits in fNumber
  int8 iNumberWidth;                // Integer number of digits in fNumber
  char s1[BUFFER_SIZE];             // Temporary string


	// Test for -Infinity, +Infinity and Not a Number.
	if (ULONGFloat(fNumber) == 0xFF800000) {// minus infinity?
		fNumber = FLT_MIN;
	}
	else if (ULONGFloat(fNumber) == 0x7F800000) { // plus infinitiy?
		fNumber = FLT_MAX;
	}
	else if (TestNaN(ULONGFloat(fNumber))) {  // Not plus infinity, perhaps NaN?
		fNumber = FLT_MAX;
	}

 	// Determine sign of mantissa and exponent
	if (fNumber >= 0.0) {
		SignOfMantissa = 1;
	}
	else {
		SignOfMantissa = -1;		
		fNumber = -fNumber;
	}

 	// Determine width of the number
	if (fNumber != 0.0)
		fNumberWidth = log10(fNumber);
	else
		fNumberWidth = 0.0;

	// Round up exponent size.
	if (fNumberWidth < 0.0) 
		iNumberWidth = (-fNumberWidth + 0.5);
	else
		iNumberWidth = (fNumberWidth+ 0.5);

	// Don't allow invalid numbers.
	if (digits < 1) digits = 1;

	// Now test to see if number without exponent fits in the field width.
	if (iNumberWidth <= digits) { 
		// Convert normal float number with no exponent (need to include the sign)
		MantissaToAscii(fNumber, SignOfMantissa, s, digits, fractions);
	}
	else { // Doesn't fit so use exponential notation.
		// Normalized the float number using exponential notation
		Exponent = 0;	
		Normalize(&fNumber, &Exponent);
		// Convert the mantissa (fNumber) and the exponent to ascii.  Include characters for the 
		// mantissa and exponent signs, and the "E" for the exponent.

		// Convert mantissa to ascii
		MantissaToAscii(fNumber, SignOfMantissa, s, digits, fractions);

		// Concatenate the exponent symbol
		strcat(s,E);

		if (Exponent >= 0){
			SignOfExponent = 1;
		}
		else {
			SignOfExponent = -1;
			Exponent = -Exponent;
 		}
		// Convert exponent to ascii
		ultoa(Exponent, s1);
		// Take care of the exponent's sign
		if (SignOfExponent == -1) {
			strcat(s, Minus);
		}
		else {
			strcat(s,Plus);
		}
		// Concatanate exponent
		strcat(s, s1);
	}
}

