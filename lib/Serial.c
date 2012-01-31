/* 
    Serial.c -- E-Leadscrew code for an electronic replacement of
    gears used for threading or hobbing on a Lathe or Gear Hobber.

	Gets and puts to the serial port either via interrupt or polled. 
	Also has a library of numeric input functions for parsing the 
	input line for numeric input.
	Note the Input routines are line oriented and nothing happens until
	a <CR> has been entered.

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
// Serial.c -- Serial port I/O and support routines. 
// (c) 1996 Automation Artisans Inc.

/* ----DEFINITIONS ----*/
#include "processor.h"

#include <stddef.h>
#include <ctype.h>
#include <stdio.h>

#include "common.h"
#include "config.h"
#include "serial.h"
#include "EE18.h"

/* ----PRIVATE VARIABLES---- */

// Input Buffer
uint8 SerialInBuffer[SERIAL_INBUFFER_SIZE];
volatile uint8 SerialInIndex;

// Output Buffer
int8 SerialOutBuffer[SERIAL_OUTBUFFER_SIZE];
volatile uint8 SerialOutIndexFront, SerialOutIndexBack;

int8 ch;

// Output Staging Buffer for HexLib output routines.
int8 OutputBuffer[SERIAL_OUTBUFFER_SIZE];		// Parameter staging buffer.

// Communications bit Flags
BITS CommFlags;

/* ----PRIVATE FUNCTIONS---- */
/*
	Transmit Character Interrupt Routine.
*/
/*
 *  FUNCTION: TxCharDevice
 *
 *  PARAMETERS:			None
 *
 *  USES GLOBALS:		SerialOutBuffer
 *						SerialOutIndexFront
 *						SerialOutIndexBack
 *
 *  DESCRIPTION:		Transmit Character Interrupt Routine.
 *
 *  RETURNS: 			Nothing
 *
 */
void 
TxCharDevice(void) {
	if (PIR1bits.TXIF) {
		PIR1bits.TXIF = 0;
		if (SerialOutIndexFront != SerialOutIndexBack ) {
			TXREG = SerialOutBuffer[SerialOutIndexFront];
			SerialOutIndexFront = ++SerialOutIndexFront & SERIAL_OUTBUFFER_MASK;	
		}
		else
			PIE1bits.TXIE = 0;
	}
}

/*
 *  FUNCTION: PutSerial
 *
 *  PARAMETERS:			ch	-- Character to send
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:		Send character out serial port.  
 *
 *  RETURNS: 			Nothing
 *
 */
void 
PutSerial(uint8 ch) {
#ifdef TX_INTERRUPT_ENABLED
  static uint8 ndx;
	// Wait while buffer is full as interrupt changes Front pointer.
	ndx = (SerialOutIndexBack+1) & SERIAL_OUTBUFFER_MASK;
	// Once int8 is removed by interrupt routine while loop exits.
	while (ndx == SerialOutIndexFront)
		;

	// Buffer not full 
	INTCONbits.PEIE = 0;	// don't let interrupt modify what we look at.
	if (PIE1bits.TXIE == 0) { 
			TXREG = ch;
		PIE1bits.TXIE = 1;	// Allow interrupt to handle the rest.
	}
	else {
		SerialOutBuffer[SerialOutIndexBack++] = ch;
		SerialOutIndexBack &= SERIAL_OUTBUFFER_MASK;
	}		
	INTCONbits.PEIE = 1;	// Let interrupt finish.
}
#else
	while (!PIR1bits.TXIF) 
		;
	TXREG = ch;
}
#endif

/*
 *  FUNCTION: RxCharDevice
 *
 *  PARAMETERS:			None
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:		Receive character Interrupt routine.
 *
 *  RETURNS: 			Nothing
 *
 */
void 
RxCharDevice(void) {
    int8 InChar;

    if (PIR1bits.RCIF) { // Character received.
        InChar = RCREG; 			 // Get int8 which clears interrupt flag.
        	   
    	if (RCSTAbits.OERR || RCSTAbits.FERR) {					 // Overrun error	   
    		RCSTAbits.CREN = 0;					 // Disable to clear overrun	   
    		RCSTAbits.CREN = 1;					 // then enable again.
		}								 // So new interrupts come through.

        if (InChar==0x0A) return;       // Ignore linefeeds

    	if (!fNET_FLAG) {
			if (InChar == 0x0d) {
				SerialInBuffer[SerialInIndex] = '\0';
				SerialInIndex = 0;	// Point to start of buffer again.
				fNET_FLAG = 1; // Signal message available.
			}
			else {
				if (fECHO_FLAG && (InChar == 0x08)) {  // Backspace
					if (SerialInIndex > 0) {
						putchar(0x08);
						putchar(' ');
						putchar(0x08);
						SerialInIndex--;
					}
				}
				else {
					SerialInBuffer[SerialInIndex++] = InChar;
	                if (fECHO_FLAG) putchar(InChar);
					// Prevent overflows of buffer.
					if (SerialInIndex == SERIAL_INBUFFER_SIZE) --SerialInIndex;
				}
			}
    	}
    }
}

/*
 *  FUNCTION: _usart_putc
 *
 *  PARAMETERS:			c 	-- Character to send.
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:		Sends character and adds CR char if LF character.
 *
 *  RETURNS: 			Character sent.
 *
 */
int16 
_usart_putc(int8 c) {
	if (c=='\n')
		PutSerial('\r');
	PutSerial(c);
	return c;
}

/* ----PUBLIC FUNCTIONS---- */

/*
    Initialize the Serial Port and all Queues and Buffers.
*/
/*
 *  FUNCTION: InitSerial
 *
 *  PARAMETERS:			baud	-- Serial Port Baud Rate
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:
 *  					Initialize the Serial Port and all Queues and Buffers.
 *
 *  RETURNS: 			Nothing
 *
 */
void 
InitSerial(uint8 baud) {	// Sets baud rate etc.
	SPBRG = baud;
    SerialInIndex = 0;
	SerialOutIndexFront = 0;
	SerialOutIndexBack = 0;
	PIE1bits.TXIE = 0;			// Tx ints disabled.
	PIE1bits.RCIE = 0;			// RX interrupts disabled
	TXSTA = TXINIT;				// Uart control bits.
	RCSTA = RXINIT;
	TXSTAbits.TXEN = 1;			// enable UART transmitter.
	RCSTAbits.CREN = 1;
	PIR1bits.RCIF = 0;
	PIE1bits.RCIE = 1;
	PIR1bits.TXIF = 0;
	PIE1bits.TXIE = 0;
}

/*
 *  FUNCTION: SetEchoFlag
 *
 *  PARAMETERS:			flg	-- Echo Flag state
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:		Method to set/clr echo flag.
 *
 *  RETURNS: 			Nothing
 *
 */
void 
SetEchoFlag(uint8 flg) {
	fECHO_FLAG = flg;
}

/*
 *  FUNCTION: putchar
 *
 *  PARAMETERS:			ch 	-- Character to output/
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:		Wrapper function for direct io
 *
 *  RETURNS: 			Nothing
 *
 */
void 
putchar(int8 ch) {
	_usart_putc (ch);
}

/*
 *  FUNCTION: PutCRLF
 *
 *  PARAMETERS:			None
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:		Wrapper to send CRLF pair.
 *
 *  RETURNS: 			Nothing
 *
 */
void 
PutCRLF(void) {
	putchar('\n');
}

/*
 *  FUNCTION: PutString
 *
 *  PARAMETERS:			ptr	-- ROM String Pointer
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:		Put a rom string pointer to serial port.
 *
 *  RETURNS: 			Nothing
 *
 */
void 
PutString( const rom pint8 ptr ) {
  uint8 ch;
	while (ch = *ptr) {
		putchar(ch);
		ptr++;
	}
}

/*
 *  FUNCTION: PutBuffer
 *
 *  PARAMETERS:			Nothing
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:		Put global OutputBuffer to serial port.
 *
 *  RETURNS: 			None
 *
 */
void 
PutBuffer(void) {
	uint8 i;
	for (i=0;i<SERIAL_OUTBUFFER_SIZE;i++) {
		if (OutputBuffer[i] != 0)
			putchar(OutputBuffer[i]);
		else
			return;
	}
}

/*
 *  FUNCTION: PutInputLine
 *
 *  PARAMETERS:			None
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:		Echo User Serial Port Input Line.
 *
 *  RETURNS: 			Nothing
 *
 */
void 
PutInputLine(void) {
	uint8 i;
	PutCRLF();
	putchar('>');
	for (i = 0; i < SerialInIndex; i++) 
		putchar(SerialInBuffer[i]);
}

/*
 *  FUNCTION: getc
 *
 *  PARAMETERS:			Nothing
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:		Returns 0 or next character.
 *
 *  RETURNS: 			Next character in input buffer
 *
 */
uint8 
getc(void) {
	// If there are characters in the buffer.
	if (fNET_FLAG) {  // Buffer full with a message for us.
		// And the buffer is empty
		if (SerialInBuffer[SerialInIndex] == '\0') { // Return null and prepare for next line.
			fNET_FLAG = 0;  // Allow interrupt routine to process chars again.
			SerialInIndex = 0;
			return( '\0');
		}
		else  // Otherwise return the next character in the input stream.
			return(SerialInBuffer[SerialInIndex++]);
	}
	else
		return( '\0');
} 

/*
 *  FUNCTION: ungetc
 *
 *  PARAMETERS:			None
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:		Puts character back by moving input buffer pointer.
 *
 *  RETURNS: 			Nothing
 *
 */
void 
ungetc(void) {
	SerialInIndex--;
	if (SerialInIndex < 0) SerialInIndex = 0;
}

/*
 *  FUNCTION: GetLine
 *
 *  PARAMETERS:			Nothing
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:		Returns pointer to Serial Input Buffer.
 *
 *  RETURNS: 			Pointer to Input Buffer line.
 *
 */
puint8 
GetLine(void) {
	return( &SerialInBuffer[SerialInIndex] );
}

/*
 *  FUNCTION: GetWord
 *
 *  PARAMETERS:
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:
 * 					Get Address of Data Byte from Serial Port.
 * 					Parse Text to turn into binary number.    
 * 					return 16 bit unsigned 16 bit integer.    
 *
 *  RETURNS: 		Word from input stream.
 *
 */
uint16 
GetWord(void) {
    uint16 result;
	result = 0;
	do {
	    ch = getc(); 

		if ( isdigit(ch) ) {
			result *= 10;
			result += ch - '0';
		}
		else if (ch != 0) {
			ungetc();  // Non digit character.
			return result;
		}
		 
	} while (ch);
	return result;
}

/*
 *  FUNCTION: GetHexWord
 *
 *  PARAMETERS:			Nothing
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:
 * 						Get Address of Data Byte from Serial Port.
 * 						Parse HEX Text to turn into binary number.
 * 						return 16 bit unsigned 16 bit integer.
 *
 *  RETURNS: 			Hex value
 *
 */
uint16 
GetHexWord(void) {
    uint16 result;
	result = 0;
	do {
	    ch = getc();
	    ch = toupper(ch); 

		if ( isdigit(ch) || ((ch >= 'A') && (ch <= 'F')) ) {
			result *= 16;
			ch -= '0';				// Strip ascii part
			if (ch > 9) ch -= 7;	// Hex digit?
			result |= ch;			// Add into result.
			ch = ' '; 				// Good int8 so don't let ch==0 break us out.
		}
		else if (ch != 0) {
			ungetc();  // Non digit character.
			return result;
		}
		 
	} while (ch);
	return result;
}

/*
 *  FUNCTION: GetHexLong
 *
 *  PARAMETERS:			None
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:
 * 						Parse HEX Text to turn into binary number.
 * 						
 *  RETURNS: 			Nothing
 *
 */
void 
GetHexLong(void) {
    uint8 result[4];

	result[0] = 0; // LowWord LowByte
	result[1] = 0; // LowWord HiByte
	result[2] = 0; // HiWord lowByte
	result[3] = 0; // HiWord hiByte
	do {
	    ch = getc();
	    ch = toupper(ch); 

		if ( isdigit(ch) || ((ch >= 'A') && (ch <= 'F')) ) {
			result[3] =  (result[3] << 4) & 0xF0;
			result[3] |= (result[2] >> 4);
			result[2] =  (result[2] << 4) & 0xF0;
			result[2] |= (result[1] >> 4);
			result[1] =  (result[1] << 4) & 0xF0;
			result[1] |= (result[0] >> 4);
			result[0] =  (result[0] << 4) & 0xF0;
			ch -= '0';				// Strip ascii part
			if (ch > 9) ch -= 7;	// Hex digit?
			result[0] |= ch;			// Add into result.
			ch = ' '; 				// Good int8 so don't let ch==0 break us out.
		}
		else if (ch != 0) {
			ungetc();  // Non digit character.
			return;
		}
		 
	} while (ch);
}

/*
 *  FUNCTION: GetHexNibl
 *
 *  PARAMETERS:			Nothing
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:		Get HEX digit from Input buffer
 *
 *  RETURNS: 			BYTE
 *
 */
uint8 
GetHexNibl(void) {
	ch = getc();
	ch = toupper(ch);
	if ( isdigit(ch) || ((ch >= 'A') && (ch <= 'F')) ) {
		ch -= '0';				// Strip ascii part
		if (ch > 9) 
			ch -= 7;			// Hex digit?
	}
	return(ch);
}

/*
 *  FUNCTION: GetHexByte
 *
 *  PARAMETERS:			None
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:		Get HEX byte from input buffer.
 *
 *  RETURNS: 			Byte
 *
 */
uint8 
GetHexByte(void) {
    uint8 result;
	result = GetHexNibl() << 4;
	result |= GetHexNibl();
	return(result);
}

