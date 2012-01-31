/* 
    IoPorts.c -- E-Leadscrew code for an electronic replacement of
    gears used for threading or hobbing on a Lathe or Gear Hobber.
	Initialize I/O and display ports on serial port.

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
// IOPorts.c  -- Processor Specific IO Library

/* ----DEFINITIONS ----*/
#include "processor.h"

#include <stddef.h>
#include <ctype.h>
#include <stdio.h>

#include "common.h"
#include "config.h"
#include "int.h"
#include "serial.h"
#include "EE18.h"
#include "IOPorts.h"

/* ----PRIVATE VARIABLES---- */

/*
 *  FUNCTION: InitIO
 *
 *  PARAMETERS:			None
 *
 *  USES GLOBALS:		
 *
 *  DESCRIPTION:		Sets Ports and TRIS registers
 *
 *  RETURNS: 			Nothing
 *						
 */
void
InitIO(void) {
	INTCON = 0;
	// Set I/O ports.
	PORTA = 0x0FF;		// High, Inputs high.
	TRISA = 0b00001011;	// 
	ADCON1 = AtoD_ADCON1;		// All Digital but RA0
	CMCON = 0x07;		// Comparators off

#ifdef X_AXIS
	PORTB = 0b11110011;	   
	TRISB = 0b00110011;	  // X Axis has step/dir on CAN lines.
#else
	PORTB = 0b11111011;	   
	TRISB = 0b00111011;	  // CAN enabled.
#endif

	PORTC = 0;
	TRISC = 0b10010000;	  //  0..2 out, 3-CLK, 4-MISO, 5-MOSI, 6-TX, 7-RX.
	SSPCON1 = 0;		  // No SPI enabled.

	PORTD = 0xff;         // Used as bidirectional port.
   	TRISD = 0xff;         // All Inputs to start.

	PORTE = 0xFF;
	TRISE = 0x00;		  // All Inputs
	
	PIE1 = 0;			  // Respective device Init() functions set the enables.
	PIE2 = 0;

	INTCON2 = 0x00;	// PORTB Pullup, Falling Edge INT0, Falling Edge INT1, 
					// Low priority Interrupts for INT1,2 TMR0
	INTCON3 = 0x00;	// All low priority, clear and disable INT2, INT1
	IPR1 = 0x04;	// Pulse Clock Interrupt is the only high priority Int.
	IPR2 = 0x00;
	RCON = 0x80;	// Allow priority interrupts.
}

/*
 *  FUNCTION: ShowPort
 *
 *  PARAMETERS:			None
 *
 *  USES GLOBALS:
 *  DESCRIPTION:		Read PORTS where Port # is in input buffer as text.
 *
 *  RETURNS: 			Nothing.  Text out serial port with port value.
 *
 */
void 
ShowPort(void) {
  int8 ch;
  uint8 data = 0;
	ch = GetHexWord();
	switch (ch) {
	  case 0 :	// PORTA
		data = PORTA;
		break;
	  case 1 : 	// PORTB
		data = PORTB;
		break;
	  case 2 : 	// PORTC
		data = PORTC;
		break;
	  case 3 :  // PORTD
		data = PORTD;
		break;
	  case 4 :  // PORTE
		data = PORTE;
		break;
	}
	if (ch < 5) 
		printf((far rom char *)"PORT %d=%02X\n",ch,data);
	else
		PutString("eh?\n");
}

