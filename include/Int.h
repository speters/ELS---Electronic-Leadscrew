/* 
    int.h -- E-Leadscrew definitions for an electronic replacement of
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
// Int.h file for global variables and definitions used in Interrupt routine.
#define MICRO_STEPPING			1
#define X_AXIS_INTERRUPT		1
#define TAPERING				1
#define	TRACK_SPINDLE_SPEED		1

// Set Interrupt rate for 10ms.
#define		RTC_DIVISOR	(65535-TXTAL_CPU)+1

#define _RBIF   0	// 0x01
#define _INT0F	1	// 0x02
#define _T0IF	2	// 0x04
#define _RBIE	3	// 0x08
#define _INT0IE	4	// 0x10
#define _T0IE	5	// 0x20
#define _GIEL 	6	// 0x40
#define _GIEH	7	// 0x80


// Table of 74HC595 values.
// First 4 bits are M0..M3 of LMD18245
// Next Bit 5 is Brake Input of LMD18245
// Bit 6 is Direction Input of LMD18245.
// The two step indexes are offset by 8 as they walk through 
// and wrap around this table.
#define MICROSTEPS		8
#define MSTEPTABLESIZE	MICROSTEPS*4
#define MSTEPTABLEMASK	MSTEPTABLESIZE-1

extern int8 PhaseAIndex;
extern int8 PhaseBIndex;
extern int8 MotorOffDelay;

extern rom uint8 MicroStepATableFwd[MSTEPTABLESIZE];
extern rom uint8 MicroStepBTableFwd[MSTEPTABLESIZE];
extern rom uint8 MicroStepATableBkwd[MSTEPTABLESIZE];
extern rom uint8 MicroStepBTableBkwd[MSTEPTABLESIZE];


extern volatile int32 MaxZVel;
extern volatile int32 ZVel;  //current velocity * 2048
extern volatile int32 ZAcc; //acceleration * 2048
extern volatile int32 StepsToZVel; 
extern volatile int32 NewZVel;
extern volatile int32 ZMotorPosition; // absolute Motor Position as a signed # of encoder steps.
extern volatile int32 SystemZBackLashCount;		// Filled from EEROM takes into account half nut backlash.
extern volatile int32 ZStepCount;					// Number of steps to turn Leadscrew.
extern volatile int32 ZHalfwayPoint;					// Number of steps to turn Leadscrew.


extern int16 SpinRate;				// Spindle Speed at start of threading
extern int16 NegSpinRate;			// Negative of Spindle Speed at start of threading
extern int16 SpinClip;				// Half of SpinRate calculated outside interrupt routine for speed.

#ifdef X_AXIS

extern volatile int32 MaxXVel;	// Target Velocity * 2048
extern volatile int32 XVel;  		// Current velocity * 2048
extern volatile int32 XAcc; 		// Acceleration
extern volatile int32 StepsToXVel;	// Number of steps it took to get to maximum speed.
extern volatile int32 NewXVel;	// Maximum RPM when a distance move is requested with fZMoveRQ
extern volatile int32 XMotorRelPosition; // Relative Motor Position as a signed # of encoder steps.
extern volatile int32 XMotorAbsPosition; // Absolute Motor Position as a signed # of encoder steps.
extern volatile int16 XMotorIncrement;		// Used to track steps for absolute rather than relative X motion.
extern volatile int32 XBackLashCount;
#endif


extern volatile uint16 SpindleClockValue;// Accumulates Pulse Clocks per Spindle Revolution

// Run time flags not loaded from EEROM.
extern BITS ActiveFlags;
#define fZAxisActive 					ActiveFlags.Bit.Bit0	
#define fXAxisActive					ActiveFlags.Bit.Bit1
#define fUseLimits						ActiveFlags.Bit.Bit2
#define fThreading						ActiveFlags.Bit.Bit3

#ifdef TAPERING
extern union BRESENHAM {
  int32	Bresenham;
  struct {
	uint16  Low;
	uint16  High;
  };
} TAccumulator;

extern int32 TaperTangent;
#endif

void InitInterruptVariables(void);

