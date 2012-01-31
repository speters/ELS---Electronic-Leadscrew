// MenuScripts
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
	03JAN09 jcd
			Updated tapers to match Machinery Handbook.
	
*/			 

// Array of data structures holding menus TMENU_DATA as defined in menu.h
// Conversion functions for data in these menus.
// Dump utility to display menu array with linkages.

// Bring in processor definition and math library
#include "processor.h"

#include <Ctype.h>
#include <stdio.h>
#include <float.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "Common.h"			// Common constants.
#include "config.h"

#include "menu.h"
#include "menuscript.h"
#include "globvars.h"
#include "serial.h"	
#include "floatio.h"
#include "MotorDriver.h"

/* ------------  Private Data -------------*/
TMENU_DATA d;

/* ------------  Private Functions and Declarations -------------*/
int8 DoNothing(  TMENU_DATA * p );
int8 CheckZero(TMENU_DATA * p);

int8 InvertToImperial( TMENU_DATA * p);
int8 InvertToMetric( TMENU_DATA * p);

int8 ConvertToImperial( TMENU_DATA * p);
int8 ConvertToMetric( TMENU_DATA * p);

int8 CalcMetricDepth( TMENU_DATA * p);
int8 CalcImperialDepth( TMENU_DATA * p);

int8 TaperDistanceToDegrees(TMENU_DATA * p);
int8 TaperDegreesToDistance(TMENU_DATA * p);

int8 TaperToImperial( TMENU_DATA * p);
int8 TaperToMetric( TMENU_DATA * p);

int8 SetOnboardStepper(TMENU_DATA * p);
int8 ClearOnboardStepper(TMENU_DATA * p);


/* ------------  Public Data -------------*/

rom TMENU_DATA MenuData[MENU_ITEMS] = {
   //0         1         2         3
   //0123456789012345678901234567890123456789
    { // 0 0x00
	pstrSignOn, //"    E-Leadscrew     Revision 1.xx ",
	0,   // Global Variable Array Index
	0x01010101,	 // Data
	0,0,
	MENU_TYPE,	 // Format
	0,	 // Pos
	0,	 // Len
	0,   // Dependancy
	0,   // Units Position that uses Dependancy for which to display
	1.0,  // Conversion Factor
	DoNothing,
	DoNothing
	},
	{ // 1 0x01
	"   Choose Option    RUN            SETUP",
	0,   // Global Variable Array Index
	0x0201010D,	 // Data
	0,0,
	MENU_TYPE,	 // Format
	0,	 // Pos
	0,	 // Len
	0,   // Dependancy
	0,   // Units Position that uses Dependancy for which to display
	1.0,  // Conversion Factor
	DoNothing,
	DoNothing
	},
    { // 2 0x02
	"      SETUP         SPDL FLGS CRSS LDSCR",
	0,   // Global Variable Array Index
	0x091C290C,	 // Data
	0,0,
	MENU_TYPE,	 // Format
	0,	 // Pos
	0,	 // Len
	0,   // Dependancy
	0,   // Units Position that uses Dependancy for which to display
	1.0,  // Conversion Factor
	DoNothing,
	DoNothing
	},
    { // 3 0x03
	" CROSS SLIDE SCREW  TPI  PITCH    METRIC",
	0,   // Global Variable Array Index
	0x3501200B,	 // Data
	0,0,
	MENU_TYPE,	 // Format
	0,	 // Pos
	0,	 // Len
	0,   // Dependancy
	0,   // Units Position that uses Dependancy for which to display
	1.0,  // Conversion Factor
	DoNothing,
	DoNothing
	},
    { // 4 0x04
	"Motor X Slew Rate                       ",
	SLEW_RATE_X_NDX,   // Global Variable Array Index
	0x0000,	 // Data
	0,PULSE_CLOCK_RATE,
	LONG_TYPE,	 // Format
	2,	 // Pos
	7,	 // Len
	0,   // Dependancy
	0,   // Units Position that uses Dependancy for which to display
	1.0,  // Conversion Factor
	DoNothing,
	DoNothing
	},
    { // 5 0x05
	" LEADSCREW PITCH    TPI  PITCH    METRIC",
	0,   // Global Variable Array Index
	0x08070706,	 // Data
	0,0,
	MENU_TYPE,	 // Format
	0,	 // Pos
	0,	 // Len
	0,   // Dependancy
	0,   // Units Position that uses Dependancy for which to display
	1.0,  // Conversion Factor
	DoNothing,
	DoNothing
	},
    { // 6 0x06 Store as TPI
	" Imperial Leadscrew                 TPI ",
	LEADSCREW_IPITCH_NDX,   // Global Variable Array Index
	16.0,	 // Data
	0.1,0,
	FLOAT_TYPE,	 // Format
	8,	 // Pos
	0x15,	 // Len
	0,   // Dependancy
	0,   // Units Position that uses Dependancy for which to display
	1.0,  // Conversion Factor
	InvertToImperial,		// When storing don't allow 0.0.
	InvertToMetric	
	},
    { // 7 0x07 Store as TPI
	" Imperial Leadscrew Pitch               ",
	LEADSCREW_IPITCH_NDX,   // Global Variable Array Index
	0.0625,	 // Data
	0.001,0,
	FLOAT_TYPE,	 // Format
	6,	 // Pos
	0x37,	 // Len
	0x2E,   // Dependancy on Leadscrew Metric Flag
	0,   // Units Position that uses Dependancy for which to display
	1.0,  // Conversion Factor
	ConvertToImperial,		// Write input value to EEROM as imperial units.
	ConvertToMetric			// Pull input back out as imperial but show as metric.
	},
    { // 8 0x08 Store as pitch in imperial units.
	" Metric Leadscrew   Pitch           mm  ",
	LEADSCREW_IPITCH_NDX,   // Global Variable Array Index
	0.01,	 // Data
	0.001,0,
	FLOAT_TYPE,	 // Format
	6,	 // Pos
	0x27,	 // Len
	0,   // Not dependant since this menu is always metric.
	0,   // Units Position that uses Dependancy for which to display
	25.4,  // Conversion Factor
	DoNothing,			// Write input value to EEROM as imperial units.
	DoNothing			// Pull input back out as imperial but show as metric.
	},
    { // 9 0x09
	"      MOTOR Z       RATE           PARAM",
	0,   // Global Variable Array Index
	0x12010113,	 // Data
	0,0,
	MENU_TYPE,	 // Format
	0,	 // Pos
	0,	 // Len
	0,   // Dependancy
	0,   // Units Position that uses Dependancy for which to display
	1.0,  // Conversion Factor
	DoNothing,
	DoNothing
	},
    { // 10 0x0A
	"   Motor Z Steps              Steps/Rev ",
	MOTOR_STEPS_REV_Z_NDX,   // Global Variable Array Index
	2000.0,	 // Data
	200,4000,
	FLOAT_TYPE,	 // Format
	1,	 // Pos
	0x37,	 // Len
	0,   // Dependancy
	0,   // Units Position that uses Dependancy for which to display
	1.0,  // Conversion Factor
	DoNothing,
	DoNothing
	},
    { // 11 0x0B
	" Imperial X Axis                   TPI  ",
	CROSS_SLIDE_IPITCH_NDX,   // Global Variable Array Index
	20.0,	 // Data
	0.1,0,
	FLOAT_TYPE,	 // Format
	8,	 // Pos
	0x15,	 // Len
	0,  // Dependancy
	0,   // Units Position that uses Dependancy for which to display
	1.0,  // Conversion Factor
	InvertToImperial,		// When storing don't allow 0.0.
	InvertToMetric	
	},
    { // 12 0x0C
	"  Spindle Encoding            Pulses/Rev",
	SPINDLE_PULSE_REV_NDX,   // Global Variable Array Index
	1,	 // Data
	1,1,
	BYTE_TYPE,	 // Format
	6,	 // Pos
	1,	 // Len
	0,   // Dependancy
	0,   // Units Position that uses Dependancy for which to display
	1.0,  // Conversion Factor
	DoNothing,
	DoNothing
	},
    { // 13 0x0D
	" RUN PARAMETERS     Z    X  TAPER  UNITS",
	0, //METRIC_PITCH_NDX,   // Global Variable Array Index
	0x5025260E,	 // Data
	0,0,
	MENU_TYPE,	 // Format
	0,	 // Pos
	0,	 // Len
	0,   // Dependancy
	0,   // Units Position that uses Dependancy for which to display
	1.0,  // Conversion Factor
	DoNothing,
	DoNothing
	},
    { // 14 0x0E
	"Z RUN PARAMETERS    TURN THRD  POS   JOG",
	0,   // Global Variable Array Index
	0x16183410,	 // Data
	0,0,
	MENU_TYPE,	 // Format
	0,	 // Pos
	0,	 // Len
	0,   // Dependancy
	0,   // Units Position that uses Dependancy for which to display
	1.0,  // Conversion Factor
	DoNothing,
	DoNothing
	},
    { // 15 0x0F  Stored as pitch.
	"     THREADING      TPI    PITCH  METRIC",
	0,	 			// Global Variable Array Index
	0x3D3B3B3C,   	// Links.
	0,0,
	MENU_TYPE,	 	// Format
	0,	 // Pos
	0,	 // Len
	0,   // Dependancy
	0,   // Units Position that uses Dependancy for which to display
	1.0,  // Conversion Factor
	DoNothing,
	DoNothing
	},
    { // 16 0x10 Stored as pitch in imperial units.
	"     Turn Pitch                         ",
	TURN_PITCH_NDX,   // Global Variable Array Index
	0,	 // Data
	0,0,
	FLOAT_TYPE,	 // Format
	2,	 // Pos
	0x47,	 // Len
	0x24,   // Dependancy fMetricMode
	0,   // Units Position that uses Dependancy for which to display
	1.0,  // Conversion Factor
	ConvertToImperial,
	ConvertToMetric
	},
    { // 17 0x11
	"Motor Z Acceleration                    ",
	ACCEL_RATE_Z_NDX,   // Global Variable Array Index
	0,	 // Data
	1,PULSE_CLOCK_RATE,
	LONG_TYPE,	 // Format
	2,	 // Pos
	7,	 // Len
	0,   // Dependancy
	0,   // Units Position that uses Dependancy for which to display
	1.0,  // Conversion Factor
	DoNothing,
	DoNothing
	},
    { // 18 0x12
	"MOTOR Z PARAMETERS  ACCL PTCH BKLSH STPS",
	0,   // Global Variable Array Index
	0x0A170511,	 // Data
	0,0,
	MENU_TYPE,	 // Format
	2,	 // Pos
	7,	 // Len
	0,   // Dependancy
	0,   // Units Position that uses Dependancy for which to display
	1.0,  // Conversion Factor
	DoNothing,
	DoNothing
	},
    { // 19 0x13
	"MOTOR Z MOVE RATE   SLEW DELAY      MOVE",
	0,   // Global Variable Array Index
	0x15011B14,	 // Data
	0,0,
	MENU_TYPE,	 // Format
	2,	 // Pos
	7,	 // Len
	0,   // Dependancy
	0,   // Units Position that uses Dependancy for which to display
	1.0,  // Conversion Factor
	DoNothing,
	DoNothing
	},
    { // 20 x14
	"Motor Z Slew Rate                       ",
	SLEW_RATE_Z_NDX,   // Global Variable Array Index
	0x0000,	 // Data
	0,PULSE_CLOCK_RATE,
	LONG_TYPE,	 // Format
	2,	 // Pos
	7,	 // Len
	0,   // Dependancy
	0,   // Units Position that uses Dependancy for which to display
	1.0,  // Conversion Factor
	DoNothing,
	DoNothing
	},
    { // 21 0x15
	"Motor Z Move Rate                       ",
	MOVE_RATE_Z_NDX,   // Global Variable Array Index
	0x0000,	 // Data
	0,PULSE_CLOCK_RATE,
	LONG_TYPE,	 // Format
	2,	 // Pos
	7,	 // Len
	0,   // Dependancy
	0,   // Units Position that uses Dependancy for which to display
	1.0,  // Conversion Factor
	DoNothing,
	DoNothing
	},
    { // 22 0x16
	"Adjst Z Jog Distance                    ",
	MOTOR_ZADJ_LENGTH_NDX,   // Global Variable Array Index
	0,	 // Data
	0,0,
	FLOAT_TYPE,	 // Format
	2,	 // Pos
	0x36,	 // Len
	0x24,   // Dependancy fMetricMode
	0,   // Units Position that uses Dependancy for which to display
	1.0,  // Conversion Factor
	ConvertToImperial,
	ConvertToMetric
	},
    { // 23 0x17
	" Leadscrew Backlash Backlash            ",
	LEADSCREW_BACKLASH_NDX,   // Global Variable Array Index
	0.0,	 // Data
	0.0,1.0,
	FLOAT_TYPE,	 // Format
	10,	 // Pos
	0x36,	 // Len
	0x24,   // Dependancy on System Metric Flag
	0,   // Units Position that uses Dependancy for which to display
	1.0,  // Conversion Factor
	ConvertToImperial,
	ConvertToMetric
	},
    { // 24 0x18
	"Z BEGIN/END POS     BEGIN           END ",
	0,   // Global Variable Array Index
	0x1A010119,	 // Data
	0,0,
	MENU_TYPE,	 // Format
	2,	 // Pos
	0x36,	 // Len
	0,   // Dependancy
	0,   // Units Position that uses Dependancy for which to display
	1.0,  // Conversion Factor
	DoNothing,
	DoNothing
	},
    { // 25 0x19
	"Z Begin Position                        ",
	THREAD_BEGIN_NDX,   // Global Variable Array Index
	0x0000,	 // Data
	0,0,
	FLOAT_TYPE,	 // Format
	2,	 // Pos
	0x37,	 // Len
	0x24,   // Dependancy fMetricMode
	0,   // Units Position that uses Dependancy for which to display
	1.0,  // Conversion Factor
	ConvertToImperial,
	ConvertToMetric
	},
    { // 26 0x1A
	"Z End Position                          ",
	THREAD_END_NDX,   // Global Variable Array Index
	0x0000,	 // Data
	0,0,
	FLOAT_TYPE,	 // Format
	2,	 // Pos
	0x37,	 // Len
	0x24,   // Dependancy fMetricMode
	0,   // Units Position that uses Dependancy for which to display
	1.0,  // Conversion Factor
	ConvertToImperial,
	ConvertToMetric
	},
    { // 27 0x1B
	"Time Till Step PowerDown          x 1.0S",
	MOTOR_POWER_DOWN_NDX,   // Global Variable Array Index
	0x0000,	 // Data
	0,255,
	BYTE_TYPE,	 // Format
	8,	 // Pos
	3,	 // Len
	0,   // Dependancy
	0,   // Units Position that uses Dependancy for which to display
	1.0,  // Conversion Factor
	DoNothing,	
	DoNothing	
	},
    { // 28 0x1C
	"      MOTOR X       RATE           PARAM",
	0,   // Global Variable Array Index
	0x1E010131,	 // Data
	0,0,
	MENU_TYPE,	 // Format
	8,	 // Pos
	3,	 // Len
	0,   // Dependancy
	0,   // Units Position that uses Dependancy for which to display
	1.0,  // Conversion Factor
	DoNothing,	
	DoNothing	
	},
    { // 29 0x1D
	"Motor X Move Rate                       ",
	MOVE_RATE_X_NDX,   // Global Variable Array Index
	12000,	 // Data
	0,PULSE_CLOCK_RATE,
	LONG_TYPE,	 // Format
	1,	 // Pos
	5,	 // Len
	0,   // Dependancy
	0,   // Units Position that uses Dependancy for which to display
	1.0,  // Conversion Factor
	DoNothing,	
	DoNothing	
	},
    { // 30 0x1E
	"MOTOR X PARAMETERS  ACCL PTCH BKLSH STPS",
	0,   // Global Variable Array Index
	0x2143031F,	 // Data
	0,0,
	MENU_TYPE,	 // Format
	2,	 // Pos
	7,	 // Len
	0,   // Dependancy
	0,   // Units Position that uses Dependancy for which to display
	1.0,  // Conversion Factor
	DoNothing,
	DoNothing
	},
    { // 31 0x1F
	"MOTOR X Acceleration                    ",
	ACCEL_RATE_X_NDX,   // Global Variable Array Index
	9000,	 // Data
	1,PULSE_CLOCK_RATE,
	LONG_TYPE,	 // Format
	1,	 // Pos
	5,	 // Len
	0,   // Dependancy
	0,   // Units Position that uses Dependancy for which to display
	1.0,  // Conversion Factor
	DoNothing,	
	DoNothing	
	},
    { // 32 0x20
	" Imperial X Axis    Pitch               ",
	CROSS_SLIDE_IPITCH_NDX,   // Global Variable Array Index
	0.050 ,	 // Data
	0.001,0.5,
	FLOAT_TYPE,	 // Format
	8,	 // Pos
	0x37,	 // Len
	0x31,   // Dependancy on X axis Metric Pitch Flag.
	0,   // Units Position that uses Dependancy for which to display
	1.0,  // Conversion Factor
	ConvertToImperial,		// Write input value to EEROM as imperial units.
	ConvertToMetric			// Pull input back out as imperial but show as metric.
	},
    { // 33 0x21
	"   Motor X Steps              Steps/Rev ",
	MOTOR_STEPS_REV_X_NDX,   // Global Variable Array Index
	1600,	 // Data
	200,4000,
	WORD_TYPE,	 // Format
	2,	 // Pos
	6,	 // Len
	0,   // Dependancy
	0,   // Units Position that uses Dependancy for which to display
	1.0,  // Conversion Factor
	DoNothing,	
	DoNothing	
	},
    { // 34 0x22 Start Position of X motion stored in floating point inches
	"Cross Slide Begin                       ",
	BEGIN_X_NDX,   // Global Variable Array Index
	0.0,	 // Data
	0,0,
	FLOAT_TYPE,	 // Format
	8,	 // Pos
	0x37,	 // Len
	0x24,   // Dependancy
	0,   // Units Position that uses Dependancy for which to display
	1.0,  // Conversion Factor
	ConvertToImperial,	// Convert and Store as inches if global Metric Mode.
	ConvertToMetric		// Restore as Metric if Global Metric Mode	
	},
    { // 35 0x23 Retracted Position of X motion stored in floating point inches
	"Cross Slide Retrctd                     ",
	RETRACTED_X_NDX,   // Global Variable Array Index
	0.125,	 // Data
	0,0,
	FLOAT_TYPE,	 // Format
	8,	 // Pos
	0x37,	 // Len
	0x24,   // Dependancy
	0,   // Units Position that uses Dependancy for which to display
	1.0,  // Conversion Factor
	ConvertToImperial,	// Convert and Store as inches if global Metric Mode.
	ConvertToMetric		// Restore as Metric if Global Metric Mode	
	},
    { // 36 0x24
	"THRD OR TURN UNITS  Metric Mode is      ",
	0xFF,   // Global Variable Array Index
	0xFFFF0D0D,  // Links back to menu above.
	0,0,
	FLAG_TYPE,	 			// '.' Selects Flag value.
	EM_SYS_FLAGS,	 		// Pos --> EEROM Location  fMetricMode Flag.
	fBITPOS_MetricMode,	 	// Len --> Bit Position
	0,   // Dependancy
	FLAG_XPOS,   // Where to put the Boolean FLAG TEXT on LCD line 2.
	1.0,  // Conversion Factor
	DoNothing,	
	DoNothing	
	},
    { // 37 0x25
	" TAPER MENU CHOICE  DIST  ANGL DIR LIST ",
	0, // Global Variable Array Index
	0x482F2E42,	 // Data
	0,0,
	MENU_TYPE,	 // Format
	8,	 // Pos
	0x37,	 // Len
	0x24,   // Dependancy
	0,   // Units Position that uses Dependancy for which to display
	1.0,  // Conversion Factor
	ConvertToImperial,
	ConvertToMetric
	},
    { // 38 0x26
	"X RUN PARAMETERS    POS  LOC         JOG",
	0,   // Global Variable Array Index
	0x28015227,	 // Data
	0,0,
	MENU_TYPE,	 // Format
	8,	 // Pos
	7,	 // Len
	0,   // Dependancy
	0,   // Units Position that uses Dependancy for which to display
	1.0,  // Conversion Factor
	DoNothing,	// Covert disatnce into steps and Write to EEROM
	DoNothing		// Read Steps from EEROM and make into distance.
	},
    { // 39 0x27
	"X BEGIN/END POS     BEGIN      RETRACTED",
	0,   // Global Variable Array Index
	0x23010122,	 // Data
	0,0,
	MENU_TYPE,	 // Format
	2,	 // Pos
	6,	 // Len
	0,   // Dependancy
	0,   // Units Position that uses Dependancy for which to display
	1.0,  // Conversion Factor
	DoNothing,
	DoNothing
	},
    { // 40 0x28
	"Adjst X Jog Distance                    ",
	MOTOR_XADJ_LENGTH_NDX,   // Global Variable Array Index
	0.001,	 // Data
	0,0,
	FLOAT_TYPE,	 // Format
	2,	 // Pos
	0x36,	 // Len
	0x24,   // Dependancy
	0,   // Units Position that uses Dependancy for which to display
	1.0,  // Conversion Factor
	ConvertToImperial,
	ConvertToMetric
	},
	/****************   FLAGS   ****************/
    { // 41 0x29 
	"SET PROGRAM FLAGS   SCRL ^,v  TOGGLE '.'",
	0xFF,   		  // Global Variable Array Index Signal to use scroll feature.
	0xFFFF333E,	 // Double linked list in first two bytes. [0] is next, [1] is previous.
	0,0,
	MENU_TYPE,	 // Format
	2,	 // Pos
	6,	 // Len
	0,   // Dependancy
	0,   // Units Position that uses Dependancy for which to display
	1.0,  // Conversion Factor
	DoNothing,
	DoNothing
	},
    { // 42 0x2A
	"INVERT ESTOP LINE   INVERTED IS         ",
	0xFF,   		  // Global Variable Array Index Signal to use scroll feature.
	0xFFFF442B,	 // Double linked list in first two bytes. [0] is next, [1] is previous.
	0,0,
	FLAG_TYPE,	 // Format
	EM_SYS_FLAGS,	 // Pos --> EEROM Location
	fBITPOS_EStop,	 // Len --> Bit Position
	0,   // Dependancy
	FLAG_XPOS,   // Where to put the Boolean FLAG TEXT.
	1.0,  // Conversion Factor
	DoNothing,
	DoNothing
	},
    { // 43 0x2B
	"INVERT LIMIT SWITCH INVERTED IS         ",
	0xFF,   		  // Global Variable Array Index Signal to use scroll feature.
	0xFFFF2A2C,	 // Double linked list in first two bytes. [0] is next, [1] is previous.
	0,0,
	FLAG_TYPE,	 // Format
	EM_SYS_FLAGS,	 // Pos --> EEROM Location
	fBITPOS_LimitSwitch,	 // Len --> Bit Position
	0,   // Dependancy
	FLAG_XPOS,   // Where to put the Boolean FLAG TEXT.
	1.0,  // Conversion Factor
	DoNothing,
	DoNothing
	},
    { // 44 0x2C
	"USE ONBOARD STEPPER STEP MOTOR IS       ",
	0xFF,   		  // Global Variable Array Index Signal to use scroll feature.
	0xFFFF2B2D,	 // Double linked list in first two bytes. [0] is next, [1] is previous.
	0,0,
	FLAG_TYPE,	 // Format
	EM_SYS_FLAGS,	 // Pos --> EEROM Location
	fBITPOS_LocalMicroStep,	 // Len --> Bit Position
	0,   // Dependancy
	FLAG_XPOS,   // Where to put the Boolean FLAG TEXT.
	1.0,  // Conversion Factor
	SetOnboardStepper,
	ClearOnboardStepper
	},
    { // 45 0x2D
	"Z AXIS DIRECTION     INVERTED IS        ",
	0xFF,   		  // Global Variable Array Index Signal to use scroll feature.
	0xFFFF2C30,	 // Double linked list in first two bytes. [0] is next, [1] is previous.
	0,0,
	FLAG_TYPE,	 // Format
	EM_ZMOTOR_FLAGS,	 // Pos --> EEROM Location
	fBITPOS_MZInvertDirMotor,	 // Len --> Bit Position
	0,   // Dependancy
	FLAG_XPOS,   // Where to put the Boolean FLAG TEXT.
	1.0,  // Conversion Factor
	DoNothing,
	DoNothing
	},
    { // 46 0x2E  Taper Angle Menu
	"  TAPER ANGLE       ANGLE IS            ",
	TAPER_NDX,   		  //
	16.7,		  // 
	0,0,
	FLOAT_TYPE,	 // Format
	10,		 // Pos
	0x36,	 // Angle to the nearest 0.001 degree
	0, 	 // Dependancy.
	0,   // Units Position that uses Dependancy for which to display
	1.0,  // Convert to 1/2 included angle since the taper rate is for the included angle. 
	TaperDegreesToDistance,	   // save degrees back to inches per inch
	TaperDistanceToDegrees	   // Pull out pull out and show as degrees.
	},
    { // 47 0x2F 
	"Taper movement to   headstock is        ",
	0xFF,   		  		// Global Variable Array Index Signal to use scroll feature.
	0xFFFF0000,	 			// Double linked list in first two bytes. [0] is next, [1] is previous.
	0,0,
	FLAG_TYPE,	 			// Format
	EM_ZMOTOR_FLAGS,	 	// Pos --> EEROM Location
	fBITPOS_TaperInwards,	// Len --> Bit Position in EEROM
	1,   					// Bit Description String Index.
	14,   					// Where on LCD to put the Boolean FLAG TEXT.
	1.0,  					// Conversion Factor
	DoNothing,
	DoNothing
	},
    { // 48 0x30
	"X AXIS DIRECTION     INVERTED IS        ",
	0xFF,   		  // Global Variable Array Index Signal to use scroll feature.
	0xFFFF2D3A,	 // Double linked list in first two bytes. [0] is next, [1] is previous.
	0,0,
	FLAG_TYPE,	 // Format
	EM_XMOTOR_FLAGS,	 // Pos --> EEROM Location
	fBITPOS_MXInvertDirMotor,	 // Len --> Bit Position
	0,   // Dependancy
	FLAG_XPOS,   // Where to put the Boolean FLAG TEXT.
	1.0,  // Conversion Factor
	DoNothing,
	DoNothing
	},
    { // 49 0x31 
	"MOTOR X MOVE RATE   SLEW            MOVE",
	0,   // Global Variable Array Index
	0x1D010104,	 // Data
	0,0,
	MENU_TYPE,	 // Format
	2,	 // Pos
	7,	 // Len
	0,   // Dependancy
	0,   // Units Position that uses Dependancy for which to display
	1.0,  // Conversion Factor
	DoNothing,
	DoNothing
	},
    { // 50 0x32
	"   AUTOMATIC X      MOVEMENT IS         ",
	0xFF,   		  // Global Variable Array Index Signal to use scroll feature.
	0xFFFF5641,	 // Double linked list in first two bytes. [0] is next, [1] is previous.
	0,0,
	FLAG_TYPE,	 // Format
	EM_XMOTOR_FLAGS,	 // Pos --> EEROM Location
	fBITPOS_AutoX,	 	// Len --> Bit Position
	0,   // Dependancy
	FLAG_XPOS,   // Where to put the Boolean FLAG TEXT.
	1.0,  // Conversion Factor
	DoNothing,
	DoNothing
	},
    { // 51 0x33
	"   SERIAL PORT      ECHO CHARS IS       ",
	0xFF,   		  // Global Variable Array Index Signal to use scroll feature.
	0xFFFF3929,	 // Double linked list in first two bytes. [0] is next, [1] is previous.
	0,0,
	FLAG_TYPE,	 // Format
	EM_COMM_FLAGS,	 // Pos --> EEROM Location
	0,	 // Len --> Bit Position
	0,   // Dependancy
	FLAG_XPOS,   // Where to put the Boolean FLAG TEXT.
	1.0,  // Conversion Factor
	DoNothing,
	DoNothing
	},
	{ // 52 0x34 -- 
	" THREAD PARAMETERS  PTCH DPTH ANGL PASS ",
	0x00,   		  // Global Variable Array Index Signal to use scroll feature.
	0x3640530F,	 // Double linked list in first two bytes. [0] is next, [1] is previous.
	0,0,
	MENU_TYPE,	 // Format
	0,	 // Pos --> 
	0,	 // Len --> 
	0,   // Dependancy
	0,   // Units Position that uses Dependancy for which to display
	1.0,  // Conversion Factor
	DoNothing,
	DoNothing
	},
	{ // 53 0x35
	" Metric X Axis      Pitch           mm  ",
	CROSS_SLIDE_IPITCH_NDX,   // Global Variable Array Index
	0.05,
	0.001,0,
	FLOAT_TYPE,	 // Format
	6,	 // Pos
	0x27,	 // Len
	0,   // Not dependant since this menu is always metric.
	0,   // Units Position that uses Dependancy for which to display
	25.4,  // Conversion Factor
	DoNothing,			// Write input value to EEROM as imperial units.
	DoNothing			// Pull input back out as imperial but show as metric.
	},
	{ // 54 0x36 -- 
	" THREAD PASS PARAM  FRST EACH LAST SPRNG",
	0x00,   		  // Menu
	0x37474645,	 // Double linked list in first two bytes. [0] is next, [1] is previous.
	0,0,
	MENU_TYPE,	 // Format
	0,	 // Pos --> 
	0,	 // Len --> 
	0,   // Dependancy
	0,   // Units Position that uses Dependancy for which to display
	1.0,  // Conversion Factor
	DoNothing,
	DoNothing
	},
	{ // 55 0x37
	"Spring Pass Count             Passes    ",
	PASS_SPRING_CNT_NDX,  // Global Variable Array Index
	0,	 //Default value
	0,255,
	BYTE_TYPE,	 // Format
	6,	 // Pos --> 
	2,	 // Len --> Bit Position
	0,   // Dependancy
	0,   // Units Position that uses Dependancy for which to display
	1.0,  // Conversion Factor
	DoNothing,
	DoNothing
	},
    { // 56 0x38 
	"  MORSE TAPER LIST                      ",
	MORSE_TAPER_LIST_NDX,	 // Location in ROM list.
	(long)MorseTaperList,	 // Address of ROM List.
	0,0,
	LIST_TYPE,	 // Format is list of records that hold strings and floats
	0,   		 // Pos where the string is put on LCD Display.
	0x57,	 	 // Len holds decimals and field length of float.
	TAPER_NDX,   		// Dependancy record entry points to menu to update with list float value when ENTER is tapped
	NUMBER_OF_MORSE_TAPERS,	// UnitsPos holds max elements.
	12.0,  // Conversion Factor
	DoNothing,	// Call save taper to take our calculated value and 
	DoNothing
	},
    { // 57 0x39
	"X AXIS STEP POL      INVERTED IS        ",
	0xFF,   		  // Global Variable Array Index Signal to use scroll feature.
	0xFFFF3A33,	 // Double linked list in first two bytes. [0] is next, [1] is previous.
	0,0,
	FLAG_TYPE,	 // Format
	EM_XMOTOR_FLAGS,	 // Pos --> EEROM Location
	fBITPOS_MXInvertedStepPulse,	 // Len --> Bit Position
	0,   // Dependancy
	FLAG_XPOS,   // Where to put the Boolean FLAG TEXT.
	1.0,  // Conversion Factor
	DoNothing,
	DoNothing
	},
    { // 58 0x3A
	"Z AXIS STEP POL      INVERTED IS        ",
	0xFF,   		  // Global Variable Array Index Signal to use scroll feature.
	0xFFFF3039,	 // Double linked list in first two bytes. [0] is next, [1] is previous.
	0,0,
	FLAG_TYPE,	 // Format
	EM_ZMOTOR_FLAGS,	 // Pos --> EEROM Location
	fBITPOS_MZInvertedStepPulse,	 // Len --> Bit Position
	0,   // Dependancy
	FLAG_XPOS,   // Where to put the Boolean FLAG TEXT.
	1.0,  // Conversion Factor
	DoNothing,
	DoNothing
	},
    { // 59 0x3B Stored as pitch to same location as TPI which is also stored as PITCH.
	"     Thread Pitch                 inch  ",
	THREAD_SIZE_NDX,   // Global Variable Array Index
	0.1,	 // Data
	0,0,
	FLOAT_TYPE,	 // Format
	4,	 // Pos
	0x37,	 // Len
	0,   // Dependancy	on nothing since we always store as inch.
	0,   // Units Position that uses Dependancy for which to display
	1.0,  // Conversion Factor
	CheckZero,		// Prevent divide by zero for TPI indication on Run Time Menu display.
	CheckZero
	},
    { // 60 0x3C Stored as pitch.
	"     Thread TPI                         ",
	THREAD_SIZE_NDX,   // Global Variable Array Index
	10.0,	 // Data
	0,0,
	FLOAT_TYPE,	 // Format
	2,	 // Pos
	0x16,	 // Len
	0,   // Dependancy,  Not dependant so Invert will keep it in inches.
	0,   // Units Position that uses Dependancy for which to display
	1.0,  // Conversion Factor
	InvertToImperial,		
	InvertToMetric
	},
    { // 61 0x3D Stored as pitch to same location as TPI which is also stored as PITCH.
	"     Thread Pitch                  mm   ",
	THREAD_SIZE_NDX,   // Global Variable Array Index
	0.1,	 // Data
	0,0,
	FLOAT_TYPE,	 // Format
	2,	 // Pos
	0x27, // Len
	0,   // Not dependant on any metric/imperial flag because it's always a metric parameter.
	0,   // Units Position that uses Dependancy for which to display
	25.4,  // Conversion Factor
	CheckZero,
	CheckZero
	},
    { // 62 0x3E
	"USE COMPOUND FOR       THREADING IS     ",
	0xFF,   		  // Global Variable Array Index Signal to use scroll feature.
	0xFFFF2956,	 // Double linked list in first two bytes. [0] is next, [1] is previous.
	0,0,
	FLAG_TYPE,	 // Format
	EM_XMOTOR_FLAGS,	 // Pos --> EEROM Location
	fBITPOS_UseCompound,	 // Len --> Bit Position
	0,   // Dependancy
	FLAG_XPOS,   // Where to put the Boolean FLAG TEXT.
	1.0,  // Conversion Factor
	DoNothing,
	DoNothing
	},
    { // 63 0x3F
	" Depth of Thread    Depth               ",
	DEPTH_X_NDX,   		  // Global Variable Array Index Signal to use scroll feature.
	75.0,	 	// Double linked list in first two bytes. [0] is next, [1] is previous.
	0,0,
	FLOAT_TYPE,	// Format
	10,	 // Pos --> 
	0x37,	 // Decimals, Field Size
	0x24,   // Dependancy on System Metric Flag.
	0,   // Units Position that uses Dependancy for which to display
	1.0,  		// Conversion Factor
	ConvertToImperial,
	ConvertToMetric
	},
	{ // 64 0x40
	"   Thread Angle               Degrees   ",
	THREAD_ANGLE_NDX,  // Global Variable Array Index
	30.0,	 //Default value
	0,0,
	FLOAT_TYPE,	 // Format
	2,	 // Pos --> 
	0x15,	 // Len --> Bit Position
	0,   // Dependancy
	0,   // Units Position that uses Dependancy for which to display
	1.0,  // Conversion Factor
	DoNothing,
	DoNothing
	},
    { // 65 0x41
	" TAPER TURNING         TAPERING  IS     ",
	0xFF,   		  // Global Variable Array Index Signal to use scroll feature.
	0xFFFF3244,	 // Double linked list in first two bytes. [0] is next, [1] is previous.
	0,0,
	FLAG_TYPE,	 // Format
	EM_XMOTOR_FLAGS,	 // Pos --> EEROM Location
	fBITPOS_Tapering,	 // Len --> Bit Position
	0,   // Dependancy
	FLAG_XPOS,   // Where to put the Boolean FLAG TEXT.
	1.0,  // Conversion Factor
	DoNothing,
	DoNothing
	},
	{ // 66 0x42
	"   Tapering Ratio             \" per Foot",
	TAPER_NDX,  // Global Variable Array Index
	0.0, //Default value
	0,0,
	FLOAT_TYPE,	 // Format
	0,	 // Pos --> 
	0x59,	 // Len --> Bit Position
	0,   // Dependancy
	0,   // Units Position that uses Dependancy for which to display
	12.0,  // Conversion Factor	// Store as inches/inch or (tan (1/2 included angle))
	DoNothing,
	DoNothing
	},
	{ // 67 0x43
	"   X Axis Backlash  Backlash            ",
	X_AXIS_BACKLASH_NDX,  // Global Variable Array Index
	0.0, //Default value
	0.0,1.0,
	FLOAT_TYPE,	 // Format
	10,	 // Pos --> 
	0x36,	 // Decimals, Field Size
	0x24,   // Dependancy on System Metric Flag.
	0,   // Units Position that uses Dependancy for which to display
	1.0,  // Conversion Factor
	ConvertToImperial,
	ConvertToMetric
	},
    { // 68 0x44
	"   ELECTRONIC       HALFNUT IS          ",
	0xFF,   		  // Global Variable Array Index Signal to use scroll feature.
	0xFFFF412A,	 // Double linked list in first two bytes. [0] is next, [1] is previous.
	0,0,
	FLAG_TYPE,	 // Format
	EM_ZMOTOR_FLAGS,	 // Pos --> EEROM Location
	fBITPOS_HalfNut,	 // Len --> Bit Position
	0,   // Dependancy
	FLAG_XPOS,   // Where to put the Boolean FLAG TEXT.
	1.0,  // Conversion Factor
	DoNothing,
	DoNothing
	},
    { // 69 0x45
	" First Pass Depth    Depth              ",
	PASS_FIRST_X_DEPTH_NDX,   		  // Global Variable Array Index Signal to use scroll feature.
	75.0,	 	//
	0,0,
	FLOAT_TYPE,	// Format
	10,	 // Pos --> 
	0x37,	 // Decimals, Field Size
	0x24,   // Dependancy on X axis Metric Flag.
	0,   // Units Position that uses Dependancy for which to display
	1.0,  		// Conversion Factor
	ConvertToImperial,
	ConvertToMetric
	},
	{ // 70 0x46
	" Each Pass Depth     Depth              ",
	PASS_EACH_X_DEPTH_NDX,  // Global Variable Array Index
	0,	 //Default value
	0,0,
	FLOAT_TYPE,	 // Format
	10,	 // Pos --> 
	0x37,	 // Decimals, Field Size
	0x24,   // Dependancy on X axis Metric Flag.
	0,   // Units Position that uses Dependancy for which to display
	1.0,  // Conversion Factor
	ConvertToImperial,
	ConvertToMetric
	},
    { // 71 0x47
	" Last Path Depth     Depth              ",
	PASS_END_X_DEPTH_NDX,   		  // Global Variable Array Index Signal to use scroll feature.
	75.0,	 	// Double linked list in first two bytes. [0] is next, [1] is previous.
	0,0,
	FLOAT_TYPE,	// Format
	10,	 // Pos --> 
	0x37,	 // Decimals, Field Size
	0x24,   // Dependancy on X axis Metric Flag.
	0,   // Units Position that uses Dependancy for which to display
	1.0,  		// Conversion Factor
	ConvertToImperial,
	ConvertToMetric
	},
	{ // 72 0x48 -- 
	"PREDEFINED TAPERS   MRSE JACB ASRT CSTM ",
	0x00,   		  // Global Variable Array Index Signal to use scroll feature.
	0x4B4A4938,	 // Double linked list in first two bytes. [0] is next, [1] is previous.
	0,0,
	MENU_TYPE,	 // Format
	0,	 // Pos --> 
	0,	 // Len --> 
	0,   // Dependancy
	0,   // Units Position that uses Dependancy for which to display
	1.0,  // Conversion Factor
	DoNothing,
	DoNothing
	},
    { // 73 0x49 
	" JACOB  TAPER LIST                      ",
	JACOBS_TAPER_LIST_NDX,	 // Location in ROM list.
	(long)JacobTaperList,	 // Address of ROM List.
	0,0,
	LIST_TYPE,	 // Format is list of records that hold strings and floats
	0,   		 // Pos where the string is put on LCD Display.
	0x57,	 	 // Len holds decimals and field length of float.
	TAPER_NDX,   		// Dependancy record entry points to menu to update with list float value when ENTER is tapped
	NUMBER_OF_JACOB_TAPERS,	// UnitsPos holds max elements.
	12.0,  // Conversion Factor
	DoNothing,	// Call save taper to take our calculated value and 
	DoNothing
	},
    { // 74 0x4A 
	"ASSORTED TAPER LIST                     ",
	ASSORTED_TAPER_LIST_NDX,	 // Location in ROM list.
	(long)AssortedTaperList,	 // Address of ROM List.
	0,0,
	LIST_TYPE,	 // Format is list of records that hold strings and floats
	0,   		 // Pos where the string is put on LCD Display.
	0x57,	 	 // Len holds decimals and field length of float.
	TAPER_NDX,   		// Dependancy record entry points to menu to update with list float value when ENTER is tapped
	NUMBER_OF_ASSORTED_TAPERS,	// UnitsPos holds max elements.
	12.0,  // Conversion Factor
	DoNothing,	// Call save taper to take our calculated value and 
	DoNothing
	},
	{ // 75 0x4B -- 
	"CUSTOMIZED TAPERS   USR1 USR2 USR3 USR4 ",
	0x00,   		  // Global Variable Array Index Signal to use scroll feature.
	0x4F4E4D4C,	 // Double linked list in first two bytes. [0] is next, [1] is previous.
	0,0,
	MENU_TYPE,	 // Format
	0,	 // Pos --> 
	0,	 // Len --> 
	0,   // Dependancy
	0,   // Units Position that uses Dependancy for which to display
	1.0,  // Conversion Factor
	DoNothing,
	DoNothing
	},
    { // 76 0x4C
	"Custom Taper        User T1             ",
	CUSTOM_TAPER1_NDX,   		  // Global Variable Array Index Signal to use scroll feature.
	0.0,	 	// Double linked list in first two bytes. [0] is next, [1] is previous.
	0,0,
	FLOAT_TYPE,	// Format
	10,	 	// Pos --> 
	0x37,	 // Decimals, Field Size
	0x24,   // Dependancy on X axis Metric Flag.
	0,   // Units Position that uses Dependancy for which to display
	1.0,  		// Conversion Factor
	TaperToImperial,
	TaperToMetric
	},
    { // 77 0x4D
	"Custom Taper        User T2             ",
	CUSTOM_TAPER2_NDX,   		  // Global Variable Array Index Signal to use scroll feature.
	0.0,	 	// Double linked list in first two bytes. [0] is next, [1] is previous.
	0,0,
	FLOAT_TYPE,	// Format
	10,	 	// Pos --> 
	0x37,	 // Decimals, Field Size
	0x24,   // Dependancy on X axis Metric Flag.
	0,   // Units Position that uses Dependancy for which to display
	1.0,  		// Conversion Factor
	TaperToImperial,
	TaperToMetric
	},
    { // 78 0x4E
	"Custom Taper        User T3             ",
	CUSTOM_TAPER3_NDX,   		  // Global Variable Array Index Signal to use scroll feature.
	0.0,	 	// Double linked list in first two bytes. [0] is next, [1] is previous.
	0,0,
	FLOAT_TYPE,	// Format
	10,	 	// Pos --> 
	0x37,	 // Decimals, Field Size
	0x24,   // Dependancy on X axis Metric Flag.
	0,   // Units Position that uses Dependancy for which to display
	1.0,  		// Conversion Factor
	TaperToImperial,
	TaperToMetric
	},
    { // 79 0x4F
	"Custom Taper        User T4             ",
	CUSTOM_TAPER4_NDX,   		  // Global Variable Array Index Signal to use scroll feature.
	0.0,	 	// Double linked list in first two bytes. [0] is next, [1] is previous.
	0,0,
	FLOAT_TYPE,	// Format
	10,	 	// Pos --> 
	0x37,	 // Decimals, Field Size
	0x24,   // Dependancy on X axis Metric Flag.
	0,   // Units Position that uses Dependancy for which to display
	1.0,  		// Conversion Factor
	TaperToImperial,
	TaperToMetric
	},
    { // 80 0x50
	"UNIT PARAMETERS     METRIC           SFM",
	0, //METRIC_PITCH_NDX,   // Global Variable Array Index
	0x51FFFF24,	 // Data
	0,0,
	MENU_TYPE,	 // Format
	0,	 // Pos
	0,	 // Len
	0,   // Dependancy
	0,   // Units Position that uses Dependancy for which to display
	1.0,  // Conversion Factor
	DoNothing,
	DoNothing
	},
    { // 81 0x51
	"SPINDLE DISPLAY     SFM Mode is         ",
	0xFF,   // Global Variable Array Index
	0xFFFF0D0D,  // Links back to menu above.
	0,0,
	FLAG_TYPE,	 			// '.' Selects Flag value.
	EM_SYS_FLAGS,	 		// Pos --> EEROM Location  fMetricMode Flag.
	fBITPOS_SFMMode,	 	// Len --> Bit Position
	0,   // Dependancy
	FLAG_XPOS,   // Where to put the Boolean FLAG TEXT on LCD line 2.
	1.0,  // Conversion Factor
	DoNothing,	
	DoNothing	
	},
    { // 82 0x52
	"Tool Tip Position    Diameter           ",
	X_DIAMETER_NDX,   		  // Diameter of work if tool tip had just cut a pass.
	0.0,	 	// Double linked list in first two bytes. [0] is next, [1] is previous.
	0,0,
	FLOAT_TYPE,	// Format
	10,	 	// Pos --> 
	0x37,	 // Decimals, Field Size
	0x24,   // Dependancy on X axis Metric Flag.
	0,   // Units Position that uses Dependancy for which to display
	1.0,  		// Conversion Factor
	ConvertToImperial,
	ConvertToMetric
	},
    { // 83 0x53
	"   DEPTH MENU       DEPTH MULT      CALC",
	0,   // Global Variable Array Index
	0x5555543F,	 // Data
	0,0,							   
	MENU_TYPE,	 // Format
	2,	 // Pos
	7,	 // Len
	0,   // Dependancy
	0,   // Units Position that uses Dependancy for which to display
	1.0,  // Conversion Factor
	DoNothing,
	DoNothing
	},
    { // 84 0x54
	"Depth Ptch Multipler                    ",
	DEPTH_MULTIPLIER_NDX,   		  // Multiply Pitch by this value to get depth of thread.
	0.0,	 	// Double linked list in first two bytes. [0] is next, [1] is previous.
	0,0,
	FLOAT_TYPE,	// Format
	10,	 	// Pos --> 
	0x57,	 // Decimals, Field Size
	0,   // Dependancy on X axis Metric Flag.
	0,   // Units Position that uses Dependancy for which to display
	1.0,  		// Conversion Factor
	DoNothing,
	DoNothing
	},
    { // 85 0x55
	"  Calculated Depth                      ",
	DEPTH_X_NDX,   		  // Diameter of work if tool tip had just cut a pass.
	0.0,	 	// Double linked list in first two bytes. [0] is next, [1] is previous.
	0,0,
	FLOAT_TYPE,	// Format
	10,	 	// Pos --> 
	0x37,	 // Decimals, Field Size
	0x24,   // Dependancy on X axis Metric Flag.
	0,   // Units Position that uses Dependancy for which to display
	1.0,  		// Conversion Factor
	CalcImperialDepth,
	CalcMetricDepth
	},
    { // 86 0x56
	"BROACH MODE MOVEMENT IS                 ",
	0xFF,   		  // Global Variable Array Index Signal to use scroll feature.
	0xFFFF3E32,	 // Double linked list in first two bytes. [0] is next, [1] is previous.
	0,0,
	FLAG_TYPE,	 // Format
	EM_SYS_FLAGS,	 // Pos --> EEROM Location
	fBITPOS_BroachMode,	 // Len --> Bit Position
	0,   // Dependancy
	FLAG_XPOS,   // Where to put the Boolean FLAG TEXT.
	1.0,  // Conversion Factor
	DoNothing,
	DoNothing
	},
};

const rom TMENU_LIST MorseTaperList[NUMBER_OF_MORSE_TAPERS] = {
	{				// Saved as Inch per Foot included (i.e. Change in diameter per foot)
	"MORSE #0 ",
	0.6246
	},
	{
	"MORSE #1 ",
	0.59858
	},
	{
	"MORSE #2 ",
	0.59941
	},
	{
	"MORSE #3 ",
	0.60235
	},
	{
	"MORSE #4 ",
	0.62326
	},
	{
	"MORSE #5 ",
	0.63151
	},
	{
	"MORSE #6 ",
	0.62565
	},
	{
	"MORSE #7 ",
	0.62400
	}
};


const rom TMENU_LIST JacobTaperList[NUMBER_OF_JACOB_TAPERS] = {
	{				// All tapers saved as inches per foot which is tan(1/2 included angle) * 12.0
	"JACOB #0 ",
	0.59145
	},
	{
	"JACOB #1 ",
	0.92508
	},
	{
	"JACOB #2 ",
	0.97861
	},
	{
	"JACOB #2A",
	0.97861
	},
	{
	"JACOB #3 ",
	0.63898
	},
	{
	"JACOB #4 ",
	0.62886
	},
	{
	"JACOB #5 ",
	0.62010
	},
	{
	"JACOB #6 ",
	0.62292
	},
	{
	"JACOB #33",
	0.76194
	}
};

const rom TMENU_LIST AssortedTaperList[NUMBER_OF_ASSORTED_TAPERS] = {
	{				// Saved as inches per foot == (tan(1/2 included angle)) * 2 * 12
	"R8 Taper ",
	0.296226056*12.0 	// 0.302870*12.0			// 16 degrees 51 minutes.
	},
	{
	"ER Taper ",
	0.281081669*12.0	// 0.286745*12.0			// 16 degrees
	},
	{
	"5C Taper ",
	0.352653961*12.0	// 0.363970*12.0			// 20 degrees  http://www.zagar.com/images/cCollets-ID-OD-Pg2.jpg
	},
	{
	"3C Taper ",
	0.425113123*12.0	// 0.445229*12.0			// 24 degrees  http://www.zagar.com/images/cCollets-ID-OD-Pg2.jpg
	},
	{
	"IT Taper ",
	3.5						// IT is  7/24 which is 3.5"/foot.
	}
};

const rom TBOOL_STRINGS FlagText[NUMBER_OF_BOOL_STRINGS] = { 
	{
	" ON",			// FLAG is SET 1
	"OFF"			// FLAG is CLR 0
	},
	{
	" IN <",		// Flag is SET 1
	"OUT >"			// Flag is CLR 0
	} 
};

/* ------------  Private Functions -------------*/
/* DoNothing:
	If the menu has a value that doesn't need converting to different units 
	then don't do anything but return a delimeter which may or may not be used.
*/
/*
 *  FUNCTION: DoNothing
 *
 *  PARAMETERS:	p	-- pointer to menu.
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:
 *				If the menu has a value that doesn't need converting to different units 
 *				then don't do anything but return a delimeter which may or may not be used.
 *
 *  RETURNS: 	blank delimeter character.
 *
 */
int8 
DoNothing(  TMENU_DATA * p ) {
	return(' ');	// Always a blank units delimeter even if it's not used.
} 

/*
 *  FUNCTION: CheckZero
 *
 *  PARAMETERS:		p 	-- Pointer to Menu
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:	Tests for zero argument and returns 1 to avoid divide by zero.
 *
 *  RETURNS: 
 *
 */
int8 
CheckZero(TMENU_DATA * p) {
	if (p->Data.FloatValue == 0.0)	// If value is zero sub in 1.0
		p->Data.FloatValue = 1.0;
	return(' ');	// Always a blank units delimeter even if it's not used.
}

/*
 *  FUNCTION: ConvertToImperial
 *
 *  PARAMETERS:		p 	-- Pointer to Menu
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:	Tests menu parameter to see if it needs to be converted
 *					from metric to imperial and returns m ' or space character.
 *
 *  RETURNS: 		' ', 'm', '"'
 *
 */
int8
ConvertToImperial( TMENU_DATA * p) {
	// Conversions are often dependant on whether another flag is set.
	if (p->Dependancy != (uint8)0) {	//
		DEBUGSTR("To Imperial Var is dependant on %02X\n",p->Dependancy );
		// First get menu from FLASH into data structure
	  	memcpypgm2ram(
			(void *)&d, 
			(MEM_MODEL rom void *)&MenuData[p->Dependancy], 
			sizeof(TMENU_DATA)
			);
		// Then test the flag.
		if (GetFlagVar(&d)) {	// If the metric flag is set convert the value to imperial.
			DEBUGSTR("Convert to Imperial\n");
			p->Data.FloatValue /= 25.4;	
			return('"');
		}
		else
			return('m');	
	}
	else
		return(' ');
}


/*
 *  FUNCTION: ConvertToMetric
 *
 *  PARAMETERS:		p 	-- Pointer to Menu
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:	Tests menu parameter to see if it needs to be converted
 *					from imperial to metric and returns m ' or space character.
 *
 *  RETURNS: 		' ', 'm', '"'
 *
 */
int8
ConvertToMetric( TMENU_DATA * p) {
	// Conversions are often dependant on whether another flag is set.
	if (p->Dependancy != (uint8)0) {	//
		DEBUGSTR("To Metric Var is dependant on %02X\n",p->Dependancy );
		// First get menu from FLASH into data structure
	  	memcpypgm2ram(
			(void *)&d, 
			(MEM_MODEL rom void *)&MenuData[p->Dependancy], 
			sizeof(TMENU_DATA)
			);
		// Then test the flag.
		if (GetFlagVar(&d)) {	// If the metric flag is set convert the value to metric.
			DEBUGSTR("Convert to Metric\n");
			p->Data.FloatValue *= 25.4;	
			return('m');
		}
		else
			return('"');	
	}
	else
		return(' ');
}

int8
CalcImperialDepth( TMENU_DATA * p) {
	// Conversions are often dependant on whether another flag is set.
	if (p->Dependancy != (uint8)0) {	//
		// First get menu from FLASH into data structure
	  	memcpypgm2ram(
			(void *)&d, 
			(MEM_MODEL rom void *)&MenuData[p->Dependancy], 
			sizeof(TMENU_DATA)
			);

		p->Data.FloatValue = (GlobalVars[DEPTH_MULTIPLIER_NDX].f * GlobalVars[THREAD_SIZE_NDX].f);

		// Then test the flag.
		if (GetFlagVar(&d)) {	// If the metric flag is set convert the value to imperial.
			return('"');
		}
		else
			return('m');	
	}
	else
		return(' ');
}		  

int8
CalcMetricDepth( TMENU_DATA * p) {
	// Conversions are often dependant on whether another flag is set.
	if (p->Dependancy != (uint8)0) {	//
		// First get menu from FLASH into data structure
	  	memcpypgm2ram(
			(void *)&d, 
			(MEM_MODEL rom void *)&MenuData[p->Dependancy], 
			sizeof(TMENU_DATA)
			);
		p->Data.FloatValue = (GlobalVars[DEPTH_MULTIPLIER_NDX].f * GlobalVars[THREAD_SIZE_NDX].f);
		// Then test the flag.
		if (GetFlagVar(&d)) {	// If the metric flag is set convert the value to metric.
			p->Data.FloatValue *= 25.4;
			return('m');
		}
		else
			return('"');	
	}
	else
		return(' ');
}


/*
 *  FUNCTION: InvertToImperial
 *
 *  PARAMETERS:		p 	-- Pointer to Menu
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:	Inverts metric value and then converts to 
 *					imperial iff data non-zero,  otherwise changes
 *					data to 1.0 since we're not allowed to invert 0.
 *
 *  RETURNS: 		' ', 'm', '"'
 *
 */
int8 
InvertToImperial( TMENU_DATA * p) {
	if (p->Data.FloatValue != 0.0)
		p->Data.FloatValue = 1.0 / p->Data.FloatValue;
	else
		p->Data.FloatValue = 1.0;
	return(ConvertToImperial(p));
}

/*
 *  FUNCTION: InvertToMetric
 *
 *  PARAMETERS:		p 	-- Pointer to Menu
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:	Inverts imperial value and then converts to 
 *					metric iff data non-zero,  otherwise changes
 *					data to 1.0 since we're not allowed to invert 0.
 *
 *  RETURNS: 		' ', 'm', '"'
 *
 */
int8 
InvertToMetric( TMENU_DATA * p) {
	if (p->Data.FloatValue != 0.0)
		p->Data.FloatValue = 1.0 / p->Data.FloatValue;
	else
		p->Data.FloatValue = 1.0;
	return(ConvertToMetric(p));
}

/*
 *  FUNCTION: TaperToImperial
 *
 *  PARAMETERS:		p 	-- Pointer to Menu
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:	Convert users value to imperial if it's in metric and then
 * 					change from Inch/foot to Inch/inch (tan angle)
 *
 *  RETURNS: 		' ', 'm', '"'
 *
 */
int8 
TaperToImperial( TMENU_DATA * p) {
  int8 units;
	 units = ConvertToImperial(p);		// Convert users value to imperial if it's in metric.
	GlobalVars[TAPER_NDX].f = p->Data.FloatValue  / 12.0;  // Convert from Inch/foot to Inch/inch (tan angle)
	// And we've updated the global tapering value too.
	return(units);
}

/*
 *  FUNCTION: TaperToMetric
 *
 *  PARAMETERS:		p 	-- Pointer to Menu
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:	Convert users value to metric.
 *					Just return users value.  In metric if need be.
 *					We don't pull in or touch the global tapering value.
 *
 *  RETURNS: 		' ', 'm', '"'
 *
 */
int8 
TaperToMetric( TMENU_DATA * p) {
  int8 units;
	units = ConvertToMetric(p);	// Just return users value.  In metric if need be.
	// We don't pull in or touch the global tapering value.
	return(units);
}

/*
 *  FUNCTION: TaperDegreesToDistance
 *
 *  PARAMETERS:		p 	-- Pointer to Menu
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:		Convert the full angle degrees to inch/foot of diameter.
 *
 *
 *  RETURNS: 
 *
 */
int8 
TaperDegreesToDistance(TMENU_DATA * p) {
	// Convert the included angle in degrees to inch/inch of diameter.
	p->Data.FloatValue = tan((p->Data.FloatValue)/2 * RADIANS_PER_DEGREE)*2;
	// Then check if metric measurements are wanted.
	if (p->Dependancy != (uint8)0) {	//
		DEBUGSTR("To Metric Var is dependant on %02X\n",p->Dependancy );
		// First get menu from FLASH into data structure
	  	memcpypgm2ram(
			(void *)&d, 
			(MEM_MODEL rom void *)&MenuData[p->Dependancy], 
 			sizeof(TMENU_DATA)
			);
		// Then test the flag.
		if (GetFlagVar(&d)) {	// If the metric flag is set convert the value to metric.
			DEBUGSTR("Convert to Metric\n");
			p->Data.FloatValue *= 25.4;	
			return('m');
		}
		else
			return('"');	
	}
	else
		return(' ');
}

/*
 *  FUNCTION: TaperDistanceToDegrees
 *
 *  PARAMETERS:		p 	-- Pointer to Menu
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:	Convert taper in inch/ft to degrees.
 *
 *  RETURNS: 		Nothing.
 *
 */
int8 
TaperDistanceToDegrees(TMENU_DATA * p) {
	// Conversions are often dependant on whether another flag is set.
	if (p->Dependancy != (uint8)0) {	//
		DEBUGSTR("To Imperial Var is dependant on %02X\n",p->Dependancy );
		// First get menu from FLASH into data structure
	  	memcpypgm2ram(
			(void *)&d, 
			(MEM_MODEL rom void *)&MenuData[p->Dependancy], 
			sizeof(TMENU_DATA)
			);
		// Then test the flag.
		if (GetFlagVar(&d)) {	// If the metric flag is set convert the value to imperial.
			DEBUGSTR("Convert to Imperial\n");
			p->Data.FloatValue /= 25.4;
		}	
	}
	// Now we know we have imperial units in inch per inch and then degrees
	// finally change that to (included) angle degrees.
	p->Data.FloatValue = (atan(p->Data.FloatValue / 2) / RADIANS_PER_DEGREE) * 2;
	return(' ');
}

int8
SetOnboardStepper(TMENU_DATA * p) {
	DEBUGSTR("Enabling SPI MicroStepper\n");
	SSPSTAT = 0x40;		// Transmit on clock high.
	SSPCON1	= 0x20;		// FOsc/4, Enable SPI, Clock Idle Low.
	bMOTOR_LATCH = 0;
	SSPBUF = 0;
	fStepHappened = 0;	// Prevent shutdown from doing anything to motor.
	MicroStep(0,0);		// Set motor off.
}
int8
ClearOnboardStepper(TMENU_DATA * p) {
	DEBUGSTR("Disabling SPI MicroStepper\n");
	SSPCON1	= 0x00;		// FOsc/4, Enable SPI, Clock Idle Low.
}

