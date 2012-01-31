/* 
    GlobVars.c -- E-Leadscrew definitions for an electronic replacement of
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
// GlobVars.c for project
#include "processor.h"
#include "common.h"

#include "menu.h"	
#include "config.h"

#include "serial.h"
#include "MotorDriver.h"

#include "OB_EEROM.h"
#include "globvars.h"

/*
	Global Variables and Functions that manipulate them.
*/
uint8 HB_OnTime, HB_OffTime;		// Timer values for context sensitive Heartbeat.

// Copies of the flags maintained in EEROM.
BITS SYSFlags;	// EEROM'd System configuratiion flags.
BITS SYS2Flags;	// EEROM'd System configuratiion flags.
BITS MgmtFlags;	// EEROM'd extra management flags.

// Flags not EEROM'd but used for program operation.
BITS CONTROLFlags;
BITS EDITFlags;
BITS RUNFlags;

// Thread control flags.
BITS OkToStopFlags;
BITS ThreadsRdy;
BITS ThreadsRun;

enum SYSTEM_COMMANDS SystemCommand;



/* 
	Global Parameters.  Contained in an array indexed by a constant.  That allows them to be 
	saved and stored to EEROM.  
	Although not implimented, the globals have an associated MIN/MAX constant used for ensuring that a 
	user doesn't enter in an invalid value.
*/  
PARAMETERS GlobalVars[GLOBAL_VAR_SIZE];

// Global Minimum values tested when a user enters data.
rom float32 GlobalMinimums[GLOBAL_VAR_SIZE] = {
	0.0, 	//	NU_METRIC_PITCH_NDX	  		0 == Imperial, 1 == Metric
	0.001,	//	LEADSCREW_IPITCH_NDX  		FLOAT_TYPE PITCH in INCHES
	0.0,	//  SLEW_RATE_X_NDX 			LONG_TYPE Fast motion for returning to start.
	0.0	, 	//	TAPER_LIST_NDX		  		uint8 index for LIST_TYPE
	0.0, 	//	ENCODER_LINES_NDX	  		WORD_TYPE
	0.0, 	//	SPINDLE_PULSE_REV_NDX 		uint8_TYPE
	0.0001,	//	TURN_PITCH_NDX		  		FLOAT_TYPE Turning Pitch in INCH
	0.0, 	//	MOTOR_STEPS_REV_Z_NDX 		FLOAT_TYPE
	0.000, 	//	LEADSCREW_BACKLASH_NDX		FLOAT_TYPE 
	0.0, 	//	MOTOR_ACCEL_Z_NDX	  		LONG_TYPE
	0.0, 	//	MOTOR_SLEW_RATE_NDX	  		LONG_TYPE
	0.0, 	//	MOTOR_HOME_RATE_NDX	  		LONG_TYPE
	0.0001,	//	MOTOR_ZADJ_LENGTH_NDX 		FLOAT_TYPE Distance moved on each jog tap or encoder click.
	0.0, 	//	NU_MOTOR_DIRECTION_NDX	  
	0.0, 	//	THREAD_BEGIN_NDX	  		FLOAT_TYPE Float Begin position in STEPS from 0.
	0.0, 	//	THREAD_END_NDX		  		FLOAT_TYPE Float End position in STEPS from 0.
	0.0, 	//	THREAD_Z_HOME_NDX	  		FLOAT_TYPE Float Home position in STEPS from Home.
	0.0, 	//	MOTOR_POWER_DOWN_NDX  		uint8 # of 1/2 sec heartbeats before motor powers down.
	0.001, 	//	CROSS_SLIDE_IPITCH_NDX		FLOAT_TYPE Leadscrew pitch in INCHES
	0.0, 	//	MOVE_RATE_X_NDX		  		LONG_TYPE 
	0.0, 	//	ACCEL_RATE_X_NDX	  		LONG_TYPE 
	0.0, 	//	MOTOR_STEPS_REV_X_NDX 		WORD_TYPE 
	0.0001,	//	MOTOR_XADJ_LENGTH_NDX 		FLOAT_TYPE Distance moved on each jog tap or encoder click.
	0.000, 	//	X_AXIS_BACKLASH_NDX			FLOAT_TYPE 
	0.0, 	//	ASSORTED_TAPER_LIST_NDX			  	
	0.0, 	//	TAPER_NDX	  				FLOAT_TYPE Taper in inches per inch.
	0.0001,	//	THREAD_SIZE_NDX		  		FLOAT_TYPE Thread pitch in INCH
	0.0, 	//	THREAD_ANGLE_NDX	  		FLOAT_TYPE Half the included angle or the angle of thread flank
	-10.0, 	//	RETRACTED_X_NDX		  		FLOAT_TYPE Float holds where X should move to when clear of threads.
	-10.0, 	//	BEGIN_X_NDX			  		FLOAT_TYPE Float holds where X cuts
	0.0, 	//	PASS_FIRST_X_DEPTH_NDX	  	uint8_TYPE How many passes to cut a thread.  0 for non-automatic.
	0.0, 	//	PASS_EACH_X_DEPTH_NDX	  	uint8_TYPE How many passes to cut a thread.  0 for non-automatic.
	0.0, 	//	PASS_END_X_DEPTH_NDX	  	uint8_TYPE How many passes to cut a thread.  0 for non-automatic.
	0.0, 	//	PASS_SPRING_CNT_NDX  		uint8_TYPE How many passes at last depth.
	0.0, 	//	DEPTH_X_NDX			  		FLOAT_TYPE At what depth to stop advancing in percent of sharp thread depth H
	0.0,	//  CUSTOM_TAPER1_NDX				FLOAT_TYPE Taper in inches per inch.
	0.0,	//  CUSTOM_TAPER2_NDX			FLOAT_TYPE Taper in inches per inch.
	0.0,	//  CUSTOM_TAPER3_NDX			FLOAT_TYPE Taper in inches per inch.
	0.0,	//  CUSTOM_TAPER4_NDX			FLOAT_TYPE Taper in inches per inch.
	0.0,	//  X_DIAMETER					FLOAT_TYPE location of tool bit tip expressed as diameter
	0.0		//  DEPTH_MULTIPLIER_NDX		FLOAT_TYPE Used to calculate thread depth from pitch.
};

// Global Maximum values tested when a user enters data. 
rom float32 GlobalMaximums[GLOBAL_VAR_SIZE] = {
	0.0, 	//	METRIC_PITCH_NDX	  		0 == Imperial, 1 == Metric
	200.0, 	//	LEADSCREW_IPITCH_NDX  		FLOAT_TYPE PITCH in INCHES
	20000.0,	//  SLEW_RATE_X_NDX			LONG_TYPE Fast motion for returning to start.
	200.0, 	//	MORSE TAPER_LIST_NDX  		uint8 Index for LIST_TYPE
	0.0, 	//	ENCODER_LINES_NDX	  		WORD_TYPE
	0.0, 	//	SPINDLE_PULSE_REV_NDX 		uint8_TYPE
	200.0, 	//	TURN_PITCH_NDX		  		FLOAT_TYPE Turning Pitch in MM
	0.0, 	//	MOTOR_STEPS_REV_Z_NDX 		FLOAT_TYPE
	0.100, 	//	LEADSCREW_BACKLASH_NDX		FLOAT_TYPE 
	0.0, 	//	MOTOR_ACCEL_Z_NDX	  		LONG_TYPE
	0.0, 	//	MOTOR_SLEW_RATE_NDX	  		LONG_TYPE
	0.0, 	//	MOTOR_HOME_RATE_NDX	  		LONG_TYPE
	0.200, 	//	MOTOR_ZADJ_LENGTH_NDX 		FLOAT_TYPE Distance moved on each jog tap or encoder click.
	0.0, 	//	JACOBS_TAPER_LIST_NDX		uint8_TYPE Index for which Jacobs Taper is active.
	0.0, 	//	THREAD_BEGIN_NDX	  		FLOAT_TYPE Float Begin position in STEPS from 0.
	0.0, 	//	THREAD_END_NDX		  		FLOAT_TYPE Float End position in STEPS from 0.
	0.0, 	//	THREAD_Z_HOME_NDX	  		FLOAT_TYPE Float Home position in STEPS from Home.
	0.0, 	//	MOTOR_POWER_DOWN_NDX  		uint8 # of 1/2 sec heartbeats before motor powers down.
	200.0, 	//	CROSS_SLIDE_IPITCH_NDX		FLOAT_TYPE Leadscrew pitch in INCHES
	0.0, 	//	MOVE_RATE_X_NDX		  		LONG_TYPE 
	0.0, 	//	ACCEL_RATE_X_NDX	  		LONG_TYPE 
	0.0, 	//	MOTOR_STEPS_REV_X_NDX 		WORD_TYPE 
	0.0001,	//	MOTOR_XADJ_LENGTH_NDX 		FLOAT_TYPE Distance moved on each jog tap or encoder click.
	1.0, 	//	X_AXIS_BACKLASH_NDX			FLOAT_TYPE 
	0.0, 	//	ASSORTED_TAPER_LIST_NDX  	uint8_TYPE Assorted Taper list index.		  	
	1.0, 	//	TAPER_NDX	  				FLOAT_TYPE Taper in inches per inch.
	5.0,	//	THREAD_SIZE_NDX		  		FLOAT_TYPE Thread pitch in INCH
	90.0, 	//	THREAD_ANGLE_NDX	  		FLOAT_TYPE Half the included angle or the angle of thread flank
	10.0, 	//	RETRACTED_X_NDX		  		FLOAT_TYPE Float holds where X should move to when clear of threads.
	10.0, 	//	BEGIN_X_NDX			  		FLOAT_TYPE Float holds where X cuts
	1.0, 	//	PASS_FIRST_X_DEPTH_NDX	  	uint8_TYPE How many passes to cut a thread.  0 for non-automatic.
	1.0, 	//	PASS_EACH_X_DEPTH_NDX	  	uint8_TYPE How many passes to cut a thread.  0 for non-automatic.
	1.0, 	//	PASS_END_X_DEPTH_NDX	  	uint8_TYPE How many passes to cut a thread.  0 for non-automatic.
	99.0, 	//	PASS_SPRING_CNT_NDX  		uint8_TYPE How many passes at last depth.
	0.0, 	//	DEPTH_X_NDX			  		FLOAT_TYPE At what depth to stop advancing in percent of sharp thread depth H
	0.0,		//  CUSTOM_TAPER1_NDX			FLOAT_TYPE Taper in inches per inch.
	0.0,		//  CUSTOM_TAPER2_NDX			FLOAT_TYPE Taper in inches per inch.
	0.0,		//  CUSTOM_TAPER3_NDX			FLOAT_TYPE Taper in inches per inch.
	0.0,		//  CUSTOM_TAPER4_NDX			FLOAT_TYPE Taper in inches per inch.
	0.0,		//  X_DIAMETER					FLOAT_TYPE location of tool bit tip expressed as diameter
	0.0		//  DEPTH_MULTIPLIER_NDX		FLOAT_TYPE Used to calculate thread depth from pitch.
};

int8 SystemError;

int8 MotionPitchIndex;	// Index into globals as to which pitch to use.

int8 ZEncoderCounter, OldZEncoderCounter;

float32 TrackingRatio;			// LeadscrewRatio * PULSE_POINT.
int32 iTrackingRatio;			// Integer version


// Serial and I/O conversion variables.
int16 DecArg;		// Holds value to manipulate or output.


int32 CurrentZPosition;		// Position of carriage in motor steps.
int32 CurrentXPosition;		// Position of cross slide in motor steps

// These two variables hold the calculated values determined from EEROM parameters.
// Holding the calculations in one single variable speeds execution time.
float32 XDistanceDivisor, ZDistanceDivisor;

int8 DisplayModeMenuIndex;	//  Which menu is currently displayed on the LCD.

enum SYSTEM_STATES SystemState;		// System state variable.

/*
	Although a function call is slower than directly addressing the array this approach, like a method
	in an object, allows a certain level of abstraction.  The following functions impliment this for
	the various menu entries and make the code more readable.
*/
// Float variables.
float32 GetGlobalVarFloat(int16 ndx) {
	return(GlobalVars[ndx].f);
}
// Long variables
int32 GetGlobalVarLong(int16 ndx) {
	return(GlobalVars[ndx].l);
}
// Word Variables.
uint16 GetGlobalVarWord(int16 ndx) {
	return((WORD)GlobalVars[ndx].l);
}
// Byte variables.
uint8 GetGlobalVarByte(int16 ndx) {
	return((uint8)GlobalVars[ndx].l);
}

/*
	As in the GetGlobal... functions the SetGlobal make the application code more readable and 
	provide a level of abstraction.  
*/
// Store a float in the global array.
void SetGlobalVarFloat(int16 ndx, float32 var) {
	GlobalVars[ndx].f = var;
}
// Store a long in the global array.
void SetGlobalVarLong(int16 ndx, int32 var) {
	GlobalVars[ndx].l = var;
}

/*
	Save a global variable to EEROM into the global array..
*/
void SaveGlobalVar(int16 ndx) {
	Put_ObEEROM_Float( (ndx * 4)+EM_GLOBAL_VARS, &GlobalVars[ndx].f);
}

/*
	Get a global variable from EEROM into the global array.
*/
void LoadGlobalVar(int16 ndx) {
  int16 e_addr;
  float f_data;
	e_addr = (ndx * 4)+EM_GLOBAL_VARS;
	Get_ObEEROM_Float(e_addr, &f_data);
	GlobalVars[ndx].f = f_data;
}

/*
	Stores the flag value contained in a Menu Entry to EEROM.
	The value of the flag is 
	A flag is a single bit within a BYTE.  The Byte is stored as the menu's Pos 
	value because the Flag String text is stored in a predetermined point on the 
	LCD display.
	The specific bit of a flag is stored as the Len value as an index from 0..7
	
*/
void SaveFlagVar(PTMENU_DATA pMenu) {
uint8 tmp;
	tmp = Get_ObEEROM_Byte(pMenu->Pos);
	if (pMenu->Data.Next[0])
		Put_ObEEROM_Byte(pMenu->Pos, tmp | (1 << pMenu->Len));
	else
		Put_ObEEROM_Byte(pMenu->Pos, tmp & ~(1 << pMenu->Len));		
}

uint8 GetFlagVar(PTMENU_DATA pMenu) {
	return ((Get_ObEEROM_Byte(pMenu->Pos) & (1<<pMenu->Len)) != 0);
}

// Bring in EEROM flags into RAM image.
// Used after flags have been modified by users.
void 
ReadFlags(void) {
	SYSFlags.Byte   = Get_ObEEROM_Byte(EM_SYS_FLAGS);	// Bit flags default to 0.
	// For now until menus arecdone, there are no set flags in SYS2Flags.
	SYS2Flags.Byte   = 0; //Get_ObEEROM_Byte(EM_SYS2_FLAGS);	// Bit flags default to 0.
	MgmtFlags.Byte = Get_ObEEROM_Byte(EM_MGMT_FLAGS);
	ZMotorFlags.Byte = Get_ObEEROM_Byte(EM_ZMOTOR_FLAGS);
	XMotorFlags.Byte = Get_ObEEROM_Byte(EM_XMOTOR_FLAGS);
   	CommFlags.Byte  = Get_ObEEROM_Byte(EM_COMM_FLAGS);
}


/*
	Hardcode EEROM parameters to default values.  Called when user executes a specific key sequence.
*/
void
InitDefaultGlobalVars(void) {
	int8 i;

	GlobalVars[LEADSCREW_IPITCH_NDX].f 		=  1.0/10.0; 	// FLOAT_TYPE Leadscrew PITCH in INCHES
	GlobalVars[ENCODER_LINES_NDX].l 		=  1; 		  	// WORD_TYPE
	GlobalVars[SPINDLE_PULSE_REV_NDX].l 	=  1; 		  	// uint8_TYPE
	GlobalVars[TURN_PITCH_NDX].f 			=  0.005; 	  	// FLOAT_TYPE Turning Pitch in INCH
	GlobalVars[MOTOR_STEPS_REV_Z_NDX].f		=  1600.0;	  	// WORD_TYPE
	GlobalVars[LEADSCREW_BACKLASH_NDX].f	= 0.000; 		// FLOAT_TYPE Backlash in carriage.
	GlobalVars[ACCEL_RATE_Z_NDX].l 			= 9000;		  	// LONG_TYPE
	GlobalVars[SLEW_RATE_Z_NDX].l 			= 10000;  		// LONG_TYPE
	GlobalVars[MOVE_RATE_Z_NDX].l			= 3000;  		// LONG_TYPE
	GlobalVars[MOTOR_ZADJ_LENGTH_NDX].f		= 0.001; 	  	// FLOAT_TYPE Distance moved on each jog tap or encoder click.
	GlobalVars[THREAD_BEGIN_NDX].f 			= 0.0; 		  	// FLOAT_TYPE Float Start position in STEPS from 0.
	GlobalVars[THREAD_END_NDX].f 			= 0.0;		  	// FLOAT_TYPE Float End position in STEPS from 0.
	GlobalVars[THREAD_Z_HOME_NDX].f 		= 0.0;	  		// FLOAT_TYPE Float Home position in STEPS from Home.
	GlobalVars[MOTOR_POWER_DOWN_NDX].l		= 5;	  		// uint8 # of 1/2 sec heartbeats before motor powers down.
	GlobalVars[CROSS_SLIDE_IPITCH_NDX].f	= 0.050; 	  	// FLOAT_TYPE Leadscrew pitch in INCHES
	GlobalVars[SLEW_RATE_X_NDX].l			= 3000;	  		// LONG_TYPE 
	GlobalVars[MOVE_RATE_X_NDX].l			= 1000;	  		// LONG_TYPE 
	GlobalVars[ACCEL_RATE_X_NDX].l	  		= 3000;	  		// LONG_TYPE 
	GlobalVars[MOTOR_STEPS_REV_X_NDX].l		= 2400;	  		// WORD_TYPE 
	GlobalVars[MOTOR_XADJ_LENGTH_NDX].f 	= 0.001; 	  	// FLOAT_TYPE Distance moved on each jog tap or encoder click.
	GlobalVars[X_AXIS_BACKLASH_NDX].f 		= 0.000; 	  	// FLOAT_TYPE Backlash in X axis.
	GlobalVars[THREAD_SIZE_NDX].f 			= 0.050; 	  	// FLOAT_TYPE Thread pitch in INCH
	GlobalVars[THREAD_ANGLE_NDX].f			= 29.5; 		// FLOAT_TYPE Half the included angle or the angle of thread flank
	GlobalVars[BEGIN_X_NDX].f				= 0.0; 		  	// FLOAT_TYPE Float holds where X cuts
	GlobalVars[RETRACTED_X_NDX].f			= 0.1; 		  	// FLOAT_TYPE Float holds where X should move to when clear of threads.
	GlobalVars[PASS_FIRST_X_DEPTH_NDX].f  	= 0.005; 		// uint8_TYPE How many passes to cut a thread.  0 for non-automatic.
	GlobalVars[PASS_EACH_X_DEPTH_NDX].f  	= 0.002; 		// uint8_TYPE How many passes to cut a thread.  0 for non-automatic.
	GlobalVars[PASS_END_X_DEPTH_NDX].f  	= 0.001; 		// uint8_TYPE How many passes to cut a thread.  0 for non-automatic.
	GlobalVars[PASS_SPRING_CNT_NDX].l		= 3; 		  	// uint8_TYPE How many passes at last depth.
	GlobalVars[DEPTH_X_NDX].f	  			= 0.050; 		// FLOAT_TYPE At what depth to stop advancing in percent of sharp thread depth H
					  					 		  			// which extends above surface of Maximum diameter.

	GlobalVars[TAPER_NDX].f			  		= 0.049950; 		// FLOAT_TYPE Taper in inches per inch.
	GlobalVars[MORSE_TAPER_LIST_NDX].l		= 0;	  		// uint8 index for LIST_TYPE 
	GlobalVars[JACOBS_TAPER_LIST_NDX].l		= 0;	  		// uint8_TYPE Index for which Jacobs Taper is active.
	GlobalVars[ASSORTED_TAPER_LIST_NDX].l  	= 0;	  		// uint8_TYPE Assorted Taper list index.		  	
	GlobalVars[CUSTOM_TAPER1_NDX].f			= 0.0;			// FLOAT_TYPE Taper in inches per inch.
	GlobalVars[CUSTOM_TAPER2_NDX].f			= 0.0;			// FLOAT_TYPE Taper in inches per inch.
	GlobalVars[CUSTOM_TAPER3_NDX].f			= 0.0;			// FLOAT_TYPE Taper in inches per inch.
	GlobalVars[CUSTOM_TAPER4_NDX].f			= 0.0;			// FLOAT_TYPE Taper in inches per inch.
	GlobalVars[X_DIAMETER_NDX].f			= 0.0;			//  FLOAT_TYPE location of tool bit tip expressed as diameter
	GlobalVars[DEPTH_MULTIPLIER_NDX].f		= 0.5413;		//  FLOAT_TYPE Used to calculate thread depth from pitch.

	// Now that they are initialized, save them to EEROM.
	for (i=0; i<GLOBAL_VAR_SIZE; i++)
		SaveGlobalVar(i);

	// config.h
	Put_ObEEROM_Byte(EM_SYS_FLAGS,0x0C);	// Default System flags.  See below
	// Serial.h
	Put_ObEEROM_Byte(EM_COMM_FLAGS,0x01);	// Bit0 == 1 if local echo and backspace enable. See Serial.h
	// MotorDriver.h
	Put_ObEEROM_Byte(EM_ZMOTOR_FLAGS,0x00);	// Bits that tell us about our Z axis motor.  See MotorDriver.h
	Put_ObEEROM_Byte(EM_XMOTOR_FLAGS,0x00);	// Bits that tell us about our X Axis motor.  See MotorDriver.h
	Put_ObEEROM_Byte(EM_MGMT_FLAGS,0x00);	// Extra flags.
	Put_ObEEROM_Byte(EM_BAUD,B115K);		// Serial Port Baud rate.
}


void 
RestoreGlobalVariables(void) {
  int8 i;
	for (i=0; i<GLOBAL_VAR_SIZE; i++) 
		LoadGlobalVar(i);
}
