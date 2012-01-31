// GlobVars.h -- Functions dealing with global variables.
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


#ifndef __GLOBVARS
#define __GLOBVARS	1

// External Global Variable External Declarations:
#define GLOBAL_VAR_SIZE		41

#define METRIC_PITCH_NDX			0			//	0 == Imperial, 1 == Metric
#define LEADSCREW_IPITCH_NDX		1			//	FLOAT_TYPE PITCH in INCHES
#define SLEW_RATE_X_NDX				2			//  LONG_TYPE Fast motion for returning to start.
#define MORSE_TAPER_LIST_NDX		3			//	uint8_TYPE for Morse Tapers LIST Index.
#define ENCODER_LINES_NDX			4			//	WORD_TYPE
#define SPINDLE_PULSE_REV_NDX		5			//	uint8_TYPE
#define TURN_PITCH_NDX				6			//	FLOAT_TYPE Turning Pitch in "
#define MOTOR_STEPS_REV_Z_NDX		7			//	FLOAT_TYPE
#define LEADSCREW_BACKLASH_NDX		8			//	FLOAT_TYPE 
#define ACCEL_RATE_Z_NDX			9			//	LONG_TYPE
#define SLEW_RATE_Z_NDX				10			//	LONG_TYPE
#define MOVE_RATE_Z_NDX				11			//	LONG_TYPE
#define MOTOR_ZADJ_LENGTH_NDX		12			//	FLOAT_TYPE Distance moved on each jog tap or encoder click.
#define JACOBS_TAPER_LIST_NDX		13			//	uint8_TYPE Index for which Jacobs Taper is active.
#define THREAD_BEGIN_NDX			14			//	FLOAT_TYPE Float Begin position in STEPS from 0.
#define THREAD_END_NDX				15			//	FLOAT_TYPE Float End position in STEPS from 0.
#define THREAD_Z_HOME_NDX			16			//	FLOAT_TYPE Float Home position in STEPS from Home.
#define MOTOR_POWER_DOWN_NDX		17			//	uint8 # of 1/2 sec heartbeats before motor powers down.
#define CROSS_SLIDE_IPITCH_NDX		18  		//	FLOAT_TYPE Leadscrew pitch in INCHES
#define MOVE_RATE_X_NDX				19  		//	LONG_TYPE 
#define ACCEL_RATE_X_NDX			20  		//	LONG_TYPE 
#define MOTOR_STEPS_REV_X_NDX		21  		//	WORD_TYPE 
#define MOTOR_XADJ_LENGTH_NDX		22			//	FLOAT_TYPE Distance moved on each jog tap or encoder click.
#define X_AXIS_BACKLASH_NDX			23			//  FLOAT_TYPE backlash of X axis in inch.
#define ASSORTED_TAPER_LIST_NDX 	24  		//	uint8 Assorted Taper list index.
#define TAPER_NDX					25			//	FLOAT_TYPE Taper in inches per inch.
#define THREAD_SIZE_NDX				26			//	FLOAT_TYPE Thread pitch in inches.
#define	THREAD_ANGLE_NDX			27			//	FLOAT_TYPE Half the included angle or the angle of thread flank
#define RETRACTED_X_NDX				28  		//	FLOAT_TYPE Float holds where X should move to when clear of threads.
#define BEGIN_X_NDX					29  		//	FLOAT_TYPE Float holds where X begins cutting
#define PASS_FIRST_X_DEPTH_NDX		30			//	FLOAT_TYPE How much to advance X for first pass.
#define PASS_EACH_X_DEPTH_NDX		31			//	FLOAT_TYPE How much to advance X for each pass.
#define PASS_END_X_DEPTH_NDX		32			//	FLOAT_TYPE Last Pass cutting depth.
#define PASS_SPRING_CNT_NDX			33			//	uint8_TYPE How many passes at last depth.
#define DEPTH_X_NDX					34			//	FLOAT_TYPE Depth of thread.
#define CUSTOM_TAPER1_NDX			35			//	FLOAT_TYPE Taper in inches per inch.
#define CUSTOM_TAPER2_NDX			36			//	FLOAT_TYPE Taper in inches per inch.
#define CUSTOM_TAPER3_NDX			37			//	FLOAT_TYPE Taper in inches per inch.
#define CUSTOM_TAPER4_NDX			38			//	FLOAT_TYPE Taper in inches per inch.
#define X_DIAMETER_NDX				39			//  FLOAT_TYPE location of tool bit tip expressed as diameter
#define DEPTH_MULTIPLIER_NDX		40			//  FLOAT_TYPE location of tool bit tip expressed as diameter
 
extern PARAMETERS GlobalVars[GLOBAL_VAR_SIZE];
extern rom float GlobalMinimums[GLOBAL_VAR_SIZE];
extern rom float GlobalMaximums[GLOBAL_VAR_SIZE];



// Functions in GlobVars.c
float32 GetGlobalVarFloat(int16 ndx);
int32 GetGlobalVarLong(int16 ndx);
uint16 GetGlobalVarWord(int16 ndx);
uint8 GetGlobalVarByte(int16 ndx);
void SetGlobalVarFloat(int16 ndx, float var);
void SetGlobalVarLong(int16 ndx, long var);
void SaveGlobalVar(int16 ndx);
void LoadGlobalVar(int16 ndx);
void SaveFlagVar(PTMENU_DATA pMenu);
uint8 GetFlagVar(PTMENU_DATA pMenu);
void ReadFlags(void);
void InitDefaultGlobalVars(void);
void RestoreGlobalVariables(void);
#endif
