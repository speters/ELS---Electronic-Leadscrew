/* 
    MotorDriver.c -- MotorControl code for an electronic replacement of
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

// Bring in processor definition and math library
#include "Processor.h"

#include <Ctype.h>
#include <stdio.h>
#include <float.h>
#include <stdlib.h>
#include <math.h>

#include "Common.h"			// Common constants.
#include "Config.h"			// Project specific definitions

#include "Menu.h"	
#include "GlobVars.h"	

#include "Int.h"

#include "Timer.h"
#include "FloatIO.h"
#include "Serial.h"
#include "OB_EEROM.h"		// EEROM Access definitions

#include "MotorDriver.h"	// Motor Driver Defintions.

/* --- Public Variables --- */
// EEROM Based Flags
BITS ZMotorFlags;
BITS XMotorFlags;

// Dynamic Flags
BITS TaperFlags;			// Used for determining backlash removal of X while tapering.
BITS ZStepFlags;
BITS XStepFlags;

// Device Driver State Machines.
enum MOTOR_STATES MotorState = MOTOR_STOPPED;
enum RPM_STATES RPMState = RPM_ZERO;

int32 XStartMotorPosition;		// Each pass starts here.  Changed if auto depth enabled.
volatile int32 XStepCount;		// Number of steps to move.

volatile uint16 SpindleClocksPerRevolution;// Holds last accumulated number of Pulse Clocks per Rev.
float32 LeadScrewRatio;			// Calculated from Leadscrew Pitch, Motor Steps and Desired feed rate.

// Running average counter array for Spindle RPM.
int32 RPMAverage[16];
static int8 RPMAverageIndex;
int32 AveragedClocksPerRev;		// An average of the number of TICKs between spindle sensor interrupts.
int16 AverageRPM;
int16 TargetRPM;
int32 LastSpindleClocks;		// Holds the newest of TICKs between spindle sensor interrupt.

/* --- Private Variables --- */
float32 r,s;						// Temporary variables.


/* --- Private Functions --- */
float32 SetupThreadDivision(int8 ndx, pint32 pTrkRatio);	// Argument is index to global variable distance per spindle rev.

/*
 *  FUNCTION: labs
 *
 *  PARAMETERS:	32 bit signed long
 *
 *  DESCRIPTION: Takes the absolute value of the long argument.
 *
 *  RETURNS: The absolute value.
 *
 */
int32 
labs(int32 arg) {
	return((arg<0) ? 0-arg : arg);
}

/* --- Public Functions --- */
/*
 *  FUNCTION: InitMotorDevice
 *
 *  PARAMETERS: none
 *
 *  DESCRIPTION: Initialize Motor Device Statemachine.
 *
 *  RETURNS: nothing
 *
 */
void
InitMotorDevice(void) {
  int8 i;

	CCPR1L = PULSE_CLOCK_DIVISOR;    // Divide 40MHz by 4,  interrupt after PULSE_CLOCK_DIVISOR clocks == PULSE_POINT HZ
	CCPR1H = PULSE_CLOCK_DIVISOR>>8;
	CCP1CON = 0b00001011;   		// Compare generates an interrupt and clears Timer1.
	T1CON = 0b00000001;

	// Initialize variables.
	ZStepFlags.Byte = 0;				// Clear all Flags.  Nothing's happened yet.
	XStepFlags.Byte = 0;				// Clear all Flags.  Nothing's happened yet.

	CurrentXPosition = 0;
	CurrentZPosition = 0;


	ZAcc = GetGlobalVarLong(ACCEL_RATE_Z_NDX) << 5; 	// Get saved Acceleration.
	ZVel = 0; 						// Motor not turning yet.
	MaxZVel = 0;						// Motor not allowed to turn yet.
	SystemZBackLashCount = 0;
	LeadScrewRatio = SetupThreadDivision(MotionPitchIndex, &iTrackingRatio);

#ifdef X_AXIS
	XAcc = GetGlobalVarLong(ACCEL_RATE_X_NDX) << 5; 	// Get saved Acceleration.
	XVel = 0; 						// Motor not turning yet.
	MaxXVel = 0;					// Motor not allowed to turn yet.
	XStartMotorPosition = 0;
	XBackLashCount = 0;				// No backlash movement to start with.
	TaperFlags.Byte = 0;		// Clear out Taper Movement flag so top bits aren't random.
#endif

	for (i=0;i<16; i++)
		RPMAverage[i] = 0;
	RPMAverageIndex = 0;
	AveragedClocksPerRev = 0;
	AverageRPM = 0;
}

/*
 *  FUNCTION: StepsToZDistance
 *
 *  PARAMETERS: Uses Global fMetricMode Flag
 *				Accesses global variables 
 *					LEADSCREW_IPITCH_NDX, 
 * 					MOTOR_STEPS_REV_Z_NDX 
 *
 *  DESCRIPTION: Converts Z axis motor steps to distance in mm or inches
 *  			 using the motor steps and lead screw pitch.
 *
 *  RETURNS: float32 distance in mm or inches.
 *
 */
float32
StepsToZDistance( int32 steps) {
  float32 result;
	if (GlobalVars[LEADSCREW_IPITCH_NDX].f <= 0.0)
		GlobalVars[LEADSCREW_IPITCH_NDX].f = 1.0;
	result = (float32)steps / (GlobalVars[MOTOR_STEPS_REV_Z_NDX].f / GlobalVars[LEADSCREW_IPITCH_NDX].f);
	// Internally distances are maintined in inches.
	// But display them as mm if flag is set.
	if (fMetricMode) 
		result *= 25.4;	
	return(result);	
}

/*
 *  FUNCTION: ZDistanceToSteps
 *
 *  PARAMETERS:	Distance in mm or inches
 *
 *  DESCRIPTION: Uses Global fMetricMode Flag
 *				 Accesses global variables 
 *				 	LEADSCREW_IPITCH_NDX, 
 *				 	MOTOR_STEPS_REV_Z_NDX 
 *
 *  RETURNS: Distance in Z axis motor steps.
 *
 */
int32
ZDistanceToSteps( float32 distance) {
  int32 result;
	// Internally distances are maintined in inches.
	// So convert them back to inches if they are displayed in mm.
	if (fMetricMode) 
		distance /= 25.4;
	// Multiply Distance by # of steps per inch.		
	result = ((distance *	
				(GlobalVars[MOTOR_STEPS_REV_Z_NDX].f / GlobalVars[LEADSCREW_IPITCH_NDX].f)) + 0.5);
	return result;
}

/*
 *  FUNCTION: StepsToXDistance
 *
 *  PARAMETERS: Uses Global fMetricMode Flag
 *				Accesses global variables 
 *					CROSS_SLIDE_IPITCH_NDX, 
 *					MOTOR_STEPS_REV_X_NDX 
 *
 *  DESCRIPTION: 
 *
 *  RETURNS: distance in mm or inches in float32 format.
 *
 */
float32
StepsToXDistance( int32 steps ) {
  float32 pitch = GlobalVars[CROSS_SLIDE_IPITCH_NDX].f;
  float32 result;

	if ((*(unsigned int32 *)&pitch == 0xFFFFFFFF) || (pitch == 0.0)) {
		pitch = 1.0;
		result = 0.0;
	}
	result = steps / ((float32)GlobalVars[MOTOR_STEPS_REV_X_NDX].l / pitch);
	// Internally distances are maintined in inches.
	// but if metric mode then return mm.
	if (fMetricMode) 
		result *= 25.4;	
	return(result);	
}

/*
 *  FUNCTION: XDistanceToSteps
 *
 *  PARAMETERS: Distance in inches or mm.
 *				Uses Global fMetricMode Flag
 *				Accesses global variables 
 *					CROSS_SLIDE_IPITCH_NDX, 
 *					MOTOR_STEPS_REV_X_NDX 
 *
 *  DESCRIPTION: Converts distance in world units into steps based 
 *				 on cross feed screw Pitch and motor steps per revoltion
 *
 *  RETURNS: Distance in steps for X axis.
 *
 */
int32 
XDistanceToSteps( float32 distance) {
  float32 pitch = GlobalVars[CROSS_SLIDE_IPITCH_NDX].f;
  int32 result;

	if ((*(unsigned int32 *)&pitch == 0xFFFFFFFF) || (pitch == 0.0))
		pitch = 1.0;

	// Internally distances are maintined in inches.
	// So convert them back to inches to calculate stepper distance.
	if (fMetricMode) 
		distance /= 25.4;		
	result = ((distance *
		((float32)GlobalVars[MOTOR_STEPS_REV_X_NDX].l / pitch))+0.5);
	return(result);
}

/*
 *  FUNCTION: PrintRPM
 *
 *  PARAMETERS: Flag to print to serial 
 *				Flag to Convert RPM to SFM
 *				Uses Global 
 *					AverageRPM
 *					fMetricMode
 *					X_DIAMETER_NDX
 *
 *  DESCRIPTION: Call this function to query the Global AverageRPM 
 *				 value and if debugging is enabled print to the serial port.
 *				 If fShowSFM is set then convert RPM to SFM or SMM if fMetric Mode.
 *
 *  RETURNS: int16 SFM (SMM) or RPM based on fShowSFM
 *
 */
int16 
PrintRPM(int8 showSerial, int8 fShowSFM) {
int16 SFM;
float XDiameter;
	if (fShowSFM) {
		XDiameter =  GetGlobalVarFloat(X_DIAMETER_NDX) * 0.262 * (float)AverageRPM;
		if (fMetricMode) 
			XDiameter /= 3.2808;	// Convert to SMM
		SFM = XDiameter + 0.5;  // Round and then truncate.
		if (showSerial) DEBUGSTR("SFM=%4d", SFM);
		return(SFM);
	}
	else {
		if (showSerial) DEBUGSTR("RPM=%4d", AverageRPM);
		return(AverageRPM);
	}
}

/*
 *  FUNCTION: SetupThreadDivision
 *
 *  PARAMETERS: ndx	--	Index to Thread or Turning Pitch global variable.
 *
 *  USES GLOBALS:	LEADSCREW_IPITCH_NDX
 *					MOTOR_STEPS_REV_Z_NDX
 *					PULSE_CLOCK_RATE
 *					iTrackingRatio
 *
 *  DESCRIPTION: Call this to set up parameters for feeding 
 *					For example: 
 *					 	If lead screw is 10 TPI then pitch is 0.1"
 *						If Stepper motor turns 2000 steps per revolution,
 *						and if desired thread is 20 TPI or 0.050" pitch,
 *						then (0.050/0.1)*2000 = 1000 steps per spindle revolution.
 *
 *  RETURNS: The number of steps that Z axis has to step to move the carriage 
 *			 during one spindle rev as a float.
 *			 iTrackingRatio is set to actual pulse rate based on PULSE_CLOCK_RATE
 *
 */
float32
SetupThreadDivision(int8 ndx, pint32 pTrkRatio) {	
  float32 r;
	r = GetGlobalVarFloat(LEADSCREW_IPITCH_NDX);
	r = (GetGlobalVarFloat(ndx)/r) * GetGlobalVarFloat(MOTOR_STEPS_REV_Z_NDX);
	if (r <= 0.0) 
		r = 1.0;

	*pTrkRatio = r * PULSE_CLOCK_RATE;			 // Steps per spindle rev * PULSE_CLOCK_RATE.

	return(r);
}

/*
 *  FUNCTION: SetupMotorSpeed
 *
 *  PARAMETERS: 		
 *		ratio : Z axis motor steps per spindle revolution times PULSE CLOCK rate.
 *
 *
 *  USES GLOBALS:
 * 		PULSE_CLOCK_RATE	    -- Frequency of pulse clock
 *		AveragedClocksPerRev	-- Number of pulse clocks per rev.
 *
 *  DESCRIPTION:
 *		Example:
 *			Spindle is running 300 RPM.
 *			PULSE_CLOCK_RATE is 20000
 *			Turning 20TPI thread with a 10 TPI lead screw and 2000 step per rev motor.
 *			Therefore ratio = (1000 calculated in SetupThreadDivision) * 20000
 *			At 300 RPM we have 5 RPS or 200ms divided by our pulse clock period 1/20000 (50uS) = 4000
 *			so Stepper clock rate is 20000000/4000 = 5000 Hz.
 *
 *  RETURNS: 
 *		(Z axis Motor Steps per spindle Rev  * PULSE_CLOCK_RATE) / AveragedCLocksPerRev
 *
 */
int32 
SetupMotorSpeed(int32 ratio) {

  int32 StepperMotorClockRate = 0;

/*#ifdef TRACK_SPINDLE_SPEED
	// If tracking spindle speed then use the speed at the time we press start.
	if (LastSpindleClocks != 0) 
		StepperMotorClockRate = (ratio / LastSpindleClocks);
#else
*/
	// If not use the average of the spindle speed over the last 10 seconds or so.
	if (AveragedClocksPerRev != 0) 
		StepperMotorClockRate = (ratio / AveragedClocksPerRev);
//#endif
	return(StepperMotorClockRate);
}

/*
 *  FUNCTION: CalculateMotorDistance
 *
 *  PARAMETERS:
 *		dist 		-- Distance to convert in inches or mm.
 * 		divisor		-- Conversion divisor to convert distance to steps.
 *		units 		-- 0 == inches, 1 == mm.
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:
 *		Calculates and returns the number of steps based on distance in inches or mm.
 *  RETURNS: 
 *		int32 		-- positive number of steps to move a given distance 
 */
int32
CalculateMotorDistance( float32 dist, float32 divisor, int8 units ) {
  int32 scnt;
	if (units) 	// If set it means the distance is in mm and we want imperial inches for this.
		dist /= 25.4;	// It's either that or set up the divisor based on mm or inch.
	scnt = ((dist * divisor) + 0.5);
	if (scnt<0) scnt = 0-scnt;
	return(scnt);
}
/*
 *  FUNCTION: CalcuateSpindleSpeed
 *
 *  PARAMETERS:		iTrkRatio
 *
 *  USES GLOBALS:	PULSE_CLOCK_RATE
 *
 *  DESCRIPTION:	Helper function for calculating a spindle speed based on a 
 *					step rate about 5% under maximum stepper speed to avoid running 
 *					too close to the edge if spindle runs too fast.
 *
 *  RETURNS: 		Recomended RPM for a given stepper speed.
 *
 */
int32 
CalcuateSpindleSpeed(uint32 iTrkRatio) {
	return( (PULSE_CLOCK_RATE * 60L) / (iTrkRatio/(PULSE_CLOCK_RATE-1000)) );
}

/*
 *  FUNCTION: MotorMoveDistance
 *
 *  PARAMETERS:	device 		-- MOTOR_X or MOTOR_Z
 *				distance 	-- Distance in steps
 *				speed       -- Motor Speed in Hz up PULSE_CLOCK_RATE
 *				dir 		-- Which direction to turn
 *				SpindleON 	-- Spindle needs to be turning if true
 *				useLimit	-- Make use of Limit switch or not.
 *
 *  USES GLOBALS:
 *				MotorState	-- for type of motion.
 *				Sets ZAcc	-- ACCEL_RATE_Z_NDX
 *
 *  DESCRIPTION:
 *
 *  RETURNS: 	0 if all is well, Non Zero error #'s
 *					MSG_SPINDLE_TO_FAST_ERROR
 *					MSG_MOTOR_STOPPED_ERROR
 *
 */
int8
MotorMoveDistance( int8 device, 	// Motor X or Z
				   int32 distance, 	// Distance in steps
				   uint16 speed, 	// Speed in Hz up PULSE_POINT, 0 if we track spindle.
				   uint8 dir, 		// Which way to turn
				   int8 SpindleON, 	// SPINDLE_TURNING or SPINDLE_EITHER
				   int8 useLimit	// Make use of limit switch in distance move.
				   ) {
  float32 res;
  int16 rpm;
  int32 vel;
  int32 halfway;

	switch (device) {
	  /*
		Set up variables and data structures and interrupt routine to do a programmed
		move for Z Axis Motor.
	  */
	  case MOTOR_Z :
		fZDirectionCmd = dir;	// MOVE_LEFT is 0, MOVE_RIGHT is 1.
		// Get acceleration from EEROM since it may have changed.		
		ZAcc = GetGlobalVarLong(ACCEL_RATE_Z_NDX) << 5; 
		
		// Check if we are supposed to turn a specific speed relative to the spindle RPM.	
		// Use MOVE rate if spindle stopped.
		if (speed == SPEED_TRACK_SPINDLE) {	// speed == 0 if we track spindle.
			// Figure out how fast the lead screw should turn.
			LeadScrewRatio = SetupThreadDivision(MotionPitchIndex, &iTrackingRatio);  // Get number of stepper steps per spindle revolution.
			speed = SetupMotorSpeed(iTrackingRatio);	// Get lead screw speed based on spindle RPM. 
		}
	
		vel = (int32)speed << 16;
		halfway = distance / 2;
		rpm = PrintRPM(1,0);			// Show console serial output if Debug enabled, RPM, not SFM	
		if (speed > PULSE_CLOCK_RATE) {
			// Calculate target RPM based on close to but not quite top Stepper Motor Speed.
			// TargetRPM is used in the ERROR screen showing current RPM and what it should be.
			TargetRPM = (int16)CalcuateSpindleSpeed(iTrackingRatio);
			DEBUGSTR(", MOVEZ:Stepper speed %u too high. Slow spindle to less than %d.\n", speed, TargetRPM);
			return(MSG_SPINDLE_TO_FAST_ERROR);
		}
		else { 
			DEBUGSTR(", MOVEZ: Step Speed=%ld pps, Accel=%ld, ", vel>>16, ZAcc >> 5);
			DEBUGSTR("Moving Z %ld steps to ", distance);
			if (!dir) 
				DEBUGSTR("left\n");
			else
				DEBUGSTR("right\n");
			// Request move for stepcount steps which has already been set up.
			if (SpindleON == SPINDLE_TURNING) {
				
				if (rpm > 0) {
#ifdef TRACK_SPINDLE_SPEED
					// Spindle Speed tracking variables get set up.  They can be done outside the interrupt
					// routine because they aren't being used until fThreading is 1.
					SpinRate = LastSpindleClocks;	// Get spindle clocks value used to calculate stepper rate.
					SpinClip = SpinRate >> 1;		// Calculate ceiling to prevent overruns.
					NegSpinRate = 0-SpinRate;		// Make the negative for faster calculation in interrupt routine.
#endif
					INTCON &= 0x3F;
					fZMoveRQ = 1;  				// On Spindle Index Interrupt the move will start.
					NewZVel = vel;				// Want to go this fast
					ZStepCount = distance;		// and this far.
					ZHalfwayPoint = halfway;	// Halfway point if we never reach max speed decelerate here.
					fUseLimits = useLimit;
					fThreading = 1;				// Track spindle speed.
					INTCON |= 0xC0;
					
					DEBUGSTR("Waiting for Index Pulse\n");
				}
				else {
					DEBUGSTR("No Motion -- Spindle Not Turning.\n");
					return(MSG_MOTOR_STOPPED_ERROR);
				}
					
			}
			else {
				// Modify the flags and data used by the interrupt routine.
				INTCON &= 0x3F;				// Done with interrupts off.
				fUseLimits = useLimit;
				if (!fZMoveBSY) {
					fZDeccel = 0;			// Not decelerating yet.
					StepsToZVel = 0;		// Keep track of how int32 it takes to get to velocity.
					fZUpToSpeed = 0;		// Motor isn't up to speed yet.
					fZStopping = 0;			// Nor is it stopping.
					MaxZVel = vel;			// Want to go this fast.
					ZStepCount = distance;	// and this far.
					fZMoveBSY = 1;			// Manual Non-sync'd move so we set that the motor is busy.
					fZAxisActive = 1;		// Enable Z axis interrupt handling.
				}
				else {
					if (fZDeccel) {		
						fZDeccel = 0;			// Not decelerating anymore
						fZUpToSpeed = 0;		// Motor isn't up to speed again
						MaxZVel = vel;			// Want to go this fast since we're not stopping.
					}
					ZStepCount += distance;	// Motor is already turning so just increase how far to go.
				}
				INTCON |= 0xC0;
			}
		}
		break;

	  case MOTOR_X :
#ifdef X_AXIS
		// Get acceleration from EEROM since it may have changed.		
		XAcc = GetGlobalVarLong(ACCEL_RATE_X_NDX) << 5;
		
		NewXVel = (int32)speed << 16;
#ifdef DIRECT_MODE_ENABLED
		distance = (fMXDirectMode) ? distance<<1 : distance;
#endif
		if ((fMXInvertDirMotor ^ dir) != fXDirection) {
			res = GetGlobalVarFloat(X_AXIS_BACKLASH_NDX);
			//Changed by RE Rev 1.10f
			if (fMetricMode)
				res = res * 25.4;
			//End of change
			XBackLashCount = CalculateMotorDistance(res, XDistanceDivisor, fMetricMode);
			distance += XBackLashCount;
		}

	    // Set Motor Direction based on flag which tells us what polarity a minus direction is.
		fXDirection = fMXInvertDirMotor ^ dir;

		if (speed > PULSE_CLOCK_RATE) {
			DEBUGSTR("MOVEX:Stepper speed %ld  too high.\n", NewXVel>>16);
			return(MSG_SPINDLE_TO_FAST_ERROR);
		}
		else { 
			DEBUGSTR("MOVEX: Step Speed=%ld pps, ", NewXVel>>16);
			DEBUGSTR("Moving X %ld steps ", distance);
			if (!dir) 
				DEBUGSTR("in");
			else
				DEBUGSTR("out");

			INTCON &= 0x3F;
			fUseLimits = useLimit;
			if (!fXMoveBSY) {
				fXDeccel = 0;			// Not decelerating.
				StepsToXVel = 0;		// Keep track of how int32 it to get to velocity.
				fXUpToSpeed = 0;		// Not up to speed yet.
				fXStopping = 0;			// So not stopping yet.
				MaxXVel = NewXVel;		// This is how fast to go
				XStepCount = distance; 	// and this is how far.
				fXMoveBSY = 1;			// It's a programmed distance so flag we're busy.
				fXAxisActive = 1;		// Enable X axis Pulse Clock interrupt handling.
			}
			else {
				XStepCount += distance; 
			}
			INTCON |= 0xC0;
		}
#endif
		break;
	}
	return(0);
}

/*
 *  FUNCTION: MotorMoveTo
 *
 *  PARAMETERS: device		-- Which Motor: Motor X or Z
 *		   		position	-- Where to in steps
 *		   		speed		-- Speed in Hz up PULSE_POINT
 *		   		SpindleON	-- Spindle needs to be turning if true
 *
 *  USES GLOBALS:		 
 *
 *  DESCRIPTION: Start a move to a specific location by turning the location into
 *				 a distance.  
 *
 *  RETURNS: Result of MotorMoveDistance
 *
 */
int8
MotorMoveTo( int8 device, 	// Motor X or Z
		   int32 position, 	// Where to in seps
		   uint16 speed, 	// Speed in Hz up PULSE_POINT
		   int8 SpindleON 	// Spindle needs to be turning if true
		   ) {
  uint8 dir;
  int32 distance;
  

	switch (device) {

	  case MOTOR_Z :

		INTCON &= 0x3F;
			CurrentZPosition = ZMotorPosition;
		INTCON |= 0xC0;
		if (position == CurrentZPosition) {
			return(0);
		}
		else if (position < CurrentZPosition) { // Move closer to Headstock
			dir = MOVE_LEFT;
		}
		else { // Move away from Headstock.
			dir = MOVE_RIGHT;
		}

		distance = labs(position - CurrentZPosition);
		DEBUGSTR("ZDist=%ld\n",distance);
		return (MotorMoveDistance( device, distance, speed, dir, SpindleON, /*track,*/ TRUE ));
		break;

	  case MOTOR_X :
#ifdef X_AXIS
		// Get value modified inside 
		INTCON &= 0x3F;
			CurrentXPosition = XMotorRelPosition;
		INTCON |= 0xC0;

		if (position == CurrentXPosition) {
			return(0);
		}
		else if (position < CurrentXPosition) { // Move closer to Lathe centre line
			dir = MOVE_IN;
		}
		else { 									// move away from Lathe centre line.
			dir = MOVE_OUT;
		}

		distance = labs(position - CurrentXPosition);
		// Test value for 4" displacement on cross slide.
		if (distance > 192000) {	// 2400 steps per rev, 4" distance max on 20TPI screw.
			// Must make this a system based constant.
			DEBUGSTR("X Distance is %ld\n",distance);
			return(MSG_MOTOR_DISTANCE_ERROR);		
		}

		return(MotorMoveDistance( device, distance, speed, dir, SpindleON, /*track,*/ TRUE ));
#endif
		break;
	}
}


/*
 *  FUNCTION: MotorJog
 *
 *  PARAMETERS:		device 		-- Which motor
 *		  			speed		-- How fast to jog, 0 means Z axis track spindle at Turning rate.
 *		  			dir			-- Which direction to turn.
 *
 *  USES GLOBALS: 	TURN_PITCH_NDX
  *					MOVE_RATE_Z_NDX
 *					iTrackingRatio
 *
 *  DESCRIPTION:
 *					Set up variables and data structures and interrupt routine to do continuous
 *					move (jog) for X or Z axis.
 *
 *  RETURNS: 
 *					speed of jog (interesting if speed was 0).
 */
uint16
MotorJog( int8 device,	  	// Which motor
		  uint16 speed, 	// How fast to jog, 0 means track spindle at Turning rate.
		  uint8 dir			// Which direction to turn.
		) {

  uint16 currVel;
  uint8 deccelFlag;
  int32 trackingRatio;
  int32 vel;


	switch (device) {
	  /*
		Set up variables and data structures and interrupt routine to do a programmed
		move for Z Axis Motor.
	  */
	  case MOTOR_Z :

		if (speed == (uint16)0) {		// Means we want to track at turning speed relative to spindle
			LeadScrewRatio = SetupThreadDivision(TURN_PITCH_NDX, &iTrackingRatio);  // Get number of steps per spindle revolution.
			// Now go get spindle speed and determine stepper rate for Z distance per spindle rev.
			speed = SetupMotorSpeed(iTrackingRatio);
			if (speed == (uint16)0) {	// Spindle not turning?  Default to move speed.
				speed = (uint16)GetGlobalVarWord(MOVE_RATE_Z_NDX);
			}
		}
	
		MotorState = MOTOR_JOGGING;

		INTCON &= 0x3F;
			vel = MaxZVel;		// Copy 32 bit value from interrupt routine.
		INTCON |= 0xC0;	// go.

		currVel = vel >> 16;	// Make into a word so test below is only on 16 bit variables.
								// And we'll print the 16 bit value later so we only do this once..
		if (speed < currVel)
			deccelFlag = 1;
		else 
			deccelFlag = 0;
	
		vel = speed;
		vel = vel << 16;		// Now make a 32 bit variable of target speed.

		INTCON &= 0x3F;
			if (deccelFlag) {
				fZDeccel = 1;
			}
			else {
				fZDeccel = 0;
				StepsToZVel = 0;		// Keep track of how int32 it to get to velocity.
				fZUpToSpeed = 0;
				fZStopping = 0;
			}			
			NewZVel = vel;				// Make a 32 bit copy for others.
			MaxZVel = NewZVel;			// Transfer 32 bit velocity value to interrupt routine.

			// Still need to tell Interrupt routine that direction has changed and motor needs to 
			// decelerate to 0 first and then accelerate but for now we'll do an abrupt reversal if 
			// requested.
			fZDirectionCmd = dir;
			fZAxisActive = 1;			// Enable Z axis Pulse Clock interrupt handling 
			
		INTCON |= 0xC0;	// go.

		DEBUGSTR("JOGZ: Stepper Motor Speed is %d pps\n", speed);
		break;

	  case MOTOR_X :

		MotorState = MOTOR_JOGGING;
		INTCON &= 0x3F;
			vel = MaxXVel;		// Copy 32 bit value from interrupt routine.
		INTCON |= 0xC0;	// go.

		currVel = vel >> 16;	// Make into a word so test below is only on 16 bit variables.
								// And we'll print the 16 bit value later so we only do this once..
		if (speed < currVel)
			deccelFlag = 1;
		else 
			deccelFlag = 0;
	
		vel = speed;
		vel = vel << 16;
	
		INTCON &= 0x3F;
			if (deccelFlag) {
				fXDeccel = 1;
			}
			else {
				fXDeccel = 0;
				StepsToXVel = 0;		// Keep track of how int32 it to get to velocity.
				fXUpToSpeed = 0;
				fXStopping = 0;
			}
			NewXVel = vel;				// Make a 32 bit copy for others.
			MaxXVel = NewXVel;
			// Still need to tell Interrupt routine that direction has changed and motor needs to 
			// decelerate to 0 first and then accelerate but for now we'll do an abrupt reversal if 
			// requested.
			fXDirection = fMXInvertDirMotor ^ dir;
			fXAxisActive = 1;			// Enable X axis pulse clock interrupt handling.
		INTCON |= 0xC0;	// go.
		DEBUGSTR("JOGX: Stepper Motor Speed is %ld pps\n", NewXVel>>16);
		break;
	}
	return(speed);
}


/*
 *  FUNCTION: MotorStop
 *
 *  PARAMETERS:			device 	-- Which motor to stop.
 *
 *  USES GLOBALS:		MotorState		Control state machine.
 *						-- Z Axis
 *						fZDeccel		Tell Z Motor to decelerate
 *						MaxZVel			Velocity gets set to 0
 *						fZMoveBSY		Cancel move.
 *						fZMoveRQ		
 *						-- X Axis
 *						fXDeccel		Tell Z Motor to decelerate
 *						MaxXVel			Velocity gets set to 0
 *						fXMoveBSY		Cancel Move.
 *						fXMoveRQ		
 *
 *  DESCRIPTION:		Ask Interrupt routine to stop the motor.
 *
 *  RETURNS: 			Nothing.
 *
 */
void
MotorStop(int8 device) {

	switch (device) {

	  case MOTOR_Z :
		MotorState = MOTOR_STOPPED;
		INTCON &= 0x3F;
		fZDeccel = 1;		// Tell interrupt routine to decelerate the motor.
		MaxZVel = 0;		// to speed 0.
		fZMoveBSY = 0;		// Cancenl any current moves.
		fZMoveRQ = 0;		// Ack the requests.
		INTCON |= 0xC0;		// go.
		break;

	  case MOTOR_X :
		MotorState = MOTOR_STOPPED;
		INTCON &= 0x3F;
		fXDeccel = 1;
		MaxXVel = 0;
		fXMoveBSY = 0;
		fXMoveRQ = 0;
		INTCON |= 0xC0;	// go.
		break;
	}
}

/*
 *  FUNCTION: MotorDevice
 *
 *  PARAMETERS:			None
 *
 *  USES GLOBALS:		MotorState
 *						RPMState			
 *						fSpindleTurning 
 *						fSpindleInterrupt
 *						SpindleClocksPerRevolution
 *						AveragedClocksPerRev
 *						fUpdatedRPM
 *						AverageRPM
 *						RPMAverage	   				-- Array
 *						RPMAverageIndex				-- Index into array
 *
 *  DESCRIPTION:		Motor device used to track spindle speed and adjust Z axis stepper.
 *						However, loop time is too long between new RPM value, calculations
 *						and updating stepper speed so the concept has been scrapped.
 *						Instead the MotorDevice monitors spindle RPM, averages the value
 *						and updates AverageRPM for display and for calculating initial 
 *						stepper speed.
 *
 *  RETURNS: 			Nothing
 *
 */
void
MotorDevice(void) {
  int8 i;				// Loop counter
  int8 deccelFlag;
  static int32 PulseClocksRunningTotal = 0;
  static int8 cycleCount = 0;
  static int8 averageSampleCount = 0;

	switch (RPMState) {
	  /*
		Sit here waiting for a spindle interrupt.
	  */
	  case RPM_ZERO :
		INTCON &= 0x3F;							// Interrupt touches these variables.
		fSpindleTurning = 0;					// Spindle not really turning yet.
		if (fSpindleInterrupt) {				// Spindle turning.
			fSpindleInterrupt = 0;				// Clear semaphore.
			SpindleClocksPerRevolution = 0;		// Trash first value.
			fUpdatedRPM = 0;					// RPM not valid yet.
			StartTimer(MOTOR_TIMER, T_2_5SEC); 	// Spindle turning timeout timer.
			RPMState = RPM_STARTING;			// Need a full turn for RPM so nothing else is done.
		}
		INTCON |= 0xC0;							// Done touching common variables.
		break;

	  /*
		One full turn has occurred.
	  */	
	  case RPM_STARTING :
		INTCON &= 0x3F;							// Interrupt touches these variables.
		if (fSpindleInterrupt) {				// Spindle turning.
			fSpindleInterrupt = 0;				// Clear semaphore.
			LastSpindleClocks = SpindleClocksPerRevolution;	// Grab time for one rev.
			fUpdatedRPM = 0;					// Flag that we've got it.
			INTCON |= 0xC0;						// Done touching common variables.

			if (LastSpindleClocks > 0) {
				PulseClocksRunningTotal = 0;
				for (RPMAverageIndex=0; RPMAverageIndex<16; RPMAverageIndex++) {
					RPMAverage[RPMAverageIndex]	= LastSpindleClocks;	// Make all filter boxes the same to start with.
					PulseClocksRunningTotal += LastSpindleClocks;
				}
				RPMAverageIndex=0;
				AveragedClocksPerRev = PulseClocksRunningTotal / 16;				
				AverageRPM = (int16)((PULSE_CLOCK_RATE * 60L) / (int32)AveragedClocksPerRev);
				RPMState = RPM_STEADY;
			}
			else {
				AverageRPM = 0;
				RPMState = RPM_SLOWING;
			}
			StartTimer(MOTOR_TIMER, T_2_5SEC); 	// Spindle turning timeout timer.
		}
		else
			INTCON |= 0xC0;						// Done touching common variables.
		if (TimerDone(MOTOR_TIMER)) {			// Spindle isn't turning anymore.
			RPMState = RPM_SLOWING;
		}	
		break;

	  /*
	   	Accumulate spindle period averaging in RPMAverage[]
	  */	
	  case RPM_STEADY :
		INTCON &= 0x3F;							// Interrupt touches these variables.
		if (fSpindleInterrupt) {				// Spindle turning.
			fSpindleInterrupt = 0;				// Clear semaphore.
			LastSpindleClocks = SpindleClocksPerRevolution;	// Grab time for one rev.
			fUpdatedRPM = 0;					// Flag that we've got it.
			INTCON |= 0xC0;						// Done touching common variables.
			
			PulseClocksRunningTotal -= RPMAverage[RPMAverageIndex];	 	// Subtract oldest value.
			RPMAverage[RPMAverageIndex++] = LastSpindleClocks;			// replace oldest value. 	
			RPMAverageIndex &= 0xf;										// Wrap index around circular buffer.
			PulseClocksRunningTotal += LastSpindleClocks;			 	// Add in newest value.
			AveragedClocksPerRev = PulseClocksRunningTotal >> 4;
			if (AveragedClocksPerRev > 0) {
				AverageRPM = (int16)((PULSE_CLOCK_RATE * 60L) / (int32)AveragedClocksPerRev);
				RPMState = RPM_STEADY;
			}
			else {
				AverageRPM = 0;
				RPMState = RPM_SLOWING;
			}
			StartTimer(MOTOR_TIMER, T_2_5SEC); 	// Spindle turning timeout timer.
#ifdef DEBUG_SPEED_BUCKETS
			// Debug 16 buckets and calculated RPM.
			for (i=0;i<16;)
				printf((far rom int8 *)"%ld, ",RPMAverage[i++]);
			printf((far rom int8 *)"%d, %ld\n",AverageRPM, NewZVel>>16);
#endif
		}
		else
			INTCON |= 0xC0;							// Disable just in case there wasn't a spindle interrupt.
		if (TimerDone(MOTOR_TIMER)) {			// Spindle isn't turning anymore.
			RPMState = RPM_SLOWING;
		}	
		break;

	  /*
		Timer expired so clear out RPM accumulation.
	  */	
	  case RPM_SLOWING :
		AveragedClocksPerRev = 0;
		AverageRPM = 0;
		LastSpindleClocks = 0;		// 12OCT08 -- Fixes slow move rate after spindles stops. 
											//            This was a side effect of spindle tracking.
		RPMState = RPM_ZERO;
		INTCONbits.INT0E = 1;  // Allow a spindle interrupt to start things up again.
		break;
	}
}
