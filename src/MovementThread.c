// Movement Thread -- using G-Code G76 simulation instead of constant area removal
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
	1.02m -- AutoInitialized PassCount and SpringPassCount to 0
	1.10h -- See Config.h for description.


*/

#include "processor.h"

#include <Ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <float.h>
#include <math.h>

#include "common.h"
#include "config.h"

#include "int.h"
#include "menu.h"
#include "globvars.h"
#include "floatio.h"
#include "serial.h"			// Need output buffer for diagnostics.

#include "Timer.h"
#include "MotorDriver.h"    // Motor Driver Defintions.

#include "MovementThread.h" // Motor Thread Defintions.

/* ------------  Special Documention -------------*/

/*
G76 Simulation has arguments:								Global Variable array indices
    X   -- X End position.                                      END_X_NDX
    Z   -- Z End position                                       THREAD_END_NDX
    Q   -- Spring Pass Count                                    SPRING_PASS_CNT_NDX
    P   -- Pitch                                                THREAD_SIZE_NDX
    H   -- First Pass cutting depth                             FIRST_X_PASS_DEPTH_NDX
    I   -- In feed thread angle.                                THREAD_ANGLE_NDX
    R   -- X Start                                              BEGIN_X_NDX
    K   -- Z Start position with room for synchronization       THREAD_BEGIN_NDX
    L   -- Thread chamfor... not used in ELS.
    C   -- X retracted position                                 RETRACTED_X_NDX
    B   -- Last Pass cutting depth                              END_X_PASS_DEPTH_NDX
    T   -- Taper angle.  Not used directly.  System uses " per foot.  TAPER_NDX
    J   -- Cutting pass depth                                   X_PASS_DEPTH_NDX

    Variables used for calculating new position of X,Z after each threading pass.
	When the system is READY (Green LED Flashes) and the FNC3 key is pressed
	the menu shows:
		  PASS COUNT = nn
        CNCL  RST   SET  CLR

 	When RST is pressed a recalculation is performed and Pass Count is updated.
    This way an operator can start and stop the threading but has to explicitely
    change to a new set of the threading operations.
	When the SET softkey is pressed the display is changed to acitivate the
	THREAD PARAMETERS Menu shown below.

   THREAD PARAMETERS
  TURN THRD  POS  JOG
        |
      PITCH ANGLE PASS SPRING
      FLOAT FLOAT   |    Spring Pass Count
                    |    uint8 n
                    |
                  THREAD Pass Param
                  DEPTH FRST EACH LAST
                    |    |     |    |
                    |    |     |   Last Path Depth
                    |    |     |   FLOAT 0.dddd
                    |    |     |
                    |    |    Each Pass Depth
                    |    |    FLOAT  0.dddd
                    |    |
                    |   First Pass Depth
                    |   FLOAT  0.dddd
                    |
                   Depth of Thread
                   FLOAT   d.dddd
*/




/* ------------  Private Declarations and Data -------------*/

// Enable MOVE_DIAGNOSTICS in order to have printf statements for debugging.
// The other diagnostic flags are in config.h
//#define MOVE_DIAGNOSTICS	1

#ifdef MOVE_DIAGNOSTICS
#define M_DEBUGSTR(s) printf((far rom int8 *)s)
#else
#define M_DEBUGSTR(s)
#endif

enum PASS_STATES PassState = CALCULATE_PASSES;

static float32 TAN_Angle, Adjust_pass, PassDepth, EachPassDepth;
float32 PassDistance;

/* ------------  Public Data -------------*/
enum MOVEMENT_STATES MovementState = MOVE_WAIT;

int32 ZBeginPositionSteps, ZEndPositionSteps;
int32 ZIncrementDistanceSteps;
float32 ZBeginPosition, ZEndPosition;

int32 	XRetractedPositionSteps, XBeginPositionSteps, XEndPositionSteps;
int32 	XPassSteps;    // Amount to move X per pas
float32 XRetractedPosition, XBeginPosition, XEndPosition;
float32 AdjustPass, RunTimeAdjust, RunTimeLast; //Added RE
int8 	PassCount = 0;
int8 	SpringPassCount = 0;
int8 	AdjustPasses = 0;	//Added RE
int16 	EachPasses = 0;	//Added RE


BITS MovementRunFlags;

/* ------------  Private Functions and Declarations -------------*/

/*
 *  FUNCTION: SetCalculatePositionState
 *
 *  PARAMETERS:		state	-- State to change to
 *
 *  USES GLOBALS:	passState
 *
 *  DESCRIPTION:	Wrapper function to change state of private variable.
 *
 *  RETURNS: 		Nothing.
 *
 */
void
SetCalculatePositionState( enum PASS_STATES state ){
	PassState = state;
}

/*
 *  FUNCTION: SetCalculatePositionState
 *
 *  PARAMETERS:		None.
 *
 *  USES GLOBALS:	
        ZBeginPositionSteps,
        ZEndPositionSteps,
        XRetractedPositionSteps,
        XBeginPositionSteps
 *
 *  DESCRIPTION:	Calculates and returns XBeginPositionSteps based on whether we're tapering
					or not.  If tapering then the begin position is dependant on a specific Z position 
					which is dependant on the taper angle.
 *
 *  RETURNS: 		XBeginPositionSteps
 *
 */
long 
GetXBeginPositionSteps(void) {
  float steps, temp;
	if (fTapering) {// Need to protect ZMotorPosition since it's changed by interrupts but, we're not really moving here either.
		steps = (float)(ZBeginPositionSteps - ZMotorPosition);
		if (steps < 0.0 ) steps = -steps;
		temp = GetGlobalVarFloat(TAPER_NDX);
		if (temp < 0.0) {
			temp = 0-temp;	// Make positive.
		}
		// The taper is stored in diameter per 1" distance, we're doing radius movements.
		temp /= 2.0;  // So we move half the X distance to get the radius.	
		steps *= temp;	// Now we have the proper X position based on X BEGIN = 0.000
		// But really, this could be non-zero so we'd better get it.
	    XBeginPosition = (fMetricMode) ? GetGlobalVarFloat(BEGIN_X_NDX) * 25.4 : GetGlobalVarFloat(BEGIN_X_NDX);
		// And then add in the BEGIN POSITION parameter.
    	XBeginPositionSteps = XDistanceToSteps(XBeginPosition) + steps;
	}
	return(XBeginPositionSteps);
}

long 
GetXRetractedPositionSteps(void) {
}

/*
 *  FUNCTION: CalculateCompoundPosition
 *
 *  PARAMETERS:		None
 *
 *  USES GLOBALS:	MOTOR_STEPS_REV_X_NDX
 *					CROSS_SLIDE_IPITCH_NDX
 *					fAutoX
 * 					PassDepth
 *					PassDistance
 *					fExternalThreading
 *
 *  DESCRIPTION:	On each pass,  the tool bit is moved in along the hypotenuse of the triangle
 *					based on the distance desired and the the thread angle parameter.  The X axis
 *					is moved a specific distance inwards (ajacent) while the Z moves the tool
 * 					left (opposite).
 *
 *  RETURNS: 		Nothing
 *					XBeginPositionSteps		-- Sets with new start position.
 *
 */
void
CalculateCompoundPosition(void) {
  float32 res1;
  int32 stepsX;
  int8 OutputBuffer[12];
	// Nice little side effect here.
	// If Use compound is set ON and also AutoX is set on, we can do the multiple passes
	// returning to the same Z and just moving the X axis in.
	// Handy for turning down a specific distance.
	// The trick is to set the TPI or pitch to very small just like a turning pitch.

	if (!fAutoX)
		res1 = sqrt((PassDepth*PassDepth) + (PassDistance * PassDistance));	// Calc. Hypotenuse.
	else
		res1 = PassDepth;													// Use PassDepth for turning.

#ifdef MOVE_DIAGNOSTICS
    floatToAscii(res1, &OutputBuffer[0],7,4); OutputBuffer[8] = '/0';
#endif
    M_DEBUGSTR("Compound move is=%s\n",OutputBuffer);
	// Now fake out X axis motion so display of X is really compound distance on LCD.
    stepsX = ((res1 *
	        ((float32)GetGlobalVarLong(MOTOR_STEPS_REV_X_NDX) / GetGlobalVarFloat(CROSS_SLIDE_IPITCH_NDX)))+0.5);
    if (fExternalThreading)     // X moves inwards into cut.
        XBeginPositionSteps = XDistanceToSteps(XBeginPosition) - stepsX;
    else                        // X moves outwards into cut.
        XBeginPositionSteps = stepsX + XDistanceToSteps(XBeginPosition);
}


/* ------------  Public Functions -------------*/

/*
    CalculatePosition :
        Determines a new X and Z start position based on the threading parameters and
        argument as to which direction to change the parameters.
	There are three custom passes.
		'First', 'Each + fraction', 'End'.
		Before the 'End' distance are n 'Each' passes.
		The number of threading passes is determined by subtracting First and End
		from the Depth.  That result is divided by Each and the remainder added to
		Each to create Adjust pass .  The numerator of that division minus 1 is the Each pass
		count iff > 0.
		To cut a thread, there are at least the three custom passes.
		For each pass the Z distance is pass * tan(THREAD_ANGLE)
		CalculatePosition() uses a state machine to set up each calculation pass.

*/


/*
Following modified by RE 23/12/08
Further modifed 24/12/08

 *  FUNCTION: CalculatePasses
 *
 *  PARAMETERS:
 *
 *  USES GLOBALS:	THREAD_ANGLE_NDX,
 *					PASS_FIRST_X_DEPTH_NDX
 *					PASS_EACH_X_DEPTH_NDX
 *					PASS_END_X_DEPTH_NDX
 *					PASS_SPRING_CNT_NDX
 *
 *  DESCRIPTION:	Sets up the values needed for the CalculatePosition State machine when doing
 *					multiple passes.
 *
 *  RETURNS: 		Nothing
 *					PassCount //Total number of passes to do
 *					SpringPassCount //Number of spring passes to do
 *					Adjust_pass //The value of the Adjust pass
 *					TAN_Angle
 *								//Plus the value of the First pass in "Pass Depth"
 */
void
CalculatePasses(void) {
  float32 totalEachPassDepth, numberOfPasses, fractionOfAPass, AdjustPass ;
  int16 passes ;		//Additional External integers (EachPasses, AdjustPasses) added earlier and in MovementThread.h
	TAN_Angle = GetGlobalVarFloat(THREAD_ANGLE_NDX);
	TAN_Angle = tan(TAN_Angle * RADIANS_PER_DEGREE );
	// Set up first pass.
	PassCount 		= 0;	// No passes to start with.
	EachPasses 		= 0; 	// Initialise Each Passes
	AdjustPasses 	= 0;	// Initialise Adjust Passes
	PassDepth 		= 0.0;	// Depth of pass is also zero.
	PassDistance 	= 0.0;	// As is the distance to travel in Z
	Adjust_pass 	= 0.0;	// No adjustment yet.
	RunTimeAdjust	= 0.0;
	RunTimeLast		= 0.0;
	M_DEBUGSTR("PassCount=%d, ", PassCount);

	// Test if operator has entered in a first pass value.
	if (GetGlobalVarFloat(PASS_FIRST_X_DEPTH_NDX) > 0.0) {
		PassCount++;			// So we have at least one pass.
		M_DEBUGSTR("PassCount=%d, ", PassCount);
		// Set depth to this and then figure out how far Z moves.
		PassDepth = GetGlobalVarFloat(PASS_FIRST_X_DEPTH_NDX);
		PassDistance = PassDepth * TAN_Angle;
	}
	// Test for special case where if First Pass is bigger than Totaldepth we don't do each or last passes.
	if ( GetGlobalVarFloat(PASS_FIRST_X_DEPTH_NDX) < GetGlobalVarFloat(DEPTH_X_NDX)) {
		// See if user has a last pass value.
		if (GetGlobalVarFloat(PASS_END_X_DEPTH_NDX) > 0.0) {
			RunTimeLast = GetGlobalVarFloat(PASS_END_X_DEPTH_NDX);
			// So we have at least one more pass.
			PassCount++;			// Yes, so add another pass.
			M_DEBUGSTR("PassCount=%d, ", PassCount);
		}
	// Now we have 2 passes (First and End) so do tests for the intermediate passes
		if ( GetGlobalVarFloat(PASS_EACH_X_DEPTH_NDX) > 0.0) {
			// Get depth minus custom passes
			totalEachPassDepth = GetGlobalVarFloat(DEPTH_X_NDX) - 
								 GetGlobalVarFloat(PASS_FIRST_X_DEPTH_NDX) - 
								 GetGlobalVarFloat(PASS_END_X_DEPTH_NDX);
			// Check if total depth is greater than first pass depth + final pass depth.
			// If true then we need to do Each passes and maybe an adjust pass
			if (totalEachPassDepth > 0.0)	{
				numberOfPasses = totalEachPassDepth / GetGlobalVarFloat(PASS_EACH_X_DEPTH_NDX);
				// Has Whole and possible fractional part
				passes = numberOfPasses;	// make an integer truncating the fractional part.
				fractionOfAPass = numberOfPasses - passes;	//Float - Integer = fraction
				EachPasses = passes; //Could be zero or greater
				// Then determine how much of the fractional part that doesn't fit into an integral distance.
				Adjust_pass = fractionOfAPass * GetGlobalVarFloat(PASS_EACH_X_DEPTH_NDX);
				RunTimeAdjust = Adjust_pass;
				// Test should really be for > zero but this covers for miniscule float division errors?
				if (Adjust_pass > 0.0001) {	
					AdjustPasses = 1;
					// Probably getting too clever here but it would be nice if the Last pass 
					// was always smaller than the previous Adjust pass
					if ( GetGlobalVarFloat(PASS_END_X_DEPTH_NDX) > Adjust_pass) {
						RunTimeAdjust = GetGlobalVarFloat(PASS_END_X_DEPTH_NDX);
						RunTimeLast	  = Adjust_pass;
					}
				}
			}
				// Now we have the exact value for an adjust passs so that the EACH passes plus the adjust pass plus LAST pass add up to the total depth.
			PassCount += (AdjustPasses + EachPasses);
		}
	}
	SpringPassCount = GetGlobalVarByte(PASS_SPRING_CNT_NDX);
	M_DEBUGSTR("PassCount=%d, SpringPassCount=%d\n", PassCount, SpringPassCount);
}
/*
Following modified by RE 23/12/08 and 24/12/08

I feel that there is a terminology problem here
The "Original" CalculatePosition function appears to actually run First Pass then Adjust Pass, then Each Passes, then Spring
passes. The first "Each" pass was the adjusted pass however it was termed "First Pass" which caused some confusion.
Assuming that the "First Pass" had a value it would fall through the switch and execute
Accordingly the PassStates have been changed slightly to give
FirstPass Not in switch as it is the exit value of CalculatePosition (usually falls straight through)
EachPass (Number as required)
AdjustPass (to nearly complete the required depth)
End pass (to complete the required depth)
Spring Passes (as required)

NOTE all debug information has been removed to make it clearer for me to read!
*/

/*
 *  FUNCTION: CalculatePosition
 *
 *  PARAMETERS:			holdPosition	--
 *
 *  USES GLOBALS:
 *						PassState
 *						PassCount
 *						SpringPassCount
 *						PassDepth
 *						Adjust_pass
 *						TAN_Angle
 *
 *  DESCRIPTION:		State machine to work through the G76 Passes for threading.
 *
 *  RETURNS: 			nothing
 *
 */
void
CalculatePosition(int8 holdPosition) {
  int32 stepsX, stepsZ;

	//Note the passes are decremented in MOVE_TO_END
switch (PassState) {
	  /*
	  	First work out first pass.
	  */
	case CALCULATE_PASSES :
		CalculatePasses();
		// PassDepth currently set to FirstPassDepth which will be added to XStart location.
		PassState = EACH_PASS;	// Next time run the Each pass.
		if (PassDepth != 0) {
			break; 				//Falls out of here on First Pass with depth set
		}
		else { // Oh Ohhh.  No First Pass so go do the Each pass right away.
			PassState = EACH_PASS;
		}
		// No break so we should fall through to Each pass.

//Stay here until the cutting passes are done.
	case EACH_PASS :
		if (EachPasses == 0) {  // Cutting passes done, go do Adjust pass.
			PassState = ADJUST_PASS;
		}
		else {	// Still cutting so do another pass.
			EachPasses --;
			PassDepth = PassDepth + GetGlobalVarFloat(PASS_EACH_X_DEPTH_NDX);
			break;
		}
//Adjustment pass should be less than "Each" pass but may not be needed
	case ADJUST_PASS :
		PassState = LAST_PASS; //Adjust pass will only be run once if at all
		if(AdjustPasses == 0) {
      	} 	// Fall through to Last pass
		else {
			PassDepth += RunTimeAdjust;
			break;
		}

	case LAST_PASS :
		PassState = SPRING_PASSES;
		if (RunTimeLast == 0.0)
			PassCount = 0;
			// Fall through to spring passes
		else {
			PassDepth += RunTimeLast;
			break;
		}

//Finally stay here while we're doing spring passes.

	case SPRING_PASSES :
		if (SpringPassCount == 0) {
			PassState = FINISHED_PASSES;
		}

	case FINISHED_PASSES :
		break;
	} // End of Switch

	// Parameters set up s now calculate distance to move.
	if (PassState != FINISHED_PASSES) {
		#ifdef MOVE_DIAGNOSTICS
	    floatToAscii(PassDepth, &OutputBuffer[0],7,4); OutputBuffer[8] = '/0';
	    M_DEBUGSTR("X=%s",OutputBuffer);
		#endif
	    // Calculate X distance in steps remembering to round rather than truncate.
	    stepsX = ((PassDepth *
	        ((float32)GetGlobalVarLong(MOTOR_STEPS_REV_X_NDX) / GetGlobalVarFloat(CROSS_SLIDE_IPITCH_NDX)))+0.5);

	    // Z = X Axis[Pass] x tan( Angle )
	    PassDistance = (PassDepth * TAN_Angle);
	    // Calculate Z distance in steps remembering to round rather than truncate.
	    stepsZ = ((PassDistance *
	         (GlobalVars[MOTOR_STEPS_REV_Z_NDX].f / GlobalVars[LEADSCREW_IPITCH_NDX].f)) + 0.5);

		#ifdef MOVE_DIAGNOSTICS
	    floatToAscii(PassDistance, &OutputBuffer[0],7,4); OutputBuffer[8] = '/0';
	    M_DEBUGSTR(", Z=%s\n",OutputBuffer);
		#endif
		if (!holdPosition) {	// Don't stay here, calculate new positions.
		    if (fExternalThreading)     // X moves inwards into cut.
		        XBeginPositionSteps = XDistanceToSteps(XBeginPosition) - stepsX;
		    else                        // X moves outwards into cut.
		        XBeginPositionSteps = stepsX + XDistanceToSteps(XBeginPosition);

		    if (fMovingRight)
		        ZBeginPositionSteps = stepsZ + ZDistanceToSteps(ZBeginPosition);
		    else
		        ZBeginPositionSteps = ZDistanceToSteps(ZBeginPosition) - stepsZ;

		    M_DEBUGSTR("ZS=%ld, ZE=%ld, XS=%ld, XR%ld\n",
		        ZBeginPositionSteps,
		        ZEndPositionSteps,
		        XBeginPositionSteps,
		        XRetractedPositionSteps
		    );
		}
		else {	// 01DEC08 -- Added so that when USE COMPOUND FOR THREADING IS ON and
				// Metric Mode is enabled, the ZBeginPosition is correct for SCREW Cutting.
			 ZBeginPositionSteps = ZDistanceToSteps(ZBeginPosition);
		}
	}
}

/*
 *  FUNCTION: InitMovementThread
 *
 *  PARAMETERS:		None
 *
 *  USES GLOBALS:	fMetricMode
 *					THREAD_BEGIN_NDX
 *					THREAD_END_NDX
 *					BEGIN_X_NDX
 *					RETRACTED_X_NDX
 *					X_DIAMETER_NDX
 * 					Changes Globals
 *   					MovementState
 *						PassState
 *   					fLOkToStop
 *   					fMTOkToStop
 *   					fMTRdy
 *   					fMTRun
 *   					fMovementThreadMode
 *						ZBeginPosition
 *						ZBeginPositionSteps
 *						XBeginPosition
 *						XBeginPositionSteps
 *
 *  DESCRIPTION:	Initialize all the globals used for Movement and Threading.
 *
 *  RETURNS: 		Nothing
 *
 */
void
InitMovementThread(void) {
    MovementState = MOVE_WAIT;
	PassState = CALCULATE_PASSES;
    fLOkToStop = 1;
    fMTOkToStop = 1;
    fMTRdy = 1;
    fMTRun = 0;
    fMovementThreadMode = 0;

    ZBeginPosition = (fMetricMode) ? GetGlobalVarFloat(THREAD_BEGIN_NDX) * 25.4 : GetGlobalVarFloat(THREAD_BEGIN_NDX);
    ZBeginPositionSteps = ZDistanceToSteps(ZBeginPosition);

    ZEndPosition = (fMetricMode) ? GetGlobalVarFloat(THREAD_END_NDX) * 25.4 : GetGlobalVarFloat(THREAD_END_NDX);
    ZEndPositionSteps = ZDistanceToSteps(ZEndPosition);

    XBeginPosition = (fMetricMode) ? GetGlobalVarFloat(BEGIN_X_NDX) * 25.4 : GetGlobalVarFloat(BEGIN_X_NDX);
    XBeginPositionSteps = XDistanceToSteps(XBeginPosition) ;

    XRetractedPosition = (fMetricMode) ? GetGlobalVarFloat(RETRACTED_X_NDX) * 25.4 : GetGlobalVarFloat(RETRACTED_X_NDX);
    XRetractedPositionSteps = XDistanceToSteps(XRetractedPosition);

	SaveGlobalVar(X_DIAMETER_NDX);	// Update EEROM menu entry with latest diameter location.

    M_DEBUGSTR("ZS=%ld, ZE=%ld, XS=%ld, XR%ld\n",
        ZBeginPositionSteps,
        ZEndPositionSteps,
        XRetractedPositionSteps,
        XBeginPositionSteps
    );
}

/*
 *  FUNCTION: MoveHome
 *
 *  PARAMETERS:			Motor#, HomeSpeed
 *
 *  USES GLOBALS:		MovementState,
 *						fLOkToStop
 *						fMTOkToStop
 *
 *  DESCRIPTION:		Calls MotorMoveTo with motor# and HomeSpeed
 *						Wrapper Function for the ALT-nHOME button combination.
 *
 *  RETURNS: 			Nothing.
 *
 */
void
MoveHome(uint8 mtr, WORD spd) {
	MotorStop(MOTOR_Z);		// Cancel any exising moves.
	MotorStop(MOTOR_X);		// Especially the X axis which has flags used in non-automatic mode.
	MotorMoveTo( mtr, 0, spd, SPINDLE_EITHER );
	fLOkToStop = 0;					// let it finish.
    fMTOkToStop = 0;            	// Tell system we're Busy
	M_DEBUGSTR("Move: MOVE_HOMING\n");
	MovementState = MOVE_HOMING;
}


/*
 *  FUNCTION: MovementStartThread
 *
 *  PARAMETERS:			None
 *
 *  USES GLOBALS:		MovementState		-- New state machine variable
 *						fMoveStartRQ		-- To request motion.
 *						fXMoveBSY			-- Cancel X move request
 *						CurrentZPosition	-- Set with ZMotorPosition.
 *						ZMotorPosition		-- Used for CurrentZPosition
 *						fMTRdy				-- Motor Ready
 *						fMTRun				-- Motor Running.
 *						fMovementThreadMode	--
 *						fXThreading
 *
 *  DESCRIPTION:
 *
 *  RETURNS:
 *
 */
void
MovementStartThread(void) {
    if (!fMTRun) {
        fMoveStartRQ = 1;       // Request motion
        // Grab current position
        INTCON &= 0x3F;
        fXMoveBSY = 0;          // X is now in position.
        CurrentZPosition = ZMotorPosition;
        INTCON |= 0xC0;
        // We're no longer ready
        fMTRdy = 0;             // but instead we're
        fMTRun = 1;             // running.
        // Mode change?
        if (fMovementThreadMode != fXThreading) {
            // Yes. Start from scratch.
            fMovementThreadMode = fXThreading;
	        MovementState = MOVE_WAIT;
        }
    }
}

/*
 *  FUNCTION: MovementStopThread
 *
 *  PARAMETERS:			None
 *
 *  USES GLOBALS:		fRunMachine
 *
 *  DESCRIPTION:		Request that machine stop.
 *
 *  RETURNS: 			Nothing
 *
 */
void
MovementStopThread(void) {
	M_DEBUGSTR("Stop Thread RQ\n");
    fRunMachine = 0;    // Request that we stop.
}

/*
 *  FUNCTION: MovementThread
 *
 *  PARAMETERS:			None
 *
 *  USES GLOBALS:		Too many to mention....
 *
 *  DESCRIPTION:		State machine thread that walks through
 *						the steps needed to thread, retract, return and enter
 *						the material for threading.
 *						See comments for each state to determine how it works.
 *
 *  RETURNS: 			Nothing.
 *
 */
void
MovementThread(void) {
static uint16 HomeSpeed, XSpeed;
static int32 TargetXPosition, TargetZPosition;

int8 i,p,c;

    if ( !fRunMachine && fLOkToStop )   {   // Execute this block any time we're stopped in Ready Mode.
        if (!fMTRdy) {			// Prevent this stuff from running over and over again....
        	fMTRun = 0;         // Say that we are not running any more.
        	fMTRdy = 1;         // Say that we are ready.
			fMTOkToStop = 1;    // Tell system we're stopped and can go to System Ready state if needed.
		}
        // Do any other stop initializations.
    }
    else switch (MovementState) {

      /*
        MOVE_WAIT --
            If tool is in the way of the work first either retract it
            automatically or ask user to.
            Once that's done (or if it's already done, then check to see if
            we're at the start of the thread.
            If not, move to the correct Z position otherwise, go to the state
            that will move the X into the work.
      */
      case MOVE_WAIT :
        if (fMoveStartRQ) {
            fMoveStartRQ = 0;
            fLOkToStop = 1;
			// Check if there is anywhere to go
			if (ZBeginPositionSteps == ZEndPositionSteps) {
                fRunMachine = 0;    // Ask machine to stop.
				SystemError = MSG_START_EQUALS_END_ERROR;
                DisplayModeMenuIndex = SystemError;
				break;
			}
            DisplayModeMenuIndex = MSG_FIND_BEGIN_MODE;
            M_DEBUGSTR("Checking for Start Position\n");
            // Pull out EEROM'd parameters and convert to steps
            HomeSpeed = GetGlobalVarWord(SLEW_RATE_Z_NDX);
			// Get variables that are changed inside interrupt routine.
            INTCON &= 0x3F;
                CurrentXPosition = XMotorRelPosition;
				CurrentZPosition = ZMotorPosition;
            INTCON |= 0xC0;
            // Check if tool in work and if so, move it out.
            if (CurrentXPosition != XRetractedPositionSteps) {  // Tool in work at the moment.
                if (fAutoX) {
                    // X axis has a stepper which is either direct or through CAN Bus.
                    // Check where we are and move to retracted position.
                    XSpeed = GetGlobalVarWord(SLEW_RATE_X_NDX);
                    ZEndPositionSteps = ZDistanceToSteps(ZEndPosition);

                    fLOkToStop = 0;
                    M_DEBUGSTR("Withdrawing Tool Automatically\n");
                    DisplayModeMenuIndex = MSG_RETRACT_TOOL_MODE;
                    // Retract tool.
                    if ((SystemError = MotorMoveTo(MOTOR_X, XRetractedPositionSteps, XSpeed, SPINDLE_EITHER)) != 0) {
                        fLOkToStop = 1;
                        fRunMachine = 0;    // Ask machine to stop.
                        MovementState = MOVE_WAIT;
                        DisplayModeMenuIndex = SystemError;
                        break;
                    }
                    // Tool move started, now wait for it to finish.
                    M_DEBUGSTR("Move: WAIT_X_OUT\n");
                    MovementState = MOVE_WAIT_X_OUT;
                }
                else {
                    // Ready to move in so ask user to move X out of work.
                    M_DEBUGSTR("User Withdraw Tool RQ\n");
            		M_DEBUGSTR("Move: AT_END\n");
            		MovementState = MOVE_AT_END;
                }
            }
            // We're at the right spot so go insert tool.
            else if (CurrentZPosition == ZBeginPositionSteps) {
                M_DEBUGSTR("Move: TO_START\n");
                MovementState = MOVE_TO_START;
            }
            // Not at the Z start but Tool is out so go move to Z start location.
            else {
                M_DEBUGSTR("Move: WAIT_X_OUT\n");
                MovementState = MOVE_WAIT_X_OUT;
            }
        }
        break;

	  /*
			User has pressed ALT-XHOME or ALT-ZHOME which sends appropriate motor to
			0.000" or 0.00mm
	  */
	  case MOVE_HOMING :
        if (!fXMoveBSY && !fZMoveBSY) {
            // Move is done or we've hit a limit or ESTOP.
            if (SystemError != 0) {
                fLOkToStop = 1;
                fRunMachine = 0;    // Ask machine to stop.
                MovementState = MOVE_WAIT;
                DisplayModeMenuIndex = SystemError;
                break;
            }
            fLOkToStop = 1;
            fRunMachine = 0;    // Ask machine to stop.

            M_DEBUGSTR("Move: WAIT\n");
            MovementState = MOVE_WAIT;
		}
		break;

      /*
            Wait for X axis to be automatically removed from work or
            for user to press START button again which says he's removed it.
            Once it's been removed, then decide which way to
            go and then issue move TO_START.
            If already at start then go to AT_START

      */
      case MOVE_WAIT_X_OUT :
        if (!fXMoveBSY) {
            // Move is done or we've hit a limit or ESTOP.
            if (SystemError != 0) {
                fLOkToStop = 1;
                fRunMachine = 0;    // Ask machine to stop.
                MovementState = MOVE_WAIT;
                DisplayModeMenuIndex = SystemError;
                break;
            }

            // Check if this is the beginning of a set of passes.
            if (PassCount > 0) {
                // Then calculate first pass trial.
                fLOkToStop = 1;
                M_DEBUGSTR("Move: WAIT_END\n");
                MovementState = MOVE_WAIT_END;
                break;
            }

            if (ZEndPositionSteps == ZBeginPositionSteps) { // At Headstock
                fLOkToStop = 1;
                DisplayModeMenuIndex = MSG_FIND_BEGIN_MODE;
                M_DEBUGSTR("Move: AT_START\n");
                MovementState = MOVE_AT_START;
                break;
            }
            else // Go to start position.
                if ((SystemError = MotorMoveTo( MOTOR_Z, ZBeginPositionSteps, HomeSpeed, SPINDLE_EITHER )) != 0) {
                    fLOkToStop = 1;
                    fRunMachine = 0;    // Ask machine to stop.
                    MovementState = MOVE_WAIT;
                    DisplayModeMenuIndex = SystemError;
                    break;
                }

            fLOkToStop = 0;
            DisplayModeMenuIndex = MSG_FIND_BEGIN_MODE;
            M_DEBUGSTR("Move: TO_START\n");
            MovementState = MOVE_TO_START;
        }
        break;


      /*
        MOVE_TO_START --
            Wait for arrival at start position.
      */
      case MOVE_TO_START :
        if (!fZMoveBSY && !fZMoveRQ) {      // move is complete.
            fLOkToStop = 1;
            // Move is done or we've hit a limit or ESTOP.
            if (SystemError != 0) {
                fRunMachine = 0;    // Ask machine to stop.
                MovementState = MOVE_WAIT;
                DisplayModeMenuIndex = SystemError;
                break;
            }
            else {
                DisplayModeMenuIndex = RUN_MODE;
                M_DEBUGSTR("Move: AT_START\n");
                MovementState = MOVE_AT_START;
            }
        }
        break;

      /*
        MOVE_AT_START --
            At the start position.
            Now issue tracked move_to_end command if total
            number of passes aren't complete.
            fZMoveRQ set when MotorMoveDistance has everything set up and requests
            interrupt routine to start move.
            fZMoveRQ cleared when Spindle Index Pulse Occurs
            fZMoveBSY set when Spindle Index Pulse Occurs
            fZMoveBSY cleared when distance move is complete.
            fAutoX set when there is a stepper on the X Axis
            fAutoX is clear when LCD dialogs and start button control
            each stage of the threading.
      */
      case MOVE_AT_START :
        // Move is done or we've hit a limit or ESTOP.
        if (SystemError != 0) {
            fLOkToStop = 1;
            fRunMachine = 0;    // Ask machine to stop.
            MovementState = MOVE_WAIT;
            DisplayModeMenuIndex = SystemError;
            break;
        }
        if (fAutoX) {
            // X axis has a stepper which is either direct or through CAN Bus.
            // So move X into work              newDistance = XRetractedPositionSteps - XCurrentPositionSteps;
            M_DEBUGSTR("Passes left %2d\n", PassCount);
            XSpeed = GetGlobalVarWord(SLEW_RATE_X_NDX);

            if ((SystemError = MotorMoveTo( MOTOR_X, XBeginPositionSteps, XSpeed, SPINDLE_EITHER )) != 0) {
                fLOkToStop = 1;
                fRunMachine = 0;    // Ask machine to stop.
                MovementState = MOVE_WAIT;
                DisplayModeMenuIndex = SystemError;
                break;
            }
            fLOkToStop = 0;     // No stop while X axis moving in.
            DisplayModeMenuIndex = MSG_INSERT_TOOL_MODE;
            M_DEBUGSTR("Moving Tool in\n");
            M_DEBUGSTR("Move: WAIT_X_DONE\n");
            MovementState = MOVE_WAIT_X_DONE;
        }
        else {
            // Ready to move in so ask user to move X and adjust compound
            // Pretend the motor is already there so the run time displays show where an operator should
            // put the X axis cross slide.
            // Diagnostics also should show start position of X axis.
#ifdef MOVE_DIAGNOSTICS
            floatToAscii(XBeginPosition, &OutputBuffer[0],7,3); OutputBuffer[8] = '/0';
            M_DEBUGSTR("Steps=%ld, Pos=%s\n",XBeginPositionSteps, OutputBuffer );
#endif

            INTCON &= 0x3F;
                XMotorRelPosition = XBeginPositionSteps;
	            fXMoveBSY = 1;
            INTCON |= 0xC0;

            fLOkToStop = 1;
            fRunMachine = 0;    // Ask machine to stop.
            DisplayModeMenuIndex = MSG_INSERT_TOOL_RQ_MODE;
            M_DEBUGSTR("Insert Tool and press START\n");
            M_DEBUGSTR("Move: WAIT_START\n");
            MovementState = MOVE_WAIT_START;
        }
        break;



        /*
            WAIT_X_DONE :
                X is being inserted so now wait for it to arrive
        */
      case MOVE_WAIT_X_DONE :
        if (!fXMoveBSY) {
            fLOkToStop = 1;             // And allow thread to stop.
            // Move is done or we've hit a limit or ESTOP.
            if (SystemError != 0) {
                MovementState = MOVE_WAIT;
                DisplayModeMenuIndex = SystemError;
                break;
            }
            else {
            #ifdef TAPERING
				c = TaperFlags.Byte;
				M_DEBUGSTR("TaperState=%02X \n", c);
/*
				if (fMovingRight)
					printf((far rom int8 *)"Moving to the right and ");
				else
					printf((far rom int8 *)"Moving to the left and ");
				if (!fExternalThreading)
					printf((far rom int8 *)"tapering internally ");
				else
					printf((far rom int8 *)"tapering externally ");
				if (fTaperIn)
					printf((far rom int8 *)"with taper smaller at headstock\n");
				else
					printf((far rom int8 *)"with taper larger at headstock\n");
*/
	            if (fTapering) {
					// Get parity of flags byte.
					//
					// Calculate parity.
					p = 0; c = TaperFlags.Byte;
					for (i=0; i<3;i++) {
						p += (c & 1);
						c >>= 1;
					}
					if (!(p&1))  {// Even parity means turn X axis around and remove backlash

	                    MotorMoveDistance(  MOTOR_X,
	                                        0,              // 0 Distance, just remove backlash.
	                                        XSpeed,
	                                        !fMovingOut,    // move in opposite direction of previous move
	                                        SPINDLE_EITHER,
											TRUE
	                                      );
	            		fLOkToStop = 0;             // Finish move.

//	                    printf((far rom int8 *)" Removing X Backlash\n");
	                    M_DEBUGSTR("Move: MOVE_WAIT_X_BACKLASH_DONE\n");
	                    MovementState = MOVE_WAIT_X_BACKLASH_DONE;
					}
					else {
	                    DisplayModeMenuIndex = RUN_MODE;
	                    M_DEBUGSTR("Move: WAIT_X_RDY\n");
	                    MovementState = MOVE_WAIT_X_RDY;
	                }
                }		// endif Tapering.
                else {
                    DisplayModeMenuIndex = RUN_MODE;
                    M_DEBUGSTR("Move: WAIT_X_RDY\n");
                    MovementState = MOVE_WAIT_X_RDY;
                }

            #else
                DisplayModeMenuIndex = RUN_MODE;
                M_DEBUGSTR("Move: WAIT_X_RDY\n");
                MovementState = MOVE_WAIT_X_RDY;
            #endif
            }
        }
        break;


        /*
            WAIT_X_BACKLASH_DONE :
                X has been inserted but taper direction states that it may now go in the opposite direction
                and if so we have to remove backlash on X or we get flat spot on the taper while Z is moving
                and X is moving through the backlash.  Really critical for tapers in the middle of a turned piece where
                dimensions are critical on either side of the taper.
        */
      case MOVE_WAIT_X_BACKLASH_DONE :
        if (!fXMoveBSY) {
            fLOkToStop = 1;             // And allow thread to stop.
            // Move is done or we've hit a limit or ESTOP.
            if (SystemError != 0) {
                MovementState = MOVE_WAIT;
                DisplayModeMenuIndex = SystemError;
                break;
            }
            else {
                M_DEBUGSTR(" X Backlash Removed\n");
                DisplayModeMenuIndex = RUN_MODE;
                M_DEBUGSTR("Move: WAIT_X_RDY\n");
                MovementState = MOVE_WAIT_X_RDY;
            }
        }
        break;


      /*
        WAIT_X_RDY :
          - X is inserted into the work.
          - Then request user insert tool by going to WAIT Start State.
          - This is the state where we'd decide how much to move X
            based on pass count and Depth Increment.  Currently not yet implimented.
          - For now we just ask user to tweak the X axis.
      */
      case MOVE_WAIT_X_RDY :
        if ((PassCount > 0) || (SpringPassCount > 0)) {
            // Then start next thread.
            // For now, for testing, we just continue as if new X,Z are set.
            // Still passes to do.  Don't ask machine to stop.
			// Since we've been allowed to stop here, double check user hasn't moved X somewhere.
			// If so, go put it back.
			if (XMotorRelPosition != XBeginPositionSteps) {
                DisplayModeMenuIndex = RUN_MODE;
                M_DEBUGSTR("Move: AT_START\n");
                MovementState = MOVE_AT_START;
			}
			else {
	            fLOkToStop = 0;
            	M_DEBUGSTR("Move: WAIT_START\n");
            	MovementState = MOVE_WAIT_START;
			}
        }
        else {
            SpringPassCount = 1;

            fLOkToStop = 1;
            fRunMachine = 0;    // Ask machine to stop.
            DisplayModeMenuIndex = MSG_SEQ_COMPLETE_MODE;
            M_DEBUGSTR("Threading Sequence Complete\n");
            M_DEBUGSTR("Move: WAIT\n");
            MovementState = MOVE_WAIT;
        }
        break;

      /*
        MOVE_WAIT_START --
            Check if we are at the start of the thread or turning location and then
            if not goto MOVE_WAIT
            Otherwise, Wait for start button to be pressed and start threading.

      */
      case MOVE_WAIT_START :
        INTCON &= 0x3F;
			CurrentZPosition = ZMotorPosition;
        INTCON |= 0xC0;

        if (CurrentZPosition != ZBeginPositionSteps) {
                fLOkToStop = 1;
                M_DEBUGSTR("Move: WAIT\n");
                MovementState = MOVE_WAIT;
        }
        else {
            if ( fSpindleTurning || fBroachMode) {

                if (SystemError != 0) { // Move is done or we've hit a limit or ESTOP.
                    fLOkToStop = 1;
                    fRunMachine = 0;    // Ask machine to stop.
                    MovementState = MOVE_WAIT;
                    DisplayModeMenuIndex = SystemError;
                    break;
                }
                // X move is complete or user pressed RUN after manual adjustment.
                fLOkToStop = 0;
                HomeSpeed = GetGlobalVarWord(MOVE_RATE_Z_NDX);

                // Do our thread or turning pass.
                M_DEBUGSTR("Thread to End ... ");
				if (fBroachMode)
					SystemError = MotorMoveTo( MOTOR_Z, ZEndPositionSteps, HomeSpeed, SPINDLE_EITHER );
                else
					SystemError = MotorMoveTo( MOTOR_Z, ZEndPositionSteps, SPEED_TRACK_SPINDLE, SPINDLE_TURNING );

				if (SystemError == 0) {
                    DisplayModeMenuIndex = MSG_THREAD_END_MODE;
                    M_DEBUGSTR("Threading to End Position\n");
                    M_DEBUGSTR("Move: TO_END\n");
                    MovementState = MOVE_TO_END;
                }
                else {  // Motor movement error of some sort.
                    fLOkToStop = 1;
                    fRunMachine = 0;    // Ask machine to stop.
                    MovementState = MOVE_WAIT;
                    DisplayModeMenuIndex = SystemError;
                    break;
                }
            }
            else {
                fRunMachine = 0;    // Ask machine to stop.
                fLOkToStop = 1;
		        INTCON &= 0x3F;
                fXMoveBSY = 0;
        		INTCON |= 0xC0;
                DisplayModeMenuIndex = MSG_SPINDLE_OFF_MODE;
                SystemError = MSG_SPINDLE_OFF_MODE;
                M_DEBUGSTR("Spindle Not Turning\n");
                M_DEBUGSTR("Move: WAIT\n");
                MovementState = MOVE_WAIT;
            }
        }
        break;



      /*
        MOVE_TO_END --
            Wait till end point is reached.
      */
      case MOVE_TO_END :
        if (!fZMoveRQ && !fZMoveBSY) {      // Z move is complete.
            fLOkToStop = 1;
            // Move is done or we've hit a limit or ESTOP.
            if (SystemError != 0) {
                fRunMachine = 0;    // Ask machine to stop.
                MovementState = MOVE_WAIT;
                DisplayModeMenuIndex = SystemError;
                break;
            }
            DisplayModeMenuIndex = MSG_THREAD_END_MODE;
            M_DEBUGSTR("Move: AT_END\n");
            MovementState = MOVE_AT_END;
        }
        else if (!fSpindleTurning && !fBroachMode) {
            MotorStop(MOTOR_Z);             // So stop motor
            MotorStop(MOTOR_X);
            fLOkToStop = 1;
            fRunMachine = 0;    // Ask machine to stop.
            DisplayModeMenuIndex = MSG_SPINDLE_OFF_MODE;
            SystemError = MSG_SPINDLE_OFF_MODE;
            M_DEBUGSTR("Move: WAIT\n");
            MovementState = MOVE_WAIT;
        }
        break;

      /*
        MOVE_AT_END --
            At end point. Now withdraw tool or ask operator to do it.
      */
      case MOVE_AT_END :
        if (fAutoX) {
            // X axis has a stepper which is either direct or through CAN Bus.
            // So move X out of the work.
            XSpeed = GetGlobalVarWord(SLEW_RATE_X_NDX);

            if ((SystemError = MotorMoveTo( MOTOR_X, XRetractedPositionSteps, XSpeed, SPINDLE_EITHER )) != 0) {
                fLOkToStop = 1;
                fRunMachine = 0;    // Ask machine to stop.
                MovementState = MOVE_WAIT;
                DisplayModeMenuIndex = SystemError;
                break;
            }
            fLOkToStop = 0;
            DisplayModeMenuIndex = MSG_RETRACT_TOOL_MODE;
        }
        else {
            // Ready to move out so ask user to move X and adjust compound
            fXMoveBSY = 1;
            fMoveStartRQ = 0;   // Start button sets this.
            fRunMachine = 0;    // Ask machine to stop.
            fLOkToStop = 1;
            INTCON &= 0x3F;
            XMotorRelPosition = XRetractedPositionSteps;       // Fake out where we are for run time display.
            INTCON |= 0xC0;
            DisplayModeMenuIndex = MSG_RETRACT_TOOL_RQ_MODE;
            M_DEBUGSTR("Withdraw Tool and press START\n");
        }
        M_DEBUGSTR("Move: WAIT_END_OUT\n");
        MovementState = MOVE_WAIT_END_OUT;
        break;


      /*
        MOVE_WAIT_END_OUT --
            Now wait for operator to remove tool (or do it automatically)
      */
      case MOVE_WAIT_END_OUT :
        if ( !fXMoveBSY )  { // X axis move is complete
            // Move is done or we've hit a limit or ESTOP.
            if (SystemError != 0) {
                fRunMachine = 0;    // Ask machine to stop.
                MovementState = MOVE_WAIT;
                DisplayModeMenuIndex = SystemError;
                break;
            }
            fLOkToStop = 1;
            M_DEBUGSTR("Move: WAIT_END\n");
            MovementState = MOVE_WAIT_END;
        }
        break;

      /*
        MOVE_WAIT_END --
            Now wait for operator to press start or loop on pass count.
      */
      case MOVE_WAIT_END :
        if ( fMoveStartRQ || ((PassCount+SpringPassCount) > 0) ) {
            // Move is done or we've hit a limit or ESTOP.
            if (SystemError != 0) {
                fLOkToStop = 1;
                fRunMachine = 0;    // Ask machine to stop.
                MovementState = MOVE_WAIT;
                DisplayModeMenuIndex = SystemError;
                break;
            }
			// Since user was allowed to stop machine the X axis may not be where we think it is.
			// Check that and if it's not retracted, go yank it out first.
			if (XMotorRelPosition != XRetractedPositionSteps) {
	            DisplayModeMenuIndex = MSG_THREAD_END_MODE;
    	        M_DEBUGSTR("Move: AT_END\n");
        	    MovementState = MOVE_AT_END;
				break;
			}

            HomeSpeed = GetGlobalVarWord(SLEW_RATE_Z_NDX);

            // Under automatic mode, and threading, do all depth passes and do the extra finishing pass
            // at the final depth.
            // If not using the compound then Calculate the new Z and X positions.
            if (/*(fAutoX || fUseCompound) &&*/ fXThreading) {
                if (PassCount > 0) {
                    PassCount = (--PassCount <= 0) ? 0 : PassCount;
                }
                else if (SpringPassCount > 0) {
                    SpringPassCount = (--SpringPassCount <= 0) ? 0 : SpringPassCount;
                }
				CalculatePosition(fUseCompound);		// If using compound don't actually change position here
				if (fUseCompound)  // Now calculate hypotenuse and use that instead).
					CalculateCompoundPosition();

            }
            M_DEBUGSTR("Return To Start ... ");
            MotorMoveTo( MOTOR_Z, ZBeginPositionSteps, HomeSpeed, SPINDLE_EITHER );

            // If we're turning terminate at the end of this single pass.
            if (!fXThreading) {
                PassCount = 0;
				SpringPassCount = 0;
            }
            fMoveStartRQ = 0;   // Start button sets this.
            fLOkToStop = 0;
            DisplayModeMenuIndex = MSG_FIND_BEGIN_MODE;
            M_DEBUGSTR("Move: WAIT_TO_START\n");
            MovementState = MOVE_WAIT_TO_START;
        }
        else if ((PassCount+SpringPassCount) <= 0)  {
            M_DEBUGSTR("Move: WAIT_X_RDY\n");
            MovementState = MOVE_WAIT_X_RDY;
        }
        break;

      /*
        MOVE_WAIT_END_TO_START --
            Heading back to start position.
      */
      case MOVE_WAIT_TO_START :
        if (!fZMoveRQ && !fZMoveBSY) {      // Z move is complete.
            fLOkToStop = 1;     // Allow stop button to work.
            // Move is done or we've hit a limit or ESTOP.
            if (SystemError != 0) {
                fRunMachine = 0;    // Ask machine to stop.
                MovementState = MOVE_WAIT;
                DisplayModeMenuIndex = SystemError;
                break;
            }
            INTCON &= 0x3F;
				CurrentZPosition = ZMotorPosition;
            INTCON |= 0xC0;
            if (CurrentZPosition == ZBeginPositionSteps) {
                M_DEBUGSTR("Move: TO_START\n");
                MovementState = MOVE_TO_START;
            }
            else {
                M_DEBUGSTR("Move: WAIT\n");
                MovementState = MOVE_WAIT;
            }
        }
        break;

    }
}
