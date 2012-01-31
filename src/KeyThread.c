// KeyThread.c
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

#include "processor.h"

#include <Ctype.h>
#include <stdio.h>
#include <stdlib.h>

#include "common.h"
#include "config.h"

#include "menu.h"
#include "menuscript.h"
#include "globvars.h"	

#include "floatio.h"
#include "OB_EEROM.H"
#include "Timer.h"

#include "int.h"
#include "beep.h"
#include "key.h"
#include "LCD.h"
#include "MotorDriver.h"	// Motor Driver Defintions.
#include "Serial.h"

#include "MovementThread.h"
#include "KeyThread.h"
#include "MPG_Thread.h"

/* ------------  Private Declarations and Data -------------*/

static int8 maskedKey;
static uint8 KeyButton;
enum ELSKEYS_STATES ELSKeys_State;

BITS KEYFlags;

void 
ToggleBacklight(void) {
	if (bLCD_BACK_Light) 
		bLCD_BACK_Light	= 0;
	else
		bLCD_BACK_Light	= 1;
}

/*
 *  FUNCTION: InitRunKeyThread
 *
 *  PARAMETERS:		None
 *
 *  USES GLOBALS:	ELSKeys_State
 *
 *  DESCRIPTION:	Sets ELSKeys_State to INIT.
 *
 *  RETURNS: 		Changes ELSKeys_State
 *
 */
void
InitRunKeyThread(void) {
	ELSKeys_State = ELSKEYS_INIT;
}

/*
 *  FUNCTION: InitRdyKeyThread
 *
 *  PARAMETERS:		None
 *
 *  USES GLOBALS:	ELSKeys_State
 *
 *  DESCRIPTION:
 *
 *  RETURNS: 		Changes ELSKeys_State 
 *
 */
void
InitRdyKeyThread(void) {
	ELSKeys_State = ELSKEYS_INIT;
}

/*
 *  FUNCTION: InitIdleKeyThread
 *
 *  PARAMETERS:		None
 *
 *  USES GLOBALS:	ELSKeys_State
 *
 *  DESCRIPTION:
 *
 *  RETURNS: 		Changes ELSKeys_State
 *
 */
void
InitIdleKeyThread(void) {
	ELSKeys_State = ELSKEYS_INIT;
	KEYFlags.Byte = 0;		// Clear Period and Number Keys Flags
}

/*
 *  FUNCTION: InitErrorKeyThread
 *
 *  PARAMETERS:		None
 *
 *  USES GLOBALS:	ELSKeys_State
 *
 *  DESCRIPTION:
 *
 *  RETURNS: 		Changes ELSKeys_State
 *
 */
void
InitErrorKeyThread(void) {
	ELSKeys_State = ELSKEYS_INIT;
}

/*
 *  FUNCTION: UpdateDistances
 *
 *  PARAMETERS:		None
 *
 *  USES GLOBALS:	LEADSCREW_IPITCH_NDX
 *					LEADSCREW_BACKLASH_NDX
 *					THREAD_END_NDX
 *					CROSS_SLIDE_IPITCH_NDX
 *					MOTOR_STEPS_REV_X_NDX
 *					CROSS_SLIDE_IPITCH_NDX
 *					RETRACTED_X_NDX
 *					TAPER_NDX
 *					fTapering, fTaperInwards
 *
 *  DESCRIPTION:	After user parameters have been manually changed this function is called
 *					to update all the global variables needed for various automatic movements.
 *
 *  RETURNS: 		Nothing
 *					Changes
 *						ZDistanceDivisor
 *						SystemZBackLashCount
 *						ZEndPosition
 *						ZEndPositionSteps
 *						XDistanceDivisor
 *						XRetractedPosition
 *						XRetractedPositionSteps
 *						fExternalThreading
 *						fMovingRight, fMovingOut
 *						fTaperIn, fTaperDirection
 *						TaperTangent
 */
void 
UpdateDistances(void) {
	float32 temp;

	// Calculate constants used during motion commands to save doing them again and again.
	if (GlobalVars[LEADSCREW_IPITCH_NDX].f != 0) // Prevent divide by zero.
		ZDistanceDivisor = GlobalVars[MOTOR_STEPS_REV_Z_NDX].f / GlobalVars[LEADSCREW_IPITCH_NDX].f;
	else
		ZDistanceDivisor = 1.0;

	// Renew the backlash count just in case a user has changed it now that there is a new ZDistanceDivisor.
	temp = GetGlobalVarFloat(LEADSCREW_BACKLASH_NDX);

	//Changed by RE Rev 1.10f
	if (fMetricMode)
		temp = temp * 25.4;
	//End of change

	SystemZBackLashCount = CalculateMotorDistance(temp, ZDistanceDivisor, fMetricMode);

	ZEndPosition = (fMetricMode) ? GetGlobalVarFloat(THREAD_END_NDX) * 25.4 : GetGlobalVarFloat(THREAD_END_NDX);
	ZEndPositionSteps = ZDistanceToSteps(ZEndPosition);
	
	if (GlobalVars[CROSS_SLIDE_IPITCH_NDX].f != 0) // Prevent divide by zero.
		XDistanceDivisor = (float32)GlobalVars[MOTOR_STEPS_REV_X_NDX].l / GlobalVars[CROSS_SLIDE_IPITCH_NDX].f;
	else
		XDistanceDivisor = 1.0;
	
	XRetractedPosition = (fMetricMode) ? GetGlobalVarFloat(RETRACTED_X_NDX) * 25.4 : GetGlobalVarFloat(RETRACTED_X_NDX);
	XRetractedPositionSteps = XDistanceToSteps(XRetractedPosition);

	fExternalThreading = (XBeginPosition < XRetractedPosition) ? 1 : 0; // External Thread if true
	fMovingRight = (ZBeginPosition < ZEndPosition) ? 1 : 0;		// Set if moving away from the headstock.
	fMovingOut = !fExternalThreading;

	if (fTapering) {

		fTaperIn = fTaperInwards;			// Used for tracking tapers for X axis backlash adjustment.
		// fTaperInwards is set if diameter gets smaller as carriage moves towards headstock
		fTaperDirection = fTaperInwards;	// Used in interrupt routine for determining step direction.

		temp = GetGlobalVarFloat(TAPER_NDX);
		if (temp < 0.0) {
			temp = 0-temp;	// Make positive.
		}
		// The taper is stored in diameter per 1" distance, we're doing radius movements.
		temp /= 2.0;  // So we move half the X distance to get the radius.	
		/*
			This is where things get interesting.  
			Tapering is driven by X slaved to the Z axis.  At best, for every Z step we can generate one
			X step.  If the two leadscrews are the same and the motors are the same then it's possible to
			do a 45 degree angle; one Z step creates one X step.
			But if the carriage lead screw is large compared to the cross slide screw (and it ususally is) then
			a 1:1 stepping ratio will not produce a 1:1 movement.  That's because 1 Z step cannot produce 
			1.3 X axis steps.
			The only way around that would be to use toothed belts and pulleys to equalize the motion so the ratios
			were the same between axis.  
			The calculation below demonstrates how we handle the different ratios.
		*/
		TaperTangent = ((temp * 65536.0) * (XDistanceDivisor / ZDistanceDivisor)+0.5);  // Round upwards.

		if (TaperTangent > 0x10000) {
			DEBUGSTR("ERROR:  Taper too large\n");
			TaperTangent = 0x10000;				// Force to one X axis step per Z axis step.
		}

#ifdef	FULL_DIAGNOSTICS	// Save some code space if we're not doing diagnostics.
		floatToAscii(temp, OutputBuffer,7,5);
		DEBUGSTR("\nTaper is: %s,",OutputBuffer);

		floatToAscii((XDistanceDivisor / ZDistanceDivisor), OutputBuffer,7,5);
		DEBUGSTR(" With Division Ratio: %s\n",OutputBuffer);
		DEBUGSTR("T=%ld, Move X ",TaperTangent);
		if (fTaperInwards) 
			DEBUGSTR("in");
		else
			DEBUGSTR("out");
		DEBUGSTR(" while moving towards headstock.\n");

		DEBUGSTR("Taperflags=%02X",TaperFlags.Byte);
#endif
 	}
}

/*
 *  FUNCTION: SetThreadingMode
 *
 *  PARAMETERS:		None
 *
 *  USES GLOBALS:	fXThreading
 *					fMetricMode
 *					
 *  DESCRIPTION:	Called each time the THREAD button is pressed.
 *					If the machine wasn't in threading mode then threading mode
 *					is set and the passes state machine is reset to recalculate
 *					thread depth and the number of passes need.
 *
 *  RETURNS: 		Changes
 *						fXThreading
 *						MotionPitchIndex	-- Display Thread pitch on display
 */
void
SetThreadingMode(void) {
	// We're threading so multiple passes and if auto X the program changes X and Z start position on each pass.
	if (!fXThreading) {
		fXThreading = 1;	// Set threading if we weren't.
		SetCalculatePositionState(CALCULATE_PASSES);
	}
	// Since a user may have been changing parameters, let's update all the run time parameters from
	// our global data variables.  
	// We don't update Z start or X start because we may have done a feed hold and the threading code
	// adjusts these so unless the operator 'resets' the parameters we leave them alone.
	UpdateDistances();
	// Set up Jog table to Metric or Imperial just in case operator has changed units.
	ChangeJogDistance(fMetricMode);
	
	// Pass counts are modified from SFN3 key when threading 

	// Select this global variable when displaying Z turning/threading pitch
	MotionPitchIndex = THREAD_SIZE_NDX;
	// Show RPM, pitch, X and Z position.
	SetDisplayMode(ADJUST_MODE);
}

/*
 *  FUNCTION: SetTurningMode
 *
 *  PARAMETERS:		None
 *
 *  USES GLOBALS:	fMetricMode
 *
 *  DESCRIPTION:	Called when TURNING Mode button is pressed. 
 *					Sets the passes so that only one automatic pass (spring) is done,
 *					and the start and end positions are recalculated.
 *
 *  RETURNS: 		Changes
 *						fXThreading
 *						PassCount
 *						SpringPassCount		
 *						ZBeginPosition
 *						ZBeginPositionSteps
 *						XBeginPosition
 *						XBeginPositionSteps
 *						MotionPitchIndex		-- Show Turning pitch on display.
 */
void
SetTurningMode(void) {
	// We're turning
	fXThreading = 0;	// Single passes without calculations

	SetCalculatePositionState(LAST_PASS); 	// No automatic passes when threading.  Single pass only.
	PassCount = 0;
	SpringPassCount = 1;					// Let movement pass pretend we do a spring pass.

	// Reset the start positions since Threading mode or Parameter Entry Mode may have changed these.
	ZBeginPosition = (fMetricMode) ? GetGlobalVarFloat(THREAD_BEGIN_NDX) * 25.4 : GetGlobalVarFloat(THREAD_BEGIN_NDX);
	ZBeginPositionSteps = ZDistanceToSteps(ZBeginPosition);

	XBeginPosition = (fMetricMode) ? GetGlobalVarFloat(BEGIN_X_NDX) * 25.4 : GetGlobalVarFloat(BEGIN_X_NDX);
	XBeginPositionSteps = XDistanceToSteps(XBeginPosition);

	// Since a user may have been changing parameters, let's update all the run time parameters from
	// our global data variables.
	UpdateDistances();

	// Set up Jog table to Metric or Imperial just in case operator has changed units.
	ChangeJogDistance(fMetricMode);
	
	// Select this global variable when displaying Z turning/threading pitch
	MotionPitchIndex = TURN_PITCH_NDX;
	// Show RPM, pitch, X and Z position.
	SetDisplayMode(ADJUST_MODE);
}

/*
 *  FUNCTION: RunKeyThread
 *
 *  PARAMETERS:		None.
 *
 *  USES GLOBALS:	ELSKeys_State
 *					KeyValue
 *					SystemCommand
 *					BeepMsg
 *					SystemError
 *
 *  DESCRIPTION:
 *		Monitors what keys do what when the machine is in the global RUN state.
 *		Green light On.
 *
 *		Enabled keys are:
 *			KEY_UP 		-- Turns on LCD Backlight.
 *			KEY_DWN		-- Turns off LCD Backlight.
 *			KEY_START 	-- Sets the display to show RUN data
 *						   Requests that the machine START
 *			KEY_STOP,
 *			KEY_ESC		-- Sets the display to show ADJUST data
 *						   Requests that the machine STOP
 *
 *		No other keys are functional and generate a 4 beep error noise.
 *
 *  RETURNS: 	Nothing
 *
 */
void
RunKeyThread(void) {
  
	switch (ELSKeys_State) {

	  /*
	   * Initialize ELS Key pad state machine
	   */
	  case ELSKEYS_INIT :
		ELSKeys_State = ELSKEYS_KEY_WAIT;
		break;

	  /*
	   * Wait for a key press.
	   */
	  case ELSKEYS_KEY_WAIT :

		if (KeyValue & KEY_PRESSED) { // Key pressed

			ELSKeys_State = ELSKEYS_KEY_DONE;  // Most cases we go here after a key press.

		  	switch( KeyValue & 0x3F ) {

			  case KEY_UP : 
//				bLCD_BACK_Light	= 1;
				break;

			  case KEY_DOWN :
//				printf((MEM_MODEL rom char *)" RunKeyThread:Backlight Toggle\n");
				ToggleBacklight();
//				bLCD_BACK_Light	= 0;
				break;

			  case KEY_START :
				SetDisplayMode(RUN_MODE);
				MovementStartThread();
				SystemCommand = START_CMD;
				DEBUGSTR("RUN:KeyStart\n");
				break;

			  /*
				When running we normally want to stop at the end of a sequence.  But if the tool
				is headed for danger, holding down the STOP button will cause an immediate stop.
				So the idea is a stop tap requests a stop but we check too for STOP held and if 
				so... Under MACH software the STOP is called FEED HOLD and there is a second button 
				that does an absolute stop right now;  we don't have that button on the ELS.
			  */
			  case KEY_STOP :
				ELSKeys_State = ELSKEYS_KEY_TAPPED;	// If tapped just do same as ESC.
//			  case KEY_ESC :	// Go up.
				MovementStopThread();
				SystemCommand = STOP_CMD;
				SetDisplayMode(ADJUST_MODE);
				DEBUGSTR("RUN:KeyStop\n");
				break;

			  default :
				BeepMsg = T_BAD_KEY | FOUR_BEEPS;
				break;
			}

			StartTimer( ELSKEYS_TIMER, T_KEY_DOWN );
		}
	    break;

	  /*
	   * check if the key has already been released. 
	   * If so then just perform the tap function,
	   * otherwise perform the pressed function.
	   */
	  case ELSKEYS_KEY_TAPPED :

		if (KeyValue & KEY_HELD_DOWN) { // Key still pressed
		
		 	switch( KeyValue & KEY_MASK ) {  // See if it's ours.
		
			  case KEY_STOP :	// Stop in middle of move if STOP key held down.
				SystemCommand = STOP_CMD;
				SystemError = MSG_MOTOR_STOPPED_ERROR;
				MotorStop(MOTOR_Z);				// So stop motor
				MotorStop(MOTOR_X);
				break;
			}
			ELSKeys_State = ELSKEYS_KEY_HOLD;
		}
		else if ((KeyValue & KEY_MASK)== KEY_RELEASED)  {	// Key no longer pressed.
		
			// When key is released change state.
			ELSKeys_State = ELSKEYS_KEY_DONE;
		}		
		break;

	  /*
	   * Now wait for the key to be released and perform any
	   * post processing.
	   */
	  case ELSKEYS_KEY_HOLD :
	  	if ((KeyValue & KEY_MASK) == KEY_RELEASED) { // no switch

			ELSKeys_State = ELSKEYS_KEY_WAIT;
		}
		break;

	  /*
	   * Now wait for the key to be released
	   */
	  case ELSKEYS_KEY_DONE :
	  	if ((KeyValue & KEY_MASK) == KEY_RELEASED) { // no switch
			ELSKeys_State = ELSKEYS_KEY_WAIT;
		}
		else if ((KeyValue & KEY_HELD_DOWN) == KEY_HELD_DOWN) {
			ELSKeys_State = ELSKEYS_KEY_WAIT;
		}
		break;

	  default :
			ELSKeys_State = ELSKEYS_KEY_WAIT;
		break;
	}
}

/*
 *  FUNCTION: RdyKeyThread
 *
 *  PARAMETERS:		None
 *
 *  USES GLOBALS:	ELSKeys_State
 *					KeyValue, fKeyRdy
 *					SystemCommand
 *					BeepMsg
 *					SystemError
 *					fRunTimeDisplay
 *					LastMenuNumber
 *					CurrentMenuNumber
 *
 *  DESCRIPTION:	Called when in READY mode (Green light flashing).
 *					This is the operational state machine that allows motion.
 *					So the steppers motors are live and the START/ALT-nHOME
 *					buttons start automatic movement.
 *
 *  RETURNS: 		Nothing
 *
 */
void
RdyKeyThread(void) {
  int32 mtr;
  float32 res;
  int32 newDistance;
  
	switch (ELSKeys_State) {

	  /*
	   * Initialize ELS Key pad state machine
	   */
	  case ELSKEYS_INIT :
		fKeyRdy = 1;
		ELSKeys_State = ELSKEYS_KEY_WAIT;
		break;

	  /*
	   * Wait for a key tap or key press.
	   * We save the key and then check it a bit later 
	   * to see if it's still pressed or was just tapped.
	   * If it's not a tap enabled key we just do what's needed
	   * right away.
	   */
	  case ELSKEYS_KEY_WAIT :

		if (KeyValue & KEY_PRESSED) { // Key pressed
//			printf((MEM_MODEL rom char *)" RdyKey:KeyValue %04X\n", KeyValue);
			
			StartTimer( ELSKEYS_TIMER, T_KEY_DOWN ); 
			fKeyRdy = 0; 
			ELSKeys_State = ELSKEYS_KEY_TAPPED;

			if (CurrentMenuNumber == -1) {  // Startup or Running with status display.
				if (!fRunTimeDisplay) {		// Must have been Startup.
					GetMenu(1, &CurrentMenu, KeyNumberBuffer);	// Get Configuration Menu.
					LastMenuNumber = 1;
					CurrentMenuNumber = 1;
				}
				else
					CurrentMenuNumber = 1;
				fRunTimeDisplay = 0;
			}
			maskedKey = KeyValue & KEY_MASK;
			KeyButton = maskedKey;				// Save key code.
		  	switch( maskedKey ) {
			   /*
			   * Do KEY Operation and then wait for key to be released.
			   */
			  case KEY_UP : // or '-' if fNumberKeys
				if (fNumberKeys && (KeyNumberIndex == 0)) {
					KeyNumberBuffer[KeyNumberIndex++] = '-';
					LCDSendChar('-');
				}
//				else
//					bLCD_BACK_Light	= 1;

				ELSKeys_State = ELSKEYS_KEY_DONE;
				break;

			  case KEY_DOWN :
				if (!fNumberKeys) {
					ToggleBacklight();
				}
				ELSKeys_State = ELSKEYS_KEY_DONE;
				break;

			  /*
			   * Move Right at the Slew speed parameter until the key is released.
			   */
			  case KEY_RFAST : // Slew right
				SetDisplayMode(ADJUST_MODE);
				MotorJog(MOTOR_Z, (WORD)GetGlobalVarWord(SLEW_RATE_Z_NDX), MOVE_RIGHT);
				fSwitchDown = 1;
				ELSKeys_State = ELSKEYS_KEY_HOLD;

				break;

			  /*
			   * Move Left at the Slew speed parameter until the key is released.
			   */
			  case KEY_LFAST : // Slew left
				SetDisplayMode(ADJUST_MODE);
				MotorJog(MOTOR_Z, (WORD)GetGlobalVarWord(SLEW_RATE_Z_NDX), MOVE_LEFT);
				fSwitchDown = 1;
				ELSKeys_State = ELSKEYS_KEY_HOLD;
				break;

			  /*
			   * Travel right a short distance or Move right after x milliseconds.
			   * Save key and check in bit to see if it was just tapped or held down.
			   * The short travel happens right away but the move will happen after the key is held down
			   * for a 1/2 second or so.
			   */
			  case KEY_RZJOG : // travel right
				SetDisplayMode(ADJUST_MODE);
				res = GetGlobalVarFloat(MOTOR_ZADJ_LENGTH_NDX);
				// Don't convert to metric since distance is store in imperial units.
				newDistance = CalculateMotorDistance( res, ZDistanceDivisor, 0 );	
				MotorMoveDistance(  MOTOR_Z, newDistance, (WORD)GetGlobalVarWord(MOVE_RATE_Z_NDX), MOVE_RIGHT, 
						SPINDLE_EITHER,  ~(bLIMIT_Switch ^ fLimitSwitch)  );
				ELSKeys_State = ELSKEYS_KEY_TAPPED;
				break;

			  /*
			   * Travel left a short distance or Move left after x milliseconds.
			   * Save key and check in bit to see if it was just tapped or held down.
			   * The short travel happens right away but the move will happen after the key is held down
			   * for a 1/2 second or so.
			   */
			  case KEY_LZJOG : // travel Left
				SetDisplayMode(ADJUST_MODE);
				res = GetGlobalVarFloat(MOTOR_ZADJ_LENGTH_NDX);
				// Don't convert to metric since distance is store in imperial units.
				newDistance = CalculateMotorDistance( res, ZDistanceDivisor, 0 );
				MotorMoveDistance( MOTOR_Z, newDistance, (WORD)GetGlobalVarWord(MOVE_RATE_Z_NDX), MOVE_LEFT,
						 SPINDLE_EITHER,  ~(bLIMIT_Switch ^ fLimitSwitch) );
				ELSKeys_State = ELSKEYS_KEY_TAPPED;
				break;

			  /*
			   * Travel inwards a short distance or at move rate after x milliseconds.
			   * Save key and check in bit to see if it was just tapped or held down.
			   * The short travel happens right away but the move will happen after the key is held down
			   * for a 1/2 second or so.
			   */
			  case KEY_XJOG_IN : // Move cross slide inwards.
				res = GetGlobalVarFloat(MOTOR_XADJ_LENGTH_NDX);
				newDistance = CalculateMotorDistance( res, XDistanceDivisor, 0 );
				MotorMoveDistance(  MOTOR_X, newDistance, (WORD)GetGlobalVarWord(MOVE_RATE_X_NDX), MOVE_IN,
						 	SPINDLE_EITHER,  ~(bLIMIT_Switch ^ fLimitSwitch) );
				SetDisplayMode(ADJUST_MODE);
				ELSKeys_State = ELSKEYS_KEY_TAPPED;
				break;

			  /*
			   * Travel outwards a short distance or at move rate after x milliseconds.
			   * Save key and check in bit to see if it was just tapped or held down.
			   * The short travel happens right away but the move will happen after the key is held down
			   * for a 1/2 second or so.
			   */
			  case KEY_XJOG_OUT : // Move cross slide outwards.
				SetDisplayMode(ADJUST_MODE);
				res = GetGlobalVarFloat(MOTOR_XADJ_LENGTH_NDX);
				newDistance = CalculateMotorDistance( res, XDistanceDivisor, 0 );
				MotorMoveDistance( MOTOR_X, newDistance, (WORD)GetGlobalVarWord(MOVE_RATE_X_NDX), MOVE_OUT,
							SPINDLE_EITHER,  ~(bLIMIT_Switch ^ fLimitSwitch) );
				ELSKeys_State = ELSKEYS_KEY_TAPPED;
				break;

			  /*
			   * Set display to show RUN mode parameters and issue START to mainline state machine.
			  */
			  case KEY_START :
				SetDisplayMode(RUN_MODE);
				SystemCommand = START_CMD;
				ELSKeys_State = ELSKEYS_KEY_DONE;
				break;

			
			  /*
			   * Set display to show ADJUST mode parameters and request machine STOP as soon as possible
			   * If key held down, then abort movement immediately.
			  */
			  case KEY_STOP :
				SystemCommand = STOP_CMD;
				SetDisplayMode(ADJUST_MODE);
				ELSKeys_State = ELSKEYS_KEY_DONE;
				break;

			  /*
			   * Threading and Turning uses a start and end position for the Z axis (Carriage).
			   * The quick way to set the start position is with a dedicated button.
			  */
			  case KEY_SET_BEGIN :
				SetDisplayMode(KEY_MENU_MODE);	// Disable run time display and allow menu's
				INTCON &= 0x3F;
				mtr = ZMotorPosition;			// Motor position in steps.
				INTCON |= 0xC0;
				res = StepsToZDistance( mtr );	// Convert to distance in either mm or inch.
				res = (fMetricMode) ? res / 25.4 : res;		// If mm then covert to inch.
				// Set the global variable so the menu displays the correct value.
				SetGlobalVarFloat(THREAD_BEGIN_NDX, res );	// Save motor position in imperial units.
				// Show Menu for BEGIN POSITION.
				LastMenuNumber = 1;
				CurrentMenuNumber = 0x19;		// Start position Menu.
				// Get Menu and value from GlobalVariable array.
				KeyNumberIndex = GetMenu(CurrentMenuNumber, &CurrentMenu, KeyNumberBuffer);
				if (KeyNumberIndex) {
					fNumberKeys = 1;
					KeyMaxLength = KeyNumberIndex;
				}
				else
					fNumberKeys = 0;

				// Since we had to set the global memory value to the current position in order
				// to potentially accept (ENTER) or reject (ESC) the value.  We'd better put the 
				// original back just in case it's rejected.
				LoadGlobalVar(CurrentMenu.GIndex);		// Get original from EEROM.

				SystemCommand = IDLE_CMD;	// Request that we idle so user can modify value.
				// If a user hits enter the new value is stored into EEROM, but if they press ESC
				// the original that we loaded back from EEROM is redisplayed and the value remains
				// unchanged.
				ELSKeys_State = ELSKEYS_KEY_DONE;
				break;

			  case KEY_SET_END :
				SetDisplayMode(KEY_MENU_MODE);		// Disable run time display and allow menus.
				INTCON &= 0x3F;
				mtr = ZMotorPosition;
				INTCON |= 0xC0;
				res = StepsToZDistance( mtr );	// Convert to distance in either mm or inch.
				res = (fMetricMode) ? res / 25.4 : res;		// If mm then covert to inch.
				SetGlobalVarFloat(THREAD_END_NDX, res );	// Save motor position in imperial units.
				LastMenuNumber = 1;
				CurrentMenuNumber = 0x1A;		// End Position Menu.
				KeyNumberIndex = GetMenu(CurrentMenuNumber, &CurrentMenu, KeyNumberBuffer);
				if (KeyNumberIndex) {
					fNumberKeys = 1;
					KeyMaxLength = KeyNumberIndex;
				}
				else
					fNumberKeys = 0;

				// Since we had to set the global memory value to the current position in order
				// to potentially accept (ENTER) or reject (ESC) the value.  We'd better put the 
				// original back just in case it's rejected.
				LoadGlobalVar(CurrentMenu.GIndex);		// Get original from EEROM.

				SystemCommand = IDLE_CMD;	// Request that we idle so user can modify value.
				// If a user hits enter the new value is stored into EEROM, but if they press ESC
				// the original that we loaded back from EEROM is redisplayed and the value remains
				// unchanged.
				ELSKeys_State = ELSKEYS_KEY_DONE;
				break;

			  /*
			   * Double function button.  
			   *	If ALT key held down slew the carriage to 0.00 which is HOME position.
			   *	Otherwise just clear the current Z axis position to 0
			  */
			  case KEY_SET_ZHOME :
				if ((KeyValue & KEY_ALT) == 0) {	// No alt key so set ZMotorPosition to 0.
					SetDisplayMode(ADJUST_MODE);
					INTCON &= 0x3F;
					ZMotorPosition = 0;
					INTCON |= 0xC0;
					SetGlobalVarFloat(THREAD_Z_HOME_NDX, 0.0 );
					ELSKeys_State = ELSKEYS_KEY_DONE;
				}
				else  {								// ALT key pressed so start automatic move to 0.000
					SetDisplayMode(RUN_MODE);
					SystemCommand = HOME_CMD;
					MoveHome(MOTOR_Z, (WORD)GetGlobalVarWord(SLEW_RATE_Z_NDX)); 
					ELSKeys_State = ELSKEYS_KEY_DONE;
				}
				break;

			  /*
			   * Double function button.  
			   *	If ALT key held down slew the cross slide to 0.00 which is HOME position.
			   *	Otherwise just clear the current X axis position to 0
			  */
		  	  case KEY_SET_XHOME : 
				if ((KeyValue & KEY_ALT) == 0) {	// No alt key so set XMotorPosition to 0.
					SetDisplayMode(ADJUST_MODE);
					INTCON &= 0x3F;
					XMotorRelPosition = 0;		
					INTCON |= 0xC0;
					ELSKeys_State = ELSKEYS_KEY_DONE;
				}
				else {
					SetDisplayMode(RUN_MODE);
					SystemCommand = HOME_CMD;
					MoveHome(MOTOR_X, (WORD)GetGlobalVarWord(SLEW_RATE_X_NDX)); 
					ELSKeys_State = ELSKEYS_KEY_DONE;
				}
				break;

			  /*
			   * When pressed sets the machine into ready mode and recalculates all the threading 
			   * parameters.
			   */
			  case KEY_THREAD :
				SetThreadingMode();
				ELSKeys_State = ELSKEYS_KEY_DONE;
				break;

			  /*
			   * When pressed sets the machine into ready mode and recalculates all the turning
			   * parameters.
			   */
			  case KEY_TURN :
				SetTurningMode();
				ELSKeys_State = ELSKEYS_KEY_DONE;
				break;

			  /*
			   * In Ready mode the function buttons have several capabilities depending on which
			   * menu is currently displayed.  These menus are accessed from the root and are called
			   * INFO1_MODE to INFO2_MODE.  One other menu is brought up by pressing the MPG knob. 
			   *	
			  */
			  case KEY_SFN1 :	
				if (fRunTimeDisplay) {
					if (DisplayModeMenuIndex == INFO3_MODE)	{  // "PASS COUNT = %2d     CNCL  RST   SET  CLR"
						SetDisplayMode(KEY_MENU_MODE);
						LastMenuNumber = 1;				// Show RUN SETUP menu on ESC key.
						CurrentMenuNumber = 0x45;		// " First Pass Depth   Depth        0.005" ",
						KeyNumberIndex = GetMenu(CurrentMenuNumber, &CurrentMenu, KeyNumberBuffer);
						if (KeyNumberIndex) {
							fNumberKeys = 1;
							KeyMaxLength = KeyNumberIndex;
						}
						else
							fNumberKeys = 0;
						fXThreading = 0;			// Cancel threading so next time new defaults are loaded.
						SystemCommand = IDLE_CMD;	// Request that we idle so user can modify value.
					}
					else if (DisplayModeMenuIndex == INFO4_MODE){	// " TAPER MODE OFF     ENABLE SET   DISABLE"
						// Now we could just set the flag and save the byte it's in but then if we move 
						// the flag to another location we'd have to change multple places that refer to the byte
						// that holds the flag.
						// We still do refer to the byte when we use fTapering in the code but there's only 
						// one way to set or clear the flag and that's through the menu.  A bit more robust.
						GetMenu(0x41, &CurrentMenu, KeyNumberBuffer);
						CurrentMenu.Data.Next[0] = 1;				// Turn on flag.
						// fTapering = 1;							// " TAPER MODE ON      ENABLE SET   DISABLE"
						SaveFlagVar(&CurrentMenu);					// Save Flag to EEROM
						ReadFlags();  // Now update RAM images of EEROM flags.	
					}
					else if (DisplayModeMenuIndex == MSG_CHANGE_JOG_DISTANCE_MODE) {
						RestoreCurrentJog();						// Set original Jog Distance.
						SetDisplayMode(ADJUST_MODE);
					}
					else
						SetDisplayMode(INFO1_MODE);					// Show Z Start/End position.
				}
				ELSKeys_State = ELSKEYS_KEY_DONE;
				break;

			  case KEY_SFN2 :	// Go Middle Left
				if (fRunTimeDisplay) {
					// If already in INFO3_MODE Reset and recalculate.
					if (DisplayModeMenuIndex == INFO3_MODE)	{ // "PASS COUNT = %2d     CNCL  RST   SET  CLR"
						InitMovementThread();				  // RST and recalculate thread parameters.
						CalculatePasses();					
					}
					// If in INFO4_MODE (Tapering) bring in SET menu...
					else if (DisplayModeMenuIndex == INFO4_MODE){  // " TAPER MODE xxx     ENABLE SET   DISABLE"
						SetDisplayMode(KEY_MENU_MODE);
						LastMenuNumber = 1;				// Show RUN SETUP menu on ESC key.
						CurrentMenuNumber = 0x25;		// Taper Choices Menu
						KeyNumberIndex = GetMenu(CurrentMenuNumber, &CurrentMenu, KeyNumberBuffer);
						if (KeyNumberIndex) {
							fNumberKeys = 1;
							KeyMaxLength = KeyNumberIndex;
						}
						else
							fNumberKeys = 0;
						SystemCommand = IDLE_CMD;	// Request that we idle so user can modify value.
					}
					// If MPG knob was pressed...
					else if (DisplayModeMenuIndex == MSG_CHANGE_JOG_DISTANCE_MODE) {
						ChangeJogMotor(fMetricMode);	// Toggle between X or Z controlled by MPG.
					}
					else
						SetDisplayMode(INFO2_MODE);		// Show X Start and X Retracted positions.
				}
				ELSKeys_State = ELSKEYS_KEY_DONE;
				break;

			  case KEY_SFN3 :	// Go Middle Right
				if (fRunTimeDisplay) {
					// If we're already into INFO3_MODE then descend into SET Thread parameter menu.
					if (DisplayModeMenuIndex == INFO3_MODE)	{  // "PASS COUNT = %2d     CNCL  RST   SET  CLR"
						SetDisplayMode(KEY_MENU_MODE);
						LastMenuNumber = 1;				// Show RUN SETUP menu on ESC key.
						CurrentMenuNumber = 0x34;		// " THREAD PARAMETERS  PTCH DPTH ANGL PASS ",
						KeyNumberIndex = GetMenu(CurrentMenuNumber, &CurrentMenu, KeyNumberBuffer);
						if (KeyNumberIndex) {
							fNumberKeys = 1;
							KeyMaxLength = KeyNumberIndex;
						}
						else
							fNumberKeys = 0;
						fXThreading = 0;			// Cancel threading so next time new defaults are loaded.
						SystemCommand = IDLE_CMD;	// Request that we idle so user can modify value.
					}
					else	// Show Threading parameters.
						SetDisplayMode(INFO3_MODE); // "PASS COUNT = %2d     CNCL  RST   SET  CLR"
				}
				ELSKeys_State = ELSKEYS_KEY_DONE;
				break;

			  case KEY_SFN4 :	// Go right
				if (fRunTimeDisplay) {
					// If we're already into INFO3_MODE reset the threading parameters back to initial values.
					if (DisplayModeMenuIndex == INFO3_MODE)	{  // "PASS COUNT = %2d     CNCL  RST   SET  CLR"
						// Reset Pass count and initial Z and X start locations.
						PassCount = 0;
						SpringPassCount = 0;
						
						ZBeginPosition = (fMetricMode) ? GetGlobalVarFloat(THREAD_BEGIN_NDX) * 25.4 : GetGlobalVarFloat(THREAD_BEGIN_NDX);
						ZBeginPositionSteps = ZDistanceToSteps(ZBeginPosition);
						XBeginPosition = GetGlobalVarFloat(BEGIN_X_NDX);
						XBeginPositionSteps = XDistanceToSteps(XBeginPosition);
					}
					else if (DisplayModeMenuIndex == INFO4_MODE){   // " TAPER MODE ON      ENABLE SET   DISABLE"
						GetMenu(0x41, &CurrentMenu, KeyNumberBuffer);		// Get the taper flag menu.
						CurrentMenu.Data.Next[0] = 0;						// Turn off flag.
						SaveFlagVar(&CurrentMenu);							// Save Flag to EEROM
						// fTapering = 0; // Disable Tapering Mode.	// " TAPER MODE OFF     ENABLE SET   DISABLE"
						ReadFlags();  // Now update RAM images of EEROM flags.	


					}
					else if (DisplayModeMenuIndex == MSG_CHANGE_JOG_DISTANCE_MODE) {
						ChangeJogDistance(fMetricMode);
						SetDisplayMode(ADJUST_MODE);					
					}
					else
						SetDisplayMode(INFO4_MODE);
				}
				ELSKeys_State = ELSKEYS_KEY_DONE;
				break;

			  /*
			   * ESCAPE Key.  
			   *	Move from READY mode into IDLE mode to be able to set parameters.
			   *	It's done by sending an IDLE_CMD to the main line state machine.
			  */
			  case KEY_ESC :	// Go up.
				SystemError = 0;
				DEBUG_MENU("CurrentMenu#=%d\n", CurrentMenuNumber);

				if ((signed int8)CurrentMenuNumber > SIGNON_MENU_INDEX) {
					SetDisplayMode(KEY_MENU_MODE);
					CurrentMenuNumber = LastMenuNumber;
				}
				else {
					CurrentMenuNumber = RUN_SETUP_MENU_INDEX;
				}	
				LastMenuNumber = 0;	// Next time go home.
				SystemCommand = IDLE_CMD;

				fNumberKeys = GetMenu(CurrentMenuNumber, &CurrentMenu, KeyNumberBuffer);
				ELSKeys_State = ELSKEYS_KEY_DONE;
				break;

			  /*
			   * 
			  */
			  case KEY_ENTER :	// Save Result and go up.
				if (CurrentMenuNumber != 0) {
				  	switch (CurrentMenu.Format) {
					  case BYTE_TYPE :	// uint8
						CurrentMenu.Data.LongValue = SGetWord((int8 *)KeyNumberBuffer);
						GlobalVars[CurrentMenu.GIndex].l = CurrentMenu.Data.LongValue ;
						SaveGlobalVar(CurrentMenu.GIndex);
						KeyNumberIndex = GetMenu(CurrentMenuNumber, &CurrentMenu, KeyNumberBuffer);
						break;
			
					  case INT_TYPE :	// WORD
						CurrentMenu.Data.LongValue = SGetWord((int8 *)KeyNumberBuffer);
						GlobalVars[CurrentMenu.GIndex].l = CurrentMenu.Data.LongValue; 
						SaveGlobalVar(CurrentMenu.GIndex);
						KeyNumberIndex = GetMenu(CurrentMenuNumber, &CurrentMenu, KeyNumberBuffer);
						break;
			
					  case WORD_TYPE :	// WORD
						CurrentMenu.Data.LongValue = SGetWord((int8 *)KeyNumberBuffer);
						GlobalVars[CurrentMenu.GIndex].l = CurrentMenu.Data.LongValue; 
						SaveGlobalVar(CurrentMenu.GIndex);
						KeyNumberIndex = GetMenu(CurrentMenuNumber, &CurrentMenu, KeyNumberBuffer);
						break;
			
					  case LONG_TYPE :	// LONG
						CurrentMenu.Data.LongValue = SGetLong((int8 *)KeyNumberBuffer);
						GlobalVars[CurrentMenu.GIndex].l = CurrentMenu.Data.LongValue; 
						SaveGlobalVar(CurrentMenu.GIndex);
						KeyNumberIndex = GetMenu(CurrentMenuNumber, &CurrentMenu, KeyNumberBuffer);
						break;
			
					  case ULONG_TYPE :	// ULONG
						CurrentMenu.Data.LongValue = SGetULong((int8 *)KeyNumberBuffer);
						GlobalVars[CurrentMenu.GIndex].l = CurrentMenu.Data.LongValue; 
						SaveGlobalVar(CurrentMenu.GIndex);
						KeyNumberIndex = GetMenu(CurrentMenuNumber, &CurrentMenu, KeyNumberBuffer);
						break;
			
		  		      case FLOAT_TYPE :	// FLOAT
						AsciiTofloat(&res, (int8 *)KeyNumberBuffer);
						CurrentMenu.Data.FloatValue = res;  // round up.
						CurrentMenu.WriteValue(&CurrentMenu);
						GlobalVars[CurrentMenu.GIndex].f = CurrentMenu.Data.FloatValue / CurrentMenu.Conversion;
						SaveGlobalVar(CurrentMenu.GIndex);
						KeyNumberIndex = GetMenu(CurrentMenuNumber, &CurrentMenu, KeyNumberBuffer);
						break;
					}
					BeepMsg = T_MENU_SAVED | TWO_BEEPS;
				}	
				ELSKeys_State = ELSKEYS_KEY_DONE;
				break;

			  default :
				BeepMsg = T_BAD_KEY | FOUR_BEEPS;
				break;
			}
		}
	    break;

	  /*
	   * After timer expires 
	   * check if the key has already been released. 
	   * If so then just perform the tap function,
	   * otherwise perform the pressed function.
	   */
	  case ELSKEYS_KEY_TAPPED :

			if (KeyValue & KEY_HELD_DOWN) { // Key still pressed

			  	switch( KeyValue & KEY_MASK ) {  // See if it's ours.

				  case KEY_RZJOG : // Turn right
					if (!fSwitchDown) {  // As int32 as switch isn't already pressed.
						fSwitchDown = 1;
						MotorJog(MOTOR_Z, 0, MOVE_RIGHT);  // Jog at feed rate.
					}
					break;

				  case KEY_LZJOG : // Turn left
					if (!fSwitchDown) {
						fSwitchDown = 1;
						MotorJog(MOTOR_Z, 0, MOVE_LEFT);  // Jog at feed rate.
					}
					break;

				  case KEY_XJOG_IN : // Jog inwards
					if (!fSwitchDown) {  // As int32 as switch isn't already pressed.
						fSwitchDown = 1;
						MotorJog(MOTOR_X, (WORD)GetGlobalVarWord(MOVE_RATE_X_NDX), MOVE_IN);
					}
					break;

				  case KEY_XJOG_OUT : // Jog Outwards
					if (!fSwitchDown) {
						fSwitchDown = 1;
						MotorJog(MOTOR_X, (WORD)GetGlobalVarWord(MOVE_RATE_X_NDX), MOVE_OUT);
					}
					break;

				  case KEY_STOP :	// Stop in middle of move if STOP key held down.
					SystemCommand = STOP_CMD;
					fSwitchDown = 1;
					break;
				}
				ELSKeys_State = ELSKEYS_KEY_HOLD;
			}
			else if ((KeyValue & KEY_MASK)== KEY_RELEASED)  {	// Key no longer pressed.
				if (fSwitchDown) { // If a motor was turning... Stop it.
					fSwitchDown = 0;
					MotorStop(MOTOR_Z);				// So stop motor
					MotorStop(MOTOR_X);
				}
				// When key is released change state.
				ELSKeys_State = ELSKEYS_KEY_DONE;
			}		
		break;

	  /*
	   * Now wait for the key to be released and perform any
	   * post processing.
	   */
	  case ELSKEYS_KEY_HOLD :
	  	if ((KeyValue & KEY_MASK) == KEY_RELEASED) { // no switch

			if (fSwitchDown) {
				fSwitchDown = 0;
				MotorStop(MOTOR_Z);				// So stop motor
				MotorStop(MOTOR_X);
			}

			fKeyRdy = 1; 
			ELSKeys_State = ELSKEYS_KEY_WAIT;
		}
		break;

	  /*
	   * Now wait for the key to be released
	   */
	  case ELSKEYS_KEY_DONE :
	  	if ((KeyValue & KEY_MASK) == KEY_RELEASED) { // no switch
			fKeyRdy = 1; 
			ELSKeys_State = ELSKEYS_KEY_WAIT;
		}
		else if ((KeyValue & KEY_HELD_DOWN) == KEY_HELD_DOWN) {
			fKeyRdy = 1; 
			ELSKeys_State = ELSKEYS_KEY_WAIT;
		}
		break;

	  default :
			fKeyRdy = 1; 
			ELSKeys_State = ELSKEYS_KEY_WAIT;
		break;
	}
}

/*
 *  FUNCTION: IdleKeyThread
 *
 *  PARAMETERS:			None.
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:		When the ESC key is pressed the ELS enters IDLE mode which 
 *						turns off the flashing LED run light.
 *						The Idle thread prevents motor motion by only allowing access
 *						to the editing keys.
 *
 *  RETURNS: 			Nothing
 *
 */
void
IdleKeyThread(void) {
  float32 res;
  int8 ndx;
  TMENU_LIST rom * listptr;

	switch (ELSKeys_State) {

	  /*
	   * Initialize ELS Key pad state machine
	   */
	  case ELSKEYS_INIT :
		fKeyRdy = 1;
		ELSKeys_State = ELSKEYS_KEY_WAIT;
		break;

	  /*
	   * Wait for a key tap or key press.
	   * We save the key and then check it a bit later 
	   * to see if it's still pressed or was just tapped.
	   * If it's not a tap enabled key we just do what's needed
	   * right away.
	   */
	  case ELSKEYS_KEY_WAIT :
		
		if (KeyValue & KEY_PRESSED) { // Key pressed

			KeyButton = KeyValue;
			StartTimer( ELSKEYS_TIMER, T_KEY_DOWN ); 
			fKeyRdy = 0; 
			ELSKeys_State = ELSKEYS_KEY_TAPPED;

			if (CurrentMenuNumber == -1) {  // Startup or Running with status display.
				if (!fRunTimeDisplay) {		// Must have been Startup.
					GetMenu(1, &CurrentMenu, KeyNumberBuffer);	// Get Configuration Menu.
					LastMenuNumber = 0;
					CurrentMenuNumber = 1;
				}
				else
					CurrentMenuNumber = 1;
				fRunTimeDisplay = 0;
			}
			maskedKey = KeyValue & 0x3F;

		  	switch( maskedKey ) {
			  /*
			   * The digit keys for entering numeric parameters	
			   */
			  case KEY_0 :
			  case KEY_1 :
			  case KEY_2 :
			  case KEY_3 :
			  case KEY_4 :
			  case KEY_5 :
			  case KEY_6 :
			  case KEY_7 :
			  case KEY_8 :
			  case KEY_9 :
				if (fNumberKeys && (KeyNumberIndex < KeyMaxLength)) {
					KeyNumberBuffer[KeyNumberIndex++] = maskedKey;
					KeyNumberBuffer[KeyNumberIndex] = '\0';
					if (KeyNumberIndex > KeyMaxLength) {	// At end of buffer.
						KeyNumberIndex--;
						LCDSendChar('\b');	// erase old one.
						LCDSendChar(maskedKey);	// print new one.
					}
					else
						LCDSendChar(maskedKey);
				}
				ELSKeys_State = ELSKEYS_KEY_DONE;
				break;

			  /*
			   * Used to enter a decimal point for a floating point # or to toggle a boolean flag entry	
			   */
			  case KEY_PERIOD :
				if (CurrentMenu.Format == FLAG_TYPE) {
					// Use the FLAG_TYPE menu information to determine where and what to print for 0/1 states
					LCDSetPosition(CurrentMenu.UnitsPos,1);		// Point to where text goes on line 1

					if (KeyNumberBuffer[0]) {	// Test flag status
						KeyNumberBuffer[0] = 0;	// toggle it.
						// Display appropriate message.
						LCDSendStr( FlagText[CurrentMenu.Dependancy].ptrOff_text );
					}
					else {
						KeyNumberBuffer[0] = 1;
						// Display appropriate message.
						LCDSendStr( FlagText[CurrentMenu.Dependancy].ptrOn_text );
					}
				}
				else if (fNumberKeys && !fPeriodEntered && (KeyNumberIndex < KeyMaxLength)) {
					fPeriodEntered = 1;
					LCDSendChar(maskedKey);
					KeyNumberBuffer[KeyNumberIndex++] = maskedKey;
					KeyNumberBuffer[KeyNumberIndex] = '\0';
					if (KeyNumberIndex >= KeyMaxLength)
						KeyNumberIndex--;
				}
				ELSKeys_State = ELSKEYS_KEY_DONE;
				break;
				
			  /*
			   * Delete character to left of cursor except when field is completely filled.
			   * When completely filled, delete the entire field so user can enter a new number.
			   *
			   */
			  case KEY_DEL :
				if (fNumberKeys && (KeyNumberIndex > 0) && !fListFlag) {
					if ( KeyNumberBuffer[KeyNumberIndex-1] == KEY_PERIOD )
						fPeriodEntered = 0;
					if (KeyNumberIndex == KeyMaxLength) {
						while (KeyNumberIndex-- > 0)
							LCDSendChar('\b');
						KeyNumberBuffer[++KeyNumberIndex] = '\0';
						fPeriodEntered = 0;
					}
					else {
						KeyNumberBuffer[--KeyNumberIndex] = '\0';
						LCDSendChar('\b');
					}
				}
				else if (fListFlag) {
					BeepMsg = T_BAD_KEY | FOUR_BEEPS;
				}
				ELSKeys_State = ELSKEYS_KEY_DONE;
				break; 
					
			   /*
			   * Scroll upwards through list or
			   * The - key for numeric input or
			   * Turns LCD backlight On
			   */
			  case KEY_UP : // or '-' if fNumberKeys
				if (fScrollFlags) {
					CurrentMenuNumber = CurrentMenu.Data.Next[1];
					KeyNumberIndex = GetMenu(CurrentMenuNumber, &CurrentMenu, KeyNumberBuffer);
				}
				else if (fListFlag) {
					// Get List Index into 8 bit variable.
					ndx = GlobalVars[CurrentMenu.GIndex].b;
					// Decrement and test if we've dropped off start of list.
					if (--ndx < 0) {
						ndx = CurrentMenu.UnitsPos-1;	// Wrap to end of list.
					}
					// Restore new list index into menu.
					GlobalVars[CurrentMenu.GIndex].b = ndx; 
					// reload menu which has upgraded index and will show new data.
					KeyNumberIndex = GetMenu(CurrentMenuNumber, &CurrentMenu, KeyNumberBuffer);
				}
				else if (fNumberKeys && (KeyNumberIndex == 0)) {
					KeyNumberBuffer[KeyNumberIndex++] = '-';
					LCDSendChar('-');
				}
//				else 
//					bLCD_BACK_Light	= 1;

				ELSKeys_State = ELSKEYS_KEY_DONE;
				break;

			  /*
			   * Scrolls down through list or turns LCD backlight off.
			  */
			  case KEY_DOWN :
				if (fScrollFlags) {
					CurrentMenuNumber = CurrentMenu.Data.Next[0];
					KeyNumberIndex = GetMenu(CurrentMenuNumber, &CurrentMenu, KeyNumberBuffer);
				}
				else if (fListFlag) {
					// Get List Index into 8 bit variable.
					ndx = GlobalVars[CurrentMenu.GIndex].b;
					// Increment and test against list size.
					if (++ndx >= CurrentMenu.UnitsPos) {
						ndx = 0;	// wrap around to beginning of list if needed.
					}
					// restore list index into menu.
					GlobalVars[CurrentMenu.GIndex].b = ndx; 
					// Now that index is upgraded reload menu which will show new data.
					KeyNumberIndex = GetMenu(CurrentMenuNumber, &CurrentMenu, KeyNumberBuffer);
				}
				else if (!fNumberKeys)
					ToggleBacklight();
					//bLCD_BACK_Light	= 0;
				ELSKeys_State = ELSKEYS_KEY_DONE;
				break;

			  /*
			   * When pressed sets the machine into ready mode and recalculates all the threading 
			   * parameters.
			   */
			  case KEY_THREAD :
				SetThreadingMode();
				SystemCommand = RDY_CMD;
				ELSKeys_State = ELSKEYS_KEY_DONE;
				break;

			  /*
			   * When pressed sets the machine into ready mode and recalculates all the turning 
			   * parameters.
			   */
			  case KEY_TURN :
				SetTurningMode();
				CurrentMenuNumber = 1;
				LastMenuNumber = 0;	// Next time go home.
				SystemCommand = RDY_CMD;
				ELSKeys_State = ELSKEYS_KEY_DONE;
				break;


			  /*
			   * There are four programmable function keys lined up under the LCD display.
			   * The keys behave differently depending on the mode of the machine.
			   * In Ready mode the four buttons bring up status information or are quick links into
			   * setup menus for threading or tapering.
			   * Each menu can have up to 4 choices or a single parameter.  The choices tend to be
			   * 3 to 5 letter acronyms and are always upper case characters.  Parameters are described
			   * with upper and lower case letters.
			   *
			   *	SFN-1	Far left button.  
			   *		Fetch a menu and if the result of GetMenu is non-zero (field length), 
			   *		then it's a parameter. So then number keys are enabled and the field length 
			   *		set to KeyMaxLength
			   *		
			   */
			  case KEY_SFN1 :	// Go Left
				if (CurrentMenu.GIndex == 0) {
					LastMenuNumber = CurrentMenuNumber;
					CurrentMenuNumber = CurrentMenu.Data.Next[0];
					KeyNumberIndex = GetMenu(CurrentMenuNumber, &CurrentMenu, KeyNumberBuffer);
					if (KeyNumberIndex) {
						fNumberKeys = 1;
						KeyMaxLength = KeyNumberIndex;
					}
					else
						fNumberKeys = 0;
				}
				ELSKeys_State = ELSKEYS_KEY_DONE;
				break;

			   /*
			   *	SFN-2	Mid left button.  
			   *		Fetch a menu and if the result of GetMenu is non-zero (field length), 
			   *		then it's a parameter. So then number keys are enabled and the field length 
			   *		set to KeyMaxLength
			   *		
			   */
			  case KEY_SFN2 :	// Go Middle Left
				if (CurrentMenu.GIndex == 0) {
					LastMenuNumber = CurrentMenuNumber;
					CurrentMenuNumber = CurrentMenu.Data.Next[1];
					KeyNumberIndex = GetMenu(CurrentMenuNumber, &CurrentMenu, KeyNumberBuffer);
					if (KeyNumberIndex) {
						fNumberKeys = 1;
						KeyMaxLength = KeyNumberIndex;
					}
					else
						fNumberKeys = 0;
				}	
				ELSKeys_State = ELSKEYS_KEY_DONE;
				break;

			   /*
			   *	SFN-3	Mid right button.  
			   *		Fetch a menu and if the result of GetMenu is non-zero (field length), 
			   *		then it's a parameter. So then number keys are enabled and the field length 
			   *		set to KeyMaxLength
			   *		
			   */
			  case KEY_SFN3 :	// Go Middle Right
				if (CurrentMenu.GIndex == 0) {
					LastMenuNumber = CurrentMenuNumber;
					CurrentMenuNumber = CurrentMenu.Data.Next[2];
					KeyNumberIndex = GetMenu(CurrentMenuNumber, &CurrentMenu, KeyNumberBuffer);
					if (KeyNumberIndex) {
						fNumberKeys = 1;
						KeyMaxLength = KeyNumberIndex;
					}
					else
						fNumberKeys = 0;
				}	
				ELSKeys_State = ELSKEYS_KEY_DONE;
				break;

			   /*
			   *	SFN-4	Far left button.  
			   *		Fetch a menu and if the result of GetMenu is non-zero (field length), 
			   *		then it's a parameter. So then number keys are enabled and the field length 
			   *		set to KeyMaxLength
			   *		
			   */
			  case KEY_SFN4 :	// Go right
				if (CurrentMenu.GIndex == 0) {
					LastMenuNumber = CurrentMenuNumber;
					CurrentMenuNumber = CurrentMenu.Data.Next[3];
					KeyNumberIndex = GetMenu(CurrentMenuNumber, &CurrentMenu, KeyNumberBuffer);
					if (KeyNumberIndex) {
						fNumberKeys = 1;
						KeyMaxLength = KeyNumberIndex;
					}
					else
						fNumberKeys = 0;
				}	
				ELSKeys_State = ELSKEYS_KEY_DONE;
				break;

			  /*
			   * ESCAPE KEY
			   *	Clear System Error
			   *	Cancel Scrolling through menu flags.
			   *	Cancel scrolling through list of string/number pairs like tapers.
			   * 	Return to previous menu and then set up to return to top menu on next
			   *	escape key.
			   */
			  case KEY_ESC :	// Go up.
				SystemError = 0;
				fScrollFlags = 0;	// Cancel scrolling through linked list of Menu flags.
				fListFlag = 0;		// Cancel Scrolling trhough list of string,number pairs.
				if (CurrentMenuNumber > 0) {
					CurrentMenuNumber = LastMenuNumber;
					LastMenuNumber = 1;	// Next time go home.
				}
				else {
					CurrentMenuNumber = 1;
					LastMenuNumber = 0;	// Next time go home.
				}	
				fNumberKeys = GetMenu(CurrentMenuNumber, &CurrentMenu, KeyNumberBuffer);
				ELSKeys_State = ELSKEYS_KEY_DONE;
				break;

			   /*
			   *	ENTER KEY
			   *		Based on the type of variable that the menu represents, parse the 
			   *		input buffer string into a number and then save it in the appropriate
			   *		global variable as indexed by the menu field GIndex.
			   *		Then fetch the current menu which redisplays the new value in the correct
			   *		format.
			   *		On sucessful save beep twice.
			   */
			  case KEY_ENTER :	// Save Result and go up.
				if (CurrentMenuNumber != 0) {
				  	switch (CurrentMenu.Format) {
					  case BYTE_TYPE :	// uint8
						CurrentMenu.Data.LongValue = SGetWord((int8 *)KeyNumberBuffer);
						GlobalVars[CurrentMenu.GIndex].l = CurrentMenu.Data.LongValue ;
						SaveGlobalVar(CurrentMenu.GIndex);
						KeyNumberIndex = GetMenu(CurrentMenuNumber, &CurrentMenu, KeyNumberBuffer);
						break;
			
					  case INT_TYPE :	// WORD
						CurrentMenu.Data.LongValue = SGetWord((int8 *)KeyNumberBuffer);
						GlobalVars[CurrentMenu.GIndex].l = CurrentMenu.Data.LongValue; 
						SaveGlobalVar(CurrentMenu.GIndex);
						KeyNumberIndex = GetMenu(CurrentMenuNumber, &CurrentMenu, KeyNumberBuffer);
						break;
			
					  case WORD_TYPE :	// WORD
						CurrentMenu.Data.LongValue = SGetWord((int8 *)KeyNumberBuffer);
						GlobalVars[CurrentMenu.GIndex].l = CurrentMenu.Data.LongValue; 
						SaveGlobalVar(CurrentMenu.GIndex);
						KeyNumberIndex = GetMenu(CurrentMenuNumber, &CurrentMenu, KeyNumberBuffer);
						break;
			
					  case LONG_TYPE :	// LONG
						CurrentMenu.Data.LongValue = SGetLong((int8 *)KeyNumberBuffer);
						GlobalVars[CurrentMenu.GIndex].l = CurrentMenu.Data.LongValue; 
						SaveGlobalVar(CurrentMenu.GIndex);
						KeyNumberIndex = GetMenu(CurrentMenuNumber, &CurrentMenu, KeyNumberBuffer);
						break;
			
					  case ULONG_TYPE :	// ULONG
						CurrentMenu.Data.LongValue = SGetULong((int8 *)KeyNumberBuffer);
						GlobalVars[CurrentMenu.GIndex].l = CurrentMenu.Data.LongValue; 
						SaveGlobalVar(CurrentMenu.GIndex);
						KeyNumberIndex = GetMenu(CurrentMenuNumber, &CurrentMenu, KeyNumberBuffer);
						break;
			
		  		      case FLOAT_TYPE :	// FLOAT
						AsciiTofloat(&res, (int8 *)KeyNumberBuffer);
						CurrentMenu.Data.FloatValue = res;  // round up.
						CurrentMenu.WriteValue(&CurrentMenu);
						GlobalVars[CurrentMenu.GIndex].f = CurrentMenu.Data.FloatValue / CurrentMenu.Conversion;
						SaveGlobalVar(CurrentMenu.GIndex);
						KeyNumberIndex = GetMenu(CurrentMenuNumber, &CurrentMenu, KeyNumberBuffer);
						break;

					  case LIST_TYPE :
						// Save which menu list item we've chosen.
						SaveGlobalVar(CurrentMenu.GIndex);
						// Then get menu list float32 value.
						// Point to array of labels and data.
						listptr = (TMENU_LIST rom * )CurrentMenu.Data.LongValue;
						// Then use that to point to our specific entry as index by GIndex.
						listptr = (TMENU_LIST rom * )&listptr[GlobalVars[CurrentMenu.GIndex].b];
						// Update menu list dependant entry converting as needed.
						GlobalVars[CurrentMenu.Dependancy].f = listptr->Data / CurrentMenu.Conversion;
						// and save menu list dependant entry.
						SaveGlobalVar(CurrentMenu.Dependancy);
						break;

					  case FLAG_TYPE : // Boolean Flag
						// Save new flag value in data structure.
						CurrentMenu.Data.Next[0] = KeyNumberBuffer[0];
						// Update flags into EEROM'd global system flag variables
						SaveFlagVar(&CurrentMenu);	
						// Then read them back to make sure they took.
						ReadFlags();  // Now update RAM images of EEROM flags.
						// Finally redisplay the menu with the updated RAM image.
						// If it's not right the wrong flag will be shown.  That would be a bad thing.
						KeyNumberIndex = GetMenu(CurrentMenuNumber, &CurrentMenu, KeyNumberBuffer);
						// Now based on EEROM value of flag call FLAG function.
						if (KeyNumberBuffer[0])
							CurrentMenu.WriteValue(&CurrentMenu);	// Flag set to 1, call WriteValueFunction.
						else
							CurrentMenu.ReadValue(&CurrentMenu); // Flag set to 0, call ReadValueFunction
						break;
					}
					BeepMsg = T_MENU_SAVED | TWO_BEEPS;
				}	
				ELSKeys_State = ELSKEYS_KEY_DONE;
				break;

			  default :
				BeepMsg = T_BAD_KEY | FOUR_BEEPS;
				break;
			}
		}
	    break;

	  /*
	   * After timer expires 
	   * check if the key has already been released. 
	   * If so then just perform the tap function,
	   * otherwise perform the pressed function.
	   */
	  case ELSKEYS_KEY_TAPPED :

		if (TimerDone(ELSKEYS_TIMER)) {
			if (KeyValue == KeyButton) { // Key still pressed
				ELSKeys_State = ELSKEYS_KEY_HOLD;
			}
			else {
				if (fSwitchDown) { 
					fSwitchDown = 0;
				}
				ELSKeys_State = ELSKEYS_KEY_DONE;
			}		

		}
		break;

	  /*
	   * Now wait for the key to be released and perform any
	   * post processing.
	   */
	  case ELSKEYS_KEY_HOLD :
	  	if ((KeyValue & KEY_MASK) == KEY_RELEASED) { // no switch

			if (fSwitchDown) {
				fSwitchDown = 0;
			}

			fKeyRdy = 1; 
			ELSKeys_State = ELSKEYS_KEY_WAIT;
		}
		break;

	  /*
	   * Now wait for the key to be released
	   */
	  case ELSKEYS_KEY_DONE :
	  	if ((KeyValue & KEY_MASK) == KEY_RELEASED) { // no switch
			fKeyRdy = 1; 
			ELSKeys_State = ELSKEYS_KEY_WAIT;
		}
		else if ((KeyValue & KEY_HELD_DOWN) == KEY_HELD_DOWN) {
			fKeyRdy = 1; 
			ELSKeys_State = ELSKEYS_KEY_WAIT;
		}
		break;

	  default :
			ELSKeys_State = ELSKEYS_KEY_WAIT;
			fKeyRdy = 1; 
		break;
	}
}

/*
 *  FUNCTION: ErrorKeyThread
 *
 *  PARAMETERS:			None
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:		Called when a system error occurs.
 *						Errors like ESTOP, LIMIT, Motor control issues.
 *						ESC cancels the error.  Editing is not possible 
 *						in the ERROR state so it's not possible to flip
 *						the ESTOP or LIMIT invert flags.  i.e. The error 
 *						must be cleared first rather than worked around.
 *
 *  RETURNS: 			Nothing.
 *
 */
void
ErrorKeyThread(void) {
  
	switch (ELSKeys_State) {

	  /*
	   * Initialize ELS Key pad state machine
	   */
	  case ELSKEYS_INIT :
		ELSKeys_State = ELSKEYS_KEY_WAIT;
		break;

	  /*
	   * Wait for a key press.
	   */
	  case ELSKEYS_KEY_WAIT :

		if (KeyValue & KEY_PRESSED) { // Key pressed

			ELSKeys_State = ELSKEYS_KEY_DONE;  // Most cases we go here after a key press.

		  	switch( KeyValue & 0x3F ) {

			  case KEY_UP : 
//				bLCD_BACK_Light	= 1;
				break;

			  case KEY_DOWN :
				ToggleBacklight();
//				bLCD_BACK_Light	= 0;
				break;

			  case KEY_ESC :	// Go up.
				SystemError = 0;
				fScrollFlags = 0;	// Cancel scrolling through linked list of Menu flags.
				fListFlag = 0;		// Cancel Scrolling trhough list of string,number pairs.
				if (CurrentMenuNumber > 0) {
					CurrentMenuNumber = LastMenuNumber;
					LastMenuNumber = 1;	// Next time go home.
				}
				else {
					CurrentMenuNumber = 1;
					LastMenuNumber = 0;	// Next time go home.
				}	
				fNumberKeys = GetMenu(CurrentMenuNumber, &CurrentMenu, KeyNumberBuffer);
				ELSKeys_State = ELSKEYS_KEY_DONE;
				break;

			  default :
				BeepMsg = T_BAD_KEY | FOUR_BEEPS;
				break;
			}

			StartTimer( ELSKEYS_TIMER, T_KEY_DOWN );
		}
	    break;


	  /*
	   * Now wait for the key to be released
	   */
	  case ELSKEYS_KEY_DONE :
	  	if ((KeyValue & KEY_MASK) == KEY_RELEASED) { // no switch
			ELSKeys_State = ELSKEYS_KEY_WAIT;
		}
		else if ((KeyValue & KEY_HELD_DOWN) == KEY_HELD_DOWN) {
			ELSKeys_State = ELSKEYS_KEY_WAIT;
		}
		break;

	  default :
			ELSKeys_State = ELSKEYS_KEY_WAIT;
		break;
	}
}
