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


   	Initial Version: 1.00

   	Version changes:
		See config.h
*/			 

// Bring in processor definition
#include "processor.h"		// Specific Processor Definition.
#include "ELSCfg.h"			// EEROM Defaults and Processor Flags.

#include <stdio.h>
#include <ctype.h>
#include <delays.h>
#include <float.h>
#include <math.h>
#include <string.h>

#include "Common.h"			// Common constants.
#include "Config.h"			// Project specific definitions

#include "menu.h"
#include "menuscript.h"	
#include "globvars.h"

#include "floatio.h"

#include "AtoD.h"  			// AnalogToDigital Definitions.
#include "Timer.h"			// Timer definitions.
#include "Serial.h"			// Serial port definitions.
#include "beep.h"
#include "key.h"
#include "LCD.h"		// 

#ifdef USE_OB_EEROM
#include "OB_EEROM.h"			// EEROM Access definitions
#include "EEROM_Monitor.h"
#endif

#include "IOPorts.h"  		// IOPort Definitions

#include "int.h"   			// Interrupt Definitions

#include "MotorDriver.h"	// ELeadscrew Motor Driver Definitons

#include "MPG_Thread.h"
#include "KeyThread.h"
#include "MovementThread.h"	// Motor Thread Defintions.
#include "HalfNutThread.h"
#include "SpindleSpeed.h"

/*
	Writes Signon Message out to serial port surrounded by CRLF pairs.
*/
void 
PutSignOn(void) {
	PutCRLF();
	PutString(pstrSignOn);
	PutCRLF();
}


#ifdef USE_HEARTBEAT
/*
	HB_Device:
	The hearbeat is integral to the health of the system.  It flashes the ONLINE LED 
	if the machine is READY to move or be moved.
	It also serves as the down count timer to decide when to drive the on board micro-stepper 
	into a lower current drive.  It does this by pulling the reference voltage lower so the 
	stepper drivers proportionally also put less current through their respective windings.
	That way the motor doesn't lose position in low current mode but still doesn't get as hot.
	The down side of this approach is that the reduced current may cause more 'singing' in the
	windings from the chopping frequency.
*/
void 
HB_Device(void) {
  int8 iphA,iphB, phA, phB;
	if (TimerDone(HB_TIMER)) {
		if (fHB)  {  				// Timer was on.
			StartTimer(HB_TIMER, HB_OffTime);
			fHB = 0;
			if (SystemState == MACHINE_READY)
				bGRN_LED = 0;	  
		}
		else {					// Timer was off.	
			StartTimer(HB_TIMER, HB_OnTime);
			fHB = 1;
			if (SystemState == MACHINE_READY)
				bGRN_LED = 1;	  
		}
  #ifdef MICRO_STEPPING
		if (fZLocalMicroStep || fXLocalMicroStep) {
			// fStepHappened is set inside stepper interrupt routine.
			if (!fStepHappened) {  // Count down till we go idle.
				if (MotorOffDelay >= 0)	// Above zero just count down.
					MotorOffDelay--;	
				else {					// equal to zero set low current to idle motor.
					// Reset MotorOffDelay because we want to check what it's set to.
					MotorOffDelay = GetGlobalVarByte(MOTOR_POWER_DOWN_NDX);
					// If MotorOffDelay is non-zero then we're allowed to set back current.  Otherwise leave it on.
					if ((MotorOffDelay != 0) && !fMotorIdle) {	// Motor Idle yet?
						INTCON &= 0x3F;		// Just in case a step pulse arrives now.
						fMotorIdle = 1;
						// PhaseBIndex holds index of last value sent to driver for PhaseB.
						// PhaseAIndex can't be used because it holds the next expected value.
						if (fZLocalMicroStep) { // Are we micro stepping Z axis?
							if (fZDirectionCmd ^ fMZInvertDirMotor) {
								phA = MicroStepATableBkwd[PhaseBIndex];			// Get current PhaseA value.
								phB = MicroStepBTableBkwd[PhaseBIndex] & 0x7F;	// PhaseB with high bit cleared reduces current. 
							}
							else {
								phA = MicroStepATableFwd[PhaseBIndex];			// Get current PhaseA value.
								phB = MicroStepBTableFwd[PhaseBIndex] & 0x7F;	// PhaseB with high bit cleared reduces current. 
							}
						}
						else { // Nope we're micro-stepping X axis.
							if (fXDirection ^ fMXInvertDirMotor) {
								phA = MicroStepATableBkwd[PhaseBIndex];			// Get current PhaseA value.
								phB = MicroStepBTableBkwd[PhaseBIndex] & 0x7F;	// PhaseB with high bit cleared reduces current. 
							}
							else {
								phA = MicroStepATableFwd[PhaseBIndex];			// Get current PhaseA value.
								phB = MicroStepBTableFwd[PhaseBIndex] & 0x7F;	// PhaseB with high bit cleared reduces current. 
							}
						}
						MicroStep(phA, phB);						// Adjust current.
						// MicroStep() re-enables the interrupts.
					}
				}
			}
			else {	// Clear fStepHappened every tick and reset down counter.
				MotorOffDelay = GetGlobalVarByte(MOTOR_POWER_DOWN_NDX);
				fMotorIdle = 0;			// Motor's been stepped by interrupt routine and is no longer idle.
				fStepHappened = 0;
			}
		}
  #endif			
	}
}
#endif

/*
	InitHardware;
	Initialize I/O ports and then test buttons to see if special action is required.
	The Bootloader does the same thing but is ifdef'd out.  In order to make the boot loader work the 
    startup code also needs to be able to scan the keypad so that if the boot programming didn't work then
    the boot loader needs to be able to be restarted.
*/
void 
InitHardware(void) {
  uint8 baud;
  int16 i;
  int8 altkey;

	if (RXB0SIDLbits.EXID) {
		RXB0SIDLbits.EXID = 1;
		RXB0SIDLbits.EXID = 0;
	}
	if (RXB0SIDL & 0x08) {
		RXB0SIDL |= 0x08;
	}
	InitIO();
	// Now test for reset pressed DEL key to load default global variables from ROM.
	if (GetKeys(&altkey) == BUTTON_DEL) 
		InitDefaultGlobalVars();
#ifdef BOOTLOADER
	else if (GetKeys(&altkey) == BUTTON_ENTER) {	// ENTER key causes a reset and branch
		Put_ObEEROM_Byte( 0x3FF, 0xff );	// To Boot loader if installed.
		while(1) {
			_asm								// If no bootloader then it just resets.
			reset
			_endasm
		}
	}
#endif

	// Pull in EEROM'd global variables.
	RestoreGlobalVariables();

	// Get Global System flags used in various modules.
	ReadFlags();	// defined in globals.c

	// Clear out System flags and then let state machines set bits.
	RUNFlags.Byte = 0;				// No Run flags set.
	ThreadsRdy.Byte = 0;			// No threads are ready yet.
	ThreadsRun.Byte = 0;			// No threads running yet.
	OkToStopFlags.Byte = 0xff;		// All threads are OK to Stop.

	InitInterruptVariables();		// Any variables used locally inside the interrupt routine are initialized here.


	InitTimerDevice();				// Initialize timers and set up for 10mS TICK
	InitBeepDevice();				// Initialize BEEPER 
	InitKeyDevice();				// Prep for scanning keypad.
	InitMotorDevice();				// Set up motor variables.

#ifdef HALF_NUT_INSTALLED 
	InitHalfNutDevice();			// A/D Half nut to verify if it's there.
#else
	InitSpindleDevice();			// Otherwise watch A/D for spindle speed.
#endif

	InitLCD();						// Setup up LCD interface and display rev.

	// Init serial port using baud as a temporary variable.
	// If MPG Knob s pressed during power up then default baud rate to 115K.
	baud = Get_ObEEROM_Byte(EM_BAUD);
    if ((baud == 0) || (bMPGSelect == 0)) {
        baud = B115K;
        // Check and restore default baud rate if needed.
        if (bMPGSelect == 0) {
            Put_ObEEROM_Byte(EM_BAUD, B115K);
        }
    }
    InitSerial(baud);				// Initi the serial port.


#ifdef USE_HEARTBEAT

  #ifdef MICRO_STEPPING
	// SYSFlags have been loaded.  Test to see if local microstepping via SPI is 
	// possible.
 	// Can't enable Micro-step if hardware isn't installed because that will disable I/O port and then
	// no step/dir pulses show up on DB-25.
	// Menu code sets up or disable SPI if flag is toggled.
	if (fZLocalMicroStep || fXLocalMicroStep) {
		SSPSTAT = 0x40;		// Transmit on clock high.
		SSPCON1	= 0x20;		// FOsc/4, Enable SPI, Clock Idle Low.
		bMOTOR_LATCH = 0;
		SSPBUF = 0;
		fStepHappened = 0;	// Prevent shutdown from doing anything to motor.
		MicroStep(0,0);		// Set motor off.
	}
	MotorOffDelay = 0;
  #endif

	// Set up Heartbeat LED and Heartbeat Timer.
	HB_OnTime = HB_DEFAULT_ON; 
	HB_OffTime = HB_DEFAULT_OFF;	
	StartTimer( HB_TIMER, HB_OnTime);
	fHB = 1;
//	if (fFlashLED) 
//		bRED_LED = 1;
#endif

	// Now get the hardware parameters and set up program constants.
	if (GlobalVars[LEADSCREW_IPITCH_NDX].f > 0.)
		ZDistanceDivisor = GlobalVars[MOTOR_STEPS_REV_Z_NDX].f / GlobalVars[LEADSCREW_IPITCH_NDX].f;
	else
		ZDistanceDivisor = 1.0;
	MotionPitchIndex = TURN_PITCH_NDX;			// Show Turning pitch.


	
    if (GlobalVars[CROSS_SLIDE_IPITCH_NDX].f != 0) // Prevent divide by zero.
		XDistanceDivisor = (float)GlobalVars[MOTOR_STEPS_REV_X_NDX].l / GlobalVars[CROSS_SLIDE_IPITCH_NDX].f;
	else
		XDistanceDivisor = 1.0;

	// Then the Run time display and Menu variables 
	fRunTimeDisplay = 0;	// No run time display.
	LastMenuNumber = 1;		// Esc. takes us to previous menu which is RUN SETUP
	CurrentMenuNumber = 0;  // Force signon menu.
	SystemError = 0;

	// Initialize the main state machine.
	SystemState = MACHINE_INIT;
	SystemCommand = NO_CMD;

	INTCON = RTC_INTEN;		// Enable Interrupts.
	PIE1bits.CCP1IE = 1;	// Start it up.
}

/*
	SetDisplayMode :
	Sets up common variables based on which menu is visible and which key was pressed.
	Called from KeyThread() and MPG_Thread() and mainline any time we move to an Error state or to Ready State.
*/
/*
 *  FUNCTION: SetDisplayMode
 *
 *  PARAMETERS:		displayMode
 *
 *  USES GLOBALS:	DisplayModeMenuIndex
 *					fRunTimeDisplay
 *					LastMenuNumber
 *					CurrentMenuNumber
 *					fNumberKeys
 *
 *  DESCRIPTION:	Wrapper function that sets up what's displayed and 
 *					what menus are available from the ESC key and if
 *					the numeric input keys are availble.
 *
 *  RETURNS: 		Nothing.
 *
 */
void 
SetDisplayMode(int8 displayMode) {
	DisplayModeMenuIndex = displayMode;
	DEBUG_MENU("DisplayModeMenuIndex=%d\n", displayMode);
	switch (DisplayModeMenuIndex) {
	  /*
		 Display config menu's and allow number keys for entering data.
	  */
	  case KEY_MENU_MODE :
		fRunTimeDisplay = 0;
		LastMenuNumber = 1;
		CurrentMenuNumber = -1;  // Force Configuration menu on keypress.
		fNumberKeys = 1;		 // Enable numeric keys.
		break;
	
	  /*
		Display main run menu and allow movement.
	  */	
	  case ADJUST_MODE :
	  case INFO1_MODE :
	  case INFO2_MODE :
	  case INFO3_MODE :
	  case INFO4_MODE :
		fRunTimeDisplay = 1;
		LastMenuNumber = 1;		// Esc. takes us to previous menu which is RUN SETUP
		fNumberKeys = 0;		// Disable numeric keys.
		break;

	  /*
		Display Error Screen
	  */	
	  case MSG_SPINDLE_OFF_MODE	:
	  case MSG_ESTOP_INPUT_ACTIVE :
	  case MSG_LIMIT_INPUT_ACTIVE :
	  case ERROR_MODE :
		fRunTimeDisplay = 1;	  
		LastMenuNumber = 1;		 // Esc. takes us to previous menu which is RUN SETUP
		fNumberKeys = 0;		 // Disable numeric keys.
		break;

	  /*
		Display main run menu and don't allow movement keys.
		Only Start/Stop/ESC allowed.
	  */	
	  case RUN_MODE :
		fRunTimeDisplay = 1;
		LastMenuNumber = 1;		// ESC will take us to previous menu which is RUN SETUP
		fNumberKeys = 0;		// Disable numeric keys.
		break;

	  /*
		Default for the ones we missed...
	  */
	  default :
		DEBUG_MENU("default\n");
		fRunTimeDisplay = 1;
		LastMenuNumber = 1;		// Esc. takes us to previous menu which is RUN SETUP
		CurrentMenuNumber = 0;  // Force signon.
		fNumberKeys = 0;		// Disable numeric keys.
		break;
	}
}

/*
 *  FUNCTION: DumpLCDStr
 *
 *  PARAMETERS:		None
 *
 *  USES GLOBALS:	OutputBuffer
 *
 *  DESCRIPTION:	Dumps the LCD display Output Buffer	for diagnostic purposes.
 *					Currently all calls to this function have been commented out.
 *					Would be better to use an #ifdef on the calls and this function.
 *
 *  RETURNS: 		Nothing
 *
 */
void 
DumpLCDStr(void) {
int8 i;
		for (i=4; i<44; i++) {
			printf((MEM_MODEL rom char *)"%02X,",OutputBuffer[i]);
		}
		PutCRLF();	
		for (i=4; i<44; i++) {
			if (OutputBuffer[i] >= 0x20)
				putchar(OutputBuffer[i]);
			else
				putchar('.');
		}
		PutCRLF(); PutCRLF();	
}

/*
 *  FUNCTION: RunTimeUnits
 *
 *  PARAMETERS:		ndx
 *
 *  USES GLOBALS:	OutputBuffer, fMetricMode
 *
 *  DESCRIPTION:	Sets 'm' or '" at index location in display buffer
 *					depending on metric mode.
 *
 *  RETURNS: 		Nothing
 *
 */
void 
RunTimeUnits(int8 ndx) {
	if (fMetricMode) {
		OutputBuffer[ndx++] = 'm';
	}
	else {
		OutputBuffer[ndx++] = '"';
	}
}

/*
 *  FUNCTION: RunTimeDisplayThread
 *
 *  PARAMETERS:	 	None
 *
 *  USES GLOBALS:
 *
 *  DESCRIPTION:	Comprehensive function to display status information on the screen.
 * 					During some of these modes some keys are functional.  
 *					The Keythread() performs specific functions based on what key is 
 *					pressed and what is displayed.  This is the easiest way to modify 
 *					RAM based variables whereas the standard menus are for parameters stored 
 *					to EEROM.
 *
 *  RETURNS: 		Nothing
 *
 */
void
RunTimeDisplayThread(void) {
static int32 spc;
  int32 tmp;
  static int32 oldmtrpos;
  int32 mtrZpos, mtrXpos;
  float32 resZ, resX;
  int8 FractionWidth;
  int16 mtrXIncr;
  int16 i;
  char sSpeed[4];
  char taperChar;

	// If the Run Time Display is enabled and 250mS have elapsed then update the info
	if 	(fRunTimeDisplay && TimerDone(RUNDISPLAY_TIMER)) {
		StartTimer(RUNDISPLAY_TIMER, T_250MS);

		// Set up to start at top left corner of LCD display.
		OutputBuffer[0] = 0x1b;
		OutputBuffer[1] = '=';
		OutputBuffer[2] = 0x20;		// Line 0.
		OutputBuffer[3] = 0x20;     // Column 0                        |

		// Get speed and position values from interrupt routine.
		// Protect long int since processor can't move it in one indivisible operation.
		INTCON &= 0x3F;
		mtrZpos = ZMotorPosition;
		mtrXpos = XMotorRelPosition;
		// track absolute motor position.
		mtrXIncr = XMotorIncrement;	// Get value incremented inside interrupt every X step.
		XMotorIncrement = 0;		// Now that we have it,  start it at zero again.
		INTCON |= 0xC0;
 
		// Check if X axis changed...
		if (mtrXIncr != 0) {
			tmp = mtrXIncr;	// Turn into 32 bit. Done so 8 bit value doesn't overflow when changed to diameter.
			// Now get current absolute position and change to steps.
			XMotorAbsPosition = XDistanceToSteps(GetGlobalVarFloat(X_DIAMETER_NDX));
			// Add to current position.
			XMotorAbsPosition += (tmp * 2);		// A single step in radius change is two steps in diameter change.
			if (XMotorAbsPosition < 0) 
				XMotorAbsPosition = 0;			// Don't allow negative dimensions
			// Update global variable used in calculating SFM or SMM in PrintRPM() function.
			SetGlobalVarFloat(X_DIAMETER_NDX, StepsToXDistance(XMotorAbsPosition));
		}

		// All display routines need distance in engineering units.
		// Distances are stored as motor steps.
		// The distance Divisors are calculated when the Thread or Turn button is pressed
		// and are based on leadscrew and stepper motor values.
		resZ = mtrZpos / ZDistanceDivisor;
		resX = mtrXpos / XDistanceDivisor;
		if (fMetricMode) {
			resZ *= 25.4;
			resX *= 25.4;
			FractionWidth = 2;
			strcpypgm2ram(sSpeed, (MEM_MODEL rom char *)"SMM");
		}
		else {
			FractionWidth = 3;
			strcpypgm2ram(sSpeed, (MEM_MODEL rom char *)"SFM");
		}

		// Now decide which menu should display which data.
		switch (DisplayModeMenuIndex) {

		  /*
		   * Main Run menu 
		  */		
		  case ADJUST_MODE :
		  case RUN_MODE :		
			if (!fSFMMode)
				strcpypgm2ram(sSpeed, (MEM_MODEL rom char *)"RPM"); 

			sprintf(&OutputBuffer[4], (MEM_MODEL rom char *)"0000  %s Z  0.0    P 0.0     X  0.0    ",sSpeed);
			
			// Display RPM
			SuLongToDec(&OutputBuffer[4], PrintRPM(0, fSFMMode), 5, 0);
			OutputBuffer[9] = ' ';	// Get rid of terminating null;
			// Then display Z Axis.
			floatToAscii(resZ, &OutputBuffer[15],7,FractionWidth);
			RunTimeUnits(22);

			// Display Feed Pitch
/*
			if (!fMetricMode && fXThreading) { // for turning in imperial mode.
				sprintf(&OutputBuffer[24], (MEM_MODEL rom char *)"      TPI");
				OutputBuffer[33] = ' ';
				// Check if Thread pitch is <= 100 TPI and if so show to one decimal place
				if (GlobalVars[MotionPitchIndex].f > 0.010) {  // Avoid divide by zero
					floatToAscii(1/GlobalVars[MotionPitchIndex].f, &OutputBuffer[24],5,1);
					OutputBuffer[29] = ' ';  // Get rid of terminating null.
				}
				else {	// otherwise don't show decimal and print one location to the right to line up with "TPI"
					floatToAscii(1/GlobalVars[MotionPitchIndex].f, &OutputBuffer[25],4,0);
					OutputBuffer[29] = ' ';  // Get rid of terminating null.
				}
			}
			else {// Display is pitch for metric turning and threading..
	Above code commented out so Z axis movement is always as distance moved per spindle rev so that metric and 
    imperial match for both threading and turning modes.  Now to identify turning verses threading the first char on 
    the second line is a T for turning or S for screw cutting.  Later we'll upgrade and add the F for facing.
	
*/
		    	sprintf(&OutputBuffer[24], (MEM_MODEL rom char *)"        ");
				OutputBuffer[32] = ' ';

				// Show value in format of metric or imperial mode.
				if (fMetricMode) { // Metric mode has two digits after the decimal point and three i front..
					floatToAscii(GlobalVars[MotionPitchIndex].f * 25.4, &OutputBuffer[25],6,FractionWidth);
					RunTimeUnits(31);
				}
				else {	// Imperial mode has 3 digits after the decimal and only two in front.
					if (GlobalVars[MotionPitchIndex].f < 100.0)
						floatToAscii(GlobalVars[MotionPitchIndex].f, &OutputBuffer[25],7,FractionWidth);
					else // Most of the time.....  8-(
						floatToAscii(GlobalVars[MotionPitchIndex].f, &OutputBuffer[25],7,FractionWidth);
					RunTimeUnits(32);
				}
//			}
			// Now show what type of automatic operation we are doing.
			if (fXThreading) { 	// Multiple passes till depth complete.
				OutputBuffer[24] = 'S';			// 'S'crew cutting
			}
			else {			  	// Single pass operations
				if (fCrossFeed)
					OutputBuffer[24] = 'F';			// 'F'acing
				else
					OutputBuffer[24] = 'T';			// 'T'urning 
			}
			// Display X Axis
			floatToAscii(resX, &OutputBuffer[35],7,FractionWidth);
			RunTimeUnits(42);

			// Show which axis has MPG active.
			// This is shown by one of three characters.  
			// The '=' is Taper Mode off and shows which axis the MPG runs.
			// If there's a '<' or '>' then taper mode is on and matches the arrow
			// in the taper setup menu.
			taperChar = '=';
			if (fTapering) {
				if (fTaperDirection)
					taperChar = '<';
				else
					taperChar = '>';
			}
			if (ActiveMotor == MOTOR_Z) {
				OutputBuffer[23] = taperChar;
				OutputBuffer[43] = ' ';
			}
			else {
				OutputBuffer[23] = ' ';
				OutputBuffer[43] = taperChar;
			}
			break;

		  /*
		   * Start, End and Current position display. 
		  */
		  case INFO1_MODE:
			sprintf(&OutputBuffer[4], (MEM_MODEL rom char *)"SETPOINTS  Z 0.000  B  0.000   E 0.000  ");
			// Display Current Position
			floatToAscii(resZ, &OutputBuffer[16],7,FractionWidth);
			RunTimeUnits(23);

			// Display Start Position
			floatToAscii(ZBeginPosition, &OutputBuffer[25],7,FractionWidth);
			RunTimeUnits(32);

			// Display End Position
			floatToAscii(ZEndPosition, &OutputBuffer[36],7,FractionWidth);
			RunTimeUnits(43);
			break;

		  case INFO2_MODE:
			sprintf(&OutputBuffer[4], (MEM_MODEL rom char *)"SETPOINTS  X 0.000  B  0.000   R 0.000  ");
			// Display Current Position
			floatToAscii(resX, &OutputBuffer[16],7,FractionWidth);
			RunTimeUnits(23);

			// Display Start Position
			floatToAscii(XBeginPosition, &OutputBuffer[25],7,FractionWidth);
			RunTimeUnits(32);

			// Display retracted Position
			floatToAscii(XRetractedPosition, &OutputBuffer[36],7,FractionWidth);
			RunTimeUnits(43);
			break;

		  case INFO3_MODE:
			sprintf(&OutputBuffer[4], (MEM_MODEL rom char *)"PASS COUNT = %2d     FIRST  RST  SET  CLR",
					 PassCount );
			break;

		  case INFO4_MODE:
			if (fTapering) 
				sprintf(&OutputBuffer[4], (MEM_MODEL rom char *)" TAPER MODE ON      ENABLE SET   DISABLE");
			else
				sprintf(&OutputBuffer[4], (MEM_MODEL rom char *)" TAPER MODE OFF     ENABLE SET   DISABLE");
			break;

		  case MSG_FIND_BEGIN_MODE :
			sprintf(&OutputBuffer[4], (MEM_MODEL rom char *)" Finding Begin Pos. Z  0.000   X  0.000 ",
				PassCount);
			// Display Current Position
			floatToAscii(resZ, &OutputBuffer[25],7,FractionWidth);
			RunTimeUnits(32);
			// Display X Axis
			floatToAscii(resX, &OutputBuffer[36],7,FractionWidth);
			RunTimeUnits(43);
			break;

		  case MSG_THREAD_END_MODE :
			if (fXThreading) {
				if (fUseCompound && !fAutoX) { // If we're using the compound then show the compound distance instead of cross slide.
					if (PassCount == 0) 
						sprintf(&OutputBuffer[4], (MEM_MODEL rom char *)"SpringPass [Pass=%2d]Z 0.000    C  0.000 ",
							SpringPassCount);
					else {
						if (fBroachMode) 
							sprintf(&OutputBuffer[4], (MEM_MODEL rom char *)"Broaching [Pass=%2d] Z 0.000    C  0.000 ",
								PassCount+SpringPassCount);
						else
							sprintf(&OutputBuffer[4], (MEM_MODEL rom char *)"Threading [Pass=%2d] Z 0.000    C  0.000 ",
								PassCount+SpringPassCount);
					}
				}
				else {// Otherwise show the cross feed distance
					if (PassCount == 0) {
						sprintf(&OutputBuffer[4], (MEM_MODEL rom char *)"SpringPass [Pass=%2d]Z 0.000    X  0.000 ",
							SpringPassCount);
					}
					else {
						if (fBroachMode) 
							sprintf(&OutputBuffer[4], (MEM_MODEL rom char *)"Broaching [Pass=%2d] Z 0.000    X  0.000 ",
								PassCount+SpringPassCount);
						else
							sprintf(&OutputBuffer[4], (MEM_MODEL rom char *)"Threading [Pass=%2d] Z 0.000    X  0.000 ",
								PassCount+SpringPassCount);
					}
				}
			}
			else
				sprintf(&OutputBuffer[4], (MEM_MODEL rom char *)"Turning to End Pos. Z 0.000    X  0.000 ");
			// Display Current Position
			floatToAscii(resZ, &OutputBuffer[25],7,FractionWidth);
			RunTimeUnits(32);
			// Display X Axis
			floatToAscii(resX, &OutputBuffer[36],7,FractionWidth);
			RunTimeUnits(43);
			break;

		  case MSG_INSERT_TOOL_RQ_MODE :
			if (fUseCompound)  // If we're using the compound then show the compound distance instead of cross slide.
				sprintf(&OutputBuffer[4], (MEM_MODEL rom char *)"Insert Tool in work Z  0.000   C  0.000 ");
			else				// otherwise show the cross feed value.
				sprintf(&OutputBuffer[4], (MEM_MODEL rom char *)"Insert Tool in work Z  0.000   X  0.000 ");
			// Display Current Position
			floatToAscii(resZ, &OutputBuffer[25],7,FractionWidth);
			RunTimeUnits(32);
			// Display X Axis
			floatToAscii(resX, &OutputBuffer[36],7,FractionWidth);
			RunTimeUnits(43);
			break;

		  case MSG_INSERT_TOOL_MODE :
			sprintf(&OutputBuffer[4], (MEM_MODEL rom char *)"Insert Tool [P=%2d]  Z  0.000   X  0.000 ",
				PassCount);
			// Display Current Position
			floatToAscii(resZ, &OutputBuffer[25],7,FractionWidth);
			RunTimeUnits(32);
			// Display X Axis
			floatToAscii(resX, &OutputBuffer[36],7,FractionWidth);
			RunTimeUnits(43);
			break;

		  case MSG_RETRACT_TOOL_RQ_MODE :
			sprintf(&OutputBuffer[4], (MEM_MODEL rom char *)"Retract Tool now         Z =   0.000\"   ");
			// Display Current Position
			floatToAscii(resZ, &OutputBuffer[33],7,FractionWidth);
			RunTimeUnits(40);
			break;

		  case MSG_RETRACT_TOOL_MODE :
			sprintf(&OutputBuffer[4], (MEM_MODEL rom char *)"Retracting Tool     Z  0.000   X  0.000 ",
				PassCount);
			// Display Current Position
			floatToAscii(resZ, &OutputBuffer[25],7,FractionWidth);
			RunTimeUnits(32);
			// Display X Axis
			floatToAscii(resX, &OutputBuffer[36],7,FractionWidth);
			RunTimeUnits(43);
			break;

		  case MSG_SPINDLE_OFF_MODE :
			sprintf(&OutputBuffer[4], (MEM_MODEL rom char *)"ERROR!! Spindle Off 0000  RPM           ");
			// Now calculate RPM.
			if (AveragedClocksPerRev > 0) 
				tmp = (PULSE_CLOCK_RATE * 60L) / AveragedClocksPerRev;
			else
				tmp = 0;
	
			// Display RPM
			SuLongToDec(&OutputBuffer[24], tmp, 5, 0);
			OutputBuffer[29] = ' ';	// Get rid of terminating null;
			break;

		  case MSG_ESTOP_INPUT_ACTIVE :
			sprintf(&OutputBuffer[4], (MEM_MODEL rom char *)"  ESTOP INPUT             ACTIVE!!      ");
			break;

		  case MSG_LIMIT_INPUT_ACTIVE :
			sprintf(&OutputBuffer[4], (MEM_MODEL rom char *)"  LIMIT INPUT             ACTIVE!!      ");
			break;
										//                                 2        33 
		  case MSG_SEQ_COMPLETE_MODE :  //             4                   4        34
			sprintf(&OutputBuffer[4], (MEM_MODEL rom char *)"Sequence Complete   Z  0.000   X  0.000 ",
				PassCount);
			// Display Current Position
			floatToAscii(resZ, &OutputBuffer[25],7,FractionWidth);
			RunTimeUnits(32);
			// Display X Axis
			floatToAscii(resX, &OutputBuffer[36],7,FractionWidth);
			RunTimeUnits(43);
			break;

		  case MSG_MOTOR_STOPPED_ERROR :
			sprintf(&OutputBuffer[4], (MEM_MODEL rom char *)"USER STOPPED MOTOR! Z  0.000   X  0.000 ",
				PassCount);
			// Display Current Position
			floatToAscii(resZ, &OutputBuffer[25],7,FractionWidth);
			RunTimeUnits(32);
			// Display X Axis
			floatToAscii(resX, &OutputBuffer[36],7,FractionWidth);
			RunTimeUnits(43);
			break;

		  case MSG_SPINDLE_TO_FAST_ERROR :
			sprintf(&OutputBuffer[4], (MEM_MODEL rom char *)"SPINDLE TOO FAST    0000 RPM.  TRY 0000");
			// Now calculate RPM.
			if (AveragedClocksPerRev > 0) 
				tmp = (PULSE_CLOCK_RATE * 60L) / AveragedClocksPerRev;
			else
				tmp = 0;
	
			// Display RPM
			SuLongToDec(&OutputBuffer[24], tmp, 4, 0);
			OutputBuffer[28] = ' ';	// Get rid of terminating null;
			// Display suggested RPM.
			tmp = TargetRPM;
			SuLongToDec(&OutputBuffer[39], tmp, 4, 0);
			OutputBuffer[43] = ' ';	// Get rid of terminating null;

			break;

		  case MSG_MOTOR_DISTANCE_ERROR :
			sprintf(&OutputBuffer[4], (MEM_MODEL rom char *)"    MOTOR DISTANCE        ERROR!        ");
			break;

		  case MSG_TAPER_TOO_BIG_ERROR :
			sprintf(&OutputBuffer[4], (MEM_MODEL rom char *)"TAPER TOO BIG ERROR    MAX              ");
			break;

		  case MSG_CHANGE_JOG_DISTANCE_MODE :
			if (ActiveMotor == MOTOR_Z)
				sprintf(&OutputBuffer[4], (MEM_MODEL rom char *)"NEW JOG DSTZ -0.000 CNCL SELX       ACPT");
			else
				sprintf(&OutputBuffer[4], (MEM_MODEL rom char *)"NEW JOG DSTX -0.000 CNCL SELZ       ACPT");
			floatToAscii(JogIncrement[NewJogIndex], &OutputBuffer[16],7,FractionWidth);
			RunTimeUnits(23);
			break;

		  case MSG_START_EQUALS_END_ERROR :
			sprintf(&OutputBuffer[4], (MEM_MODEL rom char *)"Begin Position same as End Position ERR.");
			break;

		  default :
			sprintf(&OutputBuffer[4], (MEM_MODEL rom char *)"SETPOINTS Z  0.000  B  0.000\" E  0.000\" ");
			// Display Current Position
			floatToAscii(resZ, &OutputBuffer[15],7,FractionWidth);
			RunTimeUnits(22);
			break;
		}
		OutputBuffer[44] = '\0'; 
		LCDSendBuf((PBYTE)OutputBuffer);
	}
}

/*
	SerialThread:
	This thread reads the serial port and parses the input.  Used for diagnostic output.
	The individual commands are described at each case ' ': 	
*/
void
SerialThread(void) {
  int8 ch;
  int8 i;
    if (ch = getc()) {
        PutCRLF(); 

        switch ( ch ) {

		/*
			'e' EEROM Monitor.  
			A set of commands to directly modify EEROM data.
			Syntax: See EEROM_Monitor.c for command syntax.
		*/
		case 'e' :
			#ifdef EEROM_MONITOR
			EEROM_Monitor();
			#endif
			break;

		/*
			'f' Display System Flags.
			A diagnostic command that shows what the state is of each thread and device driver.
		*/
		case 'f' :
			printf((MEM_MODEL rom char *)"ThreadFLAGS: RDY=%02X, RUN=%02X RUNFLAG=%02X, STOPFlags=%02X, SystemCmd=%d\n",
				ThreadsRdy.Byte, ThreadsRun.Byte, RUNFlags.Byte, OkToStopFlags.Byte, SystemCommand );
			
			printf((MEM_MODEL rom char *)"EEROM FLAGS: SYS=%02X, MGMT=%02X, ZMOTOR=%02X, XMOTOR=%02X, COMM=%02X\n",
				SYSFlags.Byte,
				MgmtFlags.Byte,
				ZMotorFlags.Byte,
				XMotorFlags.Byte,
			   	CommFlags.Byte
				 );
			printf((MEM_MODEL rom char *)"Local Flags: Move:RUN=%02X\n",
				MovementRunFlags);
			break;
		/*
			'i' Show Port values.
			Query the I/O ports to determine the actual value.
			Syntax i0<CR> to i4<CR>
		*/
		case 'i' :
			ShowPort();
			break;
		/*
			'm' Dump User menus
			Enable character logging on the terminal emulation program and use this command to create
			a file with all the menus used in the ELS along with the current values of their parameters.			
		*/
		case 'm' :
			DumpMenu();
			break;
		/*
			'b' Output to PortB.
			Purely for diagnostic purposes to set PORT B to a value.
		*/
		case 'o' :
			ch = GetHexWord();
			PORTB = ch;
			printf((MEM_MODEL rom char *)"PORT B =%02X\n",PORTB);
			break;

#ifdef TRACK_SPINDLE_SPEED
		case 't' :	// Spindle Tracking variables.
			printf((MEM_MODEL rom char *)"SpinRate\n",SpinRate);
			break;
#endif			

		/*
			's' Status Flags
			Another diagnostic command that displays the various Status flags.
		*/
		case 's' :
			PrintRPM(1,0);		// Display serial on console and show RPM not SFM
			printf((MEM_MODEL rom char *)" CONTROLFlags=%02X\n",CONTROLFlags.Byte);
			printf((MEM_MODEL rom char *)" SYS:%d, MV: %d \n", SystemState, MovementState); 
			printf((MEM_MODEL rom char *)" ZStepFlg:%02X, XStepFlg:%02X \n", ZStepFlags.Byte, XStepFlags.Byte); 
			printf((MEM_MODEL rom char *)" ActiveFlg:%02X\n",ActiveFlags.Byte); 
			for (i=0;i<4; i++)
				printf((MEM_MODEL MEM_MODEL rom char *)"%5ld, ", RPMAverage[i]);
			printf((MEM_MODEL rom char *)"\n");
			break;
		/*
			'v', '?'
			Display Code version #
		*/
		case 'v' :
		case '?' :
			PutSignOn();
			break;

        default :
			PutString("eh?\n");
            // Invalid command so flush buffer.
            break;
        }
		while ( getc() );
		PutCRLF();
		putchar('>');
    }
}


/*
	ChangeState:
	Rather than just assign the new state this function also sets up a string message so that the diagnostic
    output can show the machine's states.
*/
MEM_MODEL rom char * SystemStateStr = "SysState";
MEM_MODEL rom char * SystemStatesStrings[7] = {
	"M_RUNNING",		// When automated movement is in progress
	"M_READY",			// When manual movement is in progress or can happen.
	"M_IDLE",			// No manual movement allowed, just parameter entry.
	"M_STARTING",		// What happens just before RUNNING
	"M_STOPPING",		// What happends after RUNNING.
	"M_ERROR",			// Machine is in error
	"M_INIT" };			// Initialize everything.

void 
ChangeState(MEM_MODEL rom char * sname, pint8 state, int8 new_state) {
  char buf[32];
	strncpypgm2ram(buf,sname,32);
	DEBUGSTR("\n%s: ",buf);
	strncpypgm2ram(buf,SystemStatesStrings[new_state],32);
	DEBUGSTR("%s", buf);
	*state = new_state;
}

void 
main(void) {
	
	InitHardware();				// Initialize I/O ports and Memory.
	InitRunKeyThread();			// Set up each of the Key threads.
	InitRdyKeyThread();
	InitIdleKeyThread();
	InitErrorKeyThread();
	InitMPG_Thread(fMetricMode);	// Set up the MPG knob.
	
	PutSignOn();					// Say hello.
	// Add newline since signon is shared with LCD display and doesn't have embedded CRLF
	DEBUGSTR("\n>");		// And the prompt of course.

	/*
		Mainline state machine:
		Loop around each time running the device drivers and then depending on the state
		run specific threads.
	*/
    while (TRUE) {

		TimerDevice();			// Check out elapsed times.
		HB_Device();			// Blink LED if needed and power down micro-stepper after delay is done.
		BeeperDevice();			// Make a beeping sound
		KeyDevice();			// scan the keypad
		MotorDevice();			// Monitor Motor movement.

#ifndef HALF_NUT_INSTALLED 
		SpindleDevice();
#endif
		SerialThread();			// Handle serial diagnostics.  (Later MODBUS and Control of Spindle Motor).

		switch (SystemState) {

  		  /*
  		  		Green STATUS Light ON
  		  */
		  case MACHINE_RUNNING :

			if (SystemCommand == START_CMD)
				SystemCommand = NO_CMD;			// Cancel any start commands until system is ready again.

			/*
				if fRunMachine has been cleared then someone has requested that we stop what we are doing.
				Each of the running threads maintains an OkToStop flag and if set we can then 
				change to the stopping state or if it was a system error of some sort then stop now and 
				go directly to the error state.
			*/
			if ( !fRunMachine && ((OkToStopFlags.Byte & RUN_THREADS_MASK) == RUN_THREADS_MASK)) {
				// We've been asked to stop and everyone is good with that.
				if (SystemError == 0) {	// What kind of stop was it?
					ChangeState(SystemStateStr, &SystemState, MACHINE_STOPPING);
				}
				else {
					BeepMsg = T_ERROR | TWO_BEEPS;
					bRED_LED = 1;			// Show that there's an error.
					bGRN_LED = 0;			// Turn off RUN/RDY LED
					SetDisplayMode(SystemError);
					ChangeState(SystemStateStr, &SystemState, MACHINE_ERROR);
				}
			}
			else {  // Run our threads.
				MovementThread();			// Move motors.
				RunTimeDisplayThread();		// Display what's happening.
				RunKeyThread();				// Monitor relavent keys.
			}
			break;

			/*
				Green STATUS Light Flashing.
				Look for a start command or Idle command. 
				otherwise just keep running the ready threads
				which allow jogging and some parameter adjustments.
				
			*/
		  case MACHINE_READY :
			if (((ThreadsRdy.Byte & RDY_THREADS_MASK) == RDY_THREADS_MASK) && (SystemCommand != NO_CMD)) {
				if (SystemCommand == START_CMD) {
					SystemCommand = NO_CMD;
					fRunMachine = 1;
					bGRN_LED = 1;
					ChangeState(SystemStateStr, &SystemState, MACHINE_STARTING);
				}
				else if (SystemCommand == IDLE_CMD) {	// When Idle the machine does not allow motion.
					SystemCommand = NO_CMD;
					bGRN_LED = 0;
					ChangeState(SystemStateStr, &SystemState, MACHINE_IDLE);
				}
				else if (SystemCommand == HOME_CMD) {
					SystemCommand = NO_CMD;			// Clear command request.
					fRunMachine = 1;				// Test system we're running.
			        // We're no longer ready
        			fMTRdy = 0;             // but instead we're 
        			fMTRun = 1;             // running.
					bGRN_LED = 1;
					ChangeState(SystemStateStr, &SystemState, MACHINE_RUNNING);
				}
				else
					SystemCommand = NO_CMD;
			}
			else {
				MPG_RunThread();			// Check if user want's to move carriage.
				RunTimeDisplayThread();		// Display current message.
				RdyKeyThread();				// Check on what user wants to do.
					
#ifdef HALF_NUT_INSTALLED 
				if (fHalfNut)				// See if user wants to move carriage.
					HalfNutThread();
#endif											// Any of the above create an error?
				if (SystemError != 0) {	// 29JAN08 Added this to create error when jog or MPG is done and step is requested.
					// What kind of Error happens from a user input?
					BeepMsg = T_ERROR | ONE_BEEP;
					bRED_LED = 1;			// Show that there's an error.
					bGRN_LED = 0;			// Turn off RUN/RDY LED
					SetDisplayMode(SystemError);
					ChangeState(SystemStateStr, &SystemState, MACHINE_ERROR);
				}
			}
		  	break;


		  /*
		  		Green STATUS Light OFF to inform that motion is not possible.
				Wait for RDY_CMD and otherwise just monitor keys and allow
				parameter changes.
 
		  */
		  case MACHINE_IDLE :
				if (
					((ThreadsRdy.Byte & RDY_THREADS_MASK) == RDY_THREADS_MASK) && 
					 (SystemCommand == RDY_CMD)
				   ) {
					SystemCommand = NO_CMD;
					bGRN_LED = 0;
					// Going into IDLE mode means the motion setup needs to be re-initialized.
					InitMovementThread();		// So start over.
					ChangeState(SystemStateStr, &SystemState, MACHINE_READY);
				}
				else {
					IdleKeyThread();
					// If we're idle then movement isn't possible so the ESTOP input probably
					// can be ignored.  This allows the machine to stay in the idle state where
					// the flag can be changed to ignore the ESTOP line.
					if (SystemError != 0) {	
						// What kind of Error happens from a user input?
						BeepMsg = T_ERROR | ONE_BEEP;
						bRED_LED = 1;			// Show that there's an error.
						bGRN_LED = 0;			// Turn off RUN/RDY LED
						SetDisplayMode(SystemError);
						ChangeState(SystemStateStr, &SystemState, MACHINE_ERROR);
					}
				}
			break;

			/*
				Set machine to a place where it can start running.
			*/
		  case MACHINE_STARTING :
			if ((ThreadsRun.Byte && RUN_THREADS_MASK) == RUN_THREADS_MASK) {
				ChangeState(SystemStateStr, &SystemState, MACHINE_RUNNING);
			}
			else {
				MovementStartThread();		
				RunTimeDisplayThread();
			}
			break;

			/*
				Set machine to a place where it can stop.
			*/
		  case MACHINE_STOPPING :
			if ((ThreadsRdy.Byte & RDY_THREADS_MASK) == RDY_THREADS_MASK) {
				if (SystemError == 0) {	// What kind of stop was it?
					bGRN_LED = 0;
					ChangeState(SystemStateStr, &SystemState, MACHINE_READY);
				}
				else {
					BeepMsg = T_ERROR | TWO_BEEPS;
					bRED_LED = 1;			// Show that there's an error.
					bGRN_LED = 0;			// Turn off RUN/RDY LED
					SetDisplayMode(SystemError);
					ChangeState(SystemStateStr, &SystemState, MACHINE_ERROR);
				}
			}
			else {
				MovementThread();
				RunTimeDisplayThread();
				RunKeyThread();				// Monitor for STOP key held down in case
											// immediate E stop is required.
			}
		    break;

			/* 
				RED ERROR Light ON.
			*/
		  case MACHINE_ERROR :
				ErrorKeyThread();
				RunTimeDisplayThread();
				if (SystemError == 0) {
					bRED_LED = 0;
					ChangeState(SystemStateStr, &SystemState, MACHINE_INIT);
				}
			break;

		  case MACHINE_INIT :

			InitMovementThread();
			bGRN_LED = 0;
			bRED_LED = 0;
			ChangeState(SystemStateStr, &SystemState, MACHINE_IDLE);
			break;
		}
	}

}



