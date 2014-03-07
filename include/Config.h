/* 
    Config.h -- E-Leadscrew definitions for an electronic replacement of
    gears used for threading or hobbing on a Lathe or Gear Hobber.

    Copyright (C) 2005  John Dammeyer

    This program is free software; you can redistribute it and/or
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
	1.00g
		Major change in how the ZVel, ZAcc, MaxZVel and ZHolding are used.
		Now instead of offsetting the value of the velocities by 11 bits (2048)
		and then shifting left inside the interrupt routine to increment ZHolding,
		instead, the ZAcc value is shifted left 5 bits to put it into the top 11 bits
		of the low word of a long variable.
		The target or Max Velocity is now shifted by 16 bits which means the compiler does
		two high order byte transfers instead of eleven 32 bit shift sequences. Savings in
		the interrupt routine is almost 10 uSec.
	1.00h
		Added second Phase Table to avoid extra math in interrupt routine.
	1.00n
		X Axis Motion variables
	1.00o
		Thread parameters for automatic motion
		Added restore of correct X position if machine stopped and then restarted.
	1.01a 
		All distances now in imperial and menus have dependancies.
		dumpmenu() function written to show menu tree.
	1.02b
		re-organized menus so X and Z have similar appearance.
	1.02c
		Upgraded output of floats in mneu.c to use dependancies.  
		Removed most instances of units in menu's so that dependance flags
		now result in units returned from conversion routines.
	1.02d
		Got rid of the rest of the printf statements and turned them into 
		DEBUGSTR macros.
		Added code in movement thread to show compound distance if the flag is
		set to use the compound.
	1.02e
		Added Code to interrupt routine to catch not oktostop situation in movement thread.
	1.02f
		Added Menu Tables of values to handle tapers.
		Standardized on inches per foot for display of taper and
		full angle for display of taper.
		4 error beeps for invalad keys.
		Set Start, Set End can now be escaped from 
		End allowed to change while threading.
		start and end allowed to change while turning.
	1.02g
		Changed menu's for TAPER direction and added strings for ON/OFF, IN,OUT etc 
		to FLAG_TYPE records.
		Added bit offset definitions to EEROM'd flags so that labels instead of numbers
		can be used in menuscript.c  
	1.02h
		Added test for Pitch < 100.0 so that bigger numbers for turning cyles fit.
	1.02i 
		Changed how backlash is handled for Z axis since the Slew Button didn't actually do backlash 
		and that resulted in a loss of position when slewing away from thread, then pressing start which
		did a backlash compensated move to start.  i.e. Backlash removed in one direction and not in the other
		Now the Direction bit and backlash are properlly handled inside the device driver and not outide.
	1.02k
		Upgraded all definitions of variables to intxx or uintxx or pint...
	1.02n
		Changed interrupt routine to add state machine for detecting/validating spindle encoder edges.
		Therefore had to change maximum step rate to 20kHz to add some interrupt time to deal with the 
		state machine.
	1.02o
		Changed interrupt routine and other variables to use only XMotorPosition, ZMotorPosition 
		to update global CurrentXPosition, CurrentZPosition rather than a bunch of different local
		and or global variables.
		Initialized the XMotorPosition and ZMotorPosition variables inside InitInterruptVariables()
	1.02p
		Changed Electronic Half nut so variable slewing speed started at Move Speed.
	1.02q
		Debugging flag on.
	1.02r
		#ifdef'd out the Speed array debugging since it causes way to much I/O.
		Added clearing of fXMoveBSY when fXAxisActive was cleared similar to 1.02e for Z
	1.02s
		Fixed Spindle State machine to handle low going pulse so RPM was correct.
		Changed all manual X axis motion to use MOVE rate while keeping automatic motion at SLEW rate.
		This allows a specific stepper rate for automatic power cross feed while keeping manual motion fast
		or manual motion slow enough to sneak up to the work while fast retract and insert for automatic 
		motion.
	1.02t
		Documented some code and added SFM/RPM display 
	1.02u
		Fixed how passes are handled when any of the pass values are 0 depth.
	1.02v
		More fixes on how passes are handled.
	1.02w
		Code documentation
		Added init of EEROM Flags during InitDefaultGlobalVars()
	1.02x
		Changed ESTOP and LIMIT error handling so that message is displayed
		along with BEEP.  Now also testing for error in READY mode.
		Added fUseLimits flag in interrupt routine so that limits are only 
		valid during programmed moves from application (like HOME) or MPG.
		Buttons no longer cause limit behaviour so that we can move off the limit
		Previously only Z SLEW could move off the limit.  The distance move of 
		the X jog button would cause the limit before the JOG could take 
		effect.
		Finally changed RPM_STARTING timeout from 500mS to 2 Seconds.  That's because a lathe
		starting at 60RPM or one spindle pulse per second, would timeout as stopped between revolutions
		and therefore never show spindle turning or up to speed.  With 2 seconds minimum speed is now 30 RPM.
		Clear out MPG Counts too so that clearing LIMIT ERROR with ESC doesn't cause an immediate LIMIT error
		from an accumulated count.  (Note to self.  Doesn't work right yet.)
	1.02y
		Fixed Flags double linked list so that taper turning flag was visible with both ^ v arrow keys.
		Change key routines so that alt key sets bit 0 of a parameter byte.  Now ALT Z and Z Home return
		the axis home rather than just pressing alt sending Z home.
	1.02z 
		Documentation...
		Changed ob_eerom and access to global variables to use int16 instead of int8
		so that on default load/save, we'd save all the globals.  Not mormally an issue
		except for er command when creating raw cfg.c file for defaults.
	1.03
		Major change on how the direction bits are dealt with in regards to motor direction and 
		tapering.  Issue was that tapering worked if the flags were set to invert motor direction
		but tapering didn't work if the flags were cleared.
		Re-arranged flags for most used at front of list.  Z and X put in order for direction and polarity.
		Hardware change (3 diodes) so that N-key rollover worked for ALT-nHOME for both X and Y axis.
		More documentation.
	1.03a
		Fixed entry bug for DEL key in KeyThread.c where decimal point deletion didn't work if it was
		in the first column.
	1.03b
		Documentation of code complete.
	1.03c
		Change movement thread CaclulatePasses() to deal with First pass larger than total depth.
	1.03d
		Change text and variables so that Thread Start is now Thread Begin and XStart is XBegin
	1.03e
		Added Calculate Thread Depth Menu's and code to do this.
	1.03f
		Changed some DEBUGSTR messages to DEBUG_MENU for diagnosing ESC, wrong menu issues.
	1.03g
		Updated floatio.c to do better NaN checking and cleaned up code a bit.
		Changed output of PITCH field to always show distance per spindle rev rather
		than TPI for imperial threading but still pitch for metric threading.
		Now the characters 'T', 'S' and 'F' will make more sense and not run into
		the numeric field.
		Changed SetDisplayMode() so it has a RUN_MODE case and removed the CurrentMenuNumber = 0
		so that tapping ESC after a RUN event shows the Choose Option screen right away.  
		Also changed the call to InitMovementThread() so it's done as we leave IDLE mode
		back to READY mode.  That's so the FN1 BEGIN/END values are updated correctly after 
		tapping SET_BEGIN and SET_END.  It used to be called when the ESC key was tapped but the
		the two SET buttons went into IDLE mode through the back door and didn't do the update right.
		Basic functionality hasn't been changed since the idea is that if you enter IDLE mode
		your threading parameters are lost.
		Character that shows which MPG axis is active changed to [<,=,>] for no taper, or taper mode.
	1.03h
		Still problems with taper direction which became obvious now that the display shows taper direction.
		Direction swapped and problem fixed.
	1.03i
		Changed to use c018iz.o so that variables are initialized to 0 on power up or reset.  Some flags were
		random on startup causing lock up errors etc.
		Removed test of on board micro-stepper flag on init so that now SPI is always initialized.  That was 
		to fix lockup when a user selected on board micro-stepper after power had it disabled. SPI would hang
		since it was never initialized.
	1.03j
		Re-enabled test on micro-stepper flag in InitHardware since always enabling SPI always disabled I/O bits which 
		means no step/dir pulses.  Fixed by adding functions to enable/disable SPI from Flag Value in Menu.c, 
		Menuscript.c and keythread.c.  Now when user taps enter after changing a flag, the companion ReadValue 
		or WriteValue functions are called. For most flags it's DoNothing() but for the On Board Stepper it 
		enables/disables SPI.
	1.03k
		Changed movement thread for non-automatic X to only issue one message for removing tool from work.
		Fixed ESC key which moves system to IDLE mode but didn't update the screen.  Next ESC would properly show
		signon and version message.  After that 3rd ESC would bring up RUN SETUP Screen.  
			SetDisplayMode() function was missing 'case' in front of RUN_MODE :.  Compiler interpreted that as a 
			label rather than as a case.
		Fixed the problem in non-automatic mode where unit is READY, waiting for operator to retract or insert
		X and instead operator Does ALT ZeroZ which sends carriage to 0.000 but leaves unit in RUN mode.
		  That's because the fXMoveBSY flag is used to signal the the X axis move is complete.  Flag is cleared when
		  X is automatically complete or when User Taps START again signalling Movement Statemachine that X motion
		  is done and cross slide is where it's supposed to be.  (Flag wasn't being cleared).
	1.03l
		Changed Backlight on/off buttons to single toggle button on 'v' key.
	1.03m
		Added two extra tables to int.c to deal with the LMD18245 micro-stepping bug.  The direction line may only
		change when current through the windings is 0 Amps.  This bug was found by RadekCX, a CNCZONE member.
 		and verified by alan@fromorbit.com.  See int.c for more documentation about the new tables.
		Also changed Current setback timeout code so that if timeout is set to 0 it never sets back the current.
		Large heatsink required or fan cooled or LMD18245s will go into over temp shutdown with a 3A motor.

	1.04
		Added code to stop active axis when ESTOP switch happens.
	1.10 
		Changed steps per rev to float to handle odd pulley sizes.
		incomplete -- Added micro-stepping to X axis or Z axis.
		incomplete -- Removed state machine for spindle sensor
		testing -- Track Spindle RPM and adjust Z axis to maintain ratio for decreasing spindle speed.
		incomplete -- Read A/D for setting bChargePump as Spindle Speed Step rate.
	1.10a
		Added Spindle tracking for increasing speed spindle.
		Changed ELSCfg.h to reflect latest EEROM values with slower motor slew, embedded thread Calc constant and MT2 taper.
	1.10b
		Text changed Spindle "TOO" fast from "TO" fast
	1.10c
		Updated GlobVars default from DEL and Power Up to match EEROM defaults.
		Z axis slew and move, Depth Pitch Multiply, Delay to power down Z motor, MT2 taper default.
	1.10d
		Reverted back to using average clocks per rev for setting spindle tracking speed rather than the instantaneous
		speed just before threading starts.  That fixes MOVE rate to proper speed after spindle stopped.
		Also reduced the time for spindle stopped testing from 4 seconds to 2.5 seconds.
	1.10e
		-- Calculated depth now shows up as metric if METRIC flag set.
		-- Added code in CalculatePosition() so that when USE COMPOUND FOR THREADING IS ON and 
		Metric Mode is enabled, the ZBeginPosition is correct for SCREW Cutting.  It's always right for 
		Turning since the ZBeginPosition is initialized.  Without this, the value was always imperial.

	1.10f	Changes from Richard Edwards
		** REchng 23/12/08 MotorDriver.c, KeyThread.c 
			-- Backlash not working correctly in Metric Mode.
			-- Files changed MotorDriver.c (for X) line 565 KeyThread.c (for Z) line 193
			-- CalculateMotorDistance used an imperial value for backlash even in metric mode.
			-- Fixed by Multiplying Backlash by 25.4 before use (when in Metric Mode)
		** REchng 2812/08 MenuScript.c 
			-- REchng The Value being worked on is the included angle
			-- REchng We therefore need to work with 1/2 of this then multiply the result by 2

		** REChng 
			-- Changed Movement Thread to fix first of the EVERY passes to not be almost double the value.
			-- Swapped LAST and the ADJUST pass to which ever is smaller is done last.
	1.10g  jcd.
			-- Added test for MACHINE_READY in MPG interrupt routine so that we don't get accidental movement
			-- when leaving MACHINE_IDLE or MACHINE_ERROR while MPG knob was turned.
	1.10h
		** REChng -- In MovementThread.c  InitMovementThread()
			-- Changed to ZBeginPositionSteps = ZDistanceToSteps(ZBeginPosition); from using 
				THREAD_BEGIN_NDX and same with EndPosition.  Symptom was sometimes METRIC mode would have
				the imperial values.
	1.10i 	jcd -- 25MAR12
			--  Research into why long threads change pitch from pass to pass.
			--	During tracking in interrupt code change to >= so SpinAddr doesn't slowly accumulate
			--  Use Average values for clocks per rev rather than the last accumulated one so that 
				base threading RPM starts from average rather than an outlying point.

*/
// Config.h -- Project specific definitions like Crystal frequencies, Port allocations.
//========================================

#define VERSION		110

// Note signon string is exactly 40 characters long to correctly fit on the LCD display.
// Deal with processors differences between board revisions.
#if defined(__18F4620)
	// Through Hole Processor for boards above Rev 0.30	
#define pstrSignOn  "     E-LEADSCREW    Ver PIC18F4620 1.10i"
#elif defined(__18F4680)
	//	QFP processor for boards Rev 0.20 or lower and Rev. 0.30 Through Hole processors
#define pstrSignOn  "     E-LEADSCREW    Ver PIC18F4680 1.10i"
#elif defined(__18F4685)
	//	QFP processor for boards Rev 0.20 or lower and Rev. 0.30 Through Hole processors
#define pstrSignOn  "     E-LEADSCREW    Ver PIC18F4685 1.10i"
#endif

// #define FULL_DIAGNOSTICS			1		// Makes DEBUGSTR into printf so diagnostics show up on serial port.
// #define DEBUG_MENU_DIAGNOSTICS		1		// Show messages for tracing menu numbers.


#define TIMER_INTERRUPT_ENABLED		1		// Provide 10mS Interrupt driven clock and Delays.												
#define HALF_NUT_INSTALLED			1		// Allow electronic half nut operation.

#define USE_OB_EEROM 				1		// Include access to processor On Board EERPM
#define EEROM_MONITOR				1		// Display code for dumping onchip EEROM 

#ifdef TIMER_INTERRUPT_ENABLED
#define USE_HEARTBEAT				1	    // Flash A hearbeat LED.
#endif

#define X_AXIS 						1		// During development used to include/exclude X axis motion.

// Installed Crystal
#define TXTAL_CPU	TXTAL40MHZ

// Enable T0 overflow interrupt along with 
#ifdef TIMER_INTERRUPT_ENABLED
#define		RTC_INTEN	((1<<_INT0IE)+(1<<_T0IE)+(1<<_GIEL)+(1<<_GIEH))
#else
#define		RTC_INTEN	((1<<_INT0IE)+(1<<_GIEL)+(1<<_GIEH))
#endif

// Number of timers used in application.
#define MAX_TIMERS			7
#define HB_TIMER			0
#define MOTOR_TIMER			1
#define KEY_TIMER			2
#define BEEP_TIMER			3
#define ELSKEYS_TIMER		4
#define RUNDISPLAY_TIMER	5
#define HALFNUT_TIMER		6

//========================================
// Serial Constants

#define	SERIAL_XTAL	SXTAL40
#define BRGH_1	1				// Enable BRGH since Xtal is high frequency.

//========================================
#define AtoD_ADCON1			0x0E	// One AtoD channel the rest digital

//========================================
#define EEROM_SIZE	255
#define aHalfNut			PORTAbits.RA0		// I -- Analog input.
#define bMPGSelect			PORTAbits.RA1		// I -- Select Input button on MPG.
#define bCHARGE_Pump		LATAbits.LATA2		// O --	ChargePump Ouput.
#define bLIMIT_Switch		PORTAbits.RA3		// I --	Limit Switch Inputs.
#define bLCD_BACK_Light		LATAbits.LATA4		// O -- LCD_BackLight
#define bBEEP				LATAbits.LATA5		// O --	Beeper.

#define bSPINDLE			PORTBbits.RB0		// I -- Interrupt input.
#define bESTOP				PORTBbits.RB1		// I -- ESTOP
#define bMXStepMotor		LATBbits.LATB2		// I -- if CANRX or O if X STEP
#define bMXDirectionMotor	LATBbits.LATB3		// O -- X Direction
#define	bMPG_QEA			PORTBbits.RB4		// I -- QEA from MPG
#define	bMPG_QEB			PORTBbits.RB5		// I -- QEB from MPG
#define	bRED_LED			LATBbits.LATB6		// O -- LED or ICD-II
#define	bGRN_LED			LATBbits.LATB7		// O -- LED or ICD-II

#define bMOTOR_LATCH		LATCbits.LATC0	    // O -- MicroStepping Motor Latch
#define bKEY_ROW			LATCbits.LATC1	    // O -- Keypad Row 
#define bKEY_COL			LATCbits.LATC2	    // O -- Keypad Column 
#define bSPI_CLK			PORTCbits.RC3	    // O -- 
#define bMZStepMotor		LATCbits.LATC3	    // O -- 
#define bSPI_MISO			PORTCbits.RC4	    // I -- 
#define bSPI_MOSI			PORTCbits.RC5	    // O -- 
#define bMZDirectionMotor	LATCbits.LATC5	    // O -- 
#define bCOM_TX				PORTCbits.RC6	    // O -- COM Port.
#define bCOM_RX				PORTCbits.RC7	    // 1 -- 

#define bLCD_RW				LATEbits.LATE0		// O -- LCD R/W
#define bLCD_RS				LATEbits.LATE1		// O -- LCD RS
#define bLCD_E				LATEbits.LATE2		// O -- LCD Enable

#define bLCDStatus			PORTDbits.RD7
#define LCDDATA				PORTD

#define ENCODER_PORT		PORTB
#define ENCODER_MASK		0x30

#define ENCODER_POS1		0x10
#define ENCODER_POS3		0x30
#define ENCODER_POS2		0x20
#define ENCODER_POS0		0x00


#define HB_DEFAULT_ON 	5;
#define HB_DEFAULT_OFF	95;


//---------------------------------------------------------------------------------------------------------//
// EEROM Locations 
#define EM_SYS_FLAGS		0	// Default System flags.  See below
#define EM_COMM_FLAGS		1	// Bit0 == 1 if local echo and backspace enable. See Serial.h
#define EM_ZMOTOR_FLAGS		2	// Bits that tell us about our Z axis motor.  See MotorDriver.h
#define EM_XMOTOR_FLAGS		3	// Bits that tell us about our X Axis motor.  See MotorDriver.h
#define EM_MGMT_FLAGS		5	// Extra flags.
#define EM_BAUD				6	// Serial Port Baud rate.
#define EM_SYS2_FLAGS		7	// Default System flags, Byte #2.  See below

#define EM_MOTOR_OFF_DELAY  8	// Number of heartbeats to wait before shutting down motor.
#define EM_MOTOR_HOLD_VALUE 9	// value between 0 and 15 to send out as hold current. Normally 0 or 1

#define EM_GLOBAL_VARS		48	// Where in EEROM the global array starts.
//---------------------------------------------------------------------------------------------------------//


extern char MotionPitchIndex;
extern float XDistanceDivisor, ZDistanceDivisor;
extern char ZEncoderCounter, OldZEncoderCounter;

extern float TrackingRatio;			// LeadscrewRatio * PULSE_POINT.
extern long iTrackingRatio;			// Integer version

extern long CurrentZPosition;	// Position of carriage
extern long CurrentXPosition;	// Position of cross Slide.

extern char DisplayModeMenuIndex;

extern BITS SYSFlags;	// EEROM'd System configuration Flags
#define fBITPOS_HB		    		0		// Run Heartbeat Thread
#define fBITPOS_FlashLED			1      	// Blink RED LED in Heartbeat.
#define fBITPOS_EStop				2		// Invert ESTOP line.
#define fBITPOS_LimitSwitch			3		// Invert ESTOP line.
#define fBITPOS_LocalMicroStep		4		// Use onboard Stepper and SPI Microstepping.
#define fBITPOS_MetricMode			5		// Show all units as metric.
#define fBITPOS_SFMMode				6		// Show SFM based on X Diameter instead of RPM
#define fBITPOS_BroachMode			7		// Run Threading sequences without spindle turning.

#define fHB		    		SYSFlags.Bit.Bit0		// Run Heartbeat Thread
#define fFlashLED			SYSFlags.Bit.Bit1      	// Blink RED LED in Heartbeat.
#define fEStop				SYSFlags.Bit.Bit2		// Invert ESTOP line.
#define fLimitSwitch		SYSFlags.Bit.Bit3		// Invert ESTOP line.
#define fZLocalMicroStep	SYSFlags.Bit.Bit4		// Use onboard Stepper and SPI Microstepping.
#define fMetricMode			SYSFlags.Bit.Bit5		// Show all units as metric.
#define fSFMMode			SYSFlags.Bit.Bit6
#define	fBroachMode			SYSFlags.Bit.Bit7		// Allow threading cycles without spindle turning.

extern BITS SYS2Flags;	// EEROM'd System configuration Flags
#define fXLocalMicroStep	SYS2Flags.Bit.Bit0		// Use onboard Stepper and SPI Microstepping.

extern BITS MgmtFlags;	// EEROM's extra management flags.

// Non-EErom's flags used for program operation.
extern BITS CONTROLFlags;
#define fRunTimeDisplay		CONTROLFlags.Bit.Bit0		// 
#define fBLine				CONTROLFlags.Bit.Bit1
#define fBLineChanged		CONTROLFlags.Bit.Bit2
#define fRising				CONTROLFlags.Bit.Bit3
#define	fTempEncoder		CONTROLFlags.Bit.Bit4
#define fSpindleInterrupt	CONTROLFlags.Bit.Bit5
#define fSpindleTurning		CONTROLFlags.Bit.Bit6
#define fSwitchDown			CONTROLFlags.Bit.Bit7

extern BITS EDITFlags;
#define fScrollFlags		EDITFlags.Bit.Bit0		// Scrolling through LCD Flags 
#define fListFlag			EDITFlags.Bit.Bit1		// Scrolling through LCD Flags 

extern BITS RUNFlags;
#define fRunMachine			RUNFlags.Bit.Bit0
#define fMotorIdle			RUNFlags.Bit.Bit1


#define RDY_THREADS_MASK   	0b00000101
#define RUN_THREADS_MASK	0b00000001

/*
	These flags allow or prevent the change from running to stopping to ensure that a motion
	sequence is completed.
*/
extern BITS OkToStopFlags;
#define fMTOkToStop			OkToStopFlags.Bit.Bit0
#define fMTXOkToStop		OkToStopFlags.Bit.Bit1
#define fKeyOkToStop		OkToStopFlags.Bit.Bit2

/*
	When all threads are ready, then the machine can change from ready to running
*/
extern BITS ThreadsRdy;
#define fMTRdy			ThreadsRdy.Bit.Bit0
#define fMTXRdy			ThreadsRdy.Bit.Bit1
#define fKeyRdy			ThreadsRdy.Bit.Bit2

/*
	These flags represent which threads are running.
*/
extern BITS ThreadsRun;
#define fMTRun			ThreadsRun.Bit.Bit0
#define fMTXRun			ThreadsRun.Bit.Bit1
#define fKeyRun			ThreadsRun.Bit.Bit2

/*
	The mainline state machine is moved from state to state with these commands.
*/
enum SYSTEM_COMMANDS {
	NO_CMD,
	START_CMD,
	STOP_CMD,
	RDY_CMD,
	IDLE_CMD,
	HOME_CMD
};
// Which are contained in this variable.
extern enum SYSTEM_COMMANDS SystemCommand;

extern char SystemError;	// If a system wide error occurs, this variable holds the error #

extern BYTE HB_OnTime, HB_OffTime;		// Timer values for context sensitive Heartbeat.

// Serial and I/O conversion variables.
extern int DecArg;		// Holds value to manipulate or output.

// Definitions of what is displayed on the LCD.
enum DISPLAY_MODES {
	KEY_MENU_MODE,	
	ADJUST_MODE,		
	INFO1_MODE,		
	INFO2_MODE,		
	INFO3_MODE,		
	INFO4_MODE,		
	RUN_MODE,		
	MSG_FIND_BEGIN_MODE,		
	MSG_THREAD_END_MODE,		
	MSG_INSERT_TOOL_RQ_MODE,	
	MSG_RETRACT_TOOL_RQ_MODE,
	MSG_SPINDLE_OFF_MODE,	
	MSG_ESTOP_INPUT_ACTIVE,	
	MSG_LIMIT_INPUT_ACTIVE,	
	ERROR_MODE,				
	MSG_INSERT_TOOL_MODE,	
	MSG_RETRACT_TOOL_MODE,	
	MSG_SEQ_COMPLETE_MODE,	
	MSG_MOTOR_STOPPED_ERROR,	
	MSG_SPINDLE_TO_FAST_ERROR,
	MSG_MOTOR_DISTANCE_ERROR,
	MSG_TAPER_TOO_BIG_ERROR,
	MSG_CHANGE_JOG_DISTANCE_MODE,
	MSG_START_EQUALS_END_ERROR 
};

/*
	Mainline States
*/
enum SYSTEM_STATES  {
	MACHINE_RUNNING,		// When automated movement is in progress
	MACHINE_READY,			// When manual movement is in progress or can happen.
	MACHINE_IDLE,			// No manual movement allowed, just parameter entry.
	MACHINE_STARTING,		// What happens just before RUNNING
	MACHINE_STOPPING,		// What happends after RUNNING.
	MACHINE_ERROR,			// Machine is in error
	MACHINE_INIT 			// Initialize everything.
};
// Holds mainline states.
extern enum SYSTEM_STATES SystemState;

/*
	Full diagnostics allows lots of print statemets out the serial port.  Normally the code should be 
	compiled with this off as it just adds space and lots of extra processing time that impacts keypad
	reaction time.
*/
#ifdef FULL_DIAGNOSTICS
#define DEBUGSTR(s) printf((MEM_MODEL  rom char *)s)
#else
#define DEBUGSTR(s) 
#endif

#ifdef DEBUG_MENU_DIAGNOSTICS
#define DEBUG_MENU(s) printf((MEM_MODEL  rom char *)s)
#else
#define DEBUG_MENU(s) 
#endif


// Functions in ELeadscrew.c needed in other files.
void SetDisplayMode(char displayMode);

