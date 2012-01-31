/* 
    Int.c -- Interrupt routines for an electronic replacement of
    gears used for threading or hobbing on a Lathe or Gear Hobber.

    Copyright (C) 2005,2006,2007  John Dammeyer

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

    Special thanks to Dave Hyland and Art Fenerty for the interrupt 
	code suggestions.

    John Dammeyer
    johnd@autoartisans.com


   	Initial Version: 0.00a

		Uses Clocks.Fraction method of deciding when to issue a step pulse. 
		Clocks and Fraction are both 16 bit variables. 

   	Version changes: 
		
		0.10a

		Upgraded to use 32 bit math for Bresenham algorithm for creating step pulses thanks to
		some help from Art Fenerty who wrote the MACH2/MACH3 CNC software.

	    Code suggestions from Art (a fragment of his Jog code) were then rewritten for the PIC
	    environment to do both continuous jog motion and fixed distance move with decceleration 
	    starting automatically so motor is stopped smoothly. 

		1.00y
		Code is close to being ready to release.  
			X and Y axis function correctly and use less processor time now that each 
			axis are individually enabled with a flag.
			MPG functions
			RPM works
			LIMITS and ESTOP function.
			MicroStepping works but needs an upgrade to deal with LMD18245 direction change bug.
		1.02e
		Added fZMoveBSY = 0; when fZAxisActive is cleared.  The code to clear it was to  
		speed up the interrupt routine.  But there were occasions where it would never clear fZMoveBSY 
		and therefore leave the movement thread not oktostop.
		1.02r 
		Did the same for fXMoveBSY as in 1.02e.  It would cause x jog to no longer work.  Suspect, same issue
		with sometimes stopping and waiting during programmed moves.
		-----
		* No code yet for MAX5464 digital pots.
		* Look into changing LMD18245 into full step after a certain speed.  ie. Just change to a table that
		  doesn't micro-step but instead has full step waveform for more torque.
		1.03m
		Upgraded to 4 tables to handle LMD18245 direction change bug.  Although 4 tables may seem 
		wasteful of code the speed tradeoff is worth it.  Code is only slightly slower now because we have 
		to test which direction the motor is going in order to use the correct table.  But with a bit of 
		re-arranging the extra test was only added to the first SPIBUF write operation and code moved around
		to take advantage of the existing test on the second write.
		So what changed?  The Direction pin reflected in BIT 5 must change only when also writing 0000 to 
		the D/A pins.  Change it after current is 0 and there's a spike of current.  On the old version of
		the table it shows up in one direction of movement but not the other.  There are claims that other 
		systems based on the LMD18245 were noisy and ran slower in one direction than the other.  This did
		not seem to be the case with the way the ELS does the D/A stepping but better to be safe than sorry.
		
		1.04
		Added code to clear active moves rather than only prevent steps while ESTOP is active.  This was 
		needed because if ESTOP is released and power to stepper motors there, then interrupt routine tries
		to finish move which causes motors to go who knows where.
		1.10
		Spindle tracking for spindle speed decrease.
		1.10a
		Spindle tracking for spindle speed increase too.
	
*/

#include "Processor.h"

#include "Common.h"
#include "Config.h"
#include "Menu.h"
#include "GlobVars.h"

#include "Int.h"
#include "Serial.h"
#include "Timer.h"
#include "MotorDriver.h"	// Motor Driver Defintions.



// *** PRIVATE DECLARATIONS *** 

// *** PRIVATE VARIABLES ***
// Spindle Tracking variables.
#ifdef TRACK_SPINDLE_SPEED
int16 SpinRate;				// Spindle Speed at start of threading
int16 NegSpinRate;			// Negative of Spindle Speed at start of threading
int16 SpinClip;				// Half of SpinRate calculated outside interrupt routine for speed.
int16 SpinAdder;
int16 SpinCorrection;		// Difference between Spindle Speed at start of threading and during threading
#endif

static int16 IndexDebounce = 0;		// Used to count Pulse clocks before re-enabling spindle interrupt.

static uint16 Zholder = 0;			// Bresenham accumulator
static int32 tempZvel;				// Temp to speed up the math.

// Spindle Sensor state machine.
enum SPINDLE_INT_STATES {
	SPINDLE_INT_NOT_TURNING,
	SPINDLE_INT_LEVEL_HI,
	SPINDLE_INT_LEVEL_LO,
	SPINDLE_INT_FEDGE_CHECK,
	SPINDLE_INT_REDGE_CHECK
} SpindleIntState;

#define INDEX_DEBOUNCE	4			// Number of pulse clocks that a new spindle sensor value must be active.

// *** PUBLIC VARIABLES ***
volatile int32 MaxZVel;				// Target Velocity * 2048
volatile int32 ZVel;  				// Current velocity * 2048
volatile int32 ZAcc; 				// Acceleration
volatile int32 NewZVel;				// Maximum RPM when a distance move is requested with fZMoveRQ
int32 ZBackLashCount;		// Used inside interrupt routine.
volatile int32 SystemZBackLashCount;		// Filled from EEROM takes into account half nut backlash.
volatile int32 ZStepCount;			// Number of steps to turn Leadscrew.
volatile int32 StepsToZVel;			// Number of steps it took to get to maximum speed.
volatile int32 ZMotorPosition; 		// absolute Motor Position as a signed # of encoder steps.
volatile int32 ZHalfwayPoint;		// Point at which we must start slowing down even if target velocity not reached.

volatile uint16 SpindleClockValue;	// Accumulates Pulse Clocks per Spindle Revolution

// Run time flags not loaded from EEROM.
BITS ActiveFlags;		// Used to control access to stepper interrupt code.



#ifdef X_AXIS
// *** PRIVATE VARIABLES ***
static uint16 Xholder = 0;			// Bresenham accumulator
static int32 tempXvel;				// Temp to speed up the math.

// *** PUBLIC VARIABLES ***
volatile int32 MaxXVel;				// Target Velocity * 2048
volatile int32 XVel;  				// Current velocity * 2048
volatile int32 XAcc; 				// Acceleration
volatile int32 NewXVel;				// Maximum RPM when a distance move is requested with fZMoveRQ

volatile int32 XMotorRelPosition; 		// Relative Motor Position as a signed # of encoder steps.
volatile int32 XMotorAbsPosition; 		// Absolute Motor Position as a signed # of encoder steps.
volatile int16 XMotorIncrement;		// Used to track steps for absolute rather than relative X motion.
volatile int32 XBackLashCount;
volatile int32 StepsToXVel;			// Number of steps it took to get to maximum speed.
#endif

#define X4_ENCODER	1

#ifdef X4_ENCODER
// *** PRIVATE VARIABLES ***
static int8 NewEnc, OldEnc;
static int8 EncoderCounter;
#endif


#ifdef TAPERING
/*
 For every Z step we step X tan(taperangle).  To get reasonable accuracy
 TaperTangent is 65536 * tan(taperangle).  This assumes the X moves the same distance per step
 as the Z.
   For example:
		If the taper angle was 45 degrees then for very Z step we have an X step since tan(45) == 1
		So TaperTangent has the value tan(45) * 65536 which == 65536.
		On every step, adding 65536 (0x10000) to the accumulator results in an overflow but nothing
		in the low word.  Perfect 45 degree tracking.
		For a Morse #2 which is 0.04995"/inch = tan(theta).  (Theta is 1/2 the included angle)
			so 0.04995 * 65536 == 3274.
		For every step on Z we accumulate 3274 in the TAccumulator.Bresenham.
		When it rolls over into the High word we step X once and clear the High Word leaving the low word
		alone.
		In fact, the value of the TaperTangent is based on what fraction of an inch that Z moves per 
		step and what fraction X moves.
		So if the ratios are such that if Z moves one step it moves the carriage 0.0005" and each X 
		step moves the cross slide 0.00025" then Z/X * TaperTangent is the amount that X really adds 
		to the bresenham accumulator; in this example, X needs to step twice as much or the 
		TaperTangent is 6547 (2 * 3273.53)
		These constants are all worked out before any motion takes place.

*/
union BRESENHAM {
  int32	Bresenham;
  struct {
	uint16  Low;
	uint16  High;
  };
} TAccumulator;

int32 TaperTangent;
 
#endif


#ifdef MICRO_STEPPING
// *** PUBLIC VARIABLES ***
int8 PhaseAIndex = 0;
int8 PhaseBIndex = 8;
int8 MotorOffDelay = 0;

/*
Bit pattern definition for MicroStepTable.
  Common to Phase A and B
	Bit 0..3 are M1..M4 pins of LMD18245
	Bit 4 is BRAKE pin of LMD18245
	Bit 5 is DIRECTION pin of LMD18245
  For Phase A
	Bit 6 of Device Phase A is U/D pin of MAX5464 programmable resistor.
	Bit 7 of Device Phase A is _CS of MAX5464 programable resistor.
  For Phase B
	Bit 6 is unallocated.
	Bit 7 of Device Phase B when low reduces Reference voltage for low power stepper drive.
*/

// Forward direction table for Phase A coil
rom uint8 
MicroStepATableFwd[MSTEPTABLESIZE] = {
	0b10100000,
	0b10100011,
	0b10100110,
	0b10101000,
	0b10101011,
	0b10101100,
	0b10101110,
	0b10101111,
	0b10101111,
	0b10101111,
	0b10101110,
	0b10101100,
	0b10101011,
	0b10101000,
	0b10100110,
	0b10100011,
	0b10000000,
	0b10000011,
	0b10000110,
	0b10001000,
	0b10001011,
	0b10001100,
	0b10001110,
	0b10001111,
	0b10001111,
	0b10001111,
	0b10001110,
	0b10001100,
	0b10001011,
	0b10001000,
	0b10000110,
	0b10000011
};
// Forward direction table for Phase B coil.
rom uint8
MicroStepBTableFwd[MSTEPTABLESIZE] = {
	0b10101111,
	0b10101111,
	0b10101110,
	0b10101100,
	0b10101011,
	0b10101000,
	0b10100110,
	0b10100011,
	0b10000000,
	0b10000011,
	0b10000110,
	0b10001000,
	0b10001011,
	0b10001100,
	0b10001110,
	0b10001111,
	0b10001111,
	0b10001111,
	0b10001110,
	0b10001100,
	0b10001011,
	0b10001000,
	0b10000110,
	0b10000011,
	0b10100000,
	0b10100011,
	0b10100110,
	0b10101000,
	0b10101011,
	0b10101100,
	0b10101110,
	0b10101111
};
// Reverse direction Table for Phase A coil
rom uint8 
MicroStepATableBkwd[MSTEPTABLESIZE] = {
	0b10000000,
	0b10100011,
	0b10100110,
	0b10101000,
	0b10101011,
	0b10101100,
	0b10101110,
	0b10101111,
	0b10101111,
	0b10101111,
	0b10101110,
	0b10101100,
	0b10101011,
	0b10101000,
	0b10100110,
	0b10100011,
	0b10100000,
	0b10000011,
	0b10000110,
	0b10001000,
	0b10001011,
	0b10001100,
	0b10001110,
	0b10001111,
	0b10001111,
	0b10001111,
	0b10001110,
	0b10001100,
	0b10001011,
	0b10001000,
	0b10000110,
	0b10000011
};

// Reverse direction Table for Phase B coil
rom uint8
MicroStepBTableBkwd[MSTEPTABLESIZE] = {
	0b10101111,
	0b10101111,
	0b10101110,
	0b10101100,
	0b10101011,
	0b10101000,
	0b10100110,
	0b10100011,
	0b10100000,
	0b10000011,
	0b10000110,
	0b10001000,
	0b10001011,
	0b10001100,
	0b10001110,
	0b10001111,
	0b10001111,
	0b10001111,
	0b10001110,
	0b10001100,
	0b10001011,
	0b10001000,
	0b10000110,
	0b10000011,
	0b10000000,
	0b10100011,
	0b10100110,
	0b10101000,
	0b10101011,
	0b10101100,
	0b10101110,
	0b10101111
};


// *** PUBLIC FUNCTIONS *** 
void 
MicroStep( int8 phaseA, int8 phaseB) {
	// Make step access to SPI indivisible so interrupt routine 
	// can't touch SPI registers.
	INTCON &= 0x3F;
	while (!SSPSTATbits.BF) 	// Wait for SPI to be ready.
		;
	SSPBUF = phaseB;			// First byte out.
	while (!SSPSTATbits.BF) 	// Wait for it.
		;
	SSPBUF = phaseA;			// Second Byte out
	while (!SSPSTATbits.BF) 	// Wait for it.
		;
	bMOTOR_LATCH = 0;			// 16 bits shifted
	bMOTOR_LATCH = 1;			// So latch all 16 at the same time onto the outputs.
	bMOTOR_LATCH = 0;
	INTCON |= 0xC0;
}


#endif

// *** PUBLIC FUNCTIONS *** 
void
InitInterruptVariables(void) {
	NewEnc = ENCODER_PORT & ENCODER_MASK;
	OldEnc = NewEnc;
	// Tapering Variables initialization.
#ifdef TAPERING
	TAccumulator.Bresenham = 0;
#endif
	ZBackLashCount = 0;
	SpindleIntState = SPINDLE_INT_NOT_TURNING;

	ZEncoderCounter = 0;

	ZMotorPosition = 0;				// We don't know where the motor is so we stop.
	// Assume that the X position hasn't been changed since the last time we powered up.
	XMotorAbsPosition = XDistanceToSteps( GetGlobalVarFloat(X_DIAMETER_NDX) ); // We don't know where the motor is
	XMotorRelPosition = 0;	

	SpindleClocksPerRevolution = 0;	// Don't know if the spindle is turning yet so spindle RPM is 0.

#ifdef TRACK_SPINDLE_SPEED
	SpinAdder = 0;
	SpinRate = 0;
	SpinCorrection = 0;
	SpinClip = 0;
#endif
}

// *** PRIVATE FUNCTIONS ***

/*
* For PIC18cxxx devices the high interrupt vector is found at
* 00000008h. The following code will branch to the
* high_interrupt_service_routine function to handle
* interrupts that occur at this low vector.
*/
void __INTH(void);
void high_vector_branch (void);

#ifdef BOOTLOADER
#pragma code high_vector=0x808
#else
#pragma code high_vector=0x08
#endif

void interrupt_at_high_vector(void) {
_asm
CALL high_vector_branch, 1
_endasm
}
#pragma code /* return to the default code section */

/*
	High Priority interrupt which is called for the 20KHz Pulse clock (CCP1) and the index pulse 
	on the spindle (INT0).

	SpindleClockValue is incremented on each CCP1 event and saved and cleared on each index pulse 
    providing a value from RPM and step rates can be calculated.

	The speed argument to the stepper is in either NewVel or MaxVel and is the stepping rate.

	The variable Vel holds the current stepper speed.  
	
	To accelerate or decelerate, the accelertion value Acc is added	or subtracted from the Vel variable
	depending on the fDeccel flag.  Once Vel is equal to MaxVel, Acc is no longer used.


    There are two ways to make the stepper turn:

	1. Continuous motion at a preset speed.
		The main application sets a new speed value into MaxVel.

	2. Move a distance at a preset speed.
		The main application sets a new value into NewVel and sets the fZMoveRQ Flag.

	    The application requests a move by setting fZMoveRQ.  When an Index pulse occurs, this function
		ACKs the command by clearing the fZMoveRQ and setting the fZMoveBSY. It also copies NewVel into MaxVel
		which starts the motor. 
		
	Normal elapsed time for each 50uS interrupt is about 23uS.  It jumps to about at least 40uS during deeper nesting for 
	a single step. 
		
*/
// From the errata sheet to save clock cycles
void high_vector_branch (void)
{
_asm
POP
GOTO __INTH
_endasm
}

// Note!! Due to PIC Microprocessor errata, we have to tell the compiler not to make a
// RETFIE FAST hence the interruptlow __INTH.
//#pragma interruptlow __INTH
#pragma interrupt __INTH

void 
__INTH(void) {
	if (PIE1bits.CCP1IE && PIR1bits.CCP1IF) {
	    PIR1bits.CCP1IF = 0;

		if (bESTOP ^ fEStop) {
			SystemError = MSG_ESTOP_INPUT_ACTIVE;
			// Kill any motion in progress so when ESTOP button is released motion doesn't restart.
			// Z Axis (Lead Screw)
			ZVel = MaxZVel = 0;	// Clear out velocity.
			fZAxisActive = 0;	// Disable Axis.
			fZMoveBSY = 0;  	// No move in progress.
			// X Axis (Cross Slide)
			XVel = MaxXVel = 0;	// Clear out velocity.
			fXAxisActive = 0;	// Disable Axis.
			fXMoveBSY = 0;  	// No move in progress.
			// Now return which ends up also stopping charge pump output since the bit is never set.
			// That should drop power off devices like Servo motors.
			return;
		}

		// For motor drivers with simple enable lines we don't use a charge pump.
		bCHARGE_Pump = 1;	// Also a good way to see approximately how long int32 interrupt routine takes.

		// Now the actual stepping code.
		if (fZAxisActive) {	// Jogging or Programmed Move.
			// MaxVel is the velocity set point.
			if (fZDeccel) {
				ZVel -= ZAcc;  				// if we're decelerating, subtract the acceleration
				if (ZVel < MaxZVel) { 		// Don't allow velocity to go negative
					fZDeccel = 0;			// We've reached our set point so no longer decelerating.
					ZVel = MaxZVel;			// In case we underflowed set it to the proper value.
					if (ZVel == 0) {		// With the velocity value now zero, ZHolder won't increment and
						fZAxisActive = 0;	// therefore no more steps so prevent access to this code now.
						fZMoveBSY = 0;  	// But do tell the world we've finished our move.
					}
				}
	   	 	}
			else {
				tempZvel = ZVel+ZAcc;
	    		if( tempZvel <= MaxZVel ) // LT or EQ allows us to reach set point.
					ZVel = tempZvel;   // otherwise accelerate if need be.
	        	else {
					ZVel = MaxZVel;  	//  to clip to max velocity
					fZUpToSpeed = 1;	// Reached maximum speed so stop counting distance to maximum speed.
				}
	    	}
			
#ifdef TRACK_SPINDLE_SPEED
			if ( fThreading && fZUpToSpeed ) {	// If up to speed allow adding in of ZVel into ZHolding register.
				SpinAdder += SpinCorrection;	// 
				if (SpinAdder > SpinRate) 		// If we've overflowed, then don't allow the addition which is like skipping one clock period.
					SpinAdder -= SpinRate;
				else
			    	Zholder += (ZVel >> 16); // holder is the main Bresenham variable count, it rolls over PULSE_POINT when a pulse is necessary.

/*
				if (SpinAdder < NegSpinRate)
					SpinAdder = NegSpinRate; 
*/
				if (SpinAdder < NegSpinRate) { // If Spindle is turning faster, than this will make the step happen sooner.
					SpinAdder += SpinRate; 		
			    	Zholder += (ZVel >> 16); // holder is the main Bresenham variable count, it rolls over PULSE_POINT when a pulse is necessary.
				}


			}
			else	// Not up to speed so just allow acceleration to happen.
#endif
	    		Zholder += (ZVel >> 16); // holder is the main Bresenham variable count, it rolls over PULSE_POINT when a pulse is necessary.
	
	    	if( Zholder >= PULSE_CLOCK_RATE ) { // PULSE_POINT = inverse of our interrupt period -- maximum step frequency.
				// In the following code we potentially invert the direction bit but only do that once. (faster interrupt routine)
				// Then determine if direction has changed and also set the new direction
				if ((fZDirectionCmd ^ fMZInvertDirMotor) == MOVE_RIGHT)  { // Test possible new direction of motor.
					 if ( !bMZDirectionMotor ) {	// if current direction also not a 1 then direction has changed.
						ZBackLashCount = SystemZBackLashCount;	// So first do backlash motion.
						ZStepCount += ZBackLashCount;	// Increase distance by backlash.
					}
					bMZDirectionMotor = MOVE_RIGHT;		// This is faster than doing the XOR on the direction bit to change
				}										// the polarity since we know it's 1 at this point.				
				else { // Direction of motor is LEFT
					 if (bMZDirectionMotor) {	// if not also a 0 then direction has changed.
						ZBackLashCount = SystemZBackLashCount;
						ZStepCount += ZBackLashCount;
					}
					bMZDirectionMotor = MOVE_LEFT;
				}

#ifdef MICRO_STEPPING
				if (fZLocalMicroStep) {	// Send out first byte which will end up at PhaseB device U5
					// Since direction may have changed we have to point to the new location after 
					// the direction has been set up.
					PhaseBIndex = PhaseAIndex;	// Set new location.
					if (fZDirectionCmd ^ fMZInvertDirMotor) 
						SSPBUF = MicroStepBTableBkwd[PhaseBIndex];
					else
						SSPBUF = MicroStepBTableFwd[PhaseBIndex];
				}
				else {
					bMZStepMotor = 1 ^ fMZInvertedStepPulse;  // STEP
				}  
#else
					bMZStepMotor = 1 ^ fMZInvertedStepPulse;  // STEP
#endif
	  
	        	Zholder -= PULSE_CLOCK_RATE;  // wait for next one;  //then subtract PULSE_POINT from the accumulator..
				// Any overflow stays in accumulator resulting in the next pulse coming sooner.
	
				// Update DRO only if we're not doing backlash compensation.
				if (ZBackLashCount == 0) {
					if (fZDirectionCmd == MOVE_RIGHT) 	// Test if MOVE_RIGHT	
						ZMotorPosition++; 	// MOVE_RIGHT
					else 
						ZMotorPosition--;	// MOVE_LEFT
	#ifdef TAPERING
					// Only taper on X when we're _not_ doing backlash compensation.
					if (fTapering) {
						// This particular bit of addition lends itself better to assembler where
						// we could just add the low 16 bits and check the carry flag.
						TAccumulator.Bresenham += TaperTangent;		// Add in tangent.	
						if (TAccumulator.High > 0) {				// Overflow?
							TAccumulator.High = 0;					// clear overflow
							// remainder of overflow is still in Low. Now step.
							// MOVE_LEFT makes fZDirectionCmd 0.  fTaperDirection is 1 if moving inwards to headstock
							//		decrement X position
							// MOVE_RIGHT makes fXDirectionCmd 1.  fTaperDirection is 1 if moving inwards to headstock
							// 		Increment X position.
							// MOVE_LEFT makes fZDirectionCmd 0.  fTaperDirection is 0 if moving outwards to headstock
							//		increment X position
							// MOVE_RIGHT makes fXDirectionCmd 1.  fTaperDirection is 0 if moving outwards to headstock
							// 		deccrement X position.
							// So update DRO and step.
							if ((fZDirectionCmd ^ fTaperDirection) == 1) { //MOVE_IN) {
								bMXDirectionMotor = MOVE_IN ^ fMXInvertDirMotor; 
								XMotorRelPosition--; 	// decrement X position.
								XMotorIncrement--;		// External thread will track absolute position using this variable.
							}
							else {
								bMXDirectionMotor = MOVE_OUT ^ fMXInvertDirMotor; 
								XMotorRelPosition++;	// So increment X position.
								XMotorIncrement++;
							}
							bMXStepMotor = 1 ^ fMXInvertedStepPulse;  // STEP
						}
					}
	#endif
				}
				else
					ZBackLashCount--;

							
#ifdef MICRO_STEPPING
				// Another Step Done
				if (fZLocalMicroStep) {
					// Make sure SPI byte has been completely transmitted.
					while (!SSPSTATbits.BF) 
						;
					// Now send out second byte for phase A coil which ends up in U4
					// Then increment or decrement pointer
		  			// to set up for next table entry while Phase A is being shifted out.
					if (fZDirectionCmd ^ fMZInvertDirMotor) {
						SSPBUF = MicroStepATableBkwd[PhaseAIndex];  		// Get second Byte
						--PhaseAIndex;
					}
					else {
						SSPBUF = MicroStepATableFwd[PhaseAIndex];  		// Get second Byte
						++PhaseAIndex;
					}
					// wrap it.
					PhaseAIndex &= MSTEPTABLEMASK;

					// Should be done sending via 10Mhz SPI but check anyway.
					while (!SSPSTATbits.BF) 
						;
					// Transfer data from serial shift register to parallel outputs
					// on 74HC595.
					bMOTOR_LATCH = 1;
					fStepHappened = 1;	// Tell world a step occurred.
					bMOTOR_LATCH = 0;
				}
				else 
	       			bMZStepMotor = 0 ^ fMZInvertedStepPulse; // Finish step pulse.
#else
	       			bMZStepMotor = 0 ^ fMZInvertedStepPulse; // Finish step pulse.	       			
#endif

#ifdef TAPERING
				if (fTapering) {
		   			bMXStepMotor = 0 ^ fMXInvertedStepPulse; // Finish step pulse.
				}
#endif	
				if (fZMoveBSY) {	// We're doing a distance move rather than a jog.
					// Check if we're trying to move off a limit switch or accidentally ran into it.
					if (fUseLimits && (bLIMIT_Switch ^ fLimitSwitch)) {
						ZStepCount = 0;
						ZEncoderCounter = 0;	// Trash any MPG counts
						fZMoveRQ = 0;
						fZMoveBSY = 0;
						SystemError = MSG_LIMIT_INPUT_ACTIVE;
					}
					if (ZStepCount-- <= StepsToZVel) {
						StepsToZVel = -1;	// Cancel this so we only do it once.
						MaxZVel = 0; 		// Start decelerating down to 0.
						fZDeccel = 1;
						fZUpToSpeed = 1;		// Fake out up to speed even if we're deccelerating
											// before we reach it.
						// Turn off automatic tracking of spindle speed.
						fThreading = 0;
						MotorState = MOTOR_STOPPED;	
					}
					else if (ZStepCount <= 0) {
						ZVel = 0;
						MaxZVel = 0;		// Now stop.
						Zholder = 0;		// And don't come back in until new MaxVel set.
						// Turn off automatic tracking of spindle speed.
						fThreading = 0;
						MotorState = MOTOR_STOPPED;	
						fZMoveBSY = 0;
					}
				}
	
				// Test whether time to decelerate to next velocity based on distance travelled.
				if (!fZUpToSpeed) {  //  Up to speed?  
					StepsToZVel++;	// No.  Keep counting steps.  Used for determining when to decelerate for fixed moves.
				}
	   		}
		}

#ifdef X_AXIS_INTERRUPT
		// MaxVel is the velocity set point.
		if (fXAxisActive /*|| fXMoveBSY*/) {	// Jogging or programmed move.
			if (fXDeccel) {
				XVel -= XAcc;  //if we're decelerating, subtract the acceleration
				if (XVel < MaxXVel) { 		// Don't allow velocity to go negative
					fXDeccel = 0;			// We've reached our set point so no longer decelerating.
					XVel = MaxXVel;			// In case we underflowed set it to the proper value.
					if (XVel == 0) {		// Our velocity was zero and we've reached it.
						fXAxisActive = 0;	// So motor isn't moving anymore.
						fXMoveBSY = 0; 		// And if we were busy moving, we're not anymore.
					}
				}
	   	 	}
			else {
				tempXvel = XVel+XAcc;
	    		if( tempXvel <= MaxXVel ) // LT or EQ allows us to reach set point.
					XVel = tempXvel;   // otherwise accelerate if need be.
	        	else {
					XVel = MaxXVel;  //  to clip to max veleocity
					fXUpToSpeed = 1;
				}
	    	}
			
	    	Xholder += (XVel >> 16); // holder is the main Bresenham variable count, it rolls over 25000 when a pulse is necessary.
	
	    	if( Xholder >= PULSE_CLOCK_RATE ) { // pulse point = 25000hz.
	
				bMXDirectionMotor = fXDirection;	// Set direction.
				bMXStepMotor = 1 ^ fMXInvertedStepPulse;  // STEP
	
	  
				// Done test of 32 bits
	        	Xholder -= PULSE_CLOCK_RATE;  // wait for next one;  //then subtract 25000 from the accumulator..
		
				if (--XBackLashCount < 0) {
					if (fXDirection ^ fMXInvertDirMotor) {
						XMotorRelPosition++; 
						XMotorIncrement++;		// External thread will track absolute position using this variable.

					}
					else {
						XMotorRelPosition--;
						XMotorIncrement--;		// External thread will track absolute position using this variable.
					}
					XBackLashCount = 0;
				}				
				// Test whether time to decelerate to next velocity based on distance travelled.
				if (!fXUpToSpeed)  //  Up to speed?  
					StepsToXVel++;	// No.  Keep counting steps.  Used for determining when to decelerate for fixed moves.
	
				
				if (fXMoveBSY) {	// We're doing a distance move rather than a jog.
					if (fUseLimits && (bLIMIT_Switch ^ fLimitSwitch)) {
						XStepCount = 0;
						ZEncoderCounter = 0;	// Trash any MPG counts
						SystemError = MSG_LIMIT_INPUT_ACTIVE;
					}
					if (XStepCount-- == StepsToXVel) {
						MaxXVel = 0; 		// Start decelerating down to 0.
						fXDeccel = 1;
						fXUpToSpeed = 1;		// Fake out up to speed even if we're deccelerating
												// before we reach it.
					}
					else if (XStepCount <= 0) {
						XVel = 0;
						MaxXVel = 0;		// Now stop.
						Xholder = 0;		// And don't come back in until new MaxVel set.
						fXMoveBSY = 0;
					}
				}
	   			bMXStepMotor = 0 ^ fMXInvertedStepPulse; // Finish step pulse.
	   		}
		}
#endif
		// We time in Pulse Clocks how long it takes for one revolution.
		// The time is from rising edge to rising edge for a one slot per rev counter.
		// New Levels need to be active for INDEX_DEBOUNCE pulse clocks before the state change
		// is considered valid.  At 50uS per PULSE_CLOCK the default is 4 so noise glitches
		// less than 200uS are ignored.
		switch (SpindleIntState) {

		  case SPINDLE_INT_LEVEL_HI : 
			SpindleClockValue++;
			if (bSPINDLE == 0) {	// Possible falling clock?
				IndexDebounce = INDEX_DEBOUNCE;	
				SpindleIntState = SPINDLE_INT_FEDGE_CHECK;
			}
			break;

		  case SPINDLE_INT_LEVEL_LO : 
			SpindleClockValue++;
			if (bSPINDLE == 1) {
				IndexDebounce = INDEX_DEBOUNCE;			// Count 4 PulseClocks to validate edge.
				SpindleIntState = SPINDLE_INT_REDGE_CHECK;
			}
			break;

		  case SPINDLE_INT_FEDGE_CHECK :
			SpindleClockValue++;
			if (bSPINDLE == 0) {	// Still Low?
				if (--IndexDebounce < 0) {
					SpindleIntState = SPINDLE_INT_LEVEL_LO;
				}
			}
			else
				SpindleIntState = SPINDLE_INT_LEVEL_HI;	// Back to still high.
			break;

		  // Validate edge for INDEX_DEBOUNCE clock pulses.
		  // If still true then level is indeed high so set up Spindle Clock counter
		  // and save current revolution.
		  case SPINDLE_INT_REDGE_CHECK : 
			SpindleClockValue++;
			if (bSPINDLE == 1) {
				if (--IndexDebounce < 0) {
					SpindleClocksPerRevolution = SpindleClockValue;
					SpindleClockValue = INDEX_DEBOUNCE;
					// Let Device Driver know there's a new RPM.
					fUpdatedRPM = 1;
					// And that we saw the sensor.
					fSpindleInterrupt = 1;
					fSpindleTurning = 1;
#ifdef TRACK_SPINDLE_SPEED
					// If there's a move in progress then track spindle.
					if (fZMoveBSY) {						// If we've got a move going, calculate SpindCorrection
						SpinCorrection = SpindleClocksPerRevolution - SpinRate;
						if (SpinCorrection > SpinClip)
							SpinCorrection = SpinClip;
					}
#endif
					if ( fZMoveRQ ) {	   					// Always start threading at index pulse.
						fZMoveRQ = 0;						
						fZMoveBSY = 1;						// Ack Request	
						fZAxisActive = 1;					// Allow Z axis movement.
		
#ifdef TRACK_SPINDLE_SPEED
						SpinCorrection = 0;  SpinAdder = 0;	// Restart accumulators
#endif
						MaxZVel = NewZVel;					// NewVel is already shifted 16 bits.
						fZDeccel = 0;
						StepsToZVel = 0;		// Keep track of how far to get to velocity.
						fZUpToSpeed = 0;
						fZStopping = 0;
					}
					SpindleIntState = SPINDLE_INT_LEVEL_HI;
				}
			}
			else {
				SpindleIntState = SPINDLE_INT_LEVEL_LO;
			}
			break;

			case SPINDLE_INT_NOT_TURNING :
			break;
		}
	}

	if (INTCONbits.INT0E && INTCONbits.INT0F) {		// Spindle Encoder Interrupt
		INTCONbits.INT0F = 0;
		// Falling edge occurred.  Let State machine sort out RPM
		INTCONbits.INT0E = 0;
		SpindleIntState = SPINDLE_INT_LEVEL_HI;

	}

	
	/*
		Check Quadrature encoder knob.
	*/
#ifdef X4_ENCODER
	NewEnc = ENCODER_PORT & ENCODER_MASK;
	EncoderCounter = 0;
	if (NewEnc ^ OldEnc) { // Encoder value changed???
		switch(NewEnc) {
		case ENCODER_POS1 :	              // 01
			if (OldEnc == ENCODER_POS0)	  // 00
				EncoderCounter--;
			else 
				EncoderCounter++;
			break;
		case ENCODER_POS3 :				  // 11
			if (OldEnc == ENCODER_POS1)	  // 01
				EncoderCounter--;
			else 
				EncoderCounter++;
			break;
		case ENCODER_POS2 :				  // 10
			if (OldEnc == ENCODER_POS3)	  // 11
				EncoderCounter--;
			else 
				EncoderCounter++;
			break;						  // 00
		case ENCODER_POS0 :				  // 10
			if (OldEnc == ENCODER_POS2)
				EncoderCounter--;
			else 
				EncoderCounter++;
			break;
		};
		OldEnc = NewEnc;
		// Allow MPG movement only when machine is in READY state.
		// We still allow the Encoder tracking because otherwise when we leave 
		// ready state, if the encoder was moved, it would generate a step.
		// Also, we now have the opportunity to use the encoder knob when the machine 
		// is in the IDLE state to use the encoder counts to scroll through menus etc.
		if (SystemState == MACHINE_READY) 
			ZEncoderCounter += EncoderCounter;
	}; // end if encoder value changed.
#endif


	bCHARGE_Pump = 0;  // For motors with a simple enable line we don't do charge pump
}   

/*
* For PIC18cxxx devices the low interrupt vector is found at
* 00000018h. The following code will branch to the
* low_interrupt_service_routine function to handle
* interrupts that occur at the low vector.
*/

void __INTL(void);

#ifdef BOOTLOADER
#pragma code low_vector=0x818
#else
#pragma code low_vector=0x18
#endif

void 
interrupt_at_low_vector(void) {
	_asm GOTO __INTL _endasm
}

#pragma code /* return to the default code section */
#pragma interruptlow __INTL
/*
	Low priority interrupt which deals with the serial port and the 10mS Thread timer to be
	able to create Program Delays in a non-preemtive threaded code.
	Keypad and LCD support will eventually go here too.
*/
void 
__INTL(void) {


    RxCharDevice();

#ifdef TX_INTERRUPT_ENABLED
	TxCharDevice();
#endif

#ifdef TIMER_INTERRUPT_ENABLED
	if(INTCONbits.T0IF)	{		// if Timer overflow.
		INTCONbits.T0IF = 0;
		// Reload counter for correct number of ticks till overflow 
		// and the next 10ms interrupt.
		TMR0H = RTC_DIVISOR>>8;	
		TMR0L = RTC_DIVISOR;
		// increment our counter.
		TickCount++;
	}
#endif
}

