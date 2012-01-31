/* 
    MotorDriver.h -- MotorControl definitions for an electronic replacement of
    gears used for threading or hobbing on a Lathe or Gear Hobber.

    Copyright (C) 2005,2006  John Dammeyer

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

	This code was developed with the support and help of the members of the
	http://groups.yahoo.com/group/E-LeadScrew/

    John Dammeyer
    johnd@autoartisans.com


   	Initial Version: 0.00a

   	Version changes: 

*/
// MotorDriver.h Definitions.

enum MOTOR_STATES {
	MOTOR_STOPPED,
	MOTOR_JOGGING
};
extern enum MOTOR_STATES;

enum RPM_STATES {
	RPM_ZERO,
	RPM_STARTING,
	RPM_STEADY,
	RPM_SLOWING
};
extern enum RPM_STATES;
	


enum MOTOR_TYPES {
	MOTOR_Z,
	MOTOR_X,
	MOTOR_A
};

#define MOTOR_ENABLED			1
#define MOTOR_DISABLED			0

#define SPINDLE_TURNING			1
#define SPINDLE_EITHER			0

#define SPEED_TRACK_SPINDLE		0		// 0 Speed means track spindle RPM at some ratio.

// Motor Direction flags.  Bit is set when direction increases DRO value (ie. more positive).
#define MOVE_RIGHT				1		// Away from headstock DRO goes positive.
#define MOVE_LEFT				0		// To headstock, DRO goes smaller or even negative.

#define MOVE_OUT				1		// Diameter increases and DRO goes larger or positive.	
#define MOVE_IN					0		// Diameter decreases and DRO goes smaller or negative.

/*
	The PULSE_CLOCK_xxx constants are fundamental to the operation of the ELS.  Based on the processor
	Crystal Frequency and divisors the PULSE_CLOCK_RATE determines how often the interrupt routine is 
	called which in turn determines the fastest step rate.
	As stated in the comments below the project started at 25kHz and 43uS per clock tic and is now 
	at 20kHz and 50uS per clock tick.  That's in order to handle the increased number of 'features' 
	that the ELS has so that the interrupt routine doesn't take up more than about 75% of the interrupt
	time.  Otherwise there wouldn't be time to update the screen or respond to the buttons.
*/
#define PULSE_CLOCK_DIVISOR		500 // 435  // Was 400 which gave 25kHz (40uS).  Now 435 for 23kHz. (about 43uS)
											// Now 500 for 20kHz and 50 uS tick
#define PULSE_CLOCK_RATE  		((uint16)(10000000L/PULSE_CLOCK_DIVISOR))

/*
 *	EEROM BIT FLAGS
 *		Bits are define with a constant for the bit position.  It's this bit position that is used
 * 		to test the appropriate bit in the EEROM image from the menu fields.
 *		Move a bit, make sure and move the index.  The Menu's also make use of this index.
 */
// Hardware specific Z Motor information loaded from EEROM.
extern BITS ZMotorFlags;
#define	fBITPOS_MZInvertDirMotor		0 // 0x01 If set, inverts Motor direction.
#define	fBITPOS_MZInvertedStepPulse		1 // 0x02 If Set invert Step pulse.
#define fBITPOS_MZMetric1				2	// 0x04 If set use millimetres for Z axis.
#define fBITPOS_TrackSpindle			4	// 0x10 Real time update of stepper relative to spindle.
#define fBITPOS_HalfNut					5	// 0x20 Electronic Half nut control on AtoD Channel 0 enable
#define fBITPOS_TaperInwards			6	// 0x20 Electronic Half nut control on AtoD Channel 0 enable

#define	fMZInvertDirMotor			ZMotorFlags.Bit.Bit0 // 0x01 If set, inverts Motor direction.
#define	fMZInvertedStepPulse		ZMotorFlags.Bit.Bit1 // 0x02 If Set invert Step pulse.
#define fMZMetric1					ZMotorFlags.Bit.Bit2	// 0x04 If set use millimetres for Z axis.
#define fTrackSpindle				ZMotorFlags.Bit.Bit4	// 0x10 Real time update of stepper relative to spindle.
#define fHalfNut					ZMotorFlags.Bit.Bit5	// 0x20 Electronic Half nut control on AtoD Channel 0 enable
#define fTaperInwards				ZMotorFlags.Bit.Bit6	// 0x20 Electronic Half nut control on AtoD Channel 0 enable

// Hardware specific X Motor information loaded from EEROM.
extern BITS XMotorFlags;
#define	fBITPOS_MXInvertDirMotor		0 	// 0x01 
#define	fBITPOS_MXInvertedStepPulse		1 	// 0x02 
#define	fBITPOS_MXMetric				2 	// 0x04 
#define fBITPOS_MXDirectMode			3	// 0x08
#define fBITPOS_AutoX					4	// 0x10 
#define fBITPOS_UseCompound				5	// 0x20
#define fBITPOS_Tapering				6	// 0x40
#define fBITPOS_XBackLash				7	// 0x80

#define	fMXInvertDirMotor			XMotorFlags.Bit.Bit0 // 0x01 If set, inverts Motor direction.
#define	fMXInvertedStepPulse		XMotorFlags.Bit.Bit1 // 0x02 If set Invert Step Pulse.
#define	fMXMetric					XMotorFlags.Bit.Bit2 // 0x04 If set use millimetres for X axis.
#define fMXDirectMode				XMotorFlags.Bit.Bit3	// If Direct Mode then the X displacement is on the diameter
															// ie. 0.001" indication changes the diameter by 0.001" or the radius by 0.0005
															// When not direct or Indirect, a 0.001" indication moves the carriage 0.001"
#define fAutoX						XMotorFlags.Bit.Bit4	// 0x10 X axis is not connected a stepper.
#define fUseCompound				XMotorFlags.Bit.Bit5	// If set then use compound for manual tool movement -- Prompt compound distance.
															// Otherwise, prompt cross slide distance.
#define fTapering					XMotorFlags.Bit.Bit6	// Move X on tangent of Z.
#define fXBackLash					XMotorFlags.Bit.Bit7	// X Movement for backlash adjustment.

// Run time flags not loaded from EEROM.
extern BITS ZStepFlags;
#define fZMoveRQ 					ZStepFlags.Bit.Bit0		// Request to start movement.
#define fZMoveBSY					ZStepFlags.Bit.Bit1		// Status that a move is underway.
#define fUpdatedRPM					ZStepFlags.Bit.Bit2		// The calculation for RPM has updated the value.
#define fZDeccel					ZStepFlags.Bit.Bit3		// Motor is decelerating.
#define fZUpToSpeed					ZStepFlags.Bit.Bit4		// Motor is up to speed.
#define fZDirectionCmd				ZStepFlags.Bit.Bit5		// Direction motor is turning.
#define fZStopping					ZStepFlags.Bit.Bit6		// Motor is stopping.
#define fStepHappened				ZStepFlags.Bit.Bit7		// A motor step occurred.

extern BITS XStepFlags;	// As in ZStep flags except where noted.
#define fXMoveRQ 					XStepFlags.Bit.Bit0	
#define fXMoveBSY					XStepFlags.Bit.Bit1
#define	fTaperDirection				XStepFlags.Bit.Bit2		// Used to determine which way X moves for tapering
#define fXDeccel					XStepFlags.Bit.Bit3
#define fXUpToSpeed					XStepFlags.Bit.Bit4
#define fXDirection					XStepFlags.Bit.Bit5
#define fXStopping					XStepFlags.Bit.Bit6
#define fXThreading					XStepFlags.Bit.Bit7		// Set when threading, cleared when turning.

// The Taper Flags help decide if backlash needs to be taken out of the X axis for turning tapers
// When the combination of these bits has even parity then Backlash needs to be removed.
extern BITS TaperFlags;
#define fMovingRight				TaperFlags.Bit.Bit0		// Z axis will move right.
#define fMovingOut					TaperFlags.Bit.Bit1		// X axis just moved Out
#define	fTaperIn					TaperFlags.Bit.Bit2		// Tapering towards headstock.
#define fCrossFeed					TaperFlags.Bit.Bit3		// X moves and Z doesn't

extern enum MOTOR_STATES MotorState;

extern enum MOTOR_TYPES ActiveMotor ;


extern int32 XStartMotorPosition;	// New X location after every pass as depth increases

extern volatile int32 ZMotorPosition;			// Absolute Motor Position.

extern volatile int32 XStepCount;

extern  volatile uint16 SpindleClocksPerRevolution;	// Holds last accumulated number of clocks per Rev.

extern int32 RPMAverage[16];		// Holds the last 16 SpindleClocksPerRevolution
extern int32 AveragedClocksPerRev;	// Average of RPMAverage[] array
extern int AverageRPM;				// Average RPM
extern int TargetRPM;				// 
extern int32 LastSpindleClocks;

void InitMotorDevice(void);
void MotorDevice(void);
void Thread_Controller(void);
void ParameterSet(void);

float32 StepsToZDistance( int32 steps );
int32 ZDistanceToSteps( float32 distance);

float32 StepsToXDistance( int32 steps );
int32 XDistanceToSteps( float32 distance);

int32 CalculateMotorDistance( float32 dist, float32 divisor, int8 units ); 

void MotorStop(int8 device);

void MicroStep( int8 phaseA, int8 phaseB);
uint16 MotorJog(int8 device, uint16 speed, uint8 dir);

int8 MotorMoveDistance( int8 device, int32 distance, uint16 speed, uint8 dir, int8 SpindleON, /*int8 track,*/ int8 useLimit  );
int8 MotorMoveTo( int8 device, 	// Motor X or Z
		   int32 position, 	// Where to in seps
		   uint16 speed, 		// Speed in Hz up PULSE_POINT
		   int8 SpindleON 	// Spindle needs to be turning if true
		   /* int8 track */		// Track spindle speed 
		   );

int PrintRPM(int8 showSerial, int8 fShowSFM);
