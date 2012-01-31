// Movement Thread.h
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



extern int32 ZBeginPositionSteps, ZEndPositionSteps;
extern float32 ZBeginPosition, ZEndPosition;

extern int32 XRetractedPositionSteps, XBeginPositionSteps;
extern float32 XRetractedPosition, XBeginPosition;

extern int8 PassCount, SpringPassCount;


enum MOVEMENT_STATES  {

	MOVE_WAIT,
	MOVE_HOMING,
	MOVE_WAIT_X_OUT,
	MOVE_TO_START,
	MOVE_AT_START,
	MOVE_WAIT_START,
	MOVE_WAIT_X_DONE,
	MOVE_WAIT_X_BACKLASH_DONE,
	MOVE_WAIT_X_RDY,
	MOVE_TO_END,
	MOVE_AT_END,
	MOVE_WAIT_END_OUT,
	MOVE_WAIT_END,
	MOVE_WAIT_TO_START
};

extern enum MOVEMENT_STATES MovementState;
enum PASS_STATES {
	CALCULATE_PASSES,
	FIRST_PASS,
	ADJUST_PASS,
	EACH_PASS,
	LAST_PASS,
	SPRING_PASSES,
	FINISHED_PASSES
};

extern enum PASS_STATES PassState;

// Motor State Machine Run Control Flags
extern BITS MovementRunFlags;
#define fMoveStartRQ				MovementRunFlags.Bit.Bit0
#define	fLOkToStop 					MovementRunFlags.Bit.Bit1
#define fMovementThreadMode			MovementRunFlags.Bit.Bit2	
#define fExternalThreading			MovementRunFlags.Bit.Bit3
#define fBallCutting				MovementRunFlags.Bit.Bit4


void InitMovementThread(void);
void SetCalculatePositionState( enum PASS_STATES state );
void CalculatePasses(void);

void MoveHome(uint8 mtr, WORD spd);
void MovementStartThread(void);
void MovementStopThread(void);
void MovementThread(void);
