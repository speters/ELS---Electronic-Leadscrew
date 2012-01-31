// KeyThread.h
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




enum ELSKEYS_STATES {
	ELSKEYS_INIT,
	ELSKEYS_KEY_WAIT,
	ELSKEYS_KEY_TAPPED,
	ELSKEYS_KEY_HOLD,
	ELSKEYS_KEY_DONE
};

extern enum ELSKEYS_STATES ELSKeys_State;


extern BITS KEYFlags;	// KeyThread Flags.
#define fNumberKeys	    KEYFlags.Bit.Bit0	
#define fPeriodEntered	KEYFlags.Bit.Bit1


void InitRunKeyThread(void);
void InitRdyKeyThread(void);
void InitIdleKeyThread(void);
void InitErrorKeyThread(void);

void RunKeyThread(void);
void RdyKeyThread(void);
void IdleKeyThread(void);
void ErrorKeyThread(void);
