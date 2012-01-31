#ifndef __EE18_H
#define __EE18_H

extern volatile near unsigned char       EECON1;
extern volatile near union {
  struct {
    unsigned RD:1;
    unsigned WR:1;
    unsigned WREN:1;
    unsigned WRERR:1;
    unsigned FREE:1;
    unsigned EECON1_5:1;
    unsigned CFGS:1;
    unsigned EEPGD:1;
  };
} EECON1Bits;


//#define Enable_EE_WR_Int   INTCON.EEIE = 1
#define Enable_EE_WR       EECON1Bits.WREN = 1
#define Disable_EE_WR      EECON1Bits.WREN = 0
#define EEProm_Rd          EECON1Bits.RD   = 1
//#define Global_Int_Disable INTCON.GIE  = 0
//#define Global_Int_Enable  INTCON.GIE  = 1
//#define Clr_EE_Int         INTCONBits.EEIF = 0
#define EEProm_WR_Err      EECON1Bits.WRERR
#define Do_EE_Write        _asm BSF 0x08,1,0 _endasm
#define EEProm_Writing     EECON1Bits.WR
//#define EEProm_WR_Int      EECON1.EEIF

extern WORD EEROMAddress;
#define EEROMAdrHigh (BYTE)(EEROMAddress>>8)	
#define EEROMAdrLow  (BYTE)EEROMAddress		
extern WORD EEROMIndex;

void Put_ObEEROM_Byte( WORD addr, BYTE data );
void Put_ObEEROM_Word( WORD addr, WORD data );
void Put_ObEEROM_Float( WORD addr, float * pdata );
void Put_ObEEROM_Buffer( WORD addr, BYTE * pdata, BYTE len );
BYTE Get_ObEEROM_Byte(WORD addr);
WORD Get_ObEEROM_Word(WORD addr);
void Get_ObEEROM_Float(WORD addr, float * pdata);
void Get_ObEEROM_Buffer(WORD addr, BYTE * pdata, BYTE len); 

#endif
