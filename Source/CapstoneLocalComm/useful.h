
#ifndef __useful
#define __useful
#include <ctype.h>
#include <stdlib.h>
#include <stddef.h>
#include <math.h>
#include <string.h>
#include <setjmp.h>

typedef unsigned char byte ;


//The Following Structures and Defs are used to Simplify Named Bit Access
typedef struct Bits8 {
    volatile unsigned Bitx0 : 1 ;
    volatile unsigned Bitx1 : 1 ;
    volatile unsigned Bitx2 : 1 ;
    volatile unsigned Bitx3 : 1 ;
    volatile unsigned Bitx4 : 1 ;
    volatile unsigned Bitx5 : 1 ;
    volatile unsigned Bitx6 : 1 ;
    volatile unsigned Bitx7 : 1 ;
} Bits ;


#define B8_0(x) (((Bits *) (&x))->Bitx0)
#define B8_1(x) (((Bits *) (&x))->Bitx1)
#define B8_2(x) (((Bits *) (&x))->Bitx2)
#define B8_3(x) (((Bits *) (&x))->Bitx3)
#define B8_4(x) (((Bits *) (&x))->Bitx4)
#define B8_5(x) (((Bits *) (&x))->Bitx5)
#define B8_6(x) (((Bits *) (&x))->Bitx6)
#define B8_7(x) (((Bits *) (&x))->Bitx7)



//The Following Structures and Defs are used to Simplify Named Bit Access for 16
//Bit Flag Words
typedef struct WordBits {
    volatile unsigned Bitw0  : 1 ;
    volatile unsigned Bitw1  : 1 ;
    volatile unsigned Bitw2  : 1 ;
    volatile unsigned Bitw3  : 1 ;
    volatile unsigned Bitw4  : 1 ;
    volatile unsigned Bitw5  : 1 ;
    volatile unsigned Bitw6  : 1 ;
    volatile unsigned Bitw7  : 1 ;
    volatile unsigned Bitw8  : 1 ;
    volatile unsigned Bitw9  : 1 ;
    volatile unsigned Bitw10 : 1 ;
    volatile unsigned Bitw11 : 1 ;
    volatile unsigned Bitw12 : 1 ;
    volatile unsigned Bitw13 : 1 ;
    volatile unsigned Bitw14 : 1 ;
    volatile unsigned Bitw15 : 1 ;
} wBits16 ;
//Note the use of the address operator ..., somewhat simplifies the macro definitions...
#define B16_0(x)  (((wBits16 *)(&x))->Bitw0)
#define B16_1(x)  (((wBits16 *)(&x))->Bitw1)
#define B16_2(x)  (((wBits16 *)(&x))->Bitw2)
#define B16_3(x)  (((wBits16 *)(&x))->Bitw3)
#define B16_4(x)  (((wBits16 *)(&x))->Bitw4)
#define B16_5(x)  (((wBits16 *)(&x))->Bitw5)
#define B16_6(x)  (((wBits16 *)(&x))->Bitw6)
#define B16_7(x)  (((wBits16 *)(&x))->Bitw7)
#define B16_8(x)  (((wBits16 *)(&x))->Bitw8)
#define B16_9(x)  (((wBits16 *)(&x))->Bitw9)
#define B16_10(x) (((wBits16 *)(&x))->Bitw10)
#define B16_11(x) (((wBits16 *)(&x))->Bitw11)
#define B16_12(x) (((wBits16 *)(&x))->Bitw12)
#define B16_13(x) (((wBits16 *)(&x))->Bitw13)
#define B16_14(x) (((wBits16 *)(&x))->Bitw14)
#define B16_15(x) (((wBits16 *)(&x))->Bitw15)



//The Following Structures and Defs are used to Simplify Named Bit Access for 32
//Bit Flag Words
typedef struct LongBits {
    volatile unsigned Bitl0  : 1 ;
    volatile unsigned Bitl1  : 1 ;
    volatile unsigned Bitl2  : 1 ;
    volatile unsigned Bitl3  : 1 ;
    volatile unsigned Bitl4  : 1 ;
    volatile unsigned Bitl5  : 1 ;
    volatile unsigned Bitl6  : 1 ;
    volatile unsigned Bitl7  : 1 ;
    volatile unsigned Bitl8  : 1 ;
    volatile unsigned Bitl9  : 1 ;
    volatile unsigned Bitl10 : 1 ;
    volatile unsigned Bitl11 : 1 ;
    volatile unsigned Bitl12 : 1 ;
    volatile unsigned Bitl13 : 1 ;
    volatile unsigned Bitl14 : 1 ;
    volatile unsigned Bitl15 : 1 ;
    volatile unsigned Bitl16 : 1 ;
    volatile unsigned Bitl17 : 1 ;
    volatile unsigned Bitl18 : 1 ;
    volatile unsigned Bitl19 : 1 ;
    volatile unsigned Bitl20 : 1 ;
    volatile unsigned Bitl21 : 1 ;
    volatile unsigned Bitl22 : 1 ;
    volatile unsigned Bitl23 : 1 ;
    volatile unsigned Bitl24 : 1 ;
    volatile unsigned Bitl25 : 1 ;
    volatile unsigned Bitl26 : 1 ;
    volatile unsigned Bitl27 : 1 ;
    volatile unsigned Bitl28 : 1 ;
    volatile unsigned Bitl29 : 1 ;
    volatile unsigned Bitl30 : 1 ;
    volatile unsigned Bitl31 : 1 ;
} lBits32 ;
//Note the use of the address operator ..., somewhat simplifies the macro definitions...
#define B32_0(x)   (((lBits32 *)(&x))->Bitl0)
#define B32_1(x)   (((lBits32 *)(&x))->Bitl1)
#define B32_2(x)   (((lBits32 *)(&x))->Bitl2)
#define B32_3(x)   (((lBits32 *)(&x))->Bitl3)
#define B32_4(x)   (((lBits32 *)(&x))->Bitl4)
#define B32_5(x)   (((lBits32 *)(&x))->Bitl5)
#define B32_6(x)   (((lBits32 *)(&x))->Bitl6)
#define B32_7(x)   (((lBits32 *)(&x))->Bitl7)
#define B32_8(x)   (((lBits32 *)(&x))->Bitl8)
#define B32_9(x)   (((lBits32 *)(&x))->Bitl9)
#define B32_10(x)  (((lBits32 *)(&x))->Bitl10)
#define B32_11(x)  (((lBits32 *)(&x))->Bitl11)
#define B32_12(x)  (((lBits32 *)(&x))->Bitl12)
#define B32_13(x)  (((lBits32 *)(&x))->Bitl13)
#define B32_14(x)  (((lBits32 *)(&x))->Bitl14)
#define B32_15(x)  (((lBits32 *)(&x))->Bitl15)
#define B32_16(x)  (((lBits32 *)(&x))->Bitl16)
#define B32_17(x)  (((lBits32 *)(&x))->Bitl17)
#define B16_18(x)  (((lBits32 *)(&x))->Bitl18)
#define B32_19(x)  (((lBits32 *)(&x))->Bitl19)
#define B32_20(x)  (((lBits32 *)(&x))->Bitl20)
#define B32_21(x)  (((lBits32 *)(&x))->Bitl21)
#define B32_22(x)  (((lBits32 *)(&x))->Bitl22)
#define B32_23(x)  (((lBits32 *)(&x))->Bitl23)
#define B32_24(x)  (((lBits32 *)(&x))->Bitl24)
#define B32_25(x)  (((lBits32 *)(&x))->Bitl25)
#define B32_26(x)  (((lBits32 *)(&x))->Bitl26)
#define B32_27(x)  (((lBits32 *)(&x))->Bitl27)
#define B32_28(x)  (((lBits32 *)(&x))->Bitl28)
#define B32_29(x)  (((lBits32 *)(&x))->Bitl29)
#define B32_30(x)  (((lBits32 *)(&x))->Bitl30)
#define B32_31(x)  (((lBits32 *)(&x))->Bitl31)

const unsigned long BITMAP[] = {0x00000001,0x00000002,0x00000004,0x00000008,0x00000010,0x00000020,0x00000040,0x00000080,
                                0x00000100,0x00000200,0x00000400,0x00000800,0x00001000,0x00002000,0x00004000,0x00008000,
                                0x00010000,0x00020000,0x00040000,0x00080000,0x00100000,0x00200000,0x00400000,0x00800000,
                                0x01000000,0x02000000,0x04000000,0x08000000,0x10000000,0x20000000,0x40000000,0x80000000};



// Example : #define UpButton B16_3(MasterKeyBuffer)

// #define ValidCheckin B8_4(P6OUT_)
// #define InvalidCheckin B8_5(P6OUT_)
// #define ValidWord B8_6(P6OUT_)
// #define WatchdogPulse B8_7(P4OUT_)



#define HIGH 1
#define LOW 0

//    ValidCheckin = HIGH ;
//    if (ValidWord == 1) {
//        ValidCheckin = HIGH ;
//    }

#define Chunk(type,variable,offset) *(((type *)(&(variable))+ (offset) ))

/*
#set lbl 1
#macro smark(sm)
	/$
	mov SP,&@sm
	$/
#endm

#macro stest(sm)
	/$
    cmp &@sm,SP
    jhs L_##lbl
	mov SP,&@sm
L_##lbl: nop
	$/
#set lbl lbl+1
#endm
*/
#endif
