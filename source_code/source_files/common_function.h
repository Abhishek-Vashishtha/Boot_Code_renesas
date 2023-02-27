/***********************************************************************************************************************
* File Name       : common_function.h
* Current Version : rev_  
* Tool-Chain      : IAR Systems
* Description     : this is a header file of common_fuinction.c
* Creation Date   : 30-12-2019
* Company         : Genus Power Infrastructures Limited, Jaipur
* Author          : dheeraj.singhal
***********************************************************************************************************************/
#pragma once
/************************************ Includes **************************************/
#include "variables.h"
#include "system.h"
#include "intrinsics.h"
#include "string.h"
#include <stdio.h>

/************************************ Macro *****************************************/
#define SIGNED_MODE                                             (1u)
#define UNSIGNED_MODE                                           (2u)
/* Common Macros */
#define TRUE                                                    (1)
#define FALSE                                                   (0)
#define HIGH                                                    (1)
#define LOW                                                     (0)
#define high                                                    (1)
#define low                                                     (0)
#define lowByte(w)              			        ((us8) ((w) & 0xff))
#define highByte(w)             			        ((us8) ((w) >> 8))
#define bit(b)                          	                (0x0001U << (b))
#define bitSet(value, bit_hex)              	                ((value) |= (bit_hex))
#define bitClear(value, bit_hex)            	                ((value) &= ~(bit_hex))
#define bitToggle(value,bit_hex)            	                ((value) ^= (bit_hex))
#define bitIsSet(value, bit_hex)                                (((value) & (bit_hex)) == (bit_hex))
#define bitIsClear(value, bit_hex)                              (((value) & (bit_hex)) == 0)
#define inc(val,max_val)                                        ((val) == ((max_val)-1) ? (0) : ((val)+1))
#define dec(val,max_val)                                        ((val) == 0 ? ((max_val)-1) : ((val)-1))
#define SET_BIT(reg, bit_no, bit_value)                         (reg##_bit.no##bit_no = bit_value)
#define BIT_SELECT(reg, bit_no)                                 (reg##_bit.no##bit_no)
#define SHIFT_LEFT(value,shift)                                 ((value)<<(shift))
#define SHIFT_RIGHT(value,shift)                                ((value)>>(shift))


/* Bit manipulation */
#define bit0                                                    (0x01)
#define bit1                                                    (0x02)
#define bit2                                                    (0x04)
#define bit3                                                    (0x08)
#define bit4                                                    (0x10)
#define bit5                                                    (0x20)
#define bit6                                                    (0x40)
#define bit7                                                    (0x80)
#define bit8                                                    (0x0100)
#define bit9                                                    (0x0200)
#define bit10           				        (0x0400)
#define bit11           				        (0x0800)
#define bit12           				        (0x1000)
#define bit13           				        (0x2000)
#define bit14           				        (0x4000)
#define bit15           				        (0x8000)
#define bit16                                                   (0x00010000)
#define bit17                                                   (0x00020000)
#define bit18                                                   (0x00040000)
#define bit19                                                   (0x00080000)
#define bit20                                                   (0x00100000)
#define bit21                                                   (0x00200000)
#define bit22                                                   (0x00400000)
#define bit23                                                   (0x00800000)
#define bit24                                                   (0x01000000)
#define bit25                                                   (0x02000000)
#define bit26           				        (0x04000000)
#define bit27           				        (0x08000000)
#define bit28           				        (0x10000000)
#define bit29           				        (0x20000000)
#define bit30           				        (0x40000000)
#define bit31           				        (0x80000000)
#define BIT0                                                    (bit0)
#define BIT1                                                    (bit1)
#define BIT2                                                    (bit2)
#define BIT3                                                    (bit3)
#define BIT4                                                    (bit4)
#define BIT5                                                    (bit5)
#define BIT6                                                    (bit6)
#define BIT7                                                    (bit7)
#define BIT8                                                    (bit8)
#define BIT9                                                    (bit9)
#define BIT10           				        (bit10)
#define BIT11           				        (bit11)
#define BIT12           				        (bit12)
#define BIT13           				        (bit13)
#define BIT14           				        (bit14)
#define BIT15           				        (bit15)
#define BIT16                                                   (bit16)
#define BIT17                                                   (bit17)
#define BIT18                                                   (bit18)
#define BIT19                                                   (bit19)
#define BIT20                                                   (bit20)
#define BIT21                                                   (bit21)
#define BIT22                                                   (bit22)
#define BIT23                                                   (bit23)
#define BIT24                                                   (bit24)
#define BIT25                                                   (bit25)
#define BIT26           				        (bit26)
#define BIT27           				        (bit27)
#define BIT28           				        (bit28)
#define BIT29           				        (bit29)
#define BIT30           				        (bit30)
#define BIT31           				        (bit31)

extern const us8 dec2hexAscii[16];

extern us8 reverse_8bits(us8 data);
extern void delay_ms(us16 count);
extern void delay_us(us16 us);
