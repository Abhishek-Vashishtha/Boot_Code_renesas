/***********************************************************************************************************************
* File Name    : variable.h
* Version      : CodeGenerator for RL78/L12 V2.00.00.07 [22 Feb 2013]
* Device(s)    : R5F10MMG
* Tool-Chain   : CC-RL
* Description  : This file implements device driver for CGC module.
* Creation Date: 7/2/2013
***********************************************************************************************************************/

#ifndef FUN_H
#define FUN_H

/***********************************************************************************************************************
Macro definitions (Register bit)
***********************************************************************************************************************/
/***********************************************************************************************************************
Macro definitions
***********************************************************************************************************************/
#define isleapyear(year) ((!(year % 4) && (year % 100)) || (!(year % 400)))

/***********************************************************************************************************************
Typedef definitions
***********************************************************************************************************************/

/***********************************************************************************************************************
Global functions
***********************************************************************************************************************/
void fill_oprzero(void);
void FUN_vfill_2byte(unsigned  int ,unsigned char * );
void FUN_vfill_2byteR(unsigned  int ,unsigned char * );
void FUN_vfill_3byte(unsigned  long int ,unsigned char * );
void FUN_vfill_3byteR(unsigned long int ,unsigned char * );
void FUN_vfill_4byte(unsigned  long int ,unsigned char * );
void FUN_vfill_4byteR(unsigned  long int ,unsigned char * );
unsigned long int a8_to_u24(unsigned char *ptr1);
unsigned long int a8_to_u32(unsigned char *ptr1);
unsigned int a8_to_u16(unsigned char *ptr1);
void bin_2_dec(void);
unsigned char calmdpg1(unsigned char mdmth);
unsigned char calmdpg(unsigned char mdmth);
void read_add(unsigned int blk,unsigned char bmonth1);
unsigned char bcd_to_hex(unsigned char temp);
unsigned char hex_to_bcd(unsigned char temp);
void time_stamp(unsigned char *addr);
void fill_ff(unsigned char count1);
int isdatevalid(int month, int day, int year);
unsigned int sel_datediff(unsigned char,unsigned char ,unsigned char );
unsigned char chksumsr(unsigned char *chkdata,unsigned char nob);
void memzero(u8 *s1, u16 n);
void reset_meter_data_new(void);
void ClearEprom(unsigned int from, unsigned int count);
void reset_tamper(void);
void time_diff(void);
void code_gen(void);
void pon_reset(void);
unsigned long int sqrt_function(unsigned char* ptr);
void delay_1p3s(void);
u32 Compute_EnergyHighres(u8 choice);
u32 CalCumMD(u8 MdType);
#endif