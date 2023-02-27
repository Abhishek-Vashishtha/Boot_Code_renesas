/***********************************************************************************************************************
* File Name    : variable.h
* Version      : CodeGenerator for RL78/L12 V2.00.00.07 [22 Feb 2013]
* Device(s)    : R5F10MMG
* Tool-Chain   : CC-RL
* Description  : This file implements device driver for CGC module.
* Creation Date: 7/2/2013
***********************************************************************************************************************/

#ifndef BILL_H
#define BILL_H

/***********************************************************************************************************************
Macro definitions (Register bit)
***********************************************************************************************************************/
/***********************************************************************************************************************
Macro definitions
***********************************************************************************************************************/
/***********************************************************************************************************************
Typedef definitions
***********************************************************************************************************************/

/***********************************************************************************************************************
Global functions
***********************************************************************************************************************/
void BILL_vInit(void);
void check_bill(void);
void calc_bill_poff(u8 );
void GetNextDate(char tpMin,char tpHr,char tpDate, char tpMonth, char tpYear, char tbMin, char tbHr, char tbDate, char tEvenOdd);
void md_function(unsigned char type);
void mdfun(void);
void missbill(void);
void pon_min_write(void);
unsigned int read_todadd(unsigned char bmonth1);
void save_billdata(void);
void save_cumtpr(void);
void save_todbill(void);
void storezkwh(void);
void TOU_vCheck_Active_Calendar(void);
void TOU_vDeter_Season(void);
void deter_zone(void);
void deter_week(void);
void zone_default(void);
void TOD_vloadzkwh(void);

/* Start user code for function. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
#endif