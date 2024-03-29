/***********************************************************************************************************************
* File Name    : variable.h
* Version      : CodeGenerator for RL78/L12 V2.00.00.07 [22 Feb 2013]
* Device(s)    : R5F10MMG
* Tool-Chain   : CC-RL
* Description  : This file implements device driver for CGC module.
* Creation Date: 7/2/2013
***********************************************************************************************************************/

#ifndef LS_H
#define LS_H

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
void load_ls_cnt(void);
void LS_vInit(void);
void fill_darray(void);
void read_darray(void);
void load_survey(void);
void ls_miss_fill(void);
void save_loadsurvey_cnt(void);
void ls_timestamp(unsigned char ls_ip1);
void LS_vValueReset(void);
void next_miss_date(unsigned char, unsigned char, unsigned char);
/* Start user code for function. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
#endif