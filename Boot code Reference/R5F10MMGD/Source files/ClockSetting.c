/***********************************************************************************************************************
* File Name    : r_cg_cgc.c
* Version      : CodeGenerator for RL78/I1B V2.00.00.04 [21 Feb 2013]
* Device(s)    : R5F10MMG
* Tool-Chain   : CC-RL
* Description  : This file implements device driver for CGC module.
* Creation Date: 7/17/2013
***********************************************************************************************************************/


/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "r_cg_macrodriver.h"
#include "ClockSetting.h"
#include "variable.h"


/***********************************************************************************************************************
Pragma directive
***********************************************************************************************************************/
#pragma interrupt X1_correction_Interrupt(vect = INTCR)

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/

/***********************************************************************************************************************
* Function Name: CLK_vClockSet
* Description  : This function initializes the clock generator.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void CLK_vClockSet(void)
{
    volatile uint16_t w_count;
#if EXT_X1 == 1
   uint8_t           temp_stab_set;
   uint8_t           temp_stab_wait; 
#endif

    /* Set fMX */
   CMC = _40_CGC_HISYS_OSC | _10_CGC_SUB_OSC | _01_CGC_SYSOSC_OVER10M | _04_CGC_SUBMODE_ULOW;//_00_CGC_SUBMODE_LOW;//_02_CGC_SUBMODE_NORMAL;
        XTSTOP = 0U;
    /* Change the waiting time according to the system */
    for (w_count = 0U; w_count <= CGC_SUBWAITTIME; w_count++)
    {
        __nop();
    }    
    
    OSMC = _00_CGC_SUBINHALT_ON | _00_CGC_RTC_CLK_FSUB;
#if EXT_X1 == 1
    MSTOP = 0U;
    temp_stab_set = _FF_CGC_OSCSTAB_STA18;  
    do
    {
        temp_stab_wait = OSTC;
        temp_stab_wait &= temp_stab_set;
    }
    while (temp_stab_wait != temp_stab_set);
    /* Set fCLK */
    CSS = 0U;//Main system clock (fMAIN)
    MCM0 = 1U;//Selects the high-speed system clock (fMX) as the main system clock (fMAIN)
    /* Stop fIH */
    HIOSTOP = 1U;
#else
	HIOSTOP = 0U;
	HOCODIV = X1_FREQ;//0x03;//0 = 24,1=12,2=6,3=3
	/* Set fCLK */
	CSS = 0U;//Main system clock (fMAIN)
	MCM0 = 0U;//Selects the high-speed system clock (fMX) as the main system clock (fMAIN)
	/* Stop fMX */
	MSTOP = 1U;
	CRIF = 0;
	HOCOFC  = 0x40;//enable High-speed on-chip oscillator clock frequency correction circuit starts operating/frequency correction is operating
	HOCOFC  = 0x41;
	CRMK = 0;
//	while(CRIF==0);
	CRIF = 0;
//	HOCOFC = 0x00;
	
#endif    
    /* Set fSUB */
//    XTSTOP = 0U;
//    /* Change the waiting time according to the system */
//    for (w_count = 0U; w_count <= CGC_SUBWAITTIME; w_count++)
//    {
//        __nop();
//    }    
//    
//    OSMC = _00_CGC_SUBINHALT_ON | _00_CGC_RTC_CLK_FSUB;
}
static void __near X1_correction_Interrupt(void)
{
//	CRIF = 0;
	//CRMK = 1;
	HOCOFC = 0x01;
//	CLK_u8Correction_done = 1;
	
}
/* Start user code for adding. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
