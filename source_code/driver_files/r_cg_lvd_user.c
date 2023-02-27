/***********************************************************************************************************************
* DISCLAIMER
* This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products.
* No other uses are authorized. This software is owned by Renesas Electronics Corporation and is protected under all
* applicable laws, including copyright laws. 
* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING THIS SOFTWARE, WHETHER EXPRESS, IMPLIED
* OR STATUTORY, INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NON-INFRINGEMENT.  ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED.TO THE MAXIMUM EXTENT PERMITTED NOT PROHIBITED BY
* LAW, NEITHER RENESAS ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES SHALL BE LIABLE FOR ANY DIRECT,
* INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON RELATED TO THIS SOFTWARE, EVEN IF RENESAS OR
* ITS AFFILIATES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* Renesas reserves the right, without notice, to make changes to this software and to discontinue the availability 
* of this software. By using this software, you agree to the additional terms and conditions found by accessing the 
* following link:
* http://www.renesas.com/disclaimer
*
* Copyright (C) 2015, 2019 Renesas Electronics Corporation. All rights reserved.
***********************************************************************************************************************/

/***********************************************************************************************************************
* File Name    : r_cg_lvd_user.c
* Version      : Applilet4 for RL78/I1C V1.01.04.02 [20 Nov 2019]
* Device(s)    : R5F10NPJ
* Tool-Chain   : IAR Systems icc78k0r
* Description  : This file implements device driver for LVD module.
* Creation Date: 06/26/2020
***********************************************************************************************************************/

/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "r_cg_macrodriver.h"
#include "r_cg_lvd.h"
/* Start user code for include. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
#include "pcb.h"
/***********************************************************************************************************************
Pragma directive
***********************************************************************************************************************/
/* Start user code for pragma. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
/* Start user code for global. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
* Function Name: R_LVD_Create_UserInit
* Description  : This function adds user code after initializing the voltage detector.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_LVD_Create_UserInit(void)
{
    /* Start user code. Do not edit comment generated here */
    /* End user code. Do not edit comment generated here */
}
/***********************************************************************************************************************
* Function Name: r_lvd_interrupt
* Description  : None
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
#pragma vector = INTLVI_vect
__interrupt static void r_lvd_interrupt(void)
{
    /* Start user code. Do not edit comment generated here */
    last_interrupt = 18;
    flag_lvd_interrupt = 1;
    if(flag_lvd_instant == 1)
    {
        flag_lvd_status = 1;
    }
    else
    {
        flag_lvd_status = 0;
    }
    /* End user code. Do not edit comment generated here */
}

/***********************************************************************************************************************
* Function Name: r_lvd_vddinterrupt
* Description  : None
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
#pragma vector = INTLVDVDD_vect
__interrupt static void r_lvd_vddinterrupt(void)
{
    /* Start user code. Do not edit comment generated here */
    last_interrupt = 19;
    flag_lvd_vdd_interrupt = 1;
    if(flag_lvd_vdd_instant == 1)
    {
        flag_lvd_vdd_status = 1;
    }
    else
    {
        flag_lvd_vdd_status = 0;
    }
/* End user code. Do not edit comment generated here */
}

/***********************************************************************************************************************
* Function Name: r_lvd_vbatinterrupt
* Description  : None
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
#pragma vector = INTLVDVBAT_vect
__interrupt static void r_lvd_vbatinterrupt(void)
{
    /* Start user code. Do not edit comment generated here */
    last_interrupt = 20;
    flag_lvd_vbat_interrupt = 1;
    if(flag_lvd_vbat_instant == 1)
    {
        flag_lvd_vbat_status = 1;
    }
    else
    {
        flag_lvd_vbat_status = 0;
    }
    /* End user code. Do not edit comment generated here */
}

/***********************************************************************************************************************
* Function Name: r_lvd_vrtcinterrupt
* Description  : None
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
#pragma vector = INTLVDVRTC_vect
__interrupt static void r_lvd_vrtcinterrupt(void)
{
    /* Start user code. Do not edit comment generated here */
    last_interrupt = 21;
    flag_lvd_rtc_interrupt = 1;
    if(flag_lvd_rtc_instant == 1)
    {
        flag_lvd_rtc_status = 1;
    }
    else
    {
        flag_lvd_rtc_status = 0;
    }
    /* End user code. Do not edit comment generated here */
}

/***********************************************************************************************************************
* Function Name: r_lvd_exlvdinterrupt
* Description  : None
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
#pragma vector = INTLVDEXLVD_vect
__interrupt static void r_lvd_exlvdinterrupt(void)
{
    /* Start user code. Do not edit comment generated here */
    last_interrupt = 22;
    flag_lvd_exlvd_interrupt = 1;
    backlight_operation();

    /* End user code. Do not edit comment generated here */
}

/* Start user code for adding. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
