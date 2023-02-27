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
* File Name    : r_cg_lvd.c
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
* Function Name: R_LVD_Create
* Description  : This function initializes the voltage detector.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_LVD_Create(void)
{
    LVIMK = 1U;     /* disable INTLVI interrupt */
    LVIIF = 0U;     /* clear INTLVI interrupt flag */
    LVDVDMK = 1U;   /* disable INTLVDVDD interrupt */
    LVDVDIF = 0U;   /* clear INTLVDVDD interrupt flag */
    LVDVBMK = 1U;   /* disable INTLVDVBAT interrupt */
    LVDVBIF = 0U;   /* clear INTLVDVBAT interrupt flag */
    LVDVRMK = 1U;   /* disable INTLVDVRTC interrupt */
    LVDVRIF = 0U;   /* clear INTLVDVRTC interrupt flag */
    LVDEXMK = 1U;   /* disable INTLVDEXLVD interrupt */
    LVDEXIF = 0U;   /* clear INTLVDEXLVD interrupt flag */
//    /* Set INTLVDVDD level 1 priority */
//    LVDVDPR1 = 0U;
//    LVDVDPR0 = 1U;
//    /* Set INTLVDVRTC low priority */
//    LVDVRPR1 = 1U;
//    LVDVRPR0 = 1U;
    /* Set INTLVDEXLVD level 1 priority */
    LVDEXPR1 = 0U;
    LVDEXPR0 = 1U;

    R_LVD_Create_UserInit();
}

/***********************************************************************************************************************
* Function Name: R_LVD_InterruptMode_Start
* Description  : This function enables the voltage detector interrupt.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_LVD_InterruptMode_Start(void)
{
    LVIIF = 0U;     /* clear INTLVI interrupt flag */
    LVIMK = 0U;     /* enable INTLVI interrupt */
}

void R_LVD_InterruptMode_Stop(void)
{
    LVIMK = 1U;     /* enable INTLVI interrupt */
    LVIIF = 0U;     /* clear INTLVI interrupt flag */
}
/***********************************************************************************************************************
* Function Name: R_LVD_Start_VDD
* Description  : This function enables the voltage detector VDD.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_LVD_Start_VDD(void)
{
    LVDVDIF = 0U;   /* clear INTLVDVDD interrupt flag */
    LVDVDMK = 0U;   /* enable INTLVDVDD interrupt */
    LVDVDD = _80_LVD_VDD_DELECT_ENABLE | _02_LVD_VDD_VOLTAGE_288;
}

/***********************************************************************************************************************
* Function Name: R_LVD_Stop_VDD
* Description  : This function disables the voltage detector VDD.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_LVD_Stop_VDD(void)
{
    LVDVDMK = 1U;   /* disable INTLVDVDD interrupt */
    LVDVDIF = 0U;   /* clear INTLVDVDD interrupt flag */
    LVDVDDEN = 0U;  /* disables detection */
}

/***********************************************************************************************************************
* Function Name: R_LVD_Start_VBAT
* Description  : This function enables the voltage detector VBAT.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_LVD_Start_VBAT(void)
{
    LVDVBIF = 0U;   /* clear INTLVDVBAT interrupt flag */
    LVDVBMK = 0U;   /* enable INTLVDVBAT interrupt */
    LVDVBAT = _00_LVD_VBAT_DELECT_DISABLE;
}

/***********************************************************************************************************************
* Function Name: R_LVD_Stop_VBAT
* Description  : This function disables the voltage detector VBAT.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_LVD_Stop_VBAT(void)
{
    LVDVBMK = 1U;   /* disable INTLVDVBAT interrupt */
    LVDVBIF = 0U;   /* clear INTLVDVBAT interrupt flag */
    LVDVBATEN = 0U; /* disables detection */
}

/***********************************************************************************************************************
* Function Name: R_LVD_Start_VRTC
* Description  : This function enables the voltage detector VRTC.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_LVD_Start_VRTC(void)
{
    LVDVRIF = 0U;   /* clear INTLVDVRTC interrupt flag */
    LVDVRMK = 0U;   /* enable INTLVDVRTC interrupt */
    LVDVRTC = _80_LVD_VRTC_DELECT_ENABLE | _00_LVD_VRTC_VOLTAGE_216;
}

/***********************************************************************************************************************
* Function Name: R_LVD_Stop_VRTC
* Description  : This function disables the voltage detector VRTC.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_LVD_Stop_VRTC(void)
{
    LVDVRMK = 1U;   /* disable INTLVDVRTC interrupt */
    LVDVRIF = 0U;   /* clear INTLVDVRTC interrupt flag */
    LVDVRTCEN = 0U; /* disables detection */
}

/***********************************************************************************************************************
* Function Name: R_LVD_Start_EXLVD
* Description  : This function enables the voltage detector EXLVD.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_LVD_Start_EXLVD(void)
{
    LVDEXIF = 0U;   /* clear INTLVDEXLVD interrupt flag */
    LVDEXMK = 0U;   /* enable INTLVDEXLVD interrupt */
    LVDEXLVD = _80_LVD_EXLVD_DELECT_ENABLE;

    /* Set EXLVD pin */
    //PM2 |= 0x04U;
}

/***********************************************************************************************************************
* Function Name: R_LVD_Stop_EXLVD
* Description  : This function disables the voltage detector EXLVD.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_LVD_Stop_EXLVD(void)
{
    LVDEXMK = 1U;   /* disable INTLVDEXLVD interrupt */
    LVDEXIF = 0U;   /* clear INTLVDEXLVD interrupt flag */
    LVDEXLVDEN = 0U;/* disables detection */

    /* Set EXLVD pin */
    //PM2 &= 0xFBU;
}

/* Start user code for adding. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
