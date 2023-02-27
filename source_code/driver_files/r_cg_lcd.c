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
* Copyright (C) 2015, 2018 Renesas Electronics Corporation. All rights reserved.
***********************************************************************************************************************/

/***********************************************************************************************************************
* File Name    : r_cg_lcd.c
* Version      : Applilet4 for RL78/I1C V1.01.03.02 [16 Nov 2018]
* Device(s)    : R5F10NPJ
* Tool-Chain   : IAR Systems icc78k0r
* Description  : This file implements device driver for LCD module.
* Creation Date: 14-05-2020
***********************************************************************************************************************/

/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "r_cg_macrodriver.h"
#include "r_cg_lcd.h"
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
* Function Name: R_LCD_Create
* Description  : This function initializes the LCD module.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_LCD_Create(void)
{
    volatile uint32_t wt_count;

    LCDON = 0U;     /* display off (all segment outputs are deselected) */
    LCDM1 |= _00_LCD_VOLTAGE_HIGH; /* pending, check for the use of this with BLA sir*/
    LCDM0 = _00_LCD_DISPLAY_WAVEFORM_A | _14_LCD_TIMESLICE_8 | _01_LCD_BIAS_MODE_3;
    LCDM0 |= _40_LCD_VOLTAGE_MODE_INTERNAL;
    /* Set CAPL and CAPH pins */
    ISCLCD &= (uint8_t)~_01_LCD_CAPLH_BUFFER_VALID;
    P12 &= 0x3FU;
    PM12 |= 0xC0U;
    /* Set VL3 pin */
    ISCLCD &= (uint8_t)~_02_LCD_VL3_BUFFER_VALID;
    P12 &= 0xDFU;
    PM12 |= 0x20U;
    /* Set segment pins */
    PIM1 &= 0x9FU;
    PIM8 &= 0xFCU;
    POM1 &= 0x1FU;
    POM8 &= 0xF8U;    
    PFSEG0 |= 0xF0U;
    PFSEG1 |= 0xFFU;
    PFSEG2 |= 0xFFU;
    PFSEG3 |= 0x3FU;
    PU1 &= 0x00U;
    PU3 &= 0xC0U;
    PU7 &= 0x00U;
    PU8 &= 0xF0U;
    P1 &= 0x00U;
    PM1 &= 0x00U;
    P3 &= 0xC0U;
    PM3 &= 0xC0U;    
    P7 &= 0x00U;
    PM7 &= 0x00U;
    P8 &= 0xF0U;
    PM8 &= 0xF0U;

    LCDC0 = _05_LCD_CLOCK_FSX_FIL_6;
    VLCD = _06_LCD_BOOST_VOLTAGE_110V;    
    /* Change the waiting time according to the system */
    delay_ms(5);
}
/***********************************************************************************************************************
* Function Name: R_LCD_Start
* Description  : This function enables the LCD display.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_LCD_Start(void)
{
    LCDON = 1U;     /* display on */
}
/***********************************************************************************************************************
* Function Name: R_LCD_Stop
* Description  : This function disables the LCD display.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_LCD_Stop(void)
{
    LCDON = 0U;     /* display off (all segment outputs are deselected) */
}
/***********************************************************************************************************************
* Function Name: R_LCD_Voltage_On
* Description  : This function enables voltage boost circuit or capacitor split circuit.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_LCD_Voltage_On(void)
{
    VLCON = 1U;     /* enables voltage boost and capacitor split operation */

    wdt_restart();
    delay_ms(500);
    wdt_restart();
    
    SCOC = 1U;      /* select common and segment pins output */
}
/***********************************************************************************************************************
* Function Name: R_LCD_Voltage_Off
* Description  : This function disables voltage boost circuit or capacitor split circuit.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_LCD_Voltage_Off(void)
{
    SCOC = 0U;      /* output ground level to segment/common pin */
    VLCON = 0U;     /* stops voltage boost and capacitor split operation */
    LCDM0 &= (uint8_t)~(_C0_LCD_VOLTAGE_MODE_INITIALVALUE);
}

/* Start user code for adding. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
