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
* File Name    : r_cg_cgc.c
* Version      : Applilet4 for RL78/I1C V1.01.02.04 [24 May 2018]
* Device(s)    : R5F10NPJ
* Tool-Chain   : IAR Systems icc78k0r
* Description  : This file implements device driver for CGC module.
* Creation Date: 12/14/2019
***********************************************************************************************************************/

/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "r_cg_macrodriver.h"
#include "r_cg_cgc.h"
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
* Function Name: R_CGC_Create
* Description  : This function initializes the clock generator.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_CGC_Create(void)
{
  volatile uint32_t w_count;
  
  
  /******************************************************************************************************************
  Please Refer Data Sheet RL78/I1c Rev 1.00 May 2016
  * There are Five Clock Source option available Three are On chip, Two are External  
  * For CPU Clock supply We are Using On chip Clock oscillator with a Frequency of 24 Mhz
  * For RTC and Other Modules We are Using Subsystem Clock Oscillator Externally Supplied frequncy 32.768 Khz
  * It Should be Kept in notice that DSADC is supplied 12 Mhz clock with division by 2 of On chip clock regardless 
  of clock supplied to CPU i.e 24Mhz 
  ******************************************************************************************************************/
  
  /* Set fIM : middle-speed on-chip oscillator stopped, not using*/
  MIOEN = 0U;     
  
  /* Change the waiting time according to the system */
  for (w_count = 0U; w_count <= (CGC_FIMWAITTIME); w_count++)
  {
    NOP();
  }
  
  /* Set fPLL: On chip Clock Division through PLL not used */
  PCKC = _00_CGC_fIH_STOP;
  DSCCTL = _00_CGC_PLL_STOP;
  
  /* Change the waiting time according to the system */
  for (w_count = 0U; w_count <= (CGC_FPLLWAITTIME); w_count++)
  {
    NOP();
  }
  
  /* FOCO clock i.e Clock selected between fih (onchip clock) & Fim (Middle speed) will be used */
  MCKC = _00_CGC_fOCO_SELECTED;
  
  /* Set fMX : The external crystal clock is diabled */
  CMC = _00_CGC_HISYS_PORT | _00_CGC_SYSOSC_UNDER10M;
  MSTOP = 1U;     /* X1 oscillator/external clock stopped */
  
  /* Set fSUB */
  SELLOSC = 0U;   /* sub clock (fSX) */
  
  /* Set fSX */
  OSMC = _00_CGC_CLK_ENABLE | _00_CGC_IT_CLK_fSX_CLK;
  
  VRTCEN = 1U;    /* enables input clock supply */
  
  delay_ms(10);
  
  SCMC = _10_CGC_SUB_OSC | _02_CGC_NORMAL_OSCILLATION;
  
  XTSTOP = 0U;    /* XT1 oscillator operating */
  
  CMC &= (uint8_t)~_10_CGC_SYSOSC_PERMITTED;
  
  XT1SELDIS = 0U; /* Enables clock supply to CPU for On chip frequency correction */
  
  VRTCEN = 0U;    /* stops input clock supply */
}

/***********************************************************************************************************************
* Function Name: R_CGC_Set_ClockMode
* Description  : This function changes clock generator operation mode.
* Arguments    : mode -
*                    clock generator operation mode
* Return Value : status -
*                    MD_OK, MD_ARGERROR, MD_ERROR1, MD_ERROR2, MD_ERROR3, MD_ERROR4, MD_ERROR5, MD_ERROR6 or MD_ERROR7
***********************************************************************************************************************/
MD_STATUS R_CGC_Set_ClockMode(clock_mode_t mode)
{
  MD_STATUS         status = MD_OK;
  clock_mode_t      old_mode;
  uint8_t           temp_stab_set;
  uint8_t           temp_stab_wait;
  volatile uint32_t w_count;
  
  if (1U == CLS)             
  {
    if (1U == SELLOSC)
    {
      old_mode = fILCLK;
    }
    else
    {
      if ((SCMC & _30_CGC_SUB_PIN) == _10_CGC_SUB_OSC)
      {
        old_mode = SUBXT1CLK;
      }
      else if ((SCMC & _30_CGC_SUB_PIN) == _30_CGC_SUB_EXT) 
      {
        old_mode = SUBEXTCLK;
      }
      else
      {
        /* Not run */
      }
    }
  }
  else
  {
    if (1U == MCS)
    {
      if ((CMC & _C0_CGC_HISYS_PIN) == _C0_CGC_HISYS_EXT) 
      {
        old_mode = SYSEXTCLK;
      }
      else if ((CMC & _C0_CGC_HISYS_PIN) == _40_CGC_HISYS_OSC)
      {
        old_mode = SYSX1CLK;
      }
      else
      {
        /* Not run */
      }
    }
    else
    {
      if (_80_CGC_fPLL_STATE == (MCKC & _80_CGC_fPLL_STATE))
      {
        old_mode = PLLCLK;
      }
      else
      {
        if (1U == MCS1)
        {
          old_mode = MIOCLK;
        }
        else
        {
          old_mode = HIOCLK;
        }
      }
    }
  }
  
  if (mode != old_mode)
  {
    switch (mode)
    {
    case HIOCLK:
      
      if (old_mode == PLLCLK)
      {
        MCKC &= (uint8_t)~_01_CGC_fPLL_SELECTED;
        
        /* Change the waiting time according to the system */	
        for (w_count = 0U; w_count <= CGC_CKSTRWAITTIME; w_count++)	
        {	
          NOP();	
        }	
        
        DSCCTL &= (uint8_t)~_01_CGC_PLL_OUTPUT;
        PCKC &= (uint8_t)~_02_CGC_fIH_ENABLE;
      }
      else
      {         
        if (1U == HIOSTOP)
        {
          HIOSTOP = 0U;   /* high-speed on-chip oscillator operating */
          
          /* Change the waiting time according to the system */
          for (w_count = 0U; w_count <= CGC_FIHWAITTIME; w_count++)
          {
            NOP();
          }
        }
      }
      
      CSS = 0U;       /* main system clock (fMAIN) */
      MCM0 = 0U;      /* selects the main on-chip oscillator clock (fOCO) or PLL clock (fPLL) as the main system clock (fMAIN) */
      MCM1 = 0U;      /* high-speed on-chip oscillator clock */
      
      break;
      
    case MIOCLK:
      
      if (PLLCLK == old_mode)
      {
        status = MD_ERROR1;
      }
      else
      {
        if (0U == MIOEN)
        {
          MIOEN = 0U;     /* middle-speed on-chip oscillator stopped */
          
          
          /* Change the waiting time according to the system */
          for (w_count = 0U; w_count <= CGC_FIMWAITTIME; w_count++)
          {
            NOP();
          }
        }
        
        CSS = 0U;       /* main system clock (fMAIN) */
        MCM0 = 0U;      /* selects the main on-chip oscillator clock (fOCO) or PLL clock (fPLL) as the main system clock (fMAIN) */
        MCM1 = 1U;      /* middle-speed on-chip oscillator clock */
      }   
      
      break;
      
    case PLLCLK:
      
      if (HIOCLK != old_mode)
      {
        status = MD_ERROR2;
      }
      else
      {
        DSCCTL = _0C_CGC_CLOCK_fPLL | _02_CGC_MULTIPLICATION_fPLL;
        PCKC = _02_CGC_fIH_ENABLE;
        DSCCTL |= _01_CGC_PLL_OUTPUT;
        
        /* Change the waiting time according to the system */
        for (w_count = 0U; w_count <= CGC_FPLLWAITTIME; w_count++)
        {
          NOP();
        }
        
        MCKC |= _01_CGC_fPLL_SELECTED;
        
        while ((MCKC & _80_CGC_fPLL_STATE) != _80_CGC_fPLL_STATE)
        {
          ;
        }
      }
      
      break;
      
    case SYSX1CLK:
      
      if ((SYSEXTCLK == old_mode) || (PLLCLK == old_mode) || ((CMC & _C0_CGC_HISYS_PIN) != _40_CGC_HISYS_OSC)) 
      {
        status = MD_ERROR3;
      }
      else
      {
        if (1U == MSTOP)
        {
          MSTOP = 0U;     /* X1 oscillator/external clock operating */
          temp_stab_set = (uint8_t)~(0x7FU >> OSTS);
          
          do
          {
            temp_stab_wait = OSTC;
            temp_stab_wait &= temp_stab_set;
          }
          while (temp_stab_wait != temp_stab_set);
        }
        
        CSS = 0U;       /* main system clock (fMAIN) */
        MCM0 = 1U;      /* selects the high-speed system clock (fMX) as the main system clock (fMAIN) */
      }
      
      break;
      
    case SYSEXTCLK:
      
      if ((SYSX1CLK == old_mode) || (PLLCLK == old_mode) || ((CMC & _C0_CGC_HISYS_PIN) != _C0_CGC_HISYS_EXT)) 
      {
        status = MD_ERROR4;
      }
      else
      {
        if (1U == MSTOP)
        {
          MSTOP = 0U;     /* X1 oscillator/external clock operating */
        }
        
        CSS = 0U;       /* main system clock (fMAIN) */
        MCM0 = 1U;      /* selects the high-speed system clock (fMX) as the main system clock (fMAIN) */
      }
      
      break;
      
    case SUBXT1CLK:
      
      if ((SUBEXTCLK == old_mode) || (fILCLK == old_mode) || (PLLCLK == old_mode) || ((SCMC & _30_CGC_SUB_PIN) != _10_CGC_SUB_OSC))
      {
        status = MD_ERROR5;
      }
      else
      {
        VRTCEN = 1U;    /* enables input clock supply */
        
        if (1U == XTSTOP)
        {
          XTSTOP = 0U;    /* XT1 oscillator operating */
          CMC |= _10_CGC_SYSOSC_PERMITTED;
          
          /* Change the waiting time according to the system */
          for (w_count = 0U; w_count <= CGC_SUBWAITTIME; w_count++)
          {
            NOP();
          }
          
          XT1SELDIS = 0U; /* enables clock supply */
        }
        
        SELLOSC = 0U;   /* sub clock (fSX) */
        CSS = 1U;       /* subsystem clock (fSUB) */
        VRTCEN = 0U;    /* stops input clock supply */
      }
      
      break;
      
    case SUBEXTCLK:
      
      if ((SUBXT1CLK == old_mode) || (fILCLK == old_mode) || (PLLCLK == old_mode) || ((SCMC & _30_CGC_SUB_PIN) != _30_CGC_SUB_EXT))
      {
        status = MD_ERROR6;
      }
      else
      {
        VRTCEN = 1U;    /* enables input clock supply */
        CMC |= _10_CGC_SYSOSC_PERMITTED;
        
        
        /* Change the waiting time according to the system */
        for (w_count = 0U; w_count <= CGC_SUBWAITTIME; w_count++)
        {
          NOP();
        }
        
        XT1SELDIS = 0U; /* enables clock supply */
        
        SELLOSC = 0U;   /* sub clock (fSX) */
        CSS = 1U;       /* subsystem clock (fSUB) */
        VRTCEN = 0U;    /* stops input clock supply */
      }
      
      break;
      
    case fILCLK:
      
      if ((SUBXT1CLK == old_mode) || (SUBEXTCLK == old_mode) || (PLLCLK == old_mode))
      {
        status = MD_ERROR7;
      }
      else
      {
        SELLOSC = 1U;   /* low-speed on-chip oscillator clock (fIL) */
        
        /* Change the waiting time according to the system */
        for (w_count = 0U; w_count <= CGC_FILWAITTIME; w_count++)
        {
          NOP();
        }
        
        CSS = 1U;       /* subsystem clock (fSUB) */
        
      }
      
      break;
      
    default:
      
      status = MD_ARGERROR;
      
      break;
    }
  }
  
  return (status);
}
/***********************************************************************************************************************
* Function Name: R_CGC_Set_LPMode
* Description  : This function This function changes Flash Operation mode from LS(low - speed main) mode to LP(low - power main) mode.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_CGC_Set_LPMode(void)
{
  /* Transition to LP mode */
  MCSEL = 0;                  /* Normal setting */
  FLMWEN = 1;                 /* Rewriting the FLMODE register is enabled */
  FLMODE = (FLMODE ^ 0xC0);   /* LP (low-power main) mode */
  FLMWEN = 0;                 /* Rewriting the FLMODE register is disabled */
  MCSEL = 1;                  /* Low-power consumption setting */
}
/***********************************************************************************************************************
* Function Name: R_CGC_Set_LSMode
* Description  : This function This function changes Flash Operation mode from LP(low - power main) mode to LS(low - speed main) mode.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_CGC_Set_LSMode(void)
{
  /* Transition to LS mode */
  MCSEL = 0;                  /* Normal setting */
  FLMWEN = 1;                 /* Rewriting the FLMODE register is enabled */
  FLMODE = (FLMODE ^ 0xC0);   /* LS (low-speed main) mode */
  FLMWEN = 0;                 /* Rewriting the FLMODE register is disabled */
  MCSEL = 1;                  /* Low-power consumption setting */
}

/* Start user code for adding. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
