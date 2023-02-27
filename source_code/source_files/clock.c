/***********************************************************************************************************************
* File Name       : clock.c
* Current Version : rev_01  
* Tool-Chain      : IAR Systems
* Description     : 
* Creation Date   : 02-01-2020
* Company         : Genus Power Infrastructures Limited, Jaipur
* Author          : dheeraj.singhal
* Version History : rev_01 : initial fw release
***********************************************************************************************************************/
/************************************ Includes **************************************/
#include "clock.h"

/************************************ Local Variables *****************************************/
/************************************ Extern Variables *****************************************/
us8 clock_select,MulFactor; 
flag_union flag_clock;
/************************************ Local Functions *******************************/
/************************************ Extern Functions ******************************/
void clock_init();
void clock_change(us8 clock);



void clock_init()
{
    R_CGC_Create();
    clock_select = CLOCK_24MHZ;
    MulFactor = 1;
}

void clock_change(us8 clock)
{ 
  if(clock == CLOCK_24MHZ)
  {
    clock_select = CLOCK_24MHZ;
    HOCODIV = 0x00;
    MulFactor = 1;
  }
  else if(clock == CLOCK_12MHZ)
  {
    clock_select = CLOCK_12MHZ;
    HOCODIV = 0x01;
    MulFactor = 2;
  }
  else if(clock == CLOCK_6MHZ)
  {
    clock_select = CLOCK_6MHZ;
    HOCODIV = 0x02;
    MulFactor = 4;
  }
  else if(clock == CLOCK_1_5MHZ)
  {
    clock_select = CLOCK_1_5MHZ;
    HOCODIV = 0x04;
    MulFactor = 16;
  }
  else                          /* default 24MHZ */
  {
    clock_select = CLOCK_1_5MHZ;
    HOCODIV = 0x00;
    MulFactor = 1;
  }
  
  /* Set fCLK */
  CSS = 0U;       /* main system clock (fMAIN) */
  
  /* Set fMAIN */
  MCM0 = 0U;      /* selects the main on-chip oscillator clock (fOCO) or PLL clock (fPLL) as the main system clock (fMAIN) */
  
  /* Set fMAIN Control */
  MCM1 = 0U;      /* high-speed on-chip oscillator clock */
  
}
