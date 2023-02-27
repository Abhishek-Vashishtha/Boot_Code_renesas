/***********************************************************************************************************************
* File Name       : wdt.h
* Current Version : rev_01  
* Tool-Chain      : IAR Systems
* Description     : this is a header file of wdt.c
* Creation Date   : 30-12-2019
* Company         : Genus Power Infrastructures Limited, Jaipur
* Author          : dheeraj.singhal
***********************************************************************************************************************/
#pragma once
/************************************ Includes **************************************/
#include "system.h"
#include "r_cg_wdt.h"

/************************************ Macro *****************************************/
/* Configurations */
#define WDT_TIME_474MS                                          (1)
#define WDT_TIME_949MS                                          (2)
#define WDT_TIME_3799MS                                         (3)
#define WDT_TIME                                                WDT_TIME_3799MS

#define SOFT_RESET                                              {WDTE = 0xAAU;}
/* Thresholds */

/* Functions */
#define wdt_init()                                              {R_WDT_Create();}
#define wdt_restart()                                           {R_WDT_Restart();}
/************************************ Extern Variables ******************************/
/************************************ Extern Functions ******************************/
