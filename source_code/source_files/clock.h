/***********************************************************************************************************************
* File Name       : clock.h
* Current Version : rev_01  
* Tool-Chain      : IAR Systems
* Description     : this is a header file of clock.c
* Creation Date   : 02-01-2020
* Company         : Genus Power Infrastructures Limited, Jaipur
* Author          : dheeraj.singhal
***********************************************************************************************************************/
#pragma once
/************************************ Includes **************************************/
#include "system.h"
#include "r_cg_cgc.h"

/************************************ Macro *****************************************/
/* Configurations */
#define CLOCK_24MHZ                             (1)
#define CLOCK_12MHZ                             (2)
#define CLOCK_6MHZ                              (3)
#define CLOCK_1_5MHZ                            (4)

/* Thresholds */

/************************************ Extern Variables *****************************************/
extern us8 clock_select,MulFactor; 
extern flag_union flag_clock;
#define flag_clock_startup                                      flag_clock.bit.b0
#define flag_clock_1                                            flag_clock.bit.b1
#define flag_clock_2                                            flag_clock.bit.b2
#define flag_clock_3                                            flag_clock.bit.b3
#define flag_clock_4                                            flag_clock.bit.b4
#define flag_clock_5                                            flag_clock.bit.b5
#define flag_clock_6                                            flag_clock.bit.b6
#define flag_clock_7                                            flag_clock.bit.b7

/************************************ Extern Functions ******************************/
extern void clock_init();
extern void clock_change(us8 clock);