/***********************************************************************************************************************
* File Name       : system.h
* Current Version : rev_01  
* Tool-Chain      : IAR Systems
* Description     : this is a header file of system.c
* Creation Date   : 30-12-2019
* Company         : Genus Power Infrastructures Limited, Jaipur
* Author          : dheeraj.singhal
***********************************************************************************************************************/
#pragma once
/************************************ Includes **************************************/
#include "variables.h"
#include "ior5f10npj.h"
#include "ior5f10npj_ext.h"
#include "r_cg_lvd.h"
#include "r_cg_sau.h"
/************************************ Macro *****************************************/
/* Configurations */
/* Thresholds */
/************************************ Extern Variables *****************************************/


extern us8 last_interrupt;

extern flag_union flag1;
#define battery_mode_f                                  flag1.bit.b0
#define soft_reset_f                                    flag1.bit.b1

/************************************ Extern Functions ******************************/
extern void clear_ram();
extern void system_init();
extern void lvd_start();
extern us8 read_operating_mode();
extern void backlight_operation();
