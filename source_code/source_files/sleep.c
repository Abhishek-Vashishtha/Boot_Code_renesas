/***********************************************************************************************************************
* File Name       : sleep.c
* Current Version : rev_01  
* Tool-Chain      : IAR Systems RL78
* Description     : This file contains the functions needed for sleep mode operation of the meter
* Creation Date   : 02-07-2020
* Company         : Genus Power Infrastructures Limited, Jaipur
* Author          : dheeraj.singhal
* Version History : rev_01 :
***********************************************************************************************************************/
/************************************ Includes **************************************/
#include "sleep.h"
/************************************ Local Variables *****************************************/
/************************************ Extern Variables *****************************************/

flag_union flag_sleep,flag_lvd1,flag_lvd2,flag_wakeup_method;
/************************************ Local Functions *******************************/
void sleep_clear_var();
void sleep_mode_exit();
void sleep_action_early_detection();
void sleep_saving_early_detection();
/************************************ Extern Functions ******************************/
void sleep_clear_var()
{
    
}
void sleep_mode_exit()
{
  
}
void sleep_action_early_detection()
{
  backlight_operation();
}
void sleep_saving_early_detection()
{

}
