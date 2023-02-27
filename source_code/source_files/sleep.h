/***********************************************************************************************************************
* File Name       : sleep.h
* Current Version : rev_01  
* Tool-Chain      : IAR Systems
* Description     : this is a header file of sleep.c
* Creation Date   : 02-07-2020
* Company         : Genus Power Infrastructures Limited, Jaipur
* Author          : dheeraj.singhal
***********************************************************************************************************************/
#pragma once
/************************************ Includes **************************************/
#include "variables.h"
/************************************ Macro *****************************************/
/* Configurations */

/* Thresholds */
/************************************ Extern Variables *****************************************/

extern flag_union flag_wakeup_method;
#define wakeup_scrl_dn_f                                        flag_wakeup_method.bit.b0
#define wakeup_top_cover_f                                      flag_wakeup_method.bit.b1
#define wakeup_power_back_f                                     flag_wakeup_method.bit.b2
#define flag_wakeup_method_3                                    flag_wakeup_method.bit.b3
#define flag_wakeup_method_4                                    flag_wakeup_method.bit.b4
#define flag_wakeup_method_5                                    flag_wakeup_method.bit.b5
#define flag_wakeup_method_6                                    flag_wakeup_method.bit.b6
#define flag_wakeup_method_7                                    flag_wakeup_method.bit.b7

extern flag_union flag_lvd1;
#define flag_lvd_interrupt                                     flag_lvd1.bit.b0
#define flag_lvd_vdd_interrupt                                 flag_lvd1.bit.b1
#define flag_lvd_vbat_interrupt                                flag_lvd1.bit.b2
#define flag_lvd_rtc_interrupt                                 flag_lvd1.bit.b3
#define flag_lvd_exlvd_interrupt                               flag_lvd1.bit.b4
#define flag_lvd1_5                                            flag_lvd1.bit.b5
#define flag_lvd1_6                                            flag_lvd1.bit.b6
#define flag_lvd1_7                                            flag_lvd1.bit.b7

extern flag_union flag_lvd2;
#define flag_lvd_status                                        flag_lvd2.bit.b0
#define flag_lvd_vdd_status                                    flag_lvd2.bit.b1
#define flag_lvd_vbat_status                                   flag_lvd2.bit.b2
#define flag_lvd_rtc_status                                    flag_lvd2.bit.b3
#define flag_lvd2_4                                            flag_lvd2.bit.b4
#define flag_lvd2_5                                            flag_lvd2.bit.b5
#define flag_lvd2_6                                            flag_lvd2.bit.b6
#define flag_lvd2_7                                            flag_lvd2.bit.b7

extern flag_union flag_sleep;
#define flag_sleep_zc_fail                                      flag_sleep.bit.b0
#define flag_sleep_execute                                      flag_sleep.bit.b1
#define flag_sleep_pwr_dwn_detected                             flag_sleep.bit.b2
#define flag_mem_fail_pwr_up                                    flag_sleep.bit.b3
#define flag_invalid_wakeup_int                                 flag_sleep.bit.b4
#define flag_sleep_5                                            flag_sleep.bit.b5
#define flag_sleep_6                                            flag_sleep.bit.b6
#define flag_sleep_7                                            flag_sleep.bit.b7

#define flag_lvd_instant                                      (LVIF)
#define flag_lvd_vdd_instant                                  (LVDVDDF)
#define flag_lvd_vbat_instant                                 (LVDVBATF)
#define flag_lvd_rtc_instant                                  (LVDVRTCF)
#define flag_lvd_exlvd_instant                                (LVDEXLVDF)



/************************************ Extern Functions ******************************/
extern void sleep_clear_var();
extern void sleep_mode_exit();
extern void sleep_action_early_detection();
extern void sleep_saving_early_detection();
