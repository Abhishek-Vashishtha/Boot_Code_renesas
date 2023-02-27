///////////////////////////////////////////////////////////////////////////////
//
// IAR C/C++ Compiler V4.20.1.2260 for RL78               20/Dec/2020  22:37:59
// Copyright 2011-2019 IAR Systems AB.
// Network license: 192.10.10.205 (STD)
//
//    Core               =  s3
//    Calling convention =  v2
//    Code model         =  Near
//    Data model         =  Near
//                       =   
//    Source file        =
//        D:\Dheeraj\New folder\0. GDEV72 -
//        BootCode\source_code\source_files\app_load_survey.c
//    Command line       =
//        -f C:\Users\laptop\AppData\Local\Temp\EWDF4.tmp ("D:\Dheeraj\New
//        folder\0. GDEV72 -
//        BootCode\source_code\source_files\app_load_survey.c" --core s3
//        --code_model near --calling_convention v2 --near_const_location ram
//        -o "D:\Dheeraj\New folder\0. GDEV72 - BootCode\Debug\Obj"
//        --dlib_config "C:\Program Files\IAR Systems\Embedded Workbench
//        8.4\rl78\LIB\DLib_Config_Normal.h" --double=32 -e -On --no_cse
//        --no_unroll --no_inline --no_code_motion --no_tbaa --no_cross_call
//        --no_scheduling --no_clustering --debug -lA "D:\Dheeraj\New folder\0.
//        GDEV72 - BootCode\Debug\List" -I "D:\Dheeraj\New folder\0. GDEV72 -
//        BootCode\source_code\driver_files\" -I "D:\Dheeraj\New folder\0.
//        GDEV72 - BootCode\source_code\library_files\" -I "D:\Dheeraj\New
//        folder\0. GDEV72 - BootCode\source_code\misc_files\" -I
//        "D:\Dheeraj\New folder\0. GDEV72 -
//        BootCode\source_code\source_files\" --data_model near)
//    Locale             =  C
//    List file          =
//        D:\Dheeraj\New folder\0. GDEV72 -
//        BootCode\Debug\List\app_load_survey.s
//
///////////////////////////////////////////////////////////////////////////////

        RTMODEL "__SystemLibrary", "DLib"
        RTMODEL "__calling_convention", "v2"
        RTMODEL "__code_model", "near"
        RTMODEL "__core", "s3"
        RTMODEL "__data_model", "near"
        RTMODEL "__dlib_file_descriptor", "0"
        RTMODEL "__dlib_version", "6"
        RTMODEL "__double_size", "32"
        RTMODEL "__far_rt_calls", "false"
        RTMODEL "__rt_version", "2"

        #define SHT_PROGBITS 0x1

        EXTERN _opr_data
        EXTERN _TempTime
        EXTERN _Now
        EXTERN _OffTime
        EXTERN _temp_us8
        EXTERN _temp4_var
        EXTERN _temp5_var
        EXTERN _monthdays
        EXTERN _COMPART_TRANSACTION_ENTRIES
        EXTERN _temp3_var
        EXTERN _temp2_var
        EXTERN _temp1_var
        EXTERN _hard_reset_f
        EXTERN _ls_max_obj
        EXTERN _ls_conf_obj
        EXTERN ?MOVE_LONG_L06
        EXTERN ?SI_CMP_L02
        EXTERN ?SI_DIV_L02
        EXTERN ?UC_DIV_L01
        EXTERN ?UC_MOD_L01
        EXTERN ?UL_CMP_L03
        EXTERN _COMPART_TRANSACTION_START_ADD
        EXTERN _Eprom_ReadWM
        EXTERN _Eprom_WriteWM
        EXTERN _R_WDT_Restart
        EXTERN _bcd_to_decimal
        EXTERN _char_array_into_time5_sec
        EXTERN _char_array_to_int
        EXTERN _char_array_to_long3
        EXTERN _char_array_to_long4
        EXTERN _curr
        EXTERN _decimal_to_bcd
        EXTERN _energy
        EXTERN _eprom_read
        EXTERN _eprom_write
        EXTERN _fill_oprzero
        EXTERN _flag_rtc_change
        EXTERN _freq
        EXTERN _int_into_char_array
        EXTERN _long_int
        EXTERN _long_into_char_array3
        EXTERN _long_into_char_array4
        EXTERN _pf
        EXTERN _power_on_min
        EXTERN _sel_datediff
        EXTERN _seq_no_transaction
        EXTERN _temp_us16
        EXTERN _temp_us32
        EXTERN _time_inc_hour
        EXTERN _time_into_char_array5
        EXTERN _vol

        PUBLIC _d_array
        PUBLIC _day_counter
        PUBLIC _load_ls_cnt
        PUBLIC _load_survey_avg_acc
        PUBLIC _load_survey_avg_cal
        PUBLIC _load_survey_cnt
        PUBLIC _load_survey_fill_last_energy
        PUBLIC _load_survey_next_interval_timestamp
        PUBLIC _load_survey_ram_init
        PUBLIC _load_survey_reset_avg_acc
        PUBLIC _load_survey_save_avg_acc
        PUBLIC _load_survey_snap_shot
        PUBLIC _ls_act_exp_demand
        PUBLIC _ls_act_imp_demand
        PUBLIC _ls_app_exp_demand
        PUBLIC _ls_app_imp_demand
        PUBLIC _ls_avg_acc_curr_b
        PUBLIC _ls_avg_acc_curr_n
        PUBLIC _ls_avg_acc_curr_r
        PUBLIC _ls_avg_acc_curr_y
        PUBLIC _ls_avg_acc_freq_net
        PUBLIC _ls_avg_acc_pf_b
        PUBLIC _ls_avg_acc_pf_net
        PUBLIC _ls_avg_acc_pf_r
        PUBLIC _ls_avg_acc_pf_y
        PUBLIC _ls_avg_acc_vol_b
        PUBLIC _ls_avg_acc_vol_r
        PUBLIC _ls_avg_acc_vol_y
        PUBLIC _ls_cal_energy_diff
        PUBLIC _ls_cnt_at_pwrup
        PUBLIC _ls_cnt_end_1day
        PUBLIC _ls_cnt_start_1day
        PUBLIC _ls_date
        PUBLIC _ls_day_counter
        PUBLIC _ls_fg_f
        PUBLIC _ls_miss_fill
        PUBLIC _ls_month
        PUBLIC _ls_pom
        PUBLIC _ls_react_q1_demand
        PUBLIC _ls_react_q2_demand
        PUBLIC _ls_react_q3_demand
        PUBLIC _ls_react_q4_demand
        PUBLIC _ls_rev_fill
        PUBLIC _ls_rollover
        PUBLIC _ls_rtc_fill
        PUBLIC _ls_status
        PUBLIC _ls_year
        PUBLIC _lsip_period
        PUBLIC _lsro_flag
        PUBLIC _max_day_counter
        PUBLIC _max_load_survey_cnt
        PUBLIC _mdi_sel_ls
        PUBLIC _mdi_sel_ls_new
        PUBLIC _midnight_fill_rtc_f
        PUBLIC _midnight_par_cnt
        PUBLIC _midnight_par_miss_f
        PUBLIC _midnight_roll_f
        PUBLIC _midnight_var_init
        PUBLIC _miss_ls_date_f
        PUBLIC _next_miss_date
        PUBLIC _no_midnight_f
        PUBLIC _reset_load_survey
        PUBLIC _save_midnight_par
        PUBLIC _seq_no_ls
        PUBLIC _status_reg
        
          CFI Names cfiNames0
          CFI StackFrame CFA SP NEARDATA
          CFI Resource A:8, X:8, B:8, C:8, D:8, E:8, H:8, L:8, CS_REG:4, ES_REG:4
          CFI VirtualResource ?RET:20
          CFI Resource MACRH:16, MACRL:16, W0:8, W1:8, W2:8, W3:8, W4:8, W5:8
          CFI Resource W6:8, W7:8, W8:8, W9:8, W10:8, W11:8, W12:8, W13:8, W14:8
          CFI Resource W15:8, W16:8, W17:8, W18:8, W19:8, W20:8, W21:8, W22:8
          CFI Resource W23:8, W24:8, W25:8, W26:8, W27:8, W28:8, W29:8, W30:8
          CFI Resource W31:8, W32:8, W33:8, W34:8, W35:8, W36:8, W37:8, W38:8
          CFI Resource W39:8, W40:8, W41:8, W42:8, W43:8, W44:8, W45:8, W46:8
          CFI Resource W47:8, W48:8, W49:8, W50:8, W51:8, W52:8, W53:8, W54:8
          CFI Resource W55:8, W56:8, W57:8, W58:8, W59:8, W60:8, W61:8, W62:8
          CFI Resource W63:8, W64:8, W65:8, W66:8, W67:8, W68:8, W69:8, W70:8
          CFI Resource W71:8, W72:8, W73:8, W74:8, W75:8, W76:8, W77:8, W78:8
          CFI Resource W79:8, W80:8, W81:8, W82:8, W83:8, W84:8, W85:8, W86:8
          CFI Resource W87:8, W88:8, W89:8, W90:8, W91:8, W92:8, W93:8, W94:8
          CFI Resource W95:8, W96:8, W97:8, W98:8, W99:8, W100:8, W101:8, W102:8
          CFI Resource W103:8, W104:8, W105:8, W106:8, W107:8, W108:8, W109:8
          CFI Resource W110:8, W111:8, W112:8, W113:8, W114:8, W115:8, W116:8
          CFI Resource W117:8, W118:8, W119:8, W120:8, W121:8, W122:8, W123:8
          CFI Resource W124:8, W125:8, W126:8, W127:8, SP:16, ?SPH:4
          CFI EndNames cfiNames0
        
          CFI Common cfiCommon0 Using cfiNames0
          CFI CodeAlign 1
          CFI DataAlign 1
          CFI ReturnAddress ?RET CODE
          CFI CFA SP+4
          CFI A Undefined
          CFI X Undefined
          CFI B SameValue
          CFI C SameValue
          CFI D SameValue
          CFI E SameValue
          CFI H Undefined
          CFI L Undefined
          CFI CS_REG Undefined
          CFI ES_REG Undefined
          CFI ?RET Frame(CFA, -4)
          CFI MACRH Undefined
          CFI MACRL Undefined
          CFI W0 SameValue
          CFI W1 SameValue
          CFI W2 SameValue
          CFI W3 SameValue
          CFI W4 SameValue
          CFI W5 SameValue
          CFI W6 SameValue
          CFI W7 SameValue
          CFI W8 SameValue
          CFI W9 SameValue
          CFI W10 SameValue
          CFI W11 SameValue
          CFI W12 SameValue
          CFI W13 SameValue
          CFI W14 SameValue
          CFI W15 SameValue
          CFI W16 SameValue
          CFI W17 SameValue
          CFI W18 SameValue
          CFI W19 SameValue
          CFI W20 SameValue
          CFI W21 SameValue
          CFI W22 SameValue
          CFI W23 SameValue
          CFI W24 SameValue
          CFI W25 SameValue
          CFI W26 SameValue
          CFI W27 SameValue
          CFI W28 SameValue
          CFI W29 SameValue
          CFI W30 SameValue
          CFI W31 SameValue
          CFI W32 SameValue
          CFI W33 SameValue
          CFI W34 SameValue
          CFI W35 SameValue
          CFI W36 SameValue
          CFI W37 SameValue
          CFI W38 SameValue
          CFI W39 SameValue
          CFI W40 SameValue
          CFI W41 SameValue
          CFI W42 SameValue
          CFI W43 SameValue
          CFI W44 SameValue
          CFI W45 SameValue
          CFI W46 SameValue
          CFI W47 SameValue
          CFI W48 SameValue
          CFI W49 SameValue
          CFI W50 SameValue
          CFI W51 SameValue
          CFI W52 SameValue
          CFI W53 SameValue
          CFI W54 SameValue
          CFI W55 SameValue
          CFI W56 SameValue
          CFI W57 SameValue
          CFI W58 SameValue
          CFI W59 SameValue
          CFI W60 SameValue
          CFI W61 SameValue
          CFI W62 SameValue
          CFI W63 SameValue
          CFI W64 SameValue
          CFI W65 SameValue
          CFI W66 SameValue
          CFI W67 SameValue
          CFI W68 SameValue
          CFI W69 SameValue
          CFI W70 SameValue
          CFI W71 SameValue
          CFI W72 SameValue
          CFI W73 SameValue
          CFI W74 SameValue
          CFI W75 SameValue
          CFI W76 SameValue
          CFI W77 SameValue
          CFI W78 SameValue
          CFI W79 SameValue
          CFI W80 SameValue
          CFI W81 SameValue
          CFI W82 SameValue
          CFI W83 SameValue
          CFI W84 SameValue
          CFI W85 SameValue
          CFI W86 SameValue
          CFI W87 SameValue
          CFI W88 SameValue
          CFI W89 SameValue
          CFI W90 SameValue
          CFI W91 SameValue
          CFI W92 SameValue
          CFI W93 SameValue
          CFI W94 SameValue
          CFI W95 SameValue
          CFI W96 SameValue
          CFI W97 SameValue
          CFI W98 SameValue
          CFI W99 SameValue
          CFI W100 SameValue
          CFI W101 SameValue
          CFI W102 SameValue
          CFI W103 SameValue
          CFI W104 SameValue
          CFI W105 SameValue
          CFI W106 SameValue
          CFI W107 SameValue
          CFI W108 SameValue
          CFI W109 SameValue
          CFI W110 SameValue
          CFI W111 SameValue
          CFI W112 SameValue
          CFI W113 SameValue
          CFI W114 SameValue
          CFI W115 SameValue
          CFI W116 SameValue
          CFI W117 SameValue
          CFI W118 SameValue
          CFI W119 SameValue
          CFI W120 SameValue
          CFI W121 SameValue
          CFI W122 SameValue
          CFI W123 SameValue
          CFI W124 SameValue
          CFI W125 SameValue
          CFI W126 SameValue
          CFI W127 SameValue
          CFI ?SPH Undefined
          CFI EndCommon cfiCommon0
        
        
          CFI Common cfiCommon1 Using cfiNames0
          CFI CodeAlign 1
          CFI DataAlign 1
          CFI ReturnAddress ?RET CODE
          CFI CFA SP+4
          CFI A Undefined
          CFI X Undefined
          CFI B Undefined
          CFI C Undefined
          CFI D SameValue
          CFI E SameValue
          CFI H Undefined
          CFI L Undefined
          CFI CS_REG Undefined
          CFI ES_REG Undefined
          CFI ?RET Frame(CFA, -4)
          CFI MACRH Undefined
          CFI MACRL Undefined
          CFI W0 SameValue
          CFI W1 SameValue
          CFI W2 SameValue
          CFI W3 SameValue
          CFI W4 SameValue
          CFI W5 SameValue
          CFI W6 SameValue
          CFI W7 SameValue
          CFI W8 SameValue
          CFI W9 SameValue
          CFI W10 SameValue
          CFI W11 SameValue
          CFI W12 SameValue
          CFI W13 SameValue
          CFI W14 SameValue
          CFI W15 SameValue
          CFI W16 SameValue
          CFI W17 SameValue
          CFI W18 SameValue
          CFI W19 SameValue
          CFI W20 SameValue
          CFI W21 SameValue
          CFI W22 SameValue
          CFI W23 SameValue
          CFI W24 SameValue
          CFI W25 SameValue
          CFI W26 SameValue
          CFI W27 SameValue
          CFI W28 SameValue
          CFI W29 SameValue
          CFI W30 SameValue
          CFI W31 SameValue
          CFI W32 SameValue
          CFI W33 SameValue
          CFI W34 SameValue
          CFI W35 SameValue
          CFI W36 SameValue
          CFI W37 SameValue
          CFI W38 SameValue
          CFI W39 SameValue
          CFI W40 SameValue
          CFI W41 SameValue
          CFI W42 SameValue
          CFI W43 SameValue
          CFI W44 SameValue
          CFI W45 SameValue
          CFI W46 SameValue
          CFI W47 SameValue
          CFI W48 SameValue
          CFI W49 SameValue
          CFI W50 SameValue
          CFI W51 SameValue
          CFI W52 SameValue
          CFI W53 SameValue
          CFI W54 SameValue
          CFI W55 SameValue
          CFI W56 SameValue
          CFI W57 SameValue
          CFI W58 SameValue
          CFI W59 SameValue
          CFI W60 SameValue
          CFI W61 SameValue
          CFI W62 SameValue
          CFI W63 SameValue
          CFI W64 SameValue
          CFI W65 SameValue
          CFI W66 SameValue
          CFI W67 SameValue
          CFI W68 SameValue
          CFI W69 SameValue
          CFI W70 SameValue
          CFI W71 SameValue
          CFI W72 SameValue
          CFI W73 SameValue
          CFI W74 SameValue
          CFI W75 SameValue
          CFI W76 SameValue
          CFI W77 SameValue
          CFI W78 SameValue
          CFI W79 SameValue
          CFI W80 SameValue
          CFI W81 SameValue
          CFI W82 SameValue
          CFI W83 SameValue
          CFI W84 SameValue
          CFI W85 SameValue
          CFI W86 SameValue
          CFI W87 SameValue
          CFI W88 SameValue
          CFI W89 SameValue
          CFI W90 SameValue
          CFI W91 SameValue
          CFI W92 SameValue
          CFI W93 SameValue
          CFI W94 SameValue
          CFI W95 SameValue
          CFI W96 SameValue
          CFI W97 SameValue
          CFI W98 SameValue
          CFI W99 SameValue
          CFI W100 SameValue
          CFI W101 SameValue
          CFI W102 SameValue
          CFI W103 SameValue
          CFI W104 SameValue
          CFI W105 SameValue
          CFI W106 SameValue
          CFI W107 SameValue
          CFI W108 SameValue
          CFI W109 SameValue
          CFI W110 SameValue
          CFI W111 SameValue
          CFI W112 SameValue
          CFI W113 SameValue
          CFI W114 SameValue
          CFI W115 SameValue
          CFI W116 SameValue
          CFI W117 SameValue
          CFI W118 SameValue
          CFI W119 SameValue
          CFI W120 SameValue
          CFI W121 SameValue
          CFI W122 SameValue
          CFI W123 SameValue
          CFI W124 SameValue
          CFI W125 SameValue
          CFI W126 SameValue
          CFI W127 SameValue
          CFI ?SPH Undefined
          CFI EndCommon cfiCommon1
        
// D:\Dheeraj\New folder\0. GDEV72 - BootCode\source_code\source_files\app_load_survey.c
//    1 /***********************************************************************************************************************
//    2 * File Name       : app_load_survey.c
//    3 * Current Version : rev_01  
//    4 * Tool-Chain      : IAR Systems RL78
//    5 * Description     : this file will include all the code related to load survey and daily energy functionality of the meter
//    6 * Creation Date   : 5/31/2020
//    7 * Company         : Genus Power Infrastructures Limited, Jaipur
//    8 * Author          : dheeraj.singhal
//    9 * Version History : rev_01 :
//   10 ***********************************************************************************************************************/
//   11 /************************************ Includes **************************************/
//   12 #include "app_load_survey.h"
//   13 /************************************ Local Variables *****************************************/
//   14 /************************************ Extern Variables *****************************************/

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   15 us16 max_load_survey_cnt,load_survey_cnt,day_counter;
_max_load_survey_cnt:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_load_survey_cnt:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_day_counter:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   16 us8 max_day_counter;
_max_day_counter:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   17 us8 lsip_period,mdi_sel_ls,mdi_sel_ls_new,miss_ls_date_f,lsro_flag,ls_rtc_fill,midnight_fill_rtc_f;
_lsip_period:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_mdi_sel_ls:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_mdi_sel_ls_new:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_miss_ls_date_f:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_lsro_flag:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_ls_rtc_fill:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_midnight_fill_rtc_f:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   18 us8 status_reg[2];
_status_reg:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   19 us32 seq_no_ls;
_seq_no_ls:
        DS 4

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   20 us32 ls_act_imp_demand,ls_act_exp_demand,ls_app_imp_demand,ls_app_exp_demand,ls_react_q1_demand,ls_react_q2_demand,ls_react_q3_demand,ls_react_q4_demand;
_ls_act_imp_demand:
        DS 4

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_ls_act_exp_demand:
        DS 4

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_ls_app_imp_demand:
        DS 4

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_ls_app_exp_demand:
        DS 4

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_ls_react_q1_demand:
        DS 4

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_ls_react_q2_demand:
        DS 4

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_ls_react_q3_demand:
        DS 4

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_ls_react_q4_demand:
        DS 4

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   21 us8 ls_status;
_ls_status:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   22 us16 d_array[D_ARRAY_SIZE];
_d_array:
        DS 364
//   23 

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   24 us16 ls_cnt_end_1day,ls_cnt_start_1day;
_ls_cnt_end_1day:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_ls_cnt_start_1day:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   25 us8 ls_rev_fill,ls_fg_f;
_ls_rev_fill:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_ls_fg_f:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   26 us8 ls_date,ls_month,ls_year;
_ls_date:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_ls_month:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_ls_year:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   27 us8 midnight_par_cnt,midnight_roll_f,midnight_par_miss_f,no_midnight_f;
_midnight_par_cnt:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_midnight_roll_f:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_midnight_par_miss_f:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_no_midnight_f:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   28 us16 ls_avg_acc_vol_r,ls_avg_acc_vol_y,ls_avg_acc_vol_b;
_ls_avg_acc_vol_r:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_ls_avg_acc_vol_y:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_ls_avg_acc_vol_b:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   29 us32 ls_avg_acc_curr_r,ls_avg_acc_curr_y,ls_avg_acc_curr_b,ls_avg_acc_curr_n;
_ls_avg_acc_curr_r:
        DS 4

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_ls_avg_acc_curr_y:
        DS 4

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_ls_avg_acc_curr_b:
        DS 4

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_ls_avg_acc_curr_n:
        DS 4

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   30 s16 ls_avg_acc_pf_r,ls_avg_acc_pf_y,ls_avg_acc_pf_b,ls_avg_acc_pf_net;
_ls_avg_acc_pf_r:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_ls_avg_acc_pf_y:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_ls_avg_acc_pf_b:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_ls_avg_acc_pf_net:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   31 us8 ls_pom;
_ls_pom:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   32 us32 ls_avg_acc_freq_net;
_ls_avg_acc_freq_net:
        DS 4
//   33 
//   34 /************************************ Local Functions *******************************/
//   35 void ls_rollover();
//   36 void ls_day_counter();
//   37 void load_ls_cnt();
//   38 us32 ls_cal_energy_diff(us32 energy, us32 last_energy);
//   39 rtc_counter_value_t load_survey_next_interval_timestamp(us8 mdi_period1, rtc_counter_value_t Time);
//   40 void next_miss_date(uint8 date, uint8 month, uint8 year);
//   41 void midnight_var_init(void );
//   42 void save_midnight_par(void);
//   43 /************************************ Extern Functions ******************************/
//   44 void load_survey_snap_shot();
//   45 void load_survey_avg_acc();
//   46 void load_survey_avg_cal();
//   47 void load_survey_ram_init();
//   48 void reset_load_survey();
//   49 void load_survey_save_avg_acc();
//   50 void load_survey_reset_avg_acc();
//   51 void load_survey_fill_last_energy();
//   52 
//   53 
//   54 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock0 Using cfiCommon0
          CFI Function _load_survey_fill_last_energy
        CODE
//   55 void load_survey_fill_last_energy()
//   56 {
_load_survey_fill_last_energy:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//   57   fill_oprzero(48);
        MOV       A, #0x30           ;; 1 cycle
          CFI FunCall _fill_oprzero
        CALL      _fill_oprzero      ;; 3 cycles
//   58   long_into_char_array4(energy.Allph.active_imp,&opr_data[0]);
        MOVW      DE, #LWRD(_opr_data)  ;; 1 cycle
        MOVW      BC, N:_energy+54   ;; 1 cycle
        MOVW      AX, N:_energy+52   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//   59   long_into_char_array4(energy.Allph.active_exp,&opr_data[4]);
        MOVW      DE, #LWRD(_opr_data+4)  ;; 1 cycle
        MOVW      BC, N:_energy+58   ;; 1 cycle
        MOVW      AX, N:_energy+56   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//   60   long_into_char_array4(energy.Allph.reactive_q1,&opr_data[8]);
        MOVW      DE, #LWRD(_opr_data+8)  ;; 1 cycle
        MOVW      BC, N:_energy+78   ;; 1 cycle
        MOVW      AX, N:_energy+76   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//   61   long_into_char_array4(energy.Allph.reactive_q2,&opr_data[12]);
        MOVW      DE, #LWRD(_opr_data+12)  ;; 1 cycle
        MOVW      BC, N:_energy+82   ;; 1 cycle
        MOVW      AX, N:_energy+80   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//   62   long_into_char_array4(energy.Allph.reactive_q3,&opr_data[16]);
        MOVW      DE, #LWRD(_opr_data+16)  ;; 1 cycle
        MOVW      BC, N:_energy+86   ;; 1 cycle
        MOVW      AX, N:_energy+84   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//   63   long_into_char_array4(energy.Allph.reactive_q4,&opr_data[20]);
        MOVW      DE, #LWRD(_opr_data+20)  ;; 1 cycle
        MOVW      BC, N:_energy+90   ;; 1 cycle
        MOVW      AX, N:_energy+88   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//   64   long_into_char_array4(energy.Allph.apparent_imp,&opr_data[24]);
        MOVW      DE, #LWRD(_opr_data+24)  ;; 1 cycle
        MOVW      BC, N:_energy+70   ;; 1 cycle
        MOVW      AX, N:_energy+68   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//   65   long_into_char_array4(energy.Allph.apparent_exp,&opr_data[28]);
        MOVW      DE, #LWRD(_opr_data+28)  ;; 1 cycle
        MOVW      BC, N:_energy+74   ;; 1 cycle
        MOVW      AX, N:_energy+72   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//   66   eprom_write(0x0700,0,48,PAGE_3,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+6
        MOV       B, #0x2            ;; 1 cycle
        MOVW      DE, #0x30          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x700         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//   67 }
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock0
        ; ------------------------------------- Block: 68 cycles
        ; ------------------------------------- Total: 68 cycles

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock1 Using cfiCommon0
          CFI Function _load_survey_reset_avg_acc
        CODE
//   68 void load_survey_reset_avg_acc()
//   69 {
_load_survey_reset_avg_acc:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//   70   ls_avg_acc_vol_r = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_ls_avg_acc_vol_r, AX  ;; 1 cycle
//   71   ls_avg_acc_vol_y = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_ls_avg_acc_vol_y, AX  ;; 1 cycle
//   72   ls_avg_acc_vol_b = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_ls_avg_acc_vol_b, AX  ;; 1 cycle
//   73   ls_avg_acc_curr_r = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_ls_avg_acc_curr_r, AX  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_ls_avg_acc_curr_r+2, AX  ;; 1 cycle
//   74   ls_avg_acc_curr_y = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_ls_avg_acc_curr_y, AX  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_ls_avg_acc_curr_y+2, AX  ;; 1 cycle
//   75   ls_avg_acc_curr_b = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_ls_avg_acc_curr_b, AX  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_ls_avg_acc_curr_b+2, AX  ;; 1 cycle
//   76   ls_avg_acc_curr_n = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_ls_avg_acc_curr_n, AX  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_ls_avg_acc_curr_n+2, AX  ;; 1 cycle
//   77   ls_avg_acc_pf_r = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_ls_avg_acc_pf_r, AX  ;; 1 cycle
//   78   ls_avg_acc_pf_y = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_ls_avg_acc_pf_y, AX  ;; 1 cycle
//   79   ls_avg_acc_pf_b = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_ls_avg_acc_pf_b, AX  ;; 1 cycle
//   80   ls_avg_acc_pf_net = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_ls_avg_acc_pf_net, AX  ;; 1 cycle
//   81   ls_pom = 0;
        MOV       N:_ls_pom, #0x0    ;; 1 cycle
//   82   ls_avg_acc_freq_net = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_ls_avg_acc_freq_net, AX  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_ls_avg_acc_freq_net+2, AX  ;; 1 cycle
//   83   ls_status = 0;
        MOV       N:_ls_status, #0x0  ;; 1 cycle
//   84   load_survey_save_avg_acc();
          CFI FunCall _load_survey_save_avg_acc
        CALL      _load_survey_save_avg_acc  ;; 3 cycles
//   85 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock1
        ; ------------------------------------- Block: 45 cycles
        ; ------------------------------------- Total: 45 cycles

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock2 Using cfiCommon0
          CFI Function _load_survey_save_avg_acc
        CODE
//   86 void load_survey_save_avg_acc()
//   87 {
_load_survey_save_avg_acc:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//   88   fill_oprzero(32);
        MOV       A, #0x20           ;; 1 cycle
          CFI FunCall _fill_oprzero
        CALL      _fill_oprzero      ;; 3 cycles
//   89   int_into_char_array(ls_avg_acc_vol_r,&opr_data[0]);
        MOVW      BC, #LWRD(_opr_data)  ;; 1 cycle
        MOVW      AX, N:_ls_avg_acc_vol_r  ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//   90   int_into_char_array(ls_avg_acc_vol_y,&opr_data[2]);
        MOVW      BC, #LWRD(_opr_data+2)  ;; 1 cycle
        MOVW      AX, N:_ls_avg_acc_vol_y  ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//   91   int_into_char_array(ls_avg_acc_vol_b,&opr_data[4]);
        MOVW      BC, #LWRD(_opr_data+4)  ;; 1 cycle
        MOVW      AX, N:_ls_avg_acc_vol_b  ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//   92   long_into_char_array3(ls_avg_acc_curr_r,&opr_data[6]);
        MOVW      DE, #LWRD(_opr_data+6)  ;; 1 cycle
        MOVW      BC, N:_ls_avg_acc_curr_r+2  ;; 1 cycle
        MOVW      AX, N:_ls_avg_acc_curr_r  ;; 1 cycle
          CFI FunCall _long_into_char_array3
        CALL      _long_into_char_array3  ;; 3 cycles
//   93   long_into_char_array3(ls_avg_acc_curr_y,&opr_data[9]);
        MOVW      DE, #LWRD(_opr_data+9)  ;; 1 cycle
        MOVW      BC, N:_ls_avg_acc_curr_y+2  ;; 1 cycle
        MOVW      AX, N:_ls_avg_acc_curr_y  ;; 1 cycle
          CFI FunCall _long_into_char_array3
        CALL      _long_into_char_array3  ;; 3 cycles
//   94   long_into_char_array3(ls_avg_acc_curr_b,&opr_data[12]);
        MOVW      DE, #LWRD(_opr_data+12)  ;; 1 cycle
        MOVW      BC, N:_ls_avg_acc_curr_b+2  ;; 1 cycle
        MOVW      AX, N:_ls_avg_acc_curr_b  ;; 1 cycle
          CFI FunCall _long_into_char_array3
        CALL      _long_into_char_array3  ;; 3 cycles
//   95   long_into_char_array3(ls_avg_acc_curr_n,&opr_data[15]);
        MOVW      DE, #LWRD(_opr_data+15)  ;; 1 cycle
        MOVW      BC, N:_ls_avg_acc_curr_n+2  ;; 1 cycle
        MOVW      AX, N:_ls_avg_acc_curr_n  ;; 1 cycle
          CFI FunCall _long_into_char_array3
        CALL      _long_into_char_array3  ;; 3 cycles
//   96   int_into_char_array(ls_avg_acc_pf_r,&opr_data[18]);
        MOVW      BC, #LWRD(_opr_data+18)  ;; 1 cycle
        MOVW      AX, N:_ls_avg_acc_pf_r  ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//   97   int_into_char_array(ls_avg_acc_pf_y,&opr_data[20]);
        MOVW      BC, #LWRD(_opr_data+20)  ;; 1 cycle
        MOVW      AX, N:_ls_avg_acc_pf_y  ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//   98   int_into_char_array(ls_avg_acc_pf_b,&opr_data[22]);
        MOVW      BC, #LWRD(_opr_data+22)  ;; 1 cycle
        MOVW      AX, N:_ls_avg_acc_pf_b  ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//   99   int_into_char_array(ls_avg_acc_pf_net,&opr_data[24]);
        MOVW      BC, #LWRD(_opr_data+24)  ;; 1 cycle
        MOVW      AX, N:_ls_avg_acc_pf_net  ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  100   opr_data[26] = ls_pom;
        MOV       A, N:_ls_pom       ;; 1 cycle
        MOV       N:_opr_data+26, A  ;; 1 cycle
//  101   long_into_char_array3(ls_avg_acc_freq_net,&opr_data[27]);
        MOVW      DE, #LWRD(_opr_data+27)  ;; 1 cycle
        MOVW      BC, N:_ls_avg_acc_freq_net+2  ;; 1 cycle
        MOVW      AX, N:_ls_avg_acc_freq_net  ;; 1 cycle
          CFI FunCall _long_into_char_array3
        CALL      _long_into_char_array3  ;; 3 cycles
//  102   opr_data[30] = ls_status;
        MOV       A, N:_ls_status    ;; 1 cycle
        MOV       N:_opr_data+30, A  ;; 1 cycle
//  103   eprom_write(0x0730,0,32,PAGE_2,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+6
        MOV       B, #0x1            ;; 1 cycle
        MOVW      DE, #0x20          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x730         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  104 }
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock2
        ; ------------------------------------- Block: 89 cycles
        ; ------------------------------------- Total: 89 cycles

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock3 Using cfiCommon0
          CFI Function _load_survey_avg_acc
          CFI NoCalls
        CODE
//  105 void load_survey_avg_acc()
//  106 {
_load_survey_avg_acc:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  107   ls_avg_acc_vol_r += (vol.Rph.rms/60);
        MOVW      DE, #0x3C          ;; 1 cycle
        MOVW      AX, N:_vol         ;; 1 cycle
        DIVHU                        ;; 9 cycles
        NOP                          ;; 1 cycle
        ADDW      AX, N:_ls_avg_acc_vol_r  ;; 1 cycle
        MOVW      N:_ls_avg_acc_vol_r, AX  ;; 1 cycle
//  108   ls_avg_acc_vol_y += (vol.Yph.rms/60);
        MOVW      DE, #0x3C          ;; 1 cycle
        MOVW      AX, N:_vol+6       ;; 1 cycle
        DIVHU                        ;; 9 cycles
        NOP                          ;; 1 cycle
        ADDW      AX, N:_ls_avg_acc_vol_y  ;; 1 cycle
        MOVW      N:_ls_avg_acc_vol_y, AX  ;; 1 cycle
//  109   ls_avg_acc_vol_b += (vol.Bph.rms/60);
        MOVW      DE, #0x3C          ;; 1 cycle
        MOVW      AX, N:_vol+12      ;; 1 cycle
        DIVHU                        ;; 9 cycles
        NOP                          ;; 1 cycle
        ADDW      AX, N:_ls_avg_acc_vol_b  ;; 1 cycle
        MOVW      N:_ls_avg_acc_vol_b, AX  ;; 1 cycle
//  110   
//  111   ls_avg_acc_curr_r += (curr.Rph.rms);
        MOVW      BC, N:_ls_avg_acc_curr_r+2  ;; 1 cycle
        MOVW      AX, N:_ls_avg_acc_curr_r  ;; 1 cycle
        ADDW      AX, N:_curr        ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, N:_curr+2      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_ls_avg_acc_curr_r, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_ls_avg_acc_curr_r+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  112   ls_avg_acc_curr_y += (curr.Yph.rms);
        MOVW      BC, N:_ls_avg_acc_curr_y+2  ;; 1 cycle
        MOVW      AX, N:_ls_avg_acc_curr_y  ;; 1 cycle
        ADDW      AX, N:_curr+16     ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, N:_curr+18     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_ls_avg_acc_curr_y, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_ls_avg_acc_curr_y+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  113   ls_avg_acc_curr_b += (curr.Bph.rms);
        MOVW      BC, N:_ls_avg_acc_curr_b+2  ;; 1 cycle
        MOVW      AX, N:_ls_avg_acc_curr_b  ;; 1 cycle
        ADDW      AX, N:_curr+32     ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, N:_curr+34     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_ls_avg_acc_curr_b, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_ls_avg_acc_curr_b+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  114   ls_avg_acc_curr_n += (curr.Nph.rms);
        MOVW      BC, N:_ls_avg_acc_curr_n+2  ;; 1 cycle
        MOVW      AX, N:_ls_avg_acc_curr_n  ;; 1 cycle
        ADDW      AX, N:_curr+48     ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, N:_curr+50     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_ls_avg_acc_curr_n, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_ls_avg_acc_curr_n+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  115   
//  116   ls_avg_acc_pf_r += (pf.Rph);
        MOVW      AX, N:_ls_avg_acc_pf_r  ;; 1 cycle
        ADDW      AX, N:_pf          ;; 1 cycle
        MOVW      N:_ls_avg_acc_pf_r, AX  ;; 1 cycle
//  117   ls_avg_acc_pf_y += (pf.Yph);
        MOVW      AX, N:_ls_avg_acc_pf_y  ;; 1 cycle
        ADDW      AX, N:_pf+2        ;; 1 cycle
        MOVW      N:_ls_avg_acc_pf_y, AX  ;; 1 cycle
//  118   ls_avg_acc_pf_b += (pf.Bph);
        MOVW      AX, N:_ls_avg_acc_pf_b  ;; 1 cycle
        ADDW      AX, N:_pf+4        ;; 1 cycle
        MOVW      N:_ls_avg_acc_pf_b, AX  ;; 1 cycle
//  119   ls_avg_acc_pf_net += (pf.Net);
        MOVW      AX, N:_ls_avg_acc_pf_net  ;; 1 cycle
        ADDW      AX, N:_pf+6        ;; 1 cycle
        MOVW      N:_ls_avg_acc_pf_net, AX  ;; 1 cycle
//  120   
//  121   ls_pom++;
        INC       N:_ls_pom          ;; 2 cycles
//  122   
//  123   ls_avg_acc_freq_net += (freq.Net);
        MOVW      AX, N:_freq+6      ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        MOVW      HL, N:_ls_avg_acc_freq_net+2  ;; 1 cycle
        MOVW      DE, N:_ls_avg_acc_freq_net  ;; 1 cycle
        ADDW      AX, DE             ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_ls_avg_acc_freq_net, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_ls_avg_acc_freq_net+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  124 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock3
        ; ------------------------------------- Block: 139 cycles
        ; ------------------------------------- Total: 139 cycles

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock4 Using cfiCommon0
          CFI Function _load_survey_avg_cal
        CODE
//  125 void load_survey_avg_cal()
//  126 {
_load_survey_avg_cal:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  127   //  if(ls_pom > (60/mdi_sel_ls)) /* pending uncomment if needed */
//  128   //  {
//  129   //    ls_pom = 60/mdi_sel_ls;
//  130   //  }
//  131   if(ls_pom != 0)
        CMP0      N:_ls_pom          ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??load_survey_ram_init_0  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  132   {
//  133     vol.Rph.ls_avg = ((us32)ls_avg_acc_vol_r * 60)/ls_pom;
        MOV       X, N:_ls_pom       ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      HL, #0x0           ;; 1 cycle
        MOVW      AX, N:_ls_avg_acc_vol_r  ;; 1 cycle
        MOVW      BC, #0x3C          ;; 1 cycle
        MULHU                        ;; 2 cycles
        DIVWU                        ;; 17 cycles
        NOP                          ;; 1 cycle
        MOVW      N:_vol+2, AX       ;; 1 cycle
//  134     vol.Yph.ls_avg = ((us32)ls_avg_acc_vol_y * 60)/ls_pom;
        MOV       X, N:_ls_pom       ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      HL, #0x0           ;; 1 cycle
        MOVW      AX, N:_ls_avg_acc_vol_y  ;; 1 cycle
        MOVW      BC, #0x3C          ;; 1 cycle
        MULHU                        ;; 2 cycles
        DIVWU                        ;; 17 cycles
        NOP                          ;; 1 cycle
        MOVW      N:_vol+8, AX       ;; 1 cycle
//  135     vol.Bph.ls_avg = ((us32)ls_avg_acc_vol_b * 60)/ls_pom;
        MOV       X, N:_ls_pom       ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      HL, #0x0           ;; 1 cycle
        MOVW      AX, N:_ls_avg_acc_vol_b  ;; 1 cycle
        MOVW      BC, #0x3C          ;; 1 cycle
        MULHU                        ;; 2 cycles
        DIVWU                        ;; 17 cycles
        NOP                          ;; 1 cycle
        MOVW      N:_vol+14, AX      ;; 1 cycle
//  136     curr.Rph.ls_avg = ls_avg_acc_curr_r/ls_pom;
        MOV       X, N:_ls_pom       ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      HL, #0x0           ;; 1 cycle
        MOVW      BC, N:_ls_avg_acc_curr_r+2  ;; 1 cycle
        MOVW      AX, N:_ls_avg_acc_curr_r  ;; 1 cycle
        DIVWU                        ;; 17 cycles
        NOP                          ;; 1 cycle
        MOVW      N:_curr+4, AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_curr+6, AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  137     curr.Yph.ls_avg = ls_avg_acc_curr_y/ls_pom;
        MOV       X, N:_ls_pom       ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      HL, #0x0           ;; 1 cycle
        MOVW      BC, N:_ls_avg_acc_curr_y+2  ;; 1 cycle
        MOVW      AX, N:_ls_avg_acc_curr_y  ;; 1 cycle
        DIVWU                        ;; 17 cycles
        NOP                          ;; 1 cycle
        MOVW      N:_curr+20, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_curr+22, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  138     curr.Bph.ls_avg = ls_avg_acc_curr_b/ls_pom;
        MOV       X, N:_ls_pom       ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      HL, #0x0           ;; 1 cycle
        MOVW      BC, N:_ls_avg_acc_curr_b+2  ;; 1 cycle
        MOVW      AX, N:_ls_avg_acc_curr_b  ;; 1 cycle
        DIVWU                        ;; 17 cycles
        NOP                          ;; 1 cycle
        MOVW      N:_curr+36, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_curr+38, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  139     curr.Nph.ls_avg = ls_avg_acc_curr_n/ls_pom;
        MOV       X, N:_ls_pom       ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      HL, #0x0           ;; 1 cycle
        MOVW      BC, N:_ls_avg_acc_curr_n+2  ;; 1 cycle
        MOVW      AX, N:_ls_avg_acc_curr_n  ;; 1 cycle
        DIVWU                        ;; 17 cycles
        NOP                          ;; 1 cycle
        MOVW      N:_curr+52, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_curr+54, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  140     pf.Rph_ls_avg = ls_avg_acc_pf_r/ls_pom;
        MOV       C, N:_ls_pom       ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOVW      AX, N:_ls_avg_acc_pf_r  ;; 1 cycle
          CFI FunCall ?SI_DIV_L02
        CALL      N:?SI_DIV_L02      ;; 3 cycles
        MOVW      N:_pf+16, AX       ;; 1 cycle
//  141     pf.Yph_ls_avg = ls_avg_acc_pf_y/ls_pom;
        MOV       C, N:_ls_pom       ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOVW      AX, N:_ls_avg_acc_pf_y  ;; 1 cycle
          CFI FunCall ?SI_DIV_L02
        CALL      N:?SI_DIV_L02      ;; 3 cycles
        MOVW      N:_pf+18, AX       ;; 1 cycle
//  142     pf.Bph_ls_avg = ls_avg_acc_pf_b/ls_pom;
        MOV       C, N:_ls_pom       ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOVW      AX, N:_ls_avg_acc_pf_b  ;; 1 cycle
          CFI FunCall ?SI_DIV_L02
        CALL      N:?SI_DIV_L02      ;; 3 cycles
        MOVW      N:_pf+20, AX       ;; 1 cycle
//  143     pf.Net_ls_avg = ls_avg_acc_pf_net/ls_pom;
        MOV       C, N:_ls_pom       ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOVW      AX, N:_ls_avg_acc_pf_net  ;; 1 cycle
          CFI FunCall ?SI_DIV_L02
        CALL      N:?SI_DIV_L02      ;; 3 cycles
        MOVW      N:_pf+22, AX       ;; 1 cycle
//  144     freq.ls_avg = ls_avg_acc_freq_net/ls_pom;
        MOV       X, N:_ls_pom       ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      HL, #0x0           ;; 1 cycle
        MOVW      BC, N:_ls_avg_acc_freq_net+2  ;; 1 cycle
        MOVW      AX, N:_ls_avg_acc_freq_net  ;; 1 cycle
        DIVWU                        ;; 17 cycles
        NOP                          ;; 1 cycle
        MOVW      N:_freq+8, AX      ;; 1 cycle
        BR        S:??load_survey_ram_init_1  ;; 3 cycles
        ; ------------------------------------- Block: 249 cycles
//  145   }
//  146   else
//  147   {
//  148     vol.Rph.ls_avg = 0;
??load_survey_ram_init_0:
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_vol+2, AX       ;; 1 cycle
//  149     vol.Yph.ls_avg = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_vol+8, AX       ;; 1 cycle
//  150     vol.Bph.ls_avg = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_vol+14, AX      ;; 1 cycle
//  151     curr.Rph.ls_avg = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+4, AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+6, AX      ;; 1 cycle
//  152     curr.Yph.ls_avg = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+20, AX     ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+22, AX     ;; 1 cycle
//  153     curr.Bph.ls_avg = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+36, AX     ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+38, AX     ;; 1 cycle
//  154     curr.Nph.ls_avg = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+52, AX     ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+54, AX     ;; 1 cycle
//  155     pf.Rph_ls_avg = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_pf+16, AX       ;; 1 cycle
//  156     pf.Yph_ls_avg = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_pf+18, AX       ;; 1 cycle
//  157     pf.Bph_ls_avg = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_pf+20, AX       ;; 1 cycle
//  158     pf.Net_ls_avg = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_pf+22, AX       ;; 1 cycle
//  159     freq.ls_avg = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_freq+8, AX      ;; 1 cycle
        ; ------------------------------------- Block: 32 cycles
//  160   }
//  161   
//  162   /* calculating demands/energies */
//  163   if(eprom_read(0x0700,0,PAGE_3,AUTO_CALC) == EEP_OK)
??load_survey_ram_init_1:
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x2            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x700         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
        CMP       A, #0x1            ;; 1 cycle
        SKZ                          ;; 4 cycles
        BR        N:??load_survey_ram_init_2  ;; 4 cycles
        ; ------------------------------------- Block: 12 cycles
//  164   {
//  165     temp_us32 = char_array_to_long4(&opr_data[0]);
        MOVW      AX, #LWRD(_opr_data)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      S:_temp_us32, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_temp_us32+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  166     long_int = energy.Allph.active_imp;
        MOVW      BC, N:_energy+54   ;; 1 cycle
        MOVW      AX, N:_energy+52   ;; 1 cycle
        MOVW      S:_long_int, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_long_int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  167     ls_act_imp_demand = ls_cal_energy_diff(long_int, temp_us32);
        MOVW      AX, S:_temp_us32+2  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+6
        MOVW      AX, S:_temp_us32   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+8
        MOVW      BC, S:_long_int+2  ;; 1 cycle
        MOVW      AX, S:_long_int    ;; 1 cycle
          CFI FunCall _ls_cal_energy_diff
        CALL      _ls_cal_energy_diff  ;; 3 cycles
        MOVW      N:_ls_act_imp_demand, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_ls_act_imp_demand+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  168     long_into_char_array4(long_int,&opr_data[0]);
        MOVW      DE, #LWRD(_opr_data)  ;; 1 cycle
        MOVW      BC, S:_long_int+2  ;; 1 cycle
        MOVW      AX, S:_long_int    ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  169     
//  170     temp_us32 = char_array_to_long4(&opr_data[4]);
        MOVW      AX, #LWRD(_opr_data+4)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      S:_temp_us32, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_temp_us32+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  171     long_int = energy.Allph.active_exp;
        MOVW      BC, N:_energy+58   ;; 1 cycle
        MOVW      AX, N:_energy+56   ;; 1 cycle
        MOVW      S:_long_int, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_long_int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  172     ls_act_exp_demand = ls_cal_energy_diff(long_int, temp_us32);
        MOVW      AX, S:_temp_us32+2  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        MOVW      AX, S:_temp_us32   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      BC, S:_long_int+2  ;; 1 cycle
        MOVW      AX, S:_long_int    ;; 1 cycle
          CFI FunCall _ls_cal_energy_diff
        CALL      _ls_cal_energy_diff  ;; 3 cycles
        MOVW      N:_ls_act_exp_demand, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_ls_act_exp_demand+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  173     long_into_char_array4(long_int,&opr_data[4]);
        MOVW      DE, #LWRD(_opr_data+4)  ;; 1 cycle
        MOVW      BC, S:_long_int+2  ;; 1 cycle
        MOVW      AX, S:_long_int    ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  174     
//  175     temp_us32 = char_array_to_long4(&opr_data[8]);
        MOVW      AX, #LWRD(_opr_data+8)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      S:_temp_us32, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_temp_us32+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  176     long_int = energy.Allph.reactive_q1;
        MOVW      BC, N:_energy+78   ;; 1 cycle
        MOVW      AX, N:_energy+76   ;; 1 cycle
        MOVW      S:_long_int, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_long_int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  177     ls_react_q1_demand = ls_cal_energy_diff(long_int, temp_us32);
        MOVW      AX, S:_temp_us32+2  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+14
        MOVW      AX, S:_temp_us32   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+16
        MOVW      BC, S:_long_int+2  ;; 1 cycle
        MOVW      AX, S:_long_int    ;; 1 cycle
          CFI FunCall _ls_cal_energy_diff
        CALL      _ls_cal_energy_diff  ;; 3 cycles
        MOVW      N:_ls_react_q1_demand, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_ls_react_q1_demand+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  178     long_into_char_array4(long_int,&opr_data[8]);
        MOVW      DE, #LWRD(_opr_data+8)  ;; 1 cycle
        MOVW      BC, S:_long_int+2  ;; 1 cycle
        MOVW      AX, S:_long_int    ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  179     
//  180     temp_us32 = char_array_to_long4(&opr_data[12]);
        MOVW      AX, #LWRD(_opr_data+12)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      S:_temp_us32, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_temp_us32+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  181     long_int = energy.Allph.reactive_q2;
        MOVW      BC, N:_energy+82   ;; 1 cycle
        MOVW      AX, N:_energy+80   ;; 1 cycle
        MOVW      S:_long_int, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_long_int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  182     ls_react_q2_demand = ls_cal_energy_diff(long_int, temp_us32);
        MOVW      AX, S:_temp_us32+2  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+18
        MOVW      AX, S:_temp_us32   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+20
        MOVW      BC, S:_long_int+2  ;; 1 cycle
        MOVW      AX, S:_long_int    ;; 1 cycle
          CFI FunCall _ls_cal_energy_diff
        CALL      _ls_cal_energy_diff  ;; 3 cycles
        MOVW      N:_ls_react_q2_demand, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_ls_react_q2_demand+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  183     long_into_char_array4(long_int,&opr_data[12]);
        MOVW      DE, #LWRD(_opr_data+12)  ;; 1 cycle
        MOVW      BC, S:_long_int+2  ;; 1 cycle
        MOVW      AX, S:_long_int    ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  184     
//  185     temp_us32 = char_array_to_long4(&opr_data[16]);
        MOVW      AX, #LWRD(_opr_data+16)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      S:_temp_us32, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_temp_us32+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  186     long_int = energy.Allph.reactive_q3;
        MOVW      BC, N:_energy+86   ;; 1 cycle
        MOVW      AX, N:_energy+84   ;; 1 cycle
        MOVW      S:_long_int, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_long_int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  187     ls_react_q3_demand = ls_cal_energy_diff(long_int, temp_us32);
        MOVW      AX, S:_temp_us32+2  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+22
        MOVW      AX, S:_temp_us32   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+24
        MOVW      BC, S:_long_int+2  ;; 1 cycle
        MOVW      AX, S:_long_int    ;; 1 cycle
          CFI FunCall _ls_cal_energy_diff
        CALL      _ls_cal_energy_diff  ;; 3 cycles
        MOVW      N:_ls_react_q3_demand, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_ls_react_q3_demand+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  188     long_into_char_array4(long_int,&opr_data[16]);
        MOVW      DE, #LWRD(_opr_data+16)  ;; 1 cycle
        MOVW      BC, S:_long_int+2  ;; 1 cycle
        MOVW      AX, S:_long_int    ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  189     
//  190     temp_us32 = char_array_to_long4(&opr_data[20]);
        MOVW      AX, #LWRD(_opr_data+20)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      S:_temp_us32, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_temp_us32+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  191     long_int = energy.Allph.reactive_q4;
        MOVW      BC, N:_energy+90   ;; 1 cycle
        MOVW      AX, N:_energy+88   ;; 1 cycle
        MOVW      S:_long_int, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_long_int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  192     ls_react_q4_demand = ls_cal_energy_diff(long_int, temp_us32);
        MOVW      AX, S:_temp_us32+2  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+26
        MOVW      AX, S:_temp_us32   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+28
        MOVW      BC, S:_long_int+2  ;; 1 cycle
        MOVW      AX, S:_long_int    ;; 1 cycle
          CFI FunCall _ls_cal_energy_diff
        CALL      _ls_cal_energy_diff  ;; 3 cycles
        MOVW      N:_ls_react_q4_demand, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_ls_react_q4_demand+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  193     long_into_char_array4(long_int,&opr_data[20]);
        MOVW      DE, #LWRD(_opr_data+20)  ;; 1 cycle
        MOVW      BC, S:_long_int+2  ;; 1 cycle
        MOVW      AX, S:_long_int    ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  194     
//  195     temp_us32 = char_array_to_long4(&opr_data[24]);
        MOVW      AX, #LWRD(_opr_data+24)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      S:_temp_us32, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_temp_us32+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  196     long_int = energy.Allph.apparent_imp;
        MOVW      BC, N:_energy+70   ;; 1 cycle
        MOVW      AX, N:_energy+68   ;; 1 cycle
        MOVW      S:_long_int, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_long_int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  197     ls_app_imp_demand = ls_cal_energy_diff(long_int, temp_us32);
        MOVW      AX, S:_temp_us32+2  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+30
        MOVW      AX, S:_temp_us32   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
        MOVW      BC, S:_long_int+2  ;; 1 cycle
        MOVW      AX, S:_long_int    ;; 1 cycle
          CFI FunCall _ls_cal_energy_diff
        CALL      _ls_cal_energy_diff  ;; 3 cycles
        MOVW      N:_ls_app_imp_demand, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_ls_app_imp_demand+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  198     long_into_char_array4(long_int,&opr_data[24]);
        MOVW      DE, #LWRD(_opr_data+24)  ;; 1 cycle
        MOVW      BC, S:_long_int+2  ;; 1 cycle
        MOVW      AX, S:_long_int    ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  199     
//  200     temp_us32 = char_array_to_long4(&opr_data[28]);
        MOVW      AX, #LWRD(_opr_data+28)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      S:_temp_us32, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_temp_us32+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  201     long_int = energy.Allph.apparent_exp;
        MOVW      BC, N:_energy+74   ;; 1 cycle
        MOVW      AX, N:_energy+72   ;; 1 cycle
        MOVW      S:_long_int, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_long_int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  202     ls_app_exp_demand = ls_cal_energy_diff(long_int, temp_us32);
        MOVW      AX, S:_temp_us32+2  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+34
        MOVW      AX, S:_temp_us32   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+36
        MOVW      BC, S:_long_int+2  ;; 1 cycle
        MOVW      AX, S:_long_int    ;; 1 cycle
          CFI FunCall _ls_cal_energy_diff
        CALL      _ls_cal_energy_diff  ;; 3 cycles
        MOVW      N:_ls_app_exp_demand, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_ls_app_exp_demand+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  203     long_into_char_array4(long_int,&opr_data[28]);
        MOVW      DE, #LWRD(_opr_data+28)  ;; 1 cycle
        MOVW      BC, S:_long_int+2  ;; 1 cycle
        MOVW      AX, S:_long_int    ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  204     eprom_write(0x0700,0,48,PAGE_3,AUTO_CALC);  /* load_survey_fill_last_energy function broken to execute efficient way */
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+38
        MOV       B, #0x2            ;; 1 cycle
        MOVW      DE, #0x30          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x700         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
        ADDW      SP, #0x22          ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
        ; ------------------------------------- Block: 280 cycles
//  205   }
//  206   else
//  207   {
//  208     ls_act_imp_demand = 0;
??load_survey_ram_init_2:
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_ls_act_imp_demand, AX  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_ls_act_imp_demand+2, AX  ;; 1 cycle
//  209     ls_act_exp_demand = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_ls_act_exp_demand, AX  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_ls_act_exp_demand+2, AX  ;; 1 cycle
//  210     ls_react_q1_demand = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_ls_react_q1_demand, AX  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_ls_react_q1_demand+2, AX  ;; 1 cycle
//  211     ls_react_q2_demand = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_ls_react_q2_demand, AX  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_ls_react_q2_demand+2, AX  ;; 1 cycle
//  212     ls_react_q3_demand = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_ls_react_q3_demand, AX  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_ls_react_q3_demand+2, AX  ;; 1 cycle
//  213     ls_react_q4_demand = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_ls_react_q4_demand, AX  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_ls_react_q4_demand+2, AX  ;; 1 cycle
//  214     ls_app_imp_demand = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_ls_app_imp_demand, AX  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_ls_app_imp_demand+2, AX  ;; 1 cycle
//  215     ls_app_exp_demand = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_ls_app_exp_demand, AX  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_ls_app_exp_demand+2, AX  ;; 1 cycle
//  216     load_survey_fill_last_energy();
          CFI FunCall _load_survey_fill_last_energy
        CALL      _load_survey_fill_last_energy  ;; 3 cycles
//  217     ls_status |= bit1;
        SET1      N:_ls_status.1     ;; 2 cycles
//  218   }
//  219   
//  220 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock4
        ; ------------------------------------- Block: 43 cycles
        ; ------------------------------------- Total: 621 cycles
//  221 /* ls_status 
//  222 bit0 - ls snapshot mode  ( 0-normal,1-ch miss md)
//  223 bit1 - ls last energy corrupted 
//  224 bit2 - ls avg acc date error at power up
//  225 bit3 - meter hard reset detected
//  226 bit4 - Meter restarted detected 
//  227 bit5 - ls_miss_fill 
//  228 */

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock5 Using cfiCommon0
          CFI Function _load_survey_snap_shot
        CODE
//  229 void load_survey_snap_shot()
//  230 {
_load_survey_snap_shot:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 22
        SUBW      SP, #0x16          ;; 1 cycle
          CFI CFA SP+26
//  231   uint8 u8proceed;
//  232   uint16 temp_hr=0,temp_min=0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [SP+0x06], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [SP+0x04], AX      ;; 1 cycle
//  233   us8 ls_eep_id;
//  234   us16 ls_address = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [SP+0x08], AX      ;; 1 cycle
//  235   us32 ls_pg1_address = 0;
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0xA           ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
//  236   us8 ls_day_change1 = 0;
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP+0x01], A       ;; 1 cycle
//  237   
//  238   seq_no_ls++;
        MOVW      BC, N:_seq_no_ls+2  ;; 1 cycle
        MOVW      AX, N:_seq_no_ls   ;; 1 cycle
        ADDW      AX, #0x1           ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_seq_no_ls, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_seq_no_ls+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  239   if(miss_ls_date_f==1)
        CMP       N:_miss_ls_date_f, #0x1  ;; 1 cycle
        BNZ       ??load_survey_ram_init_3  ;; 4 cycles
        ; ------------------------------------- Block: 36 cycles
//  240   {
//  241     TempTime = load_survey_next_interval_timestamp(lsip_period,OffTime);
        MOVW      HL, #LWRD(_OffTime)  ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+34
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOV       C, N:_lsip_period  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x16          ;; 1 cycle
          CFI FunCall _load_survey_next_interval_timestamp
        CALL      _load_survey_next_interval_timestamp  ;; 3 cycles
        MOVW      HL, #LWRD(_TempTime)  ;; 1 cycle
        ADDW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+26
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0xE           ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOV       A, [DE]            ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
        MOV       A, [DE+0x01]       ;; 1 cycle
        MOV       [HL+0x01], A       ;; 1 cycle
        MOV       A, [DE+0x02]       ;; 1 cycle
        MOV       [HL+0x02], A       ;; 1 cycle
        MOV       A, [DE+0x03]       ;; 1 cycle
        MOV       [HL+0x03], A       ;; 1 cycle
        MOV       A, [DE+0x04]       ;; 1 cycle
        MOV       [HL+0x04], A       ;; 1 cycle
        MOV       A, [DE+0x05]       ;; 1 cycle
        MOV       [HL+0x05], A       ;; 1 cycle
        MOV       A, [DE+0x06]       ;; 1 cycle
        MOV       [HL+0x06], A       ;; 1 cycle
//  242     if(TempTime.hour == 0 && TempTime.min == 0)
        CMP0      N:_TempTime+2      ;; 1 cycle
        BNZ       ??load_survey_ram_init_4  ;; 4 cycles
        ; ------------------------------------- Block: 37 cycles
        CMP0      N:_TempTime+1      ;; 1 cycle
        BNZ       ??load_survey_ram_init_4  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  243     {
//  244       ls_day_change1 = 1;
        MOV       A, #0x1            ;; 1 cycle
        MOV       [SP+0x01], A       ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
//  245     }
//  246     ls_status |= bit0;
??load_survey_ram_init_4:
        SET1      N:_ls_status.0     ;; 2 cycles
        BR        S:??load_survey_ram_init_5  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  247   }
//  248   else
//  249   {
//  250     TempTime = Now;
??load_survey_ram_init_3:
        MOVW      HL, #LWRD(_TempTime)  ;; 1 cycle
        MOVW      DE, #LWRD(_Now)    ;; 1 cycle
        MOV       A, [DE]            ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
        MOV       A, [DE+0x01]       ;; 1 cycle
        MOV       [HL+0x01], A       ;; 1 cycle
        MOV       A, [DE+0x02]       ;; 1 cycle
        MOV       [HL+0x02], A       ;; 1 cycle
        MOV       A, [DE+0x03]       ;; 1 cycle
        MOV       [HL+0x03], A       ;; 1 cycle
        MOV       A, [DE+0x04]       ;; 1 cycle
        MOV       [HL+0x04], A       ;; 1 cycle
        MOV       A, [DE+0x05]       ;; 1 cycle
        MOV       [HL+0x05], A       ;; 1 cycle
        MOV       A, [DE+0x06]       ;; 1 cycle
        MOV       [HL+0x06], A       ;; 1 cycle
        ; ------------------------------------- Block: 16 cycles
//  251   }
//  252   temp_hr = bcd_to_decimal(TempTime.hour); 
??load_survey_ram_init_5:
        MOV       A, N:_TempTime+2   ;; 1 cycle
          CFI FunCall _bcd_to_decimal
        CALL      _bcd_to_decimal    ;; 3 cycles
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
//  253   temp_hr *= mdi_sel_ls;
        MOV       C, N:_mdi_sel_ls   ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      [SP+0x06], AX      ;; 1 cycle
//  254   temp_min = bcd_to_decimal(TempTime.min);
        MOV       A, N:_TempTime+1   ;; 1 cycle
          CFI FunCall _bcd_to_decimal
        CALL      _bcd_to_decimal    ;; 3 cycles
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
//  255   temp_min /= lsip_period;
        XCH       A, D               ;; 1 cycle
        MOV       A, N:_lsip_period  ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        DIVHU                        ;; 9 cycles
        NOP                          ;; 1 cycle
        MOVW      [SP+0x04], AX      ;; 1 cycle
//  256   
//  257   /* to avoid overwriting on previous day 00:00 location on date change as day counter
//  258   increases after this function call. */
//  259   
//  260   load_survey_cnt = (24*mdi_sel_ls*(day_counter+ls_day_change1)) + temp_hr + temp_min - 1;
        MOV       A, [SP+0x01]       ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        ADDW      AX, N:_day_counter  ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+28
        POP       HL                 ;; 1 cycle
          CFI CFA SP+26
        MOV       X, N:_mdi_sel_ls   ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x18          ;; 1 cycle
        MULHU                        ;; 2 cycles
        PUSH      HL                 ;; 1 cycle
          CFI CFA SP+28
        POP       BC                 ;; 1 cycle
          CFI CFA SP+26
        MULHU                        ;; 2 cycles
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, HL             ;; 1 cycle
        ADDW      AX, DE             ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, HL             ;; 1 cycle
        ADDW      AX, DE             ;; 1 cycle
        DECW      AX                 ;; 1 cycle
        MOVW      N:_load_survey_cnt, AX  ;; 1 cycle
//  261   
//  262   if(load_survey_cnt >= max_load_survey_cnt)
        MOVW      HL, N:_max_load_survey_cnt  ;; 1 cycle
        MOVW      AX, N:_load_survey_cnt  ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        SKC                          ;; 1 cycle
          CFI FunCall _ls_rollover
        ; ------------------------------------- Block: 65 cycles
//  263   {
//  264     ls_rollover();
        CALL      _ls_rollover       ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
//  265   }
//  266   
//  267   
//  268   u8proceed=1;
??load_survey_snap_shot_0:
        MOV       A, #0x1            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  269   if(load_survey_cnt<COMPART_LS_ENTRIES) /* eeprom 0 */
        MOVW      AX, N:_load_survey_cnt  ;; 1 cycle
        CMPW      AX, #0xE40         ;; 1 cycle
        BNC       ??load_survey_ram_init_6  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
//  270   {
//  271     ls_pg1_address = ((uint32_t)COMPART_LS_START_ADD+((us32)load_survey_cnt*COMPART_LS_SIZE));   
        MOVW      AX, N:_load_survey_cnt  ;; 1 cycle
        MOVW      BC, #0x20          ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #0xCB00        ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0xA           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  272     ls_address=ls_pg1_address%COMPART_LS_MEM_BYTES;
        MOVW      AX, [SP+0x0A]      ;; 1 cycle
        MOVW      [SP+0x08], AX      ;; 1 cycle
//  273     ls_eep_id=ls_pg1_address/COMPART_LS_MEM_BYTES;
        MOVW      AX, [SP+0x0C]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x0A]      ;; 1 cycle
        CLRW      AX                 ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       [SP+0x02], A       ;; 1 cycle
        BR        S:??load_survey_ram_init_7  ;; 3 cycles
        ; ------------------------------------- Block: 33 cycles
//  274   }
//  275   else
//  276   {
//  277     u8proceed = 0;
??load_survey_ram_init_6:
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
//  278   }
//  279   
//  280   if(1==u8proceed)
??load_survey_ram_init_7:
        MOV       A, [SP]            ;; 1 cycle
        CMP       A, #0x1            ;; 1 cycle
        SKZ                          ;; 4 cycles
        BR        N:??load_survey_ram_init_8  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  281   {
//  282     u8proceed = 0;
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  283     
//  284     /* Calculating the averages */
//  285     load_survey_avg_cal();
          CFI FunCall _load_survey_avg_cal
        CALL      _load_survey_avg_cal  ;; 3 cycles
//  286     
//  287     if(ls_act_imp_demand > ls_app_imp_demand)
        MOVW      AX, N:_ls_act_imp_demand+2  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+28
        MOVW      AX, N:_ls_act_imp_demand  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+30
        MOVW      BC, N:_ls_app_imp_demand+2  ;; 1 cycle
        MOVW      AX, N:_ls_app_imp_demand  ;; 1 cycle
          CFI FunCall ?UL_CMP_L03
        CALL      N:?UL_CMP_L03      ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+26
        BNC       ??load_survey_ram_init_9  ;; 4 cycles
        ; ------------------------------------- Block: 19 cycles
//  288     {
//  289       ls_app_imp_demand = ls_act_imp_demand;
        MOVW      BC, N:_ls_act_imp_demand+2  ;; 1 cycle
        MOVW      AX, N:_ls_act_imp_demand  ;; 1 cycle
        MOVW      N:_ls_app_imp_demand, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_ls_app_imp_demand+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
//  290     }
//  291     
//  292     time_into_char_array5(TempTime,&opr_data[0]);
??load_survey_ram_init_9:
        MOVW      HL, #LWRD(_TempTime)  ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+34
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      AX, #LWRD(_opr_data)  ;; 1 cycle
          CFI FunCall _time_into_char_array5
        CALL      _time_into_char_array5  ;; 3 cycles
//  293     int_into_char_array(vol.Rph.ls_avg,&opr_data[5]);
        MOVW      BC, #LWRD(_opr_data+5)  ;; 1 cycle
        MOVW      AX, N:_vol+2       ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  294     int_into_char_array(vol.Yph.ls_avg,&opr_data[7]);
        MOVW      BC, #LWRD(_opr_data+7)  ;; 1 cycle
        MOVW      AX, N:_vol+8       ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  295     int_into_char_array(vol.Bph.ls_avg,&opr_data[9]);
        MOVW      BC, #LWRD(_opr_data+9)  ;; 1 cycle
        MOVW      AX, N:_vol+14      ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  296     int_into_char_array(curr.Rph.ls_avg/10,&opr_data[11]);
        MOVW      BC, #LWRD(_opr_data+11)  ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+36
        MOVW      DE, #0xA           ;; 1 cycle
        MOVW      HL, #0x0           ;; 1 cycle
        MOVW      BC, N:_curr+6      ;; 1 cycle
        MOVW      AX, N:_curr+4      ;; 1 cycle
        DIVWU                        ;; 17 cycles
        NOP                          ;; 1 cycle
        POP       BC                 ;; 1 cycle
          CFI CFA SP+34
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  297     int_into_char_array(curr.Yph.ls_avg/10,&opr_data[13]);
        MOVW      BC, #LWRD(_opr_data+13)  ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+36
        MOVW      DE, #0xA           ;; 1 cycle
        MOVW      HL, #0x0           ;; 1 cycle
        MOVW      BC, N:_curr+22     ;; 1 cycle
        MOVW      AX, N:_curr+20     ;; 1 cycle
        DIVWU                        ;; 17 cycles
        NOP                          ;; 1 cycle
        POP       BC                 ;; 1 cycle
          CFI CFA SP+34
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  298     int_into_char_array(curr.Bph.ls_avg/10,&opr_data[15]);
        MOVW      BC, #LWRD(_opr_data+15)  ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+36
        MOVW      DE, #0xA           ;; 1 cycle
        MOVW      HL, #0x0           ;; 1 cycle
        MOVW      BC, N:_curr+38     ;; 1 cycle
        MOVW      AX, N:_curr+36     ;; 1 cycle
        DIVWU                        ;; 17 cycles
        NOP                          ;; 1 cycle
        POP       BC                 ;; 1 cycle
          CFI CFA SP+34
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  299     int_into_char_array((us16)ls_act_imp_demand,&opr_data[17]);
        MOVW      BC, #LWRD(_opr_data+17)  ;; 1 cycle
        MOVW      HL, N:_ls_act_imp_demand+2  ;; 1 cycle
        MOVW      DE, N:_ls_act_imp_demand  ;; 1 cycle
        MOVW      AX, DE             ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  300     int_into_char_array((us16)ls_app_imp_demand,&opr_data[19]);
        MOVW      BC, #LWRD(_opr_data+19)  ;; 1 cycle
        MOVW      HL, N:_ls_app_imp_demand+2  ;; 1 cycle
        MOVW      DE, N:_ls_app_imp_demand  ;; 1 cycle
        MOVW      AX, DE             ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  301     int_into_char_array((us16)ls_react_q1_demand,&opr_data[21]);
        MOVW      BC, #LWRD(_opr_data+21)  ;; 1 cycle
        MOVW      HL, N:_ls_react_q1_demand+2  ;; 1 cycle
        MOVW      DE, N:_ls_react_q1_demand  ;; 1 cycle
        MOVW      AX, DE             ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  302     int_into_char_array((us16)ls_react_q4_demand,&opr_data[23]);
        MOVW      BC, #LWRD(_opr_data+23)  ;; 1 cycle
        MOVW      HL, N:_ls_react_q4_demand+2  ;; 1 cycle
        MOVW      DE, N:_ls_react_q4_demand  ;; 1 cycle
        MOVW      AX, DE             ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  303     opr_data[25] = ls_pom;
        MOV       A, N:_ls_pom       ;; 1 cycle
        MOV       N:_opr_data+25, A  ;; 1 cycle
//  304     long_into_char_array3(seq_no_ls,&opr_data[26]);
        MOVW      DE, #LWRD(_opr_data+26)  ;; 1 cycle
        MOVW      BC, N:_seq_no_ls+2  ;; 1 cycle
        MOVW      AX, N:_seq_no_ls   ;; 1 cycle
          CFI FunCall _long_into_char_array3
        CALL      _long_into_char_array3  ;; 3 cycles
//  305     opr_data[29] = ls_status;
        MOV       A, N:_ls_status    ;; 1 cycle
        MOV       N:_opr_data+29, A  ;; 1 cycle
//  306     eprom_write(ls_address,ls_eep_id,COMPART_LS_SIZE,PAGE_2,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+36
        MOV       B, #0x1            ;; 1 cycle
        MOVW      DE, #0x20          ;; 1 cycle
        MOV       A, [SP+0x0C]       ;; 1 cycle
        MOV       C, A               ;; 1 cycle
        MOVW      AX, [SP+0x12]      ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  307     
//  308     eprom_read(0x0770,0,PAGE_1,AUTO_CALC);
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x770         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
//  309     long_into_char_array3(seq_no_ls,&opr_data[0]);
        MOVW      DE, #LWRD(_opr_data)  ;; 1 cycle
        MOVW      BC, N:_seq_no_ls+2  ;; 1 cycle
        MOVW      AX, N:_seq_no_ls   ;; 1 cycle
          CFI FunCall _long_into_char_array3
        CALL      _long_into_char_array3  ;; 3 cycles
//  310     eprom_write(0x0770,0,16,PAGE_1,AUTO_CALC);    
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+38
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x770         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  311     
//  312     load_survey_reset_avg_acc();
          CFI FunCall _load_survey_reset_avg_acc
        CALL      _load_survey_reset_avg_acc  ;; 3 cycles
        ADDW      SP, #0xC           ;; 1 cycle
          CFI CFA SP+26
        ; ------------------------------------- Block: 184 cycles
//  313   }
//  314 }
??load_survey_ram_init_8:
        ADDW      SP, #0x16          ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock5
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 434 cycles
//  315 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock6 Using cfiCommon1
          CFI Function _ls_cal_energy_diff
          CFI NoCalls
        CODE
//  316 us32 ls_cal_energy_diff(us32 energy, us32 last_energy)
//  317 {
_ls_cal_energy_diff:
        ; * Stack frame (at entry) *
        ; Param size: 4
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+6
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+8
        ; Auto size: 8
        SUBW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+12
//  318   us32 temp;
//  319   if(energy < last_energy )
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0xC           ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        CMP       A, [HL+0x03]       ;; 1 cycle
        BNZ       ??load_survey_ram_init_10  ;; 4 cycles
        ; ------------------------------------- Block: 15 cycles
        MOV       A, C               ;; 1 cycle
        CMP       A, [HL+0x02]       ;; 1 cycle
        BNZ       ??load_survey_ram_init_10  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOV       A, B               ;; 1 cycle
        CMP       A, [HL+0x01]       ;; 1 cycle
        BNZ       ??load_survey_ram_init_10  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOV       A, X               ;; 1 cycle
        CMP       A, [HL]            ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
??load_survey_ram_init_10:
        BNC       ??load_survey_ram_init_11  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
//  320   {
//  321     temp = (us32)(energy + (ROLL_OVER_LIMIT  - last_energy));
        MOVW      AX, [SP+0x0E]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [SP+0x0C]      ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        SUBW      AX, #0xD800        ;; 1 cycle
        SKNC
        DECW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        SUBW      AX, #0x1194        ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SUBW      AX, DE             ;; 1 cycle
        SKNC
        DECW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        SUBW      AX, HL             ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      HL, SP             ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        BR        S:??load_survey_ram_init_12  ;; 3 cycles
        ; ------------------------------------- Block: 33 cycles
//  322   }
//  323   else
//  324   {
//  325     temp = (us32)(energy - last_energy);
??load_survey_ram_init_11:
        MOVW      AX, [SP+0x0E]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [SP+0x0C]      ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        SUBW      AX, DE             ;; 1 cycle
        SKNC
        DECW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        SUBW      AX, HL             ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      HL, SP             ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ; ------------------------------------- Block: 21 cycles
//  326   }
//  327 #if LS_SNAPSHOT_IS_DEMAND == 1
//  328   temp = temp * mdi_sel_ls; 
//  329   if(temp > (MD_LIMIT))
//  330   {
//  331     temp = 0;
//  332   } 
//  333 #else
//  334   if(temp > (MD_LIMIT/mdi_sel_ls))
??load_survey_ram_init_12:
        MOVW      AX, SP             ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        PUSH      HL                 ;; 1 cycle
          CFI CFA SP+14
        MOV       X, N:_mdi_sel_ls   ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      HL, #0x0           ;; 1 cycle
        MOVW      AX, #0x2FC0        ;; 1 cycle
        MOVW      BC, #0x1           ;; 1 cycle
        DIVWU                        ;; 17 cycles
        NOP                          ;; 1 cycle
        POP       HL                 ;; 1 cycle
          CFI CFA SP+12
        XCH       A, B               ;; 1 cycle
        CMP       A, [HL+0x03]       ;; 1 cycle
        BNZ       ??load_survey_ram_init_13  ;; 4 cycles
        ; ------------------------------------- Block: 34 cycles
        MOV       A, C               ;; 1 cycle
        CMP       A, [HL+0x02]       ;; 1 cycle
        BNZ       ??load_survey_ram_init_13  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOV       A, B               ;; 1 cycle
        CMP       A, [HL+0x01]       ;; 1 cycle
        BNZ       ??load_survey_ram_init_13  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOV       A, X               ;; 1 cycle
        CMP       A, [HL]            ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
??load_survey_ram_init_13:
        BNC       ??load_survey_ram_init_14  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
//  335   {
//  336     temp = 0;
        MOVW      AX, SP             ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
//  337   } 
//  338 #endif  
//  339   return(temp);
??load_survey_ram_init_14:
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        ADDW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock6
        ; ------------------------------------- Block: 10 cycles
        ; ------------------------------------- Total: 155 cycles
//  340 }
//  341 
//  342 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock7 Using cfiCommon0
          CFI Function _ls_miss_fill
        CODE
//  343 void ls_miss_fill(void)
//  344 {
_ls_miss_fill:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 10
        SUBW      SP, #0xA           ;; 1 cycle
          CFI CFA SP+14
//  345   uint8 ls_eep_id,u8proceed;
//  346   uint16 n;
//  347   uint16 ls_cnt_start_1day,ls_cnt_end_1day,temp_mdi=0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [SP+0x02], AX      ;; 1 cycle
//  348   //  uint8 temp_min,temp_hr,temp_date,temp_month,temp_year;
//  349   
//  350   if(ls_rev_fill==1)
        CMP       N:_ls_rev_fill, #0x1  ;; 1 cycle
        BNZ       ??load_survey_ram_init_15  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
//  351   {
//  352     TempTime.hour = 0;          //var4=0;
        MOV       N:_TempTime+2, #0x0  ;; 1 cycle
//  353     TempTime.min = 0;           //var5=0;     
        MOV       N:_TempTime+1, #0x0  ;; 1 cycle
//  354     load_survey_cnt = (24*mdi_sel_ls*(day_counter)); /* d_cnt_tmp */
        MOVW      BC, N:_day_counter  ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+16
        POP       HL                 ;; 1 cycle
          CFI CFA SP+14
        MOV       X, N:_mdi_sel_ls   ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x18          ;; 1 cycle
        MULHU                        ;; 2 cycles
        PUSH      HL                 ;; 1 cycle
          CFI CFA SP+16
        POP       BC                 ;; 1 cycle
          CFI CFA SP+14
        MULHU                        ;; 2 cycles
        MOVW      N:_load_survey_cnt, AX  ;; 1 cycle
        ; ------------------------------------- Block: 15 cycles
//  355   }
//  356   
//  357   //if((Now.day != var3 || Now.month != var2 || Now.year != var1) && ls_fg_f==0)
//  358   if((Now.day != TempTime.day || Now.month != TempTime.month || Now.year != TempTime.year) && ls_fg_f==0)       /* Days change */
??load_survey_ram_init_15:
        MOV       A, N:_Now+3        ;; 1 cycle
        CMP       A, N:_TempTime+3   ;; 1 cycle
        BNZ       ??load_survey_ram_init_16  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOV       A, N:_Now+5        ;; 1 cycle
        CMP       A, N:_TempTime+5   ;; 1 cycle
        BNZ       ??load_survey_ram_init_16  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOV       A, N:_Now+6        ;; 1 cycle
        CMP       A, N:_TempTime+6   ;; 1 cycle
        BZ        ??load_survey_ram_init_17  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
??load_survey_ram_init_16:
        CMP0      N:_ls_fg_f         ;; 1 cycle
        BNZ       ??load_survey_ram_init_17  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  359   {
//  360     ls_cnt_start_1day = 2 + (bcd_to_decimal(OffTime.hour)*60+bcd_to_decimal(OffTime.min))/lsip_period - ls_rtc_fill;
        MOV       A, N:_OffTime+2    ;; 1 cycle
          CFI FunCall _bcd_to_decimal
        CALL      _bcd_to_decimal    ;; 3 cycles
        MOV       [SP+0x04], A       ;; 1 cycle
        MOV       A, N:_OffTime+1    ;; 1 cycle
          CFI FunCall _bcd_to_decimal
        CALL      _bcd_to_decimal    ;; 3 cycles
        MOV       E, A               ;; 1 cycle
        MOV       C, N:_lsip_period  ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+16
        POP       HL                 ;; 1 cycle
          CFI CFA SP+14
        MOV       A, [SP+0x04]       ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x3C          ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOV       D, #0x0            ;; 1 cycle
        ADDW      AX, DE             ;; 1 cycle
        PUSH      HL                 ;; 1 cycle
          CFI CFA SP+16
        POP       BC                 ;; 1 cycle
          CFI CFA SP+14
          CFI FunCall ?SI_DIV_L02
        CALL      N:?SI_DIV_L02      ;; 3 cycles
        ADDW      AX, #0x2           ;; 1 cycle
        MOV       C, N:_ls_rtc_fill  ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        SUBW      AX, BC             ;; 1 cycle
        MOVW      [SP+0x06], AX      ;; 1 cycle
//  361     ls_cnt_end_1day = 24*mdi_sel_ls + (bcd_to_decimal(Now.hour)*60+bcd_to_decimal(Now.min))/lsip_period;
        MOV       A, N:_Now+2        ;; 1 cycle
          CFI FunCall _bcd_to_decimal
        CALL      _bcd_to_decimal    ;; 3 cycles
        MOV       [SP+0x04], A       ;; 1 cycle
        MOV       A, N:_Now+1        ;; 1 cycle
          CFI FunCall _bcd_to_decimal
        CALL      _bcd_to_decimal    ;; 3 cycles
        MOV       E, A               ;; 1 cycle
        MOV       C, N:_lsip_period  ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+16
        POP       HL                 ;; 1 cycle
          CFI CFA SP+14
        MOV       A, [SP+0x04]       ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x3C          ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOV       D, #0x0            ;; 1 cycle
        ADDW      AX, DE             ;; 1 cycle
        PUSH      HL                 ;; 1 cycle
          CFI CFA SP+16
        POP       BC                 ;; 1 cycle
          CFI CFA SP+14
          CFI FunCall ?SI_DIV_L02
        CALL      N:?SI_DIV_L02      ;; 3 cycles
        MOVW      HL, AX             ;; 1 cycle
        MOV       X, N:_mdi_sel_ls   ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x18          ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, HL             ;; 1 cycle
        MOVW      [SP+0x08], AX      ;; 1 cycle
        BR        S:??load_survey_ram_init_18  ;; 3 cycles
        ; ------------------------------------- Block: 70 cycles
//  362   }
//  363   else
//  364   {
//  365     ls_cnt_start_1day = 1+(bcd_to_decimal(TempTime.hour)*60+bcd_to_decimal(TempTime.min))/lsip_period - ls_fg_f - ls_rtc_fill + ls_rev_fill;
??load_survey_ram_init_17:
        MOV       A, N:_TempTime+2   ;; 1 cycle
          CFI FunCall _bcd_to_decimal
        CALL      _bcd_to_decimal    ;; 3 cycles
        MOV       [SP+0x04], A       ;; 1 cycle
        MOV       A, N:_TempTime+1   ;; 1 cycle
          CFI FunCall _bcd_to_decimal
        CALL      _bcd_to_decimal    ;; 3 cycles
        MOV       E, A               ;; 1 cycle
        MOV       C, N:_lsip_period  ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+16
        POP       HL                 ;; 1 cycle
          CFI CFA SP+14
        MOV       A, [SP+0x04]       ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x3C          ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOV       D, #0x0            ;; 1 cycle
        ADDW      AX, DE             ;; 1 cycle
        PUSH      HL                 ;; 1 cycle
          CFI CFA SP+16
        POP       BC                 ;; 1 cycle
          CFI CFA SP+14
          CFI FunCall ?SI_DIV_L02
        CALL      N:?SI_DIV_L02      ;; 3 cycles
        INCW      AX                 ;; 1 cycle
        MOV       C, N:_ls_fg_f      ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        SUBW      AX, BC             ;; 1 cycle
        MOV       C, N:_ls_rtc_fill  ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        SUBW      AX, BC             ;; 1 cycle
        MOV       C, N:_ls_rev_fill  ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        ADDW      AX, BC             ;; 1 cycle
        MOVW      [SP+0x06], AX      ;; 1 cycle
//  366     ls_cnt_end_1day = (bcd_to_decimal(Now.hour)*60+bcd_to_decimal(Now.min))/lsip_period;
        MOV       A, N:_Now+2        ;; 1 cycle
          CFI FunCall _bcd_to_decimal
        CALL      _bcd_to_decimal    ;; 3 cycles
        MOV       [SP+0x04], A       ;; 1 cycle
        MOV       A, N:_Now+1        ;; 1 cycle
          CFI FunCall _bcd_to_decimal
        CALL      _bcd_to_decimal    ;; 3 cycles
        MOV       E, A               ;; 1 cycle
        MOV       C, N:_lsip_period  ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+16
        POP       HL                 ;; 1 cycle
          CFI CFA SP+14
        MOV       A, [SP+0x04]       ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x3C          ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOV       D, #0x0            ;; 1 cycle
        ADDW      AX, DE             ;; 1 cycle
        PUSH      HL                 ;; 1 cycle
          CFI CFA SP+16
        POP       BC                 ;; 1 cycle
          CFI CFA SP+14
          CFI FunCall ?SI_DIV_L02
        CALL      N:?SI_DIV_L02      ;; 3 cycles
        MOVW      [SP+0x08], AX      ;; 1 cycle
        ; ------------------------------------- Block: 66 cycles
//  367   }
//  368   
//  369   load_survey_cnt=load_survey_cnt+1-ls_fg_f;
??load_survey_ram_init_18:
        MOVW      AX, N:_load_survey_cnt  ;; 1 cycle
        INCW      AX                 ;; 1 cycle
        MOV       C, N:_ls_fg_f      ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        SUBW      AX, BC             ;; 1 cycle
        MOVW      N:_load_survey_cnt, AX  ;; 1 cycle
//  370   
//  371   if(ls_fg_f==0)
        CMP0      N:_ls_fg_f         ;; 1 cycle
        BNZ       ??load_survey_ram_init_19  ;; 4 cycles
        ; ------------------------------------- Block: 11 cycles
//  372   {
//  373     temp_mdi = bcd_to_decimal(TempTime.hour)*60+(bcd_to_decimal(TempTime.min)+lsip_period*(1-ls_rtc_fill));
        MOV       A, N:_TempTime+2   ;; 1 cycle
          CFI FunCall _bcd_to_decimal
        CALL      _bcd_to_decimal    ;; 3 cycles
        MOV       [SP+0x04], A       ;; 1 cycle
        MOV       A, N:_TempTime+1   ;; 1 cycle
          CFI FunCall _bcd_to_decimal
        CALL      _bcd_to_decimal    ;; 3 cycles
        MOV       E, A               ;; 1 cycle
        MOV       C, N:_ls_rtc_fill  ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOVW      AX, #0x1           ;; 1 cycle
        SUBW      AX, BC             ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOV       X, N:_lsip_period  ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, [SP+0x04]       ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x3C          ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOV       D, #0x0            ;; 1 cycle
        ADDW      AX, DE             ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        MOVW      [SP+0x02], AX      ;; 1 cycle
        ; ------------------------------------- Block: 30 cycles
//  374   }
//  375   
//  376   if(temp_mdi >= 1440)
??load_survey_ram_init_19:
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        CMPW      AX, #0x5A0         ;; 1 cycle
        BC        ??load_survey_ram_init_20  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  377   {
//  378     temp_mdi=0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [SP+0x02], AX      ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
//  379   }
//  380   
//  381   if(1==ls_fg_f)
??load_survey_ram_init_20:
        CMP       N:_ls_fg_f, #0x1   ;; 1 cycle
        BNZ       ??load_survey_ram_init_21  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  382   {
//  383     temp_mdi = lsip_period;
        MOV       X, N:_lsip_period  ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      [SP+0x02], AX      ;; 1 cycle
        ; ------------------------------------- Block: 3 cycles
//  384   }
//  385   
//  386   fill_oprzero(COMPART_LS_SIZE);
??load_survey_ram_init_21:
        MOV       A, #0x20           ;; 1 cycle
          CFI FunCall _fill_oprzero
        CALL      _fill_oprzero      ;; 3 cycles
//  387   for(n = ls_cnt_start_1day; n <= ls_cnt_end_1day; n++)
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        MOVW      [SP+0x04], AX      ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
??ls_miss_fill_0:
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [SP+0x08]      ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        SKNC                         ;; 4 cycles
        BR        N:??load_survey_ram_init_22  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
//  388   {
//  389     u8proceed=1;    
        MOV       A, #0x1            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  390     if(load_survey_cnt >= max_load_survey_cnt)
        MOVW      HL, N:_max_load_survey_cnt  ;; 1 cycle
        MOVW      AX, N:_load_survey_cnt  ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        SKC                          ;; 1 cycle
          CFI FunCall _ls_rollover
        ; ------------------------------------- Block: 6 cycles
//  391     {
//  392       ls_rollover();
        CALL      _ls_rollover       ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
//  393     }
//  394     
//  395     opr_data[0] = decimal_to_bcd(temp_mdi/60); /* main11; */
??ls_miss_fill_1:
        MOVW      DE, #0x3C          ;; 1 cycle
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        DIVHU                        ;; 9 cycles
        NOP                          ;; 1 cycle
        MOV       A, X               ;; 1 cycle
          CFI FunCall _decimal_to_bcd
        CALL      _decimal_to_bcd    ;; 3 cycles
        MOV       N:_opr_data, A     ;; 1 cycle
//  396     opr_data[1] = decimal_to_bcd(temp_mdi%60); /* main11; */
        MOVW      DE, #0x3C          ;; 1 cycle
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        DIVHU                        ;; 9 cycles
        NOP                          ;; 1 cycle
        MOV       A, E               ;; 1 cycle
          CFI FunCall _decimal_to_bcd
        CALL      _decimal_to_bcd    ;; 3 cycles
        MOV       N:_opr_data+1, A   ;; 1 cycle
//  397     //opr_data[1] = hr_with_dst(opr_data[1]);
//  398     temp_mdi += lsip_period;
        MOV       C, N:_lsip_period  ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        ADDW      AX, BC             ;; 1 cycle
        MOVW      [SP+0x02], AX      ;; 1 cycle
//  399     if(temp_mdi>=1440)
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        CMPW      AX, #0x5A0         ;; 1 cycle
        BC        ??load_survey_ram_init_23  ;; 4 cycles
        ; ------------------------------------- Block: 45 cycles
//  400     {
//  401       temp_mdi=0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [SP+0x02], AX      ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
//  402     }
//  403     
//  404     if(n>=(24*mdi_sel_ls) || ls_fg_f==1)
??load_survey_ram_init_23:
        MOV       X, N:_mdi_sel_ls   ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x18          ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BNC       ??load_survey_ram_init_24  ;; 4 cycles
        ; ------------------------------------- Block: 12 cycles
        CMP       N:_ls_fg_f, #0x1   ;; 1 cycle
        BNZ       ??load_survey_ram_init_25  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  405     {
//  406       if(n==24*mdi_sel_ls)
??load_survey_ram_init_24:
        MOV       X, N:_mdi_sel_ls   ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x18          ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BNZ       ??load_survey_ram_init_26  ;; 4 cycles
        ; ------------------------------------- Block: 12 cycles
//  407       {
//  408         if((TempTime.hour==0) && (TempTime.min==0))
        CMP0      N:_TempTime+2      ;; 1 cycle
        BNZ       ??load_survey_ram_init_27  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        CMP0      N:_TempTime+1      ;; 1 cycle
        BNZ       ??load_survey_ram_init_27  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  409         {
//  410           opr_data[2]=TempTime.day; 
        MOV       A, N:_TempTime+3   ;; 1 cycle
        MOV       N:_opr_data+2, A   ;; 1 cycle
//  411           opr_data[3]=TempTime.month;
        MOV       A, N:_TempTime+5   ;; 1 cycle
        MOV       N:_opr_data+3, A   ;; 1 cycle
//  412           opr_data[4]=TempTime.year;
        MOV       A, N:_TempTime+6   ;; 1 cycle
        MOV       N:_opr_data+4, A   ;; 1 cycle
        BR        S:??load_survey_ram_init_28  ;; 3 cycles
        ; ------------------------------------- Block: 9 cycles
//  413         }
//  414         else
//  415         {
//  416           next_miss_date(TempTime.day, TempTime.month, TempTime.year);
??load_survey_ram_init_27:
        MOV       C, N:_TempTime+6   ;; 1 cycle
        MOV       X, N:_TempTime+5   ;; 1 cycle
        MOV       A, N:_TempTime+3   ;; 1 cycle
          CFI FunCall _next_miss_date
        CALL      _next_miss_date    ;; 3 cycles
//  417           opr_data[2]=ls_date;
        MOV       A, N:_ls_date      ;; 1 cycle
        MOV       N:_opr_data+2, A   ;; 1 cycle
//  418           opr_data[3]=ls_month;
        MOV       A, N:_ls_month     ;; 1 cycle
        MOV       N:_opr_data+3, A   ;; 1 cycle
//  419           opr_data[4]=ls_year;
        MOV       A, N:_ls_year      ;; 1 cycle
        MOV       N:_opr_data+4, A   ;; 1 cycle
        BR        S:??load_survey_ram_init_28  ;; 3 cycles
        ; ------------------------------------- Block: 15 cycles
//  420         }
//  421       }
//  422       else
//  423       {
//  424         opr_data[2]=Now.day;
??load_survey_ram_init_26:
        MOV       A, N:_Now+3        ;; 1 cycle
        MOV       N:_opr_data+2, A   ;; 1 cycle
//  425         opr_data[3]=Now.month;
        MOV       A, N:_Now+5        ;; 1 cycle
        MOV       N:_opr_data+3, A   ;; 1 cycle
//  426         opr_data[4]=Now.year;
        MOV       A, N:_Now+6        ;; 1 cycle
        MOV       N:_opr_data+4, A   ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
//  427       }
//  428       
//  429       if(ls_rtc_fill==1)
??load_survey_ram_init_28:
        CMP       N:_ls_rtc_fill, #0x1  ;; 1 cycle
        BNZ       ??load_survey_ram_init_29  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  430       {
//  431         midnight_fill_rtc_f=1;
        MOV       N:_midnight_fill_rtc_f, #0x1  ;; 1 cycle
        BR        S:??load_survey_ram_init_29  ;; 3 cycles
        ; ------------------------------------- Block: 4 cycles
//  432       }
//  433     }
//  434     else
//  435     {
//  436       opr_data[2]=TempTime.day;
??load_survey_ram_init_25:
        MOV       A, N:_TempTime+3   ;; 1 cycle
        MOV       N:_opr_data+2, A   ;; 1 cycle
//  437       opr_data[3]=TempTime.month;
        MOV       A, N:_TempTime+5   ;; 1 cycle
        MOV       N:_opr_data+3, A   ;; 1 cycle
//  438       opr_data[4]=TempTime.year;
        MOV       A, N:_TempTime+6   ;; 1 cycle
        MOV       N:_opr_data+4, A   ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
//  439     }
//  440     //    temp_hr=opr_data[0];
//  441     //    temp_min=opr_data[1];
//  442     //    temp_date=opr_data[2];
//  443     //    temp_month=opr_data[3];
//  444     //    temp_year=opr_data[4];
//  445     
//  446     
//  447     seq_no_ls++;
??load_survey_ram_init_29:
        MOVW      BC, N:_seq_no_ls+2  ;; 1 cycle
        MOVW      AX, N:_seq_no_ls   ;; 1 cycle
        ADDW      AX, #0x1           ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_seq_no_ls, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_seq_no_ls+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  448     long_into_char_array3(seq_no_ls,&opr_data[26]);
        MOVW      DE, #LWRD(_opr_data+26)  ;; 1 cycle
        MOVW      BC, N:_seq_no_ls+2  ;; 1 cycle
        MOVW      AX, N:_seq_no_ls   ;; 1 cycle
          CFI FunCall _long_into_char_array3
        CALL      _long_into_char_array3  ;; 3 cycles
//  449     
//  450     ls_status |= bit5;
        SET1      N:_ls_status.5     ;; 2 cycles
//  451     
//  452     u8proceed=1;
        MOV       A, #0x1            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  453     if(load_survey_cnt<COMPART_LS_ENTRIES) /* eeprom 0 */
        MOVW      AX, N:_load_survey_cnt  ;; 1 cycle
        CMPW      AX, #0xE40         ;; 1 cycle
        BNC       ??load_survey_ram_init_30  ;; 4 cycles
        ; ------------------------------------- Block: 31 cycles
//  454     {
//  455       temp_us32 = ((uint32_t)COMPART_LS_START_ADD+((us32)load_survey_cnt*COMPART_LS_SIZE));   
        MOVW      AX, N:_load_survey_cnt  ;; 1 cycle
        MOVW      BC, #0x20          ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #0xCB00        ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_temp_us32, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_temp_us32+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  456       temp_us16=temp_us32%COMPART_LS_MEM_BYTES;
        MOVW      BC, S:_temp_us32+2  ;; 1 cycle
        MOVW      AX, S:_temp_us32   ;; 1 cycle
        MOVW      S:_temp_us16, AX   ;; 1 cycle
//  457       ls_eep_id=temp_us32/COMPART_LS_MEM_BYTES;
        MOVW      BC, S:_temp_us32+2  ;; 1 cycle
        MOVW      AX, S:_temp_us32   ;; 1 cycle
        CLRW      AX                 ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       [SP+0x01], A       ;; 1 cycle
        BR        S:??load_survey_ram_init_31  ;; 3 cycles
        ; ------------------------------------- Block: 29 cycles
//  458     }
//  459     else
//  460     {
//  461       u8proceed = 0;
??load_survey_ram_init_30:
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
//  462     }
//  463     
//  464     
//  465     if(1==u8proceed)
??load_survey_ram_init_31:
        MOV       A, [SP]            ;; 1 cycle
        CMP       A, #0x1            ;; 1 cycle
        BNZ       ??load_survey_ram_init_32  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  466     {
//  467       u8proceed=0;
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  468       eprom_write(temp_us16,ls_eep_id,COMPART_LS_SIZE,PAGE_2,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+16
        MOV       B, #0x1            ;; 1 cycle
        MOVW      DE, #0x20          ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        MOV       C, A               ;; 1 cycle
        MOVW      AX, S:_temp_us16   ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  469       wdt_restart();
          CFI FunCall _R_WDT_Restart
        CALL      _R_WDT_Restart     ;; 3 cycles
//  470       load_survey_cnt++;
        INCW      N:_load_survey_cnt  ;; 2 cycles
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+14
        ; ------------------------------------- Block: 18 cycles
//  471     }
//  472   }
??load_survey_ram_init_32:
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        INCW      AX                 ;; 1 cycle
        MOVW      [SP+0x04], AX      ;; 1 cycle
        BR        N:??ls_miss_fill_0  ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
//  473   
//  474   if(n!=ls_cnt_start_1day)
??load_survey_ram_init_22:
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        SKZ                          ;; 1 cycle
        ; ------------------------------------- Block: 5 cycles
//  475   {
//  476     load_survey_cnt--;
        DECW      N:_load_survey_cnt  ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
//  477   }
//  478   
//  479   ls_fg_f=0;
??ls_miss_fill_2:
        MOV       N:_ls_fg_f, #0x0   ;; 1 cycle
//  480   
//  481   Eprom_ReadWM(0x0770,0,16);
        MOV       B, #0x10           ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x770         ;; 1 cycle
          CFI FunCall _Eprom_ReadWM
        CALL      _Eprom_ReadWM      ;; 3 cycles
//  482   long_into_char_array3(seq_no_ls,&opr_data[0]);
        MOVW      DE, #LWRD(_opr_data)  ;; 1 cycle
        MOVW      BC, N:_seq_no_ls+2  ;; 1 cycle
        MOVW      AX, N:_seq_no_ls   ;; 1 cycle
          CFI FunCall _long_into_char_array3
        CALL      _long_into_char_array3  ;; 3 cycles
//  483   Eprom_WriteWM(0x0770,0,16);
        MOV       B, #0x10           ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x770         ;; 1 cycle
          CFI FunCall _Eprom_WriteWM
        CALL      _Eprom_WriteWM     ;; 3 cycles
//  484 }
        ADDW      SP, #0xA           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock7
        ; ------------------------------------- Block: 26 cycles
        ; ------------------------------------- Total: 518 cycles
//  485 
//  486 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock8 Using cfiCommon0
          CFI Function _ls_cnt_at_pwrup
          CFI FunCall _load_ls_cnt
        CODE
//  487 void ls_cnt_at_pwrup(void)
//  488 {
_ls_cnt_at_pwrup:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  489   //  main13=Now.hour;
//  490   //  hex_to_bin();
//  491   //  main1=main11*mdi_sel_ls;
//  492   //  
//  493   //  main13=Now.min;
//  494   //  hex_to_bin();
//  495   //  main2=main11/(60/mdi_sel_ls);
//  496   //  
//  497   //  load_survey_cnt=(24*mdi_sel_ls*(day_counter))+main1+main2-1; /* +miss_ls_date_f; */
//  498   
//  499   load_ls_cnt();
        CALL      _load_ls_cnt       ;; 3 cycles
//  500   
//  501   if(load_survey_cnt >= max_load_survey_cnt)
        MOVW      HL, N:_max_load_survey_cnt  ;; 1 cycle
        MOVW      AX, N:_load_survey_cnt  ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BC        ??load_survey_ram_init_33  ;; 4 cycles
          CFI FunCall _ls_rollover
        ; ------------------------------------- Block: 10 cycles
//  502   {
//  503     ls_rollover();
        CALL      _ls_rollover       ;; 3 cycles
        RET                          ;; 6 cycles
        ; ------------------------------------- Block: 9 cycles
//  504   }
//  505   else if(flag_rtc_change_day == 1)
??load_survey_ram_init_33:
        MOVW      HL, #LWRD(_flag_rtc_change)  ;; 1 cycle
        MOV1      CY, [HL].3         ;; 1 cycle
        SKNC                         ;; 1 cycle
          CFI FunCall _ls_day_counter
        ; ------------------------------------- Block: 3 cycles
//  506   {
//  507     ls_day_counter();
        CALL      _ls_day_counter    ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
//  508   }
//  509 }
??ls_cnt_at_pwrup_0:
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock8
        ; ------------------------------------- Block: 6 cycles
        ; ------------------------------------- Total: 31 cycles
//  510 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock9 Using cfiCommon0
          CFI Function _load_ls_cnt
        CODE
//  511 void load_ls_cnt()
//  512 {
_load_ls_cnt:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  513   temp_us8 = bcd_to_decimal(Now.hour) * mdi_sel_ls;
        MOV       A, N:_Now+2        ;; 1 cycle
          CFI FunCall _bcd_to_decimal
        CALL      _bcd_to_decimal    ;; 3 cycles
        MOV       X, A               ;; 1 cycle
        MOV       A, N:_mdi_sel_ls   ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       S:_temp_us8, A     ;; 1 cycle
        XCH       A, X               ;; 1 cycle
//  514   temp_us8 += (bcd_to_decimal(Now.min) / lsip_period);
        MOV       A, N:_Now+1        ;; 1 cycle
          CFI FunCall _bcd_to_decimal
        CALL      _bcd_to_decimal    ;; 3 cycles
        MOV       B, A               ;; 1 cycle
        MOV       X, N:_lsip_period  ;; 1 cycle
        MOV       A, B               ;; 1 cycle
          CFI FunCall ?UC_DIV_L01
        CALL      N:?UC_DIV_L01      ;; 3 cycles
        MOV       B, A               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        ADD       A, S:_temp_us8     ;; 1 cycle
        MOV       S:_temp_us8, A     ;; 1 cycle
        XCH       A, B               ;; 1 cycle
//  515   load_survey_cnt = (24*mdi_sel_ls*(day_counter)) + temp_us8 - 1;
        MOVW      BC, N:_day_counter  ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+6
        POP       HL                 ;; 1 cycle
          CFI CFA SP+4
        MOV       X, N:_mdi_sel_ls   ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x18          ;; 1 cycle
        MULHU                        ;; 2 cycles
        PUSH      HL                 ;; 1 cycle
          CFI CFA SP+6
        POP       BC                 ;; 1 cycle
          CFI CFA SP+4
        MULHU                        ;; 2 cycles
        MOV       C, S:_temp_us8     ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        ADDW      AX, BC             ;; 1 cycle
        DECW      AX                 ;; 1 cycle
        MOVW      N:_load_survey_cnt, AX  ;; 1 cycle
//  516 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock9
        ; ------------------------------------- Block: 50 cycles
        ; ------------------------------------- Total: 50 cycles
//  517 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock10 Using cfiCommon0
          CFI Function _ls_rollover
        CODE
//  518 void ls_rollover()
//  519 {
_ls_rollover:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  520   max_day_counter=day_counter+miss_ls_date_f;
        MOVW      AX, N:_day_counter  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        ADD       A, N:_miss_ls_date_f  ;; 1 cycle
        MOV       N:_max_day_counter, A  ;; 1 cycle
//  521   day_counter=0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_day_counter, AX  ;; 1 cycle
//  522   /*        day_no_inc_f=1; */
//  523   /*		    temp_lsro_flag=0; */
//  524   /*		    ls_cnt_at_ro=load_survey_cnt; */
//  525   load_survey_cnt=0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_load_survey_cnt, AX  ;; 1 cycle
//  526   
//  527   fill_oprzero(16);
        MOV       A, #0x10           ;; 1 cycle
          CFI FunCall _fill_oprzero
        CALL      _fill_oprzero      ;; 3 cycles
//  528   
//  529   /*      if(load_survey_cnt1!=0 || day_counter1!=0)
//  530   {
//  531   ls_change_md_ip=1;
//  532 } 
//  533   */
//  534   
//  535   lsro_flag=1;
        MOV       N:_lsro_flag, #0x1  ;; 1 cycle
//  536   opr_data[0]=lsro_flag;
        MOV       A, N:_lsro_flag    ;; 1 cycle
        MOV       N:_opr_data, A     ;; 1 cycle
//  537   opr_data[1]=0; /* ls_change_md_ip; */
        MOV       N:_opr_data+1, #0x0  ;; 1 cycle
//  538   opr_data[3]=0; /* temp_lsro_flag; */
        MOV       N:_opr_data+3, #0x0  ;; 1 cycle
//  539   Eprom_WriteWM(0x0780,0,16);
        MOV       B, #0x10           ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x780         ;; 1 cycle
          CFI FunCall _Eprom_WriteWM
        CALL      _Eprom_WriteWM     ;; 3 cycles
//  540   ls_day_counter();
          CFI FunCall _ls_day_counter
        CALL      _ls_day_counter    ;; 3 cycles
//  541   
//  542 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock10
        ; ------------------------------------- Block: 32 cycles
        ; ------------------------------------- Total: 32 cycles
//  543 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock11 Using cfiCommon1
          CFI Function _load_survey_next_interval_timestamp
        CODE
//  544 rtc_counter_value_t load_survey_next_interval_timestamp(us8 mdi_period1, rtc_counter_value_t Time)
//  545 {
_load_survey_next_interval_timestamp:
        ; * Stack frame (at entry) *
        ; Param size: 8
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+6
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+8
        ; Auto size: 6
        SUBW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+10
//  546   us8 temp_time;
//  547   
//  548   if(ls_rtc_fill==1)
        CMP       N:_ls_rtc_fill, #0x1  ;; 1 cycle
        BNZ       ??load_survey_ram_init_34  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
//  549   {
//  550     temp4_var = Time.hour;
        MOV       A, [SP+0x0C]       ;; 1 cycle
        MOV       N:_temp4_var, A    ;; 1 cycle
//  551     temp5_var = Time.min;
        MOV       A, [SP+0x0B]       ;; 1 cycle
        MOV       N:_temp5_var, A    ;; 1 cycle
        ; ------------------------------------- Block: 4 cycles
//  552   }
//  553   
//  554   temp_time = bcd_to_decimal(Time.min) + mdi_period1;
??load_survey_ram_init_34:
        MOV       A, [SP+0x0B]       ;; 1 cycle
          CFI FunCall _bcd_to_decimal
        CALL      _bcd_to_decimal    ;; 3 cycles
        MOV       X, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        ADD       A, X               ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  555   /* if not the last integration time in 60 min slot( no need to increase hr).No need to change hr */
//  556   if(temp_time<60)                      
        MOV       A, [SP]            ;; 1 cycle
        CMP       A, #0x3C           ;; 1 cycle
        BNC       ??load_survey_ram_init_35  ;; 4 cycles
        ; ------------------------------------- Block: 14 cycles
//  557   {
//  558     /* getting the next mdi period directly */
//  559     temp_time = temp_time - (temp_time % mdi_period1); 
        MOV       A, [SP+0x02]       ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
          CFI FunCall ?UC_MOD_L01
        CALL      N:?UC_MOD_L01      ;; 3 cycles
        MOV       B, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SUB       A, B               ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  560     Time.min = decimal_to_bcd(temp_time);
        MOV       A, [SP]            ;; 1 cycle
          CFI FunCall _decimal_to_bcd
        CALL      _decimal_to_bcd    ;; 3 cycles
        MOV       [SP+0x0B], A       ;; 1 cycle
        BR        S:??load_survey_ram_init_36  ;; 3 cycles
        ; ------------------------------------- Block: 18 cycles
//  561   }
//  562   else
//  563   {
//  564     /* increasing the hr and setting min to zero */
//  565     /* taking care of time change effects due to increase in hr */
//  566     Time.min = 0;
??load_survey_ram_init_35:
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP+0x0B], A       ;; 1 cycle
//  567     Time = time_inc_hour(Time);
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0xA           ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+18
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x12          ;; 1 cycle
          CFI FunCall _time_inc_hour
        CALL      _time_inc_hour     ;; 3 cycles
        ADDW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+10
        ; ------------------------------------- Block: 17 cycles
//  568   }
//  569   Time.sec = 0x00;
??load_survey_ram_init_36:
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP+0x0A], A       ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0xA           ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOV       A, [DE]            ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
        MOV       A, [DE+0x01]       ;; 1 cycle
        MOV       [HL+0x01], A       ;; 1 cycle
        MOV       A, [DE+0x02]       ;; 1 cycle
        MOV       [HL+0x02], A       ;; 1 cycle
        MOV       A, [DE+0x03]       ;; 1 cycle
        MOV       [HL+0x03], A       ;; 1 cycle
        MOV       A, [DE+0x04]       ;; 1 cycle
        MOV       [HL+0x04], A       ;; 1 cycle
        MOV       A, [DE+0x05]       ;; 1 cycle
        MOV       [HL+0x05], A       ;; 1 cycle
        MOV       A, [DE+0x06]       ;; 1 cycle
        MOV       [HL+0x06], A       ;; 1 cycle
        MOVW      AX, HL             ;; 1 cycle
        ADDW      SP, #0x6           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock11
        ; ------------------------------------- Block: 29 cycles
        ; ------------------------------------- Total: 90 cycles
//  570   return Time;
//  571 }
//  572 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock12 Using cfiCommon0
          CFI Function _ls_day_counter
        CODE
//  573 void ls_day_counter()
//  574 {
_ls_day_counter:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 4
        SUBW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
//  575   us16 temp_address = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [SP+0x02], AX      ;; 1 cycle
//  576   
//  577   opr_data[0] = day_counter;
        MOVW      AX, N:_day_counter  ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       N:_opr_data, A     ;; 1 cycle
        XCH       A, X               ;; 1 cycle
//  578   if(day_counter >= max_day_counter)
        MOV       X, N:_max_day_counter  ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      HL, N:_day_counter  ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BH        ??load_survey_ram_init_37  ;; 4 cycles
        ; ------------------------------------- Block: 15 cycles
//  579   {
//  580     if(max_day_counter==MAX_LS/(24*mdi_sel_ls)) 
        MOV       X, N:_mdi_sel_ls   ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x18          ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, #0xE40         ;; 1 cycle
          CFI FunCall ?SI_DIV_L02
        CALL      N:?SI_DIV_L02      ;; 3 cycles
        MOV       C, N:_max_day_counter  ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        CMPW      AX, BC             ;; 1 cycle
        BNZ       ??load_survey_ram_init_37  ;; 4 cycles
        ; ------------------------------------- Block: 17 cycles
//  581     {
//  582       max_day_counter= day_counter + 1;
        MOVW      AX, N:_day_counter  ;; 1 cycle
        INC       X                  ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       N:_max_day_counter, A  ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        ; ------------------------------------- Block: 5 cycles
//  583     }
//  584   }
//  585   opr_data[1] = max_day_counter;
??load_survey_ram_init_37:
        MOV       A, N:_max_day_counter  ;; 1 cycle
        MOV       N:_opr_data+1, A   ;; 1 cycle
//  586   opr_data[2] = 0; /* ls_cnt_at_ro/256; */
        MOV       N:_opr_data+2, #0x0  ;; 1 cycle
//  587   opr_data[3] = 0; /* ls_cnt_at_ro%256; */
        MOV       N:_opr_data+3, #0x0  ;; 1 cycle
//  588   Eprom_WriteWM(0x0790,0,16); /* day_counter */
        MOV       B, #0x10           ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x790         ;; 1 cycle
          CFI FunCall _Eprom_WriteWM
        CALL      _Eprom_WriteWM     ;; 3 cycles
//  589   
//  590   d_array[day_counter] = sel_datediff(Now.day,Now.month,Now.year);
        MOV       C, N:_Now+6        ;; 1 cycle
        MOV       X, N:_Now+5        ;; 1 cycle
        MOV       A, N:_Now+3        ;; 1 cycle
          CFI FunCall _sel_datediff
        CALL      _sel_datediff      ;; 3 cycles
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, N:_day_counter  ;; 1 cycle
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(_d_array)  ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      [DE], AX           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
//  591   
//  592   if(max_day_counter>MAX_LS/(24*mdi_sel_ls)&&lsro_flag==1)
        MOV       C, N:_max_day_counter  ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+10
        POP       HL                 ;; 1 cycle
          CFI CFA SP+8
        MOV       X, N:_mdi_sel_ls   ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x18          ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, #0xE40         ;; 1 cycle
          CFI FunCall ?SI_DIV_L02
        CALL      N:?SI_DIV_L02      ;; 3 cycles
        PUSH      HL                 ;; 1 cycle
          CFI CFA SP+10
        POP       BC                 ;; 1 cycle
          CFI CFA SP+8
          CFI FunCall ?SI_CMP_L02
        CALL      N:?SI_CMP_L02      ;; 3 cycles
        BNC       ??load_survey_ram_init_38  ;; 4 cycles
        ; ------------------------------------- Block: 49 cycles
        CMP       N:_lsro_flag, #0x1  ;; 1 cycle
        BNZ       ??load_survey_ram_init_38  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  593   {
//  594     d_array[max_day_counter-1]=d_array[0];
        MOV       X, N:_max_day_counter  ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(_d_array-2)  ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, N:_d_array     ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        ; ------------------------------------- Block: 9 cycles
//  595   }
//  596   
//  597   temp_address=0x0D00;
??load_survey_ram_init_38:
        MOVW      AX, #0xD00         ;; 1 cycle
        MOVW      [SP+0x02], AX      ;; 1 cycle
//  598   for(us8 index=0; index<26; index++)
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP+0x01], A       ;; 1 cycle
        ; ------------------------------------- Block: 4 cycles
??ls_day_counter_0:
        MOV       A, [SP+0x01]       ;; 1 cycle
        CMP       A, #0x1A           ;; 1 cycle
        BNC       ??load_survey_ram_init_39  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  599   {
//  600     for(us8 temp_day_counter=0; temp_day_counter<7; temp_day_counter++)
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
??ls_day_counter_1:
        MOV       A, [SP]            ;; 1 cycle
        CMP       A, #0x7            ;; 1 cycle
        BNC       ??load_survey_ram_init_40  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  601     {
//  602       int_into_char_array(d_array[temp_day_counter+7*index], &opr_data[temp_day_counter*2]);
        MOV       A, [SP]            ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(_opr_data)  ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+10
        POP       HL                 ;; 1 cycle
          CFI CFA SP+8
        MOV       A, [SP+0x01]       ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x7           ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      DE, AX             ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        ADDW      AX, DE             ;; 1 cycle
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(_d_array)  ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        PUSH      HL                 ;; 1 cycle
          CFI CFA SP+10
        POP       BC                 ;; 1 cycle
          CFI CFA SP+8
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  603     }
        MOV       A, [SP]            ;; 1 cycle
        INC       A                  ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        BR        S:??ls_day_counter_1  ;; 3 cycles
        ; ------------------------------------- Block: 38 cycles
//  604     Eprom_WriteWM(temp_address,0,16);
??load_survey_ram_init_40:
        MOV       B, #0x10           ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, [SP+0x02]      ;; 1 cycle
          CFI FunCall _Eprom_WriteWM
        CALL      _Eprom_WriteWM     ;; 3 cycles
//  605     temp_address+=0x10;
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
        MOVW      [SP+0x02], AX      ;; 1 cycle
//  606   }
        MOV       A, [SP+0x01]       ;; 1 cycle
        INC       A                  ;; 1 cycle
        MOV       [SP+0x01], A       ;; 1 cycle
        BR        S:??ls_day_counter_0  ;; 3 cycles
        ; ------------------------------------- Block: 15 cycles
//  607   
//  608 }
??load_survey_ram_init_39:
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock12
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 178 cycles
//  609 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock13 Using cfiCommon0
          CFI Function _reset_load_survey
        CODE
//  610 void reset_load_survey(void)
//  611 {
_reset_load_survey:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  612   fill_oprzero(48);
        MOV       A, #0x30           ;; 1 cycle
          CFI FunCall _fill_oprzero
        CALL      _fill_oprzero      ;; 3 cycles
//  613   
//  614   eprom_write(0x0700,0,48,PAGE_3,AUTO_CALC);                  /* LS Energy */
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+6
        MOV       B, #0x2            ;; 1 cycle
        MOVW      DE, #0x30          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x700         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  615   load_survey_reset_avg_acc();
          CFI FunCall _load_survey_reset_avg_acc
        CALL      _load_survey_reset_avg_acc  ;; 3 cycles
//  616   eprom_write(0x0780,0,16,PAGE_1,AUTO_CALC);                  /* LS count and Flag */
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+8
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x780         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  617   eprom_write(0x0770,0,16,PAGE_1,AUTO_CALC);                  /* LS count and Flag */
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x770         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  618   
//  619   load_survey_cnt=0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_load_survey_cnt, AX  ;; 1 cycle
//  620   
//  621   day_counter=0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_day_counter, AX  ;; 1 cycle
//  622   max_load_survey_cnt = MAX_LS;
        MOVW      AX, #0xE40         ;; 1 cycle
        MOVW      N:_max_load_survey_cnt, AX  ;; 1 cycle
//  623   max_day_counter = max_load_survey_cnt/(24*mdi_sel_ls);
        MOV       X, N:_mdi_sel_ls   ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x18          ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, N:_max_load_survey_cnt  ;; 1 cycle
        DIVHU                        ;; 9 cycles
        NOP                          ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       N:_max_day_counter, A  ;; 1 cycle
        XCH       A, X               ;; 1 cycle
//  624   if(max_day_counter > D_ARRAY_SIZE)
        MOV       A, N:_max_day_counter  ;; 1 cycle
        ADDW      SP, #0x6           ;; 1 cycle
          CFI CFA SP+4
        CMP       A, #0xB7           ;; 1 cycle
        SKC                          ;; 1 cycle
        ; ------------------------------------- Block: 64 cycles
//  625   {
//  626     max_day_counter = D_ARRAY_SIZE;
        MOV       N:_max_day_counter, #0xB6  ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
//  627   }
//  628   opr_data[0] = day_counter;
??reset_load_survey_0:
        MOVW      AX, N:_day_counter  ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       N:_opr_data, A     ;; 1 cycle
        XCH       A, X               ;; 1 cycle
//  629   opr_data[1] = max_day_counter;
        MOV       A, N:_max_day_counter  ;; 1 cycle
        MOV       N:_opr_data+1, A   ;; 1 cycle
//  630   eprom_write(0x0790,0,16,PAGE_1,AUTO_CALC); 
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+6
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x790         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  631   
//  632   
//  633   lsro_flag=0;                /* ls_change_md_ip=0; ls_cnt_at_ro=0; */
        MOV       N:_lsro_flag, #0x0  ;; 1 cycle
//  634   fill_oprzero(64);
        MOV       A, #0x40           ;; 1 cycle
          CFI FunCall _fill_oprzero
        CALL      _fill_oprzero      ;; 3 cycles
//  635   eprom_write(0x0D00,0,416,PAGE_1,AUTO_CALC);                         /* clearing D array */
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+8
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x1A0         ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xD00         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  636   
//  637   d_array[day_counter]=sel_datediff(Now.day,Now.month,Now.year);
        MOV       C, N:_Now+6        ;; 1 cycle
        MOV       X, N:_Now+5        ;; 1 cycle
        MOV       A, N:_Now+3        ;; 1 cycle
          CFI FunCall _sel_datediff
        CALL      _sel_datediff      ;; 3 cycles
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, N:_day_counter  ;; 1 cycle
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(_d_array)  ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      [DE], AX           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
//  638   int_into_char_array(d_array[0],&opr_data[0]);
        MOVW      BC, #LWRD(_opr_data)  ;; 1 cycle
        MOVW      AX, N:_d_array     ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  639   eprom_write(0x0D00,0,16,PAGE_1,AUTO_CALC);                                           /* Saving current day counter */
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xD00         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  640 }
        ADDW      SP, #0x6           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock13
        ; ------------------------------------- Block: 66 cycles
        ; ------------------------------------- Total: 131 cycles
//  641 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock14 Using cfiCommon1
          CFI Function _next_miss_date
        CODE
//  642 void next_miss_date(uint8 date, uint8 month, uint8 year)
//  643 {
_next_miss_date:
        ; * Stack frame (at entry) *
        ; Param size: 0
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+6
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+8
        ; Auto size: 4
//  644   ls_date=(bcd_to_decimal(date)+1);
        MOV       A, [SP+0x03]       ;; 1 cycle
          CFI FunCall _bcd_to_decimal
        CALL      _bcd_to_decimal    ;; 3 cycles
        INC       A                  ;; 1 cycle
        MOV       N:_ls_date, A      ;; 1 cycle
//  645   ls_month=bcd_to_decimal(month);
        MOV       A, [SP+0x02]       ;; 1 cycle
          CFI FunCall _bcd_to_decimal
        CALL      _bcd_to_decimal    ;; 3 cycles
        MOV       N:_ls_month, A     ;; 1 cycle
//  646   ls_year=bcd_to_decimal(year);
        MOV       A, [SP]            ;; 1 cycle
          CFI FunCall _bcd_to_decimal
        CALL      _bcd_to_decimal    ;; 3 cycles
        MOV       N:_ls_year, A      ;; 1 cycle
//  647   if((ls_month==2 && ls_year%4==0 && ls_date>29)||((ls_date>(monthdays[ls_month-1]))&&(ls_month!=2 || ls_year%4!=0)))
        CMP       N:_ls_month, #0x2  ;; 1 cycle
        BNZ       ??load_survey_ram_init_41  ;; 4 cycles
        ; ------------------------------------- Block: 23 cycles
        MOV       X, #0x4            ;; 1 cycle
        MOV       A, N:_ls_year      ;; 1 cycle
          CFI FunCall ?UC_MOD_L01
        CALL      N:?UC_MOD_L01      ;; 3 cycles
        MOV       B, A               ;; 1 cycle
        CMP0      B                  ;; 1 cycle
        BNZ       ??load_survey_ram_init_41  ;; 4 cycles
        ; ------------------------------------- Block: 11 cycles
        MOV       A, N:_ls_date      ;; 1 cycle
        CMP       A, #0x1E           ;; 1 cycle
        BNC       ??load_survey_ram_init_42  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
??load_survey_ram_init_41:
        MOV       B, N:_ls_month     ;; 1 cycle
        MOV       A, (_monthdays-1)[B]  ;; 1 cycle
        CMP       A, N:_ls_date      ;; 1 cycle
        BNC       ??load_survey_ram_init_43  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        CMP       N:_ls_month, #0x2  ;; 1 cycle
        BNZ       ??load_survey_ram_init_42  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        MOV       X, #0x4            ;; 1 cycle
        MOV       A, N:_ls_year      ;; 1 cycle
          CFI FunCall ?UC_MOD_L01
        CALL      N:?UC_MOD_L01      ;; 3 cycles
        MOV       B, A               ;; 1 cycle
        CMP0      B                  ;; 1 cycle
        BZ        ??load_survey_ram_init_43  ;; 4 cycles
        ; ------------------------------------- Block: 11 cycles
//  648   {
//  649     ls_date=1;
??load_survey_ram_init_42:
        MOV       N:_ls_date, #0x1   ;; 1 cycle
//  650     ls_month+=1; /* =hex_to_bcd(bcd_to_hex(var2)+1); */
        INC       N:_ls_month        ;; 2 cycles
//  651     if(ls_month>12) /* 0x12) */
        MOV       A, N:_ls_month     ;; 1 cycle
        CMP       A, #0xD            ;; 1 cycle
        BC        ??load_survey_ram_init_43  ;; 4 cycles
        ; ------------------------------------- Block: 9 cycles
//  652     {
//  653       ls_month=1;
        MOV       N:_ls_month, #0x1  ;; 1 cycle
//  654       ls_year+=1; /* =hex_to_bcd(bcd_to_hex(var1)+1); */
        INC       N:_ls_year         ;; 2 cycles
        ; ------------------------------------- Block: 3 cycles
//  655     }
//  656   }
//  657   
//  658   ls_date=decimal_to_bcd(ls_date);
??load_survey_ram_init_43:
        MOV       A, N:_ls_date      ;; 1 cycle
          CFI FunCall _decimal_to_bcd
        CALL      _decimal_to_bcd    ;; 3 cycles
        MOV       N:_ls_date, A      ;; 1 cycle
//  659   ls_month=decimal_to_bcd(ls_month);
        MOV       A, N:_ls_month     ;; 1 cycle
          CFI FunCall _decimal_to_bcd
        CALL      _decimal_to_bcd    ;; 3 cycles
        MOV       N:_ls_month, A     ;; 1 cycle
//  660   ls_year=decimal_to_bcd(ls_year);
        MOV       A, N:_ls_year      ;; 1 cycle
          CFI FunCall _decimal_to_bcd
        CALL      _decimal_to_bcd    ;; 3 cycles
        MOV       N:_ls_year, A      ;; 1 cycle
//  661 }
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock14
        ; ------------------------------------- Block: 22 cycles
        ; ------------------------------------- Total: 97 cycles
//  662 
//  663 /***********************************************************************************************************************
//  664 * File Name       : app_load_survey.c
//  665 * Description     : daily energy functionality of the meter
//  666 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock15 Using cfiCommon0
          CFI Function _save_midnight_par
        CODE
//  667 void save_midnight_par(void)
//  668 {
_save_midnight_par:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 14
        SUBW      SP, #0xE           ;; 1 cycle
          CFI CFA SP+18
//  669   us8 date, month, year;
//  670   us16 ulong_temp;
//  671   
//  672   eprom_read(DAILY_ENERGY_STATUS,0,PAGE_1,AUTO_CALC);
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x6300        ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
//  673   midnight_par_cnt= opr_data[0];
        MOV       A, N:_opr_data     ;; 1 cycle
        MOV       N:_midnight_par_cnt, A  ;; 1 cycle
//  674   
//  675   if (opr_data[2] != present_date || (opr_data[2] == present_date && opr_data[3] != present_month) 
//  676       || (opr_data[2] == present_date && opr_data[3] == present_month && opr_data[4] != present_year))
        MOV       A, N:_opr_data+2   ;; 1 cycle
        CMP       A, N:_Now+3        ;; 1 cycle
        BNZ       ??load_survey_ram_init_44  ;; 4 cycles
        ; ------------------------------------- Block: 16 cycles
        MOV       A, N:_opr_data+2   ;; 1 cycle
        CMP       A, N:_Now+3        ;; 1 cycle
        BNZ       ??load_survey_ram_init_45  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOV       A, N:_opr_data+3   ;; 1 cycle
        CMP       A, N:_Now+5        ;; 1 cycle
        BNZ       ??load_survey_ram_init_44  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
??load_survey_ram_init_45:
        MOV       A, N:_opr_data+2   ;; 1 cycle
        CMP       A, N:_Now+3        ;; 1 cycle
        SKZ                          ;; 4 cycles
        BR        N:??load_survey_ram_init_46  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOV       A, N:_opr_data+3   ;; 1 cycle
        CMP       A, N:_Now+5        ;; 1 cycle
        SKZ                          ;; 4 cycles
        BR        N:??load_survey_ram_init_46  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOV       A, N:_opr_data+4   ;; 1 cycle
        CMP       A, N:_Now+6        ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??load_survey_ram_init_46  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  677   {
//  678     midnight_par_cnt++;
??load_survey_ram_init_44:
        INC       N:_midnight_par_cnt  ;; 2 cycles
//  679     if(midnight_par_cnt > max_midnight_cnt)
        MOV       A, N:_midnight_par_cnt  ;; 1 cycle
        CMP       A, #0x4D           ;; 1 cycle
        BC        ??load_survey_ram_init_47  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
//  680     {
//  681       midnight_par_cnt= 1;
        MOV       N:_midnight_par_cnt, #0x1  ;; 1 cycle
//  682       midnight_roll_f= 1;
        MOV       N:_midnight_roll_f, #0x1  ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
//  683     }
//  684     
//  685     
//  686     /* **************first page*********************/
//  687     
//  688     if (midnight_fill_rtc_f == 1)
??load_survey_ram_init_47:
        CMP       N:_midnight_fill_rtc_f, #0x1  ;; 1 cycle
        SKZ                          ;; 4 cycles
        BR        N:??load_survey_ram_init_48  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  689     {
//  690       if (seq_no_transaction % COMPART_TRANSACTION_ENTRIES == 0)
        XCH       A, D               ;; 1 cycle
        MOV       A, N:_COMPART_TRANSACTION_ENTRIES  ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOVW      AX, N:_seq_no_transaction  ;; 1 cycle
        DIVHU                        ;; 9 cycles
        NOP                          ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        CMPW      AX, #0x0           ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        BNZ       ??load_survey_ram_init_49  ;; 4 cycles
        ; ------------------------------------- Block: 23 cycles
//  691       {
//  692         ulong_temp = COMPART_TRANSACTION_START_ADD + ((COMPART_TRANSACTION_ENTRIES - 1) * 0x10);
        MOV       X, N:_COMPART_TRANSACTION_ENTRIES  ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        DECW      AX                 ;; 1 cycle
        MOVW      BC, #0x10          ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, N:_COMPART_TRANSACTION_START_ADD  ;; 1 cycle
        MOVW      [SP+0x04], AX      ;; 1 cycle
        BR        S:??load_survey_ram_init_50  ;; 3 cycles
        ; ------------------------------------- Block: 11 cycles
//  693       }
//  694       else
//  695       {
//  696         ulong_temp = (us16)((us16)((seq_no_transaction - 1) % COMPART_TRANSACTION_ENTRIES) * 0x10) + COMPART_TRANSACTION_START_ADD;
??load_survey_ram_init_49:
        XCH       A, D               ;; 1 cycle
        MOV       A, N:_COMPART_TRANSACTION_ENTRIES  ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOVW      AX, N:_seq_no_transaction  ;; 1 cycle
        DECW      AX                 ;; 1 cycle
        DIVHU                        ;; 9 cycles
        NOP                          ;; 1 cycle
        MOVW      AX, DE             ;; 1 cycle
        MOVW      BC, #0x10          ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, N:_COMPART_TRANSACTION_START_ADD  ;; 1 cycle
        MOVW      [SP+0x04], AX      ;; 1 cycle
        ; ------------------------------------- Block: 23 cycles
//  697       }
//  698       
//  699       eprom_read(ulong_temp,0,PAGE_1,AUTO_CALC);
??load_survey_ram_init_50:
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
//  700       TempTime = char_array_into_time5_sec(&opr_data[0]);
        MOVW      BC, #LWRD(_opr_data)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x6           ;; 1 cycle
          CFI FunCall _char_array_into_time5_sec
        CALL      _char_array_into_time5_sec  ;; 3 cycles
        MOVW      HL, #LWRD(_TempTime)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x6           ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOV       A, [DE]            ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
        MOV       A, [DE+0x01]       ;; 1 cycle
        MOV       [HL+0x01], A       ;; 1 cycle
        MOV       A, [DE+0x02]       ;; 1 cycle
        MOV       [HL+0x02], A       ;; 1 cycle
        MOV       A, [DE+0x03]       ;; 1 cycle
        MOV       [HL+0x03], A       ;; 1 cycle
        MOV       A, [DE+0x04]       ;; 1 cycle
        MOV       [HL+0x04], A       ;; 1 cycle
        MOV       A, [DE+0x05]       ;; 1 cycle
        MOV       [HL+0x05], A       ;; 1 cycle
        MOV       A, [DE+0x06]       ;; 1 cycle
        MOV       [HL+0x06], A       ;; 1 cycle
//  701       date= TempTime.day;
        MOV       A, N:_TempTime+3   ;; 1 cycle
        MOV       [SP+0x02], A       ;; 1 cycle
//  702       month= TempTime.month;
        MOV       A, N:_TempTime+5   ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  703       year= TempTime.year;
        MOV       A, N:_TempTime+6   ;; 1 cycle
        MOV       [SP+0x01], A       ;; 1 cycle
        ; ------------------------------------- Block: 37 cycles
//  704       
//  705     }
//  706     
//  707     opr_data[0]= present_date;
??load_survey_ram_init_48:
        MOV       A, N:_Now+3        ;; 1 cycle
        MOV       N:_opr_data, A     ;; 1 cycle
//  708     opr_data[1]= present_month;
        MOV       A, N:_Now+5        ;; 1 cycle
        MOV       N:_opr_data+1, A   ;; 1 cycle
//  709     opr_data[2]= present_year;
        MOV       A, N:_Now+6        ;; 1 cycle
        MOV       N:_opr_data+2, A   ;; 1 cycle
//  710     
//  711     if(midnight_par_miss_f == 1)
        CMP       N:_midnight_par_miss_f, #0x1  ;; 1 cycle
        BNZ       ??load_survey_ram_init_51  ;; 4 cycles
        ; ------------------------------------- Block: 11 cycles
//  712     {
//  713       date= temp3_var;
        MOV       A, N:_temp3_var    ;; 1 cycle
        MOV       [SP+0x02], A       ;; 1 cycle
//  714       month= temp2_var;
        MOV       A, N:_temp2_var    ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  715       year= temp1_var;
        MOV       A, N:_temp1_var    ;; 1 cycle
        MOV       [SP+0x01], A       ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
//  716     }
//  717     
//  718     if((midnight_par_miss_f == 1) || (midnight_fill_rtc_f == 1))
??load_survey_ram_init_51:
        CMP       N:_midnight_par_miss_f, #0x1  ;; 1 cycle
        BZ        ??load_survey_ram_init_52  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        CMP       N:_midnight_fill_rtc_f, #0x1  ;; 1 cycle
        SKZ                          ;; 4 cycles
        BR        N:??load_survey_ram_init_53  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  719     {
//  720       midnight_par_miss_f= 0;
??load_survey_ram_init_52:
        MOV       N:_midnight_par_miss_f, #0x0  ;; 1 cycle
//  721       midnight_fill_rtc_f= 0;
        MOV       N:_midnight_fill_rtc_f, #0x0  ;; 1 cycle
//  722       if((present_year != year) || (present_month != month) || (present_date != date))
        MOV       X, N:_Now+6        ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        CMP       X, A               ;; 1 cycle
        BNZ       ??load_survey_ram_init_54  ;; 4 cycles
        ; ------------------------------------- Block: 9 cycles
        MOV       X, N:_Now+5        ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        CMP       X, A               ;; 1 cycle
        BNZ       ??load_survey_ram_init_54  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOV       X, N:_Now+3        ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        CMP       X, A               ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??load_survey_ram_init_53  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  723       {
//  724         opr_data[0]= decimal_to_bcd(bcd_to_decimal(date) + 1);
??load_survey_ram_init_54:
        MOV       A, [SP+0x02]       ;; 1 cycle
          CFI FunCall _bcd_to_decimal
        CALL      _bcd_to_decimal    ;; 3 cycles
        INC       A                  ;; 1 cycle
          CFI FunCall _decimal_to_bcd
        CALL      _decimal_to_bcd    ;; 3 cycles
        MOV       N:_opr_data, A     ;; 1 cycle
//  725         opr_data[1]= month;
        MOV       A, [SP]            ;; 1 cycle
        MOV       N:_opr_data+1, A   ;; 1 cycle
//  726         opr_data[2]= year;
        MOV       A, [SP+0x01]       ;; 1 cycle
        MOV       N:_opr_data+2, A   ;; 1 cycle
//  727         if(((month == 2) && ((bcd_to_decimal(year)) % 4 == 0) && (opr_data[0] > 0x29)) || ((bcd_to_decimal(opr_data[0]) > (monthdays[(bcd_to_decimal(month)) - 1])) && ((month != 2) || ((bcd_to_decimal(year)) % 4 != 0))))
        MOV       A, [SP]            ;; 1 cycle
        CMP       A, #0x2            ;; 1 cycle
        BNZ       ??load_survey_ram_init_55  ;; 4 cycles
        ; ------------------------------------- Block: 19 cycles
        MOV       X, #0x4            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+20
        MOV       A, [SP+0x03]       ;; 1 cycle
          CFI FunCall _bcd_to_decimal
        CALL      _bcd_to_decimal    ;; 3 cycles
        XCH       A, X               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+18
        XCH       A, X               ;; 1 cycle
          CFI FunCall ?UC_MOD_L01
        CALL      N:?UC_MOD_L01      ;; 3 cycles
        MOV       B, A               ;; 1 cycle
        CMP0      B                  ;; 1 cycle
        BNZ       ??load_survey_ram_init_55  ;; 4 cycles
        ; ------------------------------------- Block: 19 cycles
        MOV       A, N:_opr_data     ;; 1 cycle
        CMP       A, #0x2A           ;; 1 cycle
        BNC       ??load_survey_ram_init_56  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
??load_survey_ram_init_55:
        MOV       A, [SP]            ;; 1 cycle
          CFI FunCall _bcd_to_decimal
        CALL      _bcd_to_decimal    ;; 3 cycles
        MOV       [SP+0x03], A       ;; 1 cycle
        MOV       A, N:_opr_data     ;; 1 cycle
          CFI FunCall _bcd_to_decimal
        CALL      _bcd_to_decimal    ;; 3 cycles
        MOV       X, A               ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        MOV       A, (_monthdays-1)[B]  ;; 1 cycle
        CMP       A, X               ;; 1 cycle
        BNC       ??load_survey_ram_init_53  ;; 4 cycles
        ; ------------------------------------- Block: 18 cycles
        MOV       A, [SP]            ;; 1 cycle
        CMP       A, #0x2            ;; 1 cycle
        BNZ       ??load_survey_ram_init_56  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOV       X, #0x4            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+20
        MOV       A, [SP+0x03]       ;; 1 cycle
          CFI FunCall _bcd_to_decimal
        CALL      _bcd_to_decimal    ;; 3 cycles
        XCH       A, X               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+18
        XCH       A, X               ;; 1 cycle
          CFI FunCall ?UC_MOD_L01
        CALL      N:?UC_MOD_L01      ;; 3 cycles
        MOV       B, A               ;; 1 cycle
        CMP0      B                  ;; 1 cycle
        BZ        ??load_survey_ram_init_53  ;; 4 cycles
        ; ------------------------------------- Block: 19 cycles
//  728         {
//  729           opr_data[0]= 1;
??load_survey_ram_init_56:
        MOV       N:_opr_data, #0x1  ;; 1 cycle
//  730           opr_data[1]= decimal_to_bcd(bcd_to_decimal(month) + 1);
        MOV       A, [SP]            ;; 1 cycle
          CFI FunCall _bcd_to_decimal
        CALL      _bcd_to_decimal    ;; 3 cycles
        INC       A                  ;; 1 cycle
          CFI FunCall _decimal_to_bcd
        CALL      _decimal_to_bcd    ;; 3 cycles
        MOV       N:_opr_data+1, A   ;; 1 cycle
//  731           if(opr_data[1] > 0x12)
        MOV       A, N:_opr_data+1   ;; 1 cycle
        CMP       A, #0x13           ;; 1 cycle
        BC        ??load_survey_ram_init_53  ;; 4 cycles
        ; ------------------------------------- Block: 16 cycles
//  732           {
//  733             opr_data[1]= 1;
        MOV       N:_opr_data+1, #0x1  ;; 1 cycle
//  734             opr_data[2]= decimal_to_bcd(bcd_to_decimal(year) + 1);
        MOV       A, [SP+0x01]       ;; 1 cycle
          CFI FunCall _bcd_to_decimal
        CALL      _bcd_to_decimal    ;; 3 cycles
        INC       A                  ;; 1 cycle
          CFI FunCall _decimal_to_bcd
        CALL      _decimal_to_bcd    ;; 3 cycles
        MOV       N:_opr_data+2, A   ;; 1 cycle
        ; ------------------------------------- Block: 10 cycles
//  735           }
//  736         }
//  737       }
//  738     }
//  739     
//  740     
//  741     long_into_char_array4(energy.Allph.active_imp,&opr_data[3]);             //energy_import
??load_survey_ram_init_53:
        MOVW      DE, #LWRD(_opr_data+3)  ;; 1 cycle
        MOVW      BC, N:_energy+54   ;; 1 cycle
        MOVW      AX, N:_energy+52   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  742     long_into_char_array4(energy.Allph.apparent_imp,&opr_data[7]);           //app_energy       
        MOVW      DE, #LWRD(_opr_data+7)  ;; 1 cycle
        MOVW      BC, N:_energy+70   ;; 1 cycle
        MOVW      AX, N:_energy+68   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  743     long_into_char_array3(power_on_min,&opr_data[11]);                      //cum power on min  
        MOVW      DE, #LWRD(_opr_data+11)  ;; 1 cycle
        MOVW      BC, N:_power_on_min+2  ;; 1 cycle
        MOVW      AX, N:_power_on_min  ;; 1 cycle
          CFI FunCall _long_into_char_array3
        CALL      _long_into_char_array3  ;; 3 cycles
//  744     opr_data[14]= 0;                                                         //blank 2 byte , use if any requirement
        MOV       N:_opr_data+14, #0x0  ;; 1 cycle
//  745     opr_data[15]= 0;
        MOV       N:_opr_data+15, #0x0  ;; 1 cycle
//  746     
//  747     ulong_temp= DLOADSURVEY_INIT_ADD + ((us16)(midnight_par_cnt - 1) * DAILY_ENERGY_SNAP_SIZE);
        MOV       X, N:_midnight_par_cnt  ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x60          ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #0x3FA0        ;; 1 cycle
        MOVW      [SP+0x04], AX      ;; 1 cycle
//  748     eprom_write(ulong_temp,0,16,PAGE_1,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+20
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, [SP+0x06]      ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  749     
//  750     /* **************second page*********************/
//  751     fill_oprzero(16);     
        MOV       A, #0x10           ;; 1 cycle
          CFI FunCall _fill_oprzero
        CALL      _fill_oprzero      ;; 3 cycles
//  752     long_into_char_array4(energy.Allph.reactive_q1,&opr_data[0]);           //reactive energy lag
        MOVW      DE, #LWRD(_opr_data)  ;; 1 cycle
        MOVW      BC, N:_energy+78   ;; 1 cycle
        MOVW      AX, N:_energy+76   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  753     long_into_char_array4(energy.Allph.reactive_q4,&opr_data[4]);           //reactive energy lead
        MOVW      DE, #LWRD(_opr_data+4)  ;; 1 cycle
        MOVW      BC, N:_energy+90   ;; 1 cycle
        MOVW      AX, N:_energy+88   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  754     
//  755     ulong_temp+=0x10;
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
        MOVW      [SP+0x06], AX      ;; 1 cycle
//  756     eprom_write(ulong_temp,0,16,PAGE_1,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+22
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, [SP+0x08]      ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  757     /* **************second page end*********************/
//  758     eprom_read(DAILY_ENERGY_STATUS,0,PAGE_1,AUTO_CALC);
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x6300        ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
//  759     opr_data[0] = midnight_par_cnt;
        MOV       A, N:_midnight_par_cnt  ;; 1 cycle
        MOV       N:_opr_data, A     ;; 1 cycle
//  760     opr_data[1] = midnight_roll_f;
        MOV       A, N:_midnight_roll_f  ;; 1 cycle
        MOV       N:_opr_data+1, A   ;; 1 cycle
//  761     opr_data[2] = present_date;
        MOV       A, N:_Now+3        ;; 1 cycle
        MOV       N:_opr_data+2, A   ;; 1 cycle
//  762     opr_data[3] = present_month;
        MOV       A, N:_Now+5        ;; 1 cycle
        MOV       N:_opr_data+3, A   ;; 1 cycle
//  763     opr_data[4] = present_year;
        MOV       A, N:_Now+6        ;; 1 cycle
        MOV       N:_opr_data+4, A   ;; 1 cycle
//  764     eprom_write(DAILY_ENERGY_STATUS,0,16,PAGE_1,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+24
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x6300        ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
        ADDW      SP, #0x6           ;; 1 cycle
          CFI CFA SP+18
        ; ------------------------------------- Block: 91 cycles
//  765   }
//  766 }
??load_survey_ram_init_46:
        ADDW      SP, #0xE           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock15
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 416 cycles
//  767 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock16 Using cfiCommon0
          CFI Function _midnight_var_init
        CODE
//  768 void midnight_var_init(void )
//  769 {
_midnight_var_init:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  770   eprom_read(DAILY_ENERGY_STATUS,0,PAGE_1,AUTO_CALC);
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x6300        ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
//  771   midnight_par_cnt= opr_data[0];
        MOV       A, N:_opr_data     ;; 1 cycle
        MOV       N:_midnight_par_cnt, A  ;; 1 cycle
//  772   midnight_roll_f= opr_data[1];
        MOV       A, N:_opr_data+1   ;; 1 cycle
        MOV       N:_midnight_roll_f, A  ;; 1 cycle
//  773   
//  774   //    max_midnight_cnt= (MAX_LS / (24 * mdi_sel_ls));                      //*daily energy days must be equal to load survey days*/
//  775   //    if(max_midnight_cnt > MAX_DAILY_DAYS)
//  776   //    {
//  777   //        max_midnight_cnt= MAX_DAILY_DAYS;
//  778   //    } 
//  779   //      if(midnight_par_cnt > max_midnight_cnt)
//  780   //    {
//  781   //        midnight_par_cnt= max_midnight_cnt;
//  782   //    }
//  783   
//  784   /* ***************Event counter********** */
//  785   //    read_from_16(0x08f0);
//  786   //    event_counter= OPR10;
//  787   //    event_roll_f= OPR11[0];
//  788   
//  789 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock16
        ; ------------------------------------- Block: 17 cycles
        ; ------------------------------------- Total: 17 cycles
//  790 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock17 Using cfiCommon0
          CFI Function _load_survey_ram_init
        CODE
//  791 void load_survey_ram_init()
//  792 {
_load_survey_ram_init:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 2
        SUBW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+6
//  793   eprom_read(0x07B0,0,PAGE_1,AUTO_CALC);
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x7B0         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
//  794   mdi_sel_ls=opr_data[0];
        MOV       A, N:_opr_data     ;; 1 cycle
        MOV       N:_mdi_sel_ls, A   ;; 1 cycle
//  795   if(mdi_sel_ls==4)
        CMP       N:_mdi_sel_ls, #0x4  ;; 1 cycle
        BNZ       ??load_survey_ram_init_57  ;; 4 cycles
        ; ------------------------------------- Block: 15 cycles
//  796   {
//  797     lsip_period = 15;
        MOV       N:_lsip_period, #0xF  ;; 1 cycle
        BR        S:??load_survey_ram_init_58  ;; 3 cycles
        ; ------------------------------------- Block: 4 cycles
//  798   }
//  799   else //it will also cover the default setting in case of checksum mismatch or non configuration
//  800   {
//  801     lsip_period = 30;
??load_survey_ram_init_57:
        MOV       N:_lsip_period, #0x1E  ;; 1 cycle
//  802     mdi_sel_ls = 2;
        MOV       N:_mdi_sel_ls, #0x2  ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
//  803   }
//  804   
//  805   max_load_survey_cnt = MAX_LS;
??load_survey_ram_init_58:
        MOVW      AX, #0xE40         ;; 1 cycle
        MOVW      N:_max_load_survey_cnt, AX  ;; 1 cycle
//  806   
//  807   eprom_read(0x0770,0,PAGE_1,AUTO_CALC);
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x770         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
//  808   seq_no_ls = char_array_to_long3(&opr_data[0]);
        MOVW      AX, #LWRD(_opr_data)  ;; 1 cycle
          CFI FunCall _char_array_to_long3
        CALL      _char_array_to_long3  ;; 3 cycles
        MOVW      N:_seq_no_ls, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_seq_no_ls+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  809   
//  810   
//  811   if(eprom_read(0x0730,0,PAGE_2,AUTO_CALC) == EEP_OK)
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x1            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x730         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
        CMP       A, #0x1            ;; 1 cycle
        SKZ                          ;; 4 cycles
        BR        N:??load_survey_ram_init_59  ;; 4 cycles
        ; ------------------------------------- Block: 29 cycles
//  812   {
//  813     ls_avg_acc_vol_r = char_array_to_int(&opr_data[0]);
        MOVW      AX, #LWRD(_opr_data)  ;; 1 cycle
          CFI FunCall _char_array_to_int
        CALL      _char_array_to_int  ;; 3 cycles
        MOVW      N:_ls_avg_acc_vol_r, AX  ;; 1 cycle
//  814     ls_avg_acc_vol_y = char_array_to_int(&opr_data[2]);
        MOVW      AX, #LWRD(_opr_data+2)  ;; 1 cycle
          CFI FunCall _char_array_to_int
        CALL      _char_array_to_int  ;; 3 cycles
        MOVW      N:_ls_avg_acc_vol_y, AX  ;; 1 cycle
//  815     ls_avg_acc_vol_b = char_array_to_int(&opr_data[4]);
        MOVW      AX, #LWRD(_opr_data+4)  ;; 1 cycle
          CFI FunCall _char_array_to_int
        CALL      _char_array_to_int  ;; 3 cycles
        MOVW      N:_ls_avg_acc_vol_b, AX  ;; 1 cycle
//  816     ls_avg_acc_curr_r = char_array_to_long3(&opr_data[6]);
        MOVW      AX, #LWRD(_opr_data+6)  ;; 1 cycle
          CFI FunCall _char_array_to_long3
        CALL      _char_array_to_long3  ;; 3 cycles
        MOVW      N:_ls_avg_acc_curr_r, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_ls_avg_acc_curr_r+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  817     ls_avg_acc_curr_y = char_array_to_long3(&opr_data[9]);
        MOVW      AX, #LWRD(_opr_data+9)  ;; 1 cycle
          CFI FunCall _char_array_to_long3
        CALL      _char_array_to_long3  ;; 3 cycles
        MOVW      N:_ls_avg_acc_curr_y, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_ls_avg_acc_curr_y+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  818     ls_avg_acc_curr_b = char_array_to_long3(&opr_data[12]);
        MOVW      AX, #LWRD(_opr_data+12)  ;; 1 cycle
          CFI FunCall _char_array_to_long3
        CALL      _char_array_to_long3  ;; 3 cycles
        MOVW      N:_ls_avg_acc_curr_b, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_ls_avg_acc_curr_b+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  819     ls_avg_acc_curr_n = char_array_to_long3(&opr_data[15]);
        MOVW      AX, #LWRD(_opr_data+15)  ;; 1 cycle
          CFI FunCall _char_array_to_long3
        CALL      _char_array_to_long3  ;; 3 cycles
        MOVW      N:_ls_avg_acc_curr_n, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_ls_avg_acc_curr_n+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  820     ls_avg_acc_pf_r = char_array_to_int(&opr_data[18]);
        MOVW      AX, #LWRD(_opr_data+18)  ;; 1 cycle
          CFI FunCall _char_array_to_int
        CALL      _char_array_to_int  ;; 3 cycles
        MOVW      N:_ls_avg_acc_pf_r, AX  ;; 1 cycle
//  821     ls_avg_acc_pf_y = char_array_to_int(&opr_data[20]);
        MOVW      AX, #LWRD(_opr_data+20)  ;; 1 cycle
          CFI FunCall _char_array_to_int
        CALL      _char_array_to_int  ;; 3 cycles
        MOVW      N:_ls_avg_acc_pf_y, AX  ;; 1 cycle
//  822     ls_avg_acc_pf_b = char_array_to_int(&opr_data[22]);
        MOVW      AX, #LWRD(_opr_data+22)  ;; 1 cycle
          CFI FunCall _char_array_to_int
        CALL      _char_array_to_int  ;; 3 cycles
        MOVW      N:_ls_avg_acc_pf_b, AX  ;; 1 cycle
//  823     ls_avg_acc_pf_net = char_array_to_int(&opr_data[24]);
        MOVW      AX, #LWRD(_opr_data+24)  ;; 1 cycle
          CFI FunCall _char_array_to_int
        CALL      _char_array_to_int  ;; 3 cycles
        MOVW      N:_ls_avg_acc_pf_net, AX  ;; 1 cycle
//  824     ls_pom = opr_data[26];
        MOV       A, N:_opr_data+26  ;; 1 cycle
        MOV       N:_ls_pom, A       ;; 1 cycle
//  825     ls_avg_acc_freq_net = char_array_to_long3(&opr_data[27]);
        MOVW      AX, #LWRD(_opr_data+27)  ;; 1 cycle
          CFI FunCall _char_array_to_long3
        CALL      _char_array_to_long3  ;; 3 cycles
        MOVW      N:_ls_avg_acc_freq_net, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_ls_avg_acc_freq_net+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        BR        S:??load_survey_ram_init_60  ;; 3 cycles
        ; ------------------------------------- Block: 80 cycles
//  826   }
//  827   else
//  828   {
//  829     ls_status |= bit2;
??load_survey_ram_init_59:
        SET1      N:_ls_status.2     ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
//  830   }
//  831   ls_status |= bit4;
??load_survey_ram_init_60:
        SET1      N:_ls_status.4     ;; 2 cycles
//  832   if(hard_reset_f == 1)
        CMP       N:_hard_reset_f, #0x1  ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 4 cycles
//  833   {
//  834     ls_status |= bit3;
        SET1      N:_ls_status.3     ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
//  835   }
//  836   if(eprom_read(0x0780,0,PAGE_1,AUTO_CALC) == EEP_OK)
??load_survey_ram_init_61:
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x780         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
        CMP       A, #0x1            ;; 1 cycle
        BNZ       ??load_survey_ram_init_62  ;; 4 cycles
        ; ------------------------------------- Block: 12 cycles
//  837   {
//  838     lsro_flag=opr_data[0];
        MOV       A, N:_opr_data     ;; 1 cycle
        MOV       N:_lsro_flag, A    ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
//  839   }
//  840   
//  841   /* ********************dlms LS Configuration*********** */
//  842   
//  843   if(eprom_read(0x6310,0,PAGE_1,AUTO_CALC) == EEP_OK)
??load_survey_ram_init_62:
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x6310        ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
        CMP       A, #0x1            ;; 1 cycle
        BNZ       ??load_survey_ram_init_63  ;; 4 cycles
        ; ------------------------------------- Block: 12 cycles
//  844   {
//  845     ls_max_obj= opr_data[0];
        MOV       A, N:_opr_data     ;; 1 cycle
        MOV       N:_ls_max_obj, A   ;; 1 cycle
//  846     for(temp_us8= 0; temp_us8 < MAX_LS_OBJ; temp_us8++)
        MOV       S:_temp_us8, #0x0  ;; 1 cycle
        ; ------------------------------------- Block: 3 cycles
??load_survey_ram_init_64:
        MOV       A, S:_temp_us8     ;; 1 cycle
        CMP       A, #0xB            ;; 1 cycle
        BNC       ??load_survey_ram_init_63  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  847     {
//  848       ls_conf_obj[temp_us8]= opr_data[temp_us8+1];
        MOV       B, S:_temp_us8     ;; 1 cycle
        MOV       A, (_opr_data+1)[B]  ;; 1 cycle
        MOV       B, S:_temp_us8     ;; 1 cycle
        MOV       (_ls_conf_obj)[B], A  ;; 1 cycle
//  849     }
        INC       S:_temp_us8        ;; 2 cycles
        BR        S:??load_survey_ram_init_64  ;; 3 cycles
        ; ------------------------------------- Block: 9 cycles
//  850   }
//  851   if((0 == ls_max_obj)||(eprom_read(0x6310,0,PAGE_1,AUTO_CALC) != EEP_OK))
??load_survey_ram_init_63:
        CMP0      N:_ls_max_obj      ;; 1 cycle
        BZ        ??load_survey_ram_init_65  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x6310        ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
        CMP       A, #0x1            ;; 1 cycle
        BZ        ??load_survey_ram_init_66  ;; 4 cycles
        ; ------------------------------------- Block: 12 cycles
//  852   {
//  853     ls_max_obj= MAX_LS_OBJ;
??load_survey_ram_init_65:
        MOV       N:_ls_max_obj, #0xB  ;; 1 cycle
//  854     for(temp_us8= 0; temp_us8 < MAX_LS_OBJ; temp_us8++)
        MOV       S:_temp_us8, #0x0  ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
??load_survey_ram_init_67:
        MOV       A, S:_temp_us8     ;; 1 cycle
        CMP       A, #0xB            ;; 1 cycle
        BNC       ??load_survey_ram_init_66  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  855     {
//  856       ls_conf_obj[temp_us8]= temp_us8;
        MOV       B, S:_temp_us8     ;; 1 cycle
        MOV       A, S:_temp_us8     ;; 1 cycle
        MOV       (_ls_conf_obj)[B], A  ;; 1 cycle
//  857     }
        INC       S:_temp_us8        ;; 2 cycles
        BR        S:??load_survey_ram_init_67  ;; 3 cycles
        ; ------------------------------------- Block: 8 cycles
//  858   }
//  859   
//  860   if(eprom_read(0x0790,0,PAGE_1,AUTO_CALC) == EEP_OK)
??load_survey_ram_init_66:
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x790         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
        CMP       A, #0x1            ;; 1 cycle
        BNZ       ??load_survey_ram_init_68  ;; 4 cycles
        ; ------------------------------------- Block: 12 cycles
//  861   {
//  862     day_counter= opr_data[0];
        MOV       X, N:_opr_data     ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      N:_day_counter, AX  ;; 1 cycle
//  863     max_day_counter= opr_data[1];
        MOV       A, N:_opr_data+1   ;; 1 cycle
        MOV       N:_max_day_counter, A  ;; 1 cycle
//  864     if((max_day_counter == 0) || (max_day_counter == 255))
        CMP0      N:_max_day_counter  ;; 1 cycle
        BZ        ??load_survey_ram_init_69  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
        CMP       N:_max_day_counter, #0xFF  ;; 1 cycle
        BNZ       ??load_survey_ram_init_68  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  865     {
//  866       max_day_counter= MAX_LS / (24 * mdi_sel_ls);
??load_survey_ram_init_69:
        MOV       X, N:_mdi_sel_ls   ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x18          ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, #0xE40         ;; 1 cycle
          CFI FunCall ?SI_DIV_L02
        CALL      N:?SI_DIV_L02      ;; 3 cycles
        XCH       A, X               ;; 1 cycle
        MOV       N:_max_day_counter, A  ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        ; ------------------------------------- Block: 13 cycles
//  867     }
//  868   }
//  869   
//  870   temp_us16= 0x0D00;
??load_survey_ram_init_68:
        MOVW      S:_temp_us16, #0xD00  ;; 1 cycle
//  871   
//  872   for(us8 m1= 0; m1 < 26; m1++)
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        ; ------------------------------------- Block: 3 cycles
??load_survey_ram_init_70:
        MOV       A, [SP]            ;; 1 cycle
        CMP       A, #0x1A           ;; 1 cycle
        SKC                          ;; 4 cycles
        BR        N:??load_survey_ram_init_71  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  873   {
//  874     eprom_read(temp_us16,0,PAGE_1,AUTO_CALC);
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, S:_temp_us16   ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
//  875     d_array[(7*m1)] = char_array_to_int(&opr_data[0]);
        MOVW      AX, #LWRD(_opr_data)  ;; 1 cycle
          CFI FunCall _char_array_to_int
        CALL      _char_array_to_int  ;; 3 cycles
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0xE           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(_d_array)  ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      [DE], AX           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
//  876     d_array[(7*m1)+1] = char_array_to_int(&opr_data[2]);
        MOVW      AX, #LWRD(_opr_data+2)  ;; 1 cycle
          CFI FunCall _char_array_to_int
        CALL      _char_array_to_int  ;; 3 cycles
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x7           ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(_d_array+2)  ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      [DE], AX           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
//  877     d_array[(7*m1)+2] = char_array_to_int(&opr_data[4]);
        MOVW      AX, #LWRD(_opr_data+4)  ;; 1 cycle
          CFI FunCall _char_array_to_int
        CALL      _char_array_to_int  ;; 3 cycles
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x7           ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(_d_array+4)  ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      [DE], AX           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
//  878     d_array[(7*m1)+3] = char_array_to_int(&opr_data[6]);
        MOVW      AX, #LWRD(_opr_data+6)  ;; 1 cycle
          CFI FunCall _char_array_to_int
        CALL      _char_array_to_int  ;; 3 cycles
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x7           ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(_d_array+6)  ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      [DE], AX           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
//  879     d_array[(7*m1)+4] = char_array_to_int(&opr_data[8]);
        MOVW      AX, #LWRD(_opr_data+8)  ;; 1 cycle
          CFI FunCall _char_array_to_int
        CALL      _char_array_to_int  ;; 3 cycles
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x7           ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(_d_array+8)  ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      [DE], AX           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
//  880     d_array[(7*m1)+5] = char_array_to_int(&opr_data[10]);
        MOVW      AX, #LWRD(_opr_data+10)  ;; 1 cycle
          CFI FunCall _char_array_to_int
        CALL      _char_array_to_int  ;; 3 cycles
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x7           ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(_d_array+10)  ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      [DE], AX           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
//  881     d_array[(7*m1)+6] = char_array_to_int(&opr_data[12]);
        MOVW      AX, #LWRD(_opr_data+12)  ;; 1 cycle
          CFI FunCall _char_array_to_int
        CALL      _char_array_to_int  ;; 3 cycles
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x7           ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(_d_array+12)  ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      [DE], AX           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
//  882     temp_us16 += 0x10;
        MOVW      AX, S:_temp_us16   ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
        MOVW      S:_temp_us16, AX   ;; 1 cycle
//  883   }
        MOV       A, [SP]            ;; 1 cycle
        INC       A                  ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        BR        N:??load_survey_ram_init_70  ;; 3 cycles
          CFI FunCall _load_ls_cnt
        ; ------------------------------------- Block: 146 cycles
//  884   
//  885   
//  886   load_ls_cnt();
??load_survey_ram_init_71:
        CALL      _load_ls_cnt       ;; 3 cycles
//  887   
//  888 }
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock17
        ; ------------------------------------- Block: 10 cycles
        ; ------------------------------------- Total: 420 cycles

        SECTION `.iar_vfe_header`:DATA:NOALLOC:NOROOT(1)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
        DC32 0

        END
// 
//   465 bytes in section .bss
// 5'506 bytes in section .text
// 
// 5'506 bytes of CODE memory
//   465 bytes of DATA memory
//
//Errors: none
//Warnings: none
