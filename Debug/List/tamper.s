///////////////////////////////////////////////////////////////////////////////
//
// IAR C/C++ Compiler V4.20.1.2260 for RL78               20/Dec/2020  22:39:07
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
//        BootCode\source_code\source_files\tamper.c
//    Command line       =
//        -f C:\Users\laptop\AppData\Local\Temp\EW1203.tmp ("D:\Dheeraj\New
//        folder\0. GDEV72 - BootCode\source_code\source_files\tamper.c" --core
//        s3 --code_model near --calling_convention v2 --near_const_location
//        ram -o "D:\Dheeraj\New folder\0. GDEV72 - BootCode\Debug\Obj"
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
//        D:\Dheeraj\New folder\0. GDEV72 - BootCode\Debug\List\tamper.s
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
        #define SHT_IAR_NOINIT 0xabdc5467
        #define SHF_WRITE 0x1

        EXTERN _COMPART_TRANSACTION_SIZE
        EXTERN _COMPART_POWERFAIL_SIZE
        EXTERN _COMPART_NONROLLOVER_SIZE
        EXTERN _opr_data
        EXTERN _COMPART_VOLTAGE_SIZE
        EXTERN _COMPART_CURRENT_SIZE
        EXTERN _COMPART_OTHERS_SIZE
        EXTERN _temp_s8
        EXTERN _OnTime
        EXTERN _OffTime
        EXTERN _flag_eeprom_error
        EXTERN _TOP_RESTORE_REQ
        EXTERN _temp_us8
        EXTERN ?L_AND_FAST_L03
        EXTERN ?MOVE_LONG_L06
        EXTERN ?SI_DIV_L02
        EXTERN ?SL_DIV_L03
        EXTERN ?UL_CMP_L03
        EXTERN _COMPART_CURRENT_START_ADD
        EXTERN _COMPART_NONROLLOVER_START_ADD
        EXTERN _COMPART_OTHERS_START_ADD
        EXTERN _COMPART_POWERFAIL_START_ADD
        EXTERN _COMPART_TRANSACTION_START_ADD
        EXTERN _COMPART_VOLTAGE_START_ADD
        EXTERN _Eprom_ReadWM
        EXTERN _Eprom_WriteWM
        EXTERN _Now
        EXTERN _battery_voltage
        EXTERN _cal_neu_current
        EXTERN _char_array_to_int
        EXTERN _char_array_to_long4
        EXTERN _curr
        EXTERN _energy
        EXTERN _eprom_read
        EXTERN _eprom_write
        EXTERN _fill_oprzero
        EXTERN _flag_battery
        EXTERN _flag_quadrant
        EXTERN _get_minute_diff
        EXTERN _int_into_char_array
        EXTERN _long_into_char_array4
        EXTERN _pf
        EXTERN _power
        EXTERN _power_off_min
        EXTERN _save_pom
        EXTERN _seq_no_curr
        EXTERN _seq_no_debug
        EXTERN _seq_no_nonrollover
        EXTERN _seq_no_others
        EXTERN _seq_no_powerfail
        EXTERN _seq_no_transaction
        EXTERN _seq_no_vol
        EXTERN _temp_us16
        EXTERN _time_into_char_array5_sec
        EXTERN _tlv
        EXTERN _tlv_flag
        EXTERN _vol

        PUBLIC __A_P4
        PUBLIC _battery_function
        PUBLIC _check_tamper
        PUBLIC _check_tamper1
        PUBLIC _curr_unbal_time_indicate
        PUBLIC _curr_unbal_time_restore
        PUBLIC _curr_unbal_time_store
        PUBLIC _flag_mag
        PUBLIC _flag_tamper2
        PUBLIC _magnet_log_logic
        PUBLIC _mem_log
        PUBLIC _mem_log1
        PUBLIC _neu_disturb_time_indicate
        PUBLIC _neu_disturb_time_restore
        PUBLIC _neu_disturb_time_store
        PUBLIC _power_on_event
        PUBLIC _read_tamper_variables
        PUBLIC _tamper_event_status
        PUBLIC _tamper_function_100ms
        PUBLIC _tamper_function_1sec
        PUBLIC _tamper_instant_status
        PUBLIC _tamper_ram_init
        PUBLIC _tamper_sel
        PUBLIC _thr
        PUBLIC _time
        PUBLIC _top_cover_function
        PUBLIC _top_cover_restore
        PUBLIC _top_cover_restore_command
        PUBLIC _tpr
        PUBLIC _unbal_current
        PUBLIC _update_tamper_variables
        
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
          CFI EndCommon cfiCommon0
        
        
          CFI Common cfiCommon1 Using cfiNames0
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
          CFI EndCommon cfiCommon1
        
        
          CFI Common cfiCommon2 Using cfiNames0
          CFI CodeAlign 1
          CFI DataAlign 1
          CFI ReturnAddress ?RET CODE
          CFI CFA SP+4
          CFI A Undefined
          CFI X Undefined
          CFI B Undefined
          CFI C Undefined
          CFI D Undefined
          CFI E Undefined
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
          CFI EndCommon cfiCommon2
        
// D:\Dheeraj\New folder\0. GDEV72 - BootCode\source_code\source_files\tamper.c
//    1 /***********************************************************************************************************************
//    2 * File Name       : tamper.c
//    3 * Current Version : rev_01  
//    4 * Tool-Chain      : IAR Systems RL78
//    5 * Description     : This file handels all type of events and tampers being occured in the meter
//    6 * Creation Date   : 29-02-2020
//    7 * Company         : Genus Power Infrastructures Limited, Jaipur
//    8 * Author          : dheeraj.singhal
//    9 * Version History : rev_01 :
//   10 ***********************************************************************************************************************/
//   11 /************************************ Includes **************************************/
//   12 #include "tamper.h"

        ASEGN `.sbss.noinit`:DATA:NOROOT,0fff04H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_P4
// __no_init union <unnamed>#7 volatile __saddr _A_P4
__A_P4:
        DS 1
//   13 /************************************ Local Variables *****************************************/
//   14 /************************************ Extern Variables *****************************************/

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   15 flag_union flag_tamper2,flag_mag;
_flag_tamper2:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_flag_mag:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   16 us8 top_cover_restore_command;
_top_cover_restore_command:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   17 us16 curr_unbal_time_store, curr_unbal_time_restore, curr_unbal_time_indicate, unbal_current;
_curr_unbal_time_store:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_curr_unbal_time_restore:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_curr_unbal_time_indicate:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_unbal_current:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   18 us16 neu_disturb_time_store, neu_disturb_time_restore, neu_disturb_time_indicate;
_neu_disturb_time_store:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_neu_disturb_time_restore:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_neu_disturb_time_indicate:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   19 TAMPER tpr;
_tpr:
        DS 628

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   20 THRESHOLD thr;
_thr:
        DS 86

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   21 TAMPER_TIME time;
_time:
        DS 66

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   22 us32 tamper_sel,tamper_event_status,tamper_instant_status;
_tamper_sel:
        DS 4

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_tamper_event_status:
        DS 4

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_tamper_instant_status:
        DS 4
//   23 /************************************ Local Functions *******************************/
//   24 __near_func void check_tamper(Condition condition, TAMPER_VAR *tamper, us16 str_time, us16 rstr_time, us16 indicate_time);
//   25 __near_func void check_tamper1(Condition condition, TAMPER_VAR1 *tamper, us16 str_time, us16 rstr_time, us16 indicate_time);
//   26 void update_tamper_variables();
//   27 void read_tamper_variables();
//   28 /************************************ Extern Functions ******************************/
//   29 void tamper_ram_init();
//   30 void tamper_function_1sec();
//   31 void tamper_function_100ms();
//   32 void battery_function();
//   33 void top_cover_function();
//   34 void top_cover_restore();
//   35 void magnet_log_logic();
//   36 void mem_log1(Compartment1 compart, us16 event_id);
//   37 void mem_log(Compartment compart, us16 event_id, TAMPER_VAR tamper);
//   38 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock0 Using cfiCommon0
          CFI Function _mem_log1
        CODE
//   39 void mem_log1(Compartment1 compart, us16 event_id)
//   40 {
_mem_log1:
        ; * Stack frame (at entry) *
        ; Param size: 0
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+6
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+8
        ; Auto size: 8
        SUBW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+12
//   41     us32 address;
//   42     
//   43     if(event_id == EVENT_ID_POWER_FAIL_OCC)
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        CMPW      AX, #0x65          ;; 1 cycle
        BNZ       ??magnet_log_logic_0  ;; 4 cycles
        ; ------------------------------------- Block: 9 cycles
//   44     {
//   45         tpr.power_fail.count++;
        INC       N:_tpr+605         ;; 2 cycles
        BR        S:??magnet_log_logic_1  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//   46     }
//   47     else if(event_id == EVENT_ID_TOP_COVER_OCC)
??magnet_log_logic_0:
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        CMPW      AX, #0xFB          ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 3 cycles
//   48     {
//   49         tpr.top_cover.count++;
        INC       N:_tpr+593         ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
//   50     }
//   51     
//   52     /* increasing the event counters */
//   53     if(compart == COMPART_TRANSACTION)
??magnet_log_logic_1:
        MOV       A, [SP+0x05]       ;; 1 cycle
        CMP       A, #0x2            ;; 1 cycle
        BNZ       ??magnet_log_logic_2  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//   54     {
//   55         if(tpr.transaction_count >= tpr.transaction_entries)
        MOV       A, N:_tpr+12       ;; 1 cycle
        CMP       A, N:_tpr+14       ;; 1 cycle
        BC        ??magnet_log_logic_3  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//   56         {
//   57             tpr.transaction_count = 1;
        MOV       N:_tpr+12, #0x1    ;; 1 cycle
//   58             tpr.transaction_overflow = 1;
        MOV       N:_tpr+13, #0x1    ;; 1 cycle
        BR        S:??magnet_log_logic_4  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//   59         }
//   60         else
//   61         {
//   62             tpr.transaction_count++;
??magnet_log_logic_3:
        INC       N:_tpr+12          ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
//   63         }
//   64         seq_no_transaction++;
??magnet_log_logic_4:
        INCW      N:_seq_no_transaction  ;; 2 cycles
//   65         
//   66         address = COMPART_TRANSACTION_START_ADD + (tpr.transaction_count-1) * COMPART_TRANSACTION_SIZE;
        MOV       C, N:_COMPART_TRANSACTION_SIZE  ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       X, N:_tpr+12       ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        DECW      AX                 ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, N:_COMPART_TRANSACTION_START_ADD  ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        MOVW      HL, SP             ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        BR        N:??magnet_log_logic_5  ;; 3 cycles
        ; ------------------------------------- Block: 19 cycles
//   67     }
//   68     else if(compart == COMPART_POWERFAIL)
??magnet_log_logic_2:
        MOV       A, [SP+0x05]       ;; 1 cycle
        CMP       A, #0x1            ;; 1 cycle
        BNZ       ??magnet_log_logic_6  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//   69     {
//   70         if(tpr.power_count >= tpr.power_entries)
        MOV       A, N:_tpr+8        ;; 1 cycle
        CMP       A, N:_tpr+10       ;; 1 cycle
        BC        ??magnet_log_logic_7  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//   71         {
//   72             tpr.power_count = 1;
        MOV       N:_tpr+8, #0x1     ;; 1 cycle
//   73             tpr.power_overflow = 1;
        MOV       N:_tpr+9, #0x1     ;; 1 cycle
        BR        S:??magnet_log_logic_8  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//   74         }
//   75         else
//   76         {
//   77             tpr.power_count++;
??magnet_log_logic_7:
        INC       N:_tpr+8           ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
//   78         }
//   79         seq_no_powerfail++;
??magnet_log_logic_8:
        INCW      N:_seq_no_powerfail  ;; 2 cycles
//   80         
//   81         address = COMPART_POWERFAIL_START_ADD + (tpr.power_count-1) * COMPART_POWERFAIL_SIZE;
        MOV       C, N:_COMPART_POWERFAIL_SIZE  ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       X, N:_tpr+8        ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        DECW      AX                 ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, N:_COMPART_POWERFAIL_START_ADD  ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        MOVW      HL, SP             ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        BR        S:??magnet_log_logic_5  ;; 3 cycles
        ; ------------------------------------- Block: 19 cycles
//   82     }
//   83     else if(compart == COMPART_NONROLLOVER)
??magnet_log_logic_6:
        MOV       A, [SP+0x05]       ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        BNZ       ??magnet_log_logic_9  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//   84     {
//   85         if(tpr.non_roll_count >= tpr.non_roll_entries)
        MOV       A, N:_tpr+20       ;; 1 cycle
        CMP       A, N:_tpr+22       ;; 1 cycle
        BC        ??magnet_log_logic_10  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//   86         {
//   87             tpr.non_roll_count = 1;
        MOV       N:_tpr+20, #0x1    ;; 1 cycle
//   88             tpr.non_roll_overflow = 1;
        MOV       N:_tpr+21, #0x1    ;; 1 cycle
        BR        S:??magnet_log_logic_11  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//   89         }
//   90         else
//   91         {
//   92             tpr.non_roll_count++;
??magnet_log_logic_10:
        INC       N:_tpr+20          ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
//   93         }
//   94         seq_no_nonrollover++;
??magnet_log_logic_11:
        INCW      N:_seq_no_nonrollover  ;; 2 cycles
//   95         
//   96         address = COMPART_NONROLLOVER_START_ADD + (tpr.non_roll_count-1) * COMPART_NONROLLOVER_SIZE;
        MOV       C, N:_COMPART_NONROLLOVER_SIZE  ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       X, N:_tpr+20       ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        DECW      AX                 ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, N:_COMPART_NONROLLOVER_START_ADD  ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        MOVW      HL, SP             ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        BR        S:??magnet_log_logic_5  ;; 3 cycles
        ; ------------------------------------- Block: 19 cycles
//   97     }
//   98     else if(compart == COMPART_DEBUG)
??magnet_log_logic_9:
        MOV       A, [SP+0x05]       ;; 1 cycle
        CMP       A, #0x3            ;; 1 cycle
        BNZ       ??magnet_log_logic_5  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//   99     {
//  100         if(tpr.debug_count >= tpr.debug_entries)
        MOV       A, N:_tpr+24       ;; 1 cycle
        CMP       A, N:_tpr+26       ;; 1 cycle
        BC        ??magnet_log_logic_12  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  101         {
//  102             tpr.debug_count = 1;
        MOV       N:_tpr+24, #0x1    ;; 1 cycle
//  103             tpr.debug_overflow = 1;
        MOV       N:_tpr+25, #0x1    ;; 1 cycle
        BR        S:??magnet_log_logic_13  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  104         }
//  105         else
//  106         {
//  107             tpr.debug_count++;
??magnet_log_logic_12:
        INC       N:_tpr+24          ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
//  108         }
//  109         seq_no_debug++;
??magnet_log_logic_13:
        INCW      N:_seq_no_debug    ;; 2 cycles
//  110         
//  111         address = COMPART_DEBUG_START_ADD + (tpr.debug_count-1) * COMPART_DEBUG_SIZE;
        MOV       X, N:_tpr+24       ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x10          ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #0x73F0        ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        MOVW      HL, SP             ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ; ------------------------------------- Block: 14 cycles
//  112     }
//  113     
//  114     if(compart == COMPART_POWERFAIL)
??magnet_log_logic_5:
        MOV       A, [SP+0x05]       ;; 1 cycle
        CMP       A, #0x1            ;; 1 cycle
        BNZ       ??magnet_log_logic_14  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  115     {
//  116         tpr.power_last = EVENT_ID_POWER_FAIL_RES;
        MOV       N:_tpr+11, #0x66   ;; 1 cycle
//  117         fill_oprzero(COMPART_POWERFAIL_SIZE);
        MOV       A, N:_COMPART_POWERFAIL_SIZE  ;; 1 cycle
          CFI FunCall _fill_oprzero
        CALL      _fill_oprzero      ;; 3 cycles
//  118         time_into_char_array5_sec(OffTime, &opr_data[0]);
        MOVW      HL, #LWRD(_OffTime)  ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+20
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      AX, #LWRD(_opr_data)  ;; 1 cycle
          CFI FunCall _time_into_char_array5_sec
        CALL      _time_into_char_array5_sec  ;; 3 cycles
//  119         int_into_char_array(seq_no_powerfail, &opr_data[5]);
        MOVW      BC, #LWRD(_opr_data+5)  ;; 1 cycle
        MOVW      AX, N:_seq_no_powerfail  ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  120         time_into_char_array5_sec(Now, &opr_data[7]);
        MOVW      HL, #LWRD(_Now)    ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+28
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      AX, #LWRD(_opr_data+7)  ;; 1 cycle
          CFI FunCall _time_into_char_array5_sec
        CALL      _time_into_char_array5_sec  ;; 3 cycles
//  121         eprom_write(address,0,COMPART_POWERFAIL_SIZE,PAGE_1,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+30
        MOV       B, #0x0            ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       A, N:_COMPART_POWERFAIL_SIZE  ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, [SP+0x12]      ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
        ADDW      SP, #0x12          ;; 1 cycle
          CFI CFA SP+12
        BR        N:??magnet_log_logic_15  ;; 3 cycles
        ; ------------------------------------- Block: 49 cycles
//  122     }
//  123     else if(compart == COMPART_TRANSACTION)
??magnet_log_logic_14:
        MOV       A, [SP+0x05]       ;; 1 cycle
        CMP       A, #0x2            ;; 1 cycle
        BNZ       ??magnet_log_logic_16  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  124     {
//  125         tpr.transaction_last = event_id;
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       N:_tpr+15, A       ;; 1 cycle
        XCH       A, X               ;; 1 cycle
//  126         fill_oprzero(COMPART_TRANSACTION_SIZE);
        MOV       A, N:_COMPART_TRANSACTION_SIZE  ;; 1 cycle
          CFI FunCall _fill_oprzero
        CALL      _fill_oprzero      ;; 3 cycles
//  127         time_into_char_array5_sec(Now, &opr_data[0]);   
        MOVW      HL, #LWRD(_Now)    ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+20
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      AX, #LWRD(_opr_data)  ;; 1 cycle
          CFI FunCall _time_into_char_array5_sec
        CALL      _time_into_char_array5_sec  ;; 3 cycles
//  128         opr_data[5] = event_id;
        MOVW      AX, [SP+0x0E]      ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       N:_opr_data+5, A   ;; 1 cycle
        XCH       A, X               ;; 1 cycle
//  129         int_into_char_array(seq_no_transaction, &opr_data[6]);
        MOVW      BC, #LWRD(_opr_data+6)  ;; 1 cycle
        MOVW      AX, N:_seq_no_transaction  ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  130         eprom_write(address,0,COMPART_TRANSACTION_SIZE,PAGE_1,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+22
        MOV       B, #0x0            ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       A, N:_COMPART_TRANSACTION_SIZE  ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, [SP+0x0A]      ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
        ADDW      SP, #0xA           ;; 1 cycle
          CFI CFA SP+12
        BR        N:??magnet_log_logic_15  ;; 3 cycles
        ; ------------------------------------- Block: 45 cycles
//  131     }
//  132     else if(compart == COMPART_NONROLLOVER)
??magnet_log_logic_16:
        MOV       A, [SP+0x05]       ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        BNZ       ??magnet_log_logic_17  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  133     {
//  134         tpr.non_roll_last = event_id;
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       N:_tpr+23, A       ;; 1 cycle
        XCH       A, X               ;; 1 cycle
//  135         fill_oprzero(COMPART_NONROLLOVER_SIZE);
        MOV       A, N:_COMPART_NONROLLOVER_SIZE  ;; 1 cycle
          CFI FunCall _fill_oprzero
        CALL      _fill_oprzero      ;; 3 cycles
//  136         if(event_id == EVENT_ID_TOP_COVER_OCC || event_id == EVENT_ID_TOP_COVER_RES)
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        CMPW      AX, #0xFB          ;; 1 cycle
        BZ        ??magnet_log_logic_18  ;; 4 cycles
        ; ------------------------------------- Block: 14 cycles
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        CMPW      AX, #0xFC          ;; 1 cycle
        BNZ       ??magnet_log_logic_19  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  137         {
//  138             time_into_char_array5_sec(tpr.top_cover.time, &opr_data[0]);
??magnet_log_logic_18:
        MOVW      HL, #LWRD(_tpr+594)  ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+20
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      AX, #LWRD(_opr_data)  ;; 1 cycle
          CFI FunCall _time_into_char_array5_sec
        CALL      _time_into_char_array5_sec  ;; 3 cycles
        ADDW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+12
        ; ------------------------------------- Block: 12 cycles
//  139         }
//  140         opr_data[5] = event_id;
??magnet_log_logic_19:
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       N:_opr_data+5, A   ;; 1 cycle
        XCH       A, X               ;; 1 cycle
//  141         int_into_char_array(seq_no_nonrollover, &opr_data[6]);
        MOVW      BC, #LWRD(_opr_data+6)  ;; 1 cycle
        MOVW      AX, N:_seq_no_nonrollover  ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  142         eprom_write(address,0,COMPART_NONROLLOVER_SIZE,PAGE_1,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+14
        MOV       B, #0x0            ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       A, N:_COMPART_NONROLLOVER_SIZE  ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, [SP+0x02]      ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+12
        BR        S:??magnet_log_logic_15  ;; 3 cycles
        ; ------------------------------------- Block: 26 cycles
//  143     }
//  144     else if(compart == COMPART_DEBUG)
??magnet_log_logic_17:
        MOV       A, [SP+0x05]       ;; 1 cycle
        CMP       A, #0x3            ;; 1 cycle
        BNZ       ??magnet_log_logic_15  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  145     {
//  146         tpr.debug_last = event_id;
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        MOVW      N:_tpr+28, AX      ;; 1 cycle
//  147         fill_oprzero(COMPART_DEBUG_SIZE);
        MOV       A, #0x10           ;; 1 cycle
          CFI FunCall _fill_oprzero
        CALL      _fill_oprzero      ;; 3 cycles
//  148         time_into_char_array5_sec(Now, &opr_data[0]);
        MOVW      HL, #LWRD(_Now)    ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+20
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      AX, #LWRD(_opr_data)  ;; 1 cycle
          CFI FunCall _time_into_char_array5_sec
        CALL      _time_into_char_array5_sec  ;; 3 cycles
//  149         int_into_char_array(event_id, &opr_data[5]);
        MOVW      BC, #LWRD(_opr_data+5)  ;; 1 cycle
        MOVW      AX, [SP+0x0E]      ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  150         int_into_char_array(seq_no_debug, &opr_data[7]);
        MOVW      BC, #LWRD(_opr_data+7)  ;; 1 cycle
        MOVW      AX, N:_seq_no_debug  ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  151         eprom_write(address,0,COMPART_DEBUG_SIZE,PAGE_1,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+22
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, [SP+0x0A]      ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
        ADDW      SP, #0xA           ;; 1 cycle
          CFI CFA SP+12
        ; ------------------------------------- Block: 37 cycles
//  152     }
//  153     
//  154     eprom_read(0x0CA0,0,PAGE_1,AUTO_CALC);
??magnet_log_logic_15:
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xCA0         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
//  155     opr_data[2] = tpr.power_last;
        MOV       A, N:_tpr+11       ;; 1 cycle
        MOV       N:_opr_data+2, A   ;; 1 cycle
//  156     opr_data[3] = tpr.transaction_last;
        MOV       A, N:_tpr+15       ;; 1 cycle
        MOV       N:_opr_data+3, A   ;; 1 cycle
//  157     opr_data[5] = tpr.non_roll_last;
        MOV       A, N:_tpr+23       ;; 1 cycle
        MOV       N:_opr_data+5, A   ;; 1 cycle
//  158     int_into_char_array(tpr.debug_last,&opr_data[6]);
        MOVW      BC, #LWRD(_opr_data+6)  ;; 1 cycle
        MOVW      AX, N:_tpr+28      ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  159     eprom_write(0x0CA0,0,16,PAGE_1,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+14
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xCA0         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  160     
//  161     /* updating sequence no in the memory */
//  162     eprom_read(ADD_SEQ_NO,0,PAGE_1,AUTO_CALC);
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xC00         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
//  163     int_into_char_array(seq_no_powerfail,&opr_data[4]);
        MOVW      BC, #LWRD(_opr_data+4)  ;; 1 cycle
        MOVW      AX, N:_seq_no_powerfail  ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  164     int_into_char_array(seq_no_transaction,&opr_data[6]);
        MOVW      BC, #LWRD(_opr_data+6)  ;; 1 cycle
        MOVW      AX, N:_seq_no_transaction  ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  165     int_into_char_array(seq_no_nonrollover,&opr_data[10]);
        MOVW      BC, #LWRD(_opr_data+10)  ;; 1 cycle
        MOVW      AX, N:_seq_no_nonrollover  ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  166     int_into_char_array(seq_no_debug,&opr_data[12]);
        MOVW      BC, #LWRD(_opr_data+12)  ;; 1 cycle
        MOVW      AX, N:_seq_no_debug  ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  167     eprom_write(ADD_SEQ_NO,0,16,PAGE_1,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+16
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xC00         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  168     
//  169     /* updating event counts in the memory */
//  170     eprom_read(0x0CB0,0,PAGE_2,AUTO_CALC);
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x1            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xCB0         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
//  171     int_into_char_array(tpr.cum_tpr_count,&opr_data[0]);
        MOVW      BC, #LWRD(_opr_data)  ;; 1 cycle
        MOVW      AX, N:_tpr+30      ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  172     opr_data[20] = tpr.power_fail.count;
        MOV       A, N:_tpr+605      ;; 1 cycle
        MOV       N:_opr_data+20, A  ;; 1 cycle
//  173 //    opr_data[21] = rtc_change;
//  174     opr_data[25] = tpr.top_cover.count;
        MOV       A, N:_tpr+593      ;; 1 cycle
        MOV       N:_opr_data+25, A  ;; 1 cycle
//  175     eprom_write(0x0CB0,0,32,PAGE_2,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+18
        MOV       B, #0x1            ;; 1 cycle
        MOVW      DE, #0x20          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xCB0         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  176     
//  177     eprom_read(0x0CE0,0,PAGE_1,AUTO_CALC);
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xCE0         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
//  178     opr_data[4] = tpr.power_count;
        MOV       A, N:_tpr+8        ;; 1 cycle
        MOV       N:_opr_data+4, A   ;; 1 cycle
//  179     opr_data[5] = tpr.power_overflow;
        MOV       A, N:_tpr+9        ;; 1 cycle
        MOV       N:_opr_data+5, A   ;; 1 cycle
//  180     opr_data[6] = tpr.transaction_count;
        MOV       A, N:_tpr+12       ;; 1 cycle
        MOV       N:_opr_data+6, A   ;; 1 cycle
//  181     opr_data[7] = tpr.transaction_overflow;
        MOV       A, N:_tpr+13       ;; 1 cycle
        MOV       N:_opr_data+7, A   ;; 1 cycle
//  182     opr_data[10] = tpr.non_roll_count;
        MOV       A, N:_tpr+20       ;; 1 cycle
        MOV       N:_opr_data+10, A  ;; 1 cycle
//  183     opr_data[11] = tpr.non_roll_overflow;
        MOV       A, N:_tpr+21       ;; 1 cycle
        MOV       N:_opr_data+11, A  ;; 1 cycle
//  184     opr_data[12] = tpr.debug_count;
        MOV       A, N:_tpr+24       ;; 1 cycle
        MOV       N:_opr_data+12, A  ;; 1 cycle
//  185     opr_data[13] = tpr.debug_overflow;
        MOV       A, N:_tpr+25       ;; 1 cycle
        MOV       N:_opr_data+13, A  ;; 1 cycle
//  186     
//  187     eprom_write(0x0CE0,0,16,PAGE_1,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+20
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xCE0         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  188 }
        ADDW      SP, #0x10          ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock0
        ; ------------------------------------- Block: 127 cycles
        ; ------------------------------------- Total: 506 cycles

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock1 Using cfiCommon0
          CFI Function _mem_log
        CODE
//  189 void mem_log(Compartment compart, us16 event_id, TAMPER_VAR tamper)
//  190 {
_mem_log:
        ; * Stack frame (at entry) *
        ; Param size: 28
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+6
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+8
        ; Auto size: 8
        SUBW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+12
//  191     us32 address;
//  192     
//  193     /* increasing the event counters */
//  194     if(event_id == EVENT_ID_MAGNET_OCC)
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        CMPW      AX, #0xC9          ;; 1 cycle
        BNZ       ??magnet_log_logic_20  ;; 4 cycles
        ; ------------------------------------- Block: 9 cycles
//  195     {
//  196         tpr.magnet.count++;
        INC       N:_tpr+33          ;; 2 cycles
//  197         tpr.cum_tpr_count++;
        INCW      N:_tpr+30          ;; 2 cycles
        BR        N:??magnet_log_logic_21  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
//  198     }
//  199     else if(event_id == EVENT_ID_VOL_MISS_R_OCC)
??magnet_log_logic_20:
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        CMPW      AX, #0x1           ;; 1 cycle
        BNZ       ??magnet_log_logic_22  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  200     {
//  201         tpr.vol_miss.Rph.count++;
        INC       N:_tpr+257         ;; 2 cycles
//  202         tpr.cum_tpr_count++;
        INCW      N:_tpr+30          ;; 2 cycles
        BR        N:??magnet_log_logic_21  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
//  203     }
//  204     else if(event_id == EVENT_ID_VOL_MISS_Y_OCC)
??magnet_log_logic_22:
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        CMPW      AX, #0x3           ;; 1 cycle
        BNZ       ??magnet_log_logic_23  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  205     {
//  206         tpr.vol_miss.Yph.count++;
        INC       N:_tpr+285         ;; 2 cycles
//  207         tpr.cum_tpr_count++;
        INCW      N:_tpr+30          ;; 2 cycles
        BR        N:??magnet_log_logic_21  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
//  208     }
//  209     else if(event_id == EVENT_ID_VOL_MISS_B_OCC)
??magnet_log_logic_23:
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        CMPW      AX, #0x5           ;; 1 cycle
        BNZ       ??magnet_log_logic_24  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  210     {
//  211         tpr.vol_miss.Bph.count++;
        INC       N:_tpr+313         ;; 2 cycles
//  212         tpr.cum_tpr_count++;
        INCW      N:_tpr+30          ;; 2 cycles
        BR        N:??magnet_log_logic_21  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
//  213     }
//  214     else if(event_id == EVENT_ID_VOL_MISS_ALL_OCC)
??magnet_log_logic_24:
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        CMPW      AX, #0x3E9         ;; 1 cycle
        BNZ       ??magnet_log_logic_25  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  215     {
//  216         tpr.vol_miss_all.count++;
        INC       N:_tpr+229         ;; 2 cycles
//  217         tpr.cum_tpr_count++;
        INCW      N:_tpr+30          ;; 2 cycles
        BR        N:??magnet_log_logic_21  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
//  218     }
//  219     else if(event_id == EVENT_ID_VOL_UNBAL_OCC)
??magnet_log_logic_25:
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        CMPW      AX, #0xB           ;; 1 cycle
        BNZ       ??magnet_log_logic_26  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  220     {
//  221         tpr.vol_unbal.count++;
        INC       N:_tpr+61          ;; 2 cycles
//  222         tpr.cum_tpr_count++;
        INCW      N:_tpr+30          ;; 2 cycles
        BR        N:??magnet_log_logic_21  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
//  223     }
//  224     else if(event_id == EVENT_ID_VOL_LOW_OCC)
??magnet_log_logic_26:
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        CMPW      AX, #0x9           ;; 1 cycle
        BNZ       ??magnet_log_logic_27  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  225     {
//  226         tpr.vol_low.count++;
        INC       N:_tpr+89          ;; 2 cycles
//  227         tpr.cum_tpr_count++;
        INCW      N:_tpr+30          ;; 2 cycles
        BR        N:??magnet_log_logic_21  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
//  228     }
//  229     else if(event_id == EVENT_ID_VOL_HIGH_OCC)
??magnet_log_logic_27:
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        CMPW      AX, #0x7           ;; 1 cycle
        BNZ       ??magnet_log_logic_28  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  230     {
//  231         tpr.vol_high.count++;
        INC       N:_tpr+117         ;; 2 cycles
//  232         tpr.cum_tpr_count++;
        INCW      N:_tpr+30          ;; 2 cycles
        BR        N:??magnet_log_logic_21  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
//  233     }
//  234     else if(event_id == EVENT_ID_CT_REV_R_OCC)
??magnet_log_logic_28:
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        CMPW      AX, #0x33          ;; 1 cycle
        BNZ       ??magnet_log_logic_29  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  235     {
//  236         tpr.ct_rev.Rph.count++;
        INC       N:_tpr+341         ;; 2 cycles
//  237         tpr.cum_tpr_count++;
        INCW      N:_tpr+30          ;; 2 cycles
        BR        N:??magnet_log_logic_21  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
//  238     }
//  239     else if(event_id == EVENT_ID_CT_REV_Y_OCC)
??magnet_log_logic_29:
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        CMPW      AX, #0x35          ;; 1 cycle
        BNZ       ??magnet_log_logic_30  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  240     {
//  241         tpr.ct_rev.Yph.count++;
        INC       N:_tpr+369         ;; 2 cycles
//  242         tpr.cum_tpr_count++;
        INCW      N:_tpr+30          ;; 2 cycles
        BR        N:??magnet_log_logic_21  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
//  243     }
//  244     else if(event_id == EVENT_ID_CT_REV_B_OCC)
??magnet_log_logic_30:
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        CMPW      AX, #0x37          ;; 1 cycle
        BNZ       ??magnet_log_logic_31  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  245     {
//  246         tpr.ct_rev.Bph.count++;
        INC       N:_tpr+397         ;; 2 cycles
//  247         tpr.cum_tpr_count++;
        INCW      N:_tpr+30          ;; 2 cycles
        BR        N:??magnet_log_logic_21  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
//  248     }
//  249     else if(event_id == EVENT_ID_CT_BYPASS_OCC)
??magnet_log_logic_31:
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        CMPW      AX, #0x41          ;; 1 cycle
        BNZ       ??magnet_log_logic_32  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  250     {
//  251         tpr.ct_bypass.count++;
        INC       N:_tpr+173         ;; 2 cycles
//  252         tpr.cum_tpr_count++;
        INCW      N:_tpr+30          ;; 2 cycles
        BR        S:??magnet_log_logic_21  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
//  253     }
//  254     else if(event_id == EVENT_ID_CT_OPEN_R_OCC)
??magnet_log_logic_32:
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        CMPW      AX, #0x39          ;; 1 cycle
        BNZ       ??magnet_log_logic_33  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  255     {
//  256         tpr.ct_open.Rph.count++;
        INC       N:_tpr+425         ;; 2 cycles
//  257         tpr.cum_tpr_count++;   
        INCW      N:_tpr+30          ;; 2 cycles
        BR        S:??magnet_log_logic_21  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
//  258     }
//  259     else if(event_id == EVENT_ID_CT_OPEN_Y_OCC)
??magnet_log_logic_33:
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        CMPW      AX, #0x3B          ;; 1 cycle
        BNZ       ??magnet_log_logic_34  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  260     {
//  261         tpr.ct_open.Yph.count++;   
        INC       N:_tpr+453         ;; 2 cycles
//  262         tpr.cum_tpr_count++;
        INCW      N:_tpr+30          ;; 2 cycles
        BR        S:??magnet_log_logic_21  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
//  263     }
//  264     else if(event_id == EVENT_ID_CT_OPEN_B_OCC)
??magnet_log_logic_34:
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        CMPW      AX, #0x3D          ;; 1 cycle
        BNZ       ??magnet_log_logic_35  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  265     {
//  266         tpr.ct_open.Bph.count++;  
        INC       N:_tpr+481         ;; 2 cycles
//  267         tpr.cum_tpr_count++; 
        INCW      N:_tpr+30          ;; 2 cycles
        BR        S:??magnet_log_logic_21  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
//  268     }
//  269     else if(event_id == EVENT_ID_CURR_UNBAL_OCC)
??magnet_log_logic_35:
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        CMPW      AX, #0x3F          ;; 1 cycle
        BNZ       ??magnet_log_logic_36  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  270     {
//  271         tpr.curr_unbal.count++;
        INC       N:_tpr+145         ;; 2 cycles
//  272         tpr.cum_tpr_count++;
        INCW      N:_tpr+30          ;; 2 cycles
        BR        S:??magnet_log_logic_21  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
//  273     }
//  274     else if(event_id == EVENT_ID_OVER_CURR_R_OCC)
??magnet_log_logic_36:
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        CMPW      AX, #0x53          ;; 1 cycle
        BNZ       ??magnet_log_logic_37  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  275     {
//  276         tpr.curr_over.Rph.count++;
        INC       N:_tpr+509         ;; 2 cycles
//  277         tpr.cum_tpr_count++;
        INCW      N:_tpr+30          ;; 2 cycles
        BR        S:??magnet_log_logic_21  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
//  278     }
//  279     else if(event_id == EVENT_ID_OVER_CURR_Y_OCC)
??magnet_log_logic_37:
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        CMPW      AX, #0x51          ;; 1 cycle
        BNZ       ??magnet_log_logic_38  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  280     {
//  281         tpr.curr_over.Yph.count++;
        INC       N:_tpr+537         ;; 2 cycles
//  282         tpr.cum_tpr_count++;
        INCW      N:_tpr+30          ;; 2 cycles
        BR        S:??magnet_log_logic_21  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
//  283     }
//  284     else if(event_id == EVENT_ID_OVER_CURR_B_OCC)
??magnet_log_logic_38:
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        CMPW      AX, #0x4F          ;; 1 cycle
        BNZ       ??magnet_log_logic_39  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  285     {
//  286         tpr.curr_over.Bph.count++;
        INC       N:_tpr+565         ;; 2 cycles
//  287         tpr.cum_tpr_count++;
        INCW      N:_tpr+30          ;; 2 cycles
        BR        S:??magnet_log_logic_21  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
//  288     }
//  289     else if(event_id == EVENT_ID_NEU_DISTURB_OCC)
??magnet_log_logic_39:
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        CMPW      AX, #0xCB          ;; 1 cycle
        BNZ       ??magnet_log_logic_21  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  290     {
//  291         tpr.neu_disturb.count++;
        INC       N:_tpr+201         ;; 2 cycles
//  292         tpr.cum_tpr_count++;
        INCW      N:_tpr+30          ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
//  293     }
//  294     
//  295     if(compart == COMPART_VOLTAGE)
??magnet_log_logic_21:
        MOV       A, [SP+0x05]       ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        BNZ       ??magnet_log_logic_40  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  296     {
//  297         if(tpr.vol_related_count >= tpr.vol_related_entries)
        MOV       A, N:_tpr          ;; 1 cycle
        CMP       A, N:_tpr+2        ;; 1 cycle
        BC        ??magnet_log_logic_41  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  298         {
//  299             tpr.vol_related_count = 1;
        MOV       N:_tpr, #0x1       ;; 1 cycle
//  300             tpr.vol_related_overflow = 1;
        MOV       N:_tpr+1, #0x1     ;; 1 cycle
        BR        S:??magnet_log_logic_42  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  301         }
//  302         else
//  303         {
//  304             tpr.vol_related_count++;
??magnet_log_logic_41:
        INC       N:_tpr             ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
//  305         }
//  306         seq_no_vol++;
??magnet_log_logic_42:
        INCW      N:_seq_no_vol      ;; 2 cycles
//  307         
//  308         address = COMPART_VOLTAGE_START_ADD + (tpr.vol_related_count-1) * COMPART_VOLTAGE_SIZE;
        MOV       C, N:_COMPART_VOLTAGE_SIZE  ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       X, N:_tpr          ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        DECW      AX                 ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, N:_COMPART_VOLTAGE_START_ADD  ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        MOVW      HL, SP             ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        BR        S:??magnet_log_logic_43  ;; 3 cycles
        ; ------------------------------------- Block: 19 cycles
//  309     }
//  310     else if(compart == COMPART_CURRENT)
??magnet_log_logic_40:
        MOV       A, [SP+0x05]       ;; 1 cycle
        CMP       A, #0x1            ;; 1 cycle
        BNZ       ??magnet_log_logic_44  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  311     {
//  312         if(tpr.curr_related_count >= tpr.curr_related_entries)
        MOV       A, N:_tpr+4        ;; 1 cycle
        CMP       A, N:_tpr+6        ;; 1 cycle
        BC        ??magnet_log_logic_45  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  313         {
//  314             tpr.curr_related_count = 1;
        MOV       N:_tpr+4, #0x1     ;; 1 cycle
//  315             tpr.curr_related_overflow = 1;
        MOV       N:_tpr+5, #0x1     ;; 1 cycle
        BR        S:??magnet_log_logic_46  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  316         }
//  317         else
//  318         {
//  319             tpr.curr_related_count++;
??magnet_log_logic_45:
        INC       N:_tpr+4           ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
//  320         }
//  321         seq_no_curr++;
??magnet_log_logic_46:
        INCW      N:_seq_no_curr     ;; 2 cycles
//  322    
//  323         address = COMPART_CURRENT_START_ADD + (tpr.curr_related_count-1) * COMPART_CURRENT_SIZE;
        MOV       C, N:_COMPART_CURRENT_SIZE  ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       X, N:_tpr+4        ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        DECW      AX                 ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, N:_COMPART_CURRENT_START_ADD  ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        MOVW      HL, SP             ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        BR        S:??magnet_log_logic_43  ;; 3 cycles
        ; ------------------------------------- Block: 19 cycles
//  324     }
//  325     else if(compart == COMPART_OTHERS)
??magnet_log_logic_44:
        MOV       A, [SP+0x05]       ;; 1 cycle
        CMP       A, #0x2            ;; 1 cycle
        BNZ       ??magnet_log_logic_43  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  326     {
//  327         if(tpr.others_count >= tpr.others_entries)
        MOV       A, N:_tpr+16       ;; 1 cycle
        CMP       A, N:_tpr+18       ;; 1 cycle
        BC        ??magnet_log_logic_47  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  328         {
//  329             tpr.others_count = 1;
        MOV       N:_tpr+16, #0x1    ;; 1 cycle
//  330             tpr.others_overflow = 1;
        MOV       N:_tpr+17, #0x1    ;; 1 cycle
        BR        S:??magnet_log_logic_48  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  331         }
//  332         else
//  333         {
//  334             tpr.others_count++;
??magnet_log_logic_47:
        INC       N:_tpr+16          ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
//  335         }
//  336         seq_no_others++;
??magnet_log_logic_48:
        INCW      N:_seq_no_others   ;; 2 cycles
//  337         
//  338         address = COMPART_OTHERS_START_ADD + (tpr.others_count-1) * COMPART_OTHERS_SIZE;
        MOV       C, N:_COMPART_OTHERS_SIZE  ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       X, N:_tpr+16       ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        DECW      AX                 ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, N:_COMPART_OTHERS_START_ADD  ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        MOVW      HL, SP             ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ; ------------------------------------- Block: 16 cycles
//  339     }
//  340     
//  341     /* filling up the array and writing to the memory */
//  342     if(compart == COMPART_VOLTAGE)
??magnet_log_logic_43:
        MOV       A, [SP+0x05]       ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        SKZ                          ;; 4 cycles
        BR        N:??magnet_log_logic_49  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  343     {
//  344         tpr.vol_related_last = event_id;
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       N:_tpr+3, A        ;; 1 cycle
        XCH       A, X               ;; 1 cycle
//  345         fill_oprzero(COMPART_VOLTAGE_SIZE);
        MOV       A, N:_COMPART_VOLTAGE_SIZE  ;; 1 cycle
          CFI FunCall _fill_oprzero
        CALL      _fill_oprzero      ;; 3 cycles
//  346         time_into_char_array5_sec(tamper.time, &opr_data[0]);
        XCHW      AX, HL             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0xE           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+20
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      AX, #LWRD(_opr_data)  ;; 1 cycle
          CFI FunCall _time_into_char_array5_sec
        CALL      _time_into_char_array5_sec  ;; 3 cycles
//  347         long_into_char_array4(tamper.kwh_import,&opr_data[5]);
        MOVW      DE, #LWRD(_opr_data+5)  ;; 1 cycle
        MOVW      AX, [SP+0x22]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x20]      ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  348         long_into_char_array4(tamper.kvah_import,&opr_data[9]);
        MOVW      DE, #LWRD(_opr_data+9)  ;; 1 cycle
        MOVW      AX, [SP+0x2A]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x28]      ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  349         int_into_char_array(vol.Rph.rms, &opr_data[13]);
        MOVW      BC, #LWRD(_opr_data+13)  ;; 1 cycle
        MOVW      AX, N:_vol         ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  350         int_into_char_array(vol.Yph.rms, &opr_data[15]);
        MOVW      BC, #LWRD(_opr_data+15)  ;; 1 cycle
        MOVW      AX, N:_vol+6       ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  351         int_into_char_array(vol.Bph.rms, &opr_data[17]);
        MOVW      BC, #LWRD(_opr_data+17)  ;; 1 cycle
        MOVW      AX, N:_vol+12      ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  352         int_into_char_array(curr.Rph.rms_signed/10,&opr_data[19]);
        MOVW      BC, #LWRD(_opr_data+19)  ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+22
        POP       DE                 ;; 1 cycle
          CFI CFA SP+20
        MOVW      AX, #0x0           ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+22
        MOVW      AX, #0xA           ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+24
        MOVW      BC, N:_curr+14     ;; 1 cycle
        MOVW      AX, N:_curr+12     ;; 1 cycle
          CFI FunCall ?SL_DIV_L03
        CALL      N:?SL_DIV_L03      ;; 3 cycles
        PUSH      DE                 ;; 1 cycle
          CFI CFA SP+26
        POP       BC                 ;; 1 cycle
          CFI CFA SP+24
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+20
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  353         int_into_char_array(curr.Yph.rms_signed/10,&opr_data[21]);
        MOVW      BC, #LWRD(_opr_data+21)  ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+22
        POP       DE                 ;; 1 cycle
          CFI CFA SP+20
        MOVW      AX, #0x0           ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+22
        MOVW      AX, #0xA           ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+24
        MOVW      BC, N:_curr+30     ;; 1 cycle
        MOVW      AX, N:_curr+28     ;; 1 cycle
          CFI FunCall ?SL_DIV_L03
        CALL      N:?SL_DIV_L03      ;; 3 cycles
        PUSH      DE                 ;; 1 cycle
          CFI CFA SP+26
        POP       BC                 ;; 1 cycle
          CFI CFA SP+24
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+20
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  354         int_into_char_array(curr.Bph.rms_signed/10,&opr_data[23]);
        MOVW      BC, #LWRD(_opr_data+23)  ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+22
        POP       DE                 ;; 1 cycle
          CFI CFA SP+20
        MOVW      AX, #0x0           ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+22
        MOVW      AX, #0xA           ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+24
        MOVW      BC, N:_curr+46     ;; 1 cycle
        MOVW      AX, N:_curr+44     ;; 1 cycle
          CFI FunCall ?SL_DIV_L03
        CALL      N:?SL_DIV_L03      ;; 3 cycles
        PUSH      DE                 ;; 1 cycle
          CFI CFA SP+26
        POP       BC                 ;; 1 cycle
          CFI CFA SP+24
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+20
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  355         int_into_char_array(curr.Nph.rms, &opr_data[25]);
        MOVW      BC, #LWRD(_opr_data+25)  ;; 1 cycle
        MOVW      HL, N:_curr+50     ;; 1 cycle
        MOVW      DE, N:_curr+48     ;; 1 cycle
        MOVW      AX, DE             ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  356         temp_s8 = pf.Rph_signed/10;
        MOVW      BC, #0xA           ;; 1 cycle
        MOVW      AX, N:_pf+8        ;; 1 cycle
          CFI FunCall ?SI_DIV_L02
        CALL      N:?SI_DIV_L02      ;; 3 cycles
        XCH       A, X               ;; 1 cycle
        MOV       S:_temp_s8, A      ;; 1 cycle
        XCH       A, X               ;; 1 cycle
//  357         opr_data[27] = temp_s8;
        MOV       A, S:_temp_s8      ;; 1 cycle
        MOV       N:_opr_data+27, A  ;; 1 cycle
//  358         
//  359         temp_s8 = pf.Yph_signed/10;
        MOVW      BC, #0xA           ;; 1 cycle
        MOVW      AX, N:_pf+10       ;; 1 cycle
          CFI FunCall ?SI_DIV_L02
        CALL      N:?SI_DIV_L02      ;; 3 cycles
        XCH       A, X               ;; 1 cycle
        MOV       S:_temp_s8, A      ;; 1 cycle
        XCH       A, X               ;; 1 cycle
//  360         opr_data[28] = temp_s8;
        MOV       A, S:_temp_s8      ;; 1 cycle
        MOV       N:_opr_data+28, A  ;; 1 cycle
//  361         
//  362         temp_s8 = pf.Bph_signed/10;
        MOVW      BC, #0xA           ;; 1 cycle
        MOVW      AX, N:_pf+12       ;; 1 cycle
          CFI FunCall ?SI_DIV_L02
        CALL      N:?SI_DIV_L02      ;; 3 cycles
        XCH       A, X               ;; 1 cycle
        MOV       S:_temp_s8, A      ;; 1 cycle
        XCH       A, X               ;; 1 cycle
//  363         opr_data[29] = temp_s8;
        MOV       A, S:_temp_s8      ;; 1 cycle
        MOV       N:_opr_data+29, A  ;; 1 cycle
//  364         
//  365         temp_s8 = pf.Net_signed/10;
        MOVW      BC, #0xA           ;; 1 cycle
        MOVW      AX, N:_pf+14       ;; 1 cycle
          CFI FunCall ?SI_DIV_L02
        CALL      N:?SI_DIV_L02      ;; 3 cycles
        XCH       A, X               ;; 1 cycle
        MOV       S:_temp_s8, A      ;; 1 cycle
        XCH       A, X               ;; 1 cycle
//  366         opr_data[30] = temp_s8;
        MOV       A, S:_temp_s8      ;; 1 cycle
        MOV       N:_opr_data+30, A  ;; 1 cycle
//  367         
//  368         temp_us16 = power.Allph.active_signed/100;
        MOVW      AX, #0x0           ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+22
        MOVW      AX, #0x64          ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+24
        MOVW      BC, N:_power+82    ;; 1 cycle
        MOVW      AX, N:_power+80    ;; 1 cycle
          CFI FunCall ?SL_DIV_L03
        CALL      N:?SL_DIV_L03      ;; 3 cycles
        MOVW      S:_temp_us16, AX   ;; 1 cycle
//  369         if(flag_Rph_active == EXPORT)
        MOVW      HL, #LWRD(_flag_quadrant)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        ADDW      SP, #0xC           ;; 1 cycle
          CFI CFA SP+12
        BNC       ??magnet_log_logic_50  ;; 4 cycles
        ; ------------------------------------- Block: 169 cycles
//  370         {
//  371             temp_us16 = ~temp_us16+1;
        MOVW      AX, S:_temp_us16   ;; 1 cycle
        XOR       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        XOR       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        INCW      AX                 ;; 1 cycle
        MOVW      S:_temp_us16, AX   ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
//  372         }
//  373         int_into_char_array(temp_us16,&opr_data[31]);
??magnet_log_logic_50:
        MOVW      BC, #LWRD(_opr_data+31)  ;; 1 cycle
        MOVW      AX, S:_temp_us16   ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  374         
//  375         long_into_char_array4(tamper.kwh_export,&opr_data[33]);
        MOVW      DE, #LWRD(_opr_data+33)  ;; 1 cycle
        MOVW      AX, [SP+0x1E]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x1C]      ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  376         long_into_char_array4(tamper.kvah_export,&opr_data[37]);
        MOVW      DE, #LWRD(_opr_data+37)  ;; 1 cycle
        MOVW      AX, [SP+0x26]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x24]      ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  377         
//  378         opr_data[41] = event_id;
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       N:_opr_data+41, A  ;; 1 cycle
        XCH       A, X               ;; 1 cycle
//  379         int_into_char_array(seq_no_vol, &opr_data[42]); /* can be removed lateron if space problem*/
        MOVW      BC, #LWRD(_opr_data+42)  ;; 1 cycle
        MOVW      AX, N:_seq_no_vol  ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  380         
//  381         eprom_write(address,0,COMPART_VOLTAGE_SIZE,PAGE_3,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+14
        MOV       B, #0x2            ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       A, N:_COMPART_VOLTAGE_SIZE  ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, [SP+0x02]      ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+12
        BR        N:??magnet_log_logic_51  ;; 3 cycles
        ; ------------------------------------- Block: 45 cycles
//  382     }
//  383     else if(compart == COMPART_CURRENT)
??magnet_log_logic_49:
        MOV       A, [SP+0x05]       ;; 1 cycle
        CMP       A, #0x1            ;; 1 cycle
        SKZ                          ;; 4 cycles
        BR        N:??magnet_log_logic_52  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  384     {
//  385         tpr.current_related_last = event_id;
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       N:_tpr+7, A        ;; 1 cycle
        XCH       A, X               ;; 1 cycle
//  386         fill_oprzero(COMPART_CURRENT_SIZE);
        MOV       A, N:_COMPART_CURRENT_SIZE  ;; 1 cycle
          CFI FunCall _fill_oprzero
        CALL      _fill_oprzero      ;; 3 cycles
//  387         time_into_char_array5_sec(tamper.time, &opr_data[0]);
        XCHW      AX, HL             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0xE           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+20
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      AX, #LWRD(_opr_data)  ;; 1 cycle
          CFI FunCall _time_into_char_array5_sec
        CALL      _time_into_char_array5_sec  ;; 3 cycles
//  388         long_into_char_array4(tamper.kwh_import,&opr_data[5]);
        MOVW      DE, #LWRD(_opr_data+5)  ;; 1 cycle
        MOVW      AX, [SP+0x22]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x20]      ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  389         long_into_char_array4(tamper.kvah_import,&opr_data[9]);
        MOVW      DE, #LWRD(_opr_data+9)  ;; 1 cycle
        MOVW      AX, [SP+0x2A]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x28]      ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  390         int_into_char_array(vol.Rph.rms, &opr_data[13]);
        MOVW      BC, #LWRD(_opr_data+13)  ;; 1 cycle
        MOVW      AX, N:_vol         ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  391         int_into_char_array(vol.Yph.rms, &opr_data[15]);
        MOVW      BC, #LWRD(_opr_data+15)  ;; 1 cycle
        MOVW      AX, N:_vol+6       ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  392         int_into_char_array(vol.Bph.rms, &opr_data[17]);
        MOVW      BC, #LWRD(_opr_data+17)  ;; 1 cycle
        MOVW      AX, N:_vol+12      ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  393         int_into_char_array(curr.Rph.rms_signed/10,&opr_data[19]);
        MOVW      BC, #LWRD(_opr_data+19)  ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+22
        POP       DE                 ;; 1 cycle
          CFI CFA SP+20
        MOVW      AX, #0x0           ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+22
        MOVW      AX, #0xA           ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+24
        MOVW      BC, N:_curr+14     ;; 1 cycle
        MOVW      AX, N:_curr+12     ;; 1 cycle
          CFI FunCall ?SL_DIV_L03
        CALL      N:?SL_DIV_L03      ;; 3 cycles
        PUSH      DE                 ;; 1 cycle
          CFI CFA SP+26
        POP       BC                 ;; 1 cycle
          CFI CFA SP+24
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+20
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  394         int_into_char_array(curr.Yph.rms_signed/10,&opr_data[21]);
        MOVW      BC, #LWRD(_opr_data+21)  ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+22
        POP       DE                 ;; 1 cycle
          CFI CFA SP+20
        MOVW      AX, #0x0           ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+22
        MOVW      AX, #0xA           ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+24
        MOVW      BC, N:_curr+30     ;; 1 cycle
        MOVW      AX, N:_curr+28     ;; 1 cycle
          CFI FunCall ?SL_DIV_L03
        CALL      N:?SL_DIV_L03      ;; 3 cycles
        PUSH      DE                 ;; 1 cycle
          CFI CFA SP+26
        POP       BC                 ;; 1 cycle
          CFI CFA SP+24
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+20
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  395         int_into_char_array(curr.Bph.rms_signed/10,&opr_data[23]);
        MOVW      BC, #LWRD(_opr_data+23)  ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+22
        POP       DE                 ;; 1 cycle
          CFI CFA SP+20
        MOVW      AX, #0x0           ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+22
        MOVW      AX, #0xA           ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+24
        MOVW      BC, N:_curr+46     ;; 1 cycle
        MOVW      AX, N:_curr+44     ;; 1 cycle
          CFI FunCall ?SL_DIV_L03
        CALL      N:?SL_DIV_L03      ;; 3 cycles
        PUSH      DE                 ;; 1 cycle
          CFI CFA SP+26
        POP       BC                 ;; 1 cycle
          CFI CFA SP+24
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+20
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  396         int_into_char_array(curr.Nph.rms, &opr_data[25]);
        MOVW      BC, #LWRD(_opr_data+25)  ;; 1 cycle
        MOVW      HL, N:_curr+50     ;; 1 cycle
        MOVW      DE, N:_curr+48     ;; 1 cycle
        MOVW      AX, DE             ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  397         temp_s8 = pf.Rph_signed/10;
        MOVW      BC, #0xA           ;; 1 cycle
        MOVW      AX, N:_pf+8        ;; 1 cycle
          CFI FunCall ?SI_DIV_L02
        CALL      N:?SI_DIV_L02      ;; 3 cycles
        XCH       A, X               ;; 1 cycle
        MOV       S:_temp_s8, A      ;; 1 cycle
        XCH       A, X               ;; 1 cycle
//  398         opr_data[27] = temp_s8;
        MOV       A, S:_temp_s8      ;; 1 cycle
        MOV       N:_opr_data+27, A  ;; 1 cycle
//  399         
//  400         temp_s8 = pf.Yph_signed/10;
        MOVW      BC, #0xA           ;; 1 cycle
        MOVW      AX, N:_pf+10       ;; 1 cycle
          CFI FunCall ?SI_DIV_L02
        CALL      N:?SI_DIV_L02      ;; 3 cycles
        XCH       A, X               ;; 1 cycle
        MOV       S:_temp_s8, A      ;; 1 cycle
        XCH       A, X               ;; 1 cycle
//  401         opr_data[28] = temp_s8;
        MOV       A, S:_temp_s8      ;; 1 cycle
        MOV       N:_opr_data+28, A  ;; 1 cycle
//  402         
//  403         temp_s8 = pf.Bph_signed/10;
        MOVW      BC, #0xA           ;; 1 cycle
        MOVW      AX, N:_pf+12       ;; 1 cycle
          CFI FunCall ?SI_DIV_L02
        CALL      N:?SI_DIV_L02      ;; 3 cycles
        XCH       A, X               ;; 1 cycle
        MOV       S:_temp_s8, A      ;; 1 cycle
        XCH       A, X               ;; 1 cycle
//  404         opr_data[29] = temp_s8;
        MOV       A, S:_temp_s8      ;; 1 cycle
        MOV       N:_opr_data+29, A  ;; 1 cycle
//  405         
//  406         temp_s8 = pf.Net_signed/10;
        MOVW      BC, #0xA           ;; 1 cycle
        MOVW      AX, N:_pf+14       ;; 1 cycle
          CFI FunCall ?SI_DIV_L02
        CALL      N:?SI_DIV_L02      ;; 3 cycles
        XCH       A, X               ;; 1 cycle
        MOV       S:_temp_s8, A      ;; 1 cycle
        XCH       A, X               ;; 1 cycle
//  407         opr_data[30] = temp_s8;
        MOV       A, S:_temp_s8      ;; 1 cycle
        MOV       N:_opr_data+30, A  ;; 1 cycle
//  408         
//  409         temp_us16 = power.Allph.active_signed/100;
        MOVW      AX, #0x0           ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+22
        MOVW      AX, #0x64          ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+24
        MOVW      BC, N:_power+82    ;; 1 cycle
        MOVW      AX, N:_power+80    ;; 1 cycle
          CFI FunCall ?SL_DIV_L03
        CALL      N:?SL_DIV_L03      ;; 3 cycles
        MOVW      S:_temp_us16, AX   ;; 1 cycle
//  410         if(flag_Rph_active == EXPORT)
        MOVW      HL, #LWRD(_flag_quadrant)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        ADDW      SP, #0xC           ;; 1 cycle
          CFI CFA SP+12
        BNC       ??magnet_log_logic_53  ;; 4 cycles
        ; ------------------------------------- Block: 169 cycles
//  411         {
//  412             temp_us16 = ~temp_us16+1;
        MOVW      AX, S:_temp_us16   ;; 1 cycle
        XOR       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        XOR       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        INCW      AX                 ;; 1 cycle
        MOVW      S:_temp_us16, AX   ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
//  413         }
//  414         int_into_char_array(temp_us16,&opr_data[31]);
??magnet_log_logic_53:
        MOVW      BC, #LWRD(_opr_data+31)  ;; 1 cycle
        MOVW      AX, S:_temp_us16   ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  415         
//  416         long_into_char_array4(tamper.kwh_export,&opr_data[33]);
        MOVW      DE, #LWRD(_opr_data+33)  ;; 1 cycle
        MOVW      AX, [SP+0x1E]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x1C]      ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  417         long_into_char_array4(tamper.kvah_export,&opr_data[37]);
        MOVW      DE, #LWRD(_opr_data+37)  ;; 1 cycle
        MOVW      AX, [SP+0x26]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x24]      ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  418         
//  419         opr_data[41] = event_id;
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       N:_opr_data+41, A  ;; 1 cycle
        XCH       A, X               ;; 1 cycle
//  420         int_into_char_array(seq_no_vol, &opr_data[42]); /* can be removed lateron if space problem*/
        MOVW      BC, #LWRD(_opr_data+42)  ;; 1 cycle
        MOVW      AX, N:_seq_no_vol  ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  421         
//  422         eprom_write(address,0,COMPART_CURRENT_SIZE,PAGE_3,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+14
        MOV       B, #0x2            ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       A, N:_COMPART_CURRENT_SIZE  ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, [SP+0x02]      ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+12
        BR        N:??magnet_log_logic_51  ;; 3 cycles
        ; ------------------------------------- Block: 45 cycles
//  423     }
//  424     else if(compart == COMPART_OTHERS)
??magnet_log_logic_52:
        MOV       A, [SP+0x05]       ;; 1 cycle
        CMP       A, #0x2            ;; 1 cycle
        SKZ                          ;; 4 cycles
        BR        N:??magnet_log_logic_51  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  425     {
//  426         tpr.others_last = event_id;
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       N:_tpr+19, A       ;; 1 cycle
        XCH       A, X               ;; 1 cycle
//  427         fill_oprzero(COMPART_OTHERS_SIZE);
        MOV       A, N:_COMPART_OTHERS_SIZE  ;; 1 cycle
          CFI FunCall _fill_oprzero
        CALL      _fill_oprzero      ;; 3 cycles
//  428         
//  429         time_into_char_array5_sec(tamper.time, &opr_data[0]);
        XCHW      AX, HL             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0xE           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+20
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      AX, #LWRD(_opr_data)  ;; 1 cycle
          CFI FunCall _time_into_char_array5_sec
        CALL      _time_into_char_array5_sec  ;; 3 cycles
//  430         
//  431         if(event_id == EVENT_ID_MAGNET_OCC || event_id == EVENT_ID_MAGNET_RES)
        MOVW      AX, [SP+0x0E]      ;; 1 cycle
        ADDW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+12
        CMPW      AX, #0xC9          ;; 1 cycle
        BZ        ??magnet_log_logic_54  ;; 4 cycles
        ; ------------------------------------- Block: 29 cycles
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        CMPW      AX, #0xCA          ;; 1 cycle
        BNZ       ??magnet_log_logic_55  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  432         {
//  433             long_into_char_array4(energy.Allph.active_imp,&opr_data[5]);
??magnet_log_logic_54:
        MOVW      DE, #LWRD(_opr_data+5)  ;; 1 cycle
        MOVW      BC, N:_energy+54   ;; 1 cycle
        MOVW      AX, N:_energy+52   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  434             long_into_char_array4(energy.Allph.apparent_imp,&opr_data[9]);
        MOVW      DE, #LWRD(_opr_data+9)  ;; 1 cycle
        MOVW      BC, N:_energy+70   ;; 1 cycle
        MOVW      AX, N:_energy+68   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
        BR        S:??magnet_log_logic_56  ;; 3 cycles
        ; ------------------------------------- Block: 15 cycles
//  435         }
//  436         else
//  437         {
//  438             long_into_char_array4(tamper.kwh_import,&opr_data[5]);
??magnet_log_logic_55:
        MOVW      DE, #LWRD(_opr_data+5)  ;; 1 cycle
        MOVW      AX, [SP+0x1A]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x18]      ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  439             long_into_char_array4(tamper.kvah_import,&opr_data[9]);
        MOVW      DE, #LWRD(_opr_data+9)  ;; 1 cycle
        MOVW      AX, [SP+0x22]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x20]      ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
        ; ------------------------------------- Block: 14 cycles
//  440         }
//  441         
//  442         int_into_char_array(vol.Rph.rms, &opr_data[13]);
??magnet_log_logic_56:
        MOVW      BC, #LWRD(_opr_data+13)  ;; 1 cycle
        MOVW      AX, N:_vol         ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  443         int_into_char_array(vol.Yph.rms, &opr_data[15]);
        MOVW      BC, #LWRD(_opr_data+15)  ;; 1 cycle
        MOVW      AX, N:_vol+6       ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  444         int_into_char_array(vol.Bph.rms, &opr_data[17]);
        MOVW      BC, #LWRD(_opr_data+17)  ;; 1 cycle
        MOVW      AX, N:_vol+12      ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  445         int_into_char_array(curr.Rph.rms_signed/10,&opr_data[19]);
        MOVW      BC, #LWRD(_opr_data+19)  ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+14
        POP       DE                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      AX, #0x0           ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+14
        MOVW      AX, #0xA           ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+16
        MOVW      BC, N:_curr+14     ;; 1 cycle
        MOVW      AX, N:_curr+12     ;; 1 cycle
          CFI FunCall ?SL_DIV_L03
        CALL      N:?SL_DIV_L03      ;; 3 cycles
        PUSH      DE                 ;; 1 cycle
          CFI CFA SP+18
        POP       BC                 ;; 1 cycle
          CFI CFA SP+16
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+12
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  446         int_into_char_array(curr.Yph.rms_signed/10,&opr_data[21]);
        MOVW      BC, #LWRD(_opr_data+21)  ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+14
        POP       DE                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      AX, #0x0           ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+14
        MOVW      AX, #0xA           ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+16
        MOVW      BC, N:_curr+30     ;; 1 cycle
        MOVW      AX, N:_curr+28     ;; 1 cycle
          CFI FunCall ?SL_DIV_L03
        CALL      N:?SL_DIV_L03      ;; 3 cycles
        PUSH      DE                 ;; 1 cycle
          CFI CFA SP+18
        POP       BC                 ;; 1 cycle
          CFI CFA SP+16
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+12
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  447         int_into_char_array(curr.Bph.rms_signed/10,&opr_data[23]);
        MOVW      BC, #LWRD(_opr_data+23)  ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+14
        POP       DE                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      AX, #0x0           ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+14
        MOVW      AX, #0xA           ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+16
        MOVW      BC, N:_curr+46     ;; 1 cycle
        MOVW      AX, N:_curr+44     ;; 1 cycle
          CFI FunCall ?SL_DIV_L03
        CALL      N:?SL_DIV_L03      ;; 3 cycles
        PUSH      DE                 ;; 1 cycle
          CFI CFA SP+18
        POP       BC                 ;; 1 cycle
          CFI CFA SP+16
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+12
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  448         int_into_char_array(curr.Nph.rms, &opr_data[25]);
        MOVW      BC, #LWRD(_opr_data+25)  ;; 1 cycle
        MOVW      HL, N:_curr+50     ;; 1 cycle
        MOVW      DE, N:_curr+48     ;; 1 cycle
        MOVW      AX, DE             ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  449         temp_s8 = pf.Rph_signed/10;
        MOVW      BC, #0xA           ;; 1 cycle
        MOVW      AX, N:_pf+8        ;; 1 cycle
          CFI FunCall ?SI_DIV_L02
        CALL      N:?SI_DIV_L02      ;; 3 cycles
        XCH       A, X               ;; 1 cycle
        MOV       S:_temp_s8, A      ;; 1 cycle
        XCH       A, X               ;; 1 cycle
//  450         opr_data[27] = temp_s8;
        MOV       A, S:_temp_s8      ;; 1 cycle
        MOV       N:_opr_data+27, A  ;; 1 cycle
//  451         
//  452         temp_s8 = pf.Yph_signed/10;
        MOVW      BC, #0xA           ;; 1 cycle
        MOVW      AX, N:_pf+10       ;; 1 cycle
          CFI FunCall ?SI_DIV_L02
        CALL      N:?SI_DIV_L02      ;; 3 cycles
        XCH       A, X               ;; 1 cycle
        MOV       S:_temp_s8, A      ;; 1 cycle
        XCH       A, X               ;; 1 cycle
//  453         opr_data[28] = temp_s8;
        MOV       A, S:_temp_s8      ;; 1 cycle
        MOV       N:_opr_data+28, A  ;; 1 cycle
//  454         
//  455         temp_s8 = pf.Bph_signed/10;
        MOVW      BC, #0xA           ;; 1 cycle
        MOVW      AX, N:_pf+12       ;; 1 cycle
          CFI FunCall ?SI_DIV_L02
        CALL      N:?SI_DIV_L02      ;; 3 cycles
        XCH       A, X               ;; 1 cycle
        MOV       S:_temp_s8, A      ;; 1 cycle
        XCH       A, X               ;; 1 cycle
//  456         opr_data[29] = temp_s8;
        MOV       A, S:_temp_s8      ;; 1 cycle
        MOV       N:_opr_data+29, A  ;; 1 cycle
//  457         
//  458         temp_s8 = pf.Net_signed/10;
        MOVW      BC, #0xA           ;; 1 cycle
        MOVW      AX, N:_pf+14       ;; 1 cycle
          CFI FunCall ?SI_DIV_L02
        CALL      N:?SI_DIV_L02      ;; 3 cycles
        XCH       A, X               ;; 1 cycle
        MOV       S:_temp_s8, A      ;; 1 cycle
        XCH       A, X               ;; 1 cycle
//  459         opr_data[30] = temp_s8;
        MOV       A, S:_temp_s8      ;; 1 cycle
        MOV       N:_opr_data+30, A  ;; 1 cycle
//  460         
//  461         temp_us16 = power.Allph.active_signed/100;
        MOVW      AX, #0x0           ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+14
        MOVW      AX, #0x64          ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+16
        MOVW      BC, N:_power+82    ;; 1 cycle
        MOVW      AX, N:_power+80    ;; 1 cycle
          CFI FunCall ?SL_DIV_L03
        CALL      N:?SL_DIV_L03      ;; 3 cycles
        MOVW      S:_temp_us16, AX   ;; 1 cycle
//  462         if(flag_Rph_active == EXPORT)
        MOVW      HL, #LWRD(_flag_quadrant)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+12
        BNC       ??magnet_log_logic_57  ;; 4 cycles
        ; ------------------------------------- Block: 133 cycles
//  463         {
//  464             temp_us16 = ~temp_us16+1;
        MOVW      AX, S:_temp_us16   ;; 1 cycle
        XOR       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        XOR       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        INCW      AX                 ;; 1 cycle
        MOVW      S:_temp_us16, AX   ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
//  465         }
//  466         int_into_char_array(temp_us16,&opr_data[31]);
??magnet_log_logic_57:
        MOVW      BC, #LWRD(_opr_data+31)  ;; 1 cycle
        MOVW      AX, S:_temp_us16   ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  467         
//  468         if(event_id == EVENT_ID_MAGNET_OCC || event_id == EVENT_ID_MAGNET_RES)
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        CMPW      AX, #0xC9          ;; 1 cycle
        BZ        ??magnet_log_logic_58  ;; 4 cycles
        ; ------------------------------------- Block: 11 cycles
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        CMPW      AX, #0xCA          ;; 1 cycle
        BNZ       ??magnet_log_logic_59  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  469         {
//  470             long_into_char_array4(energy.Allph.active_exp,&opr_data[33]);
??magnet_log_logic_58:
        MOVW      DE, #LWRD(_opr_data+33)  ;; 1 cycle
        MOVW      BC, N:_energy+58   ;; 1 cycle
        MOVW      AX, N:_energy+56   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  471             long_into_char_array4(energy.Allph.apparent_exp,&opr_data[37]); 
        MOVW      DE, #LWRD(_opr_data+37)  ;; 1 cycle
        MOVW      BC, N:_energy+74   ;; 1 cycle
        MOVW      AX, N:_energy+72   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
        BR        S:??magnet_log_logic_60  ;; 3 cycles
        ; ------------------------------------- Block: 15 cycles
//  472         }
//  473         else
//  474         {
//  475             long_into_char_array4(tamper.kwh_export,&opr_data[33]);
??magnet_log_logic_59:
        MOVW      DE, #LWRD(_opr_data+33)  ;; 1 cycle
        MOVW      AX, [SP+0x1E]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x1C]      ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  476             long_into_char_array4(tamper.kvah_export,&opr_data[37]);
        MOVW      DE, #LWRD(_opr_data+37)  ;; 1 cycle
        MOVW      AX, [SP+0x26]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x24]      ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
        ; ------------------------------------- Block: 14 cycles
//  477         }
//  478         
//  479         opr_data[41] = event_id;
??magnet_log_logic_60:
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       N:_opr_data+41, A  ;; 1 cycle
        XCH       A, X               ;; 1 cycle
//  480         int_into_char_array(seq_no_vol, &opr_data[42]); /* can be removed lateron if space problem*/
        MOVW      BC, #LWRD(_opr_data+42)  ;; 1 cycle
        MOVW      AX, N:_seq_no_vol  ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  481         
//  482         eprom_write(address,0,COMPART_OTHERS_SIZE,PAGE_3,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+14
        MOV       B, #0x2            ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       A, N:_COMPART_OTHERS_SIZE  ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, [SP+0x02]      ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+12
        ; ------------------------------------- Block: 23 cycles
//  483     }
//  484     eprom_read(0x0CA0,0,PAGE_1,AUTO_CALC);
??magnet_log_logic_51:
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xCA0         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
//  485     opr_data[0] = tpr.vol_related_last;
        MOV       A, N:_tpr+3        ;; 1 cycle
        MOV       N:_opr_data, A     ;; 1 cycle
//  486     opr_data[1] = tpr.current_related_last;
        MOV       A, N:_tpr+7        ;; 1 cycle
        MOV       N:_opr_data+1, A   ;; 1 cycle
//  487     opr_data[4] = tpr.others_last;
        MOV       A, N:_tpr+19       ;; 1 cycle
        MOV       N:_opr_data+4, A   ;; 1 cycle
//  488     eprom_write(0x0CA0,0,16,PAGE_1,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+14
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xCA0         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  489     
//  490     /* updating sequence no in the memory */
//  491     eprom_read(ADD_SEQ_NO,0,PAGE_1,AUTO_CALC);
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xC00         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
//  492     int_into_char_array(seq_no_vol,&opr_data[0]);
        MOVW      BC, #LWRD(_opr_data)  ;; 1 cycle
        MOVW      AX, N:_seq_no_vol  ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  493     int_into_char_array(seq_no_curr,&opr_data[2]);
        MOVW      BC, #LWRD(_opr_data+2)  ;; 1 cycle
        MOVW      AX, N:_seq_no_curr  ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  494     int_into_char_array(seq_no_others,&opr_data[8]);
        MOVW      BC, #LWRD(_opr_data+8)  ;; 1 cycle
        MOVW      AX, N:_seq_no_others  ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  495     eprom_write(ADD_SEQ_NO,0,16,PAGE_1,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+16
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xC00         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  496     
//  497     /* updating event counts in the memory */
//  498     eprom_read(0x0CB0,0,PAGE_2,AUTO_CALC);
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x1            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xCB0         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
//  499     int_into_char_array(tpr.cum_tpr_count,&opr_data[0]);
        MOVW      BC, #LWRD(_opr_data)  ;; 1 cycle
        MOVW      AX, N:_tpr+30      ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  500     opr_data[2] = tpr.vol_miss.Rph.count;
        MOV       A, N:_tpr+257      ;; 1 cycle
        MOV       N:_opr_data+2, A   ;; 1 cycle
//  501     opr_data[3] = tpr.vol_miss.Yph.count;
        MOV       A, N:_tpr+285      ;; 1 cycle
        MOV       N:_opr_data+3, A   ;; 1 cycle
//  502     opr_data[4] = tpr.vol_miss.Bph.count;
        MOV       A, N:_tpr+313      ;; 1 cycle
        MOV       N:_opr_data+4, A   ;; 1 cycle
//  503     opr_data[5] = tpr.vol_miss_all.count;
        MOV       A, N:_tpr+229      ;; 1 cycle
        MOV       N:_opr_data+5, A   ;; 1 cycle
//  504     opr_data[6] = tpr.vol_unbal.count;
        MOV       A, N:_tpr+61       ;; 1 cycle
        MOV       N:_opr_data+6, A   ;; 1 cycle
//  505     opr_data[7] = tpr.vol_low.count;
        MOV       A, N:_tpr+89       ;; 1 cycle
        MOV       N:_opr_data+7, A   ;; 1 cycle
//  506     opr_data[8] = tpr.vol_high.count;
        MOV       A, N:_tpr+117      ;; 1 cycle
        MOV       N:_opr_data+8, A   ;; 1 cycle
//  507     opr_data[9] = tpr.curr_unbal.count;
        MOV       A, N:_tpr+145      ;; 1 cycle
        MOV       N:_opr_data+9, A   ;; 1 cycle
//  508     opr_data[10] = tpr.ct_bypass.count;
        MOV       A, N:_tpr+173      ;; 1 cycle
        MOV       N:_opr_data+10, A  ;; 1 cycle
//  509     opr_data[11] = tpr.ct_rev.Rph.count;
        MOV       A, N:_tpr+341      ;; 1 cycle
        MOV       N:_opr_data+11, A  ;; 1 cycle
//  510     opr_data[12] = tpr.ct_rev.Yph.count;
        MOV       A, N:_tpr+369      ;; 1 cycle
        MOV       N:_opr_data+12, A  ;; 1 cycle
//  511     opr_data[13] = tpr.ct_rev.Bph.count;
        MOV       A, N:_tpr+397      ;; 1 cycle
        MOV       N:_opr_data+13, A  ;; 1 cycle
//  512     opr_data[14] = tpr.ct_open.Rph.count;
        MOV       A, N:_tpr+425      ;; 1 cycle
        MOV       N:_opr_data+14, A  ;; 1 cycle
//  513     opr_data[15] = tpr.ct_open.Yph.count;
        MOV       A, N:_tpr+453      ;; 1 cycle
        MOV       N:_opr_data+15, A  ;; 1 cycle
//  514     opr_data[16] = tpr.ct_open.Bph.count;
        MOV       A, N:_tpr+481      ;; 1 cycle
        MOV       N:_opr_data+16, A  ;; 1 cycle
//  515     opr_data[17] = tpr.curr_over.Rph.count;
        MOV       A, N:_tpr+509      ;; 1 cycle
        MOV       N:_opr_data+17, A  ;; 1 cycle
//  516     opr_data[18] = tpr.curr_over.Yph.count;
        MOV       A, N:_tpr+537      ;; 1 cycle
        MOV       N:_opr_data+18, A  ;; 1 cycle
//  517     opr_data[19] = tpr.curr_over.Bph.count;
        MOV       A, N:_tpr+565      ;; 1 cycle
        MOV       N:_opr_data+19, A  ;; 1 cycle
//  518     opr_data[22] = tpr.neu_disturb.count;
        MOV       A, N:_tpr+201      ;; 1 cycle
        MOV       N:_opr_data+22, A  ;; 1 cycle
//  519     opr_data[23] = tpr.magnet.count;
        MOV       A, N:_tpr+33       ;; 1 cycle
        MOV       N:_opr_data+23, A  ;; 1 cycle
//  520 //    opr_data[24] = low_pf;
//  521     eprom_write(0x0CB0,0,32,PAGE_2,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+18
        MOV       B, #0x1            ;; 1 cycle
        MOVW      DE, #0x20          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xCB0         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  522     
//  523     eprom_read(0x0CE0,0,PAGE_1,AUTO_CALC);
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xCE0         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
//  524     opr_data[0] = tpr.vol_related_count;
        MOV       A, N:_tpr          ;; 1 cycle
        MOV       N:_opr_data, A     ;; 1 cycle
//  525     opr_data[1] = tpr.vol_related_overflow;
        MOV       A, N:_tpr+1        ;; 1 cycle
        MOV       N:_opr_data+1, A   ;; 1 cycle
//  526     opr_data[2] = tpr.curr_related_count;
        MOV       A, N:_tpr+4        ;; 1 cycle
        MOV       N:_opr_data+2, A   ;; 1 cycle
//  527     opr_data[3] = tpr.curr_related_overflow;
        MOV       A, N:_tpr+5        ;; 1 cycle
        MOV       N:_opr_data+3, A   ;; 1 cycle
//  528     opr_data[8] = tpr.others_count;
        MOV       A, N:_tpr+16       ;; 1 cycle
        MOV       N:_opr_data+8, A   ;; 1 cycle
//  529     opr_data[9] = tpr.others_overflow;
        MOV       A, N:_tpr+17       ;; 1 cycle
        MOV       N:_opr_data+9, A   ;; 1 cycle
//  530     eprom_write(0x0CE0,0,16,PAGE_1,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+20
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xCE0         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  531 }
        ADDW      SP, #0x10          ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock1
        ; ------------------------------------- Block: 149 cycles
        ; ------------------------------------- Total: 1253 cycles
//  532 
//  533 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock2 Using cfiCommon1
          CFI Function _power_on_event
        CODE
//  534 void power_on_event(void)
//  535 {
_power_on_event:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  536     if(flag_power_off == 1)
        MOVW      HL, #LWRD(_flag_tamper2)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??magnet_log_logic_61  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  537     {
//  538         if(OnTime.min != OffTime.min)
        MOV       A, N:_OnTime+1     ;; 1 cycle
        CMP       A, N:_OffTime+1    ;; 1 cycle
        BZ        ??magnet_log_logic_62  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  539         {
//  540             /* calculating power off seconds */
//  541             power_off_min += get_minute_diff(OnTime,OffTime);
        MOVW      HL, #LWRD(_OffTime)  ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+12
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      HL, #LWRD(_OnTime)  ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+20
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
          CFI FunCall _get_minute_diff
        CALL      _get_minute_diff   ;; 3 cycles
        MOVW      HL, N:_power_off_min+2  ;; 1 cycle
        MOVW      DE, N:_power_off_min  ;; 1 cycle
        ADDW      AX, DE             ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_power_off_min, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_power_off_min+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  542             
//  543             /* logging power fail event */
//  544             mem_log1(COMPART_POWERFAIL,EVENT_ID_POWER_FAIL_OCC);
        MOVW      BC, #0x65          ;; 1 cycle
        MOV       A, #0x1            ;; 1 cycle
          CFI FunCall _mem_log1
        CALL      _mem_log1          ;; 3 cycles
        ADDW      SP, #0x10          ;; 1 cycle
          CFI CFA SP+4
        ; ------------------------------------- Block: 38 cycles
//  545         } 
//  546         /* saving the time information */
//  547         flag_power_off = 0;
??magnet_log_logic_62:
        CLR1      N:_flag_tamper2.0  ;; 2 cycles
//  548         save_pom();
          CFI FunCall _save_pom
        CALL      _save_pom          ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  549     }
//  550 }
??magnet_log_logic_61:
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock2
        ; ------------------------------------- Block: 6 cycles
        ; ------------------------------------- Total: 61 cycles
//  551 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock3 Using cfiCommon1
          CFI Function _tamper_ram_init
        CODE
//  552 void tamper_ram_init()
//  553 {
_tamper_ram_init:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  554   /* thresholds */
//  555   //thr.magnet = THR_MAGNET_MT;
//  556   
//  557   thr.vol_miss_vol_occ = THR_VOL_MISS_OCC_VOL_PERCENT * (VREF / 100);  
        MOVW      AX, #0x1C20        ;; 1 cycle
        MOVW      N:_thr+2, AX       ;; 1 cycle
//  558   thr.vol_miss_curr_occ = THR_VOL_MISS_OCC_CURR_PERCENT * (IB / 100); 
        MOVW      AX, #0xC8          ;; 1 cycle
        MOVW      N:_thr+8, AX       ;; 1 cycle
//  559   thr.vol_miss_vol_res = THR_VOL_MISS_RES_VOL_PERCENT * (VREF / 100); 
        MOVW      AX, #0x1C20        ;; 1 cycle
        MOVW      N:_thr+4, AX       ;; 1 cycle
//  560   thr.vol_miss_curr_res = THR_VOL_MISS_RES_CURR_PERCENT * (IB / 100); 
        MOVW      AX, #0xC8          ;; 1 cycle
        MOVW      N:_thr+10, AX      ;; 1 cycle
//  561   thr.vol_miss_vol_any = THR_VOL_MISS_ANY_PHASE_VOL_PERCENT * (VREF / 100); 
        MOVW      AX, #0x3840        ;; 1 cycle
        MOVW      N:_thr+6, AX       ;; 1 cycle
//  562   
//  563   thr.vol_miss_all_vol_occ = THR_VOL_MISS_ALL_OCC_VOL_PERCENT * (VREF / 100);
        MOVW      AX, #0x2580        ;; 1 cycle
        MOVW      N:_thr+12, AX      ;; 1 cycle
//  564   thr.vol_miss_all_vol_res = THR_VOL_MISS_ALL_RES_VOL_PERCENT * (VREF / 100);
        MOVW      AX, #0x2EE0        ;; 1 cycle
        MOVW      N:_thr+14, AX      ;; 1 cycle
//  565   
//  566 //  thr.vol_unbal_vol_occ = THR_VOL_UNBAL_OCC_VOL_PERCENT * (VREF / 100);  
//  567 //  thr.vol_unbal_vol_res = THR_VOL_UNBAL_RES_VOL_PERCENT * (VREF / 100);  
//  568   thr.vol_unbal_curr_occ = THR_VOL_UNBAL_OCC_CURR_PERCENT * (IB / 100);  
        MOVW      AX, #0x3E8         ;; 1 cycle
        MOVW      N:_thr+20, AX      ;; 1 cycle
//  569   thr.vol_unbal_curr_res = THR_VOL_UNBAL_RES_CURR_PERCENT * (IB / 100);  
        MOVW      AX, #0x3E8         ;; 1 cycle
        MOVW      N:_thr+22, AX      ;; 1 cycle
//  570   thr.vol_unbal_all_phase_vol = THR_VOL_UNBAL_ALL_PHASE_VOL_PERCENT * (VREF / 100);  
        MOVW      AX, #0x3840        ;; 1 cycle
        MOVW      N:_thr+24, AX      ;; 1 cycle
//  571   
//  572   thr.vol_low_vol_occ =  THR_VOL_LOW_OCC_PERCENT * (VREF / 100);  
        MOVW      AX, #0x4650        ;; 1 cycle
        MOVW      N:_thr+26, AX      ;; 1 cycle
//  573   thr.vol_low_vol_res = THR_VOL_LOW_RES_PERCENT * (VREF / 100);  
        MOVW      AX, #0x4650        ;; 1 cycle
        MOVW      N:_thr+28, AX      ;; 1 cycle
//  574   thr.vol_high_vol_occ = THR_VOL_HIGH_OCC_PERCENT * (VREF / 100);  
        MOVW      AX, #0x6BD0        ;; 1 cycle
        MOVW      N:_thr+30, AX      ;; 1 cycle
//  575   thr.vol_high_vol_res = THR_VOL_HIGH_RES_PERCENT * (VREF / 100);  
        MOVW      AX, #0x6BD0        ;; 1 cycle
        MOVW      N:_thr+32, AX      ;; 1 cycle
//  576   thr.vol_high_vol_any_phase = THR_VOL_HIGH_ANY_PHASE_VOL_PERCENT * (VREF / 100);
        MOVW      AX, #0x3840        ;; 1 cycle
        MOVW      N:_thr+34, AX      ;; 1 cycle
//  577   
//  578   thr.ct_rev_curr_occ = THR_CT_REV_OCC_CURR_PERCENT * (IB / 100);
        MOVW      AX, #0x1F4         ;; 1 cycle
        MOVW      N:_thr+36, AX      ;; 1 cycle
//  579   thr.ct_rev_curr_res = THR_CT_REV_RES_CURR_PERCENT * (IB / 100);
        MOVW      AX, #0x3E8         ;; 1 cycle
        MOVW      N:_thr+38, AX      ;; 1 cycle
//  580   thr.ct_rev_pf_limit = THR_CT_REV_PF_PERCENT * 10;
        MOVW      AX, #0xC8          ;; 1 cycle
        MOVW      N:_thr+50, AX      ;; 1 cycle
//  581   thr.ct_rev_vol_limit = THR_CT_REV_VOL_LOWER_PERCENT * (VREF / 100);
        MOVW      AX, #0x3840        ;; 1 cycle
        MOVW      N:_thr+40, AX      ;; 1 cycle
//  582   
//  583 //  thr.ct_open_curr_occ = THR_CT_OPEN_OCC_CURR_PERCENT * (IB / 100);
//  584 //  thr.ct_open_curr_res = THR_CT_OPEN_RES_CURR_PERCENT * (IB / 100);
//  585   thr.ct_open_res_curr_res = THR_CT_OPEN_RES_CURR_RES_PERCENT * (IB / 100);
        MOVW      AX, #0x3E8         ;; 1 cycle
        MOVW      N:_thr+48, AX      ;; 1 cycle
//  586   thr.ct_open_res_curr_occ = THR_CT_OPEN_RES_CURR_OCC_PERCENT * (IB / 100);
        MOVW      AX, #0x3E8         ;; 1 cycle
        MOVW      N:_thr+46, AX      ;; 1 cycle
//  587   
//  588   thr.ct_bypass_residual_occ = THR_CT_BYPASS_RESIDUAL_OCC_PERCENT * (IB / 100);
        MOVW      AX, #0x7D0         ;; 1 cycle
        MOVW      N:_thr+52, AX      ;; 1 cycle
//  589   thr.ct_bypass_residual_res = THR_CT_BYPASS_RESIDUAL_RES_PERCENT * (IB / 100);
        MOVW      AX, #0x3E8         ;; 1 cycle
        MOVW      N:_thr+54, AX      ;; 1 cycle
//  590   thr.ct_bypass_all_phase_curr = THR_CT_BYPASS_ALL_PHASE_CURR_PERCENT * (IB / 100);  
        MOVW      AX, #0x3E8         ;; 1 cycle
        MOVW      N:_thr+56, AX      ;; 1 cycle
//  591   thr.ct_bypass_avg_current =  THR_CT_BYPASS_AVG_CURRENT;
        MOVW      AX, #0x64          ;; 1 cycle
        MOVW      N:_thr+58, AX      ;; 1 cycle
//  592   
//  593   thr.curr_over_curr_occ = ((us32)THR_CURR_OVER_OCC_PERCENT * (IMAX / 100));
        MOVW      AX, #0x1940        ;; 1 cycle
        MOVW      N:_thr+66, AX      ;; 1 cycle
        MOVW      AX, #0x1           ;; 1 cycle
        MOVW      N:_thr+68, AX      ;; 1 cycle
//  594   thr.curr_over_curr_res = ((us32)THR_CURR_OVER_RES_PERCENT * (IMAX / 100));
        MOVW      AX, #0x1940        ;; 1 cycle
        MOVW      N:_thr+70, AX      ;; 1 cycle
        MOVW      AX, #0x1           ;; 1 cycle
        MOVW      N:_thr+72, AX      ;; 1 cycle
//  595   thr.curr_over_all_phase_vol = THR_CURR_OVER_ALL_PHASE_VOL * (VREF / 100);
        MOVW      AX, #0x41A0        ;; 1 cycle
        MOVW      N:_thr+74, AX      ;; 1 cycle
//  596   
//  597 //  thr.curr_unbal.curr_occ = THR_CURR_UNBAL_CURR_OCC_PERCENT * (IB / 100);
//  598 //  thr.curr_unbal.curr_res = THR_CURR_UNBAL_CURR_RES_PERCENT * (IB / 100);
//  599   thr.curr_unbal.curr_neu_limit = THR_CURR_UNBAL_CURR_OCC_NEU_PERCENT * (IB /100);
        MOVW      AX, #0x7D0         ;; 1 cycle
        MOVW      N:_thr+84, AX      ;; 1 cycle
//  600   
//  601   thr.neu_dis_vol_occ = THR_NEU_DISTURB_VOL_OCC_PERCENT * (VREF / 100);
        MOVW      AX, #0x7530        ;; 1 cycle
        MOVW      N:_thr+60, AX      ;; 1 cycle
//  602   thr.neu_dis_vol_res = THR_NEU_DISTURB_VOL_RES_PERCENT * (VREF / 100);
        MOVW      AX, #0x7530        ;; 1 cycle
        MOVW      N:_thr+62, AX      ;; 1 cycle
//  603   thr.neu_dis_vol_any_phase = THR_NEU_DISTURB_VOL_ANY_PHASE_PERCENT * (VREF / 100);
        MOVW      AX, #0x3840        ;; 1 cycle
        MOVW      N:_thr+64, AX      ;; 1 cycle
//  604   
//  605   thr.sel_ref_max_vol=0; //default writing 0,but it should be configurable.
        MOV       N:_thr+76, #0x0    ;; 1 cycle
//  606   if(thr.sel_ref_max_vol > 2)
        MOV       A, N:_thr+76       ;; 1 cycle
        CMP       A, #0x3            ;; 1 cycle
        SKC                          ;; 1 cycle
        ; ------------------------------------- Block: 72 cycles
//  607   {
//  608       thr.sel_ref_max_vol = 0;
        MOV       N:_thr+76, #0x0    ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
//  609   }
//  610   
//  611   thr.ct_open_curr_occ=0;
??tamper_ram_init_0:
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_thr+42, AX      ;; 1 cycle
//  612   if(thr.ct_open_curr_occ > 2)
        MOVW      AX, N:_thr+42      ;; 1 cycle
        CMPW      AX, #0x3           ;; 1 cycle
        BC        ??magnet_log_logic_63  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
//  613   {
//  614     thr.ct_open_curr_occ = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_thr+42, AX      ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
//  615   }
//  616   
//  617   thr.ct_open_curr_res=0;
??magnet_log_logic_63:
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_thr+44, AX      ;; 1 cycle
//  618   if(thr.ct_open_curr_res > 2)
        MOVW      AX, N:_thr+44      ;; 1 cycle
        CMPW      AX, #0x3           ;; 1 cycle
        BC        ??magnet_log_logic_64  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
//  619   {
//  620     thr.ct_open_curr_res = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_thr+44, AX      ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
//  621   }
//  622   
//  623   /* Tamper time */
//  624   time.magnet_store = TIME_MAGNET_OCC;
??magnet_log_logic_64:
        MOVW      AX, #0xF           ;; 1 cycle
        MOVW      N:_time+14, AX     ;; 1 cycle
//  625   time.magnet_restore = TIME_MAGNET_RES;
        MOVW      AX, #0xF           ;; 1 cycle
        MOVW      N:_time+16, AX     ;; 1 cycle
//  626   time.magnet_indicate = TIME_MAGNET_INDI;
        MOV       N:_time, #0x1      ;; 1 cycle
//  627   
//  628   time.vol_miss_store = TIME_VOL_MISS_OCC;
        MOVW      AX, #0x3C          ;; 1 cycle
        MOVW      N:_time+18, AX     ;; 1 cycle
//  629   time.vol_miss_restore = TIME_VOL_MISS_RES;
        MOVW      AX, #0x3C          ;; 1 cycle
        MOVW      N:_time+20, AX     ;; 1 cycle
//  630   time.vol_miss_indicate = TIME_VOL_MISS_INDI;
        MOV       N:_time+1, #0x2    ;; 1 cycle
//  631   
//  632   time.vol_miss_all_store = TIME_VOL_MISS_ALL_OCC;
        MOVW      AX, #0x3C          ;; 1 cycle
        MOVW      N:_time+22, AX     ;; 1 cycle
//  633   time.vol_miss_all_restore = TIME_VOL_MISS_ALL_RES;
        MOVW      AX, #0x3C          ;; 1 cycle
        MOVW      N:_time+24, AX     ;; 1 cycle
//  634   time.vol_miss_all_indicate = TIME_VOL_MISS_ALL_INDI;
        MOV       N:_time+2, #0x2    ;; 1 cycle
//  635   
//  636   time.vol_unbal_store = TIME_VOL_UNBAL_OCC;
        MOVW      AX, #0x3C          ;; 1 cycle
        MOVW      N:_time+26, AX     ;; 1 cycle
//  637   time.vol_unbal_restore = TIME_VOL_UNBAL_RES;
        MOVW      AX, #0x3C          ;; 1 cycle
        MOVW      N:_time+28, AX     ;; 1 cycle
//  638   time.vol_unbal_indicate = TIME_VOL_UNBAL_INDI;
        MOV       N:_time+3, #0x2    ;; 1 cycle
//  639   
//  640   time.vol_low_store = TIME_VOL_LOW_OCC;
        MOVW      AX, #0x3C          ;; 1 cycle
        MOVW      N:_time+30, AX     ;; 1 cycle
//  641   time.vol_low_restore = TIME_VOL_LOW_RES;
        MOVW      AX, #0x3C          ;; 1 cycle
        MOVW      N:_time+32, AX     ;; 1 cycle
//  642   time.vol_low_indicate = TIME_VOL_LOW_INDI;
        MOV       N:_time+4, #0x2    ;; 1 cycle
//  643   
//  644   time.vol_high_store = TIME_VOL_HIGH_OCC;
        MOVW      AX, #0x3C          ;; 1 cycle
        MOVW      N:_time+34, AX     ;; 1 cycle
//  645   time.vol_high_restore = TIME_VOL_HIGH_RES;
        MOVW      AX, #0x3C          ;; 1 cycle
        MOVW      N:_time+36, AX     ;; 1 cycle
//  646   time.vol_high_indicate = TIME_VOL_HIGH_INDI;
        MOV       N:_time+5, #0x2    ;; 1 cycle
//  647   
//  648   time.ct_rev_store = TIME_CT_REV_OCC;
        MOVW      AX, #0x3C          ;; 1 cycle
        MOVW      N:_time+38, AX     ;; 1 cycle
//  649   time.ct_rev_restore = TIME_CT_REV_RES;
        MOVW      AX, #0x3C          ;; 1 cycle
        MOVW      N:_time+40, AX     ;; 1 cycle
//  650   time.ct_rev_indicate = TIME_CT_REV_INDI;
        MOV       N:_time+6, #0x2    ;; 1 cycle
//  651   
//  652   time.ct_open_store = TIME_CT_OPEN_OCC;
        MOVW      AX, #0x3C          ;; 1 cycle
        MOVW      N:_time+42, AX     ;; 1 cycle
//  653   time.ct_open_restore = TIME_CT_OPEN_RES;
        MOVW      AX, #0x3C          ;; 1 cycle
        MOVW      N:_time+44, AX     ;; 1 cycle
//  654   time.ct_open_indicate = TIME_CT_OPEN_INDI;
        MOV       N:_time+7, #0x2    ;; 1 cycle
//  655   
//  656   time.ct_bypass_store = TIME_CT_BYPASS_OCC;
        MOVW      AX, #0x3C          ;; 1 cycle
        MOVW      N:_time+46, AX     ;; 1 cycle
//  657   time.ct_bypass_restore = TIME_CT_BYPASS_RES;
        MOVW      AX, #0x3C          ;; 1 cycle
        MOVW      N:_time+48, AX     ;; 1 cycle
//  658   time.ct_bypass_indicate = TIME_CT_BYPASS_INDI;
        MOV       N:_time+8, #0x2    ;; 1 cycle
//  659   
//  660   time.curr_over_store = TIME_CURR_UNBAL_OCC;
        MOVW      AX, #0x3C          ;; 1 cycle
        MOVW      N:_time+50, AX     ;; 1 cycle
//  661   time.curr_unbal_restore = TIME_CURR_UNBAL_RES;
        MOVW      AX, #0x3C          ;; 1 cycle
        MOVW      N:_time+56, AX     ;; 1 cycle
//  662   time.curr_unbal_indicate = TIME_CURR_UNBAL_INDI;
        MOV       N:_time+10, #0x2   ;; 1 cycle
//  663   
//  664   time.curr_over_store = TIME_CURR_OVER_OCC;
        MOVW      AX, #0x3C          ;; 1 cycle
        MOVW      N:_time+50, AX     ;; 1 cycle
//  665   time.curr_over_restore = TIME_CURR_OVER_RES;
        MOVW      AX, #0x3C          ;; 1 cycle
        MOVW      N:_time+52, AX     ;; 1 cycle
//  666   time.curr_over_indicate = TIME_CURR_OVER_INDI;
        MOV       N:_time+9, #0x2    ;; 1 cycle
//  667   
//  668   time.nd_store = TIME_NEU_DISTURB_OCC;
        MOVW      AX, #0x3C          ;; 1 cycle
        MOVW      N:_time+58, AX     ;; 1 cycle
//  669   time.nd_restore = TIME_NEU_DISTURB_RES;
        MOVW      AX, #0x3C          ;; 1 cycle
        MOVW      N:_time+60, AX     ;; 1 cycle
//  670   time.nd_indicate = TIME_NEU_DISTURB_INDI;
        MOV       N:_time+11, #0x2   ;; 1 cycle
//  671   
//  672   time.top_cover_store = TIME_TOP_COVER_OCC;
        MOVW      AX, #0x3           ;; 1 cycle
        MOVW      N:_time+62, AX     ;; 1 cycle
//  673   time.top_cover_restore = TIME_TOP_COVER_RES;
        MOVW      AX, #0x5           ;; 1 cycle
        MOVW      N:_time+64, AX     ;; 1 cycle
//  674   time.top_cover_indicate = TIME_TOP_COVER_INDI;
        MOV       N:_time+12, #0x3   ;; 1 cycle
//  675   
//  676   read_tamper_variables();
          CFI FunCall _read_tamper_variables
        CALL      _read_tamper_variables  ;; 3 cycles
//  677   /* temp initialisation */
//  678   tamper_sel = (TPR_MAGNET | TPR_TOP_COVER | TPR_BATTERY_LOW); 
        MOVW      AX, #0x3001        ;; 1 cycle
        MOVW      N:_tamper_sel, AX  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_tamper_sel+2, AX  ;; 1 cycle
//  679   
//  680   if(eprom_read(MEM_FAIL_ADDR_EEP0,0,PAGE_1,AUTO_CALC) == EEP_OK)
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xB50         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
        CMP       A, #0x1            ;; 1 cycle
        BNZ       ??magnet_log_logic_65  ;; 4 cycles
        ; ------------------------------------- Block: 84 cycles
//  681   {
//  682     if(opr_data[6] == 0 || opr_data[6] == 1)
        CMP0      N:_opr_data+6      ;; 1 cycle
        BZ        ??magnet_log_logic_66  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        CMP       N:_opr_data+6, #0x1  ;; 1 cycle
        BNZ       ??magnet_log_logic_67  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  683     {
//  684       flag_eep_fail = opr_data[6];
??magnet_log_logic_66:
        MOVW      HL, #LWRD(_opr_data+6)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        MOVW      HL, #LWRD(_flag_eeprom_error)  ;; 1 cycle
        MOV1      [HL].0, CY         ;; 2 cycles
        BR        S:??magnet_log_logic_65  ;; 3 cycles
        ; ------------------------------------- Block: 8 cycles
//  685     }
//  686     else
//  687     {
//  688       flag_eep_fail = 0;
??magnet_log_logic_67:
        CLR1      N:_flag_eeprom_error.0  ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
//  689     }
//  690   }
//  691   if(eprom_read(MEM_FAIL_ADDR_EEP0+0x10,0,PAGE_1,AUTO_CALC) == EEP_OK)
??magnet_log_logic_65:
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xB60         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
        CMP       A, #0x1            ;; 1 cycle
        BNZ       ??magnet_log_logic_68  ;; 4 cycles
        ; ------------------------------------- Block: 12 cycles
//  692   {
//  693     if(opr_data[6] == 0 || opr_data[6] == 1)
        CMP0      N:_opr_data+6      ;; 1 cycle
        BZ        ??magnet_log_logic_69  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        CMP       N:_opr_data+6, #0x1  ;; 1 cycle
        BNZ       ??magnet_log_logic_70  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  694     {
//  695       flag_eep2_fail = opr_data[6];
??magnet_log_logic_69:
        MOVW      HL, #LWRD(_opr_data+6)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        MOVW      HL, #LWRD(_flag_eeprom_error)  ;; 1 cycle
        MOV1      [HL].6, CY         ;; 2 cycles
        BR        S:??magnet_log_logic_68  ;; 3 cycles
        ; ------------------------------------- Block: 8 cycles
//  696     }
//  697     else
//  698     {
//  699       flag_eep2_fail = 0;
??magnet_log_logic_70:
        CLR1      N:_flag_eeprom_error.6  ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
//  700     }
//  701   }
//  702   if(eprom_read(MEM_FAIL_ADDR_EEP2+0x10,2,PAGE_1,AUTO_CALC) == EEP_OK)
??magnet_log_logic_68:
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x2            ;; 1 cycle
        MOVW      AX, #0xFFC0        ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
        CMP       A, #0x1            ;; 1 cycle
        BNZ       ??magnet_log_logic_71  ;; 4 cycles
        ; ------------------------------------- Block: 12 cycles
//  703   {
//  704     if(opr_data[6] == 0 || opr_data[6] == 1)
        CMP0      N:_opr_data+6      ;; 1 cycle
        BZ        ??magnet_log_logic_72  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        CMP       N:_opr_data+6, #0x1  ;; 1 cycle
        BNZ       ??magnet_log_logic_73  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  705     {
//  706       flag_eep0_fail = opr_data[6];
??magnet_log_logic_72:
        MOVW      HL, #LWRD(_opr_data+6)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        MOVW      HL, #LWRD(_flag_eeprom_error)  ;; 1 cycle
        MOV1      [HL].2, CY         ;; 2 cycles
        BR        S:??magnet_log_logic_71  ;; 3 cycles
        ; ------------------------------------- Block: 8 cycles
//  707     }
//  708     else
//  709     {
//  710       flag_eep0_fail = 0;
??magnet_log_logic_73:
        CLR1      N:_flag_eeprom_error.2  ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
//  711     }
//  712   }
//  713   if(eprom_read(MEM_FAIL_ADDR_EEP2+0x20,2,PAGE_1,AUTO_CALC) == EEP_OK)
??magnet_log_logic_71:
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x2            ;; 1 cycle
        MOVW      AX, #0xFFD0        ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
        CMP       A, #0x1            ;; 1 cycle
        BNZ       ??magnet_log_logic_74  ;; 4 cycles
        ; ------------------------------------- Block: 12 cycles
//  714   {
//  715     if(opr_data[6] == 0 || opr_data[6] == 1)
        CMP0      N:_opr_data+6      ;; 1 cycle
        BZ        ??magnet_log_logic_75  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        CMP       N:_opr_data+6, #0x1  ;; 1 cycle
        BNZ       ??magnet_log_logic_76  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  716     {
//  717       flag_eep1_fail = opr_data[6];
??magnet_log_logic_75:
        MOVW      HL, #LWRD(_opr_data+6)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        MOVW      HL, #LWRD(_flag_eeprom_error)  ;; 1 cycle
        MOV1      [HL].4, CY         ;; 2 cycles
        RET                          ;; 6 cycles
        ; ------------------------------------- Block: 11 cycles
//  718     }
//  719     else
//  720     {
//  721       flag_eep1_fail = 0;
??magnet_log_logic_76:
        CLR1      N:_flag_eeprom_error.4  ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
//  722     }
//  723   }
//  724 
//  725 }
??magnet_log_logic_74:
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock3
        ; ------------------------------------- Block: 6 cycles
        ; ------------------------------------- Total: 302 cycles

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock4 Using cfiCommon1
          CFI Function _tamper_function_100ms
          CFI NoCalls
        CODE
//  726 void tamper_function_100ms()
//  727 {
_tamper_function_100ms:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  728   
//  729 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock4
        ; ------------------------------------- Block: 6 cycles
        ; ------------------------------------- Total: 6 cycles

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock5 Using cfiCommon1
          CFI Function _tamper_function_1sec
        CODE
//  730 void tamper_function_1sec()
//  731 {
_tamper_function_1sec:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 4
        SUBW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
//  732   Condition temp_condition;
//  733   /* Calculations for tampers */
//  734   /* average */
//  735   vol.avg = vol.Rph.rms/3;
        MOVW      DE, #0x3           ;; 1 cycle
        MOVW      AX, N:_vol         ;; 1 cycle
        DIVHU                        ;; 9 cycles
        NOP                          ;; 1 cycle
        MOVW      N:_vol+22, AX      ;; 1 cycle
//  736   vol.avg += vol.Yph.rms/3;
        MOVW      DE, #0x3           ;; 1 cycle
        MOVW      AX, N:_vol+6       ;; 1 cycle
        DIVHU                        ;; 9 cycles
        NOP                          ;; 1 cycle
        ADDW      AX, N:_vol+22      ;; 1 cycle
        MOVW      N:_vol+22, AX      ;; 1 cycle
//  737   vol.avg += vol.Bph.rms/3; //avg. voltage
        MOVW      DE, #0x3           ;; 1 cycle
        MOVW      AX, N:_vol+12      ;; 1 cycle
        DIVHU                        ;; 9 cycles
        NOP                          ;; 1 cycle
        ADDW      AX, N:_vol+22      ;; 1 cycle
        MOVW      N:_vol+22, AX      ;; 1 cycle
//  738   
//  739   curr.avg = curr.Rph.rms/3;
        MOVW      DE, #0x3           ;; 1 cycle
        MOVW      HL, #0x0           ;; 1 cycle
        MOVW      BC, N:_curr+2      ;; 1 cycle
        MOVW      AX, N:_curr        ;; 1 cycle
        DIVWU                        ;; 17 cycles
        NOP                          ;; 1 cycle
        MOVW      N:_curr+72, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_curr+74, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  740   curr.avg += curr.Yph.rms/3;
        MOVW      DE, #0x3           ;; 1 cycle
        MOVW      HL, #0x0           ;; 1 cycle
        MOVW      BC, N:_curr+18     ;; 1 cycle
        MOVW      AX, N:_curr+16     ;; 1 cycle
        DIVWU                        ;; 17 cycles
        NOP                          ;; 1 cycle
        MOVW      HL, N:_curr+74     ;; 1 cycle
        MOVW      DE, N:_curr+72     ;; 1 cycle
        ADDW      AX, DE             ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_curr+72, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_curr+74, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  741   curr.avg += curr.Bph.rms/3;
        MOVW      DE, #0x3           ;; 1 cycle
        MOVW      HL, #0x0           ;; 1 cycle
        MOVW      BC, N:_curr+34     ;; 1 cycle
        MOVW      AX, N:_curr+32     ;; 1 cycle
        DIVWU                        ;; 17 cycles
        NOP                          ;; 1 cycle
        MOVW      HL, N:_curr+74     ;; 1 cycle
        MOVW      DE, N:_curr+72     ;; 1 cycle
        ADDW      AX, DE             ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_curr+72, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_curr+74, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  742   
//  743   /* Maximum */
//  744   vol.max = MAX(vol.Rph.rms,vol.Yph.rms);
        MOVW      HL, N:_vol+6       ;; 1 cycle
        MOVW      AX, N:_vol         ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BNC       ??magnet_log_logic_77  ;; 4 cycles
        ; ------------------------------------- Block: 149 cycles
        MOVW      AX, N:_vol+6       ;; 1 cycle
        BR        S:??magnet_log_logic_78  ;; 3 cycles
        ; ------------------------------------- Block: 4 cycles
??magnet_log_logic_77:
        MOVW      AX, N:_vol         ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??magnet_log_logic_78:
        MOVW      N:_vol+18, AX      ;; 1 cycle
//  745   vol.max = MAX(vol.Bph.rms,vol.max);  //max. voltage
        MOVW      HL, N:_vol+18      ;; 1 cycle
        MOVW      AX, N:_vol+12      ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BNC       ??magnet_log_logic_79  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
        MOVW      AX, N:_vol+18      ;; 1 cycle
        BR        S:??magnet_log_logic_80  ;; 3 cycles
        ; ------------------------------------- Block: 4 cycles
??magnet_log_logic_79:
        MOVW      AX, N:_vol+12      ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??magnet_log_logic_80:
        MOVW      N:_vol+18, AX      ;; 1 cycle
//  746   curr.max = MAX(curr.Rph.rms,curr.Yph.rms);
        MOVW      AX, N:_curr+18     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        MOVW      AX, N:_curr+16     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      BC, N:_curr+2      ;; 1 cycle
        MOVW      AX, N:_curr        ;; 1 cycle
          CFI FunCall ?UL_CMP_L03
        CALL      N:?UL_CMP_L03      ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BNC       ??magnet_log_logic_81  ;; 4 cycles
        ; ------------------------------------- Block: 15 cycles
        MOVW      BC, N:_curr+18     ;; 1 cycle
        MOVW      AX, N:_curr+16     ;; 1 cycle
        BR        S:??magnet_log_logic_82  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
??magnet_log_logic_81:
        MOVW      BC, N:_curr+2      ;; 1 cycle
        MOVW      AX, N:_curr        ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
??magnet_log_logic_82:
        MOVW      N:_curr+64, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_curr+66, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  747   curr.max = MAX(curr.Bph.rms,curr.max);
        MOVW      AX, N:_curr+66     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        MOVW      AX, N:_curr+64     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      BC, N:_curr+34     ;; 1 cycle
        MOVW      AX, N:_curr+32     ;; 1 cycle
          CFI FunCall ?UL_CMP_L03
        CALL      N:?UL_CMP_L03      ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BNC       ??magnet_log_logic_83  ;; 4 cycles
        ; ------------------------------------- Block: 18 cycles
        MOVW      BC, N:_curr+66     ;; 1 cycle
        MOVW      AX, N:_curr+64     ;; 1 cycle
        BR        S:??magnet_log_logic_84  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
??magnet_log_logic_83:
        MOVW      BC, N:_curr+34     ;; 1 cycle
        MOVW      AX, N:_curr+32     ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
??magnet_log_logic_84:
        MOVW      N:_curr+64, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_curr+66, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  748   
//  749   /* Minimum */
//  750   vol.min = MIN(vol.Rph.rms,vol.Yph.rms);
        MOVW      HL, N:_vol+6       ;; 1 cycle
        MOVW      AX, N:_vol         ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BNC       ??magnet_log_logic_85  ;; 4 cycles
        ; ------------------------------------- Block: 11 cycles
        MOVW      AX, N:_vol         ;; 1 cycle
        BR        S:??magnet_log_logic_86  ;; 3 cycles
        ; ------------------------------------- Block: 4 cycles
??magnet_log_logic_85:
        MOVW      AX, N:_vol+6       ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??magnet_log_logic_86:
        MOVW      N:_vol+20, AX      ;; 1 cycle
//  751   vol.min = MIN(vol.Bph.rms,vol.min);
        MOVW      HL, N:_vol+20      ;; 1 cycle
        MOVW      AX, N:_vol+12      ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BNC       ??magnet_log_logic_87  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
        MOVW      AX, N:_vol+12      ;; 1 cycle
        BR        S:??magnet_log_logic_88  ;; 3 cycles
        ; ------------------------------------- Block: 4 cycles
??magnet_log_logic_87:
        MOVW      AX, N:_vol+20      ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??magnet_log_logic_88:
        MOVW      N:_vol+20, AX      ;; 1 cycle
//  752   curr.min = MIN(curr.Rph.rms,curr.Yph.rms);
        MOVW      AX, N:_curr+18     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        MOVW      AX, N:_curr+16     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      BC, N:_curr+2      ;; 1 cycle
        MOVW      AX, N:_curr        ;; 1 cycle
          CFI FunCall ?UL_CMP_L03
        CALL      N:?UL_CMP_L03      ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BNC       ??magnet_log_logic_89  ;; 4 cycles
        ; ------------------------------------- Block: 15 cycles
        MOVW      BC, N:_curr+2      ;; 1 cycle
        MOVW      AX, N:_curr        ;; 1 cycle
        BR        S:??magnet_log_logic_90  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
??magnet_log_logic_89:
        MOVW      BC, N:_curr+18     ;; 1 cycle
        MOVW      AX, N:_curr+16     ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
??magnet_log_logic_90:
        MOVW      N:_curr+68, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_curr+70, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  753   curr.min = MIN(curr.Bph.rms,curr.min);
        MOVW      AX, N:_curr+70     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        MOVW      AX, N:_curr+68     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      BC, N:_curr+34     ;; 1 cycle
        MOVW      AX, N:_curr+32     ;; 1 cycle
          CFI FunCall ?UL_CMP_L03
        CALL      N:?UL_CMP_L03      ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BNC       ??magnet_log_logic_91  ;; 4 cycles
        ; ------------------------------------- Block: 18 cycles
        MOVW      BC, N:_curr+34     ;; 1 cycle
        MOVW      AX, N:_curr+32     ;; 1 cycle
        BR        S:??magnet_log_logic_92  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
??magnet_log_logic_91:
        MOVW      BC, N:_curr+70     ;; 1 cycle
        MOVW      AX, N:_curr+68     ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
??magnet_log_logic_92:
        MOVW      N:_curr+68, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_curr+70, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  754   
//  755   /* Total */
//  756   curr.total = curr.Rph.rms + curr.Yph.rms + curr.Bph.rms;
        MOVW      BC, N:_curr+2      ;; 1 cycle
        MOVW      AX, N:_curr        ;; 1 cycle
        ADDW      AX, N:_curr+16     ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, N:_curr+18     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, N:_curr+32     ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, N:_curr+34     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_curr+76, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_curr+78, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  757   
//  758   /* Diff current */
//  759   curr.diff = cal_neu_current();
          CFI FunCall _cal_neu_current
        CALL      _cal_neu_current   ;; 3 cycles
        MOVW      N:_curr+80, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_curr+82, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  760   if(curr.Nph.rms >= curr.diff)
        MOVW      AX, N:_curr+82     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        MOVW      AX, N:_curr+80     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      BC, N:_curr+50     ;; 1 cycle
        MOVW      AX, N:_curr+48     ;; 1 cycle
          CFI FunCall ?UL_CMP_L03
        CALL      N:?UL_CMP_L03      ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BC        ??magnet_log_logic_93  ;; 4 cycles
        ; ------------------------------------- Block: 49 cycles
//  761     curr.diff = curr.Nph.rms - curr.diff;
        MOVW      BC, N:_curr+50     ;; 1 cycle
        MOVW      AX, N:_curr+48     ;; 1 cycle
        SUBW      AX, N:_curr+80     ;; 1 cycle
        SKNC
        DECW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        SUBW      AX, N:_curr+82     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_curr+80, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_curr+82, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        BR        S:??magnet_log_logic_94  ;; 3 cycles
        ; ------------------------------------- Block: 18 cycles
//  762   else
//  763     curr.diff = curr.diff - curr.Nph.rms;
??magnet_log_logic_93:
        MOVW      BC, N:_curr+82     ;; 1 cycle
        MOVW      AX, N:_curr+80     ;; 1 cycle
        SUBW      AX, N:_curr+48     ;; 1 cycle
        SKNC
        DECW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        SUBW      AX, N:_curr+50     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_curr+80, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_curr+82, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ; ------------------------------------- Block: 15 cycles
//  764   
//  765   /****************************************************************************************
//  766   ****************************  MAGNET TAMPER  ********************************************
//  767   ****************************************************************************************/
//  768   if(bitIsSet(tamper_sel,TPR_MAGNET))
??magnet_log_logic_94:
        MOVW      HL, #LWRD(_tamper_sel)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        SKC                          ;; 4 cycles
        BR        N:??magnet_log_logic_95  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  769   {
//  770     temp_condition = NOT_AVAIL;
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  771     if(flag_tlv_ac_field == 1)
        MOVW      HL, #LWRD(_tlv_flag)  ;; 1 cycle
        MOV1      CY, [HL].5         ;; 1 cycle
        BNC       ??magnet_log_logic_96  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
//  772     {
//  773       thr.magnet = 100;
        MOVW      AX, #0x64          ;; 1 cycle
        MOVW      N:_thr, AX         ;; 1 cycle
        BR        S:??magnet_log_logic_97  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  774     }
//  775     else 
//  776     {
//  777       thr.magnet = 300;
??magnet_log_logic_96:
        MOVW      AX, #0x12C         ;; 1 cycle
        MOVW      N:_thr, AX         ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
//  778     }
//  779     if(tlv.Bnet_avg >= thr.magnet || tlv.Bnet_rms >= thr.magnet)
??magnet_log_logic_97:
        MOVW      HL, N:_thr         ;; 1 cycle
        MOVW      AX, N:_tlv+34      ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BNC       ??magnet_log_logic_98  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOVW      HL, N:_thr         ;; 1 cycle
        MOVW      AX, N:_tlv+36      ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BC        ??magnet_log_logic_99  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  780     { 
//  781       if((vol.Rph.rms >= thr.vol_miss_vol_occ) || (vol.Yph.rms >= thr.vol_miss_vol_occ) || (vol.Bph.rms >= thr.vol_miss_vol_occ))
??magnet_log_logic_98:
        MOVW      HL, N:_thr+2       ;; 1 cycle
        MOVW      AX, N:_vol         ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BNC       ??magnet_log_logic_100  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOVW      HL, N:_thr+2       ;; 1 cycle
        MOVW      AX, N:_vol+6       ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BNC       ??magnet_log_logic_100  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOVW      HL, N:_thr+2       ;; 1 cycle
        MOVW      AX, N:_vol+12      ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BC        ??magnet_log_logic_101  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  782       {
//  783         temp_condition = OCCUR;
??magnet_log_logic_100:
        MOV       A, #0x1            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  784         bitSet(tamper_instant_status,BIT_MAGNET);
        SET1      N:_tamper_instant_status.0  ;; 2 cycles
        BR        S:??magnet_log_logic_101  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
//  785       }
//  786     }
//  787     else 
//  788     {
//  789       temp_condition = RESTORE;
??magnet_log_logic_99:
        MOV       A, #0x2            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  790       bitClear(tamper_instant_status,BIT_MAGNET);
        CLR1      N:_tamper_instant_status.0  ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
//  791     }
//  792     
//  793     check_tamper(temp_condition, &tpr.magnet, time.magnet_store, time.magnet_restore, time.magnet_indicate);
??magnet_log_logic_101:
        MOV       X, N:_time         ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        MOVW      AX, N:_time+16     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      DE, N:_time+14     ;; 1 cycle
        MOVW      BC, #LWRD(_tpr+32)  ;; 1 cycle
        MOV       A, [SP+0x04]       ;; 1 cycle
          CFI FunCall _check_tamper
        CALL      _check_tamper      ;; 3 cycles
//  794     if(bitIsSet(tpr.magnet.flag,action_f))
        MOVW      HL, #LWRD(_tpr+32)  ;; 1 cycle
        MOV1      CY, [HL].1         ;; 1 cycle
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BNC       ??magnet_log_logic_102  ;; 4 cycles
        ; ------------------------------------- Block: 18 cycles
//  795     {
//  796         if(flag_mag_update_metro_par == 0)
        MOVW      HL, #LWRD(_flag_mag)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BC        ??magnet_log_logic_102  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  797         {
//  798             flag_mag_update_metro_par = 1;
        SET1      N:_flag_mag.0      ;; 2 cycles
//  799             flag_mag_r_updated = 0;
        CLR1      N:_flag_mag.1      ;; 2 cycles
//  800             flag_mag_y_updated = 0;
        CLR1      N:_flag_mag.2      ;; 2 cycles
//  801             flag_mag_b_updated = 0;
        CLR1      N:_flag_mag.3      ;; 2 cycles
//  802             flag_mag_all_updated = 0;
        CLR1      N:_flag_mag.4      ;; 2 cycles
//  803             tpr.magnet.time=Now;
        MOVW      HL, #LWRD(_tpr+34)  ;; 1 cycle
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
        BR        S:??magnet_log_logic_102  ;; 3 cycles
        ; ------------------------------------- Block: 29 cycles
//  804         }
//  805     }
//  806   }
//  807   else
//  808   {
//  809     tpr.magnet.flag = 0;
??magnet_log_logic_95:
        MOV       N:_tpr+32, #0x0    ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
//  810   }
//  811   
//  812   
//  813   /****************************************************************************************
//  814   ****************************  Voltage Miss  *********************************************
//  815   ****************************************************************************************/
//  816   if(bitIsSet(tamper_sel,TPR_VOL_MISS))
??magnet_log_logic_102:
        MOVW      HL, #LWRD(_tamper_sel)  ;; 1 cycle
        MOV1      CY, [HL].1         ;; 1 cycle
        SKC                          ;; 4 cycles
        BR        N:??magnet_log_logic_103  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  817   {
//  818     /***************************** R Phase *****************************/
//  819     temp_condition = NOT_AVAIL;
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  820     if(vol.Yph.rms >= thr.vol_miss_vol_any || vol.Bph.rms >= thr.vol_miss_vol_any)
        MOVW      HL, N:_thr+6       ;; 1 cycle
        MOVW      AX, N:_vol+6       ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BNC       ??magnet_log_logic_104  ;; 4 cycles
        ; ------------------------------------- Block: 9 cycles
        MOVW      HL, N:_thr+6       ;; 1 cycle
        MOVW      AX, N:_vol+12      ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BC        ??magnet_log_logic_105  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  821     {
//  822       if((vol.Rph.rms < thr.vol_miss_vol_occ) && (curr.Rph.rms >= thr.vol_miss_curr_occ))
??magnet_log_logic_104:
        MOVW      HL, N:_thr+2       ;; 1 cycle
        MOVW      AX, N:_vol         ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BNC       ??magnet_log_logic_106  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOVW      AX, N:_thr+8       ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+10
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      BC, N:_curr+2      ;; 1 cycle
        MOVW      AX, N:_curr        ;; 1 cycle
          CFI FunCall ?UL_CMP_L03
        CALL      N:?UL_CMP_L03      ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BC        ??magnet_log_logic_106  ;; 4 cycles
        ; ------------------------------------- Block: 14 cycles
//  823       {
//  824         temp_condition = OCCUR;
        MOV       A, #0x1            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  825         bitSet(tamper_instant_status,BIT_VOL_MISS_R);
        SET1      N:_tamper_instant_status.1  ;; 2 cycles
        BR        S:??magnet_log_logic_105  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
//  826       }
//  827       else if((vol.Rph.rms >= thr.vol_miss_vol_res) && (curr.Rph.rms >= thr.vol_miss_curr_res)) 
??magnet_log_logic_106:
        MOVW      HL, N:_thr+4       ;; 1 cycle
        MOVW      AX, N:_vol         ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BC        ??magnet_log_logic_105  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOVW      AX, N:_thr+10      ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+10
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      BC, N:_curr+2      ;; 1 cycle
        MOVW      AX, N:_curr        ;; 1 cycle
          CFI FunCall ?UL_CMP_L03
        CALL      N:?UL_CMP_L03      ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BC        ??magnet_log_logic_105  ;; 4 cycles
        ; ------------------------------------- Block: 14 cycles
//  828       {
//  829         temp_condition = RESTORE;
        MOV       A, #0x2            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  830         bitClear(tamper_instant_status,BIT_VOL_MISS_R);
        CLR1      N:_tamper_instant_status.1  ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
//  831       }
//  832     }
//  833     
//  834     check_tamper(temp_condition, &tpr.vol_miss.Rph, time.vol_miss_store, time.vol_miss_restore, time.vol_miss_indicate);
??magnet_log_logic_105:
        MOV       X, N:_time+1       ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        MOVW      AX, N:_time+20     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      DE, N:_time+18     ;; 1 cycle
        MOVW      BC, #LWRD(_tpr+256)  ;; 1 cycle
        MOV       A, [SP+0x04]       ;; 1 cycle
          CFI FunCall _check_tamper
        CALL      _check_tamper      ;; 3 cycles
//  835     if(bitIsSet(tpr.vol_miss.Rph.flag,action_f))
        MOVW      HL, #LWRD(_tpr+256)  ;; 1 cycle
        MOV1      CY, [HL].1         ;; 1 cycle
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BNC       ??magnet_log_logic_107  ;; 4 cycles
        ; ------------------------------------- Block: 18 cycles
//  836     {
//  837       bitClear(tpr.vol_miss.Rph.flag,action_f);
        CLR1      N:_tpr+256.1       ;; 2 cycles
//  838       if(bitIsSet(tpr.vol_miss.Rph.flag,event_f))
        MOVW      HL, #LWRD(_tpr+256)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??magnet_log_logic_108  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
//  839       {
//  840         mem_log(COMPART_VOLTAGE,EVENT_ID_VOL_MISS_R_OCC,tpr.vol_miss.Rph);
        MOVW      HL, #LWRD(_tpr+256)  ;; 1 cycle
        SUBW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+36
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x1C          ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      BC, #0x1           ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _mem_log
        CALL      _mem_log           ;; 3 cycles
        ADDW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+8
        BR        S:??magnet_log_logic_109  ;; 3 cycles
        ; ------------------------------------- Block: 16 cycles
//  841       }
//  842       else
//  843       {
//  844         mem_log(COMPART_VOLTAGE,EVENT_ID_VOL_MISS_R_RES,tpr.vol_miss.Rph);
??magnet_log_logic_108:
        MOVW      HL, #LWRD(_tpr+256)  ;; 1 cycle
        SUBW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+36
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x1C          ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      BC, #0x2           ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _mem_log
        CALL      _mem_log           ;; 3 cycles
        ADDW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+8
          CFI FunCall _update_tamper_variables
        ; ------------------------------------- Block: 13 cycles
//  845       }
//  846       update_tamper_variables();
??magnet_log_logic_109:
        CALL      _update_tamper_variables  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
//  847     }
//  848     
//  849     /***************************** Y Phase *****************************/
//  850     temp_condition = NOT_AVAIL;
??magnet_log_logic_107:
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  851     if(vol.Rph.rms >= thr.vol_miss_vol_any || vol.Bph.rms >= thr.vol_miss_vol_any)
        MOVW      HL, N:_thr+6       ;; 1 cycle
        MOVW      AX, N:_vol         ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BNC       ??magnet_log_logic_110  ;; 4 cycles
        ; ------------------------------------- Block: 9 cycles
        MOVW      HL, N:_thr+6       ;; 1 cycle
        MOVW      AX, N:_vol+12      ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BC        ??magnet_log_logic_111  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  852     {
//  853       if((vol.Yph.rms < thr.vol_miss_vol_occ) && (curr.Yph.rms >= thr.vol_miss_curr_occ))
??magnet_log_logic_110:
        MOVW      HL, N:_thr+2       ;; 1 cycle
        MOVW      AX, N:_vol+6       ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BNC       ??magnet_log_logic_112  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOVW      AX, N:_thr+8       ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+10
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      BC, N:_curr+18     ;; 1 cycle
        MOVW      AX, N:_curr+16     ;; 1 cycle
          CFI FunCall ?UL_CMP_L03
        CALL      N:?UL_CMP_L03      ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BC        ??magnet_log_logic_112  ;; 4 cycles
        ; ------------------------------------- Block: 14 cycles
//  854       {
//  855         temp_condition = OCCUR;
        MOV       A, #0x1            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  856         bitSet(tamper_instant_status,BIT_VOL_MISS_Y);
        SET1      N:_tamper_instant_status.2  ;; 2 cycles
        BR        S:??magnet_log_logic_111  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
//  857       }
//  858       else if((vol.Yph.rms >= thr.vol_miss_vol_res) && (curr.Yph.rms >= thr.vol_miss_curr_res)) 
??magnet_log_logic_112:
        MOVW      HL, N:_thr+4       ;; 1 cycle
        MOVW      AX, N:_vol+6       ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BC        ??magnet_log_logic_111  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOVW      AX, N:_thr+10      ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+10
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      BC, N:_curr+18     ;; 1 cycle
        MOVW      AX, N:_curr+16     ;; 1 cycle
          CFI FunCall ?UL_CMP_L03
        CALL      N:?UL_CMP_L03      ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BC        ??magnet_log_logic_111  ;; 4 cycles
        ; ------------------------------------- Block: 14 cycles
//  859       {
//  860         temp_condition = RESTORE;
        MOV       A, #0x2            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  861         bitClear(tamper_instant_status,BIT_VOL_MISS_Y);
        CLR1      N:_tamper_instant_status.2  ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
//  862       }
//  863     }
//  864     
//  865     check_tamper(temp_condition, &tpr.vol_miss.Yph, time.vol_miss_store, time.vol_miss_restore, time.vol_miss_indicate);
??magnet_log_logic_111:
        MOV       X, N:_time+1       ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        MOVW      AX, N:_time+20     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      DE, N:_time+18     ;; 1 cycle
        MOVW      BC, #LWRD(_tpr+284)  ;; 1 cycle
        MOV       A, [SP+0x04]       ;; 1 cycle
          CFI FunCall _check_tamper
        CALL      _check_tamper      ;; 3 cycles
//  866     if(bitIsSet(tpr.vol_miss.Yph.flag,action_f))
        MOVW      HL, #LWRD(_tpr+284)  ;; 1 cycle
        MOV1      CY, [HL].1         ;; 1 cycle
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BNC       ??magnet_log_logic_113  ;; 4 cycles
        ; ------------------------------------- Block: 18 cycles
//  867     {
//  868       bitClear(tpr.vol_miss.Yph.flag,action_f);
        CLR1      N:_tpr+284.1       ;; 2 cycles
//  869       if(bitIsSet(tpr.vol_miss.Yph.flag,event_f))
        MOVW      HL, #LWRD(_tpr+284)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??magnet_log_logic_114  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
//  870       {
//  871         mem_log(COMPART_VOLTAGE,EVENT_ID_VOL_MISS_Y_OCC,tpr.vol_miss.Yph);
        MOVW      HL, #LWRD(_tpr+284)  ;; 1 cycle
        SUBW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+36
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x1C          ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      BC, #0x3           ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _mem_log
        CALL      _mem_log           ;; 3 cycles
        ADDW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+8
        BR        S:??magnet_log_logic_115  ;; 3 cycles
        ; ------------------------------------- Block: 16 cycles
//  872       }
//  873       else
//  874       {
//  875         mem_log(COMPART_VOLTAGE,EVENT_ID_VOL_MISS_Y_RES,tpr.vol_miss.Yph);
??magnet_log_logic_114:
        MOVW      HL, #LWRD(_tpr+284)  ;; 1 cycle
        SUBW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+36
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x1C          ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      BC, #0x4           ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _mem_log
        CALL      _mem_log           ;; 3 cycles
        ADDW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+8
          CFI FunCall _update_tamper_variables
        ; ------------------------------------- Block: 13 cycles
//  876       }
//  877       update_tamper_variables();
??magnet_log_logic_115:
        CALL      _update_tamper_variables  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
//  878     }
//  879     
//  880     /***************************** B Phase *****************************/
//  881     temp_condition = NOT_AVAIL;
??magnet_log_logic_113:
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  882     if(vol.Rph.rms >= thr.vol_miss_vol_any || vol.Yph.rms >= thr.vol_miss_vol_any)
        MOVW      HL, N:_thr+6       ;; 1 cycle
        MOVW      AX, N:_vol         ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BNC       ??magnet_log_logic_116  ;; 4 cycles
        ; ------------------------------------- Block: 9 cycles
        MOVW      HL, N:_thr+6       ;; 1 cycle
        MOVW      AX, N:_vol+6       ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BC        ??magnet_log_logic_117  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  883     {
//  884       if((vol.Bph.rms < thr.vol_miss_vol_occ) && (curr.Bph.rms >= thr.vol_miss_curr_occ))
??magnet_log_logic_116:
        MOVW      HL, N:_thr+2       ;; 1 cycle
        MOVW      AX, N:_vol+12      ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BNC       ??magnet_log_logic_118  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOVW      AX, N:_thr+8       ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+10
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      BC, N:_curr+34     ;; 1 cycle
        MOVW      AX, N:_curr+32     ;; 1 cycle
          CFI FunCall ?UL_CMP_L03
        CALL      N:?UL_CMP_L03      ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BC        ??magnet_log_logic_118  ;; 4 cycles
        ; ------------------------------------- Block: 14 cycles
//  885       {
//  886         temp_condition = OCCUR;
        MOV       A, #0x1            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  887         bitSet(tamper_instant_status,BIT_VOL_MISS_B);
        SET1      N:_tamper_instant_status.3  ;; 2 cycles
        BR        S:??magnet_log_logic_117  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
//  888       }
//  889       else if((vol.Bph.rms >= thr.vol_miss_vol_res) && (curr.Bph.rms >= thr.vol_miss_curr_res)) 
??magnet_log_logic_118:
        MOVW      HL, N:_thr+4       ;; 1 cycle
        MOVW      AX, N:_vol+12      ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BC        ??magnet_log_logic_117  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOVW      AX, N:_thr+10      ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+10
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      BC, N:_curr+34     ;; 1 cycle
        MOVW      AX, N:_curr+32     ;; 1 cycle
          CFI FunCall ?UL_CMP_L03
        CALL      N:?UL_CMP_L03      ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BC        ??magnet_log_logic_117  ;; 4 cycles
        ; ------------------------------------- Block: 14 cycles
//  890       {
//  891         temp_condition = RESTORE;
        MOV       A, #0x2            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  892         bitClear(tamper_instant_status,BIT_VOL_MISS_B);
        CLR1      N:_tamper_instant_status.3  ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
//  893       }
//  894     }
//  895     
//  896     check_tamper(temp_condition, &tpr.vol_miss.Bph, time.vol_miss_store, time.vol_miss_restore, time.vol_miss_indicate);
??magnet_log_logic_117:
        MOV       X, N:_time+1       ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        MOVW      AX, N:_time+20     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      DE, N:_time+18     ;; 1 cycle
        MOVW      BC, #LWRD(_tpr+312)  ;; 1 cycle
        MOV       A, [SP+0x04]       ;; 1 cycle
          CFI FunCall _check_tamper
        CALL      _check_tamper      ;; 3 cycles
//  897     if(bitIsSet(tpr.vol_miss.Bph.flag,action_f))
        MOVW      HL, #LWRD(_tpr+312)  ;; 1 cycle
        MOV1      CY, [HL].1         ;; 1 cycle
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BNC       ??magnet_log_logic_119  ;; 4 cycles
        ; ------------------------------------- Block: 18 cycles
//  898     {
//  899       bitClear(tpr.vol_miss.Bph.flag,action_f);
        CLR1      N:_tpr+312.1       ;; 2 cycles
//  900       if(bitIsSet(tpr.vol_miss.Bph.flag,event_f))
        MOVW      HL, #LWRD(_tpr+312)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??magnet_log_logic_120  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
//  901       {
//  902         mem_log(COMPART_VOLTAGE,EVENT_ID_VOL_MISS_B_OCC,tpr.vol_miss.Bph);
        MOVW      HL, #LWRD(_tpr+312)  ;; 1 cycle
        SUBW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+36
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x1C          ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      BC, #0x5           ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _mem_log
        CALL      _mem_log           ;; 3 cycles
        ADDW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+8
        BR        S:??magnet_log_logic_121  ;; 3 cycles
        ; ------------------------------------- Block: 16 cycles
//  903       }
//  904       else
//  905       {
//  906         mem_log(COMPART_VOLTAGE,EVENT_ID_VOL_MISS_B_RES,tpr.vol_miss.Bph);
??magnet_log_logic_120:
        MOVW      HL, #LWRD(_tpr+312)  ;; 1 cycle
        SUBW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+36
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x1C          ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      BC, #0x6           ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _mem_log
        CALL      _mem_log           ;; 3 cycles
        ADDW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+8
          CFI FunCall _update_tamper_variables
        ; ------------------------------------- Block: 13 cycles
//  907       }
//  908       update_tamper_variables();
??magnet_log_logic_121:
        CALL      _update_tamper_variables  ;; 3 cycles
        BR        S:??magnet_log_logic_119  ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
//  909     }
//  910   }
//  911   else
//  912   {
//  913     tpr.vol_miss.Rph.flag = 0;
??magnet_log_logic_103:
        MOV       N:_tpr+256, #0x0   ;; 1 cycle
//  914     tpr.vol_miss.Yph.flag = 0;
        MOV       N:_tpr+284, #0x0   ;; 1 cycle
//  915     tpr.vol_miss.Bph.flag = 0;
        MOV       N:_tpr+312, #0x0   ;; 1 cycle
        ; ------------------------------------- Block: 3 cycles
//  916   }
//  917   
//  918   /****************************************************************************************
//  919   ****************************  All voltage miss*******************************************
//  920   ****************************************************************************************/
//  921   if(bitIsSet(tamper_sel,TPR_VOL_MISS_ALL))
??magnet_log_logic_119:
        MOVW      HL, #LWRD(_tamper_sel)  ;; 1 cycle
        MOV1      CY, [HL].2         ;; 1 cycle
        SKC                          ;; 4 cycles
        BR        N:??magnet_log_logic_122  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  922   {
//  923     temp_condition = NOT_AVAIL;
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  924     
//  925     if(vol.Rph.rms < thr.vol_miss_all_vol_occ && vol.Yph.rms < thr.vol_miss_all_vol_occ && vol.Bph.rms < thr.vol_miss_all_vol_occ)
        MOVW      HL, N:_thr+12      ;; 1 cycle
        MOVW      AX, N:_vol         ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BNC       ??magnet_log_logic_123  ;; 4 cycles
        ; ------------------------------------- Block: 9 cycles
        MOVW      HL, N:_thr+12      ;; 1 cycle
        MOVW      AX, N:_vol+6       ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BNC       ??magnet_log_logic_123  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOVW      HL, N:_thr+12      ;; 1 cycle
        MOVW      AX, N:_vol+12      ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BNC       ??magnet_log_logic_123  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  926     {
//  927       temp_condition = OCCUR;
        MOV       A, #0x1            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  928       bitSet(tamper_instant_status,BIT_VOL_MISS_ALL);
        SET1      N:_tamper_instant_status.4  ;; 2 cycles
        BR        S:??magnet_log_logic_124  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
//  929     }
//  930     else if(vol.Rph.rms >= thr.vol_miss_all_vol_res || vol.Yph.rms >= thr.vol_miss_all_vol_res || vol.Bph.rms >= thr.vol_miss_all_vol_res)
??magnet_log_logic_123:
        MOVW      HL, N:_thr+14      ;; 1 cycle
        MOVW      AX, N:_vol         ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BNC       ??magnet_log_logic_125  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOVW      HL, N:_thr+14      ;; 1 cycle
        MOVW      AX, N:_vol+6       ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BNC       ??magnet_log_logic_125  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOVW      HL, N:_thr+14      ;; 1 cycle
        MOVW      AX, N:_vol+12      ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BC        ??magnet_log_logic_124  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  931     {
//  932       temp_condition = RESTORE;
??magnet_log_logic_125:
        MOV       A, #0x2            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  933       bitClear(tamper_instant_status,BIT_VOL_MISS_ALL);
        CLR1      N:_tamper_instant_status.4  ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
//  934     }
//  935     
//  936     check_tamper(temp_condition, &tpr.vol_miss_all, time.vol_miss_all_store, time.vol_miss_all_restore, time.vol_miss_all_indicate);
??magnet_log_logic_124:
        MOV       X, N:_time+2       ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        MOVW      AX, N:_time+24     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      DE, N:_time+22     ;; 1 cycle
        MOVW      BC, #LWRD(_tpr+228)  ;; 1 cycle
        MOV       A, [SP+0x04]       ;; 1 cycle
          CFI FunCall _check_tamper
        CALL      _check_tamper      ;; 3 cycles
//  937     if(bitIsSet(tpr.vol_miss_all.flag,action_f))
        MOVW      HL, #LWRD(_tpr+228)  ;; 1 cycle
        MOV1      CY, [HL].1         ;; 1 cycle
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BNC       ??magnet_log_logic_126  ;; 4 cycles
        ; ------------------------------------- Block: 18 cycles
//  938     {
//  939       bitClear(tpr.vol_miss_all.flag,action_f);
        CLR1      N:_tpr+228.1       ;; 2 cycles
//  940       if(bitIsSet(tpr.vol_miss_all.flag,event_f))
        MOVW      HL, #LWRD(_tpr+228)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??magnet_log_logic_127  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
//  941       {
//  942         mem_log(COMPART_VOLTAGE,EVENT_ID_VOL_MISS_ALL_OCC,tpr.vol_miss_all);
        MOVW      HL, #LWRD(_tpr+228)  ;; 1 cycle
        SUBW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+36
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x1C          ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      BC, #0x3E9         ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _mem_log
        CALL      _mem_log           ;; 3 cycles
        ADDW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+8
        BR        S:??magnet_log_logic_128  ;; 3 cycles
        ; ------------------------------------- Block: 16 cycles
//  943       }
//  944       else
//  945       {
//  946         mem_log(COMPART_VOLTAGE,EVENT_ID_VOL_MISS_ALL_RES,tpr.vol_miss_all);
??magnet_log_logic_127:
        MOVW      HL, #LWRD(_tpr+228)  ;; 1 cycle
        SUBW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+36
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x1C          ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      BC, #0x3EA         ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _mem_log
        CALL      _mem_log           ;; 3 cycles
        ADDW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+8
          CFI FunCall _update_tamper_variables
        ; ------------------------------------- Block: 13 cycles
//  947       }
//  948       update_tamper_variables();
??magnet_log_logic_128:
        CALL      _update_tamper_variables  ;; 3 cycles
        BR        S:??magnet_log_logic_126  ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
//  949     }
//  950   }
//  951   else
//  952   {
//  953     tpr.vol_miss_all.flag = 0;
??magnet_log_logic_122:
        MOV       N:_tpr+228, #0x0   ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
//  954   }
//  955   
//  956   
//  957   /****************************************************************************************
//  958   ****************************  Voltage Unbalance  ****************************************
//  959   ****************************************************************************************/
//  960   if(bitIsSet(tamper_sel,TPR_VOL_UNBAL))
??magnet_log_logic_126:
        MOVW      HL, #LWRD(_tamper_sel)  ;; 1 cycle
        MOV1      CY, [HL].3         ;; 1 cycle
        SKC                          ;; 4 cycles
        BR        N:??magnet_log_logic_129  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  961   {
//  962     /* Calculating unbalace in the voltage*/
//  963     us16 unbal_voltage;
//  964     unbal_voltage = vol.max - vol.min;
        MOVW      AX, N:_vol+18      ;; 1 cycle
        SUBW      AX, N:_vol+20      ;; 1 cycle
        MOVW      [SP+0x02], AX      ;; 1 cycle
//  965     
//  966     if(thr.sel_ref_max_vol == 2)
        CMP       N:_thr+76, #0x2    ;; 1 cycle
        BNZ       ??magnet_log_logic_130  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
//  967     {
//  968         thr.vol_unbal_vol_occ= THR_VOL_UNBAL_OCC_VOL_PERCENT* (vol.avg / 100); //avg voltage 
        MOVW      DE, #0x64          ;; 1 cycle
        MOVW      AX, N:_vol+22      ;; 1 cycle
        DIVHU                        ;; 9 cycles
        NOP                          ;; 1 cycle
        MOVW      BC, #0x1E          ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      N:_thr+16, AX      ;; 1 cycle
        BR        S:??magnet_log_logic_131  ;; 3 cycles
        ; ------------------------------------- Block: 19 cycles
//  969     }
//  970     else if(thr.sel_ref_max_vol == 1)
??magnet_log_logic_130:
        CMP       N:_thr+76, #0x1    ;; 1 cycle
        BNZ       ??magnet_log_logic_132  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  971     {
//  972         thr.vol_unbal_vol_occ= THR_VOL_UNBAL_OCC_VOL_PERCENT* (vol.max / 100);  //max voltage of 3 phase 
        MOVW      DE, #0x64          ;; 1 cycle
        MOVW      AX, N:_vol+18      ;; 1 cycle
        DIVHU                        ;; 9 cycles
        NOP                          ;; 1 cycle
        MOVW      BC, #0x1E          ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      N:_thr+16, AX      ;; 1 cycle
        BR        S:??magnet_log_logic_131  ;; 3 cycles
        ; ------------------------------------- Block: 19 cycles
//  973     }
//  974     else
//  975     {
//  976         thr.vol_unbal_vol_occ= THR_VOL_UNBAL_OCC_VOL_PERCENT * (VREF / 100);  //reference voltage
??magnet_log_logic_132:
        MOVW      AX, #0x1C20        ;; 1 cycle
        MOVW      N:_thr+16, AX      ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
//  977     }
//  978 
//  979     
//  980     /* Checking tamper condition */
//  981     temp_condition = NOT_AVAIL;
??magnet_log_logic_131:
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  982     if(vol.Rph.rms < thr.vol_high_vol_occ && vol.Yph.rms < thr.vol_high_vol_occ && vol.Bph.rms < thr.vol_high_vol_occ &&
//  983        vol.Rph.rms >= thr.vol_unbal_all_phase_vol && vol.Yph.rms >= thr.vol_unbal_all_phase_vol && vol.Bph.rms >= thr.vol_unbal_all_phase_vol)
        MOVW      HL, N:_thr+30      ;; 1 cycle
        MOVW      AX, N:_vol         ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        SKC                          ;; 4 cycles
        BR        N:??magnet_log_logic_133  ;; 4 cycles
        ; ------------------------------------- Block: 9 cycles
        MOVW      HL, N:_thr+30      ;; 1 cycle
        MOVW      AX, N:_vol+6       ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        SKC                          ;; 4 cycles
        BR        N:??magnet_log_logic_133  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOVW      HL, N:_thr+30      ;; 1 cycle
        MOVW      AX, N:_vol+12      ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        SKC                          ;; 4 cycles
        BR        N:??magnet_log_logic_133  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOVW      HL, N:_thr+24      ;; 1 cycle
        MOVW      AX, N:_vol         ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        SKNC                         ;; 4 cycles
        BR        N:??magnet_log_logic_133  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOVW      HL, N:_thr+24      ;; 1 cycle
        MOVW      AX, N:_vol+6       ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        SKNC                         ;; 4 cycles
        BR        N:??magnet_log_logic_133  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOVW      HL, N:_thr+24      ;; 1 cycle
        MOVW      AX, N:_vol+12      ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        SKNC                         ;; 4 cycles
        BR        N:??magnet_log_logic_133  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  984     {
//  985       if(unbal_voltage >= thr.vol_unbal_vol_occ && curr.Rph.rms >= thr.vol_unbal_curr_occ && curr.Yph.rms >= thr.vol_unbal_curr_occ && curr.Bph.rms >= thr.vol_unbal_curr_occ)
        MOVW      HL, N:_thr+16      ;; 1 cycle
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BC        ??magnet_log_logic_134  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOVW      AX, N:_thr+20      ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+10
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      BC, N:_curr+2      ;; 1 cycle
        MOVW      AX, N:_curr        ;; 1 cycle
          CFI FunCall ?UL_CMP_L03
        CALL      N:?UL_CMP_L03      ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BC        ??magnet_log_logic_134  ;; 4 cycles
        ; ------------------------------------- Block: 14 cycles
        MOVW      AX, N:_thr+20      ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+10
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      BC, N:_curr+18     ;; 1 cycle
        MOVW      AX, N:_curr+16     ;; 1 cycle
          CFI FunCall ?UL_CMP_L03
        CALL      N:?UL_CMP_L03      ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BC        ??magnet_log_logic_134  ;; 4 cycles
        ; ------------------------------------- Block: 14 cycles
        MOVW      AX, N:_thr+20      ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+10
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      BC, N:_curr+34     ;; 1 cycle
        MOVW      AX, N:_curr+32     ;; 1 cycle
          CFI FunCall ?UL_CMP_L03
        CALL      N:?UL_CMP_L03      ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BC        ??magnet_log_logic_134  ;; 4 cycles
        ; ------------------------------------- Block: 14 cycles
//  986       {
//  987         temp_condition = OCCUR;
        MOV       A, #0x1            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  988         bitSet(tamper_instant_status,BIT_VOL_UNBAL);
        SET1      N:_tamper_instant_status.5  ;; 2 cycles
        BR        S:??magnet_log_logic_133  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
//  989       }
//  990       else if(unbal_voltage < thr.vol_unbal_vol_occ && curr.Rph.rms >= thr.vol_unbal_curr_res && curr.Yph.rms >= thr.vol_unbal_curr_res && curr.Bph.rms >= thr.vol_unbal_curr_res)
??magnet_log_logic_134:
        MOVW      HL, N:_thr+16      ;; 1 cycle
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BNC       ??magnet_log_logic_133  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOVW      AX, N:_thr+22      ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+10
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      BC, N:_curr+2      ;; 1 cycle
        MOVW      AX, N:_curr        ;; 1 cycle
          CFI FunCall ?UL_CMP_L03
        CALL      N:?UL_CMP_L03      ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BC        ??magnet_log_logic_133  ;; 4 cycles
        ; ------------------------------------- Block: 14 cycles
        MOVW      AX, N:_thr+22      ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+10
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      BC, N:_curr+18     ;; 1 cycle
        MOVW      AX, N:_curr+16     ;; 1 cycle
          CFI FunCall ?UL_CMP_L03
        CALL      N:?UL_CMP_L03      ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BC        ??magnet_log_logic_133  ;; 4 cycles
        ; ------------------------------------- Block: 14 cycles
        MOVW      AX, N:_thr+22      ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+10
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      BC, N:_curr+34     ;; 1 cycle
        MOVW      AX, N:_curr+32     ;; 1 cycle
          CFI FunCall ?UL_CMP_L03
        CALL      N:?UL_CMP_L03      ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BC        ??magnet_log_logic_133  ;; 4 cycles
        ; ------------------------------------- Block: 14 cycles
//  991       {
//  992         temp_condition = RESTORE;
        MOV       A, #0x2            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  993         bitClear(tamper_instant_status,BIT_VOL_UNBAL);
        CLR1      N:_tamper_instant_status.5  ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
//  994       }
//  995     }
//  996     
//  997     check_tamper(temp_condition, &tpr.vol_unbal, time.vol_unbal_store, time.vol_unbal_restore, time.vol_unbal_indicate);
??magnet_log_logic_133:
        MOV       X, N:_time+3       ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        MOVW      AX, N:_time+28     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      DE, N:_time+26     ;; 1 cycle
        MOVW      BC, #LWRD(_tpr+60)  ;; 1 cycle
        MOV       A, [SP+0x04]       ;; 1 cycle
          CFI FunCall _check_tamper
        CALL      _check_tamper      ;; 3 cycles
//  998     if(bitIsSet(tpr.vol_unbal.flag,action_f))
        MOVW      HL, #LWRD(_tpr+60)  ;; 1 cycle
        MOV1      CY, [HL].1         ;; 1 cycle
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BNC       ??magnet_log_logic_135  ;; 4 cycles
        ; ------------------------------------- Block: 18 cycles
//  999     {
// 1000       bitClear(tpr.vol_unbal.flag,action_f);
        CLR1      N:_tpr+60.1        ;; 2 cycles
// 1001       if(bitIsSet(tpr.vol_unbal.flag,event_f))
        MOVW      HL, #LWRD(_tpr+60)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??magnet_log_logic_136  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
// 1002       {
// 1003         mem_log(COMPART_VOLTAGE,EVENT_ID_VOL_UNBAL_OCC,tpr.vol_unbal);
        MOVW      HL, #LWRD(_tpr+60)  ;; 1 cycle
        SUBW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+36
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x1C          ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      BC, #0xB           ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _mem_log
        CALL      _mem_log           ;; 3 cycles
        ADDW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+8
        BR        S:??magnet_log_logic_137  ;; 3 cycles
        ; ------------------------------------- Block: 16 cycles
// 1004       }
// 1005       else
// 1006       {
// 1007         mem_log(COMPART_VOLTAGE,EVENT_ID_VOL_UNBAL_RES,tpr.vol_unbal);
??magnet_log_logic_136:
        MOVW      HL, #LWRD(_tpr+60)  ;; 1 cycle
        SUBW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+36
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x1C          ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      BC, #0xC           ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _mem_log
        CALL      _mem_log           ;; 3 cycles
        ADDW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+8
          CFI FunCall _update_tamper_variables
        ; ------------------------------------- Block: 13 cycles
// 1008       }
// 1009       update_tamper_variables();
??magnet_log_logic_137:
        CALL      _update_tamper_variables  ;; 3 cycles
        BR        S:??magnet_log_logic_135  ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
// 1010     }
// 1011   }
// 1012   else
// 1013   {
// 1014     tpr.vol_unbal.flag = 0;
??magnet_log_logic_129:
        MOV       N:_tpr+60, #0x0    ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
// 1015   }
// 1016   
// 1017   /* Low Voltage */
// 1018   if(bitIsSet(tamper_sel,TPR_VOL_LOW))
??magnet_log_logic_135:
        MOVW      HL, #LWRD(_tamper_sel)  ;; 1 cycle
        MOV1      CY, [HL].4         ;; 1 cycle
        SKC                          ;; 4 cycles
        BR        N:??magnet_log_logic_138  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1019   {
// 1020     /* Checking tamper condition */
// 1021     temp_condition = NOT_AVAIL;
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
// 1022     if(vol.Rph.rms < thr.neu_dis_vol_occ && vol.Yph.rms < thr.neu_dis_vol_occ && vol.Bph.rms < thr.neu_dis_vol_occ)
        MOVW      HL, N:_thr+60      ;; 1 cycle
        MOVW      AX, N:_vol         ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BNC       ??magnet_log_logic_139  ;; 4 cycles
        ; ------------------------------------- Block: 9 cycles
        MOVW      HL, N:_thr+60      ;; 1 cycle
        MOVW      AX, N:_vol+6       ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BNC       ??magnet_log_logic_139  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOVW      HL, N:_thr+60      ;; 1 cycle
        MOVW      AX, N:_vol+12      ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BNC       ??magnet_log_logic_139  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
// 1023     {   
// 1024       if((vol.Rph.rms < thr.vol_low_vol_occ && vol.Rph.rms >= thr.vol_miss_vol_occ) || 
// 1025          (vol.Yph.rms < thr.vol_low_vol_occ && vol.Yph.rms >= thr.vol_miss_vol_occ) || 
// 1026            (vol.Bph.rms < thr.vol_low_vol_occ && vol.Bph.rms >= thr.vol_miss_vol_occ))
        MOVW      HL, N:_thr+26      ;; 1 cycle
        MOVW      AX, N:_vol         ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BNC       ??magnet_log_logic_140  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOVW      HL, N:_thr+2       ;; 1 cycle
        MOVW      AX, N:_vol         ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BNC       ??magnet_log_logic_141  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
??magnet_log_logic_140:
        MOVW      HL, N:_thr+26      ;; 1 cycle
        MOVW      AX, N:_vol+6       ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BNC       ??magnet_log_logic_142  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOVW      HL, N:_thr+2       ;; 1 cycle
        MOVW      AX, N:_vol+6       ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BNC       ??magnet_log_logic_141  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
??magnet_log_logic_142:
        MOVW      HL, N:_thr+26      ;; 1 cycle
        MOVW      AX, N:_vol+12      ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BNC       ??magnet_log_logic_143  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOVW      HL, N:_thr+2       ;; 1 cycle
        MOVW      AX, N:_vol+12      ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BC        ??magnet_log_logic_143  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
// 1027       {
// 1028         temp_condition = OCCUR;
??magnet_log_logic_141:
        MOV       A, #0x1            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
// 1029         bitSet(tamper_instant_status,BIT_VOL_LOW);
        SET1      N:_tamper_instant_status.7  ;; 2 cycles
        BR        S:??magnet_log_logic_139  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 1030       }
// 1031       else if(vol.Rph.rms >= thr.vol_low_vol_res && vol.Yph.rms >= thr.vol_low_vol_res && vol.Bph.rms >= thr.vol_low_vol_res)
??magnet_log_logic_143:
        MOVW      HL, N:_thr+28      ;; 1 cycle
        MOVW      AX, N:_vol         ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BC        ??magnet_log_logic_139  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOVW      HL, N:_thr+28      ;; 1 cycle
        MOVW      AX, N:_vol+6       ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BC        ??magnet_log_logic_139  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOVW      HL, N:_thr+28      ;; 1 cycle
        MOVW      AX, N:_vol+12      ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BC        ??magnet_log_logic_139  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
// 1032       {
// 1033         temp_condition = RESTORE;
        MOV       A, #0x2            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
// 1034         bitClear(tamper_instant_status,BIT_VOL_LOW);
        CLR1      N:_tamper_instant_status.7  ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
// 1035       }
// 1036     }
// 1037     
// 1038     check_tamper(temp_condition, &tpr.vol_low, time.vol_low_store, time.vol_low_restore, time.vol_low_indicate);
??magnet_log_logic_139:
        MOV       X, N:_time+4       ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        MOVW      AX, N:_time+32     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      DE, N:_time+30     ;; 1 cycle
        MOVW      BC, #LWRD(_tpr+88)  ;; 1 cycle
        MOV       A, [SP+0x04]       ;; 1 cycle
          CFI FunCall _check_tamper
        CALL      _check_tamper      ;; 3 cycles
// 1039     if(bitIsSet(tpr.vol_low.flag,action_f))
        MOVW      HL, #LWRD(_tpr+88)  ;; 1 cycle
        MOV1      CY, [HL].1         ;; 1 cycle
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BNC       ??magnet_log_logic_144  ;; 4 cycles
        ; ------------------------------------- Block: 18 cycles
// 1040     {
// 1041       bitClear(tpr.vol_low.flag,action_f);
        CLR1      N:_tpr+88.1        ;; 2 cycles
// 1042       if(bitIsSet(tpr.vol_low.flag,event_f))
        MOVW      HL, #LWRD(_tpr+88)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??magnet_log_logic_145  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
// 1043       {
// 1044         mem_log(COMPART_VOLTAGE,EVENT_ID_VOL_LOW_OCC,tpr.vol_low);
        MOVW      HL, #LWRD(_tpr+88)  ;; 1 cycle
        SUBW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+36
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x1C          ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      BC, #0x9           ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _mem_log
        CALL      _mem_log           ;; 3 cycles
        ADDW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+8
        BR        S:??magnet_log_logic_146  ;; 3 cycles
        ; ------------------------------------- Block: 16 cycles
// 1045       }
// 1046       else
// 1047       {
// 1048         mem_log(COMPART_VOLTAGE,EVENT_ID_VOL_LOW_RES,tpr.vol_low);
??magnet_log_logic_145:
        MOVW      HL, #LWRD(_tpr+88)  ;; 1 cycle
        SUBW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+36
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x1C          ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      BC, #0xA           ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _mem_log
        CALL      _mem_log           ;; 3 cycles
        ADDW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+8
          CFI FunCall _update_tamper_variables
        ; ------------------------------------- Block: 13 cycles
// 1049       }
// 1050       update_tamper_variables();
??magnet_log_logic_146:
        CALL      _update_tamper_variables  ;; 3 cycles
        BR        S:??magnet_log_logic_144  ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
// 1051     }
// 1052   }
// 1053   else
// 1054   {
// 1055     tpr.vol_low.flag = 0;
??magnet_log_logic_138:
        MOV       N:_tpr+88, #0x0    ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
// 1056   }
// 1057   
// 1058   /* High Voltage */
// 1059   if(bitIsSet(tamper_sel,TPR_VOL_HIGH))
??magnet_log_logic_144:
        MOVW      HL, #LWRD(_tamper_sel)  ;; 1 cycle
        MOV1      CY, [HL].5         ;; 1 cycle
        SKC                          ;; 4 cycles
        BR        N:??magnet_log_logic_147  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1060   {
// 1061     /* Checking tamper condition */
// 1062     temp_condition = NOT_AVAIL;
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
// 1063     if(vol.Rph.rms < thr.neu_dis_vol_occ && vol.Yph.rms < thr.neu_dis_vol_occ && vol.Bph.rms < thr.neu_dis_vol_occ)
        MOVW      HL, N:_thr+60      ;; 1 cycle
        MOVW      AX, N:_vol         ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BNC       ??magnet_log_logic_148  ;; 4 cycles
        ; ------------------------------------- Block: 9 cycles
        MOVW      HL, N:_thr+60      ;; 1 cycle
        MOVW      AX, N:_vol+6       ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BNC       ??magnet_log_logic_148  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOVW      HL, N:_thr+60      ;; 1 cycle
        MOVW      AX, N:_vol+12      ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BNC       ??magnet_log_logic_148  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
// 1064     {   
// 1065       if(vol.Rph.rms >= thr.vol_high_vol_occ || vol.Yph.rms >= thr.vol_high_vol_occ || vol.Bph.rms >= thr.vol_high_vol_occ)
        MOVW      HL, N:_thr+30      ;; 1 cycle
        MOVW      AX, N:_vol         ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BNC       ??magnet_log_logic_149  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOVW      HL, N:_thr+30      ;; 1 cycle
        MOVW      AX, N:_vol+6       ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BNC       ??magnet_log_logic_149  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOVW      HL, N:_thr+30      ;; 1 cycle
        MOVW      AX, N:_vol+12      ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BC        ??magnet_log_logic_150  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
// 1066       {
// 1067         temp_condition = OCCUR;
??magnet_log_logic_149:
        MOV       A, #0x1            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
// 1068         bitSet(tamper_instant_status,BIT_VOL_HIGH);
        SET1      N:_tamper_instant_status.6  ;; 2 cycles
        BR        S:??magnet_log_logic_148  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 1069       }
// 1070       else if(vol.Rph.rms < thr.vol_high_vol_res && vol.Yph.rms < thr.vol_high_vol_res && vol.Bph.rms < thr.vol_high_vol_res &&
// 1071               (vol.Rph.rms >= thr.vol_high_vol_any_phase || vol.Yph.rms >= thr.vol_high_vol_any_phase || vol.Bph.rms >= thr.vol_high_vol_any_phase))
??magnet_log_logic_150:
        MOVW      HL, N:_thr+32      ;; 1 cycle
        MOVW      AX, N:_vol         ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BNC       ??magnet_log_logic_148  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOVW      HL, N:_thr+32      ;; 1 cycle
        MOVW      AX, N:_vol+6       ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BNC       ??magnet_log_logic_148  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOVW      HL, N:_thr+32      ;; 1 cycle
        MOVW      AX, N:_vol+12      ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BNC       ??magnet_log_logic_148  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOVW      HL, N:_thr+34      ;; 1 cycle
        MOVW      AX, N:_vol         ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BNC       ??magnet_log_logic_151  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOVW      HL, N:_thr+34      ;; 1 cycle
        MOVW      AX, N:_vol+6       ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BNC       ??magnet_log_logic_151  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOVW      HL, N:_thr+34      ;; 1 cycle
        MOVW      AX, N:_vol+12      ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BC        ??magnet_log_logic_148  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
// 1072       {
// 1073         temp_condition = RESTORE;
??magnet_log_logic_151:
        MOV       A, #0x2            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
// 1074         bitClear(tamper_instant_status,BIT_VOL_HIGH);
        CLR1      N:_tamper_instant_status.6  ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
// 1075       }
// 1076     }
// 1077     
// 1078     check_tamper(temp_condition, &tpr.vol_high, time.vol_high_store, time.vol_high_restore, time.vol_high_indicate);
??magnet_log_logic_148:
        MOV       X, N:_time+5       ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        MOVW      AX, N:_time+36     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      DE, N:_time+34     ;; 1 cycle
        MOVW      BC, #LWRD(_tpr+116)  ;; 1 cycle
        MOV       A, [SP+0x04]       ;; 1 cycle
          CFI FunCall _check_tamper
        CALL      _check_tamper      ;; 3 cycles
// 1079     if(bitIsSet(tpr.vol_high.flag,action_f))
        MOVW      HL, #LWRD(_tpr+116)  ;; 1 cycle
        MOV1      CY, [HL].1         ;; 1 cycle
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BNC       ??magnet_log_logic_152  ;; 4 cycles
        ; ------------------------------------- Block: 18 cycles
// 1080     {
// 1081       bitClear(tpr.vol_high.flag,action_f);
        CLR1      N:_tpr+116.1       ;; 2 cycles
// 1082       if(bitIsSet(tpr.vol_high.flag,event_f))
        MOVW      HL, #LWRD(_tpr+116)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??magnet_log_logic_153  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
// 1083       {
// 1084         mem_log(COMPART_VOLTAGE,EVENT_ID_VOL_HIGH_OCC,tpr.vol_high);
        MOVW      HL, #LWRD(_tpr+116)  ;; 1 cycle
        SUBW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+36
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x1C          ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      BC, #0x7           ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _mem_log
        CALL      _mem_log           ;; 3 cycles
        ADDW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+8
        BR        S:??magnet_log_logic_154  ;; 3 cycles
        ; ------------------------------------- Block: 16 cycles
// 1085       }
// 1086       else
// 1087       {
// 1088         mem_log(COMPART_VOLTAGE,EVENT_ID_VOL_HIGH_RES,tpr.vol_high);
??magnet_log_logic_153:
        MOVW      HL, #LWRD(_tpr+116)  ;; 1 cycle
        SUBW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+36
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x1C          ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      BC, #0x8           ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _mem_log
        CALL      _mem_log           ;; 3 cycles
        ADDW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+8
          CFI FunCall _update_tamper_variables
        ; ------------------------------------- Block: 13 cycles
// 1089       }
// 1090       update_tamper_variables();
??magnet_log_logic_154:
        CALL      _update_tamper_variables  ;; 3 cycles
        BR        S:??magnet_log_logic_152  ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
// 1091     }
// 1092   }
// 1093   else
// 1094   {
// 1095     tpr.vol_high.flag = 0;
??magnet_log_logic_147:
        MOV       N:_tpr+116, #0x0   ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
// 1096   }
// 1097   
// 1098   /****************************************************************************************
// 1099   ****************************  CT Rev  ***************************************************
// 1100   ****************************************************************************************/
// 1101   if(bitIsSet(tamper_sel,TPR_CT_REV))
??magnet_log_logic_152:
        MOVW      HL, #LWRD(_tamper_sel)  ;; 1 cycle
        MOV1      CY, [HL].6         ;; 1 cycle
        SKC                          ;; 4 cycles
        BR        N:??magnet_log_logic_155  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1102   {
// 1103     /***************************** R Phase *****************************/
// 1104     temp_condition = NOT_AVAIL;
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
// 1105     if(vol.Rph.rms >= thr.ct_rev_vol_limit && pf.Rph >= thr.ct_rev_pf_limit)
        MOVW      HL, N:_thr+40      ;; 1 cycle
        MOVW      AX, N:_vol         ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BC        ??magnet_log_logic_156  ;; 4 cycles
        ; ------------------------------------- Block: 9 cycles
        MOVW      HL, N:_thr+50      ;; 1 cycle
        MOVW      AX, N:_pf          ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BC        ??magnet_log_logic_156  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
// 1106     {
// 1107       if(flag_Rph_active == EXPORT && curr.Rph.rms >= thr.ct_rev_curr_occ)
        MOVW      HL, #LWRD(_flag_quadrant)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??magnet_log_logic_157  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, N:_thr+36      ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+10
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      BC, N:_curr+2      ;; 1 cycle
        MOVW      AX, N:_curr        ;; 1 cycle
          CFI FunCall ?UL_CMP_L03
        CALL      N:?UL_CMP_L03      ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BC        ??magnet_log_logic_157  ;; 4 cycles
        ; ------------------------------------- Block: 14 cycles
// 1108       {
// 1109         temp_condition = OCCUR;
        MOV       A, #0x1            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
// 1110         bitSet(tamper_instant_status,BIT_CT_REV_R);
        SET1      N:_tamper_instant_status+1.2  ;; 2 cycles
        BR        S:??magnet_log_logic_156  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 1111       }
// 1112       else if(flag_Rph_active == IMPORT && curr.Rph.rms >= thr.ct_rev_curr_res)
??magnet_log_logic_157:
        MOVW      HL, #LWRD(_flag_quadrant)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BC        ??magnet_log_logic_156  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, N:_thr+38      ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+10
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      BC, N:_curr+2      ;; 1 cycle
        MOVW      AX, N:_curr        ;; 1 cycle
          CFI FunCall ?UL_CMP_L03
        CALL      N:?UL_CMP_L03      ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BC        ??magnet_log_logic_156  ;; 4 cycles
        ; ------------------------------------- Block: 14 cycles
// 1113       {
// 1114         temp_condition = RESTORE;
        MOV       A, #0x2            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
// 1115         bitClear(tamper_instant_status,BIT_CT_REV_R);
        CLR1      N:_tamper_instant_status+1.2  ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
// 1116       }
// 1117     }
// 1118     
// 1119     check_tamper(temp_condition, &tpr.ct_rev.Rph, time.ct_rev_store, time.ct_rev_restore, time.ct_rev_indicate);
??magnet_log_logic_156:
        MOV       X, N:_time+6       ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        MOVW      AX, N:_time+40     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      DE, N:_time+38     ;; 1 cycle
        MOVW      BC, #LWRD(_tpr+340)  ;; 1 cycle
        MOV       A, [SP+0x04]       ;; 1 cycle
          CFI FunCall _check_tamper
        CALL      _check_tamper      ;; 3 cycles
// 1120     if(bitIsSet(tpr.ct_rev.Rph.flag,action_f))
        MOVW      HL, #LWRD(_tpr+340)  ;; 1 cycle
        MOV1      CY, [HL].1         ;; 1 cycle
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BNC       ??magnet_log_logic_158  ;; 4 cycles
        ; ------------------------------------- Block: 18 cycles
// 1121     {
// 1122       bitClear(tpr.ct_rev.Rph.flag,action_f);
        CLR1      N:_tpr+340.1       ;; 2 cycles
// 1123       if(bitIsSet(tpr.ct_rev.Rph.flag,event_f))
        MOVW      HL, #LWRD(_tpr+340)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??magnet_log_logic_159  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
// 1124       {
// 1125         mem_log(COMPART_CURRENT,EVENT_ID_CT_REV_R_OCC,tpr.ct_rev.Rph);
        MOVW      HL, #LWRD(_tpr+340)  ;; 1 cycle
        SUBW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+36
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x1C          ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      BC, #0x33          ;; 1 cycle
        MOV       A, #0x1            ;; 1 cycle
          CFI FunCall _mem_log
        CALL      _mem_log           ;; 3 cycles
        ADDW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+8
        BR        S:??magnet_log_logic_160  ;; 3 cycles
        ; ------------------------------------- Block: 16 cycles
// 1126       }
// 1127       else
// 1128       {
// 1129         mem_log(COMPART_CURRENT,EVENT_ID_CT_REV_R_RES,tpr.ct_rev.Rph);
??magnet_log_logic_159:
        MOVW      HL, #LWRD(_tpr+340)  ;; 1 cycle
        SUBW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+36
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x1C          ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      BC, #0x34          ;; 1 cycle
        MOV       A, #0x1            ;; 1 cycle
          CFI FunCall _mem_log
        CALL      _mem_log           ;; 3 cycles
        ADDW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+8
          CFI FunCall _update_tamper_variables
        ; ------------------------------------- Block: 13 cycles
// 1130       }
// 1131       update_tamper_variables();
??magnet_log_logic_160:
        CALL      _update_tamper_variables  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
// 1132     }
// 1133     
// 1134     
// 1135     /***************************** Y Phase *****************************/
// 1136     temp_condition = NOT_AVAIL;
??magnet_log_logic_158:
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
// 1137     if(vol.Yph.rms >= thr.ct_rev_vol_limit && pf.Yph >= thr.ct_rev_pf_limit)
        MOVW      HL, N:_thr+40      ;; 1 cycle
        MOVW      AX, N:_vol+6       ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BC        ??magnet_log_logic_161  ;; 4 cycles
        ; ------------------------------------- Block: 9 cycles
        MOVW      HL, N:_thr+50      ;; 1 cycle
        MOVW      AX, N:_pf+2        ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BC        ??magnet_log_logic_161  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
// 1138     {
// 1139       if(flag_Yph_active == EXPORT && curr.Yph.rms >= thr.ct_rev_curr_occ)
        MOVW      HL, #LWRD(_flag_quadrant)  ;; 1 cycle
        MOV1      CY, [HL].2         ;; 1 cycle
        BNC       ??magnet_log_logic_162  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, N:_thr+36      ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+10
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      BC, N:_curr+18     ;; 1 cycle
        MOVW      AX, N:_curr+16     ;; 1 cycle
          CFI FunCall ?UL_CMP_L03
        CALL      N:?UL_CMP_L03      ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BC        ??magnet_log_logic_162  ;; 4 cycles
        ; ------------------------------------- Block: 14 cycles
// 1140       {
// 1141         temp_condition = OCCUR;
        MOV       A, #0x1            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
// 1142         bitSet(tamper_instant_status,BIT_CT_REV_Y);
        SET1      N:_tamper_instant_status+1.3  ;; 2 cycles
        BR        S:??magnet_log_logic_161  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 1143       }
// 1144       else if(flag_Yph_active == IMPORT && curr.Yph.rms >= thr.ct_rev_curr_res)
??magnet_log_logic_162:
        MOVW      HL, #LWRD(_flag_quadrant)  ;; 1 cycle
        MOV1      CY, [HL].2         ;; 1 cycle
        BC        ??magnet_log_logic_161  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, N:_thr+38      ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+10
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      BC, N:_curr+18     ;; 1 cycle
        MOVW      AX, N:_curr+16     ;; 1 cycle
          CFI FunCall ?UL_CMP_L03
        CALL      N:?UL_CMP_L03      ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BC        ??magnet_log_logic_161  ;; 4 cycles
        ; ------------------------------------- Block: 14 cycles
// 1145       {
// 1146         temp_condition = RESTORE;
        MOV       A, #0x2            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
// 1147         bitClear(tamper_instant_status,BIT_CT_REV_Y);
        CLR1      N:_tamper_instant_status+1.3  ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
// 1148       }
// 1149     }
// 1150     
// 1151     check_tamper(temp_condition, &tpr.ct_rev.Yph, time.ct_rev_store, time.ct_rev_restore, time.ct_rev_indicate);
??magnet_log_logic_161:
        MOV       X, N:_time+6       ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        MOVW      AX, N:_time+40     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      DE, N:_time+38     ;; 1 cycle
        MOVW      BC, #LWRD(_tpr+368)  ;; 1 cycle
        MOV       A, [SP+0x04]       ;; 1 cycle
          CFI FunCall _check_tamper
        CALL      _check_tamper      ;; 3 cycles
// 1152     if(bitIsSet(tpr.ct_rev.Yph.flag,action_f))
        MOVW      HL, #LWRD(_tpr+368)  ;; 1 cycle
        MOV1      CY, [HL].1         ;; 1 cycle
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BNC       ??magnet_log_logic_163  ;; 4 cycles
        ; ------------------------------------- Block: 18 cycles
// 1153     {
// 1154       bitClear(tpr.ct_rev.Yph.flag,action_f);
        CLR1      N:_tpr+368.1       ;; 2 cycles
// 1155       if(bitIsSet(tpr.ct_rev.Yph.flag,event_f))
        MOVW      HL, #LWRD(_tpr+368)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??magnet_log_logic_164  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
// 1156       {
// 1157         mem_log(COMPART_CURRENT,EVENT_ID_CT_REV_Y_OCC,tpr.ct_rev.Yph);
        MOVW      HL, #LWRD(_tpr+368)  ;; 1 cycle
        SUBW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+36
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x1C          ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      BC, #0x35          ;; 1 cycle
        MOV       A, #0x1            ;; 1 cycle
          CFI FunCall _mem_log
        CALL      _mem_log           ;; 3 cycles
        ADDW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+8
        BR        S:??magnet_log_logic_165  ;; 3 cycles
        ; ------------------------------------- Block: 16 cycles
// 1158       }
// 1159       else
// 1160       {
// 1161         mem_log(COMPART_CURRENT,EVENT_ID_CT_REV_Y_RES,tpr.ct_rev.Yph);
??magnet_log_logic_164:
        MOVW      HL, #LWRD(_tpr+368)  ;; 1 cycle
        SUBW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+36
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x1C          ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      BC, #0x36          ;; 1 cycle
        MOV       A, #0x1            ;; 1 cycle
          CFI FunCall _mem_log
        CALL      _mem_log           ;; 3 cycles
        ADDW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+8
          CFI FunCall _update_tamper_variables
        ; ------------------------------------- Block: 13 cycles
// 1162       }
// 1163       update_tamper_variables();
??magnet_log_logic_165:
        CALL      _update_tamper_variables  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
// 1164     }
// 1165     
// 1166     /***************************** B Phase *****************************/
// 1167     temp_condition = NOT_AVAIL;
??magnet_log_logic_163:
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
// 1168     if(vol.Bph.rms >= thr.ct_rev_vol_limit && pf.Bph >= thr.ct_rev_pf_limit)
        MOVW      HL, N:_thr+40      ;; 1 cycle
        MOVW      AX, N:_vol+12      ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BC        ??magnet_log_logic_166  ;; 4 cycles
        ; ------------------------------------- Block: 9 cycles
        MOVW      HL, N:_thr+50      ;; 1 cycle
        MOVW      AX, N:_pf+4        ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BC        ??magnet_log_logic_166  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
// 1169     {
// 1170       if(flag_Bph_active == EXPORT && curr.Bph.rms >= thr.ct_rev_curr_occ)
        MOVW      HL, #LWRD(_flag_quadrant)  ;; 1 cycle
        MOV1      CY, [HL].4         ;; 1 cycle
        BNC       ??magnet_log_logic_167  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, N:_thr+36      ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+10
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      BC, N:_curr+34     ;; 1 cycle
        MOVW      AX, N:_curr+32     ;; 1 cycle
          CFI FunCall ?UL_CMP_L03
        CALL      N:?UL_CMP_L03      ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BC        ??magnet_log_logic_167  ;; 4 cycles
        ; ------------------------------------- Block: 14 cycles
// 1171       {
// 1172         temp_condition = OCCUR;
        MOV       A, #0x1            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
// 1173         bitSet(tamper_instant_status,BIT_CT_REV_B);
        SET1      N:_tamper_instant_status+1.4  ;; 2 cycles
        BR        S:??magnet_log_logic_166  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 1174       }
// 1175       else if(flag_Bph_active == IMPORT && curr.Bph.rms >= thr.ct_rev_curr_res)
??magnet_log_logic_167:
        MOVW      HL, #LWRD(_flag_quadrant)  ;; 1 cycle
        MOV1      CY, [HL].4         ;; 1 cycle
        BC        ??magnet_log_logic_166  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, N:_thr+38      ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+10
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      BC, N:_curr+34     ;; 1 cycle
        MOVW      AX, N:_curr+32     ;; 1 cycle
          CFI FunCall ?UL_CMP_L03
        CALL      N:?UL_CMP_L03      ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BC        ??magnet_log_logic_166  ;; 4 cycles
        ; ------------------------------------- Block: 14 cycles
// 1176       {
// 1177         temp_condition = RESTORE;
        MOV       A, #0x2            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
// 1178         bitClear(tamper_instant_status,BIT_CT_REV_B);
        CLR1      N:_tamper_instant_status+1.4  ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
// 1179       }
// 1180     }
// 1181     
// 1182     check_tamper(temp_condition, &tpr.ct_rev.Bph, time.ct_rev_store, time.ct_rev_restore, time.ct_rev_indicate);
??magnet_log_logic_166:
        MOV       X, N:_time+6       ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        MOVW      AX, N:_time+40     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      DE, N:_time+38     ;; 1 cycle
        MOVW      BC, #LWRD(_tpr+396)  ;; 1 cycle
        MOV       A, [SP+0x04]       ;; 1 cycle
          CFI FunCall _check_tamper
        CALL      _check_tamper      ;; 3 cycles
// 1183     if(bitIsSet(tpr.ct_rev.Bph.flag,action_f))
        MOVW      HL, #LWRD(_tpr+396)  ;; 1 cycle
        MOV1      CY, [HL].1         ;; 1 cycle
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BNC       ??magnet_log_logic_168  ;; 4 cycles
        ; ------------------------------------- Block: 18 cycles
// 1184     {
// 1185       bitClear(tpr.ct_rev.Bph.flag,action_f);
        CLR1      N:_tpr+396.1       ;; 2 cycles
// 1186       if(bitIsSet(tpr.ct_rev.Bph.flag,event_f))
        MOVW      HL, #LWRD(_tpr+396)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??magnet_log_logic_169  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
// 1187       {
// 1188         mem_log(COMPART_CURRENT,EVENT_ID_CT_REV_B_OCC,tpr.ct_rev.Bph);
        MOVW      HL, #LWRD(_tpr+396)  ;; 1 cycle
        SUBW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+36
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x1C          ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      BC, #0x37          ;; 1 cycle
        MOV       A, #0x1            ;; 1 cycle
          CFI FunCall _mem_log
        CALL      _mem_log           ;; 3 cycles
        ADDW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+8
        BR        S:??magnet_log_logic_170  ;; 3 cycles
        ; ------------------------------------- Block: 16 cycles
// 1189       }
// 1190       else
// 1191       {
// 1192         mem_log(COMPART_CURRENT,EVENT_ID_CT_REV_B_RES,tpr.ct_rev.Bph);
??magnet_log_logic_169:
        MOVW      HL, #LWRD(_tpr+396)  ;; 1 cycle
        SUBW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+36
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x1C          ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      BC, #0x38          ;; 1 cycle
        MOV       A, #0x1            ;; 1 cycle
          CFI FunCall _mem_log
        CALL      _mem_log           ;; 3 cycles
        ADDW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+8
          CFI FunCall _update_tamper_variables
        ; ------------------------------------- Block: 13 cycles
// 1193       }
// 1194       update_tamper_variables();
??magnet_log_logic_170:
        CALL      _update_tamper_variables  ;; 3 cycles
        BR        S:??magnet_log_logic_168  ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
// 1195     }
// 1196   }
// 1197   else
// 1198   {
// 1199     tpr.ct_rev.Rph.flag = 0;
??magnet_log_logic_155:
        MOV       N:_tpr+340, #0x0   ;; 1 cycle
// 1200     tpr.ct_rev.Yph.flag = 0;
        MOV       N:_tpr+368, #0x0   ;; 1 cycle
// 1201     tpr.ct_rev.Bph.flag = 0;
        MOV       N:_tpr+396, #0x0   ;; 1 cycle
        ; ------------------------------------- Block: 3 cycles
// 1202   }
// 1203   
// 1204   /* CT Bypass */
// 1205   if(bitIsSet(tamper_sel,TPR_CT_BYPASS))
??magnet_log_logic_168:
        MOVW      HL, #LWRD(_tamper_sel+1)  ;; 1 cycle
        MOV1      CY, [HL].1         ;; 1 cycle
        SKC                          ;; 4 cycles
        BR        N:??magnet_log_logic_171  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1206   {
// 1207     /* Checking tamper condition */
// 1208     temp_condition = NOT_AVAIL;
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
// 1209     if(curr.Rph.rms >= thr.ct_bypass_all_phase_curr && curr.Yph.rms >= thr.ct_bypass_all_phase_curr && curr.Bph.rms >= thr.ct_bypass_all_phase_curr)
        MOVW      AX, N:_thr+56      ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+10
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      BC, N:_curr+2      ;; 1 cycle
        MOVW      AX, N:_curr        ;; 1 cycle
          CFI FunCall ?UL_CMP_L03
        CALL      N:?UL_CMP_L03      ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        SKNC                         ;; 4 cycles
        BR        N:??magnet_log_logic_172  ;; 4 cycles
        ; ------------------------------------- Block: 16 cycles
        MOVW      AX, N:_thr+56      ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+10
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      BC, N:_curr+18     ;; 1 cycle
        MOVW      AX, N:_curr+16     ;; 1 cycle
          CFI FunCall ?UL_CMP_L03
        CALL      N:?UL_CMP_L03      ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        SKNC                         ;; 4 cycles
        BR        N:??magnet_log_logic_172  ;; 4 cycles
        ; ------------------------------------- Block: 14 cycles
        MOVW      AX, N:_thr+56      ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+10
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      BC, N:_curr+34     ;; 1 cycle
        MOVW      AX, N:_curr+32     ;; 1 cycle
          CFI FunCall ?UL_CMP_L03
        CALL      N:?UL_CMP_L03      ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BC        ??magnet_log_logic_172  ;; 4 cycles
        ; ------------------------------------- Block: 14 cycles
// 1210     {   
// 1211       if((curr.diff >= thr.ct_bypass_residual_occ)&&(bitIsClear(tamper_instant_status,BIT_CT_REV_R))
// 1212        &&(bitIsClear(tamper_instant_status,BIT_CT_REV_Y))&&(bitIsClear(tamper_instant_status,BIT_CT_REV_B))
// 1213              &&(curr.avg>thr.ct_bypass_avg_current))
        MOVW      AX, N:_thr+52      ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+10
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      BC, N:_curr+82     ;; 1 cycle
        MOVW      AX, N:_curr+80     ;; 1 cycle
          CFI FunCall ?UL_CMP_L03
        CALL      N:?UL_CMP_L03      ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BC        ??magnet_log_logic_173  ;; 4 cycles
        ; ------------------------------------- Block: 14 cycles
        MOVW      BC, N:_tamper_instant_status+2  ;; 1 cycle
        MOVW      AX, N:_tamper_instant_status  ;; 1 cycle
        CLRW      BC                 ;; 1 cycle
        CLRB      X                  ;; 1 cycle
        AND       A, #0x1C           ;; 1 cycle
        OR        A, X               ;; 1 cycle
        OR        A, C               ;; 1 cycle
        OR        A, B               ;; 1 cycle
        BNZ       ??magnet_log_logic_173  ;; 4 cycles
        ; ------------------------------------- Block: 12 cycles
        MOVW      AX, N:_curr+74     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        MOVW      AX, N:_curr+72     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      AX, N:_thr+58      ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
          CFI FunCall ?UL_CMP_L03
        CALL      N:?UL_CMP_L03      ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BNC       ??magnet_log_logic_173  ;; 4 cycles
        ; ------------------------------------- Block: 14 cycles
// 1214       {
// 1215         temp_condition = OCCUR;
        MOV       A, #0x1            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
// 1216         bitSet(tamper_instant_status,BIT_CT_BYPASS);
        SET1      N:_tamper_instant_status+1.1  ;; 2 cycles
        BR        S:??magnet_log_logic_172  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 1217       }
// 1218       else if((curr.diff < thr.ct_bypass_all_phase_curr)&&(curr.avg>thr.ct_bypass_avg_current))
??magnet_log_logic_173:
        MOVW      AX, N:_thr+56      ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+10
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      BC, N:_curr+82     ;; 1 cycle
        MOVW      AX, N:_curr+80     ;; 1 cycle
          CFI FunCall ?UL_CMP_L03
        CALL      N:?UL_CMP_L03      ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BNC       ??magnet_log_logic_172  ;; 4 cycles
        ; ------------------------------------- Block: 14 cycles
        MOVW      AX, N:_curr+74     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        MOVW      AX, N:_curr+72     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      AX, N:_thr+58      ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
          CFI FunCall ?UL_CMP_L03
        CALL      N:?UL_CMP_L03      ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BNC       ??magnet_log_logic_172  ;; 4 cycles
        ; ------------------------------------- Block: 14 cycles
// 1219       {
// 1220         temp_condition = RESTORE;
        MOV       A, #0x2            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
// 1221         bitClear(tamper_instant_status,BIT_CT_BYPASS);
        CLR1      N:_tamper_instant_status+1.1  ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
// 1222       }
// 1223     }
// 1224     
// 1225     check_tamper(temp_condition, &tpr.ct_bypass, time.ct_bypass_store, time.ct_bypass_restore, time.ct_bypass_indicate);
??magnet_log_logic_172:
        MOV       X, N:_time+8       ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        MOVW      AX, N:_time+48     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      DE, N:_time+46     ;; 1 cycle
        MOVW      BC, #LWRD(_tpr+172)  ;; 1 cycle
        MOV       A, [SP+0x04]       ;; 1 cycle
          CFI FunCall _check_tamper
        CALL      _check_tamper      ;; 3 cycles
// 1226     if(bitIsSet(tpr.ct_bypass.flag,action_f))
        MOVW      HL, #LWRD(_tpr+172)  ;; 1 cycle
        MOV1      CY, [HL].1         ;; 1 cycle
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BNC       ??magnet_log_logic_174  ;; 4 cycles
        ; ------------------------------------- Block: 18 cycles
// 1227     {
// 1228       bitClear(tpr.ct_bypass.flag,action_f);
        CLR1      N:_tpr+172.1       ;; 2 cycles
// 1229       if(bitIsSet(tpr.ct_bypass.flag,event_f))
        MOVW      HL, #LWRD(_tpr+172)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??magnet_log_logic_175  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
// 1230       {
// 1231         mem_log(COMPART_CURRENT,EVENT_ID_CT_BYPASS_OCC,tpr.ct_bypass);
        MOVW      HL, #LWRD(_tpr+172)  ;; 1 cycle
        SUBW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+36
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x1C          ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      BC, #0x41          ;; 1 cycle
        MOV       A, #0x1            ;; 1 cycle
          CFI FunCall _mem_log
        CALL      _mem_log           ;; 3 cycles
        ADDW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+8
        BR        S:??magnet_log_logic_176  ;; 3 cycles
        ; ------------------------------------- Block: 16 cycles
// 1232       }
// 1233       else
// 1234       {
// 1235         mem_log(COMPART_CURRENT,EVENT_ID_CT_BYPASS_RES,tpr.ct_bypass);
??magnet_log_logic_175:
        MOVW      HL, #LWRD(_tpr+172)  ;; 1 cycle
        SUBW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+36
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x1C          ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      BC, #0x42          ;; 1 cycle
        MOV       A, #0x1            ;; 1 cycle
          CFI FunCall _mem_log
        CALL      _mem_log           ;; 3 cycles
        ADDW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+8
          CFI FunCall _update_tamper_variables
        ; ------------------------------------- Block: 13 cycles
// 1236       }
// 1237       update_tamper_variables();
??magnet_log_logic_176:
        CALL      _update_tamper_variables  ;; 3 cycles
        BR        S:??magnet_log_logic_174  ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
// 1238     }
// 1239   }
// 1240   else
// 1241   {
// 1242     tpr.ct_bypass.flag = 0;
??magnet_log_logic_171:
        MOV       N:_tpr+172, #0x0   ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
// 1243   }
// 1244   
// 1245   /****************************************************************************************
// 1246   ****************************  CT Open  **************************************************
// 1247   ****************************************************************************************/
// 1248   if(bitIsSet(tamper_sel,TPR_CT_OPEN))
??magnet_log_logic_174:
        MOVW      HL, #LWRD(_tamper_sel)  ;; 1 cycle
        MOV1      CY, [HL].7         ;; 1 cycle
        SKC                          ;; 4 cycles
        BR        N:??magnet_log_logic_177  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1249   {
// 1250     
// 1251     if(thr.sel_ref_max_cur_ctopen == 2)
        CMP       N:_thr+77, #0x2    ;; 1 cycle
        BNZ       ??magnet_log_logic_178  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 1252     {
// 1253         thr.ct_open_curr_occ= THR_CT_OPEN_OCC_CURR_PERCENT * (curr.avg / 100);
        MOVW      DE, #0x64          ;; 1 cycle
        MOVW      HL, #0x0           ;; 1 cycle
        MOVW      BC, N:_curr+74     ;; 1 cycle
        MOVW      AX, N:_curr+72     ;; 1 cycle
        DIVWU                        ;; 17 cycles
        NOP                          ;; 1 cycle
        MOVW      N:_thr+42, AX      ;; 1 cycle
// 1254         thr.ct_open_curr_res= THR_CT_OPEN_RES_CURR_PERCENT * (curr.avg / 100);
        MOVW      DE, #0x64          ;; 1 cycle
        MOVW      HL, #0x0           ;; 1 cycle
        MOVW      BC, N:_curr+74     ;; 1 cycle
        MOVW      AX, N:_curr+72     ;; 1 cycle
        DIVWU                        ;; 17 cycles
        NOP                          ;; 1 cycle
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      N:_thr+44, AX      ;; 1 cycle
        BR        S:??magnet_log_logic_179  ;; 3 cycles
        ; ------------------------------------- Block: 52 cycles
// 1255     }
// 1256     else if(thr.sel_ref_max_cur_ctopen == 1)
??magnet_log_logic_178:
        CMP       N:_thr+77, #0x1    ;; 1 cycle
        BNZ       ??magnet_log_logic_180  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 1257     {
// 1258         thr.ct_open_curr_occ= THR_CT_OPEN_OCC_CURR_PERCENT * (curr.max / 100);
        MOVW      DE, #0x64          ;; 1 cycle
        MOVW      HL, #0x0           ;; 1 cycle
        MOVW      BC, N:_curr+66     ;; 1 cycle
        MOVW      AX, N:_curr+64     ;; 1 cycle
        DIVWU                        ;; 17 cycles
        NOP                          ;; 1 cycle
        MOVW      N:_thr+42, AX      ;; 1 cycle
// 1259         thr.ct_open_curr_res= THR_CT_OPEN_RES_CURR_PERCENT * (curr.max / 100);
        MOVW      DE, #0x64          ;; 1 cycle
        MOVW      HL, #0x0           ;; 1 cycle
        MOVW      BC, N:_curr+66     ;; 1 cycle
        MOVW      AX, N:_curr+64     ;; 1 cycle
        DIVWU                        ;; 17 cycles
        NOP                          ;; 1 cycle
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      N:_thr+44, AX      ;; 1 cycle
        BR        S:??magnet_log_logic_179  ;; 3 cycles
        ; ------------------------------------- Block: 52 cycles
// 1260     }
// 1261     else
// 1262     {
// 1263         thr.ct_open_curr_occ= THR_CT_OPEN_OCC_CURR_PERCENT * (IB / 100);
??magnet_log_logic_180:
        MOVW      AX, #0x64          ;; 1 cycle
        MOVW      N:_thr+42, AX      ;; 1 cycle
// 1264         thr.ct_open_curr_res= THR_CT_OPEN_RES_CURR_PERCENT * (IB / 100);
        MOVW      AX, #0xC8          ;; 1 cycle
        MOVW      N:_thr+44, AX      ;; 1 cycle
        ; ------------------------------------- Block: 4 cycles
// 1265     }
// 1266     /***************************** R Phase *****************************/
// 1267     /* calculating threshold 2% of current Imax*/
// 1268     temp_condition = NOT_AVAIL;
??magnet_log_logic_179:
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
// 1269     if(curr.Yph.rms >= IB_10_PERCENT || curr.Bph.rms >= IB_10_PERCENT)
        MOVW      BC, N:_curr+18     ;; 1 cycle
        MOVW      AX, N:_curr+16     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 8 cycles
        CMPW      AX, #0x3E8         ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??tamper_function_1sec_0:
        BNC       ??magnet_log_logic_181  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
        MOVW      BC, N:_curr+34     ;; 1 cycle
        MOVW      AX, N:_curr+32     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
        CMPW      AX, #0x3E8         ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??tamper_function_1sec_1:
        BC        ??magnet_log_logic_182  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
// 1270     {
// 1271       if(curr.Rph.rms < thr.ct_open_curr_occ && curr.diff >= thr.ct_open_res_curr_occ)
??magnet_log_logic_181:
        MOVW      AX, N:_thr+42      ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+10
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      BC, N:_curr+2      ;; 1 cycle
        MOVW      AX, N:_curr        ;; 1 cycle
          CFI FunCall ?UL_CMP_L03
        CALL      N:?UL_CMP_L03      ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BNC       ??magnet_log_logic_183  ;; 4 cycles
        ; ------------------------------------- Block: 14 cycles
        MOVW      AX, N:_thr+46      ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+10
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      BC, N:_curr+82     ;; 1 cycle
        MOVW      AX, N:_curr+80     ;; 1 cycle
          CFI FunCall ?UL_CMP_L03
        CALL      N:?UL_CMP_L03      ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BC        ??magnet_log_logic_183  ;; 4 cycles
        ; ------------------------------------- Block: 14 cycles
// 1272       {
// 1273         temp_condition = OCCUR;
        MOV       A, #0x1            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
// 1274         bitSet(tamper_instant_status,BIT_CT_OPEN_R);
        SET1      N:_tamper_instant_status+1.5  ;; 2 cycles
        BR        S:??magnet_log_logic_182  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 1275       }
// 1276       else if(curr.Rph.rms >= thr.ct_open_curr_res && curr.diff < thr.ct_open_res_curr_res)
??magnet_log_logic_183:
        MOVW      AX, N:_thr+44      ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+10
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      BC, N:_curr+2      ;; 1 cycle
        MOVW      AX, N:_curr        ;; 1 cycle
          CFI FunCall ?UL_CMP_L03
        CALL      N:?UL_CMP_L03      ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BC        ??magnet_log_logic_182  ;; 4 cycles
        ; ------------------------------------- Block: 14 cycles
        MOVW      AX, N:_thr+48      ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+10
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      BC, N:_curr+82     ;; 1 cycle
        MOVW      AX, N:_curr+80     ;; 1 cycle
          CFI FunCall ?UL_CMP_L03
        CALL      N:?UL_CMP_L03      ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BNC       ??magnet_log_logic_182  ;; 4 cycles
        ; ------------------------------------- Block: 14 cycles
// 1277       {
// 1278         temp_condition = RESTORE;
        MOV       A, #0x2            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
// 1279         bitClear(tamper_instant_status,BIT_CT_OPEN_R);
        CLR1      N:_tamper_instant_status+1.5  ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
// 1280       }
// 1281     }
// 1282     check_tamper(temp_condition, &tpr.ct_open.Rph, time.ct_open_store, time.ct_open_restore, time.ct_open_indicate);
??magnet_log_logic_182:
        MOV       X, N:_time+7       ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        MOVW      AX, N:_time+44     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      DE, N:_time+42     ;; 1 cycle
        MOVW      BC, #LWRD(_tpr+424)  ;; 1 cycle
        MOV       A, [SP+0x04]       ;; 1 cycle
          CFI FunCall _check_tamper
        CALL      _check_tamper      ;; 3 cycles
// 1283     if(bitIsSet(tpr.ct_open.Rph.flag,action_f))
        MOVW      HL, #LWRD(_tpr+424)  ;; 1 cycle
        MOV1      CY, [HL].1         ;; 1 cycle
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BNC       ??magnet_log_logic_184  ;; 4 cycles
        ; ------------------------------------- Block: 18 cycles
// 1284     {
// 1285       bitClear(tpr.ct_open.Rph.flag,action_f);
        CLR1      N:_tpr+424.1       ;; 2 cycles
// 1286       if(bitIsSet(tpr.ct_open.Rph.flag,event_f))
        MOVW      HL, #LWRD(_tpr+424)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??magnet_log_logic_185  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
// 1287       {
// 1288         mem_log(COMPART_CURRENT,EVENT_ID_CT_OPEN_R_OCC,tpr.ct_open.Rph);
        MOVW      HL, #LWRD(_tpr+424)  ;; 1 cycle
        SUBW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+36
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x1C          ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      BC, #0x39          ;; 1 cycle
        MOV       A, #0x1            ;; 1 cycle
          CFI FunCall _mem_log
        CALL      _mem_log           ;; 3 cycles
        ADDW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+8
        BR        S:??magnet_log_logic_186  ;; 3 cycles
        ; ------------------------------------- Block: 16 cycles
// 1289       }
// 1290       else
// 1291       {
// 1292         mem_log(COMPART_CURRENT,EVENT_ID_CT_OPEN_R_RES,tpr.ct_open.Rph);
??magnet_log_logic_185:
        MOVW      HL, #LWRD(_tpr+424)  ;; 1 cycle
        SUBW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+36
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x1C          ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      BC, #0x3A          ;; 1 cycle
        MOV       A, #0x1            ;; 1 cycle
          CFI FunCall _mem_log
        CALL      _mem_log           ;; 3 cycles
        ADDW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+8
          CFI FunCall _update_tamper_variables
        ; ------------------------------------- Block: 13 cycles
// 1293       }
// 1294       update_tamper_variables();
??magnet_log_logic_186:
        CALL      _update_tamper_variables  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
// 1295     }
// 1296     
// 1297     /***************************** Y Phase *****************************/
// 1298     temp_condition = NOT_AVAIL;
??magnet_log_logic_184:
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
// 1299     if(curr.Rph.rms >= IB_10_PERCENT || curr.Bph.rms >= IB_10_PERCENT)
        MOVW      BC, N:_curr+2      ;; 1 cycle
        MOVW      AX, N:_curr        ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 8 cycles
        CMPW      AX, #0x3E8         ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??tamper_function_1sec_2:
        BNC       ??magnet_log_logic_187  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
        MOVW      BC, N:_curr+34     ;; 1 cycle
        MOVW      AX, N:_curr+32     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
        CMPW      AX, #0x3E8         ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??tamper_function_1sec_3:
        BC        ??magnet_log_logic_188  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
// 1300     {
// 1301       if(curr.Yph.rms < thr.ct_open_curr_occ && curr.diff >= thr.ct_open_res_curr_occ)
??magnet_log_logic_187:
        MOVW      AX, N:_thr+42      ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+10
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      BC, N:_curr+18     ;; 1 cycle
        MOVW      AX, N:_curr+16     ;; 1 cycle
          CFI FunCall ?UL_CMP_L03
        CALL      N:?UL_CMP_L03      ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BNC       ??magnet_log_logic_189  ;; 4 cycles
        ; ------------------------------------- Block: 14 cycles
        MOVW      AX, N:_thr+46      ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+10
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      BC, N:_curr+82     ;; 1 cycle
        MOVW      AX, N:_curr+80     ;; 1 cycle
          CFI FunCall ?UL_CMP_L03
        CALL      N:?UL_CMP_L03      ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BC        ??magnet_log_logic_189  ;; 4 cycles
        ; ------------------------------------- Block: 14 cycles
// 1302       {
// 1303         temp_condition = OCCUR;
        MOV       A, #0x1            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
// 1304         bitSet(tamper_instant_status,BIT_CT_OPEN_Y);
        SET1      N:_tamper_instant_status+1.6  ;; 2 cycles
        BR        S:??magnet_log_logic_188  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 1305       }
// 1306       else if(curr.Yph.rms >= thr.ct_open_curr_res && curr.diff < thr.ct_open_res_curr_res)
??magnet_log_logic_189:
        MOVW      AX, N:_thr+44      ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+10
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      BC, N:_curr+18     ;; 1 cycle
        MOVW      AX, N:_curr+16     ;; 1 cycle
          CFI FunCall ?UL_CMP_L03
        CALL      N:?UL_CMP_L03      ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BC        ??magnet_log_logic_188  ;; 4 cycles
        ; ------------------------------------- Block: 14 cycles
        MOVW      AX, N:_thr+48      ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+10
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      BC, N:_curr+82     ;; 1 cycle
        MOVW      AX, N:_curr+80     ;; 1 cycle
          CFI FunCall ?UL_CMP_L03
        CALL      N:?UL_CMP_L03      ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BNC       ??magnet_log_logic_188  ;; 4 cycles
        ; ------------------------------------- Block: 14 cycles
// 1307       {
// 1308         temp_condition = RESTORE;
        MOV       A, #0x2            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
// 1309         bitClear(tamper_instant_status,BIT_CT_OPEN_Y);
        CLR1      N:_tamper_instant_status+1.6  ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
// 1310       }
// 1311     }
// 1312     check_tamper(temp_condition, &tpr.ct_open.Yph, time.ct_open_store, time.ct_open_restore, time.ct_open_indicate);
??magnet_log_logic_188:
        MOV       X, N:_time+7       ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        MOVW      AX, N:_time+44     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      DE, N:_time+42     ;; 1 cycle
        MOVW      BC, #LWRD(_tpr+452)  ;; 1 cycle
        MOV       A, [SP+0x04]       ;; 1 cycle
          CFI FunCall _check_tamper
        CALL      _check_tamper      ;; 3 cycles
// 1313     if(bitIsSet(tpr.ct_open.Yph.flag,action_f))
        MOVW      HL, #LWRD(_tpr+452)  ;; 1 cycle
        MOV1      CY, [HL].1         ;; 1 cycle
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BNC       ??magnet_log_logic_190  ;; 4 cycles
        ; ------------------------------------- Block: 18 cycles
// 1314     {
// 1315       bitClear(tpr.ct_open.Yph.flag,action_f);
        CLR1      N:_tpr+452.1       ;; 2 cycles
// 1316       if(bitIsSet(tpr.ct_open.Yph.flag,event_f))
        MOVW      HL, #LWRD(_tpr+452)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??magnet_log_logic_191  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
// 1317       {
// 1318         mem_log(COMPART_CURRENT,EVENT_ID_CT_OPEN_Y_OCC,tpr.ct_open.Yph);
        MOVW      HL, #LWRD(_tpr+452)  ;; 1 cycle
        SUBW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+36
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x1C          ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      BC, #0x3B          ;; 1 cycle
        MOV       A, #0x1            ;; 1 cycle
          CFI FunCall _mem_log
        CALL      _mem_log           ;; 3 cycles
        ADDW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+8
        BR        S:??magnet_log_logic_192  ;; 3 cycles
        ; ------------------------------------- Block: 16 cycles
// 1319       }
// 1320       else
// 1321       {
// 1322         mem_log(COMPART_CURRENT,EVENT_ID_CT_OPEN_Y_RES,tpr.ct_open.Yph);
??magnet_log_logic_191:
        MOVW      HL, #LWRD(_tpr+452)  ;; 1 cycle
        SUBW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+36
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x1C          ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      BC, #0x3C          ;; 1 cycle
        MOV       A, #0x1            ;; 1 cycle
          CFI FunCall _mem_log
        CALL      _mem_log           ;; 3 cycles
        ADDW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+8
          CFI FunCall _update_tamper_variables
        ; ------------------------------------- Block: 13 cycles
// 1323       }
// 1324       update_tamper_variables();
??magnet_log_logic_192:
        CALL      _update_tamper_variables  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
// 1325     }
// 1326     
// 1327     /***************************** B Phase *****************************/
// 1328     temp_condition = NOT_AVAIL;
??magnet_log_logic_190:
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
// 1329     if(curr.Rph.rms >= IB_10_PERCENT || curr.Yph.rms >= IB_10_PERCENT)
        MOVW      BC, N:_curr+2      ;; 1 cycle
        MOVW      AX, N:_curr        ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 8 cycles
        CMPW      AX, #0x3E8         ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??tamper_function_1sec_4:
        BNC       ??magnet_log_logic_193  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
        MOVW      BC, N:_curr+18     ;; 1 cycle
        MOVW      AX, N:_curr+16     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
        CMPW      AX, #0x3E8         ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??tamper_function_1sec_5:
        BC        ??magnet_log_logic_194  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
// 1330     {
// 1331       if(curr.Bph.rms < thr.ct_open_curr_occ && curr.diff >= thr.ct_open_res_curr_occ)
??magnet_log_logic_193:
        MOVW      AX, N:_thr+42      ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+10
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      BC, N:_curr+34     ;; 1 cycle
        MOVW      AX, N:_curr+32     ;; 1 cycle
          CFI FunCall ?UL_CMP_L03
        CALL      N:?UL_CMP_L03      ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BNC       ??magnet_log_logic_195  ;; 4 cycles
        ; ------------------------------------- Block: 14 cycles
        MOVW      AX, N:_thr+46      ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+10
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      BC, N:_curr+82     ;; 1 cycle
        MOVW      AX, N:_curr+80     ;; 1 cycle
          CFI FunCall ?UL_CMP_L03
        CALL      N:?UL_CMP_L03      ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BC        ??magnet_log_logic_195  ;; 4 cycles
        ; ------------------------------------- Block: 14 cycles
// 1332       {
// 1333         temp_condition = OCCUR;
        MOV       A, #0x1            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
// 1334         bitSet(tamper_instant_status,BIT_CT_OPEN_B);
        SET1      N:_tamper_instant_status+1.7  ;; 2 cycles
        BR        S:??magnet_log_logic_194  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 1335       }
// 1336       else if(curr.Bph.rms >= thr.ct_open_curr_res && curr.diff < thr.ct_open_res_curr_res)
??magnet_log_logic_195:
        MOVW      AX, N:_thr+44      ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+10
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      BC, N:_curr+34     ;; 1 cycle
        MOVW      AX, N:_curr+32     ;; 1 cycle
          CFI FunCall ?UL_CMP_L03
        CALL      N:?UL_CMP_L03      ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BC        ??magnet_log_logic_194  ;; 4 cycles
        ; ------------------------------------- Block: 14 cycles
        MOVW      AX, N:_thr+48      ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+10
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      BC, N:_curr+82     ;; 1 cycle
        MOVW      AX, N:_curr+80     ;; 1 cycle
          CFI FunCall ?UL_CMP_L03
        CALL      N:?UL_CMP_L03      ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BNC       ??magnet_log_logic_194  ;; 4 cycles
        ; ------------------------------------- Block: 14 cycles
// 1337       {
// 1338         temp_condition = RESTORE;
        MOV       A, #0x2            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
// 1339         bitClear(tamper_instant_status,BIT_CT_OPEN_B);
        MOVW      AX, #0x0           ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        MOVW      AX, #0x7FFF        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      BC, N:_tamper_instant_status+2  ;; 1 cycle
        MOVW      AX, N:_tamper_instant_status  ;; 1 cycle
          CFI FunCall ?L_AND_FAST_L03
        CALL      N:?L_AND_FAST_L03  ;; 3 cycles
        MOVW      N:_tamper_instant_status, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_tamper_instant_status+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        ; ------------------------------------- Block: 16 cycles
// 1340       }
// 1341     }
// 1342     check_tamper(temp_condition, &tpr.ct_open.Bph, time.ct_open_store, time.ct_open_restore, time.ct_open_indicate);
??magnet_log_logic_194:
        MOV       X, N:_time+7       ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        MOVW      AX, N:_time+44     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      DE, N:_time+42     ;; 1 cycle
        MOVW      BC, #LWRD(_tpr+480)  ;; 1 cycle
        MOV       A, [SP+0x04]       ;; 1 cycle
          CFI FunCall _check_tamper
        CALL      _check_tamper      ;; 3 cycles
// 1343     if(bitIsSet(tpr.ct_open.Bph.flag,action_f))
        MOVW      HL, #LWRD(_tpr+480)  ;; 1 cycle
        MOV1      CY, [HL].1         ;; 1 cycle
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BNC       ??magnet_log_logic_196  ;; 4 cycles
        ; ------------------------------------- Block: 18 cycles
// 1344     {
// 1345       bitClear(tpr.ct_open.Bph.flag,action_f);
        CLR1      N:_tpr+480.1       ;; 2 cycles
// 1346       if(bitIsSet(tpr.ct_open.Bph.flag,event_f))
        MOVW      HL, #LWRD(_tpr+480)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??magnet_log_logic_197  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
// 1347       {
// 1348         mem_log(COMPART_CURRENT,EVENT_ID_CT_OPEN_B_OCC,tpr.ct_open.Bph);
        MOVW      HL, #LWRD(_tpr+480)  ;; 1 cycle
        SUBW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+36
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x1C          ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      BC, #0x3D          ;; 1 cycle
        MOV       A, #0x1            ;; 1 cycle
          CFI FunCall _mem_log
        CALL      _mem_log           ;; 3 cycles
        ADDW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+8
        BR        S:??magnet_log_logic_198  ;; 3 cycles
        ; ------------------------------------- Block: 16 cycles
// 1349       }
// 1350       else
// 1351       {
// 1352         mem_log(COMPART_CURRENT,EVENT_ID_CT_OPEN_B_RES,tpr.ct_open.Bph);
??magnet_log_logic_197:
        MOVW      HL, #LWRD(_tpr+480)  ;; 1 cycle
        SUBW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+36
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x1C          ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      BC, #0x3E          ;; 1 cycle
        MOV       A, #0x1            ;; 1 cycle
          CFI FunCall _mem_log
        CALL      _mem_log           ;; 3 cycles
        ADDW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+8
          CFI FunCall _update_tamper_variables
        ; ------------------------------------- Block: 13 cycles
// 1353       }
// 1354       update_tamper_variables();
??magnet_log_logic_198:
        CALL      _update_tamper_variables  ;; 3 cycles
        BR        S:??magnet_log_logic_196  ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
// 1355     }
// 1356   }
// 1357   else
// 1358   {
// 1359     tpr.ct_open.Rph.flag = 0;
??magnet_log_logic_177:
        MOV       N:_tpr+424, #0x0   ;; 1 cycle
// 1360     tpr.ct_open.Yph.flag = 0;
        MOV       N:_tpr+452, #0x0   ;; 1 cycle
// 1361     tpr.ct_open.Bph.flag = 0;
        MOV       N:_tpr+480, #0x0   ;; 1 cycle
        ; ------------------------------------- Block: 3 cycles
// 1362   }
// 1363   
// 1364   
// 1365   /****************************************************************************************
// 1366   ****************************  Current Unbalance  ****************************************
// 1367   ****************************************************************************************/
// 1368   if(bitIsSet(tamper_sel,TPR_CURR_UNBAL))
??magnet_log_logic_196:
        MOVW      HL, #LWRD(_tamper_sel+1)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        SKC                          ;; 4 cycles
        BR        N:??magnet_log_logic_199  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1369   {
// 1370     if(thr.sel_ref_max_cur == 2)
        CMP       N:_thr+78, #0x2    ;; 1 cycle
        BNZ       ??magnet_log_logic_200  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 1371     {
// 1372         thr.curr_unbal.curr_occ= THR_CURR_UNBAL_CURR_OCC_PERCENT * (curr.avg / 100);
        MOVW      DE, #0x64          ;; 1 cycle
        MOVW      HL, #0x0           ;; 1 cycle
        MOVW      BC, N:_curr+74     ;; 1 cycle
        MOVW      AX, N:_curr+72     ;; 1 cycle
        DIVWU                        ;; 17 cycles
        NOP                          ;; 1 cycle
        MOVW      BC, #0x14          ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      N:_thr+80, AX      ;; 1 cycle
// 1373         thr.curr_unbal.curr_res= THR_CURR_UNBAL_CURR_RES_PERCENT * (curr.avg / 100);
        MOVW      DE, #0x64          ;; 1 cycle
        MOVW      HL, #0x0           ;; 1 cycle
        MOVW      BC, N:_curr+74     ;; 1 cycle
        MOVW      AX, N:_curr+72     ;; 1 cycle
        DIVWU                        ;; 17 cycles
        NOP                          ;; 1 cycle
        MOVW      BC, #0xA           ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      N:_thr+82, AX      ;; 1 cycle
        BR        S:??magnet_log_logic_201  ;; 3 cycles
        ; ------------------------------------- Block: 55 cycles
// 1374     }
// 1375     else if(thr.sel_ref_max_cur == 1)
??magnet_log_logic_200:
        CMP       N:_thr+78, #0x1    ;; 1 cycle
        BNZ       ??magnet_log_logic_202  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 1376     {
// 1377         thr.curr_unbal.curr_occ= THR_CURR_UNBAL_CURR_OCC_PERCENT * (curr.max / 100);
        MOVW      DE, #0x64          ;; 1 cycle
        MOVW      HL, #0x0           ;; 1 cycle
        MOVW      BC, N:_curr+66     ;; 1 cycle
        MOVW      AX, N:_curr+64     ;; 1 cycle
        DIVWU                        ;; 17 cycles
        NOP                          ;; 1 cycle
        MOVW      BC, #0x14          ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      N:_thr+80, AX      ;; 1 cycle
// 1378         thr.curr_unbal.curr_res= THR_CURR_UNBAL_CURR_RES_PERCENT * (curr.max / 100);
        MOVW      DE, #0x64          ;; 1 cycle
        MOVW      HL, #0x0           ;; 1 cycle
        MOVW      BC, N:_curr+66     ;; 1 cycle
        MOVW      AX, N:_curr+64     ;; 1 cycle
        DIVWU                        ;; 17 cycles
        NOP                          ;; 1 cycle
        MOVW      BC, #0xA           ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      N:_thr+82, AX      ;; 1 cycle
        BR        S:??magnet_log_logic_201  ;; 3 cycles
        ; ------------------------------------- Block: 55 cycles
// 1379     }
// 1380     else
// 1381     {
// 1382         thr.curr_unbal.curr_occ= THR_CURR_UNBAL_CURR_OCC_PERCENT * (IB / 100);
??magnet_log_logic_202:
        MOVW      AX, #0x7D0         ;; 1 cycle
        MOVW      N:_thr+80, AX      ;; 1 cycle
// 1383         thr.curr_unbal.curr_res= THR_CURR_UNBAL_CURR_RES_PERCENT * (IB / 100);
        MOVW      AX, #0x3E8         ;; 1 cycle
        MOVW      N:_thr+82, AX      ;; 1 cycle
        ; ------------------------------------- Block: 4 cycles
// 1384     }
// 1385     
// 1386     /* Checking tamper condition */
// 1387     temp_condition = NOT_AVAIL;
??magnet_log_logic_201:
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
// 1388     if((curr.max - curr.min) >= thr.curr_unbal.curr_occ && curr.Nph.rms >= thr.curr_unbal.curr_neu_limit)
        MOVW      AX, N:_thr+80      ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+10
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      BC, N:_curr+66     ;; 1 cycle
        MOVW      AX, N:_curr+64     ;; 1 cycle
        SUBW      AX, N:_curr+68     ;; 1 cycle
        SKNC
        DECW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        SUBW      AX, N:_curr+70     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
          CFI FunCall ?UL_CMP_L03
        CALL      N:?UL_CMP_L03      ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BC        ??magnet_log_logic_203  ;; 4 cycles
        ; ------------------------------------- Block: 25 cycles
        MOVW      AX, N:_thr+84      ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+10
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      BC, N:_curr+50     ;; 1 cycle
        MOVW      AX, N:_curr+48     ;; 1 cycle
          CFI FunCall ?UL_CMP_L03
        CALL      N:?UL_CMP_L03      ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BC        ??magnet_log_logic_203  ;; 4 cycles
        ; ------------------------------------- Block: 14 cycles
// 1389     {
// 1390       temp_condition = OCCUR;
        MOV       A, #0x1            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
// 1391       bitSet(tamper_instant_status,BIT_CURR_UNBAL);
        SET1      N:_tamper_instant_status+1.0  ;; 2 cycles
        BR        S:??magnet_log_logic_204  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 1392     }
// 1393     else if((curr.max - curr.min) < thr.curr_unbal.curr_res)
??magnet_log_logic_203:
        MOVW      AX, N:_thr+82      ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+10
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      BC, N:_curr+66     ;; 1 cycle
        MOVW      AX, N:_curr+64     ;; 1 cycle
        SUBW      AX, N:_curr+68     ;; 1 cycle
        SKNC
        DECW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        SUBW      AX, N:_curr+70     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
          CFI FunCall ?UL_CMP_L03
        CALL      N:?UL_CMP_L03      ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BNC       ??magnet_log_logic_204  ;; 4 cycles
        ; ------------------------------------- Block: 23 cycles
// 1394     {
// 1395       temp_condition = RESTORE;
        MOV       A, #0x2            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
// 1396       bitClear(tamper_instant_status,BIT_CURR_UNBAL);
        CLR1      N:_tamper_instant_status+1.0  ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
// 1397     }
// 1398     
// 1399     check_tamper(temp_condition, &tpr.curr_unbal, time.curr_unbal_store, time.curr_unbal_restore, time.curr_unbal_indicate);
??magnet_log_logic_204:
        MOV       X, N:_time+10      ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        MOVW      AX, N:_time+56     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      DE, N:_time+54     ;; 1 cycle
        MOVW      BC, #LWRD(_tpr+144)  ;; 1 cycle
        MOV       A, [SP+0x04]       ;; 1 cycle
          CFI FunCall _check_tamper
        CALL      _check_tamper      ;; 3 cycles
// 1400     if(bitIsSet(tpr.curr_unbal.flag,action_f))
        MOVW      HL, #LWRD(_tpr+144)  ;; 1 cycle
        MOV1      CY, [HL].1         ;; 1 cycle
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BNC       ??magnet_log_logic_205  ;; 4 cycles
        ; ------------------------------------- Block: 18 cycles
// 1401     {
// 1402       bitClear(tpr.curr_unbal.flag,action_f);
        CLR1      N:_tpr+144.1       ;; 2 cycles
// 1403       if(bitIsSet(tpr.curr_unbal.flag,event_f))
        MOVW      HL, #LWRD(_tpr+144)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??magnet_log_logic_206  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
// 1404       {
// 1405         mem_log(COMPART_CURRENT,EVENT_ID_CURR_UNBAL_OCC,tpr.curr_unbal);
        MOVW      HL, #LWRD(_tpr+144)  ;; 1 cycle
        SUBW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+36
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x1C          ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      BC, #0x3F          ;; 1 cycle
        MOV       A, #0x1            ;; 1 cycle
          CFI FunCall _mem_log
        CALL      _mem_log           ;; 3 cycles
        ADDW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+8
        BR        S:??magnet_log_logic_207  ;; 3 cycles
        ; ------------------------------------- Block: 16 cycles
// 1406       }
// 1407       else
// 1408       {
// 1409         mem_log(COMPART_CURRENT,EVENT_ID_CURR_UNBAL_RES,tpr.curr_unbal);
??magnet_log_logic_206:
        MOVW      HL, #LWRD(_tpr+144)  ;; 1 cycle
        SUBW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+36
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x1C          ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      BC, #0x40          ;; 1 cycle
        MOV       A, #0x1            ;; 1 cycle
          CFI FunCall _mem_log
        CALL      _mem_log           ;; 3 cycles
        ADDW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+8
          CFI FunCall _update_tamper_variables
        ; ------------------------------------- Block: 13 cycles
// 1410       }
// 1411       update_tamper_variables();
??magnet_log_logic_207:
        CALL      _update_tamper_variables  ;; 3 cycles
        BR        S:??magnet_log_logic_205  ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
// 1412     }
// 1413   }
// 1414   else
// 1415   {
// 1416     tpr.curr_unbal.flag = 0;
??magnet_log_logic_199:
        MOV       N:_tpr+144, #0x0   ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
// 1417   }
// 1418   
// 1419   
// 1420   
// 1421   
// 1422   /****************************************************************************************
// 1423   **************************** Over Current ***********************************************
// 1424   ****************************************************************************************/
// 1425   if(bitIsSet(tamper_sel,TPR_CURR_OVER))
??magnet_log_logic_205:
        MOVW      HL, #LWRD(_tamper_sel+1)  ;; 1 cycle
        MOV1      CY, [HL].2         ;; 1 cycle
        SKC                          ;; 4 cycles
        BR        N:??magnet_log_logic_208  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1426   {
// 1427     /***************************** R Phase *****************************/
// 1428     temp_condition = NOT_AVAIL;
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
// 1429     if(vol.Rph.rms >= thr.curr_over_all_phase_vol && vol.Yph.rms >= thr.curr_over_all_phase_vol && vol.Bph.rms >= thr.curr_over_all_phase_vol)
        MOVW      HL, N:_thr+74      ;; 1 cycle
        MOVW      AX, N:_vol         ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BC        ??magnet_log_logic_209  ;; 4 cycles
        ; ------------------------------------- Block: 9 cycles
        MOVW      HL, N:_thr+74      ;; 1 cycle
        MOVW      AX, N:_vol+6       ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BC        ??magnet_log_logic_209  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOVW      HL, N:_thr+74      ;; 1 cycle
        MOVW      AX, N:_vol+12      ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BC        ??magnet_log_logic_209  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
// 1430     {
// 1431       if((curr.Rph.rms >= thr.curr_over_curr_occ)&& bitIsClear(tamper_instant_status,BIT_MAGNET))
        MOVW      AX, N:_thr+68      ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        MOVW      AX, N:_thr+66      ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      BC, N:_curr+2      ;; 1 cycle
        MOVW      AX, N:_curr        ;; 1 cycle
          CFI FunCall ?UL_CMP_L03
        CALL      N:?UL_CMP_L03      ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BC        ??magnet_log_logic_210  ;; 4 cycles
        ; ------------------------------------- Block: 14 cycles
        MOVW      HL, #LWRD(_tamper_instant_status)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BC        ??magnet_log_logic_210  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1432       {
// 1433         temp_condition = OCCUR;
        MOV       A, #0x1            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
// 1434         bitSet(tamper_instant_status,BIT_CURR_OVER_R);
        SET1      N:_tamper_instant_status+2.0  ;; 2 cycles
        BR        S:??magnet_log_logic_209  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 1435       }
// 1436       else if(curr.Rph.rms < thr.curr_over_curr_res)
??magnet_log_logic_210:
        MOVW      AX, N:_thr+72      ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        MOVW      AX, N:_thr+70      ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      BC, N:_curr+2      ;; 1 cycle
        MOVW      AX, N:_curr        ;; 1 cycle
          CFI FunCall ?UL_CMP_L03
        CALL      N:?UL_CMP_L03      ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BNC       ??magnet_log_logic_209  ;; 4 cycles
        ; ------------------------------------- Block: 14 cycles
// 1437       {
// 1438         temp_condition = RESTORE;
        MOV       A, #0x2            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
// 1439         bitClear(tamper_instant_status,BIT_CURR_OVER_R);
        CLR1      N:_tamper_instant_status+2.0  ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
// 1440       }
// 1441     }
// 1442     check_tamper(temp_condition, &tpr.curr_over.Rph, time.curr_over_store, time.curr_over_restore, time.curr_over_indicate);
??magnet_log_logic_209:
        MOV       X, N:_time+9       ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        MOVW      AX, N:_time+52     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      DE, N:_time+50     ;; 1 cycle
        MOVW      BC, #LWRD(_tpr+508)  ;; 1 cycle
        MOV       A, [SP+0x04]       ;; 1 cycle
          CFI FunCall _check_tamper
        CALL      _check_tamper      ;; 3 cycles
// 1443     if(bitIsSet(tpr.curr_over.Rph.flag,action_f))
        MOVW      HL, #LWRD(_tpr+508)  ;; 1 cycle
        MOV1      CY, [HL].1         ;; 1 cycle
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BNC       ??magnet_log_logic_211  ;; 4 cycles
        ; ------------------------------------- Block: 18 cycles
// 1444     {
// 1445       bitClear(tpr.curr_over.Rph.flag,action_f);
        CLR1      N:_tpr+508.1       ;; 2 cycles
// 1446       if(bitIsSet(tpr.curr_over.Rph.flag,event_f))
        MOVW      HL, #LWRD(_tpr+508)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??magnet_log_logic_212  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
// 1447       {
// 1448         mem_log(COMPART_CURRENT,EVENT_ID_OVER_CURR_R_OCC,tpr.curr_over.Rph);
        MOVW      HL, #LWRD(_tpr+508)  ;; 1 cycle
        SUBW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+36
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x1C          ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      BC, #0x53          ;; 1 cycle
        MOV       A, #0x1            ;; 1 cycle
          CFI FunCall _mem_log
        CALL      _mem_log           ;; 3 cycles
        ADDW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+8
        BR        S:??magnet_log_logic_213  ;; 3 cycles
        ; ------------------------------------- Block: 16 cycles
// 1449       }
// 1450       else
// 1451       {
// 1452         mem_log(COMPART_CURRENT,EVENT_ID_OVER_CURR_R_RES,tpr.curr_over.Rph);
??magnet_log_logic_212:
        MOVW      HL, #LWRD(_tpr+508)  ;; 1 cycle
        SUBW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+36
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x1C          ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      BC, #0x54          ;; 1 cycle
        MOV       A, #0x1            ;; 1 cycle
          CFI FunCall _mem_log
        CALL      _mem_log           ;; 3 cycles
        ADDW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+8
          CFI FunCall _update_tamper_variables
        ; ------------------------------------- Block: 13 cycles
// 1453       }
// 1454       update_tamper_variables();
??magnet_log_logic_213:
        CALL      _update_tamper_variables  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
// 1455     }
// 1456     
// 1457     /***************************** Y Phase *****************************/
// 1458     temp_condition = NOT_AVAIL;
??magnet_log_logic_211:
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
// 1459     if(vol.Rph.rms >= thr.curr_over_all_phase_vol && vol.Yph.rms >= thr.curr_over_all_phase_vol && vol.Bph.rms >= thr.curr_over_all_phase_vol)
        MOVW      HL, N:_thr+74      ;; 1 cycle
        MOVW      AX, N:_vol         ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BC        ??magnet_log_logic_214  ;; 4 cycles
        ; ------------------------------------- Block: 9 cycles
        MOVW      HL, N:_thr+74      ;; 1 cycle
        MOVW      AX, N:_vol+6       ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BC        ??magnet_log_logic_214  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOVW      HL, N:_thr+74      ;; 1 cycle
        MOVW      AX, N:_vol+12      ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BC        ??magnet_log_logic_214  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
// 1460     {
// 1461       if((curr.Yph.rms >= thr.curr_over_curr_occ) && bitIsClear(tamper_instant_status,BIT_MAGNET))
        MOVW      AX, N:_thr+68      ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        MOVW      AX, N:_thr+66      ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      BC, N:_curr+18     ;; 1 cycle
        MOVW      AX, N:_curr+16     ;; 1 cycle
          CFI FunCall ?UL_CMP_L03
        CALL      N:?UL_CMP_L03      ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BC        ??magnet_log_logic_215  ;; 4 cycles
        ; ------------------------------------- Block: 14 cycles
        MOVW      HL, #LWRD(_tamper_instant_status)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BC        ??magnet_log_logic_215  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1462       {
// 1463         temp_condition = OCCUR;
        MOV       A, #0x1            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
// 1464         bitSet(tamper_instant_status,BIT_CURR_OVER_Y);
        SET1      N:_tamper_instant_status+2.1  ;; 2 cycles
        BR        S:??magnet_log_logic_214  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 1465       }
// 1466       else if(curr.Yph.rms < thr.curr_over_curr_res)
??magnet_log_logic_215:
        MOVW      AX, N:_thr+72      ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        MOVW      AX, N:_thr+70      ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      BC, N:_curr+18     ;; 1 cycle
        MOVW      AX, N:_curr+16     ;; 1 cycle
          CFI FunCall ?UL_CMP_L03
        CALL      N:?UL_CMP_L03      ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BNC       ??magnet_log_logic_214  ;; 4 cycles
        ; ------------------------------------- Block: 14 cycles
// 1467       {
// 1468         temp_condition = RESTORE;
        MOV       A, #0x2            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
// 1469         bitClear(tamper_instant_status,BIT_CURR_OVER_Y);
        CLR1      N:_tamper_instant_status+2.1  ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
// 1470       }
// 1471     }
// 1472     check_tamper(temp_condition, &tpr.curr_over.Yph, time.curr_over_store, time.curr_over_restore, time.curr_over_indicate);
??magnet_log_logic_214:
        MOV       X, N:_time+9       ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        MOVW      AX, N:_time+52     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      DE, N:_time+50     ;; 1 cycle
        MOVW      BC, #LWRD(_tpr+536)  ;; 1 cycle
        MOV       A, [SP+0x04]       ;; 1 cycle
          CFI FunCall _check_tamper
        CALL      _check_tamper      ;; 3 cycles
// 1473     if(bitIsSet(tpr.curr_over.Yph.flag,action_f))
        MOVW      HL, #LWRD(_tpr+536)  ;; 1 cycle
        MOV1      CY, [HL].1         ;; 1 cycle
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BNC       ??magnet_log_logic_216  ;; 4 cycles
        ; ------------------------------------- Block: 18 cycles
// 1474     {
// 1475       bitClear(tpr.curr_over.Yph.flag,action_f);
        CLR1      N:_tpr+536.1       ;; 2 cycles
// 1476       if(bitIsSet(tpr.curr_over.Yph.flag,event_f))
        MOVW      HL, #LWRD(_tpr+536)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??magnet_log_logic_217  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
// 1477       {
// 1478         mem_log(COMPART_CURRENT,EVENT_ID_OVER_CURR_Y_OCC,tpr.curr_over.Yph);
        MOVW      HL, #LWRD(_tpr+536)  ;; 1 cycle
        SUBW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+36
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x1C          ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      BC, #0x51          ;; 1 cycle
        MOV       A, #0x1            ;; 1 cycle
          CFI FunCall _mem_log
        CALL      _mem_log           ;; 3 cycles
        ADDW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+8
        BR        S:??magnet_log_logic_218  ;; 3 cycles
        ; ------------------------------------- Block: 16 cycles
// 1479       }
// 1480       else
// 1481       {
// 1482         mem_log(COMPART_CURRENT,EVENT_ID_OVER_CURR_Y_RES,tpr.curr_over.Yph);
??magnet_log_logic_217:
        MOVW      HL, #LWRD(_tpr+536)  ;; 1 cycle
        SUBW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+36
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x1C          ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      BC, #0x52          ;; 1 cycle
        MOV       A, #0x1            ;; 1 cycle
          CFI FunCall _mem_log
        CALL      _mem_log           ;; 3 cycles
        ADDW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+8
          CFI FunCall _update_tamper_variables
        ; ------------------------------------- Block: 13 cycles
// 1483       }
// 1484       update_tamper_variables();
??magnet_log_logic_218:
        CALL      _update_tamper_variables  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
// 1485     }
// 1486     
// 1487     /***************************** B Phase *****************************/
// 1488     temp_condition = NOT_AVAIL;
??magnet_log_logic_216:
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
// 1489     if(vol.Rph.rms >= thr.curr_over_all_phase_vol && vol.Yph.rms >= thr.curr_over_all_phase_vol && vol.Bph.rms >= thr.curr_over_all_phase_vol)
        MOVW      HL, N:_thr+74      ;; 1 cycle
        MOVW      AX, N:_vol         ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BC        ??magnet_log_logic_219  ;; 4 cycles
        ; ------------------------------------- Block: 9 cycles
        MOVW      HL, N:_thr+74      ;; 1 cycle
        MOVW      AX, N:_vol+6       ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BC        ??magnet_log_logic_219  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOVW      HL, N:_thr+74      ;; 1 cycle
        MOVW      AX, N:_vol+12      ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BC        ??magnet_log_logic_219  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
// 1490     {
// 1491       if((curr.Bph.rms >= thr.curr_over_curr_occ) && bitIsClear(tamper_instant_status,BIT_MAGNET))
        MOVW      AX, N:_thr+68      ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        MOVW      AX, N:_thr+66      ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      BC, N:_curr+34     ;; 1 cycle
        MOVW      AX, N:_curr+32     ;; 1 cycle
          CFI FunCall ?UL_CMP_L03
        CALL      N:?UL_CMP_L03      ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BC        ??magnet_log_logic_220  ;; 4 cycles
        ; ------------------------------------- Block: 14 cycles
        MOVW      HL, #LWRD(_tamper_instant_status)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BC        ??magnet_log_logic_220  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1492       {
// 1493         temp_condition = OCCUR;
        MOV       A, #0x1            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
// 1494         bitSet(tamper_instant_status,BIT_CURR_OVER_B);
        SET1      N:_tamper_instant_status+2.2  ;; 2 cycles
        BR        S:??magnet_log_logic_219  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 1495       }
// 1496       else if(curr.Bph.rms < thr.curr_over_curr_res)
??magnet_log_logic_220:
        MOVW      AX, N:_thr+72      ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        MOVW      AX, N:_thr+70      ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      BC, N:_curr+34     ;; 1 cycle
        MOVW      AX, N:_curr+32     ;; 1 cycle
          CFI FunCall ?UL_CMP_L03
        CALL      N:?UL_CMP_L03      ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BNC       ??magnet_log_logic_219  ;; 4 cycles
        ; ------------------------------------- Block: 14 cycles
// 1497       {
// 1498         temp_condition = RESTORE;
        MOV       A, #0x2            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
// 1499         bitClear(tamper_instant_status,BIT_CURR_OVER_B);
        CLR1      N:_tamper_instant_status+2.2  ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
// 1500       }
// 1501     }
// 1502     check_tamper(temp_condition, &tpr.curr_over.Bph, time.curr_over_store, time.curr_over_restore, time.curr_over_indicate);
??magnet_log_logic_219:
        MOV       X, N:_time+9       ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        MOVW      AX, N:_time+52     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      DE, N:_time+50     ;; 1 cycle
        MOVW      BC, #LWRD(_tpr+564)  ;; 1 cycle
        MOV       A, [SP+0x04]       ;; 1 cycle
          CFI FunCall _check_tamper
        CALL      _check_tamper      ;; 3 cycles
// 1503     if(bitIsSet(tpr.curr_over.Bph.flag,action_f))
        MOVW      HL, #LWRD(_tpr+564)  ;; 1 cycle
        MOV1      CY, [HL].1         ;; 1 cycle
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BNC       ??magnet_log_logic_221  ;; 4 cycles
        ; ------------------------------------- Block: 18 cycles
// 1504     {
// 1505       bitClear(tpr.curr_over.Bph.flag,action_f);
        CLR1      N:_tpr+564.1       ;; 2 cycles
// 1506       if(bitIsSet(tpr.curr_over.Bph.flag,event_f))
        MOVW      HL, #LWRD(_tpr+564)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??magnet_log_logic_222  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
// 1507       {
// 1508         mem_log(COMPART_CURRENT,EVENT_ID_OVER_CURR_B_OCC,tpr.curr_over.Bph);
        MOVW      HL, #LWRD(_tpr+564)  ;; 1 cycle
        SUBW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+36
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x1C          ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      BC, #0x4F          ;; 1 cycle
        MOV       A, #0x1            ;; 1 cycle
          CFI FunCall _mem_log
        CALL      _mem_log           ;; 3 cycles
        ADDW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+8
        BR        S:??magnet_log_logic_223  ;; 3 cycles
        ; ------------------------------------- Block: 16 cycles
// 1509       }
// 1510       else
// 1511       {
// 1512         mem_log(COMPART_CURRENT,EVENT_ID_OVER_CURR_B_RES,tpr.curr_over.Bph);
??magnet_log_logic_222:
        MOVW      HL, #LWRD(_tpr+564)  ;; 1 cycle
        SUBW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+36
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x1C          ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      BC, #0x50          ;; 1 cycle
        MOV       A, #0x1            ;; 1 cycle
          CFI FunCall _mem_log
        CALL      _mem_log           ;; 3 cycles
        ADDW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+8
          CFI FunCall _update_tamper_variables
        ; ------------------------------------- Block: 13 cycles
// 1513       }
// 1514       update_tamper_variables();
??magnet_log_logic_223:
        CALL      _update_tamper_variables  ;; 3 cycles
        BR        S:??magnet_log_logic_221  ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
// 1515     }
// 1516   }
// 1517   else
// 1518   {
// 1519     tpr.curr_over.Rph.flag = 0;
??magnet_log_logic_208:
        MOV       N:_tpr+508, #0x0   ;; 1 cycle
// 1520     tpr.curr_over.Yph.flag = 0;
        MOV       N:_tpr+536, #0x0   ;; 1 cycle
// 1521     tpr.curr_over.Bph.flag = 0;
        MOV       N:_tpr+564, #0x0   ;; 1 cycle
        ; ------------------------------------- Block: 3 cycles
// 1522   }
// 1523   
// 1524   /****************************************************************************************
// 1525   ****************************  Neutral disturb  ******************************************
// 1526   ****************************************************************************************/
// 1527   if(bitIsSet(tamper_sel,TPR_NEU_DISTURB))
??magnet_log_logic_221:
        MOVW      HL, #LWRD(_tamper_sel+1)  ;; 1 cycle
        MOV1      CY, [HL].3         ;; 1 cycle
        SKC                          ;; 4 cycles
        BR        N:??magnet_log_logic_224  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1528   {
// 1529     temp_condition = NOT_AVAIL;
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
// 1530     if(vol.Rph.rms >= thr.neu_dis_vol_occ || vol.Yph.rms >= thr.neu_dis_vol_occ || vol.Bph.rms >= thr.neu_dis_vol_occ)
        MOVW      HL, N:_thr+60      ;; 1 cycle
        MOVW      AX, N:_vol         ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BNC       ??magnet_log_logic_225  ;; 4 cycles
        ; ------------------------------------- Block: 9 cycles
        MOVW      HL, N:_thr+60      ;; 1 cycle
        MOVW      AX, N:_vol+6       ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BNC       ??magnet_log_logic_225  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOVW      HL, N:_thr+60      ;; 1 cycle
        MOVW      AX, N:_vol+12      ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BC        ??magnet_log_logic_226  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
// 1531     {                                                   
// 1532       temp_condition = OCCUR;
??magnet_log_logic_225:
        MOV       A, #0x1            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
// 1533       bitSet(tamper_instant_status,BIT_NEU_DISTURB);
        SET1      N:_tamper_instant_status+2.3  ;; 2 cycles
        BR        S:??magnet_log_logic_227  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 1534     }
// 1535     else if(vol.Rph.rms < thr.neu_dis_vol_res && vol.Yph.rms < thr.neu_dis_vol_res && vol.Bph.rms < thr.neu_dis_vol_res && 
// 1536             vol.Rph.rms >= thr.neu_dis_vol_any_phase && vol.Yph.rms >= thr.neu_dis_vol_any_phase && vol.Bph.rms >= thr.neu_dis_vol_any_phase)
??magnet_log_logic_226:
        MOVW      HL, N:_thr+62      ;; 1 cycle
        MOVW      AX, N:_vol         ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BNC       ??magnet_log_logic_227  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOVW      HL, N:_thr+62      ;; 1 cycle
        MOVW      AX, N:_vol+6       ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BNC       ??magnet_log_logic_227  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOVW      HL, N:_thr+62      ;; 1 cycle
        MOVW      AX, N:_vol+12      ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BNC       ??magnet_log_logic_227  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOVW      HL, N:_thr+64      ;; 1 cycle
        MOVW      AX, N:_vol         ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BC        ??magnet_log_logic_227  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOVW      HL, N:_thr+64      ;; 1 cycle
        MOVW      AX, N:_vol+6       ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BC        ??magnet_log_logic_227  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOVW      HL, N:_thr+64      ;; 1 cycle
        MOVW      AX, N:_vol+12      ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BC        ??magnet_log_logic_227  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
// 1537     {
// 1538       temp_condition = RESTORE;
        MOV       A, #0x2            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
// 1539       bitClear(tamper_instant_status,BIT_NEU_DISTURB);
        CLR1      N:_tamper_instant_status+2.3  ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
// 1540     }
// 1541     
// 1542     check_tamper(temp_condition, &tpr.neu_disturb, time.nd_store, time.nd_restore, time.nd_indicate);
??magnet_log_logic_227:
        MOV       X, N:_time+11      ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        MOVW      AX, N:_time+60     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      DE, N:_time+58     ;; 1 cycle
        MOVW      BC, #LWRD(_tpr+200)  ;; 1 cycle
        MOV       A, [SP+0x04]       ;; 1 cycle
          CFI FunCall _check_tamper
        CALL      _check_tamper      ;; 3 cycles
// 1543     if(bitIsSet(tpr.neu_disturb.flag,action_f))
        MOVW      HL, #LWRD(_tpr+200)  ;; 1 cycle
        MOV1      CY, [HL].1         ;; 1 cycle
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BNC       ??magnet_log_logic_228  ;; 4 cycles
        ; ------------------------------------- Block: 18 cycles
// 1544     {
// 1545       bitClear(tpr.neu_disturb.flag,action_f);
        CLR1      N:_tpr+200.1       ;; 2 cycles
// 1546       if(bitIsSet(tpr.neu_disturb.flag,event_f))
        MOVW      HL, #LWRD(_tpr+200)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??magnet_log_logic_229  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
// 1547       {
// 1548         mem_log(COMPART_OTHERS,EVENT_ID_NEU_DISTURB_OCC,tpr.neu_disturb);
        MOVW      HL, #LWRD(_tpr+200)  ;; 1 cycle
        SUBW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+36
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x1C          ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      BC, #0xCB          ;; 1 cycle
        MOV       A, #0x2            ;; 1 cycle
          CFI FunCall _mem_log
        CALL      _mem_log           ;; 3 cycles
        ADDW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+8
        BR        S:??magnet_log_logic_230  ;; 3 cycles
        ; ------------------------------------- Block: 16 cycles
// 1549       }
// 1550       else
// 1551       {
// 1552         mem_log(COMPART_OTHERS,EVENT_ID_NEU_DISTURB_RES,tpr.neu_disturb);
??magnet_log_logic_229:
        MOVW      HL, #LWRD(_tpr+200)  ;; 1 cycle
        SUBW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+36
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x1C          ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      BC, #0xCC          ;; 1 cycle
        MOV       A, #0x2            ;; 1 cycle
          CFI FunCall _mem_log
        CALL      _mem_log           ;; 3 cycles
        ADDW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+8
          CFI FunCall _update_tamper_variables
        ; ------------------------------------- Block: 13 cycles
// 1553       }
// 1554       update_tamper_variables();
??magnet_log_logic_230:
        CALL      _update_tamper_variables  ;; 3 cycles
        BR        S:??magnet_log_logic_228  ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
// 1555     }
// 1556   }
// 1557   else
// 1558   {
// 1559     tpr.neu_disturb.flag = 0;
??magnet_log_logic_224:
        MOV       N:_tpr+200, #0x0   ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
// 1560   }
// 1561   
// 1562 
// 1563 }
??magnet_log_logic_228:
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock5
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 3586 cycles

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock6 Using cfiCommon1
          CFI Function _top_cover_function
        CODE
// 1564 void top_cover_function()
// 1565 {
_top_cover_function:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 2
        SUBW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+6
// 1566     Condition temp_condition;
// 1567 
// 1568     /****************************************************************************************
// 1569     ****************************  TOP Cover  ******************************************
// 1570     ****************************************************************************************/
// 1571     if(bitIsSet(tamper_sel,TPR_TOP_COVER))
        MOVW      HL, #LWRD(_tamper_sel+1)  ;; 1 cycle
        MOV1      CY, [HL].4         ;; 1 cycle
        SKC                          ;; 4 cycles
        BR        N:??magnet_log_logic_231  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
// 1572     {
// 1573         temp_condition = NOT_AVAIL;
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
// 1574         if(TOP_COVER_DETACHED)
        MOV       A, S:0xFFF04       ;; 1 cycle
        AND       A, #0x4            ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        BZ        ??magnet_log_logic_232  ;; 4 cycles
        ; ------------------------------------- Block: 9 cycles
// 1575         {                                                   
// 1576             temp_condition = OCCUR;
        MOV       A, #0x1            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
// 1577             bitSet(tamper_instant_status,BIT_TOP_COVER);
        SET1      N:_tamper_instant_status+2.4  ;; 2 cycles
        BR        S:??magnet_log_logic_233  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 1578         }
// 1579         else if(TOP_COVER_ATTACHED && TOP_RESTORE_REQ == 1)
??magnet_log_logic_232:
        MOV       A, S:0xFFF04       ;; 1 cycle
        AND       A, #0x4            ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        BNZ       ??magnet_log_logic_233  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        CMP       N:_TOP_RESTORE_REQ, #0x1  ;; 1 cycle
        BNZ       ??magnet_log_logic_233  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 1580         {
// 1581             temp_condition = RESTORE;
        MOV       A, #0x2            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
// 1582             bitClear(tamper_instant_status,BIT_TOP_COVER);
        CLR1      N:_tamper_instant_status+2.4  ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
// 1583         }
// 1584         
// 1585         check_tamper1(temp_condition, &tpr.top_cover, time.top_cover_store, time.top_cover_restore, time.top_cover_indicate);
??magnet_log_logic_233:
        MOV       X, N:_time+12      ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+8
        MOVW      AX, N:_time+64     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        MOVW      DE, N:_time+62     ;; 1 cycle
        MOVW      BC, #LWRD(_tpr+592)  ;; 1 cycle
        MOV       A, [SP+0x04]       ;; 1 cycle
          CFI FunCall _check_tamper1
        CALL      _check_tamper1     ;; 3 cycles
// 1586         if(bitIsSet(tpr.top_cover.flag,action_f))
        MOVW      HL, #LWRD(_tpr+592)  ;; 1 cycle
        MOV1      CY, [HL].1         ;; 1 cycle
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+6
        BNC       ??magnet_log_logic_234  ;; 4 cycles
        ; ------------------------------------- Block: 18 cycles
// 1587         {
// 1588             bitClear(tpr.top_cover.flag,action_f);
        CLR1      N:_tpr+592.1       ;; 2 cycles
// 1589             if(bitIsSet(tpr.top_cover.flag,event_f))
        MOVW      HL, #LWRD(_tpr+592)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??magnet_log_logic_235  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
// 1590             {
// 1591                 mem_log1(COMPART_NONROLLOVER,EVENT_ID_TOP_COVER_OCC);
        MOVW      BC, #0xFB          ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _mem_log1
        CALL      _mem_log1          ;; 3 cycles
        BR        S:??magnet_log_logic_236  ;; 3 cycles
        ; ------------------------------------- Block: 8 cycles
// 1592             }
// 1593             else
// 1594             {
// 1595                 mem_log1(COMPART_NONROLLOVER,EVENT_ID_TOP_COVER_RES);
??magnet_log_logic_235:
        MOVW      BC, #0xFC          ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _mem_log1
        CALL      _mem_log1          ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 1596             }
// 1597             eprom_read(0x0C90,0,PAGE_1,AUTO_CALC);    
??magnet_log_logic_236:
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xC90         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
// 1598             time_into_char_array5_sec(tpr.top_cover.time,&opr_data[0]);
        MOVW      HL, #LWRD(_tpr+594)  ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+14
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      AX, #LWRD(_opr_data)  ;; 1 cycle
          CFI FunCall _time_into_char_array5_sec
        CALL      _time_into_char_array5_sec  ;; 3 cycles
// 1599             eprom_write(0x0C90,0,16,PAGE_1,AUTO_CALC);          
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+16
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xC90         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
// 1600             update_tamper_variables();
          CFI FunCall _update_tamper_variables
        CALL      _update_tamper_variables  ;; 3 cycles
        ADDW      SP, #0xA           ;; 1 cycle
          CFI CFA SP+6
        BR        S:??magnet_log_logic_234  ;; 3 cycles
        ; ------------------------------------- Block: 34 cycles
// 1601         }
// 1602     }
// 1603     else
// 1604     {
// 1605         tpr.top_cover.flag = 0;
??magnet_log_logic_231:
        MOV       N:_tpr+592, #0x0   ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
// 1606     }
// 1607 }
??magnet_log_logic_234:
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock6
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 120 cycles
        REQUIRE __A_P4

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock7 Using cfiCommon1
          CFI Function _top_cover_restore
        CODE
// 1608 void top_cover_restore()
// 1609 {
_top_cover_restore:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
// 1610   if(top_cover_restore_command == 1)
        CMP       N:_top_cover_restore_command, #0x1  ;; 1 cycle
        SKZ                          ;; 4 cycles
        BR        N:??magnet_log_logic_237  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 1611   {
// 1612     if(TOP_COVER_ATTACHED && TOP_RESTORE_REQ == 1 && bitIsSet(tpr.top_cover.flag,event_f))
        MOV       A, S:0xFFF04       ;; 1 cycle
        AND       A, #0x4            ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        BNZ       ??magnet_log_logic_237  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        CMP       N:_TOP_RESTORE_REQ, #0x1  ;; 1 cycle
        BNZ       ??magnet_log_logic_237  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        MOVW      HL, #LWRD(_tpr+592)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??magnet_log_logic_237  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1613     {
// 1614       top_cover_restore_command = 0;
        MOV       N:_top_cover_restore_command, #0x0  ;; 1 cycle
// 1615       bitClear(tamper_instant_status,BIT_TOP_COVER);
        CLR1      N:_tamper_instant_status+2.4  ;; 2 cycles
// 1616       bitClear(tpr.top_cover.flag,indicate_f);
        CLR1      N:_tpr+592.2       ;; 2 cycles
// 1617       bitClear(tpr.top_cover.flag,event_f);
        CLR1      N:_tpr+592.0       ;; 2 cycles
// 1618       
// 1619       tpr.top_cover.time = Now;
        MOVW      HL, #LWRD(_tpr+594)  ;; 1 cycle
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
// 1620       mem_log1(COMPART_NONROLLOVER,EVENT_ID_TOP_COVER_RES);
        MOVW      BC, #0xFC          ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _mem_log1
        CALL      _mem_log1          ;; 3 cycles
// 1621       eprom_read(0x0C90,0,PAGE_1,AUTO_CALC);    
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xC90         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
// 1622       time_into_char_array5_sec(tpr.top_cover.time,&opr_data[0]);
        MOVW      HL, #LWRD(_tpr+594)  ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+12
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      AX, #LWRD(_opr_data)  ;; 1 cycle
          CFI FunCall _time_into_char_array5_sec
        CALL      _time_into_char_array5_sec  ;; 3 cycles
// 1623       eprom_write(0x0C90,0,16,PAGE_1,AUTO_CALC);          
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+14
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xC90         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
// 1624       update_tamper_variables();
          CFI FunCall _update_tamper_variables
        CALL      _update_tamper_variables  ;; 3 cycles
        ADDW      SP, #0xA           ;; 1 cycle
          CFI CFA SP+4
        ; ------------------------------------- Block: 59 cycles
// 1625     }
// 1626   }
// 1627 
// 1628 }
??magnet_log_logic_237:
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock7
        ; ------------------------------------- Block: 6 cycles
        ; ------------------------------------- Total: 88 cycles
        REQUIRE __A_P4

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock8 Using cfiCommon2
          CFI Function _check_tamper
          CFI NoCalls
        CODE
// 1629 __near_func void check_tamper(Condition condition, TAMPER_VAR *tamper, us16 str_time, us16 rstr_time, us16 indicate_time)
// 1630 {
_check_tamper:
        ; * Stack frame (at entry) *
        ; Param size: 4
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+6
        PUSH      DE                 ;; 1 cycle
          CFI CFA SP+8
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        ; Auto size: 6
// 1631   if(condition == OCCUR && bitIsClear(tamper->flag,event_f))              /* Occur */
        MOV       A, [SP+0x01]       ;; 1 cycle
        CMP       A, #0x1            ;; 1 cycle
        SKZ                          ;; 4 cycles
        BR        N:??magnet_log_logic_238  ;; 4 cycles
        ; ------------------------------------- Block: 9 cycles
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        SKNC                         ;; 4 cycles
        BR        N:??magnet_log_logic_238  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
// 1632   {
// 1633     tamper->timer++;
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, [DE+0x0A]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        INCW      HL                 ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      [DE+0x0A], AX      ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
// 1634     if(tamper->timer == 1)
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [HL+0x0A]      ;; 1 cycle
        CMPW      AX, #0x1           ;; 1 cycle
        BNZ       ??magnet_log_logic_239  ;; 4 cycles
        ; ------------------------------------- Block: 18 cycles
// 1635     {
// 1636       tamper->time = Now;
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        ADDW      AX, #0x2           ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
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
// 1637       tamper->kwh_import = energy.Allph.active_imp;
        MOVW      BC, N:_energy+54   ;; 1 cycle
        MOVW      AX, N:_energy+52   ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      [HL+0x0C], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x0E], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1638       tamper->kwh_export = energy.Allph.active_exp;
        MOVW      BC, N:_energy+58   ;; 1 cycle
        MOVW      AX, N:_energy+56   ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      [HL+0x10], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x12], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1639       tamper->kvah_import = energy.Allph.apparent_imp;
        MOVW      BC, N:_energy+70   ;; 1 cycle
        MOVW      AX, N:_energy+68   ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      [HL+0x14], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x16], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1640       tamper->kvah_export = energy.Allph.apparent_exp;
        MOVW      BC, N:_energy+74   ;; 1 cycle
        MOVW      AX, N:_energy+72   ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      [HL+0x18], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x1A], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ; ------------------------------------- Block: 54 cycles
// 1641     }
// 1642     if(tamper->timer >= indicate_time)
??magnet_log_logic_239:
        MOVW      AX, [SP+0x0C]      ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [HL+0x0A]      ;; 1 cycle
        CMPW      AX, DE             ;; 1 cycle
        BC        ??magnet_log_logic_240  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
// 1643     {
// 1644       bitSet(tamper->flag,indicate_f);
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        SET1      [HL].2             ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
// 1645     }
// 1646     if(tamper->timer >= str_time)
??magnet_log_logic_240:
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [HL+0x0A]      ;; 1 cycle
        CMPW      AX, DE             ;; 1 cycle
        BC        ??magnet_log_logic_238  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
// 1647     {
// 1648       tamper->timer = 0;
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x0A], AX      ;; 1 cycle
// 1649       bitSet(tamper->flag,event_f);
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        SET1      [HL].0             ;; 2 cycles
// 1650       bitSet(tamper->flag,action_f);
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        SET1      [HL].1             ;; 2 cycles
        ; ------------------------------------- Block: 12 cycles
// 1651     }
// 1652   }
// 1653   /* Timer Clearing */
// 1654   if((condition == OCCUR && bitIsSet(tamper->flag,event_f)) ||          /* Already occured */ 
// 1655      (condition == RESTORE && bitIsClear(tamper->flag,event_f)) ||      /* Already restore */
// 1656        (condition == NOT_AVAIL))                                        /* Condition does not match */
??magnet_log_logic_238:
        MOV       A, [SP+0x01]       ;; 1 cycle
        CMP       A, #0x1            ;; 1 cycle
        BNZ       ??magnet_log_logic_241  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BC        ??magnet_log_logic_242  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
??magnet_log_logic_241:
        MOV       A, [SP+0x01]       ;; 1 cycle
        CMP       A, #0x2            ;; 1 cycle
        BNZ       ??magnet_log_logic_243  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??magnet_log_logic_242  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
??magnet_log_logic_243:
        MOV       A, [SP+0x01]       ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        BNZ       ??magnet_log_logic_244  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1657   {
// 1658     tamper->timer = 0;
??magnet_log_logic_242:
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x0A], AX      ;; 1 cycle
// 1659     /* Handling indicate flag */
// 1660     if(condition == OCCUR)
        MOV       A, [SP+0x01]       ;; 1 cycle
        CMP       A, #0x1            ;; 1 cycle
        BNZ       ??magnet_log_logic_245  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
// 1661     {
// 1662       if(bitIsClear(tamper->flag,indicate_f))
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV1      CY, [HL].2         ;; 1 cycle
        BC        ??magnet_log_logic_244  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
// 1663       {
// 1664         bitSet(tamper->flag,indicate_f);
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        SET1      [HL].2             ;; 2 cycles
        BR        S:??magnet_log_logic_244  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 1665       }
// 1666     }
// 1667     else if(condition == RESTORE)
??magnet_log_logic_245:
        MOV       A, [SP+0x01]       ;; 1 cycle
        CMP       A, #0x2            ;; 1 cycle
        BNZ       ??magnet_log_logic_246  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1668     {
// 1669       if(bitIsSet(tamper->flag,indicate_f))
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV1      CY, [HL].2         ;; 1 cycle
        BNC       ??magnet_log_logic_244  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
// 1670       {
// 1671         bitClear(tamper->flag,indicate_f);
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        CLR1      [HL].2             ;; 2 cycles
        BR        S:??magnet_log_logic_244  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 1672       }
// 1673     }
// 1674     else if(condition == NOT_AVAIL)
??magnet_log_logic_246:
        MOV       A, [SP+0x01]       ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        BNZ       ??magnet_log_logic_244  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1675     {
// 1676       if(bitIsSet(tamper->flag,event_f))
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??magnet_log_logic_247  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
// 1677       {
// 1678         bitSet(tamper->flag,indicate_f);
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        SET1      [HL].2             ;; 2 cycles
        BR        S:??magnet_log_logic_244  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 1679       }
// 1680       else
// 1681       {
// 1682         bitClear(tamper->flag,indicate_f);
??magnet_log_logic_247:
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        CLR1      [HL].2             ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
// 1683       }
// 1684     }
// 1685   }
// 1686   if(condition == RESTORE && bitIsSet(tamper->flag,event_f))              /* Restore */
??magnet_log_logic_244:
        MOV       A, [SP+0x01]       ;; 1 cycle
        CMP       A, #0x2            ;; 1 cycle
        SKZ                          ;; 4 cycles
        BR        N:??magnet_log_logic_248  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        SKC                          ;; 4 cycles
        BR        N:??magnet_log_logic_248  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
// 1687   {
// 1688     tamper->timer++;
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, [DE+0x0A]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        INCW      HL                 ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      [DE+0x0A], AX      ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
// 1689     if(tamper->timer == 1)
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [HL+0x0A]      ;; 1 cycle
        CMPW      AX, #0x1           ;; 1 cycle
        BNZ       ??magnet_log_logic_249  ;; 4 cycles
        ; ------------------------------------- Block: 18 cycles
// 1690     {
// 1691       tamper->time = Now;
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        ADDW      AX, #0x2           ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
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
// 1692       tamper->kwh_import = energy.Allph.active_imp;
        MOVW      BC, N:_energy+54   ;; 1 cycle
        MOVW      AX, N:_energy+52   ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      [HL+0x0C], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x0E], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1693       tamper->kwh_export = energy.Allph.active_exp;
        MOVW      BC, N:_energy+58   ;; 1 cycle
        MOVW      AX, N:_energy+56   ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      [HL+0x10], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x12], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1694       tamper->kvah_import = energy.Allph.apparent_imp;
        MOVW      BC, N:_energy+70   ;; 1 cycle
        MOVW      AX, N:_energy+68   ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      [HL+0x14], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x16], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1695       tamper->kvah_export = energy.Allph.apparent_exp;
        MOVW      BC, N:_energy+74   ;; 1 cycle
        MOVW      AX, N:_energy+72   ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      [HL+0x18], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x1A], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ; ------------------------------------- Block: 54 cycles
// 1696     }
// 1697     if(tamper->timer >= indicate_time)
??magnet_log_logic_249:
        MOVW      AX, [SP+0x0C]      ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [HL+0x0A]      ;; 1 cycle
        CMPW      AX, DE             ;; 1 cycle
        BC        ??magnet_log_logic_250  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
// 1698     {
// 1699       bitClear(tamper->flag,indicate_f);
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        CLR1      [HL].2             ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
// 1700     }
// 1701     if(tamper->timer >= rstr_time)
??magnet_log_logic_250:
        MOVW      AX, [SP+0x0A]      ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [HL+0x0A]      ;; 1 cycle
        CMPW      AX, DE             ;; 1 cycle
        BC        ??magnet_log_logic_248  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
// 1702     {
// 1703       tamper->timer = 0;
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x0A], AX      ;; 1 cycle
// 1704       bitClear(tamper->flag,event_f);
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        CLR1      [HL].0             ;; 2 cycles
// 1705       bitSet(tamper->flag,action_f);
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        SET1      [HL].1             ;; 2 cycles
        ; ------------------------------------- Block: 12 cycles
// 1706       /* bitClear(tamper->flag,indicate_f); */
// 1707     }
// 1708   }
// 1709 }
??magnet_log_logic_248:
        ADDW      SP, #0x6           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock8
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 352 cycles
// 1710 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock9 Using cfiCommon2
          CFI Function _check_tamper1
          CFI NoCalls
        CODE
// 1711 __near_func void check_tamper1(Condition condition, TAMPER_VAR1 *tamper, us16 str_time, us16 rstr_time, us16 indicate_time)
// 1712 {
_check_tamper1:
        ; * Stack frame (at entry) *
        ; Param size: 4
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+6
        PUSH      DE                 ;; 1 cycle
          CFI CFA SP+8
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        ; Auto size: 6
        MOVW      AX, [SP+0x0C]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
// 1713   if(condition == OCCUR && bitIsClear(tamper->flag,event_f))              /* Occur */
        MOV       A, [SP+0x01]       ;; 1 cycle
        CMP       A, #0x1            ;; 1 cycle
        BNZ       ??magnet_log_logic_251  ;; 4 cycles
        ; ------------------------------------- Block: 11 cycles
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BC        ??magnet_log_logic_251  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
// 1714   {
// 1715     tamper->timer++;
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [HL+0x0A]      ;; 1 cycle
        INCW      AX                 ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        MOVW      [HL+0x0A], AX      ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
// 1716     if(tamper->timer == 1)
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [HL+0x0A]      ;; 1 cycle
        CMPW      AX, #0x1           ;; 1 cycle
        BNZ       ??magnet_log_logic_252  ;; 4 cycles
        ; ------------------------------------- Block: 18 cycles
// 1717     {
// 1718       tamper->time = Now;
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        ADDW      AX, #0x2           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
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
        ; ------------------------------------- Block: 20 cycles
// 1719     }
// 1720     if(tamper->timer >= indicate_time)
??magnet_log_logic_252:
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [HL+0x0A]      ;; 1 cycle
        CMPW      AX, BC             ;; 1 cycle
        BC        ??magnet_log_logic_253  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
// 1721     {
// 1722       bitSet(tamper->flag,indicate_f);
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        SET1      [HL].2             ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
// 1723     }
// 1724     if(tamper->timer >= str_time)
??magnet_log_logic_253:
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [HL+0x0A]      ;; 1 cycle
        CMPW      AX, DE             ;; 1 cycle
        BC        ??magnet_log_logic_251  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
// 1725     {
// 1726       tamper->timer = 0;
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x0A], AX      ;; 1 cycle
// 1727       bitSet(tamper->flag,event_f);
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        SET1      [HL].0             ;; 2 cycles
// 1728       bitSet(tamper->flag,action_f);
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        SET1      [HL].1             ;; 2 cycles
        ; ------------------------------------- Block: 12 cycles
// 1729     }
// 1730   }
// 1731   /* Timer Clearing */
// 1732   if((condition == OCCUR && bitIsSet(tamper->flag,event_f)) ||          /* Already occured */ 
// 1733      (condition == RESTORE && bitIsClear(tamper->flag,event_f)) ||      /* Already restore */
// 1734        (condition == NOT_AVAIL))                                        /* Condition does not match */
??magnet_log_logic_251:
        MOV       A, [SP+0x01]       ;; 1 cycle
        CMP       A, #0x1            ;; 1 cycle
        BNZ       ??magnet_log_logic_254  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BC        ??magnet_log_logic_255  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
??magnet_log_logic_254:
        MOV       A, [SP+0x01]       ;; 1 cycle
        CMP       A, #0x2            ;; 1 cycle
        BNZ       ??magnet_log_logic_256  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??magnet_log_logic_255  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
??magnet_log_logic_256:
        MOV       A, [SP+0x01]       ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        BNZ       ??magnet_log_logic_257  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1735   {
// 1736     tamper->timer = 0;
??magnet_log_logic_255:
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x0A], AX      ;; 1 cycle
// 1737     /* Handling indicate flag */
// 1738     if(condition == OCCUR)
        MOV       A, [SP+0x01]       ;; 1 cycle
        CMP       A, #0x1            ;; 1 cycle
        BNZ       ??magnet_log_logic_258  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
// 1739     {
// 1740       if(bitIsClear(tamper->flag,indicate_f))
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV1      CY, [HL].2         ;; 1 cycle
        BC        ??magnet_log_logic_257  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
// 1741       {
// 1742         bitSet(tamper->flag,indicate_f);
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        SET1      [HL].2             ;; 2 cycles
        BR        S:??magnet_log_logic_257  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 1743       }
// 1744     }
// 1745     else if(condition == RESTORE)
??magnet_log_logic_258:
        MOV       A, [SP+0x01]       ;; 1 cycle
        CMP       A, #0x2            ;; 1 cycle
        BNZ       ??magnet_log_logic_259  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1746     {
// 1747       if(bitIsSet(tamper->flag,indicate_f))
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV1      CY, [HL].2         ;; 1 cycle
        BNC       ??magnet_log_logic_257  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
// 1748       {
// 1749         bitClear(tamper->flag,indicate_f);
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        CLR1      [HL].2             ;; 2 cycles
        BR        S:??magnet_log_logic_257  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 1750       }
// 1751     }
// 1752     else if(condition == NOT_AVAIL)
??magnet_log_logic_259:
        MOV       A, [SP+0x01]       ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        BNZ       ??magnet_log_logic_257  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1753     {
// 1754       if(bitIsSet(tamper->flag,event_f))
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??magnet_log_logic_260  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
// 1755       {
// 1756         bitSet(tamper->flag,indicate_f);
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        SET1      [HL].2             ;; 2 cycles
        BR        S:??magnet_log_logic_257  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 1757       }
// 1758       else
// 1759       {
// 1760         bitClear(tamper->flag,indicate_f);
??magnet_log_logic_260:
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        CLR1      [HL].2             ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
// 1761       }
// 1762     }
// 1763   }
// 1764   if(condition == RESTORE && bitIsSet(tamper->flag,event_f))              /* Restore */
??magnet_log_logic_257:
        MOV       A, [SP+0x01]       ;; 1 cycle
        CMP       A, #0x2            ;; 1 cycle
        BNZ       ??magnet_log_logic_261  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??magnet_log_logic_261  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
// 1765   {
// 1766     tamper->timer++;
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [HL+0x0A]      ;; 1 cycle
        INCW      AX                 ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        MOVW      [HL+0x0A], AX      ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
// 1767     if(tamper->timer == 1)
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [HL+0x0A]      ;; 1 cycle
        CMPW      AX, #0x1           ;; 1 cycle
        BNZ       ??magnet_log_logic_262  ;; 4 cycles
        ; ------------------------------------- Block: 18 cycles
// 1768     {
// 1769       tamper->time = Now;
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        ADDW      AX, #0x2           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
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
        ; ------------------------------------- Block: 20 cycles
// 1770     }
// 1771     if(tamper->timer >= indicate_time)
??magnet_log_logic_262:
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [HL+0x0A]      ;; 1 cycle
        CMPW      AX, BC             ;; 1 cycle
        BC        ??magnet_log_logic_263  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
// 1772     {
// 1773       bitClear(tamper->flag,indicate_f);
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        CLR1      [HL].2             ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
// 1774     }
// 1775     if(tamper->timer >= rstr_time)
??magnet_log_logic_263:
        MOVW      AX, [SP+0x0A]      ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [HL+0x0A]      ;; 1 cycle
        CMPW      AX, DE             ;; 1 cycle
        BC        ??magnet_log_logic_261  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
// 1776     {
// 1777       tamper->timer = 0;
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x0A], AX      ;; 1 cycle
// 1778       bitClear(tamper->flag,event_f);
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        CLR1      [HL].0             ;; 2 cycles
// 1779       bitSet(tamper->flag,action_f);
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        SET1      [HL].1             ;; 2 cycles
        ; ------------------------------------- Block: 12 cycles
// 1780       /* bitClear(tamper->flag,indicate_f); */
// 1781     }
// 1782   }
// 1783 }
??magnet_log_logic_261:
        ADDW      SP, #0x6           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock9
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 282 cycles

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock10 Using cfiCommon1
          CFI Function _update_tamper_variables
        CODE
// 1784 void update_tamper_variables()
// 1785 {
_update_tamper_variables:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
// 1786   /* Save tamper flags */
// 1787   
// 1788   /* Magnet */
// 1789   if(bitIsSet(tpr.magnet.flag,event_f))
        MOVW      HL, #LWRD(_tpr+32)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??magnet_log_logic_264  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1790   {
// 1791     tamper_event_status |= BIT_MAGNET;
        SET1      N:_tamper_event_status.0  ;; 2 cycles
        BR        S:??magnet_log_logic_265  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 1792   }
// 1793   else
// 1794   {
// 1795     tamper_event_status &= ~BIT_MAGNET;
??magnet_log_logic_264:
        CLR1      N:_tamper_event_status.0  ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
// 1796   }
// 1797   
// 1798   /* Voltage Miss */
// 1799   if(bitIsSet(tpr.vol_miss.Rph.flag,event_f))
??magnet_log_logic_265:
        MOVW      HL, #LWRD(_tpr+256)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??magnet_log_logic_266  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1800   {
// 1801     tamper_event_status |= BIT_VOL_MISS_R;
        SET1      N:_tamper_event_status.1  ;; 2 cycles
        BR        S:??magnet_log_logic_267  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 1802   }
// 1803   else
// 1804   {
// 1805     tamper_event_status &= ~BIT_VOL_MISS_R;
??magnet_log_logic_266:
        CLR1      N:_tamper_event_status.1  ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
// 1806   }
// 1807   if(bitIsSet(tpr.vol_miss.Yph.flag,event_f))
??magnet_log_logic_267:
        MOVW      HL, #LWRD(_tpr+284)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??magnet_log_logic_268  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1808   {
// 1809     tamper_event_status |= BIT_VOL_MISS_Y;
        SET1      N:_tamper_event_status.2  ;; 2 cycles
        BR        S:??magnet_log_logic_269  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 1810   }
// 1811   else
// 1812   {
// 1813     tamper_event_status &= ~BIT_VOL_MISS_Y;
??magnet_log_logic_268:
        CLR1      N:_tamper_event_status.2  ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
// 1814   }
// 1815   if(bitIsSet(tpr.vol_miss.Bph.flag,event_f))
??magnet_log_logic_269:
        MOVW      HL, #LWRD(_tpr+312)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??magnet_log_logic_270  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1816   {
// 1817     tamper_event_status |= BIT_VOL_MISS_B;
        SET1      N:_tamper_event_status.3  ;; 2 cycles
        BR        S:??magnet_log_logic_271  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 1818   }
// 1819   else
// 1820   {
// 1821     tamper_event_status &= ~BIT_VOL_MISS_B;
??magnet_log_logic_270:
        CLR1      N:_tamper_event_status.3  ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
// 1822   }
// 1823   
// 1824   /* Voltage miss all */
// 1825   if(bitIsSet(tpr.vol_miss_all.flag,event_f))
??magnet_log_logic_271:
        MOVW      HL, #LWRD(_tpr+228)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??magnet_log_logic_272  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1826   {
// 1827     tamper_event_status |= BIT_VOL_MISS_ALL;
        SET1      N:_tamper_event_status.4  ;; 2 cycles
        BR        S:??magnet_log_logic_273  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 1828   }
// 1829   else
// 1830   {
// 1831     tamper_event_status &= ~BIT_VOL_MISS_ALL;
??magnet_log_logic_272:
        CLR1      N:_tamper_event_status.4  ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
// 1832   }
// 1833   
// 1834   /* Voltage Unbalance */
// 1835   if(bitIsSet(tpr.vol_unbal.flag,event_f))
??magnet_log_logic_273:
        MOVW      HL, #LWRD(_tpr+60)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??magnet_log_logic_274  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1836   {
// 1837     tamper_event_status |= BIT_VOL_UNBAL;
        SET1      N:_tamper_event_status.5  ;; 2 cycles
        BR        S:??magnet_log_logic_275  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 1838   }
// 1839   else
// 1840   {
// 1841     tamper_event_status &= ~BIT_VOL_UNBAL;
??magnet_log_logic_274:
        CLR1      N:_tamper_event_status.5  ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
// 1842   }
// 1843   
// 1844   /* Voltage High */
// 1845   if(bitIsSet(tpr.vol_high.flag,event_f))
??magnet_log_logic_275:
        MOVW      HL, #LWRD(_tpr+116)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??magnet_log_logic_276  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1846   {
// 1847     tamper_event_status |= BIT_VOL_HIGH;
        SET1      N:_tamper_event_status.6  ;; 2 cycles
        BR        S:??magnet_log_logic_277  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 1848   }
// 1849   else
// 1850   {
// 1851     tamper_event_status &= ~BIT_VOL_HIGH;
??magnet_log_logic_276:
        CLR1      N:_tamper_event_status.6  ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
// 1852   }
// 1853   
// 1854   /* Voltage Low */
// 1855   if(bitIsSet(tpr.vol_low.flag,event_f))
??magnet_log_logic_277:
        MOVW      HL, #LWRD(_tpr+88)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??magnet_log_logic_278  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1856   {
// 1857     tamper_event_status |= BIT_VOL_LOW;
        SET1      N:_tamper_event_status.7  ;; 2 cycles
        BR        S:??magnet_log_logic_279  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 1858   }
// 1859   else
// 1860   {
// 1861     tamper_event_status &= ~BIT_VOL_LOW;
??magnet_log_logic_278:
        CLR1      N:_tamper_event_status.7  ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
// 1862   }
// 1863   
// 1864   /* Current Unbalance */
// 1865   if(bitIsSet(tpr.curr_unbal.flag,event_f))
??magnet_log_logic_279:
        MOVW      HL, #LWRD(_tpr+144)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??magnet_log_logic_280  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1866   {
// 1867     tamper_event_status |= BIT_CURR_UNBAL;
        SET1      N:_tamper_event_status+1.0  ;; 2 cycles
        BR        S:??magnet_log_logic_281  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 1868   }
// 1869   else
// 1870   {
// 1871     tamper_event_status &= ~BIT_CURR_UNBAL;
??magnet_log_logic_280:
        CLR1      N:_tamper_event_status+1.0  ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
// 1872   }
// 1873   
// 1874   /* CT Bypass */
// 1875   if(bitIsSet(tpr.ct_bypass.flag,event_f))
??magnet_log_logic_281:
        MOVW      HL, #LWRD(_tpr+172)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??magnet_log_logic_282  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1876   {
// 1877     tamper_event_status |= BIT_CT_BYPASS;
        SET1      N:_tamper_event_status+1.1  ;; 2 cycles
        BR        S:??magnet_log_logic_283  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 1878   }
// 1879   else
// 1880   {
// 1881     tamper_event_status &= ~BIT_CT_BYPASS;
??magnet_log_logic_282:
        CLR1      N:_tamper_event_status+1.1  ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
// 1882   }
// 1883   
// 1884   /* CT rev */
// 1885   if(bitIsSet(tpr.ct_rev.Rph.flag,event_f))
??magnet_log_logic_283:
        MOVW      HL, #LWRD(_tpr+340)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??magnet_log_logic_284  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1886   {
// 1887     tamper_event_status |= BIT_CT_REV_R;
        SET1      N:_tamper_event_status+1.2  ;; 2 cycles
        BR        S:??magnet_log_logic_285  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 1888   }
// 1889   else
// 1890   {
// 1891     tamper_event_status &= ~BIT_CT_REV_R;
??magnet_log_logic_284:
        CLR1      N:_tamper_event_status+1.2  ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
// 1892   }
// 1893   if(bitIsSet(tpr.ct_rev.Yph.flag,event_f))
??magnet_log_logic_285:
        MOVW      HL, #LWRD(_tpr+368)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??magnet_log_logic_286  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1894   {
// 1895     tamper_event_status |= BIT_CT_REV_Y;
        SET1      N:_tamper_event_status+1.3  ;; 2 cycles
        BR        S:??magnet_log_logic_287  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 1896   }
// 1897   else
// 1898   {
// 1899     tamper_event_status &= ~BIT_CT_REV_Y;
??magnet_log_logic_286:
        CLR1      N:_tamper_event_status+1.3  ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
// 1900   }
// 1901   if(bitIsSet(tpr.ct_rev.Bph.flag,event_f))
??magnet_log_logic_287:
        MOVW      HL, #LWRD(_tpr+396)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??magnet_log_logic_288  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1902   {
// 1903     tamper_event_status |= BIT_CT_REV_B;
        SET1      N:_tamper_event_status+1.4  ;; 2 cycles
        BR        S:??magnet_log_logic_289  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 1904   }
// 1905   else
// 1906   {
// 1907     tamper_event_status &= ~BIT_CT_REV_B;
??magnet_log_logic_288:
        CLR1      N:_tamper_event_status+1.4  ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
// 1908   }
// 1909   
// 1910   /* CT Open */
// 1911   if(bitIsSet(tpr.ct_open.Rph.flag,event_f))
??magnet_log_logic_289:
        MOVW      HL, #LWRD(_tpr+424)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??magnet_log_logic_290  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1912   {
// 1913     tamper_event_status |= BIT_CT_OPEN_R;
        SET1      N:_tamper_event_status+1.5  ;; 2 cycles
        BR        S:??magnet_log_logic_291  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 1914   }
// 1915   else
// 1916   {
// 1917     tamper_event_status &= ~BIT_CT_OPEN_R;
??magnet_log_logic_290:
        CLR1      N:_tamper_event_status+1.5  ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
// 1918   }
// 1919   if(bitIsSet(tpr.ct_open.Yph.flag,event_f))
??magnet_log_logic_291:
        MOVW      HL, #LWRD(_tpr+452)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??magnet_log_logic_292  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1920   {
// 1921     tamper_event_status |= BIT_CT_OPEN_Y;
        SET1      N:_tamper_event_status+1.6  ;; 2 cycles
        BR        S:??magnet_log_logic_293  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 1922   }
// 1923   else
// 1924   {
// 1925     tamper_event_status &= ~BIT_CT_OPEN_Y;
??magnet_log_logic_292:
        CLR1      N:_tamper_event_status+1.6  ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
// 1926   }
// 1927   if(bitIsSet(tpr.ct_open.Bph.flag,event_f))
??magnet_log_logic_293:
        MOVW      HL, #LWRD(_tpr+480)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??magnet_log_logic_294  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1928   {
// 1929     tamper_event_status |= BIT_CT_OPEN_B;
        SET1      N:_tamper_event_status+1.7  ;; 2 cycles
        BR        S:??magnet_log_logic_295  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 1930   }
// 1931   else
// 1932   {
// 1933     tamper_event_status &= ~BIT_CT_OPEN_B;
??magnet_log_logic_294:
        MOVW      AX, #0x0           ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+6
        MOVW      AX, #0x7FFF        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+8
        MOVW      BC, N:_tamper_event_status+2  ;; 1 cycle
        MOVW      AX, N:_tamper_event_status  ;; 1 cycle
          CFI FunCall ?L_AND_FAST_L03
        CALL      N:?L_AND_FAST_L03  ;; 3 cycles
        MOVW      N:_tamper_event_status, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_tamper_event_status+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+4
        ; ------------------------------------- Block: 14 cycles
// 1934   }
// 1935   
// 1936   /* Over Current */
// 1937   if(bitIsSet(tpr.curr_over.Rph.flag,event_f))
??magnet_log_logic_295:
        MOVW      HL, #LWRD(_tpr+508)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??magnet_log_logic_296  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1938   {
// 1939     tamper_event_status |= BIT_CURR_OVER_R;
        SET1      N:_tamper_event_status+2.0  ;; 2 cycles
        BR        S:??magnet_log_logic_297  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 1940   }
// 1941   else
// 1942   {
// 1943     tamper_event_status &= ~BIT_CURR_OVER_R;
??magnet_log_logic_296:
        CLR1      N:_tamper_event_status+2.0  ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
// 1944   }
// 1945   if(bitIsSet(tpr.curr_over.Yph.flag,event_f))
??magnet_log_logic_297:
        MOVW      HL, #LWRD(_tpr+536)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??magnet_log_logic_298  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1946   {
// 1947     tamper_event_status |= BIT_CURR_OVER_Y;
        SET1      N:_tamper_event_status+2.1  ;; 2 cycles
        BR        S:??magnet_log_logic_299  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 1948   }
// 1949   else
// 1950   {
// 1951     tamper_event_status &= ~BIT_CURR_OVER_Y;
??magnet_log_logic_298:
        CLR1      N:_tamper_event_status+2.1  ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
// 1952   }
// 1953   if(bitIsSet(tpr.curr_over.Bph.flag,event_f))
??magnet_log_logic_299:
        MOVW      HL, #LWRD(_tpr+564)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??magnet_log_logic_300  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1954   {
// 1955     tamper_event_status |= BIT_CURR_OVER_B;
        SET1      N:_tamper_event_status+2.2  ;; 2 cycles
        BR        S:??magnet_log_logic_301  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 1956   }
// 1957   else
// 1958   {
// 1959     tamper_event_status &= ~BIT_CURR_OVER_B;
??magnet_log_logic_300:
        CLR1      N:_tamper_event_status+2.2  ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
// 1960   }
// 1961   
// 1962   /* Neutral Disturb */
// 1963   if(bitIsSet(tpr.neu_disturb.flag,event_f))
??magnet_log_logic_301:
        MOVW      HL, #LWRD(_tpr+200)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??magnet_log_logic_302  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1964   {
// 1965     tamper_event_status |= BIT_NEU_DISTURB;
        SET1      N:_tamper_event_status+2.3  ;; 2 cycles
        BR        S:??magnet_log_logic_303  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 1966   }
// 1967   else
// 1968   {
// 1969     tamper_event_status &= ~BIT_NEU_DISTURB;
??magnet_log_logic_302:
        CLR1      N:_tamper_event_status+2.3  ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
// 1970   }
// 1971   
// 1972   /* TOP Cover */
// 1973   if(bitIsSet(tpr.top_cover.flag,event_f))
??magnet_log_logic_303:
        MOVW      HL, #LWRD(_tpr+592)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??magnet_log_logic_304  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1974   {
// 1975     tamper_event_status |= BIT_TOP_COVER;
        SET1      N:_tamper_event_status+2.4  ;; 2 cycles
        BR        S:??magnet_log_logic_305  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 1976   }
// 1977   else
// 1978   {
// 1979     tamper_event_status &= ~BIT_TOP_COVER;
??magnet_log_logic_304:
        CLR1      N:_tamper_event_status+2.4  ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
// 1980   }
// 1981   
// 1982   /* Battery Low */
// 1983   if(bitIsSet(tpr.battery_low.flag,event_f))
??magnet_log_logic_305:
        MOVW      HL, #LWRD(_tpr+616)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??magnet_log_logic_306  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1984   {
// 1985     tamper_event_status |= BIT_BATTERY_LOW;
        SET1      N:_tamper_event_status+2.5  ;; 2 cycles
        BR        S:??magnet_log_logic_307  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 1986   }
// 1987   else
// 1988   {
// 1989     tamper_event_status &= ~BIT_BATTERY_LOW;
??magnet_log_logic_306:
        CLR1      N:_tamper_event_status+2.5  ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
// 1990   }
// 1991   
// 1992   Eprom_ReadWM(0x0C10,0,16);
??magnet_log_logic_307:
        MOV       B, #0x10           ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xC10         ;; 1 cycle
          CFI FunCall _Eprom_ReadWM
        CALL      _Eprom_ReadWM      ;; 3 cycles
// 1993   long_into_char_array4(tamper_event_status,&opr_data[0]);
        MOVW      DE, #LWRD(_opr_data)  ;; 1 cycle
        MOVW      BC, N:_tamper_event_status+2  ;; 1 cycle
        MOVW      AX, N:_tamper_event_status  ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 1994   Eprom_WriteWM(0x0C10,0,16);
        MOV       B, #0x10           ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xC10         ;; 1 cycle
          CFI FunCall _Eprom_WriteWM
        CALL      _Eprom_WriteWM     ;; 3 cycles
// 1995   /* Saving tamper counts */
// 1996   
// 1997 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock10
        ; ------------------------------------- Block: 24 cycles
        ; ------------------------------------- Total: 322 cycles
// 1998 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock11 Using cfiCommon1
          CFI Function _read_tamper_variables
        CODE
// 1999 void read_tamper_variables()
// 2000 {
_read_tamper_variables:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
// 2001   /* Reading tamper flags */
// 2002   Eprom_ReadWM(0x0C10,0,16);
        MOV       B, #0x10           ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xC10         ;; 1 cycle
          CFI FunCall _Eprom_ReadWM
        CALL      _Eprom_ReadWM      ;; 3 cycles
// 2003   tamper_event_status = char_array_to_long4(&opr_data[0]);
        MOVW      AX, #LWRD(_opr_data)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_tamper_event_status, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_tamper_event_status+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 2004   if(tamper_event_status == 0xFFFFFFFF)
        MOVW      BC, N:_tamper_event_status+2  ;; 1 cycle
        MOVW      AX, N:_tamper_event_status  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0xFFFF        ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 20 cycles
        CMPW      AX, #0xFFFF        ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??read_tamper_variables_0:
        BNZ       ??magnet_log_logic_308  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
// 2005   {
// 2006     tamper_event_status = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_tamper_event_status, AX  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_tamper_event_status+2, AX  ;; 1 cycle
        ; ------------------------------------- Block: 4 cycles
// 2007   }
// 2008   /* Magnet tamper */
// 2009   if(tamper_event_status & BIT_MAGNET)
??magnet_log_logic_308:
        MOVW      HL, #LWRD(_tamper_event_status)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??magnet_log_logic_309  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 2010   {
// 2011     bitSet(tpr.magnet.flag,event_f);
        SET1      N:_tpr+32.0        ;; 2 cycles
// 2012     bitSet(tpr.magnet.flag,indicate_f);
        SET1      N:_tpr+32.2        ;; 2 cycles
        BR        S:??magnet_log_logic_310  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 2013   }
// 2014   else
// 2015   {
// 2016     bitClear(tpr.magnet.flag,event_f);
??magnet_log_logic_309:
        CLR1      N:_tpr+32.0        ;; 2 cycles
// 2017     bitClear(tpr.magnet.flag,indicate_f);
        CLR1      N:_tpr+32.2        ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
// 2018   }
// 2019   
// 2020   /* Voltage Miss */
// 2021   if(tamper_event_status & BIT_VOL_MISS_R)
??magnet_log_logic_310:
        MOVW      HL, #LWRD(_tamper_event_status)  ;; 1 cycle
        MOV1      CY, [HL].1         ;; 1 cycle
        BNC       ??magnet_log_logic_311  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 2022   {
// 2023     bitSet(tpr.vol_miss.Rph.flag,event_f);
        SET1      N:_tpr+256.0       ;; 2 cycles
// 2024     bitSet(tpr.vol_miss.Rph.flag,indicate_f);
        SET1      N:_tpr+256.2       ;; 2 cycles
        BR        S:??magnet_log_logic_312  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 2025   }
// 2026   else
// 2027   {
// 2028     bitClear(tpr.vol_miss.Rph.flag,event_f);
??magnet_log_logic_311:
        CLR1      N:_tpr+256.0       ;; 2 cycles
// 2029     bitClear(tpr.vol_miss.Rph.flag,indicate_f);
        CLR1      N:_tpr+256.2       ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
// 2030   }
// 2031   if(tamper_event_status & BIT_VOL_MISS_Y)
??magnet_log_logic_312:
        MOVW      HL, #LWRD(_tamper_event_status)  ;; 1 cycle
        MOV1      CY, [HL].2         ;; 1 cycle
        BNC       ??magnet_log_logic_313  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 2032   {
// 2033     bitSet(tpr.vol_miss.Yph.flag,event_f);
        SET1      N:_tpr+284.0       ;; 2 cycles
// 2034     bitSet(tpr.vol_miss.Yph.flag,indicate_f);
        SET1      N:_tpr+284.2       ;; 2 cycles
        BR        S:??magnet_log_logic_314  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 2035   }
// 2036   else
// 2037   {
// 2038     bitClear(tpr.vol_miss.Yph.flag,event_f);
??magnet_log_logic_313:
        CLR1      N:_tpr+284.0       ;; 2 cycles
// 2039     bitClear(tpr.vol_miss.Yph.flag,indicate_f);
        CLR1      N:_tpr+284.2       ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
// 2040   }
// 2041   if(tamper_event_status & BIT_VOL_MISS_B)
??magnet_log_logic_314:
        MOVW      HL, #LWRD(_tamper_event_status)  ;; 1 cycle
        MOV1      CY, [HL].3         ;; 1 cycle
        BNC       ??magnet_log_logic_315  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 2042   {
// 2043     bitSet(tpr.vol_miss.Bph.flag,event_f);
        SET1      N:_tpr+312.0       ;; 2 cycles
// 2044     bitSet(tpr.vol_miss.Bph.flag,indicate_f);
        SET1      N:_tpr+312.2       ;; 2 cycles
        BR        S:??magnet_log_logic_316  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 2045   }
// 2046   else
// 2047   {
// 2048     bitClear(tpr.vol_miss.Bph.flag,event_f);
??magnet_log_logic_315:
        CLR1      N:_tpr+312.0       ;; 2 cycles
// 2049     bitClear(tpr.vol_miss.Bph.flag,indicate_f);
        CLR1      N:_tpr+312.2       ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
// 2050   }
// 2051   
// 2052   /* Voltage miss all */
// 2053   if(tamper_event_status & BIT_VOL_MISS_ALL)
??magnet_log_logic_316:
        MOVW      HL, #LWRD(_tamper_event_status)  ;; 1 cycle
        MOV1      CY, [HL].4         ;; 1 cycle
        BNC       ??magnet_log_logic_317  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 2054   {
// 2055     bitSet(tpr.vol_miss_all.flag,event_f);
        SET1      N:_tpr+228.0       ;; 2 cycles
// 2056     bitSet(tpr.vol_miss_all.flag,indicate_f);
        SET1      N:_tpr+228.2       ;; 2 cycles
        BR        S:??magnet_log_logic_318  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 2057   }
// 2058   else
// 2059   {
// 2060     bitClear(tpr.vol_miss_all.flag,event_f);
??magnet_log_logic_317:
        CLR1      N:_tpr+228.0       ;; 2 cycles
// 2061     bitClear(tpr.vol_miss_all.flag,indicate_f);
        CLR1      N:_tpr+228.2       ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
// 2062   }
// 2063   
// 2064   /* Voltage Unbalance */
// 2065   if(tamper_event_status & BIT_VOL_UNBAL)
??magnet_log_logic_318:
        MOVW      HL, #LWRD(_tamper_event_status)  ;; 1 cycle
        MOV1      CY, [HL].5         ;; 1 cycle
        BNC       ??magnet_log_logic_319  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 2066   {
// 2067     bitSet(tpr.vol_unbal.flag,event_f);
        SET1      N:_tpr+60.0        ;; 2 cycles
// 2068     bitSet(tpr.vol_unbal.flag,indicate_f);
        SET1      N:_tpr+60.2        ;; 2 cycles
        BR        S:??magnet_log_logic_320  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 2069   }
// 2070   else
// 2071   {
// 2072     bitClear(tpr.vol_unbal.flag,event_f);
??magnet_log_logic_319:
        CLR1      N:_tpr+60.0        ;; 2 cycles
// 2073     bitClear(tpr.vol_unbal.flag,indicate_f);
        CLR1      N:_tpr+60.2        ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
// 2074   }
// 2075   
// 2076   /* Voltage High */
// 2077   if(tamper_event_status & BIT_VOL_HIGH)
??magnet_log_logic_320:
        MOVW      HL, #LWRD(_tamper_event_status)  ;; 1 cycle
        MOV1      CY, [HL].6         ;; 1 cycle
        BNC       ??magnet_log_logic_321  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 2078   {
// 2079     bitSet(tpr.vol_high.flag,event_f);
        SET1      N:_tpr+116.0       ;; 2 cycles
// 2080     bitSet(tpr.vol_high.flag,indicate_f);
        SET1      N:_tpr+116.2       ;; 2 cycles
        BR        S:??magnet_log_logic_322  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 2081   }
// 2082   else
// 2083   {
// 2084     bitClear(tpr.vol_high.flag,event_f);
??magnet_log_logic_321:
        CLR1      N:_tpr+116.0       ;; 2 cycles
// 2085     bitClear(tpr.vol_high.flag,indicate_f);
        CLR1      N:_tpr+116.2       ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
// 2086   }
// 2087   
// 2088   /* Voltage Low */
// 2089   if(tamper_event_status & BIT_VOL_LOW)
??magnet_log_logic_322:
        MOVW      HL, #LWRD(_tamper_event_status)  ;; 1 cycle
        MOV1      CY, [HL].7         ;; 1 cycle
        BNC       ??magnet_log_logic_323  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 2090   {
// 2091     bitSet(tpr.vol_low.flag,event_f);
        SET1      N:_tpr+88.0        ;; 2 cycles
// 2092     bitSet(tpr.vol_low.flag,indicate_f);
        SET1      N:_tpr+88.2        ;; 2 cycles
        BR        S:??magnet_log_logic_324  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 2093   }
// 2094   else
// 2095   {
// 2096     bitClear(tpr.vol_low.flag,event_f);
??magnet_log_logic_323:
        CLR1      N:_tpr+88.0        ;; 2 cycles
// 2097     bitClear(tpr.vol_low.flag,indicate_f);
        CLR1      N:_tpr+88.2        ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
// 2098   }
// 2099   
// 2100   /* Current Unbalance */
// 2101   if(tamper_event_status & BIT_CURR_UNBAL)
??magnet_log_logic_324:
        MOVW      HL, #LWRD(_tamper_event_status+1)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??magnet_log_logic_325  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 2102   {
// 2103     bitSet(tpr.curr_unbal.flag,event_f);
        SET1      N:_tpr+144.0       ;; 2 cycles
// 2104     bitSet(tpr.curr_unbal.flag,indicate_f);
        SET1      N:_tpr+144.2       ;; 2 cycles
        BR        S:??magnet_log_logic_326  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 2105   }
// 2106   else
// 2107   {
// 2108     bitClear(tpr.curr_unbal.flag,event_f);
??magnet_log_logic_325:
        CLR1      N:_tpr+144.0       ;; 2 cycles
// 2109     bitClear(tpr.curr_unbal.flag,indicate_f);
        CLR1      N:_tpr+144.2       ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
// 2110   }
// 2111   
// 2112   /* CT Bypass */
// 2113   if(tamper_event_status & BIT_CT_BYPASS)
??magnet_log_logic_326:
        MOVW      HL, #LWRD(_tamper_event_status+1)  ;; 1 cycle
        MOV1      CY, [HL].1         ;; 1 cycle
        BNC       ??magnet_log_logic_327  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 2114   {
// 2115     bitSet(tpr.ct_bypass.flag,event_f);
        SET1      N:_tpr+172.0       ;; 2 cycles
// 2116     bitSet(tpr.ct_bypass.flag,indicate_f);
        SET1      N:_tpr+172.2       ;; 2 cycles
        BR        S:??magnet_log_logic_328  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 2117   }
// 2118   else
// 2119   {
// 2120     bitClear(tpr.ct_bypass.flag,event_f);
??magnet_log_logic_327:
        CLR1      N:_tpr+172.0       ;; 2 cycles
// 2121     bitClear(tpr.ct_bypass.flag,indicate_f);
        CLR1      N:_tpr+172.2       ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
// 2122   }
// 2123   
// 2124   /* CT Rev */
// 2125   if(tamper_event_status & BIT_CT_REV_R)
??magnet_log_logic_328:
        MOVW      HL, #LWRD(_tamper_event_status+1)  ;; 1 cycle
        MOV1      CY, [HL].2         ;; 1 cycle
        BNC       ??magnet_log_logic_329  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 2126   {
// 2127     bitSet(tpr.ct_rev.Rph.flag,event_f);
        SET1      N:_tpr+340.0       ;; 2 cycles
// 2128     bitSet(tpr.ct_rev.Rph.flag,indicate_f);
        SET1      N:_tpr+340.2       ;; 2 cycles
        BR        S:??magnet_log_logic_330  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 2129   }
// 2130   else
// 2131   {
// 2132     bitClear(tpr.ct_rev.Rph.flag,event_f);
??magnet_log_logic_329:
        CLR1      N:_tpr+340.0       ;; 2 cycles
// 2133     bitClear(tpr.ct_rev.Rph.flag,indicate_f);
        CLR1      N:_tpr+340.2       ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
// 2134   }
// 2135   if(tamper_event_status & BIT_CT_REV_Y)
??magnet_log_logic_330:
        MOVW      HL, #LWRD(_tamper_event_status+1)  ;; 1 cycle
        MOV1      CY, [HL].3         ;; 1 cycle
        BNC       ??magnet_log_logic_331  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 2136   {
// 2137     bitSet(tpr.ct_rev.Yph.flag,event_f);
        SET1      N:_tpr+368.0       ;; 2 cycles
// 2138     bitSet(tpr.ct_rev.Yph.flag,indicate_f);
        SET1      N:_tpr+368.2       ;; 2 cycles
        BR        S:??magnet_log_logic_332  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 2139   }
// 2140   else
// 2141   {
// 2142     bitClear(tpr.ct_rev.Yph.flag,event_f);
??magnet_log_logic_331:
        CLR1      N:_tpr+368.0       ;; 2 cycles
// 2143     bitClear(tpr.ct_rev.Yph.flag,indicate_f);
        CLR1      N:_tpr+368.2       ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
// 2144   }
// 2145   if(tamper_event_status & BIT_CT_REV_B)
??magnet_log_logic_332:
        MOVW      HL, #LWRD(_tamper_event_status+1)  ;; 1 cycle
        MOV1      CY, [HL].4         ;; 1 cycle
        BNC       ??magnet_log_logic_333  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 2146   {
// 2147     bitSet(tpr.ct_rev.Bph.flag,event_f);
        SET1      N:_tpr+396.0       ;; 2 cycles
// 2148     bitSet(tpr.ct_rev.Bph.flag,indicate_f);
        SET1      N:_tpr+396.2       ;; 2 cycles
        BR        S:??magnet_log_logic_334  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 2149   }
// 2150   else
// 2151   {
// 2152     bitClear(tpr.ct_rev.Bph.flag,event_f);
??magnet_log_logic_333:
        CLR1      N:_tpr+396.0       ;; 2 cycles
// 2153     bitClear(tpr.ct_rev.Bph.flag,indicate_f);
        CLR1      N:_tpr+396.2       ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
// 2154   }
// 2155   
// 2156   /* CT Open */
// 2157   if(tamper_event_status & BIT_CT_OPEN_R)
??magnet_log_logic_334:
        MOVW      HL, #LWRD(_tamper_event_status+1)  ;; 1 cycle
        MOV1      CY, [HL].5         ;; 1 cycle
        BNC       ??magnet_log_logic_335  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 2158   {
// 2159     bitSet(tpr.ct_open.Rph.flag,event_f);
        SET1      N:_tpr+424.0       ;; 2 cycles
// 2160     bitSet(tpr.ct_open.Rph.flag,indicate_f);
        SET1      N:_tpr+424.2       ;; 2 cycles
        BR        S:??magnet_log_logic_336  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 2161   }
// 2162   else
// 2163   {
// 2164     bitClear(tpr.ct_open.Rph.flag,event_f);
??magnet_log_logic_335:
        CLR1      N:_tpr+424.0       ;; 2 cycles
// 2165     bitClear(tpr.ct_open.Rph.flag,indicate_f);
        CLR1      N:_tpr+424.2       ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
// 2166   }
// 2167   
// 2168   if(tamper_event_status & BIT_CT_OPEN_Y)
??magnet_log_logic_336:
        MOVW      HL, #LWRD(_tamper_event_status+1)  ;; 1 cycle
        MOV1      CY, [HL].6         ;; 1 cycle
        BNC       ??magnet_log_logic_337  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 2169   {
// 2170     bitSet(tpr.ct_open.Yph.flag,event_f);
        SET1      N:_tpr+452.0       ;; 2 cycles
// 2171     bitSet(tpr.ct_open.Yph.flag,indicate_f);
        SET1      N:_tpr+452.2       ;; 2 cycles
        BR        S:??magnet_log_logic_338  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 2172   }
// 2173   else
// 2174   {
// 2175     bitClear(tpr.ct_open.Yph.flag,event_f);
??magnet_log_logic_337:
        CLR1      N:_tpr+452.0       ;; 2 cycles
// 2176     bitClear(tpr.ct_open.Yph.flag,indicate_f);
        CLR1      N:_tpr+452.2       ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
// 2177   }
// 2178   
// 2179   if(tamper_event_status & BIT_CT_OPEN_B)
??magnet_log_logic_338:
        MOVW      HL, #LWRD(_tamper_event_status+1)  ;; 1 cycle
        MOV1      CY, [HL].7         ;; 1 cycle
        BNC       ??magnet_log_logic_339  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 2180   {
// 2181     bitSet(tpr.ct_open.Bph.flag,event_f);
        SET1      N:_tpr+480.0       ;; 2 cycles
// 2182     bitSet(tpr.ct_open.Bph.flag,indicate_f);
        SET1      N:_tpr+480.2       ;; 2 cycles
        BR        S:??magnet_log_logic_340  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 2183   }
// 2184   else
// 2185   {
// 2186     bitClear(tpr.ct_open.Bph.flag,event_f);
??magnet_log_logic_339:
        CLR1      N:_tpr+480.0       ;; 2 cycles
// 2187     bitClear(tpr.ct_open.Bph.flag,indicate_f);
        CLR1      N:_tpr+480.2       ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
// 2188   }
// 2189   
// 2190   /* Over current */
// 2191   if(tamper_event_status & BIT_CURR_OVER_R)
??magnet_log_logic_340:
        MOVW      HL, #LWRD(_tamper_event_status+2)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??magnet_log_logic_341  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 2192   {
// 2193     bitSet(tpr.curr_over.Rph.flag,event_f);
        SET1      N:_tpr+508.0       ;; 2 cycles
// 2194     bitSet(tpr.curr_over.Rph.flag,indicate_f);
        SET1      N:_tpr+508.2       ;; 2 cycles
        BR        S:??magnet_log_logic_342  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 2195   }
// 2196   else
// 2197   {
// 2198     bitClear(tpr.curr_over.Rph.flag,event_f);
??magnet_log_logic_341:
        CLR1      N:_tpr+508.0       ;; 2 cycles
// 2199     bitClear(tpr.curr_over.Rph.flag,indicate_f);
        CLR1      N:_tpr+508.2       ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
// 2200   }
// 2201   
// 2202   if(tamper_event_status & BIT_CURR_OVER_Y)
??magnet_log_logic_342:
        MOVW      HL, #LWRD(_tamper_event_status+2)  ;; 1 cycle
        MOV1      CY, [HL].1         ;; 1 cycle
        BNC       ??magnet_log_logic_343  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 2203   {
// 2204     bitSet(tpr.curr_over.Yph.flag,event_f);
        SET1      N:_tpr+536.0       ;; 2 cycles
// 2205     bitSet(tpr.curr_over.Yph.flag,indicate_f);
        SET1      N:_tpr+536.2       ;; 2 cycles
        BR        S:??magnet_log_logic_344  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 2206   }
// 2207   else
// 2208   {
// 2209     bitClear(tpr.curr_over.Yph.flag,event_f);
??magnet_log_logic_343:
        CLR1      N:_tpr+536.0       ;; 2 cycles
// 2210     bitClear(tpr.curr_over.Yph.flag,indicate_f);
        CLR1      N:_tpr+536.2       ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
// 2211   }
// 2212   
// 2213   if(tamper_event_status & BIT_CURR_OVER_B)
??magnet_log_logic_344:
        MOVW      HL, #LWRD(_tamper_event_status+2)  ;; 1 cycle
        MOV1      CY, [HL].2         ;; 1 cycle
        BNC       ??magnet_log_logic_345  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 2214   {
// 2215     bitSet(tpr.curr_over.Bph.flag,event_f);
        SET1      N:_tpr+564.0       ;; 2 cycles
// 2216     bitSet(tpr.curr_over.Bph.flag,indicate_f);
        SET1      N:_tpr+564.2       ;; 2 cycles
        BR        S:??magnet_log_logic_346  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 2217   }
// 2218   else
// 2219   {
// 2220     bitClear(tpr.curr_over.Bph.flag,event_f);
??magnet_log_logic_345:
        CLR1      N:_tpr+564.0       ;; 2 cycles
// 2221     bitClear(tpr.curr_over.Bph.flag,indicate_f);
        CLR1      N:_tpr+564.2       ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
// 2222   }
// 2223   
// 2224   /* Neutral Disturb */
// 2225   if(tamper_event_status & BIT_NEU_DISTURB)
??magnet_log_logic_346:
        MOVW      HL, #LWRD(_tamper_event_status+2)  ;; 1 cycle
        MOV1      CY, [HL].3         ;; 1 cycle
        BNC       ??magnet_log_logic_347  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 2226   {
// 2227     bitSet(tpr.neu_disturb.flag,event_f);
        SET1      N:_tpr+200.0       ;; 2 cycles
// 2228     bitSet(tpr.neu_disturb.flag,indicate_f);
        SET1      N:_tpr+200.2       ;; 2 cycles
        BR        S:??magnet_log_logic_348  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 2229   }
// 2230   else
// 2231   {
// 2232     bitClear(tpr.neu_disturb.flag,event_f);
??magnet_log_logic_347:
        CLR1      N:_tpr+200.0       ;; 2 cycles
// 2233     bitClear(tpr.neu_disturb.flag,indicate_f);
        CLR1      N:_tpr+200.2       ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
// 2234   }
// 2235   
// 2236   /* Top cover */
// 2237   if(tamper_event_status & BIT_TOP_COVER)
??magnet_log_logic_348:
        MOVW      HL, #LWRD(_tamper_event_status+2)  ;; 1 cycle
        MOV1      CY, [HL].4         ;; 1 cycle
        BNC       ??magnet_log_logic_349  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 2238   {
// 2239     bitSet(tpr.top_cover.flag,event_f);
        SET1      N:_tpr+592.0       ;; 2 cycles
// 2240     bitSet(tpr.top_cover.flag,indicate_f);
        SET1      N:_tpr+592.2       ;; 2 cycles
        BR        S:??magnet_log_logic_350  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 2241   }
// 2242   else
// 2243   {
// 2244     bitClear(tpr.top_cover.flag,event_f);
??magnet_log_logic_349:
        CLR1      N:_tpr+592.0       ;; 2 cycles
// 2245     bitClear(tpr.top_cover.flag,indicate_f);
        CLR1      N:_tpr+592.2       ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
// 2246   }
// 2247   
// 2248   /* Battery Low */
// 2249   if(tamper_event_status & BIT_BATTERY_LOW)
??magnet_log_logic_350:
        MOVW      HL, #LWRD(_tamper_event_status+2)  ;; 1 cycle
        MOV1      CY, [HL].5         ;; 1 cycle
        BNC       ??magnet_log_logic_351  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 2250   {
// 2251     bitSet(tpr.battery_low.flag,event_f);
        SET1      N:_tpr+616.0       ;; 2 cycles
// 2252     bitSet(tpr.battery_low.flag,indicate_f);
        SET1      N:_tpr+616.2       ;; 2 cycles
        BR        S:??magnet_log_logic_352  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 2253   }
// 2254   else
// 2255   {
// 2256     bitClear(tpr.battery_low.flag,event_f);
??magnet_log_logic_351:
        CLR1      N:_tpr+616.0       ;; 2 cycles
// 2257     bitClear(tpr.battery_low.flag,indicate_f);
        CLR1      N:_tpr+616.2       ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
// 2258   }
// 2259    
// 2260   
// 2261   if(eprom_read(0x0CA0,0,PAGE_1,AUTO_CALC) == EEP_OK)
??magnet_log_logic_352:
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xCA0         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
        CMP       A, #0x1            ;; 1 cycle
        BNZ       ??magnet_log_logic_353  ;; 4 cycles
        ; ------------------------------------- Block: 12 cycles
// 2262   {
// 2263     tpr.power_last = opr_data[2];
        MOV       A, N:_opr_data+2   ;; 1 cycle
        MOV       N:_tpr+11, A       ;; 1 cycle
// 2264     tpr.transaction_last = opr_data[3];
        MOV       A, N:_opr_data+3   ;; 1 cycle
        MOV       N:_tpr+15, A       ;; 1 cycle
// 2265     tpr.non_roll_last = opr_data[5];
        MOV       A, N:_opr_data+5   ;; 1 cycle
        MOV       N:_tpr+23, A       ;; 1 cycle
// 2266     tpr.vol_related_last = opr_data[0];
        MOV       A, N:_opr_data     ;; 1 cycle
        MOV       N:_tpr+3, A        ;; 1 cycle
// 2267     tpr.current_related_last = opr_data[1];
        MOV       A, N:_opr_data+1   ;; 1 cycle
        MOV       N:_tpr+7, A        ;; 1 cycle
// 2268     tpr.others_last = opr_data[4];
        MOV       A, N:_opr_data+4   ;; 1 cycle
        MOV       N:_tpr+19, A       ;; 1 cycle
// 2269     tpr.debug_last = char_array_to_int(&opr_data[6]);
        MOVW      AX, #LWRD(_opr_data+6)  ;; 1 cycle
          CFI FunCall _char_array_to_int
        CALL      _char_array_to_int  ;; 3 cycles
        MOVW      N:_tpr+28, AX      ;; 1 cycle
        ; ------------------------------------- Block: 17 cycles
// 2270   }
// 2271   
// 2272 }
??magnet_log_logic_353:
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock11
        ; ------------------------------------- Block: 6 cycles
        ; ------------------------------------- Total: 438 cycles
// 2273 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock12 Using cfiCommon1
          CFI Function _battery_function
        CODE
// 2274 void battery_function()
// 2275 {       
_battery_function:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
// 2276     if(bitIsSet(tamper_sel,TPR_BATTERY_LOW))
        MOVW      HL, #LWRD(_tamper_sel+1)  ;; 1 cycle
        MOV1      CY, [HL].5         ;; 1 cycle
        SKC                          ;; 4 cycles
        BR        N:??magnet_log_logic_354  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 2277     {
// 2278         temp_us8 = 0;
        MOV       S:_temp_us8, #0x0  ;; 1 cycle
// 2279         if(battery_installed == TEKCELL)
        MOVW      HL, #LWRD(_flag_battery)  ;; 1 cycle
        MOV1      CY, [HL].2         ;; 1 cycle
        BC        ??magnet_log_logic_355  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
// 2280         {
// 2281             if(battery_voltage < THR_TEKCELL_LOW_OCC && bitIsClear(tpr.battery_low.flag,event_f))   /* Low battery detected */
        MOVW      AX, N:_battery_voltage  ;; 1 cycle
        CMPW      AX, #0x122         ;; 1 cycle
        BNC       ??magnet_log_logic_356  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      HL, #LWRD(_tpr+616)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BC        ??magnet_log_logic_356  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 2282             {
// 2283                 temp_us8 = 1;   
        MOV       S:_temp_us8, #0x1  ;; 1 cycle
        BR        S:??magnet_log_logic_357  ;; 3 cycles
        ; ------------------------------------- Block: 4 cycles
// 2284             }
// 2285             else if(battery_voltage >= THR_TEKCELL_LOW_RES && bitIsSet(tpr.battery_low.flag,event_f)) 
??magnet_log_logic_356:
        MOVW      AX, N:_battery_voltage  ;; 1 cycle
        CMPW      AX, #0x14A         ;; 1 cycle
        BC        ??magnet_log_logic_357  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      HL, #LWRD(_tpr+616)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??magnet_log_logic_357  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 2286             {
// 2287                 temp_us8 = 2;
        MOV       S:_temp_us8, #0x2  ;; 1 cycle
        BR        S:??magnet_log_logic_357  ;; 3 cycles
        ; ------------------------------------- Block: 4 cycles
// 2288             }
// 2289         }
// 2290         else
// 2291         {
// 2292             if(battery_voltage < THR_COINCELL_LOW_OCC && bitIsClear(tpr.battery_low.flag,event_f))   /* Low battery detected */
??magnet_log_logic_355:
        MOVW      AX, N:_battery_voltage  ;; 1 cycle
        CMPW      AX, #0x104         ;; 1 cycle
        BNC       ??magnet_log_logic_358  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      HL, #LWRD(_tpr+616)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BC        ??magnet_log_logic_358  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 2293             {
// 2294                 temp_us8 = 1;   
        MOV       S:_temp_us8, #0x1  ;; 1 cycle
        BR        S:??magnet_log_logic_357  ;; 3 cycles
        ; ------------------------------------- Block: 4 cycles
// 2295             }
// 2296             else if(battery_voltage >= THR_COINCELL_LOW_RES && bitIsSet(tpr.battery_low.flag,event_f)) 
??magnet_log_logic_358:
        MOVW      AX, N:_battery_voltage  ;; 1 cycle
        CMPW      AX, #0x122         ;; 1 cycle
        BC        ??magnet_log_logic_357  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      HL, #LWRD(_tpr+616)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        SKNC                         ;; 1 cycle
        ; ------------------------------------- Block: 3 cycles
// 2297             {
// 2298                 temp_us8 = 2;
        MOV       S:_temp_us8, #0x2  ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
// 2299             }
// 2300         }
// 2301         if(temp_us8 == 1)   /* Low battery detected */
??magnet_log_logic_357:
        CMP       S:_temp_us8, #0x1  ;; 1 cycle
        BNZ       ??magnet_log_logic_359  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 2302         {
// 2303             bitSet(tamper_instant_status,BIT_BATTERY_LOW);
        SET1      N:_tamper_instant_status+2.5  ;; 2 cycles
// 2304             bitSet(tpr.battery_low.flag,indicate_f);
        SET1      N:_tpr+616.2       ;; 2 cycles
// 2305             bitSet(tpr.battery_low.flag,event_f);
        SET1      N:_tpr+616.0       ;; 2 cycles
// 2306             bitSet(tpr.battery_low.flag,action_f);       
        SET1      N:_tpr+616.1       ;; 2 cycles
        BR        S:??magnet_log_logic_360  ;; 3 cycles
        ; ------------------------------------- Block: 11 cycles
// 2307         }
// 2308         else if(temp_us8 == 2)       /* Low battery restoration */
??magnet_log_logic_359:
        CMP       S:_temp_us8, #0x2  ;; 1 cycle
        BNZ       ??magnet_log_logic_360  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 2309         {
// 2310             bitClear(tamper_instant_status,BIT_BATTERY_LOW);
        CLR1      N:_tamper_instant_status+2.5  ;; 2 cycles
// 2311             bitClear(tpr.battery_low.flag,indicate_f);
        CLR1      N:_tpr+616.2       ;; 2 cycles
// 2312             bitClear(tpr.battery_low.flag,event_f);
        CLR1      N:_tpr+616.0       ;; 2 cycles
// 2313             bitSet(tpr.battery_low.flag,action_f);
        SET1      N:_tpr+616.1       ;; 2 cycles
        ; ------------------------------------- Block: 8 cycles
// 2314         }
// 2315         if(bitIsSet(tpr.battery_low.flag,action_f))
??magnet_log_logic_360:
        MOVW      HL, #LWRD(_tpr+616)  ;; 1 cycle
        MOV1      CY, [HL].1         ;; 1 cycle
        BNC       ??magnet_log_logic_361  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 2316         {
// 2317             bitClear(tpr.battery_low.flag,action_f);
        CLR1      N:_tpr+616.1       ;; 2 cycles
// 2318             if(bitIsSet(tpr.battery_low.flag,event_f))
        MOVW      HL, #LWRD(_tpr+616)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??magnet_log_logic_362  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
// 2319             {
// 2320                 mem_log1(COMPART_DEBUG,EVENT_ID_BATTERY_LOW_OCC);
        MOVW      BC, #0x161         ;; 1 cycle
        MOV       A, #0x3            ;; 1 cycle
          CFI FunCall _mem_log1
        CALL      _mem_log1          ;; 3 cycles
        BR        S:??magnet_log_logic_363  ;; 3 cycles
        ; ------------------------------------- Block: 8 cycles
// 2321             }
// 2322             else
// 2323             {
// 2324                 mem_log1(COMPART_DEBUG,EVENT_ID_BATTERY_LOW_RES);
??magnet_log_logic_362:
        MOVW      BC, #0x162         ;; 1 cycle
        MOV       A, #0x3            ;; 1 cycle
          CFI FunCall _mem_log1
        CALL      _mem_log1          ;; 3 cycles
          CFI FunCall _update_tamper_variables
        ; ------------------------------------- Block: 5 cycles
// 2325             }
// 2326             update_tamper_variables();
??magnet_log_logic_363:
        CALL      _update_tamper_variables  ;; 3 cycles
        RET                          ;; 6 cycles
        ; ------------------------------------- Block: 9 cycles
// 2327         }
// 2328     }
// 2329     else
// 2330     {
// 2331         tpr.battery_low.flag = 0;
??magnet_log_logic_354:
        MOV       N:_tpr+616, #0x0   ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
// 2332     }
// 2333 }
??magnet_log_logic_361:
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock12
        ; ------------------------------------- Block: 6 cycles
        ; ------------------------------------- Total: 143 cycles

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock13 Using cfiCommon1
          CFI Function _magnet_log_logic
        CODE
// 2334 void magnet_log_logic()
// 2335 {
_magnet_log_logic:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
// 2336     if(bitIsSet(tpr.magnet.flag,action_f))
        MOVW      HL, #LWRD(_tpr+32)  ;; 1 cycle
        MOV1      CY, [HL].1         ;; 1 cycle
        SKC                          ;; 4 cycles
        BR        N:??magnet_log_logic_364  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 2337     {
// 2338         if(flag_mag_update_metro_par == 1)
        MOVW      HL, #LWRD(_flag_mag)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??magnet_log_logic_364  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 2339         {
// 2340             if(flag_mag_r_updated == 1 && flag_mag_y_updated == 1 && flag_mag_b_updated == 1 && flag_mag_all_updated == 1)
        MOV       A, N:_flag_mag     ;; 1 cycle
        AND       A, #0x1E           ;; 1 cycle
        CMP       A, #0x1E           ;; 1 cycle
        BNZ       ??magnet_log_logic_364  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
// 2341             {
// 2342                 flag_mag_update_metro_par = 0;
        CLR1      N:_flag_mag.0      ;; 2 cycles
// 2343                 bitClear(tpr.magnet.flag,action_f);
        CLR1      N:_tpr+32.1        ;; 2 cycles
// 2344                 if(bitIsSet(tpr.magnet.flag,event_f))
        MOVW      HL, #LWRD(_tpr+32)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??magnet_log_logic_365  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
// 2345                 {
// 2346                     mem_log(COMPART_OTHERS,EVENT_ID_MAGNET_OCC,tpr.magnet);
        MOVW      HL, #LWRD(_tpr+32)  ;; 1 cycle
        SUBW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+32
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x1C          ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      BC, #0xC9          ;; 1 cycle
        MOV       A, #0x2            ;; 1 cycle
          CFI FunCall _mem_log
        CALL      _mem_log           ;; 3 cycles
        ADDW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+4
        BR        S:??magnet_log_logic_366  ;; 3 cycles
        ; ------------------------------------- Block: 16 cycles
// 2347                 }
// 2348                 else
// 2349                 {
// 2350                     mem_log(COMPART_OTHERS,EVENT_ID_MAGNET_RES,tpr.magnet);
??magnet_log_logic_365:
        MOVW      HL, #LWRD(_tpr+32)  ;; 1 cycle
        SUBW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+32
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x1C          ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      BC, #0xCA          ;; 1 cycle
        MOV       A, #0x2            ;; 1 cycle
          CFI FunCall _mem_log
        CALL      _mem_log           ;; 3 cycles
        ADDW      SP, #0x1C          ;; 1 cycle
          CFI CFA SP+4
        ; ------------------------------------- Block: 13 cycles
// 2351                 }
// 2352                 eprom_read(0x0C90,0,PAGE_1,AUTO_CALC);    
??magnet_log_logic_366:
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xC90         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
// 2353                 time_into_char_array5_sec(tpr.magnet.time,&opr_data[5]);
        MOVW      HL, #LWRD(_tpr+34)  ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+12
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      AX, #LWRD(_opr_data+5)  ;; 1 cycle
          CFI FunCall _time_into_char_array5_sec
        CALL      _time_into_char_array5_sec  ;; 3 cycles
// 2354                 eprom_write(0x0C90,0,16,PAGE_1,AUTO_CALC);    
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+14
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xC90         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
// 2355                 update_tamper_variables();
          CFI FunCall _update_tamper_variables
        CALL      _update_tamper_variables  ;; 3 cycles
        ADDW      SP, #0xA           ;; 1 cycle
          CFI CFA SP+4
        ; ------------------------------------- Block: 31 cycles
// 2356             }
// 2357         }
// 2358     }
// 2359 }
??magnet_log_logic_364:
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock13
        ; ------------------------------------- Block: 6 cycles
        ; ------------------------------------- Total: 95 cycles

        SECTION `.iar_vfe_header`:DATA:NOALLOC:NOROOT(1)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
        DC32 0

        END
// 
//    809 bytes in section .bss
//      1 byte  in section .sbss.noinit  (abs)
// 11'512 bytes in section .text
// 
// 11'512 bytes of CODE memory
//    809 bytes of DATA memory (+ 1 byte shared)
//
//Errors: none
//Warnings: none
