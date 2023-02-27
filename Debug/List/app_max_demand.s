///////////////////////////////////////////////////////////////////////////////
//
// IAR C/C++ Compiler V4.20.1.2260 for RL78               20/Dec/2020  22:38:01
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
//        BootCode\source_code\source_files\app_max_demand.c
//    Command line       =
//        -f C:\Users\laptop\AppData\Local\Temp\EWDF5.tmp ("D:\Dheeraj\New
//        folder\0. GDEV72 -
//        BootCode\source_code\source_files\app_max_demand.c" --core s3
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
//        BootCode\Debug\List\app_max_demand.s
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

        EXTERN _Now
        EXTERN _OffTime
        EXTERN _tariff_no
        EXTERN _zone_change_f
        EXTERN _off_min
        EXTERN _temp_us8
        EXTERN _eoi_complet_f
        EXTERN _opr_data
        EXTERN ?L_MUL_FAST_L03
        EXTERN ?MOVE_LONG_L06
        EXTERN ?UC_DIV_L01
        EXTERN ?UC_MOD_L01
        EXTERN ?UL_CMP_L03
        EXTERN _Eprom_WriteWM
        EXTERN _TempTime
        EXTERN _bcd_to_decimal
        EXTERN _char_array_to_long3
        EXTERN _char_array_to_long4
        EXTERN _datediff
        EXTERN _decimal_to_bcd
        EXTERN _demand
        EXTERN _energy
        EXTERN _eprom_read
        EXTERN _eprom_write
        EXTERN _fill_oprzero
        EXTERN _last_energy_mag
        EXTERN _last_energy_mag_Bph
        EXTERN _last_energy_mag_Rph
        EXTERN _last_energy_mag_Yph
        EXTERN _long_into_char_array3
        EXTERN _long_into_char_array4
        EXTERN _temp_us32
        EXTERN _time_inc_hour
        EXTERN _time_into_char_array4

        PUBLIC _cal_md
        PUBLIC _cal_md_tod
        PUBLIC _cal_present
        PUBLIC _cal_present_demand
        PUBLIC _cal_rising
        PUBLIC _cal_rising_demand
        PUBLIC _md_next_interval_timestamp
        PUBLIC _md_ram_init
        PUBLIC _md_reset_ip_flag
        PUBLIC _md_type
        PUBLIC _mdi_interval_time
        PUBLIC _mdi_no_of_period
        PUBLIC _mdi_period
        PUBLIC _mdi_sel
        PUBLIC _mdi_sel_new
        PUBLIC _mdi_sel_slide
        PUBLIC _miss_md_date_f
        PUBLIC _reset_md
        PUBLIC _reset_md_fg
        PUBLIC _reset_mdreset_ip_flag
        
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
          CFI EndCommon cfiCommon2
        
// D:\Dheeraj\New folder\0. GDEV72 - BootCode\source_code\source_files\app_max_demand.c
//    1 /***********************************************************************************************************************
//    2 * File Name       : app_max_demand.c
//    3 * Current Version : rev_01  
//    4 * Tool-Chain      : IAR Systems RL78
//    5 * Description     : this file will have all the routines to implment demand calculations and max demand calculations.
//    6 * Creation Date   : 5/31/2020
//    7 * Company         : Genus Power Infrastructures Limited, Jaipur
//    8 * Author          : dheeraj.singhal
//    9 * Version History : rev_01 :
//   10 ***********************************************************************************************************************/
//   11 /************************************ Includes **************************************/
//   12 #include "app_max_demand.h"
//   13 /************************************ Local Variables *****************************************/
//   14 /************************************ Extern Variables *****************************************/

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   15 us8 mdi_period,mdi_no_of_period,mdi_interval_time;
_mdi_period:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_mdi_no_of_period:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_mdi_interval_time:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   16 us8 md_type,mdi_sel,mdi_sel_slide;
_md_type:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_mdi_sel:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_mdi_sel_slide:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   17 us8 miss_md_date_f,md_reset_ip_flag;
_miss_md_date_f:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_md_reset_ip_flag:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   18 us8 mdi_sel_new;
_mdi_sel_new:
        DS 1
//   19 /************************************ Local Functions *******************************/
//   20 us32 cal_rising(us32 energy, us32 last_energy, us8 phasewise, us32 mag_energy, us32 last_mag_energy, us8 sub_mag_demand);
//   21 us32 cal_present(us32 present_energy, us32 last_energy, us8 phasewise, us32 mag_energy, us32 last_mag_energy, us8 sub_mag_demand);
//   22 void cal_md_tod(us32 present_demand, us32 *max_demand_tod, us8 opr_loc, us16 address);
//   23 void cal_md(us32 present_demand, us32 *max_demand, us8 opr_loc, us16 address);
//   24 void reset_mdreset_ip_flag();
//   25 rtc_counter_value_t md_next_interval_timestamp(us8 mdi_period1, rtc_counter_value_t Time);
//   26 /************************************ Extern Functions ******************************/
//   27 void cal_rising_demand();
//   28 void cal_present_demand();
//   29 void md_ram_init();
//   30 void reset_md(void);
//   31 void reset_md_fg(void);

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock0 Using cfiCommon0
          CFI Function _cal_present_demand
        CODE
//   32 void cal_present_demand()
//   33 {
_cal_present_demand:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 12
        SUBW      SP, #0xC           ;; 1 cycle
          CFI CFA SP+16
//   34   unsigned char on_time;
//   35   unsigned int local_address=0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [SP+0x02], AX      ;; 1 cycle
//   36   
//   37   us8 temp_min;
//   38   
//   39   /****************Reading Last Energy *******************/
//   40   
//   41   if(md_type == 1)                                                      /* Sliding window*/
        CMP       N:_md_type, #0x1   ;; 1 cycle
        BNZ       ??reset_md_0       ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
//   42   {
//   43     temp_min = bcd_to_decimal(Now.min);
        MOV       A, N:_Now+1        ;; 1 cycle
          CFI FunCall _bcd_to_decimal
        CALL      _bcd_to_decimal    ;; 3 cycles
        MOV       [SP], A            ;; 1 cycle
//   44     if(miss_md_date_f==1)                                               /* missing period case */
        CMP       N:_miss_md_date_f, #0x1  ;; 1 cycle
        BNZ       ??reset_md_1       ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
//   45     {
//   46       temp_min = bcd_to_decimal(OffTime.min);                                          
        MOV       A, N:_OffTime+1    ;; 1 cycle
          CFI FunCall _bcd_to_decimal
        CALL      _bcd_to_decimal    ;; 3 cycles
        MOV       [SP], A            ;; 1 cycle
        ; ------------------------------------- Block: 5 cycles
//   47       while((temp_min % mdi_period) != 0)                           /* finding start of next sub interval period after power off*/
??cal_present_demand_0:
        MOV       X, N:_mdi_period   ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
          CFI FunCall ?UC_MOD_L01
        CALL      N:?UC_MOD_L01      ;; 3 cycles
        MOV       B, A               ;; 1 cycle
        CMP0      B                  ;; 1 cycle
        BZ        ??reset_md_1       ;; 4 cycles
        ; ------------------------------------- Block: 11 cycles
//   48       {
//   49         temp_min++;
        MOV       A, [SP]            ;; 1 cycle
        INC       A                  ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//   50         if((temp_min % mdi_period) == 0)
        MOV       X, N:_mdi_period   ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
          CFI FunCall ?UC_MOD_L01
        CALL      N:?UC_MOD_L01      ;; 3 cycles
        MOV       B, A               ;; 1 cycle
        CMP0      B                  ;; 1 cycle
        BNZ       ??cal_present_demand_0  ;; 4 cycles
        ; ------------------------------------- Block: 14 cycles
//   51         {
//   52           break;
//   53         }
//   54       }
//   55     }
//   56     local_address=0x0800+((temp_min % mdi_interval_time) / mdi_period)*16; 
??reset_md_1:
        MOV       X, N:_mdi_interval_time  ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
          CFI FunCall ?UC_MOD_L01
        CALL      N:?UC_MOD_L01      ;; 3 cycles
        MOV       B, A               ;; 1 cycle
        MOV       A, B               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        MOV       X, N:_mdi_period   ;; 1 cycle
        MOV       A, B               ;; 1 cycle
          CFI FunCall ?UC_DIV_L01
        CALL      N:?UC_DIV_L01      ;; 3 cycles
        MOV       C, A               ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOVW      AX, #0x10          ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #0x800         ;; 1 cycle
        MOVW      [SP+0x02], AX      ;; 1 cycle
//   57     if((temp_min%mdi_interval_time)==0) 
        MOV       X, N:_mdi_interval_time  ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
          CFI FunCall ?UC_MOD_L01
        CALL      N:?UC_MOD_L01      ;; 3 cycles
        MOV       B, A               ;; 1 cycle
        CMP0      B                  ;; 1 cycle
        BNZ       ??reset_md_2       ;; 4 cycles
          CFI FunCall _reset_mdreset_ip_flag
        ; ------------------------------------- Block: 31 cycles
//   58     {
//   59       reset_mdreset_ip_flag();
        CALL      _reset_mdreset_ip_flag  ;; 3 cycles
        BR        S:??reset_md_2     ;; 3 cycles
          CFI FunCall _reset_mdreset_ip_flag
        ; ------------------------------------- Block: 6 cycles
//   60     }
//   61   }
//   62   else                  /* Fixed interval */
//   63   {
//   64     reset_mdreset_ip_flag(); //check this function
??reset_md_0:
        CALL      _reset_mdreset_ip_flag  ;; 3 cycles
//   65     local_address=0x0980;
        MOVW      AX, #0x980         ;; 1 cycle
        MOVW      [SP+0x02], AX      ;; 1 cycle
        ; ------------------------------------- Block: 5 cycles
//   66   }
//   67   if(eprom_read(local_address,0,PAGE_1,AUTO_CALC) == EEP_OK)
??reset_md_2:
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, [SP+0x02]      ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
        CMP       A, #0x1            ;; 1 cycle
        BNZ       ??reset_md_3       ;; 4 cycles
        ; ------------------------------------- Block: 12 cycles
//   68   {
//   69     demand.Allph.act_imp.present.last_energy_value = char_array_to_long4(&opr_data[0]);
        MOVW      AX, #LWRD(_opr_data)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_demand+128, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+130, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//   70     demand.Allph.act_exp.present.last_energy_value = char_array_to_long4(&opr_data[4]);
        MOVW      AX, #LWRD(_opr_data+4)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_demand+152, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+154, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//   71     demand.Allph.app_imp.present.last_energy_value = char_array_to_long4(&opr_data[8]);
        MOVW      AX, #LWRD(_opr_data+8)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_demand+176, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+178, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        BR        S:??reset_md_4     ;; 3 cycles
        ; ------------------------------------- Block: 27 cycles
//   72   }
//   73   else
//   74   {
//   75     demand.Allph.act_imp.present.last_energy_value = energy.Allph.active_imp;
??reset_md_3:
        MOVW      BC, N:_energy+54   ;; 1 cycle
        MOVW      AX, N:_energy+52   ;; 1 cycle
        MOVW      N:_demand+128, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+130, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//   76     demand.Allph.act_exp.present.last_energy_value = energy.Allph.active_exp;
        MOVW      BC, N:_energy+58   ;; 1 cycle
        MOVW      AX, N:_energy+56   ;; 1 cycle
        MOVW      N:_demand+152, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+154, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//   77     demand.Allph.app_imp.present.last_energy_value = energy.Allph.apparent_imp;
        MOVW      BC, N:_energy+70   ;; 1 cycle
        MOVW      AX, N:_energy+68   ;; 1 cycle
        MOVW      N:_demand+176, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+178, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ; ------------------------------------- Block: 18 cycles
//   78   }
//   79   if(md_type==1)                /* Sliding window */
??reset_md_4:
        CMP       N:_md_type, #0x1   ;; 1 cycle
        BNZ       ??reset_md_5       ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//   80   {
//   81     local_address+=0x00c0;		
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        ADDW      AX, #0xC0          ;; 1 cycle
        MOVW      [SP+0x02], AX      ;; 1 cycle
        BR        S:??reset_md_6     ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
//   82   }
//   83   else                          /* Fixed Interval */
//   84   {
//   85     local_address+=0x0010;		
??reset_md_5:
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
        MOVW      [SP+0x02], AX      ;; 1 cycle
        ; ------------------------------------- Block: 3 cycles
//   86   }
//   87   if(eprom_read(local_address,0,PAGE_1,AUTO_CALC) == EEP_OK)
??reset_md_6:
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, [SP+0x02]      ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
        CMP       A, #0x1            ;; 1 cycle
        BNZ       ??reset_md_7       ;; 4 cycles
        ; ------------------------------------- Block: 12 cycles
//   88   {
//   89     demand.Allph.app_exp.present.last_energy_value = char_array_to_long4(&opr_data[0]);
        MOVW      AX, #LWRD(_opr_data)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_demand+200, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+202, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        BR        S:??reset_md_8     ;; 3 cycles
        ; ------------------------------------- Block: 11 cycles
//   90   }
//   91   else
//   92   {
//   93     demand.Allph.app_exp.present.last_energy_value = energy.Allph.apparent_exp;
??reset_md_7:
        MOVW      BC, N:_energy+74   ;; 1 cycle
        MOVW      AX, N:_energy+72   ;; 1 cycle
        MOVW      N:_demand+200, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+202, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
//   94   }   
//   95   local_address=0x0360;
??reset_md_8:
        MOVW      AX, #0x360         ;; 1 cycle
        MOVW      [SP+0x02], AX      ;; 1 cycle
//   96   if(eprom_read(local_address,0,PAGE_1,AUTO_CALC) == EEP_OK)
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, [SP+0x02]      ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
        CMP       A, #0x1            ;; 1 cycle
        BNZ       ??reset_md_9       ;; 4 cycles
        ; ------------------------------------- Block: 14 cycles
//   97   {
//   98     demand.Rph.act_imp.present.last_energy_value = char_array_to_long4(&opr_data[0]);
        MOVW      AX, #LWRD(_opr_data)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_demand+8, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+10, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//   99     demand.Yph.act_imp.present.last_energy_value = char_array_to_long4(&opr_data[4]);
        MOVW      AX, #LWRD(_opr_data+4)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_demand+48, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+50, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  100     demand.Bph.act_imp.present.last_energy_value = char_array_to_long4(&opr_data[8]);
        MOVW      AX, #LWRD(_opr_data+8)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_demand+88, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+90, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        BR        S:??reset_md_10    ;; 3 cycles
        ; ------------------------------------- Block: 27 cycles
//  101   }
//  102   else
//  103   {
//  104     demand.Rph.act_imp.present.last_energy_value = energy.Rph.active_imp;
??reset_md_9:
        MOVW      BC, N:_energy+4    ;; 1 cycle
        MOVW      AX, N:_energy+2    ;; 1 cycle
        MOVW      N:_demand+8, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+10, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  105     demand.Yph.act_imp.present.last_energy_value = energy.Yph.active_imp;
        MOVW      BC, N:_energy+18   ;; 1 cycle
        MOVW      AX, N:_energy+16   ;; 1 cycle
        MOVW      N:_demand+48, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+50, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  106     demand.Bph.act_imp.present.last_energy_value = energy.Bph.active_imp;
        MOVW      BC, N:_energy+32   ;; 1 cycle
        MOVW      AX, N:_energy+30   ;; 1 cycle
        MOVW      N:_demand+88, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+90, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ; ------------------------------------- Block: 18 cycles
//  107   }  
//  108   local_address += 0x10;
??reset_md_10:
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
        MOVW      [SP+0x02], AX      ;; 1 cycle
//  109   if(eprom_read(local_address,0,PAGE_1,AUTO_CALC) == EEP_OK)
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, [SP+0x02]      ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
        CMP       A, #0x1            ;; 1 cycle
        BNZ       ??reset_md_11      ;; 4 cycles
        ; ------------------------------------- Block: 15 cycles
//  110   {
//  111     demand.Rph.act_exp.present.last_energy_value = char_array_to_long4(&opr_data[0]);
        MOVW      AX, #LWRD(_opr_data)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_demand+28, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+30, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  112     demand.Yph.act_exp.present.last_energy_value = char_array_to_long4(&opr_data[4]);
        MOVW      AX, #LWRD(_opr_data+4)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_demand+68, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+70, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  113     demand.Bph.act_exp.present.last_energy_value = char_array_to_long4(&opr_data[8]);
        MOVW      AX, #LWRD(_opr_data+8)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_demand+108, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+110, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        BR        S:??reset_md_12    ;; 3 cycles
        ; ------------------------------------- Block: 27 cycles
//  114   }
//  115   else
//  116   {
//  117     demand.Rph.act_exp.present.last_energy_value = energy.Rph.active_exp;
??reset_md_11:
        MOVW      BC, N:_energy+8    ;; 1 cycle
        MOVW      AX, N:_energy+6    ;; 1 cycle
        MOVW      N:_demand+28, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+30, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  118     demand.Yph.act_exp.present.last_energy_value = energy.Yph.active_exp;
        MOVW      BC, N:_energy+22   ;; 1 cycle
        MOVW      AX, N:_energy+20   ;; 1 cycle
        MOVW      N:_demand+68, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+70, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  119     demand.Bph.act_exp.present.last_energy_value = energy.Bph.active_exp;
        MOVW      BC, N:_energy+36   ;; 1 cycle
        MOVW      AX, N:_energy+34   ;; 1 cycle
        MOVW      N:_demand+108, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+110, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ; ------------------------------------- Block: 18 cycles
//  120   }  
//  121   /* Defraud energy */
//  122   if(eprom_read(0x09C0,0,PAGE_1,AUTO_CALC) == EEP_OK)
??reset_md_12:
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x9C0         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
        CMP       A, #0x1            ;; 1 cycle
        BNZ       ??reset_md_13      ;; 4 cycles
        ; ------------------------------------- Block: 12 cycles
//  123   {
//  124     last_energy_mag = char_array_to_long4(&opr_data[0]);
        MOVW      AX, #LWRD(_opr_data)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_last_energy_mag, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_last_energy_mag+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        BR        S:??reset_md_14    ;; 3 cycles
        ; ------------------------------------- Block: 11 cycles
//  125   }
//  126   else
//  127   {
//  128     last_energy_mag = energy.Allph.defraud_mag;
??reset_md_13:
        MOVW      BC, N:_energy+62   ;; 1 cycle
        MOVW      AX, N:_energy+60   ;; 1 cycle
        MOVW      N:_last_energy_mag, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_last_energy_mag+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
//  129   }
//  130   
//  131   if(eprom_read(0x09D0,0,PAGE_1,AUTO_CALC) == EEP_OK)
??reset_md_14:
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x9D0         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
        CMP       A, #0x1            ;; 1 cycle
        BNZ       ??reset_md_15      ;; 4 cycles
        ; ------------------------------------- Block: 12 cycles
//  132   {
//  133     last_energy_mag_Rph = char_array_to_long4(&opr_data[0]);
        MOVW      AX, #LWRD(_opr_data)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_last_energy_mag_Rph, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_last_energy_mag_Rph+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  134     last_energy_mag_Yph = char_array_to_long4(&opr_data[4]);
        MOVW      AX, #LWRD(_opr_data+4)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_last_energy_mag_Yph, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_last_energy_mag_Yph+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  135     last_energy_mag_Bph = char_array_to_long4(&opr_data[8]);
        MOVW      AX, #LWRD(_opr_data+8)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_last_energy_mag_Bph, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_last_energy_mag_Bph+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        BR        S:??reset_md_16    ;; 3 cycles
        ; ------------------------------------- Block: 27 cycles
//  136   }
//  137   else
//  138   {
//  139     last_energy_mag_Rph = energy.Rph.defraud_mag;
??reset_md_15:
        MOVW      BC, N:_energy+12   ;; 1 cycle
        MOVW      AX, N:_energy+10   ;; 1 cycle
        MOVW      N:_last_energy_mag_Rph, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_last_energy_mag_Rph+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  140     last_energy_mag_Yph = energy.Yph.defraud_mag;
        MOVW      BC, N:_energy+26   ;; 1 cycle
        MOVW      AX, N:_energy+24   ;; 1 cycle
        MOVW      N:_last_energy_mag_Yph, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_last_energy_mag_Yph+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  141     last_energy_mag_Bph = energy.Bph.defraud_mag;
        MOVW      BC, N:_energy+40   ;; 1 cycle
        MOVW      AX, N:_energy+38   ;; 1 cycle
        MOVW      N:_last_energy_mag_Bph, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_last_energy_mag_Bph+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ; ------------------------------------- Block: 18 cycles
//  142   }
//  143   /**********************Calculating present demand ********************/
//  144   /* *******************Cum maximum Demand with date & time**************** */
//  145   if(miss_md_date_f==1) /* for calculate off time to next md integration time */
??reset_md_16:
        CMP       N:_miss_md_date_f, #0x1  ;; 1 cycle
        BNZ       ??reset_md_17      ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  146   {
//  147     TempTime = md_next_interval_timestamp(mdi_period,OffTime);
        MOVW      HL, #LWRD(_OffTime)  ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+24
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOV       C, N:_mdi_period   ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0xC           ;; 1 cycle
          CFI FunCall _md_next_interval_timestamp
        CALL      _md_next_interval_timestamp  ;; 3 cycles
        MOVW      HL, #LWRD(_TempTime)  ;; 1 cycle
        ADDW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+16
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
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
        ; ------------------------------------- Block: 32 cycles
//  148   }
//  149   
//  150   /* Calculating present demand values */
//  151   demand.Allph.act_imp.present.value = cal_present(energy.Allph.active_imp, demand.Allph.act_imp.present.last_energy_value,0,energy.Allph.defraud_mag,last_energy_mag,1);
??reset_md_17:
        MOVW      AX, N:_last_energy_mag+2  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+18
        MOVW      AX, N:_last_energy_mag  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+20
        MOVW      AX, N:_energy+62   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+22
        MOVW      AX, N:_energy+60   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+24
        MOVW      AX, N:_demand+130  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+26
        MOVW      AX, N:_demand+128  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+28
        MOV       D, #0x1            ;; 1 cycle
        MOV       E, #0x0            ;; 1 cycle
        MOVW      BC, N:_energy+54   ;; 1 cycle
        MOVW      AX, N:_energy+52   ;; 1 cycle
          CFI FunCall _cal_present
        CALL      _cal_present       ;; 3 cycles
        MOVW      N:_demand+132, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+134, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  152   demand.Allph.act_imp.present.last_energy_value = energy.Allph.active_imp;
        MOVW      BC, N:_energy+54   ;; 1 cycle
        MOVW      AX, N:_energy+52   ;; 1 cycle
        MOVW      N:_demand+128, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+130, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  153   demand.Allph.act_exp.present.value = cal_present(energy.Allph.active_exp, demand.Allph.act_exp.present.last_energy_value,0,energy.Allph.defraud_mag,last_energy_mag,0);
        MOVW      AX, N:_last_energy_mag+2  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+30
        MOVW      AX, N:_last_energy_mag  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
        MOVW      AX, N:_energy+62   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+34
        MOVW      AX, N:_energy+60   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+36
        MOVW      AX, N:_demand+154  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+38
        MOVW      AX, N:_demand+152  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+40
        MOV       D, #0x0            ;; 1 cycle
        MOV       E, #0x0            ;; 1 cycle
        MOVW      BC, N:_energy+58   ;; 1 cycle
        MOVW      AX, N:_energy+56   ;; 1 cycle
          CFI FunCall _cal_present
        CALL      _cal_present       ;; 3 cycles
        MOVW      N:_demand+156, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+158, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  154   demand.Allph.act_exp.present.last_energy_value = energy.Allph.active_exp;
        MOVW      BC, N:_energy+58   ;; 1 cycle
        MOVW      AX, N:_energy+56   ;; 1 cycle
        MOVW      N:_demand+152, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+154, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  155   demand.Allph.app_imp.present.value = cal_present(energy.Allph.apparent_imp, demand.Allph.app_imp.present.last_energy_value,0,energy.Allph.defraud_mag,last_energy_mag,1);
        MOVW      AX, N:_last_energy_mag+2  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+42
        MOVW      AX, N:_last_energy_mag  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+44
        MOVW      AX, N:_energy+62   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+46
        MOVW      AX, N:_energy+60   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+48
        MOVW      AX, N:_demand+178  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+50
        MOVW      AX, N:_demand+176  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+52
        MOV       D, #0x1            ;; 1 cycle
        MOV       E, #0x0            ;; 1 cycle
        MOVW      BC, N:_energy+70   ;; 1 cycle
        MOVW      AX, N:_energy+68   ;; 1 cycle
          CFI FunCall _cal_present
        CALL      _cal_present       ;; 3 cycles
        ADDW      SP, #0x24          ;; 1 cycle
          CFI CFA SP+16
        MOVW      N:_demand+180, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+182, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  156   demand.Allph.app_imp.present.last_energy_value = energy.Allph.apparent_imp;
        MOVW      BC, N:_energy+70   ;; 1 cycle
        MOVW      AX, N:_energy+68   ;; 1 cycle
        MOVW      N:_demand+176, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+178, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  157   demand.Allph.app_exp.present.value = cal_present(energy.Allph.apparent_exp, demand.Allph.app_exp.present.last_energy_value,0,energy.Allph.defraud_mag,last_energy_mag,0);
        MOVW      AX, N:_last_energy_mag+2  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+18
        MOVW      AX, N:_last_energy_mag  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+20
        MOVW      AX, N:_energy+62   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+22
        MOVW      AX, N:_energy+60   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+24
        MOVW      AX, N:_demand+202  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+26
        MOVW      AX, N:_demand+200  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+28
        MOV       D, #0x0            ;; 1 cycle
        MOV       E, #0x0            ;; 1 cycle
        MOVW      BC, N:_energy+74   ;; 1 cycle
        MOVW      AX, N:_energy+72   ;; 1 cycle
          CFI FunCall _cal_present
        CALL      _cal_present       ;; 3 cycles
        MOVW      N:_demand+204, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+206, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  158   demand.Allph.app_exp.present.last_energy_value = energy.Allph.apparent_exp;
        MOVW      BC, N:_energy+74   ;; 1 cycle
        MOVW      AX, N:_energy+72   ;; 1 cycle
        MOVW      N:_demand+200, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+202, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  159   last_energy_mag = energy.Allph.defraud_mag;
        MOVW      BC, N:_energy+62   ;; 1 cycle
        MOVW      AX, N:_energy+60   ;; 1 cycle
        MOVW      N:_last_energy_mag, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_last_energy_mag+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  160   
//  161   demand.Rph.act_imp.present.value = cal_present(energy.Rph.active_imp, demand.Rph.act_imp.present.last_energy_value,1,energy.Rph.defraud_mag,last_energy_mag_Rph,1);
        MOVW      AX, N:_last_energy_mag_Rph+2  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+30
        MOVW      AX, N:_last_energy_mag_Rph  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
        MOVW      AX, N:_energy+12   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+34
        MOVW      AX, N:_energy+10   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+36
        MOVW      AX, N:_demand+10   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+38
        MOVW      AX, N:_demand+8    ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+40
        MOV       D, #0x1            ;; 1 cycle
        MOV       E, #0x1            ;; 1 cycle
        MOVW      BC, N:_energy+4    ;; 1 cycle
        MOVW      AX, N:_energy+2    ;; 1 cycle
          CFI FunCall _cal_present
        CALL      _cal_present       ;; 3 cycles
        MOVW      N:_demand+12, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+14, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  162   demand.Rph.act_imp.present.last_energy_value = energy.Rph.active_imp;
        MOVW      BC, N:_energy+4    ;; 1 cycle
        MOVW      AX, N:_energy+2    ;; 1 cycle
        MOVW      N:_demand+8, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+10, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  163   last_energy_mag_Rph = energy.Rph.defraud_mag;
        MOVW      BC, N:_energy+12   ;; 1 cycle
        MOVW      AX, N:_energy+10   ;; 1 cycle
        MOVW      N:_last_energy_mag_Rph, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_last_energy_mag_Rph+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  164   demand.Rph.act_exp.present.value = cal_present(energy.Rph.active_exp, demand.Rph.act_exp.present.last_energy_value,1,energy.Rph.defraud_mag,last_energy_mag_Rph,0);
        MOVW      AX, N:_last_energy_mag_Rph+2  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+42
        MOVW      AX, N:_last_energy_mag_Rph  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+44
        MOVW      AX, N:_energy+12   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+46
        MOVW      AX, N:_energy+10   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+48
        MOVW      AX, N:_demand+30   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+50
        MOVW      AX, N:_demand+28   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+52
        MOV       D, #0x0            ;; 1 cycle
        MOV       E, #0x1            ;; 1 cycle
        MOVW      BC, N:_energy+8    ;; 1 cycle
        MOVW      AX, N:_energy+6    ;; 1 cycle
          CFI FunCall _cal_present
        CALL      _cal_present       ;; 3 cycles
        ADDW      SP, #0x24          ;; 1 cycle
          CFI CFA SP+16
        MOVW      N:_demand+32, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+34, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  165   demand.Rph.act_exp.present.last_energy_value = energy.Rph.active_exp;
        MOVW      BC, N:_energy+8    ;; 1 cycle
        MOVW      AX, N:_energy+6    ;; 1 cycle
        MOVW      N:_demand+28, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+30, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  166   
//  167   demand.Yph.act_imp.present.value = cal_present(energy.Yph.active_imp, demand.Yph.act_imp.present.last_energy_value,1,energy.Yph.defraud_mag,last_energy_mag_Yph,1);
        MOVW      AX, N:_last_energy_mag_Yph+2  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+18
        MOVW      AX, N:_last_energy_mag_Yph  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+20
        MOVW      AX, N:_energy+26   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+22
        MOVW      AX, N:_energy+24   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+24
        MOVW      AX, N:_demand+50   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+26
        MOVW      AX, N:_demand+48   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+28
        MOV       D, #0x1            ;; 1 cycle
        MOV       E, #0x1            ;; 1 cycle
        MOVW      BC, N:_energy+18   ;; 1 cycle
        MOVW      AX, N:_energy+16   ;; 1 cycle
          CFI FunCall _cal_present
        CALL      _cal_present       ;; 3 cycles
        MOVW      N:_demand+52, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+54, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  168   demand.Yph.act_imp.present.last_energy_value = energy.Yph.active_imp;
        MOVW      BC, N:_energy+18   ;; 1 cycle
        MOVW      AX, N:_energy+16   ;; 1 cycle
        MOVW      N:_demand+48, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+50, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  169   last_energy_mag_Yph = energy.Yph.defraud_mag;
        MOVW      BC, N:_energy+26   ;; 1 cycle
        MOVW      AX, N:_energy+24   ;; 1 cycle
        MOVW      N:_last_energy_mag_Yph, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_last_energy_mag_Yph+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  170   demand.Yph.act_exp.present.value = cal_present(energy.Yph.active_exp, demand.Yph.act_exp.present.last_energy_value,1,energy.Yph.defraud_mag,last_energy_mag_Yph,0);
        MOVW      AX, N:_last_energy_mag_Yph+2  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+30
        MOVW      AX, N:_last_energy_mag_Yph  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
        MOVW      AX, N:_energy+26   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+34
        MOVW      AX, N:_energy+24   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+36
        MOVW      AX, N:_demand+70   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+38
        MOVW      AX, N:_demand+68   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+40
        MOV       D, #0x0            ;; 1 cycle
        MOV       E, #0x1            ;; 1 cycle
        MOVW      BC, N:_energy+22   ;; 1 cycle
        MOVW      AX, N:_energy+20   ;; 1 cycle
          CFI FunCall _cal_present
        CALL      _cal_present       ;; 3 cycles
        MOVW      N:_demand+72, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+74, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  171   demand.Yph.act_exp.present.last_energy_value = energy.Yph.active_exp;
        MOVW      BC, N:_energy+22   ;; 1 cycle
        MOVW      AX, N:_energy+20   ;; 1 cycle
        MOVW      N:_demand+68, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+70, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  172   
//  173   demand.Bph.act_imp.present.value = cal_present(energy.Bph.active_imp, demand.Bph.act_imp.present.last_energy_value,1,energy.Bph.defraud_mag,last_energy_mag_Bph,1);
        MOVW      AX, N:_last_energy_mag_Bph+2  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+42
        MOVW      AX, N:_last_energy_mag_Bph  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+44
        MOVW      AX, N:_energy+40   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+46
        MOVW      AX, N:_energy+38   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+48
        MOVW      AX, N:_demand+90   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+50
        MOVW      AX, N:_demand+88   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+52
        MOV       D, #0x1            ;; 1 cycle
        MOV       E, #0x1            ;; 1 cycle
        MOVW      BC, N:_energy+32   ;; 1 cycle
        MOVW      AX, N:_energy+30   ;; 1 cycle
          CFI FunCall _cal_present
        CALL      _cal_present       ;; 3 cycles
        ADDW      SP, #0x24          ;; 1 cycle
          CFI CFA SP+16
        MOVW      N:_demand+92, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+94, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  174   demand.Bph.act_imp.present.last_energy_value = energy.Bph.active_imp;
        MOVW      BC, N:_energy+32   ;; 1 cycle
        MOVW      AX, N:_energy+30   ;; 1 cycle
        MOVW      N:_demand+88, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+90, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  175   last_energy_mag_Bph = energy.Bph.defraud_mag;
        MOVW      BC, N:_energy+40   ;; 1 cycle
        MOVW      AX, N:_energy+38   ;; 1 cycle
        MOVW      N:_last_energy_mag_Bph, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_last_energy_mag_Bph+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  176   demand.Bph.act_exp.present.value = cal_present(energy.Bph.active_exp, demand.Bph.act_exp.present.last_energy_value,1,energy.Bph.defraud_mag,last_energy_mag_Bph,0);
        MOVW      AX, N:_last_energy_mag_Bph+2  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+18
        MOVW      AX, N:_last_energy_mag_Bph  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+20
        MOVW      AX, N:_energy+40   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+22
        MOVW      AX, N:_energy+38   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+24
        MOVW      AX, N:_demand+110  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+26
        MOVW      AX, N:_demand+108  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+28
        MOV       D, #0x0            ;; 1 cycle
        MOV       E, #0x1            ;; 1 cycle
        MOVW      BC, N:_energy+36   ;; 1 cycle
        MOVW      AX, N:_energy+34   ;; 1 cycle
          CFI FunCall _cal_present
        CALL      _cal_present       ;; 3 cycles
        MOVW      N:_demand+112, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+114, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  177   demand.Bph.act_exp.present.last_energy_value = energy.Bph.active_exp;
        MOVW      BC, N:_energy+36   ;; 1 cycle
        MOVW      AX, N:_energy+34   ;; 1 cycle
        MOVW      N:_demand+108, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+110, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  178   
//  179   
//  180   /* checks applied */
//  181   if(demand.Allph.act_imp.present.value > demand.Allph.app_imp.present.value)
        MOVW      AX, N:_demand+134  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+30
        MOVW      AX, N:_demand+132  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
        MOVW      BC, N:_demand+182  ;; 1 cycle
        MOVW      AX, N:_demand+180  ;; 1 cycle
          CFI FunCall ?UL_CMP_L03
        CALL      N:?UL_CMP_L03      ;; 3 cycles
        ADDW      SP, #0x10          ;; 1 cycle
          CFI CFA SP+16
        BNC       ??reset_md_18      ;; 4 cycles
        ; ------------------------------------- Block: 331 cycles
//  182   {
//  183     demand.Allph.app_imp.present.value = demand.Allph.act_imp.present.value;
        MOVW      BC, N:_demand+134  ;; 1 cycle
        MOVW      AX, N:_demand+132  ;; 1 cycle
        MOVW      N:_demand+180, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+182, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
//  184   }
//  185   if(demand.Allph.act_exp.present.value > demand.Allph.app_exp.present.value)
??reset_md_18:
        MOVW      AX, N:_demand+158  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+18
        MOVW      AX, N:_demand+156  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+20
        MOVW      BC, N:_demand+206  ;; 1 cycle
        MOVW      AX, N:_demand+204  ;; 1 cycle
          CFI FunCall ?UL_CMP_L03
        CALL      N:?UL_CMP_L03      ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+16
        BNC       ??reset_md_19      ;; 4 cycles
        ; ------------------------------------- Block: 14 cycles
//  186   {
//  187     demand.Allph.app_exp.present.value = demand.Allph.act_exp.present.value;
        MOVW      BC, N:_demand+158  ;; 1 cycle
        MOVW      AX, N:_demand+156  ;; 1 cycle
        MOVW      N:_demand+204, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+206, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
//  188   }
//  189   
//  190   /********************* Saving present demand values ************************/
//  191   long_into_char_array4(demand.Allph.act_imp.present.value, &opr_data[0]);
??reset_md_19:
        MOVW      DE, #LWRD(_opr_data)  ;; 1 cycle
        MOVW      BC, N:_demand+134  ;; 1 cycle
        MOVW      AX, N:_demand+132  ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  192   long_into_char_array4(demand.Allph.act_exp.present.value, &opr_data[7]);
        MOVW      DE, #LWRD(_opr_data+7)  ;; 1 cycle
        MOVW      BC, N:_demand+158  ;; 1 cycle
        MOVW      AX, N:_demand+156  ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  193   if(miss_md_date_f==1)
        CMP       N:_miss_md_date_f, #0x1  ;; 1 cycle
        BNZ       ??reset_md_20      ;; 4 cycles
        ; ------------------------------------- Block: 17 cycles
//  194   {
//  195     time_into_char_array4(TempTime, &opr_data[3]);
        MOVW      HL, #LWRD(_TempTime)  ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+24
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      AX, #LWRD(_opr_data+3)  ;; 1 cycle
          CFI FunCall _time_into_char_array4
        CALL      _time_into_char_array4  ;; 3 cycles
//  196     time_into_char_array4(TempTime, &opr_data[10]);
        MOVW      HL, #LWRD(_TempTime)  ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+32
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      AX, #LWRD(_opr_data+10)  ;; 1 cycle
          CFI FunCall _time_into_char_array4
        CALL      _time_into_char_array4  ;; 3 cycles
        ADDW      SP, #0x10          ;; 1 cycle
          CFI CFA SP+16
        BR        S:??reset_md_21    ;; 3 cycles
        ; ------------------------------------- Block: 26 cycles
//  197   }
//  198   else
//  199   {
//  200     time_into_char_array4(Now, &opr_data[3]);
??reset_md_20:
        MOVW      HL, #LWRD(_Now)    ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+24
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      AX, #LWRD(_opr_data+3)  ;; 1 cycle
          CFI FunCall _time_into_char_array4
        CALL      _time_into_char_array4  ;; 3 cycles
//  201     time_into_char_array4(Now, &opr_data[10]);
        MOVW      HL, #LWRD(_Now)    ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+32
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      AX, #LWRD(_opr_data+10)  ;; 1 cycle
          CFI FunCall _time_into_char_array4
        CALL      _time_into_char_array4  ;; 3 cycles
        ADDW      SP, #0x10          ;; 1 cycle
          CFI CFA SP+16
        ; ------------------------------------- Block: 23 cycles
//  202   }
//  203   eprom_write(0x09E0,0,16,PAGE_1,AUTO_CALC);
??reset_md_21:
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+18
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x9E0         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  204   
//  205   long_into_char_array4(demand.Allph.app_imp.present.value, &opr_data[0]);
        MOVW      DE, #LWRD(_opr_data)  ;; 1 cycle
        MOVW      BC, N:_demand+182  ;; 1 cycle
        MOVW      AX, N:_demand+180  ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  206   long_into_char_array4(demand.Allph.app_exp.present.value, &opr_data[7]);
        MOVW      DE, #LWRD(_opr_data+7)  ;; 1 cycle
        MOVW      BC, N:_demand+206  ;; 1 cycle
        MOVW      AX, N:_demand+204  ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  207   if(miss_md_date_f==1)
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+16
        CMP       N:_miss_md_date_f, #0x1  ;; 1 cycle
        BNZ       ??reset_md_22      ;; 4 cycles
        ; ------------------------------------- Block: 27 cycles
//  208   {
//  209     time_into_char_array4(TempTime, &opr_data[3]);
        MOVW      HL, #LWRD(_TempTime)  ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+24
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      AX, #LWRD(_opr_data+3)  ;; 1 cycle
          CFI FunCall _time_into_char_array4
        CALL      _time_into_char_array4  ;; 3 cycles
//  210     time_into_char_array4(TempTime, &opr_data[10]);
        MOVW      HL, #LWRD(_TempTime)  ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+32
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      AX, #LWRD(_opr_data+10)  ;; 1 cycle
          CFI FunCall _time_into_char_array4
        CALL      _time_into_char_array4  ;; 3 cycles
        ADDW      SP, #0x10          ;; 1 cycle
          CFI CFA SP+16
        BR        S:??reset_md_23    ;; 3 cycles
        ; ------------------------------------- Block: 26 cycles
//  211   }
//  212   else
//  213   {
//  214     time_into_char_array4(Now, &opr_data[3]);
??reset_md_22:
        MOVW      HL, #LWRD(_Now)    ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+24
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      AX, #LWRD(_opr_data+3)  ;; 1 cycle
          CFI FunCall _time_into_char_array4
        CALL      _time_into_char_array4  ;; 3 cycles
//  215     time_into_char_array4(Now, &opr_data[10]);
        MOVW      HL, #LWRD(_Now)    ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+32
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      AX, #LWRD(_opr_data+10)  ;; 1 cycle
          CFI FunCall _time_into_char_array4
        CALL      _time_into_char_array4  ;; 3 cycles
        ADDW      SP, #0x10          ;; 1 cycle
          CFI CFA SP+16
        ; ------------------------------------- Block: 23 cycles
//  216   }
//  217   eprom_write(0x09F0,0,16,PAGE_1,AUTO_CALC);
??reset_md_23:
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+18
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x9F0         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  218   
//  219   long_into_char_array4(demand.Rph.act_imp.present.value, &opr_data[0]);
        MOVW      DE, #LWRD(_opr_data)  ;; 1 cycle
        MOVW      BC, N:_demand+14   ;; 1 cycle
        MOVW      AX, N:_demand+12   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  220   long_into_char_array4(demand.Rph.act_exp.present.value, &opr_data[7]);
        MOVW      DE, #LWRD(_opr_data+7)  ;; 1 cycle
        MOVW      BC, N:_demand+34   ;; 1 cycle
        MOVW      AX, N:_demand+32   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  221   if(miss_md_date_f==1)
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+16
        CMP       N:_miss_md_date_f, #0x1  ;; 1 cycle
        BNZ       ??reset_md_24      ;; 4 cycles
        ; ------------------------------------- Block: 27 cycles
//  222   {
//  223       time_into_char_array4(TempTime, &opr_data[3]);
        MOVW      HL, #LWRD(_TempTime)  ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+24
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      AX, #LWRD(_opr_data+3)  ;; 1 cycle
          CFI FunCall _time_into_char_array4
        CALL      _time_into_char_array4  ;; 3 cycles
//  224       time_into_char_array4(TempTime, &opr_data[10]);
        MOVW      HL, #LWRD(_TempTime)  ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+32
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      AX, #LWRD(_opr_data+10)  ;; 1 cycle
          CFI FunCall _time_into_char_array4
        CALL      _time_into_char_array4  ;; 3 cycles
        ADDW      SP, #0x10          ;; 1 cycle
          CFI CFA SP+16
        BR        S:??reset_md_25    ;; 3 cycles
        ; ------------------------------------- Block: 26 cycles
//  225   }
//  226   else
//  227   {
//  228       time_into_char_array4(Now, &opr_data[3]);
??reset_md_24:
        MOVW      HL, #LWRD(_Now)    ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+24
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      AX, #LWRD(_opr_data+3)  ;; 1 cycle
          CFI FunCall _time_into_char_array4
        CALL      _time_into_char_array4  ;; 3 cycles
//  229       time_into_char_array4(Now, &opr_data[10]);
        MOVW      HL, #LWRD(_Now)    ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+32
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      AX, #LWRD(_opr_data+10)  ;; 1 cycle
          CFI FunCall _time_into_char_array4
        CALL      _time_into_char_array4  ;; 3 cycles
        ADDW      SP, #0x10          ;; 1 cycle
          CFI CFA SP+16
        ; ------------------------------------- Block: 23 cycles
//  230   }
//  231   eprom_write(0x0300,0,16,PAGE_1,AUTO_CALC);
??reset_md_25:
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+18
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x300         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  232   
//  233   long_into_char_array4(demand.Yph.act_imp.present.value, &opr_data[0]);
        MOVW      DE, #LWRD(_opr_data)  ;; 1 cycle
        MOVW      BC, N:_demand+54   ;; 1 cycle
        MOVW      AX, N:_demand+52   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  234   long_into_char_array4(demand.Yph.act_exp.present.value, &opr_data[7]);
        MOVW      DE, #LWRD(_opr_data+7)  ;; 1 cycle
        MOVW      BC, N:_demand+74   ;; 1 cycle
        MOVW      AX, N:_demand+72   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  235   if(miss_md_date_f==1)
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+16
        CMP       N:_miss_md_date_f, #0x1  ;; 1 cycle
        BNZ       ??reset_md_26      ;; 4 cycles
        ; ------------------------------------- Block: 27 cycles
//  236   {
//  237       time_into_char_array4(TempTime, &opr_data[3]);
        MOVW      HL, #LWRD(_TempTime)  ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+24
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      AX, #LWRD(_opr_data+3)  ;; 1 cycle
          CFI FunCall _time_into_char_array4
        CALL      _time_into_char_array4  ;; 3 cycles
//  238       time_into_char_array4(TempTime, &opr_data[10]);
        MOVW      HL, #LWRD(_TempTime)  ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+32
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      AX, #LWRD(_opr_data+10)  ;; 1 cycle
          CFI FunCall _time_into_char_array4
        CALL      _time_into_char_array4  ;; 3 cycles
        ADDW      SP, #0x10          ;; 1 cycle
          CFI CFA SP+16
        BR        S:??reset_md_27    ;; 3 cycles
        ; ------------------------------------- Block: 26 cycles
//  239   }
//  240   else
//  241   {
//  242       time_into_char_array4(Now, &opr_data[3]);
??reset_md_26:
        MOVW      HL, #LWRD(_Now)    ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+24
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      AX, #LWRD(_opr_data+3)  ;; 1 cycle
          CFI FunCall _time_into_char_array4
        CALL      _time_into_char_array4  ;; 3 cycles
//  243       time_into_char_array4(Now, &opr_data[10]);
        MOVW      HL, #LWRD(_Now)    ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+32
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      AX, #LWRD(_opr_data+10)  ;; 1 cycle
          CFI FunCall _time_into_char_array4
        CALL      _time_into_char_array4  ;; 3 cycles
        ADDW      SP, #0x10          ;; 1 cycle
          CFI CFA SP+16
        ; ------------------------------------- Block: 23 cycles
//  244   }
//  245   eprom_write(0x0310,0,16,PAGE_1,AUTO_CALC);
??reset_md_27:
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+18
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x310         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  246   
//  247   long_into_char_array4(demand.Bph.act_imp.present.value, &opr_data[0]);
        MOVW      DE, #LWRD(_opr_data)  ;; 1 cycle
        MOVW      BC, N:_demand+94   ;; 1 cycle
        MOVW      AX, N:_demand+92   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  248   long_into_char_array4(demand.Bph.act_exp.present.value, &opr_data[7]);
        MOVW      DE, #LWRD(_opr_data+7)  ;; 1 cycle
        MOVW      BC, N:_demand+114  ;; 1 cycle
        MOVW      AX, N:_demand+112  ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  249   if(miss_md_date_f==1)
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+16
        CMP       N:_miss_md_date_f, #0x1  ;; 1 cycle
        BNZ       ??reset_md_28      ;; 4 cycles
        ; ------------------------------------- Block: 27 cycles
//  250   {
//  251       time_into_char_array4(TempTime, &opr_data[3]);
        MOVW      HL, #LWRD(_TempTime)  ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+24
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      AX, #LWRD(_opr_data+3)  ;; 1 cycle
          CFI FunCall _time_into_char_array4
        CALL      _time_into_char_array4  ;; 3 cycles
//  252       time_into_char_array4(TempTime, &opr_data[10]);
        MOVW      HL, #LWRD(_TempTime)  ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+32
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      AX, #LWRD(_opr_data+10)  ;; 1 cycle
          CFI FunCall _time_into_char_array4
        CALL      _time_into_char_array4  ;; 3 cycles
        ADDW      SP, #0x10          ;; 1 cycle
          CFI CFA SP+16
        BR        S:??reset_md_29    ;; 3 cycles
        ; ------------------------------------- Block: 26 cycles
//  253   }
//  254   else
//  255   {
//  256       time_into_char_array4(Now, &opr_data[3]);
??reset_md_28:
        MOVW      HL, #LWRD(_Now)    ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+24
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      AX, #LWRD(_opr_data+3)  ;; 1 cycle
          CFI FunCall _time_into_char_array4
        CALL      _time_into_char_array4  ;; 3 cycles
//  257       time_into_char_array4(Now, &opr_data[10]);
        MOVW      HL, #LWRD(_Now)    ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+32
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      AX, #LWRD(_opr_data+10)  ;; 1 cycle
          CFI FunCall _time_into_char_array4
        CALL      _time_into_char_array4  ;; 3 cycles
        ADDW      SP, #0x10          ;; 1 cycle
          CFI CFA SP+16
        ; ------------------------------------- Block: 23 cycles
//  258   }
//  259   eprom_write(0x0320,0,16,PAGE_1,AUTO_CALC);
??reset_md_29:
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+18
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x320         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  260   
//  261   /************** Save Present Demand For TOD**********************/
//  262   eprom_read(0x0A00 + ((tariff_no-1)*0x20),0,PAGE_1,AUTO_CALC); 
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        XCH       A, E               ;; 1 cycle
        MOV       H, A               ;; 1 cycle
        XCH       A, E               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        XCH       A, C               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        XCH       A, C               ;; 1 cycle
        MOV       X, N:_tariff_no    ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x20          ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #0x9E0         ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       C, A               ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        XCH       A, E               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, E               ;; 1 cycle
        XCH       A, H               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        XCH       A, H               ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
//  263   long_into_char_array4(demand.Allph.act_imp.present.value, &opr_data[0]);
        MOVW      DE, #LWRD(_opr_data)  ;; 1 cycle
        MOVW      BC, N:_demand+134  ;; 1 cycle
        MOVW      AX, N:_demand+132  ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  264   long_into_char_array4(demand.Allph.act_exp.present.value, &opr_data[7]);
        MOVW      DE, #LWRD(_opr_data+7)  ;; 1 cycle
        MOVW      BC, N:_demand+158  ;; 1 cycle
        MOVW      AX, N:_demand+156  ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  265   if(miss_md_date_f==1)
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+16
        CMP       N:_miss_md_date_f, #0x1  ;; 1 cycle
        BNZ       ??reset_md_30      ;; 4 cycles
        ; ------------------------------------- Block: 57 cycles
//  266   {
//  267     time_into_char_array4(TempTime, &opr_data[3]);
        MOVW      HL, #LWRD(_TempTime)  ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+24
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      AX, #LWRD(_opr_data+3)  ;; 1 cycle
          CFI FunCall _time_into_char_array4
        CALL      _time_into_char_array4  ;; 3 cycles
//  268     time_into_char_array4(TempTime, &opr_data[10]);
        MOVW      HL, #LWRD(_TempTime)  ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+32
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      AX, #LWRD(_opr_data+10)  ;; 1 cycle
          CFI FunCall _time_into_char_array4
        CALL      _time_into_char_array4  ;; 3 cycles
        ADDW      SP, #0x10          ;; 1 cycle
          CFI CFA SP+16
        BR        S:??reset_md_31    ;; 3 cycles
        ; ------------------------------------- Block: 26 cycles
//  269   }
//  270   else
//  271   {
//  272     time_into_char_array4(Now, &opr_data[3]);
??reset_md_30:
        MOVW      HL, #LWRD(_Now)    ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+24
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      AX, #LWRD(_opr_data+3)  ;; 1 cycle
          CFI FunCall _time_into_char_array4
        CALL      _time_into_char_array4  ;; 3 cycles
//  273     time_into_char_array4(Now, &opr_data[10]);
        MOVW      HL, #LWRD(_Now)    ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+32
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      AX, #LWRD(_opr_data+10)  ;; 1 cycle
          CFI FunCall _time_into_char_array4
        CALL      _time_into_char_array4  ;; 3 cycles
        ADDW      SP, #0x10          ;; 1 cycle
          CFI CFA SP+16
        ; ------------------------------------- Block: 23 cycles
//  274   }
//  275   eprom_write(0x0A00 + ((tariff_no-1)*0x20),0,16,PAGE_1,AUTO_CALC); 
??reset_md_31:
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+18
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        MOV       H, A               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        PUSH      DE                 ;; 1 cycle
          CFI CFA SP+20
        XCH       A, C               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        XCH       A, C               ;; 1 cycle
        MOV       X, N:_tariff_no    ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x20          ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #0x9E0         ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       C, A               ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        POP       DE                 ;; 1 cycle
          CFI CFA SP+18
        XCH       A, H               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, H               ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  276   
//  277   eprom_read(0x0A10 + ((tariff_no-1)*0x20),0,PAGE_1,AUTO_CALC); 
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        XCH       A, E               ;; 1 cycle
        MOV       H, A               ;; 1 cycle
        XCH       A, E               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        XCH       A, C               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        XCH       A, C               ;; 1 cycle
        MOV       X, N:_tariff_no    ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x20          ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #0x9F0         ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       C, A               ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        XCH       A, E               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, E               ;; 1 cycle
        XCH       A, H               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        XCH       A, H               ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
//  278   long_into_char_array4(demand.Allph.app_imp.present.value, &opr_data[0]);
        MOVW      DE, #LWRD(_opr_data)  ;; 1 cycle
        MOVW      BC, N:_demand+182  ;; 1 cycle
        MOVW      AX, N:_demand+180  ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  279   long_into_char_array4(demand.Allph.app_exp.present.value, &opr_data[7]);
        MOVW      DE, #LWRD(_opr_data+7)  ;; 1 cycle
        MOVW      BC, N:_demand+206  ;; 1 cycle
        MOVW      AX, N:_demand+204  ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  280   if(miss_md_date_f==1)
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+16
        CMP       N:_miss_md_date_f, #0x1  ;; 1 cycle
        BNZ       ??reset_md_32      ;; 4 cycles
        ; ------------------------------------- Block: 76 cycles
//  281   {
//  282     time_into_char_array4(TempTime, &opr_data[3]);
        MOVW      HL, #LWRD(_TempTime)  ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+24
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      AX, #LWRD(_opr_data+3)  ;; 1 cycle
          CFI FunCall _time_into_char_array4
        CALL      _time_into_char_array4  ;; 3 cycles
//  283     time_into_char_array4(TempTime, &opr_data[10]);
        MOVW      HL, #LWRD(_TempTime)  ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+32
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      AX, #LWRD(_opr_data+10)  ;; 1 cycle
          CFI FunCall _time_into_char_array4
        CALL      _time_into_char_array4  ;; 3 cycles
        ADDW      SP, #0x10          ;; 1 cycle
          CFI CFA SP+16
        BR        S:??reset_md_33    ;; 3 cycles
        ; ------------------------------------- Block: 26 cycles
//  284   }
//  285   else
//  286   {
//  287     time_into_char_array4(Now, &opr_data[3]);
??reset_md_32:
        MOVW      HL, #LWRD(_Now)    ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+24
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      AX, #LWRD(_opr_data+3)  ;; 1 cycle
          CFI FunCall _time_into_char_array4
        CALL      _time_into_char_array4  ;; 3 cycles
//  288     time_into_char_array4(Now, &opr_data[10]);
        MOVW      HL, #LWRD(_Now)    ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+32
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      AX, #LWRD(_opr_data+10)  ;; 1 cycle
          CFI FunCall _time_into_char_array4
        CALL      _time_into_char_array4  ;; 3 cycles
        ADDW      SP, #0x10          ;; 1 cycle
          CFI CFA SP+16
        ; ------------------------------------- Block: 23 cycles
//  289   }
//  290   eprom_write(0x0A10 + ((tariff_no-1)*0x20),0,16,PAGE_1,AUTO_CALC); 
??reset_md_33:
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+18
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        MOV       H, A               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        PUSH      DE                 ;; 1 cycle
          CFI CFA SP+20
        XCH       A, C               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        XCH       A, C               ;; 1 cycle
        MOV       X, N:_tariff_no    ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x20          ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #0x9F0         ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       C, A               ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        POP       DE                 ;; 1 cycle
          CFI CFA SP+18
        XCH       A, H               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, H               ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  291   
//  292   /************************* checking for MD ****************************/
//  293   cal_md(demand.Allph.act_imp.present.value, &demand.Allph.act_imp.max.value, 0, 0x03E0);
        MOVW      AX, #0x3E0         ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+20
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+22
        MOVW      DE, #LWRD(_demand+140)  ;; 1 cycle
        MOVW      BC, N:_demand+134  ;; 1 cycle
        MOVW      AX, N:_demand+132  ;; 1 cycle
          CFI FunCall _cal_md
        CALL      _cal_md            ;; 3 cycles
//  294   cal_md(demand.Allph.act_exp.present.value, &demand.Allph.act_exp.max.value, 7, 0x03E0);
        MOVW      AX, #0x3E0         ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+24
        MOV       X, #0x7            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+26
        MOVW      DE, #LWRD(_demand+164)  ;; 1 cycle
        MOVW      BC, N:_demand+158  ;; 1 cycle
        MOVW      AX, N:_demand+156  ;; 1 cycle
          CFI FunCall _cal_md
        CALL      _cal_md            ;; 3 cycles
//  295   cal_md(demand.Allph.app_imp.present.value, &demand.Allph.app_imp.max.value, 0, 0x03F0);
        MOVW      AX, #0x3F0         ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+28
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+30
        MOVW      DE, #LWRD(_demand+188)  ;; 1 cycle
        MOVW      BC, N:_demand+182  ;; 1 cycle
        MOVW      AX, N:_demand+180  ;; 1 cycle
          CFI FunCall _cal_md
        CALL      _cal_md            ;; 3 cycles
//  296   cal_md(demand.Allph.app_exp.present.value, &demand.Allph.app_exp.max.value, 7, 0x03F0);
        MOVW      AX, #0x3F0         ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
        MOV       X, #0x7            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+34
        MOVW      DE, #LWRD(_demand+212)  ;; 1 cycle
        MOVW      BC, N:_demand+206  ;; 1 cycle
        MOVW      AX, N:_demand+204  ;; 1 cycle
          CFI FunCall _cal_md
        CALL      _cal_md            ;; 3 cycles
//  297   cal_md(demand.Rph.act_imp.present.value, &demand.Rph.act_imp.max.value, 0, 0x0380);
        MOVW      AX, #0x380         ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+36
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+38
        MOVW      DE, #LWRD(_demand+16)  ;; 1 cycle
        MOVW      BC, N:_demand+14   ;; 1 cycle
        MOVW      AX, N:_demand+12   ;; 1 cycle
          CFI FunCall _cal_md
        CALL      _cal_md            ;; 3 cycles
//  298   cal_md(demand.Rph.act_exp.present.value, &demand.Rph.act_exp.max.value, 7, 0x0380);
        MOVW      AX, #0x380         ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+40
        MOV       X, #0x7            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+42
        MOVW      DE, #LWRD(_demand+36)  ;; 1 cycle
        MOVW      BC, N:_demand+34   ;; 1 cycle
        MOVW      AX, N:_demand+32   ;; 1 cycle
          CFI FunCall _cal_md
        CALL      _cal_md            ;; 3 cycles
//  299   cal_md(demand.Yph.act_imp.present.value, &demand.Yph.act_imp.max.value, 0, 0x0390);
        MOVW      AX, #0x390         ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+44
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+46
        MOVW      DE, #LWRD(_demand+56)  ;; 1 cycle
        MOVW      BC, N:_demand+54   ;; 1 cycle
        MOVW      AX, N:_demand+52   ;; 1 cycle
          CFI FunCall _cal_md
        CALL      _cal_md            ;; 3 cycles
//  300   cal_md(demand.Yph.act_exp.present.value, &demand.Yph.act_exp.max.value, 7, 0x0390);
        MOVW      AX, #0x390         ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+48
        MOV       X, #0x7            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+50
        MOVW      DE, #LWRD(_demand+76)  ;; 1 cycle
        MOVW      BC, N:_demand+74   ;; 1 cycle
        MOVW      AX, N:_demand+72   ;; 1 cycle
          CFI FunCall _cal_md
        CALL      _cal_md            ;; 3 cycles
        ADDW      SP, #0x22          ;; 1 cycle
          CFI CFA SP+16
//  301   cal_md(demand.Bph.act_imp.present.value, &demand.Bph.act_imp.max.value, 0, 0x03A0);
        MOVW      AX, #0x3A0         ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+18
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+20
        MOVW      DE, #LWRD(_demand+96)  ;; 1 cycle
        MOVW      BC, N:_demand+94   ;; 1 cycle
        MOVW      AX, N:_demand+92   ;; 1 cycle
          CFI FunCall _cal_md
        CALL      _cal_md            ;; 3 cycles
//  302   cal_md(demand.Bph.act_exp.present.value, &demand.Bph.act_exp.max.value, 7, 0x03A0);
        MOVW      AX, #0x3A0         ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+22
        MOV       X, #0x7            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+24
        MOVW      DE, #LWRD(_demand+116)  ;; 1 cycle
        MOVW      BC, N:_demand+114  ;; 1 cycle
        MOVW      AX, N:_demand+112  ;; 1 cycle
          CFI FunCall _cal_md
        CALL      _cal_md            ;; 3 cycles
//  303   
//  304   cal_md_tod(demand.Allph.act_imp.present.value, &demand.Allph.act_imp.max.tod_value, 0, 0x0400);
        MOVW      AX, #0x400         ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+26
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+28
        MOVW      DE, #LWRD(_demand+136)  ;; 1 cycle
        MOVW      BC, N:_demand+134  ;; 1 cycle
        MOVW      AX, N:_demand+132  ;; 1 cycle
          CFI FunCall _cal_md_tod
        CALL      _cal_md_tod        ;; 3 cycles
//  305   cal_md_tod(demand.Allph.act_exp.present.value, &demand.Allph.act_exp.max.tod_value, 7, 0x0400);
        MOVW      AX, #0x400         ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+30
        MOV       X, #0x7            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
        MOVW      DE, #LWRD(_demand+160)  ;; 1 cycle
        MOVW      BC, N:_demand+158  ;; 1 cycle
        MOVW      AX, N:_demand+156  ;; 1 cycle
          CFI FunCall _cal_md_tod
        CALL      _cal_md_tod        ;; 3 cycles
//  306   cal_md_tod(demand.Allph.app_imp.present.value, &demand.Allph.app_imp.max.tod_value, 0, 0x0410);
        MOVW      AX, #0x410         ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+34
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+36
        MOVW      DE, #LWRD(_demand+184)  ;; 1 cycle
        MOVW      BC, N:_demand+182  ;; 1 cycle
        MOVW      AX, N:_demand+180  ;; 1 cycle
          CFI FunCall _cal_md_tod
        CALL      _cal_md_tod        ;; 3 cycles
//  307   cal_md_tod(demand.Allph.app_exp.present.value, &demand.Allph.app_exp.max.tod_value, 7, 0x0410);
        MOVW      AX, #0x410         ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+38
        MOV       X, #0x7            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+40
        MOVW      DE, #LWRD(_demand+208)  ;; 1 cycle
        MOVW      BC, N:_demand+206  ;; 1 cycle
        MOVW      AX, N:_demand+204  ;; 1 cycle
          CFI FunCall _cal_md_tod
        CALL      _cal_md_tod        ;; 3 cycles
//  308    
//  309   /* saving the last energy values for present demand */
//  310   /*********************** active demands**********************/
//  311   long_into_char_array4(demand.Allph.act_imp.present.last_energy_value,&opr_data[0]);
        MOVW      DE, #LWRD(_opr_data)  ;; 1 cycle
        MOVW      BC, N:_demand+130  ;; 1 cycle
        MOVW      AX, N:_demand+128  ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  312   long_into_char_array4(demand.Allph.act_exp.present.last_energy_value,&opr_data[4]);
        MOVW      DE, #LWRD(_opr_data+4)  ;; 1 cycle
        MOVW      BC, N:_demand+154  ;; 1 cycle
        MOVW      AX, N:_demand+152  ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  313   long_into_char_array4(demand.Allph.app_imp.present.last_energy_value,&opr_data[8]);
        MOVW      DE, #LWRD(_opr_data+8)  ;; 1 cycle
        MOVW      BC, N:_demand+178  ;; 1 cycle
        MOVW      AX, N:_demand+176  ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  314   
//  315   temp_min = bcd_to_decimal(Now.min);
        MOV       A, N:_Now+1        ;; 1 cycle
          CFI FunCall _bcd_to_decimal
        CALL      _bcd_to_decimal    ;; 3 cycles
        MOV       [SP+0x18], A       ;; 1 cycle
//  316   
//  317   if(zone_change_f==1)                 
        ADDW      SP, #0x18          ;; 1 cycle
          CFI CFA SP+16
        CMP       N:_zone_change_f, #0x1  ;; 1 cycle
        BNZ       ??reset_md_34      ;; 4 cycles
        ; ------------------------------------- Block: 198 cycles
//  318   {
//  319     eprom_write(0x0800,0,192,PAGE_1,AUTO_CALC); // sliding
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+18
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0xC0          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x800         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  320     eprom_write(0x0980,0,16,PAGE_1,AUTO_CALC); // fix 
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+20
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x980         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+16
        BR        S:??reset_md_35    ;; 3 cycles
        ; ------------------------------------- Block: 22 cycles
//  321   }
//  322   else
//  323   {
//  324     if(md_type == 1)
??reset_md_34:
        CMP       N:_md_type, #0x1   ;; 1 cycle
        BNZ       ??reset_md_36      ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  325     {
//  326       local_address=0x0800+((temp_min % mdi_interval_time) / mdi_period)*16;
        MOV       X, N:_mdi_interval_time  ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
          CFI FunCall ?UC_MOD_L01
        CALL      N:?UC_MOD_L01      ;; 3 cycles
        MOV       B, A               ;; 1 cycle
        MOV       A, B               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        MOV       X, N:_mdi_period   ;; 1 cycle
        MOV       A, B               ;; 1 cycle
          CFI FunCall ?UC_DIV_L01
        CALL      N:?UC_DIV_L01      ;; 3 cycles
        MOV       C, A               ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOVW      AX, #0x10          ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #0x800         ;; 1 cycle
        MOVW      [SP+0x02], AX      ;; 1 cycle
        BR        S:??reset_md_37    ;; 3 cycles
        ; ------------------------------------- Block: 23 cycles
//  327     }
//  328     else
//  329     {
//  330       local_address = 0x0980;
??reset_md_36:
        MOVW      AX, #0x980         ;; 1 cycle
        MOVW      [SP+0x02], AX      ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
//  331     }
//  332     eprom_write(local_address, 0, 16,PAGE_1,AUTO_CALC);
??reset_md_37:
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+18
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+16
        ; ------------------------------------- Block: 10 cycles
//  333   }
//  334   /*********************** Apparent demands**********************/
//  335   fill_oprzero(16);
??reset_md_35:
        MOV       A, #0x10           ;; 1 cycle
          CFI FunCall _fill_oprzero
        CALL      _fill_oprzero      ;; 3 cycles
//  336   long_into_char_array4(demand.Allph.app_exp.present.last_energy_value,&opr_data[0]);
        MOVW      DE, #LWRD(_opr_data)  ;; 1 cycle
        MOVW      BC, N:_demand+202  ;; 1 cycle
        MOVW      AX, N:_demand+200  ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  337   
//  338   if(zone_change_f==1)
        CMP       N:_zone_change_f, #0x1  ;; 1 cycle
        BNZ       ??reset_md_38      ;; 4 cycles
        ; ------------------------------------- Block: 15 cycles
//  339   {
//  340     eprom_write(0x08C0,0,192,PAGE_1,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+18
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0xC0          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x8C0         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  341     eprom_write(0x0990,0,16,PAGE_1,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+20
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x990         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+16
        BR        S:??reset_md_39    ;; 3 cycles
        ; ------------------------------------- Block: 22 cycles
//  342   }
//  343   else
//  344   {
//  345     if(md_type==1)
??reset_md_38:
        CMP       N:_md_type, #0x1   ;; 1 cycle
        BNZ       ??reset_md_40      ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  346     {
//  347       local_address=0x08C0+((temp_min % mdi_interval_time) / mdi_period)*16;
        MOV       X, N:_mdi_interval_time  ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
          CFI FunCall ?UC_MOD_L01
        CALL      N:?UC_MOD_L01      ;; 3 cycles
        MOV       B, A               ;; 1 cycle
        MOV       A, B               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        MOV       X, N:_mdi_period   ;; 1 cycle
        MOV       A, B               ;; 1 cycle
          CFI FunCall ?UC_DIV_L01
        CALL      N:?UC_DIV_L01      ;; 3 cycles
        MOV       C, A               ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOVW      AX, #0x10          ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #0x8C0         ;; 1 cycle
        MOVW      [SP+0x02], AX      ;; 1 cycle
        BR        S:??reset_md_41    ;; 3 cycles
        ; ------------------------------------- Block: 23 cycles
//  348     }
//  349     else
//  350     {
//  351       local_address=0x0990;
??reset_md_40:
        MOVW      AX, #0x990         ;; 1 cycle
        MOVW      [SP+0x02], AX      ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
//  352     }
//  353     eprom_write(local_address, 0, 16,PAGE_1,AUTO_CALC);
??reset_md_41:
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+18
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+16
        ; ------------------------------------- Block: 10 cycles
//  354   }
//  355   
//  356   /* Phasewise */
//  357   long_into_char_array4(demand.Rph.act_imp.present.last_energy_value,&opr_data[0]);
??reset_md_39:
        MOVW      DE, #LWRD(_opr_data)  ;; 1 cycle
        MOVW      BC, N:_demand+10   ;; 1 cycle
        MOVW      AX, N:_demand+8    ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  358   long_into_char_array4(demand.Yph.act_imp.present.last_energy_value,&opr_data[4]);
        MOVW      DE, #LWRD(_opr_data+4)  ;; 1 cycle
        MOVW      BC, N:_demand+50   ;; 1 cycle
        MOVW      AX, N:_demand+48   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  359   long_into_char_array4(demand.Bph.act_imp.present.last_energy_value,&opr_data[8]);
        MOVW      DE, #LWRD(_opr_data+8)  ;; 1 cycle
        MOVW      BC, N:_demand+90   ;; 1 cycle
        MOVW      AX, N:_demand+88   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  360   
//  361   temp_min = bcd_to_decimal(Now.min);
        MOV       A, N:_Now+1        ;; 1 cycle
          CFI FunCall _bcd_to_decimal
        CALL      _bcd_to_decimal    ;; 3 cycles
        MOV       [SP], A            ;; 1 cycle
//  362   
//  363   if(zone_change_f==1)                 
        CMP       N:_zone_change_f, #0x1  ;; 1 cycle
        BNZ       ??reset_md_42      ;; 4 cycles
        ; ------------------------------------- Block: 28 cycles
//  364   {
//  365     eprom_write(0x0360,0,16,PAGE_1,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+18
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x360         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+16
        BR        S:??reset_md_43    ;; 3 cycles
        ; ------------------------------------- Block: 13 cycles
//  366   }
//  367   else
//  368   {
//  369     local_address = 0x0360;
??reset_md_42:
        MOVW      AX, #0x360         ;; 1 cycle
        MOVW      [SP+0x02], AX      ;; 1 cycle
//  370     eprom_write(local_address, 0, 16,PAGE_1,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+18
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+16
        ; ------------------------------------- Block: 12 cycles
//  371   }
//  372   
//  373   long_into_char_array4(demand.Rph.act_exp.present.last_energy_value,&opr_data[0]);
??reset_md_43:
        MOVW      DE, #LWRD(_opr_data)  ;; 1 cycle
        MOVW      BC, N:_demand+30   ;; 1 cycle
        MOVW      AX, N:_demand+28   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  374   long_into_char_array4(demand.Yph.act_exp.present.last_energy_value,&opr_data[4]);
        MOVW      DE, #LWRD(_opr_data+4)  ;; 1 cycle
        MOVW      BC, N:_demand+70   ;; 1 cycle
        MOVW      AX, N:_demand+68   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  375   long_into_char_array4(demand.Bph.act_exp.present.last_energy_value,&opr_data[8]);
        MOVW      DE, #LWRD(_opr_data+8)  ;; 1 cycle
        MOVW      BC, N:_demand+110  ;; 1 cycle
        MOVW      AX, N:_demand+108  ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  376   
//  377   temp_min = bcd_to_decimal(Now.min);
        MOV       A, N:_Now+1        ;; 1 cycle
          CFI FunCall _bcd_to_decimal
        CALL      _bcd_to_decimal    ;; 3 cycles
        MOV       [SP], A            ;; 1 cycle
//  378   
//  379   if(zone_change_f==1)                
        CMP       N:_zone_change_f, #0x1  ;; 1 cycle
        BNZ       ??reset_md_44      ;; 4 cycles
        ; ------------------------------------- Block: 28 cycles
//  380   {
//  381     eprom_write(0x0370,0,16,PAGE_1,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+18
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x370         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+16
        BR        S:??reset_md_45    ;; 3 cycles
        ; ------------------------------------- Block: 13 cycles
//  382   }
//  383   else
//  384   {
//  385     local_address = 0x0370;
??reset_md_44:
        MOVW      AX, #0x370         ;; 1 cycle
        MOVW      [SP+0x02], AX      ;; 1 cycle
//  386     eprom_write(local_address, 0, 16,PAGE_1,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+18
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+16
        ; ------------------------------------- Block: 12 cycles
//  387   }
//  388   
//  389   /* saving last magent energies */
//  390   long_into_char_array4(last_energy_mag,&opr_data[0]);
??reset_md_45:
        MOVW      DE, #LWRD(_opr_data)  ;; 1 cycle
        MOVW      BC, N:_last_energy_mag+2  ;; 1 cycle
        MOVW      AX, N:_last_energy_mag  ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  391   
//  392   temp_min = bcd_to_decimal(Now.min);
        MOV       A, N:_Now+1        ;; 1 cycle
          CFI FunCall _bcd_to_decimal
        CALL      _bcd_to_decimal    ;; 3 cycles
        MOV       [SP], A            ;; 1 cycle
//  393   
//  394   if(zone_change_f==1)                
        CMP       N:_zone_change_f, #0x1  ;; 1 cycle
        BNZ       ??reset_md_46      ;; 4 cycles
        ; ------------------------------------- Block: 16 cycles
//  395   {
//  396     eprom_write(0x09C0,0,16,PAGE_1,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+18
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x9C0         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+16
        BR        S:??reset_md_47    ;; 3 cycles
        ; ------------------------------------- Block: 13 cycles
//  397   }
//  398   else
//  399   {
//  400     local_address = 0x09C0;
??reset_md_46:
        MOVW      AX, #0x9C0         ;; 1 cycle
        MOVW      [SP+0x02], AX      ;; 1 cycle
//  401     eprom_write(local_address, 0, 16,PAGE_1,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+18
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+16
        ; ------------------------------------- Block: 12 cycles
//  402   }
//  403   
//  404   long_into_char_array4(last_energy_mag_Rph,&opr_data[0]);
??reset_md_47:
        MOVW      DE, #LWRD(_opr_data)  ;; 1 cycle
        MOVW      BC, N:_last_energy_mag_Rph+2  ;; 1 cycle
        MOVW      AX, N:_last_energy_mag_Rph  ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  405   long_into_char_array4(last_energy_mag_Yph,&opr_data[4]);
        MOVW      DE, #LWRD(_opr_data+4)  ;; 1 cycle
        MOVW      BC, N:_last_energy_mag_Yph+2  ;; 1 cycle
        MOVW      AX, N:_last_energy_mag_Yph  ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  406   long_into_char_array4(last_energy_mag_Bph,&opr_data[8]);
        MOVW      DE, #LWRD(_opr_data+8)  ;; 1 cycle
        MOVW      BC, N:_last_energy_mag_Bph+2  ;; 1 cycle
        MOVW      AX, N:_last_energy_mag_Bph  ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  407   
//  408   temp_min = bcd_to_decimal(Now.min);
        MOV       A, N:_Now+1        ;; 1 cycle
          CFI FunCall _bcd_to_decimal
        CALL      _bcd_to_decimal    ;; 3 cycles
        MOV       [SP], A            ;; 1 cycle
//  409   
//  410   if(zone_change_f==1)                
        CMP       N:_zone_change_f, #0x1  ;; 1 cycle
        BNZ       ??reset_md_48      ;; 4 cycles
        ; ------------------------------------- Block: 28 cycles
//  411   {
//  412     eprom_write(0x09D0,0,16,PAGE_1,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+18
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x9D0         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+16
        BR        S:??reset_md_49    ;; 3 cycles
        ; ------------------------------------- Block: 13 cycles
//  413   }
//  414   else
//  415   {
//  416     local_address = 0x09D0;
??reset_md_48:
        MOVW      AX, #0x9D0         ;; 1 cycle
        MOVW      [SP+0x02], AX      ;; 1 cycle
//  417     eprom_write(local_address, 0, 16,PAGE_1,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+18
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+16
        ; ------------------------------------- Block: 12 cycles
//  418   }
//  419   
//  420   /* we are filling the missing slide here */
//  421   if(miss_md_date_f==1 && md_type==1)      /* For miss slides pending in GDEV72*/
??reset_md_49:
        CMP       N:_miss_md_date_f, #0x1  ;; 1 cycle
        SKZ                          ;; 4 cycles
        BR        N:??reset_md_50    ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        CMP       N:_md_type, #0x1   ;; 1 cycle
        SKZ                          ;; 4 cycles
        BR        N:??reset_md_50    ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  422   {
//  423     miss_md_date_f=0;
        MOV       N:_miss_md_date_f, #0x0  ;; 1 cycle
//  424     off_min = off_min % mdi_interval_time;
        MOV       X, N:_mdi_interval_time  ;; 1 cycle
        MOV       A, N:_off_min      ;; 1 cycle
          CFI FunCall ?UC_MOD_L01
        CALL      N:?UC_MOD_L01      ;; 3 cycles
        MOV       B, A               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        MOV       N:_off_min, A      ;; 1 cycle
        XCH       A, B               ;; 1 cycle
//  425     temp_min = bcd_to_decimal(Now.min) % mdi_interval_time;
        MOV       A, N:_Now+1        ;; 1 cycle
          CFI FunCall _bcd_to_decimal
        CALL      _bcd_to_decimal    ;; 3 cycles
        MOV       B, A               ;; 1 cycle
        MOV       X, N:_mdi_interval_time  ;; 1 cycle
        MOV       A, B               ;; 1 cycle
          CFI FunCall ?UC_MOD_L01
        CALL      N:?UC_MOD_L01      ;; 3 cycles
        MOV       B, A               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        XCH       A, B               ;; 1 cycle
//  426     if(off_min > temp_min)
        MOV       A, [SP]            ;; 1 cycle
        CMP       A, N:_off_min      ;; 1 cycle
        BNC       ??reset_md_51      ;; 4 cycles
        ; ------------------------------------- Block: 30 cycles
//  427     {
//  428       temp_min += mdi_interval_time;
        MOV       A, [SP]            ;; 1 cycle
        ADD       A, N:_mdi_interval_time  ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        ; ------------------------------------- Block: 3 cycles
//  429     }
//  430     
//  431     on_time=temp_min;
??reset_md_51:
        MOV       A, [SP]            ;; 1 cycle
        MOV       [SP+0x01], A       ;; 1 cycle
//  432     
//  433     long_into_char_array4(demand.Allph.act_imp.present.last_energy_value,&opr_data[0]);
        MOVW      DE, #LWRD(_opr_data)  ;; 1 cycle
        MOVW      BC, N:_demand+130  ;; 1 cycle
        MOVW      AX, N:_demand+128  ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  434     long_into_char_array4(demand.Allph.act_exp.present.last_energy_value,&opr_data[4]);
        MOVW      DE, #LWRD(_opr_data+4)  ;; 1 cycle
        MOVW      BC, N:_demand+154  ;; 1 cycle
        MOVW      AX, N:_demand+152  ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  435     long_into_char_array4(demand.Allph.app_imp.present.last_energy_value,&opr_data[8]);
        MOVW      DE, #LWRD(_opr_data+8)  ;; 1 cycle
        MOVW      BC, N:_demand+178  ;; 1 cycle
        MOVW      AX, N:_demand+176  ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  436     
//  437     
//  438     temp_us32 = datediff(OffTime.min,OffTime.hour,OffTime.day,OffTime.month,OffTime.year); /* difference in minutes */
        MOV       A, N:_OffTime+6    ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       B, N:_OffTime+5    ;; 1 cycle
        MOV       C, N:_OffTime+3    ;; 1 cycle
        MOV       X, N:_OffTime+2    ;; 1 cycle
        MOV       A, N:_OffTime+1    ;; 1 cycle
          CFI FunCall _datediff
        CALL      _datediff          ;; 3 cycles
        MOVW      S:_temp_us32, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_temp_us32+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  439     if(temp_us32 >= mdi_interval_time)                                  /* when whole period is jumped */
        MOV       X, N:_mdi_interval_time  ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+18
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+20
        MOVW      BC, S:_temp_us32+2  ;; 1 cycle
        MOVW      AX, S:_temp_us32   ;; 1 cycle
          CFI FunCall ?UL_CMP_L03
        CALL      N:?UL_CMP_L03      ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+16
        BC        ??reset_md_52      ;; 4 cycles
        ; ------------------------------------- Block: 48 cycles
//  440     {
//  441       eprom_write(0x0800,0,16*mdi_no_of_period,PAGE_1,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+18
        MOV       B, #0x0            ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        MOV       H, A               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        MOV       X, N:_mdi_no_of_period  ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x10          ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      DE, AX             ;; 1 cycle
        XCH       A, H               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, H               ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x800         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+16
        BR        S:??reset_md_53    ;; 3 cycles
        ; ------------------------------------- Block: 24 cycles
//  442     }
//  443     else
//  444     {
//  445       for(temp_us8=off_min; temp_us8<on_time; temp_us8++)
??reset_md_52:
        MOV       A, N:_off_min      ;; 1 cycle
        MOV       S:_temp_us8, A     ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
??cal_present_demand_1:
        MOV       X, S:_temp_us8     ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        CMP       X, A               ;; 1 cycle
        BNC       ??reset_md_53      ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  446       {
//  447         if(temp_us8%mdi_interval_time==0)
        MOV       X, N:_mdi_interval_time  ;; 1 cycle
        MOV       A, S:_temp_us8     ;; 1 cycle
          CFI FunCall ?UC_MOD_L01
        CALL      N:?UC_MOD_L01      ;; 3 cycles
        MOV       B, A               ;; 1 cycle
        CMP0      B                  ;; 1 cycle
        BNZ       ??reset_md_54      ;; 4 cycles
        ; ------------------------------------- Block: 11 cycles
//  448         {
//  449           eprom_write((0x0800+(16*((temp_us8%mdi_interval_time)/mdi_period))), 0, 16, PAGE_1, AUTO_CALC); 
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+18
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        MOV       H, A               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        PUSH      DE                 ;; 1 cycle
          CFI CFA SP+20
        XCH       A, C               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        XCH       A, C               ;; 1 cycle
        MOV       X, N:_mdi_interval_time  ;; 1 cycle
        MOV       A, S:_temp_us8     ;; 1 cycle
          CFI FunCall ?UC_MOD_L01
        CALL      N:?UC_MOD_L01      ;; 3 cycles
        MOV       B, A               ;; 1 cycle
        MOV       A, B               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        MOV       X, N:_mdi_period   ;; 1 cycle
        MOV       A, B               ;; 1 cycle
          CFI FunCall ?UC_DIV_L01
        CALL      N:?UC_DIV_L01      ;; 3 cycles
        MOV       C, A               ;; 1 cycle
        MOVW      AX, BC             ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x10          ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #0x800         ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       C, A               ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        POP       DE                 ;; 1 cycle
          CFI CFA SP+18
        XCH       A, H               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, H               ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+16
        ; ------------------------------------- Block: 43 cycles
//  450         }
//  451       }
??reset_md_54:
        INC       S:_temp_us8        ;; 2 cycles
        BR        S:??cal_present_demand_1  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  452     }
//  453     
//  454     fill_oprzero(16);
??reset_md_53:
        MOV       A, #0x10           ;; 1 cycle
          CFI FunCall _fill_oprzero
        CALL      _fill_oprzero      ;; 3 cycles
//  455     long_into_char_array4(demand.Allph.act_exp.present.last_energy_value,&opr_data[0]);
        MOVW      DE, #LWRD(_opr_data)  ;; 1 cycle
        MOVW      BC, N:_demand+154  ;; 1 cycle
        MOVW      AX, N:_demand+152  ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  456     
//  457     temp_us32 = datediff(OffTime.min,OffTime.hour,OffTime.day,OffTime.month,OffTime.year); /* difference in minutes */
        MOV       A, N:_OffTime+6    ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       B, N:_OffTime+5    ;; 1 cycle
        MOV       C, N:_OffTime+3    ;; 1 cycle
        MOV       X, N:_OffTime+2    ;; 1 cycle
        MOV       A, N:_OffTime+1    ;; 1 cycle
          CFI FunCall _datediff
        CALL      _datediff          ;; 3 cycles
        MOVW      S:_temp_us32, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_temp_us32+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  458     if(temp_us32 >= mdi_interval_time)
        MOV       X, N:_mdi_interval_time  ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+18
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+20
        MOVW      BC, S:_temp_us32+2  ;; 1 cycle
        MOVW      AX, S:_temp_us32   ;; 1 cycle
          CFI FunCall ?UL_CMP_L03
        CALL      N:?UL_CMP_L03      ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+16
        BC        ??reset_md_55      ;; 4 cycles
        ; ------------------------------------- Block: 38 cycles
//  459     {
//  460       eprom_write(0x08C0,0,16*mdi_no_of_period,PAGE_1,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+18
        MOV       B, #0x0            ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        MOV       H, A               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        MOV       X, N:_mdi_no_of_period  ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x10          ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      DE, AX             ;; 1 cycle
        XCH       A, H               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, H               ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x8C0         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+16
        BR        S:??reset_md_56    ;; 3 cycles
        ; ------------------------------------- Block: 24 cycles
//  461     }
//  462     else
//  463     {
//  464       for(temp_us8=off_min; temp_us8<on_time; temp_us8++)
??reset_md_55:
        MOV       A, N:_off_min      ;; 1 cycle
        MOV       S:_temp_us8, A     ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
??cal_present_demand_2:
        MOV       X, S:_temp_us8     ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        CMP       X, A               ;; 1 cycle
        BNC       ??reset_md_56      ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  465       {
//  466         if(temp_us8%mdi_interval_time==0)
        MOV       X, N:_mdi_interval_time  ;; 1 cycle
        MOV       A, S:_temp_us8     ;; 1 cycle
          CFI FunCall ?UC_MOD_L01
        CALL      N:?UC_MOD_L01      ;; 3 cycles
        MOV       B, A               ;; 1 cycle
        CMP0      B                  ;; 1 cycle
        BNZ       ??reset_md_57      ;; 4 cycles
        ; ------------------------------------- Block: 11 cycles
//  467         {
//  468             eprom_write((0x0800+(16*((temp_us8%mdi_interval_time)/mdi_period))), 0, 16, PAGE_1, AUTO_CALC); 
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+18
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        MOV       H, A               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        PUSH      DE                 ;; 1 cycle
          CFI CFA SP+20
        XCH       A, C               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        XCH       A, C               ;; 1 cycle
        MOV       X, N:_mdi_interval_time  ;; 1 cycle
        MOV       A, S:_temp_us8     ;; 1 cycle
          CFI FunCall ?UC_MOD_L01
        CALL      N:?UC_MOD_L01      ;; 3 cycles
        MOV       B, A               ;; 1 cycle
        MOV       A, B               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        MOV       X, N:_mdi_period   ;; 1 cycle
        MOV       A, B               ;; 1 cycle
          CFI FunCall ?UC_DIV_L01
        CALL      N:?UC_DIV_L01      ;; 3 cycles
        MOV       C, A               ;; 1 cycle
        MOVW      AX, BC             ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x10          ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #0x800         ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       C, A               ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        POP       DE                 ;; 1 cycle
          CFI CFA SP+18
        XCH       A, H               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, H               ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+16
        ; ------------------------------------- Block: 43 cycles
//  469         }
//  470       }
??reset_md_57:
        INC       S:_temp_us8        ;; 2 cycles
        BR        S:??cal_present_demand_2  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  471     }
//  472   }
//  473   else
//  474   {
//  475     miss_md_date_f=0;
??reset_md_50:
        MOV       N:_miss_md_date_f, #0x0  ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
//  476   }
//  477   
//  478   if(miss_md_date_f==1)
??reset_md_56:
        CMP       N:_miss_md_date_f, #0x1  ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
//  479   {
//  480     miss_md_date_f=0;
        MOV       N:_miss_md_date_f, #0x0  ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
//  481   }
//  482   eoi_complet_f=1;
??cal_present_demand_3:
        MOV       N:_eoi_complet_f, #0x1  ;; 1 cycle
//  483 }
        ADDW      SP, #0xC           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock0
        ; ------------------------------------- Block: 8 cycles
        ; ------------------------------------- Total: 2252 cycles

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock1 Using cfiCommon1
          CFI Function _cal_md
        CODE
//  484 void cal_md(us32 present_demand, us32 *max_demand, us8 opr_loc, us16 address)
//  485 {
_cal_md:
        ; * Stack frame (at entry) *
        ; Param size: 4
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+6
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+8
        PUSH      DE                 ;; 1 cycle
          CFI CFA SP+10
        ; Auto size: 6
//  486   if(present_demand >= *max_demand)
        MOVW      AX, [SP]           ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        CMP       A, [HL+0x03]       ;; 1 cycle
        BNZ       ??reset_md_58      ;; 4 cycles
        ; ------------------------------------- Block: 14 cycles
        MOV       A, C               ;; 1 cycle
        CMP       A, [HL+0x02]       ;; 1 cycle
        BNZ       ??reset_md_58      ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOV       A, B               ;; 1 cycle
        CMP       A, [HL+0x01]       ;; 1 cycle
        BNZ       ??reset_md_58      ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOV       A, X               ;; 1 cycle
        CMP       A, [HL]            ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
??reset_md_58:
        BC        ??reset_md_59      ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
//  487   {
//  488     eprom_read(address,0,PAGE_1,AUTO_CALC);
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, [SP+0x0C]      ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
//  489     *max_demand = present_demand;
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  490     long_into_char_array3(*max_demand, &opr_data[opr_loc]);
        MOV       A, [SP+0x0A]       ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        ADDW      AX, #LWRD(_opr_data)  ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [HL+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [HL]           ;; 1 cycle
          CFI FunCall _long_into_char_array3
        CALL      _long_into_char_array3  ;; 3 cycles
//  491     if(miss_md_date_f==1)
        CMP       N:_miss_md_date_f, #0x1  ;; 1 cycle
        BNZ       ??reset_md_60      ;; 4 cycles
        ; ------------------------------------- Block: 35 cycles
//  492     {
//  493       time_into_char_array4(TempTime, &opr_data[opr_loc + 3]);
        MOVW      HL, #LWRD(_TempTime)  ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+18
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOV       A, [SP+0x12]       ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        ADDW      AX, #LWRD(_opr_data+3)  ;; 1 cycle
          CFI FunCall _time_into_char_array4
        CALL      _time_into_char_array4  ;; 3 cycles
        ADDW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+10
        BR        S:??reset_md_61    ;; 3 cycles
        ; ------------------------------------- Block: 18 cycles
//  494     }
//  495     else
//  496     {
//  497       time_into_char_array4(Now, &opr_data[opr_loc + 3]);			
??reset_md_60:
        MOVW      HL, #LWRD(_Now)    ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+18
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOV       A, [SP+0x12]       ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        ADDW      AX, #LWRD(_opr_data+3)  ;; 1 cycle
          CFI FunCall _time_into_char_array4
        CALL      _time_into_char_array4  ;; 3 cycles
        ADDW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+10
        ; ------------------------------------- Block: 15 cycles
//  498     }
//  499     eprom_write(address,0,16,PAGE_1,AUTO_CALC);
??reset_md_61:
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, [SP+0x0E]      ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+10
        ; ------------------------------------- Block: 10 cycles
//  500   }
//  501 }
??reset_md_59:
        ADDW      SP, #0x6           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock1
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 117 cycles
//  502 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock2 Using cfiCommon1
          CFI Function _cal_md_tod
        CODE
//  503 void cal_md_tod(us32 present_demand, us32 *max_demand_tod, us8 opr_loc, us16 address)
//  504 {
_cal_md_tod:
        ; * Stack frame (at entry) *
        ; Param size: 4
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+6
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+8
        PUSH      DE                 ;; 1 cycle
          CFI CFA SP+10
        ; Auto size: 10
        SUBW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+14
//  505   us32 temp_demand;
//  506   eprom_read(address + ((tariff_no-1)*0x20),0,PAGE_1,AUTO_CALC);  
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        PUSH      DE                 ;; 1 cycle
          CFI CFA SP+16
        XCH       A, B               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        XCH       A, C               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        XCH       A, C               ;; 1 cycle
        MOV       X, N:_tariff_no    ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        DECW      AX                 ;; 1 cycle
        MOVW      BC, #0x20          ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [SP+0x12]      ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       C, A               ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        XCH       A, E               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+14
        XCH       A, E               ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
//  507   temp_demand = char_array_to_long3(&opr_data[opr_loc]);
        MOV       A, [SP+0x0E]       ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        ADDW      AX, #LWRD(_opr_data)  ;; 1 cycle
          CFI FunCall _char_array_to_long3
        CALL      _char_array_to_long3  ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+10
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+12
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+14
//  508   if(present_demand >= temp_demand)
        MOVW      AX, SP             ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [SP+0x08]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        CMP       A, [HL+0x03]       ;; 1 cycle
        BNZ       ??reset_md_62      ;; 4 cycles
        ; ------------------------------------- Block: 55 cycles
        MOV       A, C               ;; 1 cycle
        CMP       A, [HL+0x02]       ;; 1 cycle
        BNZ       ??reset_md_62      ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOV       A, B               ;; 1 cycle
        CMP       A, [HL+0x01]       ;; 1 cycle
        BNZ       ??reset_md_62      ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOV       A, X               ;; 1 cycle
        CMP       A, [HL]            ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
??reset_md_62:
        SKNC                         ;; 4 cycles
        BR        N:??reset_md_63    ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
//  509   {
//  510     *max_demand_tod = present_demand;
        MOVW      AX, [SP+0x08]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  511     long_into_char_array3(*max_demand_tod, &opr_data[opr_loc]);
        MOV       A, [SP+0x0E]       ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        ADDW      AX, #LWRD(_opr_data)  ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [HL+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [HL]           ;; 1 cycle
          CFI FunCall _long_into_char_array3
        CALL      _long_into_char_array3  ;; 3 cycles
//  512     
//  513     if(miss_md_date_f==1)
        CMP       N:_miss_md_date_f, #0x1  ;; 1 cycle
        BNZ       ??reset_md_64      ;; 4 cycles
        ; ------------------------------------- Block: 28 cycles
//  514     {
//  515       time_into_char_array4(TempTime, &opr_data[opr_loc + 3]);
        MOVW      HL, #LWRD(_TempTime)  ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+22
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOV       A, [SP+0x16]       ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        ADDW      AX, #LWRD(_opr_data+3)  ;; 1 cycle
          CFI FunCall _time_into_char_array4
        CALL      _time_into_char_array4  ;; 3 cycles
        ADDW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+14
        BR        S:??reset_md_65    ;; 3 cycles
        ; ------------------------------------- Block: 18 cycles
//  516     }
//  517     else
//  518     {
//  519       time_into_char_array4(Now, &opr_data[opr_loc + 3]);			
??reset_md_64:
        MOVW      HL, #LWRD(_Now)    ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+22
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOV       A, [SP+0x16]       ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        ADDW      AX, #LWRD(_opr_data+3)  ;; 1 cycle
          CFI FunCall _time_into_char_array4
        CALL      _time_into_char_array4  ;; 3 cycles
        ADDW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+14
        ; ------------------------------------- Block: 15 cycles
//  520     }
//  521     eprom_write(address+((tariff_no-1)*0x20),0,16,PAGE_1,AUTO_CALC); 
??reset_md_65:
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+16
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        MOV       H, A               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        PUSH      DE                 ;; 1 cycle
          CFI CFA SP+18
        XCH       A, C               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        XCH       A, C               ;; 1 cycle
        MOV       X, N:_tariff_no    ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        DECW      AX                 ;; 1 cycle
        MOVW      BC, #0x20          ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x14]      ;; 1 cycle
        ADDW      AX, BC             ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       C, A               ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        POP       DE                 ;; 1 cycle
          CFI CFA SP+16
        XCH       A, H               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, H               ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+14
        ; ------------------------------------- Block: 32 cycles
//  522   }
//  523 }
??reset_md_63:
        ADDW      SP, #0xA           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock2
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 173 cycles
//  524 
//  525 
//  526 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock3 Using cfiCommon1
          CFI Function _cal_present
        CODE
//  527 us32 cal_present(us32 present_energy, us32 last_energy, us8 phasewise, us32 mag_energy, us32 last_mag_energy, us8 sub_mag_demand)
//  528 {
_cal_present:
        ; * Stack frame (at entry) *
        ; Param size: 12
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+6
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+8
        PUSH      DE                 ;; 1 cycle
          CFI CFA SP+10
        ; Auto size: 14
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+18
//  529   us32 temp = 0,temp1 = 0;
        MOVW      AX, SP             ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
//  530   
//  531   /* calculating present demand */
//  532   if(present_energy < last_energy)
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x12          ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [SP+0x0C]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x0A]      ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        CMP       A, [HL+0x03]       ;; 1 cycle
        BNZ       ??reset_md_66      ;; 4 cycles
        ; ------------------------------------- Block: 29 cycles
        MOV       A, C               ;; 1 cycle
        CMP       A, [HL+0x02]       ;; 1 cycle
        BNZ       ??reset_md_66      ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOV       A, B               ;; 1 cycle
        CMP       A, [HL+0x01]       ;; 1 cycle
        BNZ       ??reset_md_66      ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOV       A, X               ;; 1 cycle
        CMP       A, [HL]            ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
??reset_md_66:
        BNC       ??reset_md_67      ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
//  533   {
//  534     temp = (us32)(present_energy + (ROLL_OVER_LIMIT - last_energy));
        MOVW      AX, [SP+0x14]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [SP+0x12]      ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, [SP+0x0C]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x0A]      ;; 1 cycle
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
        BR        S:??reset_md_68    ;; 3 cycles
        ; ------------------------------------- Block: 33 cycles
//  535   }
//  536   else
//  537   {
//  538     temp = (us32)(present_energy - last_energy);
??reset_md_67:
        MOVW      AX, [SP+0x14]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [SP+0x12]      ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, [SP+0x0C]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x0A]      ;; 1 cycle
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
//  539   }
//  540   if(sub_mag_demand == 1)
??reset_md_68:
        MOV       A, [SP+0x09]       ;; 1 cycle
        CMP       A, #0x1            ;; 1 cycle
        SKZ                          ;; 4 cycles
        BR        N:??reset_md_69    ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  541   {
//  542     if(mag_energy < last_mag_energy )
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x1A          ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [SP+0x18]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x16]      ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        CMP       A, [HL+0x03]       ;; 1 cycle
        BNZ       ??reset_md_70      ;; 4 cycles
        ; ------------------------------------- Block: 12 cycles
        MOV       A, C               ;; 1 cycle
        CMP       A, [HL+0x02]       ;; 1 cycle
        BNZ       ??reset_md_70      ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOV       A, B               ;; 1 cycle
        CMP       A, [HL+0x01]       ;; 1 cycle
        BNZ       ??reset_md_70      ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOV       A, X               ;; 1 cycle
        CMP       A, [HL]            ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
??reset_md_70:
        BNC       ??reset_md_71      ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
//  543     {
//  544       temp1 = (us32)(mag_energy + (ROLL_OVER_LIMIT  - last_mag_energy));
        MOVW      AX, [SP+0x1C]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [SP+0x1A]      ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, [SP+0x18]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x16]      ;; 1 cycle
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
        XCHW      AX, HL             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        BR        S:??reset_md_72    ;; 3 cycles
        ; ------------------------------------- Block: 36 cycles
//  545     }
//  546     else
//  547     {
//  548       temp1 = (us32)(mag_energy - last_mag_energy);
??reset_md_71:
        MOVW      AX, [SP+0x1C]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [SP+0x1A]      ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, [SP+0x18]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x16]      ;; 1 cycle
        SUBW      AX, DE             ;; 1 cycle
        SKNC
        DECW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        SUBW      AX, HL             ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ; ------------------------------------- Block: 24 cycles
//  549     }
//  550     if(temp >= temp1)
??reset_md_72:
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        CMP       A, [HL+0x03]       ;; 1 cycle
        BNZ       ??reset_md_73      ;; 4 cycles
        ; ------------------------------------- Block: 12 cycles
        MOV       A, C               ;; 1 cycle
        CMP       A, [HL+0x02]       ;; 1 cycle
        BNZ       ??reset_md_73      ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOV       A, B               ;; 1 cycle
        CMP       A, [HL+0x01]       ;; 1 cycle
        BNZ       ??reset_md_73      ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOV       A, X               ;; 1 cycle
        CMP       A, [HL]            ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
??reset_md_73:
        BC        ??reset_md_74      ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
//  551     {
//  552       temp = temp - temp1;
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
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
        BR        S:??reset_md_69    ;; 3 cycles
        ; ------------------------------------- Block: 24 cycles
//  553     }
//  554     else
//  555     {
//  556       temp = temp1 - temp;
??reset_md_74:
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
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
//  557     }
//  558   }
//  559  
//  560   temp *=  mdi_sel;
??reset_md_69:
        MOV       X, N:_mdi_sel      ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+20
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+22
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
          CFI FunCall ?L_MUL_FAST_L03
        CALL      N:?L_MUL_FAST_L03  ;; 3 cycles
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+24
        POP       HL                 ;; 1 cycle
          CFI CFA SP+22
        MOVW      DE, AX             ;; 1 cycle
//  561   
//  562   if((temp > ((us32)MD_LIMIT) && phasewise == 0) || (temp > ((us32)MD_LIMIT_PHASE) && phasewise == 1))
        MOVW      AX, DE             ;; 1 cycle
        PUSH      HL                 ;; 1 cycle
          CFI CFA SP+24
        POP       BC                 ;; 1 cycle
          CFI CFA SP+22
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+18
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x1           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 22 cycles
        CMPW      AX, #0x2FC1        ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??cal_present_0:
        BC        ??reset_md_75      ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
        MOV       A, [SP+0x08]       ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        BZ        ??reset_md_76      ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
??reset_md_75:
        MOVW      AX, DE             ;; 1 cycle
        PUSH      HL                 ;; 1 cycle
          CFI CFA SP+20
        POP       BC                 ;; 1 cycle
          CFI CFA SP+18
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x6541        ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??cal_present_1:
        BC        ??reset_md_77      ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
        MOV       A, [SP+0x08]       ;; 1 cycle
        CMP       A, #0x1            ;; 1 cycle
        BNZ       ??reset_md_77      ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  563   {
//  564     temp = 0;
??reset_md_76:
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+20
        POP       HL                 ;; 1 cycle
          CFI CFA SP+18
        ; ------------------------------------- Block: 5 cycles
//  565   }   
//  566   return(temp);
??reset_md_77:
        MOVW      AX, DE             ;; 1 cycle
        PUSH      HL                 ;; 1 cycle
          CFI CFA SP+20
        POP       BC                 ;; 1 cycle
          CFI CFA SP+18
        ADDW      SP, #0xE           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock3
        ; ------------------------------------- Block: 10 cycles
        ; ------------------------------------- Total: 338 cycles
//  567 }
//  568 
//  569 /* Called every minute */

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock4 Using cfiCommon0
          CFI Function _cal_rising_demand
        CODE
//  570 void cal_rising_demand()
//  571 {
_cal_rising_demand:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 4
        SUBW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
//  572   us16 local_address;
//  573   us8 temp = bcd_to_decimal(Now.min);
        MOV       A, N:_Now+1        ;; 1 cycle
          CFI FunCall _bcd_to_decimal
        CALL      _bcd_to_decimal    ;; 3 cycles
//  574   
//  575   /* subtracting one */
//  576   if(temp == 0)
        CMP0      A                  ;; 1 cycle
        BNZ       ??reset_md_78      ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
//  577   {
//  578     temp = 59;
        MOV       X, #0x3B           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       [SP+0x02], A       ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        BR        S:??reset_md_79    ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
//  579   }
//  580   else
//  581   {
//  582     temp--;
??reset_md_78:
        DEC       A                  ;; 1 cycle
        MOV       [SP+0x02], A       ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
//  583   }
//  584   
//  585   if(md_type == 1)      
??reset_md_79:
        CMP       N:_md_type, #0x1   ;; 1 cycle
        BNZ       ??reset_md_80      ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  586   {
//  587     local_address = 0x0800 + ((temp % mdi_interval_time)/mdi_period)*16;
        MOV       X, N:_mdi_interval_time  ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
          CFI FunCall ?UC_MOD_L01
        CALL      N:?UC_MOD_L01      ;; 3 cycles
        MOV       B, A               ;; 1 cycle
        MOV       A, B               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        MOV       X, N:_mdi_period   ;; 1 cycle
        MOV       A, B               ;; 1 cycle
          CFI FunCall ?UC_DIV_L01
        CALL      N:?UC_DIV_L01      ;; 3 cycles
        MOV       C, A               ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOVW      AX, #0x10          ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #0x800         ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
//  588     
//  589     /* incresing one page */
//  590     if(local_address == (0x0800+((mdi_no_of_period-1)*16)))
        MOV       X, N:_mdi_no_of_period  ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x10          ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #0x7F0         ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BNZ       ??reset_md_81      ;; 4 cycles
        ; ------------------------------------- Block: 31 cycles
//  591     {
//  592       local_address = 0x0800;
        MOVW      AX, #0x800         ;; 1 cycle
        MOVW      [SP], AX           ;; 1 cycle
        BR        S:??reset_md_82    ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  593     }
//  594     else
//  595     {
//  596       local_address += 0x10;
??reset_md_81:
        MOVW      AX, HL             ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
        MOVW      [SP], AX           ;; 1 cycle
        BR        S:??reset_md_82    ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
//  597     }
//  598   }
//  599   else
//  600   {
//  601     local_address = 0x0980;
??reset_md_80:
        MOVW      AX, #0x980         ;; 1 cycle
        MOVW      [SP], AX           ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
//  602   }
//  603   
//  604   if(eprom_read(local_address,0,PAGE_1,AUTO_CALC) == EEP_OK)
??reset_md_82:
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
        CMP       A, #0x1            ;; 1 cycle
        BNZ       ??reset_md_83      ;; 4 cycles
        ; ------------------------------------- Block: 12 cycles
//  605   {
//  606     demand.Allph.act_imp.rising.last_energy_value = char_array_to_long4(&opr_data[0]);
        MOVW      AX, #LWRD(_opr_data)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_demand+120, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+122, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  607     demand.Allph.act_exp.rising.last_energy_value = char_array_to_long4(&opr_data[4]);
        MOVW      AX, #LWRD(_opr_data+4)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_demand+144, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+146, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  608     demand.Allph.app_imp.rising.last_energy_value = char_array_to_long4(&opr_data[8]);
        MOVW      AX, #LWRD(_opr_data+8)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_demand+168, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+170, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        BR        S:??reset_md_84    ;; 3 cycles
        ; ------------------------------------- Block: 27 cycles
//  609   }
//  610   else
//  611   {
//  612     demand.Allph.act_imp.rising.last_energy_value = energy.Allph.active_imp;
??reset_md_83:
        MOVW      BC, N:_energy+54   ;; 1 cycle
        MOVW      AX, N:_energy+52   ;; 1 cycle
        MOVW      N:_demand+120, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+122, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  613     demand.Allph.act_exp.rising.last_energy_value = energy.Allph.active_exp;
        MOVW      BC, N:_energy+58   ;; 1 cycle
        MOVW      AX, N:_energy+56   ;; 1 cycle
        MOVW      N:_demand+144, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+146, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  614     demand.Allph.app_imp.rising.last_energy_value = energy.Allph.apparent_imp;
        MOVW      BC, N:_energy+70   ;; 1 cycle
        MOVW      AX, N:_energy+68   ;; 1 cycle
        MOVW      N:_demand+168, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+170, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ; ------------------------------------- Block: 18 cycles
//  615   }
//  616   
//  617   if(md_type==1)
??reset_md_84:
        CMP       N:_md_type, #0x1   ;; 1 cycle
        BNZ       ??reset_md_85      ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  618   {
//  619     local_address += 0x00c0;		
        MOVW      AX, [SP]           ;; 1 cycle
        ADDW      AX, #0xC0          ;; 1 cycle
        MOVW      [SP], AX           ;; 1 cycle
        BR        S:??reset_md_86    ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
//  620   }
//  621   else
//  622   {
//  623     local_address += 0x0010;		
??reset_md_85:
        MOVW      AX, [SP]           ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
        MOVW      [SP], AX           ;; 1 cycle
        ; ------------------------------------- Block: 3 cycles
//  624   }
//  625   if(eprom_read(local_address,0,PAGE_1,AUTO_CALC) == EEP_OK)
??reset_md_86:
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
        CMP       A, #0x1            ;; 1 cycle
        BNZ       ??reset_md_87      ;; 4 cycles
        ; ------------------------------------- Block: 12 cycles
//  626   {
//  627     demand.Allph.app_exp.rising.last_energy_value = char_array_to_long4(&opr_data[0]);
        MOVW      AX, #LWRD(_opr_data)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_demand+192, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+194, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        BR        S:??reset_md_88    ;; 3 cycles
        ; ------------------------------------- Block: 11 cycles
//  628   }
//  629   else
//  630   {
//  631     demand.Allph.app_exp.rising.last_energy_value = energy.Allph.apparent_exp;
??reset_md_87:
        MOVW      BC, N:_energy+74   ;; 1 cycle
        MOVW      AX, N:_energy+72   ;; 1 cycle
        MOVW      N:_demand+192, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+194, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
//  632   }
//  633   
//  634   /* Phasewise Calculation */
//  635   local_address = 0x0360;
??reset_md_88:
        MOVW      AX, #0x360         ;; 1 cycle
        MOVW      [SP], AX           ;; 1 cycle
//  636   if(eprom_read(local_address,0,PAGE_1,AUTO_CALC) == EEP_OK)
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
        CMP       A, #0x1            ;; 1 cycle
        BNZ       ??reset_md_89      ;; 4 cycles
        ; ------------------------------------- Block: 14 cycles
//  637   {
//  638     demand.Rph.act_imp.rising.last_energy_value = char_array_to_long4(&opr_data[0]);
        MOVW      AX, #LWRD(_opr_data)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_demand, AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+2, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  639     demand.Yph.act_imp.rising.last_energy_value = char_array_to_long4(&opr_data[4]);
        MOVW      AX, #LWRD(_opr_data+4)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_demand+40, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+42, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  640     demand.Bph.act_imp.rising.last_energy_value = char_array_to_long4(&opr_data[8]);
        MOVW      AX, #LWRD(_opr_data+8)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_demand+80, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+82, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        BR        S:??reset_md_90    ;; 3 cycles
        ; ------------------------------------- Block: 27 cycles
//  641   }
//  642   else
//  643   {
//  644     demand.Rph.act_imp.rising.last_energy_value = energy.Rph.active_imp;
??reset_md_89:
        MOVW      BC, N:_energy+4    ;; 1 cycle
        MOVW      AX, N:_energy+2    ;; 1 cycle
        MOVW      N:_demand, AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+2, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  645     demand.Yph.act_imp.rising.last_energy_value = energy.Yph.active_imp;
        MOVW      BC, N:_energy+18   ;; 1 cycle
        MOVW      AX, N:_energy+16   ;; 1 cycle
        MOVW      N:_demand+40, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+42, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  646     demand.Bph.act_imp.rising.last_energy_value = energy.Bph.active_imp;
        MOVW      BC, N:_energy+32   ;; 1 cycle
        MOVW      AX, N:_energy+30   ;; 1 cycle
        MOVW      N:_demand+80, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+82, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ; ------------------------------------- Block: 18 cycles
//  647   }
//  648   local_address += 0x10;
??reset_md_90:
        MOVW      AX, [SP]           ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
        MOVW      [SP], AX           ;; 1 cycle
//  649   if(eprom_read(local_address,0,PAGE_1,AUTO_CALC) == EEP_OK)
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
        CMP       A, #0x1            ;; 1 cycle
        BNZ       ??reset_md_91      ;; 4 cycles
        ; ------------------------------------- Block: 15 cycles
//  650   {
//  651     demand.Rph.act_exp.rising.last_energy_value = char_array_to_long4(&opr_data[0]);
        MOVW      AX, #LWRD(_opr_data)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_demand+20, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+22, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  652     demand.Yph.act_exp.rising.last_energy_value = char_array_to_long4(&opr_data[4]);
        MOVW      AX, #LWRD(_opr_data+4)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_demand+60, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+62, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  653     demand.Bph.act_exp.rising.last_energy_value = char_array_to_long4(&opr_data[8]);
        MOVW      AX, #LWRD(_opr_data+8)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_demand+100, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+102, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        BR        S:??reset_md_92    ;; 3 cycles
        ; ------------------------------------- Block: 27 cycles
//  654   }
//  655   else
//  656   {
//  657     demand.Rph.act_exp.rising.last_energy_value = energy.Rph.active_exp;
??reset_md_91:
        MOVW      BC, N:_energy+8    ;; 1 cycle
        MOVW      AX, N:_energy+6    ;; 1 cycle
        MOVW      N:_demand+20, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+22, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  658     demand.Yph.act_exp.rising.last_energy_value = energy.Yph.active_exp;
        MOVW      BC, N:_energy+22   ;; 1 cycle
        MOVW      AX, N:_energy+20   ;; 1 cycle
        MOVW      N:_demand+60, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+62, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  659     demand.Bph.act_exp.rising.last_energy_value = energy.Bph.active_exp;
        MOVW      BC, N:_energy+36   ;; 1 cycle
        MOVW      AX, N:_energy+34   ;; 1 cycle
        MOVW      N:_demand+100, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+102, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ; ------------------------------------- Block: 18 cycles
//  660   }
//  661   
//  662   /* Defraud energy */
//  663   if(eprom_read(0x09C0,0,PAGE_1,AUTO_CALC) == EEP_OK)
??reset_md_92:
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x9C0         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
        CMP       A, #0x1            ;; 1 cycle
        BNZ       ??reset_md_93      ;; 4 cycles
        ; ------------------------------------- Block: 12 cycles
//  664   {
//  665     last_energy_mag = char_array_to_long4(&opr_data[0]);
        MOVW      AX, #LWRD(_opr_data)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_last_energy_mag, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_last_energy_mag+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        BR        S:??reset_md_94    ;; 3 cycles
        ; ------------------------------------- Block: 11 cycles
//  666   }
//  667   else
//  668   {
//  669     last_energy_mag = energy.Allph.defraud_mag;
??reset_md_93:
        MOVW      BC, N:_energy+62   ;; 1 cycle
        MOVW      AX, N:_energy+60   ;; 1 cycle
        MOVW      N:_last_energy_mag, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_last_energy_mag+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
//  670   }
//  671   if(eprom_read(0x09D0,0,PAGE_1,AUTO_CALC) == EEP_OK)
??reset_md_94:
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x9D0         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
        CMP       A, #0x1            ;; 1 cycle
        BNZ       ??reset_md_95      ;; 4 cycles
        ; ------------------------------------- Block: 12 cycles
//  672   {
//  673     last_energy_mag_Rph = char_array_to_long4(&opr_data[0]);
        MOVW      AX, #LWRD(_opr_data)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_last_energy_mag_Rph, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_last_energy_mag_Rph+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  674     last_energy_mag_Yph = char_array_to_long4(&opr_data[4]);
        MOVW      AX, #LWRD(_opr_data+4)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_last_energy_mag_Yph, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_last_energy_mag_Yph+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  675     last_energy_mag_Bph = char_array_to_long4(&opr_data[8]);
        MOVW      AX, #LWRD(_opr_data+8)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_last_energy_mag_Bph, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_last_energy_mag_Bph+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        BR        S:??reset_md_96    ;; 3 cycles
        ; ------------------------------------- Block: 27 cycles
//  676   }
//  677   else
//  678   {
//  679     last_energy_mag_Rph = energy.Rph.defraud_mag;
??reset_md_95:
        MOVW      BC, N:_energy+12   ;; 1 cycle
        MOVW      AX, N:_energy+10   ;; 1 cycle
        MOVW      N:_last_energy_mag_Rph, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_last_energy_mag_Rph+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  680     last_energy_mag_Yph = energy.Yph.defraud_mag;
        MOVW      BC, N:_energy+26   ;; 1 cycle
        MOVW      AX, N:_energy+24   ;; 1 cycle
        MOVW      N:_last_energy_mag_Yph, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_last_energy_mag_Yph+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  681     last_energy_mag_Bph = energy.Bph.defraud_mag;
        MOVW      BC, N:_energy+40   ;; 1 cycle
        MOVW      AX, N:_energy+38   ;; 1 cycle
        MOVW      N:_last_energy_mag_Bph, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_last_energy_mag_Bph+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ; ------------------------------------- Block: 18 cycles
//  682   }
//  683   /* Calculation */
//  684   demand.Allph.act_imp.rising.value = cal_rising(energy.Allph.active_imp,demand.Allph.act_imp.rising.last_energy_value,0,energy.Allph.defraud_mag,last_energy_mag,1);
??reset_md_96:
        MOVW      AX, N:_last_energy_mag+2  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        MOVW      AX, N:_last_energy_mag  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      AX, N:_energy+62   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+14
        MOVW      AX, N:_energy+60   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+16
        MOVW      AX, N:_demand+122  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+18
        MOVW      AX, N:_demand+120  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+20
        MOV       D, #0x1            ;; 1 cycle
        MOV       E, #0x0            ;; 1 cycle
        MOVW      BC, N:_energy+54   ;; 1 cycle
        MOVW      AX, N:_energy+52   ;; 1 cycle
          CFI FunCall _cal_rising
        CALL      _cal_rising        ;; 3 cycles
        MOVW      N:_demand+124, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+126, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  685   demand.Allph.act_exp.rising.value = cal_rising(energy.Allph.active_exp,demand.Allph.act_exp.rising.last_energy_value,0,energy.Allph.defraud_mag,last_energy_mag,0);
        MOVW      AX, N:_last_energy_mag+2  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+22
        MOVW      AX, N:_last_energy_mag  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+24
        MOVW      AX, N:_energy+62   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+26
        MOVW      AX, N:_energy+60   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+28
        MOVW      AX, N:_demand+146  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+30
        MOVW      AX, N:_demand+144  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
        MOV       D, #0x0            ;; 1 cycle
        MOV       E, #0x0            ;; 1 cycle
        MOVW      BC, N:_energy+58   ;; 1 cycle
        MOVW      AX, N:_energy+56   ;; 1 cycle
          CFI FunCall _cal_rising
        CALL      _cal_rising        ;; 3 cycles
        MOVW      N:_demand+148, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+150, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  686   demand.Allph.app_imp.rising.value = cal_rising(energy.Allph.apparent_imp,demand.Allph.app_imp.rising.last_energy_value,0,energy.Allph.defraud_mag,last_energy_mag,1);
        MOVW      AX, N:_last_energy_mag+2  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+34
        MOVW      AX, N:_last_energy_mag  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+36
        MOVW      AX, N:_energy+62   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+38
        MOVW      AX, N:_energy+60   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+40
        MOVW      AX, N:_demand+170  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+42
        MOVW      AX, N:_demand+168  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+44
        MOV       D, #0x1            ;; 1 cycle
        MOV       E, #0x0            ;; 1 cycle
        MOVW      BC, N:_energy+70   ;; 1 cycle
        MOVW      AX, N:_energy+68   ;; 1 cycle
          CFI FunCall _cal_rising
        CALL      _cal_rising        ;; 3 cycles
        ADDW      SP, #0x24          ;; 1 cycle
          CFI CFA SP+8
        MOVW      N:_demand+172, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+174, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  687   demand.Allph.app_exp.rising.value = cal_rising(energy.Allph.apparent_exp,demand.Allph.app_imp.rising.last_energy_value,0,energy.Allph.defraud_mag,last_energy_mag,0);
        MOVW      AX, N:_last_energy_mag+2  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        MOVW      AX, N:_last_energy_mag  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      AX, N:_energy+62   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+14
        MOVW      AX, N:_energy+60   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+16
        MOVW      AX, N:_demand+170  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+18
        MOVW      AX, N:_demand+168  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+20
        MOV       D, #0x0            ;; 1 cycle
        MOV       E, #0x0            ;; 1 cycle
        MOVW      BC, N:_energy+74   ;; 1 cycle
        MOVW      AX, N:_energy+72   ;; 1 cycle
          CFI FunCall _cal_rising
        CALL      _cal_rising        ;; 3 cycles
        MOVW      N:_demand+196, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+198, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  688   
//  689   demand.Rph.act_imp.rising.value = cal_rising(energy.Rph.active_imp,demand.Rph.act_imp.rising.last_energy_value,1,energy.Rph.defraud_mag,last_energy_mag_Rph,1);
        MOVW      AX, N:_last_energy_mag_Rph+2  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+22
        MOVW      AX, N:_last_energy_mag_Rph  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+24
        MOVW      AX, N:_energy+12   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+26
        MOVW      AX, N:_energy+10   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+28
        MOVW      AX, N:_demand+2    ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+30
        MOVW      AX, N:_demand      ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
        MOV       D, #0x1            ;; 1 cycle
        MOV       E, #0x1            ;; 1 cycle
        MOVW      BC, N:_energy+4    ;; 1 cycle
        MOVW      AX, N:_energy+2    ;; 1 cycle
          CFI FunCall _cal_rising
        CALL      _cal_rising        ;; 3 cycles
        MOVW      N:_demand+4, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+6, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  690   demand.Rph.act_exp.rising.value = cal_rising(energy.Rph.active_exp,demand.Rph.act_exp.rising.last_energy_value,1,energy.Rph.defraud_mag,last_energy_mag_Rph,0);
        MOVW      AX, N:_last_energy_mag_Rph+2  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+34
        MOVW      AX, N:_last_energy_mag_Rph  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+36
        MOVW      AX, N:_energy+12   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+38
        MOVW      AX, N:_energy+10   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+40
        MOVW      AX, N:_demand+22   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+42
        MOVW      AX, N:_demand+20   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+44
        MOV       D, #0x0            ;; 1 cycle
        MOV       E, #0x1            ;; 1 cycle
        MOVW      BC, N:_energy+8    ;; 1 cycle
        MOVW      AX, N:_energy+6    ;; 1 cycle
          CFI FunCall _cal_rising
        CALL      _cal_rising        ;; 3 cycles
        ADDW      SP, #0x24          ;; 1 cycle
          CFI CFA SP+8
        MOVW      N:_demand+24, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+26, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  691   demand.Yph.act_imp.rising.value = cal_rising(energy.Yph.active_imp,demand.Yph.act_imp.rising.last_energy_value,1,energy.Yph.defraud_mag,last_energy_mag_Yph,1);
        MOVW      AX, N:_last_energy_mag_Yph+2  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        MOVW      AX, N:_last_energy_mag_Yph  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      AX, N:_energy+26   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+14
        MOVW      AX, N:_energy+24   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+16
        MOVW      AX, N:_demand+42   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+18
        MOVW      AX, N:_demand+40   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+20
        MOV       D, #0x1            ;; 1 cycle
        MOV       E, #0x1            ;; 1 cycle
        MOVW      BC, N:_energy+18   ;; 1 cycle
        MOVW      AX, N:_energy+16   ;; 1 cycle
          CFI FunCall _cal_rising
        CALL      _cal_rising        ;; 3 cycles
        MOVW      N:_demand+44, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+46, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  692   demand.Yph.act_exp.rising.value = cal_rising(energy.Yph.active_exp,demand.Yph.act_exp.rising.last_energy_value,1,energy.Yph.defraud_mag,last_energy_mag_Yph,0);
        MOVW      AX, N:_last_energy_mag_Yph+2  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+22
        MOVW      AX, N:_last_energy_mag_Yph  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+24
        MOVW      AX, N:_energy+26   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+26
        MOVW      AX, N:_energy+24   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+28
        MOVW      AX, N:_demand+62   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+30
        MOVW      AX, N:_demand+60   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
        MOV       D, #0x0            ;; 1 cycle
        MOV       E, #0x1            ;; 1 cycle
        MOVW      BC, N:_energy+22   ;; 1 cycle
        MOVW      AX, N:_energy+20   ;; 1 cycle
          CFI FunCall _cal_rising
        CALL      _cal_rising        ;; 3 cycles
        MOVW      N:_demand+64, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+66, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  693   demand.Bph.act_imp.rising.value = cal_rising(energy.Bph.active_imp,demand.Bph.act_imp.rising.last_energy_value,1,energy.Bph.defraud_mag,last_energy_mag_Bph,1);
        MOVW      AX, N:_last_energy_mag_Bph+2  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+34
        MOVW      AX, N:_last_energy_mag_Bph  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+36
        MOVW      AX, N:_energy+40   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+38
        MOVW      AX, N:_energy+38   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+40
        MOVW      AX, N:_demand+82   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+42
        MOVW      AX, N:_demand+80   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+44
        MOV       D, #0x1            ;; 1 cycle
        MOV       E, #0x1            ;; 1 cycle
        MOVW      BC, N:_energy+32   ;; 1 cycle
        MOVW      AX, N:_energy+30   ;; 1 cycle
          CFI FunCall _cal_rising
        CALL      _cal_rising        ;; 3 cycles
        ADDW      SP, #0x24          ;; 1 cycle
          CFI CFA SP+8
        MOVW      N:_demand+84, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+86, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  694   demand.Bph.act_exp.rising.value = cal_rising(energy.Bph.active_exp,demand.Bph.act_exp.rising.last_energy_value,1,energy.Bph.defraud_mag,last_energy_mag_Bph,0);
        MOVW      AX, N:_last_energy_mag_Bph+2  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        MOVW      AX, N:_last_energy_mag_Bph  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      AX, N:_energy+40   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+14
        MOVW      AX, N:_energy+38   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+16
        MOVW      AX, N:_demand+102  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+18
        MOVW      AX, N:_demand+100  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+20
        MOV       D, #0x0            ;; 1 cycle
        MOV       E, #0x1            ;; 1 cycle
        MOVW      BC, N:_energy+36   ;; 1 cycle
        MOVW      AX, N:_energy+34   ;; 1 cycle
          CFI FunCall _cal_rising
        CALL      _cal_rising        ;; 3 cycles
        MOVW      N:_demand+104, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+106, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  695   
//  696   /* saving into energy */
//  697   /* pending, the time being save in the following routine should not be the current time. it should be the time when 
//  698   the mdi_interval_time is started such as 6.30, 7.00 etc */
//  699   long_into_char_array4(demand.Allph.act_imp.rising.value, &opr_data[0]);
        MOVW      DE, #LWRD(_opr_data)  ;; 1 cycle
        MOVW      BC, N:_demand+126  ;; 1 cycle
        MOVW      AX, N:_demand+124  ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  700   time_into_char_array4(Now,&opr_data[3]);
        MOVW      HL, #LWRD(_Now)    ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+28
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      AX, #LWRD(_opr_data+3)  ;; 1 cycle
          CFI FunCall _time_into_char_array4
        CALL      _time_into_char_array4  ;; 3 cycles
//  701   long_into_char_array4(demand.Allph.act_exp.rising.value, &opr_data[7]);
        MOVW      DE, #LWRD(_opr_data+7)  ;; 1 cycle
        MOVW      BC, N:_demand+150  ;; 1 cycle
        MOVW      AX, N:_demand+148  ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  702   time_into_char_array4(Now,&opr_data[10]);
        MOVW      HL, #LWRD(_Now)    ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+36
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      AX, #LWRD(_opr_data+10)  ;; 1 cycle
          CFI FunCall _time_into_char_array4
        CALL      _time_into_char_array4  ;; 3 cycles
//  703   Eprom_WriteWM(0x09A0,0,16);
        MOV       B, #0x10           ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x9A0         ;; 1 cycle
          CFI FunCall _Eprom_WriteWM
        CALL      _Eprom_WriteWM     ;; 3 cycles
//  704   
//  705   long_into_char_array4(demand.Allph.app_imp.rising.value, &opr_data[0]);
        MOVW      DE, #LWRD(_opr_data)  ;; 1 cycle
        MOVW      BC, N:_demand+174  ;; 1 cycle
        MOVW      AX, N:_demand+172  ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  706   time_into_char_array4(Now,&opr_data[3]);
        MOVW      HL, #LWRD(_Now)    ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+44
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      AX, #LWRD(_opr_data+3)  ;; 1 cycle
          CFI FunCall _time_into_char_array4
        CALL      _time_into_char_array4  ;; 3 cycles
        ADDW      SP, #0x24          ;; 1 cycle
          CFI CFA SP+8
//  707   long_into_char_array4(demand.Allph.app_exp.rising.value, &opr_data[7]);
        MOVW      DE, #LWRD(_opr_data+7)  ;; 1 cycle
        MOVW      BC, N:_demand+198  ;; 1 cycle
        MOVW      AX, N:_demand+196  ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  708   time_into_char_array4(Now,&opr_data[10]);
        MOVW      HL, #LWRD(_Now)    ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+16
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      AX, #LWRD(_opr_data+10)  ;; 1 cycle
          CFI FunCall _time_into_char_array4
        CALL      _time_into_char_array4  ;; 3 cycles
//  709   Eprom_WriteWM(0x09B0,0,16);
        MOV       B, #0x10           ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x9B0         ;; 1 cycle
          CFI FunCall _Eprom_WriteWM
        CALL      _Eprom_WriteWM     ;; 3 cycles
//  710   
//  711   long_into_char_array4(demand.Rph.act_imp.rising.value, &opr_data[0]);
        MOVW      DE, #LWRD(_opr_data)  ;; 1 cycle
        MOVW      BC, N:_demand+6    ;; 1 cycle
        MOVW      AX, N:_demand+4    ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  712   time_into_char_array4(Now,&opr_data[3]);
        MOVW      HL, #LWRD(_Now)    ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+24
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      AX, #LWRD(_opr_data+3)  ;; 1 cycle
          CFI FunCall _time_into_char_array4
        CALL      _time_into_char_array4  ;; 3 cycles
//  713   long_into_char_array4(demand.Rph.act_exp.rising.value, &opr_data[7]);
        MOVW      DE, #LWRD(_opr_data+7)  ;; 1 cycle
        MOVW      BC, N:_demand+26   ;; 1 cycle
        MOVW      AX, N:_demand+24   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  714   time_into_char_array4(Now,&opr_data[10]);
        MOVW      HL, #LWRD(_Now)    ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+32
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      AX, #LWRD(_opr_data+10)  ;; 1 cycle
          CFI FunCall _time_into_char_array4
        CALL      _time_into_char_array4  ;; 3 cycles
//  715   Eprom_WriteWM(0x0330,0,16);
        MOV       B, #0x10           ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x330         ;; 1 cycle
          CFI FunCall _Eprom_WriteWM
        CALL      _Eprom_WriteWM     ;; 3 cycles
//  716   
//  717   long_into_char_array4(demand.Yph.act_imp.rising.value, &opr_data[0]);
        MOVW      DE, #LWRD(_opr_data)  ;; 1 cycle
        MOVW      BC, N:_demand+46   ;; 1 cycle
        MOVW      AX, N:_demand+44   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  718   time_into_char_array4(Now,&opr_data[3]);
        MOVW      HL, #LWRD(_Now)    ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+40
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      AX, #LWRD(_opr_data+3)  ;; 1 cycle
          CFI FunCall _time_into_char_array4
        CALL      _time_into_char_array4  ;; 3 cycles
//  719   long_into_char_array4(demand.Yph.act_exp.rising.value, &opr_data[7]);
        MOVW      DE, #LWRD(_opr_data+7)  ;; 1 cycle
        MOVW      BC, N:_demand+66   ;; 1 cycle
        MOVW      AX, N:_demand+64   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  720   time_into_char_array4(Now,&opr_data[10]);
        MOVW      HL, #LWRD(_Now)    ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+48
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      AX, #LWRD(_opr_data+10)  ;; 1 cycle
          CFI FunCall _time_into_char_array4
        CALL      _time_into_char_array4  ;; 3 cycles
        ADDW      SP, #0x28          ;; 1 cycle
          CFI CFA SP+8
//  721   Eprom_WriteWM(0x0340,0,16);
        MOV       B, #0x10           ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x340         ;; 1 cycle
          CFI FunCall _Eprom_WriteWM
        CALL      _Eprom_WriteWM     ;; 3 cycles
//  722   
//  723   long_into_char_array4(demand.Bph.act_imp.rising.value, &opr_data[0]);
        MOVW      DE, #LWRD(_opr_data)  ;; 1 cycle
        MOVW      BC, N:_demand+86   ;; 1 cycle
        MOVW      AX, N:_demand+84   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  724   time_into_char_array4(Now,&opr_data[3]);
        MOVW      HL, #LWRD(_Now)    ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+16
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      AX, #LWRD(_opr_data+3)  ;; 1 cycle
          CFI FunCall _time_into_char_array4
        CALL      _time_into_char_array4  ;; 3 cycles
//  725   long_into_char_array4(demand.Bph.act_exp.rising.value, &opr_data[7]);
        MOVW      DE, #LWRD(_opr_data+7)  ;; 1 cycle
        MOVW      BC, N:_demand+106  ;; 1 cycle
        MOVW      AX, N:_demand+104  ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  726   time_into_char_array4(Now,&opr_data[10]);
        MOVW      HL, #LWRD(_Now)    ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+24
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      AX, #LWRD(_opr_data+10)  ;; 1 cycle
          CFI FunCall _time_into_char_array4
        CALL      _time_into_char_array4  ;; 3 cycles
//  727   Eprom_WriteWM(0x0350,0,16);
        MOV       B, #0x10           ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x350         ;; 1 cycle
          CFI FunCall _Eprom_WriteWM
        CALL      _Eprom_WriteWM     ;; 3 cycles
//  728 }
        ADDW      SP, #0x14          ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock4
        ; ------------------------------------- Block: 442 cycles
        ; ------------------------------------- Total: 815 cycles
//  729 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock5 Using cfiCommon1
          CFI Function _cal_rising
        CODE
//  730 us32 cal_rising(us32 energy, us32 last_energy, us8 phasewise, us32 mag_energy, us32 last_mag_energy, us8 sub_mag_demand)
//  731 {
_cal_rising:
        ; * Stack frame (at entry) *
        ; Param size: 12
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+6
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+8
        PUSH      DE                 ;; 1 cycle
          CFI CFA SP+10
        ; Auto size: 14
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+18
//  732   us32 temp = 0, temp1 = 0;
        MOVW      AX, SP             ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
//  733   if(energy < last_energy )
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x12          ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [SP+0x0C]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x0A]      ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        CMP       A, [HL+0x03]       ;; 1 cycle
        BNZ       ??reset_md_97      ;; 4 cycles
        ; ------------------------------------- Block: 29 cycles
        MOV       A, C               ;; 1 cycle
        CMP       A, [HL+0x02]       ;; 1 cycle
        BNZ       ??reset_md_97      ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOV       A, B               ;; 1 cycle
        CMP       A, [HL+0x01]       ;; 1 cycle
        BNZ       ??reset_md_97      ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOV       A, X               ;; 1 cycle
        CMP       A, [HL]            ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
??reset_md_97:
        BNC       ??reset_md_98      ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
//  734   {
//  735     temp = (us32)(energy + (ROLL_OVER_LIMIT  - last_energy));
        MOVW      AX, [SP+0x14]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [SP+0x12]      ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, [SP+0x0C]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x0A]      ;; 1 cycle
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
        BR        S:??reset_md_99    ;; 3 cycles
        ; ------------------------------------- Block: 33 cycles
//  736   }
//  737   else
//  738   {
//  739     temp = (us32)(energy - last_energy);
??reset_md_98:
        MOVW      AX, [SP+0x14]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [SP+0x12]      ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, [SP+0x0C]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x0A]      ;; 1 cycle
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
//  740   }
//  741   
//  742   if(sub_mag_demand == 1)
??reset_md_99:
        MOV       A, [SP+0x09]       ;; 1 cycle
        CMP       A, #0x1            ;; 1 cycle
        SKZ                          ;; 4 cycles
        BR        N:??reset_md_100   ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  743   {
//  744     if(mag_energy < last_mag_energy )
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x1A          ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [SP+0x18]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x16]      ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        CMP       A, [HL+0x03]       ;; 1 cycle
        BNZ       ??reset_md_101     ;; 4 cycles
        ; ------------------------------------- Block: 12 cycles
        MOV       A, C               ;; 1 cycle
        CMP       A, [HL+0x02]       ;; 1 cycle
        BNZ       ??reset_md_101     ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOV       A, B               ;; 1 cycle
        CMP       A, [HL+0x01]       ;; 1 cycle
        BNZ       ??reset_md_101     ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOV       A, X               ;; 1 cycle
        CMP       A, [HL]            ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
??reset_md_101:
        BNC       ??reset_md_102     ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
//  745     {
//  746       temp1 = (us32)(mag_energy + (ROLL_OVER_LIMIT  - last_mag_energy));
        MOVW      AX, [SP+0x1C]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [SP+0x1A]      ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, [SP+0x18]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x16]      ;; 1 cycle
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
        XCHW      AX, HL             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        BR        S:??reset_md_103   ;; 3 cycles
        ; ------------------------------------- Block: 36 cycles
//  747     }
//  748     else
//  749     {
//  750       temp1 = (us32)(mag_energy - last_mag_energy);
??reset_md_102:
        MOVW      AX, [SP+0x1C]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [SP+0x1A]      ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, [SP+0x18]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x16]      ;; 1 cycle
        SUBW      AX, DE             ;; 1 cycle
        SKNC
        DECW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        SUBW      AX, HL             ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ; ------------------------------------- Block: 24 cycles
//  751     }
//  752     if(temp >= temp1)
??reset_md_103:
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        CMP       A, [HL+0x03]       ;; 1 cycle
        BNZ       ??reset_md_104     ;; 4 cycles
        ; ------------------------------------- Block: 12 cycles
        MOV       A, C               ;; 1 cycle
        CMP       A, [HL+0x02]       ;; 1 cycle
        BNZ       ??reset_md_104     ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOV       A, B               ;; 1 cycle
        CMP       A, [HL+0x01]       ;; 1 cycle
        BNZ       ??reset_md_104     ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOV       A, X               ;; 1 cycle
        CMP       A, [HL]            ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
??reset_md_104:
        BC        ??reset_md_105     ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
//  753     {
//  754       temp = temp - temp1;
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
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
        BR        S:??reset_md_100   ;; 3 cycles
        ; ------------------------------------- Block: 24 cycles
//  755     }
//  756     else
//  757     {
//  758       temp = temp1 - temp;
??reset_md_105:
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
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
//  759     }
//  760   }
//  761   
//  762   temp = temp * (60 / mdi_interval_time); 
??reset_md_100:
        MOV       X, N:_mdi_interval_time  ;; 1 cycle
        MOV       A, #0x3C           ;; 1 cycle
          CFI FunCall ?UC_DIV_L01
        CALL      N:?UC_DIV_L01      ;; 3 cycles
        MOV       C, A               ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOVW      AX, BC             ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+20
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+22
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
          CFI FunCall ?L_MUL_FAST_L03
        CALL      N:?L_MUL_FAST_L03  ;; 3 cycles
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+24
        POP       HL                 ;; 1 cycle
          CFI CFA SP+22
        MOVW      DE, AX             ;; 1 cycle
//  763   
//  764   if((temp > ((us32)MD_LIMIT) && phasewise == 0) || (temp > ((us32)MD_LIMIT_PHASE) && phasewise == 1))
        MOVW      AX, DE             ;; 1 cycle
        PUSH      HL                 ;; 1 cycle
          CFI CFA SP+24
        POP       BC                 ;; 1 cycle
          CFI CFA SP+22
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+18
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x1           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 28 cycles
        CMPW      AX, #0x2FC1        ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??cal_rising_0:
        BC        ??reset_md_106     ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
        MOV       A, [SP+0x08]       ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        BZ        ??reset_md_107     ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
??reset_md_106:
        MOVW      AX, DE             ;; 1 cycle
        PUSH      HL                 ;; 1 cycle
          CFI CFA SP+20
        POP       BC                 ;; 1 cycle
          CFI CFA SP+18
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x6541        ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??cal_rising_1:
        BC        ??reset_md_108     ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
        MOV       A, [SP+0x08]       ;; 1 cycle
        CMP       A, #0x1            ;; 1 cycle
        BNZ       ??reset_md_108     ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  765   {
//  766     temp = 0;
??reset_md_107:
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+20
        POP       HL                 ;; 1 cycle
          CFI CFA SP+18
        ; ------------------------------------- Block: 5 cycles
//  767   }    
//  768   return(temp);
??reset_md_108:
        MOVW      AX, DE             ;; 1 cycle
        PUSH      HL                 ;; 1 cycle
          CFI CFA SP+20
        POP       BC                 ;; 1 cycle
          CFI CFA SP+18
        ADDW      SP, #0xE           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock5
        ; ------------------------------------- Block: 10 cycles
        ; ------------------------------------- Total: 344 cycles
//  769 }

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock6 Using cfiCommon0
          CFI Function _reset_mdreset_ip_flag
        CODE
//  770 void reset_mdreset_ip_flag()
//  771 {
_reset_mdreset_ip_flag:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  772   md_reset_ip_flag=0;
        MOV       N:_md_reset_ip_flag, #0x0  ;; 1 cycle
//  773    eprom_read(0x07D0,0,PAGE_1,AUTO_CALC);
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x7D0         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
//  774   opr_data[11]= md_reset_ip_flag;
        MOV       A, N:_md_reset_ip_flag  ;; 1 cycle
        MOV       N:_opr_data+11, A  ;; 1 cycle
//  775   eprom_write(0x07D0,0,16,PAGE_1,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+6
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x7D0         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  776 }
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock6
        ; ------------------------------------- Block: 26 cycles
        ; ------------------------------------- Total: 26 cycles
//  777 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock7 Using cfiCommon2
          CFI Function _md_next_interval_timestamp
        CODE
//  778 rtc_counter_value_t md_next_interval_timestamp(us8 mdi_period1, rtc_counter_value_t Time)
//  779 {
_md_next_interval_timestamp:
        ; * Stack frame (at entry) *
        ; Param size: 8
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+6
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+8
        ; Auto size: 6
        SUBW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+10
//  780   us8 temp_time;
//  781   
//  782   temp_time = bcd_to_decimal(Time.min) + mdi_period1;
        MOV       A, [SP+0x0B]       ;; 1 cycle
          CFI FunCall _bcd_to_decimal
        CALL      _bcd_to_decimal    ;; 3 cycles
        MOV       X, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        ADD       A, X               ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  783   /* if not the last integration time in 60 min slot( no need to increase hr).No need to change hr */
//  784   if(temp_time<60)                      
        MOV       A, [SP]            ;; 1 cycle
        CMP       A, #0x3C           ;; 1 cycle
        BNC       ??reset_md_109     ;; 4 cycles
        ; ------------------------------------- Block: 17 cycles
//  785   {
//  786     /* getting the next mdi period directly */
//  787     temp_time = temp_time - (temp_time % mdi_period1); 
        MOV       A, [SP+0x02]       ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
          CFI FunCall ?UC_MOD_L01
        CALL      N:?UC_MOD_L01      ;; 3 cycles
        MOV       B, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SUB       A, B               ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  788     Time.min = decimal_to_bcd(temp_time);
        MOV       A, [SP]            ;; 1 cycle
          CFI FunCall _decimal_to_bcd
        CALL      _decimal_to_bcd    ;; 3 cycles
        MOV       [SP+0x0B], A       ;; 1 cycle
        BR        S:??reset_md_110   ;; 3 cycles
        ; ------------------------------------- Block: 18 cycles
//  789   }
//  790   else
//  791   {
//  792     /* increasing the hr and setting min to zero */
//  793     /* taking care of time change effects due to increase in hr */
//  794     Time.min = 0;
??reset_md_109:
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP+0x0B], A       ;; 1 cycle
//  795     Time = time_inc_hour(Time);
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
??reset_md_110:
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
          CFI EndBlock cfiBlock7
        ; ------------------------------------- Block: 27 cycles
        ; ------------------------------------- Total: 79 cycles
//  796   }
//  797   return Time;
//  798 }

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock8 Using cfiCommon0
          CFI Function _md_ram_init
        CODE
//  799 void md_ram_init()
//  800 {
_md_ram_init:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  801     if(eprom_read(0x03E0,0,PAGE_1,AUTO_CALC) == EEP_OK)
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x3E0         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
        CMP       A, #0x1            ;; 1 cycle
        BNZ       ??reset_md_111     ;; 4 cycles
        ; ------------------------------------- Block: 12 cycles
//  802     {
//  803         demand.Allph.act_imp.max.value = char_array_to_long3(&opr_data[0]);
        MOVW      AX, #LWRD(_opr_data)  ;; 1 cycle
          CFI FunCall _char_array_to_long3
        CALL      _char_array_to_long3  ;; 3 cycles
        MOVW      N:_demand+140, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+142, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  804         demand.Allph.act_exp.max.value = char_array_to_long3(&opr_data[7]);
        MOVW      AX, #LWRD(_opr_data+7)  ;; 1 cycle
          CFI FunCall _char_array_to_long3
        CALL      _char_array_to_long3  ;; 3 cycles
        MOVW      N:_demand+164, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+166, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ; ------------------------------------- Block: 16 cycles
//  805     }
//  806     if(eprom_read(0x03F0,0,PAGE_1,AUTO_CALC) == EEP_OK)
??reset_md_111:
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x3F0         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
        CMP       A, #0x1            ;; 1 cycle
        BNZ       ??reset_md_112     ;; 4 cycles
        ; ------------------------------------- Block: 12 cycles
//  807     {
//  808         demand.Allph.app_imp.max.value = char_array_to_long3(&opr_data[0]);
        MOVW      AX, #LWRD(_opr_data)  ;; 1 cycle
          CFI FunCall _char_array_to_long3
        CALL      _char_array_to_long3  ;; 3 cycles
        MOVW      N:_demand+188, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+190, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  809         demand.Allph.app_exp.max.value = char_array_to_long3(&opr_data[7]);
        MOVW      AX, #LWRD(_opr_data+7)  ;; 1 cycle
          CFI FunCall _char_array_to_long3
        CALL      _char_array_to_long3  ;; 3 cycles
        MOVW      N:_demand+212, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+214, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ; ------------------------------------- Block: 16 cycles
//  810     }
//  811     if(eprom_read(0x0380,0,PAGE_1,AUTO_CALC) == EEP_OK)
??reset_md_112:
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x380         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
        CMP       A, #0x1            ;; 1 cycle
        BNZ       ??reset_md_113     ;; 4 cycles
        ; ------------------------------------- Block: 12 cycles
//  812     {
//  813         demand.Rph.act_imp.max.value = char_array_to_long3(&opr_data[0]);
        MOVW      AX, #LWRD(_opr_data)  ;; 1 cycle
          CFI FunCall _char_array_to_long3
        CALL      _char_array_to_long3  ;; 3 cycles
        MOVW      N:_demand+16, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+18, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  814         demand.Rph.act_exp.max.value = char_array_to_long3(&opr_data[7]);
        MOVW      AX, #LWRD(_opr_data+7)  ;; 1 cycle
          CFI FunCall _char_array_to_long3
        CALL      _char_array_to_long3  ;; 3 cycles
        MOVW      N:_demand+36, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+38, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ; ------------------------------------- Block: 16 cycles
//  815     }
//  816     if(eprom_read(0x0390,0,PAGE_1,AUTO_CALC) == EEP_OK)
??reset_md_113:
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x390         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
        CMP       A, #0x1            ;; 1 cycle
        BNZ       ??reset_md_114     ;; 4 cycles
        ; ------------------------------------- Block: 12 cycles
//  817     {
//  818         demand.Yph.act_imp.max.value = char_array_to_long3(&opr_data[0]);
        MOVW      AX, #LWRD(_opr_data)  ;; 1 cycle
          CFI FunCall _char_array_to_long3
        CALL      _char_array_to_long3  ;; 3 cycles
        MOVW      N:_demand+56, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+58, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  819         demand.Yph.act_exp.max.value = char_array_to_long3(&opr_data[7]);
        MOVW      AX, #LWRD(_opr_data+7)  ;; 1 cycle
          CFI FunCall _char_array_to_long3
        CALL      _char_array_to_long3  ;; 3 cycles
        MOVW      N:_demand+76, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+78, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ; ------------------------------------- Block: 16 cycles
//  820     }
//  821     if(eprom_read(0x03A0,0,PAGE_1,AUTO_CALC) == EEP_OK)
??reset_md_114:
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x3A0         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
        CMP       A, #0x1            ;; 1 cycle
        BNZ       ??reset_md_115     ;; 4 cycles
        ; ------------------------------------- Block: 12 cycles
//  822     {
//  823         demand.Bph.act_imp.max.value = char_array_to_long3(&opr_data[0]);
        MOVW      AX, #LWRD(_opr_data)  ;; 1 cycle
          CFI FunCall _char_array_to_long3
        CALL      _char_array_to_long3  ;; 3 cycles
        MOVW      N:_demand+96, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+98, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  824         demand.Bph.act_exp.max.value = char_array_to_long3(&opr_data[7]);
        MOVW      AX, #LWRD(_opr_data+7)  ;; 1 cycle
          CFI FunCall _char_array_to_long3
        CALL      _char_array_to_long3  ;; 3 cycles
        MOVW      N:_demand+116, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_demand+118, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ; ------------------------------------- Block: 16 cycles
//  825     }
//  826 }
??reset_md_115:
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock8
        ; ------------------------------------- Block: 6 cycles
        ; ------------------------------------- Total: 146 cycles

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock9 Using cfiCommon0
          CFI Function _reset_md_fg
        CODE
//  827 void reset_md_fg(void)
//  828 {
_reset_md_fg:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  829   fill_oprzero(16);
        MOV       A, #0x10           ;; 1 cycle
          CFI FunCall _fill_oprzero
        CALL      _fill_oprzero      ;; 3 cycles
//  830   eprom_write(0x0300,0,96,PAGE_1,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+6
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x60          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x300         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  831   eprom_write(0x0800,0,384,PAGE_1,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+8
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x180         ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x800         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  832   eprom_write(0x0980,0,32,PAGE_1,AUTO_CALC);  
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x20          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x980         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  833   eprom_write(0x09C0,0,32,PAGE_1,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x20          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x9C0         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  834   eprom_write(0x09A0,0,32,PAGE_1,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+14
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x20          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x9A0         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  835   eprom_write(0x09E0,0,32,PAGE_1,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+16
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x20          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x9E0         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  836   eprom_write(0x0A00,0,256,PAGE_1,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+18
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x100         ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xA00         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  837 }
        ADDW      SP, #0xE           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock9
        ; ------------------------------------- Block: 74 cycles
        ; ------------------------------------- Total: 74 cycles
//  838 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock10 Using cfiCommon0
          CFI Function _reset_md
        CODE
//  839 void reset_md(void)
//  840 {
_reset_md:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  841     fill_oprzero(128);
        MOV       A, #0x80           ;; 1 cycle
          CFI FunCall _fill_oprzero
        CALL      _fill_oprzero      ;; 3 cycles
//  842     
//  843     /* PW MD */
//  844     eprom_write(0x0380,0,48,PAGE_1,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+6
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x30          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x380         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  845     
//  846     /* Total MD */
//  847     eprom_write(0x03E0,0,32,PAGE_1,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+8
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x20          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x3E0         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  848     
//  849     /* TOD MD */
//  850     eprom_write(0x0400,0,256,PAGE_1,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x100         ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x400         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  851     demand.Allph.act_imp.max.value = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_demand+140, AX  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_demand+142, AX  ;; 1 cycle
//  852     demand.Allph.act_exp.max.value = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_demand+164, AX  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_demand+166, AX  ;; 1 cycle
//  853     demand.Allph.app_imp.max.value = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_demand+188, AX  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_demand+190, AX  ;; 1 cycle
//  854     demand.Allph.app_exp.max.value = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_demand+212, AX  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_demand+214, AX  ;; 1 cycle
//  855     demand.Allph.act_imp.max.tod_value = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_demand+136, AX  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_demand+138, AX  ;; 1 cycle
//  856     demand.Allph.act_exp.max.tod_value = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_demand+160, AX  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_demand+162, AX  ;; 1 cycle
//  857     demand.Allph.app_imp.max.tod_value = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_demand+184, AX  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_demand+186, AX  ;; 1 cycle
//  858     demand.Allph.app_exp.max.tod_value = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_demand+208, AX  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_demand+210, AX  ;; 1 cycle
//  859     
//  860 //    demand.Allph.act_imp.rising.value = 0;
//  861 //    demand.Allph.act_exp.rising.value = 0;
//  862 //    demand.Allph.app_imp.rising.value = 0;
//  863 //    demand.Allph.app_exp.rising.value = 0;
//  864 //    demand.Allph.act_imp.present.value = 0;
//  865 //    demand.Allph.act_exp.present.value = 0;
//  866 //    demand.Allph.app_imp.present.value = 0;
//  867 //    demand.Allph.app_exp.present.value = 0;
//  868     
//  869     demand.Rph.act_imp.max.value = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_demand+16, AX   ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_demand+18, AX   ;; 1 cycle
//  870     demand.Rph.act_exp.max.value = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_demand+36, AX   ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_demand+38, AX   ;; 1 cycle
//  871 //    demand.Rph.act_imp.present.value = 0;
//  872 //    demand.Rph.act_exp.present.value = 0;
//  873 //    demand.Rph.act_imp.rising.value = 0;
//  874 //    demand.Rph.act_exp.rising.value = 0;
//  875     
//  876     demand.Yph.act_imp.max.value = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_demand+56, AX   ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_demand+58, AX   ;; 1 cycle
//  877     demand.Yph.act_exp.max.value = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_demand+76, AX   ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_demand+78, AX   ;; 1 cycle
//  878 //    demand.Yph.act_imp.present.value = 0;
//  879 //    demand.Yph.act_exp.present.value = 0;
//  880 //    demand.Yph.act_imp.rising.value = 0;
//  881 //    demand.Yph.act_exp.rising.value = 0;
//  882     
//  883     demand.Bph.act_imp.max.value = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_demand+96, AX   ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_demand+98, AX   ;; 1 cycle
//  884     demand.Bph.act_exp.max.value = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_demand+116, AX  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_demand+118, AX  ;; 1 cycle
//  885 //    demand.Bph.act_imp.present.value = 0;
//  886 //    demand.Bph.act_exp.present.value = 0;
//  887 //    demand.Bph.act_imp.rising.value = 0;
//  888 //    demand.Bph.act_exp.rising.value = 0;
//  889 }
        ADDW      SP, #0x6           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock10
        ; ------------------------------------- Block: 94 cycles
        ; ------------------------------------- Total: 94 cycles

        SECTION `.iar_vfe_header`:DATA:NOALLOC:NOROOT(1)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
        DC32 0

        END
//  890 
//  891 /* 
//  892 block demand 30 min 
//  893 mdi_period = 30;
//  894 mdi_no_of_period = 1;
//  895 
//  896 sliding demand 30 min 
//  897 mdi_period = 5;
//  898 mdi_no_of_period = 6;
//  899 
//  900 mdi_interval_time = mdi_period * mdi_no_of_period;
//  901 */
// 
//     9 bytes in section .bss
// 7'355 bytes in section .text
// 
// 7'355 bytes of CODE memory
//     9 bytes of DATA memory
//
//Errors: none
//Warnings: none
