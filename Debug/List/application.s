///////////////////////////////////////////////////////////////////////////////
//
// IAR C/C++ Compiler V4.20.1.2260 for RL78               20/Dec/2020  22:38:04
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
//        BootCode\source_code\source_files\application.c
//    Command line       =
//        -f C:\Users\laptop\AppData\Local\Temp\EWE83.tmp ("D:\Dheeraj\New
//        folder\0. GDEV72 - BootCode\source_code\source_files\application.c"
//        --core s3 --code_model near --calling_convention v2
//        --near_const_location ram -o "D:\Dheeraj\New folder\0. GDEV72 -
//        BootCode\Debug\Obj" --dlib_config "C:\Program Files\IAR
//        Systems\Embedded Workbench 8.4\rl78\LIB\DLib_Config_Normal.h"
//        --double=32 -e -On --no_cse --no_unroll --no_inline --no_code_motion
//        --no_tbaa --no_cross_call --no_scheduling --no_clustering --debug -lA
//        "D:\Dheeraj\New folder\0. GDEV72 - BootCode\Debug\List" -I
//        "D:\Dheeraj\New folder\0. GDEV72 -
//        BootCode\source_code\driver_files\" -I "D:\Dheeraj\New folder\0.
//        GDEV72 - BootCode\source_code\library_files\" -I "D:\Dheeraj\New
//        folder\0. GDEV72 - BootCode\source_code\misc_files\" -I
//        "D:\Dheeraj\New folder\0. GDEV72 -
//        BootCode\source_code\source_files\" --data_model near)
//    Locale             =  C
//    List file          =
//        D:\Dheeraj\New folder\0. GDEV72 - BootCode\Debug\List\application.s
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
        EXTERN _md_reset_ip_flag
        EXTERN _OffTime
        EXTERN _OnTime
        EXTERN _mdi_period
        EXTERN _miss_md_date_f
        EXTERN _lsip_period
        EXTERN _miss_ls_date_f
        EXTERN _flag_rtc_change
        EXTERN _midnight_par_miss_f
        EXTERN _Now
        EXTERN _mdreset_type
        EXTERN _flag_tamper2
        EXTERN ?MOVE_LONG_L06
        EXTERN ?UC_DIV_L01
        EXTERN _bat_mode_secs
        EXTERN _bcd_to_decimal
        EXTERN _cal_present_demand
        EXTERN _cal_rising_demand
        EXTERN _char_array_into_time6_sec_week
        EXTERN _char_array_to_long4
        EXTERN _check_bill
        EXTERN _d_array
        EXTERN _day_counter
        EXTERN _eprom_read
        EXTERN _eprom_write
        EXTERN _load_survey_snap_shot
        EXTERN _long_into_char_array4
        EXTERN _ls_cnt_at_pwrup
        EXTERN _ls_day_counter
        EXTERN _ls_miss_fill
        EXTERN _power_off_min
        EXTERN _power_on_event
        EXTERN _power_on_min
        EXTERN _save_bill_data
        EXTERN _save_midnight_par
        EXTERN _save_tod_data
        EXTERN _sel_datediff
        EXTERN _time_into_char_array6_sec_week

        PUBLIC _chk_md_miss_pd
        PUBLIC _eoi_complet_f
        PUBLIC _esw_bill
        PUBLIC _load_bat_mode_time
        PUBLIC _load_pom
        PUBLIC _miss_bill_f
        PUBLIC _off_min
        PUBLIC _save_bat_mode_time
        PUBLIC _save_pom
        PUBLIC _tariff_no
        PUBLIC _temp1_flag
        PUBLIC _temp1_var
        PUBLIC _temp2_var
        PUBLIC _temp3_var
        PUBLIC _temp4_var
        PUBLIC _temp5_var
        
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
        
// D:\Dheeraj\New folder\0. GDEV72 - BootCode\source_code\source_files\application.c
//    1 /***********************************************************************************************************************
//    2 * File Name       : application.c
//    3 * Current Version : rev_01  
//    4 * Tool-Chain      : IAR Systems RL78
//    5 * Description     : this file contains the code related functionality of meter which is other than metrology
//    6 * Creation Date   : 5/9/2020
//    7 * Company         : Genus Power Infrastructures Limited, Jaipur
//    8 * Author          : dheer
//    9 * Version History : rev_01 :
//   10 ***********************************************************************************************************************/
//   11 /************************************ Includes **************************************/
//   12 #include "application.h"
//   13 /************************************ Local Variables *****************************************/
//   14 /************************************ Extern Variables *****************************************/

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   15 us8 tariff_no;
_tariff_no:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   16 us8 esw_bill,eoi_complet_f,miss_bill_f;
_esw_bill:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_eoi_complet_f:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_miss_bill_f:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   17 us8 temp1_flag,temp1_var,temp2_var,temp3_var,temp4_var,temp5_var,off_min;
_temp1_flag:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_temp1_var:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_temp2_var:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_temp3_var:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_temp4_var:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_temp5_var:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_off_min:
        DS 1
//   18 
//   19 
//   20 /************************************ Local Functions *******************************/
//   21 
//   22 
//   23 /************************************ Extern Functions ******************************/
//   24 
//   25 void chk_md_miss_pd();
//   26 void load_pom();
//   27 void save_pom();
//   28 void power_on_event();
//   29 void save_bat_mode_time();
//   30 void load_bat_mode_time();

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock0 Using cfiCommon0
          CFI Function _chk_md_miss_pd
        CODE
//   31 void chk_md_miss_pd()
//   32 {
_chk_md_miss_pd:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 2
        SUBW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+6
//   33   us8 local_i;
//   34   eprom_read(0x07D0,0,PAGE_1,AUTO_CALC);
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x7D0         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
//   35   md_reset_ip_flag=opr_data[11];
        MOV       A, N:_opr_data+11  ;; 1 cycle
        MOV       N:_md_reset_ip_flag, A  ;; 1 cycle
//   36   if(md_reset_ip_flag != 1)
        CMP       N:_md_reset_ip_flag, #0x1  ;; 1 cycle
        SKZ                          ;; 1 cycle
        ; ------------------------------------- Block: 12 cycles
//   37   {
//   38     md_reset_ip_flag = 0;
        MOV       N:_md_reset_ip_flag, #0x0  ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
//   39   }
//   40   eprom_read(0x0F10,0,PAGE_1,AUTO_CALC);
??chk_md_miss_pd_0:
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xF10         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
//   41   esw_bill=opr_data[1];
        MOV       A, N:_opr_data+1   ;; 1 cycle
        MOV       N:_esw_bill, A     ;; 1 cycle
//   42   
//   43 //  load_pom();
//   44 //  load_bat_mode_time();
//   45   
//   46   /* Setting OnTime structure */
//   47   OnTime = Now;
        MOVW      HL, #LWRD(_OnTime)  ;; 1 cycle
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
//   48   
//   49   temp1_flag=low;
        MOV       N:_temp1_flag, #0x0  ;; 1 cycle
//   50   temp1_var=OffTime.year; 
        MOV       A, N:_OffTime+6    ;; 1 cycle
        MOV       N:_temp1_var, A    ;; 1 cycle
//   51   temp2_var=OffTime.month;
        MOV       A, N:_OffTime+5    ;; 1 cycle
        MOV       N:_temp2_var, A    ;; 1 cycle
//   52   temp3_var=OffTime.day;
        MOV       A, N:_OffTime+3    ;; 1 cycle
        MOV       N:_temp3_var, A    ;; 1 cycle
//   53   temp4_var=OffTime.hour;
        MOV       A, N:_OffTime+2    ;; 1 cycle
        MOV       N:_temp4_var, A    ;; 1 cycle
//   54   temp5_var=OffTime.min; 
        MOV       A, N:_OffTime+1    ;; 1 cycle
        MOV       N:_temp5_var, A    ;; 1 cycle
//   55   
//   56   off_min= bcd_to_decimal(OffTime.min); /* for sliding window */
        MOV       A, N:_OffTime+1    ;; 1 cycle
          CFI FunCall _bcd_to_decimal
        CALL      _bcd_to_decimal    ;; 3 cycles
        MOV       N:_off_min, A      ;; 1 cycle
//   57   
//   58   if(OffTime.year == OnTime.year)
        MOV       A, N:_OffTime+6    ;; 1 cycle
        CMP       A, N:_OnTime+6     ;; 1 cycle
        SKZ                          ;; 4 cycles
        BR        N:??load_pom_0     ;; 4 cycles
        ; ------------------------------------- Block: 47 cycles
//   59   {
//   60     if(OffTime.month == OnTime.month)
        MOV       A, N:_OffTime+5    ;; 1 cycle
        CMP       A, N:_OnTime+5     ;; 1 cycle
        SKZ                          ;; 4 cycles
        BR        N:??load_pom_1     ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//   61     {
//   62       if(OffTime.day == OnTime.day)
        MOV       A, N:_OffTime+3    ;; 1 cycle
        CMP       A, N:_OnTime+3     ;; 1 cycle
        SKZ                          ;; 4 cycles
        BR        N:??load_pom_2     ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//   63       {
//   64         if(OffTime.hour == OnTime.hour)
        MOV       A, N:_OffTime+2    ;; 1 cycle
        CMP       A, N:_OnTime+2     ;; 1 cycle
        BNZ       ??load_pom_3       ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//   65         {
//   66           if(OffTime.min != OnTime.min)    /* when minute change is detected */
        MOV       A, N:_OffTime+1    ;; 1 cycle
        CMP       A, N:_OnTime+1     ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??load_pom_4     ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//   67           {
//   68             miss_bill_f = 1;
        MOV       N:_miss_bill_f, #0x1  ;; 1 cycle
//   69             
//   70             /* checking if period is crossed during offtime so calling cal_present_demand()*/
//   71             if((bcd_to_decimal(OffTime.min)/mdi_period)<(bcd_to_decimal(OnTime.min)/mdi_period))
        MOV       A, N:_OffTime+1    ;; 1 cycle
          CFI FunCall _bcd_to_decimal
        CALL      _bcd_to_decimal    ;; 3 cycles
        MOV       [SP+0x01], A       ;; 1 cycle
        MOV       A, N:_OnTime+1     ;; 1 cycle
          CFI FunCall _bcd_to_decimal
        CALL      _bcd_to_decimal    ;; 3 cycles
        MOV       B, A               ;; 1 cycle
        MOV       X, N:_mdi_period   ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
          CFI FunCall ?UC_DIV_L01
        CALL      N:?UC_DIV_L01      ;; 3 cycles
        MOV       C, A               ;; 1 cycle
        MOV       X, N:_mdi_period   ;; 1 cycle
        MOV       A, B               ;; 1 cycle
          CFI FunCall ?UC_DIV_L01
        CALL      N:?UC_DIV_L01      ;; 3 cycles
        MOV       B, A               ;; 1 cycle
        XCH       A, C               ;; 1 cycle
        CMP       A, B               ;; 1 cycle
        XCH       A, C               ;; 1 cycle
        BNC       ??load_pom_5       ;; 4 cycles
        ; ------------------------------------- Block: 30 cycles
//   72               //if((bcd_to_decimal(OffTime.min)/(60/mdi_sel_slide))<(bcd_to_decimal(Now.min)/(60/mdi_sel_slide)))
//   73             {							
//   74               miss_md_date_f = 1;
        MOV       N:_miss_md_date_f, #0x1  ;; 1 cycle
//   75               cal_present_demand();
          CFI FunCall _cal_present_demand
        CALL      _cal_present_demand  ;; 3 cycles
//   76               cal_rising_demand();
          CFI FunCall _cal_rising_demand
        CALL      _cal_rising_demand  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
//   77             }
//   78             
//   79             if((bcd_to_decimal(OffTime.min)/lsip_period)<(bcd_to_decimal(OnTime.min)/lsip_period))
??load_pom_5:
        MOV       A, N:_OffTime+1    ;; 1 cycle
          CFI FunCall _bcd_to_decimal
        CALL      _bcd_to_decimal    ;; 3 cycles
        MOV       [SP+0x01], A       ;; 1 cycle
        MOV       A, N:_OnTime+1     ;; 1 cycle
          CFI FunCall _bcd_to_decimal
        CALL      _bcd_to_decimal    ;; 3 cycles
        MOV       B, A               ;; 1 cycle
        MOV       X, N:_lsip_period  ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
          CFI FunCall ?UC_DIV_L01
        CALL      N:?UC_DIV_L01      ;; 3 cycles
        MOV       C, A               ;; 1 cycle
        MOV       X, N:_lsip_period  ;; 1 cycle
        MOV       A, B               ;; 1 cycle
          CFI FunCall ?UC_DIV_L01
        CALL      N:?UC_DIV_L01      ;; 3 cycles
        MOV       B, A               ;; 1 cycle
        XCH       A, C               ;; 1 cycle
        CMP       A, B               ;; 1 cycle
        XCH       A, C               ;; 1 cycle
        BNC       ??load_pom_4       ;; 4 cycles
        ; ------------------------------------- Block: 29 cycles
//   80             {
//   81               miss_ls_date_f=1;
        MOV       N:_miss_ls_date_f, #0x1  ;; 1 cycle
//   82               load_survey_snap_shot();
          CFI FunCall _load_survey_snap_shot
        CALL      _load_survey_snap_shot  ;; 3 cycles
//   83               ls_miss_fill();
          CFI FunCall _ls_miss_fill
        CALL      _ls_miss_fill      ;; 3 cycles
//   84               ls_cnt_at_pwrup();
          CFI FunCall _ls_cnt_at_pwrup
        CALL      _ls_cnt_at_pwrup   ;; 3 cycles
        BR        S:??load_pom_4     ;; 3 cycles
        ; ------------------------------------- Block: 13 cycles
//   85             }
//   86           } 
//   87         }
//   88         else
//   89         {
//   90           temp1_flag=high;
??load_pom_3:
        MOV       N:_temp1_flag, #0x1  ;; 1 cycle
        BR        S:??load_pom_4     ;; 3 cycles
        ; ------------------------------------- Block: 4 cycles
//   91         }
//   92       }
//   93       else
//   94       {
//   95         temp1_flag=high; 
??load_pom_2:
        MOV       N:_temp1_flag, #0x1  ;; 1 cycle
//   96         flag_rtc_change_day=high;
        SET1      N:_flag_rtc_change.3  ;; 2 cycles
        BR        S:??load_pom_4     ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
//   97       }
//   98     }
//   99     else
//  100     {
//  101       temp1_flag=high; 
??load_pom_1:
        MOV       N:_temp1_flag, #0x1  ;; 1 cycle
//  102       flag_rtc_change_day=high;
        SET1      N:_flag_rtc_change.3  ;; 2 cycles
        BR        S:??load_pom_4     ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
//  103     }
//  104   }
//  105   else
//  106   {
//  107     temp1_flag=high; 
??load_pom_0:
        MOV       N:_temp1_flag, #0x1  ;; 1 cycle
//  108     flag_rtc_change_day=high;
        SET1      N:_flag_rtc_change.3  ;; 2 cycles
        ; ------------------------------------- Block: 3 cycles
//  109   }
//  110   
//  111   if(temp1_flag==high)
??load_pom_4:
        CMP       N:_temp1_flag, #0x1  ;; 1 cycle
        BNZ       ??load_pom_6       ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  112   {
//  113     temp1_flag=low;
        MOV       N:_temp1_flag, #0x0  ;; 1 cycle
//  114     miss_bill_f=1;
        MOV       N:_miss_bill_f, #0x1  ;; 1 cycle
//  115     miss_ls_date_f=1;
        MOV       N:_miss_ls_date_f, #0x1  ;; 1 cycle
//  116     
//  117     load_survey_snap_shot();
          CFI FunCall _load_survey_snap_shot
        CALL      _load_survey_snap_shot  ;; 3 cycles
//  118     ls_miss_fill();
          CFI FunCall _ls_miss_fill
        CALL      _ls_miss_fill      ;; 3 cycles
//  119     /*		rising_demand_kw=0; */
//  120     /*		rising_demand_kva=0; */
//  121     miss_md_date_f=1;
        MOV       N:_miss_md_date_f, #0x1  ;; 1 cycle
//  122     cal_present_demand();
          CFI FunCall _cal_present_demand
        CALL      _cal_present_demand  ;; 3 cycles
//  123     cal_rising_demand();
          CFI FunCall _cal_rising_demand
        CALL      _cal_rising_demand  ;; 3 cycles
//  124     if(flag_rtc_change_day==1) 
        MOVW      HL, #LWRD(_flag_rtc_change)  ;; 1 cycle
        MOV1      CY, [HL].3         ;; 1 cycle
        BNC       ??load_pom_7       ;; 4 cycles
        ; ------------------------------------- Block: 22 cycles
//  125     {
//  126         midnight_par_miss_f=1;
        MOV       N:_midnight_par_miss_f, #0x1  ;; 1 cycle
//  127         save_midnight_par();
          CFI FunCall _save_midnight_par
        CALL      _save_midnight_par  ;; 3 cycles
//  128         flag_rtc_change_day=low;
        CLR1      N:_flag_rtc_change.3  ;; 2 cycles
//  129         if(d_array[day_counter]!=sel_datediff(present_date,present_month,present_year) )
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
        MOVW      AX, [DE]           ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        SKZ                          ;; 1 cycle
        ; ------------------------------------- Block: 22 cycles
//  130         {
//  131             day_counter++;
        INCW      N:_day_counter     ;; 2 cycles
          CFI FunCall _ls_day_counter
        ; ------------------------------------- Block: 2 cycles
//  132         }
//  133         
//  134         ls_day_counter();
??chk_md_miss_pd_1:
        CALL      _ls_day_counter    ;; 3 cycles
          CFI FunCall _ls_cnt_at_pwrup
        ; ------------------------------------- Block: 3 cycles
//  135         
//  136     }
//  137     
//  138     /*      day_no_inc_f=0; */
//  139     ls_cnt_at_pwrup();
??load_pom_7:
        CALL      _ls_cnt_at_pwrup   ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
//  140     ////    power_off_f=1;
//  141     ////    power_on_event();
//  142     
//  143   }
//  144   
//  145   if(miss_bill_f==1)
??load_pom_6:
        CMP       N:_miss_bill_f, #0x1  ;; 1 cycle
        SKZ                          ;; 4 cycles
        BR        N:??load_pom_8     ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  146   {
//  147     
//  148         eprom_read(0x07F0,0,PAGE_1,AUTO_CALC);
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x7F0         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
//  149 
//  150         local_i= opr_data[2];
        MOV       A, N:_opr_data+2   ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  151 
//  152         check_bill();
          CFI FunCall _check_bill
        CALL      _check_bill        ;; 3 cycles
//  153         if(local_i == 0) /* If bill date is set to zero */
        MOV       A, [SP]            ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 15 cycles
//  154         {
//  155             temp1_flag= 0;
        MOV       N:_temp1_flag, #0x0  ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
//  156         }
//  157 
//  158         eprom_read(0X07E0,0,PAGE_1,AUTO_CALC);
??chk_md_miss_pd_2:
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x7E0         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
//  159 
//  160         if((md_reset_ip_flag == 0) && (temp1_flag == high) && ((present_min != opr_data[5]) || (present_hr != opr_data[6]) || (present_date != opr_data[7]) || (present_month != opr_data[8]) || (present_year != opr_data[9])) && ((present_date != 0) && (present_month != 0) && (present_date <= 0x31) && (present_month <= 0x12)))
        CMP0      N:_md_reset_ip_flag  ;; 1 cycle
        BNZ       ??load_pom_8       ;; 4 cycles
        ; ------------------------------------- Block: 12 cycles
        CMP       N:_temp1_flag, #0x1  ;; 1 cycle
        BNZ       ??load_pom_8       ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        MOV       A, N:_Now+1        ;; 1 cycle
        CMP       A, N:_opr_data+5   ;; 1 cycle
        BNZ       ??load_pom_9       ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOV       A, N:_Now+2        ;; 1 cycle
        CMP       A, N:_opr_data+6   ;; 1 cycle
        BNZ       ??load_pom_9       ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOV       A, N:_Now+3        ;; 1 cycle
        CMP       A, N:_opr_data+7   ;; 1 cycle
        BNZ       ??load_pom_9       ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOV       A, N:_Now+5        ;; 1 cycle
        CMP       A, N:_opr_data+8   ;; 1 cycle
        BNZ       ??load_pom_9       ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOV       A, N:_Now+6        ;; 1 cycle
        CMP       A, N:_opr_data+9   ;; 1 cycle
        BZ        ??load_pom_8       ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
??load_pom_9:
        CMP0      N:_Now+3           ;; 1 cycle
        BZ        ??load_pom_8       ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        CMP0      N:_Now+5           ;; 1 cycle
        BZ        ??load_pom_8       ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        MOV       A, N:_Now+3        ;; 1 cycle
        CMP       A, #0x32           ;; 1 cycle
        BNC       ??load_pom_8       ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOV       A, N:_Now+5        ;; 1 cycle
        CMP       A, #0x13           ;; 1 cycle
        BNC       ??load_pom_8       ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  161         {
//  162             mdreset_type= 3; /* Auto Bill(miss bill) */
        MOV       N:_mdreset_type, #0x3  ;; 1 cycle
//  163             temp1_flag= low;
        MOV       N:_temp1_flag, #0x0  ;; 1 cycle
//  164             save_tod_data();
          CFI FunCall _save_tod_data
        CALL      _save_tod_data     ;; 3 cycles
//  165             save_bill_data();
          CFI FunCall _save_bill_data
        CALL      _save_bill_data    ;; 3 cycles
//  166             mdreset_type= 0;
        MOV       N:_mdreset_type, #0x0  ;; 1 cycle
          CFI FunCall _power_on_event
        ; ------------------------------------- Block: 9 cycles
//  167         }
//  168     ////    check_daily();
//  169     ////    if((daily_enr_flag==1)&&(midnight_selector_f==2)&&((meter_reading_parameter[0]&0x30)==0x30))             
//  170     ////    {
//  171     ////      if((present_date!=0)&&(present_month!=0)&&(present_date<=0x31)&&(present_month<=0x12)
//  172     ////         &&(present_hr<=0x23)&&(present_min<=0x59))
//  173     ////      {
//  174     ////        daily_enr_flag=0;
//  175     ////        if((daily_hr!=0)||(daily_min!=0))
//  176     ////        {
//  177     ////          midnight_par_miss_f=1;
//  178     ////          save_midnight_par();
//  179     ////        }
//  180     ////      }
//  181     ////    }
//  182     
//  183 
//  184     ////    read_from_256(0x03c0,m256e2p1w);
//  185     ////    if((md_reset_ip_flag==0)
//  186     ////       &&(eob_selector_f==1)&&((meter_reading_parameter[1]&0x30)==0x30)&&(temp1_flag==high)&&((present_min!=OPR11[4])||(present_hr!=hr_without_dst(OPR11[5]))||(present_date!=OPR11[6])||(present_month!=OPR11[7])||(present_year!=OPR11[8]))
//  187     ////         && (present_date!=0 && present_month!=0 && present_date<=0x31 && present_month<=0x12))
//  188     ////    {
//  189     ////      temp1_flag =low;
//  190     //        save_tod_data();
//  191     ////      //cal_billing_parameter();
//  192     ////      save_bill_data();
//  193     ////      cal_billing_parameter();
//  194     ////    }
//  195   }
//  196   power_on_event();
??load_pom_8:
        CALL      _power_on_event    ;; 3 cycles
//  197   miss_ls_date_f=0;
        MOV       N:_miss_ls_date_f, #0x0  ;; 1 cycle
//  198   miss_bill_f=0;
        MOV       N:_miss_bill_f, #0x0  ;; 1 cycle
//  199 }
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock0
        ; ------------------------------------- Block: 12 cycles
        ; ------------------------------------- Total: 350 cycles
//  200 
//  201 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock1 Using cfiCommon0
          CFI Function _save_pom
        CODE
//  202 void save_pom()
//  203 {
_save_pom:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  204   /* Saving Time, power off and power on seconds */
//  205   time_into_char_array6_sec_week(Now,&opr_data[0]);
        MOVW      HL, #LWRD(_Now)    ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+12
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      AX, #LWRD(_opr_data)  ;; 1 cycle
          CFI FunCall _time_into_char_array6_sec_week
        CALL      _time_into_char_array6_sec_week  ;; 3 cycles
//  206   long_into_char_array4(power_on_min,&opr_data[6]);
        MOVW      DE, #LWRD(_opr_data+6)  ;; 1 cycle
        MOVW      BC, N:_power_on_min+2  ;; 1 cycle
        MOVW      AX, N:_power_on_min  ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  207   long_into_char_array4(power_off_min,&opr_data[10]);
        MOVW      DE, #LWRD(_opr_data+10)  ;; 1 cycle
        MOVW      BC, N:_power_off_min+2  ;; 1 cycle
        MOVW      AX, N:_power_off_min  ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  208   opr_data[14] = flag_power_off;
        MOVW      HL, #LWRD(_flag_tamper2)  ;; 1 cycle
        MOV       A, [HL]            ;; 1 cycle
        AND       A, #0x1            ;; 1 cycle
        MOV       N:_opr_data+14, A  ;; 1 cycle
//  209   eprom_write(ADD_LIFETIME_MIN,0,16,PAGE_1,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+14
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xB00         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  210   OffTime = Now;
        MOVW      HL, #LWRD(_OffTime)  ;; 1 cycle
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
//  211 }
        ADDW      SP, #0xA           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock1
        ; ------------------------------------- Block: 59 cycles
        ; ------------------------------------- Total: 59 cycles

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock2 Using cfiCommon0
          CFI Function _save_bat_mode_time
        CODE
//  212 void save_bat_mode_time()
//  213 {
_save_bat_mode_time:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  214   /* Saving Time, power off and power on seconds */
//  215   eprom_read(ADDR_Bat_mode_time,0,PAGE_1,AUTO_CALC);
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xB10         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
//  216   long_into_char_array4(bat_mode_secs,&opr_data[0]);
        MOVW      DE, #LWRD(_opr_data)  ;; 1 cycle
        MOVW      BC, N:_bat_mode_secs+2  ;; 1 cycle
        MOVW      AX, N:_bat_mode_secs  ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  217   eprom_write(ADDR_Bat_mode_time,0,16,PAGE_1,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+6
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xB10         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  218 }
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock2
        ; ------------------------------------- Block: 29 cycles
        ; ------------------------------------- Total: 29 cycles

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock3 Using cfiCommon0
          CFI Function _load_bat_mode_time
        CODE
//  219 void load_bat_mode_time()
//  220 {
_load_bat_mode_time:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  221   /* Saving Time, power off and power on seconds */
//  222   if(eprom_read(ADDR_Bat_mode_time,0,PAGE_1,AUTO_CALC) == EEP_OK)
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xB10         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
        CMP       A, #0x1            ;; 1 cycle
        BNZ       ??load_pom_10      ;; 4 cycles
        ; ------------------------------------- Block: 12 cycles
//  223   {
//  224     bat_mode_secs = char_array_to_long4(&opr_data[0]);
        MOVW      AX, #LWRD(_opr_data)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_bat_mode_secs, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_bat_mode_secs+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ; ------------------------------------- Block: 8 cycles
//  225   }
//  226 }
??load_pom_10:
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock3
        ; ------------------------------------- Block: 6 cycles
        ; ------------------------------------- Total: 26 cycles

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock4 Using cfiCommon0
          CFI Function _load_pom
        CODE
//  227 void load_pom()
//  228 {
_load_pom:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 8
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+12
//  229   /* Reading Offtime location to get meter turn off time */
//  230   if(eprom_read(ADD_LIFETIME_MIN,0,PAGE_1,AUTO_CALC) == EEP_OK)
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xB00         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
        CMP       A, #0x1            ;; 1 cycle
        BNZ       ??load_pom_11      ;; 4 cycles
        ; ------------------------------------- Block: 13 cycles
//  231   {
//  232     OffTime = char_array_into_time6_sec_week(&opr_data[0]);
        MOVW      BC, #LWRD(_opr_data)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall _char_array_into_time6_sec_week
        CALL      _char_array_into_time6_sec_week  ;; 3 cycles
        MOVW      HL, #LWRD(_OffTime)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
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
//  233     power_on_min = char_array_to_long4(&opr_data[6]);
        MOVW      AX, #LWRD(_opr_data+6)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_power_on_min, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_power_on_min+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  234     power_off_min = char_array_to_long4(&opr_data[10]);
        MOVW      AX, #LWRD(_opr_data+10)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_power_off_min, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_power_off_min+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  235     if(opr_data[14] == 1)
        CMP       N:_opr_data+14, #0x1  ;; 1 cycle
        BNZ       ??load_pom_12      ;; 4 cycles
        ; ------------------------------------- Block: 43 cycles
//  236     {
//  237       flag_power_off = 1;
        SET1      N:_flag_tamper2.0  ;; 2 cycles
        BR        S:??load_pom_11    ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  238     }
//  239     else
//  240     {
//  241       flag_power_off = 0;
??load_pom_12:
        CLR1      N:_flag_tamper2.0  ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
//  242     }
//  243   }
//  244 }
??load_pom_11:
        ADDW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock4
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 70 cycles

        SECTION `.iar_vfe_header`:DATA:NOALLOC:NOROOT(1)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
        DC32 0

        END
//  245 
//  246 
//  247 
//  248 
//  249 
// 
//  11 bytes in section .bss
// 849 bytes in section .text
// 
// 849 bytes of CODE memory
//  11 bytes of DATA memory
//
//Errors: none
//Warnings: none
