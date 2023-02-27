///////////////////////////////////////////////////////////////////////////////
//
// IAR C/C++ Compiler V4.20.1.2260 for RL78               20/Dec/2020  22:38:05
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
//        BootCode\source_code\source_files\bill.c
//    Command line       =
//        -f C:\Users\laptop\AppData\Local\Temp\EW2A9C.tmp ("D:\Dheeraj\New
//        folder\0. GDEV72 - BootCode\source_code\source_files\bill.c" --core
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
//        D:\Dheeraj\New folder\0. GDEV72 - BootCode\Debug\List\bill.s
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
        #define SHF_WRITE 0x1

        EXTERN _opr_data
        EXTERN _temp_us32
        EXTERN _MAX_BILL
        EXTERN _miss_bill_f
        EXTERN _TempTime
        EXTERN _Now
        EXTERN _md_reset_ip_flag
        EXTERN _temp1_flag
        EXTERN ?MOVE_LONG_L06
        EXTERN ?SI_MOD_L02
        EXTERN ?UC_MOD_L01
        EXTERN ?UL_CMP_L03
        EXTERN __CmpNe64
        EXTERN __Divu64
        EXTERN __L2LLU
        EXTERN __Mul64
        EXTERN _bcd_to_decimal
        EXTERN _char_array_into_time4
        EXTERN _char_array_to_int
        EXTERN _char_array_to_long3
        EXTERN _char_array_to_long4
        EXTERN _decimal_to_bcd
        EXTERN _delay_ms
        EXTERN _demand
        EXTERN _energy
        EXTERN _eprom_read
        EXTERN _eprom_write
        EXTERN _int_into_char_array
        EXTERN _long_into_char_array3
        EXTERN _long_into_char_array4
        EXTERN _power_on_min
        EXTERN _reset_md
        EXTERN _time_into_char_array4
        EXTERN _time_into_char_array5
        EXTERN _tpr
        EXTERN _zone_pf

        PUBLIC _GetNextDate
        PUBLIC __Constant_0_0
        PUBLIC __Constant_3e8_0
        PUBLIC _bill_apparent_energy
        PUBLIC _bill_count
        PUBLIC _bill_date
        PUBLIC _bill_energy_export
        PUBLIC _bill_energy_import
        PUBLIC _bill_generated_successfully_f
        PUBLIC _bill_hr
        PUBLIC _bill_min
        PUBLIC _bill_pf
        PUBLIC _bill_pom
        PUBLIC _bill_reactive_energy_lag
        PUBLIC _bill_reactive_energy_lead
        PUBLIC _bill_tpr_cnt
        PUBLIC _bill_variable_init
        PUBLIC _cal_bill_pf
        PUBLIC _check_bill
        PUBLIC _cum_max_demand_kva
        PUBLIC _cum_max_demand_kw
        PUBLIC _current_bill_pf
        PUBLIC _l_temp
        PUBLIC _last_bill_tpr_read
        PUBLIC _md_reset_count
        PUBLIC _mdreset_type
        PUBLIC _mri_bill_flag
        PUBLIC _save_bill_data
        PUBLIC _temp_bill
        PUBLIC _temp_bill_date
        PUBLIC _update_bill_tod_data
        
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
        
// D:\Dheeraj\New folder\0. GDEV72 - BootCode\source_code\source_files\bill.c
//    1 /***********************************************************************************************************************
//    2 * File Name       : bill.c
//    3 * Current Version : rev_01  
//    4 * Tool-Chain      : IAR Systems
//    5 * Description     : 
//    6 * Creation Date   : 21-10-2020
//    7 * Company         : Genus Power Infrastructures Limited, Jaipur
//    8 * Author          : Mahesh vishnoi 
//    9 * Version History : rev_01 : initial fw release
//   10 ***********************************************************************************************************************/
//   11 /************************************ Includes **************************************/
//   12 #include "clock.h"
//   13 #include "bill.h"
//   14 /************************************ Local Variables *****************************************/

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   15 us16 bill_pf,current_bill_pf,mri_bill_flag;
_bill_pf:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_current_bill_pf:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_mri_bill_flag:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   16 us8 bill_min,bill_hr,bill_date,temp_bill_date;
_bill_min:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_bill_hr:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_bill_date:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_temp_bill_date:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   17 us32 bill_energy_import,bill_energy_export,bill_apparent_energy,bill_reactive_energy_lag,bill_reactive_energy_lead;
_bill_energy_import:
        DS 4

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_bill_energy_export:
        DS 4

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_bill_apparent_energy:
        DS 4

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_bill_reactive_energy_lag:
        DS 4

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_bill_reactive_energy_lead:
        DS 4

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   18 us8 bill_pom[3],l_temp[20];
_bill_pom:
        DS 4

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_l_temp:
        DS 20

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   19 us8 bill_tpr_cnt,mdreset_type,md_reset_count,bill_count,temp_bill;
_mdreset_type:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_md_reset_count:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_bill_count:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_temp_bill:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   20 us8 last_bill_tpr_read[2],bill_tpr_cnt;
_last_bill_tpr_read:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_bill_tpr_cnt:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   21 us32 cum_max_demand_kw,cum_max_demand_kva;
_cum_max_demand_kw:
        DS 4

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_cum_max_demand_kva:
        DS 4

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   22 us8 bill_generated_successfully_f;
_bill_generated_successfully_f:
        DS 1
//   23 /************************************ Extern Variables *****************************************/
//   24 
//   25 /************************************ Local Functions *******************************/
//   26 void save_bill_data(void);
//   27 void cal_bill_pf(void);
//   28 void check_bill(void);
//   29 void GetNextDate(char tpMin,char tpHr,char tpDate, char tpMonth, char tpYear, char tbMin, char tbHr, char tbDate, char tEvenOdd);
//   30 void update_bill_tod_data(uint16_t read_address, uint16_t write_address,uint8_t bill_cnt);
//   31 void bill_variable_init(void);
//   32 /************************************ Extern Functions ******************************/
//   33 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock0 Using cfiCommon0
          CFI Function _save_bill_data
        CODE
//   34 void save_bill_data(void)
//   35 {
_save_bill_data:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 8
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+12
//   36     us8 local_i, temp_bill;
//   37     uint16_t lu16_bill_addr;
//   38   
//   39      /***********************************************************************************/
//   40     cal_bill_pf();
          CFI FunCall _cal_bill_pf
        CALL      _cal_bill_pf       ;; 3 cycles
//   41     bill_pf= current_bill_pf;
        MOVW      AX, N:_current_bill_pf  ;; 1 cycle
        MOVW      N:_bill_pf, AX     ;; 1 cycle
//   42     
//   43     eprom_read(0x03E0,0,PAGE_1,AUTO_CALC);
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x3E0         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
//   44     TempTime=char_array_into_time4(&opr_data[3]);
        MOVW      BC, #LWRD(_opr_data+3)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall _char_array_into_time4
        CALL      _char_array_into_time4  ;; 3 cycles
        MOVW      HL, #LWRD(_TempTime)  ;; 1 cycle
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
//   45     time_into_char_array4(TempTime, &l_temp[0]);                                //save md_kw_date and time into l_tamp[0-3]
        MOVW      HL, #LWRD(_TempTime)  ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+20
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      AX, #LWRD(_l_temp)  ;; 1 cycle
          CFI FunCall _time_into_char_array4
        CALL      _time_into_char_array4  ;; 3 cycles
//   46     
//   47     eprom_read(0x03F0,0,PAGE_1,AUTO_CALC);
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x3F0         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
//   48     TempTime=char_array_into_time4(&opr_data[3]);
        MOVW      BC, #LWRD(_opr_data+3)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
          CFI FunCall _char_array_into_time4
        CALL      _char_array_into_time4  ;; 3 cycles
        MOVW      HL, #LWRD(_TempTime)  ;; 1 cycle
        ADDW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+12
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
//   49     time_into_char_array4(TempTime, &l_temp[4]);                                //save md_kva_date and time into l_tamp[4-7]
        MOVW      HL, #LWRD(_TempTime)  ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+20
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      AX, #LWRD(_l_temp+4)  ;; 1 cycle
          CFI FunCall _time_into_char_array4
        CALL      _time_into_char_array4  ;; 3 cycles
//   50   
//   51     eprom_read(0x0380,0,PAGE_1,AUTO_CALC);
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x380         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
//   52     TempTime=char_array_into_time4(&opr_data[3]);
        MOVW      BC, #LWRD(_opr_data+3)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
          CFI FunCall _char_array_into_time4
        CALL      _char_array_into_time4  ;; 3 cycles
        MOVW      HL, #LWRD(_TempTime)  ;; 1 cycle
        ADDW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+12
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
//   53     time_into_char_array4(TempTime, &l_temp[8]);                                //save r_phase_md_kw_date and time into l_tamp[8-11]
        MOVW      HL, #LWRD(_TempTime)  ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+20
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      AX, #LWRD(_l_temp+8)  ;; 1 cycle
          CFI FunCall _time_into_char_array4
        CALL      _time_into_char_array4  ;; 3 cycles
//   54     
//   55     eprom_read(0x0390,0,PAGE_1,AUTO_CALC);
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x390         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
//   56     TempTime=char_array_into_time4(&opr_data[3]);
        MOVW      BC, #LWRD(_opr_data+3)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
          CFI FunCall _char_array_into_time4
        CALL      _char_array_into_time4  ;; 3 cycles
        MOVW      HL, #LWRD(_TempTime)  ;; 1 cycle
        ADDW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+12
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
//   57     time_into_char_array4(TempTime, &l_temp[12]);                                //save y_phase_md_kva_date and time into l_tamp[12-15]
        MOVW      HL, #LWRD(_TempTime)  ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+20
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      AX, #LWRD(_l_temp+12)  ;; 1 cycle
          CFI FunCall _time_into_char_array4
        CALL      _time_into_char_array4  ;; 3 cycles
//   58   
//   59     eprom_read(0x03A0,0,PAGE_1,AUTO_CALC);
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x3A0         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
//   60     TempTime=char_array_into_time4(&opr_data[3]);
        MOVW      BC, #LWRD(_opr_data+3)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
          CFI FunCall _char_array_into_time4
        CALL      _char_array_into_time4  ;; 3 cycles
        MOVW      HL, #LWRD(_TempTime)  ;; 1 cycle
        ADDW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+12
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
//   61     time_into_char_array4(TempTime, &l_temp[16]);                                //save b_phase_md_kva_date and time into l_tamp[16-19]
        MOVW      HL, #LWRD(_TempTime)  ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+20
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      AX, #LWRD(_l_temp+16)  ;; 1 cycle
          CFI FunCall _time_into_char_array4
        CALL      _time_into_char_array4  ;; 3 cycles
//   62 
//   63     eprom_read(0X07E0,0,PAGE_1,AUTO_CALC);   
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x7E0         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
//   64     last_bill_tpr_read[0]= opr_data[0]; 
        MOV       A, N:_opr_data     ;; 1 cycle
        MOV       N:_last_bill_tpr_read, A  ;; 1 cycle
//   65     last_bill_tpr_read[1]= opr_data[1];
        MOV       A, N:_opr_data+1   ;; 1 cycle
        MOV       N:_last_bill_tpr_read+1, A  ;; 1 cycle
//   66     temp_us32= char_array_to_int(last_bill_tpr_read);  
        MOVW      AX, #LWRD(_last_bill_tpr_read)  ;; 1 cycle
          CFI FunCall _char_array_to_int
        CALL      _char_array_to_int  ;; 3 cycles
        MOVW      BC, #0x0           ;; 1 cycle
        MOVW      S:_temp_us32, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_temp_us32+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//   67     bill_tpr_cnt= tpr.cum_tpr_count - temp_us32;
        MOVW      DE, N:_tpr+30      ;; 1 cycle
        MOV       A, S:_temp_us32    ;; 1 cycle
        XCH       A, E               ;; 1 cycle
        SUB       A, E               ;; 1 cycle
        XCH       A, E               ;; 1 cycle
        MOV       A, E               ;; 1 cycle
        MOV       N:_bill_tpr_cnt, A  ;; 1 cycle
//   68 
//   69     last_bill_tpr_read[0]= (tpr.cum_tpr_count/256);
        MOVW      AX, N:_tpr+30      ;; 1 cycle
        CLRB      X                  ;; 1 cycle
        MOV       N:_last_bill_tpr_read, A  ;; 1 cycle
        XCH       A, X               ;; 1 cycle
//   70     last_bill_tpr_read[1]= (tpr.cum_tpr_count%256);
        MOVW      AX, N:_tpr+30      ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       N:_last_bill_tpr_read+1, A  ;; 1 cycle
        XCH       A, X               ;; 1 cycle
//   71     
//   72     temp_us32 =char_array_to_long3(&opr_data[12]) ;  
        MOVW      AX, #LWRD(_opr_data+12)  ;; 1 cycle
          CFI FunCall _char_array_to_long3
        CALL      _char_array_to_long3  ;; 3 cycles
        MOVW      S:_temp_us32, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_temp_us32+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//   73     temp_us32 = power_on_min - temp_us32;                                     //(total pom - till last bill total pom)
        MOVW      BC, N:_power_on_min+2  ;; 1 cycle
        MOVW      AX, N:_power_on_min  ;; 1 cycle
        SUBW      AX, S:_temp_us32   ;; 1 cycle
        SKNC
        DECW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        SUBW      AX, S:_temp_us32+2  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_temp_us32, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_temp_us32+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//   74     long_into_char_array3(temp_us32,&bill_pom[0]);
        MOVW      DE, #LWRD(_bill_pom)  ;; 1 cycle
        MOVW      BC, S:_temp_us32+2  ;; 1 cycle
        MOVW      AX, S:_temp_us32   ;; 1 cycle
          CFI FunCall _long_into_char_array3
        CALL      _long_into_char_array3  ;; 3 cycles
//   75       
//   76     md_reset_count= opr_data[2];
        MOV       A, N:_opr_data+2   ;; 1 cycle
        MOV       N:_md_reset_count, A  ;; 1 cycle
//   77     
//   78     if(md_reset_count > 180)
        MOV       A, N:_md_reset_count  ;; 1 cycle
        ADDW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+12
        CMP       A, #0xB5           ;; 1 cycle
        SKC                          ;; 1 cycle
        ; ------------------------------------- Block: 284 cycles
//   79     {
//   80         md_reset_count= 0;
        MOV       N:_md_reset_count, #0x0  ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
//   81     }
//   82     
//   83     md_reset_count++;
??save_bill_data_0:
        INC       N:_md_reset_count  ;; 2 cycles
//   84     bill_count= opr_data[3];
        MOV       A, N:_opr_data+3   ;; 1 cycle
        MOV       N:_bill_count, A   ;; 1 cycle
//   85     temp_bill= opr_data[3]; /* bill counter */
        MOV       X, N:_opr_data+3   ;; 1 cycle
//   86     if(temp_bill > MAX_BILL)
        MOV       A, N:_MAX_BILL     ;; 1 cycle
        CMP       A, X               ;; 1 cycle
        BNC       ??bill_variable_init_0  ;; 4 cycles
        ; ------------------------------------- Block: 11 cycles
//   87     {
//   88         temp_bill= 0;
        MOV       A, #0x0            ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
//   89     }
//   90     temp_bill++;
??bill_variable_init_0:
        INC       X                  ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       [SP+0x01], A       ;; 1 cycle
        XCH       A, X               ;; 1 cycle
//   91     if(temp_bill > MAX_BILL)
        MOV       X, N:_MAX_BILL     ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        CMP       X, A               ;; 1 cycle
        BNC       ??bill_variable_init_1  ;; 4 cycles
        ; ------------------------------------- Block: 11 cycles
//   92     {
//   93         temp_bill= 1;
        MOV       A, #0x1            ;; 1 cycle
        MOV       [SP+0x01], A       ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
//   94     }
//   95     
//   96     if(md_reset_count > 180)
??bill_variable_init_1:
        MOV       A, N:_md_reset_count  ;; 1 cycle
        CMP       A, #0xB5           ;; 1 cycle
        SKC                          ;; 1 cycle
        ; ------------------------------------- Block: 3 cycles
//   97     {
//   98         md_reset_count= 1;
        MOV       N:_md_reset_count, #0x1  ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
//   99     }
//  100     
//  101     bill_count= temp_bill;
??save_bill_data_1:
        MOV       A, [SP+0x01]       ;; 1 cycle
        MOV       N:_bill_count, A   ;; 1 cycle
//  102     
//  103     eprom_read(0x07D0,0,PAGE_1,AUTO_CALC);
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x7D0         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
//  104     cum_max_demand_kw+= demand.Allph.act_imp.max.value;                
        MOVW      BC, N:_cum_max_demand_kw+2  ;; 1 cycle
        MOVW      AX, N:_cum_max_demand_kw  ;; 1 cycle
        ADDW      AX, N:_demand+140  ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, N:_demand+142  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_cum_max_demand_kw, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_cum_max_demand_kw+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  105     long_into_char_array4(cum_max_demand_kw, &opr_data[0]);                      //CUM_MD_KW
        MOVW      DE, #LWRD(_opr_data)  ;; 1 cycle
        MOVW      BC, N:_cum_max_demand_kw+2  ;; 1 cycle
        MOVW      AX, N:_cum_max_demand_kw  ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  106 
//  107     cum_max_demand_kva+= demand.Allph.app_imp.max.value;                       //CUM_MD_KVA
        MOVW      BC, N:_cum_max_demand_kva+2  ;; 1 cycle
        MOVW      AX, N:_cum_max_demand_kva  ;; 1 cycle
        ADDW      AX, N:_demand+188  ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, N:_demand+190  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_cum_max_demand_kva, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_cum_max_demand_kva+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  108     long_into_char_array4(cum_max_demand_kva, &opr_data[4]);
        MOVW      DE, #LWRD(_opr_data+4)  ;; 1 cycle
        MOVW      BC, N:_cum_max_demand_kva+2  ;; 1 cycle
        MOVW      AX, N:_cum_max_demand_kva  ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  109     
//  110     eprom_write(0x07D0,0,16,PAGE_1,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+14
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x7D0         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  111     
//  112     bill_energy_import= energy.Allph.active_imp;
        MOVW      BC, N:_energy+54   ;; 1 cycle
        MOVW      AX, N:_energy+52   ;; 1 cycle
        MOVW      N:_bill_energy_import, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_bill_energy_import+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  113     bill_apparent_energy= energy.Allph.apparent_imp;
        MOVW      BC, N:_energy+70   ;; 1 cycle
        MOVW      AX, N:_energy+68   ;; 1 cycle
        MOVW      N:_bill_apparent_energy, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_bill_apparent_energy+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  114     bill_reactive_energy_lag= energy.Allph.reactive_q1;
        MOVW      BC, N:_energy+78   ;; 1 cycle
        MOVW      AX, N:_energy+76   ;; 1 cycle
        MOVW      N:_bill_reactive_energy_lag, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_bill_reactive_energy_lag+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  115     bill_reactive_energy_lead= energy.Allph.reactive_q4;
        MOVW      BC, N:_energy+90   ;; 1 cycle
        MOVW      AX, N:_energy+88   ;; 1 cycle
        MOVW      N:_bill_reactive_energy_lead, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_bill_reactive_energy_lead+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  116     
//  117     if(bill_energy_import > bill_apparent_energy)
        MOVW      AX, N:_bill_energy_import+2  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+16
        MOVW      AX, N:_bill_energy_import  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+18
        MOVW      BC, N:_bill_apparent_energy+2  ;; 1 cycle
        MOVW      AX, N:_bill_apparent_energy  ;; 1 cycle
          CFI FunCall ?UL_CMP_L03
        CALL      N:?UL_CMP_L03      ;; 3 cycles
        ADDW      SP, #0x6           ;; 1 cycle
          CFI CFA SP+12
        BNC       ??bill_variable_init_2  ;; 4 cycles
        ; ------------------------------------- Block: 98 cycles
//  118     {
//  119       bill_apparent_energy = bill_energy_import;
        MOVW      BC, N:_bill_energy_import+2  ;; 1 cycle
        MOVW      AX, N:_bill_energy_import  ;; 1 cycle
        MOVW      N:_bill_apparent_energy, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_bill_apparent_energy+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
//  120     }
//  121     /***********************************************************************************/   
//  122     
//  123     if(miss_bill_f == 1)                                     //BILL RTC
??bill_variable_init_2:
        CMP       N:_miss_bill_f, #0x1  ;; 1 cycle
        BNZ       ??bill_variable_init_3  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  124     { 
//  125         miss_bill_f= 0;
        MOV       N:_miss_bill_f, #0x0  ;; 1 cycle
//  126         eprom_read(0X07B0,0,PAGE_1,AUTO_CALC);   
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x7B0         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
//  127         
//  128         TempTime.min   =   bill_min;
        MOV       A, N:_bill_min     ;; 1 cycle
        MOV       N:_TempTime+1, A   ;; 1 cycle
//  129         TempTime.hour  =   bill_hr;
        MOV       A, N:_bill_hr      ;; 1 cycle
        MOV       N:_TempTime+2, A   ;; 1 cycle
//  130         TempTime.month =   opr_data[8];
        MOV       A, N:_opr_data+8   ;; 1 cycle
        MOV       N:_TempTime+5, A   ;; 1 cycle
//  131         TempTime.year  =   opr_data[9];
        MOV       A, N:_opr_data+9   ;; 1 cycle
        MOV       N:_TempTime+6, A   ;; 1 cycle
//  132         TempTime.day   =   bill_date;
        MOV       A, N:_bill_date    ;; 1 cycle
        MOV       N:_TempTime+3, A   ;; 1 cycle
//  133         time_into_char_array4(TempTime, &opr_data[0]);
        MOVW      HL, #LWRD(_TempTime)  ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+20
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      AX, #LWRD(_opr_data)  ;; 1 cycle
          CFI FunCall _time_into_char_array4
        CALL      _time_into_char_array4  ;; 3 cycles
        ADDW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+12
        BR        S:??bill_variable_init_4  ;; 3 cycles
        ; ------------------------------------- Block: 33 cycles
//  134     }
//  135     else
//  136     {   
//  137         time_into_char_array4(Now,&opr_data[0]);
??bill_variable_init_3:
        MOVW      HL, #LWRD(_Now)    ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+20
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      AX, #LWRD(_opr_data)  ;; 1 cycle
          CFI FunCall _time_into_char_array4
        CALL      _time_into_char_array4  ;; 3 cycles
        ADDW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+12
        ; ------------------------------------- Block: 12 cycles
//  138     }
//  139     
//  140     int_into_char_array(bill_pf, &opr_data[4]);                                 //BILL PF   
??bill_variable_init_4:
        MOVW      BC, #LWRD(_opr_data+4)  ;; 1 cycle
        MOVW      AX, N:_bill_pf     ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  141     long_into_char_array4(bill_energy_import,&opr_data[6]);                     //bill_energy_import
        MOVW      DE, #LWRD(_opr_data+6)  ;; 1 cycle
        MOVW      BC, N:_bill_energy_import+2  ;; 1 cycle
        MOVW      AX, N:_bill_energy_import  ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  142     long_into_char_array4(bill_reactive_energy_lag,&opr_data[10]);              //bill_reactive_energy_lag
        MOVW      DE, #LWRD(_opr_data+10)  ;; 1 cycle
        MOVW      BC, N:_bill_reactive_energy_lag+2  ;; 1 cycle
        MOVW      AX, N:_bill_reactive_energy_lag  ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  143     long_into_char_array4(bill_reactive_energy_lead,&opr_data[14]);             //bill_reactive_energy_lead
        MOVW      DE, #LWRD(_opr_data+14)  ;; 1 cycle
        MOVW      BC, N:_bill_reactive_energy_lead+2  ;; 1 cycle
        MOVW      AX, N:_bill_reactive_energy_lead  ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  144     long_into_char_array4(bill_apparent_energy,&opr_data[18]);                  //bill_apparent energy
        MOVW      DE, #LWRD(_opr_data+18)  ;; 1 cycle
        MOVW      BC, N:_bill_apparent_energy+2  ;; 1 cycle
        MOVW      AX, N:_bill_apparent_energy  ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  145     
//  146     
//  147     
//  148     /***********************md kw,kva with date and time start******************************/
//  149     
//  150     
//  151     
//  152     long_into_char_array3(demand.Allph.act_imp.max.value,&opr_data[22]);         //active demand                                       
        MOVW      DE, #LWRD(_opr_data+22)  ;; 1 cycle
        MOVW      BC, N:_demand+142  ;; 1 cycle
        MOVW      AX, N:_demand+140  ;; 1 cycle
          CFI FunCall _long_into_char_array3
        CALL      _long_into_char_array3  ;; 3 cycles
//  153     for(local_i= 0; local_i < 4; local_i++)
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        ; ------------------------------------- Block: 37 cycles
??save_bill_data_2:
        MOV       A, [SP]            ;; 1 cycle
        CMP       A, #0x4            ;; 1 cycle
        BNC       ??bill_variable_init_5  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  154     {
//  155         *(opr_data + local_i + 25)= *(l_temp + local_i);                         //active demand date and time
        MOV       A, [SP]            ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        MOV       A, (_l_temp)[B]    ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       (_opr_data+25)[B], A  ;; 1 cycle
        XCH       A, X               ;; 1 cycle
//  156     }
        MOV       A, [SP]            ;; 1 cycle
        INC       A                  ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        BR        S:??save_bill_data_2  ;; 3 cycles
        ; ------------------------------------- Block: 15 cycles
//  157 
//  158     long_into_char_array3(demand.Allph.app_imp.max.value,&opr_data[29]);       //apparent demand energy
??bill_variable_init_5:
        MOVW      DE, #LWRD(_opr_data+29)  ;; 1 cycle
        MOVW      BC, N:_demand+190  ;; 1 cycle
        MOVW      AX, N:_demand+188  ;; 1 cycle
          CFI FunCall _long_into_char_array3
        CALL      _long_into_char_array3  ;; 3 cycles
//  159     
//  160     for(local_i= 0; local_i < 4; local_i++)
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        ; ------------------------------------- Block: 8 cycles
??save_bill_data_3:
        MOV       A, [SP]            ;; 1 cycle
        CMP       A, #0x4            ;; 1 cycle
        BNC       ??bill_variable_init_6  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  161     {
//  162         *(opr_data + local_i + 32)= *(l_temp + local_i + 4);                         //apparent demand date and time
        MOV       A, [SP]            ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        MOV       A, (_l_temp+4)[B]  ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       (_opr_data+32)[B], A  ;; 1 cycle
        XCH       A, X               ;; 1 cycle
//  163     }
        MOV       A, [SP]            ;; 1 cycle
        INC       A                  ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        BR        S:??save_bill_data_3  ;; 3 cycles
        ; ------------------------------------- Block: 15 cycles
//  164 
//  165     /* In case md not came, and bill was given, this check is applied. */        //need to be take care this case IF occur
//  166     
//  167 
//  168     /***********************md kw,kva with date and time end******************************/
//  169     
//  170     opr_data[36]= bill_pom[0];                                                   // delta bill power on minute
??bill_variable_init_6:
        MOV       A, N:_bill_pom     ;; 1 cycle
        MOV       N:_opr_data+36, A  ;; 1 cycle
//  171     opr_data[37]= bill_pom[1];
        MOV       A, N:_bill_pom+1   ;; 1 cycle
        MOV       N:_opr_data+37, A  ;; 1 cycle
//  172     opr_data[38]= bill_pom[2];
        MOV       A, N:_bill_pom+2   ;; 1 cycle
        MOV       N:_opr_data+38, A  ;; 1 cycle
//  173 
//  174     long_into_char_array4(energy.Allph.fundamental,&opr_data[39]);               //bill_fundamental/defraud energy
        MOVW      DE, #LWRD(_opr_data+39)  ;; 1 cycle
        MOVW      BC, N:_energy+66   ;; 1 cycle
        MOVW      AX, N:_energy+64   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  175        
//  176     opr_data[43]= bill_tpr_cnt;                                                   // bill tamper count
        MOV       A, N:_bill_tpr_cnt  ;; 1 cycle
        MOV       N:_opr_data+43, A  ;; 1 cycle
//  177     opr_data[44]= mdreset_type;                                                   // cause of md reset
        MOV       A, N:_mdreset_type  ;; 1 cycle
        MOV       N:_opr_data+44, A  ;; 1 cycle
//  178     /* used to distinguish Cause of MD Reset(Manual, Through Command(MRI), Auto/Bill Generated) */
//  179     /* 0-NA,1-Manual,2-MRI,3-Auto/Bill generated */
//  180     
//  181     opr_data[45]= md_reset_count;                                                  // number of md reset count
        MOV       A, N:_md_reset_count  ;; 1 cycle
        MOV       N:_opr_data+45, A  ;; 1 cycle
//  182  
//  183     /***********************phase wise r,y,b md kw with date and time start******************************/
//  184     
//  185     
//  186     long_into_char_array3(demand.Rph.act_imp.max.value,&opr_data[46]);             //r active demand                                       
        MOVW      DE, #LWRD(_opr_data+46)  ;; 1 cycle
        MOVW      BC, N:_demand+18   ;; 1 cycle
        MOVW      AX, N:_demand+16   ;; 1 cycle
          CFI FunCall _long_into_char_array3
        CALL      _long_into_char_array3  ;; 3 cycles
//  187     for(local_i= 0; local_i < 4; local_i++)
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        ; ------------------------------------- Block: 26 cycles
??save_bill_data_4:
        MOV       A, [SP]            ;; 1 cycle
        CMP       A, #0x4            ;; 1 cycle
        BNC       ??bill_variable_init_7  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  188     {
//  189         *(opr_data + local_i + 49)= *(l_temp + local_i + 8);                       //r active demand date and time
        MOV       A, [SP]            ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        MOV       A, (_l_temp+8)[B]  ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       (_opr_data+49)[B], A  ;; 1 cycle
        XCH       A, X               ;; 1 cycle
//  190     }
        MOV       A, [SP]            ;; 1 cycle
        INC       A                  ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        BR        S:??save_bill_data_4  ;; 3 cycles
        ; ------------------------------------- Block: 15 cycles
//  191 
//  192     long_into_char_array3(demand.Yph.act_imp.max.value,&opr_data[53]);             //Y active  demand energy
??bill_variable_init_7:
        MOVW      DE, #LWRD(_opr_data+53)  ;; 1 cycle
        MOVW      BC, N:_demand+58   ;; 1 cycle
        MOVW      AX, N:_demand+56   ;; 1 cycle
          CFI FunCall _long_into_char_array3
        CALL      _long_into_char_array3  ;; 3 cycles
//  193     
//  194     for(local_i= 0; local_i < 4; local_i++)
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        ; ------------------------------------- Block: 8 cycles
??save_bill_data_5:
        MOV       A, [SP]            ;; 1 cycle
        CMP       A, #0x4            ;; 1 cycle
        BNC       ??bill_variable_init_8  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  195     {
//  196         *(opr_data + local_i + 56)= *(l_temp + local_i + 12);                       //Y active demand date and time
        MOV       A, [SP]            ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        MOV       A, (_l_temp+12)[B]  ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       (_opr_data+56)[B], A  ;; 1 cycle
        XCH       A, X               ;; 1 cycle
//  197     }
        MOV       A, [SP]            ;; 1 cycle
        INC       A                  ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        BR        S:??save_bill_data_5  ;; 3 cycles
        ; ------------------------------------- Block: 15 cycles
//  198     
//  199     long_into_char_array3(demand.Bph.act_imp.max.value,&opr_data[60]);             //B active demand energy
??bill_variable_init_8:
        MOVW      DE, #LWRD(_opr_data+60)  ;; 1 cycle
        MOVW      BC, N:_demand+98   ;; 1 cycle
        MOVW      AX, N:_demand+96   ;; 1 cycle
          CFI FunCall _long_into_char_array3
        CALL      _long_into_char_array3  ;; 3 cycles
//  200     
//  201     for(local_i= 0; local_i < 4; local_i++)
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        ; ------------------------------------- Block: 8 cycles
??save_bill_data_6:
        MOV       A, [SP]            ;; 1 cycle
        CMP       A, #0x4            ;; 1 cycle
        BNC       ??bill_variable_init_9  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  202     {
//  203         *(opr_data + local_i + 63)= *(l_temp + local_i + 16);                       //B active demand date and time
        MOV       A, [SP]            ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        MOV       A, (_l_temp+16)[B]  ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       (_opr_data+63)[B], A  ;; 1 cycle
        XCH       A, X               ;; 1 cycle
//  204     }
        MOV       A, [SP]            ;; 1 cycle
        INC       A                  ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        BR        S:??save_bill_data_6  ;; 3 cycles
        ; ------------------------------------- Block: 15 cycles
//  205    
//  206     
//  207     lu16_bill_addr=(BILL_START_ADD+((temp_bill - 1)*0xA0));
??bill_variable_init_9:
        MOV       A, [SP+0x01]       ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0xA0          ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #0x1760        ;; 1 cycle
        MOVW      [SP+0x02], AX      ;; 1 cycle
//  208     eprom_write(lu16_bill_addr,0,80,PAGE_5,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+14
        MOV       B, #0x4            ;; 1 cycle
        MOVW      DE, #0x50          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  209     
//  210     /***********************end of billing data******************************/
//  211     
//  212     /***********************billing tod data******************************/  
//  213 
//  214     temp_us32= BILL_TOD_MD_KW_ADDR;
        MOVW      AX, #0x3200        ;; 1 cycle
        MOVW      S:_temp_us32, AX   ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      S:_temp_us32+2, AX  ;; 1 cycle
//  215     temp_us32= temp_us32 + (temp_bill - 1) * 0x80;
        MOV       A, [SP+0x03]       ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        DECW      AX                 ;; 1 cycle
        MOVW      BC, #0x80          ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      HL, S:_temp_us32+2  ;; 1 cycle
        MOVW      DE, S:_temp_us32   ;; 1 cycle
        ADDW      AX, DE             ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_temp_us32, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_temp_us32+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  216     for(local_i= 0; local_i < 8; local_i++)
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP+0x02], A       ;; 1 cycle
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+12
        ; ------------------------------------- Block: 49 cycles
??save_bill_data_7:
        MOV       A, [SP]            ;; 1 cycle
        CMP       A, #0x8            ;; 1 cycle
        BNC       ??bill_variable_init_10  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  217     {
//  218         eprom_read((TOD_MD_KW_ADDR+(32 * local_i)),0,PAGE_1,AUTO_CALC);                /* tod md kW import,export */
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
        MOV       A, [SP]            ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x20          ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #0x400         ;; 1 cycle
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
//  219  
//  220         eprom_write(temp_us32,0,16,PAGE_1,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+14
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        PUSH      DE                 ;; 1 cycle
          CFI CFA SP+16
        MOVW      HL, S:_temp_us32+2  ;; 1 cycle
        MOVW      DE, S:_temp_us32   ;; 1 cycle
        MOVW      AX, DE             ;; 1 cycle
        POP       DE                 ;; 1 cycle
          CFI CFA SP+14
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  221         temp_us32+= 0X10;
        MOVW      BC, S:_temp_us32+2  ;; 1 cycle
        MOVW      AX, S:_temp_us32   ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_temp_us32, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_temp_us32+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  222         delay_ms(1);
        MOVW      AX, #0x1           ;; 1 cycle
          CFI FunCall _delay_ms
        CALL      _delay_ms          ;; 3 cycles
//  223     }
        MOV       A, [SP+0x02]       ;; 1 cycle
        INC       A                  ;; 1 cycle
        MOV       [SP+0x02], A       ;; 1 cycle
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+12
        BR        S:??save_bill_data_7  ;; 3 cycles
        ; ------------------------------------- Block: 70 cycles
//  224 
//  225     temp_us32= BILL_TOD_MD_KVA_ADDR;
??bill_variable_init_10:
        MOVW      AX, #0x3800        ;; 1 cycle
        MOVW      S:_temp_us32, AX   ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      S:_temp_us32+2, AX  ;; 1 cycle
//  226     temp_us32= temp_us32 + (temp_bill - 1) * 0x80;
        MOV       A, [SP+0x01]       ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        DECW      AX                 ;; 1 cycle
        MOVW      BC, #0x80          ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      HL, S:_temp_us32+2  ;; 1 cycle
        MOVW      DE, S:_temp_us32   ;; 1 cycle
        ADDW      AX, DE             ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_temp_us32, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_temp_us32+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  227     for(local_i= 0; local_i < 8; local_i++)
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        ; ------------------------------------- Block: 31 cycles
??save_bill_data_8:
        MOV       A, [SP]            ;; 1 cycle
        CMP       A, #0x8            ;; 1 cycle
        BNC       ??bill_variable_init_11  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  228     {
//  229         eprom_read((TOD_MD_KVA_ADDR+(32 * local_i)),0,PAGE_1,AUTO_CALC);                 /* tod md kva import,export */        
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
        MOV       A, [SP]            ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x20          ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #0x410         ;; 1 cycle
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
//  230         
//  231         eprom_write(temp_us32,0,16,PAGE_1,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+14
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        PUSH      DE                 ;; 1 cycle
          CFI CFA SP+16
        MOVW      HL, S:_temp_us32+2  ;; 1 cycle
        MOVW      DE, S:_temp_us32   ;; 1 cycle
        MOVW      AX, DE             ;; 1 cycle
        POP       DE                 ;; 1 cycle
          CFI CFA SP+14
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  232         temp_us32+= 0X10;        
        MOVW      BC, S:_temp_us32+2  ;; 1 cycle
        MOVW      AX, S:_temp_us32   ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_temp_us32, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_temp_us32+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  233         delay_ms(1);
        MOVW      AX, #0x1           ;; 1 cycle
          CFI FunCall _delay_ms
        CALL      _delay_ms          ;; 3 cycles
//  234     }
        MOV       A, [SP+0x02]       ;; 1 cycle
        INC       A                  ;; 1 cycle
        MOV       [SP+0x02], A       ;; 1 cycle
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+12
        BR        S:??save_bill_data_8  ;; 3 cycles
        ; ------------------------------------- Block: 70 cycles
//  235     
//  236     update_bill_tod_data(0X1600,BILL_TOD_CUR_BILL_BLK1,temp_bill);
??bill_variable_init_11:
        MOV       A, [SP+0x01]       ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOVW      BC, #0x2000        ;; 1 cycle
        MOVW      AX, #0x1600        ;; 1 cycle
          CFI FunCall _update_bill_tod_data
        CALL      _update_bill_tod_data  ;; 3 cycles
//  237     update_bill_tod_data(0X1680,BILL_TOD_CUR_BILL_BLK2,temp_bill);
        MOV       A, [SP+0x01]       ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOVW      BC, #0x2600        ;; 1 cycle
        MOVW      AX, #0x1680        ;; 1 cycle
          CFI FunCall _update_bill_tod_data
        CALL      _update_bill_tod_data  ;; 3 cycles
//  238     update_bill_tod_data(0X1700,BILL_TOD_CUR_BILL_BLK3,temp_bill);
        MOV       A, [SP+0x01]       ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOVW      BC, #0x2C00        ;; 1 cycle
        MOVW      AX, #0x1700        ;; 1 cycle
          CFI FunCall _update_bill_tod_data
        CALL      _update_bill_tod_data  ;; 3 cycles
//  239     
//  240     /* ***********************billing tod data*********** */
//  241     
//  242 //   /* *******************Reset MD & power on hours*************** */
//  243 
//  244 //    if(md_type == 1) /* sliding */
//  245 //    {
//  246 //        main2= mdi_sel;
//  247 //        bill_mdi_slide_write();
//  248 //    }
//  249 
//  250     
//  251     /* **************************************************/
//  252     eprom_read(0X07C0,0,PAGE_1,AUTO_CALC);
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x7C0         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
//  253     long_into_char_array3(demand.Allph.act_imp.max.value,&opr_data[0]);           //active demand   
        MOVW      DE, #LWRD(_opr_data)  ;; 1 cycle
        MOVW      BC, N:_demand+142  ;; 1 cycle
        MOVW      AX, N:_demand+140  ;; 1 cycle
          CFI FunCall _long_into_char_array3
        CALL      _long_into_char_array3  ;; 3 cycles
//  254     long_into_char_array3(demand.Allph.app_imp.max.value,&opr_data[3]);           //apparent demand energy
        MOVW      DE, #LWRD(_opr_data+3)  ;; 1 cycle
        MOVW      BC, N:_demand+190  ;; 1 cycle
        MOVW      AX, N:_demand+188  ;; 1 cycle
          CFI FunCall _long_into_char_array3
        CALL      _long_into_char_array3  ;; 3 cycles
//  255     long_into_char_array3(demand.Rph.act_imp.max.value,&opr_data[6]);             //r active demand  
        MOVW      DE, #LWRD(_opr_data+6)  ;; 1 cycle
        MOVW      BC, N:_demand+18   ;; 1 cycle
        MOVW      AX, N:_demand+16   ;; 1 cycle
          CFI FunCall _long_into_char_array3
        CALL      _long_into_char_array3  ;; 3 cycles
//  256     long_into_char_array3(demand.Yph.act_imp.max.value,&opr_data[9]);             //Y active  demand energy
        MOVW      DE, #LWRD(_opr_data+9)  ;; 1 cycle
        MOVW      BC, N:_demand+58   ;; 1 cycle
        MOVW      AX, N:_demand+56   ;; 1 cycle
          CFI FunCall _long_into_char_array3
        CALL      _long_into_char_array3  ;; 3 cycles
//  257     long_into_char_array3(demand.Bph.act_imp.max.value,&opr_data[12]);            //B active demand energy
        MOVW      DE, #LWRD(_opr_data+12)  ;; 1 cycle
        MOVW      BC, N:_demand+98   ;; 1 cycle
        MOVW      AX, N:_demand+96   ;; 1 cycle
          CFI FunCall _long_into_char_array3
        CALL      _long_into_char_array3  ;; 3 cycles
//  258     eprom_write(0X07C0,0,16,PAGE_1,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+14
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x7C0         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  259     
//  260     reset_md();
          CFI FunCall _reset_md
        CALL      _reset_md          ;; 3 cycles
//  261 
//  262     eprom_read(0X07E0,0,PAGE_1,AUTO_CALC);
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x7E0         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
//  263     int_into_char_array(tpr.cum_tpr_count,&opr_data[0]);
        MOVW      BC, #LWRD(_opr_data)  ;; 1 cycle
        MOVW      AX, N:_tpr+30      ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  264     opr_data[2]= md_reset_count;
        MOV       A, N:_md_reset_count  ;; 1 cycle
        MOV       N:_opr_data+2, A   ;; 1 cycle
//  265     opr_data[3]= bill_count;
        MOV       A, N:_bill_count   ;; 1 cycle
        MOV       N:_opr_data+3, A   ;; 1 cycle
//  266     opr_data[4]= bill_tpr_cnt;
        MOV       A, N:_bill_tpr_cnt  ;; 1 cycle
        MOV       N:_opr_data+4, A   ;; 1 cycle
//  267     time_into_char_array5(Now, &opr_data[5]);
        MOVW      HL, #LWRD(_Now)    ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+22
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      AX, #LWRD(_opr_data+5)  ;; 1 cycle
          CFI FunCall _time_into_char_array5
        CALL      _time_into_char_array5  ;; 3 cycles
//  268     long_into_char_array3(power_on_min, &opr_data[12]);
        MOVW      DE, #LWRD(_opr_data+12)  ;; 1 cycle
        MOVW      BC, N:_power_on_min+2  ;; 1 cycle
        MOVW      AX, N:_power_on_min  ;; 1 cycle
          CFI FunCall _long_into_char_array3
        CALL      _long_into_char_array3  ;; 3 cycles
//  269     eprom_write(0X07E0,0,16,PAGE_1,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+24
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x7E0         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  270     
//  271     eprom_read(0X0C70,0,PAGE_2,AUTO_CALC);
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x1            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xC70         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
//  272     long_into_char_array4(bill_energy_import,&opr_data[0]); 
        MOVW      DE, #LWRD(_opr_data)  ;; 1 cycle
        MOVW      BC, N:_bill_energy_import+2  ;; 1 cycle
        MOVW      AX, N:_bill_energy_import  ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  273     long_into_char_array4(bill_reactive_energy_lag,&opr_data[4]);               //bill_reactive_energy_lag
        MOVW      DE, #LWRD(_opr_data+4)  ;; 1 cycle
        MOVW      BC, N:_bill_reactive_energy_lag+2  ;; 1 cycle
        MOVW      AX, N:_bill_reactive_energy_lag  ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  274     long_into_char_array4(bill_reactive_energy_lead,&opr_data[8]);              //bill_reactive_energy_lead
        MOVW      DE, #LWRD(_opr_data+8)  ;; 1 cycle
        MOVW      BC, N:_bill_reactive_energy_lead+2  ;; 1 cycle
        MOVW      AX, N:_bill_reactive_energy_lead  ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  275     long_into_char_array4(bill_apparent_energy,&opr_data[12]);                  //bill_apparent energy  
        MOVW      DE, #LWRD(_opr_data+12)  ;; 1 cycle
        MOVW      BC, N:_bill_apparent_energy+2  ;; 1 cycle
        MOVW      AX, N:_bill_apparent_energy  ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  276     int_into_char_array(bill_pf,&opr_data[16]);
        MOVW      BC, #LWRD(_opr_data+16)  ;; 1 cycle
        MOVW      AX, N:_bill_pf     ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  277     eprom_write(0X0C70,0,32,PAGE_2,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+26
        MOV       B, #0x1            ;; 1 cycle
        MOVW      DE, #0x20          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xC70         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  278 
//  279     /* ********************************* */
//  280     
//  281     
//  282     /* ********************************Reset Zone PF********* */
//  283    zone_pf= 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_zone_pf, AX     ;; 1 cycle
//  284   for(local_i= 0; local_i < 8; local_i++)
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP+0x0E], A       ;; 1 cycle
        ADDW      SP, #0xE           ;; 1 cycle
          CFI CFA SP+12
        ; ------------------------------------- Block: 164 cycles
??save_bill_data_9:
        MOV       A, [SP]            ;; 1 cycle
        CMP       A, #0x8            ;; 1 cycle
        BNC       ??bill_variable_init_12  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  285   {
//  286       eprom_read((TOD_CUR_BILL_BLK3*(0x10 * local_i)),0,PAGE_1,AUTO_CALC);
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
        MOV       A, [SP]            ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x7000        ;; 1 cycle
        MULHU                        ;; 2 cycles
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
//  287       opr_data[8]= 0;
        MOV       N:_opr_data+8, #0x0  ;; 1 cycle
//  288       opr_data[9]= 0;
        MOV       N:_opr_data+9, #0x0  ;; 1 cycle
//  289       eprom_write((TOD_CUR_BILL_BLK3*(0x10 * local_i)),0,16,PAGE_1,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+14
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        MOV       H, A               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        PUSH      DE                 ;; 1 cycle
          CFI CFA SP+16
        XCH       A, C               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        XCH       A, C               ;; 1 cycle
        MOV       A, [SP+0x04]       ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x7000        ;; 1 cycle
        MULHU                        ;; 2 cycles
        XCH       A, D               ;; 1 cycle
        MOV       C, A               ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        POP       DE                 ;; 1 cycle
          CFI CFA SP+14
        XCH       A, H               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, H               ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  290   }
        MOV       A, [SP+0x02]       ;; 1 cycle
        INC       A                  ;; 1 cycle
        MOV       [SP+0x02], A       ;; 1 cycle
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+12
        BR        S:??save_bill_data_9  ;; 3 cycles
        ; ------------------------------------- Block: 67 cycles
//  291 
//  292     GetNextDate(present_min, present_hr, present_date, present_month, present_year, bill_min, bill_hr, bill_date, 0);
??bill_variable_init_12:
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+14
        MOV       A, N:_bill_date    ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+16
        XCH       A, X               ;; 1 cycle
        MOV       A, N:_bill_hr      ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+18
        XCH       A, X               ;; 1 cycle
        MOV       A, N:_bill_min     ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, N:_Now+6        ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       B, N:_Now+5        ;; 1 cycle
        MOV       C, N:_Now+3        ;; 1 cycle
        MOV       X, N:_Now+2        ;; 1 cycle
        MOV       A, N:_Now+1        ;; 1 cycle
          CFI FunCall _GetNextDate
        CALL      _GetNextDate       ;; 3 cycles
//  293 
//  294     /* ***implemented in the end of save bill data function instead of starting**** */
//  295 
//  296     eprom_read(0x07D0,0,PAGE_1,AUTO_CALC);
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x7D0         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
//  297     opr_data[8]=bill_pom[0];
        MOV       A, N:_bill_pom     ;; 1 cycle
        MOV       N:_opr_data+8, A   ;; 1 cycle
//  298     opr_data[9]=bill_pom[1];
        MOV       A, N:_bill_pom+1   ;; 1 cycle
        MOV       N:_opr_data+9, A   ;; 1 cycle
//  299     opr_data[10]=bill_pom[2];   
        MOV       A, N:_bill_pom+2   ;; 1 cycle
        MOV       N:_opr_data+10, A  ;; 1 cycle
//  300     md_reset_ip_flag= 1;
        MOV       N:_md_reset_ip_flag, #0x1  ;; 1 cycle
//  301     opr_data[11]= md_reset_ip_flag;
        MOV       A, N:_md_reset_ip_flag  ;; 1 cycle
        MOV       N:_opr_data+11, A  ;; 1 cycle
//  302     eprom_write(0x07D0,0,16,PAGE_1,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+20
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x7D0         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  303     
//  304     bill_generated_successfully_f=1;
        MOV       N:_bill_generated_successfully_f, #0x1  ;; 1 cycle
//  305 }
        ADDW      SP, #0x10          ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock0
        ; ------------------------------------- Block: 54 cycles
        ; ------------------------------------- Total: 1184 cycles
//  306     
//  307     
//  308 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock1 Using cfiCommon0
          CFI Function _cal_bill_pf
        CODE
//  309 void cal_bill_pf(void)
//  310 {
_cal_bill_pf:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 24
        SUBW      SP, #0x18          ;; 1 cycle
          CFI CFA SP+28
//  311     us64 consumed_kwh,consumed_kvah;
//  312     if(bill_apparent_energy > energy.Allph.apparent_imp)
        MOVW      AX, N:_bill_apparent_energy+2  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+30
        MOVW      AX, N:_bill_apparent_energy  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
        MOVW      BC, N:_energy+70   ;; 1 cycle
        MOVW      AX, N:_energy+68   ;; 1 cycle
          CFI FunCall ?UL_CMP_L03
        CALL      N:?UL_CMP_L03      ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+28
        BNC       ??bill_variable_init_13  ;; 4 cycles
        ; ------------------------------------- Block: 15 cycles
//  313     {
//  314         consumed_kvah = energy.Allph.apparent_imp + (ROLL_OVER_LIMIT - bill_apparent_energy);
        MOVW      BC, N:_energy+70   ;; 1 cycle
        MOVW      AX, N:_energy+68   ;; 1 cycle
        SUBW      AX, #0xD800        ;; 1 cycle
        SKNC
        DECW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        SUBW      AX, #0x1194        ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SUBW      AX, N:_bill_apparent_energy  ;; 1 cycle
        SKNC
        DECW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        SUBW      AX, N:_bill_apparent_energy+2  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+30
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0xC           ;; 1 cycle
          CFI FunCall __L2LLU
        CALL      __L2LLU            ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+28
        BR        S:??bill_variable_init_14  ;; 3 cycles
        ; ------------------------------------- Block: 31 cycles
//  315     }
//  316     else
//  317     {
//  318         consumed_kvah= energy.Allph.apparent_imp - bill_apparent_energy;
??bill_variable_init_13:
        MOVW      BC, N:_energy+70   ;; 1 cycle
        MOVW      AX, N:_energy+68   ;; 1 cycle
        SUBW      AX, N:_bill_apparent_energy  ;; 1 cycle
        SKNC
        DECW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        SUBW      AX, N:_bill_apparent_energy+2  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+30
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0xC           ;; 1 cycle
          CFI FunCall __L2LLU
        CALL      __L2LLU            ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+28
        ; ------------------------------------- Block: 19 cycles
//  319     }
//  320 
//  321 
//  322     if(bill_energy_import > energy.Allph.active_imp)
??bill_variable_init_14:
        MOVW      AX, N:_bill_energy_import+2  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+30
        MOVW      AX, N:_bill_energy_import  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
        MOVW      BC, N:_energy+54   ;; 1 cycle
        MOVW      AX, N:_energy+52   ;; 1 cycle
          CFI FunCall ?UL_CMP_L03
        CALL      N:?UL_CMP_L03      ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+28
        BNC       ??bill_variable_init_15  ;; 4 cycles
        ; ------------------------------------- Block: 14 cycles
//  323     {
//  324         consumed_kwh= energy.Allph.active_imp + (ROLL_OVER_LIMIT - bill_energy_import);
        MOVW      BC, N:_energy+54   ;; 1 cycle
        MOVW      AX, N:_energy+52   ;; 1 cycle
        SUBW      AX, #0xD800        ;; 1 cycle
        SKNC
        DECW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        SUBW      AX, #0x1194        ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SUBW      AX, N:_bill_energy_import  ;; 1 cycle
        SKNC
        DECW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        SUBW      AX, N:_bill_energy_import+2  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+30
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
          CFI FunCall __L2LLU
        CALL      __L2LLU            ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+28
        BR        S:??bill_variable_init_16  ;; 3 cycles
        ; ------------------------------------- Block: 31 cycles
//  325     }
//  326     else
//  327     {
//  328         consumed_kwh= energy.Allph.active_imp - bill_energy_import;
??bill_variable_init_15:
        MOVW      BC, N:_energy+54   ;; 1 cycle
        MOVW      AX, N:_energy+52   ;; 1 cycle
        SUBW      AX, N:_bill_energy_import  ;; 1 cycle
        SKNC
        DECW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        SUBW      AX, N:_bill_energy_import+2  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+30
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
          CFI FunCall __L2LLU
        CALL      __L2LLU            ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+28
        ; ------------------------------------- Block: 19 cycles
//  329     }
//  330     consumed_kwh*=1000;
??bill_variable_init_16:
        MOVW      DE, #LWRD(__Constant_3e8_0)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Mul64
        CALL      __Mul64            ;; 3 cycles
//  331 
//  332     if(consumed_kvah != 0)
        MOVW      BC, #LWRD(__Constant_0_0)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
          CFI FunCall __CmpNe64
        CALL      __CmpNe64          ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BZ        ??bill_variable_init_17  ;; 4 cycles
        ; ------------------------------------- Block: 18 cycles
//  333     {
//  334         current_bill_pf=(uint16_t) (consumed_kwh / consumed_kvah);
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [HL]           ;; 1 cycle
        MOVW      N:_current_bill_pf, AX  ;; 1 cycle
        BR        S:??bill_variable_init_18  ;; 3 cycles
        ; ------------------------------------- Block: 18 cycles
//  335     }
//  336     else
//  337     {
//  338         current_bill_pf= 1000;
??bill_variable_init_17:
        MOVW      AX, #0x3E8         ;; 1 cycle
        MOVW      N:_current_bill_pf, AX  ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
//  339     }
//  340 
//  341     if(current_bill_pf > 1000)
??bill_variable_init_18:
        MOVW      AX, N:_current_bill_pf  ;; 1 cycle
        CMPW      AX, #0x3E9         ;; 1 cycle
        BC        ??bill_variable_init_19  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  342     {
//  343         current_bill_pf= 999;
        MOVW      AX, #0x3E7         ;; 1 cycle
        MOVW      N:_current_bill_pf, AX  ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
//  344     }
//  345 }
??bill_variable_init_19:
        ADDW      SP, #0x18          ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock1
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 182 cycles
//  346 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock2 Using cfiCommon0
          CFI Function _check_bill
        CODE
//  347 void check_bill(void)
//  348 {
_check_bill:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  349     eprom_read(0X07B0,0,PAGE_1,AUTO_CALC);
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x7B0         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
//  350     if(present_year > opr_data[9])
        MOV       A, N:_opr_data+9   ;; 1 cycle
        CMP       A, N:_Now+6        ;; 1 cycle
        BNC       ??bill_variable_init_20  ;; 4 cycles
        ; ------------------------------------- Block: 13 cycles
//  351     {
//  352         temp1_flag= 1;
        MOV       N:_temp1_flag, #0x1  ;; 1 cycle
        RET                          ;; 6 cycles
        ; ------------------------------------- Block: 7 cycles
//  353     }
//  354     else if((present_month > opr_data[8]) && (present_year == opr_data[9]))
??bill_variable_init_20:
        MOV       A, N:_opr_data+8   ;; 1 cycle
        CMP       A, N:_Now+5        ;; 1 cycle
        BNC       ??bill_variable_init_21  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOV       A, N:_Now+6        ;; 1 cycle
        CMP       A, N:_opr_data+9   ;; 1 cycle
        BNZ       ??bill_variable_init_21  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  355     {
//  356         temp1_flag= 1;
        MOV       N:_temp1_flag, #0x1  ;; 1 cycle
        RET                          ;; 6 cycles
        ; ------------------------------------- Block: 7 cycles
//  357     }
//  358     else if((present_date > opr_data[7]) && (present_month == opr_data[8]) && (present_year == opr_data[9]))
??bill_variable_init_21:
        MOV       A, N:_opr_data+7   ;; 1 cycle
        CMP       A, N:_Now+3        ;; 1 cycle
        BNC       ??bill_variable_init_22  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOV       A, N:_Now+5        ;; 1 cycle
        CMP       A, N:_opr_data+8   ;; 1 cycle
        BNZ       ??bill_variable_init_22  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOV       A, N:_Now+6        ;; 1 cycle
        CMP       A, N:_opr_data+9   ;; 1 cycle
        BNZ       ??bill_variable_init_22  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  359     {
//  360         temp1_flag= 1;
        MOV       N:_temp1_flag, #0x1  ;; 1 cycle
        RET                          ;; 6 cycles
        ; ------------------------------------- Block: 7 cycles
//  361     }
//  362     else if((present_hr > bill_hr) && (present_date == opr_data[7]) && (present_month == opr_data[8]) && (present_year == opr_data[9]))
??bill_variable_init_22:
        MOV       A, N:_bill_hr      ;; 1 cycle
        CMP       A, N:_Now+2        ;; 1 cycle
        BNC       ??bill_variable_init_23  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOV       A, N:_Now+3        ;; 1 cycle
        CMP       A, N:_opr_data+7   ;; 1 cycle
        BNZ       ??bill_variable_init_23  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOV       A, N:_Now+5        ;; 1 cycle
        CMP       A, N:_opr_data+8   ;; 1 cycle
        BNZ       ??bill_variable_init_23  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOV       A, N:_Now+6        ;; 1 cycle
        CMP       A, N:_opr_data+9   ;; 1 cycle
        BNZ       ??bill_variable_init_23  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  363     {
//  364         temp1_flag= 1;
        MOV       N:_temp1_flag, #0x1  ;; 1 cycle
        RET                          ;; 6 cycles
        ; ------------------------------------- Block: 7 cycles
//  365     }
//  366     else if((present_min >= bill_min) && (present_hr == bill_hr) && (present_date == opr_data[7]) && (present_month == opr_data[8]) && (present_year == opr_data[9]))
??bill_variable_init_23:
        MOV       A, N:_Now+1        ;; 1 cycle
        CMP       A, N:_bill_min     ;; 1 cycle
        BC        ??bill_variable_init_24  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOV       A, N:_Now+2        ;; 1 cycle
        CMP       A, N:_bill_hr      ;; 1 cycle
        BNZ       ??bill_variable_init_24  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOV       A, N:_Now+3        ;; 1 cycle
        CMP       A, N:_opr_data+7   ;; 1 cycle
        BNZ       ??bill_variable_init_24  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOV       A, N:_Now+5        ;; 1 cycle
        CMP       A, N:_opr_data+8   ;; 1 cycle
        BNZ       ??bill_variable_init_24  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOV       A, N:_Now+6        ;; 1 cycle
        CMP       A, N:_opr_data+9   ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 3 cycles
//  367     {
//  368         temp1_flag= 1;
        MOV       N:_temp1_flag, #0x1  ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
//  369     }
//  370 }
??bill_variable_init_24:
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock2
        ; ------------------------------------- Block: 6 cycles
        ; ------------------------------------- Total: 129 cycles

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock3 Using cfiCommon1
          CFI Function _GetNextDate
        CODE
//  371 void GetNextDate(char tpMin, char tpHr, char tpDate, char tpMonth, char tpYear, char tbMin, char tbHr, char tbDate, char tEvenOdd)
//  372 {
_GetNextDate:
        ; * Stack frame (at entry) *
        ; Param size: 6
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+6
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+8
        PUSH      DE                 ;; 1 cycle
          CFI CFA SP+10
        ; Auto size: 10
        SUBW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+14
//  373     us8 nbDate, nbMonth, nbYear ;
//  374 
//  375     tpDate= bcd_to_decimal(tpDate);
        MOV       A, [SP+0x06]       ;; 1 cycle
          CFI FunCall _bcd_to_decimal
        CALL      _bcd_to_decimal    ;; 3 cycles
        MOV       [SP+0x06], A       ;; 1 cycle
//  376     tpMonth= bcd_to_decimal(tpMonth);
        MOV       A, [SP+0x07]       ;; 1 cycle
          CFI FunCall _bcd_to_decimal
        CALL      _bcd_to_decimal    ;; 3 cycles
        MOV       [SP+0x07], A       ;; 1 cycle
//  377     tpYear= bcd_to_decimal(tpYear);
        MOV       A, [SP+0x04]       ;; 1 cycle
          CFI FunCall _bcd_to_decimal
        CALL      _bcd_to_decimal    ;; 3 cycles
        MOV       [SP+0x04], A       ;; 1 cycle
//  378     tbDate= bcd_to_decimal(tbDate);
        MOV       A, [SP+0x10]       ;; 1 cycle
          CFI FunCall _bcd_to_decimal
        CALL      _bcd_to_decimal    ;; 3 cycles
        MOV       [SP+0x10], A       ;; 1 cycle
//  379     temp_bill_date=bcd_to_decimal(temp_bill_date);
        MOV       A, N:_temp_bill_date  ;; 1 cycle
          CFI FunCall _bcd_to_decimal
        CALL      _bcd_to_decimal    ;; 3 cycles
        MOV       N:_temp_bill_date, A  ;; 1 cycle
//  380 
//  381     if(tEvenOdd != 0)
        MOV       A, [SP+0x12]       ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??bill_variable_init_25  ;; 4 cycles
        ; ------------------------------------- Block: 35 cycles
//  382     {
//  383         if(tEvenOdd % 2 != tpMonth % 2)
        MOV       X, #0x2            ;; 1 cycle
        MOV       A, [SP+0x12]       ;; 1 cycle
          CFI FunCall ?UC_MOD_L01
        CALL      N:?UC_MOD_L01      ;; 3 cycles
        MOV       B, A               ;; 1 cycle
        MOV       X, #0x2            ;; 1 cycle
        MOV       A, [SP+0x07]       ;; 1 cycle
          CFI FunCall ?UC_MOD_L01
        CALL      N:?UC_MOD_L01      ;; 3 cycles
        MOV       C, A               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        CMP       A, C               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        BZ        ??bill_variable_init_26  ;; 4 cycles
        ; ------------------------------------- Block: 19 cycles
//  384         {
//  385             nbDate= tbDate;
        MOV       A, [SP+0x10]       ;; 1 cycle
        MOV       [SP+0x02], A       ;; 1 cycle
//  386             nbMonth= tpMonth + 1;
        MOV       A, [SP+0x07]       ;; 1 cycle
        INC       A                  ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  387             nbYear= tpYear;
        MOV       A, [SP+0x04]       ;; 1 cycle
        MOV       [SP+0x01], A       ;; 1 cycle
//  388             if(nbMonth > 12)
        MOV       A, [SP]            ;; 1 cycle
        CMP       A, #0xD            ;; 1 cycle
        SKNC                         ;; 4 cycles
        BR        N:??bill_variable_init_27  ;; 4 cycles
        ; ------------------------------------- Block: 13 cycles
//  389             {
//  390                 nbYear++;
        MOV       A, [SP+0x01]       ;; 1 cycle
        INC       A                  ;; 1 cycle
        MOV       [SP+0x01], A       ;; 1 cycle
//  391                 nbMonth-= 12;
        MOV       A, [SP]            ;; 1 cycle
        ADD       A, #0xF4           ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        BR        N:??bill_variable_init_27  ;; 3 cycles
        ; ------------------------------------- Block: 9 cycles
//  392             }
//  393         }
//  394         else
//  395         {
//  396             if((tpDate < tbDate) || ((tpDate == tbDate) && (tpHr < tbHr)) || ((tpDate == tbDate) && (tpHr == tbHr) && (tpMin < tbMin)))
??bill_variable_init_26:
        MOV       A, [SP+0x06]       ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, [SP+0x10]       ;; 1 cycle
        CMP       X, A               ;; 1 cycle
        BC        ??bill_variable_init_28  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
        MOV       A, [SP+0x06]       ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, [SP+0x10]       ;; 1 cycle
        CMP       X, A               ;; 1 cycle
        BNZ       ??bill_variable_init_29  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
        MOV       A, [SP+0x08]       ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, [SP+0x0E]       ;; 1 cycle
        CMP       X, A               ;; 1 cycle
        BC        ??bill_variable_init_28  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
??bill_variable_init_29:
        MOV       A, [SP+0x06]       ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, [SP+0x10]       ;; 1 cycle
        CMP       X, A               ;; 1 cycle
        BNZ       ??bill_variable_init_30  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
        MOV       A, [SP+0x08]       ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, [SP+0x0E]       ;; 1 cycle
        CMP       X, A               ;; 1 cycle
        BNZ       ??bill_variable_init_30  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
        MOV       A, [SP+0x09]       ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, [SP+0x05]       ;; 1 cycle
        CMP       X, A               ;; 1 cycle
        BNC       ??bill_variable_init_30  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
//  397             {
//  398                 nbMonth= tpMonth;
??bill_variable_init_28:
        MOV       A, [SP+0x07]       ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  399                 nbDate= tbDate;
        MOV       A, [SP+0x10]       ;; 1 cycle
        MOV       [SP+0x02], A       ;; 1 cycle
//  400                 nbYear= tpYear;
        MOV       A, [SP+0x04]       ;; 1 cycle
        MOV       [SP+0x01], A       ;; 1 cycle
        BR        N:??bill_variable_init_27  ;; 3 cycles
        ; ------------------------------------- Block: 9 cycles
//  401             }
//  402             else
//  403             {
//  404                 nbMonth= tpMonth + 2;
??bill_variable_init_30:
        MOV       A, [SP+0x07]       ;; 1 cycle
        ADD       A, #0x2            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  405                 nbDate= tbDate;
        MOV       A, [SP+0x10]       ;; 1 cycle
        MOV       [SP+0x02], A       ;; 1 cycle
//  406                 nbYear= tpYear;
        MOV       A, [SP+0x04]       ;; 1 cycle
        MOV       [SP+0x01], A       ;; 1 cycle
//  407                 if(nbMonth > 12)
        MOV       A, [SP]            ;; 1 cycle
        CMP       A, #0xD            ;; 1 cycle
        SKNC                         ;; 4 cycles
        BR        N:??bill_variable_init_27  ;; 4 cycles
        ; ------------------------------------- Block: 13 cycles
//  408                 {
//  409                     nbYear++;
        MOV       A, [SP+0x01]       ;; 1 cycle
        INC       A                  ;; 1 cycle
        MOV       [SP+0x01], A       ;; 1 cycle
//  410                     nbMonth-= 12;
        MOV       A, [SP]            ;; 1 cycle
        ADD       A, #0xF4           ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        BR        N:??bill_variable_init_27  ;; 3 cycles
        ; ------------------------------------- Block: 9 cycles
//  411                 }
//  412             }
//  413         }
//  414     }
//  415     else
//  416     {
//  417         if(((tpDate < temp_bill_date)) || ((tpDate == temp_bill_date) && (tpHr < tbHr)) 
//  418         || ((tpDate == temp_bill_date) && (tpHr == tbHr) && (tpMin < tbMin)))
??bill_variable_init_25:
        MOV       A, [SP+0x06]       ;; 1 cycle
        CMP       A, N:_temp_bill_date  ;; 1 cycle
        BC        ??bill_variable_init_31  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOV       A, [SP+0x06]       ;; 1 cycle
        CMP       A, N:_temp_bill_date  ;; 1 cycle
        BNZ       ??bill_variable_init_32  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOV       A, [SP+0x08]       ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, [SP+0x0E]       ;; 1 cycle
        CMP       X, A               ;; 1 cycle
        BC        ??bill_variable_init_31  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
??bill_variable_init_32:
        MOV       A, [SP+0x06]       ;; 1 cycle
        CMP       A, N:_temp_bill_date  ;; 1 cycle
        SKZ                          ;; 4 cycles
        BR        N:??bill_variable_init_33  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOV       A, [SP+0x08]       ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, [SP+0x0E]       ;; 1 cycle
        CMP       X, A               ;; 1 cycle
        SKZ                          ;; 4 cycles
        BR        N:??bill_variable_init_33  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
        MOV       A, [SP+0x09]       ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, [SP+0x05]       ;; 1 cycle
        CMP       X, A               ;; 1 cycle
        SKC                          ;; 4 cycles
        BR        N:??bill_variable_init_33  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
//  419         {
//  420             nbMonth= tpMonth;
??bill_variable_init_31:
        MOV       A, [SP+0x07]       ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  421             nbYear= tpYear;
        MOV       A, [SP+0x04]       ;; 1 cycle
        MOV       [SP+0x01], A       ;; 1 cycle
//  422             if((tpMonth==2)&&(tpDate==29)&&((tpHr > tbHr)||((tpHr == tbHr)&&(tpMin >= tbMin)))&&
//  423             ((tpYear%4==0) && (tpYear%100!=0))) //if time is exceed then bill time
        MOV       A, [SP+0x07]       ;; 1 cycle
        CMP       A, #0x2            ;; 1 cycle
        BNZ       ??bill_variable_init_34  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
        MOV       A, [SP+0x06]       ;; 1 cycle
        CMP       A, #0x1D           ;; 1 cycle
        BNZ       ??bill_variable_init_34  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOV       A, [SP+0x0E]       ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, [SP+0x08]       ;; 1 cycle
        CMP       X, A               ;; 1 cycle
        BC        ??bill_variable_init_35  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
        MOV       A, [SP+0x08]       ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, [SP+0x0E]       ;; 1 cycle
        CMP       X, A               ;; 1 cycle
        BNZ       ??bill_variable_init_34  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
        MOV       A, [SP+0x09]       ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, [SP+0x05]       ;; 1 cycle
        CMP       X, A               ;; 1 cycle
        BC        ??bill_variable_init_34  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
??bill_variable_init_35:
        MOV       X, #0x4            ;; 1 cycle
        MOV       A, [SP+0x04]       ;; 1 cycle
          CFI FunCall ?UC_MOD_L01
        CALL      N:?UC_MOD_L01      ;; 3 cycles
        MOV       B, A               ;; 1 cycle
        CMP0      B                  ;; 1 cycle
        BNZ       ??bill_variable_init_34  ;; 4 cycles
        ; ------------------------------------- Block: 11 cycles
        MOV       X, #0x64           ;; 1 cycle
        MOV       A, [SP+0x04]       ;; 1 cycle
          CFI FunCall ?UC_MOD_L01
        CALL      N:?UC_MOD_L01      ;; 3 cycles
        MOV       B, A               ;; 1 cycle
        CMP0      B                  ;; 1 cycle
        BZ        ??bill_variable_init_34  ;; 4 cycles
        ; ------------------------------------- Block: 11 cycles
//  424             {
//  425              nbMonth= tpMonth+1;
        MOV       A, [SP+0x07]       ;; 1 cycle
        INC       A                  ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        BR        N:??bill_variable_init_36  ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
//  426             }
//  427             else if((tpMonth==2)&&(tpDate==28)&&((tpHr > tbHr)||((tpHr == tbHr)&&(tpMin >=tbMin)))&&
//  428             ((tpYear%4!=0) && (tpYear%100!=0)))
??bill_variable_init_34:
        MOV       A, [SP+0x07]       ;; 1 cycle
        CMP       A, #0x2            ;; 1 cycle
        BNZ       ??bill_variable_init_37  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOV       A, [SP+0x06]       ;; 1 cycle
        CMP       A, #0x1C           ;; 1 cycle
        BNZ       ??bill_variable_init_37  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOV       A, [SP+0x0E]       ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, [SP+0x08]       ;; 1 cycle
        CMP       X, A               ;; 1 cycle
        BC        ??bill_variable_init_38  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
        MOV       A, [SP+0x08]       ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, [SP+0x0E]       ;; 1 cycle
        CMP       X, A               ;; 1 cycle
        BNZ       ??bill_variable_init_37  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
        MOV       A, [SP+0x09]       ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, [SP+0x05]       ;; 1 cycle
        CMP       X, A               ;; 1 cycle
        BC        ??bill_variable_init_37  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
??bill_variable_init_38:
        MOV       X, #0x4            ;; 1 cycle
        MOV       A, [SP+0x04]       ;; 1 cycle
          CFI FunCall ?UC_MOD_L01
        CALL      N:?UC_MOD_L01      ;; 3 cycles
        MOV       B, A               ;; 1 cycle
        CMP0      B                  ;; 1 cycle
        BZ        ??bill_variable_init_37  ;; 4 cycles
        ; ------------------------------------- Block: 11 cycles
        MOV       X, #0x64           ;; 1 cycle
        MOV       A, [SP+0x04]       ;; 1 cycle
          CFI FunCall ?UC_MOD_L01
        CALL      N:?UC_MOD_L01      ;; 3 cycles
        MOV       B, A               ;; 1 cycle
        CMP0      B                  ;; 1 cycle
        BZ        ??bill_variable_init_37  ;; 4 cycles
        ; ------------------------------------- Block: 11 cycles
//  429             {
//  430              nbMonth= tpMonth+1;
        MOV       A, [SP+0x07]       ;; 1 cycle
        INC       A                  ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        BR        S:??bill_variable_init_36  ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
//  431             }
//  432             else if(((tpMonth==4)||(tpMonth==6)||(tpMonth==9)||(tpMonth==11))&&(tpDate==30)
//  433                      &&((tpHr > tbHr)||((tpHr == tbHr)&&(tpMin >=tbMin))))
??bill_variable_init_37:
        MOV       A, [SP+0x07]       ;; 1 cycle
        CMP       A, #0x4            ;; 1 cycle
        BZ        ??bill_variable_init_39  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOV       A, [SP+0x07]       ;; 1 cycle
        CMP       A, #0x6            ;; 1 cycle
        BZ        ??bill_variable_init_39  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOV       A, [SP+0x07]       ;; 1 cycle
        CMP       A, #0x9            ;; 1 cycle
        BZ        ??bill_variable_init_39  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOV       A, [SP+0x07]       ;; 1 cycle
        CMP       A, #0xB            ;; 1 cycle
        BNZ       ??bill_variable_init_36  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
??bill_variable_init_39:
        MOV       A, [SP+0x06]       ;; 1 cycle
        CMP       A, #0x1E           ;; 1 cycle
        BNZ       ??bill_variable_init_36  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOV       A, [SP+0x0E]       ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, [SP+0x08]       ;; 1 cycle
        CMP       X, A               ;; 1 cycle
        BC        ??bill_variable_init_40  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
        MOV       A, [SP+0x08]       ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, [SP+0x0E]       ;; 1 cycle
        CMP       X, A               ;; 1 cycle
        BNZ       ??bill_variable_init_36  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
        MOV       A, [SP+0x09]       ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, [SP+0x05]       ;; 1 cycle
        CMP       X, A               ;; 1 cycle
        BC        ??bill_variable_init_36  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
//  434             {
//  435              nbMonth= tpMonth+1;
??bill_variable_init_40:
        MOV       A, [SP+0x07]       ;; 1 cycle
        INC       A                  ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        BR        S:??bill_variable_init_36  ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
//  436             }
//  437         }
//  438         else
//  439         {
//  440             nbMonth= tpMonth + 1;
??bill_variable_init_33:
        MOV       A, [SP+0x07]       ;; 1 cycle
        INC       A                  ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  441             nbYear= tpYear;
        MOV       A, [SP+0x04]       ;; 1 cycle
        MOV       [SP+0x01], A       ;; 1 cycle
//  442             if(nbMonth > 12)
        MOV       A, [SP]            ;; 1 cycle
        CMP       A, #0xD            ;; 1 cycle
        BC        ??bill_variable_init_36  ;; 4 cycles
        ; ------------------------------------- Block: 11 cycles
//  443             {
//  444                 nbYear++;
        MOV       A, [SP+0x01]       ;; 1 cycle
        INC       A                  ;; 1 cycle
        MOV       [SP+0x01], A       ;; 1 cycle
//  445                 nbMonth-= 12;
        MOV       A, [SP]            ;; 1 cycle
        ADD       A, #0xF4           ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
//  446             }
//  447         }
//  448         
//  449                 /***********************/
//  450           if((nbMonth==2)&&(temp_bill_date>28))  //february month
??bill_variable_init_36:
        MOV       A, [SP]            ;; 1 cycle
        CMP       A, #0x2            ;; 1 cycle
        BNZ       ??bill_variable_init_41  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOV       A, N:_temp_bill_date  ;; 1 cycle
        CMP       A, #0x1D           ;; 1 cycle
        BC        ??bill_variable_init_41  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  451           {
//  452             if(((nbYear%4==0) && (nbYear%100!=0)) ||(nbYear%400==0)) //if leap year
        MOV       X, #0x4            ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
          CFI FunCall ?UC_MOD_L01
        CALL      N:?UC_MOD_L01      ;; 3 cycles
        MOV       B, A               ;; 1 cycle
        CMP0      B                  ;; 1 cycle
        BNZ       ??bill_variable_init_42  ;; 4 cycles
        ; ------------------------------------- Block: 11 cycles
        MOV       X, #0x64           ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
          CFI FunCall ?UC_MOD_L01
        CALL      N:?UC_MOD_L01      ;; 3 cycles
        MOV       B, A               ;; 1 cycle
        CMP0      B                  ;; 1 cycle
        BNZ       ??bill_variable_init_43  ;; 4 cycles
        ; ------------------------------------- Block: 11 cycles
??bill_variable_init_42:
        MOVW      BC, #0x190         ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall ?SI_MOD_L02
        CALL      N:?SI_MOD_L02      ;; 3 cycles
        MOVW      HL, AX             ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        CMPW      AX, #0x0           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        BNZ       ??bill_variable_init_44  ;; 4 cycles
        ; ------------------------------------- Block: 15 cycles
//  453             {
//  454               nbDate=29; //if max bill date is more than 29 then we will set it to 29 for leap year
??bill_variable_init_43:
        MOV       A, #0x1D           ;; 1 cycle
        MOV       [SP+0x02], A       ;; 1 cycle
        BR        S:??bill_variable_init_27  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  455             }
//  456             else
//  457             {
//  458               nbDate=28;  //non leap year  
??bill_variable_init_44:
        MOV       A, #0x1C           ;; 1 cycle
        MOV       [SP+0x02], A       ;; 1 cycle
        BR        S:??bill_variable_init_27  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  459             }
//  460           }
//  461           
//  462           else if(((nbMonth==4)||(nbMonth==6)||(nbMonth==9)||(nbMonth==11))
//  463                    &&(temp_bill_date>=30))//nbDate=31 if jump of 1 month , nbdate=30 to take care when jump of 2 month 
??bill_variable_init_41:
        MOV       A, [SP]            ;; 1 cycle
        CMP       A, #0x4            ;; 1 cycle
        BZ        ??bill_variable_init_45  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOV       A, [SP]            ;; 1 cycle
        CMP       A, #0x6            ;; 1 cycle
        BZ        ??bill_variable_init_45  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOV       A, [SP]            ;; 1 cycle
        CMP       A, #0x9            ;; 1 cycle
        BZ        ??bill_variable_init_45  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOV       A, [SP]            ;; 1 cycle
        CMP       A, #0xB            ;; 1 cycle
        BNZ       ??bill_variable_init_46  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
??bill_variable_init_45:
        MOV       A, N:_temp_bill_date  ;; 1 cycle
        CMP       A, #0x1E           ;; 1 cycle
        BC        ??bill_variable_init_46  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  464           {
//  465             nbDate=30;
        MOV       A, #0x1E           ;; 1 cycle
        MOV       [SP+0x02], A       ;; 1 cycle
        BR        S:??bill_variable_init_27  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  466           }
//  467           else //if(temp_bill_date==0x31)
//  468           {
//  469             nbDate=temp_bill_date;//31;
??bill_variable_init_46:
        MOV       A, N:_temp_bill_date  ;; 1 cycle
        MOV       [SP+0x02], A       ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
//  470           }
//  471 
//  472         /***********************/
//  473   
//  474     }
//  475     eprom_read(0X07B0,0,PAGE_1,AUTO_CALC);
??bill_variable_init_27:
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x7B0         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
//  476     opr_data[7]= decimal_to_bcd(nbDate);
        MOV       A, [SP+0x02]       ;; 1 cycle
          CFI FunCall _decimal_to_bcd
        CALL      _decimal_to_bcd    ;; 3 cycles
        MOV       N:_opr_data+7, A   ;; 1 cycle
//  477     opr_data[8]= decimal_to_bcd(nbMonth);
        MOV       A, [SP]            ;; 1 cycle
          CFI FunCall _decimal_to_bcd
        CALL      _decimal_to_bcd    ;; 3 cycles
        MOV       N:_opr_data+8, A   ;; 1 cycle
//  478     opr_data[9]= decimal_to_bcd(nbYear);
        MOV       A, [SP+0x01]       ;; 1 cycle
          CFI FunCall _decimal_to_bcd
        CALL      _decimal_to_bcd    ;; 3 cycles
        MOV       N:_opr_data+9, A   ;; 1 cycle
//  479     temp_bill_date=decimal_to_bcd(temp_bill_date);
        MOV       A, N:_temp_bill_date  ;; 1 cycle
          CFI FunCall _decimal_to_bcd
        CALL      _decimal_to_bcd    ;; 3 cycles
        MOV       N:_temp_bill_date, A  ;; 1 cycle
//  480     eprom_write(0X07B0,0,16,PAGE_1,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+16
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x7B0         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  481 
//  482     eprom_read(0x07F0,0,PAGE_1,AUTO_CALC);
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x7F0         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
//  483     bill_date=decimal_to_bcd(nbDate);
        MOV       A, [SP+0x04]       ;; 1 cycle
          CFI FunCall _decimal_to_bcd
        CALL      _decimal_to_bcd    ;; 3 cycles
        MOV       N:_bill_date, A    ;; 1 cycle
//  484     opr_data[2]=bill_date;
        MOV       A, N:_bill_date    ;; 1 cycle
        MOV       N:_opr_data+2, A   ;; 1 cycle
//  485     eprom_write(0x07F0,0,16,PAGE_1,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+18
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x7F0         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  486 
//  487 }
        ADDW      SP, #0xE           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock3
        ; ------------------------------------- Block: 66 cycles
        ; ------------------------------------- Total: 568 cycles
//  488 
//  489 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock4 Using cfiCommon1
          CFI Function _update_bill_tod_data
        CODE
//  490 void update_bill_tod_data(uint16_t read_address, uint16_t write_address,uint8_t bill_cnt)
//  491 {
_update_bill_tod_data:
        ; * Stack frame (at entry) *
        ; Param size: 0
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+6
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+8
        PUSH      DE                 ;; 1 cycle
          CFI CFA SP+10
        ; Auto size: 8
        SUBW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+12
//  492   us8 local_i  ;
//  493   temp_us32= write_address;
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        MOVW      S:_temp_us32, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_temp_us32+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  494     temp_us32= temp_us32 + (bill_cnt - 1) * 0x80;
        MOV       A, [SP+0x02]       ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        DECW      AX                 ;; 1 cycle
        MOVW      BC, #0x80          ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      HL, S:_temp_us32+2  ;; 1 cycle
        MOVW      DE, S:_temp_us32   ;; 1 cycle
        ADDW      AX, DE             ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_temp_us32, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_temp_us32+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  495 
//  496     for(local_i= 0; local_i < 8; local_i++)
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        ; ------------------------------------- Block: 37 cycles
??update_bill_tod_data_0:
        MOV       A, [SP]            ;; 1 cycle
        CMP       A, #0x8            ;; 1 cycle
        BNC       ??bill_variable_init_47  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  497     {
//  498         eprom_read((read_address+(16 * local_i)),0,PAGE_1,AUTO_CALC);                        
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        PUSH      DE                 ;; 1 cycle
          CFI CFA SP+14
        XCH       A, B               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        XCH       A, C               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        XCH       A, C               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x10          ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [SP+0x08]      ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       C, A               ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        XCH       A, E               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+12
        XCH       A, E               ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
//  499         
//  500         eprom_write(temp_us32,0,16,PAGE_1,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+14
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        PUSH      DE                 ;; 1 cycle
          CFI CFA SP+16
        MOVW      HL, S:_temp_us32+2  ;; 1 cycle
        MOVW      DE, S:_temp_us32   ;; 1 cycle
        MOVW      AX, DE             ;; 1 cycle
        POP       DE                 ;; 1 cycle
          CFI CFA SP+14
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  501         temp_us32+= 0x10;
        MOVW      BC, S:_temp_us32+2  ;; 1 cycle
        MOVW      AX, S:_temp_us32   ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_temp_us32, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_temp_us32+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  502         
//  503         delay_ms(1);
        MOVW      AX, #0x1           ;; 1 cycle
          CFI FunCall _delay_ms
        CALL      _delay_ms          ;; 3 cycles
//  504     }
        MOV       A, [SP+0x02]       ;; 1 cycle
        INC       A                  ;; 1 cycle
        MOV       [SP+0x02], A       ;; 1 cycle
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+12
        BR        S:??update_bill_tod_data_0  ;; 3 cycles
        ; ------------------------------------- Block: 69 cycles
//  505 }
??bill_variable_init_47:
        ADDW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock4
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 119 cycles
//  506 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock5 Using cfiCommon0
          CFI Function _bill_variable_init
        CODE
//  507 void bill_variable_init(void)
//  508 {
_bill_variable_init:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  509   
//  510     eprom_read(0x07E0,0,PAGE_1,AUTO_CALC);
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x7E0         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
//  511     md_reset_count= opr_data[2];
        MOV       A, N:_opr_data+2   ;; 1 cycle
        MOV       N:_md_reset_count, A  ;; 1 cycle
//  512     bill_count= opr_data[3];
        MOV       A, N:_opr_data+3   ;; 1 cycle
        MOV       N:_bill_count, A   ;; 1 cycle
//  513     bill_tpr_cnt= opr_data[4];
        MOV       A, N:_opr_data+4   ;; 1 cycle
        MOV       N:_bill_tpr_cnt, A  ;; 1 cycle
//  514     
//  515     eprom_read(0x07D0,0,PAGE_1,AUTO_CALC);
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x7D0         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
//  516     cum_max_demand_kw=char_array_to_long4(&opr_data[0]);
        MOVW      AX, #LWRD(_opr_data)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_cum_max_demand_kw, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_cum_max_demand_kw+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  517     cum_max_demand_kva=char_array_to_long4(&opr_data[4]);
        MOVW      AX, #LWRD(_opr_data+4)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_cum_max_demand_kva, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_cum_max_demand_kva+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  518     bill_pom[0]=opr_data[8];
        MOV       A, N:_opr_data+8   ;; 1 cycle
        MOV       N:_bill_pom, A     ;; 1 cycle
//  519     bill_pom[1]=opr_data[9];
        MOV       A, N:_opr_data+9   ;; 1 cycle
        MOV       N:_bill_pom+1, A   ;; 1 cycle
//  520     bill_pom[2]=opr_data[10];
        MOV       A, N:_opr_data+10  ;; 1 cycle
        MOV       N:_bill_pom+2, A   ;; 1 cycle
//  521     md_reset_ip_flag=opr_data[11];
        MOV       A, N:_opr_data+11  ;; 1 cycle
        MOV       N:_md_reset_ip_flag, A  ;; 1 cycle
//  522     
//  523     if(eprom_read(0x07F0,0,PAGE_1,AUTO_CALC) == EEP_OK)
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x7F0         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
        CMP       A, #0x1            ;; 1 cycle
        BNZ       ??bill_variable_init_48  ;; 4 cycles
        ; ------------------------------------- Block: 56 cycles
//  524     {
//  525         bill_date=opr_data[2];
        MOV       A, N:_opr_data+2   ;; 1 cycle
        MOV       N:_bill_date, A    ;; 1 cycle
//  526         bill_min= opr_data[3];
        MOV       A, N:_opr_data+3   ;; 1 cycle
        MOV       N:_bill_min, A     ;; 1 cycle
//  527         bill_hr= opr_data[4];
        MOV       A, N:_opr_data+4   ;; 1 cycle
        MOV       N:_bill_hr, A      ;; 1 cycle
//  528         temp_bill_date= opr_data[5];
        MOV       A, N:_opr_data+5   ;; 1 cycle
        MOV       N:_temp_bill_date, A  ;; 1 cycle
        BR        S:??bill_variable_init_49  ;; 3 cycles
        ; ------------------------------------- Block: 11 cycles
//  529     }
//  530     else 
//  531     {
//  532         bill_date = 1;
??bill_variable_init_48:
        MOV       N:_bill_date, #0x1  ;; 1 cycle
//  533         bill_min = 0;
        MOV       N:_bill_min, #0x0  ;; 1 cycle
//  534         bill_hr = 0;
        MOV       N:_bill_hr, #0x0   ;; 1 cycle
//  535         temp_bill_date = bill_date;
        MOV       A, N:_bill_date    ;; 1 cycle
        MOV       N:_temp_bill_date, A  ;; 1 cycle
        ; ------------------------------------- Block: 5 cycles
//  536     }
//  537     
//  538     if(bill_date==0 || bill_date>0x31) 
??bill_variable_init_49:
        CMP0      N:_bill_date       ;; 1 cycle
        BZ        ??bill_variable_init_50  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        MOV       A, N:_bill_date    ;; 1 cycle
        CMP       A, #0x32           ;; 1 cycle
        BC        ??bill_variable_init_51  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  539     {
//  540       bill_date=1;
??bill_variable_init_50:
        MOV       N:_bill_date, #0x1  ;; 1 cycle
//  541       temp_bill_date = bill_date;
        MOV       A, N:_bill_date    ;; 1 cycle
        MOV       N:_temp_bill_date, A  ;; 1 cycle
        ; ------------------------------------- Block: 3 cycles
//  542     }
//  543     if(bill_min>0x59)
??bill_variable_init_51:
        MOV       A, N:_bill_min     ;; 1 cycle
        CMP       A, #0x5A           ;; 1 cycle
        SKC                          ;; 1 cycle
        ; ------------------------------------- Block: 3 cycles
//  544     {
//  545       bill_min=0;
        MOV       N:_bill_min, #0x0  ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
//  546     }
//  547     if(bill_hr>0x23)
??bill_variable_init_52:
        MOV       A, N:_bill_hr      ;; 1 cycle
        CMP       A, #0x24           ;; 1 cycle
        SKC                          ;; 1 cycle
        ; ------------------------------------- Block: 3 cycles
//  548     {
//  549       bill_hr=0;
        MOV       N:_bill_hr, #0x0   ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
//  550     }
//  551     
//  552     if(eprom_read(0X0C70,0,PAGE_2,AUTO_CALC) == EEP_OK)
??bill_variable_init_53:
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x1            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xC70         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
        CMP       A, #0x1            ;; 1 cycle
        BNZ       ??bill_variable_init_54  ;; 4 cycles
        ; ------------------------------------- Block: 12 cycles
//  553     {
//  554         bill_energy_import = char_array_to_long4(&opr_data[0]); 
        MOVW      AX, #LWRD(_opr_data)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_bill_energy_import, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_bill_energy_import+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  555         bill_reactive_energy_lag = char_array_to_long4(&opr_data[4]); 
        MOVW      AX, #LWRD(_opr_data+4)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_bill_reactive_energy_lag, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_bill_reactive_energy_lag+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  556         bill_reactive_energy_lead = char_array_to_long4(&opr_data[8]); 
        MOVW      AX, #LWRD(_opr_data+8)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_bill_reactive_energy_lead, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_bill_reactive_energy_lead+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  557         bill_apparent_energy = char_array_to_long4(&opr_data[12]); 
        MOVW      AX, #LWRD(_opr_data+12)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_bill_apparent_energy, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_bill_apparent_energy+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  558         bill_pf = char_array_to_int(&opr_data[16]);
        MOVW      AX, #LWRD(_opr_data+16)  ;; 1 cycle
          CFI FunCall _char_array_to_int
        CALL      _char_array_to_int  ;; 3 cycles
        MOVW      N:_bill_pf, AX     ;; 1 cycle
        ; ------------------------------------- Block: 37 cycles
//  559     }
//  560     
//  561     
//  562 }
??bill_variable_init_54:
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock5
        ; ------------------------------------- Block: 6 cycles
        ; ------------------------------------- Total: 149 cycles

        SECTION `.constf`:FARCODE:REORDER:NOROOT(1)
        SECTION_TYPE SHT_PROGBITS, SHF_WRITE

        SECTION `.data`:DATA:REORDER:NOROOT(1)
        SECTION_GROUP __Constant_3e8_0
__Constant_3e8_0:
        DATA32
        DD 1'000, 0

        SECTION `.constf`:FARCODE:REORDER:NOROOT(1)
        SECTION_TYPE SHT_PROGBITS, SHF_WRITE

        SECTION `.data`:DATA:REORDER:NOROOT(1)
        SECTION_GROUP __Constant_0_0
__Constant_0_0:
        DATA32
        DD 0, 0

        SECTION `.iar_vfe_header`:DATA:NOALLOC:NOROOT(1)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
        DC32 0

        END
// 
//    70 bytes in section .bss
//    16 bytes in section .data
// 3'672 bytes in section .text
// 
// 3'672 bytes of CODE memory
//    70 bytes of DATA memory (+ 16 bytes shared)
//
//Errors: none
//Warnings: none
