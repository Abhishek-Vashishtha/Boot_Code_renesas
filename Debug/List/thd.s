///////////////////////////////////////////////////////////////////////////////
//
// IAR C/C++ Compiler V4.20.1.2260 for RL78               20/Dec/2020  22:39:08
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
//        BootCode\source_code\source_files\thd.c
//    Command line       =
//        -f C:\Users\laptop\AppData\Local\Temp\EW1963.tmp ("D:\Dheeraj\New
//        folder\0. GDEV72 - BootCode\source_code\source_files\thd.c" --core s3
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
//        D:\Dheeraj\New folder\0. GDEV72 - BootCode\Debug\List\thd.s
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

        EXTERN ?F_DIV
        EXTERN ?F_F2SL
        EXTERN ?F_F2UL
        EXTERN ?F_MUL
        EXTERN ?F_UL2F
        EXTERN ?SL_DIV_L03
        EXTERN _CalculateSinValue
        EXTERN __Add64
        EXTERN __CmpGes64
        EXTERN __Divs64
        EXTERN __L2LLS
        EXTERN __L2LLU
        EXTERN __LLS2F
        EXTERN __LLU2F
        EXTERN __LLU2L
        EXTERN __Mul64
        EXTERN __Neg64
        EXTERN __Sub64
        EXTERN _b_phase
        EXTERN _curr
        EXTERN _freq
        EXTERN _mac_union_2int
        EXTERN _mac_union_4int
        EXTERN _r_phase
        EXTERN _sqrt
        EXTERN _temp_s64
        EXTERN _temp_us64
        EXTERN _vol
        EXTERN _y_phase

        PUBLIC __A_MAC32SH
        PUBLIC __A_MAC32SL
        PUBLIC __A_MULBH
        PUBLIC __A_MULBL
        PUBLIC __A_MULC
        PUBLIC __A_MULR0
        PUBLIC __A_MULR1
        PUBLIC __A_MULR2
        PUBLIC __A_MULR3
        PUBLIC __Constant_0_0
        PUBLIC _cos_tab
        PUBLIC _curr_cos_acc
        PUBLIC _curr_cos_acc_temp
        PUBLIC _curr_rms_acc
        PUBLIC _curr_rms_acc_temp
        PUBLIC _curr_sin_acc
        PUBLIC _curr_sin_acc_temp
        PUBLIC _flag_thd1
        PUBLIC _flag_thd2
        PUBLIC _sin_tab
        PUBLIC _thd
        PUBLIC _thd_100ms_loop
        PUBLIC _thd_accumulation
        PUBLIC _thd_cntr_b_phase
        PUBLIC _thd_cntr_r_phase
        PUBLIC _thd_cntr_y_phase
        PUBLIC _thd_curr_distortion
        PUBLIC _thd_curr_rms1
        PUBLIC _thd_curr_rms2
        PUBLIC _thd_double
        PUBLIC _thd_five_sec_loop
        PUBLIC _thd_one_sec_loop
        PUBLIC _thd_process
        PUBLIC _thd_sample_cntr
        PUBLIC _thd_sample_cntr_temp
        PUBLIC _thd_var
        PUBLIC _thd_vol_distortion
        PUBLIC _thd_vol_rms1
        PUBLIC _thd_vol_rms2
        PUBLIC _vol_cos_acc
        PUBLIC _vol_cos_acc_temp
        PUBLIC _vol_rms_acc
        PUBLIC _vol_rms_acc_temp
        PUBLIC _vol_sin_acc
        PUBLIC _vol_sin_acc_temp
        
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
        
// D:\Dheeraj\New folder\0. GDEV72 - BootCode\source_code\source_files\thd.c
//    1 /***********************************************************************************************************************
//    2 * File Name       : thd.c
//    3 * Current Version : rev_01  
//    4 * Tool-Chain      : IAR Systems RL78
//    5 * Description     : 
//    6 * Creation Date   : 27-10-2020
//    7 * Company         : Genus Power Infrastructures Limited, Jaipur
//    8 * Author          : dheeraj.singhal
//    9 * Version History : rev_01 :
//   10 ***********************************************************************************************************************/
//   11 /************************************ Includes **************************************/
//   12 #include "thd.h"

        ASEGN `.bss.noinit`:DATA:NOROOT,0fff3cH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_MULBL
// __no_init union <unnamed>#58 volatile __sfr __no_bit_access _A_MULBL
__A_MULBL:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0fff3eH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_MULBH
// __no_init union <unnamed>#59 volatile __sfr __no_bit_access _A_MULBH
__A_MULBH:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f028cH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_MAC32SL
// __no_init union <unnamed>#501 volatile __no_bit_access _A_MAC32SL
__A_MAC32SL:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f028eH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_MAC32SH
// __no_init union <unnamed>#502 volatile __no_bit_access _A_MAC32SH
__A_MAC32SH:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0290H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_MULR0
// __no_init union <unnamed>#503 volatile __no_bit_access _A_MULR0
__A_MULR0:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0292H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_MULR1
// __no_init union <unnamed>#504 volatile __no_bit_access _A_MULR1
__A_MULR1:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0294H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_MULR2
// __no_init union <unnamed>#505 volatile __no_bit_access _A_MULR2
__A_MULR2:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0296H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_MULR3
// __no_init union <unnamed>#506 volatile __no_bit_access _A_MULR3
__A_MULR3:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f029aH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_MULC
// __no_init union <unnamed>#507 volatile _A_MULC
__A_MULC:
        DS 1
//   13 /************************************ Local Variables *****************************************/

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   14 flag_union flag_thd1,flag_thd2;
_flag_thd1:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_flag_thd2:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   15 THD thd;
_thd:
        DS 54

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   16 s32 vol_sin_acc,vol_cos_acc,vol_rms_acc;
_vol_sin_acc:
        DS 4

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_vol_cos_acc:
        DS 4

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_vol_rms_acc:
        DS 4

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   17 s32 vol_sin_acc_temp,vol_cos_acc_temp,vol_rms_acc_temp;
_vol_sin_acc_temp:
        DS 4

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_vol_cos_acc_temp:
        DS 4

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_vol_rms_acc_temp:
        DS 4

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   18 s64 curr_sin_acc,curr_cos_acc,curr_rms_acc;
_curr_sin_acc:
        DS 8

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_curr_cos_acc:
        DS 8

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_curr_rms_acc:
        DS 8

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   19 s64 curr_sin_acc_temp,curr_cos_acc_temp,curr_rms_acc_temp;
_curr_sin_acc_temp:
        DS 8

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_curr_cos_acc_temp:
        DS 8

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_curr_rms_acc_temp:
        DS 8
//   20 

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   21 s32 sin_tab,cos_tab;
_sin_tab:
        DS 4

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_cos_tab:
        DS 4

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   22 us16 thd_sample_cntr,thd_sample_cntr_temp;
_thd_sample_cntr:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_thd_sample_cntr_temp:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   23 us16 thd_vol_rms1,thd_vol_rms2;
_thd_vol_rms1:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_thd_vol_rms2:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   24 us32 thd_curr_rms1,thd_curr_rms2;
_thd_curr_rms1:
        DS 4

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_thd_curr_rms2:
        DS 4

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   25 double thd_var,thd_double;
_thd_var:
        DS 4

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_thd_double:
        DS 4

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   26 us8 thd_cntr_r_phase,thd_cntr_y_phase,thd_cntr_b_phase;
_thd_cntr_r_phase:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_thd_cntr_y_phase:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_thd_cntr_b_phase:
        DS 1
//   27 /************************************ Extern Variables *****************************************/
//   28 /************************************ Local Functions *******************************/
//   29 /************************************ Extern Functions ******************************/

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   30 us16 thd_vol_distortion,thd_curr_distortion;
_thd_vol_distortion:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_thd_curr_distortion:
        DS 2
//   31 void thd_accumulation();
//   32 void thd_process();
//   33 void thd_five_sec_loop();
//   34 void thd_one_sec_loop();
//   35 void thd_100ms_loop();
//   36 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock0 Using cfiCommon0
          CFI Function _thd_accumulation
        CODE
//   37 void thd_accumulation()
//   38 {
_thd_accumulation:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//   39     static us16 temp_us_16;
//   40     thd_sample_cntr_temp++;
        INCW      N:_thd_sample_cntr_temp  ;; 2 cycles
//   41 
//   42     /* 18us */
//   43     thd_double = thd_var*thd_sample_cntr_temp;
        MOVW      AX, N:_thd_var+2   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+6
        MOVW      AX, N:_thd_var     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+8
        MOVW      AX, N:_thd_sample_cntr_temp  ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
          CFI FunCall ?F_UL2F
        CALL      N:?F_UL2F          ;; 3 cycles
          CFI FunCall ?F_MUL
        CALL      N:?F_MUL           ;; 3 cycles
        MOVW      N:_thd_double, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_thd_double+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//   44     temp_us_16 = (us16)thd_double;
        MOVW      BC, N:_thd_double+2  ;; 1 cycle
        MOVW      AX, N:_thd_double  ;; 1 cycle
          CFI FunCall ?F_F2SL
        CALL      N:?F_F2SL          ;; 3 cycles
        MOVW      N:`thd_accumulation::temp_us_16`, AX  ;; 1 cycle
//   45     
//   46     sin_tab = CalculateSinValue(temp_us_16);
        MOVW      AX, N:`thd_accumulation::temp_us_16`  ;; 1 cycle
          CFI FunCall _CalculateSinValue
        CALL      _CalculateSinValue  ;; 3 cycles
        MOVW      N:_sin_tab, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_sin_tab+2, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//   47     //    temp_us_16 = (57600-(us16)thd_double)%46080;
//   48     if(temp_us_16 <= 11520)
        MOVW      AX, N:`thd_accumulation::temp_us_16`  ;; 1 cycle
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+4
        CMPW      AX, #0x2D01        ;; 1 cycle
        BNC       ??thd_five_sec_loop_0  ;; 4 cycles
        ; ------------------------------------- Block: 39 cycles
//   49     {
//   50         temp_us_16 = 11520-temp_us_16;
        MOVW      AX, #0x2D00        ;; 1 cycle
        SUBW      AX, N:`thd_accumulation::temp_us_16`  ;; 1 cycle
        MOVW      N:`thd_accumulation::temp_us_16`, AX  ;; 1 cycle
        BR        S:??thd_five_sec_loop_1  ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
//   51     }
//   52     else
//   53     {
//   54         temp_us_16 = 57600-temp_us_16;
??thd_five_sec_loop_0:
        MOVW      AX, #0xE100        ;; 1 cycle
        SUBW      AX, N:`thd_accumulation::temp_us_16`  ;; 1 cycle
        MOVW      N:`thd_accumulation::temp_us_16`, AX  ;; 1 cycle
        ; ------------------------------------- Block: 3 cycles
//   55     }
//   56     cos_tab = CalculateSinValue(temp_us_16);
??thd_five_sec_loop_1:
        MOVW      AX, N:`thd_accumulation::temp_us_16`  ;; 1 cycle
          CFI FunCall _CalculateSinValue
        CALL      _CalculateSinValue  ;; 3 cycles
        MOVW      N:_cos_tab, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_cos_tab+2, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//   57 
//   58     
//   59     
//   60     if(flag_thd_cal_r == 1)
        MOVW      HL, #LWRD(_flag_thd1)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        SKC                          ;; 4 cycles
        BR        N:??thd_five_sec_loop_2  ;; 4 cycles
        ; ------------------------------------- Block: 14 cycles
//   61     {
//   62         /* 11 us*/
//   63         HWMultiplierAccSigned(sin_tab, r_phase.vol.sample, vol_sin_acc_temp);
        MOV       0x29A, #0xC0       ;; 1 cycle
        MOVW      AX, N:_vol_sin_acc_temp+2  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+6
        MOVW      AX, N:_vol_sin_acc_temp  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+8
        MOVW      AX, #LWRD(_mac_union_4int)  ;; 1 cycle
          CFI FunCall __L2LLS
        CALL      __L2LLS            ;; 3 cycles
        MOVW      AX, N:_mac_union_4int  ;; 1 cycle
        MOVW      0x290, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+2  ;; 1 cycle
        MOVW      0x292, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+4  ;; 1 cycle
        MOVW      0x294, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+6  ;; 1 cycle
        MOVW      0x296, AX          ;; 1 cycle
        MOVW      BC, N:_sin_tab+2   ;; 1 cycle
        MOVW      AX, N:_sin_tab     ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0x28C, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0x28E, AX          ;; 1 cycle
        MOVW      AX, N:_r_phase+8   ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0xFFF3C, AX        ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0xFFF3E, AX        ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        MOVW      AX, 0x290          ;; 1 cycle
        MOVW      N:_mac_union_4int, AX  ;; 1 cycle
        MOVW      AX, 0x292          ;; 1 cycle
        MOVW      N:_mac_union_4int+2, AX  ;; 1 cycle
        MOVW      AX, 0x294          ;; 1 cycle
        MOVW      N:_mac_union_4int+4, AX  ;; 1 cycle
        MOVW      AX, 0x296          ;; 1 cycle
        MOVW      N:_mac_union_4int+6, AX  ;; 1 cycle
        MOVW      AX, #LWRD(_mac_union_4int)  ;; 1 cycle
          CFI FunCall __LLU2L
        CALL      __LLU2L            ;; 3 cycles
        MOVW      N:_vol_sin_acc_temp, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_vol_sin_acc_temp+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//   64         HWMultiplierAccSigned(cos_tab, r_phase.vol.sample, vol_cos_acc_temp);
        MOV       0x29A, #0xC0       ;; 1 cycle
        MOVW      AX, N:_vol_cos_acc_temp+2  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        MOVW      AX, N:_vol_cos_acc_temp  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      AX, #LWRD(_mac_union_4int)  ;; 1 cycle
          CFI FunCall __L2LLS
        CALL      __L2LLS            ;; 3 cycles
        MOVW      AX, N:_mac_union_4int  ;; 1 cycle
        MOVW      0x290, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+2  ;; 1 cycle
        MOVW      0x292, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+4  ;; 1 cycle
        MOVW      0x294, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+6  ;; 1 cycle
        MOVW      0x296, AX          ;; 1 cycle
        MOVW      BC, N:_cos_tab+2   ;; 1 cycle
        MOVW      AX, N:_cos_tab     ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0x28C, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0x28E, AX          ;; 1 cycle
        MOVW      AX, N:_r_phase+8   ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0xFFF3C, AX        ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0xFFF3E, AX        ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        MOVW      AX, 0x290          ;; 1 cycle
        MOVW      N:_mac_union_4int, AX  ;; 1 cycle
        MOVW      AX, 0x292          ;; 1 cycle
        MOVW      N:_mac_union_4int+2, AX  ;; 1 cycle
        MOVW      AX, 0x294          ;; 1 cycle
        MOVW      N:_mac_union_4int+4, AX  ;; 1 cycle
        MOVW      AX, 0x296          ;; 1 cycle
        MOVW      N:_mac_union_4int+6, AX  ;; 1 cycle
        MOVW      AX, #LWRD(_mac_union_4int)  ;; 1 cycle
          CFI FunCall __LLU2L
        CALL      __LLU2L            ;; 3 cycles
        MOVW      N:_vol_cos_acc_temp, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_vol_cos_acc_temp+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//   65         HWMultiplierAccSigned(r_phase.vol.sample, r_phase.vol.sample, vol_rms_acc_temp);
        MOV       0x29A, #0xC0       ;; 1 cycle
        MOVW      AX, N:_vol_rms_acc_temp+2  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+14
        MOVW      AX, N:_vol_rms_acc_temp  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+16
        MOVW      AX, #LWRD(_mac_union_4int)  ;; 1 cycle
          CFI FunCall __L2LLS
        CALL      __L2LLS            ;; 3 cycles
        MOVW      AX, N:_mac_union_4int  ;; 1 cycle
        MOVW      0x290, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+2  ;; 1 cycle
        MOVW      0x292, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+4  ;; 1 cycle
        MOVW      0x294, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+6  ;; 1 cycle
        MOVW      0x296, AX          ;; 1 cycle
        MOVW      AX, N:_r_phase+8   ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0x28C, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0x28E, AX          ;; 1 cycle
        MOVW      AX, N:_r_phase+8   ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0xFFF3C, AX        ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0xFFF3E, AX        ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        MOVW      AX, 0x290          ;; 1 cycle
        MOVW      N:_mac_union_4int, AX  ;; 1 cycle
        MOVW      AX, 0x292          ;; 1 cycle
        MOVW      N:_mac_union_4int+2, AX  ;; 1 cycle
        MOVW      AX, 0x294          ;; 1 cycle
        MOVW      N:_mac_union_4int+4, AX  ;; 1 cycle
        MOVW      AX, 0x296          ;; 1 cycle
        MOVW      N:_mac_union_4int+6, AX  ;; 1 cycle
        MOVW      AX, #LWRD(_mac_union_4int)  ;; 1 cycle
          CFI FunCall __LLU2L
        CALL      __LLU2L            ;; 3 cycles
        MOVW      N:_vol_rms_acc_temp, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_vol_rms_acc_temp+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//   66         
//   67         /* 11 us */
//   68         HWMultiplierAccSigned(sin_tab, r_phase.curr.sample, curr_sin_acc_temp);
        MOV       0x29A, #0xC0       ;; 1 cycle
        MOVW      HL, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      DE, #LWRD(_curr_sin_acc_temp)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        MOVW      AX, N:_mac_union_4int  ;; 1 cycle
        MOVW      0x290, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+2  ;; 1 cycle
        MOVW      0x292, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+4  ;; 1 cycle
        MOVW      0x294, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+6  ;; 1 cycle
        MOVW      0x296, AX          ;; 1 cycle
        MOVW      BC, N:_sin_tab+2   ;; 1 cycle
        MOVW      AX, N:_sin_tab     ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0x28C, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0x28E, AX          ;; 1 cycle
        MOVW      AX, N:_r_phase+44  ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0xFFF3C, AX        ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0xFFF3E, AX        ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        MOVW      AX, 0x290          ;; 1 cycle
        MOVW      N:_mac_union_4int, AX  ;; 1 cycle
        MOVW      AX, 0x292          ;; 1 cycle
        MOVW      N:_mac_union_4int+2, AX  ;; 1 cycle
        MOVW      AX, 0x294          ;; 1 cycle
        MOVW      N:_mac_union_4int+4, AX  ;; 1 cycle
        MOVW      AX, 0x296          ;; 1 cycle
        MOVW      N:_mac_union_4int+6, AX  ;; 1 cycle
        MOVW      HL, #LWRD(_curr_sin_acc_temp)  ;; 1 cycle
        MOVW      DE, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//   69         HWMultiplierAccSigned(cos_tab, r_phase.curr.sample, curr_cos_acc_temp);
        MOV       0x29A, #0xC0       ;; 1 cycle
        MOVW      HL, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      DE, #LWRD(_curr_cos_acc_temp)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        MOVW      AX, N:_mac_union_4int  ;; 1 cycle
        MOVW      0x290, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+2  ;; 1 cycle
        MOVW      0x292, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+4  ;; 1 cycle
        MOVW      0x294, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+6  ;; 1 cycle
        MOVW      0x296, AX          ;; 1 cycle
        MOVW      BC, N:_cos_tab+2   ;; 1 cycle
        MOVW      AX, N:_cos_tab     ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0x28C, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0x28E, AX          ;; 1 cycle
        MOVW      AX, N:_r_phase+44  ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0xFFF3C, AX        ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0xFFF3E, AX        ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        MOVW      AX, 0x290          ;; 1 cycle
        MOVW      N:_mac_union_4int, AX  ;; 1 cycle
        MOVW      AX, 0x292          ;; 1 cycle
        MOVW      N:_mac_union_4int+2, AX  ;; 1 cycle
        MOVW      AX, 0x294          ;; 1 cycle
        MOVW      N:_mac_union_4int+4, AX  ;; 1 cycle
        MOVW      AX, 0x296          ;; 1 cycle
        MOVW      N:_mac_union_4int+6, AX  ;; 1 cycle
        MOVW      HL, #LWRD(_curr_cos_acc_temp)  ;; 1 cycle
        MOVW      DE, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//   70         HWMultiplierAccSigned(r_phase.curr.sample, r_phase.curr.sample, curr_rms_acc_temp);
        MOV       0x29A, #0xC0       ;; 1 cycle
        MOVW      HL, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      DE, #LWRD(_curr_rms_acc_temp)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        MOVW      AX, N:_mac_union_4int  ;; 1 cycle
        MOVW      0x290, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+2  ;; 1 cycle
        MOVW      0x292, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+4  ;; 1 cycle
        MOVW      0x294, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+6  ;; 1 cycle
        MOVW      0x296, AX          ;; 1 cycle
        MOVW      AX, N:_r_phase+44  ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0x28C, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0x28E, AX          ;; 1 cycle
        MOVW      AX, N:_r_phase+44  ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0xFFF3C, AX        ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0xFFF3E, AX        ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        MOVW      AX, 0x290          ;; 1 cycle
        MOVW      N:_mac_union_4int, AX  ;; 1 cycle
        MOVW      AX, 0x292          ;; 1 cycle
        MOVW      N:_mac_union_4int+2, AX  ;; 1 cycle
        MOVW      AX, 0x294          ;; 1 cycle
        MOVW      N:_mac_union_4int+4, AX  ;; 1 cycle
        MOVW      AX, 0x296          ;; 1 cycle
        MOVW      N:_mac_union_4int+6, AX  ;; 1 cycle
        MOVW      HL, #LWRD(_curr_rms_acc_temp)  ;; 1 cycle
        MOVW      DE, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        ADDW      SP, #0xC           ;; 1 cycle
          CFI CFA SP+4
        BR        N:??thd_five_sec_loop_3  ;; 3 cycles
        ; ------------------------------------- Block: 362 cycles
//   71     }
//   72     else if(flag_thd_cal_y == 1)
??thd_five_sec_loop_2:
        MOVW      HL, #LWRD(_flag_thd1)  ;; 1 cycle
        MOV1      CY, [HL].2         ;; 1 cycle
        SKC                          ;; 4 cycles
        BR        N:??thd_five_sec_loop_4  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//   73     {
//   74         /* 11 us*/
//   75         HWMultiplierAccSigned(sin_tab, y_phase.vol.sample, vol_sin_acc_temp);
        MOV       0x29A, #0xC0       ;; 1 cycle
        MOVW      AX, N:_vol_sin_acc_temp+2  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+6
        MOVW      AX, N:_vol_sin_acc_temp  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+8
        MOVW      AX, #LWRD(_mac_union_4int)  ;; 1 cycle
          CFI FunCall __L2LLS
        CALL      __L2LLS            ;; 3 cycles
        MOVW      AX, N:_mac_union_4int  ;; 1 cycle
        MOVW      0x290, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+2  ;; 1 cycle
        MOVW      0x292, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+4  ;; 1 cycle
        MOVW      0x294, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+6  ;; 1 cycle
        MOVW      0x296, AX          ;; 1 cycle
        MOVW      BC, N:_sin_tab+2   ;; 1 cycle
        MOVW      AX, N:_sin_tab     ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0x28C, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0x28E, AX          ;; 1 cycle
        MOVW      AX, N:_y_phase+8   ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0xFFF3C, AX        ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0xFFF3E, AX        ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        MOVW      AX, 0x290          ;; 1 cycle
        MOVW      N:_mac_union_4int, AX  ;; 1 cycle
        MOVW      AX, 0x292          ;; 1 cycle
        MOVW      N:_mac_union_4int+2, AX  ;; 1 cycle
        MOVW      AX, 0x294          ;; 1 cycle
        MOVW      N:_mac_union_4int+4, AX  ;; 1 cycle
        MOVW      AX, 0x296          ;; 1 cycle
        MOVW      N:_mac_union_4int+6, AX  ;; 1 cycle
        MOVW      AX, #LWRD(_mac_union_4int)  ;; 1 cycle
          CFI FunCall __LLU2L
        CALL      __LLU2L            ;; 3 cycles
        MOVW      N:_vol_sin_acc_temp, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_vol_sin_acc_temp+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//   76         HWMultiplierAccSigned(cos_tab, y_phase.vol.sample, vol_cos_acc_temp);
        MOV       0x29A, #0xC0       ;; 1 cycle
        MOVW      AX, N:_vol_cos_acc_temp+2  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        MOVW      AX, N:_vol_cos_acc_temp  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      AX, #LWRD(_mac_union_4int)  ;; 1 cycle
          CFI FunCall __L2LLS
        CALL      __L2LLS            ;; 3 cycles
        MOVW      AX, N:_mac_union_4int  ;; 1 cycle
        MOVW      0x290, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+2  ;; 1 cycle
        MOVW      0x292, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+4  ;; 1 cycle
        MOVW      0x294, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+6  ;; 1 cycle
        MOVW      0x296, AX          ;; 1 cycle
        MOVW      BC, N:_cos_tab+2   ;; 1 cycle
        MOVW      AX, N:_cos_tab     ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0x28C, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0x28E, AX          ;; 1 cycle
        MOVW      AX, N:_y_phase+8   ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0xFFF3C, AX        ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0xFFF3E, AX        ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        MOVW      AX, 0x290          ;; 1 cycle
        MOVW      N:_mac_union_4int, AX  ;; 1 cycle
        MOVW      AX, 0x292          ;; 1 cycle
        MOVW      N:_mac_union_4int+2, AX  ;; 1 cycle
        MOVW      AX, 0x294          ;; 1 cycle
        MOVW      N:_mac_union_4int+4, AX  ;; 1 cycle
        MOVW      AX, 0x296          ;; 1 cycle
        MOVW      N:_mac_union_4int+6, AX  ;; 1 cycle
        MOVW      AX, #LWRD(_mac_union_4int)  ;; 1 cycle
          CFI FunCall __LLU2L
        CALL      __LLU2L            ;; 3 cycles
        MOVW      N:_vol_cos_acc_temp, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_vol_cos_acc_temp+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//   77         HWMultiplierAccSigned(y_phase.vol.sample, y_phase.vol.sample, vol_rms_acc_temp);
        MOV       0x29A, #0xC0       ;; 1 cycle
        MOVW      AX, N:_vol_rms_acc_temp+2  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+14
        MOVW      AX, N:_vol_rms_acc_temp  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+16
        MOVW      AX, #LWRD(_mac_union_4int)  ;; 1 cycle
          CFI FunCall __L2LLS
        CALL      __L2LLS            ;; 3 cycles
        MOVW      AX, N:_mac_union_4int  ;; 1 cycle
        MOVW      0x290, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+2  ;; 1 cycle
        MOVW      0x292, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+4  ;; 1 cycle
        MOVW      0x294, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+6  ;; 1 cycle
        MOVW      0x296, AX          ;; 1 cycle
        MOVW      AX, N:_y_phase+8   ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0x28C, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0x28E, AX          ;; 1 cycle
        MOVW      AX, N:_y_phase+8   ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0xFFF3C, AX        ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0xFFF3E, AX        ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        MOVW      AX, 0x290          ;; 1 cycle
        MOVW      N:_mac_union_4int, AX  ;; 1 cycle
        MOVW      AX, 0x292          ;; 1 cycle
        MOVW      N:_mac_union_4int+2, AX  ;; 1 cycle
        MOVW      AX, 0x294          ;; 1 cycle
        MOVW      N:_mac_union_4int+4, AX  ;; 1 cycle
        MOVW      AX, 0x296          ;; 1 cycle
        MOVW      N:_mac_union_4int+6, AX  ;; 1 cycle
        MOVW      AX, #LWRD(_mac_union_4int)  ;; 1 cycle
          CFI FunCall __LLU2L
        CALL      __LLU2L            ;; 3 cycles
        MOVW      N:_vol_rms_acc_temp, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_vol_rms_acc_temp+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//   78         
//   79         /* 11 us */
//   80         HWMultiplierAccSigned(sin_tab, y_phase.curr.sample, curr_sin_acc_temp);
        MOV       0x29A, #0xC0       ;; 1 cycle
        MOVW      HL, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      DE, #LWRD(_curr_sin_acc_temp)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        MOVW      AX, N:_mac_union_4int  ;; 1 cycle
        MOVW      0x290, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+2  ;; 1 cycle
        MOVW      0x292, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+4  ;; 1 cycle
        MOVW      0x294, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+6  ;; 1 cycle
        MOVW      0x296, AX          ;; 1 cycle
        MOVW      BC, N:_sin_tab+2   ;; 1 cycle
        MOVW      AX, N:_sin_tab     ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0x28C, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0x28E, AX          ;; 1 cycle
        MOVW      AX, N:_y_phase+44  ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0xFFF3C, AX        ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0xFFF3E, AX        ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        MOVW      AX, 0x290          ;; 1 cycle
        MOVW      N:_mac_union_4int, AX  ;; 1 cycle
        MOVW      AX, 0x292          ;; 1 cycle
        MOVW      N:_mac_union_4int+2, AX  ;; 1 cycle
        MOVW      AX, 0x294          ;; 1 cycle
        MOVW      N:_mac_union_4int+4, AX  ;; 1 cycle
        MOVW      AX, 0x296          ;; 1 cycle
        MOVW      N:_mac_union_4int+6, AX  ;; 1 cycle
        MOVW      HL, #LWRD(_curr_sin_acc_temp)  ;; 1 cycle
        MOVW      DE, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//   81         HWMultiplierAccSigned(cos_tab, y_phase.curr.sample, curr_cos_acc_temp);
        MOV       0x29A, #0xC0       ;; 1 cycle
        MOVW      HL, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      DE, #LWRD(_curr_cos_acc_temp)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        MOVW      AX, N:_mac_union_4int  ;; 1 cycle
        MOVW      0x290, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+2  ;; 1 cycle
        MOVW      0x292, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+4  ;; 1 cycle
        MOVW      0x294, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+6  ;; 1 cycle
        MOVW      0x296, AX          ;; 1 cycle
        MOVW      BC, N:_cos_tab+2   ;; 1 cycle
        MOVW      AX, N:_cos_tab     ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0x28C, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0x28E, AX          ;; 1 cycle
        MOVW      AX, N:_y_phase+44  ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0xFFF3C, AX        ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0xFFF3E, AX        ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        MOVW      AX, 0x290          ;; 1 cycle
        MOVW      N:_mac_union_4int, AX  ;; 1 cycle
        MOVW      AX, 0x292          ;; 1 cycle
        MOVW      N:_mac_union_4int+2, AX  ;; 1 cycle
        MOVW      AX, 0x294          ;; 1 cycle
        MOVW      N:_mac_union_4int+4, AX  ;; 1 cycle
        MOVW      AX, 0x296          ;; 1 cycle
        MOVW      N:_mac_union_4int+6, AX  ;; 1 cycle
        MOVW      HL, #LWRD(_curr_cos_acc_temp)  ;; 1 cycle
        MOVW      DE, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//   82         HWMultiplierAccSigned(y_phase.curr.sample, y_phase.curr.sample, curr_rms_acc_temp);
        MOV       0x29A, #0xC0       ;; 1 cycle
        MOVW      HL, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      DE, #LWRD(_curr_rms_acc_temp)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        MOVW      AX, N:_mac_union_4int  ;; 1 cycle
        MOVW      0x290, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+2  ;; 1 cycle
        MOVW      0x292, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+4  ;; 1 cycle
        MOVW      0x294, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+6  ;; 1 cycle
        MOVW      0x296, AX          ;; 1 cycle
        MOVW      AX, N:_y_phase+44  ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0x28C, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0x28E, AX          ;; 1 cycle
        MOVW      AX, N:_y_phase+44  ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0xFFF3C, AX        ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0xFFF3E, AX        ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        MOVW      AX, 0x290          ;; 1 cycle
        MOVW      N:_mac_union_4int, AX  ;; 1 cycle
        MOVW      AX, 0x292          ;; 1 cycle
        MOVW      N:_mac_union_4int+2, AX  ;; 1 cycle
        MOVW      AX, 0x294          ;; 1 cycle
        MOVW      N:_mac_union_4int+4, AX  ;; 1 cycle
        MOVW      AX, 0x296          ;; 1 cycle
        MOVW      N:_mac_union_4int+6, AX  ;; 1 cycle
        MOVW      HL, #LWRD(_curr_rms_acc_temp)  ;; 1 cycle
        MOVW      DE, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        ADDW      SP, #0xC           ;; 1 cycle
          CFI CFA SP+4
        BR        N:??thd_five_sec_loop_3  ;; 3 cycles
        ; ------------------------------------- Block: 362 cycles
//   83     }
//   84     else if(flag_thd_cal_b == 1)
??thd_five_sec_loop_4:
        MOVW      HL, #LWRD(_flag_thd1)  ;; 1 cycle
        MOV1      CY, [HL].4         ;; 1 cycle
        SKC                          ;; 4 cycles
        BR        N:??thd_five_sec_loop_3  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//   85     {
//   86         /* 11 us*/
//   87         HWMultiplierAccSigned(sin_tab, b_phase.vol.sample, vol_sin_acc_temp);
        MOV       0x29A, #0xC0       ;; 1 cycle
        MOVW      AX, N:_vol_sin_acc_temp+2  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+6
        MOVW      AX, N:_vol_sin_acc_temp  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+8
        MOVW      AX, #LWRD(_mac_union_4int)  ;; 1 cycle
          CFI FunCall __L2LLS
        CALL      __L2LLS            ;; 3 cycles
        MOVW      AX, N:_mac_union_4int  ;; 1 cycle
        MOVW      0x290, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+2  ;; 1 cycle
        MOVW      0x292, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+4  ;; 1 cycle
        MOVW      0x294, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+6  ;; 1 cycle
        MOVW      0x296, AX          ;; 1 cycle
        MOVW      BC, N:_sin_tab+2   ;; 1 cycle
        MOVW      AX, N:_sin_tab     ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0x28C, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0x28E, AX          ;; 1 cycle
        MOVW      AX, N:_b_phase+8   ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0xFFF3C, AX        ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0xFFF3E, AX        ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        MOVW      AX, 0x290          ;; 1 cycle
        MOVW      N:_mac_union_4int, AX  ;; 1 cycle
        MOVW      AX, 0x292          ;; 1 cycle
        MOVW      N:_mac_union_4int+2, AX  ;; 1 cycle
        MOVW      AX, 0x294          ;; 1 cycle
        MOVW      N:_mac_union_4int+4, AX  ;; 1 cycle
        MOVW      AX, 0x296          ;; 1 cycle
        MOVW      N:_mac_union_4int+6, AX  ;; 1 cycle
        MOVW      AX, #LWRD(_mac_union_4int)  ;; 1 cycle
          CFI FunCall __LLU2L
        CALL      __LLU2L            ;; 3 cycles
        MOVW      N:_vol_sin_acc_temp, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_vol_sin_acc_temp+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//   88         HWMultiplierAccSigned(cos_tab, b_phase.vol.sample, vol_cos_acc_temp);
        MOV       0x29A, #0xC0       ;; 1 cycle
        MOVW      AX, N:_vol_cos_acc_temp+2  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        MOVW      AX, N:_vol_cos_acc_temp  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      AX, #LWRD(_mac_union_4int)  ;; 1 cycle
          CFI FunCall __L2LLS
        CALL      __L2LLS            ;; 3 cycles
        MOVW      AX, N:_mac_union_4int  ;; 1 cycle
        MOVW      0x290, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+2  ;; 1 cycle
        MOVW      0x292, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+4  ;; 1 cycle
        MOVW      0x294, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+6  ;; 1 cycle
        MOVW      0x296, AX          ;; 1 cycle
        MOVW      BC, N:_cos_tab+2   ;; 1 cycle
        MOVW      AX, N:_cos_tab     ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0x28C, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0x28E, AX          ;; 1 cycle
        MOVW      AX, N:_b_phase+8   ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0xFFF3C, AX        ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0xFFF3E, AX        ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        MOVW      AX, 0x290          ;; 1 cycle
        MOVW      N:_mac_union_4int, AX  ;; 1 cycle
        MOVW      AX, 0x292          ;; 1 cycle
        MOVW      N:_mac_union_4int+2, AX  ;; 1 cycle
        MOVW      AX, 0x294          ;; 1 cycle
        MOVW      N:_mac_union_4int+4, AX  ;; 1 cycle
        MOVW      AX, 0x296          ;; 1 cycle
        MOVW      N:_mac_union_4int+6, AX  ;; 1 cycle
        MOVW      AX, #LWRD(_mac_union_4int)  ;; 1 cycle
          CFI FunCall __LLU2L
        CALL      __LLU2L            ;; 3 cycles
        MOVW      N:_vol_cos_acc_temp, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_vol_cos_acc_temp+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//   89         HWMultiplierAccSigned(b_phase.vol.sample, b_phase.vol.sample, vol_rms_acc_temp);
        MOV       0x29A, #0xC0       ;; 1 cycle
        MOVW      AX, N:_vol_rms_acc_temp+2  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+14
        MOVW      AX, N:_vol_rms_acc_temp  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+16
        MOVW      AX, #LWRD(_mac_union_4int)  ;; 1 cycle
          CFI FunCall __L2LLS
        CALL      __L2LLS            ;; 3 cycles
        MOVW      AX, N:_mac_union_4int  ;; 1 cycle
        MOVW      0x290, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+2  ;; 1 cycle
        MOVW      0x292, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+4  ;; 1 cycle
        MOVW      0x294, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+6  ;; 1 cycle
        MOVW      0x296, AX          ;; 1 cycle
        MOVW      AX, N:_b_phase+8   ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0x28C, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0x28E, AX          ;; 1 cycle
        MOVW      AX, N:_b_phase+8   ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0xFFF3C, AX        ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0xFFF3E, AX        ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        MOVW      AX, 0x290          ;; 1 cycle
        MOVW      N:_mac_union_4int, AX  ;; 1 cycle
        MOVW      AX, 0x292          ;; 1 cycle
        MOVW      N:_mac_union_4int+2, AX  ;; 1 cycle
        MOVW      AX, 0x294          ;; 1 cycle
        MOVW      N:_mac_union_4int+4, AX  ;; 1 cycle
        MOVW      AX, 0x296          ;; 1 cycle
        MOVW      N:_mac_union_4int+6, AX  ;; 1 cycle
        MOVW      AX, #LWRD(_mac_union_4int)  ;; 1 cycle
          CFI FunCall __LLU2L
        CALL      __LLU2L            ;; 3 cycles
        MOVW      N:_vol_rms_acc_temp, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_vol_rms_acc_temp+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//   90         
//   91         /* 11 us */
//   92         HWMultiplierAccSigned(sin_tab, b_phase.curr.sample, curr_sin_acc_temp);
        MOV       0x29A, #0xC0       ;; 1 cycle
        MOVW      HL, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      DE, #LWRD(_curr_sin_acc_temp)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        MOVW      AX, N:_mac_union_4int  ;; 1 cycle
        MOVW      0x290, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+2  ;; 1 cycle
        MOVW      0x292, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+4  ;; 1 cycle
        MOVW      0x294, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+6  ;; 1 cycle
        MOVW      0x296, AX          ;; 1 cycle
        MOVW      BC, N:_sin_tab+2   ;; 1 cycle
        MOVW      AX, N:_sin_tab     ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0x28C, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0x28E, AX          ;; 1 cycle
        MOVW      AX, N:_b_phase+44  ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0xFFF3C, AX        ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0xFFF3E, AX        ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        MOVW      AX, 0x290          ;; 1 cycle
        MOVW      N:_mac_union_4int, AX  ;; 1 cycle
        MOVW      AX, 0x292          ;; 1 cycle
        MOVW      N:_mac_union_4int+2, AX  ;; 1 cycle
        MOVW      AX, 0x294          ;; 1 cycle
        MOVW      N:_mac_union_4int+4, AX  ;; 1 cycle
        MOVW      AX, 0x296          ;; 1 cycle
        MOVW      N:_mac_union_4int+6, AX  ;; 1 cycle
        MOVW      HL, #LWRD(_curr_sin_acc_temp)  ;; 1 cycle
        MOVW      DE, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//   93         HWMultiplierAccSigned(cos_tab, b_phase.curr.sample, curr_cos_acc_temp);
        MOV       0x29A, #0xC0       ;; 1 cycle
        MOVW      HL, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      DE, #LWRD(_curr_cos_acc_temp)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        MOVW      AX, N:_mac_union_4int  ;; 1 cycle
        MOVW      0x290, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+2  ;; 1 cycle
        MOVW      0x292, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+4  ;; 1 cycle
        MOVW      0x294, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+6  ;; 1 cycle
        MOVW      0x296, AX          ;; 1 cycle
        MOVW      BC, N:_cos_tab+2   ;; 1 cycle
        MOVW      AX, N:_cos_tab     ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0x28C, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0x28E, AX          ;; 1 cycle
        MOVW      AX, N:_b_phase+44  ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0xFFF3C, AX        ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0xFFF3E, AX        ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        MOVW      AX, 0x290          ;; 1 cycle
        MOVW      N:_mac_union_4int, AX  ;; 1 cycle
        MOVW      AX, 0x292          ;; 1 cycle
        MOVW      N:_mac_union_4int+2, AX  ;; 1 cycle
        MOVW      AX, 0x294          ;; 1 cycle
        MOVW      N:_mac_union_4int+4, AX  ;; 1 cycle
        MOVW      AX, 0x296          ;; 1 cycle
        MOVW      N:_mac_union_4int+6, AX  ;; 1 cycle
        MOVW      HL, #LWRD(_curr_cos_acc_temp)  ;; 1 cycle
        MOVW      DE, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//   94         HWMultiplierAccSigned(b_phase.curr.sample, b_phase.curr.sample, curr_rms_acc_temp);
        MOV       0x29A, #0xC0       ;; 1 cycle
        MOVW      HL, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      DE, #LWRD(_curr_rms_acc_temp)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        MOVW      AX, N:_mac_union_4int  ;; 1 cycle
        MOVW      0x290, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+2  ;; 1 cycle
        MOVW      0x292, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+4  ;; 1 cycle
        MOVW      0x294, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+6  ;; 1 cycle
        MOVW      0x296, AX          ;; 1 cycle
        MOVW      AX, N:_b_phase+44  ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0x28C, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0x28E, AX          ;; 1 cycle
        MOVW      AX, N:_b_phase+44  ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0xFFF3C, AX        ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0xFFF3E, AX        ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        MOVW      AX, 0x290          ;; 1 cycle
        MOVW      N:_mac_union_4int, AX  ;; 1 cycle
        MOVW      AX, 0x292          ;; 1 cycle
        MOVW      N:_mac_union_4int+2, AX  ;; 1 cycle
        MOVW      AX, 0x294          ;; 1 cycle
        MOVW      N:_mac_union_4int+4, AX  ;; 1 cycle
        MOVW      AX, 0x296          ;; 1 cycle
        MOVW      N:_mac_union_4int+6, AX  ;; 1 cycle
        MOVW      HL, #LWRD(_curr_rms_acc_temp)  ;; 1 cycle
        MOVW      DE, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        ADDW      SP, #0xC           ;; 1 cycle
          CFI CFA SP+4
        ; ------------------------------------- Block: 359 cycles
//   95     }
//   96     if(thd_sample_cntr_temp >= THR_MAX_SAMPLE)
??thd_five_sec_loop_3:
        MOVW      AX, N:_thd_sample_cntr_temp  ;; 1 cycle
        CMPW      AX, #0x4E          ;; 1 cycle
        SKNC                         ;; 4 cycles
        BR        N:??thd_five_sec_loop_5  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//   97     {
//   98         vol_sin_acc = vol_sin_acc_temp;
        MOVW      BC, N:_vol_sin_acc_temp+2  ;; 1 cycle
        MOVW      AX, N:_vol_sin_acc_temp  ;; 1 cycle
        MOVW      N:_vol_sin_acc, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_vol_sin_acc+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//   99         vol_cos_acc = vol_cos_acc_temp;
        MOVW      BC, N:_vol_cos_acc_temp+2  ;; 1 cycle
        MOVW      AX, N:_vol_cos_acc_temp  ;; 1 cycle
        MOVW      N:_vol_cos_acc, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_vol_cos_acc+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  100         vol_rms_acc = vol_rms_acc_temp;
        MOVW      BC, N:_vol_rms_acc_temp+2  ;; 1 cycle
        MOVW      AX, N:_vol_rms_acc_temp  ;; 1 cycle
        MOVW      N:_vol_rms_acc, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_vol_rms_acc+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  101         curr_sin_acc = curr_sin_acc_temp;
        MOVW      HL, #LWRD(_curr_sin_acc)  ;; 1 cycle
        MOVW      DE, #LWRD(_curr_sin_acc_temp)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  102         curr_cos_acc = curr_cos_acc_temp;
        MOVW      HL, #LWRD(_curr_cos_acc)  ;; 1 cycle
        MOVW      DE, #LWRD(_curr_cos_acc_temp)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  103         curr_rms_acc = curr_rms_acc_temp;
        MOVW      HL, #LWRD(_curr_rms_acc)  ;; 1 cycle
        MOVW      DE, #LWRD(_curr_rms_acc_temp)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  104 
//  105         vol_sin_acc_temp = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_vol_sin_acc_temp, AX  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_vol_sin_acc_temp+2, AX  ;; 1 cycle
//  106         vol_cos_acc_temp = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_vol_cos_acc_temp, AX  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_vol_cos_acc_temp+2, AX  ;; 1 cycle
//  107         vol_rms_acc_temp = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_vol_rms_acc_temp, AX  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_vol_rms_acc_temp+2, AX  ;; 1 cycle
//  108         curr_sin_acc_temp = 0;
        MOVW      HL, #LWRD(_curr_sin_acc_temp)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  109         curr_cos_acc_temp = 0;
        MOVW      HL, #LWRD(_curr_cos_acc_temp)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  110         curr_rms_acc_temp = 0;
        MOVW      HL, #LWRD(_curr_rms_acc_temp)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  111         
//  112         thd_sample_cntr = thd_sample_cntr_temp;
        MOVW      AX, N:_thd_sample_cntr_temp  ;; 1 cycle
        MOVW      N:_thd_sample_cntr, AX  ;; 1 cycle
//  113         thd_sample_cntr_temp = 0; 
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_thd_sample_cntr_temp, AX  ;; 1 cycle
//  114         
//  115         if(flag_thd_cal_r == 1)
        MOVW      HL, #LWRD(_flag_thd1)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??thd_five_sec_loop_6  ;; 4 cycles
        ; ------------------------------------- Block: 97 cycles
//  116         {
//  117           flag_thd_cal_r = 0;
        CLR1      N:_flag_thd1.0     ;; 2 cycles
//  118           flag_thd_process_r = 1;
        SET1      N:_flag_thd1.1     ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
//  119         }
//  120         
//  121         if(flag_thd_cal_y == 1)
??thd_five_sec_loop_6:
        MOVW      HL, #LWRD(_flag_thd1)  ;; 1 cycle
        MOV1      CY, [HL].2         ;; 1 cycle
        BNC       ??thd_five_sec_loop_7  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  122         {
//  123           flag_thd_cal_y = 0;
        CLR1      N:_flag_thd1.2     ;; 2 cycles
//  124           flag_thd_process_y = 1;
        SET1      N:_flag_thd1.3     ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
//  125         }
//  126         
//  127         if(flag_thd_cal_b == 1)
??thd_five_sec_loop_7:
        MOVW      HL, #LWRD(_flag_thd1)  ;; 1 cycle
        MOV1      CY, [HL].4         ;; 1 cycle
        BNC       ??thd_five_sec_loop_8  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  128         {
//  129           flag_thd_cal_b = 0;
        CLR1      N:_flag_thd1.4     ;; 2 cycles
//  130           flag_thd_process_b = 1;
        SET1      N:_flag_thd1.5     ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
//  131         }
//  132         flag_thd_start_sampling = 0;
??thd_five_sec_loop_8:
        CLR1      N:_flag_thd1.6     ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
//  133     }
//  134 }
??thd_five_sec_loop_5:
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock0
        ; ------------------------------------- Block: 6 cycles
        ; ------------------------------------- Total: 1292 cycles
        REQUIRE __A_MULC
        REQUIRE __A_MULR0
        REQUIRE __A_MULR1
        REQUIRE __A_MULR2
        REQUIRE __A_MULR3
        REQUIRE __A_MAC32SL
        REQUIRE __A_MAC32SH
        REQUIRE __A_MULBL
        REQUIRE __A_MULBH

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
`thd_accumulation::temp_us_16`:
        DS 2
//  135 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock1 Using cfiCommon0
          CFI Function _thd_process
        CODE
//  136 void thd_process()
//  137 {
_thd_process:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 24
        SUBW      SP, #0x18          ;; 1 cycle
          CFI CFA SP+28
//  138     static s64 thd_acc_sin1,thd_acc_cos1,thd_acc_rms;
//  139     static us16 cntr_temp;
//  140     
//  141     if(thd_sample_cntr != 0)
        CLRW      AX                 ;; 1 cycle
        CMPW      AX, N:_thd_sample_cntr  ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??thd_five_sec_loop_9  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  142     {
//  143         cntr_temp = thd_sample_cntr*64;
        MOVW      AX, N:_thd_sample_cntr  ;; 1 cycle
        MOVW      BC, #0x40          ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      N:`thd_process::cntr_temp`, AX  ;; 1 cycle
//  144         
//  145         /* Voltage calculations */
//  146         thd_acc_sin1 = vol_sin_acc / cntr_temp;
        MOVW      AX, N:`thd_process::cntr_temp`  ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+30
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
        MOVW      BC, N:_vol_sin_acc+2  ;; 1 cycle
        MOVW      AX, N:_vol_sin_acc  ;; 1 cycle
          CFI FunCall ?SL_DIV_L03
        CALL      N:?SL_DIV_L03      ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+28
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+30
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
        MOVW      AX, #LWRD(`thd_process::thd_acc_sin1`)  ;; 1 cycle
          CFI FunCall __L2LLS
        CALL      __L2LLS            ;; 3 cycles
//  147         thd_acc_cos1 = vol_cos_acc / cntr_temp;
        MOVW      AX, N:`thd_process::cntr_temp`  ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+34
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+36
        MOVW      BC, N:_vol_cos_acc+2  ;; 1 cycle
        MOVW      AX, N:_vol_cos_acc  ;; 1 cycle
          CFI FunCall ?SL_DIV_L03
        CALL      N:?SL_DIV_L03      ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+32
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+34
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+36
        MOVW      AX, #LWRD(`thd_process::thd_acc_cos1`)  ;; 1 cycle
          CFI FunCall __L2LLS
        CALL      __L2LLS            ;; 3 cycles
//  148         thd_acc_rms = vol_rms_acc / thd_sample_cntr;
        MOVW      AX, N:_thd_sample_cntr  ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+38
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+40
        MOVW      BC, N:_vol_rms_acc+2  ;; 1 cycle
        MOVW      AX, N:_vol_rms_acc  ;; 1 cycle
          CFI FunCall ?SL_DIV_L03
        CALL      N:?SL_DIV_L03      ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+36
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+38
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+40
        MOVW      AX, #LWRD(`thd_process::thd_acc_rms`)  ;; 1 cycle
          CFI FunCall __L2LLS
        CALL      __L2LLS            ;; 3 cycles
//  149         
//  150         temp_us64 = thd_acc_sin1 * thd_acc_sin1;
        MOVW      DE, #LWRD(`thd_process::thd_acc_sin1`)  ;; 1 cycle
        MOVW      BC, #LWRD(`thd_process::thd_acc_sin1`)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0xC           ;; 1 cycle
          CFI FunCall __Mul64
        CALL      __Mul64            ;; 3 cycles
        MOVW      HL, #LWRD(_temp_us64)  ;; 1 cycle
        ADDW      SP, #0xC           ;; 1 cycle
          CFI CFA SP+28
        MOVW      AX, SP             ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  151         temp_us64 += thd_acc_cos1 * thd_acc_cos1;
        MOVW      DE, #LWRD(`thd_process::thd_acc_cos1`)  ;; 1 cycle
        MOVW      BC, #LWRD(`thd_process::thd_acc_cos1`)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Mul64
        CALL      __Mul64            ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      BC, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __Add64
        CALL      __Add64            ;; 3 cycles
//  152         thd_vol_rms1 = (us16)(sqrt(temp_us64)*100/THD_DIVISOR);
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __LLU2F
        CALL      __LLU2F            ;; 3 cycles
        MOVW      HL, #0x4435        ;; 1 cycle
        PUSH      HL                 ;; 1 cycle
          CFI CFA SP+30
        MOVW      HL, #0x4F3         ;; 1 cycle
        PUSH      HL                 ;; 1 cycle
          CFI CFA SP+32
        MOVW      HL, #0x42C8        ;; 1 cycle
        PUSH      HL                 ;; 1 cycle
          CFI CFA SP+34
        MOVW      HL, #0x0           ;; 1 cycle
        PUSH      HL                 ;; 1 cycle
          CFI CFA SP+36
          CFI FunCall _sqrt
        CALL      _sqrt              ;; 3 cycles
          CFI FunCall ?F_MUL
        CALL      N:?F_MUL           ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+32
          CFI FunCall ?F_DIV
        CALL      N:?F_DIV           ;; 3 cycles
          CFI FunCall ?F_F2SL
        CALL      N:?F_F2SL          ;; 3 cycles
        MOVW      N:_thd_vol_rms1, AX  ;; 1 cycle
//  153         
//  154         thd_vol_rms2 = (us16)(sqrt(thd_acc_rms)*100);
        MOVW      AX, #LWRD(`thd_process::thd_acc_rms`)  ;; 1 cycle
          CFI FunCall __LLS2F
        CALL      __LLS2F            ;; 3 cycles
        MOVW      HL, #0x42C8        ;; 1 cycle
        PUSH      HL                 ;; 1 cycle
          CFI CFA SP+34
        MOVW      HL, #0x0           ;; 1 cycle
        PUSH      HL                 ;; 1 cycle
          CFI CFA SP+36
          CFI FunCall _sqrt
        CALL      _sqrt              ;; 3 cycles
          CFI FunCall ?F_MUL
        CALL      N:?F_MUL           ;; 3 cycles
          CFI FunCall ?F_F2SL
        CALL      N:?F_F2SL          ;; 3 cycles
        MOVW      N:_thd_vol_rms2, AX  ;; 1 cycle
//  155         
//  156         temp_s64 = (s64)thd_vol_rms2*thd_vol_rms2;
        MOVW      AX, N:_thd_vol_rms2  ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+38
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+40
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0xC           ;; 1 cycle
          CFI FunCall __L2LLU
        CALL      __L2LLU            ;; 3 cycles
        MOVW      AX, N:_thd_vol_rms2  ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+42
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+44
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x18          ;; 1 cycle
          CFI FunCall __L2LLU
        CALL      __L2LLU            ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x18          ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, #LWRD(_temp_s64)  ;; 1 cycle
          CFI FunCall __Mul64
        CALL      __Mul64            ;; 3 cycles
//  157         temp_s64 -= (s64)thd_vol_rms1*thd_vol_rms1;
        MOVW      AX, N:_thd_vol_rms1  ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+46
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+48
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x14          ;; 1 cycle
          CFI FunCall __L2LLU
        CALL      __L2LLU            ;; 3 cycles
        MOVW      AX, N:_thd_vol_rms1  ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+50
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+52
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x20          ;; 1 cycle
          CFI FunCall __L2LLU
        CALL      __L2LLU            ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x20          ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x18          ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x28          ;; 1 cycle
          CFI FunCall __Mul64
        CALL      __Mul64            ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x28          ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      BC, #LWRD(_temp_s64)  ;; 1 cycle
        MOVW      AX, #LWRD(_temp_s64)  ;; 1 cycle
          CFI FunCall __Sub64
        CALL      __Sub64            ;; 3 cycles
//  158         
//  159         if(thd_vol_rms1 != 0)
        ADDW      SP, #0x18          ;; 1 cycle
          CFI CFA SP+28
        CLRW      AX                 ;; 1 cycle
        CMPW      AX, N:_thd_vol_rms1  ;; 1 cycle
        BZ        ??thd_five_sec_loop_10  ;; 4 cycles
        ; ------------------------------------- Block: 201 cycles
//  160             thd_vol_distortion = (us16)(sqrt(ABS(temp_s64))*1000/thd_vol_rms1);
        MOVW      BC, #LWRD(__Constant_0_0)  ;; 1 cycle
        MOVW      AX, #LWRD(_temp_s64)  ;; 1 cycle
          CFI FunCall __CmpGes64
        CALL      __CmpGes64         ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??thd_five_sec_loop_11  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
        MOVW      BC, #LWRD(_temp_s64)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Neg64
        CALL      __Neg64            ;; 3 cycles
        BR        S:??thd_five_sec_loop_12  ;; 3 cycles
        ; ------------------------------------- Block: 8 cycles
??thd_five_sec_loop_11:
        MOVW      AX, SP             ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      DE, #LWRD(_temp_s64)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        ; ------------------------------------- Block: 11 cycles
??thd_five_sec_loop_12:
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __LLS2F
        CALL      __LLS2F            ;; 3 cycles
        MOVW      DE, AX             ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+30
        POP       HL                 ;; 1 cycle
          CFI CFA SP+28
        MOVW      AX, N:_thd_vol_rms1  ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
          CFI FunCall ?F_UL2F
        CALL      N:?F_UL2F          ;; 3 cycles
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+30
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
        MOVW      AX, #0x447A        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+34
        MOVW      AX, #0x0           ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+36
        MOVW      AX, DE             ;; 1 cycle
        PUSH      HL                 ;; 1 cycle
          CFI CFA SP+38
        POP       BC                 ;; 1 cycle
          CFI CFA SP+36
          CFI FunCall _sqrt
        CALL      _sqrt              ;; 3 cycles
          CFI FunCall ?F_MUL
        CALL      N:?F_MUL           ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+32
          CFI FunCall ?F_DIV
        CALL      N:?F_DIV           ;; 3 cycles
          CFI FunCall ?F_F2SL
        CALL      N:?F_F2SL          ;; 3 cycles
        MOVW      N:_thd_vol_distortion, AX  ;; 1 cycle
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+28
        BR        S:??thd_five_sec_loop_13  ;; 3 cycles
        ; ------------------------------------- Block: 39 cycles
//  161         else
//  162             thd_vol_distortion = 0;
??thd_five_sec_loop_10:
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_thd_vol_distortion, AX  ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
//  163         
//  164         /* Current calculations */
//  165         thd_acc_sin1 = curr_sin_acc / cntr_temp;
??thd_five_sec_loop_13:
        MOVW      AX, N:`thd_process::cntr_temp`  ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+30
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
          CFI FunCall __L2LLU
        CALL      __L2LLU            ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      BC, #LWRD(_curr_sin_acc)  ;; 1 cycle
        MOVW      AX, #LWRD(`thd_process::thd_acc_sin1`)  ;; 1 cycle
          CFI FunCall __Divs64
        CALL      __Divs64           ;; 3 cycles
//  166         thd_acc_cos1 = curr_cos_acc / cntr_temp;
        MOVW      AX, N:`thd_process::cntr_temp`  ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+34
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+36
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
          CFI FunCall __L2LLU
        CALL      __L2LLU            ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      BC, #LWRD(_curr_cos_acc)  ;; 1 cycle
        MOVW      AX, #LWRD(`thd_process::thd_acc_cos1`)  ;; 1 cycle
          CFI FunCall __Divs64
        CALL      __Divs64           ;; 3 cycles
//  167         thd_acc_rms = curr_rms_acc / thd_sample_cntr;
        MOVW      AX, N:_thd_sample_cntr  ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+38
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+40
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0xC           ;; 1 cycle
          CFI FunCall __L2LLU
        CALL      __L2LLU            ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0xC           ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      BC, #LWRD(_curr_rms_acc)  ;; 1 cycle
        MOVW      AX, #LWRD(`thd_process::thd_acc_rms`)  ;; 1 cycle
          CFI FunCall __Divs64
        CALL      __Divs64           ;; 3 cycles
//  168         
//  169         temp_us64 = thd_acc_sin1 * thd_acc_sin1;
        MOVW      DE, #LWRD(`thd_process::thd_acc_sin1`)  ;; 1 cycle
        MOVW      BC, #LWRD(`thd_process::thd_acc_sin1`)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0xC           ;; 1 cycle
          CFI FunCall __Mul64
        CALL      __Mul64            ;; 3 cycles
        MOVW      HL, #LWRD(_temp_us64)  ;; 1 cycle
        ADDW      SP, #0xC           ;; 1 cycle
          CFI CFA SP+28
        MOVW      AX, SP             ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  170         temp_us64 += thd_acc_cos1 * thd_acc_cos1;
        MOVW      DE, #LWRD(`thd_process::thd_acc_cos1`)  ;; 1 cycle
        MOVW      BC, #LWRD(`thd_process::thd_acc_cos1`)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Mul64
        CALL      __Mul64            ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      BC, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __Add64
        CALL      __Add64            ;; 3 cycles
//  171         thd_curr_rms1 = (us32)(sqrt(temp_us64)*100/THD_DIVISOR);
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __LLU2F
        CALL      __LLU2F            ;; 3 cycles
        MOVW      HL, #0x4435        ;; 1 cycle
        PUSH      HL                 ;; 1 cycle
          CFI CFA SP+30
        MOVW      HL, #0x4F3         ;; 1 cycle
        PUSH      HL                 ;; 1 cycle
          CFI CFA SP+32
        MOVW      HL, #0x42C8        ;; 1 cycle
        PUSH      HL                 ;; 1 cycle
          CFI CFA SP+34
        MOVW      HL, #0x0           ;; 1 cycle
        PUSH      HL                 ;; 1 cycle
          CFI CFA SP+36
          CFI FunCall _sqrt
        CALL      _sqrt              ;; 3 cycles
          CFI FunCall ?F_MUL
        CALL      N:?F_MUL           ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+32
          CFI FunCall ?F_DIV
        CALL      N:?F_DIV           ;; 3 cycles
          CFI FunCall ?F_F2UL
        CALL      N:?F_F2UL          ;; 3 cycles
        MOVW      N:_thd_curr_rms1, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_thd_curr_rms1+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  172         
//  173         thd_curr_rms2 = (us32)(sqrt(thd_acc_rms)*100);
        MOVW      AX, #LWRD(`thd_process::thd_acc_rms`)  ;; 1 cycle
          CFI FunCall __LLS2F
        CALL      __LLS2F            ;; 3 cycles
        MOVW      HL, #0x42C8        ;; 1 cycle
        PUSH      HL                 ;; 1 cycle
          CFI CFA SP+34
        MOVW      HL, #0x0           ;; 1 cycle
        PUSH      HL                 ;; 1 cycle
          CFI CFA SP+36
          CFI FunCall _sqrt
        CALL      _sqrt              ;; 3 cycles
          CFI FunCall ?F_MUL
        CALL      N:?F_MUL           ;; 3 cycles
          CFI FunCall ?F_F2UL
        CALL      N:?F_F2UL          ;; 3 cycles
        MOVW      N:_thd_curr_rms2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_thd_curr_rms2+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  174         
//  175         temp_s64 = (s64)thd_curr_rms2*thd_curr_rms2;
        MOVW      AX, N:_thd_curr_rms2+2  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+38
        MOVW      AX, N:_thd_curr_rms2  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+40
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0xC           ;; 1 cycle
          CFI FunCall __L2LLU
        CALL      __L2LLU            ;; 3 cycles
        MOVW      AX, N:_thd_curr_rms2+2  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+42
        MOVW      AX, N:_thd_curr_rms2  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+44
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x18          ;; 1 cycle
          CFI FunCall __L2LLU
        CALL      __L2LLU            ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x18          ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, #LWRD(_temp_s64)  ;; 1 cycle
          CFI FunCall __Mul64
        CALL      __Mul64            ;; 3 cycles
//  176         temp_s64 -= (s64)thd_curr_rms1*thd_curr_rms1;
        MOVW      AX, N:_thd_curr_rms1+2  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+46
        MOVW      AX, N:_thd_curr_rms1  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+48
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x14          ;; 1 cycle
          CFI FunCall __L2LLU
        CALL      __L2LLU            ;; 3 cycles
        MOVW      AX, N:_thd_curr_rms1+2  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+50
        MOVW      AX, N:_thd_curr_rms1  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+52
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x20          ;; 1 cycle
          CFI FunCall __L2LLU
        CALL      __L2LLU            ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x20          ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x18          ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x28          ;; 1 cycle
          CFI FunCall __Mul64
        CALL      __Mul64            ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x28          ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      BC, #LWRD(_temp_s64)  ;; 1 cycle
        MOVW      AX, #LWRD(_temp_s64)  ;; 1 cycle
          CFI FunCall __Sub64
        CALL      __Sub64            ;; 3 cycles
//  177         
//  178         if(thd_curr_rms1 != 0)
        MOVW      BC, N:_thd_curr_rms1+2  ;; 1 cycle
        MOVW      AX, N:_thd_curr_rms1  ;; 1 cycle
        ADDW      SP, #0x18          ;; 1 cycle
          CFI CFA SP+28
        OR        A, X               ;; 1 cycle
        OR        A, C               ;; 1 cycle
        OR        A, B               ;; 1 cycle
        BZ        ??thd_five_sec_loop_14  ;; 4 cycles
        ; ------------------------------------- Block: 208 cycles
//  179             thd_curr_distortion = (us16)(sqrt(ABS(temp_s64))*1000/thd_curr_rms1);
        MOVW      BC, #LWRD(__Constant_0_0)  ;; 1 cycle
        MOVW      AX, #LWRD(_temp_s64)  ;; 1 cycle
          CFI FunCall __CmpGes64
        CALL      __CmpGes64         ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??thd_five_sec_loop_15  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
        MOVW      BC, #LWRD(_temp_s64)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Neg64
        CALL      __Neg64            ;; 3 cycles
        BR        S:??thd_five_sec_loop_16  ;; 3 cycles
        ; ------------------------------------- Block: 8 cycles
??thd_five_sec_loop_15:
        MOVW      AX, SP             ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      DE, #LWRD(_temp_s64)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        ; ------------------------------------- Block: 11 cycles
??thd_five_sec_loop_16:
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __LLS2F
        CALL      __LLS2F            ;; 3 cycles
        MOVW      DE, AX             ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+30
        POP       HL                 ;; 1 cycle
          CFI CFA SP+28
        MOVW      BC, N:_thd_curr_rms1+2  ;; 1 cycle
        MOVW      AX, N:_thd_curr_rms1  ;; 1 cycle
          CFI FunCall ?F_UL2F
        CALL      N:?F_UL2F          ;; 3 cycles
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+30
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
        MOVW      AX, #0x447A        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+34
        MOVW      AX, #0x0           ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+36
        MOVW      AX, DE             ;; 1 cycle
        PUSH      HL                 ;; 1 cycle
          CFI CFA SP+38
        POP       BC                 ;; 1 cycle
          CFI CFA SP+36
          CFI FunCall _sqrt
        CALL      _sqrt              ;; 3 cycles
          CFI FunCall ?F_MUL
        CALL      N:?F_MUL           ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+32
          CFI FunCall ?F_DIV
        CALL      N:?F_DIV           ;; 3 cycles
          CFI FunCall ?F_F2SL
        CALL      N:?F_F2SL          ;; 3 cycles
        MOVW      N:_thd_curr_distortion, AX  ;; 1 cycle
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+28
        BR        S:??thd_five_sec_loop_17  ;; 3 cycles
        ; ------------------------------------- Block: 39 cycles
//  180         else
//  181             thd_curr_distortion = 0;
??thd_five_sec_loop_14:
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_thd_curr_distortion, AX  ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
//  182         
//  183         
//  184         if(flag_thd_process_r == 1)
??thd_five_sec_loop_17:
        MOVW      HL, #LWRD(_flag_thd1)  ;; 1 cycle
        MOV1      CY, [HL].1         ;; 1 cycle
        SKC                          ;; 4 cycles
        BR        N:??thd_five_sec_loop_18  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  185         {
//  186             thd.Rph.vol.value = thd_vol_distortion;
        MOVW      AX, N:_thd_vol_distortion  ;; 1 cycle
        MOVW      N:_thd+14, AX      ;; 1 cycle
//  187             thd.Rph.vol.rms_funda = thd_vol_rms1;
        MOVW      AX, N:_thd_vol_rms1  ;; 1 cycle
        MOVW      N:_thd+10, AX      ;; 1 cycle
//  188             thd.Rph.vol.rms_cycle = thd_vol_rms2;
        MOVW      AX, N:_thd_vol_rms2  ;; 1 cycle
        MOVW      N:_thd+12, AX      ;; 1 cycle
//  189        
//  190             thd.Rph.curr.value = thd_curr_distortion;
        MOVW      AX, N:_thd_curr_distortion  ;; 1 cycle
        MOVW      N:_thd+8, AX       ;; 1 cycle
//  191             thd.Rph.curr.rms_funda = thd_curr_rms1;
        MOVW      BC, N:_thd_curr_rms1+2  ;; 1 cycle
        MOVW      AX, N:_thd_curr_rms1  ;; 1 cycle
        MOVW      N:_thd, AX         ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_thd+2, AX       ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  192             thd.Rph.curr.rms_cycle = thd_curr_rms2;
        MOVW      BC, N:_thd_curr_rms2+2  ;; 1 cycle
        MOVW      AX, N:_thd_curr_rms2  ;; 1 cycle
        MOVW      N:_thd+4, AX       ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_thd+6, AX       ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  193             
//  194             if(THD_CHECK == 1)
//  195             {
//  196                 if(thd.Rph.vol.value >= 80 && thd.Rph.vol.value <= 120)
        MOVW      AX, N:_thd+14      ;; 1 cycle
        CMPW      AX, #0x50          ;; 1 cycle
        BC        ??thd_five_sec_loop_19  ;; 4 cycles
        ; ------------------------------------- Block: 26 cycles
        MOVW      AX, N:_thd+14      ;; 1 cycle
        CMPW      AX, #0x79          ;; 1 cycle
        BNC       ??thd_five_sec_loop_19  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  197                     thd.Rph.vol.value = 100;
        MOVW      AX, #0x64          ;; 1 cycle
        MOVW      N:_thd+14, AX      ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
//  198                 if(thd.Rph.curr.value >= 350 && thd.Rph.curr.value <= 450)
??thd_five_sec_loop_19:
        MOVW      AX, N:_thd+8       ;; 1 cycle
        CMPW      AX, #0x15E         ;; 1 cycle
        BC        ??thd_five_sec_loop_20  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, N:_thd+8       ;; 1 cycle
        CMPW      AX, #0x1C3         ;; 1 cycle
        BNC       ??thd_five_sec_loop_20  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  199                     thd.Rph.curr.value = 400;
        MOVW      AX, #0x190         ;; 1 cycle
        MOVW      N:_thd+8, AX       ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
//  200                 if(thd.Rph.vol.value < 50  || thd.Rph.vol.value >= 800)
??thd_five_sec_loop_20:
        MOVW      AX, N:_thd+14      ;; 1 cycle
        CMPW      AX, #0x32          ;; 1 cycle
        BC        ??thd_five_sec_loop_21  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, N:_thd+14      ;; 1 cycle
        CMPW      AX, #0x320         ;; 1 cycle
        BC        ??thd_five_sec_loop_22  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  201                     thd.Rph.vol.value = 0;
??thd_five_sec_loop_21:
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_thd+14, AX      ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
//  202                 if(thd.Rph.curr.value < 50 || thd.Rph.curr.value >= 800)
??thd_five_sec_loop_22:
        MOVW      AX, N:_thd+8       ;; 1 cycle
        CMPW      AX, #0x32          ;; 1 cycle
        BC        ??thd_five_sec_loop_23  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, N:_thd+8       ;; 1 cycle
        CMPW      AX, #0x320         ;; 1 cycle
        BC        ??thd_five_sec_loop_24  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  203                     thd.Rph.curr.value = 0;
??thd_five_sec_loop_23:
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_thd+8, AX       ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
//  204             }
//  205             
//  206             if(vol.Rph.rms < THD_VOL_THR || curr.Rph.rms < THD_CURR_THR)
??thd_five_sec_loop_24:
        MOVW      AX, N:_vol         ;; 1 cycle
        CMPW      AX, #0x1388        ;; 1 cycle
        BC        ??thd_five_sec_loop_25  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      BC, N:_curr+2      ;; 1 cycle
        MOVW      AX, N:_curr        ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
        CMPW      AX, #0xFA          ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??thd_process_0:
        BNC       ??thd_five_sec_loop_26  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
//  207             {
//  208                 thd.Rph.vol.value = 0;
??thd_five_sec_loop_25:
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_thd+14, AX      ;; 1 cycle
//  209                 thd.Rph.vol.rms_funda = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_thd+10, AX      ;; 1 cycle
//  210                 thd.Rph.vol.rms_cycle = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_thd+12, AX      ;; 1 cycle
//  211                 
//  212                 thd.Rph.curr.value = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_thd+8, AX       ;; 1 cycle
//  213                 thd.Rph.curr.rms_funda = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_thd, AX         ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_thd+2, AX       ;; 1 cycle
//  214                 thd.Rph.curr.rms_cycle = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_thd+4, AX       ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_thd+6, AX       ;; 1 cycle
//  215                 
//  216                 thd.Rph.correction = 0;
        MOV       N:_thd+16, #0x0    ;; 1 cycle
        ; ------------------------------------- Block: 17 cycles
//  217             }
//  218             
//  219             thd.Rph.correction_temp = ((us32)thd.Rph.vol.value*thd.Rph.curr.value)/1000;     /* 40% curr,10% vol => 400*100 = 40000/1000 = 40 = 4.0% */
??thd_five_sec_loop_26:
        MOVW      DE, #0x3E8         ;; 1 cycle
        MOVW      HL, #0x0           ;; 1 cycle
        MOVW      BC, N:_thd+8       ;; 1 cycle
        MOVW      AX, N:_thd+14      ;; 1 cycle
        MULHU                        ;; 2 cycles
        DIVWU                        ;; 17 cycles
        NOP                          ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       N:_thd+17, A       ;; 1 cycle
//  220             
//  221             if(thd.Rph.correction_temp != 0)
        CMP0      N:_thd+17          ;; 1 cycle
        BZ        ??thd_five_sec_loop_27  ;; 4 cycles
        ; ------------------------------------- Block: 31 cycles
//  222             {
//  223                 thd_cntr_r_f = 1;
        SET1      N:_flag_thd2.0     ;; 2 cycles
//  224                 if(thd_cntr_r_phase >= 10)
        MOV       A, N:_thd_cntr_r_phase  ;; 1 cycle
        CMP       A, #0xA            ;; 1 cycle
        BC        ??thd_five_sec_loop_28  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
//  225                 {
//  226                     thd.Rph.correction = thd.Rph.correction_temp;
        MOV       A, N:_thd+17       ;; 1 cycle
        MOV       N:_thd+16, A       ;; 1 cycle
//  227                     thd_cntr_r_phase = 0;
        MOV       N:_thd_cntr_r_phase, #0x0  ;; 1 cycle
        BR        S:??thd_five_sec_loop_28  ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
//  228                 }
//  229             }
//  230             else
//  231             {
//  232                 thd_cntr_r_phase = 0;
??thd_five_sec_loop_27:
        MOV       N:_thd_cntr_r_phase, #0x0  ;; 1 cycle
//  233                 thd.Rph.correction = 0;
        MOV       N:_thd+16, #0x0    ;; 1 cycle
//  234                 thd_cntr_r_f = 0;
        CLR1      N:_flag_thd2.0     ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
//  235             }
//  236            
//  237             flag_thd_process_r = 0;
??thd_five_sec_loop_28:
        CLR1      N:_flag_thd1.1     ;; 2 cycles
//  238             
//  239             if(vol.Yph.rms >= THD_VOL_THR && curr.Yph.rms >= THD_CURR_THR)
        MOVW      AX, N:_vol+6       ;; 1 cycle
        CMPW      AX, #0x1388        ;; 1 cycle
        BC        ??thd_five_sec_loop_29  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
        MOVW      BC, N:_curr+18     ;; 1 cycle
        MOVW      AX, N:_curr+16     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
        CMPW      AX, #0xFA          ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??thd_process_1:
        BC        ??thd_five_sec_loop_29  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
//  240             {
//  241                 flag_thd_cal_y = 1;
        SET1      N:_flag_thd1.2     ;; 2 cycles
        BR        S:??thd_five_sec_loop_18  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  242             }
//  243             else if(vol.Bph.rms >= THD_VOL_THR && curr.Bph.rms >= THD_CURR_THR)
??thd_five_sec_loop_29:
        MOVW      AX, N:_vol+12      ;; 1 cycle
        CMPW      AX, #0x1388        ;; 1 cycle
        BC        ??thd_five_sec_loop_30  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      BC, N:_curr+34     ;; 1 cycle
        MOVW      AX, N:_curr+32     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
        CMPW      AX, #0xFA          ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??thd_process_2:
        BC        ??thd_five_sec_loop_30  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
//  244             {
//  245                  flag_thd_cal_b = 1;
        SET1      N:_flag_thd1.4     ;; 2 cycles
        BR        S:??thd_five_sec_loop_18  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  246             }
//  247             else if(vol.Rph.rms >= THD_VOL_THR && curr.Rph.rms >= THD_CURR_THR)
??thd_five_sec_loop_30:
        MOVW      AX, N:_vol         ;; 1 cycle
        CMPW      AX, #0x1388        ;; 1 cycle
        BC        ??thd_five_sec_loop_31  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      BC, N:_curr+2      ;; 1 cycle
        MOVW      AX, N:_curr        ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
        CMPW      AX, #0xFA          ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??thd_process_3:
        BC        ??thd_five_sec_loop_31  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
//  248             {
//  249                  flag_thd_cal_r = 1;
        SET1      N:_flag_thd1.0     ;; 2 cycles
        BR        S:??thd_five_sec_loop_18  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  250             }
//  251             else
//  252             {
//  253               thd_cal_restart = 0;
??thd_five_sec_loop_31:
        CLR1      N:_flag_thd2.3     ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
//  254             }
//  255         }
//  256         
//  257         if(flag_thd_process_y == 1)
??thd_five_sec_loop_18:
        MOVW      HL, #LWRD(_flag_thd1)  ;; 1 cycle
        MOV1      CY, [HL].3         ;; 1 cycle
        SKC                          ;; 4 cycles
        BR        N:??thd_five_sec_loop_32  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  258         {
//  259             thd.Yph.vol.value = thd_vol_distortion;
        MOVW      AX, N:_thd_vol_distortion  ;; 1 cycle
        MOVW      N:_thd+32, AX      ;; 1 cycle
//  260             thd.Yph.vol.rms_funda = thd_vol_rms1;
        MOVW      AX, N:_thd_vol_rms1  ;; 1 cycle
        MOVW      N:_thd+28, AX      ;; 1 cycle
//  261             thd.Yph.vol.rms_cycle = thd_vol_rms2;
        MOVW      AX, N:_thd_vol_rms2  ;; 1 cycle
        MOVW      N:_thd+30, AX      ;; 1 cycle
//  262        
//  263             thd.Yph.curr.value = thd_curr_distortion;
        MOVW      AX, N:_thd_curr_distortion  ;; 1 cycle
        MOVW      N:_thd+26, AX      ;; 1 cycle
//  264             thd.Yph.curr.rms_funda = thd_curr_rms1;
        MOVW      BC, N:_thd_curr_rms1+2  ;; 1 cycle
        MOVW      AX, N:_thd_curr_rms1  ;; 1 cycle
        MOVW      N:_thd+18, AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_thd+20, AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  265             thd.Yph.curr.rms_cycle = thd_curr_rms2;
        MOVW      BC, N:_thd_curr_rms2+2  ;; 1 cycle
        MOVW      AX, N:_thd_curr_rms2  ;; 1 cycle
        MOVW      N:_thd+22, AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_thd+24, AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  266             
//  267             if(THD_CHECK == 1)
//  268             {
//  269                 if(thd.Yph.vol.value >= 80 && thd.Yph.vol.value <= 120)
        MOVW      AX, N:_thd+32      ;; 1 cycle
        CMPW      AX, #0x50          ;; 1 cycle
        BC        ??thd_five_sec_loop_33  ;; 4 cycles
        ; ------------------------------------- Block: 26 cycles
        MOVW      AX, N:_thd+32      ;; 1 cycle
        CMPW      AX, #0x79          ;; 1 cycle
        BNC       ??thd_five_sec_loop_33  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  270                     thd.Yph.vol.value = 100;
        MOVW      AX, #0x64          ;; 1 cycle
        MOVW      N:_thd+32, AX      ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
//  271                 if(thd.Yph.curr.value >= 350 && thd.Yph.curr.value <= 450)
??thd_five_sec_loop_33:
        MOVW      AX, N:_thd+26      ;; 1 cycle
        CMPW      AX, #0x15E         ;; 1 cycle
        BC        ??thd_five_sec_loop_34  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, N:_thd+26      ;; 1 cycle
        CMPW      AX, #0x1C3         ;; 1 cycle
        BNC       ??thd_five_sec_loop_34  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  272                     thd.Yph.curr.value = 400;
        MOVW      AX, #0x190         ;; 1 cycle
        MOVW      N:_thd+26, AX      ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
//  273                 if(thd.Yph.vol.value < 50  || thd.Yph.vol.value >= 800)
??thd_five_sec_loop_34:
        MOVW      AX, N:_thd+32      ;; 1 cycle
        CMPW      AX, #0x32          ;; 1 cycle
        BC        ??thd_five_sec_loop_35  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, N:_thd+32      ;; 1 cycle
        CMPW      AX, #0x320         ;; 1 cycle
        BC        ??thd_five_sec_loop_36  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  274                     thd.Yph.vol.value = 0;
??thd_five_sec_loop_35:
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_thd+32, AX      ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
//  275                 if(thd.Yph.curr.value < 50 || thd.Yph.curr.value >= 800)
??thd_five_sec_loop_36:
        MOVW      AX, N:_thd+26      ;; 1 cycle
        CMPW      AX, #0x32          ;; 1 cycle
        BC        ??thd_five_sec_loop_37  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, N:_thd+26      ;; 1 cycle
        CMPW      AX, #0x320         ;; 1 cycle
        BC        ??thd_five_sec_loop_38  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  276                     thd.Yph.curr.value = 0;
??thd_five_sec_loop_37:
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_thd+26, AX      ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
//  277             }
//  278             
//  279             if(vol.Yph.rms < THD_VOL_THR || curr.Yph.rms < THD_CURR_THR)
??thd_five_sec_loop_38:
        MOVW      AX, N:_vol+6       ;; 1 cycle
        CMPW      AX, #0x1388        ;; 1 cycle
        BC        ??thd_five_sec_loop_39  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      BC, N:_curr+18     ;; 1 cycle
        MOVW      AX, N:_curr+16     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
        CMPW      AX, #0xFA          ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??thd_process_4:
        BNC       ??thd_five_sec_loop_40  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
//  280             {
//  281                 thd.Yph.vol.value = 0;
??thd_five_sec_loop_39:
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_thd+32, AX      ;; 1 cycle
//  282                 thd.Yph.vol.rms_funda = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_thd+28, AX      ;; 1 cycle
//  283                 thd.Yph.vol.rms_cycle = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_thd+30, AX      ;; 1 cycle
//  284                 
//  285                 thd.Yph.curr.value = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_thd+26, AX      ;; 1 cycle
//  286                 thd.Yph.curr.rms_funda = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_thd+18, AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_thd+20, AX      ;; 1 cycle
//  287                 thd.Yph.curr.rms_cycle = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_thd+22, AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_thd+24, AX      ;; 1 cycle
//  288                 
//  289                 thd.Yph.correction = 0;
        MOV       N:_thd+34, #0x0    ;; 1 cycle
        ; ------------------------------------- Block: 17 cycles
//  290             }
//  291             
//  292             thd.Yph.correction_temp = ((us32)thd.Yph.vol.value*thd.Yph.curr.value)/1000;     /* 40% curr,10% vol => 400*100 = 40000/1000 = 40 = 4.0% */
??thd_five_sec_loop_40:
        MOVW      DE, #0x3E8         ;; 1 cycle
        MOVW      HL, #0x0           ;; 1 cycle
        MOVW      BC, N:_thd+26      ;; 1 cycle
        MOVW      AX, N:_thd+32      ;; 1 cycle
        MULHU                        ;; 2 cycles
        DIVWU                        ;; 17 cycles
        NOP                          ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       N:_thd+35, A       ;; 1 cycle
//  293             
//  294             if(thd.Yph.correction_temp != 0)
        CMP0      N:_thd+35          ;; 1 cycle
        BZ        ??thd_five_sec_loop_41  ;; 4 cycles
        ; ------------------------------------- Block: 31 cycles
//  295             {
//  296                 thd_cntr_y_f = 1;
        SET1      N:_flag_thd2.1     ;; 2 cycles
//  297                 if(thd_cntr_y_phase >= 10)
        MOV       A, N:_thd_cntr_y_phase  ;; 1 cycle
        CMP       A, #0xA            ;; 1 cycle
        BC        ??thd_five_sec_loop_42  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
//  298                 {
//  299                     thd.Yph.correction = thd.Yph.correction_temp;
        MOV       A, N:_thd+35       ;; 1 cycle
        MOV       N:_thd+34, A       ;; 1 cycle
//  300                     thd_cntr_y_phase = 0;
        MOV       N:_thd_cntr_y_phase, #0x0  ;; 1 cycle
        BR        S:??thd_five_sec_loop_42  ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
//  301                 }
//  302             }
//  303             else
//  304             {
//  305                 thd_cntr_y_phase = 0;
??thd_five_sec_loop_41:
        MOV       N:_thd_cntr_y_phase, #0x0  ;; 1 cycle
//  306                 thd.Yph.correction = 0;
        MOV       N:_thd+34, #0x0    ;; 1 cycle
//  307                 thd_cntr_y_f = 0;
        CLR1      N:_flag_thd2.1     ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
//  308             }
//  309             
//  310             flag_thd_process_y = 0;
??thd_five_sec_loop_42:
        CLR1      N:_flag_thd1.3     ;; 2 cycles
//  311             
//  312             if(vol.Bph.rms >= THD_VOL_THR && curr.Bph.rms >= THD_CURR_THR)
        MOVW      AX, N:_vol+12      ;; 1 cycle
        CMPW      AX, #0x1388        ;; 1 cycle
        BC        ??thd_five_sec_loop_43  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
        MOVW      BC, N:_curr+34     ;; 1 cycle
        MOVW      AX, N:_curr+32     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
        CMPW      AX, #0xFA          ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??thd_process_5:
        BC        ??thd_five_sec_loop_43  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
//  313             {
//  314                 flag_thd_cal_b = 1;
        SET1      N:_flag_thd1.4     ;; 2 cycles
        BR        S:??thd_five_sec_loop_32  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  315             }
//  316             else if(vol.Rph.rms >= THD_VOL_THR && curr.Rph.rms >= THD_CURR_THR)
??thd_five_sec_loop_43:
        MOVW      AX, N:_vol         ;; 1 cycle
        CMPW      AX, #0x1388        ;; 1 cycle
        BC        ??thd_five_sec_loop_44  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      BC, N:_curr+2      ;; 1 cycle
        MOVW      AX, N:_curr        ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
        CMPW      AX, #0xFA          ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??thd_process_6:
        BC        ??thd_five_sec_loop_44  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
//  317             {
//  318                 flag_thd_cal_r = 1;
        SET1      N:_flag_thd1.0     ;; 2 cycles
        BR        S:??thd_five_sec_loop_32  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  319             }
//  320             else if(vol.Yph.rms >= THD_VOL_THR && curr.Yph.rms >= THD_CURR_THR)
??thd_five_sec_loop_44:
        MOVW      AX, N:_vol+6       ;; 1 cycle
        CMPW      AX, #0x1388        ;; 1 cycle
        BC        ??thd_five_sec_loop_45  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      BC, N:_curr+18     ;; 1 cycle
        MOVW      AX, N:_curr+16     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
        CMPW      AX, #0xFA          ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??thd_process_7:
        BC        ??thd_five_sec_loop_45  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
//  321             {
//  322                 flag_thd_cal_y = 1;
        SET1      N:_flag_thd1.2     ;; 2 cycles
        BR        S:??thd_five_sec_loop_32  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  323             }
//  324             else
//  325             {
//  326               thd_cal_restart = 0;
??thd_five_sec_loop_45:
        CLR1      N:_flag_thd2.3     ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
//  327             }
//  328         }
//  329         
//  330         if(flag_thd_process_b == 1)
??thd_five_sec_loop_32:
        MOVW      HL, #LWRD(_flag_thd1)  ;; 1 cycle
        MOV1      CY, [HL].5         ;; 1 cycle
        SKC                          ;; 4 cycles
        BR        N:??thd_five_sec_loop_9  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  331         {
//  332             thd.Bph.vol.value = thd_vol_distortion;
        MOVW      AX, N:_thd_vol_distortion  ;; 1 cycle
        MOVW      N:_thd+50, AX      ;; 1 cycle
//  333             thd.Bph.vol.rms_funda = thd_vol_rms1;
        MOVW      AX, N:_thd_vol_rms1  ;; 1 cycle
        MOVW      N:_thd+46, AX      ;; 1 cycle
//  334             thd.Bph.vol.rms_cycle = thd_vol_rms2;
        MOVW      AX, N:_thd_vol_rms2  ;; 1 cycle
        MOVW      N:_thd+48, AX      ;; 1 cycle
//  335             
//  336             thd.Bph.curr.value = thd_curr_distortion;
        MOVW      AX, N:_thd_curr_distortion  ;; 1 cycle
        MOVW      N:_thd+44, AX      ;; 1 cycle
//  337             thd.Bph.curr.rms_funda = thd_curr_rms1;
        MOVW      BC, N:_thd_curr_rms1+2  ;; 1 cycle
        MOVW      AX, N:_thd_curr_rms1  ;; 1 cycle
        MOVW      N:_thd+36, AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_thd+38, AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  338             thd.Bph.curr.rms_cycle = thd_curr_rms2;
        MOVW      BC, N:_thd_curr_rms2+2  ;; 1 cycle
        MOVW      AX, N:_thd_curr_rms2  ;; 1 cycle
        MOVW      N:_thd+40, AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_thd+42, AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  339             
//  340             if(THD_CHECK == 1)
//  341             {
//  342                 if(thd.Bph.vol.value >= 80 && thd.Bph.vol.value <= 120)
        MOVW      AX, N:_thd+50      ;; 1 cycle
        CMPW      AX, #0x50          ;; 1 cycle
        BC        ??thd_five_sec_loop_46  ;; 4 cycles
        ; ------------------------------------- Block: 26 cycles
        MOVW      AX, N:_thd+50      ;; 1 cycle
        CMPW      AX, #0x79          ;; 1 cycle
        BNC       ??thd_five_sec_loop_46  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  343                     thd.Bph.vol.value = 100;
        MOVW      AX, #0x64          ;; 1 cycle
        MOVW      N:_thd+50, AX      ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
//  344                 if(thd.Bph.curr.value >= 350 && thd.Bph.curr.value <= 450)
??thd_five_sec_loop_46:
        MOVW      AX, N:_thd+44      ;; 1 cycle
        CMPW      AX, #0x15E         ;; 1 cycle
        BC        ??thd_five_sec_loop_47  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, N:_thd+44      ;; 1 cycle
        CMPW      AX, #0x1C3         ;; 1 cycle
        BNC       ??thd_five_sec_loop_47  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  345                     thd.Bph.curr.value = 400;
        MOVW      AX, #0x190         ;; 1 cycle
        MOVW      N:_thd+44, AX      ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
//  346                 if(thd.Bph.vol.value < 50  || thd.Bph.vol.value >= 800)
??thd_five_sec_loop_47:
        MOVW      AX, N:_thd+50      ;; 1 cycle
        CMPW      AX, #0x32          ;; 1 cycle
        BC        ??thd_five_sec_loop_48  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, N:_thd+50      ;; 1 cycle
        CMPW      AX, #0x320         ;; 1 cycle
        BC        ??thd_five_sec_loop_49  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  347                     thd.Bph.vol.value = 0;
??thd_five_sec_loop_48:
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_thd+50, AX      ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
//  348                 if(thd.Bph.curr.value < 50 || thd.Bph.curr.value >= 800)
??thd_five_sec_loop_49:
        MOVW      AX, N:_thd+44      ;; 1 cycle
        CMPW      AX, #0x32          ;; 1 cycle
        BC        ??thd_five_sec_loop_50  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, N:_thd+44      ;; 1 cycle
        CMPW      AX, #0x320         ;; 1 cycle
        BC        ??thd_five_sec_loop_51  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  349                     thd.Bph.curr.value = 0;
??thd_five_sec_loop_50:
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_thd+44, AX      ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
//  350             }
//  351             
//  352             if(vol.Bph.rms < THD_VOL_THR || curr.Bph.rms < THD_CURR_THR)
??thd_five_sec_loop_51:
        MOVW      AX, N:_vol+12      ;; 1 cycle
        CMPW      AX, #0x1388        ;; 1 cycle
        BC        ??thd_five_sec_loop_52  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      BC, N:_curr+34     ;; 1 cycle
        MOVW      AX, N:_curr+32     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
        CMPW      AX, #0xFA          ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??thd_process_8:
        BNC       ??thd_five_sec_loop_53  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
//  353             {
//  354                 thd.Bph.vol.value = 0;
??thd_five_sec_loop_52:
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_thd+50, AX      ;; 1 cycle
//  355                 thd.Bph.vol.rms_funda = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_thd+46, AX      ;; 1 cycle
//  356                 thd.Bph.vol.rms_cycle = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_thd+48, AX      ;; 1 cycle
//  357                 
//  358                 thd.Bph.curr.value = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_thd+44, AX      ;; 1 cycle
//  359                 thd.Bph.curr.rms_funda = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_thd+36, AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_thd+38, AX      ;; 1 cycle
//  360                 thd.Bph.curr.rms_cycle = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_thd+40, AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_thd+42, AX      ;; 1 cycle
//  361                 
//  362                 thd.Bph.correction = 0;
        MOV       N:_thd+52, #0x0    ;; 1 cycle
        ; ------------------------------------- Block: 17 cycles
//  363             }
//  364             
//  365             thd.Bph.correction_temp = ((us32)thd.Bph.vol.value*thd.Bph.curr.value)/1000;     /* 40% curr,10% vol => 400*100 = 40000/1000 = 40 = 4.0% */
??thd_five_sec_loop_53:
        MOVW      DE, #0x3E8         ;; 1 cycle
        MOVW      HL, #0x0           ;; 1 cycle
        MOVW      BC, N:_thd+44      ;; 1 cycle
        MOVW      AX, N:_thd+50      ;; 1 cycle
        MULHU                        ;; 2 cycles
        DIVWU                        ;; 17 cycles
        NOP                          ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       N:_thd+53, A       ;; 1 cycle
//  366             
//  367             if(thd.Bph.correction_temp != 0)
        CMP0      N:_thd+53          ;; 1 cycle
        BZ        ??thd_five_sec_loop_54  ;; 4 cycles
        ; ------------------------------------- Block: 31 cycles
//  368             {
//  369                 thd_cntr_b_f = 1;
        SET1      N:_flag_thd2.2     ;; 2 cycles
//  370                 if(thd_cntr_b_phase >= 10)
        MOV       A, N:_thd_cntr_b_phase  ;; 1 cycle
        CMP       A, #0xA            ;; 1 cycle
        BC        ??thd_five_sec_loop_55  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
//  371                 {
//  372                     thd.Bph.correction = thd.Bph.correction_temp;
        MOV       A, N:_thd+53       ;; 1 cycle
        MOV       N:_thd+52, A       ;; 1 cycle
//  373                     thd_cntr_b_phase = 0;
        MOV       N:_thd_cntr_b_phase, #0x0  ;; 1 cycle
        BR        S:??thd_five_sec_loop_55  ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
//  374                 }
//  375             }
//  376             else
//  377             {
//  378                 thd_cntr_b_phase = 0;
??thd_five_sec_loop_54:
        MOV       N:_thd_cntr_b_phase, #0x0  ;; 1 cycle
//  379                 thd.Bph.correction = 0;
        MOV       N:_thd+52, #0x0    ;; 1 cycle
//  380                 thd_cntr_b_f = 0;
        CLR1      N:_flag_thd2.2     ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
//  381             }
//  382             
//  383             flag_thd_process_b = 0;
??thd_five_sec_loop_55:
        CLR1      N:_flag_thd1.5     ;; 2 cycles
//  384             
//  385             if(vol.Rph.rms >= THD_VOL_THR && curr.Rph.rms >= THD_CURR_THR)
        MOVW      AX, N:_vol         ;; 1 cycle
        CMPW      AX, #0x1388        ;; 1 cycle
        BC        ??thd_five_sec_loop_56  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
        MOVW      BC, N:_curr+2      ;; 1 cycle
        MOVW      AX, N:_curr        ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
        CMPW      AX, #0xFA          ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??thd_process_9:
        BC        ??thd_five_sec_loop_56  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
//  386             {
//  387                 flag_thd_cal_r = 1;
        SET1      N:_flag_thd1.0     ;; 2 cycles
        BR        S:??thd_five_sec_loop_9  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  388             }
//  389             else if(vol.Yph.rms >= THD_VOL_THR && curr.Yph.rms >= THD_CURR_THR)
??thd_five_sec_loop_56:
        MOVW      AX, N:_vol+6       ;; 1 cycle
        CMPW      AX, #0x1388        ;; 1 cycle
        BC        ??thd_five_sec_loop_57  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      BC, N:_curr+18     ;; 1 cycle
        MOVW      AX, N:_curr+16     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
        CMPW      AX, #0xFA          ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??thd_process_10:
        BC        ??thd_five_sec_loop_57  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
//  390             {
//  391                 flag_thd_cal_y = 1;
        SET1      N:_flag_thd1.2     ;; 2 cycles
        BR        S:??thd_five_sec_loop_9  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  392             }
//  393             else if(vol.Bph.rms >= THD_VOL_THR && curr.Bph.rms >= THD_CURR_THR)
??thd_five_sec_loop_57:
        MOVW      AX, N:_vol+12      ;; 1 cycle
        CMPW      AX, #0x1388        ;; 1 cycle
        BC        ??thd_five_sec_loop_58  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      BC, N:_curr+34     ;; 1 cycle
        MOVW      AX, N:_curr+32     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
        CMPW      AX, #0xFA          ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??thd_process_11:
        BC        ??thd_five_sec_loop_58  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
//  394             {
//  395                 flag_thd_cal_b = 1;
        SET1      N:_flag_thd1.4     ;; 2 cycles
        BR        S:??thd_five_sec_loop_9  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  396             }
//  397             else
//  398             {
//  399               thd_cal_restart = 0;
??thd_five_sec_loop_58:
        CLR1      N:_flag_thd2.3     ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
//  400             }
//  401             
//  402         }        
//  403     }
//  404 }
??thd_five_sec_loop_9:
        ADDW      SP, #0x18          ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock1
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 1268 cycles

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
`thd_process::thd_acc_sin1`:
        DS 8

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
`thd_process::thd_acc_cos1`:
        DS 8

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
`thd_process::thd_acc_rms`:
        DS 8

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
`thd_process::cntr_temp`:
        DS 2

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock2 Using cfiCommon0
          CFI Function _thd_100ms_loop
          CFI NoCalls
        CODE
//  405 void thd_100ms_loop()
//  406 {
_thd_100ms_loop:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  407   if(thd_cntr_r_f == 1)
        MOVW      HL, #LWRD(_flag_thd2)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??thd_five_sec_loop_59  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  408   {
//  409     thd_cntr_r_f = 0;
        CLR1      N:_flag_thd2.0     ;; 2 cycles
//  410     thd_cntr_r_phase++;
        INC       N:_thd_cntr_r_phase  ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
//  411   }
//  412   if(thd_cntr_y_f == 1)
??thd_five_sec_loop_59:
        MOVW      HL, #LWRD(_flag_thd2)  ;; 1 cycle
        MOV1      CY, [HL].1         ;; 1 cycle
        BNC       ??thd_five_sec_loop_60  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  413   {
//  414     thd_cntr_y_f = 0;
        CLR1      N:_flag_thd2.1     ;; 2 cycles
//  415     thd_cntr_y_phase++;
        INC       N:_thd_cntr_y_phase  ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
//  416   }
//  417   if(thd_cntr_b_f == 1)
??thd_five_sec_loop_60:
        MOVW      HL, #LWRD(_flag_thd2)  ;; 1 cycle
        MOV1      CY, [HL].2         ;; 1 cycle
        BNC       ??thd_five_sec_loop_61  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  418   {
//  419     thd_cntr_b_f = 0;
        CLR1      N:_flag_thd2.2     ;; 2 cycles
//  420     thd_cntr_b_phase++;
        INC       N:_thd_cntr_b_phase  ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
//  421   }
//  422 }
??thd_five_sec_loop_61:
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock2
        ; ------------------------------------- Block: 6 cycles
        ; ------------------------------------- Total: 36 cycles

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock3 Using cfiCommon0
          CFI Function _thd_one_sec_loop
        CODE
//  423 void thd_one_sec_loop()
//  424 {
_thd_one_sec_loop:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  425     
//  426     //    thd_var = (double)freq.Net*0.000001608495438637952; /* change it */
//  427     //    thd_var *= 7333.859777674537072;                /* converting to degree and multiplying by 128 */
//  428     thd_var = (double)freq.Net*0.011796479999999837642331;
        MOVW      AX, #0x3C41        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+6
        MOVW      AX, #0x4606        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+8
        MOVW      AX, N:_freq+6      ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
          CFI FunCall ?F_UL2F
        CALL      N:?F_UL2F          ;; 3 cycles
          CFI FunCall ?F_MUL
        CALL      N:?F_MUL           ;; 3 cycles
        MOVW      N:_thd_var, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_thd_var+2, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  429 }
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock3
        ; ------------------------------------- Block: 23 cycles
        ; ------------------------------------- Total: 23 cycles

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock4 Using cfiCommon0
          CFI Function _thd_five_sec_loop
          CFI NoCalls
        CODE
//  430 void thd_five_sec_loop()
//  431 {
_thd_five_sec_loop:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  432   if(thd_cal_restart == 0)
        MOVW      HL, #LWRD(_flag_thd2)  ;; 1 cycle
        MOV1      CY, [HL].3         ;; 1 cycle
        BC        ??thd_five_sec_loop_62  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  433   {
//  434     if(vol.Rph.rms >= THD_VOL_THR && curr.Rph.rms >= THD_CURR_THR)
        MOVW      AX, N:_vol         ;; 1 cycle
        CMPW      AX, #0x1388        ;; 1 cycle
        BC        ??thd_five_sec_loop_63  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      BC, N:_curr+2      ;; 1 cycle
        MOVW      AX, N:_curr        ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
        CMPW      AX, #0xFA          ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??thd_five_sec_loop_64:
        BC        ??thd_five_sec_loop_63  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
//  435     {
//  436       flag_thd_cal_r = 1;
        SET1      N:_flag_thd1.0     ;; 2 cycles
//  437       thd_cal_start = 1;
        SET1      N:_flag_thd1.7     ;; 2 cycles
//  438       thd_cal_restart = 1;
        SET1      N:_flag_thd2.3     ;; 2 cycles
        RET                          ;; 6 cycles
        ; ------------------------------------- Block: 12 cycles
//  439     }
//  440     else if(vol.Yph.rms >= THD_VOL_THR && curr.Yph.rms >= THD_CURR_THR)
??thd_five_sec_loop_63:
        MOVW      AX, N:_vol+6       ;; 1 cycle
        CMPW      AX, #0x1388        ;; 1 cycle
        BC        ??thd_five_sec_loop_65  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      BC, N:_curr+18     ;; 1 cycle
        MOVW      AX, N:_curr+16     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
        CMPW      AX, #0xFA          ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??thd_five_sec_loop_66:
        BC        ??thd_five_sec_loop_65  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
//  441     {
//  442       flag_thd_cal_y = 1;
        SET1      N:_flag_thd1.2     ;; 2 cycles
//  443       thd_cal_start = 1;
        SET1      N:_flag_thd1.7     ;; 2 cycles
//  444       thd_cal_restart = 1;
        SET1      N:_flag_thd2.3     ;; 2 cycles
        RET                          ;; 6 cycles
        ; ------------------------------------- Block: 12 cycles
//  445     }
//  446     else if(vol.Bph.rms >= THD_VOL_THR && curr.Bph.rms >= THD_CURR_THR)
??thd_five_sec_loop_65:
        MOVW      AX, N:_vol+12      ;; 1 cycle
        CMPW      AX, #0x1388        ;; 1 cycle
        BC        ??thd_five_sec_loop_67  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      BC, N:_curr+34     ;; 1 cycle
        MOVW      AX, N:_curr+32     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
        CMPW      AX, #0xFA          ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??thd_five_sec_loop_68:
        BC        ??thd_five_sec_loop_67  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
//  447     {
//  448       flag_thd_cal_b = 1;
        SET1      N:_flag_thd1.4     ;; 2 cycles
//  449       thd_cal_start = 1;
        SET1      N:_flag_thd1.7     ;; 2 cycles
//  450       thd_cal_restart = 1;
        SET1      N:_flag_thd2.3     ;; 2 cycles
        RET                          ;; 6 cycles
        ; ------------------------------------- Block: 12 cycles
//  451     }
//  452     else 
//  453     {
//  454       thd_cal_start = 0;
??thd_five_sec_loop_67:
        CLR1      N:_flag_thd1.7     ;; 2 cycles
//  455       thd_cal_restart = 0;
        CLR1      N:_flag_thd2.3     ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
//  456     }
//  457   }
//  458 }
??thd_five_sec_loop_62:
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock4
        ; ------------------------------------- Block: 6 cycles
        ; ------------------------------------- Total: 103 cycles

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
//   195 bytes in section .bss
//    17 bytes in section .bss.noinit  (abs)
//     8 bytes in section .data
// 5'234 bytes in section .text
// 
// 5'234 bytes of CODE memory
//   195 bytes of DATA memory (+ 25 bytes shared)
//
//Errors: none
//Warnings: none
