///////////////////////////////////////////////////////////////////////////////
//
// IAR C/C++ Compiler V4.20.1.2260 for RL78               22/Dec/2020  15:13:08
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
//        BootCode\source_code\driver_files\r_cg_tau.c
//    Command line       =
//        -f C:\Users\laptop\AppData\Local\Temp\EW2A3E.tmp ("D:\Dheeraj\New
//        folder\0. GDEV72 - BootCode\source_code\driver_files\r_cg_tau.c"
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
//        D:\Dheeraj\New folder\0. GDEV72 - BootCode\Debug\List\r_cg_tau.s
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

        EXTERN ?F_F2UL
        EXTERN ?F_MUL
        EXTERN ?F_UL2F
        EXTERN _R_TAU0_Channel1_SetValue
        EXTERN ___iar_fmex

        PUBLIC _R_TAU0_Channel0_Start
        PUBLIC _R_TAU0_Channel0_Stop
        PUBLIC _R_TAU0_Channel1_Start
        PUBLIC _R_TAU0_Channel1_Stop
        PUBLIC _R_TAU0_Channel2_Start
        PUBLIC _R_TAU0_Channel2_Stop
        PUBLIC _R_TAU0_Channel4_Set_SoftwareTriggerOn
        PUBLIC _R_TAU0_Channel4_Start
        PUBLIC _R_TAU0_Channel4_Stop
        PUBLIC _R_TAU0_Channel4_UpdatePulseHighWidth
        PUBLIC _R_TAU0_Channel6_Start
        PUBLIC _R_TAU0_Channel6_Stop
        PUBLIC _R_TAU0_Channel6_UpdateInterval
        PUBLIC _R_TAU0_Create
        PUBLIC __A_IF1
        PUBLIC __A_IF2
        PUBLIC __A_MK1
        PUBLIC __A_MK2
        PUBLIC __A_PER0
        PUBLIC __A_PR01
        PUBLIC __A_PR02
        PUBLIC __A_PR11
        PUBLIC __A_PR12
        PUBLIC __A_TDR05
        PUBLIC __A_TMR01
        PUBLIC __A_TO0
        PUBLIC __A_TOE0
        PUBLIC __A_TPS0
        PUBLIC __A_TS0
        PUBLIC __A_TT0
        PUBLIC _g_tau0_ch6_cascade_count
        PUBLIC _g_tau0_ch6_cascade_count_init
        PUBLIC _g_tau0_ch6_max_tdr_support
        PUBLIC _g_tau0_ch6_tdr_update_value
        
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
        
// D:\Dheeraj\New folder\0. GDEV72 - BootCode\source_code\driver_files\r_cg_tau.c
//    1 /***********************************************************************************************************************
//    2 * DISCLAIMER
//    3 * This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products.
//    4 * No other uses are authorized. This software is owned by Renesas Electronics Corporation and is protected under all
//    5 * applicable laws, including copyright laws. 
//    6 * THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIESREGARDING THIS SOFTWARE, WHETHER EXPRESS, IMPLIED
//    7 * OR STATUTORY, INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
//    8 * NON-INFRINGEMENT.  ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED.TO THE MAXIMUM EXTENT PERMITTED NOT PROHIBITED BY
//    9 * LAW, NEITHER RENESAS ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES SHALL BE LIABLE FOR ANY DIRECT,
//   10 * INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON RELATED TO THIS SOFTWARE, EVEN IF RENESAS OR
//   11 * ITS AFFILIATES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
//   12 * Renesas reserves the right, without notice, to make changes to this software and to discontinue the availability 
//   13 * of this software. By using this software, you agree to the additional terms and conditions found by accessing the 
//   14 * following link:
//   15 * http://www.renesas.com/disclaimer
//   16 *
//   17 * Copyright (C) 2013, 2015 Renesas Electronics Corporation. All rights reserved.
//   18 ***********************************************************************************************************************/
//   19 
//   20 /***********************************************************************************************************************
//   21 * File Name    : r_cg_tau.c
//   22 * Version      : 
//   23 * Device(s)    : RL78/I1C
//   24 * Tool-Chain   : CCRL
//   25 * Description  : This file implements device driver for TAU module.
//   26 * Creation Date: 
//   27 ***********************************************************************************************************************/
//   28 
//   29 /***********************************************************************************************************************
//   30 Pragma directive
//   31 ***********************************************************************************************************************/
//   32 /* Start user code for pragma. Do not edit comment generated here */
//   33 /* End user code. Do not edit comment generated here */
//   34 
//   35 /***********************************************************************************************************************
//   36 Includes
//   37 ***********************************************************************************************************************/
//   38 #include "r_cg_macrodriver.h"

        ASEGN `.bss.noinit`:DATA:NOROOT,0fff6aH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_TDR05
// __no_init union <unnamed>#88 volatile __sfr __no_bit_access _A_TDR05
__A_TDR05:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0fffd0H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_IF2
// __no_init union <unnamed>#112 volatile __sfr _A_IF2
__A_IF2:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0fffd4H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_MK2
// __no_init union <unnamed>#122 volatile __sfr _A_MK2
__A_MK2:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0fffd8H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_PR02
// __no_init union <unnamed>#132 volatile __sfr _A_PR02
__A_PR02:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0fffdcH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_PR12
// __no_init union <unnamed>#142 volatile __sfr _A_PR12
__A_PR12:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0fffe2H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_IF1
// __no_init union <unnamed>#160 volatile __sfr _A_IF1
__A_IF1:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0fffe6H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_MK1
// __no_init union <unnamed>#178 volatile __sfr _A_MK1
__A_MK1:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0fffeaH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_PR01
// __no_init union <unnamed>#196 volatile __sfr _A_PR01
__A_PR01:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0fffeeH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_PR11
// __no_init union <unnamed>#214 volatile __sfr _A_PR11
__A_PR11:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f00f0H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_PER0
// __no_init union <unnamed>#275 volatile _A_PER0
__A_PER0:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0192H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_TMR01
// __no_init union <unnamed>#413 volatile __no_bit_access _A_TMR01
__A_TMR01:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f01b2H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_TS0
// __no_init union <unnamed>#447 volatile _A_TS0
__A_TS0:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f01b4H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_TT0
// __no_init union <unnamed>#450 volatile _A_TT0
__A_TT0:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f01b6H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_TPS0
// __no_init union <unnamed>#453 volatile __no_bit_access _A_TPS0
__A_TPS0:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f01b8H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_TO0
// __no_init union <unnamed>#454 volatile __no_bit_access _A_TO0
__A_TO0:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f01baH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_TOE0
// __no_init union <unnamed>#457 volatile _A_TOE0
__A_TOE0:
        DS 2
//   39 #include "r_cg_tau.h"
//   40 /* Start user code for include. Do not edit comment generated here */
//   41 /* End user code. Do not edit comment generated here */
//   42 #include "r_cg_userdefine.h"
//   43 
//   44 /***********************************************************************************************************************
//   45 Global variables and functions
//   46 ***********************************************************************************************************************/
//   47 /* Start user code for global. Do not edit comment generated here */

        SECTION `.data`:DATA:REORDER:NOROOT(1)
//   48 uint16_t g_tau0_ch6_cascade_count_init = 0xFFFF;
_g_tau0_ch6_cascade_count_init:
        DATA16
        DW 65'535

        SECTION `.data`:DATA:REORDER:NOROOT(1)
//   49 uint16_t g_tau0_ch6_cascade_count = 0xFFFF;
_g_tau0_ch6_cascade_count:
        DATA16
        DW 65'535

        SECTION `.data`:DATA:REORDER:NOROOT(1)
//   50 uint16_t g_tau0_ch6_tdr_update_value = _FFFF_TAU_TDR06_VALUE;
_g_tau0_ch6_tdr_update_value:
        DATA16
        DW 65'535

        SECTION `.data`:DATA:REORDER:NOROOT(1)
//   51 const uint16_t g_tau0_ch6_max_tdr_support = 0xFFFF;
_g_tau0_ch6_max_tdr_support:
        DATA16
        DW 65'535
//   52 
//   53 /* End user code. Do not edit comment generated here */
//   54 
//   55 /***********************************************************************************************************************
//   56 * Function Name: R_TAU0_Create
//   57 * Description  : This function initializes the TAU0 module.
//   58 * Arguments    : None
//   59 * Return Value : None
//   60 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock0 Using cfiCommon0
          CFI Function _R_TAU0_Create
        CODE
//   61 void R_TAU0_Create(void)
//   62 {
_R_TAU0_Create:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//   63     TAU0EN = 1U;    /* supplies input clock */
        SET1      0xF00F0.0          ;; 2 cycles
//   64     TPS0 = _0001_TAU_CKM0_fCLK_1 | _0070_TAU_CKM1_fCLK_7 | _0000_TAU_CKM2_fCLK_1 | _0000_TAU_CKM3_fCLK_8; /* 24 MHz */
        MOVW      AX, #0x71          ;; 1 cycle
        MOVW      0x1B6, AX          ;; 1 cycle
//   65     /* Stop all channels */
//   66     TT0 = _0001_TAU_CH0_STOP_TRG_ON | _0002_TAU_CH1_STOP_TRG_ON | _0004_TAU_CH2_STOP_TRG_ON | 
//   67           _0008_TAU_CH3_STOP_TRG_ON | _0010_TAU_CH4_STOP_TRG_ON | _0020_TAU_CH5_STOP_TRG_ON | 
//   68           _0040_TAU_CH6_STOP_TRG_ON | _0080_TAU_CH7_STOP_TRG_ON | _0200_TAU_CH1_H8_STOP_TRG_ON | 
//   69           _0800_TAU_CH3_H8_STOP_TRG_ON;
        MOVW      AX, #0xAFF         ;; 1 cycle
        MOVW      0x1B4, AX          ;; 1 cycle
//   70     /* Mask channel 0 interrupt */
//   71     TMMK00 = 1U;    /* disable INTTM00 interrupt */
        SET1      0xFFFE6.4          ;; 2 cycles
//   72     TMIF00 = 0U;    /* clear INTTM00 interrupt flag */
        CLR1      0xFFFE2.4          ;; 2 cycles
//   73     /* Mask channel 1 interrupt */
//   74     TMMK01 = 1U;    /* disable INTTM01 interrupt */
        SET1      0xFFFE6.7          ;; 2 cycles
//   75     TMIF01 = 0U;    /* clear INTTM01 interrupt flag */
        CLR1      0xFFFE2.7          ;; 2 cycles
//   76     /* Mask channel 1 higher 8 bits interrupt */
//   77     TMMK01H = 1U;    /* disable INTTM01H interrupt */
        SET1      0xFFFE6.0          ;; 2 cycles
//   78     TMIF01H = 0U;    /* clear INTTM01H interrupt flag */
        CLR1      0xFFFE2.0          ;; 2 cycles
//   79     /* Mask channel 2 interrupt */
//   80     TMMK02 = 1U;    /* disable INTTM02 interrupt */
        SET1      0xFFFE7.0          ;; 2 cycles
//   81     TMIF02 = 0U;    /* clear INTTM02 interrupt flag */
        CLR1      0xFFFE3.0          ;; 2 cycles
//   82     /* Mask channel 3 interrupt */
//   83     TMMK03 = 1U;    /* disable INTTM03 interrupt */
        SET1      0xFFFE7.1          ;; 2 cycles
//   84     TMIF03 = 0U;    /* clear INTTM03 interrupt flag */
        CLR1      0xFFFE3.1          ;; 2 cycles
//   85     /* Mask channel 3 higher 8 bits interrupt */
//   86     TMMK03H = 1U;    /* disable INTTM03H interrupt */
        SET1      0xFFFE6.3          ;; 2 cycles
//   87     TMIF03H = 0U;    /* clear INTTM03H interrupt flag */
        CLR1      0xFFFE2.3          ;; 2 cycles
//   88     /* Mask channel 4 interrupt */
//   89     TMMK04 = 1U;    /* disable INTTM04 interrupt */
        SET1      0xFFFD4.1          ;; 2 cycles
//   90     TMIF04 = 0U;    /* clear INTTM04 interrupt flag */
        CLR1      0xFFFD0.1          ;; 2 cycles
//   91     /* Mask channel 5 interrupt */
//   92     TMMK05 = 1U;    /* disable INTTM05 interrupt */
        SET1      0xFFFD4.2          ;; 2 cycles
//   93     TMIF05 = 0U;    /* clear INTTM05 interrupt flag */
        CLR1      0xFFFD0.2          ;; 2 cycles
//   94     /* Mask channel 6 interrupt */
//   95     TMMK06 = 1U;    /* disable INTTM06 interrupt */
        SET1      0xFFFD5.0          ;; 2 cycles
//   96     TMIF06 = 0U;    /* clear INTTM06 interrupt flag */
        CLR1      0xFFFD1.0          ;; 2 cycles
//   97     /* Mask channel 7 interrupt */
//   98     TMMK07 = 1U;    /* disable INTTM07 interrupt */
        SET1      0xFFFD5.1          ;; 2 cycles
//   99     TMIF07 = 0U;    /* clear INTTM07 interrupt flag */
        CLR1      0xFFFD1.1          ;; 2 cycles
//  100     /* Set INTTM00 low priority */
//  101     TMPR100 = 1U;
        SET1      0xFFFEE.4          ;; 2 cycles
//  102     TMPR000 = 1U;
        SET1      0xFFFEA.4          ;; 2 cycles
//  103     /* Set INTTM01 low priority */
//  104     TMPR101 = 1U;
        SET1      0xFFFEE.7          ;; 2 cycles
//  105     TMPR001 = 1U;
        SET1      0xFFFEA.7          ;; 2 cycles
//  106     /* Set INTTM02 low priority */
//  107     TMPR102 = 1U;
        SET1      0xFFFEF.0          ;; 2 cycles
//  108     TMPR002 = 1U;
        SET1      0xFFFEB.0          ;; 2 cycles
//  109     /* Set INTTM04 level1 priority */
//  110     TMPR104 = 0U;
        CLR1      0xFFFDC.1          ;; 2 cycles
//  111     TMPR004 = 1U;
        SET1      0xFFFD8.1          ;; 2 cycles
//  112     /* Set INTTM05 level1 priority */
//  113     TMPR105 = 0U;
        CLR1      0xFFFDC.2          ;; 2 cycles
//  114     TMPR005 = 1U;
        SET1      0xFFFD8.2          ;; 2 cycles
//  115     /* Set INTTM06 level1 priority */
//  116     TMPR106 = 0U;
        CLR1      0xFFFDD.0          ;; 2 cycles
//  117     TMPR006 = 1U;
        SET1      0xFFFD9.0          ;; 2 cycles
//  118 
//  119     /* Channel 0 used as interval timer */
//  120 //    TMR00 = _0000_TAU_CLOCK_SELECT_CKM0 | _0000_TAU_CLOCK_MODE_CKS | _0000_TAU_COMBINATION_SLAVE | 
//  121 //            _0000_TAU_TRIGGER_SOFTWARE | _0000_TAU_MODE_INTERVAL_TIMER | _0000_TAU_START_INT_UNUSED;
//  122 //    TDR00 = _2EDF_TAU_TDR00_VALUE;//_5DBF_TAU_TDR00_VALUE;
//  123 //    TO0 &= ~_0001_TAU_CH0_OUTPUT_VALUE_1;
//  124 //    TOE0 &= ~_0001_TAU_CH0_OUTPUT_ENABLE;       
//  125     
//  126     /* Channel 1 used as one shot counter */
//  127     TMR01 = _0000_TAU_CLOCK_SELECT_CKM0 | _0000_TAU_CLOCK_MODE_CKS | _0000_TAU_COMBINATION_SLAVE | 
//  128             _0000_TAU_TRIGGER_SOFTWARE | _0008_TAU_MODE_ONE_COUNT | _0000_TAU_START_INT_UNUSED;
        MOVW      AX, #0x8           ;; 1 cycle
        MOVW      0x192, AX          ;; 1 cycle
//  129     R_TAU0_Channel1_SetValue(1440);//1426;//1632;//1426;
        MOVW      AX, #0x5A0         ;; 1 cycle
          CFI FunCall _R_TAU0_Channel1_SetValue
        CALL      _R_TAU0_Channel1_SetValue  ;; 3 cycles
//  130     TO0 &= ~_0002_TAU_CH1_OUTPUT_VALUE_1;
        MOVW      AX, 0x1B8          ;; 1 cycle
        AND       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        AND       A, #0xFD           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      0x1B8, AX          ;; 1 cycle
//  131     TOE0 &= ~_0001_TAU_CH0_OUTPUT_ENABLE;
        CLR1      0xF01BA.0          ;; 2 cycles
//  132     
//  133     /* Channel 2 used as interval timer */
//  134 //    TMR02 = _8000_TAU_CLOCK_SELECT_CKM1 | _0000_TAU_CLOCK_MODE_CKS | _0000_TAU_COMBINATION_SLAVE | 
//  135 //            _0000_TAU_TRIGGER_SOFTWARE | _0000_TAU_MODE_INTERVAL_TIMER | _0000_TAU_START_INT_UNUSED;
//  136 //    TDR02 = _1D4B_TAU_TDR02_VALUE;
//  137 //    TOM0 &= ~_0004_TAU_CH2_OUTPUT_COMBIN;
//  138 //    TOL0 &= ~_0004_TAU_CH2_OUTPUT_LEVEL_L;
//  139 //    TO0 &= ~_0004_TAU_CH2_OUTPUT_VALUE_1;
//  140 //    TOE0 &= ~_0004_TAU_CH2_OUTPUT_ENABLE;
//  141 //    
//  142 //    /* Channel 4 is used as master channel for oneshot output function */
//  143 //    TMR04 = _8000_TAU_CLOCK_SELECT_CKM1 | _0000_TAU_CLOCK_MODE_CKS | _0800_TAU_COMBINATION_MASTER | 
//  144 //            _0000_TAU_TRIGGER_SOFTWARE | _0008_TAU_MODE_ONESHOT;
//  145 //    TDR04 = _03A8_TAU_TDR04_VALUE;
//  146 //    TOM0 &= ~_0010_TAU_CH4_OUTPUT_COMBIN;
//  147 //    TOL0 &= ~_0010_TAU_CH4_OUTPUT_LEVEL_L;
//  148 //    TO0 &= ~_0010_TAU_CH4_OUTPUT_VALUE_1;
//  149 //    TOE0 &= ~_0010_TAU_CH4_OUTPUT_ENABLE;
//  150 //    /* Channel 5 is used as slave channel for oneshot output function */
//  151 //    TMR05 = _8000_TAU_CLOCK_SELECT_CKM1 | _0000_TAU_CLOCK_MODE_CKS | _0000_TAU_COMBINATION_SLAVE | 
//  152 //            _0400_TAU_TRIGGER_MASTER_INT | _0008_TAU_MODE_ONESHOT;
//  153 //    TDR05 = _15F8_TAU_TDR05_VALUE;
//  154 //    TOM0 |= _0020_TAU_CH5_OUTPUT_COMBIN;
//  155 //    TOL0 &= ~_0020_TAU_CH5_OUTPUT_LEVEL_L;
//  156 //    TO0 &= ~_0020_TAU_CH5_OUTPUT_VALUE_1;
//  157 //    TOE0 |= _0020_TAU_CH5_OUTPUT_ENABLE;
//  158 //    /* Channel 6 used as interval timer */
//  159 //    TMR06 = _8000_TAU_CLOCK_SELECT_CKM1 | _0000_TAU_CLOCK_MODE_CKS | _0000_TAU_COMBINATION_SLAVE | 
//  160 //            _0000_TAU_TRIGGER_SOFTWARE | _0000_TAU_MODE_INTERVAL_TIMER | _0000_TAU_START_INT_UNUSED;
//  161 //    TDR06 = _FFFF_TAU_TDR06_VALUE;
//  162 //    TOM0 &= ~_0040_TAU_CH6_OUTPUT_COMBIN;
//  163 //    TOL0 &= ~_0040_TAU_CH6_OUTPUT_LEVEL_L;
//  164 //    TO0 &= ~_0040_TAU_CH6_OUTPUT_VALUE_1;
//  165 //    TOE0 &= ~_0040_TAU_CH6_OUTPUT_ENABLE;
//  166 
//  167 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock0
        ; ------------------------------------- Block: 90 cycles
        ; ------------------------------------- Total: 90 cycles
        REQUIRE __A_PER0
        REQUIRE __A_TPS0
        REQUIRE __A_TT0
        REQUIRE __A_MK1
        REQUIRE __A_IF1
        REQUIRE __A_MK2
        REQUIRE __A_IF2
        REQUIRE __A_PR11
        REQUIRE __A_PR01
        REQUIRE __A_PR12
        REQUIRE __A_PR02
        REQUIRE __A_TMR01
        REQUIRE __A_TO0
        REQUIRE __A_TOE0
//  168 
//  169 /***********************************************************************************************************************
//  170 * Function Name: R_TAU0_Channel0_Start
//  171 * Description  : This function starts TAU0 channel 0 counter.
//  172 * Arguments    : None
//  173 * Return Value : None
//  174 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock1 Using cfiCommon0
          CFI Function _R_TAU0_Channel0_Start
          CFI NoCalls
        CODE
//  175 void R_TAU0_Channel0_Start(void)
//  176 {
_R_TAU0_Channel0_Start:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  177     TMIF00 = 0U;    /* clear INTTM00 interrupt flag */
        CLR1      0xFFFE2.4          ;; 2 cycles
//  178     TMMK00 = 0U;    /* enable INTTM00 interrupt */
        CLR1      0xFFFE6.4          ;; 2 cycles
//  179     TS0 |= _0001_TAU_CH0_START_TRG_ON;
        SET1      0xF01B2.0          ;; 2 cycles
//  180 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock1
        ; ------------------------------------- Block: 12 cycles
        ; ------------------------------------- Total: 12 cycles
        REQUIRE __A_IF1
        REQUIRE __A_MK1
        REQUIRE __A_TS0
//  181 
//  182 /***********************************************************************************************************************
//  183 * Function Name: R_TAU0_Channel0_Stop
//  184 * Description  : This function stops TAU0 channel 0 counter.
//  185 * Arguments    : None
//  186 * Return Value : None
//  187 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock2 Using cfiCommon0
          CFI Function _R_TAU0_Channel0_Stop
          CFI NoCalls
        CODE
//  188 void R_TAU0_Channel0_Stop(void)
//  189 {
_R_TAU0_Channel0_Stop:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  190     /* Mask channel 0 interrupt */
//  191     TMMK00 = 1U;    /* disable INTTM00 interrupt */
        SET1      0xFFFE6.4          ;; 2 cycles
//  192     TMIF00 = 0U;    /* clear INTTM00 interrupt flag */
        CLR1      0xFFFE2.4          ;; 2 cycles
//  193     TT0 |= _0001_TAU_CH0_STOP_TRG_ON;
        SET1      0xF01B4.0          ;; 2 cycles
//  194 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock2
        ; ------------------------------------- Block: 12 cycles
        ; ------------------------------------- Total: 12 cycles
        REQUIRE __A_MK1
        REQUIRE __A_IF1
        REQUIRE __A_TT0
//  195 
//  196 /***********************************************************************************************************************
//  197 * Function Name: R_TAU0_Channel1_Start
//  198 * Description  : This function starts TAU0 channel 1 counter.
//  199 * Arguments    : None
//  200 * Return Value : None
//  201 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock3 Using cfiCommon0
          CFI Function _R_TAU0_Channel1_Start
          CFI NoCalls
        CODE
//  202 void R_TAU0_Channel1_Start(void)
//  203 {
_R_TAU0_Channel1_Start:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  204     TMIF01 = 0U;
        CLR1      0xFFFE2.7          ;; 2 cycles
//  205     TMMK01 = 1U;    /* disable INTTM01 interrupt */
        SET1      0xFFFE6.7          ;; 2 cycles
//  206     TS0 |= _0002_TAU_CH1_START_TRG_ON;
        SET1      0xF01B2.1          ;; 2 cycles
//  207 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock3
        ; ------------------------------------- Block: 12 cycles
        ; ------------------------------------- Total: 12 cycles
        REQUIRE __A_IF1
        REQUIRE __A_MK1
        REQUIRE __A_TS0
//  208 
//  209 /***********************************************************************************************************************
//  210 * Function Name: R_TAU0_Channel1_Stop
//  211 * Description  : This function stops TAU0 channel 1 counter.
//  212 * Arguments    : None
//  213 * Return Value : None
//  214 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock4 Using cfiCommon0
          CFI Function _R_TAU0_Channel1_Stop
          CFI NoCalls
        CODE
//  215 void R_TAU0_Channel1_Stop(void)
//  216 {
_R_TAU0_Channel1_Stop:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  217     TMMK01 = 1U;    /* disable INTTM00 interrupt */
        SET1      0xFFFE6.7          ;; 2 cycles
//  218     TMIF01 = 0U;    /* clear INTTM00 interrupt flag */
        CLR1      0xFFFE2.7          ;; 2 cycles
//  219     TT0 |= _0002_TAU_CH1_START_TRG_ON;
        SET1      0xF01B4.1          ;; 2 cycles
//  220 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock4
        ; ------------------------------------- Block: 12 cycles
        ; ------------------------------------- Total: 12 cycles
        REQUIRE __A_MK1
        REQUIRE __A_IF1
        REQUIRE __A_TT0
//  221 
//  222 /***********************************************************************************************************************
//  223 * Function Name: R_TAU0_Channel2_Start
//  224 * Description  : This function starts TAU0 channel 2 counter.
//  225 * Arguments    : None
//  226 * Return Value : None
//  227 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock5 Using cfiCommon0
          CFI Function _R_TAU0_Channel2_Start
          CFI NoCalls
        CODE
//  228 void R_TAU0_Channel2_Start(void)
//  229 {
_R_TAU0_Channel2_Start:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  230     TMIF02 = 0U;    /* clear INTTM02 interrupt flag */
        CLR1      0xFFFE3.0          ;; 2 cycles
//  231     TMMK02 = 0U;    /* enable INTTM02 interrupt */
        CLR1      0xFFFE7.0          ;; 2 cycles
//  232     TS0 |= _0004_TAU_CH2_START_TRG_ON;
        SET1      0xF01B2.2          ;; 2 cycles
//  233 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock5
        ; ------------------------------------- Block: 12 cycles
        ; ------------------------------------- Total: 12 cycles
        REQUIRE __A_IF1
        REQUIRE __A_MK1
        REQUIRE __A_TS0
//  234 /***********************************************************************************************************************
//  235 * Function Name: R_TAU0_Channel2_Stop
//  236 * Description  : This function stops TAU0 channel 2 counter.
//  237 * Arguments    : None
//  238 * Return Value : None
//  239 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock6 Using cfiCommon0
          CFI Function _R_TAU0_Channel2_Stop
          CFI NoCalls
        CODE
//  240 void R_TAU0_Channel2_Stop(void)
//  241 {
_R_TAU0_Channel2_Stop:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  242     TT0 |= _0004_TAU_CH2_STOP_TRG_ON;
        SET1      0xF01B4.2          ;; 2 cycles
//  243     /* Mask channel 2 interrupt */
//  244     TMMK02 = 1U;    /* disable INTTM02 interrupt */
        SET1      0xFFFE7.0          ;; 2 cycles
//  245     TMIF02 = 0U;    /* clear INTTM02 interrupt flag */
        CLR1      0xFFFE3.0          ;; 2 cycles
//  246 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock6
        ; ------------------------------------- Block: 12 cycles
        ; ------------------------------------- Total: 12 cycles
        REQUIRE __A_TT0
        REQUIRE __A_MK1
        REQUIRE __A_IF1
//  247 
//  248 /* Start user code for adding. Do not edit comment generated here */
//  249 /***********************************************************************************************************************
//  250 * Function Name: R_TAU0_Channel1_SetValue
//  251 * Description  : This function set value into TDR01.
//  252 * Arguments    : uint16_t reg_value: value of TDR01
//  253 * Return Value : None
//  254 ***********************************************************************************************************************/
//  255 
//  256 
//  257 /***********************************************************************************************************************
//  258 * Function Name: R_TAU0_Channel4_Start
//  259 * Description  : This function starts TAU0 channel 4 counter.
//  260 * Arguments    : None
//  261 * Return Value : None
//  262 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock7 Using cfiCommon0
          CFI Function _R_TAU0_Channel4_Start
          CFI NoCalls
        CODE
//  263 void R_TAU0_Channel4_Start(void)
//  264 {
_R_TAU0_Channel4_Start:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  265     TMIF04 = 0U;    /* clear INTTM04 interrupt flag */
        CLR1      0xFFFD0.1          ;; 2 cycles
//  266     TMMK04 = 0U;    /* enable INTTM04 interrupt */
        CLR1      0xFFFD4.1          ;; 2 cycles
//  267     TMIF05 = 0U;    /* clear INTTM05 interrupt flag */
        CLR1      0xFFFD0.2          ;; 2 cycles
//  268     TMMK05 = 0U;    /* enable INTTM05 interrupt */
        CLR1      0xFFFD4.2          ;; 2 cycles
//  269     TS0 |= _0010_TAU_CH4_START_TRG_ON | _0020_TAU_CH5_START_TRG_ON;
        MOVW      AX, 0x1B2          ;; 1 cycle
        OR        A, #0x0            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        OR        A, #0x30           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      0x1B2, AX          ;; 1 cycle
//  270 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock7
        ; ------------------------------------- Block: 20 cycles
        ; ------------------------------------- Total: 20 cycles
        REQUIRE __A_IF2
        REQUIRE __A_MK2
        REQUIRE __A_TS0
//  271 /***********************************************************************************************************************
//  272 * Function Name: R_TAU0_Channel4_Stop
//  273 * Description  : This function stops TAU0 channel 4 counter.
//  274 * Arguments    : None
//  275 * Return Value : None
//  276 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock8 Using cfiCommon0
          CFI Function _R_TAU0_Channel4_Stop
          CFI NoCalls
        CODE
//  277 void R_TAU0_Channel4_Stop(void)
//  278 {
_R_TAU0_Channel4_Stop:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  279     TT0 |= _0010_TAU_CH4_STOP_TRG_ON | _0020_TAU_CH5_STOP_TRG_ON;
        MOVW      AX, 0x1B4          ;; 1 cycle
        OR        A, #0x0            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        OR        A, #0x30           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      0x1B4, AX          ;; 1 cycle
//  280     /* Mask channel 4 interrupt */
//  281     TMMK04 = 1U;    /* disable INTTM04 interrupt */
        SET1      0xFFFD4.1          ;; 2 cycles
//  282     TMIF04 = 0U;    /* clear INTTM04 interrupt flag */
        CLR1      0xFFFD0.1          ;; 2 cycles
//  283     /* Mask channel 5 interrupt */
//  284     TMMK05 = 1U;    /* disable INTTM05 interrupt */
        SET1      0xFFFD4.2          ;; 2 cycles
//  285     TMIF05 = 0U;    /* clear INTTM05 interrupt flag */
        CLR1      0xFFFD0.2          ;; 2 cycles
//  286 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock8
        ; ------------------------------------- Block: 20 cycles
        ; ------------------------------------- Total: 20 cycles
        REQUIRE __A_TT0
        REQUIRE __A_MK2
        REQUIRE __A_IF2
//  287 /***********************************************************************************************************************
//  288 * Function Name: R_TAU0_Channel4_Set_SoftwareTriggerOn
//  289 * Description  : This function generates software trigger for One-shot output function.
//  290 * Arguments    : None
//  291 * Return Value : None
//  292 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock9 Using cfiCommon0
          CFI Function _R_TAU0_Channel4_Set_SoftwareTriggerOn
          CFI NoCalls
        CODE
//  293 void R_TAU0_Channel4_Set_SoftwareTriggerOn(void)
//  294 {
_R_TAU0_Channel4_Set_SoftwareTriggerOn:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  295     TS0 |= _0010_TAU_CH4_START_TRG_ON;
        SET1      0xF01B2.4          ;; 2 cycles
//  296 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock9
        ; ------------------------------------- Block: 8 cycles
        ; ------------------------------------- Total: 8 cycles
        REQUIRE __A_TS0
//  297 
//  298 /***********************************************************************************************************************
//  299 * Function Name: R_TAU0_Channel4_UpdatePulseHighWidth
//  300 * Description  : Update pulse high width
//  301 * Arguments    : None
//  302 * Return Value : None
//  303 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock10 Using cfiCommon0
          CFI Function _R_TAU0_Channel4_UpdatePulseHighWidth
          CFI NoCalls
        CODE
//  304 void R_TAU0_Channel4_UpdatePulseHighWidth(uint16_t ms)
//  305 {
_R_TAU0_Channel4_UpdatePulseHighWidth:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
        MOVW      HL, AX             ;; 1 cycle
//  306     uint32_t counter;
//  307 
//  308     counter  = (uint32_t)ms * 1000 * TAU0_fCLK;
        MOVW      AX, HL             ;; 1 cycle
        MOVW      BC, #0x5DC0        ;; 1 cycle
        MULHU                        ;; 2 cycles
//  309     counter /= TAU0_CHANNEL5_DIVISOR;
        SHRW      AX, 0x7            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        SHRW      AX, 0x7            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        SHLW      AX, 0x9            ;; 1 cycle
        ADDW      AX, DE             ;; 1 cycle
//  310     counter--;
        SUBW      AX, #0x1           ;; 1 cycle
        SKNC
        DECW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        SUBW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  311     
//  312     TDR05 = (uint16_t)counter;
        MOVW      DE, AX             ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        MOVW      0xFFF6A, AX        ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
//  313 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock10
        ; ------------------------------------- Block: 32 cycles
        ; ------------------------------------- Total: 32 cycles
        REQUIRE __A_TDR05
//  314 
//  315 /***********************************************************************************************************************
//  316 * Function Name: R_TAU0_Channel6_Start
//  317 * Description  : This function starts TAU0 channel 6 counter.
//  318 * Arguments    : None
//  319 * Return Value : None
//  320 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock11 Using cfiCommon0
          CFI Function _R_TAU0_Channel6_Start
          CFI NoCalls
        CODE
//  321 void R_TAU0_Channel6_Start(void)
//  322 {
_R_TAU0_Channel6_Start:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  323     TMIF06 = 0U;    /* clear INTTM06 interrupt flag */
        CLR1      0xFFFD1.0          ;; 2 cycles
//  324     TMMK06 = 0U;    /* enable INTTM06 interrupt */
        CLR1      0xFFFD5.0          ;; 2 cycles
//  325     TS0 |= _0040_TAU_CH6_START_TRG_ON;
        SET1      0xF01B2.6          ;; 2 cycles
//  326 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock11
        ; ------------------------------------- Block: 12 cycles
        ; ------------------------------------- Total: 12 cycles
        REQUIRE __A_IF2
        REQUIRE __A_MK2
        REQUIRE __A_TS0
//  327 /***********************************************************************************************************************
//  328 * Function Name: R_TAU0_Channel6_Stop
//  329 * Description  : This function stops TAU0 channel 6 counter.
//  330 * Arguments    : None
//  331 * Return Value : None
//  332 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock12 Using cfiCommon0
          CFI Function _R_TAU0_Channel6_Stop
          CFI NoCalls
        CODE
//  333 void R_TAU0_Channel6_Stop(void)
//  334 {
_R_TAU0_Channel6_Stop:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  335     TT0 |= _0040_TAU_CH6_STOP_TRG_ON;
        SET1      0xF01B4.6          ;; 2 cycles
//  336     /* Mask channel 6 interrupt */
//  337     TMMK06 = 1U;    /* disable INTTM06 interrupt */
        SET1      0xFFFD5.0          ;; 2 cycles
//  338     TMIF06 = 0U;    /* clear INTTM06 interrupt flag */
        CLR1      0xFFFD1.0          ;; 2 cycles
//  339 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock12
        ; ------------------------------------- Block: 12 cycles
        ; ------------------------------------- Total: 12 cycles
        REQUIRE __A_TT0
        REQUIRE __A_MK2
        REQUIRE __A_IF2
//  340 
//  341 /***********************************************************************************************************************
//  342 * Function Name: R_TAU0_Channel6_UpdateInterval
//  343 * Description  : Update TDR06 interval
//  344 * Arguments    : None
//  345 * Return Value : None
//  346 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock13 Using cfiCommon1
          CFI Function _R_TAU0_Channel6_UpdateInterval
        CODE
//  347 void R_TAU0_Channel6_UpdateInterval(uint32_t us)
//  348 {
_R_TAU0_Channel6_UpdateInterval:
        ; * Stack frame (at entry) *
        ; Param size: 0
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+6
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+8
        ; Auto size: 8
        SUBW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+12
//  349     float counter;
//  350     
//  351     counter  = (float)us / TAU0_CHANNEL6_DIVISOR;
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
          CFI FunCall ?F_UL2F
        CALL      N:?F_UL2F          ;; 3 cycles
        MOVW      DE, #0x7E40        ;; 1 cycle
          CFI FunCall ___iar_fmex
        CALL      N:___iar_fmex      ;; 3 cycles
//  352     counter *= (float)TAU0_fCLK;
        MOVW      HL, #0x41C0        ;; 1 cycle
        PUSH      HL                 ;; 1 cycle
          CFI CFA SP+14
        MOVW      HL, #0x0           ;; 1 cycle
        PUSH      HL                 ;; 1 cycle
          CFI CFA SP+16
          CFI FunCall ?F_MUL
        CALL      N:?F_MUL           ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+12
        MOVW      HL, SP             ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  353     us = (uint32_t)counter;
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
          CFI FunCall ?F_F2UL
        CALL      N:?F_F2UL          ;; 3 cycles
//  354     us--;
        SUBW      AX, #0x1           ;; 1 cycle
        SKNC
        DECW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        SUBW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  355     
//  356     /* change the counter for update */
//  357     g_tau0_ch6_cascade_count_init = (uint16_t)(us / g_tau0_ch6_max_tdr_support);
        MOVW      DE, N:_g_tau0_ch6_max_tdr_support  ;; 1 cycle
        MOVW      HL, #0x0           ;; 1 cycle
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        DIVWU                        ;; 17 cycles
        NOP                          ;; 1 cycle
        MOVW      N:_g_tau0_ch6_cascade_count_init, AX  ;; 1 cycle
//  358     if (g_tau0_ch6_cascade_count > g_tau0_ch6_cascade_count_init)
        MOVW      HL, N:_g_tau0_ch6_cascade_count  ;; 1 cycle
        MOVW      AX, N:_g_tau0_ch6_cascade_count_init  ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BNC       ??R_TAU0_Channel6_UpdateInterval_0  ;; 4 cycles
        ; ------------------------------------- Block: 80 cycles
//  359     {
//  360         g_tau0_ch6_cascade_count = g_tau0_ch6_cascade_count_init;
        MOVW      AX, N:_g_tau0_ch6_cascade_count_init  ;; 1 cycle
        MOVW      N:_g_tau0_ch6_cascade_count, AX  ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
//  361     }
//  362     g_tau0_ch6_tdr_update_value = (uint16_t)(us % g_tau0_ch6_max_tdr_support);
??R_TAU0_Channel6_UpdateInterval_0:
        MOVW      DE, N:_g_tau0_ch6_max_tdr_support  ;; 1 cycle
        MOVW      HL, #0x0           ;; 1 cycle
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        DIVWU                        ;; 17 cycles
        NOP                          ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        MOVW      N:_g_tau0_ch6_tdr_update_value, AX  ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
//  363 }
        ADDW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock13
        ; ------------------------------------- Block: 33 cycles
        ; ------------------------------------- Total: 115 cycles

        SECTION `.iar_vfe_header`:DATA:NOALLOC:NOROOT(1)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
        DC32 0

        END
//  364 
//  365 /* End user code. Do not edit comment generated here */
// 
//  31 bytes in section .bss.noinit  (abs)
//   8 bytes in section .data
// 446 bytes in section .text
// 
// 446 bytes of CODE memory
//   8 bytes of DATA memory (+ 31 bytes shared)
//
//Errors: none
//Warnings: none
