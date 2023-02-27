///////////////////////////////////////////////////////////////////////////////
//
// IAR C/C++ Compiler V4.20.1.2260 for RL78               20/Dec/2020  22:38:47
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
//        BootCode\source_code\driver_files\r_cg_dsadc.c
//    Command line       =
//        -f C:\Users\laptop\AppData\Local\Temp\EWCB26.tmp ("D:\Dheeraj\New
//        folder\0. GDEV72 - BootCode\source_code\driver_files\r_cg_dsadc.c"
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
//        D:\Dheeraj\New folder\0. GDEV72 - BootCode\Debug\List\r_cg_dsadc.s
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

        EXTERN _R_DSADC_Create_UserInit
        EXTERN _R_DTCD0_Start
        EXTERN _R_WDT_Restart
        EXTERN _cal_coeff
        EXTERN _delay_us

        PUBLIC _R_DSADC_Channel0_Get_Result
        PUBLIC _R_DSADC_Channel0_Get_Result_16bit
        PUBLIC _R_DSADC_Channel1_Get_Result
        PUBLIC _R_DSADC_Channel1_Get_Result_16bit
        PUBLIC _R_DSADC_Channel2_Get_Result
        PUBLIC _R_DSADC_Channel2_Get_Result_16bit
        PUBLIC _R_DSADC_Channel3_Get_Result
        PUBLIC _R_DSADC_Channel3_Get_Result_16bit
        PUBLIC _R_DSADC_Create
        PUBLIC _R_DSADC_Reset
        PUBLIC _R_DSADC_Set_OperationOff
        PUBLIC _R_DSADC_Set_OperationOn
        PUBLIC _R_DSADC_Set_PowerOff
        PUBLIC _R_DSADC_Start
        PUBLIC _R_DSADC_Stop
        PUBLIC _R_DSADC_update_phase_correction
        PUBLIC __A_DSADCR0
        PUBLIC __A_DSADCR0H
        PUBLIC __A_DSADCR1
        PUBLIC __A_DSADCR1H
        PUBLIC __A_DSADCR2
        PUBLIC __A_DSADCR2H
        PUBLIC __A_DSADCR3
        PUBLIC __A_DSADCR3H
        PUBLIC __A_DSADGCR0
        PUBLIC __A_DSADGCR1
        PUBLIC __A_DSADHPFCR
        PUBLIC __A_DSADMR
        PUBLIC __A_DSADPHCR0
        PUBLIC __A_DSADPHCR1
        PUBLIC __A_DSADPHCR2
        PUBLIC __A_DSADPHCR3
        PUBLIC __A_IF2
        PUBLIC __A_IF3
        PUBLIC __A_MK2
        PUBLIC __A_MK3
        PUBLIC __A_PCKC
        PUBLIC __A_PER1
        PUBLIC __A_PR02
        PUBLIC __A_PR12
        PUBLIC __A_PRR1
        
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
        
// D:\Dheeraj\New folder\0. GDEV72 - BootCode\source_code\driver_files\r_cg_dsadc.c
//    1 /***********************************************************************************************************************
//    2 * DISCLAIMER
//    3 * This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products.
//    4 * No other uses are authorized. This software is owned by Renesas Electronics Corporation and is protected under all
//    5 * applicable laws, including copyright laws. 
//    6 * THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING THIS SOFTWARE, WHETHER EXPRESS, IMPLIED
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
//   17 * Copyright (C) 2015, 2018 Renesas Electronics Corporation. All rights reserved.
//   18 ***********************************************************************************************************************/
//   19 
//   20 /***********************************************************************************************************************
//   21 * File Name    : r_cg_dsadc.c
//   22 * Version      : Applilet4 for RL78/I1C V1.01.02.04 [24 May 2018]
//   23 * Device(s)    : R5F10NPJ
//   24 * Tool-Chain   : IAR Systems icc78k0r
//   25 * Description  : This file implements device driver for DSADC module.
//   26 * Creation Date: 01/09/2020
//   27 ***********************************************************************************************************************/
//   28 
//   29 /***********************************************************************************************************************
//   30 Includes
//   31 ***********************************************************************************************************************/
//   32 #include "r_cg_macrodriver.h"

        ASEGN `.bss.noinit`:DATA:NOROOT,0fffd0H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_IF2
// __no_init union <unnamed>#112 volatile __sfr _A_IF2
__A_IF2:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0fffd2H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_IF3
// __no_init union <unnamed>#118 volatile __sfr _A_IF3
__A_IF3:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0fffd4H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_MK2
// __no_init union <unnamed>#122 volatile __sfr _A_MK2
__A_MK2:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0fffd6H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_MK3
// __no_init union <unnamed>#128 volatile __sfr _A_MK3
__A_MK3:
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

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0098H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_PCKC
// __no_init union <unnamed>#263 volatile _A_PCKC
__A_PCKC:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f00faH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_PER1
// __no_init union <unnamed>#287 volatile _A_PER1
__A_PER1:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f00fbH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_PRR1
// __no_init union <unnamed>#289 volatile _A_PRR1
__A_PRR1:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f03c0H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_DSADMR
// __no_init union <unnamed>#575 volatile __no_bit_access _A_DSADMR
__A_DSADMR:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f03c2H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_DSADGCR0
// __no_init union <unnamed>#576 volatile __no_bit_access _A_DSADGCR0
__A_DSADGCR0:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f03c3H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_DSADGCR1
// __no_init union <unnamed>#577 volatile __no_bit_access _A_DSADGCR1
__A_DSADGCR1:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f03c5H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_DSADHPFCR
// __no_init union <unnamed>#578 volatile __no_bit_access _A_DSADHPFCR
__A_DSADHPFCR:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f03d0H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_DSADPHCR0
// __no_init union <unnamed>#582 volatile __no_bit_access _A_DSADPHCR0
__A_DSADPHCR0:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f03d2H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_DSADPHCR1
// __no_init union <unnamed>#583 volatile __no_bit_access _A_DSADPHCR1
__A_DSADPHCR1:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f03d4H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_DSADPHCR2
// __no_init union <unnamed>#584 volatile __no_bit_access _A_DSADPHCR2
__A_DSADPHCR2:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f03d6H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_DSADPHCR3
// __no_init union <unnamed>#585 volatile __no_bit_access _A_DSADPHCR3
__A_DSADPHCR3:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f03e0H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_DSADCR0
// __no_init union <unnamed>#586 const volatile __no_bit_access _A_DSADCR0
__A_DSADCR0:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f03e2H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_DSADCR0H
// __no_init union <unnamed>#590 const volatile __no_bit_access _A_DSADCR0H
__A_DSADCR0H:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f03e4H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_DSADCR1
// __no_init union <unnamed>#591 const volatile __no_bit_access _A_DSADCR1
__A_DSADCR1:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f03e6H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_DSADCR1H
// __no_init union <unnamed>#595 const volatile __no_bit_access _A_DSADCR1H
__A_DSADCR1H:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f03e8H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_DSADCR2
// __no_init union <unnamed>#596 const volatile __no_bit_access _A_DSADCR2
__A_DSADCR2:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f03eaH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_DSADCR2H
// __no_init union <unnamed>#600 const volatile __no_bit_access _A_DSADCR2H
__A_DSADCR2H:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f03ecH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_DSADCR3
// __no_init union <unnamed>#601 const volatile __no_bit_access _A_DSADCR3
__A_DSADCR3:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f03eeH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_DSADCR3H
// __no_init union <unnamed>#605 const volatile __no_bit_access _A_DSADCR3H
__A_DSADCR3H:
        DS 1
//   33 #include "r_cg_dsadc.h"
//   34 /* Start user code for include. Do not edit comment generated here */
//   35 /* End user code. Do not edit comment generated here */
//   36 #include "r_cg_userdefine.h"
//   37 
//   38 /***********************************************************************************************************************
//   39 Pragma directive
//   40 ***********************************************************************************************************************/
//   41 /* Start user code for pragma. Do not edit comment generated here */
//   42 /* End user code. Do not edit comment generated here */
//   43 
//   44 /***********************************************************************************************************************
//   45 Global variables and functions
//   46 ***********************************************************************************************************************/
//   47 /* Start user code for global. Do not edit comment generated here */
//   48 /* End user code. Do not edit comment generated here */
//   49 
//   50 /***********************************************************************************************************************
//   51 * Function Name: R_DSADC_Create
//   52 * Description  : This function initializes the DSAD converter.
//   53 * Arguments    : None
//   54 * Return Value : None
//   55 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock0 Using cfiCommon0
          CFI Function _R_DSADC_Create
        CODE
//   56 void R_DSADC_Create(void)
//   57 {
_R_DSADC_Create:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//   58     DSADCK = 0U;
        CLR1      0xF0098.0          ;; 2 cycles
//   59     DSADRES = 1U;   /* reset DSAD converter */
        SET1      0xF00FB.0          ;; 2 cycles
//   60     DSADRES = 0U;   /* reset release of DSAD converter */
        CLR1      0xF00FB.0          ;; 2 cycles
//   61     DSADCEN = 1U;   /* enables input clock supply */
        SET1      0xF00FA.0          ;; 2 cycles
//   62     DSAMK = 1U;     /* disable INTDSAD interrupt */
        SET1      0xFFFD4.0          ;; 2 cycles
//   63     DSAIF = 0U;     /* clear INTDSAD interrupt flag */
        CLR1      0xFFFD0.0          ;; 2 cycles
//   64     DSAZMK0 = 1U;   /* disable INTDSADZC0 interrupt */
        SET1      0xFFFD6.0          ;; 2 cycles
//   65     DSAZIF0 = 0U;   /* clear INTDSADZC0 interrupt flag */
        CLR1      0xFFFD2.0          ;; 2 cycles
//   66     DSAZMK1 = 1U;   /* disable INTDSADZC1 interrupt */
        SET1      0xFFFD6.1          ;; 2 cycles
//   67     DSAZIF1 = 0U;   /* clear INTDSADZC1 interrupt flag */
        CLR1      0xFFFD2.1          ;; 2 cycles
//   68     /* Set INTDSAD high priority */
//   69     DSAPR1 = 0U;
        CLR1      0xFFFDC.0          ;; 2 cycles
//   70     DSAPR0 = 0U;
        CLR1      0xFFFD8.0          ;; 2 cycles
//   71     DSADMR = _0000_DSAD_SAMPLING_FREQUENCY_0 | _4000_DSAD_RESOLUTION_16BIT;
        MOVW      AX, #0x4000        ;; 1 cycle
        MOVW      0x3C0, AX          ;; 1 cycle
//   72     DSADGCR0 = _00_DSAD_CH1_PGAGAIN_1 | _00_DSAD_CH0_PGAGAIN_1;
        MOV       0x3C2, #0x0        ;; 1 cycle
//   73     DSADGCR1 = _40_DSAD_CH1_PGAGAIN_16 | _00_DSAD_CH2_PGAGAIN_1;
        MOV       0x3C3, #0x40       ;; 1 cycle
//   74     /* Enabling HPF */
//   75 //    DSADHPFCR = _C0_DSAD_CUTOFF_FREQUENCY_3 | _00_DSAD_CH3_HIGHPASS_FILTER_ENABLE | 
//   76 //                _00_DSAD_CH2_HIGHPASS_FILTER_ENABLE | _00_DSAD_CH1_HIGHPASS_FILTER_ENABLE | 
//   77 //                _00_DSAD_CH0_HIGHPASS_FILTER_ENABLE;
//   78     /* Disabling HPF */
//   79     DSADHPFCR = _00_DSAD_CUTOFF_FREQUENCY_0 | _08_DSAD_CH3_HIGHPASS_FILTER_DISABLE | 
//   80     _04_DSAD_CH2_HIGHPASS_FILTER_DISABLE | _02_DSAD_CH1_HIGHPASS_FILTER_DISABLE | 
//   81     _01_DSAD_CH0_HIGHPASS_FILTER_DISABLE;
        MOV       0x3C5, #0xF        ;; 1 cycle
//   82     
//   83     DSADPHCR0 = cal_coeff.Rph.phase_correction; // 701
        MOVW      AX, N:_cal_coeff+12  ;; 1 cycle
        MOVW      0x3D0, AX          ;; 1 cycle
//   84     DSADPHCR1 = cal_coeff.Yph.phase_correction;
        MOVW      AX, N:_cal_coeff+24  ;; 1 cycle
        MOVW      0x3D2, AX          ;; 1 cycle
//   85     DSADPHCR2 = cal_coeff.Bph.phase_correction;
        MOVW      AX, N:_cal_coeff+36  ;; 1 cycle
        MOVW      0x3D4, AX          ;; 1 cycle
//   86     DSADPHCR3 = _0000_DSAD_PHCR3_VALUE;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      0x3D6, AX          ;; 1 cycle
//   87 
//   88     R_DSADC_Create_UserInit();
          CFI FunCall _R_DSADC_Create_UserInit
        CALL      _R_DSADC_Create_UserInit  ;; 3 cycles
//   89 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock0
        ; ------------------------------------- Block: 46 cycles
        ; ------------------------------------- Total: 46 cycles
        REQUIRE __A_PCKC
        REQUIRE __A_PRR1
        REQUIRE __A_PER1
        REQUIRE __A_MK2
        REQUIRE __A_IF2
        REQUIRE __A_MK3
        REQUIRE __A_IF3
        REQUIRE __A_PR12
        REQUIRE __A_PR02
        REQUIRE __A_DSADMR
        REQUIRE __A_DSADGCR0
        REQUIRE __A_DSADGCR1
        REQUIRE __A_DSADHPFCR
        REQUIRE __A_DSADPHCR0
        REQUIRE __A_DSADPHCR1
        REQUIRE __A_DSADPHCR2
        REQUIRE __A_DSADPHCR3
//   90 /***********************************************************************************************************************
//   91 * Function Name: R_DSADC_Start
//   92 * Description  : This function starts the DSAD converter.
//   93 * Arguments    : None
//   94 * Return Value : None
//   95 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock1 Using cfiCommon0
          CFI Function _R_DSADC_Start
        CODE
//   96 void R_DSADC_Start(void)
//   97 {
_R_DSADC_Start:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 2
        SUBW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+6
//   98 //      DSAIF = 0U;     /* clear INTDSAD interrupt flag */
//   99 //      DSAMK = 0U;     /* enable INTDSAD interrupt */
//  100 //      DSADMR |= _0008_DSAD_CH3_OPERATION | _0004_DSAD_CH2_OPERATION | _0002_DSAD_CH1_OPERATION | _0001_DSAD_CH0_OPERATION;
//  101   
//  102   uint8_t wait;
//  103   
//  104   DSAMK = 1U;  /* disable INTDSAD interrupt */
        SET1      0xFFFD4.0          ;; 2 cycles
//  105   DSAIF = 0U;  /* clear INTDSAD interrupt flag */
        CLR1      0xFFFD0.0          ;; 2 cycles
//  106   
//  107   
//  108   DSADMR &= ~(_0008_DSAD_CH3_OPERATION | _0004_DSAD_CH2_OPERATION | _0002_DSAD_CH1_OPERATION | _0001_DSAD_CH0_OPERATION);    /* Stop conversion */
        MOVW      AX, 0x3C0          ;; 1 cycle
        AND       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        AND       A, #0xF0           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      0x3C0, AX          ;; 1 cycle
//  109   DSADMR |= _0008_DSAD_CH3_OPERATION | _0004_DSAD_CH2_OPERATION | _0002_DSAD_CH1_OPERATION | _0001_DSAD_CH0_OPERATION;        /* Start conversion ADC channel 01-- */
        MOVW      AX, 0x3C0          ;; 1 cycle
        OR        A, #0x0            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        OR        A, #0xF            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      0x3C0, AX          ;; 1 cycle
//  110   
//  111   wait = 0x80;
        MOV       A, #0x80           ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
          CFI FunCall _R_WDT_Restart
        ; ------------------------------------- Block: 19 cycles
//  112   while (1)
//  113   {
//  114     wdt_restart();
??R_DSADC_Start_0:
        CALL      _R_WDT_Restart     ;; 3 cycles
//  115     if (DSAIF == 1)
        MOVW      HL, #0xFFD0        ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??R_DSADC_update_phase_correction_0  ;; 4 cycles
        ; ------------------------------------- Block: 9 cycles
//  116     {
//  117       DSAIF = 0U;  /* Clear INTSAD interrupt flag */
        CLR1      0xFFFD0.0          ;; 2 cycles
//  118       delay_us(20);
        MOVW      AX, #0x14          ;; 1 cycle
          CFI FunCall _delay_us
        CALL      _delay_us          ;; 3 cycles
//  119       R_DTCD0_Start();
          CFI FunCall _R_DTCD0_Start
        CALL      _R_DTCD0_Start     ;; 3 cycles
//  120       //TS0L = g_dtc_tau01_trigger;
//  121       wait--;
        MOV       A, [SP]            ;; 1 cycle
        DEC       A                  ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        ; ------------------------------------- Block: 12 cycles
//  122     }
//  123     if (wait == 0)
??R_DSADC_update_phase_correction_0:
        MOV       A, [SP]            ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        BNZ       ??R_DSADC_Start_0  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  124     {
//  125       break;
//  126     }
//  127   }
//  128   DSAMK = 0U;  /* Enable INTSAD interrupt */
        CLR1      0xFFFD4.0          ;; 2 cycles
//  129   
//  130   
//  131 }
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock1
        ; ------------------------------------- Block: 9 cycles
        ; ------------------------------------- Total: 55 cycles
        REQUIRE __A_MK2
        REQUIRE __A_IF2
        REQUIRE __A_DSADMR
//  132 /***********************************************************************************************************************
//  133 * Function Name: R_DSADC_Stop
//  134 * Description  : This function stops the DSAD converter.
//  135 * Arguments    : None
//  136 * Return Value : None
//  137 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock2 Using cfiCommon0
          CFI Function _R_DSADC_Stop
          CFI NoCalls
        CODE
//  138 void R_DSADC_Stop(void)
//  139 {
_R_DSADC_Stop:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  140     DSADMR &= (uint16_t) ~(_0008_DSAD_CH3_OPERATION | _0004_DSAD_CH2_OPERATION | _0002_DSAD_CH1_OPERATION | 
//  141               _0001_DSAD_CH0_OPERATION);
        MOVW      AX, 0x3C0          ;; 1 cycle
        AND       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        AND       A, #0xF0           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      0x3C0, AX          ;; 1 cycle
//  142     DSAMK = 1U;     /* disable INTDSAD interrupt */
        SET1      0xFFFD4.0          ;; 2 cycles
//  143     DSAIF = 0U;     /* clear INTDSAD interrupt flag */
        CLR1      0xFFFD0.0          ;; 2 cycles
//  144 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock2
        ; ------------------------------------- Block: 16 cycles
        ; ------------------------------------- Total: 16 cycles
        REQUIRE __A_DSADMR
        REQUIRE __A_MK2
        REQUIRE __A_IF2
//  145 /***********************************************************************************************************************
//  146 * Function Name: R_DSADC_Set_OperationOn
//  147 * Description  : This function power-on control.
//  148 * Arguments    : None
//  149 * Return Value : None
//  150 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock3 Using cfiCommon0
          CFI Function _R_DSADC_Set_OperationOn
          CFI NoCalls
        CODE
//  151 void R_DSADC_Set_OperationOn(void)
//  152 {
_R_DSADC_Set_OperationOn:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  153     DSADMR |= _0800_DSAD_CH3_POWER_ON | _0400_DSAD_CH2_POWER_ON | _0200_DSAD_CH1_POWER_ON | _0100_DSAD_CH0_POWER_ON;
        MOVW      AX, 0x3C0          ;; 1 cycle
        OR        A, #0xF            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        OR        A, #0x0            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      0x3C0, AX          ;; 1 cycle
//  154 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock3
        ; ------------------------------------- Block: 12 cycles
        ; ------------------------------------- Total: 12 cycles
        REQUIRE __A_DSADMR
//  155 /***********************************************************************************************************************
//  156 * Function Name: R_DSADC_Set_OperationOff
//  157 * Description  : This function power-down control.
//  158 * Arguments    : None
//  159 * Return Value : None
//  160 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock4 Using cfiCommon0
          CFI Function _R_DSADC_Set_OperationOff
          CFI NoCalls
        CODE
//  161 void R_DSADC_Set_OperationOff(void)
//  162 {
_R_DSADC_Set_OperationOff:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  163     DSADMR &= (uint16_t) ~(_0800_DSAD_CH3_POWER_ON | _0400_DSAD_CH2_POWER_ON | _0200_DSAD_CH1_POWER_ON | 
//  164               _0100_DSAD_CH0_POWER_ON);
        MOVW      AX, 0x3C0          ;; 1 cycle
        AND       A, #0xF0           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        AND       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      0x3C0, AX          ;; 1 cycle
//  165 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock4
        ; ------------------------------------- Block: 12 cycles
        ; ------------------------------------- Total: 12 cycles
        REQUIRE __A_DSADMR
//  166 /***********************************************************************************************************************
//  167 * Function Name: R_DSADC_Channel0_Get_Result
//  168 * Description  : This function returns the conversion result in the buffer.
//  169 * Arguments    : buffer -
//  170 *                    the address where to write the conversion result
//  171 * Return Value : None
//  172 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock5 Using cfiCommon0
          CFI Function _R_DSADC_Channel0_Get_Result
          CFI NoCalls
        CODE
//  173 void R_DSADC_Channel0_Get_Result(uint32_t * const buffer)
//  174 {
_R_DSADC_Channel0_Get_Result:
        ; * Stack frame (at entry) *
        ; Param size: 0
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+6
        ; Auto size: 2
//  175     *buffer = DSADCR0H;
        MOV       X, 0x3E2           ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  176     *buffer = (uint32_t)((*buffer << 16U) + DSADCR0);
        MOVW      DE, 0x3E0          ;; 1 cycle
        MOVW      HL, #0x0           ;; 1 cycle
        PUSH      HL                 ;; 1 cycle
          CFI CFA SP+8
        PUSH      DE                 ;; 1 cycle
          CFI CFA SP+10
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [HL+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [HL]           ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        CLRW      AX                 ;; 1 cycle
        POP       DE                 ;; 1 cycle
          CFI CFA SP+8
        POP       HL                 ;; 1 cycle
          CFI CFA SP+6
        ADDW      AX, DE             ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  177 }
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock5
        ; ------------------------------------- Block: 47 cycles
        ; ------------------------------------- Total: 47 cycles
        REQUIRE __A_DSADCR0H
        REQUIRE __A_DSADCR0
//  178 /***********************************************************************************************************************
//  179 * Function Name: R_DSADC_Channel1_Get_Result
//  180 * Description  : This function returns the conversion result in the buffer.
//  181 * Arguments    : buffer -
//  182 *                    the address where to write the conversion result
//  183 * Return Value : None
//  184 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock6 Using cfiCommon0
          CFI Function _R_DSADC_Channel1_Get_Result
          CFI NoCalls
        CODE
//  185 void R_DSADC_Channel1_Get_Result(uint32_t * const buffer)
//  186 {
_R_DSADC_Channel1_Get_Result:
        ; * Stack frame (at entry) *
        ; Param size: 0
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+6
        ; Auto size: 2
//  187     *buffer = DSADCR1H;
        MOV       X, 0x3E6           ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  188     *buffer = (uint32_t)((*buffer << 16U) + DSADCR1);
        MOVW      DE, 0x3E4          ;; 1 cycle
        MOVW      HL, #0x0           ;; 1 cycle
        PUSH      HL                 ;; 1 cycle
          CFI CFA SP+8
        PUSH      DE                 ;; 1 cycle
          CFI CFA SP+10
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [HL+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [HL]           ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        CLRW      AX                 ;; 1 cycle
        POP       DE                 ;; 1 cycle
          CFI CFA SP+8
        POP       HL                 ;; 1 cycle
          CFI CFA SP+6
        ADDW      AX, DE             ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  189 }
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock6
        ; ------------------------------------- Block: 47 cycles
        ; ------------------------------------- Total: 47 cycles
        REQUIRE __A_DSADCR1H
        REQUIRE __A_DSADCR1
//  190 /***********************************************************************************************************************
//  191 * Function Name: R_DSADC_Channel2_Get_Result
//  192 * Description  : This function returns the conversion result in the buffer.
//  193 * Arguments    : buffer -
//  194 *                    the address where to write the conversion result
//  195 * Return Value : None
//  196 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock7 Using cfiCommon0
          CFI Function _R_DSADC_Channel2_Get_Result
          CFI NoCalls
        CODE
//  197 void R_DSADC_Channel2_Get_Result(uint32_t * const buffer)
//  198 {
_R_DSADC_Channel2_Get_Result:
        ; * Stack frame (at entry) *
        ; Param size: 0
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+6
        ; Auto size: 2
//  199     *buffer = DSADCR2H;
        MOV       X, 0x3EA           ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  200     *buffer = (uint32_t)((*buffer << 16U) + DSADCR2);
        MOVW      DE, 0x3E8          ;; 1 cycle
        MOVW      HL, #0x0           ;; 1 cycle
        PUSH      HL                 ;; 1 cycle
          CFI CFA SP+8
        PUSH      DE                 ;; 1 cycle
          CFI CFA SP+10
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [HL+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [HL]           ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        CLRW      AX                 ;; 1 cycle
        POP       DE                 ;; 1 cycle
          CFI CFA SP+8
        POP       HL                 ;; 1 cycle
          CFI CFA SP+6
        ADDW      AX, DE             ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  201 }
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock7
        ; ------------------------------------- Block: 47 cycles
        ; ------------------------------------- Total: 47 cycles
        REQUIRE __A_DSADCR2H
        REQUIRE __A_DSADCR2
//  202 /***********************************************************************************************************************
//  203 * Function Name: R_DSADC_Channel3_Get_Result
//  204 * Description  : This function returns the conversion result in the buffer.
//  205 * Arguments    : buffer -
//  206 *                    the address where to write the conversion result
//  207 * Return Value : None
//  208 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock8 Using cfiCommon0
          CFI Function _R_DSADC_Channel3_Get_Result
          CFI NoCalls
        CODE
//  209 void R_DSADC_Channel3_Get_Result(uint32_t * const buffer)
//  210 {
_R_DSADC_Channel3_Get_Result:
        ; * Stack frame (at entry) *
        ; Param size: 0
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+6
        ; Auto size: 2
//  211     *buffer = DSADCR3H;
        MOV       X, 0x3EE           ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  212     *buffer = (uint32_t)((*buffer << 16U) + DSADCR3);
        MOVW      DE, 0x3EC          ;; 1 cycle
        MOVW      HL, #0x0           ;; 1 cycle
        PUSH      HL                 ;; 1 cycle
          CFI CFA SP+8
        PUSH      DE                 ;; 1 cycle
          CFI CFA SP+10
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [HL+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [HL]           ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        CLRW      AX                 ;; 1 cycle
        POP       DE                 ;; 1 cycle
          CFI CFA SP+8
        POP       HL                 ;; 1 cycle
          CFI CFA SP+6
        ADDW      AX, DE             ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  213 }
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock8
        ; ------------------------------------- Block: 47 cycles
        ; ------------------------------------- Total: 47 cycles
        REQUIRE __A_DSADCR3H
        REQUIRE __A_DSADCR3
//  214 /***********************************************************************************************************************
//  215 * Function Name: R_DSADC_Channel0_Get_Result_16bit
//  216 * Description  : This function returns the higher 16 bits conversion result.
//  217 * Arguments    : buffer -
//  218 *                    the address where to write the conversion result
//  219 * Return Value : None
//  220 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock9 Using cfiCommon0
          CFI Function _R_DSADC_Channel0_Get_Result_16bit
          CFI NoCalls
        CODE
//  221 void R_DSADC_Channel0_Get_Result_16bit(uint16_t * const buffer)
//  222 {
_R_DSADC_Channel0_Get_Result_16bit:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
        MOVW      HL, AX             ;; 1 cycle
//  223     *buffer = DSADCR0;
        MOVW      AX, 0x3E0          ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
//  224 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock9
        ; ------------------------------------- Block: 9 cycles
        ; ------------------------------------- Total: 9 cycles
        REQUIRE __A_DSADCR0
//  225 /***********************************************************************************************************************
//  226 * Function Name: R_DSADC_Channel1_Get_Result_16bit
//  227 * Description  : This function returns the higher 16 bits conversion result.
//  228 * Arguments    : buffer -
//  229 *                    the address where to write the conversion result
//  230 * Return Value : None
//  231 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock10 Using cfiCommon0
          CFI Function _R_DSADC_Channel1_Get_Result_16bit
          CFI NoCalls
        CODE
//  232 void R_DSADC_Channel1_Get_Result_16bit(uint16_t * const buffer)
//  233 {
_R_DSADC_Channel1_Get_Result_16bit:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
        MOVW      HL, AX             ;; 1 cycle
//  234     *buffer = DSADCR1;
        MOVW      AX, 0x3E4          ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
//  235 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock10
        ; ------------------------------------- Block: 9 cycles
        ; ------------------------------------- Total: 9 cycles
        REQUIRE __A_DSADCR1
//  236 /***********************************************************************************************************************
//  237 * Function Name: R_DSADC_Channel2_Get_Result_16bit
//  238 * Description  : This function returns the higher 16 bits conversion result.
//  239 * Arguments    : buffer -
//  240 *                    the address where to write the conversion result
//  241 * Return Value : None
//  242 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock11 Using cfiCommon0
          CFI Function _R_DSADC_Channel2_Get_Result_16bit
          CFI NoCalls
        CODE
//  243 void R_DSADC_Channel2_Get_Result_16bit(uint16_t * const buffer)
//  244 {
_R_DSADC_Channel2_Get_Result_16bit:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
        MOVW      HL, AX             ;; 1 cycle
//  245     *buffer = DSADCR2;
        MOVW      AX, 0x3E8          ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
//  246 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock11
        ; ------------------------------------- Block: 9 cycles
        ; ------------------------------------- Total: 9 cycles
        REQUIRE __A_DSADCR2
//  247 /***********************************************************************************************************************
//  248 * Function Name: R_DSADC_Channel3_Get_Result_16bit
//  249 * Description  : This function returns the higher 16 bits conversion result.
//  250 * Arguments    : buffer -
//  251 *                    the address where to write the conversion result
//  252 * Return Value : None
//  253 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock12 Using cfiCommon0
          CFI Function _R_DSADC_Channel3_Get_Result_16bit
          CFI NoCalls
        CODE
//  254 void R_DSADC_Channel3_Get_Result_16bit(uint16_t * const buffer)
//  255 {
_R_DSADC_Channel3_Get_Result_16bit:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
        MOVW      HL, AX             ;; 1 cycle
//  256     *buffer = DSADCR3;
        MOVW      AX, 0x3EC          ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
//  257 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock12
        ; ------------------------------------- Block: 9 cycles
        ; ------------------------------------- Total: 9 cycles
        REQUIRE __A_DSADCR3
//  258 /***********************************************************************************************************************
//  259 * Function Name: R_DSADC_Set_PowerOff
//  260 * Description  : This function stops the clock supplied for DSAD.
//  261 * Arguments    : None
//  262 * Return Value : None
//  263 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock13 Using cfiCommon0
          CFI Function _R_DSADC_Set_PowerOff
          CFI NoCalls
        CODE
//  264 void R_DSADC_Set_PowerOff(void)
//  265 {
_R_DSADC_Set_PowerOff:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  266     DSADCEN = 0U;   /* stops input clock supply */
        CLR1      0xF00FA.0          ;; 2 cycles
//  267 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock13
        ; ------------------------------------- Block: 8 cycles
        ; ------------------------------------- Total: 8 cycles
        REQUIRE __A_PER1
//  268 /***********************************************************************************************************************
//  269 * Function Name: R_DSADC_Reset
//  270 * Description  : This function reset DSAD module.
//  271 * Arguments    : None
//  272 * Return Value : None
//  273 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock14 Using cfiCommon0
          CFI Function _R_DSADC_Reset
          CFI NoCalls
        CODE
//  274 void R_DSADC_Reset(void)
//  275 {
_R_DSADC_Reset:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  276     DSADRES = 1U;   /* reset DSAD converter */
        SET1      0xF00FB.0          ;; 2 cycles
//  277 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock14
        ; ------------------------------------- Block: 8 cycles
        ; ------------------------------------- Total: 8 cycles
        REQUIRE __A_PRR1
//  278 
//  279 /* Start user code for adding. Do not edit comment generated here */
//  280 /* End user code. Do not edit comment generated here */

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock15 Using cfiCommon1
          CFI Function _R_DSADC_update_phase_correction
          CFI NoCalls
        CODE
//  281 void R_DSADC_update_phase_correction(us8 phase, us16 value)
//  282 {
_R_DSADC_update_phase_correction:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  283   if(phase == PHASE_R)
        CMP0      A                  ;; 1 cycle
        BNZ       ??R_DSADC_update_phase_correction_1  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  284   {
//  285     DSADPHCR0 = value;
        XCHW      AX, BC             ;; 1 cycle
        MOVW      0x3D0, AX          ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        RET                          ;; 6 cycles
        ; ------------------------------------- Block: 9 cycles
//  286   }
//  287   else if(phase == PHASE_Y)
??R_DSADC_update_phase_correction_1:
        CMP       A, #0x1            ;; 1 cycle
        BNZ       ??R_DSADC_update_phase_correction_2  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  288   {
//  289     DSADPHCR1 = value;
        XCHW      AX, BC             ;; 1 cycle
        MOVW      0x3D2, AX          ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        RET                          ;; 6 cycles
        ; ------------------------------------- Block: 9 cycles
//  290   }
//  291   else if(phase == PHASE_B)
??R_DSADC_update_phase_correction_2:
        CMP       A, #0x2            ;; 1 cycle
        BNZ       ??R_DSADC_update_phase_correction_3  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  292   {
//  293     DSADPHCR2 = value;
        XCHW      AX, BC             ;; 1 cycle
        MOVW      0x3D4, AX          ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ; ------------------------------------- Block: 3 cycles
//  294   }
//  295 }
??R_DSADC_update_phase_correction_3:
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock15
        ; ------------------------------------- Block: 6 cycles
        ; ------------------------------------- Total: 42 cycles
        REQUIRE __A_DSADPHCR0
        REQUIRE __A_DSADPHCR1
        REQUIRE __A_DSADPHCR2

        SECTION `.iar_vfe_header`:DATA:NOALLOC:NOROOT(1)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
        DC32 0

        END
// 
//  40 bytes in section .bss.noinit  (abs)
// 492 bytes in section .text
// 
// 492 bytes of CODE memory
//   0 bytes of DATA memory (+ 40 bytes shared)
//
//Errors: none
//Warnings: none
