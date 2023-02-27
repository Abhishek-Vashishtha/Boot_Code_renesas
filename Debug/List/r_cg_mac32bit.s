///////////////////////////////////////////////////////////////////////////////
//
// IAR C/C++ Compiler V4.20.1.2260 for RL78               20/Dec/2020  22:38:54
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
//        BootCode\source_code\driver_files\r_cg_mac32bit.c
//    Command line       =
//        -f C:\Users\laptop\AppData\Local\Temp\EWE920.tmp ("D:\Dheeraj\New
//        folder\0. GDEV72 - BootCode\source_code\driver_files\r_cg_mac32bit.c"
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
//        D:\Dheeraj\New folder\0. GDEV72 - BootCode\Debug\List\r_cg_mac32bit.s
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

        PUBLIC _R_MAC32Bit_Create
        PUBLIC _R_MAC32Bit_MACSigned
        PUBLIC _R_MAC32Bit_MACUnsigned
        PUBLIC _R_MAC32Bit_MULSigned
        PUBLIC _R_MAC32Bit_MULUnsigned
        PUBLIC _R_MAC32Bit_Reset
        PUBLIC _R_MAC32Bit_Set_PowerOff
        PUBLIC __A_IF2
        PUBLIC __A_MAC32SH
        PUBLIC __A_MAC32SL
        PUBLIC __A_MAC32UH
        PUBLIC __A_MAC32UL
        PUBLIC __A_MK2
        PUBLIC __A_MUL32SH
        PUBLIC __A_MUL32SL
        PUBLIC __A_MUL32UH
        PUBLIC __A_MUL32UL
        PUBLIC __A_MULBH
        PUBLIC __A_MULBL
        PUBLIC __A_MULC
        PUBLIC __A_MULR0
        PUBLIC __A_MULR1
        PUBLIC __A_MULR2
        PUBLIC __A_MULR3
        PUBLIC __A_PER2
        PUBLIC __A_PRR2
        PUBLIC _mac_union_2int
        PUBLIC _mac_union_4int
        
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
        
// D:\Dheeraj\New folder\0. GDEV72 - BootCode\source_code\driver_files\r_cg_mac32bit.c
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
//   21 * File Name    : r_cg_mac32bit.c
//   22 * Version      : Applilet4 for RL78/I1C V1.01.02.04 [24 May 2018]
//   23 * Device(s)    : R5F10NPJ
//   24 * Tool-Chain   : IAR Systems icc78k0r
//   25 * Description  : This file implements device driver for MAC32bit module.
//   26 * Creation Date: 03/16/2020
//   27 ***********************************************************************************************************************/
//   28 
//   29 /***********************************************************************************************************************
//   30 Includes
//   31 ***********************************************************************************************************************/
//   32 #include "r_cg_macrodriver.h"

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

        ASEGN `.bss.noinit`:DATA:NOROOT,0f00fcH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_PER2
// __no_init union <unnamed>#291 volatile _A_PER2
__A_PER2:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f00fdH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_PRR2
// __no_init union <unnamed>#293 volatile _A_PRR2
__A_PRR2:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0280H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_MUL32UL
// __no_init union <unnamed>#495 volatile __no_bit_access _A_MUL32UL
__A_MUL32UL:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0282H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_MUL32UH
// __no_init union <unnamed>#496 volatile __no_bit_access _A_MUL32UH
__A_MUL32UH:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0284H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_MUL32SL
// __no_init union <unnamed>#497 volatile __no_bit_access _A_MUL32SL
__A_MUL32SL:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0286H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_MUL32SH
// __no_init union <unnamed>#498 volatile __no_bit_access _A_MUL32SH
__A_MUL32SH:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0288H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_MAC32UL
// __no_init union <unnamed>#499 volatile __no_bit_access _A_MAC32UL
__A_MAC32UL:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f028aH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_MAC32UH
// __no_init union <unnamed>#500 volatile __no_bit_access _A_MAC32UH
__A_MAC32UH:
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
//   33 #include "r_cg_mac32bit.h"
//   34 /* Start user code for include. Do not edit comment generated here */
//   35 /* End user code. Do not edit comment generated here */
//   36 #include "r_cg_userdefine.h"
//   37 
//   38 
//   39 /***********************************************************************************************************************
//   40 Pragma directive
//   41 ***********************************************************************************************************************/
//   42 /* Start user code for pragma. Do not edit comment generated here */
//   43 /* End user code. Do not edit comment generated here */
//   44 
//   45 /***********************************************************************************************************************
//   46 Global variables and functions
//   47 ***********************************************************************************************************************/
//   48 /* Start user code for global. Do not edit comment generated here */
//   49 /* End user code. Do not edit comment generated here */

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   50 union_4int mac_union_4int;
_mac_union_4int:
        DS 8

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   51 union_2int mac_union_2int;
_mac_union_2int:
        DS 4
//   52 /***********************************************************************************************************************
//   53 * Function Name: R_MAC32Bit_Create
//   54 * Description  : This function initializes the 32bitMultiply module.
//   55 * Arguments    : None
//   56 * Return Value : None
//   57 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock0 Using cfiCommon0
          CFI Function _R_MAC32Bit_Create
          CFI NoCalls
        CODE
//   58 void R_MAC32Bit_Create(void)
//   59 {
_R_MAC32Bit_Create:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//   60     MACEN = 1U;     /* enables input clock supply */
        SET1      0xF00FC.2          ;; 2 cycles
//   61     MACMK = 1U;     /* disable INTMACLOF interrupt */
        SET1      0xFFFD5.5          ;; 2 cycles
//   62     MACIF = 0U;     /* clear INTMACLOF interrupt flag */
        CLR1      0xFFFD1.5          ;; 2 cycles
//   63     MULFRAC = 0U;   /* fixed point mode disabled */
        CLR1      0xF029A.4          ;; 2 cycles
//   64 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock0
        ; ------------------------------------- Block: 14 cycles
        ; ------------------------------------- Total: 14 cycles
        REQUIRE __A_PER2
        REQUIRE __A_MK2
        REQUIRE __A_IF2
        REQUIRE __A_MULC
//   65 /***********************************************************************************************************************
//   66 * Function Name: R_MAC32Bit_Reset
//   67 * Description  : This function resets the 32bitMultiply module.
//   68 * Arguments    : None
//   69 * Return Value : None
//   70 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock1 Using cfiCommon0
          CFI Function _R_MAC32Bit_Reset
          CFI NoCalls
        CODE
//   71 void R_MAC32Bit_Reset(void)
//   72 {
_R_MAC32Bit_Reset:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//   73     MACRES = 1U;    /* reset 32-bit multiplier and accumulator */
        SET1      0xF00FD.2          ;; 2 cycles
//   74 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock1
        ; ------------------------------------- Block: 8 cycles
        ; ------------------------------------- Total: 8 cycles
        REQUIRE __A_PRR2
//   75 /***********************************************************************************************************************
//   76 * Function Name: R_MAC32Bit_Set_PowerOff
//   77 * Description  : This function stops supply of input clock and all SFR are reset.
//   78 * Arguments    : None
//   79 * Return Value : None
//   80 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock2 Using cfiCommon0
          CFI Function _R_MAC32Bit_Set_PowerOff
          CFI NoCalls
        CODE
//   81 void R_MAC32Bit_Set_PowerOff(void)
//   82 {
_R_MAC32Bit_Set_PowerOff:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//   83     MACEN = 0U;     /* stops input clock supply */
        CLR1      0xF00FC.2          ;; 2 cycles
//   84 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock2
        ; ------------------------------------- Block: 8 cycles
        ; ------------------------------------- Total: 8 cycles
        REQUIRE __A_PER2
//   85 /***********************************************************************************************************************
//   86 * Function Name: R_MAC32Bit_MULUnsigned
//   87 * Description  : This function caculates unsigned values in multiplication mode.
//   88 * Arguments    : data_a -
//   89 *                    Multiplication data register A value
//   90 *                data_b -
//   91 *                    Multiplication data register B value
//   92 *                buffer_64bit -
//   93 *                    Multiplication result value
//   94 * Return Value : None
//   95 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock3 Using cfiCommon1
          CFI Function _R_MAC32Bit_MULUnsigned
          CFI NoCalls
        CODE
//   96 void R_MAC32Bit_MULUnsigned(uint32_t data_a, uint32_t data_b, mac32bit_uint64_t * buffer_64bit)
//   97 {
_R_MAC32Bit_MULUnsigned:
        ; * Stack frame (at entry) *
        ; Param size: 4
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+6
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+8
        ; Auto size: 4
//   98     /* Set data_a */
//   99     MUL32UL = (uint16_t)data_a;
        MOVW      AX, [SP]           ;; 1 cycle
        MOVW      0x280, AX          ;; 1 cycle
//  100     MUL32UH = (uint16_t)(data_a >> 16U);
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        CLRW      AX                 ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      0x282, AX          ;; 1 cycle
//  101 
//  102     /* Set data_b */
//  103     MULBL = (uint16_t)data_b;
        MOVW      AX, [SP+0x08]      ;; 1 cycle
        MOVW      0xFFF3C, AX        ;; 1 cycle
//  104     MULBH = (uint16_t)(data_b >> 16U);
        MOVW      AX, [SP+0x0A]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x08]      ;; 1 cycle
        CLRW      AX                 ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      0xFFF3E, AX        ;; 1 cycle
//  105 
//  106     /* Wait 2 more cycles */
//  107     NOP();
        NOP                          ;; 1 cycle
//  108     NOP();
        NOP                          ;; 1 cycle
//  109 
//  110     /* Read MULR0 */
//  111     buffer_64bit -> low_low = MULR0;
        MOVW      AX, 0x290          ;; 1 cycle
        MOVW      [DE], AX           ;; 1 cycle
//  112 
//  113     /* Wait 1 cycle */
//  114     NOP();
        NOP                          ;; 1 cycle
//  115 
//  116     /* Read MULRn */
//  117     buffer_64bit -> low_high = MULR1;
        MOVW      AX, 0x292          ;; 1 cycle
        MOVW      [DE+0x02], AX      ;; 1 cycle
//  118     buffer_64bit -> high_low = MULR2;
        MOVW      AX, 0x294          ;; 1 cycle
        MOVW      [DE+0x04], AX      ;; 1 cycle
//  119     buffer_64bit -> high_high = MULR3;
        MOVW      AX, 0x296          ;; 1 cycle
        MOVW      [DE+0x06], AX      ;; 1 cycle
//  120 }
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock3
        ; ------------------------------------- Block: 36 cycles
        ; ------------------------------------- Total: 36 cycles
        REQUIRE __A_MUL32UL
        REQUIRE __A_MUL32UH
        REQUIRE __A_MULBL
        REQUIRE __A_MULBH
        REQUIRE __A_MULR0
        REQUIRE __A_MULR1
        REQUIRE __A_MULR2
        REQUIRE __A_MULR3
//  121 /***********************************************************************************************************************
//  122 * Function Name: R_MAC32Bit_MULSigned
//  123 * Description  : This function caculates signed values in multiplication mode.
//  124 * Arguments    : data_a -
//  125 *                    Multiplication data register A value
//  126 *                data_b -
//  127 *                    Multiplication data register B value
//  128 *                buffer_64bit -
//  129 *                    Multiplication result value
//  130 * Return Value : None
//  131 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock4 Using cfiCommon1
          CFI Function _R_MAC32Bit_MULSigned
          CFI NoCalls
        CODE
//  132 void R_MAC32Bit_MULSigned(int32_t data_a, int32_t data_b, mac32bit_int64_t * buffer_64bit)
//  133 {
_R_MAC32Bit_MULSigned:
        ; * Stack frame (at entry) *
        ; Param size: 4
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+6
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+8
        ; Auto size: 4
//  134     /* Set data_a */
//  135     MUL32SL = (int16_t)data_a;
        MOVW      AX, [SP]           ;; 1 cycle
        MOVW      0x284, AX          ;; 1 cycle
//  136     MUL32SH = (int16_t)(data_a >> 16U);
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        MOVW      AX, BC             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      0x286, AX          ;; 1 cycle
//  137 
//  138     /* Set data_b */
//  139     MULBL = (int16_t)data_b;
        MOVW      AX, [SP+0x08]      ;; 1 cycle
        MOVW      0xFFF3C, AX        ;; 1 cycle
//  140     MULBH = (int16_t)(data_b >> 16U);
        MOVW      AX, [SP+0x0A]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x08]      ;; 1 cycle
        MOVW      AX, BC             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      0xFFF3E, AX        ;; 1 cycle
//  141 
//  142     /* Wait 2 more cycles */
//  143     NOP();
        NOP                          ;; 1 cycle
//  144     NOP();
        NOP                          ;; 1 cycle
//  145 
//  146     /* Read MULR0 */
//  147     buffer_64bit -> low_low = MULR0;
        MOVW      AX, 0x290          ;; 1 cycle
        MOVW      [DE], AX           ;; 1 cycle
//  148 
//  149     /* Wait 1 cycle */
//  150     NOP();
        NOP                          ;; 1 cycle
//  151 
//  152     /* Read MULRn */
//  153     buffer_64bit -> low_high = MULR1;
        MOVW      AX, 0x292          ;; 1 cycle
        MOVW      [DE+0x02], AX      ;; 1 cycle
//  154     buffer_64bit -> high_low = MULR2;
        MOVW      AX, 0x294          ;; 1 cycle
        MOVW      [DE+0x04], AX      ;; 1 cycle
//  155     buffer_64bit -> high_high = MULR3;
        MOVW      AX, 0x296          ;; 1 cycle
        MOVW      [DE+0x06], AX      ;; 1 cycle
//  156 }
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock4
        ; ------------------------------------- Block: 38 cycles
        ; ------------------------------------- Total: 38 cycles
        REQUIRE __A_MUL32SL
        REQUIRE __A_MUL32SH
        REQUIRE __A_MULBL
        REQUIRE __A_MULBH
        REQUIRE __A_MULR0
        REQUIRE __A_MULR1
        REQUIRE __A_MULR2
        REQUIRE __A_MULR3
//  157 /***********************************************************************************************************************
//  158 * Function Name: R_MAC32Bit_MACUnsigned
//  159 * Description  : This function caculates unsigned values in multiply-accumulation mode.
//  160 * Arguments    : data_a -
//  161 *                    Multiplication data register A value
//  162 *                data_b -
//  163 *                    Multiplication data register B value
//  164 *                buffer_64bit -
//  165 *                    Accumulation value and multiply-accumulation result
//  166 * Return Value : None
//  167 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock5 Using cfiCommon1
          CFI Function _R_MAC32Bit_MACUnsigned
          CFI NoCalls
        CODE
//  168 void R_MAC32Bit_MACUnsigned(uint32_t data_a, uint32_t data_b, mac32bit_uint64_t * buffer_64bit)
//  169 {
_R_MAC32Bit_MACUnsigned:
        ; * Stack frame (at entry) *
        ; Param size: 4
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+6
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+8
        ; Auto size: 4
//  170     MULC = 0x80;
        MOV       0x29A, #0x80       ;; 1 cycle
//  171     /* Set initial value */
//  172     MULR0 = buffer_64bit -> low_low;
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      0x290, AX          ;; 1 cycle
//  173     MULR1 = buffer_64bit -> low_high;
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      0x292, AX          ;; 1 cycle
//  174     MULR2 = buffer_64bit -> high_low;
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      0x294, AX          ;; 1 cycle
//  175     MULR3 = buffer_64bit -> high_high;
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      0x296, AX          ;; 1 cycle
//  176 
//  177     /* Set data_a */
//  178     MAC32UL = (uint16_t)data_a;
        MOVW      AX, [SP]           ;; 1 cycle
        MOVW      0x288, AX          ;; 1 cycle
//  179     MAC32UH = (uint16_t)(data_a >> 16U);
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        CLRW      AX                 ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      0x28A, AX          ;; 1 cycle
//  180 
//  181     /* Set data_b */
//  182     MULBL = (uint16_t)data_b;
        MOVW      AX, [SP+0x08]      ;; 1 cycle
        MOVW      0xFFF3C, AX        ;; 1 cycle
//  183     MULBH = (uint16_t)(data_b >> 16U);
        MOVW      AX, [SP+0x0A]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x08]      ;; 1 cycle
        CLRW      AX                 ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      0xFFF3E, AX        ;; 1 cycle
//  184 
//  185     /* Wait 2 more cycles */
//  186     NOP();
        NOP                          ;; 1 cycle
//  187     NOP();
        NOP                          ;; 1 cycle
//  188 
//  189     /* Read MULR0 */
//  190     buffer_64bit -> low_low = MULR0;
        MOVW      AX, 0x290          ;; 1 cycle
        MOVW      [DE], AX           ;; 1 cycle
//  191 
//  192     /* Wait 1 cycle */
//  193     NOP();
        NOP                          ;; 1 cycle
//  194 
//  195     /* Read MULRn */
//  196     buffer_64bit -> low_high = MULR1;
        MOVW      AX, 0x292          ;; 1 cycle
        MOVW      [DE+0x02], AX      ;; 1 cycle
//  197     buffer_64bit -> high_low = MULR2;
        MOVW      AX, 0x294          ;; 1 cycle
        MOVW      [DE+0x04], AX      ;; 1 cycle
//  198     buffer_64bit -> high_high = MULR3;
        MOVW      AX, 0x296          ;; 1 cycle
        MOVW      [DE+0x06], AX      ;; 1 cycle
//  199 }
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock5
        ; ------------------------------------- Block: 45 cycles
        ; ------------------------------------- Total: 45 cycles
        REQUIRE __A_MULC
        REQUIRE __A_MULR0
        REQUIRE __A_MULR1
        REQUIRE __A_MULR2
        REQUIRE __A_MULR3
        REQUIRE __A_MAC32UL
        REQUIRE __A_MAC32UH
        REQUIRE __A_MULBL
        REQUIRE __A_MULBH
//  200 /***********************************************************************************************************************
//  201 * Function Name: R_MAC32Bit_MACSigned
//  202 * Description  : This function caculates signed values in multiply-accumulation mode.
//  203 * Arguments    : data_a -
//  204 *                    Multiplication data register A value
//  205 *                data_b -
//  206 *                    Multiplication data register B value
//  207 *                buffer_64bit -
//  208 *                    Accumulation value and multiply-accumulation result
//  209 * Return Value : None
//  210 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock6 Using cfiCommon1
          CFI Function _R_MAC32Bit_MACSigned
          CFI NoCalls
        CODE
//  211 void R_MAC32Bit_MACSigned(int32_t data_a, int32_t data_b, mac32bit_int64_t * buffer_64bit)
//  212 {
_R_MAC32Bit_MACSigned:
        ; * Stack frame (at entry) *
        ; Param size: 4
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+6
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+8
        ; Auto size: 4
//  213     MULC = 0xC0;
        MOV       0x29A, #0xC0       ;; 1 cycle
//  214 
//  215     /* Set initial value */
//  216     MULR0 = buffer_64bit -> low_low;
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      0x290, AX          ;; 1 cycle
//  217     MULR1 = buffer_64bit -> low_high;
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      0x292, AX          ;; 1 cycle
//  218     MULR2 = buffer_64bit -> high_low;
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      0x294, AX          ;; 1 cycle
//  219     MULR3 = buffer_64bit -> high_high;
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      0x296, AX          ;; 1 cycle
//  220 
//  221     /* Set data_a */
//  222     MAC32SL = (int16_t)data_a;
        MOVW      AX, [SP]           ;; 1 cycle
        MOVW      0x28C, AX          ;; 1 cycle
//  223     MAC32SH = (int16_t)(data_a >> 16U);
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        MOVW      AX, BC             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      0x28E, AX          ;; 1 cycle
//  224 
//  225     /* Set data_b */
//  226     MULBL = (int16_t)data_b;
        MOVW      AX, [SP+0x08]      ;; 1 cycle
        MOVW      0xFFF3C, AX        ;; 1 cycle
//  227     MULBH = (int16_t)(data_b >> 16U);
        MOVW      AX, [SP+0x0A]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x08]      ;; 1 cycle
        MOVW      AX, BC             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      0xFFF3E, AX        ;; 1 cycle
//  228 
//  229     /* Wait 2 more cycles */
//  230     NOP();
        NOP                          ;; 1 cycle
//  231     NOP();
        NOP                          ;; 1 cycle
//  232 
//  233     /* Read MULR0 */
//  234     buffer_64bit -> low_low = MULR0;
        MOVW      AX, 0x290          ;; 1 cycle
        MOVW      [DE], AX           ;; 1 cycle
//  235 
//  236     /* Wait 1 cycle */
//  237     NOP();
        NOP                          ;; 1 cycle
//  238 
//  239     /* Read MULRn */
//  240     buffer_64bit -> low_high = MULR1;
        MOVW      AX, 0x292          ;; 1 cycle
        MOVW      [DE+0x02], AX      ;; 1 cycle
//  241     buffer_64bit -> high_low = MULR2;
        MOVW      AX, 0x294          ;; 1 cycle
        MOVW      [DE+0x04], AX      ;; 1 cycle
//  242     buffer_64bit -> high_high = MULR3;
        MOVW      AX, 0x296          ;; 1 cycle
        MOVW      [DE+0x06], AX      ;; 1 cycle
//  243 }
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock6
        ; ------------------------------------- Block: 47 cycles
        ; ------------------------------------- Total: 47 cycles
        REQUIRE __A_MULC
        REQUIRE __A_MULR0
        REQUIRE __A_MULR1
        REQUIRE __A_MULR2
        REQUIRE __A_MULR3
        REQUIRE __A_MAC32SL
        REQUIRE __A_MAC32SH
        REQUIRE __A_MULBL
        REQUIRE __A_MULBH

        SECTION `.iar_vfe_header`:DATA:NOALLOC:NOROOT(1)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
        DC32 0

        END
//  244 
//  245 /* Start user code for adding. Do not edit comment generated here */
//  246 /* End user code. Do not edit comment generated here */
// 
//  12 bytes in section .bss
//  35 bytes in section .bss.noinit  (abs)
// 299 bytes in section .text
// 
// 299 bytes of CODE memory
//  12 bytes of DATA memory (+ 35 bytes shared)
//
//Errors: none
//Warnings: none
