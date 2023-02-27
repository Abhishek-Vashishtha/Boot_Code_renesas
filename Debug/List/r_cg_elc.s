///////////////////////////////////////////////////////////////////////////////
//
// IAR C/C++ Compiler V4.20.1.2260 for RL78               20/Dec/2020  22:38:50
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
//        BootCode\source_code\driver_files\r_cg_elc.c
//    Command line       =
//        -f C:\Users\laptop\AppData\Local\Temp\EWD650.tmp ("D:\Dheeraj\New
//        folder\0. GDEV72 - BootCode\source_code\driver_files\r_cg_elc.c"
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
//        D:\Dheeraj\New folder\0. GDEV72 - BootCode\Debug\List\r_cg_elc.s
//
///////////////////////////////////////////////////////////////////////////////

        RTMODEL "__SystemLibrary", "DLib"
        RTMODEL "__calling_convention", "v2"
        RTMODEL "__code_model", "near"
        RTMODEL "__core", "s3"
        RTMODEL "__data_model", "near"
        RTMODEL "__double_size", "32"
        RTMODEL "__far_rt_calls", "false"
        RTMODEL "__rt_version", "2"

        #define SHT_PROGBITS 0x1
        #define SHT_IAR_NOINIT 0xabdc5467
        #define SHF_WRITE 0x1

        EXTERN ?UL_RSH_L03

        PUBLIC _R_ELC_Clear
        PUBLIC _R_ELC_Create
        PUBLIC _R_ELC_Set
        PUBLIC _R_ELC_Stop
        PUBLIC __A_ELSELR00
        PUBLIC __A_ELSELR01
        PUBLIC __A_ELSELR02
        PUBLIC __A_ELSELR03
        PUBLIC __A_ELSELR04
        PUBLIC __A_ELSELR05
        PUBLIC __A_ELSELR06
        PUBLIC __A_ELSELR07
        PUBLIC __A_ELSELR08
        PUBLIC __A_ELSELR09
        PUBLIC __A_ELSELR10
        PUBLIC __A_ELSELR11
        PUBLIC __A_ELSELR12
        PUBLIC __A_ELSELR13
        PUBLIC __A_ELSELR14
        PUBLIC __A_ELSELR15
        PUBLIC __A_ELSELR16
        PUBLIC __A_ELSELR17
        PUBLIC __A_ELSELR18
        PUBLIC __A_ELSELR19
        PUBLIC __A_ELSELR20
        PUBLIC __A_ELSELR21
        
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
        
// D:\Dheeraj\New folder\0. GDEV72 - BootCode\source_code\driver_files\r_cg_elc.c
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
//   21 * File Name    : r_cg_elc.c
//   22 * Version      : 
//   23 * Device(s)    : RL78/I1C
//   24 * Tool-Chain   : CCRL
//   25 * Description  : This file implements device driver for ELC module.
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

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0240H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_ELSELR00
// __no_init union <unnamed>#473 volatile __no_bit_access _A_ELSELR00
__A_ELSELR00:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0241H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_ELSELR01
// __no_init union <unnamed>#474 volatile __no_bit_access _A_ELSELR01
__A_ELSELR01:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0242H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_ELSELR02
// __no_init union <unnamed>#475 volatile __no_bit_access _A_ELSELR02
__A_ELSELR02:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0243H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_ELSELR03
// __no_init union <unnamed>#476 volatile __no_bit_access _A_ELSELR03
__A_ELSELR03:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0244H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_ELSELR04
// __no_init union <unnamed>#477 volatile __no_bit_access _A_ELSELR04
__A_ELSELR04:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0245H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_ELSELR05
// __no_init union <unnamed>#478 volatile __no_bit_access _A_ELSELR05
__A_ELSELR05:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0246H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_ELSELR06
// __no_init union <unnamed>#479 volatile __no_bit_access _A_ELSELR06
__A_ELSELR06:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0247H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_ELSELR07
// __no_init union <unnamed>#480 volatile __no_bit_access _A_ELSELR07
__A_ELSELR07:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0248H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_ELSELR08
// __no_init union <unnamed>#481 volatile __no_bit_access _A_ELSELR08
__A_ELSELR08:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0249H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_ELSELR09
// __no_init union <unnamed>#482 volatile __no_bit_access _A_ELSELR09
__A_ELSELR09:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f024aH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_ELSELR10
// __no_init union <unnamed>#483 volatile __no_bit_access _A_ELSELR10
__A_ELSELR10:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f024bH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_ELSELR11
// __no_init union <unnamed>#484 volatile __no_bit_access _A_ELSELR11
__A_ELSELR11:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f024cH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_ELSELR12
// __no_init union <unnamed>#485 volatile __no_bit_access _A_ELSELR12
__A_ELSELR12:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f024dH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_ELSELR13
// __no_init union <unnamed>#486 volatile __no_bit_access _A_ELSELR13
__A_ELSELR13:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f024eH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_ELSELR14
// __no_init union <unnamed>#487 volatile __no_bit_access _A_ELSELR14
__A_ELSELR14:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f024fH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_ELSELR15
// __no_init union <unnamed>#488 volatile __no_bit_access _A_ELSELR15
__A_ELSELR15:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0250H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_ELSELR16
// __no_init union <unnamed>#489 volatile __no_bit_access _A_ELSELR16
__A_ELSELR16:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0251H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_ELSELR17
// __no_init union <unnamed>#490 volatile __no_bit_access _A_ELSELR17
__A_ELSELR17:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0252H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_ELSELR18
// __no_init union <unnamed>#491 volatile __no_bit_access _A_ELSELR18
__A_ELSELR18:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0253H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_ELSELR19
// __no_init union <unnamed>#492 volatile __no_bit_access _A_ELSELR19
__A_ELSELR19:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0254H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_ELSELR20
// __no_init union <unnamed>#493 volatile __no_bit_access _A_ELSELR20
__A_ELSELR20:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0255H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_ELSELR21
// __no_init union <unnamed>#494 volatile __no_bit_access _A_ELSELR21
__A_ELSELR21:
        DS 1
//   39 #include "r_cg_elc.h"
//   40 /* Start user code for include. Do not edit comment generated here */
//   41 /* End user code. Do not edit comment generated here */
//   42 #include "r_cg_userdefine.h"
//   43 
//   44 /***********************************************************************************************************************
//   45 Global variables and functions
//   46 ***********************************************************************************************************************/
//   47 /* Start user code for global. Do not edit comment generated here */
//   48 /* End user code. Do not edit comment generated here */
//   49 
//   50 /***********************************************************************************************************************
//   51 * Function Name: R_ELC_Create
//   52 * Description  : This function initializes the ELC module.
//   53 * Arguments    : None
//   54 * Return Value : None
//   55 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock0 Using cfiCommon0
          CFI Function _R_ELC_Create
          CFI NoCalls
        CODE
//   56 void R_ELC_Create(void)
//   57 {
_R_ELC_Create:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//   58     ELSELR00 = _00_ELC_EVENT_LINK_OFF;
        MOV       0x240, #0x0        ;; 1 cycle
//   59     ELSELR01 = _00_ELC_EVENT_LINK_OFF;
        MOV       0x241, #0x0        ;; 1 cycle
//   60     ELSELR02 = _00_ELC_EVENT_LINK_OFF;
        MOV       0x242, #0x0        ;; 1 cycle
//   61     ELSELR03 = _00_ELC_EVENT_LINK_OFF;
        MOV       0x243, #0x0        ;; 1 cycle
//   62     ELSELR04 = _00_ELC_EVENT_LINK_OFF;
        MOV       0x244, #0x0        ;; 1 cycle
//   63     ELSELR05 = _00_ELC_EVENT_LINK_OFF;
        MOV       0x245, #0x0        ;; 1 cycle
//   64     ELSELR06 = _00_ELC_EVENT_LINK_OFF;
        MOV       0x246, #0x0        ;; 1 cycle
//   65     ELSELR07 = _00_ELC_EVENT_LINK_OFF;
        MOV       0x247, #0x0        ;; 1 cycle
//   66     ELSELR08 = _00_ELC_EVENT_LINK_OFF;
        MOV       0x248, #0x0        ;; 1 cycle
//   67     ELSELR09 = _00_ELC_EVENT_LINK_OFF;
        MOV       0x249, #0x0        ;; 1 cycle
//   68     ELSELR10 = _00_ELC_EVENT_LINK_OFF;
        MOV       0x24A, #0x0        ;; 1 cycle
//   69     ELSELR11 = _00_ELC_EVENT_LINK_OFF;
        MOV       0x24B, #0x0        ;; 1 cycle
//   70     ELSELR12 = _00_ELC_EVENT_LINK_OFF;
        MOV       0x24C, #0x0        ;; 1 cycle
//   71     ELSELR13 = _00_ELC_EVENT_LINK_OFF;
        MOV       0x24D, #0x0        ;; 1 cycle
//   72     ELSELR14 = _00_ELC_EVENT_LINK_OFF;
        MOV       0x24E, #0x0        ;; 1 cycle
//   73     ELSELR15 = _00_ELC_EVENT_LINK_OFF;
        MOV       0x24F, #0x0        ;; 1 cycle
//   74     ELSELR16 = _00_ELC_EVENT_LINK_OFF;
        MOV       0x250, #0x0        ;; 1 cycle
//   75     ELSELR17 = _00_ELC_EVENT_LINK_OFF;
        MOV       0x251, #0x0        ;; 1 cycle
//   76     ELSELR18 = _00_ELC_EVENT_LINK_OFF;
        MOV       0x252, #0x0        ;; 1 cycle
//   77     ELSELR19 = _00_ELC_EVENT_LINK_OFF;
        MOV       0x253, #0x0        ;; 1 cycle
//   78     ELSELR20 = _00_ELC_EVENT_LINK_OFF;
        MOV       0x254, #0x0        ;; 1 cycle
//   79     ELSELR21 = _00_ELC_EVENT_LINK_OFF;
        MOV       0x255, #0x0        ;; 1 cycle
//   80 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock0
        ; ------------------------------------- Block: 28 cycles
        ; ------------------------------------- Total: 28 cycles
        REQUIRE __A_ELSELR00
        REQUIRE __A_ELSELR01
        REQUIRE __A_ELSELR02
        REQUIRE __A_ELSELR03
        REQUIRE __A_ELSELR04
        REQUIRE __A_ELSELR05
        REQUIRE __A_ELSELR06
        REQUIRE __A_ELSELR07
        REQUIRE __A_ELSELR08
        REQUIRE __A_ELSELR09
        REQUIRE __A_ELSELR10
        REQUIRE __A_ELSELR11
        REQUIRE __A_ELSELR12
        REQUIRE __A_ELSELR13
        REQUIRE __A_ELSELR14
        REQUIRE __A_ELSELR15
        REQUIRE __A_ELSELR16
        REQUIRE __A_ELSELR17
        REQUIRE __A_ELSELR18
        REQUIRE __A_ELSELR19
        REQUIRE __A_ELSELR20
        REQUIRE __A_ELSELR21
//   81 
//   82 /***********************************************************************************************************************
//   83 * Function Name: R_ELC_Stop
//   84 * Description  : This function stops the ELC event resources.
//   85 * Arguments    : src -
//   86 *                    event resources to be stopped (ELSELRn)
//   87 * Return Value : None
//   88 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock1 Using cfiCommon1
          CFI Function _R_ELC_Stop
        CODE
//   89 void R_ELC_Stop(uint32_t src)
//   90 {
_R_ELC_Stop:
        ; * Stack frame (at entry) *
        ; Param size: 0
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+6
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+8
        ; Auto size: 10
        SUBW      SP, #0x6           ;; 1 cycle
          CFI CFA SP+14
//   91     volatile uint32_t   w_count;
//   92     uint8_t           * sfr_addr;
//   93 
//   94     sfr_addr = (uint8_t *)&ELSELR00;
        MOVW      AX, #0x240         ;; 1 cycle
        MOVW      [SP], AX           ;; 1 cycle
//   95     
//   96     for (w_count = 0U; w_count < ELC_TRIGGER_SRC_COUNT; w_count++) 
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x2           ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        ; ------------------------------------- Block: 12 cycles
??R_ELC_Stop_0:
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x16          ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??R_ELC_Stop_1:
        BNC       ??R_ELC_Clear_0    ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
//   97     {
//   98         if (((src >> w_count) & 0x1U) == 0x1U)
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      AX, [SP+0x08]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x06]      ;; 1 cycle
          CFI FunCall ?UL_RSH_L03
        CALL      N:?UL_RSH_L03      ;; 3 cycles
        MOV       A, X               ;; 1 cycle
        AND       A, #0x1            ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        BZ        ??R_ELC_Clear_1    ;; 4 cycles
        ; ------------------------------------- Block: 19 cycles
//   99         {
//  100             *sfr_addr = _00_ELC_EVENT_LINK_OFF;
        MOVW      AX, [SP]           ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
        ; ------------------------------------- Block: 4 cycles
//  101         }
//  102         
//  103         sfr_addr++;
??R_ELC_Clear_1:
        MOVW      AX, [SP]           ;; 1 cycle
        INCW      AX                 ;; 1 cycle
        MOVW      [SP], AX           ;; 1 cycle
//  104     }
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        ADDW      AX, #0x1           ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x2           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        BR        S:??R_ELC_Stop_0   ;; 3 cycles
        ; ------------------------------------- Block: 26 cycles
//  105 }
??R_ELC_Clear_0:
        ADDW      SP, #0xA           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock1
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 80 cycles
        REQUIRE __A_ELSELR00
//  106 
//  107 /* Start user code for adding. Do not edit comment generated here */
//  108 
//  109 /***********************************************************************************************************************
//  110 * Function Name: R_ELC_Set
//  111 * Description  : This function set the ELC event link.
//  112 * Arguments    : src -
//  113 *                    event resources to be stoped (ELSELRn)
//  114 *              : dest - 
//  115 *                    event destination
//  116 * Return Value : None
//  117 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock2 Using cfiCommon0
          CFI Function _R_ELC_Set
          CFI NoCalls
        CODE
//  118 void R_ELC_Set(elc_src_t src, uint8_t dest)
//  119 {
_R_ELC_Set:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
        MOV       C, A               ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, X               ;; 1 cycle
//  120     uint8_t * sfr_addr = (uint8_t *)(&ELSELR00 + src);
        MOVW      AX, BC             ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        ADDW      AX, #0x240         ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
//  121     *sfr_addr = dest;
        MOV       A, B               ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
//  122 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock2
        ; ------------------------------------- Block: 16 cycles
        ; ------------------------------------- Total: 16 cycles
        REQUIRE __A_ELSELR00
//  123 
//  124 /***********************************************************************************************************************
//  125 * Function Name: R_ELC_Clear
//  126 * Description  : This function clear the ELC event link.
//  127 * Arguments    : src -
//  128 *                    event resources to be stoped (ELSELRn)
//  129 * Return Value : None
//  130 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock3 Using cfiCommon0
          CFI Function _R_ELC_Clear
          CFI NoCalls
        CODE
//  131 void R_ELC_Clear(elc_src_t src)
//  132 {
_R_ELC_Clear:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
        MOV       C, A               ;; 1 cycle
//  133     uint8_t  *sfr_addr = (uint8_t *)(&ELSELR00 + src);
        MOVW      AX, BC             ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        ADDW      AX, #0x240         ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
//  134     *sfr_addr = _00_ELC_EVENT_LINK_OFF;
        MOV       A, #0x0            ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
//  135 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock3
        ; ------------------------------------- Block: 13 cycles
        ; ------------------------------------- Total: 13 cycles
        REQUIRE __A_ELSELR00

        SECTION `.iar_vfe_header`:DATA:NOALLOC:NOROOT(1)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
        DC32 0

        END
//  136 
//  137 /* End user code. Do not edit comment generated here */
// 
//  22 bytes in section .bss.noinit  (abs)
// 222 bytes in section .text
// 
// 222 bytes of CODE memory
//   0 bytes of DATA memory (+ 22 bytes shared)
//
//Errors: none
//Warnings: none
