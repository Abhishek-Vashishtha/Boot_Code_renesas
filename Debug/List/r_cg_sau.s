///////////////////////////////////////////////////////////////////////////////
//
// IAR C/C++ Compiler V4.21.1.2409/W32 for RL78           27/Feb/2023  16:59:21
// Copyright 2011-2021 IAR Systems AB.
// Network license: 192.10.10.205 (STD)
//
//    Core               =  s3
//    Calling convention =  v2
//    Code model         =  Near
//    Data model         =  Near
//                       =   
//    Source file        =
//        D:\Software code\Projects Software\Projects new\0. Non AMI\SUGAM
//        Based\0. GDEV72 - BootCode-20210104T103109Z-001\0. GDEV72 -
//        BootCode\source_code\driver_files\r_cg_sau.c
//    Command line       =
//        -f C:\Users\17054\AppData\Local\Temp\EWDCE0.tmp ("D:\Software
//        code\Projects Software\Projects new\0. Non AMI\SUGAM Based\0. GDEV72
//        - BootCode-20210104T103109Z-001\0. GDEV72 -
//        BootCode\source_code\driver_files\r_cg_sau.c" --core s3 --code_model
//        near --calling_convention v2 --near_const_location ram -o
//        "D:\Software code\Projects Software\Projects new\0. Non AMI\SUGAM
//        Based\0. GDEV72 - BootCode-20210104T103109Z-001\0. GDEV72 -
//        BootCode\Debug\Obj" --dlib_config "C:\Program Files (x86)\IAR
//        Systems\Embedded Workbench 8.5\rl78\lib\DLib_Config_Normal.h"
//        --double=32 -e -On --no_cse --no_unroll --no_inline --no_code_motion
//        --no_tbaa --no_cross_call --no_scheduling --no_clustering --debug -lA
//        "D:\Software code\Projects Software\Projects new\0. Non AMI\SUGAM
//        Based\0. GDEV72 - BootCode-20210104T103109Z-001\0. GDEV72 -
//        BootCode\Debug\List" -I "D:\Software code\Projects Software\Projects
//        new\0. Non AMI\SUGAM Based\0. GDEV72 -
//        BootCode-20210104T103109Z-001\0. GDEV72 -
//        BootCode\source_code\driver_files\\" -I "D:\Software code\Projects
//        Software\Projects new\0. Non AMI\SUGAM Based\0. GDEV72 -
//        BootCode-20210104T103109Z-001\0. GDEV72 -
//        BootCode\source_code\library_files\\" -I "D:\Software code\Projects
//        Software\Projects new\0. Non AMI\SUGAM Based\0. GDEV72 -
//        BootCode-20210104T103109Z-001\0. GDEV72 -
//        BootCode\source_code\misc_files\\" -I "D:\Software code\Projects
//        Software\Projects new\0. Non AMI\SUGAM Based\0. GDEV72 -
//        BootCode-20210104T103109Z-001\0. GDEV72 -
//        BootCode\source_code\source_files\\" --data_model near)
//    Locale             =  C
//    List file          =
//        D:\Software code\Projects Software\Projects new\0. Non AMI\SUGAM
//        Based\0. GDEV72 - BootCode-20210104T103109Z-001\0. GDEV72 -
//        BootCode\Debug\List\r_cg_sau.s
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

        EXTERN _clock_select

        PUBLIC _R_SAU0_Create
        PUBLIC _R_SAU1_Create
        PUBLIC _R_UART1_Create
        PUBLIC _R_UART1_Receive
        PUBLIC _R_UART1_Send
        PUBLIC _R_UART1_Start
        PUBLIC _R_UART1_Stop
        PUBLIC _R_UART2_Create
        PUBLIC _R_UART2_Receive
        PUBLIC _R_UART2_Send
        PUBLIC _R_UART2_Start
        PUBLIC _R_UART2_Stop
        PUBLIC __A_IF0
        PUBLIC __A_IF1
        PUBLIC __A_MK0
        PUBLIC __A_MK1
        PUBLIC __A_NFEN0
        PUBLIC __A_PER0
        PUBLIC __A_PR00
        PUBLIC __A_PR01
        PUBLIC __A_PR10
        PUBLIC __A_PR11
        PUBLIC __A_SCR02
        PUBLIC __A_SCR03
        PUBLIC __A_SCR10
        PUBLIC __A_SCR11
        PUBLIC __A_SDR02
        PUBLIC __A_SDR03
        PUBLIC __A_SDR10
        PUBLIC __A_SDR11
        PUBLIC __A_SIR03
        PUBLIC __A_SIR11
        PUBLIC __A_SMR02
        PUBLIC __A_SMR03
        PUBLIC __A_SMR10
        PUBLIC __A_SMR11
        PUBLIC __A_SO0
        PUBLIC __A_SO1
        PUBLIC __A_SOE0
        PUBLIC __A_SOE1
        PUBLIC __A_SOL0
        PUBLIC __A_SOL1
        PUBLIC __A_SPS0
        PUBLIC __A_SPS1
        PUBLIC __A_SS0
        PUBLIC __A_SS1
        PUBLIC __A_ST0
        PUBLIC __A_ST1
        PUBLIC _g_uart1_rx_count
        PUBLIC _g_uart1_rx_length
        PUBLIC _g_uart1_tx_count
        PUBLIC _g_uart2_rx_count
        PUBLIC _g_uart2_rx_length
        PUBLIC _g_uart2_tx_count
        PUBLIC _gp_uart1_rx_address
        PUBLIC _gp_uart1_tx_address
        PUBLIC _gp_uart2_rx_address
        PUBLIC _gp_uart2_tx_address
        
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
        
// D:\Software code\Projects Software\Projects new\0. Non AMI\SUGAM Based\0. GDEV72 - BootCode-20210104T103109Z-001\0. GDEV72 - BootCode\source_code\driver_files\r_cg_sau.c
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
//   17 * Copyright (C) 2015, 2019 Renesas Electronics Corporation. All rights reserved.
//   18 ***********************************************************************************************************************/
//   19 
//   20 /***********************************************************************************************************************
//   21 * File Name    : r_cg_sau.c
//   22 * Version      : Applilet4 for RL78/I1C V1.01.04.02 [20 Nov 2019]
//   23 * Device(s)    : R5F10NPJ
//   24 * Tool-Chain   : IAR Systems icc78k0r
//   25 * Description  : This file implements device driver for SAU module.
//   26 * Creation Date: 06/05/2020
//   27 ***********************************************************************************************************************/
//   28 
//   29 /***********************************************************************************************************************
//   30 Includes
//   31 ***********************************************************************************************************************/
//   32 #include "r_cg_macrodriver.h"

        ASEGN `.bss.noinit`:DATA:NOROOT,0fff44H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_SDR02
// __no_init union <unnamed>#65 volatile __sfr __no_bit_access _A_SDR02
__A_SDR02:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0fff46H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_SDR03
// __no_init union <unnamed>#68 volatile __sfr __no_bit_access _A_SDR03
__A_SDR03:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0fff48H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_SDR10
// __no_init union <unnamed>#71 volatile __sfr __no_bit_access _A_SDR10
__A_SDR10:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0fff4aH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_SDR11
// __no_init union <unnamed>#74 volatile __sfr __no_bit_access _A_SDR11
__A_SDR11:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0fffe0H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_IF0
// __no_init union <unnamed>#152 volatile __sfr _A_IF0
__A_IF0:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0fffe2H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_IF1
// __no_init union <unnamed>#160 volatile __sfr _A_IF1
__A_IF1:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0fffe4H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_MK0
// __no_init union <unnamed>#170 volatile __sfr _A_MK0
__A_MK0:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0fffe6H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_MK1
// __no_init union <unnamed>#178 volatile __sfr _A_MK1
__A_MK1:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0fffe8H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_PR00
// __no_init union <unnamed>#188 volatile __sfr _A_PR00
__A_PR00:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0fffeaH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_PR01
// __no_init union <unnamed>#196 volatile __sfr _A_PR01
__A_PR01:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0fffecH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_PR10
// __no_init union <unnamed>#206 volatile __sfr _A_PR10
__A_PR10:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0fffeeH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_PR11
// __no_init union <unnamed>#214 volatile __sfr _A_PR11
__A_PR11:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0070H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_NFEN0
// __no_init union <unnamed>#250 volatile _A_NFEN0
__A_NFEN0:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f00f0H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_PER0
// __no_init union <unnamed>#275 volatile _A_PER0
__A_PER0:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f010eH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_SIR03
// __no_init union <unnamed>#317 volatile __no_bit_access _A_SIR03
__A_SIR03:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0114H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_SMR02
// __no_init union <unnamed>#322 volatile __no_bit_access _A_SMR02
__A_SMR02:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0116H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_SMR03
// __no_init union <unnamed>#323 volatile __no_bit_access _A_SMR03
__A_SMR03:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f011cH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_SCR02
// __no_init union <unnamed>#326 volatile __no_bit_access _A_SCR02
__A_SCR02:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f011eH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_SCR03
// __no_init union <unnamed>#327 volatile __no_bit_access _A_SCR03
__A_SCR03:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0122H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_SS0
// __no_init union <unnamed>#331 volatile _A_SS0
__A_SS0:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0124H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_ST0
// __no_init union <unnamed>#334 volatile _A_ST0
__A_ST0:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0126H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_SPS0
// __no_init union <unnamed>#337 volatile __no_bit_access _A_SPS0
__A_SPS0:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0128H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_SO0
// __no_init union <unnamed>#340 volatile __no_bit_access _A_SO0
__A_SO0:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f012aH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_SOE0
// __no_init union <unnamed>#341 volatile _A_SOE0
__A_SOE0:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0134H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_SOL0
// __no_init union <unnamed>#344 volatile __no_bit_access _A_SOL0
__A_SOL0:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f014aH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_SIR11
// __no_init union <unnamed>#365 volatile __no_bit_access _A_SIR11
__A_SIR11:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0150H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_SMR10
// __no_init union <unnamed>#374 volatile __no_bit_access _A_SMR10
__A_SMR10:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0152H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_SMR11
// __no_init union <unnamed>#375 volatile __no_bit_access _A_SMR11
__A_SMR11:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0158H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_SCR10
// __no_init union <unnamed>#378 volatile __no_bit_access _A_SCR10
__A_SCR10:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f015aH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_SCR11
// __no_init union <unnamed>#379 volatile __no_bit_access _A_SCR11
__A_SCR11:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0162H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_SS1
// __no_init union <unnamed>#385 volatile _A_SS1
__A_SS1:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0164H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_ST1
// __no_init union <unnamed>#388 volatile _A_ST1
__A_ST1:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0166H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_SPS1
// __no_init union <unnamed>#391 volatile __no_bit_access _A_SPS1
__A_SPS1:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0168H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_SO1
// __no_init union <unnamed>#394 volatile __no_bit_access _A_SO1
__A_SO1:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f016aH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_SOE1
// __no_init union <unnamed>#395 volatile _A_SOE1
__A_SOE1:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0174H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_SOL1
// __no_init union <unnamed>#398 volatile __no_bit_access _A_SOL1
__A_SOL1:
        DS 2
//   33 #include "r_cg_sau.h"
//   34 /* Start user code for include. Do not edit comment generated here */
//   35 /* End user code. Do not edit comment generated here */
//   36 
//   37 /***********************************************************************************************************************
//   38 Pragma directive
//   39 ***********************************************************************************************************************/
//   40 /* Start user code for pragma. Do not edit comment generated here */
//   41 /* End user code. Do not edit comment generated here */
//   42 
//   43 /***********************************************************************************************************************
//   44 Global variables and functions
//   45 ***********************************************************************************************************************/

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   46 uint8_t * gp_uart1_tx_address;         /* uart1 transmit buffer address */
_gp_uart1_tx_address:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   47 uint16_t  g_uart1_tx_count;            /* uart1 transmit data number */
_g_uart1_tx_count:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   48 uint8_t * gp_uart1_rx_address;         /* uart1 receive buffer address */
_gp_uart1_rx_address:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   49 uint16_t  g_uart1_rx_count;            /* uart1 receive data number */
_g_uart1_rx_count:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   50 uint16_t  g_uart1_rx_length;           /* uart1 receive data length */
_g_uart1_rx_length:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   51 uint8_t * gp_uart2_tx_address;         /* uart2 transmit buffer address */
_gp_uart2_tx_address:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   52 uint16_t  g_uart2_tx_count;            /* uart2 transmit data number */
_g_uart2_tx_count:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   53 uint8_t * gp_uart2_rx_address;         /* uart2 receive buffer address */
_gp_uart2_rx_address:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   54 uint16_t  g_uart2_rx_count;            /* uart2 receive data number */
_g_uart2_rx_count:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   55 uint16_t  g_uart2_rx_length;           /* uart2 receive data length */
_g_uart2_rx_length:
        DS 2
//   56 /* Start user code for global. Do not edit comment generated here */
//   57 /* End user code. Do not edit comment generated here */
//   58 
//   59 /***********************************************************************************************************************
//   60 * Function Name: R_SAU0_Create
//   61 * Description  : This function initializes the SAU0 module.
//   62 * Arguments    : None
//   63 * Return Value : None
//   64 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock0 Using cfiCommon0
          CFI Function _R_SAU0_Create
        CODE
//   65 void R_SAU0_Create(void)
//   66 {
_R_SAU0_Create:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//   67   SAU0EN = 1U;    /* enables input clock supply */
        SET1      0xF00F0.2          ;; 2 cycles
//   68   NOP();
        NOP                          ;; 1 cycle
//   69   NOP();
        NOP                          ;; 1 cycle
//   70   NOP();
        NOP                          ;; 1 cycle
//   71   NOP();
        NOP                          ;; 1 cycle
//   72   if(clock_select == CLOCK_24MHZ)
        CMP       N:_clock_select, #0x1  ;; 1 cycle
        BNZ       ??R_UART2_Send_0   ;; 4 cycles
        ; ------------------------------------- Block: 11 cycles
//   73   {
//   74     SPS0 = _0040_SAU_CK01_fCLK_4 | _0004_SAU_CK00_fCLK_4;
        MOVW      AX, #0x44          ;; 1 cycle
        MOVW      0x126, AX          ;; 1 cycle
        BR        S:??R_UART2_Send_1  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//   75   }
//   76   else if(clock_select == CLOCK_12MHZ)
??R_UART2_Send_0:
        CMP       N:_clock_select, #0x2  ;; 1 cycle
        BNZ       ??R_UART2_Send_2   ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//   77   {
//   78     SPS0 = _0030_SAU_CK01_fCLK_3 | _0003_SAU_CK00_fCLK_3;
        MOVW      AX, #0x33          ;; 1 cycle
        MOVW      0x126, AX          ;; 1 cycle
        BR        S:??R_UART2_Send_1  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//   79   }
//   80   else if(clock_select == CLOCK_6MHZ)
??R_UART2_Send_2:
        CMP       N:_clock_select, #0x3  ;; 1 cycle
        BNZ       ??R_UART2_Send_3   ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//   81   {
//   82     SPS0 = _0020_SAU_CK01_fCLK_2 | _0002_SAU_CK00_fCLK_2;
        MOVW      AX, #0x22          ;; 1 cycle
        MOVW      0x126, AX          ;; 1 cycle
        BR        S:??R_UART2_Send_1  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//   83   }
//   84   else if(clock_select == CLOCK_1_5MHZ)
??R_UART2_Send_3:
        CMP       N:_clock_select, #0x4  ;; 1 cycle
        BNZ       ??R_UART2_Send_1   ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//   85   {
//   86     SPS0 = _0000_SAU_CK01_fCLK_0 | _0000_SAU_CK00_fCLK_0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      0x126, AX          ;; 1 cycle
          CFI FunCall _R_UART1_Create
        ; ------------------------------------- Block: 2 cycles
//   87   }
//   88   
//   89   
//   90   R_UART1_Create();
??R_UART2_Send_1:
        CALL      _R_UART1_Create    ;; 3 cycles
//   91 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock0
        ; ------------------------------------- Block: 9 cycles
        ; ------------------------------------- Total: 52 cycles
        REQUIRE __A_PER0
        REQUIRE __A_SPS0
//   92 /***********************************************************************************************************************
//   93 * Function Name: R_UART1_Create
//   94 * Description  : This function initializes the UART1 module RJ45
//   95 * Arguments    : None
//   96 * Return Value : None
//   97 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock1 Using cfiCommon0
          CFI Function _R_UART1_Create
          CFI NoCalls
        CODE
//   98 void R_UART1_Create(void)
//   99 {
_R_UART1_Create:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  100   ST0 |= _0008_SAUm_CH3_STOP_TRG_ON | _0004_SAUm_CH2_STOP_TRG_ON;
        MOVW      AX, 0x124          ;; 1 cycle
        OR        A, #0x0            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        OR        A, #0xC            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      0x124, AX          ;; 1 cycle
//  101   STMK1 = 1U;     /* disable INTST1 interrupt */
        SET1      0xFFFE6.1          ;; 2 cycles
//  102   STIF1 = 0U;     /* clear INTST1 interrupt flag */
        CLR1      0xFFFE2.1          ;; 2 cycles
//  103   SRMK1 = 1U;     /* disable INTSR1 interrupt */
        SET1      0xFFFE6.2          ;; 2 cycles
//  104   SRIF1 = 0U;     /* clear INTSR1 interrupt flag */
        CLR1      0xFFFE2.2          ;; 2 cycles
//  105   SREMK1 = 1U;    /* disable INTSRE1 interrupt */
        SET1      0xFFFE6.3          ;; 2 cycles
//  106   SREIF1 = 0U;    /* clear INTSRE1 interrupt flag */
        CLR1      0xFFFE2.3          ;; 2 cycles
//  107   /* Set INTSR1 low priority */
//  108   SRPR11 = 1U;
        SET1      0xFFFEE.2          ;; 2 cycles
//  109   SRPR01 = 1U;
        SET1      0xFFFEA.2          ;; 2 cycles
//  110   /* Set INTSRE1 low priority */
//  111   SREPR11 = 1U;
        SET1      0xFFFEE.3          ;; 2 cycles
//  112   SREPR01 = 1U;
        SET1      0xFFFEA.3          ;; 2 cycles
//  113   /* Set INTST1 low priority */
//  114   STPR11 = 1U;
        SET1      0xFFFEE.1          ;; 2 cycles
//  115   STPR01 = 1U;
        SET1      0xFFFEA.1          ;; 2 cycles
//  116   SMR02 = _0020_SMR02_DEFAULT_VALUE | _0000_SAU_CLOCK_SELECT_CK00 | _0000_SAU_CLOCK_MODE_CKS | 
//  117     _0002_SAU_MODE_UART | _0000_SAU_TRANSFER_END;
        MOVW      AX, #0x22          ;; 1 cycle
        MOVW      0x114, AX          ;; 1 cycle
//  118   SCR02 = _0004_SCR02_DEFAULT_VALUE | _8000_SAU_TRANSMISSION | _0000_SAU_TIMING_1 | _0000_SAU_INTSRE_MASK | 
//  119     _0000_SAU_PARITY_NONE | _0080_SAU_LSB | _0010_SAU_STOP_1 | _0003_SAU_LENGTH_8;
        MOVW      AX, #0x8097        ;; 1 cycle
        MOVW      0x11C, AX          ;; 1 cycle
//  120   SDR02 = _9A00_SAU0_CH2_BAUDRATE_DIVISOR;
        MOVW      0xFFF44, #0x9A00   ;; 1 cycle
//  121   NFEN0 |= _04_SAU_RXD1_FILTER_ON;
        SET1      0xF0070.2          ;; 2 cycles
//  122   SIR03 = _0004_SAU_SIRMN_FECTMN | _0002_SAU_SIRMN_PECTMN | _0001_SAU_SIRMN_OVCTMN;
        MOVW      AX, #0x7           ;; 1 cycle
        MOVW      0x10E, AX          ;; 1 cycle
//  123   SMR03 = _0020_SMR03_DEFAULT_VALUE | _0000_SAU_CLOCK_SELECT_CK00 | _0000_SAU_CLOCK_MODE_CKS | 
//  124     _0100_SAU_TRIGGER_RXD | _0000_SAU_EDGE_FALL | _0002_SAU_MODE_UART | _0000_SAU_TRANSFER_END;
        MOVW      AX, #0x122         ;; 1 cycle
        MOVW      0x116, AX          ;; 1 cycle
//  125   SCR03 = _0004_SCR03_DEFAULT_VALUE | _4000_SAU_RECEPTION | _0000_SAU_TIMING_1 | _0400_SAU_INTSRE_ENABLE | 
//  126     _0000_SAU_PARITY_NONE | _0080_SAU_LSB | _0010_SAU_STOP_1 | _0003_SAU_LENGTH_8;
        MOVW      AX, #0x4497        ;; 1 cycle
        MOVW      0x11E, AX          ;; 1 cycle
//  127   SDR03 = _9A00_SAU0_CH3_BAUDRATE_DIVISOR;
        MOVW      0xFFF46, #0x9A00   ;; 1 cycle
//  128   SO0 |= _0004_SAUm_CH2_DATA_OUTPUT_1;
        MOVW      AX, 0x128          ;; 1 cycle
        OR        A, #0x0            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        OR        A, #0x4            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      0x128, AX          ;; 1 cycle
//  129   SOL0 &= (uint16_t)~_0004_SAUm_CHANNEL2_INVERTED;
        MOVW      AX, 0x134          ;; 1 cycle
        AND       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        AND       A, #0xFB           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      0x134, AX          ;; 1 cycle
//  130   SOE0 |= _0004_SAUm_CH2_OUTPUT_ENABLE;
        SET1      0xF012A.2          ;; 2 cycles
//  131 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock1
        ; ------------------------------------- Block: 64 cycles
        ; ------------------------------------- Total: 64 cycles
        REQUIRE __A_ST0
        REQUIRE __A_MK1
        REQUIRE __A_IF1
        REQUIRE __A_PR11
        REQUIRE __A_PR01
        REQUIRE __A_SMR02
        REQUIRE __A_SCR02
        REQUIRE __A_SDR02
        REQUIRE __A_NFEN0
        REQUIRE __A_SIR03
        REQUIRE __A_SMR03
        REQUIRE __A_SCR03
        REQUIRE __A_SDR03
        REQUIRE __A_SO0
        REQUIRE __A_SOL0
        REQUIRE __A_SOE0
//  132 /***********************************************************************************************************************
//  133 * Function Name: R_UART1_Start
//  134 * Description  : This function starts the UART1 module operation.
//  135 * Arguments    : None
//  136 * Return Value : None
//  137 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock2 Using cfiCommon0
          CFI Function _R_UART1_Start
          CFI NoCalls
        CODE
//  138 void R_UART1_Start(void)
//  139 {
_R_UART1_Start:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  140   SO0 |= _0004_SAUm_CH2_DATA_OUTPUT_1;
        MOVW      AX, 0x128          ;; 1 cycle
        OR        A, #0x0            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        OR        A, #0x4            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      0x128, AX          ;; 1 cycle
//  141   SOE0 |= _0004_SAUm_CH2_OUTPUT_ENABLE;
        SET1      0xF012A.2          ;; 2 cycles
//  142   SS0 |= _0008_SAUm_CH3_START_TRG_ON | _0004_SAUm_CH2_START_TRG_ON;
        MOVW      AX, 0x122          ;; 1 cycle
        OR        A, #0x0            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        OR        A, #0xC            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      0x122, AX          ;; 1 cycle
//  143   STIF1 = 0U;     /* clear INTST1 interrupt flag */
        CLR1      0xFFFE2.1          ;; 2 cycles
//  144   SRIF1 = 0U;     /* clear INTSR1 interrupt flag */
        CLR1      0xFFFE2.2          ;; 2 cycles
//  145   SREIF1 = 0U;    /* clear INTSRE1 interrupt flag */
        CLR1      0xFFFE2.3          ;; 2 cycles
//  146   STMK1 = 0U;     /* enable INTST1 interrupt */
        CLR1      0xFFFE6.1          ;; 2 cycles
//  147   SRMK1 = 0U;     /* enable INTSR1 interrupt */
        CLR1      0xFFFE6.2          ;; 2 cycles
//  148   SREMK1 = 0U;    /* enable INTSRE1 interrupt */
        CLR1      0xFFFE6.3          ;; 2 cycles
//  149 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock2
        ; ------------------------------------- Block: 32 cycles
        ; ------------------------------------- Total: 32 cycles
        REQUIRE __A_SO0
        REQUIRE __A_SOE0
        REQUIRE __A_SS0
        REQUIRE __A_IF1
        REQUIRE __A_MK1
//  150 /***********************************************************************************************************************
//  151 * Function Name: R_UART1_Stop
//  152 * Description  : This function stops the UART1 module operation.
//  153 * Arguments    : None
//  154 * Return Value : None
//  155 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock3 Using cfiCommon0
          CFI Function _R_UART1_Stop
          CFI NoCalls
        CODE
//  156 void R_UART1_Stop(void)
//  157 {
_R_UART1_Stop:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  158   STMK1 = 1U;     /* disable INTST1 interrupt */
        SET1      0xFFFE6.1          ;; 2 cycles
//  159   SRMK1 = 1U;     /* disable INTSR1 interrupt */
        SET1      0xFFFE6.2          ;; 2 cycles
//  160   SREMK1 = 1U;    /* disable INTSRE1 interrupt */
        SET1      0xFFFE6.3          ;; 2 cycles
//  161   ST0 |= _0008_SAUm_CH3_STOP_TRG_ON | _0004_SAUm_CH2_STOP_TRG_ON;
        MOVW      AX, 0x124          ;; 1 cycle
        OR        A, #0x0            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        OR        A, #0xC            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      0x124, AX          ;; 1 cycle
//  162   SOE0 &= (uint16_t)~_0004_SAUm_CH2_OUTPUT_ENABLE;
        CLR1      0xF012A.2          ;; 2 cycles
//  163   STIF1 = 0U;     /* clear INTST1 interrupt flag */
        CLR1      0xFFFE2.1          ;; 2 cycles
//  164   SRIF1 = 0U;     /* clear INTSR1 interrupt flag */
        CLR1      0xFFFE2.2          ;; 2 cycles
//  165   SREIF1 = 0U;    /* clear INTSRE1 interrupt flag */
        CLR1      0xFFFE2.3          ;; 2 cycles
//  166 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock3
        ; ------------------------------------- Block: 26 cycles
        ; ------------------------------------- Total: 26 cycles
        REQUIRE __A_MK1
        REQUIRE __A_ST0
        REQUIRE __A_SOE0
        REQUIRE __A_IF1
//  167 /***********************************************************************************************************************
//  168 * Function Name: R_UART1_Receive
//  169 * Description  : This function receives UART1 data.
//  170 * Arguments    : rx_buf -
//  171 *                    receive buffer pointer
//  172 *                rx_num -
//  173 *                    buffer size
//  174 * Return Value : status -
//  175 *                    MD_OK or MD_ARGERROR
//  176 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock4 Using cfiCommon1
          CFI Function _R_UART1_Receive
          CFI NoCalls
        CODE
//  177 MD_STATUS R_UART1_Receive(uint8_t * const rx_buf, uint16_t rx_num)
//  178 {
_R_UART1_Receive:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
        MOVW      HL, AX             ;; 1 cycle
//  179   MD_STATUS status = MD_OK;
        MOVW      AX, #0x0           ;; 1 cycle
//  180   
//  181   if (rx_num < 1U)
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        BNZ       ??R_UART2_Send_4   ;; 4 cycles
        ; ------------------------------------- Block: 9 cycles
//  182   {
//  183     status = MD_ARGERROR;
        MOVW      DE, #0x81          ;; 1 cycle
        MOVW      AX, DE             ;; 1 cycle
        RET                          ;; 6 cycles
        ; ------------------------------------- Block: 8 cycles
//  184   }
//  185   else
//  186   {
//  187     g_uart1_rx_count = 0U;
??R_UART2_Send_4:
        MOVW      DE, #0x0           ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        MOVW      N:_g_uart1_rx_count, AX  ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
//  188     g_uart1_rx_length = rx_num;
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_g_uart1_rx_length, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  189     gp_uart1_rx_address = rx_buf;
        XCHW      AX, HL             ;; 1 cycle
        MOVW      N:_gp_uart1_rx_address, AX  ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
//  190   }
//  191   
//  192   return (status);
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock4
        ; ------------------------------------- Block: 16 cycles
        ; ------------------------------------- Total: 33 cycles
//  193 }
//  194 /***********************************************************************************************************************
//  195 * Function Name: R_UART1_Send
//  196 * Description  : This function sends UART1 data.
//  197 * Arguments    : tx_buf -
//  198 *                    transfer buffer pointer
//  199 *                tx_num -
//  200 *                    buffer size
//  201 * Return Value : status -
//  202 *                    MD_OK or MD_ARGERROR
//  203 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock5 Using cfiCommon1
          CFI Function _R_UART1_Send
          CFI NoCalls
        CODE
//  204 MD_STATUS R_UART1_Send(uint8_t * const tx_buf, uint16_t tx_num)
//  205 {
_R_UART1_Send:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 2
        SUBW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+6
        MOVW      HL, AX             ;; 1 cycle
//  206   MD_STATUS status = MD_OK;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [SP], AX           ;; 1 cycle
//  207   
//  208   if (tx_num < 1U)
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        BNZ       ??R_UART2_Send_5   ;; 4 cycles
        ; ------------------------------------- Block: 11 cycles
//  209   {
//  210     status = MD_ARGERROR;
        MOVW      AX, #0x81          ;; 1 cycle
        MOVW      [SP], AX           ;; 1 cycle
        BR        S:??R_UART2_Send_6  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  211   }
//  212   else
//  213   {
//  214     gp_uart1_tx_address = tx_buf;
??R_UART2_Send_5:
        XCHW      AX, HL             ;; 1 cycle
        MOVW      N:_gp_uart1_tx_address, AX  ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
//  215     g_uart1_tx_count = tx_num;
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_g_uart1_tx_count, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  216     STMK1 = 1U;    /* disable INTST1 interrupt */
        SET1      0xFFFE6.1          ;; 2 cycles
//  217     TXD1 = *gp_uart1_tx_address;
        MOVW      DE, N:_gp_uart1_tx_address  ;; 1 cycle
        MOV       A, [DE]            ;; 1 cycle
        MOV       0xFFF44, A         ;; 1 cycle
//  218     gp_uart1_tx_address++;
        MOVW      DE, N:_gp_uart1_tx_address  ;; 1 cycle
        INCW      DE                 ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        MOVW      N:_gp_uart1_tx_address, AX  ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
//  219     g_uart1_tx_count--;
        DECW      N:_g_uart1_tx_count  ;; 2 cycles
//  220     STMK1 = 0U;    /* enable INTST1 interrupt */
        CLR1      0xFFFE6.1          ;; 2 cycles
        ; ------------------------------------- Block: 20 cycles
//  221   }
//  222   
//  223   return (status);
??R_UART2_Send_6:
        MOVW      AX, [SP]           ;; 1 cycle
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock5
        ; ------------------------------------- Block: 8 cycles
        ; ------------------------------------- Total: 44 cycles
        REQUIRE __A_MK1
        REQUIRE __A_SDR02
//  224 }
//  225 /***********************************************************************************************************************
//  226 * Function Name: R_SAU1_Create
//  227 * Description  : This function initializes the SAU1 module.
//  228 * Arguments    : None
//  229 * Return Value : None
//  230 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock6 Using cfiCommon0
          CFI Function _R_SAU1_Create
        CODE
//  231 void R_SAU1_Create(void)
//  232 {
_R_SAU1_Create:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  233   SAU1EN = 1U;    /* enables input clock supply */
        SET1      0xF00F0.3          ;; 2 cycles
//  234   NOP();
        NOP                          ;; 1 cycle
//  235   NOP();
        NOP                          ;; 1 cycle
//  236   NOP();
        NOP                          ;; 1 cycle
//  237   NOP();
        NOP                          ;; 1 cycle
//  238   if(clock_select == CLOCK_24MHZ)
        CMP       N:_clock_select, #0x1  ;; 1 cycle
        BNZ       ??R_UART2_Send_7   ;; 4 cycles
        ; ------------------------------------- Block: 11 cycles
//  239   {
//  240     SPS1 = _0040_SAU_CK01_fCLK_4 | _0004_SAU_CK00_fCLK_4;
        MOVW      AX, #0x44          ;; 1 cycle
        MOVW      0x166, AX          ;; 1 cycle
        BR        S:??R_UART2_Send_8  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  241   }
//  242   else if(clock_select == CLOCK_12MHZ)
??R_UART2_Send_7:
        CMP       N:_clock_select, #0x2  ;; 1 cycle
        BNZ       ??R_UART2_Send_9   ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  243   {
//  244     SPS1 = _0030_SAU_CK01_fCLK_3 | _0003_SAU_CK00_fCLK_3;
        MOVW      AX, #0x33          ;; 1 cycle
        MOVW      0x166, AX          ;; 1 cycle
        BR        S:??R_UART2_Send_8  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  245   }
//  246   else if(clock_select == CLOCK_6MHZ)
??R_UART2_Send_9:
        CMP       N:_clock_select, #0x3  ;; 1 cycle
        BNZ       ??R_UART2_Send_10  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  247   {
//  248     SPS1 = _0020_SAU_CK01_fCLK_2 | _0002_SAU_CK00_fCLK_2;
        MOVW      AX, #0x22          ;; 1 cycle
        MOVW      0x166, AX          ;; 1 cycle
        BR        S:??R_UART2_Send_8  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  249   }
//  250   else if(clock_select == CLOCK_1_5MHZ)
??R_UART2_Send_10:
        CMP       N:_clock_select, #0x4  ;; 1 cycle
        BNZ       ??R_UART2_Send_8   ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  251   {
//  252     SPS1 = _0000_SAU_CK01_fCLK_0 | _0000_SAU_CK00_fCLK_0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      0x166, AX          ;; 1 cycle
          CFI FunCall _R_UART2_Create
        ; ------------------------------------- Block: 2 cycles
//  253   }
//  254   
//  255   R_UART2_Create();
??R_UART2_Send_8:
        CALL      _R_UART2_Create    ;; 3 cycles
//  256 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock6
        ; ------------------------------------- Block: 9 cycles
        ; ------------------------------------- Total: 52 cycles
        REQUIRE __A_PER0
        REQUIRE __A_SPS1
//  257 /***********************************************************************************************************************
//  258 * Function Name: R_UART2_Create
//  259 * Description  : This function initializes the UART2 module Optical port
//  260 * Arguments    : None
//  261 * Return Value : None
//  262 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock7 Using cfiCommon0
          CFI Function _R_UART2_Create
          CFI NoCalls
        CODE
//  263 void R_UART2_Create(void)
//  264 {
_R_UART2_Create:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  265   ST1 |= _0002_SAUm_CH1_STOP_TRG_ON | _0001_SAUm_CH0_STOP_TRG_ON;
        MOVW      AX, 0x164          ;; 1 cycle
        OR        A, #0x0            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        OR        A, #0x3            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      0x164, AX          ;; 1 cycle
//  266   STMK2 = 1U;     /* disable INTST2 interrupt */
        SET1      0xFFFE5.0          ;; 2 cycles
//  267   STIF2 = 0U;     /* clear INTST2 interrupt flag */
        CLR1      0xFFFE1.0          ;; 2 cycles
//  268   SRMK2 = 1U;     /* disable INTSR2 interrupt */
        SET1      0xFFFE5.1          ;; 2 cycles
//  269   SRIF2 = 0U;     /* clear INTSR2 interrupt flag */
        CLR1      0xFFFE1.1          ;; 2 cycles
//  270   SREMK2 = 1U;    /* disable INTSRE2 interrupt */
        SET1      0xFFFE5.2          ;; 2 cycles
//  271   SREIF2 = 0U;    /* clear INTSRE2 interrupt flag */
        CLR1      0xFFFE1.2          ;; 2 cycles
//  272   /* Set INTSR2 low priority */
//  273   SRPR12 = 1U;
        SET1      0xFFFED.1          ;; 2 cycles
//  274   SRPR02 = 1U;
        SET1      0xFFFE9.1          ;; 2 cycles
//  275   /* Set INTSRE2 low priority */
//  276   SREPR12 = 1U;
        SET1      0xFFFED.2          ;; 2 cycles
//  277   SREPR02 = 1U;
        SET1      0xFFFE9.2          ;; 2 cycles
//  278   /* Set INTST2 low priority */
//  279   STPR12 = 1U;
        SET1      0xFFFED.0          ;; 2 cycles
//  280   STPR02 = 1U;
        SET1      0xFFFE9.0          ;; 2 cycles
//  281   SMR10 = _0020_SMR10_DEFAULT_VALUE | _0000_SAU_CLOCK_SELECT_CK00 | _0000_SAU_CLOCK_MODE_CKS | 
//  282     _0000_SAU_TRIGGER_SOFTWARE | _0002_SAU_MODE_UART | _0000_SAU_TRANSFER_END;
        MOVW      AX, #0x22          ;; 1 cycle
        MOVW      0x150, AX          ;; 1 cycle
//  283   SCR10 = _0004_SCR10_DEFAULT_VALUE | _8000_SAU_TRANSMISSION | _0000_SAU_TIMING_1 | _0000_SAU_INTSRE_MASK | 
//  284     _0000_SAU_PARITY_NONE | _0080_SAU_LSB | _0010_SAU_STOP_1 | _0003_SAU_LENGTH_8;
        MOVW      AX, #0x8097        ;; 1 cycle
        MOVW      0x158, AX          ;; 1 cycle
//  285   SDR10 = _9A00_SAU1_CH0_BAUDRATE_DIVISOR;
        MOVW      0xFFF48, #0x9A00   ;; 1 cycle
//  286   NFEN0 |= _10_SAU_RXD2_FILTER_ON;
        SET1      0xF0070.4          ;; 2 cycles
//  287   SIR11 = _0004_SAU_SIRMN_FECTMN | _0002_SAU_SIRMN_PECTMN | _0001_SAU_SIRMN_OVCTMN;
        MOVW      AX, #0x7           ;; 1 cycle
        MOVW      0x14A, AX          ;; 1 cycle
//  288   SMR11 = _0020_SMR11_DEFAULT_VALUE | _0000_SAU_CLOCK_SELECT_CK00 | _0000_SAU_CLOCK_MODE_CKS | 
//  289     _0100_SAU_TRIGGER_RXD | _0000_SAU_EDGE_FALL | _0002_SAU_MODE_UART | _0000_SAU_TRANSFER_END;
        MOVW      AX, #0x122         ;; 1 cycle
        MOVW      0x152, AX          ;; 1 cycle
//  290   SCR11 = _0004_SCR11_DEFAULT_VALUE | _4000_SAU_RECEPTION | _0000_SAU_TIMING_1 | _0400_SAU_INTSRE_ENABLE | 
//  291     _0000_SAU_PARITY_NONE | _0080_SAU_LSB | _0010_SAU_STOP_1 | _0003_SAU_LENGTH_8;
        MOVW      AX, #0x4497        ;; 1 cycle
        MOVW      0x15A, AX          ;; 1 cycle
//  292   SDR11 = _9A00_SAU1_CH1_BAUDRATE_DIVISOR;
        MOVW      0xFFF4A, #0x9A00   ;; 1 cycle
//  293   SO1 |= _0001_SAUm_CH0_DATA_OUTPUT_1;
        MOVW      AX, 0x168          ;; 1 cycle
        OR        A, #0x0            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        OR        A, #0x1            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      0x168, AX          ;; 1 cycle
//  294   SOL1 &= (uint16_t)~_0001_SAUm_CHANNEL0_INVERTED;
        MOVW      AX, 0x174          ;; 1 cycle
        AND       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        AND       A, #0xFE           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      0x174, AX          ;; 1 cycle
//  295   SOE1 |= _0001_SAUm_CH0_OUTPUT_ENABLE;
        SET1      0xF016A.0          ;; 2 cycles
//  296 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock7
        ; ------------------------------------- Block: 64 cycles
        ; ------------------------------------- Total: 64 cycles
        REQUIRE __A_ST1
        REQUIRE __A_MK0
        REQUIRE __A_IF0
        REQUIRE __A_PR10
        REQUIRE __A_PR00
        REQUIRE __A_SMR10
        REQUIRE __A_SCR10
        REQUIRE __A_SDR10
        REQUIRE __A_NFEN0
        REQUIRE __A_SIR11
        REQUIRE __A_SMR11
        REQUIRE __A_SCR11
        REQUIRE __A_SDR11
        REQUIRE __A_SO1
        REQUIRE __A_SOL1
        REQUIRE __A_SOE1
//  297 /***********************************************************************************************************************
//  298 * Function Name: R_UART2_Start
//  299 * Description  : This function starts the UART2 module operation.
//  300 * Arguments    : None
//  301 * Return Value : None
//  302 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock8 Using cfiCommon0
          CFI Function _R_UART2_Start
          CFI NoCalls
        CODE
//  303 void R_UART2_Start(void)
//  304 {
_R_UART2_Start:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  305   SO1 |= _0001_SAUm_CH0_DATA_OUTPUT_1;
        MOVW      AX, 0x168          ;; 1 cycle
        OR        A, #0x0            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        OR        A, #0x1            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      0x168, AX          ;; 1 cycle
//  306   SOE1 |= _0001_SAUm_CH0_OUTPUT_ENABLE;
        SET1      0xF016A.0          ;; 2 cycles
//  307   SS1 |= _0002_SAUm_CH1_START_TRG_ON | _0001_SAUm_CH0_START_TRG_ON;
        MOVW      AX, 0x162          ;; 1 cycle
        OR        A, #0x0            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        OR        A, #0x3            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      0x162, AX          ;; 1 cycle
//  308   STIF2 = 0U;     /* clear INTST2 interrupt flag */
        CLR1      0xFFFE1.0          ;; 2 cycles
//  309   SRIF2 = 0U;     /* clear INTSR2 interrupt flag */
        CLR1      0xFFFE1.1          ;; 2 cycles
//  310   SREIF2 = 0U;    /* clear INTSRE2 interrupt flag */
        CLR1      0xFFFE1.2          ;; 2 cycles
//  311   STMK2 = 0U;     /* enable INTST2 interrupt */
        CLR1      0xFFFE5.0          ;; 2 cycles
//  312   SRMK2 = 0U;     /* enable INTSR2 interrupt */
        CLR1      0xFFFE5.1          ;; 2 cycles
//  313   SREMK2 = 0U;    /* enable INTSRE2 interrupt */
        CLR1      0xFFFE5.2          ;; 2 cycles
//  314 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock8
        ; ------------------------------------- Block: 32 cycles
        ; ------------------------------------- Total: 32 cycles
        REQUIRE __A_SO1
        REQUIRE __A_SOE1
        REQUIRE __A_SS1
        REQUIRE __A_IF0
        REQUIRE __A_MK0
//  315 /***********************************************************************************************************************
//  316 * Function Name: R_UART2_Stop
//  317 * Description  : This function stops the UART2 module operation.
//  318 * Arguments    : None
//  319 * Return Value : None
//  320 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock9 Using cfiCommon0
          CFI Function _R_UART2_Stop
          CFI NoCalls
        CODE
//  321 void R_UART2_Stop(void)
//  322 {
_R_UART2_Stop:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  323   STMK2 = 1U;     /* disable INTST2 interrupt */
        SET1      0xFFFE5.0          ;; 2 cycles
//  324   SRMK2 = 1U;     /* disable INTSR2 interrupt */
        SET1      0xFFFE5.1          ;; 2 cycles
//  325   SREMK2 = 1U;    /* disable INTSRE2 interrupt */
        SET1      0xFFFE5.2          ;; 2 cycles
//  326   ST1 |= _0002_SAUm_CH1_STOP_TRG_ON | _0001_SAUm_CH0_STOP_TRG_ON;
        MOVW      AX, 0x164          ;; 1 cycle
        OR        A, #0x0            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        OR        A, #0x3            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      0x164, AX          ;; 1 cycle
//  327   SOE1 &= (uint16_t)~_0001_SAUm_CH0_OUTPUT_ENABLE;
        CLR1      0xF016A.0          ;; 2 cycles
//  328   STIF2 = 0U;     /* clear INTST2 interrupt flag */
        CLR1      0xFFFE1.0          ;; 2 cycles
//  329   SRIF2 = 0U;     /* clear INTSR2 interrupt flag */
        CLR1      0xFFFE1.1          ;; 2 cycles
//  330   SREIF2 = 0U;    /* clear INTSRE2 interrupt flag */
        CLR1      0xFFFE1.2          ;; 2 cycles
//  331 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock9
        ; ------------------------------------- Block: 26 cycles
        ; ------------------------------------- Total: 26 cycles
        REQUIRE __A_MK0
        REQUIRE __A_ST1
        REQUIRE __A_SOE1
        REQUIRE __A_IF0
//  332 /***********************************************************************************************************************
//  333 * Function Name: R_UART2_Receive
//  334 * Description  : This function receives UART2 data.
//  335 * Arguments    : rx_buf -
//  336 *                    receive buffer pointer
//  337 *                rx_num -
//  338 *                    buffer size
//  339 * Return Value : status -
//  340 *                    MD_OK or MD_ARGERROR
//  341 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock10 Using cfiCommon1
          CFI Function _R_UART2_Receive
          CFI NoCalls
        CODE
//  342 MD_STATUS R_UART2_Receive(uint8_t * const rx_buf, uint16_t rx_num)
//  343 {
_R_UART2_Receive:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
        MOVW      HL, AX             ;; 1 cycle
//  344   MD_STATUS status = MD_OK;
        MOVW      AX, #0x0           ;; 1 cycle
//  345   
//  346   if (rx_num < 1U)
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        BNZ       ??R_UART2_Send_11  ;; 4 cycles
        ; ------------------------------------- Block: 9 cycles
//  347   {
//  348     status = MD_ARGERROR;
        MOVW      DE, #0x81          ;; 1 cycle
        MOVW      AX, DE             ;; 1 cycle
        RET                          ;; 6 cycles
        ; ------------------------------------- Block: 8 cycles
//  349   }
//  350   else
//  351   {
//  352     g_uart2_rx_count = 0U;
??R_UART2_Send_11:
        MOVW      DE, #0x0           ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        MOVW      N:_g_uart2_rx_count, AX  ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
//  353     g_uart2_rx_length = rx_num;
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_g_uart2_rx_length, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  354     gp_uart2_rx_address = rx_buf;
        XCHW      AX, HL             ;; 1 cycle
        MOVW      N:_gp_uart2_rx_address, AX  ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
//  355   }
//  356   
//  357   return (status);
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock10
        ; ------------------------------------- Block: 16 cycles
        ; ------------------------------------- Total: 33 cycles
//  358 }
//  359 /***********************************************************************************************************************
//  360 * Function Name: R_UART2_Send
//  361 * Description  : This function sends UART2 data.
//  362 * Arguments    : tx_buf -
//  363 *                    transfer buffer pointer
//  364 *                tx_num -
//  365 *                    buffer size
//  366 * Return Value : status -
//  367 *                    MD_OK or MD_ARGERROR
//  368 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock11 Using cfiCommon1
          CFI Function _R_UART2_Send
          CFI NoCalls
        CODE
//  369 MD_STATUS R_UART2_Send(uint8_t * const tx_buf, uint16_t tx_num)
//  370 {
_R_UART2_Send:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 2
        SUBW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+6
        MOVW      HL, AX             ;; 1 cycle
//  371   MD_STATUS status = MD_OK;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [SP], AX           ;; 1 cycle
//  372   
//  373   if (tx_num < 1U)
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        BNZ       ??R_UART2_Send_12  ;; 4 cycles
        ; ------------------------------------- Block: 11 cycles
//  374   {
//  375     status = MD_ARGERROR;
        MOVW      AX, #0x81          ;; 1 cycle
        MOVW      [SP], AX           ;; 1 cycle
        BR        S:??R_UART2_Send_13  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  376   }
//  377   else
//  378   {
//  379     gp_uart2_tx_address = tx_buf;
??R_UART2_Send_12:
        XCHW      AX, HL             ;; 1 cycle
        MOVW      N:_gp_uart2_tx_address, AX  ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
//  380     g_uart2_tx_count = tx_num;
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_g_uart2_tx_count, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  381     STMK2 = 1U;    /* disable INTST2 interrupt */
        SET1      0xFFFE5.0          ;; 2 cycles
//  382     TXD2 = *gp_uart2_tx_address;
        MOVW      DE, N:_gp_uart2_tx_address  ;; 1 cycle
        MOV       A, [DE]            ;; 1 cycle
        MOV       0xFFF48, A         ;; 1 cycle
//  383     gp_uart2_tx_address++;
        MOVW      DE, N:_gp_uart2_tx_address  ;; 1 cycle
        INCW      DE                 ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        MOVW      N:_gp_uart2_tx_address, AX  ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
//  384     g_uart2_tx_count--;
        DECW      N:_g_uart2_tx_count  ;; 2 cycles
//  385     STMK2 = 0U;    /* enable INTST2 interrupt */
        CLR1      0xFFFE5.0          ;; 2 cycles
        ; ------------------------------------- Block: 20 cycles
//  386   }
//  387   
//  388   return (status);
??R_UART2_Send_13:
        MOVW      AX, [SP]           ;; 1 cycle
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock11
        ; ------------------------------------- Block: 8 cycles
        ; ------------------------------------- Total: 44 cycles
        REQUIRE __A_MK0
        REQUIRE __A_SDR10
//  389 }

        SECTION `.iar_vfe_header`:DATA:NOALLOC:NOROOT(1)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
        DC32 0

        END
//  390 
//  391 /* Start user code for adding. Do not edit comment generated here */
//  392 /* End user code. Do not edit comment generated here */
// 
//  20 bytes in section .bss
//  70 bytes in section .bss.noinit  (abs)
// 726 bytes in section .text
// 
// 726 bytes of CODE memory
//  20 bytes of DATA memory (+ 70 bytes shared)
//
//Errors: none
//Warnings: none
