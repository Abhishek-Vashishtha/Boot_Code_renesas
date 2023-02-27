///////////////////////////////////////////////////////////////////////////////
//
// IAR C/C++ Compiler V4.20.1.2260 for RL78               20/Dec/2020  22:38:56
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
//        BootCode\source_code\driver_files\r_cg_rtc.c
//    Command line       =
//        -f C:\Users\laptop\AppData\Local\Temp\EWF0C0.tmp ("D:\Dheeraj\New
//        folder\0. GDEV72 - BootCode\source_code\driver_files\r_cg_rtc.c"
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
//        D:\Dheeraj\New folder\0. GDEV72 - BootCode\Debug\List\r_cg_rtc.s
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

        EXTERN _flag_rtc1
        EXTERN _delay_ms

        PUBLIC _CheckForSync
        PUBLIC _RTC_1Hz_Output_Set
        PUBLIC _RTC_Set_Clock_Correction
        PUBLIC _R_RTC_Create
        PUBLIC _R_RTC_Get_CalendarAlarmValue
        PUBLIC _R_RTC_Get_CalendarCounterValue
        PUBLIC _R_RTC_Set_AlarmOff
        PUBLIC _R_RTC_Set_CalendarAlarmOn
        PUBLIC _R_RTC_Set_CalendarAlarmValue
        PUBLIC _R_RTC_Set_CalendarCounterValue
        PUBLIC _R_RTC_Set_ConstPeriodInterruptOff
        PUBLIC _R_RTC_Set_ConstPeriodInterruptOn
        PUBLIC _R_RTC_Set_PowerOff
        PUBLIC _R_RTC_Set_RTCOUTOff
        PUBLIC _R_RTC_Set_RTCOUTOn
        PUBLIC _R_RTC_Start
        PUBLIC _R_RTC_Stop
        PUBLIC __A_BCNT0
        PUBLIC __A_BCNT0AER
        PUBLIC __A_BCNT0AR
        PUBLIC __A_BCNT1
        PUBLIC __A_BCNT1AER
        PUBLIC __A_BCNT1AR
        PUBLIC __A_BCNT2
        PUBLIC __A_BCNT2AER
        PUBLIC __A_BCNT2AR
        PUBLIC __A_BCNT3
        PUBLIC __A_BCNT3AER
        PUBLIC __A_BCNT3AR
        PUBLIC __A_IF1
        PUBLIC __A_LVDVRTC
        PUBLIC __A_MK1
        PUBLIC __A_PER2
        PUBLIC __A_PM6
        PUBLIC __A_PR01
        PUBLIC __A_PR11
        PUBLIC __A_RADJ
        PUBLIC __A_RCR1
        PUBLIC __A_RCR2
        PUBLIC __A_RCR4
        PUBLIC __A_RCR5
        PUBLIC __A_RCR5GD
        PUBLIC __A_RDAYCNT
        PUBLIC __A_RMONCNT
        PUBLIC __A_RSR
        PUBLIC __A_RTCPORSR
        PUBLIC __A_RYRCNT
        
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
        
// D:\Dheeraj\New folder\0. GDEV72 - BootCode\source_code\driver_files\r_cg_rtc.c
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
//   21 * File Name    : r_cg_rtc.c
//   22 * Version      : Applilet4 for RL78/I1C V1.01.04.02 [20 Nov 2019]
//   23 * Device(s)    : R5F10NPJ
//   24 * Tool-Chain   : IAR Systems icc78k0r
//   25 * Description  : This file implements device driver for RTC module.
//   26 * Creation Date: 06/04/2020
//   27 ***********************************************************************************************************************/
//   28 
//   29 /***********************************************************************************************************************
//   30 Includes
//   31 ***********************************************************************************************************************/
//   32 #include "r_cg_macrodriver.h"

        ASEGN `.bss.noinit`:DATA:NOROOT,0fff26H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_PM6
// __no_init union <unnamed>#41 volatile __sfr _A_PM6
__A_PM6:
        DS 1

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

        ASEGN `.bss.noinit`:DATA:NOROOT,0f00fcH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_PER2
// __no_init union <unnamed>#291 volatile _A_PER2
__A_PER2:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0334H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_LVDVRTC
// __no_init union <unnamed>#548 volatile _A_LVDVRTC
__A_LVDVRTC:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0380H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_RTCPORSR
// __no_init union <unnamed>#566 volatile __no_bit_access _A_RTCPORSR
__A_RTCPORSR:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0583H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_BCNT0
// __no_init union <unnamed>#657 volatile __no_bit_access _A_BCNT0
__A_BCNT0:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0585H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_BCNT1
// __no_init union <unnamed>#658 volatile __no_bit_access _A_BCNT1
__A_BCNT1:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0587H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_BCNT2
// __no_init union <unnamed>#659 volatile __no_bit_access _A_BCNT2
__A_BCNT2:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0589H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_BCNT3
// __no_init union <unnamed>#660 volatile __no_bit_access _A_BCNT3
__A_BCNT3:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f058bH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_RDAYCNT
// __no_init union <unnamed>#661 volatile __no_bit_access _A_RDAYCNT
__A_RDAYCNT:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f058dH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_RMONCNT
// __no_init union <unnamed>#662 volatile __no_bit_access _A_RMONCNT
__A_RMONCNT:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f058eH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_RYRCNT
// __no_init union <unnamed>#663 volatile __no_bit_access _A_RYRCNT
__A_RYRCNT:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0591H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_BCNT0AR
// __no_init union <unnamed>#664 volatile __no_bit_access _A_BCNT0AR
__A_BCNT0AR:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0593H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_BCNT1AR
// __no_init union <unnamed>#665 volatile __no_bit_access _A_BCNT1AR
__A_BCNT1AR:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0595H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_BCNT2AR
// __no_init union <unnamed>#666 volatile __no_bit_access _A_BCNT2AR
__A_BCNT2AR:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0597H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_BCNT3AR
// __no_init union <unnamed>#667 volatile __no_bit_access _A_BCNT3AR
__A_BCNT3AR:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0599H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_BCNT0AER
// __no_init union <unnamed>#668 volatile __no_bit_access _A_BCNT0AER
__A_BCNT0AER:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f059bH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_BCNT1AER
// __no_init union <unnamed>#669 volatile __no_bit_access _A_BCNT1AER
__A_BCNT1AER:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f059cH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_BCNT2AER
// __no_init union <unnamed>#670 volatile __no_bit_access _A_BCNT2AER
__A_BCNT2AER:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f059fH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_BCNT3AER
// __no_init union <unnamed>#671 volatile _A_BCNT3AER
__A_BCNT3AER:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f05a1H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_RSR
// __no_init union <unnamed>#672 volatile _A_RSR
__A_RSR:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f05a3H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_RCR1
// __no_init union <unnamed>#673 volatile _A_RCR1
__A_RCR1:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f05a5H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_RCR2
// __no_init union <unnamed>#674 volatile _A_RCR2
__A_RCR2:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f05a9H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_RCR4
// __no_init union <unnamed>#676 volatile _A_RCR4
__A_RCR4:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f05afH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_RADJ
// __no_init union <unnamed>#677 volatile __no_bit_access _A_RADJ
__A_RADJ:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f05b3H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_RCR5
// __no_init union <unnamed>#678 volatile __no_bit_access _A_RCR5
__A_RCR5:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f05b9H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_RCR5GD
// __no_init union <unnamed>#679 volatile __no_bit_access _A_RCR5GD
__A_RCR5GD:
        DS 1
//   33 #include "r_cg_rtc.h"
//   34 #include "common_function.h"
//   35 /* Start user code for include. Do not edit comment generated here */
//   36 /* End user code. Do not edit comment generated here */
//   37 #include "r_cg_userdefine.h"
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
//   49 us8 CheckForSync(volatile us8 *ptr, us8 val, us8 cntr);
//   50 void  RTC_Set_Clock_Correction(us8 correction);
//   51 uint8_t RTC_1Hz_Output_Set(us8 choice);
//   52 /* End user code. Do not edit comment generated here */
//   53 
//   54 /***********************************************************************************************************************
//   55 * Function Name: R_RTC_Create
//   56 * Description  : This function initializes the real-time clock module.
//   57 * Arguments    : None
//   58 * Return Value : None
//   59 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock0 Using cfiCommon0
          CFI Function _R_RTC_Create
        CODE
//   60 us8 R_RTC_Create(void)
//   61 {
_R_RTC_Create:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 4
        SUBW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
//   62   volatile uint8_t tmp;
//   63   volatile uint16_t w_count;
//   64   
//   65   VRTCEN = 1U;    /* enables input clock supply */
        SET1      0xF00FC.0          ;; 2 cycles
//   66   delay_ms(10);                         /* A 10 ms delay is required for proper initialization of Clock supply */
        MOVW      AX, #0xA           ;; 1 cycle
          CFI FunCall _delay_ms
        CALL      _delay_ms          ;; 3 cycles
//   67   /* Check for Low Voltage, if VRTC < 2.16 V then not to init RTC */
//   68   if(LVDVRTCF == 0)
        MOVW      HL, #0x334         ;; 1 cycle
        MOV1      CY, [HL].6         ;; 1 cycle
        SKNC                         ;; 4 cycles
        BR        N:??RTC_Set_Clock_Correction_0  ;; 4 cycles
        ; ------------------------------------- Block: 13 cycles
//   69   {
//   70     RTCRMK = 1U;    /* disable INTRTCRPD interrupt */
        SET1      0xFFFE7.3          ;; 2 cycles
//   71     RTCRIF = 0U;    /* clear INTRTCRPD interrupt flag */
        CLR1      0xFFFE3.3          ;; 2 cycles
//   72     RTCAMK = 1U;    /* disable INTRTCALM interrupt */
        SET1      0xFFFE7.3          ;; 2 cycles
//   73     RTCAIF = 0U;    /* clear INTRTCALM interrupt flag */
        CLR1      0xFFFE3.3          ;; 2 cycles
//   74     /* Set INTRTCRPD low priority */
//   75     RTCRPR1 = 1U;
        SET1      0xFFFEF.3          ;; 2 cycles
//   76     RTCRPR0 = 1U;
        SET1      0xFFFEB.3          ;; 2 cycles
//   77     
//   78     if((RTCPORSR == 0) || (flag_rtc1_fail==1))
        CMP0      0x380              ;; 1 cycle
        BZ        ??RTC_Set_Clock_Correction_1  ;; 4 cycles
        ; ------------------------------------- Block: 17 cycles
        MOVW      HL, #LWRD(_flag_rtc1)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        SKC                          ;; 4 cycles
        BR        N:??RTC_Set_Clock_Correction_2  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//   79     {
//   80       flag_rtc1_fail=0;
??RTC_Set_Clock_Correction_1:
        CLR1      N:_flag_rtc1.0     ;; 2 cycles
//   81       /* Selecting Sub System Clock 32.768 Khz as a Count Clock */
//   82       RCR4 = _00_RTC_SELECT_FSUB;
        MOV       0x5A9, #0x0        ;; 1 cycle
//   83       
//   84       /* Stop all counters : START is bit 0 */
//   85       tmp = RCR2;	
        MOV       A, 0x5A5           ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//   86       tmp &= (uint8_t)~_01_RTC_COUNTER_NORMAL;
        MOV       A, [SP]            ;; 1 cycle
        AND       A, #0xFE           ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//   87       RCR2 = tmp;
        MOV       A, [SP]            ;; 1 cycle
        MOV       0x5A5, A           ;; 1 cycle
//   88       if(CheckForSync(&(RCR2),tmp,10) == 0)      /* 1- No Error, 0- Error */
        MOV       B, #0xA            ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        MOV       C, A               ;; 1 cycle
        MOVW      AX, #0x5A5         ;; 1 cycle
          CFI FunCall _CheckForSync
        CALL      _CheckForSync      ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??RTC_Set_Clock_Correction_3  ;; 4 cycles
        ; ------------------------------------- Block: 22 cycles
//   89       {
//   90         return 3;
        MOV       A, #0x3            ;; 1 cycle
        BR        N:??RTC_Set_Clock_Correction_4  ;; 3 cycles
        ; ------------------------------------- Block: 4 cycles
//   91       }
//   92       /* Select RTC in Calender Count Mode */
//   93       tmp = RCR2;
??RTC_Set_Clock_Correction_3:
        MOV       A, 0x5A5           ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//   94       tmp &= (uint8_t)~_80_RTC_BINARY_MODE;
        MOV       A, [SP]            ;; 1 cycle
        AND       A, #0x7F           ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//   95       RCR2 = tmp;
        MOV       A, [SP]            ;; 1 cycle
        MOV       0x5A5, A           ;; 1 cycle
//   96       
//   97       if(CheckForSync(&(RCR2),tmp,10) == 0)      /* 1- No Error, 0- Error */
        MOV       B, #0xA            ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        MOV       C, A               ;; 1 cycle
        MOVW      AX, #0x5A5         ;; 1 cycle
          CFI FunCall _CheckForSync
        CALL      _CheckForSync      ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??RTC_Set_Clock_Correction_5  ;; 4 cycles
        ; ------------------------------------- Block: 19 cycles
//   98       {
//   99         return 4;
        MOV       A, #0x4            ;; 1 cycle
        BR        N:??RTC_Set_Clock_Correction_4  ;; 3 cycles
        ; ------------------------------------- Block: 4 cycles
//  100       }
//  101       
//  102       /* Execute RTC software reset : RESET is bit 1 */
//  103       tmp = RCR2;
??RTC_Set_Clock_Correction_5:
        MOV       A, 0x5A5           ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  104       tmp |= _02_RTC_RESET_WRITER_INITIALIZED;
        MOV       A, [SP]            ;; 1 cycle
        OR        A, #0x2            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  105       RCR2 = tmp;
        MOV       A, [SP]            ;; 1 cycle
        MOV       0x5A5, A           ;; 1 cycle
//  106       
//  107       tmp &= (~BIT1);
        MOV       A, [SP]            ;; 1 cycle
        AND       A, #0xFD           ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  108       if(CheckForSync(&(RCR2),tmp,10) == 0)      /* 1- No Error, 0- Error */
        MOV       B, #0xA            ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        MOV       C, A               ;; 1 cycle
        MOVW      AX, #0x5A5         ;; 1 cycle
          CFI FunCall _CheckForSync
        CALL      _CheckForSync      ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??RTC_Set_Clock_Correction_6  ;; 4 cycles
        ; ------------------------------------- Block: 22 cycles
//  109       {
//  110         return 5;
        MOV       A, #0x5            ;; 1 cycle
        BR        S:??RTC_Set_Clock_Correction_4  ;; 3 cycles
        ; ------------------------------------- Block: 4 cycles
//  111       }
//  112       
//  113       /* Set control registers */
//  114       tmp = _00_RTC_PERIODIC_DISABLE | _00_RTC_ALARM_DISABLE;
??RTC_Set_Clock_Correction_6:
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  115       RCR1 = tmp;
        MOV       A, [SP]            ;; 1 cycle
        MOV       0x5A3, A           ;; 1 cycle
//  116       if(CheckForSync(&(RCR1),tmp,10) == 0)      /* 1- No Error, 0- Error */
        MOV       B, #0xA            ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        MOV       C, A               ;; 1 cycle
        MOVW      AX, #0x5A3         ;; 1 cycle
          CFI FunCall _CheckForSync
        CALL      _CheckForSync      ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??RTC_Set_Clock_Correction_7  ;; 4 cycles
        ; ------------------------------------- Block: 16 cycles
//  117       {
//  118         return 6;
        MOV       A, #0x6            ;; 1 cycle
        BR        S:??RTC_Set_Clock_Correction_4  ;; 3 cycles
        ; ------------------------------------- Block: 4 cycles
//  119       }
//  120       
//  121       tmp = _00_RTC_CALENDER_MODE | _40_RTC_24HOUR_MODE | _20_RTC_CALENDER_10SECONDS | _10_RTC_ADJUSTMENT_ENABLE | _00_RTC_RTCOUT_DISABLE;
??RTC_Set_Clock_Correction_7:
        MOV       A, #0x70           ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  122       RCR2 = tmp;
        MOV       A, [SP]            ;; 1 cycle
        MOV       0x5A5, A           ;; 1 cycle
//  123       if(CheckForSync(&(RCR2),tmp,10) == 0)      /* 1- No Error, 0- Error */
        MOV       B, #0xA            ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        MOV       C, A               ;; 1 cycle
        MOVW      AX, #0x5A5         ;; 1 cycle
          CFI FunCall _CheckForSync
        CALL      _CheckForSync      ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??RTC_Set_Clock_Correction_8  ;; 4 cycles
        ; ------------------------------------- Block: 16 cycles
//  124       {
//  125         return 7;
        MOV       A, #0x7            ;; 1 cycle
        BR        S:??RTC_Set_Clock_Correction_4  ;; 3 cycles
        ; ------------------------------------- Block: 4 cycles
//  126       }
//  127       
//  128       /* Unlock, set and lock the RCR5 */
//  129       RCR5GD = 0x00U;
??RTC_Set_Clock_Correction_8:
        MOV       0x5B9, #0x0        ;; 1 cycle
//  130       RCR5GD = 0x72U;
        MOV       0x5B9, #0x72       ;; 1 cycle
//  131       RCR5GD = 0x64U;
        MOV       0x5B9, #0x64       ;; 1 cycle
//  132       RCR5 = 0x00U;
        MOV       0x5B3, #0x0        ;; 1 cycle
//  133       RCR5GD = 0x00U;
        MOV       0x5B9, #0x0        ;; 1 cycle
//  134       
//  135       
//  136       /* Change the waiting time according to the system */
//  137       for (w_count = 0U; w_count <= RTC_6CLOCKWAITTIME; w_count++)
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [SP+0x02], AX      ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
??R_RTC_Create_0:
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        CMPW      AX, #0x1F1         ;; 1 cycle
        BNC       ??RTC_Set_Clock_Correction_9  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  138       {
//  139         NOP();
        NOP                          ;; 1 cycle
//  140       }
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        INCW      AX                 ;; 1 cycle
        MOVW      [SP+0x02], AX      ;; 1 cycle
        BR        S:??R_RTC_Create_0  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
//  141 #if RTC_DEFAULT_TIME == 1      
//  142       Now.day = 0x01;
//  143       Now.month = MONTH_JAN;
//  144       Now.year = 0x20;
//  145       Now.hour = 0x00;
//  146       Now.min = 0x00;
//  147       Now.sec = 0x00;
//  148       Now.week = DAY_WED;
//  149       R_RTC_Set_CalendarCounterValue(Now);
//  150       flag_rtc_def_time_loaded = 1;
//  151 #endif      
//  152       RTCPORSR = 1U;
??RTC_Set_Clock_Correction_9:
        MOV       0x380, #0x1        ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
//  153     }
//  154   }
//  155   else
//  156   {
//  157     return 1;
//  158   }
//  159   
//  160   return 0;
??RTC_Set_Clock_Correction_2:
        MOV       A, #0x0            ;; 1 cycle
        BR        S:??RTC_Set_Clock_Correction_4  ;; 3 cycles
        ; ------------------------------------- Block: 4 cycles
??RTC_Set_Clock_Correction_0:
        MOV       A, #0x1            ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??RTC_Set_Clock_Correction_4:
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock0
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 184 cycles
        REQUIRE __A_PER2
        REQUIRE __A_LVDVRTC
        REQUIRE __A_MK1
        REQUIRE __A_IF1
        REQUIRE __A_PR11
        REQUIRE __A_PR01
        REQUIRE __A_RTCPORSR
        REQUIRE __A_RCR4
        REQUIRE __A_RCR2
        REQUIRE __A_RCR1
        REQUIRE __A_RCR5GD
        REQUIRE __A_RCR5
//  161 }
//  162 
//  163 /***********************************************************************************************************************
//  164 * Function Name: R_RTC_Start
//  165 * Description  : This function enables the real-time clock.
//  166 * Arguments    : None
//  167 * Return Value : None
//  168 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock1 Using cfiCommon0
          CFI Function _R_RTC_Start
        CODE
//  169 us8 R_RTC_Start(void)
//  170 {
_R_RTC_Start:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 2
        SUBW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+6
//  171   volatile uint8_t tmp;
//  172   
//  173   tmp = RCR2;
        MOV       A, 0x5A5           ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  174   tmp |= _01_RTC_COUNTER_NORMAL;
        MOV       A, [SP]            ;; 1 cycle
        OR        A, #0x1            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  175   RCR2 = tmp;
        MOV       A, [SP]            ;; 1 cycle
        MOV       0x5A5, A           ;; 1 cycle
//  176   if(CheckForSync(&(RCR2),tmp,10) == 0)      /* 1- No Error, 0- Error */
        MOV       B, #0xA            ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        MOV       C, A               ;; 1 cycle
        MOVW      AX, #0x5A5         ;; 1 cycle
          CFI FunCall _CheckForSync
        CALL      _CheckForSync      ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??RTC_Set_Clock_Correction_10  ;; 4 cycles
        ; ------------------------------------- Block: 20 cycles
//  177   {
//  178     return 8;
        MOV       A, #0x8            ;; 1 cycle
        BR        S:??RTC_Set_Clock_Correction_11  ;; 3 cycles
        ; ------------------------------------- Block: 4 cycles
//  179   }
//  180   return 0;
??RTC_Set_Clock_Correction_10:
        MOV       A, #0x0            ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??RTC_Set_Clock_Correction_11:
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock1
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 32 cycles
        REQUIRE __A_RCR2
//  181 }
//  182 
//  183 /***********************************************************************************************************************
//  184 * Function Name: R_RTC_Stop
//  185 * Description  : This function disables the real-time clock.
//  186 * Arguments    : None
//  187 * Return Value : None
//  188 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock2 Using cfiCommon0
          CFI Function _R_RTC_Stop
          CFI NoCalls
        CODE
//  189 void R_RTC_Stop(void)
//  190 {
_R_RTC_Stop:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 4
        SUBW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
//  191   volatile uint8_t tmp;
//  192   volatile uint16_t w_count;
//  193   
//  194   tmp = RCR2;
        MOV       A, 0x5A5           ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  195   tmp &= (uint8_t)~_01_RTC_COUNTER_NORMAL;
        MOV       A, [SP]            ;; 1 cycle
        AND       A, #0xFE           ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  196   RCR2 = tmp;
        MOV       A, [SP]            ;; 1 cycle
        MOV       0x5A5, A           ;; 1 cycle
//  197   
//  198   /* Change the waiting time according to the system */
//  199   for (w_count = 0U; w_count <= RTC_STARTWAITTIME; w_count++)
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [SP+0x02], AX      ;; 1 cycle
        ; ------------------------------------- Block: 10 cycles
??R_RTC_Stop_0:
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        CMPW      AX, #0x2           ;; 1 cycle
        BNC       ??RTC_Set_Clock_Correction_12  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  200   {
//  201     NOP();
        NOP                          ;; 1 cycle
//  202   }
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        INCW      AX                 ;; 1 cycle
        MOVW      [SP+0x02], AX      ;; 1 cycle
        BR        S:??R_RTC_Stop_0   ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
//  203   
//  204   tmp = RCR2;
??RTC_Set_Clock_Correction_12:
        MOV       A, 0x5A5           ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  205   tmp |= _02_RTC_RESET_WRITER_INITIALIZED;
        MOV       A, [SP]            ;; 1 cycle
        OR        A, #0x2            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  206   RCR2 = tmp;
        MOV       A, [SP]            ;; 1 cycle
        MOV       0x5A5, A           ;; 1 cycle
//  207   
//  208   /* Change the waiting time according to the system */
//  209   for (w_count = 0U; w_count <= RTC_RESETWAITTIME; w_count++)
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [SP+0x02], AX      ;; 1 cycle
        ; ------------------------------------- Block: 9 cycles
??R_RTC_Stop_1:
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        CMPW      AX, #0x15F         ;; 1 cycle
        BNC       ??RTC_Set_Clock_Correction_13  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  210   {
//  211     NOP();
        NOP                          ;; 1 cycle
//  212   }
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        INCW      AX                 ;; 1 cycle
        MOVW      [SP+0x02], AX      ;; 1 cycle
        BR        S:??R_RTC_Stop_1   ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
//  213   
//  214 }
??RTC_Set_Clock_Correction_13:
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock2
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 52 cycles
        REQUIRE __A_RCR2
//  215 
//  216 
//  217 /***********************************************************************************************************************
//  218 * Function Name: R_RTC_Set_CalendarCounterValue
//  219 * Description  : This function changes the calendar real-time clock value.
//  220 * Arguments    : counter_write_val -
//  221 *                    the expected real-time clock value(BCD code)
//  222 * Return Value : status -
//  223 *                    MD_OK or MD_BUSY1
//  224 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock3 Using cfiCommon0
          CFI Function _R_RTC_Set_CalendarCounterValue
        CODE
//  225 MD_STATUS R_RTC_Set_CalendarCounterValue(rtc_counter_value_t counter_write_val)
//  226 {
_R_RTC_Set_CalendarCounterValue:
        ; * Stack frame (at entry) *
        ; Param size: 8
        ; Auto size: 4
        SUBW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
//  227   MD_STATUS status = MD_OK;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [SP+0x02], AX      ;; 1 cycle
//  228   volatile uint16_t  w_count;
//  229   volatile uint8_t tmp;
//  230   
//  231   /* Only set registers if VRTC enough */
//  232   if (LVDVRTCF == 1)
        MOVW      HL, #0x334         ;; 1 cycle
        MOV1      CY, [HL].6         ;; 1 cycle
        BNC       ??RTC_Set_Clock_Correction_14  ;; 4 cycles
        ; ------------------------------------- Block: 9 cycles
//  233   {
//  234     return MD_ERROR;
        MOVW      AX, #0x80          ;; 1 cycle
        BR        S:??RTC_Set_Clock_Correction_15  ;; 3 cycles
        ; ------------------------------------- Block: 4 cycles
//  235   }
//  236   
//  237   tmp = RCR2;
??RTC_Set_Clock_Correction_14:
        MOV       A, 0x5A5           ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  238   tmp &= (uint8_t)~_01_RTC_COUNTER_NORMAL;
        MOV       A, [SP]            ;; 1 cycle
        AND       A, #0xFE           ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  239   RCR2 = tmp;
        MOV       A, [SP]            ;; 1 cycle
        MOV       0x5A5, A           ;; 1 cycle
//  240   
//  241   if(CheckForSync(&(RCR2),tmp,10) == 0)
        MOV       B, #0xA            ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        MOV       C, A               ;; 1 cycle
        MOVW      AX, #0x5A5         ;; 1 cycle
          CFI FunCall _CheckForSync
        CALL      _CheckForSync      ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??RTC_Set_Clock_Correction_16  ;; 4 cycles
        ; ------------------------------------- Block: 19 cycles
//  242   {
//  243     return MD_ERROR;
        MOVW      AX, #0x80          ;; 1 cycle
        BR        S:??RTC_Set_Clock_Correction_15  ;; 3 cycles
        ; ------------------------------------- Block: 4 cycles
//  244   }
//  245   
//  246   
//  247   if (_01_RTC_COUNTER_NORMAL == (RCR2 & _01_RTC_COUNTER_NORMAL))
??RTC_Set_Clock_Correction_16:
        MOV       A, 0x5A5           ;; 1 cycle
        AND       A, #0x1            ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        BZ        ??RTC_Set_Clock_Correction_17  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  248   {
//  249     status = MD_BUSY1;
        MOVW      AX, #0x3           ;; 1 cycle
        MOVW      [SP+0x02], AX      ;; 1 cycle
        BR        S:??RTC_Set_Clock_Correction_18  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  250   }
//  251   else
//  252   {
//  253     /* Pending, check that the time is not invalid */
//  254     RSECCNT = counter_write_val.sec;
??RTC_Set_Clock_Correction_17:
        MOV       A, [SP+0x08]       ;; 1 cycle
        MOV       0x583, A           ;; 1 cycle
//  255     RMINCNT = counter_write_val.min;
        MOV       A, [SP+0x09]       ;; 1 cycle
        MOV       0x585, A           ;; 1 cycle
//  256     RHRCNT = counter_write_val.hour & 0x3F;;
        MOV       A, [SP+0x0A]       ;; 1 cycle
        AND       A, #0x3F           ;; 1 cycle
        MOV       0x587, A           ;; 1 cycle
//  257     RWKCNT = counter_write_val.week;
        MOV       A, [SP+0x0C]       ;; 1 cycle
        MOV       0x589, A           ;; 1 cycle
//  258     RDAYCNT = counter_write_val.day;
        MOV       A, [SP+0x0B]       ;; 1 cycle
        MOV       0x58B, A           ;; 1 cycle
//  259     RMONCNT = counter_write_val.month;
        MOV       A, [SP+0x0D]       ;; 1 cycle
        MOV       0x58D, A           ;; 1 cycle
//  260     RYRCNT = counter_write_val.year;
        MOV       A, [SP+0x0E]       ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      0x58E, AX          ;; 1 cycle
          CFI FunCall _R_RTC_Start
        ; ------------------------------------- Block: 17 cycles
//  261   }
//  262   if(R_RTC_Start() != 0)
??RTC_Set_Clock_Correction_18:
        CALL      _R_RTC_Start       ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BZ        ??RTC_Set_Clock_Correction_19  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
//  263   {
//  264     return MD_ERROR;
        MOVW      AX, #0x80          ;; 1 cycle
        BR        S:??RTC_Set_Clock_Correction_15  ;; 3 cycles
        ; ------------------------------------- Block: 4 cycles
//  265   }
//  266   
//  267   return (status);
??RTC_Set_Clock_Correction_19:
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??RTC_Set_Clock_Correction_15:
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock3
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 85 cycles
        REQUIRE __A_LVDVRTC
        REQUIRE __A_RCR2
        REQUIRE __A_BCNT0
        REQUIRE __A_BCNT1
        REQUIRE __A_BCNT2
        REQUIRE __A_BCNT3
        REQUIRE __A_RDAYCNT
        REQUIRE __A_RMONCNT
        REQUIRE __A_RYRCNT
//  268 }
//  269 
//  270 
//  271 
//  272 /***********************************************************************************************************************
//  273 * Function Name: R_RTC_Get_CalendarCounterValue
//  274 * Description  : This function reads the results of real-time clock and store them in the variables.
//  275 * Arguments    : counter_read_val -
//  276 *                    the expected real-time clock value(BCD code)
//  277 * Return Value : status -
//  278 *                    MD_OK or MD_ERROR
//  279 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock4 Using cfiCommon0
          CFI Function _R_RTC_Get_CalendarCounterValue
          CFI NoCalls
        CODE
//  280 MD_STATUS R_RTC_Get_CalendarCounterValue(rtc_counter_value_t * const counter_read_val)
//  281 {
_R_RTC_Get_CalendarCounterValue:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 2
        SUBW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+6
        MOVW      DE, AX             ;; 1 cycle
//  282   MD_STATUS status = MD_OK;
        MOVW      HL, #0x0           ;; 1 cycle
//  283   volatile uint8_t tmp;
//  284   
//  285   tmp = RSR;
        MOV       A, 0x5A1           ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  286   tmp &= (uint8_t)~_02_RTC_SECOND_CARRY;
        MOV       A, [SP]            ;; 1 cycle
        AND       A, #0xFD           ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  287   RSR = tmp;
        MOV       A, [SP]            ;; 1 cycle
        MOV       0x5A1, A           ;; 1 cycle
//  288   counter_read_val->sec = RSECCNT;
        MOV       A, 0x583           ;; 1 cycle
        MOV       [DE], A            ;; 1 cycle
//  289   counter_read_val->min = RMINCNT;
        MOV       A, 0x585           ;; 1 cycle
        MOV       [DE+0x01], A       ;; 1 cycle
//  290   counter_read_val->hour = RHRCNT & 0x3F;
        MOV       A, 0x587           ;; 1 cycle
        AND       A, #0x3F           ;; 1 cycle
        MOV       [DE+0x02], A       ;; 1 cycle
//  291   counter_read_val->week = RWKCNT;
        MOV       A, 0x589           ;; 1 cycle
        MOV       [DE+0x04], A       ;; 1 cycle
//  292   counter_read_val->day = RDAYCNT;
        MOV       A, 0x58B           ;; 1 cycle
        MOV       [DE+0x03], A       ;; 1 cycle
//  293   counter_read_val->month = RMONCNT;
        MOV       A, 0x58D           ;; 1 cycle
        MOV       [DE+0x05], A       ;; 1 cycle
//  294   counter_read_val->year = RYRCNT;
        MOVW      AX, 0x58E          ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       [DE+0x06], A       ;; 1 cycle
//  295   if (_02_RTC_SECOND_CARRY == (RSR & _02_RTC_SECOND_CARRY))
        MOV       A, 0x5A1           ;; 1 cycle
        AND       A, #0x2            ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        BZ        ??RTC_Set_Clock_Correction_20  ;; 4 cycles
        ; ------------------------------------- Block: 33 cycles
//  296   {
//  297     status = MD_ERROR;
        MOVW      AX, #0x80          ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
//  298   }
//  299   
//  300   return (status);
??RTC_Set_Clock_Correction_20:
        MOVW      AX, HL             ;; 1 cycle
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock4
        ; ------------------------------------- Block: 8 cycles
        ; ------------------------------------- Total: 43 cycles
        REQUIRE __A_RSR
        REQUIRE __A_BCNT0
        REQUIRE __A_BCNT1
        REQUIRE __A_BCNT2
        REQUIRE __A_BCNT3
        REQUIRE __A_RDAYCNT
        REQUIRE __A_RMONCNT
        REQUIRE __A_RYRCNT
//  301 }
//  302 
//  303 /***********************************************************************************************************************
//  304 * Function Name: R_RTC_Set_CalendarAlarmOn
//  305 * Description  : This function start calendar alarm.
//  306 * Arguments    : enb_set -
//  307 *                    
//  308 * Return Value : None
//  309 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock5 Using cfiCommon0
          CFI Function _R_RTC_Set_CalendarAlarmOn
          CFI NoCalls
        CODE
//  310 void R_RTC_Set_CalendarAlarmOn(int8_t enb_set)
//  311 {
_R_RTC_Set_CalendarAlarmOn:
        ; * Stack frame (at entry) *
        ; Param size: 0
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+6
        ; Auto size: 6
        SUBW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+10
//  312   volatile uint16_t  w_count;
//  313   volatile uint8_t tmp;
//  314   
//  315   RTCAMK = 1U;    /* disable INTRTCALM interrupt */
        SET1      0xFFFE7.3          ;; 2 cycles
//  316   tmp = RCR1;
        MOV       A, 0x5A3           ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  317   tmp &= (uint8_t)~_01_RTC_ALARM_ENABLE;
        MOV       A, [SP]            ;; 1 cycle
        AND       A, #0xFE           ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  318   RCR1 = tmp;
        MOV       A, [SP]            ;; 1 cycle
        MOV       0x5A3, A           ;; 1 cycle
//  319   if ((enb_set & RTC_RSECAR_ENB) == RTC_RSECAR_ENB)
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x5           ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??RTC_Set_Clock_Correction_21  ;; 4 cycles
        ; ------------------------------------- Block: 19 cycles
//  320   {
//  321     RSECAR |= _80_RTC_CALENDER_COUNTER_SEC_COMPARE;
        MOVW      HL, #0x591         ;; 1 cycle
        MOV       A, [HL]            ;; 1 cycle
        OR        A, #0x80           ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
        ; ------------------------------------- Block: 4 cycles
//  322   }
//  323   if ((enb_set & RTC_RMINAR_ENB) == RTC_RMINAR_ENB)
??RTC_Set_Clock_Correction_21:
        MOV       A, [SP+0x05]       ;; 1 cycle
        SHR       A, 0x1             ;; 1 cycle
        AND       A, #0x1            ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        BZ        ??RTC_Set_Clock_Correction_22  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
//  324   {
//  325     RMINAR |= _80_RTC_CALENDER_RMINCNT_MIN_COMPARE;
        MOVW      HL, #0x593         ;; 1 cycle
        MOV       A, [HL]            ;; 1 cycle
        OR        A, #0x80           ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
        ; ------------------------------------- Block: 4 cycles
//  326   }
//  327   if ((enb_set & RTC_RHRAR_ENB) == RTC_RHRAR_ENB)
??RTC_Set_Clock_Correction_22:
        MOV       A, [SP+0x05]       ;; 1 cycle
        SHR       A, 0x2             ;; 1 cycle
        AND       A, #0x1            ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        BZ        ??RTC_Set_Clock_Correction_23  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
//  328   {
//  329     RHRAR |= _80_RTC_CALENDER_RHRCNT_HOUR_COMPARE;
        MOVW      HL, #0x595         ;; 1 cycle
        MOV       A, [HL]            ;; 1 cycle
        OR        A, #0x80           ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
        ; ------------------------------------- Block: 4 cycles
//  330   }
//  331   if ((enb_set & RTC_RWKAR_ENB) == RTC_RWKAR_ENB)
??RTC_Set_Clock_Correction_23:
        MOV       A, [SP+0x05]       ;; 1 cycle
        SHR       A, 0x3             ;; 1 cycle
        AND       A, #0x1            ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        BZ        ??RTC_Set_Clock_Correction_24  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
//  332   {
//  333     RWKAR |= _80_RTC_CALENDER_RWKCNT_WEEK_COMPARE;
        MOVW      HL, #0x597         ;; 1 cycle
        MOV       A, [HL]            ;; 1 cycle
        OR        A, #0x80           ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
        ; ------------------------------------- Block: 4 cycles
//  334   }
//  335   if ((enb_set & RTC_RDAYAR_ENB) == RTC_RDAYAR_ENB)
??RTC_Set_Clock_Correction_24:
        MOV       A, [SP+0x05]       ;; 1 cycle
        SHR       A, 0x4             ;; 1 cycle
        AND       A, #0x1            ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        BZ        ??RTC_Set_Clock_Correction_25  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
//  336   {
//  337     RDAYAR |= _80_RTC_CALENDER_RDAYCNT_DAY_COMPARE;
        MOVW      HL, #0x599         ;; 1 cycle
        MOV       A, [HL]            ;; 1 cycle
        OR        A, #0x80           ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
        ; ------------------------------------- Block: 4 cycles
//  338   }
//  339   if ((enb_set & RTC_RMONAR_ENB) == RTC_RMONAR_ENB)
??RTC_Set_Clock_Correction_25:
        MOV       A, [SP+0x05]       ;; 1 cycle
        SHR       A, 0x5             ;; 1 cycle
        AND       A, #0x1            ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        BZ        ??RTC_Set_Clock_Correction_26  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
//  340   {
//  341     RMONAR |= _80_RTC_CALENDER_RMONCNT_MONTH_COMPARE;
        MOVW      HL, #0x59B         ;; 1 cycle
        MOV       A, [HL]            ;; 1 cycle
        OR        A, #0x80           ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
        ; ------------------------------------- Block: 4 cycles
//  342   }
//  343   if ((enb_set & RTC_RYRAREN_ENB) == RTC_RYRAREN_ENB)
??RTC_Set_Clock_Correction_26:
        MOV       A, [SP+0x05]       ;; 1 cycle
        SHR       A, 0x6             ;; 1 cycle
        AND       A, #0x1            ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        SKZ                          ;; 1 cycle
        ; ------------------------------------- Block: 5 cycles
//  344   {
//  345     RYRAREN |= _80_RTC_CALENDER_RYRCNT_YEAR_COMPARE;
        SET1      0xF059F.7          ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
//  346   }
//  347   
//  348   /* Change the waiting time according to the system */
//  349   for (w_count = 0U; w_count < RTC_WAITTIME_ARSET; w_count++)
??R_RTC_Set_CalendarAlarmOn_0:
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [SP+0x02], AX      ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
??R_RTC_Set_CalendarAlarmOn_1:
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        CMPW      AX, #0xF423        ;; 1 cycle
        BNC       ??RTC_Set_Clock_Correction_27  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  350   {
//  351     NOP();
        NOP                          ;; 1 cycle
//  352   }
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        INCW      AX                 ;; 1 cycle
        MOVW      [SP+0x02], AX      ;; 1 cycle
        BR        S:??R_RTC_Set_CalendarAlarmOn_1  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
//  353   
//  354   tmp = RSR;
??RTC_Set_Clock_Correction_27:
        MOV       A, 0x5A1           ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  355   tmp |= _01_RTC_COUNTER_MATCH;
        MOV       A, [SP]            ;; 1 cycle
        OR        A, #0x1            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  356   RSR = tmp;
        MOV       A, [SP]            ;; 1 cycle
        MOV       0x5A1, A           ;; 1 cycle
//  357   tmp = RCR1;
        MOV       A, 0x5A3           ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  358   tmp |= _01_RTC_ALARM_ENABLE;
        MOV       A, [SP]            ;; 1 cycle
        OR        A, #0x1            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  359   RCR1 = tmp;
        MOV       A, [SP]            ;; 1 cycle
        MOV       0x5A3, A           ;; 1 cycle
//  360 }
        ADDW      SP, #0x6           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock5
        ; ------------------------------------- Block: 21 cycles
        ; ------------------------------------- Total: 126 cycles
        REQUIRE __A_MK1
        REQUIRE __A_RCR1
        REQUIRE __A_BCNT0AR
        REQUIRE __A_BCNT1AR
        REQUIRE __A_BCNT2AR
        REQUIRE __A_BCNT3AR
        REQUIRE __A_BCNT0AER
        REQUIRE __A_BCNT1AER
        REQUIRE __A_BCNT3AER
        REQUIRE __A_RSR
//  361 
//  362 
//  363 
//  364 /***********************************************************************************************************************
//  365 * Function Name: R_RTC_Set_AlarmOff
//  366 * Description  : This function stop alarm.
//  367 * Arguments    : None
//  368 * Return Value : status -
//  369 *                    MD_OK or MD_ERROR
//  370 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock6 Using cfiCommon0
          CFI Function _R_RTC_Set_AlarmOff
          CFI NoCalls
        CODE
//  371 MD_STATUS R_RTC_Set_AlarmOff(void)
//  372 {
_R_RTC_Set_AlarmOff:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 4
        SUBW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
//  373   volatile uint16_t w_count;
//  374   volatile uint8_t tmp;
//  375   MD_STATUS status = MD_OK;
        MOVW      HL, #0x0           ;; 1 cycle
//  376   
//  377   RTCAIF = 0U;    /* clear INTRTCALM interrupt flag */
        CLR1      0xFFFE3.3          ;; 2 cycles
//  378   RTCAMK = 0U;    /* enable INTRTCALM interrupt */
        CLR1      0xFFFE7.3          ;; 2 cycles
//  379   tmp = RCR1;
        MOV       A, 0x5A3           ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  380   tmp &= (uint8_t)~_01_RTC_ALARM_ENABLE;
        MOV       A, [SP]            ;; 1 cycle
        AND       A, #0xFE           ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  381   RCR1 = tmp;
        MOV       A, [SP]            ;; 1 cycle
        MOV       0x5A3, A           ;; 1 cycle
//  382   
//  383   /* Change the waiting time according to the system */
//  384   for (w_count = 0U; w_count < RTC_WAITTIME_AROFF; w_count++)
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [SP+0x02], AX      ;; 1 cycle
        ; ------------------------------------- Block: 15 cycles
??R_RTC_Set_AlarmOff_0:
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        CMPW      AX, #0x3D08        ;; 1 cycle
        BNC       ??RTC_Set_Clock_Correction_28  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  385   {
//  386     NOP();
        NOP                          ;; 1 cycle
//  387   }
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        INCW      AX                 ;; 1 cycle
        MOVW      [SP+0x02], AX      ;; 1 cycle
        BR        S:??R_RTC_Set_AlarmOff_0  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
//  388   
//  389   if (_01_RTC_ALARM_ENABLE == (RCR1 & _01_RTC_ALARM_ENABLE))
??RTC_Set_Clock_Correction_28:
        MOV       A, 0x5A3           ;; 1 cycle
        AND       A, #0x1            ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        BZ        ??RTC_Set_Clock_Correction_29  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  390   {
//  391     status = MD_ERROR;
        MOVW      AX, #0x80          ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
//  392   }
//  393   tmp = RSR;
??RTC_Set_Clock_Correction_29:
        MOV       A, 0x5A1           ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  394   tmp &= (uint8_t)~_01_RTC_COUNTER_MATCH;
        MOV       A, [SP]            ;; 1 cycle
        AND       A, #0xFE           ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  395   RSR = tmp;
        MOV       A, [SP]            ;; 1 cycle
        MOV       0x5A1, A           ;; 1 cycle
//  396   RTCAIF = 0U;    /* clear INTRTCALM interrupt flag */
        CLR1      0xFFFE3.3          ;; 2 cycles
//  397   
//  398   return (status);
        MOVW      AX, HL             ;; 1 cycle
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock6
        ; ------------------------------------- Block: 17 cycles
        ; ------------------------------------- Total: 54 cycles
        REQUIRE __A_IF1
        REQUIRE __A_MK1
        REQUIRE __A_RCR1
        REQUIRE __A_RSR
//  399 }
//  400 
//  401 /***********************************************************************************************************************
//  402 * Function Name: R_RTC_Set_CalendarAlarmValue
//  403 * Description  : This function write calendar alarm value.
//  404 * Arguments    : alarm_val -
//  405 *                    calendar alarm value
//  406 * Return Value : None
//  407 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock7 Using cfiCommon0
          CFI Function _R_RTC_Set_CalendarAlarmValue
          CFI NoCalls
        CODE
//  408 void R_RTC_Set_CalendarAlarmValue(rtc_alarm_value_t alarm_val)
//  409 {  
_R_RTC_Set_CalendarAlarmValue:
        ; * Stack frame (at entry) *
        ; Param size: 8
        ; Auto size: 4
        SUBW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
//  410   volatile uint8_t tmp;
//  411   volatile uint16_t w_count;
//  412   
//  413   tmp = RCR1;
        MOV       A, 0x5A3           ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  414   tmp &= (uint8_t)~_01_RTC_ALARM_ENABLE;
        MOV       A, [SP]            ;; 1 cycle
        AND       A, #0xFE           ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  415   RCR1 = tmp;
        MOV       A, [SP]            ;; 1 cycle
        MOV       0x5A3, A           ;; 1 cycle
//  416   RSECAR = alarm_val.alarmws;
        MOV       A, [SP+0x08]       ;; 1 cycle
        MOV       0x591, A           ;; 1 cycle
//  417   RMINAR = alarm_val.alarmwm;
        MOV       A, [SP+0x09]       ;; 1 cycle
        MOV       0x593, A           ;; 1 cycle
//  418   RHRAR = alarm_val.alarmwh;
        MOV       A, [SP+0x0A]       ;; 1 cycle
        MOV       0x595, A           ;; 1 cycle
//  419   RWKAR = alarm_val.alarmww;
        MOV       A, [SP+0x0B]       ;; 1 cycle
        MOV       0x597, A           ;; 1 cycle
//  420   RDAYAR = alarm_val.alarmwd;
        MOV       A, [SP+0x0C]       ;; 1 cycle
        MOV       0x599, A           ;; 1 cycle
//  421   RMONAR = alarm_val.alarmwmt;
        MOV       A, [SP+0x0D]       ;; 1 cycle
        MOV       0x59B, A           ;; 1 cycle
//  422   RYRAR = alarm_val.alarmwy;
        MOVW      AX, [SP+0x0E]      ;; 1 cycle
        MOVW      0x59C, AX          ;; 1 cycle
//  423   
//  424   /* Change the waiting time according to the system */
//  425   for (w_count = 0U; w_count < RTC_WAITTIME_ARSET; w_count++)
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [SP+0x02], AX      ;; 1 cycle
        ; ------------------------------------- Block: 24 cycles
??R_RTC_Set_CalendarAlarmValue_0:
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        CMPW      AX, #0xF423        ;; 1 cycle
        BNC       ??RTC_Set_Clock_Correction_30  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  426   {
//  427     NOP();
        NOP                          ;; 1 cycle
//  428   }
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        INCW      AX                 ;; 1 cycle
        MOVW      [SP+0x02], AX      ;; 1 cycle
        BR        S:??R_RTC_Set_CalendarAlarmValue_0  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
//  429   
//  430   tmp = RSR;
??RTC_Set_Clock_Correction_30:
        MOV       A, 0x5A1           ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  431   tmp &= (uint8_t)~_01_RTC_COUNTER_MATCH;
        MOV       A, [SP]            ;; 1 cycle
        AND       A, #0xFE           ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  432   RSR = tmp;
        MOV       A, [SP]            ;; 1 cycle
        MOV       0x5A1, A           ;; 1 cycle
//  433   tmp = RCR1;
        MOV       A, 0x5A3           ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  434   tmp |= _01_RTC_ALARM_ENABLE;
        MOV       A, [SP]            ;; 1 cycle
        OR        A, #0x1            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  435   RCR1 = tmp;
        MOV       A, [SP]            ;; 1 cycle
        MOV       0x5A3, A           ;; 1 cycle
//  436 }
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock7
        ; ------------------------------------- Block: 21 cycles
        ; ------------------------------------- Total: 58 cycles
        REQUIRE __A_RCR1
        REQUIRE __A_BCNT0AR
        REQUIRE __A_BCNT1AR
        REQUIRE __A_BCNT2AR
        REQUIRE __A_BCNT3AR
        REQUIRE __A_BCNT0AER
        REQUIRE __A_BCNT1AER
        REQUIRE __A_BCNT2AER
        REQUIRE __A_RSR
//  437 
//  438 
//  439 /***********************************************************************************************************************
//  440 * Function Name: R_RTC_Get_CalendarAlarmValue
//  441 * Description  : This function resd alarm value.
//  442 * Arguments    : alarm_val -
//  443 *                    calendar alarm value
//  444 * Return Value : None
//  445 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock8 Using cfiCommon0
          CFI Function _R_RTC_Get_CalendarAlarmValue
          CFI NoCalls
        CODE
//  446 void R_RTC_Get_CalendarAlarmValue(rtc_alarm_value_t * const alarm_val)
//  447 {
_R_RTC_Get_CalendarAlarmValue:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 2
        SUBW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+6
        MOVW      HL, AX             ;; 1 cycle
//  448   volatile uint8_t tmp;
//  449   
//  450   tmp = RCR1;	
        MOV       A, 0x5A3           ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  451   tmp &= (uint8_t)~_01_RTC_ALARM_ENABLE;
        MOV       A, [SP]            ;; 1 cycle
        AND       A, #0xFE           ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  452   RCR1 = tmp;
        MOV       A, [SP]            ;; 1 cycle
        MOV       0x5A3, A           ;; 1 cycle
//  453   alarm_val->alarmws = RSECAR;
        MOV       A, 0x591           ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
//  454   alarm_val->alarmwm = RMINAR;
        MOV       A, 0x593           ;; 1 cycle
        MOV       [HL+0x01], A       ;; 1 cycle
//  455   alarm_val->alarmwh = RHRAR;
        MOV       A, 0x595           ;; 1 cycle
        MOV       [HL+0x02], A       ;; 1 cycle
//  456   alarm_val->alarmww = RWKAR;
        MOV       A, 0x597           ;; 1 cycle
        MOV       [HL+0x03], A       ;; 1 cycle
//  457   alarm_val->alarmwd = RDAYAR;
        MOV       A, 0x599           ;; 1 cycle
        MOV       [HL+0x04], A       ;; 1 cycle
//  458   alarm_val->alarmwmt = RMONAR;
        MOV       A, 0x59B           ;; 1 cycle
        MOV       [HL+0x05], A       ;; 1 cycle
//  459   alarm_val->alarmwy = RYRAR;
        MOVW      AX, 0x59C          ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  460   tmp = RCR1;	
        MOV       A, 0x5A3           ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  461   tmp |= _01_RTC_ALARM_ENABLE;
        MOV       A, [SP]            ;; 1 cycle
        OR        A, #0x1            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  462   RCR1 = tmp;
        MOV       A, [SP]            ;; 1 cycle
        MOV       0x5A3, A           ;; 1 cycle
//  463 }
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock8
        ; ------------------------------------- Block: 37 cycles
        ; ------------------------------------- Total: 37 cycles
        REQUIRE __A_RCR1
        REQUIRE __A_BCNT0AR
        REQUIRE __A_BCNT1AR
        REQUIRE __A_BCNT2AR
        REQUIRE __A_BCNT3AR
        REQUIRE __A_BCNT0AER
        REQUIRE __A_BCNT1AER
        REQUIRE __A_BCNT2AER
//  464 
//  465 /***********************************************************************************************************************
//  466 * Function Name: R_RTC_Set_ConstPeriodInterruptOn
//  467 * Description  : This function enables constant-period interrupt.
//  468 * Arguments    : period -
//  469 *                    the constant period of INTRTCRPD
//  470 * Return Value : status -
//  471 *                    MD_OK or MD_ARGERROR
//  472 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock9 Using cfiCommon0
          CFI Function _R_RTC_Set_ConstPeriodInterruptOn
          CFI NoCalls
        CODE
//  473 MD_STATUS R_RTC_Set_ConstPeriodInterruptOn(rtc_int_period_t period)
//  474 {
_R_RTC_Set_ConstPeriodInterruptOn:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
        MOV       B, A               ;; 1 cycle
//  475   MD_STATUS status = MD_OK;
        MOVW      HL, #0x0           ;; 1 cycle
//  476   
//  477   if ((period < SEC1_256) || (period > SEC2S))
        MOV       A, B               ;; 1 cycle
        CMP       A, #0x60           ;; 1 cycle
        BC        ??RTC_Set_Clock_Correction_31  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
        MOV       A, B               ;; 1 cycle
        CMP       A, #0xF1           ;; 1 cycle
        BC        ??RTC_Set_Clock_Correction_32  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  478   {
//  479     status = MD_ARGERROR;
??RTC_Set_Clock_Correction_31:
        MOVW      AX, #0x81          ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        BR        S:??RTC_Set_Clock_Correction_33  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  480   }
//  481   else
//  482   {
//  483     RTCRMK = 1U;    /* disable INTRTCRPD interrupt */
??RTC_Set_Clock_Correction_32:
        SET1      0xFFFE7.3          ;; 2 cycles
//  484     RCR1 = (RCR1 & _09_RTC_CLEAR_PERIOD) | period | _04_RTC_PERIODIC_ENABLE;
        MOV       A, 0x5A3           ;; 1 cycle
        AND       A, #0x9            ;; 1 cycle
        OR        A, B               ;; 1 cycle
        OR        A, #0x4            ;; 1 cycle
        MOV       0x5A3, A           ;; 1 cycle
//  485     RTCRIF = 0U;    /* clear INTRTCRPD interrupt flag */
        CLR1      0xFFFE3.3          ;; 2 cycles
//  486     RTCRMK = 0U;    /* enable INTRTCRPD interrupt */
        CLR1      0xFFFE7.3          ;; 2 cycles
        ; ------------------------------------- Block: 11 cycles
//  487   }
//  488   
//  489   return (status);
??RTC_Set_Clock_Correction_33:
        MOVW      AX, HL             ;; 1 cycle
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock9
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 37 cycles
        REQUIRE __A_MK1
        REQUIRE __A_RCR1
        REQUIRE __A_IF1
//  490 }
//  491 
//  492 /***********************************************************************************************************************
//  493 * Function Name: R_RTC_Set_ConstPeriodInterruptOff
//  494 * Description  : This function disables constant-period interrupt.
//  495 * Arguments    : None
//  496 * Return Value : None
//  497 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock10 Using cfiCommon0
          CFI Function _R_RTC_Set_ConstPeriodInterruptOff
          CFI NoCalls
        CODE
//  498 void R_RTC_Set_ConstPeriodInterruptOff(void)
//  499 {
_R_RTC_Set_ConstPeriodInterruptOff:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  500   RCR1 = (RCR1 & (uint8_t)~_04_RTC_PERIODIC_ENABLE);
        CLR1      0xF05A3.2          ;; 2 cycles
//  501   RTCRIF = 0U;    /* clear INTRTCRPD interrupt flag */
        CLR1      0xFFFE3.3          ;; 2 cycles
//  502 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock10
        ; ------------------------------------- Block: 10 cycles
        ; ------------------------------------- Total: 10 cycles
        REQUIRE __A_RCR1
        REQUIRE __A_IF1
//  503 
//  504 /***********************************************************************************************************************
//  505 * Function Name: R_RTC_Set_RTCOUTOn
//  506 * Description  : This function set RTCOUT on.
//  507 * Arguments    : None
//  508 * Return Value : None
//  509 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock11 Using cfiCommon0
          CFI Function _R_RTC_Set_RTCOUTOn
          CFI NoCalls
        CODE
//  510 void R_RTC_Set_RTCOUTOn(void)
//  511 {
_R_RTC_Set_RTCOUTOn:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 2
        SUBW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+6
//  512   volatile uint8_t tmp;
//  513   
//  514   tmp = RCR2;	
        MOV       A, 0x5A5           ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  515   tmp |= _08_RTC_RTCOUT_ENABLE;
        MOV       A, [SP]            ;; 1 cycle
        OR        A, #0x8            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  516   RCR2 = tmp;	
        MOV       A, [SP]            ;; 1 cycle
        MOV       0x5A5, A           ;; 1 cycle
//  517 }
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock11
        ; ------------------------------------- Block: 15 cycles
        ; ------------------------------------- Total: 15 cycles
        REQUIRE __A_RCR2
//  518 
//  519 /***********************************************************************************************************************
//  520 * Function Name: R_RTC_Set_RTCOUTOff
//  521 * Description  : This function set RTCOUT off.
//  522 * Arguments    : None
//  523 * Return Value : None
//  524 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock12 Using cfiCommon0
          CFI Function _R_RTC_Set_RTCOUTOff
          CFI NoCalls
        CODE
//  525 void R_RTC_Set_RTCOUTOff(void)
//  526 {
_R_RTC_Set_RTCOUTOff:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 2
        SUBW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+6
//  527   volatile uint8_t tmp;
//  528   
//  529   tmp = RCR2;	
        MOV       A, 0x5A5           ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  530   tmp &= (uint8_t)~_08_RTC_RTCOUT_ENABLE;
        MOV       A, [SP]            ;; 1 cycle
        AND       A, #0xF7           ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  531   RCR2 = tmp;	
        MOV       A, [SP]            ;; 1 cycle
        MOV       0x5A5, A           ;; 1 cycle
//  532 }
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock12
        ; ------------------------------------- Block: 15 cycles
        ; ------------------------------------- Total: 15 cycles
        REQUIRE __A_RCR2
//  533 
//  534 /***********************************************************************************************************************
//  535 * Function Name: R_RTC_Set_PowerOff
//  536 * Description  : This function stops the clock supplied for real-time clock.
//  537 * Arguments    : None
//  538 * Return Value : None
//  539 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock13 Using cfiCommon0
          CFI Function _R_RTC_Set_PowerOff
          CFI NoCalls
        CODE
//  540 void R_RTC_Set_PowerOff(void)
//  541 {
_R_RTC_Set_PowerOff:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  542   VRTCEN = 0U;    /* stops input clock supply */
        CLR1      0xF00FC.0          ;; 2 cycles
//  543 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock13
        ; ------------------------------------- Block: 8 cycles
        ; ------------------------------------- Total: 8 cycles
        REQUIRE __A_PER2
//  544 
//  545 /* Start user code for adding. Do not edit comment generated here */

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock14 Using cfiCommon1
          CFI Function _CheckForSync
        CODE
//  546 uint8_t CheckForSync(volatile uint8_t *ptr, uint8_t val, uint8_t cntr)
//  547 {
_CheckForSync:
        ; * Stack frame (at entry) *
        ; Param size: 0
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+6
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+8
        ; Auto size: 6
        SUBW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+10
//  548   uint8_t i;
//  549   
//  550   for(i=0;i<cntr;i++)
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        ; ------------------------------------- Block: 5 cycles
??CheckForSync_0:
        MOV       A, [SP]            ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        CMP       X, A               ;; 1 cycle
        BNC       ??RTC_Set_Clock_Correction_34  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
//  551   {
//  552     if(*ptr != val)
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        CMP       A, [HL]            ;; 1 cycle
        BZ        ??RTC_Set_Clock_Correction_34  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
//  553     {
//  554       /* Wait for setting value sync to Register */
//  555       delay_ms(1);
        MOVW      AX, #0x1           ;; 1 cycle
          CFI FunCall _delay_ms
        CALL      _delay_ms          ;; 3 cycles
//  556     }
        MOV       A, [SP]            ;; 1 cycle
        INC       A                  ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        BR        S:??CheckForSync_0  ;; 3 cycles
        ; ------------------------------------- Block: 10 cycles
//  557     else
//  558     {
//  559       break;
//  560     }
//  561   }
//  562   if(i>=cntr)
??RTC_Set_Clock_Correction_34:
        MOV       A, [SP]            ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        CMP       X, A               ;; 1 cycle
        BC        ??RTC_Set_Clock_Correction_35  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
//  563   {
//  564     __no_operation();
        NOP                          ;; 1 cycle
//  565     return 0;
        MOV       A, #0x0            ;; 1 cycle
        BR        S:??RTC_Set_Clock_Correction_36  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  566   }
//  567   else
//  568   {
//  569     __no_operation();
??RTC_Set_Clock_Correction_35:
        NOP                          ;; 1 cycle
//  570     return 1;
        MOV       A, #0x1            ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
??RTC_Set_Clock_Correction_36:
        ADDW      SP, #0x6           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock14
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 53 cycles
//  571   }
//  572 }

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock15 Using cfiCommon0
          CFI Function _RTC_1Hz_Output_Set
        CODE
//  573 uint8_t RTC_1Hz_Output_Set(us8 choice)
//  574 {
_RTC_1Hz_Output_Set:
        ; * Stack frame (at entry) *
        ; Param size: 0
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+6
        ; Auto size: 4
        SUBW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+8
//  575   uint8_t tmp,stat;
//  576   
//  577   /* Stop all counters : START is bit 0 */
//  578   tmp = RCR2;  
        MOV       A, 0x5A5           ;; 1 cycle
//  579   tmp &= (~BIT0);
        AND       A, #0xFE           ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  580   RCR2 = tmp;
        MOV       A, [SP]            ;; 1 cycle
        MOV       0x5A5, A           ;; 1 cycle
//  581   
//  582   stat = CheckForSync(&(RCR2),tmp,10);
        MOV       B, #0xA            ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        MOV       C, A               ;; 1 cycle
        MOVW      AX, #0x5A5         ;; 1 cycle
          CFI FunCall _CheckForSync
        CALL      _CheckForSync      ;; 3 cycles
        MOV       [SP+0x01], A       ;; 1 cycle
//  583   if(stat==0)
        MOV       A, [SP+0x01]       ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        BNZ       ??RTC_Set_Clock_Correction_37  ;; 4 cycles
        ; ------------------------------------- Block: 21 cycles
//  584     return 9;
        MOV       A, #0x9            ;; 1 cycle
        BR        S:??RTC_Set_Clock_Correction_38  ;; 3 cycles
        ; ------------------------------------- Block: 4 cycles
//  585   
//  586   tmp = RCR2;
??RTC_Set_Clock_Correction_37:
        MOV       X, 0x5A5           ;; 1 cycle
//  587   
//  588   if(choice==1)  // Enable
        MOV       A, [SP+0x03]       ;; 1 cycle
        CMP       A, #0x1            ;; 1 cycle
        BNZ       ??RTC_Set_Clock_Correction_39  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  589   {
//  590     /* Writing 0x00 value in adjustment register by default */
//  591     RADJ = 0x00;
        MOV       0x5AF, #0x0        ;; 1 cycle
//  592     
//  593     tmp |= _08_RTC_RTCOUT_ENABLE;
        MOV       A, X               ;; 1 cycle
        OR        A, #0x8            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  594     PM6 &= ~(RTCOUT);
        CLR1      0xFFF26.2          ;; 2 cycles
        BR        S:??RTC_Set_Clock_Correction_40  ;; 3 cycles
        ; ------------------------------------- Block: 9 cycles
//  595     //P6 &= ~(RTCOUT);
//  596   }
//  597   else          // Disable
//  598   {
//  599     tmp &= ~(_08_RTC_RTCOUT_ENABLE);
??RTC_Set_Clock_Correction_39:
        MOV       A, X               ;; 1 cycle
        AND       A, #0xF7           ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  600     PM6 |= (RTCOUT);
        SET1      0xFFF26.2          ;; 2 cycles
        ; ------------------------------------- Block: 5 cycles
//  601   }
//  602   
//  603   RCR2 = tmp;
??RTC_Set_Clock_Correction_40:
        MOV       A, [SP]            ;; 1 cycle
        MOV       0x5A5, A           ;; 1 cycle
//  604   
//  605   stat = CheckForSync(&(RCR2),tmp,10);
        MOV       B, #0xA            ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        MOV       C, A               ;; 1 cycle
        MOVW      AX, #0x5A5         ;; 1 cycle
          CFI FunCall _CheckForSync
        CALL      _CheckForSync      ;; 3 cycles
        MOV       [SP+0x01], A       ;; 1 cycle
//  606   if(stat==0)
        MOV       A, [SP+0x01]       ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        BNZ       ??RTC_Set_Clock_Correction_41  ;; 4 cycles
        ; ------------------------------------- Block: 16 cycles
//  607     return 10;
        MOV       A, #0xA            ;; 1 cycle
        BR        S:??RTC_Set_Clock_Correction_38  ;; 3 cycles
        ; ------------------------------------- Block: 4 cycles
//  608   
//  609   /* Start the RTC once  */
//  610   tmp = RCR2;  
??RTC_Set_Clock_Correction_41:
        MOV       A, 0x5A5           ;; 1 cycle
//  611   tmp |= BIT0;
        OR        A, #0x1            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  612   RCR2 = tmp;
        MOV       A, [SP]            ;; 1 cycle
        MOV       0x5A5, A           ;; 1 cycle
//  613   
//  614   /* 1 will be return if no error and 0 is return if register value not updated in 10 ms */
//  615   stat = CheckForSync(&(RCR2),tmp,10);
        MOV       B, #0xA            ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        MOV       C, A               ;; 1 cycle
        MOVW      AX, #0x5A5         ;; 1 cycle
          CFI FunCall _CheckForSync
        CALL      _CheckForSync      ;; 3 cycles
        MOV       X, A               ;; 1 cycle
//  616   if(stat==0)
        CMP0      X                  ;; 1 cycle
        BNZ       ??RTC_Set_Clock_Correction_42  ;; 4 cycles
        ; ------------------------------------- Block: 18 cycles
//  617     return 11;
        MOV       A, #0xB            ;; 1 cycle
        BR        S:??RTC_Set_Clock_Correction_38  ;; 3 cycles
        ; ------------------------------------- Block: 4 cycles
//  618   
//  619   return 0;
??RTC_Set_Clock_Correction_42:
        MOV       A, #0x0            ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??RTC_Set_Clock_Correction_38:
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock15
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 96 cycles
        REQUIRE __A_RCR2
        REQUIRE __A_RADJ
        REQUIRE __A_PM6
//  620 }
//  621 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock16 Using cfiCommon0
          CFI Function _RTC_Set_Clock_Correction
        CODE
//  622 void RTC_Set_Clock_Correction(us8 correction)
//  623 {
_RTC_Set_Clock_Correction:
        ; * Stack frame (at entry) *
        ; Param size: 0
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+6
        ; Auto size: 2
//  624   if(RADJ != correction)
        MOV       X, 0x5AF           ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        CMP       X, A               ;; 1 cycle
        BZ        ??RTC_Set_Clock_Correction_43  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
//  625   {	
//  626     /* Writing Correction value for 10 sec period */
//  627     RADJ = correction;
        MOV       A, [SP+0x01]       ;; 1 cycle
        MOV       0x5AF, A           ;; 1 cycle
//  628     delay_ms(10);
        MOVW      AX, #0xA           ;; 1 cycle
          CFI FunCall _delay_ms
        CALL      _delay_ms          ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
//  629   }
//  630 }
??RTC_Set_Clock_Correction_43:
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock16
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 21 cycles
        REQUIRE __A_RADJ

        SECTION `.iar_vfe_header`:DATA:NOALLOC:NOROOT(1)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
        DC32 0

        END
// 
//    36 bytes in section .bss.noinit  (abs)
// 1'358 bytes in section .text
// 
// 1'358 bytes of CODE memory
//     0 bytes of DATA memory (+ 36 bytes shared)
//
//Errors: none
//Warnings: none
