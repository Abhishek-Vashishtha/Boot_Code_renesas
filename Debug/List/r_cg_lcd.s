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
//        BootCode\source_code\driver_files\r_cg_lcd.c
//    Command line       =
//        -f C:\Users\17054\AppData\Local\Temp\EWDBB5.tmp ("D:\Software
//        code\Projects Software\Projects new\0. Non AMI\SUGAM Based\0. GDEV72
//        - BootCode-20210104T103109Z-001\0. GDEV72 -
//        BootCode\source_code\driver_files\r_cg_lcd.c" --core s3 --code_model
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
//        BootCode\Debug\List\r_cg_lcd.s
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

        EXTERN _R_WDT_Restart
        EXTERN _delay_ms

        PUBLIC _R_LCD_Create
        PUBLIC _R_LCD_Start
        PUBLIC _R_LCD_Stop
        PUBLIC _R_LCD_Voltage_Off
        PUBLIC _R_LCD_Voltage_On
        PUBLIC __A_ISCLCD
        PUBLIC __A_LCDC0
        PUBLIC __A_LCDM0
        PUBLIC __A_LCDM1
        PUBLIC __A_P1
        PUBLIC __A_P12
        PUBLIC __A_P3
        PUBLIC __A_P7
        PUBLIC __A_P8
        PUBLIC __A_PFSEG0
        PUBLIC __A_PFSEG1
        PUBLIC __A_PFSEG2
        PUBLIC __A_PFSEG3
        PUBLIC __A_PIM1
        PUBLIC __A_PIM8
        PUBLIC __A_PM1
        PUBLIC __A_PM12
        PUBLIC __A_PM3
        PUBLIC __A_PM7
        PUBLIC __A_PM8
        PUBLIC __A_POM1
        PUBLIC __A_POM8
        PUBLIC __A_PU1
        PUBLIC __A_PU3
        PUBLIC __A_PU7
        PUBLIC __A_PU8
        PUBLIC __A_VLCD
        
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
        
// D:\Software code\Projects Software\Projects new\0. Non AMI\SUGAM Based\0. GDEV72 - BootCode-20210104T103109Z-001\0. GDEV72 - BootCode\source_code\driver_files\r_cg_lcd.c
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
//   21 * File Name    : r_cg_lcd.c
//   22 * Version      : Applilet4 for RL78/I1C V1.01.03.02 [16 Nov 2018]
//   23 * Device(s)    : R5F10NPJ
//   24 * Tool-Chain   : IAR Systems icc78k0r
//   25 * Description  : This file implements device driver for LCD module.
//   26 * Creation Date: 14-05-2020
//   27 ***********************************************************************************************************************/
//   28 
//   29 /***********************************************************************************************************************
//   30 Includes
//   31 ***********************************************************************************************************************/
//   32 #include "r_cg_macrodriver.h"

        ASEGN `.sbss.noinit`:DATA:NOROOT,0fff01H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_P1
// __no_init union <unnamed>#4 volatile __saddr _A_P1
__A_P1:
        DS 1

        ASEGN `.sbss.noinit`:DATA:NOROOT,0fff03H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_P3
// __no_init union <unnamed>#6 volatile __saddr _A_P3
__A_P3:
        DS 1

        ASEGN `.sbss.noinit`:DATA:NOROOT,0fff07H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_P7
// __no_init union <unnamed>#10 volatile __saddr _A_P7
__A_P7:
        DS 1

        ASEGN `.sbss.noinit`:DATA:NOROOT,0fff08H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_P8
// __no_init union <unnamed>#11 volatile __saddr _A_P8
__A_P8:
        DS 1

        ASEGN `.sbss.noinit`:DATA:NOROOT,0fff0cH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_P12
// __no_init union <unnamed>#12 volatile __saddr _A_P12
__A_P12:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0fff21H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_PM1
// __no_init union <unnamed>#36 volatile __sfr _A_PM1
__A_PM1:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0fff23H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_PM3
// __no_init union <unnamed>#38 volatile __sfr _A_PM3
__A_PM3:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0fff27H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_PM7
// __no_init union <unnamed>#42 volatile __sfr _A_PM7
__A_PM7:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0fff28H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_PM8
// __no_init union <unnamed>#43 volatile __sfr _A_PM8
__A_PM8:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0fff2cH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_PM12
// __no_init union <unnamed>#44 volatile __sfr _A_PM12
__A_PM12:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0fff40H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_LCDM0
// __no_init union <unnamed>#60 volatile __sfr __no_bit_access _A_LCDM0
__A_LCDM0:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0fff41H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_LCDM1
// __no_init union <unnamed>#61 volatile __sfr _A_LCDM1
__A_LCDM1:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0fff42H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_LCDC0
// __no_init union <unnamed>#63 volatile __sfr __no_bit_access _A_LCDC0
__A_LCDC0:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0fff43H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_VLCD
// __no_init union <unnamed>#64 volatile __sfr __no_bit_access _A_VLCD
__A_VLCD:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0031H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_PU1
// __no_init union <unnamed>#234 volatile _A_PU1
__A_PU1:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0033H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_PU3
// __no_init union <unnamed>#235 volatile _A_PU3
__A_PU3:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0037H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_PU7
// __no_init union <unnamed>#238 volatile _A_PU7
__A_PU7:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0038H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_PU8
// __no_init union <unnamed>#239 volatile _A_PU8
__A_PU8:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0041H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_PIM1
// __no_init union <unnamed>#243 volatile _A_PIM1
__A_PIM1:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0048H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_PIM8
// __no_init union <unnamed>#245 volatile _A_PIM8
__A_PIM8:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0051H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_POM1
// __no_init union <unnamed>#247 volatile _A_POM1
__A_POM1:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0058H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_POM8
// __no_init union <unnamed>#249 volatile _A_POM8
__A_POM8:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0300H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_PFSEG0
// __no_init union <unnamed>#529 volatile _A_PFSEG0
__A_PFSEG0:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0301H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_PFSEG1
// __no_init union <unnamed>#530 volatile _A_PFSEG1
__A_PFSEG1:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0302H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_PFSEG2
// __no_init union <unnamed>#531 volatile _A_PFSEG2
__A_PFSEG2:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0303H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_PFSEG3
// __no_init union <unnamed>#532 volatile _A_PFSEG3
__A_PFSEG3:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0308H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_ISCLCD
// __no_init union <unnamed>#535 volatile _A_ISCLCD
__A_ISCLCD:
        DS 1
//   33 #include "r_cg_lcd.h"
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
//   46 /* Start user code for global. Do not edit comment generated here */
//   47 /* End user code. Do not edit comment generated here */
//   48 
//   49 /***********************************************************************************************************************
//   50 * Function Name: R_LCD_Create
//   51 * Description  : This function initializes the LCD module.
//   52 * Arguments    : None
//   53 * Return Value : None
//   54 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock0 Using cfiCommon0
          CFI Function _R_LCD_Create
        CODE
//   55 void R_LCD_Create(void)
//   56 {
_R_LCD_Create:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//   57     volatile uint32_t wt_count;
//   58 
//   59     LCDON = 0U;     /* display off (all segment outputs are deselected) */
        CLR1      0xFFF41.7          ;; 2 cycles
//   60     LCDM1 |= _00_LCD_VOLTAGE_HIGH; /* pending, check for the use of this with BLA sir*/
        MOV       A, 0xFFF41         ;; 1 cycle
        MOV       0xFFF41, A         ;; 1 cycle
//   61     LCDM0 = _00_LCD_DISPLAY_WAVEFORM_A | _14_LCD_TIMESLICE_8 | _01_LCD_BIAS_MODE_3;
        MOV       0xFFF40, #0x15     ;; 1 cycle
//   62     LCDM0 |= _40_LCD_VOLTAGE_MODE_INTERNAL;
        MOV       A, 0xFFF40         ;; 1 cycle
        OR        A, #0x40           ;; 1 cycle
        MOV       0xFFF40, A         ;; 1 cycle
//   63     /* Set CAPL and CAPH pins */
//   64     ISCLCD &= (uint8_t)~_01_LCD_CAPLH_BUFFER_VALID;
        CLR1      0xF0308.0          ;; 2 cycles
//   65     P12 &= 0x3FU;
        AND       S:0xFFF0C, #0x3F   ;; 2 cycles
//   66     PM12 |= 0xC0U;
        MOV       A, 0xFFF2C         ;; 1 cycle
        OR        A, #0xC0           ;; 1 cycle
        MOV       0xFFF2C, A         ;; 1 cycle
//   67     /* Set VL3 pin */
//   68     ISCLCD &= (uint8_t)~_02_LCD_VL3_BUFFER_VALID;
        CLR1      0xF0308.1          ;; 2 cycles
//   69     P12 &= 0xDFU;
        CLR1      S:0xFFF0C.5        ;; 2 cycles
//   70     PM12 |= 0x20U;
        SET1      0xFFF2C.5          ;; 2 cycles
//   71     /* Set segment pins */
//   72     PIM1 &= 0x9FU;
        MOVW      HL, #0x41          ;; 1 cycle
        MOV       A, [HL]            ;; 1 cycle
        AND       A, #0x9F           ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
//   73     PIM8 &= 0xFCU;
        MOVW      HL, #0x48          ;; 1 cycle
        MOV       A, [HL]            ;; 1 cycle
        AND       A, #0xFC           ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
//   74     POM1 &= 0x1FU;
        MOVW      HL, #0x51          ;; 1 cycle
        MOV       A, [HL]            ;; 1 cycle
        AND       A, #0x1F           ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
//   75     POM8 &= 0xF8U;    
        MOVW      HL, #0x58          ;; 1 cycle
        MOV       A, [HL]            ;; 1 cycle
        AND       A, #0xF8           ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
//   76     PFSEG0 |= 0xF0U;
        MOVW      HL, #0x300         ;; 1 cycle
        MOV       A, [HL]            ;; 1 cycle
        OR        A, #0xF0           ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
//   77     PFSEG1 |= 0xFFU;
        MOV       A, 0x301           ;; 1 cycle
        MOV       0x301, #0xFF       ;; 1 cycle
//   78     PFSEG2 |= 0xFFU;
        MOV       A, 0x302           ;; 1 cycle
        MOV       0x302, #0xFF       ;; 1 cycle
//   79     PFSEG3 |= 0x3FU;
        MOVW      HL, #0x303         ;; 1 cycle
        MOV       A, [HL]            ;; 1 cycle
        OR        A, #0x3F           ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
//   80     PU1 &= 0x00U;
        MOV       A, 0x31            ;; 1 cycle
        MOV       0x31, #0x0         ;; 1 cycle
//   81     PU3 &= 0xC0U;
        MOVW      HL, #0x33          ;; 1 cycle
        MOV       A, [HL]            ;; 1 cycle
        AND       A, #0xC0           ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
//   82     PU7 &= 0x00U;
        MOV       A, 0x37            ;; 1 cycle
        MOV       0x37, #0x0         ;; 1 cycle
//   83     PU8 &= 0xF0U;
        MOVW      HL, #0x38          ;; 1 cycle
        MOV       A, [HL]            ;; 1 cycle
        AND       A, #0xF0           ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
//   84     P1 &= 0x00U;
        MOV       A, S:0xFFF01       ;; 1 cycle
        MOV       S:0xFFF01, #0x0    ;; 1 cycle
//   85     PM1 &= 0x00U;
        MOV       A, 0xFFF21         ;; 1 cycle
        MOV       0xFFF21, #0x0      ;; 1 cycle
//   86     P3 &= 0xC0U;
        AND       S:0xFFF03, #0xC0   ;; 2 cycles
//   87     PM3 &= 0xC0U;    
        MOV       A, 0xFFF23         ;; 1 cycle
        AND       A, #0xC0           ;; 1 cycle
        MOV       0xFFF23, A         ;; 1 cycle
//   88     P7 &= 0x00U;
        MOV       A, S:0xFFF07       ;; 1 cycle
        MOV       S:0xFFF07, #0x0    ;; 1 cycle
//   89     PM7 &= 0x00U;
        MOV       A, 0xFFF27         ;; 1 cycle
        MOV       0xFFF27, #0x0      ;; 1 cycle
//   90     P8 &= 0xF0U;
        AND       S:0xFFF08, #0xF0   ;; 2 cycles
//   91     PM8 &= 0xF0U;
        MOV       A, 0xFFF28         ;; 1 cycle
        AND       A, #0xF0           ;; 1 cycle
        MOV       0xFFF28, A         ;; 1 cycle
//   92 
//   93     LCDC0 = _05_LCD_CLOCK_FSX_FIL_6;
        MOV       0xFFF42, #0x5      ;; 1 cycle
//   94     VLCD = _06_LCD_BOOST_VOLTAGE_110V;    
        MOV       0xFFF43, #0x6      ;; 1 cycle
//   95     /* Change the waiting time according to the system */
//   96     delay_ms(5);
        MOVW      AX, #0x5           ;; 1 cycle
          CFI FunCall _delay_ms
        CALL      _delay_ms          ;; 3 cycles
//   97 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock0
        ; ------------------------------------- Block: 91 cycles
        ; ------------------------------------- Total: 91 cycles
        REQUIRE __A_LCDM1
        REQUIRE __A_LCDM0
        REQUIRE __A_ISCLCD
        REQUIRE __A_P12
        REQUIRE __A_PM12
        REQUIRE __A_PIM1
        REQUIRE __A_PIM8
        REQUIRE __A_POM1
        REQUIRE __A_POM8
        REQUIRE __A_PFSEG0
        REQUIRE __A_PFSEG1
        REQUIRE __A_PFSEG2
        REQUIRE __A_PFSEG3
        REQUIRE __A_PU1
        REQUIRE __A_PU3
        REQUIRE __A_PU7
        REQUIRE __A_PU8
        REQUIRE __A_P1
        REQUIRE __A_PM1
        REQUIRE __A_P3
        REQUIRE __A_PM3
        REQUIRE __A_P7
        REQUIRE __A_PM7
        REQUIRE __A_P8
        REQUIRE __A_PM8
        REQUIRE __A_LCDC0
        REQUIRE __A_VLCD
//   98 /***********************************************************************************************************************
//   99 * Function Name: R_LCD_Start
//  100 * Description  : This function enables the LCD display.
//  101 * Arguments    : None
//  102 * Return Value : None
//  103 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock1 Using cfiCommon0
          CFI Function _R_LCD_Start
          CFI NoCalls
        CODE
//  104 void R_LCD_Start(void)
//  105 {
_R_LCD_Start:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  106     LCDON = 1U;     /* display on */
        SET1      0xFFF41.7          ;; 2 cycles
//  107 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock1
        ; ------------------------------------- Block: 8 cycles
        ; ------------------------------------- Total: 8 cycles
        REQUIRE __A_LCDM1
//  108 /***********************************************************************************************************************
//  109 * Function Name: R_LCD_Stop
//  110 * Description  : This function disables the LCD display.
//  111 * Arguments    : None
//  112 * Return Value : None
//  113 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock2 Using cfiCommon0
          CFI Function _R_LCD_Stop
          CFI NoCalls
        CODE
//  114 void R_LCD_Stop(void)
//  115 {
_R_LCD_Stop:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  116     LCDON = 0U;     /* display off (all segment outputs are deselected) */
        CLR1      0xFFF41.7          ;; 2 cycles
//  117 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock2
        ; ------------------------------------- Block: 8 cycles
        ; ------------------------------------- Total: 8 cycles
        REQUIRE __A_LCDM1
//  118 /***********************************************************************************************************************
//  119 * Function Name: R_LCD_Voltage_On
//  120 * Description  : This function enables voltage boost circuit or capacitor split circuit.
//  121 * Arguments    : None
//  122 * Return Value : None
//  123 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock3 Using cfiCommon0
          CFI Function _R_LCD_Voltage_On
        CODE
//  124 void R_LCD_Voltage_On(void)
//  125 {
_R_LCD_Voltage_On:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  126     VLCON = 1U;     /* enables voltage boost and capacitor split operation */
        SET1      0xFFF41.5          ;; 2 cycles
//  127 
//  128     wdt_restart();
          CFI FunCall _R_WDT_Restart
        CALL      _R_WDT_Restart     ;; 3 cycles
//  129     delay_ms(500);
        MOVW      AX, #0x1F4         ;; 1 cycle
          CFI FunCall _delay_ms
        CALL      _delay_ms          ;; 3 cycles
//  130     wdt_restart();
          CFI FunCall _R_WDT_Restart
        CALL      _R_WDT_Restart     ;; 3 cycles
//  131     
//  132     SCOC = 1U;      /* select common and segment pins output */
        SET1      0xFFF41.6          ;; 2 cycles
//  133 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock3
        ; ------------------------------------- Block: 20 cycles
        ; ------------------------------------- Total: 20 cycles
        REQUIRE __A_LCDM1
//  134 /***********************************************************************************************************************
//  135 * Function Name: R_LCD_Voltage_Off
//  136 * Description  : This function disables voltage boost circuit or capacitor split circuit.
//  137 * Arguments    : None
//  138 * Return Value : None
//  139 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock4 Using cfiCommon0
          CFI Function _R_LCD_Voltage_Off
          CFI NoCalls
        CODE
//  140 void R_LCD_Voltage_Off(void)
//  141 {
_R_LCD_Voltage_Off:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  142     SCOC = 0U;      /* output ground level to segment/common pin */
        CLR1      0xFFF41.6          ;; 2 cycles
//  143     VLCON = 0U;     /* stops voltage boost and capacitor split operation */
        CLR1      0xFFF41.5          ;; 2 cycles
//  144     LCDM0 &= (uint8_t)~(_C0_LCD_VOLTAGE_MODE_INITIALVALUE);
        MOV       A, 0xFFF40         ;; 1 cycle
        AND       A, #0x3F           ;; 1 cycle
        MOV       0xFFF40, A         ;; 1 cycle
//  145 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock4
        ; ------------------------------------- Block: 13 cycles
        ; ------------------------------------- Total: 13 cycles
        REQUIRE __A_LCDM1
        REQUIRE __A_LCDM0

        SECTION `.iar_vfe_header`:DATA:NOALLOC:NOROOT(1)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
        DC32 0

        END
//  146 
//  147 /* Start user code for adding. Do not edit comment generated here */
//  148 /* End user code. Do not edit comment generated here */
// 
//  22 bytes in section .bss.noinit   (abs)
//   5 bytes in section .sbss.noinit  (abs)
// 214 bytes in section .text
// 
// 214 bytes of CODE memory
//   0 bytes of DATA memory (+ 27 bytes shared)
//
//Errors: none
//Warnings: none
