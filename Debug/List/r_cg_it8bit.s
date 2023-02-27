///////////////////////////////////////////////////////////////////////////////
//
// IAR C/C++ Compiler V4.20.1.2260 for RL78               21/Dec/2020  00:35:16
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
//        BootCode\source_code\driver_files\r_cg_it8bit.c
//    Command line       =
//        -f C:\Users\laptop\AppData\Local\Temp\EW7AD5.tmp ("D:\Dheeraj\New
//        folder\0. GDEV72 - BootCode\source_code\driver_files\r_cg_it8bit.c"
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
//        D:\Dheeraj\New folder\0. GDEV72 - BootCode\Debug\List\r_cg_it8bit.s
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

        PUBLIC _R_IT8Bit0_Channel0_Create
        PUBLIC _R_IT8Bit0_Channel0_Start
        PUBLIC _R_IT8Bit0_Channel0_Stop
        PUBLIC _R_IT8Bit0_Channel1_Create
        PUBLIC _R_IT8Bit0_Channel1_Start
        PUBLIC _R_IT8Bit0_Channel1_Stop
        PUBLIC _R_IT8Bit0_Set_PowerOff
        PUBLIC _R_IT8Bit1_Channel0_Create
        PUBLIC _R_IT8Bit1_Channel0_Start
        PUBLIC _R_IT8Bit1_Channel0_Stop
        PUBLIC _R_IT8Bit1_Channel1_Create
        PUBLIC _R_IT8Bit1_Channel1_Start
        PUBLIC _R_IT8Bit1_Channel1_Stop
        PUBLIC _R_IT8Bit1_Set_PowerOff
        PUBLIC __A_IF2
        PUBLIC __A_IF3
        PUBLIC __A_MK2
        PUBLIC __A_MK3
        PUBLIC __A_PR02
        PUBLIC __A_PR03
        PUBLIC __A_PR12
        PUBLIC __A_PR13
        PUBLIC __A_TRTCMP0
        PUBLIC __A_TRTCMP1
        PUBLIC __A_TRTCR0
        PUBLIC __A_TRTCR1
        PUBLIC __A_TRTMD0
        PUBLIC __A_TRTMD1
        
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
        
// D:\Dheeraj\New folder\0. GDEV72 - BootCode\source_code\driver_files\r_cg_it8bit.c
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
//   21 * File Name    : r_cg_it8bit.c
//   22 * Version      : Applilet4 for RL78/I1C V1.01.02.04 [24 May 2018]
//   23 * Device(s)    : R5F10NPJ
//   24 * Tool-Chain   : IAR Systems icc78k0r
//   25 * Description  : This file implements device driver for IT8Bit module.
//   26 * Creation Date: 12/17/2019
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

        ASEGN `.bss.noinit`:DATA:NOROOT,0fffdaH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_PR03
// __no_init union <unnamed>#138 volatile __sfr _A_PR03
__A_PR03:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0fffdcH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_PR12
// __no_init union <unnamed>#142 volatile __sfr _A_PR12
__A_PR12:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0fffdeH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_PR13
// __no_init union <unnamed>#148 volatile __sfr _A_PR13
__A_PR13:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0350H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_TRTCMP0
// __no_init union <unnamed>#552 volatile __no_bit_access _A_TRTCMP0
__A_TRTCMP0:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0352H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_TRTCR0
// __no_init union <unnamed>#556 volatile _A_TRTCR0
__A_TRTCR0:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0353H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_TRTMD0
// __no_init union <unnamed>#558 volatile __no_bit_access _A_TRTMD0
__A_TRTMD0:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0358H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_TRTCMP1
// __no_init union <unnamed>#559 volatile __no_bit_access _A_TRTCMP1
__A_TRTCMP1:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f035aH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_TRTCR1
// __no_init union <unnamed>#563 volatile _A_TRTCR1
__A_TRTCR1:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f035bH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_TRTMD1
// __no_init union <unnamed>#565 volatile __no_bit_access _A_TRTMD1
__A_TRTMD1:
        DS 1
//   33 #include "r_cg_it8bit.h"
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
//   51 * Function Name: R_IT8Bit0_Set_PowerOff
//   52 * Description  : This function stops the clock supplied for 8 bit interval timer unit0.
//   53 * Arguments    : None
//   54 * Return Value : None
//   55 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock0 Using cfiCommon0
          CFI Function _R_IT8Bit0_Set_PowerOff
          CFI NoCalls
        CODE
//   56 void R_IT8Bit0_Set_PowerOff(void)
//   57 {
_R_IT8Bit0_Set_PowerOff:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//   58     /* stop 8 bit IT unit0 clock */
//   59     TRTCR0 &= (uint8_t)~_10_IT8BIT_CLOCK_SUPPLY;
        CLR1      0xF0352.4          ;; 2 cycles
//   60 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock0
        ; ------------------------------------- Block: 8 cycles
        ; ------------------------------------- Total: 8 cycles
        REQUIRE __A_TRTCR0
//   61 /***********************************************************************************************************************
//   62 * Function Name: R_IT8Bit0_Channel0_Create
//   63 * Description  : This function initializes the 8 bit interval timer unit0 channel0 ||2ms||.
//   64 * Arguments    : None
//   65 * Return Value : None
//   66 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock1 Using cfiCommon0
          CFI Function _R_IT8Bit0_Channel0_Create
          CFI NoCalls
        CODE
//   67 void R_IT8Bit0_Channel0_Create(void)
//   68 {
_R_IT8Bit0_Channel0_Create:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//   69     TRTCR0 |= _10_IT8BIT_CLOCK_SUPPLY;
        SET1      0xF0352.4          ;; 2 cycles
//   70     TSTART00 = 0U;  /* counting stops */
        CLR1      0xF0352.0          ;; 2 cycles
//   71     ITMK00 = 1U;    /* disable INTIT00 interrupt */
        SET1      0xFFFD5.2          ;; 2 cycles
//   72     ITIF00 = 0U;    /* clear INTIT00 interrupt flag */
        CLR1      0xFFFD1.2          ;; 2 cycles
//   73     /* Set INTIT00 high priority */
//   74     ITPR100 = 0U;
        CLR1      0xFFFDD.2          ;; 2 cycles
//   75     ITPR000 = 0U;
        CLR1      0xFFFD9.2          ;; 2 cycles
//   76     TRTCR0 |= _00_IT8BIT_8BIT_COUNT_MODE;
        MOV       A, 0x352           ;; 1 cycle
        MOV       0x352, A           ;; 1 cycle
//   77     TRTMD0 |= _01_IT8BIT_CLOCK0_2;
        MOVW      HL, #0x353         ;; 1 cycle
        MOV       A, [HL]            ;; 1 cycle
        OR        A, #0x1            ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
//   78     TRTCMP00 = _20_IT8BIT_CMP00_VALUE;
        MOV       0x350, #0x20       ;; 1 cycle
//   79 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock1
        ; ------------------------------------- Block: 25 cycles
        ; ------------------------------------- Total: 25 cycles
        REQUIRE __A_TRTCR0
        REQUIRE __A_MK2
        REQUIRE __A_IF2
        REQUIRE __A_PR12
        REQUIRE __A_PR02
        REQUIRE __A_TRTMD0
        REQUIRE __A_TRTCMP0
//   80 /***********************************************************************************************************************
//   81 * Function Name: R_IT8Bit0_Channel0_Start
//   82 * Description  : This function starts 8 bit interval timer unit0 Channel0 operation.
//   83 * Arguments    : None
//   84 * Return Value : None
//   85 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock2 Using cfiCommon0
          CFI Function _R_IT8Bit0_Channel0_Start
          CFI NoCalls
        CODE
//   86 void R_IT8Bit0_Channel0_Start(void)
//   87 {
_R_IT8Bit0_Channel0_Start:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//   88     ITIF00 = 0U;    /* clear INTIT00 interrupt flag */
        CLR1      0xFFFD1.2          ;; 2 cycles
//   89     ITMK00 = 0U;    /* enable INTIT00 interrupt */
        CLR1      0xFFFD5.2          ;; 2 cycles
//   90     TSTART00 = 1U;  /* counting starts */
        SET1      0xF0352.0          ;; 2 cycles
//   91 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock2
        ; ------------------------------------- Block: 12 cycles
        ; ------------------------------------- Total: 12 cycles
        REQUIRE __A_IF2
        REQUIRE __A_MK2
        REQUIRE __A_TRTCR0
//   92 /***********************************************************************************************************************
//   93 * Function Name: R_IT8Bit0_Channel0_Stop
//   94 * Description  : This function stops 8 bit interval timer unit0 Channel0 operation.
//   95 * Arguments    : None
//   96 * Return Value : None
//   97 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock3 Using cfiCommon0
          CFI Function _R_IT8Bit0_Channel0_Stop
          CFI NoCalls
        CODE
//   98 void R_IT8Bit0_Channel0_Stop(void)
//   99 {
_R_IT8Bit0_Channel0_Stop:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  100     ITMK00 = 1U;    /* disable INTIT00 interrupt */
        SET1      0xFFFD5.2          ;; 2 cycles
//  101     ITIF00 = 0U;    /* clear INTIT00 interrupt flag */
        CLR1      0xFFFD1.2          ;; 2 cycles
//  102     TSTART00 = 0U;  /* counting stops */
        CLR1      0xF0352.0          ;; 2 cycles
//  103 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock3
        ; ------------------------------------- Block: 12 cycles
        ; ------------------------------------- Total: 12 cycles
        REQUIRE __A_MK2
        REQUIRE __A_IF2
        REQUIRE __A_TRTCR0
//  104 /***********************************************************************************************************************
//  105 * Function Name: R_IT8Bit0_Channel1_Create
//  106 * Description  : This function initializes the 8 bit interval timer unit0 channel1.   ||10ms||
//  107 * Arguments    : None
//  108 * Return Value : None
//  109 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock4 Using cfiCommon0
          CFI Function _R_IT8Bit0_Channel1_Create
          CFI NoCalls
        CODE
//  110 void R_IT8Bit0_Channel1_Create(void)
//  111 {
_R_IT8Bit0_Channel1_Create:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  112     TRTCR0 |= _10_IT8BIT_CLOCK_SUPPLY;
        SET1      0xF0352.4          ;; 2 cycles
//  113     TSTART01 = 0U;  /* counting stops */
        CLR1      0xF0352.2          ;; 2 cycles
//  114     ITMK01 = 1U;    /* disable INTIT01 interrupt */
        SET1      0xFFFD5.3          ;; 2 cycles
//  115     ITIF01 = 0U;    /* clear INTIT01 interrupt flag */
        CLR1      0xFFFD1.3          ;; 2 cycles
//  116     /* Set INTIT01 low priority */
//  117     ITPR101 = 1U;
        SET1      0xFFFDD.3          ;; 2 cycles
//  118     ITPR001 = 1U;
        SET1      0xFFFD9.3          ;; 2 cycles
//  119     TRTCR0 |= _00_IT8BIT_8BIT_COUNT_MODE;
        MOV       A, 0x352           ;; 1 cycle
        MOV       0x352, A           ;; 1 cycle
//  120     TRTMD0 |= _10_IT8BIT_CLOCK1_2;
        MOVW      HL, #0x353         ;; 1 cycle
        MOV       A, [HL]            ;; 1 cycle
        OR        A, #0x10           ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
//  121     TRTCMP01 = _A3_IT8BIT_CMP01_VALUE;
        MOVW      HL, #0x351         ;; 1 cycle
        MOV       A, #0xA3           ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
//  122 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock4
        ; ------------------------------------- Block: 27 cycles
        ; ------------------------------------- Total: 27 cycles
        REQUIRE __A_TRTCR0
        REQUIRE __A_MK2
        REQUIRE __A_IF2
        REQUIRE __A_PR12
        REQUIRE __A_PR02
        REQUIRE __A_TRTMD0
        REQUIRE __A_TRTCMP0
//  123 /***********************************************************************************************************************
//  124 * Function Name: R_IT8Bit0_Channel1_Start
//  125 * Description  : This function starts 8 bit interval timer unit0 Channel1 operation.
//  126 * Arguments    : None
//  127 * Return Value : None
//  128 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock5 Using cfiCommon0
          CFI Function _R_IT8Bit0_Channel1_Start
          CFI NoCalls
        CODE
//  129 void R_IT8Bit0_Channel1_Start(void)
//  130 {
_R_IT8Bit0_Channel1_Start:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  131     ITIF01 = 0U;    /* clear INTIT01 interrupt flag */
        CLR1      0xFFFD1.3          ;; 2 cycles
//  132     ITMK01 = 0U;    /* enable INTIT01 interrupt */
        CLR1      0xFFFD5.3          ;; 2 cycles
//  133     TSTART01 = 1U;  /* counting starts */
        SET1      0xF0352.2          ;; 2 cycles
//  134 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock5
        ; ------------------------------------- Block: 12 cycles
        ; ------------------------------------- Total: 12 cycles
        REQUIRE __A_IF2
        REQUIRE __A_MK2
        REQUIRE __A_TRTCR0
//  135 /***********************************************************************************************************************
//  136 * Function Name: R_IT8Bit0_Channel1_Stop
//  137 * Description  : This function stops 8 bit interval timer unit0 Channel1 operation.
//  138 * Arguments    : None
//  139 * Return Value : None
//  140 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock6 Using cfiCommon0
          CFI Function _R_IT8Bit0_Channel1_Stop
          CFI NoCalls
        CODE
//  141 void R_IT8Bit0_Channel1_Stop(void)
//  142 {
_R_IT8Bit0_Channel1_Stop:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  143     ITMK01 = 1U;    /* disable INTIT01 interrupt */
        SET1      0xFFFD5.3          ;; 2 cycles
//  144     ITIF01 = 0U;    /* clear INTIT01 interrupt flag */
        CLR1      0xFFFD1.3          ;; 2 cycles
//  145     TSTART01 = 0U;  /* counting stops */
        CLR1      0xF0352.2          ;; 2 cycles
//  146 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock6
        ; ------------------------------------- Block: 12 cycles
        ; ------------------------------------- Total: 12 cycles
        REQUIRE __A_MK2
        REQUIRE __A_IF2
        REQUIRE __A_TRTCR0
//  147 /***********************************************************************************************************************
//  148 * Function Name: R_IT8Bit1_Set_PowerOff
//  149 * Description  : This function stops the clock supplied for 8 bit interval timer unit1.
//  150 * Arguments    : None
//  151 * Return Value : None
//  152 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock7 Using cfiCommon0
          CFI Function _R_IT8Bit1_Set_PowerOff
          CFI NoCalls
        CODE
//  153 void R_IT8Bit1_Set_PowerOff(void)
//  154 {
_R_IT8Bit1_Set_PowerOff:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  155     /* stop 8 bit IT unit1 clock */
//  156     TRTCR1 &= (uint8_t)~_10_IT8BIT_CLOCK_SUPPLY;
        CLR1      0xF035A.4          ;; 2 cycles
//  157 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock7
        ; ------------------------------------- Block: 8 cycles
        ; ------------------------------------- Total: 8 cycles
        REQUIRE __A_TRTCR1
//  158 /***********************************************************************************************************************
//  159 * Function Name: R_IT8Bit1_Channel0_Create
//  160 * Description  : This function initializes the 8 bit interval timer unit1 channel0.  ||1000ms||
//  161 * Arguments    : None
//  162 * Return Value : None
//  163 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock8 Using cfiCommon0
          CFI Function _R_IT8Bit1_Channel0_Create
          CFI NoCalls
        CODE
//  164 void R_IT8Bit1_Channel0_Create(void)
//  165 {
_R_IT8Bit1_Channel0_Create:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  166     TRTCR1 |= _10_IT8BIT_CLOCK_SUPPLY;
        SET1      0xF035A.4          ;; 2 cycles
//  167     TSTART10 = 0U;  /* counting stops */
        CLR1      0xF035A.0          ;; 2 cycles
//  168     ITMK10 = 1U;    /* disable INTIT10 interrupt */
        SET1      0xFFFD6.2          ;; 2 cycles
//  169     ITIF10 = 0U;    /* clear INTIT10 interrupt flag */
        CLR1      0xFFFD2.2          ;; 2 cycles
//  170     /* Set INTIT10 low priority */
//  171     ITPR110 = 1U;
        SET1      0xFFFDE.2          ;; 2 cycles
//  172     ITPR010 = 1U;
        SET1      0xFFFDA.2          ;; 2 cycles
//  173     TRTCR1 |= _00_IT8BIT_8BIT_COUNT_MODE;
        MOV       A, 0x35A           ;; 1 cycle
        MOV       0x35A, A           ;; 1 cycle
//  174     TRTMD1 |= _07_IT8BIT_CLOCK0_128;
        MOVW      HL, #0x35B         ;; 1 cycle
        MOV       A, [HL]            ;; 1 cycle
        OR        A, #0x7            ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
//  175     TRTCMP10 = _FF_IT8BIT_CMP10_VALUE;
        MOV       0x358, #0xFF       ;; 1 cycle
//  176 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock8
        ; ------------------------------------- Block: 25 cycles
        ; ------------------------------------- Total: 25 cycles
        REQUIRE __A_TRTCR1
        REQUIRE __A_MK3
        REQUIRE __A_IF3
        REQUIRE __A_PR13
        REQUIRE __A_PR03
        REQUIRE __A_TRTMD1
        REQUIRE __A_TRTCMP1
//  177 /***********************************************************************************************************************
//  178 * Function Name: R_IT8Bit1_Channel0_Start
//  179 * Description  : This function starts 8 bit interval timer unit1 Channel0 operation.
//  180 * Arguments    : None
//  181 * Return Value : None
//  182 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock9 Using cfiCommon0
          CFI Function _R_IT8Bit1_Channel0_Start
          CFI NoCalls
        CODE
//  183 void R_IT8Bit1_Channel0_Start(void)
//  184 {
_R_IT8Bit1_Channel0_Start:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  185     ITIF10 = 0U;    /* clear INTIT10 interrupt flag */
        CLR1      0xFFFD2.2          ;; 2 cycles
//  186     ITMK10 = 0U;    /* enable INTIT10 interrupt */
        CLR1      0xFFFD6.2          ;; 2 cycles
//  187     TSTART10 = 1U;  /* counting starts */
        SET1      0xF035A.0          ;; 2 cycles
//  188 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock9
        ; ------------------------------------- Block: 12 cycles
        ; ------------------------------------- Total: 12 cycles
        REQUIRE __A_IF3
        REQUIRE __A_MK3
        REQUIRE __A_TRTCR1
//  189 /***********************************************************************************************************************
//  190 * Function Name: R_IT8Bit1_Channel0_Stop
//  191 * Description  : This function stops 8 bit interval timer unit1 Channel0 operation.
//  192 * Arguments    : None
//  193 * Return Value : None
//  194 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock10 Using cfiCommon0
          CFI Function _R_IT8Bit1_Channel0_Stop
          CFI NoCalls
        CODE
//  195 void R_IT8Bit1_Channel0_Stop(void)
//  196 {
_R_IT8Bit1_Channel0_Stop:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  197     ITMK10 = 1U;    /* disable INTIT10 interrupt */
        SET1      0xFFFD6.2          ;; 2 cycles
//  198     ITIF10 = 0U;    /* clear INTIT10 interrupt flag */
        CLR1      0xFFFD2.2          ;; 2 cycles
//  199     TSTART10 = 0U;  /* counting stops */
        CLR1      0xF035A.0          ;; 2 cycles
//  200 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock10
        ; ------------------------------------- Block: 12 cycles
        ; ------------------------------------- Total: 12 cycles
        REQUIRE __A_MK3
        REQUIRE __A_IF3
        REQUIRE __A_TRTCR1
//  201 /***********************************************************************************************************************
//  202 * Function Name: R_IT8Bit1_Channel1_Create
//  203 * Description  : This function initializes the 8 bit interval timer unit1 channel1.
//  204 * Arguments    : None
//  205 * Return Value : None
//  206 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock11 Using cfiCommon0
          CFI Function _R_IT8Bit1_Channel1_Create
          CFI NoCalls
        CODE
//  207 void R_IT8Bit1_Channel1_Create(void)
//  208 {
_R_IT8Bit1_Channel1_Create:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  209     
//  210 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock11
        ; ------------------------------------- Block: 6 cycles
        ; ------------------------------------- Total: 6 cycles
//  211 /***********************************************************************************************************************
//  212 * Function Name: R_IT8Bit1_Channel1_Start
//  213 * Description  : This function starts 8 bit interval timer unit1 Channel1 operation.
//  214 * Arguments    : None
//  215 * Return Value : None
//  216 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock12 Using cfiCommon0
          CFI Function _R_IT8Bit1_Channel1_Start
          CFI NoCalls
        CODE
//  217 void R_IT8Bit1_Channel1_Start(void)
//  218 {
_R_IT8Bit1_Channel1_Start:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  219 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock12
        ; ------------------------------------- Block: 6 cycles
        ; ------------------------------------- Total: 6 cycles
//  220 /***********************************************************************************************************************
//  221 * Function Name: R_IT8Bit1_Channel1_Stop
//  222 * Description  : This function stops 8 bit interval timer unit1 Channel1 operation.
//  223 * Arguments    : None
//  224 * Return Value : None
//  225 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock13 Using cfiCommon0
          CFI Function _R_IT8Bit1_Channel1_Stop
          CFI NoCalls
        CODE
//  226 void R_IT8Bit1_Channel1_Stop(void)
//  227 {
_R_IT8Bit1_Channel1_Stop:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  228 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock13
        ; ------------------------------------- Block: 6 cycles
        ; ------------------------------------- Total: 6 cycles

        SECTION `.iar_vfe_header`:DATA:NOALLOC:NOROOT(1)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
        DC32 0

        END
//  229 
//  230 /* Start user code for adding. Do not edit comment generated here */
//  231 /* End user code. Do not edit comment generated here */
// 
//  24 bytes in section .bss.noinit  (abs)
// 195 bytes in section .text
// 
// 195 bytes of CODE memory
//   0 bytes of DATA memory (+ 24 bytes shared)
//
//Errors: none
//Warnings: none
