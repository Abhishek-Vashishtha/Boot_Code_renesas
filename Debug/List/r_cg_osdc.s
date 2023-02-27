///////////////////////////////////////////////////////////////////////////////
//
// IAR C/C++ Compiler V4.20.1.2260 for RL78               20/Dec/2020  22:38:55
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
//        BootCode\source_code\driver_files\r_cg_osdc.c
//    Command line       =
//        -f C:\Users\laptop\AppData\Local\Temp\EWE9EB.tmp ("D:\Dheeraj\New
//        folder\0. GDEV72 - BootCode\source_code\driver_files\r_cg_osdc.c"
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
//        D:\Dheeraj\New folder\0. GDEV72 - BootCode\Debug\List\r_cg_osdc.s
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

        PUBLIC _R_OSDC_Create
        PUBLIC _R_OSDC_Reset
        PUBLIC _R_OSDC_Set_PowerOff
        PUBLIC _R_OSDC_Start
        PUBLIC _R_OSDC_Stop
        PUBLIC __A_IF2
        PUBLIC __A_MK2
        PUBLIC __A_OSDC
        PUBLIC __A_PER2
        PUBLIC __A_PR02
        PUBLIC __A_PR12
        PUBLIC __A_PRR2
        
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
        
// D:\Dheeraj\New folder\0. GDEV72 - BootCode\source_code\driver_files\r_cg_osdc.c
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
//   21 * File Name    : r_cg_osdc.c
//   22 * Version      : Applilet4 for RL78/I1C V1.01.03.02 [16 Nov 2018]
//   23 * Device(s)    : R5F10NPJ
//   24 * Tool-Chain   : IAR Systems icc78k0r
//   25 * Description  : This file implements device driver for OSDC module.
//   26 * Creation Date: 16-04-2020
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

        ASEGN `.bss.noinit`:DATA:NOROOT,0f02d0H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_OSDC
// __no_init union <unnamed>#509 volatile __no_bit_access _A_OSDC
__A_OSDC:
        DS 2
//   33 #include "r_cg_osdc.h"
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
//   51 * Function Name: R_OSDC_Create
//   52 * Description  : This function initializes the OSDC module.
//   53 * Arguments    : None
//   54 * Return Value : None
//   55 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock0 Using cfiCommon0
          CFI Function _R_OSDC_Create
          CFI NoCalls
        CODE
//   56 void R_OSDC_Create(void)
//   57 {
_R_OSDC_Create:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//   58     OSDCRES = 1U;   /* reset oscillation stop detection circuit */
        SET1      0xF00FD.6          ;; 2 cycles
//   59     OSDCRES = 0U;   /* reset release of oscillation stop detection circuit */
        CLR1      0xF00FD.6          ;; 2 cycles
//   60     OSDCEN = 1U;    /* enables input clock supply */
        SET1      0xF00FC.6          ;; 2 cycles
//   61     OSDC = _0000_OSDC_OPERATION_DISABLE;    /* disable OSDC operation */
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      0x2D0, AX          ;; 1 cycle
//   62     OSDMK = 1U;     /* disable INTOSDC interrupt */
        SET1      0xFFFD5.6          ;; 2 cycles
//   63     OSDIF = 0U;     /* clear INTOSDC interrupt flag */
        CLR1      0xFFFD1.6          ;; 2 cycles
//   64     /* Set INTOSDC level 1 priority */
//   65     OSDPR1 = 0U;
        CLR1      0xFFFDD.6          ;; 2 cycles
//   66     OSDPR0 = 1U;
        SET1      0xFFFD9.6          ;; 2 cycles
//   67     OSDC = _05DB_OSDCCMP_VALUE;
        MOVW      AX, #0x5DB         ;; 1 cycle
        MOVW      0x2D0, AX          ;; 1 cycle
//   68 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock0
        ; ------------------------------------- Block: 24 cycles
        ; ------------------------------------- Total: 24 cycles
        REQUIRE __A_PRR2
        REQUIRE __A_PER2
        REQUIRE __A_OSDC
        REQUIRE __A_MK2
        REQUIRE __A_IF2
        REQUIRE __A_PR12
        REQUIRE __A_PR02
//   69 /***********************************************************************************************************************
//   70 * Function Name: R_OSDC_Start
//   71 * Description  : This function starts OSDC module operation.
//   72 * Arguments    : None
//   73 * Return Value : None
//   74 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock1 Using cfiCommon0
          CFI Function _R_OSDC_Start
          CFI NoCalls
        CODE
//   75 void R_OSDC_Start(void)
//   76 {
_R_OSDC_Start:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//   77     OSDIF = 0U;     /* clear INTOSDC interrupt flag */
        CLR1      0xFFFD1.6          ;; 2 cycles
//   78     OSDMK = 0U;     /* enable INTOSDC interrupt */
        CLR1      0xFFFD5.6          ;; 2 cycles
//   79     OSDC |= _8000_OSDC_OPERATION_ENABLE;
        MOVW      AX, 0x2D0          ;; 1 cycle
        OR        A, #0x80           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        OR        A, #0x0            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      0x2D0, AX          ;; 1 cycle
//   80 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock1
        ; ------------------------------------- Block: 16 cycles
        ; ------------------------------------- Total: 16 cycles
        REQUIRE __A_IF2
        REQUIRE __A_MK2
        REQUIRE __A_OSDC
//   81 /***********************************************************************************************************************
//   82 * Function Name: R_OSDC_Stop
//   83 * Description  : This function stops OSDC module operation.
//   84 * Arguments    : None
//   85 * Return Value : None
//   86 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock2 Using cfiCommon0
          CFI Function _R_OSDC_Stop
          CFI NoCalls
        CODE
//   87 void R_OSDC_Stop(void)
//   88 {
_R_OSDC_Stop:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//   89     OSDMK = 1U;     /* disable INTOSDC interrupt */
        SET1      0xFFFD5.6          ;; 2 cycles
//   90     OSDIF = 0U;     /* clear INTOSDC interrupt flag */
        CLR1      0xFFFD1.6          ;; 2 cycles
//   91     OSDC &= (uint16_t)~_8000_OSDC_OPERATION_ENABLE;
        MOVW      AX, 0x2D0          ;; 1 cycle
        AND       A, #0x7F           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        AND       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      0x2D0, AX          ;; 1 cycle
//   92 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock2
        ; ------------------------------------- Block: 16 cycles
        ; ------------------------------------- Total: 16 cycles
        REQUIRE __A_MK2
        REQUIRE __A_IF2
        REQUIRE __A_OSDC
//   93 /***********************************************************************************************************************
//   94 * Function Name: R_OSDC_Set_PowerOff
//   95 * Description  : This function stops the clock supplied for OSDC.
//   96 * Arguments    : None
//   97 * Return Value : None
//   98 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock3 Using cfiCommon0
          CFI Function _R_OSDC_Set_PowerOff
          CFI NoCalls
        CODE
//   99 void R_OSDC_Set_PowerOff(void)
//  100 {
_R_OSDC_Set_PowerOff:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  101     OSDCEN = 0U;    /* stops input clock supply */
        CLR1      0xF00FC.6          ;; 2 cycles
//  102 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock3
        ; ------------------------------------- Block: 8 cycles
        ; ------------------------------------- Total: 8 cycles
        REQUIRE __A_PER2
//  103 /***********************************************************************************************************************
//  104 * Function Name: R_OSDC_Reset
//  105 * Description  : This function resets OSDC module.
//  106 * Arguments    : None
//  107 * Return Value : None
//  108 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock4 Using cfiCommon0
          CFI Function _R_OSDC_Reset
          CFI NoCalls
        CODE
//  109 void R_OSDC_Reset(void)
//  110 {
_R_OSDC_Reset:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  111     OSDCRES = 1U;   /* reset oscillation stop detection circuit */
        SET1      0xF00FD.6          ;; 2 cycles
//  112 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock4
        ; ------------------------------------- Block: 8 cycles
        ; ------------------------------------- Total: 8 cycles
        REQUIRE __A_PRR2

        SECTION `.iar_vfe_header`:DATA:NOALLOC:NOROOT(1)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
        DC32 0

        END
//  113 
//  114 /* Start user code for adding. Do not edit comment generated here */
//  115 /* End user code. Do not edit comment generated here */
// 
// 12 bytes in section .bss.noinit  (abs)
// 85 bytes in section .text
// 
// 85 bytes of CODE memory
//  0 bytes of DATA memory (+ 12 bytes shared)
//
//Errors: none
//Warnings: none
