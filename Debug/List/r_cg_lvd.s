///////////////////////////////////////////////////////////////////////////////
//
// IAR C/C++ Compiler V4.21.1.2409/W32 for RL78           27/Feb/2023  16:59:20
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
//        BootCode\source_code\driver_files\r_cg_lvd.c
//    Command line       =
//        -f C:\Users\17054\AppData\Local\Temp\EWDBB4.tmp ("D:\Software
//        code\Projects Software\Projects new\0. Non AMI\SUGAM Based\0. GDEV72
//        - BootCode-20210104T103109Z-001\0. GDEV72 -
//        BootCode\source_code\driver_files\r_cg_lvd.c" --core s3 --code_model
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
//        BootCode\Debug\List\r_cg_lvd.s
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

        EXTERN _R_LVD_Create_UserInit

        PUBLIC _R_LVD_Create
        PUBLIC _R_LVD_InterruptMode_Start
        PUBLIC _R_LVD_InterruptMode_Stop
        PUBLIC _R_LVD_Start_EXLVD
        PUBLIC _R_LVD_Start_VBAT
        PUBLIC _R_LVD_Start_VDD
        PUBLIC _R_LVD_Start_VRTC
        PUBLIC _R_LVD_Stop_EXLVD
        PUBLIC _R_LVD_Stop_VBAT
        PUBLIC _R_LVD_Stop_VDD
        PUBLIC _R_LVD_Stop_VRTC
        PUBLIC __A_IF0
        PUBLIC __A_IF3
        PUBLIC __A_LVDEXLVD
        PUBLIC __A_LVDVBAT
        PUBLIC __A_LVDVDD
        PUBLIC __A_LVDVRTC
        PUBLIC __A_MK0
        PUBLIC __A_MK3
        PUBLIC __A_PR03
        PUBLIC __A_PR13
        
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
        
// D:\Software code\Projects Software\Projects new\0. Non AMI\SUGAM Based\0. GDEV72 - BootCode-20210104T103109Z-001\0. GDEV72 - BootCode\source_code\driver_files\r_cg_lvd.c
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
//   21 * File Name    : r_cg_lvd.c
//   22 * Version      : Applilet4 for RL78/I1C V1.01.04.02 [20 Nov 2019]
//   23 * Device(s)    : R5F10NPJ
//   24 * Tool-Chain   : IAR Systems icc78k0r
//   25 * Description  : This file implements device driver for LVD module.
//   26 * Creation Date: 06/26/2020
//   27 ***********************************************************************************************************************/
//   28 
//   29 /***********************************************************************************************************************
//   30 Includes
//   31 ***********************************************************************************************************************/
//   32 #include "r_cg_macrodriver.h"

        ASEGN `.bss.noinit`:DATA:NOROOT,0fffd2H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_IF3
// __no_init union <unnamed>#118 volatile __sfr _A_IF3
__A_IF3:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0fffd6H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_MK3
// __no_init union <unnamed>#128 volatile __sfr _A_MK3
__A_MK3:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0fffdaH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_PR03
// __no_init union <unnamed>#138 volatile __sfr _A_PR03
__A_PR03:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0fffdeH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_PR13
// __no_init union <unnamed>#148 volatile __sfr _A_PR13
__A_PR13:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0fffe0H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_IF0
// __no_init union <unnamed>#152 volatile __sfr _A_IF0
__A_IF0:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0fffe4H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_MK0
// __no_init union <unnamed>#170 volatile __sfr _A_MK0
__A_MK0:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0332H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_LVDVDD
// __no_init union <unnamed>#544 volatile _A_LVDVDD
__A_LVDVDD:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0333H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_LVDVBAT
// __no_init union <unnamed>#546 volatile _A_LVDVBAT
__A_LVDVBAT:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0334H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_LVDVRTC
// __no_init union <unnamed>#548 volatile _A_LVDVRTC
__A_LVDVRTC:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0335H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_LVDEXLVD
// __no_init union <unnamed>#550 volatile _A_LVDEXLVD
__A_LVDEXLVD:
        DS 1
//   33 #include "r_cg_lvd.h"
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
//   50 * Function Name: R_LVD_Create
//   51 * Description  : This function initializes the voltage detector.
//   52 * Arguments    : None
//   53 * Return Value : None
//   54 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock0 Using cfiCommon0
          CFI Function _R_LVD_Create
        CODE
//   55 void R_LVD_Create(void)
//   56 {
_R_LVD_Create:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//   57     LVIMK = 1U;     /* disable INTLVI interrupt */
        SET1      0xFFFE4.1          ;; 2 cycles
//   58     LVIIF = 0U;     /* clear INTLVI interrupt flag */
        CLR1      0xFFFE0.1          ;; 2 cycles
//   59     LVDVDMK = 1U;   /* disable INTLVDVDD interrupt */
        SET1      0xFFFD6.4          ;; 2 cycles
//   60     LVDVDIF = 0U;   /* clear INTLVDVDD interrupt flag */
        CLR1      0xFFFD2.4          ;; 2 cycles
//   61     LVDVBMK = 1U;   /* disable INTLVDVBAT interrupt */
        SET1      0xFFFD6.5          ;; 2 cycles
//   62     LVDVBIF = 0U;   /* clear INTLVDVBAT interrupt flag */
        CLR1      0xFFFD2.5          ;; 2 cycles
//   63     LVDVRMK = 1U;   /* disable INTLVDVRTC interrupt */
        SET1      0xFFFD6.6          ;; 2 cycles
//   64     LVDVRIF = 0U;   /* clear INTLVDVRTC interrupt flag */
        CLR1      0xFFFD2.6          ;; 2 cycles
//   65     LVDEXMK = 1U;   /* disable INTLVDEXLVD interrupt */
        SET1      0xFFFD6.7          ;; 2 cycles
//   66     LVDEXIF = 0U;   /* clear INTLVDEXLVD interrupt flag */
        CLR1      0xFFFD2.7          ;; 2 cycles
//   67 //    /* Set INTLVDVDD level 1 priority */
//   68 //    LVDVDPR1 = 0U;
//   69 //    LVDVDPR0 = 1U;
//   70 //    /* Set INTLVDVRTC low priority */
//   71 //    LVDVRPR1 = 1U;
//   72 //    LVDVRPR0 = 1U;
//   73     /* Set INTLVDEXLVD level 1 priority */
//   74     LVDEXPR1 = 0U;
        CLR1      0xFFFDE.7          ;; 2 cycles
//   75     LVDEXPR0 = 1U;
        SET1      0xFFFDA.7          ;; 2 cycles
//   76 
//   77     R_LVD_Create_UserInit();
          CFI FunCall _R_LVD_Create_UserInit
        CALL      _R_LVD_Create_UserInit  ;; 3 cycles
//   78 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock0
        ; ------------------------------------- Block: 33 cycles
        ; ------------------------------------- Total: 33 cycles
        REQUIRE __A_MK0
        REQUIRE __A_IF0
        REQUIRE __A_MK3
        REQUIRE __A_IF3
        REQUIRE __A_PR13
        REQUIRE __A_PR03
//   79 
//   80 /***********************************************************************************************************************
//   81 * Function Name: R_LVD_InterruptMode_Start
//   82 * Description  : This function enables the voltage detector interrupt.
//   83 * Arguments    : None
//   84 * Return Value : None
//   85 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock1 Using cfiCommon0
          CFI Function _R_LVD_InterruptMode_Start
          CFI NoCalls
        CODE
//   86 void R_LVD_InterruptMode_Start(void)
//   87 {
_R_LVD_InterruptMode_Start:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//   88     LVIIF = 0U;     /* clear INTLVI interrupt flag */
        CLR1      0xFFFE0.1          ;; 2 cycles
//   89     LVIMK = 0U;     /* enable INTLVI interrupt */
        CLR1      0xFFFE4.1          ;; 2 cycles
//   90 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock1
        ; ------------------------------------- Block: 10 cycles
        ; ------------------------------------- Total: 10 cycles
        REQUIRE __A_IF0
        REQUIRE __A_MK0
//   91 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock2 Using cfiCommon0
          CFI Function _R_LVD_InterruptMode_Stop
          CFI NoCalls
        CODE
//   92 void R_LVD_InterruptMode_Stop(void)
//   93 {
_R_LVD_InterruptMode_Stop:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//   94     LVIMK = 1U;     /* enable INTLVI interrupt */
        SET1      0xFFFE4.1          ;; 2 cycles
//   95     LVIIF = 0U;     /* clear INTLVI interrupt flag */
        CLR1      0xFFFE0.1          ;; 2 cycles
//   96 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock2
        ; ------------------------------------- Block: 10 cycles
        ; ------------------------------------- Total: 10 cycles
        REQUIRE __A_MK0
        REQUIRE __A_IF0
//   97 /***********************************************************************************************************************
//   98 * Function Name: R_LVD_Start_VDD
//   99 * Description  : This function enables the voltage detector VDD.
//  100 * Arguments    : None
//  101 * Return Value : None
//  102 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock3 Using cfiCommon0
          CFI Function _R_LVD_Start_VDD
          CFI NoCalls
        CODE
//  103 void R_LVD_Start_VDD(void)
//  104 {
_R_LVD_Start_VDD:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  105     LVDVDIF = 0U;   /* clear INTLVDVDD interrupt flag */
        CLR1      0xFFFD2.4          ;; 2 cycles
//  106     LVDVDMK = 0U;   /* enable INTLVDVDD interrupt */
        CLR1      0xFFFD6.4          ;; 2 cycles
//  107     LVDVDD = _80_LVD_VDD_DELECT_ENABLE | _02_LVD_VDD_VOLTAGE_288;
        MOV       0x332, #0x82       ;; 1 cycle
//  108 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock3
        ; ------------------------------------- Block: 11 cycles
        ; ------------------------------------- Total: 11 cycles
        REQUIRE __A_IF3
        REQUIRE __A_MK3
        REQUIRE __A_LVDVDD
//  109 
//  110 /***********************************************************************************************************************
//  111 * Function Name: R_LVD_Stop_VDD
//  112 * Description  : This function disables the voltage detector VDD.
//  113 * Arguments    : None
//  114 * Return Value : None
//  115 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock4 Using cfiCommon0
          CFI Function _R_LVD_Stop_VDD
          CFI NoCalls
        CODE
//  116 void R_LVD_Stop_VDD(void)
//  117 {
_R_LVD_Stop_VDD:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  118     LVDVDMK = 1U;   /* disable INTLVDVDD interrupt */
        SET1      0xFFFD6.4          ;; 2 cycles
//  119     LVDVDIF = 0U;   /* clear INTLVDVDD interrupt flag */
        CLR1      0xFFFD2.4          ;; 2 cycles
//  120     LVDVDDEN = 0U;  /* disables detection */
        CLR1      0xF0332.7          ;; 2 cycles
//  121 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock4
        ; ------------------------------------- Block: 12 cycles
        ; ------------------------------------- Total: 12 cycles
        REQUIRE __A_MK3
        REQUIRE __A_IF3
        REQUIRE __A_LVDVDD
//  122 
//  123 /***********************************************************************************************************************
//  124 * Function Name: R_LVD_Start_VBAT
//  125 * Description  : This function enables the voltage detector VBAT.
//  126 * Arguments    : None
//  127 * Return Value : None
//  128 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock5 Using cfiCommon0
          CFI Function _R_LVD_Start_VBAT
          CFI NoCalls
        CODE
//  129 void R_LVD_Start_VBAT(void)
//  130 {
_R_LVD_Start_VBAT:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  131     LVDVBIF = 0U;   /* clear INTLVDVBAT interrupt flag */
        CLR1      0xFFFD2.5          ;; 2 cycles
//  132     LVDVBMK = 0U;   /* enable INTLVDVBAT interrupt */
        CLR1      0xFFFD6.5          ;; 2 cycles
//  133     LVDVBAT = _00_LVD_VBAT_DELECT_DISABLE;
        MOV       0x333, #0x0        ;; 1 cycle
//  134 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock5
        ; ------------------------------------- Block: 11 cycles
        ; ------------------------------------- Total: 11 cycles
        REQUIRE __A_IF3
        REQUIRE __A_MK3
        REQUIRE __A_LVDVBAT
//  135 
//  136 /***********************************************************************************************************************
//  137 * Function Name: R_LVD_Stop_VBAT
//  138 * Description  : This function disables the voltage detector VBAT.
//  139 * Arguments    : None
//  140 * Return Value : None
//  141 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock6 Using cfiCommon0
          CFI Function _R_LVD_Stop_VBAT
          CFI NoCalls
        CODE
//  142 void R_LVD_Stop_VBAT(void)
//  143 {
_R_LVD_Stop_VBAT:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  144     LVDVBMK = 1U;   /* disable INTLVDVBAT interrupt */
        SET1      0xFFFD6.5          ;; 2 cycles
//  145     LVDVBIF = 0U;   /* clear INTLVDVBAT interrupt flag */
        CLR1      0xFFFD2.5          ;; 2 cycles
//  146     LVDVBATEN = 0U; /* disables detection */
        CLR1      0xF0333.7          ;; 2 cycles
//  147 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock6
        ; ------------------------------------- Block: 12 cycles
        ; ------------------------------------- Total: 12 cycles
        REQUIRE __A_MK3
        REQUIRE __A_IF3
        REQUIRE __A_LVDVBAT
//  148 
//  149 /***********************************************************************************************************************
//  150 * Function Name: R_LVD_Start_VRTC
//  151 * Description  : This function enables the voltage detector VRTC.
//  152 * Arguments    : None
//  153 * Return Value : None
//  154 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock7 Using cfiCommon0
          CFI Function _R_LVD_Start_VRTC
          CFI NoCalls
        CODE
//  155 void R_LVD_Start_VRTC(void)
//  156 {
_R_LVD_Start_VRTC:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  157     LVDVRIF = 0U;   /* clear INTLVDVRTC interrupt flag */
        CLR1      0xFFFD2.6          ;; 2 cycles
//  158     LVDVRMK = 0U;   /* enable INTLVDVRTC interrupt */
        CLR1      0xFFFD6.6          ;; 2 cycles
//  159     LVDVRTC = _80_LVD_VRTC_DELECT_ENABLE | _00_LVD_VRTC_VOLTAGE_216;
        MOV       0x334, #0x80       ;; 1 cycle
//  160 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock7
        ; ------------------------------------- Block: 11 cycles
        ; ------------------------------------- Total: 11 cycles
        REQUIRE __A_IF3
        REQUIRE __A_MK3
        REQUIRE __A_LVDVRTC
//  161 
//  162 /***********************************************************************************************************************
//  163 * Function Name: R_LVD_Stop_VRTC
//  164 * Description  : This function disables the voltage detector VRTC.
//  165 * Arguments    : None
//  166 * Return Value : None
//  167 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock8 Using cfiCommon0
          CFI Function _R_LVD_Stop_VRTC
          CFI NoCalls
        CODE
//  168 void R_LVD_Stop_VRTC(void)
//  169 {
_R_LVD_Stop_VRTC:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  170     LVDVRMK = 1U;   /* disable INTLVDVRTC interrupt */
        SET1      0xFFFD6.6          ;; 2 cycles
//  171     LVDVRIF = 0U;   /* clear INTLVDVRTC interrupt flag */
        CLR1      0xFFFD2.6          ;; 2 cycles
//  172     LVDVRTCEN = 0U; /* disables detection */
        CLR1      0xF0334.7          ;; 2 cycles
//  173 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock8
        ; ------------------------------------- Block: 12 cycles
        ; ------------------------------------- Total: 12 cycles
        REQUIRE __A_MK3
        REQUIRE __A_IF3
        REQUIRE __A_LVDVRTC
//  174 
//  175 /***********************************************************************************************************************
//  176 * Function Name: R_LVD_Start_EXLVD
//  177 * Description  : This function enables the voltage detector EXLVD.
//  178 * Arguments    : None
//  179 * Return Value : None
//  180 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock9 Using cfiCommon0
          CFI Function _R_LVD_Start_EXLVD
          CFI NoCalls
        CODE
//  181 void R_LVD_Start_EXLVD(void)
//  182 {
_R_LVD_Start_EXLVD:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  183     LVDEXIF = 0U;   /* clear INTLVDEXLVD interrupt flag */
        CLR1      0xFFFD2.7          ;; 2 cycles
//  184     LVDEXMK = 0U;   /* enable INTLVDEXLVD interrupt */
        CLR1      0xFFFD6.7          ;; 2 cycles
//  185     LVDEXLVD = _80_LVD_EXLVD_DELECT_ENABLE;
        MOV       0x335, #0x80       ;; 1 cycle
//  186 
//  187     /* Set EXLVD pin */
//  188     //PM2 |= 0x04U;
//  189 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock9
        ; ------------------------------------- Block: 11 cycles
        ; ------------------------------------- Total: 11 cycles
        REQUIRE __A_IF3
        REQUIRE __A_MK3
        REQUIRE __A_LVDEXLVD
//  190 
//  191 /***********************************************************************************************************************
//  192 * Function Name: R_LVD_Stop_EXLVD
//  193 * Description  : This function disables the voltage detector EXLVD.
//  194 * Arguments    : None
//  195 * Return Value : None
//  196 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock10 Using cfiCommon0
          CFI Function _R_LVD_Stop_EXLVD
          CFI NoCalls
        CODE
//  197 void R_LVD_Stop_EXLVD(void)
//  198 {
_R_LVD_Stop_EXLVD:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  199     LVDEXMK = 1U;   /* disable INTLVDEXLVD interrupt */
        SET1      0xFFFD6.7          ;; 2 cycles
//  200     LVDEXIF = 0U;   /* clear INTLVDEXLVD interrupt flag */
        CLR1      0xFFFD2.7          ;; 2 cycles
//  201     LVDEXLVDEN = 0U;/* disables detection */
        CLR1      0xF0335.7          ;; 2 cycles
//  202 
//  203     /* Set EXLVD pin */
//  204     //PM2 &= 0xFBU;
//  205 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock10
        ; ------------------------------------- Block: 12 cycles
        ; ------------------------------------- Total: 12 cycles
        REQUIRE __A_MK3
        REQUIRE __A_IF3
        REQUIRE __A_LVDEXLVD

        SECTION `.iar_vfe_header`:DATA:NOALLOC:NOROOT(1)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
        DC32 0

        END
//  206 
//  207 /* Start user code for adding. Do not edit comment generated here */
//  208 /* End user code. Do not edit comment generated here */
// 
//  16 bytes in section .bss.noinit  (abs)
// 142 bytes in section .text
// 
// 142 bytes of CODE memory
//   0 bytes of DATA memory (+ 16 bytes shared)
//
//Errors: none
//Warnings: none
