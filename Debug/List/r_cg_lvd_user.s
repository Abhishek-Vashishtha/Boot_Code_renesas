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
//        BootCode\source_code\driver_files\r_cg_lvd_user.c
//    Command line       =
//        -f C:\Users\17054\AppData\Local\Temp\EWDCDF.tmp ("D:\Software
//        code\Projects Software\Projects new\0. Non AMI\SUGAM Based\0. GDEV72
//        - BootCode-20210104T103109Z-001\0. GDEV72 -
//        BootCode\source_code\driver_files\r_cg_lvd_user.c" --core s3
//        --code_model near --calling_convention v2 --near_const_location ram
//        -o "D:\Software code\Projects Software\Projects new\0. Non AMI\SUGAM
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
//        BootCode\Debug\List\r_cg_lvd_user.s
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

        EXTERN _last_interrupt
        EXTERN _flag_lvd1
        EXTERN _flag_lvd2
        EXTERN ___interrupt_tab_0x06
        EXTERN ___interrupt_tab_0x6C
        EXTERN ___interrupt_tab_0x6E
        EXTERN ___interrupt_tab_0x70
        EXTERN ___interrupt_tab_0x72
        EXTERN _backlight_operation

        PUBLIC _R_LVD_Create_UserInit
        PUBLIC __A_LVDVBAT
        PUBLIC __A_LVDVDD
        PUBLIC __A_LVDVRTC
        PUBLIC __A_LVIM
        PUBLIC ___interrupt_0x06
        PUBLIC ___interrupt_0x6C
        PUBLIC ___interrupt_0x6E
        PUBLIC ___interrupt_0x70
        PUBLIC ___interrupt_0x72
        
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
          CFI A SameValue
          CFI X SameValue
          CFI B SameValue
          CFI C SameValue
          CFI D SameValue
          CFI E SameValue
          CFI H SameValue
          CFI L SameValue
          CFI CS_REG SameValue
          CFI ES_REG SameValue
          CFI ?RET Frame(CFA, -4)
          CFI MACRH SameValue
          CFI MACRL SameValue
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
        
// D:\Software code\Projects Software\Projects new\0. Non AMI\SUGAM Based\0. GDEV72 - BootCode-20210104T103109Z-001\0. GDEV72 - BootCode\source_code\driver_files\r_cg_lvd_user.c
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
//   21 * File Name    : r_cg_lvd_user.c
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

        ASEGN `.bss.noinit`:DATA:NOROOT,0fffa9H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_LVIM
// __no_init union <unnamed>#106 volatile __sfr _A_LVIM
__A_LVIM:
        DS 1

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
//   33 #include "r_cg_lvd.h"
//   34 /* Start user code for include. Do not edit comment generated here */
//   35 /* End user code. Do not edit comment generated here */
//   36 #include "pcb.h"
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
//   50 * Function Name: R_LVD_Create_UserInit
//   51 * Description  : This function adds user code after initializing the voltage detector.
//   52 * Arguments    : None
//   53 * Return Value : None
//   54 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock0 Using cfiCommon0
          CFI Function _R_LVD_Create_UserInit
          CFI NoCalls
        CODE
//   55 void R_LVD_Create_UserInit(void)
//   56 {
_R_LVD_Create_UserInit:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//   57     /* Start user code. Do not edit comment generated here */
//   58     /* End user code. Do not edit comment generated here */
//   59 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock0
        ; ------------------------------------- Block: 6 cycles
        ; ------------------------------------- Total: 6 cycles
//   60 /***********************************************************************************************************************
//   61 * Function Name: r_lvd_interrupt
//   62 * Description  : None
//   63 * Arguments    : None
//   64 * Return Value : None
//   65 ***********************************************************************************************************************/
//   66 #pragma vector = INTLVI_vect

        SECTION `.text`:CODE:NOROOT(0)
        CALL_GRAPH_ROOT _r_lvd_interrupt, "interrupt"
          CFI Block cfiBlock1 Using cfiCommon1
          CFI Function _r_lvd_interrupt
          CFI NoCalls
        CODE
//   67 __interrupt static void r_lvd_interrupt(void)
//   68 {
_r_lvd_interrupt:
___interrupt_0x06:
        ; * Stack frame (at entry) *
        ; Param size: 0
        PUSH      HL                 ;; 1 cycle
          CFI L Frame(CFA, -6)
          CFI H Frame(CFA, -5)
          CFI CFA SP+6
        ; Auto size: 0
//   69     /* Start user code. Do not edit comment generated here */
//   70     last_interrupt = 18;
        MOV       N:_last_interrupt, #0x12  ;; 1 cycle
//   71     flag_lvd_interrupt = 1;
        SET1      N:_flag_lvd1.0     ;; 2 cycles
//   72     if(flag_lvd_instant == 1)
        MOVW      HL, #0xFFA9        ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??r_lvd_exlvdinterrupt_0  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
//   73     {
//   74         flag_lvd_status = 1;
        SET1      N:_flag_lvd2.0     ;; 2 cycles
        BR        S:??r_lvd_exlvdinterrupt_1  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//   75     }
//   76     else
//   77     {
//   78         flag_lvd_status = 0;
??r_lvd_exlvdinterrupt_0:
        CLR1      N:_flag_lvd2.0     ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
//   79     }
//   80     /* End user code. Do not edit comment generated here */
//   81 }
??r_lvd_exlvdinterrupt_1:
        POP       HL                 ;; 1 cycle
          CFI L SameValue
          CFI H SameValue
          CFI CFA SP+4
        RETI                         ;; 6 cycles
          CFI EndBlock cfiBlock1
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 24 cycles
        REQUIRE __A_LVIM
        REQUIRE ___interrupt_tab_0x06
//   82 
//   83 /***********************************************************************************************************************
//   84 * Function Name: r_lvd_vddinterrupt
//   85 * Description  : None
//   86 * Arguments    : None
//   87 * Return Value : None
//   88 ***********************************************************************************************************************/
//   89 #pragma vector = INTLVDVDD_vect

        SECTION `.text`:CODE:NOROOT(0)
        CALL_GRAPH_ROOT _r_lvd_vddinterrupt, "interrupt"
          CFI Block cfiBlock2 Using cfiCommon1
          CFI Function _r_lvd_vddinterrupt
          CFI NoCalls
        CODE
//   90 __interrupt static void r_lvd_vddinterrupt(void)
//   91 {
_r_lvd_vddinterrupt:
___interrupt_0x6C:
        ; * Stack frame (at entry) *
        ; Param size: 0
        PUSH      HL                 ;; 1 cycle
          CFI L Frame(CFA, -6)
          CFI H Frame(CFA, -5)
          CFI CFA SP+6
        ; Auto size: 0
//   92     /* Start user code. Do not edit comment generated here */
//   93     last_interrupt = 19;
        MOV       N:_last_interrupt, #0x13  ;; 1 cycle
//   94     flag_lvd_vdd_interrupt = 1;
        SET1      N:_flag_lvd1.1     ;; 2 cycles
//   95     if(flag_lvd_vdd_instant == 1)
        MOVW      HL, #0x332         ;; 1 cycle
        MOV1      CY, [HL].6         ;; 1 cycle
        BNC       ??r_lvd_exlvdinterrupt_2  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
//   96     {
//   97         flag_lvd_vdd_status = 1;
        SET1      N:_flag_lvd2.1     ;; 2 cycles
        BR        S:??r_lvd_exlvdinterrupt_3  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//   98     }
//   99     else
//  100     {
//  101         flag_lvd_vdd_status = 0;
??r_lvd_exlvdinterrupt_2:
        CLR1      N:_flag_lvd2.1     ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
//  102     }
//  103 /* End user code. Do not edit comment generated here */
//  104 }
??r_lvd_exlvdinterrupt_3:
        POP       HL                 ;; 1 cycle
          CFI L SameValue
          CFI H SameValue
          CFI CFA SP+4
        RETI                         ;; 6 cycles
          CFI EndBlock cfiBlock2
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 24 cycles
        REQUIRE __A_LVDVDD
        REQUIRE ___interrupt_tab_0x6C
//  105 
//  106 /***********************************************************************************************************************
//  107 * Function Name: r_lvd_vbatinterrupt
//  108 * Description  : None
//  109 * Arguments    : None
//  110 * Return Value : None
//  111 ***********************************************************************************************************************/
//  112 #pragma vector = INTLVDVBAT_vect

        SECTION `.text`:CODE:NOROOT(0)
        CALL_GRAPH_ROOT _r_lvd_vbatinterrupt, "interrupt"
          CFI Block cfiBlock3 Using cfiCommon1
          CFI Function _r_lvd_vbatinterrupt
          CFI NoCalls
        CODE
//  113 __interrupt static void r_lvd_vbatinterrupt(void)
//  114 {
_r_lvd_vbatinterrupt:
___interrupt_0x6E:
        ; * Stack frame (at entry) *
        ; Param size: 0
        PUSH      HL                 ;; 1 cycle
          CFI L Frame(CFA, -6)
          CFI H Frame(CFA, -5)
          CFI CFA SP+6
        ; Auto size: 0
//  115     /* Start user code. Do not edit comment generated here */
//  116     last_interrupt = 20;
        MOV       N:_last_interrupt, #0x14  ;; 1 cycle
//  117     flag_lvd_vbat_interrupt = 1;
        SET1      N:_flag_lvd1.2     ;; 2 cycles
//  118     if(flag_lvd_vbat_instant == 1)
        MOVW      HL, #0x333         ;; 1 cycle
        MOV1      CY, [HL].6         ;; 1 cycle
        BNC       ??r_lvd_exlvdinterrupt_4  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
//  119     {
//  120         flag_lvd_vbat_status = 1;
        SET1      N:_flag_lvd2.2     ;; 2 cycles
        BR        S:??r_lvd_exlvdinterrupt_5  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  121     }
//  122     else
//  123     {
//  124         flag_lvd_vbat_status = 0;
??r_lvd_exlvdinterrupt_4:
        CLR1      N:_flag_lvd2.2     ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
//  125     }
//  126     /* End user code. Do not edit comment generated here */
//  127 }
??r_lvd_exlvdinterrupt_5:
        POP       HL                 ;; 1 cycle
          CFI L SameValue
          CFI H SameValue
          CFI CFA SP+4
        RETI                         ;; 6 cycles
          CFI EndBlock cfiBlock3
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 24 cycles
        REQUIRE __A_LVDVBAT
        REQUIRE ___interrupt_tab_0x6E
//  128 
//  129 /***********************************************************************************************************************
//  130 * Function Name: r_lvd_vrtcinterrupt
//  131 * Description  : None
//  132 * Arguments    : None
//  133 * Return Value : None
//  134 ***********************************************************************************************************************/
//  135 #pragma vector = INTLVDVRTC_vect

        SECTION `.text`:CODE:NOROOT(0)
        CALL_GRAPH_ROOT _r_lvd_vrtcinterrupt, "interrupt"
          CFI Block cfiBlock4 Using cfiCommon1
          CFI Function _r_lvd_vrtcinterrupt
          CFI NoCalls
        CODE
//  136 __interrupt static void r_lvd_vrtcinterrupt(void)
//  137 {
_r_lvd_vrtcinterrupt:
___interrupt_0x70:
        ; * Stack frame (at entry) *
        ; Param size: 0
        PUSH      HL                 ;; 1 cycle
          CFI L Frame(CFA, -6)
          CFI H Frame(CFA, -5)
          CFI CFA SP+6
        ; Auto size: 0
//  138     /* Start user code. Do not edit comment generated here */
//  139     last_interrupt = 21;
        MOV       N:_last_interrupt, #0x15  ;; 1 cycle
//  140     flag_lvd_rtc_interrupt = 1;
        SET1      N:_flag_lvd1.3     ;; 2 cycles
//  141     if(flag_lvd_rtc_instant == 1)
        MOVW      HL, #0x334         ;; 1 cycle
        MOV1      CY, [HL].6         ;; 1 cycle
        BNC       ??r_lvd_exlvdinterrupt_6  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
//  142     {
//  143         flag_lvd_rtc_status = 1;
        SET1      N:_flag_lvd2.3     ;; 2 cycles
        BR        S:??r_lvd_exlvdinterrupt_7  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  144     }
//  145     else
//  146     {
//  147         flag_lvd_rtc_status = 0;
??r_lvd_exlvdinterrupt_6:
        CLR1      N:_flag_lvd2.3     ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
//  148     }
//  149     /* End user code. Do not edit comment generated here */
//  150 }
??r_lvd_exlvdinterrupt_7:
        POP       HL                 ;; 1 cycle
          CFI L SameValue
          CFI H SameValue
          CFI CFA SP+4
        RETI                         ;; 6 cycles
          CFI EndBlock cfiBlock4
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 24 cycles
        REQUIRE __A_LVDVRTC
        REQUIRE ___interrupt_tab_0x70
//  151 
//  152 /***********************************************************************************************************************
//  153 * Function Name: r_lvd_exlvdinterrupt
//  154 * Description  : None
//  155 * Arguments    : None
//  156 * Return Value : None
//  157 ***********************************************************************************************************************/
//  158 #pragma vector = INTLVDEXLVD_vect

        SECTION `.text`:CODE:NOROOT(0)
        CALL_GRAPH_ROOT _r_lvd_exlvdinterrupt, "interrupt"
          CFI Block cfiBlock5 Using cfiCommon1
          CFI Function _r_lvd_exlvdinterrupt
        CODE
//  159 __interrupt static void r_lvd_exlvdinterrupt(void)
//  160 {
_r_lvd_exlvdinterrupt:
___interrupt_0x72:
        ; * Stack frame (at entry) *
        ; Param size: 0
        PUSH      AX                 ;; 1 cycle
          CFI X Frame(CFA, -6)
          CFI A Frame(CFA, -5)
          CFI CFA SP+6
        PUSH      BC                 ;; 1 cycle
          CFI C Frame(CFA, -8)
          CFI B Frame(CFA, -7)
          CFI CFA SP+8
        PUSH      DE                 ;; 1 cycle
          CFI E Frame(CFA, -10)
          CFI D Frame(CFA, -9)
          CFI CFA SP+10
        PUSH      HL                 ;; 1 cycle
          CFI L Frame(CFA, -12)
          CFI H Frame(CFA, -11)
          CFI CFA SP+12
        MOVW      AX, 0xFFFFC        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+14
        ; Auto size: 0
//  161     /* Start user code. Do not edit comment generated here */
//  162     last_interrupt = 22;
        MOV       N:_last_interrupt, #0x16  ;; 1 cycle
//  163     flag_lvd_exlvd_interrupt = 1;
        SET1      N:_flag_lvd1.4     ;; 2 cycles
//  164     backlight_operation();
          CFI FunCall _backlight_operation
        CALL      _backlight_operation  ;; 3 cycles
//  165 
//  166     /* End user code. Do not edit comment generated here */
//  167 }
        POP       AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      0xFFFFC, AX        ;; 1 cycle
        POP       HL                 ;; 1 cycle
          CFI L SameValue
          CFI H SameValue
          CFI CFA SP+10
        POP       DE                 ;; 1 cycle
          CFI E SameValue
          CFI D SameValue
          CFI CFA SP+8
        POP       BC                 ;; 1 cycle
          CFI C SameValue
          CFI B SameValue
          CFI CFA SP+6
        POP       AX                 ;; 1 cycle
          CFI X SameValue
          CFI A SameValue
          CFI CFA SP+4
        RETI                         ;; 6 cycles
          CFI EndBlock cfiBlock5
        ; ------------------------------------- Block: 24 cycles
        ; ------------------------------------- Total: 24 cycles
        REQUIRE ___interrupt_tab_0x72

        SECTION `.iar_vfe_header`:DATA:NOALLOC:NOROOT(1)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
        DC32 0

        END
//  168 
//  169 /* Start user code for adding. Do not edit comment generated here */
//  170 /* End user code. Do not edit comment generated here */
// 
//   4 bytes in section .bss.noinit  (abs)
// 144 bytes in section .text
// 
// 144 bytes of CODE memory
//   0 bytes of DATA memory (+ 4 bytes shared)
//
//Errors: none
//Warnings: none
