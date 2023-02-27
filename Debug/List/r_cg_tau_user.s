///////////////////////////////////////////////////////////////////////////////
//
// IAR C/C++ Compiler V4.20.1.2260 for RL78               22/Dec/2020  15:13:09
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
//        BootCode\source_code\driver_files\r_cg_tau_user.c
//    Command line       =
//        -f C:\Users\laptop\AppData\Local\Temp\EW2CFD.tmp ("D:\Dheeraj\New
//        folder\0. GDEV72 - BootCode\source_code\driver_files\r_cg_tau_user.c"
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
//        D:\Dheeraj\New folder\0. GDEV72 - BootCode\Debug\List\r_cg_tau_user.s
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

        EXTERN _last_interrupt
        EXTERN ___interrupt_tab_0x2C
        EXTERN ___interrupt_tab_0x32
        EXTERN ___interrupt_tab_0x34
        EXTERN ___interrupt_tab_0x36
        EXTERN ___interrupt_tab_0x46
        EXTERN ___interrupt_tab_0x48
        EXTERN ___interrupt_tab_0x54

        PUBLIC ___interrupt_0x2C
        PUBLIC ___interrupt_0x32
        PUBLIC ___interrupt_0x34
        PUBLIC ___interrupt_0x36
        PUBLIC ___interrupt_0x46
        PUBLIC ___interrupt_0x48
        PUBLIC ___interrupt_0x54
        
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
          CFI EndCommon cfiCommon0
        
// D:\Dheeraj\New folder\0. GDEV72 - BootCode\source_code\driver_files\r_cg_tau_user.c
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
//   21 * File Name    : r_cg_tau_user.c
//   22 * Version      : 
//   23 * Device(s)    : RL78/I1C
//   24 * Tool-Chain   : CCRL
//   25 * Description  : This file implements device driver for TAU module.
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
//   39 #include "r_cg_tau.h"
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
//   51 * Function Name: r_tau0_channel0_interrupt
//   52 * Description  : This function INTTM00 interrupt service routine.
//   53 * Arguments    : None
//   54 * Return Value : None
//   55 ***********************************************************************************************************************/
//   56 #pragma vector = INTTM00_vect

        SECTION `.text`:CODE:NOROOT(0)
        CALL_GRAPH_ROOT _r_tau0_channel0_interrupt, "interrupt"
          CFI Block cfiBlock0 Using cfiCommon0
          CFI Function _r_tau0_channel0_interrupt
          CFI NoCalls
        CODE
//   57 __interrupt static void r_tau0_channel0_interrupt(void)
//   58 {
_r_tau0_channel0_interrupt:
___interrupt_0x2C:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//   59     /* Start user code. Do not edit comment generated here */
//   60     last_interrupt = 31;
        MOV       N:_last_interrupt, #0x1F  ;; 1 cycle
//   61     NOP();
        NOP                          ;; 1 cycle
//   62     /* End user code. Do not edit comment generated here */
//   63 }
        RETI                         ;; 6 cycles
          CFI EndBlock cfiBlock0
        ; ------------------------------------- Block: 8 cycles
        ; ------------------------------------- Total: 8 cycles
        REQUIRE ___interrupt_tab_0x2C
//   64 
//   65 /***********************************************************************************************************************
//   66 * Function Name: r_tau0_channel1_interrupt
//   67 * Description  : This function INTTM01 interrupt service routine.
//   68 * Arguments    : None
//   69 * Return Value : None
//   70 ***********************************************************************************************************************/
//   71 #pragma vector = INTTM01_vect

        SECTION `.text`:CODE:NOROOT(0)
        CALL_GRAPH_ROOT _r_tau0_channel1_interrupt, "interrupt"
          CFI Block cfiBlock1 Using cfiCommon0
          CFI Function _r_tau0_channel1_interrupt
          CFI NoCalls
        CODE
//   72 __interrupt static void r_tau0_channel1_interrupt(void)
//   73 {
_r_tau0_channel1_interrupt:
___interrupt_0x32:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//   74     /* Start user code. Do not edit comment generated here */
//   75     //R_ADC_Start();
//   76      last_interrupt = 32;
        MOV       N:_last_interrupt, #0x20  ;; 1 cycle
//   77      NOP();
        NOP                          ;; 1 cycle
//   78     /* End user code. Do not edit comment generated here */
//   79 }
        RETI                         ;; 6 cycles
          CFI EndBlock cfiBlock1
        ; ------------------------------------- Block: 8 cycles
        ; ------------------------------------- Total: 8 cycles
        REQUIRE ___interrupt_tab_0x32
//   80 
//   81 /***********************************************************************************************************************
//   82 * Function Name: r_tau0_channel2_interrupt
//   83 * Description  : This function INTTM02 interrupt service routine.
//   84 * Arguments    : None
//   85 * Return Value : None
//   86 ***********************************************************************************************************************/
//   87 #pragma vector = INTTM02_vect

        SECTION `.text`:CODE:NOROOT(0)
        CALL_GRAPH_ROOT _r_tau0_channel2_interrupt, "interrupt"
          CFI Block cfiBlock2 Using cfiCommon0
          CFI Function _r_tau0_channel2_interrupt
          CFI NoCalls
        CODE
//   88 __interrupt static void r_tau0_channel2_interrupt(void)
//   89 {
_r_tau0_channel2_interrupt:
___interrupt_0x34:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//   90     /* Start user code. Do not edit comment generated here */
//   91     last_interrupt = 33;
        MOV       N:_last_interrupt, #0x21  ;; 1 cycle
//   92     /* End user code. Do not edit comment generated here */
//   93 }
        RETI                         ;; 6 cycles
          CFI EndBlock cfiBlock2
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 7 cycles
        REQUIRE ___interrupt_tab_0x34
//   94 
//   95 /***********************************************************************************************************************
//   96 * Function Name: r_tau0_channel3_interrupt
//   97 * Description  : This function INTTM03 interrupt service routine.
//   98 * Arguments    : None
//   99 * Return Value : None
//  100 ***********************************************************************************************************************/
//  101 #pragma vector = INTTM03_vect

        SECTION `.text`:CODE:NOROOT(0)
        CALL_GRAPH_ROOT _r_tau0_channel3_interrupt, "interrupt"
          CFI Block cfiBlock3 Using cfiCommon0
          CFI Function _r_tau0_channel3_interrupt
          CFI NoCalls
        CODE
//  102 __interrupt static void r_tau0_channel3_interrupt(void)
//  103 {
_r_tau0_channel3_interrupt:
___interrupt_0x36:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  104     /* Start user code. Do not edit comment generated here */
//  105     last_interrupt = 34;
        MOV       N:_last_interrupt, #0x22  ;; 1 cycle
//  106     /* End user code. Do not edit comment generated here */
//  107 }
        RETI                         ;; 6 cycles
          CFI EndBlock cfiBlock3
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 7 cycles
        REQUIRE ___interrupt_tab_0x36
//  108 
//  109 /***********************************************************************************************************************
//  110 * Function Name: r_tau0_channel4_interrupt
//  111 * Description  : This function INTTM04 interrupt service routine.
//  112 * Arguments    : None
//  113 * Return Value : None
//  114 ***********************************************************************************************************************/
//  115 #pragma vector = INTTM04_vect

        SECTION `.text`:CODE:NOROOT(0)
        CALL_GRAPH_ROOT _r_tau0_channel4_interrupt, "interrupt"
          CFI Block cfiBlock4 Using cfiCommon0
          CFI Function _r_tau0_channel4_interrupt
          CFI NoCalls
        CODE
//  116 __interrupt static void r_tau0_channel4_interrupt(void)
//  117 {
_r_tau0_channel4_interrupt:
___interrupt_0x46:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  118     /* Start user code. Do not edit comment generated here */
//  119     last_interrupt = 35;
        MOV       N:_last_interrupt, #0x23  ;; 1 cycle
//  120     /* End user code. Do not edit comment generated here */
//  121 }
        RETI                         ;; 6 cycles
          CFI EndBlock cfiBlock4
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 7 cycles
        REQUIRE ___interrupt_tab_0x46
//  122 
//  123 /***********************************************************************************************************************
//  124 * Function Name: r_tau0_channel5_interrupt
//  125 * Description  : This function INTTM05 interrupt service routine.
//  126 * Arguments    : None
//  127 * Return Value : None
//  128 ***********************************************************************************************************************/
//  129 #pragma vector = INTTM05_vect

        SECTION `.text`:CODE:NOROOT(0)
        CALL_GRAPH_ROOT _r_tau0_channel5_interrupt, "interrupt"
          CFI Block cfiBlock5 Using cfiCommon0
          CFI Function _r_tau0_channel5_interrupt
          CFI NoCalls
        CODE
//  130 __interrupt static void r_tau0_channel5_interrupt(void)
//  131 {
_r_tau0_channel5_interrupt:
___interrupt_0x48:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  132     /* Start user code. Do not edit comment generated here */
//  133     last_interrupt = 36;
        MOV       N:_last_interrupt, #0x24  ;; 1 cycle
//  134     /* End user code. Do not edit comment generated here */
//  135 }
        RETI                         ;; 6 cycles
          CFI EndBlock cfiBlock5
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 7 cycles
        REQUIRE ___interrupt_tab_0x48
//  136 
//  137 /***********************************************************************************************************************
//  138 * Function Name: r_tau0_channel6_interrupt
//  139 * Description  : This function INTTM06 interrupt service routine.
//  140 * Arguments    : None
//  141 * Return Value : None
//  142 ***********************************************************************************************************************/
//  143 #pragma vector = INTTM06_vect

        SECTION `.text`:CODE:NOROOT(0)
        CALL_GRAPH_ROOT _r_tau0_channel6_interrupt, "interrupt"
          CFI Block cfiBlock6 Using cfiCommon0
          CFI Function _r_tau0_channel6_interrupt
          CFI NoCalls
        CODE
//  144 __interrupt static void r_tau0_channel6_interrupt(void)
//  145 {
_r_tau0_channel6_interrupt:
___interrupt_0x54:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  146     /* Start user code. Do not edit comment generated here */
//  147     last_interrupt = 37;
        MOV       N:_last_interrupt, #0x25  ;; 1 cycle
//  148     /* End user code. Do not edit comment generated here */
//  149 }
        RETI                         ;; 6 cycles
          CFI EndBlock cfiBlock6
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 7 cycles
        REQUIRE ___interrupt_tab_0x54

        SECTION `.iar_vfe_header`:DATA:NOALLOC:NOROOT(1)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
        DC32 0

        END
//  150 
//  151 /* Start user code for adding. Do not edit comment generated here */
//  152 /* End user code. Do not edit comment generated here */
// 
// 44 bytes in section .text
// 
// 44 bytes of CODE memory
//
//Errors: none
//Warnings: none
