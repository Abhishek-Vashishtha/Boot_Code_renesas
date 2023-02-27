///////////////////////////////////////////////////////////////////////////////
//
// IAR C/C++ Compiler V4.20.1.2260 for RL78               20/Dec/2020  22:38:49
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
//        BootCode\source_code\driver_files\r_cg_dsadc_user.c
//    Command line       =
//        -f C:\Users\laptop\AppData\Local\Temp\EWCB84.tmp ("D:\Dheeraj\New
//        folder\0. GDEV72 -
//        BootCode\source_code\driver_files\r_cg_dsadc_user.c" --core s3
//        --code_model near --calling_convention v2 --near_const_location ram
//        -o "D:\Dheeraj\New folder\0. GDEV72 - BootCode\Debug\Obj"
//        --dlib_config "C:\Program Files\IAR Systems\Embedded Workbench
//        8.4\rl78\LIB\DLib_Config_Normal.h" --double=32 -e -On --no_cse
//        --no_unroll --no_inline --no_code_motion --no_tbaa --no_cross_call
//        --no_scheduling --no_clustering --debug -lA "D:\Dheeraj\New folder\0.
//        GDEV72 - BootCode\Debug\List" -I "D:\Dheeraj\New folder\0. GDEV72 -
//        BootCode\source_code\driver_files\" -I "D:\Dheeraj\New folder\0.
//        GDEV72 - BootCode\source_code\library_files\" -I "D:\Dheeraj\New
//        folder\0. GDEV72 - BootCode\source_code\misc_files\" -I
//        "D:\Dheeraj\New folder\0. GDEV72 -
//        BootCode\source_code\source_files\" --data_model near)
//    Locale             =  C
//    List file          =
//        D:\Dheeraj\New folder\0. GDEV72 -
//        BootCode\Debug\List\r_cg_dsadc_user.s
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
        EXTERN _R_DTCD0_Start
        EXTERN ___interrupt_tab_0x44
        EXTERN ___interrupt_tab_0x64
        EXTERN ___interrupt_tab_0x66
        EXTERN _adc_data
        EXTERN _b_phase
        EXTERN _flag_metrology
        EXTERN _metrology_function
        EXTERN _n_phase
        EXTERN _r_phase
        EXTERN _y_phase

        PUBLIC _R_DSADC_Create_UserInit
        PUBLIC __A_DSADCR0
        PUBLIC __A_DSADCR1
        PUBLIC __A_DSADCR2
        PUBLIC __A_DSADCR3
        PUBLIC ___interrupt_0x44
        PUBLIC ___interrupt_0x64
        PUBLIC ___interrupt_0x66
        
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
        
// D:\Dheeraj\New folder\0. GDEV72 - BootCode\source_code\driver_files\r_cg_dsadc_user.c
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
//   21 * File Name    : r_cg_dsadc_user.c
//   22 * Version      : Applilet4 for RL78/I1C V1.01.02.04 [24 May 2018]
//   23 * Device(s)    : R5F10NPJ
//   24 * Tool-Chain   : IAR Systems icc78k0r
//   25 * Description  : This file implements device driver for DSADC module.
//   26 * Creation Date: 01/09/2020
//   27 ***********************************************************************************************************************/
//   28 
//   29 /***********************************************************************************************************************
//   30 Includes
//   31 ***********************************************************************************************************************/
//   32 #include "r_cg_macrodriver.h"

        ASEGN `.bss.noinit`:DATA:NOROOT,0f03e0H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_DSADCR0
// __no_init union <unnamed>#586 const volatile __no_bit_access _A_DSADCR0
__A_DSADCR0:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f03e4H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_DSADCR1
// __no_init union <unnamed>#591 const volatile __no_bit_access _A_DSADCR1
__A_DSADCR1:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f03e8H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_DSADCR2
// __no_init union <unnamed>#596 const volatile __no_bit_access _A_DSADCR2
__A_DSADCR2:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f03ecH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_DSADCR3
// __no_init union <unnamed>#601 const volatile __no_bit_access _A_DSADCR3
__A_DSADCR3:
        DS 2
//   33 #include "r_cg_dsadc.h"
//   34 /* Start user code for include. Do not edit comment generated here */
//   35 /* End user code. Do not edit comment generated here */
//   36 #include "r_cg_userdefine.h"
//   37 #include "variables.h"
//   38 /***********************************************************************************************************************
//   39 Pragma directive
//   40 ***********************************************************************************************************************/
//   41 
//   42 
//   43 /* Start user code for pragma. Do not edit comment generated here */
//   44 /* End user code. Do not edit comment generated here */
//   45 
//   46 /***********************************************************************************************************************
//   47 Global variables and functions
//   48 ***********************************************************************************************************************/
//   49 /* Start user code for global. Do not edit comment generated here */
//   50 /* End user code. Do not edit comment generated here */
//   51 
//   52 /***********************************************************************************************************************
//   53 * Function Name: R_DSADC_Create_UserInit
//   54 * Description  : This function adds user code after initializing the DSAD module.
//   55 * Arguments    : None
//   56 * Return Value : None
//   57 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock0 Using cfiCommon0
          CFI Function _R_DSADC_Create_UserInit
          CFI NoCalls
        CODE
//   58 void R_DSADC_Create_UserInit(void)
//   59 {
_R_DSADC_Create_UserInit:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//   60     /* Start user code. Do not edit comment generated here */
//   61     /* End user code. Do not edit comment generated here */
//   62 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock0
        ; ------------------------------------- Block: 6 cycles
        ; ------------------------------------- Total: 6 cycles
//   63 /***********************************************************************************************************************
//   64 * Function Name: r_dsadc_interrupt
//   65 * Description  : None
//   66 * Arguments    : None
//   67 * Return Value : None
//   68 ***********************************************************************************************************************/
//   69 #pragma vector = INTDSAD_vect

        SECTION `.text`:CODE:NOROOT(0)
        CALL_GRAPH_ROOT _r_dsadc_interrupt, "interrupt"
          CFI Block cfiBlock1 Using cfiCommon1
          CFI Function _r_dsadc_interrupt
        CODE
//   70 __interrupt static void r_dsadc_interrupt(void)
//   71 {
_r_dsadc_interrupt:
___interrupt_0x44:
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
//   72     /* Handling the syncronisation */
//   73     // LED_RCAL_HIGH;
//   74     r_phase.vol.sample_raw = (adc_data[1]>>6) & 0x03FF;
        MOVW      AX, N:_adc_data+2  ;; 1 cycle
        SARW      AX, 0x6            ;; 1 cycle
        AND       A, #0x3            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        AND       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      N:_r_phase+6, AX   ;; 1 cycle
//   75     y_phase.vol.sample_raw = (adc_data[2]>>6) & 0x03FF;
        MOVW      AX, N:_adc_data+4  ;; 1 cycle
        SARW      AX, 0x6            ;; 1 cycle
        AND       A, #0x3            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        AND       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      N:_y_phase+6, AX   ;; 1 cycle
//   76     b_phase.vol.sample_raw = (adc_data[3]>>6) & 0x03FF;
        MOVW      AX, N:_adc_data+6  ;; 1 cycle
        SARW      AX, 0x6            ;; 1 cycle
        AND       A, #0x3            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        AND       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      N:_b_phase+6, AX   ;; 1 cycle
//   77     r_phase.vol.sample_raw -= 565;              /* 0.8/1.45*1024*/
        MOVW      AX, N:_r_phase+6   ;; 1 cycle
        ADDW      AX, #0xFDCB        ;; 1 cycle
        MOVW      N:_r_phase+6, AX   ;; 1 cycle
//   78     y_phase.vol.sample_raw -= 565;
        MOVW      AX, N:_y_phase+6   ;; 1 cycle
        ADDW      AX, #0xFDCB        ;; 1 cycle
        MOVW      N:_y_phase+6, AX   ;; 1 cycle
//   79     b_phase.vol.sample_raw -= 565;
        MOVW      AX, N:_b_phase+6   ;; 1 cycle
        ADDW      AX, #0xFDCB        ;; 1 cycle
        MOVW      N:_b_phase+6, AX   ;; 1 cycle
//   80     r_phase.vol.sample_raw = ~r_phase.vol.sample_raw+1;
        MOVW      AX, N:_r_phase+6   ;; 1 cycle
        XOR       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        XOR       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        INCW      AX                 ;; 1 cycle
        MOVW      N:_r_phase+6, AX   ;; 1 cycle
//   81     y_phase.vol.sample_raw = ~y_phase.vol.sample_raw+1;
        MOVW      AX, N:_y_phase+6   ;; 1 cycle
        XOR       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        XOR       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        INCW      AX                 ;; 1 cycle
        MOVW      N:_y_phase+6, AX   ;; 1 cycle
//   82     b_phase.vol.sample_raw = ~b_phase.vol.sample_raw+1;
        MOVW      AX, N:_b_phase+6   ;; 1 cycle
        XOR       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        XOR       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        INCW      AX                 ;; 1 cycle
        MOVW      N:_b_phase+6, AX   ;; 1 cycle
//   83     
//   84     if(flag_metrology_ignore_90sample == 0)
        MOVW      HL, #LWRD(_flag_metrology)  ;; 1 cycle
        MOV1      CY, [HL].7         ;; 1 cycle
        BC        ??r_dsadzc1_interrupt_0  ;; 4 cycles
        ; ------------------------------------- Block: 63 cycles
//   85     {
//   86         r_phase.vol.sample_raw90 = (adc_data[5]>>6)& 0x03FF;
        MOVW      AX, N:_adc_data+10  ;; 1 cycle
        SARW      AX, 0x6            ;; 1 cycle
        AND       A, #0x3            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        AND       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      N:_r_phase+10, AX  ;; 1 cycle
//   87         y_phase.vol.sample_raw90 = (adc_data[6]>>6)& 0x03FF;
        MOVW      AX, N:_adc_data+12  ;; 1 cycle
        SARW      AX, 0x6            ;; 1 cycle
        AND       A, #0x3            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        AND       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      N:_y_phase+10, AX  ;; 1 cycle
//   88         b_phase.vol.sample_raw90 = (adc_data[7]>>6)& 0x03FF;
        MOVW      AX, N:_adc_data+14  ;; 1 cycle
        SARW      AX, 0x6            ;; 1 cycle
        AND       A, #0x3            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        AND       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      N:_b_phase+10, AX  ;; 1 cycle
//   89         r_phase.vol.sample_raw90 -= 565;
        MOVW      AX, N:_r_phase+10  ;; 1 cycle
        ADDW      AX, #0xFDCB        ;; 1 cycle
        MOVW      N:_r_phase+10, AX  ;; 1 cycle
//   90         y_phase.vol.sample_raw90 -= 565;
        MOVW      AX, N:_y_phase+10  ;; 1 cycle
        ADDW      AX, #0xFDCB        ;; 1 cycle
        MOVW      N:_y_phase+10, AX  ;; 1 cycle
//   91         b_phase.vol.sample_raw90 -= 565;
        MOVW      AX, N:_b_phase+10  ;; 1 cycle
        ADDW      AX, #0xFDCB        ;; 1 cycle
        MOVW      N:_b_phase+10, AX  ;; 1 cycle
//   92         r_phase.vol.sample_raw90 = ~r_phase.vol.sample_raw90+1;
        MOVW      AX, N:_r_phase+10  ;; 1 cycle
        XOR       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        XOR       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        INCW      AX                 ;; 1 cycle
        MOVW      N:_r_phase+10, AX  ;; 1 cycle
//   93         y_phase.vol.sample_raw90 = ~y_phase.vol.sample_raw90+1;
        MOVW      AX, N:_y_phase+10  ;; 1 cycle
        XOR       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        XOR       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        INCW      AX                 ;; 1 cycle
        MOVW      N:_y_phase+10, AX  ;; 1 cycle
//   94         b_phase.vol.sample_raw90 = ~b_phase.vol.sample_raw90+1;
        MOVW      AX, N:_b_phase+10  ;; 1 cycle
        XOR       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        XOR       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        INCW      AX                 ;; 1 cycle
        MOVW      N:_b_phase+10, AX  ;; 1 cycle
        BR        S:??r_dsadzc1_interrupt_1  ;; 3 cycles
        ; ------------------------------------- Block: 54 cycles
//   95     }
//   96     else
//   97     {
//   98         r_phase.vol.sample_raw90 = r_phase.vol.sample_raw;
??r_dsadzc1_interrupt_0:
        MOVW      AX, N:_r_phase+6   ;; 1 cycle
        MOVW      N:_r_phase+10, AX  ;; 1 cycle
//   99         y_phase.vol.sample_raw90 = y_phase.vol.sample_raw;
        MOVW      AX, N:_y_phase+6   ;; 1 cycle
        MOVW      N:_y_phase+10, AX  ;; 1 cycle
//  100         b_phase.vol.sample_raw90 = b_phase.vol.sample_raw;
        MOVW      AX, N:_b_phase+6   ;; 1 cycle
        MOVW      N:_b_phase+10, AX  ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
//  101     }
//  102     
//  103     r_phase.curr.sample_raw = DSADCR0;
??r_dsadzc1_interrupt_1:
        MOVW      AX, 0x3E0          ;; 1 cycle
        MOVW      N:_r_phase+42, AX  ;; 1 cycle
//  104     y_phase.curr.sample_raw = DSADCR1;
        MOVW      AX, 0x3E4          ;; 1 cycle
        MOVW      N:_y_phase+42, AX  ;; 1 cycle
//  105     b_phase.curr.sample_raw = DSADCR2;
        MOVW      AX, 0x3E8          ;; 1 cycle
        MOVW      N:_b_phase+42, AX  ;; 1 cycle
//  106     n_phase.curr.sample_raw = DSADCR3;
        MOVW      AX, 0x3EC          ;; 1 cycle
        MOVW      N:_n_phase+4, AX   ;; 1 cycle
//  107     
//  108     metrology_function();
          CFI FunCall _metrology_function
        CALL      _metrology_function  ;; 3 cycles
//  109     last_interrupt = 2;
        MOV       N:_last_interrupt, #0x2  ;; 1 cycle
//  110     R_DTCD0_Start();
          CFI FunCall _R_DTCD0_Start
        CALL      _R_DTCD0_Start     ;; 3 cycles
//  111     // LED_RCAL_LOW;
//  112 }
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
          CFI EndBlock cfiBlock1
        ; ------------------------------------- Block: 27 cycles
        ; ------------------------------------- Total: 150 cycles
        REQUIRE __A_DSADCR0
        REQUIRE __A_DSADCR1
        REQUIRE __A_DSADCR2
        REQUIRE __A_DSADCR3
        REQUIRE ___interrupt_tab_0x44
//  113 /***********************************************************************************************************************
//  114 * Function Name: r_dsadzc0_interrupt
//  115 * Description  : None
//  116 * Arguments    : None
//  117 * Return Value : None
//  118 ***********************************************************************************************************************/
//  119 #pragma vector = INTDSADZC0_vect

        SECTION `.text`:CODE:NOROOT(0)
        CALL_GRAPH_ROOT _r_dsadzc0_interrupt, "interrupt"
          CFI Block cfiBlock2 Using cfiCommon1
          CFI Function _r_dsadzc0_interrupt
          CFI NoCalls
        CODE
//  120 __interrupt static void r_dsadzc0_interrupt(void)
//  121 {
_r_dsadzc0_interrupt:
___interrupt_0x64:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  122     /* Start user code. Do not edit comment generated here */
//  123     last_interrupt = 3;
        MOV       N:_last_interrupt, #0x3  ;; 1 cycle
//  124     /* End user code. Do not edit comment generated here */
//  125 }
        RETI                         ;; 6 cycles
          CFI EndBlock cfiBlock2
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 7 cycles
        REQUIRE ___interrupt_tab_0x64
//  126 /***********************************************************************************************************************
//  127 * Function Name: r_dsadzc1_interrupt
//  128 * Description  : None
//  129 * Arguments    : None
//  130 * Return Value : None
//  131 ***********************************************************************************************************************/
//  132 #pragma vector = INTDSADZC1_vect

        SECTION `.text`:CODE:NOROOT(0)
        CALL_GRAPH_ROOT _r_dsadzc1_interrupt, "interrupt"
          CFI Block cfiBlock3 Using cfiCommon1
          CFI Function _r_dsadzc1_interrupt
          CFI NoCalls
        CODE
//  133 __interrupt static void r_dsadzc1_interrupt(void)
//  134 {
_r_dsadzc1_interrupt:
___interrupt_0x66:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  135     /* Start user code. Do not edit comment generated here */
//  136     last_interrupt = 4;
        MOV       N:_last_interrupt, #0x4  ;; 1 cycle
//  137     /* End user code. Do not edit comment generated here */
//  138 }
        RETI                         ;; 6 cycles
          CFI EndBlock cfiBlock3
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 7 cycles
        REQUIRE ___interrupt_tab_0x66

        SECTION `.iar_vfe_header`:DATA:NOALLOC:NOROOT(1)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
        DC32 0

        END
//  139 
//  140 /* Start user code for adding. Do not edit comment generated here */
//  141 /* End user code. Do not edit comment generated here */
// 
//   8 bytes in section .bss.noinit  (abs)
// 306 bytes in section .text
// 
// 306 bytes of CODE memory
//   0 bytes of DATA memory (+ 8 bytes shared)
//
//Errors: none
//Warnings: none
