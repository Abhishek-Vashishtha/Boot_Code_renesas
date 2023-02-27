///////////////////////////////////////////////////////////////////////////////
//
// IAR C/C++ Compiler V4.20.1.2260 for RL78               20/Dec/2020  23:13:36
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
//        BootCode\source_code\driver_files\r_cg_intp.c
//    Command line       =
//        -f C:\Users\laptop\AppData\Local\Temp\EWB315.tmp ("D:\Dheeraj\New
//        folder\0. GDEV72 - BootCode\source_code\driver_files\r_cg_intp.c"
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
//        D:\Dheeraj\New folder\0. GDEV72 - BootCode\Debug\List\r_cg_intp.s
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

        EXTERN _R_INTC_Create_UserInit

        PUBLIC _R_INTC0_Start
        PUBLIC _R_INTC0_Stop
        PUBLIC _R_INTC1_Start
        PUBLIC _R_INTC1_Stop
        PUBLIC _R_INTC2_Start
        PUBLIC _R_INTC2_Stop
        PUBLIC _R_INTC3_Start
        PUBLIC _R_INTC3_Stop
        PUBLIC _R_INTC4_Start
        PUBLIC _R_INTC4_Stop
        PUBLIC _R_INTC5_Start
        PUBLIC _R_INTC5_Stop
        PUBLIC _R_INTC6_Start
        PUBLIC _R_INTC6_Stop
        PUBLIC _R_INTC7_Start
        PUBLIC _R_INTC7_Stop
        PUBLIC _R_INTC_Create
        PUBLIC _R_INTRTCIC0_Start
        PUBLIC _R_INTRTCIC0_Stop
        PUBLIC _R_INTRTCIC1_Start
        PUBLIC _R_INTRTCIC1_Stop
        PUBLIC _R_INTRTCIC2_Start
        PUBLIC _R_INTRTCIC2_Stop
        PUBLIC __A_EGN0
        PUBLIC __A_EGP0
        PUBLIC __A_IF0
        PUBLIC __A_IF2
        PUBLIC __A_MK0
        PUBLIC __A_MK2
        PUBLIC __A_PR02
        PUBLIC __A_PR12
        
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
        
// D:\Dheeraj\New folder\0. GDEV72 - BootCode\source_code\driver_files\r_cg_intp.c
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
//   21 * File Name    : r_cg_intp.c
//   22 * Version      : Applilet4 for RL78/I1C V1.01.03.02 [16 Nov 2018]
//   23 * Device(s)    : R5F10NPJ
//   24 * Tool-Chain   : IAR Systems icc78k0r
//   25 * Description  : This file implements device driver for INTP module.
//   26 * Creation Date: 13-05-2020
//   27 ***********************************************************************************************************************/
//   28 
//   29 /***********************************************************************************************************************
//   30 Includes
//   31 ***********************************************************************************************************************/
//   32 #include "r_cg_macrodriver.h"

        ASEGN `.bss.noinit`:DATA:NOROOT,0fff38H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_EGP0
// __no_init union <unnamed>#54 volatile __sfr _A_EGP0
__A_EGP0:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0fff39H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_EGN0
// __no_init union <unnamed>#55 volatile __sfr _A_EGN0
__A_EGN0:
        DS 1

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
//   33 #include "r_cg_intp.h"
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
//   51 * Function Name: R_INTC_Create
//   52 * Description  : This function initializes INTP module.
//   53 * Arguments    : None
//   54 * Return Value : None
//   55 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock0 Using cfiCommon0
          CFI Function _R_INTC_Create
        CODE
//   56 void R_INTC_Create(void)
//   57 {
_R_INTC_Create:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//   58     PMK0 = 1U;      /* disable INTP0 interrupt */
        SET1      0xFFFE4.2          ;; 2 cycles
//   59     PIF0 = 0U;      /* clear INTP0 interrupt flag */
        CLR1      0xFFFE0.2          ;; 2 cycles
//   60     PMK1 = 1U;      /* disable INTP1 interrupt */
        SET1      0xFFFE4.3          ;; 2 cycles
//   61     PIF1 = 0U;      /* clear INTP1 interrupt flag */
        CLR1      0xFFFE0.3          ;; 2 cycles
//   62     PMK2 = 1U;      /* disable INTP2 interrupt */
        SET1      0xFFFE4.4          ;; 2 cycles
//   63     PIF2 = 0U;      /* clear INTP2 interrupt flag */
        CLR1      0xFFFE0.4          ;; 2 cycles
//   64     PMK3 = 1U;      /* disable INTP3 interrupt */
        SET1      0xFFFE4.5          ;; 2 cycles
//   65     PIF3 = 0U;      /* clear INTP3 interrupt flag */
        CLR1      0xFFFE0.5          ;; 2 cycles
//   66     PMK4 = 1U;      /* disable INTP4 interrupt */
        SET1      0xFFFE4.6          ;; 2 cycles
//   67     PIF4 = 0U;      /* clear INTP4 interrupt flag */
        CLR1      0xFFFE0.6          ;; 2 cycles
//   68     PMK5 = 1U;      /* disable INTP5 interrupt */
        SET1      0xFFFE4.7          ;; 2 cycles
//   69     PIF5 = 0U;      /* clear INTP5 interrupt flag */
        CLR1      0xFFFE0.7          ;; 2 cycles
//   70     PMK6 = 1U;      /* disable INTP6 interrupt */
        SET1      0xFFFD4.3          ;; 2 cycles
//   71     PIF6 = 0U;      /* clear INTP6 interrupt flag */
        CLR1      0xFFFD0.3          ;; 2 cycles
//   72     PMK7 = 1U;      /* disable INTP7 interrupt */
        SET1      0xFFFD4.4          ;; 2 cycles
//   73     PIF7 = 0U;      /* clear INTP7 interrupt flag */
        CLR1      0xFFFD0.4          ;; 2 cycles
//   74     RTCIMK0 = 1U;   /* disable INTRTCIC0 interrupt */
        SET1      0xFFFD4.5          ;; 2 cycles
//   75     RTCIIF0 = 0U;   /* clear INTRTCIC0 interrupt flag */
        CLR1      0xFFFD0.5          ;; 2 cycles
//   76     RTCIMK1 = 1U;   /* disable INTRTCIC1 interrupt */
        SET1      0xFFFD4.6          ;; 2 cycles
//   77     RTCIIF1 = 0U;   /* clear INTRTCIC1 interrupt flag */
        CLR1      0xFFFD0.6          ;; 2 cycles
//   78     RTCIMK2 = 1U;   /* disable INTRTCIC2 interrupt */
        SET1      0xFFFD4.7          ;; 2 cycles
//   79     RTCIIF2 = 0U;   /* clear INTRTCIC2 interrupt flag */
        CLR1      0xFFFD0.7          ;; 2 cycles
//   80     /* Set INTP6 low priority */
//   81     PPR16 = 1U;
        SET1      0xFFFDC.3          ;; 2 cycles
//   82     PPR06 = 1U;
        SET1      0xFFFD8.3          ;; 2 cycles
//   83     /* Set INTP7 low priority */
//   84     PPR17 = 1U;
        SET1      0xFFFDC.4          ;; 2 cycles
//   85     PPR07 = 1U;
        SET1      0xFFFD8.4          ;; 2 cycles
//   86     EGN0 =  _00_INTP7_EDGE_FALLING_DISABLE | _00_INTP6_EDGE_RISING_DISABLE;
        MOV       0xFFF39, #0x0      ;; 1 cycle
//   87     EGP0 = _80_INTP7_EDGE_FALLING_BOTH | _40_INTP6_EDGE_RISING_BOTH;
        MOV       0xFFF38, #0xC0     ;; 1 cycle
//   88     
//   89     //ISCLCD |= 0x02U;
//   90     /* Set INTP6 pin */
//   91     //PM4 |= 0x02U;
//   92     /* Set INTP7 pin */
//   93     //PM4 |= 0x04U;
//   94 
//   95     R_INTC_Create_UserInit();
          CFI FunCall _R_INTC_Create_UserInit
        CALL      _R_INTC_Create_UserInit  ;; 3 cycles
//   96 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock0
        ; ------------------------------------- Block: 63 cycles
        ; ------------------------------------- Total: 63 cycles
        REQUIRE __A_MK0
        REQUIRE __A_IF0
        REQUIRE __A_MK2
        REQUIRE __A_IF2
        REQUIRE __A_PR12
        REQUIRE __A_PR02
        REQUIRE __A_EGN0
        REQUIRE __A_EGP0
//   97 
//   98 /***********************************************************************************************************************
//   99 * Function Name: R_INTC0_Start
//  100 * Description  : This function clears INTP0 interrupt flag and enables interrupt.
//  101 * Arguments    : None
//  102 * Return Value : None
//  103 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock1 Using cfiCommon0
          CFI Function _R_INTC0_Start
          CFI NoCalls
        CODE
//  104 void R_INTC0_Start(void)
//  105 {
_R_INTC0_Start:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  106 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock1
        ; ------------------------------------- Block: 6 cycles
        ; ------------------------------------- Total: 6 cycles
//  107 /***********************************************************************************************************************
//  108 * Function Name: R_INTC0_Stop
//  109 * Description  : This function disables INTP0 interrupt and clears interrupt flag.
//  110 * Arguments    : None
//  111 * Return Value : None
//  112 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock2 Using cfiCommon0
          CFI Function _R_INTC0_Stop
          CFI NoCalls
        CODE
//  113 void R_INTC0_Stop(void)
//  114 {
_R_INTC0_Stop:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  115 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock2
        ; ------------------------------------- Block: 6 cycles
        ; ------------------------------------- Total: 6 cycles
//  116 /***********************************************************************************************************************
//  117 * Function Name: R_INTC1_Start
//  118 * Description  : This function clears INTP1 interrupt flag and enables interrupt.
//  119 * Arguments    : None
//  120 * Return Value : None
//  121 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock3 Using cfiCommon0
          CFI Function _R_INTC1_Start
          CFI NoCalls
        CODE
//  122 void R_INTC1_Start(void)
//  123 {
_R_INTC1_Start:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  124 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock3
        ; ------------------------------------- Block: 6 cycles
        ; ------------------------------------- Total: 6 cycles
//  125 /***********************************************************************************************************************
//  126 * Function Name: R_INTC1_Stop
//  127 * Description  : This function disables INTP1 interrupt and clears interrupt flag.
//  128 * Arguments    : None
//  129 * Return Value : None
//  130 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock4 Using cfiCommon0
          CFI Function _R_INTC1_Stop
          CFI NoCalls
        CODE
//  131 void R_INTC1_Stop(void)
//  132 {
_R_INTC1_Stop:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  133 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock4
        ; ------------------------------------- Block: 6 cycles
        ; ------------------------------------- Total: 6 cycles
//  134 /***********************************************************************************************************************
//  135 * Function Name: R_INTC2_Start
//  136 * Description  : This function clears INTP2 interrupt flag and enables interrupt.
//  137 * Arguments    : None
//  138 * Return Value : None
//  139 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock5 Using cfiCommon0
          CFI Function _R_INTC2_Start
          CFI NoCalls
        CODE
//  140 void R_INTC2_Start(void)
//  141 {
_R_INTC2_Start:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  142 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock5
        ; ------------------------------------- Block: 6 cycles
        ; ------------------------------------- Total: 6 cycles
//  143 /***********************************************************************************************************************
//  144 * Function Name: R_INTC2_Stop
//  145 * Description  : This function disables INTP2 interrupt and clears interrupt flag.
//  146 * Arguments    : None
//  147 * Return Value : None
//  148 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock6 Using cfiCommon0
          CFI Function _R_INTC2_Stop
          CFI NoCalls
        CODE
//  149 void R_INTC2_Stop(void)
//  150 {
_R_INTC2_Stop:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  151 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock6
        ; ------------------------------------- Block: 6 cycles
        ; ------------------------------------- Total: 6 cycles
//  152 /***********************************************************************************************************************
//  153 * Function Name: R_INTC3_Start
//  154 * Description  : This function clears INTP3 interrupt flag and enables interrupt.
//  155 * Arguments    : None
//  156 * Return Value : None
//  157 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock7 Using cfiCommon0
          CFI Function _R_INTC3_Start
          CFI NoCalls
        CODE
//  158 void R_INTC3_Start(void)
//  159 {
_R_INTC3_Start:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  160 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock7
        ; ------------------------------------- Block: 6 cycles
        ; ------------------------------------- Total: 6 cycles
//  161 /***********************************************************************************************************************
//  162 * Function Name: R_INTC3_Stop
//  163 * Description  : This function disables INTP3 interrupt and clears interrupt flag.
//  164 * Arguments    : None
//  165 * Return Value : None
//  166 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock8 Using cfiCommon0
          CFI Function _R_INTC3_Stop
          CFI NoCalls
        CODE
//  167 void R_INTC3_Stop(void)
//  168 {
_R_INTC3_Stop:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  169 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock8
        ; ------------------------------------- Block: 6 cycles
        ; ------------------------------------- Total: 6 cycles
//  170 /***********************************************************************************************************************
//  171 * Function Name: R_INTC4_Start
//  172 * Description  : This function clears INTP4 interrupt flag and enables interrupt.
//  173 * Arguments    : None
//  174 * Return Value : None
//  175 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock9 Using cfiCommon0
          CFI Function _R_INTC4_Start
          CFI NoCalls
        CODE
//  176 void R_INTC4_Start(void)
//  177 {
_R_INTC4_Start:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  178 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock9
        ; ------------------------------------- Block: 6 cycles
        ; ------------------------------------- Total: 6 cycles
//  179 /***********************************************************************************************************************
//  180 * Function Name: R_INTC4_Stop
//  181 * Description  : This function disables INTP4 interrupt and clears interrupt flag.
//  182 * Arguments    : None
//  183 * Return Value : None
//  184 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock10 Using cfiCommon0
          CFI Function _R_INTC4_Stop
          CFI NoCalls
        CODE
//  185 void R_INTC4_Stop(void)
//  186 {
_R_INTC4_Stop:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  187 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock10
        ; ------------------------------------- Block: 6 cycles
        ; ------------------------------------- Total: 6 cycles
//  188 /***********************************************************************************************************************
//  189 * Function Name: R_INTC5_Start
//  190 * Description  : This function clears INTP5 interrupt flag and enables interrupt.
//  191 * Arguments    : None
//  192 * Return Value : None
//  193 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock11 Using cfiCommon0
          CFI Function _R_INTC5_Start
          CFI NoCalls
        CODE
//  194 void R_INTC5_Start(void)
//  195 {
_R_INTC5_Start:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  196 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock11
        ; ------------------------------------- Block: 6 cycles
        ; ------------------------------------- Total: 6 cycles
//  197 /***********************************************************************************************************************
//  198 * Function Name: R_INTC5_Stop
//  199 * Description  : This function disables INTP5 interrupt and clears interrupt flag.
//  200 * Arguments    : None
//  201 * Return Value : None
//  202 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock12 Using cfiCommon0
          CFI Function _R_INTC5_Stop
          CFI NoCalls
        CODE
//  203 void R_INTC5_Stop(void)
//  204 {
_R_INTC5_Stop:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  205 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock12
        ; ------------------------------------- Block: 6 cycles
        ; ------------------------------------- Total: 6 cycles
//  206 /***********************************************************************************************************************
//  207 * Function Name: R_INTC6_Start
//  208 * Description  : This function clears INTP6 interrupt flag and enables interrupt.
//  209 * Arguments    : None
//  210 * Return Value : None
//  211 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock13 Using cfiCommon0
          CFI Function _R_INTC6_Start
          CFI NoCalls
        CODE
//  212 void R_INTC6_Start(void)
//  213 {
_R_INTC6_Start:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  214     PIF6 = 0U;      /* clear INTP6 interrupt flag */
        CLR1      0xFFFD0.3          ;; 2 cycles
//  215     PMK6 = 0U;      /* enable INTP6 interrupt */
        CLR1      0xFFFD4.3          ;; 2 cycles
//  216 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock13
        ; ------------------------------------- Block: 10 cycles
        ; ------------------------------------- Total: 10 cycles
        REQUIRE __A_IF2
        REQUIRE __A_MK2
//  217 /***********************************************************************************************************************
//  218 * Function Name: R_INTC6_Stop
//  219 * Description  : This function disables INTP6 interrupt and clears interrupt flag.
//  220 * Arguments    : None
//  221 * Return Value : None
//  222 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock14 Using cfiCommon0
          CFI Function _R_INTC6_Stop
          CFI NoCalls
        CODE
//  223 void R_INTC6_Stop(void)
//  224 {
_R_INTC6_Stop:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  225     PMK6 = 1U;      /* disable INTP6 interrupt */
        SET1      0xFFFD4.3          ;; 2 cycles
//  226     PIF6 = 0U;      /* clear INTP6 interrupt flag */
        CLR1      0xFFFD0.3          ;; 2 cycles
//  227 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock14
        ; ------------------------------------- Block: 10 cycles
        ; ------------------------------------- Total: 10 cycles
        REQUIRE __A_MK2
        REQUIRE __A_IF2
//  228 /***********************************************************************************************************************
//  229 * Function Name: R_INTC7_Start
//  230 * Description  : This function clears INTP7 interrupt flag and enables interrupt.
//  231 * Arguments    : None
//  232 * Return Value : None
//  233 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock15 Using cfiCommon0
          CFI Function _R_INTC7_Start
          CFI NoCalls
        CODE
//  234 void R_INTC7_Start(void)
//  235 {
_R_INTC7_Start:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  236     PIF7 = 0U;      /* clear INTP7 interrupt flag */
        CLR1      0xFFFD0.4          ;; 2 cycles
//  237     PMK7 = 0U;      /* enable INTP7 interrupt */
        CLR1      0xFFFD4.4          ;; 2 cycles
//  238 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock15
        ; ------------------------------------- Block: 10 cycles
        ; ------------------------------------- Total: 10 cycles
        REQUIRE __A_IF2
        REQUIRE __A_MK2
//  239 /***********************************************************************************************************************
//  240 * Function Name: R_INTC7_Stop
//  241 * Description  : This function disables INTP7 interrupt and clears interrupt flag.
//  242 * Arguments    : None
//  243 * Return Value : None
//  244 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock16 Using cfiCommon0
          CFI Function _R_INTC7_Stop
          CFI NoCalls
        CODE
//  245 void R_INTC7_Stop(void)
//  246 {
_R_INTC7_Stop:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  247     PMK7 = 1U;      /* disable INTP7 interrupt */
        SET1      0xFFFD4.4          ;; 2 cycles
//  248     PIF7 = 0U;      /* clear INTP7 interrupt flag */
        CLR1      0xFFFD0.4          ;; 2 cycles
//  249 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock16
        ; ------------------------------------- Block: 10 cycles
        ; ------------------------------------- Total: 10 cycles
        REQUIRE __A_MK2
        REQUIRE __A_IF2
//  250 /***********************************************************************************************************************
//  251 * Function Name: R_INTRTCIC0_Start
//  252 * Description  : This function clears INTRTCIC0 interrupt flag and enables interrupt.
//  253 * Arguments    : None
//  254 * Return Value : None
//  255 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock17 Using cfiCommon0
          CFI Function _R_INTRTCIC0_Start
          CFI NoCalls
        CODE
//  256 void R_INTRTCIC0_Start(void)
//  257 {
_R_INTRTCIC0_Start:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  258 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock17
        ; ------------------------------------- Block: 6 cycles
        ; ------------------------------------- Total: 6 cycles
//  259 /***********************************************************************************************************************
//  260 * Function Name: R_INTRTCIC0_Stop
//  261 * Description  : This function disables INTRTCIC0 interrupt and clears interrupt flag.
//  262 * Arguments    : None
//  263 * Return Value : None
//  264 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock18 Using cfiCommon0
          CFI Function _R_INTRTCIC0_Stop
          CFI NoCalls
        CODE
//  265 void R_INTRTCIC0_Stop(void)
//  266 {
_R_INTRTCIC0_Stop:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  267 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock18
        ; ------------------------------------- Block: 6 cycles
        ; ------------------------------------- Total: 6 cycles
//  268 /***********************************************************************************************************************
//  269 * Function Name: R_INTRTCIC1_Start
//  270 * Description  : This function clears INTRTCIC1 interrupt flag and enables interrupt.
//  271 * Arguments    : None
//  272 * Return Value : None
//  273 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock19 Using cfiCommon0
          CFI Function _R_INTRTCIC1_Start
          CFI NoCalls
        CODE
//  274 void R_INTRTCIC1_Start(void)
//  275 {
_R_INTRTCIC1_Start:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  276 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock19
        ; ------------------------------------- Block: 6 cycles
        ; ------------------------------------- Total: 6 cycles
//  277 /***********************************************************************************************************************
//  278 * Function Name: R_INTRTCIC1_Stop
//  279 * Description  : This function disables INTRTCIC1 interrupt and clears interrupt flag.
//  280 * Arguments    : None
//  281 * Return Value : None
//  282 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock20 Using cfiCommon0
          CFI Function _R_INTRTCIC1_Stop
          CFI NoCalls
        CODE
//  283 void R_INTRTCIC1_Stop(void)
//  284 {
_R_INTRTCIC1_Stop:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  285 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock20
        ; ------------------------------------- Block: 6 cycles
        ; ------------------------------------- Total: 6 cycles
//  286 /***********************************************************************************************************************
//  287 * Function Name: R_INTRTCIC2_Start
//  288 * Description  : This function clears INTRTCIC2 interrupt flag and enables interrupt.
//  289 * Arguments    : None
//  290 * Return Value : None
//  291 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock21 Using cfiCommon0
          CFI Function _R_INTRTCIC2_Start
          CFI NoCalls
        CODE
//  292 void R_INTRTCIC2_Start(void)
//  293 {
_R_INTRTCIC2_Start:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  294 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock21
        ; ------------------------------------- Block: 6 cycles
        ; ------------------------------------- Total: 6 cycles
//  295 /***********************************************************************************************************************
//  296 * Function Name: R_INTRTCIC2_Stop
//  297 * Description  : This function disables INTRTCIC2 interrupt and clears interrupt flag.
//  298 * Arguments    : None
//  299 * Return Value : None
//  300 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock22 Using cfiCommon0
          CFI Function _R_INTRTCIC2_Stop
          CFI NoCalls
        CODE
//  301 void R_INTRTCIC2_Stop(void)
//  302 {
_R_INTRTCIC2_Stop:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  303 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock22
        ; ------------------------------------- Block: 6 cycles
        ; ------------------------------------- Total: 6 cycles

        SECTION `.iar_vfe_header`:DATA:NOALLOC:NOROOT(1)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
        DC32 0

        END
//  304 
//  305 /* Start user code for adding. Do not edit comment generated here */
//  306 /* End user code. Do not edit comment generated here */
// 
//  14 bytes in section .bss.noinit  (abs)
// 134 bytes in section .text
// 
// 134 bytes of CODE memory
//   0 bytes of DATA memory (+ 14 bytes shared)
//
//Errors: none
//Warnings: none
