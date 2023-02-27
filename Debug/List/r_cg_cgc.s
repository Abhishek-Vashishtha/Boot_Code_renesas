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
//        BootCode\source_code\driver_files\r_cg_cgc.c
//    Command line       =
//        -f C:\Users\17054\AppData\Local\Temp\EWDBB2.tmp ("D:\Software
//        code\Projects Software\Projects new\0. Non AMI\SUGAM Based\0. GDEV72
//        - BootCode-20210104T103109Z-001\0. GDEV72 -
//        BootCode\source_code\driver_files\r_cg_cgc.c" --core s3 --code_model
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
//        BootCode\Debug\List\r_cg_cgc.s
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

        EXTERN ?UI_RSH_L02
        EXTERN _delay_ms

        PUBLIC _R_CGC_Create
        PUBLIC _R_CGC_Set_ClockMode
        PUBLIC _R_CGC_Set_LPMode
        PUBLIC _R_CGC_Set_LSMode
        PUBLIC __A_CKC
        PUBLIC __A_CKSEL
        PUBLIC __A_CMC
        PUBLIC __A_CSC
        PUBLIC __A_DSCCTL
        PUBLIC __A_FLMODE
        PUBLIC __A_FLMWRP
        PUBLIC __A_MCKC
        PUBLIC __A_OSMC
        PUBLIC __A_OSTC
        PUBLIC __A_OSTS
        PUBLIC __A_PCKC
        PUBLIC __A_PER2
        PUBLIC __A_PMMC
        PUBLIC __A_SCMC
        PUBLIC __A_SCSC
        
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
        
// D:\Software code\Projects Software\Projects new\0. Non AMI\SUGAM Based\0. GDEV72 - BootCode-20210104T103109Z-001\0. GDEV72 - BootCode\source_code\driver_files\r_cg_cgc.c
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
//   21 * File Name    : r_cg_cgc.c
//   22 * Version      : Applilet4 for RL78/I1C V1.01.02.04 [24 May 2018]
//   23 * Device(s)    : R5F10NPJ
//   24 * Tool-Chain   : IAR Systems icc78k0r
//   25 * Description  : This file implements device driver for CGC module.
//   26 * Creation Date: 12/14/2019
//   27 ***********************************************************************************************************************/
//   28 
//   29 /***********************************************************************************************************************
//   30 Includes
//   31 ***********************************************************************************************************************/
//   32 #include "r_cg_macrodriver.h"

        ASEGN `.bss.noinit`:DATA:NOROOT,0fffa0H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_CMC
// __no_init union <unnamed>#92 volatile __sfr __no_bit_access _A_CMC
__A_CMC:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0fffa1H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_CSC
// __no_init union <unnamed>#93 volatile __sfr _A_CSC
__A_CSC:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0fffa2H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_OSTC
// __no_init union <unnamed>#95 const volatile __sfr _A_OSTC
__A_OSTC:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0fffa3H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_OSTS
// __no_init union <unnamed>#96 volatile __sfr __no_bit_access _A_OSTS
__A_OSTS:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0fffa4H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_CKC
// __no_init union <unnamed>#97 volatile __sfr _A_CKC
__A_CKC:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0fffa7H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_CKSEL
// __no_init union <unnamed>#103 volatile __sfr _A_CKSEL
__A_CKSEL:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0098H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_PCKC
// __no_init union <unnamed>#263 volatile _A_PCKC
__A_PCKC:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f00aaH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_FLMODE
// __no_init union <unnamed>#267 volatile _A_FLMODE
__A_FLMODE:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f00abH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_FLMWRP
// __no_init union <unnamed>#269 volatile _A_FLMWRP
__A_FLMWRP:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f00f3H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_OSMC
// __no_init union <unnamed>#280 volatile _A_OSMC
__A_OSMC:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f00f8H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_PMMC
// __no_init union <unnamed>#284 volatile _A_PMMC
__A_PMMC:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f00fcH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_PER2
// __no_init union <unnamed>#291 volatile _A_PER2
__A_PER2:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f02e5H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_DSCCTL
// __no_init union <unnamed>#513 volatile _A_DSCCTL
__A_DSCCTL:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f02e6H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_MCKC
// __no_init union <unnamed>#514 volatile _A_MCKC
__A_MCKC:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0384H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_SCMC
// __no_init union <unnamed>#568 volatile __no_bit_access _A_SCMC
__A_SCMC:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0386H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_SCSC
// __no_init union <unnamed>#569 volatile _A_SCSC
__A_SCSC:
        DS 1
//   33 #include "r_cg_cgc.h"
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
//   50 * Function Name: R_CGC_Create
//   51 * Description  : This function initializes the clock generator.
//   52 * Arguments    : None
//   53 * Return Value : None
//   54 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock0 Using cfiCommon0
          CFI Function _R_CGC_Create
        CODE
//   55 void R_CGC_Create(void)
//   56 {
_R_CGC_Create:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 4
        SUBW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
//   57   volatile uint32_t w_count;
//   58   
//   59   
//   60   /******************************************************************************************************************
//   61   Please Refer Data Sheet RL78/I1c Rev 1.00 May 2016
//   62   * There are Five Clock Source option available Three are On chip, Two are External  
//   63   * For CPU Clock supply We are Using On chip Clock oscillator with a Frequency of 24 Mhz
//   64   * For RTC and Other Modules We are Using Subsystem Clock Oscillator Externally Supplied frequncy 32.768 Khz
//   65   * It Should be Kept in notice that DSADC is supplied 12 Mhz clock with division by 2 of On chip clock regardless 
//   66   of clock supplied to CPU i.e 24Mhz 
//   67   ******************************************************************************************************************/
//   68   
//   69   /* Set fIM : middle-speed on-chip oscillator stopped, not using*/
//   70   MIOEN = 0U;     
        CLR1      0xFFFA1.1          ;; 2 cycles
//   71   
//   72   /* Change the waiting time according to the system */
//   73   for (w_count = 0U; w_count <= (CGC_FIMWAITTIME); w_count++)
        MOVW      AX, SP             ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        ; ------------------------------------- Block: 9 cycles
??R_CGC_Create_0:
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x9           ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??R_CGC_Create_1:
        BNC       ??R_CGC_Set_LSMode_0  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
//   74   {
//   75     NOP();
        NOP                          ;; 1 cycle
//   76   }
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        ADDW      AX, #0x1           ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      HL, SP             ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        BR        S:??R_CGC_Create_0  ;; 3 cycles
        ; ------------------------------------- Block: 21 cycles
//   77   
//   78   /* Set fPLL: On chip Clock Division through PLL not used */
//   79   PCKC = _00_CGC_fIH_STOP;
??R_CGC_Set_LSMode_0:
        MOV       0x98, #0x0         ;; 1 cycle
//   80   DSCCTL = _00_CGC_PLL_STOP;
        MOV       0x2E5, #0x0        ;; 1 cycle
//   81   
//   82   /* Change the waiting time according to the system */
//   83   for (w_count = 0U; w_count <= (CGC_FPLLWAITTIME); w_count++)
        MOVW      AX, SP             ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        ; ------------------------------------- Block: 8 cycles
??R_CGC_Create_2:
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x51          ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??R_CGC_Create_3:
        BNC       ??R_CGC_Set_LSMode_1  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
//   84   {
//   85     NOP();
        NOP                          ;; 1 cycle
//   86   }
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        ADDW      AX, #0x1           ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      HL, SP             ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        BR        S:??R_CGC_Create_2  ;; 3 cycles
        ; ------------------------------------- Block: 21 cycles
//   87   
//   88   /* FOCO clock i.e Clock selected between fih (onchip clock) & Fim (Middle speed) will be used */
//   89   MCKC = _00_CGC_fOCO_SELECTED;
??R_CGC_Set_LSMode_1:
        MOV       0x2E6, #0x0        ;; 1 cycle
//   90   
//   91   /* Set fMX : The external crystal clock is diabled */
//   92   CMC = _00_CGC_HISYS_PORT | _00_CGC_SYSOSC_UNDER10M;
        MOV       0xFFFA0, #0x0      ;; 1 cycle
//   93   MSTOP = 1U;     /* X1 oscillator/external clock stopped */
        SET1      0xFFFA1.7          ;; 2 cycles
//   94   
//   95   /* Set fSUB */
//   96   SELLOSC = 0U;   /* sub clock (fSX) */
        CLR1      0xFFFA7.0          ;; 2 cycles
//   97   
//   98   /* Set fSX */
//   99   OSMC = _00_CGC_CLK_ENABLE | _00_CGC_IT_CLK_fSX_CLK;
        MOV       0xF3, #0x0         ;; 1 cycle
//  100   
//  101   VRTCEN = 1U;    /* enables input clock supply */
        SET1      0xF00FC.0          ;; 2 cycles
//  102   
//  103   delay_ms(10);
        MOVW      AX, #0xA           ;; 1 cycle
          CFI FunCall _delay_ms
        CALL      _delay_ms          ;; 3 cycles
//  104   
//  105   SCMC = _10_CGC_SUB_OSC | _02_CGC_NORMAL_OSCILLATION;
        MOV       0x384, #0x12       ;; 1 cycle
//  106   
//  107   XTSTOP = 0U;    /* XT1 oscillator operating */
        CLR1      0xF0386.6          ;; 2 cycles
//  108   
//  109   CMC &= (uint8_t)~_10_CGC_SYSOSC_PERMITTED;
        MOV       A, 0xFFFA0         ;; 1 cycle
        AND       A, #0xEF           ;; 1 cycle
        MOV       0xFFFA0, A         ;; 1 cycle
//  110   
//  111   XT1SELDIS = 0U; /* Enables clock supply to CPU for On chip frequency correction */
        CLR1      0xFFFA1.6          ;; 2 cycles
//  112   
//  113   VRTCEN = 0U;    /* stops input clock supply */
        CLR1      0xF00FC.0          ;; 2 cycles
//  114 }
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock0
        ; ------------------------------------- Block: 30 cycles
        ; ------------------------------------- Total: 113 cycles
        REQUIRE __A_CSC
        REQUIRE __A_PCKC
        REQUIRE __A_DSCCTL
        REQUIRE __A_MCKC
        REQUIRE __A_CMC
        REQUIRE __A_CKSEL
        REQUIRE __A_OSMC
        REQUIRE __A_PER2
        REQUIRE __A_SCMC
        REQUIRE __A_SCSC
//  115 
//  116 /***********************************************************************************************************************
//  117 * Function Name: R_CGC_Set_ClockMode
//  118 * Description  : This function changes clock generator operation mode.
//  119 * Arguments    : mode -
//  120 *                    clock generator operation mode
//  121 * Return Value : status -
//  122 *                    MD_OK, MD_ARGERROR, MD_ERROR1, MD_ERROR2, MD_ERROR3, MD_ERROR4, MD_ERROR5, MD_ERROR6 or MD_ERROR7
//  123 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock1 Using cfiCommon0
          CFI Function _R_CGC_Set_ClockMode
        CODE
//  124 MD_STATUS R_CGC_Set_ClockMode(clock_mode_t mode)
//  125 {
_R_CGC_Set_ClockMode:
        ; * Stack frame (at entry) *
        ; Param size: 0
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+6
        ; Auto size: 10
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+14
//  126   MD_STATUS         status = MD_OK;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [SP+0x04], AX      ;; 1 cycle
//  127   clock_mode_t      old_mode;
//  128   uint8_t           temp_stab_set;
//  129   uint8_t           temp_stab_wait;
//  130   volatile uint32_t w_count;
//  131   
//  132   if (1U == CLS)             
        MOVW      HL, #0xFFA4        ;; 1 cycle
        MOV1      CY, [HL].7         ;; 1 cycle
        BNC       ??R_CGC_Set_LSMode_2  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
//  133   {
//  134     if (1U == SELLOSC)
        MOVW      HL, #0xFFA7        ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??R_CGC_Set_LSMode_3  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  135     {
//  136       old_mode = fILCLK;
        MOV       A, #0x7            ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        BR        S:??R_CGC_Set_LSMode_4  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  137     }
//  138     else
//  139     {
//  140       if ((SCMC & _30_CGC_SUB_PIN) == _10_CGC_SUB_OSC)
??R_CGC_Set_LSMode_3:
        MOV       A, 0x384           ;; 1 cycle
        AND       A, #0x30           ;; 1 cycle
        CMP       A, #0x10           ;; 1 cycle
        BNZ       ??R_CGC_Set_LSMode_5  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  141       {
//  142         old_mode = SUBXT1CLK;
        MOV       A, #0x5            ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        BR        S:??R_CGC_Set_LSMode_4  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  143       }
//  144       else if ((SCMC & _30_CGC_SUB_PIN) == _30_CGC_SUB_EXT) 
??R_CGC_Set_LSMode_5:
        MOV       A, 0x384           ;; 1 cycle
        AND       A, #0x30           ;; 1 cycle
        CMP       A, #0x30           ;; 1 cycle
        BNZ       ??R_CGC_Set_LSMode_4  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  145       {
//  146         old_mode = SUBEXTCLK;
        MOV       A, #0x6            ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        BR        S:??R_CGC_Set_LSMode_4  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  147       }
//  148       else
//  149       {
//  150         /* Not run */
//  151       }
//  152     }
//  153   }
//  154   else
//  155   {
//  156     if (1U == MCS)
??R_CGC_Set_LSMode_2:
        MOVW      HL, #0xFFA4        ;; 1 cycle
        MOV1      CY, [HL].5         ;; 1 cycle
        BNC       ??R_CGC_Set_LSMode_6  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  157     {
//  158       if ((CMC & _C0_CGC_HISYS_PIN) == _C0_CGC_HISYS_EXT) 
        MOV       A, 0xFFFA0         ;; 1 cycle
        AND       A, #0xC0           ;; 1 cycle
        CMP       A, #0xC0           ;; 1 cycle
        BNZ       ??R_CGC_Set_LSMode_7  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  159       {
//  160         old_mode = SYSEXTCLK;
        MOV       A, #0x3            ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        BR        S:??R_CGC_Set_LSMode_4  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  161       }
//  162       else if ((CMC & _C0_CGC_HISYS_PIN) == _40_CGC_HISYS_OSC)
??R_CGC_Set_LSMode_7:
        MOV       A, 0xFFFA0         ;; 1 cycle
        AND       A, #0xC0           ;; 1 cycle
        CMP       A, #0x40           ;; 1 cycle
        BNZ       ??R_CGC_Set_LSMode_4  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  163       {
//  164         old_mode = SYSX1CLK;
        MOV       A, #0x2            ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        BR        S:??R_CGC_Set_LSMode_4  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  165       }
//  166       else
//  167       {
//  168         /* Not run */
//  169       }
//  170     }
//  171     else
//  172     {
//  173       if (_80_CGC_fPLL_STATE == (MCKC & _80_CGC_fPLL_STATE))
??R_CGC_Set_LSMode_6:
        MOV       A, 0x2E6           ;; 1 cycle
        AND       A, #0x80           ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        BZ        ??R_CGC_Set_LSMode_8  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  174       {
//  175         old_mode = PLLCLK;
        MOV       A, #0x4            ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        BR        S:??R_CGC_Set_LSMode_4  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  176       }
//  177       else
//  178       {
//  179         if (1U == MCS1)
??R_CGC_Set_LSMode_8:
        MOVW      HL, #0xFFA4        ;; 1 cycle
        MOV1      CY, [HL].1         ;; 1 cycle
        CLRB      A                  ;; 1 cycle
        ROLC      A, 0x1             ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        ; ------------------------------------- Block: 5 cycles
//  180         {
//  181           old_mode = MIOCLK;
//  182         }
//  183         else
//  184         {
//  185           old_mode = HIOCLK;
//  186         }
//  187       }
//  188     }
//  189   }
//  190   
//  191   if (mode != old_mode)
??R_CGC_Set_LSMode_4:
        MOV       A, [SP+0x09]       ;; 1 cycle
        CMP       A, D               ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??R_CGC_Set_LSMode_9  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  192   {
//  193     switch (mode)
        MOV       A, [SP+0x09]       ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        BZ        ??R_CGC_Set_LSMode_10  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        DEC       A                  ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??R_CGC_Set_LSMode_11  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        DEC       A                  ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??R_CGC_Set_LSMode_12  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        DEC       A                  ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??R_CGC_Set_LSMode_13  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        DEC       A                  ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??R_CGC_Set_LSMode_14  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        DEC       A                  ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??R_CGC_Set_LSMode_15  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        DEC       A                  ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??R_CGC_Set_LSMode_16  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        DEC       A                  ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??R_CGC_Set_LSMode_17  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        BR        N:??R_CGC_Set_LSMode_18  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
//  194     {
//  195     case HIOCLK:
//  196       
//  197       if (old_mode == PLLCLK)
??R_CGC_Set_LSMode_10:
        XCH       A, D               ;; 1 cycle
        CMP       A, #0x4            ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        BNZ       ??R_CGC_Set_LSMode_19  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  198       {
//  199         MCKC &= (uint8_t)~_01_CGC_fPLL_SELECTED;
        CLR1      0xF02E6.0          ;; 2 cycles
//  200         
//  201         /* Change the waiting time according to the system */	
//  202         for (w_count = 0U; w_count <= CGC_CKSTRWAITTIME; w_count++)	
        MOVW      AX, SP             ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        ; ------------------------------------- Block: 8 cycles
??R_CGC_Set_ClockMode_0:
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x2           ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??R_CGC_Set_ClockMode_1:
        BNC       ??R_CGC_Set_LSMode_20  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
//  203         {	
//  204           NOP();	
        NOP                          ;; 1 cycle
//  205         }	
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        ADDW      AX, #0x1           ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      HL, SP             ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        BR        S:??R_CGC_Set_ClockMode_0  ;; 3 cycles
        ; ------------------------------------- Block: 21 cycles
//  206         
//  207         DSCCTL &= (uint8_t)~_01_CGC_PLL_OUTPUT;
??R_CGC_Set_LSMode_20:
        CLR1      0xF02E5.0          ;; 2 cycles
//  208         PCKC &= (uint8_t)~_02_CGC_fIH_ENABLE;
        CLR1      0xF0098.1          ;; 2 cycles
        BR        S:??R_CGC_Set_LSMode_21  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
//  209       }
//  210       else
//  211       {         
//  212         if (1U == HIOSTOP)
??R_CGC_Set_LSMode_19:
        MOVW      HL, #0xFFA1        ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??R_CGC_Set_LSMode_21  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  213         {
//  214           HIOSTOP = 0U;   /* high-speed on-chip oscillator operating */
        CLR1      0xFFFA1.0          ;; 2 cycles
//  215           
//  216           /* Change the waiting time according to the system */
//  217           for (w_count = 0U; w_count <= CGC_FIHWAITTIME; w_count++)
        MOVW      AX, SP             ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        ; ------------------------------------- Block: 8 cycles
??R_CGC_Set_ClockMode_2:
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x83          ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??R_CGC_Set_ClockMode_3:
        BNC       ??R_CGC_Set_LSMode_21  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
//  218           {
//  219             NOP();
        NOP                          ;; 1 cycle
//  220           }
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        ADDW      AX, #0x1           ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      HL, SP             ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        BR        S:??R_CGC_Set_ClockMode_2  ;; 3 cycles
        ; ------------------------------------- Block: 21 cycles
//  221         }
//  222       }
//  223       
//  224       CSS = 0U;       /* main system clock (fMAIN) */
??R_CGC_Set_LSMode_21:
        CLR1      0xFFFA4.6          ;; 2 cycles
//  225       MCM0 = 0U;      /* selects the main on-chip oscillator clock (fOCO) or PLL clock (fPLL) as the main system clock (fMAIN) */
        CLR1      0xFFFA4.4          ;; 2 cycles
//  226       MCM1 = 0U;      /* high-speed on-chip oscillator clock */
        CLR1      0xFFFA4.0          ;; 2 cycles
//  227       
//  228       break;
        BR        N:??R_CGC_Set_LSMode_9  ;; 3 cycles
        ; ------------------------------------- Block: 9 cycles
//  229       
//  230     case MIOCLK:
//  231       
//  232       if (PLLCLK == old_mode)
??R_CGC_Set_LSMode_11:
        XCH       A, D               ;; 1 cycle
        CMP       A, #0x4            ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        BNZ       ??R_CGC_Set_LSMode_22  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  233       {
//  234         status = MD_ERROR1;
        MOVW      AX, #0x82          ;; 1 cycle
        MOVW      [SP+0x04], AX      ;; 1 cycle
        BR        S:??R_CGC_Set_LSMode_23  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  235       }
//  236       else
//  237       {
//  238         if (0U == MIOEN)
??R_CGC_Set_LSMode_22:
        MOVW      HL, #0xFFA1        ;; 1 cycle
        MOV1      CY, [HL].1         ;; 1 cycle
        BC        ??R_CGC_Set_LSMode_24  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  239         {
//  240           MIOEN = 0U;     /* middle-speed on-chip oscillator stopped */
        CLR1      0xFFFA1.1          ;; 2 cycles
//  241           
//  242           
//  243           /* Change the waiting time according to the system */
//  244           for (w_count = 0U; w_count <= CGC_FIMWAITTIME; w_count++)
        MOVW      AX, SP             ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        ; ------------------------------------- Block: 8 cycles
??R_CGC_Set_ClockMode_4:
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x9           ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??R_CGC_Set_ClockMode_5:
        BNC       ??R_CGC_Set_LSMode_24  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
//  245           {
//  246             NOP();
        NOP                          ;; 1 cycle
//  247           }
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        ADDW      AX, #0x1           ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      HL, SP             ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        BR        S:??R_CGC_Set_ClockMode_4  ;; 3 cycles
        ; ------------------------------------- Block: 21 cycles
//  248         }
//  249         
//  250         CSS = 0U;       /* main system clock (fMAIN) */
??R_CGC_Set_LSMode_24:
        CLR1      0xFFFA4.6          ;; 2 cycles
//  251         MCM0 = 0U;      /* selects the main on-chip oscillator clock (fOCO) or PLL clock (fPLL) as the main system clock (fMAIN) */
        CLR1      0xFFFA4.4          ;; 2 cycles
//  252         MCM1 = 1U;      /* middle-speed on-chip oscillator clock */
        SET1      0xFFFA4.0          ;; 2 cycles
        ; ------------------------------------- Block: 6 cycles
//  253       }   
//  254       
//  255       break;
??R_CGC_Set_LSMode_23:
        BR        N:??R_CGC_Set_LSMode_9  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
//  256       
//  257     case PLLCLK:
//  258       
//  259       if (HIOCLK != old_mode)
??R_CGC_Set_LSMode_14:
        XCH       A, D               ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        BZ        ??R_CGC_Set_LSMode_25  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  260       {
//  261         status = MD_ERROR2;
        MOVW      AX, #0x83          ;; 1 cycle
        MOVW      [SP+0x04], AX      ;; 1 cycle
        BR        S:??R_CGC_Set_LSMode_26  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  262       }
//  263       else
//  264       {
//  265         DSCCTL = _0C_CGC_CLOCK_fPLL | _02_CGC_MULTIPLICATION_fPLL;
??R_CGC_Set_LSMode_25:
        MOV       0x2E5, #0xE        ;; 1 cycle
//  266         PCKC = _02_CGC_fIH_ENABLE;
        MOV       0x98, #0x2         ;; 1 cycle
//  267         DSCCTL |= _01_CGC_PLL_OUTPUT;
        SET1      0xF02E5.0          ;; 2 cycles
//  268         
//  269         /* Change the waiting time according to the system */
//  270         for (w_count = 0U; w_count <= CGC_FPLLWAITTIME; w_count++)
        MOVW      AX, SP             ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        ; ------------------------------------- Block: 10 cycles
??R_CGC_Set_ClockMode_6:
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x51          ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??R_CGC_Set_ClockMode_7:
        BNC       ??R_CGC_Set_LSMode_27  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
//  271         {
//  272           NOP();
        NOP                          ;; 1 cycle
//  273         }
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        ADDW      AX, #0x1           ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      HL, SP             ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        BR        S:??R_CGC_Set_ClockMode_6  ;; 3 cycles
        ; ------------------------------------- Block: 21 cycles
//  274         
//  275         MCKC |= _01_CGC_fPLL_SELECTED;
??R_CGC_Set_LSMode_27:
        SET1      0xF02E6.0          ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
//  276         
//  277         while ((MCKC & _80_CGC_fPLL_STATE) != _80_CGC_fPLL_STATE)
??R_CGC_Set_ClockMode_8:
        MOV       A, 0x2E6           ;; 1 cycle
        AND       A, #0x80           ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        BZ        ??R_CGC_Set_ClockMode_8  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  278         {
//  279           ;
//  280         }
//  281       }
//  282       
//  283       break;
??R_CGC_Set_LSMode_26:
        BR        N:??R_CGC_Set_LSMode_9  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
//  284       
//  285     case SYSX1CLK:
//  286       
//  287       if ((SYSEXTCLK == old_mode) || (PLLCLK == old_mode) || ((CMC & _C0_CGC_HISYS_PIN) != _40_CGC_HISYS_OSC)) 
??R_CGC_Set_LSMode_12:
        XCH       A, D               ;; 1 cycle
        CMP       A, #0x3            ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        BZ        ??R_CGC_Set_LSMode_28  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        XCH       A, D               ;; 1 cycle
        CMP       A, #0x4            ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        BZ        ??R_CGC_Set_LSMode_28  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOV       A, 0xFFFA0         ;; 1 cycle
        AND       A, #0xC0           ;; 1 cycle
        CMP       A, #0x40           ;; 1 cycle
        BZ        ??R_CGC_Set_LSMode_29  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  288       {
//  289         status = MD_ERROR3;
??R_CGC_Set_LSMode_28:
        MOVW      AX, #0x84          ;; 1 cycle
        MOVW      [SP+0x04], AX      ;; 1 cycle
        BR        S:??R_CGC_Set_LSMode_30  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  290       }
//  291       else
//  292       {
//  293         if (1U == MSTOP)
??R_CGC_Set_LSMode_29:
        MOVW      HL, #0xFFA1        ;; 1 cycle
        MOV1      CY, [HL].7         ;; 1 cycle
        BNC       ??R_CGC_Set_LSMode_31  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  294         {
//  295           MSTOP = 0U;     /* X1 oscillator/external clock operating */
        CLR1      0xFFFA1.7          ;; 2 cycles
//  296           temp_stab_set = (uint8_t)~(0x7FU >> OSTS);
        MOV       A, 0xFFFA3         ;; 1 cycle
        MOV       C, A               ;; 1 cycle
        MOVW      AX, #0x7F          ;; 1 cycle
          CFI FunCall ?UI_RSH_L02
        CALL      N:?UI_RSH_L02      ;; 3 cycles
        MOV       A, X               ;; 1 cycle
        XOR       A, #0xFF           ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        ; ------------------------------------- Block: 11 cycles
//  297           
//  298           do
//  299           {
//  300             temp_stab_wait = OSTC;
??R_CGC_Set_ClockMode_9:
        MOV       A, 0xFFFA2         ;; 1 cycle
//  301             temp_stab_wait &= temp_stab_set;
        AND       A, E               ;; 1 cycle
        MOV       [SP+0x06], A       ;; 1 cycle
//  302           }
//  303           while (temp_stab_wait != temp_stab_set);
        MOV       A, [SP+0x06]       ;; 1 cycle
        CMP       A, E               ;; 1 cycle
        BNZ       ??R_CGC_Set_ClockMode_9  ;; 4 cycles
        ; ------------------------------------- Block: 9 cycles
//  304         }
//  305         
//  306         CSS = 0U;       /* main system clock (fMAIN) */
??R_CGC_Set_LSMode_31:
        CLR1      0xFFFA4.6          ;; 2 cycles
//  307         MCM0 = 1U;      /* selects the high-speed system clock (fMX) as the main system clock (fMAIN) */
        SET1      0xFFFA4.4          ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
//  308       }
//  309       
//  310       break;
??R_CGC_Set_LSMode_30:
        BR        N:??R_CGC_Set_LSMode_9  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
//  311       
//  312     case SYSEXTCLK:
//  313       
//  314       if ((SYSX1CLK == old_mode) || (PLLCLK == old_mode) || ((CMC & _C0_CGC_HISYS_PIN) != _C0_CGC_HISYS_EXT)) 
??R_CGC_Set_LSMode_13:
        XCH       A, D               ;; 1 cycle
        CMP       A, #0x2            ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        BZ        ??R_CGC_Set_LSMode_32  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        XCH       A, D               ;; 1 cycle
        CMP       A, #0x4            ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        BZ        ??R_CGC_Set_LSMode_32  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOV       A, 0xFFFA0         ;; 1 cycle
        AND       A, #0xC0           ;; 1 cycle
        CMP       A, #0xC0           ;; 1 cycle
        BZ        ??R_CGC_Set_LSMode_33  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  315       {
//  316         status = MD_ERROR4;
??R_CGC_Set_LSMode_32:
        MOVW      AX, #0x85          ;; 1 cycle
        MOVW      [SP+0x04], AX      ;; 1 cycle
        BR        S:??R_CGC_Set_LSMode_34  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  317       }
//  318       else
//  319       {
//  320         if (1U == MSTOP)
??R_CGC_Set_LSMode_33:
        MOVW      HL, #0xFFA1        ;; 1 cycle
        MOV1      CY, [HL].7         ;; 1 cycle
        SKNC                         ;; 1 cycle
        ; ------------------------------------- Block: 3 cycles
//  321         {
//  322           MSTOP = 0U;     /* X1 oscillator/external clock operating */
        CLR1      0xFFFA1.7          ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
//  323         }
//  324         
//  325         CSS = 0U;       /* main system clock (fMAIN) */
??R_CGC_Set_ClockMode_10:
        CLR1      0xFFFA4.6          ;; 2 cycles
//  326         MCM0 = 1U;      /* selects the high-speed system clock (fMX) as the main system clock (fMAIN) */
        SET1      0xFFFA4.4          ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
//  327       }
//  328       
//  329       break;
??R_CGC_Set_LSMode_34:
        BR        N:??R_CGC_Set_LSMode_9  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
//  330       
//  331     case SUBXT1CLK:
//  332       
//  333       if ((SUBEXTCLK == old_mode) || (fILCLK == old_mode) || (PLLCLK == old_mode) || ((SCMC & _30_CGC_SUB_PIN) != _10_CGC_SUB_OSC))
??R_CGC_Set_LSMode_15:
        XCH       A, D               ;; 1 cycle
        CMP       A, #0x6            ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        BZ        ??R_CGC_Set_LSMode_35  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        XCH       A, D               ;; 1 cycle
        CMP       A, #0x7            ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        BZ        ??R_CGC_Set_LSMode_35  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        XCH       A, D               ;; 1 cycle
        CMP       A, #0x4            ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        BZ        ??R_CGC_Set_LSMode_35  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOV       A, 0x384           ;; 1 cycle
        AND       A, #0x30           ;; 1 cycle
        CMP       A, #0x10           ;; 1 cycle
        BZ        ??R_CGC_Set_LSMode_36  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  334       {
//  335         status = MD_ERROR5;
??R_CGC_Set_LSMode_35:
        MOVW      AX, #0x86          ;; 1 cycle
        MOVW      [SP+0x04], AX      ;; 1 cycle
        BR        S:??R_CGC_Set_LSMode_37  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  336       }
//  337       else
//  338       {
//  339         VRTCEN = 1U;    /* enables input clock supply */
??R_CGC_Set_LSMode_36:
        SET1      0xF00FC.0          ;; 2 cycles
//  340         
//  341         if (1U == XTSTOP)
        MOVW      HL, #0x386         ;; 1 cycle
        MOV1      CY, [HL].6         ;; 1 cycle
        BNC       ??R_CGC_Set_LSMode_38  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
//  342         {
//  343           XTSTOP = 0U;    /* XT1 oscillator operating */
        CLR1      0xF0386.6          ;; 2 cycles
//  344           CMC |= _10_CGC_SYSOSC_PERMITTED;
        MOV       A, 0xFFFA0         ;; 1 cycle
        OR        A, #0x10           ;; 1 cycle
        MOV       0xFFFA0, A         ;; 1 cycle
//  345           
//  346           /* Change the waiting time according to the system */
//  347           for (w_count = 0U; w_count <= CGC_SUBWAITTIME; w_count++)
        MOVW      AX, SP             ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        ; ------------------------------------- Block: 11 cycles
??R_CGC_Set_ClockMode_11:
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x169         ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??R_CGC_Set_ClockMode_12:
        BNC       ??R_CGC_Set_LSMode_39  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
//  348           {
//  349             NOP();
        NOP                          ;; 1 cycle
//  350           }
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        ADDW      AX, #0x1           ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      HL, SP             ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        BR        S:??R_CGC_Set_ClockMode_11  ;; 3 cycles
        ; ------------------------------------- Block: 21 cycles
//  351           
//  352           XT1SELDIS = 0U; /* enables clock supply */
??R_CGC_Set_LSMode_39:
        CLR1      0xFFFA1.6          ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
//  353         }
//  354         
//  355         SELLOSC = 0U;   /* sub clock (fSX) */
??R_CGC_Set_LSMode_38:
        CLR1      0xFFFA7.0          ;; 2 cycles
//  356         CSS = 1U;       /* subsystem clock (fSUB) */
        SET1      0xFFFA4.6          ;; 2 cycles
//  357         VRTCEN = 0U;    /* stops input clock supply */
        CLR1      0xF00FC.0          ;; 2 cycles
        ; ------------------------------------- Block: 6 cycles
//  358       }
//  359       
//  360       break;
??R_CGC_Set_LSMode_37:
        BR        N:??R_CGC_Set_LSMode_9  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
//  361       
//  362     case SUBEXTCLK:
//  363       
//  364       if ((SUBXT1CLK == old_mode) || (fILCLK == old_mode) || (PLLCLK == old_mode) || ((SCMC & _30_CGC_SUB_PIN) != _30_CGC_SUB_EXT))
??R_CGC_Set_LSMode_16:
        XCH       A, D               ;; 1 cycle
        CMP       A, #0x5            ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        BZ        ??R_CGC_Set_LSMode_40  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        XCH       A, D               ;; 1 cycle
        CMP       A, #0x7            ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        BZ        ??R_CGC_Set_LSMode_40  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        XCH       A, D               ;; 1 cycle
        CMP       A, #0x4            ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        BZ        ??R_CGC_Set_LSMode_40  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOV       A, 0x384           ;; 1 cycle
        AND       A, #0x30           ;; 1 cycle
        CMP       A, #0x30           ;; 1 cycle
        BZ        ??R_CGC_Set_LSMode_41  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  365       {
//  366         status = MD_ERROR6;
??R_CGC_Set_LSMode_40:
        MOVW      AX, #0x87          ;; 1 cycle
        MOVW      [SP+0x04], AX      ;; 1 cycle
        BR        S:??R_CGC_Set_LSMode_42  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  367       }
//  368       else
//  369       {
//  370         VRTCEN = 1U;    /* enables input clock supply */
??R_CGC_Set_LSMode_41:
        SET1      0xF00FC.0          ;; 2 cycles
//  371         CMC |= _10_CGC_SYSOSC_PERMITTED;
        MOV       A, 0xFFFA0         ;; 1 cycle
        OR        A, #0x10           ;; 1 cycle
        MOV       0xFFFA0, A         ;; 1 cycle
//  372         
//  373         
//  374         /* Change the waiting time according to the system */
//  375         for (w_count = 0U; w_count <= CGC_SUBWAITTIME; w_count++)
        MOVW      AX, SP             ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        ; ------------------------------------- Block: 11 cycles
??R_CGC_Set_ClockMode_13:
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x169         ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??R_CGC_Set_ClockMode_14:
        BNC       ??R_CGC_Set_LSMode_43  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
//  376         {
//  377           NOP();
        NOP                          ;; 1 cycle
//  378         }
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        ADDW      AX, #0x1           ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      HL, SP             ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        BR        S:??R_CGC_Set_ClockMode_13  ;; 3 cycles
        ; ------------------------------------- Block: 21 cycles
//  379         
//  380         XT1SELDIS = 0U; /* enables clock supply */
??R_CGC_Set_LSMode_43:
        CLR1      0xFFFA1.6          ;; 2 cycles
//  381         
//  382         SELLOSC = 0U;   /* sub clock (fSX) */
        CLR1      0xFFFA7.0          ;; 2 cycles
//  383         CSS = 1U;       /* subsystem clock (fSUB) */
        SET1      0xFFFA4.6          ;; 2 cycles
//  384         VRTCEN = 0U;    /* stops input clock supply */
        CLR1      0xF00FC.0          ;; 2 cycles
        ; ------------------------------------- Block: 8 cycles
//  385       }
//  386       
//  387       break;
??R_CGC_Set_LSMode_42:
        BR        S:??R_CGC_Set_LSMode_9  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
//  388       
//  389     case fILCLK:
//  390       
//  391       if ((SUBXT1CLK == old_mode) || (SUBEXTCLK == old_mode) || (PLLCLK == old_mode))
??R_CGC_Set_LSMode_17:
        XCH       A, D               ;; 1 cycle
        CMP       A, #0x5            ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        BZ        ??R_CGC_Set_LSMode_44  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        XCH       A, D               ;; 1 cycle
        CMP       A, #0x6            ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        BZ        ??R_CGC_Set_LSMode_44  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        XCH       A, D               ;; 1 cycle
        CMP       A, #0x4            ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        BNZ       ??R_CGC_Set_LSMode_45  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  392       {
//  393         status = MD_ERROR7;
??R_CGC_Set_LSMode_44:
        MOVW      AX, #0x88          ;; 1 cycle
        MOVW      [SP+0x04], AX      ;; 1 cycle
        BR        S:??R_CGC_Set_LSMode_9  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  394       }
//  395       else
//  396       {
//  397         SELLOSC = 1U;   /* low-speed on-chip oscillator clock (fIL) */
??R_CGC_Set_LSMode_45:
        SET1      0xFFFA7.0          ;; 2 cycles
//  398         
//  399         /* Change the waiting time according to the system */
//  400         for (w_count = 0U; w_count <= CGC_FILWAITTIME; w_count++)
        MOVW      AX, SP             ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        ; ------------------------------------- Block: 8 cycles
??R_CGC_Set_ClockMode_15:
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x1A5         ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??R_CGC_Set_ClockMode_16:
        BNC       ??R_CGC_Set_LSMode_46  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
//  401         {
//  402           NOP();
        NOP                          ;; 1 cycle
//  403         }
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        ADDW      AX, #0x1           ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      HL, SP             ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        BR        S:??R_CGC_Set_ClockMode_15  ;; 3 cycles
        ; ------------------------------------- Block: 21 cycles
//  404         
//  405         CSS = 1U;       /* subsystem clock (fSUB) */
??R_CGC_Set_LSMode_46:
        SET1      0xFFFA4.6          ;; 2 cycles
//  406         
//  407       }
//  408       
//  409       break;
        BR        S:??R_CGC_Set_LSMode_9  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  410       
//  411     default:
//  412       
//  413       status = MD_ARGERROR;
??R_CGC_Set_LSMode_18:
        MOVW      AX, #0x81          ;; 1 cycle
        MOVW      [SP+0x04], AX      ;; 1 cycle
//  414       
//  415       break;
        ; ------------------------------------- Block: 2 cycles
//  416     }
//  417   }
//  418   
//  419   return (status);
??R_CGC_Set_LSMode_9:
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        ADDW      SP, #0xA           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock1
        ; ------------------------------------- Block: 8 cycles
        ; ------------------------------------- Total: 751 cycles
        REQUIRE __A_CKC
        REQUIRE __A_CKSEL
        REQUIRE __A_SCMC
        REQUIRE __A_CMC
        REQUIRE __A_MCKC
        REQUIRE __A_DSCCTL
        REQUIRE __A_PCKC
        REQUIRE __A_CSC
        REQUIRE __A_OSTS
        REQUIRE __A_OSTC
        REQUIRE __A_PER2
        REQUIRE __A_SCSC
//  420 }
//  421 /***********************************************************************************************************************
//  422 * Function Name: R_CGC_Set_LPMode
//  423 * Description  : This function This function changes Flash Operation mode from LS(low - speed main) mode to LP(low - power main) mode.
//  424 * Arguments    : None
//  425 * Return Value : None
//  426 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock2 Using cfiCommon0
          CFI Function _R_CGC_Set_LPMode
          CFI NoCalls
        CODE
//  427 void R_CGC_Set_LPMode(void)
//  428 {
_R_CGC_Set_LPMode:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  429   /* Transition to LP mode */
//  430   MCSEL = 0;                  /* Normal setting */
        CLR1      0xF00F8.6          ;; 2 cycles
//  431   FLMWEN = 1;                 /* Rewriting the FLMODE register is enabled */
        SET1      0xF00AB.0          ;; 2 cycles
//  432   FLMODE = (FLMODE ^ 0xC0);   /* LP (low-power main) mode */
        MOVW      HL, #0xAA          ;; 1 cycle
        MOV       A, [HL]            ;; 1 cycle
        XOR       A, #0xC0           ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
//  433   FLMWEN = 0;                 /* Rewriting the FLMODE register is disabled */
        CLR1      0xF00AB.0          ;; 2 cycles
//  434   MCSEL = 1;                  /* Low-power consumption setting */
        SET1      0xF00F8.6          ;; 2 cycles
//  435 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock2
        ; ------------------------------------- Block: 18 cycles
        ; ------------------------------------- Total: 18 cycles
        REQUIRE __A_PMMC
        REQUIRE __A_FLMWRP
        REQUIRE __A_FLMODE
//  436 /***********************************************************************************************************************
//  437 * Function Name: R_CGC_Set_LSMode
//  438 * Description  : This function This function changes Flash Operation mode from LP(low - power main) mode to LS(low - speed main) mode.
//  439 * Arguments    : None
//  440 * Return Value : None
//  441 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock3 Using cfiCommon0
          CFI Function _R_CGC_Set_LSMode
          CFI NoCalls
        CODE
//  442 void R_CGC_Set_LSMode(void)
//  443 {
_R_CGC_Set_LSMode:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  444   /* Transition to LS mode */
//  445   MCSEL = 0;                  /* Normal setting */
        CLR1      0xF00F8.6          ;; 2 cycles
//  446   FLMWEN = 1;                 /* Rewriting the FLMODE register is enabled */
        SET1      0xF00AB.0          ;; 2 cycles
//  447   FLMODE = (FLMODE ^ 0xC0);   /* LS (low-speed main) mode */
        MOVW      HL, #0xAA          ;; 1 cycle
        MOV       A, [HL]            ;; 1 cycle
        XOR       A, #0xC0           ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
//  448   FLMWEN = 0;                 /* Rewriting the FLMODE register is disabled */
        CLR1      0xF00AB.0          ;; 2 cycles
//  449   MCSEL = 1;                  /* Low-power consumption setting */
        SET1      0xF00F8.6          ;; 2 cycles
//  450 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock3
        ; ------------------------------------- Block: 18 cycles
        ; ------------------------------------- Total: 18 cycles
        REQUIRE __A_PMMC
        REQUIRE __A_FLMWRP
        REQUIRE __A_FLMODE

        SECTION `.iar_vfe_header`:DATA:NOALLOC:NOROOT(1)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
        DC32 0

        END
//  451 
//  452 /* Start user code for adding. Do not edit comment generated here */
//  453 /* End user code. Do not edit comment generated here */
// 
//    16 bytes in section .bss.noinit  (abs)
// 1'223 bytes in section .text
// 
// 1'223 bytes of CODE memory
//     0 bytes of DATA memory (+ 16 bytes shared)
//
//Errors: none
//Warnings: none
