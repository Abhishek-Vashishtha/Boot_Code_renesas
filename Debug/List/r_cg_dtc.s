///////////////////////////////////////////////////////////////////////////////
//
// IAR C/C++ Compiler V4.20.1.2260 for RL78               20/Dec/2020  22:38:47
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
//        BootCode\source_code\driver_files\r_cg_dtc.c
//    Command line       =
//        -f C:\Users\laptop\AppData\Local\Temp\EWCD2B.tmp ("D:\Dheeraj\New
//        folder\0. GDEV72 - BootCode\source_code\driver_files\r_cg_dtc.c"
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
//        D:\Dheeraj\New folder\0. GDEV72 - BootCode\Debug\List\r_cg_dtc.s
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

        PUBLIC _R_DTCD0_Start
        PUBLIC _R_DTCD0_Stop
        PUBLIC _R_DTCD1_Start
        PUBLIC _R_DTCD1_Stop
        PUBLIC _R_DTC_Create
        PUBLIC _R_DTC_Set_PowerOff
        PUBLIC __A_DTCBAR
        PUBLIC __A_DTCEN0
        PUBLIC __A_DTCEN1
        PUBLIC __A_DTCEN2
        PUBLIC __A_DTCEN3
        PUBLIC __A_DTCEN4
        PUBLIC __A_PER1
        PUBLIC __A_TS0
        PUBLIC _adc_data
        PUBLIC _dtc_controldata_0
        PUBLIC _dtc_controldata_1
        PUBLIC _dtc_vectortable
        PUBLIC _g_dtc_tau01_trigger
        
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
        
// D:\Dheeraj\New folder\0. GDEV72 - BootCode\source_code\driver_files\r_cg_dtc.c
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
//   21 * File Name    : r_cg_dtc.c
//   22 * Version      : Applilet4 for RL78/I1C V1.01.02.04 [24 May 2018]
//   23 * Device(s)    : R5F10NPJ
//   24 * Tool-Chain   : IAR Systems icc78k0r
//   25 * Description  : This file implements device driver for DTC module.
//   26 * Creation Date: 01/10/2020
//   27 ***********************************************************************************************************************/
//   28 
//   29 /***********************************************************************************************************************
//   30 Includes
//   31 ***********************************************************************************************************************/
//   32 #include "r_cg_macrodriver.h"

        ASEGN `.bss.noinit`:DATA:NOROOT,0f00faH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_PER1
// __no_init union <unnamed>#287 volatile _A_PER1
__A_PER1:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f01b2H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_TS0
// __no_init union <unnamed>#447 volatile _A_TS0
__A_TS0:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f02e0H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_DTCBAR
// __no_init union <unnamed>#512 volatile __no_bit_access _A_DTCBAR
__A_DTCBAR:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f02e8H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_DTCEN0
// __no_init union <unnamed>#515 volatile _A_DTCEN0
__A_DTCEN0:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f02e9H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_DTCEN1
// __no_init union <unnamed>#517 volatile _A_DTCEN1
__A_DTCEN1:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f02eaH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_DTCEN2
// __no_init union <unnamed>#519 volatile _A_DTCEN2
__A_DTCEN2:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f02ebH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_DTCEN3
// __no_init union <unnamed>#521 volatile _A_DTCEN3
__A_DTCEN3:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f02ecH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_DTCEN4
// __no_init union <unnamed>#523 volatile _A_DTCEN4
__A_DTCEN4:
        DS 1
//   33 #include "r_cg_dtc.h"
//   34 /* Start user code for include. Do not edit comment generated here */
//   35 /* End user code. Do not edit comment generated here */
//   36 #include "r_cg_userdefine.h"
//   37 #include "variables.h"
//   38 #include "r_cg_tau.h"
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
//   49 /* End user code. Do not edit comment generated here */
//   50 
//   51 //#pragma location = 0x0FFD00U
//   52 //__no_init uint8_t dtc_vectortable[40U];
//   53 //
//   54 //#pragma location = 0x0FFD48U
//   55 
//   56 //__no_init st_dtc_data dtc_controldata_1;
//   57 ////#pragma location="T01_Trigger"
//   58 //us8 __near g_dtc_tau01_trigger = 0x02;
//   59 //
//   60 //#pragma default_variable_attributes=@"ADC_Data"
//   61 //
//   62 
//   63 
//   64 /* DTC Vector and control data */
//   65 #pragma location = 0x0FFD00U

        ASEGN `.bss.noinit`:DATA:NOROOT,0ffd00H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP _dtc_vectortable
//   66 __no_init uint8_t dtc_vectortable[40U];
_dtc_vectortable:
        DS 40
//   67 
//   68 #pragma location = 0x0FFD40U

        ASEGN `.bss.noinit`:DATA:NOROOT,0ffd40H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP _dtc_controldata_0
//   69 __no_init st_dtc_data dtc_controldata_0;
_dtc_controldata_0:
        DS 8
//   70 
//   71 #pragma location = 0x0FFD48U

        ASEGN `.bss.noinit`:DATA:NOROOT,0ffd48H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP _dtc_controldata_1
//   72 __no_init st_dtc_data dtc_controldata_1;
_dtc_controldata_1:
        DS 8
//   73 
//   74 /* variables */
//   75 //#pragma default_variable_attributes=@"ADC_Data"

        ASEGN `.bss`:DATA:NOROOT,0fda00H
//   76 volatile int16_t adc_data[8] @ 0xFDA00;
_adc_data:
        DS 16

        ASEGN `.bss`:DATA:NOROOT,0ffb00H
//   77 us8 g_dtc_tau01_trigger @ 0xFFB00;
_g_dtc_tau01_trigger:
        DS 1
//   78 /***********************************************************************************************************************
//   79 * Function Name: R_DTC_Create
//   80 * Description  : This function initializes the DTC module.
//   81 * Arguments    : None
//   82 * Return Value : None
//   83 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock0 Using cfiCommon0
          CFI Function _R_DTC_Create
          CFI NoCalls
        CODE
//   84 void R_DTC_Create(void)
//   85 {
_R_DTC_Create:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//   86   g_dtc_tau01_trigger = 0x02u;
        MOV       0xFB00, #0x2       ;; 1 cycle
//   87   /* Enable input clock supply */
//   88   DTCEN = 1U;
        SET1      0xF00FA.3          ;; 2 cycles
//   89   /* Disable all DTC channels operation */
//   90   DTCEN0 = 0x00U;
        MOV       0x2E8, #0x0        ;; 1 cycle
//   91   DTCEN1 = 0x00U;
        MOV       0x2E9, #0x0        ;; 1 cycle
//   92   DTCEN2 = 0x00U;
        MOV       0x2EA, #0x0        ;; 1 cycle
//   93   DTCEN3 = 0x00U;
        MOV       0x2EB, #0x0        ;; 1 cycle
//   94   DTCEN4 = 0x00U;
        MOV       0x2EC, #0x0        ;; 1 cycle
//   95   /* Set base address */
//   96   DTCBAR = 0xFDU;
        MOV       0x2E0, #0xFD       ;; 1 cycle
//   97   
//   98   /* Set DTCD0 */
//   99   dtc_vectortable[10U] = 0x40U;
        MOVW      HL, #0xFD0A        ;; 1 cycle
        MOV       A, #0x40           ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
//  100   dtc_controldata_0.dtccr = _01_DTC_TRANSFER_MODE_REPEAT | _00_DTC_DATA_SIZE_8BITS | _02_DTC_REPEAT_AREA_SOURCE | 
//  101     _00_DTC_DEST_ADDR_FIXED | _20_DTC_REPEAT_INT_ENABLE | _00_DTC_CHAIN_TRANSFER_DISABLE;
        MOV       0xFD40, #0x23      ;; 1 cycle
//  102   dtc_controldata_0.dtbls = _01_DTCD0_TRANSFER_BLOCKSIZE;
        MOVW      HL, #0xFD41        ;; 1 cycle
        MOV       A, #0x1            ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
//  103   dtc_controldata_0.dtcct = _01_DTCD0_TRANSFER_BYTE;
        MOVW      HL, #0xFD42        ;; 1 cycle
        MOV       A, #0x1            ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
//  104   dtc_controldata_0.dtrld = _01_DTCD0_TRANSFER_BYTE;
        MOVW      HL, #0xFD43        ;; 1 cycle
        MOV       A, #0x1            ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
//  105   
//  106   dtc_controldata_0.dtsar = (uint16_t)&g_dtc_tau01_trigger;
        MOVW      HL, #0xFD44        ;; 1 cycle
        MOVW      AX, #0xFB00        ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
//  107   dtc_controldata_0.dtdar = (uint16_t)&TS0L;
        MOVW      HL, #0xFD46        ;; 1 cycle
        MOVW      AX, #0x1B2         ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
//  108   
//  109   /* Set DTCD1 */
//  110   dtc_vectortable[11U] = 0x48U;
        MOVW      HL, #0xFD0B        ;; 1 cycle
        MOV       A, #0x48           ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
//  111   dtc_controldata_1.dtccr = _01_DTC_TRANSFER_MODE_REPEAT | _40_DTC_DATA_SIZE_16BITS | _00_DTC_SOURCE_ADDR_FIXED | 
//  112     _00_DTC_REPEAT_AREA_DEST | _00_DTC_REPEAT_INT_DISABLE |  _00_DTC_CHAIN_TRANSFER_DISABLE;
        MOV       0xFD48, #0x41      ;; 1 cycle
//  113   dtc_controldata_1.dtbls = _01_DTCD1_TRANSFER_BLOCKSIZE;
        MOVW      HL, #0xFD49        ;; 1 cycle
        MOV       A, #0x1            ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
//  114   dtc_controldata_1.dtcct = _08_DTCD1_TRANSFER_BYTE;
        MOVW      HL, #0xFD4A        ;; 1 cycle
        MOV       A, #0x8            ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
//  115   dtc_controldata_1.dtrld = _08_DTCD1_TRANSFER_BYTE;
        MOVW      HL, #0xFD4B        ;; 1 cycle
        MOV       A, #0x8            ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
//  116   
//  117   dtc_controldata_1.dtsar = _FF1E_DTCD0_SRC_ADDRESS;
        MOVW      HL, #0xFD4C        ;; 1 cycle
        MOVW      AX, #0xFF1E        ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
//  118   dtc_controldata_1.dtdar = (uint16_t)&adc_data;
        MOVW      HL, #0xFD4E        ;; 1 cycle
        MOVW      AX, #0xDA00        ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
//  119 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock0
        ; ------------------------------------- Block: 53 cycles
        ; ------------------------------------- Total: 53 cycles
        REQUIRE _g_dtc_tau01_trigger
        REQUIRE __A_PER1
        REQUIRE __A_DTCEN0
        REQUIRE __A_DTCEN1
        REQUIRE __A_DTCEN2
        REQUIRE __A_DTCEN3
        REQUIRE __A_DTCEN4
        REQUIRE __A_DTCBAR
        REQUIRE _dtc_vectortable
        REQUIRE _dtc_controldata_0
        REQUIRE __A_TS0
        REQUIRE _dtc_controldata_1
        REQUIRE _adc_data
//  120 /***********************************************************************************************************************
//  121 * Function Name: R_DTCD0_Start
//  122 * Description  : This function enables DTCD0 module operation.
//  123 * Arguments    : None
//  124 * Return Value : None
//  125 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock1 Using cfiCommon0
          CFI Function _R_DTCD0_Start
          CFI NoCalls
        CODE
//  126 void R_DTCD0_Start(void)
//  127 {
_R_DTCD0_Start:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  128     BIT_SELECT( DTCEN1, 5 ) = 1;
        SET1      0xF02E9.5          ;; 2 cycles
//  129 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock1
        ; ------------------------------------- Block: 8 cycles
        ; ------------------------------------- Total: 8 cycles
        REQUIRE __A_DTCEN1
//  130 /***********************************************************************************************************************
//  131 * Function Name: R_DTCD0_Stop
//  132 * Description  : This function disables DTCD0 module operation.
//  133 * Arguments    : None
//  134 * Return Value : None
//  135 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock2 Using cfiCommon0
          CFI Function _R_DTCD0_Stop
          CFI NoCalls
        CODE
//  136 void R_DTCD0_Stop(void)
//  137 {
_R_DTCD0_Stop:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  138     BIT_SELECT( DTCEN1, 5 ) = 0;
        CLR1      0xF02E9.5          ;; 2 cycles
//  139     dtc_controldata_0.dtccr = _01_DTC_TRANSFER_MODE_REPEAT | _00_DTC_DATA_SIZE_8BITS | _02_DTC_REPEAT_AREA_SOURCE | 
//  140                               _00_DTC_DEST_ADDR_FIXED | _20_DTC_REPEAT_INT_ENABLE | _00_DTC_CHAIN_TRANSFER_DISABLE;
        MOV       0xFD40, #0x23      ;; 1 cycle
//  141     dtc_controldata_0.dtbls = _01_DTCD0_TRANSFER_BLOCKSIZE;
        MOVW      HL, #0xFD41        ;; 1 cycle
        MOV       A, #0x1            ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
//  142     dtc_controldata_0.dtcct = _01_DTCD0_TRANSFER_BYTE;
        MOVW      HL, #0xFD42        ;; 1 cycle
        MOV       A, #0x1            ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
//  143     dtc_controldata_0.dtrld = _01_DTCD0_TRANSFER_BYTE;
        MOVW      HL, #0xFD43        ;; 1 cycle
        MOV       A, #0x1            ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
//  144 
//  145     dtc_controldata_0.dtsar = (uint16_t)&g_dtc_tau01_trigger;
        MOVW      HL, #0xFD44        ;; 1 cycle
        MOVW      AX, #0xFB00        ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
//  146     dtc_controldata_0.dtdar = (uint16_t)&TS0L;
        MOVW      HL, #0xFD46        ;; 1 cycle
        MOVW      AX, #0x1B2         ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
//  147 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock2
        ; ------------------------------------- Block: 24 cycles
        ; ------------------------------------- Total: 24 cycles
        REQUIRE __A_DTCEN1
        REQUIRE _dtc_controldata_0
        REQUIRE _g_dtc_tau01_trigger
        REQUIRE __A_TS0
//  148 /***********************************************************************************************************************
//  149 * Function Name: R_DTCD1_Start
//  150 * Description  : This function enables DTCD1 module operation.
//  151 * Arguments    : None
//  152 * Return Value : None
//  153 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock3 Using cfiCommon0
          CFI Function _R_DTCD1_Start
          CFI NoCalls
        CODE
//  154 void R_DTCD1_Start(void)
//  155 {
_R_DTCD1_Start:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  156      BIT_SELECT( DTCEN1, 4 ) = 1;
        SET1      0xF02E9.4          ;; 2 cycles
//  157 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock3
        ; ------------------------------------- Block: 8 cycles
        ; ------------------------------------- Total: 8 cycles
        REQUIRE __A_DTCEN1
//  158 /***********************************************************************************************************************
//  159 * Function Name: R_DTCD1_Stop
//  160 * Description  : This function disables DTCD1 module operation.
//  161 * Arguments    : None
//  162 * Return Value : None
//  163 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock4 Using cfiCommon0
          CFI Function _R_DTCD1_Stop
          CFI NoCalls
        CODE
//  164 void R_DTCD1_Stop(void)
//  165 {
_R_DTCD1_Stop:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  166     BIT_SELECT( DTCEN1, 4 ) = 0;
        CLR1      0xF02E9.4          ;; 2 cycles
//  167     dtc_controldata_1.dtccr = _01_DTC_TRANSFER_MODE_REPEAT | _40_DTC_DATA_SIZE_16BITS | _00_DTC_SOURCE_ADDR_FIXED | 
//  168                               _00_DTC_REPEAT_AREA_DEST | _00_DTC_REPEAT_INT_DISABLE |  _00_DTC_CHAIN_TRANSFER_DISABLE;
        MOV       0xFD48, #0x41      ;; 1 cycle
//  169     dtc_controldata_1.dtbls = _01_DTCD1_TRANSFER_BLOCKSIZE;
        MOVW      HL, #0xFD49        ;; 1 cycle
        MOV       A, #0x1            ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
//  170     dtc_controldata_1.dtcct = _08_DTCD1_TRANSFER_BYTE;
        MOVW      HL, #0xFD4A        ;; 1 cycle
        MOV       A, #0x8            ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
//  171     dtc_controldata_1.dtrld = _08_DTCD1_TRANSFER_BYTE;
        MOVW      HL, #0xFD4B        ;; 1 cycle
        MOV       A, #0x8            ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
//  172 
//  173     dtc_controldata_1.dtsar = _FF1E_DTCD0_SRC_ADDRESS;
        MOVW      HL, #0xFD4C        ;; 1 cycle
        MOVW      AX, #0xFF1E        ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
//  174     dtc_controldata_1.dtdar = (uint16_t)&adc_data;
        MOVW      HL, #0xFD4E        ;; 1 cycle
        MOVW      AX, #0xDA00        ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
//  175 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock4
        ; ------------------------------------- Block: 24 cycles
        ; ------------------------------------- Total: 24 cycles
        REQUIRE __A_DTCEN1
        REQUIRE _dtc_controldata_1
        REQUIRE _adc_data
//  176 
//  177 /***********************************************************************************************************************
//  178 * Function Name: R_DTC_Set_PowerOff
//  179 * Description  : This function stops the clock supplied for DTC.
//  180 * Arguments    : None
//  181 * Return Value : None
//  182 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock5 Using cfiCommon0
          CFI Function _R_DTC_Set_PowerOff
          CFI NoCalls
        CODE
//  183 void R_DTC_Set_PowerOff(void)
//  184 {
_R_DTC_Set_PowerOff:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  185     DTCEN = 0U;     /* stops input clock supply */
        CLR1      0xF00FA.3          ;; 2 cycles
//  186 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock5
        ; ------------------------------------- Block: 8 cycles
        ; ------------------------------------- Total: 8 cycles
        REQUIRE __A_PER1

        SECTION `.iar_vfe_header`:DATA:NOALLOC:NOROOT(1)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
        DC32 0

        END
//  187 
//  188 /* Start user code for adding. Do not edit comment generated here */
//  189 /* End user code. Do not edit comment generated here */
// 
//  17 bytes in section .bss         (abs)
//  65 bytes in section .bss.noinit  (abs)
// 214 bytes in section .text
// 
// 214 bytes of CODE memory
//  17 bytes of DATA memory (+ 65 bytes shared)
//
//Errors: none
//Warnings: none
