///////////////////////////////////////////////////////////////////////////////
//
// IAR C/C++ Compiler V4.20.1.2260 for RL78               20/Dec/2020  22:38:45
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
//        BootCode\source_code\driver_files\r_cg_adc.c
//    Command line       =
//        -f C:\Users\laptop\AppData\Local\Temp\EWB33F.tmp ("D:\Dheeraj\New
//        folder\0. GDEV72 - BootCode\source_code\driver_files\r_cg_adc.c"
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
//        D:\Dheeraj\New folder\0. GDEV72 - BootCode\Debug\List\r_cg_adc.s
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

        EXTERN _MulFactor
        EXTERN _R_ADC_Create_UserInit
        EXTERN _delay_ms
        EXTERN _delay_us
        EXTERN _flag_battery

        PUBLIC _R_ADC_Create
        PUBLIC _R_ADC_Create_Select_Mode
        PUBLIC _R_ADC_Get_Result
        PUBLIC _R_ADC_Get_Result_8bit
        PUBLIC _R_ADC_Reset
        PUBLIC _R_ADC_Set_ADChannel
        PUBLIC _R_ADC_Set_OperationOff
        PUBLIC _R_ADC_Set_OperationOn
        PUBLIC _R_ADC_Set_PowerOff
        PUBLIC _R_ADC_Set_SnoozeOff
        PUBLIC _R_ADC_Set_SnoozeOn
        PUBLIC _R_ADC_Set_TestChannel
        PUBLIC _R_ADC_Start
        PUBLIC _R_ADC_Stop
        PUBLIC __A_ADCR
        PUBLIC __A_ADLL
        PUBLIC __A_ADM0
        PUBLIC __A_ADM1
        PUBLIC __A_ADM2
        PUBLIC __A_ADS
        PUBLIC __A_ADTES
        PUBLIC __A_ADUL
        PUBLIC __A_IF1
        PUBLIC __A_MK1
        PUBLIC __A_P3
        PUBLIC __A_PER0
        PUBLIC __A_PM2
        PUBLIC __A_PR01
        PUBLIC __A_PR11
        PUBLIC __A_PRR0
        PUBLIC _read_battery_voltage
        
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
        
// D:\Dheeraj\New folder\0. GDEV72 - BootCode\source_code\driver_files\r_cg_adc.c
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
//   21 * File Name    : r_cg_adc.c
//   22 * Version      : Applilet4 for RL78/I1C V1.01.02.04 [24 May 2018]
//   23 * Device(s)    : R5F10NPJ
//   24 * Tool-Chain   : IAR Systems icc78k0r
//   25 * Description  : This file implements device driver for ADC module.
//   26 * Creation Date: 01/09/2020
//   27 ***********************************************************************************************************************/
//   28 
//   29 /***********************************************************************************************************************
//   30 Includes
//   31 ***********************************************************************************************************************/
//   32 #include "r_cg_macrodriver.h"

        ASEGN `.sbss.noinit`:DATA:NOROOT,0fff03H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_P3
// __no_init union <unnamed>#6 volatile __saddr _A_P3
__A_P3:
        DS 1

        ASEGN `.sbss.noinit`:DATA:NOROOT,0fff1eH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_ADCR
// __no_init union <unnamed>#32 const volatile __saddr __no_bit_access _A_ADCR
__A_ADCR:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0fff22H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_PM2
// __no_init union <unnamed>#37 volatile __sfr _A_PM2
__A_PM2:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0fff30H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_ADM0
// __no_init union <unnamed>#46 volatile __sfr _A_ADM0
__A_ADM0:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0fff31H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_ADS
// __no_init union <unnamed>#48 volatile __sfr _A_ADS
__A_ADS:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0fff32H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_ADM1
// __no_init union <unnamed>#49 volatile __sfr _A_ADM1
__A_ADM1:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0fffe2H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_IF1
// __no_init union <unnamed>#160 volatile __sfr _A_IF1
__A_IF1:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0fffe6H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_MK1
// __no_init union <unnamed>#178 volatile __sfr _A_MK1
__A_MK1:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0fffeaH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_PR01
// __no_init union <unnamed>#196 volatile __sfr _A_PR01
__A_PR01:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0fffeeH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_PR11
// __no_init union <unnamed>#214 volatile __sfr _A_PR11
__A_PR11:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0010H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_ADM2
// __no_init union <unnamed>#228 volatile _A_ADM2
__A_ADM2:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0011H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_ADUL
// __no_init union <unnamed>#230 volatile __no_bit_access _A_ADUL
__A_ADUL:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0012H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_ADLL
// __no_init union <unnamed>#231 volatile __no_bit_access _A_ADLL
__A_ADLL:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0013H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_ADTES
// __no_init union <unnamed>#232 volatile __no_bit_access _A_ADTES
__A_ADTES:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f00f0H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_PER0
// __no_init union <unnamed>#275 volatile _A_PER0
__A_PER0:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f00f1H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_PRR0
// __no_init union <unnamed>#277 volatile _A_PRR0
__A_PRR0:
        DS 1
//   33 #include "r_cg_adc.h"
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

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock0 Using cfiCommon0
          CFI Function _read_battery_voltage
        CODE
//   49 us16 read_battery_voltage()
//   50 {
_read_battery_voltage:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 4
        SUBW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
//   51     us16 ret_val = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [SP], AX           ;; 1 cycle
//   52     R_ADC_Create_Select_Mode();
          CFI FunCall _R_ADC_Create_Select_Mode
        CALL      _R_ADC_Create_Select_Mode  ;; 3 cycles
//   53     //delay_ms(5);
//   54     R_ADC_Set_OperationOn();
          CFI FunCall _R_ADC_Set_OperationOn
        CALL      _R_ADC_Set_OperationOn  ;; 3 cycles
//   55     //delay_ms(5);
//   56     EN_BAT_MEAS_HIGH;                   /* Turning on the battery */
        SET1      S:0xFFF03.2        ;; 2 cycles
//   57     if(battery_installed == TEKCELL)
        MOVW      HL, #LWRD(_flag_battery)  ;; 1 cycle
        MOV1      CY, [HL].2         ;; 1 cycle
        BC        ??R_ADC_Reset_0    ;; 4 cycles
        ; ------------------------------------- Block: 17 cycles
//   58     {
//   59         delay_ms(4);
        MOVW      AX, #0x4           ;; 1 cycle
          CFI FunCall _delay_ms
        CALL      _delay_ms          ;; 3 cycles
        BR        S:??R_ADC_Reset_1  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
//   60     }
//   61     else
//   62     {
//   63         delay_ms(1);
??R_ADC_Reset_0:
        MOVW      AX, #0x1           ;; 1 cycle
          CFI FunCall _delay_ms
        CALL      _delay_ms          ;; 3 cycles
          CFI FunCall _R_ADC_Start
        ; ------------------------------------- Block: 4 cycles
//   64     }
//   65     /* ignoring first sample */
//   66     R_ADC_Start();
??R_ADC_Reset_1:
        CALL      _R_ADC_Start       ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
//   67     while(ADCS == 1);
??read_battery_voltage_0:
        MOVW      HL, #0xFF30        ;; 1 cycle
        MOV1      CY, [HL].7         ;; 1 cycle
        BC        ??read_battery_voltage_0  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//   68     ret_val = (uint16_t) (ADCR >> 6U);
        MOVW      AX, S:0xFFF1E      ;; 1 cycle
        SHRW      AX, 0x6            ;; 1 cycle
        MOVW      [SP], AX           ;; 1 cycle
//   69     ret_val = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [SP], AX           ;; 1 cycle
//   70     for(us8 index = 0; index <10; index++)
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP+0x02], A       ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
??read_battery_voltage_1:
        MOV       A, [SP+0x02]       ;; 1 cycle
        CMP       A, #0xA            ;; 1 cycle
        BNC       ??R_ADC_Reset_2    ;; 4 cycles
          CFI FunCall _R_ADC_Start
        ; ------------------------------------- Block: 6 cycles
//   71     {
//   72         R_ADC_Start();
        CALL      _R_ADC_Start       ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
//   73         while(ADCS == 1);
??read_battery_voltage_2:
        MOVW      HL, #0xFF30        ;; 1 cycle
        MOV1      CY, [HL].7         ;; 1 cycle
        BC        ??read_battery_voltage_2  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//   74         ret_val += (us16) (ADCR >> 6U);
        MOVW      AX, S:0xFFF1E      ;; 1 cycle
        SHRW      AX, 0x6            ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        MOVW      [SP], AX           ;; 1 cycle
//   75         delay_ms(2);
        MOVW      AX, #0x2           ;; 1 cycle
          CFI FunCall _delay_ms
        CALL      _delay_ms          ;; 3 cycles
//   76     }
        MOV       A, [SP+0x02]       ;; 1 cycle
        INC       A                  ;; 1 cycle
        MOV       [SP+0x02], A       ;; 1 cycle
        BR        S:??read_battery_voltage_1  ;; 3 cycles
        ; ------------------------------------- Block: 16 cycles
//   77     EN_BAT_MEAS_LOW;                    /* Turning off the battery */
??R_ADC_Reset_2:
        CLR1      S:0xFFF03.2        ;; 2 cycles
//   78     R_ADC_Set_OperationOff();
          CFI FunCall _R_ADC_Set_OperationOff
        CALL      _R_ADC_Set_OperationOff  ;; 3 cycles
//   79     R_ADC_Set_PowerOff();
          CFI FunCall _R_ADC_Set_PowerOff
        CALL      _R_ADC_Set_PowerOff  ;; 3 cycles
//   80     ret_val /= 10;
        MOVW      DE, #0xA           ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        DIVHU                        ;; 9 cycles
        NOP                          ;; 1 cycle
        MOVW      [SP], AX           ;; 1 cycle
//   81     return(ret_val);
        MOVW      AX, [SP]           ;; 1 cycle
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock0
        ; ------------------------------------- Block: 29 cycles
        ; ------------------------------------- Total: 104 cycles
        REQUIRE __A_P3
        REQUIRE __A_ADM0
        REQUIRE __A_ADCR
//   82 }

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock1 Using cfiCommon0
          CFI Function _R_ADC_Create_Select_Mode
        CODE
//   83 void R_ADC_Create_Select_Mode(void)
//   84 {
_R_ADC_Create_Select_Mode:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 2
        SUBW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+6
//   85     volatile uint16_t w_count;
//   86 
//   87     ADCRES = 1U;    /* reset A/D converter */
        SET1      0xF00F1.5          ;; 2 cycles
//   88     ADCRES = 0U;    /* reset release of A/D converter */
        CLR1      0xF00F1.5          ;; 2 cycles
//   89     ADCEN = 1U;     /* enables input clock supply */
        SET1      0xF00F0.5          ;; 2 cycles
//   90     ADM0 = 0x00U;  /* disable AD conversion and clear ADM0 register */
        MOV       0xFFF30, #0x0      ;; 1 cycle
//   91     ADMK = 1U;      /* disable INTAD interrupt */
        SET1      0xFFFE7.2          ;; 2 cycles
//   92     ADIF = 0U;      /* clear INTAD interrupt flag */
        CLR1      0xFFFE3.2          ;; 2 cycles
//   93     /* The reset status of ADPC is analog input, so it's unnecessary to set. */
//   94     /* Set ANI0 - ANI5 pin */
//   95     PM2 |= 0x3FU;
        MOV       A, 0xFFF22         ;; 1 cycle
        OR        A, #0x3F           ;; 1 cycle
        MOV       0xFFF22, A         ;; 1 cycle
//   96     ADM0 = _00_AD_OPERMODE_SELECT | _08_AD_CONVERSION_CLOCK_32 | _00_AD_TIME_MODE_NORMAL_1;
        MOV       0xFFF30, #0x8      ;; 1 cycle
//   97     ADM1 = _00_AD_TRIGGER_SOFTWARE | _20_AD_CONVMODE_ONESELECT;
        MOV       0xFFF32, #0x20     ;; 1 cycle
//   98     ADM2 = _80_AD_POSITIVE_INTERVOLT | _00_AD_NEGATIVE_VSS | _00_AD_AREA_MODE_1 | _00_AD_RESOLUTION_10BIT;
        MOV       0x10, #0x80        ;; 1 cycle
//   99     ADUL = _FF_AD_ADUL_VALUE;
        MOV       0x11, #0xFF        ;; 1 cycle
//  100     ADLL = _00_AD_ADLL_VALUE;
        MOV       0x12, #0x0         ;; 1 cycle
//  101     ADS = _01_AD_INPUT_CHANNEL_1;
        MOV       0xFFF31, #0x1      ;; 1 cycle
//  102 
//  103     /* Change the waitting time according to the system */
//  104     for (w_count = 0U; w_count < (AD_WAITTIME * MulFactor); w_count++)
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [SP], AX           ;; 1 cycle
        ; ------------------------------------- Block: 23 cycles
??R_ADC_Create_Select_Mode_0:
        MOV       X, N:_MulFactor    ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0xD           ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BNC       ??R_ADC_Reset_3    ;; 4 cycles
        ; ------------------------------------- Block: 12 cycles
//  105     {
//  106       NOP();  
        NOP                          ;; 1 cycle
//  107     }
        MOVW      AX, [SP]           ;; 1 cycle
        INCW      AX                 ;; 1 cycle
        MOVW      [SP], AX           ;; 1 cycle
        BR        S:??R_ADC_Create_Select_Mode_0  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
//  108     ADCE = 1U;      /* enables A/D voltage comparator operation */
??R_ADC_Reset_3:
        SET1      0xFFF30.0          ;; 2 cycles
//  109     delay_us(1);
        MOVW      AX, #0x1           ;; 1 cycle
          CFI FunCall _delay_us
        CALL      _delay_us          ;; 3 cycles
//  110 }
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock1
        ; ------------------------------------- Block: 13 cycles
        ; ------------------------------------- Total: 55 cycles
        REQUIRE __A_PRR0
        REQUIRE __A_PER0
        REQUIRE __A_ADM0
        REQUIRE __A_MK1
        REQUIRE __A_IF1
        REQUIRE __A_PM2
        REQUIRE __A_ADM1
        REQUIRE __A_ADM2
        REQUIRE __A_ADUL
        REQUIRE __A_ADLL
        REQUIRE __A_ADS
//  111 
//  112 /***********************************************************************************************************************
//  113 * Function Name: R_ADC_Create
//  114 * Description  : This function initializes the AD converter.
//  115 * Arguments    : None
//  116 * Return Value : None
//  117 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock2 Using cfiCommon0
          CFI Function _R_ADC_Create
        CODE
//  118 void R_ADC_Create(void)
//  119 {
_R_ADC_Create:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 2
        SUBW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+6
//  120     volatile uint16_t w_count;
//  121 
//  122     ADCRES = 1U;    /* reset A/D converter */
        SET1      0xF00F1.5          ;; 2 cycles
//  123     ADCRES = 0U;    /* reset release of A/D converter */
        CLR1      0xF00F1.5          ;; 2 cycles
//  124     ADCEN = 1U;     /* enables input clock supply */
        SET1      0xF00F0.5          ;; 2 cycles
//  125     ADM0 = 0x00U;  /* disable AD conversion and clear ADM0 register */
        MOV       0xFFF30, #0x0      ;; 1 cycle
//  126     ADMK = 1U;      /* disable INTAD interrupt */
        SET1      0xFFFE7.2          ;; 2 cycles
//  127     ADIF = 0U;      /* clear INTAD interrupt flag */
        CLR1      0xFFFE3.2          ;; 2 cycles
//  128     /* Set INTAD high priority */
//  129     ADPR1 = 0U;
        CLR1      0xFFFEF.2          ;; 2 cycles
//  130     ADPR0 = 0U;
        CLR1      0xFFFEB.2          ;; 2 cycles
//  131     /* The reset status of ADPC is analog input, so it's unnecessary to set. */
//  132     /* Set ANI0 - ANI5 pin */
//  133     PM2 |= 0x3FU;                       /* pin_config */
        MOV       A, 0xFFF22         ;; 1 cycle
        OR        A, #0x3F           ;; 1 cycle
        MOV       0xFFF22, A         ;; 1 cycle
//  134 //    ADM0 = _08_AD_CONVERSION_CLOCK_32 | _00_AD_TIME_MODE_NORMAL_1 | _40_AD_OPERMODE_SCAN;
//  135     ADM0 = _30_AD_CONVERSION_CLOCK_4 | _00_AD_TIME_MODE_NORMAL_1 | _40_AD_OPERMODE_SCAN;
        MOV       0xFFF30, #0x70     ;; 1 cycle
//  136     ADM1 = _80_AD_TRIGGER_HARDWARE_NOWAIT | _20_AD_CONVMODE_ONESELECT | _01_AD_TRIGGER_ELC;
        MOV       0xFFF32, #0xA1     ;; 1 cycle
//  137     ADM2 = _80_AD_POSITIVE_INTERVOLT | _00_AD_NEGATIVE_VSS | _00_AD_AREA_MODE_1 | _00_AD_RESOLUTION_10BIT;
        MOV       0x10, #0x80        ;; 1 cycle
//  138     
//  139     ADUL = _FF_AD_ADUL_VALUE;
        MOV       0x11, #0xFF        ;; 1 cycle
//  140     ADLL = _00_AD_ADLL_VALUE;
        MOV       0x12, #0x0         ;; 1 cycle
//  141     ADS = _02_AD_INPUT_CHANNEL_2_5;
        MOV       0xFFF31, #0x2      ;; 1 cycle
//  142     /* Change the waitting time according to the system */
//  143     for (w_count = 0U; w_count < (AD_WAITTIME * MulFactor); w_count++)
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [SP], AX           ;; 1 cycle
        ; ------------------------------------- Block: 27 cycles
??R_ADC_Create_0:
        MOV       X, N:_MulFactor    ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0xD           ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BNC       ??R_ADC_Reset_4    ;; 4 cycles
        ; ------------------------------------- Block: 12 cycles
//  144     {
//  145         NOP();  
        NOP                          ;; 1 cycle
//  146     }
        MOVW      AX, [SP]           ;; 1 cycle
        INCW      AX                 ;; 1 cycle
        MOVW      [SP], AX           ;; 1 cycle
        BR        S:??R_ADC_Create_0  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
//  147     ADCE = 1U;      /* enables A/D voltage comparator operation */
??R_ADC_Reset_4:
        SET1      0xFFF30.0          ;; 2 cycles
//  148     delay_us(1);
        MOVW      AX, #0x1           ;; 1 cycle
          CFI FunCall _delay_us
        CALL      _delay_us          ;; 3 cycles
//  149     R_ADC_Create_UserInit();
          CFI FunCall _R_ADC_Create_UserInit
        CALL      _R_ADC_Create_UserInit  ;; 3 cycles
//  150 }
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock2
        ; ------------------------------------- Block: 16 cycles
        ; ------------------------------------- Total: 62 cycles
        REQUIRE __A_PRR0
        REQUIRE __A_PER0
        REQUIRE __A_ADM0
        REQUIRE __A_MK1
        REQUIRE __A_IF1
        REQUIRE __A_PR11
        REQUIRE __A_PR01
        REQUIRE __A_PM2
        REQUIRE __A_ADM1
        REQUIRE __A_ADM2
        REQUIRE __A_ADUL
        REQUIRE __A_ADLL
        REQUIRE __A_ADS
//  151 /***********************************************************************************************************************
//  152 * Function Name: R_ADC_Start
//  153 * Description  : This function starts the AD converter.
//  154 * Arguments    : None
//  155 * Return Value : None
//  156 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock3 Using cfiCommon0
          CFI Function _R_ADC_Start
          CFI NoCalls
        CODE
//  157 void R_ADC_Start(void)
//  158 {
_R_ADC_Start:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  159     ADIF = 0U;      /* clear INTAD interrupt flag */
        CLR1      0xFFFE3.2          ;; 2 cycles
//  160     ADMK = 1U;      /* disable INTAD interrupt */
        SET1      0xFFFE7.2          ;; 2 cycles
//  161     ADCS = 1U;      /* enables conversion operation */
        SET1      0xFFF30.7          ;; 2 cycles
//  162 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock3
        ; ------------------------------------- Block: 12 cycles
        ; ------------------------------------- Total: 12 cycles
        REQUIRE __A_IF1
        REQUIRE __A_MK1
        REQUIRE __A_ADM0
//  163 /***********************************************************************************************************************
//  164 * Function Name: R_ADC_Stop
//  165 * Description  : This function stops the AD converter.
//  166 * Arguments    : None
//  167 * Return Value : None
//  168 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock4 Using cfiCommon0
          CFI Function _R_ADC_Stop
          CFI NoCalls
        CODE
//  169 void R_ADC_Stop(void)
//  170 {
_R_ADC_Stop:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  171     ADCS = 0U;      /* stops conversion operation */
        CLR1      0xFFF30.7          ;; 2 cycles
//  172     ADMK = 1U;      /* disable INTAD interrupt */
        SET1      0xFFFE7.2          ;; 2 cycles
//  173     ADIF = 0U;      /* clear INTAD interrupt flag */
        CLR1      0xFFFE3.2          ;; 2 cycles
//  174 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock4
        ; ------------------------------------- Block: 12 cycles
        ; ------------------------------------- Total: 12 cycles
        REQUIRE __A_ADM0
        REQUIRE __A_MK1
        REQUIRE __A_IF1
//  175 /***********************************************************************************************************************
//  176 * Function Name: R_ADC_Set_OperationOn
//  177 * Description  : This function enables comparator operation.
//  178 * Arguments    : None
//  179 * Return Value : None
//  180 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock5 Using cfiCommon0
          CFI Function _R_ADC_Set_OperationOn
        CODE
//  181 void R_ADC_Set_OperationOn(void)
//  182 {
_R_ADC_Set_OperationOn:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  183     ADM2 = _80_AD_POSITIVE_INTERVOLT | _00_AD_NEGATIVE_VSS | _00_AD_AREA_MODE_1 | _00_AD_RESOLUTION_10BIT;
        MOV       0x10, #0x80        ;; 1 cycle
//  184     ADCE = 1U;      /* enables A/D voltage comparator operation */
        SET1      0xFFF30.0          ;; 2 cycles
//  185     delay_us(1);
        MOVW      AX, #0x1           ;; 1 cycle
          CFI FunCall _delay_us
        CALL      _delay_us          ;; 3 cycles
//  186 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock5
        ; ------------------------------------- Block: 13 cycles
        ; ------------------------------------- Total: 13 cycles
        REQUIRE __A_ADM2
        REQUIRE __A_ADM0
//  187 /***********************************************************************************************************************
//  188 * Function Name: R_ADC_Set_OperationOff
//  189 * Description  : This function stops comparator operation.
//  190 * Arguments    : None
//  191 * Return Value : None
//  192 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock6 Using cfiCommon0
          CFI Function _R_ADC_Set_OperationOff
          CFI NoCalls
        CODE
//  193 void R_ADC_Set_OperationOff(void)
//  194 {
_R_ADC_Set_OperationOff:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  195     ADM2 = 0x00U;   /* Turn off reference voltage convesve power */
        MOV       0x10, #0x0         ;; 1 cycle
//  196     ADCE = 0U;      /* stops A/D voltage comparator operation */
        CLR1      0xFFF30.0          ;; 2 cycles
//  197 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock6
        ; ------------------------------------- Block: 9 cycles
        ; ------------------------------------- Total: 9 cycles
        REQUIRE __A_ADM2
        REQUIRE __A_ADM0
//  198 /***********************************************************************************************************************
//  199 * Function Name: R_ADC_Get_Result
//  200 * Description  : This function returns the conversion result in the buffer.
//  201 * Arguments    : buffer -
//  202 *                    the address where to write the conversion result
//  203 * Return Value : None
//  204 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock7 Using cfiCommon0
          CFI Function _R_ADC_Get_Result
          CFI NoCalls
        CODE
//  205 void R_ADC_Get_Result(uint16_t * const buffer)
//  206 {
_R_ADC_Get_Result:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
        MOVW      HL, AX             ;; 1 cycle
//  207     *buffer = (uint16_t) (ADCR >> 6U);
        MOVW      AX, S:0xFFF1E      ;; 1 cycle
        SHRW      AX, 0x6            ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
//  208 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock7
        ; ------------------------------------- Block: 10 cycles
        ; ------------------------------------- Total: 10 cycles
        REQUIRE __A_ADCR
//  209 /***********************************************************************************************************************
//  210 * Function Name: R_ADC_Get_Result_8bit
//  211 * Description  : This function returns the higher 8 bits conversion result.
//  212 * Arguments    : buffer -
//  213 *                    the address where to write the conversion result
//  214 * Return Value : None
//  215 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock8 Using cfiCommon0
          CFI Function _R_ADC_Get_Result_8bit
          CFI NoCalls
        CODE
//  216 void R_ADC_Get_Result_8bit(uint8_t * const buffer)
//  217 {
_R_ADC_Get_Result_8bit:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
        MOVW      HL, AX             ;; 1 cycle
//  218     *buffer = ADCRH;
        MOV       A, S:0xFFF1F       ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
//  219 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock8
        ; ------------------------------------- Block: 9 cycles
        ; ------------------------------------- Total: 9 cycles
        REQUIRE __A_ADCR
//  220 /***********************************************************************************************************************
//  221 * Function Name: R_ADC_Set_ADChannel
//  222 * Description  : This function selects analog input channel.
//  223 * Arguments    : channel -
//  224 *                    analog input channel
//  225 * Return Value : status -
//  226 *                    MD_OK or MD_ARGERROR
//  227 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock9 Using cfiCommon0
          CFI Function _R_ADC_Set_ADChannel
          CFI NoCalls
        CODE
//  228 MD_STATUS R_ADC_Set_ADChannel(ad_channel_t channel)
//  229 {
_R_ADC_Set_ADChannel:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
        MOV       B, A               ;; 1 cycle
//  230     MD_STATUS status = MD_OK;
        MOVW      HL, #0x0           ;; 1 cycle
//  231 
//  232     if ((channel > ADCHANNEL5) ||
//  233 		(channel < ADTEMPERSENSOR0) || (channel > ADINTERREFVOLT))
        MOV       A, B               ;; 1 cycle
        CMP       A, #0x6            ;; 1 cycle
        BNC       ??R_ADC_Reset_5    ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
        MOV       A, B               ;; 1 cycle
        CMP       A, #0x80           ;; 1 cycle
        BC        ??R_ADC_Reset_5    ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOV       A, B               ;; 1 cycle
        CMP       A, #0x82           ;; 1 cycle
        BC        ??R_ADC_Reset_6    ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  234     {
//  235         status = MD_ARGERROR;
??R_ADC_Reset_5:
        MOVW      AX, #0x81          ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        BR        S:??R_ADC_Reset_7  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  236     }
//  237     else
//  238     {
//  239         ADS = (uint8_t)channel;
??R_ADC_Reset_6:
        XCH       A, B               ;; 1 cycle
        MOV       0xFFF31, A         ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        ; ------------------------------------- Block: 3 cycles
//  240 
//  241     }
//  242 
//  243     return (status);
??R_ADC_Reset_7:
        MOVW      AX, HL             ;; 1 cycle
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock9
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 35 cycles
        REQUIRE __A_ADS
//  244 }
//  245 /***********************************************************************************************************************
//  246 * Function Name: R_ADC_Set_SnoozeOn
//  247 * Description  : This function enable wakeup function.
//  248 * Arguments    : None
//  249 * Return Value : None
//  250 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock10 Using cfiCommon0
          CFI Function _R_ADC_Set_SnoozeOn
          CFI NoCalls
        CODE
//  251 void R_ADC_Set_SnoozeOn(void)
//  252 {
_R_ADC_Set_SnoozeOn:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  253     AWC = 1U;       /* use SNOOZE function */
        SET1      0xF0010.2          ;; 2 cycles
//  254 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock10
        ; ------------------------------------- Block: 8 cycles
        ; ------------------------------------- Total: 8 cycles
        REQUIRE __A_ADM2
//  255 /***********************************************************************************************************************
//  256 * Function Name: R_ADC_Set_SnoozeOff
//  257 * Description  : This function disable wakeup function.
//  258 * Arguments    : None
//  259 * Return Value : None
//  260 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock11 Using cfiCommon0
          CFI Function _R_ADC_Set_SnoozeOff
          CFI NoCalls
        CODE
//  261 void R_ADC_Set_SnoozeOff(void)
//  262 {
_R_ADC_Set_SnoozeOff:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  263     AWC = 0U;       /* stop SNOOZE function */
        CLR1      0xF0010.2          ;; 2 cycles
//  264 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock11
        ; ------------------------------------- Block: 8 cycles
        ; ------------------------------------- Total: 8 cycles
        REQUIRE __A_ADM2
//  265 /***********************************************************************************************************************
//  266 * Function Name: R_ADC_Set_TestChannel
//  267 * Description  : This function sets test function.
//  268 * Arguments    : channel -
//  269 *                    sets test channel
//  270 * Return Value : status -
//  271 *                    MD_OK or MD_ARGERROR
//  272 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock12 Using cfiCommon0
          CFI Function _R_ADC_Set_TestChannel
          CFI NoCalls
        CODE
//  273 MD_STATUS R_ADC_Set_TestChannel(test_channel_t channel)
//  274 {
_R_ADC_Set_TestChannel:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
        MOV       B, A               ;; 1 cycle
//  275     MD_STATUS status = MD_OK;
        MOVW      HL, #0x0           ;; 1 cycle
//  276 
//  277     if ((1U == channel) || (channel > 3U))
        XCH       A, B               ;; 1 cycle
        CMP       A, #0x1            ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        BZ        ??R_ADC_Reset_8    ;; 4 cycles
        ; ------------------------------------- Block: 9 cycles
        MOV       A, B               ;; 1 cycle
        CMP       A, #0x4            ;; 1 cycle
        BC        ??R_ADC_Reset_9    ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  278     {
//  279         status = MD_ARGERROR;
??R_ADC_Reset_8:
        MOVW      AX, #0x81          ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        BR        S:??R_ADC_Reset_10  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  280     }
//  281     else
//  282     {
//  283         ADTES = (uint8_t)channel;
??R_ADC_Reset_9:
        XCH       A, B               ;; 1 cycle
        MOV       0x13, A            ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        ; ------------------------------------- Block: 3 cycles
//  284     }
//  285 
//  286     return (status);
??R_ADC_Reset_10:
        MOVW      AX, HL             ;; 1 cycle
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock12
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 30 cycles
        REQUIRE __A_ADTES
//  287 }
//  288 /***********************************************************************************************************************
//  289 * Function Name: R_ADC_Set_PowerOff
//  290 * Description  : This function stops the clock supplied for AD.
//  291 * Arguments    : None
//  292 * Return Value : None
//  293 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock13 Using cfiCommon0
          CFI Function _R_ADC_Set_PowerOff
          CFI NoCalls
        CODE
//  294 void R_ADC_Set_PowerOff(void)
//  295 {
_R_ADC_Set_PowerOff:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  296     ADCEN = 0U;     /* stops input clock supply */
        CLR1      0xF00F0.5          ;; 2 cycles
//  297 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock13
        ; ------------------------------------- Block: 8 cycles
        ; ------------------------------------- Total: 8 cycles
        REQUIRE __A_PER0
//  298 /***********************************************************************************************************************
//  299 * Function Name: R_ADC_Reset
//  300 * Description  : This function resets ADC module.
//  301 * Arguments    : None
//  302 * Return Value : None
//  303 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock14 Using cfiCommon0
          CFI Function _R_ADC_Reset
          CFI NoCalls
        CODE
//  304 void R_ADC_Reset(void)
//  305 {
_R_ADC_Reset:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  306     ADCRES = 1U;    /* reset A/D converter */
        SET1      0xF00F1.5          ;; 2 cycles
//  307 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock14
        ; ------------------------------------- Block: 8 cycles
        ; ------------------------------------- Total: 8 cycles
        REQUIRE __A_PRR0

        SECTION `.iar_vfe_header`:DATA:NOALLOC:NOROOT(1)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
        DC32 0

        END
//  308 
//  309 /* Start user code for adding. Do not edit comment generated here */
//  310 /* End user code. Do not edit comment generated here */
// 
//  18 bytes in section .bss.noinit   (abs)
//   3 bytes in section .sbss.noinit  (abs)
// 458 bytes in section .text
// 
// 458 bytes of CODE memory
//   0 bytes of DATA memory (+ 21 bytes shared)
//
//Errors: none
//Warnings: none
