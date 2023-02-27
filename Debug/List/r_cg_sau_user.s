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
//        BootCode\source_code\driver_files\r_cg_sau_user.c
//    Command line       =
//        -f C:\Users\17054\AppData\Local\Temp\EWDCE1.tmp ("D:\Software
//        code\Projects Software\Projects new\0. Non AMI\SUGAM Based\0. GDEV72
//        - BootCode-20210104T103109Z-001\0. GDEV72 -
//        BootCode\source_code\driver_files\r_cg_sau_user.c" --core s3
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
//        BootCode\Debug\List\r_cg_sau_user.s
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
        EXTERN ___interrupt_tab_0x14
        EXTERN ___interrupt_tab_0x16
        EXTERN ___interrupt_tab_0x18
        EXTERN ___interrupt_tab_0x26
        EXTERN ___interrupt_tab_0x28
        EXTERN ___interrupt_tab_0x2A
        EXTERN _gp_uart1_rx_address
        EXTERN _gp_uart2_rx_address

        PUBLIC __A_SDR03
        PUBLIC __A_SDR11
        PUBLIC __A_SIR03
        PUBLIC __A_SIR11
        PUBLIC __A_SS0
        PUBLIC __A_SS1
        PUBLIC __A_SSR03
        PUBLIC __A_SSR11
        PUBLIC __A_ST0
        PUBLIC __A_ST1
        PUBLIC ___interrupt_0x14
        PUBLIC ___interrupt_0x16
        PUBLIC ___interrupt_0x18
        PUBLIC ___interrupt_0x26
        PUBLIC ___interrupt_0x28
        PUBLIC ___interrupt_0x2A
        
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
        
// D:\Software code\Projects Software\Projects new\0. Non AMI\SUGAM Based\0. GDEV72 - BootCode-20210104T103109Z-001\0. GDEV72 - BootCode\source_code\driver_files\r_cg_sau_user.c
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
//   21 * File Name    : r_cg_sau_user.c
//   22 * Version      : Applilet4 for RL78/I1C V1.01.04.02 [20 Nov 2019]
//   23 * Device(s)    : R5F10NPJ
//   24 * Tool-Chain   : IAR Systems icc78k0r
//   25 * Description  : This file implements device driver for SAU module.
//   26 * Creation Date: 06/05/2020
//   27 ***********************************************************************************************************************/
//   28 
//   29 /***********************************************************************************************************************
//   30 Includes
//   31 ***********************************************************************************************************************/
//   32 #include "r_cg_macrodriver.h"

        ASEGN `.bss.noinit`:DATA:NOROOT,0fff46H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_SDR03
// __no_init union <unnamed>#68 volatile __sfr __no_bit_access _A_SDR03
__A_SDR03:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0fff4aH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_SDR11
// __no_init union <unnamed>#74 volatile __sfr __no_bit_access _A_SDR11
__A_SDR11:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0106H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_SSR03
// __no_init union <unnamed>#305 const volatile __no_bit_access _A_SSR03
__A_SSR03:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f010eH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_SIR03
// __no_init union <unnamed>#317 volatile __no_bit_access _A_SIR03
__A_SIR03:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0122H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_SS0
// __no_init union <unnamed>#331 volatile _A_SS0
__A_SS0:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0124H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_ST0
// __no_init union <unnamed>#334 volatile _A_ST0
__A_ST0:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0142H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_SSR11
// __no_init union <unnamed>#353 const volatile __no_bit_access _A_SSR11
__A_SSR11:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f014aH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_SIR11
// __no_init union <unnamed>#365 volatile __no_bit_access _A_SIR11
__A_SIR11:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0162H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_SS1
// __no_init union <unnamed>#385 volatile _A_SS1
__A_SS1:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0164H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_ST1
// __no_init union <unnamed>#388 volatile _A_ST1
__A_ST1:
        DS 2
//   33 #include "r_cg_sau.h"
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
//   46 extern uint8_t * gp_uart1_tx_address;         /* uart1 send buffer address */
//   47 extern uint16_t  g_uart1_tx_count;            /* uart1 send data number */
//   48 extern uint8_t * gp_uart1_rx_address;         /* uart1 receive buffer address */
//   49 extern uint16_t  g_uart1_rx_count;            /* uart1 receive data number */
//   50 extern uint16_t  g_uart1_rx_length;           /* uart1 receive data length */
//   51 extern uint8_t * gp_uart2_tx_address;         /* uart2 send buffer address */
//   52 extern uint16_t  g_uart2_tx_count;            /* uart2 send data number */
//   53 extern uint8_t * gp_uart2_rx_address;         /* uart2 receive buffer address */
//   54 extern uint16_t  g_uart2_rx_count;            /* uart2 receive data number */
//   55 extern uint16_t  g_uart2_rx_length;           /* uart2 receive data length */
//   56 /* Start user code for global. Do not edit comment generated here */
//   57 /* End user code. Do not edit comment generated here */
//   58 
//   59 /***********************************************************************************************************************
//   60 * Function Name: r_uart1_interrupt_receive
//   61 * Description  : RJ45 Port
//   62 * Arguments    : None
//   63 * Return Value : None
//   64 ***********************************************************************************************************************/
//   65 #pragma vector = INTSR1_vect

        SECTION `.text`:CODE:NOROOT(0)
        CALL_GRAPH_ROOT _r_uart1_interrupt_receive, "interrupt"
          CFI Block cfiBlock0 Using cfiCommon0
          CFI Function _r_uart1_interrupt_receive
          CFI NoCalls
        CODE
//   66 __interrupt static void r_uart1_interrupt_receive(void)
//   67 {
_r_uart1_interrupt_receive:
___interrupt_0x28:
        ; * Stack frame (at entry) *
        ; Param size: 0
        PUSH      AX                 ;; 1 cycle
          CFI X Frame(CFA, -6)
          CFI A Frame(CFA, -5)
          CFI CFA SP+6
        ; Auto size: 2
        SUBW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+8
//   68   volatile uint8_t rx_data;
//   69   last_interrupt = 25;
        MOV       N:_last_interrupt, #0x19  ;; 1 cycle
//   70   rx_data = ReceiveRJ45();
        MOV       A, 0xFFF46         ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//   71   //rcv_rj45(rx_data);
//   72 }
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+6
        POP       AX                 ;; 1 cycle
          CFI X SameValue
          CFI A SameValue
          CFI CFA SP+4
        RETI                         ;; 6 cycles
          CFI EndBlock cfiBlock0
        ; ------------------------------------- Block: 13 cycles
        ; ------------------------------------- Total: 13 cycles
        REQUIRE __A_SDR03
        REQUIRE ___interrupt_tab_0x28
//   73 /***********************************************************************************************************************
//   74 * Function Name: r_uart1_interrupt_error
//   75 * Description  : None
//   76 * Arguments    : None
//   77 * Return Value : None
//   78 ***********************************************************************************************************************/
//   79 #pragma vector = INTSRE1_vect

        SECTION `.text`:CODE:NOROOT(0)
        CALL_GRAPH_ROOT _r_uart1_interrupt_error, "interrupt"
          CFI Block cfiBlock1 Using cfiCommon0
          CFI Function _r_uart1_interrupt_error
          CFI NoCalls
        CODE
//   80 __interrupt static void r_uart1_interrupt_error(void)
//   81 {
_r_uart1_interrupt_error:
___interrupt_0x2A:
        ; * Stack frame (at entry) *
        ; Param size: 0
        PUSH      AX                 ;; 1 cycle
          CFI X Frame(CFA, -6)
          CFI A Frame(CFA, -5)
          CFI CFA SP+6
        PUSH      HL                 ;; 1 cycle
          CFI L Frame(CFA, -8)
          CFI H Frame(CFA, -7)
          CFI CFA SP+8
        ; Auto size: 2
        SUBW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+10
//   82     volatile uint8_t err_type;
//   83     last_interrupt = 26;
        MOV       N:_last_interrupt, #0x1A  ;; 1 cycle
//   84     *gp_uart1_rx_address = RXD1;
        MOVW      HL, N:_gp_uart1_rx_address  ;; 1 cycle
        MOV       A, 0xFFF46         ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
//   85     err_type = (uint8_t)(SSR03 & 0x0007U);
        MOVW      AX, 0x106          ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        AND       A, #0x7            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//   86     SIR03 = (uint16_t)err_type;
        MOV       A, [SP]            ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      0x10E, AX          ;; 1 cycle
//   87     
//   88     if((err_type & BIT2)!=0)
        MOV       A, [SP]            ;; 1 cycle
        AND       A, #0x4            ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        BZ        ??r_uart2_interrupt_send_0  ;; 4 cycles
        ; ------------------------------------- Block: 22 cycles
//   89     {
//   90         ST0 |= _0008_SAUm_CH3_STOP_TRG_ON | _0004_SAUm_CH2_STOP_TRG_ON;
        MOVW      AX, 0x124          ;; 1 cycle
        OR        A, #0x0            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        OR        A, #0xC            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      0x124, AX          ;; 1 cycle
//   91         NOP();
        NOP                          ;; 1 cycle
//   92         NOP();
        NOP                          ;; 1 cycle
//   93         SS0 |= _0008_SAUm_CH3_START_TRG_ON | _0004_SAUm_CH2_START_TRG_ON;
        MOVW      AX, 0x122          ;; 1 cycle
        OR        A, #0x0            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        OR        A, #0xC            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      0x122, AX          ;; 1 cycle
        ; ------------------------------------- Block: 14 cycles
//   94     }
//   95 }
??r_uart2_interrupt_send_0:
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+8
        POP       HL                 ;; 1 cycle
          CFI L SameValue
          CFI H SameValue
          CFI CFA SP+6
        POP       AX                 ;; 1 cycle
          CFI X SameValue
          CFI A SameValue
          CFI CFA SP+4
        RETI                         ;; 6 cycles
          CFI EndBlock cfiBlock1
        ; ------------------------------------- Block: 9 cycles
        ; ------------------------------------- Total: 45 cycles
        REQUIRE __A_SDR03
        REQUIRE __A_SSR03
        REQUIRE __A_SIR03
        REQUIRE __A_ST0
        REQUIRE __A_SS0
        REQUIRE ___interrupt_tab_0x2A
//   96 /***********************************************************************************************************************
//   97 * Function Name: r_uart1_interrupt_send
//   98 * Description  : None
//   99 * Arguments    : None
//  100 * Return Value : None
//  101 ***********************************************************************************************************************/
//  102 #pragma vector = INTST1_vect

        SECTION `.text`:CODE:NOROOT(0)
        CALL_GRAPH_ROOT _r_uart1_interrupt_send, "interrupt"
          CFI Block cfiBlock2 Using cfiCommon0
          CFI Function _r_uart1_interrupt_send
          CFI NoCalls
        CODE
//  103 __interrupt static void r_uart1_interrupt_send(void)
//  104 {
_r_uart1_interrupt_send:
___interrupt_0x26:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  105   last_interrupt = 27;
        MOV       N:_last_interrupt, #0x1B  ;; 1 cycle
//  106   //transmit_rj45();
//  107 }
        RETI                         ;; 6 cycles
          CFI EndBlock cfiBlock2
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 7 cycles
        REQUIRE ___interrupt_tab_0x26
//  108 /***********************************************************************************************************************
//  109 * Function Name: r_uart1_callback_receiveend
//  110 * Description  : This function is a callback function when UART1 finishes reception.
//  111 * Arguments    : None
//  112 * Return Value : None
//  113 ***********************************************************************************************************************/
//  114 //static void r_uart1_callback_receiveend(void)
//  115 //{
//  116 //  /* Start user code. Do not edit comment generated here */
//  117 //  /* End user code. Do not edit comment generated here */
//  118 //}
//  119 /***********************************************************************************************************************
//  120 * Function Name: r_uart1_callback_softwareoverrun
//  121 * Description  : This function is a callback function when UART1 receives an overflow data.
//  122 * Arguments    : rx_data -
//  123 *                    receive data
//  124 * Return Value : None
//  125 ***********************************************************************************************************************/
//  126 //static void r_uart1_callback_softwareoverrun(uint16_t rx_data)
//  127 //{
//  128 //  /* Start user code. Do not edit comment generated here */
//  129 //  /* End user code. Do not edit comment generated here */
//  130 //}
//  131 /***********************************************************************************************************************
//  132 * Function Name: r_uart1_callback_sendend
//  133 * Description  : This function is a callback function when UART1 finishes transmission.
//  134 * Arguments    : None
//  135 * Return Value : None
//  136 ***********************************************************************************************************************/
//  137 //static void r_uart1_callback_sendend(void)
//  138 //{
//  139 //  /* Start user code. Do not edit comment generated here */
//  140 //  /* End user code. Do not edit comment generated here */
//  141 //}
//  142 /***********************************************************************************************************************
//  143 * Function Name: r_uart1_callback_error
//  144 * Description  : This function is a callback function when UART1 reception error occurs.
//  145 * Arguments    : err_type -
//  146 *                    error type value
//  147 * Return Value : None
//  148 ***********************************************************************************************************************/
//  149 //static void r_uart1_callback_error(uint8_t err_type)
//  150 //{
//  151 //  /* Start user code. Do not edit comment generated here */
//  152 //  /* End user code. Do not edit comment generated here */
//  153 //}
//  154 /***********************************************************************************************************************
//  155 * Function Name: r_uart2_interrupt_receive
//  156 * Description  : Optical port
//  157 * Arguments    : None
//  158 * Return Value : None
//  159 ***********************************************************************************************************************/
//  160 #pragma vector = INTSR2_vect

        SECTION `.text`:CODE:NOROOT(0)
        CALL_GRAPH_ROOT _r_uart2_interrupt_receive, "interrupt"
          CFI Block cfiBlock3 Using cfiCommon0
          CFI Function _r_uart2_interrupt_receive
          CFI NoCalls
        CODE
//  161 __interrupt static void r_uart2_interrupt_receive(void)
//  162 {
_r_uart2_interrupt_receive:
___interrupt_0x16:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  163   last_interrupt = 28;
        MOV       N:_last_interrupt, #0x1C  ;; 1 cycle
//  164 //  rcv_optical(ReceiveOptical());  
//  165 }
        RETI                         ;; 6 cycles
          CFI EndBlock cfiBlock3
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 7 cycles
        REQUIRE ___interrupt_tab_0x16
//  166 /***********************************************************************************************************************
//  167 * Function Name: r_uart2_interrupt_error
//  168 * Description  : None
//  169 * Arguments    : None
//  170 * Return Value : None
//  171 ***********************************************************************************************************************/
//  172 #pragma vector = INTSRE2_vect

        SECTION `.text`:CODE:NOROOT(0)
        CALL_GRAPH_ROOT _r_uart2_interrupt_error, "interrupt"
          CFI Block cfiBlock4 Using cfiCommon0
          CFI Function _r_uart2_interrupt_error
          CFI NoCalls
        CODE
//  173 __interrupt static void r_uart2_interrupt_error(void)
//  174 {
_r_uart2_interrupt_error:
___interrupt_0x18:
        ; * Stack frame (at entry) *
        ; Param size: 0
        PUSH      AX                 ;; 1 cycle
          CFI X Frame(CFA, -6)
          CFI A Frame(CFA, -5)
          CFI CFA SP+6
        PUSH      HL                 ;; 1 cycle
          CFI L Frame(CFA, -8)
          CFI H Frame(CFA, -7)
          CFI CFA SP+8
        ; Auto size: 2
        SUBW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+10
//  175     volatile uint8_t err_type;
//  176     last_interrupt = 29;
        MOV       N:_last_interrupt, #0x1D  ;; 1 cycle
//  177     *gp_uart2_rx_address = RXD2;
        MOVW      HL, N:_gp_uart2_rx_address  ;; 1 cycle
        MOV       A, 0xFFF4A         ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
//  178     err_type = (uint8_t)(SSR11 & 0x0007U);
        MOVW      AX, 0x142          ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        AND       A, #0x7            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  179     SIR11 = (uint16_t)err_type;
        MOV       A, [SP]            ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      0x14A, AX          ;; 1 cycle
//  180 
//  181     if((err_type & BIT2)!=0)
        MOV       A, [SP]            ;; 1 cycle
        AND       A, #0x4            ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        BZ        ??r_uart2_interrupt_send_1  ;; 4 cycles
        ; ------------------------------------- Block: 22 cycles
//  182     {
//  183         ST1 |= _0002_SAUm_CH1_STOP_TRG_ON | _0001_SAUm_CH0_STOP_TRG_ON;
        MOVW      AX, 0x164          ;; 1 cycle
        OR        A, #0x0            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        OR        A, #0x3            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      0x164, AX          ;; 1 cycle
//  184         NOP();
        NOP                          ;; 1 cycle
//  185         NOP();
        NOP                          ;; 1 cycle
//  186         SS1 |= _0002_SAUm_CH1_START_TRG_ON | _0001_SAUm_CH0_START_TRG_ON;
        MOVW      AX, 0x162          ;; 1 cycle
        OR        A, #0x0            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        OR        A, #0x3            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      0x162, AX          ;; 1 cycle
        ; ------------------------------------- Block: 14 cycles
//  187     }
//  188 }
??r_uart2_interrupt_send_1:
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+8
        POP       HL                 ;; 1 cycle
          CFI L SameValue
          CFI H SameValue
          CFI CFA SP+6
        POP       AX                 ;; 1 cycle
          CFI X SameValue
          CFI A SameValue
          CFI CFA SP+4
        RETI                         ;; 6 cycles
          CFI EndBlock cfiBlock4
        ; ------------------------------------- Block: 9 cycles
        ; ------------------------------------- Total: 45 cycles
        REQUIRE __A_SDR11
        REQUIRE __A_SSR11
        REQUIRE __A_SIR11
        REQUIRE __A_ST1
        REQUIRE __A_SS1
        REQUIRE ___interrupt_tab_0x18
//  189 /***********************************************************************************************************************
//  190 * Function Name: r_uart2_interrupt_send
//  191 * Description  : None
//  192 * Arguments    : None
//  193 * Return Value : None
//  194 ***********************************************************************************************************************/
//  195 #pragma vector = INTST2_vect

        SECTION `.text`:CODE:NOROOT(0)
        CALL_GRAPH_ROOT _r_uart2_interrupt_send, "interrupt"
          CFI Block cfiBlock5 Using cfiCommon0
          CFI Function _r_uart2_interrupt_send
          CFI NoCalls
        CODE
//  196 __interrupt static void r_uart2_interrupt_send(void)
//  197 {
_r_uart2_interrupt_send:
___interrupt_0x14:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  198   last_interrupt = 30;
        MOV       N:_last_interrupt, #0x1E  ;; 1 cycle
//  199 //  transmit_optical();
//  200 }
        RETI                         ;; 6 cycles
          CFI EndBlock cfiBlock5
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 7 cycles
        REQUIRE ___interrupt_tab_0x14

        SECTION `.iar_vfe_header`:DATA:NOALLOC:NOROOT(1)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
        DC32 0

        END
//  201 /***********************************************************************************************************************
//  202 * Function Name: r_uart2_callback_receiveend
//  203 * Description  : This function is a callback function when UART2 finishes reception.
//  204 * Arguments    : None
//  205 * Return Value : None
//  206 ***********************************************************************************************************************/
//  207 //static void r_uart2_callback_receiveend(void)
//  208 //{
//  209 //  /* Start user code. Do not edit comment generated here */
//  210 //  /* End user code. Do not edit comment generated here */
//  211 //}
//  212 /***********************************************************************************************************************
//  213 * Function Name: r_uart2_callback_softwareoverrun
//  214 * Description  : This function is a callback function when UART2 receives an overflow data.
//  215 * Arguments    : rx_data -
//  216 *                    receive data
//  217 * Return Value : None
//  218 ***********************************************************************************************************************/
//  219 //static void r_uart2_callback_softwareoverrun(uint16_t rx_data)
//  220 //{
//  221 //  /* Start user code. Do not edit comment generated here */
//  222 //  /* End user code. Do not edit comment generated here */
//  223 //}
//  224 /***********************************************************************************************************************
//  225 * Function Name: r_uart2_callback_sendend
//  226 * Description  : This function is a callback function when UART2 finishes transmission.
//  227 * Arguments    : None
//  228 * Return Value : None
//  229 ***********************************************************************************************************************/
//  230 //static void r_uart2_callback_sendend(void)
//  231 //{
//  232 //  /* Start user code. Do not edit comment generated here */
//  233 //  flag_optical_sending = 0;
//  234 //  /* End user code. Do not edit comment generated here */
//  235 //}
//  236 /***********************************************************************************************************************
//  237 * Function Name: r_uart2_callback_error
//  238 * Description  : This function is a callback function when UART2 reception error occurs.
//  239 * Arguments    : err_type -
//  240 *                    error type value
//  241 * Return Value : None
//  242 ***********************************************************************************************************************/
//  243 //static void r_uart2_callback_error(uint8_t err_type)
//  244 //{
//  245 //  /* Start user code. Do not edit comment generated here */
//  246 //  /* End user code. Do not edit comment generated here */
//  247 //}
//  248 
//  249 /* Start user code for adding. Do not edit comment generated here */
//  250 /* End user code. Do not edit comment generated here */
// 
//  20 bytes in section .bss.noinit  (abs)
// 172 bytes in section .text
// 
// 172 bytes of CODE memory
//   0 bytes of DATA memory (+ 20 bytes shared)
//
//Errors: none
//Warnings: none
