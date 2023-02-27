///////////////////////////////////////////////////////////////////////////////
//
// IAR C/C++ Compiler V4.20.1.2260 for RL78               21/Dec/2020  00:35:24
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
//        BootCode\source_code\source_files\timer.c
//    Command line       =
//        -f C:\Users\laptop\AppData\Local\Temp\EW9B9C.tmp ("D:\Dheeraj\New
//        folder\0. GDEV72 - BootCode\source_code\source_files\timer.c" --core
//        s3 --code_model near --calling_convention v2 --near_const_location
//        ram -o "D:\Dheeraj\New folder\0. GDEV72 - BootCode\Debug\Obj"
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
//        D:\Dheeraj\New folder\0. GDEV72 - BootCode\Debug\List\timer.s
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

        EXTERN _flag2
        EXTERN _last_interrupt
        EXTERN _flag0
        EXTERN _R_IT8Bit0_Channel0_Create
        EXTERN _R_IT8Bit0_Channel0_Start
        EXTERN _R_IT8Bit0_Channel0_Stop
        EXTERN _R_IT8Bit0_Channel1_Create
        EXTERN _R_IT8Bit0_Channel1_Start
        EXTERN _R_IT8Bit0_Channel1_Stop
        EXTERN _R_IT8Bit1_Channel0_Create
        EXTERN _R_IT8Bit1_Channel0_Start
        EXTERN _R_IT8Bit1_Channel0_Stop
        EXTERN ___interrupt_tab_0x58
        EXTERN ___interrupt_tab_0x5A
        EXTERN ___interrupt_tab_0x68

        PUBLIC ___interrupt_0x58
        PUBLIC ___interrupt_0x5A
        PUBLIC ___interrupt_0x68
        PUBLIC _timer_loops_init
        PUBLIC _timer_loops_start
        PUBLIC _timer_loops_stop
        
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
        
// D:\Dheeraj\New folder\0. GDEV72 - BootCode\source_code\source_files\timer.c
//    1 /***********************************************************************************************************************
//    2 * File Name       : timer.c
//    3 * Current Version : rev_01  
//    4 * Tool-Chain      : IAR Systems
//    5 * Description     : This file contains routines for Timer Related functions ( Interrupts routines )
//    6 * Creation Date   : 30-12-2019
//    7 * Company         : Genus Power Infrastructures Limited, Jaipur
//    8 * Author          : dheeraj.singhal
//    9 * Version History : rev_01: Initialisation of timer and implementation of ISR
//   10 ***********************************************************************************************************************/
//   11 /************************************ Includes **************************************/
//   12 #include "timer.h"
//   13 
//   14 /************************************ Local Variables *****************************************/
//   15 /************************************ Extern Variables *****************************************/
//   16 /************************************ Local Functions *******************************/
//   17 /************************************ Extern Functions ******************************/
//   18 void timer_loops_init();
//   19 void timer_loops_start();
//   20 void timer_loops_stop();
//   21 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock0 Using cfiCommon0
          CFI Function _timer_loops_init
          CFI FunCall _R_IT8Bit0_Channel0_Create
        CODE
//   22 void timer_loops_init()
//   23 {   
_timer_loops_init:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//   24     /* 8 Bit interval timer based loops */
//   25     R_IT8Bit0_Channel0_Create();                /* 2ms */
        CALL      _R_IT8Bit0_Channel0_Create  ;; 3 cycles
//   26     R_IT8Bit0_Channel1_Create();                /* 10ms */
          CFI FunCall _R_IT8Bit0_Channel1_Create
        CALL      _R_IT8Bit0_Channel1_Create  ;; 3 cycles
//   27     R_IT8Bit1_Channel0_Create();                /* 1000ms */
          CFI FunCall _R_IT8Bit1_Channel0_Create
        CALL      _R_IT8Bit1_Channel0_Create  ;; 3 cycles
//   28 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock0
        ; ------------------------------------- Block: 15 cycles
        ; ------------------------------------- Total: 15 cycles

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock1 Using cfiCommon0
          CFI Function _timer_loops_start
          CFI FunCall _R_IT8Bit0_Channel0_Start
        CODE
//   29 void timer_loops_start()
//   30 {   
_timer_loops_start:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//   31     R_IT8Bit0_Channel0_Start();
        CALL      _R_IT8Bit0_Channel0_Start  ;; 3 cycles
//   32     R_IT8Bit0_Channel1_Start();
          CFI FunCall _R_IT8Bit0_Channel1_Start
        CALL      _R_IT8Bit0_Channel1_Start  ;; 3 cycles
//   33     R_IT8Bit1_Channel0_Start();
          CFI FunCall _R_IT8Bit1_Channel0_Start
        CALL      _R_IT8Bit1_Channel0_Start  ;; 3 cycles
//   34 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock1
        ; ------------------------------------- Block: 15 cycles
        ; ------------------------------------- Total: 15 cycles

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock2 Using cfiCommon0
          CFI Function _timer_loops_stop
          CFI FunCall _R_IT8Bit0_Channel0_Stop
        CODE
//   35 void timer_loops_stop()
//   36 {   
_timer_loops_stop:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//   37     R_IT8Bit0_Channel0_Stop();
        CALL      _R_IT8Bit0_Channel0_Stop  ;; 3 cycles
//   38     R_IT8Bit0_Channel1_Stop();
          CFI FunCall _R_IT8Bit0_Channel1_Stop
        CALL      _R_IT8Bit0_Channel1_Stop  ;; 3 cycles
//   39     R_IT8Bit1_Channel0_Stop();
          CFI FunCall _R_IT8Bit1_Channel0_Stop
        CALL      _R_IT8Bit1_Channel0_Stop  ;; 3 cycles
//   40 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock2
        ; ------------------------------------- Block: 15 cycles
        ; ------------------------------------- Total: 15 cycles
//   41 /****************************************************************************************
//   42 ****************************  8 Bit Interval Timer Loops  *******************************
//   43 ****************************************************************************************/
//   44 
//   45 /***********************************************************************************************************************
//   46 * Function Name: r_it8bit0_channel0_interrupt
//   47 * Description  : None
//   48 * Arguments    : None
//   49 * Return Value : None
//   50 ***********************************************************************************************************************/
//   51 #pragma vector = INTIT00_vect

        SECTION `.text`:CODE:NOROOT(0)
        CALL_GRAPH_ROOT _r_it8bit0_channel0_interrupt, "interrupt"
          CFI Block cfiBlock3 Using cfiCommon1
          CFI Function _r_it8bit0_channel0_interrupt
          CFI NoCalls
        CODE
//   52 __interrupt static void r_it8bit0_channel0_interrupt(void)
//   53 {
_r_it8bit0_channel0_interrupt:
___interrupt_0x58:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//   54     /* Start user code. Do not edit comment generated here */
//   55     /* 2ms timer */
//   56     two_ms_f = 1;
        SET1      N:_flag2.1         ;; 2 cycles
//   57     last_interrupt = 40;
        MOV       N:_last_interrupt, #0x28  ;; 1 cycle
//   58     /* End user code. Do not edit comment generated here */
//   59 }
        RETI                         ;; 6 cycles
          CFI EndBlock cfiBlock3
        ; ------------------------------------- Block: 9 cycles
        ; ------------------------------------- Total: 9 cycles
        REQUIRE ___interrupt_tab_0x58
//   60 
//   61 
//   62 /***********************************************************************************************************************
//   63 * Function Name: r_it8bit0_channel1_interrupt
//   64 * Description  : None
//   65 * Arguments    : None
//   66 * Return Value : None
//   67 ***********************************************************************************************************************/
//   68 #pragma vector = INTIT01_vect

        SECTION `.text`:CODE:NOROOT(0)
        CALL_GRAPH_ROOT _r_it8bit0_channel1_interrupt, "interrupt"
          CFI Block cfiBlock4 Using cfiCommon1
          CFI Function _r_it8bit0_channel1_interrupt
          CFI NoCalls
        CODE
//   69 __interrupt static void r_it8bit0_channel1_interrupt(void)
//   70 {
_r_it8bit0_channel1_interrupt:
___interrupt_0x5A:
        ; * Stack frame (at entry) *
        ; Param size: 0
        PUSH      AX                 ;; 1 cycle
          CFI X Frame(CFA, -6)
          CFI A Frame(CFA, -5)
          CFI CFA SP+6
        ; Auto size: 0
//   71     /* Start user code. Do not edit comment generated here */
//   72     /* 10ms timer */
//   73     static us8 cntr_10_ms = 0;
//   74     last_interrupt = 41;
        MOV       N:_last_interrupt, #0x29  ;; 1 cycle
//   75     ten_ms_f = 1;
        SET1      N:_flag0.0         ;; 2 cycles
//   76     cntr_10_ms++;
        INC       N:`r_it8bit0_channel1_interrupt::cntr_10_ms`  ;; 2 cycles
//   77     if(cntr_10_ms >= 10)
        MOV       A, N:`r_it8bit0_channel1_interrupt::cntr_10_ms`  ;; 1 cycle
        CMP       A, #0xA            ;; 1 cycle
        BC        ??r_it8bit1_channel0_interrupt_0  ;; 4 cycles
        ; ------------------------------------- Block: 12 cycles
//   78     {
//   79         cntr_10_ms = 0;
        MOV       N:`r_it8bit0_channel1_interrupt::cntr_10_ms`, #0x0  ;; 1 cycle
//   80         hundred_ms_f = 1;
        SET1      N:_flag0.1         ;; 2 cycles
        ; ------------------------------------- Block: 3 cycles
//   81     }
//   82     /* End user code. Do not edit comment generated here */
//   83 }
??r_it8bit1_channel0_interrupt_0:
        POP       AX                 ;; 1 cycle
          CFI X SameValue
          CFI A SameValue
          CFI CFA SP+4
        RETI                         ;; 6 cycles
          CFI EndBlock cfiBlock4
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 22 cycles
        REQUIRE ___interrupt_tab_0x5A

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
`r_it8bit0_channel1_interrupt::cntr_10_ms`:
        DS 1
//   84 
//   85 /***********************************************************************************************************************
//   86 * Function Name: r_it8bit1_channel0_interrupt
//   87 * Description  : None
//   88 * Arguments    : None
//   89 * Return Value : None
//   90 ***********************************************************************************************************************/
//   91 #pragma vector = INTIT10_vect

        SECTION `.text`:CODE:NOROOT(0)
        CALL_GRAPH_ROOT _r_it8bit1_channel0_interrupt, "interrupt"
          CFI Block cfiBlock5 Using cfiCommon1
          CFI Function _r_it8bit1_channel0_interrupt
          CFI NoCalls
        CODE
//   92 __interrupt static void r_it8bit1_channel0_interrupt(void)
//   93 {
_r_it8bit1_channel0_interrupt:
___interrupt_0x68:
        ; * Stack frame (at entry) *
        ; Param size: 0
        PUSH      AX                 ;; 1 cycle
          CFI X Frame(CFA, -6)
          CFI A Frame(CFA, -5)
          CFI CFA SP+6
        PUSH      DE                 ;; 1 cycle
          CFI E Frame(CFA, -8)
          CFI D Frame(CFA, -7)
          CFI CFA SP+8
        PUSH      HL                 ;; 1 cycle
          CFI L Frame(CFA, -10)
          CFI H Frame(CFA, -9)
          CFI CFA SP+10
        ; Auto size: 0
//   94   /* 1 sec timer , 1 min timer, 1 hr timer */
//   95   static us16 cntr_1_sec = 0;
//   96   last_interrupt = 42;
        MOV       N:_last_interrupt, #0x2A  ;; 1 cycle
//   97   one_sec_f = 1;
        SET1      N:_flag0.3         ;; 2 cycles
//   98   
//   99   /* flags setting */
//  100   cntr_1_sec++;
        INCW      N:`r_it8bit1_channel0_interrupt::cntr_1_sec`  ;; 2 cycles
//  101   
//  102   if((cntr_1_sec % 2) == 0)
        MOVW      HL, #LWRD(`r_it8bit1_channel0_interrupt::cntr_1_sec`)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        SKC                          ;; 1 cycle
        ; ------------------------------------- Block: 11 cycles
//  103   {
//  104       two_sec_f = 1;
        SET1      N:_flag2.0         ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
//  105   }
//  106   if((cntr_1_sec % 5) == 0)
??r_it8bit1_channel0_interrupt_1:
        MOVW      DE, #0x5           ;; 1 cycle
        MOVW      AX, N:`r_it8bit1_channel0_interrupt::cntr_1_sec`  ;; 1 cycle
        DIVHU                        ;; 9 cycles
        NOP                          ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        CMPW      AX, #0x0           ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 16 cycles
//  107   {
//  108       five_sec_f = 1;
        SET1      N:_flag0.4         ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
//  109   }
//  110   if((cntr_1_sec % 60) == 0)
??r_it8bit1_channel0_interrupt_2:
        MOVW      DE, #0x3C          ;; 1 cycle
        MOVW      AX, N:`r_it8bit1_channel0_interrupt::cntr_1_sec`  ;; 1 cycle
        DIVHU                        ;; 9 cycles
        NOP                          ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        CMPW      AX, #0x0           ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 16 cycles
//  111   {
//  112       one_min_f = 1;
        SET1      N:_flag0.5         ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
//  113   }
//  114   if((cntr_1_sec % 300) == 0)
??r_it8bit1_channel0_interrupt_3:
        MOVW      DE, #0x12C         ;; 1 cycle
        MOVW      AX, N:`r_it8bit1_channel0_interrupt::cntr_1_sec`  ;; 1 cycle
        DIVHU                        ;; 9 cycles
        NOP                          ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        CMPW      AX, #0x0           ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 16 cycles
//  115   {
//  116       five_min_f = 1;
        SET1      N:_flag0.6         ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
//  117   }
//  118   if((cntr_1_sec % 3600) == 0)
??r_it8bit1_channel0_interrupt_4:
        MOVW      DE, #0xE10         ;; 1 cycle
        MOVW      AX, N:`r_it8bit1_channel0_interrupt::cntr_1_sec`  ;; 1 cycle
        DIVHU                        ;; 9 cycles
        NOP                          ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        CMPW      AX, #0x0           ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        BNZ       ??r_it8bit1_channel0_interrupt_5  ;; 4 cycles
        ; ------------------------------------- Block: 19 cycles
//  119   {
//  120       one_hr_f = 1;
        SET1      N:_flag0.7         ;; 2 cycles
//  121       cntr_1_sec = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:`r_it8bit1_channel0_interrupt::cntr_1_sec`, AX  ;; 1 cycle
        ; ------------------------------------- Block: 4 cycles
//  122   }
//  123 }
??r_it8bit1_channel0_interrupt_5:
        POP       HL                 ;; 1 cycle
          CFI L SameValue
          CFI H SameValue
          CFI CFA SP+8
        POP       DE                 ;; 1 cycle
          CFI E SameValue
          CFI D SameValue
          CFI CFA SP+6
        POP       AX                 ;; 1 cycle
          CFI X SameValue
          CFI A SameValue
          CFI CFA SP+4
        RETI                         ;; 6 cycles
          CFI EndBlock cfiBlock5
        ; ------------------------------------- Block: 9 cycles
        ; ------------------------------------- Total: 99 cycles
        REQUIRE ___interrupt_tab_0x68

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
`r_it8bit1_channel0_interrupt::cntr_1_sec`:
        DS 2

        SECTION `.iar_vfe_header`:DATA:NOALLOC:NOROOT(1)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
        DC32 0

        END
// 
//   3 bytes in section .bss
// 190 bytes in section .text
// 
// 190 bytes of CODE memory
//   3 bytes of DATA memory
//
//Errors: none
//Warnings: none
