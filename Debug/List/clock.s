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
//        BootCode\source_code\source_files\clock.c
//    Command line       =
//        -f C:\Users\17054\AppData\Local\Temp\EWD833.tmp ("D:\Software
//        code\Projects Software\Projects new\0. Non AMI\SUGAM Based\0. GDEV72
//        - BootCode-20210104T103109Z-001\0. GDEV72 -
//        BootCode\source_code\source_files\clock.c" --core s3 --code_model
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
//        BootCode\Debug\List\clock.s
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

        EXTERN _R_CGC_Create

        PUBLIC _MulFactor
        PUBLIC __A_CKC
        PUBLIC __A_HOCODIV
        PUBLIC _clock_change
        PUBLIC _clock_init
        PUBLIC _clock_select
        PUBLIC _flag_clock
        
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
        
// D:\Software code\Projects Software\Projects new\0. Non AMI\SUGAM Based\0. GDEV72 - BootCode-20210104T103109Z-001\0. GDEV72 - BootCode\source_code\source_files\clock.c
//    1 /***********************************************************************************************************************
//    2 * File Name       : clock.c
//    3 * Current Version : rev_01  
//    4 * Tool-Chain      : IAR Systems
//    5 * Description     : 
//    6 * Creation Date   : 02-01-2020
//    7 * Company         : Genus Power Infrastructures Limited, Jaipur
//    8 * Author          : dheeraj.singhal
//    9 * Version History : rev_01 : initial fw release
//   10 ***********************************************************************************************************************/
//   11 /************************************ Includes **************************************/
//   12 #include "clock.h"

        ASEGN `.bss.noinit`:DATA:NOROOT,0fffa4H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_CKC
// __no_init union <unnamed>#97 volatile __sfr _A_CKC
__A_CKC:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f00a8H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_HOCODIV
// __no_init union <unnamed>#266 volatile __no_bit_access _A_HOCODIV
__A_HOCODIV:
        DS 1
//   13 
//   14 /************************************ Local Variables *****************************************/
//   15 /************************************ Extern Variables *****************************************/

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   16 us8 clock_select,MulFactor; 
_clock_select:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_MulFactor:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   17 flag_union flag_clock;
_flag_clock:
        DS 1
//   18 /************************************ Local Functions *******************************/
//   19 /************************************ Extern Functions ******************************/
//   20 void clock_init();
//   21 void clock_change(us8 clock);
//   22 
//   23 
//   24 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock0 Using cfiCommon0
          CFI Function _clock_init
          CFI FunCall _R_CGC_Create
        CODE
//   25 void clock_init()
//   26 {
_clock_init:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//   27     R_CGC_Create();
        CALL      _R_CGC_Create      ;; 3 cycles
//   28     clock_select = CLOCK_24MHZ;
        MOV       N:_clock_select, #0x1  ;; 1 cycle
//   29     MulFactor = 1;
        MOV       N:_MulFactor, #0x1  ;; 1 cycle
//   30 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock0
        ; ------------------------------------- Block: 11 cycles
        ; ------------------------------------- Total: 11 cycles
//   31 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock1 Using cfiCommon0
          CFI Function _clock_change
          CFI NoCalls
        CODE
//   32 void clock_change(us8 clock)
//   33 { 
_clock_change:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//   34   if(clock == CLOCK_24MHZ)
        CMP       A, #0x1            ;; 1 cycle
        BNZ       ??clock_change_0   ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//   35   {
//   36     clock_select = CLOCK_24MHZ;
        MOV       N:_clock_select, #0x1  ;; 1 cycle
//   37     HOCODIV = 0x00;
        MOV       0xA8, #0x0         ;; 1 cycle
//   38     MulFactor = 1;
        MOV       N:_MulFactor, #0x1  ;; 1 cycle
        BR        S:??clock_change_1  ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
//   39   }
//   40   else if(clock == CLOCK_12MHZ)
??clock_change_0:
        CMP       A, #0x2            ;; 1 cycle
        BNZ       ??clock_change_2   ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//   41   {
//   42     clock_select = CLOCK_12MHZ;
        MOV       N:_clock_select, #0x2  ;; 1 cycle
//   43     HOCODIV = 0x01;
        MOV       0xA8, #0x1         ;; 1 cycle
//   44     MulFactor = 2;
        MOV       N:_MulFactor, #0x2  ;; 1 cycle
        BR        S:??clock_change_1  ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
//   45   }
//   46   else if(clock == CLOCK_6MHZ)
??clock_change_2:
        CMP       A, #0x3            ;; 1 cycle
        BNZ       ??clock_change_3   ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//   47   {
//   48     clock_select = CLOCK_6MHZ;
        MOV       N:_clock_select, #0x3  ;; 1 cycle
//   49     HOCODIV = 0x02;
        MOV       0xA8, #0x2         ;; 1 cycle
//   50     MulFactor = 4;
        MOV       N:_MulFactor, #0x4  ;; 1 cycle
        BR        S:??clock_change_1  ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
//   51   }
//   52   else if(clock == CLOCK_1_5MHZ)
??clock_change_3:
        CMP       A, #0x4            ;; 1 cycle
        BNZ       ??clock_change_4   ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//   53   {
//   54     clock_select = CLOCK_1_5MHZ;
        MOV       N:_clock_select, #0x4  ;; 1 cycle
//   55     HOCODIV = 0x04;
        MOV       0xA8, #0x4         ;; 1 cycle
//   56     MulFactor = 16;
        MOV       N:_MulFactor, #0x10  ;; 1 cycle
        BR        S:??clock_change_1  ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
//   57   }
//   58   else                          /* default 24MHZ */
//   59   {
//   60     clock_select = CLOCK_1_5MHZ;
??clock_change_4:
        MOV       N:_clock_select, #0x4  ;; 1 cycle
//   61     HOCODIV = 0x00;
        MOV       0xA8, #0x0         ;; 1 cycle
//   62     MulFactor = 1;
        MOV       N:_MulFactor, #0x1  ;; 1 cycle
        ; ------------------------------------- Block: 3 cycles
//   63   }
//   64   
//   65   /* Set fCLK */
//   66   CSS = 0U;       /* main system clock (fMAIN) */
??clock_change_1:
        CLR1      0xFFFA4.6          ;; 2 cycles
//   67   
//   68   /* Set fMAIN */
//   69   MCM0 = 0U;      /* selects the main on-chip oscillator clock (fOCO) or PLL clock (fPLL) as the main system clock (fMAIN) */
        CLR1      0xFFFA4.4          ;; 2 cycles
//   70   
//   71   /* Set fMAIN Control */
//   72   MCM1 = 0U;      /* high-speed on-chip oscillator clock */
        CLR1      0xFFFA4.0          ;; 2 cycles
//   73   
//   74 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock1
        ; ------------------------------------- Block: 12 cycles
        ; ------------------------------------- Total: 59 cycles
        REQUIRE __A_HOCODIV
        REQUIRE __A_CKC

        SECTION `.iar_vfe_header`:DATA:NOALLOC:NOROOT(1)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
        DC32 0

        END
// 
//   3 bytes in section .bss
//   2 bytes in section .bss.noinit  (abs)
// 106 bytes in section .text
// 
// 106 bytes of CODE memory
//   3 bytes of DATA memory (+ 2 bytes shared)
//
//Errors: none
//Warnings: none
