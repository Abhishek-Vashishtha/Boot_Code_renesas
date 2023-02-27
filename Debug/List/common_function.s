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
//        BootCode\source_code\source_files\common_function.c
//    Command line       =
//        -f C:\Users\17054\AppData\Local\Temp\EWD834.tmp ("D:\Software
//        code\Projects Software\Projects new\0. Non AMI\SUGAM Based\0. GDEV72
//        - BootCode-20210104T103109Z-001\0. GDEV72 -
//        BootCode\source_code\source_files\common_function.c" --core s3
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
//        BootCode\Debug\List\common_function.s
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

        EXTERN _clock_select
        EXTERN ?C_LSH_L01

        PUBLIC _dec2hexAscii
        PUBLIC _delay
        PUBLIC _delay_ms
        PUBLIC _delay_us
        PUBLIC _reverse_8bits
        
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
        
// D:\Software code\Projects Software\Projects new\0. Non AMI\SUGAM Based\0. GDEV72 - BootCode-20210104T103109Z-001\0. GDEV72 - BootCode\source_code\source_files\common_function.c
//    1 /***********************************************************************************************************************
//    2 * File Name       : common_function.c
//    3 * Current Version : rev_01  
//    4 * Tool-Chain      : IAR Systems
//    5 * Description     : Common functions library 
//    6 * Creation Date   : 30-12-2019
//    7 * Company         : Genus Power Infrastructures Limited, Jaipur
//    8 * Author          : dheeraj.singhal
//    9 * Version History : rev_01 : Routines added for general applications
//   10 ***********************************************************************************************************************/
//   11 /************************************ Includes **************************************/
//   12 #include "common_function.h"
//   13 

        SECTION `.data`:DATA:REORDER:NOROOT(1)
//   14 const us8 dec2hexAscii[16] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
_dec2hexAscii:
        DATA8
        DB 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 65, 66, 67, 68, 69, 70
//   15 
//   16 /************************************ Local Functions *******************************/
//   17 void delay(us16 cntr);
//   18 
//   19 /************************************ Extern Functions ******************************/
//   20 
//   21 us8 reverse_8bits(us8 data);
//   22 void delay_ms(us16 count);
//   23 void delay_us(us16 us);
//   24 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock0 Using cfiCommon0
          CFI Function _reverse_8bits
        CODE
//   25 us8 reverse_8bits(us8 data)
//   26 {
_reverse_8bits:
        ; * Stack frame (at entry) *
        ; Param size: 0
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+6
        ; Auto size: 2
//   27     us8 temp=0;
        MOV       C, #0x0            ;; 1 cycle
//   28     for(us8 index=0; index<8; index++)
        MOV       B, #0x0            ;; 1 cycle
        ; ------------------------------------- Block: 3 cycles
??reverse_8bits_0:
        MOV       A, B               ;; 1 cycle
        CMP       A, #0x8            ;; 1 cycle
        BNC       ??delay_us_0       ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//   29     {
//   30 	if(data & 0x01)
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x1           ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??delay_us_1       ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
//   31 	{
//   32 	    temp |= (1<<(7-index)); 
        MOV       A, #0x7            ;; 1 cycle
        SUB       A, B               ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x1            ;; 1 cycle
          CFI FunCall ?C_LSH_L01
        CALL      N:?C_LSH_L01       ;; 3 cycles
        OR        C, A               ;; 1 cycle
        ; ------------------------------------- Block: 8 cycles
//   33 	}
//   34 	data = data>>1;
??delay_us_1:
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHR       A, 0x1             ;; 1 cycle
        MOV       [SP+0x01], A       ;; 1 cycle
//   35     }
        INC       B                  ;; 1 cycle
        BR        S:??reverse_8bits_0  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
//   36     return(temp);
??delay_us_0:
        MOV       A, C               ;; 1 cycle
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock0
        ; ------------------------------------- Block: 8 cycles
        ; ------------------------------------- Total: 40 cycles
//   37 }
//   38 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock1 Using cfiCommon0
          CFI Function _delay_ms
        CODE
//   39 void delay_ms(us16 count)
//   40 {
_delay_ms:
        ; * Stack frame (at entry) *
        ; Param size: 0
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+6
        ; Auto size: 4
        SUBW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+8
//   41 	us16 index = 0;
        MOVW      AX, #0x0           ;; 1 cycle
//   42         if(clock_select == CLOCK_24MHZ)
        CMP       N:_clock_select, #0x1  ;; 1 cycle
        BNZ       ??delay_us_2       ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
//   43         {
//   44             for (index = 0; index < count; index++)
        MOVW      HL, #0x0           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      [SP], AX           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        ; ------------------------------------- Block: 4 cycles
??delay_ms_0:
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        SKC                          ;; 4 cycles
        BR        N:??delay_us_3     ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
//   45             {
//   46                 delay(3416);
        MOVW      AX, #0xD58         ;; 1 cycle
          CFI FunCall _delay
        CALL      _delay             ;; 3 cycles
//   47             }
        MOVW      AX, [SP]           ;; 1 cycle
        INCW      AX                 ;; 1 cycle
        MOVW      [SP], AX           ;; 1 cycle
        BR        S:??delay_ms_0     ;; 3 cycles
        ; ------------------------------------- Block: 10 cycles
//   48         }
//   49         else if(clock_select == CLOCK_12MHZ)
??delay_us_2:
        CMP       N:_clock_select, #0x2  ;; 1 cycle
        BNZ       ??delay_us_4       ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//   50         {
//   51             for (index = 0; index < count; index++)
        MOVW      HL, #0x0           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      [SP], AX           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        ; ------------------------------------- Block: 4 cycles
??delay_ms_1:
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BNC       ??delay_us_3       ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
//   52             {
//   53                 delay(1708);
        MOVW      AX, #0x6AC         ;; 1 cycle
          CFI FunCall _delay
        CALL      _delay             ;; 3 cycles
//   54             }
        MOVW      AX, [SP]           ;; 1 cycle
        INCW      AX                 ;; 1 cycle
        MOVW      [SP], AX           ;; 1 cycle
        BR        S:??delay_ms_1     ;; 3 cycles
        ; ------------------------------------- Block: 10 cycles
//   55         }
//   56         else if(clock_select == CLOCK_6MHZ)
??delay_us_4:
        CMP       N:_clock_select, #0x3  ;; 1 cycle
        BNZ       ??delay_us_5       ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//   57         {
//   58             for (index = 0; index < count; index++)
        MOVW      HL, #0x0           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      [SP], AX           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        ; ------------------------------------- Block: 4 cycles
??delay_ms_2:
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BNC       ??delay_us_3       ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
//   59             {
//   60                 delay(854);
        MOVW      AX, #0x356         ;; 1 cycle
          CFI FunCall _delay
        CALL      _delay             ;; 3 cycles
//   61             }
        MOVW      AX, [SP]           ;; 1 cycle
        INCW      AX                 ;; 1 cycle
        MOVW      [SP], AX           ;; 1 cycle
        BR        S:??delay_ms_2     ;; 3 cycles
        ; ------------------------------------- Block: 10 cycles
//   62         }
//   63         else if(clock_select == CLOCK_1_5MHZ)
??delay_us_5:
        CMP       N:_clock_select, #0x4  ;; 1 cycle
        BNZ       ??delay_us_6       ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//   64         {
//   65             for (index = 0; index < count; index++)
        MOVW      HL, #0x0           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      [SP], AX           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        ; ------------------------------------- Block: 4 cycles
??delay_ms_3:
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BNC       ??delay_us_3       ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
//   66             {
//   67                 delay(214);
        MOVW      AX, #0xD6          ;; 1 cycle
          CFI FunCall _delay
        CALL      _delay             ;; 3 cycles
//   68             }
        MOVW      AX, [SP]           ;; 1 cycle
        INCW      AX                 ;; 1 cycle
        MOVW      [SP], AX           ;; 1 cycle
        BR        S:??delay_ms_3     ;; 3 cycles
        ; ------------------------------------- Block: 10 cycles
//   69         }
//   70         else
//   71         {
//   72             for (index = 0; index < count; index++)
??delay_us_6:
        MOVW      HL, #0x0           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      [SP], AX           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        ; ------------------------------------- Block: 4 cycles
??delay_ms_4:
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BNC       ??delay_us_3       ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
//   73             {
//   74                 delay(3416);
        MOVW      AX, #0xD58         ;; 1 cycle
          CFI FunCall _delay
        CALL      _delay             ;; 3 cycles
//   75             }
        MOVW      AX, [SP]           ;; 1 cycle
        INCW      AX                 ;; 1 cycle
        MOVW      [SP], AX           ;; 1 cycle
        BR        S:??delay_ms_4     ;; 3 cycles
        ; ------------------------------------- Block: 10 cycles
//   76         }
//   77 }
??delay_us_3:
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock1
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 140 cycles

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock2 Using cfiCommon0
          CFI Function _delay
          CFI NoCalls
        CODE
//   78 inline void delay(us16 cntr)
//   79 {
_delay:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//   80     while(cntr != 0)
??delay_0:
        CMPW      AX, #0x0           ;; 1 cycle
        BZ        ??delay_us_7       ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//   81     {
//   82         cntr--;
        DECW      AX                 ;; 1 cycle
        BR        S:??delay_0        ;; 3 cycles
        ; ------------------------------------- Block: 4 cycles
//   83     }
//   84 }
??delay_us_7:
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock2
        ; ------------------------------------- Block: 6 cycles
        ; ------------------------------------- Total: 15 cycles

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock3 Using cfiCommon0
          CFI Function _delay_us
          CFI NoCalls
        CODE
//   85 void delay_us(us16 us)
//   86 {
_delay_us:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//   87     if(clock_select == CLOCK_24MHZ)
        CMP       N:_clock_select, #0x1  ;; 1 cycle
        BNZ       ??delay_us_8       ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//   88     {
//   89         /* Compensate -1us for:
//   90         *  - Copy param to AX (input params)
//   91         *  - CALL _MCU_Delay 
//   92         *  - Begin & End function
//   93         * See "ASM Code Summary Table" for more detail
//   94         */
//   95         if (us < 2)
        CMPW      AX, #0x2           ;; 1 cycle
        SKNC                         ;; 4 cycles
        BR        N:??delay_us_9     ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//   96         {
//   97             return;
//   98         }
//   99         us--;
        DECW      AX                 ;; 1 cycle
//  100         us--;
        DECW      AX                 ;; 1 cycle
//  101         
//  102         /* NOP compensated for t2 */
//  103         NOP();
        NOP                          ;; 1 cycle
//  104         NOP();
        NOP                          ;; 1 cycle
//  105         NOP();
        NOP                          ;; 1 cycle
//  106         NOP();
        NOP                          ;; 1 cycle
//  107         NOP();
        NOP                          ;; 1 cycle
//  108         NOP();
        NOP                          ;; 1 cycle
//  109         NOP();
        NOP                          ;; 1 cycle
//  110         NOP();
        NOP                          ;; 1 cycle
//  111         NOP();
        NOP                          ;; 1 cycle
//  112         NOP();
        NOP                          ;; 1 cycle
//  113         NOP();
        NOP                          ;; 1 cycle
//  114         NOP();
        NOP                          ;; 1 cycle
//  115         NOP();      
        NOP                          ;; 1 cycle
//  116         NOP();
        NOP                          ;; 1 cycle
//  117         NOP();
        NOP                          ;; 1 cycle
//  118         NOP();
        NOP                          ;; 1 cycle
        ; ------------------------------------- Block: 18 cycles
//  119         
//  120         /* Implementation */
//  121         while (us)  /* Each loop must elapse 1us */
??delay_us_10:
        CMPW      AX, #0x0           ;; 1 cycle
        BZ        ??delay_us_9       ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  122         {
//  123             /* Put ASM code (NOP instruction here) */
//  124             /* 12 NOP Compensated t3 */
//  125             NOP();
        NOP                          ;; 1 cycle
//  126             NOP();
        NOP                          ;; 1 cycle
//  127             NOP();
        NOP                          ;; 1 cycle
//  128             NOP();
        NOP                          ;; 1 cycle
//  129             NOP();
        NOP                          ;; 1 cycle
//  130             NOP();
        NOP                          ;; 1 cycle
//  131             NOP();
        NOP                          ;; 1 cycle
//  132             NOP();
        NOP                          ;; 1 cycle
//  133             NOP();
        NOP                          ;; 1 cycle
//  134             NOP();
        NOP                          ;; 1 cycle
//  135             NOP();
        NOP                          ;; 1 cycle
//  136             NOP();
        NOP                          ;; 1 cycle
//  137             
//  138             us--;   /* count down number of us */
        DECW      AX                 ;; 1 cycle
        BR        S:??delay_us_10    ;; 3 cycles
        ; ------------------------------------- Block: 16 cycles
//  139         }
//  140         
//  141         /*
//  142         *-----------------------------------------------------------------------------------------------
//  143         *| ASM Code Summary Table                                                                      |
//  144         *| . Setting of RL78I1B                                                                        |
//  145         *|   fCPU = 20MHz                                                                              |
//  146         *|   -->  1Clock = 50ns , 1us = 20 clock                                                       |
//  147         *| . Requirement                                                                               |
//  148         *|   > t1 + t2 + t4 = 2us (40 clocks)                                                          |
//  149         *|   > t3           = 1us                                                                      |
//  150         *| . Actual                                                                                    |
//  151         *|   > t1 + t4 = 19 clock                                                                      |
//  152         *|   > t2 = 21 clock compensated                                                                |
//  153         *|   > t1 + t2 + t4 = 20 * 50 = 2us (Passed)                                                   |
//  154         *|   > t3           = 8clock + 12clock compensated = 20 * 50 = 1us (Passed)                    |
//  155         *-----------------------------------------------------------------------------------------------
//  156         *| ASM Code                                        | Description          | Summary (by time)  |
//  157         *-----------------------------------------------------------------------------------------------
//  158         * _MCU_Delay:                                      |                      |                    |
//  159         *                                                  |                      |                    |
//  160         *                                                  |                      |                    |
//  161         * # <<Begin Function>>                             |                      | <<Begin Function>> |
//  162         * # MOVW Input Parameter to AX                     | 1clock            t1 | t1: 6 clocks       |
//  163         * # CALL _MCU_Delay                                | 3clocks           t1 |                    |
//  164         * #                                                |                      |                    |
//  165         * #   Backup Register                              |                      |                    |
//  166         * #   Variable Stack Allocation                    |                      |                    |
//  167         *                                                  |                      |                    |
//  168         *    c7          PUSH            HL                | 1clock            t1 |                    |
//  169         *    16          MOVW            HL,AX             | 1clock            t1 |                    |
//  170         *                                                  |                      |                    |
//  171         * # <<Function Body>>                              |                      | <<Function Body>>  |
//  172         * # <<Compensate>>                                 |                      | <<Compensate>>     |
//  173         *    b7          DECW            HL                | 1clock            t2 | t2: 21 clock       |
//  174         *    b7          DECW            HL                | 1clock            t2 |                    |
//  175         *    f6          CLRW            AX                | 1clock            t2 |                    |
//  176         *    47          CMPW            AX,HL             | 1clock            t2 |                    |
//  177         *    61f8        SKNZ                              | 1clock            t2 |                    |
//  178         *    00          NOP x 16                          | 16clock           t2 |                    |        
//  179         *                                                  |                      |                    |
//  180         * # while (us)                                     |                      | << 1 Whill Loop>>  |
//  181         * # {                                              |                      | t3: 20 clocks      |
//  182         *    f6          CLRW            AX                | 1clock         t3 t4 |                    |
//  183         *    47          CMPW            AX,HL             | 1clock         t3 t4 |                    |
//  184         *    xxxx        BZ              $[End Function]   | 2clocks (NM)   t3    |                    |
//  185         *                                                  | 4clocks ( M)      t4 |                    |
//  186         * # NOP(); 12 NOP                                  |                      |                    |
//  187         *    00          NOP x 12                          | 12clock        t3    |                    |
//  188         *                                                  |                      |                    |
//  189         * #us--;                                           |                      |                    |
//  190         *    b7          DECW            HL                | 1clock         t3    |                    |
//  191         *                                                  |                      |                    |
//  192         * # }                                              |                      |                    |
//  193         *    xxxx        BR              $[# while (us)]   | 3clocks        t3    |                    |
//  194         *                                                  |                      |                    |
//  195         * # <<End Function>>                               |                      | <<End Function>>   |
//  196         *    c6          POP             HL                | 1clock            t4 | t4: 13 clocks      |
//  197         *    d7          RET                               | 6clocks           t4 |                    |
//  198         *                                                  |                      |                    |
//  199         *-----------------------------------------------------------------------------------------------
//  200         */
//  201         
//  202     }
//  203     else if(clock_select == CLOCK_12MHZ)
??delay_us_8:
        CMP       N:_clock_select, #0x2  ;; 1 cycle
        BNZ       ??delay_us_11      ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  204     {
//  205         
//  206         /* Compensate -1us for:
//  207         *  - Copy param to AX (input params)
//  208         *  - CALL _MCU_Delay 
//  209         *  - Begin & End function
//  210         * See "ASM Code Summary Table" for more detail
//  211         */
//  212         if (us < 2)
        CMPW      AX, #0x2           ;; 1 cycle
        BC        ??delay_us_9       ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  213         {
//  214             return;
//  215         }
//  216         us--;
        DECW      AX                 ;; 1 cycle
//  217         us--;
        DECW      AX                 ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
//  218         
//  219         /* No NOP compensated for t2 */
//  220         
//  221         /* Implementation */
//  222         while (us)  /* Each loop must elapse 1us */
??delay_us_12:
        CMPW      AX, #0x0           ;; 1 cycle
        BZ        ??delay_us_9       ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  223         {
//  224             /* Put ASM code (NOP instruction here) */
//  225             /* 16 NOP Compensated t3 */
//  226             NOP();  /*  01  */
        NOP                          ;; 1 cycle
//  227             NOP();  /*  02  */
        NOP                          ;; 1 cycle
//  228             NOP();  /*  03  */
        NOP                          ;; 1 cycle
//  229             NOP();  /*  04  */
        NOP                          ;; 1 cycle
//  230             
//  231             us--;   /* count down number of us */
        DECW      AX                 ;; 1 cycle
        BR        S:??delay_us_12    ;; 3 cycles
        ; ------------------------------------- Block: 8 cycles
//  232         }
//  233         
//  234         /*
//  235         *-----------------------------------------------------------------------------------------------
//  236         *| ASM Code Summary Table                                                                      |
//  237         *| . Setting of RL78I1B                                                                        |
//  238         *|   fCPU = 12MHz                                                                              |
//  239         *|   -->  1Clock = (1/12)us ~ 0.08333us --> 1us = 12clock                                      |
//  240         *| . Requirement                                                                               |
//  241         *|   > t1 + t2 + t4 = 2us (24 clock)                                                           |
//  242         *|   > t3           = 1us (12 clock)                                                           |
//  243         *| . Actual                                                                                    |
//  244         *|   > t1 + t4      = 19 clock                                                                 |
//  245         *|   > t2           = 5  clock                                                                 |
//  246         *|   > t1 + t2 + t4 = 24 clock = 2us                                                           |
//  247         *|   > t3           = 8  clock + 4clock compensated = 12clock = 1us (Passed)                   |
//  248         *-----------------------------------------------------------------------------------------------
//  249         *| ASM Code                                        | Description          | Summary (by time)  |
//  250         *-----------------------------------------------------------------------------------------------
//  251         * _MCU_Delay:                                      |                      |                    |
//  252         *                                                  |                      |                    |
//  253         *                                                  |                      |                    |
//  254         * # <<Begin Function>>                             |                      | <<Begin Function>> |
//  255         * # MOVW Input Parameter to AX                     | 1clock            t1 | t1: 6 clocks       |
//  256         * # CALL _MCU_Delay                                | 3clocks           t1 |                    |
//  257         * #                                                |                      |                    |
//  258         * #   Backup Register                              |                      |                    |
//  259         * #   Variable Stack Allocation                    |                      |                    |
//  260         *                                                  |                      |                    |
//  261         *    c7          PUSH            HL                | 1clock            t1 |                    |
//  262         *    16          MOVW            HL,AX             | 1clock            t1 |                    |
//  263         *                                                  |                      |                    |
//  264         * # <<Function Body>>                              |                      | <<Function Body>>  |
//  265         * # <<Compensate>>                                 |                      | <<Compensate>>     |
//  266         *    b7          DECW            HL                | 1clock            t2 | t2: 5 clock        |
//  267         *    b7          DECW            HL                | 1clock            t2 |                    |
//  268         *    f6          CLRW            AX                | 1clock            t2 |                    |
//  269         *    47          CMPW            AX,HL             | 1clock            t2 |                    |
//  270         *    61f8        SKNZ                              | 1clock            t2 |                    |
//  271         *                                                  |                      |                    |
//  272         * # while (us)                                     |                      | << 1 Whill Loop>>  |
//  273         * # {                                              |                      | t3: 12 clocks      |
//  274         *    f6          CLRW            AX                | 1clock         t3 t4 |                    |
//  275         *    47          CMPW            AX,HL             | 1clock         t3 t4 |                    |
//  276         *    xxxx        BZ              $[End Function]   | 2clocks (NM)   t3    |                    |
//  277         *                                                  | 4clocks ( M)      t4 |                    |
//  278         * # NOP(); 4 NOP                                   |                      |                    |
//  279         *    00          NOP x 4                           | 4clock         t3    |                    |
//  280         *                                                  |                      |                    |
//  281         * #us--;                                           |                      |                    |
//  282         *    b7          DECW            HL                | 1clock         t3    |                    |
//  283         *                                                  |                      |                    |
//  284         * # }                                              |                      |                    |
//  285         *    xxxx        BR              $[# while (us)]   | 3clocks        t3    |                    |
//  286         *                                                  |                      |                    |
//  287         * # <<End Function>>                               |                      | <<End Function>>   |
//  288         *    c6          POP             HL                | 1clock            t4 | t4: 13 clocks      |
//  289         *    d7          RET                               | 6clocks           t4 |                    |
//  290         *                                                  |                      |                    |
//  291         *-----------------------------------------------------------------------------------------------
//  292         */
//  293         
//  294     }
//  295     else if(clock_select == CLOCK_6MHZ)
??delay_us_11:
        CMP       N:_clock_select, #0x3  ;; 1 cycle
        BNZ       ??delay_us_13      ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  296     {
//  297         
//  298         /* Compensate -4us for:
//  299         *  - Copy param to AX (input params)
//  300         *  - CALL _MCU_Delay 
//  301         *  - Begin & End function
//  302         * See "ASM Code Summary Table" for more detail
//  303         */
//  304         if (us <= 4)
        CMPW      AX, #0x5           ;; 1 cycle
        BNC       ??delay_us_14      ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  305         {
//  306             us = 1;
        MOVW      HL, #0x1           ;; 1 cycle
        BR        S:??delay_us_15    ;; 3 cycles
        ; ------------------------------------- Block: 4 cycles
//  307         }
//  308         else
//  309         {
//  310             us = us / 2;
??delay_us_14:
        SHRW      AX, 0x1            ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
//  311         }
//  312         us--;
??delay_us_15:
        MOVW      AX, HL             ;; 1 cycle
        DECW      AX                 ;; 1 cycle
//  313         
//  314         /* 1 NOP Compensated t2 */
//  315         NOP();  
        NOP                          ;; 1 cycle
        ; ------------------------------------- Block: 3 cycles
//  316         
//  317         /* Implementation */
//  318         while (us)  /* Each loop must elapse 2us */
??delay_us_16:
        CMPW      AX, #0x0           ;; 1 cycle
        BZ        ??delay_us_9       ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  319         {
//  320             /* Put ASM code (NOP instruction here) */
//  321             /* 4 NOP Compensated t3 */
//  322             NOP();  /*  01  */
        NOP                          ;; 1 cycle
//  323             NOP();  /*  02  */
        NOP                          ;; 1 cycle
//  324             NOP();  /*  03  */
        NOP                          ;; 1 cycle
//  325             NOP();  /*  04  */
        NOP                          ;; 1 cycle
//  326             
//  327             us--;   /* count down number of us */
        DECW      AX                 ;; 1 cycle
        BR        S:??delay_us_16    ;; 3 cycles
        ; ------------------------------------- Block: 8 cycles
//  328         }
//  329         
//  330         /*  
//  331         *-----------------------------------------------------------------------------------------------   
//  332         *| ASM Code Summary Table                                                                      |   
//  333         *| . Setting of RL78I1B                                                                        |   
//  334         *|   fCPU = 6MHz                                                                               |   
//  335         *|   -->  1Clock = (1/6)us ~ 0.1666667us --> 1us = 6clock                                      |   
//  336         *| . Requirement                                                                               |   
//  337         *|   > t1 + t2 + t4 = 4us (24 clocks)                                                          |   
//  338         *|   > t3           = 2us (12 clocks)                                                          |   
//  339         *| . Actual                                                                                    |   
//  340         *|   > t1 + t4      = 19clock                                                                  |   
//  341         *|   > t2 = 24-19   = 5clock compensated                                                       |   
//  342         *|   > t3           = 8clock + 4clock compensated = 12 / 6 = 2us (Passed)                     |    
//  343         *-----------------------------------------------------------------------------------------------   
//  344         *| ASM Code                                        | Description          | Summary (by time)  |   
//  345         *-----------------------------------------------------------------------------------------------   
//  346         * _MCU_Delay:                                      |                      |                    |   
//  347         *                                                  |                      |                    |   
//  348         *                                                  |                      |                    |   
//  349         * # <<Begin Function>>                             |                      | <<Begin Function>> |   
//  350         * # MOVW Input Parameter to AX                     | 1clock            t1 | t1: 6 clocks       |   
//  351         * # CALL _MCU_Delay                                | 3clocks           t1 |                    |   
//  352         * #                                                |                      |                    |   
//  353         * #   Backup Register                              |                      |                    |   
//  354         * #   Variable Stack Allocation                    |                      |                    |   
//  355         *                                                  |                      |                    |   
//  356         *    c7          PUSH            HL                | 1clock            t1 |                    |   
//  357         *    16          MOVW            HL,AX             | 1clock            t1 |                    |   
//  358         *                                                  |                      |                    |   
//  359         * # <<Function Body>>                              |                      | <<Function Body>>  |   
//  360         * # <<Compensate>>                                 |                      | <<Compensate>>     |   
//  361         *    17          MOVW            AX,HL             | 1clock            t2 | t2: 5 clocks       |   
//  362         *    312e        SHRW            AX,2H             | 1clock            t2 |                    |   
//  363         *    16          MOVW            HL,AX             | 1clock            t2 |                    |   
//  364         *    b7          DECW            HL                | 1clock            t2 |                    |   
//  365         *    00          NOP                               | 1clock            t2 |                    |   
//  366         *                                                  |                      |                    |   
//  367         * # while (us)                                     |                      | << 1 Whill Loop>>  |   
//  368         * # {                                              |                      | t3: 12 clocks      |   
//  369         *    f6          CLRW            AX                | 1clock         t3 t4 |                    |   
//  370         *    47          CMPW            AX,HL             | 1clock         t3 t4 |                    |   
//  371         *    xxxx        BZ              $[End Function]   | 2clocks (NM)   t3    |                    |   
//  372         *                                                  | 4clocks ( M)      t4 |                    |   
//  373         * # NOP(); 16 NOP                                  |                      |                    |   
//  374         *    00          NOP x 4                           | 4clock         t3    |                    |   
//  375         *                                                  |                      |                    |   
//  376         * #us--;                                           |                      |                    |   
//  377         *    b7          DECW            HL                | 1clock         t3    |                    |   
//  378         *                                                  |                      |                    |   
//  379         * # }                                              |                      |                    |   
//  380         *    xxxx        BR              $[# while (us)]   | 3clocks        t3    |                    |   
//  381         *                                                  |                      |                    |   
//  382         * # <<End Function>>                               |                      | <<End Function>>   |   
//  383         *    c6          POP             HL                | 1clock            t4 | t4: 13 clocks      |   
//  384         *    d7          RET                               | 6clocks           t4 |                    |   
//  385         *                                                  |                      |                    |   
//  386         *-----------------------------------------------------------------------------------------------}  
//  387         */
//  388         
//  389     }
//  390     else if(clock_select == CLOCK_1_5MHZ)
??delay_us_13:
        CMP       N:_clock_select, #0x4  ;; 1 cycle
        BNZ       ??delay_us_9       ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  391     {
//  392         
//  393         /* Compensate -8us for:
//  394         *  - Copy param to AX (input params)
//  395         *  - CALL _MCU_Delay 
//  396         *  - Begin & End function
//  397         * See "ASM Code Summary Table" for more detail
//  398         */
//  399         if (us <= 16)
        CMPW      AX, #0x11          ;; 1 cycle
        BNC       ??delay_us_17      ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  400         {
//  401             us = 1;
        MOVW      HL, #0x1           ;; 1 cycle
        BR        S:??delay_us_18    ;; 3 cycles
        ; ------------------------------------- Block: 4 cycles
//  402         }
//  403         else
//  404         {
//  405             us = us / 8;
??delay_us_17:
        SHRW      AX, 0x3            ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
//  406         }
//  407         us--;
??delay_us_18:
        MOVW      AX, HL             ;; 1 cycle
        DECW      AX                 ;; 1 cycle
//  408         
//  409         /* 1 NOP Compensated t2 */
//  410         NOP();  
        NOP                          ;; 1 cycle
        ; ------------------------------------- Block: 3 cycles
//  411         
//  412         /* Implementation */
//  413         while (us)  /* Each loop must elapse 2us */
??delay_us_19:
        CMPW      AX, #0x0           ;; 1 cycle
        BZ        ??delay_us_9       ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  414         {
//  415             /* Put ASM code (NOP instruction here) */
//  416             /* 4 NOP Compensated t3 */
//  417             NOP();  /*  01  */
        NOP                          ;; 1 cycle
//  418             NOP();  /*  02  */
        NOP                          ;; 1 cycle
//  419             NOP();  /*  03  */
        NOP                          ;; 1 cycle
//  420             NOP();  /*  04  */
        NOP                          ;; 1 cycle
//  421             
//  422             us--;   /* count down number of us */
        DECW      AX                 ;; 1 cycle
        BR        S:??delay_us_19    ;; 3 cycles
        ; ------------------------------------- Block: 8 cycles
//  423         }
//  424         
//  425         /*  
//  426         *-----------------------------------------------------------------------------------------------   
//  427         *| ASM Code Summary Table                                                                      |   
//  428         *| . Setting of RL78I1B                                                                        |   
//  429         *|   fCPU = 6MHz                                                                               |   
//  430         *|   -->  1Clock = (1/3)us ~ 0.3333333333333333us --> 1us = 3clock                             |   
//  431         *| . Requirement                                                                               |   
//  432         *|   > t1 + t2 + t4 = 8us (24 clocks)                                                          |   
//  433         *|   > t3           = 4us (12 clocks)                                                          |   
//  434         *| . Actual                                                                                    |   
//  435         *|   > t1 + t4      = 19clock                                                                  |   
//  436         *|   > t2 = 24-19   = 5clock compensated                                                       |   
//  437         *|   > t3           = 8clock + 4clock compensated = 12 / 3 = 4us (Passed)                     |    
//  438         *-----------------------------------------------------------------------------------------------   
//  439         *| ASM Code                                        | Description          | Summary (by time)  |   
//  440         *-----------------------------------------------------------------------------------------------   
//  441         * _MCU_Delay:                                      |                      |                    |   
//  442         *                                                  |                      |                    |   
//  443         *                                                  |                      |                    |   
//  444         * # <<Begin Function>>                             |                      | <<Begin Function>> |   
//  445         * # MOVW Input Parameter to AX                     | 1clock            t1 | t1: 6 clocks       |   
//  446         * # CALL _MCU_Delay                                | 3clocks           t1 |                    |   
//  447         * #                                                |                      |                    |   
//  448         * #   Backup Register                              |                      |                    |   
//  449         * #   Variable Stack Allocation                    |                      |                    |   
//  450         *                                                  |                      |                    |   
//  451         *    c7          PUSH            HL                | 1clock            t1 |                    |   
//  452         *    16          MOVW            HL,AX             | 1clock            t1 |                    |   
//  453         *                                                  |                      |                    |   
//  454         * # <<Function Body>>                              |                      | <<Function Body>>  |   
//  455         * # <<Compensate>>                                 |                      | <<Compensate>>     |   
//  456         *    17          MOVW            AX,HL             | 1clock            t2 | t2: 5 clocks       |   
//  457         *    312e        SHRW            AX,2H             | 1clock            t2 |                    |   
//  458         *    16          MOVW            HL,AX             | 1clock            t2 |                    |   
//  459         *    b7          DECW            HL                | 1clock            t2 |                    |   
//  460         *    00          NOP                               | 1clock            t2 |                    |   
//  461         *                                                  |                      |                    |   
//  462         * # while (us)                                     |                      | << 1 Whill Loop>>  |   
//  463         * # {                                              |                      | t3: 12 clocks      |   
//  464         *    f6          CLRW            AX                | 1clock         t3 t4 |                    |   
//  465         *    47          CMPW            AX,HL             | 1clock         t3 t4 |                    |   
//  466         *    xxxx        BZ              $[End Function]   | 2clocks (NM)   t3    |                    |   
//  467         *                                                  | 4clocks ( M)      t4 |                    |   
//  468         * # NOP(); 16 NOP                                  |                      |                    |   
//  469         *    00          NOP x 4                           | 4clock         t3    |                    |   
//  470         *                                                  |                      |                    |   
//  471         * #us--;                                           |                      |                    |   
//  472         *    b7          DECW            HL                | 1clock         t3    |                    |   
//  473         *                                                  |                      |                    |   
//  474         * # }                                              |                      |                    |   
//  475         *    xxxx        BR              $[# while (us)]   | 3clocks        t3    |                    |   
//  476         *                                                  |                      |                    |   
//  477         * # <<End Function>>                               |                      | <<End Function>>   |   
//  478         *    c6          POP             HL                | 1clock            t4 | t4: 13 clocks      |   
//  479         *    d7          RET                               | 6clocks           t4 |                    |   
//  480         *                                                  |                      |                    |   
//  481         *-----------------------------------------------------------------------------------------------}  
//  482         */
//  483         
//  484     }
//  485 }
??delay_us_9:
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock3
        ; ------------------------------------- Block: 6 cycles
        ; ------------------------------------- Total: 144 cycles

        SECTION `.iar_vfe_header`:DATA:NOALLOC:NOROOT(1)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
        DC32 0

        END
// 
//  16 bytes in section .data
// 376 bytes in section .text
// 
// 376 bytes of CODE memory
//  16 bytes of DATA memory
//
//Errors: none
//Warnings: none
