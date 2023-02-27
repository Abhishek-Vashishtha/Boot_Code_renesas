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
//        BootCode\source_code\source_files\lcd.c
//    Command line       =
//        -f C:\Users\17054\AppData\Local\Temp\EWD823.tmp ("D:\Software
//        code\Projects Software\Projects new\0. Non AMI\SUGAM Based\0. GDEV72
//        - BootCode-20210104T103109Z-001\0. GDEV72 -
//        BootCode\source_code\source_files\lcd.c" --core s3 --code_model near
//        --calling_convention v2 --near_const_location ram -o "D:\Software
//        code\Projects Software\Projects new\0. Non AMI\SUGAM Based\0. GDEV72
//        - BootCode-20210104T103109Z-001\0. GDEV72 - BootCode\Debug\Obj"
//        --dlib_config "C:\Program Files (x86)\IAR Systems\Embedded Workbench
//        8.5\rl78\lib\DLib_Config_Normal.h" --double=32 -e -On --no_cse
//        --no_unroll --no_inline --no_code_motion --no_tbaa --no_cross_call
//        --no_scheduling --no_clustering --debug -lA "D:\Software
//        code\Projects Software\Projects new\0. Non AMI\SUGAM Based\0. GDEV72
//        - BootCode-20210104T103109Z-001\0. GDEV72 - BootCode\Debug\List" -I
//        "D:\Software code\Projects Software\Projects new\0. Non AMI\SUGAM
//        Based\0. GDEV72 - BootCode-20210104T103109Z-001\0. GDEV72 -
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
//        BootCode\Debug\List\lcd.s
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

        EXTERN _R_LCD_Create
        EXTERN _R_LCD_Start
        EXTERN _R_LCD_Stop
        EXTERN _R_LCD_Voltage_Off
        EXTERN _R_LCD_Voltage_On
        EXTERN _reverse_8bits

        PUBLIC __A_SEG4
        PUBLIC _lcd_clear_var
        PUBLIC _lcd_disp_off
        PUBLIC _lcd_init
        PUBLIC _lcd_map
        PUBLIC _lcd_start
        PUBLIC _lcd_stop
        PUBLIC _lcd_write
        
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
        
// D:\Software code\Projects Software\Projects new\0. Non AMI\SUGAM Based\0. GDEV72 - BootCode-20210104T103109Z-001\0. GDEV72 - BootCode\source_code\source_files\lcd.c
//    1 /***********************************************************************************************************************
//    2 * File Name       : lcd.c
//    3 * Current Version : rev_01  
//    4 * Tool-Chain      : IAR Systems
//    5 * Description     : This file comprises all the routines and variables required for the operation of LCD Module
//    6 * Creation Date   : 26-12-2019
//    7 * Company         : Genus Power Infrastructures Limited, Jaipur
//    8 * Author          : dheeraj.singhal
//    9 * Version History : rev_01 - new source file created with routine to operate LCD module 
//   10 ***********************************************************************************************************************/
//   11 
//   12 /************************************ Includes **************************************/
//   13 #include "lcd.h"

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0404H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_SEG4
// __no_init union <unnamed>#610 volatile __no_bit_access _A_SEG4
__A_SEG4:
        DS 1
//   14 
//   15 /************************************ Local Variables *****************************************/
//   16 //const us8 lcd_alphabeat[58] = {LCD_7A,LCD_7B,LCD_7C,LCD_7D,LCD_7E,LCD_7F,LCD_7G,LCD_7H,LCD_7I,LCD_7J,LCD_7K,LCD_7L,LCD_7M,LCD_7N,LCD_7O,LCD_7P,LCD_7Q,LCD_7R,LCD_7S,LCD_7T,LCD_7U,LCD_7V,LCD_7W,LCD_7X,LCD_7Y,LCD_7Z,LCD_7dash,LCD_7dash,LCD_7dash,LCD_7dash,LCD_7dash,LCD_7dash,LCD_7a,LCD_7b,LCD_7c,LCD_7d,LCD_7e,LCD_7f,LCD_7g,LCD_7h,LCD_7i,LCD_7j,LCD_7k,LCD_7l,LCD_7m,LCD_7n,LCD_7o,LCD_7p,LCD_7q,LCD_7r,LCD_7s,LCD_7t,LCD_7u,LCD_7v,LCD_7w,LCD_7x,LCD_7y,LCD_7z};
//   17 //const us8 lcd_7digit[16] = {LCD_70,LCD_71,LCD_72,LCD_73,LCD_74,LCD_75,LCD_76,LCD_77,LCD_78,LCD_79,LCD_7A,LCD_7B,LCD_7C,LCD_7D,LCD_7E,LCD_7F};
//   18 
//   19 /************************************ Global Variables *****************************************/
//   20 

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   21 us16 lcd_map[13];
_lcd_map:
        DS 26
//   22 
//   23 /************************************ Local Functions *******************************/
//   24 
//   25 /************************************ Extern Functions ******************************/
//   26 void lcd_clear_var();
//   27 void lcd_disp_off();
//   28 void lcd_init();
//   29 void lcd_stop();
//   30 void lcd_start();
//   31 void lcd_write();
//   32 
//   33 
//   34 /***********************************************************************************************************************
//   35 * Function Name: lcd_init
//   36 * Description  : Initialises driver and displays welcome text
//   37 * Arguments    : nothing
//   38 * Return Value : nothing
//   39 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock0 Using cfiCommon0
          CFI Function _lcd_init
          CFI FunCall _R_LCD_Create
        CODE
//   40 void lcd_init()
//   41 {
_lcd_init:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//   42   R_LCD_Create();
        CALL      _R_LCD_Create      ;; 3 cycles
//   43   R_LCD_Voltage_On();
          CFI FunCall _R_LCD_Voltage_On
        CALL      _R_LCD_Voltage_On  ;; 3 cycles
//   44 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock0
        ; ------------------------------------- Block: 12 cycles
        ; ------------------------------------- Total: 12 cycles

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock1 Using cfiCommon0
          CFI Function _lcd_start
          CFI FunCall _R_LCD_Start
        CODE
//   45 void lcd_start()
//   46 {
_lcd_start:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//   47   R_LCD_Start();
        CALL      _R_LCD_Start       ;; 3 cycles
//   48   lcd_disp_off();
          CFI FunCall _lcd_disp_off
        CALL      _lcd_disp_off      ;; 3 cycles
//   49 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock1
        ; ------------------------------------- Block: 12 cycles
        ; ------------------------------------- Total: 12 cycles

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock2 Using cfiCommon0
          CFI Function _lcd_stop
          CFI FunCall _R_LCD_Voltage_Off
        CODE
//   50 void lcd_stop()
//   51 {
_lcd_stop:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//   52   R_LCD_Voltage_Off();
        CALL      _R_LCD_Voltage_Off  ;; 3 cycles
//   53   R_LCD_Stop();
          CFI FunCall _R_LCD_Stop
        CALL      _R_LCD_Stop        ;; 3 cycles
//   54 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock2
        ; ------------------------------------- Block: 12 cycles
        ; ------------------------------------- Total: 12 cycles

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock3 Using cfiCommon0
          CFI Function _lcd_disp_off
          CFI NoCalls
        CODE
//   55 void lcd_disp_off()
//   56 {
_lcd_disp_off:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//   57     for(us8 index = 0; index <=12; index++)
        MOV       E, #0x0            ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??lcd_disp_off_0:
        MOV       A, E               ;; 1 cycle
        CMP       A, #0xD            ;; 1 cycle
        BNC       ??lcd_clear_var_0  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//   58     {
//   59 	U8LCDMEM[2 * index] = 0;
        MOVW      AX, DE             ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      BC, AX             ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOV       (0x404)[BC], A     ;; 1 cycle
//   60 	U8LCDMEM[2 * index + 1] = 0;
        MOVW      AX, DE             ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #0x404         ;; 1 cycle
        INCW      AX                 ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
//   61     }
        INC       E                  ;; 1 cycle
        BR        S:??lcd_disp_off_0  ;; 3 cycles
        ; ------------------------------------- Block: 22 cycles
//   62 }
??lcd_clear_var_0:
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock3
        ; ------------------------------------- Block: 6 cycles
        ; ------------------------------------- Total: 35 cycles
        REQUIRE __A_SEG4
//   63 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock4 Using cfiCommon0
          CFI Function _lcd_write
        CODE
//   64 void lcd_write()
//   65 {
_lcd_write:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 2
        SUBW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+6
//   66     for(us8 index = 0; index <=12; index++)
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        ; ------------------------------------- Block: 3 cycles
??lcd_write_0:
        MOV       A, [SP]            ;; 1 cycle
        CMP       A, #0xD            ;; 1 cycle
        BNC       ??lcd_clear_var_1  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//   67     {
//   68 	U8LCDMEM[2 * index] = reverse_8bits(lowByte(lcd_map[12 - index]));
        MOV       A, [SP]            ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        XOR       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        XOR       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        INCW      AX                 ;; 1 cycle
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(_lcd_map+24)  ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [HL]           ;; 1 cycle
        MOV       A, X               ;; 1 cycle
          CFI FunCall _reverse_8bits
        CALL      _reverse_8bits     ;; 3 cycles
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      BC, AX             ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       (0x404)[BC], A     ;; 1 cycle
        XCH       A, D               ;; 1 cycle
//   69 	U8LCDMEM[2 * index + 1] = reverse_8bits(highByte(lcd_map[12 - index]));
        MOV       A, [SP]            ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        XOR       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        XOR       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        INCW      AX                 ;; 1 cycle
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(_lcd_map+24)  ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [HL]           ;; 1 cycle
        CLRB      X                  ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       A, X               ;; 1 cycle
          CFI FunCall _reverse_8bits
        CALL      _reverse_8bits     ;; 3 cycles
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #0x404         ;; 1 cycle
        INCW      AX                 ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, D               ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
//   70     }
        MOV       A, [SP]            ;; 1 cycle
        INC       A                  ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        BR        S:??lcd_write_0    ;; 3 cycles
        ; ------------------------------------- Block: 67 cycles
//   71 }
??lcd_clear_var_1:
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock4
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 83 cycles
        REQUIRE __A_SEG4
//   72 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock5 Using cfiCommon0
          CFI Function _lcd_clear_var
          CFI NoCalls
        CODE
//   73 void lcd_clear_var()
//   74 {
_lcd_clear_var:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//   75   us8 index = 0;
        MOV       E, #0x0            ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
//   76   while(index <= 12)
??lcd_clear_var_2:
        MOV       A, E               ;; 1 cycle
        CMP       A, #0xD            ;; 1 cycle
        BNC       ??lcd_clear_var_3  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//   77   {
//   78     lcd_map[index]=0;
        MOVW      AX, DE             ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(_lcd_map)  ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
//   79     index++;
        INC       E                  ;; 1 cycle
        BR        S:??lcd_clear_var_2  ;; 3 cycles
        ; ------------------------------------- Block: 13 cycles
//   80   }
//   81 }
??lcd_clear_var_3:
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock5
        ; ------------------------------------- Block: 6 cycles
        ; ------------------------------------- Total: 26 cycles

        SECTION `.iar_vfe_header`:DATA:NOALLOC:NOROOT(1)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
        DC32 0

        END
// 
//  26 bytes in section .bss
//   1 byte  in section .bss.noinit  (abs)
// 209 bytes in section .text
// 
// 209 bytes of CODE memory
//  26 bytes of DATA memory (+ 1 byte shared)
//
//Errors: none
//Warnings: none
