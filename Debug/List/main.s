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
//        BootCode\source_code\source_files\main.c
//    Command line       =
//        -f C:\Users\17054\AppData\Local\Temp\EWD835.tmp ("D:\Software
//        code\Projects Software\Projects new\0. Non AMI\SUGAM Based\0. GDEV72
//        - BootCode-20210104T103109Z-001\0. GDEV72 -
//        BootCode\source_code\source_files\main.c" --core s3 --code_model near
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
//        BootCode\Debug\List\main.s
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

        EXTERN _R_LVD_Create
        EXTERN _R_WDT_Create
        EXTERN _R_WDT_Restart
        EXTERN _Self_Programming_main
        EXTERN _backlight_operation
        EXTERN _clear_ram
        EXTERN _clock_init
        EXTERN _delay_ms
        EXTERN _flag1
        EXTERN _lcd_clear_var
        EXTERN _lcd_init
        EXTERN _lcd_map
        EXTERN _lcd_start
        EXTERN _lcd_write
        EXTERN _lvd_start
        EXTERN _pcb_init_universal
        EXTERN _read_operating_mode

        PUBLIC __A_RPECTL
        PUBLIC _main
        
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
        
// D:\Software code\Projects Software\Projects new\0. Non AMI\SUGAM Based\0. GDEV72 - BootCode-20210104T103109Z-001\0. GDEV72 - BootCode\source_code\source_files\main.c
//    1 /***********************************************************************************************************************
//    2 * File Name       : main.c  
//    3 * Tool-Chain      : IAR Systems
//    4 * Creation Date   : 03-01-2020
//    5 * Company         : Genus Power Infrastructures Limited, Jaipur
//    6 * Author          : dheeraj.singhal
//    7 ***********************************************************************************************************************/
//    8 /************************************ Includes **************************************/
//    9 #include "main.h"

        ASEGN `.bss.noinit`:DATA:NOROOT,0f00f5H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_RPECTL
// __no_init union <unnamed>#282 volatile _A_RPECTL
__A_RPECTL:
        DS 1
//   10 #include "system.h"
//   11 #include "clock.h"
//   12 #include "variables.h"
//   13 
//   14 /************************************ Local Variables *****************************************/
//   15 /************************************ Extern Variables *****************************************/
//   16 /************************************ Local Functions *******************************/
//   17 /************************************ Extern Functions ******************************/
//   18 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock0 Using cfiCommon0
          CFI Function _main
        CODE
//   19 void main()
//   20 {
_main:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 2
        SUBW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+6
//   21   /* Reset through ram parity error are disabled */
//   22   __disable_interrupt();
        DI                           ;; 4 cycles
//   23   RPERDIS = 1;
        SET1      0xF00F5.7          ;; 2 cycles
//   24   pcb_init_universal();
          CFI FunCall _pcb_init_universal
        CALL      _pcb_init_universal  ;; 3 cycles
//   25   clock_init();
          CFI FunCall _clock_init
        CALL      _clock_init        ;; 3 cycles
//   26   wdt_init();
          CFI FunCall _R_WDT_Create
        CALL      _R_WDT_Create      ;; 3 cycles
//   27   wdt_restart();
          CFI FunCall _R_WDT_Restart
        CALL      _R_WDT_Restart     ;; 3 cycles
//   28   R_LVD_Create();
          CFI FunCall _R_LVD_Create
        CALL      _R_LVD_Create      ;; 3 cycles
//   29   lvd_start();
          CFI FunCall _lvd_start
        CALL      _lvd_start         ;; 3 cycles
//   30   
//   31   /* waiting for 0.5 Sec */
//   32   for(us16 index = 0; index <5; index++)
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [SP], AX           ;; 1 cycle
        ; ------------------------------------- Block: 27 cycles
??main_0:
        MOVW      AX, [SP]           ;; 1 cycle
        CMPW      AX, #0x5           ;; 1 cycle
        BNC       ??main_1           ;; 4 cycles
          CFI FunCall _R_WDT_Restart
        ; ------------------------------------- Block: 6 cycles
//   33   {
//   34     wdt_restart();
        CALL      _R_WDT_Restart     ;; 3 cycles
//   35     delay_ms(100);
        MOVW      AX, #0x64          ;; 1 cycle
          CFI FunCall _delay_ms
        CALL      _delay_ms          ;; 3 cycles
//   36   }
        MOVW      AX, [SP]           ;; 1 cycle
        INCW      AX                 ;; 1 cycle
        MOVW      [SP], AX           ;; 1 cycle
        BR        S:??main_0         ;; 3 cycles
          CFI FunCall _clear_ram
        ; ------------------------------------- Block: 13 cycles
//   37   clear_ram();
??main_1:
        CALL      _clear_ram         ;; 3 cycles
//   38 
//   39   lcd_init();
          CFI FunCall _lcd_init
        CALL      _lcd_init          ;; 3 cycles
//   40   lcd_start();
          CFI FunCall _lcd_start
        CALL      _lcd_start         ;; 3 cycles
//   41   
//   42   lcd_clear_var();
          CFI FunCall _lcd_clear_var
        CALL      _lcd_clear_var     ;; 3 cycles
//   43   lcd_map[3] |= 0x0009;   // Capital B
        MOVW      AX, N:_lcd_map+6   ;; 1 cycle
        OR        A, #0x0            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        OR        A, #0x9            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      N:_lcd_map+6, AX   ;; 1 cycle
//   44   lcd_map[4] |= 0x0207;
        MOVW      AX, N:_lcd_map+8   ;; 1 cycle
        OR        A, #0x2            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        OR        A, #0x7            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      N:_lcd_map+8, AX   ;; 1 cycle
//   45   lcd_map[5] |= LCD_7B;
        MOVW      AX, N:_lcd_map+10  ;; 1 cycle
        OR        A, #0xE0           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        OR        A, #0x30           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      N:_lcd_map+10, AX  ;; 1 cycle
//   46   lcd_map[6] |= LCD_7O;
        MOVW      AX, N:_lcd_map+12  ;; 1 cycle
        OR        A, #0xA0           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        OR        A, #0xF0           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      N:_lcd_map+12, AX  ;; 1 cycle
//   47   lcd_map[7] |= LCD_7O;
        MOVW      AX, N:_lcd_map+14  ;; 1 cycle
        OR        A, #0xA0           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        OR        A, #0xF0           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      N:_lcd_map+14, AX  ;; 1 cycle
//   48   lcd_map[8] |= LCD_7T;
        MOVW      AX, N:_lcd_map+16  ;; 1 cycle
        OR        A, #0xE0           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        OR        A, #0x10           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      N:_lcd_map+16, AX  ;; 1 cycle
//   49 //  lcd_map[1] |= 0x7A; // reverse_8bits(LCD_7b);
//   50   lcd_write();
          CFI FunCall _lcd_write
        CALL      _lcd_write         ;; 3 cycles
//   51   
//   52   battery_mode_f = read_operating_mode();
          CFI FunCall _read_operating_mode
        CALL      _read_operating_mode  ;; 3 cycles
        MOV       [SP], A            ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        MOVW      HL, #LWRD(_flag1)  ;; 1 cycle
        MOV1      [HL].0, CY         ;; 2 cycles
//   53   
//   54   backlight_operation();
          CFI FunCall _backlight_operation
        CALL      _backlight_operation  ;; 3 cycles
//   55   
//   56   wdt_restart();
          CFI FunCall _R_WDT_Restart
        CALL      _R_WDT_Restart     ;; 3 cycles
//   57   Self_Programming_main();
          CFI FunCall _Self_Programming_main
        CALL      _Self_Programming_main  ;; 3 cycles
        ; ------------------------------------- Block: 70 cycles
//   58   
//   59   while(1)
//   60   {
//   61     NOP();
??main_2:
        NOP                          ;; 1 cycle
        BR        S:??main_2         ;; 3 cycles
          CFI EndBlock cfiBlock0
        ; ------------------------------------- Block: 4 cycles
        ; ------------------------------------- Total: 120 cycles
        REQUIRE __A_RPECTL
//   62   }
//   63 }

        SECTION `.iar_vfe_header`:DATA:NOALLOC:NOROOT(1)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
        DC32 0

        END
// 
//   1 byte  in section .bss.noinit  (abs)
// 169 bytes in section .text
// 
// 169 bytes of CODE memory
//   0 bytes of DATA memory (+ 1 byte shared)
//
//Errors: none
//Warnings: none
