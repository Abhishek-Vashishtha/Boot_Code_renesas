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
//        BootCode\source_code\source_files\pcb.c
//    Command line       =
//        -f C:\Users\17054\AppData\Local\Temp\EWD822.tmp ("D:\Software
//        code\Projects Software\Projects new\0. Non AMI\SUGAM Based\0. GDEV72
//        - BootCode-20210104T103109Z-001\0. GDEV72 -
//        BootCode\source_code\source_files\pcb.c" --core s3 --code_model near
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
//        BootCode\Debug\List\pcb.s
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

        EXTERN _delay_ms

        PUBLIC __A_P4
        PUBLIC __A_P5
        PUBLIC __A_P7
        PUBLIC __A_PFSEG2
        PUBLIC __A_PFSEG4
        PUBLIC __A_PIOR0
        PUBLIC __A_PM4
        PUBLIC __A_PM5
        PUBLIC __A_PM7
        PUBLIC __A_POM5
        PUBLIC _pcb_init_universal
        PUBLIC _pcb_io_redirection
        
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
        
// D:\Software code\Projects Software\Projects new\0. Non AMI\SUGAM Based\0. GDEV72 - BootCode-20210104T103109Z-001\0. GDEV72 - BootCode\source_code\source_files\pcb.c
//    1 /***********************************************************************************************************************
//    2 * File Name       : pcb.c
//    3 * Current Version : rev_01  
//    4 * Tool-Chain      : IAR Systems
//    5 * Description     : Functionality related to Meter operation
//    6 * Creation Date   : 30-12-2019
//    7 * Company         : Genus Power Infrastructures Limited, Jaipur
//    8 * Author          : dheeraj.singhal
//    9 * Version History : rev_01 : initial release
//   10 ***********************************************************************************************************************/
//   11 /************************************ Includes **************************************/
//   12 #include "pcb.h"

        ASEGN `.sbss.noinit`:DATA:NOROOT,0fff04H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_P4
// __no_init union <unnamed>#7 volatile __saddr _A_P4
__A_P4:
        DS 1

        ASEGN `.sbss.noinit`:DATA:NOROOT,0fff05H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_P5
// __no_init union <unnamed>#8 volatile __saddr _A_P5
__A_P5:
        DS 1

        ASEGN `.sbss.noinit`:DATA:NOROOT,0fff07H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_P7
// __no_init union <unnamed>#10 volatile __saddr _A_P7
__A_P7:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0fff24H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_PM4
// __no_init union <unnamed>#39 volatile __sfr _A_PM4
__A_PM4:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0fff25H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_PM5
// __no_init union <unnamed>#40 volatile __sfr _A_PM5
__A_PM5:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0fff27H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_PM7
// __no_init union <unnamed>#42 volatile __sfr _A_PM7
__A_PM7:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0055H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_POM5
// __no_init union <unnamed>#248 volatile _A_POM5
__A_POM5:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0077H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_PIOR0
// __no_init union <unnamed>#255 volatile __no_bit_access _A_PIOR0
__A_PIOR0:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0302H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_PFSEG2
// __no_init union <unnamed>#531 volatile _A_PFSEG2
__A_PFSEG2:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0304H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_PFSEG4
// __no_init union <unnamed>#533 volatile _A_PFSEG4
__A_PFSEG4:
        DS 1
//   13 #include "system.h"
//   14 #include "lcd.h"
//   15 /************************************ Local Variables *****************************************/
//   16 /************************************ Extern Variables *****************************************/
//   17 
//   18 /************************************ Local Functions *******************************/
//   19 /************************************ Extern Functions ******************************/
//   20 void pcb_io_redirection();
//   21 void pcb_init_universal();
//   22 
//   23 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock0 Using cfiCommon0
          CFI Function _pcb_io_redirection
          CFI NoCalls
        CODE
//   24 void pcb_io_redirection()
//   25 {
_pcb_io_redirection:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//   26   /* IO Redirection register */
//   27   PIOR0 = bit3; /* As RTCOUT has been used on alternative pin */
        MOV       0x77, #0x8         ;; 1 cycle
//   28 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock0
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 7 cycles
        REQUIRE __A_PIOR0

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock1 Using cfiCommon0
          CFI Function _pcb_init_universal
        CODE
//   29 void pcb_init_universal()
//   30 {
_pcb_init_universal:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//   31   delay_ms(1); /* 300 us time is needed for EXLVD pin stabilisation */
        MOVW      AX, #0x1           ;; 1 cycle
          CFI FunCall _delay_ms
        CALL      _delay_ms          ;; 3 cycles
//   32   
//   33    /* BAT_CTRL P74*/
//   34     PFSEG2 &= ~bit4;
        CLR1      0xF0302.4          ;; 2 cycles
//   35     PM7 &= ~BAT_CTRL;
        CLR1      0xFFF27.4          ;; 2 cycles
//   36     BAT_CTRL_ENABLE;      
        CLR1      S:0xFFF07.4        ;; 2 cycles
//   37     
//   38     /* TXD2 P56*/
//   39     /* Set TxD1 UART Mode P04*/
//   40 //    PM0 &= ~TXD1_MISO;
//   41 //    POM0 &= ~TXD1_MISO;
//   42 //    TXD1_MISO_HIGH;
//   43     
//   44     /* TXD2 P56*/
//   45     PFSEG4 &= ~bit6;
        CLR1      0xF0304.6          ;; 2 cycles
//   46     PM5 &= ~TXD2_PIN;
        CLR1      0xFFF25.6          ;; 2 cycles
//   47     POM5 &= ~TXD2_PIN;
        CLR1      0xF0055.6          ;; 2 cycles
//   48     TXD2_PIN_HIGH;
        SET1      S:0xFFF05.6        ;; 2 cycles
//   49     
//   50     /* RXD2 P55*/
//   51     PFSEG4 &= ~bit5;
        CLR1      0xF0304.5          ;; 2 cycles
//   52     PM5 |= RXD2_PIN;
        SET1      0xFFF25.5          ;; 2 cycles
//   53     
//   54     /* SD P51 */
//   55     bitClear(PFSEG4,bit1);
        CLR1      0xF0304.1          ;; 2 cycles
//   56     bitClear(PM5,SD);
        CLR1      0xFFF25.1          ;; 2 cycles
//   57     bitClear(P5,SD); 
        CLR1      S:0xFFF05.1        ;; 2 cycles
//   58     
//   59     
//   60     /* RXD2 P55*/
//   61 //    PM0 |= RXD1_MOSI;
//   62     
//   63     /* E-CAL P43*/
//   64     PM4 &= ~LED_ECAL;
        CLR1      0xFFF24.3          ;; 2 cycles
//   65     LED_ECAL_LOW; 
        CLR1      S:0xFFF04.3        ;; 2 cycles
//   66     
//   67     
//   68     /* BACKLIGHT_CTRL P53 */
//   69     bitClear(PFSEG4,bit3);
        CLR1      0xF0304.3          ;; 2 cycles
//   70     bitClear(PM5,LED_BACKLIGHT);
        CLR1      0xFFF25.3          ;; 2 cycles
//   71     LED_BACKLIGHT_LOW;     
        CLR1      S:0xFFF05.3        ;; 2 cycles
//   72     
//   73     /* R-CAL P50 */
//   74     bitClear(PFSEG4,bit0);
        CLR1      0xF0304.0          ;; 2 cycles
//   75     bitClear(PM5,LED_RCAL);
        CLR1      0xFFF25.0          ;; 2 cycles
//   76     LED_RCAL_LOW;
        CLR1      S:0xFFF05.0        ;; 2 cycles
//   77 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock1
        ; ------------------------------------- Block: 50 cycles
        ; ------------------------------------- Total: 50 cycles
        REQUIRE __A_PFSEG2
        REQUIRE __A_PM7
        REQUIRE __A_P7
        REQUIRE __A_PFSEG4
        REQUIRE __A_PM5
        REQUIRE __A_POM5
        REQUIRE __A_P5
        REQUIRE __A_PM4
        REQUIRE __A_P4

        SECTION `.iar_vfe_header`:DATA:NOALLOC:NOROOT(1)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
        DC32 0

        END
// 
//  7 bytes in section .bss.noinit   (abs)
//  3 bytes in section .sbss.noinit  (abs)
// 79 bytes in section .text
// 
// 79 bytes of CODE memory
//  0 bytes of DATA memory (+ 10 bytes shared)
//
//Errors: none
//Warnings: none
