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
//        BootCode\source_code\source_files\system.c
//    Command line       =
//        -f C:\Users\17054\AppData\Local\Temp\EWDE2E.tmp ("D:\Software
//        code\Projects Software\Projects new\0. Non AMI\SUGAM Based\0. GDEV72
//        - BootCode-20210104T103109Z-001\0. GDEV72 -
//        BootCode\source_code\source_files\system.c" --core s3 --code_model
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
//        BootCode\Debug\List\system.s
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

        EXTERN _FSL_Serial_counter
        EXTERN _FSL_TX_Data_Pkt_length
        EXTERN _Flash_write_flag
        EXTERN _FSL_Analyse_Data_Pkt_f
        EXTERN _flash_complete
        EXTERN _FSL_RX_Data_Pkt_length
        EXTERN _FSL_Serial_rxcounter
        EXTERN _R_LVD_Create
        EXTERN _R_LVD_Start_EXLVD
        EXTERN _R_WDT_Create
        EXTERN _Reset_Add
        EXTERN _WriteBlock1
        EXTERN _Write_addr
        EXTERN _Write_addr_temp
        EXTERN _clock_init
        EXTERN _increment
        EXTERN _pcb_io_redirection

        PUBLIC __A_LVDEXLVD
        PUBLIC __A_P5
        PUBLIC __A_RPECTL
        PUBLIC _backlight_operation
        PUBLIC _clear_ram
        PUBLIC _flag1
        PUBLIC _last_interrupt
        PUBLIC _lvd_start
        PUBLIC _opbyte0
        PUBLIC _opbyte1
        PUBLIC _opbyte2
        PUBLIC _opbyte3
        PUBLIC _read_operating_mode
        PUBLIC _secuid
        PUBLIC _system_init
        
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
        
// D:\Software code\Projects Software\Projects new\0. Non AMI\SUGAM Based\0. GDEV72 - BootCode-20210104T103109Z-001\0. GDEV72 - BootCode\source_code\source_files\system.c
//    1 /***********************************************************************************************************************
//    2 * File Name       : system.c
//    3 * Current Version : rev_  
//    4 * Tool-Chain      : IAR Systems
//    5 * Description     : This file contains routines for system operation
//    6 * Creation Date   : 30-12-2019
//    7 * Company         : Genus Power Infrastructures Limited, Jaipur
//    8 * Author          : dheeraj.singhal
//    9 * Version History : 
//   10 ***********************************************************************************************************************/
//   11 /************************************ Includes **************************************/
//   12 #include "system.h"

        ASEGN `.sbss.noinit`:DATA:NOROOT,0fff05H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_P5
// __no_init union <unnamed>#8 volatile __saddr _A_P5
__A_P5:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f00f5H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_RPECTL
// __no_init union <unnamed>#282 volatile _A_RPECTL
__A_RPECTL:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0335H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_LVDEXLVD
// __no_init union <unnamed>#550 volatile _A_LVDEXLVD
__A_LVDEXLVD:
        DS 1
//   13 
//   14 /************************************ Local Variables *****************************************/
//   15 /************************************ Extern Variables *****************************************/
//   16 

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   17 flag_union flag1;
_flag1:
        DS 1
//   18 

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   19 us8 last_interrupt;
_last_interrupt:
        DS 1
//   20 
//   21 /************************************ Local Functions *******************************/
//   22 void system_reset_source();
//   23 
//   24 /************************************ Extern Functions ******************************/
//   25 void clear_ram();
//   26 void system_init();
//   27 void lvd_start();
//   28 
//   29 us8 read_operating_mode();
//   30 void backlight_operation();
//   31 /* Set option bytes */
//   32 #pragma location = "OPTBYTE"
//   33 #if WDT_TIME == WDT_TIME_474MS
//   34     __root const uint8_t opbyte0 = 0x7AU;
//   35 #elif WDT_TIME == WDT_TIME_949MS
//   36     __root const uint8_t opbyte0 = 0x7CU;
//   37 #else                                                                           //WDT_TIME_3799MS

        SECTION OPTBYTE:CONST:REORDER:ROOT(0)
//   38     __root const uint8_t opbyte0 = 0x7EU;
_opbyte0:
        DATA8
        DB 126
//   39 #endif
//   40 
//   41 #pragma location = "OPTBYTE"

        SECTION OPTBYTE:CONST:REORDER:ROOT(0)
//   42 __root const uint8_t opbyte1 = 0x36U;
_opbyte1:
        DATA8
        DB 54
//   43 #pragma location = "OPTBYTE"

        SECTION OPTBYTE:CONST:REORDER:ROOT(0)
//   44 __root const uint8_t opbyte2 = 0xE0U;
_opbyte2:
        DATA8
        DB 224
//   45 #pragma location = "OPTBYTE"

        SECTION OPTBYTE:CONST:REORDER:ROOT(0)
//   46 __root const uint8_t opbyte3 = 0x84U;
_opbyte3:
        DATA8
        DB 132
//   47 
//   48 /* Set security ID */
//   49 #pragma location = "SECUID"

        SECTION SECUID:CONST:REORDER:ROOT(0)
//   50 __root const uint8_t secuid[10] = 
_secuid:
        DATA8
        DB 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
//   51     {0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U};
//   52 
//   53 
//   54 
//   55 /***********************************************************************************************************************
//   56 * Function Name: system_init
//   57 * Description  : initialises system settings which are not relevent to particular application
//   58 * Arguments    : nothing
//   59 * Return Value : nothing
//   60 ***********************************************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock0 Using cfiCommon0
          CFI Function _system_init
        CODE
//   61 void system_init()
//   62 {
_system_init:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//   63     //system_reset_source();
//   64     
//   65     /* Reset through ram parity error are disabled */
//   66     RPERDIS = 1;
        SET1      0xF00F5.7          ;; 2 cycles
//   67     
//   68     pcb_io_redirection();
          CFI FunCall _pcb_io_redirection
        CALL      _pcb_io_redirection  ;; 3 cycles
//   69 
//   70     /* Enabling CRC checking of flash */
//   71     //CRC0CTL = 0x80U;
//   72     
//   73     /* Illegal writing access reset enabled */
//   74     //IAWCTL = 0x80U;
//   75     
//   76     clock_init();
          CFI FunCall _clock_init
        CALL      _clock_init        ;; 3 cycles
//   77     //clock_change(CLOCK_1_5MHZ);
//   78     
//   79     R_LVD_Create();
          CFI FunCall _R_LVD_Create
        CALL      _R_LVD_Create      ;; 3 cycles
//   80     
//   81     wdt_init();
          CFI FunCall _R_WDT_Create
        CALL      _R_WDT_Create      ;; 3 cycles
//   82 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock0
        ; ------------------------------------- Block: 20 cycles
        ; ------------------------------------- Total: 20 cycles
        REQUIRE __A_RPECTL
//   83 
//   84 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock1 Using cfiCommon0
          CFI Function _clear_ram
          CFI NoCalls
        CODE
//   85 void clear_ram()
//   86 {
_clear_ram:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//   87   Write_addr=0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_Write_addr, AX  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_Write_addr+2, AX  ;; 1 cycle
//   88   Write_addr_temp=0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_Write_addr_temp, AX  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_Write_addr_temp+2, AX  ;; 1 cycle
//   89   increment=0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_increment, AX   ;; 1 cycle
//   90   Reset_Add=0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_Reset_Add, AX   ;; 1 cycle
//   91   FSL_RX_Data_Pkt_length=0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_FSL_RX_Data_Pkt_length, AX  ;; 1 cycle
//   92   FSL_Serial_rxcounter=0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_FSL_Serial_rxcounter, AX  ;; 1 cycle
//   93   WriteBlock1=0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_WriteBlock1, AX  ;; 1 cycle
//   94   FSL_Serial_counter=0;
        MOV       N:_FSL_Serial_counter, #0x0  ;; 1 cycle
//   95   FSL_TX_Data_Pkt_length=0;
        MOV       N:_FSL_TX_Data_Pkt_length, #0x0  ;; 1 cycle
//   96   Flash_write_flag=0;
        MOV       N:_Flash_write_flag, #0x0  ;; 1 cycle
//   97   FSL_Analyse_Data_Pkt_f=0;
        MOV       N:_FSL_Analyse_Data_Pkt_f, #0x0  ;; 1 cycle
//   98   flash_complete=0;
        MOV       N:_flash_complete, #0x0  ;; 1 cycle
//   99 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock1
        ; ------------------------------------- Block: 29 cycles
        ; ------------------------------------- Total: 29 cycles
//  100             

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock2 Using cfiCommon0
          CFI Function _read_operating_mode
          CFI NoCalls
        CODE
//  101 us8 read_operating_mode()
//  102 {
_read_operating_mode:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  103   if(flag_lvd_exlvd_instant == 1)
        MOVW      HL, #0x335         ;; 1 cycle
        MOV1      CY, [HL].6         ;; 1 cycle
        CLRB      A                  ;; 1 cycle
        ROLC      A, 0x1             ;; 1 cycle
//  104   {
//  105     return 1;
//  106   }
//  107   else
//  108   {
//  109     return 0;
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock2
        ; ------------------------------------- Block: 10 cycles
        ; ------------------------------------- Total: 10 cycles
        REQUIRE __A_LVDEXLVD
//  110   }
//  111 }
//  112 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock3 Using cfiCommon0
          CFI Function _lvd_start
          CFI FunCall _R_LVD_Start_EXLVD
        CODE
//  113 void lvd_start()
//  114 {
_lvd_start:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  115   //R_LVD_InterruptMode_Start();
//  116   //R_LVD_Start_VDD();
//  117   //R_LVD_Start_VRTC();
//  118   R_LVD_Start_EXLVD();
        CALL      _R_LVD_Start_EXLVD  ;; 3 cycles
//  119 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock3
        ; ------------------------------------- Block: 9 cycles
        ; ------------------------------------- Total: 9 cycles

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock4 Using cfiCommon0
          CFI Function _backlight_operation
          CFI NoCalls
        CODE
//  120 void backlight_operation()
//  121 {
_backlight_operation:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  122   if(battery_mode_f == 1 || flag_lvd_exlvd_instant == 1)
        MOVW      HL, #LWRD(_flag1)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BC        ??backlight_operation_0  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      HL, #0x335         ;; 1 cycle
        MOV1      CY, [HL].6         ;; 1 cycle
        BNC       ??backlight_operation_1  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  123   {
//  124     LED_BACKLIGHT_LOW;
??backlight_operation_0:
        CLR1      S:0xFFF05.3        ;; 2 cycles
        RET                          ;; 6 cycles
        ; ------------------------------------- Block: 8 cycles
//  125   }
//  126   else
//  127   {
//  128     LED_BACKLIGHT_HIGH;
??backlight_operation_1:
        SET1      S:0xFFF05.3        ;; 2 cycles
//  129   }
//  130 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock4
        ; ------------------------------------- Block: 8 cycles
        ; ------------------------------------- Total: 28 cycles
        REQUIRE __A_LVDEXLVD
        REQUIRE __A_P5

        SECTION `.iar_vfe_header`:DATA:NOALLOC:NOROOT(1)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
        DC32 0

        END
// 
//   2 bytes in section .bss
//   2 bytes in section .bss.noinit   (abs)
//   1 byte  in section .sbss.noinit  (abs)
// 127 bytes in section .text
//   4 bytes in section OPTBYTE
//  10 bytes in section SECUID
// 
// 127 bytes of CODE  memory
//  14 bytes of CONST memory
//   2 bytes of DATA  memory (+ 3 bytes shared)
//
//Errors: none
//Warnings: none
