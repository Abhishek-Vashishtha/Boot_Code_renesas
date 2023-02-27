///////////////////////////////////////////////////////////////////////////////
//
// IAR C/C++ Compiler V4.20.1.2260 for RL78               20/Dec/2020  23:56:49
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
//        BootCode\source_code\source_files\eprom_i2c.c
//    Command line       =
//        -f C:\Users\laptop\AppData\Local\Temp\EW43C2.tmp ("D:\Dheeraj\New
//        folder\0. GDEV72 - BootCode\source_code\source_files\eprom_i2c.c"
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
//        D:\Dheeraj\New folder\0. GDEV72 - BootCode\Debug\List\eprom_i2c.s
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

        EXTERN _clock_select
        EXTERN _flag_sleep
        EXTERN ?UL_CMP_L03
        EXTERN _R_WDT_Restart
        EXTERN _checksum_mem
        EXTERN _delay_ms

        PUBLIC __A_P5
        PUBLIC __A_P8
        PUBLIC __A_PFSEG4
        PUBLIC __A_PFSEG5
        PUBLIC __A_PM5
        PUBLIC __A_PM8
        PUBLIC _byte_read
        PUBLIC _byte_write
        PUBLIC _eep_add_write
        PUBLIC _eprom_error_handler
        PUBLIC _eprom_init
        PUBLIC _eprom_magnet_vcc_disable
        PUBLIC _eprom_magnet_vcc_enable
        PUBLIC _eprom_read
        PUBLIC _eprom_write
        PUBLIC _eprom_write_disable
        PUBLIC _eprom_write_enable
        PUBLIC _error_cnt_eep
        PUBLIC _error_cnt_eep0
        PUBLIC _error_cnt_eep1
        PUBLIC _error_cnt_eep2
        PUBLIC _fill_opr_ff
        PUBLIC _fill_opr_test
        PUBLIC _fill_oprzero
        PUBLIC _flag_eep1
        PUBLIC _flag_eeprom_error
        PUBLIC _nob_in_page
        PUBLIC _opr_data
        PUBLIC _page_read
        PUBLIC _page_write
        PUBLIC _page_write_1
        PUBLIC _reset_mem
        PUBLIC _retry_cnt_read
        PUBLIC _retry_cnt_write
        PUBLIC _start_condition
        PUBLIC _stop_condition
        PUBLIC _wait_high
        PUBLIC _wait_low
        
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
          CFI A Undefined
          CFI X Undefined
          CFI B Undefined
          CFI C Undefined
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
          CFI EndCommon cfiCommon1
        
        
          CFI Common cfiCommon2 Using cfiNames0
          CFI CodeAlign 1
          CFI DataAlign 1
          CFI ReturnAddress ?RET CODE
          CFI CFA SP+4
          CFI A Undefined
          CFI X Undefined
          CFI B Undefined
          CFI C Undefined
          CFI D Undefined
          CFI E Undefined
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
          CFI EndCommon cfiCommon2
        
// D:\Dheeraj\New folder\0. GDEV72 - BootCode\source_code\source_files\eprom_i2c.c
//    1 /***********************************************************************************************************************
//    2 * File Name       : eprom_i2c.c
//    3 * Current Version : rev_02  
//    4 * Tool-Chain      : IAR Systems
//    5 * Description     : This file implements I2C routine using General purpose I/O pins
//    6 * Creation Date   : 30-12-2019
//    7 * Company         : Genus Power Infrastructures Limited, Jaipur
//    8 * Author          : dheeraj.singhal
//    9 * Version History : rev_01 : the file is prepared for basic operation of EEPROM
//   10 rev_02 : Issue resolved from old library, new features added (flexible eeprom writing), retry for write operation also
//   11 
//   12 ***********************************************************************************************************************/
//   13 /************************************ Includes **************************************/
//   14 #include "eprom_i2c.h"

        ASEGN `.sbss.noinit`:DATA:NOROOT,0fff05H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_P5
// __no_init union <unnamed>#8 volatile __saddr _A_P5
__A_P5:
        DS 1

        ASEGN `.sbss.noinit`:DATA:NOROOT,0fff08H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_P8
// __no_init union <unnamed>#11 volatile __saddr _A_P8
__A_P8:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0fff25H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_PM5
// __no_init union <unnamed>#40 volatile __sfr _A_PM5
__A_PM5:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0fff28H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_PM8
// __no_init union <unnamed>#43 volatile __sfr _A_PM8
__A_PM8:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0304H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_PFSEG4
// __no_init union <unnamed>#533 volatile _A_PFSEG4
__A_PFSEG4:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0305H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_PFSEG5
// __no_init union <unnamed>#534 volatile _A_PFSEG5
__A_PFSEG5:
        DS 1
//   15 
//   16 /************************************ Local Variables *****************************************/
//   17 static const us8 MemVerficationString[16] = {0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,0x88};
//   18 static const us8 MemVerficationString1[16] = {0xF1,0xF2,0xF3,0xF4,0xF5,0xF6,0xF7,0xF8,0xF9,0xFA,0xFB,0xFC,0xFD,0xFE,0xFF,0x78};

        SECTION `.data`:DATA:REORDER:NOROOT(1)
//   19 const us8 nob_in_page[8] = {16,32,48,64,80,96,112,128};
_nob_in_page:
        DATA8
        DB 16, 32, 48, 64, 80, 96, 112, 128

        SECTION `.data`:DATA:REORDER:NOROOT(1)
//   20 const us8 eep_add_write[NO_OF_EEP] = {EEP0_W,EEP1_W,EEP2_W};
_eep_add_write:
        DATA8
        DB 160, 162, 164, 0
//   21 
//   22 /************************************ Extern Variables *****************************************/

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   23 flag_union flag_eep1,flag_eeprom_error;
_flag_eep1:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_flag_eeprom_error:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   24 us8 opr_data[128];
_opr_data:
        DS 128

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   25 us16 error_cnt_eep,error_cnt_eep0,error_cnt_eep1,error_cnt_eep2;
_error_cnt_eep:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_error_cnt_eep0:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_error_cnt_eep1:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_error_cnt_eep2:
        DS 2
//   26 

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   27 us16 retry_cnt_write,retry_cnt_read;
_retry_cnt_write:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_retry_cnt_read:
        DS 2
//   28 /************************************ Local Functions *******************************/
//   29 void start_condition();
//   30 void stop_condition();
//   31 us8 byte_write(us8 iic_writeData);
//   32 void byte_read(us8 *iic_readData, us8 ackData);
//   33 __near_func void wait_high();
//   34 __near_func void wait_low();
//   35 void reset_mem();
//   36 EEP_ret_value page_write(us16 Add, us8 Mem, us8 len, us8 *opr_location);
//   37 EEP_ret_value page_write_1(us16 Add, us8 Mem, us8 len);
//   38 EEP_ret_value page_read(us16 Add, us8 Mem, us8 len);
//   39 
//   40 /************************************ Extern Functions ******************************/
//   41 void fill_opr_test(us8 no_of_bytes);
//   42 void fill_oprzero(us8 no_of_bytes);
//   43 void fill_opr_ff(us8 no_of_bytes);
//   44 void eprom_magnet_vcc_disable();
//   45 void eprom_magnet_vcc_enable();
//   46 void eprom_write_enable();
//   47 void eprom_write_disable();
//   48 EEP_ret_value eprom_read(us16 address, us8 mem_id, PAGE_SIZE page_size, EEP_checksum_mem_mode checksum_mem_mode);
//   49 EEP_ret_value eprom_write(us16 address, us8 mem_id, us16 no_of_bytes, PAGE_SIZE page_size, EEP_checksum_mem_mode checksum_mem_mode);
//   50 
//   51 
//   52 void eprom_init();
//   53 void Eprom_ReadWM(us16 Add,us8 Mem,us8 len);
//   54 void Eprom_WriteWM(us16 Add,us8 Mem, us8 len);
//   55 us8 eprom_diagnostic(us8 MemoryType,us8 DeviceAddress);
//   56 void eprom_error_handler(us8 Mem, us8 status);
//   57 void mem_test();
//   58 void eprom_fail_log();
//   59 us8 findDeviceAddress(us8 MemoryType, us8 MemoryNo);
//   60 
//   61 /****************************************************************************************
//   62 ****************************  Local Functions definations  ******************************
//   63 ****************************************************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock0 Using cfiCommon0
          CFI Function _start_condition
        CODE
//   64 void start_condition()
//   65 {
_start_condition:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 2
        SUBW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+6
//   66   us8 temp_cntr;
//   67   
//   68   SDA_HIGH;
        SET1      S:0xFFF08.4        ;; 2 cycles
//   69   SCL_HIGH;
        SET1      S:0xFFF05.7        ;; 2 cycles
//   70   
//   71   SDA_INPUT; 
        CLR1      0xF0305.0          ;; 2 cycles
        SET1      0xFFF28.4          ;; 2 cycles
//   72   SCL_INPUT; 
        CLR1      0xF0304.7          ;; 2 cycles
        SET1      0xFFF25.7          ;; 2 cycles
//   73   
//   74   wait_high();                       
          CFI FunCall _wait_high
        CALL      _wait_high         ;; 3 cycles
//   75   
//   76   for(temp_cntr=0; temp_cntr<10; temp_cntr++)
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        ; ------------------------------------- Block: 18 cycles
??start_condition_0:
        MOV       A, [SP]            ;; 1 cycle
        CMP       A, #0xA            ;; 1 cycle
        BNC       ??eprom_init_0     ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//   77   {
//   78     if(IS_SDA_LOW || IS_SCL_LOW)
        MOV       A, S:0xFFF08       ;; 1 cycle
        AND       A, #0x10           ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        BZ        ??eprom_init_1     ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOV       A, S:0xFFF05       ;; 1 cycle
        AND       A, #0x80           ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        BNZ       ??eprom_init_2     ;; 4 cycles
          CFI FunCall _wait_high
        ; ------------------------------------- Block: 7 cycles
//   79     {
//   80       wait_high();
??eprom_init_1:
        CALL      _wait_high         ;; 3 cycles
//   81     }
        MOV       A, [SP]            ;; 1 cycle
        INC       A                  ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        BR        S:??start_condition_0  ;; 3 cycles
        ; ------------------------------------- Block: 9 cycles
//   82     else
//   83     {
//   84       SDA_OUTPUT;
??eprom_init_2:
        CLR1      0xF0305.0          ;; 2 cycles
        CLR1      0xFFF28.4          ;; 2 cycles
//   85       SCL_OUTPUT;
        CLR1      0xF0304.7          ;; 2 cycles
        CLR1      0xFFF25.7          ;; 2 cycles
//   86       SDA_LOW;
        CLR1      S:0xFFF08.4        ;; 2 cycles
//   87       wait_high();
          CFI FunCall _wait_high
        CALL      _wait_high         ;; 3 cycles
//   88       break;
        ; ------------------------------------- Block: 13 cycles
//   89     }
//   90   }
//   91   if(temp_cntr == 10)
??eprom_init_0:
        MOV       A, [SP]            ;; 1 cycle
        CMP       A, #0xA            ;; 1 cycle
        BNZ       ??eprom_init_3     ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//   92   {
//   93     busbusy_f = 1;
        SET1      N:_flag_eep1.1     ;; 2 cycles
//   94     SDA_OUTPUT;
        CLR1      0xF0305.0          ;; 2 cycles
        CLR1      0xFFF28.4          ;; 2 cycles
//   95     SCL_OUTPUT;
        CLR1      0xF0304.7          ;; 2 cycles
        CLR1      0xFFF25.7          ;; 2 cycles
        ; ------------------------------------- Block: 10 cycles
//   96   }
//   97 }		
??eprom_init_3:
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock0
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 83 cycles
        REQUIRE __A_P8
        REQUIRE __A_P5
        REQUIRE __A_PFSEG5
        REQUIRE __A_PM8
        REQUIRE __A_PFSEG4
        REQUIRE __A_PM5
//   98 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock1 Using cfiCommon0
          CFI Function _stop_condition
        CODE
//   99 void stop_condition()
//  100 {
_stop_condition:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  101   SCL_LOW; NOP();       /* it is considered that SCL is low before executing this function. Always leave SCL as LOW from Every function */
        CLR1      S:0xFFF05.7        ;; 2 cycles
        NOP                          ;; 1 cycle
//  102   SDA_LOW;
        CLR1      S:0xFFF08.4        ;; 2 cycles
//  103   wait_low();
          CFI FunCall _wait_low
        CALL      _wait_low          ;; 3 cycles
//  104   SCL_HIGH;
        SET1      S:0xFFF05.7        ;; 2 cycles
//  105   wait_low();
          CFI FunCall _wait_low
        CALL      _wait_low          ;; 3 cycles
//  106   SDA_HIGH;
        SET1      S:0xFFF08.4        ;; 2 cycles
//  107   wait_high();
          CFI FunCall _wait_high
        CALL      _wait_high         ;; 3 cycles
//  108 }	
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock1
        ; ------------------------------------- Block: 24 cycles
        ; ------------------------------------- Total: 24 cycles
        REQUIRE __A_P5
        REQUIRE __A_P8
//  109 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock2 Using cfiCommon0
          CFI Function _wait_high
          CFI NoCalls
        CODE
//  110 __near_func void wait_high(void)                            /* 600ns */
//  111 {
_wait_high:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  112   if(clock_select == CLOCK_1_5MHZ)               /* 0.67 us */
        CMP       N:_clock_select, #0x4  ;; 1 cycle
        BNZ       ??eprom_init_4     ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  113   {
//  114     NOP();
        NOP                          ;; 1 cycle
        RET                          ;; 6 cycles
        ; ------------------------------------- Block: 7 cycles
//  115   }
//  116   else if(clock_select == CLOCK_24MHZ)  
??eprom_init_4:
        CMP       N:_clock_select, #0x1  ;; 1 cycle
        BNZ       ??eprom_init_5     ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  117   {
//  118     NOP();
        NOP                          ;; 1 cycle
        RET                          ;; 6 cycles
        ; ------------------------------------- Block: 7 cycles
//  119   }
//  120   else if(clock_select == CLOCK_6MHZ)
??eprom_init_5:
        CMP       N:_clock_select, #0x3  ;; 1 cycle
        BNZ       ??eprom_init_6     ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  121   {
//  122     NOP();
        NOP                          ;; 1 cycle
        RET                          ;; 6 cycles
        ; ------------------------------------- Block: 7 cycles
//  123   }
//  124   else if(clock_select == CLOCK_12MHZ)         /* 20us */
??eprom_init_6:
        CMP       N:_clock_select, #0x2  ;; 1 cycle
        BNZ       ??eprom_init_7     ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  125   {
//  126     NOP();
        NOP                          ;; 1 cycle
        RET                          ;; 6 cycles
        ; ------------------------------------- Block: 7 cycles
//  127   }
//  128   else                                          /* Default clock is 24MHz */
//  129   {
//  130     NOP();
??eprom_init_7:
        NOP                          ;; 1 cycle
//  131   }
//  132 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock2
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 55 cycles

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock3 Using cfiCommon0
          CFI Function _wait_low
          CFI NoCalls
        CODE
//  133 __near_func void wait_low(void)                             /* 1300ns */
//  134 {
_wait_low:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  135   if(clock_select == CLOCK_1_5MHZ)               /* 0.67 us */
        CMP       N:_clock_select, #0x4  ;; 1 cycle
        BNZ       ??eprom_init_8     ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  136   {
//  137     NOP();
        NOP                          ;; 1 cycle
        RET                          ;; 6 cycles
        ; ------------------------------------- Block: 7 cycles
//  138   }
//  139   else if(clock_select == CLOCK_24MHZ)  
??eprom_init_8:
        CMP       N:_clock_select, #0x1  ;; 1 cycle
        BNZ       ??eprom_init_9     ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  140   {
//  141     NOP();
        NOP                          ;; 1 cycle
        RET                          ;; 6 cycles
        ; ------------------------------------- Block: 7 cycles
//  142   }
//  143   else if(clock_select == CLOCK_6MHZ)
??eprom_init_9:
        CMP       N:_clock_select, #0x3  ;; 1 cycle
        BNZ       ??eprom_init_10    ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  144   {
//  145     NOP();
        NOP                          ;; 1 cycle
        RET                          ;; 6 cycles
        ; ------------------------------------- Block: 7 cycles
//  146   }
//  147   else if(clock_select == CLOCK_12MHZ)         /* 20us */
??eprom_init_10:
        CMP       N:_clock_select, #0x2  ;; 1 cycle
        BNZ       ??eprom_init_11    ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  148   {
//  149     NOP();
        NOP                          ;; 1 cycle
        RET                          ;; 6 cycles
        ; ------------------------------------- Block: 7 cycles
//  150   }
//  151   else                                          /* Default clock is 24MHz */
//  152   {
//  153     NOP();
??eprom_init_11:
        NOP                          ;; 1 cycle
//  154   }
//  155 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock3
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 55 cycles
//  156 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock4 Using cfiCommon0
          CFI Function _reset_mem
        CODE
//  157 void reset_mem(void)
//  158 {
_reset_mem:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 2
        SUBW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+6
//  159     us8 i,j;
//  160     SDA_INPUT;
        CLR1      0xF0305.0          ;; 2 cycles
        SET1      0xFFF28.4          ;; 2 cycles
//  161     for(j=0;j<5;j++)
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP+0x01], A       ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
??reset_mem_0:
        MOV       A, [SP+0x01]       ;; 1 cycle
        CMP       A, #0x5            ;; 1 cycle
        BNC       ??eprom_init_12    ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  162     {
//  163         for(i=0;i<9;i++)
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
??reset_mem_1:
        MOV       A, [SP]            ;; 1 cycle
        CMP       A, #0x9            ;; 1 cycle
        BNC       ??eprom_init_13    ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  164         {
//  165             SCL_LOW;
        CLR1      S:0xFFF05.7        ;; 2 cycles
//  166             wait_high();
          CFI FunCall _wait_high
        CALL      _wait_high         ;; 3 cycles
//  167             SCL_HIGH;
        SET1      S:0xFFF05.7        ;; 2 cycles
//  168             wait_low();
          CFI FunCall _wait_low
        CALL      _wait_low          ;; 3 cycles
//  169             if(IS_SDA_HIGH)
        MOV       A, S:0xFFF08       ;; 1 cycle
        AND       A, #0x10           ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        BNZ       ??eprom_init_13    ;; 4 cycles
        ; ------------------------------------- Block: 17 cycles
//  170             {
//  171                 break;
//  172             }
//  173         }
        MOV       A, [SP]            ;; 1 cycle
        INC       A                  ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        BR        S:??reset_mem_1    ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
//  174         if(i < 9)
??eprom_init_13:
        MOV       A, [SP]            ;; 1 cycle
        CMP       A, #0x9            ;; 1 cycle
        BC        ??eprom_init_12    ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  175         {
//  176             break;
//  177         }
//  178     }
        MOV       A, [SP+0x01]       ;; 1 cycle
        INC       A                  ;; 1 cycle
        MOV       [SP+0x01], A       ;; 1 cycle
        BR        S:??reset_mem_0    ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
//  179     SDA_OUTPUT;
??eprom_init_12:
        CLR1      0xF0305.0          ;; 2 cycles
        CLR1      0xFFF28.4          ;; 2 cycles
//  180     SDA_HIGH;
        SET1      S:0xFFF08.4        ;; 2 cycles
//  181     if(i == 9 && j == 5)
        MOV       A, [SP]            ;; 1 cycle
        CMP       A, #0x9            ;; 1 cycle
        BNZ       ??eprom_init_14    ;; 4 cycles
        ; ------------------------------------- Block: 12 cycles
        MOV       A, [SP+0x01]       ;; 1 cycle
        CMP       A, #0x5            ;; 1 cycle
        BNZ       ??eprom_init_14    ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  182     {
//  183         /* Power up reset*/
//  184         flag_mem_fail_pwr_up = 1;
        SET1      N:_flag_sleep.3    ;; 2 cycles
        BR        S:??eprom_init_15  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  185     }
//  186     else
//  187     {
//  188       flag_mem_fail_pwr_up = 0;
??eprom_init_14:
        CLR1      N:_flag_sleep.3    ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
//  189     }
//  190 }
??eprom_init_15:
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock4
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 88 cycles
        REQUIRE __A_PFSEG5
        REQUIRE __A_PM8
        REQUIRE __A_P5
        REQUIRE __A_P8
//  191 
//  192 
//  193 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock5 Using cfiCommon0
          CFI Function _byte_write
        CODE
//  194 us8 byte_write(us8 iic_writeData)
//  195 {
_byte_write:
        ; * Stack frame (at entry) *
        ; Param size: 0
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+6
        ; Auto size: 4
        SUBW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+8
//  196   us8 maskData = 0x80;	                        // MSB first
        MOV       A, #0x80           ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  197   us8 ret = NACK;
        MOV       A, #0x1            ;; 1 cycle
        MOV       [SP+0x01], A       ;; 1 cycle
//  198   us8 i;                             
//  199   
//  200   SCL_LOW;                                         
        CLR1      S:0xFFF05.7        ;; 2 cycles
        ; ------------------------------------- Block: 8 cycles
//  201   while(maskData)                 			
??byte_write_0:
        MOV       A, [SP]            ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        BZ        ??eprom_init_16    ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  202   {	
//  203     if(iic_writeData & maskData)  		
        MOV       A, [SP]            ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        AND       A, X               ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        BZ        ??eprom_init_17    ;; 4 cycles
        ; ------------------------------------- Block: 9 cycles
//  204     {
//  205       SDA_HIGH;   	    		
        SET1      S:0xFFF08.4        ;; 2 cycles
        BR        S:??eprom_init_18  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  206     }
//  207     else
//  208     {
//  209       SDA_LOW;    			
??eprom_init_17:
        CLR1      S:0xFFF08.4        ;; 2 cycles
          CFI FunCall _wait_low
        ; ------------------------------------- Block: 2 cycles
//  210     }
//  211     wait_low();		
??eprom_init_18:
        CALL      _wait_low          ;; 3 cycles
//  212     SCL_HIGH;	    			
        SET1      S:0xFFF05.7        ;; 2 cycles
//  213     wait_high();	    			
          CFI FunCall _wait_high
        CALL      _wait_high         ;; 3 cycles
//  214     SCL_LOW;					
        CLR1      S:0xFFF05.7        ;; 2 cycles
//  215     maskData >>= 1;
        MOV       A, [SP]            ;; 1 cycle
        SHR       A, 0x1             ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        BR        S:??byte_write_0   ;; 3 cycles
        ; ------------------------------------- Block: 16 cycles
//  216   } 
//  217   NOP();NOP();
??eprom_init_16:
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
//  218   // Get ACK from slave at 9th pulse
//  219   //SDA_HIGH;    /* why this is in the earlier code */                                
//  220   SDA_INPUT;  		 
        CLR1      0xF0305.0          ;; 2 cycles
        SET1      0xFFF28.4          ;; 2 cycles
//  221   wait_low();
          CFI FunCall _wait_low
        CALL      _wait_low          ;; 3 cycles
//  222   SCL_HIGH; 
        SET1      S:0xFFF05.7        ;; 2 cycles
//  223   wait_high();
          CFI FunCall _wait_high
        CALL      _wait_high         ;; 3 cycles
//  224   for(i=0;i<200;i++)
        MOV       X, #0x0            ;; 1 cycle
        ; ------------------------------------- Block: 15 cycles
??byte_write_1:
        MOV       A, X               ;; 1 cycle
        CMP       A, #0xC8           ;; 1 cycle
        BNC       ??eprom_init_19    ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  225   {
//  226     if(IS_SDA_LOW) 
        MOV       A, S:0xFFF08       ;; 1 cycle
        AND       A, #0x10           ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        BNZ       ??eprom_init_20    ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  227     {
//  228       ret = ACK;
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP+0x01], A       ;; 1 cycle
//  229       break;
        BR        S:??eprom_init_19  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  230     }				
//  231   }
??eprom_init_20:
        INC       X                  ;; 1 cycle
        BR        S:??byte_write_1   ;; 3 cycles
        ; ------------------------------------- Block: 4 cycles
//  232   SCL_LOW;	
??eprom_init_19:
        CLR1      S:0xFFF05.7        ;; 2 cycles
//  233   SDA_OUTPUT;
        CLR1      0xF0305.0          ;; 2 cycles
        CLR1      0xFFF28.4          ;; 2 cycles
//  234   if(ret == NACK)
        MOV       A, [SP+0x01]       ;; 1 cycle
        CMP       A, #0x1            ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 9 cycles
//  235   {
//  236     nack_rxd_f = 1;
        SET1      N:_flag_eep1.0     ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
//  237   }
//  238   return(ret);
??byte_write_2:
        MOV       A, [SP+0x01]       ;; 1 cycle
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock5
        ; ------------------------------------- Block: 8 cycles
        ; ------------------------------------- Total: 102 cycles
        REQUIRE __A_P5
        REQUIRE __A_P8
        REQUIRE __A_PFSEG5
        REQUIRE __A_PM8
//  239 }
//  240 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock6 Using cfiCommon1
          CFI Function _byte_read
        CODE
//  241 void byte_read(us8 *iic_readData, us8 ackData)
//  242 {
_byte_read:
        ; * Stack frame (at entry) *
        ; Param size: 0
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+6
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+8
        ; Auto size: 6
        SUBW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+10
//  243   us8 maskData=0x80;  		// MSB first
        MOV       A, #0x80           ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  244   us8 readData;
//  245   
//  246   SDA_INPUT;
        CLR1      0xF0305.0          ;; 2 cycles
        SET1      0xFFF28.4          ;; 2 cycles
//  247   *iic_readData = 0;	
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
        ; ------------------------------------- Block: 13 cycles
//  248   
//  249   while(maskData)      
??byte_read_0:
        MOV       A, [SP]            ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        BZ        ??eprom_init_21    ;; 4 cycles
          CFI FunCall _wait_low
        ; ------------------------------------- Block: 6 cycles
//  250   {	
//  251     wait_low();
        CALL      _wait_low          ;; 3 cycles
//  252     readData = *iic_readData | maskData;
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, [HL]            ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        OR        A, X               ;; 1 cycle
        MOV       [SP+0x01], A       ;; 1 cycle
//  253     SCL_HIGH;	
        SET1      S:0xFFF05.7        ;; 2 cycles
//  254     wait_high();
          CFI FunCall _wait_high
        CALL      _wait_high         ;; 3 cycles
//  255     if(IS_SDA_HIGH)
        MOV       A, S:0xFFF08       ;; 1 cycle
        AND       A, #0x10           ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        BZ        ??eprom_init_22    ;; 4 cycles
        ; ------------------------------------- Block: 22 cycles
//  256     {
//  257       *iic_readData = readData;	
        MOV       A, [SP+0x01]       ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, B               ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
//  258     }
//  259     SCL_LOW;
??eprom_init_22:
        CLR1      S:0xFFF05.7        ;; 2 cycles
//  260     maskData >>= 1;
        MOV       A, [SP]            ;; 1 cycle
        SHR       A, 0x1             ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        BR        S:??byte_read_0    ;; 3 cycles
        ; ------------------------------------- Block: 8 cycles
//  261   }
//  262   
//  263   NOP();NOP();
??eprom_init_21:
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
//  264   SDA_OUTPUT;						
        CLR1      0xF0305.0          ;; 2 cycles
        CLR1      0xFFF28.4          ;; 2 cycles
//  265   
//  266   if(ackData == ACK)							
        MOV       A, [SP+0x02]       ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        BNZ       ??eprom_init_23    ;; 4 cycles
        ; ------------------------------------- Block: 12 cycles
//  267   {
//  268     SDA_LOW;						
        CLR1      S:0xFFF08.4        ;; 2 cycles
        BR        S:??eprom_init_24  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  269   }
//  270   else									
//  271   {
//  272     SDA_HIGH;							
??eprom_init_23:
        SET1      S:0xFFF08.4        ;; 2 cycles
          CFI FunCall _wait_low
        ; ------------------------------------- Block: 2 cycles
//  273   }
//  274   
//  275   wait_low();
??eprom_init_24:
        CALL      _wait_low          ;; 3 cycles
//  276   SCL_HIGH;							
        SET1      S:0xFFF05.7        ;; 2 cycles
//  277   
//  278   wait_high();
          CFI FunCall _wait_high
        CALL      _wait_high         ;; 3 cycles
//  279   SCL_LOW;
        CLR1      S:0xFFF05.7        ;; 2 cycles
//  280   
//  281 }
        ADDW      SP, #0x6           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock6
        ; ------------------------------------- Block: 17 cycles
        ; ------------------------------------- Total: 91 cycles
        REQUIRE __A_PFSEG5
        REQUIRE __A_PM8
        REQUIRE __A_P5
        REQUIRE __A_P8
//  282 
//  283 
//  284 
//  285 /****************************************************************************************
//  286 ****************************  Extern Functions definations  ******************************
//  287 ****************************************************************************************/
//  288 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock7 Using cfiCommon0
          CFI Function _fill_opr_test
          CFI NoCalls
        CODE
//  289 void fill_opr_test(us8 no_of_bytes)
//  290 {
_fill_opr_test:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  291   us8 index = 0;
        MOV       B, #0x0            ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
//  292   while(index <  no_of_bytes)
??fill_opr_test_0:
        CMP       B, A               ;; 1 cycle
        BNC       ??eprom_init_25    ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  293   {
//  294     opr_data[index] = index;
        XCH       A, B               ;; 1 cycle
        MOV       C, A               ;; 1 cycle
        MOV       (_opr_data)[C], A  ;; 1 cycle
        XCH       A, B               ;; 1 cycle
//  295     index++;
        INC       B                  ;; 1 cycle
        BR        S:??fill_opr_test_0  ;; 3 cycles
        ; ------------------------------------- Block: 8 cycles
//  296   }
//  297 }
??eprom_init_25:
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock7
        ; ------------------------------------- Block: 6 cycles
        ; ------------------------------------- Total: 20 cycles
//  298 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock8 Using cfiCommon0
          CFI Function _fill_oprzero
          CFI NoCalls
        CODE
//  299 void fill_oprzero(us8 no_of_bytes)
//  300 {
_fill_oprzero:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  301   us8 index = 0;
        MOV       B, #0x0            ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
//  302   while(index <  no_of_bytes)
??fill_oprzero_0:
        CMP       B, A               ;; 1 cycle
        BNC       ??eprom_init_26    ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  303   {
//  304     opr_data[index++] = 0x00;
        MOV       X, #0x0            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       (_opr_data)[B], A  ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        INC       B                  ;; 1 cycle
        BR        S:??fill_oprzero_0  ;; 3 cycles
        ; ------------------------------------- Block: 8 cycles
//  305   }
//  306 }
??eprom_init_26:
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock8
        ; ------------------------------------- Block: 6 cycles
        ; ------------------------------------- Total: 20 cycles
//  307 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock9 Using cfiCommon0
          CFI Function _fill_opr_ff
          CFI NoCalls
        CODE
//  308 void fill_opr_ff(us8 no_of_bytes)
//  309 {
_fill_opr_ff:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  310   us8 index = 0;
        MOV       B, #0x0            ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
//  311   while(index < no_of_bytes)
??fill_opr_ff_0:
        CMP       B, A               ;; 1 cycle
        BNC       ??eprom_init_27    ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  312   {
//  313     opr_data[index++] = 0xFF;
        MOV       X, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       (_opr_data)[B], A  ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        INC       B                  ;; 1 cycle
        BR        S:??fill_opr_ff_0  ;; 3 cycles
        ; ------------------------------------- Block: 8 cycles
//  314   }
//  315 }
??eprom_init_27:
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock9
        ; ------------------------------------- Block: 6 cycles
        ; ------------------------------------- Total: 20 cycles
//  316 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock10 Using cfiCommon0
          CFI Function _eprom_magnet_vcc_disable
          CFI NoCalls
        CODE
//  317 void eprom_magnet_vcc_disable()
//  318 {
_eprom_magnet_vcc_disable:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  319   bitSet(P5,VCC_EPROM_CTRL);
        SET1      S:0xFFF05.4        ;; 2 cycles
//  320 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock10
        ; ------------------------------------- Block: 8 cycles
        ; ------------------------------------- Total: 8 cycles
        REQUIRE __A_P5
//  321 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock11 Using cfiCommon0
          CFI Function _eprom_magnet_vcc_enable
          CFI NoCalls
        CODE
//  322 void eprom_magnet_vcc_enable()
//  323 {
_eprom_magnet_vcc_enable:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  324   bitClear(P5,VCC_EPROM_CTRL);
        CLR1      S:0xFFF05.4        ;; 2 cycles
//  325 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock11
        ; ------------------------------------- Block: 8 cycles
        ; ------------------------------------- Total: 8 cycles
        REQUIRE __A_P5
//  326 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock12 Using cfiCommon0
          CFI Function _eprom_write_enable
          CFI NoCalls
        CODE
//  327 void eprom_write_enable()
//  328 {
_eprom_write_enable:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  329   bitClear(P8, WP);
        CLR1      S:0xFFF08.5        ;; 2 cycles
//  330 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock12
        ; ------------------------------------- Block: 8 cycles
        ; ------------------------------------- Total: 8 cycles
        REQUIRE __A_P8

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock13 Using cfiCommon0
          CFI Function _eprom_write_disable
          CFI NoCalls
        CODE
//  331 void eprom_write_disable()
//  332 {
_eprom_write_disable:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  333   bitSet(P8, WP);
        SET1      S:0xFFF08.5        ;; 2 cycles
//  334 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock13
        ; ------------------------------------- Block: 8 cycles
        ; ------------------------------------- Total: 8 cycles
        REQUIRE __A_P8

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock14 Using cfiCommon1
          CFI Function _page_write_1
        CODE
//  335 EEP_ret_value page_write_1(us16 Add, us8 Mem, us8 len)
//  336 {
_page_write_1:
        ; * Stack frame (at entry) *
        ; Param size: 0
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+6
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+8
        ; Auto size: 4
//  337     static us8 eep_page_break_f = 0;
//  338     static us16 higher_address = 0;
//  339     if(Add/EEP_PAGE_SIZE != (Add+len-1)/EEP_PAGE_SIZE)
        MOV       A, [SP+0x01]       ;; 1 cycle
        MOV       L, A               ;; 1 cycle
        MOV       H, #0x0            ;; 1 cycle
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        DECW      AX                 ;; 1 cycle
        SHRW      AX, 0x7            ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        SHRW      AX, 0x7            ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        ONEB      A                  ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 15 cycles
        CLRB      A                  ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??page_write_1_0:
        MOV       N:`page_write_1::eep_page_break_f`, A  ;; 1 cycle
//  340     {
//  341         eep_page_break_f = 1;
//  342     }
//  343     else
//  344     {
//  345         eep_page_break_f = 0;
//  346     }
//  347     
//  348     if(eep_page_break_f == 0)
        CMP0      N:`page_write_1::eep_page_break_f`  ;; 1 cycle
        BNZ       ??eprom_init_28    ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  349     {
//  350         return page_write(Add,Mem,len,opr_data);
        MOVW      DE, #LWRD(_opr_data)  ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        MOV       C, A               ;; 1 cycle
        MOVW      AX, [SP+0x02]      ;; 1 cycle
          CFI FunCall _page_write
        CALL      _page_write        ;; 3 cycles
        BR        S:??eprom_init_29  ;; 3 cycles
        ; ------------------------------------- Block: 12 cycles
//  351     }
//  352     else
//  353     {
//  354         higher_address = (us16)(((us32)Add+EEP_PAGE_SIZE)/EEP_PAGE_SIZE)*EEP_PAGE_SIZE;
??eprom_init_28:
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        ADDW      AX, #0x80          ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SHRW      AX, 0x7            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SHLW      AX, 0x9            ;; 1 cycle
        ADDW      AX, BC             ;; 1 cycle
        MOVW      BC, #0x80          ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      N:`page_write_1::higher_address`, AX  ;; 1 cycle
//  355         
//  356         if(page_write(Add,Mem,higher_address-Add,opr_data) == EEP_OK)
        MOVW      DE, #LWRD(_opr_data)  ;; 1 cycle
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      AX, N:`page_write_1::higher_address`  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        SUB       A, B               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        MOV       C, A               ;; 1 cycle
        MOVW      AX, [SP+0x02]      ;; 1 cycle
          CFI FunCall _page_write
        CALL      _page_write        ;; 3 cycles
        CMP       A, #0x1            ;; 1 cycle
        BNZ       ??eprom_init_30    ;; 4 cycles
        ; ------------------------------------- Block: 39 cycles
//  357         {
//  358             if(page_write(higher_address,Mem,Add+len-higher_address,&opr_data[higher_address-Add]) == EEP_OK)
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, N:`page_write_1::higher_address`  ;; 1 cycle
        SUBW      AX, HL             ;; 1 cycle
        ADDW      AX, #LWRD(_opr_data)  ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        ADD       A, X               ;; 1 cycle
        MOVW      HL, N:`page_write_1::higher_address`  ;; 1 cycle
        SUB       A, L               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        MOV       C, A               ;; 1 cycle
        MOVW      AX, N:`page_write_1::higher_address`  ;; 1 cycle
          CFI FunCall _page_write
        CALL      _page_write        ;; 3 cycles
        CMP       A, #0x1            ;; 1 cycle
        ONEB      A                  ;; 1 cycle
        BZ        ??eprom_init_29    ;; 4 cycles
        ; ------------------------------------- Block: 24 cycles
        CLRB      A                  ;; 1 cycle
//  359             {
//  360                 return EEP_OK;
        BR        S:??eprom_init_29  ;; 3 cycles
        ; ------------------------------------- Block: 4 cycles
//  361             }
//  362             else 
//  363             {
//  364                 return EEP_ERROR;
//  365             }
//  366         }
//  367         else 
//  368         {
//  369             return EEP_ERROR;
??eprom_init_30:
        MOV       A, #0x0            ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??eprom_init_29:
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock14
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 109 cycles
//  370         }
//  371     }
//  372 }

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
`page_write_1::eep_page_break_f`:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
`page_write_1::higher_address`:
        DS 2

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock15 Using cfiCommon2
          CFI Function _page_write
        CODE
//  373 EEP_ret_value page_write(us16 Add, us8 Mem, us8 len,us8 *opr_location)
//  374 {       
_page_write:
        ; * Stack frame (at entry) *
        ; Param size: 0
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+6
        PUSH      DE                 ;; 1 cycle
          CFI CFA SP+8
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+10
        ; Auto size: 12
        SUBW      SP, #0x6           ;; 1 cycle
          CFI CFA SP+16
//  375   us8 i,iic_DeviceAddress;
//  376   us8 *data,ret_val = 1;
        MOV       A, #0x1            ;; 1 cycle
        MOV       [SP+0x01], A       ;; 1 cycle
//  377   
//  378   if(len > MAX_OPR_BUFFER)
        MOV       A, [SP+0x07]       ;; 1 cycle
        CMP       A, #0x81           ;; 1 cycle
        BC        ??eprom_init_31    ;; 4 cycles
        ; ------------------------------------- Block: 12 cycles
//  379   {
//  380     len = MAX_OPR_BUFFER;
        MOV       A, #0x80           ;; 1 cycle
        MOV       [SP+0x07], A       ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
//  381   }
//  382   data = opr_location;
??eprom_init_31:
        MOVW      AX, [SP+0x08]      ;; 1 cycle
        MOVW      [SP+0x02], AX      ;; 1 cycle
//  383   iic_DeviceAddress = eep_add_write[Mem];
        MOV       A, [SP+0x06]       ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        MOV       A, (_eep_add_write)[B]  ;; 1 cycle
        MOV       [SP+0x04], A       ;; 1 cycle
//  384   
//  385   eprom_write_enable();
          CFI FunCall _eprom_write_enable
        CALL      _eprom_write_enable  ;; 3 cycles
//  386   
//  387   busbusy_f = 0;
        CLR1      N:_flag_eep1.1     ;; 2 cycles
//  388   nack_rxd_f = 0;
        CLR1      N:_flag_eep1.0     ;; 2 cycles
//  389   start_condition();		
          CFI FunCall _start_condition
        CALL      _start_condition   ;; 3 cycles
//  390   if(busbusy_f == 0)
        MOVW      HL, #LWRD(_flag_eep1)  ;; 1 cycle
        MOV1      CY, [HL].1         ;; 1 cycle
        BC        ??eprom_init_32    ;; 4 cycles
        ; ------------------------------------- Block: 22 cycles
//  391   {
//  392     while(1)
//  393     {
//  394       if(byte_write(iic_DeviceAddress) == NACK)     
        MOV       A, [SP+0x04]       ;; 1 cycle
          CFI FunCall _byte_write
        CALL      _byte_write        ;; 3 cycles
        CMP       A, #0x1            ;; 1 cycle
        BZ        ??eprom_init_32    ;; 4 cycles
        ; ------------------------------------- Block: 9 cycles
//  395       {
//  396         break;			          		
//  397       }
//  398       if(byte_write(highByte(Add)) == NACK)	          	
        MOVW      AX, [SP+0x0A]      ;; 1 cycle
        CLRB      X                  ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       A, X               ;; 1 cycle
          CFI FunCall _byte_write
        CALL      _byte_write        ;; 3 cycles
        CMP       A, #0x1            ;; 1 cycle
        BZ        ??eprom_init_32    ;; 4 cycles
        ; ------------------------------------- Block: 12 cycles
//  399       {
//  400         break;			          			
//  401       }
//  402       if(byte_write(lowByte(Add)) == NACK)	          
        MOVW      AX, [SP+0x0A]      ;; 1 cycle
        MOV       A, X               ;; 1 cycle
          CFI FunCall _byte_write
        CALL      _byte_write        ;; 3 cycles
        CMP       A, #0x1            ;; 1 cycle
        BZ        ??eprom_init_32    ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
//  403       {
//  404         break;			          		
//  405       }
//  406       for(i=0; i<len; i++)                               	
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
??page_write_0:
        MOV       A, [SP]            ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, [SP+0x07]       ;; 1 cycle
        CMP       X, A               ;; 1 cycle
        BNC       ??eprom_init_32    ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
//  407       {
//  408         if((byte_write(*data)) == NACK)	          		
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, [HL]            ;; 1 cycle
          CFI FunCall _byte_write
        CALL      _byte_write        ;; 3 cycles
        CMP       A, #0x1            ;; 1 cycle
        BZ        ??eprom_init_32    ;; 4 cycles
        ; ------------------------------------- Block: 11 cycles
//  409         {
//  410           break;		          				
//  411         }
//  412         data++;
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        INCW      AX                 ;; 1 cycle
        MOVW      [SP+0x02], AX      ;; 1 cycle
//  413       }
        MOV       A, [SP]            ;; 1 cycle
        INC       A                  ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        BR        S:??page_write_0   ;; 3 cycles
          CFI FunCall _stop_condition
        ; ------------------------------------- Block: 9 cycles
//  414       break;
//  415     }
//  416   }
//  417   
//  418   stop_condition();	
??eprom_init_32:
        CALL      _stop_condition    ;; 3 cycles
//  419   eprom_write_disable();
          CFI FunCall _eprom_write_disable
        CALL      _eprom_write_disable  ;; 3 cycles
//  420   
//  421   if(busbusy_f == 1 || nack_rxd_f == 1)
        MOV       A, N:_flag_eep1    ;; 1 cycle
        AND       A, #0x3            ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        BZ        ??eprom_init_33    ;; 4 cycles
        ; ------------------------------------- Block: 13 cycles
//  422   {
//  423     ret_val = 1;
        MOV       A, #0x1            ;; 1 cycle
        MOV       [SP+0x01], A       ;; 1 cycle
//  424     eprom_error_handler(Mem,1);
        MOV       X, #0x1            ;; 1 cycle
        MOV       A, [SP+0x06]       ;; 1 cycle
          CFI FunCall _eprom_error_handler
        CALL      _eprom_error_handler  ;; 3 cycles
        BR        S:??eprom_init_34  ;; 3 cycles
        ; ------------------------------------- Block: 10 cycles
//  425   }
//  426   else
//  427   {
//  428     ret_val = 0;
??eprom_init_33:
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP+0x01], A       ;; 1 cycle
//  429     eprom_error_handler(Mem,0);
        MOV       X, #0x0            ;; 1 cycle
        MOV       A, [SP+0x06]       ;; 1 cycle
          CFI FunCall _eprom_error_handler
        CALL      _eprom_error_handler  ;; 3 cycles
//  430     delay_ms(5);
        MOVW      AX, #0x5           ;; 1 cycle
          CFI FunCall _delay_ms
        CALL      _delay_ms          ;; 3 cycles
        ; ------------------------------------- Block: 11 cycles
//  431   }
//  432   
//  433   if(1 == ret_val)
??eprom_init_34:
        MOV       A, [SP+0x01]       ;; 1 cycle
        CMP       A, #0x1            ;; 1 cycle
        CLRB      A                  ;; 1 cycle
        SKZ                          ;; 1 cycle
        ; ------------------------------------- Block: 4 cycles
        ONEB      A                  ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
//  434   {
//  435     return EEP_ERROR; /* Think about calling reset_mem here */
??page_write_1:
        ADDW      SP, #0xC           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock15
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 143 cycles
//  436   }
//  437   else
//  438   {
//  439     return EEP_OK;
//  440   }
//  441 }
//  442 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock16 Using cfiCommon1
          CFI Function _page_read
        CODE
//  443 EEP_ret_value page_read(us16 Add, us8 Mem, us8 len)
//  444 {
_page_read:
        ; * Stack frame (at entry) *
        ; Param size: 0
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+6
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+8
        ; Auto size: 10
        SUBW      SP, #0x6           ;; 1 cycle
          CFI CFA SP+14
//  445   us8 *data, ret_val = 1;
        MOV       A, #0x1            ;; 1 cycle
        MOV       [SP+0x02], A       ;; 1 cycle
//  446   us8 i,iic_DeviceAddress=0;
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  447   
//  448   if(len > MAX_OPR_BUFFER)
        MOV       A, [SP+0x07]       ;; 1 cycle
        CMP       A, #0x81           ;; 1 cycle
        BC        ??eprom_init_35    ;; 4 cycles
        ; ------------------------------------- Block: 13 cycles
//  449   {
//  450     len = MAX_OPR_BUFFER;
        MOV       A, #0x80           ;; 1 cycle
        MOV       [SP+0x07], A       ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
//  451   }
//  452   data = opr_data;
??eprom_init_35:
        MOVW      AX, #LWRD(_opr_data)  ;; 1 cycle
        MOVW      [SP+0x04], AX      ;; 1 cycle
//  453   iic_DeviceAddress = eep_add_write[Mem];
        MOV       A, [SP+0x06]       ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        MOV       A, (_eep_add_write)[B]  ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  454   
//  455   busbusy_f = 0;
        CLR1      N:_flag_eep1.1     ;; 2 cycles
//  456   nack_rxd_f = 0;
        CLR1      N:_flag_eep1.0     ;; 2 cycles
//  457   start_condition();	
          CFI FunCall _start_condition
        CALL      _start_condition   ;; 3 cycles
//  458   if(busbusy_f == 0)
        MOVW      HL, #LWRD(_flag_eep1)  ;; 1 cycle
        MOV1      CY, [HL].1         ;; 1 cycle
        BC        ??eprom_init_36    ;; 4 cycles
        ; ------------------------------------- Block: 19 cycles
//  459   {
//  460     while(1)
//  461     {
//  462       if(byte_write(iic_DeviceAddress) == NACK)   		
        MOV       A, [SP]            ;; 1 cycle
          CFI FunCall _byte_write
        CALL      _byte_write        ;; 3 cycles
        CMP       A, #0x1            ;; 1 cycle
        BZ        ??eprom_init_36    ;; 4 cycles
        ; ------------------------------------- Block: 9 cycles
//  463       {
//  464         break;
//  465       }
//  466       if(byte_write(highByte(Add)) == NACK)	        		
        MOVW      AX, [SP+0x08]      ;; 1 cycle
        CLRB      X                  ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       A, X               ;; 1 cycle
          CFI FunCall _byte_write
        CALL      _byte_write        ;; 3 cycles
        CMP       A, #0x1            ;; 1 cycle
        BZ        ??eprom_init_36    ;; 4 cycles
        ; ------------------------------------- Block: 12 cycles
//  467       {
//  468         break;			        						
//  469       }           
//  470       if(byte_write(lowByte(Add)) == NACK)	        		
        MOVW      AX, [SP+0x08]      ;; 1 cycle
        MOV       A, X               ;; 1 cycle
          CFI FunCall _byte_write
        CALL      _byte_write        ;; 3 cycles
        CMP       A, #0x1            ;; 1 cycle
        BZ        ??eprom_init_36    ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
//  471       {
//  472         break;			        						
//  473       }            
//  474       iic_DeviceAddress |= 0x01;	        				                        // Read Address
        MOV       A, [SP]            ;; 1 cycle
        OR        A, #0x1            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  475       start_condition();		        				                        // Restart
          CFI FunCall _start_condition
        CALL      _start_condition   ;; 3 cycles
//  476       if(busbusy_f == 0)
        MOVW      HL, #LWRD(_flag_eep1)  ;; 1 cycle
        MOV1      CY, [HL].1         ;; 1 cycle
        BC        ??eprom_init_36    ;; 4 cycles
        ; ------------------------------------- Block: 12 cycles
//  477       {
//  478         if(byte_write(iic_DeviceAddress) == NACK)   		                                
        MOV       A, [SP]            ;; 1 cycle
          CFI FunCall _byte_write
        CALL      _byte_write        ;; 3 cycles
        CMP       A, #0x1            ;; 1 cycle
        BZ        ??eprom_init_36    ;; 4 cycles
        ; ------------------------------------- Block: 9 cycles
//  479         {
//  480           break;			        					
//  481         }
//  482         for (i=1; i<len; i++)
        MOV       A, #0x1            ;; 1 cycle
        MOV       [SP+0x01], A       ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
??page_read_0:
        MOV       A, [SP+0x01]       ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, [SP+0x07]       ;; 1 cycle
        CMP       X, A               ;; 1 cycle
        BNC       ??eprom_init_37    ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
//  483         {
//  484           byte_read(data, ACK);                          	                                        // Read data with ACK
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
          CFI FunCall _byte_read
        CALL      _byte_read         ;; 3 cycles
//  485           data++;	                        				                       
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        INCW      AX                 ;; 1 cycle
        MOVW      [SP+0x04], AX      ;; 1 cycle
//  486         }
        MOV       A, [SP+0x01]       ;; 1 cycle
        INC       A                  ;; 1 cycle
        MOV       [SP+0x01], A       ;; 1 cycle
        BR        S:??page_read_0    ;; 3 cycles
        ; ------------------------------------- Block: 14 cycles
//  487         byte_read(data, NACK);		        				                // Read data with NACK
??eprom_init_37:
        MOV       C, #0x1            ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
          CFI FunCall _byte_read
        CALL      _byte_read         ;; 3 cycles
//  488         break;                                        
          CFI FunCall _stop_condition
        ; ------------------------------------- Block: 5 cycles
//  489       } 
//  490       break;  
//  491     }
//  492   }
//  493   
//  494   stop_condition();							                        // Stop Condition
??eprom_init_36:
        CALL      _stop_condition    ;; 3 cycles
//  495   
//  496   if(busbusy_f == 1 || nack_rxd_f == 1)
        MOV       A, N:_flag_eep1    ;; 1 cycle
        AND       A, #0x3            ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        BZ        ??eprom_init_38    ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
//  497   {
//  498     ret_val = 1;
        MOV       A, #0x1            ;; 1 cycle
        MOV       [SP+0x02], A       ;; 1 cycle
//  499     eprom_error_handler(Mem,1);
        MOV       X, #0x1            ;; 1 cycle
        MOV       A, [SP+0x06]       ;; 1 cycle
          CFI FunCall _eprom_error_handler
        CALL      _eprom_error_handler  ;; 3 cycles
        BR        S:??eprom_init_39  ;; 3 cycles
        ; ------------------------------------- Block: 10 cycles
//  500   }
//  501   else
//  502   {
//  503     ret_val = 0;
??eprom_init_38:
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP+0x02], A       ;; 1 cycle
//  504     eprom_error_handler(Mem,0);
        MOV       X, #0x0            ;; 1 cycle
        MOV       A, [SP+0x06]       ;; 1 cycle
          CFI FunCall _eprom_error_handler
        CALL      _eprom_error_handler  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
//  505   }
//  506   if(1 == ret_val)
??eprom_init_39:
        MOV       A, [SP+0x02]       ;; 1 cycle
        CMP       A, #0x1            ;; 1 cycle
        CLRB      A                  ;; 1 cycle
        SKZ                          ;; 1 cycle
        ; ------------------------------------- Block: 4 cycles
        ONEB      A                  ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
//  507   {
//  508     return EEP_ERROR; /* Think about calling reset_mem here */
??page_read_1:
        ADDW      SP, #0xA           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock16
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 154 cycles
//  509   }
//  510   else
//  511   {
//  512     return EEP_OK;
//  513   }
//  514 }
//  515 /* This function provides universal interface to perform many type of memory write operations 
//  516 Parameter 1 : pass the address at which the memory write operation to be performed
//  517 Parameter 2 : the memory no where the data needs to be written
//  518 Parameter 3 : total no of bytes need to be written should be even multipe of 16
//  519 Parameter 4 : no of pages to be written in one go 
//  520 Parameter 5 : Whether checksum to be written at the last byte or not*/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock17 Using cfiCommon2
          CFI Function _eprom_write
        CODE
//  521 EEP_ret_value eprom_write(us16 address, us8 mem_id, us16 total_bytes, PAGE_SIZE page_size, EEP_checksum_mem_mode checksum_mem_mode)
//  522 {
_eprom_write:
        ; * Stack frame (at entry) *
        ; Param size: 2
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+6
        PUSH      DE                 ;; 1 cycle
          CFI CFA SP+8
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+10
        ; Auto size: 14
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+18
//  523     us32 temp_add;
//  524     us8 byte_in_page, ret_val = 1, retry_cnt = 0;
        MOV       A, #0x1            ;; 1 cycle
        MOV       [SP+0x02], A       ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  525     
//  526     /* Getting no of bytes to be written in one shot(Number of bytes as per page selected)*/
//  527     byte_in_page = nob_in_page[page_size];
        MOV       A, [SP+0x09]       ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        MOV       A, (_nob_in_page)[B]  ;; 1 cycle
        MOV       [SP+0x01], A       ;; 1 cycle
//  528     
//  529     /* checking if the no of bytes to be written should be more than the bytes in selected page size */
//  530     if(total_bytes >= byte_in_page)
        MOV       A, [SP+0x01]       ;; 1 cycle
        MOV       L, A               ;; 1 cycle
        MOV       H, #0x0            ;; 1 cycle
        MOVW      AX, [SP+0x0A]      ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        SKNC                         ;; 4 cycles
        BR        N:??eprom_init_40  ;; 4 cycles
        ; ------------------------------------- Block: 21 cycles
//  531     {
//  532         /* calculating the checksum of the bytes to be written */
//  533         if(checksum_mem_mode == AUTO_CALC)
        MOV       A, [SP+0x12]       ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        BNZ       ??eprom_init_41    ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  534         {
//  535             opr_data[byte_in_page-1] = checksum_mem(opr_data,(byte_in_page-1));
        MOV       A, [SP+0x01]       ;; 1 cycle
        MOV       C, A               ;; 1 cycle
        DEC       C                  ;; 1 cycle
        MOVW      AX, #LWRD(_opr_data)  ;; 1 cycle
          CFI FunCall _checksum_mem
        CALL      _checksum_mem      ;; 3 cycles
        MOV       X, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       (_opr_data-1)[B], A  ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        ; ------------------------------------- Block: 13 cycles
//  536         }
//  537         
//  538         /* writing bytes in the memory */
//  539         for(temp_add = address; temp_add < ((us32)address + (us32) total_bytes); temp_add += byte_in_page)
??eprom_init_41:
        MOVW      AX, [SP+0x0C]      ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ; ------------------------------------- Block: 10 cycles
??eprom_write_0:
        MOVW      AX, [SP+0x0A]      ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      HL, #0x0           ;; 1 cycle
        MOVW      AX, [SP+0x0C]      ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        ADDW      AX, DE             ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+20
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+22
        MOVW      AX, [SP+0x0A]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x08]      ;; 1 cycle
          CFI FunCall ?UL_CMP_L03
        CALL      N:?UL_CMP_L03      ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+18
        BNC       ??eprom_init_40    ;; 4 cycles
        ; ------------------------------------- Block: 27 cycles
//  540         {
//  541             /* Handing errors in writing the page */
//  542             for(retry_cnt = 0; retry_cnt < 3; retry_cnt++)
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
??eprom_write_1:
        MOV       A, [SP]            ;; 1 cycle
        CMP       A, #0x3            ;; 1 cycle
        BNC       ??eprom_init_42    ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  543             {
//  544                 if(page_write_1(temp_add,mem_id,byte_in_page) == EEP_OK)
        MOV       A, [SP+0x01]       ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        MOV       A, [SP+0x08]       ;; 1 cycle
        MOV       C, A               ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
          CFI FunCall _page_write_1
        CALL      _page_write_1      ;; 3 cycles
        CMP       A, #0x1            ;; 1 cycle
        BNZ       ??eprom_init_43    ;; 4 cycles
        ; ------------------------------------- Block: 13 cycles
//  545                 {
//  546                     ret_val = 0;
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP+0x02], A       ;; 1 cycle
//  547                     break;
        BR        S:??eprom_init_42  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  548                 }
//  549                 else
//  550                 {
//  551                     delay_ms(1);
??eprom_init_43:
        MOVW      AX, #0x1           ;; 1 cycle
          CFI FunCall _delay_ms
        CALL      _delay_ms          ;; 3 cycles
//  552                     retry_cnt_write++;                                    /* for debugging */
        INCW      N:_retry_cnt_write  ;; 2 cycles
//  553                 }
//  554             }
        MOV       A, [SP]            ;; 1 cycle
        INC       A                  ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        BR        S:??eprom_write_1  ;; 3 cycles
        ; ------------------------------------- Block: 12 cycles
//  555             if(retry_cnt == 3)
??eprom_init_42:
        MOV       A, [SP]            ;; 1 cycle
        CMP       A, #0x3            ;; 1 cycle
        BNZ       ??eprom_init_44    ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  556             {
//  557                 ret_val = 1;                                            /* unsuccessful write attempt */
        MOV       A, #0x1            ;; 1 cycle
        MOV       [SP+0x02], A       ;; 1 cycle
//  558                 break;                                                  /* verify if needed to continue or cancel successful write operation */       
        BR        S:??eprom_init_40  ;; 3 cycles
          CFI FunCall _R_WDT_Restart
        ; ------------------------------------- Block: 5 cycles
//  559             }
//  560             wdt_restart();
??eprom_init_44:
        CALL      _R_WDT_Restart     ;; 3 cycles
//  561         }
        MOV       A, [SP+0x01]       ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       D, #0x0            ;; 1 cycle
        MOVW      HL, #0x0           ;; 1 cycle
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        ADDW      AX, DE             ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        BR        S:??eprom_write_0  ;; 3 cycles
        ; ------------------------------------- Block: 30 cycles
//  562     }
//  563     if(1 == ret_val)
??eprom_init_40:
        MOV       A, [SP+0x02]       ;; 1 cycle
        CMP       A, #0x1            ;; 1 cycle
        CLRB      A                  ;; 1 cycle
        SKZ                          ;; 1 cycle
        ; ------------------------------------- Block: 4 cycles
        ONEB      A                  ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
//  564     {
//  565         return EEP_ERROR; /* Think about calling reset_mem here */
??eprom_write_2:
        ADDW      SP, #0xE           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock17
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 168 cycles
//  566     }
//  567     else
//  568     {
//  569         return EEP_OK;
//  570     }
//  571 }

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock18 Using cfiCommon2
          CFI Function _eprom_read
        CODE
//  572 EEP_ret_value eprom_read(us16 address, us8 mem_id, PAGE_SIZE page_size, EEP_checksum_mem_mode checksum_mem_mode)
//  573 {
_eprom_read:
        ; * Stack frame (at entry) *
        ; Param size: 0
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+6
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+8
        PUSH      DE                 ;; 1 cycle
          CFI CFA SP+10
        ; Auto size: 10
        SUBW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+14
//  574   us8 byte_in_page, ret_val = 1, retry_cnt = 0;
        MOV       A, #0x1            ;; 1 cycle
        MOV       [SP+0x01], A       ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  575   checksum_error_f = 0;
        CLR1      N:_flag_eep1.2     ;; 2 cycles
//  576   eep_read_ok = 0;
        CLR1      N:_flag_eep1.3     ;; 2 cycles
//  577   /* Getting no of bytes to be read in one shot */
//  578   byte_in_page = nob_in_page[page_size];
        MOV       A, [SP+0x07]       ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        MOV       A, (_nob_in_page)[B]  ;; 1 cycle
        MOV       [SP+0x02], A       ;; 1 cycle
//  579   fill_oprzero(byte_in_page);
        MOV       A, [SP+0x02]       ;; 1 cycle
          CFI FunCall _fill_oprzero
        CALL      _fill_oprzero      ;; 3 cycles
//  580   /* reading pages from the memory */
//  581   /* Handing error in reading the page */
//  582   for(retry_cnt = 0; retry_cnt < 3; retry_cnt++)
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        ; ------------------------------------- Block: 22 cycles
??eprom_read_0:
        MOV       A, [SP]            ;; 1 cycle
        CMP       A, #0x3            ;; 1 cycle
        BNC       ??eprom_init_45    ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  583   {
//  584     if(page_read(address,mem_id,byte_in_page) == EEP_OK)
        MOV       A, [SP+0x02]       ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        MOV       A, [SP+0x06]       ;; 1 cycle
        MOV       C, A               ;; 1 cycle
        MOVW      AX, [SP+0x08]      ;; 1 cycle
          CFI FunCall _page_read
        CALL      _page_read         ;; 3 cycles
        CMP       A, #0x1            ;; 1 cycle
        BNZ       ??eprom_init_46    ;; 4 cycles
        ; ------------------------------------- Block: 13 cycles
//  585     {
//  586       /* calculating the checksum_mem of the bytes to be written in one shot if required */
//  587       if(checksum_mem_mode == AUTO_CALC)
        MOV       A, [SP+0x04]       ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        BNZ       ??eprom_init_47    ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  588       {
//  589         if(opr_data[byte_in_page-1] == checksum_mem(opr_data,(byte_in_page-1)))
        MOV       A, [SP+0x02]       ;; 1 cycle
        MOV       C, A               ;; 1 cycle
        DEC       C                  ;; 1 cycle
        MOVW      AX, #LWRD(_opr_data)  ;; 1 cycle
          CFI FunCall _checksum_mem
        CALL      _checksum_mem      ;; 3 cycles
        MOV       X, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        MOV       A, (_opr_data-1)[B]  ;; 1 cycle
        CMP       A, X               ;; 1 cycle
        BNZ       ??eprom_init_48    ;; 4 cycles
        ; ------------------------------------- Block: 16 cycles
//  590         {
//  591           ret_val = 0;
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP+0x01], A       ;; 1 cycle
//  592           break;
        BR        S:??eprom_init_45  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  593         }
//  594         else
//  595         {
//  596           checksum_error_f = 1;
??eprom_init_48:
        SET1      N:_flag_eep1.2     ;; 2 cycles
//  597           ret_val = 1;
        MOV       A, #0x1            ;; 1 cycle
        MOV       [SP+0x01], A       ;; 1 cycle
        BR        S:??eprom_init_49  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
//  598         }
//  599       }
//  600       else
//  601       {
//  602         ret_val = 0;
??eprom_init_47:
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP+0x01], A       ;; 1 cycle
//  603         break;
        BR        S:??eprom_init_45  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  604       }
//  605     }
//  606     else
//  607     {
//  608       delay_ms(1);
??eprom_init_46:
        MOVW      AX, #0x1           ;; 1 cycle
          CFI FunCall _delay_ms
        CALL      _delay_ms          ;; 3 cycles
//  609       retry_cnt_read++;       /* for debugging */
        INCW      N:_retry_cnt_read  ;; 2 cycles
        ; ------------------------------------- Block: 6 cycles
//  610     }
//  611   }
??eprom_init_49:
        MOV       A, [SP]            ;; 1 cycle
        INC       A                  ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        BR        S:??eprom_read_0   ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
//  612   if(retry_cnt == 3)
??eprom_init_45:
        MOV       A, [SP]            ;; 1 cycle
        CMP       A, #0x3            ;; 1 cycle
        BNZ       ??eprom_init_50    ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  613   {
//  614     ret_val = 1;    /* unsuccessful read attempt */     
        MOV       A, #0x1            ;; 1 cycle
        MOV       [SP+0x01], A       ;; 1 cycle
          CFI FunCall _R_WDT_Restart
        ; ------------------------------------- Block: 2 cycles
//  615   }
//  616   wdt_restart();
??eprom_init_50:
        CALL      _R_WDT_Restart     ;; 3 cycles
//  617   if(1 == ret_val)
        MOV       A, [SP+0x01]       ;; 1 cycle
        CMP       A, #0x1            ;; 1 cycle
        BNZ       ??eprom_init_51    ;; 4 cycles
        ; ------------------------------------- Block: 9 cycles
//  618   {
//  619     eep_read_ok = 0;
        CLR1      N:_flag_eep1.3     ;; 2 cycles
//  620     return EEP_ERROR; /* Think about calling reset_mem here */
        MOV       A, #0x0            ;; 1 cycle
        BR        S:??eprom_init_52  ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
//  621   }
//  622   else
//  623   {
//  624     eep_read_ok = 1;
??eprom_init_51:
        SET1      N:_flag_eep1.3     ;; 2 cycles
//  625     return EEP_OK;
        MOV       A, #0x1            ;; 1 cycle
        ; ------------------------------------- Block: 3 cycles
??eprom_init_52:
        ADDW      SP, #0xA           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock18
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 125 cycles
//  626   }
//  627 }
//  628 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock19 Using cfiCommon0
          CFI Function _eprom_error_handler
          CFI NoCalls
        CODE
//  629 void eprom_error_handler(us8 Mem, us8 status)
//  630 {
_eprom_error_handler:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  631 //  if(busbusy_f == 1)
//  632 //  {
//  633 //     lcd_write_msg(325,1);
//  634 //     //     wdt_restart();
//  635 //     //delay_ms(1000);
//  636 //     //     eprom_magnet_vcc_disable();
//  637 //     //     eeprom_soft_reset_f = 1;
//  638 //  }
//  639   if(status == 1)
        XCH       A, X               ;; 1 cycle
        CMP       A, #0x1            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        BNZ       ??eprom_init_53    ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  640   {
//  641     error_cnt_eep++;
        INCW      N:_error_cnt_eep   ;; 2 cycles
//  642     if(error_cnt_eep >= 10)
        MOVW      HL, N:_error_cnt_eep  ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        CMPW      AX, #0xA           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        BC        ??eprom_init_54    ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
//  643     {
//  644       if(flag_eep_fail == 0)
        MOVW      HL, #LWRD(_flag_eeprom_error)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BC        ??eprom_init_54    ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  645       {
//  646         flag_eep_fail = 1;
        SET1      N:_flag_eeprom_error.0  ;; 2 cycles
//  647         flag_eep_fail_log = 1;
        SET1      N:_flag_eeprom_error.1  ;; 2 cycles
        BR        S:??eprom_init_54  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
//  648       }
//  649     }
//  650   }
//  651   else
//  652   {
//  653     if(flag_eep_fail_log == 0)
??eprom_init_53:
        MOVW      HL, #LWRD(_flag_eeprom_error)  ;; 1 cycle
        MOV1      CY, [HL].1         ;; 1 cycle
        BC        ??eprom_init_54    ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  654     {
//  655       error_cnt_eep = 0;
        MOVW      HL, #0x0           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      N:_error_cnt_eep, AX  ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
//  656       if(flag_eep_fail == 1)
        MOVW      HL, #LWRD(_flag_eeprom_error)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??eprom_init_54    ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
//  657       {
//  658         flag_eep_fail = 0;
        CLR1      N:_flag_eeprom_error.0  ;; 2 cycles
//  659         flag_eep_fail_log = 1;
        SET1      N:_flag_eeprom_error.1  ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
//  660       }
//  661     }
//  662   }
//  663   if(Mem == EEP0)
??eprom_init_54:
        CMP0      A                  ;; 1 cycle
        BNZ       ??eprom_init_55    ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  664   {
//  665     if(status == 1)
        XCH       A, X               ;; 1 cycle
        CMP       A, #0x1            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        BNZ       ??eprom_init_56    ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  666     {
//  667       error_cnt_eep0++;
        INCW      N:_error_cnt_eep0  ;; 2 cycles
//  668       if(error_cnt_eep0 >= 10)
        MOVW      HL, N:_error_cnt_eep0  ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        CMPW      AX, #0xA           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        SKNC                         ;; 4 cycles
        BR        N:??eprom_init_57  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
//  669       {
//  670         if(flag_eep0_fail == 0)
        MOVW      HL, #LWRD(_flag_eeprom_error)  ;; 1 cycle
        MOV1      CY, [HL].2         ;; 1 cycle
        SKNC                         ;; 4 cycles
        BR        N:??eprom_init_57  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  671         {
//  672           flag_eep0_fail = 1;
        SET1      N:_flag_eeprom_error.2  ;; 2 cycles
//  673           flag_eep0_fail_log = 1;
        SET1      N:_flag_eeprom_error.3  ;; 2 cycles
        RET                          ;; 6 cycles
        ; ------------------------------------- Block: 10 cycles
//  674         }
//  675       }
//  676     }
//  677     else
//  678     {
//  679       if(flag_eep0_fail_log == 0)
??eprom_init_56:
        MOVW      HL, #LWRD(_flag_eeprom_error)  ;; 1 cycle
        MOV1      CY, [HL].3         ;; 1 cycle
        SKNC                         ;; 4 cycles
        BR        N:??eprom_init_57  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  680       {
//  681         error_cnt_eep0 = 0;
        MOVW      HL, #0x0           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      N:_error_cnt_eep0, AX  ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
//  682         if(flag_eep0_fail == 1)
        MOVW      HL, #LWRD(_flag_eeprom_error)  ;; 1 cycle
        MOV1      CY, [HL].2         ;; 1 cycle
        SKC                          ;; 4 cycles
        BR        N:??eprom_init_57  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
//  683         {
//  684           flag_eep0_fail = 0;
        CLR1      N:_flag_eeprom_error.2  ;; 2 cycles
//  685           flag_eep0_fail_log = 1;
        SET1      N:_flag_eeprom_error.3  ;; 2 cycles
        RET                          ;; 6 cycles
        ; ------------------------------------- Block: 10 cycles
//  686         }
//  687       }
//  688     }
//  689   }
//  690   else if(Mem == EEP1)
??eprom_init_55:
        CMP       A, #0x1            ;; 1 cycle
        BNZ       ??eprom_init_58    ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  691   {
//  692     if(status == 1)
        XCH       A, X               ;; 1 cycle
        CMP       A, #0x1            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        BNZ       ??eprom_init_59    ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  693     {
//  694       error_cnt_eep1++;
        INCW      N:_error_cnt_eep1  ;; 2 cycles
//  695       if(error_cnt_eep1 >= 10)
        MOVW      HL, N:_error_cnt_eep1  ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        CMPW      AX, #0xA           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        BC        ??eprom_init_57    ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
//  696       {
//  697         if(flag_eep1_fail == 0)
        MOVW      HL, #LWRD(_flag_eeprom_error)  ;; 1 cycle
        MOV1      CY, [HL].4         ;; 1 cycle
        BC        ??eprom_init_57    ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  698         {
//  699           flag_eep1_fail = 1;
        SET1      N:_flag_eeprom_error.4  ;; 2 cycles
//  700           flag_eep1_fail_log = 1;
        SET1      N:_flag_eeprom_error.5  ;; 2 cycles
        RET                          ;; 6 cycles
        ; ------------------------------------- Block: 10 cycles
//  701         }
//  702       }
//  703     }
//  704     else
//  705     {
//  706       if(flag_eep1_fail_log == 0)
??eprom_init_59:
        MOVW      HL, #LWRD(_flag_eeprom_error)  ;; 1 cycle
        MOV1      CY, [HL].5         ;; 1 cycle
        BC        ??eprom_init_57    ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  707       {
//  708         error_cnt_eep1 = 0;
        MOVW      HL, #0x0           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      N:_error_cnt_eep1, AX  ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
//  709         if(flag_eep1_fail == 1)
        MOVW      HL, #LWRD(_flag_eeprom_error)  ;; 1 cycle
        MOV1      CY, [HL].4         ;; 1 cycle
        BNC       ??eprom_init_57    ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
//  710         {
//  711           flag_eep1_fail = 0;
        CLR1      N:_flag_eeprom_error.4  ;; 2 cycles
//  712           flag_eep1_fail_log = 1;
        SET1      N:_flag_eeprom_error.5  ;; 2 cycles
        RET                          ;; 6 cycles
        ; ------------------------------------- Block: 10 cycles
//  713         }
//  714       }
//  715     }
//  716   }
//  717   else if(Mem == EEP2)
??eprom_init_58:
        CMP       A, #0x2            ;; 1 cycle
        BNZ       ??eprom_init_57    ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  718   {
//  719     if(status == 1)
        XCH       A, X               ;; 1 cycle
        CMP       A, #0x1            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        BNZ       ??eprom_init_60    ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  720     {
//  721       error_cnt_eep2++;
        INCW      N:_error_cnt_eep2  ;; 2 cycles
//  722       if(error_cnt_eep2 >= 10)
        MOVW      HL, N:_error_cnt_eep2  ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        CMPW      AX, #0xA           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        BC        ??eprom_init_57    ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
//  723       {
//  724         if(flag_eep2_fail == 0)
        MOVW      HL, #LWRD(_flag_eeprom_error)  ;; 1 cycle
        MOV1      CY, [HL].6         ;; 1 cycle
        BC        ??eprom_init_57    ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  725         {
//  726           flag_eep2_fail = 1;
        SET1      N:_flag_eeprom_error.6  ;; 2 cycles
//  727           flag_eep2_fail_log = 1;
        SET1      N:_flag_eeprom_error.7  ;; 2 cycles
        RET                          ;; 6 cycles
        ; ------------------------------------- Block: 10 cycles
//  728         }
//  729       }
//  730     }
//  731     else
//  732     {
//  733       if(flag_eep2_fail_log == 0)
??eprom_init_60:
        MOVW      HL, #LWRD(_flag_eeprom_error)  ;; 1 cycle
        MOV1      CY, [HL].7         ;; 1 cycle
        BC        ??eprom_init_57    ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  734       {
//  735         error_cnt_eep2 = 0;
        MOVW      HL, #0x0           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      N:_error_cnt_eep2, AX  ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
//  736         if(flag_eep2_fail == 1)
        MOVW      HL, #LWRD(_flag_eeprom_error)  ;; 1 cycle
        MOV1      CY, [HL].6         ;; 1 cycle
        BNC       ??eprom_init_57    ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
//  737         {
//  738           flag_eep2_fail = 0;
        CLR1      N:_flag_eeprom_error.6  ;; 2 cycles
//  739           flag_eep2_fail_log = 1;
        SET1      N:_flag_eeprom_error.7  ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
//  740         }
//  741       }
//  742     }
//  743   }
//  744 }
??eprom_init_57:
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock19
        ; ------------------------------------- Block: 6 cycles
        ; ------------------------------------- Total: 242 cycles

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock20 Using cfiCommon0
          CFI Function _eprom_init
        CODE
//  745 void eprom_init()
//  746 {
_eprom_init:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  747   NOP();
        NOP                          ;; 1 cycle
//  748   reset_mem();
          CFI FunCall _reset_mem
        CALL      _reset_mem         ;; 3 cycles
//  749 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock20
        ; ------------------------------------- Block: 10 cycles
        ; ------------------------------------- Total: 10 cycles

        SECTION `.iar_vfe_header`:DATA:NOALLOC:NOROOT(1)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
        DC32 0

        END
// 
//   145 bytes in section .bss
//     4 bytes in section .bss.noinit   (abs)
//    12 bytes in section .data
//     2 bytes in section .sbss.noinit  (abs)
// 1'783 bytes in section .text
// 
// 1'783 bytes of CODE memory
//   157 bytes of DATA memory (+ 6 bytes shared)
//
//Errors: none
//Warnings: 2
