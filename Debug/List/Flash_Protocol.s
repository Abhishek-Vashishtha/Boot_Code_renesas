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
//        BootCode\source_code\source_files\Flash_Protocol.c
//    Command line       =
//        -f C:\Users\17054\AppData\Local\Temp\EWD836.tmp ("D:\Software
//        code\Projects Software\Projects new\0. Non AMI\SUGAM Based\0. GDEV72
//        - BootCode-20210104T103109Z-001\0. GDEV72 -
//        BootCode\source_code\source_files\Flash_Protocol.c" --core s3
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
//        BootCode\Debug\List\Flash_Protocol.s
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
        EXTERN _FSL_Erase
        EXTERN _FSL_ForceReset
        EXTERN _FSL_IVerify
        EXTERN _FSL_Init
        EXTERN _FSL_InvertBootFlag
        EXTERN _FSL_Open
        EXTERN _FSL_PrepareExtFunctions
        EXTERN _FSL_PrepareFunctions
        EXTERN _FSL_Write
        EXTERN _backlight_operation

        PUBLIC _FSL_Analyse_Data_Pkt_f
        PUBLIC _FSL_RX_Data_Pkt_length
        PUBLIC _FSL_Reset_f
        PUBLIC _FSL_Serial_counter
        PUBLIC _FSL_Serial_rxcounter
        PUBLIC _FSL_TX_Data_Pkt_length
        PUBLIC _FSL_data_array
        PUBLIC _FSL_delay
        PUBLIC _Flash_write_flag
        PUBLIC _Reset_Add
        PUBLIC _Self_Programming_Init
        PUBLIC _Self_Programming_Protocol_Analyse_Pkt
        PUBLIC _Self_Programming_Protocol_Prepare_Pkt
        PUBLIC _Self_Programming_UART_Data_Recieve
        PUBLIC _Self_Programming_UART_Data_Transmit
        PUBLIC _Self_Programming_UART_Init
        PUBLIC _Self_Programming_Write
        PUBLIC _Self_Programming_main
        PUBLIC _Verify_ChkSum
        PUBLIC _WriteBlock1
        PUBLIC _Write_addr
        PUBLIC _Write_addr_temp
        PUBLIC __A_IF0
        PUBLIC __A_MK0
        PUBLIC __A_NFEN0
        PUBLIC __A_PER0
        PUBLIC __A_PR00
        PUBLIC __A_PR10
        PUBLIC __A_SCR10
        PUBLIC __A_SCR11
        PUBLIC __A_SDR10
        PUBLIC __A_SDR11
        PUBLIC __A_SIR11
        PUBLIC __A_SMR10
        PUBLIC __A_SMR11
        PUBLIC __A_SO1
        PUBLIC __A_SOE1
        PUBLIC __A_SOL1
        PUBLIC __A_SPS1
        PUBLIC __A_SS1
        PUBLIC __A_ST1
        PUBLIC __A_WDTE
        PUBLIC _flash_complete
        PUBLIC _increment
        
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
        
// D:\Software code\Projects Software\Projects new\0. Non AMI\SUGAM Based\0. GDEV72 - BootCode-20210104T103109Z-001\0. GDEV72 - BootCode\source_code\source_files\Flash_Protocol.c
//    1 /***********************************************************************************************************************
//    2 * File Name       : Flash_Protocol.c
//    3 * Current Version : rev_01  
//    4 * Tool-Chain      : IAR Systems RL78
//    5 * Description     : this file will have all the routines related to tou
//    6 * Creation Date   : 18-12-2020
//    7 * Company         : Genus Power Infrastructures Limited, Jaipur
//    8 * Author          : dheeraj.singhal
//    9 * Version History : rev_01 :
//   10 ***********************************************************************************************************************/
//   11 /************************************ Includes **************************************/
//   12 #include "Flash_Protocol.h"

        ASEGN `.bss.noinit`:DATA:NOROOT,0fff48H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_SDR10
// __no_init union <unnamed>#71 volatile __sfr __no_bit_access _A_SDR10
__A_SDR10:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0fff4aH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_SDR11
// __no_init union <unnamed>#74 volatile __sfr __no_bit_access _A_SDR11
__A_SDR11:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0fffabH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_WDTE
// __no_init union <unnamed>#110 volatile __sfr __no_bit_access _A_WDTE
__A_WDTE:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0fffe0H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_IF0
// __no_init union <unnamed>#152 volatile __sfr _A_IF0
__A_IF0:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0fffe4H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_MK0
// __no_init union <unnamed>#170 volatile __sfr _A_MK0
__A_MK0:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0fffe8H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_PR00
// __no_init union <unnamed>#188 volatile __sfr _A_PR00
__A_PR00:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0fffecH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_PR10
// __no_init union <unnamed>#206 volatile __sfr _A_PR10
__A_PR10:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0070H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_NFEN0
// __no_init union <unnamed>#250 volatile _A_NFEN0
__A_NFEN0:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f00f0H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_PER0
// __no_init union <unnamed>#275 volatile _A_PER0
__A_PER0:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f014aH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_SIR11
// __no_init union <unnamed>#365 volatile __no_bit_access _A_SIR11
__A_SIR11:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0150H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_SMR10
// __no_init union <unnamed>#374 volatile __no_bit_access _A_SMR10
__A_SMR10:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0152H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_SMR11
// __no_init union <unnamed>#375 volatile __no_bit_access _A_SMR11
__A_SMR11:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0158H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_SCR10
// __no_init union <unnamed>#378 volatile __no_bit_access _A_SCR10
__A_SCR10:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f015aH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_SCR11
// __no_init union <unnamed>#379 volatile __no_bit_access _A_SCR11
__A_SCR11:
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

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0166H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_SPS1
// __no_init union <unnamed>#391 volatile __no_bit_access _A_SPS1
__A_SPS1:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0168H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_SO1
// __no_init union <unnamed>#394 volatile __no_bit_access _A_SO1
__A_SO1:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f016aH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_SOE1
// __no_init union <unnamed>#395 volatile _A_SOE1
__A_SOE1:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0174H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_SOL1
// __no_init union <unnamed>#398 volatile __no_bit_access _A_SOL1
__A_SOL1:
        DS 2
//   13 /************************************ Local Variables *****************************************/
//   14 /************************************ Extern Variables *****************************************/
//   15 /************************************ Local Functions *******************************/
//   16 /************************************ Extern Functions ******************************/
//   17 
//   18 
//   19 #define FLASH_SIZE	256
//   20 
//   21 #define RJ_45          (0u)
//   22 #define OPTICAL_PORT   (1u)
//   23 #define UART_TYPE      OPTICAL_PORT
//   24 

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   25 us8 FSL_data_array[262];
_FSL_data_array:
        DS 262

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   26 us32 Write_addr;
_Write_addr:
        DS 4

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   27 us32 Write_addr_temp;
_Write_addr_temp:
        DS 4

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   28 us16 increment;
_increment:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   29 us16 Reset_Add ;
_Reset_Add:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   30 us16 FSL_RX_Data_Pkt_length;
_FSL_RX_Data_Pkt_length:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   31 us16 FSL_Serial_rxcounter;
_FSL_Serial_rxcounter:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   32 us16 WriteBlock1;
_WriteBlock1:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   33 us8 FSL_Serial_counter;
_FSL_Serial_counter:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   34 us8 FSL_TX_Data_Pkt_length;
_FSL_TX_Data_Pkt_length:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   35 us8 Flash_write_flag;
_Flash_write_flag:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   36 us8 FSL_Analyse_Data_Pkt_f;
_FSL_Analyse_Data_Pkt_f:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   37 us8 flash_complete;
_flash_complete:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   38 us8 FSL_Reset_f;
_FSL_Reset_f:
        DS 1
//   39 
//   40 
//   41 
//   42 //#pragma section text ram_text
//   43 
//   44 __near_func void Self_Programming_main(void);
//   45 __near_func void Self_Programming_Read(void);
//   46 __near_func void Self_Programming_Verify(void);
//   47 __near_func void Self_Programming_SwapCluster(void);
//   48 __near_func void Self_Programming_Reset(void);
//   49 __near_func void Self_Programming_UART_Init(void);
//   50 __near_func void Self_Programming_UART_Data_Recieve(void);
//   51 __near_func void Self_Programming_UART_Data_Transmit(void);
//   52 __near_func void Self_Programming_Protocol_Analyse_Pkt(void);
//   53 __near_func void Self_Programming_Protocol_Prepare_Pkt(unsigned char Pkt_ID);
//   54 __near_func void FSL_delay(unsigned int count);
//   55 __near_func unsigned char Self_Programming_Init(void);
//   56 __near_func unsigned char Self_Programming_Write(unsigned long int Write_data);
//   57 
//   58 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock0 Using cfiCommon0
          CFI Function _Self_Programming_UART_Init
          CFI NoCalls
        CODE
//   59 __near_func void Self_Programming_UART_Init(void)
//   60 {
_Self_Programming_UART_Init:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//   61 #if UART_TYPE == RJ_45
//   62   SAU0EN = 1U;    /* enables input clock supply */
//   63   NOP();
//   64   NOP();
//   65   NOP();
//   66   NOP();
//   67   if(clock_select == CLOCK_24MHZ)
//   68   {
//   69     SPS0 = _0040_SAU_CK01_fCLK_4 | _0004_SAU_CK00_fCLK_4;
//   70   }
//   71   else if(clock_select == CLOCK_12MHZ)
//   72   {
//   73     SPS0 = _0030_SAU_CK01_fCLK_3 | _0003_SAU_CK00_fCLK_3;
//   74   }
//   75   else if(clock_select == CLOCK_6MHZ)
//   76   {
//   77     SPS0 = _0020_SAU_CK01_fCLK_2 | _0002_SAU_CK00_fCLK_2;
//   78   }
//   79   else if(clock_select == CLOCK_1_5MHZ)
//   80   {
//   81     SPS0 = _0000_SAU_CK01_fCLK_0 | _0000_SAU_CK00_fCLK_0;
//   82   }
//   83 
//   84 
//   85   ST0 |= _0008_SAUm_CH3_STOP_TRG_ON | _0004_SAUm_CH2_STOP_TRG_ON;
//   86   STMK1 = 1U;     /* disable INTST1 interrupt */
//   87   STIF1 = 0U;     /* clear INTST1 interrupt flag */
//   88   SRMK1 = 1U;     /* disable INTSR1 interrupt */
//   89   SRIF1 = 0U;     /* clear INTSR1 interrupt flag */
//   90   SREMK1 = 1U;    /* disable INTSRE1 interrupt */
//   91   SREIF1 = 0U;    /* clear INTSRE1 interrupt flag */
//   92   /* Set INTSR1 low priority */
//   93   SRPR11 = 1U;
//   94   SRPR01 = 1U;
//   95   /* Set INTSRE1 low priority */
//   96   SREPR11 = 1U;
//   97   SREPR01 = 1U;
//   98   /* Set INTST1 low priority */
//   99   STPR11 = 1U;
//  100   STPR01 = 1U;
//  101   SMR02 = _0020_SMR02_DEFAULT_VALUE | _0000_SAU_CLOCK_SELECT_CK00 | _0000_SAU_CLOCK_MODE_CKS | 
//  102     _0002_SAU_MODE_UART | _0000_SAU_TRANSFER_END;
//  103   SCR02 = _0004_SCR02_DEFAULT_VALUE | _8000_SAU_TRANSMISSION | _0000_SAU_TIMING_1 | _0000_SAU_INTSRE_MASK | 
//  104     _0000_SAU_PARITY_NONE | _0080_SAU_LSB | _0010_SAU_STOP_1 | _0003_SAU_LENGTH_8;
//  105   SDR02 = _9A00_SAU0_CH2_BAUDRATE_DIVISOR;
//  106   NFEN0 |= _04_SAU_RXD1_FILTER_ON;
//  107   SIR03 = _0004_SAU_SIRMN_FECTMN | _0002_SAU_SIRMN_PECTMN | _0001_SAU_SIRMN_OVCTMN;
//  108   SMR03 = _0020_SMR03_DEFAULT_VALUE | _0000_SAU_CLOCK_SELECT_CK00 | _0000_SAU_CLOCK_MODE_CKS | 
//  109     _0100_SAU_TRIGGER_RXD | _0000_SAU_EDGE_FALL | _0002_SAU_MODE_UART | _0000_SAU_TRANSFER_END;
//  110   SCR03 = _0004_SCR03_DEFAULT_VALUE | _4000_SAU_RECEPTION | _0000_SAU_TIMING_1 | _0400_SAU_INTSRE_ENABLE | 
//  111     _0000_SAU_PARITY_NONE | _0080_SAU_LSB | _0010_SAU_STOP_1 | _0003_SAU_LENGTH_8;
//  112   SDR03 = _9A00_SAU0_CH3_BAUDRATE_DIVISOR;
//  113   SO0 |= _0004_SAUm_CH2_DATA_OUTPUT_1;
//  114   SOL0 &= (uint16_t)~_0004_SAUm_CHANNEL2_INVERTED;
//  115   SOE0 |= _0004_SAUm_CH2_OUTPUT_ENABLE;
//  116   
//  117   
//  118   SO0 |= _0004_SAUm_CH2_DATA_OUTPUT_1;
//  119   SOE0 |= _0004_SAUm_CH2_OUTPUT_ENABLE;
//  120   SS0 |= _0008_SAUm_CH3_START_TRG_ON | _0004_SAUm_CH2_START_TRG_ON;
//  121   STIF1 = 0U;     /* clear INTST1 interrupt flag */
//  122   SRIF1 = 0U;     /* clear INTSR1 interrupt flag */
//  123   SREIF1 = 0U;    /* clear INTSRE1 interrupt flag */
//  124   STMK1 = 0U;     /* enable INTST1 interrupt */
//  125   SRMK1 = 0U;     /* enable INTSR1 interrupt */
//  126   SREMK1 = 0U;    /* enable INTSRE1 interrupt */
//  127     /* pending, there is slight vairation in uart code. check it if dont work */	
//  128 #else
//  129   SAU1EN = 1U;    /* enables input clock supply */
        SET1      0xF00F0.3          ;; 2 cycles
//  130   NOP();
        NOP                          ;; 1 cycle
//  131   NOP();
        NOP                          ;; 1 cycle
//  132   NOP();
        NOP                          ;; 1 cycle
//  133   NOP();
        NOP                          ;; 1 cycle
//  134   if(clock_select == CLOCK_24MHZ)
        CMP       N:_clock_select, #0x1  ;; 1 cycle
        BNZ       ??FSL_delay_0      ;; 4 cycles
        ; ------------------------------------- Block: 11 cycles
//  135   {
//  136     SPS1 = _0040_SAU_CK01_fCLK_4 | _0004_SAU_CK00_fCLK_4;
        MOVW      AX, #0x44          ;; 1 cycle
        MOVW      0x166, AX          ;; 1 cycle
        BR        S:??FSL_delay_1    ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  137   }
//  138   else if(clock_select == CLOCK_12MHZ)
??FSL_delay_0:
        CMP       N:_clock_select, #0x2  ;; 1 cycle
        BNZ       ??FSL_delay_2      ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  139   {
//  140     SPS1 = _0030_SAU_CK01_fCLK_3 | _0003_SAU_CK00_fCLK_3;
        MOVW      AX, #0x33          ;; 1 cycle
        MOVW      0x166, AX          ;; 1 cycle
        BR        S:??FSL_delay_1    ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  141   }
//  142   else if(clock_select == CLOCK_6MHZ)
??FSL_delay_2:
        CMP       N:_clock_select, #0x3  ;; 1 cycle
        BNZ       ??FSL_delay_3      ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  143   {
//  144     SPS1 = _0020_SAU_CK01_fCLK_2 | _0002_SAU_CK00_fCLK_2;
        MOVW      AX, #0x22          ;; 1 cycle
        MOVW      0x166, AX          ;; 1 cycle
        BR        S:??FSL_delay_1    ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  145   }
//  146   else if(clock_select == CLOCK_1_5MHZ)
??FSL_delay_3:
        CMP       N:_clock_select, #0x4  ;; 1 cycle
        BNZ       ??FSL_delay_1      ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  147   {
//  148     SPS1 = _0000_SAU_CK01_fCLK_0 | _0000_SAU_CK00_fCLK_0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      0x166, AX          ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
//  149   }
//  150 
//  151 
//  152   ST1 |= _0002_SAUm_CH1_STOP_TRG_ON | _0001_SAUm_CH0_STOP_TRG_ON;
??FSL_delay_1:
        MOVW      AX, 0x164          ;; 1 cycle
        OR        A, #0x0            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        OR        A, #0x3            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      0x164, AX          ;; 1 cycle
//  153   STMK2 = 1U;     /* disable INTST2 interrupt */
        SET1      0xFFFE5.0          ;; 2 cycles
//  154   STIF2 = 0U;     /* clear INTST2 interrupt flag */
        CLR1      0xFFFE1.0          ;; 2 cycles
//  155   SRMK2 = 1U;     /* disable INTSR2 interrupt */
        SET1      0xFFFE5.1          ;; 2 cycles
//  156   SRIF2 = 0U;     /* clear INTSR2 interrupt flag */
        CLR1      0xFFFE1.1          ;; 2 cycles
//  157   SREMK2 = 1U;    /* disable INTSRE2 interrupt */
        SET1      0xFFFE5.2          ;; 2 cycles
//  158   SREIF2 = 0U;    /* clear INTSRE2 interrupt flag */
        CLR1      0xFFFE1.2          ;; 2 cycles
//  159   /* Set INTSR2 low priority */
//  160   SRPR12 = 1U;
        SET1      0xFFFED.1          ;; 2 cycles
//  161   SRPR02 = 1U;
        SET1      0xFFFE9.1          ;; 2 cycles
//  162   /* Set INTSRE2 low priority */
//  163   SREPR12 = 1U;
        SET1      0xFFFED.2          ;; 2 cycles
//  164   SREPR02 = 1U;
        SET1      0xFFFE9.2          ;; 2 cycles
//  165   /* Set INTST2 low priority */
//  166   STPR12 = 1U;
        SET1      0xFFFED.0          ;; 2 cycles
//  167   STPR02 = 1U;
        SET1      0xFFFE9.0          ;; 2 cycles
//  168   SMR10 = _0020_SMR10_DEFAULT_VALUE | _0000_SAU_CLOCK_SELECT_CK00 | _0000_SAU_CLOCK_MODE_CKS | 
//  169     _0002_SAU_MODE_UART | _0002_SAU_MODE_UART | _0000_SAU_TRANSFER_END;
        MOVW      AX, #0x22          ;; 1 cycle
        MOVW      0x150, AX          ;; 1 cycle
//  170   SCR10 = _0004_SCR10_DEFAULT_VALUE | _8000_SAU_TRANSMISSION | _0000_SAU_TIMING_1 | _0000_SAU_INTSRE_MASK | 
//  171     _0000_SAU_PARITY_NONE | _0080_SAU_LSB | _0010_SAU_STOP_1 | _0003_SAU_LENGTH_8;
        MOVW      AX, #0x8097        ;; 1 cycle
        MOVW      0x158, AX          ;; 1 cycle
//  172   SDR10 = _9A00_SAU1_CH0_BAUDRATE_DIVISOR;
        MOVW      0xFFF48, #0x9A00   ;; 1 cycle
//  173   NFEN0 |= _10_SAU_RXD2_FILTER_ON;
        SET1      0xF0070.4          ;; 2 cycles
//  174   SIR11 = _0004_SAU_SIRMN_FECTMN | _0002_SAU_SIRMN_PECTMN | _0001_SAU_SIRMN_OVCTMN;
        MOVW      AX, #0x7           ;; 1 cycle
        MOVW      0x14A, AX          ;; 1 cycle
//  175   SMR11 = _0020_SMR11_DEFAULT_VALUE | _0000_SAU_CLOCK_SELECT_CK00 | _0000_SAU_CLOCK_MODE_CKS | 
//  176     _0100_SAU_TRIGGER_RXD | _0000_SAU_EDGE_FALL | _0002_SAU_MODE_UART | _0000_SAU_TRANSFER_END;
        MOVW      AX, #0x122         ;; 1 cycle
        MOVW      0x152, AX          ;; 1 cycle
//  177   SCR11 = _0004_SCR11_DEFAULT_VALUE | _4000_SAU_RECEPTION | _0000_SAU_TIMING_1 | _0400_SAU_INTSRE_ENABLE | 
//  178     _0000_SAU_PARITY_NONE | _0080_SAU_LSB | _0010_SAU_STOP_1 | _0003_SAU_LENGTH_8;
        MOVW      AX, #0x4497        ;; 1 cycle
        MOVW      0x15A, AX          ;; 1 cycle
//  179   SDR11 = _9A00_SAU1_CH1_BAUDRATE_DIVISOR;
        MOVW      0xFFF4A, #0x9A00   ;; 1 cycle
//  180   SO1 |= _0001_SAUm_CH0_DATA_OUTPUT_1;
        MOVW      AX, 0x168          ;; 1 cycle
        OR        A, #0x0            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        OR        A, #0x1            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      0x168, AX          ;; 1 cycle
//  181   SOL1 &= (uint16_t)~_0001_SAUm_CHANNEL0_INVERTED;
        MOVW      AX, 0x174          ;; 1 cycle
        AND       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        AND       A, #0xFE           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      0x174, AX          ;; 1 cycle
//  182   SOE1 |= _0001_SAUm_CH0_OUTPUT_ENABLE;
        SET1      0xF016A.0          ;; 2 cycles
//  183   
//  184   
//  185   SO1 |= _0001_SAUm_CH0_DATA_OUTPUT_1;
        MOVW      AX, 0x168          ;; 1 cycle
        OR        A, #0x0            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        OR        A, #0x1            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      0x168, AX          ;; 1 cycle
//  186   SOE1 |= _0001_SAUm_CH0_OUTPUT_ENABLE;
        SET1      0xF016A.0          ;; 2 cycles
//  187   SS1 |= _0002_SAUm_CH1_START_TRG_ON | _0001_SAUm_CH0_START_TRG_ON;
        MOVW      AX, 0x162          ;; 1 cycle
        OR        A, #0x0            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        OR        A, #0x3            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      0x162, AX          ;; 1 cycle
//  188   STIF2 = 0U;     /* clear INTST2 interrupt flag */
        CLR1      0xFFFE1.0          ;; 2 cycles
//  189   SRIF2 = 0U;     /* clear INTSR2 interrupt flag */
        CLR1      0xFFFE1.1          ;; 2 cycles
//  190   SREIF2 = 0U;    /* clear INTSRE2 interrupt flag */
        CLR1      0xFFFE1.2          ;; 2 cycles
//  191   STMK2 = 0U;     /* enable INTST2 interrupt */
        CLR1      0xFFFE5.0          ;; 2 cycles
//  192   SRMK2 = 0U;     /* enable INTSR2 interrupt */
        CLR1      0xFFFE5.1          ;; 2 cycles
//  193   SREMK2 = 0U;    /* enable INTSRE2 interrupt */  
        CLR1      0xFFFE5.2          ;; 2 cycles
//  194 #endif
//  195 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock0
        ; ------------------------------------- Block: 90 cycles
        ; ------------------------------------- Total: 133 cycles
        REQUIRE __A_PER0
        REQUIRE __A_SPS1
        REQUIRE __A_ST1
        REQUIRE __A_MK0
        REQUIRE __A_IF0
        REQUIRE __A_PR10
        REQUIRE __A_PR00
        REQUIRE __A_SMR10
        REQUIRE __A_SCR10
        REQUIRE __A_SDR10
        REQUIRE __A_NFEN0
        REQUIRE __A_SIR11
        REQUIRE __A_SMR11
        REQUIRE __A_SCR11
        REQUIRE __A_SDR11
        REQUIRE __A_SO1
        REQUIRE __A_SOL1
        REQUIRE __A_SOE1
        REQUIRE __A_SS1
//  196 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock1 Using cfiCommon0
          CFI Function _Self_Programming_main
        CODE
//  197 __near_func void Self_Programming_main(void)
//  198 {
_Self_Programming_main:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 2
        SUBW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+6
//  199     unsigned char FSL_Status_byte=0;
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  200     
//  201     /* Disable Interrupt Routines */
//  202     __disable_interrupt();
        DI                           ;; 4 cycles
//  203     
//  204     /* Initialize UART Channel for Communication */
//  205     Self_Programming_UART_Init();
          CFI FunCall _Self_Programming_UART_Init
        CALL      _Self_Programming_UART_Init  ;; 3 cycles
//  206     #if UART_TYPE == RJ_45
//  207     SendRJ45(0x06);
//  208     #else
//  209     SendOptical(0x06);
        MOV       0xFFF48, #0x6      ;; 1 cycle
//  210     #endif
//  211     /* Intialize Self Programming */
//  212     FSL_Status_byte = Self_Programming_Init();
          CFI FunCall _Self_Programming_Init
        CALL      _Self_Programming_Init  ;; 3 cycles
        MOV       [SP], A            ;; 1 cycle
//  213     
//  214     if(FSL_Status_byte != 0)
        MOV       A, [SP]            ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        BNZ       ??FSL_delay_4      ;; 4 cycles
        ; ------------------------------------- Block: 21 cycles
//  215     {
//  216         return ;
//  217     }
//  218     
//  219     while(1)
//  220     {
//  221         WDTE = 0xACU;
??Self_Programming_main_0:
        MOV       0xFFFAB, #0xAC     ;; 1 cycle
//  222         backlight_operation();
          CFI FunCall _backlight_operation
        CALL      _backlight_operation  ;; 3 cycles
//  223         /* Check for Data Reception */
//  224         Self_Programming_UART_Data_Recieve();
          CFI FunCall _Self_Programming_UART_Data_Recieve
        CALL      _Self_Programming_UART_Data_Recieve  ;; 3 cycles
//  225         
//  226         if(FSL_Analyse_Data_Pkt_f==1)
        CMP       N:_FSL_Analyse_Data_Pkt_f, #0x1  ;; 1 cycle
        BNZ       ??Self_Programming_main_0  ;; 4 cycles
          CFI FunCall _Self_Programming_Protocol_Analyse_Pkt
        ; ------------------------------------- Block: 12 cycles
//  227         {
//  228             Self_Programming_Protocol_Analyse_Pkt();
        CALL      _Self_Programming_Protocol_Analyse_Pkt  ;; 3 cycles
        BR        S:??Self_Programming_main_0  ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
//  229         }
//  230     }
??FSL_delay_4:
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock1
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 46 cycles
        REQUIRE __A_SDR10
        REQUIRE __A_WDTE
//  231 }
//  232 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock2 Using cfiCommon0
          CFI Function _Self_Programming_Init
        CODE
//  233 __near_func unsigned char Self_Programming_Init(void)
//  234 {
_Self_Programming_Init:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 4
        SUBW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
//  235     unsigned char Init_Status=1; 
        MOV       A, #0x1            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  236     
//  237     fsl_descriptor_t InitArg;
//  238     //fsl_fsw_t ShieldWindow;
//  239     
//  240     /* Set argument of FSL_Init()	 */
//  241     InitArg.fsl_flash_voltage_u08     = 0x00; //FULL_SPEED_MODE;
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP+0x01], A       ;; 1 cycle
//  242     InitArg.fsl_frequency_u08         = 0x03;  //FREQUENCY_32M;
        MOV       A, #0x3            ;; 1 cycle
        MOV       [SP+0x02], A       ;; 1 cycle
//  243     InitArg.fsl_auto_status_check_u08 = 0x01; //INTERNAL_MODE;
        MOV       A, #0x1            ;; 1 cycle
        MOV       [SP+0x03], A       ;; 1 cycle
//  244     
//  245     /* Initialize Flash Library Functions **/
//  246     Init_Status = FSL_Init(&InitArg);  
        MOVW      DE, SP             ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        ADDW      AX, #0x1           ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        MOV       A, #0xF            ;; 1 cycle
          CFI FunCall _FSL_Init
        CALL      F:_FSL_Init        ;; 3 cycles
        MOV       [SP], A            ;; 1 cycle
//  247     
//  248     if(Init_Status == FSL_OK)
        MOV       A, [SP]            ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        BNZ       ??FSL_delay_5      ;; 4 cycles
          CFI FunCall _FSL_Open
        ; ------------------------------------- Block: 24 cycles
//  249     {  
//  250         FSL_Open();                                                   
        CALL      F:_FSL_Open        ;; 3 cycles
//  251         FSL_PrepareFunctions();                                              
          CFI FunCall _FSL_PrepareFunctions
        CALL      F:_FSL_PrepareFunctions  ;; 3 cycles
//  252         FSL_PrepareExtFunctions();  
          CFI FunCall _FSL_PrepareExtFunctions
        CALL      F:_FSL_PrepareExtFunctions  ;; 3 cycles
        ; ------------------------------------- Block: 9 cycles
//  253     }
//  254     
//  255     return Init_Status;
??FSL_delay_5:
        MOV       A, [SP]            ;; 1 cycle
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock2
        ; ------------------------------------- Block: 8 cycles
        ; ------------------------------------- Total: 41 cycles
//  256 }
//  257 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock3 Using cfiCommon0
          CFI Function _Self_Programming_UART_Data_Recieve
        CODE
//  258 __near_func void Self_Programming_UART_Data_Recieve(void)
//  259 {
_Self_Programming_UART_Data_Recieve:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 4
        SUBW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
//  260     unsigned long int count = 0;
        MOVW      AX, SP             ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
//  261 #if UART_TYPE == RJ_45    
//  262     while(SRIF1==0)
//  263 #else
//  264     while(SRIF2==0)  
??Self_Programming_UART_Data_Recieve_0:
        MOVW      HL, #0xFFE1        ;; 1 cycle
        MOV1      CY, [HL].1         ;; 1 cycle
        BC        ??FSL_delay_6      ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  265 #endif
//  266     {
//  267         count++;
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        ADDW      AX, #0x1           ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      HL, SP             ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  268         if(count > 10000000)
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x98          ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 24 cycles
        CMPW      AX, #0x9681        ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??Self_Programming_UART_Data_Recieve_1:
        BC        ??FSL_delay_7      ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
//  269         {
//  270             FSL_Serial_rxcounter = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_FSL_Serial_rxcounter, AX  ;; 1 cycle
//  271             count = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        ; ------------------------------------- Block: 4 cycles
//  272             //SendOptical(0x05);
//  273             while(1);
??Self_Programming_UART_Data_Recieve_2:
        BR        S:??Self_Programming_UART_Data_Recieve_2  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
//  274         }
//  275         else
//  276         {
//  277             WDTE = 0xACU;
??FSL_delay_7:
        MOV       0xFFFAB, #0xAC     ;; 1 cycle
//  278         }
//  279         if(count % 100000 == 0)
        MOVW      DE, #0x86A0        ;; 1 cycle
        MOVW      HL, #0x1           ;; 1 cycle
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        DIVWU                        ;; 17 cycles
        NOP                          ;; 1 cycle
        MOV       A, D               ;; 1 cycle
        OR        A, E               ;; 1 cycle
        OR        A, L               ;; 1 cycle
        OR        A, H               ;; 1 cycle
        BNZ       ??Self_Programming_UART_Data_Recieve_0  ;; 4 cycles
          CFI FunCall _backlight_operation
        ; ------------------------------------- Block: 32 cycles
//  280         {
//  281             backlight_operation();
        CALL      _backlight_operation  ;; 3 cycles
        BR        S:??Self_Programming_UART_Data_Recieve_0  ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
//  282         }
//  283     }
//  284 #if UART_TYPE == RJ_45
//  285     SRIF1=0;
//  286     SREIF1=0;
//  287 #else
//  288     SRIF2=0;
??FSL_delay_6:
        CLR1      0xFFFE1.1          ;; 2 cycles
//  289     SREIF2=0;
        CLR1      0xFFFE1.2          ;; 2 cycles
//  290 #endif
//  291     count = 0;
        MOVW      DE, #0x0           ;; 1 cycle
        MOVW      HL, #0x0           ;; 1 cycle
//  292 #if UART_TYPE == RJ_45
//  293     FSL_data_array[FSL_Serial_rxcounter] =RXD1;
//  294 #else
//  295     FSL_data_array[FSL_Serial_rxcounter] =RXD2;
        MOVW      BC, N:_FSL_Serial_rxcounter  ;; 1 cycle
        MOV       A, 0xFFF4A         ;; 1 cycle
        MOV       (_FSL_data_array)[BC], A  ;; 1 cycle
//  296 #endif
//  297     FSL_Serial_rxcounter++ ;
        INCW      N:_FSL_Serial_rxcounter  ;; 2 cycles
//  298     
//  299     if(FSL_data_array[0]==0x01)
        CMP       N:_FSL_data_array, #0x1  ;; 1 cycle
        BNZ       ??FSL_delay_8      ;; 4 cycles
        ; ------------------------------------- Block: 16 cycles
//  300     {
//  301         FSL_RX_Data_Pkt_length=4;
        MOVW      AX, #0x4           ;; 1 cycle
        MOVW      N:_FSL_RX_Data_Pkt_length, AX  ;; 1 cycle
//  302         
//  303         if(FSL_data_array[3]==0x05)
        CMP       N:_FSL_data_array+3, #0x5  ;; 1 cycle
        BNZ       ??FSL_delay_9      ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  304         {
//  305             FSL_RX_Data_Pkt_length=11;
        MOVW      AX, #0xB           ;; 1 cycle
        MOVW      N:_FSL_RX_Data_Pkt_length, AX  ;; 1 cycle
        BR        S:??FSL_delay_10   ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  306         }
//  307         else if((FSL_data_array[3]==0x0b) || (FSL_data_array[3]==0x08) || (FSL_data_array[3]==0x07))
??FSL_delay_9:
        CMP       N:_FSL_data_array+3, #0xB  ;; 1 cycle
        BZ        ??FSL_delay_11     ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        CMP       N:_FSL_data_array+3, #0x8  ;; 1 cycle
        BZ        ??FSL_delay_11     ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        CMP       N:_FSL_data_array+3, #0x7  ;; 1 cycle
        BNZ       ??FSL_delay_12     ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  308         {
//  309             FSL_RX_Data_Pkt_length=6;
??FSL_delay_11:
        MOVW      AX, #0x6           ;; 1 cycle
        MOVW      N:_FSL_RX_Data_Pkt_length, AX  ;; 1 cycle
        BR        S:??FSL_delay_10   ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  310         }
//  311         else if(FSL_data_array[3]==0x06)
??FSL_delay_12:
        CMP       N:_FSL_data_array+3, #0x6  ;; 1 cycle
        BNZ       ??FSL_delay_10     ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  312         {
//  313             FSL_RX_Data_Pkt_length=261;
        MOVW      AX, #0x105         ;; 1 cycle
        MOVW      N:_FSL_RX_Data_Pkt_length, AX  ;; 1 cycle
        BR        S:??FSL_delay_10   ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  314         }
//  315     }
//  316     else
//  317     {
//  318         FSL_RX_Data_Pkt_length = 0;
??FSL_delay_8:
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_FSL_RX_Data_Pkt_length, AX  ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
//  319     }
//  320     
//  321     if(FSL_Serial_rxcounter>=FSL_RX_Data_Pkt_length)
??FSL_delay_10:
        MOVW      BC, N:_FSL_RX_Data_Pkt_length  ;; 1 cycle
        MOVW      AX, N:_FSL_Serial_rxcounter  ;; 1 cycle
        CMPW      AX, BC             ;; 1 cycle
        BC        ??FSL_delay_13     ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  322     {	 						
//  323         FSL_Serial_rxcounter=0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_FSL_Serial_rxcounter, AX  ;; 1 cycle
//  324         
//  325         if((FSL_RX_Data_Pkt_length==6) || (FSL_RX_Data_Pkt_length==11)||(FSL_RX_Data_Pkt_length==261)) // || (FSL_RX_Data_Pkt_length==133) || (FSL_RX_Data_Pkt_length==16))
        MOVW      AX, N:_FSL_RX_Data_Pkt_length  ;; 1 cycle
        CMPW      AX, #0x6           ;; 1 cycle
        BZ        ??FSL_delay_14     ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
        MOVW      AX, N:_FSL_RX_Data_Pkt_length  ;; 1 cycle
        CMPW      AX, #0xB           ;; 1 cycle
        BZ        ??FSL_delay_14     ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, N:_FSL_RX_Data_Pkt_length  ;; 1 cycle
        CMPW      AX, #0x105         ;; 1 cycle
        BNZ       ??FSL_delay_15     ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  326         {
//  327             FSL_Analyse_Data_Pkt_f=1;
??FSL_delay_14:
        MOV       N:_FSL_Analyse_Data_Pkt_f, #0x1  ;; 1 cycle
        BR        S:??FSL_delay_13   ;; 3 cycles
        ; ------------------------------------- Block: 4 cycles
//  328             //FSL_RX_Data_Pkt_length = 0 ;
//  329         }
//  330         else
//  331         {
//  332             FSL_RX_Data_Pkt_length=0;  
??FSL_delay_15:
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_FSL_RX_Data_Pkt_length, AX  ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
//  333         }
//  334     }
//  335 }
??FSL_delay_13:
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock3
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 187 cycles
        REQUIRE __A_WDTE
        REQUIRE __A_IF0
        REQUIRE __A_SDR11
//  336 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock4 Using cfiCommon1
          CFI Function _Verify_ChkSum
          CFI NoCalls
        CODE
//  337 __near_func unsigned char Verify_ChkSum(unsigned char *chkdata,unsigned int nob)
//  338 {
_Verify_ChkSum:
        ; * Stack frame (at entry) *
        ; Param size: 0
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+6
        ; Auto size: 4
        SUBW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+8
        MOVW      HL, AX             ;; 1 cycle
//  339     unsigned int i,chksum=0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [SP], AX           ;; 1 cycle
//  340     
//  341     for(i=0;i<nob;i++)
        MOVW      DE, #0x0           ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
??Verify_ChkSum_0:
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        CMPW      AX, DE             ;; 1 cycle
        BNH       ??FSL_delay_16     ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  342     {
//  343         chksum += *chkdata++;
        MOV       A, [HL]            ;; 1 cycle
        MOV       C, A               ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        ADDW      AX, BC             ;; 1 cycle
        MOVW      [SP], AX           ;; 1 cycle
        INCW      HL                 ;; 1 cycle
//  344     }
        INCW      DE                 ;; 1 cycle
        BR        S:??Verify_ChkSum_0  ;; 3 cycles
        ; ------------------------------------- Block: 11 cycles
//  345     
//  346     Write_addr_temp = nob;
??FSL_delay_16:
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        MOVW      N:_Write_addr_temp, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_Write_addr_temp+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  347     return(chksum);
        MOVW      AX, [SP]           ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock4
        ; ------------------------------------- Block: 15 cycles
        ; ------------------------------------- Total: 38 cycles
//  348 }
//  349 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock5 Using cfiCommon0
          CFI Function _Self_Programming_Protocol_Analyse_Pkt
        CODE
//  350 __near_func void Self_Programming_Protocol_Analyse_Pkt(void)
//  351 {
_Self_Programming_Protocol_Analyse_Pkt:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 2
        SUBW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+6
//  352     us8 Command_key=0;
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  353     
//  354     FSL_Analyse_Data_Pkt_f = 0 ;
        MOV       N:_FSL_Analyse_Data_Pkt_f, #0x0  ;; 1 cycle
//  355     
//  356     if(FSL_data_array[3] == 0x06)
        CMP       N:_FSL_data_array+3, #0x6  ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
//  357     {
//  358         NOP();
        NOP                          ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
//  359     }
//  360     
//  361     if(Verify_ChkSum(&FSL_data_array[0],FSL_RX_Data_Pkt_length-1) != FSL_data_array[FSL_RX_Data_Pkt_length-1])
??Self_Programming_Protocol_Analyse_Pkt_0:
        MOVW      BC, N:_FSL_RX_Data_Pkt_length  ;; 1 cycle
        DECW      BC                 ;; 1 cycle
        MOVW      AX, #LWRD(_FSL_data_array)  ;; 1 cycle
          CFI FunCall _Verify_ChkSum
        CALL      _Verify_ChkSum     ;; 3 cycles
        MOV       X, A               ;; 1 cycle
        MOVW      BC, N:_FSL_RX_Data_Pkt_length  ;; 1 cycle
        MOV       A, (_FSL_data_array-1)[BC]  ;; 1 cycle
        CMP       X, A               ;; 1 cycle
        BZ        ??FSL_delay_17     ;; 4 cycles
        ; ------------------------------------- Block: 14 cycles
//  362     {
//  363         FSL_RX_Data_Pkt_length = 0 ;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_FSL_RX_Data_Pkt_length, AX  ;; 1 cycle
//  364         return;
        BR        S:??FSL_delay_18   ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  365     }
//  366     Command_key = FSL_data_array[3];
??FSL_delay_17:
        MOV       A, N:_FSL_data_array+3  ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  367     
//  368     Self_Programming_Protocol_Prepare_Pkt(Command_key);
        MOV       A, [SP]            ;; 1 cycle
          CFI FunCall _Self_Programming_Protocol_Prepare_Pkt
        CALL      _Self_Programming_Protocol_Prepare_Pkt  ;; 3 cycles
//  369 }
        ; ------------------------------------- Block: 6 cycles
??FSL_delay_18:
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock5
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 39 cycles
//  370 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock6 Using cfiCommon0
          CFI Function _Self_Programming_Protocol_Prepare_Pkt
        CODE
//  371 __near_func void Self_Programming_Protocol_Prepare_Pkt(unsigned char data_ID)
//  372 {
_Self_Programming_Protocol_Prepare_Pkt:
        ; * Stack frame (at entry) *
        ; Param size: 0
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+6
        ; Auto size: 4
        SUBW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+8
//  373     unsigned int Satus_check = 0 ; 
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [SP], AX           ;; 1 cycle
//  374     
//  375     FSL_Reset_f = 0;
        MOV       N:_FSL_Reset_f, #0x0  ;; 1 cycle
//  376     
//  377     FSL_data_array[0] = 0x01;
        MOV       N:_FSL_data_array, #0x1  ;; 1 cycle
//  378     FSL_data_array[1] = 0x00;
        MOV       N:_FSL_data_array+1, #0x0  ;; 1 cycle
//  379     FSL_data_array[2] = 0x03;
        MOV       N:_FSL_data_array+2, #0x3  ;; 1 cycle
//  380     
//  381     switch(data_ID)
        MOV       A, [SP+0x03]       ;; 1 cycle
        SUB       A, #0x5            ;; 1 cycle
        BZ        ??FSL_delay_19     ;; 4 cycles
        ; ------------------------------------- Block: 14 cycles
        DEC       A                  ;; 1 cycle
        BZ        ??FSL_delay_20     ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        DEC       A                  ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??FSL_delay_21   ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        DEC       A                  ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??FSL_delay_22   ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        SUB       A, #0x3            ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??FSL_delay_23   ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        BR        N:??FSL_delay_24   ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
//  382     {
//  383     case 0x05:
//  384         WriteBlock1 = FSL_data_array[4]; 
??FSL_delay_19:
        MOV       X, N:_FSL_data_array+4  ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      N:_WriteBlock1, AX  ;; 1 cycle
//  385         //Write_addr = (Write_addr<<8) + FSL_data_array[7];
//  386         //WriteBlock1 = Write_addr;	///0x400 ;
//  387         //    lcd_clear_var();
//  388         //    lcd_map[6] |= LCD_7b;
//  389         //    lcd_map[7] |= LCD_7O;
//  390         //    lcd_map[8] |= LCD_7O;
//  391         //    lcd_map[9] |= LCD_7t;
//  392         //    lcd_map[5] |= 0x7A; // reverse_8bits(LCD_7b);
//  393         //    lcd_map[11] |= lcd_7digit[WriteBlock1/0x10];
//  394         //    lcd_map[12] |= lcd_7digit[WriteBlock1%0x10];
//  395         //    lcd_write();
//  396         
//  397         if(WriteBlock1 < FLASH_SIZE)
        MOVW      AX, N:_WriteBlock1  ;; 1 cycle
        CMPW      AX, #0x100         ;; 1 cycle
        BNC       ??FSL_delay_25     ;; 4 cycles
        ; ------------------------------------- Block: 9 cycles
//  398         {
//  399             Satus_check = FSL_Erase(WriteBlock1); 
        MOVW      AX, N:_WriteBlock1  ;; 1 cycle
          CFI FunCall _FSL_Erase
        CALL      F:_FSL_Erase       ;; 3 cycles
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      [SP], AX           ;; 1 cycle
//  400             if(Satus_check == FSL_OK)
        MOVW      AX, [SP]           ;; 1 cycle
        CMPW      AX, #0x0           ;; 1 cycle
        BNZ       ??FSL_delay_25     ;; 4 cycles
        ; ------------------------------------- Block: 13 cycles
//  401             {
//  402                 Flash_write_flag =1;
        MOV       N:_Flash_write_flag, #0x1  ;; 1 cycle
//  403                 increment = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_increment, AX   ;; 1 cycle
        ; ------------------------------------- Block: 3 cycles
//  404             }
//  405         }
//  406         
//  407         FSL_data_array[3]=data_ID ;
??FSL_delay_25:
        MOV       A, [SP+0x03]       ;; 1 cycle
        MOV       N:_FSL_data_array+3, A  ;; 1 cycle
//  408         FSL_data_array[4]=Satus_check;
        MOVW      AX, [SP]           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       N:_FSL_data_array+4, A  ;; 1 cycle
        XCH       A, X               ;; 1 cycle
//  409         FSL_data_array[5]=0x09;
        MOV       N:_FSL_data_array+5, #0x9  ;; 1 cycle
//  410         FSL_TX_Data_Pkt_length=6;
        MOV       N:_FSL_TX_Data_Pkt_length, #0x6  ;; 1 cycle
//  411         break;
        BR        N:??FSL_delay_26   ;; 3 cycles
        ; ------------------------------------- Block: 11 cycles
//  412         
//  413     case 0x06:
//  414         if(WriteBlock1 < FLASH_SIZE)
??FSL_delay_20:
        MOVW      AX, N:_WriteBlock1  ;; 1 cycle
        CMPW      AX, #0x100         ;; 1 cycle
        BNC       ??FSL_delay_27     ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  415         {
//  416             Write_addr = (us32)WriteBlock1*0x0400;
        MOVW      AX, N:_WriteBlock1  ;; 1 cycle
        MOVW      BC, #0x400         ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      N:_Write_addr, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_Write_addr+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  417             Satus_check = Self_Programming_Write(Write_addr);
        MOVW      BC, N:_Write_addr+2  ;; 1 cycle
        MOVW      AX, N:_Write_addr  ;; 1 cycle
          CFI FunCall _Self_Programming_Write
        CALL      _Self_Programming_Write  ;; 3 cycles
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      [SP], AX           ;; 1 cycle
        ; ------------------------------------- Block: 16 cycles
//  418         }
//  419         FSL_delay(100);
??FSL_delay_27:
        MOVW      AX, #0x64          ;; 1 cycle
          CFI FunCall _FSL_delay
        CALL      _FSL_delay         ;; 3 cycles
//  420         FSL_data_array[3]=data_ID ;
        MOV       A, [SP+0x03]       ;; 1 cycle
        MOV       N:_FSL_data_array+3, A  ;; 1 cycle
//  421         FSL_data_array[4]=Satus_check;
        MOVW      AX, [SP]           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       N:_FSL_data_array+4, A  ;; 1 cycle
        XCH       A, X               ;; 1 cycle
//  422         if(Satus_check	== 0x05)
        MOVW      AX, [SP]           ;; 1 cycle
        CMPW      AX, #0x5           ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 13 cycles
//  423         {
//  424             NOP();
        NOP                          ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
//  425         }
//  426         FSL_data_array[5]=0x0a;
??Self_Programming_Protocol_Prepare_Pkt_0:
        MOV       N:_FSL_data_array+5, #0xA  ;; 1 cycle
//  427         FSL_TX_Data_Pkt_length=6;
        MOV       N:_FSL_TX_Data_Pkt_length, #0x6  ;; 1 cycle
//  428         break;
        BR        S:??FSL_delay_26   ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  429         
//  430     case 0x07: 
//  431         FSL_Reset_f = 1; //FSL_ForceReset();
??FSL_delay_21:
        MOV       N:_FSL_Reset_f, #0x1  ;; 1 cycle
//  432         FSL_data_array[3]=data_ID ;
        MOV       A, [SP+0x03]       ;; 1 cycle
        MOV       N:_FSL_data_array+3, A  ;; 1 cycle
//  433         FSL_data_array[4]=0x00;
        MOV       N:_FSL_data_array+4, #0x0  ;; 1 cycle
//  434         FSL_data_array[5]=0x0b;
        MOV       N:_FSL_data_array+5, #0xB  ;; 1 cycle
//  435         FSL_TX_Data_Pkt_length=6;
        MOV       N:_FSL_TX_Data_Pkt_length, #0x6  ;; 1 cycle
//  436         break;
        BR        S:??FSL_delay_26   ;; 3 cycles
          CFI FunCall _FSL_InvertBootFlag
        ; ------------------------------------- Block: 9 cycles
//  437         
//  438     case 0x08: 
//  439         Satus_check = FSL_InvertBootFlag(); 
??FSL_delay_22:
        CALL      F:_FSL_InvertBootFlag  ;; 3 cycles
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      [SP], AX           ;; 1 cycle
//  440         FSL_delay(200);
        MOVW      AX, #0xC8          ;; 1 cycle
          CFI FunCall _FSL_delay
        CALL      _FSL_delay         ;; 3 cycles
//  441         FSL_Reset_f=1;
        MOV       N:_FSL_Reset_f, #0x1  ;; 1 cycle
//  442         FSL_data_array[3]=data_ID ;
        MOV       A, [SP+0x03]       ;; 1 cycle
        MOV       N:_FSL_data_array+3, A  ;; 1 cycle
//  443         FSL_data_array[4]=Satus_check;
        MOVW      AX, [SP]           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       N:_FSL_data_array+4, A  ;; 1 cycle
        XCH       A, X               ;; 1 cycle
//  444         FSL_data_array[5]=0x0c;
        MOV       N:_FSL_data_array+5, #0xC  ;; 1 cycle
//  445         FSL_TX_Data_Pkt_length=6;
        MOV       N:_FSL_TX_Data_Pkt_length, #0x6  ;; 1 cycle
//  446         break;
        BR        S:??FSL_delay_26   ;; 3 cycles
        ; ------------------------------------- Block: 22 cycles
//  447         
//  448     case 0x0b: 
//  449         Flash_write_flag = 0;
??FSL_delay_23:
        MOV       N:_Flash_write_flag, #0x0  ;; 1 cycle
//  450         //WriteBlock1 = Write_addr; ///0x400;
//  451         Satus_check = FSL_IVerify(WriteBlock1);
        MOVW      AX, N:_WriteBlock1  ;; 1 cycle
          CFI FunCall _FSL_IVerify
        CALL      F:_FSL_IVerify     ;; 3 cycles
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      [SP], AX           ;; 1 cycle
//  452         FSL_data_array[3]=data_ID ;
        MOV       A, [SP+0x03]       ;; 1 cycle
        MOV       N:_FSL_data_array+3, A  ;; 1 cycle
//  453         FSL_data_array[4]=Satus_check;
        MOVW      AX, [SP]           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       N:_FSL_data_array+4, A  ;; 1 cycle
        XCH       A, X               ;; 1 cycle
//  454         FSL_data_array[5]=0x0f;
        MOV       N:_FSL_data_array+5, #0xF  ;; 1 cycle
//  455         FSL_TX_Data_Pkt_length=6;
        MOV       N:_FSL_TX_Data_Pkt_length, #0x6  ;; 1 cycle
//  456         break;
        BR        S:??FSL_delay_26   ;; 3 cycles
        ; ------------------------------------- Block: 19 cycles
//  457     default:  
//  458         NOP();
??FSL_delay_24:
        NOP                          ;; 1 cycle
//  459         break;
        ; ------------------------------------- Block: 1 cycles
//  460     }
//  461     
//  462     FSL_Serial_counter=0;
??FSL_delay_26:
        MOV       N:_FSL_Serial_counter, #0x0  ;; 1 cycle
//  463     Self_Programming_UART_Data_Transmit();
          CFI FunCall _Self_Programming_UART_Data_Transmit
        CALL      _Self_Programming_UART_Data_Transmit  ;; 3 cycles
//  464     
//  465     if(FSL_Reset_f==1)
        CMP       N:_FSL_Reset_f, #0x1  ;; 1 cycle
        BNZ       ??FSL_delay_28     ;; 4 cycles
        ; ------------------------------------- Block: 9 cycles
//  466     {
//  467         FSL_Reset_f = 0;
        MOV       N:_FSL_Reset_f, #0x0  ;; 1 cycle
//  468         FSL_delay(200);
        MOVW      AX, #0xC8          ;; 1 cycle
          CFI FunCall _FSL_delay
        CALL      _FSL_delay         ;; 3 cycles
//  469         FSL_ForceReset();
          CFI FunCall _FSL_ForceReset
        CALL      F:_FSL_ForceReset  ;; 3 cycles
        ; ------------------------------------- Block: 8 cycles
//  470     }
//  471 }
??FSL_delay_28:
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock6
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 189 cycles
//  472 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock7 Using cfiCommon0
          CFI Function _Self_Programming_UART_Data_Transmit
          CFI NoCalls
        CODE
//  473 __near_func void Self_Programming_UART_Data_Transmit(void)
//  474 {	
_Self_Programming_UART_Data_Transmit:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 4
        SUBW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
//  475     unsigned long int count = 0;
        MOVW      AX, SP             ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
//  476     while(FSL_Serial_counter<FSL_TX_Data_Pkt_length)
??Self_Programming_UART_Data_Transmit_0:
        MOV       A, N:_FSL_Serial_counter  ;; 1 cycle
        CMP       A, N:_FSL_TX_Data_Pkt_length  ;; 1 cycle
        BNC       ??FSL_delay_29     ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  477     {
//  478 #if UART_TYPE == RJ_45
//  479       TXD1 = FSL_data_array[FSL_Serial_counter];
//  480 #else
//  481       TXD2 = FSL_data_array[FSL_Serial_counter];
        MOV       B, N:_FSL_Serial_counter  ;; 1 cycle
        MOV       A, (_FSL_data_array)[B]  ;; 1 cycle
        MOV       0xFFF48, A         ;; 1 cycle
        ; ------------------------------------- Block: 3 cycles
//  482 #endif
//  483 #if UART_TYPE == RJ_45
//  484       while(STIF1 == 0) 
//  485 #else 
//  486       while(STIF2 == 0)
??Self_Programming_UART_Data_Transmit_1:
        MOVW      HL, #0xFFE1        ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BC        ??FSL_delay_30     ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  487 #endif
//  488         {
//  489             WDTE = 0xACU;
        MOV       0xFFFAB, #0xAC     ;; 1 cycle
//  490             count++;
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        ADDW      AX, #0x1           ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      HL, SP             ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  491             if(count > 1000000)
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0xF           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 25 cycles
        CMPW      AX, #0x4241        ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??Self_Programming_UART_Data_Transmit_2:
        BC        ??Self_Programming_UART_Data_Transmit_1  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
//  492             {
//  493                 FSL_Serial_rxcounter = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_FSL_Serial_rxcounter, AX  ;; 1 cycle
//  494                 count = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        ; ------------------------------------- Block: 4 cycles
//  495                 while(1);
??Self_Programming_UART_Data_Transmit_3:
        BR        S:??Self_Programming_UART_Data_Transmit_3  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
//  496             }
//  497         }
//  498 #if UART_TYPE == RJ_45
//  499         STIF1=0;
//  500 #else
//  501         STIF2=0;
??FSL_delay_30:
        CLR1      0xFFFE1.0          ;; 2 cycles
//  502 #endif
//  503         FSL_Serial_counter++;
        INC       N:_FSL_Serial_counter  ;; 2 cycles
        BR        S:??Self_Programming_UART_Data_Transmit_0  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
//  504     }
//  505     
//  506     FSL_Serial_counter=0; 
??FSL_delay_29:
        MOV       N:_FSL_Serial_counter, #0x0  ;; 1 cycle
//  507 }
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock7
        ; ------------------------------------- Block: 8 cycles
        ; ------------------------------------- Total: 74 cycles
        REQUIRE __A_SDR10
        REQUIRE __A_WDTE
        REQUIRE __A_IF0
//  508 
//  509 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock8 Using cfiCommon1
          CFI Function _Self_Programming_Write
        CODE
//  510 __near_func unsigned char Self_Programming_Write(unsigned long int Write_data)
//  511 {
_Self_Programming_Write:
        ; * Stack frame (at entry) *
        ; Param size: 0
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+6
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+8
        ; Auto size: 14
        SUBW      SP, #0xA           ;; 1 cycle
          CFI CFA SP+18
//  512     unsigned char status=0, count;
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  513     fsl_write_t WriteArg;
//  514     
//  515     if(Flash_write_flag == 1)
        CMP       N:_Flash_write_flag, #0x1  ;; 1 cycle
        BNZ       ??FSL_delay_31     ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
//  516     {
//  517         count = 0x40;
        MOV       A, #0x40           ;; 1 cycle
        MOV       [SP+0x01], A       ;; 1 cycle
//  518         Write_addr_temp = Write_addr + increment;
        MOVW      AX, N:_increment   ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        MOVW      HL, N:_Write_addr+2  ;; 1 cycle
        MOVW      DE, N:_Write_addr  ;; 1 cycle
        ADDW      AX, DE             ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_Write_addr_temp, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_Write_addr_temp+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  519         /* ---- Set argument of FSL_Write() ---- */
//  520         WriteArg.fsl_data_buffer_p_u08       = &FSL_data_array[4];
        MOVW      AX, #LWRD(_FSL_data_array+4)  ;; 1 cycle
        MOVW      [SP+0x02], AX      ;; 1 cycle
//  521         WriteArg.fsl_destination_address_u32 = Write_addr + increment;
        MOVW      AX, N:_increment   ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        MOVW      HL, N:_Write_addr+2  ;; 1 cycle
        MOVW      DE, N:_Write_addr  ;; 1 cycle
        ADDW      AX, DE             ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  522         WriteArg.fsl_word_count_u08          = count;
        MOV       A, [SP+0x01]       ;; 1 cycle
        MOV       [SP+0x08], A       ;; 1 cycle
//  523         
//  524         status = FSL_Write(&WriteArg);    /* Write execution */
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x2           ;; 1 cycle
          CFI FunCall _FSL_Write
        CALL      F:_FSL_Write       ;; 3 cycles
        MOV       [SP], A            ;; 1 cycle
//  525         
//  526         if(status == FSL_OK)
        MOV       A, [SP]            ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        BNZ       ??FSL_delay_32     ;; 4 cycles
        ; ------------------------------------- Block: 55 cycles
//  527         {
//  528             increment= increment + 0x100 ;
        MOVW      AX, N:_increment   ;; 1 cycle
        ADDW      AX, #0x100         ;; 1 cycle
        MOVW      N:_increment, AX   ;; 1 cycle
//  529             if(increment>0x100)
        MOVW      AX, N:_increment   ;; 1 cycle
        CMPW      AX, #0x101         ;; 1 cycle
        BC        ??FSL_delay_32     ;; 4 cycles
        ; ------------------------------------- Block: 9 cycles
//  530             {
//  531                 NOP();
        NOP                          ;; 1 cycle
        BR        S:??FSL_delay_32   ;; 3 cycles
        ; ------------------------------------- Block: 4 cycles
//  532             }
//  533         }
//  534     }
//  535     else
//  536     {
//  537         NOP();
??FSL_delay_31:
        NOP                          ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
//  538     }
//  539     
//  540     return status;
??FSL_delay_32:
        MOV       A, [SP]            ;; 1 cycle
        ADDW      SP, #0xE           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock8
        ; ------------------------------------- Block: 8 cycles
        ; ------------------------------------- Total: 87 cycles
//  541 }
//  542 
//  543 
//  544 /**** 1 for 1 ms ****/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock9 Using cfiCommon0
          CFI Function _FSL_delay
          CFI NoCalls
        CODE
//  545 __near_func void FSL_delay(unsigned int count)
//  546 {
_FSL_delay:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  547     unsigned int i,k;
//  548     
//  549     for(i=0; i<count; i++)
        MOVW      DE, #0x0           ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??FSL_delay_33:
        CMPW      AX, DE             ;; 1 cycle
        BNH       ??FSL_delay_34     ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  550     {
//  551         for(k=0;k<510;k++)
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+6
        POP       HL                 ;; 1 cycle
          CFI CFA SP+4
        ; ------------------------------------- Block: 3 cycles
??FSL_delay_35:
        XCHW      AX, HL             ;; 1 cycle
        CMPW      AX, #0x1FE         ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        BNC       ??FSL_delay_36     ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  552         {
//  553             NOP();
        NOP                          ;; 1 cycle
//  554         }
        INCW      HL                 ;; 1 cycle
        BR        S:??FSL_delay_35   ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  555     }
??FSL_delay_36:
        INCW      DE                 ;; 1 cycle
        BR        S:??FSL_delay_33   ;; 3 cycles
        ; ------------------------------------- Block: 4 cycles
//  556 }
??FSL_delay_34:
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock9
        ; ------------------------------------- Block: 6 cycles
        ; ------------------------------------- Total: 31 cycles

        SECTION `.iar_vfe_header`:DATA:NOALLOC:NOROOT(1)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
        DC32 0

        END
//  557 
//  558 
// 
//   286 bytes in section .bss
//    37 bytes in section .bss.noinit  (abs)
// 1'290 bytes in section .text
// 
// 1'290 bytes of CODE memory
//   286 bytes of DATA memory (+ 37 bytes shared)
//
//Errors: none
//Warnings: none
