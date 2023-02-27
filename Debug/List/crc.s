///////////////////////////////////////////////////////////////////////////////
//
// IAR C/C++ Compiler V4.20.1.2260 for RL78               25/Dec/2020  21:42:24
// Copyright 2011-2019 IAR Systems AB.
// Evaluation license - IAR Embedded Workbench for Renesas RL78, Evaluation version 4.20
//
//    Core               =  s3
//    Calling convention =  v2
//    Code model         =  Near
//    Data model         =  Near
//                       =   
//    Source file        =
//        E:\0. Dheeraj\0. Official\1. Genus\2. Projects\0. GDEV72 -
//        BootCode\source_code\source_files\crc.c
//    Command line       =
//        -f C:\Users\DHEERA~1\AppData\Local\Temp\EWBC63.tmp ("E:\0. Dheeraj\0.
//        Official\1. Genus\2. Projects\0. GDEV72 -
//        BootCode\source_code\source_files\crc.c" --core s3 --code_model near
//        --calling_convention v2 --near_const_location ram -o "E:\0.
//        Dheeraj\0. Official\1. Genus\2. Projects\0. GDEV72 -
//        BootCode\Debug\Obj" --dlib_config "C:\Program Files (x86)\IAR
//        Systems\Embedded Workbench 8.4\rl78\LIB\DLib_Config_Normal.h"
//        --double=32 -e -On --no_cse --no_unroll --no_inline --no_code_motion
//        --no_tbaa --no_cross_call --no_scheduling --no_clustering --debug -lA
//        "E:\0. Dheeraj\0. Official\1. Genus\2. Projects\0. GDEV72 -
//        BootCode\Debug\List" -I "E:\0. Dheeraj\0. Official\1. Genus\2.
//        Projects\0. GDEV72 - BootCode\source_code\driver_files\" -I "E:\0.
//        Dheeraj\0. Official\1. Genus\2. Projects\0. GDEV72 -
//        BootCode\source_code\library_files\" -I "E:\0. Dheeraj\0. Official\1.
//        Genus\2. Projects\0. GDEV72 - BootCode\source_code\misc_files\" -I
//        "E:\0. Dheeraj\0. Official\1. Genus\2. Projects\0. GDEV72 -
//        BootCode\source_code\source_files\" --data_model near)
//    Locale             =  C
//    List file          =
//        E:\0. Dheeraj\0. Official\1. Genus\2. Projects\0. GDEV72 -
//        BootCode\Debug\List\crc.s
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

        PUBLIC _checksum_mem
        
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
          CFI EndCommon cfiCommon0
        
// E:\0. Dheeraj\0. Official\1. Genus\2. Projects\0. GDEV72 - BootCode\source_code\source_files\crc.c
//    1 /***********************************************************************************************************************
//    2 * File Name       : crc.c
//    3 * Current Version : rev_01  
//    4 * Tool-Chain      : IAR Systems RL78
//    5 * Description     : All the routines for calculation of various kind of CRC calculation/Verification
//    6 * Creation Date   : 13-07-2020
//    7 * Company         : Genus Power Infrastructures Limited, Jaipur
//    8 * Author          : dheeraj.singhal
//    9 * Version History : rev_01 :
//   10 ***********************************************************************************************************************/
//   11 /************************************ Includes **************************************/
//   12 #include "crc.h"
//   13 /************************************ Local Variables *****************************************/
//   14 
//   15 /************************************ Local Functions *******************************/
//   16 /************************************ Extern Functions ******************************/
//   17 us8 checksum_mem(us8 *chkdata, us8 nob);
//   18 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock0 Using cfiCommon0
          CFI Function _checksum_mem
          CFI NoCalls
        CODE
//   19 us8 checksum_mem(us8 *chkdata, us8 nob)
//   20 {
_checksum_mem:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
        MOVW      HL, AX             ;; 1 cycle
//   21   us8 i,chksum=0;
        MOV       A, #0x0            ;; 1 cycle
//   22   
//   23   for(i=0;i<nob;i++)
        MOV       X, #0x0            ;; 1 cycle
        ; ------------------------------------- Block: 3 cycles
??checksum_mem_0:
        XCH       A, X               ;; 1 cycle
        CMP       A, C               ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        BNC       ??checksum_mem_1   ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//   24   {
//   25     chksum += *chkdata++;
        ADD       A, [HL]            ;; 1 cycle
        INCW      HL                 ;; 1 cycle
//   26   }
        INC       X                  ;; 1 cycle
        BR        S:??checksum_mem_0  ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
//   27   chksum = chksum ^ 0xff;
??checksum_mem_1:
        XOR       A, #0xFF           ;; 1 cycle
//   28   chksum += 1;
        INC       A                  ;; 1 cycle
//   29   return(chksum);
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock0
        ; ------------------------------------- Block: 8 cycles
        ; ------------------------------------- Total: 24 cycles
//   30 }

        SECTION `.iar_vfe_header`:DATA:NOALLOC:NOROOT(1)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
        DC32 0

        END
// 
// 20 bytes in section .text
// 
// 20 bytes of CODE memory
//
//Errors: none
//Warnings: none
