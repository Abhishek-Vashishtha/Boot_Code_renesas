///////////////////////////////////////////////////////////////////////////////
//
// IAR C/C++ Compiler V4.20.1.2260 for RL78               20/Dec/2020  22:38:34
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
//        BootCode\source_code\source_files\memory_log.c
//    Command line       =
//        -f C:\Users\laptop\AppData\Local\Temp\EW98D8.tmp ("D:\Dheeraj\New
//        folder\0. GDEV72 - BootCode\source_code\source_files\memory_log.c"
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
//        D:\Dheeraj\New folder\0. GDEV72 - BootCode\Debug\List\memory_log.s
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

        EXTERN _COMP1_TPR_EVENT
        EXTERN _COMP2_TPR_EVENT
        EXTERN _COMP3_TPR_EVENT
        EXTERN _opr_data
        EXTERN _tpr
        EXTERN ?MOVE_LONG_L06
        EXTERN _char_array_to_int
        EXTERN _eprom_read
        EXTERN _eprom_write
        EXTERN _fill_oprzero
        EXTERN _int_into_char_array
        EXTERN _read_tamper_variables
        EXTERN _tamper_instant_status
        EXTERN _time_into_char_array5_sec
        EXTERN _update_tamper_variables

        PUBLIC _COMPART_CURRENT_END_ADD
        PUBLIC _COMPART_CURRENT_ENTRIES
        PUBLIC _COMPART_CURRENT_SIZE
        PUBLIC _COMPART_CURRENT_START_ADD
        PUBLIC _COMPART_NONROLLOVER_END_ADD
        PUBLIC _COMPART_NONROLLOVER_ENTRIES
        PUBLIC _COMPART_NONROLLOVER_SIZE
        PUBLIC _COMPART_NONROLLOVER_START_ADD
        PUBLIC _COMPART_OTHERS_END_ADD
        PUBLIC _COMPART_OTHERS_ENTRIES
        PUBLIC _COMPART_OTHERS_SIZE
        PUBLIC _COMPART_OTHERS_START_ADD
        PUBLIC _COMPART_POWERFAIL_END_ADD
        PUBLIC _COMPART_POWERFAIL_ENTRIES
        PUBLIC _COMPART_POWERFAIL_SIZE
        PUBLIC _COMPART_POWERFAIL_START_ADD
        PUBLIC _COMPART_TRANSACTION_END_ADD
        PUBLIC _COMPART_TRANSACTION_ENTRIES
        PUBLIC _COMPART_TRANSACTION_SIZE
        PUBLIC _COMPART_TRANSACTION_START_ADD
        PUBLIC _COMPART_VOLTAGE_END_ADD
        PUBLIC _COMPART_VOLTAGE_ENTRIES
        PUBLIC _COMPART_VOLTAGE_SIZE
        PUBLIC _COMPART_VOLTAGE_START_ADD
        PUBLIC _memory_log_init
        PUBLIC _memory_log_reset
        PUBLIC _memory_log_reset_non_roll
        PUBLIC _seq_no_curr
        PUBLIC _seq_no_debug
        PUBLIC _seq_no_nonrollover
        PUBLIC _seq_no_others
        PUBLIC _seq_no_powerfail
        PUBLIC _seq_no_transaction
        PUBLIC _seq_no_vol
        
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
        
// D:\Dheeraj\New folder\0. GDEV72 - BootCode\source_code\source_files\memory_log.c
//    1 /***********************************************************************************************************************
//    2 * File Name       : memory_log.c
//    3 * Current Version : rev_01  
//    4 * Tool-Chain      : IAR Systems
//    5 * Description     : This file includes routines to enable and disable Watchdog timer
//    6 * Creation Date   : 01-04-2020
//    7 * Company         : Genus Power Infrastructures Limited, Jaipur
//    8 * Author          : dheeraj.singhal
//    9 * Version History : rev_01 : new source file created with routine to take log into memory for events and profiles 
//   10 ***********************************************************************************************************************/
//   11 /************************************ Includes **************************************/
//   12 #include "memory_log.h"
//   13 
//   14 /************************************ Local Variables *****************************************/
//   15 /************************************ Extern Variables *****************************************/

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   16 us16 seq_no_vol,seq_no_curr,seq_no_powerfail,seq_no_transaction,seq_no_others,seq_no_nonrollover,seq_no_debug;
_seq_no_vol:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_seq_no_curr:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_seq_no_powerfail:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_seq_no_transaction:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_seq_no_others:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_seq_no_nonrollover:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_seq_no_debug:
        DS 2
//   17 

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   18 us16 COMPART_VOLTAGE_START_ADD,COMPART_CURRENT_START_ADD,COMPART_POWERFAIL_START_ADD,COMPART_TRANSACTION_START_ADD,COMPART_OTHERS_START_ADD,COMPART_NONROLLOVER_START_ADD;
_COMPART_VOLTAGE_START_ADD:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_COMPART_CURRENT_START_ADD:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_COMPART_POWERFAIL_START_ADD:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_COMPART_TRANSACTION_START_ADD:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_COMPART_OTHERS_START_ADD:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_COMPART_NONROLLOVER_START_ADD:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   19 us8 COMPART_VOLTAGE_ENTRIES,COMPART_CURRENT_ENTRIES,COMPART_POWERFAIL_ENTRIES,COMPART_TRANSACTION_ENTRIES,COMPART_OTHERS_ENTRIES,COMPART_NONROLLOVER_ENTRIES;
_COMPART_VOLTAGE_ENTRIES:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_COMPART_CURRENT_ENTRIES:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_COMPART_POWERFAIL_ENTRIES:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_COMPART_TRANSACTION_ENTRIES:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_COMPART_OTHERS_ENTRIES:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_COMPART_NONROLLOVER_ENTRIES:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   20 us8 COMPART_VOLTAGE_SIZE,COMPART_CURRENT_SIZE,COMPART_POWERFAIL_SIZE,COMPART_TRANSACTION_SIZE,COMPART_OTHERS_SIZE,COMPART_NONROLLOVER_SIZE;
_COMPART_VOLTAGE_SIZE:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_COMPART_CURRENT_SIZE:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_COMPART_POWERFAIL_SIZE:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_COMPART_TRANSACTION_SIZE:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_COMPART_OTHERS_SIZE:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_COMPART_NONROLLOVER_SIZE:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   21 us16 COMPART_VOLTAGE_END_ADD,COMPART_CURRENT_END_ADD,COMPART_POWERFAIL_END_ADD,COMPART_TRANSACTION_END_ADD,COMPART_OTHERS_END_ADD,COMPART_NONROLLOVER_END_ADD;
_COMPART_VOLTAGE_END_ADD:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_COMPART_CURRENT_END_ADD:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_COMPART_POWERFAIL_END_ADD:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_COMPART_TRANSACTION_END_ADD:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_COMPART_OTHERS_END_ADD:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_COMPART_NONROLLOVER_END_ADD:
        DS 2
//   22 
//   23 
//   24 /************************************ Local Functions *******************************/
//   25 /************************************ Extern Functions ******************************/
//   26 
//   27 void memory_log_init();
//   28 void memory_log_reset();
//   29 void memory_log_reset_non_roll();

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock0 Using cfiCommon0
          CFI Function _memory_log_init
        CODE
//   30 void memory_log_init()
//   31 {
_memory_log_init:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//   32 
//   33   COMPART_VOLTAGE_START_ADD = 0x7500;
        MOVW      AX, #0x7500        ;; 1 cycle
        MOVW      N:_COMPART_VOLTAGE_START_ADD, AX  ;; 1 cycle
//   34   COMPART_VOLTAGE_ENTRIES = COMP1_TPR_EVENT;
        MOV       A, N:_COMP1_TPR_EVENT  ;; 1 cycle
        MOV       N:_COMPART_VOLTAGE_ENTRIES, A  ;; 1 cycle
//   35   COMPART_VOLTAGE_SIZE = 48u;
        MOV       N:_COMPART_VOLTAGE_SIZE, #0x30  ;; 1 cycle
//   36   COMPART_VOLTAGE_END_ADD = ((COMPART_VOLTAGE_START_ADD) + ((us16)COMPART_VOLTAGE_ENTRIES*COMPART_VOLTAGE_SIZE));
        MOV       X, N:_COMPART_VOLTAGE_SIZE  ;; 1 cycle
        MOV       A, N:_COMPART_VOLTAGE_ENTRIES  ;; 1 cycle
        MULU      X                  ;; 1 cycle
        ADDW      AX, N:_COMPART_VOLTAGE_START_ADD  ;; 1 cycle
        MOVW      N:_COMPART_VOLTAGE_END_ADD, AX  ;; 1 cycle
//   37   
//   38   COMPART_CURRENT_START_ADD = COMPART_VOLTAGE_END_ADD;
        MOVW      AX, N:_COMPART_VOLTAGE_END_ADD  ;; 1 cycle
        MOVW      N:_COMPART_CURRENT_START_ADD, AX  ;; 1 cycle
//   39   COMPART_CURRENT_ENTRIES = COMP2_TPR_EVENT;
        MOV       A, N:_COMP2_TPR_EVENT  ;; 1 cycle
        MOV       N:_COMPART_CURRENT_ENTRIES, A  ;; 1 cycle
//   40   COMPART_CURRENT_SIZE = 48u;
        MOV       N:_COMPART_CURRENT_SIZE, #0x30  ;; 1 cycle
//   41   COMPART_CURRENT_END_ADD = ((COMPART_CURRENT_START_ADD) + ((us16)COMPART_CURRENT_ENTRIES*COMPART_CURRENT_SIZE));
        MOV       X, N:_COMPART_CURRENT_SIZE  ;; 1 cycle
        MOV       A, N:_COMPART_CURRENT_ENTRIES  ;; 1 cycle
        MULU      X                  ;; 1 cycle
        ADDW      AX, N:_COMPART_CURRENT_START_ADD  ;; 1 cycle
        MOVW      N:_COMPART_CURRENT_END_ADD, AX  ;; 1 cycle
//   42   
//   43   COMPART_POWERFAIL_START_ADD = COMPART_CURRENT_END_ADD;
        MOVW      AX, N:_COMPART_CURRENT_END_ADD  ;; 1 cycle
        MOVW      N:_COMPART_POWERFAIL_START_ADD, AX  ;; 1 cycle
//   44   COMPART_POWERFAIL_ENTRIES = 90u;
        MOV       N:_COMPART_POWERFAIL_ENTRIES, #0x5A  ;; 1 cycle
//   45   COMPART_POWERFAIL_SIZE = 16u;
        MOV       N:_COMPART_POWERFAIL_SIZE, #0x10  ;; 1 cycle
//   46   COMPART_POWERFAIL_END_ADD = ((COMPART_POWERFAIL_START_ADD) + ((us16)COMPART_POWERFAIL_ENTRIES*COMPART_POWERFAIL_SIZE));
        MOV       X, N:_COMPART_POWERFAIL_SIZE  ;; 1 cycle
        MOV       A, N:_COMPART_POWERFAIL_ENTRIES  ;; 1 cycle
        MULU      X                  ;; 1 cycle
        ADDW      AX, N:_COMPART_POWERFAIL_START_ADD  ;; 1 cycle
        MOVW      N:_COMPART_POWERFAIL_END_ADD, AX  ;; 1 cycle
//   47   
//   48   COMPART_TRANSACTION_START_ADD = COMPART_POWERFAIL_END_ADD;
        MOVW      AX, N:_COMPART_POWERFAIL_END_ADD  ;; 1 cycle
        MOVW      N:_COMPART_TRANSACTION_START_ADD, AX  ;; 1 cycle
//   49   COMPART_TRANSACTION_ENTRIES = 32u;
        MOV       N:_COMPART_TRANSACTION_ENTRIES, #0x20  ;; 1 cycle
//   50   COMPART_TRANSACTION_SIZE = 16u;
        MOV       N:_COMPART_TRANSACTION_SIZE, #0x10  ;; 1 cycle
//   51   COMPART_TRANSACTION_END_ADD = ((COMPART_TRANSACTION_START_ADD) + ((us16)COMPART_TRANSACTION_ENTRIES*COMPART_TRANSACTION_SIZE));
        MOV       X, N:_COMPART_TRANSACTION_SIZE  ;; 1 cycle
        MOV       A, N:_COMPART_TRANSACTION_ENTRIES  ;; 1 cycle
        MULU      X                  ;; 1 cycle
        ADDW      AX, N:_COMPART_TRANSACTION_START_ADD  ;; 1 cycle
        MOVW      N:_COMPART_TRANSACTION_END_ADD, AX  ;; 1 cycle
//   52   
//   53   COMPART_OTHERS_START_ADD = COMPART_TRANSACTION_END_ADD;
        MOVW      AX, N:_COMPART_TRANSACTION_END_ADD  ;; 1 cycle
        MOVW      N:_COMPART_OTHERS_START_ADD, AX  ;; 1 cycle
//   54   COMPART_OTHERS_ENTRIES = COMP3_TPR_EVENT;
        MOV       A, N:_COMP3_TPR_EVENT  ;; 1 cycle
        MOV       N:_COMPART_OTHERS_ENTRIES, A  ;; 1 cycle
//   55   COMPART_OTHERS_SIZE = 48u;
        MOV       N:_COMPART_OTHERS_SIZE, #0x30  ;; 1 cycle
//   56   COMPART_OTHERS_END_ADD = ((COMPART_OTHERS_START_ADD) + ((us16)COMPART_OTHERS_ENTRIES*COMPART_OTHERS_SIZE));
        MOV       X, N:_COMPART_OTHERS_SIZE  ;; 1 cycle
        MOV       A, N:_COMPART_OTHERS_ENTRIES  ;; 1 cycle
        MULU      X                  ;; 1 cycle
        ADDW      AX, N:_COMPART_OTHERS_START_ADD  ;; 1 cycle
        MOVW      N:_COMPART_OTHERS_END_ADD, AX  ;; 1 cycle
//   57   
//   58   COMPART_NONROLLOVER_START_ADD = COMPART_OTHERS_END_ADD;
        MOVW      AX, N:_COMPART_OTHERS_END_ADD  ;; 1 cycle
        MOVW      N:_COMPART_NONROLLOVER_START_ADD, AX  ;; 1 cycle
//   59   COMPART_NONROLLOVER_ENTRIES = 10u;
        MOV       N:_COMPART_NONROLLOVER_ENTRIES, #0xA  ;; 1 cycle
//   60   COMPART_NONROLLOVER_SIZE = 16u;
        MOV       N:_COMPART_NONROLLOVER_SIZE, #0x10  ;; 1 cycle
//   61   COMPART_NONROLLOVER_END_ADD = ((COMPART_NONROLLOVER_START_ADD) + ((us16)COMPART_NONROLLOVER_ENTRIES*COMPART_NONROLLOVER_SIZE));  
        MOV       X, N:_COMPART_NONROLLOVER_SIZE  ;; 1 cycle
        MOV       A, N:_COMPART_NONROLLOVER_ENTRIES  ;; 1 cycle
        MULU      X                  ;; 1 cycle
        ADDW      AX, N:_COMPART_NONROLLOVER_START_ADD  ;; 1 cycle
        MOVW      N:_COMPART_NONROLLOVER_END_ADD, AX  ;; 1 cycle
//   62   
//   63   if(eprom_read(ADD_SEQ_NO,0,PAGE_1,AUTO_CALC) == EEP_OK)
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xC00         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
        CMP       A, #0x1            ;; 1 cycle
        BNZ       ??memory_log_reset_non_roll_0  ;; 4 cycles
        ; ------------------------------------- Block: 69 cycles
//   64     {
//   65         seq_no_vol          = char_array_to_int(&opr_data[0]);
        MOVW      AX, #LWRD(_opr_data)  ;; 1 cycle
          CFI FunCall _char_array_to_int
        CALL      _char_array_to_int  ;; 3 cycles
        MOVW      N:_seq_no_vol, AX  ;; 1 cycle
//   66         seq_no_curr         = char_array_to_int(&opr_data[2]);
        MOVW      AX, #LWRD(_opr_data+2)  ;; 1 cycle
          CFI FunCall _char_array_to_int
        CALL      _char_array_to_int  ;; 3 cycles
        MOVW      N:_seq_no_curr, AX  ;; 1 cycle
//   67         seq_no_powerfail    = char_array_to_int(&opr_data[4]);
        MOVW      AX, #LWRD(_opr_data+4)  ;; 1 cycle
          CFI FunCall _char_array_to_int
        CALL      _char_array_to_int  ;; 3 cycles
        MOVW      N:_seq_no_powerfail, AX  ;; 1 cycle
//   68         seq_no_transaction  = char_array_to_int(&opr_data[6]);
        MOVW      AX, #LWRD(_opr_data+6)  ;; 1 cycle
          CFI FunCall _char_array_to_int
        CALL      _char_array_to_int  ;; 3 cycles
        MOVW      N:_seq_no_transaction, AX  ;; 1 cycle
//   69         seq_no_others       = char_array_to_int(&opr_data[8]);
        MOVW      AX, #LWRD(_opr_data+8)  ;; 1 cycle
          CFI FunCall _char_array_to_int
        CALL      _char_array_to_int  ;; 3 cycles
        MOVW      N:_seq_no_others, AX  ;; 1 cycle
//   70         seq_no_nonrollover  = char_array_to_int(&opr_data[10]);
        MOVW      AX, #LWRD(_opr_data+10)  ;; 1 cycle
          CFI FunCall _char_array_to_int
        CALL      _char_array_to_int  ;; 3 cycles
        MOVW      N:_seq_no_nonrollover, AX  ;; 1 cycle
//   71         seq_no_debug        = char_array_to_int(&opr_data[12]);
        MOVW      AX, #LWRD(_opr_data+12)  ;; 1 cycle
          CFI FunCall _char_array_to_int
        CALL      _char_array_to_int  ;; 3 cycles
        MOVW      N:_seq_no_debug, AX  ;; 1 cycle
        ; ------------------------------------- Block: 35 cycles
//   72     }    
//   73     if(eprom_read(0x0CB0,0,PAGE_2,AUTO_CALC) == EEP_OK)
??memory_log_reset_non_roll_0:
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x1            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xCB0         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
        CMP       A, #0x1            ;; 1 cycle
        SKZ                          ;; 4 cycles
        BR        N:??memory_log_reset_non_roll_1  ;; 4 cycles
        ; ------------------------------------- Block: 12 cycles
//   74     {
//   75         tpr.cum_tpr_count = char_array_to_int(&opr_data[0]);
        MOVW      AX, #LWRD(_opr_data)  ;; 1 cycle
          CFI FunCall _char_array_to_int
        CALL      _char_array_to_int  ;; 3 cycles
        MOVW      N:_tpr+30, AX      ;; 1 cycle
//   76         tpr.vol_miss.Rph.count = opr_data[2];
        MOV       A, N:_opr_data+2   ;; 1 cycle
        MOV       N:_tpr+257, A      ;; 1 cycle
//   77         tpr.vol_miss.Yph.count = opr_data[3];
        MOV       A, N:_opr_data+3   ;; 1 cycle
        MOV       N:_tpr+285, A      ;; 1 cycle
//   78         tpr.vol_miss.Bph.count = opr_data[4];
        MOV       A, N:_opr_data+4   ;; 1 cycle
        MOV       N:_tpr+313, A      ;; 1 cycle
//   79         tpr.vol_miss_all.count = opr_data[5];
        MOV       A, N:_opr_data+5   ;; 1 cycle
        MOV       N:_tpr+229, A      ;; 1 cycle
//   80         tpr.vol_unbal.count = opr_data[6];
        MOV       A, N:_opr_data+6   ;; 1 cycle
        MOV       N:_tpr+61, A       ;; 1 cycle
//   81         tpr.vol_low.count = opr_data[7];
        MOV       A, N:_opr_data+7   ;; 1 cycle
        MOV       N:_tpr+89, A       ;; 1 cycle
//   82         tpr.vol_high.count = opr_data[8];
        MOV       A, N:_opr_data+8   ;; 1 cycle
        MOV       N:_tpr+117, A      ;; 1 cycle
//   83         tpr.curr_unbal.count = opr_data[9];
        MOV       A, N:_opr_data+9   ;; 1 cycle
        MOV       N:_tpr+145, A      ;; 1 cycle
//   84         tpr.ct_bypass.count = opr_data[10];
        MOV       A, N:_opr_data+10  ;; 1 cycle
        MOV       N:_tpr+173, A      ;; 1 cycle
//   85         tpr.ct_rev.Rph.count = opr_data[11];
        MOV       A, N:_opr_data+11  ;; 1 cycle
        MOV       N:_tpr+341, A      ;; 1 cycle
//   86         tpr.ct_rev.Yph.count = opr_data[12];
        MOV       A, N:_opr_data+12  ;; 1 cycle
        MOV       N:_tpr+369, A      ;; 1 cycle
//   87         tpr.ct_rev.Bph.count = opr_data[13];
        MOV       A, N:_opr_data+13  ;; 1 cycle
        MOV       N:_tpr+397, A      ;; 1 cycle
//   88         tpr.ct_open.Rph.count = opr_data[14];
        MOV       A, N:_opr_data+14  ;; 1 cycle
        MOV       N:_tpr+425, A      ;; 1 cycle
//   89         tpr.ct_open.Yph.count = opr_data[15];
        MOV       A, N:_opr_data+15  ;; 1 cycle
        MOV       N:_tpr+453, A      ;; 1 cycle
//   90         tpr.ct_open.Bph.count = opr_data[16];
        MOV       A, N:_opr_data+16  ;; 1 cycle
        MOV       N:_tpr+481, A      ;; 1 cycle
//   91         tpr.curr_over.Rph.count = opr_data[17];
        MOV       A, N:_opr_data+17  ;; 1 cycle
        MOV       N:_tpr+509, A      ;; 1 cycle
//   92         tpr.curr_over.Yph.count = opr_data[18];
        MOV       A, N:_opr_data+18  ;; 1 cycle
        MOV       N:_tpr+537, A      ;; 1 cycle
//   93         tpr.curr_over.Bph.count = opr_data[19];
        MOV       A, N:_opr_data+19  ;; 1 cycle
        MOV       N:_tpr+565, A      ;; 1 cycle
//   94         tpr.power_fail.count = opr_data[20];
        MOV       A, N:_opr_data+20  ;; 1 cycle
        MOV       N:_tpr+605, A      ;; 1 cycle
//   95 //        rtc = opr_data[21];
//   96         tpr.neu_disturb.count = opr_data[22];
        MOV       A, N:_opr_data+22  ;; 1 cycle
        MOV       N:_tpr+201, A      ;; 1 cycle
//   97         tpr.magnet.count = opr_data[23];
        MOV       A, N:_opr_data+23  ;; 1 cycle
        MOV       N:_tpr+33, A       ;; 1 cycle
//   98 //        low_pf = opr_data[24];
//   99         tpr.top_cover.count = opr_data[25];
        MOV       A, N:_opr_data+25  ;; 1 cycle
        MOV       N:_tpr+593, A      ;; 1 cycle
        ; ------------------------------------- Block: 49 cycles
//  100     }
//  101     if(eprom_read(0x0CE0,0,PAGE_1,AUTO_CALC) == EEP_OK)
??memory_log_reset_non_roll_1:
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xCE0         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
        CMP       A, #0x1            ;; 1 cycle
        BNZ       ??memory_log_reset_non_roll_2  ;; 4 cycles
        ; ------------------------------------- Block: 12 cycles
//  102     {
//  103         tpr.vol_related_count = opr_data[0];
        MOV       A, N:_opr_data     ;; 1 cycle
        MOV       N:_tpr, A          ;; 1 cycle
//  104         tpr.vol_related_overflow = opr_data[1];
        MOV       A, N:_opr_data+1   ;; 1 cycle
        MOV       N:_tpr+1, A        ;; 1 cycle
//  105         tpr.curr_related_count = opr_data[2];
        MOV       A, N:_opr_data+2   ;; 1 cycle
        MOV       N:_tpr+4, A        ;; 1 cycle
//  106         tpr.curr_related_overflow = opr_data[3];
        MOV       A, N:_opr_data+3   ;; 1 cycle
        MOV       N:_tpr+5, A        ;; 1 cycle
//  107         tpr.power_count = opr_data[4];
        MOV       A, N:_opr_data+4   ;; 1 cycle
        MOV       N:_tpr+8, A        ;; 1 cycle
//  108         tpr.power_overflow = opr_data[5];
        MOV       A, N:_opr_data+5   ;; 1 cycle
        MOV       N:_tpr+9, A        ;; 1 cycle
//  109         tpr.transaction_count = opr_data[6];
        MOV       A, N:_opr_data+6   ;; 1 cycle
        MOV       N:_tpr+12, A       ;; 1 cycle
//  110         tpr.transaction_overflow = opr_data[7];
        MOV       A, N:_opr_data+7   ;; 1 cycle
        MOV       N:_tpr+13, A       ;; 1 cycle
//  111         tpr.others_count = opr_data[8];
        MOV       A, N:_opr_data+8   ;; 1 cycle
        MOV       N:_tpr+16, A       ;; 1 cycle
//  112         tpr.others_overflow = opr_data[9];
        MOV       A, N:_opr_data+9   ;; 1 cycle
        MOV       N:_tpr+17, A       ;; 1 cycle
//  113         tpr.non_roll_count = opr_data[10];
        MOV       A, N:_opr_data+10  ;; 1 cycle
        MOV       N:_tpr+20, A       ;; 1 cycle
//  114         tpr.non_roll_overflow = opr_data[11];
        MOV       A, N:_opr_data+11  ;; 1 cycle
        MOV       N:_tpr+21, A       ;; 1 cycle
//  115         tpr.debug_count = opr_data[12];
        MOV       A, N:_opr_data+12  ;; 1 cycle
        MOV       N:_tpr+24, A       ;; 1 cycle
//  116         tpr.debug_overflow = opr_data[13];
        MOV       A, N:_opr_data+13  ;; 1 cycle
        MOV       N:_tpr+25, A       ;; 1 cycle
        ; ------------------------------------- Block: 28 cycles
//  117     }
//  118     tpr.vol_related_entries = COMPART_VOLTAGE_ENTRIES;
??memory_log_reset_non_roll_2:
        MOV       A, N:_COMPART_VOLTAGE_ENTRIES  ;; 1 cycle
        MOV       N:_tpr+2, A        ;; 1 cycle
//  119     tpr.curr_related_entries = COMPART_CURRENT_ENTRIES;
        MOV       A, N:_COMPART_CURRENT_ENTRIES  ;; 1 cycle
        MOV       N:_tpr+6, A        ;; 1 cycle
//  120     tpr.power_entries = COMPART_POWERFAIL_ENTRIES;
        MOV       A, N:_COMPART_POWERFAIL_ENTRIES  ;; 1 cycle
        MOV       N:_tpr+10, A       ;; 1 cycle
//  121     tpr.others_entries = COMPART_OTHERS_ENTRIES;
        MOV       A, N:_COMPART_OTHERS_ENTRIES  ;; 1 cycle
        MOV       N:_tpr+18, A       ;; 1 cycle
//  122     tpr.non_roll_entries = COMPART_NONROLLOVER_ENTRIES;
        MOV       A, N:_COMPART_NONROLLOVER_ENTRIES  ;; 1 cycle
        MOV       N:_tpr+22, A       ;; 1 cycle
//  123     tpr.transaction_entries = COMPART_TRANSACTION_ENTRIES;
        MOV       A, N:_COMPART_TRANSACTION_ENTRIES  ;; 1 cycle
        MOV       N:_tpr+14, A       ;; 1 cycle
//  124     tpr.debug_entries = COMPART_DEBUG_ENTRIES;
        MOV       N:_tpr+26, #0x10   ;; 1 cycle
//  125 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock0
        ; ------------------------------------- Block: 19 cycles
        ; ------------------------------------- Total: 224 cycles
//  126 
//  127 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock1 Using cfiCommon0
          CFI Function _memory_log_reset
        CODE
//  128 void memory_log_reset()
//  129 {
_memory_log_reset:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  130     /* clearing variables */
//  131     seq_no_vol = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_seq_no_vol, AX  ;; 1 cycle
//  132     seq_no_curr = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_seq_no_curr, AX  ;; 1 cycle
//  133     seq_no_powerfail = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_seq_no_powerfail, AX  ;; 1 cycle
//  134     seq_no_transaction = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_seq_no_transaction, AX  ;; 1 cycle
//  135     seq_no_others = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_seq_no_others, AX  ;; 1 cycle
//  136     seq_no_nonrollover = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_seq_no_nonrollover, AX  ;; 1 cycle
//  137     seq_no_debug = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_seq_no_debug, AX  ;; 1 cycle
//  138     tpr.cum_tpr_count = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_tpr+30, AX      ;; 1 cycle
//  139     tpr.vol_miss.Rph.count = 0;
        MOV       N:_tpr+257, #0x0   ;; 1 cycle
//  140     tpr.vol_miss.Yph.count = 0;
        MOV       N:_tpr+285, #0x0   ;; 1 cycle
//  141     tpr.vol_miss.Bph.count = 0;
        MOV       N:_tpr+313, #0x0   ;; 1 cycle
//  142     tpr.vol_miss_all.count = 0;
        MOV       N:_tpr+229, #0x0   ;; 1 cycle
//  143     tpr.vol_unbal.count = 0;
        MOV       N:_tpr+61, #0x0    ;; 1 cycle
//  144     tpr.vol_low.count = 0;
        MOV       N:_tpr+89, #0x0    ;; 1 cycle
//  145     tpr.vol_high.count = 0;
        MOV       N:_tpr+117, #0x0   ;; 1 cycle
//  146     tpr.curr_unbal.count = 0;
        MOV       N:_tpr+145, #0x0   ;; 1 cycle
//  147     tpr.ct_bypass.count = 0;
        MOV       N:_tpr+173, #0x0   ;; 1 cycle
//  148     tpr.ct_rev.Rph.count = 0;
        MOV       N:_tpr+341, #0x0   ;; 1 cycle
//  149     tpr.ct_rev.Yph.count = 0;
        MOV       N:_tpr+369, #0x0   ;; 1 cycle
//  150     tpr.ct_rev.Bph.count = 0;
        MOV       N:_tpr+397, #0x0   ;; 1 cycle
//  151     tpr.ct_open.Rph.count = 0;
        MOV       N:_tpr+425, #0x0   ;; 1 cycle
//  152     tpr.ct_open.Yph.count = 0;
        MOV       N:_tpr+453, #0x0   ;; 1 cycle
//  153     tpr.ct_open.Bph.count = 0;
        MOV       N:_tpr+481, #0x0   ;; 1 cycle
//  154     tpr.curr_over.Rph.count = 0;
        MOV       N:_tpr+509, #0x0   ;; 1 cycle
//  155     tpr.curr_over.Yph.count = 0;
        MOV       N:_tpr+537, #0x0   ;; 1 cycle
//  156     tpr.curr_over.Bph.count = 0;
        MOV       N:_tpr+565, #0x0   ;; 1 cycle
//  157     tpr.power_fail.count = 0;
        MOV       N:_tpr+605, #0x0   ;; 1 cycle
//  158     //        rtc = 0;
//  159     tpr.neu_disturb.count = 0;
        MOV       N:_tpr+201, #0x0   ;; 1 cycle
//  160     tpr.magnet.count = 0;
        MOV       N:_tpr+33, #0x0    ;; 1 cycle
//  161     //        low_pf = 0;
//  162     tpr.top_cover.count = 0; 
        MOV       N:_tpr+593, #0x0   ;; 1 cycle
//  163     tpr.vol_related_count = 0;
        MOV       N:_tpr, #0x0       ;; 1 cycle
//  164     tpr.vol_related_overflow = 0;
        MOV       N:_tpr+1, #0x0     ;; 1 cycle
//  165     tpr.curr_related_count = 0;
        MOV       N:_tpr+4, #0x0     ;; 1 cycle
//  166     tpr.curr_related_overflow = 0;
        MOV       N:_tpr+5, #0x0     ;; 1 cycle
//  167     tpr.power_count = 0;
        MOV       N:_tpr+8, #0x0     ;; 1 cycle
//  168     tpr.power_overflow = 0;
        MOV       N:_tpr+9, #0x0     ;; 1 cycle
//  169     tpr.transaction_count = 0;
        MOV       N:_tpr+12, #0x0    ;; 1 cycle
//  170     tpr.transaction_overflow = 0;
        MOV       N:_tpr+13, #0x0    ;; 1 cycle
//  171     tpr.others_count = 0;
        MOV       N:_tpr+16, #0x0    ;; 1 cycle
//  172     tpr.others_overflow = 0;
        MOV       N:_tpr+17, #0x0    ;; 1 cycle
//  173     tpr.non_roll_count = 0;
        MOV       N:_tpr+20, #0x0    ;; 1 cycle
//  174     tpr.non_roll_overflow = 0;
        MOV       N:_tpr+21, #0x0    ;; 1 cycle
//  175     tpr.debug_count = 0;
        MOV       N:_tpr+24, #0x0    ;; 1 cycle
//  176     tpr.debug_overflow = 0;
        MOV       N:_tpr+25, #0x0    ;; 1 cycle
//  177         
//  178     tamper_instant_status = 0;    
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_tamper_instant_status, AX  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_tamper_instant_status+2, AX  ;; 1 cycle
//  179     
//  180     fill_oprzero(48);
        MOV       A, #0x30           ;; 1 cycle
          CFI FunCall _fill_oprzero
        CALL      _fill_oprzero      ;; 3 cycles
//  181     eprom_write(ADD_SEQ_NO,0,16,PAGE_1,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+6
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xC00         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  182     eprom_write(0x0C10,0,16,PAGE_1,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+8
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xC10         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  183     eprom_write(0x0CB0,0,32,PAGE_2,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        MOV       B, #0x1            ;; 1 cycle
        MOVW      DE, #0x20          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xCB0         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  184     eprom_write(0x0CE0,0,16,PAGE_1,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xCE0         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  185 
//  186     eprom_write(0x0CA0,0,16,PAGE_1,AUTO_CALC);          /* last event ID */
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+14
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xCA0         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  187     eprom_write(0x0C90,0,16,PAGE_1,AUTO_CALC);          /* Magnet and top cover time */
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+16
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xC90         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  188     
//  189     eprom_write(COMPART_DEBUG_START_ADD,0,(us16)COMPART_DEBUG_ENTRIES*COMPART_DEBUG_SIZE,PAGE_1,AUTO_CALC); 
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+18
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x100         ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x7400        ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  190     eprom_write(COMPART_VOLTAGE_START_ADD,0,(us16)COMPART_VOLTAGE_ENTRIES*COMPART_VOLTAGE_SIZE,PAGE_3,AUTO_CALC); 
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+20
        MOV       B, #0x2            ;; 1 cycle
        MOV       X, N:_COMPART_VOLTAGE_SIZE  ;; 1 cycle
        MOV       A, N:_COMPART_VOLTAGE_ENTRIES  ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, N:_COMPART_VOLTAGE_START_ADD  ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  191     eprom_write(COMPART_CURRENT_START_ADD,0,(us16)COMPART_CURRENT_ENTRIES*COMPART_CURRENT_SIZE,PAGE_3,AUTO_CALC); 
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+22
        MOV       B, #0x2            ;; 1 cycle
        MOV       X, N:_COMPART_CURRENT_SIZE  ;; 1 cycle
        MOV       A, N:_COMPART_CURRENT_ENTRIES  ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, N:_COMPART_CURRENT_START_ADD  ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  192     eprom_write(COMPART_POWERFAIL_START_ADD,0,(us16)COMPART_POWERFAIL_ENTRIES*COMPART_POWERFAIL_SIZE,PAGE_1,AUTO_CALC); 
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+24
        MOV       B, #0x0            ;; 1 cycle
        MOV       X, N:_COMPART_POWERFAIL_SIZE  ;; 1 cycle
        MOV       A, N:_COMPART_POWERFAIL_ENTRIES  ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, N:_COMPART_POWERFAIL_START_ADD  ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  193     eprom_write(COMPART_TRANSACTION_START_ADD,0,(us16)COMPART_TRANSACTION_ENTRIES*COMPART_TRANSACTION_SIZE,PAGE_1,AUTO_CALC); 
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+26
        MOV       B, #0x0            ;; 1 cycle
        MOV       X, N:_COMPART_TRANSACTION_SIZE  ;; 1 cycle
        MOV       A, N:_COMPART_TRANSACTION_ENTRIES  ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, N:_COMPART_TRANSACTION_START_ADD  ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  194     eprom_write(COMPART_OTHERS_START_ADD,0,(us16)COMPART_OTHERS_ENTRIES*COMPART_OTHERS_SIZE,PAGE_3,AUTO_CALC); 
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+28
        MOV       B, #0x2            ;; 1 cycle
        MOV       X, N:_COMPART_OTHERS_SIZE  ;; 1 cycle
        MOV       A, N:_COMPART_OTHERS_ENTRIES  ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, N:_COMPART_OTHERS_START_ADD  ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  195     eprom_write(COMPART_NONROLLOVER_START_ADD,0,(us16)COMPART_NONROLLOVER_ENTRIES*COMPART_NONROLLOVER_SIZE,PAGE_1,AUTO_CALC); 
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+30
        MOV       B, #0x0            ;; 1 cycle
        MOV       X, N:_COMPART_NONROLLOVER_SIZE  ;; 1 cycle
        MOV       A, N:_COMPART_NONROLLOVER_ENTRIES  ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, N:_COMPART_NONROLLOVER_START_ADD  ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  196     read_tamper_variables();
          CFI FunCall _read_tamper_variables
        CALL      _read_tamper_variables  ;; 3 cycles
//  197 }
        ADDW      SP, #0x1A          ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock1
        ; ------------------------------------- Block: 205 cycles
        ; ------------------------------------- Total: 205 cycles
//  198 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock2 Using cfiCommon0
          CFI Function _memory_log_reset_non_roll
        CODE
//  199 void memory_log_reset_non_roll()
//  200 {
_memory_log_reset_non_roll:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  201     /* Clearing variables */
//  202             
//  203     eprom_read(ADD_SEQ_NO,0,PAGE_1,AUTO_CALC);
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xC00         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
//  204     seq_no_nonrollover = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_seq_no_nonrollover, AX  ;; 1 cycle
//  205     int_into_char_array(seq_no_nonrollover,&opr_data[10]);
        MOVW      BC, #LWRD(_opr_data+10)  ;; 1 cycle
        MOVW      AX, N:_seq_no_nonrollover  ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  206     eprom_write(ADD_SEQ_NO,0,16,PAGE_1,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+6
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xC00         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  207     
//  208     tpr.top_cover.flag = 0;
        MOV       N:_tpr+592, #0x0   ;; 1 cycle
//  209     update_tamper_variables();
          CFI FunCall _update_tamper_variables
        CALL      _update_tamper_variables  ;; 3 cycles
//  210     
//  211     eprom_read(0x0CB0,0,PAGE_2,AUTO_CALC);
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x1            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xCB0         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
//  212     tpr.top_cover.count = 0;
        MOV       N:_tpr+593, #0x0   ;; 1 cycle
//  213     opr_data[25] = tpr.top_cover.count;
        MOV       A, N:_tpr+593      ;; 1 cycle
        MOV       N:_opr_data+25, A  ;; 1 cycle
//  214     eprom_write(0x0CB0,0,32,PAGE_2,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+8
        MOV       B, #0x1            ;; 1 cycle
        MOVW      DE, #0x20          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xCB0         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  215     
//  216     eprom_read(0x0CE0,0,PAGE_1,AUTO_CALC);
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xCE0         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
//  217     tpr.non_roll_count = 0;
        MOV       N:_tpr+20, #0x0    ;; 1 cycle
//  218     tpr.non_roll_overflow = 0;
        MOV       N:_tpr+21, #0x0    ;; 1 cycle
//  219     opr_data[10] = tpr.non_roll_count;
        MOV       A, N:_tpr+20       ;; 1 cycle
        MOV       N:_opr_data+10, A  ;; 1 cycle
//  220     opr_data[11] = tpr.non_roll_overflow;
        MOV       A, N:_tpr+21       ;; 1 cycle
        MOV       N:_opr_data+11, A  ;; 1 cycle
//  221     eprom_write(0x0CE0,0,16,PAGE_1,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xCE0         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  222 
//  223     eprom_read(0x0CA0,0,PAGE_1,AUTO_CALC);
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xCA0         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
//  224     tpr.non_roll_last = 0;
        MOV       N:_tpr+23, #0x0    ;; 1 cycle
//  225     opr_data[5] = tpr.non_roll_last;
        MOV       A, N:_tpr+23       ;; 1 cycle
        MOV       N:_opr_data+5, A   ;; 1 cycle
//  226     eprom_write(0x0CA0,0,16,PAGE_1,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xCA0         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  227     
//  228     eprom_read(0x0C90,0,PAGE_1,AUTO_CALC);    
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xC90         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
//  229     tpr.top_cover.time.day = 0;
        MOV       N:_tpr+597, #0x0   ;; 1 cycle
//  230     tpr.top_cover.time.month = 0;
        MOV       N:_tpr+599, #0x0   ;; 1 cycle
//  231     tpr.top_cover.time.year = 0;
        MOV       N:_tpr+600, #0x0   ;; 1 cycle
//  232     tpr.top_cover.time.hour = 0;
        MOV       N:_tpr+596, #0x0   ;; 1 cycle
//  233     tpr.top_cover.time.min = 0;
        MOV       N:_tpr+595, #0x0   ;; 1 cycle
//  234     tpr.top_cover.time.sec = 0;
        MOV       N:_tpr+594, #0x0   ;; 1 cycle
//  235     time_into_char_array5_sec(tpr.top_cover.time,&opr_data[0]);
        MOVW      HL, #LWRD(_tpr+594)  ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+20
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      AX, #LWRD(_opr_data)  ;; 1 cycle
          CFI FunCall _time_into_char_array5_sec
        CALL      _time_into_char_array5_sec  ;; 3 cycles
//  236     eprom_write(0x0C90,0,16,PAGE_1,AUTO_CALC);    
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+22
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xC90         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  237     
//  238     fill_oprzero(16);
        MOV       A, #0x10           ;; 1 cycle
          CFI FunCall _fill_oprzero
        CALL      _fill_oprzero      ;; 3 cycles
//  239     eprom_write(COMPART_NONROLLOVER_START_ADD,0,(us16)COMPART_NONROLLOVER_ENTRIES*COMPART_NONROLLOVER_SIZE,PAGE_1,AUTO_CALC); 
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+24
        MOV       B, #0x0            ;; 1 cycle
        MOV       X, N:_COMPART_NONROLLOVER_SIZE  ;; 1 cycle
        MOV       A, N:_COMPART_NONROLLOVER_ENTRIES  ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, N:_COMPART_NONROLLOVER_START_ADD  ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//  240     
//  241     read_tamper_variables();
          CFI FunCall _read_tamper_variables
        CALL      _read_tamper_variables  ;; 3 cycles
//  242 }
        ADDW      SP, #0x14          ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock2
        ; ------------------------------------- Block: 146 cycles
        ; ------------------------------------- Total: 146 cycles

        SECTION `.iar_vfe_header`:DATA:NOALLOC:NOROOT(1)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
        DC32 0

        END
// 
//    50 bytes in section .bss
// 1'279 bytes in section .text
// 
// 1'279 bytes of CODE memory
//    50 bytes of DATA memory
//
//Errors: none
//Warnings: none
