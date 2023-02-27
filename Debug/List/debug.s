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
//        BootCode\source_code\source_files\debug.c
//    Command line       =
//        -f C:\Users\DHEERA~1\AppData\Local\Temp\EWBC74.tmp ("E:\0. Dheeraj\0.
//        Official\1. Genus\2. Projects\0. GDEV72 -
//        BootCode\source_code\source_files\debug.c" --core s3 --code_model
//        near --calling_convention v2 --near_const_location ram -o "E:\0.
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
//        BootCode\Debug\List\debug.s
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

        EXTERN _trn_buf
        EXTERN _optical_f
        EXTERN _req_cnt
        EXTERN _send_data_to_uart
        EXTERN _trn_cnt

        PUBLIC _debug_data_send
        PUBLIC _debug_data_type
        PUBLIC _flag_debug
        
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
        
// E:\0. Dheeraj\0. Official\1. Genus\2. Projects\0. GDEV72 - BootCode\source_code\source_files\debug.c
//    1 /***********************************************************************************************************************
//    2 * File Name       : debug.c
//    3 * Current Version : rev_01  
//    4 * Tool-Chain      : IAR Systems RL78
//    5 * Description     : this file has the routines to diagnose the meter
//    6 * Creation Date   : 31-07-2020
//    7 * Company         : Genus Power Infrastructures Limited, Jaipur
//    8 * Author          : dheeraj.singhal
//    9 * Version History : rev_01 :
//   10 ***********************************************************************************************************************/
//   11 /************************************ Includes **************************************/
//   12 #include "debug.h"
//   13 /************************************ Local Variables *****************************************/
//   14 /************************************ Extern Variables *****************************************/

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   15 us8 debug_data_type;
_debug_data_type:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   16 flag_union flag_debug;
_flag_debug:
        DS 1
//   17 
//   18 /************************************ Local Functions *******************************/
//   19 /************************************ Extern Functions ******************************/
//   20 void debug_data_send();
//   21 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock0 Using cfiCommon0
          CFI Function _debug_data_send
        CODE
//   22 void debug_data_send()
//   23 {
_debug_data_send:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 2
        SUBW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+6
//   24   us16 trn_buf_ptr = 0;
        MOVW      BC, #0x0           ;; 1 cycle
//   25   trn_buf[(trn_buf_ptr)++] = '\n';
        MOV       A, #0xA            ;; 1 cycle
        MOV       (_trn_buf)[BC], A  ;; 1 cycle
        INCW      BC                 ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [SP], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//   26   if(debug_data_type == '0')
        CMP       N:_debug_data_type, #0x30  ;; 1 cycle
        BNZ       ??debug_data_send_0  ;; 4 cycles
        ; ------------------------------------- Block: 13 cycles
//   27   {
//   28     /* Instant parameters */
//   29 #if 0
//   30     stringToBufferString("Time:",trn_buf,&trn_buf_ptr); 
//   31     if(Now.hour < 0x10)
//   32     {
//   33       stringToBufferString("0",trn_buf,&trn_buf_ptr); 
//   34     }
//   35     numberToBufferString(bcd_to_decimal(Now.hour), trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);     trn_buf[(trn_buf_ptr)++] = ':';
//   36     if(Now.min < 0x10)
//   37     {
//   38       stringToBufferString("0",trn_buf,&trn_buf_ptr); 
//   39     }
//   40     numberToBufferString(bcd_to_decimal(Now.min), trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);     trn_buf[(trn_buf_ptr)++] = ':';   
//   41     if(Now.sec < 0x10)
//   42     {
//   43       stringToBufferString("0",trn_buf,&trn_buf_ptr); 
//   44     }
//   45     numberToBufferString(bcd_to_decimal(Now.sec), trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);     trn_buf[(trn_buf_ptr)++] = ' ';
//   46     if(Now.day < 0x10)
//   47     {
//   48       stringToBufferString("0",trn_buf,&trn_buf_ptr); 
//   49     }
//   50     numberToBufferString(bcd_to_decimal(Now.day), trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);     trn_buf[(trn_buf_ptr)++] = '-';
//   51     if(Now.month < 0x10)
//   52     {
//   53       stringToBufferString("0",trn_buf,&trn_buf_ptr); 
//   54     }
//   55     numberToBufferString(bcd_to_decimal(Now.month), trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);     trn_buf[(trn_buf_ptr)++] = '-';
//   56     if(Now.year < 0x10)
//   57     {
//   58       stringToBufferString("0",trn_buf,&trn_buf_ptr); 
//   59     }
//   60     numberToBufferString(bcd_to_decimal(Now.year), trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);     trn_buf[(trn_buf_ptr)++] = '\t';
//   61     
//   62 #endif
//   63 #if 0
//   64       numberToBufferString(test_counter, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);
//   65 #endif
//   66 #if 0
//   67     stringToBufferString("Interrupt Flags:\t",trn_buf,&trn_buf_ptr); 
//   68     numberToBufferString(system_interrupt_flags[0], trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);     trn_buf[(trn_buf_ptr)++] = '\t';
//   69     numberToBufferString(system_interrupt_flags[1], trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);     trn_buf[(trn_buf_ptr)++] = '\t';
//   70     numberToBufferString(system_interrupt_flags[2], trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);     trn_buf[(trn_buf_ptr)++] = '\t';
//   71     numberToBufferString(system_interrupt_flags[3], trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);     trn_buf[(trn_buf_ptr)++] = '\t';
//   72     numberToBufferString(system_interrupt_flags[4], trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);     trn_buf[(trn_buf_ptr)++] = '\t';
//   73     numberToBufferString(system_interrupt_flags[5], trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);     trn_buf[(trn_buf_ptr)++] = '\t';
//   74     numberToBufferString(system_interrupt_flags[6], trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);     trn_buf[(trn_buf_ptr)++] = '\t';
//   75     numberToBufferString(system_interrupt_flags[7], trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);     trn_buf[(trn_buf_ptr)++] = '\t';
//   76     numberToBufferString(system_interrupt_flags[8], trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);     trn_buf[(trn_buf_ptr)++] = '\n';
//   77 #endif
//   78 #if 0
//   79     stringToBufferString("Voltage:\t",trn_buf,&trn_buf_ptr); 
//   80     numberToBufferString(vol.Rph.rms, trn_buf, &trn_buf_ptr, 2, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';trn_buf[(trn_buf_ptr)++] = '\t';  numberToBufferString(vol.Yph.rms, trn_buf, &trn_buf_ptr, 2, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';trn_buf[(trn_buf_ptr)++] = '\t';  numberToBufferString(vol.Bph.rms, trn_buf, &trn_buf_ptr, 2, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\n';
//   81 #endif
//   82 #if 0
//   83     stringToBufferString("V offset:\t",trn_buf,&trn_buf_ptr); 
//   84     numberToBufferString(r_phase.vol.dc_offset, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';trn_buf[(trn_buf_ptr)++] = '\t';  numberToBufferString(y_phase.vol.dc_offset, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';trn_buf[(trn_buf_ptr)++] = '\t';  numberToBufferString(b_phase.vol.dc_offset, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\n';
//   85 #endif
//   86 #if 0
//   87     stringToBufferString("Current:\t",trn_buf,&trn_buf_ptr); 
//   88     numberToBufferString(curr.Rph.rms, trn_buf, &trn_buf_ptr, 3, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';trn_buf[(trn_buf_ptr)++] = '\t';  numberToBufferString(curr.Yph.rms, trn_buf, &trn_buf_ptr, 3, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';trn_buf[(trn_buf_ptr)++] = '\t';  numberToBufferString(curr.Bph.rms, trn_buf, &trn_buf_ptr, 3, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';trn_buf[(trn_buf_ptr)++] = '\t';numberToBufferString(curr.Nph.rms, trn_buf, &trn_buf_ptr, 3, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\n';
//   89 #endif
//   90 #if 0
//   91     stringToBufferString("I offset:\t",trn_buf,&trn_buf_ptr); 
//   92     numberToBufferString(r_phase.curr.dc_offset, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';trn_buf[(trn_buf_ptr)++] = '\t';  numberToBufferString(y_phase.curr.dc_offset, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';trn_buf[(trn_buf_ptr)++] = '\t';  numberToBufferString(b_phase.curr.dc_offset, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';trn_buf[(trn_buf_ptr)++] = '\t';numberToBufferString(n_phase.curr.dc_offset, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\n';
//   93 #endif
//   94 #if 0
//   95     stringToBufferString("Power W:\t",trn_buf,&trn_buf_ptr); 
//   96     numberToBufferString(power.Rph.active, trn_buf, &trn_buf_ptr, 1, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';trn_buf[(trn_buf_ptr)++] = '\t'; numberToBufferString(power.Yph.active, trn_buf, &trn_buf_ptr, 1, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';trn_buf[(trn_buf_ptr)++] = '\t'; numberToBufferString(power.Bph.active, trn_buf, &trn_buf_ptr, 1, SIGNED_MODE); trn_buf[(trn_buf_ptr)++] = '\t';trn_buf[(trn_buf_ptr)++] = '\t'; numberToBufferString(power.Allph.active, trn_buf, &trn_buf_ptr, 1, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\n';
//   97     stringToBufferString("Power VAR:\t",trn_buf,&trn_buf_ptr); 
//   98     numberToBufferString(power.Rph.reactive, trn_buf, &trn_buf_ptr, 1, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';trn_buf[(trn_buf_ptr)++] = '\t'; numberToBufferString(power.Yph.reactive, trn_buf, &trn_buf_ptr, 1, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';trn_buf[(trn_buf_ptr)++] = '\t'; numberToBufferString(power.Bph.reactive, trn_buf, &trn_buf_ptr, 1, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';trn_buf[(trn_buf_ptr)++] = '\t'; numberToBufferString(power.Allph.reactive, trn_buf, &trn_buf_ptr, 1, SIGNED_MODE); trn_buf[(trn_buf_ptr)++] = '\n';
//   99     stringToBufferString("Power VA:\t",trn_buf,&trn_buf_ptr); 
//  100     numberToBufferString(power.Rph.apparent, trn_buf, &trn_buf_ptr, 1, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';trn_buf[(trn_buf_ptr)++] = '\t'; numberToBufferString(power.Yph.apparent, trn_buf, &trn_buf_ptr, 1, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';trn_buf[(trn_buf_ptr)++] = '\t'; numberToBufferString(power.Bph.apparent, trn_buf, &trn_buf_ptr, 1, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';trn_buf[(trn_buf_ptr)++] = '\t'; numberToBufferString(power.Allph.apparent, trn_buf, &trn_buf_ptr, 1, SIGNED_MODE); trn_buf[(trn_buf_ptr)++] = '\n';
//  101     stringToBufferString("PF:\t\t",trn_buf,&trn_buf_ptr); 
//  102     numberToBufferString(pf.Rph, trn_buf, &trn_buf_ptr, 3, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';trn_buf[(trn_buf_ptr)++] = '\t';  numberToBufferString(pf.Yph, trn_buf, &trn_buf_ptr, 3, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';trn_buf[(trn_buf_ptr)++] = '\t';  numberToBufferString(pf.Bph, trn_buf, &trn_buf_ptr, 3, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';trn_buf[(trn_buf_ptr)++] = '\t';  numberToBufferString(pf.Net, trn_buf, &trn_buf_ptr, 3, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\n';
//  103 #endif
//  104 #if 0
//  105     stringToBufferString("Signed values\n",trn_buf,&trn_buf_ptr);
//  106     stringToBufferString("Current:\t",trn_buf,&trn_buf_ptr); 
//  107     numberToBufferString(curr.Rph.rms_signed, trn_buf, &trn_buf_ptr, 3, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';trn_buf[(trn_buf_ptr)++] = '\t';  numberToBufferString(curr.Yph.rms_signed, trn_buf, &trn_buf_ptr, 3, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';trn_buf[(trn_buf_ptr)++] = '\t';  numberToBufferString(curr.Bph.rms_signed, trn_buf, &trn_buf_ptr, 3, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\n';
//  108     stringToBufferString("Power W:\t",trn_buf,&trn_buf_ptr); 
//  109     numberToBufferString(power.Rph.active_signed, trn_buf, &trn_buf_ptr, 1, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';trn_buf[(trn_buf_ptr)++] = '\t'; numberToBufferString(power.Yph.active_signed, trn_buf, &trn_buf_ptr, 1, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';trn_buf[(trn_buf_ptr)++] = '\t'; numberToBufferString(power.Bph.active_signed, trn_buf, &trn_buf_ptr, 1, SIGNED_MODE); trn_buf[(trn_buf_ptr)++] = '\t';trn_buf[(trn_buf_ptr)++] = '\t'; numberToBufferString(power.Allph.active_signed, trn_buf, &trn_buf_ptr, 1, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\n';
//  110     stringToBufferString("Power VAR:\t",trn_buf,&trn_buf_ptr); 
//  111     numberToBufferString(power.Rph.reactive_signed, trn_buf, &trn_buf_ptr, 1, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';trn_buf[(trn_buf_ptr)++] = '\t'; numberToBufferString(power.Yph.reactive_signed, trn_buf, &trn_buf_ptr, 1, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';trn_buf[(trn_buf_ptr)++] = '\t'; numberToBufferString(power.Bph.reactive_signed, trn_buf, &trn_buf_ptr, 1, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';trn_buf[(trn_buf_ptr)++] = '\t'; numberToBufferString(power.Allph.reactive_signed, trn_buf, &trn_buf_ptr, 1, SIGNED_MODE); trn_buf[(trn_buf_ptr)++] = '\n';
//  112     stringToBufferString("Power VA:\t",trn_buf,&trn_buf_ptr); 
//  113     numberToBufferString(power.Rph.apparent_signed, trn_buf, &trn_buf_ptr, 1, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';trn_buf[(trn_buf_ptr)++] = '\t'; numberToBufferString(power.Yph.apparent_signed, trn_buf, &trn_buf_ptr, 1, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';trn_buf[(trn_buf_ptr)++] = '\t'; numberToBufferString(power.Bph.apparent_signed, trn_buf, &trn_buf_ptr, 1, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';trn_buf[(trn_buf_ptr)++] = '\t'; numberToBufferString(power.Allph.apparent_signed, trn_buf, &trn_buf_ptr, 1, SIGNED_MODE); trn_buf[(trn_buf_ptr)++] = '\n';
//  114     stringToBufferString("PF:\t\t",trn_buf,&trn_buf_ptr); 
//  115     numberToBufferString(pf.Rph_signed, trn_buf, &trn_buf_ptr, 3, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';trn_buf[(trn_buf_ptr)++] = '\t';  numberToBufferString(pf.Yph_signed, trn_buf, &trn_buf_ptr, 3, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';trn_buf[(trn_buf_ptr)++] = '\t';  numberToBufferString(pf.Bph_signed, trn_buf, &trn_buf_ptr, 3, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';trn_buf[(trn_buf_ptr)++] = '\t';  numberToBufferString(pf.Net_signed, trn_buf, &trn_buf_ptr, 3, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\n';
//  116 #endif
//  117 #if 0    
//  118     stringToBufferString("Frequency:\t",trn_buf,&trn_buf_ptr); 
//  119     numberToBufferString(freq.Rph, trn_buf, &trn_buf_ptr, 3, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';trn_buf[(trn_buf_ptr)++] = '\t'; numberToBufferString(freq.Yph, trn_buf, &trn_buf_ptr, 3, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';trn_buf[(trn_buf_ptr)++] = '\t'; numberToBufferString(freq.Bph, trn_buf, &trn_buf_ptr, 3, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';trn_buf[(trn_buf_ptr)++] = '\t'; numberToBufferString(freq.Net, trn_buf, &trn_buf_ptr, 3, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\n';
//  120 #endif
//  121 #if 0
//  122     stringToBufferString("Angle:\t\t",trn_buf,&trn_buf_ptr); 
//  123     numberToBufferString(angle.Rph, trn_buf, &trn_buf_ptr, 2, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = 186;trn_buf[(trn_buf_ptr)++] = '\t';  numberToBufferString(angle.Yph, trn_buf, &trn_buf_ptr, 2, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = 186;trn_buf[(trn_buf_ptr)++] = '\t';  numberToBufferString(angle.Bph, trn_buf, &trn_buf_ptr, 2, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = 186;trn_buf[(trn_buf_ptr)++] = '\n';
//  124 #endif
//  125 #if 0
//  126     stringToBufferString("Angle P-P:\t",trn_buf,&trn_buf_ptr); 
//  127     numberToBufferString(ph_ph.Ph_RY.angle.value, trn_buf, &trn_buf_ptr, 2, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = 186;trn_buf[(trn_buf_ptr)++] = '\t';  numberToBufferString(ph_ph.Ph_YB.angle.value, trn_buf, &trn_buf_ptr, 2, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = 186;trn_buf[(trn_buf_ptr)++] = '\t';  numberToBufferString(ph_ph.Ph_BR.angle.value, trn_buf, &trn_buf_ptr, 2, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = 186;trn_buf[(trn_buf_ptr)++] = '\n';
//  128     stringToBufferString("Vol(V)P-P:\t",trn_buf,&trn_buf_ptr); 
//  129     numberToBufferString(ph_ph.Ph_RY.vol.value, trn_buf, &trn_buf_ptr, 2, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';trn_buf[(trn_buf_ptr)++] = '\t';  numberToBufferString(ph_ph.Ph_YB.vol.value, trn_buf, &trn_buf_ptr, 2, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';trn_buf[(trn_buf_ptr)++] = '\t';  numberToBufferString(ph_ph.Ph_BR.vol.value, trn_buf, &trn_buf_ptr, 2, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\n';
//  130     stringToBufferString("Quadrant:\t",trn_buf,&trn_buf_ptr); 
//  131     numberToBufferString(quadrant.Rph, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';trn_buf[(trn_buf_ptr)++] = '\t';numberToBufferString(quadrant.Yph, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';trn_buf[(trn_buf_ptr)++] = '\t';numberToBufferString(quadrant.Bph, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';trn_buf[(trn_buf_ptr)++] = '\t';numberToBufferString(quadrant.Allph, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\n'; 
//  132     stringToBufferString("Two wire \t\t",trn_buf,&trn_buf_ptr); 
//  133     numberToBufferString(flag_metro_two_wire, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\n';
//  134 #endif
//  135 #if 0
//  136     stringToBufferString("ACT_IMP(Wh):\t",trn_buf,&trn_buf_ptr); numberToBufferString(get_hr_energy(energy.Allph.active_imp,energy.Allph.active_imp_pulse), trn_buf, &trn_buf_ptr, 6, UNSIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\n';
//  137     stringToBufferString("ACT_EXP(Wh):\t",trn_buf,&trn_buf_ptr); numberToBufferString(get_hr_energy(energy.Allph.active_exp,energy.Allph.active_exp_pulse), trn_buf, &trn_buf_ptr, 6, UNSIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\n';
//  138     stringToBufferString("APP_IMP(Wh):\t",trn_buf,&trn_buf_ptr); numberToBufferString(get_hr_energy(energy.Allph.apparent_imp,energy.Allph.apparent_imp_pulse), trn_buf, &trn_buf_ptr, 6, UNSIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\n';
//  139     stringToBufferString("APP_EXP(Wh):\t",trn_buf,&trn_buf_ptr); numberToBufferString(get_hr_energy(energy.Allph.apparent_exp,energy.Allph.apparent_exp_pulse), trn_buf, &trn_buf_ptr, 6, UNSIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\n';
//  140     stringToBufferString("REACT_Q1(Wh):\t",trn_buf,&trn_buf_ptr);numberToBufferString(get_hr_energy(energy.Allph.reactive_q1,energy.Allph.reactive_q1_pulse), trn_buf, &trn_buf_ptr, 6, UNSIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\n';
//  141     stringToBufferString("REACT_Q2(Wh):\t",trn_buf,&trn_buf_ptr);numberToBufferString(get_hr_energy(energy.Allph.reactive_q2,energy.Allph.reactive_q2_pulse), trn_buf, &trn_buf_ptr, 6, UNSIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\n';
//  142     stringToBufferString("REACT_Q3(Wh):\t",trn_buf,&trn_buf_ptr);numberToBufferString(get_hr_energy(energy.Allph.reactive_q3,energy.Allph.reactive_q3_pulse), trn_buf, &trn_buf_ptr, 6, UNSIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\n';
//  143     stringToBufferString("REACT_Q4(Wh):\t",trn_buf,&trn_buf_ptr);numberToBufferString(get_hr_energy(energy.Allph.reactive_q4,energy.Allph.reactive_q4_pulse), trn_buf, &trn_buf_ptr, 6, UNSIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\n';
//  144 #endif
//  145     
//  146 #if 0    
//  147     stringToBufferString("Samples:\t",trn_buf,&trn_buf_ptr); 
//  148     numberToBufferString(r_phase.no_of_samples, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';trn_buf[(trn_buf_ptr)++] = '\t'; numberToBufferString(y_phase.no_of_samples, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';trn_buf[(trn_buf_ptr)++] = '\t'; numberToBufferString(b_phase.no_of_samples, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';trn_buf[(trn_buf_ptr)++] = '\t';numberToBufferString(n_phase.no_of_samples, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';trn_buf[(trn_buf_ptr)++] = '\t';  numberToBufferString(samples_per_sec, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\n';
//  149 #endif
//  150     
//  151 #if 0    
//  152     stringToBufferString("AC:",trn_buf,&trn_buf_ptr);numberToBufferString(flag_tlv_ac_field, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = ',';
//  153     stringToBufferString("Bx: ",trn_buf,&trn_buf_ptr);numberToBufferString(tlv.Bx, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = ',';
//  154     stringToBufferString("  By: ",trn_buf,&trn_buf_ptr);numberToBufferString(tlv.By, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = ',';
//  155     stringToBufferString("  Bz: ",trn_buf,&trn_buf_ptr);numberToBufferString(tlv.Bz, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = ',';
//  156     stringToBufferString("  Bnet: ",trn_buf,&trn_buf_ptr);numberToBufferString(tlv.Bnet, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = ',';
//  157     stringToBufferString("  BnetAVG: ",trn_buf,&trn_buf_ptr);numberToBufferString(tlv.Bnet_avg, trn_buf, &trn_buf_ptr, 1, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = ',';
//  158     stringToBufferString("  BnetRMS: ",trn_buf,&trn_buf_ptr);numberToBufferString(tlv.Bnet_rms, trn_buf, &trn_buf_ptr, 1, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = ',';
//  159     stringToBufferString("  Bx_sum: ",trn_buf,&trn_buf_ptr);numberToBufferString(tlv.bx_sum, trn_buf, &trn_buf_ptr, 1, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = ',';
//  160     stringToBufferString("  Bx_diff: ",trn_buf,&trn_buf_ptr);numberToBufferString(tlv.bx_diff, trn_buf, &trn_buf_ptr, 1, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = ',';
//  161     stringToBufferString("  By_sum: ",trn_buf,&trn_buf_ptr);numberToBufferString(tlv.by_sum, trn_buf, &trn_buf_ptr, 1, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = ',';
//  162     stringToBufferString("  By_diff: ",trn_buf,&trn_buf_ptr);numberToBufferString(tlv.by_diff, trn_buf, &trn_buf_ptr, 1, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = ',';
//  163     stringToBufferString("  Bz_sum: ",trn_buf,&trn_buf_ptr);numberToBufferString(tlv.bz_sum, trn_buf, &trn_buf_ptr, 1, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = ',';
//  164     stringToBufferString("  Bz_diff: ",trn_buf,&trn_buf_ptr);numberToBufferString(tlv.bz_diff, trn_buf, &trn_buf_ptr, 1, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = ',';
//  165     stringToBufferString("  BnetAVGMAX: ",trn_buf,&trn_buf_ptr);numberToBufferString(tlv.Bnet_avg_max, trn_buf, &trn_buf_ptr, 1, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = ',';
//  166     numberToBufferString(tlv.adc_hang_cnt_total, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = ',';
//  167     numberToBufferString(tlv.fault_detect_count, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);
//  168 #endif
//  169     
//  170 #if 0
//  171     stringToBufferString("TPR_instant:\t",trn_buf,&trn_buf_ptr); 
//  172     numberToBufferString(tamper_instant_status, trn_buf, &trn_buf_ptr, 0, UNSIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\n';
//  173 #endif
//  174     
//  175 #if 0
//  176     stringToBufferString("cal_done_f\t\t",trn_buf,&trn_buf_ptr); 
//  177     numberToBufferString(cal_done_f, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\n';
//  178     stringToBufferString("Cal V:\t\t",trn_buf,&trn_buf_ptr); 
//  179     numberToBufferString(cal_coeff.Rph.vol, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';trn_buf[(trn_buf_ptr)++] = '\t'; numberToBufferString(cal_coeff.Yph.vol, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';trn_buf[(trn_buf_ptr)++] = '\t'; numberToBufferString(cal_coeff.Bph.vol, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\n';
//  180     stringToBufferString("Cal I:\t\t",trn_buf,&trn_buf_ptr); 
//  181     numberToBufferString(cal_coeff.Rph.curr, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';trn_buf[(trn_buf_ptr)++] = '\t'; numberToBufferString(cal_coeff.Yph.curr, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';trn_buf[(trn_buf_ptr)++] = '\t'; numberToBufferString(cal_coeff.Bph.curr, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';trn_buf[(trn_buf_ptr)++] = '\t'; numberToBufferString(cal_coeff.Nph.curr, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\n';
//  182     stringToBufferString("Cal P:\t\t",trn_buf,&trn_buf_ptr); 
//  183     numberToBufferString(cal_coeff.Rph.power, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';trn_buf[(trn_buf_ptr)++] = '\t'; numberToBufferString(cal_coeff.Yph.power, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';trn_buf[(trn_buf_ptr)++] = '\t'; numberToBufferString(cal_coeff.Bph.power, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\n';
//  184     stringToBufferString("Cal Phase:\t",trn_buf,&trn_buf_ptr); 
//  185     numberToBufferString(cal_coeff.Rph.phase_correction, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';trn_buf[(trn_buf_ptr)++] = '\t'; numberToBufferString(cal_coeff.Yph.phase_correction, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';trn_buf[(trn_buf_ptr)++] = '\t'; numberToBufferString(cal_coeff.Bph.phase_correction, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\n';
//  186 #endif
//  187     
//  188 #if 0
//  189     stringToBufferString("Ang Act R,Y,B:  \t",trn_buf,&trn_buf_ptr);numberToBufferString(cal_RPh.angle_active, trn_buf, &trn_buf_ptr, 2, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = ',';numberToBufferString(cal_YPh.angle_active, trn_buf, &trn_buf_ptr, 2, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = ',';numberToBufferString(cal_BPh.angle_active, trn_buf, &trn_buf_ptr, 2, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\n';
//  190     stringToBufferString("Ang React R,Y,B:\t",trn_buf,&trn_buf_ptr);numberToBufferString(cal_RPh.angle_reactive, trn_buf, &trn_buf_ptr, 2, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = ',';numberToBufferString(cal_YPh.angle_reactive, trn_buf, &trn_buf_ptr, 2, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = ',';numberToBufferString(cal_BPh.angle_reactive, trn_buf, &trn_buf_ptr, 2, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\n';
//  191 //    numberToBufferString(cal_BPh.l_shift.active.power, trn_buf, &trn_buf_ptr, 1, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';trn_buf[(trn_buf_ptr)++] = '\t'; numberToBufferString(cal_BPh.r_shift.active.power, trn_buf, &trn_buf_ptr, 1, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';trn_buf[(trn_buf_ptr)++] = '\t'; numberToBufferString(cal_BPh.angle_active, trn_buf, &trn_buf_ptr, 2, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = 186;trn_buf[(trn_buf_ptr)++] = '\n';
//  192 //    
//  193 //    numberToBufferString(cal_RPh.l_shift.reactive.power, trn_buf, &trn_buf_ptr, 1, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';trn_buf[(trn_buf_ptr)++] = '\t'; numberToBufferString(cal_RPh.r_shift.reactive.power, trn_buf, &trn_buf_ptr, 1, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';trn_buf[(trn_buf_ptr)++] = '\t'; numberToBufferString(cal_RPh.angle_reactive-9000, trn_buf, &trn_buf_ptr, 2, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = 186;trn_buf[(trn_buf_ptr)++] = '\n';
//  194 //    numberToBufferString(cal_YPh.l_shift.reactive.power, trn_buf, &trn_buf_ptr, 1, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';trn_buf[(trn_buf_ptr)++] = '\t'; numberToBufferString(cal_YPh.r_shift.reactive.power, trn_buf, &trn_buf_ptr, 1, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';trn_buf[(trn_buf_ptr)++] = '\t'; numberToBufferString(cal_YPh.angle_reactive-9000, trn_buf, &trn_buf_ptr, 2, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = 186;trn_buf[(trn_buf_ptr)++] = '\n';
//  195 //    numberToBufferString(cal_BPh.l_shift.reactive.power, trn_buf, &trn_buf_ptr, 1, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';trn_buf[(trn_buf_ptr)++] = '\t'; numberToBufferString(cal_BPh.r_shift.reactive.power, trn_buf, &trn_buf_ptr, 1, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';trn_buf[(trn_buf_ptr)++] = '\t'; numberToBufferString(cal_BPh.angle_reactive-9000, trn_buf, &trn_buf_ptr, 2, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = 186;trn_buf[(trn_buf_ptr)++] = '\n';
//  196 #endif   
//  197 #if 0
//  198     stringToBufferString("delta Act:\t",trn_buf,&trn_buf_ptr);
//  199     numberToBufferString(r_phase.active.delta_cnts, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  200     numberToBufferString(y_phase.active.delta_cnts, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  201     numberToBufferString(b_phase.active.delta_cnts, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  202     numberToBufferString(all_phase.active.delta_cnts, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  203     stringToBufferString("delta React:\t",trn_buf,&trn_buf_ptr);
//  204     numberToBufferString(r_phase.reactive.delta_cnts, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  205     numberToBufferString(y_phase.reactive.delta_cnts, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  206     numberToBufferString(b_phase.reactive.delta_cnts, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  207     numberToBufferString(all_phase.reactive.delta_cnts, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  208     stringToBufferString("delta App:\t",trn_buf,&trn_buf_ptr);
//  209     numberToBufferString(r_phase.apparent.delta_cnts, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  210     numberToBufferString(y_phase.apparent.delta_cnts, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  211     numberToBufferString(b_phase.apparent.delta_cnts, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  212     numberToBufferString(all_phase.apparent.delta_cnts, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  213 #endif 
//  214 #if 0 
//  215     stringToBufferString("THD Data Vol:\t",trn_buf,&trn_buf_ptr); 
//  216     numberToBufferString(vol.Rph.rms, trn_buf, &trn_buf_ptr, 2, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  217     numberToBufferString(thd.Rph.vol.rms_cycle, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  218     numberToBufferString(thd.Rph.vol.rms_funda, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  219     numberToBufferString(thd.Rph.vol.value, trn_buf, &trn_buf_ptr, 1, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  220     
//  221     numberToBufferString(vol.Yph.rms, trn_buf, &trn_buf_ptr, 2, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  222     numberToBufferString(thd.Yph.vol.rms_cycle, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  223     numberToBufferString(thd.Yph.vol.rms_funda, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  224     numberToBufferString(thd.Yph.vol.value, trn_buf, &trn_buf_ptr, 1, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  225    
//  226     numberToBufferString(vol.Bph.rms, trn_buf, &trn_buf_ptr, 2, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  227     numberToBufferString(thd.Bph.vol.rms_cycle, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  228     numberToBufferString(thd.Bph.vol.rms_funda, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  229     numberToBufferString(thd.Bph.vol.value, trn_buf, &trn_buf_ptr, 1, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  230     
//  231     trn_buf[(trn_buf_ptr)++] = '\n';
//  232     
//  233     stringToBufferString("THD Data Curr:\t",trn_buf,&trn_buf_ptr); 
//  234     numberToBufferString(curr.Rph.rms, trn_buf, &trn_buf_ptr, 3, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  235     numberToBufferString(thd.Rph.curr.rms_cycle, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  236     numberToBufferString(thd.Rph.curr.rms_funda, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  237     numberToBufferString(thd.Rph.curr.value, trn_buf, &trn_buf_ptr, 1, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  238     
//  239     numberToBufferString(curr.Yph.rms, trn_buf, &trn_buf_ptr, 3, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  240     numberToBufferString(thd.Yph.curr.rms_cycle, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  241     numberToBufferString(thd.Yph.curr.rms_funda, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  242     numberToBufferString(thd.Yph.curr.value, trn_buf, &trn_buf_ptr, 1, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  243     
//  244     numberToBufferString(curr.Bph.rms, trn_buf, &trn_buf_ptr, 3, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  245     numberToBufferString(thd.Bph.curr.rms_cycle, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  246     numberToBufferString(thd.Bph.curr.rms_funda, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  247     numberToBufferString(thd.Bph.curr.value, trn_buf, &trn_buf_ptr, 1, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  248     
//  249     trn_buf[(trn_buf_ptr)++] = '\n';
//  250     
//  251     
//  252     numberToBufferString(thd.Rph.correction, trn_buf, &trn_buf_ptr, 1, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  253     numberToBufferString(thd.Yph.correction, trn_buf, &trn_buf_ptr, 1, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  254     numberToBufferString(thd.Bph.correction, trn_buf, &trn_buf_ptr, 1, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  255     
//  256     trn_buf[(trn_buf_ptr)++] = '\n';
//  257 #endif
//  258     
//  259 #if 0
//  260     numberToBufferString(get_hr_energy(energy.Allph.active_imp,energy.Allph.active_imp_pulse), trn_buf, &trn_buf_ptr, 6, UNSIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  261     numberToBufferString(get_hr_energy(energy.Allph.active_exp,energy.Allph.active_exp_pulse), trn_buf, &trn_buf_ptr, 6, UNSIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  262     numberToBufferString(get_hr_energy(energy.Allph.apparent_imp,energy.Allph.apparent_imp_pulse), trn_buf, &trn_buf_ptr, 6, UNSIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  263     numberToBufferString(get_hr_energy(energy.Allph.apparent_exp,energy.Allph.apparent_exp_pulse), trn_buf, &trn_buf_ptr, 6, UNSIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  264     numberToBufferString(get_hr_energy(energy.Allph.reactive_q1,energy.Allph.reactive_q1_pulse), trn_buf, &trn_buf_ptr, 6, UNSIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  265     numberToBufferString(get_hr_energy(energy.Allph.reactive_q2,energy.Allph.reactive_q2_pulse), trn_buf, &trn_buf_ptr, 6, UNSIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  266     numberToBufferString(get_hr_energy(energy.Allph.reactive_q3,energy.Allph.reactive_q3_pulse), trn_buf, &trn_buf_ptr, 6, UNSIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  267     numberToBufferString(get_hr_energy(energy.Allph.reactive_q4,energy.Allph.reactive_q4_pulse), trn_buf, &trn_buf_ptr, 6, UNSIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  268     numberToBufferString(get_hr_energy(energy.Allph.fundamental,energy.Allph.fundamental_pulse), trn_buf, &trn_buf_ptr, 6, UNSIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  269     
//  270     
//  271 #endif
//  272 #if 0
//  273     trn_buf[(trn_buf_ptr)++] = '\t';
//  274     numberToBufferString(power.Rph.active_signed, trn_buf, &trn_buf_ptr, 1, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  275     numberToBufferString(power.Yph.active_signed, trn_buf, &trn_buf_ptr, 1, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  276     numberToBufferString(power.Bph.active_signed, trn_buf, &trn_buf_ptr, 1, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  277     numberToBufferString(power.Allph.active_signed, trn_buf, &trn_buf_ptr, 1, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  278     numberToBufferString(power.Rph.reactive_signed, trn_buf, &trn_buf_ptr, 1, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  279     numberToBufferString(power.Yph.reactive_signed, trn_buf, &trn_buf_ptr, 1, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  280     numberToBufferString(power.Bph.reactive_signed, trn_buf, &trn_buf_ptr, 1, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  281     numberToBufferString(power.Allph.reactive_signed, trn_buf, &trn_buf_ptr, 1, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  282     numberToBufferString(power.Rph.apparent, trn_buf, &trn_buf_ptr, 1, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  283     numberToBufferString(power.Yph.apparent, trn_buf, &trn_buf_ptr, 1, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  284     numberToBufferString(power.Bph.apparent, trn_buf, &trn_buf_ptr, 1, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  285     numberToBufferString(power.Allph.apparent, trn_buf, &trn_buf_ptr, 1, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  286     numberToBufferString(quadrant.Rph, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  287     numberToBufferString(quadrant.Yph, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  288     numberToBufferString(quadrant.Bph, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  289     numberToBufferString(quadrant.Allph, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  290     numberToBufferString(vol.Rph.rms, trn_buf, &trn_buf_ptr, 2, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  291     numberToBufferString(vol.Yph.rms, trn_buf, &trn_buf_ptr, 2, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  292     numberToBufferString(vol.Bph.rms, trn_buf, &trn_buf_ptr, 2, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  293     numberToBufferString(curr.Rph.rms, trn_buf, &trn_buf_ptr, 3, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  294     numberToBufferString(curr.Yph.rms, trn_buf, &trn_buf_ptr, 3, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  295     numberToBufferString(curr.Bph.rms, trn_buf, &trn_buf_ptr, 3, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  296     numberToBufferString(r_phase.no_of_samples, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  297     numberToBufferString(y_phase.no_of_samples, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  298     numberToBufferString(b_phase.no_of_samples, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  299     numberToBufferString(samples_per_sec, trn_buf, &trn_buf_ptr, 0, SIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  300     numberToBufferString(get_hr_energy(energy.Allph.active_imp,energy.Allph.active_imp_pulse), trn_buf, &trn_buf_ptr, 0, UNSIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  301     numberToBufferString(get_hr_energy(energy.Allph.active_exp,energy.Allph.active_exp_pulse), trn_buf, &trn_buf_ptr, 0, UNSIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  302     numberToBufferString(get_hr_energy(energy.Allph.apparent_imp,energy.Allph.apparent_imp_pulse), trn_buf, &trn_buf_ptr, 0, UNSIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  303     numberToBufferString(get_hr_energy(energy.Allph.apparent_exp,energy.Allph.apparent_exp_pulse), trn_buf, &trn_buf_ptr, 0, UNSIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  304     numberToBufferString(get_hr_energy(energy.Allph.reactive_q1,energy.Allph.reactive_q1_pulse), trn_buf, &trn_buf_ptr, 0, UNSIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  305     numberToBufferString(get_hr_energy(energy.Allph.reactive_q2,energy.Allph.reactive_q2_pulse), trn_buf, &trn_buf_ptr, 0, UNSIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  306     numberToBufferString(get_hr_energy(energy.Allph.reactive_q3,energy.Allph.reactive_q3_pulse), trn_buf, &trn_buf_ptr, 0, UNSIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  307     numberToBufferString(get_hr_energy(energy.Allph.reactive_q4,energy.Allph.reactive_q4_pulse), trn_buf, &trn_buf_ptr, 0, UNSIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  308     numberToBufferString(get_hr_energy(energy.Rph.active_imp,energy.Rph.active_imp_pulse), trn_buf, &trn_buf_ptr, 0, UNSIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  309     numberToBufferString(get_hr_energy(energy.Rph.active_exp,energy.Rph.active_exp_pulse), trn_buf, &trn_buf_ptr, 0, UNSIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  310     numberToBufferString(get_hr_energy(energy.Yph.active_imp,energy.Yph.active_imp_pulse), trn_buf, &trn_buf_ptr, 0, UNSIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  311     numberToBufferString(get_hr_energy(energy.Yph.active_exp,energy.Yph.active_exp_pulse), trn_buf, &trn_buf_ptr, 0, UNSIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  312     numberToBufferString(get_hr_energy(energy.Bph.active_imp,energy.Bph.active_imp_pulse), trn_buf, &trn_buf_ptr, 0, UNSIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  313     numberToBufferString(get_hr_energy(energy.Bph.active_exp,energy.Bph.active_exp_pulse), trn_buf, &trn_buf_ptr, 0, UNSIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  314     numberToBufferString(get_hr_energy(energy.Allph.fundamental,energy.Allph.fundamental_pulse), trn_buf, &trn_buf_ptr, 0, UNSIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  315     //numberToBufferString(duplicate_total_apparent_energy, trn_buf, &trn_buf_ptr, 3, UNSIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  316     numberToBufferString(energy.Allph.zkwh_imp, trn_buf, &trn_buf_ptr, 3, UNSIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  317     numberToBufferString(energy.Allph.zkwh_exp, trn_buf, &trn_buf_ptr, 3, UNSIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  318     numberToBufferString(energy.Allph.zkvah_imp, trn_buf, &trn_buf_ptr, 3, UNSIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  319     numberToBufferString(energy.Allph.zkvah_exp, trn_buf, &trn_buf_ptr, 3, UNSIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  320     numberToBufferString(energy.Allph.zkvarh_q1, trn_buf, &trn_buf_ptr, 3, UNSIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  321     numberToBufferString(energy.Allph.zkvarh_q2, trn_buf, &trn_buf_ptr, 3, UNSIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  322     numberToBufferString(energy.Allph.zkvarh_q3, trn_buf, &trn_buf_ptr, 3, UNSIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  323     numberToBufferString(energy.Allph.zkvarh_q4, trn_buf, &trn_buf_ptr, 3, UNSIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  324     //numberToBufferString(e2416_byte, trn_buf, &trn_buf_ptr, 0, UNSIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  325     //numberToBufferString(e2416_q2_byte, trn_buf, &trn_buf_ptr, 0, UNSIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  326     numberToBufferString(get_hr_energy(energy.Allph.defraud_mag,0), trn_buf, &trn_buf_ptr, 0, UNSIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  327     numberToBufferString(get_hr_energy(energy.Rph.defraud_mag,0), trn_buf, &trn_buf_ptr, 0, UNSIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  328     numberToBufferString(get_hr_energy(energy.Yph.defraud_mag,0), trn_buf, &trn_buf_ptr, 0, UNSIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  329     numberToBufferString(get_hr_energy(energy.Bph.defraud_mag,0), trn_buf, &trn_buf_ptr, 0, UNSIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  330     numberToBufferString(r_phase.active.delta_cnts, trn_buf, &trn_buf_ptr, 0, UNSIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  331     numberToBufferString(y_phase.active.delta_cnts, trn_buf, &trn_buf_ptr, 0, UNSIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  332     numberToBufferString(b_phase.active.delta_cnts, trn_buf, &trn_buf_ptr, 0, UNSIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  333     numberToBufferString(all_phase.active.delta_cnts, trn_buf, &trn_buf_ptr, 0, UNSIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  334     numberToBufferString(r_phase.reactive.delta_cnts, trn_buf, &trn_buf_ptr, 0, UNSIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  335     numberToBufferString(y_phase.reactive.delta_cnts, trn_buf, &trn_buf_ptr, 0, UNSIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  336     numberToBufferString(b_phase.reactive.delta_cnts, trn_buf, &trn_buf_ptr, 0, UNSIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  337     numberToBufferString(all_phase.reactive.delta_cnts, trn_buf, &trn_buf_ptr, 0, UNSIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  338     numberToBufferString(r_phase.apparent.delta_cnts, trn_buf, &trn_buf_ptr, 0, UNSIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  339     numberToBufferString(y_phase.apparent.delta_cnts, trn_buf, &trn_buf_ptr, 0, UNSIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  340     numberToBufferString(b_phase.apparent.delta_cnts, trn_buf, &trn_buf_ptr, 0, UNSIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  341     numberToBufferString(all_phase.apparent.delta_cnts, trn_buf, &trn_buf_ptr, 0, UNSIGNED_MODE);trn_buf[(trn_buf_ptr)++] = '\t';
//  342  
//  343 #else
//  344     trn_buf[(trn_buf_ptr)++] = '\n';
        MOVW      AX, [SP]           ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOV       A, #0xA            ;; 1 cycle
        MOV       (_trn_buf)[BC], A  ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        INCW      AX                 ;; 1 cycle
        MOVW      [SP], AX           ;; 1 cycle
        BR        S:??debug_data_send_1  ;; 3 cycles
        ; ------------------------------------- Block: 10 cycles
//  345 #endif
//  346   }
//  347   else if(debug_data_type == '1' || debug_data_type == '2' || debug_data_type == '3')
??debug_data_send_0:
        CMP       N:_debug_data_type, #0x31  ;; 1 cycle
        BZ        ??debug_data_send_2  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        CMP       N:_debug_data_type, #0x32  ;; 1 cycle
        BZ        ??debug_data_send_2  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        CMP       N:_debug_data_type, #0x33  ;; 1 cycle
        BNZ       ??debug_data_send_1  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  348   {
//  349     /* Voltage and current signal */
//  350     for(us8 index  = 0; index <80; index++)
??debug_data_send_2:
        MOV       D, #0x0            ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??debug_data_send_3:
        MOV       A, D               ;; 1 cycle
        CMP       A, #0x50           ;; 1 cycle
        BNC       ??debug_data_send_1  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  351     {
//  352 #if DEBUG_MODE == 1
//  353         numberToBufferString(samples_vol[index], trn_buf, &trn_buf_ptr, 0, SIGNED_MODE); trn_buf[(trn_buf_ptr)++] = '\t';
//  354         //numberToBufferString(samples_vol90[index], trn_buf, &trn_buf_ptr, 0, SIGNED_MODE); trn_buf[(trn_buf_ptr)++] = '\t';
//  355         numberToBufferString(samples_curr[index], trn_buf, &trn_buf_ptr, 0, SIGNED_MODE); trn_buf[(trn_buf_ptr)++] = '\t';
//  356         samples_vol[index] = 0;
//  357         samples_vol90[index] = 0;
//  358         samples_curr[index] = 0;
//  359 #endif
//  360         trn_buf[(trn_buf_ptr)++] = '0'; trn_buf[(trn_buf_ptr)++] = '\n';
        MOVW      AX, [SP]           ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOV       A, #0x30           ;; 1 cycle
        MOV       (_trn_buf)[BC], A  ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        INCW      BC                 ;; 1 cycle
        MOV       A, #0xA            ;; 1 cycle
        MOV       (_trn_buf)[BC], A  ;; 1 cycle
        INCW      BC                 ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [SP], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  361     }
        INC       D                  ;; 1 cycle
        BR        S:??debug_data_send_3  ;; 3 cycles
        ; ------------------------------------- Block: 17 cycles
//  362   }
//  363   
//  364   trn_cnt = 0;
??debug_data_send_1:
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_trn_cnt, AX     ;; 1 cycle
//  365   req_cnt = trn_buf_ptr;
        MOVW      AX, [SP]           ;; 1 cycle
        MOVW      N:_req_cnt, AX     ;; 1 cycle
//  366   optical_f = 1;
        MOV       N:_optical_f, #0x1  ;; 1 cycle
//  367   send_data_to_uart();
          CFI FunCall _send_data_to_uart
        CALL      _send_data_to_uart  ;; 3 cycles
//  368 }
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock0
        ; ------------------------------------- Block: 15 cycles
        ; ------------------------------------- Total: 77 cycles

        SECTION `.iar_vfe_header`:DATA:NOALLOC:NOROOT(1)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
        DC32 0

        END
//  369 
// 
//   2 bytes in section .bss
// 107 bytes in section .text
// 
// 107 bytes of CODE memory
//   2 bytes of DATA memory
//
//Errors: none
//Warnings: none
