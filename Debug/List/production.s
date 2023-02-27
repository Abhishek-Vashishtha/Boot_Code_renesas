///////////////////////////////////////////////////////////////////////////////
//
// IAR C/C++ Compiler V4.20.1.2260 for RL78               20/Dec/2020  22:38:39
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
//        BootCode\source_code\source_files\production.c
//    Command line       =
//        -f C:\Users\laptop\AppData\Local\Temp\EWAB90.tmp ("D:\Dheeraj\New
//        folder\0. GDEV72 - BootCode\source_code\source_files\production.c"
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
//        D:\Dheeraj\New folder\0. GDEV72 - BootCode\Debug\List\production.s
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

        EXTERN _disp_test_pkt_flag
        EXTERN _data_array
        EXTERN _flag_lcd
        EXTERN __lcd_msg_disp_timer
        EXTERN _info
        EXTERN _char_array_to_int
        EXTERN _checkBCC
        EXTERN _flag_key_press_dn
        EXTERN _flag_key_press_md_reset
        EXTERN _flag_key_press_top_cover
        EXTERN _flag_key_press_up
        EXTERN _k
        EXTERN _lcd_write_msg

        PUBLIC _MemoryStatus1
        PUBLIC _MemoryStatus2
        PUBLIC __A_SDR10
        PUBLIC _disp_test_id
        PUBLIC _hardware_testing_status
        PUBLIC _production_loop
        PUBLIC _push_button_count
        PUBLIC _push_count_down
        PUBLIC _push_count_md_reset
        PUBLIC _push_count_tc
        PUBLIC _push_count_up
        
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
        
// D:\Dheeraj\New folder\0. GDEV72 - BootCode\source_code\source_files\production.c
//    1 /***********************************************************************************************************************
//    2 * File Name       : production.c
//    3 * Current Version : rev_01  
//    4 * Tool-Chain      : IAR Systems RL78
//    5 * Description     : This file has the code to aid the manufacturing process
//    6 * Creation Date   : 04-06-2020
//    7 * Company         : Genus Power Infrastructures Limited, Jaipur
//    8 * Author          : dheeraj.singhal
//    9 * Version History : rev_01 :
//   10 ***********************************************************************************************************************/
//   11 /************************************ Includes **************************************/
//   12 #include "production.h"

        ASEGN `.bss.noinit`:DATA:NOROOT,0fff48H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_SDR10
// __no_init union <unnamed>#71 volatile __sfr __no_bit_access _A_SDR10
__A_SDR10:
        DS 2
//   13 /************************************ Local Variables *****************************************/
//   14 /************************************ Extern Variables *****************************************/

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   15 us8 push_count_up,push_count_down,push_count_tc,push_count_md_reset;
_push_count_up:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_push_count_down:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_push_count_tc:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_push_count_md_reset:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   16 us16 disp_test_id;
_disp_test_id:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   17 us8 MemoryStatus1,MemoryStatus2;
_MemoryStatus1:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_MemoryStatus2:
        DS 1
//   18 
//   19 /************************************ Local Functions *******************************/
//   20 /************************************ Extern Functions ******************************/
//   21 void push_button_count();
//   22 void production_loop();
//   23 void hardware_testing_status(us8 Switch_PortPin);
//   24 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock0 Using cfiCommon0
          CFI Function _push_button_count
          CFI NoCalls
        CODE
//   25 void push_button_count()
//   26 {
_push_button_count:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//   27   if(button_up_f == 1)
        MOVW      HL, #LWRD(_flag_key_press_up)  ;; 1 cycle
        MOV1      CY, [HL].1         ;; 1 cycle
        BNC       ??hardware_testing_status_0  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//   28   {
//   29     if(push_count_up < 250)
        MOV       A, N:_push_count_up  ;; 1 cycle
        CMP       A, #0xFA           ;; 1 cycle
        SKNC                         ;; 1 cycle
        ; ------------------------------------- Block: 3 cycles
//   30     {
//   31       push_count_up++;
        INC       N:_push_count_up   ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
//   32     }
//   33   }
//   34   if(button_dn_f == 1)
??hardware_testing_status_0:
        MOVW      HL, #LWRD(_flag_key_press_dn)  ;; 1 cycle
        MOV1      CY, [HL].1         ;; 1 cycle
        BNC       ??hardware_testing_status_1  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//   35   {
//   36     if(push_count_down < 250)
        MOV       A, N:_push_count_down  ;; 1 cycle
        CMP       A, #0xFA           ;; 1 cycle
        SKNC                         ;; 1 cycle
        ; ------------------------------------- Block: 3 cycles
//   37     {
//   38       push_count_down++;
        INC       N:_push_count_down  ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
//   39     }
//   40   }
//   41   if(button_md_reset_f == 1)
??hardware_testing_status_1:
        MOVW      HL, #LWRD(_flag_key_press_md_reset)  ;; 1 cycle
        MOV1      CY, [HL].1         ;; 1 cycle
        BNC       ??hardware_testing_status_2  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//   42   {
//   43     if(push_count_md_reset < 250)
        MOV       A, N:_push_count_md_reset  ;; 1 cycle
        CMP       A, #0xFA           ;; 1 cycle
        SKNC                         ;; 1 cycle
        ; ------------------------------------- Block: 3 cycles
//   44     {
//   45       push_count_md_reset++;
        INC       N:_push_count_md_reset  ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
//   46     }
//   47   }
//   48   if(button_top_cover_f == 1)
??hardware_testing_status_2:
        MOVW      HL, #LWRD(_flag_key_press_top_cover)  ;; 1 cycle
        MOV1      CY, [HL].1         ;; 1 cycle
        BNC       ??hardware_testing_status_3  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//   49   {
//   50     if(push_count_tc < 250)
        MOV       A, N:_push_count_tc  ;; 1 cycle
        CMP       A, #0xFA           ;; 1 cycle
        SKNC                         ;; 1 cycle
        ; ------------------------------------- Block: 3 cycles
//   51     {
//   52       push_count_tc++;
        INC       N:_push_count_tc   ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
//   53     }
//   54   }
//   55 }
??hardware_testing_status_3:
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock0
        ; ------------------------------------- Block: 6 cycles
        ; ------------------------------------- Total: 50 cycles
//   56 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock1 Using cfiCommon0
          CFI Function _production_loop
        CODE
//   57 void production_loop()
//   58 {
_production_loop:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//   59   if(disp_test_pkt_flag == 1)
        CMP       N:_disp_test_pkt_flag, #0x1  ;; 1 cycle
        BNZ       ??hardware_testing_status_4  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//   60   {
//   61     disp_test_pkt_flag = 0;
        MOV       N:_disp_test_pkt_flag, #0x0  ;; 1 cycle
//   62     if(data_array[1] == 0x58 && checkBCC(&data_array[1],6) == data_array[7])
        CMP       N:_data_array+1, #0x58  ;; 1 cycle
        BNZ       ??hardware_testing_status_4  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOV       C, #0x6            ;; 1 cycle
        MOVW      AX, #LWRD(_data_array+1)  ;; 1 cycle
          CFI FunCall _checkBCC
        CALL      _checkBCC          ;; 3 cycles
        CMP       A, N:_data_array+7  ;; 1 cycle
        BNZ       ??hardware_testing_status_4  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
//   63     {
//   64       disp_test_id = char_array_to_int(&data_array[4]);
        MOVW      AX, #LWRD(_data_array+4)  ;; 1 cycle
          CFI FunCall _char_array_to_int
        CALL      _char_array_to_int  ;; 3 cycles
        MOVW      N:_disp_test_id, AX  ;; 1 cycle
//   65       if(disp_test_id == 0xFFFF)
        MOVW      AX, N:_disp_test_id  ;; 1 cycle
        CMPW      AX, #0xFFFF        ;; 1 cycle
        BNZ       ??hardware_testing_status_5  ;; 4 cycles
        ; ------------------------------------- Block: 11 cycles
//   66       {
//   67         if(flag_lcd_msg_disp == 1)
        MOVW      HL, #LWRD(_flag_lcd)  ;; 1 cycle
        MOV1      CY, [HL].1         ;; 1 cycle
        BNC       ??hardware_testing_status_4  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//   68         {
//   69           flag_lcd_msg_disp = 0;
        CLR1      N:_flag_lcd.1      ;; 2 cycles
//   70           _lcd_msg_disp_timer = 0;
        MOV       N:__lcd_msg_disp_timer, #0x0  ;; 1 cycle
        RET                          ;; 6 cycles
        ; ------------------------------------- Block: 9 cycles
//   71         }
//   72       }
//   73       else
//   74       {
//   75         lcd_write_msg(326,240);         /* 4 min */
??hardware_testing_status_5:
        MOV       C, #0xF0           ;; 1 cycle
        MOVW      AX, #0x146         ;; 1 cycle
          CFI FunCall _lcd_write_msg
        CALL      _lcd_write_msg     ;; 3 cycles
//   76         SendOptical(0x06);
        MOV       0xFFF48, #0x6      ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
//   77       }
//   78     }
//   79   }
//   80 }
??hardware_testing_status_4:
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock1
        ; ------------------------------------- Block: 6 cycles
        ; ------------------------------------- Total: 59 cycles
        REQUIRE __A_SDR10

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock2 Using cfiCommon0
          CFI Function _hardware_testing_status
          CFI NoCalls
        CODE
//   81 void hardware_testing_status(us8 Switch_PortPin)
//   82 {
_hardware_testing_status:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
        MOV       X, A               ;; 1 cycle
//   83   if(1==Switch_PortPin)
        XCH       A, X               ;; 1 cycle
        CMP       A, #0x1            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        BNZ       ??hardware_testing_status_6  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
//   84   {
//   85     info[k++] = 0x00;
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
//   86     info[k++] = 0x09;
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, #0x9            ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
//   87     info[k++] = 0x08;
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, #0x8            ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
//   88     info[k++] = 0x00;
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
//   89     info[k++] = 0x00;
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
//   90     info[k++] = 0x00;
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
//   91     info[k++] = push_count_down;
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, N:_push_count_down  ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
//   92     info[k++] = push_count_up;
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, N:_push_count_up  ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
//   93     info[k++] = 0x00; //termc_sw_cnt;
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
//   94     info[k++] = 0x00; //module_sw_cnt;
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
//   95     info[k++] = push_count_tc;
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, N:_push_count_tc  ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
        ; ------------------------------------- Block: 55 cycles
//   96   }
//   97   else if(2==Switch_PortPin)
//   98   {
//   99     //    info[k++]=0x00;
//  100     //    info[k++]=0x09;
//  101     //    info[k++]=0x10;		// For 16 PORTS
//  102     //    for(u8Temp=0;u8Temp<16;u8Temp++)
//  103     //    {
//  104     //      info[k++]=port_status[u8Temp];
//  105     //    }
//  106   }
//  107     
//  108 }
??hardware_testing_status_6:
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock2
        ; ------------------------------------- Block: 6 cycles
        ; ------------------------------------- Total: 69 cycles

        SECTION `.iar_vfe_header`:DATA:NOALLOC:NOROOT(1)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
        DC32 0

        END
// 
//   8 bytes in section .bss
//   2 bytes in section .bss.noinit  (abs)
// 275 bytes in section .text
// 
// 275 bytes of CODE memory
//   8 bytes of DATA memory (+ 2 bytes shared)
//
//Errors: none
//Warnings: none
