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
//        BootCode\source_code\source_files\hdlc.c
//    Command line       =
//        -f C:\Users\DHEERA~1\AppData\Local\Temp\EWBED7.tmp ("E:\0. Dheeraj\0.
//        Official\1. Genus\2. Projects\0. GDEV72 -
//        BootCode\source_code\source_files\hdlc.c" --core s3 --code_model near
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
//        BootCode\Debug\List\hdlc.s
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

        EXTERN _R_SAU1_Create
        EXTERN _R_UART2_Start
        EXTERN _R_UART2_Stop
        EXTERN _R_WDT_Restart

        PUBLIC __A_MK0
        PUBLIC __A_MK1
        PUBLIC __A_SDR02
        PUBLIC __A_SDR10
        PUBLIC _communication_init
        PUBLIC _communication_start
        PUBLIC _communication_stop
        PUBLIC _optical_f
        PUBLIC _rcv_buf
        PUBLIC _rcv_cnt
        PUBLIC _rcv_optical
        PUBLIC _rcv_rj45
        PUBLIC _req_cnt
        PUBLIC _rj_f
        PUBLIC _send_data_to_uart
        PUBLIC _transmit_optical
        PUBLIC _transmit_rj45
        PUBLIC _trn_buf
        PUBLIC _trn_cnt
        
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
        
// E:\0. Dheeraj\0. Official\1. Genus\2. Projects\0. GDEV72 - BootCode\source_code\source_files\hdlc.c
//    1 /***********************************************************************************************************************
//    2 * File Name       : hdlc.c
//    3 * Current Version : rev_01  
//    4 * Tool-Chain      : IAR Systems RL78
//    5 * Description     : This file contains routines to implement HDLC layer for DLMS Implementation
//    6 * Creation Date   : 06-06-2020
//    7 * Company         : Genus Power Infrastructures Limited, Jaipur
//    8 * Author          : dheeraj.singhal
//    9 * Version History : rev_01 :
//   10 ***********************************************************************************************************************/
//   11 /************************************ Includes **************************************/
//   12 #include "hdlc.h"

        ASEGN `.bss.noinit`:DATA:NOROOT,0fff44H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_SDR02
// __no_init union <unnamed>#65 volatile __sfr __no_bit_access _A_SDR02
__A_SDR02:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0fff48H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_SDR10
// __no_init union <unnamed>#71 volatile __sfr __no_bit_access _A_SDR10
__A_SDR10:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0fffe4H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_MK0
// __no_init union <unnamed>#170 volatile __sfr _A_MK0
__A_MK0:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0fffe6H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_MK1
// __no_init union <unnamed>#178 volatile __sfr _A_MK1
__A_MK1:
        DS 2
//   13 /************************************ Local Variables *****************************************/
//   14 
//   15 /************************************ Extern Variables *****************************************/
//   16 /************************************ Local Functions *******************************/
//   17 
//   18 /************************************ Extern Functions ******************************/

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   19 us16 trn_cnt,rcv_cnt,req_cnt;
_trn_cnt:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_rcv_cnt:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_req_cnt:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   20 us8 rcv_buf[300]; 
_rcv_buf:
        DS 300

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   21 us8 trn_buf[300]; 
_trn_buf:
        DS 300
//   22 

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   23 us8 optical_f,rj_f;
_optical_f:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_rj_f:
        DS 1
//   24 
//   25 
//   26 void communication_init();
//   27 void communication_start();
//   28 void communication_stop();
//   29 
//   30 void rcv_optical(us8 rx_char);
//   31 void transmit_optical();
//   32 void rcv_rj45(us8 rx_char);
//   33 void transmit_rj45();
//   34 void send_data_to_uart();
//   35 
//   36 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock0 Using cfiCommon0
          CFI Function _communication_init
          CFI FunCall _R_SAU1_Create
        CODE
//   37 void communication_init()
//   38 {
_communication_init:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//   39   //R_SAU0_Create();              /* RJ45 */
//   40   R_SAU1_Create();              /* Optical port */
        CALL      _R_SAU1_Create     ;; 3 cycles
//   41 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock0
        ; ------------------------------------- Block: 9 cycles
        ; ------------------------------------- Total: 9 cycles
//   42 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock1 Using cfiCommon0
          CFI Function _communication_start
          CFI FunCall _R_UART2_Start
        CODE
//   43 void communication_start()
//   44 {
_communication_start:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//   45   //R_UART1_Start();              /* RJ45 */
//   46   R_UART2_Start();              /* Optical port */
        CALL      _R_UART2_Start     ;; 3 cycles
//   47 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock1
        ; ------------------------------------- Block: 9 cycles
        ; ------------------------------------- Total: 9 cycles
//   48 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock2 Using cfiCommon0
          CFI Function _communication_stop
          CFI FunCall _R_UART2_Stop
        CODE
//   49 void communication_stop()
//   50 {
_communication_stop:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//   51   //R_UART1_Stop();               /* RJ45 */
//   52   R_UART2_Stop();               /* Optical port */
        CALL      _R_UART2_Stop      ;; 3 cycles
//   53 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock2
        ; ------------------------------------- Block: 9 cycles
        ; ------------------------------------- Total: 9 cycles
//   54 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock3 Using cfiCommon0
          CFI Function _rcv_optical
          CFI NoCalls
        CODE
//   55 void rcv_optical(us8 rx_char)
//   56 {
_rcv_optical:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//   57 //  us8 index;
//   58 //  us16 temp_us16 = 0;
//   59 //  
//   60 //  Cntr_2Min= 0;
//   61 //  interframe_timeout= 0;
//   62 //  debug_data_type = rx_char; /* can be commented when diag serial data selectivity not needed */
//   63 //  wdt_restart();                                        /* Pending, Change the implementation. make it like wdt = 1 */
//   64 //  if((rx_char == 0x7e) && (rj_f == 1) && (optical_f == 0))
//   65 //  {
//   66 //    nrm_flag= 0;
//   67 //    cosem_flag= 0;
//   68 //    rrr_s= 0;
//   69 //    rrr_c= 0;
//   70 //    rrr_c1= 0;
//   71 //    sss_c= 0;
//   72 //    sss_c1= 0;
//   73 //    sss_s= 0;
//   74 //    asso0_flag = 0;
//   75 //    asso1_flag = 0;
//   76 //    asso2_flag = 0;
//   77 //    asso3_flag = 0;
//   78 //    asso4_flag = 0;
//   79 //    assoG_flag = 0;
//   80 //    infore_flag= 0;
//   81 //    
//   82 //    for(index= 0; index < 6; index++)
//   83 //    {
//   84 //      obis_code[index]= 0x00;
//   85 //    }
//   86 //    rcv_cnt1= rcv_cnt;
//   87 //    if((rcv_cnt1 != 0) && (rcv_cnt1 < 150))
//   88 //    {
//   89 //      for(index= 0; index < rcv_cnt1; index++)
//   90 //      {
//   91 //        rcv_buf1[index]= rcv_buf[index];
//   92 //      }
//   93 //    }
//   94 //    
//   95 //    rcv_cnt= 0;
//   96 //    buffer_first_not_fill_f= 0;
//   97 //    rj_disc_f= 1;
//   98 //  }
//   99 //  if(((rx_char == 0x7e) || (rx_char == 0x27)) && (optical_f == 0))
//  100 //  {
//  101 //    optical_f= 1;
//  102 //  }
//  103 //  
//  104 //  /* receive count check placed for maximum buffer size to avoid
//  105 //  garbage data filling in rcv_buf array- 30/06/2016- ravi*/
//  106 //  if(rcv_cnt >= (DLMS_MAX_BUFF_SIZE+14))
//  107 //  {
//  108 //    rcv_cnt= 0;
//  109 //  }
//  110 //  
//  111 //  if(1 == optical_f)
//  112 //  {
//  113 //    rj_f= 0;
//  114 //    rx_timeout= 0;
//  115 //    battery_timeout= 0;
//  116 //    rcv_buf[rcv_cnt]= rx_char;
//  117 //    
//  118 //    rcv_cnt++;
//  119 //    if((rcv_buf[0] == 0x27) && (rcv_cnt == 8) && (checkBCC(&rcv_buf[1], 6) == rcv_buf[7]))
//  120 //    {
//  121 //      for(index= 0; index < 8; index++)
//  122 //      {
//  123 //        data_array[index]= rcv_buf[index];
//  124 //      }
//  125 //      rcv_cnt= 0;
//  126 //      analyse_cal_pkt_flag= 1;
//  127 //      disp_test_pkt_flag = 1;
//  128 //    }
//  129 //    
//  130 //    if((rcv_buf[0] == 0x7e) && (rcv_buf[1] == 0x7e))
//  131 //    {
//  132 //      rcv_cnt= 1;
//  133 //    }
//  134 //    
//  135 //    temp_us16 = (rcv_buf[1] & 0x07) * 256 + rcv_buf[2] + 2;
//  136 //    if((rcv_buf[0] == 0x7e) && (rcv_cnt >= 3) && (rcv_cnt == temp_us16 )) /* rcv_buf[2]+2) */
//  137 //    {
//  138 //      frm_rcv_flg= 1;
//  139 //      rcv_cnt= 0;
//  140 //    }
//  141 //    else if((rcv_buf[0] != 0x7e) && (rcv_buf[0] != 0x27) && (rcv_buf[0] != 0x38))
//  142 //    {
//  143 //      rcv_cnt= 0;
//  144 //    }
//  145 //  }
//  146 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock3
        ; ------------------------------------- Block: 6 cycles
        ; ------------------------------------- Total: 6 cycles
//  147 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock4 Using cfiCommon0
          CFI Function _transmit_optical
          CFI FunCall _R_WDT_Restart
        CODE
//  148 void transmit_optical()
//  149 {
_transmit_optical:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  150   wdt_restart();
        CALL      _R_WDT_Restart     ;; 3 cycles
//  151   if(trn_cnt < req_cnt)
        MOVW      HL, N:_req_cnt     ;; 1 cycle
        MOVW      AX, N:_trn_cnt     ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BNC       ??send_data_to_uart_0  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
//  152   {
//  153     trn_cnt++;                                  /* Transmission completed counter +1 */
        INCW      N:_trn_cnt         ;; 2 cycles
//  154     if (trn_cnt == req_cnt)
        MOVW      HL, N:_req_cnt     ;; 1 cycle
        MOVW      AX, N:_trn_cnt     ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BNZ       ??send_data_to_uart_1  ;; 4 cycles
        ; ------------------------------------- Block: 9 cycles
//  155     {
//  156       trn_cnt = 0;                              /* Transmission counter clear */
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_trn_cnt, AX     ;; 1 cycle
//  157       req_cnt = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_req_cnt, AX     ;; 1 cycle
        RET                          ;; 6 cycles
        ; ------------------------------------- Block: 10 cycles
//  158     }
//  159     else
//  160     {
//  161       SendOptical(trn_buf[trn_cnt]);              /* Set transmission data */
??send_data_to_uart_1:
        MOVW      BC, N:_trn_cnt     ;; 1 cycle
        MOV       A, (_trn_buf)[BC]  ;; 1 cycle
        MOV       0xFFF48, A         ;; 1 cycle
        ; ------------------------------------- Block: 3 cycles
//  162     }
//  163   }
//  164 }
??send_data_to_uart_0:
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock4
        ; ------------------------------------- Block: 6 cycles
        ; ------------------------------------- Total: 38 cycles
        REQUIRE __A_SDR10
//  165 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock5 Using cfiCommon0
          CFI Function _rcv_rj45
          CFI NoCalls
        CODE
//  166 void rcv_rj45(us8 rx_char)
//  167 {
_rcv_rj45:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  168 ////  /*  UCHAR ucRxData; */
//  169 ////  uint16_t u16TempData= 0;
//  170 ////  wdt_restart();
//  171 ////  if(optical_f != 1)
//  172 ////  {
//  173 ////    Cntr_2Min= 0;
//  174 ////    interframe_timeout= 0;
//  175 ////    
//  176 ////    rj_f= 1;
//  177 ////    
//  178 ////    rx_timeout= 0;
//  179 ////    battery_timeout= 0;
//  180 ////    /*      if(optical_transparent_f==1) */
//  181 ////    /*      { */
//  182 ////    /*	trn_cnt=0; */
//  183 ////    /*	rcv_cnt=0; */
//  184 ////    /*	SCI1D = SCI3D; */
//  185 ////    /*	SCI1_ENABLE_TXINT(); */
//  186 ////    /*      } */
//  187 ////    /*      else */
//  188 ////    {
//  189 ////      rcv_buf[rcv_cnt]= rx_char;
//  190 ////      rcv_cnt++;
//  191 ////      
//  192 ////      if(rcv_cnt > (DLMS_MAX_BUFF_SIZE + 14))
//  193 ////      {
//  194 ////        rcv_cnt= 0;
//  195 ////      }
//  196 ////      
//  197 ////      if((rcv_buf[0] == 0x7e) && (rcv_buf[1] == 0x7e))
//  198 ////      {
//  199 ////        rcv_cnt= 1;
//  200 ////      }
//  201 ////      
//  202 ////      u16TempData= (rec[1] & 0x07) * 256 + rec[2] + 2;
//  203 ////      if((rcv_buf[0] == 0x7e) && (rcv_cnt >= 3) && (rcv_cnt == u16TempData))
//  204 ////      {
//  205 ////        frm_rcv_flg= 1;
//  206 ////        rcv_cnt= 0;
//  207 ////      }
//  208 ////      else if((rcv_cnt >= 11) && (rcv_buf[2] == '+') && (rcv_buf[3] == 'C') && (rcv_buf[4] == 'S') && (rcv_buf[5] == 'Q') && (rcv_buf[6] == ':'))
//  209 ////      {
//  210 ////        gsm_pkt_flag= 1;
//  211 ////      }
//  212 ////      else if(((rcv_buf[0] != 0x7e) && (gsm_tr_f != 1)) || ((rcv_buf[0] != 0x0d) && (gsm_tr_f == 1)))
//  213 ////      {
//  214 ////        rcv_cnt= 0;
//  215 ////      }
//  216 ////    }
//  217 ////  }
//  218 ////  else if(optical_f == 1) /* && lpr_signon_f==1) */
//  219 ////  {
//  220 ////    rj_disc_f= 1;
//  221 ////    if(rcv_cnt1 >= 150)
//  222 ////    {
//  223 ////      rcv_cnt1= 0;
//  224 ////    }
//  225 ////    rcv_buf1[rcv_cnt1]= rx_char;
//  226 ////    rcv_cnt1++;
//  227 ////    if((rcv_buf1[0] == 0x7e) && (rcv_buf1[1] == 0x7e))
//  228 ////    {
//  229 ////      rcv_cnt1= 1;
//  230 ////    }
//  231 ////    
//  232 ////    u16TempData= (rcv_buf1[1] & 0x07) * 256 + rcv_buf1[2] + 2;
//  233 ////    if((rcv_buf1[0] == 0x7e) && (rcv_cnt1 >= 3) && (rcv_cnt1 == u16TempData))
//  234 ////    {
//  235 ////      /* rj_disc_f = 1; */
//  236 ////      rcv_cnt1= 0;
//  237 ////    }
//  238 ////    else if(rcv_buf1[0] != 0x7e)
//  239 ////    {
//  240 ////      rcv_cnt1= 0;
//  241 ////    }
//  242 ////  }
//  243 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock5
        ; ------------------------------------- Block: 6 cycles
        ; ------------------------------------- Total: 6 cycles
//  244 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock6 Using cfiCommon0
          CFI Function _transmit_rj45
          CFI FunCall _R_WDT_Restart
        CODE
//  245 void transmit_rj45()
//  246 {
_transmit_rj45:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  247   wdt_restart();
        CALL      _R_WDT_Restart     ;; 3 cycles
//  248   if(trn_cnt < req_cnt)
        MOVW      HL, N:_req_cnt     ;; 1 cycle
        MOVW      AX, N:_trn_cnt     ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BNC       ??send_data_to_uart_2  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
//  249   {
//  250     trn_cnt++;                                  /* Transmission completed counter +1 */
        INCW      N:_trn_cnt         ;; 2 cycles
//  251     if (trn_cnt == req_cnt)
        MOVW      HL, N:_req_cnt     ;; 1 cycle
        MOVW      AX, N:_trn_cnt     ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BNZ       ??send_data_to_uart_3  ;; 4 cycles
        ; ------------------------------------- Block: 9 cycles
//  252     {
//  253       trn_cnt = 0;                              /* Transmission counter clear */
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_trn_cnt, AX     ;; 1 cycle
//  254       req_cnt = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_req_cnt, AX     ;; 1 cycle
        RET                          ;; 6 cycles
        ; ------------------------------------- Block: 10 cycles
//  255     }
//  256     else
//  257     {
//  258       SendRJ45(trn_buf[trn_cnt]);              /* Set transmission data */
??send_data_to_uart_3:
        MOVW      BC, N:_trn_cnt     ;; 1 cycle
        MOV       A, (_trn_buf)[BC]  ;; 1 cycle
        MOV       0xFFF44, A         ;; 1 cycle
        ; ------------------------------------- Block: 3 cycles
//  259     }
//  260   }
//  261 }
??send_data_to_uart_2:
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock6
        ; ------------------------------------- Block: 6 cycles
        ; ------------------------------------- Total: 38 cycles
        REQUIRE __A_SDR02

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock7 Using cfiCommon0
          CFI Function _send_data_to_uart
          CFI NoCalls
        CODE
//  262 void send_data_to_uart(void)
//  263 {
_send_data_to_uart:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  264   if(optical_f == 1)
        CMP       N:_optical_f, #0x1  ;; 1 cycle
        BNZ       ??send_data_to_uart_4  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  265   {
//  266     STMK2 = 1U;                                 /* disable INTST2 interrupt */
        SET1      0xFFFE5.0          ;; 2 cycles
//  267     SendOptical(trn_buf[trn_cnt]);              /* Set transmission data */
        MOVW      BC, N:_trn_cnt     ;; 1 cycle
        MOV       A, (_trn_buf)[BC]  ;; 1 cycle
        MOV       0xFFF48, A         ;; 1 cycle
//  268     STMK2 = 0U;                                 /* enable INTST2 interrupt */
        CLR1      0xFFFE5.0          ;; 2 cycles
        RET                          ;; 6 cycles
        ; ------------------------------------- Block: 13 cycles
//  269   }
//  270   else if(rj_f == 1)
??send_data_to_uart_4:
        CMP       N:_rj_f, #0x1      ;; 1 cycle
        BNZ       ??send_data_to_uart_5  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  271   {
//  272     STMK1 = 1U;                                 /* disable INTST2 interrupt */
        SET1      0xFFFE6.1          ;; 2 cycles
//  273     SendRJ45(trn_buf[trn_cnt]);                 /* Set transmission data */
        MOVW      BC, N:_trn_cnt     ;; 1 cycle
        MOV       A, (_trn_buf)[BC]  ;; 1 cycle
        MOV       0xFFF44, A         ;; 1 cycle
//  274     STMK1 = 0U;                                 /* enable INTST2 interrupt */
        CLR1      0xFFFE6.1          ;; 2 cycles
        ; ------------------------------------- Block: 7 cycles
//  275   }
//  276 }
??send_data_to_uart_5:
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock7
        ; ------------------------------------- Block: 6 cycles
        ; ------------------------------------- Total: 36 cycles
        REQUIRE __A_MK0
        REQUIRE __A_SDR10
        REQUIRE __A_MK1
        REQUIRE __A_SDR02

        SECTION `.iar_vfe_header`:DATA:NOALLOC:NOROOT(1)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
        DC32 0

        END
// 
// 608 bytes in section .bss
//   8 bytes in section .bss.noinit  (abs)
// 148 bytes in section .text
// 
// 148 bytes of CODE memory
// 608 bytes of DATA memory (+ 8 bytes shared)
//
//Errors: none
//Warnings: none
