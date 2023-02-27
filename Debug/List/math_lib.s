///////////////////////////////////////////////////////////////////////////////
//
// IAR C/C++ Compiler V4.20.1.2260 for RL78               20/Dec/2020  22:38:33
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
//        BootCode\source_code\library_files\math_lib.c
//    Command line       =
//        -f C:\Users\laptop\AppData\Local\Temp\EW903F.tmp ("D:\Dheeraj\New
//        folder\0. GDEV72 - BootCode\source_code\library_files\math_lib.c"
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
//        D:\Dheeraj\New folder\0. GDEV72 - BootCode\Debug\List\math_lib.s
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
        #define SHF_WRITE 0x1

        EXTERN ?F_DIV
        EXTERN ?F_F2SL
        EXTERN ?F_MUL
        EXTERN ?F_UL2F
        EXTERN ?L_NEG_L03
        EXTERN ?UL_RSH_L03
        EXTERN __Add64
        EXTERN __CmpGes64
        EXTERN __L2LLS
        EXTERN __L2LLU
        EXTERN __LLS2F
        EXTERN __Neg64
        EXTERN __Sub64
        EXTERN ___iar_fmex
        EXTERN _cos
        EXTERN _sqrt

        PUBLIC _CalculateSinValue
        PUBLIC _SINTABLE_STEP001
        PUBLIC _SINTABLE_STEP1
        PUBLIC __Constant_0_0
        PUBLIC _vector_addition_2byte
        
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
        
// D:\Dheeraj\New folder\0. GDEV72 - BootCode\source_code\library_files\math_lib.c
//    1 /***********************************************************************************************************************
//    2 * File Name       : math_lib.c
//    3 * Current Version : rev_01  
//    4 * Tool-Chain      : IAR Systems RL78
//    5 * Description     : this library contains mathematical functions which is being used in the firmware
//    6 * Creation Date   : 07-03-2020
//    7 * Company         : Genus Power Infrastructures Limited, Jaipur
//    8 * Author          : dheeraj.singhal
//    9 * Version History : rev_01 : Math functions 
//   10 ***********************************************************************************************************************/
//   11 /************************************ Includes **************************************/
//   12 #include "math_lib.h"
//   13 /************************************ Local Variables *****************************************/
//   14 /************************************ Extern Variables *****************************************/
//   15 /************************************ Local Functions *******************************/
//   16 /************************************ Extern Functions ******************************/

        SECTION `.data`:DATA:REORDER:NOROOT(1)
//   17 const unsigned int SINTABLE_STEP1[91] = {0,1143,2287,3429,4571,5711,6850,7986,9120,10252,11380,12504,13625,14742,15854,16961,18064,19160,
_SINTABLE_STEP1:
        DATA16
        DW 0, 1'143, 2'287, 3'429, 4'571, 5'711, 6'850, 7'986, 9'120, 10'252
        DW 11'380, 12'504, 13'625, 14'742, 15'854, 16'961, 18'064, 19'160
        DW 20'251, 21'336, 22'414, 23'486, 24'550, 25'606, 26'655, 27'696
        DW 28'729, 29'752, 30'767, 31'772, 32'768, 33'753, 34'728, 35'693
        DW 36'647, 37'589, 38'521, 39'440, 40'347, 41'243, 42'125, 42'995
        DW 43'852, 44'695, 45'525, 46'340, 47'142, 47'929, 48'702, 49'460
        DW 50'203, 50'931, 51'643, 52'339, 53'019, 53'683, 54'331, 54'963
        DW 55'577, 56'175, 56'755, 57'319, 57'864, 58'393, 58'903, 59'395
        DW 59'870, 60'326, 60'763, 61'183, 61'583, 61'965, 62'328, 62'672
        DW 62'997, 63'302, 63'589, 63'856, 64'103, 64'331, 64'540, 64'729
        DW 64'898, 65'047, 65'176, 65'286, 65'376, 65'446, 65'496, 65'526
        DW 65'535
//   18 20251,21336,22414,23486,24550,25606,26655,27696,28729,29752,30767,31772,32768,33753,34728,35693,36647,37589,38521,39440,40347,41243,42125,
//   19 42995,43852,44695,45525,46340,47142,47929,48702,49460,50203,50931,51643,52339,53019,53683,54331,54963,55577,56175,56755,57319,57864,58393,
//   20 58903,59395,59870,60326,60763,61183,61583,61965,62328,62672,62997,63302,63589,63856,64103,64331,64540,64729,64898,65047,65176,65286,65376,
//   21 65446,65496,65526,65535};
//   22 

        SECTION `.data`:DATA:REORDER:NOROOT(1)
//   23 const unsigned int SINTABLE_STEP001[128] = {0,285,571,857,1143,1429,1715,2001,2287,2573,2859,3145,3431,3717,4003,4289,4575,4861,5147,
_SINTABLE_STEP001:
        DATA16
        DW 0, 285, 571, 857, 1'143, 1'429, 1'715, 2'001, 2'287, 2'573, 2'859
        DW 3'145, 3'431, 3'717, 4'003, 4'289, 4'575, 4'861, 5'147, 5'433, 5'719
        DW 6'005, 6'290, 6'576, 6'862, 7'148, 7'434, 7'720, 8'006, 8'292, 8'578
        DW 8'864, 9'150, 9'436, 9'722, 10'008, 10'294, 10'580, 10'866, 11'152
        DW 11'438, 11'724, 12'010, 12'295, 12'581, 12'867, 13'153, 13'439
        DW 13'725, 14'011, 14'297, 14'583, 14'869, 15'155, 15'441, 15'727
        DW 16'013, 16'299, 16'585, 16'871, 17'157, 17'443, 17'728, 18'014
        DW 18'300, 18'586, 18'872, 19'158, 19'444, 19'730, 20'016, 20'302
        DW 20'588, 20'874, 21'160, 21'446, 21'732, 22'018, 22'304, 22'589
        DW 22'875, 23'161, 23'447, 23'733, 24'019, 24'305, 24'591, 24'877
        DW 25'163, 25'449, 25'735, 26'021, 26'307, 26'593, 26'879, 27'164
        DW 27'450, 27'736, 28'022, 28'308, 28'594, 28'880, 29'166, 29'452
        DW 29'738, 30'024, 30'310, 30'596, 30'881, 31'167, 31'453, 31'739
        DW 32'025, 32'311, 32'597, 32'883, 33'169, 33'455, 33'741, 34'027
        DW 34'313, 34'598, 34'884, 35'170, 35'456, 35'742, 36'028, 36'314

        SECTION `.data`:DATA:REORDER:NOROOT(1)
        SECTION_GROUP __Constant_0_0
__Constant_0_0:
        DATA32
        DD 0, 0
//   24 5433,5719,6005,6290,6576,6862,7148,7434,7720,8006,8292,8578,8864,9150,9436,9722,10008,10294,10580,10866,11152,11438,11724,12010,12295,12581,
//   25 12867,13153,13439,13725,14011,14297,14583,14869,15155,15441,15727,16013,16299,16585,16871,17157,17443,17728,18014,18300,18586,18872,19158,
//   26 19444,19730,20016,20302,20588,20874,21160,21446,21732,22018,22304,22589,22875,23161,23447,23733,24019,24305,24591,24877,25163,25449,25735,
//   27 26021,26307,26593,26879,27164,27450,27736,28022,28308,28594,28880,29166,29452,29738,30024,30310,30596,30881,31167,31453,31739,32025,32311,
//   28 32597,32883,33169,33455,33741,34027,34313,34598,34884,35170,35456,35742,36028,36314};
//   29 
//   30 
//   31 
//   32 void vector_addition_2byte(us16 mag1, us16 mag2, us16 ang, us16 *res_mag);
//   33 signed long CalculateSinValue(unsigned int AngleInDeg128TimesInt0to360);
//   34 
//   35 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock0 Using cfiCommon0
          CFI Function _vector_addition_2byte
        CODE
//   36 void vector_addition_2byte(us16 mag1, us16 mag2, us16 ang, us16 *res_mag)
//   37 {
_vector_addition_2byte:
        ; * Stack frame (at entry) *
        ; Param size: 2
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+6
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+8
        ; Auto size: 26
        SUBW      SP, #0x16          ;; 1 cycle
          CFI CFA SP+30
        PUSH      DE                 ;; 1 cycle
          CFI CFA SP+32
        POP       HL                 ;; 1 cycle
          CFI CFA SP+30
//   38     s64 temp_mag = 0;
        MOVW      AX, SP             ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [DE], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [DE+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [DE+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [DE+0x06], AX      ;; 1 cycle
//   39     float temp =0;
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0xA           ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [DE], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [DE+0x02], AX      ;; 1 cycle
//   40     ang = ang%36000;
        MOVW      DE, #0x8CA0        ;; 1 cycle
        MOVW      AX, HL             ;; 1 cycle
        DIVHU                        ;; 9 cycles
        NOP                          ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        MOVW      [SP+0x08], AX      ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
//   41     temp = cos(RADIAN((float)ang/100.0));
        MOVW      AX, #0x3C8E        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
        MOVW      AX, #0xFA35        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+34
        MOVW      AX, #0x42C8        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+36
        MOVW      AX, #0x0           ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+38
        MOVW      AX, [SP+0x10]      ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
          CFI FunCall ?F_UL2F
        CALL      N:?F_UL2F          ;; 3 cycles
          CFI FunCall ?F_DIV
        CALL      N:?F_DIV           ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+34
          CFI FunCall ?F_MUL
        CALL      N:?F_MUL           ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+30
          CFI FunCall _cos
        CALL      _cos               ;; 3 cycles
//   42     temp *= mag1;
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+32
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+34
        MOVW      AX, [SP+0x1C]      ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
          CFI FunCall ?F_UL2F
        CALL      N:?F_UL2F          ;; 3 cycles
          CFI FunCall ?F_MUL
        CALL      N:?F_MUL           ;; 3 cycles
//   43     temp *= mag2;
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+36
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+38
        MOVW      AX, [SP+0x1E]      ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
          CFI FunCall ?F_UL2F
        CALL      N:?F_UL2F          ;; 3 cycles
          CFI FunCall ?F_MUL
        CALL      N:?F_MUL           ;; 3 cycles
//   44     temp *= 2;
        MOVW      DE, #0x40          ;; 1 cycle
          CFI FunCall ___iar_fmex
        CALL      N:___iar_fmex      ;; 3 cycles
        ADDW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+30
        XCHW      AX, HL             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0xA           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//   45     
//   46     temp_mag = (us32)mag1*mag1;
        MOVW      AX, [SP+0x18]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x18]      ;; 1 cycle
        MULHU                        ;; 2 cycles
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+32
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+34
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
          CFI FunCall __L2LLU
        CALL      __L2LLU            ;; 3 cycles
//   47     temp_mag += (us32)mag2*mag2;
        MOVW      AX, [SP+0x1A]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x1A]      ;; 1 cycle
        MULHU                        ;; 2 cycles
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+36
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+38
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x16          ;; 1 cycle
          CFI FunCall __L2LLU
        CALL      __L2LLU            ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x16          ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
          CFI FunCall __Add64
        CALL      __Add64            ;; 3 cycles
//   48     temp_mag -= (s32)(temp);
        MOVW      AX, [SP+0x14]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x12]      ;; 1 cycle
          CFI FunCall ?F_F2SL
        CALL      N:?F_F2SL          ;; 3 cycles
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+40
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+42
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x1A          ;; 1 cycle
          CFI FunCall __L2LLS
        CALL      __L2LLS            ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x1A          ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0xC           ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0xC           ;; 1 cycle
          CFI FunCall __Sub64
        CALL      __Sub64            ;; 3 cycles
//   49     *res_mag = (us16)sqrt(ABS(temp_mag));
        MOVW      BC, #LWRD(__Constant_0_0)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0xC           ;; 1 cycle
          CFI FunCall __CmpGes64
        CALL      __CmpGes64         ;; 3 cycles
        ADDW      SP, #0xC           ;; 1 cycle
          CFI CFA SP+30
        CMP0      A                  ;; 1 cycle
        BNZ       ??CalculateSinValue_0  ;; 4 cycles
        ; ------------------------------------- Block: 165 cycles
        MOVW      AX, SP             ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0xE           ;; 1 cycle
          CFI FunCall __Neg64
        CALL      __Neg64            ;; 3 cycles
        BR        S:??CalculateSinValue_1  ;; 3 cycles
        ; ------------------------------------- Block: 10 cycles
??CalculateSinValue_0:
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0xE           ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        ; ------------------------------------- Block: 13 cycles
??CalculateSinValue_1:
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0xE           ;; 1 cycle
          CFI FunCall __LLS2F
        CALL      __LLS2F            ;; 3 cycles
          CFI FunCall _sqrt
        CALL      _sqrt              ;; 3 cycles
          CFI FunCall ?F_F2SL
        CALL      N:?F_F2SL          ;; 3 cycles
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [SP+0x1E]      ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      [DE], AX           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
//   50 }
        ADDW      SP, #0x1A          ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock0
        ; ------------------------------------- Block: 24 cycles
        ; ------------------------------------- Total: 212 cycles
//   51 
//   52 
//   53 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock1 Using cfiCommon1
          CFI Function _CalculateSinValue
        CODE
//   54 signed long CalculateSinValue(unsigned int AngleInDeg128TimesInt0to360)
//   55 {
_CalculateSinValue:
        ; * Stack frame (at entry) *
        ; Param size: 0
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+6
        ; Auto size: 16
        SUBW      SP, #0xE           ;; 1 cycle
          CFI CFA SP+20
//   56     unsigned short AngleIntigerPart;
//   57     unsigned short AngleDecimalPart;
//   58     unsigned short localangle;
//   59     unsigned short SinA, CosA, SinB;
//   60     unsigned long  TempVal;
//   61     signed long SinVal65536Multiplied;
//   62     signed char SignOfSinVal;
//   63     
//   64     SignOfSinVal = 0;
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//   65     
//   66     if(AngleInDeg128TimesInt0to360 > DEG180_MULTIPLIED_BY_128)
        MOVW      AX, [SP+0x0E]      ;; 1 cycle
        CMPW      AX, #0x5A01        ;; 1 cycle
        BC        ??CalculateSinValue_2  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
//   67     {
//   68         SignOfSinVal = -1;
        MOV       A, #0xFF           ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//   69         AngleInDeg128TimesInt0to360 = AngleInDeg128TimesInt0to360 -  DEG180_MULTIPLIED_BY_128;
        MOVW      AX, [SP+0x0E]      ;; 1 cycle
        ADDW      AX, #0xA600        ;; 1 cycle
        MOVW      [SP+0x0E], AX      ;; 1 cycle
        ; ------------------------------------- Block: 5 cycles
//   70     }
//   71     
//   72     if(AngleInDeg128TimesInt0to360 > DEG090_MULTIPLIED_BY_128) AngleInDeg128TimesInt0to360 = DEG180_MULTIPLIED_BY_128 - AngleInDeg128TimesInt0to360;
??CalculateSinValue_2:
        MOVW      AX, [SP+0x0E]      ;; 1 cycle
        CMPW      AX, #0x2D01        ;; 1 cycle
        BC        ??CalculateSinValue_3  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, [SP+0x0E]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, #0x5A00        ;; 1 cycle
        SUBW      AX, HL             ;; 1 cycle
        MOVW      [SP+0x0E], AX      ;; 1 cycle
        ; ------------------------------------- Block: 5 cycles
//   73     localangle = AngleInDeg128TimesInt0to360;
??CalculateSinValue_3:
        MOVW      AX, [SP+0x0E]      ;; 1 cycle
        MOVW      [SP+0x02], AX      ;; 1 cycle
//   74     
//   75     AngleIntigerPart = localangle >> 7;
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        SHRW      AX, 0x7            ;; 1 cycle
        MOVW      [SP+0x04], AX      ;; 1 cycle
//   76     AngleDecimalPart = localangle & 0x7F;
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        AND       A, #0x0            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        AND       A, #0x7F           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      [SP+0x0C], AX      ;; 1 cycle
//   77     
//   78     SinA = SINTABLE_STEP1[AngleIntigerPart];
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(_SINTABLE_STEP1)  ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [HL]           ;; 1 cycle
        MOVW      [SP+0x0A], AX      ;; 1 cycle
//   79     CosA = SINTABLE_STEP1[90 - AngleIntigerPart];
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        XOR       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        XOR       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        INCW      AX                 ;; 1 cycle
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(_SINTABLE_STEP1+180)  ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [HL]           ;; 1 cycle
        MOVW      [SP+0x08], AX      ;; 1 cycle
//   80     SinB = SINTABLE_STEP001[AngleDecimalPart];
        MOVW      AX, [SP+0x0C]      ;; 1 cycle
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(_SINTABLE_STEP001)  ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [HL]           ;; 1 cycle
        MOVW      [SP+0x06], AX      ;; 1 cycle
//   81     
//   82     TempVal = (unsigned long)CosA * (unsigned long)SinB;
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x08]      ;; 1 cycle
        MULHU                        ;; 2 cycles
//   83     TempVal >>= 21;
        MOV       E, #0x15           ;; 1 cycle
          CFI FunCall ?UL_RSH_L03
        CALL      N:?UL_RSH_L03      ;; 3 cycles
        MOVW      DE, AX             ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+22
        POP       HL                 ;; 1 cycle
          CFI CFA SP+20
//   84     TempVal = TempVal + SinA;
        MOVW      AX, [SP+0x0A]      ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        ADDW      AX, DE             ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+22
        POP       HL                 ;; 1 cycle
          CFI CFA SP+20
        MOVW      DE, AX             ;; 1 cycle
//   85     
//   86     if(SignOfSinVal) TempVal = 0 - TempVal;
        MOV       A, [SP]            ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        BZ        ??CalculateSinValue_4  ;; 4 cycles
        ; ------------------------------------- Block: 72 cycles
        MOVW      AX, DE             ;; 1 cycle
        PUSH      HL                 ;; 1 cycle
          CFI CFA SP+22
        POP       BC                 ;; 1 cycle
          CFI CFA SP+20
          CFI FunCall ?L_NEG_L03
        CALL      N:?L_NEG_L03       ;; 3 cycles
        MOVW      DE, AX             ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+22
        POP       HL                 ;; 1 cycle
          CFI CFA SP+20
        ; ------------------------------------- Block: 9 cycles
//   87     SinVal65536Multiplied = TempVal;
??CalculateSinValue_4:
        MOVW      AX, DE             ;; 1 cycle
        PUSH      HL                 ;; 1 cycle
          CFI CFA SP+22
        POP       BC                 ;; 1 cycle
          CFI CFA SP+20
//   88     
//   89     return SinVal65536Multiplied;
        ADDW      SP, #0x10          ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock1
        ; ------------------------------------- Block: 10 cycles
        ; ------------------------------------- Total: 117 cycles
//   90 }

        SECTION `.constf`:FARCODE:REORDER:NOROOT(1)
        SECTION_TYPE SHT_PROGBITS, SHF_WRITE

        SECTION `.iar_vfe_header`:DATA:NOALLOC:NOROOT(1)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
        DC32 0

        END
//   91 
//   92 
// 
// 446 bytes in section .data
// 472 bytes in section .text
// 
// 472 bytes of CODE memory
// 438 bytes of DATA memory (+ 8 bytes shared)
//
//Errors: none
//Warnings: none
