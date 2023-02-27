///////////////////////////////////////////////////////////////////////////////
//
// IAR C/C++ Compiler V4.20.1.2260 for RL78               20/Dec/2020  22:38:07
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
//        BootCode\source_code\source_files\calibration.c
//    Command line       =
//        -f C:\Users\laptop\AppData\Local\Temp\EW2AAC.tmp ("D:\Dheeraj\New
//        folder\0. GDEV72 - BootCode\source_code\source_files\calibration.c"
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
//        D:\Dheeraj\New folder\0. GDEV72 - BootCode\Debug\List\calibration.s
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

        EXTERN _opr_data
        EXTERN _flag1
        EXTERN ?F_ADD
        EXTERN ?F_DIV
        EXTERN ?F_F2SL
        EXTERN ?F_MUL
        EXTERN ?F_SL2F
        EXTERN ?F_SUB
        EXTERN ?MOVE_LONG_L06
        EXTERN ?SI_MOD_L02
        EXTERN _Now
        EXTERN _R_DSADC_update_phase_correction
        EXTERN __Divu64
        EXTERN __L2LLU
        EXTERN __Mul64
        EXTERN _atan
        EXTERN _b_phase
        EXTERN _cal_coeff
        EXTERN _char_array_to_int
        EXTERN _curr
        EXTERN _eprom_read
        EXTERN _eprom_write
        EXTERN _flag_eep1
        EXTERN _int_into_char_array
        EXTERN _lcd_write_msg
        EXTERN _n_phase
        EXTERN _power
        EXTERN _r_phase
        EXTERN _temp_double
        EXTERN _temp_us64
        EXTERN _time_into_char_array6_sec
        EXTERN _vol
        EXTERN _y_phase

        PUBLIC __A_P6
        PUBLIC _cal_BPh
        PUBLIC _cal_RPh
        PUBLIC _cal_YPh
        PUBLIC _cal_angle_calculate_act
        PUBLIC _cal_angle_calculate_react
        PUBLIC _cal_buffer_currb
        PUBLIC _cal_buffer_currr
        PUBLIC _cal_buffer_curry
        PUBLIC _cal_buffer_ptr
        PUBLIC _cal_buffer_volb
        PUBLIC _cal_buffer_volb90
        PUBLIC _cal_buffer_volr
        PUBLIC _cal_buffer_volr90
        PUBLIC _cal_buffer_voly
        PUBLIC _cal_buffer_voly90
        PUBLIC _cal_coeff_range_check
        PUBLIC _cal_done_f
        PUBLIC _cal_error_code
        PUBLIC _cal_stage
        PUBLIC _cal_timer
        PUBLIC _calibration_1sec_loop
        PUBLIC _calibration_delay_voltage_samples
        PUBLIC _calibration_load
        PUBLIC _calibration_load_default
        PUBLIC _calibration_process
        PUBLIC _calibration_reload_def_coeff
        PUBLIC _calibration_reset
        PUBLIC _calibration_save
        PUBLIC _flag_cal
        PUBLIC _flag_calibration
        PUBLIC _invtanB
        PUBLIC _tanB
        
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
        
// D:\Dheeraj\New folder\0. GDEV72 - BootCode\source_code\source_files\calibration.c
//    1 /***********************************************************************************************************************
//    2 * File Name       : calibration.c
//    3 * Current Version : rev_01  
//    4 * Tool-Chain      : IAR Systems RL78
//    5 * Description     : All routines related to calibration of the Energy meter (Metering, RTC)
//    6 * Creation Date   : 10-02-2020
//    7 * Company         : Genus Power Infrastructures Limited, Jaipur
//    8 * Author          : dheeraj.singhal
//    9 * Version History : rev_01 :    Initial release with required functionality
//   10 ***********************************************************************************************************************/
//   11 /************************************ Includes **************************************/
//   12 #include "calibration.h"

        ASEGN `.sbss.noinit`:DATA:NOROOT,0fff06H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_P6
// __no_init union <unnamed>#9 volatile __saddr _A_P6
__A_P6:
        DS 1
//   13 /************************************ Local Variables *****************************************/
//   14 /************************************ Extern Variables *****************************************/

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   15 flag_union flag_calibration,flag_cal;
_flag_calibration:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_flag_cal:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   16 CAL_PHASES cal_RPh,cal_YPh,cal_BPh;
_cal_RPh:
        DS 98

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_cal_YPh:
        DS 98

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_cal_BPh:
        DS 98

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   17 us8 cal_done_f,cal_timer,cal_error_code,cal_stage;
_cal_done_f:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_cal_timer:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_cal_error_code:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_cal_stage:
        DS 1
//   18 

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   19 s16 cal_buffer_volr[CAL_BUF_SIZE_VOL],cal_buffer_voly[CAL_BUF_SIZE_VOL],cal_buffer_volb[CAL_BUF_SIZE_VOL];
_cal_buffer_volr:
        DS 54

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_cal_buffer_voly:
        DS 54

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_cal_buffer_volb:
        DS 54

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   20 s16 cal_buffer_currr[CAL_BUF_SIZE_VOL],cal_buffer_curry[CAL_BUF_SIZE_VOL],cal_buffer_currb[CAL_BUF_SIZE_VOL];
_cal_buffer_currr:
        DS 54

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_cal_buffer_curry:
        DS 54

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_cal_buffer_currb:
        DS 54

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   21 us8 cal_buffer_ptr;
_cal_buffer_ptr:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   22 s16 cal_buffer_volr90[CAL_BUF_SIZE_VOL],cal_buffer_voly90[CAL_BUF_SIZE_VOL],cal_buffer_volb90[CAL_BUF_SIZE_VOL];
_cal_buffer_volr90:
        DS 54

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_cal_buffer_voly90:
        DS 54

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_cal_buffer_volb90:
        DS 54

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   23 double tanB,invtanB;
_tanB:
        DS 4

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_invtanB:
        DS 4
//   24 
//   25 /************************************ Local Functions *******************************/
//   26 us8 cal_coeff_range_check();
//   27 void calibration_reload_def_coeff();
//   28 /************************************ Extern Functions ******************************/
//   29 us8 calibration_reset();
//   30 void calibration_load();
//   31 void calibration_save();
//   32 void calibration_load_default();
//   33 void calibration_process(us8 phase,us8 stage);
//   34 void calibration_delay_voltage_samples();
//   35 void calibration_1sec_loop();
//   36 s16 cal_angle_calculate_act(s32 left_power,s32 right_power);
//   37 s16 cal_angle_calculate_react(s32 left_power, s32 right_power);

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock0 Using cfiCommon0
          CFI Function _calibration_save
        CODE
//   38 void calibration_save()
//   39 {
_calibration_save:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//   40   eprom_read(ADDR_CALIBRATION,0,PAGE_4,AUTO_CALC);
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x3            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x100         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
//   41   int_into_char_array(cal_coeff.Rph.vol,&opr_data[0]);
        MOVW      BC, #LWRD(_opr_data)  ;; 1 cycle
        MOVW      AX, N:_cal_coeff+6  ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//   42   int_into_char_array(cal_coeff.Yph.vol,&opr_data[2]);
        MOVW      BC, #LWRD(_opr_data+2)  ;; 1 cycle
        MOVW      AX, N:_cal_coeff+18  ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//   43   int_into_char_array(cal_coeff.Bph.vol,&opr_data[4]);
        MOVW      BC, #LWRD(_opr_data+4)  ;; 1 cycle
        MOVW      AX, N:_cal_coeff+30  ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//   44   int_into_char_array(cal_coeff.Rph.curr,&opr_data[6]);
        MOVW      BC, #LWRD(_opr_data+6)  ;; 1 cycle
        MOVW      AX, N:_cal_coeff+8  ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//   45   int_into_char_array(cal_coeff.Yph.curr,&opr_data[8]);
        MOVW      BC, #LWRD(_opr_data+8)  ;; 1 cycle
        MOVW      AX, N:_cal_coeff+20  ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//   46   int_into_char_array(cal_coeff.Bph.curr,&opr_data[10]);
        MOVW      BC, #LWRD(_opr_data+10)  ;; 1 cycle
        MOVW      AX, N:_cal_coeff+32  ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//   47   int_into_char_array(cal_coeff.Nph.curr,&opr_data[12]);
        MOVW      BC, #LWRD(_opr_data+12)  ;; 1 cycle
        MOVW      AX, N:_cal_coeff+40  ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//   48   int_into_char_array(cal_coeff.Rph.power,&opr_data[14]);
        MOVW      BC, #LWRD(_opr_data+14)  ;; 1 cycle
        MOVW      AX, N:_cal_coeff+10  ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//   49   int_into_char_array(cal_coeff.Yph.power,&opr_data[16]);
        MOVW      BC, #LWRD(_opr_data+16)  ;; 1 cycle
        MOVW      AX, N:_cal_coeff+22  ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//   50   int_into_char_array(cal_coeff.Bph.power,&opr_data[18]);
        MOVW      BC, #LWRD(_opr_data+18)  ;; 1 cycle
        MOVW      AX, N:_cal_coeff+34  ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//   51   int_into_char_array(cal_coeff.Rph.vol_offset,&opr_data[20]);
        MOVW      BC, #LWRD(_opr_data+20)  ;; 1 cycle
        MOVW      AX, N:_cal_coeff+2  ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//   52   int_into_char_array(cal_coeff.Yph.vol_offset,&opr_data[22]);
        MOVW      BC, #LWRD(_opr_data+22)  ;; 1 cycle
        MOVW      AX, N:_cal_coeff+14  ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//   53   int_into_char_array(cal_coeff.Bph.vol_offset,&opr_data[24]);
        MOVW      BC, #LWRD(_opr_data+24)  ;; 1 cycle
        MOVW      AX, N:_cal_coeff+26  ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//   54   int_into_char_array(cal_coeff.Rph.curr_offset,&opr_data[26]);
        MOVW      BC, #LWRD(_opr_data+26)  ;; 1 cycle
        MOVW      AX, N:_cal_coeff+4  ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//   55   int_into_char_array(cal_coeff.Yph.curr_offset,&opr_data[28]);
        MOVW      BC, #LWRD(_opr_data+28)  ;; 1 cycle
        MOVW      AX, N:_cal_coeff+16  ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//   56   int_into_char_array(cal_coeff.Bph.curr_offset,&opr_data[30]);
        MOVW      BC, #LWRD(_opr_data+30)  ;; 1 cycle
        MOVW      AX, N:_cal_coeff+28  ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//   57   int_into_char_array(cal_coeff.Nph.curr_offset,&opr_data[32]);
        MOVW      BC, #LWRD(_opr_data+32)  ;; 1 cycle
        MOVW      AX, N:_cal_coeff+38  ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//   58   int_into_char_array(cal_coeff.Rph.phase_correction,&opr_data[34]);
        MOVW      BC, #LWRD(_opr_data+34)  ;; 1 cycle
        MOVW      AX, N:_cal_coeff+12  ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//   59   int_into_char_array(cal_coeff.Yph.phase_correction,&opr_data[36]);
        MOVW      BC, #LWRD(_opr_data+36)  ;; 1 cycle
        MOVW      AX, N:_cal_coeff+24  ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//   60   int_into_char_array(cal_coeff.Bph.phase_correction,&opr_data[38]);
        MOVW      BC, #LWRD(_opr_data+38)  ;; 1 cycle
        MOVW      AX, N:_cal_coeff+36  ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//   61   time_into_char_array6_sec(Now,&opr_data[54]);
        MOVW      HL, #LWRD(_Now)    ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+12
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
        MOVW      AX, #LWRD(_opr_data+54)  ;; 1 cycle
          CFI FunCall _time_into_char_array6_sec
        CALL      _time_into_char_array6_sec  ;; 3 cycles
//   62   opr_data[60]++;
        INC       N:_opr_data+60     ;; 2 cycles
//   63   opr_data[61] = cal_error_code;
        MOV       A, N:_cal_error_code  ;; 1 cycle
        MOV       N:_opr_data+61, A  ;; 1 cycle
//   64   opr_data[62] = cal_done_f;
        MOV       A, N:_cal_done_f   ;; 1 cycle
        MOV       N:_opr_data+62, A  ;; 1 cycle
//   65   eprom_write(ADDR_CALIBRATION,0,64,PAGE_4,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+14
        MOV       B, #0x3            ;; 1 cycle
        MOVW      DE, #0x40          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x100         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//   66   eprom_write(ADDR_CALIBRATION_ALT,2,64,PAGE_4,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+16
        MOV       B, #0x3            ;; 1 cycle
        MOVW      DE, #0x40          ;; 1 cycle
        MOV       C, #0x2            ;; 1 cycle
        MOVW      AX, #0xFEC0        ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
//   67 }
        ADDW      SP, #0xC           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock0
        ; ------------------------------------- Block: 149 cycles
        ; ------------------------------------- Total: 149 cycles

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock1 Using cfiCommon0
          CFI Function _calibration_reset
        CODE
//   68 us8 calibration_reset()
//   69 {
_calibration_reset:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//   70   if(fg_done_f == 0 && battery_mode_f == 0)
        MOV       A, N:_flag1        ;; 1 cycle
        AND       A, #0x3            ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        BNZ       ??cal_angle_calculate_react_0  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//   71   {
//   72     cal_error_code = 0;
        MOV       N:_cal_error_code, #0x0  ;; 1 cycle
//   73     cal_done_f = 0;
        MOV       N:_cal_done_f, #0x0  ;; 1 cycle
//   74     calibration_load_default();
          CFI FunCall _calibration_load_default
        CALL      _calibration_load_default  ;; 3 cycles
//   75     calibration_save();
          CFI FunCall _calibration_save
        CALL      _calibration_save  ;; 3 cycles
//   76     return 1;
        MOV       A, #0x1            ;; 1 cycle
        RET                          ;; 6 cycles
        ; ------------------------------------- Block: 15 cycles
//   77   }
//   78   else
//   79   {
//   80     return 0;
??cal_angle_calculate_react_0:
        MOV       A, #0x0            ;; 1 cycle
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock1
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 29 cycles
//   81   }
//   82 }

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock2 Using cfiCommon0
          CFI Function _calibration_reload_def_coeff
          CFI FunCall _calibration_load_default
        CODE
//   83 void calibration_reload_def_coeff()
//   84 {
_calibration_reload_def_coeff:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//   85   calibration_load_default();
        CALL      _calibration_load_default  ;; 3 cycles
//   86   R_DSADC_update_phase_correction(PHASE_R,cal_coeff.Rph.phase_correction);
        MOVW      BC, N:_cal_coeff+12  ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _R_DSADC_update_phase_correction
        CALL      _R_DSADC_update_phase_correction  ;; 3 cycles
//   87   R_DSADC_update_phase_correction(PHASE_Y,cal_coeff.Yph.phase_correction);
        MOVW      BC, N:_cal_coeff+24  ;; 1 cycle
        MOV       A, #0x1            ;; 1 cycle
          CFI FunCall _R_DSADC_update_phase_correction
        CALL      _R_DSADC_update_phase_correction  ;; 3 cycles
//   88   R_DSADC_update_phase_correction(PHASE_B,cal_coeff.Bph.phase_correction);
        MOVW      BC, N:_cal_coeff+36  ;; 1 cycle
        MOV       A, #0x2            ;; 1 cycle
          CFI FunCall _R_DSADC_update_phase_correction
        CALL      _R_DSADC_update_phase_correction  ;; 3 cycles
//   89 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock2
        ; ------------------------------------- Block: 24 cycles
        ; ------------------------------------- Total: 24 cycles

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock3 Using cfiCommon0
          CFI Function _calibration_load
        CODE
//   90 void calibration_load()
//   91 {
_calibration_load:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 2
        SUBW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+6
//   92   flag_cal_error_f = 0;
        CLR1      N:_flag_calibration.0  ;; 2 cycles
//   93   cal_error_code = 0;
        MOV       N:_cal_error_code, #0x0  ;; 1 cycle
//   94   cal_stage = 0;
        MOV       N:_cal_stage, #0x0  ;; 1 cycle
//   95   flag_cal_reset_powerup = 0;
        CLR1      N:_flag_cal.3      ;; 2 cycles
//   96   if(CAL_JMP_CONNECTED)       
        MOV       A, S:0xFFF06       ;; 1 cycle
        AND       A, #0x2            ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        BNZ       ??cal_angle_calculate_react_1  ;; 4 cycles
          CFI FunCall _calibration_reset
        ; ------------------------------------- Block: 14 cycles
//   97   {
//   98     flag_cal_reset_powerup = calibration_reset();
        CALL      _calibration_reset  ;; 3 cycles
        MOV       [SP], A            ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        MOVW      HL, #LWRD(_flag_cal)  ;; 1 cycle
        MOV1      [HL].3, CY         ;; 2 cycles
        ; ------------------------------------- Block: 10 cycles
//   99   }
//  100   if(flag_cal_reset_powerup == 0)
??cal_angle_calculate_react_1:
        MOVW      HL, #LWRD(_flag_cal)  ;; 1 cycle
        MOV1      CY, [HL].3         ;; 1 cycle
        SKNC                         ;; 4 cycles
        BR        N:??cal_angle_calculate_react_2  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  101   {
//  102     if(eprom_read(ADDR_CALIBRATION,0,PAGE_4,AUTO_CALC) == EEP_ERROR)
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x3            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x100         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??cal_angle_calculate_react_3  ;; 4 cycles
        ; ------------------------------------- Block: 12 cycles
//  103     {
//  104       if(eprom_read(ADDR_CALIBRATION_ALT,2,PAGE_4,AUTO_CALC) == EEP_OK)
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x3            ;; 1 cycle
        MOV       C, #0x2            ;; 1 cycle
        MOVW      AX, #0xFEC0        ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
        CMP       A, #0x1            ;; 1 cycle
        BNZ       ??cal_angle_calculate_react_3  ;; 4 cycles
        ; ------------------------------------- Block: 12 cycles
//  105       {
//  106         eprom_write(ADDR_CALIBRATION,0,64,PAGE_4,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+8
        MOV       B, #0x3            ;; 1 cycle
        MOVW      DE, #0x40          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x100         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+6
        ; ------------------------------------- Block: 10 cycles
//  107       }
//  108     }
//  109     if(eep_read_ok == 1)
??cal_angle_calculate_react_3:
        MOVW      HL, #LWRD(_flag_eep1)  ;; 1 cycle
        MOV1      CY, [HL].3         ;; 1 cycle
        SKC                          ;; 4 cycles
        BR        N:??cal_angle_calculate_react_4  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  110     {
//  111       cal_done_f = opr_data[62];
        MOV       A, N:_opr_data+62  ;; 1 cycle
        MOV       N:_cal_done_f, A   ;; 1 cycle
//  112       if(cal_done_f == 1 || cal_done_f == 2)
        CMP       N:_cal_done_f, #0x1  ;; 1 cycle
        BZ        ??cal_angle_calculate_react_5  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        CMP       N:_cal_done_f, #0x2  ;; 1 cycle
        SKZ                          ;; 4 cycles
        BR        N:??cal_angle_calculate_react_6  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  113       {
//  114         if(cal_done_f == 2)
??cal_angle_calculate_react_5:
        CMP       N:_cal_done_f, #0x2  ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
//  115         {
//  116           cal_done_f = 3;
        MOV       N:_cal_done_f, #0x3  ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
//  117         }
//  118         cal_coeff.Rph.vol = char_array_to_int(&opr_data[0]);
??calibration_load_0:
        MOVW      AX, #LWRD(_opr_data)  ;; 1 cycle
          CFI FunCall _char_array_to_int
        CALL      _char_array_to_int  ;; 3 cycles
        MOVW      N:_cal_coeff+6, AX  ;; 1 cycle
//  119         cal_coeff.Yph.vol = char_array_to_int(&opr_data[2]);
        MOVW      AX, #LWRD(_opr_data+2)  ;; 1 cycle
          CFI FunCall _char_array_to_int
        CALL      _char_array_to_int  ;; 3 cycles
        MOVW      N:_cal_coeff+18, AX  ;; 1 cycle
//  120         cal_coeff.Bph.vol = char_array_to_int(&opr_data[4]);
        MOVW      AX, #LWRD(_opr_data+4)  ;; 1 cycle
          CFI FunCall _char_array_to_int
        CALL      _char_array_to_int  ;; 3 cycles
        MOVW      N:_cal_coeff+30, AX  ;; 1 cycle
//  121         cal_coeff.Rph.curr = char_array_to_int(&opr_data[6]);
        MOVW      AX, #LWRD(_opr_data+6)  ;; 1 cycle
          CFI FunCall _char_array_to_int
        CALL      _char_array_to_int  ;; 3 cycles
        MOVW      N:_cal_coeff+8, AX  ;; 1 cycle
//  122         cal_coeff.Yph.curr = char_array_to_int(&opr_data[8]);
        MOVW      AX, #LWRD(_opr_data+8)  ;; 1 cycle
          CFI FunCall _char_array_to_int
        CALL      _char_array_to_int  ;; 3 cycles
        MOVW      N:_cal_coeff+20, AX  ;; 1 cycle
//  123         cal_coeff.Bph.curr = char_array_to_int(&opr_data[10]);
        MOVW      AX, #LWRD(_opr_data+10)  ;; 1 cycle
          CFI FunCall _char_array_to_int
        CALL      _char_array_to_int  ;; 3 cycles
        MOVW      N:_cal_coeff+32, AX  ;; 1 cycle
//  124         cal_coeff.Nph.curr = char_array_to_int(&opr_data[12]);
        MOVW      AX, #LWRD(_opr_data+12)  ;; 1 cycle
          CFI FunCall _char_array_to_int
        CALL      _char_array_to_int  ;; 3 cycles
        MOVW      N:_cal_coeff+40, AX  ;; 1 cycle
//  125         cal_coeff.Rph.power = char_array_to_int(&opr_data[14]);
        MOVW      AX, #LWRD(_opr_data+14)  ;; 1 cycle
          CFI FunCall _char_array_to_int
        CALL      _char_array_to_int  ;; 3 cycles
        MOVW      N:_cal_coeff+10, AX  ;; 1 cycle
//  126         cal_coeff.Yph.power = char_array_to_int(&opr_data[16]);
        MOVW      AX, #LWRD(_opr_data+16)  ;; 1 cycle
          CFI FunCall _char_array_to_int
        CALL      _char_array_to_int  ;; 3 cycles
        MOVW      N:_cal_coeff+22, AX  ;; 1 cycle
//  127         cal_coeff.Bph.power = char_array_to_int(&opr_data[18]);
        MOVW      AX, #LWRD(_opr_data+18)  ;; 1 cycle
          CFI FunCall _char_array_to_int
        CALL      _char_array_to_int  ;; 3 cycles
        MOVW      N:_cal_coeff+34, AX  ;; 1 cycle
//  128         cal_coeff.Rph.vol_offset = char_array_to_int(&opr_data[20]);
        MOVW      AX, #LWRD(_opr_data+20)  ;; 1 cycle
          CFI FunCall _char_array_to_int
        CALL      _char_array_to_int  ;; 3 cycles
        MOVW      N:_cal_coeff+2, AX  ;; 1 cycle
//  129         cal_coeff.Yph.vol_offset = char_array_to_int(&opr_data[22]);
        MOVW      AX, #LWRD(_opr_data+22)  ;; 1 cycle
          CFI FunCall _char_array_to_int
        CALL      _char_array_to_int  ;; 3 cycles
        MOVW      N:_cal_coeff+14, AX  ;; 1 cycle
//  130         cal_coeff.Bph.vol_offset = char_array_to_int(&opr_data[24]);
        MOVW      AX, #LWRD(_opr_data+24)  ;; 1 cycle
          CFI FunCall _char_array_to_int
        CALL      _char_array_to_int  ;; 3 cycles
        MOVW      N:_cal_coeff+26, AX  ;; 1 cycle
//  131         cal_coeff.Rph.curr_offset = char_array_to_int(&opr_data[26]);
        MOVW      AX, #LWRD(_opr_data+26)  ;; 1 cycle
          CFI FunCall _char_array_to_int
        CALL      _char_array_to_int  ;; 3 cycles
        MOVW      N:_cal_coeff+4, AX  ;; 1 cycle
//  132         cal_coeff.Yph.curr_offset = char_array_to_int(&opr_data[28]);
        MOVW      AX, #LWRD(_opr_data+28)  ;; 1 cycle
          CFI FunCall _char_array_to_int
        CALL      _char_array_to_int  ;; 3 cycles
        MOVW      N:_cal_coeff+16, AX  ;; 1 cycle
//  133         cal_coeff.Bph.curr_offset = char_array_to_int(&opr_data[30]);
        MOVW      AX, #LWRD(_opr_data+30)  ;; 1 cycle
          CFI FunCall _char_array_to_int
        CALL      _char_array_to_int  ;; 3 cycles
        MOVW      N:_cal_coeff+28, AX  ;; 1 cycle
//  134         cal_coeff.Nph.curr_offset = char_array_to_int(&opr_data[32]);
        MOVW      AX, #LWRD(_opr_data+32)  ;; 1 cycle
          CFI FunCall _char_array_to_int
        CALL      _char_array_to_int  ;; 3 cycles
        MOVW      N:_cal_coeff+38, AX  ;; 1 cycle
//  135         cal_coeff.Rph.phase_correction = char_array_to_int(&opr_data[34]);
        MOVW      AX, #LWRD(_opr_data+34)  ;; 1 cycle
          CFI FunCall _char_array_to_int
        CALL      _char_array_to_int  ;; 3 cycles
        MOVW      N:_cal_coeff+12, AX  ;; 1 cycle
//  136         cal_coeff.Yph.phase_correction = char_array_to_int(&opr_data[36]);
        MOVW      AX, #LWRD(_opr_data+36)  ;; 1 cycle
          CFI FunCall _char_array_to_int
        CALL      _char_array_to_int  ;; 3 cycles
        MOVW      N:_cal_coeff+24, AX  ;; 1 cycle
//  137         cal_coeff.Bph.phase_correction = char_array_to_int(&opr_data[38]);
        MOVW      AX, #LWRD(_opr_data+38)  ;; 1 cycle
          CFI FunCall _char_array_to_int
        CALL      _char_array_to_int  ;; 3 cycles
        MOVW      N:_cal_coeff+36, AX  ;; 1 cycle
//  138         if(cal_coeff_range_check() == 1)
          CFI FunCall _cal_coeff_range_check
        CALL      _cal_coeff_range_check  ;; 3 cycles
        CMP       A, #0x1            ;; 1 cycle
        BNZ       ??cal_angle_calculate_react_7  ;; 4 cycles
        ; ------------------------------------- Block: 108 cycles
//  139         {
//  140           NOP();
        NOP                          ;; 1 cycle
        BR        S:??cal_angle_calculate_react_8  ;; 3 cycles
        ; ------------------------------------- Block: 4 cycles
//  141         }
//  142         else
//  143         {
//  144           cal_done_f = 0;
??cal_angle_calculate_react_7:
        MOV       N:_cal_done_f, #0x0  ;; 1 cycle
//  145           calibration_load_default();
          CFI FunCall _calibration_load_default
        CALL      _calibration_load_default  ;; 3 cycles
        BR        S:??cal_angle_calculate_react_8  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
//  146         }
//  147       }
//  148       else
//  149       {
//  150         cal_done_f = 0;
??cal_angle_calculate_react_6:
        MOV       N:_cal_done_f, #0x0  ;; 1 cycle
//  151         calibration_load_default();
          CFI FunCall _calibration_load_default
        CALL      _calibration_load_default  ;; 3 cycles
        BR        S:??cal_angle_calculate_react_8  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
//  152       }
//  153     }
//  154     else
//  155     {
//  156       cal_done_f = 0;
??cal_angle_calculate_react_4:
        MOV       N:_cal_done_f, #0x0  ;; 1 cycle
//  157       calibration_load_default();
          CFI FunCall _calibration_load_default
        CALL      _calibration_load_default  ;; 3 cycles
        BR        S:??cal_angle_calculate_react_8  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
//  158     }
//  159   }
//  160   else
//  161   {
//  162     cal_done_f = 0;
??cal_angle_calculate_react_2:
        MOV       N:_cal_done_f, #0x0  ;; 1 cycle
//  163     calibration_load_default();
          CFI FunCall _calibration_load_default
        CALL      _calibration_load_default  ;; 3 cycles
        ; ------------------------------------- Block: 4 cycles
//  164   }
//  165 }
??cal_angle_calculate_react_8:
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock3
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 229 cycles
        REQUIRE __A_P6

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock4 Using cfiCommon0
          CFI Function _calibration_load_default
          CFI NoCalls
        CODE
//  166 void calibration_load_default()
//  167 {
_calibration_load_default:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  168   cal_coeff.Rph.vol = 34800;
        MOVW      AX, #0x87F0        ;; 1 cycle
        MOVW      N:_cal_coeff+6, AX  ;; 1 cycle
//  169   cal_coeff.Yph.vol = 34800;
        MOVW      AX, #0x87F0        ;; 1 cycle
        MOVW      N:_cal_coeff+18, AX  ;; 1 cycle
//  170   cal_coeff.Bph.vol = 34800;
        MOVW      AX, #0x87F0        ;; 1 cycle
        MOVW      N:_cal_coeff+30, AX  ;; 1 cycle
//  171   cal_coeff.Rph.curr = 21000;
        MOVW      AX, #0x5208        ;; 1 cycle
        MOVW      N:_cal_coeff+8, AX  ;; 1 cycle
//  172   cal_coeff.Yph.curr = 21000;
        MOVW      AX, #0x5208        ;; 1 cycle
        MOVW      N:_cal_coeff+20, AX  ;; 1 cycle
//  173   cal_coeff.Bph.curr = 21000;
        MOVW      AX, #0x5208        ;; 1 cycle
        MOVW      N:_cal_coeff+32, AX  ;; 1 cycle
//  174   cal_coeff.Nph.curr = 26000;
        MOVW      AX, #0x6590        ;; 1 cycle
        MOVW      N:_cal_coeff+40, AX  ;; 1 cycle
//  175   cal_coeff.Rph.power = 8500;
        MOVW      AX, #0x2134        ;; 1 cycle
        MOVW      N:_cal_coeff+10, AX  ;; 1 cycle
//  176   cal_coeff.Yph.power = 8500;
        MOVW      AX, #0x2134        ;; 1 cycle
        MOVW      N:_cal_coeff+22, AX  ;; 1 cycle
//  177   cal_coeff.Bph.power = 8500;
        MOVW      AX, #0x2134        ;; 1 cycle
        MOVW      N:_cal_coeff+34, AX  ;; 1 cycle
//  178   cal_coeff.Rph.vol_offset = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_cal_coeff+2, AX  ;; 1 cycle
//  179   cal_coeff.Yph.vol_offset = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_cal_coeff+14, AX  ;; 1 cycle
//  180   cal_coeff.Bph.vol_offset = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_cal_coeff+26, AX  ;; 1 cycle
//  181   cal_coeff.Rph.curr_offset = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_cal_coeff+4, AX  ;; 1 cycle
//  182   cal_coeff.Yph.curr_offset = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_cal_coeff+16, AX  ;; 1 cycle
//  183   cal_coeff.Bph.curr_offset = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_cal_coeff+28, AX  ;; 1 cycle
//  184   cal_coeff.Nph.curr_offset = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_cal_coeff+38, AX  ;; 1 cycle
//  185   cal_coeff.Rph.phase_correction = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_cal_coeff+12, AX  ;; 1 cycle
//  186   cal_coeff.Yph.phase_correction = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_cal_coeff+24, AX  ;; 1 cycle
//  187   cal_coeff.Bph.phase_correction = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_cal_coeff+36, AX  ;; 1 cycle
//  188 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock4
        ; ------------------------------------- Block: 46 cycles
        ; ------------------------------------- Total: 46 cycles
//  189 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock5 Using cfiCommon0
          CFI Function _calibration_process
        CODE
//  190 void calibration_process(us8 phase,us8 stage)
//  191 {
_calibration_process:
        ; * Stack frame (at entry) *
        ; Param size: 0
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+6
        ; Auto size: 26
        SUBW      SP, #0x18          ;; 1 cycle
          CFI CFA SP+30
//  192   if(phase == PHASE_ALL)
        MOV       A, [SP+0x19]       ;; 1 cycle
        CMP       A, #0x3            ;; 1 cycle
        SKZ                          ;; 4 cycles
        BR        N:??cal_angle_calculate_react_9  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
//  193   {
//  194     if(stage == 1)
        MOV       A, [SP+0x18]       ;; 1 cycle
        CMP       A, #0x1            ;; 1 cycle
        SKZ                          ;; 4 cycles
        BR        N:??cal_angle_calculate_react_10  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  195     {
//  196       /* Voltage */
//  197       temp_us64 = ((us64)CAL_VOL * CAL_VOL);
        MOVW      HL, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, #0x1000        ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x2255        ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  198       temp_us64 *= cal_coeff.Rph.vol;
        MOVW      AX, N:_cal_coeff+6  ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+32
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+34
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
          CFI FunCall __L2LLU
        CALL      __L2LLU            ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      BC, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __Mul64
        CALL      __Mul64            ;; 3 cycles
//  199       temp_us64 /= ((us64)vol.Rph.rms*vol.Rph.rms);
        MOVW      AX, N:_vol         ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+36
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+38
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
          CFI FunCall __L2LLU
        CALL      __L2LLU            ;; 3 cycles
        MOVW      AX, N:_vol         ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+40
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+42
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x1C          ;; 1 cycle
          CFI FunCall __L2LLU
        CALL      __L2LLU            ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x1C          ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0xC           ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x14          ;; 1 cycle
          CFI FunCall __Mul64
        CALL      __Mul64            ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x14          ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      BC, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
//  200       cal_coeff.Rph.vol = temp_us64;
        MOVW      HL, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, [HL]           ;; 1 cycle
        MOVW      N:_cal_coeff+6, AX  ;; 1 cycle
//  201       
//  202       temp_us64 = ((us64)CAL_VOL * CAL_VOL);
        MOVW      HL, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, #0x1000        ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x2255        ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  203       temp_us64 *= cal_coeff.Yph.vol;
        MOVW      AX, N:_cal_coeff+18  ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+44
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+46
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
          CFI FunCall __L2LLU
        CALL      __L2LLU            ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      BC, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __Mul64
        CALL      __Mul64            ;; 3 cycles
//  204       temp_us64 /= ((us64)vol.Yph.rms*vol.Yph.rms);
        MOVW      AX, N:_vol+6       ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+48
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+50
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x14          ;; 1 cycle
          CFI FunCall __L2LLU
        CALL      __L2LLU            ;; 3 cycles
        MOVW      AX, N:_vol+6       ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+52
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+54
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x28          ;; 1 cycle
          CFI FunCall __L2LLU
        CALL      __L2LLU            ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x28          ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x18          ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x20          ;; 1 cycle
          CFI FunCall __Mul64
        CALL      __Mul64            ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x20          ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      BC, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
//  205       cal_coeff.Yph.vol = temp_us64;
        MOVW      HL, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, [HL]           ;; 1 cycle
        MOVW      N:_cal_coeff+18, AX  ;; 1 cycle
//  206       
//  207       temp_us64 = ((us64)CAL_VOL * CAL_VOL);
        MOVW      HL, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, #0x1000        ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x2255        ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  208       temp_us64 *= cal_coeff.Bph.vol;
        MOVW      AX, N:_cal_coeff+30  ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+56
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+58
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x1C          ;; 1 cycle
          CFI FunCall __L2LLU
        CALL      __L2LLU            ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x1C          ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      BC, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __Mul64
        CALL      __Mul64            ;; 3 cycles
//  209       temp_us64 /= ((us64)vol.Bph.rms*vol.Bph.rms);
        MOVW      AX, N:_vol+12      ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+60
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+62
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x20          ;; 1 cycle
          CFI FunCall __L2LLU
        CALL      __L2LLU            ;; 3 cycles
        MOVW      AX, N:_vol+12      ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+64
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+66
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x34          ;; 1 cycle
          CFI FunCall __L2LLU
        CALL      __L2LLU            ;; 3 cycles
        ADDW      SP, #0x24          ;; 1 cycle
          CFI CFA SP+30
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
          CFI FunCall __Mul64
        CALL      __Mul64            ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      BC, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
//  210       cal_coeff.Bph.vol = temp_us64;
        MOVW      HL, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, [HL]           ;; 1 cycle
        MOVW      N:_cal_coeff+30, AX  ;; 1 cycle
//  211       
//  212       cal_coeff.Rph.vol_offset = r_phase.vol.dc_offset;
        MOVW      AX, N:_r_phase+14  ;; 1 cycle
        MOVW      N:_cal_coeff+2, AX  ;; 1 cycle
//  213       cal_coeff.Yph.vol_offset = y_phase.vol.dc_offset;
        MOVW      AX, N:_y_phase+14  ;; 1 cycle
        MOVW      N:_cal_coeff+14, AX  ;; 1 cycle
//  214       cal_coeff.Bph.vol_offset = b_phase.vol.dc_offset;
        MOVW      AX, N:_b_phase+14  ;; 1 cycle
        MOVW      N:_cal_coeff+26, AX  ;; 1 cycle
//  215       
//  216       /* Current */
//  217       temp_us64 = ((us64)CAL_CURR * CAL_CURR);
        MOVW      HL, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, #0xE100        ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x5F5         ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  218       temp_us64 *= cal_coeff.Rph.curr;
        MOVW      AX, N:_cal_coeff+8  ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+32
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+34
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
          CFI FunCall __L2LLU
        CALL      __L2LLU            ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      BC, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __Mul64
        CALL      __Mul64            ;; 3 cycles
//  219       temp_us64 /= ((us64)curr.Rph.rms*curr.Rph.rms);
        MOVW      AX, N:_curr+2      ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+36
        MOVW      AX, N:_curr        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+38
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
          CFI FunCall __L2LLU
        CALL      __L2LLU            ;; 3 cycles
        MOVW      AX, N:_curr+2      ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+40
        MOVW      AX, N:_curr        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+42
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x1C          ;; 1 cycle
          CFI FunCall __L2LLU
        CALL      __L2LLU            ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x1C          ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0xC           ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x14          ;; 1 cycle
          CFI FunCall __Mul64
        CALL      __Mul64            ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x14          ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      BC, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
//  220       cal_coeff.Rph.curr = temp_us64;
        MOVW      HL, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, [HL]           ;; 1 cycle
        MOVW      N:_cal_coeff+8, AX  ;; 1 cycle
//  221       
//  222       temp_us64 = ((us64)CAL_CURR * CAL_CURR);
        MOVW      HL, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, #0xE100        ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x5F5         ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  223       temp_us64 *= cal_coeff.Yph.curr;
        MOVW      AX, N:_cal_coeff+20  ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+44
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+46
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
          CFI FunCall __L2LLU
        CALL      __L2LLU            ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      BC, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __Mul64
        CALL      __Mul64            ;; 3 cycles
//  224       temp_us64 /= ((us64)curr.Yph.rms*curr.Yph.rms);
        MOVW      AX, N:_curr+18     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+48
        MOVW      AX, N:_curr+16     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+50
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x14          ;; 1 cycle
          CFI FunCall __L2LLU
        CALL      __L2LLU            ;; 3 cycles
        MOVW      AX, N:_curr+18     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+52
        MOVW      AX, N:_curr+16     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+54
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x28          ;; 1 cycle
          CFI FunCall __L2LLU
        CALL      __L2LLU            ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x28          ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x18          ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x20          ;; 1 cycle
          CFI FunCall __Mul64
        CALL      __Mul64            ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x20          ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      BC, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
//  225       cal_coeff.Yph.curr = temp_us64;
        MOVW      HL, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, [HL]           ;; 1 cycle
        MOVW      N:_cal_coeff+20, AX  ;; 1 cycle
//  226       
//  227       temp_us64 = ((us64)CAL_CURR * CAL_CURR);
        MOVW      HL, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, #0xE100        ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x5F5         ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  228       temp_us64 *= cal_coeff.Bph.curr;
        MOVW      AX, N:_cal_coeff+32  ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+56
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+58
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x1C          ;; 1 cycle
          CFI FunCall __L2LLU
        CALL      __L2LLU            ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x1C          ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      BC, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __Mul64
        CALL      __Mul64            ;; 3 cycles
//  229       temp_us64 /= ((us64)curr.Bph.rms*curr.Bph.rms);
        MOVW      AX, N:_curr+34     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+60
        MOVW      AX, N:_curr+32     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+62
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x20          ;; 1 cycle
          CFI FunCall __L2LLU
        CALL      __L2LLU            ;; 3 cycles
        MOVW      AX, N:_curr+34     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+64
        MOVW      AX, N:_curr+32     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+66
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x34          ;; 1 cycle
          CFI FunCall __L2LLU
        CALL      __L2LLU            ;; 3 cycles
        ADDW      SP, #0x24          ;; 1 cycle
          CFI CFA SP+30
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
          CFI FunCall __Mul64
        CALL      __Mul64            ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      BC, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
//  230       cal_coeff.Bph.curr = temp_us64;
        MOVW      HL, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, [HL]           ;; 1 cycle
        MOVW      N:_cal_coeff+32, AX  ;; 1 cycle
//  231       
//  232       cal_coeff.Rph.curr_offset = r_phase.curr.dc_offset;
        MOVW      AX, N:_r_phase+46  ;; 1 cycle
        MOVW      N:_cal_coeff+4, AX  ;; 1 cycle
//  233       cal_coeff.Yph.curr_offset = y_phase.curr.dc_offset;
        MOVW      AX, N:_y_phase+46  ;; 1 cycle
        MOVW      N:_cal_coeff+16, AX  ;; 1 cycle
//  234       cal_coeff.Bph.curr_offset = b_phase.curr.dc_offset;
        MOVW      AX, N:_b_phase+46  ;; 1 cycle
        MOVW      N:_cal_coeff+28, AX  ;; 1 cycle
//  235       
//  236       /* Phase Compensation */
//  237       cal_RPh.angle_active = cal_angle_calculate_act(cal_RPh.l_shift.active.power,cal_RPh.r_shift.active.power);
        MOVW      AX, N:_cal_RPh+60  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
        MOVW      AX, N:_cal_RPh+58  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+34
        MOVW      BC, N:_cal_RPh+16  ;; 1 cycle
        MOVW      AX, N:_cal_RPh+14  ;; 1 cycle
          CFI FunCall _cal_angle_calculate_act
        CALL      _cal_angle_calculate_act  ;; 3 cycles
        MOVW      N:_cal_RPh+6, AX   ;; 1 cycle
//  238       cal_coeff.Rph.phase_correction = -(s16)(temp_double*RADIAN_TO_PH_VALUE); /* temp_double calculated from cal_angle_calculate_act */
        MOVW      AX, #0x4595        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+36
        MOVW      AX, #0x3530        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+38
        MOVW      BC, S:_temp_double+2  ;; 1 cycle
        MOVW      AX, S:_temp_double  ;; 1 cycle
          CFI FunCall ?F_MUL
        CALL      N:?F_MUL           ;; 3 cycles
          CFI FunCall ?F_F2SL
        CALL      N:?F_F2SL          ;; 3 cycles
        XOR       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        XOR       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        INCW      AX                 ;; 1 cycle
        MOVW      N:_cal_coeff+12, AX  ;; 1 cycle
//  239       if(cal_coeff.Rph.phase_correction > MAX_DSADPHCR_VALUE)
        MOVW      AX, N:_cal_coeff+12  ;; 1 cycle
        ADDW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+30
        XOR       A, #0x80           ;; 1 cycle
        CMPW      AX, #0x8480        ;; 1 cycle
        BC        ??cal_angle_calculate_react_11  ;; 4 cycles
        ; ------------------------------------- Block: 444 cycles
//  240       {
//  241         cal_coeff.Rph.phase_correction = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_cal_coeff+12, AX  ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
//  242       }
//  243       R_DSADC_update_phase_correction(PHASE_R,cal_coeff.Rph.phase_correction);
??cal_angle_calculate_react_11:
        MOVW      BC, N:_cal_coeff+12  ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _R_DSADC_update_phase_correction
        CALL      _R_DSADC_update_phase_correction  ;; 3 cycles
//  244       
//  245       cal_YPh.angle_active = cal_angle_calculate_act(cal_YPh.l_shift.active.power,cal_YPh.r_shift.active.power);
        MOVW      AX, N:_cal_YPh+60  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
        MOVW      AX, N:_cal_YPh+58  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+34
        MOVW      BC, N:_cal_YPh+16  ;; 1 cycle
        MOVW      AX, N:_cal_YPh+14  ;; 1 cycle
          CFI FunCall _cal_angle_calculate_act
        CALL      _cal_angle_calculate_act  ;; 3 cycles
        MOVW      N:_cal_YPh+6, AX   ;; 1 cycle
//  246       cal_coeff.Yph.phase_correction = -(s16)(temp_double*RADIAN_TO_PH_VALUE);
        MOVW      AX, #0x4595        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+36
        MOVW      AX, #0x3530        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+38
        MOVW      BC, S:_temp_double+2  ;; 1 cycle
        MOVW      AX, S:_temp_double  ;; 1 cycle
          CFI FunCall ?F_MUL
        CALL      N:?F_MUL           ;; 3 cycles
          CFI FunCall ?F_F2SL
        CALL      N:?F_F2SL          ;; 3 cycles
        XOR       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        XOR       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        INCW      AX                 ;; 1 cycle
        MOVW      N:_cal_coeff+24, AX  ;; 1 cycle
//  247       if(cal_coeff.Yph.phase_correction > MAX_DSADPHCR_VALUE)
        MOVW      AX, N:_cal_coeff+24  ;; 1 cycle
        ADDW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+30
        XOR       A, #0x80           ;; 1 cycle
        CMPW      AX, #0x8480        ;; 1 cycle
        BC        ??cal_angle_calculate_react_12  ;; 4 cycles
        ; ------------------------------------- Block: 41 cycles
//  248       {
//  249         cal_coeff.Yph.phase_correction = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_cal_coeff+24, AX  ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
//  250       }
//  251       R_DSADC_update_phase_correction(PHASE_Y,cal_coeff.Yph.phase_correction);
??cal_angle_calculate_react_12:
        MOVW      BC, N:_cal_coeff+24  ;; 1 cycle
        MOV       A, #0x1            ;; 1 cycle
          CFI FunCall _R_DSADC_update_phase_correction
        CALL      _R_DSADC_update_phase_correction  ;; 3 cycles
//  252       
//  253       cal_BPh.angle_active = cal_angle_calculate_act(cal_BPh.l_shift.active.power,cal_BPh.r_shift.active.power);
        MOVW      AX, N:_cal_BPh+60  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
        MOVW      AX, N:_cal_BPh+58  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+34
        MOVW      BC, N:_cal_BPh+16  ;; 1 cycle
        MOVW      AX, N:_cal_BPh+14  ;; 1 cycle
          CFI FunCall _cal_angle_calculate_act
        CALL      _cal_angle_calculate_act  ;; 3 cycles
        MOVW      N:_cal_BPh+6, AX   ;; 1 cycle
//  254       cal_coeff.Bph.phase_correction = -(s16)(temp_double*RADIAN_TO_PH_VALUE);
        MOVW      AX, #0x4595        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+36
        MOVW      AX, #0x3530        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+38
        MOVW      BC, S:_temp_double+2  ;; 1 cycle
        MOVW      AX, S:_temp_double  ;; 1 cycle
          CFI FunCall ?F_MUL
        CALL      N:?F_MUL           ;; 3 cycles
          CFI FunCall ?F_F2SL
        CALL      N:?F_F2SL          ;; 3 cycles
        XOR       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        XOR       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        INCW      AX                 ;; 1 cycle
        MOVW      N:_cal_coeff+36, AX  ;; 1 cycle
//  255       if(cal_coeff.Bph.phase_correction > MAX_DSADPHCR_VALUE)
        MOVW      AX, N:_cal_coeff+36  ;; 1 cycle
        ADDW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+30
        XOR       A, #0x80           ;; 1 cycle
        CMPW      AX, #0x8480        ;; 1 cycle
        BC        ??cal_angle_calculate_react_13  ;; 4 cycles
        ; ------------------------------------- Block: 41 cycles
//  256       {
//  257         cal_coeff.Bph.phase_correction = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_cal_coeff+36, AX  ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
//  258       }
//  259       R_DSADC_update_phase_correction(PHASE_B,cal_coeff.Bph.phase_correction);
??cal_angle_calculate_react_13:
        MOVW      BC, N:_cal_coeff+36  ;; 1 cycle
        MOV       A, #0x2            ;; 1 cycle
          CFI FunCall _R_DSADC_update_phase_correction
        CALL      _R_DSADC_update_phase_correction  ;; 3 cycles
        BR        N:??cal_angle_calculate_react_14  ;; 3 cycles
        ; ------------------------------------- Block: 8 cycles
//  260     }
//  261     else if(stage == 2)
??cal_angle_calculate_react_10:
        MOV       A, [SP+0x18]       ;; 1 cycle
        CMP       A, #0x2            ;; 1 cycle
        SKZ                          ;; 4 cycles
        BR        N:??cal_angle_calculate_react_14  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  262     {
//  263       /* Power */
//  264       temp_us64 = CAL_POWER;
        MOVW      HL, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, #0x5DC0        ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  265       temp_us64 *= cal_coeff.Rph.power;
        MOVW      AX, N:_cal_coeff+10  ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+32
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+34
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
          CFI FunCall __L2LLU
        CALL      __L2LLU            ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      BC, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __Mul64
        CALL      __Mul64            ;; 3 cycles
//  266       temp_us64 /= power.Rph.active;
        MOVW      AX, N:_power+2     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+36
        MOVW      AX, N:_power       ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+38
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
          CFI FunCall __L2LLU
        CALL      __L2LLU            ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      BC, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
//  267       cal_coeff.Rph.power = temp_us64;
        MOVW      HL, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, [HL]           ;; 1 cycle
        MOVW      N:_cal_coeff+10, AX  ;; 1 cycle
//  268       
//  269       temp_us64 = CAL_POWER;
        MOVW      HL, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, #0x5DC0        ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  270       temp_us64 *= cal_coeff.Yph.power;
        MOVW      AX, N:_cal_coeff+22  ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+40
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+42
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0xC           ;; 1 cycle
          CFI FunCall __L2LLU
        CALL      __L2LLU            ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0xC           ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      BC, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __Mul64
        CALL      __Mul64            ;; 3 cycles
//  271       temp_us64 /= power.Yph.active;
        MOVW      AX, N:_power+22    ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+44
        MOVW      AX, N:_power+20    ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+46
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
          CFI FunCall __L2LLU
        CALL      __L2LLU            ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      BC, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
//  272       cal_coeff.Yph.power = temp_us64;
        MOVW      HL, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, [HL]           ;; 1 cycle
        MOVW      N:_cal_coeff+22, AX  ;; 1 cycle
//  273       
//  274       temp_us64 = CAL_POWER;
        MOVW      HL, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, #0x5DC0        ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  275       temp_us64 *= cal_coeff.Bph.power;
        MOVW      AX, N:_cal_coeff+34  ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+48
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+50
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x14          ;; 1 cycle
          CFI FunCall __L2LLU
        CALL      __L2LLU            ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x14          ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      BC, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __Mul64
        CALL      __Mul64            ;; 3 cycles
//  276       temp_us64 /= power.Bph.active;
        MOVW      AX, N:_power+42    ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+52
        MOVW      AX, N:_power+40    ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+54
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x18          ;; 1 cycle
          CFI FunCall __L2LLU
        CALL      __L2LLU            ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x18          ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      BC, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
//  277       cal_coeff.Bph.power = temp_us64;
        MOVW      HL, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, [HL]           ;; 1 cycle
        MOVW      N:_cal_coeff+34, AX  ;; 1 cycle
        ADDW      SP, #0x18          ;; 1 cycle
          CFI CFA SP+30
        BR        N:??cal_angle_calculate_react_14  ;; 3 cycles
        ; ------------------------------------- Block: 142 cycles
//  278       
//  279     }
//  280   }
//  281   else if(phase == PHASE_N)
??cal_angle_calculate_react_9:
        MOV       A, [SP+0x19]       ;; 1 cycle
        CMP       A, #0x4            ;; 1 cycle
        SKZ                          ;; 4 cycles
        BR        N:??cal_angle_calculate_react_14  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  282   {
//  283     if(stage == 1)
        MOV       A, [SP+0x18]       ;; 1 cycle
        CMP       A, #0x1            ;; 1 cycle
        SKZ                          ;; 4 cycles
        BR        N:??cal_angle_calculate_react_14  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  284     {
//  285       /* Current */
//  286       temp_us64 = ((us64)CAL_CURR_NEU * CAL_CURR_NEU);
        MOVW      HL, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, #0xE100        ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x5F5         ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  287       temp_us64 *= cal_coeff.Nph.curr;
        MOVW      AX, N:_cal_coeff+40  ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+32
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+34
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
          CFI FunCall __L2LLU
        CALL      __L2LLU            ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      BC, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __Mul64
        CALL      __Mul64            ;; 3 cycles
//  288       temp_us64 /= ((us64)curr.Nph.rms*curr.Nph.rms);
        MOVW      AX, N:_curr+50     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+36
        MOVW      AX, N:_curr+48     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+38
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
          CFI FunCall __L2LLU
        CALL      __L2LLU            ;; 3 cycles
        MOVW      AX, N:_curr+50     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+40
        MOVW      AX, N:_curr+48     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+42
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x1C          ;; 1 cycle
          CFI FunCall __L2LLU
        CALL      __L2LLU            ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x1C          ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0xC           ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x14          ;; 1 cycle
          CFI FunCall __Mul64
        CALL      __Mul64            ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x14          ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      BC, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
//  289       cal_coeff.Nph.curr = temp_us64;
        MOVW      HL, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, [HL]           ;; 1 cycle
        MOVW      N:_cal_coeff+40, AX  ;; 1 cycle
//  290       
//  291       /* Offset */
//  292       cal_coeff.Nph.curr_offset = n_phase.curr.dc_offset;
        MOVW      AX, N:_n_phase+8   ;; 1 cycle
        MOVW      N:_cal_coeff+38, AX  ;; 1 cycle
        ADDW      SP, #0xC           ;; 1 cycle
          CFI CFA SP+30
        ; ------------------------------------- Block: 69 cycles
//  293     }
//  294   }
//  295 }
??cal_angle_calculate_react_14:
        ADDW      SP, #0x1A          ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock5
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 790 cycles
//  296 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock6 Using cfiCommon0
          CFI Function _calibration_delay_voltage_samples
        CODE
//  297 void calibration_delay_voltage_samples()
//  298 {
_calibration_delay_voltage_samples:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 2
        SUBW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+6
//  299   us8 ptr_vol_right_shift,ptr_vol_left_shift,ptr_curr;
//  300   
//  301   /* Finding pointers */
//  302   ptr_vol_right_shift   = (cal_buffer_ptr-1+CAL_BUF_SIZE_VOL)%CAL_BUF_SIZE_VOL;
        MOVW      BC, #0x1B          ;; 1 cycle
        MOV       X, N:_cal_buffer_ptr  ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        ADDW      AX, #0x1A          ;; 1 cycle
          CFI FunCall ?SI_MOD_L02
        CALL      N:?SI_MOD_L02      ;; 3 cycles
        MOVW      DE, AX             ;; 1 cycle
//  303   ptr_curr              = (ptr_vol_right_shift-CAL_DELAY_VOL+CAL_BUF_SIZE_VOL)%CAL_BUF_SIZE_VOL;
        MOVW      BC, #0x1B          ;; 1 cycle
        MOVW      AX, DE             ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        ADDW      AX, #0xE           ;; 1 cycle
          CFI FunCall ?SI_MOD_L02
        CALL      N:?SI_MOD_L02      ;; 3 cycles
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, L               ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  304   ptr_vol_left_shift    = (ptr_curr-CAL_DELAY_VOL+CAL_BUF_SIZE_VOL)%CAL_BUF_SIZE_VOL;
        MOVW      BC, #0x1B          ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        ADDW      AX, #0xE           ;; 1 cycle
          CFI FunCall ?SI_MOD_L02
        CALL      N:?SI_MOD_L02      ;; 3 cycles
        MOVW      HL, AX             ;; 1 cycle
        XCH       A, L               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        XCH       A, L               ;; 1 cycle
//  305   
//  306   /* R Phase */
//  307   /* reading sample from buffer */
//  308   cal_RPh.curr_sample           = cal_buffer_currr[ptr_curr];
        MOV       A, [SP]            ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(_cal_buffer_currr)  ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [HL]           ;; 1 cycle
        MOVW      N:_cal_RPh+4, AX   ;; 1 cycle
//  309   cal_RPh.l_shift.vol.sample    = cal_buffer_volr[ptr_vol_left_shift];
        MOV       A, D               ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(_cal_buffer_volr)  ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [HL]           ;; 1 cycle
        MOVW      N:_cal_RPh+10, AX  ;; 1 cycle
//  310   cal_RPh.r_shift.vol.sample    = cal_buffer_volr[ptr_vol_right_shift];
        MOVW      AX, DE             ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(_cal_buffer_volr)  ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [HL]           ;; 1 cycle
        MOVW      N:_cal_RPh+54, AX  ;; 1 cycle
//  311   cal_RPh.l_shift.vol.sample90  = cal_buffer_volr90[ptr_vol_left_shift];
        MOV       A, D               ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(_cal_buffer_volr90)  ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [HL]           ;; 1 cycle
        MOVW      N:_cal_RPh+12, AX  ;; 1 cycle
//  312   cal_RPh.r_shift.vol.sample90  = cal_buffer_volr90[ptr_vol_right_shift];
        MOVW      AX, DE             ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(_cal_buffer_volr90)  ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [HL]           ;; 1 cycle
        MOVW      N:_cal_RPh+56, AX  ;; 1 cycle
//  313   /* Filling buffer */
//  314   cal_buffer_volr[cal_buffer_ptr]     = r_phase.vol.sample;
        MOVW      AX, N:_r_phase+8   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+8
        MOV       X, N:_cal_buffer_ptr  ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(_cal_buffer_volr)  ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        POP       AX                 ;; 1 cycle
          CFI CFA SP+6
        MOVW      [HL], AX           ;; 1 cycle
//  315   cal_buffer_currr[cal_buffer_ptr]    = r_phase.curr.sample;
        MOVW      AX, N:_r_phase+44  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+8
        MOV       X, N:_cal_buffer_ptr  ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(_cal_buffer_currr)  ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        POP       AX                 ;; 1 cycle
          CFI CFA SP+6
        MOVW      [HL], AX           ;; 1 cycle
//  316   cal_buffer_volr90[cal_buffer_ptr]   = r_phase.vol.sample90;
        MOVW      AX, N:_r_phase+12  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+8
        MOV       X, N:_cal_buffer_ptr  ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(_cal_buffer_volr90)  ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        POP       AX                 ;; 1 cycle
          CFI CFA SP+6
        MOVW      [HL], AX           ;; 1 cycle
//  317   
//  318   /* Y Phase */
//  319   cal_YPh.curr_sample           = cal_buffer_curry[ptr_curr];
        MOV       A, [SP]            ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(_cal_buffer_curry)  ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [HL]           ;; 1 cycle
        MOVW      N:_cal_YPh+4, AX   ;; 1 cycle
//  320   cal_YPh.l_shift.vol.sample    = cal_buffer_voly[ptr_vol_left_shift];
        MOV       A, D               ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(_cal_buffer_voly)  ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [HL]           ;; 1 cycle
        MOVW      N:_cal_YPh+10, AX  ;; 1 cycle
//  321   cal_YPh.r_shift.vol.sample    = cal_buffer_voly[ptr_vol_right_shift];
        MOVW      AX, DE             ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(_cal_buffer_voly)  ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [HL]           ;; 1 cycle
        MOVW      N:_cal_YPh+54, AX  ;; 1 cycle
//  322   cal_YPh.l_shift.vol.sample90  = cal_buffer_voly90[ptr_vol_left_shift];
        MOV       A, D               ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(_cal_buffer_voly90)  ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [HL]           ;; 1 cycle
        MOVW      N:_cal_YPh+12, AX  ;; 1 cycle
//  323   cal_YPh.r_shift.vol.sample90  = cal_buffer_voly90[ptr_vol_right_shift];
        MOVW      AX, DE             ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(_cal_buffer_voly90)  ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [HL]           ;; 1 cycle
        MOVW      N:_cal_YPh+56, AX  ;; 1 cycle
//  324   
//  325   cal_buffer_voly[cal_buffer_ptr]     = y_phase.vol.sample;
        MOVW      AX, N:_y_phase+8   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+8
        MOV       X, N:_cal_buffer_ptr  ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(_cal_buffer_voly)  ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        POP       AX                 ;; 1 cycle
          CFI CFA SP+6
        MOVW      [HL], AX           ;; 1 cycle
//  326   cal_buffer_curry[cal_buffer_ptr]    = y_phase.curr.sample;
        MOVW      AX, N:_y_phase+44  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+8
        MOV       X, N:_cal_buffer_ptr  ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(_cal_buffer_curry)  ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        POP       AX                 ;; 1 cycle
          CFI CFA SP+6
        MOVW      [HL], AX           ;; 1 cycle
//  327   cal_buffer_voly90[cal_buffer_ptr]   = y_phase.vol.sample90;
        MOVW      AX, N:_y_phase+12  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+8
        MOV       X, N:_cal_buffer_ptr  ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(_cal_buffer_voly90)  ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        POP       AX                 ;; 1 cycle
          CFI CFA SP+6
        MOVW      [HL], AX           ;; 1 cycle
//  328   
//  329   /* B Phase */
//  330   cal_BPh.curr_sample           = cal_buffer_currb[ptr_curr];
        MOV       A, [SP]            ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(_cal_buffer_currb)  ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [HL]           ;; 1 cycle
        MOVW      N:_cal_BPh+4, AX   ;; 1 cycle
//  331   cal_BPh.l_shift.vol.sample    = cal_buffer_volb[ptr_vol_left_shift];
        MOV       A, D               ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(_cal_buffer_volb)  ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [HL]           ;; 1 cycle
        MOVW      N:_cal_BPh+10, AX  ;; 1 cycle
//  332   cal_BPh.r_shift.vol.sample    = cal_buffer_volb[ptr_vol_right_shift];
        MOVW      AX, DE             ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(_cal_buffer_volb)  ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [HL]           ;; 1 cycle
        MOVW      N:_cal_BPh+54, AX  ;; 1 cycle
//  333   cal_BPh.l_shift.vol.sample90  = cal_buffer_volb90[ptr_vol_left_shift];
        MOV       A, D               ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(_cal_buffer_volb90)  ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [HL]           ;; 1 cycle
        MOVW      N:_cal_BPh+12, AX  ;; 1 cycle
//  334   cal_BPh.r_shift.vol.sample90  = cal_buffer_volb90[ptr_vol_right_shift];
        MOVW      AX, DE             ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(_cal_buffer_volb90)  ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [HL]           ;; 1 cycle
        MOVW      N:_cal_BPh+56, AX  ;; 1 cycle
//  335   
//  336   cal_buffer_volb[cal_buffer_ptr]     = b_phase.vol.sample;
        MOVW      AX, N:_b_phase+8   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+8
        MOV       X, N:_cal_buffer_ptr  ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(_cal_buffer_volb)  ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        POP       AX                 ;; 1 cycle
          CFI CFA SP+6
        MOVW      [HL], AX           ;; 1 cycle
//  337   cal_buffer_currb[cal_buffer_ptr]    = b_phase.curr.sample;
        MOVW      AX, N:_b_phase+44  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+8
        MOV       X, N:_cal_buffer_ptr  ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(_cal_buffer_currb)  ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        POP       AX                 ;; 1 cycle
          CFI CFA SP+6
        MOVW      [HL], AX           ;; 1 cycle
//  338   cal_buffer_volb90[cal_buffer_ptr]   = b_phase.vol.sample90;
        MOVW      AX, N:_b_phase+12  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+8
        MOV       X, N:_cal_buffer_ptr  ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(_cal_buffer_volb90)  ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        POP       AX                 ;; 1 cycle
          CFI CFA SP+6
        MOVW      [HL], AX           ;; 1 cycle
//  339   
//  340   /* increasing pointer */
//  341   cal_buffer_ptr++;
        INC       N:_cal_buffer_ptr  ;; 2 cycles
//  342   if(cal_buffer_ptr >= CAL_BUF_SIZE_VOL)
        MOV       A, N:_cal_buffer_ptr  ;; 1 cycle
        CMP       A, #0x1B           ;; 1 cycle
        SKC                          ;; 1 cycle
        ; ------------------------------------- Block: 279 cycles
//  343   {
//  344     cal_buffer_ptr = 0;
        MOV       N:_cal_buffer_ptr, #0x0  ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
//  345   }
//  346 }
??calibration_delay_voltage_samples_0:
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock6
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 287 cycles

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock7 Using cfiCommon0
          CFI Function _calibration_1sec_loop
        CODE
//  347 void calibration_1sec_loop()
//  348 {
_calibration_1sec_loop:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  349   if(cal_timer < CAL_TIMER_MAX && flag_cal_error_f == 0)
        MOV       A, N:_cal_timer    ;; 1 cycle
        CMP       A, #0x14           ;; 1 cycle
        SKC                          ;; 4 cycles
        BR        N:??cal_angle_calculate_react_15  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      HL, #LWRD(_flag_calibration)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        SKNC                         ;; 4 cycles
        BR        N:??cal_angle_calculate_react_15  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  350   {
//  351     cal_timer++;
        INC       N:_cal_timer       ;; 2 cycles
//  352     if(cal_done_f == 0)                                 /* Phases */
        CMP0      N:_cal_done_f      ;; 1 cycle
        SKZ                          ;; 4 cycles
        BR        N:??cal_angle_calculate_react_16  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  353     {
//  354       if(cal_timer == 6)                                  /* entering calibration */
        CMP       N:_cal_timer, #0x6  ;; 1 cycle
        BNZ       ??cal_angle_calculate_react_17  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  355       {
//  356         if(flag_calibration_r_vi_ok == 0 || flag_calibration_y_vi_ok == 0 || flag_calibration_b_vi_ok == 0)       /* if voltage and currents are ok */
        MOV       A, N:_flag_cal     ;; 1 cycle
        AND       A, #0x7            ;; 1 cycle
        CMP       A, #0x7            ;; 1 cycle
        BZ        ??cal_angle_calculate_react_18  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  357         {
//  358           flag_cal_error_f = 1;                         /* exiting calibration */
        SET1      N:_flag_calibration.0  ;; 2 cycles
//  359           cal_error_code = 1;                           /* instant values */
        MOV       N:_cal_error_code, #0x1  ;; 1 cycle
        BR        S:??cal_angle_calculate_react_17  ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
//  360         }
//  361         else
//  362         {
//  363           cal_stage = 0;
??cal_angle_calculate_react_18:
        MOV       N:_cal_stage, #0x0  ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
//  364         }
//  365       }
//  366       if(cal_timer == 8 && cal_stage == 0)
??cal_angle_calculate_react_17:
        CMP       N:_cal_timer, #0x8  ;; 1 cycle
        BNZ       ??cal_angle_calculate_react_19  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        CMP0      N:_cal_stage       ;; 1 cycle
        BNZ       ??cal_angle_calculate_react_19  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  367       {
//  368         if(flag_calibration_r_vi_ok == 1 && flag_calibration_y_vi_ok == 1 && flag_calibration_b_vi_ok == 1 &&
//  369            flag_calibration_r_pf_freq_pow_ok == 1 && flag_calibration_y_pf_freq_pow_ok == 1 && flag_calibration_b_pf_freq_pow_ok == 1)
        MOV       A, N:_flag_cal     ;; 1 cycle
        AND       A, #0x7            ;; 1 cycle
        CMP       A, #0x7            ;; 1 cycle
        BNZ       ??cal_angle_calculate_react_20  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOV       A, N:_flag_calibration  ;; 1 cycle
        AND       A, #0x1C           ;; 1 cycle
        CMP       A, #0x1C           ;; 1 cycle
        BNZ       ??cal_angle_calculate_react_20  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  370         {
//  371           if(flag_calibration_r_ct_ok == 1 && flag_calibration_y_ct_ok == 1 && flag_calibration_b_ct_ok == 1)
        MOV       A, N:_flag_calibration  ;; 1 cycle
        AND       A, #0xE0           ;; 1 cycle
        CMP       A, #0xE0           ;; 1 cycle
        BNZ       ??cal_angle_calculate_react_21  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  372           {
//  373             cal_stage = 1;
        MOV       N:_cal_stage, #0x1  ;; 1 cycle
//  374             calibration_process(PHASE_ALL,cal_stage);
        MOV       X, N:_cal_stage    ;; 1 cycle
        MOV       A, #0x3            ;; 1 cycle
          CFI FunCall _calibration_process
        CALL      _calibration_process  ;; 3 cycles
//  375             if(cal_coeff_range_check() != 1)
          CFI FunCall _cal_coeff_range_check
        CALL      _cal_coeff_range_check  ;; 3 cycles
        CMP       A, #0x1            ;; 1 cycle
        BZ        ??cal_angle_calculate_react_19  ;; 4 cycles
        ; ------------------------------------- Block: 14 cycles
//  376             {
//  377               flag_cal_error_f = 1;
        SET1      N:_flag_calibration.0  ;; 2 cycles
//  378               cal_error_code = 4;                       /* Range error */
        MOV       N:_cal_error_code, #0x4  ;; 1 cycle
//  379               calibration_reload_def_coeff();
          CFI FunCall _calibration_reload_def_coeff
        CALL      _calibration_reload_def_coeff  ;; 3 cycles
        BR        S:??cal_angle_calculate_react_19  ;; 3 cycles
        ; ------------------------------------- Block: 9 cycles
//  380             }
//  381           }
//  382           else
//  383           {
//  384             flag_cal_error_f = 1;
??cal_angle_calculate_react_21:
        SET1      N:_flag_calibration.0  ;; 2 cycles
//  385             cal_error_code = 2;                         /* CT Rev */
        MOV       N:_cal_error_code, #0x2  ;; 1 cycle
        BR        S:??cal_angle_calculate_react_19  ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
//  386           }
//  387         }
//  388         else
//  389         {
//  390           flag_cal_error_f = 1;
??cal_angle_calculate_react_20:
        SET1      N:_flag_calibration.0  ;; 2 cycles
//  391           cal_error_code = 1;
        MOV       N:_cal_error_code, #0x1  ;; 1 cycle
        ; ------------------------------------- Block: 3 cycles
//  392         }
//  393       }
//  394       if(cal_timer == 14 && cal_stage == 1)
??cal_angle_calculate_react_19:
        CMP       N:_cal_timer, #0xE  ;; 1 cycle
        BNZ       ??cal_angle_calculate_react_22  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        CMP       N:_cal_stage, #0x1  ;; 1 cycle
        BNZ       ??cal_angle_calculate_react_22  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  395       {
//  396         if(flag_calibration_r_vi_ok == 1 && flag_calibration_y_vi_ok == 1 && flag_calibration_b_vi_ok == 1 &&
//  397            flag_calibration_r_pf_freq_pow_ok == 1 && flag_calibration_y_pf_freq_pow_ok == 1 && flag_calibration_b_pf_freq_pow_ok == 1)
        MOV       A, N:_flag_cal     ;; 1 cycle
        AND       A, #0x7            ;; 1 cycle
        CMP       A, #0x7            ;; 1 cycle
        BNZ       ??cal_angle_calculate_react_23  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOV       A, N:_flag_calibration  ;; 1 cycle
        AND       A, #0x1C           ;; 1 cycle
        CMP       A, #0x1C           ;; 1 cycle
        BNZ       ??cal_angle_calculate_react_23  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  398         {
//  399           if(flag_calibration_r_ct_ok == 1 && flag_calibration_y_ct_ok == 1 && flag_calibration_b_ct_ok == 1)
        MOV       A, N:_flag_calibration  ;; 1 cycle
        AND       A, #0xE0           ;; 1 cycle
        CMP       A, #0xE0           ;; 1 cycle
        BNZ       ??cal_angle_calculate_react_24  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  400           {
//  401             cal_stage = 2;
        MOV       N:_cal_stage, #0x2  ;; 1 cycle
//  402             calibration_process(PHASE_ALL,cal_stage);
        MOV       X, N:_cal_stage    ;; 1 cycle
        MOV       A, #0x3            ;; 1 cycle
          CFI FunCall _calibration_process
        CALL      _calibration_process  ;; 3 cycles
//  403             if(cal_coeff_range_check() != 1)
          CFI FunCall _cal_coeff_range_check
        CALL      _cal_coeff_range_check  ;; 3 cycles
        CMP       A, #0x1            ;; 1 cycle
        BZ        ??cal_angle_calculate_react_22  ;; 4 cycles
        ; ------------------------------------- Block: 14 cycles
//  404             {
//  405               flag_cal_error_f = 1;
        SET1      N:_flag_calibration.0  ;; 2 cycles
//  406               cal_error_code = 4;                       /* Range error */
        MOV       N:_cal_error_code, #0x4  ;; 1 cycle
//  407               calibration_reload_def_coeff();
          CFI FunCall _calibration_reload_def_coeff
        CALL      _calibration_reload_def_coeff  ;; 3 cycles
        BR        S:??cal_angle_calculate_react_22  ;; 3 cycles
        ; ------------------------------------- Block: 9 cycles
//  408             }
//  409           }
//  410           else
//  411           {
//  412             flag_cal_error_f = 1;
??cal_angle_calculate_react_24:
        SET1      N:_flag_calibration.0  ;; 2 cycles
//  413             cal_error_code = 2;                       /* CT Rev */
        MOV       N:_cal_error_code, #0x2  ;; 1 cycle
        BR        S:??cal_angle_calculate_react_22  ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
//  414           }
//  415         }
//  416         else
//  417         {
//  418           flag_cal_error_f = 1;
??cal_angle_calculate_react_23:
        SET1      N:_flag_calibration.0  ;; 2 cycles
//  419           cal_error_code = 1;                         /* Instant values */
        MOV       N:_cal_error_code, #0x1  ;; 1 cycle
        ; ------------------------------------- Block: 3 cycles
//  420         }
//  421       }
//  422       if(cal_timer == 16 && cal_stage == 2)
??cal_angle_calculate_react_22:
        CMP       N:_cal_timer, #0x10  ;; 1 cycle
        BNZ       ??cal_angle_calculate_react_25  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        CMP       N:_cal_stage, #0x2  ;; 1 cycle
        BNZ       ??cal_angle_calculate_react_25  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  423       {
//  424           if(CAL_NEUTRAL == 1)
//  425           {
//  426               cal_done_f = 2;
//  427           }
//  428           else
//  429           {
//  430               cal_done_f = 1;
        MOV       N:_cal_done_f, #0x1  ;; 1 cycle
//  431           }
//  432         /* saving the parameters into memory */
//  433         calibration_save();
          CFI FunCall _calibration_save
        CALL      _calibration_save  ;; 3 cycles
        BR        S:??cal_angle_calculate_react_25  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
//  434       }
//  435     }
//  436     else if(cal_done_f == 3)                            /* Neutral */
??cal_angle_calculate_react_16:
        CMP       N:_cal_done_f, #0x3  ;; 1 cycle
        BNZ       ??cal_angle_calculate_react_25  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  437     {
//  438       if(cal_timer == 8 && cal_stage == 0)
        CMP       N:_cal_timer, #0x8  ;; 1 cycle
        BNZ       ??cal_angle_calculate_react_26  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        CMP0      N:_cal_stage       ;; 1 cycle
        BNZ       ??cal_angle_calculate_react_26  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  439       {
//  440         if(flag_calibration_n_i_ok == 1)
        MOVW      HL, #LWRD(_flag_calibration)  ;; 1 cycle
        MOV1      CY, [HL].1         ;; 1 cycle
        BNC       ??cal_angle_calculate_react_27  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  441         {
//  442           cal_stage = 1;
        MOV       N:_cal_stage, #0x1  ;; 1 cycle
//  443           calibration_process(PHASE_N,cal_stage);
        MOV       X, N:_cal_stage    ;; 1 cycle
        MOV       A, #0x4            ;; 1 cycle
          CFI FunCall _calibration_process
        CALL      _calibration_process  ;; 3 cycles
//  444           if(cal_coeff_range_check() != 1)
          CFI FunCall _cal_coeff_range_check
        CALL      _cal_coeff_range_check  ;; 3 cycles
        CMP       A, #0x1            ;; 1 cycle
        BZ        ??cal_angle_calculate_react_26  ;; 4 cycles
        ; ------------------------------------- Block: 14 cycles
//  445           {
//  446             flag_cal_error_f = 1;
        SET1      N:_flag_calibration.0  ;; 2 cycles
//  447             cal_error_code = 4;                       /* Range error */
        MOV       N:_cal_error_code, #0x4  ;; 1 cycle
//  448             calibration_reload_def_coeff();
          CFI FunCall _calibration_reload_def_coeff
        CALL      _calibration_reload_def_coeff  ;; 3 cycles
        BR        S:??cal_angle_calculate_react_26  ;; 3 cycles
        ; ------------------------------------- Block: 9 cycles
//  449           }
//  450         }
//  451         else
//  452         {
//  453           flag_cal_error_f = 1;
??cal_angle_calculate_react_27:
        SET1      N:_flag_calibration.0  ;; 2 cycles
//  454           cal_error_code = 3;                         /* Neu error */
        MOV       N:_cal_error_code, #0x3  ;; 1 cycle
        ; ------------------------------------- Block: 3 cycles
//  455         }
//  456       }
//  457       if(cal_timer == 10 && cal_stage == 1)
??cal_angle_calculate_react_26:
        CMP       N:_cal_timer, #0xA  ;; 1 cycle
        BNZ       ??cal_angle_calculate_react_25  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        CMP       N:_cal_stage, #0x1  ;; 1 cycle
        BNZ       ??cal_angle_calculate_react_25  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  458       {
//  459         cal_done_f = 1;
        MOV       N:_cal_done_f, #0x1  ;; 1 cycle
//  460         /* saving the parameters into memory */
//  461         calibration_save();
          CFI FunCall _calibration_save
        CALL      _calibration_save  ;; 3 cycles
        ; ------------------------------------- Block: 4 cycles
//  462       }
//  463     }
//  464     if(cal_timer >= 6)
??cal_angle_calculate_react_25:
        MOV       A, N:_cal_timer    ;; 1 cycle
        CMP       A, #0x6            ;; 1 cycle
        BC        ??cal_angle_calculate_react_15  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  465     {
//  466       if(cal_done_f == 0 && cal_timer == 6 &&(flag_calibration_r_vi_ok == 0 || flag_calibration_y_vi_ok == 0 || flag_calibration_b_vi_ok == 0))
        CMP0      N:_cal_done_f      ;; 1 cycle
        BNZ       ??cal_angle_calculate_react_28  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        CMP       N:_cal_timer, #0x6  ;; 1 cycle
        BNZ       ??cal_angle_calculate_react_28  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        MOV       A, N:_flag_cal     ;; 1 cycle
        AND       A, #0x7            ;; 1 cycle
        CMP       A, #0x7            ;; 1 cycle
        BZ        ??cal_angle_calculate_react_28  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  467       {
//  468         NOP();
        NOP                          ;; 1 cycle
        RET                          ;; 6 cycles
        ; ------------------------------------- Block: 7 cycles
//  469       }
//  470       else
//  471       {
//  472         lcd_write_msg(311,2);
??cal_angle_calculate_react_28:
        MOV       C, #0x2            ;; 1 cycle
        MOVW      AX, #0x137         ;; 1 cycle
          CFI FunCall _lcd_write_msg
        CALL      _lcd_write_msg     ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  473       }
//  474     }
//  475   }
//  476 }
??cal_angle_calculate_react_15:
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock7
        ; ------------------------------------- Block: 6 cycles
        ; ------------------------------------- Total: 283 cycles
//  477 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock8 Using cfiCommon0
          CFI Function _cal_coeff_range_check
          CFI NoCalls
        CODE
//  478 us8 cal_coeff_range_check()
//  479 {
_cal_coeff_range_check:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  480   if(IGNORE_RANGE_CHECKS == 1)
//  481   {
//  482     return 1;
//  483   }
//  484   
//  485   if(cal_coeff.Rph.vol < THR_VOL_RMS_COFF_MIN || cal_coeff.Rph.vol > THR_VOL_RMS_COFF_MAX ||
//  486      cal_coeff.Yph.vol < THR_VOL_RMS_COFF_MIN || cal_coeff.Yph.vol > THR_VOL_RMS_COFF_MAX ||
//  487        cal_coeff.Bph.vol < THR_VOL_RMS_COFF_MIN || cal_coeff.Bph.vol > THR_VOL_RMS_COFF_MAX)
        MOVW      AX, N:_cal_coeff+6  ;; 1 cycle
        CMPW      AX, #0x7918        ;; 1 cycle
        BC        ??cal_angle_calculate_react_29  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, N:_cal_coeff+6  ;; 1 cycle
        CMPW      AX, #0x9089        ;; 1 cycle
        BNC       ??cal_angle_calculate_react_29  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, N:_cal_coeff+18  ;; 1 cycle
        CMPW      AX, #0x7918        ;; 1 cycle
        BC        ??cal_angle_calculate_react_29  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, N:_cal_coeff+18  ;; 1 cycle
        CMPW      AX, #0x9089        ;; 1 cycle
        BNC       ??cal_angle_calculate_react_29  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, N:_cal_coeff+30  ;; 1 cycle
        CMPW      AX, #0x7918        ;; 1 cycle
        BC        ??cal_angle_calculate_react_29  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, N:_cal_coeff+30  ;; 1 cycle
        CMPW      AX, #0x9089        ;; 1 cycle
        BC        ??cal_angle_calculate_react_30  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  488   {
//  489     return 2;
??cal_angle_calculate_react_29:
        MOV       A, #0x2            ;; 1 cycle
        RET                          ;; 6 cycles
        ; ------------------------------------- Block: 7 cycles
//  490   }
//  491   
//  492   if(cal_coeff.Rph.vol_offset < THR_VOL_OFF_MIN || cal_coeff.Rph.vol_offset > THR_VOL_OFF_MAX ||
//  493      cal_coeff.Yph.vol_offset < THR_VOL_OFF_MIN || cal_coeff.Yph.vol_offset > THR_VOL_OFF_MAX ||
//  494        cal_coeff.Bph.vol_offset < THR_VOL_OFF_MIN || cal_coeff.Bph.vol_offset > THR_VOL_OFF_MAX)
??cal_angle_calculate_react_30:
        MOVW      AX, N:_cal_coeff+2  ;; 1 cycle
        XOR       A, #0x80           ;; 1 cycle
        CMPW      AX, #0x7F9C        ;; 1 cycle
        BC        ??cal_angle_calculate_react_31  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOVW      AX, N:_cal_coeff+2  ;; 1 cycle
        XOR       A, #0x80           ;; 1 cycle
        CMPW      AX, #0x8065        ;; 1 cycle
        BNC       ??cal_angle_calculate_react_31  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOVW      AX, N:_cal_coeff+14  ;; 1 cycle
        XOR       A, #0x80           ;; 1 cycle
        CMPW      AX, #0x7F9C        ;; 1 cycle
        BC        ??cal_angle_calculate_react_31  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOVW      AX, N:_cal_coeff+14  ;; 1 cycle
        XOR       A, #0x80           ;; 1 cycle
        CMPW      AX, #0x8065        ;; 1 cycle
        BNC       ??cal_angle_calculate_react_31  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOVW      AX, N:_cal_coeff+26  ;; 1 cycle
        XOR       A, #0x80           ;; 1 cycle
        CMPW      AX, #0x7F9C        ;; 1 cycle
        BC        ??cal_angle_calculate_react_31  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOVW      AX, N:_cal_coeff+26  ;; 1 cycle
        XOR       A, #0x80           ;; 1 cycle
        CMPW      AX, #0x8065        ;; 1 cycle
        BC        ??cal_angle_calculate_react_32  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  495   {
//  496     return 3;
??cal_angle_calculate_react_31:
        MOV       A, #0x3            ;; 1 cycle
        RET                          ;; 6 cycles
        ; ------------------------------------- Block: 7 cycles
//  497   }
//  498   
//  499   if(cal_coeff.Rph.curr < THR_CURR_RMS_COFF_MIN || cal_coeff.Rph.curr > THR_CURR_RMS_COFF_MAX ||
//  500      cal_coeff.Yph.curr < THR_CURR_RMS_COFF_MIN || cal_coeff.Yph.curr > THR_CURR_RMS_COFF_MAX ||
//  501        cal_coeff.Bph.curr < THR_CURR_RMS_COFF_MIN || cal_coeff.Bph.curr > THR_CURR_RMS_COFF_MAX)
??cal_angle_calculate_react_32:
        MOVW      AX, N:_cal_coeff+8  ;; 1 cycle
        CMPW      AX, #0x4A38        ;; 1 cycle
        BC        ??cal_angle_calculate_react_33  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, N:_cal_coeff+8  ;; 1 cycle
        CMPW      AX, #0x59D9        ;; 1 cycle
        BNC       ??cal_angle_calculate_react_33  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, N:_cal_coeff+20  ;; 1 cycle
        CMPW      AX, #0x4A38        ;; 1 cycle
        BC        ??cal_angle_calculate_react_33  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, N:_cal_coeff+20  ;; 1 cycle
        CMPW      AX, #0x59D9        ;; 1 cycle
        BNC       ??cal_angle_calculate_react_33  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, N:_cal_coeff+32  ;; 1 cycle
        CMPW      AX, #0x4A38        ;; 1 cycle
        BC        ??cal_angle_calculate_react_33  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, N:_cal_coeff+32  ;; 1 cycle
        CMPW      AX, #0x59D9        ;; 1 cycle
        BC        ??cal_angle_calculate_react_34  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  502   {
//  503     return 4;
??cal_angle_calculate_react_33:
        MOV       A, #0x4            ;; 1 cycle
        RET                          ;; 6 cycles
        ; ------------------------------------- Block: 7 cycles
//  504   }
//  505   
//  506   if(cal_coeff.Rph.curr_offset < THR_CURR_OFF_MIN || cal_coeff.Rph.curr_offset > THR_CURR_OFF_MAX ||
//  507      cal_coeff.Yph.curr_offset < THR_CURR_OFF_MIN || cal_coeff.Yph.curr_offset > THR_CURR_OFF_MAX ||
//  508        cal_coeff.Bph.curr_offset < THR_CURR_OFF_MIN || cal_coeff.Bph.curr_offset > THR_CURR_OFF_MAX)
??cal_angle_calculate_react_34:
        MOVW      AX, N:_cal_coeff+4  ;; 1 cycle
        XOR       A, #0x80           ;; 1 cycle
        CMPW      AX, #0x7DA8        ;; 1 cycle
        BC        ??cal_angle_calculate_react_35  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOVW      AX, N:_cal_coeff+4  ;; 1 cycle
        XOR       A, #0x80           ;; 1 cycle
        CMPW      AX, #0x8259        ;; 1 cycle
        BNC       ??cal_angle_calculate_react_35  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOVW      AX, N:_cal_coeff+16  ;; 1 cycle
        XOR       A, #0x80           ;; 1 cycle
        CMPW      AX, #0x7DA8        ;; 1 cycle
        BC        ??cal_angle_calculate_react_35  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOVW      AX, N:_cal_coeff+16  ;; 1 cycle
        XOR       A, #0x80           ;; 1 cycle
        CMPW      AX, #0x8259        ;; 1 cycle
        BNC       ??cal_angle_calculate_react_35  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOVW      AX, N:_cal_coeff+28  ;; 1 cycle
        XOR       A, #0x80           ;; 1 cycle
        CMPW      AX, #0x7DA8        ;; 1 cycle
        BC        ??cal_angle_calculate_react_35  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOVW      AX, N:_cal_coeff+28  ;; 1 cycle
        XOR       A, #0x80           ;; 1 cycle
        CMPW      AX, #0x8259        ;; 1 cycle
        BC        ??cal_angle_calculate_react_36  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  509   {
//  510     return 5;
??cal_angle_calculate_react_35:
        MOV       A, #0x5            ;; 1 cycle
        RET                          ;; 6 cycles
        ; ------------------------------------- Block: 7 cycles
//  511   }
//  512   
//  513   if(cal_coeff.Rph.phase_correction < THR_PHASE_CORRECT_MIN || cal_coeff.Rph.phase_correction > THR_PHASE_CORRECT_MAX ||
//  514      cal_coeff.Yph.phase_correction < THR_PHASE_CORRECT_MIN || cal_coeff.Yph.phase_correction > THR_PHASE_CORRECT_MAX ||
//  515        cal_coeff.Bph.phase_correction < THR_PHASE_CORRECT_MIN || cal_coeff.Bph.phase_correction > THR_PHASE_CORRECT_MAX)
??cal_angle_calculate_react_36:
        MOVW      AX, N:_cal_coeff+12  ;; 1 cycle
        XOR       A, #0x80           ;; 1 cycle
        CMPW      AX, #0x800A        ;; 1 cycle
        BC        ??cal_angle_calculate_react_37  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOVW      AX, N:_cal_coeff+12  ;; 1 cycle
        XOR       A, #0x80           ;; 1 cycle
        CMPW      AX, #0x83E9        ;; 1 cycle
        BNC       ??cal_angle_calculate_react_37  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOVW      AX, N:_cal_coeff+24  ;; 1 cycle
        XOR       A, #0x80           ;; 1 cycle
        CMPW      AX, #0x800A        ;; 1 cycle
        BC        ??cal_angle_calculate_react_37  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOVW      AX, N:_cal_coeff+24  ;; 1 cycle
        XOR       A, #0x80           ;; 1 cycle
        CMPW      AX, #0x83E9        ;; 1 cycle
        BNC       ??cal_angle_calculate_react_37  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOVW      AX, N:_cal_coeff+36  ;; 1 cycle
        XOR       A, #0x80           ;; 1 cycle
        CMPW      AX, #0x800A        ;; 1 cycle
        BC        ??cal_angle_calculate_react_37  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOVW      AX, N:_cal_coeff+36  ;; 1 cycle
        XOR       A, #0x80           ;; 1 cycle
        CMPW      AX, #0x83E9        ;; 1 cycle
        BC        ??cal_angle_calculate_react_38  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  516   {
//  517     return 6; 
??cal_angle_calculate_react_37:
        MOV       A, #0x6            ;; 1 cycle
        RET                          ;; 6 cycles
        ; ------------------------------------- Block: 7 cycles
//  518   }
//  519   
//  520   if(cal_coeff.Rph.power < THR_POW_COFF_MIN || cal_coeff.Rph.power > THR_POW_COFF_MAX ||
//  521      cal_coeff.Yph.power < THR_POW_COFF_MIN || cal_coeff.Yph.power > THR_POW_COFF_MAX ||
//  522        cal_coeff.Bph.power < THR_POW_COFF_MIN || cal_coeff.Bph.power > THR_POW_COFF_MAX)
??cal_angle_calculate_react_38:
        MOVW      AX, N:_cal_coeff+10  ;; 1 cycle
        CMPW      AX, #0x1B58        ;; 1 cycle
        BC        ??cal_angle_calculate_react_39  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, N:_cal_coeff+10  ;; 1 cycle
        CMPW      AX, #0x2711        ;; 1 cycle
        BNC       ??cal_angle_calculate_react_39  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, N:_cal_coeff+22  ;; 1 cycle
        CMPW      AX, #0x1B58        ;; 1 cycle
        BC        ??cal_angle_calculate_react_39  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, N:_cal_coeff+22  ;; 1 cycle
        CMPW      AX, #0x2711        ;; 1 cycle
        BNC       ??cal_angle_calculate_react_39  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, N:_cal_coeff+34  ;; 1 cycle
        CMPW      AX, #0x1B58        ;; 1 cycle
        BC        ??cal_angle_calculate_react_39  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, N:_cal_coeff+34  ;; 1 cycle
        CMPW      AX, #0x2711        ;; 1 cycle
        BC        ??cal_angle_calculate_react_40  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  523   {
//  524     return 7;
??cal_angle_calculate_react_39:
        MOV       A, #0x7            ;; 1 cycle
        RET                          ;; 6 cycles
        ; ------------------------------------- Block: 7 cycles
//  525   }
//  526   if(cal_coeff.Nph.curr < THR_CURR_NEU_RMS_COFF_MIN || cal_coeff.Nph.curr > THR_CURR_NEU_RMS_COFF_MAX ||
//  527      cal_coeff.Nph.curr_offset < THR_CURR_NEU_OFF_MIN || cal_coeff.Nph.curr_offset > THR_CURR_NEU_OFF_MAX)
??cal_angle_calculate_react_40:
        MOVW      AX, N:_cal_coeff+40  ;; 1 cycle
        CMPW      AX, #0x5DC0        ;; 1 cycle
        BC        ??cal_angle_calculate_react_41  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, N:_cal_coeff+40  ;; 1 cycle
        CMPW      AX, #0x6D61        ;; 1 cycle
        BNC       ??cal_angle_calculate_react_41  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, N:_cal_coeff+38  ;; 1 cycle
        XOR       A, #0x80           ;; 1 cycle
        CMPW      AX, #0x7E70        ;; 1 cycle
        BC        ??cal_angle_calculate_react_41  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOVW      AX, N:_cal_coeff+38  ;; 1 cycle
        XOR       A, #0x80           ;; 1 cycle
        CMPW      AX, #0x8191        ;; 1 cycle
        BC        ??cal_angle_calculate_react_42  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  528   {
//  529     return 8;
??cal_angle_calculate_react_41:
        MOV       A, #0x8            ;; 1 cycle
        RET                          ;; 6 cycles
        ; ------------------------------------- Block: 7 cycles
//  530   }
//  531   return 1;
??cal_angle_calculate_react_42:
        MOV       A, #0x1            ;; 1 cycle
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock8
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 316 cycles
//  532 }
//  533 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock9 Using cfiCommon1
          CFI Function _cal_angle_calculate_act
        CODE
//  534 s16 cal_angle_calculate_act(s32 left_power,s32 right_power)
//  535 {
_cal_angle_calculate_act:
        ; * Stack frame (at entry) *
        ; Param size: 4
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+6
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+8
        ; Auto size: 4
//  536     s16 angle;
//  537     temp_double = (double)left_power/right_power;
        MOVW      AX, [SP+0x0A]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x08]      ;; 1 cycle
          CFI FunCall ?F_SL2F
        CALL      N:?F_SL2F          ;; 3 cycles
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+10
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
          CFI FunCall ?F_SL2F
        CALL      N:?F_SL2F          ;; 3 cycles
          CFI FunCall ?F_DIV
        CALL      N:?F_DIV           ;; 3 cycles
        MOVW      S:_temp_double, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_temp_double+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  538     //temp_double = 1 - ((double)2/(temp_double+1));
//  539     temp_double = (temp_double-1)/(temp_double+1);
        MOVW      AX, #0x3F80        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+14
        MOVW      AX, #0x0           ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+16
        MOVW      BC, S:_temp_double+2  ;; 1 cycle
        MOVW      AX, S:_temp_double  ;; 1 cycle
          CFI FunCall ?F_ADD
        CALL      N:?F_ADD           ;; 3 cycles
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+18
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+20
        MOVW      AX, #0xBF80        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+22
        MOVW      AX, #0x0           ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+24
        MOVW      BC, S:_temp_double+2  ;; 1 cycle
        MOVW      AX, S:_temp_double  ;; 1 cycle
          CFI FunCall ?F_ADD
        CALL      N:?F_ADD           ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+20
          CFI FunCall ?F_DIV
        CALL      N:?F_DIV           ;; 3 cycles
        MOVW      S:_temp_double, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_temp_double+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  540     //temp_double *= ONE_BY_TAN_B;
//  541     temp_double *= invtanB;
        MOVW      AX, N:_invtanB+2   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+22
        MOVW      AX, N:_invtanB     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+24
        MOVW      BC, S:_temp_double+2  ;; 1 cycle
        MOVW      AX, S:_temp_double  ;; 1 cycle
          CFI FunCall ?F_MUL
        CALL      N:?F_MUL           ;; 3 cycles
        MOVW      S:_temp_double, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_temp_double+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  542     temp_double = atan(temp_double);
        MOVW      BC, S:_temp_double+2  ;; 1 cycle
        MOVW      AX, S:_temp_double  ;; 1 cycle
          CFI FunCall _atan
        CALL      _atan              ;; 3 cycles
        MOVW      S:_temp_double, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_temp_double+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  543     angle = (s16)DEGREE(temp_double*100.0f);
        MOVW      AX, #0x4265        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+26
        MOVW      AX, #0x2EE1        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+28
        MOVW      AX, #0x42C8        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+30
        MOVW      AX, #0x0           ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
        MOVW      BC, S:_temp_double+2  ;; 1 cycle
        MOVW      AX, S:_temp_double  ;; 1 cycle
          CFI FunCall ?F_MUL
        CALL      N:?F_MUL           ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+28
          CFI FunCall ?F_MUL
        CALL      N:?F_MUL           ;; 3 cycles
          CFI FunCall ?F_F2SL
        CALL      N:?F_F2SL          ;; 3 cycles
//  544     return angle;
        ADDW      SP, #0x18          ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock9
        ; ------------------------------------- Block: 100 cycles
        ; ------------------------------------- Total: 100 cycles
//  545 }

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock10 Using cfiCommon1
          CFI Function _cal_angle_calculate_react
        CODE
//  546 s16 cal_angle_calculate_react(s32 left_power, s32 right_power)
//  547 {
_cal_angle_calculate_react:
        ; * Stack frame (at entry) *
        ; Param size: 4
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+6
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+8
        ; Auto size: 4
//  548     s16 angle;
//  549     temp_double = (double)left_power/right_power;
        MOVW      AX, [SP+0x0A]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x08]      ;; 1 cycle
          CFI FunCall ?F_SL2F
        CALL      N:?F_SL2F          ;; 3 cycles
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+10
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
          CFI FunCall ?F_SL2F
        CALL      N:?F_SL2F          ;; 3 cycles
          CFI FunCall ?F_DIV
        CALL      N:?F_DIV           ;; 3 cycles
        MOVW      S:_temp_double, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_temp_double+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  550     //temp_double = 1 + ((double)2/(temp_double-1)); /* check it firt as per below statement*/
//  551     temp_double = (1+temp_double)/(1-temp_double);
        MOVW      AX, S:_temp_double+2  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+14
        MOVW      AX, S:_temp_double  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+16
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      BC, #0x3F80        ;; 1 cycle
          CFI FunCall ?F_SUB
        CALL      N:?F_SUB           ;; 3 cycles
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+18
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+20
        MOVW      AX, #0x3F80        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+22
        MOVW      AX, #0x0           ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+24
        MOVW      BC, S:_temp_double+2  ;; 1 cycle
        MOVW      AX, S:_temp_double  ;; 1 cycle
          CFI FunCall ?F_ADD
        CALL      N:?F_ADD           ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+20
          CFI FunCall ?F_DIV
        CALL      N:?F_DIV           ;; 3 cycles
        MOVW      S:_temp_double, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_temp_double+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  552     //temp_double *= TAN_B;
//  553     temp_double *= tanB;
        MOVW      AX, N:_tanB+2      ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+22
        MOVW      AX, N:_tanB        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+24
        MOVW      BC, S:_temp_double+2  ;; 1 cycle
        MOVW      AX, S:_temp_double  ;; 1 cycle
          CFI FunCall ?F_MUL
        CALL      N:?F_MUL           ;; 3 cycles
        MOVW      S:_temp_double, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_temp_double+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  554     temp_double = atan(temp_double);
        MOVW      BC, S:_temp_double+2  ;; 1 cycle
        MOVW      AX, S:_temp_double  ;; 1 cycle
          CFI FunCall _atan
        CALL      _atan              ;; 3 cycles
        MOVW      S:_temp_double, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_temp_double+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  555     angle = (s16)DEGREE(temp_double*100.0f);
        MOVW      AX, #0x4265        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+26
        MOVW      AX, #0x2EE1        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+28
        MOVW      AX, #0x42C8        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+30
        MOVW      AX, #0x0           ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
        MOVW      BC, S:_temp_double+2  ;; 1 cycle
        MOVW      AX, S:_temp_double  ;; 1 cycle
          CFI FunCall ?F_MUL
        CALL      N:?F_MUL           ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+28
          CFI FunCall ?F_MUL
        CALL      N:?F_MUL           ;; 3 cycles
          CFI FunCall ?F_F2SL
        CALL      N:?F_F2SL          ;; 3 cycles
//  556     return angle;
        ADDW      SP, #0x18          ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock10
        ; ------------------------------------- Block: 100 cycles
        ; ------------------------------------- Total: 100 cycles
//  557 }

        SECTION `.iar_vfe_header`:DATA:NOALLOC:NOROOT(1)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
        DC32 0

        END
// 
//   795 bytes in section .bss
//     1 byte  in section .sbss.noinit  (abs)
// 3'860 bytes in section .text
// 
// 3'860 bytes of CODE memory
//   795 bytes of DATA memory (+ 1 byte shared)
//
//Errors: none
//Warnings: none
