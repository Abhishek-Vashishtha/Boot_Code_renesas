///////////////////////////////////////////////////////////////////////////////
//
// IAR C/C++ Compiler V4.20.1.2260 for RL78               20/Dec/2020  22:38:41
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
//        BootCode\source_code\source_files\metrology.c
//    Command line       =
//        -f C:\Users\laptop\AppData\Local\Temp\EW9D0D.tmp ("D:\Dheeraj\New
//        folder\0. GDEV72 - BootCode\source_code\source_files\metrology.c"
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
//        D:\Dheeraj\New folder\0. GDEV72 - BootCode\Debug\List\metrology.s
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

        EXTERN _cal_done_f
        EXTERN _thd
        EXTERN _flag_mag
        EXTERN _flag_cal
        EXTERN _flag_calibration
        EXTERN _opr_data
        EXTERN _flag_thd1
        EXTERN _flag_sleep
        EXTERN _utility_id
        EXTERN _flag_system1
        EXTERN _serial_no
        EXTERN _meter_type
        EXTERN _dlms_firm_ver
        EXTERN _active_calendar
        EXTERN _calendar_change_f
        EXTERN _slot_no
        EXTERN _tariff_index
        EXTERN _tariff_no
        EXTERN _eeblk
        EXTERN _eepg
        EXTERN ?FCMP_GE
        EXTERN ?FCMP_LT
        EXTERN ?F_ADD
        EXTERN ?F_DIV
        EXTERN ?F_F2SL
        EXTERN ?F_F2UL
        EXTERN ?F_MUL
        EXTERN ?F_SL2F
        EXTERN ?F_SUB
        EXTERN ?F_UL2F
        EXTERN ?L_MUL_FAST_L03
        EXTERN ?L_NEG_L03
        EXTERN ?L_NOT_L03
        EXTERN ?MOVE_LONG_L06
        EXTERN ?SL_DIV_L03
        EXTERN ?UC_DIV_L01
        EXTERN ?UC_MOD_L01
        EXTERN ?UL_CMP_L03
        EXTERN _CTR
        EXTERN _PTR
        EXTERN _R_ADC_Create
        EXTERN _R_ADC_Set_OperationOff
        EXTERN _R_ADC_Set_OperationOn
        EXTERN _R_ADC_Set_PowerOff
        EXTERN _R_ADC_Start
        EXTERN _R_ADC_Stop
        EXTERN _R_DSADC_Create
        EXTERN _R_DSADC_Set_OperationOff
        EXTERN _R_DSADC_Set_OperationOn
        EXTERN _R_DSADC_Set_PowerOff
        EXTERN _R_DSADC_Start
        EXTERN _R_DSADC_Stop
        EXTERN _R_DTCD0_Start
        EXTERN _R_DTCD0_Stop
        EXTERN _R_DTCD1_Start
        EXTERN _R_DTCD1_Stop
        EXTERN _R_DTC_Create
        EXTERN _R_DTC_Set_PowerOff
        EXTERN _R_ELC_Create
        EXTERN _R_ELC_Set
        EXTERN _R_ELC_Stop
        EXTERN _R_TAU0_Channel1_SetValue
        EXTERN _R_TAU0_Channel1_Start
        EXTERN _R_TAU0_Channel1_Stop
        EXTERN _R_TAU0_Create
        EXTERN _R_WDT_Restart
        EXTERN __Add64
        EXTERN __CmpGes64
        EXTERN __CmpGeu64
        EXTERN __CmpLtu64
        EXTERN __CmpNe64
        EXTERN __Divs64
        EXTERN __Divu64
        EXTERN __F2LLU
        EXTERN __L2LLS
        EXTERN __L2LLU
        EXTERN __LLS2L
        EXTERN __LLU2F
        EXTERN __LLU2L
        EXTERN __Mul64
        EXTERN __Neg64
        EXTERN __Not64
        EXTERN __Sub64
        EXTERN ___iar_fmex
        EXTERN _acos
        EXTERN _asin
        EXTERN _atan
        EXTERN _battery_vol_cnt
        EXTERN _battery_voltage
        EXTERN _cal_BPh
        EXTERN _cal_RPh
        EXTERN _cal_YPh
        EXTERN _cal_angle_calculate_act
        EXTERN _calibration_delay_voltage_samples
        EXTERN _char_array_to_int
        EXTERN _char_array_to_long4
        EXTERN _check_active_calendar
        EXTERN _cos
        EXTERN _delay_us
        EXTERN _deter_season
        EXTERN _eprom_read
        EXTERN _eprom_write
        EXTERN _fill_oprzero
        EXTERN _flag_battery
        EXTERN _int_into_char_array
        EXTERN _long_into_char_array4
        EXTERN _mac_union_2int
        EXTERN _mac_union_4int
        EXTERN _read_battery_voltage
        EXTERN _sin
        EXTERN _sleep_action_early_detection
        EXTERN _sqrt
        EXTERN _t_zkvah1
        EXTERN _t_zkvah2
        EXTERN _t_zkwh1
        EXTERN _t_zkwh2
        EXTERN _temp_s64
        EXTERN _temp_us16
        EXTERN _temp_us64
        EXTERN _ten_power
        EXTERN _thd_accumulation
        EXTERN _thd_one_sec_loop
        EXTERN _tpr
        EXTERN _vector_addition_2byte
        EXTERN _zone_pf

        PUBLIC _ACT_THRESHOLD
        PUBLIC _APP_THRESHOLD
        PUBLIC _METERING_MODE
        PUBLIC _PULSE_WEIGHT_TABLE
        PUBLIC _REACT_THRESHOLD
        PUBLIC __A_MAC32SH
        PUBLIC __A_MAC32SL
        PUBLIC __A_MACRH
        PUBLIC __A_MACRL
        PUBLIC __A_MULBH
        PUBLIC __A_MULBL
        PUBLIC __A_MULC
        PUBLIC __A_MULR0
        PUBLIC __A_MULR1
        PUBLIC __A_MULR2
        PUBLIC __A_MULR3
        PUBLIC __A_P4
        PUBLIC __A_P5
        PUBLIC __Constant_0_0
        PUBLIC __Constant_12_0
        PUBLIC __Constant_186a0_0
        PUBLIC __Constant_1_0
        PUBLIC __Constant_3e8_0
        PUBLIC __Constant_64_0
        PUBLIC __Constant_a_0
        PUBLIC __ZZ19zerocross_detectionE15b_zc_check_cntr
        PUBLIC __ZZ19zerocross_detectionE15r_zc_check_cntr
        PUBLIC __ZZ19zerocross_detectionE15y_zc_check_cntr
        PUBLIC __ZZ19zerocross_detectionE19zc_power_fail_timer
        PUBLIC __ZZ21delay_voltage_samplesE10buffer_ptr
        PUBLIC __ZZ21delay_voltage_samplesE11buffer_volb
        PUBLIC __ZZ21delay_voltage_samplesE11buffer_volr
        PUBLIC __ZZ21delay_voltage_samplesE11buffer_voly
        PUBLIC __ZZ21delay_voltage_samplesE12buffer_ptr90
        PUBLIC __ZZ21delay_voltage_samplesE13buffer_volb90
        PUBLIC __ZZ21delay_voltage_samplesE13buffer_volr90
        PUBLIC __ZZ21delay_voltage_samplesE13buffer_voly90
        PUBLIC _active_calculation_fwd
        PUBLIC _active_calculation_net
        PUBLIC _all_phase
        PUBLIC _alt_energy_save_cntr
        PUBLIC _alternate_energy_counter1
        PUBLIC _alternate_energy_counter2
        PUBLIC _angle
        PUBLIC _apparent_calculation_fwd
        PUBLIC _apparent_calculation_net
        PUBLIC _b_phase
        PUBLIC _cal_angle
        PUBLIC _cal_coeff
        PUBLIC _cal_neu_current
        PUBLIC _cal_pf
        PUBLIC _cal_pf_signed
        PUBLIC _calculate_irms
        PUBLIC _calculate_vrms
        PUBLIC _change_active_delay_buffer_par
        PUBLIC _config_parameter_init
        PUBLIC _curr
        PUBLIC _decimal
        PUBLIC _delay_voltage_samples
        PUBLIC _demand
        PUBLIC _duplicate_total_apparent_energy
        PUBLIC _e2416_byte
        PUBLIC _e2416_q2_byte
        PUBLIC _energy
        PUBLIC _energy_clear
        PUBLIC _energy_rollover_count
        PUBLIC _flag_metro1
        PUBLIC _flag_metro2
        PUBLIC _flag_metro3
        PUBLIC _flag_metrology
        PUBLIC _flag_metrology1
        PUBLIC _flag_quadrant
        PUBLIC _flag_zc
        PUBLIC _freq
        PUBLIC _freq_variation_timer_update
        PUBLIC _get_hr_energy
        PUBLIC _get_quadrant
        PUBLIC _last_energy_mag
        PUBLIC _last_energy_mag_Bph
        PUBLIC _last_energy_mag_Rph
        PUBLIC _last_energy_mag_Yph
        PUBLIC _metrology_1sec_loop
        PUBLIC _metrology_all_phase_calculation
        PUBLIC _metrology_function
        PUBLIC _metrology_process
        PUBLIC _metrology_ram_init
        PUBLIC _metrology_save_energy
        PUBLIC _metrology_save_pulse
        PUBLIC _metrology_setup
        PUBLIC _metrology_start
        PUBLIC _metrology_stop
        PUBLIC _n_phase
        PUBLIC _pf
        PUBLIC _ph_ph
        PUBLIC _power
        PUBLIC _power_filter
        PUBLIC _pwrup_sec_cnt
        PUBLIC _quadrant
        PUBLIC _r_phase
        PUBLIC _r_samp_issue_f
        PUBLIC _react_sample_delay
        PUBLIC _react_time_delay
        PUBLIC _reactive_calculation_fwd
        PUBLIC _reactive_calculation_net
        PUBLIC _rollover
        PUBLIC _rollover_energy
        PUBLIC _samples_per_sec
        PUBLIC _samples_per_sec_temp
        PUBLIC _save_all_energy_and_pulse
        PUBLIC _save_energy_alternate
        PUBLIC _temp_apparent_energy
        PUBLIC _temp_reactive_lead_energy
        PUBLIC _time_delay
        PUBLIC _time_deviation
        PUBLIC _timer_value
        PUBLIC _tod_init
        PUBLIC _vol
        PUBLIC _y_phase
        PUBLIC _zerocross_detection
        
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
          CFI EndCommon cfiCommon2
        
// D:\Dheeraj\New folder\0. GDEV72 - BootCode\source_code\source_files\metrology.c
//    1 /***********************************************************************************************************************
//    2 * File Name       : metrology.c
//    3 * Current Version : rev_  
//    4 * Tool-Chain      : IAR Systems
//    5 * Description     : This file include routines to implement metrology functionality into energy meter
//    6 * Creation Date   : 04-01-2020
//    7 * Company         : Genus Power Infrastructures Limited, Jaipur
//    8 * Author          : dheeraj.singhal
//    9 * Version History : 
//   10 ***********************************************************************************************************************/
//   11 /************************************ Includes **************************************/
//   12 #include "metrology.h"

        ASEGN `.sbss.noinit`:DATA:NOROOT,0fff04H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_P4
// __no_init union <unnamed>#7 volatile __saddr _A_P4
__A_P4:
        DS 1

        ASEGN `.sbss.noinit`:DATA:NOROOT,0fff05H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_P5
// __no_init union <unnamed>#8 volatile __saddr _A_P5
__A_P5:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0fff3cH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_MULBL
// __no_init union <unnamed>#58 volatile __sfr __no_bit_access _A_MULBL
__A_MULBL:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0fff3eH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_MULBH
// __no_init union <unnamed>#59 volatile __sfr __no_bit_access _A_MULBH
__A_MULBH:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0ffff0H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_MACRL
// __no_init union <unnamed>#224 volatile __sfr __no_bit_access _A_MACRL
__A_MACRL:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0ffff2H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_MACRH
// __no_init union <unnamed>#225 volatile __sfr __no_bit_access _A_MACRH
__A_MACRH:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f028cH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_MAC32SL
// __no_init union <unnamed>#501 volatile __no_bit_access _A_MAC32SL
__A_MAC32SL:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f028eH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_MAC32SH
// __no_init union <unnamed>#502 volatile __no_bit_access _A_MAC32SH
__A_MAC32SH:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0290H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_MULR0
// __no_init union <unnamed>#503 volatile __no_bit_access _A_MULR0
__A_MULR0:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0292H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_MULR1
// __no_init union <unnamed>#504 volatile __no_bit_access _A_MULR1
__A_MULR1:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0294H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_MULR2
// __no_init union <unnamed>#505 volatile __no_bit_access _A_MULR2
__A_MULR2:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0296H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_MULR3
// __no_init union <unnamed>#506 volatile __no_bit_access _A_MULR3
__A_MULR3:
        DS 2

        ASEGN `.bss.noinit`:DATA:NOROOT,0f029aH
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_MULC
// __no_init union <unnamed>#507 volatile _A_MULC
__A_MULC:
        DS 1
//   13 #include "r_cg_dsadc.h"
//   14 #include "r_cg_elc.h"
//   15 #include "r_cg_adc.h"
//   16 #include "r_cg_dtc.h"
//   17 #include "r_cg_tau.h"
//   18 #include "r_cg_mac32bit.h"
//   19 /************************************ Local Variables *****************************************/
//   20 
//   21 
//   22 #if METER_CONST == 1200

        SECTION `.data`:DATA:REORDER:NOROOT(1)
//   23 const us16 PULSE_WEIGHT_TABLE[6] = {0,833,1666,2500,3333,4166};
_PULSE_WEIGHT_TABLE:
        DATA16
        DW 0, 833, 1'666, 2'500, 3'333, 4'166

        SECTION `.data`:DATA:REORDER:NOROOT(1)
        SECTION_GROUP __Constant_0_0
__Constant_0_0:
        DATA32
        DD 0, 0

        SECTION `.data`:DATA:REORDER:NOROOT(1)
        SECTION_GROUP __Constant_1_0
__Constant_1_0:
        DATA32
        DD 1, 0

        SECTION `.data`:DATA:REORDER:NOROOT(1)
        SECTION_GROUP __Constant_a_0
__Constant_a_0:
        DATA32
        DD 10, 0

        SECTION `.data`:DATA:REORDER:NOROOT(1)
        SECTION_GROUP __Constant_64_0
__Constant_64_0:
        DATA32
        DD 100, 0

        SECTION `.data`:DATA:REORDER:NOROOT(1)
        SECTION_GROUP __Constant_186a0_0
__Constant_186a0_0:
        DATA32
        DD 100'000, 0

        SECTION `.data`:DATA:REORDER:NOROOT(1)
        SECTION_GROUP __Constant_12_0
__Constant_12_0:
        DATA32
        DD 18, 0

        SECTION `.data`:DATA:REORDER:NOROOT(1)
        SECTION_GROUP __Constant_3e8_0
__Constant_3e8_0:
        DATA32
        DD 1'000, 0
//   24 #elif METER_CONST == 600
//   25 const us16 PULSE_WEIGHT_TABLE[6] = {0,600,1200,1800,2400,3000};
//   26 #else
//   27 const us16 PULSE_WEIGHT_TABLE[6] = {0,1000,2000,3000,4000,5000};
//   28 #endif
//   29 
//   30 /************************************ Extern Variables *****************************************/

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   31 flag_union flag_metrology,flag_metrology1, flag_metro1, flag_metro2, flag_metro3;
_flag_metrology:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_flag_metrology1:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_flag_metro1:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_flag_metro2:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_flag_metro3:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   32 flag_union flag_quadrant;
_flag_quadrant:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   33 flag_union flag_zc;
_flag_zc:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   34 Decimal decimal;
_decimal:
        DS 9

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   35 Voltage vol;
_vol:
        DS 24

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   36 Current curr;
_curr:
        DS 84

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   37 PF pf;
_pf:
        DS 24

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   38 QUADRANT quadrant;
_quadrant:
        DS 4

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   39 Frequency freq;
_freq:
        DS 10

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   40 Angle angle;
_angle:
        DS 6

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   41 PH_PH ph_ph;
_ph_ph:
        DS 24

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   42 Power power;
_power:
        DS 96

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   43 Energy energy;
_energy:
        DS 124

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   44 Demand demand;
_demand:
        DS 216

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   45 CALIBRATION cal_coeff;
_cal_coeff:
        DS 42
//   46 

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   47 PHASES r_phase,y_phase,b_phase;
_r_phase:
        DS 208

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_y_phase:
        DS 208

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_b_phase:
        DS 208

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   48 PHASES_N n_phase;
_n_phase:
        DS 34

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   49 PHASES_ALL all_phase;
_all_phase:
        DS 136

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   50 us16 samples_per_sec,samples_per_sec_temp;
_samples_per_sec:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_samples_per_sec_temp:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   51 us8 METERING_MODE;
_METERING_MODE:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   52 us8 pwrup_sec_cnt;
_pwrup_sec_cnt:
        DS 1
//   53 
//   54 /* frequency variation */

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   55 us8 react_sample_delay;
_react_sample_delay:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   56 float react_time_delay;
_react_time_delay:
        DS 4

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   57 us8 r_samp_issue_f;
_r_samp_issue_f:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   58 us16 time_delay,time_deviation,timer_value;
_time_delay:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_time_deviation:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_timer_value:
        DS 2
//   59 
//   60 /* Energy Saving */

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   61 us64 temp_apparent_energy,temp_reactive_lead_energy;
_temp_apparent_energy:
        DS 8

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_temp_reactive_lead_energy:
        DS 8

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   62 us8 alt_energy_save_cntr;
_alt_energy_save_cntr:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   63 us8 energy_rollover_count;
_energy_rollover_count:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   64 us8 alternate_energy_counter1,alternate_energy_counter2;
_alternate_energy_counter1:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_alternate_energy_counter2:
        DS 1
//   65 /* thresholds */

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   66 us64 ACT_THRESHOLD,REACT_THRESHOLD,APP_THRESHOLD;
_ACT_THRESHOLD:
        DS 8

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_REACT_THRESHOLD:
        DS 8

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_APP_THRESHOLD:
        DS 8
//   67 

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   68 us16 e2416_byte;
_e2416_byte:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   69 us8 e2416_q2_byte;
_e2416_q2_byte:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   70 us32 duplicate_total_apparent_energy;
_duplicate_total_apparent_energy:
        DS 4

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   71 us32 last_energy_mag,last_energy_mag_Rph,last_energy_mag_Yph,last_energy_mag_Bph;
_last_energy_mag:
        DS 4

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_last_energy_mag_Rph:
        DS 4

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_last_energy_mag_Yph:
        DS 4

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_last_energy_mag_Bph:
        DS 4
//   72 
//   73 /************************************ Local Functions *******************************/
//   74 void active_calculation_net(us8 phase,us8 quad);
//   75 void active_calculation_fwd(us8 phase,us8 quad);
//   76 void reactive_calculation_net(us8 phase,us8 quad);
//   77 void reactive_calculation_fwd(us8 phase,us8 quad);
//   78 void apparent_calculation_net(us8 phase,us8 quad);
//   79 void apparent_calculation_fwd(us8 phase,us8 quad);
//   80 void delay_voltage_samples();
//   81 s16 cal_angle(us32 active_power,us32 reactive_power,us32 apparent_power,us8 active_f, us8 reactive_f, us8 trig);
//   82 us16 cal_pf(us32 active_power,us32 apparent_power);
//   83 s16 cal_pf_signed(s32 active_power,s32 apparent_power);
//   84 void freq_variation_timer_update();
//   85 void change_active_delay_buffer_par(us16 freq);
//   86 us32 calculate_irms(us64 acc_cnt, us16 cal_coeff, us16 no_of_samples);
//   87 us16 calculate_vrms(us64 acc_cnt, us16 cal_coeff, us16 no_of_samples);
//   88 void zerocross_detection();
//   89 us8 get_quadrant(us8 act_flag, us8 react_flag);
//   90 /************************************ Extern Functions ******************************/
//   91 void metrology_setup();
//   92 void metrology_start();
//   93 void metrology_stop();
//   94 void metrology_function();
//   95 void metrology_process();
//   96 void metrology_save_energy();
//   97 void save_all_energy_and_pulse();
//   98 void metrology_save_pulse();
//   99 void metrology_1sec_loop();
//  100 us64 get_hr_energy(us32 energy,us8 pulse);
//  101 us32 cal_neu_current();
//  102 void metrology_ram_init();
//  103 void metrology_all_phase_calculation();
//  104 void save_energy_alternate();
//  105 void tod_init();
//  106 void config_parameter_init();
//  107 void energy_clear();
//  108 void rollover();
//  109 us32 rollover_energy(us32 energy);
//  110 us32 power_filter(us32 power1, us8 counter);
//  111 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock0 Using cfiCommon0
          CFI Function _metrology_setup
          CFI FunCall _read_battery_voltage
        CODE
//  112 void metrology_setup()
//  113 {
_metrology_setup:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  114     /* Checking battery voltage */
//  115     
//  116     battery_vol_cnt = read_battery_voltage();
        CALL      _read_battery_voltage  ;; 3 cycles
        MOVW      N:_battery_vol_cnt, AX  ;; 1 cycle
//  117     if(battery_installed == TEKCELL)
        MOVW      HL, #LWRD(_flag_battery)  ;; 1 cycle
        MOV1      CY, [HL].2         ;; 1 cycle
        BC        ??power_filter_0   ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
//  118     {
//  119         if(battery_vol_cnt <= 15)
        MOVW      AX, N:_battery_vol_cnt  ;; 1 cycle
        CMPW      AX, #0x10          ;; 1 cycle
        BNC       ??power_filter_1   ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  120         {
//  121             battery_voltage = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_battery_voltage, AX  ;; 1 cycle
        BR        S:??power_filter_2  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  122         }
//  123         else 
//  124         {
//  125             battery_voltage = (us16)((float)0.464396f*battery_vol_cnt-6.62539f);
??power_filter_1:
        MOVW      AX, #0xC0D4        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+6
        MOVW      AX, #0x332         ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+8
        MOVW      AX, #0x3EED        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        MOVW      AX, #0xC550        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      AX, N:_battery_vol_cnt  ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
          CFI FunCall ?F_UL2F
        CALL      N:?F_UL2F          ;; 3 cycles
          CFI FunCall ?F_MUL
        CALL      N:?F_MUL           ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
          CFI FunCall ?F_ADD
        CALL      N:?F_ADD           ;; 3 cycles
          CFI FunCall ?F_F2SL
        CALL      N:?F_F2SL          ;; 3 cycles
        MOVW      N:_battery_voltage, AX  ;; 1 cycle
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+4
        BR        S:??power_filter_2  ;; 3 cycles
        ; ------------------------------------- Block: 28 cycles
//  126         }
//  127     }
//  128     else
//  129     {
//  130         if(battery_vol_cnt <= 37)
??power_filter_0:
        MOVW      AX, N:_battery_vol_cnt  ;; 1 cycle
        CMPW      AX, #0x26          ;; 1 cycle
        BNC       ??power_filter_3   ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  131         {
//  132             battery_voltage = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_battery_voltage, AX  ;; 1 cycle
        BR        S:??power_filter_2  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  133         }
//  134         else 
//  135         {
//  136             battery_voltage = (us16)((float)0.460829f*battery_vol_cnt-16.8664f);
??power_filter_3:
        MOVW      AX, #0xC186        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+6
        MOVW      AX, #0xEE63        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+8
        MOVW      AX, #0x3EEB        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        MOVW      AX, #0xF1C7        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      AX, N:_battery_vol_cnt  ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
          CFI FunCall ?F_UL2F
        CALL      N:?F_UL2F          ;; 3 cycles
          CFI FunCall ?F_MUL
        CALL      N:?F_MUL           ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
          CFI FunCall ?F_ADD
        CALL      N:?F_ADD           ;; 3 cycles
          CFI FunCall ?F_F2SL
        CALL      N:?F_F2SL          ;; 3 cycles
        MOVW      N:_battery_voltage, AX  ;; 1 cycle
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+4
          CFI FunCall _R_TAU0_Create
        ; ------------------------------------- Block: 25 cycles
//  137         }
//  138     }
//  139     
//  140     //lcd_write_msg(315,2);
//  141     /* Creating the relevent peripheral */
//  142     R_TAU0_Create();
??power_filter_2:
        CALL      _R_TAU0_Create     ;; 3 cycles
//  143     R_DSADC_Create();
          CFI FunCall _R_DSADC_Create
        CALL      _R_DSADC_Create    ;; 3 cycles
//  144     R_ADC_Create();
          CFI FunCall _R_ADC_Create
        CALL      _R_ADC_Create      ;; 3 cycles
//  145     R_DTC_Create();
          CFI FunCall _R_DTC_Create
        CALL      _R_DTC_Create      ;; 3 cycles
//  146     R_ELC_Create();
          CFI FunCall _R_ELC_Create
        CALL      _R_ELC_Create      ;; 3 cycles
//  147 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock0
        ; ------------------------------------- Block: 21 cycles
        ; ------------------------------------- Total: 106 cycles
//  148 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock1 Using cfiCommon0
          CFI Function _metrology_start
          CFI FunCall _R_WDT_Restart
        CODE
//  149 void metrology_start()
//  150 {
_metrology_start:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  151     wdt_restart();
        CALL      _R_WDT_Restart     ;; 3 cycles
//  152     
//  153     R_ELC_Set(ELC_TRIGGER_SRC_INTDSAD, _01_ELC_EVENT_LINK_AD);  /* triggering ADC conversion Active samples */
        MOV       X, #0x1            ;; 1 cycle
        MOV       A, #0x13           ;; 1 cycle
          CFI FunCall _R_ELC_Set
        CALL      _R_ELC_Set         ;; 3 cycles
//  154     R_ELC_Set(ELC_TRIGGER_SRC_INTTM01, _01_ELC_EVENT_LINK_AD);  /* triggering ADC conversion Reactive samples */
        MOV       X, #0x1            ;; 1 cycle
        MOV       A, #0xE            ;; 1 cycle
          CFI FunCall _R_ELC_Set
        CALL      _R_ELC_Set         ;; 3 cycles
//  155     
//  156     R_DTCD0_Start();                                            /* to transfer start bit in timer start register */
          CFI FunCall _R_DTCD0_Start
        CALL      _R_DTCD0_Start     ;; 3 cycles
//  157     R_DTCD1_Start();                                            /* transfering ADC result to ram variable */ 
          CFI FunCall _R_DTCD1_Start
        CALL      _R_DTCD1_Start     ;; 3 cycles
//  158     R_TAU0_Channel1_Start();
          CFI FunCall _R_TAU0_Channel1_Start
        CALL      _R_TAU0_Channel1_Start  ;; 3 cycles
//  159     R_ADC_Set_OperationOn();
          CFI FunCall _R_ADC_Set_OperationOn
        CALL      _R_ADC_Set_OperationOn  ;; 3 cycles
//  160     delay_us(5);
        MOVW      AX, #0x5           ;; 1 cycle
          CFI FunCall _delay_us
        CALL      _delay_us          ;; 3 cycles
//  161     R_ADC_Start();
          CFI FunCall _R_ADC_Start
        CALL      _R_ADC_Start       ;; 3 cycles
//  162     R_DSADC_Set_OperationOn();
          CFI FunCall _R_DSADC_Set_OperationOn
        CALL      _R_DSADC_Set_OperationOn  ;; 3 cycles
//  163     R_DSADC_Start();
          CFI FunCall _R_DSADC_Start
        CALL      _R_DSADC_Start     ;; 3 cycles
//  164     
//  165     react_sample_delay = 19;
        MOV       N:_react_sample_delay, #0x13  ;; 1 cycle
//  166 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock1
        ; ------------------------------------- Block: 45 cycles
        ; ------------------------------------- Total: 45 cycles
//  167 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock2 Using cfiCommon0
          CFI Function _metrology_stop
          CFI FunCall _R_DSADC_Stop
        CODE
//  168 void metrology_stop()
//  169 {
_metrology_stop:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  170     R_DSADC_Stop();
        CALL      _R_DSADC_Stop      ;; 3 cycles
//  171     R_DSADC_Set_OperationOff();
          CFI FunCall _R_DSADC_Set_OperationOff
        CALL      _R_DSADC_Set_OperationOff  ;; 3 cycles
//  172     R_DSADC_Set_PowerOff();
          CFI FunCall _R_DSADC_Set_PowerOff
        CALL      _R_DSADC_Set_PowerOff  ;; 3 cycles
//  173     R_ADC_Stop();
          CFI FunCall _R_ADC_Stop
        CALL      _R_ADC_Stop        ;; 3 cycles
//  174     R_ADC_Set_OperationOff();
          CFI FunCall _R_ADC_Set_OperationOff
        CALL      _R_ADC_Set_OperationOff  ;; 3 cycles
//  175     R_ADC_Set_PowerOff();
          CFI FunCall _R_ADC_Set_PowerOff
        CALL      _R_ADC_Set_PowerOff  ;; 3 cycles
//  176     delay_us(5);
        MOVW      AX, #0x5           ;; 1 cycle
          CFI FunCall _delay_us
        CALL      _delay_us          ;; 3 cycles
//  177     R_TAU0_Channel1_Stop();
          CFI FunCall _R_TAU0_Channel1_Stop
        CALL      _R_TAU0_Channel1_Stop  ;; 3 cycles
//  178     R_DTCD0_Stop();        
          CFI FunCall _R_DTCD0_Stop
        CALL      _R_DTCD0_Stop      ;; 3 cycles
//  179     R_DTCD1_Stop(); 
          CFI FunCall _R_DTCD1_Stop
        CALL      _R_DTCD1_Stop      ;; 3 cycles
//  180     R_DTC_Set_PowerOff();
          CFI FunCall _R_DTC_Set_PowerOff
        CALL      _R_DTC_Set_PowerOff  ;; 3 cycles
//  181     R_ELC_Stop(ELC_TRIGGER_SRC_INTDSAD);
        MOVW      AX, #0x13          ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
          CFI FunCall _R_ELC_Stop
        CALL      _R_ELC_Stop        ;; 3 cycles
//  182     R_ELC_Stop(ELC_TRIGGER_SRC_INTTM01);
        MOVW      AX, #0xE           ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
          CFI FunCall _R_ELC_Stop
        CALL      _R_ELC_Stop        ;; 3 cycles
//  183 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock2
        ; ------------------------------------- Block: 50 cycles
        ; ------------------------------------- Total: 50 cycles

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock3 Using cfiCommon0
          CFI Function _metrology_function
        CODE
//  184 void metrology_function()
//  185 {
_metrology_function:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  186     static us8 dispense_ecal = 0u,dispense_rcal = 0u,two_ms_loop_cntr = 0;
//  187     static us16 bkup_macrh = 0;
//  188     static us16 bkup_macrl = 0;
//  189 #if DEBUG_MODE == 1
//  190         static us8 index = 0;
//  191 #endif
//  192     /* Push MACRx Registers */
//  193     bkup_macrh = MACRH;
        MOVW      AX, 0xFFFF2        ;; 1 cycle
        MOVW      N:`metrology_function::bkup_macrh`, AX  ;; 1 cycle
//  194     bkup_macrl = MACRL;
        MOVW      AX, 0xFFFF0        ;; 1 cycle
        MOVW      N:`metrology_function::bkup_macrl`, AX  ;; 1 cycle
//  195     
//  196     /* Meterology */
//  197     samples_per_sec_temp++;
        INCW      N:_samples_per_sec_temp  ;; 2 cycles
//  198     r_phase.no_of_samples_temp++;
        INCW      N:_r_phase+2       ;; 2 cycles
//  199     y_phase.no_of_samples_temp++;
        INCW      N:_y_phase+2       ;; 2 cycles
//  200     b_phase.no_of_samples_temp++;
        INCW      N:_b_phase+2       ;; 2 cycles
//  201     n_phase.no_of_samples_temp++;
        INCW      N:_n_phase+2       ;; 2 cycles
//  202     if(cal_done_f != 1)
        CMP       N:_cal_done_f, #0x1  ;; 1 cycle
        BZ        ??power_filter_4   ;; 4 cycles
        ; ------------------------------------- Block: 19 cycles
//  203     {
//  204         cal_RPh.no_of_samples_temp++;
        INCW      N:_cal_RPh+2       ;; 2 cycles
//  205         cal_YPh.no_of_samples_temp++;
        INCW      N:_cal_YPh+2       ;; 2 cycles
//  206         cal_BPh.no_of_samples_temp++;
        INCW      N:_cal_BPh+2       ;; 2 cycles
          CFI FunCall _delay_voltage_samples
        ; ------------------------------------- Block: 6 cycles
//  207     }
//  208     delay_voltage_samples();
??power_filter_4:
        CALL      _delay_voltage_samples  ;; 3 cycles
//  209     
//  210     /* calibration offset subtraction */
//  211     r_phase.curr.sample_raw -= cal_coeff.Rph.curr_offset;
        MOVW      AX, N:_r_phase+42  ;; 1 cycle
        SUBW      AX, N:_cal_coeff+4  ;; 1 cycle
        MOVW      N:_r_phase+42, AX  ;; 1 cycle
//  212     y_phase.curr.sample_raw -= cal_coeff.Yph.curr_offset;
        MOVW      AX, N:_y_phase+42  ;; 1 cycle
        SUBW      AX, N:_cal_coeff+16  ;; 1 cycle
        MOVW      N:_y_phase+42, AX  ;; 1 cycle
//  213     b_phase.curr.sample_raw -= cal_coeff.Bph.curr_offset;
        MOVW      AX, N:_b_phase+42  ;; 1 cycle
        SUBW      AX, N:_cal_coeff+28  ;; 1 cycle
        MOVW      N:_b_phase+42, AX  ;; 1 cycle
//  214     n_phase.curr.sample_raw -= cal_coeff.Nph.curr_offset;
        MOVW      AX, N:_n_phase+4   ;; 1 cycle
        SUBW      AX, N:_cal_coeff+38  ;; 1 cycle
        MOVW      N:_n_phase+4, AX   ;; 1 cycle
//  215     r_phase.vol.sample_raw -= cal_coeff.Rph.vol_offset;
        MOVW      AX, N:_r_phase+6   ;; 1 cycle
        SUBW      AX, N:_cal_coeff+2  ;; 1 cycle
        MOVW      N:_r_phase+6, AX   ;; 1 cycle
//  216     y_phase.vol.sample_raw -= cal_coeff.Yph.vol_offset;
        MOVW      AX, N:_y_phase+6   ;; 1 cycle
        SUBW      AX, N:_cal_coeff+14  ;; 1 cycle
        MOVW      N:_y_phase+6, AX   ;; 1 cycle
//  217     b_phase.vol.sample_raw -= cal_coeff.Bph.vol_offset;
        MOVW      AX, N:_b_phase+6   ;; 1 cycle
        SUBW      AX, N:_cal_coeff+26  ;; 1 cycle
        MOVW      N:_b_phase+6, AX   ;; 1 cycle
//  218     r_phase.vol.sample_raw90 -= cal_coeff.Rph.vol_offset;
        MOVW      AX, N:_r_phase+10  ;; 1 cycle
        SUBW      AX, N:_cal_coeff+2  ;; 1 cycle
        MOVW      N:_r_phase+10, AX  ;; 1 cycle
//  219     y_phase.vol.sample_raw90 -= cal_coeff.Yph.vol_offset;
        MOVW      AX, N:_y_phase+10  ;; 1 cycle
        SUBW      AX, N:_cal_coeff+14  ;; 1 cycle
        MOVW      N:_y_phase+10, AX  ;; 1 cycle
//  220     b_phase.vol.sample_raw90 -= cal_coeff.Bph.vol_offset;
        MOVW      AX, N:_b_phase+10  ;; 1 cycle
        SUBW      AX, N:_cal_coeff+26  ;; 1 cycle
        MOVW      N:_b_phase+10, AX  ;; 1 cycle
//  221     
//  222     /* dc offset calculation accumulation using raw sample*/
//  223     r_phase.curr.offset_acc_cnt_temp += r_phase.curr.sample_raw;
        MOVW      AX, N:_r_phase+42  ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      HL, N:_r_phase+54  ;; 1 cycle
        MOVW      DE, N:_r_phase+52  ;; 1 cycle
        ADDW      AX, DE             ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_r_phase+52, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_r_phase+54, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  224     y_phase.curr.offset_acc_cnt_temp += y_phase.curr.sample_raw;
        MOVW      AX, N:_y_phase+42  ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      HL, N:_y_phase+54  ;; 1 cycle
        MOVW      DE, N:_y_phase+52  ;; 1 cycle
        ADDW      AX, DE             ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_y_phase+52, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_y_phase+54, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  225     b_phase.curr.offset_acc_cnt_temp += b_phase.curr.sample_raw;
        MOVW      AX, N:_b_phase+42  ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      HL, N:_b_phase+54  ;; 1 cycle
        MOVW      DE, N:_b_phase+52  ;; 1 cycle
        ADDW      AX, DE             ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_b_phase+52, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_b_phase+54, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  226     n_phase.curr.offset_acc_cnt_temp += n_phase.curr.sample_raw;
        MOVW      AX, N:_n_phase+4   ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      HL, N:_n_phase+16  ;; 1 cycle
        MOVW      DE, N:_n_phase+14  ;; 1 cycle
        ADDW      AX, DE             ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_n_phase+14, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_n_phase+16, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  227     r_phase.vol.offset_acc_cnt_temp += r_phase.vol.sample_raw;
        MOVW      AX, N:_r_phase+6   ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      HL, N:_r_phase+24  ;; 1 cycle
        MOVW      DE, N:_r_phase+22  ;; 1 cycle
        ADDW      AX, DE             ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_r_phase+22, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_r_phase+24, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  228     y_phase.vol.offset_acc_cnt_temp += y_phase.vol.sample_raw;
        MOVW      AX, N:_y_phase+6   ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      HL, N:_y_phase+24  ;; 1 cycle
        MOVW      DE, N:_y_phase+22  ;; 1 cycle
        ADDW      AX, DE             ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_y_phase+22, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_y_phase+24, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  229     b_phase.vol.offset_acc_cnt_temp += b_phase.vol.sample_raw;
        MOVW      AX, N:_b_phase+6   ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      HL, N:_b_phase+24  ;; 1 cycle
        MOVW      DE, N:_b_phase+22  ;; 1 cycle
        ADDW      AX, DE             ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_b_phase+22, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_b_phase+24, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  230     
//  231     /* remove dc offset */
//  232     r_phase.curr.sample = r_phase.curr.sample_raw - r_phase.curr.dc_offset;
        MOVW      AX, N:_r_phase+42  ;; 1 cycle
        SUBW      AX, N:_r_phase+46  ;; 1 cycle
        MOVW      N:_r_phase+44, AX  ;; 1 cycle
//  233     y_phase.curr.sample = y_phase.curr.sample_raw - y_phase.curr.dc_offset;
        MOVW      AX, N:_y_phase+42  ;; 1 cycle
        SUBW      AX, N:_y_phase+46  ;; 1 cycle
        MOVW      N:_y_phase+44, AX  ;; 1 cycle
//  234     b_phase.curr.sample = b_phase.curr.sample_raw - b_phase.curr.dc_offset;
        MOVW      AX, N:_b_phase+42  ;; 1 cycle
        SUBW      AX, N:_b_phase+46  ;; 1 cycle
        MOVW      N:_b_phase+44, AX  ;; 1 cycle
//  235     n_phase.curr.sample = n_phase.curr.sample_raw - n_phase.curr.dc_offset;
        MOVW      AX, N:_n_phase+4   ;; 1 cycle
        SUBW      AX, N:_n_phase+8   ;; 1 cycle
        MOVW      N:_n_phase+6, AX   ;; 1 cycle
//  236     r_phase.vol.sample = r_phase.vol.sample_raw - r_phase.vol.dc_offset;
        MOVW      AX, N:_r_phase+6   ;; 1 cycle
        SUBW      AX, N:_r_phase+14  ;; 1 cycle
        MOVW      N:_r_phase+8, AX   ;; 1 cycle
//  237     y_phase.vol.sample = y_phase.vol.sample_raw - y_phase.vol.dc_offset;
        MOVW      AX, N:_y_phase+6   ;; 1 cycle
        SUBW      AX, N:_y_phase+14  ;; 1 cycle
        MOVW      N:_y_phase+8, AX   ;; 1 cycle
//  238     b_phase.vol.sample = b_phase.vol.sample_raw - b_phase.vol.dc_offset;
        MOVW      AX, N:_b_phase+6   ;; 1 cycle
        SUBW      AX, N:_b_phase+14  ;; 1 cycle
        MOVW      N:_b_phase+8, AX   ;; 1 cycle
//  239     r_phase.vol.sample90 = r_phase.vol.sample_raw90 - r_phase.vol.dc_offset;
        MOVW      AX, N:_r_phase+10  ;; 1 cycle
        SUBW      AX, N:_r_phase+14  ;; 1 cycle
        MOVW      N:_r_phase+12, AX  ;; 1 cycle
//  240     y_phase.vol.sample90 = y_phase.vol.sample_raw90 - y_phase.vol.dc_offset;
        MOVW      AX, N:_y_phase+10  ;; 1 cycle
        SUBW      AX, N:_y_phase+14  ;; 1 cycle
        MOVW      N:_y_phase+12, AX  ;; 1 cycle
//  241     b_phase.vol.sample90 = b_phase.vol.sample_raw90 - b_phase.vol.dc_offset;
        MOVW      AX, N:_b_phase+10  ;; 1 cycle
        SUBW      AX, N:_b_phase+14  ;; 1 cycle
        MOVW      N:_b_phase+12, AX  ;; 1 cycle
//  242     
//  243     zerocross_detection();
          CFI FunCall _zerocross_detection
        CALL      _zerocross_detection  ;; 3 cycles
//  244     
//  245     /* Calibration delayed samples*/
//  246     if(cal_done_f != 1)
        CMP       N:_cal_done_f, #0x1  ;; 1 cycle
        SKZ                          ;; 1 cycle
          CFI FunCall _calibration_delay_voltage_samples
        ; ------------------------------------- Block: 201 cycles
//  247     {
//  248         calibration_delay_voltage_samples();
        CALL      _calibration_delay_voltage_samples  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
//  249     }
//  250     
//  251 #if DEBUG_MODE == 1
//  252     if(debug_data_zc_found == 1)
//  253     {
//  254         if(index < 157)
//  255         {
//  256            if(debug_data_type == '1')
//  257             {
//  258                 samples_vol[index] = r_phase.vol.sample;
//  259                 samples_vol90[index] = r_phase.vol.sample90;
//  260                 samples_curr[index] = r_phase.curr.sample;
//  261             }
//  262             else if(debug_data_type == '2')
//  263             {
//  264                 samples_vol[index] = y_phase.vol.sample;
//  265                 samples_vol90[index] = y_phase.vol.sample90;
//  266                 samples_curr[index] = y_phase.curr.sample;
//  267             }
//  268             else if(debug_data_type == '3')
//  269             {
//  270                 samples_vol[index] = b_phase.vol.sample;
//  271                 samples_vol90[index] = b_phase.vol.sample90;
//  272                 samples_curr[index] = b_phase.curr.sample;
//  273             }
//  274             index++;
//  275         }
//  276         else
//  277         {
//  278             index = 0;
//  279             debug_data_zc_found = 0;
//  280             debug_data_wf_ready_f = 1;
//  281         }
//  282     }
//  283 #endif    
//  284     
//  285     /* decision to calculate metrology parameters */
//  286     if((flag_phase_present_r == 1 && r_phase.vol.zc_cntr >= 50) ||
//  287        (flag_phase_present_r == 0 && r_phase.no_of_samples_temp >= SAMPLING_FREQ) ||
//  288            (r_phase.no_of_samples_temp >= (SAMPLING_FREQ+450)))
??metrology_function_0:
        MOVW      HL, #LWRD(_flag_metro2)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??power_filter_5   ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, N:_r_phase+4   ;; 1 cycle
        CMPW      AX, #0x32          ;; 1 cycle
        BNC       ??power_filter_6   ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
??power_filter_5:
        MOVW      HL, #LWRD(_flag_metro2)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BC        ??power_filter_7   ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, N:_r_phase+2   ;; 1 cycle
        CMPW      AX, #0xF42         ;; 1 cycle
        BNC       ??power_filter_6   ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
??power_filter_7:
        MOVW      AX, N:_r_phase+2   ;; 1 cycle
        CMPW      AX, #0x1104        ;; 1 cycle
        SKNC                         ;; 4 cycles
        BR        N:??power_filter_8  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  289     {
//  290         ph_ph.Ph_RY.angle.cntr = ph_ph.Ph_RY.angle.cntr_temp;
??power_filter_6:
        MOVW      AX, N:_ph_ph+6     ;; 1 cycle
        MOVW      N:_ph_ph+4, AX     ;; 1 cycle
//  291         r_phase.no_of_samples = r_phase.no_of_samples_temp;
        MOVW      AX, N:_r_phase+2   ;; 1 cycle
        MOVW      N:_r_phase, AX     ;; 1 cycle
//  292         r_phase.curr.offset_acc_cnt = r_phase.curr.offset_acc_cnt_temp;
        MOVW      BC, N:_r_phase+54  ;; 1 cycle
        MOVW      AX, N:_r_phase+52  ;; 1 cycle
        MOVW      N:_r_phase+48, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_r_phase+50, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  293         r_phase.curr.rms_acc_cnt = r_phase.curr.rms_acc_cnt_temp;
        MOVW      HL, #LWRD(_r_phase+56)  ;; 1 cycle
        MOVW      DE, #LWRD(_r_phase+64)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  294         r_phase.vol.offset_acc_cnt = r_phase.vol.offset_acc_cnt_temp;
        MOVW      BC, N:_r_phase+24  ;; 1 cycle
        MOVW      AX, N:_r_phase+22  ;; 1 cycle
        MOVW      N:_r_phase+18, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_r_phase+20, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  295         r_phase.vol.rms_acc_cnt = r_phase.vol.rms_acc_cnt_temp;
        MOVW      HL, #LWRD(_r_phase+26)  ;; 1 cycle
        MOVW      DE, #LWRD(_r_phase+34)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  296         r_phase.active.acc_cnt = r_phase.active.acc_cnt_temp;
        MOVW      HL, #LWRD(_r_phase+72)  ;; 1 cycle
        MOVW      DE, #LWRD(_r_phase+80)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  297         r_phase.reactive.acc_cnt = r_phase.reactive.acc_cnt_temp;
        MOVW      HL, #LWRD(_r_phase+112)  ;; 1 cycle
        MOVW      DE, #LWRD(_r_phase+120)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  298         
//  299         ph_ph.Ph_RY.angle.cntr_temp = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_ph_ph+6, AX     ;; 1 cycle
//  300         r_phase.no_of_samples_temp = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_r_phase+2, AX   ;; 1 cycle
//  301         r_phase.vol.zc_cntr = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_r_phase+4, AX   ;; 1 cycle
//  302         r_phase.curr.offset_acc_cnt_temp = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_r_phase+52, AX  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_r_phase+54, AX  ;; 1 cycle
//  303         r_phase.curr.rms_acc_cnt_temp = 0;
        MOVW      HL, #LWRD(_r_phase+64)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  304         r_phase.vol.offset_acc_cnt_temp = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_r_phase+22, AX  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_r_phase+24, AX  ;; 1 cycle
//  305         r_phase.vol.rms_acc_cnt_temp = 0;
        MOVW      HL, #LWRD(_r_phase+34)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  306         r_phase.active.acc_cnt_temp = 0;
        MOVW      HL, #LWRD(_r_phase+80)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  307         r_phase.reactive.acc_cnt_temp = 0;
        MOVW      HL, #LWRD(_r_phase+120)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  308         
//  309         flag_metrology_process_r = 1;
        SET1      N:_flag_metrology.0  ;; 2 cycles
//  310         flag_metrology_process_n_trigger_r = 1;
        SET1      N:_flag_metrology1.0  ;; 2 cycles
//  311         
//  312         /* Calibration */
//  313         if(cal_done_f != 1)
        CMP       N:_cal_done_f, #0x1  ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??power_filter_8  ;; 4 cycles
        ; ------------------------------------- Block: 115 cycles
//  314         {
//  315             cal_RPh.no_of_samples = cal_RPh.no_of_samples_temp;
        MOVW      AX, N:_cal_RPh+2   ;; 1 cycle
        MOVW      N:_cal_RPh, AX     ;; 1 cycle
//  316             cal_RPh.no_of_samples_temp = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_cal_RPh+2, AX   ;; 1 cycle
//  317             
//  318             cal_RPh.l_shift.active.acc_cnt = cal_RPh.l_shift.active.acc_cnt_temp;
        MOVW      HL, #LWRD(_cal_RPh+18)  ;; 1 cycle
        MOVW      DE, #LWRD(_cal_RPh+26)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  319             cal_RPh.r_shift.active.acc_cnt = cal_RPh.r_shift.active.acc_cnt_temp;
        MOVW      HL, #LWRD(_cal_RPh+62)  ;; 1 cycle
        MOVW      DE, #LWRD(_cal_RPh+70)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  320             cal_RPh.l_shift.active.acc_cnt_temp = 0;
        MOVW      HL, #LWRD(_cal_RPh+26)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  321             cal_RPh.r_shift.active.acc_cnt_temp = 0;
        MOVW      HL, #LWRD(_cal_RPh+70)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  322             
//  323             cal_RPh.l_shift.reactive.acc_cnt = cal_RPh.l_shift.reactive.acc_cnt_temp;
        MOVW      HL, #LWRD(_cal_RPh+38)  ;; 1 cycle
        MOVW      DE, #LWRD(_cal_RPh+46)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  324             cal_RPh.r_shift.reactive.acc_cnt = cal_RPh.r_shift.reactive.acc_cnt_temp;
        MOVW      HL, #LWRD(_cal_RPh+82)  ;; 1 cycle
        MOVW      DE, #LWRD(_cal_RPh+90)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  325             cal_RPh.l_shift.reactive.acc_cnt_temp = 0;
        MOVW      HL, #LWRD(_cal_RPh+46)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  326             cal_RPh.r_shift.reactive.acc_cnt_temp = 0;
        MOVW      HL, #LWRD(_cal_RPh+90)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        ; ------------------------------------- Block: 80 cycles
//  327         }
//  328     }
//  329     if((flag_phase_present_y == 1 && y_phase.vol.zc_cntr >= 50) ||
//  330        (flag_phase_present_y == 0 && y_phase.no_of_samples_temp >= SAMPLING_FREQ) ||
//  331            (y_phase.no_of_samples_temp >= (SAMPLING_FREQ+450)))
??power_filter_8:
        MOVW      HL, #LWRD(_flag_metro2)  ;; 1 cycle
        MOV1      CY, [HL].1         ;; 1 cycle
        BNC       ??power_filter_9   ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, N:_y_phase+4   ;; 1 cycle
        CMPW      AX, #0x32          ;; 1 cycle
        BNC       ??power_filter_10  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
??power_filter_9:
        MOVW      HL, #LWRD(_flag_metro2)  ;; 1 cycle
        MOV1      CY, [HL].1         ;; 1 cycle
        BC        ??power_filter_11  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, N:_y_phase+2   ;; 1 cycle
        CMPW      AX, #0xF42         ;; 1 cycle
        BNC       ??power_filter_10  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
??power_filter_11:
        MOVW      AX, N:_y_phase+2   ;; 1 cycle
        CMPW      AX, #0x1104        ;; 1 cycle
        SKNC                         ;; 4 cycles
        BR        N:??power_filter_12  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  332     {
//  333         ph_ph.Ph_YB.angle.cntr = ph_ph.Ph_YB.angle.cntr_temp;
??power_filter_10:
        MOVW      AX, N:_ph_ph+14    ;; 1 cycle
        MOVW      N:_ph_ph+12, AX    ;; 1 cycle
//  334         y_phase.no_of_samples = y_phase.no_of_samples_temp;
        MOVW      AX, N:_y_phase+2   ;; 1 cycle
        MOVW      N:_y_phase, AX     ;; 1 cycle
//  335         y_phase.curr.offset_acc_cnt = y_phase.curr.offset_acc_cnt_temp;
        MOVW      BC, N:_y_phase+54  ;; 1 cycle
        MOVW      AX, N:_y_phase+52  ;; 1 cycle
        MOVW      N:_y_phase+48, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_y_phase+50, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  336         y_phase.curr.rms_acc_cnt = y_phase.curr.rms_acc_cnt_temp;
        MOVW      HL, #LWRD(_y_phase+56)  ;; 1 cycle
        MOVW      DE, #LWRD(_y_phase+64)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  337         y_phase.vol.offset_acc_cnt = y_phase.vol.offset_acc_cnt_temp;
        MOVW      BC, N:_y_phase+24  ;; 1 cycle
        MOVW      AX, N:_y_phase+22  ;; 1 cycle
        MOVW      N:_y_phase+18, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_y_phase+20, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  338         y_phase.vol.rms_acc_cnt = y_phase.vol.rms_acc_cnt_temp;
        MOVW      HL, #LWRD(_y_phase+26)  ;; 1 cycle
        MOVW      DE, #LWRD(_y_phase+34)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  339         y_phase.active.acc_cnt = y_phase.active.acc_cnt_temp;
        MOVW      HL, #LWRD(_y_phase+72)  ;; 1 cycle
        MOVW      DE, #LWRD(_y_phase+80)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  340         y_phase.reactive.acc_cnt = y_phase.reactive.acc_cnt_temp;
        MOVW      HL, #LWRD(_y_phase+112)  ;; 1 cycle
        MOVW      DE, #LWRD(_y_phase+120)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  341         
//  342         ph_ph.Ph_YB.angle.cntr_temp = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_ph_ph+14, AX    ;; 1 cycle
//  343         y_phase.no_of_samples_temp = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_y_phase+2, AX   ;; 1 cycle
//  344         y_phase.vol.zc_cntr = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_y_phase+4, AX   ;; 1 cycle
//  345         y_phase.curr.offset_acc_cnt_temp = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_y_phase+52, AX  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_y_phase+54, AX  ;; 1 cycle
//  346         y_phase.curr.rms_acc_cnt_temp = 0;
        MOVW      HL, #LWRD(_y_phase+64)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  347         y_phase.vol.offset_acc_cnt_temp = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_y_phase+22, AX  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_y_phase+24, AX  ;; 1 cycle
//  348         y_phase.vol.rms_acc_cnt_temp = 0;
        MOVW      HL, #LWRD(_y_phase+34)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  349         y_phase.active.acc_cnt_temp = 0;
        MOVW      HL, #LWRD(_y_phase+80)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  350         y_phase.reactive.acc_cnt_temp = 0;
        MOVW      HL, #LWRD(_y_phase+120)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  351         
//  352         flag_metrology_process_y = 1;
        SET1      N:_flag_metrology.1  ;; 2 cycles
//  353         flag_metrology_process_n_trigger_y = 1;
        SET1      N:_flag_metrology1.1  ;; 2 cycles
//  354         
//  355         /* Calibration */
//  356         if(cal_done_f != 1)
        CMP       N:_cal_done_f, #0x1  ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??power_filter_12  ;; 4 cycles
        ; ------------------------------------- Block: 115 cycles
//  357         {
//  358             cal_YPh.no_of_samples = cal_YPh.no_of_samples_temp;
        MOVW      AX, N:_cal_YPh+2   ;; 1 cycle
        MOVW      N:_cal_YPh, AX     ;; 1 cycle
//  359             cal_YPh.no_of_samples_temp = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_cal_YPh+2, AX   ;; 1 cycle
//  360             
//  361             cal_YPh.l_shift.active.acc_cnt = cal_YPh.l_shift.active.acc_cnt_temp;
        MOVW      HL, #LWRD(_cal_YPh+18)  ;; 1 cycle
        MOVW      DE, #LWRD(_cal_YPh+26)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  362             cal_YPh.r_shift.active.acc_cnt = cal_YPh.r_shift.active.acc_cnt_temp;
        MOVW      HL, #LWRD(_cal_YPh+62)  ;; 1 cycle
        MOVW      DE, #LWRD(_cal_YPh+70)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  363             cal_YPh.l_shift.active.acc_cnt_temp = 0;
        MOVW      HL, #LWRD(_cal_YPh+26)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  364             cal_YPh.r_shift.active.acc_cnt_temp = 0;
        MOVW      HL, #LWRD(_cal_YPh+70)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  365             
//  366             cal_YPh.l_shift.reactive.acc_cnt = cal_YPh.l_shift.reactive.acc_cnt_temp;
        MOVW      HL, #LWRD(_cal_YPh+38)  ;; 1 cycle
        MOVW      DE, #LWRD(_cal_YPh+46)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  367             cal_YPh.r_shift.reactive.acc_cnt = cal_YPh.r_shift.reactive.acc_cnt_temp;
        MOVW      HL, #LWRD(_cal_YPh+82)  ;; 1 cycle
        MOVW      DE, #LWRD(_cal_YPh+90)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  368             cal_YPh.l_shift.reactive.acc_cnt_temp = 0;
        MOVW      HL, #LWRD(_cal_YPh+46)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  369             cal_YPh.r_shift.reactive.acc_cnt_temp = 0;
        MOVW      HL, #LWRD(_cal_YPh+90)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        ; ------------------------------------- Block: 80 cycles
//  370         }
//  371     }
//  372     if((flag_phase_present_b == 1 && b_phase.vol.zc_cntr >= 50) ||
//  373        (flag_phase_present_b == 0 && b_phase.no_of_samples_temp >= SAMPLING_FREQ) ||
//  374            (b_phase.no_of_samples_temp >= (SAMPLING_FREQ+450)))
??power_filter_12:
        MOVW      HL, #LWRD(_flag_metro2)  ;; 1 cycle
        MOV1      CY, [HL].2         ;; 1 cycle
        BNC       ??power_filter_13  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, N:_b_phase+4   ;; 1 cycle
        CMPW      AX, #0x32          ;; 1 cycle
        BNC       ??power_filter_14  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
??power_filter_13:
        MOVW      HL, #LWRD(_flag_metro2)  ;; 1 cycle
        MOV1      CY, [HL].2         ;; 1 cycle
        BC        ??power_filter_15  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, N:_b_phase+2   ;; 1 cycle
        CMPW      AX, #0xF42         ;; 1 cycle
        BNC       ??power_filter_14  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
??power_filter_15:
        MOVW      AX, N:_b_phase+2   ;; 1 cycle
        CMPW      AX, #0x1104        ;; 1 cycle
        SKNC                         ;; 4 cycles
        BR        N:??power_filter_16  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  375     {
//  376         ph_ph.Ph_BR.angle.cntr = ph_ph.Ph_BR.angle.cntr_temp;
??power_filter_14:
        MOVW      AX, N:_ph_ph+22    ;; 1 cycle
        MOVW      N:_ph_ph+20, AX    ;; 1 cycle
//  377         b_phase.no_of_samples = b_phase.no_of_samples_temp;
        MOVW      AX, N:_b_phase+2   ;; 1 cycle
        MOVW      N:_b_phase, AX     ;; 1 cycle
//  378         b_phase.curr.offset_acc_cnt = b_phase.curr.offset_acc_cnt_temp;
        MOVW      BC, N:_b_phase+54  ;; 1 cycle
        MOVW      AX, N:_b_phase+52  ;; 1 cycle
        MOVW      N:_b_phase+48, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_b_phase+50, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  379         b_phase.curr.rms_acc_cnt = b_phase.curr.rms_acc_cnt_temp;
        MOVW      HL, #LWRD(_b_phase+56)  ;; 1 cycle
        MOVW      DE, #LWRD(_b_phase+64)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  380         b_phase.vol.offset_acc_cnt = b_phase.vol.offset_acc_cnt_temp;
        MOVW      BC, N:_b_phase+24  ;; 1 cycle
        MOVW      AX, N:_b_phase+22  ;; 1 cycle
        MOVW      N:_b_phase+18, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_b_phase+20, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  381         b_phase.vol.rms_acc_cnt = b_phase.vol.rms_acc_cnt_temp;
        MOVW      HL, #LWRD(_b_phase+26)  ;; 1 cycle
        MOVW      DE, #LWRD(_b_phase+34)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  382         b_phase.active.acc_cnt = b_phase.active.acc_cnt_temp;
        MOVW      HL, #LWRD(_b_phase+72)  ;; 1 cycle
        MOVW      DE, #LWRD(_b_phase+80)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  383         b_phase.reactive.acc_cnt = b_phase.reactive.acc_cnt_temp;
        MOVW      HL, #LWRD(_b_phase+112)  ;; 1 cycle
        MOVW      DE, #LWRD(_b_phase+120)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  384         
//  385         ph_ph.Ph_BR.angle.cntr_temp = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_ph_ph+22, AX    ;; 1 cycle
//  386         b_phase.no_of_samples_temp = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_b_phase+2, AX   ;; 1 cycle
//  387         b_phase.vol.zc_cntr = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_b_phase+4, AX   ;; 1 cycle
//  388         b_phase.curr.offset_acc_cnt_temp = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_b_phase+52, AX  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_b_phase+54, AX  ;; 1 cycle
//  389         b_phase.curr.rms_acc_cnt_temp = 0;
        MOVW      HL, #LWRD(_b_phase+64)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  390         b_phase.vol.offset_acc_cnt_temp = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_b_phase+22, AX  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_b_phase+24, AX  ;; 1 cycle
//  391         b_phase.vol.rms_acc_cnt_temp = 0;
        MOVW      HL, #LWRD(_b_phase+34)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  392         b_phase.active.acc_cnt_temp = 0;
        MOVW      HL, #LWRD(_b_phase+80)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  393         b_phase.reactive.acc_cnt_temp = 0;
        MOVW      HL, #LWRD(_b_phase+120)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  394         
//  395         flag_metrology_process_b = 1;
        SET1      N:_flag_metrology.2  ;; 2 cycles
//  396         flag_metrology_process_n_trigger_b = 1;
        SET1      N:_flag_metrology1.2  ;; 2 cycles
//  397         
//  398         /* Calibration */
//  399         if(cal_done_f != 1)
        CMP       N:_cal_done_f, #0x1  ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??power_filter_16  ;; 4 cycles
        ; ------------------------------------- Block: 115 cycles
//  400         {
//  401             cal_BPh.no_of_samples = cal_BPh.no_of_samples_temp;
        MOVW      AX, N:_cal_BPh+2   ;; 1 cycle
        MOVW      N:_cal_BPh, AX     ;; 1 cycle
//  402             cal_BPh.no_of_samples_temp = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_cal_BPh+2, AX   ;; 1 cycle
//  403             
//  404             cal_BPh.l_shift.active.acc_cnt = cal_BPh.l_shift.active.acc_cnt_temp;
        MOVW      HL, #LWRD(_cal_BPh+18)  ;; 1 cycle
        MOVW      DE, #LWRD(_cal_BPh+26)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  405             cal_BPh.r_shift.active.acc_cnt = cal_BPh.r_shift.active.acc_cnt_temp;
        MOVW      HL, #LWRD(_cal_BPh+62)  ;; 1 cycle
        MOVW      DE, #LWRD(_cal_BPh+70)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  406             cal_BPh.l_shift.active.acc_cnt_temp = 0;
        MOVW      HL, #LWRD(_cal_BPh+26)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  407             cal_BPh.r_shift.active.acc_cnt_temp = 0;
        MOVW      HL, #LWRD(_cal_BPh+70)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  408             
//  409             cal_BPh.l_shift.reactive.acc_cnt = cal_BPh.l_shift.reactive.acc_cnt_temp;
        MOVW      HL, #LWRD(_cal_BPh+38)  ;; 1 cycle
        MOVW      DE, #LWRD(_cal_BPh+46)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  410             cal_BPh.r_shift.reactive.acc_cnt = cal_BPh.r_shift.reactive.acc_cnt_temp;
        MOVW      HL, #LWRD(_cal_BPh+82)  ;; 1 cycle
        MOVW      DE, #LWRD(_cal_BPh+90)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  411             cal_BPh.l_shift.reactive.acc_cnt_temp = 0;
        MOVW      HL, #LWRD(_cal_BPh+46)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  412             cal_BPh.r_shift.reactive.acc_cnt_temp = 0;
        MOVW      HL, #LWRD(_cal_BPh+90)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        ; ------------------------------------- Block: 80 cycles
//  413         }
//  414     }
//  415     
//  416     if(flag_phase_present_r == 1)
??power_filter_16:
        MOVW      HL, #LWRD(_flag_metro2)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??power_filter_17  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  417     {
//  418         if(flag_metrology_process_n_trigger_r == 1)
        MOVW      HL, #LWRD(_flag_metrology1)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??power_filter_18  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  419         {
//  420             flag_metrology_process_n_trigger_r = 0;
        CLR1      N:_flag_metrology1.0  ;; 2 cycles
//  421             flag_neutral_trigger_from_phase = 1;
        SET1      N:_flag_metrology1.3  ;; 2 cycles
        BR        S:??power_filter_18  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
//  422         }
//  423     }
//  424     else if(flag_phase_present_y == 1)
??power_filter_17:
        MOVW      HL, #LWRD(_flag_metro2)  ;; 1 cycle
        MOV1      CY, [HL].1         ;; 1 cycle
        BNC       ??power_filter_19  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  425     {
//  426         if(flag_metrology_process_n_trigger_y == 1)
        MOVW      HL, #LWRD(_flag_metrology1)  ;; 1 cycle
        MOV1      CY, [HL].1         ;; 1 cycle
        BNC       ??power_filter_18  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  427         {
//  428             flag_metrology_process_n_trigger_y = 0;
        CLR1      N:_flag_metrology1.1  ;; 2 cycles
//  429             flag_neutral_trigger_from_phase = 1;
        SET1      N:_flag_metrology1.3  ;; 2 cycles
        BR        S:??power_filter_18  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
//  430         }
//  431     }
//  432     else if(flag_phase_present_b == 1)
??power_filter_19:
        MOVW      HL, #LWRD(_flag_metro2)  ;; 1 cycle
        MOV1      CY, [HL].2         ;; 1 cycle
        BNC       ??power_filter_18  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  433     {
//  434         if(flag_metrology_process_n_trigger_b == 1)
        MOVW      HL, #LWRD(_flag_metrology1)  ;; 1 cycle
        MOV1      CY, [HL].2         ;; 1 cycle
        BNC       ??power_filter_18  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  435         {
//  436             flag_metrology_process_n_trigger_b = 0;
        CLR1      N:_flag_metrology1.2  ;; 2 cycles
//  437             flag_neutral_trigger_from_phase = 1;
        SET1      N:_flag_metrology1.3  ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
//  438         }
//  439     }
//  440     
//  441     if(flag_neutral_trigger_from_phase == 1 ||
//  442        n_phase.no_of_samples_temp >= (SAMPLING_FREQ+450))
??power_filter_18:
        MOVW      HL, #LWRD(_flag_metrology1)  ;; 1 cycle
        MOV1      CY, [HL].3         ;; 1 cycle
        BC        ??power_filter_20  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, N:_n_phase+2   ;; 1 cycle
        CMPW      AX, #0x1104        ;; 1 cycle
        BC        ??power_filter_21  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  443     {
//  444         flag_neutral_trigger_from_phase = 0;
??power_filter_20:
        CLR1      N:_flag_metrology1.3  ;; 2 cycles
//  445         n_phase.no_of_samples = n_phase.no_of_samples_temp;
        MOVW      AX, N:_n_phase+2   ;; 1 cycle
        MOVW      N:_n_phase, AX     ;; 1 cycle
//  446         n_phase.curr.offset_acc_cnt = n_phase.curr.offset_acc_cnt_temp;
        MOVW      BC, N:_n_phase+16  ;; 1 cycle
        MOVW      AX, N:_n_phase+14  ;; 1 cycle
        MOVW      N:_n_phase+10, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_n_phase+12, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  447         n_phase.curr.rms_acc_cnt = n_phase.curr.rms_acc_cnt_temp;
        MOVW      HL, #LWRD(_n_phase+18)  ;; 1 cycle
        MOVW      DE, #LWRD(_n_phase+26)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  448         n_phase.curr.offset_acc_cnt_temp = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_n_phase+14, AX  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_n_phase+16, AX  ;; 1 cycle
//  449         n_phase.curr.rms_acc_cnt_temp = 0;
        MOVW      HL, #LWRD(_n_phase+26)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  450         n_phase.no_of_samples_temp = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_n_phase+2, AX   ;; 1 cycle
//  451         
//  452         flag_metrology_process_n = 1;
        SET1      N:_flag_metrology.3  ;; 2 cycles
        ; ------------------------------------- Block: 37 cycles
//  453     }
//  454     
//  455     /* RMS calculation accumulation */
//  456     HWMultiplierAccSigned(r_phase.vol.sample, r_phase.vol.sample, r_phase.vol.rms_acc_cnt_temp);
??power_filter_21:
        MOV       0x29A, #0xC0       ;; 1 cycle
        MOVW      HL, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      DE, #LWRD(_r_phase+34)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        MOVW      AX, N:_mac_union_4int  ;; 1 cycle
        MOVW      0x290, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+2  ;; 1 cycle
        MOVW      0x292, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+4  ;; 1 cycle
        MOVW      0x294, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+6  ;; 1 cycle
        MOVW      0x296, AX          ;; 1 cycle
        MOVW      AX, N:_r_phase+8   ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0x28C, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0x28E, AX          ;; 1 cycle
        MOVW      AX, N:_r_phase+8   ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0xFFF3C, AX        ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0xFFF3E, AX        ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        MOVW      AX, 0x290          ;; 1 cycle
        MOVW      N:_mac_union_4int, AX  ;; 1 cycle
        MOVW      AX, 0x292          ;; 1 cycle
        MOVW      N:_mac_union_4int+2, AX  ;; 1 cycle
        MOVW      AX, 0x294          ;; 1 cycle
        MOVW      N:_mac_union_4int+4, AX  ;; 1 cycle
        MOVW      AX, 0x296          ;; 1 cycle
        MOVW      N:_mac_union_4int+6, AX  ;; 1 cycle
        MOVW      HL, #LWRD(_r_phase+34)  ;; 1 cycle
        MOVW      DE, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  457     HWMultiplierAccSigned(y_phase.vol.sample, y_phase.vol.sample, y_phase.vol.rms_acc_cnt_temp);
        MOV       0x29A, #0xC0       ;; 1 cycle
        MOVW      HL, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      DE, #LWRD(_y_phase+34)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        MOVW      AX, N:_mac_union_4int  ;; 1 cycle
        MOVW      0x290, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+2  ;; 1 cycle
        MOVW      0x292, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+4  ;; 1 cycle
        MOVW      0x294, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+6  ;; 1 cycle
        MOVW      0x296, AX          ;; 1 cycle
        MOVW      AX, N:_y_phase+8   ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0x28C, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0x28E, AX          ;; 1 cycle
        MOVW      AX, N:_y_phase+8   ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0xFFF3C, AX        ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0xFFF3E, AX        ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        MOVW      AX, 0x290          ;; 1 cycle
        MOVW      N:_mac_union_4int, AX  ;; 1 cycle
        MOVW      AX, 0x292          ;; 1 cycle
        MOVW      N:_mac_union_4int+2, AX  ;; 1 cycle
        MOVW      AX, 0x294          ;; 1 cycle
        MOVW      N:_mac_union_4int+4, AX  ;; 1 cycle
        MOVW      AX, 0x296          ;; 1 cycle
        MOVW      N:_mac_union_4int+6, AX  ;; 1 cycle
        MOVW      HL, #LWRD(_y_phase+34)  ;; 1 cycle
        MOVW      DE, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  458     HWMultiplierAccSigned(b_phase.vol.sample, b_phase.vol.sample, b_phase.vol.rms_acc_cnt_temp);
        MOV       0x29A, #0xC0       ;; 1 cycle
        MOVW      HL, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      DE, #LWRD(_b_phase+34)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        MOVW      AX, N:_mac_union_4int  ;; 1 cycle
        MOVW      0x290, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+2  ;; 1 cycle
        MOVW      0x292, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+4  ;; 1 cycle
        MOVW      0x294, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+6  ;; 1 cycle
        MOVW      0x296, AX          ;; 1 cycle
        MOVW      AX, N:_b_phase+8   ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0x28C, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0x28E, AX          ;; 1 cycle
        MOVW      AX, N:_b_phase+8   ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0xFFF3C, AX        ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0xFFF3E, AX        ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        MOVW      AX, 0x290          ;; 1 cycle
        MOVW      N:_mac_union_4int, AX  ;; 1 cycle
        MOVW      AX, 0x292          ;; 1 cycle
        MOVW      N:_mac_union_4int+2, AX  ;; 1 cycle
        MOVW      AX, 0x294          ;; 1 cycle
        MOVW      N:_mac_union_4int+4, AX  ;; 1 cycle
        MOVW      AX, 0x296          ;; 1 cycle
        MOVW      N:_mac_union_4int+6, AX  ;; 1 cycle
        MOVW      HL, #LWRD(_b_phase+34)  ;; 1 cycle
        MOVW      DE, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  459     HWMultiplierAccSigned(r_phase.curr.sample, r_phase.curr.sample, r_phase.curr.rms_acc_cnt_temp);
        MOV       0x29A, #0xC0       ;; 1 cycle
        MOVW      HL, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      DE, #LWRD(_r_phase+64)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        MOVW      AX, N:_mac_union_4int  ;; 1 cycle
        MOVW      0x290, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+2  ;; 1 cycle
        MOVW      0x292, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+4  ;; 1 cycle
        MOVW      0x294, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+6  ;; 1 cycle
        MOVW      0x296, AX          ;; 1 cycle
        MOVW      AX, N:_r_phase+44  ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0x28C, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0x28E, AX          ;; 1 cycle
        MOVW      AX, N:_r_phase+44  ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0xFFF3C, AX        ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0xFFF3E, AX        ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        MOVW      AX, 0x290          ;; 1 cycle
        MOVW      N:_mac_union_4int, AX  ;; 1 cycle
        MOVW      AX, 0x292          ;; 1 cycle
        MOVW      N:_mac_union_4int+2, AX  ;; 1 cycle
        MOVW      AX, 0x294          ;; 1 cycle
        MOVW      N:_mac_union_4int+4, AX  ;; 1 cycle
        MOVW      AX, 0x296          ;; 1 cycle
        MOVW      N:_mac_union_4int+6, AX  ;; 1 cycle
        MOVW      HL, #LWRD(_r_phase+64)  ;; 1 cycle
        MOVW      DE, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  460     HWMultiplierAccSigned(y_phase.curr.sample, y_phase.curr.sample, y_phase.curr.rms_acc_cnt_temp);
        MOV       0x29A, #0xC0       ;; 1 cycle
        MOVW      HL, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      DE, #LWRD(_y_phase+64)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        MOVW      AX, N:_mac_union_4int  ;; 1 cycle
        MOVW      0x290, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+2  ;; 1 cycle
        MOVW      0x292, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+4  ;; 1 cycle
        MOVW      0x294, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+6  ;; 1 cycle
        MOVW      0x296, AX          ;; 1 cycle
        MOVW      AX, N:_y_phase+44  ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0x28C, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0x28E, AX          ;; 1 cycle
        MOVW      AX, N:_y_phase+44  ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0xFFF3C, AX        ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0xFFF3E, AX        ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        MOVW      AX, 0x290          ;; 1 cycle
        MOVW      N:_mac_union_4int, AX  ;; 1 cycle
        MOVW      AX, 0x292          ;; 1 cycle
        MOVW      N:_mac_union_4int+2, AX  ;; 1 cycle
        MOVW      AX, 0x294          ;; 1 cycle
        MOVW      N:_mac_union_4int+4, AX  ;; 1 cycle
        MOVW      AX, 0x296          ;; 1 cycle
        MOVW      N:_mac_union_4int+6, AX  ;; 1 cycle
        MOVW      HL, #LWRD(_y_phase+64)  ;; 1 cycle
        MOVW      DE, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  461     HWMultiplierAccSigned(b_phase.curr.sample, b_phase.curr.sample, b_phase.curr.rms_acc_cnt_temp);
        MOV       0x29A, #0xC0       ;; 1 cycle
        MOVW      HL, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      DE, #LWRD(_b_phase+64)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        MOVW      AX, N:_mac_union_4int  ;; 1 cycle
        MOVW      0x290, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+2  ;; 1 cycle
        MOVW      0x292, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+4  ;; 1 cycle
        MOVW      0x294, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+6  ;; 1 cycle
        MOVW      0x296, AX          ;; 1 cycle
        MOVW      AX, N:_b_phase+44  ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0x28C, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0x28E, AX          ;; 1 cycle
        MOVW      AX, N:_b_phase+44  ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0xFFF3C, AX        ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0xFFF3E, AX        ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        MOVW      AX, 0x290          ;; 1 cycle
        MOVW      N:_mac_union_4int, AX  ;; 1 cycle
        MOVW      AX, 0x292          ;; 1 cycle
        MOVW      N:_mac_union_4int+2, AX  ;; 1 cycle
        MOVW      AX, 0x294          ;; 1 cycle
        MOVW      N:_mac_union_4int+4, AX  ;; 1 cycle
        MOVW      AX, 0x296          ;; 1 cycle
        MOVW      N:_mac_union_4int+6, AX  ;; 1 cycle
        MOVW      HL, #LWRD(_b_phase+64)  ;; 1 cycle
        MOVW      DE, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  462     HWMultiplierAccSigned(n_phase.curr.sample, n_phase.curr.sample, n_phase.curr.rms_acc_cnt_temp);
        MOV       0x29A, #0xC0       ;; 1 cycle
        MOVW      HL, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      DE, #LWRD(_n_phase+26)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        MOVW      AX, N:_mac_union_4int  ;; 1 cycle
        MOVW      0x290, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+2  ;; 1 cycle
        MOVW      0x292, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+4  ;; 1 cycle
        MOVW      0x294, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+6  ;; 1 cycle
        MOVW      0x296, AX          ;; 1 cycle
        MOVW      AX, N:_n_phase+6   ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0x28C, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0x28E, AX          ;; 1 cycle
        MOVW      AX, N:_n_phase+6   ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0xFFF3C, AX        ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0xFFF3E, AX        ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        MOVW      AX, 0x290          ;; 1 cycle
        MOVW      N:_mac_union_4int, AX  ;; 1 cycle
        MOVW      AX, 0x292          ;; 1 cycle
        MOVW      N:_mac_union_4int+2, AX  ;; 1 cycle
        MOVW      AX, 0x294          ;; 1 cycle
        MOVW      N:_mac_union_4int+4, AX  ;; 1 cycle
        MOVW      AX, 0x296          ;; 1 cycle
        MOVW      N:_mac_union_4int+6, AX  ;; 1 cycle
        MOVW      HL, #LWRD(_n_phase+26)  ;; 1 cycle
        MOVW      DE, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  463     
//  464     /* energy calculation accumulation */
//  465     HWMultiplierAccSigned(r_phase.vol.sample, r_phase.curr.sample, r_phase.active.acc_cnt_temp);
        MOV       0x29A, #0xC0       ;; 1 cycle
        MOVW      HL, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      DE, #LWRD(_r_phase+80)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        MOVW      AX, N:_mac_union_4int  ;; 1 cycle
        MOVW      0x290, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+2  ;; 1 cycle
        MOVW      0x292, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+4  ;; 1 cycle
        MOVW      0x294, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+6  ;; 1 cycle
        MOVW      0x296, AX          ;; 1 cycle
        MOVW      AX, N:_r_phase+8   ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0x28C, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0x28E, AX          ;; 1 cycle
        MOVW      AX, N:_r_phase+44  ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0xFFF3C, AX        ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0xFFF3E, AX        ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        MOVW      AX, 0x290          ;; 1 cycle
        MOVW      N:_mac_union_4int, AX  ;; 1 cycle
        MOVW      AX, 0x292          ;; 1 cycle
        MOVW      N:_mac_union_4int+2, AX  ;; 1 cycle
        MOVW      AX, 0x294          ;; 1 cycle
        MOVW      N:_mac_union_4int+4, AX  ;; 1 cycle
        MOVW      AX, 0x296          ;; 1 cycle
        MOVW      N:_mac_union_4int+6, AX  ;; 1 cycle
        MOVW      HL, #LWRD(_r_phase+80)  ;; 1 cycle
        MOVW      DE, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  466     HWMultiplierAccSigned(y_phase.vol.sample, y_phase.curr.sample, y_phase.active.acc_cnt_temp);     
        MOV       0x29A, #0xC0       ;; 1 cycle
        MOVW      HL, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      DE, #LWRD(_y_phase+80)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        MOVW      AX, N:_mac_union_4int  ;; 1 cycle
        MOVW      0x290, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+2  ;; 1 cycle
        MOVW      0x292, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+4  ;; 1 cycle
        MOVW      0x294, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+6  ;; 1 cycle
        MOVW      0x296, AX          ;; 1 cycle
        MOVW      AX, N:_y_phase+8   ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0x28C, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0x28E, AX          ;; 1 cycle
        MOVW      AX, N:_y_phase+44  ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0xFFF3C, AX        ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0xFFF3E, AX        ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        MOVW      AX, 0x290          ;; 1 cycle
        MOVW      N:_mac_union_4int, AX  ;; 1 cycle
        MOVW      AX, 0x292          ;; 1 cycle
        MOVW      N:_mac_union_4int+2, AX  ;; 1 cycle
        MOVW      AX, 0x294          ;; 1 cycle
        MOVW      N:_mac_union_4int+4, AX  ;; 1 cycle
        MOVW      AX, 0x296          ;; 1 cycle
        MOVW      N:_mac_union_4int+6, AX  ;; 1 cycle
        MOVW      HL, #LWRD(_y_phase+80)  ;; 1 cycle
        MOVW      DE, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  467     HWMultiplierAccSigned(b_phase.vol.sample, b_phase.curr.sample, b_phase.active.acc_cnt_temp);     
        MOV       0x29A, #0xC0       ;; 1 cycle
        MOVW      HL, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      DE, #LWRD(_b_phase+80)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        MOVW      AX, N:_mac_union_4int  ;; 1 cycle
        MOVW      0x290, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+2  ;; 1 cycle
        MOVW      0x292, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+4  ;; 1 cycle
        MOVW      0x294, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+6  ;; 1 cycle
        MOVW      0x296, AX          ;; 1 cycle
        MOVW      AX, N:_b_phase+8   ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0x28C, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0x28E, AX          ;; 1 cycle
        MOVW      AX, N:_b_phase+44  ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0xFFF3C, AX        ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0xFFF3E, AX        ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        MOVW      AX, 0x290          ;; 1 cycle
        MOVW      N:_mac_union_4int, AX  ;; 1 cycle
        MOVW      AX, 0x292          ;; 1 cycle
        MOVW      N:_mac_union_4int+2, AX  ;; 1 cycle
        MOVW      AX, 0x294          ;; 1 cycle
        MOVW      N:_mac_union_4int+4, AX  ;; 1 cycle
        MOVW      AX, 0x296          ;; 1 cycle
        MOVW      N:_mac_union_4int+6, AX  ;; 1 cycle
        MOVW      HL, #LWRD(_b_phase+80)  ;; 1 cycle
        MOVW      DE, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  468     HWMultiplierAccSigned(r_phase.vol.sample90, r_phase.curr.sample, r_phase.reactive.acc_cnt_temp);     
        MOV       0x29A, #0xC0       ;; 1 cycle
        MOVW      HL, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      DE, #LWRD(_r_phase+120)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        MOVW      AX, N:_mac_union_4int  ;; 1 cycle
        MOVW      0x290, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+2  ;; 1 cycle
        MOVW      0x292, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+4  ;; 1 cycle
        MOVW      0x294, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+6  ;; 1 cycle
        MOVW      0x296, AX          ;; 1 cycle
        MOVW      AX, N:_r_phase+12  ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0x28C, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0x28E, AX          ;; 1 cycle
        MOVW      AX, N:_r_phase+44  ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0xFFF3C, AX        ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0xFFF3E, AX        ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        MOVW      AX, 0x290          ;; 1 cycle
        MOVW      N:_mac_union_4int, AX  ;; 1 cycle
        MOVW      AX, 0x292          ;; 1 cycle
        MOVW      N:_mac_union_4int+2, AX  ;; 1 cycle
        MOVW      AX, 0x294          ;; 1 cycle
        MOVW      N:_mac_union_4int+4, AX  ;; 1 cycle
        MOVW      AX, 0x296          ;; 1 cycle
        MOVW      N:_mac_union_4int+6, AX  ;; 1 cycle
        MOVW      HL, #LWRD(_r_phase+120)  ;; 1 cycle
        MOVW      DE, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  469     HWMultiplierAccSigned(y_phase.vol.sample90, y_phase.curr.sample, y_phase.reactive.acc_cnt_temp);     
        MOV       0x29A, #0xC0       ;; 1 cycle
        MOVW      HL, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      DE, #LWRD(_y_phase+120)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        MOVW      AX, N:_mac_union_4int  ;; 1 cycle
        MOVW      0x290, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+2  ;; 1 cycle
        MOVW      0x292, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+4  ;; 1 cycle
        MOVW      0x294, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+6  ;; 1 cycle
        MOVW      0x296, AX          ;; 1 cycle
        MOVW      AX, N:_y_phase+12  ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0x28C, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0x28E, AX          ;; 1 cycle
        MOVW      AX, N:_y_phase+44  ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0xFFF3C, AX        ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0xFFF3E, AX        ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        MOVW      AX, 0x290          ;; 1 cycle
        MOVW      N:_mac_union_4int, AX  ;; 1 cycle
        MOVW      AX, 0x292          ;; 1 cycle
        MOVW      N:_mac_union_4int+2, AX  ;; 1 cycle
        MOVW      AX, 0x294          ;; 1 cycle
        MOVW      N:_mac_union_4int+4, AX  ;; 1 cycle
        MOVW      AX, 0x296          ;; 1 cycle
        MOVW      N:_mac_union_4int+6, AX  ;; 1 cycle
        MOVW      HL, #LWRD(_y_phase+120)  ;; 1 cycle
        MOVW      DE, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  470     HWMultiplierAccSigned(b_phase.vol.sample90, b_phase.curr.sample, b_phase.reactive.acc_cnt_temp);     
        MOV       0x29A, #0xC0       ;; 1 cycle
        MOVW      HL, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      DE, #LWRD(_b_phase+120)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        MOVW      AX, N:_mac_union_4int  ;; 1 cycle
        MOVW      0x290, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+2  ;; 1 cycle
        MOVW      0x292, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+4  ;; 1 cycle
        MOVW      0x294, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+6  ;; 1 cycle
        MOVW      0x296, AX          ;; 1 cycle
        MOVW      AX, N:_b_phase+12  ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0x28C, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0x28E, AX          ;; 1 cycle
        MOVW      AX, N:_b_phase+44  ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0xFFF3C, AX        ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0xFFF3E, AX        ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        MOVW      AX, 0x290          ;; 1 cycle
        MOVW      N:_mac_union_4int, AX  ;; 1 cycle
        MOVW      AX, 0x292          ;; 1 cycle
        MOVW      N:_mac_union_4int+2, AX  ;; 1 cycle
        MOVW      AX, 0x294          ;; 1 cycle
        MOVW      N:_mac_union_4int+4, AX  ;; 1 cycle
        MOVW      AX, 0x296          ;; 1 cycle
        MOVW      N:_mac_union_4int+6, AX  ;; 1 cycle
        MOVW      HL, #LWRD(_b_phase+120)  ;; 1 cycle
        MOVW      DE, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  471     
//  472     if(flag_thd_start_sampling == 1 && cal_done_f == 1)
        MOVW      HL, #LWRD(_flag_thd1)  ;; 1 cycle
        MOV1      CY, [HL].6         ;; 1 cycle
        BNC       ??power_filter_22  ;; 4 cycles
        ; ------------------------------------- Block: 825 cycles
        CMP       N:_cal_done_f, #0x1  ;; 1 cycle
        SKNZ                         ;; 1 cycle
          CFI FunCall _thd_accumulation
        ; ------------------------------------- Block: 2 cycles
//  473     {
//  474         thd_accumulation();
        CALL      _thd_accumulation  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
//  475     }
//  476     /* Calibration */
//  477     if(cal_done_f != 1)
??power_filter_22:
        CMP       N:_cal_done_f, #0x1  ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??power_filter_23  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  478     {
//  479         HWMultiplierAccSigned(cal_RPh.l_shift.vol.sample,cal_RPh.curr_sample,cal_RPh.l_shift.active.acc_cnt_temp);
        MOV       0x29A, #0xC0       ;; 1 cycle
        MOVW      HL, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      DE, #LWRD(_cal_RPh+26)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        MOVW      AX, N:_mac_union_4int  ;; 1 cycle
        MOVW      0x290, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+2  ;; 1 cycle
        MOVW      0x292, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+4  ;; 1 cycle
        MOVW      0x294, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+6  ;; 1 cycle
        MOVW      0x296, AX          ;; 1 cycle
        MOVW      AX, N:_cal_RPh+10  ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0x28C, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0x28E, AX          ;; 1 cycle
        MOVW      AX, N:_cal_RPh+4   ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0xFFF3C, AX        ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0xFFF3E, AX        ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        MOVW      AX, 0x290          ;; 1 cycle
        MOVW      N:_mac_union_4int, AX  ;; 1 cycle
        MOVW      AX, 0x292          ;; 1 cycle
        MOVW      N:_mac_union_4int+2, AX  ;; 1 cycle
        MOVW      AX, 0x294          ;; 1 cycle
        MOVW      N:_mac_union_4int+4, AX  ;; 1 cycle
        MOVW      AX, 0x296          ;; 1 cycle
        MOVW      N:_mac_union_4int+6, AX  ;; 1 cycle
        MOVW      HL, #LWRD(_cal_RPh+26)  ;; 1 cycle
        MOVW      DE, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  480         HWMultiplierAccSigned(cal_YPh.l_shift.vol.sample,cal_YPh.curr_sample,cal_YPh.l_shift.active.acc_cnt_temp);
        MOV       0x29A, #0xC0       ;; 1 cycle
        MOVW      HL, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      DE, #LWRD(_cal_YPh+26)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        MOVW      AX, N:_mac_union_4int  ;; 1 cycle
        MOVW      0x290, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+2  ;; 1 cycle
        MOVW      0x292, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+4  ;; 1 cycle
        MOVW      0x294, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+6  ;; 1 cycle
        MOVW      0x296, AX          ;; 1 cycle
        MOVW      AX, N:_cal_YPh+10  ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0x28C, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0x28E, AX          ;; 1 cycle
        MOVW      AX, N:_cal_YPh+4   ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0xFFF3C, AX        ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0xFFF3E, AX        ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        MOVW      AX, 0x290          ;; 1 cycle
        MOVW      N:_mac_union_4int, AX  ;; 1 cycle
        MOVW      AX, 0x292          ;; 1 cycle
        MOVW      N:_mac_union_4int+2, AX  ;; 1 cycle
        MOVW      AX, 0x294          ;; 1 cycle
        MOVW      N:_mac_union_4int+4, AX  ;; 1 cycle
        MOVW      AX, 0x296          ;; 1 cycle
        MOVW      N:_mac_union_4int+6, AX  ;; 1 cycle
        MOVW      HL, #LWRD(_cal_YPh+26)  ;; 1 cycle
        MOVW      DE, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  481         HWMultiplierAccSigned(cal_BPh.l_shift.vol.sample,cal_BPh.curr_sample,cal_BPh.l_shift.active.acc_cnt_temp);
        MOV       0x29A, #0xC0       ;; 1 cycle
        MOVW      HL, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      DE, #LWRD(_cal_BPh+26)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        MOVW      AX, N:_mac_union_4int  ;; 1 cycle
        MOVW      0x290, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+2  ;; 1 cycle
        MOVW      0x292, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+4  ;; 1 cycle
        MOVW      0x294, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+6  ;; 1 cycle
        MOVW      0x296, AX          ;; 1 cycle
        MOVW      AX, N:_cal_BPh+10  ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0x28C, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0x28E, AX          ;; 1 cycle
        MOVW      AX, N:_cal_BPh+4   ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0xFFF3C, AX        ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0xFFF3E, AX        ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        MOVW      AX, 0x290          ;; 1 cycle
        MOVW      N:_mac_union_4int, AX  ;; 1 cycle
        MOVW      AX, 0x292          ;; 1 cycle
        MOVW      N:_mac_union_4int+2, AX  ;; 1 cycle
        MOVW      AX, 0x294          ;; 1 cycle
        MOVW      N:_mac_union_4int+4, AX  ;; 1 cycle
        MOVW      AX, 0x296          ;; 1 cycle
        MOVW      N:_mac_union_4int+6, AX  ;; 1 cycle
        MOVW      HL, #LWRD(_cal_BPh+26)  ;; 1 cycle
        MOVW      DE, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  482         HWMultiplierAccSigned(cal_RPh.r_shift.vol.sample,cal_RPh.curr_sample,cal_RPh.r_shift.active.acc_cnt_temp);
        MOV       0x29A, #0xC0       ;; 1 cycle
        MOVW      HL, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      DE, #LWRD(_cal_RPh+70)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        MOVW      AX, N:_mac_union_4int  ;; 1 cycle
        MOVW      0x290, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+2  ;; 1 cycle
        MOVW      0x292, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+4  ;; 1 cycle
        MOVW      0x294, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+6  ;; 1 cycle
        MOVW      0x296, AX          ;; 1 cycle
        MOVW      AX, N:_cal_RPh+54  ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0x28C, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0x28E, AX          ;; 1 cycle
        MOVW      AX, N:_cal_RPh+4   ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0xFFF3C, AX        ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0xFFF3E, AX        ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        MOVW      AX, 0x290          ;; 1 cycle
        MOVW      N:_mac_union_4int, AX  ;; 1 cycle
        MOVW      AX, 0x292          ;; 1 cycle
        MOVW      N:_mac_union_4int+2, AX  ;; 1 cycle
        MOVW      AX, 0x294          ;; 1 cycle
        MOVW      N:_mac_union_4int+4, AX  ;; 1 cycle
        MOVW      AX, 0x296          ;; 1 cycle
        MOVW      N:_mac_union_4int+6, AX  ;; 1 cycle
        MOVW      HL, #LWRD(_cal_RPh+70)  ;; 1 cycle
        MOVW      DE, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  483         HWMultiplierAccSigned(cal_YPh.r_shift.vol.sample,cal_YPh.curr_sample,cal_YPh.r_shift.active.acc_cnt_temp);
        MOV       0x29A, #0xC0       ;; 1 cycle
        MOVW      HL, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      DE, #LWRD(_cal_YPh+70)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        MOVW      AX, N:_mac_union_4int  ;; 1 cycle
        MOVW      0x290, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+2  ;; 1 cycle
        MOVW      0x292, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+4  ;; 1 cycle
        MOVW      0x294, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+6  ;; 1 cycle
        MOVW      0x296, AX          ;; 1 cycle
        MOVW      AX, N:_cal_YPh+54  ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0x28C, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0x28E, AX          ;; 1 cycle
        MOVW      AX, N:_cal_YPh+4   ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0xFFF3C, AX        ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0xFFF3E, AX        ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        MOVW      AX, 0x290          ;; 1 cycle
        MOVW      N:_mac_union_4int, AX  ;; 1 cycle
        MOVW      AX, 0x292          ;; 1 cycle
        MOVW      N:_mac_union_4int+2, AX  ;; 1 cycle
        MOVW      AX, 0x294          ;; 1 cycle
        MOVW      N:_mac_union_4int+4, AX  ;; 1 cycle
        MOVW      AX, 0x296          ;; 1 cycle
        MOVW      N:_mac_union_4int+6, AX  ;; 1 cycle
        MOVW      HL, #LWRD(_cal_YPh+70)  ;; 1 cycle
        MOVW      DE, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  484         HWMultiplierAccSigned(cal_BPh.r_shift.vol.sample,cal_BPh.curr_sample,cal_BPh.r_shift.active.acc_cnt_temp);
        MOV       0x29A, #0xC0       ;; 1 cycle
        MOVW      HL, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      DE, #LWRD(_cal_BPh+70)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        MOVW      AX, N:_mac_union_4int  ;; 1 cycle
        MOVW      0x290, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+2  ;; 1 cycle
        MOVW      0x292, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+4  ;; 1 cycle
        MOVW      0x294, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+6  ;; 1 cycle
        MOVW      0x296, AX          ;; 1 cycle
        MOVW      AX, N:_cal_BPh+54  ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0x28C, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0x28E, AX          ;; 1 cycle
        MOVW      AX, N:_cal_BPh+4   ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0xFFF3C, AX        ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0xFFF3E, AX        ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        MOVW      AX, 0x290          ;; 1 cycle
        MOVW      N:_mac_union_4int, AX  ;; 1 cycle
        MOVW      AX, 0x292          ;; 1 cycle
        MOVW      N:_mac_union_4int+2, AX  ;; 1 cycle
        MOVW      AX, 0x294          ;; 1 cycle
        MOVW      N:_mac_union_4int+4, AX  ;; 1 cycle
        MOVW      AX, 0x296          ;; 1 cycle
        MOVW      N:_mac_union_4int+6, AX  ;; 1 cycle
        MOVW      HL, #LWRD(_cal_BPh+70)  ;; 1 cycle
        MOVW      DE, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  485         HWMultiplierAccSigned(cal_RPh.l_shift.vol.sample90,cal_RPh.curr_sample,cal_RPh.l_shift.reactive.acc_cnt_temp);
        MOV       0x29A, #0xC0       ;; 1 cycle
        MOVW      HL, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      DE, #LWRD(_cal_RPh+46)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        MOVW      AX, N:_mac_union_4int  ;; 1 cycle
        MOVW      0x290, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+2  ;; 1 cycle
        MOVW      0x292, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+4  ;; 1 cycle
        MOVW      0x294, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+6  ;; 1 cycle
        MOVW      0x296, AX          ;; 1 cycle
        MOVW      AX, N:_cal_RPh+12  ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0x28C, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0x28E, AX          ;; 1 cycle
        MOVW      AX, N:_cal_RPh+4   ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0xFFF3C, AX        ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0xFFF3E, AX        ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        MOVW      AX, 0x290          ;; 1 cycle
        MOVW      N:_mac_union_4int, AX  ;; 1 cycle
        MOVW      AX, 0x292          ;; 1 cycle
        MOVW      N:_mac_union_4int+2, AX  ;; 1 cycle
        MOVW      AX, 0x294          ;; 1 cycle
        MOVW      N:_mac_union_4int+4, AX  ;; 1 cycle
        MOVW      AX, 0x296          ;; 1 cycle
        MOVW      N:_mac_union_4int+6, AX  ;; 1 cycle
        MOVW      HL, #LWRD(_cal_RPh+46)  ;; 1 cycle
        MOVW      DE, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  486         HWMultiplierAccSigned(cal_YPh.l_shift.vol.sample90,cal_YPh.curr_sample,cal_YPh.l_shift.reactive.acc_cnt_temp);
        MOV       0x29A, #0xC0       ;; 1 cycle
        MOVW      HL, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      DE, #LWRD(_cal_YPh+46)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        MOVW      AX, N:_mac_union_4int  ;; 1 cycle
        MOVW      0x290, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+2  ;; 1 cycle
        MOVW      0x292, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+4  ;; 1 cycle
        MOVW      0x294, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+6  ;; 1 cycle
        MOVW      0x296, AX          ;; 1 cycle
        MOVW      AX, N:_cal_YPh+12  ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0x28C, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0x28E, AX          ;; 1 cycle
        MOVW      AX, N:_cal_YPh+4   ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0xFFF3C, AX        ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0xFFF3E, AX        ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        MOVW      AX, 0x290          ;; 1 cycle
        MOVW      N:_mac_union_4int, AX  ;; 1 cycle
        MOVW      AX, 0x292          ;; 1 cycle
        MOVW      N:_mac_union_4int+2, AX  ;; 1 cycle
        MOVW      AX, 0x294          ;; 1 cycle
        MOVW      N:_mac_union_4int+4, AX  ;; 1 cycle
        MOVW      AX, 0x296          ;; 1 cycle
        MOVW      N:_mac_union_4int+6, AX  ;; 1 cycle
        MOVW      HL, #LWRD(_cal_YPh+46)  ;; 1 cycle
        MOVW      DE, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  487         HWMultiplierAccSigned(cal_BPh.l_shift.vol.sample90,cal_BPh.curr_sample,cal_BPh.l_shift.reactive.acc_cnt_temp);
        MOV       0x29A, #0xC0       ;; 1 cycle
        MOVW      HL, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      DE, #LWRD(_cal_BPh+46)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        MOVW      AX, N:_mac_union_4int  ;; 1 cycle
        MOVW      0x290, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+2  ;; 1 cycle
        MOVW      0x292, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+4  ;; 1 cycle
        MOVW      0x294, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+6  ;; 1 cycle
        MOVW      0x296, AX          ;; 1 cycle
        MOVW      AX, N:_cal_BPh+12  ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0x28C, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0x28E, AX          ;; 1 cycle
        MOVW      AX, N:_cal_BPh+4   ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0xFFF3C, AX        ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0xFFF3E, AX        ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        MOVW      AX, 0x290          ;; 1 cycle
        MOVW      N:_mac_union_4int, AX  ;; 1 cycle
        MOVW      AX, 0x292          ;; 1 cycle
        MOVW      N:_mac_union_4int+2, AX  ;; 1 cycle
        MOVW      AX, 0x294          ;; 1 cycle
        MOVW      N:_mac_union_4int+4, AX  ;; 1 cycle
        MOVW      AX, 0x296          ;; 1 cycle
        MOVW      N:_mac_union_4int+6, AX  ;; 1 cycle
        MOVW      HL, #LWRD(_cal_BPh+46)  ;; 1 cycle
        MOVW      DE, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  488         HWMultiplierAccSigned(cal_RPh.r_shift.vol.sample90,cal_RPh.curr_sample,cal_RPh.r_shift.reactive.acc_cnt_temp);
        MOV       0x29A, #0xC0       ;; 1 cycle
        MOVW      HL, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      DE, #LWRD(_cal_RPh+90)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        MOVW      AX, N:_mac_union_4int  ;; 1 cycle
        MOVW      0x290, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+2  ;; 1 cycle
        MOVW      0x292, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+4  ;; 1 cycle
        MOVW      0x294, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+6  ;; 1 cycle
        MOVW      0x296, AX          ;; 1 cycle
        MOVW      AX, N:_cal_RPh+56  ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0x28C, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0x28E, AX          ;; 1 cycle
        MOVW      AX, N:_cal_RPh+4   ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0xFFF3C, AX        ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0xFFF3E, AX        ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        MOVW      AX, 0x290          ;; 1 cycle
        MOVW      N:_mac_union_4int, AX  ;; 1 cycle
        MOVW      AX, 0x292          ;; 1 cycle
        MOVW      N:_mac_union_4int+2, AX  ;; 1 cycle
        MOVW      AX, 0x294          ;; 1 cycle
        MOVW      N:_mac_union_4int+4, AX  ;; 1 cycle
        MOVW      AX, 0x296          ;; 1 cycle
        MOVW      N:_mac_union_4int+6, AX  ;; 1 cycle
        MOVW      HL, #LWRD(_cal_RPh+90)  ;; 1 cycle
        MOVW      DE, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  489         HWMultiplierAccSigned(cal_YPh.r_shift.vol.sample90,cal_YPh.curr_sample,cal_YPh.r_shift.reactive.acc_cnt_temp);
        MOV       0x29A, #0xC0       ;; 1 cycle
        MOVW      HL, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      DE, #LWRD(_cal_YPh+90)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        MOVW      AX, N:_mac_union_4int  ;; 1 cycle
        MOVW      0x290, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+2  ;; 1 cycle
        MOVW      0x292, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+4  ;; 1 cycle
        MOVW      0x294, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+6  ;; 1 cycle
        MOVW      0x296, AX          ;; 1 cycle
        MOVW      AX, N:_cal_YPh+56  ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0x28C, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0x28E, AX          ;; 1 cycle
        MOVW      AX, N:_cal_YPh+4   ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0xFFF3C, AX        ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0xFFF3E, AX        ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        MOVW      AX, 0x290          ;; 1 cycle
        MOVW      N:_mac_union_4int, AX  ;; 1 cycle
        MOVW      AX, 0x292          ;; 1 cycle
        MOVW      N:_mac_union_4int+2, AX  ;; 1 cycle
        MOVW      AX, 0x294          ;; 1 cycle
        MOVW      N:_mac_union_4int+4, AX  ;; 1 cycle
        MOVW      AX, 0x296          ;; 1 cycle
        MOVW      N:_mac_union_4int+6, AX  ;; 1 cycle
        MOVW      HL, #LWRD(_cal_YPh+90)  ;; 1 cycle
        MOVW      DE, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
//  490         HWMultiplierAccSigned(cal_BPh.r_shift.vol.sample90,cal_BPh.curr_sample,cal_BPh.r_shift.reactive.acc_cnt_temp);
        MOV       0x29A, #0xC0       ;; 1 cycle
        MOVW      HL, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      DE, #LWRD(_cal_BPh+90)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        MOVW      AX, N:_mac_union_4int  ;; 1 cycle
        MOVW      0x290, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+2  ;; 1 cycle
        MOVW      0x292, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+4  ;; 1 cycle
        MOVW      0x294, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_4int+6  ;; 1 cycle
        MOVW      0x296, AX          ;; 1 cycle
        MOVW      AX, N:_cal_BPh+56  ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0x28C, AX          ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0x28E, AX          ;; 1 cycle
        MOVW      AX, N:_cal_BPh+4   ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_mac_union_2int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      AX, N:_mac_union_2int  ;; 1 cycle
        MOVW      0xFFF3C, AX        ;; 1 cycle
        MOVW      AX, N:_mac_union_2int+2  ;; 1 cycle
        MOVW      0xFFF3E, AX        ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        MOVW      AX, 0x290          ;; 1 cycle
        MOVW      N:_mac_union_4int, AX  ;; 1 cycle
        MOVW      AX, 0x292          ;; 1 cycle
        MOVW      N:_mac_union_4int+2, AX  ;; 1 cycle
        MOVW      AX, 0x294          ;; 1 cycle
        MOVW      N:_mac_union_4int+4, AX  ;; 1 cycle
        MOVW      AX, 0x296          ;; 1 cycle
        MOVW      N:_mac_union_4int+6, AX  ;; 1 cycle
        MOVW      HL, #LWRD(_cal_BPh+90)  ;; 1 cycle
        MOVW      DE, #LWRD(_mac_union_4int)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        ; ------------------------------------- Block: 756 cycles
//  491     }
//  492     
//  493     // pending verify the following code for various combination of fwd and reverse of CT
//  494     if(flag_zc_r_detect == 1)
??power_filter_23:
        MOVW      HL, #LWRD(_flag_zc)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??power_filter_24  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  495     {
//  496         flag_metrology_angle_start_r_y = 1;
        SET1      N:_flag_metrology.4  ;; 2 cycles
//  497         flag_metrology_angle_start_b_r = 0;
        CLR1      N:_flag_metrology.6  ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
//  498     }
//  499     if(flag_zc_y_detect == 1)
??power_filter_24:
        MOVW      HL, #LWRD(_flag_zc)  ;; 1 cycle
        MOV1      CY, [HL].1         ;; 1 cycle
        BNC       ??power_filter_25  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  500     {
//  501         flag_metrology_angle_start_r_y = 0;
        CLR1      N:_flag_metrology.4  ;; 2 cycles
//  502         flag_metrology_angle_start_y_b = 1;
        SET1      N:_flag_metrology.5  ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
//  503     }
//  504     if(flag_zc_b_detect == 1)
??power_filter_25:
        MOVW      HL, #LWRD(_flag_zc)  ;; 1 cycle
        MOV1      CY, [HL].2         ;; 1 cycle
        BNC       ??power_filter_26  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  505     {
//  506         flag_metrology_angle_start_y_b = 0;
        CLR1      N:_flag_metrology.5  ;; 2 cycles
//  507         flag_metrology_angle_start_b_r = 1;
        SET1      N:_flag_metrology.6  ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
//  508     }
//  509     
//  510     
//  511     /* Angle RY */
//  512     if(flag_phase_present_r == 1 && flag_phase_present_y == 1)
??power_filter_26:
        MOV       A, N:_flag_metro2  ;; 1 cycle
        AND       A, #0x3            ;; 1 cycle
        CMP       A, #0x3            ;; 1 cycle
        BNZ       ??power_filter_27  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  513     {
//  514         if(flag_metrology_angle_start_r_y == 1)
        MOVW      HL, #LWRD(_flag_metrology)  ;; 1 cycle
        MOV1      CY, [HL].4         ;; 1 cycle
        BNC       ??power_filter_28  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  515         {
//  516             ph_ph.Ph_RY.angle.cntr_temp++;
        INCW      N:_ph_ph+6         ;; 2 cycles
        BR        S:??power_filter_28  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  517         }
//  518     }
//  519     else
//  520     {
//  521         ph_ph.Ph_RY.angle.cntr_temp = 0;
??power_filter_27:
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_ph_ph+6, AX     ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
//  522     }
//  523     
//  524     /* Angle YB */
//  525     if(flag_phase_present_y == 1 && flag_phase_present_b == 1)
??power_filter_28:
        MOV       A, N:_flag_metro2  ;; 1 cycle
        AND       A, #0x6            ;; 1 cycle
        CMP       A, #0x6            ;; 1 cycle
        BNZ       ??power_filter_29  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  526     {
//  527         if(flag_metrology_angle_start_y_b == 1)
        MOVW      HL, #LWRD(_flag_metrology)  ;; 1 cycle
        MOV1      CY, [HL].5         ;; 1 cycle
        BNC       ??power_filter_30  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  528         {
//  529             ph_ph.Ph_YB.angle.cntr_temp++;
        INCW      N:_ph_ph+14        ;; 2 cycles
        BR        S:??power_filter_30  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  530         }
//  531     }
//  532     else
//  533     {
//  534         ph_ph.Ph_YB.angle.cntr_temp = 0;
??power_filter_29:
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_ph_ph+14, AX    ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
//  535     }
//  536     
//  537     /* Angle RB */
//  538     if(flag_phase_present_r == 1 && flag_phase_present_b == 1)
??power_filter_30:
        MOV       A, N:_flag_metro2  ;; 1 cycle
        AND       A, #0x5            ;; 1 cycle
        CMP       A, #0x5            ;; 1 cycle
        BNZ       ??power_filter_31  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  539     {
//  540         if(flag_metrology_angle_start_b_r == 1)
        MOVW      HL, #LWRD(_flag_metrology)  ;; 1 cycle
        MOV1      CY, [HL].6         ;; 1 cycle
        BNC       ??power_filter_32  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  541         {
//  542             ph_ph.Ph_BR.angle.cntr_temp++;
        INCW      N:_ph_ph+22        ;; 2 cycles
        BR        S:??power_filter_32  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  543         }
//  544     }
//  545     else
//  546     {
//  547         ph_ph.Ph_BR.angle.cntr_temp = 0;
??power_filter_31:
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_ph_ph+22, AX    ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
//  548     }
//  549     
//  550     two_ms_loop_cntr++;
??power_filter_32:
        INC       N:`metrology_function::two_ms_loop_cntr`  ;; 2 cycles
//  551     if(two_ms_loop_cntr >= 8)                   /* 2.048 ms loop */
        MOV       A, N:`metrology_function::two_ms_loop_cntr`  ;; 1 cycle
        CMP       A, #0x8            ;; 1 cycle
        SKNC                         ;; 4 cycles
        BR        N:??power_filter_33  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
//  552     {
//  553         two_ms_loop_cntr = 0;
        MOV       N:`metrology_function::two_ms_loop_cntr`, #0x0  ;; 1 cycle
//  554         
//  555         /* Pulse out calculation */
//  556         dispense_ecal++;
        INC       N:`metrology_function::dispense_ecal`  ;; 2 cycles
//  557         if(dispense_ecal >= (LED_PULSE_TIME))
        MOV       A, N:`metrology_function::dispense_ecal`  ;; 1 cycle
        CMP       A, #0xE            ;; 1 cycle
        BC        ??power_filter_34  ;; 4 cycles
        ; ------------------------------------- Block: 9 cycles
//  558         {
//  559             LED_ECAL_LOW;
        CLR1      S:0xFFF04.3        ;; 2 cycles
//  560             dispense_ecal = 0;
        MOV       N:`metrology_function::dispense_ecal`, #0x0  ;; 1 cycle
        ; ------------------------------------- Block: 3 cycles
//  561         }
//  562         dispense_rcal++;
??power_filter_34:
        INC       N:`metrology_function::dispense_rcal`  ;; 2 cycles
//  563         if(dispense_rcal >= (LED_PULSE_TIME))
        MOV       A, N:`metrology_function::dispense_rcal`  ;; 1 cycle
        CMP       A, #0xE            ;; 1 cycle
        BC        ??power_filter_35  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
//  564         {
//  565             LED_RCAL_LOW;
        CLR1      S:0xFFF05.0        ;; 2 cycles
//  566             dispense_rcal = 0;
        MOV       N:`metrology_function::dispense_rcal`, #0x0  ;; 1 cycle
        ; ------------------------------------- Block: 3 cycles
//  567         }
//  568         
//  569         /****************************************************************************************
//  570         ****************************  Active Energy Calculation  ********************************
//  571         ****************************************************************************************/
//  572         r_phase.active.acc_delta_cnts += r_phase.active.delta_cnts;
??power_filter_35:
        MOVW      DE, #LWRD(_r_phase+96)  ;; 1 cycle
        MOVW      BC, #LWRD(_r_phase+88)  ;; 1 cycle
        MOVW      AX, #LWRD(_r_phase+88)  ;; 1 cycle
          CFI FunCall __Add64
        CALL      __Add64            ;; 3 cycles
//  573         if(r_phase.active.acc_delta_cnts >= ACT_THRESHOLD)
        MOVW      BC, #LWRD(_ACT_THRESHOLD)  ;; 1 cycle
        MOVW      AX, #LWRD(_r_phase+88)  ;; 1 cycle
          CFI FunCall __CmpLtu64
        CALL      __CmpLtu64         ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_36  ;; 4 cycles
        ; ------------------------------------- Block: 16 cycles
//  574         {
//  575             r_phase.active.acc_delta_cnts -= ACT_THRESHOLD;
        MOVW      DE, #LWRD(_ACT_THRESHOLD)  ;; 1 cycle
        MOVW      BC, #LWRD(_r_phase+88)  ;; 1 cycle
        MOVW      AX, #LWRD(_r_phase+88)  ;; 1 cycle
          CFI FunCall __Sub64
        CALL      __Sub64            ;; 3 cycles
//  576             if(METERING_MODE == NET)
        CMP       N:_METERING_MODE, #0x1  ;; 1 cycle
        BNZ       ??power_filter_37  ;; 4 cycles
        ; ------------------------------------- Block: 11 cycles
//  577             {
//  578                 active_calculation_net(PHASE_R,quadrant.Rph); 
        MOV       X, N:_quadrant     ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _active_calculation_net
        CALL      _active_calculation_net  ;; 3 cycles
        BR        S:??power_filter_36  ;; 3 cycles
        ; ------------------------------------- Block: 8 cycles
//  579             }
//  580             else 
//  581             {
//  582                 active_calculation_fwd(PHASE_R,quadrant.Rph);
??power_filter_37:
        MOV       X, N:_quadrant     ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _active_calculation_fwd
        CALL      _active_calculation_fwd  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  583             }
//  584         }
//  585         
//  586         y_phase.active.acc_delta_cnts += y_phase.active.delta_cnts;
??power_filter_36:
        MOVW      DE, #LWRD(_y_phase+96)  ;; 1 cycle
        MOVW      BC, #LWRD(_y_phase+88)  ;; 1 cycle
        MOVW      AX, #LWRD(_y_phase+88)  ;; 1 cycle
          CFI FunCall __Add64
        CALL      __Add64            ;; 3 cycles
//  587         if(y_phase.active.acc_delta_cnts >= ACT_THRESHOLD)
        MOVW      BC, #LWRD(_ACT_THRESHOLD)  ;; 1 cycle
        MOVW      AX, #LWRD(_y_phase+88)  ;; 1 cycle
          CFI FunCall __CmpLtu64
        CALL      __CmpLtu64         ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_38  ;; 4 cycles
        ; ------------------------------------- Block: 16 cycles
//  588         {
//  589             y_phase.active.acc_delta_cnts -= ACT_THRESHOLD;
        MOVW      DE, #LWRD(_ACT_THRESHOLD)  ;; 1 cycle
        MOVW      BC, #LWRD(_y_phase+88)  ;; 1 cycle
        MOVW      AX, #LWRD(_y_phase+88)  ;; 1 cycle
          CFI FunCall __Sub64
        CALL      __Sub64            ;; 3 cycles
//  590             if(METERING_MODE == NET)
        CMP       N:_METERING_MODE, #0x1  ;; 1 cycle
        BNZ       ??power_filter_39  ;; 4 cycles
        ; ------------------------------------- Block: 11 cycles
//  591             {
//  592                 active_calculation_net(PHASE_Y,quadrant.Yph); 
        MOV       X, N:_quadrant+1   ;; 1 cycle
        MOV       A, #0x1            ;; 1 cycle
          CFI FunCall _active_calculation_net
        CALL      _active_calculation_net  ;; 3 cycles
        BR        S:??power_filter_38  ;; 3 cycles
        ; ------------------------------------- Block: 8 cycles
//  593             }
//  594             else 
//  595             {
//  596                 active_calculation_fwd(PHASE_Y,quadrant.Yph);
??power_filter_39:
        MOV       X, N:_quadrant+1   ;; 1 cycle
        MOV       A, #0x1            ;; 1 cycle
          CFI FunCall _active_calculation_fwd
        CALL      _active_calculation_fwd  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  597             }
//  598         }
//  599         
//  600         b_phase.active.acc_delta_cnts += b_phase.active.delta_cnts;
??power_filter_38:
        MOVW      DE, #LWRD(_b_phase+96)  ;; 1 cycle
        MOVW      BC, #LWRD(_b_phase+88)  ;; 1 cycle
        MOVW      AX, #LWRD(_b_phase+88)  ;; 1 cycle
          CFI FunCall __Add64
        CALL      __Add64            ;; 3 cycles
//  601         if(b_phase.active.acc_delta_cnts >= ACT_THRESHOLD)
        MOVW      BC, #LWRD(_ACT_THRESHOLD)  ;; 1 cycle
        MOVW      AX, #LWRD(_b_phase+88)  ;; 1 cycle
          CFI FunCall __CmpLtu64
        CALL      __CmpLtu64         ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_40  ;; 4 cycles
        ; ------------------------------------- Block: 16 cycles
//  602         {
//  603             b_phase.active.acc_delta_cnts -= ACT_THRESHOLD;
        MOVW      DE, #LWRD(_ACT_THRESHOLD)  ;; 1 cycle
        MOVW      BC, #LWRD(_b_phase+88)  ;; 1 cycle
        MOVW      AX, #LWRD(_b_phase+88)  ;; 1 cycle
          CFI FunCall __Sub64
        CALL      __Sub64            ;; 3 cycles
//  604             if(METERING_MODE == NET)
        CMP       N:_METERING_MODE, #0x1  ;; 1 cycle
        BNZ       ??power_filter_41  ;; 4 cycles
        ; ------------------------------------- Block: 11 cycles
//  605             {
//  606                 active_calculation_net(PHASE_B,quadrant.Bph); 
        MOV       X, N:_quadrant+2   ;; 1 cycle
        MOV       A, #0x2            ;; 1 cycle
          CFI FunCall _active_calculation_net
        CALL      _active_calculation_net  ;; 3 cycles
        BR        S:??power_filter_40  ;; 3 cycles
        ; ------------------------------------- Block: 8 cycles
//  607             }
//  608             else 
//  609             {
//  610                 active_calculation_fwd(PHASE_B,quadrant.Bph);
??power_filter_41:
        MOV       X, N:_quadrant+2   ;; 1 cycle
        MOV       A, #0x2            ;; 1 cycle
          CFI FunCall _active_calculation_fwd
        CALL      _active_calculation_fwd  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  611             }
//  612         }
//  613         
//  614         all_phase.active.acc_delta_cnts += all_phase.active.delta_cnts;
??power_filter_40:
        MOVW      DE, #LWRD(_all_phase+24)  ;; 1 cycle
        MOVW      BC, #LWRD(_all_phase+16)  ;; 1 cycle
        MOVW      AX, #LWRD(_all_phase+16)  ;; 1 cycle
          CFI FunCall __Add64
        CALL      __Add64            ;; 3 cycles
//  615         if(all_phase.active.acc_delta_cnts >= ACT_THRESHOLD)
        MOVW      BC, #LWRD(_ACT_THRESHOLD)  ;; 1 cycle
        MOVW      AX, #LWRD(_all_phase+16)  ;; 1 cycle
          CFI FunCall __CmpLtu64
        CALL      __CmpLtu64         ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_42  ;; 4 cycles
        ; ------------------------------------- Block: 16 cycles
//  616         {
//  617             all_phase.active.acc_delta_cnts -= ACT_THRESHOLD;
        MOVW      DE, #LWRD(_ACT_THRESHOLD)  ;; 1 cycle
        MOVW      BC, #LWRD(_all_phase+16)  ;; 1 cycle
        MOVW      AX, #LWRD(_all_phase+16)  ;; 1 cycle
          CFI FunCall __Sub64
        CALL      __Sub64            ;; 3 cycles
//  618             LED_ECAL_HIGH;
        SET1      S:0xFFF04.3        ;; 2 cycles
//  619             dispense_ecal=0;
        MOV       N:`metrology_function::dispense_ecal`, #0x0  ;; 1 cycle
//  620             if(METERING_MODE == FWD)
        CMP0      N:_METERING_MODE   ;; 1 cycle
        BNZ       ??power_filter_42  ;; 4 cycles
        ; ------------------------------------- Block: 14 cycles
//  621             {
//  622                 active_calculation_fwd(PHASE_ALL,quadrant.Allph);
        MOV       X, N:_quadrant+3   ;; 1 cycle
        MOV       A, #0x3            ;; 1 cycle
          CFI FunCall _active_calculation_fwd
        CALL      _active_calculation_fwd  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  623             }
//  624         }
//  625         
//  626         all_phase.fundamental.active.acc_delta_cnts += all_phase.fundamental.active.delta_cnts;
??power_filter_42:
        MOVW      DE, #LWRD(_all_phase+128)  ;; 1 cycle
        MOVW      BC, #LWRD(_all_phase+120)  ;; 1 cycle
        MOVW      AX, #LWRD(_all_phase+120)  ;; 1 cycle
          CFI FunCall __Add64
        CALL      __Add64            ;; 3 cycles
//  627         if(all_phase.fundamental.active.acc_delta_cnts >= ACT_THRESHOLD)
        MOVW      BC, #LWRD(_ACT_THRESHOLD)  ;; 1 cycle
        MOVW      AX, #LWRD(_all_phase+120)  ;; 1 cycle
          CFI FunCall __CmpLtu64
        CALL      __CmpLtu64         ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_43  ;; 4 cycles
        ; ------------------------------------- Block: 16 cycles
//  628         {
//  629             all_phase.fundamental.active.acc_delta_cnts -= ACT_THRESHOLD;
        MOVW      DE, #LWRD(_ACT_THRESHOLD)  ;; 1 cycle
        MOVW      BC, #LWRD(_all_phase+120)  ;; 1 cycle
        MOVW      AX, #LWRD(_all_phase+120)  ;; 1 cycle
          CFI FunCall __Sub64
        CALL      __Sub64            ;; 3 cycles
//  630             if(METERING_MODE == FWD)
        CMP0      N:_METERING_MODE   ;; 1 cycle
        BNZ       ??power_filter_43  ;; 4 cycles
        ; ------------------------------------- Block: 11 cycles
//  631             {
//  632                 if(thd.Rph.correction != 0 || thd.Yph.correction != 0 || thd.Bph.correction != 0)
        CMP0      N:_thd+16          ;; 1 cycle
        BNZ       ??power_filter_44  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        CMP0      N:_thd+34          ;; 1 cycle
        BNZ       ??power_filter_44  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        CMP0      N:_thd+52          ;; 1 cycle
        BZ        ??power_filter_43  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  633                 {
//  634                     energy.Allph.fundamental_pulse++;
??power_filter_44:
        INC       N:_energy+44       ;; 2 cycles
//  635                     if(energy.Allph.fundamental_pulse >= PULSE)
        MOV       A, N:_energy+44    ;; 1 cycle
        CMP       A, #0x6            ;; 1 cycle
        BC        ??power_filter_43  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
//  636                     {
//  637                         energy.Allph.fundamental_pulse = 0;
        MOV       N:_energy+44, #0x0  ;; 1 cycle
//  638                         energy.Allph.fundamental += QUANTA;
        MOVW      BC, N:_energy+66   ;; 1 cycle
        MOVW      AX, N:_energy+64   ;; 1 cycle
        ADDW      AX, #0x5           ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+64, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+66, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ; ------------------------------------- Block: 16 cycles
//  639                     }
//  640                 }
//  641             }
//  642         }
//  643         
//  644         
//  645         /****************************************************************************************
//  646         ****************************  Reactive Energy Calculation  ******************************
//  647         ****************************************************************************************/
//  648         if(METERING_MODE == NET)
??power_filter_43:
        CMP       N:_METERING_MODE, #0x1  ;; 1 cycle
        SKZ                          ;; 4 cycles
        BR        N:??power_filter_45  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  649         {
//  650             r_phase.reactive.acc_delta_cnts += r_phase.reactive.delta_cnts;
        MOVW      DE, #LWRD(_r_phase+136)  ;; 1 cycle
        MOVW      BC, #LWRD(_r_phase+128)  ;; 1 cycle
        MOVW      AX, #LWRD(_r_phase+128)  ;; 1 cycle
          CFI FunCall __Add64
        CALL      __Add64            ;; 3 cycles
//  651             if(r_phase.reactive.acc_delta_cnts >= REACT_THRESHOLD)
        MOVW      BC, #LWRD(_REACT_THRESHOLD)  ;; 1 cycle
        MOVW      AX, #LWRD(_r_phase+128)  ;; 1 cycle
          CFI FunCall __CmpLtu64
        CALL      __CmpLtu64         ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_46  ;; 4 cycles
        ; ------------------------------------- Block: 16 cycles
//  652             {
//  653                 r_phase.reactive.acc_delta_cnts -= REACT_THRESHOLD;
        MOVW      DE, #LWRD(_REACT_THRESHOLD)  ;; 1 cycle
        MOVW      BC, #LWRD(_r_phase+128)  ;; 1 cycle
        MOVW      AX, #LWRD(_r_phase+128)  ;; 1 cycle
          CFI FunCall __Sub64
        CALL      __Sub64            ;; 3 cycles
//  654                 reactive_calculation_net(PHASE_R,quadrant.Rph); 
        MOV       X, N:_quadrant     ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _reactive_calculation_net
        CALL      _reactive_calculation_net  ;; 3 cycles
        ; ------------------------------------- Block: 11 cycles
//  655             }
//  656             
//  657             y_phase.reactive.acc_delta_cnts += y_phase.reactive.delta_cnts;
??power_filter_46:
        MOVW      DE, #LWRD(_y_phase+136)  ;; 1 cycle
        MOVW      BC, #LWRD(_y_phase+128)  ;; 1 cycle
        MOVW      AX, #LWRD(_y_phase+128)  ;; 1 cycle
          CFI FunCall __Add64
        CALL      __Add64            ;; 3 cycles
//  658             if(y_phase.reactive.acc_delta_cnts >= REACT_THRESHOLD)
        MOVW      BC, #LWRD(_REACT_THRESHOLD)  ;; 1 cycle
        MOVW      AX, #LWRD(_y_phase+128)  ;; 1 cycle
          CFI FunCall __CmpLtu64
        CALL      __CmpLtu64         ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_47  ;; 4 cycles
        ; ------------------------------------- Block: 16 cycles
//  659             {
//  660                 y_phase.reactive.acc_delta_cnts -= REACT_THRESHOLD;
        MOVW      DE, #LWRD(_REACT_THRESHOLD)  ;; 1 cycle
        MOVW      BC, #LWRD(_y_phase+128)  ;; 1 cycle
        MOVW      AX, #LWRD(_y_phase+128)  ;; 1 cycle
          CFI FunCall __Sub64
        CALL      __Sub64            ;; 3 cycles
//  661                 reactive_calculation_net(PHASE_Y,quadrant.Yph); 
        MOV       X, N:_quadrant+1   ;; 1 cycle
        MOV       A, #0x1            ;; 1 cycle
          CFI FunCall _reactive_calculation_net
        CALL      _reactive_calculation_net  ;; 3 cycles
        ; ------------------------------------- Block: 11 cycles
//  662             }
//  663             
//  664             b_phase.reactive.acc_delta_cnts += b_phase.reactive.delta_cnts;
??power_filter_47:
        MOVW      DE, #LWRD(_b_phase+136)  ;; 1 cycle
        MOVW      BC, #LWRD(_b_phase+128)  ;; 1 cycle
        MOVW      AX, #LWRD(_b_phase+128)  ;; 1 cycle
          CFI FunCall __Add64
        CALL      __Add64            ;; 3 cycles
//  665             if(b_phase.reactive.acc_delta_cnts >= REACT_THRESHOLD)
        MOVW      BC, #LWRD(_REACT_THRESHOLD)  ;; 1 cycle
        MOVW      AX, #LWRD(_b_phase+128)  ;; 1 cycle
          CFI FunCall __CmpLtu64
        CALL      __CmpLtu64         ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_45  ;; 4 cycles
        ; ------------------------------------- Block: 16 cycles
//  666             {
//  667                 b_phase.reactive.acc_delta_cnts -= REACT_THRESHOLD;
        MOVW      DE, #LWRD(_REACT_THRESHOLD)  ;; 1 cycle
        MOVW      BC, #LWRD(_b_phase+128)  ;; 1 cycle
        MOVW      AX, #LWRD(_b_phase+128)  ;; 1 cycle
          CFI FunCall __Sub64
        CALL      __Sub64            ;; 3 cycles
//  668                 reactive_calculation_net(PHASE_B,quadrant.Bph); 
        MOV       X, N:_quadrant+2   ;; 1 cycle
        MOV       A, #0x2            ;; 1 cycle
          CFI FunCall _reactive_calculation_net
        CALL      _reactive_calculation_net  ;; 3 cycles
        ; ------------------------------------- Block: 11 cycles
//  669             }
//  670         }
//  671         all_phase.reactive.acc_delta_cnts += all_phase.reactive.delta_cnts_net;
??power_filter_45:
        MOVW      DE, #LWRD(_all_phase+72)  ;; 1 cycle
        MOVW      BC, #LWRD(_all_phase+56)  ;; 1 cycle
        MOVW      AX, #LWRD(_all_phase+56)  ;; 1 cycle
          CFI FunCall __Add64
        CALL      __Add64            ;; 3 cycles
//  672         if(all_phase.reactive.acc_delta_cnts >= REACT_THRESHOLD)
        MOVW      BC, #LWRD(_REACT_THRESHOLD)  ;; 1 cycle
        MOVW      AX, #LWRD(_all_phase+56)  ;; 1 cycle
          CFI FunCall __CmpLtu64
        CALL      __CmpLtu64         ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_48  ;; 4 cycles
        ; ------------------------------------- Block: 16 cycles
//  673         {
//  674             all_phase.reactive.acc_delta_cnts -= REACT_THRESHOLD;
        MOVW      DE, #LWRD(_REACT_THRESHOLD)  ;; 1 cycle
        MOVW      BC, #LWRD(_all_phase+56)  ;; 1 cycle
        MOVW      AX, #LWRD(_all_phase+56)  ;; 1 cycle
          CFI FunCall __Sub64
        CALL      __Sub64            ;; 3 cycles
//  675             if(METERING_MODE == FWD)
        CMP0      N:_METERING_MODE   ;; 1 cycle
        BNZ       ??power_filter_49  ;; 4 cycles
        ; ------------------------------------- Block: 11 cycles
//  676             {
//  677                 reactive_calculation_fwd(PHASE_ALL,quadrant.Allph); 
        MOV       X, N:_quadrant+3   ;; 1 cycle
        MOV       A, #0x3            ;; 1 cycle
          CFI FunCall _reactive_calculation_fwd
        CALL      _reactive_calculation_fwd  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  678             }
//  679             LED_RCAL_HIGH;
??power_filter_49:
        SET1      S:0xFFF05.0        ;; 2 cycles
//  680             dispense_rcal=0;
        MOV       N:`metrology_function::dispense_rcal`, #0x0  ;; 1 cycle
        ; ------------------------------------- Block: 3 cycles
//  681         }
//  682         
//  683         
//  684         /****************************************************************************************
//  685         ****************************  Apparent Energy Calculation  ******************************
//  686         ****************************************************************************************/
//  687         if(METERING_MODE == NET)
??power_filter_48:
        CMP       N:_METERING_MODE, #0x1  ;; 1 cycle
        SKZ                          ;; 4 cycles
        BR        N:??power_filter_50  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  688         {
//  689             r_phase.apparent.acc_delta_cnts += r_phase.apparent.delta_cnts;
        MOVW      DE, #LWRD(_r_phase+176)  ;; 1 cycle
        MOVW      BC, #LWRD(_r_phase+168)  ;; 1 cycle
        MOVW      AX, #LWRD(_r_phase+168)  ;; 1 cycle
          CFI FunCall __Add64
        CALL      __Add64            ;; 3 cycles
//  690             if(r_phase.apparent.acc_delta_cnts >= APP_THRESHOLD)
        MOVW      BC, #LWRD(_APP_THRESHOLD)  ;; 1 cycle
        MOVW      AX, #LWRD(_r_phase+168)  ;; 1 cycle
          CFI FunCall __CmpLtu64
        CALL      __CmpLtu64         ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_51  ;; 4 cycles
        ; ------------------------------------- Block: 16 cycles
//  691             {
//  692                 r_phase.apparent.acc_delta_cnts -= APP_THRESHOLD;
        MOVW      DE, #LWRD(_APP_THRESHOLD)  ;; 1 cycle
        MOVW      BC, #LWRD(_r_phase+168)  ;; 1 cycle
        MOVW      AX, #LWRD(_r_phase+168)  ;; 1 cycle
          CFI FunCall __Sub64
        CALL      __Sub64            ;; 3 cycles
//  693                 apparent_calculation_net(PHASE_R,quadrant.Rph); 
        MOV       X, N:_quadrant     ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _apparent_calculation_net
        CALL      _apparent_calculation_net  ;; 3 cycles
        ; ------------------------------------- Block: 11 cycles
//  694             }
//  695             
//  696             y_phase.apparent.acc_delta_cnts += y_phase.apparent.delta_cnts;
??power_filter_51:
        MOVW      DE, #LWRD(_y_phase+176)  ;; 1 cycle
        MOVW      BC, #LWRD(_y_phase+168)  ;; 1 cycle
        MOVW      AX, #LWRD(_y_phase+168)  ;; 1 cycle
          CFI FunCall __Add64
        CALL      __Add64            ;; 3 cycles
//  697             if(y_phase.apparent.acc_delta_cnts >= APP_THRESHOLD)
        MOVW      BC, #LWRD(_APP_THRESHOLD)  ;; 1 cycle
        MOVW      AX, #LWRD(_y_phase+168)  ;; 1 cycle
          CFI FunCall __CmpLtu64
        CALL      __CmpLtu64         ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_52  ;; 4 cycles
        ; ------------------------------------- Block: 16 cycles
//  698             {
//  699                 y_phase.apparent.acc_delta_cnts -= APP_THRESHOLD;
        MOVW      DE, #LWRD(_APP_THRESHOLD)  ;; 1 cycle
        MOVW      BC, #LWRD(_y_phase+168)  ;; 1 cycle
        MOVW      AX, #LWRD(_y_phase+168)  ;; 1 cycle
          CFI FunCall __Sub64
        CALL      __Sub64            ;; 3 cycles
//  700                 apparent_calculation_net(PHASE_Y,quadrant.Yph); 
        MOV       X, N:_quadrant+1   ;; 1 cycle
        MOV       A, #0x1            ;; 1 cycle
          CFI FunCall _apparent_calculation_net
        CALL      _apparent_calculation_net  ;; 3 cycles
        ; ------------------------------------- Block: 11 cycles
//  701             }
//  702             
//  703             b_phase.apparent.acc_delta_cnts += b_phase.apparent.delta_cnts;
??power_filter_52:
        MOVW      DE, #LWRD(_b_phase+176)  ;; 1 cycle
        MOVW      BC, #LWRD(_b_phase+168)  ;; 1 cycle
        MOVW      AX, #LWRD(_b_phase+168)  ;; 1 cycle
          CFI FunCall __Add64
        CALL      __Add64            ;; 3 cycles
//  704             if(b_phase.apparent.acc_delta_cnts >= APP_THRESHOLD)
        MOVW      BC, #LWRD(_APP_THRESHOLD)  ;; 1 cycle
        MOVW      AX, #LWRD(_b_phase+168)  ;; 1 cycle
          CFI FunCall __CmpLtu64
        CALL      __CmpLtu64         ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_50  ;; 4 cycles
        ; ------------------------------------- Block: 16 cycles
//  705             {
//  706                 b_phase.apparent.acc_delta_cnts -= APP_THRESHOLD;
        MOVW      DE, #LWRD(_APP_THRESHOLD)  ;; 1 cycle
        MOVW      BC, #LWRD(_b_phase+168)  ;; 1 cycle
        MOVW      AX, #LWRD(_b_phase+168)  ;; 1 cycle
          CFI FunCall __Sub64
        CALL      __Sub64            ;; 3 cycles
//  707                 apparent_calculation_net(PHASE_B,quadrant.Bph); 
        MOV       X, N:_quadrant+2   ;; 1 cycle
        MOV       A, #0x2            ;; 1 cycle
          CFI FunCall _apparent_calculation_net
        CALL      _apparent_calculation_net  ;; 3 cycles
        ; ------------------------------------- Block: 11 cycles
//  708             }
//  709         }
//  710         
//  711         all_phase.apparent.acc_delta_cnts += all_phase.apparent.delta_cnts_net;
??power_filter_50:
        MOVW      DE, #LWRD(_all_phase+112)  ;; 1 cycle
        MOVW      BC, #LWRD(_all_phase+96)  ;; 1 cycle
        MOVW      AX, #LWRD(_all_phase+96)  ;; 1 cycle
          CFI FunCall __Add64
        CALL      __Add64            ;; 3 cycles
//  712         if(all_phase.apparent.acc_delta_cnts >= APP_THRESHOLD)
        MOVW      BC, #LWRD(_APP_THRESHOLD)  ;; 1 cycle
        MOVW      AX, #LWRD(_all_phase+96)  ;; 1 cycle
          CFI FunCall __CmpLtu64
        CALL      __CmpLtu64         ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_33  ;; 4 cycles
        ; ------------------------------------- Block: 16 cycles
//  713         {
//  714             all_phase.apparent.acc_delta_cnts -= APP_THRESHOLD;
        MOVW      DE, #LWRD(_APP_THRESHOLD)  ;; 1 cycle
        MOVW      BC, #LWRD(_all_phase+96)  ;; 1 cycle
        MOVW      AX, #LWRD(_all_phase+96)  ;; 1 cycle
          CFI FunCall __Sub64
        CALL      __Sub64            ;; 3 cycles
//  715             if(METERING_MODE == FWD)
        CMP0      N:_METERING_MODE   ;; 1 cycle
        BNZ       ??power_filter_33  ;; 4 cycles
        ; ------------------------------------- Block: 11 cycles
//  716             {
//  717                 apparent_calculation_fwd(PHASE_ALL,quadrant.Allph); 
        MOV       X, N:_quadrant+3   ;; 1 cycle
        MOV       A, #0x3            ;; 1 cycle
          CFI FunCall _apparent_calculation_fwd
        CALL      _apparent_calculation_fwd  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  718             }
//  719             //      LED_RCAL_HIGH;
//  720             //      dispense_rcal=0;
//  721         }
//  722     }
//  723     
//  724     /* Pop MACRx registers */
//  725     MACRH = bkup_macrh;
??power_filter_33:
        MOVW      AX, N:`metrology_function::bkup_macrh`  ;; 1 cycle
        MOVW      0xFFFF2, AX        ;; 1 cycle
//  726     MACRL = bkup_macrl;
        MOVW      AX, N:`metrology_function::bkup_macrl`  ;; 1 cycle
        MOVW      0xFFFF0, AX        ;; 1 cycle
//  727 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock3
        ; ------------------------------------- Block: 10 cycles
        ; ------------------------------------- Total: 3189 cycles
        REQUIRE __A_MACRH
        REQUIRE __A_MACRL
        REQUIRE __A_MULC
        REQUIRE __A_MULR0
        REQUIRE __A_MULR1
        REQUIRE __A_MULR2
        REQUIRE __A_MULR3
        REQUIRE __A_MAC32SL
        REQUIRE __A_MAC32SH
        REQUIRE __A_MULBL
        REQUIRE __A_MULBH
        REQUIRE __A_P4
        REQUIRE __A_P5

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
`metrology_function::dispense_ecal`:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
`metrology_function::dispense_rcal`:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
`metrology_function::two_ms_loop_cntr`:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
`metrology_function::bkup_macrh`:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
`metrology_function::bkup_macrl`:
        DS 2

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock4 Using cfiCommon0
          CFI Function _active_calculation_net
          CFI NoCalls
        CODE
//  728 void active_calculation_net(us8 phase,us8 quad)
//  729 {
_active_calculation_net:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
        MOV       B, A               ;; 1 cycle
//  730     flag_metro_save_energy = 1;
        SET1      N:_flag_metro1.4   ;; 2 cycles
//  731     if(phase == PHASE_R)
        CMP0      B                  ;; 1 cycle
        BNZ       ??power_filter_53  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
//  732     {
//  733         if(quad== Q1 || quad == Q4)
        XCH       A, X               ;; 1 cycle
        CMP       A, #0x1            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        BZ        ??power_filter_54  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        XCH       A, X               ;; 1 cycle
        CMP       A, #0x4            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        BNZ       ??power_filter_55  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  734         {
//  735             energy.Rph.active_imp_pulse++;
??power_filter_54:
        INC       N:_energy          ;; 2 cycles
//  736             energy.Allph.active_imp_pulse++;
        INC       N:_energy+42       ;; 2 cycles
//  737             if(energy.Rph.active_imp_pulse >= PULSE)
        MOV       A, N:_energy       ;; 1 cycle
        CMP       A, #0x6            ;; 1 cycle
        SKNC                         ;; 4 cycles
        BR        N:??power_filter_56  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
//  738             {
//  739                 energy.Rph.active_imp_pulse = 0;
        MOV       N:_energy, #0x0    ;; 1 cycle
//  740                 energy.Rph.active_imp += QUANTA;
        MOVW      HL, N:_energy+4    ;; 1 cycle
        MOVW      DE, N:_energy+2    ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        ADDW      AX, #0x5           ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        SKNC
        INCW      HL                 ;; 5 cycles
        XCHW      AX, HL             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        MOVW      N:_energy+2, AX    ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      N:_energy+4, AX    ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
//  741                 if(bitIsSet(tpr.magnet.flag,event_f) && (MAG_SWITCH_IMAX == 1))
        MOVW      HL, #LWRD(_tpr+32)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        SKC                          ;; 4 cycles
        BR        N:??power_filter_56  ;; 4 cycles
        ; ------------------------------------- Block: 26 cycles
//  742                 {
//  743                   energy.Rph.defraud_mag += QUANTA;
        MOVW      HL, N:_energy+12   ;; 1 cycle
        MOVW      DE, N:_energy+10   ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        ADDW      AX, #0x5           ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        SKNC
        INCW      HL                 ;; 5 cycles
        XCHW      AX, HL             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        MOVW      N:_energy+10, AX   ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      N:_energy+12, AX   ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        BR        N:??power_filter_56  ;; 3 cycles
        ; ------------------------------------- Block: 22 cycles
//  744                 }
//  745             }
//  746         }
//  747         else
//  748         {
//  749             energy.Rph.active_exp_pulse++;
??power_filter_55:
        INC       N:_energy+1        ;; 2 cycles
//  750             energy.Allph.active_exp_pulse++;
        INC       N:_energy+43       ;; 2 cycles
//  751             if(energy.Rph.active_exp_pulse >= PULSE)
        MOV       A, N:_energy+1     ;; 1 cycle
        CMP       A, #0x6            ;; 1 cycle
        SKNC                         ;; 4 cycles
        BR        N:??power_filter_56  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
//  752             {
//  753                 energy.Rph.active_exp_pulse = 0;
        MOV       N:_energy+1, #0x0  ;; 1 cycle
        BR        N:??power_filter_56  ;; 3 cycles
        ; ------------------------------------- Block: 4 cycles
//  754             }
//  755         }
//  756     }
//  757     else if(phase == PHASE_Y)
??power_filter_53:
        XCH       A, B               ;; 1 cycle
        CMP       A, #0x1            ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        BNZ       ??power_filter_57  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  758     {
//  759         if(quad== Q1 || quad == Q4)
        XCH       A, X               ;; 1 cycle
        CMP       A, #0x1            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        BZ        ??power_filter_58  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        XCH       A, X               ;; 1 cycle
        CMP       A, #0x4            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        BNZ       ??power_filter_59  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  760         {
//  761             energy.Yph.active_imp_pulse++;
??power_filter_58:
        INC       N:_energy+14       ;; 2 cycles
//  762             energy.Allph.active_imp_pulse++;
        INC       N:_energy+42       ;; 2 cycles
//  763             if(energy.Yph.active_imp_pulse >= PULSE)
        MOV       A, N:_energy+14    ;; 1 cycle
        CMP       A, #0x6            ;; 1 cycle
        SKNC                         ;; 4 cycles
        BR        N:??power_filter_56  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
//  764             {
//  765                 energy.Yph.active_imp_pulse = 0;
        MOV       N:_energy+14, #0x0  ;; 1 cycle
//  766                 energy.Yph.active_imp += QUANTA;
        MOVW      HL, N:_energy+18   ;; 1 cycle
        MOVW      DE, N:_energy+16   ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        ADDW      AX, #0x5           ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        SKNC
        INCW      HL                 ;; 5 cycles
        XCHW      AX, HL             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        MOVW      N:_energy+16, AX   ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      N:_energy+18, AX   ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
//  767                 if(bitIsSet(tpr.magnet.flag,event_f) && (MAG_SWITCH_IMAX == 1))
        MOVW      HL, #LWRD(_tpr+32)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        SKC                          ;; 4 cycles
        BR        N:??power_filter_56  ;; 4 cycles
        ; ------------------------------------- Block: 26 cycles
//  768                 {
//  769                   energy.Yph.defraud_mag += QUANTA;
        MOVW      HL, N:_energy+26   ;; 1 cycle
        MOVW      DE, N:_energy+24   ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        ADDW      AX, #0x5           ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        SKNC
        INCW      HL                 ;; 5 cycles
        XCHW      AX, HL             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        MOVW      N:_energy+24, AX   ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      N:_energy+26, AX   ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        BR        N:??power_filter_56  ;; 3 cycles
        ; ------------------------------------- Block: 22 cycles
//  770                 }
//  771             }
//  772         }
//  773         else
//  774         {
//  775             energy.Yph.active_exp_pulse++;
??power_filter_59:
        INC       N:_energy+15       ;; 2 cycles
//  776             energy.Allph.active_exp_pulse++;
        INC       N:_energy+43       ;; 2 cycles
//  777             if(energy.Yph.active_exp_pulse >= PULSE)
        MOV       A, N:_energy+15    ;; 1 cycle
        CMP       A, #0x6            ;; 1 cycle
        BC        ??power_filter_56  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
//  778             {
//  779                 energy.Yph.active_exp_pulse = 0;
        MOV       N:_energy+15, #0x0  ;; 1 cycle
        BR        S:??power_filter_56  ;; 3 cycles
        ; ------------------------------------- Block: 4 cycles
//  780             }
//  781         }
//  782     }
//  783     else if(phase == PHASE_B)
??power_filter_57:
        XCH       A, B               ;; 1 cycle
        CMP       A, #0x2            ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        BNZ       ??power_filter_56  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  784     {
//  785         if(quad== Q1 || quad == Q4)
        XCH       A, X               ;; 1 cycle
        CMP       A, #0x1            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        BZ        ??power_filter_60  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        XCH       A, X               ;; 1 cycle
        CMP       A, #0x4            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        BNZ       ??power_filter_61  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  786         {
//  787             energy.Bph.active_imp_pulse++;
??power_filter_60:
        INC       N:_energy+28       ;; 2 cycles
//  788             energy.Allph.active_imp_pulse++;
        INC       N:_energy+42       ;; 2 cycles
//  789             if(energy.Bph.active_imp_pulse >= PULSE)
        MOV       A, N:_energy+28    ;; 1 cycle
        CMP       A, #0x6            ;; 1 cycle
        BC        ??power_filter_56  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
//  790             {
//  791                 energy.Bph.active_imp_pulse = 0;
        MOV       N:_energy+28, #0x0  ;; 1 cycle
//  792                 energy.Bph.active_imp += QUANTA;
        MOVW      HL, N:_energy+32   ;; 1 cycle
        MOVW      DE, N:_energy+30   ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        ADDW      AX, #0x5           ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        SKNC
        INCW      HL                 ;; 5 cycles
        XCHW      AX, HL             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        MOVW      N:_energy+30, AX   ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      N:_energy+32, AX   ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
//  793                 if(bitIsSet(tpr.magnet.flag,event_f) && (MAG_SWITCH_IMAX == 1))
        MOVW      HL, #LWRD(_tpr+32)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??power_filter_56  ;; 4 cycles
        ; ------------------------------------- Block: 26 cycles
//  794                 {
//  795                   energy.Bph.defraud_mag += QUANTA;
        MOVW      HL, N:_energy+40   ;; 1 cycle
        MOVW      DE, N:_energy+38   ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        ADDW      AX, #0x5           ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        SKNC
        INCW      HL                 ;; 5 cycles
        XCHW      AX, HL             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        MOVW      N:_energy+38, AX   ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      N:_energy+40, AX   ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        BR        S:??power_filter_56  ;; 3 cycles
        ; ------------------------------------- Block: 22 cycles
//  796                 }
//  797             }
//  798         }
//  799         else
//  800         {
//  801             energy.Bph.active_exp_pulse++;
??power_filter_61:
        INC       N:_energy+29       ;; 2 cycles
//  802             energy.Allph.active_exp_pulse++;
        INC       N:_energy+43       ;; 2 cycles
//  803             if(energy.Bph.active_exp_pulse >= PULSE)
        MOV       A, N:_energy+29    ;; 1 cycle
        CMP       A, #0x6            ;; 1 cycle
        SKC                          ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
//  804             {
//  805                 energy.Bph.active_exp_pulse = 0;
        MOV       N:_energy+29, #0x0  ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
//  806             }
//  807         }
//  808     }
//  809     if(energy.Allph.active_imp_pulse >= PULSE)
??power_filter_56:
        MOV       A, N:_energy+42    ;; 1 cycle
        CMP       A, #0x6            ;; 1 cycle
        BC        ??power_filter_62  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  810     {
//  811         energy.Allph.active_imp_pulse = 0;
        MOV       N:_energy+42, #0x0  ;; 1 cycle
//  812         energy.Allph.active_imp += QUANTA;
        MOVW      HL, N:_energy+54   ;; 1 cycle
        MOVW      DE, N:_energy+52   ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        ADDW      AX, #0x5           ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        SKNC
        INCW      HL                 ;; 5 cycles
        XCHW      AX, HL             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        MOVW      N:_energy+52, AX   ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      N:_energy+54, AX   ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
//  813         energy.Allph.zkwh_imp += QUANTA;
        MOVW      HL, N:_energy+94   ;; 1 cycle
        MOVW      DE, N:_energy+92   ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        ADDW      AX, #0x5           ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        SKNC
        INCW      HL                 ;; 5 cycles
        XCHW      AX, HL             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        MOVW      N:_energy+92, AX   ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      N:_energy+94, AX   ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
//  814         if(bitIsSet(tpr.magnet.flag,event_f) && (MAG_SWITCH_IMAX == 1))
        MOVW      HL, #LWRD(_tpr+32)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??power_filter_62  ;; 4 cycles
        ; ------------------------------------- Block: 45 cycles
//  815         {
//  816             energy.Allph.defraud_mag += QUANTA;
        MOVW      HL, N:_energy+62   ;; 1 cycle
        MOVW      DE, N:_energy+60   ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        ADDW      AX, #0x5           ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        SKNC
        INCW      HL                 ;; 5 cycles
        XCHW      AX, HL             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        MOVW      N:_energy+60, AX   ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      N:_energy+62, AX   ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        ; ------------------------------------- Block: 19 cycles
//  817         }
//  818     }
//  819     if(energy.Allph.active_exp_pulse >= PULSE)
??power_filter_62:
        MOV       A, N:_energy+43    ;; 1 cycle
        CMP       A, #0x6            ;; 1 cycle
        BC        ??power_filter_63  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  820     {
//  821         energy.Allph.active_exp_pulse = 0;
        MOV       N:_energy+43, #0x0  ;; 1 cycle
//  822         energy.Allph.active_exp += QUANTA;
        MOVW      HL, N:_energy+58   ;; 1 cycle
        MOVW      DE, N:_energy+56   ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        ADDW      AX, #0x5           ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        SKNC
        INCW      HL                 ;; 5 cycles
        XCHW      AX, HL             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        MOVW      N:_energy+56, AX   ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      N:_energy+58, AX   ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
//  823         energy.Allph.zkwh_exp += QUANTA;
        MOVW      HL, N:_energy+98   ;; 1 cycle
        MOVW      DE, N:_energy+96   ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        ADDW      AX, #0x5           ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        SKNC
        INCW      HL                 ;; 5 cycles
        XCHW      AX, HL             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        MOVW      N:_energy+96, AX   ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      N:_energy+98, AX   ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        ; ------------------------------------- Block: 39 cycles
//  824     }
//  825 }
??power_filter_63:
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock4
        ; ------------------------------------- Block: 6 cycles
        ; ------------------------------------- Total: 395 cycles

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock5 Using cfiCommon0
          CFI Function _active_calculation_fwd
          CFI NoCalls
        CODE
//  826 void active_calculation_fwd(us8 phase,us8 quad)
//  827 {
_active_calculation_fwd:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
        MOV       B, A               ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       C, A               ;; 1 cycle
        XCH       A, X               ;; 1 cycle
//  828     flag_metro_save_energy = 1;
        SET1      N:_flag_metro1.4   ;; 2 cycles
//  829     
//  830     if(phase == PHASE_R)
        CMP0      B                  ;; 1 cycle
        BNZ       ??power_filter_64  ;; 4 cycles
        ; ------------------------------------- Block: 11 cycles
//  831     {
//  832         energy.Rph.active_imp_pulse++;
        INC       N:_energy          ;; 2 cycles
//  833         if(energy.Rph.active_imp_pulse >= PULSE)
        MOV       A, N:_energy       ;; 1 cycle
        CMP       A, #0x6            ;; 1 cycle
        SKNC                         ;; 4 cycles
        BR        N:??power_filter_65  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
//  834         {
//  835           energy.Rph.active_imp_pulse = 0;
        MOV       N:_energy, #0x0    ;; 1 cycle
//  836           energy.Rph.active_imp += QUANTA;
        MOVW      HL, N:_energy+4    ;; 1 cycle
        MOVW      DE, N:_energy+2    ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        ADDW      AX, #0x5           ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        SKNC
        INCW      HL                 ;; 5 cycles
        XCHW      AX, HL             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        MOVW      N:_energy+2, AX    ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      N:_energy+4, AX    ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
//  837           if(bitIsSet(tpr.magnet.flag,event_f) && (MAG_SWITCH_IMAX == 1))
        MOVW      HL, #LWRD(_tpr+32)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        SKC                          ;; 4 cycles
        BR        N:??power_filter_65  ;; 4 cycles
        ; ------------------------------------- Block: 26 cycles
//  838           {
//  839             energy.Rph.defraud_mag += QUANTA;
        MOVW      HL, N:_energy+12   ;; 1 cycle
        MOVW      DE, N:_energy+10   ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        ADDW      AX, #0x5           ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        SKNC
        INCW      HL                 ;; 5 cycles
        XCHW      AX, HL             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        MOVW      N:_energy+10, AX   ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      N:_energy+12, AX   ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        RET                          ;; 6 cycles
        ; ------------------------------------- Block: 25 cycles
//  840           }
//  841         }
//  842     }
//  843     else if(phase == PHASE_Y)
??power_filter_64:
        XCH       A, B               ;; 1 cycle
        CMP       A, #0x1            ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        BNZ       ??power_filter_66  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  844     {
//  845         energy.Yph.active_imp_pulse++;
        INC       N:_energy+14       ;; 2 cycles
//  846         if(energy.Yph.active_imp_pulse >= PULSE)
        MOV       A, N:_energy+14    ;; 1 cycle
        CMP       A, #0x6            ;; 1 cycle
        SKNC                         ;; 4 cycles
        BR        N:??power_filter_65  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
//  847         {
//  848             energy.Yph.active_imp_pulse = 0;
        MOV       N:_energy+14, #0x0  ;; 1 cycle
//  849             energy.Yph.active_imp += QUANTA;
        MOVW      HL, N:_energy+18   ;; 1 cycle
        MOVW      DE, N:_energy+16   ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        ADDW      AX, #0x5           ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        SKNC
        INCW      HL                 ;; 5 cycles
        XCHW      AX, HL             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        MOVW      N:_energy+16, AX   ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      N:_energy+18, AX   ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
//  850             if(bitIsSet(tpr.magnet.flag,event_f) && (MAG_SWITCH_IMAX == 1))
        MOVW      HL, #LWRD(_tpr+32)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        SKC                          ;; 4 cycles
        BR        N:??power_filter_65  ;; 4 cycles
        ; ------------------------------------- Block: 26 cycles
//  851             {
//  852               energy.Yph.defraud_mag += QUANTA;
        MOVW      HL, N:_energy+26   ;; 1 cycle
        MOVW      DE, N:_energy+24   ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        ADDW      AX, #0x5           ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        SKNC
        INCW      HL                 ;; 5 cycles
        XCHW      AX, HL             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        MOVW      N:_energy+24, AX   ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      N:_energy+26, AX   ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        RET                          ;; 6 cycles
        ; ------------------------------------- Block: 25 cycles
//  853             }
//  854         }
//  855     }
//  856     else if(phase == PHASE_B)
??power_filter_66:
        XCH       A, B               ;; 1 cycle
        CMP       A, #0x2            ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        BNZ       ??power_filter_67  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  857     {
//  858         energy.Bph.active_imp_pulse++;
        INC       N:_energy+28       ;; 2 cycles
//  859         if(energy.Bph.active_imp_pulse >= PULSE)
        MOV       A, N:_energy+28    ;; 1 cycle
        CMP       A, #0x6            ;; 1 cycle
        SKNC                         ;; 4 cycles
        BR        N:??power_filter_65  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
//  860         {
//  861             energy.Bph.active_imp_pulse = 0;
        MOV       N:_energy+28, #0x0  ;; 1 cycle
//  862             energy.Bph.active_imp += QUANTA;
        MOVW      HL, N:_energy+32   ;; 1 cycle
        MOVW      DE, N:_energy+30   ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        ADDW      AX, #0x5           ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        SKNC
        INCW      HL                 ;; 5 cycles
        XCHW      AX, HL             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        MOVW      N:_energy+30, AX   ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      N:_energy+32, AX   ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
//  863             if(bitIsSet(tpr.magnet.flag,event_f) && (MAG_SWITCH_IMAX == 1))
        MOVW      HL, #LWRD(_tpr+32)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        SKC                          ;; 4 cycles
        BR        N:??power_filter_65  ;; 4 cycles
        ; ------------------------------------- Block: 26 cycles
//  864             {
//  865               energy.Bph.defraud_mag += QUANTA;
        MOVW      HL, N:_energy+40   ;; 1 cycle
        MOVW      DE, N:_energy+38   ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        ADDW      AX, #0x5           ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        SKNC
        INCW      HL                 ;; 5 cycles
        XCHW      AX, HL             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        MOVW      N:_energy+38, AX   ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      N:_energy+40, AX   ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        RET                          ;; 6 cycles
        ; ------------------------------------- Block: 25 cycles
//  866             }
//  867         }
//  868     }
//  869     else if(phase == PHASE_ALL)
??power_filter_67:
        XCH       A, B               ;; 1 cycle
        CMP       A, #0x3            ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        SKZ                          ;; 4 cycles
        BR        N:??power_filter_65  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  870     {
//  871         energy.Allph.active_imp_pulse++;
        INC       N:_energy+42       ;; 2 cycles
//  872         if(energy.Allph.active_imp_pulse >= PULSE)
        MOV       A, N:_energy+42    ;; 1 cycle
        CMP       A, #0x6            ;; 1 cycle
        BC        ??power_filter_68  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
//  873         {
//  874             energy.Allph.active_imp_pulse = 0;
        MOV       N:_energy+42, #0x0  ;; 1 cycle
//  875             energy.Allph.active_imp += QUANTA;
        MOVW      HL, N:_energy+54   ;; 1 cycle
        MOVW      DE, N:_energy+52   ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        ADDW      AX, #0x5           ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        SKNC
        INCW      HL                 ;; 5 cycles
        XCHW      AX, HL             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        MOVW      N:_energy+52, AX   ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      N:_energy+54, AX   ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
//  876             energy.Allph.zkwh_imp += QUANTA;
        MOVW      HL, N:_energy+94   ;; 1 cycle
        MOVW      DE, N:_energy+92   ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        ADDW      AX, #0x5           ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        SKNC
        INCW      HL                 ;; 5 cycles
        XCHW      AX, HL             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        MOVW      N:_energy+92, AX   ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      N:_energy+94, AX   ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
//  877             if(bitIsSet(tpr.magnet.flag,event_f) && (MAG_SWITCH_IMAX == 1))
        MOVW      HL, #LWRD(_tpr+32)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??power_filter_68  ;; 4 cycles
        ; ------------------------------------- Block: 45 cycles
//  878             {
//  879                 energy.Allph.defraud_mag += QUANTA;
        MOVW      HL, N:_energy+62   ;; 1 cycle
        MOVW      DE, N:_energy+60   ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        ADDW      AX, #0x5           ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        SKNC
        INCW      HL                 ;; 5 cycles
        XCHW      AX, HL             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        MOVW      N:_energy+60, AX   ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      N:_energy+62, AX   ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        ; ------------------------------------- Block: 19 cycles
//  880             }
//  881         }
//  882         if(thd.Rph.correction == 0 && thd.Yph.correction == 0 && thd.Bph.correction == 0)
??power_filter_68:
        CMP0      N:_thd+16          ;; 1 cycle
        BNZ       ??power_filter_65  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        CMP0      N:_thd+34          ;; 1 cycle
        BNZ       ??power_filter_65  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        CMP0      N:_thd+52          ;; 1 cycle
        BNZ       ??power_filter_65  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  883         {
//  884             energy.Allph.fundamental_pulse++;
        INC       N:_energy+44       ;; 2 cycles
//  885             if(energy.Allph.fundamental_pulse >= PULSE)
        MOV       A, N:_energy+44    ;; 1 cycle
        CMP       A, #0x6            ;; 1 cycle
        BC        ??power_filter_69  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
//  886             {
//  887                 energy.Allph.fundamental_pulse = 0;
        MOV       N:_energy+44, #0x0  ;; 1 cycle
//  888                 energy.Allph.fundamental += QUANTA;
        MOVW      HL, N:_energy+66   ;; 1 cycle
        MOVW      DE, N:_energy+64   ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        ADDW      AX, #0x5           ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        SKNC
        INCW      HL                 ;; 5 cycles
        XCHW      AX, HL             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        MOVW      N:_energy+64, AX   ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      N:_energy+66, AX   ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        ; ------------------------------------- Block: 20 cycles
//  889             }
//  890             all_phase.fundamental.active.acc_delta_cnts = 0;
??power_filter_69:
        MOVW      HL, #LWRD(_all_phase+120)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        ; ------------------------------------- Block: 9 cycles
//  891         }
//  892     }
//  893 }
??power_filter_65:
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock5
        ; ------------------------------------- Block: 6 cycles
        ; ------------------------------------- Total: 339 cycles

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock6 Using cfiCommon0
          CFI Function _reactive_calculation_net
        CODE
//  894 void reactive_calculation_net(us8 phase,us8 quad)
//  895 {
_reactive_calculation_net:
        ; * Stack frame (at entry) *
        ; Param size: 0
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+6
        ; Auto size: 4
        SUBW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+8
//  896     us8 us8_temp;
//  897     
//  898     flag_metro_save_energy = 1;
        SET1      N:_flag_metro1.4   ;; 2 cycles
//  899     if(pwrup_sec_cnt < 10 && pwrup_sec_cnt != 0)
        MOV       A, N:_pwrup_sec_cnt  ;; 1 cycle
        CMP       A, #0xA            ;; 1 cycle
        BNC       ??power_filter_70  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
        CMP0      N:_pwrup_sec_cnt   ;; 1 cycle
        BZ        ??power_filter_70  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  900     {
//  901         energy.Allph.reactive_powerup_pulse++;
        INC       N:_energy+51       ;; 2 cycles
        BR        N:??power_filter_71  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  902     }
//  903     else
//  904     {
//  905         if(quad == Q1)    
??power_filter_70:
        MOV       A, [SP+0x02]       ;; 1 cycle
        CMP       A, #0x1            ;; 1 cycle
        BNZ       ??power_filter_72  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  906         {
//  907             energy.Allph.reactive_q1_pulse++;
        INC       N:_energy+47       ;; 2 cycles
//  908             energy.Allph.reactive_q1_pulse += energy.Allph.reactive_powerup_pulse;
        MOV       A, N:_energy+51    ;; 1 cycle
        MOVW      HL, #LWRD(_energy+47)  ;; 1 cycle
        ADD       A, [HL]            ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
        BR        S:??power_filter_73  ;; 3 cycles
        ; ------------------------------------- Block: 9 cycles
//  909         }
//  910         else if(quad == Q2)  
??power_filter_72:
        MOV       A, [SP+0x02]       ;; 1 cycle
        CMP       A, #0x2            ;; 1 cycle
        BNZ       ??power_filter_74  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  911         {
//  912             energy.Allph.reactive_q2_pulse++;
        INC       N:_energy+48       ;; 2 cycles
//  913             energy.Allph.reactive_q2_pulse += energy.Allph.reactive_powerup_pulse;
        MOV       A, N:_energy+51    ;; 1 cycle
        MOVW      HL, #LWRD(_energy+48)  ;; 1 cycle
        ADD       A, [HL]            ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
        BR        S:??power_filter_73  ;; 3 cycles
        ; ------------------------------------- Block: 9 cycles
//  914         }
//  915         else if(quad == Q3)  
??power_filter_74:
        MOV       A, [SP+0x02]       ;; 1 cycle
        CMP       A, #0x3            ;; 1 cycle
        BNZ       ??power_filter_75  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  916         {
//  917             energy.Allph.reactive_q3_pulse++;
        INC       N:_energy+49       ;; 2 cycles
//  918             energy.Allph.reactive_q3_pulse += energy.Allph.reactive_powerup_pulse;
        MOV       A, N:_energy+51    ;; 1 cycle
        MOVW      HL, #LWRD(_energy+49)  ;; 1 cycle
        ADD       A, [HL]            ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
        BR        S:??power_filter_73  ;; 3 cycles
        ; ------------------------------------- Block: 9 cycles
//  919         }
//  920         else if(quad == Q4)   
??power_filter_75:
        MOV       A, [SP+0x02]       ;; 1 cycle
        CMP       A, #0x4            ;; 1 cycle
        BNZ       ??power_filter_73  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  921         {
//  922             energy.Allph.reactive_q4_pulse++;
        INC       N:_energy+50       ;; 2 cycles
//  923             energy.Allph.reactive_q4_pulse += energy.Allph.reactive_powerup_pulse;
        MOV       A, N:_energy+51    ;; 1 cycle
        MOVW      HL, #LWRD(_energy+50)  ;; 1 cycle
        ADD       A, [HL]            ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
//  924         }
//  925         energy.Allph.reactive_powerup_pulse = 0;
??power_filter_73:
        MOV       N:_energy+51, #0x0  ;; 1 cycle
//  926         if(energy.Allph.reactive_q1_pulse >= PULSE)
        MOV       A, N:_energy+47    ;; 1 cycle
        CMP       A, #0x6            ;; 1 cycle
        SKNC                         ;; 4 cycles
        BR        N:??power_filter_76  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  927         {
//  928             if(pwrup_sec_cnt >= 10)
        MOV       A, N:_pwrup_sec_cnt  ;; 1 cycle
        CMP       A, #0xA            ;; 1 cycle
        BC        ??power_filter_77  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  929             {
//  930                 us8_temp = (energy.Allph.reactive_q1_pulse/PULSE)*QUANTA;
        MOV       A, N:_energy+47    ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        MOV       X, #0x6            ;; 1 cycle
        MOV       A, B               ;; 1 cycle
          CFI FunCall ?UC_DIV_L01
        CALL      N:?UC_DIV_L01      ;; 3 cycles
        MOV       B, A               ;; 1 cycle
        MOV       X, #0x5            ;; 1 cycle
        MOV       A, B               ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  931                 energy.Allph.reactive_q1 += us8_temp;
        MOV       A, [SP]            ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        MOVW      HL, N:_energy+78   ;; 1 cycle
        MOVW      DE, N:_energy+76   ;; 1 cycle
        ADDW      AX, DE             ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+76, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+78, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  932                 energy.Allph.zkvarh_q1 += us8_temp;
        MOV       A, [SP]            ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        MOVW      HL, N:_energy+110  ;; 1 cycle
        MOVW      DE, N:_energy+108  ;; 1 cycle
        ADDW      AX, DE             ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+108, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+110, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  933                 energy.Allph.reactive_q1_pulse  -= (energy.Allph.reactive_q1_pulse/PULSE)*PULSE;
        MOV       A, N:_energy+47    ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        MOV       X, #0x6            ;; 1 cycle
        MOV       A, B               ;; 1 cycle
          CFI FunCall ?UC_DIV_L01
        CALL      N:?UC_DIV_L01      ;; 3 cycles
        MOV       B, A               ;; 1 cycle
        MOV       X, #0x6            ;; 1 cycle
        MOV       A, B               ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, N:_energy+47    ;; 1 cycle
        SUB       A, X               ;; 1 cycle
        MOV       N:_energy+47, A    ;; 1 cycle
        BR        N:??power_filter_78  ;; 3 cycles
        ; ------------------------------------- Block: 68 cycles
//  934                 
//  935                 //                energy.Allph.reactive_q1 += (energy.Allph.reactive_q1_pulse/PULSE)*QUANTA;
//  936                 //                energy.Allph.zkvarh_q1 += (energy.Allph.reactive_q1_pulse/PULSE)*QUANTA;
//  937                 //                energy.Allph.reactive_q1_pulse %= PULSE;
//  938             }
//  939             else
//  940             {
//  941                 energy.Allph.reactive_q1 += QUANTA;
??power_filter_77:
        MOVW      BC, N:_energy+78   ;; 1 cycle
        MOVW      AX, N:_energy+76   ;; 1 cycle
        ADDW      AX, #0x5           ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+76, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+78, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  942                 energy.Allph.zkvarh_q1 += QUANTA;
        MOVW      BC, N:_energy+110  ;; 1 cycle
        MOVW      AX, N:_energy+108  ;; 1 cycle
        ADDW      AX, #0x5           ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+108, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+110, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  943                 energy.Allph.reactive_q1_pulse = 0;
        MOV       N:_energy+47, #0x0  ;; 1 cycle
        BR        N:??power_filter_78  ;; 3 cycles
        ; ------------------------------------- Block: 34 cycles
//  944             }
//  945         }
//  946         else if(energy.Allph.reactive_q2_pulse >= PULSE)
??power_filter_76:
        MOV       A, N:_energy+48    ;; 1 cycle
        CMP       A, #0x6            ;; 1 cycle
        SKNC                         ;; 4 cycles
        BR        N:??power_filter_79  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  947         {
//  948             if(pwrup_sec_cnt >= 10)
        MOV       A, N:_pwrup_sec_cnt  ;; 1 cycle
        CMP       A, #0xA            ;; 1 cycle
        BC        ??power_filter_80  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  949             {
//  950                 us8_temp = (energy.Allph.reactive_q2_pulse/PULSE)*QUANTA;
        MOV       A, N:_energy+48    ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        MOV       X, #0x6            ;; 1 cycle
        MOV       A, B               ;; 1 cycle
          CFI FunCall ?UC_DIV_L01
        CALL      N:?UC_DIV_L01      ;; 3 cycles
        MOV       B, A               ;; 1 cycle
        MOV       X, #0x5            ;; 1 cycle
        MOV       A, B               ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  951                 energy.Allph.reactive_q2 += us8_temp;
        MOV       A, [SP]            ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        MOVW      HL, N:_energy+82   ;; 1 cycle
        MOVW      DE, N:_energy+80   ;; 1 cycle
        ADDW      AX, DE             ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+80, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+82, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  952                 energy.Allph.zkvarh_q2 += us8_temp;
        MOV       A, [SP]            ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        MOVW      HL, N:_energy+114  ;; 1 cycle
        MOVW      DE, N:_energy+112  ;; 1 cycle
        ADDW      AX, DE             ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+112, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+114, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  953                 energy.Allph.reactive_q2_pulse  -= (energy.Allph.reactive_q2_pulse/PULSE)*PULSE;
        MOV       A, N:_energy+48    ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        MOV       X, #0x6            ;; 1 cycle
        MOV       A, B               ;; 1 cycle
          CFI FunCall ?UC_DIV_L01
        CALL      N:?UC_DIV_L01      ;; 3 cycles
        MOV       B, A               ;; 1 cycle
        MOV       X, #0x6            ;; 1 cycle
        MOV       A, B               ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, N:_energy+48    ;; 1 cycle
        SUB       A, X               ;; 1 cycle
        MOV       N:_energy+48, A    ;; 1 cycle
        BR        N:??power_filter_78  ;; 3 cycles
        ; ------------------------------------- Block: 68 cycles
//  954             }
//  955             else
//  956             {
//  957                 energy.Allph.reactive_q2 += QUANTA;
??power_filter_80:
        MOVW      BC, N:_energy+82   ;; 1 cycle
        MOVW      AX, N:_energy+80   ;; 1 cycle
        ADDW      AX, #0x5           ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+80, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+82, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  958                 energy.Allph.zkvarh_q2 += QUANTA;
        MOVW      BC, N:_energy+114  ;; 1 cycle
        MOVW      AX, N:_energy+112  ;; 1 cycle
        ADDW      AX, #0x5           ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+112, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+114, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  959                 energy.Allph.reactive_q2_pulse = 0;
        MOV       N:_energy+48, #0x0  ;; 1 cycle
        BR        N:??power_filter_78  ;; 3 cycles
        ; ------------------------------------- Block: 34 cycles
//  960             }
//  961         }
//  962         else if(energy.Allph.reactive_q3_pulse >= PULSE)
??power_filter_79:
        MOV       A, N:_energy+49    ;; 1 cycle
        CMP       A, #0x6            ;; 1 cycle
        SKNC                         ;; 4 cycles
        BR        N:??power_filter_81  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  963         {
//  964             if(pwrup_sec_cnt >= 10)
        MOV       A, N:_pwrup_sec_cnt  ;; 1 cycle
        CMP       A, #0xA            ;; 1 cycle
        BC        ??power_filter_82  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  965             {
//  966                 us8_temp = (energy.Allph.reactive_q3_pulse/PULSE)*QUANTA;
        MOV       A, N:_energy+49    ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        MOV       X, #0x6            ;; 1 cycle
        MOV       A, B               ;; 1 cycle
          CFI FunCall ?UC_DIV_L01
        CALL      N:?UC_DIV_L01      ;; 3 cycles
        MOV       B, A               ;; 1 cycle
        MOV       X, #0x5            ;; 1 cycle
        MOV       A, B               ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  967                 energy.Allph.reactive_q3 += us8_temp;
        MOV       A, [SP]            ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        MOVW      HL, N:_energy+86   ;; 1 cycle
        MOVW      DE, N:_energy+84   ;; 1 cycle
        ADDW      AX, DE             ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+84, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+86, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  968                 energy.Allph.zkvarh_q3 += us8_temp;
        MOV       A, [SP]            ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        MOVW      HL, N:_energy+118  ;; 1 cycle
        MOVW      DE, N:_energy+116  ;; 1 cycle
        ADDW      AX, DE             ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+116, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+118, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  969                 energy.Allph.reactive_q3_pulse  -= (energy.Allph.reactive_q3_pulse/PULSE)*PULSE;
        MOV       A, N:_energy+49    ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        MOV       X, #0x6            ;; 1 cycle
        MOV       A, B               ;; 1 cycle
          CFI FunCall ?UC_DIV_L01
        CALL      N:?UC_DIV_L01      ;; 3 cycles
        MOV       B, A               ;; 1 cycle
        MOV       X, #0x6            ;; 1 cycle
        MOV       A, B               ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, N:_energy+49    ;; 1 cycle
        SUB       A, X               ;; 1 cycle
        MOV       N:_energy+49, A    ;; 1 cycle
        BR        N:??power_filter_78  ;; 3 cycles
        ; ------------------------------------- Block: 68 cycles
//  970             }
//  971             else
//  972             {
//  973                 energy.Allph.reactive_q3 += QUANTA;
??power_filter_82:
        MOVW      BC, N:_energy+86   ;; 1 cycle
        MOVW      AX, N:_energy+84   ;; 1 cycle
        ADDW      AX, #0x5           ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+84, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+86, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  974                 energy.Allph.zkvarh_q3 += QUANTA;
        MOVW      BC, N:_energy+118  ;; 1 cycle
        MOVW      AX, N:_energy+116  ;; 1 cycle
        ADDW      AX, #0x5           ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+116, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+118, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  975                 energy.Allph.reactive_q3_pulse = 0;
        MOV       N:_energy+49, #0x0  ;; 1 cycle
        BR        N:??power_filter_78  ;; 3 cycles
        ; ------------------------------------- Block: 34 cycles
//  976             }
//  977         }
//  978         else if(energy.Allph.reactive_q4_pulse >= PULSE)
??power_filter_81:
        MOV       A, N:_energy+50    ;; 1 cycle
        CMP       A, #0x6            ;; 1 cycle
        SKNC                         ;; 4 cycles
        BR        N:??power_filter_78  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  979         {
//  980             if(pwrup_sec_cnt >= 10)
        MOV       A, N:_pwrup_sec_cnt  ;; 1 cycle
        CMP       A, #0xA            ;; 1 cycle
        BC        ??power_filter_83  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  981             {
//  982                 us8_temp = (energy.Allph.reactive_q4_pulse/PULSE)*QUANTA;
        MOV       A, N:_energy+50    ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        MOV       X, #0x6            ;; 1 cycle
        MOV       A, B               ;; 1 cycle
          CFI FunCall ?UC_DIV_L01
        CALL      N:?UC_DIV_L01      ;; 3 cycles
        MOV       B, A               ;; 1 cycle
        MOV       X, #0x5            ;; 1 cycle
        MOV       A, B               ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  983                 energy.Allph.reactive_q4 += us8_temp;
        MOV       A, [SP]            ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        MOVW      HL, N:_energy+90   ;; 1 cycle
        MOVW      DE, N:_energy+88   ;; 1 cycle
        ADDW      AX, DE             ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+88, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+90, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  984                 energy.Allph.zkvarh_q4 += us8_temp;
        MOV       A, [SP]            ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        MOVW      HL, N:_energy+122  ;; 1 cycle
        MOVW      DE, N:_energy+120  ;; 1 cycle
        ADDW      AX, DE             ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+120, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+122, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  985                 energy.Allph.reactive_q4_pulse  -= (energy.Allph.reactive_q4_pulse/PULSE)*PULSE;
        MOV       A, N:_energy+50    ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        MOV       X, #0x6            ;; 1 cycle
        MOV       A, B               ;; 1 cycle
          CFI FunCall ?UC_DIV_L01
        CALL      N:?UC_DIV_L01      ;; 3 cycles
        MOV       B, A               ;; 1 cycle
        MOV       X, #0x6            ;; 1 cycle
        MOV       A, B               ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, N:_energy+50    ;; 1 cycle
        SUB       A, X               ;; 1 cycle
        MOV       N:_energy+50, A    ;; 1 cycle
        BR        S:??power_filter_78  ;; 3 cycles
        ; ------------------------------------- Block: 68 cycles
//  986             }
//  987             else
//  988             {
//  989                 energy.Allph.reactive_q4 += QUANTA;
??power_filter_83:
        MOVW      BC, N:_energy+90   ;; 1 cycle
        MOVW      AX, N:_energy+88   ;; 1 cycle
        ADDW      AX, #0x5           ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+88, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+90, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  990                 energy.Allph.zkvarh_q4 += QUANTA;
        MOVW      BC, N:_energy+122  ;; 1 cycle
        MOVW      AX, N:_energy+120  ;; 1 cycle
        ADDW      AX, #0x5           ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+120, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+122, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  991                 energy.Allph.reactive_q4_pulse = 0;
        MOV       N:_energy+50, #0x0  ;; 1 cycle
        ; ------------------------------------- Block: 31 cycles
//  992             }
//  993         }
//  994         pwrup_sec_cnt = 0;
??power_filter_78:
        MOV       N:_pwrup_sec_cnt, #0x0  ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
//  995     }
//  996     
//  997     
//  998 }
??power_filter_71:
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock6
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 539 cycles

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock7 Using cfiCommon0
          CFI Function _reactive_calculation_fwd
        CODE
//  999 void reactive_calculation_fwd(us8 phase,us8 quad)
// 1000 {
_reactive_calculation_fwd:
        ; * Stack frame (at entry) *
        ; Param size: 0
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+6
        ; Auto size: 4
        SUBW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+8
// 1001     us8 us8_temp;
// 1002     flag_metro_save_energy = 1;
        SET1      N:_flag_metro1.4   ;; 2 cycles
// 1003     if(pwrup_sec_cnt < 10 && pwrup_sec_cnt != 0)
        MOV       A, N:_pwrup_sec_cnt  ;; 1 cycle
        CMP       A, #0xA            ;; 1 cycle
        BNC       ??power_filter_84  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
        CMP0      N:_pwrup_sec_cnt   ;; 1 cycle
        BZ        ??power_filter_84  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 1004     {
// 1005         energy.Allph.reactive_powerup_pulse++;
        INC       N:_energy+51       ;; 2 cycles
        BR        N:??power_filter_85  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 1006     }
// 1007     else
// 1008     {
// 1009         if(quad == Q1)    
??power_filter_84:
        MOV       A, [SP+0x02]       ;; 1 cycle
        CMP       A, #0x1            ;; 1 cycle
        BNZ       ??power_filter_86  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1010         {
// 1011             energy.Allph.reactive_q1_pulse++;
        INC       N:_energy+47       ;; 2 cycles
// 1012             energy.Allph.reactive_q1_pulse += energy.Allph.reactive_powerup_pulse;
        MOV       A, N:_energy+51    ;; 1 cycle
        MOVW      HL, #LWRD(_energy+47)  ;; 1 cycle
        ADD       A, [HL]            ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
        BR        S:??power_filter_87  ;; 3 cycles
        ; ------------------------------------- Block: 9 cycles
// 1013         }
// 1014         else if(quad == Q4)   
??power_filter_86:
        MOV       A, [SP+0x02]       ;; 1 cycle
        CMP       A, #0x4            ;; 1 cycle
        BNZ       ??power_filter_87  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1015         {
// 1016             energy.Allph.reactive_q4_pulse++;
        INC       N:_energy+50       ;; 2 cycles
// 1017             energy.Allph.reactive_q4_pulse += energy.Allph.reactive_powerup_pulse;
        MOV       A, N:_energy+51    ;; 1 cycle
        MOVW      HL, #LWRD(_energy+50)  ;; 1 cycle
        ADD       A, [HL]            ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
// 1018         }
// 1019         energy.Allph.reactive_powerup_pulse = 0;
??power_filter_87:
        MOV       N:_energy+51, #0x0  ;; 1 cycle
// 1020         if(energy.Allph.reactive_q1_pulse >= PULSE)
        MOV       A, N:_energy+47    ;; 1 cycle
        CMP       A, #0x6            ;; 1 cycle
        SKNC                         ;; 4 cycles
        BR        N:??power_filter_88  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
// 1021         {
// 1022             if(pwrup_sec_cnt >= 10)
        MOV       A, N:_pwrup_sec_cnt  ;; 1 cycle
        CMP       A, #0xA            ;; 1 cycle
        BC        ??power_filter_89  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1023             {
// 1024                 us8_temp = (energy.Allph.reactive_q1_pulse/PULSE)*QUANTA;
        MOV       A, N:_energy+47    ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        MOV       X, #0x6            ;; 1 cycle
        MOV       A, B               ;; 1 cycle
          CFI FunCall ?UC_DIV_L01
        CALL      N:?UC_DIV_L01      ;; 3 cycles
        MOV       B, A               ;; 1 cycle
        MOV       X, #0x5            ;; 1 cycle
        MOV       A, B               ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
// 1025                 energy.Allph.reactive_q1 += us8_temp;
        MOV       A, [SP]            ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        MOVW      HL, N:_energy+78   ;; 1 cycle
        MOVW      DE, N:_energy+76   ;; 1 cycle
        ADDW      AX, DE             ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+76, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+78, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1026                 energy.Allph.zkvarh_q1 += us8_temp;
        MOV       A, [SP]            ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        MOVW      HL, N:_energy+110  ;; 1 cycle
        MOVW      DE, N:_energy+108  ;; 1 cycle
        ADDW      AX, DE             ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+108, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+110, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1027                 energy.Allph.reactive_q1_pulse -= (energy.Allph.reactive_q1_pulse/PULSE)*PULSE;
        MOV       A, N:_energy+47    ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        MOV       X, #0x6            ;; 1 cycle
        MOV       A, B               ;; 1 cycle
          CFI FunCall ?UC_DIV_L01
        CALL      N:?UC_DIV_L01      ;; 3 cycles
        MOV       B, A               ;; 1 cycle
        MOV       X, #0x6            ;; 1 cycle
        MOV       A, B               ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, N:_energy+47    ;; 1 cycle
        SUB       A, X               ;; 1 cycle
        MOV       N:_energy+47, A    ;; 1 cycle
        BR        S:??power_filter_88  ;; 3 cycles
        ; ------------------------------------- Block: 68 cycles
// 1028                 
// 1029 //                energy.Allph.reactive_q1 += (energy.Allph.reactive_q1_pulse/PULSE)*QUANTA;
// 1030 //                energy.Allph.zkvarh_q1 += (energy.Allph.reactive_q1_pulse/PULSE)*QUANTA;
// 1031 //                energy.Allph.reactive_q1_pulse %= PULSE;
// 1032             }
// 1033             else
// 1034             {
// 1035                 energy.Allph.reactive_q1 += QUANTA;
??power_filter_89:
        MOVW      BC, N:_energy+78   ;; 1 cycle
        MOVW      AX, N:_energy+76   ;; 1 cycle
        ADDW      AX, #0x5           ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+76, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+78, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1036                 energy.Allph.zkvarh_q1 += QUANTA;
        MOVW      BC, N:_energy+110  ;; 1 cycle
        MOVW      AX, N:_energy+108  ;; 1 cycle
        ADDW      AX, #0x5           ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+108, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+110, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1037                 energy.Allph.reactive_q1_pulse = 0;
        MOV       N:_energy+47, #0x0  ;; 1 cycle
        ; ------------------------------------- Block: 31 cycles
// 1038             }
// 1039         }
// 1040         if(energy.Allph.reactive_q4_pulse >= PULSE)
??power_filter_88:
        MOV       A, N:_energy+50    ;; 1 cycle
        CMP       A, #0x6            ;; 1 cycle
        SKNC                         ;; 4 cycles
        BR        N:??power_filter_90  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1041         {
// 1042             if(pwrup_sec_cnt >= 10)
        MOV       A, N:_pwrup_sec_cnt  ;; 1 cycle
        CMP       A, #0xA            ;; 1 cycle
        BC        ??power_filter_91  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1043             {
// 1044                 us8_temp = (energy.Allph.reactive_q4_pulse/PULSE)*QUANTA;
        MOV       A, N:_energy+50    ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        MOV       X, #0x6            ;; 1 cycle
        MOV       A, B               ;; 1 cycle
          CFI FunCall ?UC_DIV_L01
        CALL      N:?UC_DIV_L01      ;; 3 cycles
        MOV       B, A               ;; 1 cycle
        MOV       X, #0x5            ;; 1 cycle
        MOV       A, B               ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
// 1045                 energy.Allph.reactive_q4 += us8_temp;
        MOV       A, [SP]            ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        MOVW      HL, N:_energy+90   ;; 1 cycle
        MOVW      DE, N:_energy+88   ;; 1 cycle
        ADDW      AX, DE             ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+88, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+90, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1046                 energy.Allph.zkvarh_q4 += us8_temp;
        MOV       A, [SP]            ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        MOVW      HL, N:_energy+122  ;; 1 cycle
        MOVW      DE, N:_energy+120  ;; 1 cycle
        ADDW      AX, DE             ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+120, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+122, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1047                 energy.Allph.reactive_q4_pulse  -= (energy.Allph.reactive_q4_pulse/PULSE)*PULSE;
        MOV       A, N:_energy+50    ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        MOV       X, #0x6            ;; 1 cycle
        MOV       A, B               ;; 1 cycle
          CFI FunCall ?UC_DIV_L01
        CALL      N:?UC_DIV_L01      ;; 3 cycles
        MOV       B, A               ;; 1 cycle
        MOV       X, #0x6            ;; 1 cycle
        MOV       A, B               ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, N:_energy+50    ;; 1 cycle
        SUB       A, X               ;; 1 cycle
        MOV       N:_energy+50, A    ;; 1 cycle
        BR        S:??power_filter_90  ;; 3 cycles
        ; ------------------------------------- Block: 68 cycles
// 1048             }
// 1049             else
// 1050             {
// 1051                 energy.Allph.reactive_q4 += QUANTA;
??power_filter_91:
        MOVW      BC, N:_energy+90   ;; 1 cycle
        MOVW      AX, N:_energy+88   ;; 1 cycle
        ADDW      AX, #0x5           ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+88, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+90, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1052                 energy.Allph.zkvarh_q4 += QUANTA;
        MOVW      BC, N:_energy+122  ;; 1 cycle
        MOVW      AX, N:_energy+120  ;; 1 cycle
        ADDW      AX, #0x5           ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+120, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+122, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1053                 energy.Allph.reactive_q4_pulse = 0;
        MOV       N:_energy+50, #0x0  ;; 1 cycle
        ; ------------------------------------- Block: 31 cycles
// 1054             }
// 1055         }
// 1056         pwrup_sec_cnt = 0;
??power_filter_90:
        MOV       N:_pwrup_sec_cnt, #0x0  ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
// 1057     }
// 1058 }
??power_filter_85:
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock7
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 278 cycles

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock8 Using cfiCommon0
          CFI Function _apparent_calculation_net
          CFI NoCalls
        CODE
// 1059 void apparent_calculation_net(us8 phase,us8 quad)
// 1060 {
_apparent_calculation_net:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
        MOV       B, A               ;; 1 cycle
// 1061     flag_metro_save_energy = 1;
        SET1      N:_flag_metro1.4   ;; 2 cycles
// 1062     if(quad == Q1 || quad == Q4)
        XCH       A, X               ;; 1 cycle
        CMP       A, #0x1            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        BZ        ??power_filter_92  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
        XCH       A, X               ;; 1 cycle
        CMP       A, #0x4            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        BNZ       ??power_filter_93  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
// 1063     {
// 1064         energy.Allph.apparent_imp_pulse++;
??power_filter_92:
        INC       N:_energy+45       ;; 2 cycles
        BR        S:??power_filter_94  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 1065     }
// 1066     else
// 1067     {
// 1068         energy.Allph.apparent_exp_pulse++;
??power_filter_93:
        INC       N:_energy+46       ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
// 1069     }
// 1070     if(energy.Allph.apparent_imp_pulse >= PULSE)
??power_filter_94:
        MOV       A, N:_energy+45    ;; 1 cycle
        CMP       A, #0x6            ;; 1 cycle
        BC        ??power_filter_95  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1071     {
// 1072         energy.Allph.apparent_imp_pulse = 0;
        MOV       N:_energy+45, #0x0  ;; 1 cycle
// 1073         energy.Allph.apparent_imp += QUANTA;
        MOVW      HL, N:_energy+70   ;; 1 cycle
        MOVW      DE, N:_energy+68   ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        ADDW      AX, #0x5           ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        SKNC
        INCW      HL                 ;; 5 cycles
        XCHW      AX, HL             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        MOVW      N:_energy+68, AX   ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      N:_energy+70, AX   ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
// 1074         energy.Allph.zkvah_imp += QUANTA;
        MOVW      HL, N:_energy+102  ;; 1 cycle
        MOVW      DE, N:_energy+100  ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        ADDW      AX, #0x5           ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        SKNC
        INCW      HL                 ;; 5 cycles
        XCHW      AX, HL             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        MOVW      N:_energy+100, AX  ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      N:_energy+102, AX  ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        ; ------------------------------------- Block: 39 cycles
// 1075     }
// 1076     if(energy.Allph.apparent_exp_pulse >= PULSE)
??power_filter_95:
        MOV       A, N:_energy+46    ;; 1 cycle
        CMP       A, #0x6            ;; 1 cycle
        BC        ??power_filter_96  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1077     {
// 1078         energy.Allph.apparent_exp_pulse = 0;
        MOV       N:_energy+46, #0x0  ;; 1 cycle
// 1079         energy.Allph.apparent_exp += QUANTA;
        MOVW      HL, N:_energy+74   ;; 1 cycle
        MOVW      DE, N:_energy+72   ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        ADDW      AX, #0x5           ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        SKNC
        INCW      HL                 ;; 5 cycles
        XCHW      AX, HL             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        MOVW      N:_energy+72, AX   ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      N:_energy+74, AX   ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
// 1080         energy.Allph.zkvah_exp += QUANTA;
        MOVW      HL, N:_energy+106  ;; 1 cycle
        MOVW      DE, N:_energy+104  ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        ADDW      AX, #0x5           ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        SKNC
        INCW      HL                 ;; 5 cycles
        XCHW      AX, HL             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        MOVW      N:_energy+104, AX  ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      N:_energy+106, AX  ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        ; ------------------------------------- Block: 39 cycles
// 1081     }
// 1082 }
??power_filter_96:
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock8
        ; ------------------------------------- Block: 6 cycles
        ; ------------------------------------- Total: 120 cycles

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock9 Using cfiCommon0
          CFI Function _apparent_calculation_fwd
          CFI NoCalls
        CODE
// 1083 void apparent_calculation_fwd(us8 phase,us8 quad)
// 1084 {
_apparent_calculation_fwd:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
        MOV       B, A               ;; 1 cycle
// 1085     flag_metro_save_energy = 1;
        SET1      N:_flag_metro1.4   ;; 2 cycles
// 1086     energy.Allph.apparent_imp_pulse++;
        INC       N:_energy+45       ;; 2 cycles
// 1087     if(energy.Allph.apparent_imp_pulse >= PULSE)
        MOV       A, N:_energy+45    ;; 1 cycle
        CMP       A, #0x6            ;; 1 cycle
        BC        ??power_filter_97  ;; 4 cycles
        ; ------------------------------------- Block: 11 cycles
// 1088     {
// 1089         energy.Allph.apparent_imp_pulse = 0;
        MOV       N:_energy+45, #0x0  ;; 1 cycle
// 1090         energy.Allph.apparent_imp += QUANTA;
        MOVW      HL, N:_energy+70   ;; 1 cycle
        MOVW      DE, N:_energy+68   ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        ADDW      AX, #0x5           ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        SKNC
        INCW      HL                 ;; 5 cycles
        XCHW      AX, HL             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        MOVW      N:_energy+68, AX   ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      N:_energy+70, AX   ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
// 1091         energy.Allph.zkvah_imp += QUANTA;
        MOVW      HL, N:_energy+102  ;; 1 cycle
        MOVW      DE, N:_energy+100  ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        ADDW      AX, #0x5           ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        SKNC
        INCW      HL                 ;; 5 cycles
        XCHW      AX, HL             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        MOVW      N:_energy+100, AX  ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      N:_energy+102, AX  ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
// 1092         duplicate_total_apparent_energy += QUANTA; //duplicate functionality work only for forwarded meter.
        MOVW      HL, N:_duplicate_total_apparent_energy+2  ;; 1 cycle
        MOVW      DE, N:_duplicate_total_apparent_energy  ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        ADDW      AX, #0x5           ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        SKNC
        INCW      HL                 ;; 5 cycles
        XCHW      AX, HL             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        MOVW      N:_duplicate_total_apparent_energy, AX  ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      N:_duplicate_total_apparent_energy+2, AX  ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        ; ------------------------------------- Block: 58 cycles
// 1093     } 
// 1094 }
??power_filter_97:
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock9
        ; ------------------------------------- Block: 6 cycles
        ; ------------------------------------- Total: 75 cycles
// 1095 
// 1096 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock10 Using cfiCommon0
          CFI Function _metrology_process
        CODE
// 1097 void metrology_process()
// 1098 {
_metrology_process:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 24
        SUBW      SP, #0x18          ;; 1 cycle
          CFI CFA SP+28
// 1099     /************************************************************************************************
// 1100     *****************************************   R PHASE    ******************************************
// 1101     ************************************************************************************************/
// 1102     if(flag_metrology_process_r == 1 && r_phase.no_of_samples != 0)
        MOVW      HL, #LWRD(_flag_metrology)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        SKC                          ;; 4 cycles
        BR        N:??power_filter_98  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        CLRW      AX                 ;; 1 cycle
        CMPW      AX, N:_r_phase     ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??power_filter_98  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1103     {
// 1104         flag_metrology_process_r = 0;
        CLR1      N:_flag_metrology.0  ;; 2 cycles
// 1105         
// 1106         /* Current */ 
// 1107         curr.Rph.rms = calculate_irms(r_phase.curr.rms_acc_cnt,cal_coeff.Rph.curr,r_phase.no_of_samples);
        MOVW      AX, N:_r_phase     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+30
        MOVW      AX, N:_cal_coeff+8  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
        MOVW      HL, #LWRD(_r_phase+56)  ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+40
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
          CFI FunCall _calculate_irms
        CALL      _calculate_irms    ;; 3 cycles
        MOVW      N:_curr, AX        ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_curr+2, AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1108         curr.Rph.dc = calculate_irms(r_phase.curr.offset_acc_cnt,cal_coeff.Rph.curr,r_phase.no_of_samples);
        MOVW      AX, N:_r_phase+50  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+42
        MOVW      AX, N:_r_phase+48  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+44
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
          CFI FunCall __L2LLS
        CALL      __L2LLS            ;; 3 cycles
        MOVW      AX, N:_r_phase     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+46
        MOVW      AX, N:_cal_coeff+8  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+48
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x14          ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+56
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
          CFI FunCall _calculate_irms
        CALL      _calculate_irms    ;; 3 cycles
        MOVW      N:_curr+8, AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_curr+10, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1109         r_phase.curr.dc_offset = r_phase.curr.offset_acc_cnt / r_phase.no_of_samples;
        MOVW      AX, N:_r_phase     ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+58
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+60
        MOVW      BC, N:_r_phase+50  ;; 1 cycle
        MOVW      AX, N:_r_phase+48  ;; 1 cycle
          CFI FunCall ?SL_DIV_L03
        CALL      N:?SL_DIV_L03      ;; 3 cycles
        MOVW      N:_r_phase+46, AX  ;; 1 cycle
// 1110         
// 1111         /* Voltage */
// 1112         vol.Rph.rms = calculate_vrms(r_phase.vol.rms_acc_cnt,cal_coeff.Rph.vol,r_phase.no_of_samples);
        MOVW      AX, N:_r_phase     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+62
        MOVW      AX, N:_cal_coeff+6  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+64
        MOVW      HL, #LWRD(_r_phase+26)  ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+72
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
          CFI FunCall _calculate_vrms
        CALL      _calculate_vrms    ;; 3 cycles
        ADDW      SP, #0x2C          ;; 1 cycle
          CFI CFA SP+28
        MOVW      N:_vol, AX         ;; 1 cycle
// 1113         vol.Rph.dc = calculate_vrms(r_phase.vol.offset_acc_cnt,cal_coeff.Rph.vol,r_phase.no_of_samples);
        MOVW      AX, N:_r_phase+20  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+30
        MOVW      AX, N:_r_phase+18  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
          CFI FunCall __L2LLS
        CALL      __L2LLS            ;; 3 cycles
        MOVW      AX, N:_r_phase     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+34
        MOVW      AX, N:_cal_coeff+6  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+36
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+44
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
          CFI FunCall _calculate_vrms
        CALL      _calculate_vrms    ;; 3 cycles
        MOVW      N:_vol+4, AX       ;; 1 cycle
// 1114         r_phase.vol.dc_offset = r_phase.vol.offset_acc_cnt / r_phase.no_of_samples;
        MOVW      AX, N:_r_phase     ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+46
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+48
        MOVW      BC, N:_r_phase+20  ;; 1 cycle
        MOVW      AX, N:_r_phase+18  ;; 1 cycle
          CFI FunCall ?SL_DIV_L03
        CALL      N:?SL_DIV_L03      ;; 3 cycles
        MOVW      N:_r_phase+14, AX  ;; 1 cycle
// 1115         
// 1116         /* detecting low voltage or low current*/
// 1117         if(vol.Rph.rms >= THR_PHASE_PRESENT_VOL)
        MOVW      AX, N:_vol         ;; 1 cycle
        ADDW      SP, #0x14          ;; 1 cycle
          CFI CFA SP+28
        CMPW      AX, #0x7D0         ;; 1 cycle
        BC        ??power_filter_99  ;; 4 cycles
        ; ------------------------------------- Block: 118 cycles
// 1118         {
// 1119             flag_phase_present_r = 1;
        SET1      N:_flag_metro2.0   ;; 2 cycles
        BR        S:??power_filter_100  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 1120         }
// 1121         else
// 1122         {
// 1123             flag_phase_present_r = 0;
??power_filter_99:
        CLR1      N:_flag_metro2.0   ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
// 1124         }  
// 1125         /* detecting healthy voltage for frequency calculation */
// 1126         if(vol.Rph.rms >= THR_FREQ_DECISION)
??power_filter_100:
        MOVW      AX, N:_vol         ;; 1 cycle
        CMPW      AX, #0x1D4C        ;; 1 cycle
        BC        ??power_filter_101  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1127         {
// 1128             flag_freq_vol_r = 1;
        SET1      N:_flag_metro3.0   ;; 2 cycles
        BR        S:??power_filter_102  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 1129         }
// 1130         else
// 1131         {
// 1132             flag_freq_vol_r = 0;
??power_filter_101:
        CLR1      N:_flag_metro3.0   ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
// 1133         }  
// 1134         
// 1135         if(curr.Rph.rms < MIN_CURRENT)
??power_filter_102:
        MOVW      BC, N:_curr+2      ;; 1 cycle
        MOVW      AX, N:_curr        ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
        CMPW      AX, #0xA           ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??metrology_process_0:
        BNC       ??power_filter_103  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
// 1136         {
// 1137             flag_min_current_r = 1;
        SET1      N:_flag_metro2.4   ;; 2 cycles
        BR        S:??power_filter_104  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 1138         }
// 1139         else
// 1140         {
// 1141             flag_min_current_r = 0;
??power_filter_103:
        CLR1      N:_flag_metro2.4   ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
// 1142         }
// 1143         /* Frequency */
// 1144         if(flag_phase_present_r == 1)
??power_filter_104:
        MOVW      HL, #LWRD(_flag_metro2)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??power_filter_105  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1145         {
// 1146             freq.Rph = (us16)((us32)195312500 / r_phase.no_of_samples);
        MOVW      DE, N:_r_phase     ;; 1 cycle
        MOVW      HL, #0x0           ;; 1 cycle
        MOVW      AX, #0x3B74        ;; 1 cycle
        MOVW      BC, #0xBA4         ;; 1 cycle
        DIVWU                        ;; 17 cycles
        NOP                          ;; 1 cycle
        MOVW      N:_freq, AX        ;; 1 cycle
        BR        S:??power_filter_106  ;; 3 cycles
        ; ------------------------------------- Block: 26 cycles
// 1147         }
// 1148         else
// 1149         {
// 1150             freq.Rph = 0;
??power_filter_105:
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_freq, AX        ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
// 1151         }
// 1152         
// 1153         /* Active Energy */
// 1154         if(r_phase.active.acc_cnt < 0)
??power_filter_106:
        MOVW      BC, #LWRD(__Constant_0_0)  ;; 1 cycle
        MOVW      AX, #LWRD(_r_phase+72)  ;; 1 cycle
          CFI FunCall __CmpGes64
        CALL      __CmpGes64         ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_107  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
// 1155         {
// 1156             temp_us64 = ~r_phase.active.acc_cnt + 1;
        MOVW      BC, #LWRD(_r_phase+72)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
          CFI FunCall __Not64
        CALL      __Not64            ;; 3 cycles
        MOVW      DE, #LWRD(__Constant_1_0)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Add64
        CALL      __Add64            ;; 3 cycles
        MOVW      HL, #LWRD(_temp_us64)  ;; 1 cycle
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
// 1157             flag_Rph_active = EXPORT;
        SET1      N:_flag_quadrant.0  ;; 2 cycles
        BR        S:??power_filter_108  ;; 3 cycles
        ; ------------------------------------- Block: 30 cycles
// 1158         }
// 1159         else
// 1160         {
// 1161             temp_us64 = r_phase.active.acc_cnt;
??power_filter_107:
        MOVW      HL, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      DE, #LWRD(_r_phase+72)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 1162             flag_Rph_active = IMPORT;
        CLR1      N:_flag_quadrant.0  ;; 2 cycles
        ; ------------------------------------- Block: 12 cycles
// 1163         }
// 1164         temp_us64 *= cal_coeff.Rph.power;
??power_filter_108:
        MOVW      AX, N:_cal_coeff+10  ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+30
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
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
// 1165         temp_us64 /= r_phase.no_of_samples;
        MOVW      AX, N:_r_phase     ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+34
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+36
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
// 1166         r_phase.active.delta_cnts_temp = temp_us64;
        MOVW      HL, #LWRD(_r_phase+104)  ;; 1 cycle
        MOVW      DE, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 1167         
// 1168         /* Reactive Energy */
// 1169         if(flag_metro_react_cal_method == REACT_CAL_DELAY)
        MOVW      HL, #LWRD(_flag_metro1)  ;; 1 cycle
        MOV1      CY, [HL].3         ;; 1 cycle
        ADDW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+28
        SKNC                         ;; 4 cycles
        BR        N:??power_filter_109  ;; 4 cycles
        ; ------------------------------------- Block: 51 cycles
// 1170         {
// 1171             if(r_phase.reactive.acc_cnt < 0)
        MOVW      BC, #LWRD(__Constant_0_0)  ;; 1 cycle
        MOVW      AX, #LWRD(_r_phase+112)  ;; 1 cycle
          CFI FunCall __CmpGes64
        CALL      __CmpGes64         ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_110  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
// 1172             {
// 1173                 temp_us64 = ~r_phase.reactive.acc_cnt + 1;
        MOVW      BC, #LWRD(_r_phase+112)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
          CFI FunCall __Not64
        CALL      __Not64            ;; 3 cycles
        MOVW      DE, #LWRD(__Constant_1_0)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Add64
        CALL      __Add64            ;; 3 cycles
        MOVW      HL, #LWRD(_temp_us64)  ;; 1 cycle
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
// 1174                 flag_Rph_reactive = EXPORT;
        SET1      N:_flag_quadrant.1  ;; 2 cycles
        BR        S:??power_filter_111  ;; 3 cycles
        ; ------------------------------------- Block: 30 cycles
// 1175             }
// 1176             else
// 1177             {
// 1178                 temp_us64 = r_phase.reactive.acc_cnt;
??power_filter_110:
        MOVW      HL, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      DE, #LWRD(_r_phase+112)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 1179                 flag_Rph_reactive = IMPORT;
        CLR1      N:_flag_quadrant.1  ;; 2 cycles
        ; ------------------------------------- Block: 12 cycles
// 1180             }
// 1181             temp_us64 *= cal_coeff.Rph.power;
??power_filter_111:
        MOVW      AX, N:_cal_coeff+10  ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+30
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
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
// 1182             temp_us64 /= r_phase.no_of_samples;
        MOVW      AX, N:_r_phase     ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+34
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+36
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
// 1183             r_phase.reactive.delta_cnts_temp = temp_us64;
        MOVW      HL, #LWRD(_r_phase+144)  ;; 1 cycle
        MOVW      DE, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        ADDW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+28
        ; ------------------------------------- Block: 45 cycles
// 1184         }
// 1185         
// 1186         /* Apparent Energy */
// 1187         if(flag_metro_app_cal_method == APP_CAL_VI)
??power_filter_109:
        MOVW      HL, #LWRD(_flag_metro1)  ;; 1 cycle
        MOV1      CY, [HL].1         ;; 1 cycle
        BC        ??power_filter_112  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1188         {
// 1189             r_phase.apparent.delta_cnts_temp = ((us64)curr.Rph.rms * vol.Rph.rms)*10;
        MOVW      AX, N:_curr+2      ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+30
        MOVW      AX, N:_curr        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
          CFI FunCall __L2LLU
        CALL      __L2LLU            ;; 3 cycles
        MOVW      AX, N:_vol         ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+34
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+36
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
          CFI FunCall __L2LLU
        CALL      __L2LLU            ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x18          ;; 1 cycle
          CFI FunCall __Mul64
        CALL      __Mul64            ;; 3 cycles
        MOVW      DE, #LWRD(__Constant_a_0)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x18          ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, #LWRD(_r_phase+184)  ;; 1 cycle
          CFI FunCall __Mul64
        CALL      __Mul64            ;; 3 cycles
        ADDW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+28
        BR        N:??power_filter_113  ;; 3 cycles
        ; ------------------------------------- Block: 41 cycles
// 1190         }
// 1191         else
// 1192         {
// 1193             temp_us64 = (us64)((us64)r_phase.active.delta_cnts_temp/100)*((us64)r_phase.active.delta_cnts_temp/100);
??power_filter_112:
        MOVW      DE, #LWRD(__Constant_64_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_r_phase+104)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
        MOVW      DE, #LWRD(__Constant_64_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_r_phase+104)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __Mul64
        CALL      __Mul64            ;; 3 cycles
// 1194             temp_us64 += (us64)((us64)r_phase.reactive.delta_cnts_temp/100)*((us64)r_phase.reactive.delta_cnts_temp/100);
        MOVW      DE, #LWRD(__Constant_64_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_r_phase+144)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
        MOVW      DE, #LWRD(__Constant_64_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_r_phase+144)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
          CFI FunCall __Mul64
        CALL      __Mul64            ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      BC, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __Add64
        CALL      __Add64            ;; 3 cycles
// 1195             r_phase.apparent.delta_cnts_temp = (us64)sqrt(temp_us64)*100;
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __LLU2F
        CALL      __LLU2F            ;; 3 cycles
          CFI FunCall _sqrt
        CALL      _sqrt              ;; 3 cycles
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+30
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
          CFI FunCall __F2LLU
        CALL      __F2LLU            ;; 3 cycles
        MOVW      DE, #LWRD(__Constant_64_0)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, #LWRD(_r_phase+184)  ;; 1 cycle
          CFI FunCall __Mul64
        CALL      __Mul64            ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+28
        ; ------------------------------------- Block: 76 cycles
// 1196         }
// 1197         if((r_phase.active.delta_cnts_temp > r_phase.apparent.delta_cnts_temp) && (SP_CHECK_ACT_APP == 1) && (ZERORISE_METROLOGY_PAR == 1))
??power_filter_113:
        MOVW      BC, #LWRD(_r_phase+104)  ;; 1 cycle
        MOVW      AX, #LWRD(_r_phase+184)  ;; 1 cycle
          CFI FunCall __CmpGeu64
        CALL      __CmpGeu64         ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_114  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
// 1198         {
// 1199             r_phase.apparent.delta_cnts_temp =  r_phase.active.delta_cnts_temp;
        MOVW      HL, #LWRD(_r_phase+184)  ;; 1 cycle
        MOVW      DE, #LWRD(_r_phase+104)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        ; ------------------------------------- Block: 10 cycles
// 1200         }
// 1201         
// 1202         /* Reactive energy */
// 1203         if(flag_metro_react_cal_method == REACT_CAL_POW_TRIANGLE)
??power_filter_114:
        MOVW      HL, #LWRD(_flag_metro1)  ;; 1 cycle
        MOV1      CY, [HL].3         ;; 1 cycle
        SKC                          ;; 4 cycles
        BR        N:??power_filter_115  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1204         {
// 1205             if(r_phase.reactive.acc_cnt < 0) /* there may be issues in the calculation of quadrants at boundry conditions at specific freq points*/
        MOVW      BC, #LWRD(__Constant_0_0)  ;; 1 cycle
        MOVW      AX, #LWRD(_r_phase+112)  ;; 1 cycle
          CFI FunCall __CmpGes64
        CALL      __CmpGes64         ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_116  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
// 1206             {
// 1207                 flag_Rph_reactive = EXPORT;
        SET1      N:_flag_quadrant.1  ;; 2 cycles
        BR        S:??power_filter_117  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 1208             }
// 1209             else
// 1210             {
// 1211                 flag_Rph_reactive = IMPORT;
??power_filter_116:
        CLR1      N:_flag_quadrant.1  ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
// 1212             }
// 1213             temp_us64 = (us64)((us64)r_phase.apparent.delta_cnts_temp/100)*((us64)r_phase.apparent.delta_cnts_temp/100);
??power_filter_117:
        MOVW      DE, #LWRD(__Constant_64_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_r_phase+184)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
        MOVW      DE, #LWRD(__Constant_64_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_r_phase+184)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __Mul64
        CALL      __Mul64            ;; 3 cycles
// 1214             temp_us64 -= (us64)((us64)r_phase.active.delta_cnts_temp/100)*((us64)r_phase.active.delta_cnts_temp/100);
        MOVW      DE, #LWRD(__Constant_64_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_r_phase+104)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
        MOVW      DE, #LWRD(__Constant_64_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_r_phase+104)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
          CFI FunCall __Mul64
        CALL      __Mul64            ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      BC, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __Sub64
        CALL      __Sub64            ;; 3 cycles
// 1215             r_phase.reactive.delta_cnts_temp = (us64)sqrt(temp_us64)*100;
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __LLU2F
        CALL      __LLU2F            ;; 3 cycles
          CFI FunCall _sqrt
        CALL      _sqrt              ;; 3 cycles
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+30
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
          CFI FunCall __F2LLU
        CALL      __F2LLU            ;; 3 cycles
        MOVW      DE, #LWRD(__Constant_64_0)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, #LWRD(_r_phase+144)  ;; 1 cycle
          CFI FunCall __Mul64
        CALL      __Mul64            ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+28
        ; ------------------------------------- Block: 76 cycles
// 1216         }
// 1217         /* Special checks */
// 1218         /* checks : 
// 1219         1. magnet tamper 
// 1220         2. ZPF
// 1221         3. UPF 
// 1222         4. Low power active
// 1223         5. Low power reactive */
// 1224        
// 1225         temp_us16 = cal_pf(r_phase.active.delta_cnts_temp / 100000, r_phase.apparent.delta_cnts_temp / 100000);
??power_filter_115:
        MOVW      DE, #LWRD(__Constant_186a0_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_r_phase+184)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __LLU2L
        CALL      __LLU2L            ;; 3 cycles
        MOVW      [SP+0x08], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [SP+0x0A], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      DE, #LWRD(__Constant_186a0_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_r_phase+104)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __LLU2L
        CALL      __LLU2L            ;; 3 cycles
        MOVW      DE, AX             ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+30
        POP       HL                 ;; 1 cycle
          CFI CFA SP+28
        MOVW      AX, [SP+0x0A]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x08]      ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+30
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
        MOVW      AX, DE             ;; 1 cycle
        PUSH      HL                 ;; 1 cycle
          CFI CFA SP+34
        POP       BC                 ;; 1 cycle
          CFI CFA SP+32
          CFI FunCall _cal_pf
        CALL      _cal_pf            ;; 3 cycles
        MOVW      S:_temp_us16, AX   ;; 1 cycle
// 1226         
// 1227         if(temp_us16 < 40 && (SP_CHECK_ZPF == 1) && (ZERORISE_METROLOGY_PAR == 1))      /* ZPF */
        MOVW      AX, S:_temp_us16   ;; 1 cycle
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+28
        CMPW      AX, #0x28          ;; 1 cycle
        BNC       ??power_filter_118  ;; 4 cycles
        ; ------------------------------------- Block: 46 cycles
// 1228         {
// 1229             r_phase.active.delta_cnts_temp = 0;
        MOVW      HL, #LWRD(_r_phase+104)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 1230             r_phase.apparent.delta_cnts_temp =  r_phase.reactive.delta_cnts_temp;
        MOVW      HL, #LWRD(_r_phase+184)  ;; 1 cycle
        MOVW      DE, #LWRD(_r_phase+144)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 1231             flag_Rph_active = IMPORT;
        CLR1      N:_flag_quadrant.0  ;; 2 cycles
        ; ------------------------------------- Block: 21 cycles
// 1232         }
// 1233         if(temp_us16 > 998 && (SP_CHECK_UPF == 1) && (ZERORISE_METROLOGY_PAR == 1) && curr.Rph.rms >= DIAL_IMAX_CURR)       /* UPF */
??power_filter_118:
        MOVW      AX, S:_temp_us16   ;; 1 cycle
        CMPW      AX, #0x3E7         ;; 1 cycle
        BC        ??power_filter_119  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      BC, N:_curr+2      ;; 1 cycle
        MOVW      AX, N:_curr        ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
        CMPW      AX, #0xE678        ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??metrology_process_1:
        BC        ??power_filter_119  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
// 1234         {
// 1235             r_phase.reactive.delta_cnts_temp = 0;
        MOVW      HL, #LWRD(_r_phase+144)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 1236             r_phase.apparent.delta_cnts_temp =  r_phase.active.delta_cnts_temp;
        MOVW      HL, #LWRD(_r_phase+184)  ;; 1 cycle
        MOVW      DE, #LWRD(_r_phase+104)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 1237             flag_Rph_reactive = IMPORT;
        CLR1      N:_flag_quadrant.1  ;; 2 cycles
        ; ------------------------------------- Block: 21 cycles
// 1238         }
// 1239         if((r_phase.active.delta_cnts_temp / 100000) < 18)
??power_filter_119:
        MOVW      DE, #LWRD(__Constant_186a0_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_r_phase+104)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
        MOVW      BC, #LWRD(__Constant_12_0)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __CmpGeu64
        CALL      __CmpGeu64         ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_120  ;; 4 cycles
        ; ------------------------------------- Block: 16 cycles
// 1240         {
// 1241             r_phase.active.delta_cnts_temp = 0;
        MOVW      HL, #LWRD(_r_phase+104)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 1242             r_phase.apparent.delta_cnts_temp =  r_phase.reactive.delta_cnts_temp;
        MOVW      HL, #LWRD(_r_phase+184)  ;; 1 cycle
        MOVW      DE, #LWRD(_r_phase+144)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 1243             flag_Rph_active = IMPORT;
        CLR1      N:_flag_quadrant.0  ;; 2 cycles
        ; ------------------------------------- Block: 21 cycles
// 1244         }
// 1245         if((r_phase.reactive.delta_cnts_temp / 100000) < 18)
??power_filter_120:
        MOVW      DE, #LWRD(__Constant_186a0_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_r_phase+144)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
        MOVW      BC, #LWRD(__Constant_12_0)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __CmpGeu64
        CALL      __CmpGeu64         ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_121  ;; 4 cycles
        ; ------------------------------------- Block: 16 cycles
// 1246         {
// 1247             r_phase.reactive.delta_cnts_temp = 0;
        MOVW      HL, #LWRD(_r_phase+144)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 1248             r_phase.apparent.delta_cnts_temp =  r_phase.active.delta_cnts_temp;
        MOVW      HL, #LWRD(_r_phase+184)  ;; 1 cycle
        MOVW      DE, #LWRD(_r_phase+104)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 1249             flag_Rph_reactive = IMPORT;
        CLR1      N:_flag_quadrant.1  ;; 2 cycles
        ; ------------------------------------- Block: 21 cycles
// 1250         }
// 1251         if(bitIsSet(tpr.magnet.flag,event_f) && (MAG_SWITCH_IMAX == 1) && (flag_phase_present_r == 1))
??power_filter_121:
        MOVW      HL, #LWRD(_tpr+32)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??power_filter_122  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      HL, #LWRD(_flag_metro2)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??power_filter_122  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1252         {
// 1253             r_phase.active.delta_cnts_temp = MAG_DELTA_CNTS_IMAX;
        MOVW      HL, #LWRD(_r_phase+104)  ;; 1 cycle
        MOVW      AX, #0x9000        ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x5A4E        ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x3           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 1254             r_phase.reactive.delta_cnts_temp = 0;
        MOVW      HL, #LWRD(_r_phase+144)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 1255             r_phase.apparent.delta_cnts_temp =  r_phase.active.delta_cnts_temp;
        MOVW      HL, #LWRD(_r_phase+184)  ;; 1 cycle
        MOVW      DE, #LWRD(_r_phase+104)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 1256             flag_Rph_active = IMPORT;
        CLR1      N:_flag_quadrant.0  ;; 2 cycles
// 1257             flag_Rph_reactive = IMPORT;
        CLR1      N:_flag_quadrant.1  ;; 2 cycles
        ; ------------------------------------- Block: 32 cycles
// 1258         }
// 1259         
// 1260         if(ZERORISE_METROLOGY_PAR == 0 || 
// 1261            (flag_phase_present_r == 1 && flag_min_current_r == 0 && ZERORISE_METROLOGY_PAR == 1))
??power_filter_122:
        MOV       A, N:_flag_metro2  ;; 1 cycle
        AND       A, #0x11           ;; 1 cycle
        CMP       A, #0x1            ;; 1 cycle
        SKZ                          ;; 4 cycles
        BR        N:??power_filter_123  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
// 1262         {
// 1263             r_phase.active.delta_cnts = r_phase.active.delta_cnts_temp;
        MOVW      HL, #LWRD(_r_phase+96)  ;; 1 cycle
        MOVW      DE, #LWRD(_r_phase+104)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 1264             r_phase.reactive.delta_cnts = r_phase.reactive.delta_cnts_temp;
        MOVW      HL, #LWRD(_r_phase+136)  ;; 1 cycle
        MOVW      DE, #LWRD(_r_phase+144)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 1265             r_phase.apparent.delta_cnts = r_phase.apparent.delta_cnts_temp;
        MOVW      HL, #LWRD(_r_phase+176)  ;; 1 cycle
        MOVW      DE, #LWRD(_r_phase+184)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 1266             
// 1267             r_phase.fundamental.active.delta_cnts = r_phase.active.delta_cnts * 1000;
        MOVW      DE, #LWRD(__Constant_3e8_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_r_phase+96)  ;; 1 cycle
        MOVW      AX, #LWRD(_r_phase+200)  ;; 1 cycle
          CFI FunCall __Mul64
        CALL      __Mul64            ;; 3 cycles
// 1268             r_phase.fundamental.active.delta_cnts /= (1000+thd.Rph.correction);
        MOV       X, N:_thd+16       ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        ADDW      AX, #0x3E8         ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+30
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
          CFI FunCall __L2LLS
        CALL      __L2LLS            ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      BC, #LWRD(_r_phase+200)  ;; 1 cycle
        MOVW      AX, #LWRD(_r_phase+200)  ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
// 1269             
// 1270             if(bitIsSet(tpr.magnet.flag,event_f) && (MAG_SWITCH_IMAX == 1))
        MOVW      HL, #LWRD(_tpr+32)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+28
        SKC                          ;; 4 cycles
        BR        N:??power_filter_124  ;; 4 cycles
        ; ------------------------------------- Block: 64 cycles
// 1271             {
// 1272                 curr.Rph.rms = IMAX;
        MOVW      AX, #0xEA60        ;; 1 cycle
        MOVW      N:_curr, AX        ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+2, AX      ;; 1 cycle
// 1273                 curr.Rph.dc = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+8, AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+10, AX     ;; 1 cycle
// 1274                 curr.Rph.rms_signed = IMAX;
        MOVW      AX, #0xEA60        ;; 1 cycle
        MOVW      N:_curr+12, AX     ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+14, AX     ;; 1 cycle
// 1275                 vol.Rph.rms = VREF;
        MOVW      AX, #0x5DC0        ;; 1 cycle
        MOVW      N:_vol, AX         ;; 1 cycle
        BR        N:??power_filter_124  ;; 3 cycles
        ; ------------------------------------- Block: 17 cycles
// 1276             }
// 1277         }
// 1278         else
// 1279         {
// 1280 //            r_phase.active.acc_cnt = 0;
// 1281 //            r_phase.reactive.acc_cnt = 0;
// 1282 //            r_phase.apparent.acc_cnt = 0;
// 1283             if(bitIsSet(tpr.magnet.flag,event_f) && (MAG_SWITCH_IMAX == 1))
??power_filter_123:
        MOVW      HL, #LWRD(_tpr+32)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??power_filter_125  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1284             {
// 1285                 NOP();
        NOP                          ;; 1 cycle
        BR        S:??power_filter_126  ;; 3 cycles
        ; ------------------------------------- Block: 4 cycles
// 1286             }
// 1287             else
// 1288             {
// 1289                 if(EMPTY_BUCKET == 1) 
// 1290                 {
// 1291                     r_phase.active.acc_delta_cnts = 0;
??power_filter_125:
        MOVW      HL, #LWRD(_r_phase+88)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 1292                     r_phase.reactive.acc_delta_cnts = 0;
        MOVW      HL, #LWRD(_r_phase+128)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 1293                     r_phase.apparent.acc_delta_cnts = 0;
        MOVW      HL, #LWRD(_r_phase+168)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 1294                     r_phase.fundamental.active.delta_cnts = 0;
        MOVW      HL, #LWRD(_r_phase+200)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        ; ------------------------------------- Block: 36 cycles
// 1295                 }
// 1296             }
// 1297           
// 1298             if(flag_phase_present_r == 0 && flag_min_current_r == 1)      /* when vol <20 and curr < 10 */
??power_filter_126:
        MOV       A, N:_flag_metro2  ;; 1 cycle
        AND       A, #0x11           ;; 1 cycle
        CMP       A, #0x10           ;; 1 cycle
        BNZ       ??power_filter_127  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
// 1299             {
// 1300                 vol.Rph.rms = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_vol, AX         ;; 1 cycle
// 1301                 vol.Rph.dc = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_vol+4, AX       ;; 1 cycle
// 1302                 curr.Rph.rms = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr, AX        ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+2, AX      ;; 1 cycle
// 1303                 curr.Rph.dc = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+8, AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+10, AX     ;; 1 cycle
// 1304                 curr.Rph.rms_signed = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+12, AX     ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+14, AX     ;; 1 cycle
// 1305                 r_phase.active.delta_cnts_temp = 0;
        MOVW      HL, #LWRD(_r_phase+104)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 1306                 r_phase.reactive.delta_cnts_temp = 0;
        MOVW      HL, #LWRD(_r_phase+144)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 1307                 r_phase.apparent.delta_cnts_temp = 0;
        MOVW      HL, #LWRD(_r_phase+184)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        BR        N:??power_filter_128  ;; 3 cycles
        ; ------------------------------------- Block: 46 cycles
// 1308             }
// 1309             else if(flag_phase_present_r == 0 && flag_min_current_r == 0) /* when vol <20 and curr >= 10 */
??power_filter_127:
        MOV       A, N:_flag_metro2  ;; 1 cycle
        AND       A, #0x11           ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_129  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
// 1310             {
// 1311                 vol.Rph.rms = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_vol, AX         ;; 1 cycle
// 1312                 vol.Rph.dc = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_vol+4, AX       ;; 1 cycle
// 1313                 //                curr.Rph.rms = 0;
// 1314                 //                curr.Rph.dc = 0;
// 1315                 //                curr.Rph.rms_signed = 0;
// 1316                 r_phase.active.delta_cnts_temp = 0;
        MOVW      HL, #LWRD(_r_phase+104)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 1317                 r_phase.reactive.delta_cnts_temp = 0;
        MOVW      HL, #LWRD(_r_phase+144)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 1318                 r_phase.apparent.delta_cnts_temp = 0;
        MOVW      HL, #LWRD(_r_phase+184)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        BR        N:??power_filter_128  ;; 3 cycles
        ; ------------------------------------- Block: 34 cycles
// 1319             }
// 1320             else if(flag_phase_present_r == 1 && flag_min_current_r == 1) /* when vol > 20 and curr < 10 */
??power_filter_129:
        MOV       A, N:_flag_metro2  ;; 1 cycle
        AND       A, #0x11           ;; 1 cycle
        CMP       A, #0x11           ;; 1 cycle
        SKZ                          ;; 4 cycles
        BR        N:??power_filter_128  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
// 1321             {
// 1322                 if(bitIsSet(tpr.magnet.flag,event_f) && (MAG_SWITCH_IMAX == 1))
        MOVW      HL, #LWRD(_tpr+32)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??power_filter_130  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1323                 {
// 1324                     curr.Rph.rms = IMAX;
        MOVW      AX, #0xEA60        ;; 1 cycle
        MOVW      N:_curr, AX        ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+2, AX      ;; 1 cycle
// 1325                     curr.Rph.dc = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+8, AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+10, AX     ;; 1 cycle
// 1326                     curr.Rph.rms_signed = IMAX;
        MOVW      AX, #0xEA60        ;; 1 cycle
        MOVW      N:_curr+12, AX     ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+14, AX     ;; 1 cycle
// 1327                     vol.Rph.rms = VREF;
        MOVW      AX, #0x5DC0        ;; 1 cycle
        MOVW      N:_vol, AX         ;; 1 cycle
        BR        S:??power_filter_128  ;; 3 cycles
        ; ------------------------------------- Block: 17 cycles
// 1328                 }
// 1329                 else
// 1330                 {
// 1331                     curr.Rph.rms = 0;
??power_filter_130:
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr, AX        ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+2, AX      ;; 1 cycle
// 1332                     curr.Rph.dc = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+8, AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+10, AX     ;; 1 cycle
// 1333                     curr.Rph.rms_signed = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+12, AX     ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+14, AX     ;; 1 cycle
// 1334                     r_phase.active.delta_cnts_temp = 0;
        MOVW      HL, #LWRD(_r_phase+104)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 1335                     r_phase.reactive.delta_cnts_temp = 0;
        MOVW      HL, #LWRD(_r_phase+144)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 1336                     r_phase.apparent.delta_cnts_temp = 0;
        MOVW      HL, #LWRD(_r_phase+184)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        ; ------------------------------------- Block: 39 cycles
// 1337                 }
// 1338             }
// 1339             flag_Rph_active = IMPORT;
??power_filter_128:
        CLR1      N:_flag_quadrant.0  ;; 2 cycles
// 1340             flag_Rph_reactive = IMPORT;
        CLR1      N:_flag_quadrant.1  ;; 2 cycles
// 1341             r_phase.active.delta_cnts = r_phase.active.delta_cnts_temp;
        MOVW      HL, #LWRD(_r_phase+96)  ;; 1 cycle
        MOVW      DE, #LWRD(_r_phase+104)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 1342             r_phase.reactive.delta_cnts = r_phase.reactive.delta_cnts_temp;
        MOVW      HL, #LWRD(_r_phase+136)  ;; 1 cycle
        MOVW      DE, #LWRD(_r_phase+144)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 1343             r_phase.apparent.delta_cnts = r_phase.apparent.delta_cnts_temp;
        MOVW      HL, #LWRD(_r_phase+176)  ;; 1 cycle
        MOVW      DE, #LWRD(_r_phase+184)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 1344             
// 1345             r_phase.fundamental.active.delta_cnts = r_phase.active.delta_cnts * 1000;
        MOVW      DE, #LWRD(__Constant_3e8_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_r_phase+96)  ;; 1 cycle
        MOVW      AX, #LWRD(_r_phase+200)  ;; 1 cycle
          CFI FunCall __Mul64
        CALL      __Mul64            ;; 3 cycles
// 1346             r_phase.fundamental.active.delta_cnts /= (1000+thd.Rph.correction);
        MOV       X, N:_thd+16       ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        ADDW      AX, #0x3E8         ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+30
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
          CFI FunCall __L2LLS
        CALL      __L2LLS            ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      BC, #LWRD(_r_phase+200)  ;; 1 cycle
        MOVW      AX, #LWRD(_r_phase+200)  ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+28
        ; ------------------------------------- Block: 62 cycles
// 1347         }
// 1348         
// 1349          
// 1350         /* Calculating powers */
// 1351         power.Rph.active = r_phase.active.delta_cnts / 100000;
??power_filter_124:
        MOVW      DE, #LWRD(__Constant_186a0_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_r_phase+96)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __LLU2L
        CALL      __LLU2L            ;; 3 cycles
        MOVW      N:_power, AX       ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_power+2, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1352         power.Rph.reactive = r_phase.reactive.delta_cnts / 100000;
        MOVW      DE, #LWRD(__Constant_186a0_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_r_phase+136)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __LLU2L
        CALL      __LLU2L            ;; 3 cycles
        MOVW      N:_power+4, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_power+6, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1353         power.Rph.apparent =  r_phase.apparent.delta_cnts / 100000;
        MOVW      DE, #LWRD(__Constant_186a0_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_r_phase+176)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __LLU2L
        CALL      __LLU2L            ;; 3 cycles
        MOVW      N:_power+8, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_power+10, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1354         pf.Rph = cal_pf(power.Rph.active,power.Rph.apparent);
        MOVW      AX, N:_power+10    ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+30
        MOVW      AX, N:_power+8     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
        MOVW      BC, N:_power+2     ;; 1 cycle
        MOVW      AX, N:_power       ;; 1 cycle
          CFI FunCall _cal_pf
        CALL      _cal_pf            ;; 3 cycles
        MOVW      N:_pf, AX          ;; 1 cycle
// 1355         
// 1356         /* Quadrant */
// 1357         quadrant.Rph = get_quadrant(flag_Rph_active,flag_Rph_reactive);
        MOVW      HL, #LWRD(_flag_quadrant)  ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOV1      CY, [HL].1         ;; 1 cycle
        ROLC      A, 0x1             ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOVW      HL, #LWRD(_flag_quadrant)  ;; 1 cycle
        MOV       A, [HL]            ;; 1 cycle
        AND       A, #0x1            ;; 1 cycle
          CFI FunCall _get_quadrant
        CALL      _get_quadrant      ;; 3 cycles
        MOV       N:_quadrant, A     ;; 1 cycle
// 1358         
// 1359         if(METERING_MODE == NET)
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+28
        CMP       N:_METERING_MODE, #0x1  ;; 1 cycle
        SKZ                          ;; 4 cycles
        BR        N:??power_filter_131  ;; 4 cycles
        ; ------------------------------------- Block: 70 cycles
// 1360         {
// 1361             /* Active related */
// 1362             if(quadrant.Rph == Q1 || quadrant.Rph == Q4)   
        CMP       N:_quadrant, #0x1  ;; 1 cycle
        BZ        ??power_filter_132  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        CMP       N:_quadrant, #0x4  ;; 1 cycle
        BNZ       ??power_filter_133  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 1363             {
// 1364                 power.Rph.active_signed = power.Rph.active;
??power_filter_132:
        MOVW      BC, N:_power+2     ;; 1 cycle
        MOVW      AX, N:_power       ;; 1 cycle
        MOVW      N:_power+12, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_power+14, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1365                 curr.Rph.rms_signed = curr.Rph.rms;
        MOVW      BC, N:_curr+2      ;; 1 cycle
        MOVW      AX, N:_curr        ;; 1 cycle
        MOVW      N:_curr+12, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_curr+14, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1366                 pf.Rph_signed = pf.Rph;
        MOVW      AX, N:_pf          ;; 1 cycle
        MOVW      N:_pf+8, AX        ;; 1 cycle
        BR        S:??power_filter_134  ;; 3 cycles
        ; ------------------------------------- Block: 17 cycles
// 1367             }
// 1368             else                                            
// 1369             {
// 1370                 power.Rph.active_signed = -power.Rph.active;
??power_filter_133:
        MOVW      BC, N:_power+2     ;; 1 cycle
        MOVW      AX, N:_power       ;; 1 cycle
          CFI FunCall ?L_NEG_L03
        CALL      N:?L_NEG_L03       ;; 3 cycles
        MOVW      N:_power+12, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_power+14, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1371                 curr.Rph.rms_signed = -curr.Rph.rms;
        MOVW      BC, N:_curr+2      ;; 1 cycle
        MOVW      AX, N:_curr        ;; 1 cycle
          CFI FunCall ?L_NEG_L03
        CALL      N:?L_NEG_L03       ;; 3 cycles
        MOVW      N:_curr+12, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_curr+14, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1372                 pf.Rph_signed = -pf.Rph;
        MOVW      AX, N:_pf          ;; 1 cycle
        XOR       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        XOR       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        INCW      AX                 ;; 1 cycle
        MOVW      N:_pf+8, AX        ;; 1 cycle
        ; ------------------------------------- Block: 25 cycles
// 1373             }
// 1374             
// 1375             /* Reactive related */
// 1376             if(quadrant.Rph == Q1 || quadrant.Rph == Q2)   
??power_filter_134:
        CMP       N:_quadrant, #0x1  ;; 1 cycle
        BZ        ??power_filter_135  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        CMP       N:_quadrant, #0x2  ;; 1 cycle
        BNZ       ??power_filter_136  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 1377             {
// 1378                 power.Rph.reactive_signed = power.Rph.reactive;
??power_filter_135:
        MOVW      BC, N:_power+6     ;; 1 cycle
        MOVW      AX, N:_power+4     ;; 1 cycle
        MOVW      N:_power+16, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_power+18, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        BR        N:??power_filter_137  ;; 3 cycles
        ; ------------------------------------- Block: 9 cycles
// 1379             }
// 1380             else                                         
// 1381             {
// 1382                 power.Rph.reactive_signed = -power.Rph.reactive;
??power_filter_136:
        MOVW      BC, N:_power+6     ;; 1 cycle
        MOVW      AX, N:_power+4     ;; 1 cycle
          CFI FunCall ?L_NEG_L03
        CALL      N:?L_NEG_L03       ;; 3 cycles
        MOVW      N:_power+16, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_power+18, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        BR        S:??power_filter_137  ;; 3 cycles
        ; ------------------------------------- Block: 12 cycles
// 1383             }
// 1384         }
// 1385         else 
// 1386         {
// 1387             /* Active related */
// 1388             power.Rph.active_signed = power.Rph.active;
??power_filter_131:
        MOVW      BC, N:_power+2     ;; 1 cycle
        MOVW      AX, N:_power       ;; 1 cycle
        MOVW      N:_power+12, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_power+14, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1389             if(flag_Rph_active == EXPORT)
        MOVW      HL, #LWRD(_flag_quadrant)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??power_filter_138  ;; 4 cycles
        ; ------------------------------------- Block: 12 cycles
// 1390             {
// 1391                 curr.Rph.rms_signed = -curr.Rph.rms;
        MOVW      BC, N:_curr+2      ;; 1 cycle
        MOVW      AX, N:_curr        ;; 1 cycle
          CFI FunCall ?L_NEG_L03
        CALL      N:?L_NEG_L03       ;; 3 cycles
        MOVW      N:_curr+12, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_curr+14, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        BR        S:??power_filter_139  ;; 3 cycles
        ; ------------------------------------- Block: 12 cycles
// 1392             }
// 1393             else
// 1394             {
// 1395                 curr.Rph.rms_signed = curr.Rph.rms;
??power_filter_138:
        MOVW      BC, N:_curr+2      ;; 1 cycle
        MOVW      AX, N:_curr        ;; 1 cycle
        MOVW      N:_curr+12, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_curr+14, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
// 1396             }
// 1397             
// 1398             /* Reactive related */
// 1399             if(quadrant.Rph == Q1 || quadrant.Rph == Q3)   
??power_filter_139:
        CMP       N:_quadrant, #0x1  ;; 1 cycle
        BZ        ??power_filter_140  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        CMP       N:_quadrant, #0x3  ;; 1 cycle
        BNZ       ??power_filter_141  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 1400             {
// 1401                 power.Rph.reactive_signed = power.Rph.reactive;
??power_filter_140:
        MOVW      BC, N:_power+6     ;; 1 cycle
        MOVW      AX, N:_power+4     ;; 1 cycle
        MOVW      N:_power+16, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_power+18, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1402                 pf.Rph_signed = pf.Rph;
        MOVW      AX, N:_pf          ;; 1 cycle
        MOVW      N:_pf+8, AX        ;; 1 cycle
        BR        S:??power_filter_137  ;; 3 cycles
        ; ------------------------------------- Block: 11 cycles
// 1403             }
// 1404             else                                         
// 1405             {
// 1406                 power.Rph.reactive_signed = -power.Rph.reactive;
??power_filter_141:
        MOVW      BC, N:_power+6     ;; 1 cycle
        MOVW      AX, N:_power+4     ;; 1 cycle
          CFI FunCall ?L_NEG_L03
        CALL      N:?L_NEG_L03       ;; 3 cycles
        MOVW      N:_power+16, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_power+18, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1407                 pf.Rph_signed = -pf.Rph;
        MOVW      AX, N:_pf          ;; 1 cycle
        XOR       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        XOR       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        INCW      AX                 ;; 1 cycle
        MOVW      N:_pf+8, AX        ;; 1 cycle
        ; ------------------------------------- Block: 16 cycles
// 1408             }
// 1409         }
// 1410 
// 1411         /* Phase angle */
// 1412         //        angle.Rph = cal_angle(power.Rph.active,power.Rph.reactive,power.Rph.apparent,flag_Rph_active,flag_Rph_reactive,1);
// 1413         
// 1414         if(flag_mag_update_metro_par == 1)
??power_filter_137:
        MOVW      HL, #LWRD(_flag_mag)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        SKNC                         ;; 1 cycle
        ; ------------------------------------- Block: 3 cycles
// 1415         {
// 1416             flag_mag_r_updated = 1;
        SET1      N:_flag_mag.1      ;; 2 cycles
          CFI FunCall _metrology_all_phase_calculation
        ; ------------------------------------- Block: 2 cycles
// 1417         }
// 1418         /* calculation for all phase variables */
// 1419         metrology_all_phase_calculation();
??metrology_process_2:
        CALL      _metrology_all_phase_calculation  ;; 3 cycles
// 1420         
// 1421         /* Calibration */
// 1422         if(cal_done_f != 1)
        CMP       N:_cal_done_f, #0x1  ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??power_filter_142  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
// 1423         {
// 1424             if(vol.Rph.rms >= CAL_VOL_MIN && vol.Rph.rms <= CAL_VOL_MAX && 
// 1425                curr.Rph.rms >= CAL_CURR_MIN && curr.Rph.rms <= CAL_CURR_MAX)
        MOVW      AX, N:_vol         ;; 1 cycle
        CMPW      AX, #0x55F0        ;; 1 cycle
        BC        ??power_filter_143  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, N:_vol         ;; 1 cycle
        CMPW      AX, #0x6591        ;; 1 cycle
        BNC       ??power_filter_143  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      BC, N:_curr+2      ;; 1 cycle
        MOVW      AX, N:_curr        ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
        CMPW      AX, #0x1F40        ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??metrology_process_3:
        BC        ??power_filter_143  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
        MOVW      BC, N:_curr+2      ;; 1 cycle
        MOVW      AX, N:_curr        ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
        CMPW      AX, #0x2EE1        ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??metrology_process_4:
        BNC       ??power_filter_143  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
// 1426             {
// 1427                 flag_calibration_r_vi_ok = 1;
        SET1      N:_flag_cal.0      ;; 2 cycles
        BR        S:??power_filter_144  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 1428             }
// 1429             else
// 1430             {
// 1431                 flag_calibration_r_vi_ok = 0;
??power_filter_143:
        CLR1      N:_flag_cal.0      ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
// 1432             }
// 1433             if(power.Rph.active >= CAL_POWER_MIN && power.Rph.active <= CAL_POWER_MAX &&
// 1434                freq.Rph >= CAL_FREQ_MIN && freq.Rph <= CAL_FREQ_MAX && 
// 1435                    pf.Rph >= CAL_PF_MIN)
??power_filter_144:
        MOVW      BC, N:_power+2     ;; 1 cycle
        MOVW      AX, N:_power       ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
        CMPW      AX, #0x44C0        ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??metrology_process_5:
        BC        ??power_filter_145  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
        MOVW      BC, N:_power+2     ;; 1 cycle
        MOVW      AX, N:_power       ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
        CMPW      AX, #0x79E1        ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??metrology_process_6:
        BNC       ??power_filter_145  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
        MOVW      AX, N:_freq        ;; 1 cycle
        CMPW      AX, #0xC288        ;; 1 cycle
        BC        ??power_filter_145  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, N:_freq        ;; 1 cycle
        CMPW      AX, #0xC419        ;; 1 cycle
        BNC       ??power_filter_145  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, N:_pf          ;; 1 cycle
        CMPW      AX, #0x384         ;; 1 cycle
        BC        ??power_filter_145  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1436             {
// 1437                 flag_calibration_r_pf_freq_pow_ok = 1;
        SET1      N:_flag_calibration.2  ;; 2 cycles
        BR        S:??power_filter_146  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 1438             }
// 1439             else
// 1440             {
// 1441                 flag_calibration_r_pf_freq_pow_ok = 0;
??power_filter_145:
        CLR1      N:_flag_calibration.2  ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
// 1442             }
// 1443             if(flag_Rph_active == IMPORT)
??power_filter_146:
        MOVW      HL, #LWRD(_flag_quadrant)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BC        ??power_filter_147  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1444             {
// 1445                 flag_calibration_r_ct_ok = 1;
        SET1      N:_flag_calibration.5  ;; 2 cycles
        BR        S:??power_filter_148  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 1446             }
// 1447             else
// 1448             {
// 1449                 flag_calibration_r_ct_ok = 0;
??power_filter_147:
        CLR1      N:_flag_calibration.5  ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
// 1450             }
// 1451             
// 1452             if(cal_RPh.l_shift.active.acc_cnt < 0)
??power_filter_148:
        MOVW      BC, #LWRD(__Constant_0_0)  ;; 1 cycle
        MOVW      AX, #LWRD(_cal_RPh+18)  ;; 1 cycle
          CFI FunCall __CmpGes64
        CALL      __CmpGes64         ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_149  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
// 1453             {
// 1454                 temp_us64 = ~cal_RPh.l_shift.active.acc_cnt + 1;
        MOVW      BC, #LWRD(_cal_RPh+18)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
          CFI FunCall __Not64
        CALL      __Not64            ;; 3 cycles
        MOVW      DE, #LWRD(__Constant_1_0)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Add64
        CALL      __Add64            ;; 3 cycles
        MOVW      HL, #LWRD(_temp_us64)  ;; 1 cycle
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
        BR        S:??power_filter_150  ;; 3 cycles
        ; ------------------------------------- Block: 28 cycles
// 1455             }
// 1456             else
// 1457             {
// 1458                 temp_us64 = cal_RPh.l_shift.active.acc_cnt;
??power_filter_149:
        MOVW      HL, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      DE, #LWRD(_cal_RPh+18)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        ; ------------------------------------- Block: 10 cycles
// 1459             }
// 1460             
// 1461             temp_us64 *= cal_coeff.Rph.power;
??power_filter_150:
        MOVW      AX, N:_cal_coeff+10  ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+30
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
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
// 1462             temp_us64 /= r_phase.no_of_samples;
        MOVW      AX, N:_r_phase     ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+34
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+36
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
// 1463             temp_us64 /= 100000;
        MOVW      DE, #LWRD(__Constant_186a0_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
// 1464             cal_RPh.l_shift.active.power = temp_us64;
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __LLU2L
        CALL      __LLU2L            ;; 3 cycles
        MOVW      N:_cal_RPh+14, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_cal_RPh+16, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1465             
// 1466             if(cal_RPh.l_shift.active.acc_cnt < 0)
        MOVW      BC, #LWRD(__Constant_0_0)  ;; 1 cycle
        MOVW      AX, #LWRD(_cal_RPh+18)  ;; 1 cycle
          CFI FunCall __CmpGes64
        CALL      __CmpGes64         ;; 3 cycles
        ADDW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+28
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_151  ;; 4 cycles
        ; ------------------------------------- Block: 59 cycles
// 1467             {
// 1468                 cal_RPh.l_shift.active.power = ~cal_RPh.l_shift.active.power + 1;
        MOVW      BC, N:_cal_RPh+16  ;; 1 cycle
        MOVW      AX, N:_cal_RPh+14  ;; 1 cycle
          CFI FunCall ?L_NOT_L03
        CALL      N:?L_NOT_L03       ;; 3 cycles
        ADDW      AX, #0x1           ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_cal_RPh+14, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_cal_RPh+16, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ; ------------------------------------- Block: 18 cycles
// 1469             }
// 1470             
// 1471             if(cal_RPh.r_shift.active.acc_cnt < 0)
??power_filter_151:
        MOVW      BC, #LWRD(__Constant_0_0)  ;; 1 cycle
        MOVW      AX, #LWRD(_cal_RPh+62)  ;; 1 cycle
          CFI FunCall __CmpGes64
        CALL      __CmpGes64         ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_152  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
// 1472             {
// 1473                 temp_us64 = ~cal_RPh.r_shift.active.acc_cnt + 1;
        MOVW      BC, #LWRD(_cal_RPh+62)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
          CFI FunCall __Not64
        CALL      __Not64            ;; 3 cycles
        MOVW      DE, #LWRD(__Constant_1_0)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Add64
        CALL      __Add64            ;; 3 cycles
        MOVW      HL, #LWRD(_temp_us64)  ;; 1 cycle
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
        BR        S:??power_filter_153  ;; 3 cycles
        ; ------------------------------------- Block: 28 cycles
// 1474             }
// 1475             else
// 1476             {
// 1477                 temp_us64 = cal_RPh.r_shift.active.acc_cnt;
??power_filter_152:
        MOVW      HL, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      DE, #LWRD(_cal_RPh+62)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        ; ------------------------------------- Block: 10 cycles
// 1478             }
// 1479             
// 1480             temp_us64 *= cal_coeff.Rph.power;
??power_filter_153:
        MOVW      AX, N:_cal_coeff+10  ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+30
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
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
// 1481             temp_us64 /= r_phase.no_of_samples;
        MOVW      AX, N:_r_phase     ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+34
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+36
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
// 1482             temp_us64 /= 100000;
        MOVW      DE, #LWRD(__Constant_186a0_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
// 1483             cal_RPh.r_shift.active.power = temp_us64;
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __LLU2L
        CALL      __LLU2L            ;; 3 cycles
        MOVW      N:_cal_RPh+58, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_cal_RPh+60, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1484             if(cal_RPh.r_shift.active.acc_cnt < 0)
        MOVW      BC, #LWRD(__Constant_0_0)  ;; 1 cycle
        MOVW      AX, #LWRD(_cal_RPh+62)  ;; 1 cycle
          CFI FunCall __CmpGes64
        CALL      __CmpGes64         ;; 3 cycles
        ADDW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+28
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_154  ;; 4 cycles
        ; ------------------------------------- Block: 59 cycles
// 1485             {
// 1486                 cal_RPh.r_shift.active.power = ~cal_RPh.r_shift.active.power + 1;
        MOVW      BC, N:_cal_RPh+60  ;; 1 cycle
        MOVW      AX, N:_cal_RPh+58  ;; 1 cycle
          CFI FunCall ?L_NOT_L03
        CALL      N:?L_NOT_L03       ;; 3 cycles
        ADDW      AX, #0x1           ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_cal_RPh+58, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_cal_RPh+60, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ; ------------------------------------- Block: 18 cycles
// 1487             }
// 1488             
// 1489             cal_RPh.angle_active = cal_angle_calculate_act(cal_RPh.l_shift.active.power,cal_RPh.r_shift.active.power);
??power_filter_154:
        MOVW      AX, N:_cal_RPh+60  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+30
        MOVW      AX, N:_cal_RPh+58  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
        MOVW      BC, N:_cal_RPh+16  ;; 1 cycle
        MOVW      AX, N:_cal_RPh+14  ;; 1 cycle
          CFI FunCall _cal_angle_calculate_act
        CALL      _cal_angle_calculate_act  ;; 3 cycles
        MOVW      N:_cal_RPh+6, AX   ;; 1 cycle
// 1490             
// 1491             if(cal_RPh.l_shift.reactive.acc_cnt < 0)
        MOVW      BC, #LWRD(__Constant_0_0)  ;; 1 cycle
        MOVW      AX, #LWRD(_cal_RPh+38)  ;; 1 cycle
          CFI FunCall __CmpGes64
        CALL      __CmpGes64         ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+28
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_155  ;; 4 cycles
        ; ------------------------------------- Block: 21 cycles
// 1492             {
// 1493                 temp_us64 = ~cal_RPh.l_shift.reactive.acc_cnt + 1;
        MOVW      BC, #LWRD(_cal_RPh+38)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
          CFI FunCall __Not64
        CALL      __Not64            ;; 3 cycles
        MOVW      DE, #LWRD(__Constant_1_0)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Add64
        CALL      __Add64            ;; 3 cycles
        MOVW      HL, #LWRD(_temp_us64)  ;; 1 cycle
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
        BR        S:??power_filter_156  ;; 3 cycles
        ; ------------------------------------- Block: 28 cycles
// 1494             }
// 1495             else
// 1496             {
// 1497                 temp_us64 = cal_RPh.l_shift.reactive.acc_cnt;
??power_filter_155:
        MOVW      HL, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      DE, #LWRD(_cal_RPh+38)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        ; ------------------------------------- Block: 10 cycles
// 1498             }
// 1499             
// 1500             temp_us64 *= cal_coeff.Rph.power;
??power_filter_156:
        MOVW      AX, N:_cal_coeff+10  ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+30
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
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
// 1501             temp_us64 /= r_phase.no_of_samples;
        MOVW      AX, N:_r_phase     ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+34
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+36
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
// 1502             temp_us64 /= 100000;
        MOVW      DE, #LWRD(__Constant_186a0_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
// 1503             cal_RPh.l_shift.reactive.power = temp_us64;    
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __LLU2L
        CALL      __LLU2L            ;; 3 cycles
        MOVW      N:_cal_RPh+34, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_cal_RPh+36, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1504             if(cal_RPh.l_shift.reactive.acc_cnt < 0)
        MOVW      BC, #LWRD(__Constant_0_0)  ;; 1 cycle
        MOVW      AX, #LWRD(_cal_RPh+38)  ;; 1 cycle
          CFI FunCall __CmpGes64
        CALL      __CmpGes64         ;; 3 cycles
        ADDW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+28
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_157  ;; 4 cycles
        ; ------------------------------------- Block: 59 cycles
// 1505             {
// 1506                 cal_RPh.l_shift.reactive.power = ~cal_RPh.l_shift.reactive.power + 1;
        MOVW      BC, N:_cal_RPh+36  ;; 1 cycle
        MOVW      AX, N:_cal_RPh+34  ;; 1 cycle
          CFI FunCall ?L_NOT_L03
        CALL      N:?L_NOT_L03       ;; 3 cycles
        ADDW      AX, #0x1           ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_cal_RPh+34, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_cal_RPh+36, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ; ------------------------------------- Block: 18 cycles
// 1507             }
// 1508             
// 1509             if(cal_RPh.r_shift.reactive.acc_cnt < 0)
??power_filter_157:
        MOVW      BC, #LWRD(__Constant_0_0)  ;; 1 cycle
        MOVW      AX, #LWRD(_cal_RPh+82)  ;; 1 cycle
          CFI FunCall __CmpGes64
        CALL      __CmpGes64         ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_158  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
// 1510             {
// 1511                 temp_us64 = ~cal_RPh.r_shift.reactive.acc_cnt + 1;
        MOVW      BC, #LWRD(_cal_RPh+82)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
          CFI FunCall __Not64
        CALL      __Not64            ;; 3 cycles
        MOVW      DE, #LWRD(__Constant_1_0)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Add64
        CALL      __Add64            ;; 3 cycles
        MOVW      HL, #LWRD(_temp_us64)  ;; 1 cycle
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
        BR        S:??power_filter_159  ;; 3 cycles
        ; ------------------------------------- Block: 28 cycles
// 1512             }
// 1513             else
// 1514             {
// 1515                 temp_us64 = cal_RPh.r_shift.reactive.acc_cnt;
??power_filter_158:
        MOVW      HL, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      DE, #LWRD(_cal_RPh+82)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        ; ------------------------------------- Block: 10 cycles
// 1516             }
// 1517             
// 1518             temp_us64 *= cal_coeff.Rph.power;
??power_filter_159:
        MOVW      AX, N:_cal_coeff+10  ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+30
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
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
// 1519             temp_us64 /= r_phase.no_of_samples;
        MOVW      AX, N:_r_phase     ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+34
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+36
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
// 1520             temp_us64 /= 100000;
        MOVW      DE, #LWRD(__Constant_186a0_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
// 1521             cal_RPh.r_shift.reactive.power = temp_us64;
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __LLU2L
        CALL      __LLU2L            ;; 3 cycles
        MOVW      N:_cal_RPh+78, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_cal_RPh+80, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1522             if(cal_RPh.r_shift.reactive.acc_cnt < 0)
        MOVW      BC, #LWRD(__Constant_0_0)  ;; 1 cycle
        MOVW      AX, #LWRD(_cal_RPh+82)  ;; 1 cycle
          CFI FunCall __CmpGes64
        CALL      __CmpGes64         ;; 3 cycles
        ADDW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+28
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_160  ;; 4 cycles
        ; ------------------------------------- Block: 59 cycles
// 1523             {
// 1524                 cal_RPh.r_shift.reactive.power = ~cal_RPh.r_shift.reactive.power + 1;
        MOVW      BC, N:_cal_RPh+80  ;; 1 cycle
        MOVW      AX, N:_cal_RPh+78  ;; 1 cycle
          CFI FunCall ?L_NOT_L03
        CALL      N:?L_NOT_L03       ;; 3 cycles
        ADDW      AX, #0x1           ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_cal_RPh+78, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_cal_RPh+80, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ; ------------------------------------- Block: 18 cycles
// 1525             }
// 1526             
// 1527             cal_RPh.angle_reactive = cal_angle_calculate_act(cal_RPh.l_shift.reactive.power,cal_RPh.r_shift.reactive.power);
??power_filter_160:
        MOVW      AX, N:_cal_RPh+80  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+30
        MOVW      AX, N:_cal_RPh+78  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
        MOVW      BC, N:_cal_RPh+36  ;; 1 cycle
        MOVW      AX, N:_cal_RPh+34  ;; 1 cycle
          CFI FunCall _cal_angle_calculate_act
        CALL      _cal_angle_calculate_act  ;; 3 cycles
        MOVW      N:_cal_RPh+8, AX   ;; 1 cycle
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+28
        BR        S:??power_filter_98  ;; 3 cycles
        ; ------------------------------------- Block: 14 cycles
// 1528         }
// 1529         else
// 1530         {
// 1531             flag_calibration_r_vi_ok = 0;
??power_filter_142:
        CLR1      N:_flag_cal.0      ;; 2 cycles
// 1532             flag_calibration_r_pf_freq_pow_ok = 0;
        CLR1      N:_flag_calibration.2  ;; 2 cycles
// 1533             flag_calibration_r_ct_ok = 0;
        CLR1      N:_flag_calibration.5  ;; 2 cycles
        ; ------------------------------------- Block: 6 cycles
// 1534         }
// 1535     }
// 1536     /************************************************************************************************
// 1537     *****************************************   Y PHASE    ******************************************
// 1538     ************************************************************************************************/
// 1539     if(flag_metrology_process_y == 1 && y_phase.no_of_samples != 0)
??power_filter_98:
        MOVW      HL, #LWRD(_flag_metrology)  ;; 1 cycle
        MOV1      CY, [HL].1         ;; 1 cycle
        SKC                          ;; 4 cycles
        BR        N:??power_filter_161  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        CLRW      AX                 ;; 1 cycle
        CMPW      AX, N:_y_phase     ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??power_filter_161  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1540     {
// 1541         flag_metrology_process_y = 0;
        CLR1      N:_flag_metrology.1  ;; 2 cycles
// 1542         
// 1543         /* Current */ 
// 1544         curr.Yph.rms = calculate_irms(y_phase.curr.rms_acc_cnt,cal_coeff.Yph.curr,y_phase.no_of_samples);
        MOVW      AX, N:_y_phase     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+30
        MOVW      AX, N:_cal_coeff+20  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
        MOVW      HL, #LWRD(_y_phase+56)  ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+40
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
          CFI FunCall _calculate_irms
        CALL      _calculate_irms    ;; 3 cycles
        MOVW      N:_curr+16, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_curr+18, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1545         curr.Yph.dc = calculate_irms(y_phase.curr.offset_acc_cnt,cal_coeff.Yph.curr,y_phase.no_of_samples);
        MOVW      AX, N:_y_phase+50  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+42
        MOVW      AX, N:_y_phase+48  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+44
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
          CFI FunCall __L2LLS
        CALL      __L2LLS            ;; 3 cycles
        MOVW      AX, N:_y_phase     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+46
        MOVW      AX, N:_cal_coeff+20  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+48
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x14          ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+56
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
          CFI FunCall _calculate_irms
        CALL      _calculate_irms    ;; 3 cycles
        MOVW      N:_curr+24, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_curr+26, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1546         y_phase.curr.dc_offset = y_phase.curr.offset_acc_cnt / y_phase.no_of_samples;
        MOVW      AX, N:_y_phase     ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+58
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+60
        MOVW      BC, N:_y_phase+50  ;; 1 cycle
        MOVW      AX, N:_y_phase+48  ;; 1 cycle
          CFI FunCall ?SL_DIV_L03
        CALL      N:?SL_DIV_L03      ;; 3 cycles
        MOVW      N:_y_phase+46, AX  ;; 1 cycle
// 1547         
// 1548         /* Voltage */
// 1549         vol.Yph.rms = calculate_vrms(y_phase.vol.rms_acc_cnt,cal_coeff.Yph.vol,y_phase.no_of_samples);
        MOVW      AX, N:_y_phase     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+62
        MOVW      AX, N:_cal_coeff+18  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+64
        MOVW      HL, #LWRD(_y_phase+26)  ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+72
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
          CFI FunCall _calculate_vrms
        CALL      _calculate_vrms    ;; 3 cycles
        ADDW      SP, #0x2C          ;; 1 cycle
          CFI CFA SP+28
        MOVW      N:_vol+6, AX       ;; 1 cycle
// 1550         vol.Yph.dc = calculate_vrms(y_phase.vol.offset_acc_cnt,cal_coeff.Yph.vol,y_phase.no_of_samples);
        MOVW      AX, N:_y_phase+20  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+30
        MOVW      AX, N:_y_phase+18  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
          CFI FunCall __L2LLS
        CALL      __L2LLS            ;; 3 cycles
        MOVW      AX, N:_y_phase     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+34
        MOVW      AX, N:_cal_coeff+18  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+36
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+44
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
          CFI FunCall _calculate_vrms
        CALL      _calculate_vrms    ;; 3 cycles
        MOVW      N:_vol+10, AX      ;; 1 cycle
// 1551         y_phase.vol.dc_offset = y_phase.vol.offset_acc_cnt / y_phase.no_of_samples;
        MOVW      AX, N:_y_phase     ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+46
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+48
        MOVW      BC, N:_y_phase+20  ;; 1 cycle
        MOVW      AX, N:_y_phase+18  ;; 1 cycle
          CFI FunCall ?SL_DIV_L03
        CALL      N:?SL_DIV_L03      ;; 3 cycles
        MOVW      N:_y_phase+14, AX  ;; 1 cycle
// 1552         
// 1553         /* detecting low voltage or low current*/
// 1554         if(vol.Yph.rms >= THR_PHASE_PRESENT_VOL)
        MOVW      AX, N:_vol+6       ;; 1 cycle
        ADDW      SP, #0x14          ;; 1 cycle
          CFI CFA SP+28
        CMPW      AX, #0x7D0         ;; 1 cycle
        BC        ??power_filter_162  ;; 4 cycles
        ; ------------------------------------- Block: 118 cycles
// 1555         {
// 1556             flag_phase_present_y = 1;
        SET1      N:_flag_metro2.1   ;; 2 cycles
        BR        S:??power_filter_163  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 1557         }
// 1558         else
// 1559         {
// 1560             flag_phase_present_y = 0;
??power_filter_162:
        CLR1      N:_flag_metro2.1   ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
// 1561         }  
// 1562         /* detecting healthy voltage for frequency calculation */
// 1563         if(vol.Yph.rms >= THR_FREQ_DECISION)
??power_filter_163:
        MOVW      AX, N:_vol+6       ;; 1 cycle
        CMPW      AX, #0x1D4C        ;; 1 cycle
        BC        ??power_filter_164  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1564         {
// 1565             flag_freq_vol_y = 1;
        SET1      N:_flag_metro3.1   ;; 2 cycles
        BR        S:??power_filter_165  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 1566         }
// 1567         else
// 1568         {
// 1569             flag_freq_vol_y = 0;
??power_filter_164:
        CLR1      N:_flag_metro3.1   ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
// 1570         }  
// 1571         if(curr.Yph.rms < MIN_CURRENT)
??power_filter_165:
        MOVW      BC, N:_curr+18     ;; 1 cycle
        MOVW      AX, N:_curr+16     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
        CMPW      AX, #0xA           ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??metrology_process_7:
        BNC       ??power_filter_166  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
// 1572         {
// 1573             flag_min_current_y = 1;
        SET1      N:_flag_metro2.5   ;; 2 cycles
        BR        S:??power_filter_167  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 1574         }
// 1575         else
// 1576         {
// 1577             flag_min_current_y = 0;
??power_filter_166:
        CLR1      N:_flag_metro2.5   ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
// 1578         }
// 1579         /* Frequency */
// 1580         if(flag_phase_present_y == 1)
??power_filter_167:
        MOVW      HL, #LWRD(_flag_metro2)  ;; 1 cycle
        MOV1      CY, [HL].1         ;; 1 cycle
        BNC       ??power_filter_168  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1581         {
// 1582             freq.Yph = (us16)((us32)195312500 / y_phase.no_of_samples);
        MOVW      DE, N:_y_phase     ;; 1 cycle
        MOVW      HL, #0x0           ;; 1 cycle
        MOVW      AX, #0x3B74        ;; 1 cycle
        MOVW      BC, #0xBA4         ;; 1 cycle
        DIVWU                        ;; 17 cycles
        NOP                          ;; 1 cycle
        MOVW      N:_freq+2, AX      ;; 1 cycle
        BR        S:??power_filter_169  ;; 3 cycles
        ; ------------------------------------- Block: 26 cycles
// 1583         }
// 1584         else
// 1585         {
// 1586             freq.Yph = 0;
??power_filter_168:
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_freq+2, AX      ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
// 1587         }
// 1588         
// 1589         /* Active Energy */
// 1590         if(y_phase.active.acc_cnt < 0)
??power_filter_169:
        MOVW      BC, #LWRD(__Constant_0_0)  ;; 1 cycle
        MOVW      AX, #LWRD(_y_phase+72)  ;; 1 cycle
          CFI FunCall __CmpGes64
        CALL      __CmpGes64         ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_170  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
// 1591         {
// 1592             temp_us64 = ~y_phase.active.acc_cnt + 1;
        MOVW      BC, #LWRD(_y_phase+72)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
          CFI FunCall __Not64
        CALL      __Not64            ;; 3 cycles
        MOVW      DE, #LWRD(__Constant_1_0)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Add64
        CALL      __Add64            ;; 3 cycles
        MOVW      HL, #LWRD(_temp_us64)  ;; 1 cycle
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
// 1593             flag_Yph_active = EXPORT;
        SET1      N:_flag_quadrant.2  ;; 2 cycles
        BR        S:??power_filter_171  ;; 3 cycles
        ; ------------------------------------- Block: 30 cycles
// 1594         }
// 1595         else
// 1596         {
// 1597             temp_us64 = y_phase.active.acc_cnt;
??power_filter_170:
        MOVW      HL, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      DE, #LWRD(_y_phase+72)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 1598             flag_Yph_active = IMPORT;
        CLR1      N:_flag_quadrant.2  ;; 2 cycles
        ; ------------------------------------- Block: 12 cycles
// 1599         }
// 1600         temp_us64 *= cal_coeff.Yph.power;
??power_filter_171:
        MOVW      AX, N:_cal_coeff+22  ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+30
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
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
// 1601         temp_us64 /= y_phase.no_of_samples;
        MOVW      AX, N:_y_phase     ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+34
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+36
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
// 1602         y_phase.active.delta_cnts_temp = temp_us64;
        MOVW      HL, #LWRD(_y_phase+104)  ;; 1 cycle
        MOVW      DE, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 1603         
// 1604         /* Reactive Energy */
// 1605         if(flag_metro_react_cal_method == REACT_CAL_DELAY)
        MOVW      HL, #LWRD(_flag_metro1)  ;; 1 cycle
        MOV1      CY, [HL].3         ;; 1 cycle
        ADDW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+28
        SKNC                         ;; 4 cycles
        BR        N:??power_filter_172  ;; 4 cycles
        ; ------------------------------------- Block: 51 cycles
// 1606         {
// 1607             if(y_phase.reactive.acc_cnt < 0)
        MOVW      BC, #LWRD(__Constant_0_0)  ;; 1 cycle
        MOVW      AX, #LWRD(_y_phase+112)  ;; 1 cycle
          CFI FunCall __CmpGes64
        CALL      __CmpGes64         ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_173  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
// 1608             {
// 1609                 temp_us64 = ~y_phase.reactive.acc_cnt + 1;
        MOVW      BC, #LWRD(_y_phase+112)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
          CFI FunCall __Not64
        CALL      __Not64            ;; 3 cycles
        MOVW      DE, #LWRD(__Constant_1_0)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Add64
        CALL      __Add64            ;; 3 cycles
        MOVW      HL, #LWRD(_temp_us64)  ;; 1 cycle
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
// 1610                 flag_Yph_reactive = EXPORT;
        SET1      N:_flag_quadrant.3  ;; 2 cycles
        BR        S:??power_filter_174  ;; 3 cycles
        ; ------------------------------------- Block: 30 cycles
// 1611             }
// 1612             else
// 1613             {
// 1614                 temp_us64 = y_phase.reactive.acc_cnt;
??power_filter_173:
        MOVW      HL, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      DE, #LWRD(_y_phase+112)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 1615                 flag_Yph_reactive = IMPORT;
        CLR1      N:_flag_quadrant.3  ;; 2 cycles
        ; ------------------------------------- Block: 12 cycles
// 1616             }
// 1617             temp_us64 *= cal_coeff.Yph.power;
??power_filter_174:
        MOVW      AX, N:_cal_coeff+22  ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+30
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
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
// 1618             temp_us64 /= y_phase.no_of_samples;
        MOVW      AX, N:_y_phase     ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+34
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+36
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
// 1619             y_phase.reactive.delta_cnts_temp = temp_us64;
        MOVW      HL, #LWRD(_y_phase+144)  ;; 1 cycle
        MOVW      DE, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        ADDW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+28
        ; ------------------------------------- Block: 45 cycles
// 1620         }
// 1621         
// 1622         /* Apparent Energy */
// 1623         if(flag_metro_app_cal_method == APP_CAL_VI)
??power_filter_172:
        MOVW      HL, #LWRD(_flag_metro1)  ;; 1 cycle
        MOV1      CY, [HL].1         ;; 1 cycle
        BC        ??power_filter_175  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1624         {
// 1625             y_phase.apparent.delta_cnts_temp = ((us64)curr.Yph.rms * vol.Yph.rms)*10;
        MOVW      AX, N:_curr+18     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+30
        MOVW      AX, N:_curr+16     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
          CFI FunCall __L2LLU
        CALL      __L2LLU            ;; 3 cycles
        MOVW      AX, N:_vol+6       ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+34
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+36
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
          CFI FunCall __L2LLU
        CALL      __L2LLU            ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x18          ;; 1 cycle
          CFI FunCall __Mul64
        CALL      __Mul64            ;; 3 cycles
        MOVW      DE, #LWRD(__Constant_a_0)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x18          ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, #LWRD(_y_phase+184)  ;; 1 cycle
          CFI FunCall __Mul64
        CALL      __Mul64            ;; 3 cycles
        ADDW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+28
        BR        N:??power_filter_176  ;; 3 cycles
        ; ------------------------------------- Block: 41 cycles
// 1626         }
// 1627         else
// 1628         {
// 1629             temp_us64 = (us64)((us64)y_phase.active.delta_cnts_temp/100)*((us64)y_phase.active.delta_cnts_temp/100);
??power_filter_175:
        MOVW      DE, #LWRD(__Constant_64_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_y_phase+104)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
        MOVW      DE, #LWRD(__Constant_64_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_y_phase+104)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __Mul64
        CALL      __Mul64            ;; 3 cycles
// 1630             temp_us64 += (us64)((us64)y_phase.reactive.delta_cnts_temp/100)*((us64)y_phase.reactive.delta_cnts_temp/100);
        MOVW      DE, #LWRD(__Constant_64_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_y_phase+144)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
        MOVW      DE, #LWRD(__Constant_64_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_y_phase+144)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
          CFI FunCall __Mul64
        CALL      __Mul64            ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      BC, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __Add64
        CALL      __Add64            ;; 3 cycles
// 1631             y_phase.apparent.delta_cnts_temp = (us64)sqrt(temp_us64)*100;
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __LLU2F
        CALL      __LLU2F            ;; 3 cycles
          CFI FunCall _sqrt
        CALL      _sqrt              ;; 3 cycles
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+30
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
          CFI FunCall __F2LLU
        CALL      __F2LLU            ;; 3 cycles
        MOVW      DE, #LWRD(__Constant_64_0)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, #LWRD(_y_phase+184)  ;; 1 cycle
          CFI FunCall __Mul64
        CALL      __Mul64            ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+28
        ; ------------------------------------- Block: 76 cycles
// 1632         }
// 1633         if((y_phase.active.delta_cnts_temp > y_phase.apparent.delta_cnts_temp) && (SP_CHECK_ACT_APP == 1) && (ZERORISE_METROLOGY_PAR == 1))
??power_filter_176:
        MOVW      BC, #LWRD(_y_phase+104)  ;; 1 cycle
        MOVW      AX, #LWRD(_y_phase+184)  ;; 1 cycle
          CFI FunCall __CmpGeu64
        CALL      __CmpGeu64         ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_177  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
// 1634         {
// 1635             y_phase.apparent.delta_cnts_temp =  y_phase.active.delta_cnts_temp;
        MOVW      HL, #LWRD(_y_phase+184)  ;; 1 cycle
        MOVW      DE, #LWRD(_y_phase+104)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        ; ------------------------------------- Block: 10 cycles
// 1636         }
// 1637         
// 1638         /* Reactive energy */
// 1639         if(flag_metro_react_cal_method == REACT_CAL_POW_TRIANGLE)
??power_filter_177:
        MOVW      HL, #LWRD(_flag_metro1)  ;; 1 cycle
        MOV1      CY, [HL].3         ;; 1 cycle
        SKC                          ;; 4 cycles
        BR        N:??power_filter_178  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1640         {
// 1641             if(y_phase.reactive.acc_cnt < 0) /* there may be issues in the calculation of quadrants at boundry conditions at specific freq points*/
        MOVW      BC, #LWRD(__Constant_0_0)  ;; 1 cycle
        MOVW      AX, #LWRD(_y_phase+112)  ;; 1 cycle
          CFI FunCall __CmpGes64
        CALL      __CmpGes64         ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_179  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
// 1642             {
// 1643                 flag_Yph_reactive = EXPORT;
        SET1      N:_flag_quadrant.3  ;; 2 cycles
        BR        S:??power_filter_180  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 1644             }
// 1645             else
// 1646             {
// 1647                 flag_Yph_reactive = IMPORT;
??power_filter_179:
        CLR1      N:_flag_quadrant.3  ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
// 1648             }
// 1649             temp_us64 = (us64)((us64)y_phase.apparent.delta_cnts_temp/100)*((us64)y_phase.apparent.delta_cnts_temp/100);
??power_filter_180:
        MOVW      DE, #LWRD(__Constant_64_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_y_phase+184)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
        MOVW      DE, #LWRD(__Constant_64_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_y_phase+184)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __Mul64
        CALL      __Mul64            ;; 3 cycles
// 1650             temp_us64 -= (us64)((us64)y_phase.active.delta_cnts_temp/100)*((us64)y_phase.active.delta_cnts_temp/100);
        MOVW      DE, #LWRD(__Constant_64_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_y_phase+104)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
        MOVW      DE, #LWRD(__Constant_64_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_y_phase+104)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
          CFI FunCall __Mul64
        CALL      __Mul64            ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      BC, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __Sub64
        CALL      __Sub64            ;; 3 cycles
// 1651             y_phase.reactive.delta_cnts_temp = (us64)sqrt(temp_us64)*100;
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __LLU2F
        CALL      __LLU2F            ;; 3 cycles
          CFI FunCall _sqrt
        CALL      _sqrt              ;; 3 cycles
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+30
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
          CFI FunCall __F2LLU
        CALL      __F2LLU            ;; 3 cycles
        MOVW      DE, #LWRD(__Constant_64_0)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, #LWRD(_y_phase+144)  ;; 1 cycle
          CFI FunCall __Mul64
        CALL      __Mul64            ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+28
        ; ------------------------------------- Block: 76 cycles
// 1652         }
// 1653         /* Special checks */
// 1654         /* checks : 
// 1655         1. magnet tamper 
// 1656         2. ZPF
// 1657         3. UPF 
// 1658         4. Low power active
// 1659         5. Low power reactive */
// 1660        
// 1661         temp_us16 = cal_pf(y_phase.active.delta_cnts_temp / 100000, y_phase.apparent.delta_cnts_temp / 100000);
??power_filter_178:
        MOVW      DE, #LWRD(__Constant_186a0_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_y_phase+184)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __LLU2L
        CALL      __LLU2L            ;; 3 cycles
        MOVW      [SP+0x08], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [SP+0x0A], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      DE, #LWRD(__Constant_186a0_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_y_phase+104)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __LLU2L
        CALL      __LLU2L            ;; 3 cycles
        MOVW      DE, AX             ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+30
        POP       HL                 ;; 1 cycle
          CFI CFA SP+28
        MOVW      AX, [SP+0x0A]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x08]      ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+30
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
        MOVW      AX, DE             ;; 1 cycle
        PUSH      HL                 ;; 1 cycle
          CFI CFA SP+34
        POP       BC                 ;; 1 cycle
          CFI CFA SP+32
          CFI FunCall _cal_pf
        CALL      _cal_pf            ;; 3 cycles
        MOVW      S:_temp_us16, AX   ;; 1 cycle
// 1662         
// 1663         if(temp_us16 < 40 && (SP_CHECK_ZPF == 1) && (ZERORISE_METROLOGY_PAR == 1))      /* ZPF */
        MOVW      AX, S:_temp_us16   ;; 1 cycle
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+28
        CMPW      AX, #0x28          ;; 1 cycle
        BNC       ??power_filter_181  ;; 4 cycles
        ; ------------------------------------- Block: 46 cycles
// 1664         {
// 1665             y_phase.active.delta_cnts_temp = 0;
        MOVW      HL, #LWRD(_y_phase+104)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 1666             y_phase.apparent.delta_cnts_temp =  y_phase.reactive.delta_cnts_temp;
        MOVW      HL, #LWRD(_y_phase+184)  ;; 1 cycle
        MOVW      DE, #LWRD(_y_phase+144)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 1667             flag_Yph_active = IMPORT;
        CLR1      N:_flag_quadrant.2  ;; 2 cycles
        ; ------------------------------------- Block: 21 cycles
// 1668         }
// 1669         if(temp_us16 > 998 && (SP_CHECK_UPF == 1) && (ZERORISE_METROLOGY_PAR == 1) && curr.Yph.rms >= DIAL_IMAX_CURR)       /* UPF */
??power_filter_181:
        MOVW      AX, S:_temp_us16   ;; 1 cycle
        CMPW      AX, #0x3E7         ;; 1 cycle
        BC        ??power_filter_182  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      BC, N:_curr+18     ;; 1 cycle
        MOVW      AX, N:_curr+16     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
        CMPW      AX, #0xE678        ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??metrology_process_8:
        BC        ??power_filter_182  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
// 1670         {
// 1671             y_phase.reactive.delta_cnts_temp = 0;
        MOVW      HL, #LWRD(_y_phase+144)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 1672             y_phase.apparent.delta_cnts_temp =  y_phase.active.delta_cnts_temp;
        MOVW      HL, #LWRD(_y_phase+184)  ;; 1 cycle
        MOVW      DE, #LWRD(_y_phase+104)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 1673             flag_Yph_reactive = IMPORT;
        CLR1      N:_flag_quadrant.3  ;; 2 cycles
        ; ------------------------------------- Block: 21 cycles
// 1674         }
// 1675         if((y_phase.active.delta_cnts_temp / 100000) < 18)
??power_filter_182:
        MOVW      DE, #LWRD(__Constant_186a0_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_y_phase+104)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
        MOVW      BC, #LWRD(__Constant_12_0)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __CmpGeu64
        CALL      __CmpGeu64         ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_183  ;; 4 cycles
        ; ------------------------------------- Block: 16 cycles
// 1676         {
// 1677             y_phase.active.delta_cnts_temp = 0;
        MOVW      HL, #LWRD(_y_phase+104)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 1678             y_phase.apparent.delta_cnts_temp =  y_phase.reactive.delta_cnts_temp;
        MOVW      HL, #LWRD(_y_phase+184)  ;; 1 cycle
        MOVW      DE, #LWRD(_y_phase+144)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 1679             flag_Yph_active = IMPORT;
        CLR1      N:_flag_quadrant.2  ;; 2 cycles
        ; ------------------------------------- Block: 21 cycles
// 1680         }
// 1681         if((y_phase.reactive.delta_cnts_temp / 100000) < 18)
??power_filter_183:
        MOVW      DE, #LWRD(__Constant_186a0_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_y_phase+144)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
        MOVW      BC, #LWRD(__Constant_12_0)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __CmpGeu64
        CALL      __CmpGeu64         ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_184  ;; 4 cycles
        ; ------------------------------------- Block: 16 cycles
// 1682         {
// 1683             y_phase.reactive.delta_cnts_temp = 0;
        MOVW      HL, #LWRD(_y_phase+144)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 1684             y_phase.apparent.delta_cnts_temp =  y_phase.active.delta_cnts_temp;
        MOVW      HL, #LWRD(_y_phase+184)  ;; 1 cycle
        MOVW      DE, #LWRD(_y_phase+104)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 1685             flag_Yph_reactive = IMPORT;
        CLR1      N:_flag_quadrant.3  ;; 2 cycles
        ; ------------------------------------- Block: 21 cycles
// 1686         }
// 1687         if(bitIsSet(tpr.magnet.flag,event_f) && (MAG_SWITCH_IMAX == 1) && (flag_phase_present_y == 1))
??power_filter_184:
        MOVW      HL, #LWRD(_tpr+32)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??power_filter_185  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      HL, #LWRD(_flag_metro2)  ;; 1 cycle
        MOV1      CY, [HL].1         ;; 1 cycle
        BNC       ??power_filter_185  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1688         {
// 1689             y_phase.active.delta_cnts_temp = MAG_DELTA_CNTS_IMAX;
        MOVW      HL, #LWRD(_y_phase+104)  ;; 1 cycle
        MOVW      AX, #0x9000        ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x5A4E        ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x3           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 1690             y_phase.reactive.delta_cnts_temp = 0;
        MOVW      HL, #LWRD(_y_phase+144)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 1691             y_phase.apparent.delta_cnts_temp =  y_phase.active.delta_cnts_temp;
        MOVW      HL, #LWRD(_y_phase+184)  ;; 1 cycle
        MOVW      DE, #LWRD(_y_phase+104)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 1692             flag_Yph_active = IMPORT;
        CLR1      N:_flag_quadrant.2  ;; 2 cycles
// 1693             flag_Yph_reactive = IMPORT;
        CLR1      N:_flag_quadrant.3  ;; 2 cycles
        ; ------------------------------------- Block: 32 cycles
// 1694         }
// 1695         
// 1696         if(ZERORISE_METROLOGY_PAR == 0 || 
// 1697            (flag_phase_present_y == 1 && flag_min_current_y == 0 && ZERORISE_METROLOGY_PAR == 1))
??power_filter_185:
        MOV       A, N:_flag_metro2  ;; 1 cycle
        AND       A, #0x22           ;; 1 cycle
        CMP       A, #0x2            ;; 1 cycle
        SKZ                          ;; 4 cycles
        BR        N:??power_filter_186  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
// 1698         {
// 1699             y_phase.active.delta_cnts = y_phase.active.delta_cnts_temp;
        MOVW      HL, #LWRD(_y_phase+96)  ;; 1 cycle
        MOVW      DE, #LWRD(_y_phase+104)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 1700             y_phase.reactive.delta_cnts = y_phase.reactive.delta_cnts_temp;
        MOVW      HL, #LWRD(_y_phase+136)  ;; 1 cycle
        MOVW      DE, #LWRD(_y_phase+144)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 1701             y_phase.apparent.delta_cnts = y_phase.apparent.delta_cnts_temp;
        MOVW      HL, #LWRD(_y_phase+176)  ;; 1 cycle
        MOVW      DE, #LWRD(_y_phase+184)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 1702             
// 1703             y_phase.fundamental.active.delta_cnts = y_phase.active.delta_cnts * 1000;
        MOVW      DE, #LWRD(__Constant_3e8_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_y_phase+96)  ;; 1 cycle
        MOVW      AX, #LWRD(_y_phase+200)  ;; 1 cycle
          CFI FunCall __Mul64
        CALL      __Mul64            ;; 3 cycles
// 1704             y_phase.fundamental.active.delta_cnts /= (1000+thd.Yph.correction);
        MOV       X, N:_thd+34       ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        ADDW      AX, #0x3E8         ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+30
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
          CFI FunCall __L2LLS
        CALL      __L2LLS            ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      BC, #LWRD(_y_phase+200)  ;; 1 cycle
        MOVW      AX, #LWRD(_y_phase+200)  ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
// 1705             if(bitIsSet(tpr.magnet.flag,event_f) && (MAG_SWITCH_IMAX == 1))
        MOVW      HL, #LWRD(_tpr+32)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+28
        SKC                          ;; 4 cycles
        BR        N:??power_filter_187  ;; 4 cycles
        ; ------------------------------------- Block: 64 cycles
// 1706             {
// 1707                 curr.Yph.rms = IMAX;
        MOVW      AX, #0xEA60        ;; 1 cycle
        MOVW      N:_curr+16, AX     ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+18, AX     ;; 1 cycle
// 1708                 curr.Yph.dc = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+24, AX     ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+26, AX     ;; 1 cycle
// 1709                 curr.Yph.rms_signed = IMAX;
        MOVW      AX, #0xEA60        ;; 1 cycle
        MOVW      N:_curr+28, AX     ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+30, AX     ;; 1 cycle
// 1710                 vol.Yph.rms = VREF;
        MOVW      AX, #0x5DC0        ;; 1 cycle
        MOVW      N:_vol+6, AX       ;; 1 cycle
        BR        N:??power_filter_187  ;; 3 cycles
        ; ------------------------------------- Block: 17 cycles
// 1711             }
// 1712         }
// 1713         else
// 1714         {
// 1715 //            y_phase.active.acc_cnt = 0;
// 1716 //            y_phase.reactive.acc_cnt = 0;
// 1717 //            y_phase.apparent.acc_cnt = 0;
// 1718             if(bitIsSet(tpr.magnet.flag,event_f) && (MAG_SWITCH_IMAX == 1))
??power_filter_186:
        MOVW      HL, #LWRD(_tpr+32)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??power_filter_188  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1719             {
// 1720                 NOP();
        NOP                          ;; 1 cycle
        BR        S:??power_filter_189  ;; 3 cycles
        ; ------------------------------------- Block: 4 cycles
// 1721             }
// 1722             else
// 1723             {
// 1724                 if(EMPTY_BUCKET == 1) 
// 1725                 {
// 1726                     y_phase.active.acc_delta_cnts = 0;
??power_filter_188:
        MOVW      HL, #LWRD(_y_phase+88)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 1727                     y_phase.reactive.acc_delta_cnts = 0;
        MOVW      HL, #LWRD(_y_phase+128)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 1728                     y_phase.apparent.acc_delta_cnts = 0;
        MOVW      HL, #LWRD(_y_phase+168)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 1729                     y_phase.fundamental.active.delta_cnts = 0;
        MOVW      HL, #LWRD(_y_phase+200)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        ; ------------------------------------- Block: 36 cycles
// 1730                 }
// 1731             }
// 1732                 
// 1733             if(flag_phase_present_y == 0 && flag_min_current_y == 1)      /* when vol <20 and curr < 10 */
??power_filter_189:
        MOV       A, N:_flag_metro2  ;; 1 cycle
        AND       A, #0x22           ;; 1 cycle
        CMP       A, #0x20           ;; 1 cycle
        BNZ       ??power_filter_190  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
// 1734             {
// 1735                 vol.Yph.rms = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_vol+6, AX       ;; 1 cycle
// 1736                 vol.Yph.dc = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_vol+10, AX      ;; 1 cycle
// 1737                 curr.Yph.rms = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+16, AX     ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+18, AX     ;; 1 cycle
// 1738                 curr.Yph.dc = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+24, AX     ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+26, AX     ;; 1 cycle
// 1739                 curr.Yph.rms_signed = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+28, AX     ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+30, AX     ;; 1 cycle
// 1740                 y_phase.active.delta_cnts_temp = 0;
        MOVW      HL, #LWRD(_y_phase+104)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 1741                 y_phase.reactive.delta_cnts_temp = 0;
        MOVW      HL, #LWRD(_y_phase+144)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 1742                 y_phase.apparent.delta_cnts_temp = 0;
        MOVW      HL, #LWRD(_y_phase+184)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        BR        N:??power_filter_191  ;; 3 cycles
        ; ------------------------------------- Block: 46 cycles
// 1743             }
// 1744             else if(flag_phase_present_y == 0 && flag_min_current_y == 0) /* when vol <20 and curr >= 10 */
??power_filter_190:
        MOV       A, N:_flag_metro2  ;; 1 cycle
        AND       A, #0x22           ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_192  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
// 1745             {
// 1746                 vol.Yph.rms = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_vol+6, AX       ;; 1 cycle
// 1747                 vol.Yph.dc = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_vol+10, AX      ;; 1 cycle
// 1748                 //                curr.Yph.rms = 0;
// 1749                 //                curr.Yph.dc = 0;
// 1750                 //                curr.Yph.rms_signed = 0;
// 1751                 y_phase.active.delta_cnts_temp = 0;
        MOVW      HL, #LWRD(_y_phase+104)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 1752                 y_phase.reactive.delta_cnts_temp = 0;
        MOVW      HL, #LWRD(_y_phase+144)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 1753                 y_phase.apparent.delta_cnts_temp = 0;
        MOVW      HL, #LWRD(_y_phase+184)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        BR        N:??power_filter_191  ;; 3 cycles
        ; ------------------------------------- Block: 34 cycles
// 1754             }
// 1755             else if(flag_phase_present_y == 1 && flag_min_current_y == 1) /* when vol > 20 and curr < 10 */
??power_filter_192:
        MOV       A, N:_flag_metro2  ;; 1 cycle
        AND       A, #0x22           ;; 1 cycle
        CMP       A, #0x22           ;; 1 cycle
        SKZ                          ;; 4 cycles
        BR        N:??power_filter_191  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
// 1756             {
// 1757                 if(bitIsSet(tpr.magnet.flag,event_f) && (MAG_SWITCH_IMAX == 1))
        MOVW      HL, #LWRD(_tpr+32)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??power_filter_193  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1758                 {
// 1759                     curr.Yph.rms = IMAX;
        MOVW      AX, #0xEA60        ;; 1 cycle
        MOVW      N:_curr+16, AX     ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+18, AX     ;; 1 cycle
// 1760                     curr.Yph.dc = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+24, AX     ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+26, AX     ;; 1 cycle
// 1761                     curr.Yph.rms_signed = IMAX;
        MOVW      AX, #0xEA60        ;; 1 cycle
        MOVW      N:_curr+28, AX     ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+30, AX     ;; 1 cycle
// 1762                     vol.Yph.rms = VREF;
        MOVW      AX, #0x5DC0        ;; 1 cycle
        MOVW      N:_vol+6, AX       ;; 1 cycle
        BR        S:??power_filter_191  ;; 3 cycles
        ; ------------------------------------- Block: 17 cycles
// 1763                 }
// 1764                 else
// 1765                 {
// 1766                     curr.Yph.rms = 0;
??power_filter_193:
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+16, AX     ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+18, AX     ;; 1 cycle
// 1767                     curr.Yph.dc = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+24, AX     ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+26, AX     ;; 1 cycle
// 1768                     curr.Yph.rms_signed = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+28, AX     ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+30, AX     ;; 1 cycle
// 1769                     y_phase.active.delta_cnts_temp = 0;
        MOVW      HL, #LWRD(_y_phase+104)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 1770                     y_phase.reactive.delta_cnts_temp = 0;
        MOVW      HL, #LWRD(_y_phase+144)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 1771                     y_phase.apparent.delta_cnts_temp = 0;
        MOVW      HL, #LWRD(_y_phase+184)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        ; ------------------------------------- Block: 39 cycles
// 1772                 }
// 1773             }
// 1774             flag_Yph_active = IMPORT;
??power_filter_191:
        CLR1      N:_flag_quadrant.2  ;; 2 cycles
// 1775             flag_Yph_reactive = IMPORT;
        CLR1      N:_flag_quadrant.3  ;; 2 cycles
// 1776             y_phase.active.delta_cnts = y_phase.active.delta_cnts_temp;
        MOVW      HL, #LWRD(_y_phase+96)  ;; 1 cycle
        MOVW      DE, #LWRD(_y_phase+104)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 1777             y_phase.reactive.delta_cnts = y_phase.reactive.delta_cnts_temp;
        MOVW      HL, #LWRD(_y_phase+136)  ;; 1 cycle
        MOVW      DE, #LWRD(_y_phase+144)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 1778             y_phase.apparent.delta_cnts = y_phase.apparent.delta_cnts_temp;
        MOVW      HL, #LWRD(_y_phase+176)  ;; 1 cycle
        MOVW      DE, #LWRD(_y_phase+184)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 1779             
// 1780             y_phase.fundamental.active.delta_cnts = y_phase.active.delta_cnts * 1000;
        MOVW      DE, #LWRD(__Constant_3e8_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_y_phase+96)  ;; 1 cycle
        MOVW      AX, #LWRD(_y_phase+200)  ;; 1 cycle
          CFI FunCall __Mul64
        CALL      __Mul64            ;; 3 cycles
// 1781             y_phase.fundamental.active.delta_cnts /= (1000+thd.Yph.correction);
        MOV       X, N:_thd+34       ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        ADDW      AX, #0x3E8         ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+30
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
          CFI FunCall __L2LLS
        CALL      __L2LLS            ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      BC, #LWRD(_y_phase+200)  ;; 1 cycle
        MOVW      AX, #LWRD(_y_phase+200)  ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+28
        ; ------------------------------------- Block: 62 cycles
// 1782         }
// 1783          
// 1784         /* Calculating powers */
// 1785         power.Yph.active = y_phase.active.delta_cnts / 100000;
??power_filter_187:
        MOVW      DE, #LWRD(__Constant_186a0_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_y_phase+96)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __LLU2L
        CALL      __LLU2L            ;; 3 cycles
        MOVW      N:_power+20, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_power+22, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1786         power.Yph.reactive = y_phase.reactive.delta_cnts / 100000;
        MOVW      DE, #LWRD(__Constant_186a0_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_y_phase+136)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __LLU2L
        CALL      __LLU2L            ;; 3 cycles
        MOVW      N:_power+24, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_power+26, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1787         power.Yph.apparent =  y_phase.apparent.delta_cnts / 100000;
        MOVW      DE, #LWRD(__Constant_186a0_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_y_phase+176)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __LLU2L
        CALL      __LLU2L            ;; 3 cycles
        MOVW      N:_power+28, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_power+30, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1788         pf.Yph = cal_pf(power.Yph.active,power.Yph.apparent);
        MOVW      AX, N:_power+30    ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+30
        MOVW      AX, N:_power+28    ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
        MOVW      BC, N:_power+22    ;; 1 cycle
        MOVW      AX, N:_power+20    ;; 1 cycle
          CFI FunCall _cal_pf
        CALL      _cal_pf            ;; 3 cycles
        MOVW      N:_pf+2, AX        ;; 1 cycle
// 1789         
// 1790         /* Quadrant */
// 1791         quadrant.Yph = get_quadrant(flag_Yph_active,flag_Yph_reactive);
        MOVW      HL, #LWRD(_flag_quadrant)  ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOV1      CY, [HL].3         ;; 1 cycle
        ROLC      A, 0x1             ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOVW      HL, #LWRD(_flag_quadrant)  ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOV1      CY, [HL].2         ;; 1 cycle
        ROLC      A, 0x1             ;; 1 cycle
          CFI FunCall _get_quadrant
        CALL      _get_quadrant      ;; 3 cycles
        MOV       N:_quadrant+1, A   ;; 1 cycle
// 1792         
// 1793         if(METERING_MODE == NET)
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+28
        CMP       N:_METERING_MODE, #0x1  ;; 1 cycle
        SKZ                          ;; 4 cycles
        BR        N:??power_filter_194  ;; 4 cycles
        ; ------------------------------------- Block: 71 cycles
// 1794         {
// 1795             /* Active related */
// 1796             if(quadrant.Yph == Q1 || quadrant.Yph == Q4)   
        CMP       N:_quadrant+1, #0x1  ;; 1 cycle
        BZ        ??power_filter_195  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        CMP       N:_quadrant+1, #0x4  ;; 1 cycle
        BNZ       ??power_filter_196  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 1797             {
// 1798                 power.Yph.active_signed = power.Yph.active;
??power_filter_195:
        MOVW      BC, N:_power+22    ;; 1 cycle
        MOVW      AX, N:_power+20    ;; 1 cycle
        MOVW      N:_power+32, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_power+34, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1799                 curr.Yph.rms_signed = curr.Yph.rms;
        MOVW      BC, N:_curr+18     ;; 1 cycle
        MOVW      AX, N:_curr+16     ;; 1 cycle
        MOVW      N:_curr+28, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_curr+30, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1800                 pf.Yph_signed = pf.Yph;
        MOVW      AX, N:_pf+2        ;; 1 cycle
        MOVW      N:_pf+10, AX       ;; 1 cycle
        BR        S:??power_filter_197  ;; 3 cycles
        ; ------------------------------------- Block: 17 cycles
// 1801             }
// 1802             else                                            
// 1803             {
// 1804                 power.Yph.active_signed = -power.Yph.active;
??power_filter_196:
        MOVW      BC, N:_power+22    ;; 1 cycle
        MOVW      AX, N:_power+20    ;; 1 cycle
          CFI FunCall ?L_NEG_L03
        CALL      N:?L_NEG_L03       ;; 3 cycles
        MOVW      N:_power+32, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_power+34, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1805                 curr.Yph.rms_signed = -curr.Yph.rms;
        MOVW      BC, N:_curr+18     ;; 1 cycle
        MOVW      AX, N:_curr+16     ;; 1 cycle
          CFI FunCall ?L_NEG_L03
        CALL      N:?L_NEG_L03       ;; 3 cycles
        MOVW      N:_curr+28, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_curr+30, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1806                 pf.Yph_signed = -pf.Yph;
        MOVW      AX, N:_pf+2        ;; 1 cycle
        XOR       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        XOR       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        INCW      AX                 ;; 1 cycle
        MOVW      N:_pf+10, AX       ;; 1 cycle
        ; ------------------------------------- Block: 25 cycles
// 1807             }
// 1808             
// 1809             /* Reactive related */
// 1810             if(quadrant.Yph == Q1 || quadrant.Yph == Q2)   
??power_filter_197:
        CMP       N:_quadrant+1, #0x1  ;; 1 cycle
        BZ        ??power_filter_198  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        CMP       N:_quadrant+1, #0x2  ;; 1 cycle
        BNZ       ??power_filter_199  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 1811             {
// 1812                 power.Yph.reactive_signed = power.Yph.reactive;
??power_filter_198:
        MOVW      BC, N:_power+26    ;; 1 cycle
        MOVW      AX, N:_power+24    ;; 1 cycle
        MOVW      N:_power+36, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_power+38, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        BR        N:??power_filter_200  ;; 3 cycles
        ; ------------------------------------- Block: 9 cycles
// 1813             }
// 1814             else                                         
// 1815             {
// 1816                 power.Yph.reactive_signed = -power.Yph.reactive;
??power_filter_199:
        MOVW      BC, N:_power+26    ;; 1 cycle
        MOVW      AX, N:_power+24    ;; 1 cycle
          CFI FunCall ?L_NEG_L03
        CALL      N:?L_NEG_L03       ;; 3 cycles
        MOVW      N:_power+36, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_power+38, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        BR        S:??power_filter_200  ;; 3 cycles
        ; ------------------------------------- Block: 12 cycles
// 1817             }
// 1818         }
// 1819         else 
// 1820         {
// 1821             /* Active related */
// 1822             power.Yph.active_signed = power.Yph.active;
??power_filter_194:
        MOVW      BC, N:_power+22    ;; 1 cycle
        MOVW      AX, N:_power+20    ;; 1 cycle
        MOVW      N:_power+32, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_power+34, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1823             if(flag_Yph_active == EXPORT)
        MOVW      HL, #LWRD(_flag_quadrant)  ;; 1 cycle
        MOV1      CY, [HL].2         ;; 1 cycle
        BNC       ??power_filter_201  ;; 4 cycles
        ; ------------------------------------- Block: 12 cycles
// 1824             {
// 1825                 curr.Yph.rms_signed = -curr.Yph.rms;
        MOVW      BC, N:_curr+18     ;; 1 cycle
        MOVW      AX, N:_curr+16     ;; 1 cycle
          CFI FunCall ?L_NEG_L03
        CALL      N:?L_NEG_L03       ;; 3 cycles
        MOVW      N:_curr+28, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_curr+30, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        BR        S:??power_filter_202  ;; 3 cycles
        ; ------------------------------------- Block: 12 cycles
// 1826             }
// 1827             else
// 1828             {
// 1829                 curr.Yph.rms_signed = curr.Yph.rms;
??power_filter_201:
        MOVW      BC, N:_curr+18     ;; 1 cycle
        MOVW      AX, N:_curr+16     ;; 1 cycle
        MOVW      N:_curr+28, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_curr+30, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
// 1830             }
// 1831             
// 1832             /* Reactive related */
// 1833             if(quadrant.Yph == Q1 || quadrant.Yph == Q3)   
??power_filter_202:
        CMP       N:_quadrant+1, #0x1  ;; 1 cycle
        BZ        ??power_filter_203  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        CMP       N:_quadrant+1, #0x3  ;; 1 cycle
        BNZ       ??power_filter_204  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 1834             {
// 1835                 power.Yph.reactive_signed = power.Yph.reactive;
??power_filter_203:
        MOVW      BC, N:_power+26    ;; 1 cycle
        MOVW      AX, N:_power+24    ;; 1 cycle
        MOVW      N:_power+36, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_power+38, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1836                 pf.Yph_signed = pf.Yph;
        MOVW      AX, N:_pf+2        ;; 1 cycle
        MOVW      N:_pf+10, AX       ;; 1 cycle
        BR        S:??power_filter_200  ;; 3 cycles
        ; ------------------------------------- Block: 11 cycles
// 1837             }
// 1838             else                                         
// 1839             {
// 1840                 power.Yph.reactive_signed = -power.Yph.reactive;
??power_filter_204:
        MOVW      BC, N:_power+26    ;; 1 cycle
        MOVW      AX, N:_power+24    ;; 1 cycle
          CFI FunCall ?L_NEG_L03
        CALL      N:?L_NEG_L03       ;; 3 cycles
        MOVW      N:_power+36, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_power+38, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1841                 pf.Yph_signed = -pf.Yph;
        MOVW      AX, N:_pf+2        ;; 1 cycle
        XOR       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        XOR       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        INCW      AX                 ;; 1 cycle
        MOVW      N:_pf+10, AX       ;; 1 cycle
        ; ------------------------------------- Block: 16 cycles
// 1842             }
// 1843         }
// 1844 
// 1845         /* Phase angle */
// 1846         //        angle.Yph = cal_angle(power.Yph.active,power.Yph.reactive,power.Yph.apparent,flag_Yph_active,flag_Yph_reactive,1);
// 1847         if(flag_mag_update_metro_par == 1)
??power_filter_200:
        MOVW      HL, #LWRD(_flag_mag)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        SKNC                         ;; 1 cycle
        ; ------------------------------------- Block: 3 cycles
// 1848         {
// 1849             flag_mag_y_updated = 1;
        SET1      N:_flag_mag.2      ;; 2 cycles
          CFI FunCall _metrology_all_phase_calculation
        ; ------------------------------------- Block: 2 cycles
// 1850         }
// 1851         /* calculation for all phase variables */
// 1852         metrology_all_phase_calculation();
??metrology_process_9:
        CALL      _metrology_all_phase_calculation  ;; 3 cycles
// 1853         
// 1854         /* Calibration */
// 1855         if(cal_done_f != 1)
        CMP       N:_cal_done_f, #0x1  ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??power_filter_205  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
// 1856         {
// 1857             if(vol.Yph.rms >= CAL_VOL_MIN && vol.Yph.rms <= CAL_VOL_MAX && 
// 1858                curr.Yph.rms >= CAL_CURR_MIN && curr.Yph.rms <= CAL_CURR_MAX)
        MOVW      AX, N:_vol+6       ;; 1 cycle
        CMPW      AX, #0x55F0        ;; 1 cycle
        BC        ??power_filter_206  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, N:_vol+6       ;; 1 cycle
        CMPW      AX, #0x6591        ;; 1 cycle
        BNC       ??power_filter_206  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      BC, N:_curr+18     ;; 1 cycle
        MOVW      AX, N:_curr+16     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
        CMPW      AX, #0x1F40        ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??metrology_process_10:
        BC        ??power_filter_206  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
        MOVW      BC, N:_curr+18     ;; 1 cycle
        MOVW      AX, N:_curr+16     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
        CMPW      AX, #0x2EE1        ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??metrology_process_11:
        BNC       ??power_filter_206  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
// 1859             {
// 1860                 flag_calibration_y_vi_ok = 1;
        SET1      N:_flag_cal.1      ;; 2 cycles
        BR        S:??power_filter_207  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 1861             }
// 1862             else
// 1863             {
// 1864                 flag_calibration_y_vi_ok = 0;
??power_filter_206:
        CLR1      N:_flag_cal.1      ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
// 1865             }
// 1866             if(power.Yph.active >= CAL_POWER_MIN && power.Yph.active <= CAL_POWER_MAX &&
// 1867                freq.Yph >= CAL_FREQ_MIN && freq.Yph <= CAL_FREQ_MAX && 
// 1868                    pf.Yph >= CAL_PF_MIN)
??power_filter_207:
        MOVW      BC, N:_power+22    ;; 1 cycle
        MOVW      AX, N:_power+20    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
        CMPW      AX, #0x44C0        ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??metrology_process_12:
        BC        ??power_filter_208  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
        MOVW      BC, N:_power+22    ;; 1 cycle
        MOVW      AX, N:_power+20    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
        CMPW      AX, #0x79E1        ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??metrology_process_13:
        BNC       ??power_filter_208  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
        MOVW      AX, N:_freq+2      ;; 1 cycle
        CMPW      AX, #0xC288        ;; 1 cycle
        BC        ??power_filter_208  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, N:_freq+2      ;; 1 cycle
        CMPW      AX, #0xC419        ;; 1 cycle
        BNC       ??power_filter_208  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, N:_pf+2        ;; 1 cycle
        CMPW      AX, #0x384         ;; 1 cycle
        BC        ??power_filter_208  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1869             {
// 1870                 flag_calibration_y_pf_freq_pow_ok = 1;
        SET1      N:_flag_calibration.3  ;; 2 cycles
        BR        S:??power_filter_209  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 1871             }
// 1872             else
// 1873             {
// 1874                 flag_calibration_y_pf_freq_pow_ok = 0;
??power_filter_208:
        CLR1      N:_flag_calibration.3  ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
// 1875             }
// 1876             if(flag_Yph_active == IMPORT)
??power_filter_209:
        MOVW      HL, #LWRD(_flag_quadrant)  ;; 1 cycle
        MOV1      CY, [HL].2         ;; 1 cycle
        BC        ??power_filter_210  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1877             {
// 1878                 flag_calibration_y_ct_ok = 1;
        SET1      N:_flag_calibration.6  ;; 2 cycles
        BR        S:??power_filter_211  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 1879             }
// 1880             else
// 1881             {
// 1882                 flag_calibration_y_ct_ok = 0;
??power_filter_210:
        CLR1      N:_flag_calibration.6  ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
// 1883             }
// 1884             
// 1885             if(cal_YPh.l_shift.active.acc_cnt < 0)
??power_filter_211:
        MOVW      BC, #LWRD(__Constant_0_0)  ;; 1 cycle
        MOVW      AX, #LWRD(_cal_YPh+18)  ;; 1 cycle
          CFI FunCall __CmpGes64
        CALL      __CmpGes64         ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_212  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
// 1886             {
// 1887                 temp_us64 = ~cal_YPh.l_shift.active.acc_cnt + 1;
        MOVW      BC, #LWRD(_cal_YPh+18)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
          CFI FunCall __Not64
        CALL      __Not64            ;; 3 cycles
        MOVW      DE, #LWRD(__Constant_1_0)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Add64
        CALL      __Add64            ;; 3 cycles
        MOVW      HL, #LWRD(_temp_us64)  ;; 1 cycle
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
        BR        S:??power_filter_213  ;; 3 cycles
        ; ------------------------------------- Block: 28 cycles
// 1888             }
// 1889             else
// 1890             {
// 1891                 temp_us64 = cal_YPh.l_shift.active.acc_cnt;
??power_filter_212:
        MOVW      HL, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      DE, #LWRD(_cal_YPh+18)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        ; ------------------------------------- Block: 10 cycles
// 1892             }
// 1893             
// 1894             temp_us64 *= cal_coeff.Yph.power;
??power_filter_213:
        MOVW      AX, N:_cal_coeff+22  ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+30
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
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
// 1895             temp_us64 /= y_phase.no_of_samples;
        MOVW      AX, N:_y_phase     ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+34
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+36
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
// 1896             temp_us64 /= 100000;
        MOVW      DE, #LWRD(__Constant_186a0_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
// 1897             cal_YPh.l_shift.active.power = temp_us64;
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __LLU2L
        CALL      __LLU2L            ;; 3 cycles
        MOVW      N:_cal_YPh+14, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_cal_YPh+16, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1898             if(cal_YPh.l_shift.active.acc_cnt < 0)
        MOVW      BC, #LWRD(__Constant_0_0)  ;; 1 cycle
        MOVW      AX, #LWRD(_cal_YPh+18)  ;; 1 cycle
          CFI FunCall __CmpGes64
        CALL      __CmpGes64         ;; 3 cycles
        ADDW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+28
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_214  ;; 4 cycles
        ; ------------------------------------- Block: 59 cycles
// 1899             {
// 1900                 cal_YPh.l_shift.active.power = ~cal_YPh.l_shift.active.power + 1;
        MOVW      BC, N:_cal_YPh+16  ;; 1 cycle
        MOVW      AX, N:_cal_YPh+14  ;; 1 cycle
          CFI FunCall ?L_NOT_L03
        CALL      N:?L_NOT_L03       ;; 3 cycles
        ADDW      AX, #0x1           ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_cal_YPh+14, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_cal_YPh+16, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ; ------------------------------------- Block: 18 cycles
// 1901             }
// 1902             
// 1903             if(cal_YPh.r_shift.active.acc_cnt < 0)
??power_filter_214:
        MOVW      BC, #LWRD(__Constant_0_0)  ;; 1 cycle
        MOVW      AX, #LWRD(_cal_YPh+62)  ;; 1 cycle
          CFI FunCall __CmpGes64
        CALL      __CmpGes64         ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_215  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
// 1904             {
// 1905                 temp_us64 = ~cal_YPh.r_shift.active.acc_cnt + 1;
        MOVW      BC, #LWRD(_cal_YPh+62)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
          CFI FunCall __Not64
        CALL      __Not64            ;; 3 cycles
        MOVW      DE, #LWRD(__Constant_1_0)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Add64
        CALL      __Add64            ;; 3 cycles
        MOVW      HL, #LWRD(_temp_us64)  ;; 1 cycle
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
        BR        S:??power_filter_216  ;; 3 cycles
        ; ------------------------------------- Block: 28 cycles
// 1906             }
// 1907             else
// 1908             {
// 1909                 temp_us64 = cal_YPh.r_shift.active.acc_cnt;
??power_filter_215:
        MOVW      HL, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      DE, #LWRD(_cal_YPh+62)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        ; ------------------------------------- Block: 10 cycles
// 1910             }
// 1911             
// 1912             temp_us64 *= cal_coeff.Yph.power;
??power_filter_216:
        MOVW      AX, N:_cal_coeff+22  ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+30
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
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
// 1913             temp_us64 /= y_phase.no_of_samples;
        MOVW      AX, N:_y_phase     ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+34
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+36
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
// 1914             temp_us64 /= 100000;
        MOVW      DE, #LWRD(__Constant_186a0_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
// 1915             cal_YPh.r_shift.active.power = temp_us64;
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __LLU2L
        CALL      __LLU2L            ;; 3 cycles
        MOVW      N:_cal_YPh+58, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_cal_YPh+60, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1916             if(cal_YPh.r_shift.active.acc_cnt < 0)
        MOVW      BC, #LWRD(__Constant_0_0)  ;; 1 cycle
        MOVW      AX, #LWRD(_cal_YPh+62)  ;; 1 cycle
          CFI FunCall __CmpGes64
        CALL      __CmpGes64         ;; 3 cycles
        ADDW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+28
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_217  ;; 4 cycles
        ; ------------------------------------- Block: 59 cycles
// 1917             {
// 1918                 cal_YPh.r_shift.active.power = ~cal_YPh.r_shift.active.power + 1;
        MOVW      BC, N:_cal_YPh+60  ;; 1 cycle
        MOVW      AX, N:_cal_YPh+58  ;; 1 cycle
          CFI FunCall ?L_NOT_L03
        CALL      N:?L_NOT_L03       ;; 3 cycles
        ADDW      AX, #0x1           ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_cal_YPh+58, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_cal_YPh+60, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ; ------------------------------------- Block: 18 cycles
// 1919             }
// 1920             
// 1921             cal_YPh.angle_active = cal_angle_calculate_act(cal_YPh.l_shift.active.power,cal_YPh.r_shift.active.power);
??power_filter_217:
        MOVW      AX, N:_cal_YPh+60  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+30
        MOVW      AX, N:_cal_YPh+58  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
        MOVW      BC, N:_cal_YPh+16  ;; 1 cycle
        MOVW      AX, N:_cal_YPh+14  ;; 1 cycle
          CFI FunCall _cal_angle_calculate_act
        CALL      _cal_angle_calculate_act  ;; 3 cycles
        MOVW      N:_cal_YPh+6, AX   ;; 1 cycle
// 1922             
// 1923             if(cal_YPh.l_shift.reactive.acc_cnt < 0)
        MOVW      BC, #LWRD(__Constant_0_0)  ;; 1 cycle
        MOVW      AX, #LWRD(_cal_YPh+38)  ;; 1 cycle
          CFI FunCall __CmpGes64
        CALL      __CmpGes64         ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+28
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_218  ;; 4 cycles
        ; ------------------------------------- Block: 21 cycles
// 1924             {
// 1925                 temp_us64 = ~cal_YPh.l_shift.reactive.acc_cnt + 1;
        MOVW      BC, #LWRD(_cal_YPh+38)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
          CFI FunCall __Not64
        CALL      __Not64            ;; 3 cycles
        MOVW      DE, #LWRD(__Constant_1_0)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Add64
        CALL      __Add64            ;; 3 cycles
        MOVW      HL, #LWRD(_temp_us64)  ;; 1 cycle
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
        BR        S:??power_filter_219  ;; 3 cycles
        ; ------------------------------------- Block: 28 cycles
// 1926             }
// 1927             else
// 1928             {
// 1929                 temp_us64 = cal_YPh.l_shift.reactive.acc_cnt;
??power_filter_218:
        MOVW      HL, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      DE, #LWRD(_cal_YPh+38)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        ; ------------------------------------- Block: 10 cycles
// 1930             }
// 1931             
// 1932             temp_us64 *= cal_coeff.Yph.power;
??power_filter_219:
        MOVW      AX, N:_cal_coeff+22  ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+30
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
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
// 1933             temp_us64 /= y_phase.no_of_samples;
        MOVW      AX, N:_y_phase     ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+34
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+36
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
// 1934             temp_us64 /= 100000;
        MOVW      DE, #LWRD(__Constant_186a0_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
// 1935             cal_YPh.l_shift.reactive.power = temp_us64;
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __LLU2L
        CALL      __LLU2L            ;; 3 cycles
        MOVW      N:_cal_YPh+34, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_cal_YPh+36, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1936             if(cal_YPh.l_shift.reactive.acc_cnt < 0)
        MOVW      BC, #LWRD(__Constant_0_0)  ;; 1 cycle
        MOVW      AX, #LWRD(_cal_YPh+38)  ;; 1 cycle
          CFI FunCall __CmpGes64
        CALL      __CmpGes64         ;; 3 cycles
        ADDW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+28
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_220  ;; 4 cycles
        ; ------------------------------------- Block: 59 cycles
// 1937             {
// 1938                 cal_YPh.l_shift.reactive.power = ~cal_YPh.l_shift.reactive.power + 1;
        MOVW      BC, N:_cal_YPh+36  ;; 1 cycle
        MOVW      AX, N:_cal_YPh+34  ;; 1 cycle
          CFI FunCall ?L_NOT_L03
        CALL      N:?L_NOT_L03       ;; 3 cycles
        ADDW      AX, #0x1           ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_cal_YPh+34, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_cal_YPh+36, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ; ------------------------------------- Block: 18 cycles
// 1939             }
// 1940             
// 1941             if(cal_YPh.r_shift.reactive.acc_cnt < 0)
??power_filter_220:
        MOVW      BC, #LWRD(__Constant_0_0)  ;; 1 cycle
        MOVW      AX, #LWRD(_cal_YPh+82)  ;; 1 cycle
          CFI FunCall __CmpGes64
        CALL      __CmpGes64         ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_221  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
// 1942             {
// 1943                 temp_us64 = ~cal_YPh.r_shift.reactive.acc_cnt + 1;
        MOVW      BC, #LWRD(_cal_YPh+82)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
          CFI FunCall __Not64
        CALL      __Not64            ;; 3 cycles
        MOVW      DE, #LWRD(__Constant_1_0)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Add64
        CALL      __Add64            ;; 3 cycles
        MOVW      HL, #LWRD(_temp_us64)  ;; 1 cycle
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
        BR        S:??power_filter_222  ;; 3 cycles
        ; ------------------------------------- Block: 28 cycles
// 1944             }
// 1945             else
// 1946             {
// 1947                 temp_us64 = cal_YPh.r_shift.reactive.acc_cnt;
??power_filter_221:
        MOVW      HL, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      DE, #LWRD(_cal_YPh+82)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        ; ------------------------------------- Block: 10 cycles
// 1948             }
// 1949             
// 1950             temp_us64 *= cal_coeff.Yph.power;
??power_filter_222:
        MOVW      AX, N:_cal_coeff+22  ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+30
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
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
// 1951             temp_us64 /= y_phase.no_of_samples;
        MOVW      AX, N:_y_phase     ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+34
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+36
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
// 1952             temp_us64 /= 100000;
        MOVW      DE, #LWRD(__Constant_186a0_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
// 1953             cal_YPh.r_shift.reactive.power = temp_us64;
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __LLU2L
        CALL      __LLU2L            ;; 3 cycles
        MOVW      N:_cal_YPh+78, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_cal_YPh+80, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1954             if(cal_YPh.r_shift.reactive.acc_cnt < 0)
        MOVW      BC, #LWRD(__Constant_0_0)  ;; 1 cycle
        MOVW      AX, #LWRD(_cal_YPh+82)  ;; 1 cycle
          CFI FunCall __CmpGes64
        CALL      __CmpGes64         ;; 3 cycles
        ADDW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+28
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_223  ;; 4 cycles
        ; ------------------------------------- Block: 59 cycles
// 1955             {
// 1956                 cal_YPh.r_shift.reactive.power = ~cal_YPh.r_shift.reactive.power + 1;
        MOVW      BC, N:_cal_YPh+80  ;; 1 cycle
        MOVW      AX, N:_cal_YPh+78  ;; 1 cycle
          CFI FunCall ?L_NOT_L03
        CALL      N:?L_NOT_L03       ;; 3 cycles
        ADDW      AX, #0x1           ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_cal_YPh+78, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_cal_YPh+80, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ; ------------------------------------- Block: 18 cycles
// 1957             }
// 1958             
// 1959             cal_YPh.angle_reactive = cal_angle_calculate_act(cal_YPh.l_shift.reactive.power,cal_YPh.r_shift.reactive.power);
??power_filter_223:
        MOVW      AX, N:_cal_YPh+80  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+30
        MOVW      AX, N:_cal_YPh+78  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
        MOVW      BC, N:_cal_YPh+36  ;; 1 cycle
        MOVW      AX, N:_cal_YPh+34  ;; 1 cycle
          CFI FunCall _cal_angle_calculate_act
        CALL      _cal_angle_calculate_act  ;; 3 cycles
        MOVW      N:_cal_YPh+8, AX   ;; 1 cycle
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+28
        BR        S:??power_filter_161  ;; 3 cycles
        ; ------------------------------------- Block: 14 cycles
// 1960         }
// 1961         else
// 1962         {
// 1963             flag_calibration_y_vi_ok = 0;
??power_filter_205:
        CLR1      N:_flag_cal.1      ;; 2 cycles
// 1964             flag_calibration_y_pf_freq_pow_ok = 0;
        CLR1      N:_flag_calibration.3  ;; 2 cycles
// 1965             flag_calibration_y_ct_ok = 0;
        CLR1      N:_flag_calibration.6  ;; 2 cycles
        ; ------------------------------------- Block: 6 cycles
// 1966         }
// 1967         
// 1968     }
// 1969     /************************************************************************************************
// 1970     *****************************************   B PHASE    ******************************************
// 1971     ************************************************************************************************/
// 1972     if(flag_metrology_process_b == 1 && b_phase.no_of_samples != 0)
??power_filter_161:
        MOVW      HL, #LWRD(_flag_metrology)  ;; 1 cycle
        MOV1      CY, [HL].2         ;; 1 cycle
        SKC                          ;; 4 cycles
        BR        N:??power_filter_224  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        CLRW      AX                 ;; 1 cycle
        CMPW      AX, N:_b_phase     ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??power_filter_224  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1973     {
// 1974         flag_metrology_process_b = 0;
        CLR1      N:_flag_metrology.2  ;; 2 cycles
// 1975         
// 1976         /* Current */ 
// 1977         curr.Bph.rms = calculate_irms(b_phase.curr.rms_acc_cnt,cal_coeff.Bph.curr,b_phase.no_of_samples);
        MOVW      AX, N:_b_phase     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+30
        MOVW      AX, N:_cal_coeff+32  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
        MOVW      HL, #LWRD(_b_phase+56)  ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+40
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
          CFI FunCall _calculate_irms
        CALL      _calculate_irms    ;; 3 cycles
        MOVW      N:_curr+32, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_curr+34, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1978         curr.Bph.dc = calculate_irms(b_phase.curr.offset_acc_cnt,cal_coeff.Bph.curr,b_phase.no_of_samples);
        MOVW      AX, N:_b_phase+50  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+42
        MOVW      AX, N:_b_phase+48  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+44
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
          CFI FunCall __L2LLS
        CALL      __L2LLS            ;; 3 cycles
        MOVW      AX, N:_b_phase     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+46
        MOVW      AX, N:_cal_coeff+32  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+48
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x14          ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+56
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
          CFI FunCall _calculate_irms
        CALL      _calculate_irms    ;; 3 cycles
        MOVW      N:_curr+40, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_curr+42, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1979         b_phase.curr.dc_offset = b_phase.curr.offset_acc_cnt / b_phase.no_of_samples;
        MOVW      AX, N:_b_phase     ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+58
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+60
        MOVW      BC, N:_b_phase+50  ;; 1 cycle
        MOVW      AX, N:_b_phase+48  ;; 1 cycle
          CFI FunCall ?SL_DIV_L03
        CALL      N:?SL_DIV_L03      ;; 3 cycles
        MOVW      N:_b_phase+46, AX  ;; 1 cycle
// 1980         
// 1981         /* Voltage */
// 1982         vol.Bph.rms = calculate_vrms(b_phase.vol.rms_acc_cnt,cal_coeff.Bph.vol,b_phase.no_of_samples);
        MOVW      AX, N:_b_phase     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+62
        MOVW      AX, N:_cal_coeff+30  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+64
        MOVW      HL, #LWRD(_b_phase+26)  ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+72
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
          CFI FunCall _calculate_vrms
        CALL      _calculate_vrms    ;; 3 cycles
        ADDW      SP, #0x2C          ;; 1 cycle
          CFI CFA SP+28
        MOVW      N:_vol+12, AX      ;; 1 cycle
// 1983         vol.Bph.dc = calculate_vrms(b_phase.vol.offset_acc_cnt,cal_coeff.Bph.vol,b_phase.no_of_samples);
        MOVW      AX, N:_b_phase+20  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+30
        MOVW      AX, N:_b_phase+18  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
          CFI FunCall __L2LLS
        CALL      __L2LLS            ;; 3 cycles
        MOVW      AX, N:_b_phase     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+34
        MOVW      AX, N:_cal_coeff+30  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+36
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+44
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
          CFI FunCall _calculate_vrms
        CALL      _calculate_vrms    ;; 3 cycles
        MOVW      N:_vol+16, AX      ;; 1 cycle
// 1984         b_phase.vol.dc_offset = b_phase.vol.offset_acc_cnt / b_phase.no_of_samples;
        MOVW      AX, N:_b_phase     ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+46
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+48
        MOVW      BC, N:_b_phase+20  ;; 1 cycle
        MOVW      AX, N:_b_phase+18  ;; 1 cycle
          CFI FunCall ?SL_DIV_L03
        CALL      N:?SL_DIV_L03      ;; 3 cycles
        MOVW      N:_b_phase+14, AX  ;; 1 cycle
// 1985         
// 1986         /* detecting low voltage or low current*/
// 1987         if(vol.Bph.rms >= THR_PHASE_PRESENT_VOL)
        MOVW      AX, N:_vol+12      ;; 1 cycle
        ADDW      SP, #0x14          ;; 1 cycle
          CFI CFA SP+28
        CMPW      AX, #0x7D0         ;; 1 cycle
        BC        ??power_filter_225  ;; 4 cycles
        ; ------------------------------------- Block: 118 cycles
// 1988         {
// 1989             flag_phase_present_b = 1;
        SET1      N:_flag_metro2.2   ;; 2 cycles
        BR        S:??power_filter_226  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 1990         }
// 1991         else
// 1992         {
// 1993             flag_phase_present_b = 0;
??power_filter_225:
        CLR1      N:_flag_metro2.2   ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
// 1994         }  
// 1995         /* detecting healthy voltage for frequency calculation */
// 1996         if(vol.Bph.rms >= THR_FREQ_DECISION)
??power_filter_226:
        MOVW      AX, N:_vol+12      ;; 1 cycle
        CMPW      AX, #0x1D4C        ;; 1 cycle
        BC        ??power_filter_227  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1997         {
// 1998             flag_freq_vol_b = 1;
        SET1      N:_flag_metro3.2   ;; 2 cycles
        BR        S:??power_filter_228  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 1999         }
// 2000         else
// 2001         {
// 2002             flag_freq_vol_b = 0;
??power_filter_227:
        CLR1      N:_flag_metro3.2   ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
// 2003         }  
// 2004         if(curr.Bph.rms < MIN_CURRENT)
??power_filter_228:
        MOVW      BC, N:_curr+34     ;; 1 cycle
        MOVW      AX, N:_curr+32     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
        CMPW      AX, #0xA           ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??metrology_process_14:
        BNC       ??power_filter_229  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
// 2005         {
// 2006             flag_min_current_b = 1;
        SET1      N:_flag_metro2.6   ;; 2 cycles
        BR        S:??power_filter_230  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 2007         }
// 2008         else
// 2009         {
// 2010             flag_min_current_b = 0;
??power_filter_229:
        CLR1      N:_flag_metro2.6   ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
// 2011         }
// 2012         /* Frequency */
// 2013         if(flag_phase_present_b == 1)
??power_filter_230:
        MOVW      HL, #LWRD(_flag_metro2)  ;; 1 cycle
        MOV1      CY, [HL].2         ;; 1 cycle
        BNC       ??power_filter_231  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 2014         {
// 2015             freq.Bph = (us16)((us32)195312500 / b_phase.no_of_samples);
        MOVW      DE, N:_b_phase     ;; 1 cycle
        MOVW      HL, #0x0           ;; 1 cycle
        MOVW      AX, #0x3B74        ;; 1 cycle
        MOVW      BC, #0xBA4         ;; 1 cycle
        DIVWU                        ;; 17 cycles
        NOP                          ;; 1 cycle
        MOVW      N:_freq+4, AX      ;; 1 cycle
        BR        S:??power_filter_232  ;; 3 cycles
        ; ------------------------------------- Block: 26 cycles
// 2016         }
// 2017         else
// 2018         {
// 2019             freq.Bph = 0;
??power_filter_231:
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_freq+4, AX      ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
// 2020         }
// 2021         
// 2022         /* Active Energy */
// 2023         if(b_phase.active.acc_cnt < 0)
??power_filter_232:
        MOVW      BC, #LWRD(__Constant_0_0)  ;; 1 cycle
        MOVW      AX, #LWRD(_b_phase+72)  ;; 1 cycle
          CFI FunCall __CmpGes64
        CALL      __CmpGes64         ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_233  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
// 2024         {
// 2025             temp_us64 = ~b_phase.active.acc_cnt + 1;
        MOVW      BC, #LWRD(_b_phase+72)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
          CFI FunCall __Not64
        CALL      __Not64            ;; 3 cycles
        MOVW      DE, #LWRD(__Constant_1_0)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Add64
        CALL      __Add64            ;; 3 cycles
        MOVW      HL, #LWRD(_temp_us64)  ;; 1 cycle
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
// 2026             flag_Bph_active = EXPORT;
        SET1      N:_flag_quadrant.4  ;; 2 cycles
        BR        S:??power_filter_234  ;; 3 cycles
        ; ------------------------------------- Block: 30 cycles
// 2027         }
// 2028         else
// 2029         {
// 2030             temp_us64 = b_phase.active.acc_cnt;
??power_filter_233:
        MOVW      HL, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      DE, #LWRD(_b_phase+72)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 2031             flag_Bph_active = IMPORT;
        CLR1      N:_flag_quadrant.4  ;; 2 cycles
        ; ------------------------------------- Block: 12 cycles
// 2032         }
// 2033         temp_us64 *= cal_coeff.Bph.power;
??power_filter_234:
        MOVW      AX, N:_cal_coeff+34  ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+30
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
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
// 2034         temp_us64 /= b_phase.no_of_samples;
        MOVW      AX, N:_b_phase     ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+34
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+36
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
// 2035         b_phase.active.delta_cnts_temp = temp_us64;
        MOVW      HL, #LWRD(_b_phase+104)  ;; 1 cycle
        MOVW      DE, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 2036         
// 2037         /* Reactive Energy */
// 2038         if(flag_metro_react_cal_method == REACT_CAL_DELAY)
        MOVW      HL, #LWRD(_flag_metro1)  ;; 1 cycle
        MOV1      CY, [HL].3         ;; 1 cycle
        ADDW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+28
        SKNC                         ;; 4 cycles
        BR        N:??power_filter_235  ;; 4 cycles
        ; ------------------------------------- Block: 51 cycles
// 2039         {
// 2040             if(b_phase.reactive.acc_cnt < 0)
        MOVW      BC, #LWRD(__Constant_0_0)  ;; 1 cycle
        MOVW      AX, #LWRD(_b_phase+112)  ;; 1 cycle
          CFI FunCall __CmpGes64
        CALL      __CmpGes64         ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_236  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
// 2041             {
// 2042                 temp_us64 = ~b_phase.reactive.acc_cnt + 1;
        MOVW      BC, #LWRD(_b_phase+112)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
          CFI FunCall __Not64
        CALL      __Not64            ;; 3 cycles
        MOVW      DE, #LWRD(__Constant_1_0)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Add64
        CALL      __Add64            ;; 3 cycles
        MOVW      HL, #LWRD(_temp_us64)  ;; 1 cycle
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
// 2043                 flag_Bph_reactive = EXPORT;
        SET1      N:_flag_quadrant.5  ;; 2 cycles
        BR        S:??power_filter_237  ;; 3 cycles
        ; ------------------------------------- Block: 30 cycles
// 2044             }
// 2045             else
// 2046             {
// 2047                 temp_us64 = b_phase.reactive.acc_cnt;
??power_filter_236:
        MOVW      HL, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      DE, #LWRD(_b_phase+112)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 2048                 flag_Bph_reactive = IMPORT;
        CLR1      N:_flag_quadrant.5  ;; 2 cycles
        ; ------------------------------------- Block: 12 cycles
// 2049             }
// 2050             temp_us64 *= cal_coeff.Bph.power;
??power_filter_237:
        MOVW      AX, N:_cal_coeff+34  ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+30
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
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
// 2051             temp_us64 /= b_phase.no_of_samples;
        MOVW      AX, N:_b_phase     ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+34
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+36
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
// 2052             b_phase.reactive.delta_cnts_temp = temp_us64;
        MOVW      HL, #LWRD(_b_phase+144)  ;; 1 cycle
        MOVW      DE, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        ADDW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+28
        ; ------------------------------------- Block: 45 cycles
// 2053         }
// 2054         
// 2055         /* Apparent Energy */
// 2056         if(flag_metro_app_cal_method == APP_CAL_VI)
??power_filter_235:
        MOVW      HL, #LWRD(_flag_metro1)  ;; 1 cycle
        MOV1      CY, [HL].1         ;; 1 cycle
        BC        ??power_filter_238  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 2057         {
// 2058             b_phase.apparent.delta_cnts_temp = ((us64)curr.Bph.rms * vol.Bph.rms)*10;
        MOVW      AX, N:_curr+34     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+30
        MOVW      AX, N:_curr+32     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
          CFI FunCall __L2LLU
        CALL      __L2LLU            ;; 3 cycles
        MOVW      AX, N:_vol+12      ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+34
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+36
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
          CFI FunCall __L2LLU
        CALL      __L2LLU            ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x18          ;; 1 cycle
          CFI FunCall __Mul64
        CALL      __Mul64            ;; 3 cycles
        MOVW      DE, #LWRD(__Constant_a_0)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x18          ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, #LWRD(_b_phase+184)  ;; 1 cycle
          CFI FunCall __Mul64
        CALL      __Mul64            ;; 3 cycles
        ADDW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+28
        BR        N:??power_filter_239  ;; 3 cycles
        ; ------------------------------------- Block: 41 cycles
// 2059         }
// 2060         else
// 2061         {
// 2062             temp_us64 = (us64)((us64)b_phase.active.delta_cnts_temp/100)*((us64)b_phase.active.delta_cnts_temp/100);
??power_filter_238:
        MOVW      DE, #LWRD(__Constant_64_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_b_phase+104)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
        MOVW      DE, #LWRD(__Constant_64_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_b_phase+104)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __Mul64
        CALL      __Mul64            ;; 3 cycles
// 2063             temp_us64 += (us64)((us64)b_phase.reactive.delta_cnts_temp/100)*((us64)b_phase.reactive.delta_cnts_temp/100);
        MOVW      DE, #LWRD(__Constant_64_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_b_phase+144)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
        MOVW      DE, #LWRD(__Constant_64_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_b_phase+144)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
          CFI FunCall __Mul64
        CALL      __Mul64            ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      BC, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __Add64
        CALL      __Add64            ;; 3 cycles
// 2064             b_phase.apparent.delta_cnts_temp = (us64)sqrt(temp_us64)*100;
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __LLU2F
        CALL      __LLU2F            ;; 3 cycles
          CFI FunCall _sqrt
        CALL      _sqrt              ;; 3 cycles
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+30
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
          CFI FunCall __F2LLU
        CALL      __F2LLU            ;; 3 cycles
        MOVW      DE, #LWRD(__Constant_64_0)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, #LWRD(_b_phase+184)  ;; 1 cycle
          CFI FunCall __Mul64
        CALL      __Mul64            ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+28
        ; ------------------------------------- Block: 76 cycles
// 2065         }
// 2066         if((b_phase.active.delta_cnts_temp > b_phase.apparent.delta_cnts_temp) && (SP_CHECK_ACT_APP == 1) && (ZERORISE_METROLOGY_PAR == 1))
??power_filter_239:
        MOVW      BC, #LWRD(_b_phase+104)  ;; 1 cycle
        MOVW      AX, #LWRD(_b_phase+184)  ;; 1 cycle
          CFI FunCall __CmpGeu64
        CALL      __CmpGeu64         ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_240  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
// 2067         {
// 2068             b_phase.apparent.delta_cnts_temp =  b_phase.active.delta_cnts_temp;
        MOVW      HL, #LWRD(_b_phase+184)  ;; 1 cycle
        MOVW      DE, #LWRD(_b_phase+104)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        ; ------------------------------------- Block: 10 cycles
// 2069         }
// 2070         
// 2071         /* Reactive energy */
// 2072         if(flag_metro_react_cal_method == REACT_CAL_POW_TRIANGLE)
??power_filter_240:
        MOVW      HL, #LWRD(_flag_metro1)  ;; 1 cycle
        MOV1      CY, [HL].3         ;; 1 cycle
        SKC                          ;; 4 cycles
        BR        N:??power_filter_241  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 2073         {
// 2074             if(b_phase.reactive.acc_cnt < 0) /* there may be issues in the calculation of quadrants at boundry conditions at specific freq points*/
        MOVW      BC, #LWRD(__Constant_0_0)  ;; 1 cycle
        MOVW      AX, #LWRD(_b_phase+112)  ;; 1 cycle
          CFI FunCall __CmpGes64
        CALL      __CmpGes64         ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_242  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
// 2075             {
// 2076                 flag_Bph_reactive = EXPORT;
        SET1      N:_flag_quadrant.5  ;; 2 cycles
        BR        S:??power_filter_243  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 2077             }
// 2078             else
// 2079             {
// 2080                 flag_Bph_reactive = IMPORT;
??power_filter_242:
        CLR1      N:_flag_quadrant.5  ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
// 2081             }
// 2082             temp_us64 = (us64)((us64)b_phase.apparent.delta_cnts_temp/100)*((us64)b_phase.apparent.delta_cnts_temp/100);
??power_filter_243:
        MOVW      DE, #LWRD(__Constant_64_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_b_phase+184)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
        MOVW      DE, #LWRD(__Constant_64_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_b_phase+184)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __Mul64
        CALL      __Mul64            ;; 3 cycles
// 2083             temp_us64 -= (us64)((us64)b_phase.active.delta_cnts_temp/100)*((us64)b_phase.active.delta_cnts_temp/100);
        MOVW      DE, #LWRD(__Constant_64_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_b_phase+104)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
        MOVW      DE, #LWRD(__Constant_64_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_b_phase+104)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
          CFI FunCall __Mul64
        CALL      __Mul64            ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      BC, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __Sub64
        CALL      __Sub64            ;; 3 cycles
// 2084             b_phase.reactive.delta_cnts_temp = (us64)sqrt(temp_us64)*100;
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __LLU2F
        CALL      __LLU2F            ;; 3 cycles
          CFI FunCall _sqrt
        CALL      _sqrt              ;; 3 cycles
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+30
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
          CFI FunCall __F2LLU
        CALL      __F2LLU            ;; 3 cycles
        MOVW      DE, #LWRD(__Constant_64_0)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, #LWRD(_b_phase+144)  ;; 1 cycle
          CFI FunCall __Mul64
        CALL      __Mul64            ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+28
        ; ------------------------------------- Block: 76 cycles
// 2085         }
// 2086         /* Special checks */
// 2087         /* checks : 
// 2088         1. magnet tamper 
// 2089         2. ZPF
// 2090         3. UPF 
// 2091         4. Low power active
// 2092         5. Low power reactive */
// 2093        
// 2094         temp_us16 = cal_pf(b_phase.active.delta_cnts_temp / 100000, b_phase.apparent.delta_cnts_temp / 100000);
??power_filter_241:
        MOVW      DE, #LWRD(__Constant_186a0_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_b_phase+184)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __LLU2L
        CALL      __LLU2L            ;; 3 cycles
        MOVW      [SP+0x08], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [SP+0x0A], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      DE, #LWRD(__Constant_186a0_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_b_phase+104)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __LLU2L
        CALL      __LLU2L            ;; 3 cycles
        MOVW      DE, AX             ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+30
        POP       HL                 ;; 1 cycle
          CFI CFA SP+28
        MOVW      AX, [SP+0x0A]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x08]      ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+30
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
        MOVW      AX, DE             ;; 1 cycle
        PUSH      HL                 ;; 1 cycle
          CFI CFA SP+34
        POP       BC                 ;; 1 cycle
          CFI CFA SP+32
          CFI FunCall _cal_pf
        CALL      _cal_pf            ;; 3 cycles
        MOVW      S:_temp_us16, AX   ;; 1 cycle
// 2095         
// 2096         if(temp_us16 < 40 && (SP_CHECK_ZPF == 1) && (ZERORISE_METROLOGY_PAR == 1))       /* ZPF */
        MOVW      AX, S:_temp_us16   ;; 1 cycle
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+28
        CMPW      AX, #0x28          ;; 1 cycle
        BNC       ??power_filter_244  ;; 4 cycles
        ; ------------------------------------- Block: 46 cycles
// 2097         {
// 2098             b_phase.active.delta_cnts_temp = 0;
        MOVW      HL, #LWRD(_b_phase+104)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 2099             b_phase.apparent.delta_cnts_temp =  b_phase.reactive.delta_cnts_temp;
        MOVW      HL, #LWRD(_b_phase+184)  ;; 1 cycle
        MOVW      DE, #LWRD(_b_phase+144)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 2100             flag_Bph_active = IMPORT;
        CLR1      N:_flag_quadrant.4  ;; 2 cycles
        ; ------------------------------------- Block: 21 cycles
// 2101         }
// 2102         if(temp_us16 > 998 && (SP_CHECK_UPF == 1) && (ZERORISE_METROLOGY_PAR == 1) && curr.Bph.rms >= DIAL_IMAX_CURR)       /* UPF */
??power_filter_244:
        MOVW      AX, S:_temp_us16   ;; 1 cycle
        CMPW      AX, #0x3E7         ;; 1 cycle
        BC        ??power_filter_245  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      BC, N:_curr+34     ;; 1 cycle
        MOVW      AX, N:_curr+32     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
        CMPW      AX, #0xE678        ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??metrology_process_15:
        BC        ??power_filter_245  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
// 2103         {
// 2104             b_phase.reactive.delta_cnts_temp = 0;
        MOVW      HL, #LWRD(_b_phase+144)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 2105             b_phase.apparent.delta_cnts_temp =  b_phase.active.delta_cnts_temp;
        MOVW      HL, #LWRD(_b_phase+184)  ;; 1 cycle
        MOVW      DE, #LWRD(_b_phase+104)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 2106             flag_Bph_reactive = IMPORT;
        CLR1      N:_flag_quadrant.5  ;; 2 cycles
        ; ------------------------------------- Block: 21 cycles
// 2107         }
// 2108         if((b_phase.active.delta_cnts_temp / 100000) < 18)
??power_filter_245:
        MOVW      DE, #LWRD(__Constant_186a0_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_b_phase+104)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
        MOVW      BC, #LWRD(__Constant_12_0)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __CmpGeu64
        CALL      __CmpGeu64         ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_246  ;; 4 cycles
        ; ------------------------------------- Block: 16 cycles
// 2109         {
// 2110             b_phase.active.delta_cnts_temp = 0;
        MOVW      HL, #LWRD(_b_phase+104)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 2111             b_phase.apparent.delta_cnts_temp =  b_phase.reactive.delta_cnts_temp;
        MOVW      HL, #LWRD(_b_phase+184)  ;; 1 cycle
        MOVW      DE, #LWRD(_b_phase+144)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 2112             flag_Bph_active = IMPORT;
        CLR1      N:_flag_quadrant.4  ;; 2 cycles
        ; ------------------------------------- Block: 21 cycles
// 2113         }
// 2114         if((b_phase.reactive.delta_cnts_temp / 100000) < 18)
??power_filter_246:
        MOVW      DE, #LWRD(__Constant_186a0_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_b_phase+144)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
        MOVW      BC, #LWRD(__Constant_12_0)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __CmpGeu64
        CALL      __CmpGeu64         ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_247  ;; 4 cycles
        ; ------------------------------------- Block: 16 cycles
// 2115         {
// 2116             b_phase.reactive.delta_cnts_temp = 0;
        MOVW      HL, #LWRD(_b_phase+144)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 2117             b_phase.apparent.delta_cnts_temp =  b_phase.active.delta_cnts_temp;
        MOVW      HL, #LWRD(_b_phase+184)  ;; 1 cycle
        MOVW      DE, #LWRD(_b_phase+104)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 2118             flag_Bph_reactive = IMPORT;
        CLR1      N:_flag_quadrant.5  ;; 2 cycles
        ; ------------------------------------- Block: 21 cycles
// 2119         }
// 2120         if(bitIsSet(tpr.magnet.flag,event_f) && (MAG_SWITCH_IMAX == 1) && (flag_phase_present_b == 1))
??power_filter_247:
        MOVW      HL, #LWRD(_tpr+32)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??power_filter_248  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      HL, #LWRD(_flag_metro2)  ;; 1 cycle
        MOV1      CY, [HL].2         ;; 1 cycle
        BNC       ??power_filter_248  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 2121         {
// 2122             b_phase.active.delta_cnts_temp = MAG_DELTA_CNTS_IMAX;
        MOVW      HL, #LWRD(_b_phase+104)  ;; 1 cycle
        MOVW      AX, #0x9000        ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x5A4E        ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x3           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 2123             b_phase.reactive.delta_cnts_temp = 0;
        MOVW      HL, #LWRD(_b_phase+144)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 2124             b_phase.apparent.delta_cnts_temp =  b_phase.active.delta_cnts_temp;
        MOVW      HL, #LWRD(_b_phase+184)  ;; 1 cycle
        MOVW      DE, #LWRD(_b_phase+104)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 2125             flag_Bph_active = IMPORT;
        CLR1      N:_flag_quadrant.4  ;; 2 cycles
// 2126             flag_Bph_reactive = IMPORT;
        CLR1      N:_flag_quadrant.5  ;; 2 cycles
        ; ------------------------------------- Block: 32 cycles
// 2127         }
// 2128         
// 2129         if(ZERORISE_METROLOGY_PAR == 0 || 
// 2130            (flag_phase_present_b == 1 && flag_min_current_b == 0 && ZERORISE_METROLOGY_PAR == 1))
??power_filter_248:
        MOV       A, N:_flag_metro2  ;; 1 cycle
        AND       A, #0x44           ;; 1 cycle
        CMP       A, #0x4            ;; 1 cycle
        SKZ                          ;; 4 cycles
        BR        N:??power_filter_249  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
// 2131         {
// 2132             b_phase.active.delta_cnts = b_phase.active.delta_cnts_temp;
        MOVW      HL, #LWRD(_b_phase+96)  ;; 1 cycle
        MOVW      DE, #LWRD(_b_phase+104)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 2133             b_phase.reactive.delta_cnts = b_phase.reactive.delta_cnts_temp;
        MOVW      HL, #LWRD(_b_phase+136)  ;; 1 cycle
        MOVW      DE, #LWRD(_b_phase+144)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 2134             b_phase.apparent.delta_cnts = b_phase.apparent.delta_cnts_temp;
        MOVW      HL, #LWRD(_b_phase+176)  ;; 1 cycle
        MOVW      DE, #LWRD(_b_phase+184)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 2135             
// 2136             b_phase.fundamental.active.delta_cnts = b_phase.active.delta_cnts * 1000;
        MOVW      DE, #LWRD(__Constant_3e8_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_b_phase+96)  ;; 1 cycle
        MOVW      AX, #LWRD(_b_phase+200)  ;; 1 cycle
          CFI FunCall __Mul64
        CALL      __Mul64            ;; 3 cycles
// 2137             b_phase.fundamental.active.delta_cnts /= (1000+thd.Bph.correction);
        MOV       X, N:_thd+52       ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        ADDW      AX, #0x3E8         ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+30
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
          CFI FunCall __L2LLS
        CALL      __L2LLS            ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      BC, #LWRD(_b_phase+200)  ;; 1 cycle
        MOVW      AX, #LWRD(_b_phase+200)  ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
// 2138             
// 2139             if(bitIsSet(tpr.magnet.flag,event_f) && (MAG_SWITCH_IMAX == 1))
        MOVW      HL, #LWRD(_tpr+32)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+28
        SKC                          ;; 4 cycles
        BR        N:??power_filter_250  ;; 4 cycles
        ; ------------------------------------- Block: 64 cycles
// 2140             {
// 2141                 curr.Bph.rms = IMAX;
        MOVW      AX, #0xEA60        ;; 1 cycle
        MOVW      N:_curr+32, AX     ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+34, AX     ;; 1 cycle
// 2142                 curr.Bph.dc = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+40, AX     ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+42, AX     ;; 1 cycle
// 2143                 curr.Bph.rms_signed = IMAX;
        MOVW      AX, #0xEA60        ;; 1 cycle
        MOVW      N:_curr+44, AX     ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+46, AX     ;; 1 cycle
// 2144                 vol.Bph.rms = VREF;
        MOVW      AX, #0x5DC0        ;; 1 cycle
        MOVW      N:_vol+12, AX      ;; 1 cycle
        BR        N:??power_filter_250  ;; 3 cycles
        ; ------------------------------------- Block: 17 cycles
// 2145             }
// 2146         }
// 2147         else
// 2148         {
// 2149 //            b_phase.active.acc_cnt = 0;
// 2150 //            b_phase.reactive.acc_cnt = 0;
// 2151 //            b_phase.apparent.acc_cnt = 0;
// 2152             if(bitIsSet(tpr.magnet.flag,event_f) && (MAG_SWITCH_IMAX == 1))
??power_filter_249:
        MOVW      HL, #LWRD(_tpr+32)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??power_filter_251  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 2153             {
// 2154                 NOP();
        NOP                          ;; 1 cycle
        BR        S:??power_filter_252  ;; 3 cycles
        ; ------------------------------------- Block: 4 cycles
// 2155             }
// 2156             else
// 2157             {
// 2158                 if(EMPTY_BUCKET == 1) 
// 2159                 {
// 2160                     b_phase.active.acc_delta_cnts = 0;
??power_filter_251:
        MOVW      HL, #LWRD(_b_phase+88)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 2161                     b_phase.reactive.acc_delta_cnts = 0;
        MOVW      HL, #LWRD(_b_phase+128)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 2162                     b_phase.apparent.acc_delta_cnts = 0;
        MOVW      HL, #LWRD(_b_phase+168)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 2163                     b_phase.fundamental.active.delta_cnts = 0;
        MOVW      HL, #LWRD(_b_phase+200)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        ; ------------------------------------- Block: 36 cycles
// 2164                 }
// 2165             }
// 2166                    
// 2167             if(flag_phase_present_b == 0 && flag_min_current_b == 1)      /* when vol <20 and curr < 10 */
??power_filter_252:
        MOV       A, N:_flag_metro2  ;; 1 cycle
        AND       A, #0x44           ;; 1 cycle
        CMP       A, #0x40           ;; 1 cycle
        BNZ       ??power_filter_253  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
// 2168             {
// 2169                 vol.Bph.rms = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_vol+12, AX      ;; 1 cycle
// 2170                 vol.Bph.dc = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_vol+16, AX      ;; 1 cycle
// 2171                 curr.Bph.rms = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+32, AX     ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+34, AX     ;; 1 cycle
// 2172                 curr.Bph.dc = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+40, AX     ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+42, AX     ;; 1 cycle
// 2173                 curr.Bph.rms_signed = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+44, AX     ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+46, AX     ;; 1 cycle
// 2174                 b_phase.active.delta_cnts_temp = 0;
        MOVW      HL, #LWRD(_b_phase+104)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 2175                 b_phase.reactive.delta_cnts_temp = 0;
        MOVW      HL, #LWRD(_b_phase+144)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 2176                 b_phase.apparent.delta_cnts_temp = 0;
        MOVW      HL, #LWRD(_b_phase+184)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        BR        N:??power_filter_254  ;; 3 cycles
        ; ------------------------------------- Block: 46 cycles
// 2177             }
// 2178             else if(flag_phase_present_b == 0 && flag_min_current_b == 0) /* when vol <20 and curr >= 10 */
??power_filter_253:
        MOV       A, N:_flag_metro2  ;; 1 cycle
        AND       A, #0x44           ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_255  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
// 2179             {
// 2180                 vol.Bph.rms = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_vol+12, AX      ;; 1 cycle
// 2181                 vol.Bph.dc = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_vol+16, AX      ;; 1 cycle
// 2182                 //                curr.Bph.rms = 0;
// 2183                 //                curr.Bph.dc = 0;
// 2184                 //                curr.Bph.rms_signed = 0;
// 2185                 b_phase.active.delta_cnts_temp = 0;
        MOVW      HL, #LWRD(_b_phase+104)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 2186                 b_phase.reactive.delta_cnts_temp = 0;
        MOVW      HL, #LWRD(_b_phase+144)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 2187                 b_phase.apparent.delta_cnts_temp = 0;
        MOVW      HL, #LWRD(_b_phase+184)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        BR        N:??power_filter_254  ;; 3 cycles
        ; ------------------------------------- Block: 34 cycles
// 2188             }
// 2189             else if(flag_phase_present_b == 1 && flag_min_current_b == 1) /* when vol > 20 and curr < 10 */
??power_filter_255:
        MOV       A, N:_flag_metro2  ;; 1 cycle
        AND       A, #0x44           ;; 1 cycle
        CMP       A, #0x44           ;; 1 cycle
        SKZ                          ;; 4 cycles
        BR        N:??power_filter_254  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
// 2190             {
// 2191                 if(bitIsSet(tpr.magnet.flag,event_f) && (MAG_SWITCH_IMAX == 1))
        MOVW      HL, #LWRD(_tpr+32)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??power_filter_256  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 2192                 {
// 2193                     curr.Bph.rms = IMAX;
        MOVW      AX, #0xEA60        ;; 1 cycle
        MOVW      N:_curr+32, AX     ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+34, AX     ;; 1 cycle
// 2194                     curr.Bph.dc = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+40, AX     ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+42, AX     ;; 1 cycle
// 2195                     curr.Bph.rms_signed = IMAX;
        MOVW      AX, #0xEA60        ;; 1 cycle
        MOVW      N:_curr+44, AX     ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+46, AX     ;; 1 cycle
// 2196                     vol.Bph.rms = VREF;
        MOVW      AX, #0x5DC0        ;; 1 cycle
        MOVW      N:_vol+12, AX      ;; 1 cycle
        BR        S:??power_filter_254  ;; 3 cycles
        ; ------------------------------------- Block: 17 cycles
// 2197                 }
// 2198                 else
// 2199                 {
// 2200                     curr.Bph.rms = 0;
??power_filter_256:
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+32, AX     ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+34, AX     ;; 1 cycle
// 2201                     curr.Bph.dc = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+40, AX     ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+42, AX     ;; 1 cycle
// 2202                     curr.Bph.rms_signed = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+44, AX     ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+46, AX     ;; 1 cycle
// 2203                     b_phase.active.delta_cnts_temp = 0;
        MOVW      HL, #LWRD(_b_phase+104)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 2204                     b_phase.reactive.delta_cnts_temp = 0;
        MOVW      HL, #LWRD(_b_phase+144)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 2205                     b_phase.apparent.delta_cnts_temp = 0;
        MOVW      HL, #LWRD(_b_phase+184)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        ; ------------------------------------- Block: 39 cycles
// 2206                 }
// 2207             }
// 2208             flag_Bph_active = IMPORT;
??power_filter_254:
        CLR1      N:_flag_quadrant.4  ;; 2 cycles
// 2209             flag_Bph_reactive = IMPORT;
        CLR1      N:_flag_quadrant.5  ;; 2 cycles
// 2210             b_phase.active.delta_cnts = b_phase.active.delta_cnts_temp;
        MOVW      HL, #LWRD(_b_phase+96)  ;; 1 cycle
        MOVW      DE, #LWRD(_b_phase+104)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 2211             b_phase.reactive.delta_cnts = b_phase.reactive.delta_cnts_temp;
        MOVW      HL, #LWRD(_b_phase+136)  ;; 1 cycle
        MOVW      DE, #LWRD(_b_phase+144)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 2212             b_phase.apparent.delta_cnts = b_phase.apparent.delta_cnts_temp;
        MOVW      HL, #LWRD(_b_phase+176)  ;; 1 cycle
        MOVW      DE, #LWRD(_b_phase+184)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 2213             
// 2214             b_phase.fundamental.active.delta_cnts = b_phase.active.delta_cnts * 1000;
        MOVW      DE, #LWRD(__Constant_3e8_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_b_phase+96)  ;; 1 cycle
        MOVW      AX, #LWRD(_b_phase+200)  ;; 1 cycle
          CFI FunCall __Mul64
        CALL      __Mul64            ;; 3 cycles
// 2215             b_phase.fundamental.active.delta_cnts /= (1000+thd.Bph.correction);
        MOV       X, N:_thd+52       ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        ADDW      AX, #0x3E8         ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+30
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
          CFI FunCall __L2LLS
        CALL      __L2LLS            ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      BC, #LWRD(_b_phase+200)  ;; 1 cycle
        MOVW      AX, #LWRD(_b_phase+200)  ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+28
        ; ------------------------------------- Block: 62 cycles
// 2216         }
// 2217         
// 2218         /* Calculating powers */
// 2219         power.Bph.active = b_phase.active.delta_cnts / 100000;
??power_filter_250:
        MOVW      DE, #LWRD(__Constant_186a0_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_b_phase+96)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __LLU2L
        CALL      __LLU2L            ;; 3 cycles
        MOVW      N:_power+40, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_power+42, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 2220         power.Bph.reactive = b_phase.reactive.delta_cnts / 100000;
        MOVW      DE, #LWRD(__Constant_186a0_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_b_phase+136)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __LLU2L
        CALL      __LLU2L            ;; 3 cycles
        MOVW      N:_power+44, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_power+46, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 2221         power.Bph.apparent =  b_phase.apparent.delta_cnts / 100000;
        MOVW      DE, #LWRD(__Constant_186a0_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_b_phase+176)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __LLU2L
        CALL      __LLU2L            ;; 3 cycles
        MOVW      N:_power+48, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_power+50, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 2222         pf.Bph = cal_pf(power.Bph.active,power.Bph.apparent);
        MOVW      AX, N:_power+50    ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+30
        MOVW      AX, N:_power+48    ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
        MOVW      BC, N:_power+42    ;; 1 cycle
        MOVW      AX, N:_power+40    ;; 1 cycle
          CFI FunCall _cal_pf
        CALL      _cal_pf            ;; 3 cycles
        MOVW      N:_pf+4, AX        ;; 1 cycle
// 2223         
// 2224         /* Quadrant */
// 2225         quadrant.Bph = get_quadrant(flag_Bph_active,flag_Bph_reactive);
        MOVW      HL, #LWRD(_flag_quadrant)  ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOV1      CY, [HL].5         ;; 1 cycle
        ROLC      A, 0x1             ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOVW      HL, #LWRD(_flag_quadrant)  ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOV1      CY, [HL].4         ;; 1 cycle
        ROLC      A, 0x1             ;; 1 cycle
          CFI FunCall _get_quadrant
        CALL      _get_quadrant      ;; 3 cycles
        MOV       N:_quadrant+2, A   ;; 1 cycle
// 2226         
// 2227         if(METERING_MODE == NET)
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+28
        CMP       N:_METERING_MODE, #0x1  ;; 1 cycle
        SKZ                          ;; 4 cycles
        BR        N:??power_filter_257  ;; 4 cycles
        ; ------------------------------------- Block: 71 cycles
// 2228         {
// 2229             /* Active related */
// 2230             if(quadrant.Bph == Q1 || quadrant.Bph == Q4)   
        CMP       N:_quadrant+2, #0x1  ;; 1 cycle
        BZ        ??power_filter_258  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        CMP       N:_quadrant+2, #0x4  ;; 1 cycle
        BNZ       ??power_filter_259  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 2231             {
// 2232                 power.Bph.active_signed = power.Bph.active;
??power_filter_258:
        MOVW      BC, N:_power+42    ;; 1 cycle
        MOVW      AX, N:_power+40    ;; 1 cycle
        MOVW      N:_power+52, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_power+54, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 2233                 curr.Bph.rms_signed = curr.Bph.rms;
        MOVW      BC, N:_curr+34     ;; 1 cycle
        MOVW      AX, N:_curr+32     ;; 1 cycle
        MOVW      N:_curr+44, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_curr+46, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 2234                 pf.Bph_signed = pf.Bph;
        MOVW      AX, N:_pf+4        ;; 1 cycle
        MOVW      N:_pf+12, AX       ;; 1 cycle
        BR        S:??power_filter_260  ;; 3 cycles
        ; ------------------------------------- Block: 17 cycles
// 2235             }
// 2236             else                                            
// 2237             {
// 2238                 power.Bph.active_signed = -power.Bph.active;
??power_filter_259:
        MOVW      BC, N:_power+42    ;; 1 cycle
        MOVW      AX, N:_power+40    ;; 1 cycle
          CFI FunCall ?L_NEG_L03
        CALL      N:?L_NEG_L03       ;; 3 cycles
        MOVW      N:_power+52, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_power+54, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 2239                 curr.Bph.rms_signed = -curr.Bph.rms;
        MOVW      BC, N:_curr+34     ;; 1 cycle
        MOVW      AX, N:_curr+32     ;; 1 cycle
          CFI FunCall ?L_NEG_L03
        CALL      N:?L_NEG_L03       ;; 3 cycles
        MOVW      N:_curr+44, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_curr+46, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 2240                 pf.Bph_signed = -pf.Bph;
        MOVW      AX, N:_pf+4        ;; 1 cycle
        XOR       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        XOR       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        INCW      AX                 ;; 1 cycle
        MOVW      N:_pf+12, AX       ;; 1 cycle
        ; ------------------------------------- Block: 25 cycles
// 2241             }
// 2242             
// 2243             /* Reactive related */
// 2244             if(quadrant.Bph == Q1 || quadrant.Bph == Q2)   
??power_filter_260:
        CMP       N:_quadrant+2, #0x1  ;; 1 cycle
        BZ        ??power_filter_261  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        CMP       N:_quadrant+2, #0x2  ;; 1 cycle
        BNZ       ??power_filter_262  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 2245             {
// 2246                 power.Bph.reactive_signed = power.Bph.reactive;
??power_filter_261:
        MOVW      BC, N:_power+46    ;; 1 cycle
        MOVW      AX, N:_power+44    ;; 1 cycle
        MOVW      N:_power+56, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_power+58, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        BR        N:??power_filter_263  ;; 3 cycles
        ; ------------------------------------- Block: 9 cycles
// 2247             }
// 2248             else                                         
// 2249             {
// 2250                 power.Bph.reactive_signed = -power.Bph.reactive;
??power_filter_262:
        MOVW      BC, N:_power+46    ;; 1 cycle
        MOVW      AX, N:_power+44    ;; 1 cycle
          CFI FunCall ?L_NEG_L03
        CALL      N:?L_NEG_L03       ;; 3 cycles
        MOVW      N:_power+56, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_power+58, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        BR        S:??power_filter_263  ;; 3 cycles
        ; ------------------------------------- Block: 12 cycles
// 2251             }
// 2252         }
// 2253         else 
// 2254         {
// 2255             /* Active related */
// 2256             power.Bph.active_signed = power.Bph.active;
??power_filter_257:
        MOVW      BC, N:_power+42    ;; 1 cycle
        MOVW      AX, N:_power+40    ;; 1 cycle
        MOVW      N:_power+52, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_power+54, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 2257             if(flag_Bph_active == EXPORT)
        MOVW      HL, #LWRD(_flag_quadrant)  ;; 1 cycle
        MOV1      CY, [HL].4         ;; 1 cycle
        BNC       ??power_filter_264  ;; 4 cycles
        ; ------------------------------------- Block: 12 cycles
// 2258             {
// 2259                 curr.Bph.rms_signed = -curr.Bph.rms;
        MOVW      BC, N:_curr+34     ;; 1 cycle
        MOVW      AX, N:_curr+32     ;; 1 cycle
          CFI FunCall ?L_NEG_L03
        CALL      N:?L_NEG_L03       ;; 3 cycles
        MOVW      N:_curr+44, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_curr+46, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        BR        S:??power_filter_265  ;; 3 cycles
        ; ------------------------------------- Block: 12 cycles
// 2260             }
// 2261             else
// 2262             {
// 2263                 curr.Bph.rms_signed = curr.Bph.rms;
??power_filter_264:
        MOVW      BC, N:_curr+34     ;; 1 cycle
        MOVW      AX, N:_curr+32     ;; 1 cycle
        MOVW      N:_curr+44, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_curr+46, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
// 2264             }
// 2265             
// 2266             /* Reactive related */
// 2267             if(quadrant.Bph == Q1 || quadrant.Bph == Q3)   
??power_filter_265:
        CMP       N:_quadrant+2, #0x1  ;; 1 cycle
        BZ        ??power_filter_266  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        CMP       N:_quadrant+2, #0x3  ;; 1 cycle
        BNZ       ??power_filter_267  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 2268             {
// 2269                 power.Bph.reactive_signed = power.Bph.reactive;
??power_filter_266:
        MOVW      BC, N:_power+46    ;; 1 cycle
        MOVW      AX, N:_power+44    ;; 1 cycle
        MOVW      N:_power+56, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_power+58, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 2270                 pf.Bph_signed = pf.Bph;
        MOVW      AX, N:_pf+4        ;; 1 cycle
        MOVW      N:_pf+12, AX       ;; 1 cycle
        BR        S:??power_filter_263  ;; 3 cycles
        ; ------------------------------------- Block: 11 cycles
// 2271             }
// 2272             else                                         
// 2273             {
// 2274                 power.Bph.reactive_signed = -power.Bph.reactive;
??power_filter_267:
        MOVW      BC, N:_power+46    ;; 1 cycle
        MOVW      AX, N:_power+44    ;; 1 cycle
          CFI FunCall ?L_NEG_L03
        CALL      N:?L_NEG_L03       ;; 3 cycles
        MOVW      N:_power+56, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_power+58, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 2275                 pf.Bph_signed = -pf.Bph;
        MOVW      AX, N:_pf+4        ;; 1 cycle
        XOR       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        XOR       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        INCW      AX                 ;; 1 cycle
        MOVW      N:_pf+12, AX       ;; 1 cycle
        ; ------------------------------------- Block: 16 cycles
// 2276             }
// 2277         }
// 2278 
// 2279         /* Phase angle */
// 2280         //        angle.Bph = cal_angle(power.Bph.active,power.Bph.reactive,power.Bph.apparent,flag_Bph_active,flag_Bph_reactive,1);
// 2281         
// 2282         if(flag_mag_update_metro_par == 1)
??power_filter_263:
        MOVW      HL, #LWRD(_flag_mag)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        SKNC                         ;; 1 cycle
        ; ------------------------------------- Block: 3 cycles
// 2283         {
// 2284             flag_mag_b_updated = 1;
        SET1      N:_flag_mag.3      ;; 2 cycles
          CFI FunCall _metrology_all_phase_calculation
        ; ------------------------------------- Block: 2 cycles
// 2285         }
// 2286         /* calculation for all phase variables */
// 2287         metrology_all_phase_calculation();
??metrology_process_16:
        CALL      _metrology_all_phase_calculation  ;; 3 cycles
// 2288         
// 2289         /* Calibration */
// 2290         if(cal_done_f != 1)
        CMP       N:_cal_done_f, #0x1  ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??power_filter_268  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
// 2291         {
// 2292             if(vol.Bph.rms >= CAL_VOL_MIN && vol.Bph.rms <= CAL_VOL_MAX && 
// 2293                curr.Bph.rms >= CAL_CURR_MIN && curr.Bph.rms <= CAL_CURR_MAX)
        MOVW      AX, N:_vol+12      ;; 1 cycle
        CMPW      AX, #0x55F0        ;; 1 cycle
        BC        ??power_filter_269  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, N:_vol+12      ;; 1 cycle
        CMPW      AX, #0x6591        ;; 1 cycle
        BNC       ??power_filter_269  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      BC, N:_curr+34     ;; 1 cycle
        MOVW      AX, N:_curr+32     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
        CMPW      AX, #0x1F40        ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??metrology_process_17:
        BC        ??power_filter_269  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
        MOVW      BC, N:_curr+34     ;; 1 cycle
        MOVW      AX, N:_curr+32     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
        CMPW      AX, #0x2EE1        ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??metrology_process_18:
        BNC       ??power_filter_269  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
// 2294             {
// 2295                 flag_calibration_b_vi_ok = 1;
        SET1      N:_flag_cal.2      ;; 2 cycles
        BR        S:??power_filter_270  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 2296             }
// 2297             else
// 2298             {
// 2299                 flag_calibration_b_vi_ok = 0;
??power_filter_269:
        CLR1      N:_flag_cal.2      ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
// 2300             }
// 2301             if(power.Bph.active >= CAL_POWER_MIN && power.Bph.active <= CAL_POWER_MAX &&
// 2302                freq.Bph >= CAL_FREQ_MIN && freq.Bph <= CAL_FREQ_MAX && 
// 2303                    pf.Bph >= CAL_PF_MIN)
??power_filter_270:
        MOVW      BC, N:_power+42    ;; 1 cycle
        MOVW      AX, N:_power+40    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
        CMPW      AX, #0x44C0        ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??metrology_process_19:
        BC        ??power_filter_271  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
        MOVW      BC, N:_power+42    ;; 1 cycle
        MOVW      AX, N:_power+40    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
        CMPW      AX, #0x79E1        ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??metrology_process_20:
        BNC       ??power_filter_271  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
        MOVW      AX, N:_freq+4      ;; 1 cycle
        CMPW      AX, #0xC288        ;; 1 cycle
        BC        ??power_filter_271  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, N:_freq+4      ;; 1 cycle
        CMPW      AX, #0xC419        ;; 1 cycle
        BNC       ??power_filter_271  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, N:_pf+4        ;; 1 cycle
        CMPW      AX, #0x384         ;; 1 cycle
        BC        ??power_filter_271  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 2304             {
// 2305                 flag_calibration_b_pf_freq_pow_ok = 1;
        SET1      N:_flag_calibration.4  ;; 2 cycles
        BR        S:??power_filter_272  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 2306             }
// 2307             else
// 2308             {
// 2309                 flag_calibration_b_pf_freq_pow_ok = 0;
??power_filter_271:
        CLR1      N:_flag_calibration.4  ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
// 2310             }
// 2311             if(flag_Bph_active == IMPORT)
??power_filter_272:
        MOVW      HL, #LWRD(_flag_quadrant)  ;; 1 cycle
        MOV1      CY, [HL].4         ;; 1 cycle
        BC        ??power_filter_273  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 2312             {
// 2313                 flag_calibration_b_ct_ok = 1;
        SET1      N:_flag_calibration.7  ;; 2 cycles
        BR        S:??power_filter_274  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 2314             }
// 2315             else
// 2316             {
// 2317                 flag_calibration_b_ct_ok = 0;
??power_filter_273:
        CLR1      N:_flag_calibration.7  ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
// 2318             }
// 2319             if(cal_BPh.l_shift.active.acc_cnt < 0)
??power_filter_274:
        MOVW      BC, #LWRD(__Constant_0_0)  ;; 1 cycle
        MOVW      AX, #LWRD(_cal_BPh+18)  ;; 1 cycle
          CFI FunCall __CmpGes64
        CALL      __CmpGes64         ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_275  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
// 2320             {
// 2321                 temp_us64 = ~cal_BPh.l_shift.active.acc_cnt + 1;
        MOVW      BC, #LWRD(_cal_BPh+18)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
          CFI FunCall __Not64
        CALL      __Not64            ;; 3 cycles
        MOVW      DE, #LWRD(__Constant_1_0)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Add64
        CALL      __Add64            ;; 3 cycles
        MOVW      HL, #LWRD(_temp_us64)  ;; 1 cycle
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
        BR        S:??power_filter_276  ;; 3 cycles
        ; ------------------------------------- Block: 28 cycles
// 2322             }
// 2323             else
// 2324             {
// 2325                 temp_us64 = cal_BPh.l_shift.active.acc_cnt;
??power_filter_275:
        MOVW      HL, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      DE, #LWRD(_cal_BPh+18)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        ; ------------------------------------- Block: 10 cycles
// 2326             }
// 2327             
// 2328             temp_us64 *= cal_coeff.Bph.power;
??power_filter_276:
        MOVW      AX, N:_cal_coeff+34  ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+30
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
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
// 2329             temp_us64 /= b_phase.no_of_samples;
        MOVW      AX, N:_b_phase     ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+34
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+36
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
// 2330             temp_us64 /= 100000;
        MOVW      DE, #LWRD(__Constant_186a0_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
// 2331             cal_BPh.l_shift.active.power = temp_us64;
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __LLU2L
        CALL      __LLU2L            ;; 3 cycles
        MOVW      N:_cal_BPh+14, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_cal_BPh+16, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 2332             if(cal_BPh.l_shift.active.acc_cnt < 0)
        MOVW      BC, #LWRD(__Constant_0_0)  ;; 1 cycle
        MOVW      AX, #LWRD(_cal_BPh+18)  ;; 1 cycle
          CFI FunCall __CmpGes64
        CALL      __CmpGes64         ;; 3 cycles
        ADDW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+28
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_277  ;; 4 cycles
        ; ------------------------------------- Block: 59 cycles
// 2333             {
// 2334                 cal_BPh.l_shift.active.power = ~cal_BPh.l_shift.active.power + 1;
        MOVW      BC, N:_cal_BPh+16  ;; 1 cycle
        MOVW      AX, N:_cal_BPh+14  ;; 1 cycle
          CFI FunCall ?L_NOT_L03
        CALL      N:?L_NOT_L03       ;; 3 cycles
        ADDW      AX, #0x1           ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_cal_BPh+14, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_cal_BPh+16, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ; ------------------------------------- Block: 18 cycles
// 2335             }
// 2336             
// 2337             if(cal_BPh.r_shift.active.acc_cnt < 0)
??power_filter_277:
        MOVW      BC, #LWRD(__Constant_0_0)  ;; 1 cycle
        MOVW      AX, #LWRD(_cal_BPh+62)  ;; 1 cycle
          CFI FunCall __CmpGes64
        CALL      __CmpGes64         ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_278  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
// 2338             {
// 2339                 temp_us64 = ~cal_BPh.r_shift.active.acc_cnt + 1;
        MOVW      BC, #LWRD(_cal_BPh+62)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
          CFI FunCall __Not64
        CALL      __Not64            ;; 3 cycles
        MOVW      DE, #LWRD(__Constant_1_0)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Add64
        CALL      __Add64            ;; 3 cycles
        MOVW      HL, #LWRD(_temp_us64)  ;; 1 cycle
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
        BR        S:??power_filter_279  ;; 3 cycles
        ; ------------------------------------- Block: 28 cycles
// 2340             }
// 2341             else
// 2342             {
// 2343                 temp_us64 = cal_BPh.r_shift.active.acc_cnt;
??power_filter_278:
        MOVW      HL, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      DE, #LWRD(_cal_BPh+62)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        ; ------------------------------------- Block: 10 cycles
// 2344             }
// 2345             
// 2346             temp_us64 *= cal_coeff.Bph.power;
??power_filter_279:
        MOVW      AX, N:_cal_coeff+34  ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+30
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
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
// 2347             temp_us64 /= b_phase.no_of_samples;
        MOVW      AX, N:_b_phase     ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+34
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+36
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
// 2348             temp_us64 /= 100000;
        MOVW      DE, #LWRD(__Constant_186a0_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
// 2349             cal_BPh.r_shift.active.power = temp_us64;
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __LLU2L
        CALL      __LLU2L            ;; 3 cycles
        MOVW      N:_cal_BPh+58, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_cal_BPh+60, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 2350             if(cal_BPh.r_shift.active.acc_cnt < 0)
        MOVW      BC, #LWRD(__Constant_0_0)  ;; 1 cycle
        MOVW      AX, #LWRD(_cal_BPh+62)  ;; 1 cycle
          CFI FunCall __CmpGes64
        CALL      __CmpGes64         ;; 3 cycles
        ADDW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+28
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_280  ;; 4 cycles
        ; ------------------------------------- Block: 59 cycles
// 2351             {
// 2352                 cal_BPh.r_shift.active.power = ~cal_BPh.r_shift.active.power + 1;
        MOVW      BC, N:_cal_BPh+60  ;; 1 cycle
        MOVW      AX, N:_cal_BPh+58  ;; 1 cycle
          CFI FunCall ?L_NOT_L03
        CALL      N:?L_NOT_L03       ;; 3 cycles
        ADDW      AX, #0x1           ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_cal_BPh+58, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_cal_BPh+60, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ; ------------------------------------- Block: 18 cycles
// 2353             }
// 2354             
// 2355             cal_BPh.angle_active = cal_angle_calculate_act(cal_BPh.l_shift.active.power,cal_BPh.r_shift.active.power);
??power_filter_280:
        MOVW      AX, N:_cal_BPh+60  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+30
        MOVW      AX, N:_cal_BPh+58  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
        MOVW      BC, N:_cal_BPh+16  ;; 1 cycle
        MOVW      AX, N:_cal_BPh+14  ;; 1 cycle
          CFI FunCall _cal_angle_calculate_act
        CALL      _cal_angle_calculate_act  ;; 3 cycles
        MOVW      N:_cal_BPh+6, AX   ;; 1 cycle
// 2356             
// 2357             
// 2358             if(cal_BPh.l_shift.reactive.acc_cnt < 0)
        MOVW      BC, #LWRD(__Constant_0_0)  ;; 1 cycle
        MOVW      AX, #LWRD(_cal_BPh+38)  ;; 1 cycle
          CFI FunCall __CmpGes64
        CALL      __CmpGes64         ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+28
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_281  ;; 4 cycles
        ; ------------------------------------- Block: 21 cycles
// 2359             {
// 2360                 temp_us64 = ~cal_BPh.l_shift.reactive.acc_cnt + 1;
        MOVW      BC, #LWRD(_cal_BPh+38)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
          CFI FunCall __Not64
        CALL      __Not64            ;; 3 cycles
        MOVW      DE, #LWRD(__Constant_1_0)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Add64
        CALL      __Add64            ;; 3 cycles
        MOVW      HL, #LWRD(_temp_us64)  ;; 1 cycle
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
        BR        S:??power_filter_282  ;; 3 cycles
        ; ------------------------------------- Block: 28 cycles
// 2361             }
// 2362             else
// 2363             {
// 2364                 temp_us64 = cal_BPh.l_shift.reactive.acc_cnt;
??power_filter_281:
        MOVW      HL, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      DE, #LWRD(_cal_BPh+38)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        ; ------------------------------------- Block: 10 cycles
// 2365             }
// 2366             
// 2367             temp_us64 *= cal_coeff.Bph.power;
??power_filter_282:
        MOVW      AX, N:_cal_coeff+34  ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+30
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
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
// 2368             temp_us64 /= b_phase.no_of_samples;
        MOVW      AX, N:_b_phase     ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+34
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+36
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
// 2369             temp_us64 /= 100000;
        MOVW      DE, #LWRD(__Constant_186a0_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
// 2370             cal_BPh.l_shift.reactive.power = temp_us64;
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __LLU2L
        CALL      __LLU2L            ;; 3 cycles
        MOVW      N:_cal_BPh+34, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_cal_BPh+36, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 2371             if(cal_BPh.l_shift.reactive.acc_cnt < 0)
        MOVW      BC, #LWRD(__Constant_0_0)  ;; 1 cycle
        MOVW      AX, #LWRD(_cal_BPh+38)  ;; 1 cycle
          CFI FunCall __CmpGes64
        CALL      __CmpGes64         ;; 3 cycles
        ADDW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+28
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_283  ;; 4 cycles
        ; ------------------------------------- Block: 59 cycles
// 2372             {
// 2373                 cal_BPh.l_shift.reactive.power = ~cal_BPh.l_shift.reactive.power + 1;
        MOVW      BC, N:_cal_BPh+36  ;; 1 cycle
        MOVW      AX, N:_cal_BPh+34  ;; 1 cycle
          CFI FunCall ?L_NOT_L03
        CALL      N:?L_NOT_L03       ;; 3 cycles
        ADDW      AX, #0x1           ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_cal_BPh+34, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_cal_BPh+36, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ; ------------------------------------- Block: 18 cycles
// 2374             }
// 2375             
// 2376             if(cal_BPh.r_shift.reactive.acc_cnt < 0)
??power_filter_283:
        MOVW      BC, #LWRD(__Constant_0_0)  ;; 1 cycle
        MOVW      AX, #LWRD(_cal_BPh+82)  ;; 1 cycle
          CFI FunCall __CmpGes64
        CALL      __CmpGes64         ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_284  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
// 2377             {
// 2378                 temp_us64 = ~cal_BPh.r_shift.reactive.acc_cnt + 1;
        MOVW      BC, #LWRD(_cal_BPh+82)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
          CFI FunCall __Not64
        CALL      __Not64            ;; 3 cycles
        MOVW      DE, #LWRD(__Constant_1_0)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Add64
        CALL      __Add64            ;; 3 cycles
        MOVW      HL, #LWRD(_temp_us64)  ;; 1 cycle
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
        BR        S:??power_filter_285  ;; 3 cycles
        ; ------------------------------------- Block: 28 cycles
// 2379             }
// 2380             else
// 2381             {
// 2382                 temp_us64 = cal_BPh.r_shift.reactive.acc_cnt;
??power_filter_284:
        MOVW      HL, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      DE, #LWRD(_cal_BPh+82)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        ; ------------------------------------- Block: 10 cycles
// 2383             }
// 2384             
// 2385             temp_us64 *= cal_coeff.Bph.power;
??power_filter_285:
        MOVW      AX, N:_cal_coeff+34  ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+30
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
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
// 2386             temp_us64 /= b_phase.no_of_samples;
        MOVW      AX, N:_b_phase     ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+34
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+36
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
// 2387             temp_us64 /= 100000;
        MOVW      DE, #LWRD(__Constant_186a0_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
// 2388             cal_BPh.r_shift.reactive.power = temp_us64;
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __LLU2L
        CALL      __LLU2L            ;; 3 cycles
        MOVW      N:_cal_BPh+78, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_cal_BPh+80, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 2389             if(cal_BPh.r_shift.reactive.acc_cnt < 0)
        MOVW      BC, #LWRD(__Constant_0_0)  ;; 1 cycle
        MOVW      AX, #LWRD(_cal_BPh+82)  ;; 1 cycle
          CFI FunCall __CmpGes64
        CALL      __CmpGes64         ;; 3 cycles
        ADDW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+28
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_286  ;; 4 cycles
        ; ------------------------------------- Block: 59 cycles
// 2390             {
// 2391                 cal_BPh.r_shift.reactive.power = ~cal_BPh.r_shift.reactive.power + 1;
        MOVW      BC, N:_cal_BPh+80  ;; 1 cycle
        MOVW      AX, N:_cal_BPh+78  ;; 1 cycle
          CFI FunCall ?L_NOT_L03
        CALL      N:?L_NOT_L03       ;; 3 cycles
        ADDW      AX, #0x1           ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_cal_BPh+78, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_cal_BPh+80, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ; ------------------------------------- Block: 18 cycles
// 2392             }
// 2393             
// 2394             cal_BPh.angle_reactive = cal_angle_calculate_act(cal_BPh.l_shift.reactive.power,cal_BPh.r_shift.reactive.power);
??power_filter_286:
        MOVW      AX, N:_cal_BPh+80  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+30
        MOVW      AX, N:_cal_BPh+78  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
        MOVW      BC, N:_cal_BPh+36  ;; 1 cycle
        MOVW      AX, N:_cal_BPh+34  ;; 1 cycle
          CFI FunCall _cal_angle_calculate_act
        CALL      _cal_angle_calculate_act  ;; 3 cycles
        MOVW      N:_cal_BPh+8, AX   ;; 1 cycle
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+28
        BR        S:??power_filter_224  ;; 3 cycles
        ; ------------------------------------- Block: 14 cycles
// 2395             
// 2396         }
// 2397         else
// 2398         {
// 2399             flag_calibration_b_vi_ok = 0;
??power_filter_268:
        CLR1      N:_flag_cal.2      ;; 2 cycles
// 2400             flag_calibration_b_pf_freq_pow_ok = 0;
        CLR1      N:_flag_calibration.4  ;; 2 cycles
// 2401             flag_calibration_b_ct_ok = 0;
        CLR1      N:_flag_calibration.7  ;; 2 cycles
        ; ------------------------------------- Block: 6 cycles
// 2402         }
// 2403         
// 2404     }
// 2405     if(flag_metrology_process_n == 1 && n_phase.no_of_samples != 0)
??power_filter_224:
        MOVW      HL, #LWRD(_flag_metrology)  ;; 1 cycle
        MOV1      CY, [HL].3         ;; 1 cycle
        SKC                          ;; 4 cycles
        BR        N:??power_filter_287  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        CLRW      AX                 ;; 1 cycle
        CMPW      AX, N:_n_phase     ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??power_filter_287  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 2406     {
// 2407         flag_metrology_process_n = 0;
        CLR1      N:_flag_metrology.3  ;; 2 cycles
// 2408         
// 2409         curr.Nph.rms = calculate_irms(n_phase.curr.rms_acc_cnt,cal_coeff.Nph.curr,n_phase.no_of_samples);
        MOVW      AX, N:_n_phase     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+30
        MOVW      AX, N:_cal_coeff+40  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
        MOVW      HL, #LWRD(_n_phase+18)  ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+40
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
          CFI FunCall _calculate_irms
        CALL      _calculate_irms    ;; 3 cycles
        MOVW      N:_curr+48, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_curr+50, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 2410         curr.Nph.dc = calculate_irms(n_phase.curr.offset_acc_cnt,cal_coeff.Nph.curr,n_phase.no_of_samples);
        MOVW      AX, N:_n_phase+12  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+42
        MOVW      AX, N:_n_phase+10  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+44
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
          CFI FunCall __L2LLS
        CALL      __L2LLS            ;; 3 cycles
        MOVW      AX, N:_n_phase     ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+46
        MOVW      AX, N:_cal_coeff+40  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+48
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x14          ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+56
        MOVW      DE, SP             ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
          CFI FunCall ?MOVE_LONG_L06
        CALL      N:?MOVE_LONG_L06   ;; 3 cycles
          CFI FunCall _calculate_irms
        CALL      _calculate_irms    ;; 3 cycles
        MOVW      N:_curr+56, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_curr+58, AX     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 2411         n_phase.curr.dc_offset = n_phase.curr.offset_acc_cnt / n_phase.no_of_samples;
        MOVW      AX, N:_n_phase     ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+58
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+60
        MOVW      BC, N:_n_phase+12  ;; 1 cycle
        MOVW      AX, N:_n_phase+10  ;; 1 cycle
          CFI FunCall ?SL_DIV_L03
        CALL      N:?SL_DIV_L03      ;; 3 cycles
        MOVW      N:_n_phase+8, AX   ;; 1 cycle
// 2412         
// 2413         /* Calibration */
// 2414         if(cal_done_f != 1)
        ADDW      SP, #0x20          ;; 1 cycle
          CFI CFA SP+28
        CMP       N:_cal_done_f, #0x1  ;; 1 cycle
        BZ        ??power_filter_288  ;; 4 cycles
        ; ------------------------------------- Block: 65 cycles
// 2415         {
// 2416             if(curr.Nph.rms >= CAL_CURR_MIN && curr.Nph.rms <= CAL_CURR_MAX)
        MOVW      BC, N:_curr+50     ;; 1 cycle
        MOVW      AX, N:_curr+48     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
        CMPW      AX, #0x1F40        ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??metrology_process_21:
        BC        ??power_filter_289  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
        MOVW      BC, N:_curr+50     ;; 1 cycle
        MOVW      AX, N:_curr+48     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
        CMPW      AX, #0x2EE1        ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??metrology_process_22:
        BNC       ??power_filter_289  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
// 2417             {
// 2418                 flag_calibration_n_i_ok = 1;
        SET1      N:_flag_calibration.1  ;; 2 cycles
        BR        S:??power_filter_290  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 2419             }
// 2420             else
// 2421             {
// 2422                 flag_calibration_n_i_ok = 0;
??power_filter_289:
        CLR1      N:_flag_calibration.1  ;; 2 cycles
        BR        S:??power_filter_290  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 2423             }
// 2424         }
// 2425         else
// 2426         {
// 2427             flag_calibration_n_i_ok = 0;
??power_filter_288:
        CLR1      N:_flag_calibration.1  ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
// 2428         }
// 2429         
// 2430         /* Data modification */
// 2431         if(curr.Nph.rms < MIN_CURRENT_NEU && ZERORISE_METROLOGY_PAR == 1)
??power_filter_290:
        MOVW      BC, N:_curr+50     ;; 1 cycle
        MOVW      AX, N:_curr+48     ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x0           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
        CMPW      AX, #0x64          ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??metrology_process_23:
        BNC       ??power_filter_287  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
// 2432         {
// 2433             curr.Nph.rms = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+48, AX     ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+50, AX     ;; 1 cycle
// 2434             curr.Nph.dc = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+56, AX     ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_curr+58, AX     ;; 1 cycle
        ; ------------------------------------- Block: 8 cycles
// 2435         }
// 2436     }
// 2437     
// 2438 }
??power_filter_287:
        ADDW      SP, #0x18          ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock10
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 6413 cycles

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock11 Using cfiCommon0
          CFI Function _save_energy_alternate
        CODE
// 2439 void save_energy_alternate()
// 2440 {
_save_energy_alternate:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
// 2441     fill_oprzero(112); //7 pages
        MOV       A, #0x70           ;; 1 cycle
          CFI FunCall _fill_oprzero
        CALL      _fill_oprzero      ;; 3 cycles
// 2442     
// 2443     /* Page 1 */
// 2444     long_into_char_array4(energy.Allph.active_imp,&opr_data[0]); //cum kwh for forwarded meter
        MOVW      DE, #LWRD(_opr_data)  ;; 1 cycle
        MOVW      BC, N:_energy+54   ;; 1 cycle
        MOVW      AX, N:_energy+52   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 2445     long_into_char_array4(energy.Allph.active_exp,&opr_data[4]);
        MOVW      DE, #LWRD(_opr_data+4)  ;; 1 cycle
        MOVW      BC, N:_energy+58   ;; 1 cycle
        MOVW      AX, N:_energy+56   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 2446     long_into_char_array4(energy.Allph.apparent_imp,&opr_data[8]);//cum kvah for forwarded meter
        MOVW      DE, #LWRD(_opr_data+8)  ;; 1 cycle
        MOVW      BC, N:_energy+70   ;; 1 cycle
        MOVW      AX, N:_energy+68   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 2447     long_into_char_array4(energy.Allph.apparent_exp,&opr_data[12]);
        MOVW      DE, #LWRD(_opr_data+12)  ;; 1 cycle
        MOVW      BC, N:_energy+74   ;; 1 cycle
        MOVW      AX, N:_energy+72   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 2448     
// 2449     /* Page 2 */
// 2450     long_into_char_array4(energy.Allph.zkwh_imp,&opr_data[16]);//current zone cum active energy for forwarded meter
        MOVW      DE, #LWRD(_opr_data+16)  ;; 1 cycle
        MOVW      BC, N:_energy+94   ;; 1 cycle
        MOVW      AX, N:_energy+92   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 2451     long_into_char_array4(energy.Allph.zkwh_exp,&opr_data[20]);
        MOVW      DE, #LWRD(_opr_data+20)  ;; 1 cycle
        MOVW      BC, N:_energy+98   ;; 1 cycle
        MOVW      AX, N:_energy+96   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 2452     long_into_char_array4(energy.Allph.zkvah_imp,&opr_data[24]);//current zone cum apparent energy for forwarded meter
        MOVW      DE, #LWRD(_opr_data+24)  ;; 1 cycle
        MOVW      BC, N:_energy+102  ;; 1 cycle
        MOVW      AX, N:_energy+100  ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 2453     long_into_char_array4(energy.Allph.zkvah_exp,&opr_data[28]);
        MOVW      DE, #LWRD(_opr_data+28)  ;; 1 cycle
        MOVW      BC, N:_energy+106  ;; 1 cycle
        MOVW      AX, N:_energy+104  ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 2454     
// 2455     /* Page 3 */
// 2456     long_into_char_array4(energy.Allph.reactive_q1,&opr_data[32]);//kvarh lag for forwarded meter
        MOVW      DE, #LWRD(_opr_data+32)  ;; 1 cycle
        MOVW      BC, N:_energy+78   ;; 1 cycle
        MOVW      AX, N:_energy+76   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 2457     long_into_char_array4(energy.Allph.reactive_q3,&opr_data[36]);
        MOVW      DE, #LWRD(_opr_data+36)  ;; 1 cycle
        MOVW      BC, N:_energy+86   ;; 1 cycle
        MOVW      AX, N:_energy+84   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 2458     long_into_char_array4(energy.Allph.reactive_q2,&opr_data[40]);
        MOVW      DE, #LWRD(_opr_data+40)  ;; 1 cycle
        MOVW      BC, N:_energy+82   ;; 1 cycle
        MOVW      AX, N:_energy+80   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 2459     long_into_char_array4(energy.Allph.zkvarh_q1,&opr_data[44]);//current zone cum kvarh lag energy for forwarded meter
        MOVW      DE, #LWRD(_opr_data+44)  ;; 1 cycle
        MOVW      BC, N:_energy+110  ;; 1 cycle
        MOVW      AX, N:_energy+108  ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 2460     
// 2461     /* Page 4 */
// 2462     long_into_char_array4(energy.Allph.zkvarh_q3,&opr_data[48]);
        MOVW      DE, #LWRD(_opr_data+48)  ;; 1 cycle
        MOVW      BC, N:_energy+118  ;; 1 cycle
        MOVW      AX, N:_energy+116  ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 2463     long_into_char_array4(energy.Allph.zkvarh_q2,&opr_data[52]);
        MOVW      DE, #LWRD(_opr_data+52)  ;; 1 cycle
        MOVW      BC, N:_energy+114  ;; 1 cycle
        MOVW      AX, N:_energy+112  ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 2464     long_into_char_array4(energy.Allph.defraud_mag,&opr_data[56]); //defraud energy
        MOVW      DE, #LWRD(_opr_data+56)  ;; 1 cycle
        MOVW      BC, N:_energy+62   ;; 1 cycle
        MOVW      AX, N:_energy+60   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 2465     long_into_char_array4(energy.Allph.fundamental,&opr_data[60]); //fundamental energy
        MOVW      DE, #LWRD(_opr_data+60)  ;; 1 cycle
        MOVW      BC, N:_energy+66   ;; 1 cycle
        MOVW      AX, N:_energy+64   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 2466 
// 2467     /* Page 5 */
// 2468     long_into_char_array4(energy.Rph.active_imp,&opr_data[64]);
        MOVW      DE, #LWRD(_opr_data+64)  ;; 1 cycle
        MOVW      BC, N:_energy+4    ;; 1 cycle
        MOVW      AX, N:_energy+2    ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 2469     long_into_char_array4(energy.Rph.active_exp,&opr_data[68]);
        MOVW      DE, #LWRD(_opr_data+68)  ;; 1 cycle
        MOVW      BC, N:_energy+8    ;; 1 cycle
        MOVW      AX, N:_energy+6    ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 2470     long_into_char_array4(energy.Yph.active_imp,&opr_data[72]);
        MOVW      DE, #LWRD(_opr_data+72)  ;; 1 cycle
        MOVW      BC, N:_energy+18   ;; 1 cycle
        MOVW      AX, N:_energy+16   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 2471     long_into_char_array4(energy.Yph.active_exp,&opr_data[76]);
        MOVW      DE, #LWRD(_opr_data+76)  ;; 1 cycle
        MOVW      BC, N:_energy+22   ;; 1 cycle
        MOVW      AX, N:_energy+20   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 2472     
// 2473     /* Page 6 */
// 2474     long_into_char_array4(energy.Bph.active_imp,&opr_data[80]);
        MOVW      DE, #LWRD(_opr_data+80)  ;; 1 cycle
        MOVW      BC, N:_energy+32   ;; 1 cycle
        MOVW      AX, N:_energy+30   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 2475     long_into_char_array4(energy.Bph.active_exp,&opr_data[84]);
        MOVW      DE, #LWRD(_opr_data+84)  ;; 1 cycle
        MOVW      BC, N:_energy+36   ;; 1 cycle
        MOVW      AX, N:_energy+34   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 2476     long_into_char_array4(energy.Rph.defraud_mag,&opr_data[88]); //defraud energy
        MOVW      DE, #LWRD(_opr_data+88)  ;; 1 cycle
        MOVW      BC, N:_energy+12   ;; 1 cycle
        MOVW      AX, N:_energy+10   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 2477     long_into_char_array4(energy.Yph.defraud_mag,&opr_data[92]); //defraud energy
        MOVW      DE, #LWRD(_opr_data+92)  ;; 1 cycle
        MOVW      BC, N:_energy+26   ;; 1 cycle
        MOVW      AX, N:_energy+24   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 2478     
// 2479     /* Page 7 */
// 2480     long_into_char_array4(energy.Bph.defraud_mag,&opr_data[96]); //defraud energy
        MOVW      DE, #LWRD(_opr_data+96)  ;; 1 cycle
        MOVW      BC, N:_energy+40   ;; 1 cycle
        MOVW      AX, N:_energy+38   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 2481     int_into_char_array(zone_pf,&opr_data[100]);
        MOVW      BC, #LWRD(_opr_data+100)  ;; 1 cycle
        MOVW      AX, N:_zone_pf     ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
// 2482     
// 2483     eprom_write(ALTERNATE_ALL_ENERGY_SAVE_START_ADD,2,ALL_ENERGY_SAVE_BLOCK_SIZE,PAGE_7,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+6
        MOV       B, #0x6            ;; 1 cycle
        MOVW      DE, #0x70          ;; 1 cycle
        MOV       C, #0x2            ;; 1 cycle
        MOVW      AX, #0xFF00        ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
// 2484     
// 2485     fill_oprzero(16);
        MOV       A, #0x10           ;; 1 cycle
          CFI FunCall _fill_oprzero
        CALL      _fill_oprzero      ;; 3 cycles
// 2486     
// 2487     long_into_char_array4(energy.Allph.reactive_q4,&opr_data[0]);//kvarh lead for forwarded meter
        MOVW      DE, #LWRD(_opr_data)  ;; 1 cycle
        MOVW      BC, N:_energy+90   ;; 1 cycle
        MOVW      AX, N:_energy+88   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 2488     long_into_char_array4(energy.Allph.zkvarh_q4,&opr_data[4]);//current zone cum kvarh lead energy for forwarded meter
        MOVW      DE, #LWRD(_opr_data+4)  ;; 1 cycle
        MOVW      BC, N:_energy+122  ;; 1 cycle
        MOVW      AX, N:_energy+120  ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 2489     
// 2490     eprom_write(ALTERNATE_LEAD_ENERGY_SAVE_START_ADD,2,LEAD_ENERGY_SAVE_BLOCK_SIZE,PAGE_1,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+8
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x2            ;; 1 cycle
        MOVW      AX, #0xFF70        ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
// 2491 }
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock11
        ; ------------------------------------- Block: 200 cycles
        ; ------------------------------------- Total: 200 cycles
// 2492 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock12 Using cfiCommon0
          CFI Function _metrology_save_energy
        CODE
// 2493 void metrology_save_energy()
// 2494 {
_metrology_save_energy:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 6
        SUBW      SP, #0x6           ;; 1 cycle
          CFI CFA SP+10
// 2495     us32 mem_address;
// 2496     us16  lu8_cir_buf_loc;
// 2497     fill_oprzero(112); //7 pages
        MOV       A, #0x70           ;; 1 cycle
          CFI FunCall _fill_oprzero
        CALL      _fill_oprzero      ;; 3 cycles
// 2498     
// 2499     
// 2500     if(energy.Allph.apparent_imp != duplicate_total_apparent_energy)
        MOVW      AX, N:_duplicate_total_apparent_energy+2  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      AX, N:_duplicate_total_apparent_energy  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+14
        MOVW      BC, N:_energy+70   ;; 1 cycle
        MOVW      AX, N:_energy+68   ;; 1 cycle
          CFI FunCall ?UL_CMP_L03
        CALL      N:?UL_CMP_L03      ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+10
        SKNZ                         ;; 4 cycles
        BR        N:??power_filter_291  ;; 4 cycles
        ; ------------------------------------- Block: 19 cycles
// 2501     {
// 2502         if(0x00 == e2416_byte) //if it is a first page of circular buffer,then previous page will be last location
        CLRW      AX                 ;; 1 cycle
        CMPW      AX, N:_e2416_byte  ;; 1 cycle
        BNZ       ??power_filter_292  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 2503         {
// 2504           lu8_cir_buf_loc=0x0f*ALL_ENERGY_SAVE_BLOCK_SIZE; 
        MOVW      AX, #0x690         ;; 1 cycle
        MOVW      [SP], AX           ;; 1 cycle
        BR        S:??power_filter_293  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 2505         }
// 2506         else
// 2507         {
// 2508           lu8_cir_buf_loc=e2416_byte-ALL_ENERGY_SAVE_BLOCK_SIZE;
??power_filter_292:
        MOVW      AX, N:_e2416_byte  ;; 1 cycle
        ADDW      AX, #0xFF90        ;; 1 cycle
        MOVW      [SP], AX           ;; 1 cycle
        ; ------------------------------------- Block: 3 cycles
// 2509         }
// 2510         
// 2511         eprom_read((ALL_ENERGY_SAVE_START_ADD+lu8_cir_buf_loc),0,PAGE_7,AUTO_CALC);
??power_filter_293:
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x6            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        ADDW      AX, #0x6500        ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
// 2512 
// 2513         energy.Allph.active_imp = char_array_to_long4(&opr_data[0]);
        MOVW      AX, #LWRD(_opr_data)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_energy+52, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+54, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 2514         energy.Allph.active_exp = char_array_to_long4(&opr_data[4]);
        MOVW      AX, #LWRD(_opr_data+4)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_energy+56, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+58, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 2515         energy.Allph.apparent_imp = char_array_to_long4(&opr_data[8]);
        MOVW      AX, #LWRD(_opr_data+8)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_energy+68, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+70, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 2516         energy.Allph.apparent_exp = char_array_to_long4(&opr_data[12]);
        MOVW      AX, #LWRD(_opr_data+12)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_energy+72, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+74, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 2517 
// 2518         energy.Allph.zkwh_imp = char_array_to_long4(&opr_data[16]);
        MOVW      AX, #LWRD(_opr_data+16)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_energy+92, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+94, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 2519         energy.Allph.zkwh_exp = char_array_to_long4(&opr_data[20]);
        MOVW      AX, #LWRD(_opr_data+20)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_energy+96, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+98, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 2520 
// 2521         energy.Allph.zkvah_imp = char_array_to_long4(&opr_data[24]);
        MOVW      AX, #LWRD(_opr_data+24)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_energy+100, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+102, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 2522         energy.Allph.zkvah_exp = char_array_to_long4(&opr_data[28]);
        MOVW      AX, #LWRD(_opr_data+28)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_energy+104, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+106, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 2523 
// 2524         energy.Allph.reactive_q1 = char_array_to_long4(&opr_data[32]);
        MOVW      AX, #LWRD(_opr_data+32)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_energy+76, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+78, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 2525         energy.Allph.reactive_q3 = char_array_to_long4(&opr_data[36]);
        MOVW      AX, #LWRD(_opr_data+36)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_energy+84, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+86, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 2526         energy.Allph.reactive_q2 = char_array_to_long4(&opr_data[40]);
        MOVW      AX, #LWRD(_opr_data+40)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_energy+80, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+82, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 2527 
// 2528         energy.Allph.zkvarh_q1 = char_array_to_long4(&opr_data[44]);
        MOVW      AX, #LWRD(_opr_data+44)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_energy+108, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+110, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 2529         energy.Allph.zkvarh_q3 = char_array_to_long4(&opr_data[48]);
        MOVW      AX, #LWRD(_opr_data+48)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_energy+116, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+118, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 2530         energy.Allph.zkvarh_q2 = char_array_to_long4(&opr_data[52]);
        MOVW      AX, #LWRD(_opr_data+52)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_energy+112, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+114, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 2531 
// 2532         energy.Allph.defraud_mag = char_array_to_long4(&opr_data[56]);
        MOVW      AX, #LWRD(_opr_data+56)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_energy+60, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+62, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 2533         energy.Allph.fundamental = char_array_to_long4(&opr_data[60]);
        MOVW      AX, #LWRD(_opr_data+60)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_energy+64, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+66, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 2534 
// 2535         energy.Rph.active_imp = char_array_to_long4(&opr_data[64]);
        MOVW      AX, #LWRD(_opr_data+64)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_energy+2, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+4, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 2536         energy.Rph.active_exp = char_array_to_long4(&opr_data[68]);
        MOVW      AX, #LWRD(_opr_data+68)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_energy+6, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+8, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 2537         energy.Yph.active_imp = char_array_to_long4(&opr_data[72]);
        MOVW      AX, #LWRD(_opr_data+72)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_energy+16, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+18, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 2538         energy.Yph.active_exp = char_array_to_long4(&opr_data[76]);
        MOVW      AX, #LWRD(_opr_data+76)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_energy+20, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+22, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 2539         energy.Bph.active_imp = char_array_to_long4(&opr_data[80]);
        MOVW      AX, #LWRD(_opr_data+80)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_energy+30, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+32, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 2540         energy.Bph.active_exp = char_array_to_long4(&opr_data[84]);
        MOVW      AX, #LWRD(_opr_data+84)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_energy+34, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+36, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 2541 
// 2542         energy.Rph.defraud_mag = char_array_to_long4(&opr_data[88]);
        MOVW      AX, #LWRD(_opr_data+88)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_energy+10, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+12, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 2543         energy.Yph.defraud_mag = char_array_to_long4(&opr_data[92]);
        MOVW      AX, #LWRD(_opr_data+92)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_energy+24, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+26, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 2544         energy.Bph.defraud_mag = char_array_to_long4(&opr_data[96]);
        MOVW      AX, #LWRD(_opr_data+96)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_energy+38, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+40, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 2545         zone_pf = char_array_to_int(&opr_data[100]);
        MOVW      AX, #LWRD(_opr_data+100)  ;; 1 cycle
          CFI FunCall _char_array_to_int
        CALL      _char_array_to_int  ;; 3 cycles
        MOVW      N:_zone_pf, AX     ;; 1 cycle
// 2546         
// 2547         duplicate_total_apparent_energy=energy.Allph.apparent_imp;
        MOVW      BC, N:_energy+70   ;; 1 cycle
        MOVW      AX, N:_energy+68   ;; 1 cycle
        MOVW      N:_duplicate_total_apparent_energy, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_duplicate_total_apparent_energy+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ; ------------------------------------- Block: 219 cycles
// 2548 
// 2549     }
// 2550     
// 2551     /* Page 1 */
// 2552     long_into_char_array4(energy.Allph.active_imp,&opr_data[0]); //cum kwh for forwarded meter
??power_filter_291:
        MOVW      DE, #LWRD(_opr_data)  ;; 1 cycle
        MOVW      BC, N:_energy+54   ;; 1 cycle
        MOVW      AX, N:_energy+52   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 2553     long_into_char_array4(energy.Allph.active_exp,&opr_data[4]);
        MOVW      DE, #LWRD(_opr_data+4)  ;; 1 cycle
        MOVW      BC, N:_energy+58   ;; 1 cycle
        MOVW      AX, N:_energy+56   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 2554     long_into_char_array4(energy.Allph.apparent_imp,&opr_data[8]);//cum kvah for forwarded meter
        MOVW      DE, #LWRD(_opr_data+8)  ;; 1 cycle
        MOVW      BC, N:_energy+70   ;; 1 cycle
        MOVW      AX, N:_energy+68   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 2555     long_into_char_array4(energy.Allph.apparent_exp,&opr_data[12]);
        MOVW      DE, #LWRD(_opr_data+12)  ;; 1 cycle
        MOVW      BC, N:_energy+74   ;; 1 cycle
        MOVW      AX, N:_energy+72   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 2556     
// 2557     /* Page 2 */
// 2558     long_into_char_array4(energy.Allph.zkwh_imp,&opr_data[16]);//current zone cum active energy for forwarded meter
        MOVW      DE, #LWRD(_opr_data+16)  ;; 1 cycle
        MOVW      BC, N:_energy+94   ;; 1 cycle
        MOVW      AX, N:_energy+92   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 2559     long_into_char_array4(energy.Allph.zkwh_exp,&opr_data[20]);
        MOVW      DE, #LWRD(_opr_data+20)  ;; 1 cycle
        MOVW      BC, N:_energy+98   ;; 1 cycle
        MOVW      AX, N:_energy+96   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 2560     long_into_char_array4(energy.Allph.zkvah_imp,&opr_data[24]);//current zone cum apparent energy for forwarded meter
        MOVW      DE, #LWRD(_opr_data+24)  ;; 1 cycle
        MOVW      BC, N:_energy+102  ;; 1 cycle
        MOVW      AX, N:_energy+100  ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 2561     long_into_char_array4(energy.Allph.zkvah_exp,&opr_data[28]);
        MOVW      DE, #LWRD(_opr_data+28)  ;; 1 cycle
        MOVW      BC, N:_energy+106  ;; 1 cycle
        MOVW      AX, N:_energy+104  ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 2562     
// 2563     /* Page 3 */
// 2564     long_into_char_array4(energy.Allph.reactive_q1,&opr_data[32]);//kvarh lag for forwarded meter
        MOVW      DE, #LWRD(_opr_data+32)  ;; 1 cycle
        MOVW      BC, N:_energy+78   ;; 1 cycle
        MOVW      AX, N:_energy+76   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 2565     long_into_char_array4(energy.Allph.reactive_q3,&opr_data[36]);
        MOVW      DE, #LWRD(_opr_data+36)  ;; 1 cycle
        MOVW      BC, N:_energy+86   ;; 1 cycle
        MOVW      AX, N:_energy+84   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 2566     long_into_char_array4(energy.Allph.reactive_q2,&opr_data[40]);
        MOVW      DE, #LWRD(_opr_data+40)  ;; 1 cycle
        MOVW      BC, N:_energy+82   ;; 1 cycle
        MOVW      AX, N:_energy+80   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 2567     long_into_char_array4(energy.Allph.zkvarh_q1,&opr_data[44]);//current zone cum kvarh lag energy for forwarded meter
        MOVW      DE, #LWRD(_opr_data+44)  ;; 1 cycle
        MOVW      BC, N:_energy+110  ;; 1 cycle
        MOVW      AX, N:_energy+108  ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 2568     
// 2569     /* Page 4 */
// 2570     long_into_char_array4(energy.Allph.zkvarh_q3,&opr_data[48]);
        MOVW      DE, #LWRD(_opr_data+48)  ;; 1 cycle
        MOVW      BC, N:_energy+118  ;; 1 cycle
        MOVW      AX, N:_energy+116  ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 2571     long_into_char_array4(energy.Allph.zkvarh_q2,&opr_data[52]);
        MOVW      DE, #LWRD(_opr_data+52)  ;; 1 cycle
        MOVW      BC, N:_energy+114  ;; 1 cycle
        MOVW      AX, N:_energy+112  ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 2572     long_into_char_array4(energy.Allph.defraud_mag,&opr_data[56]); //defraud energy
        MOVW      DE, #LWRD(_opr_data+56)  ;; 1 cycle
        MOVW      BC, N:_energy+62   ;; 1 cycle
        MOVW      AX, N:_energy+60   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 2573     long_into_char_array4(energy.Allph.fundamental,&opr_data[60]); //fundamental energy
        MOVW      DE, #LWRD(_opr_data+60)  ;; 1 cycle
        MOVW      BC, N:_energy+66   ;; 1 cycle
        MOVW      AX, N:_energy+64   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 2574 
// 2575     /* Page 5 */
// 2576     long_into_char_array4(energy.Rph.active_imp,&opr_data[64]);
        MOVW      DE, #LWRD(_opr_data+64)  ;; 1 cycle
        MOVW      BC, N:_energy+4    ;; 1 cycle
        MOVW      AX, N:_energy+2    ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 2577     long_into_char_array4(energy.Rph.active_exp,&opr_data[68]);
        MOVW      DE, #LWRD(_opr_data+68)  ;; 1 cycle
        MOVW      BC, N:_energy+8    ;; 1 cycle
        MOVW      AX, N:_energy+6    ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 2578     long_into_char_array4(energy.Yph.active_imp,&opr_data[72]);
        MOVW      DE, #LWRD(_opr_data+72)  ;; 1 cycle
        MOVW      BC, N:_energy+18   ;; 1 cycle
        MOVW      AX, N:_energy+16   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 2579     long_into_char_array4(energy.Yph.active_exp,&opr_data[76]);
        MOVW      DE, #LWRD(_opr_data+76)  ;; 1 cycle
        MOVW      BC, N:_energy+22   ;; 1 cycle
        MOVW      AX, N:_energy+20   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 2580     
// 2581     /* Page 6 */
// 2582     long_into_char_array4(energy.Bph.active_imp,&opr_data[80]);
        MOVW      DE, #LWRD(_opr_data+80)  ;; 1 cycle
        MOVW      BC, N:_energy+32   ;; 1 cycle
        MOVW      AX, N:_energy+30   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 2583     long_into_char_array4(energy.Bph.active_exp,&opr_data[84]);
        MOVW      DE, #LWRD(_opr_data+84)  ;; 1 cycle
        MOVW      BC, N:_energy+36   ;; 1 cycle
        MOVW      AX, N:_energy+34   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 2584     long_into_char_array4(energy.Rph.defraud_mag,&opr_data[88]); //defraud energy
        MOVW      DE, #LWRD(_opr_data+88)  ;; 1 cycle
        MOVW      BC, N:_energy+12   ;; 1 cycle
        MOVW      AX, N:_energy+10   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 2585     long_into_char_array4(energy.Yph.defraud_mag,&opr_data[92]); //defraud energy
        MOVW      DE, #LWRD(_opr_data+92)  ;; 1 cycle
        MOVW      BC, N:_energy+26   ;; 1 cycle
        MOVW      AX, N:_energy+24   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 2586     
// 2587     /* Page 7 */
// 2588     long_into_char_array4(energy.Bph.defraud_mag,&opr_data[96]); //defraud energy
        MOVW      DE, #LWRD(_opr_data+96)  ;; 1 cycle
        MOVW      BC, N:_energy+40   ;; 1 cycle
        MOVW      AX, N:_energy+38   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 2589     int_into_char_array(zone_pf,&opr_data[100]);
        MOVW      BC, #LWRD(_opr_data+100)  ;; 1 cycle
        MOVW      AX, N:_zone_pf     ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
// 2590     
// 2591     mem_address=ALL_ENERGY_SAVE_START_ADD+e2416_byte; 
        MOVW      AX, N:_e2416_byte  ;; 1 cycle
        ADDW      AX, #0x6500        ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x2           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 2592     eprom_write(mem_address,0,ALL_ENERGY_SAVE_BLOCK_SIZE,PAGE_7,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOV       B, #0x6            ;; 1 cycle
        MOVW      DE, #0x70          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
// 2593     
// 2594     fill_oprzero(16);
        MOV       A, #0x10           ;; 1 cycle
          CFI FunCall _fill_oprzero
        CALL      _fill_oprzero      ;; 3 cycles
// 2595     long_into_char_array4(energy.Allph.reactive_q4,&opr_data[0]);//kvarh lead for forwarded meter
        MOVW      DE, #LWRD(_opr_data)  ;; 1 cycle
        MOVW      BC, N:_energy+90   ;; 1 cycle
        MOVW      AX, N:_energy+88   ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 2596     long_into_char_array4(energy.Allph.zkvarh_q4,&opr_data[4]);//current zone cum kvarh lead energy for forwarded meter
        MOVW      DE, #LWRD(_opr_data+4)  ;; 1 cycle
        MOVW      BC, N:_energy+122  ;; 1 cycle
        MOVW      AX, N:_energy+120  ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 2597     mem_address=LEAD_ENERGY_SAVE_START_ADD+e2416_q2_byte;
        MOV       X, N:_e2416_q2_byte  ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        ADDW      AX, #0x6C00        ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+10
        XCHW      AX, HL             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x2           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 2598     eprom_write(mem_address,0,LEAD_ENERGY_SAVE_BLOCK_SIZE,PAGE_1,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
// 2599     
// 2600     e2416_byte+=ALL_ENERGY_SAVE_BLOCK_SIZE;
        MOVW      AX, N:_e2416_byte  ;; 1 cycle
        ADDW      AX, #0x70          ;; 1 cycle
        MOVW      N:_e2416_byte, AX  ;; 1 cycle
// 2601     e2416_q2_byte+=LEAD_ENERGY_SAVE_BLOCK_SIZE;
        MOVW      HL, #LWRD(_e2416_q2_byte)  ;; 1 cycle
        MOV       A, [HL]            ;; 1 cycle
        ADD       A, #0x10           ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
// 2602     
// 2603     if(e2416_byte==(ALL_ENERGY_SAVE_BLOCK_SIZE*0x10))
        MOVW      AX, N:_e2416_byte  ;; 1 cycle
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+10
        CMPW      AX, #0x700         ;; 1 cycle
        BNZ       ??power_filter_294  ;; 4 cycles
        ; ------------------------------------- Block: 229 cycles
// 2604     {
// 2605       e2416_byte=0x00;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_e2416_byte, AX  ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
// 2606     }
// 2607 }
??power_filter_294:
        ADDW      SP, #0x6           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock12
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 490 cycles

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock13 Using cfiCommon0
          CFI Function _metrology_save_pulse
        CODE
// 2608 void metrology_save_pulse()
// 2609 {
_metrology_save_pulse:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
// 2610     opr_data[0] = energy.Allph.active_imp_pulse;
        MOV       A, N:_energy+42    ;; 1 cycle
        MOV       N:_opr_data, A     ;; 1 cycle
// 2611     opr_data[1] = energy.Allph.active_exp_pulse;
        MOV       A, N:_energy+43    ;; 1 cycle
        MOV       N:_opr_data+1, A   ;; 1 cycle
// 2612     opr_data[2] = energy.Allph.reactive_q1_pulse;
        MOV       A, N:_energy+47    ;; 1 cycle
        MOV       N:_opr_data+2, A   ;; 1 cycle
// 2613     opr_data[3] = energy.Allph.reactive_q2_pulse;
        MOV       A, N:_energy+48    ;; 1 cycle
        MOV       N:_opr_data+3, A   ;; 1 cycle
// 2614     opr_data[4] = energy.Allph.reactive_q3_pulse;
        MOV       A, N:_energy+49    ;; 1 cycle
        MOV       N:_opr_data+4, A   ;; 1 cycle
// 2615     opr_data[5] = energy.Allph.reactive_q4_pulse;
        MOV       A, N:_energy+50    ;; 1 cycle
        MOV       N:_opr_data+5, A   ;; 1 cycle
// 2616     opr_data[6] = energy.Allph.apparent_imp_pulse;
        MOV       A, N:_energy+45    ;; 1 cycle
        MOV       N:_opr_data+6, A   ;; 1 cycle
// 2617     opr_data[7] = energy.Allph.apparent_exp_pulse;
        MOV       A, N:_energy+46    ;; 1 cycle
        MOV       N:_opr_data+7, A   ;; 1 cycle
// 2618     opr_data[8] = energy.Rph.active_imp_pulse;
        MOV       A, N:_energy       ;; 1 cycle
        MOV       N:_opr_data+8, A   ;; 1 cycle
// 2619     opr_data[9] = energy.Rph.active_exp_pulse;
        MOV       A, N:_energy+1     ;; 1 cycle
        MOV       N:_opr_data+9, A   ;; 1 cycle
// 2620     opr_data[10] = energy.Yph.active_imp_pulse;
        MOV       A, N:_energy+14    ;; 1 cycle
        MOV       N:_opr_data+10, A  ;; 1 cycle
// 2621     opr_data[11] = energy.Yph.active_exp_pulse;
        MOV       A, N:_energy+15    ;; 1 cycle
        MOV       N:_opr_data+11, A  ;; 1 cycle
// 2622     opr_data[12] = energy.Bph.active_imp_pulse;
        MOV       A, N:_energy+28    ;; 1 cycle
        MOV       N:_opr_data+12, A  ;; 1 cycle
// 2623     opr_data[13] = energy.Bph.active_exp_pulse;
        MOV       A, N:_energy+29    ;; 1 cycle
        MOV       N:_opr_data+13, A  ;; 1 cycle
// 2624     opr_data[14] = alt_energy_save_cntr;
        MOV       A, N:_alt_energy_save_cntr  ;; 1 cycle
        MOV       N:_opr_data+14, A  ;; 1 cycle
// 2625     eprom_write(0x0CF0,0,16,PAGE_1,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+6
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xCF0         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
// 2626     opr_data[0] = energy.Allph.fundamental_pulse;
        MOV       A, N:_energy+44    ;; 1 cycle
        MOV       N:_opr_data, A     ;; 1 cycle
// 2627     eprom_write(0x0CD0,0,16,PAGE_1,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+8
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xCD0         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
// 2628 }
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock13
        ; ------------------------------------- Block: 57 cycles
        ; ------------------------------------- Total: 57 cycles

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock14 Using cfiCommon0
          CFI Function _save_all_energy_and_pulse
          CFI FunCall _metrology_save_energy
        CODE
// 2629 void save_all_energy_and_pulse()
// 2630 {
_save_all_energy_and_pulse:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
// 2631     metrology_save_energy();
        CALL      _metrology_save_energy  ;; 3 cycles
// 2632     metrology_save_pulse();
          CFI FunCall _metrology_save_pulse
        CALL      _metrology_save_pulse  ;; 3 cycles
// 2633 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock14
        ; ------------------------------------- Block: 12 cycles
        ; ------------------------------------- Total: 12 cycles

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock15 Using cfiCommon0
          CFI Function _delay_voltage_samples
          CFI NoCalls
        CODE
// 2634 inline void delay_voltage_samples()
// 2635 { 
_delay_voltage_samples:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 2
        SUBW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+6
// 2636         /* Global variables */
// 2637     static s16 buffer_volr[BUF_SIZE];
// 2638     static s16 buffer_voly[BUF_SIZE];
// 2639     static s16 buffer_volb[BUF_SIZE];
// 2640     static s16 buffer_volr90[BUF_SIZE90];
// 2641     static s16 buffer_voly90[BUF_SIZE90];
// 2642     static s16 buffer_volb90[BUF_SIZE90];
// 2643     static us8 buffer_ptr = 0,buffer_ptr90 = 0;
// 2644     
// 2645     /* Local variables */
// 2646     us8 sample_ptr,sample_ptr90;
// 2647     
// 2648     sample_ptr = (buffer_ptr-DELAY_VOL+BUF_SIZE)%BUF_SIZE; 
        MOVW      DE, #0xA           ;; 1 cycle
        MOV       X, N:__ZZ21delay_voltage_samplesE10buffer_ptr  ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        ADDW      AX, #0x3           ;; 1 cycle
        DIVHU                        ;; 9 cycles
        NOP                          ;; 1 cycle
        MOV       A, E               ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
// 2649     sample_ptr90 = (buffer_ptr90-react_sample_delay-1-DELAY_VOL+BUF_SIZE90)%BUF_SIZE90;
        MOVW      DE, #0x23          ;; 1 cycle
        MOV       C, N:_react_sample_delay  ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       X, N:__ZZ21delay_voltage_samplesE12buffer_ptr90  ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        SUBW      AX, BC             ;; 1 cycle
        ADDW      AX, #0x1B          ;; 1 cycle
        DIVHU                        ;; 9 cycles
        NOP                          ;; 1 cycle
// 2650     
// 2651     /* R Phase */
// 2652     buffer_volr[buffer_ptr] = r_phase.vol.sample_raw;
        MOVW      AX, N:_r_phase+6   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+8
        MOV       X, N:__ZZ21delay_voltage_samplesE10buffer_ptr  ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(__ZZ21delay_voltage_samplesE11buffer_volr)  ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        POP       AX                 ;; 1 cycle
          CFI CFA SP+6
        MOVW      [HL], AX           ;; 1 cycle
// 2653     buffer_volr90[buffer_ptr90] = r_phase.vol.sample_raw90;
        MOVW      AX, N:_r_phase+10  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+8
        MOV       X, N:__ZZ21delay_voltage_samplesE12buffer_ptr90  ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(__ZZ21delay_voltage_samplesE13buffer_volr90)  ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        POP       AX                 ;; 1 cycle
          CFI CFA SP+6
        MOVW      [HL], AX           ;; 1 cycle
// 2654     r_phase.vol.sample_raw  = buffer_volr[sample_ptr];
        MOV       A, [SP]            ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(__ZZ21delay_voltage_samplesE11buffer_volr)  ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [HL]           ;; 1 cycle
        MOVW      N:_r_phase+6, AX   ;; 1 cycle
// 2655     r_phase.vol.sample_raw90 = buffer_volr90[sample_ptr90];
        MOVW      AX, DE             ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(__ZZ21delay_voltage_samplesE13buffer_volr90)  ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [HL]           ;; 1 cycle
        MOVW      N:_r_phase+10, AX  ;; 1 cycle
// 2656     
// 2657     /* Y Phase */
// 2658     buffer_voly[buffer_ptr] = y_phase.vol.sample_raw;
        MOVW      AX, N:_y_phase+6   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+8
        MOV       X, N:__ZZ21delay_voltage_samplesE10buffer_ptr  ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(__ZZ21delay_voltage_samplesE11buffer_voly)  ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        POP       AX                 ;; 1 cycle
          CFI CFA SP+6
        MOVW      [HL], AX           ;; 1 cycle
// 2659     buffer_voly90[buffer_ptr90] = y_phase.vol.sample_raw90;
        MOVW      AX, N:_y_phase+10  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+8
        MOV       X, N:__ZZ21delay_voltage_samplesE12buffer_ptr90  ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(__ZZ21delay_voltage_samplesE13buffer_voly90)  ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        POP       AX                 ;; 1 cycle
          CFI CFA SP+6
        MOVW      [HL], AX           ;; 1 cycle
// 2660     y_phase.vol.sample_raw  = buffer_voly[sample_ptr];
        MOV       A, [SP]            ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(__ZZ21delay_voltage_samplesE11buffer_voly)  ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [HL]           ;; 1 cycle
        MOVW      N:_y_phase+6, AX   ;; 1 cycle
// 2661     y_phase.vol.sample_raw90 = buffer_voly90[sample_ptr90];
        MOVW      AX, DE             ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(__ZZ21delay_voltage_samplesE13buffer_voly90)  ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [HL]           ;; 1 cycle
        MOVW      N:_y_phase+10, AX  ;; 1 cycle
// 2662     
// 2663     /* B Phase */
// 2664     buffer_volb[buffer_ptr] = b_phase.vol.sample_raw;
        MOVW      AX, N:_b_phase+6   ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+8
        MOV       X, N:__ZZ21delay_voltage_samplesE10buffer_ptr  ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(__ZZ21delay_voltage_samplesE11buffer_volb)  ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        POP       AX                 ;; 1 cycle
          CFI CFA SP+6
        MOVW      [HL], AX           ;; 1 cycle
// 2665     buffer_volb90[buffer_ptr90] = b_phase.vol.sample_raw90;
        MOVW      AX, N:_b_phase+10  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+8
        MOV       X, N:__ZZ21delay_voltage_samplesE12buffer_ptr90  ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(__ZZ21delay_voltage_samplesE13buffer_volb90)  ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        POP       AX                 ;; 1 cycle
          CFI CFA SP+6
        MOVW      [HL], AX           ;; 1 cycle
// 2666     b_phase.vol.sample_raw  = buffer_volb[sample_ptr];
        MOV       A, [SP]            ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(__ZZ21delay_voltage_samplesE11buffer_volb)  ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [HL]           ;; 1 cycle
        MOVW      N:_b_phase+6, AX   ;; 1 cycle
// 2667     b_phase.vol.sample_raw90 = buffer_volb90[sample_ptr90];
        MOVW      AX, DE             ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(__ZZ21delay_voltage_samplesE13buffer_volb90)  ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [HL]           ;; 1 cycle
        MOVW      N:_b_phase+10, AX  ;; 1 cycle
// 2668     
// 2669     buffer_ptr++;
        INC       N:__ZZ21delay_voltage_samplesE10buffer_ptr  ;; 2 cycles
// 2670     if(buffer_ptr >= BUF_SIZE)          /* 10 */
        MOV       A, N:__ZZ21delay_voltage_samplesE10buffer_ptr  ;; 1 cycle
        CMP       A, #0xA            ;; 1 cycle
        SKC                          ;; 1 cycle
        ; ------------------------------------- Block: 162 cycles
// 2671     {
// 2672         buffer_ptr = 0;
        MOV       N:__ZZ21delay_voltage_samplesE10buffer_ptr, #0x0  ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
// 2673     }
// 2674     buffer_ptr90++;
??delay_voltage_samples_0:
        INC       N:__ZZ21delay_voltage_samplesE12buffer_ptr90  ;; 2 cycles
// 2675     if(buffer_ptr90 >= BUF_SIZE90)      /* 33 */
        MOV       A, N:__ZZ21delay_voltage_samplesE12buffer_ptr90  ;; 1 cycle
        CMP       A, #0x23           ;; 1 cycle
        SKC                          ;; 1 cycle
        ; ------------------------------------- Block: 5 cycles
// 2676     {
// 2677         buffer_ptr90 = 0;
        MOV       N:__ZZ21delay_voltage_samplesE12buffer_ptr90, #0x0  ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
// 2678     }
// 2679 }
??delay_voltage_samples_1:
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock15
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 176 cycles

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
        SECTION_GROUP __ZZ21delay_voltage_samplesE11buffer_volr
__ZZ21delay_voltage_samplesE11buffer_volr:
        DS 20

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
        SECTION_GROUP __ZZ21delay_voltage_samplesE11buffer_voly
__ZZ21delay_voltage_samplesE11buffer_voly:
        DS 20

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
        SECTION_GROUP __ZZ21delay_voltage_samplesE11buffer_volb
__ZZ21delay_voltage_samplesE11buffer_volb:
        DS 20

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
        SECTION_GROUP __ZZ21delay_voltage_samplesE13buffer_volr90
__ZZ21delay_voltage_samplesE13buffer_volr90:
        DS 70

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
        SECTION_GROUP __ZZ21delay_voltage_samplesE13buffer_voly90
__ZZ21delay_voltage_samplesE13buffer_voly90:
        DS 70

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
        SECTION_GROUP __ZZ21delay_voltage_samplesE13buffer_volb90
__ZZ21delay_voltage_samplesE13buffer_volb90:
        DS 70

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
        SECTION_GROUP __ZZ21delay_voltage_samplesE10buffer_ptr
__ZZ21delay_voltage_samplesE10buffer_ptr:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
        SECTION_GROUP __ZZ21delay_voltage_samplesE12buffer_ptr90
__ZZ21delay_voltage_samplesE12buffer_ptr90:
        DS 1

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock16 Using cfiCommon1
          CFI Function _cal_angle
        CODE
// 2680 s16 cal_angle(us32 active_power,us32 reactive_power,us32 apparent_power,us8 active_f, us8 reactive_f, us8 trig)
// 2681 {
_cal_angle:
        ; * Stack frame (at entry) *
        ; Param size: 10
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+6
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+8
        PUSH      DE                 ;; 1 cycle
          CFI CFA SP+10
        ; Auto size: 8
        SUBW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+12
// 2682     s16 angle=0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [SP], AX           ;; 1 cycle
// 2683     if(trig == 0 && reactive_power != 0 && apparent_power != 0)
        MOV       A, [SP+0x14]       ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_295  ;; 4 cycles
        ; ------------------------------------- Block: 12 cycles
        MOVW      AX, [SP+0x0E]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x0C]      ;; 1 cycle
        OR        A, X               ;; 1 cycle
        OR        A, C               ;; 1 cycle
        OR        A, B               ;; 1 cycle
        BZ        ??power_filter_295  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
        MOVW      AX, [SP+0x12]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x10]      ;; 1 cycle
        OR        A, X               ;; 1 cycle
        OR        A, C               ;; 1 cycle
        OR        A, B               ;; 1 cycle
        BZ        ??power_filter_295  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
// 2684     {
// 2685         angle = (s16)DEGREE(asin((double)reactive_power/apparent_power)*100);
        MOVW      AX, #0x4265        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+14
        MOVW      AX, #0x2EE1        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+16
        MOVW      AX, #0x42C8        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+18
        MOVW      AX, #0x0           ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+20
        MOVW      AX, [SP+0x1A]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x18]      ;; 1 cycle
          CFI FunCall ?F_UL2F
        CALL      N:?F_UL2F          ;; 3 cycles
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+22
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+24
        MOVW      AX, [SP+0x1A]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x18]      ;; 1 cycle
          CFI FunCall ?F_UL2F
        CALL      N:?F_UL2F          ;; 3 cycles
          CFI FunCall ?F_DIV
        CALL      N:?F_DIV           ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+20
          CFI FunCall _asin
        CALL      _asin              ;; 3 cycles
          CFI FunCall ?F_MUL
        CALL      N:?F_MUL           ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+16
          CFI FunCall ?F_MUL
        CALL      N:?F_MUL           ;; 3 cycles
          CFI FunCall ?F_F2SL
        CALL      N:?F_F2SL          ;; 3 cycles
        MOVW      HL, AX             ;; 1 cycle
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+12
        BR        N:??power_filter_296  ;; 3 cycles
        ; ------------------------------------- Block: 44 cycles
// 2686     }
// 2687     else if(trig == 1 && active_power != 0 && apparent_power != 0)
??power_filter_295:
        MOV       A, [SP+0x14]       ;; 1 cycle
        CMP       A, #0x1            ;; 1 cycle
        BNZ       ??power_filter_297  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        OR        A, X               ;; 1 cycle
        OR        A, C               ;; 1 cycle
        OR        A, B               ;; 1 cycle
        BZ        ??power_filter_297  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
        MOVW      AX, [SP+0x12]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x10]      ;; 1 cycle
        OR        A, X               ;; 1 cycle
        OR        A, C               ;; 1 cycle
        OR        A, B               ;; 1 cycle
        BZ        ??power_filter_297  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
// 2688     {
// 2689         angle = (s16)DEGREE(acos((double)active_power/apparent_power)*100);
        MOVW      AX, #0x4265        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+14
        MOVW      AX, #0x2EE1        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+16
        MOVW      AX, #0x42C8        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+18
        MOVW      AX, #0x0           ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+20
        MOVW      AX, [SP+0x1A]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x18]      ;; 1 cycle
          CFI FunCall ?F_UL2F
        CALL      N:?F_UL2F          ;; 3 cycles
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+22
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+24
        MOVW      AX, [SP+0x12]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x10]      ;; 1 cycle
          CFI FunCall ?F_UL2F
        CALL      N:?F_UL2F          ;; 3 cycles
          CFI FunCall ?F_DIV
        CALL      N:?F_DIV           ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+20
          CFI FunCall _acos
        CALL      _acos              ;; 3 cycles
          CFI FunCall ?F_MUL
        CALL      N:?F_MUL           ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+16
          CFI FunCall ?F_MUL
        CALL      N:?F_MUL           ;; 3 cycles
          CFI FunCall ?F_F2SL
        CALL      N:?F_F2SL          ;; 3 cycles
        MOVW      HL, AX             ;; 1 cycle
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+12
        BR        S:??power_filter_296  ;; 3 cycles
        ; ------------------------------------- Block: 44 cycles
// 2690     }
// 2691     else if(trig == 2 && reactive_power != 0 && active_power != 0)
??power_filter_297:
        MOV       A, [SP+0x14]       ;; 1 cycle
        CMP       A, #0x2            ;; 1 cycle
        BNZ       ??power_filter_298  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, [SP+0x0E]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x0C]      ;; 1 cycle
        OR        A, X               ;; 1 cycle
        OR        A, C               ;; 1 cycle
        OR        A, B               ;; 1 cycle
        BZ        ??power_filter_298  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        OR        A, X               ;; 1 cycle
        OR        A, C               ;; 1 cycle
        OR        A, B               ;; 1 cycle
        BZ        ??power_filter_298  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
// 2692     {
// 2693         angle = (s16)DEGREE(atan((double)reactive_power/active_power)*100);
        MOVW      AX, #0x4265        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+14
        MOVW      AX, #0x2EE1        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+16
        MOVW      AX, #0x42C8        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+18
        MOVW      AX, #0x0           ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+20
        MOVW      AX, [SP+0x0E]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x0C]      ;; 1 cycle
          CFI FunCall ?F_UL2F
        CALL      N:?F_UL2F          ;; 3 cycles
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+22
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+24
        MOVW      AX, [SP+0x1A]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x18]      ;; 1 cycle
          CFI FunCall ?F_UL2F
        CALL      N:?F_UL2F          ;; 3 cycles
          CFI FunCall ?F_DIV
        CALL      N:?F_DIV           ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+20
          CFI FunCall _atan
        CALL      _atan              ;; 3 cycles
          CFI FunCall ?F_MUL
        CALL      N:?F_MUL           ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+16
          CFI FunCall ?F_MUL
        CALL      N:?F_MUL           ;; 3 cycles
          CFI FunCall ?F_F2SL
        CALL      N:?F_F2SL          ;; 3 cycles
        MOVW      HL, AX             ;; 1 cycle
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+12
        BR        S:??power_filter_296  ;; 3 cycles
        ; ------------------------------------- Block: 44 cycles
// 2694     }
// 2695     else
// 2696     {
// 2697         angle = 0;
??power_filter_298:
        MOVW      HL, #0x0           ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
// 2698     }
// 2699     
// 2700     if(angle > 9000)
??power_filter_296:
        MOVW      AX, HL             ;; 1 cycle
        XOR       A, #0x80           ;; 1 cycle
        CMPW      AX, #0xA329        ;; 1 cycle
        BC        ??power_filter_299  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
// 2701     {
// 2702         angle = 9000;
        MOVW      AX, #0x2328        ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
// 2703     }
// 2704     if(active_f == IMPORT && reactive_f == IMPORT)              /* Q1 */
??power_filter_299:
        MOV       A, [SP+0x02]       ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_300  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOV       A, [SP+0x03]       ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_300  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 2705     {
// 2706         NOP();
        NOP                          ;; 1 cycle
        BR        S:??power_filter_301  ;; 3 cycles
        ; ------------------------------------- Block: 4 cycles
// 2707     }
// 2708     else if(active_f == EXPORT && reactive_f == IMPORT)         /* Q2 */
??power_filter_300:
        MOV       A, [SP+0x02]       ;; 1 cycle
        CMP       A, #0x1            ;; 1 cycle
        BNZ       ??power_filter_302  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOV       A, [SP+0x03]       ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_302  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 2709     {
// 2710         angle = 18000 - angle;
        MOVW      AX, #0x4650        ;; 1 cycle
        SUBW      AX, HL             ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        BR        S:??power_filter_301  ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
// 2711     }
// 2712     else if(active_f == EXPORT && reactive_f == EXPORT)         /* Q3 */
??power_filter_302:
        MOV       A, [SP+0x02]       ;; 1 cycle
        CMP       A, #0x1            ;; 1 cycle
        BNZ       ??power_filter_303  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOV       A, [SP+0x03]       ;; 1 cycle
        CMP       A, #0x1            ;; 1 cycle
        BNZ       ??power_filter_303  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 2713     {
// 2714         angle = -18000 + angle;
        MOVW      AX, HL             ;; 1 cycle
        ADDW      AX, #0xB9B0        ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        BR        S:??power_filter_301  ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
// 2715     }
// 2716     else if(active_f == IMPORT && reactive_f == EXPORT)         /* Q4 */
??power_filter_303:
        MOV       A, [SP+0x02]       ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_301  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOV       A, [SP+0x03]       ;; 1 cycle
        CMP       A, #0x1            ;; 1 cycle
        BNZ       ??power_filter_301  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 2717     {
// 2718         angle = 0 - angle;
        XCHW      AX, HL             ;; 1 cycle
        XOR       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        XOR       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        INCW      AX                 ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
// 2719     }
// 2720     return(angle);
??power_filter_301:
        MOVW      AX, HL             ;; 1 cycle
        ADDW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock16
        ; ------------------------------------- Block: 8 cycles
        ; ------------------------------------- Total: 305 cycles
// 2721 }
// 2722 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock17 Using cfiCommon2
          CFI Function _cal_pf
        CODE
// 2723 us16 cal_pf(us32 active_power, us32 apparent_power)
// 2724 {
_cal_pf:
        ; * Stack frame (at entry) *
        ; Param size: 4
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+6
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+8
        ; Auto size: 6
        SUBW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+10
// 2725     us16 pf = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [SP], AX           ;; 1 cycle
// 2726     if((active_power != 0u) && (apparent_power != 0u))
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        OR        A, X               ;; 1 cycle
        OR        A, C               ;; 1 cycle
        OR        A, B               ;; 1 cycle
        BZ        ??power_filter_304  ;; 4 cycles
        ; ------------------------------------- Block: 15 cycles
        MOVW      AX, [SP+0x0C]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x0A]      ;; 1 cycle
        OR        A, X               ;; 1 cycle
        OR        A, C               ;; 1 cycle
        OR        A, B               ;; 1 cycle
        BZ        ??power_filter_304  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
// 2727     {
// 2728         pf = (us16)(((us32)active_power*1000u)/apparent_power);
        MOVW      AX, [SP+0x0C]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [SP+0x0A]      ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        PUSH      HL                 ;; 1 cycle
          CFI CFA SP+12
        PUSH      DE                 ;; 1 cycle
          CFI CFA SP+14
        MOVW      AX, #0x0           ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+16
        MOVW      AX, #0x3E8         ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+18
        MOVW      AX, [SP+0x0C]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x0A]      ;; 1 cycle
          CFI FunCall ?L_MUL_FAST_L03
        CALL      N:?L_MUL_FAST_L03  ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+14
        POP       DE                 ;; 1 cycle
          CFI CFA SP+12
        POP       HL                 ;; 1 cycle
          CFI CFA SP+10
        DIVWU                        ;; 17 cycles
        NOP                          ;; 1 cycle
        MOVW      [SP], AX           ;; 1 cycle
        BR        S:??power_filter_305  ;; 3 cycles
        ; ------------------------------------- Block: 41 cycles
// 2729     }
// 2730     else if(active_power == 0 && apparent_power != 0)
??power_filter_304:
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        OR        A, X               ;; 1 cycle
        OR        A, C               ;; 1 cycle
        OR        A, B               ;; 1 cycle
        BNZ       ??power_filter_306  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
        MOVW      AX, [SP+0x0C]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x0A]      ;; 1 cycle
        OR        A, X               ;; 1 cycle
        OR        A, C               ;; 1 cycle
        OR        A, B               ;; 1 cycle
        BZ        ??power_filter_306  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
// 2731     {
// 2732         pf = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [SP], AX           ;; 1 cycle
        BR        S:??power_filter_305  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 2733     }
// 2734     else if(active_power == 0 && apparent_power == 0)
??power_filter_306:
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        OR        A, X               ;; 1 cycle
        OR        A, C               ;; 1 cycle
        OR        A, B               ;; 1 cycle
        BNZ       ??power_filter_305  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
        MOVW      AX, [SP+0x0C]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x0A]      ;; 1 cycle
        OR        A, X               ;; 1 cycle
        OR        A, C               ;; 1 cycle
        OR        A, B               ;; 1 cycle
        BNZ       ??power_filter_305  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
// 2735     {
// 2736         pf = 1000;
        MOVW      AX, #0x3E8         ;; 1 cycle
        MOVW      [SP], AX           ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
// 2737     }
// 2738     
// 2739     if((pf > 1000u))
??power_filter_305:
        MOVW      AX, [SP]           ;; 1 cycle
        CMPW      AX, #0x3E9         ;; 1 cycle
        BC        ??power_filter_307  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 2740     {
// 2741         pf = 1000u;
        MOVW      AX, #0x3E8         ;; 1 cycle
        MOVW      [SP], AX           ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
// 2742     }
// 2743     return(pf);
??power_filter_307:
        MOVW      AX, [SP]           ;; 1 cycle
        ADDW      SP, #0x6           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock17
        ; ------------------------------------- Block: 8 cycles
        ; ------------------------------------- Total: 129 cycles
// 2744 }
// 2745 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock18 Using cfiCommon2
          CFI Function _cal_pf_signed
        CODE
// 2746 s16 cal_pf_signed(s32 active_power, s32 apparent_power)
// 2747 {
_cal_pf_signed:
        ; * Stack frame (at entry) *
        ; Param size: 4
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+6
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+8
        ; Auto size: 4
// 2748     s16 pf;
// 2749     if((active_power != 0) && (apparent_power != 0))
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        OR        A, X               ;; 1 cycle
        OR        A, C               ;; 1 cycle
        OR        A, B               ;; 1 cycle
        BZ        ??power_filter_308  ;; 4 cycles
        ; ------------------------------------- Block: 12 cycles
        MOVW      AX, [SP+0x0A]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x08]      ;; 1 cycle
        OR        A, X               ;; 1 cycle
        OR        A, C               ;; 1 cycle
        OR        A, B               ;; 1 cycle
        BZ        ??power_filter_308  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
// 2750     {
// 2751         pf = (s16)(((s32)active_power*1000)/apparent_power);
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [HL+0x02]      ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        MOVW      AX, [HL]           ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      AX, #0x0           ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+14
        MOVW      AX, #0x3E8         ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+16
        MOVW      AX, [SP+0x0A]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x08]      ;; 1 cycle
          CFI FunCall ?L_MUL_FAST_L03
        CALL      N:?L_MUL_FAST_L03  ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+12
          CFI FunCall ?SL_DIV_L03
        CALL      N:?SL_DIV_L03      ;; 3 cycles
        MOVW      DE, AX             ;; 1 cycle
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        BR        S:??power_filter_309  ;; 3 cycles
        ; ------------------------------------- Block: 26 cycles
// 2752     }
// 2753     else if(active_power == 0 && apparent_power != 0)
??power_filter_308:
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        OR        A, X               ;; 1 cycle
        OR        A, C               ;; 1 cycle
        OR        A, B               ;; 1 cycle
        BNZ       ??power_filter_310  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
        MOVW      AX, [SP+0x0A]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x08]      ;; 1 cycle
        OR        A, X               ;; 1 cycle
        OR        A, C               ;; 1 cycle
        OR        A, B               ;; 1 cycle
        BZ        ??power_filter_310  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
// 2754     {
// 2755         pf = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        BR        S:??power_filter_309  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 2756     }
// 2757     else if(active_power == 0 && apparent_power == 0)
??power_filter_310:
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        OR        A, X               ;; 1 cycle
        OR        A, C               ;; 1 cycle
        OR        A, B               ;; 1 cycle
        BNZ       ??power_filter_309  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
        MOVW      AX, [SP+0x0A]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x08]      ;; 1 cycle
        OR        A, X               ;; 1 cycle
        OR        A, C               ;; 1 cycle
        OR        A, B               ;; 1 cycle
        BNZ       ??power_filter_309  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
// 2758     {
// 2759         pf = 1000;
        MOVW      AX, #0x3E8         ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
// 2760     }
// 2761 
// 2762     if(pf > 1000)
??power_filter_309:
        MOVW      AX, DE             ;; 1 cycle
        XOR       A, #0x80           ;; 1 cycle
        CMPW      AX, #0x83E9        ;; 1 cycle
        BC        ??power_filter_311  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
// 2763     {
// 2764         pf = 1000;
        MOVW      AX, #0x3E8         ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        BR        S:??power_filter_312  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 2765     }
// 2766     else if(pf < -1000)
??power_filter_311:
        MOVW      AX, DE             ;; 1 cycle
        XOR       A, #0x80           ;; 1 cycle
        CMPW      AX, #0x7C18        ;; 1 cycle
        BNC       ??power_filter_312  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
// 2767     {
// 2768         pf = -1000;
        MOVW      AX, #0xFC18        ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
// 2769     }
// 2770     return(pf);
??power_filter_312:
        MOVW      AX, DE             ;; 1 cycle
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock18
        ; ------------------------------------- Block: 8 cycles
        ; ------------------------------------- Total: 124 cycles
// 2771 }
// 2772 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock19 Using cfiCommon0
          CFI Function _metrology_1sec_loop
        CODE
// 2773 void metrology_1sec_loop()
// 2774 {
_metrology_1sec_loop:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
// 2775     static us16 freq_last,freq_now = 0;
// 2776     /* Frequency */
// 2777     if(flag_freq_vol_r == 1)
        MOVW      HL, #LWRD(_flag_metro3)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??power_filter_313  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 2778     {
// 2779         freq_now = freq.Rph;
        MOVW      AX, N:_freq        ;; 1 cycle
        MOVW      N:`metrology_1sec_loop::freq_now`, AX  ;; 1 cycle
        BR        S:??power_filter_314  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 2780     }
// 2781     else if(flag_freq_vol_y == 1)
??power_filter_313:
        MOVW      HL, #LWRD(_flag_metro3)  ;; 1 cycle
        MOV1      CY, [HL].1         ;; 1 cycle
        BNC       ??power_filter_315  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 2782     {
// 2783         freq_now = freq.Yph;
        MOVW      AX, N:_freq+2      ;; 1 cycle
        MOVW      N:`metrology_1sec_loop::freq_now`, AX  ;; 1 cycle
        BR        S:??power_filter_314  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 2784     }
// 2785     else if(flag_freq_vol_b == 1)
??power_filter_315:
        MOVW      HL, #LWRD(_flag_metro3)  ;; 1 cycle
        MOV1      CY, [HL].2         ;; 1 cycle
        BNC       ??power_filter_316  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 2786     {
// 2787         freq_now = freq.Bph;
        MOVW      AX, N:_freq+4      ;; 1 cycle
        MOVW      N:`metrology_1sec_loop::freq_now`, AX  ;; 1 cycle
        BR        S:??power_filter_314  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 2788     }
// 2789     else
// 2790     {
// 2791         freq_now = freq.Rph;
??power_filter_316:
        MOVW      AX, N:_freq        ;; 1 cycle
        MOVW      N:`metrology_1sec_loop::freq_now`, AX  ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
// 2792     }
// 2793     
// 2794     freq.Net = ((us32)freq_now + freq_last)/2 ;
??power_filter_314:
        MOVW      AX, N:`metrology_1sec_loop::freq_last`  ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        MOVW      DE, N:`metrology_1sec_loop::freq_now`  ;; 1 cycle
        MOVW      HL, #0x0           ;; 1 cycle
        ADDW      AX, DE             ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SHRW      AX, 0x1            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SHRW      AX, 0x1            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOV1      A.7, CY            ;; 1 cycle
        MOVW      N:_freq+6, AX      ;; 1 cycle
// 2795     freq_last = freq_now;
        MOVW      AX, N:`metrology_1sec_loop::freq_now`  ;; 1 cycle
        MOVW      N:`metrology_1sec_loop::freq_last`, AX  ;; 1 cycle
// 2796     
// 2797     /* frequency variation */
// 2798     freq_variation_timer_update();
          CFI FunCall _freq_variation_timer_update
        CALL      _freq_variation_timer_update  ;; 3 cycles
// 2799     
// 2800     /* phase to phase angle */
// 2801     ph_ph.Ph_RY.angle.value = (us16)((float)ph_ph.Ph_RY.angle.cntr * 36000.0 / r_phase.no_of_samples);
        MOVW      AX, N:_r_phase     ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
          CFI FunCall ?F_UL2F
        CALL      N:?F_UL2F          ;; 3 cycles
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+6
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+8
        MOVW      AX, #0x470C        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        MOVW      AX, #0xA000        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      AX, N:_ph_ph+4     ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
          CFI FunCall ?F_UL2F
        CALL      N:?F_UL2F          ;; 3 cycles
          CFI FunCall ?F_MUL
        CALL      N:?F_MUL           ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
          CFI FunCall ?F_DIV
        CALL      N:?F_DIV           ;; 3 cycles
          CFI FunCall ?F_F2SL
        CALL      N:?F_F2SL          ;; 3 cycles
        MOVW      N:_ph_ph+2, AX     ;; 1 cycle
// 2802     ph_ph.Ph_YB.angle.value = (us16)((float)ph_ph.Ph_YB.angle.cntr * 36000.0 / y_phase.no_of_samples);
        MOVW      AX, N:_y_phase     ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
          CFI FunCall ?F_UL2F
        CALL      N:?F_UL2F          ;; 3 cycles
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+10
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      AX, #0x470C        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+14
        MOVW      AX, #0xA000        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+16
        MOVW      AX, N:_ph_ph+12    ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
          CFI FunCall ?F_UL2F
        CALL      N:?F_UL2F          ;; 3 cycles
          CFI FunCall ?F_MUL
        CALL      N:?F_MUL           ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+12
          CFI FunCall ?F_DIV
        CALL      N:?F_DIV           ;; 3 cycles
          CFI FunCall ?F_F2SL
        CALL      N:?F_F2SL          ;; 3 cycles
        MOVW      N:_ph_ph+10, AX    ;; 1 cycle
// 2803     ph_ph.Ph_BR.angle.value = (us16)((float)ph_ph.Ph_BR.angle.cntr * 36000.0 / b_phase.no_of_samples);
        MOVW      AX, N:_b_phase     ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
          CFI FunCall ?F_UL2F
        CALL      N:?F_UL2F          ;; 3 cycles
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+14
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+16
        MOVW      AX, #0x470C        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+18
        MOVW      AX, #0xA000        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+20
        MOVW      AX, N:_ph_ph+20    ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
          CFI FunCall ?F_UL2F
        CALL      N:?F_UL2F          ;; 3 cycles
          CFI FunCall ?F_MUL
        CALL      N:?F_MUL           ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+16
          CFI FunCall ?F_DIV
        CALL      N:?F_DIV           ;; 3 cycles
          CFI FunCall ?F_F2SL
        CALL      N:?F_F2SL          ;; 3 cycles
        MOVW      N:_ph_ph+18, AX    ;; 1 cycle
// 2804     
// 2805         
// 2806     /* Phase sequence detections */
// 2807     if(vol.Rph.rms >= THR_PHASE_SEQ_HEALTHY && vol.Yph.rms >= THR_PHASE_SEQ_HEALTHY && vol.Bph.rms >= THR_PHASE_SEQ_HEALTHY)
        MOVW      AX, N:_vol         ;; 1 cycle
        ADDW      SP, #0xC           ;; 1 cycle
          CFI CFA SP+4
        CMPW      AX, #0x34BC        ;; 1 cycle
        BC        ??power_filter_317  ;; 4 cycles
        ; ------------------------------------- Block: 112 cycles
        MOVW      AX, N:_vol+6       ;; 1 cycle
        CMPW      AX, #0x34BC        ;; 1 cycle
        BC        ??power_filter_317  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, N:_vol+12      ;; 1 cycle
        CMPW      AX, #0x34BC        ;; 1 cycle
        BC        ??power_filter_317  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 2808     {
// 2809       if(ph_ph.Ph_RY.angle.value >= 10000 && ph_ph.Ph_RY.angle.value < 14000 && 
// 2810          ph_ph.Ph_YB.angle.value >= 10000 && ph_ph.Ph_YB.angle.value < 14000 && 
// 2811            ph_ph.Ph_BR.angle.value >= 10000 && ph_ph.Ph_BR.angle.value < 14000)
        MOVW      AX, N:_ph_ph+2     ;; 1 cycle
        CMPW      AX, #0x2710        ;; 1 cycle
        BC        ??power_filter_318  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, N:_ph_ph+2     ;; 1 cycle
        CMPW      AX, #0x36B0        ;; 1 cycle
        BNC       ??power_filter_318  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, N:_ph_ph+10    ;; 1 cycle
        CMPW      AX, #0x2710        ;; 1 cycle
        BC        ??power_filter_318  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, N:_ph_ph+10    ;; 1 cycle
        CMPW      AX, #0x36B0        ;; 1 cycle
        BNC       ??power_filter_318  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, N:_ph_ph+18    ;; 1 cycle
        CMPW      AX, #0x2710        ;; 1 cycle
        BC        ??power_filter_318  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, N:_ph_ph+18    ;; 1 cycle
        CMPW      AX, #0x36B0        ;; 1 cycle
        BNC       ??power_filter_318  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 2812       {
// 2813         flag_phase_seq = 1;
        SET1      N:_flag_metro1.6   ;; 2 cycles
        BR        S:??power_filter_319  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 2814       }
// 2815       else
// 2816       {
// 2817         flag_phase_seq = 0;
??power_filter_318:
        CLR1      N:_flag_metro1.6   ;; 2 cycles
        BR        S:??power_filter_319  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 2818       }
// 2819     }
// 2820     else 
// 2821     {
// 2822       flag_phase_seq = 1;
??power_filter_317:
        SET1      N:_flag_metro1.6   ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
// 2823     }
// 2824     
// 2825     
// 2826     /* phase to phase voltage */
// 2827     vector_addition_2byte(vol.Rph.rms,vol.Yph.rms,ph_ph.Ph_RY.angle.value,&ph_ph.Ph_RY.vol.value);
??power_filter_319:
        MOVW      AX, #LWRD(_ph_ph)  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+6
        MOVW      DE, N:_ph_ph+2     ;; 1 cycle
        MOVW      BC, N:_vol+6       ;; 1 cycle
        MOVW      AX, N:_vol         ;; 1 cycle
          CFI FunCall _vector_addition_2byte
        CALL      _vector_addition_2byte  ;; 3 cycles
// 2828     vector_addition_2byte(vol.Yph.rms,vol.Bph.rms,ph_ph.Ph_YB.angle.value,&ph_ph.Ph_YB.vol.value);
        MOVW      AX, #LWRD(_ph_ph+8)  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+8
        MOVW      DE, N:_ph_ph+10    ;; 1 cycle
        MOVW      BC, N:_vol+12      ;; 1 cycle
        MOVW      AX, N:_vol+6       ;; 1 cycle
          CFI FunCall _vector_addition_2byte
        CALL      _vector_addition_2byte  ;; 3 cycles
// 2829     vector_addition_2byte(vol.Bph.rms,vol.Rph.rms,ph_ph.Ph_BR.angle.value,&ph_ph.Ph_BR.vol.value);
        MOVW      AX, #LWRD(_ph_ph+16)  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        MOVW      DE, N:_ph_ph+18    ;; 1 cycle
        MOVW      BC, N:_vol         ;; 1 cycle
        MOVW      AX, N:_vol+12      ;; 1 cycle
          CFI FunCall _vector_addition_2byte
        CALL      _vector_addition_2byte  ;; 3 cycles
// 2830     
// 2831     if((ph_ph.Ph_RY.angle.value >= 16300 && ph_ph.Ph_RY.angle.value < 19700) || 
// 2832        (ph_ph.Ph_YB.angle.value >= 16300 && ph_ph.Ph_YB.angle.value < 19700)  || 
// 2833            (ph_ph.Ph_BR.angle.value >= 16300 && ph_ph.Ph_BR.angle.value < 19700))
        MOVW      AX, N:_ph_ph+2     ;; 1 cycle
        ADDW      SP, #0x6           ;; 1 cycle
          CFI CFA SP+4
        CMPW      AX, #0x3FAC        ;; 1 cycle
        BC        ??power_filter_320  ;; 4 cycles
        ; ------------------------------------- Block: 31 cycles
        MOVW      AX, N:_ph_ph+2     ;; 1 cycle
        CMPW      AX, #0x4CF4        ;; 1 cycle
        BC        ??power_filter_321  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
??power_filter_320:
        MOVW      AX, N:_ph_ph+10    ;; 1 cycle
        CMPW      AX, #0x3FAC        ;; 1 cycle
        BC        ??power_filter_322  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, N:_ph_ph+10    ;; 1 cycle
        CMPW      AX, #0x4CF4        ;; 1 cycle
        BC        ??power_filter_321  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
??power_filter_322:
        MOVW      AX, N:_ph_ph+18    ;; 1 cycle
        CMPW      AX, #0x3FAC        ;; 1 cycle
        BC        ??power_filter_323  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, N:_ph_ph+18    ;; 1 cycle
        CMPW      AX, #0x4CF4        ;; 1 cycle
        BNC       ??power_filter_323  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 2834     {
// 2835         flag_metro_two_wire = 1;
??power_filter_321:
        SET1      N:_flag_metro1.5   ;; 2 cycles
        BR        S:??power_filter_324  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 2836     }
// 2837     else
// 2838     {
// 2839         flag_metro_two_wire = 0;
??power_filter_323:
        CLR1      N:_flag_metro1.5   ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
// 2840     }
// 2841     
// 2842     if(bitIsSet(tpr.magnet.flag,event_f) && (MAG_SWITCH_IMAX == 1))
??power_filter_324:
        MOVW      HL, #LWRD(_tpr+32)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??power_filter_325  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 2843     {
// 2844       ACT_THRESHOLD = ACT_THRESHOLD_MAGNET;
        MOVW      HL, #LWRD(_ACT_THRESHOLD)  ;; 1 cycle
        MOVW      AX, #0x8856        ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x66CD        ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x155         ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 2845       REACT_THRESHOLD = REACT_THRESHOLD_MAGNET;
        MOVW      HL, #LWRD(_REACT_THRESHOLD)  ;; 1 cycle
        MOVW      AX, #0x8856        ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x66CD        ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x155         ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 2846       APP_THRESHOLD = APP_THRESHOLD_MAGNET;
        MOVW      HL, #LWRD(_APP_THRESHOLD)  ;; 1 cycle
        MOVW      AX, #0x8856        ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x66CD        ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x155         ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        BR        N:??power_filter_326  ;; 3 cycles
        ; ------------------------------------- Block: 30 cycles
// 2847     }
// 2848     else if(flag_metro_two_wire == 1)
??power_filter_325:
        MOVW      HL, #LWRD(_flag_metro1)  ;; 1 cycle
        MOV1      CY, [HL].5         ;; 1 cycle
        BNC       ??power_filter_327  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 2849     {
// 2850       ACT_THRESHOLD = TWO_WIRE_ACT_THRESHOLD1;
        MOVW      HL, #LWRD(_ACT_THRESHOLD)  ;; 1 cycle
        MOVW      AX, #0xA578        ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0xA8B1        ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0xFF          ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 2851       REACT_THRESHOLD = TWO_WIRE_REACT_THRESHOLD1;
        MOVW      HL, #LWRD(_REACT_THRESHOLD)  ;; 1 cycle
        MOVW      AX, #0x11B5        ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x243A        ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x125         ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 2852       APP_THRESHOLD = TWO_WIRE_APP_THRESHOLD1;
        MOVW      HL, #LWRD(_APP_THRESHOLD)  ;; 1 cycle
        MOVW      AX, #0xA578        ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0xA8B1        ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0xFF          ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        BR        S:??power_filter_326  ;; 3 cycles
        ; ------------------------------------- Block: 30 cycles
// 2853     }
// 2854     else
// 2855     {
// 2856       ACT_THRESHOLD = ACT_THRESHOLD1;
??power_filter_327:
        MOVW      HL, #LWRD(_ACT_THRESHOLD)  ;; 1 cycle
        MOVW      AX, #0xCA70        ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0xF7D         ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x155         ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 2857       REACT_THRESHOLD = REACT_THRESHOLD1;
        MOVW      HL, #LWRD(_REACT_THRESHOLD)  ;; 1 cycle
        MOVW      AX, #0xCA70        ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0xF7D         ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x155         ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 2858       APP_THRESHOLD = APP_THRESHOLD1;
        MOVW      HL, #LWRD(_APP_THRESHOLD)  ;; 1 cycle
        MOVW      AX, #0xCA70        ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0xF7D         ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x155         ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
          CFI FunCall _thd_one_sec_loop
        ; ------------------------------------- Block: 27 cycles
// 2859     }
// 2860 
// 2861     thd_one_sec_loop();
??power_filter_326:
        CALL      _thd_one_sec_loop  ;; 3 cycles
// 2862     
// 2863     if(flag_min_current_r == 1 && flag_min_current_y == 1 && flag_min_current_b == 1)
        MOV       A, N:_flag_metro2  ;; 1 cycle
        AND       A, #0x70           ;; 1 cycle
        CMP       A, #0x70           ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
// 2864     {
// 2865         pwrup_sec_cnt= 1;
        MOV       N:_pwrup_sec_cnt, #0x1  ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
// 2866     }
// 2867     if(pwrup_sec_cnt >= 1)
??metrology_1sec_loop_0:
        CMP0      N:_pwrup_sec_cnt   ;; 1 cycle
        SKZ                          ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
// 2868     {
// 2869         pwrup_sec_cnt++;
        INC       N:_pwrup_sec_cnt   ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
// 2870     }
// 2871     
// 2872     power.Allph.active_filtered = power_filter(power.Allph.active_signed,3);
??metrology_1sec_loop_1:
        MOV       E, #0x3            ;; 1 cycle
        MOVW      BC, N:_power+82    ;; 1 cycle
        MOVW      AX, N:_power+80    ;; 1 cycle
          CFI FunCall _power_filter
        CALL      _power_filter      ;; 3 cycles
        MOVW      N:_power+92, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_power+94, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 2873     
// 2874 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock19
        ; ------------------------------------- Block: 16 cycles
        ; ------------------------------------- Total: 402 cycles

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
`metrology_1sec_loop::freq_last`:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
`metrology_1sec_loop::freq_now`:
        DS 2

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock20 Using cfiCommon0
          CFI Function _freq_variation_timer_update
        CODE
// 2875 void freq_variation_timer_update()
// 2876 {
_freq_variation_timer_update:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
// 2877     //  static us16 freq_prev = 0;
// 2878     //  static us8 freq_changed_cntr = 0;
// 2879     //  
// 2880     //  if(freq.Net >= FREQ_MIN && freq.Net <= FREQ_MAX)
// 2881     //  {
// 2882     //    if(flag_metro_powerup == 0)
// 2883     //    {
// 2884     //      change_active_delay_buffer_par(freq.Net);
// 2885     //      freq_prev = freq.Net;
// 2886     //      flag_metro_powerup = 1;
// 2887     //    }
// 2888     //    if(ABS(freq.Net - freq_prev) >= 20)        /* 0.02 Hz difference */ 50.100
// 2889     //    {
// 2890     //      freq_changed_cntr++;
// 2891     //    }
// 2892     //    else
// 2893     //    {
// 2894     //      freq_changed_cntr = 0;
// 2895     //    }
// 2896     //    if(freq_changed_cntr >= 1)                  /* freq change detected */
// 2897     //    {
// 2898     //      freq_changed_cntr = 0;
// 2899     //      change_active_delay_buffer_par(freq.Net);
// 2900     //      freq_prev = freq.Net;
// 2901     //    }
// 2902     //  }
// 2903     
// 2904     change_active_delay_buffer_par(freq.Net);
        MOVW      AX, N:_freq+6      ;; 1 cycle
          CFI FunCall _change_active_delay_buffer_par
        CALL      _change_active_delay_buffer_par  ;; 3 cycles
// 2905 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock20
        ; ------------------------------------- Block: 10 cycles
        ; ------------------------------------- Total: 10 cycles

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock21 Using cfiCommon0
          CFI Function _change_active_delay_buffer_par
        CODE
// 2906 void change_active_delay_buffer_par(us16 freq)
// 2907 {
_change_active_delay_buffer_par:
        ; * Stack frame (at entry) *
        ; Param size: 0
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+6
        ; Auto size: 2
// 2908     if(freq > FREQ_MIN && freq < FREQ_MAX)
        MOVW      AX, [SP]           ;; 1 cycle
        CMPW      AX, #0x9A4D        ;; 1 cycle
        SKNC                         ;; 4 cycles
        BR        N:??power_filter_328  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOVW      AX, [SP]           ;; 1 cycle
        CMPW      AX, #0xEC54        ;; 1 cycle
        SKC                          ;; 4 cycles
        BR        N:??power_filter_328  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 2909     {
// 2910         r_samp_issue_f = 0;
        MOV       N:_r_samp_issue_f, #0x0  ;; 1 cycle
// 2911         /* calculating required time delay as per frequency */
// 2912         react_time_delay = (250000000.0/freq);
        MOVW      AX, [SP]           ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
          CFI FunCall ?F_UL2F
        CALL      N:?F_UL2F          ;; 3 cycles
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+8
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        MOVW      AX, #0x6B28        ;; 1 cycle
        MOVW      BC, #0x4D6E        ;; 1 cycle
          CFI FunCall ?F_DIV
        CALL      N:?F_DIV           ;; 3 cycles
        MOVW      N:_react_time_delay, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_react_time_delay+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 2913         react_sample_delay = (us8)(react_time_delay/256);
        MOVW      BC, N:_react_time_delay+2  ;; 1 cycle
        MOVW      AX, N:_react_time_delay  ;; 1 cycle
        MOVW      DE, #0x7E00        ;; 1 cycle
          CFI FunCall ___iar_fmex
        CALL      N:___iar_fmex      ;; 3 cycles
          CFI FunCall ?F_F2SL
        CALL      N:?F_F2SL          ;; 3 cycles
        MOV       A, X               ;; 1 cycle
        MOV       N:_react_sample_delay, A  ;; 1 cycle
// 2914         time_delay = (us16)(react_time_delay - react_sample_delay*256);
        MOV       X, N:_react_sample_delay  ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x100         ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
          CFI FunCall ?F_SL2F
        CALL      N:?F_SL2F          ;; 3 cycles
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+12
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+14
        MOVW      BC, N:_react_time_delay+2  ;; 1 cycle
        MOVW      AX, N:_react_time_delay  ;; 1 cycle
          CFI FunCall ?F_SUB
        CALL      N:?F_SUB           ;; 3 cycles
          CFI FunCall ?F_F2SL
        CALL      N:?F_F2SL          ;; 3 cycles
        MOVW      N:_time_delay, AX  ;; 1 cycle
// 2915         /* considering cases when delay is not possible */
// 2916         if(time_delay >= MAX_TIME_DELAY_US || time_delay <= MIN_TIME_DELAY_US)
        MOVW      AX, #0x4367        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+16
        MOVW      AX, #0x0           ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+18
        MOVW      AX, N:_time_delay  ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
          CFI FunCall ?F_UL2F
        CALL      N:?F_UL2F          ;; 3 cycles
          CFI FunCall ?FCMP_GE
        CALL      N:?FCMP_GE         ;; 3 cycles
        ADDW      SP, #0xC           ;; 1 cycle
          CFI CFA SP+6
        BNC       ??power_filter_329  ;; 4 cycles
        ; ------------------------------------- Block: 67 cycles
        MOVW      AX, #0x41C8        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+8
        MOVW      AX, #0x1           ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        MOVW      AX, N:_time_delay  ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
          CFI FunCall ?F_UL2F
        CALL      N:?F_UL2F          ;; 3 cycles
          CFI FunCall ?FCMP_LT
        CALL      N:?FCMP_LT         ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+6
        BNC       ??power_filter_330  ;; 4 cycles
        ; ------------------------------------- Block: 17 cycles
// 2917         {
// 2918             r_samp_issue_f = 1;
??power_filter_329:
        MOV       N:_r_samp_issue_f, #0x1  ;; 1 cycle
// 2919             if(time_delay >= MAX_TIME_DELAY_US)
        MOVW      AX, #0x4367        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+8
        MOVW      AX, #0x0           ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        MOVW      AX, N:_time_delay  ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
          CFI FunCall ?F_UL2F
        CALL      N:?F_UL2F          ;; 3 cycles
          CFI FunCall ?FCMP_GE
        CALL      N:?FCMP_GE         ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+6
        SKC                          ;; 1 cycle
        ; ------------------------------------- Block: 15 cycles
// 2920             {
// 2921                 react_sample_delay++;
        INC       N:_react_sample_delay  ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
// 2922             }
// 2923             
// 2924             time_delay = MIN_TIME_DELAY_US;
??change_active_delay_buffer_par_0:
        MOVW      AX, #0x19          ;; 1 cycle
        MOVW      N:_time_delay, AX  ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
// 2925         }
// 2926         time_deviation = (us16)(react_time_delay - (react_sample_delay*256+time_delay));
??power_filter_330:
        MOV       X, N:_react_sample_delay  ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x100         ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, N:_time_delay  ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
          CFI FunCall ?F_UL2F
        CALL      N:?F_UL2F          ;; 3 cycles
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+8
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        MOVW      BC, N:_react_time_delay+2  ;; 1 cycle
        MOVW      AX, N:_react_time_delay  ;; 1 cycle
          CFI FunCall ?F_SUB
        CALL      N:?F_SUB           ;; 3 cycles
          CFI FunCall ?F_F2SL
        CALL      N:?F_F2SL          ;; 3 cycles
        MOVW      N:_time_deviation, AX  ;; 1 cycle
// 2927         
// 2928         
// 2929         timer_value = (unsigned int)TIME_US_TO_TIMER(256-time_delay);
        MOVW      AX, #0x100         ;; 1 cycle
        SUBW      AX, N:_time_delay  ;; 1 cycle
        MOVW      BC, #0xC           ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      N:_timer_value, AX  ;; 1 cycle
// 2930         R_TAU0_Channel1_SetValue(timer_value);
        MOVW      AX, N:_timer_value  ;; 1 cycle
          CFI FunCall _R_TAU0_Channel1_SetValue
        CALL      _R_TAU0_Channel1_SetValue  ;; 3 cycles
// 2931         
// 2932         if(r_samp_issue_f == 1)
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+6
        CMP       N:_r_samp_issue_f, #0x1  ;; 1 cycle
        BNZ       ??power_filter_331  ;; 4 cycles
        ; ------------------------------------- Block: 37 cycles
// 2933         {
// 2934             flag_metro_app_cal_method = APP_CAL_VI;
        CLR1      N:_flag_metro1.1   ;; 2 cycles
// 2935             flag_metro_react_cal_method = REACT_CAL_POW_TRIANGLE;
        SET1      N:_flag_metro1.3   ;; 2 cycles
        BR        S:??power_filter_328  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 2936         }
// 2937         else
// 2938         {
// 2939             flag_metro_app_cal_method = APP_CAL_POW_TRIANGLE;
??power_filter_331:
        SET1      N:_flag_metro1.1   ;; 2 cycles
// 2940             flag_metro_react_cal_method = REACT_CAL_DELAY;
        CLR1      N:_flag_metro1.3   ;; 2 cycles
        ; ------------------------------------- Block: 4 cycles
// 2941         }
// 2942     }
// 2943 }
??power_filter_328:
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock21
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 171 cycles
// 2944 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock22 Using cfiCommon0
          CFI Function _zerocross_detection
        CODE
// 2945 inline void zerocross_detection()
// 2946 {
_zerocross_detection:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
// 2947     static us8 r_zc_check_cntr = 0,y_zc_check_cntr = 0,b_zc_check_cntr = 0;
// 2948     static us8 zc_power_fail_timer;
// 2949     /* zerocross detected flag setting */
// 2950     flag_zc_r_detect = 0;
        CLR1      N:_flag_zc.0       ;; 2 cycles
// 2951     if(r_zc_check_cntr == 0)
        CMP0      N:__ZZ19zerocross_detectionE15r_zc_check_cntr  ;; 1 cycle
        BNZ       ??power_filter_332  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
// 2952     {
// 2953         if((r_phase.vol.sample >= 0 && r_phase.vol.sample_old < 0) && (r_phase.vol.sample - r_phase.vol.sample_old) >= THR_SLOPE_AT_ZEROCROSS)
        MOVW      AX, N:_r_phase+8   ;; 1 cycle
        BT        A.7, ??power_filter_333  ;; 5 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, N:_r_phase+16  ;; 1 cycle
        BF        A.7, ??power_filter_333  ;; 5 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, N:_r_phase+8   ;; 1 cycle
        SUBW      AX, N:_r_phase+16  ;; 1 cycle
        XOR       A, #0x80           ;; 1 cycle
        CMPW      AX, #0x8003        ;; 1 cycle
        BC        ??power_filter_333  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
// 2954         {
// 2955             r_zc_check_cntr = THR_ZC_CHECK_DELAY;
        MOV       N:__ZZ19zerocross_detectionE15r_zc_check_cntr, #0x3A  ;; 1 cycle
// 2956             flag_zc_r_detect = 1;
        SET1      N:_flag_zc.0       ;; 2 cycles
// 2957             r_phase.vol.zc_cntr++;
        INCW      N:_r_phase+4       ;; 2 cycles
        BR        S:??power_filter_333  ;; 3 cycles
        ; ------------------------------------- Block: 8 cycles
// 2958         }
// 2959     }
// 2960     else
// 2961     {
// 2962         r_zc_check_cntr--;
??power_filter_332:
        DEC       N:__ZZ19zerocross_detectionE15r_zc_check_cntr  ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
// 2963     }
// 2964     
// 2965     flag_zc_y_detect = 0;
??power_filter_333:
        CLR1      N:_flag_zc.1       ;; 2 cycles
// 2966     if(y_zc_check_cntr == 0)
        CMP0      N:__ZZ19zerocross_detectionE15y_zc_check_cntr  ;; 1 cycle
        BNZ       ??power_filter_334  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
// 2967     {
// 2968         if((y_phase.vol.sample >= 0 && y_phase.vol.sample_old < 0) && (y_phase.vol.sample - y_phase.vol.sample_old) >= THR_SLOPE_AT_ZEROCROSS)
        MOVW      AX, N:_y_phase+8   ;; 1 cycle
        BT        A.7, ??power_filter_335  ;; 5 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, N:_y_phase+16  ;; 1 cycle
        BF        A.7, ??power_filter_335  ;; 5 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, N:_y_phase+8   ;; 1 cycle
        SUBW      AX, N:_y_phase+16  ;; 1 cycle
        XOR       A, #0x80           ;; 1 cycle
        CMPW      AX, #0x8003        ;; 1 cycle
        BC        ??power_filter_335  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
// 2969         {
// 2970             y_zc_check_cntr = THR_ZC_CHECK_DELAY;
        MOV       N:__ZZ19zerocross_detectionE15y_zc_check_cntr, #0x3A  ;; 1 cycle
// 2971             flag_zc_y_detect = 1;
        SET1      N:_flag_zc.1       ;; 2 cycles
// 2972             y_phase.vol.zc_cntr++;
        INCW      N:_y_phase+4       ;; 2 cycles
        BR        S:??power_filter_335  ;; 3 cycles
        ; ------------------------------------- Block: 8 cycles
// 2973         }
// 2974     }
// 2975     else
// 2976     {
// 2977         y_zc_check_cntr--;
??power_filter_334:
        DEC       N:__ZZ19zerocross_detectionE15y_zc_check_cntr  ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
// 2978     }
// 2979     
// 2980     flag_zc_b_detect = 0;
??power_filter_335:
        CLR1      N:_flag_zc.2       ;; 2 cycles
// 2981     if(b_zc_check_cntr == 0)
        CMP0      N:__ZZ19zerocross_detectionE15b_zc_check_cntr  ;; 1 cycle
        BNZ       ??power_filter_336  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
// 2982     {
// 2983         if((b_phase.vol.sample >= 0 && b_phase.vol.sample_old < 0) && (b_phase.vol.sample - b_phase.vol.sample_old) >= THR_SLOPE_AT_ZEROCROSS)
        MOVW      AX, N:_b_phase+8   ;; 1 cycle
        BT        A.7, ??power_filter_337  ;; 5 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, N:_b_phase+16  ;; 1 cycle
        BF        A.7, ??power_filter_337  ;; 5 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, N:_b_phase+8   ;; 1 cycle
        SUBW      AX, N:_b_phase+16  ;; 1 cycle
        XOR       A, #0x80           ;; 1 cycle
        CMPW      AX, #0x8003        ;; 1 cycle
        BC        ??power_filter_337  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
// 2984         {
// 2985             b_zc_check_cntr = THR_ZC_CHECK_DELAY;
        MOV       N:__ZZ19zerocross_detectionE15b_zc_check_cntr, #0x3A  ;; 1 cycle
// 2986             flag_zc_b_detect = 1;
        SET1      N:_flag_zc.2       ;; 2 cycles
// 2987             b_phase.vol.zc_cntr++;
        INCW      N:_b_phase+4       ;; 2 cycles
        BR        S:??power_filter_337  ;; 3 cycles
        ; ------------------------------------- Block: 8 cycles
// 2988         }
// 2989     }
// 2990     else
// 2991     {
// 2992         b_zc_check_cntr--;
??power_filter_336:
        DEC       N:__ZZ19zerocross_detectionE15b_zc_check_cntr  ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
// 2993     }
// 2994     
// 2995     r_phase.vol.sample_old = r_phase.vol.sample;
??power_filter_337:
        MOVW      AX, N:_r_phase+8   ;; 1 cycle
        MOVW      N:_r_phase+16, AX  ;; 1 cycle
// 2996     y_phase.vol.sample_old = y_phase.vol.sample;
        MOVW      AX, N:_y_phase+8   ;; 1 cycle
        MOVW      N:_y_phase+16, AX  ;; 1 cycle
// 2997     b_phase.vol.sample_old = b_phase.vol.sample;
        MOVW      AX, N:_b_phase+8   ;; 1 cycle
        MOVW      N:_b_phase+16, AX  ;; 1 cycle
// 2998     
// 2999     flag_zc_detect = flag_zc_r_detect | flag_zc_y_detect | flag_zc_b_detect;
        MOVW      HL, #LWRD(_flag_zc)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        MOVW      HL, #LWRD(_flag_zc)  ;; 1 cycle
        OR1       CY, [HL].1         ;; 1 cycle
        MOVW      HL, #LWRD(_flag_zc)  ;; 1 cycle
        OR1       CY, [HL].2         ;; 1 cycle
        MOVW      HL, #LWRD(_flag_zc)  ;; 1 cycle
        MOV1      [HL].3, CY         ;; 2 cycles
// 3000     if(flag_zc_detect == 1)
        MOVW      HL, #LWRD(_flag_zc)  ;; 1 cycle
        MOV1      CY, [HL].3         ;; 1 cycle
        SKNC                         ;; 1 cycle
        ; ------------------------------------- Block: 18 cycles
// 3001     {
// 3002         NOP();
        NOP                          ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
// 3003     }
// 3004     if(thd_cal_start == 1)
??zerocross_detection_0:
        MOVW      HL, #LWRD(_flag_thd1)  ;; 1 cycle
        MOV1      CY, [HL].7         ;; 1 cycle
        BNC       ??power_filter_338  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 3005     {
// 3006         if((flag_zc_r_detect == 1 && flag_thd_cal_r == 1) || 
// 3007            (flag_zc_y_detect == 1 && flag_thd_cal_y == 1) ||
// 3008                (flag_zc_b_detect == 1 && flag_thd_cal_b == 1))
        MOVW      HL, #LWRD(_flag_zc)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??power_filter_339  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      HL, #LWRD(_flag_thd1)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BC        ??power_filter_340  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
??power_filter_339:
        MOVW      HL, #LWRD(_flag_zc)  ;; 1 cycle
        MOV1      CY, [HL].1         ;; 1 cycle
        BNC       ??power_filter_341  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      HL, #LWRD(_flag_thd1)  ;; 1 cycle
        MOV1      CY, [HL].2         ;; 1 cycle
        BC        ??power_filter_340  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
??power_filter_341:
        MOVW      HL, #LWRD(_flag_zc)  ;; 1 cycle
        MOV1      CY, [HL].2         ;; 1 cycle
        BNC       ??power_filter_338  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      HL, #LWRD(_flag_thd1)  ;; 1 cycle
        MOV1      CY, [HL].4         ;; 1 cycle
        SKNC                         ;; 1 cycle
        ; ------------------------------------- Block: 3 cycles
// 3009         {
// 3010             flag_thd_start_sampling = 1;
??power_filter_340:
        SET1      N:_flag_thd1.6     ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
// 3011         }
// 3012     }
// 3013     /* flag_zc_power_fail setting */
// 3014     if(flag_zc_detect == 0 && flag_zc_power_fail == 0)
??power_filter_338:
        MOV       A, N:_flag_zc      ;; 1 cycle
        AND       A, #0x18           ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_342  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
// 3015     {
// 3016         zc_power_fail_timer++;
        INC       N:__ZZ19zerocross_detectionE19zc_power_fail_timer  ;; 2 cycles
// 3017         if(zc_power_fail_timer >= THR_ZC_POW_FAIL_SAMPLES)
        MOV       A, N:__ZZ19zerocross_detectionE19zc_power_fail_timer  ;; 1 cycle
        CMP       A, #0xC3           ;; 1 cycle
        BC        ??power_filter_342  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
// 3018         {
// 3019             flag_zc_power_fail = 1;
        SET1      N:_flag_zc.4       ;; 2 cycles
// 3020             zc_power_fail_timer = 0;
        MOV       N:__ZZ19zerocross_detectionE19zc_power_fail_timer, #0x0  ;; 1 cycle
        ; ------------------------------------- Block: 3 cycles
// 3021         }
// 3022     }
// 3023     if(flag_zc_detect == 1 && flag_zc_power_fail == 0) 
??power_filter_342:
        MOV       A, N:_flag_zc      ;; 1 cycle
        AND       A, #0x18           ;; 1 cycle
        CMP       A, #0x8            ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 4 cycles
// 3024     {
// 3025         zc_power_fail_timer = 0;
        MOV       N:__ZZ19zerocross_detectionE19zc_power_fail_timer, #0x0  ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
// 3026     }
// 3027     if(flag_zc_detect == 1 && flag_zc_power_fail == 1) 
??zerocross_detection_1:
        MOV       A, N:_flag_zc      ;; 1 cycle
        AND       A, #0x18           ;; 1 cycle
        CMP       A, #0x18           ;; 1 cycle
        BNZ       ??power_filter_343  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
// 3028     {
// 3029         zc_power_fail_timer++;
        INC       N:__ZZ19zerocross_detectionE19zc_power_fail_timer  ;; 2 cycles
// 3030         if(zc_power_fail_timer >= THR_ZC_POW_RESTORE_CYCLES)
        MOV       A, N:__ZZ19zerocross_detectionE19zc_power_fail_timer  ;; 1 cycle
        CMP       A, #0x3            ;; 1 cycle
        BC        ??power_filter_343  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
// 3031         {
// 3032             flag_zc_power_fail = 0;
        CLR1      N:_flag_zc.4       ;; 2 cycles
// 3033             zc_power_fail_timer = 0;
        MOV       N:__ZZ19zerocross_detectionE19zc_power_fail_timer, #0x0  ;; 1 cycle
        ; ------------------------------------- Block: 3 cycles
// 3034         }
// 3035     }
// 3036 
// 3037 #if DEBUG_MODE == 1
// 3038     if(debug_data_zc_find == 1)
// 3039     {
// 3040         if((debug_data_type == '1' && flag_zc_r_detect == 1) || 
// 3041             (debug_data_type == '2' && flag_zc_y_detect == 2) ||
// 3042                 (debug_data_type == '3' && flag_zc_b_detect == 3))
// 3043         {
// 3044           debug_data_zc_found = 1;
// 3045           debug_data_zc_find = 0;
// 3046         }
// 3047     } 
// 3048 #endif
// 3049     /* sleep mode */
// 3050     if(flag_zc_power_fail == 1)
??power_filter_343:
        MOVW      HL, #LWRD(_flag_zc)  ;; 1 cycle
        MOV1      CY, [HL].4         ;; 1 cycle
        BNC       ??power_filter_344  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 3051     {
// 3052         flag_sleep_zc_fail = 1;
        SET1      N:_flag_sleep.0    ;; 2 cycles
// 3053         sleep_action_early_detection();
          CFI FunCall _sleep_action_early_detection
        CALL      _sleep_action_early_detection  ;; 3 cycles
        RET                          ;; 6 cycles
        ; ------------------------------------- Block: 11 cycles
// 3054     }
// 3055     else
// 3056     {
// 3057         flag_sleep_zc_fail = 0;
??power_filter_344:
        CLR1      N:_flag_sleep.0    ;; 2 cycles
// 3058     }
// 3059 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock22
        ; ------------------------------------- Block: 8 cycles
        ; ------------------------------------- Total: 237 cycles

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
        SECTION_GROUP __ZZ19zerocross_detectionE15r_zc_check_cntr
__ZZ19zerocross_detectionE15r_zc_check_cntr:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
        SECTION_GROUP __ZZ19zerocross_detectionE15y_zc_check_cntr
__ZZ19zerocross_detectionE15y_zc_check_cntr:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
        SECTION_GROUP __ZZ19zerocross_detectionE15b_zc_check_cntr
__ZZ19zerocross_detectionE15b_zc_check_cntr:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
        SECTION_GROUP __ZZ19zerocross_detectionE19zc_power_fail_timer
__ZZ19zerocross_detectionE19zc_power_fail_timer:
        DS 1
// 3060 
// 3061 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock23 Using cfiCommon2
          CFI Function _calculate_irms
        CODE
// 3062 inline us32 calculate_irms(us64 acc_cnt, us16 cal_coeff, us16 no_of_samples)
// 3063 {
_calculate_irms:
        ; * Stack frame (at entry) *
        ; Param size: 12
        ; Auto size: 8
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+12
// 3064     temp_us64 = acc_cnt;
        MOVW      HL, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0xC           ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 3065     temp_us64 *= cal_coeff;
        MOVW      AX, [SP+0x14]      ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+14
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+16
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
// 3066     temp_us64 /= no_of_samples;
        MOVW      AX, [SP+0x1A]      ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+18
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+20
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
// 3067     temp_us64 /= 1000;
        MOVW      DE, #LWRD(__Constant_3e8_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
// 3068     temp_us64 = (us64)sqrt(temp_us64);
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __LLU2F
        CALL      __LLU2F            ;; 3 cycles
          CFI FunCall _sqrt
        CALL      _sqrt              ;; 3 cycles
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+22
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+24
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __F2LLU
        CALL      __F2LLU            ;; 3 cycles
// 3069     return(temp_us64);
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __LLU2L
        CALL      __LLU2L            ;; 3 cycles
        ADDW      SP, #0x14          ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock23
        ; ------------------------------------- Block: 77 cycles
        ; ------------------------------------- Total: 77 cycles
// 3070 }

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock24 Using cfiCommon0
          CFI Function _calculate_vrms
        CODE
// 3071 inline us16 calculate_vrms(us64 acc_cnt, us16 cal_coeff, us16 no_of_samples)
// 3072 {
_calculate_vrms:
        ; * Stack frame (at entry) *
        ; Param size: 12
        ; Auto size: 8
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+12
// 3073     temp_us64 = acc_cnt;
        MOVW      HL, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0xC           ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 3074     temp_us64 *= cal_coeff;
        MOVW      AX, [SP+0x14]      ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+14
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+16
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
// 3075     temp_us64 /= no_of_samples;
        MOVW      AX, [SP+0x1A]      ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+18
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+20
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
// 3076     temp_us64 = (us64)sqrt(temp_us64);
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __LLU2F
        CALL      __LLU2F            ;; 3 cycles
          CFI FunCall _sqrt
        CALL      _sqrt              ;; 3 cycles
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+22
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+24
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __F2LLU
        CALL      __F2LLU            ;; 3 cycles
// 3077     return(temp_us64);
        MOVW      HL, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, [HL]           ;; 1 cycle
        ADDW      SP, #0x14          ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock24
        ; ------------------------------------- Block: 69 cycles
        ; ------------------------------------- Total: 69 cycles
// 3078 }

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock25 Using cfiCommon0
          CFI Function _get_hr_energy
        CODE
// 3079 us64 get_hr_energy(us32 energy,us8 pulse)
// 3080 {
_get_hr_energy:
        ; * Stack frame (at entry) *
        ; Param size: 2
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+6
        PUSH      DE                 ;; 1 cycle
          CFI CFA SP+8
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+10
        ; Auto size: 22
        SUBW      SP, #0x10          ;; 1 cycle
          CFI CFA SP+26
// 3081     us64 temp = 0;
        MOVW      AX, SP             ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 3082     temp = energy;
        MOVW      AX, [SP+0x12]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x10]      ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+28
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+30
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
          CFI FunCall __L2LLU
        CALL      __L2LLU            ;; 3 cycles
// 3083     temp += QUANTA*(pulse/PULSE);
        MOV       A, [SP+0x1E]       ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        MOV       X, #0x6            ;; 1 cycle
        MOV       A, B               ;; 1 cycle
          CFI FunCall ?UC_DIV_L01
        CALL      N:?UC_DIV_L01      ;; 3 cycles
        MOV       C, A               ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOVW      AX, #0x5           ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+32
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+34
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
          CFI FunCall __L2LLU
        CALL      __L2LLU            ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
          CFI FunCall __Add64
        CALL      __Add64            ;; 3 cycles
// 3084     temp = (temp*ten_power[3]);
        MOVW      AX, N:_ten_power+14  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+36
        MOVW      AX, N:_ten_power+12  ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+38
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x14          ;; 1 cycle
          CFI FunCall __L2LLU
        CALL      __L2LLU            ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x14          ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0xC           ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0xC           ;; 1 cycle
          CFI FunCall __Mul64
        CALL      __Mul64            ;; 3 cycles
// 3085     temp += PULSE_WEIGHT_TABLE[pulse%PULSE];
        MOV       X, #0x6            ;; 1 cycle
        MOV       A, [SP+0x26]       ;; 1 cycle
          CFI FunCall ?UC_MOD_L01
        CALL      N:?UC_MOD_L01      ;; 3 cycles
        MOV       C, A               ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOVW      AX, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(_PULSE_WEIGHT_TABLE)  ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [HL]           ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+40
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+42
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x18          ;; 1 cycle
          CFI FunCall __L2LLU
        CALL      __L2LLU            ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x18          ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
          CFI FunCall __Add64
        CALL      __Add64            ;; 3 cycles
        MOVW      AX, [SP+0x24]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        ADDW      SP, #0x10          ;; 1 cycle
          CFI CFA SP+26
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
        MOVW      AX, HL             ;; 1 cycle
        ADDW      SP, #0x16          ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock25
        ; ------------------------------------- Block: 128 cycles
        ; ------------------------------------- Total: 128 cycles
// 3086     return temp;
// 3087 
// 3088 }

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock26 Using cfiCommon0
          CFI Function _get_quadrant
          CFI NoCalls
        CODE
// 3089 us8 get_quadrant(us8 act_flag, us8 react_flag)
// 3090 {
_get_quadrant:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
        MOV       B, A               ;; 1 cycle
// 3091     us8 quad;
// 3092     if(METERING_MODE == NET)
        CMP       N:_METERING_MODE, #0x1  ;; 1 cycle
        BNZ       ??power_filter_345  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 3093     {
// 3094         if(act_flag == IMPORT)
        CMP0      B                  ;; 1 cycle
        BNZ       ??power_filter_346  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 3095         {
// 3096             if(react_flag == IMPORT)
        CMP0      X                  ;; 1 cycle
        BNZ       ??power_filter_347  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 3097             {
// 3098                 quad = Q1;
        MOV       A, #0x1            ;; 1 cycle
        RET                          ;; 6 cycles
        ; ------------------------------------- Block: 7 cycles
// 3099             }
// 3100             else
// 3101             {
// 3102                 quad = Q4;
??power_filter_347:
        MOV       A, #0x4            ;; 1 cycle
        RET                          ;; 6 cycles
        ; ------------------------------------- Block: 7 cycles
// 3103             }
// 3104         }
// 3105         else
// 3106         {
// 3107             if(react_flag == IMPORT)
??power_filter_346:
        CMP0      X                  ;; 1 cycle
        BNZ       ??power_filter_348  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 3108             {
// 3109                 quad = Q2;
        MOV       A, #0x2            ;; 1 cycle
        RET                          ;; 6 cycles
        ; ------------------------------------- Block: 7 cycles
// 3110             }
// 3111             else
// 3112             {
// 3113                 quad = Q3;
??power_filter_348:
        MOV       A, #0x3            ;; 1 cycle
        RET                          ;; 6 cycles
        ; ------------------------------------- Block: 7 cycles
// 3114             }
// 3115         }
// 3116     }
// 3117     else
// 3118     {
// 3119         if((act_flag == IMPORT && react_flag == IMPORT) || (act_flag == EXPORT && react_flag == EXPORT))
??power_filter_345:
        CMP0      B                  ;; 1 cycle
        BNZ       ??power_filter_349  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        CMP0      X                  ;; 1 cycle
        BZ        ??power_filter_350  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
??power_filter_349:
        XCH       A, B               ;; 1 cycle
        CMP       A, #0x1            ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        BNZ       ??power_filter_351  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        XCH       A, X               ;; 1 cycle
        CMP       A, #0x1            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        BNZ       ??power_filter_351  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
// 3120         {
// 3121             quad = Q1;
??power_filter_350:
        MOV       A, #0x1            ;; 1 cycle
        RET                          ;; 6 cycles
        ; ------------------------------------- Block: 7 cycles
// 3122         }
// 3123         else
// 3124         {
// 3125             quad = Q4;
??power_filter_351:
        MOV       A, #0x4            ;; 1 cycle
// 3126         }
// 3127     }
// 3128     return quad;
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock26
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 87 cycles
// 3129 }
// 3130 //inline s32 calculate_idc(s32 acc_cnt, us16 cal_coeff, us16 no_of_samples)
// 3131 //{
// 3132 //    temp_s32 = acc_cnt;
// 3133 //    temp_s32 *= cal_coeff;
// 3134 //    temp_s32 /= no_of_samples;
// 3135 //    //temp_s32 /= 1000;
// 3136 //    //temp_s32 = (us64)sqrt(temp_s32);
// 3137 //    return(temp_s32);
// 3138 //}
// 3139 //inline s16 calculate_vdc(s32 acc_cnt, us16 cal_coeff, us16 no_of_samples)
// 3140 //{
// 3141 //    temp_s32 = acc_cnt;
// 3142 //    temp_s32 *= cal_coeff;
// 3143 //    temp_s32 /= no_of_samples;
// 3144 //    //temp_s32 = (us64)sqrt(temp_s32);
// 3145 //    return(temp_s32);
// 3146 //}

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock27 Using cfiCommon2
          CFI Function _cal_neu_current
        CODE
// 3147 us32 cal_neu_current()
// 3148 {
_cal_neu_current:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 16
        SUBW      SP, #0x10          ;; 1 cycle
          CFI CFA SP+20
// 3149     /* Pending, the code below may need modifications for FWD and net meter */
// 3150     
// 3151     us32 net_curr = 0;
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0xC           ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
// 3152     float ang,horz,vert;
// 3153     
// 3154     ang = (float)pf.Rph;
        MOVW      AX, N:_pf          ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
          CFI FunCall ?F_UL2F
        CALL      N:?F_UL2F          ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+16
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+18
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+20
// 3155     ang = acos(ang/1000);
        MOVW      AX, #0x447A        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+22
        MOVW      AX, #0x0           ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+24
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
          CFI FunCall ?F_DIV
        CALL      N:?F_DIV           ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+20
          CFI FunCall _acos
        CALL      _acos              ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+16
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+18
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+20
// 3156     if(flag_Rph_active == EXPORT)
        MOVW      HL, #LWRD(_flag_quadrant)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??power_filter_352  ;; 4 cycles
        ; ------------------------------------- Block: 39 cycles
// 3157     {
// 3158         ang += PI;
        MOVW      AX, #0x4049        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+22
        MOVW      AX, #0xFDB         ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+24
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
          CFI FunCall ?F_ADD
        CALL      N:?F_ADD           ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+20
        MOVW      HL, SP             ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ; ------------------------------------- Block: 16 cycles
// 3159     }
// 3160     if(quadrant.Rph == Q2 || quadrant.Rph == Q4)
??power_filter_352:
        CMP       N:_quadrant, #0x2  ;; 1 cycle
        BZ        ??power_filter_353  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        CMP       N:_quadrant, #0x4  ;; 1 cycle
        BNZ       ??power_filter_354  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 3161     {
// 3162         horz = curr.Rph.rms*cos(ang);
??power_filter_353:
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
          CFI FunCall _cos
        CALL      _cos               ;; 3 cycles
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+22
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+24
        MOVW      BC, N:_curr+2      ;; 1 cycle
        MOVW      AX, N:_curr        ;; 1 cycle
          CFI FunCall ?F_UL2F
        CALL      N:?F_UL2F          ;; 3 cycles
          CFI FunCall ?F_MUL
        CALL      N:?F_MUL           ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+20
        XCHW      AX, HL             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3163         vert = curr.Rph.rms*sin(ang);
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
          CFI FunCall _sin
        CALL      _sin               ;; 3 cycles
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+22
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+24
        MOVW      BC, N:_curr+2      ;; 1 cycle
        MOVW      AX, N:_curr        ;; 1 cycle
          CFI FunCall ?F_UL2F
        CALL      N:?F_UL2F          ;; 3 cycles
          CFI FunCall ?F_MUL
        CALL      N:?F_MUL           ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+20
        XCHW      AX, HL             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        BR        S:??power_filter_355  ;; 3 cycles
        ; ------------------------------------- Block: 53 cycles
// 3164     }
// 3165     else
// 3166     {
// 3167         horz =  curr.Rph.rms*cos(ang);
??power_filter_354:
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
          CFI FunCall _cos
        CALL      _cos               ;; 3 cycles
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+22
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+24
        MOVW      BC, N:_curr+2      ;; 1 cycle
        MOVW      AX, N:_curr        ;; 1 cycle
          CFI FunCall ?F_UL2F
        CALL      N:?F_UL2F          ;; 3 cycles
          CFI FunCall ?F_MUL
        CALL      N:?F_MUL           ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+20
        XCHW      AX, HL             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3168         vert = -(float)(curr.Rph.rms*sin(ang));
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
          CFI FunCall _sin
        CALL      _sin               ;; 3 cycles
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+22
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+24
        MOVW      BC, N:_curr+2      ;; 1 cycle
        MOVW      AX, N:_curr        ;; 1 cycle
          CFI FunCall ?F_UL2F
        CALL      N:?F_UL2F          ;; 3 cycles
          CFI FunCall ?F_MUL
        CALL      N:?F_MUL           ;; 3 cycles
        XCH       A, B               ;; 1 cycle
        XOR       A, #0x80           ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+20
        XCHW      AX, HL             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ; ------------------------------------- Block: 53 cycles
// 3169     }
// 3170     
// 3171     
// 3172     ang = (float)(pf.Yph);
??power_filter_355:
        MOVW      AX, N:_pf+2        ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
          CFI FunCall ?F_UL2F
        CALL      N:?F_UL2F          ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+16
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+18
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+20
// 3173     ang = acos(ang/1000);
        MOVW      AX, #0x447A        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+22
        MOVW      AX, #0x0           ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+24
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
          CFI FunCall ?F_DIV
        CALL      N:?F_DIV           ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+20
          CFI FunCall _acos
        CALL      _acos              ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+16
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+18
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+20
// 3174     if(flag_Yph_active == EXPORT)
        MOVW      HL, #LWRD(_flag_quadrant)  ;; 1 cycle
        MOV1      CY, [HL].2         ;; 1 cycle
        BNC       ??power_filter_356  ;; 4 cycles
        ; ------------------------------------- Block: 31 cycles
// 3175     {
// 3176         ang += PI;
        MOVW      AX, #0x4049        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+22
        MOVW      AX, #0xFDB         ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+24
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
          CFI FunCall ?F_ADD
        CALL      N:?F_ADD           ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+20
        MOVW      HL, SP             ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ; ------------------------------------- Block: 16 cycles
// 3177     }
// 3178     if(quadrant.Yph == Q2 || quadrant.Yph == Q4)
??power_filter_356:
        CMP       N:_quadrant+1, #0x2  ;; 1 cycle
        BZ        ??power_filter_357  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        CMP       N:_quadrant+1, #0x4  ;; 1 cycle
        SKZ                          ;; 4 cycles
        BR        N:??power_filter_358  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 3179     {
// 3180         horz -= curr.Yph.rms*cos(PI_BY_THREE-ang);
??power_filter_357:
        MOVW      AX, SP             ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [HL+0x02]      ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+22
        MOVW      AX, [HL]           ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+24
        MOVW      AX, #0xA92         ;; 1 cycle
        MOVW      BC, #0x3F86        ;; 1 cycle
          CFI FunCall ?F_SUB
        CALL      N:?F_SUB           ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+20
          CFI FunCall _cos
        CALL      _cos               ;; 3 cycles
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+22
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+24
        MOVW      BC, N:_curr+18     ;; 1 cycle
        MOVW      AX, N:_curr+16     ;; 1 cycle
          CFI FunCall ?F_UL2F
        CALL      N:?F_UL2F          ;; 3 cycles
          CFI FunCall ?F_MUL
        CALL      N:?F_MUL           ;; 3 cycles
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+26
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+28
        MOVW      AX, [SP+0x12]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x10]      ;; 1 cycle
          CFI FunCall ?F_SUB
        CALL      N:?F_SUB           ;; 3 cycles
        ADDW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+20
        XCHW      AX, HL             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3181         vert += curr.Yph.rms*sin(PI_BY_THREE-ang);
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [HL+0x02]      ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+22
        MOVW      AX, [HL]           ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+24
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [HL+0x02]      ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+26
        MOVW      AX, [HL]           ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+28
        MOVW      AX, #0xA92         ;; 1 cycle
        MOVW      BC, #0x3F86        ;; 1 cycle
          CFI FunCall ?F_SUB
        CALL      N:?F_SUB           ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+24
          CFI FunCall _sin
        CALL      _sin               ;; 3 cycles
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+26
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+28
        MOVW      BC, N:_curr+18     ;; 1 cycle
        MOVW      AX, N:_curr+16     ;; 1 cycle
          CFI FunCall ?F_UL2F
        CALL      N:?F_UL2F          ;; 3 cycles
          CFI FunCall ?F_MUL
        CALL      N:?F_MUL           ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+24
          CFI FunCall ?F_ADD
        CALL      N:?F_ADD           ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+20
        XCHW      AX, HL             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        BR        S:??power_filter_359  ;; 3 cycles
        ; ------------------------------------- Block: 91 cycles
// 3182     }
// 3183     else
// 3184     {
// 3185         horz -= curr.Yph.rms*cos(PI_BY_THREE+ang);
??power_filter_358:
        MOVW      AX, #0x3F86        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+22
        MOVW      AX, #0xA92         ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+24
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
          CFI FunCall ?F_ADD
        CALL      N:?F_ADD           ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+20
          CFI FunCall _cos
        CALL      _cos               ;; 3 cycles
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+22
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+24
        MOVW      BC, N:_curr+18     ;; 1 cycle
        MOVW      AX, N:_curr+16     ;; 1 cycle
          CFI FunCall ?F_UL2F
        CALL      N:?F_UL2F          ;; 3 cycles
          CFI FunCall ?F_MUL
        CALL      N:?F_MUL           ;; 3 cycles
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+26
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+28
        MOVW      AX, [SP+0x12]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x10]      ;; 1 cycle
          CFI FunCall ?F_SUB
        CALL      N:?F_SUB           ;; 3 cycles
        ADDW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+20
        XCHW      AX, HL             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3186         vert += curr.Yph.rms*sin(PI_BY_THREE+ang);
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [HL+0x02]      ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+22
        MOVW      AX, [HL]           ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+24
        MOVW      AX, #0x3F86        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+26
        MOVW      AX, #0xA92         ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+28
        MOVW      AX, [SP+0x0A]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x08]      ;; 1 cycle
          CFI FunCall ?F_ADD
        CALL      N:?F_ADD           ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+24
          CFI FunCall _sin
        CALL      _sin               ;; 3 cycles
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+26
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+28
        MOVW      BC, N:_curr+18     ;; 1 cycle
        MOVW      AX, N:_curr+16     ;; 1 cycle
          CFI FunCall ?F_UL2F
        CALL      N:?F_UL2F          ;; 3 cycles
          CFI FunCall ?F_MUL
        CALL      N:?F_MUL           ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+24
          CFI FunCall ?F_ADD
        CALL      N:?F_ADD           ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+20
        XCHW      AX, HL             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ; ------------------------------------- Block: 85 cycles
// 3187     }
// 3188     
// 3189     ang = (float)(pf.Bph);
??power_filter_359:
        MOVW      AX, N:_pf+4        ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
          CFI FunCall ?F_UL2F
        CALL      N:?F_UL2F          ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+16
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+18
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+20
// 3190     ang = acos(ang/1000);
        MOVW      AX, #0x447A        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+22
        MOVW      AX, #0x0           ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+24
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
          CFI FunCall ?F_DIV
        CALL      N:?F_DIV           ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+20
          CFI FunCall _acos
        CALL      _acos              ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+16
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+18
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+20
// 3191     
// 3192     if(flag_Bph_active == EXPORT)
        MOVW      HL, #LWRD(_flag_quadrant)  ;; 1 cycle
        MOV1      CY, [HL].4         ;; 1 cycle
        BNC       ??power_filter_360  ;; 4 cycles
        ; ------------------------------------- Block: 31 cycles
// 3193     {
// 3194         ang += PI;
        MOVW      AX, #0x4049        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+22
        MOVW      AX, #0xFDB         ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+24
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
          CFI FunCall ?F_ADD
        CALL      N:?F_ADD           ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+20
        MOVW      HL, SP             ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ; ------------------------------------- Block: 16 cycles
// 3195     }
// 3196     
// 3197     if(quadrant.Bph == Q2 || quadrant.Bph == Q4)
??power_filter_360:
        CMP       N:_quadrant+2, #0x2  ;; 1 cycle
        BZ        ??power_filter_361  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        CMP       N:_quadrant+2, #0x4  ;; 1 cycle
        BNZ       ??power_filter_362  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 3198     {
// 3199         horz -= curr.Bph.rms*cos(PI_BY_THREE+ang);
??power_filter_361:
        MOVW      AX, #0x3F86        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+22
        MOVW      AX, #0xA92         ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+24
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
          CFI FunCall ?F_ADD
        CALL      N:?F_ADD           ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+20
          CFI FunCall _cos
        CALL      _cos               ;; 3 cycles
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+22
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+24
        MOVW      BC, N:_curr+34     ;; 1 cycle
        MOVW      AX, N:_curr+32     ;; 1 cycle
          CFI FunCall ?F_UL2F
        CALL      N:?F_UL2F          ;; 3 cycles
          CFI FunCall ?F_MUL
        CALL      N:?F_MUL           ;; 3 cycles
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+26
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+28
        MOVW      AX, [SP+0x12]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x10]      ;; 1 cycle
          CFI FunCall ?F_SUB
        CALL      N:?F_SUB           ;; 3 cycles
        ADDW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+20
        XCHW      AX, HL             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3200         vert -= curr.Bph.rms*sin(PI_BY_THREE+ang);
        MOVW      AX, #0x3F86        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+22
        MOVW      AX, #0xA92         ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+24
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
          CFI FunCall ?F_ADD
        CALL      N:?F_ADD           ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+20
          CFI FunCall _sin
        CALL      _sin               ;; 3 cycles
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+22
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+24
        MOVW      BC, N:_curr+34     ;; 1 cycle
        MOVW      AX, N:_curr+32     ;; 1 cycle
          CFI FunCall ?F_UL2F
        CALL      N:?F_UL2F          ;; 3 cycles
          CFI FunCall ?F_MUL
        CALL      N:?F_MUL           ;; 3 cycles
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+26
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+28
        MOVW      AX, [SP+0x0E]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x0C]      ;; 1 cycle
          CFI FunCall ?F_SUB
        CALL      N:?F_SUB           ;; 3 cycles
        ADDW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+20
        XCHW      AX, HL             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        BR        S:??power_filter_363  ;; 3 cycles
        ; ------------------------------------- Block: 85 cycles
// 3201     }
// 3202     else
// 3203     {
// 3204         horz -= curr.Bph.rms*cos(PI_BY_THREE-ang);
??power_filter_362:
        MOVW      AX, SP             ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [HL+0x02]      ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+22
        MOVW      AX, [HL]           ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+24
        MOVW      AX, #0xA92         ;; 1 cycle
        MOVW      BC, #0x3F86        ;; 1 cycle
          CFI FunCall ?F_SUB
        CALL      N:?F_SUB           ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+20
          CFI FunCall _cos
        CALL      _cos               ;; 3 cycles
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+22
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+24
        MOVW      BC, N:_curr+34     ;; 1 cycle
        MOVW      AX, N:_curr+32     ;; 1 cycle
          CFI FunCall ?F_UL2F
        CALL      N:?F_UL2F          ;; 3 cycles
          CFI FunCall ?F_MUL
        CALL      N:?F_MUL           ;; 3 cycles
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+26
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+28
        MOVW      AX, [SP+0x12]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x10]      ;; 1 cycle
          CFI FunCall ?F_SUB
        CALL      N:?F_SUB           ;; 3 cycles
        ADDW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+20
        XCHW      AX, HL             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3205         vert -= curr.Bph.rms*sin(PI_BY_THREE-ang);
        MOVW      AX, SP             ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [HL+0x02]      ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+22
        MOVW      AX, [HL]           ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+24
        MOVW      AX, #0xA92         ;; 1 cycle
        MOVW      BC, #0x3F86        ;; 1 cycle
          CFI FunCall ?F_SUB
        CALL      N:?F_SUB           ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+20
          CFI FunCall _sin
        CALL      _sin               ;; 3 cycles
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+22
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+24
        MOVW      BC, N:_curr+34     ;; 1 cycle
        MOVW      AX, N:_curr+32     ;; 1 cycle
          CFI FunCall ?F_UL2F
        CALL      N:?F_UL2F          ;; 3 cycles
          CFI FunCall ?F_MUL
        CALL      N:?F_MUL           ;; 3 cycles
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+26
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+28
        MOVW      AX, [SP+0x0E]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x0C]      ;; 1 cycle
          CFI FunCall ?F_SUB
        CALL      N:?F_SUB           ;; 3 cycles
        ADDW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+20
        XCHW      AX, HL             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
        XCHW      AX, HL             ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ; ------------------------------------- Block: 84 cycles
// 3206     }
// 3207     
// 3208     net_curr = (us32)sqrt((double)horz*horz + (double)vert*vert);
??power_filter_363:
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [HL+0x02]      ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+22
        MOVW      AX, [HL]           ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+24
        MOVW      AX, [SP+0x0A]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x08]      ;; 1 cycle
          CFI FunCall ?F_MUL
        CALL      N:?F_MUL           ;; 3 cycles
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+26
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+28
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [HL+0x02]      ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+30
        MOVW      AX, [HL]           ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+32
        MOVW      AX, [SP+0x16]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x14]      ;; 1 cycle
          CFI FunCall ?F_MUL
        CALL      N:?F_MUL           ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+28
          CFI FunCall ?F_ADD
        CALL      N:?F_ADD           ;; 3 cycles
        ADDW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+20
          CFI FunCall _sqrt
        CALL      _sqrt              ;; 3 cycles
          CFI FunCall ?F_F2UL
        CALL      N:?F_F2UL          ;; 3 cycles
// 3209     return(net_curr);
        ADDW      SP, #0x10          ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock27
        ; ------------------------------------- Block: 46 cycles
        ; ------------------------------------- Total: 676 cycles
// 3210 }
// 3211 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock28 Using cfiCommon0
          CFI Function _metrology_ram_init
        CODE
// 3212 void metrology_ram_init()
// 3213 {
_metrology_ram_init:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 10
        SUBW      SP, #0xA           ;; 1 cycle
          CFI CFA SP+14
// 3214   us32 long1,long_int;
// 3215   us8 main2,read_default_energy_value_f=0;
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP+0x01], A       ;; 1 cycle
// 3216   e2416_byte = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_e2416_byte, AX  ;; 1 cycle
// 3217   e2416_q2_byte=0;
        MOV       N:_e2416_q2_byte, #0x0  ;; 1 cycle
// 3218   
// 3219     eprom_read(0x6320,0,PAGE_1,AUTO_CALC);
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x6320        ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
// 3220     alternate_energy_counter1=opr_data[0];
        MOV       A, N:_opr_data     ;; 1 cycle
        MOV       N:_alternate_energy_counter1, A  ;; 1 cycle
// 3221     alternate_energy_counter2=opr_data[1];
        MOV       A, N:_opr_data+1   ;; 1 cycle
        MOV       N:_alternate_energy_counter2, A  ;; 1 cycle
// 3222   
// 3223     /* Energy */
// 3224  
// 3225    long1 =0;   
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x6           ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
// 3226    for(main2= 0; main2 < 16; main2++)
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        ; ------------------------------------- Block: 26 cycles
??metrology_ram_init_0:
        MOV       A, [SP]            ;; 1 cycle
        CMP       A, #0x10           ;; 1 cycle
        BNC       ??power_filter_364  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 3227    {
// 3228         eprom_read((ALL_ENERGY_SAVE_START_ADD+e2416_byte),0,PAGE_7,AUTO_CALC);
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x6            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, N:_e2416_byte  ;; 1 cycle
        ADDW      AX, #0x6500        ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
// 3229         long_int=char_array_to_long4(&opr_data[8]); //reading apparent energy
        MOVW      AX, #LWRD(_opr_data+8)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      [SP+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [SP+0x04], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3230         
// 3231         if(long1 >= long_int)
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x2           ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [SP+0x08]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        CMP       A, [HL+0x03]       ;; 1 cycle
        BNZ       ??power_filter_365  ;; 4 cycles
        ; ------------------------------------- Block: 28 cycles
        MOV       A, C               ;; 1 cycle
        CMP       A, [HL+0x02]       ;; 1 cycle
        BNZ       ??power_filter_365  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOV       A, B               ;; 1 cycle
        CMP       A, [HL+0x01]       ;; 1 cycle
        BNZ       ??power_filter_365  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOV       A, X               ;; 1 cycle
        CMP       A, [HL]            ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
??power_filter_365:
        BNC       ??power_filter_364  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
// 3232         {
// 3233             break;
// 3234         }
// 3235         else
// 3236         {
// 3237             long1= long_int;
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      [SP+0x06], AX      ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      [SP+0x08], AX      ;; 1 cycle
// 3238         }
// 3239         e2416_byte= e2416_byte + ALL_ENERGY_SAVE_BLOCK_SIZE;
        MOVW      AX, N:_e2416_byte  ;; 1 cycle
        ADDW      AX, #0x70          ;; 1 cycle
        MOVW      N:_e2416_byte, AX  ;; 1 cycle
// 3240         if(e2416_byte>=(ALL_ENERGY_SAVE_BLOCK_SIZE*0x10))
        MOVW      AX, N:_e2416_byte  ;; 1 cycle
        CMPW      AX, #0x700         ;; 1 cycle
        BC        ??power_filter_366  ;; 4 cycles
        ; ------------------------------------- Block: 13 cycles
// 3241         {
// 3242           e2416_byte=0x00;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_e2416_byte, AX  ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
// 3243         }
// 3244     }
??power_filter_366:
        MOV       A, [SP]            ;; 1 cycle
        INC       A                  ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        BR        S:??metrology_ram_init_0  ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
// 3245    if(main2== 16 && e2416_byte==0)
??power_filter_364:
        MOV       A, [SP]            ;; 1 cycle
        CMP       A, #0x10           ;; 1 cycle
        BNZ       ??power_filter_367  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        CLRW      AX                 ;; 1 cycle
        CMPW      AX, N:_e2416_byte  ;; 1 cycle
        BNZ       ??power_filter_367  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 3246    {
// 3247      e2416_byte=(ALL_ENERGY_SAVE_BLOCK_SIZE*0x10);
        MOVW      AX, #0x700         ;; 1 cycle
        MOVW      N:_e2416_byte, AX  ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
// 3248    }
// 3249    e2416_byte= e2416_byte - ALL_ENERGY_SAVE_BLOCK_SIZE;
??power_filter_367:
        MOVW      AX, N:_e2416_byte  ;; 1 cycle
        ADDW      AX, #0xFF90        ;; 1 cycle
        MOVW      N:_e2416_byte, AX  ;; 1 cycle
// 3250    /////////////////////
// 3251    
// 3252    if(eprom_read((ALL_ENERGY_SAVE_START_ADD+e2416_byte),0,PAGE_7,AUTO_CALC) == EEP_ERROR)//reading from main energy location
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x6            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, N:_e2416_byte  ;; 1 cycle
        ADDW      AX, #0x6500        ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_368  ;; 4 cycles
        ; ------------------------------------- Block: 16 cycles
// 3253    {
// 3254      if(eprom_read(ALTERNATE_ALL_ENERGY_SAVE_START_ADD,2,PAGE_7,AUTO_CALC) == EEP_ERROR)//alternate location
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x6            ;; 1 cycle
        MOV       C, #0x2            ;; 1 cycle
        MOVW      AX, #0xFF00        ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_369  ;; 4 cycles
        ; ------------------------------------- Block: 12 cycles
// 3255      {
// 3256        read_default_energy_value_f=1;
        MOV       A, #0x1            ;; 1 cycle
        MOV       [SP+0x01], A       ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
// 3257      }
// 3258      alternate_energy_counter1++;
??power_filter_369:
        INC       N:_alternate_energy_counter1  ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
// 3259    }
// 3260    
// 3261    if(read_default_energy_value_f!=1)
??power_filter_368:
        MOV       A, [SP+0x01]       ;; 1 cycle
        CMP       A, #0x1            ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??power_filter_370  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 3262    {
// 3263         energy.Allph.active_imp = char_array_to_long4(&opr_data[0]);
        MOVW      AX, #LWRD(_opr_data)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_energy+52, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+54, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3264         energy.Allph.active_exp = char_array_to_long4(&opr_data[4]);
        MOVW      AX, #LWRD(_opr_data+4)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_energy+56, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+58, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3265         energy.Allph.apparent_imp = char_array_to_long4(&opr_data[8]);
        MOVW      AX, #LWRD(_opr_data+8)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_energy+68, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+70, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3266         energy.Allph.apparent_exp = char_array_to_long4(&opr_data[12]);
        MOVW      AX, #LWRD(_opr_data+12)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_energy+72, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+74, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3267 
// 3268         energy.Allph.zkwh_imp = char_array_to_long4(&opr_data[16]);
        MOVW      AX, #LWRD(_opr_data+16)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_energy+92, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+94, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3269         energy.Allph.zkwh_exp = char_array_to_long4(&opr_data[20]);
        MOVW      AX, #LWRD(_opr_data+20)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_energy+96, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+98, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3270 
// 3271         energy.Allph.zkvah_imp = char_array_to_long4(&opr_data[24]);
        MOVW      AX, #LWRD(_opr_data+24)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_energy+100, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+102, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3272         energy.Allph.zkvah_exp = char_array_to_long4(&opr_data[28]);
        MOVW      AX, #LWRD(_opr_data+28)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_energy+104, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+106, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3273 
// 3274         energy.Allph.reactive_q1 = char_array_to_long4(&opr_data[32]);
        MOVW      AX, #LWRD(_opr_data+32)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_energy+76, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+78, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3275         energy.Allph.reactive_q3 = char_array_to_long4(&opr_data[36]);
        MOVW      AX, #LWRD(_opr_data+36)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_energy+84, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+86, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3276         energy.Allph.reactive_q2 = char_array_to_long4(&opr_data[40]);
        MOVW      AX, #LWRD(_opr_data+40)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_energy+80, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+82, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3277 
// 3278         energy.Allph.zkvarh_q1 = char_array_to_long4(&opr_data[44]);
        MOVW      AX, #LWRD(_opr_data+44)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_energy+108, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+110, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3279         energy.Allph.zkvarh_q3 = char_array_to_long4(&opr_data[48]);
        MOVW      AX, #LWRD(_opr_data+48)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_energy+116, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+118, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3280         energy.Allph.zkvarh_q2 = char_array_to_long4(&opr_data[52]);
        MOVW      AX, #LWRD(_opr_data+52)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_energy+112, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+114, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3281 
// 3282         energy.Allph.defraud_mag = char_array_to_long4(&opr_data[56]);
        MOVW      AX, #LWRD(_opr_data+56)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_energy+60, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+62, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3283         energy.Allph.fundamental = char_array_to_long4(&opr_data[60]);
        MOVW      AX, #LWRD(_opr_data+60)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_energy+64, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+66, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3284 
// 3285         energy.Rph.active_imp = char_array_to_long4(&opr_data[64]);
        MOVW      AX, #LWRD(_opr_data+64)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_energy+2, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+4, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3286         energy.Rph.active_exp = char_array_to_long4(&opr_data[68]);
        MOVW      AX, #LWRD(_opr_data+68)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_energy+6, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+8, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3287         energy.Yph.active_imp = char_array_to_long4(&opr_data[72]);
        MOVW      AX, #LWRD(_opr_data+72)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_energy+16, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+18, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3288         energy.Yph.active_exp = char_array_to_long4(&opr_data[76]);
        MOVW      AX, #LWRD(_opr_data+76)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_energy+20, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+22, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3289         energy.Bph.active_imp = char_array_to_long4(&opr_data[80]);
        MOVW      AX, #LWRD(_opr_data+80)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_energy+30, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+32, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3290         energy.Bph.active_exp = char_array_to_long4(&opr_data[84]);
        MOVW      AX, #LWRD(_opr_data+84)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_energy+34, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+36, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3291 
// 3292         energy.Rph.defraud_mag = char_array_to_long4(&opr_data[88]);
        MOVW      AX, #LWRD(_opr_data+88)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_energy+10, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+12, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3293         energy.Yph.defraud_mag = char_array_to_long4(&opr_data[92]);
        MOVW      AX, #LWRD(_opr_data+92)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_energy+24, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+26, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3294         energy.Bph.defraud_mag = char_array_to_long4(&opr_data[96]);
        MOVW      AX, #LWRD(_opr_data+96)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_energy+38, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+40, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3295         zone_pf = char_array_to_int(&opr_data[100]);
        MOVW      AX, #LWRD(_opr_data+100)  ;; 1 cycle
          CFI FunCall _char_array_to_int
        CALL      _char_array_to_int  ;; 3 cycles
        MOVW      N:_zone_pf, AX     ;; 1 cycle
        ; ------------------------------------- Block: 205 cycles
// 3296    }
// 3297    //this assignment is done out side of if() condition to take care checksum mismatch case
// 3298    duplicate_total_apparent_energy=energy.Allph.apparent_imp;
??power_filter_370:
        MOVW      BC, N:_energy+70   ;; 1 cycle
        MOVW      AX, N:_energy+68   ;; 1 cycle
        MOVW      N:_duplicate_total_apparent_energy, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_duplicate_total_apparent_energy+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3299    
// 3300    //this is for only lead energy
// 3301    long1 = 0;
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x6           ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
// 3302    read_default_energy_value_f=0;
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP+0x01], A       ;; 1 cycle
// 3303    
// 3304    for(main2= 0; main2 < 16; main2++)
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        ; ------------------------------------- Block: 17 cycles
??metrology_ram_init_1:
        MOV       A, [SP]            ;; 1 cycle
        CMP       A, #0x10           ;; 1 cycle
        BNC       ??power_filter_371  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 3305     {
// 3306         eprom_read((LEAD_ENERGY_SAVE_START_ADD+e2416_q2_byte),0,PAGE_1,AUTO_CALC);
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOV       X, N:_e2416_q2_byte  ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        ADDW      AX, #0x6C00        ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
// 3307         long_int=char_array_to_long4(&opr_data[0]);
        MOVW      AX, #LWRD(_opr_data)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      [SP+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [SP+0x04], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3308         if(long1 >= long_int)
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x2           ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [SP+0x08]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        CMP       A, [HL+0x03]       ;; 1 cycle
        BNZ       ??power_filter_372  ;; 4 cycles
        ; ------------------------------------- Block: 29 cycles
        MOV       A, C               ;; 1 cycle
        CMP       A, [HL+0x02]       ;; 1 cycle
        BNZ       ??power_filter_372  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOV       A, B               ;; 1 cycle
        CMP       A, [HL+0x01]       ;; 1 cycle
        BNZ       ??power_filter_372  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOV       A, X               ;; 1 cycle
        CMP       A, [HL]            ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
??power_filter_372:
        BNC       ??power_filter_371  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
// 3309         {
// 3310             break;
// 3311         }
// 3312         else
// 3313         {
// 3314             long1= long_int;
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      [SP+0x06], AX      ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      [SP+0x08], AX      ;; 1 cycle
// 3315         }
// 3316         e2416_q2_byte= e2416_q2_byte + LEAD_ENERGY_SAVE_BLOCK_SIZE;
        MOVW      HL, #LWRD(_e2416_q2_byte)  ;; 1 cycle
        MOV       A, [HL]            ;; 1 cycle
        ADD       A, #0x10           ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
// 3317     }
        MOV       A, [SP]            ;; 1 cycle
        INC       A                  ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        BR        S:??metrology_ram_init_1  ;; 3 cycles
        ; ------------------------------------- Block: 14 cycles
// 3318    
// 3319    e2416_q2_byte= e2416_q2_byte - LEAD_ENERGY_SAVE_BLOCK_SIZE;
??power_filter_371:
        MOVW      HL, #LWRD(_e2416_q2_byte)  ;; 1 cycle
        MOV       A, [HL]            ;; 1 cycle
        ADD       A, #0xF0           ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
// 3320  ///////////////////////
// 3321    
// 3322    if(eprom_read((LEAD_ENERGY_SAVE_START_ADD+e2416_q2_byte),0,PAGE_1,AUTO_CALC) == EEP_ERROR)//main location of lead energy
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOV       X, N:_e2416_q2_byte  ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        ADDW      AX, #0x6C00        ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_373  ;; 4 cycles
        ; ------------------------------------- Block: 18 cycles
// 3323    {
// 3324      if(eprom_read(ALTERNATE_LEAD_ENERGY_SAVE_START_ADD,2,PAGE_1,AUTO_CALC)== EEP_ERROR)//alternate location of lead energy
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x2            ;; 1 cycle
        MOVW      AX, #0xFF70        ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_374  ;; 4 cycles
        ; ------------------------------------- Block: 12 cycles
// 3325      {
// 3326        read_default_energy_value_f=1;
        MOV       A, #0x1            ;; 1 cycle
        MOV       [SP+0x01], A       ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
// 3327      }
// 3328      alternate_energy_counter2++;
??power_filter_374:
        INC       N:_alternate_energy_counter2  ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
// 3329    }
// 3330    if(read_default_energy_value_f !=1)
??power_filter_373:
        MOV       A, [SP+0x01]       ;; 1 cycle
        CMP       A, #0x1            ;; 1 cycle
        BZ        ??power_filter_375  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 3331    {
// 3332      energy.Allph.reactive_q4=char_array_to_long4(&opr_data[0]);
        MOVW      AX, #LWRD(_opr_data)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_energy+88, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+90, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3333      energy.Allph.zkvarh_q4=char_array_to_long4(&opr_data[4]);
        MOVW      AX, #LWRD(_opr_data+4)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        MOVW      N:_energy+120, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+122, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ; ------------------------------------- Block: 16 cycles
// 3334    }
// 3335    
// 3336    
// 3337     e2416_byte+= ALL_ENERGY_SAVE_BLOCK_SIZE;
??power_filter_375:
        MOVW      AX, N:_e2416_byte  ;; 1 cycle
        ADDW      AX, #0x70          ;; 1 cycle
        MOVW      N:_e2416_byte, AX  ;; 1 cycle
// 3338     e2416_q2_byte+= LEAD_ENERGY_SAVE_BLOCK_SIZE;
        MOVW      HL, #LWRD(_e2416_q2_byte)  ;; 1 cycle
        MOV       A, [HL]            ;; 1 cycle
        ADD       A, #0x10           ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
// 3339     
// 3340     if(e2416_byte==(ALL_ENERGY_SAVE_BLOCK_SIZE*0x10))
        MOVW      AX, N:_e2416_byte  ;; 1 cycle
        CMPW      AX, #0x700         ;; 1 cycle
        BNZ       ??power_filter_376  ;; 4 cycles
        ; ------------------------------------- Block: 13 cycles
// 3341     {
// 3342       e2416_byte=0x00;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_e2416_byte, AX  ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
// 3343     }
// 3344 
// 3345     
// 3346     /* Energy pulses */
// 3347     if(eprom_read(0x0CF0,0,PAGE_1,AUTO_CALC) == EEP_OK)
??power_filter_376:
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xCF0         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
        CMP       A, #0x1            ;; 1 cycle
        SKZ                          ;; 4 cycles
        BR        N:??power_filter_377  ;; 4 cycles
        ; ------------------------------------- Block: 12 cycles
// 3348     {
// 3349         energy.Allph.active_imp_pulse = opr_data[0];
        MOV       A, N:_opr_data     ;; 1 cycle
        MOV       N:_energy+42, A    ;; 1 cycle
// 3350         if(energy.Allph.active_imp_pulse > 10)
        MOV       A, N:_energy+42    ;; 1 cycle
        CMP       A, #0xB            ;; 1 cycle
        SKC                          ;; 1 cycle
        ; ------------------------------------- Block: 5 cycles
// 3351         {
// 3352           energy.Allph.active_imp_pulse=0;
        MOV       N:_energy+42, #0x0  ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
// 3353         }
// 3354         
// 3355         energy.Allph.active_exp_pulse = opr_data[1];
??metrology_ram_init_2:
        MOV       A, N:_opr_data+1   ;; 1 cycle
        MOV       N:_energy+43, A    ;; 1 cycle
// 3356         if(energy.Allph.active_exp_pulse > 10)
        MOV       A, N:_energy+43    ;; 1 cycle
        CMP       A, #0xB            ;; 1 cycle
        SKC                          ;; 1 cycle
        ; ------------------------------------- Block: 5 cycles
// 3357         {
// 3358           energy.Allph.active_exp_pulse=0;
        MOV       N:_energy+43, #0x0  ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
// 3359         }
// 3360         
// 3361         energy.Allph.reactive_q1_pulse = opr_data[2];
??metrology_ram_init_3:
        MOV       A, N:_opr_data+2   ;; 1 cycle
        MOV       N:_energy+47, A    ;; 1 cycle
// 3362         if(energy.Allph.reactive_q1_pulse > 10)
        MOV       A, N:_energy+47    ;; 1 cycle
        CMP       A, #0xB            ;; 1 cycle
        SKC                          ;; 1 cycle
        ; ------------------------------------- Block: 5 cycles
// 3363         {
// 3364           energy.Allph.reactive_q1_pulse=0;
        MOV       N:_energy+47, #0x0  ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
// 3365         }
// 3366         
// 3367         energy.Allph.reactive_q2_pulse = opr_data[3];
??metrology_ram_init_4:
        MOV       A, N:_opr_data+3   ;; 1 cycle
        MOV       N:_energy+48, A    ;; 1 cycle
// 3368         if(energy.Allph.reactive_q2_pulse > 10)
        MOV       A, N:_energy+48    ;; 1 cycle
        CMP       A, #0xB            ;; 1 cycle
        SKC                          ;; 1 cycle
        ; ------------------------------------- Block: 5 cycles
// 3369         {
// 3370           energy.Allph.reactive_q2_pulse=0;
        MOV       N:_energy+48, #0x0  ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
// 3371         }
// 3372         
// 3373         energy.Allph.reactive_q3_pulse = opr_data[4];
??metrology_ram_init_5:
        MOV       A, N:_opr_data+4   ;; 1 cycle
        MOV       N:_energy+49, A    ;; 1 cycle
// 3374         if(energy.Allph.reactive_q3_pulse > 10)
        MOV       A, N:_energy+49    ;; 1 cycle
        CMP       A, #0xB            ;; 1 cycle
        SKC                          ;; 1 cycle
        ; ------------------------------------- Block: 5 cycles
// 3375         {
// 3376           energy.Allph.reactive_q3_pulse=0;
        MOV       N:_energy+49, #0x0  ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
// 3377         }
// 3378         
// 3379         energy.Allph.reactive_q4_pulse = opr_data[5];
??metrology_ram_init_6:
        MOV       A, N:_opr_data+5   ;; 1 cycle
        MOV       N:_energy+50, A    ;; 1 cycle
// 3380         if(energy.Allph.reactive_q4_pulse > 10)
        MOV       A, N:_energy+50    ;; 1 cycle
        CMP       A, #0xB            ;; 1 cycle
        SKC                          ;; 1 cycle
        ; ------------------------------------- Block: 5 cycles
// 3381         {
// 3382           energy.Allph.reactive_q4_pulse=0;
        MOV       N:_energy+50, #0x0  ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
// 3383         }
// 3384         
// 3385         energy.Allph.apparent_imp_pulse = opr_data[6];
??metrology_ram_init_7:
        MOV       A, N:_opr_data+6   ;; 1 cycle
        MOV       N:_energy+45, A    ;; 1 cycle
// 3386         if(energy.Allph.apparent_imp_pulse > 10)
        MOV       A, N:_energy+45    ;; 1 cycle
        CMP       A, #0xB            ;; 1 cycle
        SKC                          ;; 1 cycle
        ; ------------------------------------- Block: 5 cycles
// 3387         {
// 3388           energy.Allph.apparent_imp_pulse=0;
        MOV       N:_energy+45, #0x0  ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
// 3389         }
// 3390         
// 3391         energy.Allph.apparent_exp_pulse = opr_data[7];
??metrology_ram_init_8:
        MOV       A, N:_opr_data+7   ;; 1 cycle
        MOV       N:_energy+46, A    ;; 1 cycle
// 3392         if(energy.Allph.apparent_exp_pulse > 10)
        MOV       A, N:_energy+46    ;; 1 cycle
        CMP       A, #0xB            ;; 1 cycle
        SKC                          ;; 1 cycle
        ; ------------------------------------- Block: 5 cycles
// 3393         {
// 3394           energy.Allph.apparent_exp_pulse=0;
        MOV       N:_energy+46, #0x0  ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
// 3395         }
// 3396         
// 3397         energy.Rph.active_imp_pulse = opr_data[8];
??metrology_ram_init_9:
        MOV       A, N:_opr_data+8   ;; 1 cycle
        MOV       N:_energy, A       ;; 1 cycle
// 3398         if(energy.Rph.active_imp_pulse > 10)
        MOV       A, N:_energy       ;; 1 cycle
        CMP       A, #0xB            ;; 1 cycle
        SKC                          ;; 1 cycle
        ; ------------------------------------- Block: 5 cycles
// 3399         {
// 3400           energy.Rph.active_imp_pulse=0;
        MOV       N:_energy, #0x0    ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
// 3401         }
// 3402         
// 3403         energy.Rph.active_exp_pulse = opr_data[9];
??metrology_ram_init_10:
        MOV       A, N:_opr_data+9   ;; 1 cycle
        MOV       N:_energy+1, A     ;; 1 cycle
// 3404         if(energy.Rph.active_exp_pulse > 10)
        MOV       A, N:_energy+1     ;; 1 cycle
        CMP       A, #0xB            ;; 1 cycle
        SKC                          ;; 1 cycle
        ; ------------------------------------- Block: 5 cycles
// 3405         {
// 3406           energy.Rph.active_exp_pulse=0;
        MOV       N:_energy+1, #0x0  ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
// 3407         }
// 3408         
// 3409         energy.Yph.active_imp_pulse = opr_data[10];
??metrology_ram_init_11:
        MOV       A, N:_opr_data+10  ;; 1 cycle
        MOV       N:_energy+14, A    ;; 1 cycle
// 3410         if(energy.Yph.active_imp_pulse > 10)
        MOV       A, N:_energy+14    ;; 1 cycle
        CMP       A, #0xB            ;; 1 cycle
        SKC                          ;; 1 cycle
        ; ------------------------------------- Block: 5 cycles
// 3411         {
// 3412           energy.Yph.active_imp_pulse=0;
        MOV       N:_energy+14, #0x0  ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
// 3413         }
// 3414         
// 3415         energy.Yph.active_exp_pulse = opr_data[11];
??metrology_ram_init_12:
        MOV       A, N:_opr_data+11  ;; 1 cycle
        MOV       N:_energy+15, A    ;; 1 cycle
// 3416         if(energy.Yph.active_exp_pulse > 10)
        MOV       A, N:_energy+15    ;; 1 cycle
        CMP       A, #0xB            ;; 1 cycle
        SKC                          ;; 1 cycle
        ; ------------------------------------- Block: 5 cycles
// 3417         {
// 3418           energy.Yph.active_exp_pulse=0;
        MOV       N:_energy+15, #0x0  ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
// 3419         }
// 3420         energy.Bph.active_imp_pulse = opr_data[12];
??metrology_ram_init_13:
        MOV       A, N:_opr_data+12  ;; 1 cycle
        MOV       N:_energy+28, A    ;; 1 cycle
// 3421         if(energy.Bph.active_imp_pulse > 10)
        MOV       A, N:_energy+28    ;; 1 cycle
        CMP       A, #0xB            ;; 1 cycle
        SKC                          ;; 1 cycle
        ; ------------------------------------- Block: 5 cycles
// 3422         {
// 3423           energy.Bph.active_imp_pulse=0;
        MOV       N:_energy+28, #0x0  ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
// 3424         }
// 3425         energy.Bph.active_exp_pulse = opr_data[13];
??metrology_ram_init_14:
        MOV       A, N:_opr_data+13  ;; 1 cycle
        MOV       N:_energy+29, A    ;; 1 cycle
// 3426         if(energy.Bph.active_exp_pulse > 10)
        MOV       A, N:_energy+29    ;; 1 cycle
        CMP       A, #0xB            ;; 1 cycle
        SKC                          ;; 1 cycle
        ; ------------------------------------- Block: 5 cycles
// 3427         {
// 3428           energy.Bph.active_exp_pulse=0;
        MOV       N:_energy+29, #0x0  ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
// 3429         }
// 3430         alt_energy_save_cntr = opr_data[14];
??metrology_ram_init_15:
        MOV       A, N:_opr_data+14  ;; 1 cycle
        MOV       N:_alt_energy_save_cntr, A  ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
// 3431     }
// 3432     if(eprom_read(0x0CD0,0,PAGE_1,AUTO_CALC) == EEP_OK)
??power_filter_377:
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xCD0         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
        CMP       A, #0x1            ;; 1 cycle
        BNZ       ??power_filter_378  ;; 4 cycles
        ; ------------------------------------- Block: 12 cycles
// 3433     {
// 3434         energy.Allph.fundamental_pulse = opr_data[0];
        MOV       A, N:_opr_data     ;; 1 cycle
        MOV       N:_energy+44, A    ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
// 3435     }   
// 3436 
// 3437     
// 3438     if(eprom_read(0x0F20,0,PAGE_1,AUTO_CALC) == EEP_OK)
??power_filter_378:
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xF20         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
        CMP       A, #0x1            ;; 1 cycle
        BNZ       ??power_filter_379  ;; 4 cycles
        ; ------------------------------------- Block: 12 cycles
// 3439     {
// 3440         if(opr_data[0] == 0 || opr_data[1] == 1)
        CMP0      N:_opr_data        ;; 1 cycle
        BZ        ??power_filter_380  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        CMP       N:_opr_data+1, #0x1  ;; 1 cycle
        BNZ       ??power_filter_381  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 3441         {
// 3442           METERING_MODE = opr_data[0];
??power_filter_380:
        MOV       A, N:_opr_data     ;; 1 cycle
        MOV       N:_METERING_MODE, A  ;; 1 cycle
        BR        S:??power_filter_382  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 3443         }
// 3444         else
// 3445         {
// 3446           METERING_MODE = FWD;
??power_filter_381:
        MOV       N:_METERING_MODE, #0x0  ;; 1 cycle
        BR        S:??power_filter_382  ;; 3 cycles
        ; ------------------------------------- Block: 4 cycles
// 3447         }
// 3448     }
// 3449     else
// 3450     {
// 3451       METERING_MODE = FWD;
??power_filter_379:
        MOV       N:_METERING_MODE, #0x0  ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
// 3452     }
// 3453     
// 3454     
// 3455     ACT_THRESHOLD = ACT_THRESHOLD1;
??power_filter_382:
        MOVW      HL, #LWRD(_ACT_THRESHOLD)  ;; 1 cycle
        MOVW      AX, #0xCA70        ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0xF7D         ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x155         ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 3456     REACT_THRESHOLD = REACT_THRESHOLD1;
        MOVW      HL, #LWRD(_REACT_THRESHOLD)  ;; 1 cycle
        MOVW      AX, #0xCA70        ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0xF7D         ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x155         ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 3457     APP_THRESHOLD = APP_THRESHOLD1;
        MOVW      HL, #LWRD(_APP_THRESHOLD)  ;; 1 cycle
        MOVW      AX, #0xCA70        ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0xF7D         ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x155         ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 3458     
// 3459     pwrup_sec_cnt= 1; 
        MOV       N:_pwrup_sec_cnt, #0x1  ;; 1 cycle
// 3460     flag_phase_seq = 1;
        SET1      N:_flag_metro1.6   ;; 2 cycles
// 3461     
// 3462     if(eprom_read(0x07A0,0,PAGE_1,AUTO_CALC) == EEP_OK)
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x7A0         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
        CMP       A, #0x1            ;; 1 cycle
        BNZ       ??power_filter_383  ;; 4 cycles
        ; ------------------------------------- Block: 42 cycles
// 3463     {
// 3464        energy_rollover_f = opr_data[0];
        MOVW      HL, #LWRD(_opr_data)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        MOVW      HL, #LWRD(_flag_metro3)  ;; 1 cycle
        MOV1      [HL].3, CY         ;; 2 cycles
// 3465        energy_rollover_count = opr_data[1];
        MOV       A, N:_opr_data+1   ;; 1 cycle
        MOV       N:_energy_rollover_count, A  ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
// 3466     }
// 3467     
// 3468     eprom_read(0x6320,0,PAGE_1,AUTO_CALC);
??power_filter_383:
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x6320        ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
// 3469     opr_data[0]=alternate_energy_counter1;
        MOV       A, N:_alternate_energy_counter1  ;; 1 cycle
        MOV       N:_opr_data, A     ;; 1 cycle
// 3470     opr_data[1]=alternate_energy_counter2;
        MOV       A, N:_alternate_energy_counter2  ;; 1 cycle
        MOV       N:_opr_data+1, A   ;; 1 cycle
// 3471     eprom_write(0x6320,0,16,PAGE_1,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+16
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x6320        ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
// 3472 }
        ADDW      SP, #0xC           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock28
        ; ------------------------------------- Block: 27 cycles
        ; ------------------------------------- Total: 731 cycles
// 3473 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock29 Using cfiCommon0
          CFI Function _config_parameter_init
        CODE
// 3474 void config_parameter_init()
// 3475 {
_config_parameter_init:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 2
        SUBW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+6
// 3476   uint8_t m1;
// 3477   
// 3478   eprom_read(FG_DATE_AND_TIME,0,PAGE_1,AUTO_CALC);
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xFD0         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
// 3479   for(m1= 0; m1 < 4; m1++)
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        ; ------------------------------------- Block: 10 cycles
??config_parameter_init_0:
        MOV       A, [SP]            ;; 1 cycle
        CMP       A, #0x4            ;; 1 cycle
        BNC       ??power_filter_384  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 3480     {
// 3481         *(utility_id + m1)= opr_data[5+m1];
        MOV       A, [SP]            ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        MOV       A, (_opr_data+5)[B]  ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       (_utility_id)[B], A  ;; 1 cycle
        XCH       A, X               ;; 1 cycle
// 3482     }
        MOV       A, [SP]            ;; 1 cycle
        INC       A                  ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        BR        S:??config_parameter_init_0  ;; 3 cycles
        ; ------------------------------------- Block: 15 cycles
// 3483   
// 3484   if(eprom_read(SERIAL_NO_CONFIG_ADD,0,PAGE_1,AUTO_CALC) == EEP_ERROR) //main location for serial no
??power_filter_384:
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xFE0         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_385  ;; 4 cycles
        ; ------------------------------------- Block: 12 cycles
// 3485   {
// 3486     if(eprom_read(ALTERNATE_SERIAL_NO_CONFIG_ADD,2,PAGE_1,AUTO_CALC) == EEP_ERROR) //alternate location for serial no
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x2            ;; 1 cycle
        MOVW      AX, #0xFE00        ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_386  ;; 4 cycles
        ; ------------------------------------- Block: 12 cycles
// 3487     {
// 3488       invalid_serial_no = 1;
        SET1      N:_flag_system1.0  ;; 2 cycles
        BR        S:??power_filter_387  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 3489     }
// 3490     else
// 3491     {
// 3492       invalid_serial_no = 0;
??power_filter_386:
        CLR1      N:_flag_system1.0  ;; 2 cycles
        BR        S:??power_filter_387  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 3493     }
// 3494   }
// 3495   else
// 3496   {
// 3497     invalid_serial_no=0;
??power_filter_385:
        CLR1      N:_flag_system1.0  ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
// 3498   }
// 3499   
// 3500   if(invalid_serial_no==0)
??power_filter_387:
        MOVW      HL, #LWRD(_flag_system1)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BC        ??power_filter_388  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 3501   {
// 3502       for(m1= 0; m1 < 8; m1++)
        MOV       B, #0x0            ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??config_parameter_init_1:
        MOV       A, B               ;; 1 cycle
        CMP       A, #0x8            ;; 1 cycle
        BNC       ??power_filter_389  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 3503       {
// 3504           *(serial_no + (m1+5))= opr_data[m1];
        MOV       A, (_opr_data)[B]  ;; 1 cycle
        MOV       (_serial_no+5)[B], A  ;; 1 cycle
// 3505       }
        INC       B                  ;; 1 cycle
        BR        S:??config_parameter_init_1  ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
// 3506 
// 3507       for(m1= 0; m1 < 5; m1++)
??power_filter_389:
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
??config_parameter_init_2:
        MOV       A, [SP]            ;; 1 cycle
        CMP       A, #0x5            ;; 1 cycle
        BNC       ??power_filter_388  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 3508       {
// 3509           *(serial_no + m1)= opr_data[10+m1];
        MOV       A, [SP]            ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        MOV       A, (_opr_data+10)[B]  ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       (_serial_no)[B], A  ;; 1 cycle
        XCH       A, X               ;; 1 cycle
// 3510       }
        MOV       A, [SP]            ;; 1 cycle
        INC       A                  ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        BR        S:??config_parameter_init_2  ;; 3 cycles
        ; ------------------------------------- Block: 15 cycles
// 3511   }
// 3512   
// 3513 
// 3514   if(eprom_read(SET_PAR_ADDRESS,0,PAGE_1,AUTO_CALC)==EEP_ERROR)
??power_filter_388:
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xFF0         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_390  ;; 4 cycles
        ; ------------------------------------- Block: 12 cycles
// 3515   {
// 3516     eprom_read(ALTERNATE_SET_PAR_ADDRESS,2,PAGE_1,AUTO_CALC);
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x2            ;; 1 cycle
        MOVW      AX, #0xFE10        ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
// 3517     eprom_write(SET_PAR_ADDRESS,0,16,PAGE_1,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+8
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xFF0         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+6
        ; ------------------------------------- Block: 17 cycles
// 3518   }
// 3519   CTR= (uint16_t)opr_data[0] * 256 + opr_data[1];
??power_filter_390:
        MOV       X, N:_opr_data     ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x100         ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOV       C, N:_opr_data+1   ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        ADDW      AX, BC             ;; 1 cycle
        MOVW      N:_CTR, AX         ;; 1 cycle
// 3520   PTR= (uint16_t)opr_data[2] * 256 + opr_data[3];
        MOV       X, N:_opr_data+2   ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x100         ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOV       C, N:_opr_data+3   ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        ADDW      AX, BC             ;; 1 cycle
        MOVW      N:_PTR, AX         ;; 1 cycle
// 3521   meter_type[0]= opr_data[4];
        MOV       A, N:_opr_data+4   ;; 1 cycle
        MOV       N:_meter_type, A   ;; 1 cycle
// 3522   meter_type[1]= opr_data[5];
        MOV       A, N:_opr_data+5   ;; 1 cycle
        MOV       N:_meter_type+1, A  ;; 1 cycle
// 3523   
// 3524   for(m1= 0; m1 < 8; m1++)
        MOV       B, #0x0            ;; 1 cycle
        ; ------------------------------------- Block: 23 cycles
??config_parameter_init_3:
        MOV       A, B               ;; 1 cycle
        CMP       A, #0x8            ;; 1 cycle
        BNC       ??power_filter_391  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 3525   {
// 3526       dlms_firm_ver[m1]= opr_data[6 + m1];
        MOV       A, (_opr_data+6)[B]  ;; 1 cycle
        MOV       (_dlms_firm_ver)[B], A  ;; 1 cycle
// 3527   }
        INC       B                  ;; 1 cycle
        BR        S:??config_parameter_init_3  ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
// 3528 }
??power_filter_391:
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock29
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 180 cycles
// 3529 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock30 Using cfiCommon0
          CFI Function _tod_init
        CODE
// 3530 void tod_init()
// 3531 {
_tod_init:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 2
        SUBW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+6
// 3532     uint8_t m2;
// 3533     
// 3534     eprom_read(TOU_ActiveCalPtr,0,PAGE_1,AUTO_CALC);
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x15E0        ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
// 3535     active_calendar= opr_data[0];
        MOV       A, N:_opr_data     ;; 1 cycle
        MOV       N:_active_calendar, A  ;; 1 cycle
// 3536 
// 3537     eprom_read(TOU_CheckPassiveApli,0,PAGE_1,AUTO_CALC);
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x15D0        ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
// 3538     if(opr_data[0] == 1)
        CMP       N:_opr_data, #0x1  ;; 1 cycle
        ONEB      A                  ;; 1 cycle
        SKZ                          ;; 1 cycle
        ; ------------------------------------- Block: 20 cycles
        CLRB      A                  ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??tod_init_0:
        MOV       N:_calendar_change_f, A  ;; 1 cycle
// 3539         calendar_change_f= 1;
// 3540     else
// 3541         calendar_change_f= 0;
// 3542 
// 3543       check_active_calendar(); 
          CFI FunCall _check_active_calendar
        CALL      _check_active_calendar  ;; 3 cycles
// 3544       deter_season();
          CFI FunCall _deter_season
        CALL      _deter_season      ;; 3 cycles
// 3545       
// 3546       //this will be after chk_md_miss_pd() to take care miss cases in bill
// 3547 //      if(active_f != active_calendar)
// 3548 //      {
// 3549 //        deter_season();					              
// 3550 //        zone_change_f= 0;
// 3551 //        save_tod_flag= 0;                 
// 3552 //        save_tod_data();
// 3553 //        load_tod_data();
// 3554 //        mri_bill_flag=1;
// 3555 //        save_bill_data();
// 3556 //        mri_bill_flag=0;
// 3557 //      }
// 3558     
// 3559     eprom_read(TOU_VAR_SAVE,0,PAGE_1,AUTO_CALC);
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x7F0         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
// 3560     slot_no= opr_data[0];
        MOV       A, N:_opr_data     ;; 1 cycle
        MOV       N:_slot_no, A      ;; 1 cycle
// 3561     if(slot_no == 0)
        CMP0      N:_slot_no         ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 18 cycles
// 3562     {
// 3563         slot_no= 1;
        MOV       N:_slot_no, #0x1   ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
// 3564     }
// 3565 
// 3566     tariff_index= opr_data[1]; 
??tod_init_1:
        MOV       A, N:_opr_data+1   ;; 1 cycle
        MOV       N:_tariff_index, A  ;; 1 cycle
// 3567     if(tariff_index == 0)
        CMP0      N:_tariff_index    ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 4 cycles
// 3568     {
// 3569         tariff_index= 1;
        MOV       N:_tariff_index, #0x1  ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
// 3570     }
// 3571     tariff_no= tariff_index;
??tod_init_2:
        MOV       A, N:_tariff_index  ;; 1 cycle
        MOV       N:_tariff_no, A    ;; 1 cycle
// 3572     
// 3573     for(m2=0;  m2<8; m2++)
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        ; ------------------------------------- Block: 4 cycles
??tod_init_3:
        MOV       A, [SP]            ;; 1 cycle
        CMP       A, #0x8            ;; 1 cycle
        SKC                          ;; 4 cycles
        BR        N:??power_filter_392  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 3574     {
// 3575       eeblk=TOD_CUR_BILL_BLK1/256; 
        MOV       N:_eeblk, #0x16    ;; 1 cycle
// 3576       eepg=m2*16;
        MOV       A, [SP]            ;; 1 cycle
        MOV       X, #0x10           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       N:_eepg, A         ;; 1 cycle
// 3577 
// 3578       eprom_read(((eeblk*0x100)+eepg),0,PAGE_1,AUTO_CALC);
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        XCH       A, E               ;; 1 cycle
        MOV       H, A               ;; 1 cycle
        XCH       A, E               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        XCH       A, C               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        XCH       A, C               ;; 1 cycle
        MOV       X, N:_eeblk        ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x100         ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOV       C, N:_eepg         ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        ADDW      AX, BC             ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       C, A               ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        XCH       A, E               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, E               ;; 1 cycle
        XCH       A, H               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        XCH       A, H               ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
// 3579       t_zkwh1[m2] = char_array_to_long4(&opr_data[0]);      //all zone kwh imp.
        MOVW      AX, #LWRD(_opr_data)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+8
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        MOV       A, [SP+0x04]       ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x4           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(_t_zkwh1)  ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        POP       AX                 ;; 1 cycle
          CFI CFA SP+8
        POP       BC                 ;; 1 cycle
          CFI CFA SP+6
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3580       t_zkwh2[m2] = char_array_to_long4(&opr_data[4]);      //all zone kwh exp.
        MOVW      AX, #LWRD(_opr_data+4)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+8
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        MOV       A, [SP+0x04]       ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x4           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(_t_zkwh2)  ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        POP       AX                 ;; 1 cycle
          CFI CFA SP+8
        POP       BC                 ;; 1 cycle
          CFI CFA SP+6
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3581 
// 3582 
// 3583       eeblk=TOD_CUR_BILL_BLK3/256; 
        MOV       N:_eeblk, #0x17    ;; 1 cycle
// 3584       eepg=m2*16;
        MOV       A, [SP]            ;; 1 cycle
        MOV       X, #0x10           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       N:_eepg, A         ;; 1 cycle
// 3585       
// 3586       eprom_read(((eeblk*0x100)+eepg),0,PAGE_1,AUTO_CALC); 
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        XCH       A, E               ;; 1 cycle
        MOV       H, A               ;; 1 cycle
        XCH       A, E               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        XCH       A, C               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        XCH       A, C               ;; 1 cycle
        MOV       X, N:_eeblk        ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x100         ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOV       C, N:_eepg         ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        ADDW      AX, BC             ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       C, A               ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        XCH       A, E               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, E               ;; 1 cycle
        XCH       A, H               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        XCH       A, H               ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
// 3587       t_zkvah1[m2] = char_array_to_long4(&opr_data[0]);      //all zone kvah imp.
        MOVW      AX, #LWRD(_opr_data)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+8
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        MOV       A, [SP+0x04]       ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x4           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(_t_zkvah1)  ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        POP       AX                 ;; 1 cycle
          CFI CFA SP+8
        POP       BC                 ;; 1 cycle
          CFI CFA SP+6
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3588       t_zkvah2[m2] = char_array_to_long4(&opr_data[4]);      //all zone kvah exp.
        MOVW      AX, #LWRD(_opr_data+4)  ;; 1 cycle
          CFI FunCall _char_array_to_long4
        CALL      _char_array_to_long4  ;; 3 cycles
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+8
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        MOV       A, [SP+0x04]       ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x4           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(_t_zkvah2)  ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        POP       AX                 ;; 1 cycle
          CFI CFA SP+8
        POP       BC                 ;; 1 cycle
          CFI CFA SP+6
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3589     }
        MOV       A, [SP]            ;; 1 cycle
        INC       A                  ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        BR        N:??tod_init_3     ;; 3 cycles
        ; ------------------------------------- Block: 162 cycles
// 3590 }
??power_filter_392:
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock30
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 224 cycles
// 3591 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock31 Using cfiCommon0
          CFI Function _metrology_all_phase_calculation
        CODE
// 3592 void metrology_all_phase_calculation()
// 3593 {
_metrology_all_phase_calculation:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 32
        SUBW      SP, #0x20          ;; 1 cycle
          CFI CFA SP+36
// 3594     s64 temp_s641;
// 3595     /* Active calculation */
// 3596     all_phase.active.delta_cnts = r_phase.active.delta_cnts + y_phase.active.delta_cnts + b_phase.active.delta_cnts;
        MOVW      DE, #LWRD(_y_phase+96)  ;; 1 cycle
        MOVW      BC, #LWRD(_r_phase+96)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Add64
        CALL      __Add64            ;; 3 cycles
        MOVW      DE, #LWRD(_b_phase+96)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, #LWRD(_all_phase+24)  ;; 1 cycle
          CFI FunCall __Add64
        CALL      __Add64            ;; 3 cycles
// 3597     all_phase.fundamental.active.delta_cnts = r_phase.fundamental.active.delta_cnts + y_phase.fundamental.active.delta_cnts + b_phase.fundamental.active.delta_cnts; 
        MOVW      DE, #LWRD(_y_phase+200)  ;; 1 cycle
        MOVW      BC, #LWRD(_r_phase+200)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Add64
        CALL      __Add64            ;; 3 cycles
        MOVW      DE, #LWRD(_b_phase+200)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, #LWRD(_all_phase+128)  ;; 1 cycle
          CFI FunCall __Add64
        CALL      __Add64            ;; 3 cycles
// 3598     power.Allph.active = all_phase.active.delta_cnts/100000;
        MOVW      DE, #LWRD(__Constant_186a0_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_all_phase+24)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __LLU2L
        CALL      __LLU2L            ;; 3 cycles
        MOVW      N:_power+60, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_power+62, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3599     power.Allph.active_fundamental = all_phase.fundamental.active.delta_cnts/100000;
        MOVW      DE, #LWRD(__Constant_186a0_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_all_phase+128)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __LLU2L
        CALL      __LLU2L            ;; 3 cycles
        MOVW      N:_power+76, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_power+78, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3600     if(METERING_MODE == NET)
        CMP       N:_METERING_MODE, #0x1  ;; 1 cycle
        SKZ                          ;; 4 cycles
        BR        N:??power_filter_393  ;; 4 cycles
        ; ------------------------------------- Block: 60 cycles
// 3601     {
// 3602         if(quadrant.Rph == Q1 || quadrant.Rph == Q4)
        CMP       N:_quadrant, #0x1  ;; 1 cycle
        BZ        ??power_filter_394  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        CMP       N:_quadrant, #0x4  ;; 1 cycle
        BNZ       ??power_filter_395  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 3603         {
// 3604             temp_s64 = r_phase.active.delta_cnts;
??power_filter_394:
        MOVW      HL, #LWRD(_temp_s64)  ;; 1 cycle
        MOVW      DE, #LWRD(_r_phase+96)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 3605             temp_s641 = r_phase.fundamental.active.delta_cnts;
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      DE, #LWRD(_r_phase+200)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        BR        S:??power_filter_396  ;; 3 cycles
        ; ------------------------------------- Block: 25 cycles
// 3606         }
// 3607         else
// 3608         {
// 3609             temp_s64 = -r_phase.active.delta_cnts;
??power_filter_395:
        MOVW      BC, #LWRD(_r_phase+96)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Neg64
        CALL      __Neg64            ;; 3 cycles
        MOVW      HL, #LWRD(_temp_s64)  ;; 1 cycle
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
// 3610             temp_s641 = -r_phase.fundamental.active.delta_cnts;
        MOVW      BC, #LWRD(_r_phase+200)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Neg64
        CALL      __Neg64            ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
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
        ; ------------------------------------- Block: 34 cycles
// 3611         }
// 3612         if(quadrant.Yph == Q1 || quadrant.Yph == Q4)
??power_filter_396:
        CMP       N:_quadrant+1, #0x1  ;; 1 cycle
        BZ        ??power_filter_397  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        CMP       N:_quadrant+1, #0x4  ;; 1 cycle
        BNZ       ??power_filter_398  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 3613         {
// 3614             temp_s64 += y_phase.active.delta_cnts;
??power_filter_397:
        MOVW      DE, #LWRD(_y_phase+96)  ;; 1 cycle
        MOVW      BC, #LWRD(_temp_s64)  ;; 1 cycle
        MOVW      AX, #LWRD(_temp_s64)  ;; 1 cycle
          CFI FunCall __Add64
        CALL      __Add64            ;; 3 cycles
// 3615             temp_s641 += y_phase.fundamental.active.delta_cnts;
        MOVW      DE, #LWRD(_y_phase+200)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
          CFI FunCall __Add64
        CALL      __Add64            ;; 3 cycles
        BR        S:??power_filter_399  ;; 3 cycles
        ; ------------------------------------- Block: 18 cycles
// 3616         }
// 3617         else
// 3618         {
// 3619             temp_s64 -= y_phase.active.delta_cnts;
??power_filter_398:
        MOVW      DE, #LWRD(_y_phase+96)  ;; 1 cycle
        MOVW      BC, #LWRD(_temp_s64)  ;; 1 cycle
        MOVW      AX, #LWRD(_temp_s64)  ;; 1 cycle
          CFI FunCall __Sub64
        CALL      __Sub64            ;; 3 cycles
// 3620             temp_s641 -= y_phase.fundamental.active.delta_cnts;
        MOVW      DE, #LWRD(_y_phase+200)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
          CFI FunCall __Sub64
        CALL      __Sub64            ;; 3 cycles
        ; ------------------------------------- Block: 15 cycles
// 3621         }
// 3622         if(quadrant.Bph == Q1 || quadrant.Bph == Q4)
??power_filter_399:
        CMP       N:_quadrant+2, #0x1  ;; 1 cycle
        BZ        ??power_filter_400  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        CMP       N:_quadrant+2, #0x4  ;; 1 cycle
        BNZ       ??power_filter_401  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 3623         {
// 3624             temp_s64 += b_phase.active.delta_cnts;
??power_filter_400:
        MOVW      DE, #LWRD(_b_phase+96)  ;; 1 cycle
        MOVW      BC, #LWRD(_temp_s64)  ;; 1 cycle
        MOVW      AX, #LWRD(_temp_s64)  ;; 1 cycle
          CFI FunCall __Add64
        CALL      __Add64            ;; 3 cycles
// 3625             temp_s641 += b_phase.fundamental.active.delta_cnts;
        MOVW      DE, #LWRD(_b_phase+200)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
          CFI FunCall __Add64
        CALL      __Add64            ;; 3 cycles
        BR        S:??power_filter_402  ;; 3 cycles
        ; ------------------------------------- Block: 18 cycles
// 3626         }
// 3627         else
// 3628         {
// 3629             temp_s64 -= b_phase.active.delta_cnts;
??power_filter_401:
        MOVW      DE, #LWRD(_b_phase+96)  ;; 1 cycle
        MOVW      BC, #LWRD(_temp_s64)  ;; 1 cycle
        MOVW      AX, #LWRD(_temp_s64)  ;; 1 cycle
          CFI FunCall __Sub64
        CALL      __Sub64            ;; 3 cycles
// 3630             temp_s641 -= b_phase.fundamental.active.delta_cnts;
        MOVW      DE, #LWRD(_b_phase+200)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
          CFI FunCall __Sub64
        CALL      __Sub64            ;; 3 cycles
        ; ------------------------------------- Block: 15 cycles
// 3631         }
// 3632         power.Allph.active_signed = temp_s64/100000;
??power_filter_402:
        MOVW      DE, #LWRD(__Constant_186a0_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_temp_s64)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Divs64
        CALL      __Divs64           ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __LLS2L
        CALL      __LLS2L            ;; 3 cycles
        MOVW      N:_power+80, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_power+82, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3633         power.Allph.active_signed_fundamental = temp_s641/100000;
        MOVW      DE, #LWRD(__Constant_186a0_0)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Divs64
        CALL      __Divs64           ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __LLS2L
        CALL      __LLS2L            ;; 3 cycles
        MOVW      N:_power+88, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_power+90, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3634         all_phase.active.delta_cnts_net = ABS(temp_s64);
        MOVW      BC, #LWRD(__Constant_0_0)  ;; 1 cycle
        MOVW      AX, #LWRD(_temp_s64)  ;; 1 cycle
          CFI FunCall __CmpGes64
        CALL      __CmpGes64         ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_403  ;; 4 cycles
        ; ------------------------------------- Block: 40 cycles
        MOVW      BC, #LWRD(_temp_s64)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Neg64
        CALL      __Neg64            ;; 3 cycles
        BR        S:??power_filter_404  ;; 3 cycles
        ; ------------------------------------- Block: 8 cycles
??power_filter_403:
        MOVW      AX, SP             ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      DE, #LWRD(_temp_s64)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        ; ------------------------------------- Block: 11 cycles
??power_filter_404:
        MOVW      HL, #LWRD(_all_phase+32)  ;; 1 cycle
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
        BR        S:??power_filter_405  ;; 3 cycles
        ; ------------------------------------- Block: 14 cycles
// 3635     }
// 3636     else 
// 3637     {
// 3638         power.Allph.active_signed = power.Allph.active;
??power_filter_393:
        MOVW      BC, N:_power+62    ;; 1 cycle
        MOVW      AX, N:_power+60    ;; 1 cycle
        MOVW      N:_power+80, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_power+82, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3639         power.Allph.active_signed_fundamental = power.Allph.active_fundamental;
        MOVW      BC, N:_power+78    ;; 1 cycle
        MOVW      AX, N:_power+76    ;; 1 cycle
        MOVW      N:_power+88, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_power+90, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3640         all_phase.active.delta_cnts_net = all_phase.active.delta_cnts;
        MOVW      HL, #LWRD(_all_phase+32)  ;; 1 cycle
        MOVW      DE, #LWRD(_all_phase+24)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        ; ------------------------------------- Block: 22 cycles
// 3641     }
// 3642     if(power.Allph.active_signed < 0)
??power_filter_405:
        MOVW      BC, N:_power+82    ;; 1 cycle
        MOVW      AX, N:_power+80    ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        XOR       A, #0x80           ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x8000        ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 9 cycles
        CMPW      AX, #0x0           ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??metrology_all_phase_calculation_0:
        BNC       ??power_filter_406  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
// 3643     {
// 3644         flag_Allph_active = EXPORT;
        SET1      N:_flag_quadrant.6  ;; 2 cycles
        BR        S:??power_filter_407  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 3645     }
// 3646     else
// 3647     {
// 3648         flag_Allph_active = IMPORT;
??power_filter_406:
        CLR1      N:_flag_quadrant.6  ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
// 3649     }
// 3650     
// 3651     /* Reactive Calculation */
// 3652     all_phase.reactive.delta_cnts = r_phase.reactive.delta_cnts + y_phase.reactive.delta_cnts + b_phase.reactive.delta_cnts;
??power_filter_407:
        MOVW      DE, #LWRD(_y_phase+136)  ;; 1 cycle
        MOVW      BC, #LWRD(_r_phase+136)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Add64
        CALL      __Add64            ;; 3 cycles
        MOVW      DE, #LWRD(_b_phase+136)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, #LWRD(_all_phase+64)  ;; 1 cycle
          CFI FunCall __Add64
        CALL      __Add64            ;; 3 cycles
// 3653     power.Allph.reactive = all_phase.reactive.delta_cnts/100000;
        MOVW      DE, #LWRD(__Constant_186a0_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_all_phase+64)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __LLU2L
        CALL      __LLU2L            ;; 3 cycles
        MOVW      N:_power+64, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_power+66, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3654     if(METERING_MODE == NET)
        CMP       N:_METERING_MODE, #0x1  ;; 1 cycle
        SKZ                          ;; 4 cycles
        BR        N:??power_filter_408  ;; 4 cycles
        ; ------------------------------------- Block: 32 cycles
// 3655     {
// 3656         if(quadrant.Rph == Q1 || quadrant.Rph == Q2)
        CMP       N:_quadrant, #0x1  ;; 1 cycle
        BZ        ??power_filter_409  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        CMP       N:_quadrant, #0x2  ;; 1 cycle
        BNZ       ??power_filter_410  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 3657         {
// 3658             temp_s64 = r_phase.reactive.delta_cnts;
??power_filter_409:
        MOVW      HL, #LWRD(_temp_s64)  ;; 1 cycle
        MOVW      DE, #LWRD(_r_phase+136)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        BR        S:??power_filter_411  ;; 3 cycles
        ; ------------------------------------- Block: 13 cycles
// 3659         }
// 3660         else
// 3661         {
// 3662             temp_s64 = -r_phase.reactive.delta_cnts;
??power_filter_410:
        MOVW      BC, #LWRD(_r_phase+136)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Neg64
        CALL      __Neg64            ;; 3 cycles
        MOVW      HL, #LWRD(_temp_s64)  ;; 1 cycle
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
        ; ------------------------------------- Block: 16 cycles
// 3663         }
// 3664         if(quadrant.Yph == Q1 || quadrant.Yph == Q2)
??power_filter_411:
        CMP       N:_quadrant+1, #0x1  ;; 1 cycle
        BZ        ??power_filter_412  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        CMP       N:_quadrant+1, #0x2  ;; 1 cycle
        BNZ       ??power_filter_413  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 3665         {
// 3666             temp_s64 += y_phase.reactive.delta_cnts;
??power_filter_412:
        MOVW      DE, #LWRD(_y_phase+136)  ;; 1 cycle
        MOVW      BC, #LWRD(_temp_s64)  ;; 1 cycle
        MOVW      AX, #LWRD(_temp_s64)  ;; 1 cycle
          CFI FunCall __Add64
        CALL      __Add64            ;; 3 cycles
        BR        S:??power_filter_414  ;; 3 cycles
        ; ------------------------------------- Block: 9 cycles
// 3667         }
// 3668         else
// 3669         {
// 3670             temp_s64 -= y_phase.reactive.delta_cnts;
??power_filter_413:
        MOVW      DE, #LWRD(_y_phase+136)  ;; 1 cycle
        MOVW      BC, #LWRD(_temp_s64)  ;; 1 cycle
        MOVW      AX, #LWRD(_temp_s64)  ;; 1 cycle
          CFI FunCall __Sub64
        CALL      __Sub64            ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
// 3671         }
// 3672         if(quadrant.Bph == Q1 || quadrant.Bph == Q2)
??power_filter_414:
        CMP       N:_quadrant+2, #0x1  ;; 1 cycle
        BZ        ??power_filter_415  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        CMP       N:_quadrant+2, #0x2  ;; 1 cycle
        BNZ       ??power_filter_416  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 3673         {
// 3674             temp_s64 += b_phase.reactive.delta_cnts;
??power_filter_415:
        MOVW      DE, #LWRD(_b_phase+136)  ;; 1 cycle
        MOVW      BC, #LWRD(_temp_s64)  ;; 1 cycle
        MOVW      AX, #LWRD(_temp_s64)  ;; 1 cycle
          CFI FunCall __Add64
        CALL      __Add64            ;; 3 cycles
        BR        N:??power_filter_417  ;; 3 cycles
        ; ------------------------------------- Block: 9 cycles
// 3675         }
// 3676         else
// 3677         {
// 3678             temp_s64 -= b_phase.reactive.delta_cnts;
??power_filter_416:
        MOVW      DE, #LWRD(_b_phase+136)  ;; 1 cycle
        MOVW      BC, #LWRD(_temp_s64)  ;; 1 cycle
        MOVW      AX, #LWRD(_temp_s64)  ;; 1 cycle
          CFI FunCall __Sub64
        CALL      __Sub64            ;; 3 cycles
        BR        N:??power_filter_417  ;; 3 cycles
        ; ------------------------------------- Block: 9 cycles
// 3679         }
// 3680     }
// 3681     else
// 3682     {
// 3683         if(quadrant.Rph == Q1 || quadrant.Rph == Q3)
??power_filter_408:
        CMP       N:_quadrant, #0x1  ;; 1 cycle
        BZ        ??power_filter_418  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        CMP       N:_quadrant, #0x3  ;; 1 cycle
        BNZ       ??power_filter_419  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 3684         {
// 3685             temp_s64 = r_phase.reactive.delta_cnts;
??power_filter_418:
        MOVW      HL, #LWRD(_temp_s64)  ;; 1 cycle
        MOVW      DE, #LWRD(_r_phase+136)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        BR        S:??power_filter_420  ;; 3 cycles
        ; ------------------------------------- Block: 13 cycles
// 3686         }
// 3687         else
// 3688         {
// 3689             temp_s64 = -r_phase.reactive.delta_cnts;
??power_filter_419:
        MOVW      BC, #LWRD(_r_phase+136)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Neg64
        CALL      __Neg64            ;; 3 cycles
        MOVW      HL, #LWRD(_temp_s64)  ;; 1 cycle
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
        ; ------------------------------------- Block: 16 cycles
// 3690         }
// 3691         if(quadrant.Yph == Q1 || quadrant.Yph == Q3)
??power_filter_420:
        CMP       N:_quadrant+1, #0x1  ;; 1 cycle
        BZ        ??power_filter_421  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        CMP       N:_quadrant+1, #0x3  ;; 1 cycle
        BNZ       ??power_filter_422  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 3692         {
// 3693             temp_s64 += y_phase.reactive.delta_cnts;
??power_filter_421:
        MOVW      DE, #LWRD(_y_phase+136)  ;; 1 cycle
        MOVW      BC, #LWRD(_temp_s64)  ;; 1 cycle
        MOVW      AX, #LWRD(_temp_s64)  ;; 1 cycle
          CFI FunCall __Add64
        CALL      __Add64            ;; 3 cycles
        BR        S:??power_filter_423  ;; 3 cycles
        ; ------------------------------------- Block: 9 cycles
// 3694         }
// 3695         else
// 3696         {
// 3697             temp_s64 -= y_phase.reactive.delta_cnts;
??power_filter_422:
        MOVW      DE, #LWRD(_y_phase+136)  ;; 1 cycle
        MOVW      BC, #LWRD(_temp_s64)  ;; 1 cycle
        MOVW      AX, #LWRD(_temp_s64)  ;; 1 cycle
          CFI FunCall __Sub64
        CALL      __Sub64            ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
// 3698         }
// 3699         if(quadrant.Bph == Q1 || quadrant.Bph == Q3)
??power_filter_423:
        CMP       N:_quadrant+2, #0x1  ;; 1 cycle
        BZ        ??power_filter_424  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        CMP       N:_quadrant+2, #0x3  ;; 1 cycle
        BNZ       ??power_filter_425  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 3700         {
// 3701             temp_s64 += b_phase.reactive.delta_cnts;
??power_filter_424:
        MOVW      DE, #LWRD(_b_phase+136)  ;; 1 cycle
        MOVW      BC, #LWRD(_temp_s64)  ;; 1 cycle
        MOVW      AX, #LWRD(_temp_s64)  ;; 1 cycle
          CFI FunCall __Add64
        CALL      __Add64            ;; 3 cycles
        BR        S:??power_filter_417  ;; 3 cycles
        ; ------------------------------------- Block: 9 cycles
// 3702         }
// 3703         else
// 3704         {
// 3705             temp_s64 -= b_phase.reactive.delta_cnts;
??power_filter_425:
        MOVW      DE, #LWRD(_b_phase+136)  ;; 1 cycle
        MOVW      BC, #LWRD(_temp_s64)  ;; 1 cycle
        MOVW      AX, #LWRD(_temp_s64)  ;; 1 cycle
          CFI FunCall __Sub64
        CALL      __Sub64            ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
// 3706         }
// 3707     }
// 3708     power.Allph.reactive_signed = temp_s64/100000;    
??power_filter_417:
        MOVW      DE, #LWRD(__Constant_186a0_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_temp_s64)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Divs64
        CALL      __Divs64           ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __LLS2L
        CALL      __LLS2L            ;; 3 cycles
        MOVW      N:_power+84, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_power+86, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3709     all_phase.reactive.delta_cnts_net = ABS(temp_s64);
        MOVW      BC, #LWRD(__Constant_0_0)  ;; 1 cycle
        MOVW      AX, #LWRD(_temp_s64)  ;; 1 cycle
          CFI FunCall __CmpGes64
        CALL      __CmpGes64         ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_426  ;; 4 cycles
        ; ------------------------------------- Block: 24 cycles
        MOVW      BC, #LWRD(_temp_s64)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Neg64
        CALL      __Neg64            ;; 3 cycles
        BR        S:??power_filter_427  ;; 3 cycles
        ; ------------------------------------- Block: 8 cycles
??power_filter_426:
        MOVW      AX, SP             ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      DE, #LWRD(_temp_s64)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        ; ------------------------------------- Block: 11 cycles
??power_filter_427:
        MOVW      HL, #LWRD(_all_phase+72)  ;; 1 cycle
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
// 3710     if(power.Allph.reactive_signed < 0)
        MOVW      BC, N:_power+86    ;; 1 cycle
        MOVW      AX, N:_power+84    ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        XOR       A, #0x80           ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x8000        ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 20 cycles
        CMPW      AX, #0x0           ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??metrology_all_phase_calculation_1:
        BNC       ??power_filter_428  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
// 3711     {
// 3712         flag_Allph_reactive = EXPORT;
        SET1      N:_flag_quadrant.7  ;; 2 cycles
        BR        S:??power_filter_429  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 3713     }
// 3714     else
// 3715     {
// 3716         flag_Allph_reactive = IMPORT;
??power_filter_428:
        CLR1      N:_flag_quadrant.7  ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
// 3717     }
// 3718     
// 3719     
// 3720     /* Apparent Calculation */
// 3721     temp_us64 = (us64)((us64)all_phase.active.delta_cnts/100)*((us64)all_phase.active.delta_cnts/100);
??power_filter_429:
        MOVW      DE, #LWRD(__Constant_64_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_all_phase+24)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
        MOVW      DE, #LWRD(__Constant_64_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_all_phase+24)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __Mul64
        CALL      __Mul64            ;; 3 cycles
// 3722     temp_us64 += (us64)((us64)all_phase.reactive.delta_cnts/100)*((us64)all_phase.reactive.delta_cnts/100);
        MOVW      DE, #LWRD(__Constant_64_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_all_phase+64)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
        MOVW      DE, #LWRD(__Constant_64_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_all_phase+64)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x18          ;; 1 cycle
          CFI FunCall __Mul64
        CALL      __Mul64            ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x18          ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      BC, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __Add64
        CALL      __Add64            ;; 3 cycles
// 3723     all_phase.apparent.delta_cnts = (us64)sqrt(temp_us64)*100;
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __LLU2F
        CALL      __LLU2F            ;; 3 cycles
          CFI FunCall _sqrt
        CALL      _sqrt              ;; 3 cycles
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+38
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+40
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
          CFI FunCall __F2LLU
        CALL      __F2LLU            ;; 3 cycles
        MOVW      DE, #LWRD(__Constant_64_0)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, #LWRD(_all_phase+104)  ;; 1 cycle
          CFI FunCall __Mul64
        CALL      __Mul64            ;; 3 cycles
// 3724     if(all_phase.apparent.delta_cnts < all_phase.active.delta_cnts)
        MOVW      BC, #LWRD(_all_phase+24)  ;; 1 cycle
        MOVW      AX, #LWRD(_all_phase+104)  ;; 1 cycle
          CFI FunCall __CmpGeu64
        CALL      __CmpGeu64         ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+36
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_430  ;; 4 cycles
        ; ------------------------------------- Block: 86 cycles
// 3725     {
// 3726         all_phase.apparent.delta_cnts = all_phase.active.delta_cnts;
        MOVW      HL, #LWRD(_all_phase+104)  ;; 1 cycle
        MOVW      DE, #LWRD(_all_phase+24)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        ; ------------------------------------- Block: 10 cycles
// 3727     }
// 3728     power.Allph.apparent = all_phase.apparent.delta_cnts/100000;
??power_filter_430:
        MOVW      DE, #LWRD(__Constant_186a0_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_all_phase+104)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __LLU2L
        CALL      __LLU2L            ;; 3 cycles
        MOVW      N:_power+68, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_power+70, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3729     
// 3730     temp_us64 = (us64)((us64)all_phase.active.delta_cnts_net/100)*((us64)all_phase.active.delta_cnts_net/100);
        MOVW      DE, #LWRD(__Constant_64_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_all_phase+32)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
        MOVW      DE, #LWRD(__Constant_64_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_all_phase+32)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __Mul64
        CALL      __Mul64            ;; 3 cycles
// 3731     temp_us64 += (us64)((us64)all_phase.reactive.delta_cnts_net/100)*((us64)all_phase.reactive.delta_cnts_net/100);
        MOVW      DE, #LWRD(__Constant_64_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_all_phase+72)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
        MOVW      DE, #LWRD(__Constant_64_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_all_phase+72)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
          CFI FunCall __Divu64
        CALL      __Divu64           ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x18          ;; 1 cycle
          CFI FunCall __Mul64
        CALL      __Mul64            ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x18          ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      BC, #LWRD(_temp_us64)  ;; 1 cycle
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __Add64
        CALL      __Add64            ;; 3 cycles
// 3732     all_phase.apparent.delta_cnts_net = (us64)sqrt(temp_us64)*100;
        MOVW      AX, #LWRD(_temp_us64)  ;; 1 cycle
          CFI FunCall __LLU2F
        CALL      __LLU2F            ;; 3 cycles
          CFI FunCall _sqrt
        CALL      _sqrt              ;; 3 cycles
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+38
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+40
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
          CFI FunCall __F2LLU
        CALL      __F2LLU            ;; 3 cycles
        MOVW      DE, #LWRD(__Constant_64_0)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x14          ;; 1 cycle
          CFI FunCall __Mul64
        CALL      __Mul64            ;; 3 cycles
        MOVW      HL, #LWRD(_all_phase+112)  ;; 1 cycle
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+36
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 3733     if(all_phase.apparent.delta_cnts_net < all_phase.active.delta_cnts_net)
        MOVW      BC, #LWRD(_all_phase+32)  ;; 1 cycle
        MOVW      AX, #LWRD(_all_phase+112)  ;; 1 cycle
          CFI FunCall __CmpGes64
        CALL      __CmpGes64         ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_431  ;; 4 cycles
        ; ------------------------------------- Block: 113 cycles
// 3734     {
// 3735         all_phase.apparent.delta_cnts_net = all_phase.active.delta_cnts_net;
        MOVW      HL, #LWRD(_all_phase+112)  ;; 1 cycle
        MOVW      DE, #LWRD(_all_phase+32)  ;; 1 cycle
        MOVW      AX, [DE]           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, [DE+0x02]      ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, [DE+0x04]      ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, [DE+0x06]      ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        ; ------------------------------------- Block: 10 cycles
// 3736     }
// 3737     power.Allph.apparent_net = all_phase.apparent.delta_cnts_net/100000;
??power_filter_431:
        MOVW      DE, #LWRD(__Constant_186a0_0)  ;; 1 cycle
        MOVW      BC, #LWRD(_all_phase+112)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __Divs64
        CALL      __Divs64           ;; 3 cycles
        MOVW      AX, SP             ;; 1 cycle
          CFI FunCall __LLS2L
        CALL      __LLS2L            ;; 3 cycles
        MOVW      N:_power+72, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_power+74, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3738     
// 3739     /* PF */
// 3740     pf.Net = cal_pf(power.Allph.active,power.Allph.apparent);
        MOVW      AX, N:_power+70    ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+38
        MOVW      AX, N:_power+68    ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+40
        MOVW      BC, N:_power+62    ;; 1 cycle
        MOVW      AX, N:_power+60    ;; 1 cycle
          CFI FunCall _cal_pf
        CALL      _cal_pf            ;; 3 cycles
        MOVW      N:_pf+6, AX        ;; 1 cycle
// 3741     pf.Net_signed = cal_pf_signed(power.Allph.active_signed,power.Allph.apparent_net);
        MOVW      AX, N:_power+74    ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+42
        MOVW      AX, N:_power+72    ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+44
        MOVW      BC, N:_power+82    ;; 1 cycle
        MOVW      AX, N:_power+80    ;; 1 cycle
          CFI FunCall _cal_pf_signed
        CALL      _cal_pf_signed     ;; 3 cycles
        MOVW      N:_pf+14, AX       ;; 1 cycle
// 3742 
// 3743     /* Quadrant */
// 3744     quadrant.Allph = get_quadrant(flag_Allph_active,flag_Allph_reactive);
        MOVW      HL, #LWRD(_flag_quadrant)  ;; 1 cycle
        MOV       A, [HL]            ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOVW      HL, #LWRD(_flag_quadrant)  ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOV1      CY, [HL].6         ;; 1 cycle
        ROLC      A, 0x1             ;; 1 cycle
          CFI FunCall _get_quadrant
        CALL      _get_quadrant      ;; 3 cycles
        MOV       N:_quadrant+3, A   ;; 1 cycle
// 3745     
// 3746     /* emptying bucket */
// 3747     if(EMPTY_BUCKET == 1)
// 3748     {
// 3749         if(r_phase.active.acc_delta_cnts==0 && y_phase.active.acc_delta_cnts==0 && b_phase.active.acc_delta_cnts==0)
        MOVW      BC, #LWRD(__Constant_0_0)  ;; 1 cycle
        MOVW      AX, #LWRD(_r_phase+88)  ;; 1 cycle
          CFI FunCall __CmpNe64
        CALL      __CmpNe64          ;; 3 cycles
        ADDW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+36
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_432  ;; 4 cycles
        ; ------------------------------------- Block: 57 cycles
        MOVW      BC, #LWRD(__Constant_0_0)  ;; 1 cycle
        MOVW      AX, #LWRD(_y_phase+88)  ;; 1 cycle
          CFI FunCall __CmpNe64
        CALL      __CmpNe64          ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_432  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
        MOVW      BC, #LWRD(__Constant_0_0)  ;; 1 cycle
        MOVW      AX, #LWRD(_b_phase+88)  ;; 1 cycle
          CFI FunCall __CmpNe64
        CALL      __CmpNe64          ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_432  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
// 3750         {
// 3751             all_phase.active.acc_delta_cnts = 0;
        MOVW      HL, #LWRD(_all_phase+16)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
// 3752             all_phase.fundamental.active.acc_delta_cnts = 0;
        MOVW      HL, #LWRD(_all_phase+120)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        ; ------------------------------------- Block: 18 cycles
// 3753         }
// 3754         if(METERING_MODE == NET)
??power_filter_432:
        CMP       N:_METERING_MODE, #0x1  ;; 1 cycle
        BNZ       ??power_filter_433  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 3755         {
// 3756             if(r_phase.reactive.acc_delta_cnts==0 && y_phase.reactive.acc_delta_cnts==0 && b_phase.reactive.acc_delta_cnts==0)
        MOVW      BC, #LWRD(__Constant_0_0)  ;; 1 cycle
        MOVW      AX, #LWRD(_r_phase+128)  ;; 1 cycle
          CFI FunCall __CmpNe64
        CALL      __CmpNe64          ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_434  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
        MOVW      BC, #LWRD(__Constant_0_0)  ;; 1 cycle
        MOVW      AX, #LWRD(_y_phase+128)  ;; 1 cycle
          CFI FunCall __CmpNe64
        CALL      __CmpNe64          ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_434  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
        MOVW      BC, #LWRD(__Constant_0_0)  ;; 1 cycle
        MOVW      AX, #LWRD(_b_phase+128)  ;; 1 cycle
          CFI FunCall __CmpNe64
        CALL      __CmpNe64          ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_434  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
// 3757             {
// 3758                 all_phase.reactive.acc_delta_cnts = 0;
        MOVW      HL, #LWRD(_all_phase+56)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        BR        S:??power_filter_434  ;; 3 cycles
        ; ------------------------------------- Block: 12 cycles
// 3759             }
// 3760         }
// 3761         else
// 3762         {
// 3763             if(all_phase.reactive.delta_cnts_net==0)
??power_filter_433:
        MOVW      BC, #LWRD(__Constant_0_0)  ;; 1 cycle
        MOVW      AX, #LWRD(_all_phase+72)  ;; 1 cycle
          CFI FunCall __CmpNe64
        CALL      __CmpNe64          ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_434  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
// 3764             {
// 3765                 all_phase.reactive.acc_delta_cnts = 0;
        MOVW      HL, #LWRD(_all_phase+56)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        ; ------------------------------------- Block: 9 cycles
// 3766             }
// 3767         }
// 3768         if(all_phase.active.acc_delta_cnts == 0 && all_phase.reactive.acc_delta_cnts == 0)
??power_filter_434:
        MOVW      BC, #LWRD(__Constant_0_0)  ;; 1 cycle
        MOVW      AX, #LWRD(_all_phase+16)  ;; 1 cycle
          CFI FunCall __CmpNe64
        CALL      __CmpNe64          ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_435  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
        MOVW      BC, #LWRD(__Constant_0_0)  ;; 1 cycle
        MOVW      AX, #LWRD(_all_phase+56)  ;; 1 cycle
          CFI FunCall __CmpNe64
        CALL      __CmpNe64          ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??power_filter_435  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
// 3769         {
// 3770           all_phase.apparent.acc_delta_cnts = 0;
        MOVW      HL, #LWRD(_all_phase+96)  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x04], AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x06], AX      ;; 1 cycle
        ; ------------------------------------- Block: 9 cycles
// 3771         }
// 3772     }
// 3773     if(flag_mag_update_metro_par == 1)
??power_filter_435:
        MOVW      HL, #LWRD(_flag_mag)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??power_filter_436  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 3774     {
// 3775         if(flag_mag_r_updated == 1 && flag_mag_y_updated == 1 && flag_mag_b_updated == 1)
        MOV       A, N:_flag_mag     ;; 1 cycle
        AND       A, #0xE            ;; 1 cycle
        CMP       A, #0xE            ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 4 cycles
// 3776         {
// 3777             flag_mag_all_updated = 1;
        SET1      N:_flag_mag.4      ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
// 3778         }
// 3779     }
// 3780 }
??power_filter_436:
        ADDW      SP, #0x20          ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock31
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 1047 cycles
// 3781 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock32 Using cfiCommon0
          CFI Function _energy_clear
        CODE
// 3782 void energy_clear()
// 3783 {
_energy_clear:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 2
        SUBW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+6
// 3784    uint8_t m1;
// 3785    
// 3786    fill_oprzero(128);
        MOV       A, #0x80           ;; 1 cycle
          CFI FunCall _fill_oprzero
        CALL      _fill_oprzero      ;; 3 cycles
// 3787       /* Variables clearing flag_sleep_execute clear all the variables in FG which are being saved during going to sleep */
// 3788     energy.Allph.active_imp = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_energy+52, AX   ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_energy+54, AX   ;; 1 cycle
// 3789     energy.Allph.apparent_imp = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_energy+68, AX   ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_energy+70, AX   ;; 1 cycle
// 3790     energy.Allph.reactive_q1 = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_energy+76, AX   ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_energy+78, AX   ;; 1 cycle
// 3791     energy.Allph.reactive_q4 = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_energy+88, AX   ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_energy+90, AX   ;; 1 cycle
// 3792     energy.Allph.active_exp = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_energy+56, AX   ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_energy+58, AX   ;; 1 cycle
// 3793     energy.Allph.apparent_exp = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_energy+72, AX   ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_energy+74, AX   ;; 1 cycle
// 3794     energy.Allph.reactive_q2 = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_energy+80, AX   ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_energy+82, AX   ;; 1 cycle
// 3795     energy.Allph.reactive_q3 = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_energy+84, AX   ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_energy+86, AX   ;; 1 cycle
// 3796     energy.Allph.defraud_mag = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_energy+60, AX   ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_energy+62, AX   ;; 1 cycle
// 3797     energy.Allph.fundamental = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_energy+64, AX   ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_energy+66, AX   ;; 1 cycle
// 3798     energy.Rph.active_imp = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_energy+2, AX    ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_energy+4, AX    ;; 1 cycle
// 3799     energy.Rph.active_exp = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_energy+6, AX    ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_energy+8, AX    ;; 1 cycle
// 3800     energy.Yph.active_imp = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_energy+16, AX   ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_energy+18, AX   ;; 1 cycle
// 3801     energy.Yph.active_exp = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_energy+20, AX   ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_energy+22, AX   ;; 1 cycle
// 3802     energy.Bph.active_imp = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_energy+30, AX   ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_energy+32, AX   ;; 1 cycle
// 3803     energy.Bph.active_exp = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_energy+34, AX   ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_energy+36, AX   ;; 1 cycle
// 3804     energy.Rph.defraud_mag = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_energy+10, AX   ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_energy+12, AX   ;; 1 cycle
// 3805     energy.Yph.defraud_mag = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_energy+24, AX   ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_energy+26, AX   ;; 1 cycle
// 3806     energy.Bph.defraud_mag = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_energy+38, AX   ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_energy+40, AX   ;; 1 cycle
// 3807     energy.Allph.zkwh_imp = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_energy+92, AX   ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_energy+94, AX   ;; 1 cycle
// 3808     energy.Allph.zkvah_imp = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_energy+100, AX  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_energy+102, AX  ;; 1 cycle
// 3809     energy.Allph.zkvarh_q1 = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_energy+108, AX  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_energy+110, AX  ;; 1 cycle
// 3810     energy.Allph.zkvarh_q4 = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_energy+120, AX  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_energy+122, AX  ;; 1 cycle
// 3811     energy.Allph.zkwh_exp = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_energy+96, AX   ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_energy+98, AX   ;; 1 cycle
// 3812     energy.Allph.zkvah_exp = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_energy+104, AX  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_energy+106, AX  ;; 1 cycle
// 3813     energy.Allph.zkvarh_q2 = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_energy+112, AX  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_energy+114, AX  ;; 1 cycle
// 3814     energy.Allph.zkvarh_q3 = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_energy+116, AX  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_energy+118, AX  ;; 1 cycle
// 3815     energy.Allph.active_imp_pulse = 0;
        MOV       N:_energy+42, #0x0  ;; 1 cycle
// 3816     energy.Allph.active_exp_pulse = 0;
        MOV       N:_energy+43, #0x0  ;; 1 cycle
// 3817     energy.Allph.reactive_q1_pulse = 0;
        MOV       N:_energy+47, #0x0  ;; 1 cycle
// 3818     energy.Allph.reactive_q2_pulse = 0;
        MOV       N:_energy+48, #0x0  ;; 1 cycle
// 3819     energy.Allph.reactive_q3_pulse = 0;
        MOV       N:_energy+49, #0x0  ;; 1 cycle
// 3820     energy.Allph.reactive_q4_pulse = 0;
        MOV       N:_energy+50, #0x0  ;; 1 cycle
// 3821     energy.Allph.apparent_imp_pulse = 0;
        MOV       N:_energy+45, #0x0  ;; 1 cycle
// 3822     energy.Allph.apparent_exp_pulse = 0;
        MOV       N:_energy+46, #0x0  ;; 1 cycle
// 3823     energy.Rph.active_imp_pulse = 0;
        MOV       N:_energy, #0x0    ;; 1 cycle
// 3824     energy.Rph.active_exp_pulse = 0;
        MOV       N:_energy+1, #0x0  ;; 1 cycle
// 3825     energy.Yph.active_imp_pulse = 0;
        MOV       N:_energy+14, #0x0  ;; 1 cycle
// 3826     energy.Yph.active_exp_pulse = 0;
        MOV       N:_energy+15, #0x0  ;; 1 cycle
// 3827     energy.Bph.active_imp_pulse = 0;
        MOV       N:_energy+28, #0x0  ;; 1 cycle
// 3828     energy.Bph.active_exp_pulse = 0;
        MOV       N:_energy+29, #0x0  ;; 1 cycle
// 3829     alt_energy_save_cntr = 0;
        MOV       N:_alt_energy_save_cntr, #0x0  ;; 1 cycle
// 3830     energy.Allph.fundamental_pulse = 0;
        MOV       N:_energy+44, #0x0  ;; 1 cycle
// 3831     zone_pf= 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_zone_pf, AX     ;; 1 cycle
// 3832     duplicate_total_apparent_energy=0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_duplicate_total_apparent_energy, AX  ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_duplicate_total_apparent_energy+2, AX  ;; 1 cycle
// 3833 
// 3834      /* Energy */
// 3835     for(m1=0;m1<16;m1++) //main energy circular buffer location cleared
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        ; ------------------------------------- Block: 137 cycles
??energy_clear_0:
        MOV       A, [SP]            ;; 1 cycle
        CMP       A, #0x10           ;; 1 cycle
        BNC       ??power_filter_437  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 3836     {
// 3837       eprom_write((ALL_ENERGY_SAVE_START_ADD+(m1 * ALL_ENERGY_SAVE_BLOCK_SIZE)),0,ALL_ENERGY_SAVE_BLOCK_SIZE,PAGE_7,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+8
        MOV       B, #0x6            ;; 1 cycle
        MOVW      DE, #0x70          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        MOV       H, A               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        PUSH      DE                 ;; 1 cycle
          CFI CFA SP+10
        XCH       A, C               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        XCH       A, C               ;; 1 cycle
        MOV       A, [SP+0x04]       ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x70          ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #0x6500        ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       C, A               ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        POP       DE                 ;; 1 cycle
          CFI CFA SP+8
        XCH       A, H               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, H               ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
// 3838       eprom_write((LEAD_ENERGY_SAVE_START_ADD+(m1 * LEAD_ENERGY_SAVE_BLOCK_SIZE)),0,LEAD_ENERGY_SAVE_BLOCK_SIZE,PAGE_1,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        MOV       H, A               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        PUSH      DE                 ;; 1 cycle
          CFI CFA SP+12
        XCH       A, C               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        XCH       A, C               ;; 1 cycle
        MOV       A, [SP+0x06]       ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x10          ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #0x6C00        ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       C, A               ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        POP       DE                 ;; 1 cycle
          CFI CFA SP+10
        XCH       A, H               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, H               ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
// 3839     }
        MOV       A, [SP+0x04]       ;; 1 cycle
        INC       A                  ;; 1 cycle
        MOV       [SP+0x04], A       ;; 1 cycle
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+6
        BR        S:??energy_clear_0  ;; 3 cycles
        ; ------------------------------------- Block: 65 cycles
// 3840     eprom_write(ALTERNATE_ALL_ENERGY_SAVE_START_ADD,2,ALL_ENERGY_SAVE_BLOCK_SIZE,PAGE_7,AUTO_CALC);
??power_filter_437:
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+8
        MOV       B, #0x6            ;; 1 cycle
        MOVW      DE, #0x70          ;; 1 cycle
        MOV       C, #0x2            ;; 1 cycle
        MOVW      AX, #0xFF00        ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
// 3841     eprom_write( ALTERNATE_LEAD_ENERGY_SAVE_START_ADD,2,LEAD_ENERGY_SAVE_BLOCK_SIZE,PAGE_1,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x2            ;; 1 cycle
        MOVW      AX, #0xFF70        ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
// 3842     /* Energy Pulses */
// 3843     eprom_write(0x0CD0,0,16,PAGE_1,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xCD0         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
// 3844     eprom_write(0x0CF0,0,16,PAGE_1,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+14
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0xCF0         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
// 3845 
// 3846 }
        ADDW      SP, #0xA           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock32
        ; ------------------------------------- Block: 43 cycles
        ; ------------------------------------- Total: 251 cycles
// 3847 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock33 Using cfiCommon0
          CFI Function _rollover
        CODE
// 3848 void rollover()
// 3849 {
_rollover:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
// 3850    /* Pending, rollover check based on summation of energy */
// 3851    energy.Allph.active_imp = rollover_energy(energy.Allph.active_imp);
        MOVW      BC, N:_energy+54   ;; 1 cycle
        MOVW      AX, N:_energy+52   ;; 1 cycle
          CFI FunCall _rollover_energy
        CALL      _rollover_energy   ;; 3 cycles
        MOVW      N:_energy+52, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+54, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3852    energy.Allph.active_exp = rollover_energy(energy.Allph.active_exp);
        MOVW      BC, N:_energy+58   ;; 1 cycle
        MOVW      AX, N:_energy+56   ;; 1 cycle
          CFI FunCall _rollover_energy
        CALL      _rollover_energy   ;; 3 cycles
        MOVW      N:_energy+56, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+58, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3853    energy.Allph.apparent_imp = rollover_energy(energy.Allph.apparent_imp);
        MOVW      BC, N:_energy+70   ;; 1 cycle
        MOVW      AX, N:_energy+68   ;; 1 cycle
          CFI FunCall _rollover_energy
        CALL      _rollover_energy   ;; 3 cycles
        MOVW      N:_energy+68, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+70, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3854    energy.Allph.apparent_exp = rollover_energy(energy.Allph.apparent_exp);
        MOVW      BC, N:_energy+74   ;; 1 cycle
        MOVW      AX, N:_energy+72   ;; 1 cycle
          CFI FunCall _rollover_energy
        CALL      _rollover_energy   ;; 3 cycles
        MOVW      N:_energy+72, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+74, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3855    energy.Allph.reactive_q1 = rollover_energy(energy.Allph.reactive_q1);
        MOVW      BC, N:_energy+78   ;; 1 cycle
        MOVW      AX, N:_energy+76   ;; 1 cycle
          CFI FunCall _rollover_energy
        CALL      _rollover_energy   ;; 3 cycles
        MOVW      N:_energy+76, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+78, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3856    energy.Allph.reactive_q2 = rollover_energy(energy.Allph.reactive_q2);
        MOVW      BC, N:_energy+82   ;; 1 cycle
        MOVW      AX, N:_energy+80   ;; 1 cycle
          CFI FunCall _rollover_energy
        CALL      _rollover_energy   ;; 3 cycles
        MOVW      N:_energy+80, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+82, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3857    energy.Allph.reactive_q3 = rollover_energy(energy.Allph.reactive_q3);
        MOVW      BC, N:_energy+86   ;; 1 cycle
        MOVW      AX, N:_energy+84   ;; 1 cycle
          CFI FunCall _rollover_energy
        CALL      _rollover_energy   ;; 3 cycles
        MOVW      N:_energy+84, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+86, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3858    energy.Allph.reactive_q4 = rollover_energy(energy.Allph.reactive_q4);
        MOVW      BC, N:_energy+90   ;; 1 cycle
        MOVW      AX, N:_energy+88   ;; 1 cycle
          CFI FunCall _rollover_energy
        CALL      _rollover_energy   ;; 3 cycles
        MOVW      N:_energy+88, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+90, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3859    energy.Allph.defraud_mag = rollover_energy(energy.Allph.defraud_mag);
        MOVW      BC, N:_energy+62   ;; 1 cycle
        MOVW      AX, N:_energy+60   ;; 1 cycle
          CFI FunCall _rollover_energy
        CALL      _rollover_energy   ;; 3 cycles
        MOVW      N:_energy+60, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+62, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3860    energy.Allph.fundamental = rollover_energy(energy.Allph.fundamental);
        MOVW      BC, N:_energy+66   ;; 1 cycle
        MOVW      AX, N:_energy+64   ;; 1 cycle
          CFI FunCall _rollover_energy
        CALL      _rollover_energy   ;; 3 cycles
        MOVW      N:_energy+64, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+66, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3861    energy.Allph.zkwh_imp = rollover_energy(energy.Allph.zkwh_imp);
        MOVW      BC, N:_energy+94   ;; 1 cycle
        MOVW      AX, N:_energy+92   ;; 1 cycle
          CFI FunCall _rollover_energy
        CALL      _rollover_energy   ;; 3 cycles
        MOVW      N:_energy+92, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+94, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3862    energy.Allph.zkwh_exp = rollover_energy(energy.Allph.zkwh_exp);
        MOVW      BC, N:_energy+98   ;; 1 cycle
        MOVW      AX, N:_energy+96   ;; 1 cycle
          CFI FunCall _rollover_energy
        CALL      _rollover_energy   ;; 3 cycles
        MOVW      N:_energy+96, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+98, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3863    energy.Allph.zkvah_imp = rollover_energy(energy.Allph.zkvah_imp);
        MOVW      BC, N:_energy+102  ;; 1 cycle
        MOVW      AX, N:_energy+100  ;; 1 cycle
          CFI FunCall _rollover_energy
        CALL      _rollover_energy   ;; 3 cycles
        MOVW      N:_energy+100, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+102, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3864    energy.Allph.zkvah_exp = rollover_energy(energy.Allph.zkvah_exp);
        MOVW      BC, N:_energy+106  ;; 1 cycle
        MOVW      AX, N:_energy+104  ;; 1 cycle
          CFI FunCall _rollover_energy
        CALL      _rollover_energy   ;; 3 cycles
        MOVW      N:_energy+104, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+106, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3865    energy.Allph.zkvarh_q1 = rollover_energy(energy.Allph.zkvarh_q1);
        MOVW      BC, N:_energy+110  ;; 1 cycle
        MOVW      AX, N:_energy+108  ;; 1 cycle
          CFI FunCall _rollover_energy
        CALL      _rollover_energy   ;; 3 cycles
        MOVW      N:_energy+108, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+110, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3866    energy.Allph.zkvarh_q2 = rollover_energy(energy.Allph.zkvarh_q2);
        MOVW      BC, N:_energy+114  ;; 1 cycle
        MOVW      AX, N:_energy+112  ;; 1 cycle
          CFI FunCall _rollover_energy
        CALL      _rollover_energy   ;; 3 cycles
        MOVW      N:_energy+112, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+114, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3867    energy.Allph.zkvarh_q3 = rollover_energy(energy.Allph.zkvarh_q3);
        MOVW      BC, N:_energy+118  ;; 1 cycle
        MOVW      AX, N:_energy+116  ;; 1 cycle
          CFI FunCall _rollover_energy
        CALL      _rollover_energy   ;; 3 cycles
        MOVW      N:_energy+116, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+118, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3868    energy.Allph.zkvarh_q4 = rollover_energy(energy.Allph.zkvarh_q4);
        MOVW      BC, N:_energy+122  ;; 1 cycle
        MOVW      AX, N:_energy+120  ;; 1 cycle
          CFI FunCall _rollover_energy
        CALL      _rollover_energy   ;; 3 cycles
        MOVW      N:_energy+120, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+122, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3869    
// 3870    energy.Rph.active_imp = rollover_energy(energy.Rph.active_imp);
        MOVW      BC, N:_energy+4    ;; 1 cycle
        MOVW      AX, N:_energy+2    ;; 1 cycle
          CFI FunCall _rollover_energy
        CALL      _rollover_energy   ;; 3 cycles
        MOVW      N:_energy+2, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+4, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3871    energy.Rph.active_exp = rollover_energy(energy.Rph.active_exp);
        MOVW      BC, N:_energy+8    ;; 1 cycle
        MOVW      AX, N:_energy+6    ;; 1 cycle
          CFI FunCall _rollover_energy
        CALL      _rollover_energy   ;; 3 cycles
        MOVW      N:_energy+6, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+8, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3872    energy.Rph.defraud_mag = rollover_energy(energy.Rph.defraud_mag);
        MOVW      BC, N:_energy+12   ;; 1 cycle
        MOVW      AX, N:_energy+10   ;; 1 cycle
          CFI FunCall _rollover_energy
        CALL      _rollover_energy   ;; 3 cycles
        MOVW      N:_energy+10, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+12, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3873    
// 3874    energy.Yph.active_imp = rollover_energy(energy.Yph.active_imp);
        MOVW      BC, N:_energy+18   ;; 1 cycle
        MOVW      AX, N:_energy+16   ;; 1 cycle
          CFI FunCall _rollover_energy
        CALL      _rollover_energy   ;; 3 cycles
        MOVW      N:_energy+16, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+18, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3875    energy.Yph.active_exp = rollover_energy(energy.Yph.active_exp);
        MOVW      BC, N:_energy+22   ;; 1 cycle
        MOVW      AX, N:_energy+20   ;; 1 cycle
          CFI FunCall _rollover_energy
        CALL      _rollover_energy   ;; 3 cycles
        MOVW      N:_energy+20, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+22, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3876    energy.Yph.defraud_mag = rollover_energy(energy.Yph.defraud_mag);
        MOVW      BC, N:_energy+26   ;; 1 cycle
        MOVW      AX, N:_energy+24   ;; 1 cycle
          CFI FunCall _rollover_energy
        CALL      _rollover_energy   ;; 3 cycles
        MOVW      N:_energy+24, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+26, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3877    
// 3878    energy.Bph.active_imp = rollover_energy(energy.Bph.active_imp);
        MOVW      BC, N:_energy+32   ;; 1 cycle
        MOVW      AX, N:_energy+30   ;; 1 cycle
          CFI FunCall _rollover_energy
        CALL      _rollover_energy   ;; 3 cycles
        MOVW      N:_energy+30, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+32, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3879    energy.Bph.active_exp = rollover_energy(energy.Bph.active_exp);
        MOVW      BC, N:_energy+36   ;; 1 cycle
        MOVW      AX, N:_energy+34   ;; 1 cycle
          CFI FunCall _rollover_energy
        CALL      _rollover_energy   ;; 3 cycles
        MOVW      N:_energy+34, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+36, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3880    energy.Bph.defraud_mag = rollover_energy(energy.Bph.defraud_mag); 
        MOVW      BC, N:_energy+40   ;; 1 cycle
        MOVW      AX, N:_energy+38   ;; 1 cycle
          CFI FunCall _rollover_energy
        CALL      _rollover_energy   ;; 3 cycles
        MOVW      N:_energy+38, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_energy+40, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3881    
// 3882    duplicate_total_apparent_energy = energy.Allph.apparent_imp ;
        MOVW      BC, N:_energy+70   ;; 1 cycle
        MOVW      AX, N:_energy+68   ;; 1 cycle
        MOVW      N:_duplicate_total_apparent_energy, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_duplicate_total_apparent_energy+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 3883 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock33
        ; ------------------------------------- Block: 255 cycles
        ; ------------------------------------- Total: 255 cycles

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock34 Using cfiCommon2
          CFI Function _rollover_energy
        CODE
// 3884 us32 rollover_energy(us32 energy)
// 3885 {
_rollover_energy:
        ; * Stack frame (at entry) *
        ; Param size: 0
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+6
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+8
        ; Auto size: 4
// 3886   if(energy >= ROLL_OVER_LIMIT)
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0xEE6B        ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 9 cycles
        CMPW      AX, #0x2800        ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??rollover_energy_0:
        BC        ??power_filter_438  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
// 3887   {
// 3888     energy = 0;
        MOVW      AX, SP             ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
// 3889     energy_rollover_f = 1;
        SET1      N:_flag_metro3.3   ;; 2 cycles
// 3890     energy_rollover_count++;
        INC       N:_energy_rollover_count  ;; 2 cycles
// 3891     eprom_read(0x07A0,0,PAGE_1,AUTO_CALC);
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x7A0         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
// 3892     opr_data[0] = energy_rollover_f;
        MOVW      HL, #LWRD(_flag_metro3)  ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOV1      CY, [HL].3         ;; 1 cycle
        ROLC      A, 0x1             ;; 1 cycle
        MOV       N:_opr_data, A     ;; 1 cycle
// 3893     opr_data[1] = energy_rollover_count;
        MOV       A, N:_energy_rollover_count  ;; 1 cycle
        MOV       N:_opr_data+1, A   ;; 1 cycle
// 3894     eprom_write(0x07A0,0,16,PAGE_1,AUTO_CALC);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #0x10          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x7A0         ;; 1 cycle
          CFI FunCall _eprom_write
        CALL      _eprom_write       ;; 3 cycles
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+8
        ; ------------------------------------- Block: 34 cycles
// 3895   }
// 3896   return(energy);
??power_filter_438:
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock34
        ; ------------------------------------- Block: 10 cycles
        ; ------------------------------------- Total: 58 cycles
// 3897 }

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock35 Using cfiCommon1
          CFI Function _power_filter
          CFI NoCalls
        CODE
// 3898 us32 power_filter(us32 power1, us8 counter)
// 3899 {
_power_filter:
        ; * Stack frame (at entry) *
        ; Param size: 0
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+6
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+8
        ; Auto size: 4
// 3900     static us8 count = 0;
// 3901     static us32 long_ret = 0;
// 3902     if(count == 0)
        CMP0      N:`power_filter::count`  ;; 1 cycle
        BNZ       ??power_filter_439  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
// 3903     {
// 3904         long_ret = power1;
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        MOVW      N:`power_filter::long_ret`, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:`power_filter::long_ret`+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
// 3905     }
// 3906     count++;
??power_filter_439:
        INC       N:`power_filter::count`  ;; 2 cycles
// 3907     if(count >= counter)
        MOV       A, N:`power_filter::count`  ;; 1 cycle
        CMP       A, E               ;; 1 cycle
        SKC                          ;; 1 cycle
        ; ------------------------------------- Block: 5 cycles
// 3908     {
// 3909         count = 0;
        MOV       N:`power_filter::count`, #0x0  ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
// 3910     }    
// 3911     return long_ret;
??power_filter_440:
        MOVW      BC, N:`power_filter::long_ret`+2  ;; 1 cycle
        MOVW      AX, N:`power_filter::long_ret`  ;; 1 cycle
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock35
        ; ------------------------------------- Block: 9 cycles
        ; ------------------------------------- Total: 29 cycles
// 3912 }

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
`power_filter::count`:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
`power_filter::long_ret`:
        DS 4

        SECTION `.constf`:FARCODE:REORDER:NOROOT(1)
        SECTION_TYPE SHT_PROGBITS, SHF_WRITE

        SECTION `.constf`:FARCODE:REORDER:NOROOT(1)
        SECTION_TYPE SHT_PROGBITS, SHF_WRITE

        SECTION `.constf`:FARCODE:REORDER:NOROOT(1)
        SECTION_TYPE SHT_PROGBITS, SHF_WRITE

        SECTION `.constf`:FARCODE:REORDER:NOROOT(1)
        SECTION_TYPE SHT_PROGBITS, SHF_WRITE

        SECTION `.constf`:FARCODE:REORDER:NOROOT(1)
        SECTION_TYPE SHT_PROGBITS, SHF_WRITE

        SECTION `.constf`:FARCODE:REORDER:NOROOT(1)
        SECTION_TYPE SHT_PROGBITS, SHF_WRITE

        SECTION `.constf`:FARCODE:REORDER:NOROOT(1)
        SECTION_TYPE SHT_PROGBITS, SHF_WRITE

        SECTION `.iar_vfe_header`:DATA:NOALLOC:NOROOT(1)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
        DC32 0

        END
// 
//  1'841 bytes in section .bss
//     21 bytes in section .bss.noinit   (abs)
//     68 bytes in section .data
//      2 bytes in section .sbss.noinit  (abs)
// 30'880 bytes in section .text
// 
// 30'880 bytes of CODE memory
//  1'577 bytes of DATA memory (+ 355 bytes shared)
//
//Errors: none
//Warnings: none
