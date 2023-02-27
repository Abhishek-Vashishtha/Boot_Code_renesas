///////////////////////////////////////////////////////////////////////////////
//
// IAR C/C++ Compiler V4.20.1.2260 for RL78               20/Dec/2020  22:38:15
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
//        BootCode\source_code\source_files\dlms.c
//    Command line       =
//        -f C:\Users\laptop\AppData\Local\Temp\EW3FD8.tmp ("D:\Dheeraj\New
//        folder\0. GDEV72 - BootCode\source_code\source_files\dlms.c" --core
//        s3 --code_model near --calling_convention v2 --near_const_location
//        ram -o "D:\Dheeraj\New folder\0. GDEV72 - BootCode\Debug\Obj"
//        --dlib_config "C:\Program Files\IAR Systems\Embedded Workbench
//        8.4\rl78\LIB\DLib_Config_Normal.h" --double=32 -e -On --no_cse
//        --no_unroll --no_inline --no_code_motion --no_tbaa --no_cross_call
//        --no_scheduling --no_clustering --debug -lA "D:\Dheeraj\New folder\0.
//        GDEV72 - BootCode\Debug\List" -I "D:\Dheeraj\New folder\0. GDEV72 -
//        BootCode\source_code\driver_files\" -I "D:\Dheeraj\New folder\0.
//        GDEV72 - BootCode\source_code\library_files\" -I "D:\Dheeraj\New
//        folder\0. GDEV72 - BootCode\source_code\misc_files\" -I
//        "D:\Dheeraj\New folder\0. GDEV72 -
//        BootCode\source_code\source_files\" --data_model near)
//    Locale             =  C
//    List file          =
//        D:\Dheeraj\New folder\0. GDEV72 - BootCode\Debug\List\dlms.s
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

        EXTERN _flag_eeprom_error
        EXTERN _Now
        EXTERN _opr_data
        EXTERN _cal_done_f
        EXTERN _MemoryStatus1
        EXTERN _MemoryStatus2
        EXTERN _OBJ_LIST
        EXTERN _MAX_BILL
        EXTERN _md_reset_count
        EXTERN _tpr
        EXTERN _md_type
        EXTERN _mdi_sel
        EXTERN _mdi_sel_ls
        EXTERN _bill_count
        EXTERN _TempTime
        EXTERN _METERING_MODE
        EXTERN _quadrant
        EXTERN _energy
        EXTERN _lsro_flag
        EXTERN _midnight_par_cnt
        EXTERN _midnight_roll_f
        EXTERN _COMPART_VOLTAGE_SIZE
        EXTERN _COMPART_VOLTAGE_ENTRIES
        EXTERN _COMPART_CURRENT_SIZE
        EXTERN _COMPART_CURRENT_ENTRIES
        EXTERN _COMPART_TRANSACTION_SIZE
        EXTERN _COMPART_TRANSACTION_ENTRIES
        EXTERN _COMPART_OTHERS_SIZE
        EXTERN _COMPART_OTHERS_ENTRIES
        EXTERN _TOP_RESTORE_REQ
        EXTERN _COMPART_NONROLLOVER_SIZE
        EXTERN _COMPART_NONROLLOVER_ENTRIES
        EXTERN _COMPART_POWERFAIL_ENTRIES
        EXTERN _active_calendar
        EXTERN _bill_hr
        EXTERN _bill_min
        EXTERN _bill_date
        EXTERN _KVAH_SNAP
        EXTERN _TOD_energy_config
        EXTERN _D_KVARH_REQ
        EXTERN _CUM_MAX_DEMAND
        EXTERN _FUENERGY_REQ
        EXTERN _BILLTPR_CNT
        EXTERN _MDRESET_TYPE_CONFIG
        EXTERN ?L_VSWITCH_L10
        EXTERN ?MEMCPY_NEAR
        EXTERN ?MEMCPY_SMALL_NEAR
        EXTERN ?SI_DIV_L02
        EXTERN ?SI_MOD_L02
        EXTERN ?UC_MOD_L01
        EXTERN _COMPART_CURRENT_END_ADD
        EXTERN _COMPART_CURRENT_START_ADD
        EXTERN _COMPART_NONROLLOVER_END_ADD
        EXTERN _COMPART_NONROLLOVER_START_ADD
        EXTERN _COMPART_OTHERS_END_ADD
        EXTERN _COMPART_OTHERS_START_ADD
        EXTERN _COMPART_POWERFAIL_END_ADD
        EXTERN _COMPART_POWERFAIL_START_ADD
        EXTERN _COMPART_TRANSACTION_END_ADD
        EXTERN _COMPART_TRANSACTION_START_ADD
        EXTERN _COMPART_VOLTAGE_END_ADD
        EXTERN _COMPART_VOLTAGE_START_ADD
        EXTERN _Sel_DailyLoadsurvey_buffer
        EXTERN _Sel_Loadsurvey_buffer
        EXTERN _Start_Info2
        EXTERN _Tarrif_script
        EXTERN __LLU2L
        EXTERN _angle
        EXTERN _app_con
        EXTERN _array
        EXTERN _asso_status
        EXTERN _auth_name
        EXTERN _bcd_to_decimal
        EXTERN _bill_buffer
        EXTERN _bill_profile_parameter_cap_obj
        EXTERN _bill_profile_parameter_scaler_buffer
        EXTERN _bill_profile_parameter_scaler_cap_obj
        EXTERN _blockload_survey_parameter_scaler_buffer
        EXTERN _blockload_survey_parameter_scaler_cap_obj
        EXTERN _buffer_instantaneous_parameter
        EXTERN _char_array_into_time4
        EXTERN _char_array_into_time5_sec
        EXTERN _cum_max_demand_kva
        EXTERN _cum_max_demand_kw
        EXTERN _curr
        EXTERN _current_event_capture_obj
        EXTERN _current_rating
        EXTERN _dailyload_profile_parameter_cap_obj
        EXTERN _dailyload_profile_parameter_scaler_buffer
        EXTERN _dailyload_profile_parameter_scaler_cap_obj
        EXTERN _date_time
        EXTERN _day_profile
        EXTERN _debug_event_capture_obj
        EXTERN _demand
        EXTERN _enum_d2
        EXTERN _eprom_read
        EXTERN _event_log_profile_scaler_buffer
        EXTERN _event_log_profile_scaler_cap_obj
        EXTERN _fill_0b
        EXTERN _fill_0d
        EXTERN _fill_firmware_version
        EXTERN _fill_manufacturer_name
        EXTERN _fill_meter_category
        EXTERN _fill_oprzero
        EXTERN _fill_pcb_firm_ver_calib_status
        EXTERN _fill_rtc_calib
        EXTERN _fill_utility_id
        EXTERN _fill_yr_of_manufacture
        EXTERN _flag1
        EXTERN _flag_metro1
        EXTERN _flag_quadrant
        EXTERN _flag_rtc2
        EXTERN _freq
        EXTERN _get_hr_energy
        EXTERN _hardware_testing_status
        EXTERN _info_l
        EXTERN _instantaneous_parameter_cap_obj
        EXTERN _instantaneous_parameter_scaler_buffer
        EXTERN _instantaneous_parameter_scaler_cap_obj
        EXTERN _int_into_char_array
        EXTERN _integer8
        EXTERN _load_survey_cnt
        EXTERN _load_survey_parameter_cap_obj
        EXTERN _log_name2
        EXTERN _logical_device_name
        EXTERN _long_int
        EXTERN _long_into_char_array3
        EXTERN _long_into_char_array4
        EXTERN _ls_count_dlms
        EXTERN _ls_count_local
        EXTERN _name_plate_buffer
        EXTERN _name_plate_profile_capture_obj
        EXTERN _non_rollover_event_capture_obj
        EXTERN _object_list
        EXTERN _octet_s
        EXTERN _other_event_capture_obj
        EXTERN _pf
        EXTERN _ph_ph
        EXTERN _power
        EXTERN _power_event_capture_obj
        EXTERN _power_off_min
        EXTERN _power_on_min
        EXTERN _profile_sel
        EXTERN _sap_assg_list
        EXTERN _save_tod_data
        EXTERN _sca_unit
        EXTERN _send_data
        EXTERN _send_type_multi
        EXTERN _seq_no_transaction
        EXTERN _signed_integer
        EXTERN _sort_object
        EXTERN _sort_object1
        EXTERN _sr_no_ascii
        EXTERN _structure
        EXTERN _tamper_compart
        EXTERN _tamper_instant_status
        EXTERN _temp_us32
        EXTERN _tpr_fill
        EXTERN _transaction_event_capture_obj
        EXTERN _unsigned8
        EXTERN _val_1byt
        EXTERN _val_2byt
        EXTERN _val_2byt2
        EXTERN _val_4byt2
        EXTERN _val_signed_4byt2
        EXTERN _vol
        EXTERN _voltage_event_capture_obj
        EXTERN _xdlms_type

        PUBLIC _CTR
        PUBLIC _Cntr_2Min
        PUBLIC _Data_block
        PUBLIC _FcsFlag
        PUBLIC _Format_type
        PUBLIC _PTR
        PUBLIC _UintLoadSurptr
        PUBLIC _UintLoadSurptr1
        PUBLIC _access_selector
        PUBLIC _analyse_cal_pkt_flag
        PUBLIC _ass_ser
        PUBLIC _asserr_flag
        PUBLIC _asso0_flag
        PUBLIC _asso1_flag
        PUBLIC _asso2_flag
        PUBLIC _asso3_flag
        PUBLIC _asso4_flag
        PUBLIC _assoG_flag
        PUBLIC _assresult_flag
        PUBLIC _attribute_id
        PUBLIC _aut_pswd
        PUBLIC _aut_pswd1
        PUBLIC _aut_pswd1_1
        PUBLIC _aut_pswd1_2
        PUBLIC _aut_pswd1_default
        PUBLIC _aut_pswd2
        PUBLIC _aut_pswd_default
        PUBLIC _auth_fill
        PUBLIC _battery_timeout
        PUBLIC _block_no
        PUBLIC _block_size
        PUBLIC _buffer_first_not_fill_f
        PUBLIC _buffer_scaler_filler
        PUBLIC _byte_cont
        PUBLIC _capture_objects_filler
        PUBLIC _char_array
        PUBLIC _char_array_ptr
        PUBLIC _class_id
        PUBLIC _cli_id
        PUBLIC _client_add
        PUBLIC _compart1
        PUBLIC _compartment3
        PUBLIC _compartment6
        PUBLIC _conf_blk
        PUBLIC _conf_err_flag
        PUBLIC _conf_ser_flag
        PUBLIC _conf_serror_flag
        PUBLIC _conf_type_flag
        PUBLIC _cont_field
        PUBLIC _conv
        PUBLIC _cosem_flag
        PUBLIC _data_array
        PUBLIC _decerr_flag
        PUBLIC _disp_test_pkt_flag
        PUBLIC _dlms_address
        PUBLIC _dlms_bill_no
        PUBLIC _dlms_bill_start
        PUBLIC _dlms_bill_stop
        PUBLIC _dlms_firm_ver
        PUBLIC _dlms_rece_flag
        PUBLIC _dls_count_dlms
        PUBLIC _element_filled
        PUBLIC _err_buf
        PUBLIC _err_flag
        PUBLIC _firm_rev
        PUBLIC _flag_communication
        PUBLIC _flag_hdlc1
        PUBLIC _flag_hdlc2
        PUBLIC _flag_hdlc3
        PUBLIC _flag_hdlc4
        PUBLIC _flag_optical
        PUBLIC _flag_rj45
        PUBLIC _four_pass_f
        PUBLIC _frame_type
        PUBLIC _frm_rcv_flg
        PUBLIC _from_cntr
        PUBLIC _from_days
        PUBLIC _from_ptr
        PUBLIC _from_val
        PUBLIC _get_resp
        PUBLIC _get_resp1
        PUBLIC _global_i
        PUBLIC _gsm_pkt_flag
        PUBLIC _gsm_signal_f
        PUBLIC _gsm_signal_strength
        PUBLIC _gsm_tr_f
        PUBLIC _i_dlms
        PUBLIC _info
        PUBLIC _info2
        PUBLIC _info_send
        PUBLIC _info_sended
        PUBLIC _info_sended_old
        PUBLIC _info_total
        PUBLIC _infore_flag
        PUBLIC _infose_flag
        PUBLIC _interframe_timeout
        PUBLIC _invo_prio
        PUBLIC _k
        PUBLIC _last_block
        PUBLIC _length
        PUBLIC _long_data
        PUBLIC _long_data1
        PUBLIC _max_info_rec
        PUBLIC _max_info_tra
        PUBLIC _max_win_rec
        PUBLIC _max_win_tra
        PUBLIC _meter_address
        PUBLIC _meter_type
        PUBLIC _multi_filling_f
        PUBLIC _multi_resp
        PUBLIC _no_bytes
        PUBLIC _no_obj
        PUBLIC _nrm_flag
        PUBLIC _obis_a
        PUBLIC _obis_b
        PUBLIC _obis_c
        PUBLIC _obis_code
        PUBLIC _obis_d
        PUBLIC _obis_e
        PUBLIC _obis_f
        PUBLIC _obis_short
        PUBLIC _obis_short_cal
        PUBLIC _one_byte_add_f
        PUBLIC _optical_f
        PUBLIC _p_fbit
        PUBLIC _packet_len
        PUBLIC _phdevice_add
        PUBLIC _phy_id
        PUBLIC _rcv_buf
        PUBLIC _rcv_buf1
        PUBLIC _rcv_cnt
        PUBLIC _rcv_cnt1
        PUBLIC _read_rtc_cnt
        PUBLIC _rec_flag
        PUBLIC _recv_buffer
        PUBLIC _req_cnt
        PUBLIC _req_cnt1
        PUBLIC _req_typ
        PUBLIC _rj45_disc_cnt
        PUBLIC _rj45_dm_buf
        PUBLIC _rj_disc_cnt
        PUBLIC _rj_disc_f
        PUBLIC _rj_disc_f2
        PUBLIC _rj_dm_buf
        PUBLIC _rj_f
        PUBLIC _rrr_c
        PUBLIC _rrr_c1
        PUBLIC _rrr_s
        PUBLIC _rx_timeout
        PUBLIC _scalar_cur
        PUBLIC _scalar_energy
        PUBLIC _scalar_vol
        PUBLIC _seg_flag
        PUBLIC _seg_flagsd
        PUBLIC _seg_type
        PUBLIC _sel_access_flag
        PUBLIC _sel_obj
        PUBLIC _sel_obj_tamper
        PUBLIC _selective_values_byte
        PUBLIC _send_type_multi_f
        PUBLIC _serial_no
        PUBLIC _server_add
        PUBLIC _server_lowerhi
        PUBLIC _server_lowerlow
        PUBLIC _server_upperhi
        PUBLIC _server_upperlow
        PUBLIC _sss_c
        PUBLIC _sss_c1
        PUBLIC _sss_s
        PUBLIC _tamper_data
        PUBLIC _tamper_status
        PUBLIC _to_cntr
        PUBLIC _to_days
        PUBLIC _to_ptr
        PUBLIC _to_val
        PUBLIC _trn_buf
        PUBLIC _trn_cnt
        PUBLIC _trn_cnt1
        PUBLIC _u8Lock_multi_transfer
        PUBLIC _utility_code
        PUBLIC _utility_id
        PUBLIC _y
        
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
        
// D:\Dheeraj\New folder\0. GDEV72 - BootCode\source_code\source_files\dlms.c
//    1 #include "dlms.h"

        SECTION `.data`:DATA:REORDER:NOROOT(1)
//    2 const uint8_t firm_rev[2]= {0xE0, 0x20};
_firm_rev:
        DATA8
        DB 224, 32

        SECTION `.data`:DATA:REORDER:NOROOT(1)
//    3 const uint8_t utility_code[4]= {'G', 'S', 'T', 'G'};
_utility_code:
        DATA8
        DB 71, 83, 84, 71

        SECTION `.data`:DATA:REORDER:NOROOT(1)
//    4 const uint8_t aut_pswd_default[8]= {0x31, 0x41, 0x32, 0x42, 0x33, 0x43, 0x34, 0x44};
_aut_pswd_default:
        DATA8
        DB 49, 65, 50, 66, 51, 67, 52, 68

        SECTION `.data`:DATA:REORDER:NOROOT(1)
//    5 const uint8_t aut_pswd1_default[16]= {'d', 'l', 'm', 's', 'p', 'a', 's', 's', 'w', 'o', 'r', 'd', '1', '2', '3', '4'};
_aut_pswd1_default:
        DATA8
        DB 100, 108, 109, 115, 112, 97, 115, 115, 119, 111, 114, 100, 49, 50
        DB 51, 52
//    6 
//    7 
//    8 /* Variables */

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//    9 uint8_t dlms_firm_ver[8];
_dlms_firm_ver:
        DS 8

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   10 uint8_t gsm_signal_f,gsm_tr_f,gsm_pkt_flag;
_gsm_signal_f:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_gsm_tr_f:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_gsm_pkt_flag:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   11 uint8_t gsm_signal_strength[2];
_gsm_signal_strength:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   12 uint8_t dlms_bill_start, dlms_bill_stop, dlms_bill_no;
_dlms_bill_start:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_dlms_bill_stop:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_dlms_bill_no:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   13 uint8_t dlms_rece_flag= 0, rec_flag= 0, err_flag= 0;
_dlms_rece_flag:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_rec_flag:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_err_flag:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   14 uint8_t server_upperlow;
_server_upperlow:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   15 uint8_t server_lowerlow;
_server_lowerlow:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   16 uint8_t server_upperhi;
_server_upperhi:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   17 uint8_t server_lowerhi;
_server_lowerhi:
        DS 1
//   18 uint8_t client_add;

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   19 uint8_t one_byte_add_f;
_one_byte_add_f:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   20 uint16_t phdevice_add;
_phdevice_add:
        DS 2
//   21 uint16_t byte_cont;
//   22 uint8_t seg_type;
//   23 uint8_t length;
//   24 uint16_t rrr_c;
//   25 uint16_t rrr_s;
//   26 uint16_t rrr_c1;
//   27 uint16_t sss_c;
//   28 uint16_t sss_c1;
//   29 uint16_t sss_s;
//   30 uint8_t cont_field;
//   31 uint16_t frame_type;

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   32 uint16_t meter_address;
_meter_address:
        DS 2
//   33 uint16_t info_total; /* int */

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   34 uint16_t p_fbit;
_p_fbit:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   35 uint8_t info[DLMS_MAX_BUFF_SIZE];
_info:
        DS 512

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   36 uint8_t infore_flag;
_infore_flag:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   37 uint8_t infose_flag;
_infose_flag:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   38 uint8_t decerr_flag;
_decerr_flag:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   39 uint8_t nrm_flag;
_nrm_flag:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   40 uint8_t seg_flag;
_seg_flag:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   41 uint8_t seg_flagsd;
_seg_flagsd:
        DS 1
//   42 uint16_t max_win_rec;
//   43 uint16_t max_win_tra;
//   44 uint16_t max_info_rec;
//   45 uint16_t max_info_tra;
//   46 uint8_t req_typ;

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   47 uint8_t asserr_flag;
_asserr_flag:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   48 uint8_t ass_ser;
_ass_ser:
        DS 1

        SECTION `.data`:DATA:REORDER:NOROOT(1)
//   49 uint8_t conf_blk[3]= {0x00, 0x00, 0x10};
_conf_blk:
        DATA8
        DB 0, 0, 16, 0

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   50 uint8_t optical_f, rj_f;
_optical_f:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_rj_f:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   51 uint8_t rj_disc_f, rj_disc_f2;
_rj_disc_f:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_rj_disc_f2:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   52 uint8_t assresult_flag;
_assresult_flag:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   53 uint8_t conf_ser_flag;
_conf_ser_flag:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   54 uint8_t conf_err_flag;
_conf_err_flag:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   55 uint8_t conf_type_flag;
_conf_type_flag:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   56 uint8_t conf_serror_flag;
_conf_serror_flag:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   57 uint8_t cosem_flag;
_cosem_flag:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   58 uint8_t obis_code[6];
_obis_code:
        DS 6

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   59 uint8_t attribute_id;
_attribute_id:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   60 uint8_t invo_prio;
_invo_prio:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   61 uint16_t class_id;
_class_id:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   62 uint16_t long_data;
_long_data:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   63 uint16_t long_data1;
_long_data1:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   64 uint8_t asso0_flag;
_asso0_flag:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   65 uint8_t asso1_flag;
_asso1_flag:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   66 uint8_t asso2_flag;
_asso2_flag:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   67 uint8_t asso3_flag;
_asso3_flag:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   68 uint8_t asso4_flag;
_asso4_flag:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   69 uint8_t assoG_flag;
_assoG_flag:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   70 uint16_t phy_id;
_phy_id:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   71 uint16_t cli_id;
_cli_id:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   72 uint8_t y= 0;
_y:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   73 uint16_t k= 0;
_k:
        DS 2
//   74 /* uint16_t j=0; */

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   75 uint16_t Cntr_2Min= 0;
_Cntr_2Min:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   76 uint16_t FcsFlag= 0;
_FcsFlag:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   77 uint8_t frm_rcv_flg= 0;
_frm_rcv_flg:
        DS 1
//   78 uint8_t data_array[10];

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   79 uint8_t recv_buffer[8];
_recv_buffer:
        DS 8

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   80 uint16_t rcv_cnt= 0;
_rcv_cnt:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   81 uint16_t rcv_cnt1= 0;
_rcv_cnt1:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   82 uint8_t rcv_buf[DLMS_MAX_BUFF_SIZE + 14]; /* update by dinesh */
_rcv_buf:
        DS 526
//   83 uint8_t rcv_buf1[150];                    /* this is used in simultaneous communication- */

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   84 uint8_t err_buf= 0x00;
_err_buf:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   85 uint8_t trn_buf[DLMS_MAX_BUFF_SIZE + 14]; /* update by dinesh */
_trn_buf:
        DS 526

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   86 uint16_t trn_cnt= 0;
_trn_cnt:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   87 uint16_t req_cnt= 0;
_req_cnt:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   88 uint16_t trn_cnt1= 0;
_trn_cnt1:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   89 uint16_t req_cnt1= 0;
_req_cnt1:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   90 uint8_t rj_dm_buf[12];
_rj_dm_buf:
        DS 12
//   91 uint16_t interframe_timeout;

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   92 uint8_t sel_access_flag, access_selector, from_val[5], to_val[5], sel_obj[15], no_obj, to_cntr, from_cntr, no_bytes;
_sel_access_flag:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_access_selector:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_from_val:
        DS 6

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_to_val:
        DS 6

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_sel_obj:
        DS 16

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_no_obj:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_to_cntr:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_from_cntr:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_no_bytes:
        DS 1
//   93 

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   94 uint16_t to_days, from_days;
_to_days:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_from_days:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   95 uint8_t rj_disc_cnt;
_rj_disc_cnt:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   96 uint8_t read_rtc_cnt;
_read_rtc_cnt:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   97 uint16_t tamper_data;
_tamper_data:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   98 uint16_t compart1;
_compart1:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   99 uint8_t aut_pswd[8];
_aut_pswd:
        DS 8

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  100 uint8_t aut_pswd1[16];
_aut_pswd1:
        DS 16

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  101 uint16_t block_no;
_block_no:
        DS 2
//  102 uint16_t info_sended;
//  103 uint16_t info_send;
//  104 uint16_t packet_len;
//  105 uint8_t Format_type;
//  106 uint8_t server_add[4];

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//  107 uint8_t send_type_multi_f, multi_filling_f, buffer_first_not_fill_f;
_send_type_multi_f:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_multi_filling_f:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_buffer_first_not_fill_f:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  108 uint16_t element_filled;
_element_filled:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//  109 uint8_t u8Lock_multi_transfer, multi_resp;
_u8Lock_multi_transfer:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_multi_resp:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  110 uint32_t obis_short;
_obis_short:
        DS 4

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  111 uint16_t i_dlms;
_i_dlms:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//  112 uint8_t selective_values_byte;
_selective_values_byte:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  113 uint16_t dlms_address;
_dlms_address:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  114 uint16_t dls_count_dlms, UintLoadSurptr1;
_dls_count_dlms:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_UintLoadSurptr1:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  115 uint16_t UintLoadSurptr;
_UintLoadSurptr:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  116 us8 rj45_dm_buf[12];
_rj45_dm_buf:
        DS 12
//  117 us16 rcv_cnt,rcv_cnt1,trn_cnt,trn_cnt1,req_cnt,req_cnt1;

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  118 us16 interframe_timeout,Cntr_2Min;
_interframe_timeout:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  119 us16 rrr_c,rrr_s,rrr_c1,sss_c,sss_c1,sss_s;
_rrr_c:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_rrr_s:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_rrr_c1:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_sss_c:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_sss_c1:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_sss_s:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  120 us8 rcv_buf1[150];                    /* this is used in simultaneous communication- */
_rcv_buf1:
        DS 150
//  121 us8 rx_timeout,battery_timeout;

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  122 us8 data_array[10];
_data_array:
        DS 10

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//  123 us8 rj45_disc_cnt;
_rj45_disc_cnt:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//  124 us8 cont_field;
_cont_field:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  125 us16 frame_type;
_frame_type:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//  126 us8 client_add,req_typ;
_client_add:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_req_typ:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  127 us8 server_add[4];
_server_add:
        DS 4

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//  128 us8 length,seg_type;
_length:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_seg_type:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  129 us16 info_sended,info_sended_old,info_total,k,byte_cont,max_info_rec,max_info_tra,max_win_rec,max_win_tra,info_send;
_info_sended:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_info_sended_old:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_info_total:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_byte_cont:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_max_info_rec:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_max_info_tra:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_max_win_rec:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_max_win_tra:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_info_send:
        DS 2
//  130 us8 conf_blk[3];

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  131 us16 packet_len;
_packet_len:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//  132 us8 Format_type;
_Format_type:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  133 uint8_t char_array[4];
_char_array:
        DS 4

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  134 uint16_t PTR;
_PTR:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  135 uint16_t CTR;
_CTR:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  136 uint8_t *char_array_ptr;
_char_array_ptr:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//  137 uint8_t scalar_energy, scalar_cur, scalar_vol;
_scalar_energy:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_scalar_cur:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_scalar_vol:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//  138 us8 rx_timeout,battery_timeout;
_rx_timeout:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_battery_timeout:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//  139 us8 analyse_cal_pkt_flag,disp_test_pkt_flag;
_analyse_cal_pkt_flag:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_disp_test_pkt_flag:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//  140 uint8_t last_block;
_last_block:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  141 us16 global_i;
_global_i:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  142 uint16_t from_ptr, to_ptr;
_from_ptr:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_to_ptr:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  143 us8 sel_obj_tamper[13];  
_sel_obj_tamper:
        DS 14

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  144 uint8_t serial_no[13];
_serial_no:
        DS 14

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  145 uint8_t meter_type[2];
_meter_type:
        DS 2

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  146 uint8_t utility_id[4];
_utility_id:
        DS 4

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  147 uint8_t info2[200];
_info2:
        DS 200

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//  148 uint8_t four_pass_f;
_four_pass_f:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//  149 uint8_t block_size;
_block_size:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  150 uint8_t conv[4];
_conv:
        DS 4

        SECTION `.data`:DATA:REORDER:NOROOT(0)
//  151 uint8_t Data_block= 0x01;
_Data_block:
        DATA8
        DB 1

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//  152 uint8_t aut_pswd1_1[16];
_aut_pswd1_1:
        DS 16

        SECTION `.data`:DATA:REORDER:NOROOT(1)
//  153 uint8_t aut_pswd1_2[16]= {0x5f, 0x5f, 0x44, 0x4c, 0x4d, 0x53, 0x2d, 0x45, 0x78, 0x70, 0x6c, 0x6f, 0x72, 0x65, 0x72, 0x5f};
_aut_pswd1_2:
        DATA8
        DB 95, 95, 68, 76, 77, 83, 45, 69, 120, 112, 108, 111, 114, 101, 114
        DB 95
//  154 /* {'i','j','k','l','m','n','o','p','q','r','s','t','u','v','w','x'}; */

        SECTION `.data`:DATA:REORDER:NOROOT(1)
//  155 us8 const aut_pswd2[8]= {'1','A','2','B','3','D','D','D'};
_aut_pswd2:
        DATA8
        DB 49, 65, 50, 66, 51, 68, 68, 68

        SECTION `.data`:DATA:REORDER:NOROOT(1)
//  156 us8 const auth_fill[]= {0x00, 0x02, 0x07, 0x11, 0x02, 0x11, 0x10, 0x12, 0x02, 0xf4, 0x11, 0x05, 0x11, 0x08};
_auth_fill:
        DATA8
        DB 0, 2, 7, 17, 2, 17, 16, 18, 2, 244, 17, 5, 17, 8

        SECTION `.data`:DATA:REORDER:NOROOT(0)
        SECTION_TYPE SHT_PROGBITS, 0
??get_resp1_0:
        DATA16
        DW        4
        DW        LWRD(??get_resp1_28)
        DW        LWRD(??get_resp1_24)
        DW        LWRD(??get_resp1_25)
        DW        LWRD(??get_resp1_26)
        DW        LWRD(??get_resp1_27)
        DATA32
        DD        2752512
        DD        6291712
        DD        23101449
        DD        23101458

        SECTION `.data`:DATA:REORDER:NOROOT(0)
        SECTION_TYPE SHT_PROGBITS, 0
??get_resp1_1:
        DATA16
        DW        10
        DW        LWRD(??get_resp1_38)
        DW        LWRD(??get_resp1_29)
        DW        LWRD(??get_resp1_30)
        DW        LWRD(??compartment3_29)
        DW        LWRD(??get_resp1_37)
        DW        LWRD(??get_resp1_31)
        DW        LWRD(??get_resp1_32)
        DW        LWRD(??get_resp1_33)
        DW        LWRD(??get_resp1_36)
        DW        LWRD(??get_resp1_34)
        DW        LWRD(??get_resp1_35)
        DATA32
        DD        2752512
        DD        6291712
        DD        6324736
        DD        6324992
        DD        23101449
        DD        23101454
        DD        23101457
        DD        23101458
        DD        23101500
        DD        23101502

        SECTION `.data`:DATA:REORDER:NOROOT(0)
        SECTION_TYPE SHT_PROGBITS, 0
??get_resp_0:
        DATA16
        DW        28
        DW        LWRD(??get_resp_43)
        DW        LWRD(??get_resp_24)
        DW        LWRD(??get_resp_15)
        DW        LWRD(??get_resp_16)
        DW        LWRD(??get_resp_23)
        DW        LWRD(??get_resp_26)
        DW        LWRD(??get_resp_27)
        DW        LWRD(??get_resp_28)
        DW        LWRD(??get_resp_17)
        DW        LWRD(??get_resp_29)
        DW        LWRD(??get_resp_33)
        DW        LWRD(??get_resp_25)
        DW        LWRD(??get_resp_22)
        DW        LWRD(??get_resp_36)
        DW        LWRD(??get_resp_37)
        DW        LWRD(??get_resp_38)
        DW        LWRD(??get_resp_39)
        DW        LWRD(??get_resp_40)
        DW        LWRD(??get_resp_41)
        DW        LWRD(??get_resp_42)
        DW        LWRD(??get_resp_30)
        DW        LWRD(??get_resp_31)
        DW        LWRD(??get_resp_32)
        DW        LWRD(??get_resp_34)
        DW        LWRD(??get_resp_35)
        DW        LWRD(??get_resp_18)
        DW        LWRD(??get_resp_19)
        DW        LWRD(??get_resp_20)
        DW        LWRD(??get_resp_21)
        DATA32
        DD        256
        DD        257
        DD        2752512
        DD        6183680
        DD        6183689
        DD        6183691
        DD        6183692
        DD        6291712
        DD        6291713
        DD        6291716
        DD        6291968
        DD        6293248
        DD        6294272
        DD        6294273
        DD        6294274
        DD        6294275
        DD        6294276
        DD        6294277
        DD        6294371
        DD        16777728
        DD        16778242
        DD        16778243
        DD        16779264
        DD        16779268
        DD        23101449
        DD        23101454
        DD        23101457
        DD        23101458

        SECTION `.data`:DATA:REORDER:NOROOT(0)
        SECTION_TYPE SHT_PROGBITS, 0
??get_resp_1:
        DATA16
        DW        54
        DW        LWRD(??get_resp_98)
        DW        LWRD(??get_resp_44)
        DW        LWRD(??get_resp_75)
        DW        LWRD(??get_resp_76)
        DW        LWRD(??get_resp_85)
        DW        LWRD(??get_resp_63)
        DW        LWRD(??get_resp_77)
        DW        LWRD(??get_resp_78)
        DW        LWRD(??get_resp_67)
        DW        LWRD(??get_resp_79)
        DW        LWRD(??get_resp_80)
        DW        LWRD(??get_resp_81)
        DW        LWRD(??get_resp_82)
        DW        LWRD(??get_resp_86)
        DW        LWRD(??get_resp_71)
        DW        LWRD(??get_resp_83)
        DW        LWRD(??get_resp_84)
        DW        LWRD(??get_resp_55)
        DW        LWRD(??get_resp_59)
        DW        LWRD(??get_resp_64)
        DW        LWRD(??get_resp_68)
        DW        LWRD(??get_resp_72)
        DW        LWRD(??get_resp_45)
        DW        LWRD(??get_resp_49)
        DW        LWRD(??get_resp_56)
        DW        LWRD(??get_resp_60)
        DW        LWRD(??get_resp_65)
        DW        LWRD(??get_resp_69)
        DW        LWRD(??get_resp_73)
        DW        LWRD(??get_resp_46)
        DW        LWRD(??get_resp_50)
        DW        LWRD(??get_resp_57)
        DW        LWRD(??get_resp_61)
        DW        LWRD(??get_resp_66)
        DW        LWRD(??get_resp_70)
        DW        LWRD(??get_resp_74)
        DW        LWRD(??get_resp_47)
        DW        LWRD(??get_resp_51)
        DW        LWRD(??get_resp_58)
        DW        LWRD(??get_resp_62)
        DW        LWRD(??get_resp_95)
        DW        LWRD(??get_resp_96)
        DW        LWRD(??get_resp_97)
        DW        LWRD(??get_resp_92)
        DW        LWRD(??get_resp_93)
        DW        LWRD(??get_resp_94)
        DW        LWRD(??get_resp_48)
        DW        LWRD(??get_resp_52)
        DW        LWRD(??get_resp_53)
        DW        LWRD(??get_resp_54)
        DW        LWRD(??get_resp_87)
        DW        LWRD(??get_resp_88)
        DW        LWRD(??get_resp_89)
        DW        LWRD(??get_resp_90)
        DW        LWRD(??get_resp_91)
        DATA32
        DD        258
        DD        6183688
        DD        6183693
        DD        16843264
        DD        16844544
        DD        16844800
        DD        16910336
        DD        16975616
        DD        17106944
        DD        17172480
        DD        17238016
        DD        17303552
        DD        17367552
        DD        17368832
        DD        17369088
        DD        17434624
        DD        17630976
        DD        17696512
        DD        18155264
        DD        18286336
        DD        18679552
        DD        18810624
        DD        18876160
        DD        18941696
        DD        19007232
        DD        19465984
        DD        19597056
        DD        19990272
        DD        20121344
        DD        20186880
        DD        20252416
        DD        20317952
        DD        20776704
        DD        20907776
        DD        21300992
        DD        21432064
        DD        21497600
        DD        21563136
        DD        21628672
        DD        22087425
        DD        22087426
        DD        22087436
        DD        22087464
        DD        22087475
        DD        22087486
        DD        22742784
        DD        24905472
        DD        24971008
        DD        25036544
        DD        25167872
        DD        26085376
        DD        26150912
        DD        26216448
        DD        26281984

        SECTION `.data`:DATA:REORDER:NOROOT(0)
        SECTION_TYPE SHT_PROGBITS, 0
??get_resp_2:
        DATA16
        DW        54
        DW        LWRD(??get_resp_115)
        DW        LWRD(??get_resp_99)
        DW        LWRD(??get_resp_107)
        DW        LWRD(??get_resp_107)
        DW        LWRD(??get_resp_104)
        DW        LWRD(??get_resp_104)
        DW        LWRD(??get_resp_110)
        DW        LWRD(??get_resp_110)
        DW        LWRD(??get_resp_105)
        DW        LWRD(??get_resp_108)
        DW        LWRD(??get_resp_108)
        DW        LWRD(??get_resp_108)
        DW        LWRD(??get_resp_108)
        DW        LWRD(??get_resp_106)
        DW        LWRD(??get_resp_106)
        DW        LWRD(??get_resp_109)
        DW        LWRD(??get_resp_109)
        DW        LWRD(??get_resp_102)
        DW        LWRD(??get_resp_103)
        DW        LWRD(??get_resp_104)
        DW        LWRD(??get_resp_105)
        DW        LWRD(??get_resp_106)
        DW        LWRD(??get_resp_100)
        DW        LWRD(??get_resp_101)
        DW        LWRD(??get_resp_102)
        DW        LWRD(??get_resp_103)
        DW        LWRD(??get_resp_104)
        DW        LWRD(??get_resp_105)
        DW        LWRD(??get_resp_106)
        DW        LWRD(??get_resp_100)
        DW        LWRD(??get_resp_101)
        DW        LWRD(??get_resp_102)
        DW        LWRD(??get_resp_103)
        DW        LWRD(??get_resp_104)
        DW        LWRD(??get_resp_105)
        DW        LWRD(??get_resp_106)
        DW        LWRD(??get_resp_100)
        DW        LWRD(??get_resp_101)
        DW        LWRD(??get_resp_102)
        DW        LWRD(??get_resp_103)
        DW        LWRD(??get_resp_114)
        DW        LWRD(??get_resp_114)
        DW        LWRD(??get_resp_114)
        DW        LWRD(??get_resp_114)
        DW        LWRD(??get_resp_114)
        DW        LWRD(??get_resp_114)
        DW        LWRD(??get_resp_100)
        DW        LWRD(??get_resp_101)
        DW        LWRD(??get_resp_101)
        DW        LWRD(??get_resp_101)
        DW        LWRD(??get_resp_110)
        DW        LWRD(??get_resp_111)
        DW        LWRD(??get_resp_112)
        DW        LWRD(??get_resp_112)
        DW        LWRD(??get_resp_113)
        DATA32
        DD        258
        DD        6183688
        DD        6183693
        DD        16843264
        DD        16844544
        DD        16844800
        DD        16910336
        DD        16975616
        DD        17106944
        DD        17172480
        DD        17238016
        DD        17303552
        DD        17367552
        DD        17368832
        DD        17369088
        DD        17434624
        DD        17630976
        DD        17696512
        DD        18155264
        DD        18286336
        DD        18679552
        DD        18810624
        DD        18876160
        DD        18941696
        DD        19007232
        DD        19465984
        DD        19597056
        DD        19990272
        DD        20121344
        DD        20186880
        DD        20252416
        DD        20317952
        DD        20776704
        DD        20907776
        DD        21300992
        DD        21432064
        DD        21497600
        DD        21563136
        DD        21628672
        DD        22087425
        DD        22087426
        DD        22087436
        DD        22087464
        DD        22087475
        DD        22087486
        DD        22742784
        DD        24905472
        DD        24971008
        DD        25036544
        DD        25167872
        DD        26085376
        DD        26150912
        DD        26216448
        DD        26281984

        SECTION `.data`:DATA:REORDER:NOROOT(0)
        SECTION_TYPE SHT_PROGBITS, 0
??get_resp_3:
        DATA16
        DW        42
        DW        LWRD(??get_resp_130)
        DW        LWRD(??get_resp_116)
        DW        LWRD(??get_resp_117)
        DW        LWRD(??get_resp_117)
        DW        LWRD(??get_resp_117)
        DW        LWRD(??get_resp_117)
        DW        LWRD(??get_resp_117)
        DW        LWRD(??get_resp_117)
        DW        LWRD(??get_resp_117)
        DW        LWRD(??get_resp_117)
        DW        LWRD(??get_resp_118)
        DW        LWRD(??get_resp_119)
        DW        LWRD(??get_resp_119)
        DW        LWRD(??get_resp_119)
        DW        LWRD(??get_resp_119)
        DW        LWRD(??get_resp_119)
        DW        LWRD(??get_resp_119)
        DW        LWRD(??get_resp_119)
        DW        LWRD(??get_resp_119)
        DW        LWRD(??get_resp_120)
        DW        LWRD(??get_resp_121)
        DW        LWRD(??get_resp_121)
        DW        LWRD(??get_resp_121)
        DW        LWRD(??get_resp_121)
        DW        LWRD(??get_resp_121)
        DW        LWRD(??get_resp_121)
        DW        LWRD(??get_resp_121)
        DW        LWRD(??get_resp_121)
        DW        LWRD(??get_resp_122)
        DW        LWRD(??get_resp_123)
        DW        LWRD(??get_resp_123)
        DW        LWRD(??get_resp_123)
        DW        LWRD(??get_resp_123)
        DW        LWRD(??get_resp_123)
        DW        LWRD(??get_resp_123)
        DW        LWRD(??get_resp_123)
        DW        LWRD(??get_resp_123)
        DW        LWRD(??get_resp_124)
        DW        LWRD(??get_resp_125)
        DW        LWRD(??get_resp_126)
        DW        LWRD(??get_resp_127)
        DW        LWRD(??get_resp_128)
        DW        LWRD(??get_resp_129)
        DATA32
        DD        16844288
        DD        16844289
        DD        16844290
        DD        16844291
        DD        16844292
        DD        16844293
        DD        16844294
        DD        16844295
        DD        16844296
        DD        16909824
        DD        16909825
        DD        16909826
        DD        16909827
        DD        16909828
        DD        16909829
        DD        16909830
        DD        16909831
        DD        16909832
        DD        17368576
        DD        17368577
        DD        17368578
        DD        17368579
        DD        17368580
        DD        17368581
        DD        17368582
        DD        17368583
        DD        17368584
        DD        17434112
        DD        17434113
        DD        17434114
        DD        17434115
        DD        17434116
        DD        17434117
        DD        17434118
        DD        17434119
        DD        17434120
        DD        18155008
        DD        18220544
        DD        19465728
        DD        19531264
        DD        20776448
        DD        20841984

        SECTION `.data`:DATA:REORDER:NOROOT(0)
        SECTION_TYPE SHT_PROGBITS, 0
??get_resp_4:
        DATA16
        DW        42
        DW        LWRD(??get_resp_133)
        DW        LWRD(??get_resp_131)
        DW        LWRD(??get_resp_131)
        DW        LWRD(??get_resp_131)
        DW        LWRD(??get_resp_131)
        DW        LWRD(??get_resp_131)
        DW        LWRD(??get_resp_131)
        DW        LWRD(??get_resp_131)
        DW        LWRD(??get_resp_131)
        DW        LWRD(??get_resp_131)
        DW        LWRD(??get_resp_131)
        DW        LWRD(??get_resp_131)
        DW        LWRD(??get_resp_131)
        DW        LWRD(??get_resp_131)
        DW        LWRD(??get_resp_131)
        DW        LWRD(??get_resp_131)
        DW        LWRD(??get_resp_131)
        DW        LWRD(??get_resp_131)
        DW        LWRD(??get_resp_131)
        DW        LWRD(??get_resp_132)
        DW        LWRD(??get_resp_131)
        DW        LWRD(??get_resp_131)
        DW        LWRD(??get_resp_131)
        DW        LWRD(??get_resp_131)
        DW        LWRD(??get_resp_131)
        DW        LWRD(??get_resp_131)
        DW        LWRD(??get_resp_131)
        DW        LWRD(??get_resp_131)
        DW        LWRD(??get_resp_132)
        DW        LWRD(??get_resp_131)
        DW        LWRD(??get_resp_131)
        DW        LWRD(??get_resp_131)
        DW        LWRD(??get_resp_131)
        DW        LWRD(??get_resp_131)
        DW        LWRD(??get_resp_131)
        DW        LWRD(??get_resp_131)
        DW        LWRD(??get_resp_131)
        DW        LWRD(??get_resp_131)
        DW        LWRD(??get_resp_131)
        DW        LWRD(??get_resp_131)
        DW        LWRD(??get_resp_131)
        DW        LWRD(??get_resp_131)
        DW        LWRD(??get_resp_131)
        DATA32
        DD        16844288
        DD        16844289
        DD        16844290
        DD        16844291
        DD        16844292
        DD        16844293
        DD        16844294
        DD        16844295
        DD        16844296
        DD        16909824
        DD        16909825
        DD        16909826
        DD        16909827
        DD        16909828
        DD        16909829
        DD        16909830
        DD        16909831
        DD        16909832
        DD        17368576
        DD        17368577
        DD        17368578
        DD        17368579
        DD        17368580
        DD        17368581
        DD        17368582
        DD        17368583
        DD        17368584
        DD        17434112
        DD        17434113
        DD        17434114
        DD        17434115
        DD        17434116
        DD        17434117
        DD        17434118
        DD        17434119
        DD        17434120
        DD        18155008
        DD        18220544
        DD        19465728
        DD        19531264
        DD        20776448
        DD        20841984

        SECTION `.data`:DATA:REORDER:NOROOT(0)
        SECTION_TYPE SHT_PROGBITS, 0
??get_resp_5:
        DATA16
        DW        42
        DW        LWRD(??get_resp_148)
        DW        LWRD(??get_resp_134)
        DW        LWRD(??get_resp_135)
        DW        LWRD(??get_resp_135)
        DW        LWRD(??get_resp_135)
        DW        LWRD(??get_resp_135)
        DW        LWRD(??get_resp_135)
        DW        LWRD(??get_resp_135)
        DW        LWRD(??get_resp_135)
        DW        LWRD(??get_resp_135)
        DW        LWRD(??get_resp_136)
        DW        LWRD(??get_resp_137)
        DW        LWRD(??get_resp_137)
        DW        LWRD(??get_resp_137)
        DW        LWRD(??get_resp_137)
        DW        LWRD(??get_resp_137)
        DW        LWRD(??get_resp_137)
        DW        LWRD(??get_resp_137)
        DW        LWRD(??get_resp_137)
        DW        LWRD(??get_resp_138)
        DW        LWRD(??get_resp_139)
        DW        LWRD(??get_resp_139)
        DW        LWRD(??get_resp_139)
        DW        LWRD(??get_resp_139)
        DW        LWRD(??get_resp_139)
        DW        LWRD(??get_resp_139)
        DW        LWRD(??get_resp_139)
        DW        LWRD(??get_resp_139)
        DW        LWRD(??get_resp_140)
        DW        LWRD(??get_resp_141)
        DW        LWRD(??get_resp_141)
        DW        LWRD(??get_resp_141)
        DW        LWRD(??get_resp_141)
        DW        LWRD(??get_resp_141)
        DW        LWRD(??get_resp_141)
        DW        LWRD(??get_resp_141)
        DW        LWRD(??get_resp_141)
        DW        LWRD(??get_resp_142)
        DW        LWRD(??get_resp_143)
        DW        LWRD(??get_resp_144)
        DW        LWRD(??get_resp_145)
        DW        LWRD(??get_resp_146)
        DW        LWRD(??get_resp_147)
        DATA32
        DD        16844288
        DD        16844289
        DD        16844290
        DD        16844291
        DD        16844292
        DD        16844293
        DD        16844294
        DD        16844295
        DD        16844296
        DD        16909824
        DD        16909825
        DD        16909826
        DD        16909827
        DD        16909828
        DD        16909829
        DD        16909830
        DD        16909831
        DD        16909832
        DD        17368576
        DD        17368577
        DD        17368578
        DD        17368579
        DD        17368580
        DD        17368581
        DD        17368582
        DD        17368583
        DD        17368584
        DD        17434112
        DD        17434113
        DD        17434114
        DD        17434115
        DD        17434116
        DD        17434117
        DD        17434118
        DD        17434119
        DD        17434120
        DD        18155008
        DD        18220544
        DD        19465728
        DD        19531264
        DD        20776448
        DD        20841984

        SECTION `.data`:DATA:REORDER:NOROOT(0)
        SECTION_TYPE SHT_PROGBITS, 0
??get_resp_6:
        DATA16
        DW        17
        DW        LWRD(??get_resp_166)
        DW        LWRD(??get_resp_165)
        DW        LWRD(??get_resp_158)
        DW        LWRD(??get_resp_159)
        DW        LWRD(??get_resp_160)
        DW        LWRD(??get_resp_161)
        DW        LWRD(??get_resp_162)
        DW        LWRD(??get_resp_163)
        DW        LWRD(??get_resp_164)
        DW        LWRD(??get_resp_149)
        DW        LWRD(??get_resp_150)
        DW        LWRD(??get_resp_151)
        DW        LWRD(??get_resp_152)
        DW        LWRD(??get_resp_153)
        DW        LWRD(??get_resp_154)
        DW        LWRD(??get_resp_157)
        DW        LWRD(??get_resp_155)
        DW        LWRD(??get_resp_156)
        DATA32
        DD        6183690
        DD        6513152
        DD        6513153
        DD        6513154
        DD        6513155
        DD        6513156
        DD        6513157
        DD        6513251
        DD        22960896
        DD        22960899
        DD        22960900
        DD        22960901
        DD        22960902
        DD        22960903
        DD        23200000
        DD        23265536
        DD        23265792

        SECTION `.data`:DATA:REORDER:NOROOT(0)
        SECTION_TYPE SHT_PROGBITS, 0
??get_resp_7:
        DATA16
        DW        17
        DW        LWRD(??get_resp_184)
        DW        LWRD(??get_resp_183)
        DW        LWRD(??get_resp_176)
        DW        LWRD(??get_resp_177)
        DW        LWRD(??get_resp_178)
        DW        LWRD(??get_resp_179)
        DW        LWRD(??get_resp_180)
        DW        LWRD(??get_resp_181)
        DW        LWRD(??get_resp_182)
        DW        LWRD(??get_resp_167)
        DW        LWRD(??get_resp_168)
        DW        LWRD(??get_resp_169)
        DW        LWRD(??get_resp_170)
        DW        LWRD(??get_resp_171)
        DW        LWRD(??get_resp_172)
        DW        LWRD(??get_resp_175)
        DW        LWRD(??get_resp_173)
        DW        LWRD(??get_resp_174)
        DATA32
        DD        6183690
        DD        6513152
        DD        6513153
        DD        6513154
        DD        6513155
        DD        6513156
        DD        6513157
        DD        6513251
        DD        22960896
        DD        22960899
        DD        22960900
        DD        22960901
        DD        22960902
        DD        22960903
        DD        23200000
        DD        23265536
        DD        23265792

        SECTION `.data`:DATA:REORDER:NOROOT(0)
        SECTION_TYPE SHT_PROGBITS, 0
??get_resp_8:
        DATA16
        DW        17
        DW        LWRD(??get_resp_188)
        DW        LWRD(??get_resp_185)
        DW        LWRD(??get_resp_185)
        DW        LWRD(??get_resp_185)
        DW        LWRD(??get_resp_185)
        DW        LWRD(??get_resp_185)
        DW        LWRD(??get_resp_185)
        DW        LWRD(??get_resp_185)
        DW        LWRD(??get_resp_185)
        DW        LWRD(??get_resp_185)
        DW        LWRD(??get_resp_185)
        DW        LWRD(??get_resp_185)
        DW        LWRD(??get_resp_185)
        DW        LWRD(??get_resp_185)
        DW        LWRD(??get_resp_185)
        DW        LWRD(??get_resp_185)
        DW        LWRD(??get_resp_186)
        DW        LWRD(??get_resp_187)
        DATA32
        DD        6183690
        DD        6513152
        DD        6513153
        DD        6513154
        DD        6513155
        DD        6513156
        DD        6513157
        DD        6513251
        DD        22960896
        DD        22960899
        DD        22960900
        DD        22960901
        DD        22960902
        DD        22960903
        DD        23200000
        DD        23265536
        DD        23265792

        SECTION `.data`:DATA:REORDER:NOROOT(0)
        SECTION_TYPE SHT_PROGBITS, 0
??get_resp_9:
        DATA16
        DW        17
        DW        LWRD(??get_resp_190)
        DW        LWRD(??get_resp_189)
        DW        LWRD(??get_resp_189)
        DW        LWRD(??get_resp_189)
        DW        LWRD(??get_resp_189)
        DW        LWRD(??get_resp_189)
        DW        LWRD(??get_resp_189)
        DW        LWRD(??get_resp_189)
        DW        LWRD(??get_resp_189)
        DW        LWRD(??get_resp_189)
        DW        LWRD(??get_resp_189)
        DW        LWRD(??get_resp_189)
        DW        LWRD(??get_resp_189)
        DW        LWRD(??get_resp_189)
        DW        LWRD(??get_resp_189)
        DW        LWRD(??get_resp_189)
        DW        LWRD(??get_resp_189)
        DW        LWRD(??get_resp_189)
        DATA32
        DD        6183690
        DD        6513152
        DD        6513153
        DD        6513154
        DD        6513155
        DD        6513156
        DD        6513157
        DD        6513251
        DD        22960896
        DD        22960899
        DD        22960900
        DD        22960901
        DD        22960902
        DD        22960903
        DD        23200000
        DD        23265536
        DD        23265792

        SECTION `.data`:DATA:REORDER:NOROOT(0)
        SECTION_TYPE SHT_PROGBITS, 0
??get_resp_10:
        DATA16
        DW        17
        DW        LWRD(??get_resp_193)
        DW        LWRD(??get_resp_191)
        DW        LWRD(??get_resp_191)
        DW        LWRD(??get_resp_191)
        DW        LWRD(??get_resp_191)
        DW        LWRD(??get_resp_191)
        DW        LWRD(??get_resp_191)
        DW        LWRD(??get_resp_191)
        DW        LWRD(??get_resp_191)
        DW        LWRD(??get_resp_191)
        DW        LWRD(??get_resp_191)
        DW        LWRD(??get_resp_191)
        DW        LWRD(??get_resp_191)
        DW        LWRD(??get_resp_191)
        DW        LWRD(??get_resp_191)
        DW        LWRD(??get_resp_191)
        DW        LWRD(??get_resp_192)
        DW        LWRD(??get_resp_192)
        DATA32
        DD        6183690
        DD        6513152
        DD        6513153
        DD        6513154
        DD        6513155
        DD        6513156
        DD        6513157
        DD        6513251
        DD        22960896
        DD        22960899
        DD        22960900
        DD        22960901
        DD        22960902
        DD        22960903
        DD        23200000
        DD        23265536
        DD        23265792

        SECTION `.data`:DATA:REORDER:NOROOT(0)
        SECTION_TYPE SHT_PROGBITS, 0
??get_resp_11:
        DATA16
        DW        17
        DW        LWRD(??get_resp_205)
        DW        LWRD(??get_resp_194)
        DW        LWRD(??get_resp_198)
        DW        LWRD(??get_resp_199)
        DW        LWRD(??get_resp_200)
        DW        LWRD(??get_resp_201)
        DW        LWRD(??get_resp_202)
        DW        LWRD(??get_resp_203)
        DW        LWRD(??get_resp_204)
        DW        LWRD(??get_resp_194)
        DW        LWRD(??get_resp_194)
        DW        LWRD(??get_resp_194)
        DW        LWRD(??get_resp_194)
        DW        LWRD(??get_resp_194)
        DW        LWRD(??get_resp_194)
        DW        LWRD(??get_resp_197)
        DW        LWRD(??get_resp_195)
        DW        LWRD(??get_resp_196)
        DATA32
        DD        6183690
        DD        6513152
        DD        6513153
        DD        6513154
        DD        6513155
        DD        6513156
        DD        6513157
        DD        6513251
        DD        22960896
        DD        22960899
        DD        22960900
        DD        22960901
        DD        22960902
        DD        22960903
        DD        23200000
        DD        23265536
        DD        23265792

        SECTION `.data`:DATA:REORDER:NOROOT(0)
        SECTION_TYPE SHT_PROGBITS, 0
??get_resp_12:
        DATA16
        DW        17
        DW        LWRD(??get_resp_217)
        DW        LWRD(??get_resp_206)
        DW        LWRD(??get_resp_210)
        DW        LWRD(??get_resp_211)
        DW        LWRD(??get_resp_212)
        DW        LWRD(??get_resp_213)
        DW        LWRD(??get_resp_214)
        DW        LWRD(??get_resp_215)
        DW        LWRD(??get_resp_216)
        DW        LWRD(??get_resp_206)
        DW        LWRD(??get_resp_206)
        DW        LWRD(??get_resp_206)
        DW        LWRD(??get_resp_206)
        DW        LWRD(??get_resp_206)
        DW        LWRD(??get_resp_206)
        DW        LWRD(??get_resp_209)
        DW        LWRD(??get_resp_207)
        DW        LWRD(??get_resp_208)
        DATA32
        DD        6183690
        DD        6513152
        DD        6513153
        DD        6513154
        DD        6513155
        DD        6513156
        DD        6513157
        DD        6513251
        DD        22960896
        DD        22960899
        DD        22960900
        DD        22960901
        DD        22960902
        DD        22960903
        DD        23200000
        DD        23265536
        DD        23265792
//  157 /* unsigned char code Xdlms_type[] = {0x00,0x02,0x06,0x04,0x18,0x00,0x00,0x18,0x12,0x01,0xff,0x12,0x01,0xff,0x11,0x06,0x0f,0x00,0x09,0x00}; */

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//  158 us8 obis_a,obis_b,obis_c,obis_d,obis_e,obis_f;
_obis_a:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_obis_b:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_obis_c:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_obis_d:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_obis_e:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_obis_f:
        DS 1
//  159 
//  160 

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//  161 flag_union flag_optical, flag_rj45, flag_communication,flag_hdlc1,flag_hdlc2,flag_hdlc3,flag_hdlc4;
_flag_optical:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_flag_rj45:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_flag_communication:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_flag_hdlc1:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_flag_hdlc2:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_flag_hdlc3:
        DS 1

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
_flag_hdlc4:
        DS 1
//  162 
//  163 
//  164 /* Function */
//  165 void compartment6();
//  166 void compartment3();
//  167 us32 obis_short_cal(us8 *obis);
//  168 void get_resp1();
//  169 void get_resp(void);
//  170 void capture_objects_filler(unsigned char const *p_f);
//  171 void buffer_scaler_filler(unsigned char const  *s_f);
//  172 void tamper_status(void);
//  173 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock0 Using cfiCommon0
          CFI Function _tamper_status
        CODE
//  174 void tamper_status(void)
//  175 {
_tamper_status:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 4
        SUBW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
//  176   us32 tamper_byte = 0;
        MOVW      AX, SP             ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
//  177   
//  178   //for now only two tampers required ,so we are sending status of only two tampers.
//  179   //memory ,rtc and battery status will come fine always because we are not sending there flags.will take care in project.    pending
//  180   
//  181   if(flag_Rph_active == EXPORT) 
        MOVW      HL, #LWRD(_flag_quadrant)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??compartment3_0   ;; 4 cycles
        ; ------------------------------------- Block: 13 cycles
//  182   {
//  183     bitSet(tamper_byte,bit6);
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        OR        A, #0x40           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      HL, SP             ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ; ------------------------------------- Block: 11 cycles
//  184   }
//  185   if(flag_Yph_active == EXPORT)
??compartment3_0:
        MOVW      HL, #LWRD(_flag_quadrant)  ;; 1 cycle
        MOV1      CY, [HL].2         ;; 1 cycle
        BNC       ??compartment3_1   ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  186   {
//  187     bitSet(tamper_byte,bit7);
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        OR        A, #0x80           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      HL, SP             ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ; ------------------------------------- Block: 11 cycles
//  188   }
//  189   if(flag_Bph_active == EXPORT)
??compartment3_1:
        MOVW      HL, #LWRD(_flag_quadrant)  ;; 1 cycle
        MOV1      CY, [HL].4         ;; 1 cycle
        BNC       ??compartment3_2   ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  190   {
//  191     bitSet(tamper_byte,bit8);
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        OR        A, #0x1            ;; 1 cycle
        MOVW      HL, SP             ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ; ------------------------------------- Block: 9 cycles
//  192   }
//  193   if(bitIsSet(tamper_instant_status,BIT_MAGNET))
??compartment3_2:
        MOVW      HL, #LWRD(_tamper_instant_status)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??compartment3_3   ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  194   {
//  195     bitSet(tamper_byte,bit15);
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        OR        A, #0x80           ;; 1 cycle
        MOVW      HL, SP             ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ; ------------------------------------- Block: 9 cycles
//  196   }
//  197   if(bitIsSet(tpr.top_cover.flag,event_f))
??compartment3_3:
        MOVW      HL, #LWRD(_tpr+592)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??compartment3_4   ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  198   {
//  199     bitSet(tamper_byte,bit18);
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCH       A, C               ;; 1 cycle
        OR        A, #0x4            ;; 1 cycle
        XCH       A, C               ;; 1 cycle
        MOVW      HL, SP             ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ; ------------------------------------- Block: 11 cycles
//  200   }
//  201   if(flag_phase_seq == 0)
??compartment3_4:
        MOVW      HL, #LWRD(_flag_metro1)  ;; 1 cycle
        MOV1      CY, [HL].6         ;; 1 cycle
        BC        ??compartment3_5   ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  202   {
//  203      bitSet(tamper_byte,bit19);
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCH       A, C               ;; 1 cycle
        OR        A, #0x8            ;; 1 cycle
        XCH       A, C               ;; 1 cycle
        MOVW      HL, SP             ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ; ------------------------------------- Block: 11 cycles
//  204   }
//  205   if(flag_eep_fail == 1 || flag_eep0_fail == 1 || flag_eep1_fail == 1 || flag_eep2_fail == 1)
??compartment3_5:
        MOV       A, N:_flag_eeprom_error  ;; 1 cycle
        AND       A, #0x55           ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        BZ        ??compartment3_6   ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  206   {
//  207      bitSet(tamper_byte,bit23);
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCH       A, C               ;; 1 cycle
        OR        A, #0x80           ;; 1 cycle
        XCH       A, C               ;; 1 cycle
        MOVW      HL, SP             ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ; ------------------------------------- Block: 11 cycles
//  208   }
//  209   if(bitIsSet(tpr.battery_low.flag,event_f))
??compartment3_6:
        MOVW      HL, #LWRD(_tpr+616)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??compartment3_7   ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  210   {
//  211      bitSet(tamper_byte,bit26);
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        OR        A, #0x4            ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        MOVW      HL, SP             ;; 1 cycle
        MOVW      [HL], AX           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [HL+0x02], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ; ------------------------------------- Block: 11 cycles
//  212   }
//  213   
//  214   info[k++]= 0x00;
??compartment3_7:
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
//  215   info[k++]= 0x04;
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, #0x4            ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
//  216   info[k++]= 32;
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, #0x20           ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
//  217   long_into_char_array4(tamper_byte,&info[k]);
        MOVW      AX, N:_k           ;; 1 cycle
        ADDW      AX, #LWRD(_info)   ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  218   k += 4;
        MOVW      AX, N:_k           ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
        MOVW      N:_k, AX           ;; 1 cycle
//  219 }
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock0
        ; ------------------------------------- Block: 34 cycles
        ; ------------------------------------- Total: 174 cycles
//  220 
//  221 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock1 Using cfiCommon1
          CFI Function _obis_short_cal
          CFI NoCalls
        CODE
//  222 us32 obis_short_cal(us8 *obis)
//  223 {
_obis_short_cal:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
        MOVW      HL, AX             ;; 1 cycle
//  224   static union_4byte temp_union;
//  225   temp_union.s.byte3 = obis[0];
        MOV       A, [HL]            ;; 1 cycle
        MOV       N:`obis_short_cal::temp_union`+3, A  ;; 1 cycle
//  226   temp_union.s.byte2 = obis[2];
        MOV       A, [HL+0x02]       ;; 1 cycle
        MOV       N:`obis_short_cal::temp_union`+2, A  ;; 1 cycle
//  227   temp_union.s.byte1 = obis[3];
        MOV       A, [HL+0x03]       ;; 1 cycle
        MOV       N:`obis_short_cal::temp_union`+1, A  ;; 1 cycle
//  228   temp_union.s.byte0 = obis[4];
        MOV       A, [HL+0x04]       ;; 1 cycle
        MOV       N:`obis_short_cal::temp_union`, A  ;; 1 cycle
//  229   
//  230   obis_a = obis[0];
        MOV       A, [HL]            ;; 1 cycle
        MOV       N:_obis_a, A       ;; 1 cycle
//  231   obis_b = obis[1];
        MOV       A, [HL+0x01]       ;; 1 cycle
        MOV       N:_obis_b, A       ;; 1 cycle
//  232   obis_c = obis[2];
        MOV       A, [HL+0x02]       ;; 1 cycle
        MOV       N:_obis_c, A       ;; 1 cycle
//  233   obis_d = obis[3];
        MOV       A, [HL+0x03]       ;; 1 cycle
        MOV       N:_obis_d, A       ;; 1 cycle
//  234   obis_e = obis[4];
        MOV       A, [HL+0x04]       ;; 1 cycle
        MOV       N:_obis_e, A       ;; 1 cycle
//  235   obis_f = obis[5];
        MOV       A, [HL+0x05]       ;; 1 cycle
        MOV       N:_obis_f, A       ;; 1 cycle
//  236   
//  237   return(temp_union.val);
        MOVW      BC, N:`obis_short_cal::temp_union`+2  ;; 1 cycle
        MOVW      AX, N:`obis_short_cal::temp_union`  ;; 1 cycle
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock1
        ; ------------------------------------- Block: 29 cycles
        ; ------------------------------------- Total: 29 cycles
//  238 }

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
`obis_short_cal::temp_union`:
        DS 4

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock2 Using cfiCommon0
          CFI Function _get_resp1
        CODE
//  239 void get_resp1()
//  240 {
_get_resp1:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 8
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+12
//  241   uint16_t classatt;
//  242   uint32_t obis1;
//  243   uint8_t class_att[2];
//  244   
//  245   class_att[0]= info[7]; /* class id */
        MOV       A, N:_info+7       ;; 1 cycle
        MOV       [SP+0x04], A       ;; 1 cycle
//  246   class_att[1]= attribute_id;
        MOV       A, N:_attribute_id  ;; 1 cycle
        MOV       [SP+0x05], A       ;; 1 cycle
//  247   
//  248   classatt= (uint16_t)(class_att[0] * 256) + class_att[1];
        MOV       A, [SP+0x04]       ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x100         ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, [SP+0x05]       ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       D, #0x0            ;; 1 cycle
        MOVW      AX, HL             ;; 1 cycle
        ADDW      AX, DE             ;; 1 cycle
        MOVW      [SP+0x06], AX      ;; 1 cycle
//  249   
//  250   obis1= obis_short_cal(&info[8]);
        MOVW      AX, #LWRD(_info+8)  ;; 1 cycle
          CFI FunCall _obis_short_cal
        CALL      _obis_short_cal    ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+10
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
//  251   
//  252   switch(classatt)
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        SUBW      AX, #0x101         ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??compartment3_8  ;; 4 cycles
        ; ------------------------------------- Block: 31 cycles
        SUBW      AX, #0x1           ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??compartment3_9  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        SUBW      AX, #0x6FF         ;; 1 cycle
        BZ        ??compartment3_10  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        SUBW      AX, #0x1           ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??compartment3_11  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        SUBW      AX, #0x1           ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??compartment3_12  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        SUBW      AX, #0x1           ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??compartment3_13  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        SUBW      AX, #0x1           ;; 1 cycle
        CMPW      AX, #0x4           ;; 1 cycle
        SKNC                         ;; 4 cycles
        BR        N:??compartment3_14  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        SUBW      AX, #0x4           ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??compartment3_15  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        SUBW      AX, #0x6F8         ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??compartment3_16  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        SUBW      AX, #0x1           ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??compartment3_17  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        SUBW      AX, #0x1           ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??compartment3_18  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        SUBW      AX, #0x1           ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??compartment3_19  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        SUBW      AX, #0x1           ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??compartment3_20  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        SUBW      AX, #0x1           ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??compartment3_21  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        SUBW      AX, #0x1           ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??compartment3_22  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        SUBW      AX, #0x1           ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??compartment3_23  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        SUBW      AX, #0x2FFA        ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??compartment3_24  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        SUBW      AX, #0x1           ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??compartment3_25  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        BR        N:??compartment3_26  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
//  253   {
//  254   case 0x0801:
//  255     if(obis1 == 0x00010000)
??compartment3_10:
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x1           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x0           ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??get_resp1_2:
        BNZ       ??compartment3_27  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
//  256     {
//  257       log_name2(0, 0, 1, 0, 0);
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x1            ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _log_name2
        CALL      _log_name2         ;; 3 cycles
        BR        S:??compartment3_28  ;; 3 cycles
          CFI FunCall _fill_0b
        ; ------------------------------------- Block: 11 cycles
//  258     }
//  259     else
//  260     {
//  261       fill_0b();
??compartment3_27:
        CALL      _fill_0b           ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
//  262     }
//  263     break;
??compartment3_28:
        BR        N:??compartment3_29  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
//  264     
//  265   case 0x0802:
//  266     if(obis1 == 0x00010000)
??compartment3_11:
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x1           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x0           ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??get_resp1_3:
        BNZ       ??compartment3_30  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
//  267     {
//  268       date_time(Now.day, Now.month, Now.year, Now.hour, Now.min, Now.sec, 0x03);
        MOV       X, #0x3            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+14
        MOV       A, N:_Now          ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, N:_Now+1        ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       B, N:_Now+2        ;; 1 cycle
        MOV       C, N:_Now+6        ;; 1 cycle
        MOV       X, N:_Now+5        ;; 1 cycle
        MOV       A, N:_Now+3        ;; 1 cycle
          CFI FunCall _date_time
        CALL      _date_time         ;; 3 cycles
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+12
        BR        S:??compartment3_31  ;; 3 cycles
          CFI FunCall _fill_0b
        ; ------------------------------------- Block: 17 cycles
//  269     }
//  270     else
//  271     {
//  272       fill_0b();
??compartment3_30:
        CALL      _fill_0b           ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
//  273     }
//  274     break;
??compartment3_31:
        BR        N:??compartment3_29  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
//  275     
//  276   case 0x0803:
//  277     if(obis1 == 0x00010000)
??compartment3_12:
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x1           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x0           ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??get_resp1_4:
        BNZ       ??compartment3_32  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
//  278     {
//  279       info[k]= 0x00;
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
//  280       info[k + 1]= 0x10; /* integer16 */
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, #0x10           ;; 1 cycle
        MOV       (_info+1)[BC], A   ;; 1 cycle
//  281       info[k + 2]= 0x01; /* byte-1				//01 */
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, #0x1            ;; 1 cycle
        MOV       (_info+2)[BC], A   ;; 1 cycle
//  282       info[k + 3]= 0x4a; /* byte-2				//4a */
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, #0x4A           ;; 1 cycle
        MOV       (_info+3)[BC], A   ;; 1 cycle
//  283       info_l();
          CFI FunCall _info_l
        CALL      _info_l            ;; 3 cycles
        BR        S:??compartment3_33  ;; 3 cycles
          CFI FunCall _fill_0b
        ; ------------------------------------- Block: 18 cycles
//  284     }
//  285     else
//  286     {
//  287       fill_0b();
??compartment3_32:
        CALL      _fill_0b           ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
//  288     }
//  289     break;
??compartment3_33:
        BR        N:??compartment3_29  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
//  290     
//  291   case 0x0804:
//  292     if(obis1 == 0x00010000)
??compartment3_13:
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x1           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x0           ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??get_resp1_5:
        BNZ       ??compartment3_34  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
//  293     {
//  294       if(flag_rtc_read_error_after_retry == 1)
        MOVW      HL, #LWRD(_flag_rtc2)  ;; 1 cycle
        MOV1      CY, [HL].1         ;; 1 cycle
        BNC       ??compartment3_35  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  295       {
//  296         unsigned8(0x01);
        MOV       A, #0x1            ;; 1 cycle
          CFI FunCall _unsigned8
        CALL      _unsigned8         ;; 3 cycles
        BR        S:??compartment3_36  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
//  297       }
//  298       else
//  299       {
//  300         unsigned8(0x00);
??compartment3_35:
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _unsigned8
        CALL      _unsigned8         ;; 3 cycles
        BR        S:??compartment3_36  ;; 3 cycles
          CFI FunCall _fill_0b
        ; ------------------------------------- Block: 7 cycles
//  301       }
//  302     }
//  303     else
//  304     {
//  305       fill_0b();
??compartment3_34:
        CALL      _fill_0b           ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
//  306     }
//  307     break;
??compartment3_36:
        BR        N:??compartment3_29  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
//  308     
//  309   case 0x0805:
//  310   case 0x0806:
//  311   case 0x0807:
//  312   case 0x0808:
//  313     if(obis1 == 0x00010000)
??compartment3_14:
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x1           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x0           ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??get_resp1_6:
        BNZ       ??compartment3_37  ;; 4 cycles
          CFI FunCall _fill_0d
        ; ------------------------------------- Block: 4 cycles
//  314     {
//  315       fill_0d();
        CALL      _fill_0d           ;; 3 cycles
        BR        S:??compartment3_38  ;; 3 cycles
          CFI FunCall _fill_0b
        ; ------------------------------------- Block: 6 cycles
//  316     }
//  317     else
//  318     {
//  319       fill_0b();
??compartment3_37:
        CALL      _fill_0b           ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
//  320     }
//  321     break;
??compartment3_38:
        BR        N:??compartment3_29  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
//  322     
//  323   case 0x0809:
//  324     if(obis1 == 0x00010000)
??compartment3_15:
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x1           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x0           ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??get_resp1_7:
        BNZ       ??compartment3_39  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
//  325     {
//  326       enum_d2(0x01);
        MOV       A, #0x1            ;; 1 cycle
          CFI FunCall _enum_d2
        CALL      _enum_d2           ;; 3 cycles
        BR        S:??compartment3_40  ;; 3 cycles
          CFI FunCall _fill_0b
        ; ------------------------------------- Block: 7 cycles
//  327     }
//  328     else
//  329     {
//  330       fill_0b();
??compartment3_39:
        CALL      _fill_0b           ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
//  331     }
//  332     break;
??compartment3_40:
        BR        N:??compartment3_29  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
//  333     
//  334   case 0x0f01:                                /* logical_name */
//  335     if(obis1 == 0x00280000)
??compartment3_16:
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x28          ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x0           ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??get_resp1_8:
        BNZ       ??compartment3_41  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
//  336     {
//  337       log_name2(0, 0, 40, 0, 0);
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x28           ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _log_name2
        CALL      _log_name2         ;; 3 cycles
        BR        S:??compartment3_42  ;; 3 cycles
        ; ------------------------------------- Block: 11 cycles
//  338     }
//  339     else if(obis1 == 0x00280001)
??compartment3_41:
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x28          ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x1           ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??get_resp1_9:
        BNZ       ??compartment3_43  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
//  340     {
//  341       log_name2(0, 0, 40, 0, 1);
        MOV       E, #0x1            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x28           ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _log_name2
        CALL      _log_name2         ;; 3 cycles
        BR        S:??compartment3_42  ;; 3 cycles
          CFI FunCall _fill_0b
        ; ------------------------------------- Block: 11 cycles
//  342     }
//  343     else
//  344     {
//  345       fill_0b();
??compartment3_43:
        CALL      _fill_0b           ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
//  346     }
//  347     break;
??compartment3_42:
        BR        N:??compartment3_29  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
//  348     
//  349   case 0x0f02:                                /* object_list */
//  350     if((obis1 == 0x00280000) || (obis1 == 0x00280001))
??compartment3_17:
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x28          ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x0           ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??get_resp1_10:
        BZ        ??compartment3_44  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x28          ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x1           ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??get_resp1_11:
        BNZ       ??compartment3_45  ;; 4 cycles
          CFI FunCall _object_list
        ; ------------------------------------- Block: 4 cycles
//  351     {
//  352       object_list();
??compartment3_44:
        CALL      _object_list       ;; 3 cycles
        BR        S:??compartment3_46  ;; 3 cycles
          CFI FunCall _fill_0b
        ; ------------------------------------- Block: 6 cycles
//  353     }
//  354     else
//  355     {
//  356       fill_0b();
??compartment3_45:
        CALL      _fill_0b           ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
//  357     }
//  358     break;
??compartment3_46:
        BR        N:??compartment3_29  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
//  359     
//  360   case 0x0f03:                                /* partners_id */
//  361     if((obis1 == 0x00280000) || (obis1 == 0x00280001))
??compartment3_18:
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x28          ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x0           ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??get_resp1_12:
        BZ        ??compartment3_47  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x28          ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x1           ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??get_resp1_13:
        BNZ       ??compartment3_48  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
//  362     {
//  363       info[k++]= 0;
??compartment3_47:
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
//  364       structure(2);
        MOV       A, #0x2            ;; 1 cycle
          CFI FunCall _structure
        CALL      _structure         ;; 3 cycles
//  365       if(asso3_flag == 1)
        CMP       N:_asso3_flag, #0x1  ;; 1 cycle
        BNZ       ??compartment3_49  ;; 4 cycles
        ; ------------------------------------- Block: 14 cycles
//  366       {
//  367         integer8(0x40);
        MOV       A, #0x40           ;; 1 cycle
          CFI FunCall _integer8
        CALL      _integer8          ;; 3 cycles
        BR        S:??compartment3_50  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
//  368       }
//  369       else if(asso2_flag == 1)
??compartment3_49:
        CMP       N:_asso2_flag, #0x1  ;; 1 cycle
        BNZ       ??compartment3_51  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  370       {
//  371         integer8(0x30);
        MOV       A, #0x30           ;; 1 cycle
          CFI FunCall _integer8
        CALL      _integer8          ;; 3 cycles
        BR        S:??compartment3_50  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
//  372       }
//  373       else if(asso1_flag == 1)
??compartment3_51:
        CMP       N:_asso1_flag, #0x1  ;; 1 cycle
        BNZ       ??compartment3_52  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  374       {
//  375         integer8(0x20);
        MOV       A, #0x20           ;; 1 cycle
          CFI FunCall _integer8
        CALL      _integer8          ;; 3 cycles
        BR        S:??compartment3_50  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
//  376       }
//  377       else
//  378       {
//  379         integer8(0x10);
??compartment3_52:
        MOV       A, #0x10           ;; 1 cycle
          CFI FunCall _integer8
        CALL      _integer8          ;; 3 cycles
        ; ------------------------------------- Block: 4 cycles
//  380       }
//  381       
//  382       val_2byt(0x00, 0x01);
??compartment3_50:
        MOV       X, #0x1            ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_2byt
        CALL      _val_2byt          ;; 3 cycles
        BR        S:??compartment3_53  ;; 3 cycles
          CFI FunCall _fill_0b
        ; ------------------------------------- Block: 8 cycles
//  383     }
//  384     else
//  385     {
//  386       fill_0b();
??compartment3_48:
        CALL      _fill_0b           ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
//  387     }
//  388     break;
??compartment3_53:
        BR        N:??compartment3_29  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
//  389     
//  390   case 0x0f04:                                /* application_context_name */
//  391     if((obis1 == 0x00280000) || (obis1 == 0x00280001))
??compartment3_19:
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x28          ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x0           ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??get_resp1_14:
        BZ        ??compartment3_54  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x28          ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x1           ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??get_resp1_15:
        BNZ       ??compartment3_55  ;; 4 cycles
          CFI FunCall _app_con
        ; ------------------------------------- Block: 4 cycles
//  392     {
//  393       app_con();
??compartment3_54:
        CALL      _app_con           ;; 3 cycles
        BR        S:??compartment3_56  ;; 3 cycles
          CFI FunCall _fill_0b
        ; ------------------------------------- Block: 6 cycles
//  394     }
//  395     else
//  396     {
//  397       fill_0b();
??compartment3_55:
        CALL      _fill_0b           ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
//  398     }
//  399     break;
??compartment3_56:
        BR        N:??compartment3_29  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
//  400     
//  401   case 0x0f05:                                /* xDLMS_context_info */
//  402     if((obis1 == 0x00280000) || (obis1 == 0x00280001))
??compartment3_20:
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x28          ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x0           ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??get_resp1_16:
        BZ        ??compartment3_57  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x28          ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x1           ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??get_resp1_17:
        BNZ       ??compartment3_58  ;; 4 cycles
          CFI FunCall _xdlms_type
        ; ------------------------------------- Block: 4 cycles
//  403     {
//  404       xdlms_type();
??compartment3_57:
        CALL      _xdlms_type        ;; 3 cycles
        BR        S:??compartment3_59  ;; 3 cycles
          CFI FunCall _fill_0b
        ; ------------------------------------- Block: 6 cycles
//  405     }
//  406     else
//  407     {
//  408       fill_0b();
??compartment3_58:
        CALL      _fill_0b           ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
//  409     }
//  410     break;
??compartment3_59:
        BR        N:??compartment3_29  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
//  411     
//  412   case 0x0f06:                                /* authentication mech_name */
//  413     if((obis1 == 0x00280000) || (obis1 == 0x00280001))
??compartment3_21:
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x28          ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x0           ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??get_resp1_18:
        BZ        ??compartment3_60  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x28          ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x1           ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??get_resp1_19:
        BNZ       ??compartment3_61  ;; 4 cycles
          CFI FunCall _auth_name
        ; ------------------------------------- Block: 4 cycles
//  414     {
//  415       auth_name();
??compartment3_60:
        CALL      _auth_name         ;; 3 cycles
        BR        S:??compartment3_62  ;; 3 cycles
          CFI FunCall _fill_0b
        ; ------------------------------------- Block: 6 cycles
//  416     }
//  417     else
//  418     {
//  419       fill_0b();
??compartment3_61:
        CALL      _fill_0b           ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
//  420     }
//  421     break;
??compartment3_62:
        BR        N:??compartment3_29  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
//  422     
//  423   case 0x0f07:
//  424     if((obis1 == 0x00280000) || (obis1 == 0x00280001))
??compartment3_22:
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x28          ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x0           ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??get_resp1_20:
        BZ        ??compartment3_63  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x28          ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x1           ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??get_resp1_21:
        BNZ       ??compartment3_64  ;; 4 cycles
          CFI FunCall _fill_0d
        ; ------------------------------------- Block: 4 cycles
//  425     {
//  426       fill_0d();
??compartment3_63:
        CALL      _fill_0d           ;; 3 cycles
        BR        S:??compartment3_65  ;; 3 cycles
          CFI FunCall _fill_0b
        ; ------------------------------------- Block: 6 cycles
//  427     }
//  428     else
//  429     {
//  430       fill_0b();
??compartment3_64:
        CALL      _fill_0b           ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
//  431     }
//  432     
//  433     break;
??compartment3_65:
        BR        N:??compartment3_29  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
//  434     
//  435   case 0x0f08:                                /* association status */
//  436     if((obis1 == 0x00280000) || (obis1 == 0x00280001))
??compartment3_23:
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x28          ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x0           ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??get_resp1_22:
        BZ        ??compartment3_66  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x28          ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x1           ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??get_resp1_23:
        BNZ       ??compartment3_67  ;; 4 cycles
          CFI FunCall _asso_status
        ; ------------------------------------- Block: 4 cycles
//  437     {
//  438       asso_status();
??compartment3_66:
        CALL      _asso_status       ;; 3 cycles
        BR        S:??compartment3_68  ;; 3 cycles
          CFI FunCall _fill_0b
        ; ------------------------------------- Block: 6 cycles
//  439     }
//  440     else
//  441     {
//  442       fill_0b();
??compartment3_67:
        CALL      _fill_0b           ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
//  443     }
//  444     
//  445     break;
??compartment3_68:
        BR        N:??compartment3_29  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
//  446     
//  447   case 0x0101:
//  448     switch(obis1)
??compartment3_8:
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        MOVW      HL, #LWRD(??get_resp1_0)  ;; 1 cycle
        MOV       ES, #BYTE3(??get_resp1_0)  ;; 1 cycle
        MOV       CS, #BYTE3(_get_resp1)  ;; 1 cycle
        BR        N:?L_VSWITCH_L10   ;; 3 cycles
        ; ------------------------------------- Block: 9 cycles
//  449     {
//  450     case 0x002a0000:                        /* logical device name */
//  451       log_name2(0, 0, 42, 0, 0);
??get_resp1_24:
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x2A           ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _log_name2
        CALL      _log_name2         ;; 3 cycles
//  452       break;
        BR        S:??compartment3_69  ;; 3 cycles
        ; ------------------------------------- Block: 11 cycles
//  453     case 0x00600100:                        /* meter serial number */
//  454       log_name2(0, 0, 96, 1, 0);
??get_resp1_25:
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x1            ;; 1 cycle
        MOV       C, #0x60           ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _log_name2
        CALL      _log_name2         ;; 3 cycles
//  455       break;
        BR        S:??compartment3_69  ;; 3 cycles
        ; ------------------------------------- Block: 11 cycles
//  456     case 0x01608009:                        /* pcb serial number */
//  457       log_name2(1, 0, 96, 128, 9);
??get_resp1_26:
        MOV       E, #0x9            ;; 1 cycle
        MOV       B, #0x80           ;; 1 cycle
        MOV       C, #0x60           ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MOV       A, #0x1            ;; 1 cycle
          CFI FunCall _log_name2
        CALL      _log_name2         ;; 3 cycles
//  458       break;
        BR        S:??compartment3_69  ;; 3 cycles
        ; ------------------------------------- Block: 11 cycles
//  459     case 0x01608012:                        /* utility ID */
//  460       log_name2(1, 0, 96, 128, 18);
??get_resp1_27:
        MOV       E, #0x12           ;; 1 cycle
        MOV       B, #0x80           ;; 1 cycle
        MOV       C, #0x60           ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MOV       A, #0x1            ;; 1 cycle
          CFI FunCall _log_name2
        CALL      _log_name2         ;; 3 cycles
//  461       break;
        BR        S:??compartment3_69  ;; 3 cycles
          CFI FunCall _fill_0b
        ; ------------------------------------- Block: 11 cycles
//  462     default:
//  463       fill_0b();
??get_resp1_28:
        CALL      _fill_0b           ;; 3 cycles
//  464       break;
        ; ------------------------------------- Block: 3 cycles
//  465     }
//  466     break;
??compartment3_69:
        BR        N:??compartment3_29  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
//  467   case 0x0102:
//  468     switch(obis1)
??compartment3_9:
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        MOVW      HL, #LWRD(??get_resp1_1)  ;; 1 cycle
        MOV       ES, #BYTE3(??get_resp1_1)  ;; 1 cycle
        MOV       CS, #BYTE3(_get_resp1)  ;; 1 cycle
        BR        N:?L_VSWITCH_L10   ;; 3 cycles
        ; ------------------------------------- Block: 9 cycles
//  469     {
//  470     case 0x002a0000:                        /* logical device name */
//  471       info[k++]= 0;
??get_resp1_29:
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
//  472       logical_device_name(2);
        MOV       A, #0x2            ;; 1 cycle
          CFI FunCall _logical_device_name
        CALL      _logical_device_name  ;; 3 cycles
//  473       break;
        BR        N:??compartment3_29  ;; 3 cycles
        ; ------------------------------------- Block: 12 cycles
//  474     case 0x00600100:                        /* meter serial number */
//  475       info[k++]= 0;
??get_resp1_30:
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
//  476       sr_no_ascii();
          CFI FunCall _sr_no_ascii
        CALL      _sr_no_ascii       ;; 3 cycles
//  477       break;
        BR        N:??compartment3_29  ;; 3 cycles
        ; ------------------------------------- Block: 11 cycles
//  478     case 0x01608009:
//  479       if(asso3_flag == 1)
??get_resp1_31:
        CMP       N:_asso3_flag, #0x1  ;; 1 cycle
        BNZ       ??compartment3_70  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  480       {
//  481         eprom_read(TOU_VAR_SAVE,0,PAGE_1,AUTO_CALC);
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x7F0         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
//  482         if(opr_data[10] != 0x01)
        CMP       N:_opr_data+10, #0x1  ;; 1 cycle
        BZ        ??compartment3_71  ;; 4 cycles
        ; ------------------------------------- Block: 12 cycles
//  483         {
//  484           val_4byt2(0x00, 0x00, 0x00, 0x00);
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
        BR        S:??compartment3_72  ;; 3 cycles
        ; ------------------------------------- Block: 10 cycles
//  485         }
//  486         else
//  487         {
//  488           val_4byt2(opr_data[6], opr_data[7], opr_data[8], opr_data[9]);
??compartment3_71:
        MOV       B, N:_opr_data+9   ;; 1 cycle
        MOV       C, N:_opr_data+8   ;; 1 cycle
        MOV       X, N:_opr_data+7   ;; 1 cycle
        MOV       A, N:_opr_data+6   ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
        BR        S:??compartment3_72  ;; 3 cycles
          CFI FunCall _fill_0d
        ; ------------------------------------- Block: 10 cycles
//  489         }
//  490       }
//  491       else
//  492       {
//  493         fill_0d();
??compartment3_70:
        CALL      _fill_0d           ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
//  494       }
//  495       
//  496       break;
??compartment3_72:
        BR        N:??compartment3_29  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
//  497     case 0x0160800E:
//  498       if(asso3_flag == 1)
??get_resp1_32:
        CMP       N:_asso3_flag, #0x1  ;; 1 cycle
        BNZ       ??compartment3_73  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  499       {
//  500         info[k++]= 0;
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
//  501         fill_pcb_firm_ver_calib_status();
          CFI FunCall _fill_pcb_firm_ver_calib_status
        CALL      _fill_pcb_firm_ver_calib_status  ;; 3 cycles
        BR        S:??compartment3_74  ;; 3 cycles
          CFI FunCall _fill_0d
        ; ------------------------------------- Block: 11 cycles
//  502       }
//  503       else
//  504       {
//  505         fill_0d();
??compartment3_73:
        CALL      _fill_0d           ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
//  506       }
//  507       
//  508       break;
??compartment3_74:
        BR        N:??compartment3_29  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
//  509     case 0x01608011: /* RTC calibration status */
//  510       if(asso3_flag == 1)
??get_resp1_33:
        CMP       N:_asso3_flag, #0x1  ;; 1 cycle
        BNZ       ??compartment3_75  ;; 4 cycles
          CFI FunCall _fill_rtc_calib
        ; ------------------------------------- Block: 5 cycles
//  511       {
//  512         fill_rtc_calib();
        CALL      _fill_rtc_calib    ;; 3 cycles
        BR        S:??compartment3_76  ;; 3 cycles
          CFI FunCall _fill_0d
        ; ------------------------------------- Block: 6 cycles
//  513       }
//  514       else
//  515       {
//  516         fill_0d();
??compartment3_75:
        CALL      _fill_0d           ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
//  517       }
//  518       
//  519       break;
??compartment3_76:
        BR        N:??compartment3_29  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
//  520     case 0x0160803C:            //Calibration enable and reset(1.0.96.128.60.255)
//  521       if(asso3_flag == 1)
??get_resp1_34:
        CMP       N:_asso3_flag, #0x1  ;; 1 cycle
        BNZ       ??compartment3_77  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  522       {
//  523         if(cal_done_f != 1)
        CMP       N:_cal_done_f, #0x1  ;; 1 cycle
        BZ        ??compartment3_78  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  524         {
//  525           val_1byt(1,1);
        MOV       X, #0x1            ;; 1 cycle
        MOV       A, #0x1            ;; 1 cycle
          CFI FunCall _val_1byt
        CALL      _val_1byt          ;; 3 cycles
        BR        S:??compartment3_79  ;; 3 cycles
        ; ------------------------------------- Block: 8 cycles
//  526         }
//  527         else
//  528         {	
//  529           val_1byt(0,1);
??compartment3_78:
        MOV       X, #0x1            ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_1byt
        CALL      _val_1byt          ;; 3 cycles
        BR        S:??compartment3_79  ;; 3 cycles
          CFI FunCall _fill_0d
        ; ------------------------------------- Block: 8 cycles
//  530         }
//  531       }
//  532       else
//  533         fill_0d();
??compartment3_77:
        CALL      _fill_0d           ;; 3 cycles
//  534       break;
        BR        S:??compartment3_79  ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
//  535     case 0x0160803E:                                  //instant Eprom testing
//  536       if((asso3_flag==1) && (0==fg_done_f))
??get_resp1_35:
        CMP       N:_asso3_flag, #0x1  ;; 1 cycle
        BNZ       ??compartment3_80  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        MOVW      HL, #LWRD(_flag1)  ;; 1 cycle
        MOV1      CY, [HL].1         ;; 1 cycle
        BC        ??compartment3_80  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  537       {
//  538         // structure--- No of Memories
//  539         // MemoryStatus1,MemoryStatus2--- Memory Type	1:16K, 2:256K, 3:512K, 4:1Mb
//  540         info[k++]=0x00;
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
//  541         structure(NoOfMemoryPresent);
        MOV       A, #0x2            ;; 1 cycle
          CFI FunCall _structure
        CALL      _structure         ;; 3 cycles
//  542         val_1byt(MemoryStatus1,0);
        MOV       X, #0x0            ;; 1 cycle
        MOV       A, N:_MemoryStatus1  ;; 1 cycle
          CFI FunCall _val_1byt
        CALL      _val_1byt          ;; 3 cycles
//  543 #if NoOfMemoryPresent ==2
//  544         val_1byt(MemoryStatus2,0);
        MOV       X, #0x0            ;; 1 cycle
        MOV       A, N:_MemoryStatus2  ;; 1 cycle
          CFI FunCall _val_1byt
        CALL      _val_1byt          ;; 3 cycles
        BR        S:??compartment3_79  ;; 3 cycles
          CFI FunCall _fill_0d
        ; ------------------------------------- Block: 22 cycles
//  545 #endif
//  546       }
//  547       else
//  548       {
//  549         fill_0d();
??compartment3_80:
        CALL      _fill_0d           ;; 3 cycles
//  550       }
//  551       break;
        BR        S:??compartment3_79  ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
//  552     case 0x01608012: /* utility id */
//  553       if(asso3_flag == 1)
??get_resp1_36:
        CMP       N:_asso3_flag, #0x1  ;; 1 cycle
        BNZ       ??compartment3_81  ;; 4 cycles
          CFI FunCall _fill_utility_id
        ; ------------------------------------- Block: 5 cycles
//  554       {
//  555         fill_utility_id();
        CALL      _fill_utility_id   ;; 3 cycles
        BR        S:??compartment3_79  ;; 3 cycles
          CFI FunCall _fill_0d
        ; ------------------------------------- Block: 6 cycles
//  556       }
//  557       else
//  558       {
//  559         fill_0d();
??compartment3_81:
        CALL      _fill_0d           ;; 3 cycles
//  560       }
//  561       
//  562       break;
        BR        S:??compartment3_79  ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
//  563       
//  564     case 0x00608200:
//  565       ////////////            fill_lcdmap_zero();
//  566       ////////////            lcd_map[4]|=LCD_7r;
//  567       ////////////            lcd_map[5]|=LCD_7e;
//  568       ////////////            lcd_map[6]|=LCD_7A;
//  569       ////////////            lcd_map[7]|=LCD_7d;
//  570       ////////////            lcd_map[8]=0x00;
//  571       ////////////            lcd_map[9]|=LCD_7C;
//  572       ////////////            lcd_map[10]|=LCD_7r;
//  573       ////////////            lcd_map[11]|=LCD_7C;
//  574       ////////////            write_to_lcd();
//  575       ////////////            
//  576       ////////////            if(1==pwrup_crc_cal_f)
//  577       ////////////            {
//  578       ////////////                pwrup_crc_cal_f=0;
//  579       ////////////                hw_crc = calculate_flash_crc32_hw(FLASH_START_ADDRESS,FLASH_LENGTH);
//  580       ////////////            }
//  581       ////////////            long_int= hw_crc;
//  582       ////////////            long_to_char_array();
//  583       ////////////            val_4byt2(char_array[0],char_array[1],char_array[2],char_array[3]);
//  584       break;
//  585     case 0x00608300:                                  //instant Hardware switches testing
//  586       if(asso3_flag == 1 && fg_done_f == 0)
??get_resp1_37:
        CMP       N:_asso3_flag, #0x1  ;; 1 cycle
        BNZ       ??compartment3_82  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        MOVW      HL, #LWRD(_flag1)  ;; 1 cycle
        MOV1      CY, [HL].1         ;; 1 cycle
        BC        ??compartment3_82  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  587       {
//  588         hardware_testing_status(1);
        MOV       A, #0x1            ;; 1 cycle
          CFI FunCall _hardware_testing_status
        CALL      _hardware_testing_status  ;; 3 cycles
        BR        S:??compartment3_29  ;; 3 cycles
          CFI FunCall _fill_0d
        ; ------------------------------------- Block: 7 cycles
//  589       }
//  590       else
//  591       {
//  592         fill_0d();
??compartment3_82:
        CALL      _fill_0d           ;; 3 cycles
//  593       }
//  594       break;
        BR        S:??compartment3_29  ;; 3 cycles
          CFI FunCall _fill_0b
        ; ------------------------------------- Block: 6 cycles
//  595     default:
//  596       fill_0b();
??get_resp1_38:
        CALL      _fill_0b           ;; 3 cycles
//  597       break;
        ; ------------------------------------- Block: 3 cycles
//  598     }
//  599     break;
??compartment3_79:
        BR        S:??compartment3_29  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
//  600   case 0x3f02:
//  601     switch(obis1)
??compartment3_24:
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x60          ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0xA01         ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??get_resp1_39:
        BNZ       ??compartment3_83  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
//  602     {
//  603     case 0x00600A01:
//  604       if(asso3_flag == 1)
        CMP       N:_asso3_flag, #0x1  ;; 1 cycle
        BNZ       ??compartment3_84  ;; 4 cycles
          CFI FunCall _tamper_status
        ; ------------------------------------- Block: 5 cycles
//  605       {
//  606         tamper_status();
        CALL      _tamper_status     ;; 3 cycles
        BR        S:??compartment3_29  ;; 3 cycles
          CFI FunCall _fill_0d
        ; ------------------------------------- Block: 6 cycles
//  607       }
//  608       else
//  609       {
//  610         fill_0d();
??compartment3_84:
        CALL      _fill_0d           ;; 3 cycles
//  611       }
//  612       break;
        BR        S:??compartment3_29  ;; 3 cycles
          CFI FunCall _fill_0b
        ; ------------------------------------- Block: 6 cycles
//  613       
//  614       ////////////        case 0x01600500:
//  615       ////////////            if(asso3_flag == 1)
//  616       ////////////            {
//  617       ////////////                self_diag_status();
//  618       ////////////            }
//  619       ////////////            else
//  620       ////////////            {
//  621       ////////////                fill_0d();
//  622       ////////////            }
//  623       ////////////            break;
//  624       ////////////            
//  625     default:
//  626       fill_0b();
??compartment3_83:
        CALL      _fill_0b           ;; 3 cycles
//  627       break;
//  628     }
//  629     break;
        BR        S:??compartment3_29  ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
//  630   case 0x3f03:
//  631     switch(obis1)
??compartment3_25:
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        CMPW      AX, #0xA01         ;; 1 cycle
        BNZ       ??compartment3_85  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x60          ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ; ------------------------------------- Block: 3 cycles
??compartment3_85:
        BZ        ??compartment3_86  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
        CMPW      AX, #0x500         ;; 1 cycle
        BNZ       ??compartment3_87  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x160         ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        ; ------------------------------------- Block: 3 cycles
??compartment3_87:
        BZ        ??compartment3_88  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
        BR        S:??compartment3_89  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
//  632     {
//  633     case 0x00600A01:                        /* tamper status */
//  634       if(asso3_flag == 1)
??compartment3_86:
        CMP       N:_asso3_flag, #0x1  ;; 1 cycle
        BNZ       ??compartment3_90  ;; 4 cycles
          CFI FunCall _fill_0d
        ; ------------------------------------- Block: 5 cycles
//  635       {
//  636         /*	tamper_mapping();  */
//  637         fill_0d();
        CALL      _fill_0d           ;; 3 cycles
        BR        S:??compartment3_29  ;; 3 cycles
          CFI FunCall _fill_0d
        ; ------------------------------------- Block: 6 cycles
//  638       }
//  639       else
//  640       {
//  641         fill_0d();
??compartment3_90:
        CALL      _fill_0d           ;; 3 cycles
//  642       }
//  643       break;
        BR        S:??compartment3_29  ;; 3 cycles
          CFI FunCall _fill_0d
        ; ------------------------------------- Block: 6 cycles
//  644       
//  645     case 0x01600500:
//  646       fill_0d();
??compartment3_88:
        CALL      _fill_0d           ;; 3 cycles
//  647       break;
        BR        S:??compartment3_29  ;; 3 cycles
          CFI FunCall _fill_0b
        ; ------------------------------------- Block: 6 cycles
//  648       
//  649     default:
//  650       fill_0b();
??compartment3_89:
        CALL      _fill_0b           ;; 3 cycles
//  651       break;
//  652     }
//  653     break;
        BR        S:??compartment3_29  ;; 3 cycles
          CFI FunCall _fill_0b
        ; ------------------------------------- Block: 6 cycles
//  654   default:
//  655     fill_0b();
??compartment3_26:
        CALL      _fill_0b           ;; 3 cycles
//  656     break;
        ; ------------------------------------- Block: 3 cycles
//  657   }
//  658 }
??compartment3_29:
        ADDW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock2
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 1057 cycles
//  659 
//  660 /* ************************************************************************* */

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock3 Using cfiCommon0
          CFI Function _get_resp
        CODE
//  661 void get_resp(void)
//  662 {
_get_resp:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 42
        SUBW      SP, #0x2A          ;; 1 cycle
          CFI CFA SP+46
//  663   uint16_t classatt, i_g, address, lu16_add, lu16_add_temp;
//  664   unsigned long int obis1;
//  665   uint8_t class_att[2], ident= 1, not_found_f= 1, lu8_i, lu8_j, lu8_k,temp_arr[2];
        MOV       A, #0x1            ;; 1 cycle
        MOV       [SP+0x09], A       ;; 1 cycle
        MOV       A, #0x1            ;; 1 cycle
        MOV       [SP+0x10], A       ;; 1 cycle
//  666   
//  667   class_att[0]= info[7];
        MOV       A, N:_info+7       ;; 1 cycle
        MOV       [SP+0x0E], A       ;; 1 cycle
//  668   class_att[1]= attribute_id;
        MOV       A, N:_attribute_id  ;; 1 cycle
        MOV       [SP+0x0F], A       ;; 1 cycle
//  669   if(attribute_id == 1)
        CMP       N:_attribute_id, #0x1  ;; 1 cycle
        BNZ       ??compartment3_91  ;; 4 cycles
        ; ------------------------------------- Block: 14 cycles
//  670   {
//  671     class_att[0]= 0;
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP+0x0E], A       ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
//  672   }
//  673   
//  674   char_array_ptr= &class_att[0];
??compartment3_91:
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0xE           ;; 1 cycle
        MOVW      N:_char_array_ptr, AX  ;; 1 cycle
//  675   classatt= (uint16_t)class_att[0] * 256 + class_att[1]; /* char_array_to_int(); */
        MOV       A, [SP+0x0E]       ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x100         ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, [SP+0x0F]       ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       D, #0x0            ;; 1 cycle
        MOVW      AX, HL             ;; 1 cycle
        ADDW      AX, DE             ;; 1 cycle
        MOVW      [SP+0x12], AX      ;; 1 cycle
//  676   obis1= obis_short_cal(&info[8]);
        MOVW      AX, #LWRD(_info+8)  ;; 1 cycle
          CFI FunCall _obis_short_cal
        CALL      _obis_short_cal    ;; 3 cycles
        MOVW      [SP+0x04], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      [SP+0x06], AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  677   
//  678   switch(classatt)
        MOVW      AX, [SP+0x12]      ;; 1 cycle
        SUBW      AX, #0x1           ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??compartment3_92  ;; 4 cycles
        ; ------------------------------------- Block: 30 cycles
        SUBW      AX, #0x101         ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??compartment3_93  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        SUBW      AX, #0x200         ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??compartment3_94  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        SUBW      AX, #0x1           ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??compartment3_95  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        SUBW      AX, #0xFF          ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??compartment3_96  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        SUBW      AX, #0x1           ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??compartment3_97  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        SUBW      AX, #0x1           ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??compartment3_98  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        SUBW      AX, #0x1           ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??compartment3_99  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        SUBW      AX, #0x2FD         ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??compartment3_100  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        SUBW      AX, #0x1           ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??compartment3_101  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        SUBW      AX, #0x1           ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??compartment3_102  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        SUBW      AX, #0x1           ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??compartment3_103  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        SUBW      AX, #0x1           ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??compartment3_104  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        SUBW      AX, #0x1           ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??compartment3_105  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        SUBW      AX, #0x1           ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??compartment3_106  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        SUBW      AX, #0xFA          ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??compartment3_107  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        SUBW      AX, #0x1           ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??compartment3_108  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        SUBW      AX, #0x1           ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??compartment3_109  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        SUBW      AX, #0x1           ;; 1 cycle
        CMPW      AX, #0x4           ;; 1 cycle
        SKNC                         ;; 4 cycles
        BR        N:??compartment3_110  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        SUBW      AX, #0x4           ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??compartment3_111  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        SUBW      AX, #0xF9          ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??compartment3_112  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        SUBW      AX, #0x600         ;; 1 cycle
        CMPW      AX, #0x8           ;; 1 cycle
        SKNC                         ;; 4 cycles
        BR        N:??compartment3_113  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        SUBW      AX, #0x200         ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??compartment3_114  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        SUBW      AX, #0x300         ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??compartment3_115  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        SUBW      AX, #0x1           ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??compartment3_116  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        SUBW      AX, #0x1           ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??compartment3_117  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        SUBW      AX, #0x1           ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??compartment3_118  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        SUBW      AX, #0x1           ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??compartment3_115  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        SUBW      AX, #0x1           ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??compartment3_116  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        SUBW      AX, #0x1           ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??compartment3_117  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        SUBW      AX, #0x1           ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??compartment3_118  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        SUBW      AX, #0x1           ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??compartment3_119  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        SUBW      AX, #0x1F8         ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??compartment3_120  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        SUBW      AX, #0x1           ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??compartment3_121  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        SUBW      AX, #0x1           ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??compartment3_122  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        SUBW      AX, #0x28FE        ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??compartment3_123  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        BR        N:??compartment3_124  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
//  679   {
//  680   case 0x0001:
//  681     if(((asso1_flag) || (asso2_flag)) && (obis1 == 0x00280001))
??compartment3_92:
        CMP0      N:_asso1_flag      ;; 1 cycle
        BNZ       ??compartment3_125  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        CMP0      N:_asso2_flag      ;; 1 cycle
        BZ        ??compartment3_126  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
??compartment3_125:
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x28          ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x1           ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??get_resp_13:
        SKNZ                         ;; 1 cycle
          CFI FunCall _fill_0b
        ; ------------------------------------- Block: 1 cycles
//  682     {
//  683       fill_0b();
        CALL      _fill_0b           ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
//  684     }
//  685     
//  686     for(i_g= 3; i_g <= (OBJ_LIST[2] * 9 + 3); i_g+= 9)
??compartment3_126:
        MOVW      AX, #0x3           ;; 1 cycle
        MOVW      [SP+0x0A], AX      ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
??get_resp_14:
        MOVW      AX, [SP+0x0A]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV       X, N:_OBJ_LIST+2   ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x9           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #0x3           ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        SKNC                         ;; 4 cycles
        BR        N:??compartment3_127  ;; 4 cycles
        ; ------------------------------------- Block: 13 cycles
//  687     {
//  688       if((OBJ_LIST[i_g + 1] == info[7]) && (OBJ_LIST[i_g + 3] == info[8]) && (OBJ_LIST[i_g + 4] == info[9]) && (OBJ_LIST[i_g + 5] == info[10]) && (OBJ_LIST[i_g + 6] == info[11]) && (OBJ_LIST[i_g + 7] == info[12]))
        MOVW      AX, [SP+0x0A]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOV       A, (_OBJ_LIST+1)[BC]  ;; 1 cycle
        CMP       A, N:_info+7       ;; 1 cycle
        SKZ                          ;; 4 cycles
        BR        N:??compartment3_128  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
        MOVW      AX, [SP+0x0A]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOV       A, (_OBJ_LIST+3)[BC]  ;; 1 cycle
        CMP       A, N:_info+8       ;; 1 cycle
        SKZ                          ;; 4 cycles
        BR        N:??compartment3_128  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
        MOVW      AX, [SP+0x0A]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOV       A, (_OBJ_LIST+4)[BC]  ;; 1 cycle
        CMP       A, N:_info+9       ;; 1 cycle
        BNZ       ??compartment3_128  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
        MOVW      AX, [SP+0x0A]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOV       A, (_OBJ_LIST+5)[BC]  ;; 1 cycle
        CMP       A, N:_info+10      ;; 1 cycle
        BNZ       ??compartment3_128  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
        MOVW      AX, [SP+0x0A]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOV       A, (_OBJ_LIST+6)[BC]  ;; 1 cycle
        CMP       A, N:_info+11      ;; 1 cycle
        BNZ       ??compartment3_128  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
        MOVW      AX, [SP+0x0A]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOV       A, (_OBJ_LIST+7)[BC]  ;; 1 cycle
        CMP       A, N:_info+12      ;; 1 cycle
        BNZ       ??compartment3_128  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
//  689       {
//  690         not_found_f= 0;
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP+0x10], A       ;; 1 cycle
//  691         if((obis_code[2] == 40) && (((obis_code[4] == 3) && (asso2_flag == 1)) || ((obis_code[4] == 2) && (asso1_flag == 1)) || ((obis_code[4] == 1) || (obis_code[4] == 0))))
        CMP       N:_obis_code+2, #0x28  ;; 1 cycle
        BNZ       ??compartment3_129  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        CMP       N:_obis_code+4, #0x3  ;; 1 cycle
        BNZ       ??compartment3_130  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        CMP       N:_asso2_flag, #0x1  ;; 1 cycle
        BZ        ??compartment3_131  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
??compartment3_130:
        CMP       N:_obis_code+4, #0x2  ;; 1 cycle
        BNZ       ??compartment3_132  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        CMP       N:_asso1_flag, #0x1  ;; 1 cycle
        BZ        ??compartment3_131  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
??compartment3_132:
        CMP       N:_obis_code+4, #0x1  ;; 1 cycle
        BZ        ??compartment3_131  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        CMP0      N:_obis_code+4     ;; 1 cycle
        BNZ       ??compartment3_129  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  692         {
//  693           log_name2(info[8], info[9], info[10], info[11], info[12]);
??compartment3_131:
        MOV       A, N:_info+12      ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       B, N:_info+11      ;; 1 cycle
        MOV       C, N:_info+10      ;; 1 cycle
        MOV       X, N:_info+9       ;; 1 cycle
        MOV       A, N:_info+8       ;; 1 cycle
          CFI FunCall _log_name2
        CALL      _log_name2         ;; 3 cycles
        BR        S:??compartment3_127  ;; 3 cycles
        ; ------------------------------------- Block: 12 cycles
//  694         }
//  695         else if(obis_code[2] != 40)
??compartment3_129:
        CMP       N:_obis_code+2, #0x28  ;; 1 cycle
        BZ        ??compartment3_127  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  696         {
//  697           log_name2(info[8], info[9], info[10], info[11], info[12]);
        MOV       A, N:_info+12      ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       B, N:_info+11      ;; 1 cycle
        MOV       C, N:_info+10      ;; 1 cycle
        MOV       X, N:_info+9       ;; 1 cycle
        MOV       A, N:_info+8       ;; 1 cycle
          CFI FunCall _log_name2
        CALL      _log_name2         ;; 3 cycles
//  698         }
//  699         
//  700         break;
        BR        S:??compartment3_127  ;; 3 cycles
        ; ------------------------------------- Block: 12 cycles
//  701       }
//  702     }
??compartment3_128:
        MOVW      AX, [SP+0x0A]      ;; 1 cycle
        ADDW      AX, #0x9           ;; 1 cycle
        MOVW      [SP+0x0A], AX      ;; 1 cycle
        BR        N:??get_resp_14    ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
//  703     
//  704     if((i_g > (OBJ_LIST[2] * 9)) && (not_found_f == 1))
??compartment3_127:
        MOVW      AX, [SP+0x0A]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV       X, N:_OBJ_LIST+2   ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x9           ;; 1 cycle
        MULHU                        ;; 2 cycles
        CMPW      AX, HL             ;; 1 cycle
        BNC       ??compartment3_133  ;; 4 cycles
        ; ------------------------------------- Block: 12 cycles
        MOV       A, [SP+0x10]       ;; 1 cycle
        CMP       A, #0x1            ;; 1 cycle
        SKNZ                         ;; 1 cycle
          CFI FunCall _fill_0b
        ; ------------------------------------- Block: 3 cycles
//  705     {
//  706       fill_0b();
        CALL      _fill_0b           ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
//  707     }
//  708     
//  709     break;
??compartment3_133:
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
//  710   case 0x0102:
//  711     switch(obis1)
??compartment3_93:
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, #LWRD(??get_resp_0)  ;; 1 cycle
        MOV       ES, #BYTE3(??get_resp_0)  ;; 1 cycle
        MOV       CS, #BYTE3(_get_resp)  ;; 1 cycle
        BR        N:?L_VSWITCH_L10   ;; 3 cycles
        ; ------------------------------------- Block: 9 cycles
//  712     {
//  713     case 0x00000101:                        /* available billing period */
//  714       if(md_reset_count <= MAX_BILL)
??get_resp_15:
        MOV       A, N:_MAX_BILL     ;; 1 cycle
        CMP       A, N:_md_reset_count  ;; 1 cycle
        BC        ??compartment3_135  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  715       {
//  716         lu8_i= md_reset_count;
        MOV       A, N:_md_reset_count  ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        BR        S:??compartment3_136  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  717       }
//  718       else
//  719       {
//  720         lu8_i= MAX_BILL;
??compartment3_135:
        MOV       A, N:_MAX_BILL     ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
//  721       }
//  722       unsigned8(lu8_i);
??compartment3_136:
        MOV       A, [SP]            ;; 1 cycle
          CFI FunCall _unsigned8
        CALL      _unsigned8         ;; 3 cycles
//  723       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
//  724       
//  725     case 0x002A0000:
//  726       info[k++]=0;
??get_resp_16:
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
//  727       logical_device_name(2);
        MOV       A, #0x2            ;; 1 cycle
          CFI FunCall _logical_device_name
        CALL      _logical_device_name  ;; 3 cycles
//  728       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 12 cycles
//  729       ////////            
//  730     case 0x00600100:
//  731       info[k++]= 0;
??get_resp_17:
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
//  732       sr_no_ascii();
          CFI FunCall _sr_no_ascii
        CALL      _sr_no_ascii       ;; 3 cycles
//  733       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 11 cycles
//  734       
//  735     case 0x01608009:                        /* PCB TRACKING READ CASE */
//  736       eprom_read(TOU_VAR_SAVE,0,PAGE_1,AUTO_CALC);              /* pcb serial no */
??get_resp_18:
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x7F0         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
//  737       if(opr_data[10] != 0x01)
        CMP       N:_opr_data+10, #0x1  ;; 1 cycle
        BZ        ??compartment3_137  ;; 4 cycles
        ; ------------------------------------- Block: 12 cycles
//  738       {
//  739         val_4byt2(0x00, 0x00, 0x00, 0x00);
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
        BR        S:??compartment3_138  ;; 3 cycles
        ; ------------------------------------- Block: 10 cycles
//  740       }
//  741       else
//  742       {
//  743         val_4byt2(opr_data[6], opr_data[7], opr_data[8], opr_data[9]);
??compartment3_137:
        MOV       B, N:_opr_data+9   ;; 1 cycle
        MOV       C, N:_opr_data+8   ;; 1 cycle
        MOV       X, N:_opr_data+7   ;; 1 cycle
        MOV       A, N:_opr_data+6   ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
//  744       }
//  745       break;
??compartment3_138:
        BR        N:??compartment3_134  ;; 3 cycles
          CFI FunCall _fill_pcb_firm_ver_calib_status
        ; ------------------------------------- Block: 3 cycles
//  746       
//  747     case 0x0160800E:                        /* pcb firmware version and pcb calibration status */
//  748       fill_pcb_firm_ver_calib_status();
??get_resp_19:
        CALL      _fill_pcb_firm_ver_calib_status  ;; 3 cycles
//  749       break;
        BR        N:??compartment3_134  ;; 3 cycles
          CFI FunCall _fill_rtc_calib
        ; ------------------------------------- Block: 6 cycles
//  750       
//  751     case 0x01608011:
//  752       fill_rtc_calib();
??get_resp_20:
        CALL      _fill_rtc_calib    ;; 3 cycles
//  753       break;
        BR        N:??compartment3_134  ;; 3 cycles
          CFI FunCall _fill_utility_id
        ; ------------------------------------- Block: 6 cycles
//  754       
//  755     case 0x01608012:                        /* utility id */
//  756       fill_utility_id();
??get_resp_21:
        CALL      _fill_utility_id   ;; 3 cycles
//  757       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
//  758       
//  759     case 0x00600700:
//  760       long_int= tpr.power_fail.count;
??get_resp_22:
        MOV       X, N:_tpr+605      ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        MOVW      S:_long_int, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_long_int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  761       int_into_char_array(long_int,char_array);
        MOVW      BC, #LWRD(_char_array)  ;; 1 cycle
        MOVW      HL, S:_long_int+2  ;; 1 cycle
        MOVW      DE, S:_long_int    ;; 1 cycle
        MOVW      AX, DE             ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  762       val_2byt2(char_array[0], char_array[1]);
        MOV       X, N:_char_array+1  ;; 1 cycle
        MOV       A, N:_char_array   ;; 1 cycle
          CFI FunCall _val_2byt2
        CALL      _val_2byt2         ;; 3 cycles
//  763       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 22 cycles
//  764       
//  765     case 0x005e5b00:
//  766       val_2byt2(tpr.cum_tpr_count/256, tpr.cum_tpr_count%256);
??get_resp_23:
        MOVW      AX, N:_tpr+30      ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      AX, N:_tpr+30      ;; 1 cycle
        CLRB      X                  ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
          CFI FunCall _val_2byt2
        CALL      _val_2byt2         ;; 3 cycles
//  767       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 17 cycles
//  768       
//  769     case 0x00000100:
//  770       val_4byt2(0, 0, 0, md_reset_count);
??get_resp_24:
        MOV       B, N:_md_reset_count  ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
//  771       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 10 cycles
//  772       
//  773     case 0x00600200:
//  774       unsigned8(seq_no_transaction);
??get_resp_25:
        MOVW      AX, N:_seq_no_transaction  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
          CFI FunCall _unsigned8
        CALL      _unsigned8         ;; 3 cycles
//  775       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 8 cycles
//  776       
//  777     case 0x005e5b09:
//  778       unsigned8(4);                       /* Meter_Type=4: for WC  */
??get_resp_26:
        MOV       A, #0x4            ;; 1 cycle
          CFI FunCall _unsigned8
        CALL      _unsigned8         ;; 3 cycles
//  779       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
//  780       
//  781     case 0x005e5b0b:
//  782       info[k++]= 0x00;
??get_resp_27:
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
//  783       fill_meter_category();
          CFI FunCall _fill_meter_category
        CALL      _fill_meter_category  ;; 3 cycles
//  784       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 11 cycles
//  785       
//  786     case 0x005e5b0c:                        /* Current Rating */
//  787       info[k++]= 0x00;
??get_resp_28:
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
//  788       current_rating();
          CFI FunCall _current_rating
        CALL      _current_rating    ;; 3 cycles
//  789       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 11 cycles
//  790       
//  791     case 0x00600101:
//  792       info[k++]= 0x00;
??get_resp_29:
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
//  793       fill_manufacturer_name();
          CFI FunCall _fill_manufacturer_name
        CALL      _fill_manufacturer_name  ;; 3 cycles
//  794       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 11 cycles
//  795       
//  796     case 0x01000200:
//  797       info[k++]= 0x00;
??get_resp_30:
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
//  798       fill_firmware_version();
          CFI FunCall _fill_firmware_version
        CALL      _fill_firmware_version  ;; 3 cycles
//  799       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 11 cycles
//  800       
//  801     case 0x01000402:
//  802       long_int= CTR;
??get_resp_31:
        MOVW      AX, N:_CTR         ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        MOVW      S:_long_int, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_long_int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  803       int_into_char_array(long_int,char_array);
        MOVW      BC, #LWRD(_char_array)  ;; 1 cycle
        MOVW      HL, S:_long_int+2  ;; 1 cycle
        MOVW      DE, S:_long_int    ;; 1 cycle
        MOVW      AX, DE             ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  804       val_2byt2(char_array[0], char_array[1]);
        MOV       X, N:_char_array+1  ;; 1 cycle
        MOV       A, N:_char_array   ;; 1 cycle
          CFI FunCall _val_2byt2
        CALL      _val_2byt2         ;; 3 cycles
//  805       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 21 cycles
//  806       
//  807     case 0x01000403:
//  808       long_int= PTR;
??get_resp_32:
        MOVW      AX, N:_PTR         ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        MOVW      S:_long_int, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_long_int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  809       int_into_char_array(long_int,char_array);
        MOVW      BC, #LWRD(_char_array)  ;; 1 cycle
        MOVW      HL, S:_long_int+2  ;; 1 cycle
        MOVW      DE, S:_long_int    ;; 1 cycle
        MOVW      AX, DE             ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  810       val_2byt2(char_array[0], char_array[1]);
        MOV       X, N:_char_array+1  ;; 1 cycle
        MOV       A, N:_char_array   ;; 1 cycle
          CFI FunCall _val_2byt2
        CALL      _val_2byt2         ;; 3 cycles
//  811       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 21 cycles
//  812       
//  813     case 0x00600104:                        /* year of manufacturing */
//  814       info[k++]=0x00;
??get_resp_33:
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
//  815       fill_yr_of_manufacture();
          CFI FunCall _fill_yr_of_manufacture
        CALL      _fill_yr_of_manufacture  ;; 3 cycles
//  816       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 11 cycles
//  817       
//  818     case 0x01000800:
//  819       if(md_type == 1)
??get_resp_34:
        CMP       N:_md_type, #0x1   ;; 1 cycle
        BNZ       ??compartment3_139  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  820       {
//  821         temp_arr[0]= (10800 / mdi_sel) / 256;
        MOVW      BC, #0x100         ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+48
        POP       HL                 ;; 1 cycle
          CFI CFA SP+46
        MOV       C, N:_mdi_sel      ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOVW      AX, #0x2A30        ;; 1 cycle
          CFI FunCall ?SI_DIV_L02
        CALL      N:?SI_DIV_L02      ;; 3 cycles
        PUSH      HL                 ;; 1 cycle
          CFI CFA SP+48
        POP       BC                 ;; 1 cycle
          CFI CFA SP+46
          CFI FunCall ?SI_DIV_L02
        CALL      N:?SI_DIV_L02      ;; 3 cycles
        MOV       A, X               ;; 1 cycle
        MOV       [SP+0x0C], A       ;; 1 cycle
//  822         temp_arr[1]= (10800 / mdi_sel) % 256;
        MOVW      BC, #0x100         ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+48
        POP       HL                 ;; 1 cycle
          CFI CFA SP+46
        MOV       C, N:_mdi_sel      ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOVW      AX, #0x2A30        ;; 1 cycle
          CFI FunCall ?SI_DIV_L02
        CALL      N:?SI_DIV_L02      ;; 3 cycles
        PUSH      HL                 ;; 1 cycle
          CFI CFA SP+48
        POP       BC                 ;; 1 cycle
          CFI CFA SP+46
          CFI FunCall ?SI_MOD_L02
        CALL      N:?SI_MOD_L02      ;; 3 cycles
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, L               ;; 1 cycle
        MOV       [SP+0x0D], A       ;; 1 cycle
        BR        S:??compartment3_140  ;; 3 cycles
        ; ------------------------------------- Block: 36 cycles
//  823       }
//  824       else
//  825       {
//  826         temp_arr[0]= (3600 / mdi_sel) / 256;
??compartment3_139:
        MOVW      BC, #0x100         ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+48
        POP       HL                 ;; 1 cycle
          CFI CFA SP+46
        MOV       C, N:_mdi_sel      ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOVW      AX, #0xE10         ;; 1 cycle
          CFI FunCall ?SI_DIV_L02
        CALL      N:?SI_DIV_L02      ;; 3 cycles
        PUSH      HL                 ;; 1 cycle
          CFI CFA SP+48
        POP       BC                 ;; 1 cycle
          CFI CFA SP+46
          CFI FunCall ?SI_DIV_L02
        CALL      N:?SI_DIV_L02      ;; 3 cycles
        MOV       A, X               ;; 1 cycle
        MOV       [SP+0x0C], A       ;; 1 cycle
//  827         temp_arr[1]= (3600 / mdi_sel) % 256;
        MOVW      BC, #0x100         ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+48
        POP       HL                 ;; 1 cycle
          CFI CFA SP+46
        MOV       C, N:_mdi_sel      ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOVW      AX, #0xE10         ;; 1 cycle
          CFI FunCall ?SI_DIV_L02
        CALL      N:?SI_DIV_L02      ;; 3 cycles
        PUSH      HL                 ;; 1 cycle
          CFI CFA SP+48
        POP       BC                 ;; 1 cycle
          CFI CFA SP+46
          CFI FunCall ?SI_MOD_L02
        CALL      N:?SI_MOD_L02      ;; 3 cycles
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, L               ;; 1 cycle
        MOV       [SP+0x0D], A       ;; 1 cycle
        ; ------------------------------------- Block: 33 cycles
//  828       }
//  829       val_2byt2(temp_arr[0],temp_arr[1]);
??compartment3_140:
        MOV       A, [SP+0x0D]       ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, [SP+0x0C]       ;; 1 cycle
          CFI FunCall _val_2byt2
        CALL      _val_2byt2         ;; 3 cycles
//  830       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 9 cycles
//  831       
//  832     case 0x01000804:
//  833       val_2byt2(((3600 / mdi_sel_ls) / 256),((3600 / mdi_sel_ls) % 256));
??get_resp_35:
        MOVW      BC, #0x100         ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+48
        POP       HL                 ;; 1 cycle
          CFI CFA SP+46
        MOV       C, N:_mdi_sel_ls   ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOVW      AX, #0xE10         ;; 1 cycle
          CFI FunCall ?SI_DIV_L02
        CALL      N:?SI_DIV_L02      ;; 3 cycles
        PUSH      HL                 ;; 1 cycle
          CFI CFA SP+48
        POP       BC                 ;; 1 cycle
          CFI CFA SP+46
          CFI FunCall ?SI_MOD_L02
        CALL      N:?SI_MOD_L02      ;; 3 cycles
        MOVW      HL, AX             ;; 1 cycle
        XCH       A, L               ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        XCH       A, L               ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      BC, #0x100         ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+48
        POP       HL                 ;; 1 cycle
          CFI CFA SP+46
        MOV       C, N:_mdi_sel_ls   ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOVW      AX, #0xE10         ;; 1 cycle
          CFI FunCall ?SI_DIV_L02
        CALL      N:?SI_DIV_L02      ;; 3 cycles
        PUSH      HL                 ;; 1 cycle
          CFI CFA SP+48
        POP       BC                 ;; 1 cycle
          CFI CFA SP+46
          CFI FunCall ?SI_DIV_L02
        CALL      N:?SI_DIV_L02      ;; 3 cycles
        MOV       A, X               ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        XCH       A, D               ;; 1 cycle
          CFI FunCall _val_2byt2
        CALL      _val_2byt2         ;; 3 cycles
//  834       break;
        BR        S:??compartment3_141  ;; 3 cycles
        ; ------------------------------------- Block: 45 cycles
//  835       
//  836     case 0x00600B00:
//  837       val_2byt2(0, tpr.vol_related_last);
??get_resp_36:
        MOV       X, N:_tpr+3        ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_2byt2
        CALL      _val_2byt2         ;; 3 cycles
//  838       break;
        BR        S:??compartment3_141  ;; 3 cycles
        ; ------------------------------------- Block: 8 cycles
//  839       
//  840     case 0x00600B01:
//  841       val_2byt2(0, tpr.current_related_last);
??get_resp_37:
        MOV       X, N:_tpr+7        ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_2byt2
        CALL      _val_2byt2         ;; 3 cycles
//  842       break;
        BR        S:??compartment3_141  ;; 3 cycles
        ; ------------------------------------- Block: 8 cycles
//  843       
//  844     case 0x00600B02:
//  845       val_2byt2(0, tpr.power_last);
??get_resp_38:
        MOV       X, N:_tpr+11       ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_2byt2
        CALL      _val_2byt2         ;; 3 cycles
//  846       break;
        BR        S:??compartment3_141  ;; 3 cycles
        ; ------------------------------------- Block: 8 cycles
//  847       
//  848     case 0x00600B03:
//  849       val_2byt2(0, tpr.transaction_last);
??get_resp_39:
        MOV       X, N:_tpr+15       ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_2byt2
        CALL      _val_2byt2         ;; 3 cycles
//  850       break;
        BR        S:??compartment3_141  ;; 3 cycles
        ; ------------------------------------- Block: 8 cycles
//  851       
//  852     case 0x00600B04:
//  853       val_2byt2(0, tpr.others_last);
??get_resp_40:
        MOV       X, N:_tpr+19       ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_2byt2
        CALL      _val_2byt2         ;; 3 cycles
//  854       break;
        BR        S:??compartment3_141  ;; 3 cycles
        ; ------------------------------------- Block: 8 cycles
//  855       
//  856     case 0x00600B05:
//  857       val_2byt2(0, tpr.non_roll_last);
??get_resp_41:
        MOV       X, N:_tpr+23       ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_2byt2
        CALL      _val_2byt2         ;; 3 cycles
//  858       break;
        BR        S:??compartment3_141  ;; 3 cycles
        ; ------------------------------------- Block: 8 cycles
//  859       
//  860     case 0x00600B63:
//  861       val_2byt2(0, tpr.debug_last);
??get_resp_42:
        MOVW      AX, N:_tpr+28      ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_2byt2
        CALL      _val_2byt2         ;; 3 cycles
//  862       break;    
        BR        S:??compartment3_141  ;; 3 cycles
          CFI FunCall _fill_0b
        ; ------------------------------------- Block: 8 cycles
//  863     default:
//  864       fill_0b();
??get_resp_43:
        CALL      _fill_0b           ;; 3 cycles
//  865       break;
        ; ------------------------------------- Block: 3 cycles
//  866     }
//  867     break;
??compartment3_141:
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
//  868     
//  869   case 0x0302:
//  870     switch(obis1)
??compartment3_94:
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, #LWRD(??get_resp_1)  ;; 1 cycle
        MOV       ES, #BYTE3(??get_resp_1)  ;; 1 cycle
        MOV       CS, #BYTE3(_get_resp)  ;; 1 cycle
        BR        N:?L_VSWITCH_L10   ;; 3 cycles
        ; ------------------------------------- Block: 9 cycles
//  871     {
//  872     case 0x00000102:                        /* bill date */
//  873       info[k++]= 0x00;
??get_resp_44:
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
//  874       fill_oprzero(16);
        MOV       A, #0x10           ;; 1 cycle
          CFI FunCall _fill_oprzero
        CALL      _fill_oprzero      ;; 3 cycles
//  875       if(bill_count != 0)
        CMP0      N:_bill_count      ;; 1 cycle
        BZ        ??compartment3_142  ;; 4 cycles
        ; ------------------------------------- Block: 14 cycles
//  876       {
//  877         address= BILL_START_ADD + (bill_count - 1) * BILL_JUMP_PG;
        MOV       X, N:_bill_count   ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0xA0          ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #0x1760        ;; 1 cycle
        MOVW      [SP+0x18], AX      ;; 1 cycle
//  878         eprom_read(address,0,PAGE_5,AUTO_CALC);      //page1-5 read
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x4            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, [SP+0x18]      ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
        ; ------------------------------------- Block: 14 cycles
//  879       }
//  880       TempTime = char_array_into_time4(&opr_data[0]);
??compartment3_142:
        MOVW      BC, #LWRD(_opr_data)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x1A          ;; 1 cycle
          CFI FunCall _char_array_into_time4
        CALL      _char_array_into_time4  ;; 3 cycles
        MOVW      HL, #LWRD(_TempTime)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x1A          ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOV       A, [DE]            ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
        MOV       A, [DE+0x01]       ;; 1 cycle
        MOV       [HL+0x01], A       ;; 1 cycle
        MOV       A, [DE+0x02]       ;; 1 cycle
        MOV       [HL+0x02], A       ;; 1 cycle
        MOV       A, [DE+0x03]       ;; 1 cycle
        MOV       [HL+0x03], A       ;; 1 cycle
        MOV       A, [DE+0x04]       ;; 1 cycle
        MOV       [HL+0x04], A       ;; 1 cycle
        MOV       A, [DE+0x05]       ;; 1 cycle
        MOV       [HL+0x05], A       ;; 1 cycle
        MOV       A, [DE+0x06]       ;; 1 cycle
        MOV       [HL+0x06], A       ;; 1 cycle
//  881       date_time(TempTime.day,TempTime.month,TempTime.year,TempTime.hour,TempTime.min,TempTime.sec,0); //Billing Date
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+48
        MOV       A, N:_TempTime     ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, N:_TempTime+1   ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       B, N:_TempTime+2   ;; 1 cycle
        MOV       C, N:_TempTime+6   ;; 1 cycle
        MOV       X, N:_TempTime+5   ;; 1 cycle
        MOV       A, N:_TempTime+3   ;; 1 cycle
          CFI FunCall _date_time
        CALL      _date_time         ;; 3 cycles
//  882       
//  883       break;
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+46
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 41 cycles
//  884       
//  885     case 0x011f0700:
//  886       long_int= curr.Rph.rms_signed;
??get_resp_45:
        MOVW      BC, N:_curr+14     ;; 1 cycle
        MOVW      AX, N:_curr+12     ;; 1 cycle
        MOVW      S:_long_int, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_long_int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  887       long_into_char_array4(long_int,char_array);
        MOVW      DE, #LWRD(_char_array)  ;; 1 cycle
        MOVW      BC, S:_long_int+2  ;; 1 cycle
        MOVW      AX, S:_long_int    ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  888       val_signed_4byt2(char_array[0],char_array[1],char_array[2],char_array[3]);
        MOV       B, N:_char_array+3  ;; 1 cycle
        MOV       C, N:_char_array+2  ;; 1 cycle
        MOV       X, N:_char_array+1  ;; 1 cycle
        MOV       A, N:_char_array   ;; 1 cycle
          CFI FunCall _val_signed_4byt2
        CALL      _val_signed_4byt2  ;; 3 cycles
//  889       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 22 cycles
//  890       
//  891     case 0x01330700:
//  892       long_int= curr.Yph.rms_signed;
??get_resp_46:
        MOVW      BC, N:_curr+30     ;; 1 cycle
        MOVW      AX, N:_curr+28     ;; 1 cycle
        MOVW      S:_long_int, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_long_int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  893       long_into_char_array4(long_int,char_array);
        MOVW      DE, #LWRD(_char_array)  ;; 1 cycle
        MOVW      BC, S:_long_int+2  ;; 1 cycle
        MOVW      AX, S:_long_int    ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  894       val_signed_4byt2(char_array[0],char_array[1],char_array[2],char_array[3]);
        MOV       B, N:_char_array+3  ;; 1 cycle
        MOV       C, N:_char_array+2  ;; 1 cycle
        MOV       X, N:_char_array+1  ;; 1 cycle
        MOV       A, N:_char_array   ;; 1 cycle
          CFI FunCall _val_signed_4byt2
        CALL      _val_signed_4byt2  ;; 3 cycles
//  895       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 22 cycles
//  896       
//  897     case 0x01470700:
//  898       long_int= curr.Bph.rms_signed;
??get_resp_47:
        MOVW      BC, N:_curr+46     ;; 1 cycle
        MOVW      AX, N:_curr+44     ;; 1 cycle
        MOVW      S:_long_int, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_long_int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  899       long_into_char_array4(long_int,char_array);
        MOVW      DE, #LWRD(_char_array)  ;; 1 cycle
        MOVW      BC, S:_long_int+2  ;; 1 cycle
        MOVW      AX, S:_long_int    ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  900       val_signed_4byt2(char_array[0],char_array[1],char_array[2],char_array[3]);
        MOV       B, N:_char_array+3  ;; 1 cycle
        MOV       C, N:_char_array+2  ;; 1 cycle
        MOV       X, N:_char_array+1  ;; 1 cycle
        MOV       A, N:_char_array   ;; 1 cycle
          CFI FunCall _val_signed_4byt2
        CALL      _val_signed_4byt2  ;; 3 cycles
//  901       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 22 cycles
//  902       
//  903     case 0x015b0700:
//  904       long_int= curr.Nph.rms;
??get_resp_48:
        MOVW      BC, N:_curr+50     ;; 1 cycle
        MOVW      AX, N:_curr+48     ;; 1 cycle
        MOVW      S:_long_int, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_long_int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  905       long_into_char_array4(long_int,char_array);
        MOVW      DE, #LWRD(_char_array)  ;; 1 cycle
        MOVW      BC, S:_long_int+2  ;; 1 cycle
        MOVW      AX, S:_long_int    ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
//  906       val_signed_4byt2(char_array[0],char_array[1],char_array[2],char_array[3]);
        MOV       B, N:_char_array+3  ;; 1 cycle
        MOV       C, N:_char_array+2  ;; 1 cycle
        MOV       X, N:_char_array+1  ;; 1 cycle
        MOV       A, N:_char_array   ;; 1 cycle
          CFI FunCall _val_signed_4byt2
        CALL      _val_signed_4byt2  ;; 3 cycles
//  907       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 22 cycles
//  908       
//  909     case 0x01200700:
//  910       long_int= vol.Rph.rms;
??get_resp_49:
        MOVW      AX, N:_vol         ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        MOVW      S:_long_int, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_long_int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  911       int_into_char_array(long_int,char_array);
        MOVW      BC, #LWRD(_char_array)  ;; 1 cycle
        MOVW      HL, S:_long_int+2  ;; 1 cycle
        MOVW      DE, S:_long_int    ;; 1 cycle
        MOVW      AX, DE             ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  912       val_2byt2(char_array[0], char_array[1]);
        MOV       X, N:_char_array+1  ;; 1 cycle
        MOV       A, N:_char_array   ;; 1 cycle
          CFI FunCall _val_2byt2
        CALL      _val_2byt2         ;; 3 cycles
//  913       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 21 cycles
//  914       
//  915     case 0x01340700:
//  916       long_int= vol.Yph.rms;
??get_resp_50:
        MOVW      AX, N:_vol+6       ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        MOVW      S:_long_int, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_long_int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  917       int_into_char_array(long_int,char_array);
        MOVW      BC, #LWRD(_char_array)  ;; 1 cycle
        MOVW      HL, S:_long_int+2  ;; 1 cycle
        MOVW      DE, S:_long_int    ;; 1 cycle
        MOVW      AX, DE             ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  918       val_2byt2(char_array[0], char_array[1]);
        MOV       X, N:_char_array+1  ;; 1 cycle
        MOV       A, N:_char_array   ;; 1 cycle
          CFI FunCall _val_2byt2
        CALL      _val_2byt2         ;; 3 cycles
//  919       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 21 cycles
//  920       
//  921     case 0x01480700:
//  922       long_int= vol.Bph.rms;
??get_resp_51:
        MOVW      AX, N:_vol+12      ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        MOVW      S:_long_int, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_long_int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  923       int_into_char_array(long_int,char_array);
        MOVW      BC, #LWRD(_char_array)  ;; 1 cycle
        MOVW      HL, S:_long_int+2  ;; 1 cycle
        MOVW      DE, S:_long_int    ;; 1 cycle
        MOVW      AX, DE             ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  924       val_2byt2(char_array[0], char_array[1]);
        MOV       X, N:_char_array+1  ;; 1 cycle
        MOV       A, N:_char_array   ;; 1 cycle
          CFI FunCall _val_2byt2
        CALL      _val_2byt2         ;; 3 cycles
//  925       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 21 cycles
//  926       
//  927     case 0x017C0700:
//  928       long_int= ph_ph.Ph_RY.vol.value;
??get_resp_52:
        MOVW      AX, N:_ph_ph       ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        MOVW      S:_long_int, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_long_int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  929       int_into_char_array(long_int,char_array);
        MOVW      BC, #LWRD(_char_array)  ;; 1 cycle
        MOVW      HL, S:_long_int+2  ;; 1 cycle
        MOVW      DE, S:_long_int    ;; 1 cycle
        MOVW      AX, DE             ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  930       val_2byt2(char_array[0], char_array[1]);
        MOV       X, N:_char_array+1  ;; 1 cycle
        MOV       A, N:_char_array   ;; 1 cycle
          CFI FunCall _val_2byt2
        CALL      _val_2byt2         ;; 3 cycles
//  931       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 21 cycles
//  932       
//  933     case 0x017D0700:
//  934       long_int= ph_ph.Ph_YB.vol.value;
??get_resp_53:
        MOVW      AX, N:_ph_ph+8     ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        MOVW      S:_long_int, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_long_int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  935       int_into_char_array(long_int,char_array);
        MOVW      BC, #LWRD(_char_array)  ;; 1 cycle
        MOVW      HL, S:_long_int+2  ;; 1 cycle
        MOVW      DE, S:_long_int    ;; 1 cycle
        MOVW      AX, DE             ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  936       val_2byt2(char_array[0], char_array[1]);
        MOV       X, N:_char_array+1  ;; 1 cycle
        MOV       A, N:_char_array   ;; 1 cycle
          CFI FunCall _val_2byt2
        CALL      _val_2byt2         ;; 3 cycles
//  937       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 21 cycles
//  938       
//  939     case 0x017E0700:
//  940       long_int= ph_ph.Ph_BR.vol.value;
??get_resp_54:
        MOVW      AX, N:_ph_ph+16    ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        MOVW      S:_long_int, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_long_int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  941       int_into_char_array(long_int,char_array);
        MOVW      BC, #LWRD(_char_array)  ;; 1 cycle
        MOVW      HL, S:_long_int+2  ;; 1 cycle
        MOVW      DE, S:_long_int    ;; 1 cycle
        MOVW      AX, DE             ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
//  942       val_2byt2(char_array[0], char_array[1]);
        MOV       X, N:_char_array+1  ;; 1 cycle
        MOV       A, N:_char_array   ;; 1 cycle
          CFI FunCall _val_2byt2
        CALL      _val_2byt2         ;; 3 cycles
//  943       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 21 cycles
//  944       
//  945     case 0x010D0700:
//  946       info[k++]= 0x00;
??get_resp_55:
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
//  947       if(METERING_MODE == NET)
        CMP       N:_METERING_MODE, #0x1  ;; 1 cycle
        BNZ       ??compartment3_143  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
//  948       {
//  949         if(quadrant.Allph == Q2 || quadrant.Allph == Q3)
        CMP       N:_quadrant+3, #0x2  ;; 1 cycle
        BZ        ??compartment3_144  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        CMP       N:_quadrant+3, #0x3  ;; 1 cycle
        BNZ       ??compartment3_145  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  950         {
//  951           signed_integer(-pf.Net_signed);
??compartment3_144:
        MOVW      AX, N:_pf+14       ;; 1 cycle
        XOR       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        XOR       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        INCW      AX                 ;; 1 cycle
          CFI FunCall _signed_integer
        CALL      _signed_integer    ;; 3 cycles
        BR        S:??compartment3_146  ;; 3 cycles
        ; ------------------------------------- Block: 12 cycles
//  952         }
//  953         else
//  954         {
//  955           signed_integer(pf.Net_signed);
??compartment3_145:
        MOVW      AX, N:_pf+14       ;; 1 cycle
          CFI FunCall _signed_integer
        CALL      _signed_integer    ;; 3 cycles
        BR        S:??compartment3_146  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
//  956         }
//  957       }
//  958       else
//  959       {
//  960         if(quadrant.Allph == Q2 || quadrant.Allph == Q4)
??compartment3_143:
        CMP       N:_quadrant+3, #0x2  ;; 1 cycle
        BZ        ??compartment3_147  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        CMP       N:_quadrant+3, #0x4  ;; 1 cycle
        BNZ       ??compartment3_148  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  961         {
//  962           signed_integer(-pf.Net_signed);
??compartment3_147:
        MOVW      AX, N:_pf+14       ;; 1 cycle
        XOR       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        XOR       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        INCW      AX                 ;; 1 cycle
          CFI FunCall _signed_integer
        CALL      _signed_integer    ;; 3 cycles
        BR        S:??compartment3_146  ;; 3 cycles
        ; ------------------------------------- Block: 12 cycles
//  963         }
//  964         else
//  965         {
//  966           signed_integer(pf.Net_signed);
??compartment3_148:
        MOVW      AX, N:_pf+14       ;; 1 cycle
          CFI FunCall _signed_integer
        CALL      _signed_integer    ;; 3 cycles
        ; ------------------------------------- Block: 4 cycles
//  967         }
//  968       }
//  969       break;
??compartment3_146:
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
//  970       
//  971     case 0x01210700:
//  972       info[k++]= 0x00;
??get_resp_56:
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
//  973       signed_integer(pf.Rph_signed);
        MOVW      AX, N:_pf+8        ;; 1 cycle
          CFI FunCall _signed_integer
        CALL      _signed_integer    ;; 3 cycles
//  974       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 12 cycles
//  975       
//  976     case 0x01350700:
//  977       info[k++]= 0x00;
??get_resp_57:
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
//  978       signed_integer(pf.Yph_signed);
        MOVW      AX, N:_pf+10       ;; 1 cycle
          CFI FunCall _signed_integer
        CALL      _signed_integer    ;; 3 cycles
//  979       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 12 cycles
//  980       
//  981     case 0x01490700:
//  982       info[k++]= 0x00;
??get_resp_58:
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
//  983       signed_integer(pf.Bph_signed);
        MOVW      AX, N:_pf+12       ;; 1 cycle
          CFI FunCall _signed_integer
        CALL      _signed_integer    ;; 3 cycles
//  984       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 12 cycles
//  985       
//  986     case 0x010E0700:
//  987       val_2byt2(freq.Net / 256, freq.Net % 256);
??get_resp_59:
        MOVW      AX, N:_freq+6      ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      AX, N:_freq+6      ;; 1 cycle
        CLRB      X                  ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
          CFI FunCall _val_2byt2
        CALL      _val_2byt2         ;; 3 cycles
//  988       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 17 cycles
//  989       
//  990     case 0x01220700:
//  991       val_2byt2(freq.Rph / 256, freq.Rph % 256);
??get_resp_60:
        MOVW      AX, N:_freq        ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      AX, N:_freq        ;; 1 cycle
        CLRB      X                  ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
          CFI FunCall _val_2byt2
        CALL      _val_2byt2         ;; 3 cycles
//  992       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 17 cycles
//  993       
//  994     case 0x01360700:
//  995       val_2byt2(freq.Yph / 256, freq.Yph % 256);
??get_resp_61:
        MOVW      AX, N:_freq+2      ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      AX, N:_freq+2      ;; 1 cycle
        CLRB      X                  ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
          CFI FunCall _val_2byt2
        CALL      _val_2byt2         ;; 3 cycles
//  996       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 17 cycles
//  997       
//  998     case 0x014A0700:
//  999       val_2byt2(freq.Bph / 256, freq.Bph % 256);
??get_resp_62:
        MOVW      AX, N:_freq+4      ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      AX, N:_freq+4      ;; 1 cycle
        CLRB      X                  ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
          CFI FunCall _val_2byt2
        CALL      _val_2byt2         ;; 3 cycles
// 1000       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 17 cycles
// 1001       
// 1002     case 0x01010700:
// 1003       long_int= power.Allph.active_signed;                 
??get_resp_63:
        MOVW      BC, N:_power+82    ;; 1 cycle
        MOVW      AX, N:_power+80    ;; 1 cycle
        MOVW      S:_long_int, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_long_int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1004       long_into_char_array4(long_int,char_array);
        MOVW      DE, #LWRD(_char_array)  ;; 1 cycle
        MOVW      BC, S:_long_int+2  ;; 1 cycle
        MOVW      AX, S:_long_int    ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 1005       val_signed_4byt2(char_array[0],char_array[1],char_array[2],char_array[3]);
        MOV       B, N:_char_array+3  ;; 1 cycle
        MOV       C, N:_char_array+2  ;; 1 cycle
        MOV       X, N:_char_array+1  ;; 1 cycle
        MOV       A, N:_char_array   ;; 1 cycle
          CFI FunCall _val_signed_4byt2
        CALL      _val_signed_4byt2  ;; 3 cycles
// 1006       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 22 cycles
// 1007       
// 1008     case 0x01150700:
// 1009       long_int= power.Rph.active_signed;                 
??get_resp_64:
        MOVW      BC, N:_power+14    ;; 1 cycle
        MOVW      AX, N:_power+12    ;; 1 cycle
        MOVW      S:_long_int, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_long_int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1010       long_into_char_array4(long_int,char_array);
        MOVW      DE, #LWRD(_char_array)  ;; 1 cycle
        MOVW      BC, S:_long_int+2  ;; 1 cycle
        MOVW      AX, S:_long_int    ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 1011       val_signed_4byt2(char_array[0],char_array[1],char_array[2],char_array[3]);
        MOV       B, N:_char_array+3  ;; 1 cycle
        MOV       C, N:_char_array+2  ;; 1 cycle
        MOV       X, N:_char_array+1  ;; 1 cycle
        MOV       A, N:_char_array   ;; 1 cycle
          CFI FunCall _val_signed_4byt2
        CALL      _val_signed_4byt2  ;; 3 cycles
// 1012       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 22 cycles
// 1013       
// 1014     case 0x01290700:
// 1015       long_int= power.Yph.active_signed;                 
??get_resp_65:
        MOVW      BC, N:_power+34    ;; 1 cycle
        MOVW      AX, N:_power+32    ;; 1 cycle
        MOVW      S:_long_int, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_long_int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1016       long_into_char_array4(long_int,char_array);
        MOVW      DE, #LWRD(_char_array)  ;; 1 cycle
        MOVW      BC, S:_long_int+2  ;; 1 cycle
        MOVW      AX, S:_long_int    ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 1017       val_signed_4byt2(char_array[0],char_array[1],char_array[2],char_array[3]);
        MOV       B, N:_char_array+3  ;; 1 cycle
        MOV       C, N:_char_array+2  ;; 1 cycle
        MOV       X, N:_char_array+1  ;; 1 cycle
        MOV       A, N:_char_array   ;; 1 cycle
          CFI FunCall _val_signed_4byt2
        CALL      _val_signed_4byt2  ;; 3 cycles
// 1018       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 22 cycles
// 1019       
// 1020     case 0x013D0700:
// 1021       long_int= power.Bph.active_signed;                 
??get_resp_66:
        MOVW      BC, N:_power+54    ;; 1 cycle
        MOVW      AX, N:_power+52    ;; 1 cycle
        MOVW      S:_long_int, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_long_int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1022       long_into_char_array4(long_int,char_array);
        MOVW      DE, #LWRD(_char_array)  ;; 1 cycle
        MOVW      BC, S:_long_int+2  ;; 1 cycle
        MOVW      AX, S:_long_int    ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 1023       val_signed_4byt2(char_array[0],char_array[1],char_array[2],char_array[3]);
        MOV       B, N:_char_array+3  ;; 1 cycle
        MOV       C, N:_char_array+2  ;; 1 cycle
        MOV       X, N:_char_array+1  ;; 1 cycle
        MOV       A, N:_char_array   ;; 1 cycle
          CFI FunCall _val_signed_4byt2
        CALL      _val_signed_4byt2  ;; 3 cycles
// 1024       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 22 cycles
// 1025       
// 1026     case 0x01030700:
// 1027       long_int= power.Allph.reactive_signed;
??get_resp_67:
        MOVW      BC, N:_power+86    ;; 1 cycle
        MOVW      AX, N:_power+84    ;; 1 cycle
        MOVW      S:_long_int, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_long_int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1028       long_into_char_array4(long_int,char_array);
        MOVW      DE, #LWRD(_char_array)  ;; 1 cycle
        MOVW      BC, S:_long_int+2  ;; 1 cycle
        MOVW      AX, S:_long_int    ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 1029       val_signed_4byt2(char_array[0],char_array[1],char_array[2],char_array[3]);
        MOV       B, N:_char_array+3  ;; 1 cycle
        MOV       C, N:_char_array+2  ;; 1 cycle
        MOV       X, N:_char_array+1  ;; 1 cycle
        MOV       A, N:_char_array   ;; 1 cycle
          CFI FunCall _val_signed_4byt2
        CALL      _val_signed_4byt2  ;; 3 cycles
// 1030       break;    
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 22 cycles
// 1031       
// 1032     case 0x01170700:
// 1033       long_int= power.Rph.reactive_signed;
??get_resp_68:
        MOVW      BC, N:_power+18    ;; 1 cycle
        MOVW      AX, N:_power+16    ;; 1 cycle
        MOVW      S:_long_int, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_long_int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1034       long_into_char_array4(long_int,char_array);
        MOVW      DE, #LWRD(_char_array)  ;; 1 cycle
        MOVW      BC, S:_long_int+2  ;; 1 cycle
        MOVW      AX, S:_long_int    ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 1035       val_signed_4byt2(char_array[0],char_array[1],char_array[2],char_array[3]);
        MOV       B, N:_char_array+3  ;; 1 cycle
        MOV       C, N:_char_array+2  ;; 1 cycle
        MOV       X, N:_char_array+1  ;; 1 cycle
        MOV       A, N:_char_array   ;; 1 cycle
          CFI FunCall _val_signed_4byt2
        CALL      _val_signed_4byt2  ;; 3 cycles
// 1036       break;    
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 22 cycles
// 1037       
// 1038     case 0x012B0700:
// 1039       long_int= power.Yph.reactive_signed;
??get_resp_69:
        MOVW      BC, N:_power+38    ;; 1 cycle
        MOVW      AX, N:_power+36    ;; 1 cycle
        MOVW      S:_long_int, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_long_int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1040       long_into_char_array4(long_int,char_array);
        MOVW      DE, #LWRD(_char_array)  ;; 1 cycle
        MOVW      BC, S:_long_int+2  ;; 1 cycle
        MOVW      AX, S:_long_int    ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 1041       val_signed_4byt2(char_array[0],char_array[1],char_array[2],char_array[3]);
        MOV       B, N:_char_array+3  ;; 1 cycle
        MOV       C, N:_char_array+2  ;; 1 cycle
        MOV       X, N:_char_array+1  ;; 1 cycle
        MOV       A, N:_char_array   ;; 1 cycle
          CFI FunCall _val_signed_4byt2
        CALL      _val_signed_4byt2  ;; 3 cycles
// 1042       break;    
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 22 cycles
// 1043       
// 1044     case 0x013F0700:
// 1045       long_int= power.Bph.reactive_signed;
??get_resp_70:
        MOVW      BC, N:_power+58    ;; 1 cycle
        MOVW      AX, N:_power+56    ;; 1 cycle
        MOVW      S:_long_int, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_long_int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1046       long_into_char_array4(long_int,char_array);
        MOVW      DE, #LWRD(_char_array)  ;; 1 cycle
        MOVW      BC, S:_long_int+2  ;; 1 cycle
        MOVW      AX, S:_long_int    ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 1047       val_signed_4byt2(char_array[0],char_array[1],char_array[2],char_array[3]);
        MOV       B, N:_char_array+3  ;; 1 cycle
        MOV       C, N:_char_array+2  ;; 1 cycle
        MOV       X, N:_char_array+1  ;; 1 cycle
        MOV       A, N:_char_array   ;; 1 cycle
          CFI FunCall _val_signed_4byt2
        CALL      _val_signed_4byt2  ;; 3 cycles
// 1048       break;    
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 22 cycles
// 1049       
// 1050     case 0x01090700:
// 1051       long_int= power.Allph.apparent_net;    
??get_resp_71:
        MOVW      BC, N:_power+74    ;; 1 cycle
        MOVW      AX, N:_power+72    ;; 1 cycle
        MOVW      S:_long_int, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_long_int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1052       long_into_char_array4(long_int,char_array);
        MOVW      DE, #LWRD(_char_array)  ;; 1 cycle
        MOVW      BC, S:_long_int+2  ;; 1 cycle
        MOVW      AX, S:_long_int    ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 1053       val_4byt2(char_array[0], char_array[1], char_array[2], char_array[3]);
        MOV       B, N:_char_array+3  ;; 1 cycle
        MOV       C, N:_char_array+2  ;; 1 cycle
        MOV       X, N:_char_array+1  ;; 1 cycle
        MOV       A, N:_char_array   ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
// 1054       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 22 cycles
// 1055       
// 1056     case 0x011D0700:
// 1057       long_int= power.Rph.apparent;    
??get_resp_72:
        MOVW      BC, N:_power+10    ;; 1 cycle
        MOVW      AX, N:_power+8     ;; 1 cycle
        MOVW      S:_long_int, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_long_int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1058       long_into_char_array4(long_int,char_array);
        MOVW      DE, #LWRD(_char_array)  ;; 1 cycle
        MOVW      BC, S:_long_int+2  ;; 1 cycle
        MOVW      AX, S:_long_int    ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 1059       val_4byt2(char_array[0], char_array[1], char_array[2], char_array[3]);
        MOV       B, N:_char_array+3  ;; 1 cycle
        MOV       C, N:_char_array+2  ;; 1 cycle
        MOV       X, N:_char_array+1  ;; 1 cycle
        MOV       A, N:_char_array   ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
// 1060       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 22 cycles
// 1061       
// 1062     case 0x01310700:
// 1063       long_int= power.Yph.apparent;   
??get_resp_73:
        MOVW      BC, N:_power+30    ;; 1 cycle
        MOVW      AX, N:_power+28    ;; 1 cycle
        MOVW      S:_long_int, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_long_int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1064       long_into_char_array4(long_int,char_array);
        MOVW      DE, #LWRD(_char_array)  ;; 1 cycle
        MOVW      BC, S:_long_int+2  ;; 1 cycle
        MOVW      AX, S:_long_int    ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 1065       val_4byt2(char_array[0], char_array[1], char_array[2], char_array[3]);
        MOV       B, N:_char_array+3  ;; 1 cycle
        MOV       C, N:_char_array+2  ;; 1 cycle
        MOV       X, N:_char_array+1  ;; 1 cycle
        MOV       A, N:_char_array   ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
// 1066       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 22 cycles
// 1067       
// 1068     case 0x01450700:
// 1069       long_int= power.Bph.apparent;    
??get_resp_74:
        MOVW      BC, N:_power+50    ;; 1 cycle
        MOVW      AX, N:_power+48    ;; 1 cycle
        MOVW      S:_long_int, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_long_int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1070       long_into_char_array4(long_int,char_array);
        MOVW      DE, #LWRD(_char_array)  ;; 1 cycle
        MOVW      BC, S:_long_int+2  ;; 1 cycle
        MOVW      AX, S:_long_int    ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 1071       val_4byt2(char_array[0], char_array[1], char_array[2], char_array[3]);
        MOV       B, N:_char_array+3  ;; 1 cycle
        MOV       C, N:_char_array+2  ;; 1 cycle
        MOV       X, N:_char_array+1  ;; 1 cycle
        MOV       A, N:_char_array   ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
// 1072       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 22 cycles
// 1073       
// 1074     case 0x005e5b08:
// 1075       long_into_char_array4(power_off_min , char_array);
??get_resp_75:
        MOVW      DE, #LWRD(_char_array)  ;; 1 cycle
        MOVW      BC, N:_power_off_min+2  ;; 1 cycle
        MOVW      AX, N:_power_off_min  ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 1076       val_4byt2(char_array[0],char_array[1],char_array[2],char_array[3]);
        MOV       B, N:_char_array+3  ;; 1 cycle
        MOV       C, N:_char_array+2  ;; 1 cycle
        MOV       X, N:_char_array+1  ;; 1 cycle
        MOV       A, N:_char_array   ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
// 1077       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 16 cycles
// 1078       
// 1079     case 0x005e5b0d:
// 1080       long_into_char_array4(power_on_min , char_array);
??get_resp_76:
        MOVW      DE, #LWRD(_char_array)  ;; 1 cycle
        MOVW      BC, N:_power_on_min+2  ;; 1 cycle
        MOVW      AX, N:_power_on_min  ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 1081       val_4byt2(char_array[0],char_array[1],char_array[2],char_array[3]);
        MOV       B, N:_char_array+3  ;; 1 cycle
        MOV       C, N:_char_array+2  ;; 1 cycle
        MOV       X, N:_char_array+1  ;; 1 cycle
        MOV       A, N:_char_array   ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
// 1082       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 16 cycles
// 1083       
// 1084     case 0x01010800:
// 1085       long_int= energy.Allph.active_imp;
??get_resp_77:
        MOVW      BC, N:_energy+54   ;; 1 cycle
        MOVW      AX, N:_energy+52   ;; 1 cycle
        MOVW      S:_long_int, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_long_int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1086       long_into_char_array4(long_int,char_array);
        MOVW      DE, #LWRD(_char_array)  ;; 1 cycle
        MOVW      BC, S:_long_int+2  ;; 1 cycle
        MOVW      AX, S:_long_int    ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 1087       val_4byt2(char_array[0], char_array[1], char_array[2], char_array[3]); 
        MOV       B, N:_char_array+3  ;; 1 cycle
        MOV       C, N:_char_array+2  ;; 1 cycle
        MOV       X, N:_char_array+1  ;; 1 cycle
        MOV       A, N:_char_array   ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
// 1088       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 22 cycles
// 1089       
// 1090     case 0x01020800:
// 1091       long_int= energy.Allph.active_exp;
??get_resp_78:
        MOVW      BC, N:_energy+58   ;; 1 cycle
        MOVW      AX, N:_energy+56   ;; 1 cycle
        MOVW      S:_long_int, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_long_int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1092       long_into_char_array4(long_int,char_array);
        MOVW      DE, #LWRD(_char_array)  ;; 1 cycle
        MOVW      BC, S:_long_int+2  ;; 1 cycle
        MOVW      AX, S:_long_int    ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 1093       val_4byt2(char_array[0], char_array[1], char_array[2], char_array[3]); 
        MOV       B, N:_char_array+3  ;; 1 cycle
        MOV       C, N:_char_array+2  ;; 1 cycle
        MOV       X, N:_char_array+1  ;; 1 cycle
        MOV       A, N:_char_array   ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
// 1094       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 22 cycles
// 1095       
// 1096     case 0x01050800:
// 1097       long_int= energy.Allph.reactive_q1; 
??get_resp_79:
        MOVW      BC, N:_energy+78   ;; 1 cycle
        MOVW      AX, N:_energy+76   ;; 1 cycle
        MOVW      S:_long_int, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_long_int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1098       long_into_char_array4(long_int,char_array);
        MOVW      DE, #LWRD(_char_array)  ;; 1 cycle
        MOVW      BC, S:_long_int+2  ;; 1 cycle
        MOVW      AX, S:_long_int    ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 1099       val_4byt2(char_array[0], char_array[1], char_array[2], char_array[3]); 
        MOV       B, N:_char_array+3  ;; 1 cycle
        MOV       C, N:_char_array+2  ;; 1 cycle
        MOV       X, N:_char_array+1  ;; 1 cycle
        MOV       A, N:_char_array   ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
// 1100       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 22 cycles
// 1101       
// 1102     case 0x01060800:
// 1103       long_int= energy.Allph.reactive_q2; 
??get_resp_80:
        MOVW      BC, N:_energy+82   ;; 1 cycle
        MOVW      AX, N:_energy+80   ;; 1 cycle
        MOVW      S:_long_int, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_long_int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1104       long_into_char_array4(long_int,char_array);
        MOVW      DE, #LWRD(_char_array)  ;; 1 cycle
        MOVW      BC, S:_long_int+2  ;; 1 cycle
        MOVW      AX, S:_long_int    ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 1105       val_4byt2(char_array[0], char_array[1], char_array[2], char_array[3]); 
        MOV       B, N:_char_array+3  ;; 1 cycle
        MOV       C, N:_char_array+2  ;; 1 cycle
        MOV       X, N:_char_array+1  ;; 1 cycle
        MOV       A, N:_char_array   ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
// 1106       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 22 cycles
// 1107       
// 1108     case 0x01070800:
// 1109       long_int= energy.Allph.reactive_q3; 
??get_resp_81:
        MOVW      BC, N:_energy+86   ;; 1 cycle
        MOVW      AX, N:_energy+84   ;; 1 cycle
        MOVW      S:_long_int, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_long_int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1110       long_into_char_array4(long_int,char_array);
        MOVW      DE, #LWRD(_char_array)  ;; 1 cycle
        MOVW      BC, S:_long_int+2  ;; 1 cycle
        MOVW      AX, S:_long_int    ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 1111       val_4byt2(char_array[0], char_array[1], char_array[2], char_array[3]); 
        MOV       B, N:_char_array+3  ;; 1 cycle
        MOV       C, N:_char_array+2  ;; 1 cycle
        MOV       X, N:_char_array+1  ;; 1 cycle
        MOV       A, N:_char_array   ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
// 1112       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 22 cycles
// 1113       
// 1114     case 0x01080800:
// 1115       long_int= energy.Allph.reactive_q4; 
??get_resp_82:
        MOVW      BC, N:_energy+90   ;; 1 cycle
        MOVW      AX, N:_energy+88   ;; 1 cycle
        MOVW      S:_long_int, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_long_int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1116       long_into_char_array4(long_int,char_array);
        MOVW      DE, #LWRD(_char_array)  ;; 1 cycle
        MOVW      BC, S:_long_int+2  ;; 1 cycle
        MOVW      AX, S:_long_int    ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 1117       val_4byt2(char_array[0], char_array[1], char_array[2], char_array[3]); 
        MOV       B, N:_char_array+3  ;; 1 cycle
        MOV       C, N:_char_array+2  ;; 1 cycle
        MOV       X, N:_char_array+1  ;; 1 cycle
        MOV       A, N:_char_array   ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
// 1118       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 22 cycles
// 1119       
// 1120     case 0x01090800:
// 1121       long_int= energy.Allph.apparent_imp;
??get_resp_83:
        MOVW      BC, N:_energy+70   ;; 1 cycle
        MOVW      AX, N:_energy+68   ;; 1 cycle
        MOVW      S:_long_int, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_long_int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1122       long_into_char_array4(long_int,char_array);
        MOVW      DE, #LWRD(_char_array)  ;; 1 cycle
        MOVW      BC, S:_long_int+2  ;; 1 cycle
        MOVW      AX, S:_long_int    ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 1123       val_4byt2(char_array[0], char_array[1], char_array[2], char_array[3]); 
        MOV       B, N:_char_array+3  ;; 1 cycle
        MOV       C, N:_char_array+2  ;; 1 cycle
        MOV       X, N:_char_array+1  ;; 1 cycle
        MOV       A, N:_char_array   ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
// 1124       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 22 cycles
// 1125       
// 1126     case 0x010A0800:
// 1127       long_int= energy.Allph.apparent_exp;
??get_resp_84:
        MOVW      BC, N:_energy+74   ;; 1 cycle
        MOVW      AX, N:_energy+72   ;; 1 cycle
        MOVW      S:_long_int, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_long_int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1128       long_into_char_array4(long_int,char_array);
        MOVW      DE, #LWRD(_char_array)  ;; 1 cycle
        MOVW      BC, S:_long_int+2  ;; 1 cycle
        MOVW      AX, S:_long_int    ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 1129       val_4byt2(char_array[0], char_array[1], char_array[2], char_array[3]); 
        MOV       B, N:_char_array+3  ;; 1 cycle
        MOV       C, N:_char_array+2  ;; 1 cycle
        MOV       X, N:_char_array+1  ;; 1 cycle
        MOV       A, N:_char_array   ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
// 1130       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 22 cycles
// 1131       
// 1132     case 0x01010200:
// 1133       long_int= cum_max_demand_kw;
??get_resp_85:
        MOVW      BC, N:_cum_max_demand_kw+2  ;; 1 cycle
        MOVW      AX, N:_cum_max_demand_kw  ;; 1 cycle
        MOVW      S:_long_int, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_long_int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1134       long_into_char_array4(long_int,char_array);
        MOVW      DE, #LWRD(_char_array)  ;; 1 cycle
        MOVW      BC, S:_long_int+2  ;; 1 cycle
        MOVW      AX, S:_long_int    ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 1135       val_4byt2(char_array[0], char_array[1], char_array[2], char_array[3]);
        MOV       B, N:_char_array+3  ;; 1 cycle
        MOV       C, N:_char_array+2  ;; 1 cycle
        MOV       X, N:_char_array+1  ;; 1 cycle
        MOV       A, N:_char_array   ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
// 1136       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 22 cycles
// 1137       
// 1138     case 0x01090200:
// 1139       long_int= cum_max_demand_kva;
??get_resp_86:
        MOVW      BC, N:_cum_max_demand_kva+2  ;; 1 cycle
        MOVW      AX, N:_cum_max_demand_kva  ;; 1 cycle
        MOVW      S:_long_int, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_long_int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1140       long_into_char_array4(long_int,char_array);
        MOVW      DE, #LWRD(_char_array)  ;; 1 cycle
        MOVW      BC, S:_long_int+2  ;; 1 cycle
        MOVW      AX, S:_long_int    ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 1141       val_4byt2(char_array[0], char_array[1], char_array[2], char_array[3]); 
        MOV       B, N:_char_array+3  ;; 1 cycle
        MOV       C, N:_char_array+2  ;; 1 cycle
        MOV       X, N:_char_array+1  ;; 1 cycle
        MOV       A, N:_char_array   ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
// 1142       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 22 cycles
// 1143       
// 1144     case 0x01800800:
// 1145       long_int= energy.Allph.fundamental;
??get_resp_87:
        MOVW      BC, N:_energy+66   ;; 1 cycle
        MOVW      AX, N:_energy+64   ;; 1 cycle
        MOVW      S:_long_int, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_long_int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1146       long_into_char_array4(long_int,char_array);
        MOVW      DE, #LWRD(_char_array)  ;; 1 cycle
        MOVW      BC, S:_long_int+2  ;; 1 cycle
        MOVW      AX, S:_long_int    ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 1147       val_4byt2(char_array[0], char_array[1], char_array[2], char_array[3]); 
        MOV       B, N:_char_array+3  ;; 1 cycle
        MOV       C, N:_char_array+2  ;; 1 cycle
        MOV       X, N:_char_array+1  ;; 1 cycle
        MOV       A, N:_char_array   ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
// 1148       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 22 cycles
// 1149       /*        case 0x01960800:
// 1150       long_int= defraud_kwh;
// 1151       long_to_char_array();
// 1152       val_4byt2(char_array[0], char_array[1], char_array[2], char_array[3]); // kwh 
// 1153       break;
// 1154       */
// 1155       
// 1156     case 0x018E0800:
// 1157       long_int= get_hr_energy(energy.Allph.active_imp,energy.Allph.active_imp_pulse);
??get_resp_88:
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x1A          ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        PUSH      DE                 ;; 1 cycle
          CFI CFA SP+48
        MOV       A, N:_energy+42    ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+50
        XCH       A, X               ;; 1 cycle
        MOVW      BC, N:_energy+54   ;; 1 cycle
        MOVW      AX, N:_energy+52   ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+52
        POP       DE                 ;; 1 cycle
          CFI CFA SP+50
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x26          ;; 1 cycle
          CFI FunCall _get_hr_energy
        CALL      _get_hr_energy     ;; 3 cycles
        MOVW      HL, AX             ;; 1 cycle
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+48
        POP       DE                 ;; 1 cycle
          CFI CFA SP+46
        MOVW      AX, [HL]           ;; 1 cycle
        MOVW      [DE], AX           ;; 1 cycle
        MOVW      AX, [HL+0x02]      ;; 1 cycle
        MOVW      [DE+0x02], AX      ;; 1 cycle
        MOVW      AX, [HL+0x04]      ;; 1 cycle
        MOVW      [DE+0x04], AX      ;; 1 cycle
        MOVW      AX, [HL+0x06]      ;; 1 cycle
        MOVW      [DE+0x06], AX      ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x1A          ;; 1 cycle
          CFI FunCall __LLU2L
        CALL      __LLU2L            ;; 3 cycles
        MOVW      S:_long_int, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_long_int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1158       long_into_char_array4(long_int,char_array);
        MOVW      DE, #LWRD(_char_array)  ;; 1 cycle
        MOVW      BC, S:_long_int+2  ;; 1 cycle
        MOVW      AX, S:_long_int    ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 1159       val_4byt2(char_array[0], char_array[1], char_array[2], char_array[3]);
        MOV       B, N:_char_array+3  ;; 1 cycle
        MOV       C, N:_char_array+2  ;; 1 cycle
        MOV       X, N:_char_array+1  ;; 1 cycle
        MOV       A, N:_char_array   ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
// 1160       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 54 cycles
// 1161       
// 1162     case 0x018F0800:
// 1163       long_int= get_hr_energy(energy.Allph.reactive_q1,energy.Allph.reactive_q1_pulse);
??get_resp_89:
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x1A          ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        PUSH      DE                 ;; 1 cycle
          CFI CFA SP+48
        MOV       A, N:_energy+47    ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+50
        XCH       A, X               ;; 1 cycle
        MOVW      BC, N:_energy+78   ;; 1 cycle
        MOVW      AX, N:_energy+76   ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+52
        POP       DE                 ;; 1 cycle
          CFI CFA SP+50
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x26          ;; 1 cycle
          CFI FunCall _get_hr_energy
        CALL      _get_hr_energy     ;; 3 cycles
        MOVW      HL, AX             ;; 1 cycle
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+48
        POP       DE                 ;; 1 cycle
          CFI CFA SP+46
        MOVW      AX, [HL]           ;; 1 cycle
        MOVW      [DE], AX           ;; 1 cycle
        MOVW      AX, [HL+0x02]      ;; 1 cycle
        MOVW      [DE+0x02], AX      ;; 1 cycle
        MOVW      AX, [HL+0x04]      ;; 1 cycle
        MOVW      [DE+0x04], AX      ;; 1 cycle
        MOVW      AX, [HL+0x06]      ;; 1 cycle
        MOVW      [DE+0x06], AX      ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x1A          ;; 1 cycle
          CFI FunCall __LLU2L
        CALL      __LLU2L            ;; 3 cycles
        MOVW      S:_long_int, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_long_int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1164       long_into_char_array4(long_int,char_array);
        MOVW      DE, #LWRD(_char_array)  ;; 1 cycle
        MOVW      BC, S:_long_int+2  ;; 1 cycle
        MOVW      AX, S:_long_int    ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 1165       val_4byt2(char_array[0], char_array[1], char_array[2], char_array[3]); 
        MOV       B, N:_char_array+3  ;; 1 cycle
        MOV       C, N:_char_array+2  ;; 1 cycle
        MOV       X, N:_char_array+1  ;; 1 cycle
        MOV       A, N:_char_array   ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
// 1166       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 54 cycles
// 1167       
// 1168     case 0x01900800:
// 1169       long_int= get_hr_energy(energy.Allph.reactive_q4,energy.Allph.reactive_q4_pulse); 
??get_resp_90:
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x1A          ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        PUSH      DE                 ;; 1 cycle
          CFI CFA SP+48
        MOV       A, N:_energy+50    ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+50
        XCH       A, X               ;; 1 cycle
        MOVW      BC, N:_energy+90   ;; 1 cycle
        MOVW      AX, N:_energy+88   ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+52
        POP       DE                 ;; 1 cycle
          CFI CFA SP+50
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x26          ;; 1 cycle
          CFI FunCall _get_hr_energy
        CALL      _get_hr_energy     ;; 3 cycles
        MOVW      HL, AX             ;; 1 cycle
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+48
        POP       DE                 ;; 1 cycle
          CFI CFA SP+46
        MOVW      AX, [HL]           ;; 1 cycle
        MOVW      [DE], AX           ;; 1 cycle
        MOVW      AX, [HL+0x02]      ;; 1 cycle
        MOVW      [DE+0x02], AX      ;; 1 cycle
        MOVW      AX, [HL+0x04]      ;; 1 cycle
        MOVW      [DE+0x04], AX      ;; 1 cycle
        MOVW      AX, [HL+0x06]      ;; 1 cycle
        MOVW      [DE+0x06], AX      ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x1A          ;; 1 cycle
          CFI FunCall __LLU2L
        CALL      __LLU2L            ;; 3 cycles
        MOVW      S:_long_int, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_long_int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1170       long_into_char_array4(long_int,char_array);
        MOVW      DE, #LWRD(_char_array)  ;; 1 cycle
        MOVW      BC, S:_long_int+2  ;; 1 cycle
        MOVW      AX, S:_long_int    ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 1171       val_4byt2(char_array[0], char_array[1], char_array[2], char_array[3]); 
        MOV       B, N:_char_array+3  ;; 1 cycle
        MOV       C, N:_char_array+2  ;; 1 cycle
        MOV       X, N:_char_array+1  ;; 1 cycle
        MOV       A, N:_char_array   ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
// 1172       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 54 cycles
// 1173       
// 1174     case 0x01910800:
// 1175       long_int= get_hr_energy(energy.Allph.apparent_imp,energy.Allph.apparent_imp_pulse);
??get_resp_91:
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x1A          ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        PUSH      DE                 ;; 1 cycle
          CFI CFA SP+48
        MOV       A, N:_energy+45    ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+50
        XCH       A, X               ;; 1 cycle
        MOVW      BC, N:_energy+70   ;; 1 cycle
        MOVW      AX, N:_energy+68   ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+52
        POP       DE                 ;; 1 cycle
          CFI CFA SP+50
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x26          ;; 1 cycle
          CFI FunCall _get_hr_energy
        CALL      _get_hr_energy     ;; 3 cycles
        MOVW      HL, AX             ;; 1 cycle
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+48
        POP       DE                 ;; 1 cycle
          CFI CFA SP+46
        MOVW      AX, [HL]           ;; 1 cycle
        MOVW      [DE], AX           ;; 1 cycle
        MOVW      AX, [HL+0x02]      ;; 1 cycle
        MOVW      [DE+0x02], AX      ;; 1 cycle
        MOVW      AX, [HL+0x04]      ;; 1 cycle
        MOVW      [DE+0x04], AX      ;; 1 cycle
        MOVW      AX, [HL+0x06]      ;; 1 cycle
        MOVW      [DE+0x06], AX      ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x1A          ;; 1 cycle
          CFI FunCall __LLU2L
        CALL      __LLU2L            ;; 3 cycles
        MOVW      S:_long_int, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_long_int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1176       long_into_char_array4(long_int,char_array);
        MOVW      DE, #LWRD(_char_array)  ;; 1 cycle
        MOVW      BC, S:_long_int+2  ;; 1 cycle
        MOVW      AX, S:_long_int    ;; 1 cycle
          CFI FunCall _long_into_char_array4
        CALL      _long_into_char_array4  ;; 3 cycles
// 1177       val_4byt2(char_array[0], char_array[1], char_array[2], char_array[3]); 
        MOV       B, N:_char_array+3  ;; 1 cycle
        MOV       C, N:_char_array+2  ;; 1 cycle
        MOV       X, N:_char_array+1  ;; 1 cycle
        MOV       A, N:_char_array   ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
// 1178       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 54 cycles
// 1179       
// 1180     case 0x01510728:                                                /* Phase angle RN */
// 1181       long_int= angle.Rph;
??get_resp_92:
        MOVW      AX, N:_angle       ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_long_int, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_long_int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1182       int_into_char_array(long_int,char_array);
        MOVW      BC, #LWRD(_char_array)  ;; 1 cycle
        MOVW      HL, S:_long_int+2  ;; 1 cycle
        MOVW      DE, S:_long_int    ;; 1 cycle
        MOVW      AX, DE             ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
// 1183       info[k++]=0x00;
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
// 1184       info[k++]=0x10;
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, #0x10           ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
// 1185       info[k++]=char_array[0];
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, N:_char_array   ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
// 1186       info[k++]=char_array[1];
        MOV       A, N:_char_array+1  ;; 1 cycle
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
// 1187       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 38 cycles
// 1188       
// 1189     case 0x01510733:                                                /* Phase angle YN*/
// 1190       long_int= angle.Yph;                       
??get_resp_93:
        MOVW      AX, N:_angle+2     ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_long_int, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_long_int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1191       int_into_char_array(long_int,char_array);
        MOVW      BC, #LWRD(_char_array)  ;; 1 cycle
        MOVW      HL, S:_long_int+2  ;; 1 cycle
        MOVW      DE, S:_long_int    ;; 1 cycle
        MOVW      AX, DE             ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
// 1192       info[k++]=0x00;
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
// 1193       info[k++]=0x10;
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, #0x10           ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
// 1194       info[k++]=char_array[0];
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, N:_char_array   ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
// 1195       info[k++]=char_array[1];
        MOV       A, N:_char_array+1  ;; 1 cycle
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
// 1196       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 38 cycles
// 1197       
// 1198     case 0x0151073E:                                                /* Phase angle BN */
// 1199       long_int= angle.Bph;
??get_resp_94:
        MOVW      AX, N:_angle+4     ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_long_int, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_long_int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1200       int_into_char_array(long_int,char_array);
        MOVW      BC, #LWRD(_char_array)  ;; 1 cycle
        MOVW      HL, S:_long_int+2  ;; 1 cycle
        MOVW      DE, S:_long_int    ;; 1 cycle
        MOVW      AX, DE             ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
// 1201       info[k++]=0x00;
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
// 1202       info[k++]=0x10;
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, #0x10           ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
// 1203       info[k++]=char_array[0];
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, N:_char_array   ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
// 1204       info[k++]=char_array[1];
        MOV       A, N:_char_array+1  ;; 1 cycle
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
// 1205       break;	
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 38 cycles
// 1206       
// 1207     case 0x01510701:                                                /* Phase angle RY */
// 1208       long_int= ph_ph.Ph_RY.angle.value;
??get_resp_95:
        MOVW      AX, N:_ph_ph+2     ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        MOVW      S:_long_int, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_long_int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1209       int_into_char_array(long_int,char_array);
        MOVW      BC, #LWRD(_char_array)  ;; 1 cycle
        MOVW      HL, S:_long_int+2  ;; 1 cycle
        MOVW      DE, S:_long_int    ;; 1 cycle
        MOVW      AX, DE             ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
// 1210       info[k++]=0x00;
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
// 1211       info[k++]=0x10;
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, #0x10           ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
// 1212       info[k++]=char_array[0];
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, N:_char_array   ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
// 1213       info[k++]=char_array[1];
        MOV       A, N:_char_array+1  ;; 1 cycle
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
// 1214       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 36 cycles
// 1215       
// 1216     case 0x01510702:                                                /* Phase angle YB */
// 1217       long_int= ph_ph.Ph_YB.angle.value;                          /* Pending correct this */
??get_resp_96:
        MOVW      AX, N:_ph_ph+10    ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        MOVW      S:_long_int, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_long_int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1218       int_into_char_array(long_int,char_array);
        MOVW      BC, #LWRD(_char_array)  ;; 1 cycle
        MOVW      HL, S:_long_int+2  ;; 1 cycle
        MOVW      DE, S:_long_int    ;; 1 cycle
        MOVW      AX, DE             ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
// 1219       info[k++]=0x00;
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
// 1220       info[k++]=0x10;
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, #0x10           ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
// 1221       info[k++]=char_array[0];
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, N:_char_array   ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
// 1222       info[k++]=char_array[1];
        MOV       A, N:_char_array+1  ;; 1 cycle
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
// 1223       break;
        BR        S:??compartment3_149  ;; 3 cycles
        ; ------------------------------------- Block: 36 cycles
// 1224       
// 1225     case 0x0151070C:                                                /* Phase angle BR */
// 1226       long_int= ph_ph.Ph_BR.angle.value;
??get_resp_97:
        MOVW      AX, N:_ph_ph+18    ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        MOVW      S:_long_int, AX    ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_long_int+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
// 1227       int_into_char_array(long_int,char_array);
        MOVW      BC, #LWRD(_char_array)  ;; 1 cycle
        MOVW      HL, S:_long_int+2  ;; 1 cycle
        MOVW      DE, S:_long_int    ;; 1 cycle
        MOVW      AX, DE             ;; 1 cycle
          CFI FunCall _int_into_char_array
        CALL      _int_into_char_array  ;; 3 cycles
// 1228       info[k++]=0x00;
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
// 1229       info[k++]=0x10;
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, #0x10           ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
// 1230       info[k++]=char_array[0];
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, N:_char_array   ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
// 1231       info[k++]=char_array[1];
        MOV       A, N:_char_array+1  ;; 1 cycle
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
// 1232       break;	
        BR        S:??compartment3_149  ;; 3 cycles
          CFI FunCall _fill_0b
        ; ------------------------------------- Block: 36 cycles
// 1233       
// 1234       /*case 0x01910800:
// 1235       long_int = high_res_cal(f_energy_cum, f_energy_pulse);
// 1236       long_to_char_array();
// 1237       val_4byt2(char_array[0],char_array[1],char_array[2],char_array[3]); // Hr fun 
// 1238       break;*/
// 1239       
// 1240     default:
// 1241       fill_0b();
??get_resp_98:
        CALL      _fill_0b           ;; 3 cycles
// 1242       break;
        ; ------------------------------------- Block: 3 cycles
// 1243     }
// 1244     break;
??compartment3_149:
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
// 1245   case 0x0303:
// 1246     switch(obis1)
??compartment3_95:
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, #LWRD(??get_resp_2)  ;; 1 cycle
        MOV       ES, #BYTE3(??get_resp_2)  ;; 1 cycle
        MOV       CS, #BYTE3(_get_resp)  ;; 1 cycle
        BR        N:?L_VSWITCH_L10   ;; 3 cycles
        ; ------------------------------------- Block: 9 cycles
// 1247     {
// 1248     case 0x00000102:
// 1249       sca_unit(0, 255, 1);
??get_resp_99:
        MOV       C, #0x1            ;; 1 cycle
        MOV       X, #0xFF           ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _sca_unit
        CALL      _sca_unit          ;; 3 cycles
// 1250       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 9 cycles
// 1251       
// 1252     case 0x011f0700:
// 1253     case 0x01330700:
// 1254     case 0x01470700:
// 1255     case 0x015b0700:
// 1256       sca_unit(SCALER_CURRENT, 33, 1);
??get_resp_100:
        MOV       C, #0x1            ;; 1 cycle
        MOV       X, #0x21           ;; 1 cycle
        MOV       A, #0xFD           ;; 1 cycle
          CFI FunCall _sca_unit
        CALL      _sca_unit          ;; 3 cycles
// 1257       //sca_unit((0xfe) + scalar_cur, 33, 1);
// 1258       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 9 cycles
// 1259       
// 1260     case 0x01200700:
// 1261     case 0x01340700:
// 1262     case 0x01480700:
// 1263     case 0x017C0700:
// 1264     case 0x017D0700:
// 1265     case 0x017E0700:
// 1266       //sca_unit(0-decimal.voltage, 35, 1);
// 1267       sca_unit(SCALER_VOLTAGE, 35, 1);
??get_resp_101:
        MOV       C, #0x1            ;; 1 cycle
        MOV       X, #0x23           ;; 1 cycle
        MOV       A, #0xFE           ;; 1 cycle
          CFI FunCall _sca_unit
        CALL      _sca_unit          ;; 3 cycles
// 1268       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 9 cycles
// 1269       
// 1270     case 0x010D0700:
// 1271     case 0x01210700:
// 1272     case 0x01350700:
// 1273     case 0x01490700:
// 1274       sca_unit(SCALER_PF, 0xff, 1);
??get_resp_102:
        MOV       C, #0x1            ;; 1 cycle
        MOV       X, #0xFF           ;; 1 cycle
        MOV       A, #0xFD           ;; 1 cycle
          CFI FunCall _sca_unit
        CALL      _sca_unit          ;; 3 cycles
// 1275       //sca_unit(0xfd, 0xff, 1);
// 1276       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 9 cycles
// 1277       
// 1278     case 0x010E0700:
// 1279     case 0x01220700:
// 1280     case 0x01360700:
// 1281     case 0x014A0700:
// 1282       sca_unit(SCALER_FREQ, 44, 1);
??get_resp_103:
        MOV       C, #0x1            ;; 1 cycle
        MOV       X, #0x2C           ;; 1 cycle
        MOV       A, #0xFD           ;; 1 cycle
          CFI FunCall _sca_unit
        CALL      _sca_unit          ;; 3 cycles
// 1283       //sca_unit(0xfd, 44, 1);
// 1284       break;
        BR        S:??compartment3_150  ;; 3 cycles
        ; ------------------------------------- Block: 9 cycles
// 1285       
// 1286     case 0x01010700:
// 1287     case 0x01150700:
// 1288     case 0x01290700:
// 1289     case 0x013D0700:
// 1290     case 0x01010200:
// 1291       sca_unit(SCALER_POWER, 27, 1); /* pending, decimal for demand is also as same as power */
??get_resp_104:
        MOV       C, #0x1            ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MOV       A, #0xFF           ;; 1 cycle
          CFI FunCall _sca_unit
        CALL      _sca_unit          ;; 3 cycles
// 1292       break;
        BR        S:??compartment3_150  ;; 3 cycles
        ; ------------------------------------- Block: 9 cycles
// 1293       
// 1294     case 0x01030700:
// 1295     case 0x01170700:
// 1296     case 0x012B0700:
// 1297     case 0x013F0700:
// 1298       sca_unit(SCALER_POWER, 29, 1);
??get_resp_105:
        MOV       C, #0x1            ;; 1 cycle
        MOV       X, #0x1D           ;; 1 cycle
        MOV       A, #0xFF           ;; 1 cycle
          CFI FunCall _sca_unit
        CALL      _sca_unit          ;; 3 cycles
// 1299       //sca_unit(0, 29, 1);
// 1300       break;
        BR        S:??compartment3_150  ;; 3 cycles
        ; ------------------------------------- Block: 9 cycles
// 1301       
// 1302     case 0x01090700:
// 1303     case 0x011D0700:
// 1304     case 0x01310700:
// 1305     case 0x01450700:
// 1306     case 0x01090200:
// 1307       sca_unit(SCALER_POWER, 28, 1);
??get_resp_106:
        MOV       C, #0x1            ;; 1 cycle
        MOV       X, #0x1C           ;; 1 cycle
        MOV       A, #0xFF           ;; 1 cycle
          CFI FunCall _sca_unit
        CALL      _sca_unit          ;; 3 cycles
// 1308       //sca_unit(0, 28, 1);
// 1309       break;
        BR        S:??compartment3_150  ;; 3 cycles
        ; ------------------------------------- Block: 9 cycles
// 1310       
// 1311     case 0x005e5b08:
// 1312     case 0x005e5b0d:
// 1313       sca_unit(0, 0x06, 1);
??get_resp_107:
        MOV       C, #0x1            ;; 1 cycle
        MOV       X, #0x6            ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _sca_unit
        CALL      _sca_unit          ;; 3 cycles
// 1314       break;
        BR        S:??compartment3_150  ;; 3 cycles
        ; ------------------------------------- Block: 9 cycles
// 1315       
// 1316     case 0x01050800:
// 1317     case 0x01060800:
// 1318     case 0x01070800:
// 1319     case 0x01080800:
// 1320       //sca_unit(0x00 + scalar_energy, 32, 1);
// 1321       sca_unit(SCALER_ENERGY, 32, 1);
??get_resp_108:
        MOV       C, #0x1            ;; 1 cycle
        MOV       X, #0x20           ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _sca_unit
        CALL      _sca_unit          ;; 3 cycles
// 1322       break;
        BR        S:??compartment3_150  ;; 3 cycles
        ; ------------------------------------- Block: 9 cycles
// 1323       
// 1324     case 0x01090800:
// 1325     case 0x010A0800:
// 1326       //sca_unit(0x00 + scalar_energy, 31, 1);
// 1327       sca_unit(SCALER_ENERGY, 31, 1);
??get_resp_109:
        MOV       C, #0x1            ;; 1 cycle
        MOV       X, #0x1F           ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _sca_unit
        CALL      _sca_unit          ;; 3 cycles
// 1328       break;
        BR        S:??compartment3_150  ;; 3 cycles
        ; ------------------------------------- Block: 9 cycles
// 1329       
// 1330     case 0x01800800:
// 1331       //        case 0x01960800:
// 1332     case 0x01010800:
// 1333     case 0x01020800:
// 1334       //sca_unit(0x00 + scalar_energy, 30, 1);
// 1335       sca_unit(SCALER_ENERGY, 30, 1);
??get_resp_110:
        MOV       C, #0x1            ;; 1 cycle
        MOV       X, #0x1E           ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _sca_unit
        CALL      _sca_unit          ;; 3 cycles
// 1336       break;
        BR        S:??compartment3_150  ;; 3 cycles
        ; ------------------------------------- Block: 9 cycles
// 1337       
// 1338     case 0x018E0800:
// 1339       //sca_unit(0xfd + (6 - HREnergy_decimal), 30, 1);
// 1340       sca_unit(SCALER_ENERGY_HR, 30, 1);
??get_resp_111:
        MOV       C, #0x1            ;; 1 cycle
        MOV       X, #0x1E           ;; 1 cycle
        MOV       A, #0xFD           ;; 1 cycle
          CFI FunCall _sca_unit
        CALL      _sca_unit          ;; 3 cycles
// 1341       break;
        BR        S:??compartment3_150  ;; 3 cycles
        ; ------------------------------------- Block: 9 cycles
// 1342       
// 1343     case 0x018F0800:
// 1344     case 0x01900800:
// 1345       //sca_unit(0xfd + (6 - HREnergy_decimal), 32, 1);
// 1346       sca_unit(SCALER_ENERGY_HR, 32, 1);
??get_resp_112:
        MOV       C, #0x1            ;; 1 cycle
        MOV       X, #0x20           ;; 1 cycle
        MOV       A, #0xFD           ;; 1 cycle
          CFI FunCall _sca_unit
        CALL      _sca_unit          ;; 3 cycles
// 1347       break;
        BR        S:??compartment3_150  ;; 3 cycles
        ; ------------------------------------- Block: 9 cycles
// 1348       
// 1349     case 0x01910800:
// 1350       //sca_unit(0xfd + (6 - HREnergy_decimal), 31, 1);
// 1351       sca_unit(SCALER_ENERGY_HR, 31, 1);
??get_resp_113:
        MOV       C, #0x1            ;; 1 cycle
        MOV       X, #0x1F           ;; 1 cycle
        MOV       A, #0xFD           ;; 1 cycle
          CFI FunCall _sca_unit
        CALL      _sca_unit          ;; 3 cycles
// 1352       break;
        BR        S:??compartment3_150  ;; 3 cycles
        ; ------------------------------------- Block: 9 cycles
// 1353     case 0x01510728:
// 1354     case 0x01510733:
// 1355     case 0x0151073E:    
// 1356     case 0x01510701:                                                        /* Phase angle RY */
// 1357     case 0x01510702:                                                        /* Phase angle RB */
// 1358     case 0x0151070C:                                                        /* Phase angle YB */
// 1359       sca_unit(SCALER_ANGLE,0x08,1);
??get_resp_114:
        MOV       C, #0x1            ;; 1 cycle
        MOV       X, #0x8            ;; 1 cycle
        MOV       A, #0xFE           ;; 1 cycle
          CFI FunCall _sca_unit
        CALL      _sca_unit          ;; 3 cycles
// 1360       break;
        BR        S:??compartment3_150  ;; 3 cycles
          CFI FunCall _fill_0b
        ; ------------------------------------- Block: 9 cycles
// 1361       
// 1362     default:
// 1363       fill_0b();
??get_resp_115:
        CALL      _fill_0b           ;; 3 cycles
// 1364       break;
        ; ------------------------------------- Block: 3 cycles
// 1365     }
// 1366     break;
??compartment3_150:
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
// 1367     
// 1368   case 0x0402:
// 1369     switch(obis1)
??compartment3_96:
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, #LWRD(??get_resp_3)  ;; 1 cycle
        MOV       ES, #BYTE3(??get_resp_3)  ;; 1 cycle
        MOV       CS, #BYTE3(_get_resp)  ;; 1 cycle
        BR        N:?L_VSWITCH_L10   ;; 3 cycles
        ; ------------------------------------- Block: 9 cycles
// 1370     {
// 1371     case 0x01010600:
// 1372       long_into_char_array3(demand.Allph.act_imp.max.value,char_array);
??get_resp_116:
        MOVW      DE, #LWRD(_char_array)  ;; 1 cycle
        MOVW      BC, N:_demand+142  ;; 1 cycle
        MOVW      AX, N:_demand+140  ;; 1 cycle
          CFI FunCall _long_into_char_array3
        CALL      _long_into_char_array3  ;; 3 cycles
// 1373       val_4byt2(0, char_array[0], char_array[1], char_array[2]);                /* md kW imp*/
        MOV       B, N:_char_array+2  ;; 1 cycle
        MOV       C, N:_char_array+1  ;; 1 cycle
        MOV       X, N:_char_array   ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
// 1374       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 16 cycles
// 1375     case 0x01010601:
// 1376     case 0x01010602:
// 1377     case 0x01010603:
// 1378     case 0x01010604:
// 1379     case 0x01010605:
// 1380     case 0x01010606:
// 1381     case 0x01010607:
// 1382     case 0x01010608:
// 1383       eprom_read(0x0400+(obis_e-1)*0x20,0,PAGE_1,AUTO_CALC);
??get_resp_117:
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
        MOV       X, N:_obis_e       ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x20          ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #0x3E0         ;; 1 cycle
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
// 1384       val_4byt2(0, opr_data[0], opr_data[1], opr_data[2]);               
        MOV       B, N:_opr_data+2   ;; 1 cycle
        MOV       C, N:_opr_data+1   ;; 1 cycle
        MOV       X, N:_opr_data     ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
// 1385       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 40 cycles
// 1386     case 0x01020600:
// 1387       long_into_char_array3(demand.Allph.act_exp.max.value,char_array);
??get_resp_118:
        MOVW      DE, #LWRD(_char_array)  ;; 1 cycle
        MOVW      BC, N:_demand+166  ;; 1 cycle
        MOVW      AX, N:_demand+164  ;; 1 cycle
          CFI FunCall _long_into_char_array3
        CALL      _long_into_char_array3  ;; 3 cycles
// 1388       val_4byt2(0, char_array[0], char_array[1], char_array[2]);                     /* md kW exp*/
        MOV       B, N:_char_array+2  ;; 1 cycle
        MOV       C, N:_char_array+1  ;; 1 cycle
        MOV       X, N:_char_array   ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
// 1389       break;    
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 16 cycles
// 1390     case 0x01020601:
// 1391     case 0x01020602:
// 1392     case 0x01020603:
// 1393     case 0x01020604:
// 1394     case 0x01020605:
// 1395     case 0x01020606:
// 1396     case 0x01020607:
// 1397     case 0x01020608:
// 1398       eprom_read(0x0400+(obis_e-1)*0x20,0,PAGE_1,AUTO_CALC);
??get_resp_119:
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
        MOV       X, N:_obis_e       ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x20          ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #0x3E0         ;; 1 cycle
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
// 1399       val_4byt2(0, opr_data[7], opr_data[8], opr_data[9]);               
        MOV       B, N:_opr_data+9   ;; 1 cycle
        MOV       C, N:_opr_data+8   ;; 1 cycle
        MOV       X, N:_opr_data+7   ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
// 1400       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 40 cycles
// 1401     case 0x01090600:
// 1402       long_into_char_array3(demand.Allph.app_imp.max.value,char_array);
??get_resp_120:
        MOVW      DE, #LWRD(_char_array)  ;; 1 cycle
        MOVW      BC, N:_demand+190  ;; 1 cycle
        MOVW      AX, N:_demand+188  ;; 1 cycle
          CFI FunCall _long_into_char_array3
        CALL      _long_into_char_array3  ;; 3 cycles
// 1403       val_4byt2(0, char_array[0], char_array[1], char_array[2]);                   /* md kva imp*/
        MOV       B, N:_char_array+2  ;; 1 cycle
        MOV       C, N:_char_array+1  ;; 1 cycle
        MOV       X, N:_char_array   ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
// 1404       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 16 cycles
// 1405     case 0x01090601:
// 1406     case 0x01090602:
// 1407     case 0x01090603:
// 1408     case 0x01090604:
// 1409     case 0x01090605:
// 1410     case 0x01090606:
// 1411     case 0x01090607:
// 1412     case 0x01090608:
// 1413       eprom_read(0x0410+(obis_e-1)*0x20,0,PAGE_1,AUTO_CALC);
??get_resp_121:
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
        MOV       X, N:_obis_e       ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x20          ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #0x3F0         ;; 1 cycle
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
// 1414       val_4byt2(0, opr_data[0], opr_data[1], opr_data[2]);               
        MOV       B, N:_opr_data+2   ;; 1 cycle
        MOV       C, N:_opr_data+1   ;; 1 cycle
        MOV       X, N:_opr_data     ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
// 1415       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 40 cycles
// 1416     case 0x010A0600:
// 1417       long_into_char_array3(demand.Allph.app_exp.max.value,char_array);
??get_resp_122:
        MOVW      DE, #LWRD(_char_array)  ;; 1 cycle
        MOVW      BC, N:_demand+214  ;; 1 cycle
        MOVW      AX, N:_demand+212  ;; 1 cycle
          CFI FunCall _long_into_char_array3
        CALL      _long_into_char_array3  ;; 3 cycles
// 1418       val_4byt2(0, char_array[0], char_array[1], char_array[2]);                 /* md kva exp*/
        MOV       B, N:_char_array+2  ;; 1 cycle
        MOV       C, N:_char_array+1  ;; 1 cycle
        MOV       X, N:_char_array   ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
// 1419       break;   
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 16 cycles
// 1420     case 0x010A0601:
// 1421     case 0x010A0602:
// 1422     case 0x010A0603:
// 1423     case 0x010A0604:
// 1424     case 0x010A0605:
// 1425     case 0x010A0606:
// 1426     case 0x010A0607:
// 1427     case 0x010A0608:
// 1428       eprom_read(0x0410+(obis_e-1)*0x20,0,PAGE_1,AUTO_CALC);
??get_resp_123:
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
        MOV       X, N:_obis_e       ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x20          ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #0x3F0         ;; 1 cycle
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
// 1429       val_4byt2(0, opr_data[7], opr_data[8], opr_data[9]);               
        MOV       B, N:_opr_data+9   ;; 1 cycle
        MOV       C, N:_opr_data+8   ;; 1 cycle
        MOV       X, N:_opr_data+7   ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
// 1430       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 40 cycles
// 1431     case 0x01150600:
// 1432       long_into_char_array3(demand.Rph.act_imp.max.value,char_array);
??get_resp_124:
        MOVW      DE, #LWRD(_char_array)  ;; 1 cycle
        MOVW      BC, N:_demand+18   ;; 1 cycle
        MOVW      AX, N:_demand+16   ;; 1 cycle
          CFI FunCall _long_into_char_array3
        CALL      _long_into_char_array3  ;; 3 cycles
// 1433       val_4byt2(0, char_array[0], char_array[1], char_array[2]);                /* md kW imp*/
        MOV       B, N:_char_array+2  ;; 1 cycle
        MOV       C, N:_char_array+1  ;; 1 cycle
        MOV       X, N:_char_array   ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
// 1434       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 16 cycles
// 1435     case 0x01160600:
// 1436       long_into_char_array3(demand.Rph.act_exp.max.value,char_array);
??get_resp_125:
        MOVW      DE, #LWRD(_char_array)  ;; 1 cycle
        MOVW      BC, N:_demand+38   ;; 1 cycle
        MOVW      AX, N:_demand+36   ;; 1 cycle
          CFI FunCall _long_into_char_array3
        CALL      _long_into_char_array3  ;; 3 cycles
// 1437       val_4byt2(0, char_array[0], char_array[1], char_array[2]);                     /* md kW exp*/
        MOV       B, N:_char_array+2  ;; 1 cycle
        MOV       C, N:_char_array+1  ;; 1 cycle
        MOV       X, N:_char_array   ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
// 1438       break;    
        BR        S:??compartment3_151  ;; 3 cycles
        ; ------------------------------------- Block: 16 cycles
// 1439     case 0x01290600:
// 1440       long_into_char_array3(demand.Yph.act_imp.max.value,char_array);
??get_resp_126:
        MOVW      DE, #LWRD(_char_array)  ;; 1 cycle
        MOVW      BC, N:_demand+58   ;; 1 cycle
        MOVW      AX, N:_demand+56   ;; 1 cycle
          CFI FunCall _long_into_char_array3
        CALL      _long_into_char_array3  ;; 3 cycles
// 1441       val_4byt2(0, char_array[0], char_array[1], char_array[2]);                /* md kW imp*/
        MOV       B, N:_char_array+2  ;; 1 cycle
        MOV       C, N:_char_array+1  ;; 1 cycle
        MOV       X, N:_char_array   ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
// 1442       break;
        BR        S:??compartment3_151  ;; 3 cycles
        ; ------------------------------------- Block: 16 cycles
// 1443     case 0x012A0600:
// 1444       long_into_char_array3(demand.Yph.act_exp.max.value,char_array);
??get_resp_127:
        MOVW      DE, #LWRD(_char_array)  ;; 1 cycle
        MOVW      BC, N:_demand+78   ;; 1 cycle
        MOVW      AX, N:_demand+76   ;; 1 cycle
          CFI FunCall _long_into_char_array3
        CALL      _long_into_char_array3  ;; 3 cycles
// 1445       val_4byt2(0, char_array[0], char_array[1], char_array[2]);                     /* md kW exp*/
        MOV       B, N:_char_array+2  ;; 1 cycle
        MOV       C, N:_char_array+1  ;; 1 cycle
        MOV       X, N:_char_array   ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
// 1446       break;  
        BR        S:??compartment3_151  ;; 3 cycles
        ; ------------------------------------- Block: 16 cycles
// 1447     case 0x013D0600:
// 1448       long_into_char_array3(demand.Bph.act_imp.max.value,char_array);
??get_resp_128:
        MOVW      DE, #LWRD(_char_array)  ;; 1 cycle
        MOVW      BC, N:_demand+98   ;; 1 cycle
        MOVW      AX, N:_demand+96   ;; 1 cycle
          CFI FunCall _long_into_char_array3
        CALL      _long_into_char_array3  ;; 3 cycles
// 1449       val_4byt2(0, char_array[0], char_array[1], char_array[2]);                /* md kW imp*/
        MOV       B, N:_char_array+2  ;; 1 cycle
        MOV       C, N:_char_array+1  ;; 1 cycle
        MOV       X, N:_char_array   ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
// 1450       break;
        BR        S:??compartment3_151  ;; 3 cycles
        ; ------------------------------------- Block: 16 cycles
// 1451     case 0x013E0600:
// 1452       long_into_char_array3(demand.Bph.act_exp.max.value,char_array);
??get_resp_129:
        MOVW      DE, #LWRD(_char_array)  ;; 1 cycle
        MOVW      BC, N:_demand+118  ;; 1 cycle
        MOVW      AX, N:_demand+116  ;; 1 cycle
          CFI FunCall _long_into_char_array3
        CALL      _long_into_char_array3  ;; 3 cycles
// 1453       val_4byt2(0, char_array[0], char_array[1], char_array[2]);                     /* md kW exp*/
        MOV       B, N:_char_array+2  ;; 1 cycle
        MOV       C, N:_char_array+1  ;; 1 cycle
        MOV       X, N:_char_array   ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
// 1454       break;  
        BR        S:??compartment3_151  ;; 3 cycles
          CFI FunCall _fill_0b
        ; ------------------------------------- Block: 16 cycles
// 1455     default:
// 1456       fill_0b();
??get_resp_130:
        CALL      _fill_0b           ;; 3 cycles
// 1457       break;
        ; ------------------------------------- Block: 3 cycles
// 1458     }
// 1459     break;
??compartment3_151:
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
// 1460   case 0x0403:
// 1461     switch(obis1)
??compartment3_97:
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, #LWRD(??get_resp_4)  ;; 1 cycle
        MOV       ES, #BYTE3(??get_resp_4)  ;; 1 cycle
        MOV       CS, #BYTE3(_get_resp)  ;; 1 cycle
        BR        N:?L_VSWITCH_L10   ;; 3 cycles
        ; ------------------------------------- Block: 9 cycles
// 1462     {
// 1463     case 0x01010600:
// 1464     case 0x01010601:
// 1465     case 0x01010602:
// 1466     case 0x01010603:
// 1467     case 0x01010604:
// 1468     case 0x01010605:
// 1469     case 0x01010606:
// 1470     case 0x01010607:
// 1471     case 0x01010608:
// 1472     case 0x01020600:
// 1473     case 0x01020601:
// 1474     case 0x01020602:
// 1475     case 0x01020603:
// 1476     case 0x01020604:
// 1477     case 0x01020605:
// 1478     case 0x01020606:
// 1479     case 0x01020607:
// 1480     case 0x01020608:
// 1481     case 0x01150600:
// 1482     case 0x01160600:
// 1483     case 0x01290600:
// 1484     case 0x01090601:
// 1485     case 0x01090602:
// 1486     case 0x01090603:
// 1487     case 0x01090604:
// 1488     case 0x01090605:
// 1489     case 0x01090606:
// 1490     case 0x01090607:
// 1491     case 0x01090608:
// 1492     case 0x012A0600:
// 1493     case 0x010A0601:
// 1494     case 0x010A0602:
// 1495     case 0x010A0603:
// 1496     case 0x010A0604:
// 1497     case 0x010A0605:
// 1498     case 0x010A0606:
// 1499     case 0x010A0607:
// 1500     case 0x010A0608:
// 1501     case 0x013D0600:
// 1502     case 0x013E0600:
// 1503       sca_unit(0 + scalar_energy, 27, 1);
??get_resp_131:
        MOV       C, #0x1            ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MOV       A, N:_scalar_energy  ;; 1 cycle
          CFI FunCall _sca_unit
        CALL      _sca_unit          ;; 3 cycles
// 1504       break;
        BR        S:??compartment3_152  ;; 3 cycles
        ; ------------------------------------- Block: 9 cycles
// 1505       
// 1506     case 0x01090600:
// 1507     case 0x010A0600:
// 1508       sca_unit(0 + scalar_energy, 28, 1);
??get_resp_132:
        MOV       C, #0x1            ;; 1 cycle
        MOV       X, #0x1C           ;; 1 cycle
        MOV       A, N:_scalar_energy  ;; 1 cycle
          CFI FunCall _sca_unit
        CALL      _sca_unit          ;; 3 cycles
// 1509       break;
        BR        S:??compartment3_152  ;; 3 cycles
          CFI FunCall _fill_0b
        ; ------------------------------------- Block: 9 cycles
// 1510       
// 1511     default:
// 1512       fill_0b();
??get_resp_133:
        CALL      _fill_0b           ;; 3 cycles
// 1513       break;
        ; ------------------------------------- Block: 3 cycles
// 1514     }
// 1515     break;
??compartment3_152:
        BR        N:??compartment3_134  ;; 3 cycles
          CFI FunCall _fill_0d
        ; ------------------------------------- Block: 3 cycles
// 1516   case 0x0404:
// 1517     fill_0d();
??compartment3_98:
        CALL      _fill_0d           ;; 3 cycles
// 1518     break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
// 1519     
// 1520   case 0x0405:
// 1521     switch(obis1)
??compartment3_99:
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, #LWRD(??get_resp_5)  ;; 1 cycle
        MOV       ES, #BYTE3(??get_resp_5)  ;; 1 cycle
        MOV       CS, #BYTE3(_get_resp)  ;; 1 cycle
        BR        N:?L_VSWITCH_L10   ;; 3 cycles
        ; ------------------------------------- Block: 9 cycles
// 1522     {
// 1523     case 0x01010600:
// 1524       eprom_read(0x03E0,0,PAGE_1,AUTO_CALC);
??get_resp_134:
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x3E0         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
// 1525       TempTime = char_array_into_time4(&opr_data[3]);
        MOVW      BC, #LWRD(_opr_data+3)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x1A          ;; 1 cycle
          CFI FunCall _char_array_into_time4
        CALL      _char_array_into_time4  ;; 3 cycles
        MOVW      HL, #LWRD(_TempTime)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x1A          ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOV       A, [DE]            ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
        MOV       A, [DE+0x01]       ;; 1 cycle
        MOV       [HL+0x01], A       ;; 1 cycle
        MOV       A, [DE+0x02]       ;; 1 cycle
        MOV       [HL+0x02], A       ;; 1 cycle
        MOV       A, [DE+0x03]       ;; 1 cycle
        MOV       [HL+0x03], A       ;; 1 cycle
        MOV       A, [DE+0x04]       ;; 1 cycle
        MOV       [HL+0x04], A       ;; 1 cycle
        MOV       A, [DE+0x05]       ;; 1 cycle
        MOV       [HL+0x05], A       ;; 1 cycle
        MOV       A, [DE+0x06]       ;; 1 cycle
        MOV       [HL+0x06], A       ;; 1 cycle
// 1526       date_time(TempTime.day,TempTime.month,TempTime.year,TempTime.hour,TempTime.min,TempTime.sec,1);
        MOV       X, #0x1            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+48
        MOV       A, N:_TempTime     ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, N:_TempTime+1   ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       B, N:_TempTime+2   ;; 1 cycle
        MOV       C, N:_TempTime+6   ;; 1 cycle
        MOV       X, N:_TempTime+5   ;; 1 cycle
        MOV       A, N:_TempTime+3   ;; 1 cycle
          CFI FunCall _date_time
        CALL      _date_time         ;; 3 cycles
// 1527       break;
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+46
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 48 cycles
// 1528     case 0x01010601:
// 1529     case 0x01010602:
// 1530     case 0x01010603:
// 1531     case 0x01010604:
// 1532     case 0x01010605:
// 1533     case 0x01010606:
// 1534     case 0x01010607:
// 1535     case 0x01010608:
// 1536       eprom_read(0x0400+(obis_e-1)*0x20,0,PAGE_1,AUTO_CALC);
??get_resp_135:
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
        MOV       X, N:_obis_e       ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x20          ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #0x3E0         ;; 1 cycle
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
// 1537       TempTime = char_array_into_time4(&opr_data[3]);
        MOVW      BC, #LWRD(_opr_data+3)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x1A          ;; 1 cycle
          CFI FunCall _char_array_into_time4
        CALL      _char_array_into_time4  ;; 3 cycles
        MOVW      HL, #LWRD(_TempTime)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x1A          ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOV       A, [DE]            ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
        MOV       A, [DE+0x01]       ;; 1 cycle
        MOV       [HL+0x01], A       ;; 1 cycle
        MOV       A, [DE+0x02]       ;; 1 cycle
        MOV       [HL+0x02], A       ;; 1 cycle
        MOV       A, [DE+0x03]       ;; 1 cycle
        MOV       [HL+0x03], A       ;; 1 cycle
        MOV       A, [DE+0x04]       ;; 1 cycle
        MOV       [HL+0x04], A       ;; 1 cycle
        MOV       A, [DE+0x05]       ;; 1 cycle
        MOV       [HL+0x05], A       ;; 1 cycle
        MOV       A, [DE+0x06]       ;; 1 cycle
        MOV       [HL+0x06], A       ;; 1 cycle
// 1538       date_time(TempTime.day,TempTime.month,TempTime.year,TempTime.hour,TempTime.min,TempTime.sec,1);
        MOV       X, #0x1            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+48
        MOV       A, N:_TempTime     ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, N:_TempTime+1   ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       B, N:_TempTime+2   ;; 1 cycle
        MOV       C, N:_TempTime+6   ;; 1 cycle
        MOV       X, N:_TempTime+5   ;; 1 cycle
        MOV       A, N:_TempTime+3   ;; 1 cycle
          CFI FunCall _date_time
        CALL      _date_time         ;; 3 cycles
// 1539       break;
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+46
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 71 cycles
// 1540     case 0x01020600:
// 1541       eprom_read(0x03E0,0,PAGE_1,AUTO_CALC);
??get_resp_136:
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x3E0         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
// 1542       TempTime = char_array_into_time4(&opr_data[10]);
        MOVW      BC, #LWRD(_opr_data+10)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x1A          ;; 1 cycle
          CFI FunCall _char_array_into_time4
        CALL      _char_array_into_time4  ;; 3 cycles
        MOVW      HL, #LWRD(_TempTime)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x1A          ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOV       A, [DE]            ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
        MOV       A, [DE+0x01]       ;; 1 cycle
        MOV       [HL+0x01], A       ;; 1 cycle
        MOV       A, [DE+0x02]       ;; 1 cycle
        MOV       [HL+0x02], A       ;; 1 cycle
        MOV       A, [DE+0x03]       ;; 1 cycle
        MOV       [HL+0x03], A       ;; 1 cycle
        MOV       A, [DE+0x04]       ;; 1 cycle
        MOV       [HL+0x04], A       ;; 1 cycle
        MOV       A, [DE+0x05]       ;; 1 cycle
        MOV       [HL+0x05], A       ;; 1 cycle
        MOV       A, [DE+0x06]       ;; 1 cycle
        MOV       [HL+0x06], A       ;; 1 cycle
// 1543       date_time(TempTime.day,TempTime.month,TempTime.year,TempTime.hour,TempTime.min,TempTime.sec,1);
        MOV       X, #0x1            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+48
        MOV       A, N:_TempTime     ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, N:_TempTime+1   ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       B, N:_TempTime+2   ;; 1 cycle
        MOV       C, N:_TempTime+6   ;; 1 cycle
        MOV       X, N:_TempTime+5   ;; 1 cycle
        MOV       A, N:_TempTime+3   ;; 1 cycle
          CFI FunCall _date_time
        CALL      _date_time         ;; 3 cycles
// 1544       break;    
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+46
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 48 cycles
// 1545     case 0x01020601:
// 1546     case 0x01020602:
// 1547     case 0x01020603:
// 1548     case 0x01020604:
// 1549     case 0x01020605:
// 1550     case 0x01020606:
// 1551     case 0x01020607:
// 1552     case 0x01020608:
// 1553       eprom_read(0x0400+(obis_e-1)*0x20,0,PAGE_1,AUTO_CALC);
??get_resp_137:
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
        MOV       X, N:_obis_e       ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x20          ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #0x3E0         ;; 1 cycle
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
// 1554       TempTime = char_array_into_time4(&opr_data[10]);
        MOVW      BC, #LWRD(_opr_data+10)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x1A          ;; 1 cycle
          CFI FunCall _char_array_into_time4
        CALL      _char_array_into_time4  ;; 3 cycles
        MOVW      HL, #LWRD(_TempTime)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x1A          ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOV       A, [DE]            ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
        MOV       A, [DE+0x01]       ;; 1 cycle
        MOV       [HL+0x01], A       ;; 1 cycle
        MOV       A, [DE+0x02]       ;; 1 cycle
        MOV       [HL+0x02], A       ;; 1 cycle
        MOV       A, [DE+0x03]       ;; 1 cycle
        MOV       [HL+0x03], A       ;; 1 cycle
        MOV       A, [DE+0x04]       ;; 1 cycle
        MOV       [HL+0x04], A       ;; 1 cycle
        MOV       A, [DE+0x05]       ;; 1 cycle
        MOV       [HL+0x05], A       ;; 1 cycle
        MOV       A, [DE+0x06]       ;; 1 cycle
        MOV       [HL+0x06], A       ;; 1 cycle
// 1555       date_time(TempTime.day,TempTime.month,TempTime.year,TempTime.hour,TempTime.min,TempTime.sec,1);
        MOV       X, #0x1            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+48
        MOV       A, N:_TempTime     ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, N:_TempTime+1   ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       B, N:_TempTime+2   ;; 1 cycle
        MOV       C, N:_TempTime+6   ;; 1 cycle
        MOV       X, N:_TempTime+5   ;; 1 cycle
        MOV       A, N:_TempTime+3   ;; 1 cycle
          CFI FunCall _date_time
        CALL      _date_time         ;; 3 cycles
// 1556       break;
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+46
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 71 cycles
// 1557     case 0x01090600:
// 1558       eprom_read(0x03F0,0,PAGE_1,AUTO_CALC);
??get_resp_138:
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x3F0         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
// 1559       TempTime = char_array_into_time4(&opr_data[3]);
        MOVW      BC, #LWRD(_opr_data+3)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x1A          ;; 1 cycle
          CFI FunCall _char_array_into_time4
        CALL      _char_array_into_time4  ;; 3 cycles
        MOVW      HL, #LWRD(_TempTime)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x1A          ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOV       A, [DE]            ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
        MOV       A, [DE+0x01]       ;; 1 cycle
        MOV       [HL+0x01], A       ;; 1 cycle
        MOV       A, [DE+0x02]       ;; 1 cycle
        MOV       [HL+0x02], A       ;; 1 cycle
        MOV       A, [DE+0x03]       ;; 1 cycle
        MOV       [HL+0x03], A       ;; 1 cycle
        MOV       A, [DE+0x04]       ;; 1 cycle
        MOV       [HL+0x04], A       ;; 1 cycle
        MOV       A, [DE+0x05]       ;; 1 cycle
        MOV       [HL+0x05], A       ;; 1 cycle
        MOV       A, [DE+0x06]       ;; 1 cycle
        MOV       [HL+0x06], A       ;; 1 cycle
// 1560       date_time(TempTime.day,TempTime.month,TempTime.year,TempTime.hour,TempTime.min,TempTime.sec,1);
        MOV       X, #0x1            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+48
        MOV       A, N:_TempTime     ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, N:_TempTime+1   ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       B, N:_TempTime+2   ;; 1 cycle
        MOV       C, N:_TempTime+6   ;; 1 cycle
        MOV       X, N:_TempTime+5   ;; 1 cycle
        MOV       A, N:_TempTime+3   ;; 1 cycle
          CFI FunCall _date_time
        CALL      _date_time         ;; 3 cycles
// 1561       break;
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+46
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 48 cycles
// 1562     case 0x01090601:
// 1563     case 0x01090602:
// 1564     case 0x01090603:
// 1565     case 0x01090604:
// 1566     case 0x01090605:
// 1567     case 0x01090606:
// 1568     case 0x01090607:
// 1569     case 0x01090608:
// 1570       eprom_read(0x0410+(obis_e-1)*0x20,0,PAGE_1,AUTO_CALC);
??get_resp_139:
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
        MOV       X, N:_obis_e       ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x20          ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #0x3F0         ;; 1 cycle
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
// 1571       TempTime = char_array_into_time4(&opr_data[3]);
        MOVW      BC, #LWRD(_opr_data+3)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x1A          ;; 1 cycle
          CFI FunCall _char_array_into_time4
        CALL      _char_array_into_time4  ;; 3 cycles
        MOVW      HL, #LWRD(_TempTime)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x1A          ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOV       A, [DE]            ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
        MOV       A, [DE+0x01]       ;; 1 cycle
        MOV       [HL+0x01], A       ;; 1 cycle
        MOV       A, [DE+0x02]       ;; 1 cycle
        MOV       [HL+0x02], A       ;; 1 cycle
        MOV       A, [DE+0x03]       ;; 1 cycle
        MOV       [HL+0x03], A       ;; 1 cycle
        MOV       A, [DE+0x04]       ;; 1 cycle
        MOV       [HL+0x04], A       ;; 1 cycle
        MOV       A, [DE+0x05]       ;; 1 cycle
        MOV       [HL+0x05], A       ;; 1 cycle
        MOV       A, [DE+0x06]       ;; 1 cycle
        MOV       [HL+0x06], A       ;; 1 cycle
// 1572       date_time(TempTime.day,TempTime.month,TempTime.year,TempTime.hour,TempTime.min,TempTime.sec,1);
        MOV       X, #0x1            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+48
        MOV       A, N:_TempTime     ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, N:_TempTime+1   ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       B, N:_TempTime+2   ;; 1 cycle
        MOV       C, N:_TempTime+6   ;; 1 cycle
        MOV       X, N:_TempTime+5   ;; 1 cycle
        MOV       A, N:_TempTime+3   ;; 1 cycle
          CFI FunCall _date_time
        CALL      _date_time         ;; 3 cycles
// 1573       break;
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+46
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 71 cycles
// 1574     case 0x010A0600:
// 1575       eprom_read(0x03F0,0,PAGE_1,AUTO_CALC);
??get_resp_140:
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x3F0         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
// 1576       TempTime = char_array_into_time4(&opr_data[10]);
        MOVW      BC, #LWRD(_opr_data+10)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x1A          ;; 1 cycle
          CFI FunCall _char_array_into_time4
        CALL      _char_array_into_time4  ;; 3 cycles
        MOVW      HL, #LWRD(_TempTime)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x1A          ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOV       A, [DE]            ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
        MOV       A, [DE+0x01]       ;; 1 cycle
        MOV       [HL+0x01], A       ;; 1 cycle
        MOV       A, [DE+0x02]       ;; 1 cycle
        MOV       [HL+0x02], A       ;; 1 cycle
        MOV       A, [DE+0x03]       ;; 1 cycle
        MOV       [HL+0x03], A       ;; 1 cycle
        MOV       A, [DE+0x04]       ;; 1 cycle
        MOV       [HL+0x04], A       ;; 1 cycle
        MOV       A, [DE+0x05]       ;; 1 cycle
        MOV       [HL+0x05], A       ;; 1 cycle
        MOV       A, [DE+0x06]       ;; 1 cycle
        MOV       [HL+0x06], A       ;; 1 cycle
// 1577       date_time(TempTime.day,TempTime.month,TempTime.year,TempTime.hour,TempTime.min,TempTime.sec,1);
        MOV       X, #0x1            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+48
        MOV       A, N:_TempTime     ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, N:_TempTime+1   ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       B, N:_TempTime+2   ;; 1 cycle
        MOV       C, N:_TempTime+6   ;; 1 cycle
        MOV       X, N:_TempTime+5   ;; 1 cycle
        MOV       A, N:_TempTime+3   ;; 1 cycle
          CFI FunCall _date_time
        CALL      _date_time         ;; 3 cycles
// 1578       break; 
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+46
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 48 cycles
// 1579     case 0x010A0601:
// 1580     case 0x010A0602:
// 1581     case 0x010A0603:
// 1582     case 0x010A0604:
// 1583     case 0x010A0605:
// 1584     case 0x010A0606:
// 1585     case 0x010A0607:
// 1586     case 0x010A0608:
// 1587       eprom_read(0x0410+(obis_e-1)*0x20,0,PAGE_1,AUTO_CALC);
??get_resp_141:
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
        MOV       X, N:_obis_e       ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x20          ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #0x3F0         ;; 1 cycle
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
// 1588       TempTime = char_array_into_time4(&opr_data[10]);
        MOVW      BC, #LWRD(_opr_data+10)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x1A          ;; 1 cycle
          CFI FunCall _char_array_into_time4
        CALL      _char_array_into_time4  ;; 3 cycles
        MOVW      HL, #LWRD(_TempTime)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x1A          ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOV       A, [DE]            ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
        MOV       A, [DE+0x01]       ;; 1 cycle
        MOV       [HL+0x01], A       ;; 1 cycle
        MOV       A, [DE+0x02]       ;; 1 cycle
        MOV       [HL+0x02], A       ;; 1 cycle
        MOV       A, [DE+0x03]       ;; 1 cycle
        MOV       [HL+0x03], A       ;; 1 cycle
        MOV       A, [DE+0x04]       ;; 1 cycle
        MOV       [HL+0x04], A       ;; 1 cycle
        MOV       A, [DE+0x05]       ;; 1 cycle
        MOV       [HL+0x05], A       ;; 1 cycle
        MOV       A, [DE+0x06]       ;; 1 cycle
        MOV       [HL+0x06], A       ;; 1 cycle
// 1589       date_time(TempTime.day,TempTime.month,TempTime.year,TempTime.hour,TempTime.min,TempTime.sec,1);
        MOV       X, #0x1            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+48
        MOV       A, N:_TempTime     ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, N:_TempTime+1   ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       B, N:_TempTime+2   ;; 1 cycle
        MOV       C, N:_TempTime+6   ;; 1 cycle
        MOV       X, N:_TempTime+5   ;; 1 cycle
        MOV       A, N:_TempTime+3   ;; 1 cycle
          CFI FunCall _date_time
        CALL      _date_time         ;; 3 cycles
// 1590       break;
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+46
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 71 cycles
// 1591     case 0x01150600:
// 1592       eprom_read(0x0380,0,PAGE_1,AUTO_CALC);
??get_resp_142:
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x380         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
// 1593       TempTime = char_array_into_time4(&opr_data[3]);
        MOVW      BC, #LWRD(_opr_data+3)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x1A          ;; 1 cycle
          CFI FunCall _char_array_into_time4
        CALL      _char_array_into_time4  ;; 3 cycles
        MOVW      HL, #LWRD(_TempTime)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x1A          ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOV       A, [DE]            ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
        MOV       A, [DE+0x01]       ;; 1 cycle
        MOV       [HL+0x01], A       ;; 1 cycle
        MOV       A, [DE+0x02]       ;; 1 cycle
        MOV       [HL+0x02], A       ;; 1 cycle
        MOV       A, [DE+0x03]       ;; 1 cycle
        MOV       [HL+0x03], A       ;; 1 cycle
        MOV       A, [DE+0x04]       ;; 1 cycle
        MOV       [HL+0x04], A       ;; 1 cycle
        MOV       A, [DE+0x05]       ;; 1 cycle
        MOV       [HL+0x05], A       ;; 1 cycle
        MOV       A, [DE+0x06]       ;; 1 cycle
        MOV       [HL+0x06], A       ;; 1 cycle
// 1594       date_time(TempTime.day,TempTime.month,TempTime.year,TempTime.hour,TempTime.min,TempTime.sec,1);
        MOV       X, #0x1            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+48
        MOV       A, N:_TempTime     ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, N:_TempTime+1   ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       B, N:_TempTime+2   ;; 1 cycle
        MOV       C, N:_TempTime+6   ;; 1 cycle
        MOV       X, N:_TempTime+5   ;; 1 cycle
        MOV       A, N:_TempTime+3   ;; 1 cycle
          CFI FunCall _date_time
        CALL      _date_time         ;; 3 cycles
// 1595       break;
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+46
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 48 cycles
// 1596     case 0x01160600:
// 1597       eprom_read(0x0380,0,PAGE_1,AUTO_CALC);
??get_resp_143:
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x380         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
// 1598       TempTime = char_array_into_time4(&opr_data[10]);
        MOVW      BC, #LWRD(_opr_data+10)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x1A          ;; 1 cycle
          CFI FunCall _char_array_into_time4
        CALL      _char_array_into_time4  ;; 3 cycles
        MOVW      HL, #LWRD(_TempTime)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x1A          ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOV       A, [DE]            ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
        MOV       A, [DE+0x01]       ;; 1 cycle
        MOV       [HL+0x01], A       ;; 1 cycle
        MOV       A, [DE+0x02]       ;; 1 cycle
        MOV       [HL+0x02], A       ;; 1 cycle
        MOV       A, [DE+0x03]       ;; 1 cycle
        MOV       [HL+0x03], A       ;; 1 cycle
        MOV       A, [DE+0x04]       ;; 1 cycle
        MOV       [HL+0x04], A       ;; 1 cycle
        MOV       A, [DE+0x05]       ;; 1 cycle
        MOV       [HL+0x05], A       ;; 1 cycle
        MOV       A, [DE+0x06]       ;; 1 cycle
        MOV       [HL+0x06], A       ;; 1 cycle
// 1599       date_time(TempTime.day,TempTime.month,TempTime.year,TempTime.hour,TempTime.min,TempTime.sec,1);
        MOV       X, #0x1            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+48
        MOV       A, N:_TempTime     ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, N:_TempTime+1   ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       B, N:_TempTime+2   ;; 1 cycle
        MOV       C, N:_TempTime+6   ;; 1 cycle
        MOV       X, N:_TempTime+5   ;; 1 cycle
        MOV       A, N:_TempTime+3   ;; 1 cycle
          CFI FunCall _date_time
        CALL      _date_time         ;; 3 cycles
// 1600       break;    
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+46
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 48 cycles
// 1601     case 0x01290600:
// 1602       eprom_read(0x0390,0,PAGE_1,AUTO_CALC);
??get_resp_144:
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x390         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
// 1603       TempTime = char_array_into_time4(&opr_data[3]);
        MOVW      BC, #LWRD(_opr_data+3)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x1A          ;; 1 cycle
          CFI FunCall _char_array_into_time4
        CALL      _char_array_into_time4  ;; 3 cycles
        MOVW      HL, #LWRD(_TempTime)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x1A          ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOV       A, [DE]            ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
        MOV       A, [DE+0x01]       ;; 1 cycle
        MOV       [HL+0x01], A       ;; 1 cycle
        MOV       A, [DE+0x02]       ;; 1 cycle
        MOV       [HL+0x02], A       ;; 1 cycle
        MOV       A, [DE+0x03]       ;; 1 cycle
        MOV       [HL+0x03], A       ;; 1 cycle
        MOV       A, [DE+0x04]       ;; 1 cycle
        MOV       [HL+0x04], A       ;; 1 cycle
        MOV       A, [DE+0x05]       ;; 1 cycle
        MOV       [HL+0x05], A       ;; 1 cycle
        MOV       A, [DE+0x06]       ;; 1 cycle
        MOV       [HL+0x06], A       ;; 1 cycle
// 1604       date_time(TempTime.day,TempTime.month,TempTime.year,TempTime.hour,TempTime.min,TempTime.sec,1);
        MOV       X, #0x1            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+48
        MOV       A, N:_TempTime     ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, N:_TempTime+1   ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       B, N:_TempTime+2   ;; 1 cycle
        MOV       C, N:_TempTime+6   ;; 1 cycle
        MOV       X, N:_TempTime+5   ;; 1 cycle
        MOV       A, N:_TempTime+3   ;; 1 cycle
          CFI FunCall _date_time
        CALL      _date_time         ;; 3 cycles
// 1605       break;
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+46
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 48 cycles
// 1606     case 0x012A0600:
// 1607       eprom_read(0x0390,0,PAGE_1,AUTO_CALC);
??get_resp_145:
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x390         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
// 1608       TempTime = char_array_into_time4(&opr_data[10]);
        MOVW      BC, #LWRD(_opr_data+10)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x1A          ;; 1 cycle
          CFI FunCall _char_array_into_time4
        CALL      _char_array_into_time4  ;; 3 cycles
        MOVW      HL, #LWRD(_TempTime)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x1A          ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOV       A, [DE]            ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
        MOV       A, [DE+0x01]       ;; 1 cycle
        MOV       [HL+0x01], A       ;; 1 cycle
        MOV       A, [DE+0x02]       ;; 1 cycle
        MOV       [HL+0x02], A       ;; 1 cycle
        MOV       A, [DE+0x03]       ;; 1 cycle
        MOV       [HL+0x03], A       ;; 1 cycle
        MOV       A, [DE+0x04]       ;; 1 cycle
        MOV       [HL+0x04], A       ;; 1 cycle
        MOV       A, [DE+0x05]       ;; 1 cycle
        MOV       [HL+0x05], A       ;; 1 cycle
        MOV       A, [DE+0x06]       ;; 1 cycle
        MOV       [HL+0x06], A       ;; 1 cycle
// 1609       date_time(TempTime.day,TempTime.month,TempTime.year,TempTime.hour,TempTime.min,TempTime.sec,1);
        MOV       X, #0x1            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+48
        MOV       A, N:_TempTime     ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, N:_TempTime+1   ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       B, N:_TempTime+2   ;; 1 cycle
        MOV       C, N:_TempTime+6   ;; 1 cycle
        MOV       X, N:_TempTime+5   ;; 1 cycle
        MOV       A, N:_TempTime+3   ;; 1 cycle
          CFI FunCall _date_time
        CALL      _date_time         ;; 3 cycles
// 1610       break;    
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+46
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 48 cycles
// 1611     case 0x013D0600:
// 1612       eprom_read(0x03A0,0,PAGE_1,AUTO_CALC);
??get_resp_146:
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x3A0         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
// 1613       TempTime = char_array_into_time4(&opr_data[3]);
        MOVW      BC, #LWRD(_opr_data+3)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x1A          ;; 1 cycle
          CFI FunCall _char_array_into_time4
        CALL      _char_array_into_time4  ;; 3 cycles
        MOVW      HL, #LWRD(_TempTime)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x1A          ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOV       A, [DE]            ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
        MOV       A, [DE+0x01]       ;; 1 cycle
        MOV       [HL+0x01], A       ;; 1 cycle
        MOV       A, [DE+0x02]       ;; 1 cycle
        MOV       [HL+0x02], A       ;; 1 cycle
        MOV       A, [DE+0x03]       ;; 1 cycle
        MOV       [HL+0x03], A       ;; 1 cycle
        MOV       A, [DE+0x04]       ;; 1 cycle
        MOV       [HL+0x04], A       ;; 1 cycle
        MOV       A, [DE+0x05]       ;; 1 cycle
        MOV       [HL+0x05], A       ;; 1 cycle
        MOV       A, [DE+0x06]       ;; 1 cycle
        MOV       [HL+0x06], A       ;; 1 cycle
// 1614       date_time(TempTime.day,TempTime.month,TempTime.year,TempTime.hour,TempTime.min,TempTime.sec,1);
        MOV       X, #0x1            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+48
        MOV       A, N:_TempTime     ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, N:_TempTime+1   ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       B, N:_TempTime+2   ;; 1 cycle
        MOV       C, N:_TempTime+6   ;; 1 cycle
        MOV       X, N:_TempTime+5   ;; 1 cycle
        MOV       A, N:_TempTime+3   ;; 1 cycle
          CFI FunCall _date_time
        CALL      _date_time         ;; 3 cycles
// 1615       break;
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+46
        BR        S:??compartment3_153  ;; 3 cycles
        ; ------------------------------------- Block: 48 cycles
// 1616     case 0x013E0600:
// 1617       eprom_read(0x03A0,0,PAGE_1,AUTO_CALC);
??get_resp_147:
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x3A0         ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
// 1618       TempTime = char_array_into_time4(&opr_data[10]);
        MOVW      BC, #LWRD(_opr_data+10)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x1A          ;; 1 cycle
          CFI FunCall _char_array_into_time4
        CALL      _char_array_into_time4  ;; 3 cycles
        MOVW      HL, #LWRD(_TempTime)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x1A          ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOV       A, [DE]            ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
        MOV       A, [DE+0x01]       ;; 1 cycle
        MOV       [HL+0x01], A       ;; 1 cycle
        MOV       A, [DE+0x02]       ;; 1 cycle
        MOV       [HL+0x02], A       ;; 1 cycle
        MOV       A, [DE+0x03]       ;; 1 cycle
        MOV       [HL+0x03], A       ;; 1 cycle
        MOV       A, [DE+0x04]       ;; 1 cycle
        MOV       [HL+0x04], A       ;; 1 cycle
        MOV       A, [DE+0x05]       ;; 1 cycle
        MOV       [HL+0x05], A       ;; 1 cycle
        MOV       A, [DE+0x06]       ;; 1 cycle
        MOV       [HL+0x06], A       ;; 1 cycle
// 1619       date_time(TempTime.day,TempTime.month,TempTime.year,TempTime.hour,TempTime.min,TempTime.sec,1);
        MOV       X, #0x1            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+48
        MOV       A, N:_TempTime     ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, N:_TempTime+1   ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       B, N:_TempTime+2   ;; 1 cycle
        MOV       C, N:_TempTime+6   ;; 1 cycle
        MOV       X, N:_TempTime+5   ;; 1 cycle
        MOV       A, N:_TempTime+3   ;; 1 cycle
          CFI FunCall _date_time
        CALL      _date_time         ;; 3 cycles
// 1620       break;   
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+46
        BR        S:??compartment3_153  ;; 3 cycles
          CFI FunCall _fill_0b
        ; ------------------------------------- Block: 48 cycles
// 1621     default:
// 1622       fill_0b();
??get_resp_148:
        CALL      _fill_0b           ;; 3 cycles
// 1623       break;
        ; ------------------------------------- Block: 3 cycles
// 1624     }
// 1625     break;
??compartment3_153:
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
// 1626     
// 1627   case 0x0702:
// 1628     i_dlms= 0;
??compartment3_100:
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_i_dlms, AX      ;; 1 cycle
// 1629     switch(obis1)
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, #LWRD(??get_resp_6)  ;; 1 cycle
        MOV       ES, #BYTE3(??get_resp_6)  ;; 1 cycle
        MOV       CS, #BYTE3(_get_resp)  ;; 1 cycle
        BR        N:?L_VSWITCH_L10   ;; 3 cycles
          CFI FunCall _buffer_instantaneous_parameter
        ; ------------------------------------- Block: 11 cycles
// 1630     {
// 1631     case 0x015e5b00:
// 1632       buffer_instantaneous_parameter();
??get_resp_149:
        CALL      _buffer_instantaneous_parameter  ;; 3 cycles
// 1633       /***Meter read count is incremented whenever instant profile is read****/
// 1634       ////////            meter_read_cnt++;
// 1635       ////////            OPR10=meter_read_cnt;
// 1636       ////////            OPR11[14]=cal_chksum(&OPR10,15);
// 1637       ////////            write_data_16(16,0x08e0);
// 1638       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
// 1639       
// 1640     case 0x015e5b03:
// 1641       buffer_scaler_filler(instantaneous_parameter_scaler_buffer);
??get_resp_150:
        MOVW      AX, #LWRD(_instantaneous_parameter_scaler_buffer)  ;; 1 cycle
          CFI FunCall _buffer_scaler_filler
        CALL      _buffer_scaler_filler  ;; 3 cycles
// 1642       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 1643       
// 1644     case 0x015e5b04:
// 1645       buffer_scaler_filler(blockload_survey_parameter_scaler_buffer);
??get_resp_151:
        MOVW      AX, #LWRD(_blockload_survey_parameter_scaler_buffer)  ;; 1 cycle
          CFI FunCall _buffer_scaler_filler
        CALL      _buffer_scaler_filler  ;; 3 cycles
// 1646       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 1647       
// 1648     case 0x015e5b05:
// 1649       buffer_scaler_filler(dailyload_profile_parameter_scaler_buffer);
??get_resp_152:
        MOVW      AX, #LWRD(_dailyload_profile_parameter_scaler_buffer)  ;; 1 cycle
          CFI FunCall _buffer_scaler_filler
        CALL      _buffer_scaler_filler  ;; 3 cycles
// 1650       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 1651       
// 1652     case 0x015e5b06:
// 1653       buffer_scaler_filler(bill_profile_parameter_scaler_buffer);
??get_resp_153:
        MOVW      AX, #LWRD(_bill_profile_parameter_scaler_buffer)  ;; 1 cycle
          CFI FunCall _buffer_scaler_filler
        CALL      _buffer_scaler_filler  ;; 3 cycles
// 1654       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 1655       
// 1656     case 0x015e5b07:
// 1657       buffer_scaler_filler(event_log_profile_scaler_buffer);
??get_resp_154:
        MOVW      AX, #LWRD(_event_log_profile_scaler_buffer)  ;; 1 cycle
          CFI FunCall _buffer_scaler_filler
        CALL      _buffer_scaler_filler  ;; 3 cycles
// 1658       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 1659       
// 1660     case 0x01630100:
// 1661       ls_count_local= 0x00;
??get_resp_155:
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_ls_count_local, AX  ;; 1 cycle
// 1662       ls_count_dlms= load_survey_cnt + 1; /* no. of load survey */
        MOVW      AX, N:_load_survey_cnt  ;; 1 cycle
        INCW      AX                 ;; 1 cycle
        MOVW      N:_ls_count_dlms, AX  ;; 1 cycle
// 1663       if(lsro_flag == 1)
        CMP       N:_lsro_flag, #0x1  ;; 1 cycle
        BNZ       ??compartment3_154  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
// 1664       {
// 1665         UintLoadSurptr= ls_count_dlms;
        MOVW      AX, N:_ls_count_dlms  ;; 1 cycle
        MOVW      N:_UintLoadSurptr, AX  ;; 1 cycle
// 1666         ls_count_dlms= MAX_LS;
        MOVW      AX, #0xE40         ;; 1 cycle
        MOVW      N:_ls_count_dlms, AX  ;; 1 cycle
        BR        S:??compartment3_155  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 1667       }
// 1668       else
// 1669       {
// 1670         UintLoadSurptr= 0;
??compartment3_154:
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_UintLoadSurptr, AX  ;; 1 cycle
          CFI FunCall _Sel_Loadsurvey_buffer
        ; ------------------------------------- Block: 2 cycles
// 1671       }
// 1672       Sel_Loadsurvey_buffer();
??compartment3_155:
        CALL      _Sel_Loadsurvey_buffer  ;; 3 cycles
// 1673       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
// 1674       
// 1675     case 0x01630200:                      /* //daily load profile */
// 1676       dls_count_dlms= midnight_par_cnt; /* char_array_to_int(&opr_data[0]);//no. of load survey */
??get_resp_156:
        MOV       X, N:_midnight_par_cnt  ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      N:_dls_count_dlms, AX  ;; 1 cycle
// 1677       UintLoadSurptr1= DLOADSURVEY_INIT_ADD;
        MOVW      AX, #0x4000        ;; 1 cycle
        MOVW      N:_UintLoadSurptr1, AX  ;; 1 cycle
// 1678       if((midnight_roll_f == 1) && (sel_access_flag != 1))
        CMP       N:_midnight_roll_f, #0x1  ;; 1 cycle
        BNZ       ??compartment3_156  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
        CMP       N:_sel_access_flag, #0x1  ;; 1 cycle
        BZ        ??compartment3_156  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 1679       {
// 1680         UintLoadSurptr1+= (uint16_t)(midnight_par_cnt * DAILY_ENERGY_SNAP_SIZE);
        MOV       X, N:_midnight_par_cnt  ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x60          ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, N:_UintLoadSurptr1  ;; 1 cycle
        MOVW      N:_UintLoadSurptr1, AX  ;; 1 cycle
// 1681         dls_count_dlms= max_midnight_cnt;
        MOVW      AX, #0x4C          ;; 1 cycle
        MOVW      N:_dls_count_dlms, AX  ;; 1 cycle
          CFI FunCall _Sel_DailyLoadsurvey_buffer
        ; ------------------------------------- Block: 9 cycles
// 1682       }
// 1683       Sel_DailyLoadsurvey_buffer();
??compartment3_156:
        CALL      _Sel_DailyLoadsurvey_buffer  ;; 3 cycles
// 1684       break;
        BR        N:??compartment3_134  ;; 3 cycles
          CFI FunCall _save_tod_data
        ; ------------------------------------- Block: 6 cycles
// 1685       
// 1686     case 0x01620100:
// 1687       save_tod_data();
??get_resp_157:
        CALL      _save_tod_data     ;; 3 cycles
// 1688       bill_buffer();
          CFI FunCall _bill_buffer
        CALL      _bill_buffer       ;; 3 cycles
// 1689       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 9 cycles
// 1690       ////////            
// 1691     case 0x00636200:
// 1692       tamper_data= tpr.vol_related_count;
??get_resp_158:
        MOV       X, N:_tpr          ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      N:_tamper_data, AX  ;; 1 cycle
// 1693       tamper_compart(COMPART_VOLTAGE_ENTRIES, COMPART_VOLTAGE_START_ADD, COMPART_VOLTAGE_END_ADD, COMPART_VOLTAGE_SIZE, tpr.vol_related_overflow);
        MOV       A, N:_tpr+1        ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+48
        XCH       A, X               ;; 1 cycle
        MOV       X, N:_COMPART_VOLTAGE_SIZE  ;; 1 cycle
        MOVW      DE, N:_COMPART_VOLTAGE_END_ADD  ;; 1 cycle
        MOVW      BC, N:_COMPART_VOLTAGE_START_ADD  ;; 1 cycle
        MOV       A, N:_COMPART_VOLTAGE_ENTRIES  ;; 1 cycle
          CFI FunCall _tamper_compart
        CALL      _tamper_compart    ;; 3 cycles
// 1694       break;
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+46
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 18 cycles
// 1695       
// 1696     case 0x00636201:
// 1697       tamper_data= tpr.curr_related_count;
??get_resp_159:
        MOV       X, N:_tpr+4        ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      N:_tamper_data, AX  ;; 1 cycle
// 1698       tamper_compart(COMPART_CURRENT_ENTRIES, COMPART_CURRENT_START_ADD, COMPART_CURRENT_END_ADD, COMPART_CURRENT_SIZE, tpr.curr_related_overflow);
        MOV       A, N:_tpr+5        ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+48
        XCH       A, X               ;; 1 cycle
        MOV       X, N:_COMPART_CURRENT_SIZE  ;; 1 cycle
        MOVW      DE, N:_COMPART_CURRENT_END_ADD  ;; 1 cycle
        MOVW      BC, N:_COMPART_CURRENT_START_ADD  ;; 1 cycle
        MOV       A, N:_COMPART_CURRENT_ENTRIES  ;; 1 cycle
          CFI FunCall _tamper_compart
        CALL      _tamper_compart    ;; 3 cycles
// 1699       break;
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+46
        BR        N:??compartment3_134  ;; 3 cycles
          CFI FunCall _compartment3
        ; ------------------------------------- Block: 18 cycles
// 1700       
// 1701     case 0x00636202:
// 1702       compartment3(); 
??get_resp_160:
        CALL      _compartment3      ;; 3 cycles
// 1703       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
// 1704       
// 1705     case 0x00636203:
// 1706       tamper_data= tpr.transaction_count;
??get_resp_161:
        MOV       X, N:_tpr+12       ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      N:_tamper_data, AX  ;; 1 cycle
// 1707       tamper_compart(COMPART_TRANSACTION_ENTRIES, COMPART_TRANSACTION_START_ADD, COMPART_TRANSACTION_END_ADD, COMPART_TRANSACTION_SIZE, tpr.transaction_overflow);
        MOV       A, N:_tpr+13       ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+48
        XCH       A, X               ;; 1 cycle
        MOV       X, N:_COMPART_TRANSACTION_SIZE  ;; 1 cycle
        MOVW      DE, N:_COMPART_TRANSACTION_END_ADD  ;; 1 cycle
        MOVW      BC, N:_COMPART_TRANSACTION_START_ADD  ;; 1 cycle
        MOV       A, N:_COMPART_TRANSACTION_ENTRIES  ;; 1 cycle
          CFI FunCall _tamper_compart
        CALL      _tamper_compart    ;; 3 cycles
// 1708       break;
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+46
        BR        S:??compartment3_157  ;; 3 cycles
        ; ------------------------------------- Block: 18 cycles
// 1709       
// 1710     case 0x00636204:
// 1711       tamper_data= tpr.others_count;
??get_resp_162:
        MOV       X, N:_tpr+16       ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      N:_tamper_data, AX  ;; 1 cycle
// 1712       tamper_compart(COMPART_OTHERS_ENTRIES, COMPART_OTHERS_START_ADD, COMPART_OTHERS_END_ADD, COMPART_OTHERS_SIZE, tpr.others_overflow);
        MOV       A, N:_tpr+17       ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+48
        XCH       A, X               ;; 1 cycle
        MOV       X, N:_COMPART_OTHERS_SIZE  ;; 1 cycle
        MOVW      DE, N:_COMPART_OTHERS_END_ADD  ;; 1 cycle
        MOVW      BC, N:_COMPART_OTHERS_START_ADD  ;; 1 cycle
        MOV       A, N:_COMPART_OTHERS_ENTRIES  ;; 1 cycle
          CFI FunCall _tamper_compart
        CALL      _tamper_compart    ;; 3 cycles
// 1713       break;
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+46
        BR        S:??compartment3_157  ;; 3 cycles
        ; ------------------------------------- Block: 18 cycles
// 1714       
// 1715     case 0x00636205:
// 1716       tamper_data= tpr.non_roll_count;
??get_resp_163:
        MOV       X, N:_tpr+20       ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      N:_tamper_data, AX  ;; 1 cycle
// 1717       if(TOP_RESTORE_REQ == 1)
        CMP       N:_TOP_RESTORE_REQ, #0x1  ;; 1 cycle
        BNZ       ??compartment3_158  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
// 1718       {
// 1719         tamper_compart(COMPART_NONROLLOVER_ENTRIES, COMPART_NONROLLOVER_START_ADD, COMPART_NONROLLOVER_END_ADD, COMPART_NONROLLOVER_SIZE, tpr.non_roll_overflow);
        MOV       A, N:_tpr+21       ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+48
        XCH       A, X               ;; 1 cycle
        MOV       X, N:_COMPART_NONROLLOVER_SIZE  ;; 1 cycle
        MOVW      DE, N:_COMPART_NONROLLOVER_END_ADD  ;; 1 cycle
        MOVW      BC, N:_COMPART_NONROLLOVER_START_ADD  ;; 1 cycle
        MOV       A, N:_COMPART_NONROLLOVER_ENTRIES  ;; 1 cycle
          CFI FunCall _tamper_compart
        CALL      _tamper_compart    ;; 3 cycles
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+46
        BR        S:??compartment3_157  ;; 3 cycles
          CFI FunCall _compartment6
        ; ------------------------------------- Block: 15 cycles
// 1720       }
// 1721       else
// 1722       {
// 1723         compartment6();
??compartment3_158:
        CALL      _compartment6      ;; 3 cycles
// 1724       }
// 1725       break;
        BR        S:??compartment3_157  ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
// 1726     case 0x00636263:
// 1727       tamper_data= tpr.debug_count;
??get_resp_164:
        MOV       X, N:_tpr+24       ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      N:_tamper_data, AX  ;; 1 cycle
// 1728       tamper_compart(COMPART_DEBUG_ENTRIES, COMPART_DEBUG_START_ADD, COMPART_DEBUG_END_ADD, COMPART_DEBUG_SIZE, tpr.debug_overflow);
        MOV       A, N:_tpr+25       ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+48
        XCH       A, X               ;; 1 cycle
        MOV       X, #0x10           ;; 1 cycle
        MOVW      DE, #0x7500        ;; 1 cycle
        MOVW      BC, #0x7400        ;; 1 cycle
        MOV       A, #0x10           ;; 1 cycle
          CFI FunCall _tamper_compart
        CALL      _tamper_compart    ;; 3 cycles
// 1729       break;
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+46
        BR        S:??compartment3_157  ;; 3 cycles
          CFI FunCall _name_plate_buffer
        ; ------------------------------------- Block: 18 cycles
// 1730     case 0x005e5b0a:
// 1731       name_plate_buffer();
??get_resp_165:
        CALL      _name_plate_buffer  ;; 3 cycles
// 1732       break;  
        BR        S:??compartment3_157  ;; 3 cycles
          CFI FunCall _fill_0b
        ; ------------------------------------- Block: 6 cycles
// 1733       
// 1734     default:
// 1735       fill_0b();
??get_resp_166:
        CALL      _fill_0b           ;; 3 cycles
// 1736       break;
        ; ------------------------------------- Block: 3 cycles
// 1737     }
// 1738     break;
??compartment3_157:
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
// 1739   case 0x0703:
// 1740     i_dlms= 0;
??compartment3_101:
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_i_dlms, AX      ;; 1 cycle
// 1741     switch(obis1)
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, #LWRD(??get_resp_7)  ;; 1 cycle
        MOV       ES, #BYTE3(??get_resp_7)  ;; 1 cycle
        MOV       CS, #BYTE3(_get_resp)  ;; 1 cycle
        BR        N:?L_VSWITCH_L10   ;; 3 cycles
        ; ------------------------------------- Block: 11 cycles
// 1742     {
// 1743     case 0x015e5b00:
// 1744       capture_objects_filler(instantaneous_parameter_cap_obj);
??get_resp_167:
        MOVW      AX, #LWRD(_instantaneous_parameter_cap_obj)  ;; 1 cycle
          CFI FunCall _capture_objects_filler
        CALL      _capture_objects_filler  ;; 3 cycles
// 1745       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 1746       
// 1747     case 0x015e5b03:
// 1748       capture_objects_filler(instantaneous_parameter_scaler_cap_obj); /* capture object attribute for billing */
??get_resp_168:
        MOVW      AX, #LWRD(_instantaneous_parameter_scaler_cap_obj)  ;; 1 cycle
          CFI FunCall _capture_objects_filler
        CALL      _capture_objects_filler  ;; 3 cycles
// 1749       break;
        BR        S:??compartment3_159  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 1750       
// 1751     case 0x015e5b04:
// 1752       capture_objects_filler(blockload_survey_parameter_scaler_cap_obj);
??get_resp_169:
        MOVW      AX, #LWRD(_blockload_survey_parameter_scaler_cap_obj)  ;; 1 cycle
          CFI FunCall _capture_objects_filler
        CALL      _capture_objects_filler  ;; 3 cycles
// 1753       break;
        BR        S:??compartment3_159  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 1754       
// 1755     case 0x015e5b05:
// 1756       capture_objects_filler(dailyload_profile_parameter_scaler_cap_obj);
??get_resp_170:
        MOVW      AX, #LWRD(_dailyload_profile_parameter_scaler_cap_obj)  ;; 1 cycle
          CFI FunCall _capture_objects_filler
        CALL      _capture_objects_filler  ;; 3 cycles
// 1757       break;
        BR        S:??compartment3_159  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 1758       
// 1759     case 0x015e5b06:
// 1760       capture_objects_filler(bill_profile_parameter_scaler_cap_obj); /* capture object attribute for billing */
??get_resp_171:
        MOVW      AX, #LWRD(_bill_profile_parameter_scaler_cap_obj)  ;; 1 cycle
          CFI FunCall _capture_objects_filler
        CALL      _capture_objects_filler  ;; 3 cycles
// 1761       break;
        BR        S:??compartment3_159  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 1762       
// 1763     case 0x015e5b07:
// 1764       capture_objects_filler(event_log_profile_scaler_cap_obj);
??get_resp_172:
        MOVW      AX, #LWRD(_event_log_profile_scaler_cap_obj)  ;; 1 cycle
          CFI FunCall _capture_objects_filler
        CALL      _capture_objects_filler  ;; 3 cycles
// 1765       break;
        BR        S:??compartment3_159  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 1766       
// 1767     case 0x01630100:
// 1768       capture_objects_filler(load_survey_parameter_cap_obj); /* capture object attribute for load survey */
??get_resp_173:
        MOVW      AX, #LWRD(_load_survey_parameter_cap_obj)  ;; 1 cycle
          CFI FunCall _capture_objects_filler
        CALL      _capture_objects_filler  ;; 3 cycles
// 1769       break;
        BR        S:??compartment3_159  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 1770       
// 1771     case 0x01630200:
// 1772       capture_objects_filler(dailyload_profile_parameter_cap_obj); /* capture object attribute for load survey */
??get_resp_174:
        MOVW      AX, #LWRD(_dailyload_profile_parameter_cap_obj)  ;; 1 cycle
          CFI FunCall _capture_objects_filler
        CALL      _capture_objects_filler  ;; 3 cycles
// 1773       break;
        BR        S:??compartment3_159  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 1774       
// 1775     case 0x01620100:
// 1776       capture_objects_filler(bill_profile_parameter_cap_obj); /* capture object attribute for billing */
??get_resp_175:
        MOVW      AX, #LWRD(_bill_profile_parameter_cap_obj)  ;; 1 cycle
          CFI FunCall _capture_objects_filler
        CALL      _capture_objects_filler  ;; 3 cycles
// 1777       break;
        BR        S:??compartment3_159  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 1778       
// 1779     case 0x00636200:
// 1780       capture_objects_filler(voltage_event_capture_obj);
??get_resp_176:
        MOVW      AX, #LWRD(_voltage_event_capture_obj)  ;; 1 cycle
          CFI FunCall _capture_objects_filler
        CALL      _capture_objects_filler  ;; 3 cycles
// 1781       break;
        BR        S:??compartment3_159  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 1782       
// 1783     case 0x00636201:
// 1784       capture_objects_filler(current_event_capture_obj);
??get_resp_177:
        MOVW      AX, #LWRD(_current_event_capture_obj)  ;; 1 cycle
          CFI FunCall _capture_objects_filler
        CALL      _capture_objects_filler  ;; 3 cycles
// 1785       break;
        BR        S:??compartment3_159  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 1786       
// 1787     case 0x00636202:
// 1788       capture_objects_filler(power_event_capture_obj);
??get_resp_178:
        MOVW      AX, #LWRD(_power_event_capture_obj)  ;; 1 cycle
          CFI FunCall _capture_objects_filler
        CALL      _capture_objects_filler  ;; 3 cycles
// 1789       break;
        BR        S:??compartment3_159  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 1790       
// 1791     case 0x00636203:
// 1792       capture_objects_filler(transaction_event_capture_obj);
??get_resp_179:
        MOVW      AX, #LWRD(_transaction_event_capture_obj)  ;; 1 cycle
          CFI FunCall _capture_objects_filler
        CALL      _capture_objects_filler  ;; 3 cycles
// 1793       break;
        BR        S:??compartment3_159  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 1794       
// 1795     case 0x00636204:
// 1796       capture_objects_filler(other_event_capture_obj);
??get_resp_180:
        MOVW      AX, #LWRD(_other_event_capture_obj)  ;; 1 cycle
          CFI FunCall _capture_objects_filler
        CALL      _capture_objects_filler  ;; 3 cycles
// 1797       break;
        BR        S:??compartment3_159  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 1798       
// 1799     case 0x00636205:
// 1800       capture_objects_filler(non_rollover_event_capture_obj);
??get_resp_181:
        MOVW      AX, #LWRD(_non_rollover_event_capture_obj)  ;; 1 cycle
          CFI FunCall _capture_objects_filler
        CALL      _capture_objects_filler  ;; 3 cycles
// 1801       break;
        BR        S:??compartment3_159  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 1802       
// 1803     case 0x00636263:
// 1804       capture_objects_filler(debug_event_capture_obj);
??get_resp_182:
        MOVW      AX, #LWRD(_debug_event_capture_obj)  ;; 1 cycle
          CFI FunCall _capture_objects_filler
        CALL      _capture_objects_filler  ;; 3 cycles
// 1805       break;    
        BR        S:??compartment3_159  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 1806       
// 1807     case 0x005e5b0a:
// 1808       capture_objects_filler(name_plate_profile_capture_obj);
??get_resp_183:
        MOVW      AX, #LWRD(_name_plate_profile_capture_obj)  ;; 1 cycle
          CFI FunCall _capture_objects_filler
        CALL      _capture_objects_filler  ;; 3 cycles
// 1809       break;
        BR        S:??compartment3_159  ;; 3 cycles
          CFI FunCall _fill_0b
        ; ------------------------------------- Block: 7 cycles
// 1810       
// 1811     default:
// 1812       fill_0b();
??get_resp_184:
        CALL      _fill_0b           ;; 3 cycles
// 1813       break;
        ; ------------------------------------- Block: 3 cycles
// 1814     }
// 1815     break;
??compartment3_159:
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
// 1816   case 0x0704:
// 1817     switch(obis1)
??compartment3_102:
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, #LWRD(??get_resp_8)  ;; 1 cycle
        MOV       ES, #BYTE3(??get_resp_8)  ;; 1 cycle
        MOV       CS, #BYTE3(_get_resp)  ;; 1 cycle
        BR        N:?L_VSWITCH_L10   ;; 3 cycles
        ; ------------------------------------- Block: 9 cycles
// 1818     {
// 1819     case 0x015e5b00:
// 1820     case 0x015e5b03:
// 1821     case 0x015e5b04:
// 1822     case 0x015e5b05:
// 1823     case 0x015e5b06:
// 1824     case 0x015e5b07:
// 1825     case 0x01620100:
// 1826     case 0x00636200:
// 1827     case 0x00636201:
// 1828     case 0x00636202:
// 1829     case 0x00636203:
// 1830     case 0x00636204:
// 1831     case 0x00636205:
// 1832     case 0x00636263:
// 1833     case 0x005e5b0a:
// 1834       val_4byt2(0x00, 0x00, 0x00, 0x00);
??get_resp_185:
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
// 1835       break;
        BR        S:??compartment3_160  ;; 3 cycles
        ; ------------------------------------- Block: 10 cycles
// 1836     case 0x01630100:
// 1837       val_4byt2(0x00, 0x00, ((3600 / mdi_sel_ls) / 256), ((3600 / mdi_sel_ls) % 256));
??get_resp_186:
        MOVW      BC, #0x100         ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+48
        POP       HL                 ;; 1 cycle
          CFI CFA SP+46
        MOV       C, N:_mdi_sel_ls   ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOVW      AX, #0xE10         ;; 1 cycle
          CFI FunCall ?SI_DIV_L02
        CALL      N:?SI_DIV_L02      ;; 3 cycles
        PUSH      HL                 ;; 1 cycle
          CFI CFA SP+48
        POP       BC                 ;; 1 cycle
          CFI CFA SP+46
          CFI FunCall ?SI_MOD_L02
        CALL      N:?SI_MOD_L02      ;; 3 cycles
        MOVW      HL, AX             ;; 1 cycle
        XCH       A, L               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, L               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        MOVW      BC, #0x100         ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+48
        POP       HL                 ;; 1 cycle
          CFI CFA SP+46
        MOV       C, N:_mdi_sel_ls   ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOVW      AX, #0xE10         ;; 1 cycle
          CFI FunCall ?SI_DIV_L02
        CALL      N:?SI_DIV_L02      ;; 3 cycles
        PUSH      HL                 ;; 1 cycle
          CFI CFA SP+48
        POP       BC                 ;; 1 cycle
          CFI CFA SP+46
          CFI FunCall ?SI_DIV_L02
        CALL      N:?SI_DIV_L02      ;; 3 cycles
        XCH       A, X               ;; 1 cycle
        MOV       C, A               ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
// 1838       break;
        BR        S:??compartment3_160  ;; 3 cycles
        ; ------------------------------------- Block: 49 cycles
// 1839     case 0x01630200:
// 1840       val_4byt2(0x00, 0x01, 0x51, 0x80); /* capture period */
??get_resp_187:
        MOV       B, #0x80           ;; 1 cycle
        MOV       C, #0x51           ;; 1 cycle
        MOV       X, #0x1            ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
// 1841       break;
        BR        S:??compartment3_160  ;; 3 cycles
          CFI FunCall _fill_0b
        ; ------------------------------------- Block: 10 cycles
// 1842     default:
// 1843       fill_0b();
??get_resp_188:
        CALL      _fill_0b           ;; 3 cycles
// 1844       break;
        ; ------------------------------------- Block: 3 cycles
// 1845     }
// 1846     break;
??compartment3_160:
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
// 1847   case 0x0705:
// 1848     switch(obis1)
??compartment3_103:
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, #LWRD(??get_resp_9)  ;; 1 cycle
        MOV       ES, #BYTE3(??get_resp_9)  ;; 1 cycle
        MOV       CS, #BYTE3(_get_resp)  ;; 1 cycle
        BR        N:?L_VSWITCH_L10   ;; 3 cycles
        ; ------------------------------------- Block: 9 cycles
// 1849     {
// 1850     case 0x015e5b00:
// 1851     case 0x015e5b03:
// 1852     case 0x015e5b04:
// 1853     case 0x015e5b05:
// 1854     case 0x015e5b06:
// 1855     case 0x015e5b07:
// 1856     case 0x01630100:
// 1857     case 0x01630200:
// 1858     case 0x01620100:
// 1859     case 0x00636200:
// 1860     case 0x00636201:
// 1861     case 0x00636202:
// 1862     case 0x00636203:
// 1863     case 0x00636204:
// 1864     case 0x00636205:
// 1865     case 0x00636263:
// 1866     case 0x005e5b0a:
// 1867       enum_d2(0x01); /* 00 */
??get_resp_189:
        MOV       A, #0x1            ;; 1 cycle
          CFI FunCall _enum_d2
        CALL      _enum_d2           ;; 3 cycles
// 1868       break;
        BR        S:??compartment3_161  ;; 3 cycles
          CFI FunCall _fill_0b
        ; ------------------------------------- Block: 7 cycles
// 1869     default:
// 1870       fill_0b();
??get_resp_190:
        CALL      _fill_0b           ;; 3 cycles
// 1871       break;
        ; ------------------------------------- Block: 3 cycles
// 1872     }
// 1873     break;
??compartment3_161:
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
// 1874   case 0x0706:
// 1875     switch(obis1)
??compartment3_104:
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, #LWRD(??get_resp_10)  ;; 1 cycle
        MOV       ES, #BYTE3(??get_resp_10)  ;; 1 cycle
        MOV       CS, #BYTE3(_get_resp)  ;; 1 cycle
        BR        N:?L_VSWITCH_L10   ;; 3 cycles
          CFI FunCall _sort_object
        ; ------------------------------------- Block: 9 cycles
// 1876     {
// 1877     case 0x015e5b00:
// 1878     case 0x015e5b03:
// 1879     case 0x015e5b04:
// 1880     case 0x015e5b05:
// 1881     case 0x015e5b06:
// 1882     case 0x015e5b07:
// 1883     case 0x01620100:
// 1884     case 0x00636200:
// 1885     case 0x00636201:
// 1886     case 0x00636202:
// 1887     case 0x00636203:
// 1888     case 0x00636204:
// 1889     case 0x00636205:
// 1890     case 0x00636263:
// 1891     case 0x005e5b0a:
// 1892       sort_object();
??get_resp_191:
        CALL      _sort_object       ;; 3 cycles
// 1893       break;
        BR        S:??compartment3_162  ;; 3 cycles
          CFI FunCall _sort_object1
        ; ------------------------------------- Block: 6 cycles
// 1894     case 0x01630100:
// 1895     case 0x01630200:
// 1896       sort_object1();
??get_resp_192:
        CALL      _sort_object1      ;; 3 cycles
// 1897       break;
        BR        S:??compartment3_162  ;; 3 cycles
          CFI FunCall _fill_0b
        ; ------------------------------------- Block: 6 cycles
// 1898     default:
// 1899       fill_0b();
??get_resp_193:
        CALL      _fill_0b           ;; 3 cycles
// 1900       break;
        ; ------------------------------------- Block: 3 cycles
// 1901     }
// 1902     break;
??compartment3_162:
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
// 1903   case 0x0707:
// 1904     switch(obis1)
??compartment3_105:
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, #LWRD(??get_resp_11)  ;; 1 cycle
        MOV       ES, #BYTE3(??get_resp_11)  ;; 1 cycle
        MOV       CS, #BYTE3(_get_resp)  ;; 1 cycle
        BR        N:?L_VSWITCH_L10   ;; 3 cycles
        ; ------------------------------------- Block: 9 cycles
// 1905     {
// 1906     case 0x015e5b00:
// 1907     case 0x015e5b03:
// 1908     case 0x015e5b04:
// 1909     case 0x015e5b05:
// 1910     case 0x015e5b06:
// 1911     case 0x015e5b07:
// 1912     case 0x005e5b0a:
// 1913       val_4byt2(0x00, 0x00, 0x00, 1);
??get_resp_194:
        MOV       B, #0x1            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
// 1914       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 10 cycles
// 1915       
// 1916     case 0x01630100:
// 1917       if(lsro_flag == 1)
??get_resp_195:
        CMP       N:_lsro_flag, #0x1  ;; 1 cycle
        BNZ       ??compartment3_163  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 1918       {
// 1919         val_4byt2(0x00, 0x00, MAX_LS / 0x100, MAX_LS % 0x100);
        MOV       B, #0x40           ;; 1 cycle
        MOV       C, #0xE            ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
        BR        S:??compartment3_164  ;; 3 cycles
        ; ------------------------------------- Block: 10 cycles
// 1920       }
// 1921       else
// 1922       {
// 1923         ls_count_dlms= load_survey_cnt + 1;
??compartment3_163:
        MOVW      AX, N:_load_survey_cnt  ;; 1 cycle
        INCW      AX                 ;; 1 cycle
        MOVW      N:_ls_count_dlms, AX  ;; 1 cycle
// 1924         val_4byt2(0, 0, ls_count_dlms / 0x100, ls_count_dlms % 0x100);
        MOVW      AX, N:_ls_count_dlms  ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      AX, N:_ls_count_dlms  ;; 1 cycle
        CLRB      X                  ;; 1 cycle
        MOV       C, A               ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
        ; ------------------------------------- Block: 16 cycles
// 1925       }
// 1926       break;
??compartment3_164:
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
// 1927       
// 1928     case 0x01630200:
// 1929       if(midnight_roll_f == 1)
??get_resp_196:
        CMP       N:_midnight_roll_f, #0x1  ;; 1 cycle
        BNZ       ??compartment3_165  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 1930       {
// 1931         val_4byt2(0x00, 0x00, 0x00, max_midnight_cnt);
        MOV       B, #0x4C           ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
        BR        S:??compartment3_166  ;; 3 cycles
        ; ------------------------------------- Block: 10 cycles
// 1932       }
// 1933       else
// 1934       {
// 1935         val_4byt2(0, 0, 0x00, midnight_par_cnt);
??compartment3_165:
        MOV       B, N:_midnight_par_cnt  ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 1936       }
// 1937       break;
??compartment3_166:
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
// 1938       ////////            
// 1939     case 0x01620100:
// 1940       if(md_reset_count >= MAX_BILL)
??get_resp_197:
        MOV       A, N:_md_reset_count  ;; 1 cycle
        CMP       A, N:_MAX_BILL     ;; 1 cycle
        BC        ??compartment3_167  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 1941       {
// 1942         val_4byt2(0x00, 0x00, 0x00, MAX_BILL + 1);
        MOV       B, N:_MAX_BILL     ;; 1 cycle
        INC       B                  ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
        BR        S:??compartment3_168  ;; 3 cycles
        ; ------------------------------------- Block: 11 cycles
// 1943       }
// 1944       else
// 1945       {
// 1946         val_4byt2(0x00, 0x00, 0x00, md_reset_count + 1);
??compartment3_167:
        MOV       B, N:_md_reset_count  ;; 1 cycle
        INC       B                  ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
        ; ------------------------------------- Block: 8 cycles
// 1947       }
// 1948       break;
??compartment3_168:
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
// 1949       ////////            
// 1950     case 0x00636200:
// 1951       if(tpr.vol_related_overflow == 1)
??get_resp_198:
        CMP       N:_tpr+1, #0x1     ;; 1 cycle
        BNZ       ??compartment3_169  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 1952       {
// 1953         val_4byt2(0, 0, 0, tpr.vol_related_entries);
        MOV       B, N:_tpr+2        ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
        BR        S:??compartment3_170  ;; 3 cycles
        ; ------------------------------------- Block: 10 cycles
// 1954       }
// 1955       else
// 1956       {
// 1957         val_4byt2(0, 0, 0, tpr.vol_related_count);
??compartment3_169:
        MOV       B, N:_tpr          ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 1958       }
// 1959       break;
??compartment3_170:
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
// 1960       
// 1961     case 0x00636201:
// 1962       if(tpr.curr_related_overflow == 1)
??get_resp_199:
        CMP       N:_tpr+5, #0x1     ;; 1 cycle
        BNZ       ??compartment3_171  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 1963       {
// 1964         val_4byt2(0, 0, 0, tpr.curr_related_entries);
        MOV       B, N:_tpr+6        ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
        BR        S:??compartment3_172  ;; 3 cycles
        ; ------------------------------------- Block: 10 cycles
// 1965       }
// 1966       else
// 1967       {
// 1968         val_4byt2(0, 0, 0, tpr.curr_related_count);
??compartment3_171:
        MOV       B, N:_tpr+4        ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 1969       }
// 1970       break;
??compartment3_172:
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
// 1971       
// 1972     case 0x00636202:
// 1973       if(tpr.power_overflow == 1)
??get_resp_200:
        CMP       N:_tpr+9, #0x1     ;; 1 cycle
        BNZ       ??compartment3_173  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 1974       {
// 1975         val_4byt2(0, 0, (tpr.power_entries * 2) / 256, (tpr.power_entries * 2) % 256);
        MOVW      BC, #0x100         ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+48
        POP       HL                 ;; 1 cycle
          CFI CFA SP+46
        MOV       X, N:_tpr+10       ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        PUSH      HL                 ;; 1 cycle
          CFI CFA SP+48
        POP       BC                 ;; 1 cycle
          CFI CFA SP+46
          CFI FunCall ?SI_MOD_L02
        CALL      N:?SI_MOD_L02      ;; 3 cycles
        MOVW      HL, AX             ;; 1 cycle
        XCH       A, L               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, L               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        MOVW      BC, #0x100         ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+48
        POP       HL                 ;; 1 cycle
          CFI CFA SP+46
        MOV       X, N:_tpr+10       ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        PUSH      HL                 ;; 1 cycle
          CFI CFA SP+48
        POP       BC                 ;; 1 cycle
          CFI CFA SP+46
          CFI FunCall ?SI_DIV_L02
        CALL      N:?SI_DIV_L02      ;; 3 cycles
        XCH       A, X               ;; 1 cycle
        MOV       C, A               ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
        BR        S:??compartment3_174  ;; 3 cycles
        ; ------------------------------------- Block: 47 cycles
// 1976       }
// 1977       else
// 1978       {
// 1979         val_4byt2(0, 0, (tpr.power_count * 2) / 256, (tpr.power_count * 2) % 256);
??compartment3_173:
        MOVW      BC, #0x100         ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+48
        POP       HL                 ;; 1 cycle
          CFI CFA SP+46
        MOV       X, N:_tpr+8        ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        PUSH      HL                 ;; 1 cycle
          CFI CFA SP+48
        POP       BC                 ;; 1 cycle
          CFI CFA SP+46
          CFI FunCall ?SI_MOD_L02
        CALL      N:?SI_MOD_L02      ;; 3 cycles
        MOVW      HL, AX             ;; 1 cycle
        XCH       A, L               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, L               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        MOVW      BC, #0x100         ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+48
        POP       HL                 ;; 1 cycle
          CFI CFA SP+46
        MOV       X, N:_tpr+8        ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        PUSH      HL                 ;; 1 cycle
          CFI CFA SP+48
        POP       BC                 ;; 1 cycle
          CFI CFA SP+46
          CFI FunCall ?SI_DIV_L02
        CALL      N:?SI_DIV_L02      ;; 3 cycles
        XCH       A, X               ;; 1 cycle
        MOV       C, A               ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
        ; ------------------------------------- Block: 44 cycles
// 1980       }
// 1981       break;
??compartment3_174:
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
// 1982       
// 1983     case 0x00636203:
// 1984       if(tpr.transaction_overflow == 1)
??get_resp_201:
        CMP       N:_tpr+13, #0x1    ;; 1 cycle
        BNZ       ??compartment3_175  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 1985       {
// 1986         val_4byt2(0, 0, 0, tpr.transaction_entries);
        MOV       B, N:_tpr+14       ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
        BR        S:??compartment3_176  ;; 3 cycles
        ; ------------------------------------- Block: 10 cycles
// 1987       }
// 1988       else
// 1989       {
// 1990         val_4byt2(0, 0, 0, tpr.transaction_count);
??compartment3_175:
        MOV       B, N:_tpr+12       ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 1991       }
// 1992       break;
??compartment3_176:
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
// 1993       
// 1994       
// 1995     case 0x00636204:
// 1996       if(tpr.others_overflow == 1)
??get_resp_202:
        CMP       N:_tpr+17, #0x1    ;; 1 cycle
        BNZ       ??compartment3_177  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 1997       {
// 1998         val_4byt2(0, 0, 0, tpr.others_entries);
        MOV       B, N:_tpr+18       ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
        BR        S:??compartment3_178  ;; 3 cycles
        ; ------------------------------------- Block: 10 cycles
// 1999       }
// 2000       else
// 2001       {
// 2002         val_4byt2(0, 0, 0, tpr.others_count);
??compartment3_177:
        MOV       B, N:_tpr+16       ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
// 2003       }
// 2004       break;
        BR        S:??compartment3_178  ;; 3 cycles
        ; ------------------------------------- Block: 10 cycles
// 2005       
// 2006     case 0x00636205:
// 2007       if(TOP_RESTORE_REQ == 1)
??get_resp_203:
        CMP       N:_TOP_RESTORE_REQ, #0x1  ;; 1 cycle
        BNZ       ??compartment3_179  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 2008       {
// 2009         if(tpr.non_roll_overflow == 1)
        CMP       N:_tpr+21, #0x1    ;; 1 cycle
        BNZ       ??compartment3_180  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 2010         {
// 2011           val_4byt2(0, 0, 0, tpr.non_roll_entries);
        MOV       B, N:_tpr+22       ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
        BR        S:??compartment3_178  ;; 3 cycles
        ; ------------------------------------- Block: 10 cycles
// 2012         }
// 2013         else
// 2014         {
// 2015           val_4byt2(0, 0, 0, tpr.non_roll_count);
??compartment3_180:
        MOV       B, N:_tpr+20       ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
        BR        S:??compartment3_178  ;; 3 cycles
        ; ------------------------------------- Block: 10 cycles
// 2016         }
// 2017       }
// 2018       else
// 2019       {
// 2020         if(bitIsSet(tpr.top_cover.flag,event_f))
??compartment3_179:
        MOVW      HL, #LWRD(_tpr+592)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??compartment3_181  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 2021         {
// 2022           val_4byt2(0, 0, 0, 1);
        MOV       B, #0x1            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
        BR        S:??compartment3_178  ;; 3 cycles
        ; ------------------------------------- Block: 10 cycles
// 2023         }
// 2024         else
// 2025         {
// 2026           val_4byt2(0, 0, 0, 0);
??compartment3_181:
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
// 2027         }
// 2028       }
// 2029       break;
        BR        S:??compartment3_178  ;; 3 cycles
        ; ------------------------------------- Block: 10 cycles
// 2030     case 0x00636263:
// 2031       if(tpr.debug_overflow == 1)
??get_resp_204:
        CMP       N:_tpr+25, #0x1    ;; 1 cycle
        BNZ       ??compartment3_182  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 2032       {
// 2033         val_4byt2(0, 0, 0, tpr.debug_entries);
        MOV       B, N:_tpr+26       ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
        BR        S:??compartment3_178  ;; 3 cycles
        ; ------------------------------------- Block: 10 cycles
// 2034       }
// 2035       else
// 2036       {
// 2037         val_4byt2(0, 0, 0, tpr.debug_count);
??compartment3_182:
        MOV       B, N:_tpr+24       ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
// 2038       }
// 2039       break;  
        BR        S:??compartment3_178  ;; 3 cycles
          CFI FunCall _fill_0b
        ; ------------------------------------- Block: 10 cycles
// 2040     default:
// 2041       fill_0b();
??get_resp_205:
        CALL      _fill_0b           ;; 3 cycles
// 2042       break;
        ; ------------------------------------- Block: 3 cycles
// 2043     }
// 2044     break;
??compartment3_178:
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
// 2045   case 0x0708:
// 2046     switch(obis1)
??compartment3_106:
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        MOVW      HL, #LWRD(??get_resp_12)  ;; 1 cycle
        MOV       ES, #BYTE3(??get_resp_12)  ;; 1 cycle
        MOV       CS, #BYTE3(_get_resp)  ;; 1 cycle
        BR        N:?L_VSWITCH_L10   ;; 3 cycles
        ; ------------------------------------- Block: 9 cycles
// 2047     {
// 2048     case 0x015e5b00:
// 2049     case 0x015e5b03:
// 2050     case 0x015e5b04:
// 2051     case 0x015e5b05:
// 2052     case 0x015e5b06:
// 2053     case 0x015e5b07:
// 2054     case 0x005e5b0a:
// 2055       val_4byt2(0x00, 0x00, 0x00, 0x01);
??get_resp_206:
        MOV       B, #0x1            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
// 2056       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 10 cycles
// 2057       
// 2058     case 0x01630100:
// 2059       val_4byt2(0x00, 0x00, MAX_LS / 0x100, MAX_LS % 0x100);
??get_resp_207:
        MOV       B, #0x40           ;; 1 cycle
        MOV       C, #0xE            ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
// 2060       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 10 cycles
// 2061       
// 2062     case 0x01630200:
// 2063       val_4byt2(0x00, 0x00, 0, max_midnight_cnt);
??get_resp_208:
        MOV       B, #0x4C           ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
// 2064       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 10 cycles
// 2065       
// 2066     case 0x01620100:
// 2067       val_4byt2(0x00, 0x00, 0x00, MAX_BILL + 1);
??get_resp_209:
        MOV       B, N:_MAX_BILL     ;; 1 cycle
        INC       B                  ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
// 2068       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 11 cycles
// 2069       
// 2070     case 0x00636200:
// 2071       val_4byt2(0, 0, 0, COMPART_VOLTAGE_ENTRIES);
??get_resp_210:
        MOV       B, N:_COMPART_VOLTAGE_ENTRIES  ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
// 2072       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 10 cycles
// 2073       
// 2074     case 0x00636201:
// 2075       val_4byt2(0, 0, 0, COMPART_CURRENT_ENTRIES);
??get_resp_211:
        MOV       B, N:_COMPART_CURRENT_ENTRIES  ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
// 2076       break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 10 cycles
// 2077       
// 2078     case 0x00636202:
// 2079       val_4byt2(0, 0, (COMPART_POWERFAIL_ENTRIES * 2) / 256, (COMPART_POWERFAIL_ENTRIES * 2) % 256);
??get_resp_212:
        MOVW      BC, #0x100         ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+48
        POP       HL                 ;; 1 cycle
          CFI CFA SP+46
        MOV       X, N:_COMPART_POWERFAIL_ENTRIES  ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        PUSH      HL                 ;; 1 cycle
          CFI CFA SP+48
        POP       BC                 ;; 1 cycle
          CFI CFA SP+46
          CFI FunCall ?SI_MOD_L02
        CALL      N:?SI_MOD_L02      ;; 3 cycles
        MOVW      HL, AX             ;; 1 cycle
        XCH       A, L               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, L               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        MOVW      BC, #0x100         ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+48
        POP       HL                 ;; 1 cycle
          CFI CFA SP+46
        MOV       X, N:_COMPART_POWERFAIL_ENTRIES  ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        PUSH      HL                 ;; 1 cycle
          CFI CFA SP+48
        POP       BC                 ;; 1 cycle
          CFI CFA SP+46
          CFI FunCall ?SI_DIV_L02
        CALL      N:?SI_DIV_L02      ;; 3 cycles
        XCH       A, X               ;; 1 cycle
        MOV       C, A               ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
// 2080       break;
        BR        S:??compartment3_183  ;; 3 cycles
        ; ------------------------------------- Block: 47 cycles
// 2081       
// 2082     case 0x00636203:
// 2083       val_4byt2(0, 0, 0, COMPART_TRANSACTION_ENTRIES);
??get_resp_213:
        MOV       B, N:_COMPART_TRANSACTION_ENTRIES  ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
// 2084       break;
        BR        S:??compartment3_183  ;; 3 cycles
        ; ------------------------------------- Block: 10 cycles
// 2085       
// 2086     case 0x00636204:
// 2087       val_4byt2(0, 0, 0, COMPART_OTHERS_ENTRIES);
??get_resp_214:
        MOV       B, N:_COMPART_OTHERS_ENTRIES  ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
// 2088       break;
        BR        S:??compartment3_183  ;; 3 cycles
        ; ------------------------------------- Block: 10 cycles
// 2089       
// 2090     case 0x00636205:
// 2091       if(TOP_RESTORE_REQ == 1)
??get_resp_215:
        CMP       N:_TOP_RESTORE_REQ, #0x1  ;; 1 cycle
        BNZ       ??compartment3_184  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 2092       {
// 2093         val_4byt2(0, 0, 0, COMPART_NONROLLOVER_ENTRIES);
        MOV       B, N:_COMPART_NONROLLOVER_ENTRIES  ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
        BR        S:??compartment3_183  ;; 3 cycles
        ; ------------------------------------- Block: 10 cycles
// 2094       }
// 2095       else
// 2096       {
// 2097         val_4byt2(0, 0, 0, 1);
??compartment3_184:
        MOV       B, #0x1            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
// 2098       }
// 2099       break;
        BR        S:??compartment3_183  ;; 3 cycles
        ; ------------------------------------- Block: 10 cycles
// 2100     case 0x00636263:
// 2101       val_4byt2(0, 0, 0, COMPART_DEBUG_ENTRIES);
??get_resp_216:
        MOV       B, #0x10           ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_4byt2
        CALL      _val_4byt2         ;; 3 cycles
// 2102       break;
        BR        S:??compartment3_183  ;; 3 cycles
          CFI FunCall _fill_0b
        ; ------------------------------------- Block: 10 cycles
// 2103       
// 2104     default:
// 2105       fill_0b();
??get_resp_217:
        CALL      _fill_0b           ;; 3 cycles
// 2106       break;
        ; ------------------------------------- Block: 3 cycles
// 2107     }
// 2108     break;
??compartment3_183:
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
// 2109     
// 2110   case 0x0802:
// 2111     if(0x00010000==obis1)
??compartment3_107:
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x1           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x0           ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??get_resp_218:
        BNZ       ??compartment3_185  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
// 2112     {
// 2113       date_time(Now.day, Now.month, Now.year, Now.hour, Now.min, Now.sec, 0x03);
        MOV       X, #0x3            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+48
        MOV       A, N:_Now          ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, N:_Now+1        ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       B, N:_Now+2        ;; 1 cycle
        MOV       C, N:_Now+6        ;; 1 cycle
        MOV       X, N:_Now+5        ;; 1 cycle
        MOV       A, N:_Now+3        ;; 1 cycle
          CFI FunCall _date_time
        CALL      _date_time         ;; 3 cycles
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+46
        BR        S:??compartment3_186  ;; 3 cycles
          CFI FunCall _fill_0b
        ; ------------------------------------- Block: 17 cycles
// 2114     }
// 2115     else
// 2116     {
// 2117       fill_0b();
??compartment3_185:
        CALL      _fill_0b           ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
// 2118     }
// 2119     break;
??compartment3_186:
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
// 2120     
// 2121   case 0x0803:
// 2122     if(0x00010000==obis1)
??compartment3_108:
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x1           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x0           ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??get_resp_219:
        BNZ       ??compartment3_187  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
// 2123     {
// 2124       info[k++]= 0x00;
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
// 2125       signed_integer(0x014a);
        MOVW      AX, #0x14A         ;; 1 cycle
          CFI FunCall _signed_integer
        CALL      _signed_integer    ;; 3 cycles
        BR        S:??compartment3_188  ;; 3 cycles
          CFI FunCall _fill_0b
        ; ------------------------------------- Block: 12 cycles
// 2126     }
// 2127     else
// 2128     {
// 2129       fill_0b();
??compartment3_187:
        CALL      _fill_0b           ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
// 2130     }
// 2131     break;
??compartment3_188:
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
// 2132     
// 2133   case 0x0804:
// 2134     if(0x00010000==obis1)
??compartment3_109:
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x1           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x0           ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??get_resp_220:
        BNZ       ??compartment3_189  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
// 2135     {
// 2136       unsigned8(0x00);
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _unsigned8
        CALL      _unsigned8         ;; 3 cycles
        BR        S:??compartment3_190  ;; 3 cycles
          CFI FunCall _fill_0b
        ; ------------------------------------- Block: 7 cycles
// 2137     }
// 2138     else
// 2139     {
// 2140       fill_0b();
??compartment3_189:
        CALL      _fill_0b           ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
// 2141     }
// 2142     break;
??compartment3_190:
        BR        N:??compartment3_134  ;; 3 cycles
          CFI FunCall _fill_0d
        ; ------------------------------------- Block: 3 cycles
// 2143     
// 2144   case 0x0805:
// 2145   case 0x0806:
// 2146   case 0x0807:
// 2147   case 0x0808:
// 2148     fill_0d();
??compartment3_110:
        CALL      _fill_0d           ;; 3 cycles
// 2149     break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
// 2150     
// 2151   case 0x0809:
// 2152     if(0x00010000==obis1)
??compartment3_111:
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x1           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x0           ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??get_resp_221:
        BNZ       ??compartment3_191  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
// 2153     {
// 2154       enum_d2(0x01);
        MOV       A, #0x1            ;; 1 cycle
          CFI FunCall _enum_d2
        CALL      _enum_d2           ;; 3 cycles
        BR        S:??compartment3_192  ;; 3 cycles
          CFI FunCall _fill_0b
        ; ------------------------------------- Block: 7 cycles
// 2155     }
// 2156     else
// 2157     {
// 2158       fill_0b();
??compartment3_191:
        CALL      _fill_0b           ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
// 2159     }
// 2160     break;
??compartment3_192:
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
// 2161     
// 2162   case 0x0f02:
// 2163   case 0x0f03:
// 2164   case 0x0f04:
// 2165   case 0x0f05:
// 2166   case 0x0f06:
// 2167   case 0x0f07:
// 2168   case 0x0f08:
// 2169   case 0x0f09:
// 2170     if((obis_code[4] == 3) && (asso2_flag != 1))
??compartment3_113:
        CMP       N:_obis_code+4, #0x3  ;; 1 cycle
        BNZ       ??compartment3_193  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        CMP       N:_asso2_flag, #0x1  ;; 1 cycle
        BZ        ??compartment3_193  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 2171     {
// 2172       ident= 0;
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP+0x09], A       ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
// 2173     }
// 2174     
// 2175     if((obis_code[4] == 2) && (asso1_flag != 1))
??compartment3_193:
        CMP       N:_obis_code+4, #0x2  ;; 1 cycle
        BNZ       ??compartment3_194  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        CMP       N:_asso1_flag, #0x1  ;; 1 cycle
        BZ        ??compartment3_194  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 2176     {
// 2177       ident= 0;
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP+0x09], A       ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
// 2178     }
// 2179     
// 2180     switch(obis1)
??compartment3_194:
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x28          ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKZ                          ;; 4 cycles
        BR        N:??compartment3_195  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
        CMPW      AX, #0x0           ;; 1 cycle
        BZ        ??compartment3_196  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        CMPW      AX, #0x2           ;; 1 cycle
        BZ        ??compartment3_196  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        CMPW      AX, #0x3           ;; 1 cycle
        SKZ                          ;; 4 cycles
        BR        N:??compartment3_195  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 2181     {
// 2182     case 0x00280000:
// 2183       /*		  case 0x00280001: */
// 2184     case 0x00280002:
// 2185     case 0x00280003:
// 2186       if(ident == 1)
??compartment3_196:
        MOV       A, [SP+0x09]       ;; 1 cycle
        CMP       A, #0x1            ;; 1 cycle
        SKZ                          ;; 4 cycles
        BR        N:??compartment3_197  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 2187       {
// 2188         switch(attribute_id)
        MOV       A, N:_attribute_id  ;; 1 cycle
        SUB       A, #0x2            ;; 1 cycle
        BZ        ??compartment3_198  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        DEC       A                  ;; 1 cycle
        BZ        ??compartment3_199  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        DEC       A                  ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??compartment3_200  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        DEC       A                  ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??compartment3_201  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        DEC       A                  ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??compartment3_202  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        DEC       A                  ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??compartment3_203  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        DEC       A                  ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??compartment3_204  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        DEC       A                  ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??compartment3_205  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        BR        N:??compartment3_206  ;; 3 cycles
          CFI FunCall _object_list
        ; ------------------------------------- Block: 3 cycles
// 2189         {
// 2190         case 2: /* object_list */
// 2191           /*					i_dlms=1; */
// 2192           object_list();
??compartment3_198:
        CALL      _object_list       ;; 3 cycles
// 2193           break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
// 2194           
// 2195         case 3: /* partners_id */
// 2196           info[k++]= 0;
??compartment3_199:
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
// 2197           structure(2);
        MOV       A, #0x2            ;; 1 cycle
          CFI FunCall _structure
        CALL      _structure         ;; 3 cycles
// 2198           switch(obis_code[4])
        MOV       A, N:_obis_code+4  ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        BZ        ??compartment3_207  ;; 4 cycles
        ; ------------------------------------- Block: 15 cycles
        DEC       A                  ;; 1 cycle
        BZ        ??compartment3_208  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        DEC       A                  ;; 1 cycle
        BZ        ??compartment3_209  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        DEC       A                  ;; 1 cycle
        BZ        ??compartment3_210  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        BR        S:??compartment3_211  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
// 2199           {
// 2200           case 0:
// 2201             if(asso3_flag == 1)
??compartment3_207:
        CMP       N:_asso3_flag, #0x1  ;; 1 cycle
        BNZ       ??compartment3_212  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 2202             {
// 2203               integer8(0x40);
        MOV       A, #0x40           ;; 1 cycle
          CFI FunCall _integer8
        CALL      _integer8          ;; 3 cycles
        BR        S:??compartment3_211  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 2204             }
// 2205             else if(asso2_flag == 1)
??compartment3_212:
        CMP       N:_asso2_flag, #0x1  ;; 1 cycle
        BNZ       ??compartment3_213  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 2206             {
// 2207               integer8(0x30);
        MOV       A, #0x30           ;; 1 cycle
          CFI FunCall _integer8
        CALL      _integer8          ;; 3 cycles
        BR        S:??compartment3_211  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 2208             }
// 2209             else if(asso1_flag == 1)
??compartment3_213:
        CMP       N:_asso1_flag, #0x1  ;; 1 cycle
        BNZ       ??compartment3_214  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 2210             {
// 2211               integer8(0x20);
        MOV       A, #0x20           ;; 1 cycle
          CFI FunCall _integer8
        CALL      _integer8          ;; 3 cycles
        BR        S:??compartment3_211  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 2212             }
// 2213             else
// 2214             {
// 2215               integer8(0x10);
??compartment3_214:
        MOV       A, #0x10           ;; 1 cycle
          CFI FunCall _integer8
        CALL      _integer8          ;; 3 cycles
// 2216             }
// 2217             break;
        BR        S:??compartment3_211  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 2218             
// 2219           case 1:
// 2220             integer8(0x10);
??compartment3_208:
        MOV       A, #0x10           ;; 1 cycle
          CFI FunCall _integer8
        CALL      _integer8          ;; 3 cycles
// 2221             break;
        BR        S:??compartment3_211  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 2222             
// 2223           case 2:
// 2224             integer8(0x20);
??compartment3_209:
        MOV       A, #0x20           ;; 1 cycle
          CFI FunCall _integer8
        CALL      _integer8          ;; 3 cycles
// 2225             break;
        BR        S:??compartment3_211  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 2226             
// 2227           case 3:
// 2228             integer8(0x30);
??compartment3_210:
        MOV       A, #0x30           ;; 1 cycle
          CFI FunCall _integer8
        CALL      _integer8          ;; 3 cycles
// 2229             break;
        ; ------------------------------------- Block: 4 cycles
// 2230             
// 2231           default:
// 2232             break;
// 2233           }
// 2234           val_2byt(0x00, 0x01);
??compartment3_211:
        MOV       X, #0x1            ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_2byt
        CALL      _val_2byt          ;; 3 cycles
// 2235           break;
        BR        N:??compartment3_134  ;; 3 cycles
          CFI FunCall _app_con
        ; ------------------------------------- Block: 8 cycles
// 2236           
// 2237         case 4: /* application_context_name */
// 2238           app_con();
??compartment3_200:
        CALL      _app_con           ;; 3 cycles
// 2239           break;
        BR        N:??compartment3_134  ;; 3 cycles
          CFI FunCall _xdlms_type
        ; ------------------------------------- Block: 6 cycles
// 2240           
// 2241         case 5: /* xDLMS_context_info */
// 2242           xdlms_type();
??compartment3_201:
        CALL      _xdlms_type        ;; 3 cycles
// 2243           break;
        BR        N:??compartment3_134  ;; 3 cycles
          CFI FunCall _auth_name
        ; ------------------------------------- Block: 6 cycles
// 2244           
// 2245         case 6: /* authentication mech_name */
// 2246           auth_name();
??compartment3_202:
        CALL      _auth_name         ;; 3 cycles
// 2247           break;
        BR        N:??compartment3_134  ;; 3 cycles
          CFI FunCall _fill_0d
        ; ------------------------------------- Block: 6 cycles
// 2248           
// 2249         case 7:
// 2250           fill_0d();
??compartment3_203:
        CALL      _fill_0d           ;; 3 cycles
// 2251           break;
        BR        N:??compartment3_134  ;; 3 cycles
          CFI FunCall _asso_status
        ; ------------------------------------- Block: 6 cycles
// 2252           
// 2253         case 8: /* association status */
// 2254           asso_status();
??compartment3_204:
        CALL      _asso_status       ;; 3 cycles
// 2255           break;
        BR        S:??compartment3_215  ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
// 2256           
// 2257         case 9: /* security_setup_reference */
// 2258           octet_s(6,1);
??compartment3_205:
        MOV       X, #0x1            ;; 1 cycle
        MOV       A, #0x6            ;; 1 cycle
          CFI FunCall _octet_s
        CALL      _octet_s           ;; 3 cycles
// 2259           info[k++]=0x00;
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
// 2260           info[k++]=0x00;
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
// 2261           info[k++]=0x2B;
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, #0x2B           ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
// 2262           info[k++]=0x00;
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
// 2263           if(asso1_flag==1)
        CMP       N:_asso1_flag, #0x1  ;; 1 cycle
        BNZ       ??compartment3_216  ;; 4 cycles
        ; ------------------------------------- Block: 30 cycles
// 2264           {
// 2265             info[k++]=0x02;
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, #0x2            ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
        BR        S:??compartment3_217  ;; 3 cycles
        ; ------------------------------------- Block: 8 cycles
// 2266           }
// 2267           else if(asso2_flag==1)
??compartment3_216:
        CMP       N:_asso2_flag, #0x1  ;; 1 cycle
        BNZ       ??compartment3_217  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 2268           {
// 2269             info[k++]=0x03;
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, #0x3            ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
        ; ------------------------------------- Block: 5 cycles
// 2270           }
// 2271           
// 2272           info[k++]=0xFF;
??compartment3_217:
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, #0xFF           ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
// 2273           break;
        BR        S:??compartment3_218  ;; 3 cycles
          CFI FunCall _fill_0b
        ; ------------------------------------- Block: 8 cycles
// 2274           
// 2275         default:
// 2276           fill_0b();
??compartment3_206:
        CALL      _fill_0b           ;; 3 cycles
// 2277           break;
        BR        S:??compartment3_218  ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
// 2278         }
// 2279       }
// 2280       else
// 2281       {
// 2282         if((attribute_id < 9) && (attribute_id > 0))
??compartment3_197:
        MOV       A, N:_attribute_id  ;; 1 cycle
        CMP       A, #0x9            ;; 1 cycle
        BNC       ??compartment3_219  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        CMP0      N:_attribute_id    ;; 1 cycle
        BZ        ??compartment3_219  ;; 4 cycles
          CFI FunCall _fill_0d
        ; ------------------------------------- Block: 5 cycles
// 2283         {
// 2284           fill_0d();
        CALL      _fill_0d           ;; 3 cycles
        BR        S:??compartment3_218  ;; 3 cycles
          CFI FunCall _fill_0b
        ; ------------------------------------- Block: 6 cycles
// 2285         }
// 2286         else
// 2287         {
// 2288           fill_0b();
??compartment3_219:
        CALL      _fill_0b           ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
// 2289         }
// 2290       }
// 2291       break;
??compartment3_215:
        BR        S:??compartment3_218  ;; 3 cycles
          CFI FunCall _fill_0b
        ; ------------------------------------- Block: 3 cycles
// 2292       
// 2293     default:
// 2294       fill_0b();
??compartment3_195:
        CALL      _fill_0b           ;; 3 cycles
// 2295       break;
        ; ------------------------------------- Block: 3 cycles
// 2296       
// 2297     }
// 2298     break;
??compartment3_218:
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
// 2299     
// 2300   case 0x1102:
// 2301     switch(obis1)
??compartment3_114:
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x29          ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x0           ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??get_resp_222:
        BNZ       ??compartment3_220  ;; 4 cycles
          CFI FunCall _sap_assg_list
        ; ------------------------------------- Block: 4 cycles
// 2302     {
// 2303     case 0x00290000:    /* SAP association list */
// 2304       sap_assg_list();
        CALL      _sap_assg_list     ;; 3 cycles
// 2305       break;
        BR        S:??compartment3_221  ;; 3 cycles
          CFI FunCall _fill_0b
        ; ------------------------------------- Block: 6 cycles
// 2306       
// 2307     default:
// 2308       fill_0b();
??compartment3_220:
        CALL      _fill_0b           ;; 3 cycles
// 2309       break;
        ; ------------------------------------- Block: 3 cycles
// 2310     }
// 2311     break;
??compartment3_221:
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
// 2312   case 0x1402:
// 2313   case 0x1406:
// 2314     switch(obis1)
??compartment3_115:
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0xD           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x0           ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??get_resp_223:
        SKZ                          ;; 4 cycles
        BR        N:??compartment3_222  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
// 2315     {
// 2316     case 0x000D0000:
// 2317       if(classatt == 0x1402)
        MOVW      AX, [SP+0x12]      ;; 1 cycle
        CMPW      AX, #0x1402        ;; 1 cycle
        BNZ       ??compartment3_223  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 2318       {
// 2319         if(active_calendar == 0)
        CMP0      N:_active_calendar  ;; 1 cycle
        BNZ       ??compartment3_224  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 2320         {
// 2321           temp_us32= (TOU_CAL_ACTIVE_ADD);
        MOVW      AX, #0x1400        ;; 1 cycle
        MOVW      S:_temp_us32, AX   ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      S:_temp_us32+2, AX  ;; 1 cycle
// 2322           eprom_read(temp_us32,0,PAGE_1,AUTO_CALC);
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        PUSH      DE                 ;; 1 cycle
          CFI CFA SP+48
        MOVW      HL, S:_temp_us32+2  ;; 1 cycle
        MOVW      DE, S:_temp_us32   ;; 1 cycle
        MOVW      AX, DE             ;; 1 cycle
        XCH       A, E               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+46
        XCH       A, E               ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
        BR        S:??compartment3_225  ;; 3 cycles
        ; ------------------------------------- Block: 21 cycles
// 2323         }
// 2324         else if(active_calendar == 1)
??compartment3_224:
        CMP       N:_active_calendar, #0x1  ;; 1 cycle
        BNZ       ??compartment3_225  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 2325         {
// 2326           temp_us32= (TOU_CAL_PASSIVE_ADD);
        MOVW      AX, #0x1440        ;; 1 cycle
        MOVW      S:_temp_us32, AX   ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      S:_temp_us32+2, AX  ;; 1 cycle
// 2327           eprom_read(temp_us32,0,PAGE_1,AUTO_CALC);
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        PUSH      DE                 ;; 1 cycle
          CFI CFA SP+48
        MOVW      HL, S:_temp_us32+2  ;; 1 cycle
        MOVW      DE, S:_temp_us32   ;; 1 cycle
        MOVW      AX, DE             ;; 1 cycle
        XCH       A, E               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+46
        XCH       A, E               ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
        BR        S:??compartment3_225  ;; 3 cycles
        ; ------------------------------------- Block: 21 cycles
// 2328         }
// 2329       }
// 2330       else if(classatt == 0x1406)
??compartment3_223:
        MOVW      AX, [SP+0x12]      ;; 1 cycle
        CMPW      AX, #0x1406        ;; 1 cycle
        BNZ       ??compartment3_225  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 2331       {
// 2332         if(active_calendar == 0)
        CMP0      N:_active_calendar  ;; 1 cycle
        BNZ       ??compartment3_226  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 2333         {
// 2334           temp_us32= (TOU_CAL_PASSIVE_ADD);
        MOVW      AX, #0x1440        ;; 1 cycle
        MOVW      S:_temp_us32, AX   ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      S:_temp_us32+2, AX  ;; 1 cycle
// 2335           eprom_read(temp_us32,0,PAGE_1,AUTO_CALC);
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        PUSH      DE                 ;; 1 cycle
          CFI CFA SP+48
        MOVW      HL, S:_temp_us32+2  ;; 1 cycle
        MOVW      DE, S:_temp_us32   ;; 1 cycle
        MOVW      AX, DE             ;; 1 cycle
        XCH       A, E               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+46
        XCH       A, E               ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
        BR        S:??compartment3_225  ;; 3 cycles
        ; ------------------------------------- Block: 21 cycles
// 2336         }
// 2337         else if(active_calendar == 1)
??compartment3_226:
        CMP       N:_active_calendar, #0x1  ;; 1 cycle
        BNZ       ??compartment3_225  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 2338         {
// 2339           temp_us32= (TOU_CAL_ACTIVE_ADD);
        MOVW      AX, #0x1400        ;; 1 cycle
        MOVW      S:_temp_us32, AX   ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      S:_temp_us32+2, AX  ;; 1 cycle
// 2340           eprom_read(temp_us32,0,PAGE_1,AUTO_CALC);
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        PUSH      DE                 ;; 1 cycle
          CFI CFA SP+48
        MOVW      HL, S:_temp_us32+2  ;; 1 cycle
        MOVW      DE, S:_temp_us32   ;; 1 cycle
        MOVW      AX, DE             ;; 1 cycle
        XCH       A, E               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+46
        XCH       A, E               ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
        ; ------------------------------------- Block: 18 cycles
// 2341         }
// 2342       }
// 2343       octet_s(opr_data[0], 1);
??compartment3_225:
        MOV       X, #0x1            ;; 1 cycle
        MOV       A, N:_opr_data     ;; 1 cycle
          CFI FunCall _octet_s
        CALL      _octet_s           ;; 3 cycles
// 2344       memcpy(&info[k], &opr_data[1], opr_data[0]);
        MOV       C, N:_opr_data     ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOVW      DE, #LWRD(_opr_data+1)  ;; 1 cycle
        MOVW      AX, N:_k           ;; 1 cycle
        ADDW      AX, #LWRD(_info)   ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, BC             ;; 1 cycle
          CFI FunCall ?MEMCPY_NEAR
        CALL      N:?MEMCPY_NEAR     ;; 3 cycles
// 2345       k+= opr_data[0];
        MOV       X, N:_opr_data     ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        ADDW      AX, N:_k           ;; 1 cycle
        MOVW      N:_k, AX           ;; 1 cycle
// 2346       break;
        BR        S:??compartment3_227  ;; 3 cycles
          CFI FunCall _fill_0b
        ; ------------------------------------- Block: 22 cycles
// 2347       
// 2348     default:
// 2349       fill_0b();
??compartment3_222:
        CALL      _fill_0b           ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
// 2350     }
// 2351     break;
??compartment3_227:
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
// 2352   case 0x1403:
// 2353   case 0x1407:
// 2354     if(0x000D0000 == obis1)
??compartment3_116:
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0xD           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x0           ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??get_resp_224:
        SKZ                          ;; 4 cycles
        BR        N:??compartment3_228  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
// 2355     {
// 2356       if((0==active_calendar && 3==attribute_id)||(1==active_calendar && 7==attribute_id))
        CMP0      N:_active_calendar  ;; 1 cycle
        BNZ       ??compartment3_229  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        CMP       N:_attribute_id, #0x3  ;; 1 cycle
        BZ        ??compartment3_230  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
??compartment3_229:
        CMP       N:_active_calendar, #0x1  ;; 1 cycle
        BNZ       ??compartment3_231  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        CMP       N:_attribute_id, #0x7  ;; 1 cycle
        BNZ       ??compartment3_231  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 2357       {
// 2358         lu16_add=TOU_CAL_ACTIVE_ADD+0x10;
??compartment3_230:
        MOVW      AX, #0x1410        ;; 1 cycle
        MOVW      [SP+0x02], AX      ;; 1 cycle
        BR        S:??compartment3_232  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 2359       }
// 2360       else
// 2361       {
// 2362         lu16_add=TOU_CAL_PASSIVE_ADD+0x10;
??compartment3_231:
        MOVW      AX, #0x1450        ;; 1 cycle
        MOVW      [SP+0x02], AX      ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
// 2363       }
// 2364       eprom_read(lu16_add,0,PAGE_1,AUTO_CALC);
??compartment3_232:
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, [SP+0x02]      ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
// 2365       array(opr_data[0],1);
        MOV       X, #0x1            ;; 1 cycle
        MOV       A, N:_opr_data     ;; 1 cycle
          CFI FunCall _array
        CALL      _array             ;; 3 cycles
// 2366       lu16_add_temp=lu16_add;
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      [SP+0x16], AX      ;; 1 cycle
// 2367       lu8_k=1;
        MOV       A, #0x1            ;; 1 cycle
        MOV       [SP+0x01], A       ;; 1 cycle
// 2368       for(lu8_i=0; lu8_i<2 && 1==lu8_k;  lu8_i++)
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        ; ------------------------------------- Block: 18 cycles
??get_resp_225:
        MOV       A, [SP]            ;; 1 cycle
        CMP       A, #0x2            ;; 1 cycle
        SKC                          ;; 4 cycles
        BR        N:??compartment3_134  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOV       A, [SP+0x01]       ;; 1 cycle
        CMP       A, #0x1            ;; 1 cycle
        SKZ                          ;; 4 cycles
        BR        N:??compartment3_134  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 2369       {
// 2370         lu8_k=0;
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP+0x01], A       ;; 1 cycle
// 2371         structure(3);	
        MOV       A, #0x3            ;; 1 cycle
          CFI FunCall _structure
        CALL      _structure         ;; 3 cycles
// 2372         octet_s(7,0);
        MOV       X, #0x0            ;; 1 cycle
        MOV       A, #0x7            ;; 1 cycle
          CFI FunCall _octet_s
        CALL      _octet_s           ;; 3 cycles
// 2373         memcpy(&info[k],&opr_data[lu8_i*7+1],7);
        MOV       A, [SP]            ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x7           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(_opr_data+1)  ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, N:_k           ;; 1 cycle
        ADDW      AX, #LWRD(_info)   ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV       B, #0x7            ;; 1 cycle
          CFI FunCall ?MEMCPY_SMALL_NEAR
        CALL      N:?MEMCPY_SMALL_NEAR  ;; 3 cycles
// 2374         k+=7;
        MOVW      AX, N:_k           ;; 1 cycle
        ADDW      AX, #0x7           ;; 1 cycle
        MOVW      N:_k, AX           ;; 1 cycle
// 2375         lu16_add+=0x10;
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
        MOVW      [SP+0x02], AX      ;; 1 cycle
// 2376         eprom_read(lu16_add,0,PAGE_1,AUTO_CALC);
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, [SP+0x02]      ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
// 2377         date_time(opr_data[4+(lu8_i*5)],opr_data[3+(lu8_i*5)],opr_data[2+(lu8_i*5)],opr_data[1+(lu8_i*5)],opr_data[lu8_i*5],0x00,0);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+48
        MOV       D, #0x0            ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x5           ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      BC, AX             ;; 1 cycle
        MOV       A, (_opr_data)[BC]  ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x5           ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      BC, AX             ;; 1 cycle
        MOV       A, (_opr_data+1)[BC]  ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       H, A               ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x5           ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      BC, AX             ;; 1 cycle
        MOV       A, (_opr_data+2)[BC]  ;; 1 cycle
        MOV       C, A               ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        XCH       A, H               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        XCH       A, H               ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       L, A               ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        XCH       A, E               ;; 1 cycle
        MOV       H, A               ;; 1 cycle
        XCH       A, E               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        XCH       A, C               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        XCH       A, C               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x5           ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      BC, AX             ;; 1 cycle
        MOV       A, (_opr_data+3)[BC]  ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       C, A               ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        XCH       A, E               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, E               ;; 1 cycle
        XCH       A, H               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        XCH       A, H               ;; 1 cycle
        XCH       A, L               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        XCH       A, L               ;; 1 cycle
        PUSH      DE                 ;; 1 cycle
          CFI CFA SP+50
        XCH       A, E               ;; 1 cycle
        MOV       L, A               ;; 1 cycle
        XCH       A, E               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        MOV       H, A               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        XCH       A, C               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        XCH       A, C               ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       A, [SP+0x04]       ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x5           ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      BC, AX             ;; 1 cycle
        MOV       A, (_opr_data+4)[BC]  ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        XCH       A, E               ;; 1 cycle
        MOV       C, A               ;; 1 cycle
        XCH       A, E               ;; 1 cycle
        XCH       A, H               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, H               ;; 1 cycle
        XCH       A, L               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        XCH       A, L               ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+48
        XCH       A, D               ;; 1 cycle
          CFI FunCall _date_time
        CALL      _date_time         ;; 3 cycles
// 2378         lu16_add+=0x10;
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
        MOVW      [SP+0x04], AX      ;; 1 cycle
// 2379         eprom_read(lu16_add,0,PAGE_1,AUTO_CALC);
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
// 2380         octet_s(7,0);
        MOV       X, #0x0            ;; 1 cycle
        MOV       A, #0x7            ;; 1 cycle
          CFI FunCall _octet_s
        CALL      _octet_s           ;; 3 cycles
// 2381         memcpy(&info[k],&opr_data[(lu8_i*7)],7);
        MOV       A, [SP+0x02]       ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x7           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(_opr_data)  ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      AX, N:_k           ;; 1 cycle
        ADDW      AX, #LWRD(_info)   ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV       B, #0x7            ;; 1 cycle
          CFI FunCall ?MEMCPY_SMALL_NEAR
        CALL      N:?MEMCPY_SMALL_NEAR  ;; 3 cycles
// 2382         k+=7;
        MOVW      AX, N:_k           ;; 1 cycle
        ADDW      AX, #0x7           ;; 1 cycle
        MOVW      N:_k, AX           ;; 1 cycle
// 2383         
// 2384         if(0==lu8_i)
        MOV       A, [SP+0x02]       ;; 1 cycle
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+46
        CMP0      A                  ;; 1 cycle
        BNZ       ??compartment3_233  ;; 4 cycles
        ; ------------------------------------- Block: 196 cycles
// 2385         {
// 2386           lu16_add=lu16_add_temp;
        MOVW      AX, [SP+0x16]      ;; 1 cycle
        MOVW      [SP+0x02], AX      ;; 1 cycle
// 2387           eprom_read(lu16_add,0,PAGE_1,AUTO_CALC);
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, [SP+0x02]      ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
// 2388           if(2==opr_data[0])
        CMP       N:_opr_data, #0x2  ;; 1 cycle
        BNZ       ??compartment3_233  ;; 4 cycles
        ; ------------------------------------- Block: 14 cycles
// 2389           {
// 2390             lu8_k=1;
        MOV       A, #0x1            ;; 1 cycle
        MOV       [SP+0x01], A       ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
// 2391           }
// 2392         }
// 2393       }
??compartment3_233:
        MOV       A, [SP]            ;; 1 cycle
        INC       A                  ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        BR        N:??get_resp_225   ;; 3 cycles
          CFI FunCall _fill_0b
        ; ------------------------------------- Block: 6 cycles
// 2394     }
// 2395     else
// 2396     {
// 2397       fill_0b();
??compartment3_228:
        CALL      _fill_0b           ;; 3 cycles
// 2398     }
// 2399     break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
// 2400     
// 2401   case 0x1404:
// 2402   case 0x1408:
// 2403     if(0x000D0000 == obis1)
??compartment3_117:
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0xD           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x0           ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??get_resp_226:
        SKZ                          ;; 4 cycles
        BR        N:??compartment3_234  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
// 2404     {
// 2405       if((0==active_calendar && 4==attribute_id)||(1==active_calendar && 8==attribute_id))
        CMP0      N:_active_calendar  ;; 1 cycle
        BNZ       ??compartment3_235  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        CMP       N:_attribute_id, #0x4  ;; 1 cycle
        BZ        ??compartment3_236  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
??compartment3_235:
        CMP       N:_active_calendar, #0x1  ;; 1 cycle
        BNZ       ??compartment3_237  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        CMP       N:_attribute_id, #0x8  ;; 1 cycle
        BNZ       ??compartment3_237  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 2406       {
// 2407         lu16_add=TOU_WEEK_ACTIVE_ADD;
??compartment3_236:
        MOVW      AX, #0x1480        ;; 1 cycle
        MOVW      [SP+0x02], AX      ;; 1 cycle
        BR        S:??compartment3_238  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 2408       }
// 2409       else
// 2410       {
// 2411         lu16_add=TOU_WEEK_ACTIVE_ADD+0x40;
??compartment3_237:
        MOVW      AX, #0x14C0        ;; 1 cycle
        MOVW      [SP+0x02], AX      ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
// 2412       }
// 2413       
// 2414       eprom_read(lu16_add,0,PAGE_1,AUTO_CALC);
??compartment3_238:
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, [SP+0x02]      ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
// 2415       array(opr_data[14],1);
        MOV       X, #0x1            ;; 1 cycle
        MOV       A, N:_opr_data+14  ;; 1 cycle
          CFI FunCall _array
        CALL      _array             ;; 3 cycles
// 2416       lu8_k=1;
        MOV       A, #0x1            ;; 1 cycle
        MOV       [SP+0x01], A       ;; 1 cycle
// 2417       
// 2418       for(lu8_i=0; lu8_i<2 && 1==lu8_k;  lu8_i++)
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        ; ------------------------------------- Block: 16 cycles
??get_resp_227:
        MOV       A, [SP]            ;; 1 cycle
        CMP       A, #0x2            ;; 1 cycle
        SKC                          ;; 4 cycles
        BR        N:??compartment3_134  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOV       A, [SP+0x01]       ;; 1 cycle
        CMP       A, #0x1            ;; 1 cycle
        SKZ                          ;; 4 cycles
        BR        N:??compartment3_134  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 2419       {
// 2420         lu8_k=0;
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP+0x01], A       ;; 1 cycle
// 2421         structure(8);	
        MOV       A, #0x8            ;; 1 cycle
          CFI FunCall _structure
        CALL      _structure         ;; 3 cycles
// 2422         octet_s(7,0);
        MOV       X, #0x0            ;; 1 cycle
        MOV       A, #0x7            ;; 1 cycle
          CFI FunCall _octet_s
        CALL      _octet_s           ;; 3 cycles
// 2423         memcpy(&info[k],&opr_data[0],7);
        MOVW      DE, #LWRD(_opr_data)  ;; 1 cycle
        MOVW      AX, N:_k           ;; 1 cycle
        ADDW      AX, #LWRD(_info)   ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV       B, #0x7            ;; 1 cycle
          CFI FunCall ?MEMCPY_SMALL_NEAR
        CALL      N:?MEMCPY_SMALL_NEAR  ;; 3 cycles
// 2424         k+=7;
        MOVW      AX, N:_k           ;; 1 cycle
        ADDW      AX, #0x7           ;; 1 cycle
        MOVW      N:_k, AX           ;; 1 cycle
// 2425         lu16_add+=0x10;
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
        MOVW      [SP+0x02], AX      ;; 1 cycle
// 2426         eprom_read(lu16_add,0,PAGE_1,AUTO_CALC);
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, [SP+0x02]      ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
// 2427         
// 2428         for(lu8_j=0;  lu8_j<7;  lu8_j++)
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP+0x08], A       ;; 1 cycle
        ; ------------------------------------- Block: 34 cycles
??get_resp_228:
        MOV       A, [SP+0x08]       ;; 1 cycle
        CMP       A, #0x7            ;; 1 cycle
        BNC       ??compartment3_239  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 2429         {
// 2430           val_1byt(*(&opr_data[0]+lu8_j),0);
        MOV       X, #0x0            ;; 1 cycle
        MOV       A, [SP+0x08]       ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        MOV       A, (_opr_data)[B]  ;; 1 cycle
          CFI FunCall _val_1byt
        CALL      _val_1byt          ;; 3 cycles
// 2431         } 			
        MOV       A, [SP+0x08]       ;; 1 cycle
        INC       A                  ;; 1 cycle
        MOV       [SP+0x08], A       ;; 1 cycle
        BR        S:??get_resp_228   ;; 3 cycles
        ; ------------------------------------- Block: 13 cycles
// 2432         
// 2433         if(0==lu8_i)
??compartment3_239:
        MOV       A, [SP]            ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        BNZ       ??compartment3_240  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 2434         {
// 2435           lu16_add+=0x10;
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
        MOVW      [SP+0x02], AX      ;; 1 cycle
// 2436           eprom_read(lu16_add,0,PAGE_1,AUTO_CALC);
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, [SP+0x02]      ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
// 2437           if(2==opr_data[14])
        CMP       N:_opr_data+14, #0x2  ;; 1 cycle
        BNZ       ??compartment3_240  ;; 4 cycles
        ; ------------------------------------- Block: 15 cycles
// 2438           {
// 2439             lu8_k=1;
        MOV       A, #0x1            ;; 1 cycle
        MOV       [SP+0x01], A       ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
// 2440           }
// 2441         }
// 2442         
// 2443       }
??compartment3_240:
        MOV       A, [SP]            ;; 1 cycle
        INC       A                  ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        BR        N:??get_resp_227   ;; 3 cycles
          CFI FunCall _fill_0b
        ; ------------------------------------- Block: 6 cycles
// 2444     }
// 2445     else
// 2446     {
// 2447       fill_0b();
??compartment3_234:
        CALL      _fill_0b           ;; 3 cycles
// 2448     }
// 2449     break;
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
// 2450     
// 2451   case 0x1405:
// 2452   case 0x1409:
// 2453     switch(obis1)
??compartment3_118:
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0xD           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x0           ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??get_resp_229:
        BNZ       ??compartment3_241  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
// 2454     {
// 2455     case 0x000D0000:
// 2456       if(classatt == 0x1405)
        MOVW      AX, [SP+0x12]      ;; 1 cycle
        CMPW      AX, #0x1405        ;; 1 cycle
        BNZ       ??compartment3_242  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 2457       {
// 2458         day_profile(active_calendar);
        MOV       A, N:_active_calendar  ;; 1 cycle
          CFI FunCall _day_profile
        CALL      _day_profile       ;; 3 cycles
        BR        S:??compartment3_243  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 2459       }
// 2460       else if(classatt == 0x1409)
??compartment3_242:
        MOVW      AX, [SP+0x12]      ;; 1 cycle
        CMPW      AX, #0x1409        ;; 1 cycle
        BNZ       ??compartment3_243  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 2461       {
// 2462         if(active_calendar)
        CMP0      N:_active_calendar  ;; 1 cycle
        BZ        ??compartment3_244  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 2463           day_profile(0);
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _day_profile
        CALL      _day_profile       ;; 3 cycles
        BR        S:??compartment3_243  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 2464         else
// 2465           day_profile(1);
??compartment3_244:
        MOV       A, #0x1            ;; 1 cycle
          CFI FunCall _day_profile
        CALL      _day_profile       ;; 3 cycles
// 2466       }
// 2467       break;
        BR        S:??compartment3_243  ;; 3 cycles
          CFI FunCall _fill_0b
        ; ------------------------------------- Block: 7 cycles
// 2468     default:
// 2469       fill_0b();
??compartment3_241:
        CALL      _fill_0b           ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
// 2470     }
// 2471     break;
??compartment3_243:
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
// 2472     
// 2473   case 0x140a:
// 2474     eprom_read(TOU_PassiveApliDate,0,PAGE_1,AUTO_CALC);
??compartment3_119:
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, #0x15C0        ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
// 2475     date_time(opr_data[2], opr_data[1], opr_data[0], opr_data[3], opr_data[4], 0, 1);
        MOV       X, #0x1            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+48
        MOV       D, #0x0            ;; 1 cycle
        MOV       A, N:_opr_data+4   ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       B, N:_opr_data+3   ;; 1 cycle
        MOV       C, N:_opr_data     ;; 1 cycle
        MOV       X, N:_opr_data+1   ;; 1 cycle
        MOV       A, N:_opr_data+2   ;; 1 cycle
          CFI FunCall _date_time
        CALL      _date_time         ;; 3 cycles
// 2476     break;
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+46
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 23 cycles
// 2477     
// 2478   case 0x1602:
// 2479     switch(obis1)
??compartment3_120:
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0xF           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x0           ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??get_resp_230:
        BNZ       ??compartment3_245  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
// 2480     {
// 2481     case 0x000f0000:
// 2482       {
// 2483         info[k++]= 0;
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
// 2484         structure(0x02);
        MOV       A, #0x2            ;; 1 cycle
          CFI FunCall _structure
        CALL      _structure         ;; 3 cycles
// 2485         octet_s(6,0);
        MOV       X, #0x0            ;; 1 cycle
        MOV       A, #0x6            ;; 1 cycle
          CFI FunCall _octet_s
        CALL      _octet_s           ;; 3 cycles
// 2486         info[k++]=0;
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
// 2487         info[k++]=0;
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
// 2488         info[k++]=10;
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, #0xA            ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
// 2489         info[k++]=0;
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
// 2490         info[k++]=1;
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, #0x1            ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
// 2491         info[k++]=255;
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, #0xFF           ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
// 2492         val_2byt(0,1);
        MOV       X, #0x1            ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_2byt
        CALL      _val_2byt          ;; 3 cycles
// 2493       }
// 2494       break;
        BR        S:??compartment3_246  ;; 3 cycles
          CFI FunCall _fill_0b
        ; ------------------------------------- Block: 52 cycles
// 2495     default:
// 2496       fill_0b();
??compartment3_245:
        CALL      _fill_0b           ;; 3 cycles
// 2497       break;
        ; ------------------------------------- Block: 3 cycles
// 2498     }
// 2499     break;
??compartment3_246:
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
// 2500     
// 2501   case 0x1603:
// 2502     switch(obis1)
??compartment3_121:
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0xF           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x0           ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??get_resp_231:
        BNZ       ??compartment3_247  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
// 2503     {
// 2504     case 0x000f0000:
// 2505       enum_d2(0x01);
        MOV       A, #0x1            ;; 1 cycle
          CFI FunCall _enum_d2
        CALL      _enum_d2           ;; 3 cycles
// 2506       break;
        BR        S:??compartment3_248  ;; 3 cycles
          CFI FunCall _fill_0b
        ; ------------------------------------- Block: 7 cycles
// 2507     default:
// 2508       fill_0b();
??compartment3_247:
        CALL      _fill_0b           ;; 3 cycles
// 2509       break;
        ; ------------------------------------- Block: 3 cycles
// 2510     }
// 2511     break;
??compartment3_248:
        BR        N:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 3 cycles
// 2512     
// 2513   case 0x1604:
// 2514     switch(obis1)
??compartment3_122:
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0xF           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x0           ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??get_resp_232:
        SKZ                          ;; 4 cycles
        BR        N:??compartment3_249  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
// 2515     {
// 2516     case 0x000f0000:
// 2517       
// 2518       array(1,1);
        MOV       X, #0x1            ;; 1 cycle
        MOV       A, #0x1            ;; 1 cycle
          CFI FunCall _array
        CALL      _array             ;; 3 cycles
// 2519       structure(0x02);
        MOV       A, #0x2            ;; 1 cycle
          CFI FunCall _structure
        CALL      _structure         ;; 3 cycles
// 2520       octet_s(4,0);
        MOV       X, #0x0            ;; 1 cycle
        MOV       A, #0x4            ;; 1 cycle
          CFI FunCall _octet_s
        CALL      _octet_s           ;; 3 cycles
// 2521       info[k++]= bcd_to_decimal(bill_hr);
        MOVW      AX, N:_k           ;; 1 cycle
        MOVW      [SP+0x14], AX      ;; 1 cycle
        MOVW      AX, [SP+0x14]      ;; 1 cycle
        INCW      AX                 ;; 1 cycle
        MOVW      N:_k, AX           ;; 1 cycle
        MOV       A, N:_bill_hr      ;; 1 cycle
          CFI FunCall _bcd_to_decimal
        CALL      _bcd_to_decimal    ;; 3 cycles
        MOV       D, A               ;; 1 cycle
        MOVW      AX, [SP+0x14]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        XCH       A, D               ;; 1 cycle
// 2522       info[k++]= bcd_to_decimal(bill_min);
        MOVW      AX, N:_k           ;; 1 cycle
        MOVW      [SP+0x14], AX      ;; 1 cycle
        MOVW      AX, [SP+0x14]      ;; 1 cycle
        INCW      AX                 ;; 1 cycle
        MOVW      N:_k, AX           ;; 1 cycle
        MOV       A, N:_bill_min     ;; 1 cycle
          CFI FunCall _bcd_to_decimal
        CALL      _bcd_to_decimal    ;; 3 cycles
        MOV       D, A               ;; 1 cycle
        MOVW      AX, [SP+0x14]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        XCH       A, D               ;; 1 cycle
// 2523       info[k++]= 0x00;
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
// 2524       info[k++]= 0x00;
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
// 2525       octet_s(5,0);
        MOV       X, #0x0            ;; 1 cycle
        MOV       A, #0x5            ;; 1 cycle
          CFI FunCall _octet_s
        CALL      _octet_s           ;; 3 cycles
// 2526       info[k++]= 0xff;                  /* year */
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, #0xFF           ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
// 2527       info[k++]= 0xff;                  /* year */
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, #0xFF           ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
// 2528       info[k++]= 0xff;                  /* month */
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, #0xFF           ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
// 2529       info[k++]= bcd_to_decimal(bill_date); /* day of month */
        MOVW      AX, N:_k           ;; 1 cycle
        MOVW      [SP+0x14], AX      ;; 1 cycle
        MOVW      AX, [SP+0x14]      ;; 1 cycle
        INCW      AX                 ;; 1 cycle
        MOVW      N:_k, AX           ;; 1 cycle
        MOV       A, N:_bill_date    ;; 1 cycle
          CFI FunCall _bcd_to_decimal
        CALL      _bcd_to_decimal    ;; 3 cycles
        MOV       D, A               ;; 1 cycle
        MOVW      AX, [SP+0x14]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        XCH       A, D               ;; 1 cycle
// 2530       info[k++]= 0xff;                  /* day od week */
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       A, #0xFF           ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
// 2531       
// 2532       break;
        BR        S:??compartment3_134  ;; 3 cycles
          CFI FunCall _fill_0b
        ; ------------------------------------- Block: 97 cycles
// 2533       
// 2534     default:
// 2535       fill_0b();
??compartment3_249:
        CALL      _fill_0b           ;; 3 cycles
// 2536       break;
// 2537     }
// 2538     break;
        BR        S:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
// 2539     
// 2540     /*	case 0x1702:
// 2541     switch(obis1)
// 2542     {
// 2543   case 0x00160000:
// 2544     
// 2545     enum_d2(5);
// 2546     break;
// 2547     
// 2548     default:
// 2549     fill_0b();
// 2550     break;
// 2551   }
// 2552     break;
// 2553     
// 2554   case 0x1703:
// 2555   case 0x1704:
// 2556     switch(obis1)
// 2557     {
// 2558   case 0x00160000:
// 2559     
// 2560     info[k++]=0;
// 2561     info[k++]=0x11;
// 2562     info[k++]=0x01;
// 2563     break;
// 2564     
// 2565     default:
// 2566     fill_0b();
// 2567     break;
// 2568   }
// 2569     break;
// 2570     
// 2571   case 0x1705:
// 2572   case 0x1706:
// 2573     switch(obis1)
// 2574     {
// 2575   case 0x00160000:
// 2576     
// 2577     val_2byt2(DLMS_MAX_BUFF_SIZE/256,DLMS_MAX_BUFF_SIZE%256); // info size tx rx
// 2578     break;
// 2579     
// 2580     default:
// 2581     fill_0b();
// 2582     break;
// 2583   }
// 2584     break;
// 2585     
// 2586   case 0x1707:
// 2587     switch(obis1)
// 2588     {
// 2589   case 0x00160000:
// 2590     
// 2591     val_2byt2(0x03,0xe8); // iinter_octet_time_out milisec
// 2592     break;
// 2593     
// 2594     default:
// 2595     fill_0b();
// 2596     break;
// 2597   }
// 2598     break;
// 2599     
// 2600   case 0x1708:
// 2601     switch(obis1)
// 2602     {
// 2603   case 0x00160000:
// 2604     
// 2605     val_2byt2(0,120); // inactivity_time_out sec
// 2606     break;
// 2607     
// 2608     default:
// 2609     fill_0b();
// 2610     break;
// 2611   }
// 2612     break;
// 2613     
// 2614   case 0x1709:
// 2615     switch(obis1)
// 2616     {
// 2617   case 0x00160000:
// 2618     
// 2619     info[k++]=0;
// 2620     info[k++]=0x12;
// 2621     info[k++]=meter_address/256;
// 2622     info[k++]=meter_address%256;
// 2623     break;
// 2624     
// 2625     default:
// 2626     fill_0b();
// 2627     break;
// 2628   }
// 2629     break; */
// 2630     
// 2631     
// 2632     
// 2633   case 0x0902:
// 2634     {
// 2635       switch(obis1)
??compartment3_112:
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0xA           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x64          ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??get_resp_233:
        BNZ       ??compartment3_134  ;; 4 cycles
          CFI FunCall _Tarrif_script
        ; ------------------------------------- Block: 4 cycles
// 2636       {
// 2637       case 0x000a0064:
// 2638         Tarrif_script();
        CALL      _Tarrif_script     ;; 3 cycles
// 2639         break;
// 2640       default:
// 2641         break;
// 2642       }
// 2643     }
// 2644     break;
        BR        S:??compartment3_134  ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
// 2645     
// 2646   case 0x3f02:
// 2647     switch(obis1)
??compartment3_123:
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x60          ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0xA01         ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??get_resp_234:
        BNZ       ??compartment3_250  ;; 4 cycles
          CFI FunCall _tamper_status
        ; ------------------------------------- Block: 4 cycles
// 2648     {
// 2649       
// 2650     case 0x00600A01:
// 2651       tamper_status();
        CALL      _tamper_status     ;; 3 cycles
// 2652       break;
        BR        S:??compartment3_134  ;; 3 cycles
          CFI FunCall _fill_0b
        ; ------------------------------------- Block: 6 cycles
// 2653       
// 2654       //            case 0x01600500:
// 2655       //                self_diag_status();
// 2656       //                break;
// 2657       
// 2658     default:
// 2659       fill_0b();
??compartment3_250:
        CALL      _fill_0b           ;; 3 cycles
// 2660       break;
// 2661     }
// 2662     break;
        BR        S:??compartment3_134  ;; 3 cycles
          CFI FunCall _fill_0b
        ; ------------------------------------- Block: 6 cycles
// 2663     
// 2664   default:
// 2665     fill_0b();
??compartment3_124:
        CALL      _fill_0b           ;; 3 cycles
// 2666     break;
        ; ------------------------------------- Block: 3 cycles
// 2667   }
// 2668 }
??compartment3_134:
        ADDW      SP, #0x2A          ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock3
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 6122 cycles
// 2669 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock4 Using cfiCommon0
          CFI Function _capture_objects_filler
        CODE
// 2670 void capture_objects_filler(unsigned char const *p_f)
// 2671 {
_capture_objects_filler:
        ; * Stack frame (at entry) *
        ; Param size: 0
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+6
        ; Auto size: 10
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+14
// 2672   uint16_t buffer_filled_u16= 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [SP+0x06], AX      ;; 1 cycle
// 2673   uint8_t u8temp= 0;
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP+0x04], A       ;; 1 cycle
// 2674   uint32_t obis_cd;
// 2675   
// 2676   obis_cd= obis_short_cal(obis_code);
        MOVW      AX, #LWRD(_obis_code)  ;; 1 cycle
          CFI FunCall _obis_short_cal
        CALL      _obis_short_cal    ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+10
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+12
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+14
// 2677   
// 2678   k= 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_k, AX           ;; 1 cycle
// 2679   Start_Info2();
          CFI FunCall _Start_Info2
        CALL      _Start_Info2       ;; 3 cycles
// 2680   k= 15;
        MOVW      AX, #0xF           ;; 1 cycle
        MOVW      N:_k, AX           ;; 1 cycle
// 2681   
// 2682   buffer_filled_u16= k;
        MOVW      AX, N:_k           ;; 1 cycle
        MOVW      [SP+0x06], AX      ;; 1 cycle
// 2683   if(buffer_first_not_fill_f == 0)
        CMP0      N:_buffer_first_not_fill_f  ;; 1 cycle
        SKZ                          ;; 4 cycles
        BR        N:??compartment3_251  ;; 4 cycles
        ; ------------------------------------- Block: 27 cycles
// 2684   {
// 2685     
// 2686     if((KVAH_SNAP != 1) && ((0x00636200 == obis_cd) || (0x00636201 == obis_cd) || (0x00636204 == obis_cd) || (0x015e5b07 == obis_cd)))
        CMP       N:_KVAH_SNAP, #0x1  ;; 1 cycle
        BZ        ??compartment3_252  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x63          ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x6200        ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??capture_objects_filler_0:
        BZ        ??compartment3_253  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x63          ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x6201        ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??capture_objects_filler_1:
        BZ        ??compartment3_253  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x63          ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x6204        ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??capture_objects_filler_2:
        BZ        ??compartment3_253  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x15E         ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x5B07        ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??capture_objects_filler_3:
        BNZ       ??compartment3_252  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
// 2687     {
// 2688       array(p_f[0] - 1, 0);
??compartment3_253:
        MOV       X, #0x0            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      AX, [SP+0x08]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, [HL]            ;; 1 cycle
        DEC       A                  ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
          CFI FunCall _array
        CALL      _array             ;; 3 cycles
        BR        N:??compartment3_254  ;; 3 cycles
        ; ------------------------------------- Block: 17 cycles
// 2689     }
// 2690     else if((0x01620100 == obis_cd))
??compartment3_252:
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x162         ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x100         ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??capture_objects_filler_4:
        BNZ       ??compartment3_255  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
// 2691     {
// 2692       if((2 == TOD_energy_config) || (3 == TOD_energy_config))
        CMP       N:_TOD_energy_config, #0x2  ;; 1 cycle
        BZ        ??compartment3_256  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        CMP       N:_TOD_energy_config, #0x3  ;; 1 cycle
        BNZ       ??compartment3_257  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 2693       {
// 2694         array(p_f[0] - FUENERGY_REQ - BILLTPR_CNT - MDRESET_TYPE_CONFIG - 8, 0);
??compartment3_256:
        MOV       X, #0x0            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      AX, [SP+0x08]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, [HL]            ;; 1 cycle
        SUB       A, N:_FUENERGY_REQ  ;; 1 cycle
        SUB       A, N:_BILLTPR_CNT  ;; 1 cycle
        SUB       A, N:_MDRESET_TYPE_CONFIG  ;; 1 cycle
        ADD       A, #0xF8           ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
          CFI FunCall _array
        CALL      _array             ;; 3 cycles
        BR        N:??compartment3_254  ;; 3 cycles
        ; ------------------------------------- Block: 20 cycles
// 2695       }
// 2696       else if(TOD_energy_config == 1)
??compartment3_257:
        CMP       N:_TOD_energy_config, #0x1  ;; 1 cycle
        BNZ       ??compartment3_258  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 2697       {
// 2698         array(p_f[0] - FUENERGY_REQ - BILLTPR_CNT - MDRESET_TYPE_CONFIG - 16, 0);
        MOV       X, #0x0            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      AX, [SP+0x08]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, [HL]            ;; 1 cycle
        SUB       A, N:_FUENERGY_REQ  ;; 1 cycle
        SUB       A, N:_BILLTPR_CNT  ;; 1 cycle
        SUB       A, N:_MDRESET_TYPE_CONFIG  ;; 1 cycle
        ADD       A, #0xF0           ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
          CFI FunCall _array
        CALL      _array             ;; 3 cycles
        BR        N:??compartment3_254  ;; 3 cycles
        ; ------------------------------------- Block: 20 cycles
// 2699       }
// 2700       else
// 2701         array(p_f[0] - FUENERGY_REQ - BILLTPR_CNT - MDRESET_TYPE_CONFIG - 24, 0);
??compartment3_258:
        MOV       X, #0x0            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      AX, [SP+0x08]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, [HL]            ;; 1 cycle
        SUB       A, N:_FUENERGY_REQ  ;; 1 cycle
        SUB       A, N:_BILLTPR_CNT  ;; 1 cycle
        SUB       A, N:_MDRESET_TYPE_CONFIG  ;; 1 cycle
        ADD       A, #0xE8           ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
          CFI FunCall _array
        CALL      _array             ;; 3 cycles
        BR        N:??compartment3_254  ;; 3 cycles
        ; ------------------------------------- Block: 20 cycles
// 2702     }
// 2703     else if((0x015e5b06 == obis_cd))
??compartment3_255:
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x15E         ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x5B06        ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??capture_objects_filler_5:
        BNZ       ??compartment3_259  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
// 2704     {
// 2705       if((2 == TOD_energy_config) || (3 == TOD_energy_config))
        CMP       N:_TOD_energy_config, #0x2  ;; 1 cycle
        BZ        ??compartment3_260  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        CMP       N:_TOD_energy_config, #0x3  ;; 1 cycle
        BNZ       ??compartment3_261  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 2706       {
// 2707         array(p_f[0] - FUENERGY_REQ - 8, 0);
??compartment3_260:
        MOV       X, #0x0            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      AX, [SP+0x08]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, [HL]            ;; 1 cycle
        SUB       A, N:_FUENERGY_REQ  ;; 1 cycle
        ADD       A, #0xF8           ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
          CFI FunCall _array
        CALL      _array             ;; 3 cycles
        BR        N:??compartment3_254  ;; 3 cycles
        ; ------------------------------------- Block: 18 cycles
// 2708       }
// 2709       else if(TOD_energy_config == 1)
??compartment3_261:
        CMP       N:_TOD_energy_config, #0x1  ;; 1 cycle
        BNZ       ??compartment3_262  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 2710       {
// 2711         array(p_f[0] - FUENERGY_REQ - 16, 0);
        MOV       X, #0x0            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      AX, [SP+0x08]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, [HL]            ;; 1 cycle
        SUB       A, N:_FUENERGY_REQ  ;; 1 cycle
        ADD       A, #0xF0           ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
          CFI FunCall _array
        CALL      _array             ;; 3 cycles
        BR        N:??compartment3_254  ;; 3 cycles
        ; ------------------------------------- Block: 18 cycles
// 2712       }
// 2713       else
// 2714         array(p_f[0] - FUENERGY_REQ - 24, 0);
??compartment3_262:
        MOV       X, #0x0            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      AX, [SP+0x08]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, [HL]            ;; 1 cycle
        SUB       A, N:_FUENERGY_REQ  ;; 1 cycle
        ADD       A, #0xE8           ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
          CFI FunCall _array
        CALL      _array             ;; 3 cycles
        BR        N:??compartment3_254  ;; 3 cycles
        ; ------------------------------------- Block: 18 cycles
// 2715     }
// 2716     else if((D_KVARH_REQ != 1) && ((0x01630200 == obis_cd) || (0x015e5b05 == obis_cd)))
??compartment3_259:
        CMP       N:_D_KVARH_REQ, #0x1  ;; 1 cycle
        BZ        ??compartment3_263  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x163         ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x200         ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??capture_objects_filler_6:
        BZ        ??compartment3_264  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x15E         ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x5B05        ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??capture_objects_filler_7:
        BNZ       ??compartment3_263  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
// 2717     {
// 2718       array(p_f[0] - 2, 0);
??compartment3_264:
        MOV       X, #0x0            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      AX, [SP+0x08]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, [HL]            ;; 1 cycle
        ADD       A, #0xFE           ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
          CFI FunCall _array
        CALL      _array             ;; 3 cycles
        BR        S:??compartment3_254  ;; 3 cycles
        ; ------------------------------------- Block: 17 cycles
// 2719     }
// 2720     else if((CUM_MAX_DEMAND != 1) && ((0x015e5b00 == obis_cd) || (0x015e5b03 == obis_cd)))
??compartment3_263:
        CMP       N:_CUM_MAX_DEMAND, #0x1  ;; 1 cycle
        BZ        ??compartment3_265  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x15E         ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x5B00        ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??capture_objects_filler_8:
        BZ        ??compartment3_266  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x15E         ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x5B03        ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??capture_objects_filler_9:
        BNZ       ??compartment3_265  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
// 2721     {
// 2722       array(p_f[0] - 2, 0);
??compartment3_266:
        MOV       X, #0x0            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      AX, [SP+0x08]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, [HL]            ;; 1 cycle
        ADD       A, #0xFE           ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
          CFI FunCall _array
        CALL      _array             ;; 3 cycles
        BR        S:??compartment3_254  ;; 3 cycles
        ; ------------------------------------- Block: 17 cycles
// 2723     }
// 2724     
// 2725     else
// 2726     {
// 2727       array(p_f[0], 0);
??compartment3_265:
        MOV       X, #0x0            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      AX, [SP+0x08]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, [HL]            ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
          CFI FunCall _array
        CALL      _array             ;; 3 cycles
        ; ------------------------------------- Block: 13 cycles
// 2728     }
// 2729     
// 2730     block_no= 1;
??compartment3_254:
        MOVW      AX, #0x1           ;; 1 cycle
        MOVW      N:_block_no, AX    ;; 1 cycle
// 2731     
// 2732     element_filled= 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_element_filled, AX  ;; 1 cycle
// 2733     multi_filling_f= 1;
        MOV       N:_multi_filling_f, #0x1  ;; 1 cycle
// 2734     buffer_first_not_fill_f= 1;
        MOV       N:_buffer_first_not_fill_f, #0x1  ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
// 2735   }
// 2736   
// 2737   for(; element_filled < p_f[0]; element_filled++)
??compartment3_251:
        MOVW      AX, [SP+0x08]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, [HL]            ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      HL, N:_element_filled  ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        SKH                          ;; 4 cycles
        BR        N:??compartment3_267  ;; 4 cycles
        ; ------------------------------------- Block: 11 cycles
// 2738   {
// 2739     if((KVAH_SNAP != 1) && (12 == element_filled) && ((0x00636200 == obis_cd) || (0x00636201 == obis_cd) || (0x00636204 == obis_cd)))
        CMP       N:_KVAH_SNAP, #0x1  ;; 1 cycle
        BZ        ??compartment3_268  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        MOVW      AX, N:_element_filled  ;; 1 cycle
        CMPW      AX, #0xC           ;; 1 cycle
        BNZ       ??compartment3_268  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x63          ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x6200        ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??capture_objects_filler_10:
        BZ        ??compartment3_269  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x63          ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x6201        ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??capture_objects_filler_11:
        BZ        ??compartment3_269  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x63          ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x6204        ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??capture_objects_filler_12:
        BNZ       ??compartment3_268  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
// 2740     {
// 2741       element_filled++;
??compartment3_269:
        INCW      N:_element_filled  ;; 2 cycles
// 2742       break;
        BR        N:??compartment3_267  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 2743     }
// 2744     if((KVAH_SNAP != 1) && (10 == element_filled) && (0x015e5b07 == obis_cd))
??compartment3_268:
        CMP       N:_KVAH_SNAP, #0x1  ;; 1 cycle
        BZ        ??compartment3_270  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        MOVW      AX, N:_element_filled  ;; 1 cycle
        CMPW      AX, #0xA           ;; 1 cycle
        BNZ       ??compartment3_270  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x15E         ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x5B07        ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??capture_objects_filler_13:
        BNZ       ??compartment3_270  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
// 2745     {
// 2746       element_filled++;
        INCW      N:_element_filled  ;; 2 cycles
// 2747       break;
        BR        N:??compartment3_267  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 2748     }
// 2749     
// 2750     if((FUENERGY_REQ != 0) && (44 == element_filled) && (0x015e5b06 == obis_cd))
??compartment3_270:
        CMP0      N:_FUENERGY_REQ    ;; 1 cycle
        BZ        ??compartment3_271  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        MOVW      AX, N:_element_filled  ;; 1 cycle
        CMPW      AX, #0x2C          ;; 1 cycle
        BNZ       ??compartment3_271  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x15E         ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x5B06        ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??capture_objects_filler_14:
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
// 2751     {
// 2752       element_filled++;
        INCW      N:_element_filled  ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
// 2753     }
// 2754     if(TOD_energy_config != 0) //2,77
??compartment3_271:
        CMP0      N:_TOD_energy_config  ;; 1 cycle
        BZ        ??compartment3_272  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 2755     {
// 2756       if((TOD_energy_config == 1) && (53 == element_filled) && (0x015e5b06 == obis_cd))
        CMP       N:_TOD_energy_config, #0x1  ;; 1 cycle
        BNZ       ??compartment3_273  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        MOVW      AX, N:_element_filled  ;; 1 cycle
        CMPW      AX, #0x35          ;; 1 cycle
        BNZ       ??compartment3_273  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x15E         ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x5B06        ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??capture_objects_filler_15:
        BNZ       ??compartment3_273  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
// 2757       {
// 2758         element_filled+= 16;
        MOVW      AX, N:_element_filled  ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
        MOVW      N:_element_filled, AX  ;; 1 cycle
        ; ------------------------------------- Block: 3 cycles
// 2759         //break;
// 2760       }
// 2761       if((TOD_energy_config == 3) && (53 == element_filled) && (0x015e5b06 == obis_cd))
??compartment3_273:
        CMP       N:_TOD_energy_config, #0x3  ;; 1 cycle
        BNZ       ??compartment3_274  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        MOVW      AX, N:_element_filled  ;; 1 cycle
        CMPW      AX, #0x35          ;; 1 cycle
        BNZ       ??compartment3_274  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x15E         ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x5B06        ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??capture_objects_filler_16:
        BNZ       ??compartment3_274  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
// 2762       {
// 2763         element_filled+= 8;
        MOVW      AX, N:_element_filled  ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
        MOVW      N:_element_filled, AX  ;; 1 cycle
        ; ------------------------------------- Block: 3 cycles
// 2764         //break;
// 2765       }
// 2766       if((TOD_energy_config == 2) && (61 == element_filled) && (0x015e5b06 == obis_cd))
??compartment3_274:
        CMP       N:_TOD_energy_config, #0x2  ;; 1 cycle
        BNZ       ??compartment3_275  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        MOVW      AX, N:_element_filled  ;; 1 cycle
        CMPW      AX, #0x3D          ;; 1 cycle
        BNZ       ??compartment3_275  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x15E         ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x5B06        ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??capture_objects_filler_17:
        BNZ       ??compartment3_275  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
// 2767       {
// 2768         element_filled+= 8;
        MOVW      AX, N:_element_filled  ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
        MOVW      N:_element_filled, AX  ;; 1 cycle
        BR        S:??compartment3_275  ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
// 2769         // break;
// 2770       }
// 2771     }
// 2772     else
// 2773     {
// 2774       if((TOD_energy_config == 0) && (45 == element_filled) && (0x015e5b06 == obis_cd))
??compartment3_272:
        CMP0      N:_TOD_energy_config  ;; 1 cycle
        BNZ       ??compartment3_275  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        MOVW      AX, N:_element_filled  ;; 1 cycle
        CMPW      AX, #0x2D          ;; 1 cycle
        BNZ       ??compartment3_275  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x15E         ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x5B06        ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??capture_objects_filler_18:
        BNZ       ??compartment3_275  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
// 2775       {
// 2776         element_filled+= 24;
        MOVW      AX, N:_element_filled  ;; 1 cycle
        ADDW      AX, #0x18          ;; 1 cycle
        MOVW      N:_element_filled, AX  ;; 1 cycle
// 2777         break;
        BR        N:??compartment3_267  ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
// 2778       }
// 2779     }
// 2780     if((FUENERGY_REQ != 0) && (65 == element_filled) && (0x01620100 == obis_cd))
??compartment3_275:
        CMP0      N:_FUENERGY_REQ    ;; 1 cycle
        BZ        ??compartment3_276  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        MOVW      AX, N:_element_filled  ;; 1 cycle
        CMPW      AX, #0x41          ;; 1 cycle
        BNZ       ??compartment3_276  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x162         ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x100         ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??capture_objects_filler_19:
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
// 2781     {
// 2782       element_filled++;
        INCW      N:_element_filled  ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
// 2783     }
// 2784     if((BILLTPR_CNT != 0) && (66 == element_filled) && (0x01620100 == obis_cd))
??compartment3_276:
        CMP0      N:_BILLTPR_CNT     ;; 1 cycle
        BZ        ??compartment3_277  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        MOVW      AX, N:_element_filled  ;; 1 cycle
        CMPW      AX, #0x42          ;; 1 cycle
        BNZ       ??compartment3_277  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x162         ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x100         ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??capture_objects_filler_20:
        BNZ       ??compartment3_277  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
// 2785     {
// 2786       element_filled++;
        INCW      N:_element_filled  ;; 2 cycles
// 2787       break;
        BR        N:??compartment3_267  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 2788     }
// 2789     if((MDRESET_TYPE_CONFIG != 0) && (67 == element_filled) && (0x01620100 == obis_cd))
??compartment3_277:
        CMP0      N:_MDRESET_TYPE_CONFIG  ;; 1 cycle
        BZ        ??compartment3_278  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        MOVW      AX, N:_element_filled  ;; 1 cycle
        CMPW      AX, #0x43          ;; 1 cycle
        BNZ       ??compartment3_278  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x162         ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x100         ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??capture_objects_filler_21:
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
// 2790     {
// 2791       element_filled++;
        INCW      N:_element_filled  ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
// 2792       //break;
// 2793     }
// 2794     if(TOD_energy_config != 0) //2,77
??compartment3_278:
        CMP0      N:_TOD_energy_config  ;; 1 cycle
        BZ        ??compartment3_279  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 2795     {
// 2796       
// 2797       if((TOD_energy_config == 1) && (76 == element_filled) && (0x01620100 == obis_cd))
        CMP       N:_TOD_energy_config, #0x1  ;; 1 cycle
        BNZ       ??compartment3_280  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        MOVW      AX, N:_element_filled  ;; 1 cycle
        CMPW      AX, #0x4C          ;; 1 cycle
        BNZ       ??compartment3_280  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x162         ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x100         ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??capture_objects_filler_22:
        BNZ       ??compartment3_280  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
// 2798       {
// 2799         element_filled+= 16;
        MOVW      AX, N:_element_filled  ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
        MOVW      N:_element_filled, AX  ;; 1 cycle
        ; ------------------------------------- Block: 3 cycles
// 2800         //break;
// 2801       }
// 2802       if((TOD_energy_config == 3) && (76 == element_filled) && (0x01620100 == obis_cd))
??compartment3_280:
        CMP       N:_TOD_energy_config, #0x3  ;; 1 cycle
        BNZ       ??compartment3_281  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        MOVW      AX, N:_element_filled  ;; 1 cycle
        CMPW      AX, #0x4C          ;; 1 cycle
        BNZ       ??compartment3_281  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x162         ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x100         ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??capture_objects_filler_23:
        BNZ       ??compartment3_281  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
// 2803       {
// 2804         element_filled+= 8;
        MOVW      AX, N:_element_filled  ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
        MOVW      N:_element_filled, AX  ;; 1 cycle
        ; ------------------------------------- Block: 3 cycles
// 2805         //break;
// 2806       }
// 2807       if((TOD_energy_config == 2) && (84 == element_filled) && (0x01620100 == obis_cd))
??compartment3_281:
        CMP       N:_TOD_energy_config, #0x2  ;; 1 cycle
        BNZ       ??compartment3_282  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        MOVW      AX, N:_element_filled  ;; 1 cycle
        CMPW      AX, #0x54          ;; 1 cycle
        BNZ       ??compartment3_282  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x162         ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x100         ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??capture_objects_filler_24:
        BNZ       ??compartment3_282  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
// 2808       {
// 2809         element_filled+= 8;
        MOVW      AX, N:_element_filled  ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
        MOVW      N:_element_filled, AX  ;; 1 cycle
        BR        S:??compartment3_282  ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
// 2810         // break;
// 2811       }
// 2812     }
// 2813     else
// 2814     {
// 2815       if((TOD_energy_config == 0) && (68 == element_filled) && (0x01620100 == obis_cd))
??compartment3_279:
        CMP0      N:_TOD_energy_config  ;; 1 cycle
        BNZ       ??compartment3_282  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        MOVW      AX, N:_element_filled  ;; 1 cycle
        CMPW      AX, #0x44          ;; 1 cycle
        BNZ       ??compartment3_282  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x162         ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x100         ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??capture_objects_filler_25:
        BNZ       ??compartment3_282  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
// 2816       {
// 2817         element_filled+= 24;
        MOVW      AX, N:_element_filled  ;; 1 cycle
        ADDW      AX, #0x18          ;; 1 cycle
        MOVW      N:_element_filled, AX  ;; 1 cycle
// 2818         break;
        BR        N:??compartment3_267  ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
// 2819       }
// 2820     }
// 2821     
// 2822     if((D_KVARH_REQ != 1) && (4 == element_filled) && (0x01630200 == obis_cd))
??compartment3_282:
        CMP       N:_D_KVARH_REQ, #0x1  ;; 1 cycle
        BZ        ??compartment3_283  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        MOVW      AX, N:_element_filled  ;; 1 cycle
        CMPW      AX, #0x4           ;; 1 cycle
        BNZ       ??compartment3_283  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x163         ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x200         ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??capture_objects_filler_26:
        BNZ       ??compartment3_283  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
// 2823     {
// 2824       element_filled+=2;
        MOVW      AX, N:_element_filled  ;; 1 cycle
        ADDW      AX, #0x2           ;; 1 cycle
        MOVW      N:_element_filled, AX  ;; 1 cycle
// 2825       break;
        BR        N:??compartment3_267  ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
// 2826     }
// 2827     
// 2828     if((D_KVARH_REQ != 1) && (3 == element_filled) && (0x015e5b05 == obis_cd))
??compartment3_283:
        CMP       N:_D_KVARH_REQ, #0x1  ;; 1 cycle
        BZ        ??compartment3_284  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        MOVW      AX, N:_element_filled  ;; 1 cycle
        CMPW      AX, #0x3           ;; 1 cycle
        BNZ       ??compartment3_284  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x15E         ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x5B05        ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??capture_objects_filler_27:
        BNZ       ??compartment3_284  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
// 2829     {
// 2830       element_filled+=2;
        MOVW      AX, N:_element_filled  ;; 1 cycle
        ADDW      AX, #0x2           ;; 1 cycle
        MOVW      N:_element_filled, AX  ;; 1 cycle
// 2831       break;
        BR        N:??compartment3_267  ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
// 2832     }
// 2833     
// 2834     /***/
// 2835     else if((CUM_MAX_DEMAND != 1) && (29 == element_filled) && ((0x015e5b00 == obis_cd)))
??compartment3_284:
        CMP       N:_CUM_MAX_DEMAND, #0x1  ;; 1 cycle
        BZ        ??compartment3_285  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        MOVW      AX, N:_element_filled  ;; 1 cycle
        CMPW      AX, #0x1D          ;; 1 cycle
        BNZ       ??compartment3_285  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x15E         ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x5B00        ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??capture_objects_filler_28:
        BNZ       ??compartment3_285  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
// 2836     {
// 2837       element_filled++;
        INCW      N:_element_filled  ;; 2 cycles
// 2838       element_filled++;
        INCW      N:_element_filled  ;; 2 cycles
// 2839       break;
        BR        N:??compartment3_267  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 2840     }
// 2841     
// 2842     else if((CUM_MAX_DEMAND != 1) && (22 == element_filled) && ((0x015e5b03 == obis_cd)))
??compartment3_285:
        CMP       N:_CUM_MAX_DEMAND, #0x1  ;; 1 cycle
        BZ        ??compartment3_286  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        MOVW      AX, N:_element_filled  ;; 1 cycle
        CMPW      AX, #0x16          ;; 1 cycle
        BNZ       ??compartment3_286  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x15E         ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x5B03        ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??capture_objects_filler_29:
        BNZ       ??compartment3_286  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
// 2843     {
// 2844       element_filled++;
        INCW      N:_element_filled  ;; 2 cycles
// 2845       element_filled++;
        INCW      N:_element_filled  ;; 2 cycles
// 2846       break;
        BR        N:??compartment3_267  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 2847     }
// 2848     
// 2849     buffer_filled_u16+= 18;
??compartment3_286:
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        ADDW      AX, #0x12          ;; 1 cycle
        MOVW      [SP+0x06], AX      ;; 1 cycle
// 2850     
// 2851     profile_sel(p_f[1 + (element_filled * 8)], p_f[1 + (element_filled * 8) + 1], p_f[1 + (element_filled * 8) + 2],
// 2852                 p_f[1 + (element_filled * 8) + 3], p_f[1 + (element_filled * 8) + 4], p_f[1 + (element_filled * 8) + 5],
// 2853                 p_f[1 + (element_filled * 8) + 6], p_f[1 + (element_filled * 8) + 7]);
        MOVW      AX, N:_element_filled  ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [SP+0x08]      ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, [HL+0x08]       ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+16
        XCH       A, X               ;; 1 cycle
        MOVW      AX, N:_element_filled  ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [SP+0x0A]      ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, [HL+0x07]       ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+18
        XCH       A, X               ;; 1 cycle
        MOVW      AX, N:_element_filled  ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [SP+0x0C]      ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, [HL+0x06]       ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+20
        XCH       A, X               ;; 1 cycle
        MOVW      AX, N:_element_filled  ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [SP+0x0E]      ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, [HL+0x05]       ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOVW      AX, N:_element_filled  ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [SP+0x0E]      ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, [HL+0x04]       ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOVW      AX, N:_element_filled  ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [SP+0x0E]      ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, [HL+0x03]       ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        PUSH      DE                 ;; 1 cycle
          CFI CFA SP+22
        XCH       A, B               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        MOVW      AX, N:_element_filled  ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [SP+0x10]      ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, [HL+0x02]       ;; 1 cycle
        MOV       C, A               ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+20
        XCH       A, D               ;; 1 cycle
        PUSH      DE                 ;; 1 cycle
          CFI CFA SP+22
        PUSH      DE                 ;; 1 cycle
          CFI CFA SP+24
        XCH       A, B               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        XCH       A, C               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        XCH       A, C               ;; 1 cycle
        MOVW      AX, N:_element_filled  ;; 1 cycle
        MOVW      BC, #0x8           ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [SP+0x12]      ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        INCW      AX                 ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, [HL]            ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       C, A               ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        XCH       A, E               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+22
        XCH       A, E               ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+20
        XCH       A, D               ;; 1 cycle
          CFI FunCall _profile_sel
        CALL      _profile_sel       ;; 3 cycles
// 2854     if(DLMS_MAX_BUFF_SIZE < (buffer_filled_u16 + 18))
        MOVW      AX, [SP+0x0C]      ;; 1 cycle
        ADDW      AX, #0x12          ;; 1 cycle
        ADDW      SP, #0x6           ;; 1 cycle
          CFI CFA SP+14
        CMPW      AX, #0x201         ;; 1 cycle
        BNC       ??compartment3_267  ;; 4 cycles
        ; ------------------------------------- Block: 131 cycles
// 2855     {
// 2856       break;
// 2857     }
// 2858   }
        INCW      N:_element_filled  ;; 2 cycles
        BR        N:??compartment3_251  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 2859   element_filled++;
??compartment3_267:
        INCW      N:_element_filled  ;; 2 cycles
// 2860   if(element_filled >= p_f[0])
        MOVW      AX, [SP+0x08]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, [HL]            ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      HL, N:_element_filled  ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BH        ??compartment3_287  ;; 4 cycles
        ; ------------------------------------- Block: 13 cycles
// 2861   {
// 2862     multi_filling_f= 0;
        MOV       N:_multi_filling_f, #0x0  ;; 1 cycle
// 2863     buffer_first_not_fill_f= 0;
        MOV       N:_buffer_first_not_fill_f, #0x0  ;; 1 cycle
// 2864     u8temp= 1;
        MOV       A, #0x1            ;; 1 cycle
        MOV       [SP+0x04], A       ;; 1 cycle
        ; ------------------------------------- Block: 4 cycles
// 2865   }
// 2866   
// 2867   info[6]= u8temp;
??compartment3_287:
        MOV       A, [SP+0x04]       ;; 1 cycle
        MOV       N:_info+6, A       ;; 1 cycle
// 2868   info[10]= block_no;
        MOVW      AX, N:_block_no    ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       N:_info+10, A      ;; 1 cycle
        XCH       A, X               ;; 1 cycle
// 2869   info[13]= (k - 15) / 256;
        MOVW      AX, N:_k           ;; 1 cycle
        ADDW      AX, #0xFFF1        ;; 1 cycle
        CLRB      X                  ;; 1 cycle
        MOV       N:_info+13, A      ;; 1 cycle
        XCH       A, X               ;; 1 cycle
// 2870   info[14]= (uint8_t)(k - 15);
        MOVW      AX, N:_k           ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        ADD       A, #0xF1           ;; 1 cycle
        MOV       N:_info+14, A      ;; 1 cycle
// 2871   
// 2872   block_no++;
        INCW      N:_block_no        ;; 2 cycles
// 2873   info_send= k;
        MOVW      AX, N:_k           ;; 1 cycle
        MOVW      N:_info_send, AX   ;; 1 cycle
// 2874   info_sended= 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_info_sended, AX  ;; 1 cycle
// 2875   send_type_multi();
          CFI FunCall _send_type_multi
        CALL      _send_type_multi   ;; 3 cycles
// 2876 }
        ADDW      SP, #0xA           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock4
        ; ------------------------------------- Block: 31 cycles
        ; ------------------------------------- Total: 1093 cycles
// 2877 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock5 Using cfiCommon0
          CFI Function _buffer_scaler_filler
        CODE
// 2878 void buffer_scaler_filler(unsigned char const  *s_f)
// 2879 {
_buffer_scaler_filler:
        ; * Stack frame (at entry) *
        ; Param size: 0
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+6
        ; Auto size: 10
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+14
// 2880   uint16_t buffer_filled_u16= 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [SP+0x06], AX      ;; 1 cycle
// 2881   uint8_t u8temp= 0;
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP+0x04], A       ;; 1 cycle
// 2882   uint32_t obis_cd;
// 2883   
// 2884   
// 2885   
// 2886   k= 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_k, AX           ;; 1 cycle
// 2887   Start_Info2();
          CFI FunCall _Start_Info2
        CALL      _Start_Info2       ;; 3 cycles
// 2888   k= 15;
        MOVW      AX, #0xF           ;; 1 cycle
        MOVW      N:_k, AX           ;; 1 cycle
// 2889   
// 2890   obis_cd= obis_short_cal(obis_code);
        MOVW      AX, #LWRD(_obis_code)  ;; 1 cycle
          CFI FunCall _obis_short_cal
        CALL      _obis_short_cal    ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+10
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+12
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+14
// 2891   
// 2892   buffer_filled_u16= k;
        MOVW      AX, N:_k           ;; 1 cycle
        MOVW      [SP+0x06], AX      ;; 1 cycle
// 2893   if(buffer_first_not_fill_f == 0)
        CMP0      N:_buffer_first_not_fill_f  ;; 1 cycle
        SKZ                          ;; 4 cycles
        BR        N:??compartment3_288  ;; 4 cycles
        ; ------------------------------------- Block: 27 cycles
// 2894   {
// 2895     array(0x01, 0); /* s_f[0]; */
        MOV       X, #0x0            ;; 1 cycle
        MOV       A, #0x1            ;; 1 cycle
          CFI FunCall _array
        CALL      _array             ;; 3 cycles
// 2896     
// 2897     if((0 == KVAH_SNAP) && (0x015e5b07 == obis_cd))
        CMP0      N:_KVAH_SNAP       ;; 1 cycle
        BNZ       ??compartment3_289  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x15E         ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x5B07        ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??buffer_scaler_filler_0:
        BNZ       ??compartment3_289  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
// 2898     {
// 2899       structure(s_f[0] - 1);
        MOVW      AX, [SP+0x08]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, [HL]            ;; 1 cycle
        DEC       A                  ;; 1 cycle
          CFI FunCall _structure
        CALL      _structure         ;; 3 cycles
        BR        N:??compartment3_290  ;; 3 cycles
        ; ------------------------------------- Block: 10 cycles
// 2900     }
// 2901     
// 2902     /* if((0x015e5b06==obis_cd))
// 2903     {
// 2904     structure(s_f[0]-FUENERGY_REQ);
// 2905   }*/
// 2906     
// 2907     else if((0x015e5b06 == obis_cd))
??compartment3_289:
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x15E         ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x5B06        ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??buffer_scaler_filler_1:
        BNZ       ??compartment3_291  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
// 2908     {
// 2909       if((2 == TOD_energy_config) || (3 == TOD_energy_config))
        CMP       N:_TOD_energy_config, #0x2  ;; 1 cycle
        BZ        ??compartment3_292  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        CMP       N:_TOD_energy_config, #0x3  ;; 1 cycle
        BNZ       ??compartment3_293  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 2910       {
// 2911         structure(s_f[0] - FUENERGY_REQ - 8);
??compartment3_292:
        MOVW      AX, [SP+0x08]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, [HL]            ;; 1 cycle
        SUB       A, N:_FUENERGY_REQ  ;; 1 cycle
        ADD       A, #0xF8           ;; 1 cycle
          CFI FunCall _structure
        CALL      _structure         ;; 3 cycles
        BR        S:??compartment3_290  ;; 3 cycles
        ; ------------------------------------- Block: 11 cycles
// 2912       }
// 2913       else if(TOD_energy_config == 1)
??compartment3_293:
        CMP       N:_TOD_energy_config, #0x1  ;; 1 cycle
        BNZ       ??compartment3_294  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 2914       {
// 2915         structure(s_f[0] - FUENERGY_REQ - 16);
        MOVW      AX, [SP+0x08]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, [HL]            ;; 1 cycle
        SUB       A, N:_FUENERGY_REQ  ;; 1 cycle
        ADD       A, #0xF0           ;; 1 cycle
          CFI FunCall _structure
        CALL      _structure         ;; 3 cycles
        BR        S:??compartment3_290  ;; 3 cycles
        ; ------------------------------------- Block: 11 cycles
// 2916       }
// 2917       else
// 2918         structure(s_f[0] - FUENERGY_REQ - 24);
??compartment3_294:
        MOVW      AX, [SP+0x08]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, [HL]            ;; 1 cycle
        SUB       A, N:_FUENERGY_REQ  ;; 1 cycle
        ADD       A, #0xE8           ;; 1 cycle
          CFI FunCall _structure
        CALL      _structure         ;; 3 cycles
        BR        S:??compartment3_290  ;; 3 cycles
        ; ------------------------------------- Block: 11 cycles
// 2919     }
// 2920     
// 2921     else if((D_KVARH_REQ == 0) && (0x015e5b05 == obis_cd))
??compartment3_291:
        CMP0      N:_D_KVARH_REQ     ;; 1 cycle
        BNZ       ??compartment3_295  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x15E         ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x5B05        ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??buffer_scaler_filler_2:
        BNZ       ??compartment3_295  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
// 2922     {
// 2923       structure(s_f[0] - 2);
        MOVW      AX, [SP+0x08]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, [HL]            ;; 1 cycle
        ADD       A, #0xFE           ;; 1 cycle
          CFI FunCall _structure
        CALL      _structure         ;; 3 cycles
        BR        S:??compartment3_290  ;; 3 cycles
        ; ------------------------------------- Block: 10 cycles
// 2924     }
// 2925     
// 2926     else if((CUM_MAX_DEMAND == 0) && (0x015e5b03 == obis_cd))
??compartment3_295:
        CMP0      N:_CUM_MAX_DEMAND  ;; 1 cycle
        BNZ       ??compartment3_296  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x15E         ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x5B03        ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??buffer_scaler_filler_3:
        BNZ       ??compartment3_296  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
// 2927     {
// 2928       structure(s_f[0] - 2);
        MOVW      AX, [SP+0x08]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, [HL]            ;; 1 cycle
        ADD       A, #0xFE           ;; 1 cycle
          CFI FunCall _structure
        CALL      _structure         ;; 3 cycles
        BR        S:??compartment3_290  ;; 3 cycles
        ; ------------------------------------- Block: 10 cycles
// 2929     }
// 2930     
// 2931     else
// 2932     {
// 2933       structure(s_f[0]);
??compartment3_296:
        MOVW      AX, [SP+0x08]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, [HL]            ;; 1 cycle
          CFI FunCall _structure
        CALL      _structure         ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
// 2934     }
// 2935     
// 2936     block_no= 1;
??compartment3_290:
        MOVW      AX, #0x1           ;; 1 cycle
        MOVW      N:_block_no, AX    ;; 1 cycle
// 2937     element_filled= 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_element_filled, AX  ;; 1 cycle
// 2938     multi_filling_f= 1;
        MOV       N:_multi_filling_f, #0x1  ;; 1 cycle
// 2939     buffer_first_not_fill_f= 1;
        MOV       N:_buffer_first_not_fill_f, #0x1  ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
// 2940   }
// 2941   
// 2942   for(; element_filled < s_f[0]; element_filled++)
??compartment3_288:
        MOVW      AX, [SP+0x08]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, [HL]            ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      HL, N:_element_filled  ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        SKH                          ;; 4 cycles
        BR        N:??compartment3_297  ;; 4 cycles
        ; ------------------------------------- Block: 11 cycles
// 2943   {
// 2944     
// 2945     if((FUENERGY_REQ != 0) && (44 == element_filled) && (0x015e5b06 == obis_cd))
        CMP0      N:_FUENERGY_REQ    ;; 1 cycle
        BZ        ??compartment3_298  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        MOVW      AX, N:_element_filled  ;; 1 cycle
        CMPW      AX, #0x2C          ;; 1 cycle
        BNZ       ??compartment3_298  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x15E         ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x5B06        ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??buffer_scaler_filler_4:
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
// 2946     {
// 2947       element_filled++;
        INCW      N:_element_filled  ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
// 2948       // break;
// 2949     }
// 2950     
// 2951     if(TOD_energy_config != 0) //2,77
??compartment3_298:
        CMP0      N:_TOD_energy_config  ;; 1 cycle
        BZ        ??compartment3_299  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 2952     {
// 2953       if((TOD_energy_config == 1) && (53 == element_filled) && (0x015e5b06 == obis_cd))
        CMP       N:_TOD_energy_config, #0x1  ;; 1 cycle
        BNZ       ??compartment3_300  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        MOVW      AX, N:_element_filled  ;; 1 cycle
        CMPW      AX, #0x35          ;; 1 cycle
        BNZ       ??compartment3_300  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x15E         ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x5B06        ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??buffer_scaler_filler_5:
        BNZ       ??compartment3_300  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
// 2954       {
// 2955         element_filled+= 16;
        MOVW      AX, N:_element_filled  ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
        MOVW      N:_element_filled, AX  ;; 1 cycle
        ; ------------------------------------- Block: 3 cycles
// 2956         //  break;
// 2957       }
// 2958       if((TOD_energy_config == 3) && (53 == element_filled) && (0x015e5b06 == obis_cd))
??compartment3_300:
        CMP       N:_TOD_energy_config, #0x3  ;; 1 cycle
        BNZ       ??compartment3_301  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        MOVW      AX, N:_element_filled  ;; 1 cycle
        CMPW      AX, #0x35          ;; 1 cycle
        BNZ       ??compartment3_301  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x15E         ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x5B06        ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??buffer_scaler_filler_6:
        BNZ       ??compartment3_301  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
// 2959       {
// 2960         element_filled+= 8;
        MOVW      AX, N:_element_filled  ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
        MOVW      N:_element_filled, AX  ;; 1 cycle
        ; ------------------------------------- Block: 3 cycles
// 2961         //   break;
// 2962       }
// 2963       if((TOD_energy_config == 2) && (61 == element_filled) && (0x015e5b06 == obis_cd))
??compartment3_301:
        CMP       N:_TOD_energy_config, #0x2  ;; 1 cycle
        BNZ       ??compartment3_302  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        MOVW      AX, N:_element_filled  ;; 1 cycle
        CMPW      AX, #0x3D          ;; 1 cycle
        BNZ       ??compartment3_302  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x15E         ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x5B06        ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??buffer_scaler_filler_7:
        BNZ       ??compartment3_302  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
// 2964       {
// 2965         element_filled+= 8;
        MOVW      AX, N:_element_filled  ;; 1 cycle
        ADDW      AX, #0x8           ;; 1 cycle
        MOVW      N:_element_filled, AX  ;; 1 cycle
        BR        S:??compartment3_302  ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
// 2966         //  break;
// 2967       }
// 2968     }
// 2969     else
// 2970     {
// 2971       if((TOD_energy_config == 0) && (45 == element_filled) && (0x015e5b06 == obis_cd))
??compartment3_299:
        CMP0      N:_TOD_energy_config  ;; 1 cycle
        BNZ       ??compartment3_302  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        MOVW      AX, N:_element_filled  ;; 1 cycle
        CMPW      AX, #0x2D          ;; 1 cycle
        BNZ       ??compartment3_302  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x15E         ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x5B06        ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??buffer_scaler_filler_8:
        BNZ       ??compartment3_302  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
// 2972       {
// 2973         element_filled+= 24;
        MOVW      AX, N:_element_filled  ;; 1 cycle
        ADDW      AX, #0x18          ;; 1 cycle
        MOVW      N:_element_filled, AX  ;; 1 cycle
// 2974         break;
        BR        N:??compartment3_297  ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
// 2975       }
// 2976     }
// 2977     if((0 == KVAH_SNAP) && (10 == element_filled) && (0x015e5b07 == obis_cd))
??compartment3_302:
        CMP0      N:_KVAH_SNAP       ;; 1 cycle
        BNZ       ??compartment3_303  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        MOVW      AX, N:_element_filled  ;; 1 cycle
        CMPW      AX, #0xA           ;; 1 cycle
        BNZ       ??compartment3_303  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x15E         ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x5B07        ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??buffer_scaler_filler_9:
        BNZ       ??compartment3_303  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
// 2978     {
// 2979       element_filled++;
        INCW      N:_element_filled  ;; 2 cycles
// 2980       break;
        BR        N:??compartment3_297  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 2981     }
// 2982     if((D_KVARH_REQ != 1) && (3 == element_filled) && (0x015e5b05 == obis_cd))
??compartment3_303:
        CMP       N:_D_KVARH_REQ, #0x1  ;; 1 cycle
        BZ        ??compartment3_304  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        MOVW      AX, N:_element_filled  ;; 1 cycle
        CMPW      AX, #0x3           ;; 1 cycle
        BNZ       ??compartment3_304  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x15E         ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x5B05        ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??buffer_scaler_filler_10:
        BNZ       ??compartment3_304  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
// 2983     {
// 2984       element_filled++;
        INCW      N:_element_filled  ;; 2 cycles
// 2985       element_filled++;
        INCW      N:_element_filled  ;; 2 cycles
// 2986       break;
        BR        N:??compartment3_297  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 2987     }
// 2988     /**/
// 2989     
// 2990     /***/
// 2991     if((CUM_MAX_DEMAND != 1) && (22 == element_filled) && ((0x015e5b03 == obis_cd)))
??compartment3_304:
        CMP       N:_CUM_MAX_DEMAND, #0x1  ;; 1 cycle
        BZ        ??compartment3_305  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        MOVW      AX, N:_element_filled  ;; 1 cycle
        CMPW      AX, #0x16          ;; 1 cycle
        BNZ       ??compartment3_305  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        CMPW      AX, #0x15E         ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 7 cycles
        CMPW      AX, #0x5B03        ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??buffer_scaler_filler_11:
        BNZ       ??compartment3_305  ;; 4 cycles
        ; ------------------------------------- Block: 4 cycles
// 2992     {
// 2993       element_filled++;
        INCW      N:_element_filled  ;; 2 cycles
// 2994       element_filled++;
        INCW      N:_element_filled  ;; 2 cycles
// 2995       break;
        BR        N:??compartment3_297  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 2996     }
// 2997     
// 2998     buffer_filled_u16+= 6;
??compartment3_305:
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        ADDW      AX, #0x6           ;; 1 cycle
        MOVW      [SP+0x06], AX      ;; 1 cycle
// 2999     
// 3000     if(s_f[3 + element_filled * 3] == 2) /* for scalar_vol */
        MOVW      AX, N:_element_filled  ;; 1 cycle
        MOVW      BC, #0x3           ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [SP+0x08]      ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, [HL+0x03]       ;; 1 cycle
        CMP       A, #0x2            ;; 1 cycle
        BNZ       ??compartment3_306  ;; 4 cycles
        ; ------------------------------------- Block: 17 cycles
// 3001     {
// 3002       sca_unit(s_f[1 + element_filled * 3] + scalar_vol, s_f[1 + element_filled * 3 + 1], 0);
        MOV       C, #0x0            ;; 1 cycle
        XCH       A, C               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        XCH       A, C               ;; 1 cycle
        MOVW      AX, N:_element_filled  ;; 1 cycle
        MOVW      BC, #0x3           ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [SP+0x08]      ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, [HL+0x02]       ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       C, A               ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        XCH       A, C               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        XCH       A, C               ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      AX, N:_element_filled  ;; 1 cycle
        MOVW      BC, #0x3           ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [SP+0x08]      ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        INCW      AX                 ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, [HL]            ;; 1 cycle
        ADD       A, N:_scalar_vol   ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        XCH       A, E               ;; 1 cycle
        MOV       C, A               ;; 1 cycle
        XCH       A, E               ;; 1 cycle
          CFI FunCall _sca_unit
        CALL      _sca_unit          ;; 3 cycles
        BR        N:??compartment3_307  ;; 3 cycles
        ; ------------------------------------- Block: 46 cycles
// 3003     }
// 3004     else if(s_f[3 + element_filled * 3] == 1) /* for scalar_cur */
??compartment3_306:
        MOVW      AX, N:_element_filled  ;; 1 cycle
        MOVW      BC, #0x3           ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [SP+0x08]      ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, [HL+0x03]       ;; 1 cycle
        CMP       A, #0x1            ;; 1 cycle
        BNZ       ??compartment3_308  ;; 4 cycles
        ; ------------------------------------- Block: 14 cycles
// 3005     {
// 3006       sca_unit(s_f[1 + element_filled * 3] + scalar_cur, s_f[1 + element_filled * 3 + 1], 0);
        MOV       C, #0x0            ;; 1 cycle
        XCH       A, C               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        XCH       A, C               ;; 1 cycle
        MOVW      AX, N:_element_filled  ;; 1 cycle
        MOVW      BC, #0x3           ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [SP+0x08]      ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, [HL+0x02]       ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       C, A               ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        XCH       A, C               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        XCH       A, C               ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      AX, N:_element_filled  ;; 1 cycle
        MOVW      BC, #0x3           ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [SP+0x08]      ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        INCW      AX                 ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, [HL]            ;; 1 cycle
        ADD       A, N:_scalar_cur   ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        XCH       A, E               ;; 1 cycle
        MOV       C, A               ;; 1 cycle
        XCH       A, E               ;; 1 cycle
          CFI FunCall _sca_unit
        CALL      _sca_unit          ;; 3 cycles
        BR        N:??compartment3_307  ;; 3 cycles
        ; ------------------------------------- Block: 46 cycles
// 3007     }
// 3008     else if(s_f[3 + element_filled * 3] == 3) /* for scalar energy */
??compartment3_308:
        MOVW      AX, N:_element_filled  ;; 1 cycle
        MOVW      BC, #0x3           ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [SP+0x08]      ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, [HL+0x03]       ;; 1 cycle
        CMP       A, #0x3            ;; 1 cycle
        BNZ       ??compartment3_309  ;; 4 cycles
        ; ------------------------------------- Block: 14 cycles
// 3009     {
// 3010       sca_unit(s_f[1 + element_filled * 3] + scalar_energy, s_f[1 + element_filled * 3 + 1], 0);
        MOV       C, #0x0            ;; 1 cycle
        XCH       A, C               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        XCH       A, C               ;; 1 cycle
        MOVW      AX, N:_element_filled  ;; 1 cycle
        MOVW      BC, #0x3           ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [SP+0x08]      ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, [HL+0x02]       ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       C, A               ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        XCH       A, C               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        XCH       A, C               ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      AX, N:_element_filled  ;; 1 cycle
        MOVW      BC, #0x3           ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [SP+0x08]      ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        INCW      AX                 ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, [HL]            ;; 1 cycle
        ADD       A, N:_scalar_energy  ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        XCH       A, E               ;; 1 cycle
        MOV       C, A               ;; 1 cycle
        XCH       A, E               ;; 1 cycle
          CFI FunCall _sca_unit
        CALL      _sca_unit          ;; 3 cycles
        BR        S:??compartment3_307  ;; 3 cycles
        ; ------------------------------------- Block: 46 cycles
// 3011     }
// 3012     else
// 3013     {
// 3014       sca_unit(s_f[1 + element_filled * 3], s_f[1 + element_filled * 3 + 1], 0);
??compartment3_309:
        MOV       C, #0x0            ;; 1 cycle
        XCH       A, C               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        XCH       A, C               ;; 1 cycle
        MOVW      AX, N:_element_filled  ;; 1 cycle
        MOVW      BC, #0x3           ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [SP+0x08]      ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, [HL+0x02]       ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       C, A               ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        XCH       A, C               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        XCH       A, C               ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      AX, N:_element_filled  ;; 1 cycle
        MOVW      BC, #0x3           ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [SP+0x08]      ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        INCW      AX                 ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, [HL]            ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        XCH       A, E               ;; 1 cycle
        MOV       C, A               ;; 1 cycle
        XCH       A, E               ;; 1 cycle
          CFI FunCall _sca_unit
        CALL      _sca_unit          ;; 3 cycles
        ; ------------------------------------- Block: 42 cycles
// 3015     }
// 3016     
// 3017     if(DLMS_MAX_BUFF_SIZE < (buffer_filled_u16 + 6))
??compartment3_307:
        MOVW      AX, [SP+0x06]      ;; 1 cycle
        ADDW      AX, #0x6           ;; 1 cycle
        CMPW      AX, #0x201         ;; 1 cycle
        BNC       ??compartment3_297  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
// 3018     {
// 3019       break;
// 3020     }
// 3021   }
        INCW      N:_element_filled  ;; 2 cycles
        BR        N:??compartment3_288  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 3022   element_filled++;
??compartment3_297:
        INCW      N:_element_filled  ;; 2 cycles
// 3023   if(element_filled >= s_f[0])
        MOVW      AX, [SP+0x08]      ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, [HL]            ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      HL, N:_element_filled  ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BH        ??compartment3_310  ;; 4 cycles
        ; ------------------------------------- Block: 13 cycles
// 3024   {
// 3025     multi_filling_f= 0;
        MOV       N:_multi_filling_f, #0x0  ;; 1 cycle
// 3026     buffer_first_not_fill_f= 0;
        MOV       N:_buffer_first_not_fill_f, #0x0  ;; 1 cycle
// 3027     u8temp= 1;
        MOV       A, #0x1            ;; 1 cycle
        MOV       [SP+0x04], A       ;; 1 cycle
        ; ------------------------------------- Block: 4 cycles
// 3028   }
// 3029   
// 3030   info[6]= u8temp;
??compartment3_310:
        MOV       A, [SP+0x04]       ;; 1 cycle
        MOV       N:_info+6, A       ;; 1 cycle
// 3031   info[10]= block_no;
        MOVW      AX, N:_block_no    ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       N:_info+10, A      ;; 1 cycle
        XCH       A, X               ;; 1 cycle
// 3032   info[13]= (k - 15) / 256;
        MOVW      AX, N:_k           ;; 1 cycle
        ADDW      AX, #0xFFF1        ;; 1 cycle
        CLRB      X                  ;; 1 cycle
        MOV       N:_info+13, A      ;; 1 cycle
        XCH       A, X               ;; 1 cycle
// 3033   info[14]= (uint8_t)(k - 15);
        MOVW      AX, N:_k           ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        ADD       A, #0xF1           ;; 1 cycle
        MOV       N:_info+14, A      ;; 1 cycle
// 3034   
// 3035   block_no++;
        INCW      N:_block_no        ;; 2 cycles
// 3036   info_send= k;
        MOVW      AX, N:_k           ;; 1 cycle
        MOVW      N:_info_send, AX   ;; 1 cycle
// 3037   info_sended= 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_info_sended, AX  ;; 1 cycle
// 3038   send_type_multi();
          CFI FunCall _send_type_multi
        CALL      _send_type_multi   ;; 3 cycles
// 3039 }
        ADDW      SP, #0xA           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock5
        ; ------------------------------------- Block: 31 cycles
        ; ------------------------------------- Total: 706 cycles
// 3040 
// 3041 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock6 Using cfiCommon0
          CFI Function _compartment6
        CODE
// 3042 void compartment6()
// 3043 {
_compartment6:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 2
        SUBW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+6
// 3044   uint8_t u8temp= 0;
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
// 3045   
// 3046   k= 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_k, AX           ;; 1 cycle
// 3047   Start_Info2();
          CFI FunCall _Start_Info2
        CALL      _Start_Info2       ;; 3 cycles
// 3048   k= 15;
        MOVW      AX, #0xF           ;; 1 cycle
        MOVW      N:_k, AX           ;; 1 cycle
// 3049   
// 3050   
// 3051   if((1 == sel_access_flag) && ((1 == access_selector) || ((from_ptr != 1 || to_ptr != 1) && (2 == access_selector))))
        CMP       N:_sel_access_flag, #0x1  ;; 1 cycle
        BNZ       ??compartment3_311  ;; 4 cycles
        ; ------------------------------------- Block: 15 cycles
        CMP       N:_access_selector, #0x1  ;; 1 cycle
        BZ        ??compartment3_312  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
        ONEW      AX                 ;; 1 cycle
        CMPW      AX, N:_from_ptr    ;; 1 cycle
        BNZ       ??compartment3_313  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
        ONEW      AX                 ;; 1 cycle
        CMPW      AX, N:_to_ptr      ;; 1 cycle
        BZ        ??compartment3_311  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
??compartment3_313:
        CMP       N:_access_selector, #0x2  ;; 1 cycle
        BNZ       ??compartment3_311  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 3052   {
// 3053     array(0, 0);
??compartment3_312:
        MOV       X, #0x0            ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _array
        CALL      _array             ;; 3 cycles
        BR        S:??compartment3_314  ;; 3 cycles
        ; ------------------------------------- Block: 8 cycles
// 3054   }
// 3055   else
// 3056   {
// 3057     if(TOP_RESTORE_REQ == 0)
??compartment3_311:
        CMP0      N:_TOP_RESTORE_REQ  ;; 1 cycle
        BNZ       ??compartment3_315  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 3058     {
// 3059       if(bitIsSet(tpr.top_cover.flag,event_f))
        MOVW      HL, #LWRD(_tpr+592)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??compartment3_316  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 3060       {
// 3061         array(1, 0);
        MOV       X, #0x0            ;; 1 cycle
        MOV       A, #0x1            ;; 1 cycle
          CFI FunCall _array
        CALL      _array             ;; 3 cycles
        BR        S:??compartment3_317  ;; 3 cycles
        ; ------------------------------------- Block: 8 cycles
// 3062       }
// 3063       else
// 3064       {
// 3065         array(0, 0);
??compartment3_316:
        MOV       X, #0x0            ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _array
        CALL      _array             ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 3066       }
// 3067       
// 3068       if(bitIsSet(tpr.top_cover.flag,event_f))
??compartment3_317:
        MOVW      HL, #LWRD(_tpr+592)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BNC       ??compartment3_314  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 3069       {
// 3070         tpr_fill(COMPART_NONROLLOVER_START_ADD);
        MOVW      AX, N:_COMPART_NONROLLOVER_START_ADD  ;; 1 cycle
          CFI FunCall _tpr_fill
        CALL      _tpr_fill          ;; 3 cycles
        BR        S:??compartment3_314  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 3071       }
// 3072     }
// 3073     else
// 3074     {
// 3075       array(1, 0);
??compartment3_315:
        MOV       X, #0x0            ;; 1 cycle
        MOV       A, #0x1            ;; 1 cycle
          CFI FunCall _array
        CALL      _array             ;; 3 cycles
// 3076       tpr_fill(COMPART_NONROLLOVER_START_ADD);
        MOVW      AX, N:_COMPART_NONROLLOVER_START_ADD  ;; 1 cycle
          CFI FunCall _tpr_fill
        CALL      _tpr_fill          ;; 3 cycles
        ; ------------------------------------- Block: 9 cycles
// 3077     }
// 3078   }
// 3079   
// 3080   multi_filling_f= 0;
??compartment3_314:
        MOV       N:_multi_filling_f, #0x0  ;; 1 cycle
// 3081   buffer_first_not_fill_f= 0;
        MOV       N:_buffer_first_not_fill_f, #0x0  ;; 1 cycle
// 3082   u8temp= 1;
        MOV       A, #0x1            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
// 3083   send_data(u8temp);
        MOV       A, [SP]            ;; 1 cycle
          CFI FunCall _send_data
        CALL      _send_data         ;; 3 cycles
// 3084 }
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock6
        ; ------------------------------------- Block: 15 cycles
        ; ------------------------------------- Total: 106 cycles
// 3085 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock7 Using cfiCommon0
          CFI Function _compartment3
        CODE
// 3086 void compartment3(void)
// 3087 {
_compartment3:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 12
        SUBW      SP, #0xC           ;; 1 cycle
          CFI CFA SP+16
// 3088   uint16_t buffer_filled_u16= 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [SP+0x02], AX      ;; 1 cycle
// 3089   uint8_t u8temp= 0;
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
// 3090   /*	uint16_t temp_data; */
// 3091   
// 3092   k= 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_k, AX           ;; 1 cycle
// 3093   Start_Info2();
          CFI FunCall _Start_Info2
        CALL      _Start_Info2       ;; 3 cycles
// 3094   k= 15;
        MOVW      AX, #0xF           ;; 1 cycle
        MOVW      N:_k, AX           ;; 1 cycle
// 3095   
// 3096   buffer_filled_u16= k;
        MOVW      AX, N:_k           ;; 1 cycle
        MOVW      [SP+0x02], AX      ;; 1 cycle
// 3097   
// 3098   if(buffer_first_not_fill_f == 0)
        CMP0      N:_buffer_first_not_fill_f  ;; 1 cycle
        SKZ                          ;; 4 cycles
        BR        N:??compartment3_318  ;; 4 cycles
        ; ------------------------------------- Block: 19 cycles
// 3099   {
// 3100     /*	tamper_data=opr_data[0]; */
// 3101     tamper_data= (tpr.power_count % COMPART_POWERFAIL_ENTRIES) * 2;
        MOV       X, N:_COMPART_POWERFAIL_ENTRIES  ;; 1 cycle
        MOV       A, N:_tpr+8        ;; 1 cycle
          CFI FunCall ?UC_MOD_L01
        CALL      N:?UC_MOD_L01      ;; 3 cycles
        MOV       C, A               ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOVW      AX, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      N:_tamper_data, AX  ;; 1 cycle
// 3102     if(1 == tpr.power_overflow) /* opr_data[1]) */
        CMP       N:_tpr+9, #0x1     ;; 1 cycle
        BNZ       ??compartment3_319  ;; 4 cycles
        ; ------------------------------------- Block: 16 cycles
// 3103     {
// 3104       compart1= COMPART_POWERFAIL_ENTRIES;
        MOV       X, N:_COMPART_POWERFAIL_ENTRIES  ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      N:_compart1, AX    ;; 1 cycle
        BR        S:??compartment3_320  ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
// 3105     }
// 3106     else
// 3107     {
// 3108       compart1= tpr.power_count; /* tamper_data; */
??compartment3_319:
        MOV       X, N:_tpr+8        ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      N:_compart1, AX    ;; 1 cycle
// 3109       tamper_data= 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_tamper_data, AX  ;; 1 cycle
        ; ------------------------------------- Block: 5 cycles
// 3110     }
// 3111     
// 3112     compart1*= 2;
??compartment3_320:
        MOVW      AX, N:_compart1    ;; 1 cycle
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      N:_compart1, AX    ;; 1 cycle
// 3113     if(sel_access_flag == 1)
        CMP       N:_sel_access_flag, #0x1  ;; 1 cycle
        SKZ                          ;; 4 cycles
        BR        N:??compartment3_321  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
// 3114     {
// 3115       if(1 == tpr.power_overflow)
        CMP       N:_tpr+9, #0x1     ;; 1 cycle
        BNZ       ??compartment3_322  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 3116       {
// 3117         tamper_data= (tpr.power_count % COMPART_POWERFAIL_ENTRIES) * 2;
        MOV       X, N:_COMPART_POWERFAIL_ENTRIES  ;; 1 cycle
        MOV       A, N:_tpr+8        ;; 1 cycle
          CFI FunCall ?UC_MOD_L01
        CALL      N:?UC_MOD_L01      ;; 3 cycles
        MOV       C, A               ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOVW      AX, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      N:_tamper_data, AX  ;; 1 cycle
        BR        S:??compartment3_323  ;; 3 cycles
        ; ------------------------------------- Block: 14 cycles
// 3118       }
// 3119       else
// 3120       {
// 3121         tamper_data= 0;
??compartment3_322:
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_tamper_data, AX  ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
// 3122       }
// 3123       
// 3124       /* temp_data=(COMPART_POWERFAIL_ENTRIES*2)+(pow_off_cnt%COMPART_POWERFAIL_ENTRIES)*2; */
// 3125       if((compart1 < to_ptr) || (compart1 < from_ptr) || (access_selector != 2))
??compartment3_323:
        MOVW      HL, N:_to_ptr      ;; 1 cycle
        MOVW      AX, N:_compart1    ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BC        ??compartment3_324  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOVW      HL, N:_from_ptr    ;; 1 cycle
        MOVW      AX, N:_compart1    ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BC        ??compartment3_324  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        CMP       N:_access_selector, #0x2  ;; 1 cycle
        BZ        ??compartment3_325  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 3126       {
// 3127         compart1= 0;
??compartment3_324:
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_compart1, AX    ;; 1 cycle
        BR        S:??compartment3_321  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 3128       }
// 3129       else
// 3130       {
// 3131         if(to_ptr == 0)
??compartment3_325:
        CLRW      AX                 ;; 1 cycle
        CMPW      AX, N:_to_ptr      ;; 1 cycle
        BNZ       ??compartment3_326  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 3132         {
// 3133           compart1= compart1 - from_ptr + 1;
        MOVW      AX, N:_compart1    ;; 1 cycle
        SUBW      AX, N:_from_ptr    ;; 1 cycle
        INCW      AX                 ;; 1 cycle
        MOVW      N:_compart1, AX    ;; 1 cycle
        BR        S:??compartment3_327  ;; 3 cycles
        ; ------------------------------------- Block: 7 cycles
// 3134         }
// 3135         else
// 3136         {
// 3137           compart1= to_ptr - from_ptr + 1;
??compartment3_326:
        MOVW      AX, N:_to_ptr      ;; 1 cycle
        SUBW      AX, N:_from_ptr    ;; 1 cycle
        INCW      AX                 ;; 1 cycle
        MOVW      N:_compart1, AX    ;; 1 cycle
        ; ------------------------------------- Block: 4 cycles
// 3138         }
// 3139         
// 3140         if((from_ptr > to_ptr) && (to_ptr != 0))
??compartment3_327:
        MOVW      HL, N:_from_ptr    ;; 1 cycle
        MOVW      AX, N:_to_ptr      ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BNC       ??compartment3_328  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        CLRW      AX                 ;; 1 cycle
        CMPW      AX, N:_to_ptr      ;; 1 cycle
        BZ        ??compartment3_328  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 3141         {
// 3142           compart1= 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_compart1, AX    ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
// 3143         }
// 3144         
// 3145         if(from_ptr == 0)
??compartment3_328:
        CLRW      AX                 ;; 1 cycle
        CMPW      AX, N:_from_ptr    ;; 1 cycle
        BNZ       ??compartment3_329  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 3146         {
// 3147           compart1= 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_compart1, AX    ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
// 3148         }
// 3149         
// 3150         tamper_data= tamper_data + from_ptr - 1;
??compartment3_329:
        MOVW      AX, N:_from_ptr    ;; 1 cycle
        DECW      AX                 ;; 1 cycle
        ADDW      AX, N:_tamper_data  ;; 1 cycle
        MOVW      N:_tamper_data, AX  ;; 1 cycle
        ; ------------------------------------- Block: 4 cycles
// 3151         /* temp_data = temp_data + from_ptr-1; */
// 3152         /*         tamper_data =from_ptr - 1 + long_data; */
// 3153         
// 3154         while(tamper_data >= (COMPART_POWERFAIL_ENTRIES * 2))
??compartment3_330:
        MOV       X, N:_COMPART_POWERFAIL_ENTRIES  ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      HL, N:_tamper_data  ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BH        ??compartment3_321  ;; 4 cycles
        ; ------------------------------------- Block: 11 cycles
// 3155         {
// 3156           /*	while(temp_data>=(COMPART_POWERFAIL_ENTRIES*2)) */
// 3157           tamper_data= tamper_data - (COMPART_POWERFAIL_ENTRIES * 2);
        MOV       X, N:_COMPART_POWERFAIL_ENTRIES  ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, N:_tamper_data  ;; 1 cycle
        SUBW      AX, HL             ;; 1 cycle
        MOVW      N:_tamper_data, AX  ;; 1 cycle
        BR        S:??compartment3_330  ;; 3 cycles
        ; ------------------------------------- Block: 12 cycles
// 3158           /* temp_data = temp_data-(COMPART_POWERFAIL_ENTRIES*2); */
// 3159         }
// 3160         /* tamper_data=temp_data; */
// 3161       }
// 3162     }
// 3163     
// 3164     /*		Start_Info(); */
// 3165     dlms_address= COMPART_POWERFAIL_START_ADD + ((uint16_t)(tamper_data / 2) * 0x10);
??compartment3_321:
        MOVW      AX, N:_tamper_data  ;; 1 cycle
        SHRW      AX, 0x1            ;; 1 cycle
        MOVW      BC, #0x10          ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, N:_COMPART_POWERFAIL_START_ADD  ;; 1 cycle
        MOVW      N:_dlms_address, AX  ;; 1 cycle
// 3166     /*		array(compart1,0); */
// 3167     array(0x82,0);
        MOV       X, #0x0            ;; 1 cycle
        MOV       A, #0x82           ;; 1 cycle
          CFI FunCall _array
        CALL      _array             ;; 3 cycles
// 3168     info[k++]= compart1 / 256;
        MOVW      AX, N:_compart1    ;; 1 cycle
        CLRB      X                  ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
// 3169     info[k++]= compart1 % 256;
        MOVW      AX, N:_compart1    ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOVW      BC, N:_k           ;; 1 cycle
        MOV       (_info)[BC], A     ;; 1 cycle
        INCW      N:_k               ;; 2 cycles
// 3170     
// 3171     block_no= 1;
        MOVW      AX, #0x1           ;; 1 cycle
        MOVW      N:_block_no, AX    ;; 1 cycle
// 3172     
// 3173     element_filled= 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_element_filled, AX  ;; 1 cycle
// 3174     multi_filling_f= 1;
        MOV       N:_multi_filling_f, #0x1  ;; 1 cycle
// 3175     buffer_first_not_fill_f= 1;
        MOV       N:_buffer_first_not_fill_f, #0x1  ;; 1 cycle
// 3176     
// 3177     if(sel_access_flag == 1)
        CMP       N:_sel_access_flag, #0x1  ;; 1 cycle
        BNZ       ??compartment3_331  ;; 4 cycles
        ; ------------------------------------- Block: 37 cycles
// 3178     {
// 3179       if(no_obj < 2)
        MOV       A, N:_no_obj       ;; 1 cycle
        CMP       A, #0x2            ;; 1 cycle
        BNC       ??compartment3_332  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 3180       {
// 3181         selective_values_byte= 16;
        MOV       N:_selective_values_byte, #0x10  ;; 1 cycle
        BR        S:??compartment3_333  ;; 3 cycles
        ; ------------------------------------- Block: 4 cycles
// 3182       }
// 3183       else
// 3184       {
// 3185         no_obj= 2;
??compartment3_332:
        MOV       N:_no_obj, #0x2    ;; 1 cycle
// 3186         selective_values_byte= 19;
        MOV       N:_selective_values_byte, #0x13  ;; 1 cycle
        BR        S:??compartment3_333  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 3187       }
// 3188     }
// 3189     else
// 3190     {
// 3191       selective_values_byte= 19 + 2 + 19;
??compartment3_331:
        MOV       N:_selective_values_byte, #0x28  ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
// 3192     }
// 3193     
// 3194     if(compart1 == 0)
??compartment3_333:
        CLRW      AX                 ;; 1 cycle
        CMPW      AX, N:_compart1    ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??compartment3_334  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
// 3195     {
// 3196       goto comp1;
// 3197     }
// 3198   }
// 3199   
// 3200   for(; element_filled < compart1; element_filled++)
??compartment3_318:
        MOVW      HL, N:_compart1    ;; 1 cycle
        MOVW      AX, N:_element_filled  ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        SKC                          ;; 4 cycles
        BR        N:??compartment3_334  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
// 3201   {
// 3202     buffer_filled_u16+= selective_values_byte;
        MOV       C, N:_selective_values_byte  ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        ADDW      AX, BC             ;; 1 cycle
        MOVW      [SP+0x02], AX      ;; 1 cycle
// 3203     
// 3204     if(dlms_address == COMPART_POWERFAIL_END_ADD)
        MOVW      HL, N:_COMPART_POWERFAIL_END_ADD  ;; 1 cycle
        MOVW      AX, N:_dlms_address  ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BNZ       ??compartment3_335  ;; 4 cycles
        ; ------------------------------------- Block: 12 cycles
// 3205     {
// 3206       dlms_address = COMPART_POWERFAIL_START_ADD;
        MOVW      AX, N:_COMPART_POWERFAIL_START_ADD  ;; 1 cycle
        MOVW      N:_dlms_address, AX  ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
// 3207     }
// 3208     
// 3209     eprom_read(dlms_address, 0,PAGE_1,AUTO_CALC);
??compartment3_335:
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, N:_dlms_address  ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
// 3210     if(sel_access_flag != 1)
        CMP       N:_sel_access_flag, #0x1  ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??compartment3_336  ;; 4 cycles
        ; ------------------------------------- Block: 12 cycles
// 3211     {
// 3212       structure(2); /* 19 */
        MOV       A, #0x2            ;; 1 cycle
          CFI FunCall _structure
        CALL      _structure         ;; 3 cycles
// 3213       TempTime = char_array_into_time5_sec(&opr_data[0]);
        MOVW      BC, #LWRD(_opr_data)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
          CFI FunCall _char_array_into_time5_sec
        CALL      _char_array_into_time5_sec  ;; 3 cycles
        MOVW      HL, #LWRD(_TempTime)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOV       A, [DE]            ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
        MOV       A, [DE+0x01]       ;; 1 cycle
        MOV       [HL+0x01], A       ;; 1 cycle
        MOV       A, [DE+0x02]       ;; 1 cycle
        MOV       [HL+0x02], A       ;; 1 cycle
        MOV       A, [DE+0x03]       ;; 1 cycle
        MOV       [HL+0x03], A       ;; 1 cycle
        MOV       A, [DE+0x04]       ;; 1 cycle
        MOV       [HL+0x04], A       ;; 1 cycle
        MOV       A, [DE+0x05]       ;; 1 cycle
        MOV       [HL+0x05], A       ;; 1 cycle
        MOV       A, [DE+0x06]       ;; 1 cycle
        MOV       [HL+0x06], A       ;; 1 cycle
// 3214       date_time(TempTime.day,TempTime.month,TempTime.year,TempTime.hour,TempTime.min,TempTime.sec,0);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+18
        MOV       A, N:_TempTime     ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, N:_TempTime+1   ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       B, N:_TempTime+2   ;; 1 cycle
        MOV       C, N:_TempTime+6   ;; 1 cycle
        MOV       X, N:_TempTime+5   ;; 1 cycle
        MOV       A, N:_TempTime+3   ;; 1 cycle
          CFI FunCall _date_time
        CALL      _date_time         ;; 3 cycles
// 3215       val_2byt(EVENT_ID_POWER_FAIL_OCC / 256, EVENT_ID_POWER_FAIL_OCC%256);
        MOV       X, #0x65           ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_2byt
        CALL      _val_2byt          ;; 3 cycles
// 3216       
// 3217       
// 3218       structure(2);
        MOV       A, #0x2            ;; 1 cycle
          CFI FunCall _structure
        CALL      _structure         ;; 3 cycles
// 3219       TempTime = char_array_into_time5_sec(&opr_data[7]);
        MOVW      BC, #LWRD(_opr_data+7)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x6           ;; 1 cycle
          CFI FunCall _char_array_into_time5_sec
        CALL      _char_array_into_time5_sec  ;; 3 cycles
        MOVW      HL, #LWRD(_TempTime)  ;; 1 cycle
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+16
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOV       A, [DE]            ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
        MOV       A, [DE+0x01]       ;; 1 cycle
        MOV       [HL+0x01], A       ;; 1 cycle
        MOV       A, [DE+0x02]       ;; 1 cycle
        MOV       [HL+0x02], A       ;; 1 cycle
        MOV       A, [DE+0x03]       ;; 1 cycle
        MOV       [HL+0x03], A       ;; 1 cycle
        MOV       A, [DE+0x04]       ;; 1 cycle
        MOV       [HL+0x04], A       ;; 1 cycle
        MOV       A, [DE+0x05]       ;; 1 cycle
        MOV       [HL+0x05], A       ;; 1 cycle
        MOV       A, [DE+0x06]       ;; 1 cycle
        MOV       [HL+0x06], A       ;; 1 cycle
// 3220       date_time(TempTime.day,TempTime.month,TempTime.year,TempTime.hour,TempTime.min,TempTime.sec,0);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+18
        MOV       A, N:_TempTime     ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, N:_TempTime+1   ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       B, N:_TempTime+2   ;; 1 cycle
        MOV       C, N:_TempTime+6   ;; 1 cycle
        MOV       X, N:_TempTime+5   ;; 1 cycle
        MOV       A, N:_TempTime+3   ;; 1 cycle
          CFI FunCall _date_time
        CALL      _date_time         ;; 3 cycles
// 3221       val_2byt(EVENT_ID_POWER_FAIL_RES / 256, EVENT_ID_POWER_FAIL_RES%256);
        MOV       X, #0x66           ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_2byt
        CALL      _val_2byt          ;; 3 cycles
// 3222       element_filled++;
        INCW      N:_element_filled  ;; 2 cycles
// 3223       dlms_address= dlms_address + 0x10;
        MOVW      AX, N:_dlms_address  ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
        MOVW      N:_dlms_address, AX  ;; 1 cycle
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+16
        BR        N:??compartment3_337  ;; 3 cycles
        ; ------------------------------------- Block: 102 cycles
// 3224     }
// 3225     else
// 3226     {
// 3227       structure(no_obj); /* 19 */
??compartment3_336:
        MOV       A, N:_no_obj       ;; 1 cycle
          CFI FunCall _structure
        CALL      _structure         ;; 3 cycles
// 3228       
// 3229       eprom_read(dlms_address, 0,PAGE_1,AUTO_CALC);
        MOV       E, #0x0            ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       C, #0x0            ;; 1 cycle
        MOVW      AX, N:_dlms_address  ;; 1 cycle
          CFI FunCall _eprom_read
        CALL      _eprom_read        ;; 3 cycles
// 3230       
// 3231       if((tamper_data % 2) == 0)
        MOVW      HL, #LWRD(_tamper_data)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        BC        ??compartment3_338  ;; 4 cycles
        ; ------------------------------------- Block: 17 cycles
// 3232       {
// 3233         if(sel_obj_tamper[0] == 1)
        CMP       N:_sel_obj_tamper, #0x1  ;; 1 cycle
        BNZ       ??compartment3_339  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 3234         {
// 3235           TempTime = char_array_into_time5_sec(&opr_data[0]);
        MOVW      BC, #LWRD(_opr_data)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
          CFI FunCall _char_array_into_time5_sec
        CALL      _char_array_into_time5_sec  ;; 3 cycles
        MOVW      HL, #LWRD(_TempTime)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOV       A, [DE]            ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
        MOV       A, [DE+0x01]       ;; 1 cycle
        MOV       [HL+0x01], A       ;; 1 cycle
        MOV       A, [DE+0x02]       ;; 1 cycle
        MOV       [HL+0x02], A       ;; 1 cycle
        MOV       A, [DE+0x03]       ;; 1 cycle
        MOV       [HL+0x03], A       ;; 1 cycle
        MOV       A, [DE+0x04]       ;; 1 cycle
        MOV       [HL+0x04], A       ;; 1 cycle
        MOV       A, [DE+0x05]       ;; 1 cycle
        MOV       [HL+0x05], A       ;; 1 cycle
        MOV       A, [DE+0x06]       ;; 1 cycle
        MOV       [HL+0x06], A       ;; 1 cycle
// 3236           date_time(TempTime.day,TempTime.month,TempTime.year,TempTime.hour,TempTime.min,TempTime.sec,0);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+18
        MOV       A, N:_TempTime     ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, N:_TempTime+1   ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       B, N:_TempTime+2   ;; 1 cycle
        MOV       C, N:_TempTime+6   ;; 1 cycle
        MOV       X, N:_TempTime+5   ;; 1 cycle
        MOV       A, N:_TempTime+3   ;; 1 cycle
          CFI FunCall _date_time
        CALL      _date_time         ;; 3 cycles
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+16
        ; ------------------------------------- Block: 38 cycles
// 3237         }
// 3238         
// 3239         if(sel_obj_tamper[1] == 1)
??compartment3_339:
        CMP       N:_sel_obj_tamper+1, #0x1  ;; 1 cycle
        BNZ       ??compartment3_340  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 3240         {
// 3241           val_2byt(EVENT_ID_POWER_FAIL_OCC / 256, EVENT_ID_POWER_FAIL_OCC%256);
        MOV       X, #0x65           ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_2byt
        CALL      _val_2byt          ;; 3 cycles
        BR        S:??compartment3_340  ;; 3 cycles
        ; ------------------------------------- Block: 8 cycles
// 3242         }
// 3243       }
// 3244       else
// 3245       {
// 3246         if(sel_obj_tamper[0] == 1)
??compartment3_338:
        CMP       N:_sel_obj_tamper, #0x1  ;; 1 cycle
        BNZ       ??compartment3_341  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 3247         {
// 3248           TempTime = char_array_into_time5_sec(&opr_data[7]);
        MOVW      BC, #LWRD(_opr_data+7)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
          CFI FunCall _char_array_into_time5_sec
        CALL      _char_array_into_time5_sec  ;; 3 cycles
        MOVW      HL, #LWRD(_TempTime)  ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOV       A, [DE]            ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
        MOV       A, [DE+0x01]       ;; 1 cycle
        MOV       [HL+0x01], A       ;; 1 cycle
        MOV       A, [DE+0x02]       ;; 1 cycle
        MOV       [HL+0x02], A       ;; 1 cycle
        MOV       A, [DE+0x03]       ;; 1 cycle
        MOV       [HL+0x03], A       ;; 1 cycle
        MOV       A, [DE+0x04]       ;; 1 cycle
        MOV       [HL+0x04], A       ;; 1 cycle
        MOV       A, [DE+0x05]       ;; 1 cycle
        MOV       [HL+0x05], A       ;; 1 cycle
        MOV       A, [DE+0x06]       ;; 1 cycle
        MOV       [HL+0x06], A       ;; 1 cycle
// 3249           date_time(TempTime.day,TempTime.month,TempTime.year,TempTime.hour,TempTime.min,TempTime.sec,0);
        MOV       X, #0x0            ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+18
        MOV       A, N:_TempTime     ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, N:_TempTime+1   ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       B, N:_TempTime+2   ;; 1 cycle
        MOV       C, N:_TempTime+6   ;; 1 cycle
        MOV       X, N:_TempTime+5   ;; 1 cycle
        MOV       A, N:_TempTime+3   ;; 1 cycle
          CFI FunCall _date_time
        CALL      _date_time         ;; 3 cycles
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+16
        ; ------------------------------------- Block: 38 cycles
// 3250         }
// 3251         
// 3252         if(sel_obj_tamper[1] == 1)
??compartment3_341:
        CMP       N:_sel_obj_tamper+1, #0x1  ;; 1 cycle
        BNZ       ??compartment3_342  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
// 3253         {
// 3254           val_2byt(EVENT_ID_POWER_FAIL_RES / 256, EVENT_ID_POWER_FAIL_RES%256);
        MOV       X, #0x66           ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _val_2byt
        CALL      _val_2byt          ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 3255         }
// 3256         dlms_address= dlms_address + 0x10;
??compartment3_342:
        MOVW      AX, N:_dlms_address  ;; 1 cycle
        ADDW      AX, #0x10          ;; 1 cycle
        MOVW      N:_dlms_address, AX  ;; 1 cycle
        ; ------------------------------------- Block: 3 cycles
// 3257       }
// 3258       
// 3259       tamper_data++;
??compartment3_340:
        INCW      N:_tamper_data     ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
// 3260     }
// 3261     
// 3262     if(DLMS_MAX_BUFF_SIZE < (buffer_filled_u16 + selective_values_byte * 2))
??compartment3_337:
        MOV       X, N:_selective_values_byte  ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x2           ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [SP+0x02]      ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        CMPW      AX, #0x201         ;; 1 cycle
        BNC       ??compartment3_334  ;; 4 cycles
        ; ------------------------------------- Block: 13 cycles
// 3263     {
// 3264       break;
// 3265     }
// 3266   }
        INCW      N:_element_filled  ;; 2 cycles
        BR        N:??compartment3_318  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
// 3267   
// 3268 comp1:
// 3269   element_filled++;
??compartment3_334:
        INCW      N:_element_filled  ;; 2 cycles
// 3270   if(element_filled >= compart1)
        MOVW      HL, N:_compart1    ;; 1 cycle
        MOVW      AX, N:_element_filled  ;; 1 cycle
        CMPW      AX, HL             ;; 1 cycle
        BC        ??compartment3_343  ;; 4 cycles
        ; ------------------------------------- Block: 9 cycles
// 3271   {
// 3272     multi_filling_f= 0;
        MOV       N:_multi_filling_f, #0x0  ;; 1 cycle
// 3273     buffer_first_not_fill_f= 0;
        MOV       N:_buffer_first_not_fill_f, #0x0  ;; 1 cycle
// 3274     u8temp= 1;
        MOV       A, #0x1            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        ; ------------------------------------- Block: 4 cycles
// 3275   }
// 3276   
// 3277   send_data(u8temp);
??compartment3_343:
        MOV       A, [SP]            ;; 1 cycle
          CFI FunCall _send_data
        CALL      _send_data         ;; 3 cycles
// 3278 }
        ADDW      SP, #0xC           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock7
        ; ------------------------------------- Block: 11 cycles
        ; ------------------------------------- Total: 535 cycles

        SECTION `.iar_vfe_header`:DATA:NOALLOC:NOROOT(1)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
        DC32 0

        END
// 
//  2'285 bytes in section .bss
//  2'503 bytes in section .data
// 15'101 bytes in section .text
// 
// 15'101 bytes of CODE memory
//  4'788 bytes of DATA memory
//
//Errors: none
//Warnings: none
