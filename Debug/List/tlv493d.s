///////////////////////////////////////////////////////////////////////////////
//
// IAR C/C++ Compiler V4.20.1.2260 for RL78               20/Dec/2020  22:39:11
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
//        BootCode\source_code\source_files\tlv493d.c
//    Command line       =
//        -f C:\Users\laptop\AppData\Local\Temp\EW1E93.tmp ("D:\Dheeraj\New
//        folder\0. GDEV72 - BootCode\source_code\source_files\tlv493d.c"
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
//        D:\Dheeraj\New folder\0. GDEV72 - BootCode\Debug\List\tlv493d.s
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

        EXTERN ?F_DIV
        EXTERN ?F_F2SL
        EXTERN ?F_MUL
        EXTERN ?F_SL2F
        EXTERN ?F_UL2F
        EXTERN ?MEMCPY_SMALL_NEAR
        EXTERN ?SI_CMP_L02
        EXTERN ?SL_DIV_L03
        EXTERN _delay_us
        EXTERN _sqrt
        EXTERN _temp_us16
        EXTERN _temp_us32

        PUBLIC __A_P0
        PUBLIC __A_PM0
        PUBLIC __A_POM0
        PUBLIC _tlv
        PUBLIC _tlv493d_init
        PUBLIC _tlv493d_powerdown
        PUBLIC _tlv493d_scan
        PUBLIC _tlv_cal_max_min_counts
        PUBLIC _tlv_clear_var
        PUBLIC _tlv_configure
        PUBLIC _tlv_detect_ac_dc_field
        PUBLIC _tlv_flag
        PUBLIC _tlv_i2c_byte_read
        PUBLIC _tlv_i2c_byte_write
        PUBLIC _tlv_i2c_restart
        PUBLIC _tlv_i2c_start
        PUBLIC _tlv_i2c_stop
        PUBLIC _tlv_read_reg
        PUBLIC _tlv_recovery
        PUBLIC _tlv_reset
        
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
        
// D:\Dheeraj\New folder\0. GDEV72 - BootCode\source_code\source_files\tlv493d.c
//    1 /***********************************************************************************************************************
//    2 * File Name       : tlv493d.c
//    3 * Current Version : rev_01  
//    4 * Tool-Chain      : IAR Systems
//    5 * Description     : This file includes routines to enable and disable Watchdog timer
//    6 * Creation Date   : 08-04-2020
//    7 * Company         : Genus Power Infrastructures Limited, Jaipur
//    8 * Author          : dheeraj.singhal
//    9 * Version History : rev_01 : new source file created with routine to operate I2C basec TLV493D magnet sensor 
//   10 ***********************************************************************************************************************/
//   11 /************************************ Includes **************************************/
//   12 #include "tlv493d.h"

        ASEGN `.sbss.noinit`:DATA:NOROOT,0fff00H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_P0
// __no_init union <unnamed>#3 volatile __saddr _A_P0
__A_P0:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0fff20H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_PM0
// __no_init union <unnamed>#35 volatile __sfr _A_PM0
__A_PM0:
        DS 1

        ASEGN `.bss.noinit`:DATA:NOROOT,0f0050H
        SECTION_TYPE SHT_IAR_NOINIT, SHF_WRITE
        SECTION_GROUP __A_POM0
// __no_init union <unnamed>#246 volatile _A_POM0
__A_POM0:
        DS 1
//   13 
//   14 /************************************ Local Variables *****************************************/
//   15 /************************************ Extern Variables *****************************************/

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   16 TLV tlv;
_tlv:
        DS 76

        SECTION `.bss`:DATA:REORDER:NOROOT(0)
//   17 flag_union tlv_flag;
_tlv_flag:
        DS 1
//   18 /************************************ Local Functions *******************************/
//   19 /* I2C */
//   20 void tlv_i2c_start();
//   21 void tlv_i2c_stop();
//   22 void tlv_i2c_restart();
//   23 void tlv_i2c_byte_read(us8 *iic_data, us8 ack_data);
//   24 us8 tlv_i2c_byte_write(us8 iic_data);
//   25 //void tlv_i2c_delay();
//   26 
//   27 /* others */
//   28 us8 tlv_read_reg();
//   29 us8 tlv_configure();
//   30 us8 tlv_recovery();
//   31 us8 tlv_reset();
//   32 void tlv_clear_var();
//   33 void tlv_cal_max_min_counts();
//   34 void tlv_detect_ac_dc_field();
//   35 /************************************ Extern Functions ******************************/
//   36 us8 tlv493d_init();
//   37 void tlv493d_scan();
//   38 void tlv493d_powerdown();
//   39 
//   40 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock0 Using cfiCommon0
          CFI Function _tlv_read_reg
        CODE
//   41 us8 tlv_read_reg()
//   42 {
_tlv_read_reg:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 2
        SUBW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+6
//   43     tlv_i2c_start();
          CFI FunCall _tlv_i2c_start
        CALL      _tlv_i2c_start     ;; 3 cycles
//   44     if(flag_tlv_bus_busy == 0)
        MOVW      HL, #LWRD(_tlv_flag)  ;; 1 cycle
        MOV1      CY, [HL].3         ;; 1 cycle
        BC        ??tlv_detect_ac_dc_field_0  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
//   45     {
//   46         if((tlv_i2c_byte_write(ADDR_MAG_R)) == MAG_ACK)  
        MOV       A, #0xBD           ;; 1 cycle
          CFI FunCall _tlv_i2c_byte_write
        CALL      _tlv_i2c_byte_write  ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??tlv_detect_ac_dc_field_1  ;; 4 cycles
        ; ------------------------------------- Block: 9 cycles
//   47 	{
//   48 	    for(us8 index = 0; index < 9; index++)
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
??tlv_read_reg_0:
        MOV       A, [SP]            ;; 1 cycle
        CMP       A, #0x9            ;; 1 cycle
        BNC       ??tlv_detect_ac_dc_field_2  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//   49 	    {
//   50 		tlv_i2c_byte_read(&tlv.sensor_reg_read[index],MAG_ACK);
        MOV       C, #0x0            ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        ADDW      AX, #LWRD(_tlv+12)  ;; 1 cycle
          CFI FunCall _tlv_i2c_byte_read
        CALL      _tlv_i2c_byte_read  ;; 3 cycles
//   51 	    }
        MOV       A, [SP]            ;; 1 cycle
        INC       A                  ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        BR        S:??tlv_read_reg_0  ;; 3 cycles
        ; ------------------------------------- Block: 14 cycles
//   52 	}
//   53 	else
//   54 	{
//   55 	    tlv_i2c_stop();
//   56             return MAG_I2C_ERROR;
//   57 	}
//   58 	tlv_i2c_byte_read(&tlv.sensor_reg_read[9],MAG_NACK);
??tlv_detect_ac_dc_field_2:
        MOV       C, #0x1            ;; 1 cycle
        MOVW      AX, #LWRD(_tlv+21)  ;; 1 cycle
          CFI FunCall _tlv_i2c_byte_read
        CALL      _tlv_i2c_byte_read  ;; 3 cycles
//   59 	tlv_i2c_stop();
          CFI FunCall _tlv_i2c_stop
        CALL      _tlv_i2c_stop      ;; 3 cycles
//   60 	memcpy(&tlv.sensor_reg_write[0],&tlv.sensor_reg_read[7],3);
        MOVW      DE, #LWRD(_tlv+19)  ;; 1 cycle
        MOVW      HL, #LWRD(_tlv+22)  ;; 1 cycle
        MOV       B, #0x3            ;; 1 cycle
          CFI FunCall ?MEMCPY_SMALL_NEAR
        CALL      N:?MEMCPY_SMALL_NEAR  ;; 3 cycles
//   61         return MAG_I2C_OK;
        MOV       A, #0x0            ;; 1 cycle
        BR        S:??tlv_detect_ac_dc_field_3  ;; 3 cycles
          CFI FunCall _tlv_i2c_stop
        ; ------------------------------------- Block: 18 cycles
??tlv_detect_ac_dc_field_1:
        CALL      _tlv_i2c_stop      ;; 3 cycles
        MOV       A, #0x1            ;; 1 cycle
        BR        S:??tlv_detect_ac_dc_field_3  ;; 3 cycles
          CFI FunCall _tlv_i2c_stop
        ; ------------------------------------- Block: 7 cycles
//   62     }
//   63     tlv_i2c_stop();
??tlv_detect_ac_dc_field_0:
        CALL      _tlv_i2c_stop      ;; 3 cycles
//   64     return MAG_I2C_ERROR;
        MOV       A, #0x1            ;; 1 cycle
        ; ------------------------------------- Block: 4 cycles
??tlv_detect_ac_dc_field_3:
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock0
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 77 cycles
//   65 }
//   66 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock1 Using cfiCommon0
          CFI Function _tlv_configure
        CODE
//   67 us8 tlv_configure()
//   68 {
_tlv_configure:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//   69     tlv.sensor_reg_write[0] = 0x00; 				                // reser val 00
        MOV       N:_tlv+22, #0x0    ;; 1 cycle
//   70     tlv.sensor_reg_write[2] = tlv.sensor_reg_read[1];                              //8h 7-0
        MOV       A, N:_tlv+13       ;; 1 cycle
        MOV       N:_tlv+24, A       ;; 1 cycle
//   71     
//   72     /* Low power mode 100Hz without interruot */
//   73 //    tlv.sensor_reg_write[1] = tlv.sensor_reg_read[0];                              //7h 4:3
//   74 //    tlv.sensor_reg_write[1] = (tlv.sensor_reg_write[1] & 0x18) | 0x01;
//   75 //    
//   76 //    tlv.sensor_reg_write[3] = tlv.sensor_reg_read[2];                              //9h 4-0
//   77 //    tlv.sensor_reg_write[3] = (tlv.sensor_reg_write[3] & 0x1f) | 0x40;
//   78     
//   79     /* master controlled mode with variable frequency without interrupt*/
//   80     tlv.sensor_reg_write[1] = tlv.sensor_reg_read[0];                              //7h 4:3
        MOV       A, N:_tlv+12       ;; 1 cycle
        MOV       N:_tlv+23, A       ;; 1 cycle
//   81     tlv.sensor_reg_write[1] = (tlv.sensor_reg_write[1] & 0x18) | 0x03;
        MOVW      HL, #LWRD(_tlv+23)  ;; 1 cycle
        MOV       A, [HL]            ;; 1 cycle
        AND       A, #0x18           ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
        MOVW      HL, #LWRD(_tlv+23)  ;; 1 cycle
        MOV       A, [HL]            ;; 1 cycle
        OR        A, #0x3            ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
//   82     
//   83     tlv.sensor_reg_write[3] = tlv.sensor_reg_read[2];                              //9h 4-0
        MOV       A, N:_tlv+14       ;; 1 cycle
        MOV       N:_tlv+25, A       ;; 1 cycle
//   84     tlv.sensor_reg_write[3] = (tlv.sensor_reg_write[3] & 0x1f);
        MOVW      HL, #LWRD(_tlv+25)  ;; 1 cycle
        MOV       A, [HL]            ;; 1 cycle
        AND       A, #0x1F           ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
//   85     // ultra low power mode 10Hz and polling mode
//   86     tlv_i2c_start();
          CFI FunCall _tlv_i2c_start
        CALL      _tlv_i2c_start     ;; 3 cycles
//   87     
//   88     if(flag_tlv_bus_busy == 0)
        MOVW      HL, #LWRD(_tlv_flag)  ;; 1 cycle
        MOV1      CY, [HL].3         ;; 1 cycle
        BC        ??tlv_detect_ac_dc_field_4  ;; 4 cycles
        ; ------------------------------------- Block: 28 cycles
//   89     {
//   90 	if((tlv_i2c_byte_write(ADDR_MAG_W)) == MAG_ACK)     	// Write Device Address
        MOV       A, #0xBC           ;; 1 cycle
          CFI FunCall _tlv_i2c_byte_write
        CALL      _tlv_i2c_byte_write  ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??tlv_detect_ac_dc_field_4  ;; 4 cycles
        ; ------------------------------------- Block: 9 cycles
//   91 	{
//   92 	    if((tlv_i2c_byte_write(tlv.sensor_reg_write[0])) == MAG_ACK)
        MOV       A, N:_tlv+22       ;; 1 cycle
          CFI FunCall _tlv_i2c_byte_write
        CALL      _tlv_i2c_byte_write  ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??tlv_detect_ac_dc_field_4  ;; 4 cycles
        ; ------------------------------------- Block: 9 cycles
//   93 	    {
//   94 		if((tlv_i2c_byte_write(tlv.sensor_reg_write[1])) == MAG_ACK)
        MOV       A, N:_tlv+23       ;; 1 cycle
          CFI FunCall _tlv_i2c_byte_write
        CALL      _tlv_i2c_byte_write  ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??tlv_detect_ac_dc_field_4  ;; 4 cycles
        ; ------------------------------------- Block: 9 cycles
//   95 		{
//   96 		    if((tlv_i2c_byte_write(tlv.sensor_reg_write[2])) == MAG_ACK)
        MOV       A, N:_tlv+24       ;; 1 cycle
          CFI FunCall _tlv_i2c_byte_write
        CALL      _tlv_i2c_byte_write  ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??tlv_detect_ac_dc_field_4  ;; 4 cycles
        ; ------------------------------------- Block: 9 cycles
//   97 		    {
//   98 			if((tlv_i2c_byte_write(tlv.sensor_reg_write[3])) == MAG_ACK)
        MOV       A, N:_tlv+25       ;; 1 cycle
          CFI FunCall _tlv_i2c_byte_write
        CALL      _tlv_i2c_byte_write  ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??tlv_detect_ac_dc_field_4  ;; 4 cycles
          CFI FunCall _tlv_i2c_stop
        ; ------------------------------------- Block: 9 cycles
//   99 			{
//  100 			    tlv_i2c_stop();
        CALL      _tlv_i2c_stop      ;; 3 cycles
//  101 			    return MAG_I2C_OK;
        MOV       A, #0x0            ;; 1 cycle
        RET                          ;; 6 cycles
          CFI FunCall _tlv_i2c_stop
        ; ------------------------------------- Block: 10 cycles
//  102 			}
//  103 		    }
//  104 		}
//  105 	    }
//  106 	}
//  107     }
//  108     tlv_i2c_stop();
??tlv_detect_ac_dc_field_4:
        CALL      _tlv_i2c_stop      ;; 3 cycles
//  109     return MAG_I2C_ERROR;
        MOV       A, #0x1            ;; 1 cycle
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock1
        ; ------------------------------------- Block: 10 cycles
        ; ------------------------------------- Total: 93 cycles
//  110 }
//  111 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock2 Using cfiCommon0
          CFI Function _tlv_i2c_start
          CFI NoCalls
        CODE
//  112 void tlv_i2c_start()
//  113 {
_tlv_i2c_start:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  114     us8 temp_cntr;
//  115     
//  116     flag_tlv_nack_received = 0;
        CLR1      N:_tlv_flag.4      ;; 2 cycles
//  117     flag_tlv_bus_busy = 0;
        CLR1      N:_tlv_flag.3      ;; 2 cycles
//  118     MAG_SDA_HIGH;
        SET1      S:0xFFF00.6        ;; 2 cycles
//  119     MAG_SCL_HIGH;  
        SET1      S:0xFFF00.5        ;; 2 cycles
//  120     
//  121     MAG_SDA_INPUT;
        SET1      0xFFF20.6          ;; 2 cycles
//  122     MAG_SCL_INPUT;
        SET1      0xFFF20.5          ;; 2 cycles
//  123     
//  124     tlv_i2c_delay();
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
//  125     for(temp_cntr=0; temp_cntr<10; temp_cntr++)
        MOV       X, #0x0            ;; 1 cycle
        ; ------------------------------------- Block: 32 cycles
??tlv_i2c_start_0:
        MOV       A, X               ;; 1 cycle
        CMP       A, #0xA            ;; 1 cycle
        BNC       ??tlv_detect_ac_dc_field_5  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  126     {
//  127         if(IS_MAG_SCL_LOW || IS_MAG_SDA_LOW)
        MOV       A, S:0xFFF00       ;; 1 cycle
        AND       A, #0x20           ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        BZ        ??tlv_detect_ac_dc_field_6  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
        MOV       A, S:0xFFF00       ;; 1 cycle
        AND       A, #0x40           ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        BNZ       ??tlv_detect_ac_dc_field_7  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  128         {
//  129             tlv_i2c_delay();
??tlv_detect_ac_dc_field_6:
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
//  130         }
        INC       X                  ;; 1 cycle
        BR        S:??tlv_i2c_start_0  ;; 3 cycles
        ; ------------------------------------- Block: 23 cycles
//  131         else
//  132         {
//  133             MAG_SDA_OUTPUT;
??tlv_detect_ac_dc_field_7:
        CLR1      0xFFF20.6          ;; 2 cycles
        CLR1      0xF0050.6          ;; 2 cycles
//  134             MAG_SCL_OUTPUT;
        CLR1      0xFFF20.5          ;; 2 cycles
        CLR1      0xF0050.5          ;; 2 cycles
//  135             MAG_SDA_LOW;
        CLR1      S:0xFFF00.6        ;; 2 cycles
//  136             tlv_i2c_delay();
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
//  137             break;
        ; ------------------------------------- Block: 29 cycles
//  138         }
//  139     }
//  140     if(temp_cntr == 10)
??tlv_detect_ac_dc_field_5:
        XCH       A, X               ;; 1 cycle
        CMP       A, #0xA            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        BNZ       ??tlv_detect_ac_dc_field_8  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  141     {
//  142         flag_tlv_bus_busy = 1;
        SET1      N:_tlv_flag.3      ;; 2 cycles
//  143         MAG_SCL_OUTPUT;
        CLR1      0xFFF20.5          ;; 2 cycles
        CLR1      0xF0050.5          ;; 2 cycles
//  144         MAG_SCL_OUTPUT;
        CLR1      0xFFF20.5          ;; 2 cycles
        CLR1      0xF0050.5          ;; 2 cycles
        ; ------------------------------------- Block: 10 cycles
//  145     }
//  146 }
??tlv_detect_ac_dc_field_8:
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock2
        ; ------------------------------------- Block: 6 cycles
        ; ------------------------------------- Total: 127 cycles
        REQUIRE __A_P0
        REQUIRE __A_PM0
        REQUIRE __A_POM0
//  147 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock3 Using cfiCommon0
          CFI Function _tlv_i2c_stop
          CFI NoCalls
        CODE
//  148 void tlv_i2c_stop()
//  149 {
_tlv_i2c_stop:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  150     MAG_SCL_LOW; NOP();       /* it is considered that SCL is low before executing this function. Always leave SCL as LOW from Every function */
        CLR1      S:0xFFF00.5        ;; 2 cycles
        NOP                          ;; 1 cycle
//  151     MAG_SDA_LOW;
        CLR1      S:0xFFF00.6        ;; 2 cycles
//  152     tlv_i2c_delay();
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
//  153     MAG_SCL_HIGH;
        SET1      S:0xFFF00.5        ;; 2 cycles
//  154     tlv_i2c_delay();
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
//  155     MAG_SDA_HIGH;
        SET1      S:0xFFF00.6        ;; 2 cycles
//  156     tlv_i2c_delay();
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
//  157 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock3
        ; ------------------------------------- Block: 72 cycles
        ; ------------------------------------- Total: 72 cycles
        REQUIRE __A_P0
//  158 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock4 Using cfiCommon0
          CFI Function _tlv_i2c_restart
          CFI FunCall _tlv_i2c_start
        CODE
//  159 void tlv_i2c_restart()
//  160 {
_tlv_i2c_restart:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  161     tlv_i2c_start();
        CALL      _tlv_i2c_start     ;; 3 cycles
//  162 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock4
        ; ------------------------------------- Block: 9 cycles
        ; ------------------------------------- Total: 9 cycles
//  163 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock5 Using cfiCommon1
          CFI Function _tlv_i2c_byte_read
          CFI NoCalls
        CODE
//  164 void tlv_i2c_byte_read(us8 *iic_data, us8 ack_data)
//  165 {
_tlv_i2c_byte_read:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
        MOVW      HL, AX             ;; 1 cycle
//  166     us8 maskData = 0x80u,readData;
        MOV       X, #0x80           ;; 1 cycle
//  167     *iic_data = 0;  
        MOV       A, #0x0            ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
//  168     
//  169     MAG_SDA_INPUT;
        SET1      0xFFF20.6          ;; 2 cycles
        ; ------------------------------------- Block: 6 cycles
//  170     while(maskData)          
??tlv_i2c_byte_read_0:
        CMP0      X                  ;; 1 cycle
        BZ        ??tlv_detect_ac_dc_field_9  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  171     {	
//  172 	tlv_i2c_delay();
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
//  173 	readData = *iic_data | maskData;    
        MOV       A, [HL]            ;; 1 cycle
        OR        A, X               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
//  174 	MAG_SCL_HIGH;      
        SET1      S:0xFFF00.5        ;; 2 cycles
//  175 	tlv_i2c_delay();  
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
//  176 	if(IS_MAG_SDA_HIGH)    
        MOV       A, S:0xFFF00       ;; 1 cycle
        AND       A, #0x40           ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        BZ        ??tlv_detect_ac_dc_field_10  ;; 4 cycles
        ; ------------------------------------- Block: 50 cycles
//  177 	{
//  178 	    *iic_data = readData;	
        MOV       A, B               ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
//  179 	}
//  180         MAG_SCL_LOW;    
??tlv_detect_ac_dc_field_10:
        CLR1      S:0xFFF00.5        ;; 2 cycles
//  181 	maskData >>= 1;
        MOV       A, X               ;; 1 cycle
        SHR       A, 0x1             ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        BR        S:??tlv_i2c_byte_read_0  ;; 3 cycles
        ; ------------------------------------- Block: 8 cycles
//  182     }
//  183     NOP();NOP();
??tlv_detect_ac_dc_field_9:
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
//  184     MAG_SDA_OUTPUT; 
        CLR1      0xFFF20.6          ;; 2 cycles
        CLR1      0xF0050.6          ;; 2 cycles
//  185     if(ack_data == MAG_ACK)       
        CMP0      C                  ;; 1 cycle
        BNZ       ??tlv_detect_ac_dc_field_11  ;; 4 cycles
        ; ------------------------------------- Block: 11 cycles
//  186     {
//  187 	MAG_SDA_LOW;	
        CLR1      S:0xFFF00.6        ;; 2 cycles
        BR        S:??tlv_detect_ac_dc_field_12  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  188     }
//  189     else			
//  190     {
//  191 	MAG_SDA_HIGH;	
??tlv_detect_ac_dc_field_11:
        SET1      S:0xFFF00.6        ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
//  192     }
//  193     tlv_i2c_delay();
??tlv_detect_ac_dc_field_12:
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
//  194     MAG_SCL_HIGH;      
        SET1      S:0xFFF00.5        ;; 2 cycles
//  195     tlv_i2c_delay();
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
//  196     MAG_SCL_LOW;      
        CLR1      S:0xFFF00.5        ;; 2 cycles
//  197 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock5
        ; ------------------------------------- Block: 48 cycles
        ; ------------------------------------- Total: 137 cycles
        REQUIRE __A_PM0
        REQUIRE __A_P0
        REQUIRE __A_POM0
//  198 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock6 Using cfiCommon0
          CFI Function _tlv_i2c_byte_write
          CFI NoCalls
        CODE
//  199 us8 tlv_i2c_byte_write(us8 iic_data)
//  200 {
_tlv_i2c_byte_write:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
        MOV       C, A               ;; 1 cycle
//  201     us8 maskData = 0x80, ret = MAG_NACK,i;     
        MOV       X, #0x80           ;; 1 cycle
        MOV       D, #0x1            ;; 1 cycle
//  202     MAG_SCL_LOW;
        CLR1      S:0xFFF00.5        ;; 2 cycles
        ; ------------------------------------- Block: 5 cycles
//  203     while(maskData)       
??tlv_i2c_byte_write_0:
        CMP0      X                  ;; 1 cycle
        BZ        ??tlv_detect_ac_dc_field_13  ;; 4 cycles
        ; ------------------------------------- Block: 5 cycles
//  204     {	
//  205 	NOP();NOP();NOP();NOP();NOP();NOP();NOP();   
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
//  206 	if(iic_data & maskData) 
        MOV       A, C               ;; 1 cycle
        AND       A, X               ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        BZ        ??tlv_detect_ac_dc_field_14  ;; 4 cycles
        ; ------------------------------------- Block: 14 cycles
//  207 	{
//  208 	    MAG_SDA_HIGH;   
        SET1      S:0xFFF00.6        ;; 2 cycles
        BR        S:??tlv_detect_ac_dc_field_15  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  209 	}
//  210 	else
//  211 	{
//  212 	    MAG_SDA_LOW;    
??tlv_detect_ac_dc_field_14:
        CLR1      S:0xFFF00.6        ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
//  213 	}
//  214 	tlv_i2c_delay();
??tlv_detect_ac_dc_field_15:
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
//  215 	MAG_SCL_HIGH;
        SET1      S:0xFFF00.5        ;; 2 cycles
//  216 	tlv_i2c_delay();
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
//  217 	MAG_SCL_LOW;     
        CLR1      S:0xFFF00.5        ;; 2 cycles
//  218 	maskData >>= 1;
        MOV       A, X               ;; 1 cycle
        SHR       A, 0x1             ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        BR        S:??tlv_i2c_byte_write_0  ;; 3 cycles
        ; ------------------------------------- Block: 48 cycles
//  219     }
//  220     NOP();NOP();
??tlv_detect_ac_dc_field_13:
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
//  221     // Get ACK from slave at 9th pulse
//  222     MAG_SDA_INPUT;        
        SET1      0xFFF20.6          ;; 2 cycles
//  223     tlv_i2c_delay();
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
//  224     MAG_SCL_HIGH;
        SET1      S:0xFFF00.5        ;; 2 cycles
//  225     tlv_i2c_delay();
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
        NOP                          ;; 1 cycle
//  226     for(i=0;i<200;i++)
        MOV       B, #0x0            ;; 1 cycle
        ; ------------------------------------- Block: 45 cycles
??tlv_i2c_byte_write_1:
        MOV       A, B               ;; 1 cycle
        CMP       A, #0xC8           ;; 1 cycle
        BNC       ??tlv_detect_ac_dc_field_16  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  227     {
//  228         if(IS_MAG_SDA_LOW) 
        MOV       A, S:0xFFF00       ;; 1 cycle
        AND       A, #0x40           ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        BNZ       ??tlv_detect_ac_dc_field_17  ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  229         {
//  230             ret = ACK;
        MOV       A, #0x0            ;; 1 cycle
        MOV       D, A               ;; 1 cycle
//  231             break;
        BR        S:??tlv_detect_ac_dc_field_16  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  232         }				
//  233     }
??tlv_detect_ac_dc_field_17:
        INC       B                  ;; 1 cycle
        BR        S:??tlv_i2c_byte_write_1  ;; 3 cycles
        ; ------------------------------------- Block: 4 cycles
//  234     MAG_SCL_LOW;     
??tlv_detect_ac_dc_field_16:
        CLR1      S:0xFFF00.5        ;; 2 cycles
//  235     MAG_SDA_OUTPUT;
        CLR1      0xFFF20.6          ;; 2 cycles
        CLR1      0xF0050.6          ;; 2 cycles
//  236     if(ret == NACK)
        XCH       A, D               ;; 1 cycle
        CMP       A, #0x1            ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 10 cycles
//  237     {
//  238         flag_tlv_nack_received = 1;
        SET1      N:_tlv_flag.4      ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
//  239     }
//  240     return(ret);
??tlv_i2c_byte_write_2:
        MOV       A, D               ;; 1 cycle
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock6
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 165 cycles
        REQUIRE __A_P0
        REQUIRE __A_PM0
        REQUIRE __A_POM0
//  241 }
//  242 //void tlv_i2c_delay()
//  243 //{
//  244 //    if(clock_select == CLOCK_24MHZ)
//  245 //    {
//  246 //        NOP();NOP();NOP();NOP();NOP();
//  247 //        NOP();NOP();NOP();NOP();NOP();
//  248 //    }
//  249 //    else if(clock_select == CLOCK_12MHZ)
//  250 //    {
//  251 //        NOP();NOP();NOP();NOP();NOP();
//  252 //        NOP();NOP();NOP();NOP();NOP();
//  253 //    }
//  254 //    else if(clock_select == CLOCK_6MHZ)
//  255 //    {
//  256 //        NOP();NOP();NOP();NOP();NOP();
//  257 //    }
//  258 //    else if(clock_select == CLOCK_1_5MHZ)
//  259 //    {
//  260 //        NOP();
//  261 //    }
//  262 //}

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock7 Using cfiCommon0
          CFI Function _tlv_recovery
          CFI FunCall _tlv_i2c_start
        CODE
//  263 us8 tlv_recovery()
//  264 {
_tlv_recovery:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  265     tlv_i2c_start();  
        CALL      _tlv_i2c_start     ;; 3 cycles
//  266     if(flag_tlv_bus_busy == 0)
        MOVW      HL, #LWRD(_tlv_flag)  ;; 1 cycle
        MOV1      CY, [HL].3         ;; 1 cycle
        BC        ??tlv_detect_ac_dc_field_18  ;; 4 cycles
        ; ------------------------------------- Block: 9 cycles
//  267     {
//  268 	if((tlv_i2c_byte_write(0xFF)) == MAG_ACK)     
        MOV       A, #0xFF           ;; 1 cycle
          CFI FunCall _tlv_i2c_byte_write
        CALL      _tlv_i2c_byte_write  ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??tlv_detect_ac_dc_field_19  ;; 4 cycles
          CFI FunCall _tlv_i2c_stop
        ; ------------------------------------- Block: 9 cycles
//  269 	{
//  270 	    tlv_i2c_stop();
        CALL      _tlv_i2c_stop      ;; 3 cycles
//  271 	    return MAG_I2C_OK;
        MOV       A, #0x0            ;; 1 cycle
        RET                          ;; 6 cycles
          CFI FunCall _tlv_i2c_stop
        ; ------------------------------------- Block: 10 cycles
//  272 	}
//  273 	else
//  274 	{
//  275 	    tlv_i2c_stop();  
??tlv_detect_ac_dc_field_19:
        CALL      _tlv_i2c_stop      ;; 3 cycles
//  276 	    return MAG_I2C_ERROR;
        MOV       A, #0x1            ;; 1 cycle
        RET                          ;; 6 cycles
          CFI FunCall _tlv_i2c_stop
        ; ------------------------------------- Block: 10 cycles
//  277 	}
//  278     }
//  279     tlv_i2c_stop();  
??tlv_detect_ac_dc_field_18:
        CALL      _tlv_i2c_stop      ;; 3 cycles
//  280     return MAG_I2C_ERROR;
        MOV       A, #0x1            ;; 1 cycle
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock7
        ; ------------------------------------- Block: 10 cycles
        ; ------------------------------------- Total: 48 cycles
//  281 }

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock8 Using cfiCommon0
          CFI Function _tlv_reset
          CFI FunCall _tlv_i2c_start
        CODE
//  282 us8 tlv_reset()
//  283 {
_tlv_reset:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  284     tlv_i2c_start();  
        CALL      _tlv_i2c_start     ;; 3 cycles
//  285     if(flag_tlv_bus_busy == 0)
        MOVW      HL, #LWRD(_tlv_flag)  ;; 1 cycle
        MOV1      CY, [HL].3         ;; 1 cycle
        BC        ??tlv_detect_ac_dc_field_20  ;; 4 cycles
        ; ------------------------------------- Block: 9 cycles
//  286     {
//  287 	if((tlv_i2c_byte_write(0x00)) == MAG_ACK)     
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _tlv_i2c_byte_write
        CALL      _tlv_i2c_byte_write  ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??tlv_detect_ac_dc_field_21  ;; 4 cycles
        ; ------------------------------------- Block: 9 cycles
//  288 	{
//  289 	    MAG_SDA_HIGH;
        SET1      S:0xFFF00.6        ;; 2 cycles
//  290 	    delay_us(30);
        MOVW      AX, #0x1E          ;; 1 cycle
          CFI FunCall _delay_us
        CALL      _delay_us          ;; 3 cycles
//  291 	    MAG_SDA_LOW;
        CLR1      S:0xFFF00.6        ;; 2 cycles
//  292 	    tlv_i2c_stop();
          CFI FunCall _tlv_i2c_stop
        CALL      _tlv_i2c_stop      ;; 3 cycles
//  293 	    return MAG_I2C_OK;
        MOV       A, #0x0            ;; 1 cycle
        RET                          ;; 6 cycles
          CFI FunCall _tlv_i2c_stop
        ; ------------------------------------- Block: 18 cycles
//  294 	}
//  295 	else
//  296 	{
//  297 	    tlv_i2c_stop();  
??tlv_detect_ac_dc_field_21:
        CALL      _tlv_i2c_stop      ;; 3 cycles
//  298 	    return MAG_I2C_ERROR;
        MOV       A, #0x1            ;; 1 cycle
        RET                          ;; 6 cycles
          CFI FunCall _tlv_i2c_stop
        ; ------------------------------------- Block: 10 cycles
//  299 	}
//  300     }
//  301     tlv_i2c_stop();  
??tlv_detect_ac_dc_field_20:
        CALL      _tlv_i2c_stop      ;; 3 cycles
//  302     return MAG_I2C_ERROR;
        MOV       A, #0x1            ;; 1 cycle
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock8
        ; ------------------------------------- Block: 10 cycles
        ; ------------------------------------- Total: 56 cycles
        REQUIRE __A_P0
//  303 }

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock9 Using cfiCommon0
          CFI Function _tlv_clear_var
          CFI NoCalls
        CODE
//  304 void tlv_clear_var()
//  305 {
_tlv_clear_var:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  306     tlv.sensor_reg_read[0] = 0;
        MOV       N:_tlv+12, #0x0    ;; 1 cycle
//  307     tlv.sensor_reg_read[1] = 0;
        MOV       N:_tlv+13, #0x0    ;; 1 cycle
//  308     tlv.sensor_reg_read[2] = 0;
        MOV       N:_tlv+14, #0x0    ;; 1 cycle
//  309     tlv.sensor_reg_read[3] = 0;
        MOV       N:_tlv+15, #0x0    ;; 1 cycle
//  310     tlv.sensor_reg_read[4] = 0;
        MOV       N:_tlv+16, #0x0    ;; 1 cycle
//  311     tlv.sensor_reg_read[5] = 0;
        MOV       N:_tlv+17, #0x0    ;; 1 cycle
//  312     tlv.sensor_reg_read[6] = 0;
        MOV       N:_tlv+18, #0x0    ;; 1 cycle
//  313     tlv.sensor_reg_read[7] = 0;
        MOV       N:_tlv+19, #0x0    ;; 1 cycle
//  314     tlv.sensor_reg_read[8] = 0;
        MOV       N:_tlv+20, #0x0    ;; 1 cycle
//  315     tlv.sensor_reg_read[9] = 0;
        MOV       N:_tlv+21, #0x0    ;; 1 cycle
//  316     tlv.sensor_reg_write[0] = 0;
        MOV       N:_tlv+22, #0x0    ;; 1 cycle
//  317     tlv.sensor_reg_write[1] = 0;
        MOV       N:_tlv+23, #0x0    ;; 1 cycle
//  318     tlv.sensor_reg_write[2] = 0;
        MOV       N:_tlv+24, #0x0    ;; 1 cycle
//  319     tlv.sensor_reg_write[3] = 0;
        MOV       N:_tlv+25, #0x0    ;; 1 cycle
//  320     
//  321     tlv.avg_sample_counter = 0;
        MOV       N:_tlv+26, #0x0    ;; 1 cycle
//  322     tlv.frame_cnt = 0;
        MOV       N:_tlv+27, #0x0    ;; 1 cycle
//  323     tlv.fram_cnt_prev = 0;
        MOV       N:_tlv+28, #0x0    ;; 1 cycle
//  324     //tlv.fault_detect_count = 0;
//  325     tlv.adc_hang_cnt = 0;
        MOV       N:_tlv+31, #0x0    ;; 1 cycle
//  326     tlv.Bnet = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_tlv+32, AX      ;; 1 cycle
//  327     tlv.Bnet_avg = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_tlv+34, AX      ;; 1 cycle
//  328     tlv.Bnet_rms = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_tlv+36, AX      ;; 1 cycle
//  329     tlv.Bnet_avg_acc = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_tlv+52, AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_tlv+54, AX      ;; 1 cycle
//  330     tlv.Bnet_rms_acc = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_tlv+56, AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_tlv+58, AX      ;; 1 cycle
//  331     tlv_flag.all = 0;
        MOV       N:_tlv_flag, #0x0  ;; 1 cycle
//  332 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock9
        ; ------------------------------------- Block: 39 cycles
        ; ------------------------------------- Total: 39 cycles
//  333 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock10 Using cfiCommon0
          CFI Function _tlv493d_init
        CODE
//  334 us8 tlv493d_init()
//  335 {
_tlv493d_init:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  336     if(tlv.tlv_init_cntr < 250)
        MOV       A, N:_tlv+30       ;; 1 cycle
        CMP       A, #0xFA           ;; 1 cycle
        SKNC                         ;; 1 cycle
        ; ------------------------------------- Block: 3 cycles
//  337     {
//  338       tlv.tlv_init_cntr++;
        INC       N:_tlv+30          ;; 2 cycles
          CFI FunCall _tlv_reset
        ; ------------------------------------- Block: 2 cycles
//  339     }
//  340     if(tlv_reset() == MAG_I2C_OK)
??tlv493d_init_0:
        CALL      _tlv_reset         ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??tlv_detect_ac_dc_field_22  ;; 4 cycles
          CFI FunCall _tlv_read_reg
        ; ------------------------------------- Block: 8 cycles
//  341     {
//  342         if(tlv_read_reg() == MAG_I2C_OK)
        CALL      _tlv_read_reg      ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        BNZ       ??tlv_detect_ac_dc_field_23  ;; 4 cycles
          CFI FunCall _tlv_configure
        ; ------------------------------------- Block: 8 cycles
//  343         {
//  344             if(tlv_configure() == MAG_I2C_OK)
        CALL      _tlv_configure     ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        CLRB      A                  ;; 1 cycle
        BZ        ??tlv_detect_ac_dc_field_24  ;; 4 cycles
        ; ------------------------------------- Block: 9 cycles
        ONEB      A                  ;; 1 cycle
//  345             {
//  346                 return(MAG_I2C_OK);
        RET                          ;; 6 cycles
        ; ------------------------------------- Block: 7 cycles
//  347             }
//  348             else
//  349             {
//  350                 return(MAG_I2C_ERROR);
//  351             }
//  352         }
//  353         else
//  354         {
//  355             return(MAG_I2C_ERROR);
??tlv_detect_ac_dc_field_23:
        MOV       A, #0x1            ;; 1 cycle
        RET                          ;; 6 cycles
        ; ------------------------------------- Block: 7 cycles
//  356         }
//  357     }
//  358     else
//  359     {
//  360         return(MAG_I2C_ERROR);
??tlv_detect_ac_dc_field_22:
        MOV       A, #0x1            ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??tlv_detect_ac_dc_field_24:
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock10
        ; ------------------------------------- Block: 6 cycles
        ; ------------------------------------- Total: 51 cycles
//  361     }
//  362 }
//  363 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock11 Using cfiCommon0
          CFI Function _tlv493d_scan
          CFI FunCall _tlv_read_reg
        CODE
//  364 void tlv493d_scan()
//  365 {
_tlv493d_scan:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  366     tlv_read_reg();
        CALL      _tlv_read_reg      ;; 3 cycles
//  367     
//  368     /* if data read is valid then proceed further */
//  369     
//  370     tlv.frame_cnt = ((tlv.sensor_reg_read[3] >> 2) & 0x03);
        MOV       A, N:_tlv+15       ;; 1 cycle
        SHR       A, 0x2             ;; 1 cycle
        AND       A, #0x3            ;; 1 cycle
        MOV       N:_tlv+27, A       ;; 1 cycle
//  371     if(tlv.frame_cnt != tlv.fram_cnt_prev) 
        MOV       A, N:_tlv+27       ;; 1 cycle
        CMP       A, N:_tlv+28       ;; 1 cycle
        BZ        ??tlv_detect_ac_dc_field_25  ;; 4 cycles
        ; ------------------------------------- Block: 13 cycles
//  372     {
//  373         tlv.fram_cnt_prev = tlv.frame_cnt;
        MOV       A, N:_tlv+27       ;; 1 cycle
        MOV       N:_tlv+28, A       ;; 1 cycle
//  374         tlv.adc_hang_cnt = 0;
        MOV       N:_tlv+31, #0x0    ;; 1 cycle
        BR        S:??tlv_detect_ac_dc_field_26  ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
//  375     }
//  376     else
//  377     {
//  378         tlv.adc_hang_cnt++;
??tlv_detect_ac_dc_field_25:
        INC       N:_tlv+31          ;; 2 cycles
//  379         if(tlv.adc_hang_cnt_total < 60000)
        MOVW      AX, N:_tlv+40      ;; 1 cycle
        CMPW      AX, #0xEA60        ;; 1 cycle
        SKNC                         ;; 1 cycle
        ; ------------------------------------- Block: 5 cycles
//  380         {
//  381           tlv.adc_hang_cnt_total++;
        INCW      N:_tlv+40          ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
//  382         }
//  383     } 
//  384     // check ch bits . they should be zero otherwise the conversion is going on
//  385     
//  386     
//  387     /* X axis */
//  388     temp_us16 = (((us16)tlv.sensor_reg_read[0])<<4) | ((tlv.sensor_reg_read[4]>>4) & 0x0F); 
??tlv_detect_ac_dc_field_26:
        MOV       A, N:_tlv+16       ;; 1 cycle
        SHR       A, 0x4             ;; 1 cycle
        MOV       L, A               ;; 1 cycle
        MOV       H, #0x0            ;; 1 cycle
        MOV       X, N:_tlv+12       ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        SHLW      AX, 0x4            ;; 1 cycle
        OR        A, H               ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        OR        A, L               ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      S:_temp_us16, AX   ;; 1 cycle
//  389     if(temp_us16 & 0x800)
        BF        S:_temp_us16+1.3, ??tlv_detect_ac_dc_field_27  ;; 5 cycles
        ; ------------------------------------- Block: 17 cycles
//  390     {
//  391     	temp_us16 |= 0xf000;
        MOVW      AX, S:_temp_us16   ;; 1 cycle
        OR        A, #0xF0           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        OR        A, #0x0            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      S:_temp_us16, AX   ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
//  392     }
//  393     tlv.Bx = (s16)temp_us16;
??tlv_detect_ac_dc_field_27:
        MOVW      AX, S:_temp_us16   ;; 1 cycle
        MOVW      N:_tlv+42, AX      ;; 1 cycle
//  394     if(tlv.Bx & 0x8000)
        MOVW      HL, #LWRD(_tlv+43)  ;; 1 cycle
        MOV1      CY, [HL].7         ;; 1 cycle
        BNC       ??tlv_detect_ac_dc_field_28  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
//  395     {
//  396         temp_us16 = ABS(tlv.Bx);
        MOVW      AX, N:_tlv+42      ;; 1 cycle
        BF        A.7, ??tlv_detect_ac_dc_field_29  ;; 5 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, N:_tlv+42      ;; 1 cycle
        XOR       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        XOR       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        INCW      AX                 ;; 1 cycle
        MOVW      S:_temp_us16, AX   ;; 1 cycle
        BR        S:??tlv_detect_ac_dc_field_28  ;; 3 cycles
        ; ------------------------------------- Block: 10 cycles
??tlv_detect_ac_dc_field_29:
        MOVW      AX, N:_tlv+42      ;; 1 cycle
        MOVW      S:_temp_us16, AX   ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
//  397     }
//  398     temp_us32 = (us32)temp_us16 * temp_us16;
??tlv_detect_ac_dc_field_28:
        MOVW      AX, S:_temp_us16   ;; 1 cycle
        MOVW      BC, S:_temp_us16   ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      S:_temp_us32, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_temp_us32+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  399     
//  400     /* Y axis */
//  401     temp_us16 = (((us16)tlv.sensor_reg_read[1])<<4) | (tlv.sensor_reg_read[4]&0x0f);
        MOV       X, N:_tlv+16       ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        AND       A, #0x0            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        AND       A, #0xF            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV       X, N:_tlv+13       ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        SHLW      AX, 0x4            ;; 1 cycle
        OR        A, H               ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        OR        A, L               ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      S:_temp_us16, AX   ;; 1 cycle
//  402     if(temp_us16 & 0x800)
        BF        S:_temp_us16+1.3, ??tlv_detect_ac_dc_field_30  ;; 5 cycles
        ; ------------------------------------- Block: 28 cycles
//  403     {
//  404     	temp_us16 |= 0xf000;
        MOVW      AX, S:_temp_us16   ;; 1 cycle
        OR        A, #0xF0           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        OR        A, #0x0            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      S:_temp_us16, AX   ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
//  405     }
//  406     tlv.By = (s16)temp_us16;
??tlv_detect_ac_dc_field_30:
        MOVW      AX, S:_temp_us16   ;; 1 cycle
        MOVW      N:_tlv+44, AX      ;; 1 cycle
//  407     if(tlv.By & 0x8000)
        MOVW      HL, #LWRD(_tlv+45)  ;; 1 cycle
        MOV1      CY, [HL].7         ;; 1 cycle
        BNC       ??tlv_detect_ac_dc_field_31  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
//  408     {
//  409         temp_us16 = ABS(tlv.By);
        MOVW      AX, N:_tlv+44      ;; 1 cycle
        BF        A.7, ??tlv_detect_ac_dc_field_32  ;; 5 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, N:_tlv+44      ;; 1 cycle
        XOR       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        XOR       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        INCW      AX                 ;; 1 cycle
        MOVW      S:_temp_us16, AX   ;; 1 cycle
        BR        S:??tlv_detect_ac_dc_field_31  ;; 3 cycles
        ; ------------------------------------- Block: 10 cycles
??tlv_detect_ac_dc_field_32:
        MOVW      AX, N:_tlv+44      ;; 1 cycle
        MOVW      S:_temp_us16, AX   ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
//  410     }
//  411     temp_us32 += (us32)temp_us16 * temp_us16;
??tlv_detect_ac_dc_field_31:
        MOVW      AX, S:_temp_us16   ;; 1 cycle
        MOVW      BC, S:_temp_us16   ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      HL, S:_temp_us32+2  ;; 1 cycle
        MOVW      DE, S:_temp_us32   ;; 1 cycle
        ADDW      AX, DE             ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_temp_us32, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_temp_us32+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  412     
//  413     /* B axis */
//  414     temp_us16 = (((us16)tlv.sensor_reg_read[2])<<4) | (tlv.sensor_reg_read[5]&0x0f);
        MOV       X, N:_tlv+17       ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        AND       A, #0x0            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        AND       A, #0xF            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV       X, N:_tlv+14       ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        SHLW      AX, 0x4            ;; 1 cycle
        OR        A, H               ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        OR        A, L               ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      S:_temp_us16, AX   ;; 1 cycle
//  415     if(temp_us16 & 0x800)
        BF        S:_temp_us16+1.3, ??tlv_detect_ac_dc_field_33  ;; 5 cycles
        ; ------------------------------------- Block: 39 cycles
//  416     {
//  417     	temp_us16 |= 0xf000;
        MOVW      AX, S:_temp_us16   ;; 1 cycle
        OR        A, #0xF0           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        OR        A, #0x0            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      S:_temp_us16, AX   ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
//  418     }
//  419     tlv.Bz = (s16)temp_us16;
??tlv_detect_ac_dc_field_33:
        MOVW      AX, S:_temp_us16   ;; 1 cycle
        MOVW      N:_tlv+46, AX      ;; 1 cycle
//  420     if(tlv.Bz & 0x8000)
        MOVW      HL, #LWRD(_tlv+47)  ;; 1 cycle
        MOV1      CY, [HL].7         ;; 1 cycle
        BNC       ??tlv_detect_ac_dc_field_34  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
//  421     {
//  422         temp_us16 = ABS(tlv.Bz);
        MOVW      AX, N:_tlv+46      ;; 1 cycle
        BF        A.7, ??tlv_detect_ac_dc_field_35  ;; 5 cycles
        ; ------------------------------------- Block: 6 cycles
        MOVW      AX, N:_tlv+46      ;; 1 cycle
        XOR       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        XOR       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        INCW      AX                 ;; 1 cycle
        MOVW      S:_temp_us16, AX   ;; 1 cycle
        BR        S:??tlv_detect_ac_dc_field_34  ;; 3 cycles
        ; ------------------------------------- Block: 10 cycles
??tlv_detect_ac_dc_field_35:
        MOVW      AX, N:_tlv+46      ;; 1 cycle
        MOVW      S:_temp_us16, AX   ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
//  423     }
//  424     temp_us32 += (us32)temp_us16 * temp_us16;
??tlv_detect_ac_dc_field_34:
        MOVW      AX, S:_temp_us16   ;; 1 cycle
        MOVW      BC, S:_temp_us16   ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      HL, S:_temp_us32+2  ;; 1 cycle
        MOVW      DE, S:_temp_us32   ;; 1 cycle
        ADDW      AX, DE             ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_temp_us32, AX   ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      S:_temp_us32+2, AX  ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  425     
//  426     /* Net calculation */
//  427     tlv.Bnet = (us16)sqrt(temp_us32);
        MOVW      BC, S:_temp_us32+2  ;; 1 cycle
        MOVW      AX, S:_temp_us32   ;; 1 cycle
          CFI FunCall ?F_UL2F
        CALL      N:?F_UL2F          ;; 3 cycles
          CFI FunCall _sqrt
        CALL      _sqrt              ;; 3 cycles
          CFI FunCall ?F_F2SL
        CALL      N:?F_F2SL          ;; 3 cycles
        MOVW      N:_tlv+32, AX      ;; 1 cycle
//  428     
//  429     /* Temperature calculation */
//  430     temp_us16 = ((((us16)tlv.sensor_reg_read[3])<<4) & 0x0F00) | tlv.sensor_reg_read[6];
        MOV       C, N:_tlv+18       ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOV       X, N:_tlv+15       ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        SHLW      AX, 0x4            ;; 1 cycle
        AND       A, #0xF            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        AND       A, #0x0            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        OR        A, B               ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        OR        A, C               ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      S:_temp_us16, AX   ;; 1 cycle
//  431     if(temp_us16 & 0x800)
        BF        S:_temp_us16+1.3, ??tlv_detect_ac_dc_field_36  ;; 5 cycles
        ; ------------------------------------- Block: 50 cycles
//  432     {
//  433 	temp_us16 |= (0xf000);
        MOVW      AX, S:_temp_us16   ;; 1 cycle
        OR        A, #0xF0           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        OR        A, #0x0            ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        MOVW      S:_temp_us16, AX   ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
//  434     }
//  435     tlv.temperature_instant = temp_us16;
??tlv_detect_ac_dc_field_36:
        MOVW      AX, S:_temp_us16   ;; 1 cycle
        MOVW      N:_tlv+48, AX      ;; 1 cycle
//  436     
//  437     tlv_cal_max_min_counts();
          CFI FunCall _tlv_cal_max_min_counts
        CALL      _tlv_cal_max_min_counts  ;; 3 cycles
//  438     /* accumulating and averaging */
//  439     tlv.temperature_avg_acc += tlv.temperature_instant;
        MOVW      AX, N:_tlv+48      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      HL, N:_tlv+62      ;; 1 cycle
        MOVW      DE, N:_tlv+60      ;; 1 cycle
        ADDW      AX, DE             ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_tlv+60, AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_tlv+62, AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  440     tlv.Bnet_avg_acc += tlv.Bnet;
        MOVW      AX, N:_tlv+32      ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        MOVW      HL, N:_tlv+54      ;; 1 cycle
        MOVW      DE, N:_tlv+52      ;; 1 cycle
        ADDW      AX, DE             ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_tlv+52, AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_tlv+54, AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  441     tlv.Bnet_rms_acc += (us32)tlv.Bnet*tlv.Bnet;
        MOVW      AX, N:_tlv+32      ;; 1 cycle
        MOVW      BC, N:_tlv+32      ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      HL, N:_tlv+58      ;; 1 cycle
        MOVW      DE, N:_tlv+56      ;; 1 cycle
        ADDW      AX, DE             ;; 1 cycle
        SKNC
        INCW      BC                 ;; 5 cycles
        XCHW      AX, BC             ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_tlv+56, AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
        MOVW      N:_tlv+58, AX      ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
//  442     tlv.avg_sample_counter++;
        INC       N:_tlv+26          ;; 2 cycles
//  443     if(flag_tlv_cal_avg == 1 && tlv.avg_sample_counter != 0)
        MOVW      HL, #LWRD(_tlv_flag)  ;; 1 cycle
        MOV1      CY, [HL].0         ;; 1 cycle
        SKC                          ;; 4 cycles
        BR        N:??tlv_detect_ac_dc_field_37  ;; 4 cycles
        ; ------------------------------------- Block: 68 cycles
        CMP0      N:_tlv+26          ;; 1 cycle
        SKNZ                         ;; 4 cycles
        BR        N:??tlv_detect_ac_dc_field_37  ;; 4 cycles
          CFI FunCall _tlv_detect_ac_dc_field
        ; ------------------------------------- Block: 5 cycles
//  444     {
//  445         tlv_detect_ac_dc_field();
        CALL      _tlv_detect_ac_dc_field  ;; 3 cycles
//  446         flag_tlv_cal_avg = 0;
        CLR1      N:_tlv_flag.0      ;; 2 cycles
//  447         tlv.Bnet_avg = tlv.Bnet_avg_acc / tlv.avg_sample_counter;
        MOV       X, N:_tlv+26       ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOVW      HL, #0x0           ;; 1 cycle
        MOVW      BC, N:_tlv+54      ;; 1 cycle
        MOVW      AX, N:_tlv+52      ;; 1 cycle
        DIVWU                        ;; 17 cycles
        NOP                          ;; 1 cycle
        MOVW      N:_tlv+34, AX      ;; 1 cycle
//  448         tlv.Bnet_avg = (us16)(tlv.Bnet_avg * 0.98f);
        MOVW      AX, #0x3F7A        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+6
        MOVW      AX, #0xE148        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+8
        MOVW      AX, N:_tlv+34      ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
          CFI FunCall ?F_UL2F
        CALL      N:?F_UL2F          ;; 3 cycles
          CFI FunCall ?F_MUL
        CALL      N:?F_MUL           ;; 3 cycles
          CFI FunCall ?F_F2SL
        CALL      N:?F_F2SL          ;; 3 cycles
        MOVW      N:_tlv+34, AX      ;; 1 cycle
//  449         tlv.Bnet_avg_max = MAX(tlv.Bnet_avg,tlv.Bnet_avg_max);
        MOVW      HL, N:_tlv+38      ;; 1 cycle
        MOVW      AX, N:_tlv+34      ;; 1 cycle
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+4
        CMPW      AX, HL             ;; 1 cycle
        BNC       ??tlv_detect_ac_dc_field_38  ;; 4 cycles
        ; ------------------------------------- Block: 54 cycles
        MOVW      AX, N:_tlv+38      ;; 1 cycle
        BR        S:??tlv_detect_ac_dc_field_39  ;; 3 cycles
        ; ------------------------------------- Block: 4 cycles
??tlv_detect_ac_dc_field_38:
        MOVW      AX, N:_tlv+34      ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??tlv_detect_ac_dc_field_39:
        MOVW      N:_tlv+38, AX      ;; 1 cycle
//  450         tlv.Bnet_rms = (us16)sqrt((double)tlv.Bnet_rms_acc / tlv.avg_sample_counter);
        MOV       X, N:_tlv+26       ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
          CFI FunCall ?F_UL2F
        CALL      N:?F_UL2F          ;; 3 cycles
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+6
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+8
        MOVW      BC, N:_tlv+58      ;; 1 cycle
        MOVW      AX, N:_tlv+56      ;; 1 cycle
          CFI FunCall ?F_UL2F
        CALL      N:?F_UL2F          ;; 3 cycles
          CFI FunCall ?F_DIV
        CALL      N:?F_DIV           ;; 3 cycles
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+4
          CFI FunCall _sqrt
        CALL      _sqrt              ;; 3 cycles
          CFI FunCall ?F_F2SL
        CALL      N:?F_F2SL          ;; 3 cycles
        MOVW      N:_tlv+36, AX      ;; 1 cycle
//  451         tlv.temperature_avg = tlv.temperature_avg_acc / tlv.avg_sample_counter;
        MOV       X, N:_tlv+26       ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x0           ;; 1 cycle
        PUSH      BC                 ;; 1 cycle
          CFI CFA SP+6
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+8
        MOVW      BC, N:_tlv+62      ;; 1 cycle
        MOVW      AX, N:_tlv+60      ;; 1 cycle
          CFI FunCall ?SL_DIV_L03
        CALL      N:?SL_DIV_L03      ;; 3 cycles
        MOVW      N:_tlv+50, AX      ;; 1 cycle
//  452         tlv.temperature_avg = (s16)(1.1f * tlv.temperature_avg);
        MOVW      AX, #0x3F8C        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+10
        MOVW      AX, #0xCCCD        ;; 1 cycle
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+12
        MOVW      AX, N:_tlv+50      ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        SARW      AX, 0xF            ;; 1 cycle
        XCHW      AX, BC             ;; 1 cycle
          CFI FunCall ?F_SL2F
        CALL      N:?F_SL2F          ;; 3 cycles
          CFI FunCall ?F_MUL
        CALL      N:?F_MUL           ;; 3 cycles
          CFI FunCall ?F_F2SL
        CALL      N:?F_F2SL          ;; 3 cycles
        MOVW      N:_tlv+50, AX      ;; 1 cycle
//  453         tlv.temperature_avg -= 349;
        MOVW      AX, N:_tlv+50      ;; 1 cycle
        ADDW      AX, #0xFEA3        ;; 1 cycle
        MOVW      N:_tlv+50, AX      ;; 1 cycle
//  454         tlv.avg_sample_counter = 0;
        MOV       N:_tlv+26, #0x0    ;; 1 cycle
//  455         tlv.Bnet_avg_acc = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_tlv+52, AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_tlv+54, AX      ;; 1 cycle
//  456         tlv.Bnet_rms_acc = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_tlv+56, AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_tlv+58, AX      ;; 1 cycle
//  457         tlv.temperature_avg_acc = 0;
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_tlv+60, AX      ;; 1 cycle
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      N:_tlv+62, AX      ;; 1 cycle
//  458         tlv.Bx_max = tlv.Bx;
        MOVW      AX, N:_tlv+42      ;; 1 cycle
        MOVW      N:_tlv+2, AX       ;; 1 cycle
//  459         tlv.By_max = tlv.By;
        MOVW      AX, N:_tlv+44      ;; 1 cycle
        MOVW      N:_tlv+6, AX       ;; 1 cycle
//  460         tlv.Bz_max = tlv.Bz;
        MOVW      AX, N:_tlv+46      ;; 1 cycle
        MOVW      N:_tlv+10, AX      ;; 1 cycle
//  461         tlv.Bx_min = tlv.Bx;
        MOVW      AX, N:_tlv+42      ;; 1 cycle
        MOVW      N:_tlv, AX         ;; 1 cycle
//  462         tlv.By_min = tlv.By;
        MOVW      AX, N:_tlv+44      ;; 1 cycle
        MOVW      N:_tlv+4, AX       ;; 1 cycle
//  463         tlv.Bz_min = tlv.Bz;
        MOVW      AX, N:_tlv+46      ;; 1 cycle
        MOVW      N:_tlv+8, AX       ;; 1 cycle
//  464         
//  465         /* diagnosis of magnet sensor */
//  466         if(tlv.adc_hang_cnt > 5) 
        MOV       A, N:_tlv+31       ;; 1 cycle
        ADDW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+4
        CMP       A, #0x6            ;; 1 cycle
        BC        ??tlv_detect_ac_dc_field_37  ;; 4 cycles
        ; ------------------------------------- Block: 89 cycles
//  467 	{
//  468 	    if(tlv.fault_detect_count < 250)
        MOV       A, N:_tlv+29       ;; 1 cycle
        CMP       A, #0xFA           ;; 1 cycle
        SKNC                         ;; 1 cycle
        ; ------------------------------------- Block: 3 cycles
//  469             {
//  470                 tlv.fault_detect_count++;
        INC       N:_tlv+29          ;; 2 cycles
          CFI FunCall _tlv_clear_var
        ; ------------------------------------- Block: 2 cycles
//  471             }
//  472 	    tlv_clear_var();
??tlv493d_scan_0:
        CALL      _tlv_clear_var     ;; 3 cycles
//  473 	    /* Resetting and reonfiguring the sensor */ 
//  474             tlv_recovery();
          CFI FunCall _tlv_recovery
        CALL      _tlv_recovery      ;; 3 cycles
//  475             tlv493d_init();
          CFI FunCall _tlv493d_init
        CALL      _tlv493d_init      ;; 3 cycles
        ; ------------------------------------- Block: 9 cycles
//  476 //	    while(tlv493d_init() == MAG_I2C_ERROR)
//  477 //            {
//  478 //              delay_ms(10);
//  479 //              tlv_recovery();
//  480 //              delay_ms(10);
//  481 //              wdt_restart();
//  482 //            }
//  483 	}
//  484     }
//  485     tlv.sensor_reg_read[0]=0;
??tlv_detect_ac_dc_field_37:
        MOV       N:_tlv+12, #0x0    ;; 1 cycle
//  486     tlv.sensor_reg_read[1]=0;
        MOV       N:_tlv+13, #0x0    ;; 1 cycle
//  487     tlv.sensor_reg_read[2]=0;
        MOV       N:_tlv+14, #0x0    ;; 1 cycle
//  488     tlv.sensor_reg_read[3]=0;
        MOV       N:_tlv+15, #0x0    ;; 1 cycle
//  489     tlv.sensor_reg_read[4]=0;
        MOV       N:_tlv+16, #0x0    ;; 1 cycle
//  490     tlv.sensor_reg_read[5]=0;
        MOV       N:_tlv+17, #0x0    ;; 1 cycle
//  491     tlv.sensor_reg_read[6]=0;
        MOV       N:_tlv+18, #0x0    ;; 1 cycle
//  492     tlv.sensor_reg_read[7]=0;
        MOV       N:_tlv+19, #0x0    ;; 1 cycle
//  493     tlv.sensor_reg_read[8]=0;
        MOV       N:_tlv+20, #0x0    ;; 1 cycle
//  494     tlv.sensor_reg_read[9]=0;
        MOV       N:_tlv+21, #0x0    ;; 1 cycle
//  495 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock11
        ; ------------------------------------- Block: 16 cycles
        ; ------------------------------------- Total: 513 cycles
//  496 
//  497 
//  498 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock12 Using cfiCommon0
          CFI Function _tlv493d_powerdown
        CODE
//  499 void tlv493d_powerdown()
//  500 {
_tlv493d_powerdown:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  501     tlv.sensor_reg_write[0] = 0x00;                                // reser val 00
        MOV       N:_tlv+22, #0x0    ;; 1 cycle
//  502     
//  503     tlv.sensor_reg_write[1] = tlv.sensor_reg_read[0];              //7h 4:3
        MOV       A, N:_tlv+12       ;; 1 cycle
        MOV       N:_tlv+23, A       ;; 1 cycle
//  504     tlv.sensor_reg_write[1] = (tlv.sensor_reg_write[1] & 0x18);
        MOVW      HL, #LWRD(_tlv+23)  ;; 1 cycle
        MOV       A, [HL]            ;; 1 cycle
        AND       A, #0x18           ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
//  505     
//  506     tlv.sensor_reg_write[2] = tlv.sensor_reg_read[1];              //8h 7-0
        MOV       A, N:_tlv+13       ;; 1 cycle
        MOV       N:_tlv+24, A       ;; 1 cycle
//  507     
//  508     tlv.sensor_reg_write[3] = tlv.sensor_reg_read[2];              //9h 4-0
        MOV       A, N:_tlv+14       ;; 1 cycle
        MOV       N:_tlv+25, A       ;; 1 cycle
//  509     tlv.sensor_reg_write[3] = tlv.sensor_reg_write[3] & 0x3f;
        MOVW      HL, #LWRD(_tlv+25)  ;; 1 cycle
        MOV       A, [HL]            ;; 1 cycle
        AND       A, #0x3F           ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
//  510     
//  511     tlv_i2c_start();
          CFI FunCall _tlv_i2c_start
        CALL      _tlv_i2c_start     ;; 3 cycles
//  512     
//  513     if(flag_tlv_bus_busy == 0)
        MOVW      HL, #LWRD(_tlv_flag)  ;; 1 cycle
        MOV1      CY, [HL].3         ;; 1 cycle
        BC        ??tlv_detect_ac_dc_field_40  ;; 4 cycles
        ; ------------------------------------- Block: 24 cycles
//  514     {
//  515 	if((tlv_i2c_byte_write(ADDR_MAG_W)) == MAG_ACK)     	// Write Device Address
        MOV       A, #0xBC           ;; 1 cycle
          CFI FunCall _tlv_i2c_byte_write
        CALL      _tlv_i2c_byte_write  ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
//  516 	{
//  517 	    NOP();
        NOP                          ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
//  518 	}
//  519 	if((tlv_i2c_byte_write(tlv.sensor_reg_write[0])) == MAG_ACK)     	// Write Device Address
??tlv493d_powerdown_0:
        MOV       A, N:_tlv+22       ;; 1 cycle
          CFI FunCall _tlv_i2c_byte_write
        CALL      _tlv_i2c_byte_write  ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
//  520 	{
//  521 	    NOP();
        NOP                          ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
//  522 	}
//  523 	if((tlv_i2c_byte_write(tlv.sensor_reg_write[1])) == MAG_ACK)     	// Write Device Address
??tlv493d_powerdown_1:
        MOV       A, N:_tlv+23       ;; 1 cycle
          CFI FunCall _tlv_i2c_byte_write
        CALL      _tlv_i2c_byte_write  ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
//  524 	{
//  525 	    NOP();
        NOP                          ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
//  526 	}    
//  527 	if((tlv_i2c_byte_write(tlv.sensor_reg_write[2])) == MAG_ACK)     	// Write Device Address
??tlv493d_powerdown_2:
        MOV       A, N:_tlv+24       ;; 1 cycle
          CFI FunCall _tlv_i2c_byte_write
        CALL      _tlv_i2c_byte_write  ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
//  528 	{
//  529 	    NOP();
        NOP                          ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
//  530 	}    
//  531 	if((tlv_i2c_byte_write(tlv.sensor_reg_write[3])) == MAG_ACK)     	// Write Device Address
??tlv493d_powerdown_3:
        MOV       A, N:_tlv+25       ;; 1 cycle
          CFI FunCall _tlv_i2c_byte_write
        CALL      _tlv_i2c_byte_write  ;; 3 cycles
        CMP0      A                  ;; 1 cycle
        SKNZ                         ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
//  532 	{
//  533 	    NOP();
        NOP                          ;; 1 cycle
          CFI FunCall _tlv_i2c_stop
        ; ------------------------------------- Block: 1 cycles
//  534 	}
//  535     }  
//  536     tlv_i2c_stop();  
??tlv_detect_ac_dc_field_40:
        CALL      _tlv_i2c_stop      ;; 3 cycles
//  537 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock12
        ; ------------------------------------- Block: 9 cycles
        ; ------------------------------------- Total: 68 cycles
//  538 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock13 Using cfiCommon0
          CFI Function _tlv_cal_max_min_counts
        CODE
//  539 void tlv_cal_max_min_counts()
//  540 {
_tlv_cal_max_min_counts:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  541     tlv.Bx_max = MAX(tlv.Bx_max,tlv.Bx);
        MOVW      BC, N:_tlv+42      ;; 1 cycle
        MOVW      AX, N:_tlv+2       ;; 1 cycle
          CFI FunCall ?SI_CMP_L02
        CALL      N:?SI_CMP_L02      ;; 3 cycles
        BNC       ??tlv_detect_ac_dc_field_41  ;; 4 cycles
        ; ------------------------------------- Block: 9 cycles
        MOVW      AX, N:_tlv+42      ;; 1 cycle
        BR        S:??tlv_detect_ac_dc_field_42  ;; 3 cycles
        ; ------------------------------------- Block: 4 cycles
??tlv_detect_ac_dc_field_41:
        MOVW      AX, N:_tlv+2       ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??tlv_detect_ac_dc_field_42:
        MOVW      N:_tlv+2, AX       ;; 1 cycle
//  542     tlv.By_max = MAX(tlv.By_max,tlv.By);
        MOVW      BC, N:_tlv+44      ;; 1 cycle
        MOVW      AX, N:_tlv+6       ;; 1 cycle
          CFI FunCall ?SI_CMP_L02
        CALL      N:?SI_CMP_L02      ;; 3 cycles
        BNC       ??tlv_detect_ac_dc_field_43  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
        MOVW      AX, N:_tlv+44      ;; 1 cycle
        BR        S:??tlv_detect_ac_dc_field_44  ;; 3 cycles
        ; ------------------------------------- Block: 4 cycles
??tlv_detect_ac_dc_field_43:
        MOVW      AX, N:_tlv+6       ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??tlv_detect_ac_dc_field_44:
        MOVW      N:_tlv+6, AX       ;; 1 cycle
//  543     tlv.Bz_max = MAX(tlv.Bz_max,tlv.Bz);
        MOVW      BC, N:_tlv+46      ;; 1 cycle
        MOVW      AX, N:_tlv+10      ;; 1 cycle
          CFI FunCall ?SI_CMP_L02
        CALL      N:?SI_CMP_L02      ;; 3 cycles
        BNC       ??tlv_detect_ac_dc_field_45  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
        MOVW      AX, N:_tlv+46      ;; 1 cycle
        BR        S:??tlv_detect_ac_dc_field_46  ;; 3 cycles
        ; ------------------------------------- Block: 4 cycles
??tlv_detect_ac_dc_field_45:
        MOVW      AX, N:_tlv+10      ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??tlv_detect_ac_dc_field_46:
        MOVW      N:_tlv+10, AX      ;; 1 cycle
//  544     tlv.Bx_min = MIN(tlv.Bx_min,tlv.Bx);
        MOVW      BC, N:_tlv+42      ;; 1 cycle
        MOVW      AX, N:_tlv         ;; 1 cycle
          CFI FunCall ?SI_CMP_L02
        CALL      N:?SI_CMP_L02      ;; 3 cycles
        BNC       ??tlv_detect_ac_dc_field_47  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
        MOVW      AX, N:_tlv         ;; 1 cycle
        BR        S:??tlv_detect_ac_dc_field_48  ;; 3 cycles
        ; ------------------------------------- Block: 4 cycles
??tlv_detect_ac_dc_field_47:
        MOVW      AX, N:_tlv+42      ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??tlv_detect_ac_dc_field_48:
        MOVW      N:_tlv, AX         ;; 1 cycle
//  545     tlv.By_min = MIN(tlv.By_min,tlv.By);
        MOVW      BC, N:_tlv+44      ;; 1 cycle
        MOVW      AX, N:_tlv+4       ;; 1 cycle
          CFI FunCall ?SI_CMP_L02
        CALL      N:?SI_CMP_L02      ;; 3 cycles
        BNC       ??tlv_detect_ac_dc_field_49  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
        MOVW      AX, N:_tlv+4       ;; 1 cycle
        BR        S:??tlv_detect_ac_dc_field_50  ;; 3 cycles
        ; ------------------------------------- Block: 4 cycles
??tlv_detect_ac_dc_field_49:
        MOVW      AX, N:_tlv+44      ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??tlv_detect_ac_dc_field_50:
        MOVW      N:_tlv+4, AX       ;; 1 cycle
//  546     tlv.Bz_min = MIN(tlv.Bz_min,tlv.Bz);
        MOVW      BC, N:_tlv+46      ;; 1 cycle
        MOVW      AX, N:_tlv+8       ;; 1 cycle
          CFI FunCall ?SI_CMP_L02
        CALL      N:?SI_CMP_L02      ;; 3 cycles
        BNC       ??tlv_detect_ac_dc_field_51  ;; 4 cycles
        ; ------------------------------------- Block: 10 cycles
        MOVW      AX, N:_tlv+8       ;; 1 cycle
        BR        S:??tlv_detect_ac_dc_field_52  ;; 3 cycles
        ; ------------------------------------- Block: 4 cycles
??tlv_detect_ac_dc_field_51:
        MOVW      AX, N:_tlv+46      ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??tlv_detect_ac_dc_field_52:
        MOVW      N:_tlv+8, AX       ;; 1 cycle
//  547 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock13
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 96 cycles

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock14 Using cfiCommon0
          CFI Function _tlv_detect_ac_dc_field
          CFI NoCalls
        CODE
//  548 void tlv_detect_ac_dc_field()
//  549 {
_tlv_detect_ac_dc_field:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  550     if(((tlv.Bx_max - tlv.Bx_min) >= THR_AC_DIFF && ABS(tlv.Bx_max + tlv.Bx_min) < THR_AC_SUM) ||
//  551        ((tlv.By_max - tlv.By_min) >= THR_AC_DIFF && ABS(tlv.By_max + tlv.By_min) < THR_AC_SUM) ||
//  552            ((tlv.Bz_max - tlv.Bz_min) >= THR_AC_DIFF && ABS(tlv.Bz_max + tlv.Bz_min) < THR_AC_SUM))
        MOVW      AX, N:_tlv+2       ;; 1 cycle
        SUBW      AX, N:_tlv         ;; 1 cycle
        XOR       A, #0x80           ;; 1 cycle
        CMPW      AX, #0x803C        ;; 1 cycle
        BC        ??tlv_detect_ac_dc_field_53  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
        MOVW      AX, N:_tlv+2       ;; 1 cycle
        ADDW      AX, N:_tlv         ;; 1 cycle
        BF        A.7, ??tlv_detect_ac_dc_field_54  ;; 5 cycles
        ; ------------------------------------- Block: 7 cycles
        MOVW      AX, N:_tlv+2       ;; 1 cycle
        ADDW      AX, N:_tlv         ;; 1 cycle
        XOR       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        XOR       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        INCW      AX                 ;; 1 cycle
        BR        S:??tlv_detect_ac_dc_field_55  ;; 3 cycles
        ; ------------------------------------- Block: 10 cycles
??tlv_detect_ac_dc_field_54:
        MOVW      AX, N:_tlv+2       ;; 1 cycle
        ADDW      AX, N:_tlv         ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
??tlv_detect_ac_dc_field_55:
        XOR       A, #0x80           ;; 1 cycle
        CMPW      AX, #0x8032        ;; 1 cycle
        BC        ??tlv_detect_ac_dc_field_56  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
??tlv_detect_ac_dc_field_53:
        MOVW      AX, N:_tlv+6       ;; 1 cycle
        SUBW      AX, N:_tlv+4       ;; 1 cycle
        XOR       A, #0x80           ;; 1 cycle
        CMPW      AX, #0x803C        ;; 1 cycle
        BC        ??tlv_detect_ac_dc_field_57  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
        MOVW      AX, N:_tlv+6       ;; 1 cycle
        ADDW      AX, N:_tlv+4       ;; 1 cycle
        BF        A.7, ??tlv_detect_ac_dc_field_58  ;; 5 cycles
        ; ------------------------------------- Block: 7 cycles
        MOVW      AX, N:_tlv+6       ;; 1 cycle
        ADDW      AX, N:_tlv+4       ;; 1 cycle
        XOR       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        XOR       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        INCW      AX                 ;; 1 cycle
        BR        S:??tlv_detect_ac_dc_field_59  ;; 3 cycles
        ; ------------------------------------- Block: 10 cycles
??tlv_detect_ac_dc_field_58:
        MOVW      AX, N:_tlv+6       ;; 1 cycle
        ADDW      AX, N:_tlv+4       ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
??tlv_detect_ac_dc_field_59:
        XOR       A, #0x80           ;; 1 cycle
        CMPW      AX, #0x8032        ;; 1 cycle
        BC        ??tlv_detect_ac_dc_field_56  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
??tlv_detect_ac_dc_field_57:
        MOVW      AX, N:_tlv+10      ;; 1 cycle
        SUBW      AX, N:_tlv+8       ;; 1 cycle
        XOR       A, #0x80           ;; 1 cycle
        CMPW      AX, #0x803C        ;; 1 cycle
        BC        ??tlv_detect_ac_dc_field_60  ;; 4 cycles
        ; ------------------------------------- Block: 8 cycles
        MOVW      AX, N:_tlv+10      ;; 1 cycle
        ADDW      AX, N:_tlv+8       ;; 1 cycle
        BF        A.7, ??tlv_detect_ac_dc_field_61  ;; 5 cycles
        ; ------------------------------------- Block: 7 cycles
        MOVW      AX, N:_tlv+10      ;; 1 cycle
        ADDW      AX, N:_tlv+8       ;; 1 cycle
        XOR       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        XOR       A, #0xFF           ;; 1 cycle
        XCH       A, X               ;; 1 cycle
        INCW      AX                 ;; 1 cycle
        BR        S:??tlv_detect_ac_dc_field_62  ;; 3 cycles
        ; ------------------------------------- Block: 10 cycles
??tlv_detect_ac_dc_field_61:
        MOVW      AX, N:_tlv+10      ;; 1 cycle
        ADDW      AX, N:_tlv+8       ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
??tlv_detect_ac_dc_field_62:
        XOR       A, #0x80           ;; 1 cycle
        CMPW      AX, #0x8032        ;; 1 cycle
        BNC       ??tlv_detect_ac_dc_field_60  ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  553     {
//  554         flag_tlv_ac_field = 1;
??tlv_detect_ac_dc_field_56:
        SET1      N:_tlv_flag.5      ;; 2 cycles
        BR        S:??tlv_detect_ac_dc_field_63  ;; 3 cycles
        ; ------------------------------------- Block: 5 cycles
//  555     }   
//  556     else
//  557     {
//  558         flag_tlv_ac_field = 0;
??tlv_detect_ac_dc_field_60:
        CLR1      N:_tlv_flag.5      ;; 2 cycles
        ; ------------------------------------- Block: 2 cycles
//  559     }
//  560     
//  561     tlv.bx_sum = (tlv.Bx_max + tlv.Bx_min);
??tlv_detect_ac_dc_field_63:
        MOVW      AX, N:_tlv+2       ;; 1 cycle
        ADDW      AX, N:_tlv         ;; 1 cycle
        MOVW      N:_tlv+64, AX      ;; 1 cycle
//  562     tlv.bx_diff = (tlv.Bx_max - tlv.Bx_min);
        MOVW      AX, N:_tlv+2       ;; 1 cycle
        SUBW      AX, N:_tlv         ;; 1 cycle
        MOVW      N:_tlv+66, AX      ;; 1 cycle
//  563     tlv.by_sum = (tlv.By_max + tlv.By_min);
        MOVW      AX, N:_tlv+6       ;; 1 cycle
        ADDW      AX, N:_tlv+4       ;; 1 cycle
        MOVW      N:_tlv+68, AX      ;; 1 cycle
//  564     tlv.by_diff = (tlv.By_max - tlv.By_min);
        MOVW      AX, N:_tlv+6       ;; 1 cycle
        SUBW      AX, N:_tlv+4       ;; 1 cycle
        MOVW      N:_tlv+70, AX      ;; 1 cycle
//  565     tlv.bz_sum = (tlv.Bz_max + tlv.Bz_min);
        MOVW      AX, N:_tlv+10      ;; 1 cycle
        ADDW      AX, N:_tlv+8       ;; 1 cycle
        MOVW      N:_tlv+72, AX      ;; 1 cycle
//  566     tlv.bz_diff = (tlv.Bz_max - tlv.Bz_min);
        MOVW      AX, N:_tlv+10      ;; 1 cycle
        SUBW      AX, N:_tlv+8       ;; 1 cycle
        MOVW      N:_tlv+74, AX      ;; 1 cycle
//  567 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock14
        ; ------------------------------------- Block: 24 cycles
        ; ------------------------------------- Total: 130 cycles

        SECTION `.iar_vfe_header`:DATA:NOALLOC:NOROOT(1)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
        DC32 0

        END
// 
//    77 bytes in section .bss
//     2 bytes in section .bss.noinit   (abs)
//     1 byte  in section .sbss.noinit  (abs)
// 2'243 bytes in section .text
// 
// 2'243 bytes of CODE memory
//    77 bytes of DATA memory (+ 3 bytes shared)
//
//Errors: none
//Warnings: none
