///////////////////////////////////////////////////////////////////////////////
//
// IAR C/C++ Compiler V4.20.1.2260 for RL78               20/Dec/2020  22:39:03
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
//        BootCode\source_code\source_files\self_test.c
//    Command line       =
//        -f C:\Users\laptop\AppData\Local\Temp\EWB1D.tmp ("D:\Dheeraj\New
//        folder\0. GDEV72 - BootCode\source_code\source_files\self_test.c"
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
//        D:\Dheeraj\New folder\0. GDEV72 - BootCode\Debug\List\self_test.s
//
///////////////////////////////////////////////////////////////////////////////

        RTMODEL "__SystemLibrary", "DLib"
        RTMODEL "__calling_convention", "v2"
        RTMODEL "__code_model", "near"
        RTMODEL "__core", "s3"
        RTMODEL "__data_model", "near"
        RTMODEL "__double_size", "32"
        RTMODEL "__far_rt_calls", "false"
        RTMODEL "__rt_version", "2"

        #define SHT_PROGBITS 0x1


        SECTION `.iar_vfe_header`:DATA:NOALLOC:NOROOT(1)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
        DC32 0

        END
// D:\Dheeraj\New folder\0. GDEV72 - BootCode\source_code\source_files\self_test.c
//    1 /***********************************************************************************************************************
//    2 * File Name       : self_test.c
//    3 * Current Version : rev_01  
//    4 * Tool-Chain      : IAR Systems RL78
//    5 * Description     : this library includes routines which are used for self test software and hardware in run time as well as 
//    6                     at manufacturing time
//    7 * Creation Date   : 05-06-2020
//    8 * Company         : Genus Power Infrastructures Limited, Jaipur
//    9 * Author          : dheeraj.singhal
//   10 * Version History : rev_01 :
//   11 ***********************************************************************************************************************/
//   12 /************************************ Includes **************************************/
//   13 #include "self_test.h"
//   14 /************************************ Local Variables *****************************************/
//   15 /************************************ Extern Variables *****************************************/
//   16 /************************************ Local Functions *******************************/
//   17 /************************************ Extern Functions ******************************/
// 
// 
// 0 bytes of memory
//
//Errors: none
//Warnings: none
