///////////////////////////////////////////////////////////////////////////////
//
// IAR C/C++ Compiler V4.20.1.2260 for RL78               20/Dec/2020  22:38:01
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
//        BootCode\source_code\source_files\AES.c
//    Command line       =
//        -f C:\Users\laptop\AppData\Local\Temp\EWDF6.tmp ("D:\Dheeraj\New
//        folder\0. GDEV72 - BootCode\source_code\source_files\AES.c" --core s3
//        --code_model near --calling_convention v2 --near_const_location ram
//        -o "D:\Dheeraj\New folder\0. GDEV72 - BootCode\Debug\Obj"
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
//        D:\Dheeraj\New folder\0. GDEV72 - BootCode\Debug\List\AES.s
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

        EXTERN ?SI_MOD_L02

        PUBLIC _AddRoundKey
        PUBLIC _Cipher
        PUBLIC _InvCipher
        PUBLIC _InvMixColumns
        PUBLIC _InvShiftRows
        PUBLIC _Key
        PUBLIC _KeyExpansion
        PUBLIC _MixColumns
        PUBLIC _Rcon
        PUBLIC _RoundKey
        PUBLIC _SBox
        PUBLIC _S_r_Box
        PUBLIC _in
        PUBLIC _out
        PUBLIC _r_SBox
        PUBLIC _shiftrows_left
        PUBLIC _state
        
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
        
// D:\Dheeraj\New folder\0. GDEV72 - BootCode\source_code\source_files\AES.c
//    1 /************************************************************************************
//    2 *																					*
//    3 *  File Name	: AES.c
//    4 *  Contents	: encryption and decryption as NIST FIPS PUB 197:2001
//    5 *  Copyright	: Genus Power Infrastructure Ltd.
//    6 *  Version	: 1.0
//    7 *  note        :
//    8 *  Author      : Bijendra Bhagasra
//    9 
//   10 ***********************************************************************************/
//   11 #include "AES.h"
//   12 
//   13 #define Nb 4
//   14 #define Nk 4
//   15 #define Nr 10
//   16 #define xtime(x)   (((x) << 1) ^ ((((x)>>7) & 1) * 0x1b))
//   17 #define Multiply(x,y) ((((y) & 1) * (x)) ^ (((y)>>1 & 1) * xtime(x)) ^ (((y)>>2 & 1) * xtime(xtime(x))) ^ (((y)>>3 & 1) * xtime(xtime(xtime(x)))) ^ (((y)>>4 & 1) * xtime(xtime(xtime(xtime(x))))))
//   18 

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   19 unsigned char out[16], state[4][4];
_out:
        DS 16

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
_state:
        DS 16

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   20 unsigned char in[16]; /* {0x2b,0x7e,0x15,0x16,0x28,0xae,0xd2,0xa6,0xab,0xf7,0x15,0x88,0x09,0xcf,0x4f,0x3c}; */
_in:
        DS 16

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   21 unsigned char RoundKey[176];
_RoundKey:
        DS 176
//   22 void S_r_Box(unsigned char sat);

        SECTION `.bss`:DATA:REORDER:NOROOT(1)
//   23 unsigned char Key[16]; /* = {0x2b ,0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6, 0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c}; */
_Key:
        DS 16
//   24 void MixColumns(void);
//   25 void InvShiftRows(void);
//   26 void InvMixColumns(void);
//   27 

        SECTION `.data`:DATA:REORDER:NOROOT(1)
//   28 const unsigned char SBox[256] =   {
_SBox:
        DATA8
        DB 99, 124, 119, 123, 242, 107, 111, 197, 48, 1, 103, 43, 254, 215, 171
        DB 118, 202, 130, 201, 125, 250, 89, 71, 240, 173, 212, 162, 175, 156
        DB 164, 114, 192, 183, 253, 147, 38, 54, 63, 247, 204, 52, 165, 229
        DB 241, 113, 216, 49, 21, 4, 199, 35, 195, 24, 150, 5, 154, 7, 18, 128
        DB 226, 235, 39, 178, 117, 9, 131, 44, 26, 27, 110, 90, 160, 82, 59
        DB 214, 179, 41, 227, 47, 132, 83, 209, 0, 237, 32, 252, 177, 91, 106
        DB 203, 190, 57, 74, 76, 88, 207, 208, 239, 170, 251, 67, 77, 51, 133
        DB 69, 249, 2, 127, 80, 60, 159, 168, 81, 163, 64, 143, 146, 157, 56
        DB 245, 188, 182, 218, 33, 16, 255, 243, 210, 205, 12, 19, 236, 95, 151
        DB 68, 23, 196, 167, 126, 61, 100, 93, 25, 115, 96, 129, 79, 220, 34
        DB 42, 144, 136, 70, 238, 184, 20, 222, 94, 11, 219, 224, 50, 58, 10
        DB 73, 6, 36, 92, 194, 211, 172, 98, 145, 149, 228, 121, 231, 200, 55
        DB 109, 141, 213, 78, 169, 108, 86, 244, 234, 101, 122, 174, 8, 186
        DB 120, 37, 46, 28, 166, 180, 198, 232, 221, 116, 31, 75, 189, 139, 138
        DB 112, 62, 181, 102, 72, 3, 246, 14, 97, 53, 87, 185, 134, 193, 29
        DB 158, 225, 248, 152, 17, 105, 217, 142, 148, 155, 30, 135, 233, 206
        DB 85, 40, 223, 140, 161, 137, 13, 191, 230, 66, 104, 65, 153, 45, 15
        DB 176, 84, 187, 22
//   29   /* 0     1    2      3     4    5     6     7      8    9     A      B    C     D     E     F */
//   30   0x63, 0x7c, 0x77, 0x7b, 0xf2, 0x6b, 0x6f, 0xc5, 0x30, 0x01, 0x67, 0x2b, 0xfe, 0xd7, 0xab, 0x76, /* 0 */
//   31   0xca, 0x82, 0xc9, 0x7d, 0xfa, 0x59, 0x47, 0xf0, 0xad, 0xd4, 0xa2, 0xaf, 0x9c, 0xa4, 0x72, 0xc0, /* 1 */
//   32   0xb7, 0xfd, 0x93, 0x26, 0x36, 0x3f, 0xf7, 0xcc, 0x34, 0xa5, 0xe5, 0xf1, 0x71, 0xd8, 0x31, 0x15, /* 2 */
//   33   0x04, 0xc7, 0x23, 0xc3, 0x18, 0x96, 0x05, 0x9a, 0x07, 0x12, 0x80, 0xe2, 0xeb, 0x27, 0xb2, 0x75, /* 3 */
//   34   0x09, 0x83, 0x2c, 0x1a, 0x1b, 0x6e, 0x5a, 0xa0, 0x52, 0x3b, 0xd6, 0xb3, 0x29, 0xe3, 0x2f, 0x84, /* 4 */
//   35   0x53, 0xd1, 0x00, 0xed, 0x20, 0xfc, 0xb1, 0x5b, 0x6a, 0xcb, 0xbe, 0x39, 0x4a, 0x4c, 0x58, 0xcf, /* 5 */
//   36   0xd0, 0xef, 0xaa, 0xfb, 0x43, 0x4d, 0x33, 0x85, 0x45, 0xf9, 0x02, 0x7f, 0x50, 0x3c, 0x9f, 0xa8, /* 6 */
//   37   0x51, 0xa3, 0x40, 0x8f, 0x92, 0x9d, 0x38, 0xf5, 0xbc, 0xb6, 0xda, 0x21, 0x10, 0xff, 0xf3, 0xd2, /* 7 */
//   38   0xcd, 0x0c, 0x13, 0xec, 0x5f, 0x97, 0x44, 0x17, 0xc4, 0xa7, 0x7e, 0x3d, 0x64, 0x5d, 0x19, 0x73, /* 8 */
//   39   0x60, 0x81, 0x4f, 0xdc, 0x22, 0x2a, 0x90, 0x88, 0x46, 0xee, 0xb8, 0x14, 0xde, 0x5e, 0x0b, 0xdb, /* 9 */
//   40   0xe0, 0x32, 0x3a, 0x0a, 0x49, 0x06, 0x24, 0x5c, 0xc2, 0xd3, 0xac, 0x62, 0x91, 0x95, 0xe4, 0x79, /* A */
//   41   0xe7, 0xc8, 0x37, 0x6d, 0x8d, 0xd5, 0x4e, 0xa9, 0x6c, 0x56, 0xf4, 0xea, 0x65, 0x7a, 0xae, 0x08, /* B */
//   42   0xba, 0x78, 0x25, 0x2e, 0x1c, 0xa6, 0xb4, 0xc6, 0xe8, 0xdd, 0x74, 0x1f, 0x4b, 0xbd, 0x8b, 0x8a, /* C */
//   43   0x70, 0x3e, 0xb5, 0x66, 0x48, 0x03, 0xf6, 0x0e, 0x61, 0x35, 0x57, 0xb9, 0x86, 0xc1, 0x1d, 0x9e, /* D */
//   44   0xe1, 0xf8, 0x98, 0x11, 0x69, 0xd9, 0x8e, 0x94, 0x9b, 0x1e, 0x87, 0xe9, 0xce, 0x55, 0x28, 0xdf, /* E */
//   45   0x8c, 0xa1, 0x89, 0x0d, 0xbf, 0xe6, 0x42, 0x68, 0x41, 0x99, 0x2d, 0x0f, 0xb0, 0x54, 0xbb, 0x16
//   46 };
//   47 

        SECTION `.data`:DATA:REORDER:NOROOT(1)
//   48 const unsigned char r_SBox[256] =
_r_SBox:
        DATA8
        DB 82, 9, 106, 213, 48, 54, 165, 56, 191, 64, 163, 158, 129, 243, 215
        DB 251, 124, 227, 57, 130, 155, 47, 255, 135, 52, 142, 67, 68, 196, 222
        DB 233, 203, 84, 123, 148, 50, 166, 194, 35, 61, 238, 76, 149, 11, 66
        DB 250, 195, 78, 8, 46, 161, 102, 40, 217, 36, 178, 118, 91, 162, 73
        DB 109, 139, 209, 37, 114, 248, 246, 100, 134, 104, 152, 22, 212, 164
        DB 92, 204, 93, 101, 182, 146, 108, 112, 72, 80, 253, 237, 185, 218, 94
        DB 21, 70, 87, 167, 141, 157, 132, 144, 216, 171, 0, 140, 188, 211, 10
        DB 247, 228, 88, 5, 184, 179, 69, 6, 208, 44, 30, 143, 202, 63, 15, 2
        DB 193, 175, 189, 3, 1, 19, 138, 107, 58, 145, 17, 65, 79, 103, 220
        DB 234, 151, 242, 207, 206, 240, 180, 230, 115, 150, 172, 116, 34, 231
        DB 173, 53, 133, 226, 249, 55, 232, 28, 117, 223, 110, 71, 241, 26, 113
        DB 29, 41, 197, 137, 111, 183, 98, 14, 170, 24, 190, 27, 252, 86, 62
        DB 75, 198, 210, 121, 32, 154, 219, 192, 254, 120, 205, 90, 244, 31
        DB 221, 168, 51, 136, 7, 199, 49, 177, 18, 16, 89, 39, 128, 236, 95, 96
        DB 81, 127, 169, 25, 181, 74, 13, 45, 229, 122, 159, 147, 201, 156, 239
        DB 160, 224, 59, 77, 174, 42, 245, 176, 200, 235, 187, 60, 131, 83, 153
        DB 97, 23, 43, 4, 126, 186, 119, 214, 38, 225, 105, 20, 99, 85, 33, 12
        DB 125
//   49 { 0x52, 0x09, 0x6a, 0xd5, 0x30, 0x36, 0xa5, 0x38, 0xbf, 0x40, 0xa3, 0x9e, 0x81, 0xf3, 0xd7, 0xfb,
//   50 0x7c, 0xe3, 0x39, 0x82, 0x9b, 0x2f, 0xff, 0x87, 0x34, 0x8e, 0x43, 0x44, 0xc4, 0xde, 0xe9, 0xcb,
//   51 0x54, 0x7b, 0x94, 0x32, 0xa6, 0xc2, 0x23, 0x3d, 0xee, 0x4c, 0x95, 0x0b, 0x42, 0xfa, 0xc3, 0x4e,
//   52 0x08, 0x2e, 0xa1, 0x66, 0x28, 0xd9, 0x24, 0xb2, 0x76, 0x5b, 0xa2, 0x49, 0x6d, 0x8b, 0xd1, 0x25,
//   53 0x72, 0xf8, 0xf6, 0x64, 0x86, 0x68, 0x98, 0x16, 0xd4, 0xa4, 0x5c, 0xcc, 0x5d, 0x65, 0xb6, 0x92,
//   54 0x6c, 0x70, 0x48, 0x50, 0xfd, 0xed, 0xb9, 0xda, 0x5e, 0x15, 0x46, 0x57, 0xa7, 0x8d, 0x9d, 0x84,
//   55 0x90, 0xd8, 0xab, 0x00, 0x8c, 0xbc, 0xd3, 0x0a, 0xf7, 0xe4, 0x58, 0x05, 0xb8, 0xb3, 0x45, 0x06,
//   56 0xd0, 0x2c, 0x1e, 0x8f, 0xca, 0x3f, 0x0f, 0x02, 0xc1, 0xaf, 0xbd, 0x03, 0x01, 0x13, 0x8a, 0x6b,
//   57 0x3a, 0x91, 0x11, 0x41, 0x4f, 0x67, 0xdc, 0xea, 0x97, 0xf2, 0xcf, 0xce, 0xf0, 0xb4, 0xe6, 0x73,
//   58 0x96, 0xac, 0x74, 0x22, 0xe7, 0xad, 0x35, 0x85, 0xe2, 0xf9, 0x37, 0xe8, 0x1c, 0x75, 0xdf, 0x6e,
//   59 0x47, 0xf1, 0x1a, 0x71, 0x1d, 0x29, 0xc5, 0x89, 0x6f, 0xb7, 0x62, 0x0e, 0xaa, 0x18, 0xbe, 0x1b,
//   60 0xfc, 0x56, 0x3e, 0x4b, 0xc6, 0xd2, 0x79, 0x20, 0x9a, 0xdb, 0xc0, 0xfe, 0x78, 0xcd, 0x5a, 0xf4,
//   61 0x1f, 0xdd, 0xa8, 0x33, 0x88, 0x07, 0xc7, 0x31, 0xb1, 0x12, 0x10, 0x59, 0x27, 0x80, 0xec, 0x5f,
//   62 0x60, 0x51, 0x7f, 0xa9, 0x19, 0xb5, 0x4a, 0x0d, 0x2d, 0xe5, 0x7a, 0x9f, 0x93, 0xc9, 0x9c, 0xef,
//   63 0xa0, 0xe0, 0x3b, 0x4d, 0xae, 0x2a, 0xf5, 0xb0, 0xc8, 0xeb, 0xbb, 0x3c, 0x83, 0x53, 0x99, 0x61,
//   64 0x17, 0x2b, 0x04, 0x7e, 0xba, 0x77, 0xd6, 0x26, 0xe1, 0x69, 0x14, 0x63, 0x55, 0x21, 0x0c, 0x7d };
//   65 

        SECTION `.data`:DATA:REORDER:NOROOT(1)
//   66 const unsigned char Rcon[255] = {
_Rcon:
        DATA8
        DB 141, 1, 2, 4, 8, 16, 32, 64, 128, 27, 54, 108, 216, 171, 77, 154, 47
        DB 94, 188, 99, 198, 151, 53, 106, 212, 179, 125, 250, 239, 197, 145
        DB 57, 114, 228, 211, 189, 97, 194, 159, 37, 74, 148, 51, 102, 204, 131
        DB 29, 58, 116, 232, 203, 141, 1, 2, 4, 8, 16, 32, 64, 128, 27, 54, 108
        DB 216, 171, 77, 154, 47, 94, 188, 99, 198, 151, 53, 106, 212, 179, 125
        DB 250, 239, 197, 145, 57, 114, 228, 211, 189, 97, 194, 159, 37, 74
        DB 148, 51, 102, 204, 131, 29, 58, 116, 232, 203, 141, 1, 2, 4, 8, 16
        DB 32, 64, 128, 27, 54, 108, 216, 171, 77, 154, 47, 94, 188, 99, 198
        DB 151, 53, 106, 212, 179, 125, 250, 239, 197, 145, 57, 114, 228, 211
        DB 189, 97, 194, 159, 37, 74, 148, 51, 102, 204, 131, 29, 58, 116, 232
        DB 203, 141, 1, 2, 4, 8, 16, 32, 64, 128, 27, 54, 108, 216, 171, 77
        DB 154, 47, 94, 188, 99, 198, 151, 53, 106, 212, 179, 125, 250, 239
        DB 197, 145, 57, 114, 228, 211, 189, 97, 194, 159, 37, 74, 148, 51, 102
        DB 204, 131, 29, 58, 116, 232, 203, 141, 1, 2, 4, 8, 16, 32, 64, 128
        DB 27, 54, 108, 216, 171, 77, 154, 47, 94, 188, 99, 198, 151, 53, 106
        DB 212, 179, 125, 250, 239, 197, 145, 57, 114, 228, 211, 189, 97, 194
        DB 159, 37, 74, 148, 51, 102, 204, 131, 29, 58, 116, 232, 203, 0
//   67   0x8d, 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x1b, 0x36, 0x6c, 0xd8, 0xab, 0x4d, 0x9a,
//   68   0x2f, 0x5e, 0xbc, 0x63, 0xc6, 0x97, 0x35, 0x6a, 0xd4, 0xb3, 0x7d, 0xfa, 0xef, 0xc5, 0x91, 0x39,
//   69   0x72, 0xe4, 0xd3, 0xbd, 0x61, 0xc2, 0x9f, 0x25, 0x4a, 0x94, 0x33, 0x66, 0xcc, 0x83, 0x1d, 0x3a,
//   70   0x74, 0xe8, 0xcb, 0x8d, 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x1b, 0x36, 0x6c, 0xd8,
//   71   0xab, 0x4d, 0x9a, 0x2f, 0x5e, 0xbc, 0x63, 0xc6, 0x97, 0x35, 0x6a, 0xd4, 0xb3, 0x7d, 0xfa, 0xef,
//   72   0xc5, 0x91, 0x39, 0x72, 0xe4, 0xd3, 0xbd, 0x61, 0xc2, 0x9f, 0x25, 0x4a, 0x94, 0x33, 0x66, 0xcc,
//   73   0x83, 0x1d, 0x3a, 0x74, 0xe8, 0xcb, 0x8d, 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x1b,
//   74   0x36, 0x6c, 0xd8, 0xab, 0x4d, 0x9a, 0x2f, 0x5e, 0xbc, 0x63, 0xc6, 0x97, 0x35, 0x6a, 0xd4, 0xb3,
//   75   0x7d, 0xfa, 0xef, 0xc5, 0x91, 0x39, 0x72, 0xe4, 0xd3, 0xbd, 0x61, 0xc2, 0x9f, 0x25, 0x4a, 0x94,
//   76   0x33, 0x66, 0xcc, 0x83, 0x1d, 0x3a, 0x74, 0xe8, 0xcb, 0x8d, 0x01, 0x02, 0x04, 0x08, 0x10, 0x20,
//   77   0x40, 0x80, 0x1b, 0x36, 0x6c, 0xd8, 0xab, 0x4d, 0x9a, 0x2f, 0x5e, 0xbc, 0x63, 0xc6, 0x97, 0x35,
//   78   0x6a, 0xd4, 0xb3, 0x7d, 0xfa, 0xef, 0xc5, 0x91, 0x39, 0x72, 0xe4, 0xd3, 0xbd, 0x61, 0xc2, 0x9f,
//   79   0x25, 0x4a, 0x94, 0x33, 0x66, 0xcc, 0x83, 0x1d, 0x3a, 0x74, 0xe8, 0xcb, 0x8d, 0x01, 0x02, 0x04,
//   80   0x08, 0x10, 0x20, 0x40, 0x80, 0x1b, 0x36, 0x6c, 0xd8, 0xab, 0x4d, 0x9a, 0x2f, 0x5e, 0xbc, 0x63,
//   81   0xc6, 0x97, 0x35, 0x6a, 0xd4, 0xb3, 0x7d, 0xfa, 0xef, 0xc5, 0x91, 0x39, 0x72, 0xe4, 0xd3, 0xbd,
//   82   0x61, 0xc2, 0x9f, 0x25, 0x4a, 0x94, 0x33, 0x66, 0xcc, 0x83, 0x1d, 0x3a, 0x74, 0xe8, 0xcb
//   83 };
//   84 
//   85 /************************************************/
//   86 void KeyExpansion(unsigned char *key);
//   87 void AddRoundKey(unsigned char round);
//   88 void S_r_Box(unsigned char sat);
//   89 void shiftrows_left(void);
//   90 void MixColumns(void);
//   91 void InvShiftRows(void);
//   92 void InvMixColumns(void);
//   93 void InvCipher(unsigned char *input);
//   94 void Cipher(unsigned char *input);
//   95 /*************************************************/

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock0 Using cfiCommon0
          CFI Function _KeyExpansion
        CODE
//   96 void KeyExpansion(unsigned char *key)
//   97 {
_KeyExpansion:
        ; * Stack frame (at entry) *
        ; Param size: 0
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+6
        ; Auto size: 10
        SUBW      SP, #0x8           ;; 1 cycle
          CFI CFA SP+14
//   98   int ls8_i,ls8_j;
//   99   unsigned char temp[4],lu8_k,pp=1;
        MOV       A, #0x1            ;; 1 cycle
        MOV       [SP+0x02], A       ;; 1 cycle
//  100   for(ls8_i=0; ls8_i<16; ls8_i++)
        MOVW      AX, #0x0           ;; 1 cycle
        MOVW      [SP], AX           ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
??KeyExpansion_0:
        MOVW      AX, [SP]           ;; 1 cycle
        XOR       A, #0x80           ;; 1 cycle
        CMPW      AX, #0x8010        ;; 1 cycle
        BNC       ??Cipher_0         ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  101   {
//  102     Key[ls8_i] = *(key+ls8_i);
        MOVW      AX, [SP]           ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [SP+0x08]      ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, [HL]            ;; 1 cycle
        MOV       H, A               ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        XCH       A, H               ;; 1 cycle
        MOV       (_Key)[BC], A      ;; 1 cycle
        XCH       A, H               ;; 1 cycle
//  103   }
        MOVW      AX, [SP]           ;; 1 cycle
        INCW      AX                 ;; 1 cycle
        MOVW      [SP], AX           ;; 1 cycle
        BR        S:??KeyExpansion_0  ;; 3 cycles
        ; ------------------------------------- Block: 18 cycles
//  104   for(ls8_i=0; ls8_i<(Nk*4); ls8_i++)
??Cipher_0:
        MOVW      BC, #0x0           ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??KeyExpansion_1:
        MOVW      AX, BC             ;; 1 cycle
        XOR       A, #0x80           ;; 1 cycle
        CMPW      AX, #0x8010        ;; 1 cycle
        BNC       ??Cipher_1         ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  105   {
//  106     RoundKey[ls8_i]= Key[ls8_i];
        MOV       A, (_Key)[BC]      ;; 1 cycle
        MOV       (_RoundKey)[BC], A  ;; 1 cycle
//  107   }
        INCW      BC                 ;; 1 cycle
        BR        S:??KeyExpansion_1  ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
//  108   ls8_i = Nk*4;
??Cipher_1:
        MOVW      AX, #0x10          ;; 1 cycle
        MOVW      [SP], AX           ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
//  109   while (ls8_i < 176)
??KeyExpansion_2:
        MOVW      AX, [SP]           ;; 1 cycle
        XOR       A, #0x80           ;; 1 cycle
        CMPW      AX, #0x80B0        ;; 1 cycle
        SKC                          ;; 4 cycles
        BR        N:??Cipher_2       ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  110   {
//  111     for(ls8_j=0; ls8_j<4; ls8_j++)
        MOVW      HL, #0x0           ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??KeyExpansion_3:
        MOVW      AX, HL             ;; 1 cycle
        XOR       A, #0x80           ;; 1 cycle
        CMPW      AX, #0x8004        ;; 1 cycle
        BNC       ??Cipher_3         ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  112     {
//  113       temp[ls8_j] = RoundKey[(ls8_i-4) + ls8_j];
        MOVW      AX, [SP]           ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOV       A, (_RoundKey-4)[BC]  ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        MOVW      DE, AX             ;; 1 cycle
        MOV       A, B               ;; 1 cycle
        MOV       [DE], A            ;; 1 cycle
//  114     }
        INCW      HL                 ;; 1 cycle
        BR        S:??KeyExpansion_3  ;; 3 cycles
        ; ------------------------------------- Block: 15 cycles
//  115     if (ls8_i % 16 == 0)
??Cipher_3:
        MOVW      BC, #0x10          ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
          CFI FunCall ?SI_MOD_L02
        CALL      N:?SI_MOD_L02      ;; 3 cycles
        MOVW      DE, AX             ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        CMPW      AX, #0x0           ;; 1 cycle
        XCHW      AX, DE             ;; 1 cycle
        BNZ       ??Cipher_4         ;; 4 cycles
        ; ------------------------------------- Block: 13 cycles
//  116     {
//  117       lu8_k = temp[0];                     /* rotation+assignSBox+leftmostgetRCon */
        MOV       A, [SP+0x04]       ;; 1 cycle
        MOV       [SP+0x03], A       ;; 1 cycle
//  118       temp[0] = SBox[temp[1]];
        MOV       A, [SP+0x05]       ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        MOV       A, (_SBox)[B]      ;; 1 cycle
        MOV       [SP+0x04], A       ;; 1 cycle
//  119       temp[1] = SBox[temp[2]];
        MOV       A, [SP+0x06]       ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        MOV       A, (_SBox)[B]      ;; 1 cycle
        MOV       [SP+0x05], A       ;; 1 cycle
//  120       temp[2] = SBox[temp[3]];
        MOV       A, [SP+0x07]       ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        MOV       A, (_SBox)[B]      ;; 1 cycle
        MOV       [SP+0x06], A       ;; 1 cycle
//  121       temp[3] = SBox[lu8_k];
        MOV       A, [SP+0x03]       ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        MOV       A, (_SBox)[B]      ;; 1 cycle
        MOV       [SP+0x07], A       ;; 1 cycle
//  122       temp[0] ^= Rcon[pp++];
        MOV       A, [SP+0x02]       ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        MOV       A, (_Rcon)[B]      ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, [SP+0x04]       ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        MOV       [SP+0x04], A       ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        INC       A                  ;; 1 cycle
        MOV       [SP+0x02], A       ;; 1 cycle
        ; ------------------------------------- Block: 28 cycles
//  123     }
//  124     
//  125     for(ls8_j=0; ls8_j<4; ls8_j++)
??Cipher_4:
        MOVW      DE, #0x0           ;; 1 cycle
        ; ------------------------------------- Block: 1 cycles
??KeyExpansion_4:
        MOVW      AX, DE             ;; 1 cycle
        XOR       A, #0x80           ;; 1 cycle
        CMPW      AX, #0x8004        ;; 1 cycle
        BNC       ??KeyExpansion_2   ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  126     {
//  127       RoundKey[ls8_i] =(unsigned char)(RoundKey[ls8_i-16] ^ temp[ls8_j]);
        MOVW      AX, [SP]           ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOV       A, (_RoundKey-16)[BC]  ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        MOVW      AX, SP             ;; 1 cycle
        ADDW      AX, #0x4           ;; 1 cycle
        ADDW      AX, DE             ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, B               ;; 1 cycle
        XOR       A, [HL]            ;; 1 cycle
        MOV       H, A               ;; 1 cycle
        MOVW      AX, [SP]           ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        XCH       A, H               ;; 1 cycle
        MOV       (_RoundKey)[BC], A  ;; 1 cycle
        XCH       A, H               ;; 1 cycle
//  128       ls8_i++;
        MOVW      AX, [SP]           ;; 1 cycle
        INCW      AX                 ;; 1 cycle
        MOVW      [SP], AX           ;; 1 cycle
//  129     }
        INCW      DE                 ;; 1 cycle
        BR        S:??KeyExpansion_4  ;; 3 cycles
        ; ------------------------------------- Block: 23 cycles
//  130   }
//  131 }
??Cipher_2:
        ADDW      SP, #0xA           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock0
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 156 cycles
//  132 
//  133 
//  134 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock1 Using cfiCommon0
          CFI Function _AddRoundKey
          CFI NoCalls
        CODE
//  135 void AddRoundKey(unsigned char round)
//  136 {
_AddRoundKey:
        ; * Stack frame (at entry) *
        ; Param size: 0
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+6
        ; Auto size: 4
        SUBW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+8
//  137   unsigned char lu8_i,lu8_j;
//  138   for(lu8_i=0; lu8_i<4; lu8_i++)
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        ; ------------------------------------- Block: 4 cycles
??AddRoundKey_0:
        MOV       A, [SP]            ;; 1 cycle
        CMP       A, #0x4            ;; 1 cycle
        BNC       ??Cipher_5         ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  139   {
//  140     for(lu8_j=0; lu8_j<4; lu8_j++)
        MOV       A, #0x0            ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
??AddRoundKey_1:
        MOV       A, E               ;; 1 cycle
        CMP       A, #0x4            ;; 1 cycle
        BNC       ??Cipher_6         ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  141     {
//  142       state[lu8_j][lu8_i] ^= RoundKey[((round * 4 )+ lu8_i)* Nb + lu8_j];
        MOV       A, [SP+0x03]       ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x4           ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        MOV       C, A               ;; 1 cycle
        MOV       B, #0x0            ;; 1 cycle
        MOVW      AX, HL             ;; 1 cycle
        ADDW      AX, BC             ;; 1 cycle
        MOVW      BC, #0x4           ;; 1 cycle
        MULHU                        ;; 2 cycles
        PUSH      DE                 ;; 1 cycle
          CFI CFA SP+10
        POP       HL                 ;; 1 cycle
          CFI CFA SP+8
        MOV       H, #0x0            ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        MOV       A, (_RoundKey)[BC]  ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        MOV       L, A               ;; 1 cycle
        MOV       H, #0x0            ;; 1 cycle
        MOVW      AX, DE             ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x4           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(_state)  ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        XOR       A, [HL]            ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
        XCH       A, D               ;; 1 cycle
//  143     }
        INC       E                  ;; 1 cycle
        BR        S:??AddRoundKey_1  ;; 3 cycles
        ; ------------------------------------- Block: 41 cycles
//  144   }
??Cipher_6:
        MOV       A, [SP]            ;; 1 cycle
        INC       A                  ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        BR        S:??AddRoundKey_0  ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
//  145 }
??Cipher_5:
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock1
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 72 cycles
//  146 
//  147 
//  148 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock2 Using cfiCommon0
          CFI Function _S_r_Box
          CFI NoCalls
        CODE
//  149 void S_r_Box(unsigned char sat)
//  150 {
_S_r_Box:
        ; * Stack frame (at entry) *
        ; Param size: 0
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+6
        ; Auto size: 4
        SUBW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+8
//  151   unsigned char lu8_i,lu8_j;
//  152   for(lu8_i=0; lu8_i<4; lu8_i++)
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        ; ------------------------------------- Block: 4 cycles
??S_r_Box_0:
        MOV       A, [SP]            ;; 1 cycle
        CMP       A, #0x4            ;; 1 cycle
        SKC                          ;; 4 cycles
        BR        N:??Cipher_7       ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  153   {
//  154     for(lu8_j=0; lu8_j<4; lu8_j++)
        MOV       A, #0x0            ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
??S_r_Box_1:
        MOV       A, D               ;; 1 cycle
        CMP       A, #0x4            ;; 1 cycle
        SKC                          ;; 4 cycles
        BR        N:??Cipher_8       ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  155     {
//  156       if(sat == 0)
        MOV       A, [SP+0x03]       ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        BNZ       ??Cipher_9         ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  157       {
//  158         state[lu8_i][lu8_j] = SBox[state[lu8_i][lu8_j]];
        XCH       A, D               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x4           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(_state)  ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       A, [HL+B]          ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        MOV       A, (_SBox)[B]      ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x4           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(_state)  ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        XCH       A, E               ;; 1 cycle
        MOV       [HL+B], A          ;; 1 cycle
        XCH       A, E               ;; 1 cycle
        BR        S:??Cipher_10      ;; 3 cycles
        ; ------------------------------------- Block: 44 cycles
//  159       }
//  160       else
//  161       {
//  162         state[lu8_i][lu8_j] = r_SBox[state[lu8_i][lu8_j]];
??Cipher_9:
        XCH       A, D               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x4           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(_state)  ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       A, [HL+B]          ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        MOV       A, (_r_SBox)[B]    ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x4           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(_state)  ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        XCH       A, E               ;; 1 cycle
        MOV       [HL+B], A          ;; 1 cycle
        XCH       A, E               ;; 1 cycle
        ; ------------------------------------- Block: 41 cycles
//  163       }
//  164     }
??Cipher_10:
        INC       D                  ;; 1 cycle
        BR        N:??S_r_Box_1      ;; 3 cycles
        ; ------------------------------------- Block: 4 cycles
//  165   }
??Cipher_8:
        MOV       A, [SP]            ;; 1 cycle
        INC       A                  ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        BR        N:??S_r_Box_0      ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
//  166   
//  167 }
??Cipher_7:
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock2
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 126 cycles
//  168 
//  169 
//  170 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock3 Using cfiCommon0
          CFI Function _shiftrows_left
          CFI NoCalls
        CODE
//  171 void shiftrows_left(void)
//  172 {
_shiftrows_left:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  173   unsigned char temp;
//  174   temp = state[1][0];
        MOV       A, N:_state+4      ;; 1 cycle
        MOV       D, A               ;; 1 cycle
//  175   state[1][0]=state[1][1];
        MOV       A, N:_state+5      ;; 1 cycle
        MOV       N:_state+4, A      ;; 1 cycle
//  176   state[1][1]=state[1][2];
        MOV       A, N:_state+6      ;; 1 cycle
        MOV       N:_state+5, A      ;; 1 cycle
//  177   state[1][2]=state[1][3];
        MOV       A, N:_state+7      ;; 1 cycle
        MOV       N:_state+6, A      ;; 1 cycle
//  178   state[1][3]=temp;
        XCH       A, D               ;; 1 cycle
        MOV       N:_state+7, A      ;; 1 cycle
        XCH       A, D               ;; 1 cycle
//  179   
//  180   temp=state[2][0];
        MOV       C, N:_state+8      ;; 1 cycle
//  181   state[2][0]=state[2][2];
        MOV       A, N:_state+10     ;; 1 cycle
        MOV       N:_state+8, A      ;; 1 cycle
//  182   state[2][2]= temp;
        XCH       A, C               ;; 1 cycle
        MOV       N:_state+10, A     ;; 1 cycle
        XCH       A, C               ;; 1 cycle
//  183   temp = state[2][1];
        MOV       B, N:_state+9      ;; 1 cycle
//  184   state[2][1] = state[2][3];
        MOV       A, N:_state+11     ;; 1 cycle
        MOV       N:_state+9, A      ;; 1 cycle
//  185   state[2][3] = temp;
        XCH       A, B               ;; 1 cycle
        MOV       N:_state+11, A     ;; 1 cycle
        XCH       A, B               ;; 1 cycle
//  186   
//  187   temp=state[3][0];
        MOV       X, N:_state+12     ;; 1 cycle
//  188   state[3][0]=state[3][3];
        MOV       A, N:_state+15     ;; 1 cycle
        MOV       N:_state+12, A     ;; 1 cycle
//  189   state[3][3]=state[3][2];
        MOV       A, N:_state+14     ;; 1 cycle
        MOV       N:_state+15, A     ;; 1 cycle
//  190   state[3][2]=state[3][1];
        MOV       A, N:_state+13     ;; 1 cycle
        MOV       N:_state+14, A     ;; 1 cycle
//  191   state[3][1]= temp;
        XCH       A, X               ;; 1 cycle
        MOV       N:_state+13, A     ;; 1 cycle
        XCH       A, X               ;; 1 cycle
//  192   
//  193 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock3
        ; ------------------------------------- Block: 39 cycles
        ; ------------------------------------- Total: 39 cycles
//  194 
//  195 
//  196 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock4 Using cfiCommon0
          CFI Function _MixColumns
          CFI NoCalls
        CODE
//  197 void MixColumns(void)
//  198 {
_MixColumns:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 2
        SUBW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+6
//  199   int ls16_i;
//  200   unsigned char Tmp,Tm,t;
//  201   for(ls16_i=0; ls16_i<4; ls16_i++)
        MOVW      BC, #0x0           ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
??MixColumns_0:
        MOVW      AX, BC             ;; 1 cycle
        XOR       A, #0x80           ;; 1 cycle
        CMPW      AX, #0x8004        ;; 1 cycle
        SKC                          ;; 4 cycles
        BR        N:??Cipher_11      ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  202   {
//  203     t = state[0][ls16_i];
        MOV       A, (_state)[BC]    ;; 1 cycle
        MOV       [SP+0x01], A       ;; 1 cycle
//  204     Tmp =(unsigned char) (state[0][ls16_i] ^ state[1][ls16_i] ^ state[2][ls16_i] ^ state[3][ls16_i]);
        MOV       A, (_state+4)[BC]  ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, (_state)[BC]    ;; 1 cycle
        XOR       X, A               ;; 1 cycle
        MOV       A, (_state+8)[BC]  ;; 1 cycle
        XOR       X, A               ;; 1 cycle
        MOV       A, (_state+12)[BC]  ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
//  205     Tm =(unsigned char) (state[0][ls16_i] ^ state[1][ls16_i]);
        MOV       A, (_state+4)[BC]  ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, (_state)[BC]    ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  206     Tm = (unsigned char)xtime(Tm);
        MOV       A, [SP]            ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  207     state[0][ls16_i] ^= Tm ^ Tmp;
        MOV       A, [SP]            ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOVW      AX, BC             ;; 1 cycle
        ADDW      AX, #LWRD(_state)  ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        XCH       A, E               ;; 1 cycle
        XOR       A, [HL]            ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
        XCH       A, E               ;; 1 cycle
//  208     Tm =(unsigned char)( state[1][ls16_i] ^ state[2][ls16_i]);
        MOV       A, (_state+8)[BC]  ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, (_state+4)[BC]  ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  209     Tm = (unsigned char)xtime(Tm);
        MOV       A, [SP]            ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  210     state[1][ls16_i] ^= Tm ^ Tmp;
        MOV       A, [SP]            ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOVW      AX, BC             ;; 1 cycle
        ADDW      AX, #LWRD(_state+4)  ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        XCH       A, E               ;; 1 cycle
        XOR       A, [HL]            ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
        XCH       A, E               ;; 1 cycle
//  211     Tm = (unsigned char)(state[2][ls16_i] ^ state[3][ls16_i]);
        MOV       A, (_state+12)[BC]  ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, (_state+8)[BC]  ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  212     Tm = (unsigned char)xtime(Tm);
        MOV       A, [SP]            ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  213     state[2][ls16_i] ^= Tm ^ Tmp;
        MOV       A, [SP]            ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOVW      AX, BC             ;; 1 cycle
        ADDW      AX, #LWRD(_state+8)  ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        XCH       A, E               ;; 1 cycle
        XOR       A, [HL]            ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
        XCH       A, E               ;; 1 cycle
//  214     Tm = (unsigned char)(state[3][ls16_i] ^ t);
        MOV       A, (_state+12)[BC]  ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  215     Tm = (unsigned char)xtime(Tm);
        MOV       A, [SP]            ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  216     state[3][ls16_i] ^= Tm ^ Tmp;
        MOV       A, [SP]            ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOVW      AX, BC             ;; 1 cycle
        ADDW      AX, #LWRD(_state+12)  ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        XCH       A, E               ;; 1 cycle
        XOR       A, [HL]            ;; 1 cycle
        MOV       [HL], A            ;; 1 cycle
        XCH       A, E               ;; 1 cycle
//  217   }
        INCW      BC                 ;; 1 cycle
        BR        N:??MixColumns_0   ;; 3 cycles
        ; ------------------------------------- Block: 107 cycles
//  218 }
??Cipher_11:
        ADDW      SP, #0x2           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock4
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 123 cycles
//  219 
//  220 
//  221 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock5 Using cfiCommon0
          CFI Function _InvShiftRows
          CFI NoCalls
        CODE
//  222 void InvShiftRows(void)
//  223 {
_InvShiftRows:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 0
//  224   unsigned char temp;
//  225   
//  226   /* Rotate first row 1 columns to right */
//  227   temp=state[1][3];
        MOV       A, N:_state+7      ;; 1 cycle
        MOV       D, A               ;; 1 cycle
//  228   state[1][3]=state[1][2];
        MOV       A, N:_state+6      ;; 1 cycle
        MOV       N:_state+7, A      ;; 1 cycle
//  229   state[1][2]=state[1][1];
        MOV       A, N:_state+5      ;; 1 cycle
        MOV       N:_state+6, A      ;; 1 cycle
//  230   state[1][1]=state[1][0];
        MOV       A, N:_state+4      ;; 1 cycle
        MOV       N:_state+5, A      ;; 1 cycle
//  231   state[1][0]=temp;
        XCH       A, D               ;; 1 cycle
        MOV       N:_state+4, A      ;; 1 cycle
        XCH       A, D               ;; 1 cycle
//  232   
//  233   /* Rotate second row 2 columns to right */
//  234   temp=state[2][0];
        MOV       C, N:_state+8      ;; 1 cycle
//  235   state[2][0]=state[2][2];
        MOV       A, N:_state+10     ;; 1 cycle
        MOV       N:_state+8, A      ;; 1 cycle
//  236   state[2][2]=temp;
        XCH       A, C               ;; 1 cycle
        MOV       N:_state+10, A     ;; 1 cycle
        XCH       A, C               ;; 1 cycle
//  237   
//  238   temp=state[2][1];
        MOV       B, N:_state+9      ;; 1 cycle
//  239   state[2][1]=state[2][3];
        MOV       A, N:_state+11     ;; 1 cycle
        MOV       N:_state+9, A      ;; 1 cycle
//  240   state[2][3]=temp;
        XCH       A, B               ;; 1 cycle
        MOV       N:_state+11, A     ;; 1 cycle
        XCH       A, B               ;; 1 cycle
//  241   
//  242   /* Rotate third row 3 columns to right */
//  243   temp=state[3][0];
        MOV       X, N:_state+12     ;; 1 cycle
//  244   state[3][0]=state[3][1];
        MOV       A, N:_state+13     ;; 1 cycle
        MOV       N:_state+12, A     ;; 1 cycle
//  245   state[3][1]=state[3][2];
        MOV       A, N:_state+14     ;; 1 cycle
        MOV       N:_state+13, A     ;; 1 cycle
//  246   state[3][2]=state[3][3];
        MOV       A, N:_state+15     ;; 1 cycle
        MOV       N:_state+14, A     ;; 1 cycle
//  247   state[3][3]=temp;
        XCH       A, X               ;; 1 cycle
        MOV       N:_state+15, A     ;; 1 cycle
        XCH       A, X               ;; 1 cycle
//  248 }
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock5
        ; ------------------------------------- Block: 39 cycles
        ; ------------------------------------- Total: 39 cycles
//  249 
//  250 
//  251 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock6 Using cfiCommon0
          CFI Function _InvMixColumns
          CFI NoCalls
        CODE
//  252 void InvMixColumns(void)
//  253 {
_InvMixColumns:
        ; * Stack frame (at entry) *
        ; Param size: 0
        ; Auto size: 4
        SUBW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+8
//  254   int ls16_i;
//  255   unsigned char a,b,c,d;
//  256   for(ls16_i=0; ls16_i<4; ls16_i++)
        MOVW      BC, #0x0           ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
??InvMixColumns_0:
        MOVW      AX, BC             ;; 1 cycle
        XOR       A, #0x80           ;; 1 cycle
        CMPW      AX, #0x8004        ;; 1 cycle
        SKC                          ;; 4 cycles
        BR        N:??Cipher_12      ;; 4 cycles
        ; ------------------------------------- Block: 7 cycles
//  257   {
//  258     a = state[0][ls16_i];
        MOV       A, (_state)[BC]    ;; 1 cycle
        MOV       [SP+0x03], A       ;; 1 cycle
//  259     b = state[1][ls16_i];
        MOV       A, (_state+4)[BC]  ;; 1 cycle
        MOV       [SP+0x02], A       ;; 1 cycle
//  260     c = state[2][ls16_i];
        MOV       A, (_state+8)[BC]  ;; 1 cycle
        MOV       [SP+0x01], A       ;; 1 cycle
//  261     d = state[3][ls16_i];
        MOV       A, (_state+12)[BC]  ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
//  262     
//  263     state[0][ls16_i] =(unsigned char)( Multiply(a, 0x0e) ^ Multiply(b, 0x0b) ^ Multiply(c, 0x0d) ^ Multiply(d, 0x09));
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       H, A               ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, H               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        XOR       E, A               ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       E, A               ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        MOV       H, A               ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       H, A               ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        XOR       A, H               ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        XOR       X, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        XOR       E, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       E, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       H, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, H               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       H, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, H               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       H, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       L, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, L               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, H               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        XOR       D, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       D, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       E, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       H, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, H               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       H, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, H               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       H, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       L, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, L               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, H               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        XOR       E, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       E, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       H, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, H               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       H, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, H               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       H, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       L, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, L               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, H               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        MOV       (_state)[BC], A    ;; 1 cycle
//  264     state[1][ls16_i] =(unsigned char)( Multiply(a, 0x09) ^ Multiply(b, 0x0e) ^ Multiply(c, 0x0b) ^ Multiply(d, 0x0d));
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       H, A               ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, H               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        MOV       H, A               ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       H, A               ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        XOR       A, H               ;; 1 cycle
        XOR       D, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        XOR       D, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       D, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       E, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       H, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, H               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       H, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, H               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       H, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       L, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, L               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, H               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        XOR       X, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        XOR       E, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       E, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       H, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, H               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       H, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, H               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       H, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       L, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, L               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, H               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        XOR       D, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       D, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       E, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       H, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, H               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       H, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, H               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       H, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       L, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, L               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, H               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        MOV       (_state+4)[BC], A  ;; 1 cycle
//  265     state[2][ls16_i] =(unsigned char)( Multiply(a, 0x0d) ^ Multiply(b, 0x09) ^ Multiply(c, 0x0e) ^ Multiply(d, 0x0b));
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       H, A               ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, H               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       E, A               ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        MOV       H, A               ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       H, A               ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        XOR       A, H               ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        XOR       E, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       E, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       H, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, H               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       H, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, H               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       H, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       L, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, L               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, H               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        XOR       D, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        XOR       D, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       D, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       E, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       H, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, H               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       H, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, H               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       H, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       L, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, L               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, H               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        XOR       X, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        XOR       E, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       E, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       H, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, H               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       H, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, H               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       H, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       L, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, L               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, H               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        MOV       (_state+8)[BC], A  ;; 1 cycle
//  266     state[3][ls16_i] =(unsigned char)( Multiply(a, 0x0b) ^ Multiply(b, 0x0d) ^ Multiply(c, 0x09) ^ Multiply(d, 0x0e));
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       H, A               ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, H               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        MOV       H, A               ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        XOR       A, H               ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        MOV       H, A               ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       H, A               ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x03]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        XOR       A, H               ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        XOR       D, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       D, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       E, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       H, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, H               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       H, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, H               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       H, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       L, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, L               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, H               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        XOR       E, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       E, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       H, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, H               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       H, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, H               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       H, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       L, A               ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP+0x01]       ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, L               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, H               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        XOR       D, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        XOR       D, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       D, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       E, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       H, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, H               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       H, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, H               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       H, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        MOV       L, A               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHR       A, 0x7             ;; 1 cycle
        MOV       X, #0x1B           ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, X               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, L               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, H               ;; 1 cycle
        SHL       A, 0x1             ;; 1 cycle
        XOR       A, E               ;; 1 cycle
        MOV       X, #0x0            ;; 1 cycle
        MULU      X                  ;; 1 cycle
        MOV       A, X               ;; 1 cycle
        XOR       A, D               ;; 1 cycle
        MOV       (_state+12)[BC], A  ;; 1 cycle
//  267   }
        INCW      BC                 ;; 1 cycle
        BR        N:??InvMixColumns_0  ;; 3 cycles
        ; ------------------------------------- Block: 3171 cycles
//  268 }
??Cipher_12:
        ADDW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock6
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 3187 cycles
//  269 
//  270 
//  271 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock7 Using cfiCommon0
          CFI Function _InvCipher
        CODE
//  272 void InvCipher(unsigned char *input)
//  273 {
_InvCipher:
        ; * Stack frame (at entry) *
        ; Param size: 0
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+6
        ; Auto size: 6
        SUBW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+10
//  274   unsigned char lu8_i,lu8_j,round=0;
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP+0x01], A       ;; 1 cycle
//  275   for(lu8_i=0; lu8_i<4; lu8_i++)
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP+0x02], A       ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
??InvCipher_0:
        MOV       A, [SP+0x02]       ;; 1 cycle
        CMP       A, #0x4            ;; 1 cycle
        BNC       ??Cipher_13        ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  276   {
//  277     for(lu8_j=0; lu8_j<4; lu8_j++)
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
??InvCipher_1:
        MOV       A, [SP]            ;; 1 cycle
        CMP       A, #0x4            ;; 1 cycle
        BNC       ??Cipher_14        ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  278     {
//  279       state[lu8_j][lu8_i] = input[lu8_i*4 + lu8_j];
        MOV       A, [SP+0x02]       ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x4           ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       D, #0x0            ;; 1 cycle
        MOVW      AX, HL             ;; 1 cycle
        ADDW      AX, DE             ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, [HL]            ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x4           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(_state)  ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        XCH       A, E               ;; 1 cycle
        MOV       [HL+B], A          ;; 1 cycle
        XCH       A, E               ;; 1 cycle
//  280     }
        MOV       A, [SP]            ;; 1 cycle
        INC       A                  ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        BR        S:??InvCipher_1    ;; 3 cycles
        ; ------------------------------------- Block: 43 cycles
//  281   }
??Cipher_14:
        MOV       A, [SP+0x02]       ;; 1 cycle
        INC       A                  ;; 1 cycle
        MOV       [SP+0x02], A       ;; 1 cycle
        BR        S:??InvCipher_0    ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
//  282   
//  283   AddRoundKey(Nr);
??Cipher_13:
        MOV       A, #0xA            ;; 1 cycle
          CFI FunCall _AddRoundKey
        CALL      _AddRoundKey       ;; 3 cycles
//  284   for(round=Nr-1; round>0; round--)
        MOV       A, #0x9            ;; 1 cycle
        MOV       [SP+0x01], A       ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
??InvCipher_2:
        MOV       A, [SP+0x01]       ;; 1 cycle
        CMP0      A                  ;; 1 cycle
        BZ        ??Cipher_15        ;; 4 cycles
          CFI FunCall _InvShiftRows
        ; ------------------------------------- Block: 6 cycles
//  285   {
//  286     InvShiftRows();
        CALL      _InvShiftRows      ;; 3 cycles
//  287     S_r_Box(1);
        MOV       A, #0x1            ;; 1 cycle
          CFI FunCall _S_r_Box
        CALL      _S_r_Box           ;; 3 cycles
//  288     AddRoundKey(round);
        MOV       A, [SP+0x01]       ;; 1 cycle
          CFI FunCall _AddRoundKey
        CALL      _AddRoundKey       ;; 3 cycles
//  289     InvMixColumns();
          CFI FunCall _InvMixColumns
        CALL      _InvMixColumns     ;; 3 cycles
//  290   }
        MOV       A, [SP+0x01]       ;; 1 cycle
        DEC       A                  ;; 1 cycle
        MOV       [SP+0x01], A       ;; 1 cycle
        BR        S:??InvCipher_2    ;; 3 cycles
          CFI FunCall _InvShiftRows
        ; ------------------------------------- Block: 20 cycles
//  291   
//  292   InvShiftRows();
??Cipher_15:
        CALL      _InvShiftRows      ;; 3 cycles
//  293   S_r_Box(1);
        MOV       A, #0x1            ;; 1 cycle
          CFI FunCall _S_r_Box
        CALL      _S_r_Box           ;; 3 cycles
//  294   AddRoundKey(0);
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _AddRoundKey
        CALL      _AddRoundKey       ;; 3 cycles
//  295   
//  296   for(lu8_i=0; lu8_i<4; lu8_i++)
        MOV       E, #0x0            ;; 1 cycle
        ; ------------------------------------- Block: 12 cycles
??InvCipher_3:
        MOV       A, E               ;; 1 cycle
        CMP       A, #0x4            ;; 1 cycle
        BNC       ??Cipher_16        ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  297   {
//  298     for(lu8_j=0; lu8_j<4; lu8_j++)
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
??InvCipher_4:
        MOV       A, [SP]            ;; 1 cycle
        CMP       A, #0x4            ;; 1 cycle
        BNC       ??Cipher_17        ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  299     {
//  300       out[lu8_i*4+lu8_j]=state[lu8_j][lu8_i];
        XCH       A, E               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, E               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x4           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(_state)  ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       A, [HL+B]          ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOVW      AX, DE             ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x4           ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      BC, AX             ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        MOV       L, A               ;; 1 cycle
        MOV       H, #0x0            ;; 1 cycle
        MOVW      AX, BC             ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       (_out)[BC], A      ;; 1 cycle
        XCH       A, D               ;; 1 cycle
//  301     }
        MOV       A, [SP]            ;; 1 cycle
        INC       A                  ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        BR        S:??InvCipher_4    ;; 3 cycles
        ; ------------------------------------- Block: 40 cycles
//  302   }
??Cipher_17:
        INC       E                  ;; 1 cycle
        BR        S:??InvCipher_3    ;; 3 cycles
        ; ------------------------------------- Block: 4 cycles
//  303 }
??Cipher_16:
        ADDW      SP, #0x6           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock7
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 178 cycles
//  304 
//  305 
//  306 

        SECTION `.text`:CODE:NOROOT(0)
          CFI Block cfiBlock8 Using cfiCommon0
          CFI Function _Cipher
        CODE
//  307 void Cipher(unsigned char *input)
//  308 {
_Cipher:
        ; * Stack frame (at entry) *
        ; Param size: 0
        PUSH      AX                 ;; 1 cycle
          CFI CFA SP+6
        ; Auto size: 6
        SUBW      SP, #0x4           ;; 1 cycle
          CFI CFA SP+10
//  309   unsigned char lu8_i,lu8_j,round=0;
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP+0x01], A       ;; 1 cycle
//  310   for(lu8_i=0; lu8_i<4; lu8_i++)
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP+0x02], A       ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
??Cipher_18:
        MOV       A, [SP+0x02]       ;; 1 cycle
        CMP       A, #0x4            ;; 1 cycle
        BNC       ??Cipher_19        ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  311   {
//  312     for(lu8_j=0; lu8_j<4; lu8_j++)
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
??Cipher_20:
        MOV       A, [SP]            ;; 1 cycle
        CMP       A, #0x4            ;; 1 cycle
        BNC       ??Cipher_21        ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  313     {
//  314       state[lu8_j][lu8_i] = input[lu8_i*4 + lu8_j];
        MOV       A, [SP+0x02]       ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x4           ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       D, #0x0            ;; 1 cycle
        MOVW      AX, HL             ;; 1 cycle
        ADDW      AX, DE             ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOVW      AX, [SP+0x04]      ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        MOV       A, [HL]            ;; 1 cycle
        MOV       E, A               ;; 1 cycle
        MOV       A, [SP+0x02]       ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x4           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(_state)  ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        XCH       A, E               ;; 1 cycle
        MOV       [HL+B], A          ;; 1 cycle
        XCH       A, E               ;; 1 cycle
//  315     }
        MOV       A, [SP]            ;; 1 cycle
        INC       A                  ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        BR        S:??Cipher_20      ;; 3 cycles
        ; ------------------------------------- Block: 43 cycles
//  316   }
??Cipher_21:
        MOV       A, [SP+0x02]       ;; 1 cycle
        INC       A                  ;; 1 cycle
        MOV       [SP+0x02], A       ;; 1 cycle
        BR        S:??Cipher_18      ;; 3 cycles
        ; ------------------------------------- Block: 6 cycles
//  317   AddRoundKey(0);
??Cipher_19:
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _AddRoundKey
        CALL      _AddRoundKey       ;; 3 cycles
//  318   for(round=1; round<Nr; round++)
        MOV       A, #0x1            ;; 1 cycle
        MOV       [SP+0x01], A       ;; 1 cycle
        ; ------------------------------------- Block: 6 cycles
??Cipher_22:
        MOV       A, [SP+0x01]       ;; 1 cycle
        CMP       A, #0xA            ;; 1 cycle
        BNC       ??Cipher_23        ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  319   {
//  320     S_r_Box(0);
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _S_r_Box
        CALL      _S_r_Box           ;; 3 cycles
//  321     shiftrows_left();
          CFI FunCall _shiftrows_left
        CALL      _shiftrows_left    ;; 3 cycles
//  322     MixColumns();
          CFI FunCall _MixColumns
        CALL      _MixColumns        ;; 3 cycles
//  323     AddRoundKey(round);
        MOV       A, [SP+0x01]       ;; 1 cycle
          CFI FunCall _AddRoundKey
        CALL      _AddRoundKey       ;; 3 cycles
//  324   }
        MOV       A, [SP+0x01]       ;; 1 cycle
        INC       A                  ;; 1 cycle
        MOV       [SP+0x01], A       ;; 1 cycle
        BR        S:??Cipher_22      ;; 3 cycles
        ; ------------------------------------- Block: 20 cycles
//  325   
//  326   S_r_Box(0);
??Cipher_23:
        MOV       A, #0x0            ;; 1 cycle
          CFI FunCall _S_r_Box
        CALL      _S_r_Box           ;; 3 cycles
//  327   shiftrows_left();
          CFI FunCall _shiftrows_left
        CALL      _shiftrows_left    ;; 3 cycles
//  328   AddRoundKey(Nr);
        MOV       A, #0xA            ;; 1 cycle
          CFI FunCall _AddRoundKey
        CALL      _AddRoundKey       ;; 3 cycles
//  329   for(lu8_i=0; lu8_i<4; lu8_i++)
        MOV       E, #0x0            ;; 1 cycle
        ; ------------------------------------- Block: 12 cycles
??Cipher_24:
        MOV       A, E               ;; 1 cycle
        CMP       A, #0x4            ;; 1 cycle
        BNC       ??Cipher_25        ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  330   {
//  331     for(lu8_j=0; lu8_j<4; lu8_j++)
        MOV       A, #0x0            ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        ; ------------------------------------- Block: 2 cycles
??Cipher_26:
        MOV       A, [SP]            ;; 1 cycle
        CMP       A, #0x4            ;; 1 cycle
        BNC       ??Cipher_27        ;; 4 cycles
        ; ------------------------------------- Block: 6 cycles
//  332     {
//  333       out[lu8_i*4+lu8_j] = state[lu8_j][lu8_i];
        XCH       A, E               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, E               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        XCH       A, B               ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        MOV       X, A               ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x4           ;; 1 cycle
        MULHU                        ;; 2 cycles
        ADDW      AX, #LWRD(_state)  ;; 1 cycle
        MOVW      HL, AX             ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       B, A               ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       A, [HL+B]          ;; 1 cycle
        MOV       D, A               ;; 1 cycle
        MOVW      AX, DE             ;; 1 cycle
        MOV       A, #0x0            ;; 1 cycle
        MOVW      BC, #0x4           ;; 1 cycle
        MULHU                        ;; 2 cycles
        MOVW      BC, AX             ;; 1 cycle
        MOV       A, [SP]            ;; 1 cycle
        MOV       L, A               ;; 1 cycle
        MOV       H, #0x0            ;; 1 cycle
        MOVW      AX, BC             ;; 1 cycle
        ADDW      AX, HL             ;; 1 cycle
        MOVW      BC, AX             ;; 1 cycle
        XCH       A, D               ;; 1 cycle
        MOV       (_out)[BC], A      ;; 1 cycle
        XCH       A, D               ;; 1 cycle
//  334     }
        MOV       A, [SP]            ;; 1 cycle
        INC       A                  ;; 1 cycle
        MOV       [SP], A            ;; 1 cycle
        BR        S:??Cipher_26      ;; 3 cycles
        ; ------------------------------------- Block: 40 cycles
//  335   }
??Cipher_27:
        INC       E                  ;; 1 cycle
        BR        S:??Cipher_24      ;; 3 cycles
        ; ------------------------------------- Block: 4 cycles
//  336 }
??Cipher_25:
        ADDW      SP, #0x6           ;; 1 cycle
          CFI CFA SP+4
        RET                          ;; 6 cycles
          CFI EndBlock cfiBlock8
        ; ------------------------------------- Block: 7 cycles
        ; ------------------------------------- Total: 178 cycles

        SECTION `.iar_vfe_header`:DATA:NOALLOC:NOROOT(1)
        SECTION_TYPE SHT_PROGBITS, 0
        DATA
        DC32 0

        END
//  337 
//  338 
//  339 
// 
//   240 bytes in section .bss
//   768 bytes in section .data
// 6'875 bytes in section .text
// 
// 6'875 bytes of CODE memory
// 1'008 bytes of DATA memory
//
//Errors: none
//Warnings: none
