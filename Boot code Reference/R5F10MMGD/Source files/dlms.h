#ifndef DLMS_H
#define DLMS_H
/*******************************************************************************
 * File Name   : dlms.h
 * Module      : dlms
 * Description : Support file for main.c
 * Author      : Shishir Chowdhry & Harshita Jain
 * Company     : Genus Power Infrastructures Ltd., Jaipur
 ******************************************************************************/
/*******************************************************************************
 * File includes
 ******************************************************************************/


/*******************************************************************************
 * Constant Definitions
 ******************************************************************************/

#define sd trn_buf
#define rec rcv_buf

static const unsigned int fcstab[256] = {
0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78};




static const unsigned char GEN_CAL[]={ 'G','E','N','U','S','C','A','L','_','1'};
static const unsigned char default_tod[]={0x09,0x00,0x09,0x15,0x09,0x30,0x09,0x45,0x10,0x00,0x10,0x15,0x10,0x30,0x10,0x45};
static const unsigned char default_tariff[]={8,1,2,3,4,5,6,7,8};
static unsigned const char auth_fill[15] = {14,0x00,0x02,0x07,0x11,0x02,0x11,0x10,0x12,0x02,0xf4,0x11,0x05,0x11,0x08};//structure of seven
static unsigned char const aut_pswd2[8]= {0x31,0x41,0x32,0x42,0x33,0x44,0x44,0x44};
//static unsigned char const aut_pswd1_23[16]={0x5f,0x5f,0x44,0x4c,0x4d,0x53,0x2d,0x45,0x78,0x70,0x6c,0x6f,0x72,0x65,0x72,0x5f};
static unsigned char const aut_pswd1_23[16]={"GenusPowerInfras"};//0x5f,0x5f,0x44,0x4c,0x4d,0x53,0x2d,0x45,0x78,0x70,0x6c,0x6f,0x72,0x65,0x72,0x5f};
static unsigned char const pswrd1[8]={0x31,0x41,0x32,0x42,0x33,0x43,0x34,0x44};
static unsigned char const pswrd2[16]={'d','l','m','s','p','a','s','s','w','o','r','d','1','2','3','4'};

static const unsigned char XDLMS_TYPE_CONST[]={
	20,
	0x00,
	0x02,//structure
	0x06,//length of structure
	0x04,//conformance(bit string(24))
	0x18,//length 3
	0x00,//conformance[0]
	0x00,//conformance[1]
	0x18,//conformance[3]
	0x12,//max-rec-PDU-size(long-unsigned)
	0x01,//max-rec-PDU-size[0]
	0xFF,//max-rec-PDU-size[1]
	0x12,//max-send-PDU-size(long-unsigned)
	0x01,//max-send-PDU-size[0]
	0xFF,//max-send-PDU-size[1]
	0x11,//DLMS-version-number(unsigned)
	0x06,//DLMS-version-number[0]
	0x0F,//quality of service(integer8)
	0x00,//value
	0x09,//cyphering_info(octet-string)
	0x00 //length*/
};

static const unsigned char sap_assgn_list[]={
	57,
	0x00,
	0x01,
	0x03,//length of array
	0x02,
	0x02,//length of structure
	0x12,//SAP address(long_unsigned)
	0x00,
	0x01,//SAP address[1]
	0x09,
	0x0B,//logical_device_name(octet_string)
	'G',
	'O',
	'E',
	'S',
	'P',
	'H',
	'W',
	'C',
	'N',
	'-',
	'S',
	0x02,
	0x02,
	0x12,
	0x00,
	0x10,//SAP address[2]
	0x09,
	0x0B,
	'G',
	'O',
	'E',
	'S',
	'P',
	'H',
	'W',
	'C',
	'N',
	'1',
	'S',
	0x02,
	0x02,
	0x12,
	0x00,
	0x20,//SAP address[3]
	0x09,
	0x0B,
	'G',
	'O',
	'E',
	'S',
	'P',
	'H',
	'W',
	'C',
	'N',
	'2',
	'S',
};
static const unsigned char sap_assgn_list_GIL[]={
	57,
	0x00,
	0x01,
	0x03,//length of array
	0x02,
	0x02,//length of structure
	0x12,//SAP address(long_unsigned)
	0x00,
	0x01,//SAP address[1]
	0x09,
	0x0B,//logical_device_name(octet_string)
	'G',
	'I',
	'L',
	'S',
	'P',
	'H',
	'W',
	'C',
	'N',
	'-',
	'S',
	0x02,
	0x02,
	0x12,
	0x00,
	0x10,//SAP address[2]
	0x09,
	0x0B,
	'G',
	'I',
	'L',
	'S',
	'P',
	'H',
	'W',
	'C',
	'N',
	'1',
	'S',
	0x02,
	0x02,
	0x12,
	0x00,
	0x20,//SAP address[3]
	0x09,
	0x0B,
	'G',
	'I',
	'L',
	'S',
	'P',
	'H',
	'W',
	'C',
	'N',
	'2',
	'S',
};

static const unsigned char app_con[] =
{ 4,
  0x11,//app_con_ele(unsigned)
  0x01,//app_con_ele[0]
  0x11,//context_id_ele(unsigned)
  0x01
};//context_id_ele[0]

static const unsigned char Start_Info_CONT[]={
  0xE6,//0
  0xE7,//1
  0x00,//2
  0xC4,//GET.RESP
  0x01,//normal
  0x81,//INVOKE_ID,PRIORITY
  0x00,//data_result
  0x01,//array
};

static const unsigned char Start_Info_CONT1[]={
  0xE6,//0
  0xE7,//1
  0x00,//2
  0xC4,//GET.RESP
  0x02,//get response with datablock
  0x81,//invoke-id priority
  0x00,//if not the last block
  0x00,//data block no.
  0x00,//data block no.
  0x00,//data block no.
  0x00,//data block no.
  0x00,		//raw data
  0x82          //length Consider next 2 byte
       //Data Block length 1
};

static const unsigned char GEN_SEA_NAME_1[]={
  'G',//season_profile_name[0]
  'N',//season_profile_name[2]
  'S',//season_profile_name[4]
  'S',//season_profile_name[5]
  'E',//season_profile_name[6]
  'A',//season_profile_name[7]
  '1',//season_profile_name[9]   //struct_element_1   //struct_element_2

  'G',//season_profile_name[0]
  'N',//season_profile_name[2]
  'S',//season_profile_name[4]
  'S',//season_profile_name[5]
  'E',//season_profile_name[6]
  'A',//season_profile_name[7]
  '2',//season_profile_name[9]   //struct_element_1   //struct_element_2
   };



static const unsigned char GEN_SEA_START[]={
  0,0,0x12,1,1,
  0,0,0x12,6,1

   };

static const unsigned char GEN_WEEK_NAME[]={
  'G',//season_profile_name[0]
  'N',//season_profile_name[2]
  'S',//season_profile_name[4]
  'W',//season_profile_name[5]
  'E',//season_profile_name[6]
  'E',//season_profile_name[7]
  'K',//season_profile_name[8]
};

//static const unsigned char GEN_WEEK1[]={
//  28,
////  0x00,
////  0x01,//array
////  0x01,//length of array(1 element only)
//  0x02,//structure
//  0x08,//length of structure
//
//  //struct_element_1
//  0x09,//week_name(octet_string)
//  0x0A,//length of string(10 bytes)
//  'G',//season_profile_name[0]
//  'E',//season_profile_name[1]
//  'N',//season_profile_name[2]
//  'U',//season_profile_name[3]
//  'S',//season_profile_name[4]
//  'W',//season_profile_name[5]
//  'E',//season_profile_name[6]
//  'E',//season_profile_name[7]
//  'K',//season_profile_name[8]
//  '1',//season_profile_name[9]
//  //struct_element_1
//
//  //struct_element_2
//  0x11,//monday_id(unsigned)
//  0x01,//day_id
//  //struct_element_2
//
//  //struct_element_3
//  0x11,//tuesday_id(unsigned)
//  0x01,//day_id
//  //struct_element_3
//
//  //struct_element_4
//  0x11,//wednesday_id(unsigned)
//  0x01,//day_id
//  //struct_element_4
//
//  //struct_element_5
//  0x11,//thrusday_id(unsigned)
//  0x01,//day_id
//  //struct_element_5
//
//  //struct_element_6
//  0x11,//friday_id(unsigned)
//  0x01,//day_id
//  //struct_element_6
//
//  //struct_element_7
//  0x11,//saturday_id(unsigned)
//  0x01,//day_id
//  //struct_element_7
//
//  //struct_element_8
//  0x11,//saturday_id(unsigned)
//  0x01//day_id
//  //struct_element_8
//};
//
//
//static const unsigned char GEN_WEEK2[]={
//  28,
////  0x00,
////  0x01,//array
////  0x01,//length of array(1 element only)
//  0x02,//structure
//  0x08,//length of structure
//
//  //struct_element_1
//  0x09,//week_name(octet_string)
//  0x0A,//length of string(10 bytes)
//  'G',//season_profile_name[0]
//  'E',//season_profile_name[1]
//  'N',//season_profile_name[2]
//  'U',//season_profile_name[3]
//  'S',//season_profile_name[4]
//  'W',//season_profile_name[5]
//  'E',//season_profile_name[6]
//  'E',//season_profile_name[7]
//  'K',//season_profile_name[8]
//  '2',//season_profile_name[9]
//  //struct_element_1
//
//  //struct_element_2
//  0x11,//monday_id(unsigned)
//  0x02,//day_id
//  //struct_element_2
//
//  //struct_element_3
//  0x11,//tuesday_id(unsigned)
//  0x02,//day_id
//  //struct_element_3
//
//  //struct_element_4
//  0x11,//wednesday_id(unsigned)
//  0x02,//day_id
//  //struct_element_4
//
//  //struct_element_5
//  0x11,//thrusday_id(unsigned)
//  0x02,//day_id
//  //struct_element_5
//
//  //struct_element_6
//  0x11,//friday_id(unsigned)
//  0x02,//day_id
//  //struct_element_6
//
//  //struct_element_7
//  0x11,//saturday_id(unsigned)
//  0x02,//day_id
//  //struct_element_7
//
//  //struct_element_8
//  0x11,//saturday_id(unsigned)
//  0x02//day_id
//  //struct_element_8
//};

static const unsigned char LOGICAL_DEVICE_NAME[]={
	11,
	'G',
	'O',
	'E',
	'S',
	'P',
	'H',
	'W',
	'C',
	'N',
	'-',
	'S',
};
static const unsigned char LOGICAL_DEVICE_NAME_GIL[]={
	11,
	'G',
	'I',
	'L',
	'S',
	'P',
	'H',
	'W',
	'C',
	'N',
	'-',
	'S',
};
static const unsigned char SINGLE_ACTION_BILL_ATT2[]={
  14,
  0,
  0x02,
  0x02,
  0x09,
  0x06,
  1,0,98,1,0,255,
  0x12,
  0x00,
  0x02,
};
static const unsigned char SINGLE_ACTION_FW_ATT2[]={
  14,
  0,
  0x02,
  0x02,
  0x09,
  0x06,
  1,0,98,1,0,255,
  0x12,
  0x00,
  0x02,
};

static const unsigned char Over_th[]={
  13,
  0x02,
  0x02,
  0x09,
  0x06,
  0,0,96,3,10,255,// 'r','l','y','_','d','i','s','c','n','c','t',
  0x12,
  0x00,
  0x01,
};

static const unsigned char Under_th[]={
  13,
  0x02,
  0x02,
  0x09,
  0x06,
  0,0,96,3,10,255,//'r','l','y','_','c','n','c','t',
  0x12,
  0x00,
  0x01,
};


static const unsigned char SINGLE_ACTION_ATT4[]={
  5,
  0x00,
  0x01,
  0x01,
  0x02,
  0x02,
};
static const unsigned char manufacturer_name[]={
  34,
  0x00,
  0x09,
  31,
  'G','E','N','U','S',' ','P','O','W','E','R',' ','I','N','F','R','A','S','T','R','U','C','T','U',
  'R','E','S',' ','L','T','D'
};
static const unsigned char manufacturer_name_GIL[]={
  23,
  0x00,
  0x09,
  20,
  'G','E','N','U','S',' ','I','N','N','O','V','A','T','I','O','N',' ','L','T','D'
};
static const unsigned char firmware_ver[]={
	14,
	0x09,
	12,
	'G','0','0','0','0','.','1','6','0','0','0','1'
};
static const unsigned char firmware_ver_GIL[]={
	14,
	0x09,
	12,
	'G','0','0','0','0','.','1','6','0','5','0','1'
};
static const unsigned char hex_in_ascii[]={"0123456789ABCDEF"};
static const unsigned char APN[]={
	//genusgprs.com
	16,
	0,
	0x09,
	13,
	'g','e','n','u','s','g','p','r','s','.','c','o','m'
};


static const unsigned char event_log_profile_scaler_cap_obj[]={
  4,  //no of objects
  3,3,1,0,94,91,14,255,	 // current stamp(ip)
//3,3,1,0,91,7,0,255,     //current stamp(in)
  3,3,1,0,12,7,0,255,	// voltage stamp
  3,3,1,0,13,7,0,255,    //power factor
  3,3,1,0,1,8,0,255,     //cumulative energy
};


static const unsigned char NamePlateDetails_parameter_cap_obj[]={
      7,
      1,2,0,0,96,1,0,255,               //meter serial no
      1,2,0,0,96,1,1,255,               //manufacturer name
      1,2,1,0,0,2,0,255,               //firmware version for meter
      1,2,0,0,94,91,9,255,               //meter type
      1,2,0,0,94,91,11,255,               //caategory
      1,2,0,0,94,91,12,255,               //current raiting
      1,2,0,0,96,1,4,255,               //meter year of manufacture
    
};

static const unsigned char instantaneous_parameter_scaler_cap_obj[]={
	12,
    3,3,1,0,12,7,0,255,           // inst volt
	3,3,1,0,11,7,0,255,           // inst current ph
	3,3,1,0,91,7,0,255,          // inst current n
	
	3,3,1,0,13,7,0,255,         // pf
	3,3,1,0,14,7,0,255,            // freq
	3,3,1,0,9,7,0,255,                // kva
    3,3,1,0,1,7,0,255,             // inst load kW
	3,3,1,0,1,8,0,255,     //cumulative energy kwh
	3,3,1,0,9,8,0,255,       //cumulative energy kvah
	4,3,1,0,1,6,0,255,              // md kw
	4,3,1,0,9,6,0,255,                //md kva
//    3,3,0,0,94,91,8,255,           //pwr off duration
    3,3,0,0,94,91,14,255,           //pwr off duration
//    4,3,1,0,1,17,0,255,              // universal md kw
//	4,3,1,0,9,17,0,255,                //universal md kva



};


static const unsigned char instantaneous_parameter_cap_obj[]={
  18,//20,
  8,2,0,0,1,0,0,255,              //inst D&T0
  3,2,1,0,12,7,0,255,           // inst volt3
  3,2,1,0,11,7,0,255,           // inst current ph1
  3,2,1,0,91,7,0,255,          // inst current n2
  
  3,2,1,0,13,7,0,255,         // pf4
  3,2,1,0,14,7,0,255,            // freq            5
  3,2,1,0,9,7,0,255,                // kva
  3,2,1,0,1,7,0,255,             // inst load kW6
  3,2,1,0,1,8,0,255,     //cumulative energy kWh9
  3,2,1,0,9,8,0,255,       //cumulative energy kVAh
  4,2,1,0,1,6,0,255,              // md kw110
  4,5,1,0,1,6,0,255,              // md kw D&T11  
  4,2,1,0,9,6,0,255,               //md kVA 
  4,5,1,0,9,6,0,255,               //md kVA D&T  
//  1,2,0,0,96,7,0,255,          // no.of pwr failures12
//  3,2,0,0,94,91,8,255,           //pwr off duration7
  3,2,0,0,94,91,14,255,           //pwr on duration7
  1,2,0,0,94,91,0,255,          // cum tamper count13
  1,2,0,0,0,1,0,255,               // md count14(billing count)
  1,2,0,0,96,2,0,255,                 // cum prog count15
//  4,2,1,0,1,17,0,255,              // universal md kw110
//  4,5,1,0,1,17,0,255,              // universal md kw D&T11  
//  4,2,1,0,9,17,0,255,               //universal md kVA 
//  4,5,1,0,9,17,0,255,               //universal md kVA D&T  
//  3,2,0,0,0,1,2,255,            //d&t of last mdreset8
};

static const unsigned char instantaneous_parameter_scaler_buffer[]={
	12,
	0xfe,35, //vp
    0xfd,33, //ip
	0xfd,33, //in
	0xfd,255,//pf
	0xfd,44, //hz
    0,28,    //apparernt power kva
	0,27,    //active power kW
	
	2,30,    //kWh cumulative energy
	2,31,    //kvah cumulative energy
	0,27,    //Kw md
	0,28,     //kva md;
    0,6,     //power off duration min
//    0,27,    //universal Kw md
//	0,28,     //universal kva md;
};

static const unsigned char load_survey_parameter_cap_obj[]={
  4,
  8,2,0,0,1,0,0,255, 	//rtc D&T
//  3,2,1,0,11,5,0,255,           // inst current ph1
  3,2,1,0,12,27,0,255,           // inst volt3
  3,2,1,0,1,29,0,255,	//kWh stamp block
  3,2,1,0,9,29,0,255,	//kVAh stamp block
//  70,2,0,0,96,3,10,255, //relay status
//  1,2,0,0,96,12,5,255  //signal strength
  };

static const unsigned char blockload_survey_parameter_scaler_cap_obj[]={
  3,
//  3,3,1,0,11,5,0,255,           // inst current ph1
  3,3,1,0,12,27,0,255,           // inst volt3
  3,3,1,0,1,29,0,255,	//kWh stamp block
  3,3,1,0,9,29,0,255//kVAh stamp block
  };

#if DailyEnergy == 2
static const unsigned char dailyload_profile_parameter_cap_obj[]={
  3,
  8,2,0,0,1,0,0,255, 	//rtc D&T
  3,2,1,0,1,8,0,255,	//kWh stamp
  3,2,1,0,9,8,0,255,    //kVAh stamp
//  4,2,1,0,1,6,0,255, 	// md kw
//  3,2,0,0,94,91,8,255,  //Power failure duration
//  1,2,0,0,96,7,0,255,  //no. of power failures
//  1,2,0,0,96,51,1,255,  //no. of long power failures
  //1,2,0,0,96,51,2,255,  //no. of short power failures
//  63,2,0,0,96,50,0,255, //tamper status byte
};



static const unsigned char dailyload_profile_parameter_scaler_cap_obj[]={
  2,
  3,3,1,0,1,8,0,255,	//kWh stamp block
  3,3,1,0,9,8,0,255,   //kVAh stamp
//  4,3,1,0,1,6,0,255, // md kw
//  3,3,0,0,94,91,8,255,  //Power failure duration     
};
#endif




static const unsigned char bill_profile_parameter_cap_obj[]={
	17,//58,
	3,2,0,0,0,1,2,255,// D&T
	3,2,1,0,13,0,0,255,// avg pf
	
	3,2,1,0,1,8,0,255,// kWh stamp
	3,2,1,0,1,8,1,255, // kWh stamp z1
	3,2,1,0,1,8,2,255, // kWh stamp z2
	3,2,1,0,1,8,3,255, // kWh stamp z3
	3,2,1,0,1,8,4,255, // kWh stamp z4
	
//	3,2,1,0,1,8,5,255, // kWh stamp z5
//	3,2,1,0,1,8,6,255, // kWh stamp z6
//	3,2,1,0,1,8,7,255, // kWh stamp z7
//	3,2,1,0,1,8,8,255, // kWh stamp z8
	  
	3,2,1,0,9,8,0,255, // kVAh stamp
	3,2,1,0,9,8,1,255, // kVAh stamp z1
	3,2,1,0,9,8,2,255, // kVAh stamp z2
	3,2,1,0,9,8,3,255, // kVAh stamp z3
	3,2,1,0,9,8,4,255, // kVAh stamp z4
   
//	3,2,1,0,9,8,5,255, // kVAh stamp z5
//	3,2,1,0,9,8,6,255, // kVAh stamp z6
//	3,2,1,0,9,8,7,255, // kVAh stamp z7
//	3,2,1,0,9,8,8,255, // kVAh stamp z8
	
	4,2,1,0,1,6,0,255, // md kw
	4,5,1,0,1,6,0,255,// md D&T
	4,2,1,0,9,6,0,255,// md kva
	4,5,1,0,9,6,0,255,// md kva D&T
		
//	1,2,0,0,94,91,0,255,
//	1,2,0,0,94,91,128,255, // billing tamper count
	3,2,0,0,94,91,13,255, // billing power on duration
//	1,2,0,0,94,91,0,255,	// cumulative tamper count
//	3,2,0,0,94,91,8,255,    // Cumulative power-failure duration
//	3,2,0,0,94,91,13,255,	// Cumulative power-failure duration
//	1,2,0,0,0,1,0,255,	// cumulative MD reset count

};
static const unsigned char bill_profile_parameter_scaler_cap_obj[]={
	14,
	3,3,1,0,13,0,0,255, // pf
	3,3,1,0,1,8,0,255, // kWh stamp
	3,3,1,0,1,8,1,255, // kWh stamp z1
	3,3,1,0,1,8,2,255, // kWh stamp z2
	3,3,1,0,1,8,3,255, // kWh stamp z3
	3,3,1,0,1,8,4,255, // kWh stamp z4	
	
	
	3,3,1,0,9,8,0,255, // kVAh stamp
	3,3,1,0,9,8,1,255, // kVAh stamp z1
	3,3,1,0,9,8,2,255, // kVAh stamp z2
	3,3,1,0,9,8,3,255, // kVAh stamp z3
	3,3,1,0,9,8,4,255, // kVAh stamp z4
	
	4,3,1,0,1,6,0,255, // md kw
	4,3,1,0,9,6,0,255, // md kva

	3,3,0,0,94,91,13,255,   // billing power-on duration
//	3,3,0,0,94,91,8,255,	// Cumulative power-failure duration
//    3,3,0,0,94,91,13,255,	// Cumulative power-failure duration
};

static const unsigned char bill_profile_parameter_scaler_buffer[]={
14,//38,
0xfd,255, //power factor
0x02,30,  //kwh
0x02,30,  //kWh stamp z1
0x02,30,  //kWh stamp z2
0x02,30,  //kWh stamp z3
0x02,30,  //kWh stamp z4

0x02,31,  //kvah
0x02,31,  //kVAh stamp z1
0x02,31,  //kVAh stamp z2
0x02,31,  //kVAh stamp z3
0x02,31,  //kVAh stamp z4

0,27,     //md kw
0,28,     //md kva

0,6,      //bill Power On hours
};

static const unsigned char voltage_event_capture_obj[]={
  6,
  8,2,0,0,1,0,0,255,	  // D&T
  1,2,0,0,96,11,0,255, // event code voltage
  3,2,1,0,94,91,14,255,  //current ip stamp
  //3,2,1,0,91,7,0,255, //current in
  3,2,1,0,12,7,0,255,  // voltage stamp
  3,2,1,0,13,7,0,255,    //pf
  3,2,1,0,1,8,0,255  //KWH stamp
};

static const unsigned char current_event_capture_obj[]={
  6,
  8,2,0,0,1,0,0,255,	  // D&T
  1,2,0,0,96,11,1,255, // event code current
  3,2,1,0,94,91,14,255,  //current ip stamp
  //3,2,1,0,91,7,0,255, //current in
  3,2,1,0,12,7,0,255,  // voltage stamp
  3,2,1,0,13,7,0,255,    //pf
  3,2,1,0,1,8,0,255  //KWH stamp
};

static const unsigned char power_event_capture_obj[]={
  2,
  8,2,0,0,1,0,0,255,	  // D&T
  1,2,0,0,96,11,2,255// event code power

};

static const unsigned char transaction_event_capture_obj[]={
  2,
  8,2,0,0,1,0,0,255,	  // D&T
  1,2,0,0,96,11,3,255, // event code transaction
//  3,2,1,0,11,7,0,255,  //current ip stamp
//  //3,2,1,0,91,7,0,255, //current in
//  3,2,1,0,12,7,0,255,  // voltage stamp
//  3,2,1,0,13,7,0,255,    //pf
//  3,2,1,0,1,8,0,255  //KWH stamp
};

static const unsigned char other_event_capture_obj[]={
  6,
  8,2,0,0,1,0,0,255,	  // D&T
  1,2,0,0,96,11,4,255, // event code other
  3,2,1,0,94,91,14,255,  //current ip stamp
  //3,2,1,0,91,7,0,255, //current in
  3,2,1,0,12,7,0,255,  // voltage stamp
  3,2,1,0,13,7,0,255,    //pf
  3,2,1,0,1,8,0,255  //KWH stamp
};

static const unsigned char non_rollover_event_capture_obj[]={
  2,
  8,2,0,0,1,0,0,255,	  // D&T
  1,2,0,0,96,11,5,255, // event code non_rollover
//  3,2,1,0,11,7,0,255,  //current ip stamp
//  //3,2,1,0,91,7,0,255, //current in
//  3,2,1,0,12,7,0,255,  // voltage stamp
//  3,2,1,0,13,7,0,255,    //pf
//  3,2,1,0,1,8,0,255  //KWH stamp
};
static const unsigned char control_event_capture_obj[]={
  2,
  8,2,0,0,1,0,0,255,	  // D&T
  1,2,0,0,96,11,6,255, // event code non_rollover
//  3,2,1,0,11,7,0,255,  //current ip stamp
//  //3,2,1,0,91,7,0,255, //current in
//  3,2,1,0,12,7,0,255,  // voltage stamp
//  3,2,1,0,13,7,0,255,    //pf
//  3,2,1,0,1,8,0,255  //KWH stamp
};
static const unsigned char Diagnostics_event_capture_obj[]={
  2,
  8,2,0,0,1,0,0,255,	  // D&T
  1,2,0,0,96,11,7,255, // event code non_rollover
//  3,2,1,0,11,7,0,255,  //current ip stamp
//  //3,2,1,0,91,7,0,255, //current in
//  3,2,1,0,12,7,0,255,  // voltage stamp
//  3,2,1,0,13,7,0,255,    //pf
//  3,2,1,0,1,8,0,255  //KWH stamp
};

//static const unsigned char control_event_capture_obj[]={
//  6,
//  8,2,0,0,1,0,0,255,// D&T
//  1,2,0,0,96,11,6,255,// event code connection
//  63,2,0,0,96,50,0,255,//tamper status byte
//  1,2,1,0,96,128,5,255,//relay disconnection reason
//  71,3,0,0,17,0,2,255,//active load thrshld
//  3,2,1,0,1,8,0,255//KWH stamp
//};

static const unsigned char event_log_profile_scaler_buffer[]={
  4,
  0xfd,33,      //
  0xfe,35,
  0xfe,255,
  2,30
};
static const unsigned char OBJ_LIST[]={
  4,
  54 + DailyEnergy,
  55 + DailyEnergy,

  77,15,0,0,0,40,0,1,255,  //Association PC
  84,8,0,0,0,1,0,0,255,    //clock84
  35,1,0,0,0,42,0,0,255,   //logical device name
  35,1,0,0,0,96,1,0,255,   // Sr number


  77,15,0,0,0,40,0,2,255,  //Association MR
  35,17,0,0,0,41,0,0,255,  //SAP assignment list
  49,22,0,0,0,15,0,0,255,  //Single-action Schedule for Billing Dates
  91,20,0,0,0,13,0,0,255,  // activity calender
  
  77,7,1,1,0,94,91,0,255,  // snap shot of instantaneous parameters
  77,7,1,1,0,94,91,3,255,  // scaler unit of instantaneous parameters
  77,7,1,1,0,99,1,0,255,   // Load profile
  77,7,1,1,0,94,91,4,255,  // scaler unit of load Survey
  
#if DailyEnergy == 2  
  80,7,1,1,0,99,2,0,255,   // Daily Load profile
  77,7,1,1,0,94,91,5,255,  // scaler unit of load daily loadSurvey
#endif

  80,7,1,1,0,98,1,0,255,   // bill profile cumulative
  77,7,1,1,0,94,91,6,255,  // scaler unit of bill profile
  77,7,1,1,0,94,91,7,255,  // scaler unit of event log profile
  80,7,1,0,0,99,98,0,255,  //voltage related
  
  80,7,1,0,0,99,98,1,255,  // current related
  80,7,1,0,0,99,98,2,255,  //power related evnt profile(reverse )
  80,7,1,0,0,99,98,3,255,  //transaction related event
  80,7,1,0,0,99,98,5,255,  //non-rollover

  80,7,1,0,0,99,98,4,255,  //other event profile-magnet, cover open
  
  
//  80,7,1,0,0,99,98,6,255,  // control events
 //80,7,1,0,0,99,98,129,255,	//voltage  realted//dignostic event
//  49,70,0,0,0,96,3,10,255, //disconnector class

//  98,71,0,0,0,17,0,0,255,  //limiter class variants overvoltage
//  98,71,0,0,0,17,0,1,255,  //limiter class variants  undervoltage
//  98,71,0,0,0,17,0,2,255,  //limiter class variants overcurrent
//  98,71,0,0,0,17,0,3,255,  //limiter class variants ovrload
//  70,18,0,0,0,44,0,0,255,  //image transfer class instance 1
//  63,41,0,0,0,25,0,0,255,  //TCP/UDP set up
//  91,42,0,0,0,25,1,0,255,  //IPv4 set up
//  49,45,0,0,0,25,4,0,255,  // GPRS modem set up
    35,9,0,0,0,10,0,100,255, // tariff script
//  35,9,0,0,0,10,0,107,255, //image script

  42,3,0,1,0,11,7,0,255,	// phase current
  42,3,0,1,0,91,7,0,255,   // inst current n
  42,3,0,1,0,12,7,0,255,	//  phase voltage
  42,3,0,1,0,13,7,0,255,	//  pf
  
  42,3,0,1,0,14,7,0,255,   //freq
  42,3,0,1,0,1,7,0,255,   // inst load kW
  42,3,0,1,0,9,7,0,255,    // instantaneous load kVA
  //25/11/15 35,1,0,0,0,96,7,0,255,   // Number of power-failures
  
// 42,3,0,0,0,94,91,8,255,	// Cumulative power-failure duration
  42,3,0,0,0,94,91,14,255,	// Cumulative power-on duration
  35,1,0,0,0,94,91,0,255,	// cumulative tamper count
 // 35,1,0,0,0,96,91,128,255, // billing tamper count  
 /* 25/11/15*/
//  35,1,0,1,0,96,50,0,255,	//        TPRCNT_u8NeuMiss    =opr_data[0];
//  35,1,0,1,0,96,51,0,255,	//        TPRCNT_u8VHigh      =opr_data[1];
//  35,1,0,1,0,96,52,0,255,	//        TPRCNT_u8VLow       =opr_data[2];
//  35,1,0,1,0,96,53,0,255,	//        TPRCNT_u8Rev        =opr_data[3];
//  35,1,0,1,0,96,54,0,255,	//        TPRCNT_u8EL         =opr_data[4];
//  35,1,0,1,0,96,55,0,255,	//        TPRCNT_u8OC         =opr_data[5];
//  35,1,0,1,0,96,56,0,255,	//        TPRCNT_u8OverLoad   =opr_data[6];
//  35,1,0,1,0,96,57,0,255,	//        TPRCNT_u8Mag        =opr_data[7];
//  
//  35,1,0,1,0,96,58,0,255,	//        TPRCNT_u8NeuDis     =opr_data[8];
//  35,1,0,1,0,96,59,0,255,	//        TPRCNT_u8LowPF      =opr_data[9];
////  35,1,0,1,0,96,60,0,255,	//        TPRCNT_u835kv       =opr_data[10];
//  35,1,0,1,0,96,61,0,255,	//        TPRCNT_u8FreqTamp   =opr_data[11];
//  35,1,0,1,0,96,62,0,255,	//        TPRCNT_u8TC         =opr_data[12];
  35,1,0,0,0,0,1,0,255,	// cumulative MD reset count
  35,1,0,0,0,0,1,1,255,	// available billing period
  35,1,0,0,0,96,2,0,255,	// cumulative PROG  count
//25/11/15  35,3,0,0,0,0,1,2,255,	// Date and time of last MD reset
  35,1,0,0,0,96,1,1,255,   // manufacture name
  35,1,0,1,0,0,2,0,255,	// Firmware ver
  
  35,1,0,0,0,94,91,9,255,	// Meter type
  35,1,0,0,0,96,1,4,255,	// year of manufac
  35,1,0,1,0,0,8,0,255,	// Demand Integration Period
  35,1,0,1,0,0,8,4,255,	// Profile Capture Period
  35,1,0,0,0,96,11,0,255,  //voltage related
  35,1,0,0,0,96,11,1,255,  //current related
  35,1,0,0,0,96,11,2,255,  //power related
  35,1,0,0,0,96,11,3,255,  //transiction events

  35,1,0,0,0,96,11,4,255,  //other event profile-magnet, cover open
  35,1,0,0,0,96,11,5,255,  //non-rollover
//  35,1,0,0,0,96,11,6,255,  //control events
 // 35,1,0,0,0,96,11,129,255,  //dignostic events
  42,3,0,1,0,1,8,0,255,   //cumulative energy
  
  42,3,0,1,0,9,8,0,255,    //cumulative energy KVAh
//25/11/15  42,3,0,1,0,150,8,0,255,   //cumulative defraud mag energy 
//25/11/15  42,3,0,1,0,151,8,0,255,   //cumulative defraud neutral energy
  56,4,0,1,0,1,6,0,255,   // md kw
  
  56,4,0,1,0,9,6,0,255,    //md kva
  
  42,3,0,1,0,1,2,0,255,		// cum md kw
//  56,4,0,1,0,1,17,0,255,   // universal md kw
//  56,4,0,1,0,9,17,0,255,    //universal md kva
  35,1,0,0,0,94,91,12,255,    //current rating
  35,1,0,0,0,94,91,11,255,    //mter category
  77,7,1,0,0,94,91,10,255,    //name plate profile
  
  35,1,0,1,0,96,128,18,255,    //utility code
//42,63,0,0,0,96,10,01,255,//tamper status bytes00600A01
//25/11/15 42,3,0,1,0,142,8,0,255,//High Resolution kWh
//25/11/15 42,3,0,1,0,145,8,0,255,//High Resolution kvah
 
//25/11/15	42,3,0,1,0,142,7,0,255,//High Resolution kw
//25/11/15	42,3,0,1,0,145,7,0,255,//High Resolution kVA
//25/11/15	42,3,0,1,0,169,7,0,255, //High Resolution power kW (CT element) 
//  35,1,0,0,0,96,51,0,255,  //no. of daily power failures
//  42,63,0,0,0,96,50,0,255, //event status byte
//  70,40,0,0,0,25,9,0,255,  //push set up class (audit)
//  70,40,0,0,0,25,9,1,255,  //push set up class (bill)
//  70,40,0,0,0,25,9,2,255,  //push set up class (event)
//  70,40,0,0,0,25,9,3,255,  //push set up class (GPRS connctn)
//  18,60,0,0,0,0,2,3,255,   //message handler class
//  49,22,0,0,0,15,0,2,255,     //Single action shedule firmware
//  35,1,0,0,0,96,12,5,255,   //GSM field strength
  77,15,0,0,0,40,0,3,255   //Association US should be in last
////35,1,0,0,0,96,2,13,255,  //date of last firmware activation 0.0.96.2.13.255
////70,40,0,0,0,25,9,4,255,  //push set up class (SMS connctn)
////49,27,1,0,0,2,0,0,255,   //modem configuration
////  35,9,0,0,0,10,0,108,255, //Push script  41+16 41+17 39
////  35,1,0,1,0,0,4,2,255,	// Internal CT ratio
////  35,1,0,1,0,0,4,3,255,	// Internal PT ratio
////  35,1,0,0,0,94,91,1,255,	// bill tamper count
////  77,28,0,0,0,2,2,0,255,   //Auto Answer
////  63,29,1,0,0,2,1,0,255,   //auto connect
////  56,44,0,0,0,25,3,0,255,  //PPP set up
////  35,1,0,1,0,96,128,5,255,   //relay disconnect reason
////  35,1,0,0,0,96,51,1,255,  //no. of long daily power failures
////  42,63,0,0,0,96,50,1,255, //Alerts status byte
////  42,63,0,0,0,96,50,2,255,//Transaction status byte
////  42,63,0,0,0,96,50,3,255,//tamper,alert,transaction
////  42,63,0,0,0,96,50,4,255,//Zone wise load limit
////  35,1,0,0,0,96,7,9,255,   // Number of long power-failures
////35,1,0,0,0,96,7,2,255,   // Number of short power-failures

	
};


#if DailyEnergy == 2
static const unsigned char dailyload_profile_parameter_scaler_buffer[]={
  2,
  2,30,    // kWh
  2,31,    //kvah
//  0,27,   //Kw md
//  0,6,
};
#endif



static const unsigned char blockload_survey_parameter_scaler_buffer[]={
3,//4,
//0xfe,33,// 3,2,1,0,11,7,0,255,           // inst current ph1
0xfe,35,//  3,2,1,0,12,27,0,255,           // inst volt3
0x00,30,//  3,2,1,0,1,29,0,255,	//kWh stamp block
0x00,31// 3,2,1,0,9,29,0,255,	//kVAh stamp block
};


static const unsigned char A[]={
0x10,//length of tag
0x04,//choice of user information
0x0E,//lengh of octet string
0x08,//tag
0x00,//Quality of service(optional not present)
0x06,//DLMS version number
0x5F,//ASN.1 tag[0]
0x1F,//ASN.1 tag[1]
0x04,//length of ASN.1
0x00,//number of unused bits
0x00,//0x01;//max_receive_pdu_size[0]23/02/2007
0x80,//0xFF;//max_receive_pdu_size[1]23/02/2007
0x00,//VAA-name-component[0]
0x07
};
static const unsigned char B[]={
0x81,
0x80,
0x14,//next no. of bytes
0x05,//max tx info size
0x02,//no. of byte used
0x00,
0x00,
0x06,//max rx info size
0x02,//no. of byte used
0x00,
0x00,
0x07,//max win size tx
0x04,//no. of byte used
0x00,
0x00,
0x00,
0x00,
0x08,//max win size rec
0x04,//no. of byte used
0x00,
0x00,
0x00,
0x00
};



static const unsigned char obis_fill[]={
0x09,0x06,
0x00,//val-A
0x00,//val-B
0x0A,//val-C
0x00,//val-D
0x64,//val-E
0xFF//val-F
};

static const unsigned char method_fill[]={
32,
0x01,
0x02,
0x02,
0x0F,
0x01,
0x03,
0x01,
//if method not equal to 1
0x04,
0x02,
0x02,
0x0F,
0x01,
0x03,
0x01,
0x02,
0x02,
0x0F,
0x02,
0x03,
0x01,
0x02,
0x02,
0x0F,
0x03,
0x03,
0x00,
0x02,
0x02,
0x0F,
0x04,
0x03,
0x00
};


static const unsigned char sort_obj[]={
  19,
  0x00,
  0x02,
  0x04,
  0x12,
  0x00,
  0x00,
  0x09,
  0x06,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x00,
  0x0f,
  0x00,
  0x12,
  0x00,
  0x00
};
static const unsigned char sort_obj1[]={
  19,
  0x00,
  0x02,
  0x04,
  0x12,
  0x00,
  0x08,
  0x09,
  0x06,
  0x00,
  0x00,
  0x01,
  0x00,
  0x00,
  0xff,
  0x0f,
  0x02,
  0x12,
  0x00,
  0x00
};



static const unsigned char Set_tou[]={
7,
0xC5,//set_response
0x02,//normal
0x00,//invoke_id,priority
0x00,
0x00,
0x00,
0x00
};


static const unsigned char AARQ[] = {
	25,
  0x61,
  0x00,
  //application_context_name
  0xA1,
  0x09,
  0x06,
  0x07,
  0x60,
  0x85,//country
  0x74,//country name
  0x05,//Organisation
  0x08,//DLMS_UA
  0x01,//app_context
  0x01,//LN reference
  //application_context_name

  //result
  0xA2,//tag of result component
  0x03,//length of taged component
  0x02,//choice for result
  0x01,//length of result*/
  0x00,//result

  //result_source_diagnostic
  0xA3,//tag for result diagnostic
  0x05,//length of taged component
  0x00,//tag for acse_service_user/provider(0xA1/0xA2)
  0x03,//length
  0x02,//choice of result_source_diagnostic
  0x01,//length of value*/
  0x00,//value of diagnosis
  //result_source_diagnostic
};
static const unsigned char AARE_PASS[]={
  18,
  0x0E,
  0x88,// encoding of the tag of the acse-requirements field ([8], IMPLICIT, Context-specific)
  0x02,// encoding of the length of the tagged component�s value field.
  0x07,// encoding of the number of unused bits in the last byte of the BIT STRING
  0x80,// encoding of the authentication functional unit (0)
  0x89,// encoding of the tag ([9], IMPLICIT, Context-specific)
  0x07,// encoding of the length of the tagged component�s value field
  0x60,// encoding the value of the object identifier:- high-level-security-mechanism-name (5)
  0x85,
  0x74,
  0x05,
  0x08,
  0x02,
  0x02,
  0xAA,// encoding of the tag ([10], Context-specific)
  0x12,// encoding of the length of the tagged component�s value field
  0x80,// encoding of the choice for Authentication-value (charstring [0] IMPLICIT GraphicString)
  0x10,// encoding of the length of the Authentication-information�s value field (8 octets)
  };



/*******************************************************************************
 * External Variables
 ******************************************************************************/
//extern unsigned char out[16];

/*******************************************************************************
 * Global Function Prototypes
 ******************************************************************************/
void recv_frm(void);
void init_dlmsvar(unsigned char);
void resp(void);
void UART_vResetDlmsData(void);
void date_time(unsigned char dd,unsigned char mm,unsigned char yy,unsigned char hh,unsigned char min,unsigned char sec,unsigned char flg);
void array(unsigned char len,unsigned char flag);
void structure(unsigned char len);/********/
void val_4byt(unsigned char a,unsigned char b,unsigned char c,unsigned char d);
void sca_unit(char a,unsigned char b,unsigned char add_zero);
void unsigned8(unsigned char value,unsigned char flag);
void val_2byt(unsigned char a,unsigned char b);
void obiscode(unsigned char a,unsigned char b,unsigned char c,unsigned char d,unsigned char e,unsigned char f);
void daily_enr_fill(u16);
void load_survey_fill(u16);
void sr_no_ascii(void);
void fill_info(unsigned char const *);
void Uart_BillFill(unsigned char );
void object_list(void);
void day_profile(unsigned char );
void capture_objects_filler(unsigned char const *);
void tamper_compart(unsigned char ,unsigned int ,unsigned int );
void buffer_scaler_filler(unsigned char const *);
void bit_string(u8,u8);

void fill_firmware_version(void);
void zero_flash_opr_data(void);
//void tpr_fill(unsigned int comptt_address);
void octet_s(unsigned char len,unsigned char flag);
u16 fcs_cal(u16,u8*,u16);
u8 set_resp(void);
u8 action_resp(void);




/*******************************************************************************
 * Local Function Prototypes
 ******************************************************************************/
void bill_buffer(void);
void vBlock_transfer_list(void);
void image_block_status(u16 );
u8 tou_pssv_store(void);

void SaveDisplayList(u16 addrr,u8 num,u8 auto_push);
u8 save_tou_pass_data(u8 u8index_t, u8 *info_t, u8 buffer_trace);

unsigned int fcs_cal(unsigned int fcs,unsigned char *cp,unsigned int len);
unsigned char fcs(unsigned char *cp,unsigned int len,unsigned char flag);
unsigned char hdlc1_s4(void);

unsigned char data_dec(void);
void tpr_fill(unsigned int comptt_address);
void prof_curr_month_MD(void);
void tamper_compart(unsigned char,unsigned int,unsigned int);
void tamper_buffer(unsigned char event);
void day_profile(unsigned char passive);
void fill_tamper(unsigned char add_block,unsigned char add_page);
void control_field(void);
void req_type(void);
void conf_err(unsigned char len);
void access_rights(unsigned int cnt_att,unsigned char att1,unsigned char att2,unsigned char att3,unsigned char att4,unsigned char att5,unsigned char att6,unsigned char att7,unsigned char att8,unsigned char att9,unsigned char att10,unsigned char att11,unsigned char sec_ass,unsigned char method,unsigned char m1,unsigned char m2,unsigned char m3,unsigned char m4);
void array(unsigned char len,unsigned char flag);
void conf_ser(unsigned char len);
void Clear_Buff(void);
void conf_err(unsigned char len);
void date_time(unsigned char dd,unsigned char mm,unsigned char yy,unsigned char hh,unsigned char min,unsigned char sec,unsigned char flg);
void enum_d(unsigned char len);
void fill_profilesel(unsigned char const *p_f);
void fill_A0(unsigned char len);/********/
void info_l(void);
void info_l5(void);
void info_l6(void);
void info_l7(void);
void info_l8(void);
void load_date(unsigned char dd,unsigned char mm,unsigned char yy,unsigned char dofw);
void load_time(unsigned char hh,unsigned char min,unsigned char sec);
void Loadsurvey_buffer(void);
void long_unsign(void);			//note: used only when first byte is 0x00
void obiscode(unsigned char a,unsigned char b,unsigned char c,unsigned char d,unsigned char e,unsigned char f);
void octet_s(unsigned char len,unsigned char flag);
void profile_sel(unsigned int ic,unsigned char att,unsigned char obis_a,unsigned char obis_b,unsigned char obis_c,unsigned char obis_d,unsigned char obis_e,unsigned char obis_f);
void recv_frm(void);
void sca_unit(char a,unsigned char b,unsigned char add_zero);
void seg_flags(void);
void fill_0d(void);
void enum_d2(unsigned char len);
void fill_0b(void);
void val_4byt(unsigned char a,unsigned char b,unsigned char c,unsigned char d);
void val_4byt2(unsigned char a,unsigned char b,unsigned char c,unsigned char d);
void val_2byt2(unsigned char a,unsigned char b);
void Start_Info(void);
void send_type(void);
void structure(unsigned char len);/********/
void val_2byt(unsigned char a,unsigned char b);
void unsigned8(unsigned char value,unsigned char flag);
void fill_obj_list(unsigned char const *o_f);
void asso_status(void);
void auth_name(void);
void class_sel(unsigned int ic,unsigned char ver,unsigned char obis_a,unsigned char obis_b,unsigned char obis_c,unsigned char obis_d,unsigned char obis_e,unsigned char obis_f);
void event_log_profile_buffer_s(void);
void get_resp(void);
void get_resp1(void);
void buffer_instantaneous_parameter(void);
void log_name2(unsigned char a,unsigned char b,unsigned char c,unsigned char d,unsigned char e);
void log_name23(unsigned char temp);
void sap__assg_list(void);
void sort_object(void);
void xdlms_type(void);
void fill_info(unsigned char const *f_d);
void init_dlmsvar(unsigned char flag);
void dlms_timecntr(void);
void integer8(char value);
void integer16(unsigned char byte1,unsigned char byte2);
void fill_info1(unsigned char *f_d);
unsigned int a8_to_u161(unsigned char *char_array_ptr);

void log_trans_event(unsigned int event);
void MASS_READ1(unsigned char eprom_no,unsigned char block_no, unsigned char no_of_block);
void daily_enr_fill(unsigned int comptt_address);

void value_monitored(unsigned char identifier);
void object_list(void);
void capture_objects_filler(unsigned char const *p_f);
void buffer_scaler_filler(unsigned char const *s_f);
void UART_vInit(void);
void UART_vStart(void);
void UART_vStop(void);
void bill_buffer(void);
void buffer_instantaneous_parameter(void);
void buffer_NamePlateDetails_parameter(void);
void Uart_CurrentBillFill(void);
/*******************************************************************************
 * Extern variables
 ******************************************************************************/

//extern u8 week1_set;
/*******************************************************************************
 * End of file
 ******************************************************************************/






#endif
