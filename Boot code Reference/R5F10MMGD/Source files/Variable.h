/***********************************************************************************************************************
* File Name    : variable.h
* Version      : CodeGenerator for RL78/L12 V2.00.00.07 [22 Feb 2013]
* Device(s)    : R5F10MMG
* Tool-Chain   : CC-RL
* Description  : This file implements device driver for CGC module.
* Creation Date: 7/2/2013
***********************************************************************************************************************/

#ifndef VAR_H
#define VAR_H
#include "rtc.h"
#include "iodefine.h"

/*********** User define macro ****************************/
#define EXT_X1					0
#define X1_FREQ					3
#define SampleCounts 			0 		//1 = 1953, 0 = 3906
#define LCD_TYPE 				3
#define Genus_Protocol 			0
#define IR 						0
#define RJ 						1
#define RollOverFile   			0


/*********** Hard Coded Constants ************************/
#define BIT0					0x01
#define BIT1					0x02
#define BIT2					0x04
#define BIT3					0x08
#define BIT4					0x10
#define BIT5					0x20
#define BIT6					0x40
#define BIT7					0x80

#define MET_u32mcl_kw  					1125000000
#define CreepThrMainElemntInWatt		1.8
#define EarthLoadDetectionThr			20  
#define ReverseDetectionThr				60
/* EarthLoadDetectionThr *****************************
 * Set 16 for 6.25% diff
 * Set  8 for 12.5% diff
 *****************************************************/
#define EarthLoadDiffFactor				16
#define DailyEnergy 					2
#define Vol_Rating_div100 				240
#define I_MAX_Rating_div1000 			30
#define Vol_Rating						24000  //Vol_Rating_div100*100
#define I_MAX_Rating					30000  //I_MAX_Rating_div1000*1000

/* I_dcinac ******************************************
 * (I_MAX_Rating*100)/(2828) Here 2828 = 2*1.414
 * For  5-30 A Meter	(30000*100/2828 = 1060)
 * For 10-60 A Meter	(60000*100/2828 = 2121)
 *****************************************************/
#define I_dcinac      				    1060 //((u32)I_MAX_Rating*100)/((u32)2828)  	//(I_MAX_Rating*100)/(2828) // 2828 = 2*1.414 *100
#define MaxDarraySize           		154   									//divisible of 7 
#define BattBckupTime 					60 										// time_in_sec kept in two bytes
#define PushToAutoTime					15  									// time_in_sec*200 kept in two bytes
#define PushToAutoTimeButtonPress		4
#define LCD_MAX_AUTO_LOC        		200
#define LCD_MAX_PUSH_LOC        		200
#define KWh_RolloverValue 				1000000
#define MaxBillDate             		12
#define EnbleDispIDTwoByte				0										// if this is  1 Display id will be considered in two bytes
#define NeuDistVoltThr					30000 									// 300 V = 300*100 i.e 100 for resolution
#define NeuMissCurntThr					900

/*****************************************************
 * Set only 4 or 8 if required other than this change
   profile values in dlms.h
 *****************************************************/
#define TariffCnt						4
#define ACMagnetPersistTime				5
#define tpr_u16TCStrTime        		1000
#define TprSel_NeuMiss          		0x00000001		
#define TprSel_VoltageHigh				0x00000002		
#define TprSel_VoltageLow				0x00000004		
#define TprSel_Reverse					0x00000008		
#define TprSel_EL			    		0x00000010		
#define TprSel_CurrentHigh				0x00000020		
#define TprSel_OverLoad 				0x00000040		
#define TprSel_TCOpen					0x00000080		
#define TprSel_MagnetTamper	    		0x00000100		
#define TprSel_NeutralDis     			0x00000200		
#define TprSel_35KV			    		0x00000400		
#define TprSel_LowPF		    		0x00000800
#define TprSel_AbFreq		    		0x00001000
#define DLMS_MAX_BUFF_SIZE      		512  			//all code design according to 512(do not decrease), increase only accepted to 2030


#define bat_disable						PFSEG3 &= ~BIT3; \
										PM3 &= ~BIT3;    \
										P3 |= BIT3

#define bat_enable						PFSEG3 &= ~BIT3; \
										PM3 &= ~BIT3;    \
										P3 &= ~(BIT3)
										
/*********** Port configrations *****************************************************************************/			
#define rev_led             			(0x0040)
#define el_led              			(0x0080)
#define ph_led              			(BIT6) 
#define POWER_UP            			(BIT0) 
#define TC_OPEN_Bit						(BIT2)
#define TC_OPEN_Port					P4
#define DN_Switch_Bit					BIT5
#define DN_Switch_Port					P12


/************************************************************************************************************
Global Constants
*************************************************************************************************************/
static const unsigned char DefaultCurrentRating[]={"5-30A"};
static const unsigned char UtilituID[]={"DHBVN"};
static const unsigned char FirmId[]={0x1B,0xD0}; 
static const unsigned char FirmIdDisp[]={0,4,4,5,0,0}; 
static const unsigned char FirmIdDispButton[]={0,4,4,5,0,0}; 
static const unsigned char Default_AutoDisplay[]={11,12,13,16,20,175,4,217,8};
static const unsigned char Default_PushDisplay[]={13,14,15,16,17,18,11,12,22,23,234,235,236,237,238,239,4,5,6,9,10};


/***********************************************************************************************************************
Typedef definitions
***********************************************************************************************************************/
typedef unsigned long int u32;
typedef signed long int s32;
typedef unsigned int u16;
typedef int s16;
typedef unsigned char u8;
typedef char s8;
typedef signed char sc8;
typedef double f32;
struct bit_define 
{
    unsigned char  b0:1;
    unsigned char  b1:1;
    unsigned char  b2:1;
    unsigned char  b3:1;
    unsigned char  b4:1;
    unsigned char  b5:1;
    unsigned char  b6:1;
    unsigned char  b7:1;
};

union  flag_union
{
		struct bit_define b_bit;   
		unsigned char all;
                   
};
union change32
{
  u32 u32_temp;
  u16 u16_temp[2];
  u8 u8_temp[4];
};

//Loadsurvey
struct LS_bits
{
    u8 ls_of			:1;
    u8 ls_rtc_fill		:1;
    u8 ls_rev_fill		:1;
    u8 ls_daycng_f		:1;
    u8 ls_fg_f			:1;
    u8 miss_load_survey_f	:1;
//    u8 Take_Bill  		:1;
    u8 Unused			:2;
};
union LS_union
{
	struct LS_bits b;
  	u8 all;    
};
struct DE_bits
{
    u8 miss_dailyenr			:1;
    u8 daily_rev_fill		:1;
    u8 dls_of		:1;
//    u8 ls_daycng_f		:1;
//    u8 ls_fg_f			:1;
//    u8 miss_load_survey_f	:1;
//    u8 Take_Bill  		:1;
    u8 Unused			:2;
};
union DE_union
{
	struct DE_bits b;
  	u8 all;    
};			
struct TP_bits
{
    u8 CurrentRelatedEvent	:1;
	u8 VoltageRelatedEvent  :1;
//    u8 ls_rtc_fill		:1;
    u8 OtherEvent		:1;
    u8 NonRollEvent		:1;
    u8 DiagnosticsEvent		:1;
    u8 TransactionEvent		:1;
//    u8 Take_Bill  		:1;
    u8 Unused			:2;
};
union TP_union
{
	struct TP_bits b;
  	u8 all;    
};
struct TP1_bits
{
    u8 mag_active_f		:1;
    u8 mag_tpr_f		:1;
    u8 mag_tprstr_f		:1;
    u8 mag_tprrestr_f		:1;
    u8 rev_tpr_f		:1;
    u8 rev_tprstr_f 		:1;
    u8 rev_tprrestr_f  		:1;
    u8 ol_tpr_f			:1;
};
union TP1_union
{
	struct TP1_bits b;
  	u8 all;    
};
struct TP2_bits
{
    u8 el_tpr_f			:1;
    u8 el_tprstr_f		:1;
    u8 el_tprrestr_f		:1;
    u8 oc_tpr_f			:1;
    u8 oc_tprstr_f		:1;
    u8 oc_tprrestr_f 		:1;
    u8 ol_tprrestr_f  		:1;
    u8 ol_tprstr_f		:1;
};
union TP2_union
{
	struct TP2_bits b;
  	u8 all;    
};
struct TP3_bits
{
    u8 neu_miss_f			:1;
    u8 neu_miss_tpr_f		:1;
    u8 neu_miss_tprrestr_f	:1;
    u8 neu_miss_tprstr_f	:1;
    u8 neu_dis_f			:1;
    u8 neu_dis_tpr_f 		:1;
    u8 neu_dis_tprstr_f  	:1;
    u8 neu_dis_tprrestr_f	:1;
};
union TP3_union
{
	struct TP3_bits b;
  	u8 all;    
};
struct TP4_bits
{
    u8 LowPF_tpr_f			:1;
    u8 LowPF_tprstr_f		:1;
    u8 LowPF_tprrestr_f		:1;
    u8 AbFreq_tpr_f			:1;
    u8 AbFreq_tprstr_f		:1;
    u8 AbFreq_tprrestr_f	:1;
    u8 neu_fea_f1	  		:1;
    u8 dcchop_f				:1;
};
union TP4_union
{
	struct TP4_bits b;
  	u8 all;    
};
struct TP5_bits
{
    u8 kv35_tpr_f		:1;
    u8 dcinac_f			:1;
    u8 kv35_tprstr_f	:1;
    u8 kv35_tprrestr_f	:1;
    u8 tc_tprstr_f		:1;
    u8 tc_tpr_f			:1;
    u8 tc_tprstopmode	:1;
};
union TP5_union
{
	struct TP5_bits b;
  	u8 all;    
};
struct TP6_bits
{
    u8 LowPF_tpr_f				:1;
    u8 bat_discharge_f			:1;
    u8 bat_discharge_tprstr_f	:1;
//	u8 AbFreq_tpr_f				:1;
//	u8 AbFreq_tprstr_f			:1;
//	u8 AbFreq_tprrestr_f		:1;
    u8 sleep_f	  				:1;
    u8 tc_exit_f				:1;
};
union TP6_union
{
	struct TP6_bits b;
  	u8 all;    
};

struct TP7_bits
{
	u8 LowVolt_tpr_f 		:1;
	u8 LowVolt_tprstr_f 	:1;
	u8 LowVolt_tprrestr_f 	:1;
	u8 highv_tprstr_f 		:1; 
	u8 highv_tprrestr_f 	:1;
	u8 high_v_f 			:1;
};
union TP7_union
{
	struct TP7_bits b;
  	u8 all;    
};

#define MET_bph_ct_f1			flag0.b_bit.b0
#define MET_bpf_sign			flag0.b_bit.b1
#define lag_cal_done			flag0.b_bit.b2
#define lag_ok					flag0.b_bit.b3
#define MET_bsample_ready		flag0.b_bit.b4
#define MET_bcal_done_f			flag0.b_bit.b5
#define MET_bcal_mode_f			flag0.b_bit.b6
#define MET_bneg_vol_cnt_f		flag0.b_bit.b7

#define DISP_bParCng			flag1.b_bit.b0
#define MET_bkw_negative_f		flag1.b_bit.b1
#define MET_bkw_el_negative_f	flag1.b_bit.b2
#define MET_bEl_glow			flag1.b_bit.b3
#define MET_bsleep_mode_f		flag1.b_bit.b4
#define MET_bten_kva_f			flag1.b_bit.b5
#define power_up_f1				flag1.b_bit.b6
#define MET_bcal_freq_f			flag1.b_bit.b7

#define PWR_bBatt_backup_f		flag2.b_bit.b0
#define PWR_bbat_discharge_f	flag2.b_bit.b1
#define batt_disp_f				flag2.b_bit.b2
#define sw_lock_f				flag2.b_bit.b3
#define first_pres_f			flag2.b_bit.b4
#define disp_lock_f				flag2.b_bit.b5
#define cal_switch_f			flag2.b_bit.b6
#define dispshow_unlock_f		flag2.b_bit.b7

#define cal_pwr_f				flag3.b_bit.b0
#define DISP_bToggle			flag3.b_bit.b1
#define TPR_bLowPF				flag3.b_bit.b2
//#define TPR_bTransactionEvent	flag3.b_bit.b3
#define disable_bkup_f			flag3.b_bit.b4
#define MET_bResetOn80Cnts		flag3.b_bit.b5
#define fg_done_f				flag3.b_bit.b6
#define BattLowCheck			flag3.b_bit.b7

#define one_byte_add_f			flag4.b_bit.b0
#define optical_f				flag4.b_bit.b1
#define rj_disc_f2				flag4.b_bit.b2
#define rj_disc_f				flag4.b_bit.b3
#define nrm_flag				flag4.b_bit.b4
#define asso0_flag				flag4.b_bit.b5
#define asso1_flag				flag4.b_bit.b6
#define asso2_flag				flag4.b_bit.b7

#define dlms_rece_flag			flag5.b_bit.b0
#define infore_flag				flag5.b_bit.b1
#define conf_serror_flag		flag5.b_bit.b2
#define sel_access_flag			flag5.b_bit.b3
#define sel_access_flag1		flag5.b_bit.b4
#define four_pass_f				flag5.b_bit.b5
#define asso3_flag				flag5.b_bit.b6
#define cosem_flag				flag5.b_bit.b7

#define last_block				flag6.b_bit.b0
#define send_type_multi_f		flag6.b_bit.b1
#define buffer_first_not_fill_f	flag6.b_bit.b2
#define multi_filling_f			flag6.b_bit.b3
#define GPR_u8TCPIP_F			flag6.b_bit.b4
#define Selaccess_err_f         flag6.b_bit.b5
#define rj_f 					flag6.b_bit.b6
//#define four_pass_f			flag6.b_bit.b5
//#define asso3_flag			flag6.b_bit.b6
#define ram_crpt_f0				flag6.b_bit.b7

#define KEY_bPress  			flag7.b_bit.b0
#define switch_disp_f 			flag7.b_bit.b1
#define MET_blag_calib_f		flag7.b_bit.b2
#define MET_bcalib_flag			flag7.b_bit.b3
#define MET_bkwh1p_f 			flag7.b_bit.b4
#define MET_bten_wh_f 			flag7.b_bit.b5
#define MET_bkvah1p_f 			flag7.b_bit.b6
#define MET_bten_kvah_f 		flag7.b_bit.b7


//#define dls_of				flag8.b_bit.b0
//#define ls_of					flag8.b_bit.b1
#define TPR_bFirstRestrTpr		flag8.b_bit.b0
#define	TPR_bFirstStrTpr		flag8.b_bit.b1
#define volt_of					flag8.b_bit.b2
#define curr_of					flag8.b_bit.b3
#define on_off_of				flag8.b_bit.b4
#define trans_of				flag8.b_bit.b5
#define others_of				flag8.b_bit.b6
#define control_of				flag8.b_bit.b7

#define Diagnostics_of			flag9.b_bit.b0
#define nonroll_of				flag9.b_bit.b1
#define analyse_cal_pkt_flag 	flag9.b_bit.b2
#define rtc_set_flag			flag9.b_bit.b3
#define pow_up_f				flag9.b_bit.b4
#define disp_kwh_f				flag9.b_bit.b5
#define lock_kwh_f              flag9.b_bit.b6
#define pow_nextday_f           flag9.b_bit.b7

#define TOD_bcalendar_change_f	flag10.b_bit.b0
#define TOD_bSeason_f			flag10.b_bit.b1
#define TOD_bActive_Calendar	flag10.b_bit.b2
#define	TOD_bzonecng_f0			flag10.b_bit.b3
#define TOD_bstore_eoicngmd_f6	flag10.b_bit.b4
#define TOD_bActive_f			flag10.b_bit.b5
#define savebill_flag			flag10.b_bit.b6
#define missbill_f				flag10.b_bit.b7

#define mri_bill_f				flag11.b_bit.b0
#define bill_f					flag11.b_bit.b1
#define zone_default_f0			flag11.b_bit.b2
#define IrDA_f		        	flag11.b_bit.b3
#define disp_zoneindex_f    	flag11.b_bit.b4
#define MET_u8pfsign_cal_f  	flag11.b_bit.b5
#define disp_lock_f1			flag11.b_bit.b6
#define MEM2_256_f				flag11.b_bit.b7
//#define miss_load_survey_f	flag11.b_bit.b5
//#define ls_rev_fill			flag11.b_bit.b6
//#define ls_rtc_fill			flag11.b_bit.b7

#define EPROM_bChksum_ok  		flag12.b_bit.b0
#define EPR_bResetFlag  		flag12.b_bit.b1
#define EPR_bReadFlag	  		flag12.b_bit.b2
#define EPR_bWriteFlag  		flag12.b_bit.b3
#define EPR_bFirstReadFlag  	flag12.b_bit.b4
#define busbusy_f0				flag12.b_bit.b5
#define acknotr_f0				flag12.b_bit.b6

#define seccng_f				flag13.b_bit.b0
#define mincng_f				flag13.b_bit.b1
#define hrscng_f				flag13.b_bit.b2
#define	daycng_f				flag13.b_bit.b3
#define monthcng_f				flag13.b_bit.b4
#define rtc_calib_f				flag13.b_bit.b5
#define RtcRst_DispFlag			flag13.b_bit.b6
#define RTC_bReadFlag			flag13.b_bit.b7

//#define mag_active_f			flag14.b_bit.b0
//#define mag_tpr_f				flag14.b_bit.b1
//#define mag_tprstr_f			flag14.b_bit.b2
//#define	mag_tprrestr_f		flag14.b_bit.b3
//#define monthcng_f			flag14.b_bit.b4
//#define rtc_calib_f			flag14.b_bit.b5
//#define RtcRst_DispFlag		flag14.b_bit.b6
//#define mag_tpr_f				flag14.b_bit.b7

#define rtc_correction_f 	 	flag15.b_bit.b0
#define RTC_Onetimecal_f 	 	flag15.b_bit.b1
#define init_lcd_f           	flag15.b_bit.b2
#define zcd1_f               	flag15.b_bit.b3
#define RTC_BATT_discharge_f 	flag15.b_bit.b4
#define RTC_bat_ststus_f     	flag15.b_bit.b5
#define zcd_f                	flag15.b_bit.b6
#define Mask_curntreg_f      	flag15.b_bit.b7   // do not change this flag if change then specify in asm file 


#define MET_bStore_energy_f		flag21.b_bit.b0
#define LCD_DispRTC_Time_f		flag21.b_bit.b1


/**********************************************************************
 * Memory Mapping 
***********************************************************************/
 
/***** Storation Block 00 *********************************************
 * Circular Buffer Energy start add 0x0000, end add 0x00f0
***********************************************************************/
#define CircularBufferEnergy 		 		0x0000

/***** Storation Block 01 *********************************************
 * Circular Buffer Power  start add 0x0100, end add 0x01f0
 *********************************************************************/
#define CircularBufferMDLSkwh      			0x0100

/****** Storation Block 02 ********************************************
 * Storation zonal energy   start add 0x0200, end add 0x0270
 * Storation zonal MD start start add 0x0280, end add 0x02f0
 *********************************************************************/
#define Zone_energy_data_Add    			0x0200
#define Zone_MD_energy_data_Add 			0x0280

/****** Storation Block 03 *******************************************
 * Storation Power On Minutes add 0x0300, end add 0x03f0
**********************************************************************/
#define PowerOnMinStoreAdd			 		0x0300

/****** Storation Block 04 *******************************************/
#define CumMDAddr               			0x0400
#define FirstStrAddr            			0x0410
#define FirstRestrAddr          			0x0420 
#define LastStrAddr           				0x0430
#define LastRestrAddr           			0x0440
#define TPRCNT_Addr             			0x0450
#define RTC_StatusByte_Add      			0x0480
#define Dupli_SerialNo_UtilityID_add 		0x0490
#define Dupli_PCBSerialNo_add        		0x04a0
#define Dupli_UpfCalibrationCoffadd  		0x04b0
#define Dupli_LagCalibrationCoffadd  		0x04c0
#define NeutralTestCountAdd			 		0x04d0
#define DefraudEnergyRegAdd			 		0x04e0
#define PowerDownByteAdd        			0x04f0


/****** Storation Block 05 *******************************************/
#define SerialNo_UtilityID_add  			0x0500
#define PCBSerialNo_add         			0x0510
#define RTC_Calibration_add     			0x0520
#define kw_pulse_c_addrs        			0x0530                   
#define UpfCalibrationCoffadd   			0x0540
#define LagCalibrationCoffadd   			0x0550
#define Diagnostics_status      			0x0570
#define volt_tamp_status        			0x0580
#define curr_tamp_status        			0x0590
#define other_tamp_status       			0x05A0
#define trans_tamp_status       			0x05B0
#define nonroll_status          			0x05c0
#define control_tamp_status     			0x05d0
#define cum_tpr_addrs           			0x05e0                 // cum tamper data address or location 
#define pwr_tamp_status         			0x05F0

/****** Storation Block 06 *********************************************
 * Storation Block 06  Add 0x0600 to 0x06c0 reserved for bill MD Import
************************************************************************/
#define billmd_data_addrs       			0x0600                   // bill md data starting address
#define BatteryStatus						0x06d0
#define UtilityIDAdd				 		0x06e0
#define CodeAdd                 			0x06f0

/****** Storation Block 07 ************************************************  
 * Storation Block 07 Add 0x0700 to 0x07c0 reserved for bill Energy Import
***************************************************************************/
#define billkwh_data_addrs      			0x0700                   //bill kwh data starting address
#define rms_avg_addrs           			0x07d0
#define cum_poff_addrs         				0x07e0                 // cum paower off data location or address 

/****** Storation Block 08 ***********************************************/
#define PowerDownTimeAdd        			0x0800
#define NextBillDateAdd         			0x0810
#define CurrentBillDataAdd      			0x0820
#define BILL_POff_minAdd        			0x0830
#define MD_IP_Add               			0x0840
#define FG_DateTimeAdd          			0x0850
#define kvah_zkvah_rollover_f_add 			0x0880
#define CurrentRating_addrs     			0x0890
#define DailyEnergyStatus       			0x08a0
#define DailyOffCount           			0x08b0 
#define DailyMDAdd              			0x08c0
#define DLMS_LLS_PassAdd        			0x08d0
#define DLMS_HLS_Pass1Add       			0x08e0
#define DLMS_HLS_Pass2Add       			0x08f0 

/****** Storation Block 09 *******************************************/
#define billtod_data_addrs      			0x0900                    

/****** Storation Block 0f *******************************************/
#define TOU_CAL_ACTIVE_ADD      			0x0f00
#define TOU_CAL_PASSIVE_ADD     			0x0f40
#define TOU_SEA_ACTIVE_STATUS   			0x0f80
#define TOU_PassiveApliDate     			0x0fa0
#define TOU_CheckPassiveApli    			0x0fb0
#define TOU_ActiveCalPtr        			0x0fc0

/****** Storation Block 10 *******************************************/
#define TOU_WEEK_ACTIVE_ADD     			0x1000

/****** Storation Block 11 *******************************************/
#define TOU_DAY_ACTIVE_ADD     	 			0x1100

/****** Storation Block 13 *******************************************/
#define lcd_Auto_Push_NO_Addr   			0x1300
#define lcd_Auto_Addr           			0x1310

/****** Storation Block 14 *******************************************/
#define lcd_Push_Addr           			0x1410

/****** Storation Block 15 *******************************************/
#define TamperSelByteAdd        			0x1510            
#define ThresholdAdd            			0x1520

/****** Storation Block 16 *******************************************/
#define load_survey_status      			0x1600
#define LS_IP_Add               			0x1610
#define DARRAY_ADD              			0X1620 

/****** Storation Block 18 *******************************************/
#define volt_init_add         				0x1800
#define volt_max_loc          				16
#define volt_max_add          				(volt_init_add+volt_max_loc*0x10)
#define curr_init_add         				volt_max_add
#define curr_max_loc          				16
#define curr_max_add          				(curr_init_add+curr_max_loc*0x10)//0x1300
#define pwr_init_add          				curr_max_add
#define pwr_max_loc           				20
#define pwr_max_add           				(pwr_init_add+pwr_max_loc*0x10)
#define trans_init_add        				pwr_max_add
#define trans_max_loc         				32
#define trans_max_add         				(trans_init_add+trans_max_loc*0x10)
#define other_init_add        				trans_max_add
#define other_max_loc         				50
#define other_max_add         				(other_init_add+other_max_loc*0x10)
#define control_init_add      				other_max_add
#define control_max_loc       				0
#define control_max_add       				(control_init_add+control_max_loc*0x10)
#define nonroll_init_add      				control_max_add
#define nonroll_max_loc       				1
#define nonroll_max_add       				(nonroll_init_add+nonroll_max_loc*0x10)
#define Diagnostics_init_add				nonroll_max_add
#define Diagnostics_max_loc     			6
#define Diagnostics_max_add     			(Diagnostics_init_add+Diagnostics_max_loc*0x10)

 
/****** Storation Block 36 *******************************************/
#define dloadsurvey_init_add    			0x3600
#define max_dloadsurvey         			0x0014 //1days
#define max_dloadsurvey_add     			(dloadsurvey_init_add+(max_dloadsurvey*2*0x10))//0x31a0

/****** Storation Block 40 *******************************************/
#define loadsurvey_init_add     			0x4000
#define max_loadsurvey          			0x0480 //0x0E40 = 76*48  


/***********************************************************************************************************************
Global functions
***********************************************************************************************************************/
extern union DE_union		DE;
extern union LS_union		LS;
extern union TP_union		TP;
extern union TP1_union		TP1;
extern union TP2_union		TP2;
extern union TP3_union		TP3;
extern union TP4_union		TP4;
extern union TP5_union		TP5;
extern union TP6_union		TP6;
extern union TP7_union		TP7;
extern union flag_union  	flag0;
extern union flag_union  	flag1;
extern union flag_union  	flag2;
extern union flag_union  	flag3;
extern union flag_union  	flag4;
extern union flag_union  	flag5;
extern union flag_union  	flag6;
extern union flag_union  	flag7;
extern union flag_union  	flag8;
extern union flag_union  	flag9;
extern union flag_union  	flag10;
extern union flag_union  	flag11;
extern union flag_union  	flag12;
extern union flag_union  	flag13;
//extern union flag_union  	flag14;
extern union flag_union  	flag15;
//extern union flag_union  	flag16;
//extern union flag_union  	flag17;
//extern union flag_union  	flag18;
//extern union flag_union  	flag19;
extern union flag_union  	flag21;
/****LCD****/
extern u8 DISP_u8UpdateCntr;
extern u8 DISP_u8PushSeqCntr;
extern u8 DISP_u8AutoSeqCntr;
extern u8 LCD_u8Max_PUSH;
extern u8 LCD_u8Max_AUTO;
extern u8 lcd_scroll_time;
extern u8 lcd_a8AutoList[];
extern u8 lcd_a8PushList[];
//extern u8 meter_id[4];
/****RTC****/
extern rtc_counter_value_t dt;
extern u8 rtc_status_byte;
extern u8 rtcfail_cntr;
extern u8 data_array[];
extern u8 RTC_u8ReadCounter;
extern u8 RTC_u8RtcRstCounter;
extern u8 u8RtcCalSign;
extern u8 u8RtcCalSec;
extern u8 rtc_correction_value;
extern u8 rtc_correction_state;
/****Function****/
extern union change32 chngtemp;
extern const u16 monthdays1[];
extern const u8 monthdays[];
extern u32 bat_on_secs;
/***PUSH*****/
extern u16 SW_u16unlock_cntr;
extern u16 SW_u16normal_scroll;
extern u8 SW_u8Test_menu_f;
extern u8 SW_u8Sw_f1;
extern u8 SW_u8Sw_f2;
extern u8 SW_u8Sw_f3;
extern u16 SW_u16cal_counter;
extern u16 SW_u16bat_cntr;
extern u16 SW_u16bkup_cntr;
extern u8 SW_u8bat_disp_c;
extern u8 SW_u8disp_cntr;
extern u8 SW_u8cal_bat_disp_c;
extern u8 scrol_cnt;
/****Meterology****/
extern u32 MET_u32Cum_kwh;
extern u32 MET_u32Cum_kvah;
extern u32 MET_u32dkvah;
extern u16 MET_u16kw_pulse_cntr;
extern u16 MET_u16kva_pulse_cntr;
extern u32 MET_u32kwh_pulse_cntr2;
extern u32 MET_u32kvah_pulse_cntr2;
extern u16 MET_u16Kva;
extern u16 MET_u16Kw;
extern u16 MET_u16ELKw;
extern u16 cum_diff;
extern u32 MET_u32Cum_MDKW;
extern u32 MET_u32Cum_MDKVA;
extern u32 MET_u32ip_rms;
extern u32 MET_u32in_rms;
extern double MET_u32ip_rms_Hires;
extern double MET_u32in_rms_Hires;
extern u16 MET_u16v_rms;
extern u16 MET_u16pf;
extern u16 MET_u16sign_net_pf;
extern u8 MET_u8pfsign;
extern u16 MET_u16Avg_pf;
extern u16 MET_u16freq;
extern u16 MET_u16freq1;
extern u16 MET_u16signed_Kw;
extern u16 MET_u16signed_ELKw;
//extern s16 MET_s16delay1_gain;
//extern s16 MET_s16delay2_gain;
//extern u16 MET_u16inverse1_gain;
//extern u16 MET_u16inverse2_gain;
extern u16 MET_u16StepsP1_gain;
extern u16 MET_u16StepsP2_gain;
extern u32 MET_u32v_cal_coff;
extern u32 MET_u32i1_cal_coff;
extern u32 MET_u32i2_cal_coff;
extern u32 MET_u32kw_cal;
extern u32 MET_u32kw_el_cal;
extern u8 MET_u8CircularPtr;
extern u8 MET_a8IpRawCnt[6];
extern u8 MET_a8VRawCnt[6];
extern u8 MET_a8InRawCnt[6];
extern u8 MET_a8IpRawCntNew[6];
extern u8 MET_a8VRawCntNew[6];
extern u8 MET_a8InRawCntNew[6];
extern u8 MET_a8PpRawCnt[6];
extern u8 MET_a8PpRawCntNew[6];
extern u8 MET_a8PnRawCnt[6];
extern u8 MET_a8PnRawCntNew[6];
extern u8 MET_u8clib_upf_status;
extern u8 MET_u8clib_lag_status;
extern u32 MET_u32kw_cnts;
extern u32 MET_u32delta_kw_cnts;
extern u32 MET_u32kva_cnts;
extern u32 MET_u32delta_kva_cnts;
extern u16 MET_u16Cntr;
extern u8 MET_u8led_blink;
//extern u32 MET_u32mcl_kw;
extern s32 MET_s32IpDC_acc;
extern s32 MET_s32InDC_acc;
extern s16 MET_s16IpDC_content;
extern s16 MET_s16InDC_content;
extern s16 MET_s16ip_new;
extern s16 MET_s16in_new;
extern double MET_dTemp;
extern u32 MET_u32kw_el_1s;
extern u32 MET_u32kwHires_el_1s;
extern u32 MET_u32kw_1s;
extern u32 MET_u32kwHires_1s;
extern u32 MET_u32kva_1s;
extern u8 MET_u8watt_100;
extern u8 MET_u8watt_kva_100;
extern u16 MET_u16sd16_cnts;
extern u16 MET_u16freq_cntr;
extern u8 MET_u8fzcd_cntr;
//extern u16 MET_u16i1_previous;
//extern u16 MET_u16i2_previous;
extern u16 MET_u16POS_MAX;
extern u16 MET_u16NEG_MAX;
extern u16 MET_u16NEG_MAX0;
extern u16 MET_u16POS_MAX_1s;
extern u16 MET_u16NEG_MAX_1s;
extern u16 MET_u16POS_MAX_C;
extern u16 MET_u16NEG_MAX_C;
extern u16 MET_u16NEG_MAX_C0;
extern u16 MET_u16POS_MAX_C_1s;
extern u16 MET_u16NEG_MAX_C_1s;
extern u8 MET_u8IpDC_acc_f;
extern u8 MET_u8InDC_acc_f;
extern u8 kvah_rollover_f;
extern u8 zkvah_rollover_f;
/****BILL****/
//extern u32 BILL_u32Last_kvah;
//extern u32 BILL_u32Last_kwh;
extern u8 BILL_u8md_count;
extern u8 BILL_u8mdmonth;
extern u8 TOD_u8todmonth;
extern u8 BILL_u8btpr_c;
extern u8 TOD_u8zone_index;
extern u32 TOD_u32cum_zkwh;
extern u32 TOD_u32cum_zkvah;
extern u8 BILL_a8date_array[];
extern u8 no_bills;
extern u32 bp_kwh;
extern u32 bp_kvah;
extern u32 delta_kwh;
extern u32 delta_kvah;
extern u8 TOD_u8vmain0;
extern u8 TOD_u8zmd_index;
extern u16 BILL_u16mdkw_c;
extern u16 BILL_u16mdkva_c;
extern u8 tou_a8pssv_zone_time[];
extern u8 tou_a8pssv_traiff[];
extern u8 tou_a8zone_time[];
extern u8 tou_a8traiff[];
extern u8 tariff_cnt;
extern u8 Week_Name[];
extern u8 u8ChangeCalnder_flag;
/****LoadSurvey****/
extern u16 LS_u16kwh;
extern u16 LS_u16kvah;
extern u16 d_array[MaxDarraySize];
extern u8 day_counter_ls;
extern u8 maxday_counter_l;
extern u8 day_counter;
extern u8 md_ip;
extern u8 md_ip_new;
extern u8 ls_ip;
extern u8 ls_ip_new;
extern u16 load_survey_cnt;
extern u8 var1;
extern u8 var2;
extern u8 var3;
extern u8 var4;
extern u8 var5;
extern u8 temp1_var;
extern u8 temp2_var;
extern u8 temp3_var;
extern u8 temp4_var;
extern u8 temp5_var;
extern u8 week1_set;
extern u8 time_array[6];
extern u16 cal_avg_cntr;
extern u32 i_rms_avg;
extern u32 v_rms_avg;
/****DailyEnergy****/
extern u8 daily_enrcount;
/****Power****/
extern u8 PWR_u8on_off_cnt;
extern u32 PWR_u32cum_poff_min;
extern u8 PWR_u8test_mode;
extern u16 PWR_u16poff_mins;
/****Tamper****/
extern u16 TPR_u16cum_tpr_c;
extern u8 TPRCNT_u8Mag;
extern u8 TPRCNT_u8NeuMiss;
extern u8 TPRCNT_u8NeuDis;
extern u8 TPRCNT_u8Rev;
extern u8 TPRCNT_u8EL;
extern u8 TPRCNT_u8OC;
extern u8 TPRCNT_u8FreqTamp;
extern u8 TPRCNT_u8VHigh;
extern u8 TPRCNT_u8VLow;
extern u8 TPRCNT_u8OverLoad;
extern u8 TPRCNT_u8TC;
extern u8 TPRCNT_u8LowPF;
extern u8 TPRCNT_u835kv;
extern u8 ls_date;
extern u8 ls_month;
extern u8 ls_year;
extern u32 TPR_u32cum_mag_defraud;
extern u32 TPR_u32cum_neutemp_defraud;
extern u16 TPR_u16magt_cnt_s;
extern u16 TPR_u16magt_cnt_r;
extern u16 TPR_u16rmagt_cnt;
extern u16 TPR_u16rev_per_c;
extern u16 TPR_u16rev_per_c1;
extern u16 TPR_u16el_per_c1;
extern u16 TPR_u16el_per_c;
extern u16 TPR_u16oc_per_c;
extern u16 TPR_u16oc_per_c1;
extern u16 TPR_u16ol_per_c;
extern u16 TPR_u16ol_per_c1;
extern u16 TPR_u16neu_per_c;
extern u16 TPR_u16neu_per_c1;
extern u16 TPR_u16neud_per_c;
extern u16 TPR_u16neud_per_c1;
extern u16 TPR_u16LowPF_per_c;
extern u16 TPR_u16LowPF_per_c1;
extern u16 TPR_u16AbFreq_per_c;
extern u16 TPR_u16AbFreq_per_c1;
extern u16 TPR_u16LowVolt_per_c;
extern u16 TPR_u16LowVolt_per_c1;
extern u16 TPR_u16HighVolt_per_c; 
extern u16 TPR_u16HighVolt_per_c1;
extern u8 TPR_u8dc_counter;
extern u8 TPR_u8freq_d_cntr;
extern u8 TPR_u8freq_d_cntr2;
extern u8 TPR_u8Neutamper_persist;
extern u8 TPR_u8Neutamper_persist1;
extern u16 TPR_u16tct_cnt;
extern u32 TamperSelByte;
extern u16 tpr_u16NeuMissStrTime;
extern u16 tpr_u16NeuMissRestrTime;
extern u16 tpr_u16NeuD_Thrshld;
extern u16 tpr_u16NeuD_ThrshldRest;
extern u16 tpr_u16OverV_Thrshld;
extern u16 tpr_u16OverV_ThrshldRest;
extern u16 tpr_u16OverVStrTime;
extern u16 tpr_u16OverVRestrTime;
extern u16 tpr_u16LowV_Thrshld;
extern u16 tpr_u16LowV_ThrshldRest;
extern u16 tpr_u16LowVStrTime;
extern u16 tpr_u16LowVRestrTime;
extern u32 tpr_u32OverC_Thrshld;
extern u32 tpr_u32OverC_ThrshldRest;
extern u16 tpr_u16OverCStrTime;
extern u16 tpr_u16OverCRestrTime;
extern u16 tpr_u16OverL_Thrshld;
extern u16 tpr_u16OverL_ThrshldRest;
extern u16 tpr_u16OverLStrTime;
extern u16 tpr_u16OverLRestrTime;
extern u16 tpr_u16addr;
extern u16 tpr_u16MagStrTime;
extern u16 tpr_u16MagRestrTime;
extern u16 tpr_u16NeuDStrTime;
extern u16 tpr_u16NeuDRestrTime;
extern u16 tpr_u16KV35RestrTime;
extern u16 tpr_u16RevStrTime;
extern u16 tpr_u16RevRestrTime;
extern u16 tpr_u16ELStrTime;
extern u16 tpr_u16ELRestrTime;
extern u16 tpr_u16LowPFRestrTime;
extern u16 tpr_u16LowPFStrTime;
extern u16 tpr_u16LowPF_ThrshldRest;
extern u16 tpr_u16LowPF_Thrshld;
extern u16 tpr_u16AbFreqRestrTime;
extern u16 tpr_u16AbFreqStrTime;
extern u16 tpr_u16AbFreq_Thrshldlow;
extern u16 tpr_u16AbFreq_Thrshldhigh;
extern u16 tpr_u16pwr_time_thrshld;
extern u32 tpr_u32hvcnt;
/****EEPROM****/
extern u8 opr_data[16];
//extern u8 EPR_u8Cnt;
//extern u8 EPR_u8Wait;
//extern u8 *EPR_p8data;
//extern union change32 EPR_uLocalAddr;
//extern u8 EPR_a8Address[];
/****DLMS****/
extern u32 hi_tempkwh;
extern u32 hi_tempkvah;
extern u8 trn_buf[];
extern u8 rcv_buf[];
extern u16 long_data;
extern u8 frm_rcv_flg;
extern u8 infose_flag;
extern u8 asserr_flag;
extern u8 assresult_flag;
extern u8 conf_err_flag;
extern u8 conf_type_flag;
//extern u8 rec_flag;
//extern u8 err_flag;
extern u8 seg_flag;
extern u8 decerr_flag;
extern u8 seg_flagsd;
extern u8 conf_ser_flag;
extern u16 max_info_rec;
extern u16 max_info_tra;
extern u16 k;
extern u16 req_cnt;
extern u16 trn_cnt;
extern u8 Data_block;
extern u8 cont_field;
extern u8 client_add;
extern u8 seg_type;
extern u8 req_typ;
extern u8 invo_prio;
extern u8 attribute_id;
extern u8 no_obj;
extern u8 from_ptr;
extern u8 to_ptr;
extern u16 rrr_c;
extern u16 rrr_s;
extern u16 rrr_c1;
extern u16 sss_c;
extern u16 sss_c1;
extern u16 sss_s;
extern u16 frame_type;
extern u8 server_add[];
extern u8 info[];
extern u8 dlms_x;
extern u8 dlms_y;
extern u16 rcv_cnt;
extern u16 rcv_cnt1;
extern u16 req_cnt1;
extern u8 rj_disc_cnt;
extern u8 length;
extern u16 p_fbit;
extern u8 rj_dm_buf[];
extern u8 info2[];
extern u8 rcv_buf1[];
extern u8 trn_cnt1;
extern u16 block_no;
extern u8 no_bytes;
extern u8 sel_obj_tamper[];
extern u8 to_val[];
extern u8 from_val[];
extern u8 conf_blk[];
extern u8 to_cntr_d;
extern u8 from_cntr_d;
extern u16 to_days;
extern u16 from_days;
extern u16 dls_count_dlms;
extern u8 access_selector;
extern u16 UintLoadSurptr1;
extern u8 sel_obj[];
extern u16 block_size;
extern u16 ls_count_dlms;
extern u8 to_cntr;
extern u8 from_cntr;
extern u16 const monthdays1[];
extern u16 UintLoadSurptr;
extern u16 ls_count_dlms;
extern u16 ls_count_local;
extern u8 obis_code[];
extern u8 ass_ser;
extern u8 aut_pswd1[16];
extern u8 aut_pswd12[16];
extern u8 aut_pswd1_2[16];
extern u8 aut_pswd1_1[16];
extern u16 byte_cont;
extern u8 max_win_rec;
extern u8 max_win_tra;
extern u8 aut_pswd[8];
extern u16 class_id;
extern u8 tou_u8pssv_dayid;
extern u8 tou_u8pssv_no_zone;
extern u8 tou_u8pssv_no_days;
extern u8 tou_u8pssv_buffer_traced;
extern u8 tou_u8pssv_ptr;
extern u8 tou_u8pssv_day;
extern u8 tou_u8pssv_up_zone;
extern u16 i_dlms;
extern u8 daily_on_off;
extern u8 trans_count;
extern u16 dls_count_local;
extern u8 volt_count;
extern u8 curr_count;
extern u8 on_off_cnt;
extern u8 on_off_loc;
extern u8 others_count;
extern u8 nonroll_count;
extern u8 Diagnostics_count;
extern u8 compart1;
extern u16 dlms_address;
extern u8 tamper_data;
extern u16 Cntr_2Min;
extern u16 event_id;
extern u16 cum_prog_count;
extern u16 info_sended;
extern u16 info_sended_old;
extern u16 info_send;
extern u16 info_total;
extern u16 packet_len;
extern u8 Format_type;
extern u8 multi_resp;
extern u16 element_filled;
extern u8 selective_values_byte;
extern u32 obis_short;
extern u16 serial_timeout;
extern u32 u32temp_CumMdkw;
extern u32 u32temp_CumMdkva;
//////LPM
extern u8 stop_mode;
extern u8 pd_counter;
extern u16 pd_counter2;
extern u16 TC_u16sleepcounter;
extern u8 zero_cross_counter;
//extern u8 test_mode;
extern u8 stopmode;
//extern u8 CLK_u8Correction_done;
extern u16 power_up_c;
extern u8 persis_c;
extern u8 persis_c1;
extern u8 neu_test_c;
//extern u16 bkup_cntr;
//////LPM
extern u8 TC_u8exitcounter;
extern u8 PON_u8SaveFlag;
extern u16 u16_bill_pow_on;
extern u32 cum_pow_on;
extern u16 pow_on;
extern u8 min_check;
extern u16 hv_persis_c;
extern u8 abnrm_c;
extern u8 test_mode;
//extern u8 tamper_persist;
extern u8 pon_ptr;
extern u8 u8WakeupIntruptCntr;
extern u8 charge_c1;
extern u8 discharge_c1;
//extern u8 charge_c;
//extern u8 discharge_c;
#endif
