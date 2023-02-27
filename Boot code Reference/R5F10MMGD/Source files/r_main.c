
/***********************************************************************************************************************
* File Name    : r_main.c
* Version      : CodeGenerator for RL78/L12 V2.00.00.07 [22 Feb 2013]
* Device(s)    : R5F10MMG
* Tool-Chain   : CC-RL
* Description  : This file implements main function.
* Creation Date: 7/2/2013
***********************************************************************************************************************/

/***********************************************************************************************************************
Pragma directive
***********************************************************************************************************************/

/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "r_cg_macrodriver.h"
#include "variable.h"
#include "lcd.h"
#include "timer.h"
#include "r_cg_wdt.h"
#include "ClockSetting.h"
#include "meterology.h"
#include "Eprom_i2c.h"
#include "LPM.h"
#include <stdlib.h>
#include "dlms.h"
#include "function.h"
#include "tamper.h"
#include "loadsurvey.h"
#include "bill.h"
#include "dailyenergy.h"
#include "Flash_Protocol.h"

/***********************************************************************************************************************
Global functions
***********************************************************************************************************************/
void R_MAIN_UserInit(void);
/***********************************************************************************************************************
Global variable
***********************************************************************************************************************/

#pragma section bss noinit 
////No init
unsigned int hv_persis_c ;
unsigned char abnrm_c;
unsigned char test_mode;
unsigned char min_check;
//unsigned char tamper_persist;
/****RTC****/
rtc_counter_value_t dt;
#pragma section

//ARRAY
/****Meterology****/
u8 MET_a8IpRawCnt[6];
u8 MET_a8VRawCnt[6];
u8 MET_a8InRawCnt[6];
u8 MET_a8IpRawCntNew[6];
u8 MET_a8VRawCntNew[6];
u8 MET_a8InRawCntNew[6];
u8 MET_a8PpRawCnt[6];
u8 MET_a8PpRawCntNew[6];
u8 MET_a8PnRawCnt[6];
u8 MET_a8PnRawCntNew[6];
/****Function****/
u8 opr_data[16];
union change32 chngtemp;
u32 bat_on_secs;
//u16 FR6;
//u16 FR7;
//u16 FR8;
//u16 FR9;
//u16 FR10;
//u16 FR11;
//u16 FR12;
//u16 FR13;
//u16 FR14;
//u16 FR15;
/****LCD****/
u8 lcd_a8AutoList[256];
u8 lcd_a8PushList[256];
//u8 meter_id[4];
/****RTC****/
u8 data_array[8];
/////u16 and u32
/***PUSH*****/
u16 SW_u16unlock_cntr;
u16 SW_u16normal_scroll;
u16 SW_u16cal_counter;
u16 SW_u16bat_cntr;
u16 SW_u16bkup_cntr;
/****Meterology****/
u32 MET_u32Cum_kwh;
u32 MET_u32dkvah;
u32 MET_u32Cum_kvah;
u16 MET_u16kw_pulse_cntr;
u16 MET_u16kva_pulse_cntr;
u32 MET_u32kwh_pulse_cntr2;
u32 MET_u32kvah_pulse_cntr2;
u16 MET_u16Kva;
u16 MET_u16Kw;
u16 MET_u16ELKw;
u32 MET_u32Cum_MDKW;
u32 MET_u32Cum_MDKVA;
u32 MET_u32ip_rms;
u32 MET_u32in_rms;
u16 MET_u16Avg_pf;
u16 MET_u16v_rms;
u16 MET_u16pf;
u16 MET_u16sign_net_pf;
u16 MET_u16freq;
u16 MET_u16freq1;
u16 MET_u16signed_Kw;
u16 MET_u16signed_ELKw;
//s16 MET_s16delay1_gain;
//s16 MET_s16delay2_gain;
//u16 MET_u16inverse1_gain;
//u16 MET_u16inverse2_gain;
u16 MET_u16StepsP1_gain;
u16 MET_u16StepsP2_gain;
u16 cum_diff;
u32 MET_u32v_cal_coff;
u32 MET_u32i1_cal_coff;
u32 MET_u32i2_cal_coff;
u32 MET_u32kw_cal;
u32 MET_u32kw_el_cal;
double MET_dTemp;
double MET_u32ip_rms_Hires;
double MET_u32in_rms_Hires;
u32 MET_u32kw_cnts;
u32 MET_u32kva_cnts;
u32 MET_u32delta_kw_cnts;
u32 MET_u32delta_kva_cnts;
u16 MET_u16Cntr;
//u32 MET_u32mcl_kw;
s32 MET_s32IpDC_acc;
s32 MET_s32InDC_acc;
s16 MET_s16IpDC_content;
s16 MET_s16InDC_content;
s16 MET_s16ip_new;
s16 MET_s16in_new;
u32 MET_u32kw_el_1s;
u32 MET_u32kwHires_el_1s;
u32 MET_u32kw_1s;
u32 MET_u32kwHires_1s;
u32 MET_u32kva_1s;
u16 MET_u16sd16_cnts;
u16 MET_u16freq_cntr;
//u16 MET_u16i1_previous;
//u16 MET_u16i2_previous;
/****BILL****/
//u32 BILL_u32Last_kvah;
//u32 BILL_u32Last_kwh;
u32 TOD_u32cum_zkwh;
u32 TOD_u32cum_zkvah;
u16 BILL_u16mdkw_c;
u16 BILL_u16mdkva_c;
u32 bp_kwh;
u32 bp_kvah;
u32 delta_kwh;
u32 delta_kvah;
u16 MET_u16POS_MAX;
u16 MET_u16NEG_MAX;
u16 MET_u16NEG_MAX0;
u16 MET_u16POS_MAX_1s;
u16 MET_u16NEG_MAX_1s;
u16 MET_u16POS_MAX_C;
u16 MET_u16NEG_MAX_C;
u16 MET_u16NEG_MAX_C0;
u16 MET_u16POS_MAX_C_1s;
u16 MET_u16NEG_MAX_C_1s;
u8 BILL_a8date_array[6];
u8 tou_a8pssv_zone_time[16];
u8 tou_a8pssv_traiff[8];
u8 tou_a8zone_time[16];
u8 tou_a8traiff[10];
u8 Week_Name[8];
/****LoadSurvey****/
u16 LS_u16kwh;
u16 LS_u16kvah;
u16 d_array[MaxDarraySize];
u16 load_survey_cnt;
u16 cal_avg_cntr;
u32 i_rms_avg;
u32 v_rms_avg;
u8 time_array[6];
/**** DLMS ***/
u32 hi_tempkwh;
u32 hi_tempkvah;
/****Power****/
u32 PWR_u32cum_poff_min;
u16 PWR_u16poff_mins;
/****Tamper****/
u32 TPR_u32cum_mag_defraud;
u32 TPR_u32cum_neutemp_defraud;
u16 TPR_u16cum_tpr_c;
u16 TPR_u16magt_cnt_s;
u16 TPR_u16magt_cnt_r;
u16 TPR_u16rmagt_cnt;
u16 TPR_u16rev_per_c;
u16 TPR_u16rev_per_c1;
u16 TPR_u16el_per_c1;
u16 TPR_u16el_per_c;
u16 TPR_u16oc_per_c;
u16 TPR_u16oc_per_c1;
u16 TPR_u16ol_per_c;
u16 TPR_u16ol_per_c1;
u16 TPR_u16neu_per_c;
u16 TPR_u16neu_per_c1;
u16 TPR_u16neud_per_c;
u16 TPR_u16neud_per_c1;
u16 TPR_u16LowPF_per_c;
u16 TPR_u16LowPF_per_c1;
u16 TPR_u16AbFreq_per_c;
u16 TPR_u16AbFreq_per_c1;
u16 TPR_u16LowVolt_per_c;
u16 TPR_u16LowVolt_per_c1;
u16 TPR_u16HighVolt_per_c; 
u16 TPR_u16HighVolt_per_c1;
u16 TPR_u16tct_cnt;

u32 TamperSelByte;
u16 tpr_u16NeuMissStrTime;
u16 tpr_u16NeuMissRestrTime;
u16 tpr_u16NeuD_Thrshld;
u16 tpr_u16NeuD_ThrshldRest;
u16 tpr_u16OverV_Thrshld;
u16 tpr_u16OverV_ThrshldRest;
u16 tpr_u16OverVStrTime;
u16 tpr_u16OverVRestrTime;
u16 tpr_u16LowV_Thrshld;
u16 tpr_u16LowV_ThrshldRest;
u16 tpr_u16LowVStrTime;
u16 tpr_u16LowVRestrTime;
u32 tpr_u32OverC_Thrshld;
u32 tpr_u32OverC_ThrshldRest;
u16 tpr_u16OverCStrTime;
u16 tpr_u16OverCRestrTime;
u16 tpr_u16OverL_Thrshld;
u16 tpr_u16OverL_ThrshldRest;
u16 tpr_u16OverLStrTime;
u16 tpr_u16OverLRestrTime;
u16 tpr_u16addr;
u16 tpr_u16MagStrTime;
u16 tpr_u16MagRestrTime;
u16 tpr_u16NeuDStrTime;
u16 tpr_u16NeuDRestrTime;
u16 tpr_u16KV35RestrTime;
u16 tpr_u16RevStrTime;
u16 tpr_u16RevRestrTime;
u16 tpr_u16ELStrTime;
u16 tpr_u16ELRestrTime;
u16 tpr_u16LowPFRestrTime;
u16 tpr_u16LowPFStrTime;
u16 tpr_u16LowPF_ThrshldRest;
u16 tpr_u16LowPF_Thrshld;
u16 tpr_u16AbFreqRestrTime;
u16 tpr_u16AbFreqStrTime;
u16 tpr_u16AbFreq_Thrshldlow;
u16 tpr_u16AbFreq_Thrshldhigh;
u16 tpr_u16pwr_time_thrshld;
u32 tpr_u32hvcnt;
u32 u32temp_CumMdkw;
u32 u32temp_CumMdkva;
/****EEPROM****/
u8 opr_data[16];
//u8 *EPR_p8data;
//union change32 EPR_uLocalAddr;
//u8 EPR_a8Address[2];
/****DLMS****/
u16 long_data;
u16 max_info_rec;
u16 max_info_tra;
u16 k;
u16 req_cnt;
u16 trn_cnt;
u16 rrr_c;
u16 rrr_s;
u16 rrr_c1;
u16 sss_c;
u16 sss_c1;
u16 sss_s;
u16 frame_type;
u16 rcv_cnt;
u16 rcv_cnt1;
u16 req_cnt1;
u16 p_fbit;
u16 block_no;
u16 to_days;
u16 from_days;
u16 dls_count_dlms;
u16 UintLoadSurptr1;
u16 block_size;
u16 ls_count_dlms;
u16 UintLoadSurptr;
u16 ls_count_dlms;
u16 ls_count_local;
u16 dls_count_local;
u16 byte_cont;
u16 class_id;
u16 i_dlms;
u16 dlms_address;
u16 Cntr_2Min;
u16 event_id;
u16 cum_prog_count;
u16 info_sended;
u16 info_sended_old;
u16 info_send;
u16 info_total;
u16 packet_len;
u16 element_filled;
u32 obis_short;
u16 serial_timeout;
//s32 s32temp0;
//s32 s32temp1;
//s32 s32temp2;
/****LPM****/
u16 power_up_c;
u16 u16_bill_pow_on;
u32 cum_pow_on;
u16 pow_on;
u16 pd_counter2;
u16 TC_u16sleepcounter;


/****LCD****/
u8 DISP_u8UpdateCntr;
u8 DISP_u8PushSeqCntr;
u8 DISP_u8AutoSeqCntr;
u8 LCD_u8Max_PUSH;
u8 LCD_u8Max_AUTO;
u8 lcd_scroll_time;
/****RTC****/
u8 rtc_correction_value;
u8 rtc_correction_state;
u8 rtc_status_byte;
u8 rtcfail_cntr;
u8 RTC_u8ReadCounter;
u8 RTC_u8RtcRstCounter;
u8 u8RtcCalSign;
u8 u8RtcCalSec;
/***PUSH*****/
u8 SW_u8Test_menu_f;
u8 SW_u8Sw_f1;
u8 SW_u8Sw_f2;
u8 SW_u8Sw_f3;
u8 SW_u8bat_disp_c;
u8 SW_u8disp_cntr;
u8 LCD_u8Max_AUTO;
u8 LCD_u8Max_PUSH;
u8 SW_u8cal_bat_disp_c;
/****Meterology****/
u8 MET_u8pfsign;
u8 MET_u8CircularPtr;//tcount1
u8 MET_u8clib_upf_status;
u8 MET_u8clib_lag_status;
u8 MET_u8led_blink;
u8 MET_u8watt_100;
u8 MET_u8watt_kva_100;
u8 MET_u8fzcd_cntr;
u8 MET_u8IpDC_acc_f;
u8 MET_u8InDC_acc_f;
u8 kvah_rollover_f;
u8 zkvah_rollover_f;
/****BILL****/
u8 BILL_u8md_count;
u8 BILL_u8mdmonth;
u8 TOD_u8todmonth;
u8 BILL_u8btpr_c;
u8 TOD_u8zone_index;
u8 TOD_u8vmain0;
u8 TOD_u8zmd_index;
u8 no_bills;
u8 tariff_cnt;
u8 u8ChangeCalnder_flag;
/****LoadSurvey****/
u8 day_counter_ls;
u8 maxday_counter_l;
u8 day_counter;
u8 md_ip;
u8 md_ip_new;
u8 ls_ip;
u8 ls_ip_new;
u8 var1;
u8 var2;
u8 var3;
u8 var4;
u8 var5;
u8 temp1_var;
u8 temp2_var;
u8 temp3_var;
u8 temp4_var;
u8 temp5_var;
u8 week1_set;
/****DailyEnergy****/
u8 daily_enrcount;
u8 daily_on_off;
/****Power****/
u8 PWR_u8on_off_cnt;
u8 PWR_u8test_mode;
/****Tamper****/
u8 TPRCNT_u8Mag;
u8 TPRCNT_u8NeuMiss;
u8 TPRCNT_u8NeuDis;
u8 TPRCNT_u8Rev;
u8 TPRCNT_u8EL;
u8 TPRCNT_u8OC;
u8 TPRCNT_u8FreqTamp;
u8 TPRCNT_u8VHigh;
u8 TPRCNT_u8VLow;
u8 TPRCNT_u8OverLoad;
u8 TPRCNT_u8TC;
u8 TPRCNT_u8LowPF;
u8 TPRCNT_u835kv;
u8 TPR_u8dc_counter;
u8 TPR_u8freq_d_cntr;
u8 TPR_u8freq_d_cntr2;
u8 TPR_u8Neutamper_persist;
u8 TPR_u8Neutamper_persist1;
/****EEPROM****/
//u8 EPR_u8Cnt;
//u8 EPR_u8Wait;
/****DLMS****/
u8 server_add[4];
u8 trn_buf[DLMS_MAX_BUFF_SIZE+14];
u8 rcv_buf[DLMS_MAX_BUFF_SIZE + 14];
u8 info[DLMS_MAX_BUFF_SIZE];
u8 rj_dm_buf[12];
u8 info2[550];
u8 rcv_buf1[150];
u8 sel_obj_tamper[6];
u8 to_val[6];
u8 from_val[6];
u8 conf_blk[4];
u8 sel_obj[10];
u8 obis_code[6];
u8 aut_pswd1[16];
u8 aut_pswd12[16];
u8 aut_pswd1_2[16];
u8 aut_pswd1_1[16];
u8 aut_pswd[8];
u8 frm_rcv_flg;
u8 infose_flag;
u8 asserr_flag;
u8 assresult_flag;
u8 conf_err_flag;
u8 conf_type_flag;
//u8 rec_flag;
//u8 err_flag;
u8 seg_flag;
u8 decerr_flag;
u8 seg_flagsd;
u8 conf_ser_flag;
u8 Data_block;
u8 cont_field;
u8 client_add;
u8 seg_type;
u8 req_typ;
u8 invo_prio;
u8 attribute_id;
u8 no_obj;
u8 from_ptr;
u8 to_ptr;
u8 dlms_x;
u8 dlms_y;
u8 rj_disc_cnt;
u8 length;
u8 trn_cnt1;
u8 no_bytes;
u8 to_cntr_d;
u8 from_cntr_d;
u8 access_selector;
u8 to_cntr;
u8 from_cntr;
u8 ass_ser;
u8 max_win_rec;
u8 max_win_tra;
u8 tou_u8pssv_dayid;
u8 tou_u8pssv_no_zone;
u8 tou_u8pssv_no_days;
u8 tou_u8pssv_buffer_traced;
u8 tou_u8pssv_ptr;
u8 tou_u8pssv_day;
u8 tou_u8pssv_up_zone;
u8 volt_count;
u8 curr_count;
u8 on_off_cnt;
u8 on_off_loc;
u8 trans_count;
u8 others_count;
u8 nonroll_count;
u8 Diagnostics_count;
u8 compart1;
u8 tamper_data;
u8 Format_type;
u8 multi_resp;
u8 selective_values_byte;
//////LPM
u8 stop_mode;
u8 pd_counter;

u8 zero_cross_counter;
//u8 test_mode;
u8 stopmode;
//u8 CLK_u8Correction_done;
u8 persis_c;
u8 persis_c1;
u8 neu_test_c;
u8 scrol_cnt;
//u16 bkup_cntr;
//////LPM
u8 TC_u8exitcounter;
u8 PON_u8SaveFlag;
u8 pon_ptr;
u8 u8WakeupIntruptCntr;
//u8 temp_variable,temp_variable1;
u8 charge_c1;
u8 discharge_c1;
u8 ls_date;
u8 ls_month;
u8 ls_year;
//u8 charge_c;
//u8 discharge_c;
//__far const u8 OptionByte[4] @ 0x00C0u = {0xFEu, 0xFFu, 0xE0u, 0x85u};

const u16 monthdays1[12]={0,31,59,90,120,151,181,212,243,273,304,334};
const u8 monthdays[12]={31,28,31,30,31,30,31,31,30,31,30,31};
//unsigned char change_flag;
union flag_union  flag0;
union flag_union  flag1;
union flag_union  flag2;
union flag_union  flag3;
union flag_union  flag4;
union flag_union  flag5;
union flag_union  flag6;
union flag_union  flag7;
union flag_union  flag8;
union flag_union  flag9;
union flag_union  flag10;
union flag_union  flag11;
union flag_union  flag12;
union flag_union  flag13;
//union flag_union  flag14;
union flag_union  flag15;
//union flag_union  flag16;
//union flag_union  flag17;
//union flag_union  flag18;
//union flag_union  flag19;
union flag_union  flag21;
union DE_union		DE;
union LS_union		LS;
union TP_union		TP;
union TP1_union		TP1;
union TP2_union		TP2;
union TP3_union		TP3;
union TP4_union		TP4;
union TP5_union		TP5;
union TP6_union		TP6;
union TP7_union		TP7;


/***********************************************************************************************************************
* Function Name: main
* Description  : This function implements main function.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
int main(void)
{
	//	u8 bogus1[]= {"Shishirchowdhary"};
	//	u16 bogus=0;
	uint8_t reset_flag = RESF;
	RPERDIS = 1;
	RTC_vInit();
	test_mode=0;
init:	
	ISCLCD = 0x02;
	PM12_bit.no5=1;
	if(P12_bit.no5 == 0)
	{
		Self_Programming_main();
	}
	__DI();
	////Init values start
	R_MAIN_UserInit();
	delay_1p3s();
	R_WDT_Restart();
	delay_1p3s();
	R_WDT_Restart();
	delay_1p3s();
	//	Div4832();
	R_WDT_Restart();
	EPR_vReset_Mem();
	R_WDT_Restart();
	MET_vInit();
	R_WDT_Restart();
	LCD_vLCDRamInit();
	init_dlmsvar(0);
	
	Eprom_Read(RTC_StatusByte_Add);
	if(EPROM_bChksum_ok == 1)
	{
		if(opr_data[0] != 0)
		{
			rtc_status_byte=1;
		}
	}
    
	R_WDT_Restart();
	TPR_vInitRam();
	LS_vInit();
	DE_vInit();  // call after the loadsurvey because daily energy count compared with max_day counter
	//	R_WDT_Restart();
	BILL_vInit();
	//	R_WDT_Restart();
	RTC_RamInit();
	time_stamp(&opr_data[0]);
	opr_data[5]=dt.sec;
	Eprom_Write(PowerDownTimeAdd);
	R_TAU0_Channel0_Start();
#if IR==1
	R_TAU0_Channel1_Start();
#endif
	UART_vStart();
	DSAMK = 0U;//sd24
	__EI();
	while (1U)
	{
		if(init_lcd_f==1 && TP6.b.sleep_f ==1)// PWR_bBatt_backup_f==1)
		{
			init_lcd_f=0;
			BattMode_init();
		}
		LCD_vUpdatePerSec();
		R_WDT_Restart();
		if(RTC_bReadFlag)
		{
			RTC_bReadFlag=0;
			RTC_vRead();
		}
		if(((TP6.b.tc_exit_f==1) &&((TC_u8exitcounter >= (tpr_u16TCStrTime/200)+1)||(TP5.b.tc_tpr_f==1)))
				||((TP5.b.tc_tpr_f==1)&&(TP5.b.tc_tprstopmode==1)&&(batt_disp_f==1)))
		{
			TP6.b.tc_exit_f=0;
			TC_u8exitcounter=0;
			TP5.b.tc_tprstopmode=0;
			RTC_bReadFlag=0;
			RTC_vRead();
			check_tpr();
			if(((TC_OPEN_Port & TC_OPEN_Bit)!=0) && (TP5.b.tc_tpr_f==0) )
			{				
				TP6.b.tc_exit_f=1;
				TP5.b.tc_tprstopmode=1;
			}			
			else if(PWR_bBatt_backup_f==0)
			{
				goto_stopmode();
			}
			else if(LCDM1==0x01)
			{
				BattMode_init();
			}
		}
		if(stopmode==1)
		{
			stopmode=0;
			MET_bsample_ready=0;
			MET_bcal_freq_f=0;
			DISP_bParCng=0;
			MET_u32ip_rms=0;
			MET_u32in_rms=0;
			MET_u16v_rms= 0;
			MET_u16ELKw = 0;
			MET_u16Kw = 0;
			MET_u16Kva =0;
			MET_u16freq = 0;
			MET_u16sign_net_pf = 0;
			MET_u16pf = 0;
			MET_u16signed_Kw = 0;
			time_stamp(&time_array[0]);
			stopmode_backup();
			goto_stopmode();
		}
		if(power_up_f1==1)
		{
			power_up_f1=0;
			R_WDT_Create();
			save_pulse_cntrs();
			save_pd();
			opr_data[0] = neu_test_c;
			Eprom_Write(NeutralTestCountAdd);
			goto init;
		}
		
		if(BattLowCheck==1)
		{
			BattLowCheck=0;
			BAT_vReadADC10();
			log_battery_status();
		}
		
		MET_vSampleReady();
		MET_vCal_freq();
		compute_energy();
//		if(analyse_pkt_flag == 1)
//			analyse_pkt();
		recv_frm();
		RTC_vCalib(); 

		if(PON_u8SaveFlag == 1)
		{
			PON_u8SaveFlag =0 ;
			pon_min_write();
		}
		if(seccng_f==1)
		{
			MET_vcheck_coffs();
			seccng_f=0;
			if(PWR_bBatt_backup_f==1)
			check_tpr();
			if(bill_f == 1)
			{
				bill_f=0;
				check_bill();
			}
		}
		if(mincng_f == 1)   //mincng_f //start
		{
			mincng_f = 0;
			BattLowCheck=1;
			deter_zone();
			TOD_u8zmd_index=TOD_u8vmain0;
			if(TOD_bzonecng_f0==1)
			{
				R_WDT_Restart();
				TOD_bzonecng_f0=0;
				storezkwh();
				TOD_vloadzkwh();
			}
			if(batt_disp_f==0)
			{
				if((LS.b.ls_rtc_fill)==1)
				{
					LS_vValueReset();
					ls_miss_fill();
					LS.b.ls_rtc_fill=0;
					load_ls_cnt();//in case of date change load_survey_cnt value will be corrected in
					//call of ls_cnt_at_pwrup (call in case of date change).
					//	LS.b.ls_rev_fill=0;
					
				}
				if(BILL_a8date_array[4]==dt.day && BILL_a8date_array[0]==dt.min && BILL_a8date_array[1]==dt.hour&& dt.day<=0x28)
				{
					check_bill();
				}
				if(((bcd_to_hex(dt.min)) % md_ip) == 0)
				mdfun();
			}
			if(daycng_f == 1)
			{
				daycng_f=0;
				TOU_vDeter_Season();
				if(batt_disp_f==0)
				{
					if(d_array[day_counter_ls]!=sel_datediff(dt.day,dt.month,dt.year) && LS.b.ls_rev_fill==0)//for correct recording of date at day change(00:00 hrs) in LS
					{
						day_counter_ls++;
						load_ls_cnt();
//						if(day_counter_ls > (maxday_counter_l-1))
//						day_counter_ls=0;
						Eprom_Read(load_survey_status);
						opr_data[10]= day_counter_ls;
						Eprom_Write(load_survey_status);
					}
					d_array[day_counter_ls]=sel_datediff(dt.day,dt.month,dt.year);
					fill_darray();
					
					if(DE.b.daily_rev_fill == 0)
					DE_vdaily_energy_save();
					DE.b.daily_rev_fill=0;
					LS.b.ls_rev_fill=0;
					load_ls_cnt();
				}
			}

			if(batt_disp_f==0)
			{
				if(((bcd_to_hex(dt.min)) % ls_ip) == 0)
				load_survey();
			}
		}//mincng_f //end
		if(TOD_bcalendar_change_f==1)
		{
			TOU_vCheck_Active_Calendar();
			if(TOD_bActive_f != TOD_bActive_Calendar)
			{
				TOU_vDeter_Season();
				TOD_bzonecng_f0=0;
//				storezkwh();
//				TOD_vloadzkwh();
			}
		}
		
		if(rtc_correction_f==1)
		{
			rtc_correction_f=0;
			rtc_calib_f=1;
			if(rtc_correction_state==0x01)
			{
				Eprom_Read(RTC_Calibration_add);
				rtc_correction_value = a8_to_u16(&opr_data[0]);
			}
			else if(rtc_correction_state==0x02)
			{
				rtc_correction_value=0x0020;
			}
			RTITIF=0;        			// clear interrupt flag
			RTITMK=0;        			// interrupt service enabled
		}
		
		if(batt_disp_f==0)
		{
			if((savebill_flag == 1 && dt.day!=0 && dt.month!=0 && dt.month<=0x12)||(mri_bill_f == 1))
			{
				savebill_flag =0;
				save_billdata();
				mri_bill_f=0;
//				PUS_u8Bill=1;
//				Eprom_Read(0x2990);
//				opr_data[1]=PUS_u8Bill;
//				Eprom_Write(0x2990);
			}
		}
		if(monthcng_f == 1)
		{
			monthcng_f = 0;
		}
	}
}

/***********************************************************************************************************************
* Function Name: R_MAIN_UserInit
* Description  : This function adds user code before implementing main function.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_MAIN_UserInit(void)
{
	u8 i_count;
	
	/**** Clock initialization ****/
	CLK_vClockSet();
	
	/**** Ram variable clearing ****/
	clear_ram();
	
	/**** RTC Initialization *****/
	RTCWEN = 1;
	RTCC0 |= BIT5;
	P13 &= ~BIT0;
	RTCMK = 1;
	RTCWEN = 0;
	
	/*** Watchdog restart *****/
	R_WDT_Create();
	
	/**** Eprom Initialization ****/
	EPR_vInit();
	
	/**** RTC Fail with after 5 try count */
	for(i_count=0;i_count<5;i_count++) 
	{
		RTC_vRead();
		if(rtcfail_cntr==0)
			break;
	}
	
	/**** Timer Initialization ****/
	TIM_vInit();
	
	/**** Zero Crossing detection Interrupt INTP0 ****/
	PIOR &= ~BIT4;
	EGP0 &= ~BIT0;//EGP0 = 0 and EGN0 = 1 falling edge
	EGN0 |= BIT0;
	PPR00=0;
	PPR10=0;
	PMK0 = 0;

	
	/**** TOP Cover open Detection only in sleep mode using interrrupt INTP7 ****/
	PM4 |= BIT2;
	EGP0 |= BIT7;//EGP7 = 1 and EGN7 = 0 rising edge
	EGN0 &= (u8)(~BIT7);
	PPR07 = 1;
	PPR17 = 1;
	PIF7 = 0;
	PMK7 = 1;

	/**** LCD Segments that are unused set to as a I/O ****/
	PFSEG3 &= 0xF0;
	PFSEG4 &= 0xc0;

	/**** Switch routine ****/
	PM12 |= BIT5;
	ISCLCD = 0x02;
	
	/*** LCD initialization ****/
	LCD_vInit(1);
	
	/*** switch to battery mode using interrupt INTP1 Only in sleep mode ****/
	EGP0 &= ~BIT1;//EGP1 = 0 and EGN1 = 1 falling edge
	EGN0 |= BIT1;
	PPR01 = 1;
	PPR11 = 1;
	PIF1 = 0;
	PMK1 = 1;	
	
	/****  cal led ***/
	PM6 &= 0xFB;
	P6 |= BIT2;

	/*** 10 bit ADC initialization ***/
	PM2 |= BIT0;
	ADPC = 0x01;
	
	/*** disabling battery ***/
	bat_disable;

	/*** switching to secondary battery in case Primary battery becomes low ****/
	R_VBATT_SetOperationOn();
	
	/**** magnet enable output ****/
	PM3 &= ~BIT1;
	P3 |= BIT1;
	PM3 |= BIT2;

	/*** Uart Initialization ****/
	UART_vInit();
	
	/*** delta sigma initialization ****/
	MET_vSD24Init();
}

/* Start user code for adding. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
