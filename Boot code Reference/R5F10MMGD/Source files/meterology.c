/***********************************************************************************************************************
* File Name    : meterology.c
* Version      : 
* Device(s)    : R5F10MMG
* Tool-Chain   : CC-RL
* Description  : This file implements meterology.
* Creation Date: 
***********************************************************************************************************************/


/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "r_cg_macrodriver.h"
#include "Variable.h"
#include "Eprom_i2c.h"
#include "function.h"
#include "meterology.h"
#include "lpm.h"
#include "lcd.h"
#include "bill.h"
#include "tamper.h"
#include "r_cg_wdt.h"
#include "dlms.h"
#include "timer.h"
#include <string.h>

/***********************************************************************************************************************
Pragma directive
***********************************************************************************************************************/
#pragma interrupt MET_iSD24_Interrupt(vect = INTDSAD) 

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/


/***********************************************************************************************************************
* Function Name: MET_vSD24Init
* Description  : This function initializes the clock generator.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void MET_vSD24Init(void)
{
	DSADMR = 0x0000;
#if EXT_X1 == 1	
	DSADCK = 1;//external 12MHz
#else
	DSADCK = 0;//internal 12MHz
#endif
	__nop();
	__nop();
	DSADCEN = 1;//enable module
	
#if SampleCounts == 1
	DSADMR |= 0x8000;//1953.125 Hz//comment select to 3906.25 Hz
#endif
	DSADMR |= 0x4000;//16bit
	
	DSADGCR0 = 0x00;//gain ch 0 = x1 , 1 = x1 
	DSADGCR1 = 0x04;//gain ch 2 = x16
	
	if(batt_disp_f==0)
	{
		DSADHPFCR = 0xc0;//high pass filter value =  4.857 Hz, not use on ch 0,1,2
	}
	else
	{
		DSADHPFCR = 0xc2;// Deactivate high pass filter in sleep mode to detect neutral disturbance
	}
	
	DSADPHCR0 = MET_u16StepsP2_gain;//0x8000;//select ch 0,1 Phase compensation step also
	DSADPHCR1 = MET_u16StepsP1_gain;//0x8000;//select ch 2 Phase compensation step also
	if(batt_disp_f==0)
	{
		DSADMR |= 0x0700;
		DSADMR |= 0x000f;
	}
	else
	{
		DSADMR |= 0x070f;
	}
	DSAPR0 = 0;
	DSAPR1 = 0;
	
	MET_bResetOn80Cnts=1;
	
}

static void __near MET_iSD24_Interrupt(void)
{
	DSAIF=0;
	MET_u16sd16_cnts++;
	
	meterology();
	
	if((s16)MET_u16POS_MAX<(s16)DSADCR1)
	{
		MET_u16POS_MAX = DSADCR1;
	}
		
	if((s16)MET_u16NEG_MAX0>(s16)DSADCR1)
	{
		MET_u16NEG_MAX0 = DSADCR1;
		MET_u16NEG_MAX = ~MET_u16NEG_MAX0;
		MET_u16NEG_MAX++;
	}
	
	if((s16)MET_u16POS_MAX_C<(s16)DSADCR0)
		MET_u16POS_MAX_C = DSADCR0;
		
	if((s16)MET_u16NEG_MAX_C0>(s16)DSADCR0)
	{
		MET_u16NEG_MAX_C0 = DSADCR0;
		MET_u16NEG_MAX_C = ~MET_u16NEG_MAX_C0;
		MET_u16NEG_MAX_C++;
	}
	
	if((DSADCR1 & 0x8000)&&(batt_disp_f==0)&&(test_mode==0))//(MET_bsleep_mode_f==0))
	{
		if(MET_bneg_vol_cnt_f==0)
		{
			MET_bneg_vol_cnt_f=1;
			MET_u8fzcd_cntr++;
			if(MET_u8fzcd_cntr==50)
			{
				MET_u8fzcd_cntr=0;
				MET_u8pfsign_cal_f = 1 ;
				MET_u16freq_cntr=MET_u16sd16_cnts;
				MET_u16sd16_cnts=0;
				MET_bcal_freq_f = 1;
			}
		}
	}
	else
	{
		MET_bneg_vol_cnt_f=0;
	}
	
	MET_u16Cntr++;

#if SampleCounts == 1	
	if((MET_u16Cntr==1953)||((MET_bsleep_mode_f==1)&&(MET_u16Cntr==244))||((MET_bResetOn80Cnts==1)&&(MET_u16Cntr==80)))
#else 
	if((MET_u16Cntr==3907)||((MET_bsleep_mode_f==1)&&(MET_u16Cntr==244))||((MET_bResetOn80Cnts==1)&&(MET_u16Cntr==80)))
#endif	
	{
		CopyBits();
		if(MET_bResetOn80Cnts==0)
		{
			check_MET_s32IpDC_acc();
			MET_s16IpDC_content = (s16)(MET_s32IpDC_acc/MET_u16Cntr);
			if(MET_u8IpDC_acc_f>=1)       
			{
				MET_u8IpDC_acc_f=0;
				INVERSEIpDCcon();
			}

			check_MET_s32InDC_acc();
			MET_s16InDC_content = (s16)(MET_s32InDC_acc/MET_u16Cntr);
			if(MET_u8InDC_acc_f>=1)
			{
				MET_u8InDC_acc_f=0;
				INVERSEInDCcon();
			}
		}
		MET_s32IpDC_acc=0;
		MET_s32InDC_acc=0;

		MET_u16POS_MAX_1s   = MET_u16POS_MAX;  
		MET_u16NEG_MAX_1s   = MET_u16NEG_MAX;
		MET_u16POS_MAX_C_1s = MET_u16POS_MAX_C;
		MET_u16NEG_MAX_C_1s = MET_u16NEG_MAX_C;
		MET_u16POS_MAX	=0;
		MET_u16NEG_MAX	=0;
		MET_u16POS_MAX_C=0;
		MET_u16NEG_MAX_C=0;		
		MET_u16NEG_MAX0	=0;
		MET_u16NEG_MAX_C0=0;
		if(MET_bResetOn80Cnts==0)
		{
			if(MET_bsleep_mode_f==0)
			{
				MET_bsample_ready=1;
			}
			else
			{
				MET_bten_kva_f=1;
			}
		}
		else
		{
			MET_bResetOn80Cnts=0;
			MET_u16POS_MAX_1s   =0;
			MET_u16NEG_MAX_1s   =0;
			MET_u16POS_MAX_C_1s =0;
			MET_u16NEG_MAX_C_1s =0;
		}
		MET_u16Cntr=0;
	}
}

u16 MET_vcompute_freq(void)
{
	if((batt_disp_f==1)||(test_mode==1))
	{
		return 0;
	}
		
	#if SampleCounts == 1
	return 97650000/MET_u16freq_cntr;//1953 count  (1953/2)*100000
	#else
	return 195300000/MET_u16freq_cntr;//3906 count
	#endif
	
}
void MET_vCal_freq(void)
{
	u16 delta1;
	if(MET_bcal_freq_f == 1)
	{
		MET_u16freq1=MET_u16freq;
		MET_u16freq = MET_vcompute_freq();
		MET_bcal_freq_f = 0;
		
		if((MET_u16freq1 >4000) && pow_up_f==0 && MET_u16v_rms > 10000)     //5sec delay at powerupchkpower_up_f1==0 ,voltage should be greater than 100v
		{
			if(MET_u16freq>MET_u16freq1)
			{
				delta1 = MET_u16freq-MET_u16freq1;
				cum_diff+=(MET_u16freq-MET_u16freq1);
			}
			else
			{
				delta1 = MET_u16freq1-MET_u16freq;
				cum_diff+=(MET_u16freq1-MET_u16freq);
			}
			if(delta1 > 500)
			{
				TPR_u8freq_d_cntr++;
				TPR_u8freq_d_cntr2=0;
			}
			else
			{
				TPR_u8freq_d_cntr2++;
				if(TPR_u8freq_d_cntr2==2)
				{
					TPR_u8freq_d_cntr2=0;
					TPR_u8freq_d_cntr=0;
					TP4.b.dcchop_f=0;
					cum_diff=0;
				}
			}
			if(TPR_u8freq_d_cntr==10)
			{
				TPR_u8freq_d_cntr=0;
				cum_diff=cum_diff/10;
				if(cum_diff>1000)          //1000
				{
					TP4.b.dcchop_f=1;
				}
				else
				{
					TP4.b.dcchop_f=0;
				}
				cum_diff=0;
			}
		}
		if((TamperSelByte & TprSel_AbFreq)==TprSel_AbFreq)
		{
			if(pow_up_f==0)
			{
				TPR_vAbFreq_Detection();
			}                
		}
	}
}	

void MET_vSampleReady(void)
{
	u32 u32temp;
	if(MET_bten_kva_f==1)
	{
		MET_bten_kva_f=0;
		DSADCEN = 0;
		DSADMR=0;
		DSAMK = 1;
		
		//compute voltage
//		chngtemp.u8_temp[0]=MET_a8VRawCntNew[0];
//		chngtemp.u8_temp[1]=MET_a8VRawCntNew[1];
//		chngtemp.u8_temp[2]=MET_a8VRawCntNew[2];
//		chngtemp.u8_temp[3]=MET_a8VRawCntNew[3];
		MET_dTemp = sqrt_function(MET_a8VRawCntNew);//sqrt(chngtemp.u32_temp);
		MET_dTemp = MET_dTemp * 4;  // as samples are taken 1/16 of 3906 samples so amultiplying factor sqrt(16) used
		MET_u16v_rms = (u16)((MET_dTemp*MET_u32v_cal_coff)/100000);
		
		//compute current Ip
//		chngtemp.u8_temp[0]=MET_a8IpRawCntNew[0];
//		chngtemp.u8_temp[1]=MET_a8IpRawCntNew[1];
//		chngtemp.u8_temp[2]=MET_a8IpRawCntNew[2];
//		chngtemp.u8_temp[3]=MET_a8IpRawCntNew[3];
		MET_dTemp = sqrt_function(MET_a8IpRawCntNew);
		MET_dTemp = MET_dTemp * 4; // as samples are taken 1/16 of 3906 samples so amultiplying factor sqrt(16) used
		MET_u32ip_rms = (u32)((MET_dTemp*MET_u32i1_cal_coff)/100000);
		
		//compute current In
//		chngtemp.u8_temp[0]=MET_a8InRawCntNew[0];
//		chngtemp.u8_temp[1]=MET_a8InRawCntNew[1];
//		chngtemp.u8_temp[2]=MET_a8InRawCntNew[2];
//		chngtemp.u8_temp[3]=MET_a8InRawCntNew[3];
		MET_dTemp = sqrt_function(MET_a8InRawCntNew);
		MET_dTemp = MET_dTemp * 4;  // as samples are taken 1/16 of 3906 samples so amultiplying factor sqrt(16) used
		MET_u32in_rms = (u32)((MET_dTemp*MET_u32i2_cal_coff)/100000);
		
		if(MET_u16v_rms<1000)
		{
			MET_u16v_rms=0;
		}
		
		if(MET_u32in_rms <NeuMissCurntThr && MET_u32ip_rms <NeuMissCurntThr)
		{
			MET_u32ip_rms =0;
			MET_u32in_rms=0;
			MET_u16ELKw=0;
			MET_u16Kw=0;
			MET_u16pf=0;
			TPR_u8Neutamper_persist=0;
			TPR_u8Neutamper_persist1 = 0;
		}
		else 
		{
			if(dt.min % 0x30 == 0)
			{
				deter_zone();
				TOD_u8zmd_index=TOD_u8vmain0;
				if(TOD_bzonecng_f0==1)
				{
					TOD_bzonecng_f0=0;
					storezkwh();
					TOD_vloadzkwh();
				}
			}
			
			MET_u16pf=1000;
			MET_vcheck_kwh_kvah();
			calculate_kwh();
			if(MET_u16v_rms < NeuDistVoltThr)
			{
				if(TP3.b.neu_miss_tpr_f==0)
				{
					MET_u16v_rms=0;
					TP4.b.neu_fea_f1 = 1;
					TPR_u8Neutamper_persist++;
					if(TPR_u8Neutamper_persist >=(tpr_u16NeuMissStrTime/60))
					{
						TPR_u8Neutamper_persist=0;
						TP3.b.neu_miss_tprstr_f=1;
						TP3.b.neu_miss_tpr_f=1;			
						others_count++;
						TP.b.OtherEvent=1;
						TPR_u16cum_tpr_c++;
						BILL_u8btpr_c++;
						RTC_vRead();
						check_tpr();
					}
				}
			}
			else
			{
				if(TP3.b.neu_dis_tpr_f==0)
				{
					TPR_u8Neutamper_persist1++;
					TP4.b.neu_fea_f1 = 1;
					if(TPR_u8Neutamper_persist1 >=(tpr_u16NeuDStrTime/60))
					{
						TPR_u8Neutamper_persist1=0;
						TP3.b.neu_dis_tpr_f=1;
						TP3.b.neu_dis_tprstr_f=1;
						TP.b.OtherEvent=1;
						others_count++;
						TPR_u16cum_tpr_c++;
						BILL_u8btpr_c++;
						RTC_vRead();
						check_tpr();
					}
				}
			}
		}
		
		ClearRawNew();
		MET_u16Cntr=0;

		if(PWR_bBatt_backup_f==0)
		{
			if(TP5.b.tc_tprstopmode==0)
			{
				goto_stopmode();
			}
		}
		else if(LCDM1==0x01)
		{
			BattMode_init();
		}
	}
	
	if(MET_bsample_ready==1)
	{
		MET_bsample_ready=0;
		//compute voltage
//		chngtemp.u8_temp[0]=MET_a8VRawCntNew[0];
//		chngtemp.u8_temp[1]=MET_a8VRawCntNew[1];
//		chngtemp.u8_temp[2]=MET_a8VRawCntNew[2];
//		chngtemp.u8_temp[3]=MET_a8VRawCntNew[3];
		MET_dTemp = sqrt_function(MET_a8VRawCntNew);//sqrt(chngtemp.u32_temp);
		MET_u16v_rms = (u16)((MET_dTemp*MET_u32v_cal_coff)/100000);
		if(MET_u16v_rms<5000)
		{
			MET_u16v_rms=0;
		}
		//compute current Ip
//		chngtemp.u8_temp[0]=MET_a8IpRawCntNew[0];
//		chngtemp.u8_temp[1]=MET_a8IpRawCntNew[1];
//		chngtemp.u8_temp[2]=MET_a8IpRawCntNew[2];
//		chngtemp.u8_temp[3]=MET_a8IpRawCntNew[3];
		MET_dTemp = sqrt_function(MET_a8IpRawCntNew);
		MET_u32ip_rms_Hires=(double)(MET_dTemp*MET_u32i1_cal_coff);
		MET_u32ip_rms = (u32)(MET_u32ip_rms_Hires/100000);
		
		if(MET_u32ip_rms<40)
		{
			Mask_curntreg_f = 1 ;
		}
		
		//compute current In
//		chngtemp.u8_temp[0]=MET_a8InRawCntNew[0];
//		chngtemp.u8_temp[1]=MET_a8InRawCntNew[1];
//		chngtemp.u8_temp[2]=MET_a8InRawCntNew[2];
//		chngtemp.u8_temp[3]=MET_a8InRawCntNew[3];
		MET_dTemp = sqrt_function(MET_a8InRawCntNew);
		MET_u32in_rms_Hires=(double)(MET_dTemp*MET_u32i2_cal_coff);
		MET_u32in_rms =(u32)(MET_u32in_rms_Hires/100000);
		
		////Active power
		if((TP1.b.mag_tpr_f==0)||(TP4.b.neu_fea_f1==1))
		{
			if(TP4.b.neu_fea_f1==0)
			{
				chngtemp.u8_temp[0]=MET_a8PpRawCntNew[0];
				chngtemp.u8_temp[1]=MET_a8PpRawCntNew[1];
				chngtemp.u8_temp[2]=MET_a8PpRawCntNew[2];
				chngtemp.u8_temp[3]=MET_a8PpRawCntNew[3];
				if((chngtemp.u8_temp[3]&0x80)==0x80)
				{
					u32temp = ~(chngtemp.u32_temp);
					u32temp += 1; 
					if(MET_u16Kw>ReverseDetectionThr)
					{
						MET_bkw_negative_f=1;
					}
					else
					{
						MET_bkw_negative_f=0;
					}
				}
				else
				{
					u32temp = (chngtemp.u32_temp);
					MET_bkw_negative_f=0;
				}
				MET_dTemp = u32temp;
				MET_dTemp = (double)(MET_dTemp*MET_u32kw_cal);
				MET_u32kwHires_1s = (u32)(MET_dTemp/100000);
				MET_dTemp = MET_dTemp/200000;
			}
			else 
			{
				MET_dTemp = MET_u32ip_rms*Vol_Rating_div100*5;//ip*24000/20
				MET_u32kwHires_1s = (u32)(MET_dTemp*2) ;
			}
			MET_u32kw_1s = (u32)(MET_dTemp);
			if(MET_u32kw_1s > CreepThrMainElemntInWatt*5000)//creep 1.8w*5000
			{
				MET_u16Kw = (u16)(MET_dTemp/5000);
			}
			else
			{
				MET_u32kw_1s=0;
				MET_u16Kw=0;
				MET_bkw_negative_f=0;
				MET_u32ip_rms=0;
				MET_u32ip_rms_Hires=0;
				if(MET_bph_ct_f1==0)
				{
					MET_bkw_el_negative_f=0;
					MET_u32kw_el_1s=0;
					MET_u32delta_kw_cnts=0;
					MET_u32kw_cnts=0;
				}
			}
			if(MET_bkw_negative_f)
			{
				MET_u16signed_Kw=(~MET_u16Kw);
				u32temp += 1;
				MET_u16signed_Kw+=1;
			}
			else
			{
			MET_u16signed_Kw=MET_u16Kw;
			}
		}
		////Active power EL
		if(TP5.b.dcinac_f==0)
		{
			if(TP4.b.neu_fea_f1==0)
			{
				chngtemp.u8_temp[0]=MET_a8PnRawCntNew[0];
				chngtemp.u8_temp[1]=MET_a8PnRawCntNew[1];
				chngtemp.u8_temp[2]=MET_a8PnRawCntNew[2];
				chngtemp.u8_temp[3]=MET_a8PnRawCntNew[3];
				if((chngtemp.u8_temp[3]&0x80)==0x80)
				{
					u32temp = ~(chngtemp.u32_temp);
					u32temp += 1; 
					if(MET_u16ELKw>ReverseDetectionThr)
					{
						MET_bkw_el_negative_f=1;
					}
					else
					{
						MET_bkw_el_negative_f=0;
					}
				}
				else
				{
					u32temp = (chngtemp.u32_temp);
					MET_bkw_el_negative_f=0;
				}
				MET_dTemp = u32temp;
				MET_dTemp = (double)(MET_dTemp*MET_u32kw_el_cal);
				MET_u32kwHires_el_1s = (u32)(MET_dTemp/100000);
				MET_dTemp = MET_dTemp/200000;
			}
			else
			{
				MET_dTemp = MET_u32in_rms*Vol_Rating_div100*5;//in*24000/20
				MET_u32kwHires_el_1s = (u32)(MET_dTemp*2);
			}
		}
		else
		{
			MET_dTemp = I_dcinac/2;
			MET_dTemp = MET_dTemp * MET_u16v_rms ;
		}
		MET_u32kw_el_1s = (u32)MET_dTemp;
		MET_u16ELKw = (u16)(MET_dTemp/5000);
		if(MET_u32kw_el_1s <= 11500)//creep 2.3w*5000
		{
			MET_bkw_el_negative_f=0;
		}
		
		if(MET_bkw_el_negative_f)
		{
			MET_u16signed_ELKw=(~MET_u16ELKw);
			MET_u16signed_ELKw+=1;
		}
		else
		{
			MET_u16signed_ELKw=MET_u16ELKw;
		}
		
		///detect earth load
		MET_bph_ct_f1=0;
		MET_bEl_glow=0;
		if((MET_u32kw_el_1s>(u32)(EarthLoadDetectionThr*(u32)5000))||(MET_u32kw_1s>(u32)(EarthLoadDetectionThr*(u32)5000)))///20w*5000
		{
			if(MET_u32kw_el_1s>MET_u32kw_1s)
			{
				u32temp = MET_u32kw_el_1s - MET_u32kw_1s;
				if(u32temp>(MET_u32kw_el_1s/EarthLoadDiffFactor))       // for diff 6.25%
				{
					MET_bph_ct_f1=1;
					MET_bEl_glow=1;	
				}
			}
			else
			{
				u32temp = MET_u32kw_1s - MET_u32kw_el_1s;
				if(u32temp>(MET_u32kw_1s/EarthLoadDiffFactor))
				{
					MET_bph_ct_f1=0;
					MET_bEl_glow=1;	
				}
			}				
		}
		if((TP1.b.mag_tpr_f==1)&&(TP4.b.neu_fea_f1==0))
		{
			MET_bph_ct_f1=0;
			MET_bEl_glow=0;
		}		

		if((TP1.b.mag_tpr_f==1)&&(TP4.b.neu_fea_f1==0)&&(test_mode==0))
		{
			MET_bph_ct_f1=0;
			MET_u32delta_kw_cnts = (u32)(Vol_Rating_div100*I_MAX_Rating_div1000)*(u32)5000;
			MET_u32delta_kva_cnts = (u32)(Vol_Rating_div100*I_MAX_Rating_div1000)*(u32)5000;
			MET_u16v_rms = Vol_Rating;
			MET_u32ip_rms = I_MAX_Rating;
			MET_bkw_negative_f=0;
			MET_bkw_el_negative_f=0;
			MET_u16Kw = (Vol_Rating_div100*I_MAX_Rating_div1000);
			MET_u32kwHires_1s = (u32)(Vol_Rating_div100*I_MAX_Rating_div1000)*(u32)10000;    // hiresolution active power 
			MET_u16signed_Kw = (Vol_Rating_div100*I_MAX_Rating_div1000);
			MET_u16Kva= (Vol_Rating_div100*I_MAX_Rating_div1000);
			MET_u32kva_1s = (u32)(Vol_Rating_div100*I_MAX_Rating_div1000)*(u32)10000;      // hiresolution apparent power
			MET_u16pf = 1000;
		}
		else
		{
			cal_app_power_vi();
			
			if(MET_u16Kva<MET_u16Kw)
			{
				MET_u16Kva=MET_u16Kw;
			}
			
			if(MET_u16v_rms>30000 && MET_u32ip_rms>250 && MET_u32in_rms>250)
            {
              MET_u32kw_1s = (unsigned long int)(Vol_Rating_div100*(MET_u32ip_rms+MET_u32in_rms));//*pf))/1000;
              MET_u32kva_1s = (unsigned long int)(Vol_Rating_div100*(MET_u32ip_rms+MET_u32in_rms));
              MET_u16Kw = (u16)(MET_u32kw_1s/1000);
              MET_u16Kva = (u16)(MET_u32kva_1s/1000);
			  MET_u32delta_kw_cnts = (u32)(MET_u32kw_1s*5);
			  MET_u32delta_kva_cnts = (u32)(MET_u32kva_1s*5);
            }
			else
			{
				///select cal source ip/in
				if(MET_bph_ct_f1)
				{
				MET_u32delta_kw_cnts = MET_u32kw_el_1s;
				}
				else
				{
				MET_u32delta_kw_cnts = MET_u32kw_1s;
				}
				MET_u32delta_kva_cnts = MET_u32kva_1s/2;
			}
			

			if(MET_u16Kva < 2)
			{
				MET_u32delta_kva_cnts=0;
				MET_u16Kva=0;
				MET_u16pf=0;
			}
			else
			{
				cal_pf();
			}
		}
		pfsigncal();
		
		MET_bpf_sign=MET_u8pfsign;
		if((MET_bph_ct_f1==0 && MET_u32ip_rms<10) || (MET_bph_ct_f1==1 && MET_u32in_rms<10))
		{
			MET_u16pf=0;
			MET_bpf_sign=0;
		}
		
		if(MET_bpf_sign==0)
		{
			MET_u16sign_net_pf=MET_u16pf;
		}
		else
		{
			MET_u16sign_net_pf=~(MET_u16pf);
			MET_u16sign_net_pf+=1;
		}
		
		cal_avg_cntr++;
		v_rms_avg += MET_u16v_rms;
		if(MET_bph_ct_f1==1)
		{
		i_rms_avg += MET_u32in_rms;
		}
		else
		{
		i_rms_avg += MET_u32ip_rms;
		}
		
		check_neu_feature();
		
		if((TamperSelByte & TprSel_VoltageHigh)==TprSel_VoltageHigh)
		{
			TPR_vVoltageOver_Detection();
		}
		if((TamperSelByte & TprSel_CurrentHigh)==TprSel_CurrentHigh)
		{
			TPR_vCurrentOver_Detection();
		}
		if((TamperSelByte & TprSel_OverLoad)==TprSel_OverLoad)
		{
			TPR_vCurrentOLoad_Detection();
		}
		if((TamperSelByte & TprSel_Reverse)==TprSel_Reverse)
		{
			TPR_vCurrentRev_Detection();
		}
		if((TamperSelByte & TprSel_EL)==TprSel_EL)
		{
			TPR_vCurrentEL_Detection();
		}
		TPR_vNeutralMissDisturbanceDetection();
		if((TamperSelByte & TprSel_LowPF)==TprSel_LowPF)
		{
			TPR_vLowPF_Detection();
		}
		if((TamperSelByte & TprSel_VoltageLow)==TprSel_VoltageLow)
		{
			TPR_vVoltageLow_Detection();
		}
		
		if(MET_bcalib_flag==1 && cal_switch_f == 1)
		{
			cal_switch_f=0;
			if(MET_blag_calib_f==0)
			{
				calibration();
			}
			else
			{
				lag_calibration();
			}
		}
		check_tpr();
	}	
}

void cal_app_power_vi(void)
{
	if(TP4.b.neu_fea_f1==1)
	{
		MET_dTemp = Vol_Rating;
	}
	else
	{
		MET_dTemp = MET_u16v_rms;
	}
	
	if(MET_bph_ct_f1==1)
	{
		MET_dTemp = MET_dTemp * MET_u32in_rms_Hires;
	}
	else
	{
		MET_dTemp = MET_dTemp * MET_u32ip_rms_Hires;
	}
	
	MET_dTemp = MET_dTemp/1000000;
	MET_u32kva_1s = (u32)MET_dTemp;
	
	if((MET_bph_ct_f1==0)&&(MET_u32kva_1s<MET_u32kw_1s))
	{
		MET_u32kva_1s=MET_u32kw_1s;
	}
	else if((MET_bph_ct_f1==1)&&(MET_u32kva_1s<MET_u32kw_el_1s))
	{
		MET_u32kva_1s=MET_u32kw_el_1s;
	}
	MET_u16Kva = (u16)(MET_dTemp/10000);
}

void cal_pf(void)
{
	if(MET_bph_ct_f1==1)
	{
		MET_dTemp = MET_u16ELKw;
	}
	else
	{
		MET_dTemp = MET_u16Kw;
	}
	MET_dTemp = MET_dTemp * 1000;
	MET_u16pf = (u16)(MET_dTemp/MET_u16Kva);
	if(MET_u16pf>999)
	{
		MET_u16pf = 999;
	}
	
}
void pfsigncal(void)
{
	if(MET_u16pf >=990 && MET_u16pf<=10)
	{
	MET_u8pfsign=0;
	}


	if((MET_bkw_negative_f == 1) || (MET_bkw_el_negative_f == 1))
	{
		if(MET_u16pf < 950)
		{
			if(MET_u8pfsign == 1)
			{
				MET_u8pfsign =0;
			}
			else if(MET_u8pfsign == 0)
			{
				MET_u8pfsign =1;
			}
		}
		else
		{
			MET_u8pfsign =0;
		}
	}

	if((MET_u32ip_rms==0 && MET_u32in_rms == 0) || TP4.b.neu_fea_f1==1 || /*test_mode==1 ||*/ MET_u16pf>=990)
	{
		MET_u8pfsign=0;
	}
}

void MET_vInit(void)
{
	u8 u8LocalPtr;
	u8 u8LocalLoop;
	u32 u32long_int;
	u32 u32dupli;
	pow_up_f=1;

//	fill_oprzero();
//	Eprom_Write(LagCalibrationCoffadd);
//	Eprom_Write(UpfCalibrationCoffadd);
	
	Eprom_Read(LagCalibrationCoffadd);
	R_WDT_Restart();
	if(EPROM_bChksum_ok == 1)
	{
		MET_u16StepsP1_gain = a8_to_u16(&opr_data[0]);   
		MET_u16StepsP2_gain = a8_to_u16(&opr_data[2]);  
		MET_u8clib_upf_status=opr_data[10];
		MET_u8clib_lag_status=opr_data[11];
		DSADPHCR0 = MET_u16StepsP2_gain;
		DSADPHCR1 = MET_u16StepsP1_gain;
	}
	else
	{
		Eprom_Read(Dupli_LagCalibrationCoffadd);
		if(EPROM_bChksum_ok == 1)
		{
		Eprom_Write(LagCalibrationCoffadd);
		
		MET_u16StepsP1_gain = a8_to_u16(&opr_data[0]);   
		MET_u16StepsP2_gain = a8_to_u16(&opr_data[2]);  
		MET_u8clib_upf_status=opr_data[10];
		MET_u8clib_lag_status=opr_data[11];
		DSADPHCR0 = MET_u16StepsP2_gain;
		DSADPHCR1 = MET_u16StepsP1_gain;
		}
		
	}
	
	Eprom_Read(UpfCalibrationCoffadd);
	if(EPROM_bChksum_ok ==1)
	{
		MET_u32v_cal_coff = a8_to_u24(&opr_data[0]);
		MET_u32i1_cal_coff = a8_to_u24(&opr_data[3]);
		MET_u32kw_cal = a8_to_u24(&opr_data[6]);
		MET_u32i2_cal_coff = a8_to_u24(&opr_data[9]);
		MET_u32kw_el_cal = a8_to_u24(&opr_data[12]);
	}
	else
	{
		Eprom_Read(Dupli_UpfCalibrationCoffadd);
		if(EPROM_bChksum_ok ==1)
		{
		Eprom_Write(UpfCalibrationCoffadd);
		
		MET_u32v_cal_coff = a8_to_u24(&opr_data[0]);
		MET_u32i1_cal_coff = a8_to_u24(&opr_data[3]);
		MET_u32kw_cal = a8_to_u24(&opr_data[6]);
		MET_u32i2_cal_coff = a8_to_u24(&opr_data[9]);
		MET_u32kw_el_cal = a8_to_u24(&opr_data[12]);
		}
	}
	
	if(MET_u8clib_upf_status>3)
	{
		MET_u8clib_upf_status=0;
	}
	
	if(MET_u8clib_lag_status>3)
	{
		MET_u8clib_lag_status=0;
	}
	
	MET_vcheck_coffs();
	
/*--------- Load kwh from eeprom on power-up -------------------------------------------------------------------
	* read eeprom for maxima of kwh and get zone energy and zone index
	* get maxima and transfer to kwh for further counting read from block 0
	
	** NOTE HERE INITIALLY ALL KWH VARIABLES ARE ZERO total 8 blocks are used f000-f7f0   && f800 - fff0
---------------------------------------------------------------------------------------------------------------*/
	Eprom_Read(kvah_zkvah_rollover_f_add);
	if(opr_data[0]==1)
	{
		kvah_rollover_f=1;
	}
	if(opr_data[1]==1)
	{
		zkvah_rollover_f=1;
	}
	u8LocalPtr=0;
	TOD_u8zone_index = 1;
	for(u8LocalLoop=0;u8LocalLoop<16;u8LocalLoop++)            // since 8 pages are used for energy write
	{
		Eprom_Read(CircularBufferEnergy+MET_u8CircularPtr);
		if(EPROM_bChksum_ok ==1)
		{
			u32dupli=a8_to_u24(&opr_data[3]);
			u32long_int=a8_to_u24(&opr_data[0]);
			if((MET_u32Cum_kvah < u32long_int ||((MET_u32Cum_kvah == u32long_int)&& MET_u32Cum_kwh<a8_to_u24(&opr_data[6]))) && u32long_int==u32dupli)
			{
				MET_u32Cum_kvah = u32long_int;
				MET_u32Cum_kwh = a8_to_u24(&opr_data[6]);
				TOD_u32cum_zkwh = a8_to_u24(&opr_data[9]);
				TOD_u32cum_zkvah = a8_to_u24(&opr_data[12]);
				MET_u32dkvah = MET_u32Cum_kvah;
				u8LocalPtr = MET_u8CircularPtr;
				Eprom_Read(CircularBufferMDLSkwh+MET_u8CircularPtr);
				TOD_u8zone_index = opr_data[0];
			}
		}
		MET_u8CircularPtr += 0x10;
		R_WDT_Restart();
	}
	MET_u8CircularPtr = u8LocalPtr + 0x10;
	
	Eprom_Read(kw_pulse_c_addrs);    //reading puls cntr at pwr up
	if(EPROM_bChksum_ok ==1)
	{
		MET_u16kw_pulse_cntr = a8_to_u16(&opr_data[0]);
		MET_u16kva_pulse_cntr = a8_to_u16(&opr_data[2]);
		TPR_u32cum_mag_defraud = a8_to_u24(&opr_data[4]);
		TPR_u32cum_neutemp_defraud=a8_to_u24(&opr_data[7]);
		
	}
	MET_vcheck_kwh_kvah();
	
}
void calibration(void)
{
	if((TP4.b.neu_fea_f1==1)||(MET_u16Kw > 2700)||(MET_u16Kw < 2100) ||(MET_u16ELKw > 2700)||(MET_u16ELKw < 2100))
	{
		MET_bcal_done_f=0;
		MET_bcal_mode_f=0;
//		MET_bcalib_flag=0;
		return;
	}
	else
	{
		MET_u32v_cal_coff= ((unsigned long)24000 * MET_u32v_cal_coff)/(unsigned long)MET_u16v_rms;
		
		MET_u32i1_cal_coff=((unsigned long)10000 * MET_u32i1_cal_coff)/MET_u32ip_rms;
		
		MET_u32kw_cal=((unsigned long)2400 * MET_u32kw_cal)/(unsigned long)MET_u16Kw;
		MET_u16StepsP1_gain=0;
		DSADPHCR1 = 0;
		MET_u32i2_cal_coff=((unsigned long)10000 * MET_u32i2_cal_coff)/MET_u32in_rms;
		
		MET_u32kw_el_cal=((unsigned long)2400 * MET_u32kw_el_cal)/(unsigned long)MET_u16ELKw;
		MET_u16StepsP2_gain=0;
		DSADPHCR0 = 0;
	}  
	
	fill_oprzero();
	
	FUN_vfill_3byteR(MET_u32v_cal_coff,&opr_data[2]);
	FUN_vfill_3byteR(MET_u32i1_cal_coff,&opr_data[5]);
	FUN_vfill_3byteR(MET_u32kw_cal,&opr_data[8]);
	FUN_vfill_3byteR(MET_u32i2_cal_coff,&opr_data[11]);
	FUN_vfill_3byteR(MET_u32kw_el_cal,&opr_data[14]);
	
	Eprom_Write(UpfCalibrationCoffadd);
	Eprom_Write(Dupli_UpfCalibrationCoffadd);
	fill_oprzero();

	MET_u8clib_upf_status=0x03;
	opr_data[10]=0x03;
	Eprom_Write(LagCalibrationCoffadd); 
	Eprom_Write(Dupli_LagCalibrationCoffadd);

	MET_bcal_mode_f=0;
	MET_bcal_done_f=1;
}

void lag_calibration(void)
{
	signed int error,error1 ; //temp2,temp1;   
	unsigned int ref_act_power=1200;
	signed long s32Temp;
	u8 u8Sign=0;
	u8 u8Sign1=0;
	
	
	if((MET_u16Kw > 1200)&&(MET_u16Kw < 1300)&&(MET_u16ELKw > 1175)&&(MET_u16ELKw < 1300))
	{
		error = ref_act_power-MET_u16Kw;
		error1 = ref_act_power-MET_u16ELKw;  
		
		s32Temp = (signed long)(error *(signed long)2754);//10000);//1step = 
		error = (s16)(s32Temp/(signed long)ref_act_power);

		s32Temp = (signed long)(error1 *(signed long)2754);//10000);
		error1 = (s16)(s32Temp/(signed long)ref_act_power);
		
		if(lag_ok==0)
		{
			if(error < 0)
			{
				u8Sign = 1;
				error = ~(error)+1;
			}
			
			if(error1 < 0)
			{
				u8Sign1 = 1;
				error1 = ~(error1)+1;
			}
			
			if(u8Sign==1)
			{
				MET_u16StepsP1_gain = error;
				MET_u16StepsP1_gain |= 0x8000 ;
				DSADPHCR1 = MET_u16StepsP1_gain;

				MET_u16StepsP2_gain = error1;
				MET_u16StepsP2_gain |= 0x8000 * u8Sign1;
				DSADPHCR0 = MET_u16StepsP2_gain;

				Eprom_Read(LagCalibrationCoffadd);
				
				if(EPROM_bChksum_ok== 1)
				{
					FUN_vfill_2byteR(MET_u16StepsP1_gain,&opr_data[1]);
					FUN_vfill_2byteR(MET_u16StepsP2_gain,&opr_data[3]);
					FUN_vfill_2byteR(MET_u16Kw,&opr_data[5]);
					FUN_vfill_2byteR(MET_u16ELKw,&opr_data[7]);
					opr_data[11]=0x03;
					Eprom_Write(LagCalibrationCoffadd);
					Eprom_Write(Dupli_LagCalibrationCoffadd);
					MET_u8clib_lag_status=opr_data[11];
					MET_bcal_done_f=1;
					lag_ok=1;
				}
				else
				{
					Eprom_Read(Dupli_LagCalibrationCoffadd);
					if(EPROM_bChksum_ok == 1)
					{
						FUN_vfill_2byteR(MET_u16StepsP1_gain,&opr_data[1]);
						FUN_vfill_2byteR(MET_u16StepsP2_gain,&opr_data[3]);
						FUN_vfill_2byteR(MET_u16Kw,&opr_data[5]);
						FUN_vfill_2byteR(MET_u16ELKw,&opr_data[7]);
						opr_data[11]=0x03;
						Eprom_Write(LagCalibrationCoffadd);
						Eprom_Write(Dupli_LagCalibrationCoffadd);
						MET_u8clib_lag_status=opr_data[11];
						MET_bcal_done_f=1;
						lag_ok=1;
					}
				}
			}
			else
			{
				MET_bcal_done_f=0;
				lag_ok=0;
			}
			MET_bcal_mode_f=0;
			lag_cal_done=0;
		}
	}
}

void cal_avg_pf(void)
{
	u32 u32temp;
	u32 u32Delta_kwh;
	u32 u32Delta_kvah;
	if(MET_u32Cum_kwh < bp_kwh)
	{
		u32Delta_kwh = MET_u32Cum_kwh + KWh_RolloverValue-bp_kwh;
	}
	else
	{
		u32Delta_kwh = MET_u32Cum_kwh-bp_kwh;
	}

	if(MET_u32Cum_kvah < bp_kvah)
	{
		u32Delta_kvah = MET_u32Cum_kvah + KWh_RolloverValue-bp_kvah;
	}
	else
	{
		u32Delta_kvah = MET_u32Cum_kvah-bp_kvah;
	}

	u32temp=((unsigned long)(u32Delta_kwh*1000));
	u32temp=(u32temp/u32Delta_kvah);
	MET_u16Avg_pf = (unsigned int)(u32temp);

	if(MET_u16Avg_pf>1000)
	{
		MET_u16Avg_pf=1000;
	}
}
void MET_vcheck_kwh_kvah(void)
{
	if((MET_u32Cum_kvah<MET_u32Cum_kwh) && kvah_rollover_f==0)   
	{
		MET_u32Cum_kvah=MET_u32Cum_kwh;
		MET_u32dkvah = MET_u32Cum_kvah;
		if(MET_u16kva_pulse_cntr>MET_u16kw_pulse_cntr)
		{
			MET_u16kva_pulse_cntr=MET_u16kw_pulse_cntr;  
		}
		if((TOD_u32cum_zkvah<TOD_u32cum_zkwh) && zkvah_rollover_f==0)
		{
			TOD_u32cum_zkvah = TOD_u32cum_zkwh;
		}
	}
	else if(MET_u32Cum_kvah == MET_u32Cum_kwh)
	{
		if(MET_u16kva_pulse_cntr<MET_u16kw_pulse_cntr)
		{
			MET_u16kva_pulse_cntr = MET_u16kw_pulse_cntr;
		}
	} 
}

void MET_vcheck_coffs(void)
{
	#if SampleCounts == 1
	if((MET_u32v_cal_coff == 0) || (MET_u32v_cal_coff == 0xffffff))
	{
		MET_u32v_cal_coff = 5000;//33696;//2243;////0x00001B04;//0x00001aeb;
	}
	if((MET_u32i1_cal_coff == 0) || (MET_u32i1_cal_coff == 0xffffff))
	{
		MET_u32i1_cal_coff = 7000;//34952;//315;//0x000035AA;
	}
	if((MET_u32i2_cal_coff == 0) || (MET_u32i2_cal_coff == 0xffffff))
	{
		MET_u32i2_cal_coff = 7000;//32352;//323;//0x000043F9;
	}
	if((MET_u32kw_cal == 0) || (MET_u32kw_cal == 0xffffff))
	{
		MET_u32kw_cal  = 70000;//240383;//2159;//0x000171DD;
	}
	if((MET_u32kw_el_cal == 0) || (MET_u32kw_el_cal == 0xffffff))
	{
		MET_u32kw_el_cal = 70000;//222532;//2217;//0x0001D46B;
	}
	#else
	if((MET_u32v_cal_coff == 0) || (MET_u32v_cal_coff == 0xffffff))
	{
		MET_u32v_cal_coff = 3384;//15970;//1592;////0x00001B04;//0x00001aeb;
	}
	if((MET_u32i1_cal_coff == 0) || (MET_u32i1_cal_coff == 0xffffff))
	{
		MET_u32i1_cal_coff = 5130;//23423;//230;//0x000035AA;
	}
	if((MET_u32i2_cal_coff == 0) || (MET_u32i2_cal_coff == 0xffffff))
	{
		MET_u32i2_cal_coff = 5025;//22999;//229;//0x000043F9;
	}
	if((MET_u32kw_cal == 0) || (MET_u32kw_cal == 0xffffff))
	{
		MET_u32kw_cal  = 35620;//114423;//1140;//0x000171DD;
	}
	if((MET_u32kw_el_cal == 0) || (MET_u32kw_el_cal == 0xffffff))
	{
		MET_u32kw_el_cal = 34960;//111111;//1116;//0x0001D46B;
	}
	#endif
}

void calculate_kwh(void)
{
	unsigned long int temp;

//	if(mag_tpr_f ==1)
//		temp=((unsigned long)240 *30000)/6;                         //6 = 1 minute and 24 = 15 seconds wakeup time
//	else
	{
		if(MET_u32ip_rms > MET_u32in_rms)
		{
			MET_bph_ct_f1=0;
			if(MET_u32in_rms>700)
			{
				temp=((unsigned long)Vol_Rating_div100 *(MET_u32ip_rms+MET_u32in_rms))/6;  //240*.866                         //6 = 1 minute and 24 = 15 seconds wakeup time				
			}
			else
			{
				temp=((unsigned long)Vol_Rating_div100 *MET_u32ip_rms)/6;  //240*.866                         //6 = 1 minute and 24 = 15 seconds wakeup time
			}
		}
		else
		{
			MET_bph_ct_f1=1;
			if(MET_u32ip_rms>700)
			{
				temp=((unsigned long)Vol_Rating_div100 *(MET_u32in_rms+MET_u32ip_rms))/6;    //6 = 1 minute and 24 = 15 seconds wakeup time
			}
			else
			{
				temp=((unsigned long)Vol_Rating_div100 *MET_u32in_rms)/6;   //6 = 1 minute and 24 = 15 seconds wakeup time
			}
		}
	}
	
	MET_u32kwh_pulse_cntr2=MET_u32kwh_pulse_cntr2+temp;
	
	
	while(MET_u32kwh_pulse_cntr2 >= 1000000)
	{
		MET_u32kwh_pulse_cntr2=MET_u32kwh_pulse_cntr2-1000000;
		write_energy();
	}
	while(MET_u32kwh_pulse_cntr2 >= 3125)
	{
		MET_u32kwh_pulse_cntr2=MET_u32kwh_pulse_cntr2-3125;
		MET_u16kw_pulse_cntr++;
		if(MET_u16kw_pulse_cntr >=320)
		{
			MET_u16kw_pulse_cntr=0;
			write_energy();
		}
	}
	
	MET_u32kvah_pulse_cntr2=MET_u32kvah_pulse_cntr2+temp;
	
	while(MET_u32kvah_pulse_cntr2 >= 1000000)
	{
		MET_u32kvah_pulse_cntr2=MET_u32kvah_pulse_cntr2-1000000;
		write_kvaenergy();
	}
	while(MET_u32kvah_pulse_cntr2 >= 3125)
	{
		MET_u32kvah_pulse_cntr2=MET_u32kvah_pulse_cntr2-3125;
		MET_u16kva_pulse_cntr++;
		if(MET_u16kva_pulse_cntr >=320)
		{
			MET_u16kva_pulse_cntr=0;
			write_kvaenergy();
		}
	}
}
void compute_energy(void)
{
	if(ram_crpt_f0==0)
	{
		if(MET_bten_wh_f==1)
		{
			MET_bten_wh_f =0;
			BILL_u16mdkw_c +=10;
 			LS_u16kwh+=10;
		}
		if(MET_bten_kvah_f==1)
		{
			MET_bten_kvah_f=0;
			BILL_u16mdkva_c +=10;
			LS_u16kvah +=10;
		}
		if(MET_bkvah1p_f == 1)
		{
			MET_bkvah1p_f =0;
			MET_u32Cum_kvah +=1;
			MET_u32dkvah +=1;
			TOD_u32cum_zkvah +=1;
			if(MET_u32Cum_kvah>=KWh_RolloverValue)
			{
				roll_over_kvah();                  //5+1 roll over
			}
			if(TOD_u32cum_zkvah>=KWh_RolloverValue)
			{
				TOD_u32cum_zkvah=0;
				zkvah_rollover_f=1;
				Eprom_Read(kvah_zkvah_rollover_f_add);
				opr_data[1]=1;
				Eprom_Write(kvah_zkvah_rollover_f_add);
			}
			store_energy();
		}

		if(MET_bkwh1p_f==1)
		{
			MET_bkwh1p_f=0;
			MET_u32Cum_kwh +=1;
			TOD_u32cum_zkwh +=1;
			if(TP1.b.mag_tpr_f==1)
			{
				TPR_u32cum_mag_defraud +=1;
				if(TPR_u32cum_mag_defraud >= KWh_RolloverValue)
				{
				TPR_u32cum_mag_defraud=0;
				}
			}
			else if(TP3.b.neu_miss_tpr_f==1 || TP3.b.neu_dis_tpr_f==1)
			{
				TPR_u32cum_neutemp_defraud +=1;
				if(TPR_u32cum_neutemp_defraud >=KWh_RolloverValue)
				{
				TPR_u32cum_neutemp_defraud =0;
				}
			}
			if(MET_u32Cum_kwh >= KWh_RolloverValue)
			{
				roll_over();		//5+1 roll over
			}
			if(TOD_u32cum_zkwh >= KWh_RolloverValue)
			{
				TOD_u32cum_zkwh=0;
				zkvah_rollover_f=0;
				Eprom_Read(kvah_zkvah_rollover_f_add);
				opr_data[1]=0;
				Eprom_Write(kvah_zkvah_rollover_f_add);				
			}
			store_energy();
		}
	}
}
void store_energy(void)
{
	if(MET_u32Cum_kvah == MET_u32dkvah)
	{
		MET_vcheck_kwh_kvah();
		
		FUN_vfill_3byteR(MET_u32Cum_kvah,&opr_data[2]);
		FUN_vfill_3byteR(MET_u32dkvah,&opr_data[5]);
		
		FUN_vfill_3byteR(MET_u32Cum_kwh,&opr_data[8]);
		
		FUN_vfill_3byteR(TOD_u32cum_zkwh,&opr_data[11]);
		
		FUN_vfill_3byteR(TOD_u32cum_zkvah,&opr_data[14]);
		
		Eprom_Write(CircularBufferEnergy+MET_u8CircularPtr);
		
		//if(batt_disp_f==1)
		//Eprom_Read(0x0100+(MET_u8CircularPtr-0x10));
		//else
		{
			opr_data[0] = TOD_u8zone_index;
			FUN_vfill_2byteR(BILL_u16mdkw_c ,&opr_data[2]);
			FUN_vfill_2byteR(BILL_u16mdkva_c ,&opr_data[4]);
			
			FUN_vfill_2byteR(LS_u16kwh,&opr_data[11]);
			FUN_vfill_2byteR(LS_u16kvah,&opr_data[13]);
			
			
			if(batt_disp_f==0)
			{
			time_stamp(&opr_data[5]);
			}
			else
			{
				opr_data[5]=time_array[0];
				opr_data[6]=time_array[1];
				opr_data[7]=time_array[2];
				opr_data[8]=time_array[3];
				opr_data[9]=time_array[4];
				
			}
		}
		Eprom_Write(CircularBufferMDLSkwh+MET_u8CircularPtr);
		MET_u8CircularPtr += 0x10;
	}
	else
	{
	ram_crpt_f0 = 1;
	}
}
void roll_over_kvah(void)
{
	unsigned char i;
	kvah_rollover_f=1;
	Eprom_Read(kvah_zkvah_rollover_f_add);
	opr_data[0]=1;
	Eprom_Write(kvah_zkvah_rollover_f_add);
	MET_u32Cum_kvah = 0;
	MET_u32dkvah = 0;
	MET_u8CircularPtr = 0;
	for(i=0;i<16;i++)
	{
		Eprom_Read(CircularBufferEnergy+MET_u8CircularPtr);
		opr_data[0]=0;
		opr_data[1]=0;
		opr_data[2]=0;
		opr_data[3]=0;
		opr_data[4]=0;
		opr_data[5]=0;

		Eprom_Write(CircularBufferEnergy+MET_u8CircularPtr);
		MET_u8CircularPtr +=16;
	}
}


void roll_over(void)
{
	u8 i;
	kvah_rollover_f=0;
	Eprom_Read(kvah_zkvah_rollover_f_add);
	opr_data[0]=0;
	Eprom_Write(kvah_zkvah_rollover_f_add);	
	MET_u32Cum_kwh = 0;
	MET_u8CircularPtr = 0;
	for(i=0;i<16;i++)
	{
		Eprom_Read(CircularBufferEnergy+MET_u8CircularPtr);
		opr_data[6]=0;
		opr_data[7]=0;
		opr_data[8]=0;
		Eprom_Write(CircularBufferEnergy+MET_u8CircularPtr);
		MET_u8CircularPtr +=16;
	}

}
void save_pd(void)
{
	Eprom_Read(PowerDownByteAdd);
	opr_data[3]=1;
	Eprom_Write(PowerDownByteAdd);
}

void save_pulse_cntrs(void)
{
	FUN_vfill_2byteR(MET_u16kw_pulse_cntr,&opr_data[1]);
	FUN_vfill_2byteR(MET_u16kva_pulse_cntr,&opr_data[3]);
	FUN_vfill_3byteR(TPR_u32cum_mag_defraud,&opr_data[6]);
	FUN_vfill_3byteR(TPR_u32cum_neutemp_defraud,&opr_data[9]);
	Eprom_Write(kw_pulse_c_addrs);
}
void write_energy(void)
{
	MET_u32Cum_kwh +=1;

	TOD_u32cum_zkwh +=1;
	
	if(TP1.b.mag_tpr_f)
	{
		TPR_u32cum_mag_defraud +=1;
		if(TPR_u32cum_mag_defraud==KWh_RolloverValue)
		{
			TPR_u32cum_mag_defraud=0;
		}
	}
	else if(TP3.b.neu_miss_tpr_f==1 || TP3.b.neu_dis_tpr_f==1)
	{
		TPR_u32cum_neutemp_defraud +=1;
		if(TPR_u32cum_neutemp_defraud==KWh_RolloverValue)
		{
			TPR_u32cum_neutemp_defraud=0;
		}
	}

	if(MET_u32Cum_kwh>=KWh_RolloverValue)
	{
		roll_over();                  //5+1 roll over
	}
	if(TOD_u32cum_zkwh>=KWh_RolloverValue)
	{
		TOD_u32cum_zkwh=0;
		zkvah_rollover_f=0;
		Eprom_Read(kvah_zkvah_rollover_f_add);
		opr_data[1]=0;
		Eprom_Write(kvah_zkvah_rollover_f_add);		
	}
	store_energy();

}

void write_kvaenergy(void)
{
	MET_u32Cum_kvah +=1;

	MET_u32dkvah +=1;
	TOD_u32cum_zkvah +=1;

	if(MET_u32Cum_kvah >= KWh_RolloverValue)
	{
		roll_over_kvah();                  //5+1 roll over
	}
	if(TOD_u32cum_zkvah>=KWh_RolloverValue)
	{
		TOD_u32cum_zkvah=0;
		zkvah_rollover_f=1;
		Eprom_Read(kvah_zkvah_rollover_f_add);
		opr_data[1]=1;
		Eprom_Write(kvah_zkvah_rollover_f_add);
	}
	store_energy();
}

void log_battery_status(void)
{
	Eprom_Read(BatteryStatus);
	if(zcd1_f==1)
	{
		zcd1_f=0;
		if(TP6.b.bat_discharge_f==1 && disable_bkup_f==0)
		{
			persis_c++;
			persis_c1=0;
			if(persis_c>=2)
			{
				persis_c=0;
				disable_bkup_f = 1;
				
				fill_oprzero();
				time_stamp(&opr_data[0]);
				opr_data[5]=1;
				Eprom_Write(BatteryStatus);

			}
		}
		else if(TP6.b.bat_discharge_f==0 && disable_bkup_f==1)
		{
			persis_c1++;
			persis_c=0;
			if(persis_c1>=2)
			{
				persis_c1=0;
				disable_bkup_f=0;
				Eprom_Read(BatteryStatus);
				opr_data[5]=0;
				Eprom_Write(BatteryStatus);
			}
		}       
	}	
}

