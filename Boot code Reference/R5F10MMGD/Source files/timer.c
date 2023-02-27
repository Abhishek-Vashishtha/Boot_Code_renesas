
/*
*******************************************************************************
** Include files
*******************************************************************************
*/
#include "r_cg_macrodriver.h"
#include "variable.h"
#include "timer.h"
#include "function.h"
#include "LPM.h"
#include "tamper.h"
#include "meterology.h"
#include "Eprom_i2c.h"
/* Start user code for include. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/********************************************************************************
** Pragma directive
********************************************************************************/
#pragma interrupt R_TAU0_Channel0_Interrupt(vect = INTTM00)

/*
*******************************************************************************
**  Global define
*******************************************************************************
*/
/* Start user code for global. Do not edit comment generated here */
//uint16_t  tm00_int;
//uint16_t  tm02_int;
//uint16_t  tm04_int;
/* End user code. Do not edit comment generated here */

/*
**-----------------------------------------------------------------------------
**
**  Abstract:
**	This function initializes the TAU0 module.
**
**  Parameters:
**	None
**
**  Returns:
**	None
**
**-----------------------------------------------------------------------------
*/
void TIM_vInit(void)
{
	TAU0EN = 1U;		/* supplies input clock */
	TPS0 = 0x0000;//_0000_TAU_CKM0_FCLK_0 | _0040_TAU_CKM1_FCLK_4 | _0000_TAU_CKM2_FCLK_1 | _0000_TAU_CKM3_FCLK_8;
	/* Stop all channels */
	TT0 = _0001_TAU_CH0_STOP_TRG_ON 
	| _0002_TAU_CH1_STOP_TRG_ON 
	| _0004_TAU_CH2_STOP_TRG_ON 
	| _0008_TAU_CH3_STOP_TRG_ON 
	| _0010_TAU_CH4_STOP_TRG_ON 
	| _0020_TAU_CH5_STOP_TRG_ON 
	| _0040_TAU_CH6_STOP_TRG_ON 
	| _0080_TAU_CH7_STOP_TRG_ON 
	| _0200_TAU_CH1_H8_STOP_TRG_ON 
	| _0800_TAU_CH3_H8_STOP_TRG_ON;
	/* Mask channel 0 interrupt */
	TMMK00 = 1U;	/* disable INTTM00 interrupt */
	TMIF00 = 0U;	/* clear INTTM00 interrupt flag */
	/* Mask channel 1 interrupt */
	TMMK01 = 1U;	/* disable INTTM01 interrupt */
	TMIF01 = 0U;	/* clear INTTM01 interrupt flag */
	/* Mask channel 1 higher 8 bits interrupt */
	TMMK01H = 1U;	/* disable INTTM01H interrupt */
	TMIF01H = 0U;	/* clear INTTM01H interrupt flag */
	/* Mask channel 2 interrupt */
	TMMK02 = 1U;	/* disable INTTM02 interrupt */
	TMIF02 = 0U;	/* clear INTTM02 interrupt flag */
	/* Mask channel 3 interrupt */
	TMMK03 = 1U;	/* disable INTTM03 interrupt */
	TMIF03 = 0U;	/* clear INTTM03 interrupt flag */
	/* Mask channel 3 higher 8 bits interrupt */
	TMMK03H = 1U;	/* disable INTTM03H interrupt */
	TMIF03H = 0U;	/* clear INTTM03H interrupt flag */
	/* Mask channel 4 interrupt */
	TMMK04 = 1U;	/* disable INTTM04 interrupt */
	TMIF04 = 0U;	/* clear INTTM04 interrupt flag */
	/* Mask channel 5 interrupt */
	TMMK05 = 1U;	/* disable INTTM05 interrupt */
	TMIF05 = 0U;	/* clear INTTM05 interrupt flag */
	/* Mask channel 6 interrupt */
	TMMK06 = 1U;	/* disable INTTM06 interrupt */
	TMIF06 = 0U;	/* clear INTTM06 interrupt flag */
	/* Mask channel 7 interrupt */
	TMMK07 = 1U;	/* disable INTTM07 interrupt */
	TMIF07 = 0U;	/* clear INTTM07 interrupt flag */
	/* Set INTTM00 level 1 priority */
	TMPR100 = 0U;
	TMPR000 = 1U;
	/* Set INTTM02 level 2 priority */
	TMPR102 = 1U;
	TMPR002 = 1U;
	/* Set INTTM04 level 1 priority */
	TMPR104 = 1U;
	TMPR004 = 1U;
	/* Channel 0 used as interval timer */
	TMR00 = _0000_TAU_CLOCK_SELECT_CKM0 
	| _0000_TAU_CLOCK_MODE_CKS 
	| _0000_TAU_COMBINATION_SLAVE 
	| _0000_TAU_TRIGGER_SOFTWARE 
	| _0000_TAU_MODE_INTERVAL_TIMER 
	| _0000_TAU_START_INT_UNUSED;
	//	TDR00 = _0752_TAU_TDR00_VALUE;
#if X1_FREQ == 1
	TDR00 = 0xEA5F;//(1/12000khz)*(59999+1)=5ms
#elif X1_FREQ == 2	
	TDR00 = 0x752F;//(1/6000khz)*(29999+1)=5ms
#elif X1_FREQ == 3	
	TDR00 = 0x3A97;//(1/3000khz)*(14999+1)=5ms
#endif	
	TO0 &= ~_0001_TAU_CH0_OUTPUT_VALUE_1;
	TOE0 &= ~_0001_TAU_CH0_OUTPUT_ENABLE;
}

#if IR==1
void TIM_Output_Init(void)
{
	TAU0EN = 1U;		/* supplies input clock *///PER0 enabled
	TPS0 = 0x0000;//_0000_TAU_CKM0_FCLK_0 | _0040_TAU_CKM1_FCLK_4 | _0000_TAU_CKM2_FCLK_1 | _0000_TAU_CKM3_FCLK_8;
	/* Stop all channels */
	TT0 = _0001_TAU_CH0_STOP_TRG_ON 
	| _0002_TAU_CH1_STOP_TRG_ON 
	| _0004_TAU_CH2_STOP_TRG_ON 
	| _0008_TAU_CH3_STOP_TRG_ON 
	| _0010_TAU_CH4_STOP_TRG_ON 
	| _0020_TAU_CH5_STOP_TRG_ON 
	| _0040_TAU_CH6_STOP_TRG_ON 
	| _0080_TAU_CH7_STOP_TRG_ON 
	| _0200_TAU_CH1_H8_STOP_TRG_ON 
	| _0800_TAU_CH3_H8_STOP_TRG_ON;
	TMMK00 = 1U;	/* disable INTTM00 interrupt */
	TMIF00 = 0U;	/* clear INTTM00 interrupt flag */
	/* Mask channel 1 interrupt */
	TMMK01 = 1U;	/* disable INTTM01 interrupt */
	TMIF01 = 0U;	/* clear INTTM01 interrupt flag */
	/* Mask channel 1 higher 8 bits interrupt */
	TMMK01H = 1U;	/* disable INTTM01H interrupt */
	TMIF01H = 0U;	/* clear INTTM01H interrupt flag */
	/* Mask channel 2 interrupt */
	TMMK02 = 1U;	/* disable INTTM02 interrupt */
	TMIF02 = 0U;	/* clear INTTM02 interrupt flag */
	/* Mask channel 3 interrupt */
	TMMK03 = 1U;	/* disable INTTM03 interrupt */
	TMIF03 = 0U;	/* clear INTTM03 interrupt flag */
	/* Mask channel 3 higher 8 bits interrupt */
	TMMK03H = 1U;	/* disable INTTM03H interrupt */
	TMIF03H = 0U;	/* clear INTTM03H interrupt flag */
	/* Mask channel 4 interrupt */
	TMMK04 = 1U;	/* disable INTTM04 interrupt */
	TMIF04 = 0U;	/* clear INTTM04 interrupt flag */
	/* Mask channel 5 interrupt */
	TMMK05 = 1U;	/* disable INTTM05 interrupt */
	TMIF05 = 0U;	/* clear INTTM05 interrupt flag */
	/* Mask channel 6 interrupt */
	TMMK06 = 1U;	/* disable INTTM06 interrupt */
	TMIF06 = 0U;	/* clear INTTM06 interrupt flag */
	/* Mask channel 7 interrupt */
	TMMK07 = 1U;	/* disable INTTM07 interrupt */
	TMIF07 = 0U;	/* clear INTTM07 interrupt flag */
//	/* Set INTTM00 level 1 priority */
//	TMPR100 = 0U;
//	TMPR000 = 1U;
//	/* Set INTTM02 level 2 priority */
//	TMPR102 = 1U;
//	TMPR002 = 1U;
//	/* Set INTTM04 level 1 priority */
//	TMPR104 = 1U;
//	TMPR004 = 1U;
	/* Channel 0 used as interval timer */
	TMR01 = _0000_TAU_CLOCK_SELECT_CKM0 
	| _0000_TAU_CLOCK_MODE_CKS 
	| _0000_TAU_COMBINATION_SLAVE 
	| _0000_TAU_TRIGGER_SOFTWARE 
	| _0000_TAU_MODE_INTERVAL_TIMER 
	| _0000_TAU_START_INT_UNUSED
	| _0001_TAU_MODE_PWM_MASTER; /* PWM Function (Master Channel) mode */
	//	TDR00 = _0752_TAU_TDR00_VALUE;
#if X1_FREQ == 1
	TDR01 = 0xEA5F;//(1/12000khz)*(59999+1)=5ms
#elif X1_FREQ == 2	
	TDR01 = 0x752F;//(1/6000khz)*(29999+1)=5ms
#elif X1_FREQ == 3	
	TDR01 = 0x0027;
#endif	

	TO0  =  _0002_TAU_CH1_OUTPUT_VALUE_1;
	TOE0 =  _0002_TAU_CH1_OUTPUT_ENABLE;
	TOM0 =  _0000_TAU_CH1_OUTPUT_TOGGLE; /*toggle operation mode */
	TOL0 =  _0000_TAU_CH1_OUTPUT_LEVEL_H;
	PM4 &= ~(BIT1);
	P4  &= ~(BIT1);
}
#endif
/*
**-----------------------------------------------------------------------------
**
**  Abstract:
**	This function starts TAU0 channel 0 counter.
**
**  Parameters:
**	None
**
**  Returns:
**	None
**
**-----------------------------------------------------------------------------
*/
void R_TAU0_Channel0_Start(void)
{
	TMIF00 = 0U;	/* clear INTTM00 interrupt flag */
	TMMK00 = 0U;	/* enable INTTM00 interrupt */
	TS0 |= _0001_TAU_CH0_START_TRG_ON;
}
void R_TAU0_Channel1_Start(void)
{
	TMIF01 = 0U;	/* clear INTTM00 interrupt flag */
	TMMK01 = 1U;	/* disable INTTM00 interrupt */
	TS0   |= _0002_TAU_CH1_START_TRG_ON;  /* operation is enabled (start software trigger is generated) */
}

void R_TAU0_Channel1_Stop(void)
{
	TT0 |= _0002_TAU_CH1_STOP_TRG_ON;
	TMMK01 = 1U;	/* disable INTTM00 interrupt */
	TMIF01 = 0U;	/* clear INTTM00 interrupt flag */
}
void R_TAU0_Channel0_Stop(void)
{
	TT0 |= _0001_TAU_CH0_STOP_TRG_ON;
	/* Mask channel 0 interrupt */
	TMMK00 = 1U;	/* disable INTTM00 interrupt */
	TMIF00 = 0U;	/* clear INTTM00 interrupt flag */
}
/*
**-----------------------------------------------------------------------------
**
**  Abstract:
**	This function starts TAU0 channel 2 counter.
**
**  Parameters:
**	None
**
**  Returns:
**	None
**
**-----------------------------------------------------------------------------
*/
void R_TAU0_Channel2_Start(void)
{
	TMIF02 = 0U;	/* clear INTTM02 interrupt flag */
	TMMK02 = 0U;	/* enable INTTM02 interrupt */
	TS0 |= _0004_TAU_CH2_START_TRG_ON;
}
/*
**-----------------------------------------------------------------------------
**
**  Abstract:
**	This function stops TAU0 channel 2 counter.
**
**  Parameters:
**	None
**
**  Returns:
**	None
**
**-----------------------------------------------------------------------------
*/
void R_TAU0_Channel2_Stop(void)
{
	TT0 |= _0004_TAU_CH2_STOP_TRG_ON;
	/* Mask channel 2 interrupt */
	TMMK02 = 1U;	/* disable INTTM02 interrupt */
	TMIF02 = 0U;	/* clear INTTM02 interrupt flag */
}
/*
**-----------------------------------------------------------------------------
**
**  Abstract:
**	This function starts TAU0 channel 4 counter.
**
**  Parameters:
**	None
**
**  Returns:
**	None
**
**-----------------------------------------------------------------------------
*/
void R_TAU0_Channel4_Start(void)
{
	TMIF04 = 0U;	/* clear INTTM04 interrupt flag */
	TMMK04 = 0U;	/* enable INTTM04 interrupt */
	TS0 |= _0010_TAU_CH4_START_TRG_ON;
}
/*
**-----------------------------------------------------------------------------
**
**  Abstract:
**	This function stops TAU0 channel 4 counter.
**
**  Parameters:
**	None
**
**  Returns:
**	None
**
**-----------------------------------------------------------------------------
*/
void R_TAU0_Channel4_Stop(void)
{
	TT0 |= _0010_TAU_CH4_STOP_TRG_ON;
	/* Mask channel 4 interrupt */
	TMMK04 = 1U;	/* disable INTTM04 interrupt */
	TMIF04 = 0U;	/* clear INTTM04 interrupt flag */
}

/* Start user code for adding. Do not edit comment generated here */
/*
**-----------------------------------------------------------------------------
**
**  Abstract:
**	This function is INTTM00 interrupt service routine.
**
**  Parameters:
**	None
**
**  Returns:
**	None
**
**-----------------------------------------------------------------------------
*/
static void __near R_TAU0_Channel0_Interrupt(void)
{

	TMIF00=0;
	
	if((TP6.b.sleep_f == 0) || (PWR_bBatt_backup_f==1))
	{
#if Genus_Protocol == 1
		timeout++;
	  	if(timeout>=1000)       // 5 sec
	  	{
	    		timeout=0;
	    		serial_rxcounter=0;
	    		serial_counter=0;
	    		flag15.all=0;
	    		flag16.all &=0xf1;
	    		time_out_f=1;
	    		signout_time++;
	    		if(signout_time>=12)   // 1 minute (60 sec)
	    		{
	      			event_log_f=0;
	      			signout_time=0;
	      			signon_flag=0;
	      			write_enable_flag=0;
	    		}
		}
#endif

		if(pow_up_f==1)
		{
			power_up_c++;
			if(power_up_c >= 1000)
			{
				power_up_c =0;
				pow_up_f=0;
			}
		}
		
		RTC_u8ReadCounter++;
		if(RTC_u8ReadCounter>=120)
		{
			RTC_u8ReadCounter=0;
			RTC_bReadFlag=1;
		}
		
		SW_u16normal_scroll++;
		if((SW_u16normal_scroll >= (u16)(lcd_scroll_time*200))&&(switch_disp_f==0))
		{
			SW_u16normal_scroll = 0;
			SW_u8disp_cntr++;
		}
		
		DISP_u8UpdateCntr++;
		if(DISP_u8UpdateCntr>200)
		{
			if(TP6.b.tc_exit_f == 1)
			{
				TC_u8exitcounter++;
			}		
			if((test_mode==0)&& (batt_disp_f==0))//&&(tc_exit_f == 0))
			{
				min_check++;
				if(min_check >= 60)
				{
					min_check = 0;
					pow_on++;
					cum_pow_on++;
					u16_bill_pow_on++;
					PON_u8SaveFlag = 1;
				}
			}
			DISP_bParCng=1;
			DISP_u8UpdateCntr=0;
			if(test_mode==1)
			{
	            pd_counter2++; 
			}
#if Genus_Protocol == 0
			////DLMS////
			Cntr_2Min++;   //For Inactivity Time Out
			if(Cntr_2Min >= 120 && nrm_flag == 1)			//30
			{
				nrm_flag=0;
				cosem_flag=0;
				rrr_s=0;
				rrr_c=0;
				rrr_c1=0;
				sss_c=0;
				sss_c1=0;
				sss_s=0;
				infore_flag=0;
				asso0_flag=0;
				asso1_flag=0;
				Cntr_2Min = 0;
				asso2_flag=0;
				asso3_flag=0;
				optical_f = 0;
				rj_disc_f2=0;
				rj_disc_f=0;
				rj_disc_cnt=0;
				//			rj_f=0;
				rcv_cnt1=0;
				multi_filling_f=0;
			}
#endif
			////DLMS////END
		}
		
		////DLMS////START
		serial_timeout++;
		if(serial_timeout>=20)
		{
			serial_timeout=0;
			rcv_cnt=0;
		}
	}
	
	if(rj_disc_f ==1)
	{
		rj_disc_cnt++;
	}
	
	
	if(TP6.b.tc_exit_f == 1)
	{
		TC_u16sleepcounter++;
		if(TC_u16sleepcounter >200)
		{
			TC_u16sleepcounter=0;
			TC_u8exitcounter++;			
		}
	}
	
	if(batt_disp_f==0)
	{
		////CALIBRATION////START
		if(((P2 & BIT0)==0 && (MET_bcalib_flag==0) && ((MET_bkw_negative_f == 0) && (MET_bkw_el_negative_f == 0))))// && batt_backup_f==0)doubt
		{
			
			MET_bcal_mode_f=1;
			cal_switch_f=0;
			SW_u16cal_counter++;
			if(SW_u16cal_counter>=1000)
			{
				SW_u16cal_counter=0;
				MET_bcalib_flag=1;
			}
			if((MET_blag_calib_f==1 && lag_ok==0))            
			{
				if((MET_u16Kw>1000)&&(MET_u16ELKw>1000))
				{
					MET_u16StepsP1_gain=0;
					DSADPHCR1 = 0;
					MET_u16StepsP2_gain=0;
					DSADPHCR0 = 0;
				}
			}        
		}
		if(MET_u16pf > 350 && MET_u16pf < 650)
		MET_blag_calib_f=1;
		else
		{
			MET_blag_calib_f=0;
			lag_ok=0;
		}
		////CALIBRATION////END		
		////CAL LED////START
		MET_u8led_blink++;  
		if(MET_u8led_blink>=8)
		P6 |= BIT2;//cal_led_off();//P6OUT |= cal_led;
		
		
		MET_u32kva_cnts += MET_u32delta_kva_cnts;
		if(MET_u32kva_cnts >= MET_u32mcl_kw)
		{
			MET_u32kva_cnts = MET_u32kva_cnts - MET_u32mcl_kw;
			MET_u16kva_pulse_cntr++;
			MET_u8watt_kva_100++;
			
#if RollOverFile ==1			
			MET_u32Cum_kvah +=80;
			MET_u32dkvah +=80;
			TOD_u32cum_zkvah +=80;
			if(MET_u32Cum_kvah>=KWh_RolloverValue)
			roll_over_kvah();                  //5+1 roll over
			if(TOD_u32cum_zkvah>=KWh_RolloverValue)
			{
				TOD_u32cum_zkvah=0;
				zkvah_rollover_f=1;
				Eprom_Read(kvah_zkvah_rollover_f_add);
				opr_data[1]=1;
				Eprom_Write(kvah_zkvah_rollover_f_add);
			}	
#endif 
			if(MET_u8watt_kva_100 >= 32)
			{
				MET_bten_kvah_f = 1;
				MET_u8watt_kva_100 = 0;
			}
			if(MET_u16kva_pulse_cntr >= 320)
			{
				MET_u16kva_pulse_cntr = 0;
				MET_bkvah1p_f = 1;
			}
		}
		MET_u32kw_cnts += MET_u32delta_kw_cnts;
		if(MET_u32kw_cnts >= MET_u32mcl_kw)
		{
			MET_u32kw_cnts = MET_u32kw_cnts - MET_u32mcl_kw;
			P6 &= ~(BIT2);//cal_led_glow
			MET_u8led_blink = 0;
			MET_u16kw_pulse_cntr++;
			MET_u8watt_100++;

#if RollOverFile ==1
			MET_u32Cum_kwh +=80;
			TOD_u32cum_zkwh +=80;
			if(TP1.b.mag_tpr_f==1)
			{
				TPR_u32cum_mag_defraud +=80;
				if(TPR_u32cum_mag_defraud >= KWh_RolloverValue)
				TPR_u32cum_mag_defraud=0;
			}
			else if(TP3.b.neu_miss_tpr_f==1 || TP3.b.neu_dis_tpr_f==1)
			{
				TPR_u32cum_neutemp_defraud +=80;
				if(TPR_u32cum_neutemp_defraud >=KWh_RolloverValue)
				TPR_u32cum_neutemp_defraud =0;
			}
			if(MET_u32Cum_kwh >= KWh_RolloverValue)
			roll_over();		//5+1 roll over
			if(TOD_u32cum_zkwh >= KWh_RolloverValue)
			{
				TOD_u32cum_zkwh=0;
				zkvah_rollover_f=0;
				Eprom_Read(kvah_zkvah_rollover_f_add);
				opr_data[1]=0;
				Eprom_Write(kvah_zkvah_rollover_f_add);				
			}
#endif		
			if(MET_u8watt_100 >= 32)
			{    
				MET_bten_wh_f = 1;
				MET_u8watt_100 = 0;
			}
			if(MET_u16kw_pulse_cntr >= 320)// 0.1 least count
			{
				MET_u16kw_pulse_cntr = 0;     
				MET_bkwh1p_f = 1;
			}
		}    			

		////CAL LED////END	
		stop_mode = 1;
		pd_counter++;
		if(pd_counter >= 15)    
		{
			bat_enable;   
		} 
		if(pd_counter >= 100)
		{
			stop_mode ^= 0x01;
			pd_counter = 0;
			zero_cross_counter = 0;
			MET_u16freq=0;
		}
		if((stop_mode == 0) && (test_mode==0)&& (stopmode==0))            // && v_rms <=2000
		{			
			stopmode=1;
			TP5.b.tc_tprstopmode=0;
		}
		
		if(MET_u32ip_rms >1000 || MET_u32in_rms >1000)
		{      
			if(pd_counter2 >= 600)//10 min=600 
			{
				pd_counter2=0;
				test_mode=0;
			}
		}
		else
		{
			if(pd_counter2 >= 120)//increase on second intrupt
			{
				pd_counter2=0;
				test_mode=0;
			}    
		}
		////MAGNET////START
		if((MET_u16v_rms>=10000) && (test_mode == 0)) 
		{
			if((TamperSelByte & TprSel_MagnetTamper)==TprSel_MagnetTamper)
			{
				if(((P3 & BIT2)==0) && TP1.b.mag_tpr_f==0)
				{
					TP1.b.mag_active_f=1;
					TPR_u16magt_cnt_s++;
					if(TPR_u16magt_cnt_s >= (tpr_u16MagStrTime*200))//6000)        //
					{
						TPR_u16magt_cnt_s=0x0000;
						TP1.b.mag_tpr_f=1;
						TP1.b.mag_tprstr_f=1;
						TPRCNT_u8Mag++;
						TPR_u16cum_tpr_c++;
						BILL_u8btpr_c++;
						TP.b.OtherEvent=1;
						others_count++;
						TP4.b.neu_fea_f1 = 0;
						TPR_u16neu_per_c = 0;
						TPR_u16neud_per_c = 0;
						TPR_u16neud_per_c1 = 0;
						if(TP3.b.neu_miss_tpr_f == 1)
						{
							TPR_u16neu_per_c1=0;
							TP3.b.neu_miss_tprrestr_f=1;
							TP.b.OtherEvent=1;
							others_count++;
							TP3.b.neu_miss_tpr_f=0;
						}     
					}
				}
				else if(((P3 & BIT2)==BIT2) && TP1.b.mag_tpr_f==1)
				{
					TPR_u16magt_cnt_r++;
					if(TPR_u16magt_cnt_r >= (tpr_u16MagRestrTime*200))//6000)      //7d0
					{
						TPR_u16magt_cnt_r=0x0000;
						TP1.b.mag_tpr_f=0;
						TP1.b.mag_tprrestr_f=1;
						TP.b.OtherEvent=1;
						others_count++;
					}
				}
				if((((P3 & BIT2)==0) && TP1.b.mag_tpr_f==1)||(((P3 & BIT2)==BIT2) && TP1.b.mag_tpr_f==0))
				{
					TPR_u16rmagt_cnt++;
					if(TPR_u16rmagt_cnt >= (ACMagnetPersistTime*200))//1000)
					{
						TPR_u16rmagt_cnt=0x0000;
						TPR_u16magt_cnt_s=0x0000;
						TPR_u16magt_cnt_r=0; 
						TP1.b.mag_active_f=0;
					}
				}
			}
		}
		////MAGNET///STOP
		////35KV///START
		if((TamperSelByte & TprSel_35KV)==TprSel_35KV)	//if(test_mode==0)
		{
			if(TP5.b.kv35_tpr_f == 1)
			{
				tpr_u32hvcnt++;
				if(tpr_u32hvcnt >= (tpr_u16KV35RestrTime*200))// 1 min restoration
				{
					tpr_u32hvcnt=0;
					TP5.b.kv35_tpr_f=0;
					TP5.b.kv35_tprrestr_f=1;
					TP.b.OtherEvent=1;
					others_count++;
					TP5.b.kv35_tprstr_f=0;
					abnrm_c=0;	
					check_tpr();
				}
			}
		}
	}
	
	////LPM
	if(PWR_bBatt_backup_f==1)  
	{
		SW_u16bkup_cntr++;
		if(SW_u16bkup_cntr>=BattBckupTime*200)//(60/.005)
		{
			SW_u16bkup_cntr=0;
			switch_disp_f=0;
			SW_u8bat_disp_c=0;
			sw_lock_f=0;
			first_pres_f=0;
			SW_u8disp_cntr = 0;
			SW_u16normal_scroll=0;
			SW_u16unlock_cntr=0;
			dispshow_unlock_f=1; 
			PWR_bBatt_backup_f =0;
			stopmode=1;    
		}
	} 
	
	////PUSH BUTTON////START
	if((TP6.b.sleep_f == 0) || (PWR_bBatt_backup_f==1))
	{
		if((P12 & BIT5)==0)//if((P2IN & up_switch)==0)  // switch is pressed
		{
			if(SW_u16unlock_cntr<12000)
			{	
				SW_u16unlock_cntr++;
			}
			if(SW_u16unlock_cntr>=4)
			{
				SW_u8Sw_f1=1;
				SW_u8Sw_f2=0;
			}
			if(SW_u16unlock_cntr==PushToAutoTimeButtonPress*200)
			{
				if(batt_disp_f==0)
				{
					SW_u8Sw_f1=0;
					SW_u8Sw_f2=1;
				}
			}

//			if(SW_u16unlock_cntr==1000)
//			{
//				if(batt_disp_f==0)
//				{
//					SW_u8Sw_f1=0;
//					SW_u8Sw_f2=0;
//					SW_u8Sw_f3=1;
//				}
//			}
		}
		else
		{
			SW_u16unlock_cntr=0;
			sw_lock_f=0;
		}
		
		if(SW_u8Test_menu_f==0)
		{
			if(SW_u8Sw_f1==1)
			{
				if(first_pres_f==0)
				{
					first_pres_f=1;
					SW_u16bat_cntr=0;
					SW_u8bat_disp_c=0;
					disp_lock_f=1;
				}
				if(((sw_lock_f==0) && (((P2 & BIT0)==0) && ((MET_bkw_negative_f == 0) && (MET_bkw_el_negative_f == 0))&&(PWR_bBatt_backup_f==0))))
				{
					cal_switch_f=1;
					SW_u8cal_bat_disp_c++;
					sw_lock_f=1;
					SW_u16bat_cntr=0;
				}
				if(sw_lock_f==0 && dispshow_unlock_f==0)
				{
					switch_disp_f=1;
					SW_u8bat_disp_c++;
					sw_lock_f=1;
					SW_u16normal_scroll=0;
					SW_u16bat_cntr=0;
				}
				SW_u8Sw_f1=0;
			}
			if(SW_u8Sw_f2==1)
			{
				switch_disp_f=0;
				SW_u8bat_disp_c=0;
				first_pres_f=0;
				dispshow_unlock_f=1;
				SW_u16normal_scroll=0;
				SW_u8disp_cntr=0;
				SW_u8Sw_f2=0;
				if(batt_disp_f==0)
				{
					first_pres_f=0;      
					dispshow_unlock_f=1;
				}
			}
//			if(SW_u8Sw_f3==1)
//			{
//				if(disp_kwh_f==0)
//				{
//					disp_kwh_f=1;
//					lock_kwh_f=1;
//				}
//				else
//				{
//					disp_kwh_f=0;
//					lock_kwh_f=0;
//					sw_lock_f=1;
//					switch_disp_f=0;
//					SW_u8bat_disp_c=0;
//					SW_u8disp_cntr=0;
//					disp_lock_f=1;
//					dispshow_unlock_f=1;
//				}
//				SW_u8Sw_f3=0;
//			}
		}
		if(first_pres_f==1 && batt_disp_f==0)
		{
			SW_u16bat_cntr++;
			if(SW_u16bat_cntr >= PushToAutoTime*200)    //6000 for 30 second demand display on time(1200 for 6 sec)
			{
				SW_u16bat_cntr=0;
				switch_disp_f=0;
				SW_u8bat_disp_c=0;
				sw_lock_f=0;
				batt_disp_f=0;
				first_pres_f=0;
				SW_u8disp_cntr = 0;
				PWR_bBatt_backup_f=0;
				SW_u16normal_scroll=0;
				dispshow_unlock_f=1;
			}
		}
	}
	////TOP COVER////START
	if((TamperSelByte & TprSel_TCOpen)==TprSel_TCOpen)
	{
		if(((TC_OPEN_Port & TC_OPEN_Bit)!=0) && (TP5.b.tc_tpr_f==0) )
		{
			TPR_u16tct_cnt++;
			if(TPR_u16tct_cnt>= tpr_u16TCStrTime)
			{
				TPR_u16tct_cnt=0x0000;
				TP5.b.tc_tpr_f=1;
				TP5.b.tc_tprstr_f=1;
				nonroll_count++;
				TP.b.NonRollEvent=1;
			}
		}
		
		if((((TC_OPEN_Port & TC_OPEN_Bit)!=0) && TP5.b.tc_tpr_f==1)||(((TC_OPEN_Port & TC_OPEN_Bit)==0) && TP5.b.tc_tpr_f==0))
		{
			TPR_u16tct_cnt=0x0000;
		}
	}
}

