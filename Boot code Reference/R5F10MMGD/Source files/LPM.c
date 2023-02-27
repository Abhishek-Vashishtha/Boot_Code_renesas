/***********************************************************************************************************************
* File Name    : LPM.c
* Version      : 
* Device(s)    : R5F10MMG
* Tool-Chain   : CC-RL
* Description  : This file implements device driver for CGC module.
* Creation Date: 
***********************************************************************************************************************/


/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "r_cg_macrodriver.h"
#include "variable.h"
#include "Eprom_i2c.h"
#include "LCD.h"
#include "timer.h"
#include "dlms.h"
#include "tamper.h"
#include "function.h"
#include "meterology.h"

/***********************************************************************************************************************
Pragma directive
***********************************************************************************************************************/
#pragma interrupt LPM_PORT125_Interrupt(vect = INTP1)
#pragma interrupt LPM_PORT13_Interrupt(vect = INTP0)
/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/

/***********************************************************************************************************************
* Function Name: goto_stopmode
* Description  : This function initializes the clock generator.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void goto_stopmode(void)
{

	MET_bsleep_mode_f=0;
	MET_u32delta_kw_cnts=0;
	MET_u32kw_cnts=0;
	MET_u32kw_el_1s=0;
	MET_u32kw_1s=0;
	batt_disp_f=1;
	SW_u16bat_cntr=0;
 	SW_u16bkup_cntr=0;
 	SW_u8bat_disp_c=0;
 	PWR_bBatt_backup_f=0;
 	dispshow_unlock_f=0;
 	disp_lock_f=0;
 	SW_u8disp_cntr=0;
 	first_pres_f=0;
 	switch_disp_f=0;
 	sw_lock_f=0;
 	SW_u16normal_scroll=0;
 	SW_u16unlock_cntr=0;
	
	DSADCEN = 0;		//enable module
	DSAMK = 1U;
	DSADMR=0;
	WDTIMK = 1U;    	/* enable INTWDTI interrupt */
	
	TAU0EN = 0U;		/* supplies input clock */
	R_TAU0_Channel0_Stop();
#if IR==1
    R_TAU0_Channel1_Stop();
#endif 

	LCDON = 0U;    /* disable LCD clock operation */
	R_LCD_Set_VoltageOff();
	UART_vStop();
	
	IICA0EN = 0U;	/* supply IICA0 clock */
	IICAMK0 = 1;

	TMIF00 = 0;
	RTCIF = 0;
	DSAIF = 0;
	
	ADPC = 0x00;
	ADCEN = 0;
	
	PM0 = 0xff;
	PM1 = 0xff;
	PM2 = 0xff;
	
	PM3 = 0xf5;
	P3  &= 0xf5;
	
	PM4 = 0xff;
	PM6 = 0xff;
	PM7 = 0xff;
	PM8 = 0xff;
	PM12 = 0xff;
	
	PU1 = 0xff;
	PU7 = 0xff;
	PU8 = 0x0f;
	//PU0 |= BIT0 | BIT1 ;//irda
	//PU4 |= BIT1  ;
	PU4 |= BIT3; 
	PU0 |= BIT2;
	PU0 |= BIT5;
	
	TP6.b.sleep_f=1;
	
	PPR11 = 1U;
   	PPR01 = 1U;
	
	if(disable_bkup_f==0)
	{
	    PIF1 = 0U;    /* clear INTP1 interrupt flag */
	    PMK1 = 0U;    /* enable INTP1 interrupt */
		
		/* Enabling Top cover interrupt */
		PIF7 = 0;
		PMK7 = 0;
		
		RTCWEN = 1;
		RTCC0 |= 0x03;
		RTCC0 &= ~BIT5;
		RTCC1 &= 0xF7;
		P13 |= BIT0;
		RTCIF = 0U;
		RTCMK = 0;
		RTCWEN = 0;
	}
	else
	{
		/* Disable switch */
		PIF1 = 0U;     
	    PMK1 = 1U;
		
		RTCWEN = 1;
		RTCC0 |= 0x00;
		RTCC0 &= ~BIT5;
		RTCC1 &= 0xF7;
		P13 |= BIT0;
		RTCIF = 0U;
		RTCMK = 1;
		RTCWEN = 0;
	}
	
	__stop();
	__nop();
	
	CSS = 0U;
	__nop();
	__nop();
	__nop();
	__nop();

	
	EPR_vInit();
	
	if(TP5.b.tc_tprstopmode==1)// enable timer in case of TC tamper interrupt is present
	{
		TIM_vInit();
		R_TAU0_Channel0_Start();
	}

	TP6.b.sleep_f=1;
	
}
/***********************************************************************************************************************
* Function Name: port13
* Description  : This function initializes the clock generator.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
static void __near LPM_PORT13_Interrupt(void)
{
	PIF0 = 0;	
	if((batt_disp_f==1)||(test_mode==1))
	{
            power_up_f1=1;
	}
    zero_cross_counter++;
    if(zero_cross_counter >= 15)
    {
        bat_disable;
        zero_cross_counter = 0;
        SW_u16bkup_cntr = 0;
    }
        
	if(zero_cross_counter>=2)
	{
    batt_disp_f=0;
	MET_bsleep_mode_f=0;
	MET_bten_kva_f=0;
    test_mode=0;
	PWR_bBatt_backup_f=0;
    u8WakeupIntruptCntr=0;	
	}
        pd_counter = 0;                //  batt_disp_cntr = 0;
	stopmode=0;	
}
/***********************************************************************************************************************
* Function Name: port125
* Description  : This function handle interrupt
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
static void __near LPM_PORT125_Interrupt(void)
{
	if(((DN_Switch_Port & DN_Switch_Bit)==0) && batt_disp_f==1 && disable_bkup_f==0) 
	{
		if((MET_u32ip_rms > NeuMissCurntThr || MET_u32in_rms > NeuMissCurntThr) && neu_test_c < 50)
		{
			if((MET_u32ip_rms >NeuMissCurntThr && MET_u32in_rms <NeuMissCurntThr) ||(MET_u32in_rms >NeuMissCurntThr && MET_u32ip_rms <NeuMissCurntThr))
			{
				power_up_f1=1;
				test_mode=1;
				pd_counter2=0;
				neu_test_c++;
				PMK1 = 1U;    /* disable INTP1 interrupt */
				PIF1 = 0U;    /* clear INTP1 interrupt flag */
			}
		}
		else if(PWR_bBatt_backup_f==0)
		{
			PWR_bBatt_backup_f=1;
			first_pres_f=1;
			dispshow_unlock_f=0;
			switch_disp_f=1;
			SW_u8bat_disp_c=1;  
			SW_u8disp_cntr=0;
			SW_u16bkup_cntr=0;
			PMK1 = 1U;    /* disable INTP1 interrupt */
			PIF1 = 0U;    /* clear INTP1 interrupt flag */
			init_lcd_f=1;
		}
    }
	else
	{
		goto_stopmode();
	}
    PIF1 = 0U;    /* clear INTP1 interrupt flag */
}
void BAT_vReadADC10(void)
{
	u16 u16ADC_counter = 100;
    u16 u16ADCValue;
	
	ADCEN = 1;
	 // Select pin as analog input
	ADPC = 0x03;
	// Select pin as input
	PM2 |= BIT1;
	// Select Internal Reference voltage of 1.5V
	ADM0 = 0x3E;
	ADM1 = 0x20;
	ADM2 = 0x80;
	__nop();
	__nop();
	__nop();
	__nop();
	__nop();
	ADCE = 1;
	__nop();
	ADS = 0x01;
	ADM0 |= BIT7;
	for(u16ADC_counter=0;(u16ADC_counter<=1200)&&(ADCRH==0);u16ADC_counter++){__nop();}

    //add current value to accumalated 
    u16ADCValue=ADCRH;//ADC10MEM0;	//0.0178v = 1 count
    if(((u16ADCValue<150)&&(fg_done_f))||((u16ADCValue<185)&&(fg_done_f==0)))	//3.56=199;
    {
		charge_c1=0;
		discharge_c1++;
		if(disable_bkup_f==0 && discharge_c1 >=2)
        {
            TP6.b.bat_discharge_f=1;
			zcd1_f=1;
       		discharge_c1=0;
        }
	}
	else
	{
		discharge_c1=0;
		charge_c1++;
		if(disable_bkup_f==1 && charge_c1 >=2)
		{
			TP6.b.bat_discharge_f=0;
			zcd1_f=1;
			charge_c1=0;
    	}
	}
	ADCEN = 0;
	ADPC = 0x01;
}

/*************************************************
void RTC_BAT_vReadADC10(void)
{
	u16 u16ADC_counter = 100;
    u16 u16ADCValue;
	ADCEN = 1;
	ADPC = 0x04;
	PM2 |= BIT2;
	ADM0 = 0x3d;
	ADM1 = 0x20;
	ADM2 = 0x80;
	__nop();
	__nop();
	__nop();
	__nop();
	__nop();
	ADCE = 1;
	__nop();
	ADS = 0x02;
	ADM0 |= BIT7;
	for(u16ADC_counter=0;(u16ADC_counter<=1200)&&(ADCRH==0);u16ADC_counter++){__nop();}
	u16ADCValue=ADCRH;//ADC10MEM0;	//0.0178v = 1 count
	if(u16ADCValue<107)//&&(fg_done_f))||((u16ADCValue<185)&&(fg_done_f==0)))	//3.56=199;
	{
		charge_c=0;
		discharge_c++;
		if(RTC_BATT_discharge_f==0 && discharge_c >=2)
		{
			RTC_bat_ststus_f=1;
			zcd_f=1;
			discharge_c=0;
		}	
	}
	else
	{
		discharge_c=0;
		charge_c++;
		if(RTC_BATT_discharge_f==1 && charge_c >=2)
		{
			RTC_bat_ststus_f=0;
			zcd_f=1;
		}   
	}	
	ADCEN = 0;
	ADPC = 0x01;
}
******************************************************/

void stopmode_backup(void)
{
	save_pulse_cntrs();
    save_pd();
	MET_s32IpDC_acc=0;
	MET_s32InDC_acc=0;

//time_stamp(&time_array[0]);
	MET_u8CircularPtr=0;
	store_energy();
	fill_oprzero();
	FUN_vfill_4byteR(i_rms_avg,&opr_data[3]);
	FUN_vfill_4byteR(v_rms_avg,&opr_data[7]);
//	FUN_vfill_2byteR(u16RSSI_avg,&opr_data[9]);
	FUN_vfill_2byteR(cal_avg_cntr,&opr_data[13]);
	Eprom_Write(rms_avg_addrs);
	//Update Power down time only when Power supply cut
	if(batt_disp_f==0)
	{
		fill_oprzero();
		time_stamp(&opr_data[0]);
		opr_data[5]=dt.sec;
		Eprom_Write(PowerDownTimeAdd);
	}
}
void R_VBATT_SetOperationOn(void)
{
	BUPCTL1 = 0x80;
	VBATEN = 0U;
	VBATEN = 1U;
	VBATIE = 1;
	VBATIS = 1;
    GDIDIS = 0;
	VBATSEL = 0U;
}

void BattMode_init(void)
{
	LCD_vInit(0); 
	UART_vInit();
	UART_vStart();
	TIM_vInit();
	R_TAU0_Channel0_Start();
	#if IR==1
	  TIM_Output_Init();
	  R_TAU0_Channel1_Start();
	#endif	
}

