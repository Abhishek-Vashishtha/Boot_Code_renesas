
/***********************************************************************************************************************
* File Name    : r_cg_rtc.c
* Version      : CodeGenerator for RL78/L12 E1.00.00c [23 Mar 2012]
* Device(s)    : R5F10MMG
* Tool-Chain   : CC-RL
* Description  : This file implements device driver for RTC module.
* Creation Date: 11/1/2012
***********************************************************************************************************************/




/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "r_cg_macrodriver.h"
#include "variable.h"
#include "rtc.h"
#include "meterology.h"
#include "string.h"
#include "Eprom_i2c.h"
#include "function.h"
/* Start user code for include. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
Pragma directive
***********************************************************************************************************************/
#pragma interrupt r_rtc_interrupt(vect = INTRTC)
#pragma interrupt r_rtc_correction_interrupt(vect = INTRTIT)

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
/* Start user code for global. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
* Function Name: R_RTC_Create
* Description  : This function initializes the real-time clock module.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void RTC_vInit(void)
{
	RTCWEN = 1U;   /* supply RTC clock */
	RTCE = 0U;     /* disable RTC clock operation */
	__nop();
	__nop();
	RTCMK = 1U;    /* disable INTRTC interrupt */
	RTCIF = 0U;    /* clear INTRTC interrupt flag */
	/* Set INTRTC low priority */
	RTCPR1 = 1U;
	RTCPR0 = 1U;
	//    RTCC0 = _00_RTC_RTC1HZ_DISABLE | _08_RTC_24HOUR_SYSTEM | _01_RTC_INTRTC_CLOCK_0;
	RTCC0 = _20_RTC_RTC1HZ_ENABLE /*| 0x40*/ | _08_RTC_24HOUR_SYSTEM | _02_RTC_INTRTC_CLOCK_1;//_01_RTC_INTRTC_CLOCK_0;
	RTCC1 &= 0xF7;
	RTCC1 |= 0x20;   // enable INTITIT clock error correction interrupt  
	RTCIF = 0U;    /* clear INTRTC interrupt flag */
	RTCE = 1U;     /* enable RTC clock operation */
	__nop();
	__nop();
	RTCWEN = 0;
}
void RTC_RamInit(void)
{
	u16 u16temp;
	rtc_calib_f=1;
//	fill_oprzero();
//	Eprom_Write(0x0520);
//	FMCEN = 1;
//	SUBCUD = 0x0020;
//	FMCEN = 0;
	seccng_f=0;
	mincng_f=0;
	hrscng_f=0;
	daycng_f=0;
	monthcng_f=0;
	
	if((P2 & BIT0)!=0)
	{
		Eprom_Read(RTC_Calibration_add);
	if((EPROM_bChksum_ok ==1)&&(opr_data[0]!=0))
	{
		u16temp = a8_to_u16(&opr_data[0]);
		if(SUBCUD != u16temp)
		{
				rtc_correction_f=1;
				rtc_correction_state = 0x01;          
//FMCEN = 1;
//			SUBCUD = u16temp;
//			FMCEN = 0;
		}
		else
		{
			rtc_calib_f=0;		
		}
		u8RtcCalSign = opr_data[0];
		u8RtcCalSec =opr_data[1]; 
		}
	}
	else
	{
		rtc_correction_f=1;
		rtc_correction_state= 0x02;
//		FMCEN = 1;                          // this routine written into interrupt service : r_rtc_correction_interrupt
//		SUBCUD = 0x0020;
//		FMCEN = 0;	
	}
}

/***********************************************************************************************************************
* Function Name: R_RTC_Start
* Description  : This function enables the real-time clock.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void RTC_vStart(void)
{
	RTCIF = 0U;    /* clear INTRTC interrupt flag */
	RTCMK = 0U;    /* enable INTRTC interrupt */
	RTCE = 1U;     /* enable RTC clock operation */
}

/***********************************************************************************************************************
* Function Name: R_RTC_Stop
* Description  : This function disables the real-time clock.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void RTC_vStop(void)
{
	RTCE = 0U;    /* disable RTC clock operation */
	RTCMK = 1U;   /* disable INTRTC interrupt */
	RTCIF = 0U;   /* clear INTRTC interrupt flag */
}

/***********************************************************************************************************************
* Function Name: R_RTC_Get_CounterValue
* Description  : This function reads the results of real-time clock and store them in the variables.
* Arguments    : counter_read_val -
*                    the current real-time clock value(BCD code)
* Return Value : status -
*                    MD_OK, MD_BUSY1 or MD_BUSY2
***********************************************************************************************************************/
MD_STATUS R_RTC_Get_CounterValue(rtc_counter_value_t * const counter_read_val)
{
	MD_STATUS status = MD_OK;
	volatile uint32_t  w_count;
	
	RTCWEN = 1U;
	
	RTCC1 |= _01_RTC_COUNTER_PAUSE;

	/* Change the waiting time according to the system */
	for (w_count = 0U; w_count < RTC_WAITTIME; w_count++)
	{
		__nop();
	}

	if (0U == RWST)
	{
		status = MD_BUSY1;
	}
	else
	{
		counter_read_val->sec = SEC;
		counter_read_val->min = MIN;
		counter_read_val->hour = HOUR;
		counter_read_val->week = WEEK;
		counter_read_val->day = DAY;
		counter_read_val->month = MONTH;
		counter_read_val->year = YEAR;

		RTCC1 &= (uint8_t)~_01_RTC_COUNTER_PAUSE;

		/* Change the waiting time according to the system */
		for (w_count = 0U; w_count < RTC_WAITTIME; w_count++)
		{
			__nop();
		}

		if (1U == RWST)
		{
			status = MD_BUSY2;
		}
	}

	RTCWEN = 0;
	
	return (status);
}

/***********************************************************************************************************************
* Function Name: R_RTC_Set_CounterValue
* Description  : This function changes the real-time clock value.
* Arguments    : counter_write_val -
*                    the expected real-time clock value(BCD code)
* Return Value : status -
*                    MD_OK, MD_BUSY1 or MD_BUSY2
***********************************************************************************************************************/
MD_STATUS R_RTC_Set_CounterValue(rtc_counter_value_t counter_write_val)
{
	MD_STATUS status = MD_OK;
	volatile uint32_t  w_count;
	
	RTCWEN = 1U;
	
	RTCC1 |= _01_RTC_COUNTER_PAUSE;

	/* Change the waiting time according to the system */
	for (w_count = 0U; w_count < RTC_WAITTIME; w_count++)
	{
		__nop();
	}

	if (0U == RWST)
	{
		status = MD_BUSY1;
	}
	else
	{
		SEC = counter_write_val.sec;
		MIN = counter_write_val.min;
		HOUR = counter_write_val.hour;
		WEEK = counter_write_val.week;
		DAY = counter_write_val.day;
		MONTH = counter_write_val.month;
		YEAR = counter_write_val.year;
		RTCC1 &= (uint8_t)~_01_RTC_COUNTER_PAUSE;

		/* Change the waiting time according to the system */
		for (w_count = 0U; w_count < RTC_WAITTIME; w_count++)
		{
			__nop();
		}

		if (1U == RWST)
		{
			status = MD_BUSY2;
		}
	}
	
	RTCWEN = 0;

	return (status);
}

/***********************************************************************************************************************
* Function Name: R_RTC_Set_ConstPeriodInterruptOn
* Description  : This function enables constant-period interrupt.
* Arguments    : period -
*                    the constant period of INTRTC
* Return Value : status -
*                    MD_OK or MD_ARGERROR
***********************************************************************************************************************/
MD_STATUS R_RTC_Set_ConstPeriodInterruptOn(rtc_int_period_t period)
{
	MD_STATUS status = MD_OK;

	RTCWEN = 1U;
	
	if ((period < HALFSEC) || (period > ONEMONTH))
	{
		status = MD_ARGERROR;
	}
	else
	{
		RTCMK = 1U;    /* disable INTRTC */
		RTCC0 = (RTCC0 & _F8_RTC_INTRTC_CLEAR) | period;
		RTCC1 &= (uint8_t)~_08_RTC_INTC_GENERATE_FLAG;
		RTCIF = 0U;    /* clear INTRTC interrupt flag */
		RTCMK = 0U;    /* enable INTRTC interrupt */
	}
	
	RTCWEN = 0;

	return (status);
}

/***********************************************************************************************************************
* Function Name: R_RTC_Set_ConstPeriodInterruptOff
* Description  : This function disables constant-period interrupt.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_RTC_Set_ConstPeriodInterruptOff(void)
{
	RTCC0 &= _F8_RTC_INTRTC_CLEAR;
	RTCC1 &= (uint8_t)~_08_RTC_INTC_GENERATE_FLAG;
	RTCIF = 0U;        /* clear INTRTC interrupt flag */
}
/***********************************************************************************************************************
* Function Name: r_rtc_interrupt
* Description  : This function is INTRTC interrupt service routine.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
static void __near r_rtc_interrupt(void)
{
	if((1U == RIFG))
	{
		RTCWEN = 1U;
		
		RTCC1 &= (uint8_t)~_08_RTC_INTC_GENERATE_FLAG;    /* clear RIFG */
		if(batt_disp_f==0)
		{
			//		RTC_vRead();//R_RTC_Get_CounterValue(&dt);
			//        	r_rtc_callback_constperiod();
		}
		else 
		{
			//		u8WakeupIntruptCntr++;
			//            	if(u8WakeupIntruptCntr>59)
			//            	{
			u8WakeupIntruptCntr=0;
			//RTCC0 |= 0x03;
			MET_bsleep_mode_f=1;
			BattLowCheck=1;
			MET_bten_kva_f=0;
			CSS = 0;
			ClearRaw();
			MET_vSD24Init();
			DSAMK = 0U;//sd24
			//		}
		}
		RTCWEN = 0;
	}
}

static void __near r_rtc_correction_interrupt(void)
{
	u8 i=0;
	FMCEN = 1;
	for(i=0;i<10;i++)
	__nop();
	if(rtc_correction_state == 0x01)                    // correction State on power up RTC Ram INIT
	{
		SUBCUD = (rtc_correction_value & 0x01ff) ;  
		SUBCUD |= 0x8000;
		rtc_calib_f=0;
	}
	else if(rtc_correction_state == 0x02)              // correction State on power up to set default correction 0 ppm
	{
		SUBCUD = (rtc_correction_value & 0x01ff) ;
	}
//	else if(rtc_correction_state == 0x03)             // correction State : to update SUBCUD register with values send by RTC Calibrator
//	{
//		SUBCUD = UT.s16RtcCalSec1 & 0x01ff ;
//		SUBCUD |= 0x8000;
//	}
	FMCEN = 0;
	RTITIF=0;        // clear interrupt flag
	RTITMK=1;        // interrupt service disabled
}
/***********************************************************************************************************************
* Function Name: r_rtc_callback_constperiod
* Description  : This function is real-time clock constant-period interrupt service handler.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
//static void r_rtc_callback_constperiod(void)
//{
//	/* Start user code. Do not edit comment generated here */
//	//	P5.1 ^= 1;
//	/* End user code. Do not edit comment generated here */
//}
/***********************************************************************************************************************
* Function Name: RTC_vRead
* Description  : This function is real-time clock constant-period interrupt service handler.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void RTC_vRead(void)
{	
	rtc_counter_value_t time2;
	u8 time1[7];
	u8 i_rtc;
	u8 var1;
	
	R_RTC_Get_CounterValue(&time2);
	//	time2[0]=RTCSEC;
	//	time2[1]=RTCMIN;
	//	time2[2]=RTCHOUR;
	//	time2[3]=RTCYEARL;
	//	time2[4]=RTCMON;
	//	time2[5]=RTCDAY;
	//	time2[6]=RTCDOW;
	memcpy(&time1,&time2,7);
	for(i_rtc=0;i_rtc<7;i_rtc++)
	{
		var1 = time1[i_rtc];
		if((var1 & 0x0f) > 0x09)
		{
			break;
		}
		var1=var1>>4;
		if(var1 > 0x09)
		{
			break;
		}
	}
	if(i_rtc < 7)
	{
		//        	RTCCTL0_H = RTCKEY_H;                   // Unlock RTC_C module
		//        	RTCCTL0_L =  RTCRDYIE;       // Enable RTC time  interrupt
		//		RTCCTL0=RTCKEY+RTCRDYIE;
		
		if(dt.sec<0x60 && dt.min<0x60 &&  dt.hour<0x24 && (dt.day<=0x31 && dt.day!=0) && (dt.month<=0x12 && dt.month!=0 ))  
		{
			memcpy(&time2,&dt,7);
			//			RTCSEC=dt.sec;
			//			RTCMIN=dt.min;
			//			RTCHOUR=dt.hour;
			//			RTCDAY=dt.day;
			//            		if(dt.week>=7)
			//                		RTCDOW=0;
			//            		else
			//                		RTCDOW=dt.week;
			//			RTCMON=dt.month;
			//			RTCYEARL=dt.year;
			R_RTC_Set_CounterValue(time2);
		}
		else
		{			
			rtcfail_cntr++;
		}
		//        	RTCCTL0_H = 0;                          // Lock RTC_C module
	}
	else
	{
		if(time2.sec<0x60 && time2.min<0x60 &&  time2.hour<0x24 && (time2.day<=0x31 && time2.day!=0) && (time2.month<=0x12 && time2.month!=0 ))  			
		{	//rtcfail_f=0;
			rtcfail_cntr=0;
			
			if(time2.sec!=dt.sec)
			seccng_f=1;
			
			if(time2.min!=dt.min)
			{
				seccng_f=1;
				mincng_f=1;	
			}
			
			if(time2.hour!=dt.hour)
			{
				seccng_f=1;
				mincng_f=1;
				hrscng_f=1;
			}
			
			if(time2.day!=dt.day)
			{
				seccng_f=1;
				mincng_f=1;
				hrscng_f=1;
				daycng_f=1;
			}
			
			if(time2.month!=dt.month)
			{				
				seccng_f=1;
				mincng_f=1;
				hrscng_f=1;
				daycng_f=1;
				monthcng_f=1;	
			}
			
			if(time2.year!=dt.year)
			{				
				seccng_f=1;
				mincng_f=1;
				hrscng_f=1;
				daycng_f=1;
				monthcng_f=1;	
			}
			rtcfail_cntr=0;
			memcpy(&dt,&time2,7);//update_TIvariables();
		}
		else 
		{ 
			rtcfail_cntr++;			
		}
	}
	if(rtcfail_cntr >= 5)
	{
		rtcfail_cntr=0;
       
			  
		time2.sec	= 0x00	;//RTCSEC=0x00;
		time2.min	= 0x00	;//RTCMIN=0x00;
		time2.hour	= 0x09	;//RTCHOUR=0x09;
		time2.year	= 0x00	;//RTCDAY=0x01;
		time2.month	= 0x01	;//RTCDOW=0x06;  //07
		time2.day	= 0x01	;//RTCMON=0x01;
		time2.week	= 0x06	;//RTCYEAR=0x2000;
 	Eprom_Read(PowerDownTimeAdd);//read power down min
    if( ((opr_data[0]<0x60)&&((opr_data[0]&0x0f)<0x0A)))//min
       if((opr_data[1]<0x60)&&((opr_data[1]&0x0f)<0x0A))//hr
           if((opr_data[2]<0xA0)&&((opr_data[2]&0x0f)<0x0A))//year
               if((opr_data[3]<0x20)&&((opr_data[3]&0x0f)<0x0A))//month
                   if((opr_data[4]<0x40)&&((opr_data[4]&0x0f)<0x0A))//day
                      if((opr_data[2]!=0)&&(opr_data[3]!=0)&&(opr_data[4]!=0) )
			  {
				  time2.sec	= 0x00	;
				  time2.min	= opr_data[0]	;
				  time2.hour	= opr_data[1]	;
				  time2.year	= opr_data[2]	;
				  time2.month	= opr_data[3]	;
				  time2.day	= opr_data[4]	;
				  time2.week	= opr_data[5]	;
			  }		
		R_RTC_Set_CounterValue(time2);
		memcpy(&dt,&time2,7);//update_TIvariables();
		seccng_f=1;
		mincng_f=1;
		hrscng_f=1;
		daycng_f=1;
		monthcng_f=1;	
//		if(rtcfail_tprstr_f==0)
//		{
//			rtcfail_tprstr_f=1;
//	        TPR_bDiagnosticsEvent=1;
//	        Diagnostics_count++;
//	        check_tpr();
//		}
		rtc_status_byte=0x01;
		save_rtc_status();
	}
	
	//	if(rtc_status_byte == 1)
	//		a8Daily_alert_status[11]|=DES_rtc_fail;
}
void RTC_vCalib(void)
{
//	u8 u8RtcCalSign;
//	u8 u8RtcCalSec;
//	s16 s16RtcCalSec1;
	static union
	{
		u8 a8RtcCalSec1[2];
		s16 s16RtcCalSec1;
	}UT;
	if(analyse_cal_pkt_flag==1)
	{
		if(RTC_Onetimecal_f == 1)
		{
			return;
		}
		analyse_cal_pkt_flag=0;
		RtcRst_DispFlag=1;

		if(data_array[1]==0x57)
		{
			u8RtcCalSign = data_array[4];
			u8RtcCalSec = data_array[5];

			if((u8RtcCalSign > 1) && (u8RtcCalSec >= 0xc8))
			{
				u8RtcCalSign =0;
				u8RtcCalSec =0;
			}
			//FMCEN = 1;
			
			UT.s16RtcCalSec1 = SUBCUD & 0x01ff;
			if(UT.s16RtcCalSec1 & 0x0100)
			UT.s16RtcCalSec1 |= 0xfe00;
			
			if(u8RtcCalSign == 1)
			{
				UT.s16RtcCalSec1 = UT.s16RtcCalSec1 - u8RtcCalSec;
			}
			else
			{
				UT.s16RtcCalSec1 = UT.s16RtcCalSec1 + u8RtcCalSec;
			}
			//SUBCUD = UT.s16RtcCalSec1 & 0x01ff ;
			//SUBCUD |= 0x8000;
			//FMCEN = 0;
			UT.s16RtcCalSec1 = (UT.s16RtcCalSec1 & 0x01ff)|0x8000;
			//				// Unlock RTC
			//				RTCCTL0_H = RTCKEY_H;
			//				
			//				// Load calibration values in HW RTC
			//				if(u8RtcCalSign == 1)
			//				RTCOCAL_H |= 0x80;
			//				else
			//				RTCOCAL_H &= 0x7f;
			//				
			//				RTCOCAL_L = u16RtcCalSec;
			//				
			//				// Lock RTC
			//				RTCCTL0_H = 0;

			opr_data[0]=  UT.a8RtcCalSec1[1]; 
			opr_data[1] = UT.a8RtcCalSec1[0]; 

			opr_data[2] = data_array[4];
			opr_data[3] = data_array[5];
			Eprom_Write(RTC_Calibration_add);

			RTC_Onetimecal_f=1;
			rtc_correction_state=0x01;
			rtc_correction_f=1;
			//rtc_calib_f=0;
			// RTCCLK_OUT_DIS();
		}
	}
}

void save_rtc_status(void)
{
    Eprom_Read(RTC_StatusByte_Add);
    if(rtc_status_byte!=opr_data[0])  
    {                                                   
        opr_data[0]=rtc_status_byte;
        Eprom_Write(RTC_StatusByte_Add);
    }
}
/* Start user code for adding. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
