/***********************************************************************************************************************
* File Name    : bill.c
* Version      : 
* Device(s)    : R5F10MMG
* Tool-Chain   : CC-RL
* Description  : This file implements device driver for CGC module.
* Creation Date: 
***********************************************************************************************************************/

/***********************************************************************************************************************
Pragma directive
***********************************************************************************************************************/

/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "r_cg_macrodriver.h"
#include "variable.h"
#include "Eprom_i2c.h"
#include "function.h"
#include "meterology.h"
#include "bill.h"
#include "string.h"
#include "loadsurvey.h"
#include "r_cg_wdt.h"
/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/

/***********************************************************************************************************************
* Function Name: BILL_vInit
* Description  : This function initializes the clock generator.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void BILL_vInit(void)
{
	Eprom_Read(TOU_CheckPassiveApli);
	if(EPROM_bChksum_ok == 1)
	{
		if(opr_data[0]==1)
		TOD_bcalendar_change_f=1;
		
	}	
	
	Eprom_Read(NextBillDateAdd);
	if(EPROM_bChksum_ok == 1)
	{
		
		memcpy(BILL_a8date_array,opr_data,5);
		//for(i_m=0;i_m<5;i_m++)
		//BILL_a8date_array[i_m]=opr_data[i_m];
	}
	Eprom_Read(TOU_ActiveCalPtr);
	if(EPROM_bChksum_ok == 1)
	{
		if(opr_data[0] == 1)
		TOD_bActive_Calendar=1;
		else
		TOD_bActive_Calendar=0;
		
	}
	R_WDT_Restart();
	Eprom_Read(TOU_SEA_ACTIVE_STATUS);
	if(EPROM_bChksum_ok == 1)
	{
		if(opr_data[0] == 1)
		TOD_bSeason_f=1;
		else 
		TOD_bSeason_f=0;  		
	}
	TOD_u8vmain0  = TOD_u8zone_index;
	storezkwh();
	TOU_vCheck_Active_Calendar();
	R_WDT_Restart();
	TOU_vDeter_Season();
	
	if((TP3.b.neu_miss_tpr_f==1)&&(test_mode==1))
	TOD_u8zone_index = 0x01;
	
	TOD_u8zmd_index=TOD_u8zone_index;
	TOD_bzonecng_f0=0;
	TOD_vloadzkwh();
	
	Eprom_Read(MD_IP_Add);
	if(EPROM_bChksum_ok == 1)
	{
		md_ip = opr_data[0];
	}
	if((md_ip != 0x1e) &&  (md_ip != 0x0f) && (md_ip != 0x3c))
	md_ip = 0x1e;
	Eprom_Read(CurrentBillDataAdd);
	if(EPROM_bChksum_ok ==1)
	{
		bp_kwh = a8_to_u24(&opr_data[8]);
		bp_kvah = a8_to_u24(&opr_data[12]);
		BILL_u8md_count = opr_data[0];
		if(BILL_u8md_count==0xff)
		BILL_u8md_count=0;
		BILL_u8mdmonth = opr_data[7];
		TOD_u8todmonth= opr_data[11];
	}
	if((BILL_u8mdmonth == 0) || (BILL_u8mdmonth > (MaxBillDate+1)))
	BILL_u8mdmonth = 0x01;
	
	if((TOD_u8todmonth == 0) || (TOD_u8todmonth > MaxBillDate))
	TOD_u8todmonth=0x01;
	
	Eprom_Read(CircularBufferMDLSkwh+(MET_u8CircularPtr - 0x10));//reading power down md
	if(EPROM_bChksum_ok ==1)
	{
		BILL_u16mdkw_c = a8_to_u16(&opr_data[1]);
		BILL_u16mdkva_c = a8_to_u16(&opr_data[3]);
		LS_u16kwh=a8_to_u16(&opr_data[10]);
		LS_u16kvah=a8_to_u16(&opr_data[12]);
		if(md_ip == 0x1e)
		{
			if((dt.year != opr_data[7]) || (dt.month != opr_data[8]) || (dt.day != opr_data[9]) || (dt.hour != opr_data[6]) || ((opr_data[5] < 0x30) && (dt.min >= 0x30) ) )
			{
				TOD_bstore_eoicngmd_f6 = 1;
				TOD_u8zmd_index=opr_data[0];
				mdfun();
			}
		}
		else if(md_ip == 0x0f)
		{
			if((dt.year != opr_data[7]) || (dt.month != opr_data[8]) || (dt.day != opr_data[9]) || (dt.hour != opr_data[6]) || ((opr_data[5] < 0x15) && (dt.min >= 0x15) ) || ((opr_data[5] < 0x30) && (dt.min >= 0x30) ) || ((opr_data[5] < 0x45) && (dt.min >= 0x45)))
			{
				TOD_bstore_eoicngmd_f6 = 1;
				TOD_u8zmd_index=opr_data[0];
				mdfun();
			}
		}

		else if(md_ip == 0x3c)
		{
			if((dt.year != opr_data[7]) || (dt.month != opr_data[8]) || (dt.day != opr_data[9]) || (dt.hour != opr_data[6]))
			{

				TOD_bstore_eoicngmd_f6 = 1;
				TOD_u8zmd_index=opr_data[0];
				mdfun();
			}
		}

	}
	Eprom_Read(CumMDAddr);
	MET_u32Cum_MDKW=a8_to_u32(&opr_data[0]);
	MET_u32Cum_MDKVA=a8_to_u32(&opr_data[4]);
	Eprom_Read(NextBillDateAdd);
	memcpy(BILL_a8date_array,opr_data,5);
	if(BILL_a8date_array[4]==0)
	BILL_a8date_array[4]=1;

	missbill();
	check_bill();
	if((savebill_flag == 1 && dt.day!=0 && dt.month!=0 && dt.month<=0x12) && missbill_f == 1)
	{
		missbill_f=0;
		savebill_flag =0;
		save_billdata();
		mri_bill_f=0;
		//      PUS_u8Bill=1;
		//      Eprom_Read(0x2990);
		//      opr_data[1]=PUS_u8Bill;
		//      Eprom_Write(0x2990);
	}	
}
/***********************************************************
; Save billing parameters
;***********************************************************/
void save_billdata(void)
{
	u16 u16temp_mdkw;
	u16 u16temp_mdkva;
	unsigned char add1,temp_pf;
	
//    calc_bill_poff(0);
	
	BILL_u8md_count++;
	
	
	add1 = calmdpg(BILL_u8mdmonth);
	Eprom_Read(billmd_data_addrs+add1);
	
	
	opr_data[14]=BILL_u8btpr_c;
	add1 = calmdpg(BILL_u8mdmonth);
	u16temp_mdkw=a8_to_u16(&opr_data[0]);
	u16temp_mdkva=a8_to_u16(&opr_data[7]);        
	Eprom_Write(billmd_data_addrs+add1);
	
	fill_oprzero();
	Eprom_Read(NextBillDateAdd);
	BILL_a8date_array[0]=opr_data[0];
	BILL_a8date_array[1]=opr_data[1];
	BILL_a8date_array[4]=opr_data[4];
	if(mri_bill_f==1)
	{
		memcpy(&opr_data[6],&dt.min,5);
	}
	else
	{
		memcpy(&opr_data[6],&opr_data[0],5);
	}
	FUN_vfill_3byteR(MET_u32Cum_kwh,&opr_data[2]);
	FUN_vfill_3byteR(MET_u32Cum_kvah,&opr_data[5]);
	
	opr_data[11] = BILL_u8md_count;
	
	
	FUN_vfill_2byteR(u16_bill_pow_on,&opr_data[13]);
	cal_avg_pf();
	temp_pf=(u8)(MET_u16Avg_pf/4);
	opr_data[14]=temp_pf;
	add1 = calmdpg(BILL_u8mdmonth);
	Eprom_Write(billkwh_data_addrs+add1);
	
	BILL_u8btpr_c=0;
	//pon_reset();
	save_cumtpr();
	save_todbill();
	
	////    WDTCTL = WDT_ARST_1000;
	R_WDT_Restart();
	TOD_u8todmonth++;
	if(TOD_u8todmonth>=(MaxBillDate+1))//0x0d
	TOD_u8todmonth=0x01;
	
	BILL_u8mdmonth++;
	if(BILL_u8mdmonth >=(MaxBillDate+2))//== 14)
	BILL_u8mdmonth = 0x01;
	opr_data[0] = BILL_u8md_count;
	opr_data[1] = mri_bill_f;
	//  time_stamp(&opr_data[2]);
	//for(i_b=0;i_b<5;i_b++)
	// opr_data[2+i_b]=BILL_a8date_array[i_b];
	memcpy(&opr_data[2],&dt.min,5);
	opr_data[7] = BILL_u8mdmonth;
	FUN_vfill_3byteR(MET_u32Cum_kwh,&opr_data[10]);
	FUN_vfill_3byteR(MET_u32Cum_kvah,&opr_data[14]);
	bp_kwh = MET_u32Cum_kwh;
	bp_kvah= MET_u32Cum_kvah;
	opr_data[11]=TOD_u8todmonth;
	Eprom_Write(CurrentBillDataAdd);
	MET_u32Cum_MDKW+=u16temp_mdkw;
	MET_u32Cum_MDKVA+=u16temp_mdkva;
	FUN_vfill_4byteR(MET_u32Cum_MDKW,&opr_data[3]);
	FUN_vfill_4byteR(MET_u32Cum_MDKVA,&opr_data[7]);
	Eprom_Write(CumMDAddr);
	
	fill_oprzero();
	add1 = calmdpg(BILL_u8mdmonth);
	Eprom_Write(billmd_data_addrs+add1);
	Eprom_Write(billkwh_data_addrs+add1);
	BILL_u16mdkw_c = 0;
	BILL_u16mdkva_c= 0;
	MET_u8CircularPtr=0;
//	Eprom_Read(0x0100+MET_u8CircularPtr);
//	for(i_b=1;i_b<5;i_b++)
//	opr_data[i_b]=0;
//	time_stamp(&opr_data[5]);
//	Eprom_Write(0x0100+MET_u8CircularPtr);
	store_energy();
	code_gen();
//  poff_mins=0;
//  u16_bill_pow_off = 0;
	u16_bill_pow_on = 0;
	pon_min_write();
	fill_oprzero();
	Eprom_Write(BILL_POff_minAdd);
	if(u8ChangeCalnder_flag ==1)
	{
		u8ChangeCalnder_flag = 0;
		TOD_bActive_Calendar ^= 0x01;
		fill_oprzero();
		opr_data[0]=TOD_bActive_Calendar;
		Eprom_Write(TOU_ActiveCalPtr);
		opr_data[0]=TOD_bcalendar_change_f=0;
		Eprom_Write(TOU_CheckPassiveApli);
		TOU_vDeter_Season();
		TOD_bzonecng_f0=0;
		storezkwh();
		TOD_vloadzkwh();
	}
	
	//mdrst_f7 = 0;
	missbill_f = 0;
	GetNextDate(dt.min,dt.hour,dt.day,dt.month,dt.year, BILL_a8date_array[0], BILL_a8date_array[1], BILL_a8date_array[4], 0);
}
void GetNextDate(char tpMin,char tpHr,char tpDate, char tpMonth, char tpYear, char tbMin, char tbHr, char tbDate, char tEvenOdd)
{
	unsigned char nbDate=0, nbMonth=0, nbYear=0;

	tpDate=bcd_to_hex(tpDate);
	tpMonth=bcd_to_hex(tpMonth);
	tpYear=bcd_to_hex(tpYear);
	tbDate=bcd_to_hex(tbDate);

	if(tEvenOdd!=0)
	{
		if(tEvenOdd%2!= tpMonth%2)
		{
			nbDate= tbDate;
			nbMonth = tpMonth+1;
			nbYear= tpYear;
			if(nbMonth>12)
			{
				nbYear++;
				nbMonth-=12;
			}
		}
		else
		{

			if(tpDate<tbDate || (tpDate==tbDate && tpHr<tbHr) || (tpDate==tbDate && tpHr==tbHr && tpMin<tbMin))
			{
				nbMonth=tpMonth;
				nbDate= tbDate;
				nbYear= tpYear;
			}
			else
			{
				nbMonth= tpMonth+2;
				nbDate= tbDate;
				nbYear= tpYear;
				if(nbMonth>12)
				{
					nbYear++;
					nbMonth-=12;
				}

			}
		}
	}
	else
	{
		if(tpDate<tbDate || (tpDate==tbDate && tpHr<tbHr) || (tpDate==tbDate && tpHr==tbHr && tpMin<tbMin))
		{
			nbMonth=tpMonth;
			nbDate= tbDate;
			nbYear= tpYear;
		}
		else
		{
			nbMonth= tpMonth+1;

			nbDate= tbDate;
			nbYear= tpYear;
			if(nbMonth>12)
			{
				nbYear++;
				nbMonth-=12;
			}

		}

	}
	Eprom_Read(NextBillDateAdd);

	fill_oprzero();
	BILL_a8date_array[4]=opr_data[4]=hex_to_bcd(nbDate);
	BILL_a8date_array[3]=opr_data[3]=hex_to_bcd(nbMonth);
	BILL_a8date_array[2]=opr_data[2]=hex_to_bcd(nbYear);
	BILL_a8date_array[1]=opr_data[1]=(tbHr);
	BILL_a8date_array[0]=opr_data[0]=(tbMin);


	Eprom_Write(NextBillDateAdd);

}

void pon_min_write(void)
{
	fill_oprzero();   
	FUN_vfill_4byteR(cum_pow_on,&opr_data[3]);
	FUN_vfill_2byteR(pow_on,&opr_data[5]);
	FUN_vfill_2byteR(u16_bill_pow_on,&opr_data[7]);
	
	time_stamp(&opr_data[9]);
	Eprom_Write(CircularBufferMDLSkwh+(pon_ptr));
	pon_ptr += 0x10;
}

void md_function(unsigned char type)
{
	unsigned int temp_int,temp_int1;
	unsigned char temp1;//,i_md;
	unsigned char time_array_md[5];

	Eprom_Read(CircularBufferMDLSkwh+(MET_u8CircularPtr-16));
	memcpy(time_array_md,&opr_data[5],5);

	if(type==1)
	{
		temp1=calmdpg(TOD_u8zmd_index);
		Eprom_Read(Zone_MD_energy_data_Add+temp1);
	}
	else
	{
		temp1=calmdpg(BILL_u8mdmonth);
		Eprom_Read(billmd_data_addrs+temp1);
	}

	if(EPROM_bChksum_ok ==1)
	{
		temp_int=a8_to_u16(&opr_data[0]);
		temp_int1=a8_to_u16(&opr_data[7]);
		if(BILL_u16mdkw_c > temp_int || BILL_u16mdkva_c >temp_int1)
		goto lebel2;

	}
	else
	{
lebel2:
		if(TOD_bstore_eoicngmd_f6 == 0)
		{
			if(BILL_u16mdkw_c > temp_int )
			{
				time_stamp(&opr_data[2]);
				FUN_vfill_2byteR(BILL_u16mdkw_c ,&opr_data[1]);
			}
			if(BILL_u16mdkva_c >temp_int1)
			{
				time_stamp(&opr_data[9]);
				FUN_vfill_2byteR(BILL_u16mdkva_c,&opr_data[8]);
			}
		}
		else
		{
			//ls_timestamp(md_ip);
			if(BILL_u16mdkw_c > temp_int )
			{
				FUN_vfill_2byteR(BILL_u16mdkw_c ,&opr_data[1]);
				//				memcpy(&opr_data[2],time_array_md,5);
				opr_data[2]=var5;
				opr_data[3]=var4;
				opr_data[4]=var1;
				opr_data[5]=var2;
				opr_data[6]=var3;
			}
			if(BILL_u16mdkva_c >temp_int1)
			{
				FUN_vfill_2byteR(BILL_u16mdkva_c ,&opr_data[8]);
				//				memcpy(&opr_data[9],time_array_md,5);
				opr_data[9]=var5;
				opr_data[10]=var4;
				opr_data[11]=var1;
				opr_data[12]=var2;
				opr_data[13]=var3;
			}
		}

		if(type==1)
		{
			Eprom_Write(Zone_MD_energy_data_Add+temp1);
		}
		else
		{
			Eprom_Write(billmd_data_addrs+temp1);//
		}
	}       
}
void mdfun(void)
{

	if(md_ip == 0x1e || md_ip == 0x0f)
	{
		BILL_u16mdkw_c +=BILL_u16mdkw_c;
		BILL_u16mdkva_c +=BILL_u16mdkva_c;
	}

	if(md_ip == 0x0f)
	{
		BILL_u16mdkw_c+=BILL_u16mdkw_c;
		BILL_u16mdkva_c+=BILL_u16mdkva_c;
	}


	if(BILL_u16mdkw_c > BILL_u16mdkva_c)
	BILL_u16mdkva_c=BILL_u16mdkw_c;

	ls_timestamp(md_ip);
	//	write_dmd(BILL_u16mdkw_c);
	md_function(1);   // for zmd
	md_function(2);

	BILL_u16mdkw_c = 0;
	BILL_u16mdkva_c = 0;
	TOD_bstore_eoicngmd_f6 = 0;
	MET_u8CircularPtr = 0;
	store_energy();

}
void missbill(void)
{
	Eprom_Read(CurrentBillDataAdd);    // read from last bill date
	if(EPROM_bChksum_ok ==1)
	{
		if((opr_data[4] != dt.year) || (opr_data[5] != dt.month) || (opr_data[6] != dt.day) || (opr_data[3] != dt.hour) || (opr_data[2] != dt.min))
		{
			missbill_f = 1;
		}
	}
}
void check_bill(void)
{
	Eprom_Read(NextBillDateAdd);
	if(dt.year>opr_data[2])
	{
		savebill_flag=1;
	}
	else if(dt.month>opr_data[3] && dt.year==opr_data[2])
	{
		savebill_flag=1;
	}
	else if(dt.day>opr_data[4] && dt.month==opr_data[3] && dt.year==opr_data[2])
	{
		savebill_flag=1;
	}
	else if(dt.hour>BILL_a8date_array[1] && dt.day==opr_data[4] && dt.month==opr_data[3] && dt.year==opr_data[2])
	{
		savebill_flag=1;
	}
	else if(dt.min>=BILL_a8date_array[0] && dt.hour==BILL_a8date_array[1] && dt.day==opr_data[4] && dt.month==opr_data[3] && dt.year==opr_data[2])
	{
		savebill_flag=1;
	}
}
void save_cumtpr(void)
{

	FUN_vfill_2byteR(TPR_u16cum_tpr_c,&opr_data[1]);
	opr_data[2]=BILL_u8btpr_c;

	Eprom_Write(cum_tpr_addrs);//
}
void save_todbill(void)
{
	//	unsigned long int tkwh,tkvah;
	//	u8 add1;
	u8 j;
	//    u8 add2;
	//    u8 temp_todmnth;

	TOD_u8vmain0=TOD_u8zone_index;
	storezkwh();

	tariff_cnt=0;
	R_WDT_Restart();
	do
	{
		Eprom_Read(Zone_energy_data_Add+(tariff_cnt<<4));
		Eprom_Write(billtod_data_addrs+(0x0080*(TOD_u8todmonth-1))+(tariff_cnt<<4));
//		Eprom_Read(Zone_MD_energy_data_Add+(tariff_cnt<<4));
//		Eprom_Write(0x0980+(0x0100*(TOD_u8todmonth-1))+(tariff_cnt<<4));
		//		tkwh=a8_to_u24(&opr_data[0]);
		//		tkvah=a8_to_u24(&opr_data[3]);
		//
		//		Eprom_Read(Zone_MD_energy_data_Add+(tariff_cnt<<4));
		//
		//		FUN_vfill_3byteR(tkwh,&opr_data[9]);
		//		FUN_vfill_3byteR(tkvah,&opr_data[12]);


		//		if(TOD_u8todmonth % 2 == 0)
		//		{
		//			temp_todmnth=TOD_u8todmonth-1;
		//			add1=caltodblk(temp_todmnth);
		//			add2=0x80+(tariff_cnt<<4);
		//		}
		//		else if(TOD_u8todmonth % 2 != 0)
		//		{
		//			add1=caltodblk(TOD_u8todmonth);
		//			add2=tariff_cnt<<4;
		//		}
		//
		//		Eprom_Write((add1*0x100)+add2);
		//        Eprom_Write(billtod_data_addrs+
		//		tkwh=0;
		//		tkvah=0;
		tariff_cnt=tariff_cnt+1;

	}while(tariff_cnt < 8 );

	tariff_cnt=0;
 
	R_WDT_Restart();
	fill_oprzero();
	for(j=8;j<16;j++)
	Eprom_Write(Zone_energy_data_Add+(j*16));   // block 02 is reset

}

void TOD_vloadzkwh(void)
{
	TOD_u8vmain0 = TOD_u8zone_index-1;
	TOD_u8vmain0 = TOD_u8vmain0 << 4;
	
	Eprom_Read(Zone_energy_data_Add+TOD_u8vmain0);	
	
	TOD_u32cum_zkwh = 0;
	TOD_u32cum_zkvah = 0;

	if(EPROM_bChksum_ok ==1)
	{
		TOD_u32cum_zkwh= a8_to_u24(&opr_data[0]);
		TOD_u32cum_zkvah= a8_to_u24(&opr_data[3]);
	}

	if(TOD_bzonecng_f0==1)
	{
		MET_u8CircularPtr = 0;
		store_energy();
	}
}
//void calc_bill_poff(u8 temp_current_bill)
//{    
//	u16 add1;
//	u8 prvs_mnth, prvs_year, prvs_date, prvs_hr, prvs_min;
//	u16 prv_temp, prst_temp, temp_days, ttl_min;
//	u16 temp_prv_min, temp_prst_min, temp_min_diff;
//	u8 temp_mdmonth;
//	
//	
//	if(BILL_u8md_count == 0)
//	{
//		Eprom_Read(FG_DateTimeAdd);
//		prvs_min = 0;//opr_data[0];
//		prvs_hr = 0;//opr_data[1];
//		prvs_year = opr_data[2];
//		prvs_mnth = opr_data[3];
//		prvs_date = opr_data[4];
//	}
//	else 
//	{
//		temp_mdmonth = BILL_u8mdmonth-1;
//		if(temp_mdmonth==0)
//		temp_mdmonth = MaxBillDate+1;
//		
//		add1 = calmdpg(temp_mdmonth);
//		Eprom_Read(billkwh_data_addrs+add1);
//		
//		prvs_min = opr_data[6];
//		prvs_hr = opr_data[7];
//		prvs_year = opr_data[8];
//		prvs_mnth = opr_data[9];
//		prvs_date = opr_data[10];
//	}
//	
//	if((mri_bill_f!=1) && (temp_current_bill == 0))
//	{
//		Eprom_Read(NextBillDateAdd);
//	}
//	else
//	{
//		opr_data[0] = dt.min;
//		opr_data[1] = dt.hour;
//		opr_data[2] = dt.year;
//		opr_data[3] = dt.month;
//		opr_data[4] = dt.day;
//	}
//	prv_temp = sel_datediff(prvs_date,prvs_mnth,prvs_year);
//	prst_temp = sel_datediff(opr_data[4],opr_data[3],opr_data[2]);
//	temp_days = prst_temp - prv_temp;
//	temp_prv_min = ((u16)bcd_to_hex(prvs_hr) * 60) + bcd_to_hex(prvs_min);
//	temp_prst_min = ((u16)bcd_to_hex(opr_data[1]) * 60) + bcd_to_hex(opr_data[0]);
//	
//	if(temp_prv_min > temp_prst_min)
//	{
//		temp_days--;
//		temp_min_diff =(24*60) - (temp_prv_min - temp_prst_min);
//	}
//	else
//	temp_min_diff = temp_prst_min - temp_prv_min;
//	
//	
//	
//	ttl_min = (temp_days*1440) + temp_min_diff;
//	//    if(u16_bill_pow_on < ttl_min)
//	//    {
//	//        u16_bill_pow_off = ttl_min - u16_bill_pow_on; 
//	//    }
//	//    else
//	//    {
//	//        u16_bill_pow_off = 0;
//	//    }
//}
void TOU_vCheck_Active_Calendar(void)
{
	unsigned int m1;

	TOD_bActive_f=0;

	Eprom_Read(TOU_PassiveApliDate);//calendar chnge date

	TOD_bActive_f=TOD_bActive_Calendar;

	m1=0;

	if(dt.year>opr_data[0])
	{
		m1=1;
	}
	else if(dt.year==opr_data[0] && dt.month>opr_data[1]  )
	{
		m1=1;
	}
	else if( dt.year==opr_data[0] && dt.month==opr_data[1] && dt.day>opr_data[2] )
	{
		m1=1;
	}
	else if(dt.year==opr_data[0] && dt.month==opr_data[1] && dt.day==opr_data[2] && dt.hour>opr_data[3]   )
	{
		m1=1;
	}
	else if( dt.year==opr_data[0] && dt.month==opr_data[1] && dt.day==opr_data[2] && dt.hour==opr_data[3] && dt.min>=opr_data[4])
	{
		m1=1;
	}



	if((TOD_bcalendar_change_f==1)&& m1==1)
	{
		u8ChangeCalnder_flag = 1;
		mri_bill_f = 1;             //change of calander is performed after bill is generated. its been updated in the bill.

		//		TOD_bActive_Calendar ^= 0x01;
		//        fill_oprzero();
		//		opr_data[0]=TOD_bActive_Calendar;
		//		Eprom_Write(TOU_ActiveCalPtr);
		//		opr_data[0]=TOD_bcalendar_change_f=0;
		//		Eprom_Write(TOU_CheckPassiveApli);
	}
}
void TOU_vDeter_Season(void)
{
	u8 u8NoOffSeason;

	Eprom_Read(TOU_CAL_ACTIVE_ADD+0x10+0x40*TOD_bActive_Calendar);
	u8NoOffSeason=opr_data[0];
	if(TOD_bActive_Calendar == 0)
	{
		Eprom_Read(TOU_CAL_ACTIVE_ADD+0x20);
	}
	else if(TOD_bActive_Calendar == 1)
	{
		Eprom_Read(TOU_CAL_ACTIVE_ADD+0x60);
	}

	TOD_bSeason_f=0;
	if(u8NoOffSeason!=1)
	{
		if((dt.month > opr_data[3]) || ((dt.month == opr_data[3]) && (dt.day >= opr_data[4])))
		{
			if((dt.month > opr_data[8]) || ((dt.month == opr_data[8]) && (dt.day >= opr_data[9])))
			{
				TOD_bSeason_f=1;
				Eprom_Read(TOU_CAL_ACTIVE_ADD+0x30+0x40*TOD_bActive_Calendar);
				memcpy(Week_Name,opr_data+7,7);
			}
			else
			{
				Eprom_Read(TOU_CAL_ACTIVE_ADD+0x30+0x40*TOD_bActive_Calendar);
				memcpy(Week_Name,opr_data,7);
			}
		}
		else
		{
			TOD_bSeason_f=1;
			Eprom_Read(TOU_CAL_ACTIVE_ADD+0x30+0x40*TOD_bActive_Calendar);
			memcpy(Week_Name,opr_data+7,7);
		}
	}
	else
	{
		Eprom_Read(TOU_CAL_ACTIVE_ADD+0x30+0x40*TOD_bActive_Calendar);
		memcpy(Week_Name,opr_data,7);
	}

	opr_data[0]=TOD_bSeason_f;
	Eprom_Write(TOU_SEA_ACTIVE_STATUS);
	R_WDT_Restart();
	deter_week();
}
void deter_week(void)
{
	u8 u8day_id;
	u8 pointer;

	Eprom_Read(TOU_WEEK_ACTIVE_ADD+0x40*TOD_bActive_Calendar);
	for(pointer = 0; pointer < 7; ++pointer)
	{
		if(opr_data[pointer]!=Week_Name[pointer])
		break;
	}
	if(pointer==7)
	{
		Eprom_Read(TOU_WEEK_ACTIVE_ADD+0X10+0x40*TOD_bActive_Calendar);
		u8day_id=opr_data[dt.week];
		Eprom_Read(TOU_DAY_ACTIVE_ADD+0x60*TOD_bActive_Calendar);
		if(opr_data[14]==u8day_id)
		{
			memcpy(tou_a8traiff,opr_data,9);
			Eprom_Read(TOU_DAY_ACTIVE_ADD+0x10+0x60*TOD_bActive_Calendar);
			memcpy(tou_a8zone_time,opr_data,14);
			Eprom_Read(TOU_DAY_ACTIVE_ADD+0x20+0x60*TOD_bActive_Calendar);
			memcpy(tou_a8zone_time+14,opr_data,2);
		}
		else
		{
			Eprom_Read(TOU_DAY_ACTIVE_ADD+0x30+0x60*TOD_bActive_Calendar);
			if(opr_data[14]==u8day_id)
			{
				memcpy(tou_a8traiff,opr_data,9);
				Eprom_Read(TOU_DAY_ACTIVE_ADD+0x40+0x60*TOD_bActive_Calendar);
				memcpy(tou_a8zone_time,opr_data,14);
				Eprom_Read(TOU_DAY_ACTIVE_ADD+0x50+0x60*TOD_bActive_Calendar);
				memcpy(tou_a8zone_time+14,opr_data,2);
			}
		}
	}
	else
	{
		Eprom_Read(TOU_WEEK_ACTIVE_ADD+0x20+0x40*TOD_bActive_Calendar);
		for(pointer = 0; pointer < 7; ++pointer)
		{
			if(opr_data[pointer]!=Week_Name[pointer])
			break;
		}
		if(pointer==7)
		{
			Eprom_Read(TOU_WEEK_ACTIVE_ADD+0x30+0x40*TOD_bActive_Calendar);
			u8day_id=opr_data[dt.week];
			Eprom_Read(TOU_DAY_ACTIVE_ADD+0x00+0x60*TOD_bActive_Calendar);
			if(opr_data[14]==u8day_id)
			{
				memcpy(tou_a8traiff,opr_data,9);
				Eprom_Read(TOU_DAY_ACTIVE_ADD+0x10+0x60*TOD_bActive_Calendar);
				memcpy(tou_a8zone_time,opr_data,14);
				Eprom_Read(TOU_DAY_ACTIVE_ADD+0x20+0x60*TOD_bActive_Calendar);
				memcpy(tou_a8zone_time+14,opr_data,2);
			}
			else
			{
				Eprom_Read(TOU_DAY_ACTIVE_ADD+0x30+0x60*TOD_bActive_Calendar);
				if(opr_data[14]==u8day_id)
				{
					memcpy(tou_a8traiff,opr_data,9);
					Eprom_Read(TOU_DAY_ACTIVE_ADD+0x40+0x60*TOD_bActive_Calendar);
					memcpy(tou_a8zone_time,opr_data,14);
					Eprom_Read(TOU_DAY_ACTIVE_ADD+0x50+0x60*TOD_bActive_Calendar);
					memcpy(tou_a8zone_time+14,opr_data,2);
				}
			}
		}
	}


	R_WDT_Restart();
	deter_zone();
}
void deter_zone(void)
{
	unsigned char vmain1,i,j;

	TOD_u8vmain0 = TOD_u8zone_index;
	TOD_u8zone_index = 1;

	if((tou_a8traiff[0] != 0)) 
	{
		vmain1 = tou_a8traiff[0];

		j=0;
		for(i=0;i<vmain1;i++)
		{
			if(*(tou_a8zone_time + j)>(dt.hour))
			{
				if(TOD_u8zone_index==0x01)
				TOD_u8zone_index=vmain1;
				else
				TOD_u8zone_index--;
				break;
			}
			if(*(tou_a8zone_time + j)<(dt.hour))
			{
				if(TOD_u8zone_index!=vmain1)
				{
					j +=2;
					TOD_u8zone_index++;
				}
			}
			else if(*(tou_a8zone_time + j)==(dt.hour))
			{
				j++;
				if(*(tou_a8zone_time + j)>(dt.min))
				{
					if(TOD_u8zone_index==0x01)
					{
						TOD_u8zone_index=vmain1;

					}
					else TOD_u8zone_index--;
					break;
				}
				else
				{
					if(TOD_u8zone_index!=vmain1)
					{
						j++;
						TOD_u8zone_index++;
					}
				}
			}
		}

		TOD_u8zone_index=*(tou_a8traiff + TOD_u8zone_index);

		zone_default_f0=0;
		if(TOD_u8zone_index!=TOD_u8vmain0)
		TOD_bzonecng_f0=1;

	}
	else
	{
		zone_default();
	}
}

void storezkwh(void)
{
	fill_oprzero();
	FUN_vfill_3byteR(TOD_u32cum_zkwh,&opr_data[2]);
	FUN_vfill_3byteR(TOD_u32cum_zkvah,&opr_data[5]);	
	TOD_u8vmain0=calmdpg(TOD_u8vmain0);
	Eprom_Write(Zone_energy_data_Add+TOD_u8vmain0);	  
	
}

unsigned int read_todadd(unsigned char bmonth1)
{
	u16 address;
	sc8 temp1;
	//    u8 temp_todmnth1;
	//    u8 add1;
	//    u8 add2;



	temp1=TOD_u8todmonth-bmonth1;
	if(temp1<=0)
	temp1+=MaxBillDate;
	//temp1--;

	//	if((temp1 % 2 == 0) && (temp1 != 0))
	//		{
	//			temp_todmnth1=temp1-1;
	//			add1=caltodblk(temp_todmnth1);
	//		    add2=0x80;//+(tariff_cnt<<4);
	//
	//		}
	//
	//		else if((temp1 % 2 != 0) || (temp1 == 0))
	//		{
	//			add1=caltodblk(temp1);
	//			add2=0x00;//tariff_cnt<<4;
	//		}

	address=billtod_data_addrs+((temp1-1)*0x080);//+add2;

	return(address);
}

void zone_default(void)
{
	zone_default_f0=1;
	TOD_u8zone_index=0x01;   //default zone 1
	if(TOD_u8zone_index!=TOD_u8vmain0)
	TOD_bzonecng_f0=1;

	//	active_th_f=0;//normal threshold

}


/* Start user code for adding. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
