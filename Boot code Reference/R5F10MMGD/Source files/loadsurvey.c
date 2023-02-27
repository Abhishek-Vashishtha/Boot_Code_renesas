/***********************************************************************************************************************
* File Name    : loadsurvey.c
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
#include "loadsurvey.h"
#include "bill.h"
#include "Eprom_i2c.h"
#include "function.h"
#include "meterology.h"
#include "r_cg_wdt.h"
/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/

/***********************************************************************************************************************
* Function Name: fill_oprzero
* Description  : This function initializes the clock generator.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void LS_vInit(void)
{
	u8 temp1_f;
	Eprom_Read(rms_avg_addrs);
	if(EPROM_bChksum_ok ==1)
	{
		i_rms_avg = a8_to_u32(&opr_data[0]);
		v_rms_avg = a8_to_u32(&opr_data[4]);
		//        u16RSSI_avg = char_array_to_int(&opr_data[8]);
		cal_avg_cntr=a8_to_u16(&opr_data[12]);
	}
	Eprom_Read(load_survey_status);
	if(EPROM_bChksum_ok == 1)
	{
		day_counter_ls = opr_data[10];
		LS.b.ls_of=opr_data[2];
	}

	Eprom_Read(LS_IP_Add);
	if(EPROM_bChksum_ok == 1)
	{
		ls_ip = opr_data[0];
	}

	if(ls_ip != 0x1e &&  ls_ip != 0x0f && ls_ip != 0x3c)
		ls_ip = 0x1e;

    maxday_counter_l=(max_loadsurvey/(24*(60/ls_ip)));
	
	if(maxday_counter_l>MaxDarraySize)
	{
		maxday_counter_l=MaxDarraySize;
	}
	
    read_darray();
	load_ls_cnt();
	temp1_f=0;
	Eprom_Read(PowerDownTimeAdd);//read power down min
	temp1_var=opr_data[2];//year
	temp2_var=opr_data[3];//month
	temp3_var=opr_data[4];//day
	temp4_var=opr_data[1]; //hour
	temp5_var=opr_data[0];//min
	if(opr_data[2]==dt.year)
	{
		if(opr_data[3]==dt.month)
		{
			if(opr_data[4]==dt.day)
			{
				if(opr_data[1]==dt.hour)
				{
					if(opr_data[0] != dt.min)
					{
						if(((bcd_to_hex(dt.min))/ls_ip) > ((bcd_to_hex(opr_data[0]))/ls_ip))
						{
							LS.b.miss_load_survey_f=1;
							load_survey();
							ls_miss_fill();
							load_ls_cnt();
						}
					}
				}
				else
				{
					temp1_f=1;
				}
			}
			else
			{
				temp1_f=1;
				LS.b.ls_daycng_f=1;
			}
		}
		else
		{
			temp1_f=1;
			LS.b.ls_daycng_f=1;
		}
	}
	else
	{
		temp1_f=1;
		LS.b.ls_daycng_f=1;
	}

	if(temp1_f==1)
	{
		temp1_f=0;
		LS.b.miss_load_survey_f=1;
		load_survey();
		R_WDT_Restart();
		ls_miss_fill();
		if(LS.b.ls_daycng_f)
		{
			if(d_array[day_counter_ls]!=sel_datediff(dt.day,dt.month,dt.year))
			{
				day_counter_ls++;
				if(day_counter_ls > (maxday_counter_l-1))
				{
					day_counter_ls=0;
				}

				d_array[day_counter_ls]=sel_datediff(dt.day,dt.month,dt.year);
				fill_darray();
				Eprom_Read(load_survey_status);
				opr_data[10]= day_counter_ls;
				Eprom_Write(load_survey_status);
				load_ls_cnt();
			}
		}
	}
}

void load_ls_cnt(void)
{
	u8 u8temp_hr,u8temp_min;

	u8temp_hr=bcd_to_hex(dt.hour);
	u8temp_hr=u8temp_hr*(60/ls_ip);
	u8temp_min=bcd_to_hex(dt.min);
	u8temp_min=u8temp_min/ls_ip;
    load_survey_cnt=(u16)((u16)24*(60/ls_ip)*(u16)(day_counter_ls))+u8temp_hr+u8temp_min-1;
	if((day_counter_ls==0)&&(u8temp_hr==0)&&(u8temp_min==0))
	{
		load_survey_cnt=0;
	}
	save_loadsurvey_cnt();
}

void fill_darray(void)
{
	u16 address;
	u16 i_darray;

    i_darray = day_counter_ls/7;
	
    address = i_darray <<4;
	i_darray=day_counter_ls%7;
	Eprom_Read(DARRAY_ADD+address);
	FUN_vfill_2byteR(d_array[day_counter_ls],&opr_data[(2*i_darray)+1]);
	Eprom_Write(DARRAY_ADD+address);
}


void read_darray(void)
{
	u16 ird;
	u16 jrd,krd;
	
	R_WDT_Restart();
	for(ird=0x00,jrd=0;ird<0x120;ird+=0x10)
	{
		Eprom_Read(DARRAY_ADD+ird);
		for(krd=0;(krd<14 && jrd < maxday_counter_l);krd+=2)
		{
			d_array[jrd]=a8_to_u16(&opr_data[krd]);
			jrd+=1;
		}
		R_WDT_Restart();
	}
}

void next_miss_date(u8 date, u8 month, u8 year)
{
	ls_date=(bcd_to_hex(date)+1);
	ls_month=bcd_to_hex(month);
	ls_year=bcd_to_hex(year);
	if((ls_month==2 && ls_year%4==0 && ls_date>29)||((ls_date>(monthdays[ls_month-1]))&&(ls_month!=2 || ls_year%4!=0)))
	{
		ls_date=1;
		ls_month+=1; /* =hex_to_bcd(bcd_to_hex(var2)+1); */
		if(ls_month>12) /* 0x12) */
		{
			ls_month=1;
			ls_year+=1; /* =hex_to_bcd(bcd_to_hex(var1)+1); */
		}
	}

	ls_date=hex_to_bcd(ls_date);
	ls_month=hex_to_bcd(ls_month);
	ls_year=hex_to_bcd(ls_year);
}

void ls_miss_fill(void)
{
    u16 n,m1;
    u16 ls_cnt_start_1day,ls_cnt_end_1day,temp_mdi=0,address;

    if(LS.b.ls_rev_fill==1)
    {
		var4=0;
		var5=0;
		load_survey_cnt=(u16)((u16)24*(60/ls_ip)*(u16)(day_counter_ls));//d_cnt_tmp
    }

    if((dt.day!=var3 || dt.month!=var2 || dt.year!=var1)&& LS.b.ls_fg_f==0)
	{
        ls_cnt_start_1day=2+(u16)((u16)bcd_to_hex(temp4_var)*60+bcd_to_hex(temp5_var))/(ls_ip)-LS.b.ls_rtc_fill;
        ls_cnt_end_1day=24*(60/ls_ip)+(u16)((u16)bcd_to_hex(dt.hour)*60+bcd_to_hex(dt.min))/(ls_ip);
	}
    else
	{
        ls_cnt_start_1day=1+(u16)((u16)bcd_to_hex(var4)*60+bcd_to_hex(var5))/(ls_ip)-LS.b.ls_fg_f-LS.b.ls_rtc_fill+LS.b.ls_rev_fill;
        ls_cnt_end_1day=(u16)((u16)bcd_to_hex(dt.hour)*(u16)60+bcd_to_hex(dt.min))/(ls_ip);
	}

    load_survey_cnt=load_survey_cnt+1-LS.b.ls_fg_f;
	
    if(LS.b.ls_fg_f==0)
	{
		temp_mdi=(u16)bcd_to_hex(var4)*(u16)60+(bcd_to_hex(var5)+(u16)(ls_ip)*(1-LS.b.ls_rtc_fill));
	}

    if(temp_mdi>=1440)
	{
		temp_mdi=0;
	}
	if(LS.b.ls_fg_f==1)
	{
		temp_mdi=ls_ip;
	}

	//  for(n=0;n<16;n++)
	//{opr_data[n]=0;}
	fill_oprzero();

  	for(n=ls_cnt_start_1day;n<=ls_cnt_end_1day;n++)
	{
		if(load_survey_cnt>=max_loadsurvey)
		{
			load_survey_cnt=0;
			LS.b.ls_of=1;
		}

		opr_data[0]=hex_to_bcd(temp_mdi%60);//main11;
		opr_data[1]=hex_to_bcd(temp_mdi/60);//main11;

		temp_mdi+=ls_ip;
        if(temp_mdi>=1440)
		{
			temp_mdi=0;
		}

   		if(n>=(24*(60/ls_ip)) || LS.b.ls_fg_f==1)
		{
			if(n==(24*(60/ls_ip)))
			{
				if((var4==0) && (var5==0))
				{
				 	opr_data[4]=var3;
				 	opr_data[3]=var2;
				 	opr_data[2]=var1;
				}
				else
				{
					next_miss_date(var3, var2, var1);
					opr_data[4]=ls_date;
				 	opr_data[3]=ls_month;
					opr_data[2]=ls_year;
				}
			}
			else
			{
			opr_data[4]=dt.day;
			opr_data[3]=dt.month;
			opr_data[2]=dt.year;
			}
			if(LS.b.ls_rtc_fill==1)
			{
			    DE.b.miss_dailyenr=1;
			}
		}
		else
		{
			opr_data[4]=var3;
			opr_data[3]=var2;
			opr_data[2]=var1;
		}

        for(m1=5;m1<16;m1++)
		{
			opr_data[m1]=0;
		}

		address=loadsurvey_init_add+((load_survey_cnt)*0x10); // <=2047
		//MEM2_256_f=1;
		
		Eprom_Write(address);
		R_WDT_Restart();
		load_survey_cnt++;
	}

	if((n!=ls_cnt_start_1day))//&&(LS.b.ls_fg_f == 0))
	{
		load_survey_cnt--;
	}

	load_ls_cnt();
	LS.b.ls_fg_f=0;
}


void load_survey(void)
{
	u8 u8temp_hr,u8temp_min;
	unsigned int  address;

	if(LS.b.miss_load_survey_f==1)
	{
		ls_timestamp(ls_ip);

  		u8temp_hr=bcd_to_hex(temp4_var);  // to avoid overwriting on previous day 00:00 location on date change as day counter increases after this function call.
  		u8temp_hr*=(60/ls_ip);
  		u8temp_min=bcd_to_hex(temp5_var);
  		u8temp_min=u8temp_min/ls_ip;
  	}
	else
	{
		var5=dt.min;
		var4=dt.hour;
		var3=dt.day;
		var2=dt.month;
		var1=dt.year;
	}

	if(LS.b.miss_load_survey_f == 0)
	{
		u8temp_hr=((bcd_to_hex(var4))*(60/ls_ip));
		u8temp_min=bcd_to_hex(var5);
		u8temp_min=u8temp_min/ls_ip;
	}
	
	load_survey_cnt=(u16)((u16)24*(60/ls_ip)*(u16)(day_counter_ls))+u8temp_hr+u8temp_min+LS.b.miss_load_survey_f-1;

	if(load_survey_cnt>=(max_loadsurvey))
	{
        save_loadsurvey_cnt();
    }

	if(cal_avg_cntr)
	{
		v_rms_avg = (unsigned long int)(v_rms_avg/(unsigned long int)cal_avg_cntr);
		i_rms_avg= (unsigned long int)(i_rms_avg/(unsigned long int)cal_avg_cntr);
	}

	address=loadsurvey_init_add+(load_survey_cnt*0x10);
	//MEM2_256_f=1;

	opr_data[0]=var5;
	opr_data[1]=var4;
	opr_data[2]=var1;
	opr_data[3]=var2;
	opr_data[4]=var3;
	
	if(LS_u16kvah<LS_u16kwh)
	{
        LS_u16kvah=LS_u16kwh;
	}

	FUN_vfill_2byteR(LS_u16kwh,&opr_data[6]);
	FUN_vfill_2byteR(LS_u16kvah,&opr_data[8]);
	FUN_vfill_2byteR(v_rms_avg,&opr_data[10]);
	i_rms_avg=(unsigned int)(i_rms_avg/(unsigned long int)10);
	FUN_vfill_2byteR(i_rms_avg,&opr_data[12]);
//	opr_data[13]=RLY_bRelayStatus;
//	opr_data[14]=(u8)u16RSSI_avg;
	Eprom_Write(address);
//	load_survey_cnt++;
//	save_loadsurvey_cnt();
	cal_avg_cntr=0;
	i_rms_avg=0;
	v_rms_avg=0;
	LS_u16kwh =0;
	LS_u16kvah=0;
//	u16RSSI_avg=0;
    fill_oprzero();
	Eprom_Write(rms_avg_addrs);
	LS.b.miss_load_survey_f=0;
//	relay_event_LS_f=0;
	MET_u8CircularPtr = 0;            //to update LS_u16kwh and kvah for miss load survey
	store_energy();
}

void save_loadsurvey_cnt(void)
{
	if(load_survey_cnt>=(max_loadsurvey))
	{
		load_survey_cnt=0;
		LS.b.ls_of=1;
//		if(LS.b.miss_load_survey_f == 1)
//		{
			day_counter_ls=0;
			d_array[day_counter_ls]=sel_datediff(dt.day,dt.month,dt.year);
			fill_darray();
			Eprom_Read(load_survey_status);
			opr_data[10]= day_counter_ls;
			Eprom_Write(load_survey_status);
//		}
	}
	Eprom_Read(load_survey_status);
	FUN_vfill_2byteR(load_survey_cnt,&opr_data[1]);
	opr_data[2] = LS.b.ls_of;
	Eprom_Write(load_survey_status);
}

void ls_timestamp(unsigned char ls_ip1)
{

	Eprom_Read(PowerDownTimeAdd);//read power down min
	var1=opr_data[2];//year
	var2=opr_data[3];//month
	var3=opr_data[4];//day
	var4=opr_data[1]; //hour
	var5=opr_data[0];//min

	if(LS.b.ls_rtc_fill==1)
    {
		temp4_var=var4;
		temp5_var=var5;
    }

	var5=bcd_to_hex(var5);
	var5=var5+((ls_ip1));

	if(var5<60)
	{
		var5=var5-(var5%(ls_ip1));    //(60/ls_ip1)*(temp5_var/(60/ls_ip1));
		var5=hex_to_bcd(var5);
	}
	else
	{
		var5=0;
		var4=hex_to_bcd(bcd_to_hex(var4)+1);
		if(var4>=0x24)
		{
			var4=0;
			var3=(bcd_to_hex(var3)+1);
			if((var2==2 && bcd_to_hex(var1)%4==0 && var3>29)||((var3>(monthdays[(bcd_to_hex(var2))-1]))&&(var2!=2 || bcd_to_hex(var1)%4!=0)))
			{
                var3=1;
				var2=hex_to_bcd(bcd_to_hex(var2)+1);
				if(var2>0x12)
				{
					var2=1;
					var1=hex_to_bcd(bcd_to_hex(var1)+1);
				}
			}
			var3=hex_to_bcd(var3);
		}
	}
}

void LS_vValueReset(void)
{
    cal_avg_cntr=0;
    i_rms_avg=0;
    v_rms_avg=0;
    LS_u16kwh =0;
    LS_u16kvah=0;
    fill_oprzero();
    Eprom_Write(rms_avg_addrs);
}

