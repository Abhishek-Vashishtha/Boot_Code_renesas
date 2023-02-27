/***********************************************************************************************************************
* File Name    : dailyenergy.c
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
#include "dailyenergy.h"
#include "string.h"
/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/

/***********************************************************************************************************************
* Function Name: fill_oprzero
* Description  : This function initializes the clock generator.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void DE_vInit(void)
{
	Eprom_Read(DailyEnergyStatus);
	if(EPROM_bChksum_ok == 1)
	{
		daily_enrcount=opr_data[0];
		DE.b.dls_of=opr_data[5];
	}
	Eprom_Read(DailyOffCount);
	if(EPROM_bChksum_ok == 1)
	{
		daily_on_off=opr_data[0];
	}
	Eprom_Read(PowerDownTimeAdd);//read power down min
	if((dt.year != opr_data[2]) || (dt.month != opr_data[3]) || (dt.day != opr_data[4]))
	{
		DE.b.miss_dailyenr = 1;
		DE_vdaily_energy_save();
	}
}
//void write_dmd(unsigned int daily_md)
//{
//  unsigned int dtemp;
//  Eprom_Read(DailyMDAdd);
//  if(EPROM_bChksum_ok == 1)
//  {
//    dtemp=a8_to_u16(&opr_data[0]);
//    if(daily_md > dtemp)
//    {
//      FUN_vfill_2byteR(daily_md,&opr_data[1]);
//      time_stamp(&opr_data[2]);
//      Eprom_Write(DailyMDAdd);
//    }
// }
//}
void DE_vdaily_energy_save(void)
{
 unsigned char i,array[2];
    unsigned int temp_j,daddress,u16temp0,u16temp1;//,u16daily_offmin;
//    update_daily_alert();
//	if(EPROM_bChksum_ok == 1)
//	{
		if(daily_enrcount>=maxday_counter_l) 
		{
			daily_enrcount=0;
			DE.b.dls_of=1;
		}
//	}
//	else
//	{
//		daily_enrcount=0;
//	}
	Eprom_Read(PowerDownTimeAdd);//read power down min
	temp1_var=opr_data[2];//year
	temp2_var=opr_data[3];//month
	temp3_var=opr_data[4];//day
	temp4_var=opr_data[1]; //hour
	temp5_var=opr_data[0];//min


	daddress=dloadsurvey_init_add+((u16)daily_enrcount * 0x20);

	Eprom_Read(dloadsurvey_init_add+(u16)(daily_enrcount-1)*0x20);
	u16temp0=sel_datediff(dt.day,dt.month,dt.year);
	u16temp1=sel_datediff(opr_data[4],opr_data[3],opr_data[2]);
	if(u16temp0!=u16temp1)
	{
		Eprom_Read(DailyMDAdd);
		temp_j=a8_to_u16(&opr_data[0]);
		
		memcpy(array,&opr_data[2],2);
		
		if(DE.b.miss_dailyenr == 1)
		{
			temp3_var=(bcd_to_hex(temp3_var)+1);
			if(((temp2_var==2) && ((bcd_to_hex(temp1_var)%4)==0) && (temp3_var>29))
			||((temp3_var>(monthdays[(bcd_to_hex(temp2_var))-1]))&&((temp2_var!=2) || ((bcd_to_hex(temp1_var)%4)!=0))))
			{
				temp3_var=1;
				temp2_var=hex_to_bcd(bcd_to_hex(temp2_var)+1);
				if(temp2_var>0x12)
				{
					temp2_var=1;
					temp1_var=hex_to_bcd(bcd_to_hex(temp1_var)+1);
				}
			}
			
			temp3_var=hex_to_bcd(temp3_var);
			opr_data[2]=temp1_var;
			opr_data[3]=temp2_var;
			opr_data[4]=temp3_var;

		}
		else
		{
			opr_data[2]=dt.year;
			opr_data[3]=dt.month;
			opr_data[4]=dt.day;
		}
		
		opr_data[0]=0x00;//time of 
		opr_data[1]=0x00;
		
		FUN_vfill_3byteR(MET_u32Cum_kwh,&opr_data[7]);
		FUN_vfill_3byteR(MET_u32Cum_kvah,&opr_data[10]);     
		FUN_vfill_2byteR(temp_j,&opr_data[12]);
		
		memcpy(&opr_data[13],array,2);
		
		Eprom_Write(daddress);
		
		opr_data[0]=daily_on_off;
//		opr_data[1]=daily_lon_off;
//		opr_data[2]=daily_son_off;
//		memcpy(opr_data+3,a8Daily_alert_status,12);		
		
		Eprom_Write(daddress+0x10);
		fill_oprzero();
		
        daily_enrcount+=0x01;


		
		opr_data[0]=daily_enrcount;
		opr_data[2]=dt.year;
		opr_data[3]=dt.month;
		opr_data[4]=dt.day;
		opr_data[5]=DE.b.dls_of;
		for(i=6;i<15;i++)
			opr_data[i]=0;
		
		Eprom_Write(DailyEnergyStatus);
//        memzero(a8Daily_alert_status,12);
//		update_daily_alert();
//		if(DE.b.miss_dailyenr == 1)
//		{
//			a8Daily_alert_status[2]|=DES_power_outageR;
//		}

		DE.b.miss_dailyenr=0;
		fill_oprzero();
		Eprom_Write(DailyMDAdd);
		Eprom_Write(DailyOffCount);
		daily_on_off=0;
//		daily_son_off=0;
//		daily_lon_off=0;

	}
}
/* Start user code for adding. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
