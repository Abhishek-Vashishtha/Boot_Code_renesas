/***********************************************************************************************************************
* File Name    : function.c
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
#include "function.h"
#include "Eprom_i2c.h"
#include "lcd.h"
#include "loadsurvey.h"
#include "bill.h"
#include "r_cg_wdt.h"
#include "dlms.h"
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
void fill_oprzero(void)
{
	unsigned char i;
	for(i=0;i<16;i++)
        	opr_data[i] = 0;
//	memzero(opr_data,16);
}	
void FUN_vfill_2byte(unsigned  int vks,unsigned char * ptr1)//fill_2data1
{
	chngtemp.u16_temp[0] = vks;
        *ptr1++ = chngtemp.u8_temp[0]; 
        *ptr1 = chngtemp.u8_temp[1]; 
}
void FUN_vfill_2byteR(unsigned  int vks,unsigned char * ptr1)//fill_2data
{
        chngtemp.u16_temp[0] = vks;
        *ptr1-- = chngtemp.u8_temp[0]; 
        *ptr1 = chngtemp.u8_temp[1]; 
}
void FUN_vfill_3byte(unsigned long int vks,unsigned char * ptr1)
{
	chngtemp.u32_temp = vks;
        *ptr1++ = chngtemp.u8_temp[0];
        *ptr1++ = chngtemp.u8_temp[1];
        *ptr1 =   chngtemp.u8_temp[2];
}
void FUN_vfill_3byteR(unsigned long int vks,unsigned char * ptr1)//fill_data
{
         chngtemp.u32_temp = vks;
        *ptr1-- = chngtemp.u8_temp[0]; 
        *ptr1-- = chngtemp.u8_temp[1]; 
        *ptr1   = chngtemp.u8_temp[2];
}


void FUN_vfill_4byte(unsigned long int vks,unsigned char * ptr1)//fill_data2
{
    chngtemp.u32_temp = vks;
    *ptr1++ = chngtemp.u8_temp[0];
    *ptr1++ = chngtemp.u8_temp[1];
    *ptr1++ = chngtemp.u8_temp[2];
    *ptr1   = chngtemp.u8_temp[3];
}
void FUN_vfill_4byteR(unsigned long int vks,unsigned char * ptr1)//fill_4data
{
        chngtemp.u32_temp = vks;
        *ptr1-- = chngtemp.u8_temp[0]; 
        *ptr1-- = chngtemp.u8_temp[1]; 
        *ptr1-- = chngtemp.u8_temp[2];
        *ptr1   = chngtemp.u8_temp[3];
}

/*-----------------------------------------------------------------
Function : To change char array of 4 bytes into long variable
Input    : Base address of char array [MS byte]
Output   : Equivalent long variable in long_int variable
*/

unsigned long int a8_to_u24(unsigned char *ptr1)
{
	chngtemp.u32_temp=0;
	chngtemp.u8_temp[2] = *ptr1++;
        chngtemp.u8_temp[1] = *ptr1++;
        chngtemp.u8_temp[0] = *ptr1;
	return(chngtemp.u32_temp);
//	unsigned char dumy1;	
//	unsigned long int long_int=0;
//	long_int = 0;
//	long_int = *(char_array_ptr);
//	long_int = long_int << 8;
//	dumy1 = *(char_array_ptr+1);
//	long_int = long_int | dumy1;
//	long_int = long_int << 8;
//	dumy1 = *(char_array_ptr+2);
//	long_int = long_int | dumy1;
//	return(long_int);
}
unsigned long int a8_to_u32(unsigned char *ptr1)
{
	chngtemp.u32_temp=0;
	chngtemp.u8_temp[3] = *ptr1++;
        chngtemp.u8_temp[2] = *ptr1++;
        chngtemp.u8_temp[1] = *ptr1++;
	chngtemp.u8_temp[0] = *ptr1;
	return(chngtemp.u32_temp);
//	unsigned char dumy1;
//	long_int = 0;
//	long_int = *(char_array_ptr);
//	long_int = long_int << 8;
//	dumy1 = *(char_array_ptr+1);
//	long_int = long_int | dumy1;
//	long_int = long_int << 8;
//	dumy1 = *(char_array_ptr+2);
//	long_int = long_int | dumy1;
//	long_int = long_int << 8;
//	dumy1 = *(char_array_ptr+3);
//	long_int = long_int | dumy1;
//	return(long_int);
}
unsigned int a8_to_u16(unsigned char *ptr1)
{
	chngtemp.u16_temp[0]=0;
	chngtemp.u8_temp[1] = *ptr1++;
        chngtemp.u8_temp[0] = *ptr1;
	return(chngtemp.u16_temp[0]);

//	unsigned char dumy1;
//	long_int=0;
//
//	long_int = *(char_array_ptr);
//	long_int = long_int << 8;
//	dumy1 = *(char_array_ptr+1);
//	long_int = long_int | dumy1;
//	return(long_int);
}
void bin_2_dec(void)
{
 	unsigned char i=8;
	unsigned char a=0;
	union change32 chngtemp0;
	chngtemp0.u32_temp = 0;
	chngtemp0.u8_temp[0] = opr_data[0];
	chngtemp0.u8_temp[1] = opr_data[1];
	chngtemp0.u8_temp[2] = opr_data[2];
	chngtemp0.u8_temp[3] = opr_data[3];

//	opr_data[0]=0;
//	opr_data[1]=0;
//	opr_data[2]=0;
//	opr_data[3]=0;
//	opr_data[4]=0;
//	opr_data[5]=0;

//  asm("clr.w    &opr_data");         // LSB, clear output
//  asm("clr.w    &opr_data+2");
//  asm("clr.w    &opr_data+4");       // MSB
	do{
		opr_data[a]=(u8)(chngtemp0.u32_temp%10);
		chngtemp0.u32_temp = chngtemp0.u32_temp/10;
		a++;
//    asm("rla.w    &opr_data+6");     // LSB
//    asm("rlc.w    &opr_data+8");     // MSB
//    asm("dadd.w   &opr_data ,&opr_data");
//    asm("dadd.w   &opr_data+2,&opr_data+2");
//    asm("dadd.w   &opr_data+4,&opr_data+4");
    i--;
  }while(i!=0);



}

//unsigned char calmdpg1(unsigned char mdmth)
//{
//  mdmth--;
//  return(mdmth<<4);
//}
unsigned char calmdpg(unsigned char mdmth)
{
  mdmth--;
  return(mdmth<<4);
}
void read_add(unsigned int blk,unsigned char bmonth1)
{

   signed char temp1;
   unsigned char add;
   temp1 =BILL_u8mdmonth-bmonth1;
   if(temp1<=0)
     temp1+=MaxBillDate+1;
   temp1--;

   temp1=temp1<<4;
   add=temp1;

    Eprom_Read(blk+add);
}

//unsigned char bcd_to_hex(unsigned char temp)
//{
//	return ((temp%0x10)+((temp/0x10)*0x0a));
//}
unsigned char bcd_to_hex(unsigned char temp)
{
    if(temp<=0x99)
        return ((temp%0x10)+((temp/0x10)*0x0a));
    else
        return temp;
}
//unsigned char hex_to_bcd2(unsigned char temp)
//{
//	return ((temp%0x0a)+((temp/0x0a)*0x10));
//}
unsigned char hex_to_bcd(unsigned char temp)
{
	return((temp/10)*16+(temp%10));
}


/**********************************************************
; Loads the time stamp
;**********************************************************/

void time_stamp(unsigned char *addr)
{

	*addr++ = dt.min;
	*addr++ = dt.hour;
	*addr++ = dt.year;
	*addr++ = dt.month;
	*addr = dt.day;
}
void fill_ff(unsigned char count1)
{
	unsigned char i_f;

	for(i_f=count1;i_f<15;i_f++)
	{
		opr_data[i_f]=0xff;

	}
}
int  isdatevalid(int month, int day, int year)
{
	if(!(year >= 2005 && year <= 2079)) return 0;
	if (day <= 0) return 0 ;
	switch( month )
	{
	  case 1  :
	  case 3  :
	  case 5  :
	  case 7  :
	  case 8  :
	  case 10 :
	  case 12 : if (day > 31) return 0 ; else return 1 ;
	  case 4  :
	  case 6  :
	  case 9  :
	  case 11 : if (day > 30) return 0 ; else return 1 ;
	  case 2  :
		if ( day > 29 ) return 0 ;
		if ( day < 29 ) return 1 ;
		if (isleapyear(year)) return 1 ;   // leap year
		else return 0 ;
	}
	return 0 ;
}
unsigned int sel_datediff(unsigned char p_date,unsigned char p_month,unsigned char p_year)//,unsigned char f_date,unsigned char f_month,unsigned char f_year)
{
    unsigned char i;
    unsigned char d2,m2,y2;
    unsigned int eldays=0;

    d2= bcd_to_hex(p_date);
    m2=bcd_to_hex(p_month);
    y2=bcd_to_hex(p_year);
    eldays = (u16)d2 + monthdays1[m2-1] + (u16)y2 * 365;
    for(i=0;i<y2;i++)
    {
		if(i%4==0)
        {
			eldays +=1;
			i +=3;
        }
    }
    if(y2%4==0 && m2>2)
		eldays +=1;
    return(eldays);
}
unsigned char chksumsr(unsigned char *chkdata,unsigned char nob)
{
  unsigned char i,chksum=0;
  
  for(i=0;i<nob;i++)
    chksum += *chkdata++;
    chksum = chksum ^ 0xff;
    chksum += 1;
    return(chksum);
}
void memzero(u8 *s1, u16 n)
{       /* copy char s2[n] to s1[n] in any order */
//	char *su1 = (char *)s1;
	for (; 0 < n; ++s1, --n)
		*s1 = 0;
}
unsigned int days_diff(void)
{
	unsigned int temp=0x0000;
	switch((bcd_to_hex(opr_data[3]))-1)
	{
	  case 11:
		temp +=0x1e;
	  case 10:
		temp +=0x1f;
	  case 9:
		temp +=0x1e;
	  case 8:
		temp +=0x1f;
	  case 7:
		temp +=0x1f;
	  case 6:
		temp +=0x1e;
	  case 5:
		temp +=0x1f;
	  case 4:
		temp +=0x1e;
	  case 3:
		temp +=0x1f;
	  case 2:
		if((bcd_to_hex(opr_data[2]))%4==0x00)
			temp +=0x1d;
		else
			temp +=0x1c;
	  case 1:
		temp +=0x1f;
		break;
	  default:
		break;
	}
	temp += bcd_to_hex(opr_data[4]);
	return(temp);
}
void time_diff(void)
{
	unsigned int d1,d2;
	d2=0x0000;

	if(EPROM_bChksum_ok ==1)
	{
		d1=days_diff();
		if(dt.year != opr_data[2])
		{
			if((bcd_to_hex(opr_data[2]))%4==0x00)
				d2=366;
			else
				d2=365;
		}
		opr_data[4]=dt.day;
		opr_data[3]=dt.month;
		opr_data[2]=dt.year;
		d2 += days_diff();
		bat_on_secs =(unsigned long)(d2-d1)*1440;
		if(dt.hour>opr_data[1])
			bat_on_secs += (unsigned long)(bcd_to_hex(dt.hour)-bcd_to_hex(opr_data[1]))*60;
		else
			bat_on_secs -= (unsigned long)(bcd_to_hex(opr_data[1])-bcd_to_hex(dt.hour))*60;
		if(dt.min>opr_data[0])
			bat_on_secs += (unsigned long)(bcd_to_hex(dt.min)-bcd_to_hex(opr_data[0]));
		else
			bat_on_secs -= (unsigned long)(bcd_to_hex(opr_data[0])-bcd_to_hex(dt.min));
	}
}

void reset_meter_data_new(void)
{
	unsigned char i,i_m;
    u8 flagoptical;
    
    flagoptical=optical_f;
	fill_oprzero();
	lcd_disp_off();
//	LCD_vDispChar(LCD_f,6);///f
//	LCD_vDispChar(LCD_g,5);///g
	LCD_vDispValue(9,4);//COUNTER
    
    /*************  ENERGIES    ****************************************/
    
    ClearEprom(0x0000,48);                                   // EPROM block from 00 to 48 pages i.e from 00 - 0300
    
    Eprom_Write(kw_pulse_c_addrs);                          //kw_pulse_cntr location is cleared
    Eprom_Write(BILL_POff_minAdd);                          // bill power off minutes is cleared
    ClearEprom(billmd_data_addrs,13);                         //bill data block is cleared
    LCD_vDispValue(8,4);//COUNTER
    ClearEprom(billkwh_data_addrs,13);                       //MD data block is cleared
    ClearEprom(billtod_data_addrs,96);                        //TOD Billing reset is cleared
    
    /******************  temper    *******************************/      
    reset_tamper();
    
    /***************************  code adress, power off mins etc   ***************************************/
	ClearEprom(NeutralTestCountAdd,1);
    ClearEprom(CodeAdd,1);                              //CodeAdd
    ClearEprom(cum_poff_addrs,1);                       //cum_poff_addrs
    ClearEprom(rms_avg_addrs,1);                        //rms_avg_addrs

	
    /**************************** daily energy    *********************/        
    LCD_vDispValue(7,4);//COUNTER 
    Eprom_Write(DailyEnergyStatus);                               //DailyEnergyStatus
    Eprom_Write(DailyMDAdd);                                    //DailyMDAdd
	Eprom_Write(DailyOffCount);                                 //DailyOffCount
    ClearEprom(dloadsurvey_init_add,max_dloadsurvey*2);        //daily load Survey 
    /**********************load survey ******************************/      
    LCD_vDispValue(6,4);//COUNTER
    fill_oprzero();
    Eprom_Write(load_survey_status);//load_survey_status
    LS.b.ls_of=0;
//    ClearEprom(loadsurvey_init_add,max_loadsurvey);       // load survey//0xff
    load_survey_cnt=0;
	LS.b.ls_fg_f=1;
	var5=0;
	var4=0;
	LS.b.ls_rtc_fill=0;
	LS.b.ls_rev_fill=0;
	ls_miss_fill();
	day_counter_ls=0;
	for(i=0;i<MaxDarraySize;i++)
		d_array[i]=0;
    ClearEprom(DARRAY_ADD,22);
	d_array[day_counter_ls]=sel_datediff(dt.day,dt.month,dt.year);////*/*/////// seting defaulta values
	fill_darray();
    /****************************************************************/       
    LCD_vDispValue(5,4);//COUNTER
    ClearEprom(FirstStrAddr,1);
    ClearEprom(FirstRestrAddr,1);
    ClearEprom(LastStrAddr,1);
    ClearEprom(LastRestrAddr,1);
    ClearEprom(CumMDAddr,1);
    ClearEprom(TPRCNT_Addr,1);
    ClearEprom(0x04e0,1);        // magnet defraud values.
	ClearEprom(BatteryStatus,1);
	
    ClearEprom(0x15e0,2);         // unused pages of block of threshold settings.
    day_counter_ls=0;
//    pow_on=0;
//    u16_bill_pow_off = 0;
//    u16_bill_pow_on = 0;
	PWR_u32cum_poff_min=0;
	PWR_u16poff_mins=0;
	MET_u32Cum_kwh = 0;
	TPR_u32cum_mag_defraud=0;
	TPR_u32cum_neutemp_defraud=0;
	MET_u32Cum_kvah = 0;
	TOD_u32cum_zkvah = 0;
	TOD_u32cum_zkwh = 0;
	TOD_u32cum_zkvah = 0;
	MET_u32dkvah = 0;
	BILL_u16mdkw_c = 0;
	BILL_u16mdkva_c = 0;
	MET_u8CircularPtr = 0;
	MET_u16kw_pulse_cntr=0;
	MET_u16kva_pulse_cntr=0;
	MET_u32Cum_MDKW=0;
	MET_u32Cum_MDKVA=0;
	LS_u16kwh=0;
	LS_u16kvah=0;
	zkvah_rollover_f=0;
	kvah_rollover_f=0;
	 ClearEprom(kvah_zkvah_rollover_f_add,1);
	TP5.b.tc_tpr_f=0;
    
//	adhoc_f=0;
	flag3.all=0;
	flag9.all=0;
	flag10.all=0;
//	flag14.all=0;
//	flag15.all=0;
//	flag16.all=0;
//	flag17.all=0;
//	flag18.all=0;
	TPR_u16cum_tpr_c = 0;
	BILL_u8btpr_c=0;
	volt_count=0;
	curr_count=0;
//	tpc_count=0;
	on_off_cnt=0;
	others_count=0;
	trans_count=0;
//	volt_loc=0;
//	curr_loc=0;
//	trans_loc=0;
//	others_loc=0;
//	mag_loc=0;
//	mag_count=0;
//	mag_of=0;
//	control_count=0;
//	conct_loc=0;
	nonroll_count=0;
//    poff_mins=0;
//    TPR_bFirstStrTpr=0;
//    TPR_bFirstRestrTpr=0;
    /********************* current bill default data***************/        
	LCD_vDispValue(4,4);//COUNTER
	BILL_u8mdmonth = 1;
	TOD_u8todmonth=1;
	bp_kwh = 0;
	bp_kvah = 0;
	BILL_u8md_count = 0;
	opr_data[0] = 0;
	opr_data[1] = 0;
	time_stamp(&opr_data[2]);
	opr_data[7] = 1;//mdmonth;
	opr_data[11] =1;// todmonth;
    
    
	Eprom_Write(CurrentBillDataAdd);// last bill
    /******************  next bill data  ***************************************/
    //		LCDMEM[7]=0xfe;///3
    LCD_vDispValue(3,4);//COUNTER
    for(i_m=0;i_m<5;i_m++)
        BILL_a8date_array[i_m]=0;
    
	fill_oprzero();
	opr_data[4]=1;
	BILL_a8date_array[4]=1;
	Eprom_Write(NextBillDateAdd);
    
    //	LCDMEM[7]=LCD_Tab[6];//COUNTER
    /***************************************************************************/
//	pon_reset();
	////////////writing default parameters
    
	UART_vResetDlmsData();
    
    LCD_vDispValue(2,4);//COUNTER
    
	R_WDT_Restart();//	WDTCTL = WDT_ARST_1000;
    
    
    
    
    
	daily_enrcount=0;
    //	fill_oprzero();
	
    //	fill_oprzero();
	
//	PUS_u8Audit = 0;
//	PUS_u8Bill = 0;
//	PUS_u8Alert = 0;
//	PUS_u8Comm_check = 0;
//	PUS_u8Comm_connect = 0;
    pon_reset();
    
	fill_oprzero();
	opr_data[0]=sizeof(DefaultCurrentRating) - 1;
	memcpy(opr_data+1,DefaultCurrentRating,opr_data[0]);
    Eprom_Write(CurrentRating_addrs);
	
	fill_oprzero();
	opr_data[0] = sizeof(UtilituID) - 1;
	memcpy(opr_data+1,UtilituID,opr_data[0]);
	Eprom_Write(0x06E0);
	
    /************************************ FG ***********************************/        
	LCD_vDispValue(1,4);//COUNTER
    time_stamp(&opr_data[0]);
	opr_data[5]=1;
	Eprom_Write(FG_DateTimeAdd);//FG date n time
    Eprom_Write(PowerDownTimeAdd);
    opr_data[0]=0;
    opr_data[1]=0;
//    time_diff();
//	pow_on+=bat_on_secs;
//    u16_bill_pow_on += bat_on_secs;
//    cum_pow_on += bat_on_secs;
//    pon_min_write();
//    Eprom_Read(PowerDownTimeAdd);
////    fill_2data(pow_on,&opr_data[6]);
//    Eprom_Write(PowerDownTimeAdd);
	fg_done_f=1;
    
    
    /*****************************************************************************************/
//	dls_count=0;
    LCD_vDispValue(0,4);//COUNTER
    //	LCDMEM[7]=0x7c;///10x06
    
    //	Eprom_Write(0x29A0);
    //	pus_u8AlertArrayCntr=0;
    //	memzero(pus_a16AlertArray,14);
    
	lcd_disp_off();
    optical_f=flagoptical;
}
void ClearEprom(unsigned int from, unsigned int count)
{
    u16 i;
    u8 temp=9;

    LCD_vDispValue(temp,3);//COUNTER
    fill_oprzero();
    
    for (i=0; i<count; ++i)
    {
        Eprom_Write(from);
        from+=0x10;
		R_WDT_Restart();//        WDTCTL = WDT_ARST_1000;
        LCD_vDispValue(temp--,3);//COUNTERLCDMEM[6]=LCD_Tab[temp--];  
        glow_2p();
        if(temp>9)
            temp=9;
    }
}
void reset_tamper(void)
{
//    LCDMEM[7]=LCD_Tab[7];
    ClearEprom(volt_init_add,volt_max_loc);                  //temper data... 10 blocks, is cleared
    ClearEprom(curr_init_add,curr_max_loc);
    ClearEprom(pwr_init_add,pwr_max_loc);
    ClearEprom(trans_init_add,trans_max_loc);
    ClearEprom(other_init_add,other_max_loc);
    ClearEprom(control_init_add,control_max_loc);
    ClearEprom(nonroll_init_add,nonroll_max_loc);
    ClearEprom(Diagnostics_init_add,Diagnostics_max_loc);
    Eprom_Write(cum_poff_addrs);
    Eprom_Write(volt_tamp_status);                             //voltage related
	R_WDT_Restart();//WDTCTL = WDT_ARST_1000; 
	Eprom_Write(curr_tamp_status);                           // current related
    Eprom_Write(pwr_tamp_status);                            //on_off_event
	Eprom_Write(trans_tamp_status);                           //trans
	Eprom_Write(other_tamp_status);                           //other events
	Eprom_Write(nonroll_status);                             //nonroll cover related
	Eprom_Write(control_tamp_status);                       // control evnts
    Eprom_Write(Diagnostics_status);                          // Diagnostics_status
    ClearEprom(cum_tpr_addrs,1);                              //cum_tpr_addrs
    ClearEprom(pwr_tamp_status,1);                           //pwr_tamp_status
    TPRCNT_u8NeuMiss    =0;
    TPRCNT_u8VHigh      =0;
    TPRCNT_u8VLow       =0;
    TPRCNT_u8Rev        =0;
    TPRCNT_u8EL         =0;
    TPRCNT_u8OC         =0;
    TPRCNT_u8OverLoad   =0;
    TPRCNT_u8Mag        =0;
    TPRCNT_u8NeuDis     =0;
    TPRCNT_u8LowPF      =0;
    TPRCNT_u835kv       =0;
    TPRCNT_u8FreqTamp   =0;
    TPRCNT_u8TC         =0;
    
    R_WDT_Restart();//WDTCTL = WDT_ARST_1000;
	TP.all = 0;
	TP1.all = 0;
	TP2.all = 0;
	TP3.all = 0;
	TP4.all = 0;
	TP5.all = 0;
}

void code_gen(void)
{
// 	unsigned char main12,main13,swaprnd1;
//	u8 *ch_ptr;
//      ch_ptr = (unsigned char*)&MET_u16v_rms;
//      opr_data[8] = *ch_ptr++;        //lsb
//      opr_data[9] = *ch_ptr;
//
//      fill_data1(cum_kwh,&opr_data[10]);
//      ch_ptr = (unsigned char*)&freq;
//      opr_data[13] = *ch_ptr++;        //lsb
//      opr_data[14] = *ch_ptr;
//
//      opr_data[0]=opr_data[8] ^ opr_data[10];
//
//      main12=opr_data[8] & 0x0f;
//      main13=opr_data[8] & 0xf0;
//      main12=main12<<4;
//      main13=main13>>4;
//      swaprnd1=main12 | main13;
//      opr_data[6]=swaprnd1 ^ opr_data[12];
//
//      opr_data[4]=opr_data[13] ^ btpr_c;
//
//      main12=opr_data[13] & 0x0f;
//      main13=opr_data[13] & 0xf0;
//      main12=main12<<4;
//      main13=main13>>4;
//      swaprnd1=main12 | main13;
//      opr_data[2]=swaprnd1 ^ opr_data[11];
//
//      opr_data[3]=opr_data[8];
//      opr_data[5]=opr_data[13];
//
//      opr_data[7]=opr_data[13] ^ opr_data[3];
//
//      opr_data[1]=0;
//
//      opr_data[1] = chksumsr(opr_data,8);
//
//      Eprom_Write(CodeAdd);

}
void pon_reset(void)
{
  unsigned char j;
  
   fill_oprzero();
   for(j=0;j<16;j++)                   // Power on hours
   {
     Eprom_Write(CircularBufferMDLSkwh+(j*16));
   }
   pow_on = 0;
   cum_pow_on = 0;
   u16_bill_pow_on = 0;
   pon_ptr=0;
   
}
unsigned long int sqrt_function(unsigned char* ptr)
{

	unsigned long int x_data,y_data,h_data;
	unsigned int x_data1,y_data1,h_data1;
	unsigned char i;
	union change32 temp;
	temp.u8_temp[0]= *(ptr);
	temp.u8_temp[1]= *(ptr+1);
	temp.u8_temp[2]= *(ptr+2);
	temp.u8_temp[3]= *(ptr+3);
	h_data = temp.u32_temp;
	temp.u8_temp[0]= *(ptr+4);
	temp.u8_temp[1]= *(ptr+5);
	h_data1 = temp.u16_temp[0];
	x_data = y_data =0;
	x_data1 = y_data1 =0;
	for(i=0;i<48;i++)
	{
		x_data1 <<= 1;
		if(x_data & 0x80000000)
		x_data1++;	  
		x_data <<= 1;	
		x_data++;	  
		if((y_data1 < x_data1)||((y_data1 == x_data1)&&(y_data < x_data)))
		{
			if(x_data<2)
				x_data1--;
			x_data -= 2;
//			if(x_data & 0x80000000)
//				x_data1--;			
		}
		else
		{
			if(y_data<x_data)
			y_data1--;
			y_data1 -= x_data1;
			y_data -= x_data;
		}
		if(x_data == 0xffffffff)
		x_data1++;	  
		x_data++;
		y_data1<<= 1;
		if(y_data & 0x80000000)
		y_data1++;
		y_data <<= 1;
		if(h_data1 & 0x8000)		// Check for Minus sign
		{
			if(y_data == 0xffffffff)
				y_data1++;
			y_data++;
		}
		y_data1<<= 1;
		if(y_data & 0x80000000)
		y_data1++;
		y_data <<= 1;
		h_data1<<= 1;
		if(h_data & 0x80000000)
		h_data1++;
		h_data <<= 1;
		if(h_data1 & 0x8000)
		{
			if(y_data == 0xffffffff)
				y_data1++;
			y_data++;
		}
		h_data1<<= 1;
		if(h_data & 0x80000000)
		h_data1++;
		h_data <<= 1;				  		  
	}
	temp.u32_temp = x_data;
	temp.u16_temp[0] = x_data1;
	temp.u8_temp[2] = temp.u8_temp[1];
	temp.u8_temp[1] = temp.u8_temp[0];
	temp.u8_temp[0] = temp.u8_temp[3];
	temp.u8_temp[3] = 0;
	return temp.u32_temp; 

}

void delay_1p3s(void)
{
  unsigned long int i;
  for(i=0;i<40000;i++);//WDTCTL =WDTPW+WDTCNTCL ;
  
}

u32 Compute_EnergyHighres(u8 choice)
{
	u32 temp_hireso;
	
	if(choice==1)
	{
		temp_hireso = (MET_u32Cum_kwh%10000);
		temp_hireso = (temp_hireso*1000) + ((MET_u16kw_pulse_cntr*(unsigned long int)3125)/1000) ;
	}
	else if(choice==2)
	{
		temp_hireso = (MET_u32Cum_kvah%10000);
		temp_hireso = (temp_hireso*1000) + ((MET_u16kva_pulse_cntr*(unsigned long int)3125)/1000) ;
	}
	
	return temp_hireso;
}

u32 CalCumMD(u8 MdType)
{
	u16 temp_md=0;
	u32 temp_CumMd=0;
	
	fill_oprzero();
	
	read_add(billmd_data_addrs,1);
	
	if(MdType==0)
	{
	  	temp_md=a8_to_u16(&opr_data[0]);
		temp_CumMd = MET_u32Cum_MDKW - (u32)(temp_md);
	}
	else if(MdType==1)
	{
		temp_md=a8_to_u16(&opr_data[7]);
		temp_CumMd = MET_u32Cum_MDKVA - (u32)(temp_md);
	}
	
	return temp_CumMd;
}

/* Start user code for adding. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
