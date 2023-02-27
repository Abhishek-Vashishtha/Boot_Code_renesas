
/***********************************************************************************************************************
* File Name    : r_cg_lcd.c
* Version      : CodeGenerator for RL78/L12 E1.00.00c [23 Mar 2012]
* Device(s)    : R5F10MMG
* Tool-Chain   : CC-RL
* Description  : This file implements device driver for LCD module.
* Creation Date: 11/1/2012
***********************************************************************************************************************/

/***********************************************************************************************************************
Pragma directive
***********************************************************************************************************************/
/* Start user code for pragma. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "r_cg_macrodriver.h"
#include "variable.h"
#include "lcd.h"
#include "function.h"
#include "meterology.h"
#include "Eprom_i2c.h"
#include "string.h"
#include "r_cg_wdt.h"

/* Start user code for include. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
/* Start user code for global. Do not edit comment generated here */

static const u8 LCD_TC_OPN[]    = {0x00,LCD_n,LCD_e,LCD_p,LCD_o,0x00,LCD_c};
static const u8 LCD_TC_CLO[]    = {LCD_e,LCD_s,LCD_o,LCD_l,LCD_c,LCD_c,LCD_m};
//static const u8 LCD_RTCFAIL[]   = {LCD_l,LCD_i,LCD_A,LCD_f,LCD_c,LCD_t,LCD_r};
//static const u8 LCD_RTCOK[]     = {LCD_k,LCD_o,LCD_c,LCD_t,LCD_r};
//static const u8 LCD_MAGTPR[]    = {LCD_r,LCD_p,LCD_t,LCD_G,LCD_A,LCD_m};
//static const u8 LCD_NOMAG[]     = {LCD_g,LCD_A,LCD_m,LCD_o,LCD_n};
static const u8 LCD_LO_PF[]     = {0x00,0x00,LCD_f,LCD_p,0x00,LCD_o,LCD_l};
static const u8 LCD_NE_MIS[]    = {0x00,LCD_s,LCD_s,LCD_i,LCD_m,0x00,LCD_n};
static const u8 LCD_OV_VOL[]    = {LCD_t,LCD_l,LCD_o,LCD_v,0x00,LCD_v,LCD_o};
static const u8 LCD_LO_VOL[]    = {LCD_t,LCD_l,LCD_o,LCD_v,0x00,LCD_o,LCD_l};
static const u8 LCD_CU_REV[]    = {0x00,LCD_e,LCD_s,LCD_r,LCD_v,LCD_e,LCD_r};
static const u8 LCD_EL[]        = {0x00,LCD_r,LCD_p,LCD_t,0x00,LCD_l,LCD_e};
static const u8 LCD_OV_CU[]     = {0x00,LCD_r,LCD_u,LCD_c,LCD_r,LCD_v,LCD_o};
static const u8 LCD_OV_LD[]     = {0x00,0x00,LCD_d,LCD_l,0x00,LCD_v,LCD_o};
static const u8 LCD_MAG[]       = {0x00,LCD_t,LCD_e,LCD_n,LCD_G,LCD_A,LCD_m};
static const u8 LCD_NE_DIS[]    = {0x00,0x00,LCD_s,LCD_i,LCD_d,0x00,LCD_n};
static const u8 LCD_35KV[]      = {0x00,LCD_d,LCD_s,LCD_e,0x00,LCD_v,LCD_h1};
static const u8 LCD_ABFRE[]     = {0x00,0x00,0x00,0x00,0x00,LCD_f,LCD_A};
static const u8 LCD_NO_TPR[]    = {0x00,LCD_r,LCD_p,LCD_t,0x00,LCD_o,LCD_n};
static const u8 LCD_TPR[]       = {0x00,LCD_r,LCD_e,LCD_p,LCD_m,LCD_A,LCD_t};
//static const u8 LCD_RBT_LO[]     = {LCD_o,LCD_l,0x00,LCD_t,LCD_b,LCD_r};
//static const u8 LCD_C_OPEN[]    = {LCD_n,LCD_e,LCD_p,LCD_o,0x00,LCD_c};  // changed to C OPEN

const unsigned char LCD_Tab[16] = {0xbe,0x06,0x7c,0x5e,0xc6,0xda,0xfa,0x0e,0xfe,0xde,0xee,0xf2,0xb8,0x76,0xf8,0xe8};
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
* Function Name: R_LCD_Create
* Description  : This function initializes the LCD module.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void LCD_vInit(unsigned char select)
{
    volatile uint32_t wt_count;
   // unsigned char temp;
    
    
 	OSMC |= 0x80;  /* Supply lock to LCD */
    LCDON = 0U;    /* disable LCD clock operation */
    LCDM1 |= _00_LCD_VOLTAGE_HIGH;
    LCDM0 = _00_LCD_DISPLAY_WAVEFORM_A | _0D_LCD_DISPLAY_MODE1;
    LCDM0 |= _40_LCD_VOLTAGE_MODE_INTERNAL;
    /* Set CAPL and CAPH pins */
    ISCLCD &= (uint8_t)~_01_LCD_CAPLH_BUFFER_VALID;
    P12 &= 0x3FU;
    PM12 |= 0xC0U;
    /* Set segment pins */
    PFSEG0 |= 0xF0U;
    PFSEG1 |= 0xFFU;
	PFSEG2 |= 0xFFU;
	
	/* Set Port 10 - 17 output 0 */
    P1 &= 0x00U;	
    PM1 &= 0x00U;
	/* Set Port 80 - 83 output 0 */
	P8 &= 0xF0U;	
	PM8 &= 0xF0U;
	/* Set Port 70 - 73 output 0 */
	P7 &= 0x00U;	
	PM7 &= 0x00U;

    LCDM1 |= _00_LCD_DISPLAY_PATTERN_A;
#if LCD_TYPE == 1
    LCDC0 = _00_LCD_SOURCE_CLOCK_FSUB | _05_LCD_CLOCK_FLCD_64;
#endif

#if LCD_TYPE == 2
	LCDC0 = _00_LCD_SOURCE_CLOCK_FSUB | _06_LCD_CLOCK_FLCD_128;
#endif

#if LCD_TYPE == 3
    LCDC0 = _00_LCD_SOURCE_CLOCK_FSUB | _05_LCD_CLOCK_FLCD_64;
#endif

    VLCD = _04_LCD_BOOST_VOLTAGE_100V;
	R_WDT_Restart();
	for (wt_count = 0U; wt_count <= LCD_REFVOLTAGE_WAITTIME; wt_count++)
	{
		__nop();
	}    
    LCDVLM = 1;
	LCD_vSetVoltageOn();
	LCD_vStart();
	lcd_disp_off();


#if LCD_TYPE==1
	if(select == 1)
	{
		if((P12 & BIT5)==0 && test_mode==0)
		{
		LCD_vDispValue(FirmIdDispButton[0],6);
		LCD_vDispValue(FirmIdDispButton[1],5);
		LCD_vDispValue(FirmIdDispButton[2],4);
		LCD_vDispValue(FirmIdDispButton[3],3);
		LCD_vDispValue(FirmIdDispButton[4],2);
		LCD_vDispValue(FirmIdDispButton[5],1);
		glow_2p();
		}
		else
		{
		LCD_vDispValue(FirmIdDisp[0],6);
		LCD_vDispValue(FirmIdDisp[1],5);
		LCD_vDispValue(FirmIdDisp[2],4);
		LCD_vDispValue(FirmIdDisp[3],3);
		LCD_vDispValue(FirmIdDisp[4],2);
		LCD_vDispValue(FirmIdDisp[5],1);
		glow_2p();
		}
	}
#endif

#if LCD_TYPE==2	
	if(select == 1)
	{
//		if((P12 & BIT5)==0 && test_mode==0)
//		{
//		LCD_vDispValue(0,7);
//		LCD_vDispValue(3,6);
//		LCD_vDispValue(5,5);
//		LCD_vDispValue(1,4);
//		LCD_vDispValue(0,3);
//		LCD_vDispValue(5,2);
//		glow_2p();
//		}
//		else
		{
		LCD_vDispValue(FirmIdDisp[0],6);
		LCD_vDispValue(FirmIdDisp[1],5);
		LCD_vDispValue(FirmIdDisp[2],4);
		LCD_vDispValue(FirmIdDisp[3],3);
		LCD_vDispValue(FirmIdDisp[4],2);
		LCD_vDispValue(FirmIdDisp[5],1);
		glow_2p();
		}
	}
#endif

#if LCD_TYPE==3
	if(select == 1)
	{
//		if((P12 & BIT5)==0 && test_mode==0)
//		{
//		LCD_vDispValue(FirmIdDispButton[0],6);
//		LCD_vDispValue(FirmIdDispButton[1],5);
//		LCD_vDispValue(FirmIdDispButton[2],4);
//		LCD_vDispValue(FirmIdDispButton[3],3);
//		LCD_vDispValue(FirmIdDispButton[4],2);
//		LCD_vDispValue(FirmIdDispButton[5],1);
//		glow_2p();
//		}
//		else
		{
		LCD_vDispValue(FirmIdDisp[1],7);
		LCD_vDispValue(FirmIdDisp[2],6);
		LCD_vDispValue(FirmIdDisp[3],5);
		LCD_vDispValue(FirmIdDisp[4],4);
		LCD_vDispValue(FirmIdDisp[5],3);
		glow_1p();
		}
	}
#endif
}

/***********************************************************************************************************************
* Function Name: R_LCD_Start
* Description  : This function enables the LCD display.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void LCD_vStart(void)
{
    LCDON = 1U;
}

/***********************************************************************************************************************
* Function Name: R_LCD_Stop
* Description  : This function disables the LCD display.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
//void LCD_vStop(void)
//{
//    LCDON = 0U;
//}

/***********************************************************************************************************************
* Function Name: R_LCD_Set_VoltageOn
* Description  : This function enables voltage boost circuit or capacitor split circuit.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void LCD_vSetVoltageOn(void)
{
    volatile uint32_t wt_count1;
    u8 i=0;    
    VLCON = 1U;
    R_WDT_Restart();
    for(i=0;i<45;i++)
    {
    /* Change the waiting time according to the system */
	    for(wt_count1 = 0U; wt_count1 <= 1600; wt_count1++)
	    {
	        __nop();
	    }
	    R_WDT_Restart();
    }
    
    SCOC = 1U;
}

/***********************************************************************************************************************
* Function Name: R_LCD_Set_VoltageOff
* Description  : This function disables voltage boost circuit or capacitor split circuit.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_LCD_Set_VoltageOff(void)
{
    SCOC = 0U;
    VLCON = 0U;
  //  LCDM0 &= (uint8_t)~_C0_LCD_VOLTAGE_MODE_INITIALVALUE;
}
/***********************************************************************************************************************
* Function Name: lcd_disp_off
* Description  : clear all seg register
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void lcd_disp_off(void)
{
	unsigned char i;
	volatile __near unsigned char* pointer;
	pointer = &SEG0;
	for(i=0;i<=23;++i)
		*(pointer + i) = 0x00;
}
/***********************************************************************************************************************
* Function Name: lcd_disp_off
* Description  : clear all seg register
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void lcd_disp_all(void)
{
	unsigned char i;
	volatile __near unsigned char* pointer;
	pointer = &SEG0;
	for(i=0;i<=23;++i)
		*(pointer + i) = 0xff;
}
void LCD_vLCDRamInit(void)
{
	u16 addrr;
	u8 tem;
	u8 i;
	
    Eprom_Read(lcd_Auto_Push_NO_Addr);
    if((EPROM_bChksum_ok == 1)&&((opr_data[0]!=0)&&(opr_data[1]!=0)))
    {
        if(opr_data[0]<=LCD_MAX_AUTO_LOC)
		{
            LCD_u8Max_AUTO = opr_data[0];
		}
        else
		{
            LCD_u8Max_AUTO = LCD_MAX_AUTO_LOC;
		}
		
        if(opr_data[1]<=LCD_MAX_PUSH_LOC)
		{
            LCD_u8Max_PUSH = opr_data[1];
		}
        else
		{
            LCD_u8Max_PUSH = LCD_MAX_PUSH_LOC;
		}

		lcd_scroll_time = opr_data[14] ;

		if(lcd_scroll_time ==0 || opr_data[14]>30)
		{
			lcd_scroll_time = 10 ;
		}

        addrr=lcd_Auto_Addr;
        for(tem=0;tem<LCD_u8Max_AUTO;tem=tem+15)
        {
            Eprom_Read(addrr);
            memcpy(lcd_a8AutoList+tem,opr_data,15);
            addrr=addrr+0x10;
        }
		
        addrr=lcd_Push_Addr;
        for(tem=0;tem<LCD_u8Max_PUSH;tem=tem+15)
        {
            Eprom_Read(addrr);
            memcpy(lcd_a8PushList+tem,opr_data,15);
            addrr=addrr+0x10;
        }
    }
    else
    {
//		LCD_u8Max_AUTO = sizeof(Default_AutoDisplay);
//		LCD_u8Max_PUSH = sizeof(Default_PushDisplay);
		
//        for(i=0;i<LCD_u8Max_AUTO;i++)
//        {
//            lcd_a8AutoList[i]=Default_AutoDisplay[i];
//        }
		 
//		for(i=0;i<LCD_u8Max_PUSH;i++)
//		{
//			lcd_a8PushList[i]=Default_PushDisplay[i];
//		}

	    lcd_scroll_time = 10 ; 
		
		LCD_u8Max_AUTO = 233;
		LCD_u8Max_PUSH = 233;
		
        for(i=0;i<LCD_u8Max_AUTO;i++)
        {
            lcd_a8AutoList[i]=i;
        }
		 
		for(i=0;i<LCD_u8Max_PUSH;i++)
        {
            lcd_a8PushList[i]=i;
		}

    }
}

/***********************************************************************************************************************
* Function Name: LCD_vDispChar
* Description  : This function disables voltage boost circuit or capacitor split circuit.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void LCD_vUpdatePerSec(void)
{
	if((DISP_bParCng == 1)&& (TP6.b.tc_exit_f == 0))
	{
		//lcd_disp_off();
		DISP_bParCng = 0;
		
		if(((P2 & BIT0)==0)&& ((MET_bkw_negative_f == 0) && (MET_bkw_el_negative_f == 0)&& (PWR_bBatt_backup_f ==0)))// // if((P1IN & cal_mode)==0)
		{
			fill_oprzero();
			lcd_disp_off();
			if(MET_bcal_mode_f==1)
			{
				scrol_cnt++;
				if(scrol_cnt>=2)
				{
				scrol_cnt=0;
				flag3.all ^=0x01;
				}
				disp_cal_act_power();
				//disp_act_power();
				if(MET_blag_calib_f==1)     
				{
					LCD_vDispChar(LCD_l,7);       //L//commented
					LCD_vDispChar(LCD_g,6);       //g//commented
					lag_cal_done=1;
				}
				else
				{
					lag_cal_done=0;
				}
				
				SW_u8cal_bat_disp_c=0;
			}
			else if(lag_cal_done==0)
			{
				if(MET_bcal_done_f==0)
				{
					LCD_vDispChar(LCD_e,6);       //E//commented
					LCD_vDispChar(LCD_r,5);       //r
					LCD_vDispChar(LCD_r,4);       //r
					LCD_vDispChar(LCD_o,3);       //o
					LCD_vDispChar(LCD_r,2);       //r
				}
				else
				{
#if LCD_TYPE !=3
					LCD_vDispChar(LCD_c,7);       //C////commented
					LCD_vDispChar(LCD_A,6);       //A
					LCD_vDispChar(LCD_l,5);       //L
					LCD_vDispChar(LCD_d,4);       //d
					LCD_vDispChar(LCD_o,3);       //o
					LCD_vDispChar(LCD_n,2);       //n
					LCD_vDispChar(LCD_e,1);       //e
#else
					LCD_vDispChar(LCD_c,7);       //C
					LCD_vDispChar(LCD_A,6);       //A
					LCD_vDispChar(LCD_l,5);       //L
					LCD_vDispChar(LCD_d,4);       //d
					LCD_vDispChar(LCD_n,3);       //n
					LCD_vDispChar(LCD_e,2);       //e
#endif
				}
				SW_u16cal_counter++;
				if(SW_u16cal_counter > 4)
				{
					MET_bcalib_flag=0;
				}
			}
		}  
		else
		{
		if(RtcRst_DispFlag)
        {
			lcd_disp_off();
            LCD_vDispChar(LCD_r,7);
            LCD_vDispChar(LCD_t,6);
            LCD_vDispChar(LCD_c,5);
            LCD_vDispChar(LCD_r,4);
            LCD_vDispChar(LCD_s,3);
            LCD_vDispChar(LCD_t,2);
            RTC_u8RtcRstCounter++;
            if(RTC_u8RtcRstCounter>=1)
            {
                RTC_u8RtcRstCounter=0;
                RtcRst_DispFlag=0;
            }                                
        }
		else if(SW_u8Test_menu_f==0)
			{
				if(switch_disp_f==1)
				{
					if(disp_lock_f==0)
					{
						if(PWR_bBatt_backup_f==1)
						{
							battpush_disp_par();
						}
						else
						{
							push_disp_par();
						}
					}
					else
					{
						lcd_disp_off();
						LCD_vDispChar(LCD_p,6);  //push
						LCD_vDispChar(LCD_u,5);
						LCD_vDispChar(LCD_s,4);
						LCD_vDispChar(LCD_h,3);
						disp_lock_f=0;
						SW_u8disp_cntr = 0;
					}
				}
				else
				{
					fill_oprzero();
					if(dispshow_unlock_f==0)
					{
//						if(TP5.b.tc_tpr_f==1)
//						{
//							TC_Open_Display();
//							dispshow_unlock_f=0;
//						}
//						else
						{
							disp_par();//disp_act_power();//disp_act_power_cnts();disp_ip(2);//disp_emer_cnt();//
						}
					}	
					else
					{
						lcd_disp_off();
						LCD_vDispChar(LCD_A,6); //a
						LCD_vDispChar(LCD_u,5); //u
						LCD_vDispChar(LCD_t,4); //t
						LCD_vDispChar(LCD_o,3); //o
						dispshow_unlock_f=0;
						SW_u8disp_cntr=0;
						SW_u16normal_scroll = 0;
					}
				}
			}
			else
			{
				//	man_vTest_Display();
			}
		}
		if((batt_disp_f==0) && (lcd_a8AutoList[SW_u8disp_cntr] != 214) && (lcd_a8PushList[SW_u8bat_disp_c] != 214))
		{
			if (TP4.b.neu_fea_f1==1)
				glow_N();
//			if (TP5.b.kv35_tpr_f==1)
//				glow_HV();
			if(((MET_bkw_negative_f == 1) || (MET_bkw_el_negative_f == 1)) && (TP1.b.mag_tpr_f==0))
				glow_REV();
			if((TP1.b.mag_tpr_f==1))//||(TP1.b.mag_active_f==1))
				glow_M();
			if((MET_bEl_glow == 1)&& (TP1.b.mag_tpr_f==0))
	            glow_EL();
			if((test_mode==0)&&(TP4.b.neu_fea_f1==0))
				glow_ON();
		}
		if((lcd_a8AutoList[SW_u8disp_cntr] != 214) && (lcd_a8PushList[SW_u8bat_disp_c] != 214))
		{
//			if((TC_OPEN_Port & TC_OPEN_Bit)!=0)
//			glow_OP();
//			else
//			SEG22&=0xfd ;
			
			if(TP5.b.tc_tpr_f == 1)
			{
				glow_OP();
			}
			if(nrm_flag == 1)
			{
				glow_C();
			}
			if(disable_bkup_f==1)
			{
				glow_B();
			}
		}
		if(DISP_bToggle)
		{
			DISP_bToggle =0;	
		}
		else
		{
			DISP_bToggle =1;	
		}
	
		    	
		disp_ShowT8();	
		
		if(((((P2 & BIT0)==0) && (MET_bkw_negative_f == 0) && (MET_bkw_el_negative_f == 0))||((MET_u8clib_lag_status !=0x03 || MET_u8clib_upf_status !=0x03) && switch_disp_f==0 && ((MET_u8clib_upf_status>=1) || (MET_u8clib_lag_status>=1)) && fg_done_f==0)) && batt_disp_f==0)
		{
			if(((P2 & BIT0)!=0) && (MET_u8clib_lag_status !=0x03 || MET_u8clib_upf_status !=0x03) && switch_disp_f==0 && batt_disp_f==0 && ((MET_u8clib_upf_status>=1) || (MET_u8clib_lag_status>=1)) && fg_done_f==0)
			{
				lcd_disp_off();
#if LCD_TYPE !=3
				LCD_vDispChar(LCD_n,7);       //N////commented
				LCD_vDispChar(LCD_o,6);       //o
				LCD_vDispChar(LCD_t,5);       //T
				LCD_vDispChar(0,4);        
				LCD_vDispChar(LCD_c,3);       //C
				LCD_vDispChar(LCD_A,2);       //A
				LCD_vDispChar(LCD_l,1);       //L
#else
				LCD_vDispChar(LCD_n,7);       //N////commented
				LCD_vDispChar(LCD_t,6);       //T
				LCD_vDispChar(LCD_c,4);       //C
				LCD_vDispChar(LCD_A,3);       //A
				LCD_vDispChar(LCD_l,2);       //L
#endif

				if(MET_u8clib_lag_status !=0x03)
				{
					glow_I();
					LCD_vDispChar(LCD_g,0);       //g
				}
				else if(MET_u8clib_upf_status !=0x03)
				{
					LCD_vDispChar(LCD_u,0);       //g
				}
			} 

			LCDM1 ^= 0x80;
		}
		else if(LCDON==0)
		{
			LCDM1 |= 0x80;
		}

	    if(SCOC == 0 || VLCON == 0)
	    {
		  LCD_vInit(0);  
	    }
	}
	
}

#if LCD_TYPE ==1
/***********************************************************************************************************************
* Function Name: LCD_vDispChar
* Description  : This function disables voltage boost circuit or capacitor split circuit.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void LCD_vDispValue(unsigned char val,unsigned char loc)
{
	volatile __near unsigned char* pointer;
	unsigned char temp;
	pointer = &SEG7;
	temp = val & 0x0f;
	*(pointer+loc*2) = ((*(LCD_Tab+temp)) | (*(pointer+loc*2) & 0x01));
	*(pointer+loc*2+1) = (*(LCD_Tab+temp))>>4;
}
void LCD_vDispChar(unsigned char val,unsigned char loc)
{
	volatile __near unsigned char* pointer;
	unsigned char temp;
	pointer = &SEG7;
	temp = val & 0x0f;
	pointer = pointer+loc* 2;
	*(pointer++) = (temp);
	*(pointer) = (val)>>4;
}
void disp_vDispString(const u8* par,u8 u8cnt)
{
	volatile __near unsigned char* pointer;
	unsigned char temp;
	
	lcd_disp_off();
//	pointer = &SEG4;
	for(temp=0;temp<u8cnt;temp++)
	{
		pointer = &SEG5+temp* 2+4;
		*(pointer++) = *(par)&0x0f;
		*(pointer) = (*par++)>>4;
	}
}	
	
/***********************************************************************************************************************
* Function Name: time_bcd
* Description  : This function disables voltage boost circuit or capacitor split circuit.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void time_bcd(volatile unsigned char *data)
{
	unsigned char i,temp,x=0;	//8
	volatile unsigned char __near *pointer = &SEG11;
	
	lcd_disp_off();
	for(i=0;i<3;i++)
	{
		temp = *(data+i);
		temp &= 0x0f;
		*(pointer+x) = *(LCD_Tab+temp);
		x++;
		*(pointer+x) = (*(LCD_Tab+temp))>>4;
		x++;
		temp = *(data+i);
		temp = temp>>4;
		temp &= 0x0f;
		*(pointer+x) = *(LCD_Tab+temp);
		x++;
		*(pointer+x) = (*(LCD_Tab+temp))>>4;
		x++;
	}
	glow_COL();
}

void disp_onlcd(u8 x,u8 z)
{
	unsigned char i,y;
	volatile unsigned char __near *pointer = &SEG7;
	lcd_disp_off();
	
	x=x*2;
	for(i=0;i<z;i++)  
	{
		y = *(opr_data+i);
		y &= 0x0f;
		*(pointer+x) = *(LCD_Tab+y);
		x++;
		*(pointer+x) = (*(LCD_Tab+y))>>4;
		x++;		
	}
}
#endif

#if LCD_TYPE ==2
void disp_vDispString(const u8* par,u8 u8cnt)
{
	volatile __near unsigned char* pointer;
	unsigned char temp;
	lcd_disp_off();
//	pointer = &SEG4;
	for(temp=0;temp<u8cnt;temp++)
	{
		pointer = &SEG0+temp*2+4;
		*(pointer++) = *(par)&0x0f;
		*(pointer) = (*par++)>>4;
	}
}	

void LCD_vDispValue(unsigned char val,unsigned char loc)
{
	volatile unsigned char __near* pointer;
	unsigned char temp;
	
	if(loc ==0)
	loc=8;
	
	pointer = &SEG2;
	temp = val & 0x0f;
	*(pointer+loc*2) = ((*(LCD_Tab+temp)) | (*(pointer+loc*2) & 0x01));
	*(pointer+loc*2+1) = (*(LCD_Tab+temp))>>4;
}
void time_bcd(volatile unsigned char *data)
{
	unsigned char i,temp,x=0;	//8
	volatile unsigned char __near* pointer = &SEG6;
	lcd_disp_off();
	for(i=0;i<3;i++)
	{
		temp = *(data+i);
		temp &= 0x0f;
		*(pointer+x) = *(LCD_Tab+temp);
		x++;
		*(pointer+x) = (*(LCD_Tab+temp))>>4;
		x++;
		temp = *(data+i);
		temp = temp>>4;
		temp &= 0x0f;
		*(pointer+x) = *(LCD_Tab+temp);
		x++;
		*(pointer+x) = (*(LCD_Tab+temp))>>4;
		x++;
	}
	glow_COL();
	//disp_zoneindex_f=0;
}
void LCD_vDispChar(unsigned char val,unsigned char loc)
{
	volatile unsigned char __near* pointer;
	unsigned char temp;
	
	if(loc ==0)
	loc=8;
	pointer = &SEG2;
	temp = val & 0x0f;
	pointer = pointer+loc* 2;
	*(pointer++) = (temp);
	*(pointer) = (val)>>4;
}
void disp_onlcd(unsigned char x,u8 z)
{
	unsigned char i,y;
	volatile unsigned char __near* pointer = &SEG2 ;
	lcd_disp_off();
	
	x=x*2;
	for(i=0;i<z;i++)  
	{
		y = *(opr_data+i);
		y &= 0x0f;
		*(pointer+x) = *(LCD_Tab+y);
		x++;
		*(pointer+x) = (*(LCD_Tab+y))>>4;
		x++;
	}
}
#endif

#if LCD_TYPE ==3
/***********************************************************************************************************************
* Function Name: LCD_vDispChar
* Description  : This function disables voltage boost circuit or capacitor split circuit.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void LCD_vDispValue(unsigned char val,unsigned char loc)
{
	volatile __near unsigned char* pointer;
	unsigned char temp;
	pointer = &SEG6;
	temp = val & 0x0f;
	*(pointer+loc*2) = ((*(LCD_Tab+temp)) | (*(pointer+loc*2) & 0x01));
	*(pointer+loc*2+1) = (*(LCD_Tab+temp))>>4;
}
void LCD_vDispChar(unsigned char val,unsigned char loc)
{
	volatile __near unsigned char* pointer;
	unsigned char temp;
	pointer = &SEG6;
	temp = val & 0x0f;
	pointer = pointer+loc* 2;
	*(pointer++) = (temp);
	*(pointer) = (val)>>4;
}
void disp_vDispString(const u8* par,u8 u8cnt)
{
	volatile __near unsigned char* pointer;
	unsigned char temp;
	
	lcd_disp_off();
	
	for(temp=0;temp<u8cnt;temp++)
	{
		pointer = &SEG6+temp* 2 + 2;
		*(pointer++) = *(par)&0x0f;
		*(pointer) = (*par++)>>4;
	}
}	

/***********************************************************************************************************************
* Function Name: time_bcd
* Description  : This function disables voltage boost circuit or capacitor split circuit.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void time_bcd(volatile unsigned char *data)
{
	unsigned char i,temp,x=0;	//8
	volatile unsigned char __near *pointer = &SEG10;
	
	lcd_disp_off();
	for(i=0;i<3;i++)
	{
		temp = *(data+i);
		temp &= 0x0f;
		*(pointer+x) = *(LCD_Tab+temp);
		x++;
		*(pointer+x) = (*(LCD_Tab+temp))>>4;
		x++;
		temp = *(data+i);
		temp = temp>>4;
		temp &= 0x0f;
		*(pointer+x) = *(LCD_Tab+temp);
		x++;
		*(pointer+x) = (*(LCD_Tab+temp))>>4;
		x++;
	}
	
	if(LCD_DispRTC_Time_f == 0)
	{
		SEG10 &=0x01;
		SEG11 &=0x00;
		SEG12 &=0x01;
		SEG13 &=0x00;
		glow_4p();
		LCD_vDispChar(LCD_t,2);
	}
	else
	{
		glow_COL();
		LCD_DispRTC_Time_f=0;
	}
}

void disp_onlcd(u8 x,u8 z)
{
	unsigned char i,y;
	volatile unsigned char __near *pointer = &SEG6;
	lcd_disp_off();
	
	if(x==1)
	x=2;
	
	x=x*2;
	for(i=0;i<z;i++)  
	{
		y = *(opr_data+i);
		y &= 0x0f;
		*(pointer+x) = *(LCD_Tab+y);
		x++;
		*(pointer+x) = (*(LCD_Tab+y))>>4;
		x++;		
	}
}
#endif

void disp_call(u8 z)
{
	bin_2_dec();
	disp_onlcd(2,z);
}
/***********************************************************************************************************************
* Function Name: time_bcd
* Description  : This function disables voltage boost circuit or capacitor split circuit.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void disp_avg_pf(unsigned char bmonth)
{
	unsigned int u16temp_pf;
    
	cal_avg_pf();
	u16temp_pf=MET_u16Avg_pf;
    fill_oprzero();
    FUN_vfill_2byte(u16temp_pf,opr_data);
    bin_2_dec();
    disp_onlcd(2,4);

    glow_1p();
#if ((LCD_TYPE == 1) || (LCD_TYPE == 3))
	LCD_vDispChar(LCD_p,7);
	LCD_vDispChar(LCD_f,6);
	LCD_vDispChar(LCD_A,0);
#else
	glow_PF();
	LCD_vDispChar(LCD_A,10);
#endif
}
void disp_billpcount(void)
{
    opr_data[0]=BILL_u8md_count;
    disp_call(3);

	LCD_vDispChar(LCD_c,7);
    glow_BP();
}
/*void disp_LastBillDate(void)
{
	signed char temp1;
	temp1 =BILL_u8mdmonth-1;
	if(temp1<=0)
		temp1=temp1+MaxBillDate;                               
	temp1=calmdpg(temp1);    
	Eprom_Read(billkwh_data_addrs+(unsigned char)temp1);
	time_bcd(&opr_data[8]);
	glow_BP();
	
}*/

/*void disp_NoPowerFailureCnt(void)
{
	opr_data[0] = PWR_u8on_off_cnt;
	disp_call(4);
	 if(DISP_bToggle)
	  LCD_vDispChar(LCD_p,7);
	  else
	  LCD_vDispChar(LCD_c,7);     
}*/

void disp_PowerFailureDuration(void)
{
	memcpy(&opr_data[0],&PWR_u32cum_poff_min,4);
	disp_call(4);   	
}


void mdkVA_time(unsigned char bmonth,unsigned char ttime)
{
	read_add(billmd_data_addrs,bmonth);
	
    if(ttime==0)//time
    { 
		opr_data[8]=0;
		time_bcd(&opr_data[8]);
#if LCD_TYPE != 3
		LCD_vDispChar(LCD_t,1);
#endif
    }
    else
    {   
		LCD_DispRTC_Time_f = 1;
		time_bcd(&opr_data[11]);
#if LCD_TYPE == 1 
		LCD_vDispChar(LCD_d,1);
#elif LCD_TYPE == 2
		LCD_vDispChar(LCD_d,10);
#endif		
    }

    glow_MD(); 
	//glow_k();
	//glow_V();
	//glow_A();
#if ((LCD_TYPE == 1) || (LCD_TYPE == 3))
    if(bmonth==0)
    {

    }
    else
    {
	LCD_vDispValue(bmonth,0);
	glow_BP();   //BP   
	}
	//disp_zoneindex_f =0;
#else	
	if(bmonth<10)
	{
		LCD_vDispChar(LCD_b,9);
		LCD_vDispValue(bmonth,8);
	}
	else
	{
		LCD_vDispValue(bmonth/10,9);
		LCD_vDispValue(bmonth%10,8);		
	}
	glow_BP();   //BP   
#endif

}
void disp_code(unsigned char tmcode)
{
    u8 temp1,temp2,tcode=tmcode-1;
    Eprom_Read(CodeAdd);
    temp1 = opr_data[tcode*2];
    temp2 = opr_data[(tcode*2)+1];
    fill_oprzero();
    opr_data[0]=temp2;
    opr_data[1]=temp1;
    bin_2_dec();
    disp_onlcd(2,5);
	LCD_vDispValue(tmcode+9,0);
#if LCD_TYPE == 2
	LCD_vDispChar(LCD_c,9);
	LCD_vDispChar(LCD_d,8);
#endif
}

void md1_time(unsigned char bmonth,unsigned char ttime)
{
	read_add(billmd_data_addrs,bmonth);
	
    if(ttime==0)//time
    { 
	    opr_data[1]=0;
		time_bcd(&opr_data[1]);
#if LCD_TYPE !=3
		LCD_vDispChar(LCD_t,1);
#endif
    }
    else
    {
		LCD_DispRTC_Time_f = 1;
		time_bcd(&opr_data[4]);
#if LCD_TYPE == 1
		LCD_vDispChar(LCD_d,1);
#elif LCD_TYPE == 2
		LCD_vDispChar(LCD_d,10);
#endif
    }

    glow_MD();
	//glow_k();
	//glow_W();
#if ((LCD_TYPE == 1) || (LCD_TYPE == 3))
    if(bmonth==0)
    {

    }
    else	
    {
	 LCD_vDispValue(bmonth,0);
	 glow_BP();   //BP  
	}
	//disp_zoneindex_f =0;
#else
    if(bmonth==0)
    {
		LCD_vDispChar(LCD_c,9);
	    LCD_vDispChar(LCD_o,8);
    }
    else
	{
	if(bmonth<10)
	{
		LCD_vDispChar(LCD_b,9);
		LCD_vDispValue(bmonth,8);
	}
	else
	{
		LCD_vDispValue(bmonth%10,8);
		LCD_vDispValue(bmonth/10,9);
	}
	 glow_BP();   //BP    
    }
#endif
}
void bill_time(unsigned char bmonth,unsigned char ttime)
{
	read_add(billkwh_data_addrs,bmonth);
	
    if(ttime==0)//time
    { 
		time_bcd(&opr_data[6]);
#if LCD_TYPE != 3
		LCD_vDispChar(LCD_t,1);
#endif
    }
    else
    {   
		LCD_DispRTC_Time_f = 1;
		time_bcd(&opr_data[8]);
#if LCD_TYPE == 1
		LCD_vDispChar(LCD_d,1);
#elif LCD_TYPE == 2
		LCD_vDispChar(LCD_d,10);
#endif
    }

    if(bmonth==0)
    {

    }
    else	
    {

#if LCD_TYPE ==2
	if(bmonth <10)
	{
		LCD_vDispChar(LCD_b,9);
		LCD_vDispValue(bmonth,8);
	}
	else
	{
		LCD_vDispValue((bmonth/10),9);
		LCD_vDispValue((bmonth%10),8);
	}
#else
	LCD_vDispValue(bmonth,0);
	//disp_zoneindex_f=0;
#endif
	glow_BP();   //BP    
    }
}
void disp_ZKW(u8 u8zone)
{
    u8 temp;
    temp = u8zone-1;
    if(u8zone == TOD_u8zone_index)
    {
        FUN_vfill_3byte(TOD_u32cum_zkwh,opr_data);
    }
    else
    {
    Eprom_Read(Zone_energy_data_Add+temp*0x10);
    temp=opr_data[0];
    opr_data[0]=opr_data[2];
    opr_data[2]=temp;
    opr_data[3]=0x00;
    }
	bin_2_dec();
	disp_onlcd(1,6);
    glow_3p();  //for decimal point
    glow_k();//k
    glow_W();//W
    glow_h();
	glow_T();
#if ((LCD_TYPE == 1) || (LCD_TYPE == 3))
	LCD_vDispValue(u8zone,0);
#else
	LCD_vDispValue(u8zone,10);
#endif
	//disp_zoneindex_f=0;
}

void disp_ZKVA(u8 u8zone)
{
    u8 temp;
    temp = u8zone-1;
    if(u8zone == TOD_u8zone_index)
    {
        FUN_vfill_3byte(TOD_u32cum_zkvah,opr_data);
    }
    else
    {
    Eprom_Read(Zone_energy_data_Add+temp*0x10);
    opr_data[0] = opr_data[5];
    opr_data[1] = opr_data[4];
    opr_data[2] = opr_data[3];
    opr_data[3]=0x00;
    }
	bin_2_dec();
	disp_onlcd(1,6);
    glow_3p();  //for decimal point
    glow_k();//k
    glow_V();//W
	glow_A();
    glow_h();
	glow_T();
#if ((LCD_TYPE == 1) || (LCD_TYPE == 3))
	LCD_vDispValue(u8zone,0);
#else
	LCD_vDispValue(u8zone,10);
#endif
	//disp_zoneindex_f=0;
}

void disp_srno(void)
{   	
	u8 m1,m2;
	
	Eprom_Read(SerialNo_UtilityID_add);	
	if(EPROM_bChksum_ok==0)
	{
		Eprom_Read(Dupli_SerialNo_UtilityID_add);
		if(EPROM_bChksum_ok==1)
		Eprom_Write(SerialNo_UtilityID_add);
	}
	lcd_disp_off();
	for(m1=6,m2=2;m1>0;m1--)
	{
		if(opr_data[m1-1]!=0x20)
		{
			LCD_vDispValue(opr_data[m1-1]-0x30,m2); 
			m2++;
		}
	}
//#if LCD_TYPE == 2
//      LCD_vDispChar(LCD_s,9);
//	  LCD_vDispChar(LCD_r,8);
//#endif
}
void disp_md1(unsigned char bmonth)
{
     unsigned char temp;
    read_add(billmd_data_addrs,bmonth);
    temp=opr_data[0];
    opr_data[0]=opr_data[1];
    opr_data[1]=temp;
    opr_data[2]=0x00;
    opr_data[3]=0x00;
    disp_call(5);


#if ((LCD_TYPE == 1) || (LCD_TYPE == 3))
    if(bmonth==0)
    {

    }
    else
    {
	  LCD_vDispValue(bmonth,0);
	  glow_BP();   //BP 
	  //disp_zoneindex_f =0;
    }
#else

    if(bmonth==0)
    {
	  LCD_vDispChar(LCD_c,9);
	  LCD_vDispChar(LCD_o,8);
    }
    else
    {
		if(bmonth<10)
		{
			LCD_vDispChar(LCD_b,9);
			LCD_vDispValue(bmonth,8);
		}
		else
		{
			LCD_vDispValue(bmonth/10,9);
			LCD_vDispValue(bmonth%10,8);		
		}
	  glow_BP();   //BP 
    }
#endif

    glow_MD();// md
    glow_1p();  //for decimal point
    glow_k();//k
    glow_W();//W
}
void disp_md_kVA(unsigned char bmonth)
{
     unsigned char temp;
    read_add(billmd_data_addrs,bmonth);
    temp=opr_data[7];
    opr_data[0]=opr_data[8];
    opr_data[1]=temp;
    opr_data[2]=0x00;
    opr_data[3]=0x00;
    disp_call(5);

#if ((LCD_TYPE == 1) || (LCD_TYPE == 3))
    if(bmonth==0)
    {

    }
    else
    {
	  LCD_vDispValue(bmonth,0);
	  glow_BP();   //BP 
	  //disp_zoneindex_f =0;
    }
#else

    if(bmonth==0)
    {
	  LCD_vDispChar(LCD_c,9);
	  LCD_vDispChar(LCD_o,8);
    }
    else
    {
		if(bmonth<10)
		{
			LCD_vDispChar(LCD_b,9);
			LCD_vDispValue(bmonth,8);
		}
		else
		{
			LCD_vDispValue(bmonth/10,9);
			LCD_vDispValue(bmonth%10,8);		
		}
	  glow_BP();   //BP 
    }
#endif

    glow_MD();// md
    glow_1p();  //for decimal point
    glow_k();//k
    glow_V();
    glow_A();
}

void disp_bkwh1(unsigned char bmonth)
{
    unsigned char temp1,temp2,temp3;//,add;
    read_add(billkwh_data_addrs,bmonth);
    temp1=opr_data[2];
    temp2=opr_data[1];
    temp3=opr_data[0];
    opr_data[0] = temp1;
    opr_data[1] = temp2;
    opr_data[2] = temp3;
    opr_data[3] = 0x00;
    bin_2_dec();
//    kwh_disp_f=0;
    disp_onlcd(1,6);
#if LCD_TYPE == 2
	if(bmonth !=0)
  	{
		if(bmonth<10)
		{
			LCD_vDispChar(LCD_b,9);
			LCD_vDispValue(bmonth,8);
		}
		else
		{
			LCD_vDispValue(bmonth/10,9);
			LCD_vDispValue(bmonth%10,8);		
		}
	}
#else
		LCD_vDispValue(bmonth,0);
		//disp_zoneindex_f =0;
#endif
    glow_k();
    glow_W();
    glow_h(); 
	glow_3p();
    glow_BP();

}
void disp_bkvah1(unsigned char bmonth)
{
    unsigned char temp1,temp2,temp3;//,add;
    read_add(billkwh_data_addrs,bmonth);
    temp1=opr_data[5];
    temp2=opr_data[4];
    temp3=opr_data[3];
    opr_data[0] = temp1;
    opr_data[1] = temp2;
    opr_data[2] = temp3;
    opr_data[3] = 0x00;
    bin_2_dec();
    disp_onlcd(1,6);
#if LCD_TYPE == 2
	if(bmonth !=0)
  	{
		if(bmonth<10)
		{
			LCD_vDispChar(LCD_b,9);
			LCD_vDispValue(bmonth,8);
		}
		else
		{
			LCD_vDispValue(bmonth/10,9);
			LCD_vDispValue(bmonth%10,8);		
		}
	}
#else
		LCD_vDispValue(bmonth,0);
		//disp_zoneindex_f =0;
#endif
    glow_k();
    glow_V();
    glow_A();
    glow_h(); 
	glow_3p();
    glow_BP();
}

void disp_tcnt(void)
{
      FUN_vfill_2byte(TPR_u16cum_tpr_c,opr_data);
      disp_call(5);
#if ((LCD_TYPE == 1) || (LCD_TYPE == 3))
      LCD_vDispChar(LCD_c,0);
#else
     LCD_vDispChar(LCD_c,9);
	 LCD_vDispChar(LCD_u,8);
	 LCD_vDispChar(LCD_c,10);
#endif
      glow_T();    
}

void disp_kwh(void)
{
    FUN_vfill_3byte(MET_u32Cum_kwh,opr_data);

    bin_2_dec();
    disp_onlcd(1,6);
    
    glow_k();//k
    glow_h();//h
    glow_W();//W
    glow_3p();//for decimal point & LSB of cumkwh blank(.x is blank)  
#if LCD_TYPE == 2
      LCD_vDispChar(LCD_e,9);
	  LCD_vDispChar(LCD_n,8);
#endif
}

void disp_kvah(void)
{
    FUN_vfill_3byte(MET_u32Cum_kvah,opr_data);

    bin_2_dec();
    disp_onlcd(1,6);
    
    glow_k();//k
    glow_h();//h
    glow_V();
	glow_A();//
    glow_3p();//for decimal point & LSB of cumkwh blank(.x is blank) 
#if LCD_TYPE == 2
      LCD_vDispChar(LCD_e,9);
	  LCD_vDispChar(LCD_n,8);
#endif
}

void disp_app_power(void)
{
    FUN_vfill_2byte(MET_u16Kva,opr_data);
    disp_call(5);
    
    glow_k();//k
    glow_V();
	glow_A();//
    glow_1p();//for decimal point & LSB of cumkwh blank(.x is blank)
	
#if LCD_TYPE ==2
	LCD_vDispChar(LCD_l,9);
	LCD_vDispChar(LCD_d,8);
#else
	glow_I();
	LCD_vDispChar(LCD_d,0);
#endif
}
void disp_act_power(u8 tempChoice)
{
	
	if(tempChoice==0)
	{
		 chngtemp.u16_temp[0] = MET_u16Kw;
	}
	else if(tempChoice==1)
	{
		chngtemp.u16_temp[0] = MET_u16ELKw; 
	}
	else 
	{
		if(MET_bph_ct_f1==0)  
		{
		  chngtemp.u16_temp[0] = MET_u16Kw;
		}
		else
		{
		    chngtemp.u16_temp[0] = MET_u16ELKw;  
		}
	}

    opr_data[0] = chngtemp.u8_temp[0];
    opr_data[1] = chngtemp.u8_temp[1];
    disp_call(5);
    
    glow_k();//k
    glow_W();
    glow_1p();//for decimal point & LSB of cumkwh blank(.x is blank)    

#if LCD_TYPE ==2
	LCD_vDispChar(LCD_l,9);
	LCD_vDispChar(LCD_d,8);
#else

	if(tempChoice==0)
	{
	 	LCD_vDispChar(LCD_p,7);
	}
	else if(tempChoice==1)
	{
		LCD_vDispChar(LCD_n,7);
	}
	glow_I();
	LCD_vDispChar(LCD_d,0);
#endif

}

void disp_cal_act_power(void)
{
    if(cal_pwr_f==1)  
        chngtemp.u16_temp[0] = MET_u16Kw;
    else
        chngtemp.u16_temp[0] = MET_u16ELKw;    
   	opr_data[0] = chngtemp.u8_temp[0];
    opr_data[1] = chngtemp.u8_temp[1];
    
    disp_call(5);
    
    if(cal_pwr_f==1)
    {
        LCD_vDispChar(LCD_p,0);//LCDMEM[10] |= LCD_n;//0xce;//n
		cal_pwr_f=0;
    }
    else
    {
        LCD_vDispChar(LCD_n,0);//LCDMEM[10] |= LCD_p;//0xea;//p
		cal_pwr_f=1;
    }
	glow_k();
    glow_W();
    glow_1p();//for decimal point & LSB of cumkwh blank(.x is blank) 
}
void disp_ip(unsigned char current)
{
    if(PWR_bBatt_backup_f==1)
    {
        MET_u32ip_rms=0;
        MET_u32in_rms=0;
    }
    if((MET_u32in_rms<=7)&&(PWR_u8test_mode==0))
	{
        MET_u32in_rms=0;
	}
    if((MET_u32ip_rms<=7)&&(PWR_u8test_mode==0))
	{
        MET_u32ip_rms=0;
	}
		
	if(current==1)
	{
		FUN_vfill_3byte(MET_u32ip_rms,opr_data);//FUN_vfill_3byte(MET_u32ip_rms,opr_data);
	}
	else if(current==2)
	{
		FUN_vfill_3byte(MET_u32in_rms,opr_data);
	}
	else if(current==3)
	{
		if(MET_bph_ct_f1==0)
		{
			FUN_vfill_3byte(MET_u32ip_rms,opr_data);
		}
		else
		{
			FUN_vfill_3byte(MET_u32in_rms,opr_data);
		}
	}
	
	
	disp_call(5);        
	
	glow_1p();
    glow_I();
	glow_A();

	if(current==1)
	{
		LCD_vDispChar(LCD_p,0);
	}
	else if(current==2)
	{
		LCD_vDispChar(LCD_n,0);
	}
	else if(current==3)
	{
		if(MET_bph_ct_f1==0)
		{
			LCD_vDispChar(LCD_p,0);
		}
		else
		{
			LCD_vDispChar(LCD_n,0);
		}
	}
	
#if LCD_TYPE == 1
		//disp_zoneindex_f =0;
#endif

}


void disp_v(void)
{       
	if((PWR_bBatt_backup_f==1)||(PWR_u8test_mode==1))
		MET_u16v_rms=0;
	FUN_vfill_2byte(MET_u16v_rms,opr_data);
	bin_2_dec();
#if LCD_TYPE == 2
	disp_onlcd(2,5);
	glow_2p();
#else
	disp_onlcd(3,5);
	glow_1p();
#endif
	glow_V();

}

void disp_freq(void)
{       
       FUN_vfill_2byte(MET_u16freq,opr_data);
        disp_call(5);
        glow_1p();
#if ((LCD_TYPE == 1) || (LCD_TYPE == 3))
		LCD_vDispChar(LCD_f,0);
#else
		LCD_vDispChar(LCD_f,9);
		LCD_vDispChar(LCD_r,8);
#endif
}

void disp_pf(void)
{
	FUN_vfill_2byte(MET_u16pf,opr_data);
	bin_2_dec();
	disp_onlcd(2,4);
	glow_1p();//for decimal point & LSB of cumkwh blank(.x is blank)    
#if ((LCD_TYPE == 1) || (LCD_TYPE == 3))
	LCD_vDispChar(LCD_p,7);
	LCD_vDispChar(LCD_f,6);
#else
	glow_PF();
#endif

	if((MET_u32ip_rms==0 && MET_u32in_rms == 0) || TP4.b.neu_fea_f1==1 || /*PWR_u8test_mode==1 ||*/ (MET_u16pf >= 990))
	   MET_bpf_sign=0;            
}

void disp_hires_kwh(void)
{
    unsigned long int hi_kwh1=0;
	
	MET_vcheck_kwh_kvah();
	hi_kwh1 = Compute_EnergyHighres(1);
	FUN_vfill_4byte(hi_kwh1,opr_data);
	
    bin_2_dec();
	disp_onlcd(1,6);

    glow_k();//k
    glow_h();//h
    glow_W();
    glow_4p();//for decimal point & LSB of cumkwh blank(.x is blank)    

#if ((LCD_TYPE == 1) || (LCD_TYPE == 3))
	LCD_vDispChar(LCD_h,0);
#else
	LCD_vDispChar(LCD_h1,9);
	LCD_vDispChar(LCD_i,8);
#endif
	
}

void disp_hires_kvah(void)
{
    unsigned long int hi_kvah1=0;
	
	MET_vcheck_kwh_kvah();
	hi_kvah1 = Compute_EnergyHighres(2);
	FUN_vfill_4byte(hi_kvah1,opr_data);
	
    bin_2_dec();
	disp_onlcd(1,6);

    glow_k();//k
	glow_V();
	glow_A();
    glow_h();//h

    glow_4p();//for decimal point & LSB of cumkwh blank(.x is blank)    

#if ((LCD_TYPE == 1) || (LCD_TYPE == 3))
	LCD_vDispChar(LCD_h,0);
#else
	LCD_vDispChar(LCD_h1,9);
	LCD_vDispChar(LCD_i,8);
#endif
}

//void Disp_vErrorCode(u8 u8ErrorNo)
//{
//	memcpy(&LCDMEM[4],MAN_ERR,7);
//	LCDMEM[5]=LCD_Tab[u8ErrorNo+1];
//}


void disp_avg_billpf(unsigned char bmonth)        // function to read and calculate then display the average pf of bill data
{
	u16 temp;

	read_add(billkwh_data_addrs,bmonth);

	temp=opr_data[14];
	temp*=4;
	fill_oprzero();

	FUN_vfill_2byte(temp,opr_data);
	bin_2_dec();
	disp_onlcd(2,4);
	glow_1p();//for decimal point & LSB of cumkwh blank(.x is blank)    
#if ((LCD_TYPE == 1) || (LCD_TYPE == 3))
	LCD_vDispChar(LCD_p,7);
	LCD_vDispChar(LCD_f,6);
	if(bmonth != 0)
	LCD_vDispValue(bmonth,0);
	//disp_zoneindex_f=0;
#else
	glow_PF();
	if(bmonth != 0)
	{
		if(bmonth < 10)
		{
		LCD_vDispChar(LCD_b,9);
		LCD_vDispValue(bmonth,8);
		}
		else
		{
		LCD_vDispValue(bmonth/10,9);
		LCD_vDispValue(bmonth%10,8);
		}
	}
#endif
	glow_BP();
}

void disp_curnt_btprcont(void)
{
    read_add(billmd_data_addrs,1);
    opr_data[0]=opr_data[14];
    opr_data[1]=0;
    opr_data[2]=0;
    opr_data[3]=0;
    disp_call(3);
#if ((LCD_TYPE == 1) || (LCD_TYPE == 3))
	LCD_vDispChar(LCD_t,7);
	LCD_vDispChar(LCD_c,6);
	LCD_vDispChar(0,5);
#else
	LCD_vDispChar(LCD_b,9);
	LCD_vDispValue(1,8);
	LCD_vDispChar(LCD_c,10);
	glow_T();
#endif
    //LCD_vDispValue(1,0);
    glow_BP();
}

void disp_bpoff_min(unsigned char bmonth)
{
	u16 temp,min,hr;

	if (bmonth==0)
	    temp=PWR_u16poff_mins;
	else
	{
	    read_add(billkwh_data_addrs,bmonth);
	    temp=((opr_data[12]*0x100)+opr_data[13]);
	}
	min=temp%60;
	hr=temp/60;
	fill_oprzero();

	FUN_vfill_2byte(min,&opr_data[0]);
	FUN_vfill_2byte(hr,&opr_data[2]);
	bin_2_dec();
	disp_onlcd(2,6);

	glow_BP();
	glow_1p();
}

void disp_tc_status(void)
{
   
	if(TP5.b.tc_tpr_f == 1)
    {
		disp_vDispString(LCD_TC_OPN,7);
    }
    else 
    {
		disp_vDispString(LCD_TC_CLO,7);
    }
    
}


void disp_rtc_status(void)
{
    if(rtc_status_byte==1)              // && test_mode_f==0)
      { 
                             
		LCD_vDispChar(LCD_r,7);
		LCD_vDispChar(LCD_t,6);
		LCD_vDispChar(LCD_c,5);
		LCD_vDispChar(LCD_f,4);
		LCD_vDispChar(LCD_A,3);
		//LCD_vDispChar(LCD_i,2);
		LCD_vDispChar(LCD_l,2);
      }
    else
    {
		LCD_vDispChar(LCD_r,7);
		LCD_vDispChar(LCD_t,6);
		LCD_vDispChar(LCD_c,5);
		LCD_vDispChar(LCD_o,3);
		LCD_vDispChar(LCD_k,2);
    }
}

void disp_bat_status(void)
{
    if(PWR_bbat_discharge_f==1)              // && test_mode_f==0)
      { 
                             
		LCD_vDispChar(LCD_b,7);
		LCD_vDispChar(LCD_A,6);
		LCD_vDispChar(LCD_t,5);
		LCD_vDispChar(LCD_l,4);
		LCD_vDispChar(LCD_o,3);
      }
    else
    {
		LCD_vDispChar(LCD_b,7);
		LCD_vDispChar(LCD_A,6);
		LCD_vDispChar(LCD_t,5);
		LCD_vDispChar(LCD_o,3);
		LCD_vDispChar(LCD_k,2);
    }
}

void disp_eprom_status(void)
{
     if(acknotr_f0==1)
	{
		LCD_vDispChar(LCD_n,7);
		LCD_vDispChar(LCD_v,6);
		LCD_vDispChar(LCD_m,5);
		LCD_vDispChar(LCD_f,4);
		LCD_vDispChar(LCD_A,3);
		//LCD_vDispChar(LCD_i,2);
		LCD_vDispChar(LCD_l,2);
     }
     else
     {
		LCD_vDispChar(LCD_n,7);
		LCD_vDispChar(LCD_v,6);
		LCD_vDispChar(LCD_m,5);
		LCD_vDispChar(LCD_o,3);
		LCD_vDispChar(LCD_k,2);
     }
         
}

void disp_tc_time(unsigned char ttime)
{
    Eprom_Read(nonroll_init_add);
	
    if(ttime==0)                            //time display ttime=0
    { 
		// To display 'second' digits values swaped 
		opr_data[2]=opr_data[1];
		opr_data[1]=opr_data[0];
		opr_data[0]=0;
		LCD_DispRTC_Time_f = 1;
		time_bcd(&opr_data[0]);
		glow_T();
    }
    else
    {   
		LCD_DispRTC_Time_f=1;                                    //date display ttime=1
		time_bcd(&opr_data[2]);
#if ((LCD_TYPE == 1) || (LCD_TYPE == 3))
		LCD_vDispChar(LCD_d,0);
#else
		LCD_vDispChar(LCD_d,10);
#endif
    }
}

/*void disp_tc_cnt(void)
{
    u8 temp;
    Eprom_Read(nonroll_status);
    temp=opr_data[0];
    fill_oprzero();
    opr_data[0]=temp;
    disp_call(5);
//     LCDMEM[11]=LCD_c;       //c
//    LCDMEM[12]=LCD_t;       //t
}*/

void disp_defrd_mag(void)               // display the magnet defraud energy
{
    fill_oprzero();
    FUN_vfill_3byte(TPR_u32cum_mag_defraud,opr_data);
    bin_2_dec();
	disp_onlcd(1,6);

#if LCD_TYPE ==2
	LCD_vDispChar(LCD_e,9);
	LCD_vDispChar(LCD_n,8);
#endif
	glow_3p();
	glow_M();
	glow_k();
	glow_W();
	glow_h();
//    LCDMEM[12]=LCD_d;
//    LCDMEM[11]=LCD_Tab[1];
  
}

void disp_defrd_neutemp(void)           // display the neutral miss defraud energy
{
    fill_oprzero();
    FUN_vfill_3byte(TPR_u32cum_neutemp_defraud,opr_data);
    bin_2_dec();
	disp_onlcd(1,6);
	glow_3p();
#if LCD_TYPE ==2
	LCD_vDispChar(LCD_e,9);
	LCD_vDispChar(LCD_n,8);
#endif
	glow_N();
	glow_k();
	glow_W();
	glow_h();
  
}
void disp_mag_cnt(void)
{
    fill_oprzero();
//    FUN_vfill_2byte(TPRCNT_u8Mag,opr_data);
    opr_data[0]=TPRCNT_u8Mag;
    disp_call(3);

	LCD_vDispChar(0,5);   //blank display
	LCD_vDispChar(LCD_m,7);
	LCD_vDispChar(LCD_g,6);
	

#if ((LCD_TYPE == 1) || (LCD_TYPE == 3))
	LCD_vDispChar(LCD_c,0);
#else
	LCD_vDispChar(LCD_c,10);
#endif
	glow_T();
//    LCDMEM[12]=LCD_c;
//    LCDMEM[10]=LCD_m;
//    LCDMEM[9]=LCD_g;
//    LCDMEM[8]=0x40;

}
void disp_NeuMiss_cnt(void)
{
    fill_oprzero();
//    FUN_vfill_2byte(TPRCNT_u8NeuMiss,opr_data);
    opr_data[0]=TPRCNT_u8NeuMiss;
    disp_call(3);

	LCD_vDispChar(0,5);   //blank display
	LCD_vDispChar(LCD_n,7);
	LCD_vDispChar(LCD_m,6);

#if ((LCD_TYPE == 1) || (LCD_TYPE == 3))
	LCD_vDispChar(LCD_c,0);
#else
	LCD_vDispChar(LCD_c,10);
#endif
	
	glow_T();
//    LCDMEM[12]=LCD_c;
//    LCDMEM[10]=LCD_n;
//    LCDMEM[9]=LCD_m;
//    LCDMEM[8]=0x40;

}
void disp_VHigh_cnt(void)
{
    fill_oprzero();
//    FUN_vfill_2byte(TPRCNT_u8VHigh,opr_data);
    opr_data[0]=TPRCNT_u8VHigh;
    disp_call(3);

	LCD_vDispChar(0,5);   //blank display
	LCD_vDispChar(LCD_h,7);
	LCD_vDispChar(LCD_i,6);
	

#if ((LCD_TYPE == 1) || (LCD_TYPE == 3))
	LCD_vDispChar(LCD_c,0);
#else
	LCD_vDispChar(LCD_c,10);
#endif
	glow_T();
}
void disp_VLow_cnt(void)
{
    fill_oprzero();
//    FUN_vfill_2byte(TPRCNT_u8VLow,opr_data);
    opr_data[0]=TPRCNT_u8VLow;
    disp_call(3);

	LCD_vDispChar(0,5);   //blank display
	LCD_vDispChar(LCD_l,7);
	LCD_vDispChar(LCD_o,6);
	

#if ((LCD_TYPE == 1) || (LCD_TYPE == 3))
	LCD_vDispChar(LCD_c,0);
#else
	LCD_vDispChar(LCD_c,10);
#endif
	glow_T();

}
void disp_Rev_cnt(void)
{
    fill_oprzero();
//    FUN_vfill_2byte(TPRCNT_u8Rev,opr_data);
    opr_data[0]=TPRCNT_u8Rev;
    disp_call(3);

	LCD_vDispChar(0,5);   //blank display
	LCD_vDispChar(LCD_r,7);
	LCD_vDispChar(LCD_v,6);
	
#if ((LCD_TYPE == 1) || (LCD_TYPE == 3))
	LCD_vDispChar(LCD_c,0);
#else
	LCD_vDispChar(LCD_c,10);
#endif
	glow_T();
}
void disp_EL_cnt(void)
{
    fill_oprzero();
//    FUN_vfill_2byte(TPRCNT_u8EL,opr_data);
    opr_data[0]=TPRCNT_u8EL;
     disp_call(3);

	LCD_vDispChar(0,5);   //blank display
	LCD_vDispChar(LCD_e,7);
	LCD_vDispChar(LCD_l,6);
	
#if ((LCD_TYPE == 1) || (LCD_TYPE == 3))
	LCD_vDispChar(LCD_c,0);
#else
	LCD_vDispChar(LCD_c,10);
#endif

	glow_T();

}
void disp_OverLoad_cnt(void)
{
    fill_oprzero();
//    FUN_vfill_2byte(TPRCNT_u8OverLoad,opr_data);
    opr_data[0]=TPRCNT_u8OverLoad;
    disp_call(3);

	LCD_vDispChar(0,5);   //blank display
	LCD_vDispChar(LCD_o,7);
	LCD_vDispChar(LCD_l,6);
	
#if ((LCD_TYPE == 1) || (LCD_TYPE == 3))
	LCD_vDispChar(LCD_c,0);
#else
	LCD_vDispChar(LCD_c,10);
#endif
	glow_T();
}
void disp_NeuDis_cnt(void)
{
    fill_oprzero();
//    FUN_vfill_2byte(TPRCNT_u8NeuDis,opr_data);
    opr_data[0]=TPRCNT_u8NeuDis;
    disp_call(3);

	LCD_vDispChar(0,5);   //blank display
	LCD_vDispChar(LCD_n,7);
	LCD_vDispChar(LCD_d,6);
	
#if ((LCD_TYPE == 1) || (LCD_TYPE == 3))
	LCD_vDispChar(LCD_c,0);
#else
	LCD_vDispChar(LCD_c,10);
#endif
	glow_T();
}
void disp_FreqTamp_cnt(void)
{
    fill_oprzero();
//    FUN_vfill_2byte(TPRCNT_u8FreqTamp,opr_data);
    opr_data[0]=TPRCNT_u8FreqTamp;
    disp_call(3);

	LCD_vDispChar(0,5);   //blank display
	LCD_vDispChar(LCD_f,7);
	LCD_vDispChar(LCD_r,6);
	
#if ((LCD_TYPE == 1) || (LCD_TYPE == 3))
	LCD_vDispChar(LCD_c,0);
#else
	LCD_vDispChar(LCD_c,10);
#endif
	glow_T();
}
void disp_LowPF_cnt(void)
{
    fill_oprzero();
//    FUN_vfill_2byte(TPRCNT_u8LowPF,opr_data);
    opr_data[0]=TPRCNT_u8LowPF;
    disp_call(3);

	LCD_vDispChar(0,5);   //blank display
	LCD_vDispChar(LCD_p,7);
	LCD_vDispChar(LCD_f,6);
	
#if ((LCD_TYPE == 1) || (LCD_TYPE == 3))
	LCD_vDispChar(LCD_c,0);
#else
	LCD_vDispChar(LCD_c,10);
#endif
	glow_T();
}
void disp_OC_cnt(void)
{
    fill_oprzero();
//    FUN_vfill_2byte(TPRCNT_u8OC,opr_data);
    opr_data[0]=TPRCNT_u8OC;
    disp_call(3);

	LCD_vDispChar(0,5);   //blank display
	LCD_vDispChar(LCD_o,7);
	LCD_vDispChar(LCD_c,6);
	
#if ((LCD_TYPE == 1) || (LCD_TYPE == 3))
	LCD_vDispChar(LCD_c,0);
#else
	LCD_vDispChar(LCD_c,10);
#endif
	glow_T();
}
void disp_TC_cnt(void)
{
    fill_oprzero();
//    FUN_vfill_2byte(TPRCNT_u8TC,opr_data);
    opr_data[0]=TPRCNT_u8TC;
    disp_call(3);

	LCD_vDispChar(0,5);   //blank display
	LCD_vDispChar(LCD_o,7);
	LCD_vDispChar(LCD_p,6);
	
#if ((LCD_TYPE == 1) || (LCD_TYPE == 3))
	LCD_vDispChar(LCD_c,0);
#else
	LCD_vDispChar(LCD_c,10);
#endif
	glow_T();
}

void disp_cum_MDKW(u8 u8bmonth)
{
	u32 temp_CumMdkw=0;
	
    fill_oprzero();
	if(u8bmonth==0)
	{
    	FUN_vfill_3byte(MET_u32Cum_MDKW/10,opr_data);
	}
	else
	{
		temp_CumMdkw = CalCumMD(0);
		fill_oprzero();
		FUN_vfill_3byte(temp_CumMdkw/10,opr_data);
	}
	
    bin_2_dec();
#if LCD_TYPE != 3
	disp_onlcd(1,6);
#else
	disp_onlcd(3,5);
#endif
    glow_MD();
    glow_k();
    glow_W();
    glow_1p();
	
	if(u8bmonth!=0)
	{
		LCD_vDispValue(u8bmonth,0);
		glow_BP();
	}
	
#if LCD_TYPE == 1
	LCD_vDispChar(LCD_c,7);
#elif LCD_TYPE == 2
	LCD_vDispChar(LCD_c,9);
	LCD_vDispChar(LCD_u,8);
#endif

}
void disp_cum_MDKVA(u8 u8bmonth)
{
	u32 temp_CumMdkva=0;
	
    fill_oprzero();
	if(u8bmonth==0)
	{
    	FUN_vfill_3byte(MET_u32Cum_MDKVA/10,opr_data);
	}
	else
	{
		temp_CumMdkva = CalCumMD(1);
		fill_oprzero();
		FUN_vfill_3byte(temp_CumMdkva/10,opr_data);
	}
    bin_2_dec();
#if LCD_TYPE != 3
	disp_onlcd(1,6);
#else
	disp_onlcd(3,5);
#endif
    glow_MD();
    glow_k();
    glow_V();
    glow_A();
    glow_1p();
	
	if(u8bmonth!=0)
	{
		LCD_vDispValue(u8bmonth,0);
		glow_BP();
	}
	
#if LCD_TYPE == 1
	LCD_vDispChar(LCD_c,7);
#elif LCD_TYPE == 2
	LCD_vDispChar(LCD_c,9);
	LCD_vDispChar(LCD_u,8);
#endif
}


//void disp_cum_MDKW(void)
//{
//    fill_oprzero();
//    FUN_vfill_3byte(MET_u32Cum_MDKW/10,opr_data);
//    bin_2_dec();
//#if LCD_TYPE != 3
//	disp_onlcd(1,6);
//#else
//	disp_onlcd(3,5);
//#endif
//    glow_MD();
//    glow_k();
//    glow_W();
//    glow_1p();
//#if ((LCD_TYPE == 1) || (LCD_TYPE == 3))
//	LCD_vDispChar(LCD_c,0);
//#else
//	LCD_vDispChar(LCD_c,9);
//	LCD_vDispChar(LCD_u,8);
//#endif

//}
//void disp_cum_MDKVA(void)
//{
//    fill_oprzero();
//    FUN_vfill_3byte(MET_u32Cum_MDKVA/10,opr_data);
//    bin_2_dec();
//#if LCD_TYPE != 3
//	disp_onlcd(1,6);
//#else
//	disp_onlcd(3,5);
//#endif
//    glow_MD();
//    glow_k();
//    glow_V();
//    glow_A();
//    glow_1p();
//#if ((LCD_TYPE == 1) || (LCD_TYPE == 3))
//	LCD_vDispChar(LCD_c,0);
//#else
//	LCD_vDispChar(LCD_c,9);
//	LCD_vDispChar(LCD_u,8);
//#endif
//}

void disp_vTamperList(u8 u8List_no)
{
	switch(u8List_no)
	{
	case 7:
	case 8:
	disp_vDispString(LCD_OV_VOL,7);
	//        memcpy(&LCDMEM[5],LCD_HI_VOL,6);
	break;   
	case 9:
	case 10:
	disp_vDispString(LCD_LO_VOL,7);
	//        memcpy(&LCDMEM[5],LCD_LO_VOL,6);
	break;   
	case 51:
	case 52:
	disp_vDispString(LCD_CU_REV,7);
	//        memcpy(&LCDMEM[5],LCD_CU_REV,6);
	break;   
	case 67:
	case 68:
	disp_vDispString(LCD_OV_CU,7);
	//        memcpy(&LCDMEM[5],LCD_OV_CU,6);
	break;   
	case 69:
	case 70:
	disp_vDispString(LCD_EL,7);
	//        memcpy(&LCDMEM[5],LCD_EL  ,6);
	break;   
	case 71:
	case 72:
	disp_vDispString(LCD_OV_LD,7);
	//        memcpy(&LCDMEM[5],LCD_OV_LD,6);
	break;   
	case 201:                                       
	case 202:                                       
	disp_vDispString(LCD_MAG,7);						
	//        memcpy(&LCDMEM[5],LCD_MAG ,6);              
	break;   
	case 203:
	case 204:
	disp_vDispString(LCD_NE_DIS,7);
	//        memcpy(&LCDMEM[5],LCD_NE_DIS,6);
	break;   
	case 205:
	case 206:
	disp_vDispString(LCD_LO_PF,7);
	//        memcpy(&LCDMEM[5],LCD_LO_PF,6);
	break;   
	case 207:
	case 208:
	disp_vDispString(LCD_NE_MIS,7);
	//        memcpy(&LCDMEM[5],LCD_NE_MIS,6);
	break;   
	case 211:
	case 212:
	disp_vDispString(LCD_ABFRE,7);
	//        memcpy(&LCDMEM[5],LCD_ABFRE,6);
	break;   
	case 213:
	case 214:
	disp_vDispString(LCD_35KV,7);
	//        memcpy(&LCDMEM[5],LCD_35KV,6);
	break;   
	case 251:
	disp_vDispString(LCD_TC_OPN,7);
	//         memcpy(&LCDMEM[5],LCD_TC_OPN,6);
	break;  
	default:
	disp_vDispString(LCD_NO_TPR,7);
	//        memcpy(&LCDMEM[5],LCD_NO_TPR,6);
	break; 

	}    
}
void disp_frstT_str_typ(unsigned char value)
{
    Eprom_Read(FirstStrAddr);
    if(value == 0)
    {
		disp_vTamperList(opr_data[0]);

    }
    else
    {
        if(value==1)//time
        { 
			opr_data[0]=0;
			time_bcd(&opr_data[0]);
#if LCD_TYPE !=3
			LCD_vDispChar(LCD_t,1);
#endif
        }
        else
        {
			LCD_DispRTC_Time_f = 1;
            time_bcd(&opr_data[3]);
#if LCD_TYPE == 2
			LCD_vDispChar(LCD_d,10);
#elif LCD_TYPE == 1
			LCD_vDispChar(LCD_d,1);
#endif
        }

    }
#if LCD_TYPE ==2
	  LCD_vDispChar(LCD_f,9);
#else 
	  LCD_vDispChar(LCD_s,0);
	  //disp_zoneindex_f =0;
#endif
	 
}
void disp_frstT_restr_typ(unsigned char value)
{
    Eprom_Read(FirstRestrAddr);
    if(value == 0)
    {
        disp_vTamperList(opr_data[0]);
    }
    else
    {
        if(value==1)//time
        { 
			opr_data[0]=0;
            time_bcd(&opr_data[0]);
#if LCD_TYPE !=3
			LCD_vDispChar(LCD_t,1);
#endif
        }
        else
        {   
			LCD_DispRTC_Time_f = 1;
            time_bcd(&opr_data[3]);
#if LCD_TYPE ==2
			LCD_vDispChar(LCD_d,10);
#elif LCD_TYPE == 1
			LCD_vDispChar(LCD_d,1);
#endif
        }

    }
#if LCD_TYPE ==2
		LCD_vDispChar(LCD_f,9);
#else 
		LCD_vDispChar(LCD_r,0);
		//disp_zoneindex_f =0;
#endif

}
void disp_lastT_str_typ(unsigned char value)
{
    Eprom_Read(LastStrAddr);
    if(value == 0)
    {
        disp_vTamperList(opr_data[0]);
#if LCD_TYPE == 2
		LCD_vDispChar(LCD_s,8);
		LCD_vDispChar(LCD_l,9);
#else
		LCD_vDispChar(LCD_o,0);
		glow_I();
#endif
    }
    else
    {
        if(value==1)//time
        { 
			opr_data[0]=0;
            time_bcd(&opr_data[0]);
            
#if LCD_TYPE == 2
		LCD_vDispChar(LCD_s,8);
		LCD_vDispChar(LCD_l,9);
#elif LCD_TYPE == 1
		LCD_vDispChar(LCD_t,1);
		LCD_vDispChar(LCD_s,0);
		glow_I();
#elif LCD_TYPE == 3
		LCD_vDispChar(LCD_o,0);
		glow_I();
#endif

        }
        else
        {   LCD_DispRTC_Time_f = 1; 
            time_bcd(&opr_data[3]);
#if LCD_TYPE ==2
		LCD_vDispChar(LCD_d,10);
		LCD_vDispChar(LCD_s,8);
		LCD_vDispChar(LCD_l,9);
#elif LCD_TYPE == 1
		LCD_vDispChar(LCD_d,1);
		LCD_vDispChar(LCD_o,0);
		glow_I();
#elif LCD_TYPE == 3
		LCD_vDispChar(LCD_o,0);
		glow_I();
#endif
        }

    }
}
void disp_lastT_restr_typ(unsigned char value)
{
    Eprom_Read(LastRestrAddr);
    if(value == 0)
    {
        disp_vTamperList(opr_data[0]);
#if LCD_TYPE == 2
		LCD_vDispChar(LCD_l,9);
		LCD_vDispChar(LCD_r,8);
#else 
		glow_I();
		LCD_vDispChar(LCD_r,0);
#endif
    }
    else
    {
        if(value==1)//time
        { 
			opr_data[0]=0;
            time_bcd(&opr_data[0]);
#if LCD_TYPE == 2
			LCD_vDispChar(LCD_l,9);
			LCD_vDispChar(LCD_r,8);
#elif LCD_TYPE == 1
			LCD_vDispChar(LCD_t,1);
			LCD_vDispChar(LCD_r,0);
			glow_I();
#elif LCD_TYPE == 3
			LCD_vDispChar(LCD_r,0);
			glow_I();
#endif
        }
        else
        {   
			LCD_DispRTC_Time_f = 1;
            time_bcd(&opr_data[3]);
#if LCD_TYPE ==2
			LCD_vDispChar(LCD_d,10);
			LCD_vDispChar(LCD_l,9);
			LCD_vDispChar(LCD_r,8);
#elif LCD_TYPE == 1
			LCD_vDispChar(LCD_d,1);
			LCD_vDispChar(LCD_r,0);
			glow_I();
#elif LCD_TYPE == 3
			LCD_vDispChar(LCD_r,0);
			glow_I();
#endif
        }

    }
}
void disp_ShowT8(void)
{
	
	if((P2 & BIT0)!=0)
	{
		if(rtc_calib_f==1)
		{
#if LCD_TYPE == 1 
			if(DISP_bToggle)
			{
				LCD_vDispValue(8,0);
				glow_T();
			}
			else
			{
				SEG5 &=0xfe;
				SEG4 &=0xf7;
				SEG7 &= 0x01;
				SEG8 &= 0x00;
				
			}
#endif

#if LCD_TYPE == 2
			if(DISP_bToggle)
			{
				LCD_vDispValue(8,10);
				glow_T();
			}
			else
			{
				SEG22 = 0x00;
				SEG23 = 0x00;
			}
			
#endif

#if LCD_TYPE == 3 
			if(DISP_bToggle)
			{
				LCD_vDispValue(8,0);
				glow_T();
			}
			else
			{
				SEG8 &=0xfe;
				SEG10 &=0xfe;
				SEG6 &= 0x01;
				SEG7 &= 0x00;
			}
#endif
		}
		else if(disp_zoneindex_f == 1)
		{
#if ((LCD_TYPE == 1) || (LCD_TYPE == 3))
			LCD_vDispValue(TOD_u8zone_index,0);
#else
			LCD_vDispValue(TOD_u8zone_index,10);
#endif
			glow_T();
		}
	}
}

void disp_TamperCnt(unsigned char Tpr_ID)
{
	unsigned char Disp_TprCount;
	unsigned char temp_disp1, temp_disp2;
	fill_oprzero();
	switch(Tpr_ID)
	{
		case 9:
		case 10:
		Disp_TprCount=TPRCNT_u8VLow;
		temp_disp1 = LCD_l ;
		temp_disp2 = LCD_o ;
		break;
		
		case 51:
		case 52:
		Disp_TprCount=TPRCNT_u8Rev;
		temp_disp1 = LCD_r ;
		temp_disp2 = LCD_v ;
		break;
		
		case 67:
		case 68:
		Disp_TprCount=TPRCNT_u8OC;
		temp_disp1 = LCD_o ;
		temp_disp2 = LCD_c ;
		break;
		
		case 69:
		case 70:
		Disp_TprCount=TPRCNT_u8EL;
		temp_disp1 = LCD_e ;
		temp_disp2 = LCD_l ;		
		break;

		case 71:
		case 72:
		Disp_TprCount=TPRCNT_u8OverLoad;
		temp_disp1 = LCD_o ;
		temp_disp2 = LCD_l ;
		break;
		
		case 201:
		case 202:
		Disp_TprCount=TPRCNT_u8Mag;
		temp_disp1 = LCD_m ;
		temp_disp2 = LCD_g ;
		break;
		
		case 203:
		case 204:
		Disp_TprCount=TPRCNT_u8NeuDis;
		temp_disp1 = LCD_n ;
		temp_disp2 = LCD_d ;
		break;
		
		case 207:
		case 208:
		Disp_TprCount=TPRCNT_u8NeuMiss;
		temp_disp1 = LCD_n ;
		temp_disp2 = LCD_m ;
		break;

		case 213:
		case 214:
		Disp_TprCount=TPRCNT_u835kv;
		temp_disp1 = LCD_h ;
		temp_disp2 = LCD_v ;
		break;
		
		case 251:
		case 252:
		Disp_TprCount=TPRCNT_u8TC;
		temp_disp1 = LCD_c ;
		temp_disp2 = LCD_o ;
		break;
		
		default:
		Disp_TprCount=0;
		break;
	}
	
	opr_data[0] = Disp_TprCount;
	disp_call(3);
	LCD_vDispChar(0,5);
	LCD_vDispChar(temp_disp1,7);
	LCD_vDispChar(temp_disp2,6);


#if ((LCD_TYPE == 1) || (LCD_TYPE == 3))
	LCD_vDispChar(LCD_c,0);
#else
	LCD_vDispChar(LCD_c,10);
#endif
	glow_T();
}


void push_disp_par(void)
{ 
	if(SW_u8bat_disp_c > LCD_u8Max_PUSH)    
		SW_u8bat_disp_c=1;
	fill_oprzero(); 
	//lcd_disp_off();
	disp_list(lcd_a8PushList[SW_u8bat_disp_c-1]); 
}
/***********************************************************************************************************************
* Function Name: time_bcd
* Description  : This function disables voltage boost circuit or capacitor split circuit.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void battpush_disp_par(void)
{
	//const u8 a8PushList[]={24,48,152,200};//last be first as 200
	if(SW_u8bat_disp_c > LCD_u8Max_AUTO)    
		SW_u8bat_disp_c=1;
	fill_oprzero(); 
	//lcd_disp_off();
	disp_list(lcd_a8AutoList[SW_u8bat_disp_c-1]);
}
/***********************************************************************************************************************
* Function Name: time_bcd
* Description  : This function disables voltage boost circuit or capacitor split circuit.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void disp_par(void)
{
	if(SW_u8disp_cntr>LCD_u8Max_AUTO-1)
		SW_u8disp_cntr=0;
	fill_oprzero();
	//lcd_disp_off();
	disp_list(lcd_a8AutoList[SW_u8disp_cntr]);
}
/***********************************************************************************************************************
* Function Name: disp_list
* Description  : This function disables voltage boost circuit or capacitor split circuit.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void disp_list(u8 u8para)
{
	disp_zoneindex_f = 0;
    switch(u8para)                         
    {
		case 0:
		lcd_disp_all(); 
		//disp_zoneindex_f=0;
		break;
		case 1:
		disp_srno();
		//disp_zoneindex_f = 0;
		break; 
		case 2:
		LCD_DispRTC_Time_f = 1;
		time_bcd(&dt.sec);
		glow_T(); 
		break;
		case 3:
		LCD_DispRTC_Time_f = 1;
		time_bcd(&dt.year);
#if ((LCD_TYPE == 1) || (LCD_TYPE == 3))
		//disp_zoneindex_f =0;
		LCD_vDispChar(LCD_d,0);
#else
		LCD_vDispChar(LCD_d,10);
#endif
		break;
		case 4:
		disp_v();
		break;                
		case 5:
		disp_ip(1);
		break;                
		case 6:
		disp_ip(2);
		break; 
		case 7: 
		disp_freq();
#if LCD_TYPE == 1
		//disp_zoneindex_f =0;
#endif
		break;		  
		case 8:
		disp_pf();
		break;	  
		case 9:
		disp_act_power(3);
		break; 
		case 10:
		disp_app_power();
		break; 
		case 11:
		disp_kwh();
		break;        
		case 12:
		disp_kvah();
		break;
		case 13:
		disp_md1(0);       
		break;
		case 14:
		md1_time(0,1);
		break;  
		case 15:
		md1_time(0,0);
		break;	
		case 16:
		disp_md_kVA(0);       
		break;
		case 17:
		mdkVA_time(0,1);
		break;  
		case 18:
		mdkVA_time(0,0);
		break;	
		case 19:
		disp_avg_pf(0);                     //avg pf
		break;
		case 20:
		disp_cum_MDKW(0);                    //Cumulative MD kW
		break;  
		case 21:
		disp_cum_MDKVA(0);                   //Cumulative MD kVA
		break;  
		case 22:
		disp_hires_kwh();                        //High Resolution kWh
		break;  
		case 23:
		disp_hires_kvah();                        //High Resolution KVAh
		break;  
		case 24:
		disp_bkwh1(1);
		break;              
		case 25:
		disp_bkwh1(2);
		break;               
		case 26:
		disp_bkwh1(3);
		break;               
		case 27:
		disp_bkwh1(4);
		break;               
		case 28:
		disp_bkwh1(5);
		break;        
		case 29:
		disp_bkwh1(6);
		break; 
		case 30:
		disp_bkwh1(7);
		break; 
		case 31:
		disp_bkwh1(8);
		break; 
		case 32:
		disp_bkwh1(9);
		break; 
		case 33:
		disp_bkwh1(10);
		break; 
		case 34:
		disp_bkwh1(11);
		break; 
		case 35:
		disp_bkwh1(12);
		break;  	  
		case 36:
		disp_bkvah1(1);
		break;              
		case 37:
		disp_bkvah1(2);
		break;               
		case 38:
		disp_bkvah1(3);
		break;               
		case 39:
		disp_bkvah1(4);
		break;               
		case 40:
		disp_bkvah1(5);
		break;        
		case 41:
		disp_bkvah1(6);
		break; 
		case 42:
		disp_bkvah1(7);
		break; 
		case 43:
		disp_bkvah1(8);
		break; 
		case 44:
		disp_bkvah1(9);
		break; 
		case 45:
		disp_bkvah1(10);
		break; 
		case 46:
		disp_bkvah1(11);
		break; 
		case 47:
		disp_bkvah1(12);
		break;  	  
		case 48:
		disp_md1(1);
		break;	  	  
		case 49:
		md1_time(1,1);
		break;	
		case 50:
		md1_time(1,0);
		break;  
		case 51:
		disp_md1(2);
		break; 
		case 52:
		md1_time(2,1);
		break; 
		case 53:
		md1_time(2,0);
		break;	   
		case 54:
		disp_md1(3);
		break; 
		case 55:
		md1_time(3,1);
		break; 
		case 56:
		md1_time(3,0);
		break;     
		case 57:
		disp_md1(4);
		break;
		case 58:
		md1_time(4,1);
		break; 
		case 59:
		md1_time(4,0);
		break;	    
		case 60:
		disp_md1(5);
		break; 
		case 61:
		md1_time(5,1);
		break;
		case 62:
		md1_time(5,0);
		break;    
		case 63:
		disp_md1(6);
		break; 
		case 64:
		md1_time(6,1);
		break; 
		case 65:
		md1_time(6,0);
		break;     
		case 66: 
		disp_md1(7);
		break;
		case 67:
		md1_time(7,1);                      //id
		break;
		case 68:
		md1_time(7,0);                      //id	  
		break;    
		case 69:
		disp_md1(8);
		break;
		case 70:
		md1_time(8,1);                      //id
		break;
		case 71:
		md1_time(8,0);                      //id
		break;   
		case 72:
		disp_md1(9);
		break;
		case 73:
		md1_time(9,1);                      //id
		break;
		case 74:
		md1_time(9,0);                      //id
		break;    
		case 75:
		disp_md1(10);
		break;
		case 76:
		md1_time(10,1);                     //id
		break;
		case 77:
		md1_time(10,0);                     //id
		break;    
		case 78: 
		disp_md1(11);
		break; 
		case 79:
		md1_time(11,1);                     //id
		break;
		case 80:
		md1_time(11,0);                     //id
		break;     
		case 81: 
		disp_md1(12);
		break;	  
		case 82:
		md1_time(12,1);                     //id
		break;
		case 83:
		md1_time(12,0);                     //id
		break ;    
		case 84:
		disp_md_kVA(1);       
		break;
		case 85:
		mdkVA_time(1,1);
		break;  
		case 86:
		mdkVA_time(1,0);
		break;	  
		case 87:
		disp_md_kVA(2);       
		break;
		case 88:
		mdkVA_time(2,1);
		break;  
		case 89:
		mdkVA_time(2,0);
		break;  
		case 90:
		disp_md_kVA(3);       
		break;
		case 91:
		mdkVA_time(3,1);
		break;  
		case 92:
		mdkVA_time(3,0);
		break;	  
		case 93:
		disp_md_kVA(4);       
		break;
		case 94:
		mdkVA_time(4,1);
		break;  
		case 95:
		mdkVA_time(4,0);
		break;	  
		case 96:
		disp_md_kVA(5);       
		break;
		case 97:
		mdkVA_time(5,1);
		break;  
		case 98:
		mdkVA_time(5,0);
		break;	   
		case 99:
		disp_md_kVA(6);       
		break;
		case 100:
		mdkVA_time(6,1);
		break;  
		case 101:
		mdkVA_time(6,0);
		break;	  
		case 102:
		disp_md_kVA(7);       
		break;
		case 103:
		mdkVA_time(7,1);
		break;  
		case 104:
		mdkVA_time(7,0);
		break;  
		case 105:
		disp_md_kVA(8);       
		break;
		case 106:
		mdkVA_time(8,1);
		break;  
		case 107:
		mdkVA_time(8,0);
		break;	  
		case 108:
		disp_md_kVA(9);       
		break;
		case 109:
		mdkVA_time(9,1);
		break;  
		case 110:
		mdkVA_time(9,0);
		break;   
		case 111:
		disp_md_kVA(10);       
		break;
		case 112:
		mdkVA_time(10,1);
		break;  
		case 113:
		mdkVA_time(10,0);
		break;	   
		case 114:
		disp_md_kVA(11);       
		break;
		case 115:
		mdkVA_time(11,1);
		break;  
		case 116:
		mdkVA_time(11,0);
		break  ;
		case 117:
		disp_md_kVA(12);       
		break;
		case 118:
		mdkVA_time(12,1);
		break;  
		case 119:
		mdkVA_time(12,0);
		break;  
		case 120:
		bill_time(1,1);
		break;	
		case 121:
		bill_time(1,0);
		break;
		case 122:
		bill_time(2,1);
		break;	
		case 123:
		bill_time(2,0);
		break;  
		case 124:
		bill_time(3,1);
		break;	
		case 125:
		bill_time(3,0);
		break;  
		case 126:
		bill_time(4,1);
		break;	
		case 127:
		bill_time(4,0);
		break;
		case 128:
		bill_time(5,1);
		break;	
		case 129:
		bill_time(5,0);
		break;
		case 130:
		bill_time(6,1);
		break;	
		case 131:
		bill_time(6,0);
		break;
		case 132:
		bill_time(7,1);
		break;	
		case 133:
		bill_time(7,0);
		break;
		case 134:
		bill_time(8,1);
		break;	
		case 135:
		bill_time(8,0);
		break;
		case 136:
		bill_time(9,1);
		break;	
		case 137:
		bill_time(9,0);
		break;
		case 138:
		bill_time(10,1);
		break;	
		case 139:
		bill_time(10,0);
		break;
		case 140:
		bill_time(11,1);
		break;	
		case 141:
		bill_time(11,0);
		break;
		case 142:
		bill_time(12,1);
		break;	
		case 143:
		bill_time(12,0);
		break;
		case 144:
		disp_ZKW(1);                    //zone1 kwh
		break;
		case 145:
		disp_ZKW(2);                    //zone2 kwh
		break;
		case 146:
		disp_ZKW(3);                    //zone3 kwh
		break;
		case 147:
		disp_ZKW(4);                    //zone4 kwh
		break;
		case 148:
		disp_ZKW(5);                    //zone5 kwh
		break;
		case 149:
		disp_ZKW(6);                    //zone6 kwh
		break;
		case 150:
		disp_ZKW(7);                    //zone7 kwh
		break;
		case 151:
		disp_ZKW(8);                    //zone8 kwh
		break;
		case 152:
		disp_avg_billpf(1);             //avg pf
		break;
		case 153:
		disp_avg_billpf(2);             //avg pf
		break;
		case 154:
		disp_avg_billpf(3);             //avg pf
		break;
		case 155:
		disp_avg_billpf(4);             //avg pf
		break;
		case 156:
		disp_avg_billpf(5);             //avg pf
		break;
		case 157:
		disp_avg_billpf(6);             //avg pf
		break; 
		case 158:
		disp_avg_billpf(7);             //avg pf
		break;
		case 159:
		disp_avg_billpf(8);             //avg pf
		break;
		case 160:
		disp_avg_billpf(9);             //avg pf
		break;
		case 161:
		disp_avg_billpf(10);                //avg pf
		break;
		case 162:
		disp_avg_billpf(11);                //avg pf
		break;
		case 163:
		disp_avg_billpf(12);                //avg pf
		break;
		
//		case 164:
//		lcd_disp_off(); //current month power on hrs
//		disp_bpon_min(0); 
//		break;
		
		case 165:
		lcd_disp_off();
		disp_bpoff_min(0);                //current month power off hes
		break;
		
//		case 166:
//		lcd_disp_off(); //last month power on hrs
//		disp_bpon_min(1); 
//		break;

		case 167:
		lcd_disp_off();
		disp_bpoff_min(1);               //last month power off hrs
		break;
		
//		case 168:
//		lcd_disp_off(); //display meter status
//		break;
		
		case 169: 
		disp_code(1);                      //display code1
		break;
		case 170:
		disp_code(2);                      //display code2
		break;
		case 171:
		disp_code(3);                          //display code3
		break;
		case 172:
		disp_code(4);                      //display code4
		break;
		
//		case 173:
//		lcd_disp_off(); 
//		cum power on time
//		break;
		
		case 174:
		lcd_disp_off();
		disp_PowerFailureDuration();     //cum power off time
		break;
		
		case 175:
		//disp_zoneindex_f=0;
		disp_billpcount();                 //md reset count
		break;
		
		case 176:
		disp_frstT_str_typ(0);               //first tamper store type
		break;
		
		case 177:
		disp_frstT_str_typ(2);               //first tamper store date
		break;
		
		case 178:
		disp_frstT_str_typ(1);               //first tamper store time
		break;
		
		case 179:
		disp_frstT_restr_typ(0);                 //first tamper restore  type
		break;
		
		case 180:
		disp_frstT_restr_typ(2);                 //first tamper restore date
		break;
		
		case 181:
		disp_frstT_restr_typ(1);                 //first tamper restore  time
		break;
		
		case 182:
		disp_lastT_str_typ(0);                   //last tamper store type
		break;
		
		case 183:
		disp_lastT_str_typ(2);                   //last tamper store date
		break;
		
		case 184:
		disp_lastT_str_typ(1);                   //last tamper store time
		break;
		
		case 185:
		disp_lastT_restr_typ(0);                 //last tamper store type
		break;
		
		case 186:
		disp_lastT_restr_typ(2);                 //last tamper store date
		break;
		
		case 187:
		disp_lastT_restr_typ(1);                 //last tamper store time
		break;
		
		case 188:
		disp_TamperCnt(201);                     // magnetic temper count
		break;
		
		case 189:
		disp_TamperCnt(207);                 //neutral miss count
		break;
		
		case 190:
		disp_TamperCnt(203);  //neutral disturbance count
		break; 
		
		case 191:
		disp_TamperCnt(51);  //  CT Reverse Count
		break;
		
//		case 192:
//		//disp_TamperCnt(0);
//		//disp_FreqTamp_cnt();                //Frequency Variation Count
//		break;
		
//		case 193:
//		//disp_TamperCnt(0);
//		//disp_VHigh_cnt();                  //High Voltage Count
//		break;
		
		case 194:
		disp_TamperCnt(9);  //Low Voltage Count
		break;
		
		case 195:
		disp_TamperCnt(71);         //Over Load Count
		break;
		
		case 196:
		disp_TamperCnt(251);                //Top Cover Open Tamper Count
		break;
		
//		case 197:
//		//disp_TamperCnt(0);
//		//disp_LowPF_cnt();                  //Low Pf Count
//		break;
		
		case 198:
		disp_TamperCnt(69);           //earth load count
		break;
		
		case 199:
		disp_TamperCnt(67);          //over current count
		break;
		
		case 200:
		//disp_zoneindex_f=0;
		disp_curnt_btprcont();             //Bill Tamper Count
		break;
		
		case 201:
		disp_tcnt();                     ///Cumulative Tamper Count
		break;
		
		case 202: 
		if(TPRCNT_u8Mag>0)
		{
			 disp_vDispString(LCD_MAG,7); 
		}
		else
		{
		SW_u8disp_cntr++;
		SW_u8bat_disp_c++;
		DISP_bParCng=1;
		}
		break;
		
		case 203:
		if(TPR_bLowPF==1)
		{
		 disp_vDispString(LCD_LO_PF,7); 
		break;
		}
		else
		{
		SW_u8disp_cntr++;
		SW_u8bat_disp_c++;
		DISP_bParCng=1;
		break;
		}
		case 204:
		disp_tc_status();                //Top cover Tamper Status
		break;
		
		case 205:
		disp_tc_time(1);             //Top cover Tamper Date
		glow_OP();
		break;
		case 206:
		disp_tc_time(0);             //Top cover Tamper Time
		glow_OP();
		break;
		case 207:
		lcd_disp_off();
		disp_eprom_status();        //Memory OK/ Not OK
		break;
		case 208:
		lcd_disp_off();
		disp_bat_status();          //Battery OK/ Not OK
		break;
		case 209:
		lcd_disp_off(); 
		disp_rtc_status();          //RTC OK/Not OK
		break;
		case 210:
		disp_defrd_mag();           //Defraud Energy Magnet
		break;
		case 211:
		disp_defrd_neutemp();       //Defraud Energy Neutral miss/disturbance
		break;
//		case 212:
//		//disp_NoPowerFailureCnt();       //power failure count
//		break;  
//		case 213:
//      disp_ProgCnt();                 //programing count
//		break;	 
		case 214:
		//disp_zoneindex_f=0;        // Blank display
		lcd_disp_off();            
		break;	
		
		
		case 215:
		disp_cum_MDKW(1);
		break ;
		
		case 216:
		disp_cum_MDKVA(1);
		break ;
		
		
		case 217:
		//Active channel current
		disp_ip(3);
		break ;

		case 218:
		disp_ZKVA(1);          // Zone 1  Kvah
		break ;
		
		case 219:
		disp_ZKVA(2);          // Zone 2  Kvah
		break ;
		
		case 220:
		disp_ZKVA(3);          // Zone 3  Kvah
		break ;
		
		case 221:
		disp_ZKVA(4);          // Zone 4  Kvah
		break ;
		
		case 222:
		disp_ZKVA(5);          // Zone 5  Kvah
		break ;
		
		case 223:
		disp_ZKVA(6);          // Zone 6  Kvah
		break ;
		
		case 224:
		disp_ZKVA(7);          // Zone 7  Kvah
		break ;
		
		case 225:
		disp_ZKVA(8);          // Zone 8  Kvah
		break ;  
		
		case 226:
		if(TPRCNT_u835kv>0 || TPRCNT_u8NeuMiss>0 || TPRCNT_u8NeuDis >0 || TPRCNT_u8FreqTamp>0 || TPRCNT_u8EL>0 || TPRCNT_u8TC>0)
		{
			disp_vDispString(LCD_TPR,7);
		}
		else
		{
			disp_vDispString(LCD_NO_TPR,7);
		}
		break;
		
		case 227:
		if(TPRCNT_u835kv>0)
		{
			disp_vDispString(LCD_35KV,7); 
		}
		else
		{
		SW_u8disp_cntr++;
		SW_u8bat_disp_c++;
		DISP_bParCng=1;
		}
		break;
		
		case 228:
		if(TPRCNT_u8NeuMiss>0)
		{
			 disp_vDispString(LCD_NE_MIS,7); 
		}
		else
		{
		SW_u8disp_cntr++;
		SW_u8bat_disp_c++;
		DISP_bParCng=1;
		}
		break;
		
		
		case 229:
		if(TPRCNT_u8NeuDis >0)
		{
			disp_vDispString(LCD_NE_DIS,7);
		}
		else
		{
		SW_u8disp_cntr++;
		SW_u8bat_disp_c++;
		DISP_bParCng=1;
		}
		break;
		
		case 230:
		if(TPRCNT_u8FreqTamp>0)
		{
		  disp_vDispString(LCD_ABFRE,7);
		}
		else
		{
		SW_u8disp_cntr++;
		SW_u8bat_disp_c++;
		DISP_bParCng=1;
		}
		break;
		
		case 231:
		if(TPRCNT_u8EL>0)
		{
		  disp_vDispString(LCD_EL,7);
		}
		else
		{
		SW_u8disp_cntr++;
		SW_u8bat_disp_c++;
		DISP_bParCng=1;
		}
		break;
		
		case 232:
		if(TPRCNT_u8Rev>0)
		{
		  disp_vDispString(LCD_CU_REV,7);
		}
		else
		{
		SW_u8disp_cntr++;
		SW_u8bat_disp_c++;
		DISP_bParCng=1;
		}
		break;
		
		case 233:
		if(TPRCNT_u8TC>0)
		{
		  disp_vDispString(LCD_TC_OPN,7);
		}
		else
		{
		SW_u8disp_cntr++;
		SW_u8bat_disp_c++;
		DISP_bParCng=1;
		}
		break;	
		
		case 234:
		disp_act_power(0);    // phase
		break; 
		
		case 235:
		disp_act_power(1);   // neutral
		break;
		
		default :  disp_kwh();
		break;
    }
}