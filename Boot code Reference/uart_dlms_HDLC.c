
/*******************************************************************************
* File Name   :  dlms.c
* Module      :  Uart for HDLC(DLMS)
* Description :  All routines related to DLMS
* Author      :  Shishir Chowdhary & Harshita Jain
* Company     :  Genus Power Infrastructures Ltd., Jaipur
******************************************************************************/


/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "r_cg_macrodriver.h"
#include "variable.h"
#include "TI_aes.h"
#include "lcd.h"
#include "dlms.h"
#include "Eprom_i2c.h"
#include "function.h"
#include "meterology.h"
#include "bill.h"
#include "dailyenergy.h"
#include "loadsurvey.h"
#include "tamper.h"
#include "string.h"
#include "r_cg_serial.h"
#include "r_cg_wdt.h"
#include "LPM.h"
#include "Flash_Protocol.h"
#include "function.h"

/***********************************************************************************************************************
Pragma directive
***********************************************************************************************************************/
#pragma interrupt UART_iOpticalRx_Interrupt(vect = INTSR0)
#pragma interrupt UART_iOpticalTx_Interrupt(vect = INTST0)
#pragma interrupt UART_iOpticalInterrupt_Error(vect = INTSRE0)

#pragma interrupt UART_iRJRx_Interrupt(vect = INTSR1)
#pragma interrupt UART_iRJTx_Interrupt(vect = INTST1)
#pragma interrupt UART_iRJInterrupt_Error(vect = INTSRE1)

#pragma interrupt UART_IRDA_Rx_Interrupt(vect = INTSR2)
#pragma interrupt UART_IRDA_Tx_Interrupt(vect = INTST2)
#pragma interrupt UART_IRDA_Interrupt_Error(vect = INTSRE2)

/*******************************************************************************
* Global Variables
******************************************************************************/
static unsigned char const LCD_BOOT[] = {0x00,0x00,LCD_t,LCD_o_1,LCD_o_1,LCD_b,0x00};
/*******************************************************************************
* Local Static Variables
******************************************************************************/
/*******************************************************************************
* Constant Definitions
******************************************************************************/
#define PPPINITFCS16 0xFFFF
#define PPPGOODFCS16 0xF0B8
#define IrDA 0

void UART_vInit(void)
{
	SAU0EN = 1;//enable module     
	__nop();
	__nop();
	__nop();
	__nop();

#if IrDA == 1	
    SAU1EN = 1;//enable module
	__nop();
	__nop();
	__nop();
	__nop();
#endif 
	
#if X1_FREQ == 0	
	SPS0 =  _0004_SAU_CK00_FCLK_4 | _0070_SAU_CK01_FCLK_7;//_000A_SAU_CK00_FCLK_10;	
#elif X1_FREQ == 1	
	SPS0 =  _0003_SAU_CK00_FCLK_3 | _0070_SAU_CK01_FCLK_7;//_000A_SAU_CK00_FCLK_10;
#elif X1_FREQ == 2
	SPS0 =  _0002_SAU_CK00_FCLK_2 | _0070_SAU_CK01_FCLK_7;//_000A_SAU_CK00_FCLK_10;
#elif X1_FREQ == 3
	SPS0 =  _0001_SAU_CK00_FCLK_1 | _0070_SAU_CK01_FCLK_7;//_000A_SAU_CK00_FCLK_10;
  #if IrDA == 1	
	SPS1 =  _0001_SAU_CK00_FCLK_1 | _0070_SAU_CK01_FCLK_7;
  #endif
#endif
	ST0 |= _0002_SAU_CH1_STOP_TRG_ON | _0001_SAU_CH0_STOP_TRG_ON | _0004_SAU_CH2_STOP_TRG_ON | _0008_SAU_CH3_STOP_TRG_ON ;	/* disable UART2 receive and transmit */
	
#if IrDA == 1	
    ST1 |= _0002_SAU_CH1_STOP_TRG_ON | _0001_SAU_CH0_STOP_TRG_ON ;  
#endif	
	
	STMK0 = 1U;	/* disable INTST0 interrupt TX*/
	STIF0 = 0U;	/* clear INTST0 interrupt flag TX*/
	SRMK0 = 1U;	/* disable INTSR0 interrupt RX*/
	SRIF0 = 0U;	/* clear INTSR0 interrupt flag RX*/
	SREMK0 = 1U;	/* disable INTSRE0 interrupt Err*/
	SREIF0 = 0U;	/* clear INTSRE0 interrupt flag Err*/
#if RJ == 1
    STMK1 = 1U;    /* disable INTST1 interrupt */
    STIF1 = 0U;    /* clear INTST1 interrupt flag */
    SRMK1 = 1U;    /* disable INTSR1 interrupt */
    SRIF1 = 0U;    /* clear INTSR1 interrupt flag */
    SREMK1 = 1U;   /* disable INTSRE1 interrupt */
    SREIF1 = 0U;   /* clear INTSRE1 interrupt flag */	
#endif	
#if IrDA == 1	
	STMK2 = 1U;	/* disable INTST2 interrupt TX*/
	STIF2 = 0U;	/* clear INTST2 interrupt flag TX*/
	SRMK2 = 1U;	/* disable INTSR2 interrupt RX*/
	SRIF2 = 0U;	/* clear INTSR2 interrupt flag RX*/
	SREMK2 = 1U;	/* disable INTSRE2 interrupt Err*/
	SREIF2 = 0U;	/* clear INTSRE2 interrupt flag Err*/
#endif		
	/* Set INTST0 low priority */
	STPR00 = 1U;
	STPR10 = 1U;
	/* Set INTSR0 low priority */
	SRPR00 = 1U;
	SRPR10 = 1U;
	/* Set INTSRE0 low priority */
	SREPR00 = 1U;
	SREPR10 = 1U;
#if RJ == 1
        STPR11 = 1U;
        STPR01 = 1U;
        SRPR11 = 1U;
        SRPR01 = 1U;
        SREPR11 = 1U;
        SREPR01 = 1U;
#endif    

#if IrDA == 1	
	/* Set INTST2 low priority */
	STPR02 = 1U;
	STPR12 = 1U;
	/* Set INTSR2 low priority */
	SRPR02 = 1U;
	SRPR12 = 1U;
	/* Set INTSRE2 low priority */
	SREPR02 = 1U;
	SREPR12 = 1U;
#endif	
	
	SMR00 = _0020_SAU_SMRMN_INITIALVALUE | _0000_SAU_CLOCK_SELECT_CK00 | _0000_SAU_TRIGGER_SOFTWARE | _0002_SAU_MODE_UART | _0000_SAU_TRANSFER_END;
	SCR00 = _8000_SAU_TRANSMISSION | _0000_SAU_INTSRE_MASK | _0000_SAU_PARITY_NONE | _0080_SAU_LSB + _0010_SAU_STOP_1 + _0007_SAU_LENGTH_8;
	SDR00 = _9A00_UART2_TRANSMIT_DIVISOR;
	NFEN0 |= _01_SAU_RXD0_FILTER_ON;
	SIR01 = _0004_SAU_SIRMN_FECTMN | _0002_SAU_SIRMN_PECTMN | _0001_SAU_SIRMN_OVCTMN;	/* clear error flag */
	SMR01 = _0020_SAU_SMRMN_INITIALVALUE | _0000_SAU_CLOCK_SELECT_CK00 | _0100_SAU_TRIGGER_RXD | _0000_SAU_EDGE_FALL | _0002_SAU_MODE_UART | _0000_SAU_TRANSFER_END;
	SCR01 = _4000_SAU_RECEPTION | _0400_SAU_INTSRE_ENABLE | _0000_SAU_PARITY_NONE | _0080_SAU_LSB | _0010_SAU_STOP_1 | _0007_SAU_LENGTH_8;
	SDR01 = _9A00_UART2_RECEIVE_DIVISOR;
#if RJ == 1
	SMR02 = _0020_SAU_SMRMN_INITIALVALUE | _0000_SAU_CLOCK_SELECT_CK00 | _0000_SAU_TRIGGER_SOFTWARE | _0002_SAU_MODE_UART | _0000_SAU_TRANSFER_END;
	SCR02 = _8000_SAU_TRANSMISSION | _0000_SAU_INTSRE_MASK | _0000_SAU_PARITY_NONE | _0080_SAU_LSB + _0010_SAU_STOP_1 + _0007_SAU_LENGTH_8;
	SDR02 = _9A00_UART2_TRANSMIT_DIVISOR;
	NFEN0 |= _04_SAU_RXD1_FILTER_ON;
	SIR03 = _0004_SAU_SIRMN_FECTMN | _0002_SAU_SIRMN_PECTMN | _0001_SAU_SIRMN_OVCTMN;	/* clear error flag */
	SMR03 = _0020_SAU_SMRMN_INITIALVALUE | _0000_SAU_CLOCK_SELECT_CK00 | _0100_SAU_TRIGGER_RXD | _0000_SAU_EDGE_FALL | _0002_SAU_MODE_UART | _0000_SAU_TRANSFER_END;
	SCR03 = _4000_SAU_RECEPTION | _0400_SAU_INTSRE_ENABLE | _0000_SAU_PARITY_NONE | _0080_SAU_LSB | _0010_SAU_STOP_1 | _0007_SAU_LENGTH_8;
	SDR03 = _9A00_UART2_RECEIVE_DIVISOR;
#endif	
#if IrDA == 1	
	SMR10 = _0020_SAU_SMRMN_INITIALVALUE | _0000_SAU_CLOCK_SELECT_CK00 | _0000_SAU_TRIGGER_SOFTWARE | _0002_SAU_MODE_UART | _0000_SAU_TRANSFER_END;
	SCR10 = _8000_SAU_TRANSMISSION | _0000_SAU_INTSRE_MASK | _0000_SAU_PARITY_NONE | _0080_SAU_LSB + _0010_SAU_STOP_1 + _0007_SAU_LENGTH_8;
	SDR10 = _9A00_UART2_TRANSMIT_DIVISOR;
	NFEN0 |= _10_SAU_RXD2_FILTER_ON;
	SIR11 = _0004_SAU_SIRMN_FECTMN | _0002_SAU_SIRMN_PECTMN | _0001_SAU_SIRMN_OVCTMN;	/* clear error flag */
	SMR11 = _0020_SAU_SMRMN_INITIALVALUE | _0000_SAU_CLOCK_SELECT_CK00 | _0100_SAU_TRIGGER_RXD | _0000_SAU_EDGE_FALL | _0002_SAU_MODE_UART | _0000_SAU_TRANSFER_END;
	SCR11 = _4000_SAU_RECEPTION | _0400_SAU_INTSRE_ENABLE | _0000_SAU_PARITY_NONE | _0080_SAU_LSB | _0010_SAU_STOP_1 | _0007_SAU_LENGTH_8;
	SDR11 = _9A00_UART2_RECEIVE_DIVISOR;	
#endif	
	
	SO0 |= _0001_SAU_CH0_DATA_OUTPUT_1;	
	SOL0 |= _0000_SAU_CHANNEL0_NORMAL;	/* output level normal */
	SOE0 |= _0001_SAU_CH0_OUTPUT_ENABLE;	/* enable UART2 output */
	
#if RJ == 1
	SO0 |= _0004_SAU_CH2_DATA_OUTPUT_1;	
	SOL0 |= _0000_SAU_CHANNEL2_NORMAL;	/* output level normal */
	SOE0 |= _0004_SAU_CH2_OUTPUT_ENABLE;	/* enable UART1 output */	
#endif
#if IrDA == 1	
	SO1 |= _0001_SAU_CH0_DATA_OUTPUT_1;
	SOL1 |= _0000_SAU_CHANNEL0_NORMAL;	/* output level normal */
	SOE1 |= _0001_SAU_CH0_OUTPUT_ENABLE;	/* enable UART2 output */	
#endif		
	/* Set RxD0 pin */
	PM0 |= BIT6;    //set 1 for input and 0 for output
	PIM0 |= BIT6;
	PM3 &= ~BIT0;
	P3 &= ~BIT0;
	/* Set TxD0 pin */
	P0 |= BIT7;
	PM0 &= ~BIT7;
#if RJ ==1

	/* Set RxD1 pin */
	PM0 |= BIT3;
	PIM0 |= BIT3;
	
	/* Set TxD1 pin */
	P0 |= BIT4;
	PM0 &=(u8)(~BIT4);		/* Use P0.4 as TxD */
#endif	
#if IrDA == 1	
	/* Set RxD2 pin */
	PM0 |= BIT0;    //set 1 for input and 0 for output
	PIM0 |= BIT0;
	PM4 &= ~BIT4;
	P4 &= ~BIT4;
	/* Set TxD2 pin */
	P0 |= BIT1;
	POM0  |= BIT1;
	PM0 &= ~BIT1;
	
	IRDAEN = 1U;
	IRCR = _80_IRDA_ENABLE|_00_IRDA_CLK_fCLK0|_00_IRDA_TXD_POLARITY_NORMAL|_04_IRDA_RXD_POLARITY_INVERTED;
	PER0 &= ~(BIT1);
#endif
}


void UART_vStart(void)
{
	STIF0 = 0U;	/* clear INTST2 interrupt flag */
	STMK0 = 0U;	/* enable INTST2 interrupt */
	SRIF0 = 0U;	/* clear INTSR2 interrupt flag */
	SRMK0 = 0U;	/* enable INTSR2 interrupt */
	SREIF0 = 0U;	/* clear INTSRE2 interrupt flag */
	SREMK0 = 0U;	/* enable INTSRE2 interrupt */
	SO0 |= _0001_SAU_CH0_DATA_OUTPUT_1;	/* output level normal */
	SOE0 |= _0001_SAU_CH0_OUTPUT_ENABLE;	/* enable UART2 output */
	SS0 |= _0002_SAU_CH1_START_TRG_ON | _0001_SAU_CH0_START_TRG_ON;	/* enable UART2 receive and transmit */	
#if RJ == 1
	STIF1 = 0U;    /* clear INTST1 interrupt flag */
	STMK1 = 0U;    /* enable INTST1 interrupt */
	SRIF1 = 0U;    /* clear INTSR1 interrupt flag */
	SRMK1 = 0U;    /* enable INTSR1 interrupt */
	SREIF1 = 0U;   /* clear INTSRE1 interrupt flag */
	SREMK1 = 0U;   /* enable INTSRE1 interrupt */	
    SO0 |= _0004_SAU_CH2_DATA_OUTPUT_1;    /* output level normal */
    SOE0 |= _0004_SAU_CH2_OUTPUT_ENABLE;    /* enable UART1 output */
    SS0 |= _0008_SAU_CH3_START_TRG_ON | _0004_SAU_CH2_START_TRG_ON;    /* enable UART1 receive and transmit */
#endif	
#if IrDA == 1	
	STIF2 = 0U;	/* clear INTST2 interrupt flag */
	STMK2 = 0U;	/* enable INTST2 interrupt */
	SRIF2 = 0U;	/* clear INTSR2 interrupt flag */
	SRMK2 = 0U;	/* enable INTSR2 interrupt */
	SREIF2 = 0U;	/* clear INTSRE2 interrupt flag */
	SREMK2 = 0U;	/* enable INTSRE2 interrupt */
	SO1 |= _0001_SAU_CH0_DATA_OUTPUT_1;	/* output level normal */
	SOE1 |= _0001_SAU_CH0_OUTPUT_ENABLE;	/* enable UART2 output */
	SS1 |= _0002_SAU_CH1_START_TRG_ON | _0001_SAU_CH0_START_TRG_ON;	/* enable UART2 receive and transmit */	
	
	IRDAEN = 1U;
	IRCR = _80_IRDA_ENABLE|_00_IRDA_CLK_fCLK0|_00_IRDA_TXD_POLARITY_NORMAL|_04_IRDA_RXD_POLARITY_INVERTED;
	PER0 &= ~(BIT1);
#endif
}

void UART_vStop(void)
{
	SAU0EN = 0;
	ST0 |= _0002_SAU_CH1_STOP_TRG_ON | _0001_SAU_CH0_STOP_TRG_ON;	/* disable UART2 receive and transmit */
	SOE0 &= ~_0001_SAU_CH0_OUTPUT_ENABLE;	/* disable UART2 output */
	STMK0 = 1U;	/* disable INTST2 interrupt */
	STIF0 = 0U;	/* clear INTST2 interrupt flag */
	SRMK0 = 1U;	/* disable INTSR2 interrupt */
	SRIF0 = 0U;	/* clear INTSR2 interrupt flag */
	SREMK0 = 1U;	/* disable INTSRE2 interrupt */
	SREIF0 = 0U;	/* clear INTSRE2 interrupt flag */
#if RJ == 1
    ST0 |= _0008_SAU_CH3_STOP_TRG_ON | _0004_SAU_CH2_STOP_TRG_ON;    /* disable UART1 receive and transmit */
    SOE0 &= ~_0004_SAU_CH2_OUTPUT_ENABLE;    /* disable UART1 output */
    STMK1 = 1U;    /* disable INTST1 interrupt */
    STIF1 = 0U;    /* clear INTST1 interrupt flag */
    SRMK1 = 1U;    /* disable INTSR1 interrupt */
    SRIF1 = 0U;    /* clear INTSR1 interrupt flag */
    SREMK1 = 1U;   /* disable INTSRE1 interrupt */
    SREIF1 = 0U;   /* clear INTSRE1 interrupt flag */
#endif

#if IrDA == 1	
	SAU1EN = 0;
	ST1 |= _0002_SAU_CH1_STOP_TRG_ON | _0001_SAU_CH0_STOP_TRG_ON;	/* disable UART2 receive and transmit */
	SOE1 &= ~_0001_SAU_CH0_OUTPUT_ENABLE;	/* disable UART2 output */
	STMK2 = 1U;	/* disable INTST2 interrupt */
	STIF2 = 0U;	/* clear INTST2 interrupt flag */
	SRMK2 = 1U;	/* disable INTSR2 interrupt */
	SRIF2 = 0U;	/* clear INTSR2 interrupt flag */
	SREMK2 = 1U;	/* disable INTSRE2 interrupt */
	SREIF2= 0U;	/* clear INTSRE2 interrupt flag */
	
	IRDAEN = 0U;
#endif	
}
void fill_zerodarray(unsigned char counter)
{
	u16 address;
	u16 i_darray;

	i_darray = counter/7;
	address = i_darray <<4;
	i_darray=counter%7;

	Eprom_Read(DARRAY_ADD+address);
	FUN_vfill_2byteR(d_array[counter],&opr_data[(2*i_darray)+1]);
	Eprom_Write(DARRAY_ADD+address);
}

static void __near UART_iOpticalInterrupt_Error(void)
{
	uint8_t err_type;
	SREIF0 = 0U;
	err_type=RXD0;
	err_type = (uint8_t)(SSR01 & 0x0007U);
	SIR01 = (uint16_t)err_type;	
	if((err_type & BIT2)!=0)
	{
	  ST0 |= _0002_SAU_CH1_STOP_TRG_ON | _0001_SAU_CH0_STOP_TRG_ON;  
	  SS0 |= _0002_SAU_CH1_START_TRG_ON | _0001_SAU_CH0_START_TRG_ON;	/* enable UART2 receive and transmit */
	}	
}
static void __near UART_iOpticalRx_Interrupt(void)
{
	unsigned char i;
	u8 u8Tempbuff;
	unsigned int u16TempData;
	static union
	{
		u8 a8Temp[2];
		u16 u16Temp;
	}unionTemp;

	Cntr_2Min = 0; 	//For Inactivity Timeout
	if(batt_disp_f==1)
	{
		SW_u32bat_cntr=0;
	}
	SW_u16bkup_cntr=0;
	serial_timeout = 0;
	u8Tempbuff=RXD0;
	if(((u8Tempbuff==0x7e)&&(rj_f==1))&&(optical_f==0))//RJ compatibility
	{
		nrm_flag=0;
		cosem_flag=0;
		rrr_s=0;
		rrr_c=0;
		rrr_c1=0;
		sss_c=0;
		sss_c1=0;
		sss_s=0;
		asso0_flag=0;
		asso1_flag=0;
		asso2_flag=0;
		asso3_flag=0;
		infore_flag=0;

		for(i=0;i<6;i++)
		{
			obis_code[i]=0x00;
		}
        rcv_cnt1=rcv_cnt;
        if(rcv_cnt1!=0)
        {
            for(u16TempData=0;u16TempData<rcv_cnt1;u16TempData++)
                rcv_buf1[u16TempData]=rcv_buf[u16TempData];
        }
        rcv_cnt=0;
        buffer_first_not_fill_f=0;
	}
 	if(((u8Tempbuff==0x7e)||(u8Tempbuff==0x27))&&(optical_f==0))
	{
			optical_f=1;
			optical_cntr=0;
			
	}

	if(optical_f==1)
	{
		rj_f = 0;
		optical_cntr=0;		
		rcv_buf[rcv_cnt]=u8Tempbuff;
		rcv_cnt++;

	if(rcv_buf[0]==0x27 && rcv_cnt==8)       //rtc calib  saperate commond for MSP meters
	{
		for(i=0;i<8;i++)
		{
			data_array[i]=rcv_buf[i];
		}
		rcv_cnt=0;

		analyse_cal_pkt_flag=1;
	}

	if(rcv_buf[0] == 0x7e && rcv_buf[1] == 0x7e)
	{
		rcv_cnt = 1;
	}
	
	unionTemp.a8Temp[1] = (rec[1]&0x07);
	unionTemp.a8Temp[0] = rec[2];
	u16TempData = unionTemp.u16Temp+2;

	if((rcv_buf[0] == 0x7e) && (rcv_cnt>=3) && (rcv_cnt == u16TempData ))
	{
		frm_rcv_flg = 1;
		rcv_cnt = 0;
	}
	else if(rcv_buf[0]!=0x7e && rcv_buf[0]!=0x27)
	{
		rcv_cnt = 0;
	}
	}
	if(rcv_cnt >=(DLMS_MAX_BUFF_SIZE+14))
		rcv_cnt=0;
}


static void __near UART_iOpticalTx_Interrupt(void)
{

	Cntr_2Min=0; 	//For Inactivity Timeout

	if(batt_disp_f==1)
	{
		SW_u32bat_cntr=0;
	}
	SW_u16bkup_cntr=0;
	if (trn_cnt < req_cnt)
	{
		trn_cnt++;            		// Transmission completed counter +1
		if (trn_cnt == req_cnt)
		{
			trn_cnt = 0;      		// Transmission counter clear
			req_cnt = 0;
		}
		else
		{
			TXD0 = trn_buf[trn_cnt];// Set transmission data
		}
	}
}


static void __near UART_iRJInterrupt_Error(void)
{
	uint8_t err_type;
	SREIF1 = 0U;
	err_type=RXD1; 
	err_type = (uint8_t)(SSR03 & 0x0007U);
	SIR03 = (uint16_t)err_type;
	
	if((err_type & BIT2)!=0)
	{
	  ST0 |= _0008_SAU_CH3_STOP_TRG_ON | _0004_SAU_CH2_STOP_TRG_ON;  
	  SS0 |= _0008_SAU_CH3_START_TRG_ON | _0004_SAU_CH2_START_TRG_ON;	/* enable UART2 receive and transmit */
	}
	//UART_vInit();
	//UART_vStart();
	/* clear INTSRE0 interrupt flag */
	
}
static void __near UART_iRJRx_Interrupt(void)
{
	//unsigned char i;
	unsigned int u16TempData;
	static union
	{
		u8 a8Temp[2];
		u16 u16Temp;
	}unionTemp;

	Cntr_2Min = 0; 	//For Inactivity Timeout
	serial_timeout = 0;
	
	if(batt_disp_f==1)
	{
		SW_u32bat_cntr=0;
	}
	SW_u16bkup_cntr=0;
	
	if(optical_f != 1)
	{
        rj_f=1;
		rcv_buf[rcv_cnt]=RXD1;
		rcv_cnt++;

		if(rcv_buf[0] == 0x7e && rcv_buf[1] == 0x7e)
		{
			rcv_cnt = 1;
		}
		
		unionTemp.a8Temp[1] = (rec[1]&0x07);
		unionTemp.a8Temp[0] = rec[2];
		u16TempData = unionTemp.u16Temp+2;

		if((rcv_buf[0] == 0x7e) && (rcv_cnt>=3) && (rcv_cnt == u16TempData ))
		{
			frm_rcv_flg = 1;
			rcv_cnt = 0;
		}
		else if(rcv_buf[0]!=0x7e)
		{
			rcv_cnt = 0;
		}
            if(rcv_cnt >=(DLMS_MAX_BUFF_SIZE+14))
				rcv_cnt=0;
	}
	else
	{
		rj_disc_f = 1;
		rcv_buf1[rcv_cnt1]=RXD1;
		rcv_cnt1++;
		
		if(rcv_buf1[0] == 0x7e && rcv_buf1[1] == 0x7e)
		{
			rcv_cnt1 = 1;
		}
		
		if(rcv_buf1[0] == 0x7e && rcv_cnt1>=3 && rcv_cnt1 == rcv_buf1[2]+2)
		{
			rj_disc_f = 1;
			rcv_cnt1 = 0;
		}
		else if(rcv_buf1[0]!=0x7e)
		{
			rcv_cnt1 = 0;
		}
		
		if(rcv_cnt1 >=150)
		{
			rcv_cnt1=0;
		}
	}
}


static void __near UART_iRJTx_Interrupt(void)
{

	Cntr_2Min=0; 	//For Inactivity Timeout

	if(batt_disp_f==1)
	{
		SW_u32bat_cntr=0;
	}
	SW_u16bkup_cntr=0;
	if(rj_disc_f2==0)
	{
		if (trn_cnt < req_cnt)
		{
			trn_cnt++;            		// Transmission completed counter +1
			if (trn_cnt == req_cnt)
			{
				trn_cnt = 0;      		// Transmission counter clear
				req_cnt = 0;
			}
			else
			{
				TXD1 = trn_buf[trn_cnt];// Set transmission data
			}
		}
	}
	else
	{
           if (trn_cnt1 < req_cnt1)
            {
                trn_cnt1++;            /* Transmission completed counter +1 */
                if (trn_cnt1 >= req_cnt1)
                {
                    trn_cnt1 = 0;       /* Transmission counter clear */
                    req_cnt1 = 0;
                    rj_disc_f2=0;
                }
                else
                {
                    TXD1 = rj_dm_buf[trn_cnt1];/* Set transmission data */
                }
            }
	}
}


/*
static void __near UART_IRDA_Interrupt_Error(void)
{
	uint8_t err_type;
	err_type = (uint8_t)(SSR11 & 0x0007U);
	SIR11 = (uint16_t)err_type;
	//UART_vInit();
	//UART_vStart();
}
*/

/*
static void __near UART_IRDA_Rx_Interrupt(void)
{
	unsigned char i;
	unsigned int u16TempData;
	static union
	{
		u8 a8Temp[2];
		u16 u16Temp;
	}unionTemp;

	Cntr_2Min = 0; 	//For Inactivity Timeout
	serial_timeout = 0;
	if(batt_disp_f==1)
	{
		SW_u32bat_cntr=0;
	}
	SW_u16bkup_cntr=0;

	if(GPR_u8TCPIP_F==1)//RJ compatibility
	{
		nrm_flag=0;
		cosem_flag=0;
		rrr_s=0;
		rrr_c=0;
		rrr_c1=0;
		sss_c=0;
		sss_c1=0;
		sss_s=0;
		asso0_flag=0;
		asso1_flag=0;
		asso2_flag=0;
		asso3_flag=0;
		infore_flag=0;

		for(i=0;i<6;i++)
		{
			obis_code[i]=0x00;
		}
		rcv_cnt=0;
	}
	rcv_buf[rcv_cnt]=RXD2;
	if(rcv_buf[rcv_cnt]==0x7e)
	{
		IrDA_f=1;
	}

	rcv_cnt++;

	if(rcv_buf[0]==0x27 && rcv_cnt==8)       //rtc calib  saperate commond for MSP meters
	{
		for(i=0;i<8;i++)
		{
			data_array[i]=rcv_buf[i];
		}
		rcv_cnt=0;

		analyse_cal_pkt_flag=1;
	}

	if(rcv_buf[0] == 0x7e && rcv_buf[1] == 0x7e)
	rcv_cnt = 1;
	unionTemp.a8Temp[1] = (rec[1]&0x07);
	unionTemp.a8Temp[0] = rec[2];
	u16TempData = unionTemp.u16Temp+2;

	if((rcv_buf[0] == 0x7e) && (rcv_cnt>=3) && (rcv_cnt == u16TempData ))
	{
		frm_rcv_flg = 1;
		rcv_cnt = 0;
	}
	else if(rcv_buf[0]!=0x7e && rcv_buf[0]!=0x27)
	{
		rcv_cnt = 0;
	}
}
*/

/*
static void __near UART_IRDA_Tx_Interrupt(void)
{
	Cntr_2Min=0; 	//For Inactivity Timeout
	
	if(batt_disp_f==1)
	{
		SW_u32bat_cntr=0;
	}
	SW_u16bkup_cntr=0;
	if (trn_cnt < req_cnt)
	{
		trn_cnt++;            			// Transmission completed counter +1
		if (trn_cnt == req_cnt)
		{
			trn_cnt = 0;       			// Transmission counter clear
			req_cnt = 0;
		}
		else
		{
			TXD2 = trn_buf[trn_cnt];	// Set transmission data
		}
	}
}
*/

void Clear_Buff(void)
{
	u16 i;

	for(i=0;i<(DLMS_MAX_BUFF_SIZE+14);i++)
	rcv_buf[i] = 0x00;
}



void send_type_multi(void)
{

	//	if(GPR_u8TCPIP_F==1)
	//	{
	//		if((multi_filling_f==1)||(u8Lock_multi_transfer==1))
	//		{
	//			u8Lock_multi_transfer=1;
	//			if(multi_filling_f==0)
	//				u8Lock_multi_transfer=0;
	//			info[0]=0x00;
	//			info[1]=0x01;
	//			info[2]=0x00;
	//			info[3]=0x01;
	//			info[4]=0x00;
	//			info[5]=client_add;
	//			info[6]=(k-8)/256;
	//			info[7]=(u8)(k-8);
	//			memcpy(gpr_TXbuffr,info,k);
	////			gpr_TXbuffr[k]=0x1a;
	////			gpr_vSnd2mdm(k);
	//
	////			if(GPR_u8Send_Packet==0)
	////				GPR_u8Send_Packet=1;
	////			else
	////				GPR_u8Send_Packet_pending=1;
	//		}
	//		else
	//		{
	//			info[0]=0x00;
	//			info[1]=0x01;
	//			info[2]=0x00;
	//			info[3]=0x01;
	//			info[4]=0x00;
	//			info[5]=client_add;
	//			info[6]=(k-8)/256;
	//			info[7]=(u8)(k-8);
	//			memcpy(gpr_TXbuffr,info,k);
	//			gpr_vSnd2mdm(k);
	////			if(GPR_u8Send_Packet==0)
	////				GPR_u8Send_Packet=1;
	////			else
	////				GPR_u8Send_Packet_pending=1;
	//		}
	//	}
	//	else
	{
		if(long_data==0)
		{
			seg_flagsd=1;
			info_sended=0;
		}
		if(info_send>max_info_tra)
		{
			seg_type=0xA8;
			info_total=max_info_tra;
			info_send-=max_info_tra;
			if(send_type_multi_f==1)
			{
				frame_type=0;
			}

			send_type_multi_f=1;
		}
		else
		{
			seg_type=0xA0;
			seg_flagsd=0;
			info_total=info_send;
			send_type_multi_f=0;
		}
		if(frame_type!=0x73)
		frame_type=((rrr_s<<5)|(0x10)|(sss_s<<1));
		send_type();
		long_data++;
		if(seg_type==0xA0)
		long_data=0;
	}
}

void send_type(void)
{//send_type
	unsigned int i,j,infolen;

	infolen = info_total;
	sd[0]=0x7E;//stating flag
	sd[1]=seg_type;//frame type
	sd[3]=client_add ;//client address
	if(one_byte_add_f!=1)
	{
		//for(i=0;i<4;i++)
		//sd[i+4]=server_add[i];
		memcpy(&sd[4],server_add,4);
		sd[8]=(u8)frame_type;//control field format
	}
	else
	{
		sd[4]=server_add[3];
		sd[5]=(u8)frame_type;
	}
	if(infolen!=0)
	{//ifinfo
		if(one_byte_add_f!=1)
		j=11;
		else
		j=8;
		info_sended_old=info_sended;
		for(i=0;i<infolen;j++,i++,info_sended++)
		sd[j]=info[info_sended];

		j--;

		if(one_byte_add_f!=1)
		{
			infolen=0x0a+info_total+2;
			sd[1]=(u8)(seg_type|((infolen>>8)&0x0007));
			sd[2]=(u8)(infolen%256);//0x0a+info_len+2;//(len of HdLC init frame format+info length(1)+FCS(2)
			fcs(sd+1,8,1);
			sd[9]=dlms_x;
			sd[10]=dlms_y;
		}
		else
		{
			infolen=0x07+info_total+2;
			sd[1]=(u8)(seg_type|((infolen>>8)&0x0007));
			sd[2]=(u8)(infolen%256);//0x07+info_len+2;//(len of HdLC init frame format+info length(1)+FCS(2)
			fcs(sd+1,5,1);
			sd[6]=dlms_x;
			sd[7]=dlms_y;
		}

		fcs(sd+1,j,1);
		j++;
		sd[j]=dlms_x;
		j++;
		sd[j]=dlms_y;
		j++;
	}//ifinfo
	else
	{//elseinfo
		if(one_byte_add_f!=1)
		{
			j=11;
			sd[2]=0x0a;//(len of HdLC init frame format+info length(1)+FCS(2)
			fcs(sd+1,8,1);
			sd[9]=dlms_x;
			sd[10]=dlms_y;
		}
		else
		{
			j=8;
			sd[2]=0x07;//(len of HdLC init frame format+info length(1)+FCS(2)
			fcs(sd+1,5,1);
			sd[6]=dlms_x;
			sd[7]=dlms_y;
		}
	}//elseinfo
	sd[j]=0x7E;

	for(i=0;i<135;i++)
	rec[i] = 0x00;

	req_cnt=j+1;
	rcv_cnt=0x00;
	trn_cnt=0;
	if(optical_f==1)
	{
		TXD0 = trn_buf[trn_cnt];// Set transmission data
	}
	else if(IrDA_f==1)
	{
		TXD2 = trn_buf[trn_cnt];
	}
	else if(rj_f == 1)
	{
		TXD1 = trn_buf[trn_cnt];
	}
	
	if(((sd[11]==0xE6)&&(sd[12]==0xE7)&&(sd[13]==0x00)&&((sd[1]&0xf8)!=0xA8))||((sd[8]==0xE6)&&(sd[9]==0xE7)&&(sd[10]==0x00)&&((sd[1]&0xf8)!=0xA8)&&(one_byte_add_f==1)))
	infose_flag=1;
	else
	infose_flag=0;

	if(one_byte_add_f!=1)
	{
		if(sd[8]==0x1f)
		{

			optical_f=0;
			IrDA_f=0;
			rj_disc_f2=0;
			rj_disc_f=0;
			rj_disc_cnt=0;
		}
	}
	else
	{
		if(sd[5]==0x1f)
		{
			optical_f=0;
			IrDA_f=0;
			rj_disc_f2=0;
			rj_disc_f=0;
			rj_disc_cnt=0;
		}
	}
}//send_type



void fill_A0(unsigned char len)/********/
{
	info_total=0;
	seg_type=0xA0;
	frame_type=len;
	send_type();
}

unsigned char hdlc1_s4(void)
{//hdlc1_s4
	unsigned int logical_add=0,physical_add=0;//,packet_len;//,client_address=0
	unsigned char old_cont_field;
	packet_len=(rec[1]&0x07)*256+rec[2];
	Format_type=rec[1]&0xF8;
	if(fcs(rec+1,packet_len,2)==0)
	{
		return 0;
	}
	old_cont_field = cont_field;
	if(one_byte_add_f!=1)
	{
		logical_add = rec[3]>>1;
		logical_add = logical_add<<7;
		logical_add |=(rec[4]>>1);
		physical_add = rec[5]>>1;
		physical_add = physical_add<<7;
		physical_add |= (rec[6]>>1);
		client_add=rec[7];
		cont_field=rec[8];
	}
	else
	{
		if(rec[3]==0x03)
		{
			logical_add=1;
			physical_add=0x0100;
		}
		else
		{
			logical_add=0;
			physical_add=0x0000;
		}
		client_add=rec[4];
		cont_field=rec[5];
	}


	if((rec[0]!=0x7E)||
			(rec[packet_len+1]!=0x7E)||
			((Format_type!=0xA0)&&(Format_type!=0xA8))||
			((packet_len<10)&&(one_byte_add_f==0))||
			((packet_len<7)&&(one_byte_add_f==1)))
	{//if2
		return 0;
	}//if2

	if((client_add!=0x21)&&(client_add!=0x41)&&(client_add!=0x61)&&(client_add!=0x81))
	{
		return 0;
	}
	if((logical_add!=0x0001)||(physical_add!=0x0100))
	{
		return 0;
	}
	else if((((packet_len==0x0A)&&(nrm_flag==0)) || ((packet_len==0x0A)&&(nrm_flag==1)&& (asso0_flag != 1 ||asso1_flag!= 1||asso2_flag!= 1)))|| //unknown cmd identifier 22/02/2007
			(((packet_len==0x07)&&(nrm_flag==0)) || ((packet_len==0x07)&&(nrm_flag==1)&& (asso0_flag != 1 ||asso1_flag!= 1||asso2_flag!= 1))))
	{//elsecli
		if((cont_field!=0x93)&&(cont_field!=0x53))
		{//ifclient
			if(((cont_field&0x01)==0x01) && ((cont_field&0x0f)!=0x01))//I frame in NDM mode 22/01/2007
			{//ifsta
				if(one_byte_add_f!=1)
				{
					//for(i=0;i<4;i++)
					//server_add[i]=rec[i+3];
					memcpy(server_add,&rec[3],4);
					client_add=rec[7];
					length=0x0A;
				}
				else
				{
					server_add[3]=rec[3];// server_lowerlow=rec[3];
					client_add=rec[4];
					length=0x07;
				}
				fill_A0(0x97);
				return 0;
			}//ifsta
		}//ifclient
	}//elsecli

    if((cont_field!=0x93)&&(cont_field!=0x53)
	   &&(((cont_field == old_cont_field)&&(nrm_flag==1))
		  ||((cont_field == ((old_cont_field&0xf0)|0x01))&&(nrm_flag==1))))
	{
		info_sended=info_sended_old;
		send_type();
		return (0);
	}	

	if((cont_field!=0x93)&&(cont_field!=0x53)&&(cont_field!=0x13))
	rrr_c=cont_field>>5;

	if(cont_field%2==0)
	sss_c=(cont_field>>1)&(0x07);

	p_fbit=(cont_field>>4)&(0x01);

	if(p_fbit!=1)
	{//ifp
		fill_A0(0x97);
		return 0;
	}//ifp

	if(nrm_flag==1)
	{//ifnrm
		if((rec[11]==0xE6)&&(rec[12]==0xE6)&&(rec[13]==0x00)&&(cont_field%2!=0))
		{//ifre
			fill_A0(0x97);
			return 0;
		}//ifre

		else if((infore_flag==1)&&(cont_field!=0x53)&&((cont_field&0x0F)!=0x01)&&(cont_field!=0x93))
		{//ifinf
			if(sss_c1!=7)
			{//ifsss
				if(((sss_c-sss_c1)!=1)&&((sd[1]&0xF8)==0xA0))
				{//ifsss
					fill_A0((u8)(rrr_s<<5)|(0x11));
					return 0;
				}//ifsss
				else if(((sd[1]&0xF8)==0xA8)&&((cont_field&0x0F)!=0x01))
				{//ifa8
					fill_A0((u8)(rrr_s<<5)|(0x11));
					return 0;
				}//ifa8
			}//ifsss
			else
			{//elsesss
				if(((sss_c1-sss_c)!=7)&&((sd[1]&0xF8)==0xA0))
				{//ifsss
					fill_A0((u8)(rrr_s<<5)|(0x11));
					return 0;
				}//ifsss
				else if(((sd[1]&0xF8)==0xA8)&&((cont_field&0x0F)!=0x01))
				{//ifa8
					fill_A0((u8)(rrr_s<<5)|(0x11));
					return 0;
				}//ifa8
			}//elsesss
		}//ifinf

		else if(((sd[1]&0xF8)==0xA0)&&(cont_field!=0x53)&&((cont_field&0x0F)!=0x01)&&(cont_field!=0x93))
		{//elseinfo
			if((sss_c-sss_c1)!=0)
			{//ifsss
				fill_A0((u8)(rrr_s<<5)|(0x11));
				return 0;
			}//ifsss
		}//elseinfo

        else if((infose_flag==1)&&(cont_field!=0x53)&&(cont_field!=0x93)) //ifinfse
		{
			if(rrr_c1!=7) //ifrrr
			{
				if((rrr_c-rrr_c1)!=1) //ifrr
				{
					fill_A0(0x97);
					return(0);
				} //ifrr
				else if((cont_field&0x0F)==0x01)	// In case of info command not received and RR received with incremented sequence number
				{
					fill_A0((rrr_s<<5)|(0x11));
					return(0);
				}
			} //ifrrr
			else //elserrr
			{
				if((rrr_c1-rrr_c)!=7) //ifrr
				{
					fill_A0(0x97);
					return(0);
				} //ifrr
				else if((cont_field&0x0F)==0x01)	// In case of info command not received and RR received with incremented sequence number
				{
					fill_A0((rrr_s<<5)|(0x11));
					return(0);
				}
			} //elserrr

		} //ifinfse

		else if(((sd[1]&0xF8)==0xA0)&&((cont_field&0x0F)==0x01))
		{//ifinfse
			if((rrr_c-rrr_c1)!=0)
			{//ifrr
				fill_A0(0x97);
				return 0;
			}//ifrr
		}//ifinfse
	}//ifnrm
	return 1;
}//End hdlc1_s4

void recv_frm(void)
{
	if((frm_rcv_flg ==1)&&(req_cnt==0))
	{
		frm_rcv_flg = 0;
		if((rcv_buf[3] == 0x03)&&((rcv_buf[4] & 0x01)==1))
		{
			one_byte_add_f=1;
		}
		else
		{
			one_byte_add_f=0;
		}
		rcv_cnt=0;
		dlms_rece_flag=hdlc1_s4();
		if(dlms_rece_flag==0)
		{
			Clear_Buff();
		}
	}
	else
	{
		dlms_rece_flag=0;
	}

	if(dlms_rece_flag==1)
	{//ifrec_fl
		if(batt_disp_f == 1)
		SW_u32bat_cntr=0;
		if(nrm_flag==1)
		{
			control_field();
		}
		req_type();
	}//ifrec_fl

	if(rj_disc_f==1 && rj_disc_cnt>50)
	{//simultanious test start
		rj_disc_f=0; 
		rj_disc_cnt=0;
		rj_dm_buf[0] = 0x7e;
		rj_dm_buf[1] = 0xa0;
		if(rcv_buf1[3]==0x03)
		{
			rj_dm_buf[2] = 0x07;
			if(rcv_buf1[4]==0x21 || rcv_buf1[4]==0x41 || rcv_buf1[4]==0x61)
			{
				rj_dm_buf[3] = rcv_buf1[4];
			}
			else
			{
				if(asso0_flag==1)
				rj_dm_buf[3] = 0x21;
				else if(asso1_flag==1)
				rj_dm_buf[3] = 0x41;
				else if(asso2_flag==1)
				rj_dm_buf[3] = 0x61;
			}

			rj_dm_buf[4] = 0x03;//rcv_buf1[3];
			rj_dm_buf[5] = 0x1F;
			fcs(rj_dm_buf+1,5,1);
			rj_dm_buf[6]=dlms_x;
			rj_dm_buf[7]=dlms_y;
			rj_dm_buf[8] = 0x7e;
			req_cnt1=9;
		}
		else if(rcv_buf1[3]==0x00)
		{
			rj_dm_buf[2] = 0x0a;
			if(rcv_buf1[7]==0x21 || rcv_buf1[7]==0x41 || rcv_buf1[7]==0x61)
			rj_dm_buf[3] = rcv_buf1[7];
			else
			{
				if(asso0_flag==1)
				rj_dm_buf[3] = 0x21;
				else if(asso1_flag==1)
				rj_dm_buf[3] = 0x41;
				else if(asso2_flag==1)
				rj_dm_buf[3] = 0x61;
			}
			
			rj_dm_buf[4] = 0x00;//rcv_buf1[3];
			rj_dm_buf[5] = 2;//rcv_buf1[4];
			rj_dm_buf[6] = 4;//rcv_buf1[5];
			rj_dm_buf[7] = 1;//rcv_buf1[6];
			rj_dm_buf[8] = 0x1F;
				fcs(rj_dm_buf+1,8,1);
			rj_dm_buf[9]=dlms_x;
			rj_dm_buf[10]=dlms_y;
			rj_dm_buf[11] = 0x7e;
			req_cnt1=12;
			//    		 rcv_cnt1=0;
		}
		trn_cnt1=0;
		rj_disc_f2=1;
		TXD1 = rj_dm_buf[0];/* Set transmission data */
	}//simultanious test end
}//End Function recv_frm

void control_field(void)
{//start of fun

	if((cont_field!=0x93)&&(cont_field!=0x53)&&(cont_field!=0x13))  //If SNRM,DISC,UI Command
	rrr_c=cont_field>>5;

	if(cont_field%2==0)
	sss_c=(cont_field>>1)&(0x07);

	if(Format_type==0xA0)
	{//ifa0
		if(((rec[11]==0xE6)&&(rec[12]==0xE6)&&(rec[13]==0x00))||(seg_flag==1)||((rec[8]==0xE6)&&(rec[9]==0xE6)&&(rec[10]==0x00)&&one_byte_add_f))
		{//ifinfo
			if(rrr_s!=7)
			rrr_s=rrr_s+1;
			else
			rrr_s=0;
			infore_flag=1;
		}//ifinfo
		else if ((sd[1]&0XF8)!=0xA8)
		infore_flag=0;
	}//ifa0

	else if(Format_type==0xA8)
	{//ifa8
		if(rrr_s!=7)
		rrr_s=rrr_s+1;
		else
		rrr_s=0;
		infore_flag=1;
	}//ifa8

	if(rrr_c1!=7)
	{//ifrrr
		if(((rrr_c-rrr_c1)==1)&&(cont_field!=0x93)&&(cont_field!=0x53))////////increased
		{//ifsss
			if(sss_s!=7)
			sss_s=sss_s+1;
			else
			sss_s=0;
		}//ifsss
	}//ifrrr
	else
	{//elserrr
		if(((rrr_c1-rrr_c)==7)&&(cont_field!=0x93)&&(cont_field!=0x53))
		{//ifsss
			if(sss_s!=7)
			sss_s=sss_s+1;
			else
			sss_s=0;
		}//ifsss
	}//elserrr
	rrr_c1=rrr_c;
	sss_c1=sss_c;
}//end or fun

void update_k(void)
{
	k++;
	k=info[k]+k;k++;
}

void conf_ser(unsigned char len)
{
	asserr_flag=1;
	conf_ser_flag=1;
	conf_serror_flag=0x01;
	conf_err_flag=0x06;
	conf_type_flag=len;
}

void conf_err(unsigned char len)
{
	conf_err_flag=0x01;
	conf_type_flag=len;
}


void info_l(void)
{
	k=k+4;
}
void info_l5(void)
{
	k=k+5;
}

void info_l6(void)
{
	k=k+6;
}
void info_l7(void)
{
	k=k+7;
}

void info_l8(void)
{
	k=k+8;
}

void seg_flags(void)
{
	seg_flagsd=1;
	long_data=0;
}




void octet_s(unsigned char len,unsigned char flag)
{
	if(flag == 1)
	info[k++]=0;
	info[k++]=0x09;
	info[k++]=len;
}

void obiscode(unsigned char a,unsigned char b,unsigned char c,unsigned char d,unsigned char e,unsigned char f)
{//obiscode
	octet_s(0x06,0);
	info[k]=a;//val-A
	info[k+1]=b;//val-B
	info[k+2]=c;//val-C
	info[k+3]=d;//val-D
	info[k+4]=e;//val-E
	info[k+5]=f;//val-E
	info_l6();
}//obiscode




void load_date(unsigned char dd,unsigned char mm,unsigned char yy,unsigned char dofw)
{
	unsigned int uint_temp;
	
	if((yy == 0xFF)|| (yy == 0x00))
	{
		info[k] = 0xFF;//year high byte
		info[k+1] = 0xFF;//year high byte
	}		
	else
	{
		uint_temp =bcd_to_hex(yy)+ 0x07D0; //Hex 2000 ((yy/16)*10)+(yy%16)
		info[k] = (unsigned char)(uint_temp / 256);//year high byte
		info[k+1] = (unsigned char)(uint_temp % 256);//year high byte
	}
	if((mm == 0xFF)||(mm == 0x00))
	info[k+2] = 0xFF;//month
	else
	info[k+2]=bcd_to_hex(mm);//(mm/16)*10+(mm%16);//month
	if((dd == 0xFF)||(dd == 0x00))
	info[k+3] = 0xFF;//date
	else
	info[k+3]=bcd_to_hex(dd);//(dd/16)*10+(dd%16);//date
	if(dofw>7)
	info[k+4]=0xff;//day of week
	else
	info[k+4]=dofw;//day of week
}
//void load_date(unsigned char dd,unsigned char mm,unsigned char yy)
//{
//	unsigned int uint_temp;
//
//	if(yy == 0xFF || yy == 0x00)
//	{
//		info[k] = 0xFF;//year high byte
//		info[k+1] = 0xFF;//year high byte
//	}
//	else
//	{
//		uint_temp =bcd_to_hex(yy)+ 0x07D0; //Hex 2000 ((yy/16)*10)+(yy%16)
//		info[k] = (unsigned char)(uint_temp / 256);//year high byte
//		info[k+1] = (unsigned char)(uint_temp % 256);//year lower byte
//	}
//	if(mm == 0xFF || mm == 0x00)
//	{
//		info[k+2] = 0xFF;//month
//	}
//	else
//	{
//		info[k+2]=bcd_to_hex(mm);//(mm/16)*10+(mm%16);//month
//	}
//	if(dd == 0xFF || dd == 0x00)
//	{
//		info[k+3] = 0xFF;//date
//	}
//	else
//	{
//		info[k+3]=bcd_to_hex(dd);//(dd/16)*10+(dd%16);//date
//	}
//	info[k+4]=0xFF;//day of week
//}

void load_time(unsigned char hh,unsigned char min,unsigned char sec)
{
	if(hh == 0xFF)
	info[k] = 0xFF;//hour
	else
	info[k]=bcd_to_hex(hh);//(hh/16)*10+(hh%16);//hour
	if(min == 0xFF)
	info[k+1] = 0xFF;//minute
	else
	info[k+1]=bcd_to_hex(min);//(min/16)*10+(min%16);//minute
	if(sec == 0xFF)
	info[k+2] = 0xFF;//month
	else
	info[k+2]=bcd_to_hex(sec);//(sec/16)*10+(sec%16);//seconds
	info[k+3]=0xff;//hunderdths of seconds(not specified)
	info_l();
}
void date_time(unsigned char dd,unsigned char mm,unsigned char yy,unsigned char hh,unsigned char min,unsigned char sec,unsigned char flg)
{//date_time
	//	unsigned int uint_temp = 0x00;
	unsigned char dow;
	if(flg == 0x01)//for individual data without day of week
	{
		info[k++]=0x00;
		dow=0xff;
	}
	else if(flg == 0x02)//for combine data with day of week
	{
		if(dt.week==0)
		dow=7;
		else
		dow=dt.week;
	}
	else if(flg == 0x03)//for individual data with day of week
	{
		info[k++]=0x00;
		if(dt.week==0)
		dow=7;
		else
		dow=dt.week;
	} 
	else//for combine data without day of week
	{
		dow=0xff;
	}
	octet_s(0x0C,0);
	load_date(dd,mm,yy,dow);
	info_l5();

	load_time(hh,min,sec);
	info[k]=0x01;//deviation high byte(not specified)
	info[k+1]=0x4a;//deviation low byte(not specified)
	if((flg == 0x02)||(flg == 0x03))
	{
		info[k+2]=rtc_status_byte;
	}else
	{
		info[k+2]=0x00;//clock_status(ok)
	}
	k=k+3;
}//date_time

//void date_time(unsigned char dd,unsigned char mm,unsigned char yy,unsigned char hh,unsigned char min,unsigned char sec,unsigned char flg)
//{//date_time
//	if(flg == 0x01)
//		info[k++]=0x00;
//	octet_s(0x0C,0);
//	load_date(dd,mm,yy);
//	info_l5();
//
//	load_time(hh,min,sec);
//	info[k]=0x01;//deviation high byte(not specified)
//	info[k+1]=0x4A;//deviation low byte(not specified)
//	info[k+2]=rtc_status_byte;//clock_status(ok)
//	k=k+3;
//}//date_time

void structure(unsigned char len)/********/
{
	info[k++]=0x02;
	info[k++]=len;
}

void array(unsigned char len,unsigned char flag)
{
	if(flag==1)
	info[k++]=0x00;
	info[k++]=0x01;
	info[k++]=len;
}

void long_unsign(void)				//note: used only when first byte is 0x00
{
	info[k++]=0x12;
	info[k++]=0x00;
}

void Tarrif_script(void)
{
	unsigned char i;

	array(Tarriff_slots,1);
	for(i=1;i<=Tarriff_slots;i++)
	{
		structure(0x02);//7 bytes one  structure
		long_unsign();
		info[k++]=i;//script_id[1]
		info[k++]=0x01;//array
		info[k++]=0x00;//array element 1
	}
	//	info_len=k;
}

void Push_script(void)
{
	array(0x01,1);
	structure(0x02);//7 bytes one  structure
	long_unsign();
	info[k++]=0x07;//script_id[1]
	info[k++]=0x01;//array
	info[k++]=0x00;//array element 1
	//	info_len=k;
}

void Image_script(void)
{
	array(0x01,1);
	structure(0x02);//7 bytes one  structure
	long_unsign();
	info[k++]=0x08;//script_id[1]
	info[k++]=0x01;//array
	info[k++]=0x00;//array element 1
	//	info_len=k;
}

void Disconnect_script(void)
{
	array(0x01,1);
	structure(0x02);//7 bytes one  structure
	long_unsign();
	info[k++]=0x09;//script_id[1]
	info[k++]=0x01;//array
	info[k++]=0x00;//array element 1
	//	info_len=k;
}

void Start_Info2(void)
{

	//for(i_s=0;i_s<13;i_s++)
	//{
	//	info[k++]=Start_Info_CONT1[i_s];
	//}

	memcpy(&info[k],Start_Info_CONT1,13);
	k+=13;
	multi_resp=1;
}

void Start_Info(void)	//only for use in buffer
{

	//for(i_s2=0;i_s2<8;i_s2++)
	//{
	//info[i_s2]=Start_Info_CONT[i_s2];
	//}

	memcpy(info,Start_Info_CONT,8);
	multi_resp=1;
}
void send_data(u8 u8temp)
{
	if(GPR_u8TCPIP_F==1)
	{
		info[11]=u8temp;
		info[14]=(u8)(block_no/256);
		info[15]=(u8)block_no;
		info[18]=(u8)((k-20)/256);
		info[19]=(u8)(k-20);
	}
	else
	{
		info[6]=u8temp;
		info[9]=(u8)(block_no/256);
		info[10]=(u8)block_no;
		info[13]=(u8)((k-15)/256);
		info[14]=(u8)(k-15);
	}
	block_no++;


	frame_type=0;//((rrr_s<<5)|(0x10)|(sss_s<<1));
	info_send=k;
	info_sended=0;
	send_type_multi();


}
void day_profile(unsigned char passive)
{
	u16 buffer_filled_u16;
	unsigned char i_d,addpg,u8temp;
	u8 u8dayid1;
	u8 u8dayid2;
	u8 u8noofdayid;
	u8 tempdayid;
	unsigned char temp1[9],temp2[9],no_bytes1[2];
	if(GPR_u8TCPIP_F==1)
	{
		k=5;
		Start_Info2();
		k=20;
	}
	else
	{
		k=0;
		Start_Info2();
		k=15;
	}
	
	
	buffer_filled_u16=k;
	
	if(passive == 0)
	addpg=0x00;
	else if(passive == 1)
	addpg=0x60;

	Eprom_Read(TOU_DAY_ACTIVE_ADD+addpg);//season 1
	//for(i_d=0;i_d<9;i_d++)
	//temp1[i_d]=opr_data[i_d];
	memcpy(temp1,opr_data,9);
	u8noofdayid=opr_data[13];
	u8dayid1=opr_data[14];
	Eprom_Read(TOU_DAY_ACTIVE_ADD+addpg+0x30);//season 2

	//for(i_d=0;i_d<9;i_d++)
	//temp2[i_d]=opr_data[i_d];
	memcpy(temp2,opr_data,9);
	u8dayid2=opr_data[14];

	if(buffer_first_not_fill_f == 0)
	{
		buffer_first_not_fill_f=1;
		multi_filling_f=1;
		element_filled=0;
		block_no=1;
		//info[k-11]=0x01;
		//info[k-7]=0x01;//Data Block 1

		no_bytes=2+(temp1[0]*19)+(temp2[0]*19);
		//info[k-4]=(no_bytes/256);
		//info[k-3]=(no_bytes%256);
		//info[k-2]=0x01;
		//info[k-1]=0x02;
		array(u8noofdayid,0);

	}

	no_bytes1[0]=temp1[0]*19;
	no_bytes1[1]=temp2[0]*19;

	for(;element_filled<u8noofdayid;element_filled++)
	{
		
		buffer_filled_u16+=6+no_bytes1[element_filled];
		
		structure(2);
		info[k++]= 0x11;
		if(element_filled==0)
		info[k++]= u8dayid1;
		else
		info[k++]= u8dayid2;
		//		info[k++]= element_filled+1;//day_id
		if(element_filled == 0)
		tempdayid = temp1[0];
		else
		tempdayid = temp2[0];
		
		array(tempdayid,0);
		Eprom_Read(TOU_DAY_ACTIVE_ADD+addpg+0x10+(element_filled *0x0030));	//6
		for(i_d=0;(i_d<tempdayid && i_d < 7); i_d++)
		{
			structure(3);
			octet_s(4,0);
			load_time(*(opr_data+2*i_d), *(opr_data+2*i_d+1),0);
			//for(i_d1=0;i_d1<8;i_d1++)
			//info[k++]=obis_fill[i_d1];
			memcpy(&info[k],obis_fill,8);
			k+=8;
			long_unsign();
			if(element_filled == 0)
			info[k++]=temp1[i_d+1];
			else
			info[k++]=temp2[i_d+1];
		}
		
		if((temp1[0] > 7 && element_filled == 0) || (temp2[0] > 7 && element_filled == 1))
		{
			
			Eprom_Read(TOU_DAY_ACTIVE_ADD+addpg+0x20+(element_filled * 0x0030));
			structure(3);
			octet_s(4,0);
			load_time(*(opr_data), *(opr_data+1),0);
			//for(i_d1=0;i_d1<8;i_d1++)
			//info[k++]=obis_fill[i_d1];
			memcpy(&info[k],obis_fill,8);
			k+=8;
			long_unsign();
			if(element_filled == 0)
			info[k++]=temp1[8];
			else
			info[k++]=temp2[8];
		}
		
		if(DLMS_MAX_BUFF_SIZE<(buffer_filled_u16+6+no_bytes1[element_filled+1]))
		{
			break;
		}
		
		
		
	}
	element_filled++;
	if(element_filled >= 2)	
	{
		buffer_first_not_fill_f=0;
		multi_filling_f=0;
		u8temp=1;
	}
	send_data(u8temp);
}

void access_rights(unsigned int cnt_att,unsigned char att1,unsigned char att2,unsigned char att3,unsigned char att4,unsigned char att5,unsigned char att6,unsigned char att7,unsigned char att8,unsigned char att9,unsigned char att10,unsigned char att11,unsigned char sec_ass,unsigned char method,unsigned char m1,unsigned char m2,unsigned char m3,unsigned char m4)
{//access_rights
	unsigned int at;
	unsigned char i_f;
	info[k++]=(u8)cnt_att;//length of array which shows attribute

	for(at=0;at<cnt_att;at++)
	{//foracce
		structure(0x03);
		info[k]=0x0F;//integer
		info[k+1]=(u8)(at+1);//attribute_id-->1
		info[k+2]=0x16;//enum
		switch(at)
		{//swat
		case 0:
			info[k+3]=att1;
			break;
		case 1:
			info[k+3]=att2;
			break;
		case 2:
			info[k+3]=att3;
			break;
		case 3:
			info[k+3]=att4;
			break;
		case 4:
			info[k+3]=att5;
			break;
		case 5:
			info[k+3]=att6;
			break;
		case 6:
			info[k+3]=att7;
			break;
		case 7:
			info[k+3]=att8;
			break;
		case 8:
			info[k+3]=att9;
			break;
		case 9:
			info[k+3]=att10;
			break;
		case 10:
			info[k+3]=att11;
			break;
		}//swat
		if((at==1)&&(sec_ass!=0))
		{
			info[k+4]=0x01;
			info[k+5]=0x01;
			info[k+6]=0x0f;
			info[k+7]=sec_ass;
			k=k+3;
		}
		else
		info[k+4]=0x00;//no_access_selector
		info_l5();
	}//foracce

	info[k++]=0x01;//method_access_descriptor
	info[k++]=method;
	for(i_f=0;i_f<method;i_f++)
	{
		structure(2);
		info[k]=0x0f;
		info[k+1]=i_f+1;
		info[k+2]=0x16;//enum
		switch(i_f)
		{//swat
		case 0:
			info[k+3]=m1;
			break;
		case 1:
			info[k+3]=m2;
			break;
		case 2:
			info[k+3]=m3;
			break;
		case 3:
			info[k+3]=m4;
			break;
		}//swat
		// info[k+3]=access[i_f];
		k=k+4;
	}
}//access_rights


void val_4byt(unsigned char a,unsigned char b,unsigned char c,unsigned char d)
{//val_4byt
	info[k]=0x06;//double_long_unsigned
	info[k+1]=a;//byte-1
	info[k+2]=b;//byte-2
	info[k+3]=c;//byte-3
	info[k+4]=d;//byte-4
	info_l5();
}//val_4byt
void bit_string(u8 length,u8 flag)
{
	if(flag)
	info[k++]=0x00;
	info[k++]=0x04;//bit string
	info[k++]=length;
}
void val_4byt2(unsigned char a,unsigned char b,unsigned char c,unsigned char d)
{//val_4byt
	info[k++]=0x00;
	val_4byt(a,b,c,d);
}//val_4byt

void val_2byt_signed(unsigned char a,unsigned char b)
{//val_2byt
	info[k++]=0x10;//integer
	info[k++]=a;//byte-1
	info[k++]=b;//byte-2
	//130807 info_len=k;
}//val_2byt

void val_2byt(unsigned char a,unsigned char b)
{//val_2byt
	info[k++]=0x12;//integer
	info[k++]=a;//byte-1
	info[k++]=b;//byte-2
	//130807 info_len=k;
}//val_2byt

void val_2byt2(unsigned char a,unsigned char b)
{//val_2byt2
	info[k++]=0x00;
	val_2byt(a,b);
}//val_2byt2

void enum_d(unsigned char len)
{
	info[k++]=0x16;
	info[k++]=len;
}

void sca_unit(char a,unsigned char b,unsigned char add_zero)
{//sca_unit
	if(add_zero==1)
	info[k++]=0x00;
	structure(0x02);
	integer8(a);
	enum_d(b);
}//sca_unit



#if DailyEnergy == 2
void Sel_DailyLoadsurvey_buffer(void)
{
	unsigned char j;
	u8 i_daily;
	unsigned int dcounter,diff1,filled_k=0,min_days;
	u8 u8temp=0;
	if(GPR_u8TCPIP_F==1)
	{
		k=5;
		Start_Info2();
		k=20;
	}
	else
	{
		k=0;
		Start_Info2();
		k=15;
	}
	filled_k=k;
	if(buffer_first_not_fill_f==0)
	{
		if(sel_access_flag == 1)
		{
			if(DE.b.dls_of == 0)
			{
				Eprom_Read(dloadsurvey_init_add);
				min_days=sel_datediff((opr_data[4]),(opr_data[3]),(opr_data[2]));
			}
			
			if(from_val[0]>dt.year || to_val[0]>dt.year || access_selector!=1)			//in case of valid from value it will not send data
			{
				dls_count_dlms=0;
			}
			else
			{
				diff1=sel_datediff(dt.day,dt.month,dt.year);
				to_days = sel_datediff(to_val[2],to_val[1],to_val[0]);
				from_days = sel_datediff(from_val[2],from_val[1],from_val[0]);
				if(Correct_DE_entry_f==1)
				{
					from_days = from_days+1;
				}
				to_cntr_d = 250;
				if((to_days > diff1) || (to_days < from_days) || (from_days > diff1))
				{
					dls_count_dlms=0;
				}
				else
				{	
					if(daily_enrcount==0)
					dcounter=maxday_counter_d; //maxday_counter_l;
					else
					dcounter=daily_enrcount;//a8_to_u16(&opr_data[0]);
					R_WDT_Restart();
					for(i_daily=dcounter-1,j=0;j<maxday_counter_d;j++)
					{
						Eprom_Read(dloadsurvey_init_add+(u16)((u16)i_daily * 0x0020));
						diff1=sel_datediff(opr_data[4],opr_data[3],opr_data[2]);
						if(to_days >=diff1 && diff1!=0)
						{
							to_cntr_d = i_daily;
							break;
						}
						i_daily = i_daily-1;
						if(i_daily==255)
						i_daily=maxday_counter_d-1;
						if(j%20==0)
						{
							R_WDT_Restart();
						}
					}
				}
				if(to_cntr_d == 250)
				{
					dls_count_dlms=0;
					to_cntr_d = 0;
				}
				else
				{
					from_cntr_d = 250;
					Eprom_Read(dloadsurvey_init_add+((dcounter-1)*0x20));
					diff1=sel_datediff(opr_data[4],opr_data[3],opr_data[2]);
					if(from_days > diff1)
					{
						dls_count_dlms=0;
					}
					else
					{
						R_WDT_Restart();
						for(i_daily=to_cntr_d,j=0;j<maxday_counter_d;j++)
						{
							Eprom_Read(dloadsurvey_init_add+(u16)((u16)i_daily * 0x0020));
							diff1=sel_datediff(opr_data[4],opr_data[3],opr_data[2]);
							if(from_days >= diff1 && diff1 !=0)
							{
								if(from_days == diff1)
								{
									from_cntr_d =i_daily;
									break;
								}
								else
								{
									from_cntr_d =i_daily+1;// if from_days=24 value not present in the d_array=25,23,22 selection of 25
									if(from_cntr_d>=maxday_counter_d)//in case of rollover
									from_cntr_d=0;
									if(j==0)                //if from_cntr greater than to_cntr
									from_cntr_d=250;
									break;
								}
							}
							if((DE.b.dls_of==0)&&(from_days<min_days))
							{
								from_cntr_d=0;
								break;
							}

							i_daily = i_daily-1;
							if(i_daily==255)
							i_daily=maxday_counter_d-1;
							if(j%20==0)
							{
								R_WDT_Restart();
							}
						}
						R_WDT_Restart();
					}
				}
				if(from_cntr_d == 250)
				{
					if(DE.b.dls_of==1)
					{
						if(dcounter==maxday_counter_d)
						{
							Eprom_Read(dloadsurvey_init_add);
							diff1=sel_datediff(opr_data[4],opr_data[3],opr_data[2]);
							if(diff1>from_days)
							{
								from_cntr_d=0;
							}
						}
						else
						{
							Eprom_Read(dloadsurvey_init_add+((dcounter+1) * 0x20));
							diff1=sel_datediff(opr_data[4],opr_data[3],opr_data[2]);
							if(diff1>from_days)  //after rollover from_days less than lowest value
							{
								from_cntr_d=(u8)dcounter;//+1;                        
							} 
							else
							{
								dls_count_dlms=0;
							}
						}
					}
					else if((DE.b.dls_of==0)&&(dcounter==maxday_counter_d))
					{
						Eprom_Read(dloadsurvey_init_add+((dcounter) * 0x20));
						diff1 = sel_datediff(opr_data[4],opr_data[3],opr_data[2]);
						if(diff1>from_days)
						{
							from_cntr_d=0;
						}
						else
						{
							dls_count_dlms=0;
						}
					}
					else
					{
						dls_count_dlms=0;
					}
				}	
				if(dls_count_dlms!=0)        
				{
					if(to_cntr_d>from_cntr_d)
					{
						dls_count_dlms=to_cntr_d-from_cntr_d+1;
					}
					else
					{
						dls_count_dlms=to_cntr_d-from_cntr_d+1;
						if((dls_count_dlms>maxday_counter_d)|| (dls_count_dlms==0))
						{
							dls_count_dlms += maxday_counter_d;
						}
					}
					//					for(i_daily=0;i_daily<from_cntr_d;i_daily++)
					//					{
					UintLoadSurptr1=dloadsurvey_init_add+(u16)from_cntr_d*0x0020;
					//					}
				}
			}
			if((to_days==from_days)&&(from_cntr_d>to_cntr_d))
			{
				dls_count_dlms=0;
			}
		}
	}
	if(sel_access_flag == 0)
	no_bytes=36;
	else if(sel_access_flag == 1)
	{
		no_bytes=2;
		if(sel_obj[0] == 1)
		no_bytes+=14;
		if(sel_obj[1] == 1)
		no_bytes+=5;
		if(sel_obj[2] == 1)
		no_bytes+=5;
		if(sel_obj[3] == 1)
		no_bytes+=5;
		if(sel_obj[4] == 1)
		no_bytes+=5;
	}
	if(buffer_first_not_fill_f==0)
	{
		//array((u8)dls_count_dlms,0);
		info[k++]=0x01;//array
		info[k++]=0x82;
		info[k++]=(u8)(dls_count_dlms/0x100);//No Of load Survey
		info[k++]=(u8)(dls_count_dlms%0x100);
		block_size=0;
		block_no=1;
		multi_filling_f=1;
		buffer_first_not_fill_f=1;
		if(dls_count_dlms == 0)
		goto cnp;
	}
	for(;block_size<dls_count_dlms;block_size++)  // THIS LOOP CAN RUN FOR MAXIMUM NUMBER OF 15 TIMES.
	{
		filled_k+=no_bytes;//6 byte increment on each fill
		if(UintLoadSurptr1 == (u16)(dloadsurvey_init_add+(u16)(maxday_counter_d*2*0x10)))//maxday_counter_l_add
		{
			UintLoadSurptr1= dloadsurvey_init_add;
		}
		daily_enr_fill(UintLoadSurptr1);   //EXECUTION TIME === 16.500 ms for one daily_enr_fill   ===>>>Total time to execute for loop 16.500*15=~249ms
		UintLoadSurptr1 = UintLoadSurptr1+ 0x20 ;
		if(DLMS_MAX_BUFF_SIZE<(filled_k+no_bytes))
		{
			break;
		}
	}
cnp:
	block_size++;
	if(block_size>=dls_count_dlms)
	{
		multi_filling_f=0;
		buffer_first_not_fill_f=0;
		u8temp=1;
	}
	send_data(u8temp);
}
#endif

void load_survey_fill(unsigned int address)
{
	structure(no_obj);
	Eprom_Read(address);
	if(sel_obj[0]==1)
	date_time(opr_data[4],opr_data[3],opr_data[2],opr_data[1],opr_data[0],0,0);
	if(sel_obj[1]==1)
	val_2byt(opr_data[9],opr_data[10]);//voltage
	if(sel_obj[2]==1)
	val_2byt(opr_data[5],opr_data[6]);//kwh
	if(sel_obj[3]==1)
	val_2byt(opr_data[7],opr_data[8]);//kvah
	if(sel_obj[4]==1)
	val_2byt(opr_data[11],opr_data[12]);//Q1-LAG KVARH
	if(sel_obj[5]==1)
	val_2byt(opr_data[13],opr_data[14]);//Q2-LEAD KVARH
}

void Sel_Loadsurvey_buffer(void)
{
	unsigned char j,i_l,u8temp=0;
	unsigned int from_cntr1,to_cntr1;
	unsigned int buffer_filled_u16=0;
	unsigned int miss_to_day=0,miss_from_day=0;
	unsigned int prev_from_val_min=0,prev_from_val_hr=0;
	unsigned int rollover_case=0;

	if(GPR_u8TCPIP_F==1)
	{
		k=5;
		Start_Info2();
		k=20;
	}
	else
	{
		k=0;
		Start_Info2();
		k=15;
	}
	
	if(buffer_first_not_fill_f==0)
	{
		buffer_first_not_fill_f=1;
		block_no=1;
		multi_filling_f=1;
		if(sel_access_flag == 1)
		{
			if(from_val[0]>dt.year || to_val[0]>dt.year || access_selector!=1)			//in case of valid from value it will not send data
			{
				ls_count_dlms=0;
			}
			else
			{
				to_days = sel_datediff(to_val[2],to_val[1],to_val[0]);
				from_days = sel_datediff(from_val[2],from_val[1],from_val[0]);
				to_cntr = 250; //to be discussed  d_array size
				if(to_days > d_array[day_counter_ls] || to_days < from_days)
				ls_count_dlms=0;
				else if(to_days == d_array[day_counter_ls] && (to_val[3] >dt.hour || (to_val[3] ==dt.hour && to_val[4] >dt.min)))
				ls_count_dlms=0;
				else
				{
					for(i_l=day_counter_ls,j=0;j<maxday_counter_l;j++)
					{
						if(to_days >= d_array[i_l] && d_array[i_l] !=0)// || to_days > d_array[i])
						{
							to_cntr =i_l;
							if(to_days >d_array[i_l])
							{
								miss_to_day=1;
								to_cntr=to_cntr+1;
								to_val[3]=0x00;
								to_val[4]=0x00;
							}
							break;
						}
						i_l = i_l-1;
						if(i_l==255)
						i_l=maxday_counter_l-1;
					}
				}
                if(to_cntr == 250)
				{
					ls_count_dlms=0;
					to_cntr = day_counter;
				}
				else
				{
					from_cntr = 250;
					if(from_days > d_array[day_counter_ls])
					ls_count_dlms=0;
					else
					{
						for(i_l=to_cntr,j=0;j<maxday_counter_l;j++)
						{
							if(from_days >= d_array[i_l] && d_array[i_l] !=0)// || to_days > d_array[i])
							{
								if(from_days == d_array[i_l])
								{
									from_cntr =i_l;
									break;
								}
								else
								{
									from_cntr =i_l+1;// if from_days=24 value not present in the d_array=25,23,22 selection of 25
									miss_from_day=1;
									prev_from_val_hr=from_val[3];
									prev_from_val_min=from_val[4];
									from_val[3]=0x00;
									from_val[4]=0x00;
									if(from_cntr>=maxday_counter_l)//in case of rollover
									from_cntr=0;
									if(j==0)                //if from_cntr greater than to_cntr
										from_cntr=250;
									break;
								}
							}
							if((LS.b.ls_of==0)&&(d_array[i_l]==0)) //not rollover ,vlaue less than d_array[0]
							{
								from_cntr = 0;
								from_val[3]=0x00;
								from_val[4]=0x00;
								break;
							}
							i_l = i_l-1;
							if(i_l==255)
							i_l=maxday_counter_l-1;
						}
					}
					if(from_cntr == 250)
					{
						if(LS.b.ls_of==1)
						{
							if(day_counter_ls==maxday_counter_l-1)
							{
								if(d_array[0]>from_days)
								{
									from_cntr=0;
									from_val[3]=0x00;
									from_val[4]=0x00;
								}
							}
							else
							{
								if(d_array[day_counter_ls+1]>from_days)  //after rollover from_days less than lowest value
								{
									from_cntr=day_counter_ls+1;
									from_val[3]=0x00;
									from_val[4]=0x00;
								}
								else
								{
									ls_count_dlms=0;
								}
							}
						}
						else if((LS.b.ls_of==0)&&(day_counter_ls>=maxday_counter_l-1))
						{
							if(d_array[0]>from_days)
							{
								from_cntr=0;
								from_val[3]=0x00;
								from_val[4]=0x00;
							}else
							{
								ls_count_dlms=0;
							}
						}else
						{
							ls_count_dlms=0;
						}
					}
				}
				if(ls_count_dlms!=0)// && LS.b.ls_of!=1)
				{
					from_val[3]=bcd_to_hex(from_val[3]);
					from_val[4]=bcd_to_hex(from_val[4]);
					to_val[3]=bcd_to_hex(to_val[3]);
					to_val[4]=bcd_to_hex(to_val[4]);

					from_cntr1=((24*(60/ls_ip)*((u16)(from_cntr)))+(from_val[3]*(60/ls_ip))+(from_val[4]/(ls_ip)));

					if((from_val[4]%(ls_ip))!=0)
					{
						from_cntr1+=1;
					}
					to_cntr1=((24*(60/ls_ip)*(u16)(to_cntr))+(to_val[3]*(60/ls_ip))+(to_val[4]/(ls_ip)));
					if((from_cntr1==0)&&(to_cntr1==0)&&(1!=LS.b.ls_of))
					{
						ls_count_dlms=0;
					}
					else
					{
						if((from_val[4]==0x00)&&(from_val[3]==0x00)&&((d_array[from_cntr]>d_array[from_cntr-1])&&(d_array[from_cntr]-d_array[from_cntr-1])>1)&&(from_cntr!=0)&&(miss_from_day!=1))
						{
							from_cntr1+=1;
						}
						if(((((prev_from_val_hr>0x00)||((prev_from_val_hr==0)&&(prev_from_val_min>0x00)))&&((from_days-d_array[from_cntr-1])==1))||((from_days-d_array[from_cntr-1])>1))&&miss_from_day==1)
						{
							from_cntr1+=1;
						}
						if(from_cntr1>0)
						{
							from_cntr1=from_cntr1-1;
						}
						if(to_cntr1>0)
						{
							to_cntr1=to_cntr1-1;
						}
						if((0==from_cntr1)&&(1==LS.b.ls_of)&&(0==to_cntr1))
						{
							if((from_val[4]>0)&&(to_val[4]>0)&&(to_val[4]<(ls_ip)))
							{
								ls_count_dlms=0;
							}
							else if(from_val[4]>0)
							{
								from_cntr1=0;
								ls_count_dlms=(to_val[4]/(ls_ip));
								rollover_case=1;
							}
							else
							{
								from_cntr1=max_loadsurvey-1;
								ls_count_dlms= 1+(to_val[4]/(ls_ip));
								rollover_case=1;
							}
						}                
						else if((0==from_cntr1)&&(1==LS.b.ls_of)&&(to_cntr1>0))
						{
							if(from_val[4]>0)
							{
								from_cntr1=0;
								ls_count_dlms=to_cntr1+1;
								rollover_case=1;
							}
							else
							{
								from_cntr1=max_loadsurvey-1;
								ls_count_dlms= to_cntr1+1+1;
								rollover_case=1;
							}
						}                
						else if((0==to_cntr1)&&(from_cntr1>1))
						{
							if(0==to_val[4])
							{
								ls_count_dlms = max_loadsurvey - from_cntr1;
							}
							else
							{
								ls_count_dlms = max_loadsurvey - from_cntr1+1;
							}
						}
						else if(to_cntr1<from_cntr1)
						{
							ls_count_dlms = max_loadsurvey-(from_cntr1-to_cntr1-1);
						}
						else
						{
							ls_count_dlms=to_cntr1 - from_cntr1+1;
						}
					}
//					ls_count_dlms=to_cntr1 - from_cntr1+1;
					if(((1==miss_to_day)&&(1==miss_from_day)&&((prev_from_val_hr>0x00)||((0==prev_from_val_hr)&&(prev_from_val_min>0x00)))&&(to_cntr==from_cntr)))
					{
						ls_count_dlms=0;
					}
					if((1==miss_from_day)&&((prev_from_val_hr>0x00)||((0==prev_from_val_hr)&&((prev_from_val_min>0x00)||(((d_array[from_cntr]-d_array[from_cntr-1])>2)&&(1!=miss_to_day)&&(from_days>(d_array[from_cntr-1]+1))))))
					  &&((0==to_val[3])&&(to_val[4]<(ls_ip)))&&(to_cntr==from_cntr))
					{
						ls_count_dlms=0;
					}
					if(ls_count_dlms>=max_loadsurvey)
					{
						ls_count_dlms += max_loadsurvey;
					}
					if(from_cntr1>=max_loadsurvey)
					UintLoadSurptr = from_cntr1-max_loadsurvey;
					else
					UintLoadSurptr = from_cntr1;


				}

			}
			if((to_days==from_days)&&(from_cntr1>to_cntr1)&&(rollover_case!=1))
			{
				ls_count_dlms=0;
			}
		}
	}
	if(sel_access_flag==0)
	{
		no_bytes = 31;
		no_obj=load_survey_parameter_cap_obj[0];
		for(i_l =0;i_l<no_obj;i_l++)
		sel_obj[i_l]=1;
	}
	else if(sel_access_flag == 1)
	{
		no_bytes=2;
		if(sel_obj[0] == 1)
		no_bytes+=14;
		if(sel_obj[1] == 1)
		no_bytes+=3;
		if(sel_obj[2] == 1)
		no_bytes+=3;
		if(sel_obj[3] == 1)
		no_bytes+=3;
		if(sel_obj[4] == 1)
		no_bytes+=3;
		if(sel_obj[5] == 1)
		no_bytes+=3;
	}
	if(block_no==1)
	{
		info[k++]=0x01;//array
		info[k++]=0x82;
		info[k++]=(u8)(ls_count_dlms/0x100);//No Of load Survey
		info[k++]=(u8)(ls_count_dlms%0x100);//
		buffer_filled_u16 = (DLMS_MAX_BUFF_SIZE-k-2)/no_bytes;
	}
	else
	{
		buffer_filled_u16 = (DLMS_MAX_BUFF_SIZE-k)/no_bytes;
	}
	for(i_l=0;i_l<buffer_filled_u16;i_l++,UintLoadSurptr++)//3
	{
		if(UintLoadSurptr >= max_loadsurvey)
		{
			from_cntr1= loadsurvey_init_add;
			MEM_Device_add=0;
			UintLoadSurptr=0;
		}
//		else if(UintLoadSurptr >= max_loadsurvey_1)
//		{
//			from_cntr1=loadsurvey_init_add_1+((UintLoadSurptr-max_loadsurvey_1)*0x10);
//			MEM_Device_add=1;
//		}
		else
		{
			from_cntr1=loadsurvey_init_add+(UintLoadSurptr*0x10);
			MEM_Device_add=0;
		}

		if((ls_count_dlms==ls_count_local))
		{
			multi_filling_f=0;
			buffer_first_not_fill_f=0;
			u8temp=1;
			MEM_Device_add=0;
			break;
		}
		ls_count_local++;
		load_survey_fill(from_cntr1);
	}
	send_data(u8temp);
}
		
void profile_sel(unsigned int ic,unsigned char att,unsigned char obis_a,unsigned char obis_b,unsigned char obis_c,unsigned char obis_d,unsigned char obis_e,unsigned char obis_f)
{//profile_sel
	structure(0x04);
	val_2byt(ic/256,ic%256);//data class
	obiscode(obis_a,obis_b,obis_c,obis_d,obis_e,obis_f);
	integer8(att);//attribute
	val_2byt(0x00,0x00);//data index

}//profile_sel


void tpr_fill1(unsigned int comptt_address)
{
	unsigned char i1;

	if(sel_access_flag==1)
	structure(no_obj);
	else
	{
		for(i1=0;i1<6;i1++)
		sel_obj_tamper[i1]=1;
		structure(6);
	}

	Eprom_Read(comptt_address);

	if(sel_obj_tamper[0]==1)//14
	date_time(opr_data[4],opr_data[3],opr_data[2],opr_data[1],opr_data[0],0,0);//d&t


	if(sel_obj_tamper[1]==1)//3
	{
		val_2byt(opr_data[5],opr_data[6]);
	}

	if(sel_obj_tamper[2]==1)//5//tamper status
	{
		bit_string(0x10,0);
		info[k++]=opr_data[7];
		info[k++]=opr_data[8];
	}


	if(sel_obj_tamper[3]==1)//3
	{
		bit_string(8,0);
		info[k++]=opr_data[9];
	}
	//   unsigned8(opr_data[9],0);//rly disconnection reason

	if(sel_obj_tamper[4]==1)//3
	{
		val_2byt(opr_data[10],opr_data[11]);//pf
	}

	if(sel_obj_tamper[5]==1)//5
	val_4byt(0,opr_data[12],opr_data[13],opr_data[14]);//cum kwh



}


void tpr_fill(unsigned int comptt_address)
{
	union change32 temp;  
	//	unsigned int temp;
	unsigned char i;
	
	if(sel_access_flag==1 || sel_access_flag1==1)
	{
		structure(no_obj);
	}
	else
	{
		for(i=0;i<7;i++)
		sel_obj_tamper[i]=1;
		structure(7);
	}
	Eprom_Read(comptt_address);
	
	if(sel_obj_tamper[0]==1)//14
	date_time(opr_data[4],opr_data[3],opr_data[2],opr_data[1],opr_data[0],0,0);//d&t
	
	if(sel_obj_tamper[1]==1)//3
	{
		if(obis_code[4]==6)
		{
			if(opr_data[5])
			{
				temp.u16_temp[0]=opr_data[5]+300;
			}
			else
			{
				temp.u16_temp[0]=0;
				val_2byt(temp.u8_temp[1],temp.u8_temp[0]);
			}
		}
		else
		{
			val_2byt(0,opr_data[5]);
		}
	}
	
	if(sel_obj_tamper[2]==1)//5
	val_4byt(0,opr_data[6],opr_data[7],opr_data[8]);//iprms
	
	if(sel_obj_tamper[3]==1)//3
	val_2byt(opr_data[9],opr_data[10]);//vrms
	
	if(sel_obj_tamper[4]==1)//3
	{
		temp.u16_temp[0] =(unsigned int)opr_data[11];//*10;
		val_2byt(temp.u8_temp[1],temp.u8_temp[0]);//pf
	}
	
	if(sel_obj_tamper[5]==1)//5
	val_4byt(0,opr_data[12],opr_data[13],opr_data[14]);//cum kwh

	Eprom_Read(comptt_address+0x10);

	if(sel_obj_tamper[6]==1)//5
	val_4byt(0,opr_data[0],opr_data[1],opr_data[2]);//cum kvah
}
#if DailyEnergy == 2
void daily_enr_fill(unsigned int comptt_address)
{

	unsigned char i;

	if(sel_access_flag == 1)
	structure(no_obj);
	else
	{
		for(i=0;i<dailyload_profile_parameter_cap_obj[0];i++)
		sel_obj[i]=1;

		structure(dailyload_profile_parameter_cap_obj[0]);
	}


	Eprom_Read(comptt_address);

	if(sel_obj[0]==1)
	date_time(opr_data[4],opr_data[3],opr_data[2],opr_data[1],opr_data[0],0,0);//d&t

	if(sel_obj[1]==1)
	val_4byt(0,opr_data[5],opr_data[6],opr_data[7]);//cum kwh

	if(sel_obj[2]==1)
	val_4byt(0,opr_data[8],opr_data[9],opr_data[10]);//cum kvah
	
	Eprom_Read(comptt_address+0x10);
	
	if(sel_obj[3]==1)
	val_4byt(0,opr_data[1],opr_data[2],opr_data[3]);//cum Q1 lag-kvarh

	if(sel_obj[4]==1)
	val_4byt(0,opr_data[4],opr_data[5],opr_data[6]);//cum Q2 lead-kvarh

}
#endif

void resp(void)
{
	//resp
	unsigned char local_dlms_i;
	static union
	{
		u8 a8Temp[2];
		u16 u16Temp;
	}unionTemp;

	asserr_flag=0;//association type of error
	ass_ser=0;//acse field
	assresult_flag=0;//association result
	conf_ser_flag=0;//conformation service flag
	conf_err_flag=0;//conformation sub field type
	conf_type_flag=0;//type of error in conformation.
	conf_serror_flag=0;//conformation main field type

	//for(local_dlms_i=0;local_dlms_i<3;local_dlms_i++)
	//info[local_dlms_i]=Start_Info_CONT[local_dlms_i];

	memcpy(info,Start_Info_CONT,3);

	seg_flagsd=0;


	switch(req_typ)
	{//swreq
	case 0x60://aarq request
		assresult_flag=data_dec();
		seg_type=0xA0;
		frame_type=0;
		if(assresult_flag==0x01)
		{//ifsuccess
			if(four_pass_f==0)
			cosem_flag =1;
			else
			cosem_flag =0;
		}//ifsuccess
		else
		{//elsefail
			cosem_flag=0;
		}//elsefail
//		if(GPR_u8TCPIP_F==1)
//		k=8;
//		else
		k=3;
		fill_info(AARQ);

		/*info[3]=0x61;
		//application_context_name
		info[5]= 0xA1;
		info[6]= 0x09;
		info[7]= 0x06;
		info[8]= 0x07;
		info[9]= 0x60;
		info[10]= 0x85;//country
		info[11]= 0x74;//country name
		info[12]= 0x05;//Organisation
		info[13]= 0x08;//DLMS_UA
		info[14]= 0x01;//app_context
		info[15]= 0x01;//LN reference
		//application_context_name

		//result
		info[16]= 0xA2;//tag of result component
		info[17]= 0x03;//length of taged component
		info[18]= 0x02;//choice for result
		info[19]= 0x01;//length of result*/

		if(assresult_flag==0x01)
		info[k-8]= 0x00;//result
		else
		info[k-8]= 0x01;
		//result

		//result_source_diagnostic
		/*info[21]= 0xA3;//tag for result diagnostic
		info[22]= 0x05;//length of taged component*/
		info[k-5]= ass_ser;//tag for acse_service_user/provider(0xA1/0xA2)
		/*info[24]= 0x03;//length
		info[25]= 0x02;//choice of result_source_diagnostic
		info[26]= 0x01;//length of value*/
		info[k-1]= asserr_flag;//value of diagnosis
		//result_source_diagnostic

		//        k=28;
		//HLS
		if(asso2_flag==1)
		{
			k--;
			//info[k++]= 0x00;
			fill_info(AARE_PASS);
			/*
			info[k++]= 0x0E;
			info[k++]= 0x88;// encoding of the tag of the acse-requirements field ([8], IMPLICIT, Context-specific)
			info[k++]= 0x02;// encoding of the length of the tagged component�s value field.
			info[k++]= 0x07;// encoding of the number of unused bits in the last byte of the BIT STRING
			info[k++]= 0x80;// encoding of the authentication functional unit (0)
			info[k++]= 0x89;// encoding of the tag ([9], IMPLICIT, Context-specific)
			info[k++]= 0x07;// encoding of the length of the tagged component�s value field
			info[k++]= 0x60;// encoding the value of the object identifier:- high-level-security-mechanism-name (5)
			info[k++]= 0x85;
			info[k++]= 0x74;
			info[k++]= 0x05;
			info[k++]= 0x08;
			info[k++]= 0x02;
			info[k++]= 0x02;
			info[k++]= 0xAA;// encoding of the tag ([10], Context-specific)
			info[k++]= 0x12;// encoding of the length of the tagged component�s value field
			info[k++]= 0x80;// encoding of the choice for Authentication-value (charstring [0] IMPLICIT GraphicString)
			info[k++]= 0x10;// encoding of the length of the Authentication-information�s value field (8 octets)
			*/
			for(local_dlms_i=0;local_dlms_i<16;local_dlms_i++)
			{
				info[k++]=aut_pswd1_23[local_dlms_i];
			}
		}
		/*
		info[k++]= aut_pswd1_2[0];// encoding of the value of the challenge StoC "PQRSTUVW"
		info[k++]= aut_pswd1_2[1];
		info[k++]= aut_pswd1_2[2];
		info[k++]= aut_pswd1_2[3];
		info[k++]= aut_pswd1_2[4];
		info[k++]= aut_pswd1_2[5];
		info[k++]= aut_pswd1_2[6];
		info[k++]= aut_pswd1_2[7];
		info[k++]= aut_pswd1_2[8];// encoding of the value of the challenge StoC "PQRSTUVW"
		info[k++]= aut_pswd1_2[9];
		info[k++]= aut_pswd1_2[10];
		info[k++]= aut_pswd1_2[11];
		info[k++]= aut_pswd1_2[12];
		info[k++]= aut_pswd1_2[13];
		info[k++]= aut_pswd1_2[14];
		info[k++]= aut_pswd1_2[15];
		*/

		//HLS
		//user_information_field_component
		info[k++]= 0xBE;//user_information_tag
		//confirmed service error
		if(conf_ser_flag==1)
		{//conf_ser_flag
			info[k++]= 0x06;//length of tag
			info[k++]= 0x04;//choice of user information
			info[k++]= 0x04;//lengh of octet string
			info[k++]= 0x0E;//tag for conf ser error
			info[k++]= 0x01;//conf_serror_flag;//service error type
			info[k++]= conf_err_flag;//error sub type
			info[k++]= conf_type_flag;//error field
			//			info[k++]= 0x1F;//
			conf_ser_flag=0;
			//			k=36;
		}//conf_ser_flag
		//confirmed service error
		//xDLMS-initiate-response PDU
		else
		{//elseini
			//for(local_dlms_i=0;local_dlms_i<10;local_dlms_i++)
			//info[k++]=A[local_dlms_i];
			memcpy(&info[k],A,10);
			k+=10;
			/*info[k++]= 0x10;//length of tag
			info[k++]= 0x04;//choice of user information
			info[k++]= 0x0E;//lengh of octet string
			info[k++]= 0x08;//tag
			info[k++]= 0x00;//Quality of service(optional not present)
			info[k++]= 0x06;//DLMS version number
			info[k++]= 0x5F;//ASN.1 tag[0]
			info[k++]= 0x1F;//ASN.1 tag[1]
			info[k++]= 0x04;//length of ASN.1
			info[k++]= 0x00;//number of unused bits*/
			//for(local_dlms_i=0;local_dlms_i<3;local_dlms_i++)
			//info[k++]=conf_blk[local_dlms_i];
			memcpy(&info[k],conf_blk,3);
			k+=3;
			/*info[k++]= conf_blk[0];//bit string(fixed length 3 bytes)
			info[k++]= conf_blk[1];//bit string byte2
			info[k++]= conf_blk[2];//bit string byte3*/

			//for(local_dlms_i=10;local_dlms_i<14;local_dlms_i++)
			//{
			//	info[k++]=A[local_dlms_i];//0x01;//max_receive_pdu_size[0]23/02/2007
			//}
			memcpy(&info[k],&A[10],4);
			unionTemp.u16Temp=max_info_rec;
            info[k]=unionTemp.a8Temp[1];
            info[k+1]=unionTemp.a8Temp[0];
			k+=4;
			/*info[k++]= 0x00;//0x01;//max_receive_pdu_size[0]23/02/2007
			info[k++]= 0x80;//0xFF;//max_receive_pdu_size[1]23/02/2007
			info[k++]= 0x00;//VAA-name-component[0]
			info[k++]= 0x07;//VAA-name-component[1]*/
			////k=46;

		}//elseinit
		//xDLMS-initiate-response PDU
		//conf_ser_flag, conf_err_flag,conf_type_flag,conf_serror_flag;
		//user_information_field_component
		if(GPR_u8TCPIP_F==1)
		info[9]=(u8)(k-10);//length
		else
		info[4]=(u8) (k-5);//length

		info_send=k;
		send_type_multi();
		break;
	case 0xC0://get_request
		if(cosem_flag==1)
		{//if cosem_open
			if(info[4]==2)
			{
				vBlock_transfer_list();
			}
			else
			{
				assresult_flag=data_dec();
				if(GPR_u8TCPIP_F==1)
				k=11;
				else
				k=6;
				multi_filling_f=0;
				if(assresult_flag==1)
				{//ifresult
					seg_flagsd=0;
					multi_resp=0;
					if((asso1_flag==1)||(asso2_flag==1))
					{
						get_resp();
					}
					else
					{
						get_resp1();
					}
					if((seg_flagsd==0)&&(multi_resp==0))
					{//ifsegsd
						if(GPR_u8TCPIP_F==1)
						{
							info[8]=0xC4;//get_response
							info[9]=0x01;//normal
							info[10]=invo_prio;//invoke_id,priority
						}
						else
						{
							info[3]=0xC4;//get_response
							info[4]=0x01;//normal
							info[5]=invo_prio;//invoke_id,priority
						}
						info_send=k;
						frame_type=0;
						seg_type=0xA0;
						send_type_multi();
					}else
					{
						multi_resp=0;
					}
				}//ifresult
				else
				{//ifres
					if(GPR_u8TCPIP_F==1)
					k=8;
					else
					k=3;

					info[k++]=0xC4;//get_response
					info[k++]=0x01;//normal
					info[k++]=invo_prio;//invoke_id,priority
					info[k++]=conf_err_flag;
					info[k++]=conf_type_flag;
					//					k=8;
					info_send=k;
					frame_type=0;
					seg_type=0xA0;
					send_type_multi();
				}//ifres
			}
		}//if cosem_open
		else
		{
//			if(GPR_u8TCPIP_F==1)
//			k=8;
//			else
			k=3;
			info[k++]=0x0E;//confirmed_service_error
			info[k++]=0x05;//read
			info[k++]=0x05;//access
			info[k++]=0x01;//scope of access violated
			//			k=7;
			info_send=k;				//k
			frame_type=0;
			send_type_multi();
		}//else cosem_release
		break;
	case 0xC1://set req
		if(cosem_flag==1)
		{//if cosem_open
			assresult_flag=data_dec();
			if(GPR_u8TCPIP_F==1)
			k=8;
			else
			k=3;
			multi_filling_f=0;
			if(assresult_flag!=2)
			{
				info[k++]=0xC5;//set_response
				info[k++]=0x01;//normal
				info[k++]=invo_prio;//invoke_id,priority
			}
			if(assresult_flag==2)
			{
				info[k++]=0xC5; //set response
				if(last_block==1)
				info[k++]=0x03;//last Datablock
				else
				info[k++]=2;//datablock

				info[k++]=invo_prio;//invoke_id,priority
				info[k++]=0;
				info[k++]=0;
				info[k++]=0;
				if(last_block==1)//&& max_info_rec == 128)
				{
					info[k++]=0;//data access result
					info[k++]=block_no;
					//					k=11;
				}
				else
				{
					info[k++]=block_no;
					//					k=10;
				}
			}
			if(assresult_flag==1)
			{//ifresult
				info[k++]=0x00;
				//				k=7;
			}//ifresult
			else if(assresult_flag==0)
			{//ifres
				//info[k++]=conf_err_flag;/*commentted as per SME*/
				info[k++]=conf_type_flag;
				//				k=8;
			}//ifres
			info_send=k;
			send_type_multi();
		}//if cosem_open
        else
		{
			//if(GPR_u8TCPIP_F==1)
			//	k=8;
			//else
				k=3;
			info[k++]=0x0E;//confirmed_service_error
			info[k++]=0x06;//write
			info[k++]=0x05;//access
			info[k++]=0x01;//scope of access violated
			info_send=k;				//k
			frame_type=0;
			send_type_multi();
		}//else cosem_release		
		break;
	case 0xC3://action
		if((asso2_flag==1 && asso0_flag==0 && asso1_flag==0 )||
				(asso3_flag==1 && asso0_flag==0 && asso1_flag==0 ))
		{//if cosem_open
			assresult_flag=data_dec();
			if(GPR_u8TCPIP_F==1)
			k=8;
			else
			k=3;
			multi_filling_f=0;
			info[k++]=0xC7;//ACTION.RESP
			info[k++]=0x01;//normal
			info[k++]=0xc1;//INVOKE_ID,PRIORITY
			if(assresult_flag==1)
			{
				info[k++]=0;//assresult_flag;
			}
			else if(assresult_flag==0)
			{//ifres
				info[k++]=conf_err_flag;
				info[k++]=conf_type_flag;
				//				k=8;
			}//ifres

			if(four_pass_f==1)
			{
				aes_encrypt(aut_pswd1_1,aut_pswd12);
				info[k++]=0x01;
				octet_s(16,0);
				memcpy(&info[k],aut_pswd1_1,16);
				k+=16;
			}
			info_send=k;
			send_type_multi();
			four_pass_f=0;
		}
		else 
		{
			//if(GPR_u8TCPIP_F==1)
			//	k=8;
			//else
				k=3;
			info[k++]=0x0E;//confirmed_service_error
			info[k++]=0x06;//write
			info[k++]=0x05;//access
			info[k++]=0x01;//scope of access violated
			info_send=k;				//k
			frame_type=0;
			send_type_multi();
		}		
		break;
	case 0x05:
		//info[3]=0x0E;//confirmed_service_error
		//info[4]=0x05;//read
		//info[5]=0x05;//access
		//info[6]=0x01;//scope of access violated
		//k=7;
		//info_len=k;
		//send_type();
		//break;
	case 0x06:
	default:
		info[3]=0x0E;//confirmed_service_error
		info[4]=0x05;//read
		info[5]=0x05;//access
		info[6]=0x01;//scope of access violated
		k=7;
		info_send=k;
		frame_type=((rrr_s<<5)|(0x10)|(sss_s<<1));
		seg_type=0xA0;
		send_type_multi();
		break;

	}//swreq

}//resp End resp

void vBlock_transfer_list(void)
{
	switch(obis_short)
	{
	case 0x00280000:
	case 0x00280001:
	case 0x00280002:
	case 0x00280003:
		if((attribute_id==2))
		object_list();
		break;
	case 0x000D0000:
		if((attribute_id == 5))
		{
			if(TOD_bActive_Calendar == 0)
			day_profile(0);
			else if(TOD_bActive_Calendar == 1)
			day_profile(1);
		}
		if(attribute_id == 9)
		{
			if(TOD_bActive_Calendar == 0)
			day_profile(1);
			else if(TOD_bActive_Calendar == 1)
			day_profile(0);
		}
		break;
	case 0x01620100:
		if(attribute_id==2)
		bill_buffer();

		if(attribute_id==3)
		capture_objects_filler(bill_profile_parameter_cap_obj);

		break;
	  case 0x00636200:
		if(attribute_id==2)
		{
			tamper_compart(volt_max_loc,volt_init_add,volt_max_add);
		}
		if(attribute_id == 3)
		{
			capture_objects_filler(voltage_event_capture_obj);//tamper_capture_objects();
		}
		break;
	  case 0x00636201:
		if(attribute_id==2)
		{
			tamper_compart(curr_max_loc,curr_init_add,curr_max_add);
		}
		if(attribute_id == 3)
		{
			capture_objects_filler(current_event_capture_obj);//tamper_capture_objects();
		}
		break;
	  case 0x00636202:
		if(attribute_id==2)
		{
			tamper_compart(pwr_max_loc,pwr_init_add,pwr_max_add);
		}
		if(attribute_id == 3)
		{
			capture_objects_filler(power_event_capture_obj);//tamper_capture_objects();
		}
		break;
	  case 0x00636203:
		if(attribute_id==2)
		{
			tamper_compart(trans_max_loc,trans_init_add,trans_max_add);
		}
		if(attribute_id == 3)
		{
			capture_objects_filler(transaction_event_capture_obj);//tamper_capture_objects();
		}
		break;
	  case 0x00636204:
		if(attribute_id==2)
		{
			tamper_compart(other_max_loc,other_init_add,other_max_add);
		}
		if(attribute_id == 3)
		{
			capture_objects_filler(other_event_capture_obj);//tamper_capture_objects();
		}
		break;
	  case 0x00636205:
		if(attribute_id == 2)
		{
			tamper_compart(nonroll_max_loc,nonroll_init_add,nonroll_max_add);
		}
		if(attribute_id == 3)
		{
			capture_objects_filler(non_rollover_event_capture_obj);//tamper_capture_objects();
		}
		break;
	  case 0x00636206:
		if(attribute_id==2)
		{
			tamper_compart((control_max_loc),control_init_add,control_max_add);//tamper_compart(16,0x1900,0x1a00);
		}
		if(attribute_id == 3)
		{
			capture_objects_filler(control_event_capture_obj);//tamper_capture_objects();
		}
		break;
	  case 0x00636207:
		if(attribute_id==2)
		{
			tamper_compart((Diagnostics_max_loc),Diagnostics_init_add,Diagnostics_max_add);//tamper_compart(16,0x1900,0x1a00);
		}
		if(attribute_id == 3)
		{
			capture_objects_filler(Diagnostics_event_capture_obj);//tamper_capture_objects();
		}
		break;
	case 0x015e5b00:
		if(attribute_id == 3)
		capture_objects_filler(instantaneous_parameter_cap_obj);//inst_para_cap_obj(0);
		break;
	case 0x015e5b03:
		if(attribute_id == 2)
		buffer_scaler_filler(instantaneous_parameter_scaler_buffer);//inst_para_buffer_s();
		if(attribute_id==3)
		capture_objects_filler(instantaneous_parameter_scaler_cap_obj);// inst_para_cap_obj('s');
		break;
	case 0x015e5b04:
		if(attribute_id==3)
		capture_objects_filler(blockload_survey_parameter_scaler_cap_obj);//load_survey_cap_obj('s');
		break;
#if DailyEnergy == 2        
	case 0x015e5b05:
		if(attribute_id==3)
		capture_objects_filler(dailyload_profile_parameter_scaler_cap_obj);//load_survey_cap_obj('s');
		break;
#endif

	case 0x015e5b06:
		if(attribute_id==3)
		capture_objects_filler(bill_profile_parameter_scaler_cap_obj);//bill_cap_obj_s();
		if(attribute_id==2)
		buffer_scaler_filler(bill_profile_parameter_scaler_buffer);
		break;
	case 0x015e5b07:
		if(attribute_id == 3)
		capture_objects_filler(event_log_profile_scaler_cap_obj);//event_log_profile_cap_obj_s();
		break;
		// 1,0,99,1,0,255,
	case 0x01630100:
		if(attribute_id==2)
		{

			Sel_Loadsurvey_buffer();
		}

		break;
#if DailyEnergy == 2        
	case 0x01630200:
		if(attribute_id==2)
		{
			Sel_DailyLoadsurvey_buffer();
		}
		break;
#endif
	default:
		if (seg_flagsd==0)
		{//elserr
			if(nrm_flag==1)
			{//ifnrm
				fill_A0(0x11);
			}//ifnrm
			else
			{//elnrm
				fill_A0(0x97);
			}//ielnrm
		}//elserr
		break;
	}
}
void req_type(void)
{//req_type
	unsigned int local_dlms_i1=0;
	//unsigned char cl_obis[4];
	//unsigned long int clid_obis;
	//7e a0 08 02 03 23 93 36 54 7e



	if((cont_field==0x93)||(cont_field==0x53))
	{//ifrec[6]
		switch(cont_field)
		{//swcont
		case 0x93:  //snrm
			if(one_byte_add_f!=1)
			{
				//for(local_dlms_i1=0;local_dlms_i1<4;local_dlms_i1++)
				//server_add[local_dlms_i1]=rec[local_dlms_i1+3];
				memcpy(server_add,&rec[3],4);
				client_add=rec[7];
			}
			else
			{
				server_add[3]=rec[3];//server_lowerlow=rec[3];
				client_add=rec[4];
			}
			if((client_add==0x21))
			{
				asso0_flag=1;
				asso1_flag=0;
				asso2_flag=0;
				conf_blk[0]=0x00;
				conf_blk[1]=0x00;  //changed for only GET for Public client
				conf_blk[2]=0x10;
			}
			else if((client_add==0x41))
			{
				asso0_flag=0;
				asso1_flag=1;
				asso2_flag=0;
				conf_blk[0]=0x00;
				conf_blk[1]=0x10;
				conf_blk[2]=0x14;//0x94(With Data notification)//0x96 (With Event noti too)
			}
			else if((client_add==0x61))
			{
				asso0_flag=0;
				asso1_flag=0;
				asso2_flag=1;
				conf_blk[0]=0x00;
				conf_blk[1]=0x18;
				conf_blk[2]=0x1d;
			}
			else if((client_add==0x81))
			{
				asso3_flag=1;
				conf_blk[0]=0x00;
				conf_blk[1]=0x10;
				conf_blk[2]=0x1d;
			}
			if(((rec[11]!=0x7E)&&(one_byte_add_f==0))||((rec[8]!=0x7E)&&(one_byte_add_f==1)))
			{//if7e
				if(((packet_len<0x17)&&(one_byte_add_f==0))||((packet_len<0x14)&&(one_byte_add_f==1)))
				decerr_flag=1;
				if(((rec[11]!=0x81)&&(one_byte_add_f==0))||((rec[8]!=0x81)&&(one_byte_add_f==1)))
				decerr_flag=1;
				if(((rec[12]!=0x80)&&(one_byte_add_f==0))||((rec[9]!=0x80)&&(one_byte_add_f==1)))
				decerr_flag=1;

				if(one_byte_add_f==1)
				k=11;
				else
				k=14;

				if(rec[k]==0x05)//read req
				{
					k++;

					if(rec[k]==0x02)// data block result
					{
						local_dlms_i1=rec[k+1];
						local_dlms_i1=local_dlms_i1<<8;
						local_dlms_i1=local_dlms_i1+rec[k+2];
						if((local_dlms_i1>=0x20)&&(local_dlms_i1<=DLMS_MAX_BUFF_SIZE))
						max_info_tra=local_dlms_i1;
						else if((local_dlms_i1<0x20)&&(local_dlms_i1==0x00))
						decerr_flag=1;
						else
						max_info_tra=DLMS_MAX_BUFF_SIZE;//70;//0x80;23/02/2007
					}
					else
					{
						if(DLMS_MAX_BUFF_SIZE>0xff)
						local_dlms_i1=0xff;
						else
						local_dlms_i1=DLMS_MAX_BUFF_SIZE;
						if((rec[k+1]>=0x20)&&(rec[k+1]<=local_dlms_i1))
						max_info_tra=rec[k+1];
						else if(rec[k+1]<0x20)
						decerr_flag=1;
						else
						max_info_tra=DLMS_MAX_BUFF_SIZE;//0x80;23/02/2007
					}

					k=rec[k]+k;k++;
					k++;
					if(rec[k]==0x02)
					{
						local_dlms_i1=rec[k+1];
						local_dlms_i1=local_dlms_i1<<8;
						local_dlms_i1=local_dlms_i1+rec[k+2];

						if((local_dlms_i1>=0x20)&&(local_dlms_i1<=DLMS_MAX_BUFF_SIZE))
						max_info_rec=local_dlms_i1;
						else if((local_dlms_i1<0x20)&&(local_dlms_i1==0x00))
						decerr_flag=1;
						else
						max_info_rec=DLMS_MAX_BUFF_SIZE;//0x80;23/02/2007
					}
					else
					{
						if((rec[k+1]>=0x20)&&(rec[k+1]<=0x70))
						max_info_rec=rec[k+1];
						else if(rec[k+1]<0x20)
						decerr_flag=1;
						else
						max_info_rec=DLMS_MAX_BUFF_SIZE;//0x80;23/02/2007

					}
					k=rec[k]+k;k++;
				}

				if(rec[k]==0x07)
				{
					max_win_tra=rec[k+2]*256*256*256+rec[k+3]*256*256+rec[k+4]*256+rec[k+5];
					if(max_win_tra<0x01)
					{//ifk
						decerr_flag=1;
					}//ifk
					else
					max_win_tra=0x01;
					k++;
					k=rec[k]+k;k++;
					max_win_rec=rec[k+2]*256*256*256+rec[k+3]*256*256+rec[k+4]*256+rec[k+5];
					if(max_win_rec<0x01)
					{//ifk
						decerr_flag=1;
					}//ifk
					else
					max_win_rec=0x01;
					k=rec[k]+k;k++;
				}
				if(decerr_flag==1)
				{//ifflag
					fill_A0(0x1F);  //DM mode
					break;
				}//ifflag
			}//if7e

			if(decerr_flag==0)
			{
				frame_type=0x73;
				seg_type=0xA0;

				//for(local_dlms_i1=0;local_dlms_i1<23;local_dlms_i1++)
				//{
				//  info[local_dlms_i1]=B[local_dlms_i1];
				//}
				memcpy(info,B,23);

				info[5]=max_info_tra/256;
				info[6]=max_info_tra%256;
				info[9]=max_info_rec/256;
				info[10]=max_info_rec%256;
				info[16]=max_win_tra;
				info[22]=max_win_rec;


				info_send=23;
				nrm_flag=1;
				cosem_flag=0; //added
				rrr_s=0;
				rrr_c=0;
				rrr_c1=0;
				sss_c=0;
				sss_c1=0;
				sss_s=0;
				infore_flag=0; //added
				Cntr_2Min = 0;//added
				multi_filling_f=0; //added
				for(local_dlms_i1=0;local_dlms_i1<150;local_dlms_i1++)
				rec[local_dlms_i1] = 0x00;

				send_type_multi();
			}
			decerr_flag=0;
			break;
		case 0x53://DISC//for chckin the acceptance of disc
			if(one_byte_add_f!=1)
			{
				//for(local_dlms_i1=0;local_dlms_i1<4;local_dlms_i1++)
				//server_add[local_dlms_i1]=rec[local_dlms_i1+3];
				memcpy(server_add,&rec[3],4);
				client_add=rec[7];
			}
			else
			{
				server_add[3]=rec[3];//server_lowerlow=rec[3];
				client_add=rec[4];
			}
			seg_flagsd=0;
			long_data=0;
			if(nrm_flag==1)
			{//ifnrm
				fill_A0(0x73);
				init_dlmsvar(1);

				optical_f = 0;
				IrDA_f=0;
				rj_f = 0 ;
				rj_disc_f2=0;
				rj_disc_f=0;
				rj_disc_cnt=0;
				rcv_cnt1=0;
				for(local_dlms_i1=0;local_dlms_i1<6;local_dlms_i1++)
				obis_code[local_dlms_i1]=0x00;
			}//ifnrm
			else
			{//elnrm
				seg_flagsd=0;
				nrm_flag=0;
				for(local_dlms_i1=0;local_dlms_i1<6;local_dlms_i1++)
				obis_code[local_dlms_i1]=0x00;
				fill_A0(0x1F);
			}//ielnrm



			break;
		}//swcont End Switch
	}//ifrec[6]

	//data block transfer
	//	else if(((rec[11]==0xE6)&&(rec[12]==0xE6)&&(rec[13]==0x00)&&(Format_type==0xA0)&&(rec[14]==0xC0)&&(rec[15]==0x02)&&(rec[20]==(Data_block-1)))||
	//			((rec[8]==0xE6)&&(rec[9]==0xE6)&&(rec[10]==0x00)&&(Format_type==0xA0)&&(rec[11]==0xC0)&&(rec[12]==0x02)&&(rec[17]==(Data_block-1))&&(one_byte_add_f==1)))
	//	{
	//		if((obis_code[0]==0x01)&&(obis_code[1]==0x00)&&(obis_code[2]==0x63))
	//		{
	//			//buffer_first_not_fill_f=0;
	//			Sel_Loadsurvey_buffer();
	//		}
	//	}
	else if(((rec[11]==0xE6)&&(rec[12]==0xE6)&&(rec[13]==0x00)&&(Format_type==0xA0))||((rec[8]==0xE6)&&(rec[9]==0xE6)&&(rec[10]==0x00)&&(Format_type==0xA0)&&(one_byte_add_f==1)))
	{//elkiran
		if(nrm_flag==1)
		{//ifnrm1
			byte_cont=0;
			if(one_byte_add_f)
			{
				req_typ=rec[11];
				local_dlms_i1=8;
			}
			else
			{
				req_typ=rec[14];
				local_dlms_i1=11;
			}
			for(;local_dlms_i1<(packet_len-2);local_dlms_i1++)
			info[byte_cont++]=rec[local_dlms_i1];
			info[byte_cont]=rec[local_dlms_i1];  

			resp();

		}//ifnrm1
	}//elkiran
	else if(Format_type==0xA8)
	{//ifa8
		if(((rec[11]==0xE6)&&(rec[12]==0xE6)&&(rec[13]==0x00)&&(nrm_flag==1))||
				((rec[8]==0xE6)&&(rec[9]==0xE6)&&(rec[10]==0x00)&&(nrm_flag==1)&&(one_byte_add_f==1)))
		{//ifrem
			seg_flag=1;
			byte_cont=0;
			if(one_byte_add_f)
			{
				req_typ=rec[11];
				local_dlms_i1=8;
			}
			else
			{
				req_typ=rec[14];
				local_dlms_i1=11;
			}

			for(;local_dlms_i1<(packet_len-2);local_dlms_i1++)
			info[byte_cont++]=rec[local_dlms_i1];
			info[byte_cont++]=rec[local_dlms_i1];  //Updated By Dinesh 23-02-2007
		}//ifrem
		else
		{//elrem
			if(one_byte_add_f)
			local_dlms_i1=8;
			else
			local_dlms_i1=11;

			for(;local_dlms_i1<(packet_len-2);local_dlms_i1++)
			{
				info[byte_cont++]=rec[local_dlms_i1];
				if(byte_cont>DLMS_MAX_BUFF_SIZE)
				byte_cont=0;
			}
			info[byte_cont++]=rec[local_dlms_i1];  //Updated By Dinesh 23-02-2007
		}//elrem
		frame_type=((rrr_s<<5)|(0x10)|0x01);
		fill_A0(frame_type);
	}//ifa8
	else if((Format_type==0xA0)&&(seg_flag==1)&&(nrm_flag==1)&&((cont_field&0x0F)!=0x01))
	{//ifa0
		if(one_byte_add_f)
		local_dlms_i1=8;
		else
		local_dlms_i1=11;

		for(;local_dlms_i1<(packet_len-2);local_dlms_i1++)
		info[byte_cont++]=rec[local_dlms_i1];
		info[byte_cont]=rec[local_dlms_i1];  //Updated By Dinesh 23-02-2007
		resp();
		seg_flag=0;
	}//ifa0
	//multipacket transfer
	else if((Format_type==0xA0)&&((cont_field&0x0F)==0x01))
	{
		if(send_type_multi_f==1)
		{
			send_type_multi();
		}
		else
		{
			//		cl_obis[0]=obis_code[0];
			//		cl_obis[1]=obis_code[2];
			//		cl_obis[2]=obis_code[3];
			//		cl_obis[3]=obis_code[4];
			//		clid_obis=a8_to_u32(&cl_obis[0]);

			//		switch(obis_short)
			//		{
			//		  case 0x00280000:
			//		  case 0x00280001:
			//		  case 0x00280002:
			//		  case 0x00280003:
			//			if((attribute_id==2))
			//				object_list();
			//			break;
			//		  case 0x000D0000:
			//			if((attribute_id == 5))
			//			{
			//				if(TOD_bActive_Calendar == 0)
			//					day_profile(0);
			//				else if(TOD_bActive_Calendar == 1)
			//					day_profile(1);
			//			}
			//			if(attribute_id == 9)
			//			{
			//				if(TOD_bActive_Calendar == 0)
			//					day_profile(1);
			//				else if(TOD_bActive_Calendar == 1)
			//					day_profile(0);
			//			}
			//			break;
			//		  case 0x01620100:
			//			if(attribute_id==2)
			//				bill_buffer();
			//
			//			if(attribute_id==3)
			//				capture_objects_filler(bill_profile_parameter_cap_obj);
			//
			//			break;
			//		  case 0x00636200:
			//			if(attribute_id==2)
			//			{
			//				tamper_compart(volt_max_loc+1,volt_init_add,volt_max_add);
			//			}
			//			if(attribute_id == 3)
			//			{
			//				capture_objects_filler(voltage_event_capture_obj);//tamper_capture_objects();
			//			}
			//			break;
			//		  case 0x00636201:
			//			if(attribute_id==2)
			//			{
			//				tamper_compart(curr_max_loc+1,curr_init_add,curr_max_add);
			//			}
			//			if(attribute_id == 3)
			//			{
			//				capture_objects_filler(current_event_capture_obj);//tamper_capture_objects();
			//			}
			//			break;
			//		  case 0x00636202:
			//			if(attribute_id==2)
			//			{
			//				tamper_compart(pwr_max_loc+1,pwr_init_add,pwr_max_add);
			//			}
			//			if(attribute_id == 3)
			//			{
			//				capture_objects_filler(power_event_capture_obj);//tamper_capture_objects();
			//			}
			//			break;
			//		  case 0x00636203:
			//			if(attribute_id==2)
			//			{
			//				tamper_compart(trans_max_loc+1,trans_init_add,trans_max_add);
			//			}
			//			if(attribute_id == 3)
			//			{
			//				capture_objects_filler(transaction_event_capture_obj);//tamper_capture_objects();
			//			}
			//			break;
			//		  case 0x00636204:
			//			if(attribute_id==2)
			//			{
			//				tamper_compart(other_max_loc+1,other_init_add,other_max_add);
			//			}
			//			if(attribute_id == 3)
			//			{
			//				capture_objects_filler(other_event_capture_obj);//tamper_capture_objects();
			//			}
			//			break;
			//		  case 0x00636205:
			//			if(attribute_id == 3)
			//			{
			//				capture_objects_filler(non_rollover_event_capture_obj);//tamper_capture_objects();
			//			}
			//			break;
			//		  case 0x00636206:
			//			if(attribute_id==2)
			//			{
			//				tamper_compart(16,0x1900,0x1a00);
			//			}
			//			if(attribute_id == 3)
			//			{
			//				capture_objects_filler(control_event_capture_obj);//tamper_capture_objects();
			//			}
			//			break;
			//		  case 0x015e5b00:
			//			if(attribute_id == 3)
			//				capture_objects_filler(instantaneous_parameter_cap_obj);//inst_para_cap_obj(0);
			//			break;
			//		  case 0x015e5b03:
			//			if(attribute_id == 2)
			//				buffer_scaler_filler(instantaneous_parameter_scaler_buffer);//inst_para_buffer_s();
			//			if(attribute_id==3)
			//				capture_objects_filler(instantaneous_parameter_scaler_cap_obj);// inst_para_cap_obj('s');
			//			break;
			//		  case 0x015e5b04:
			//			if(attribute_id==3)
			//				capture_objects_filler(blockload_survey_parameter_scaler_cap_obj);//load_survey_cap_obj('s');
			//			break;
			//		  case 0x015e5b05:
			//			if(attribute_id==3)
			//				capture_objects_filler(dailyload_profile_parameter_scaler_cap_obj);//load_survey_cap_obj('s');
			//			break;
			//
			//		  case 0x015e5b06:
			//			if(attribute_id==3)
			//				capture_objects_filler(bill_profile_parameter_scaler_cap_obj);//bill_cap_obj_s();
			//			if(attribute_id==2)
			//				buffer_scaler_filler(bill_profile_parameter_scaler_buffer);
			//			break;
			//		  case 0x015e5b07:
			//			if(attribute_id == 3)
			//				capture_objects_filler(event_log_profile_scaler_cap_obj);//event_log_profile_cap_obj_s();
			//			break;
			//			// 1,0,99,1,0,255,
			//		  case 0x01630100:
			//			if(attribute_id==2)
			//			{
			//
			//				Sel_Loadsurvey_buffer();
			//			}
			//
			//			break;
			//		  case 0x01630200:
			//			if(attribute_id==2)
			//			{
			//				Sel_DailyLoadsurvey_buffer();
			//			}
			//			break;
			//		  default:
			if (seg_flagsd==0)
			{//elserr
				if(nrm_flag==1)
				{//ifnrm
					fill_A0(0x11);
				}//ifnrm
				else
				{//elnrm
					fill_A0(0x97);
				}//ielnrm
			}//elserr
			//			break;
			//		}
		}
	}
}//req_type





void fill_0d(void)/********/
{
	info[k++]=0x01;//data result
	info[k++]=0x0D;//object Inaccessible
}

void enum_d2(unsigned char len)
{
	info[k++]=0x00;
	info[k++]=0x16;
	info[k++]=len;
	//130807 info_len=k;
}
void fill_0b(void)/********/
{
	info[k++]=0x01;//data result
	info[k++]=0x0B;//object Unavaliable
}

unsigned char data_dec(void)
{//data_dec
	unsigned char i_data=0x00;
	u8 UcharTemp=0x00;
	unsigned int uint_temp=0x00;
	unsigned int  di;
	//    u16 diff1;
	//    u8 addpg;
	//    u8 u8tem_mm;
	//    u8 u8tem_dd;
	//    u8 u8tem_h;
	//    u8 u8tem_m;
	//	u8 u8tem_dow;
	//    u8 u8tem_s;
	//    u8 count;
	//    u8 noweek;
	//    u16 u16tem_yy;

	switch(info[3])
	{//swda
	case 0x60:       //AARQ-APDU
		ass_ser=0xA1;
		if(info[4]!=byte_cont-4)  ////check length of informatin
		{//ifasslen
			asserr_flag=1;
			return 0;
		}//ifasslen

		k=5;
		//protocol version
		if((info[5]==0xA0)||(info[5]==0x80))
		{//if protocol version
			update_k();
			if((info[k-1]!=0x01)&&(info[k-1]!=0x84))//prtocol version default 22/02/2007
			{//ifapplen
				asserr_flag=2;
				ass_ser=0xA2;
				return 0;
			}//ifapplen
		}//if protocol version
		//protocol version

		//application context name
		if(info[k]==0xA1)
		{//ifapp_context_name
			k++;
			if(info[k]!=0x09 || info[k+1]!=0x06 || info[k+2]!=0x07)
			{//ifapplen
				asserr_flag=1;
				return 0;
			}//ifapplen
			else if(info[k+3]!=0x60 || info[k+4]!=0x85 || info[k+5]!=0x74 || info[k+6]!=0x05 || info[k+7]!=0x08 || info[k+8]!=0x01 || info[k+9]!=0x01)
			{//ifobjname[0]
				asserr_flag=2;
				return 0;
			}//ifobjname[0]
			k=k+10;

		}//ifapp_context_name
		//application context name

		//called -AP-title
		if(info[k]==0xA2)				//check with mr. dinesh is it can be else if, if yes then can be use switch
		{//if called -AP-title
			update_k();
		}//if called -AP-title
		//called -AP-title

		//called -AE-Qualifier
		if(info[k]==0xA3)
		{//if called -AE-Qualifier
			update_k();
		}//if called -AE-Qualifier
		//called -AE-Qualifier

		//called-AP-invocation id
		if(info[k]==0xA4)
		{//if called -AP-invocation id
			update_k();
		}//if called -AP-invocation id
		//called -AP-invocation id

		//called-AE-invocation id
		if(info[k]==0xA5)
		{//if called -AE-invocation id
			update_k();
		}//if called -AE-invocation id
		//called -AE-invocation id


		//calling -AP-title
		if(info[k]==0xA6)
		{//if calling -AP-title
			update_k();
		}//if calling -AP-title
		//calling -AP-title

		//calling -AE-Qualifier
		if(info[k]==0xA7)
		{//if calling -AE-Qualifier
			update_k();
		}//if calling -AE-Qualifier
		//calling -AE-Qualifier


		//calling-AP-invocation id
		if(info[k]==0xA8)
		{//if calling -AP-invocation id
			update_k();
		}//if calling -AP-invocation id
		//calling -AP-invocation id


		//calling-AE-invocation id
		if(info[k]==0xA9)
		{//if calling -AE-invocation id
			update_k();
		}//if calling -AE-invocation id
		//calling -AE-invocation id

		//		if(one_byte_add_f!=1)
		//		{
		//			for(di=0;di<4;di++)
		//				server_add[di]=rec[di+3];
		//			client_add=rec[7];
		//		}
		//		else
		//		{
		//			server_add[3]=rec[3];//server_lowerlow=rec[3];
		//			client_add=rec[4];
		//		}
		//
		//		//if(server_upperlow==0x20)
		if((client_add==0x41)||(client_add==0x61)||(client_add==0x81))
		{//iflen
			if(info[k]!=0x8A)
			{//iflow
				asserr_flag=0x0C;
				return 0;
			}//iflow
		}//iflen
		four_pass_f=0;

		//sender ACSE requirement field
		if(info[k]==0x8A)
		{//ifacse
			k++;
            if(info[k]!=0x02 || info[k+1]!=0x07 || info[k+2]!=0x80)
            {//ifobjname[0]
              asserr_flag=1;
              return 0;
            }
			if(info[k]!=0x02 || info[k+1]!=0x07 || info[k+2]!=0x80 || info[k+3]!=0x8B || info[k+4]!=0x07 || info[k+5]!=0x60 || info[k+6]!=0x85 || info[k+7]!=0x74 || info[k+8]!=0x05 || info[k+9]!=0x08 || info[k+10]!=0x02 )//|| ((info[k+11]!=0x01)&&(info[k+11]!=0x02)) || info[k+12]!=0xAC  || info[k+13]!=0x0A || info[k+14]!=0x80 || info[k+15]!=0x08)
			{//iflen
				asserr_flag=0x0B;
				return 0;
			}//iflen
			if(asso2_flag==1)
			{
				if(info[k+11]!=0x02)//low level 1-high level 2
				{//iflen
					asserr_flag=0x0B;
					return 0;
				}//iflen
				if(info[k+12]!=0xAC  || info[k+13]!=0x12 || info[k+14]!=0x80 || info[k+15]!=0x10)
				{//iflen
					asserr_flag=0x0E;
					return 0;
				}//iflen
				memcpy(aut_pswd1_1,&info[k+16],16);
				four_pass_f=1;
			}
			else
			{
				if(info[k+11]!=0x01)
				{//iflen
					asserr_flag=0x0B;
					return 0;
				}//iflen
				if(info[k+12]!=0xAC  || info[k+14]!=0x80 )
				{//iflen
					asserr_flag=0x0E;
					return 0;
				}//iflen
			}
			if(asso1_flag==1)
			{
				if(info[k+13]!=0x0A  || info[k+15]!=0x08 ||info[k+16]!=aut_pswd[0] || info[k+17]!=aut_pswd[1] || info[k+18]!=aut_pswd[2] || info[k+19]!=aut_pswd[3] || info[k+20]!=aut_pswd[4] || info[k+21]!=aut_pswd[5] || info[k+22]!=aut_pswd[6] || info[k+23]!=aut_pswd[7])
				{//ifaut_len
					asserr_flag=0x0D;
					return 0;
				}//ifaut_len
			}
			else if(asso3_flag==1)
			{
				if(info[k+13]!=0x0A  || info[k+15]!=0x08 ||info[k+16]!=aut_pswd2[0] || info[k+17]!=aut_pswd2[1] || info[k+18]!=aut_pswd2[2] || info[k+19]!=aut_pswd2[3] || info[k+20]!=aut_pswd2[4] || info[k+21]!=aut_pswd2[5] || info[k+22]!=aut_pswd2[6] || info[k+23]!=aut_pswd2[7])
				{//ifaut_len
					asserr_flag=0x0D;
					return 0;
				}
			}
            else if(asso0_flag == 1)
			{
              if(info[k+12]==0xAC  || info[k+14]==0x80)
				{	//ifaut_len
                asserr_flag=0x01;
					return 0;
				}
			}
			k=k+24;
			if(four_pass_f==1)
			k=k+8;
		}//ifacse
		//sender ACSE requirement field

		//implementation field
		if(info[k]==0xBD)
		{//if implementation field
			update_k();
			asserr_flag=0x01;
			return 0;
		}//if implementation field
		//implementation field
		//conf_ser_flag, conf_err_flag,conf_type_flag,conf_serror_flag;
		//user-information+xDLMS initiate.request PDU
		if(info[k]==0xBE)
		{//ifuser infomation tag
			k++;
			if((info[k++]!=0x10)&&(info[k-1]!=0x11))
			{//iflen
				conf_ser(0x00);
				return 0;
			}//iflen
			else if(info[k]!=0x04 || ((info[k+1]!=0x0E)&&(info[k+1]!=0x0F)) || info[k+2]!=0x01 || info[k+3]!=0x00 || info[k+4]!=0x00 || ((info[k+5]!=0x00)&&(info[k+5]!=0x01)))
			{//ifchoice
				asserr_flag=1;
				return 0;
			}//ifchoice
			info_l6();
			if((info[k-1]==0x01))
			{//ifinit.req[3]
				k++;
			}//ifinit.req[3]

			if(info[k++]<0x06)
			{//ifinit.req[4]
				conf_ser(0x01);		// Proposed dlms version is too low
				return 0;
			}//ifinit.req[4]
			else if(info[k++]!=0x5F)
			{//ifinit.req[5]
				asserr_flag=1;
				return 0;
			}//ifinit.req[5]
			else if((info[k]!=0x1F)&&(info[k]!=0x04))
			{//ifinit.req[6]
				asserr_flag=1;
				return 0;
			}//ifinit.req[6]
			else if(info[k]==0x1F || info[k]==0x04)
			{//if1f
				k++;
				if(info[k-1]!=0x04)
				{
					if(info[k++]!=0x04)
					{//ifinit.req[7]
						asserr_flag=1;
						return 0;
					}//ifinit.req[7]
				}
				if(info[k++]!=0x00)
				{//ifinit.req[8]
					asserr_flag=1;
					return 0;
				}//ifinit.req[8]
				//				if(info[k] == 0x10 && info[k+1] == 0x00 && info[k+2] == 0x10)
				//				{
				//					conf_blk[0]=0x00;
				//					conf_blk[1]=0x00;
				//					conf_blk[2]=0x10;
				//					k+=3;
				//				}
				//				else
				//				{
				//					if((info[k++])&(conf_blk[0])!=conf_blk[0])
				//					{//ifconf_blk[0]
				//						conf_ser(0x02);
				//						return 0;
				//					}//ifconf_blk[0]
				//					else if(((info[k++])&(conf_blk[1]))!=conf_blk[1])
				//					{//ifconf_blk[1]
				//						conf_ser(0x02);
				//						return 0;
				//					}//ifconf_blk[1]
				//					else if(((info[k++])&(conf_blk[2]))!=conf_blk[2])
				//					{//ifconf_blk[2]
				//						conf_ser(0x02);
				//						return 0;
				//					}//ifconf_blk[2]
				//				}
				if(((info[k]&conf_blk[0])==0x00)&&((info[k+1]&conf_blk[1])==0x00)&&((info[k+2]&conf_blk[2])==0x00))
				{//ifconf_blk[2]
					conf_ser(0x02);				// incompatible-conformance
					return 0;
				}//ifconf_blk[2]
				else
				{
					conf_blk[0]=(info[k])&conf_blk[0];
					conf_blk[1]=(info[k+1])&conf_blk[1];
					conf_blk[2]=(info[k+2])&conf_blk[2];
					k=k+3;
				}
				if(((unsigned int)(info[k]*256+info[k+1]))<12)//(unsigned int) Updated by Dinesh
				{//max PDU size
					conf_ser(0x03);			// max pdu size is too short
					return 0;
				}//max PDU size
				k=k+2;
			}//if1f
		}//ifuser information tag
		//user-information+xDLMS initiate.request PDU
		break;
	case 0xC0:     //get
		invo_prio=info[5];//invoke_id,priority
		if((info[4]!=0x01)&&(info[4]!=0x02)&&(info[4]!=0x03))
		{//ifasslen
			conf_err(0x0C);
			return 0;
		}//ifasslen
		else if(byte_cont<15)//byte_cont
		{//elifbyte_cont
			conf_err(0x0B);
			return 0;
		}//elifbyte_cont
		invo_prio=info[5];//invoke_id,priority
		class_id=(unsigned int)info[6]*256+info[7];		//info[k++]*256+info[k++];
		//for(i_data=0;i_data<6;i_data++)
		// obis_code[i_data]=info[i_data+8];
		memcpy(obis_code,&info[8],6);
		attribute_id=info[14];
		chngtemp.u8_temp[0]=obis_code[4];
		chngtemp.u8_temp[1]=obis_code[3];
		chngtemp.u8_temp[2]=obis_code[2];
		chngtemp.u8_temp[3]=obis_code[0];
		obis_short = chngtemp.u32_temp;
		////		asm("mov.b &obis_code,&obis_short+3");
		////		asm("mov.b &obis_code+2,&obis_short+2");
		////		asm("mov.b &obis_code+3,&obis_short+1");
		////		asm("mov.b &obis_code+4,&obis_short");
		//selective access
		if(info[15]==1)
		{
			sel_access_flag=1;
			access_selector=info[16];
			if(access_selector==1) // by range
			{
				from_ptr=0;
				if(((obis_code[0]==0x01)&&(obis_code[2]== 0x63)&&(obis_code[4]== 0x00)) && ((obis_code[3]==0x01)||(obis_code[3]== 0x02)))
				{
					if (info[23]==8) // class id, for clock
					{
						uint_temp = (unsigned int)info[39]*256;
						uint_temp = uint_temp + info[40] - 0x07D0;
						UcharTemp = (unsigned char)(uint_temp % 256);
						from_val[0] = hex_to_bcd(UcharTemp);//yr
						for(i_data=1,di=41;(i_data<5 && di<46);i_data++,di++)
						{
							if(di == 43)
							di++;
							from_val[i_data]=hex_to_bcd(info[di]);
						}

						uint_temp = (unsigned int)info[53]*256;
						uint_temp = uint_temp + info[54] - 0x07D0;
						UcharTemp = (unsigned char)(uint_temp % 256);
						to_val[0] = hex_to_bcd(UcharTemp);//yr
						for(i_data=1,di=55;(i_data<5 && di<60);i_data++,di++)
						{
							if(di == 57)
							di++;
							to_val[i_data]=hex_to_bcd(info[di]);
						}
						if((obis_code[0]==0x01)&&(obis_code[1]==0x00)&&(obis_code[2]==0x63)&& (obis_code[3]==0x02))
						{
							if((from_val[3]==0)&&(from_val[4]==0))
							{
								Correct_DE_entry_f	= 0;
							}
							else
							{
								Correct_DE_entry_f	= 1;
							}
							
							for(i_data=3;i_data<5;i_data++)
							{
								from_val[i_data]=0;
								to_val[i_data]=0;
							}

						}
						if(info[66]==0) // all objects
						{
							unsigned char i1;
							for(i1=0;i1<10;i1++)
							{
								sel_obj[i1]=1;
							}
							no_obj=load_survey_parameter_cap_obj[0];

	#if DailyEnergy == 2
							if((obis_code[0]==0x01)&&(obis_code[1]==0x00)&&(obis_code[2]==0x63)&&obis_code[3]==0x02)
							no_obj=dailyload_profile_parameter_cap_obj[0];
	#endif                        
							k=66;
						}
						else
						{
							if((obis_code[0]==0x01)&&(obis_code[1]==0x00)&&(obis_code[2]==0x63)&&(obis_code[3]==0x01))
							{
								unsigned char i_1;
								for(i_1=0;i_1<info[66];i_1++) //looping for array elements(objects)
								{
									if((info[74+i_1*18]==0) && (info[75+i_1*18]==0) && (info[76+i_1*18]==1) && (info[77+i_1*18]==0) && (info[78+i_1*18]==0))
									{
										sel_obj[0]=1;
									}
									else if((info[74+i_1*18]==1) && (info[75+i_1*18]==0) && (info[76+i_1*18]==11) && (info[77+i_1*18]==27) && (info[78+i_1*18]==0))
									sel_obj[1]=1;

									else if((info[74+i_1*18]==1) && (info[75+i_1*18]==0) && (info[76+i_1*18]==12) && (info[77+i_1*18]==27) && (info[78+i_1*18]==0))
									sel_obj[2]=1;

									else if((info[74+i_1*18]==1) && (info[75+i_1*18]==0) && (info[76+i_1*18]==1) && (info[77+i_1*18]==29) && (info[78+i_1*18]==0))
									sel_obj[3]=1;

									else if((info[74+i_1*18]==1) && (info[75+i_1*18]==0) && (info[76+i_1*18]==9) && (info[77+i_1*18]==29) && (info[78+i_1*18]==0))
									sel_obj[4]=1;

									else if((info[74+i_1*18]==0) && (info[75+i_1*18]==0) && (info[76+i_1*18]==96) && (info[77+i_1*18]==3) && (info[78+i_1*18]==10))
									sel_obj[5]=1;

									else if((info[76+i_1*18]==0) && (info[77+i_1*18]==0) && (info[77+i_1*18]==96) && (info[77+i_1*18]==12) && (info[77+i_1*18]==5))
									sel_obj[6]=1;

								}
							}
							else if((obis_code[0]==0x01)&&(obis_code[1]==0x00)&&(obis_code[2]==0x63)&&(obis_code[3]==0x02))
							{
								unsigned char i_1;
								for(i_1=0;i_1<info[66];i_1++) //looping for array elements(objects)
								{
									if((info[74+i_1*18]==0) && (info[75+i_1*18]==0) && (info[76+i_1*18]==1) && (info[77+i_1*18]==0) && (info[78+i_1*18]==0))
									{
										sel_obj[0]=1;
									}
									else if((info[74+i_1*18]==1) && (info[75+i_1*18]==0) && (info[76+i_1*18]==1) && (info[77+i_1*18]==8) && (info[78+i_1*18]==0))
									sel_obj[1]=1;

									else if((info[74+i_1*18]==1) && (info[75+i_1*18]==0) && (info[76+i_1*18]==9) && (info[77+i_1*18]==8) && (info[78+i_1*18]==0))
									sel_obj[2]=1;

									else if((info[74+i_1*18]==1) && (info[75+i_1*18]==0) && (info[76+i_1*18]==1) && (info[77+i_1*18]==6) && (info[78+i_1*18]==0))
									sel_obj[3]=1;

									else if((info[74+i_1*18]==0) && (info[75+i_1*18]==0) && (info[76+i_1*18]==96) && (info[77+i_1*18]==51) && (info[78+i_1*18]==0))
									sel_obj[4]=1;

									else if((info[74+i_1*18]==0) && (info[75+i_1*18]==0) && (info[76+i_1*18]==96) && (info[77+i_1*18]==51) && (info[78+i_1*18]==1))
									sel_obj[5]=1;

									//else if((info[74+i_1*18]==0) && (info[75+i_1*18]==0) && (info[76+i_1*18]==96) && (info[77+i_1*18]==51) && (info[78+i_1*18]==2))
									//sel_obj[6]=1;

									else if((info[74+i_1*18]==0) && (info[75+i_1*18]==0) && (info[76+i_1*18]==96) && (info[77+i_1*18]==50) && (info[78+i_1*18]==0))
									sel_obj[6]=1;

									else if((info[76+i_1*18]==0) && (info[77+i_1*18]==0) && (info[77+i_1*18]==96) && (info[77+i_1*18]==50) && (info[77+i_1*18]==1))
									sel_obj[7]=1;

									else if((info[76+i_1*18]==0) && (info[77+i_1*18]==0) && (info[77+i_1*18]==96) && (info[77+i_1*18]==50) && (info[77+i_1*18]==2))
									sel_obj[8]=1;
								}
							}
							no_obj=info[66];
							if((obis_code[0]==0x01)&&(obis_code[1]==0x00)&&(obis_code[2]==0x63) && (obis_code[3]==0x01))
							{
								if(no_obj>load_survey_parameter_cap_obj[0])
								no_obj=load_survey_parameter_cap_obj[0];
								info[66]=load_survey_parameter_cap_obj[0];
							}
	#if DailyEnergy == 2                        
							else if((obis_code[0]==0x01)&&(obis_code[1]==0x00)&&(obis_code[2]==0x63) && (obis_code[3]==0x02))
							{
								if(no_obj>dailyload_profile_parameter_cap_obj[0])
								no_obj=dailyload_profile_parameter_cap_obj[0];
								info[66]=dailyload_profile_parameter_cap_obj[0];
							}
	#endif                        
							k = 66+ info[66]*18;
						}
					}
				}
				else
				{
					sel_access_flag=0;
					Selaccess_err_f = 1 ;
				}
			}
			else if(access_selector==2) // by entry
			{
				unsigned char i_a, from_ob,to_ob;

				from_ptr=info[23];
				to_ptr=info[28];
				//				if(from_ptr==0) from_ptr=1;
				from_ob=info[31];
				to_ob=info[34];
				if(from_ob==0 || from_ob >7) from_ob=1;
				
				if(((obis_code[2]==0x63)&&(obis_code[3]== 0x62)&& (obis_code[4]!= 0x05)&&(obis_code[0]==0x00)) || ((obis_code[2]==0x62)&&(obis_code[3]== 0x01)&& (obis_code[4]== 0x00)&&(obis_code[0]==0x01)))
				{
					if(obis_code[2]==0x63)
					{
						if((obis_code[4] == 0x00)||(obis_code[4] == 0x04)||(obis_code[4] == 0x01))
						{
							if(to_ob==0 || to_ob > 7)
							to_ob=7;
						}
						else
						{
							if(to_ob==0 || to_ob >2)
							to_ob=2;
						}
					}
					no_obj= to_ob-from_ob+1;
					for(i_a=0;i_a<7;i_a++)
					{
						sel_obj_tamper[i_a]=0;
					}
					for(i_a=from_ob-1;i_a<to_ob;i_a++)
					{
						if(i_a<7)
						{
							sel_obj_tamper[i_a]=1;
						}
					}
				}
				else
				{
					sel_access_flag=0;
					Selaccess_err_f = 1 ;
				}
			}
		}
		else // no selective access
		{
			sel_access_flag=0;
			k=15;
		}
		for(i_data=0;i_data<6;i_data++)
		{
			if(obis_code[i_data]!=0xff)
			break;
		}
		if(i_data==0x06)
		{
			conf_err(0x04);
			return 0;
		}
		
		if(Selaccess_err_f == 1)
		{
			Selaccess_err_f=0;
			conf_err(0x0d);
			return 0;
		}
		
		break;
	case 0xc1:  //for setting parameters
		invo_prio=info[5];//invoke_id,priority
		if(info[4]>0x03 || info[4]==0x00)
		{//ifasslen
			conf_err(0x0C);
			return 0;
		}//ifasslen
		else if((byte_cont<15)&&(info[4]!=0x03))//byte_cont
		{//elifbyte_cont
			conf_err(0x0B);
			return 0;
		}//elifbyte_cont
		if(info[4] !=0x03)
		{
			class_id=(unsigned int)info[6]*256+info[7];
			attribute_id=info[14];
			memcpy(obis_code,&info[8],6);
			chngtemp.u8_temp[0]=obis_code[4];
			chngtemp.u8_temp[1]=obis_code[3];
			chngtemp.u8_temp[2]=obis_code[2];
			chngtemp.u8_temp[3]=obis_code[0];
			obis_short = chngtemp.u32_temp;
			//			asm("mov.b &obis_code,&obis_short+3");
			//			asm("mov.b &obis_code+2,&obis_short+2");
			//			asm("mov.b &obis_code+3,&obis_short+1");
			//			asm("mov.b &obis_code+4,&obis_short");
		}
		k=16;
		for(i_data=0;i_data<6;i_data++)
		{
			if(obis_code[i_data]!=0xff)
			break;
		}
		if(i_data==0x06)
		{
			conf_err(0x04);
			return 0;
		}
		if((asso0_flag == 1))//||(asso1_flag == 1))
		{//if9
			conf_err(0x0D);
			return 0;
		}//if9
		if((1==asso2_flag)||(1==asso3_flag))
		{
                  return(set_resp());
                }
                else
                {
                  conf_err(0x0D);
                  return(0);
		}
		//return set_resp();
		//break;

	case 0xC3://action
		invo_prio=info[5];//invoke_id,priority

		if(info[4]> 0x03 || info[4]==0x00)//
		{
			conf_err(0x0C);
			return 0;
		}
		else if(byte_cont < 15)//byte_cont
		{
			conf_err(0x0B);
			return 0;
		}

		if(info[4] !=0x03)
		{
			class_id=(unsigned int)info[6]*256+info[7];
			//for(i_data=0;i_data<6;i_data++)
			//	obis_code[i_data]=info[i_data+8];

			memcpy(obis_code,&info[8],6);
			attribute_id=info[14];
			chngtemp.u8_temp[0]=obis_code[4];
			chngtemp.u8_temp[1]=obis_code[3];
			chngtemp.u8_temp[2]=obis_code[2];
			chngtemp.u8_temp[3]=obis_code[0];
			obis_short = chngtemp.u32_temp;
			//			asm("mov.b &obis_code,&obis_short+3");
			//			asm("mov.b &obis_code+2,&obis_short+2");
			//			asm("mov.b &obis_code+3,&obis_short+1");
			//			asm("mov.b &obis_code+4,&obis_short");
		}

		k=16;

		for(i_data=0;i_data<6;i_data++)
		{
			if(obis_code[i_data]!=0xff)
			break;
		}

		if(i_data == 0x06)
		{
			conf_err(0x04);
			return 0;
		}

		if((asso0_flag != 0)||(asso1_flag != 0))
		{
			conf_err(0x0D);
			return 0;
		}

		if(cosem_flag==1)
		{
			return action_resp();
		}
		if(four_pass_f==1)
		{
			if((obis_code[0]==0)&&(obis_code[1]==0)&&(obis_code[2]==40)
					&&(obis_code[3]==0)&&(obis_code[4]==0)&&(obis_code[5]==255))
			{
				if(attribute_id==1)
				{

					//for(i_data=0;i_data<16;i_data++)
					//aut_pswd12[i_data]=aut_pswd1[i_data];
					memcpy(aut_pswd12,aut_pswd1,16);

					//					KeyExpansion(aut_pswd12);

					//for(i_data=0;i_data<16;i_data++)
					//aut_pswd1_2[i_data]=aut_pswd1_23[i_data];
					memcpy(aut_pswd1_2,aut_pswd1_23,16);

					//					Cipher(aut_pswd1_2);
					aes_encrypt(aut_pswd1_2,aut_pswd12);
					if(info[16]!=0x09&&info[17]!=0x10)
					{
						conf_err(0x0C);
						return 0;
					}
					for(i_data=0;i_data<16;i_data++)
					if(aut_pswd1_2[i_data]!=info[i_data+18])
					{
						four_pass_f=0;
						return 0;
						//four_pass_f=0;
					}
					cosem_flag = 1;
				}
			}
		}
		break;
	}//swda
	return 1;
}//data_dec
u8 tou_pssv_store(void)
{
	u8 u8index;

    if((info[4]==0x01)&&(info[16]==0x01)&&(info[17]<=0x02)&&(info[18]==0x02)&&(info[19]==0x02)&&(info[20]==0x11)&&(info[21]<=0x02)&&(info[22]==0x01)&&(info[23]<=Tarriff_slots))
	{
		
		tou_u8pssv_dayid=info[21];
		tou_u8pssv_no_zone=info[23];
		tou_u8pssv_no_days=info[17];
		//		tou_u8pssv_buffer_traced=info[21];
		u8index=24;
		last_block = 1;
		//		tou_u8pssv_buffer_traced -= 8;
		tou_u8pssv_ptr=0;
		tou_u8pssv_day=0;
		tou_u8pssv_up_zone=0;
		memzero(tou_a8pssv_zone_time,16);
		memzero(tou_a8pssv_traiff,8);
		
		return save_tou_pass_data(u8index, info, 0);

		
	}
	else 
	if((info[4]==0x02)&&(info[20]==0x01)&&(info[22]==0x01)&&(info[23]<=0x02)
            &&(info[24]==0x02)&&(info[25]==0x02)&&(info[26]==0x11)&&(info[28]==0x01)&&(info[29]<=Tarriff_slots))//mri
	{
		block_no=info[20];
		last_block=info[16];
		tou_u8pssv_dayid=info[27];
		tou_u8pssv_no_zone=info[29];
		tou_u8pssv_no_days=info[23];
		tou_u8pssv_buffer_traced=info[21];
		u8index=30;
		tou_u8pssv_buffer_traced -= 8;
		tou_u8pssv_ptr=0;
		tou_u8pssv_day=0;
		tou_u8pssv_up_zone=0;
		memzero(tou_a8pssv_zone_time,16);
		memzero(tou_a8pssv_traiff,8);
		memcpy(info2,info, (info[21] + 22));
		
		return save_tou_pass_data(u8index, info2, 1);
	}
	else if((info[4]==0x03)&&(info[10]>0x01))
	{
		block_no=info[10];
		last_block=info[6];
		memcpy(info2+tou_u8pssv_buffer_traced,&info[12],info[11]);
		tou_u8pssv_buffer_traced+=info[11];
		u8index=0;
		
		return save_tou_pass_data(u8index, info2, 1);
	}
	else
	{
		return 0;
	}



}

void tamper_status(void)
{
	u8 tamper_byte11,tamper_byte12,tamper_byte13,tamper_byte14;//,ab=0;//,tamper_byte4;
	
	    tamper_byte11=0x00;
	    if(TP3.b.neu_miss_tpr_f == 1) //neutral miss     neu_tpr_f==1
	        tamper_byte11+=0x01;
		if(TP7.b.high_v_f==1) //Over Voltage
			tamper_byte11+=0x02;     
	    if(TP7.b.LowVolt_tpr_f==1) //Low Voltage
	        tamper_byte11+=0x04;
	   // if(((rev_tpr_sign == 1) || (rev_el_tpr_sign == 1)))    //Reverse Current   (rev_tpr_f==1)
		if(TP1.b.rev_tpr_f==1)
	        tamper_byte11+=0x08;
	    if(TP2.b.el_tpr_f==1)
	        tamper_byte11+=0x10;
	    if(TP2.b.oc_tpr_f==1) //Over Current
	        tamper_byte11+=0x20;
	    if(TP1.b.ol_tpr_f==1) //Over Load
	        tamper_byte11+=0x40;
		if(TP1.b.mag_tpr_f==1) //||(TP1.b.mag_active_f==1))    //Magnet Tamper
	        tamper_byte11+=0x80;

	    tamper_byte12=0x00;
	    if(TP3.b.neu_dis_tpr_f==1)    //Neutral Disturbance    neu_dis_tpr_f==1
	        tamper_byte12+=0x01;
	    if(TP5.b.tc_tpr_f==1)     //TopCover Open
	        tamper_byte12+=0x02;
		if(TP4.b.AbFreq_tpr_f==1)     //TopCover Open
	        tamper_byte12+=0x04;
	    if(rtc_status_byte==1) //RTC Fail
	        tamper_byte12+=0x08;
	    if(disable_bkup_f==1)//Primary Low Battery
	        tamper_byte12+=0x10;
	    if(acknotr_f0==1)       //Memory Fail
	        tamper_byte12+=0x20;
	    if(TP5.b.kv35_tpr_f==1)        //highvoltage
	        tamper_byte12+=0x40;
//	    if(TP8.b.AbnrmVolt_tpr_f==1)  //reserve for current mismatch
//	        tamper_byte12+=0x80;
			
	    tamper_byte13=0x00;
//	    if(TP8.b.AbnrmVolt_tpr_f==1)  //abnormal voltage to be confirm from s/w cel
//	        tamper_byte13+=0x01;			
//	    if(TP6.b.HighTemprature_tpr_f==1) //high temperature to be confirm from s/w cel
//	        tamper_byte13+=0x02;
//	    if(TP4.b.LowPF_tpr_f==1) //Low PF to be confirm from s/w cel
//	        tamper_byte13+=0x04;
	//        tamper_byte13+=0x08;
	//    
	//    if(ab==1)     // 1 Negative and 0 Positive
	//        tamper_byte13+=0x10;
	//    if(ab==1)
	//        tamper_byte13+=0x20;
	//    if(ab==1)
	//        tamper_byte13+=0x40;
	//    if(ab==1)
	//        tamper_byte13+=0x80; 
	tamper_byte14=0x00;
    if(RTC_BATT_discharge_f==1)//Low RTC Battery
    	tamper_byte14+=0x01;
	
	info[k++]=0x00;
	info[k++]=0x04;
	info[k++]=32;
	info[k++]=tamper_byte14;
	info[k++]=tamper_byte13;
	info[k++]=tamper_byte12;
	info[k++]=tamper_byte11;
	
	//val_2byt2(tamper_byte1,tamper_byte2);//,tampre_byte3,tamper_byte4);
}
void fill_info(unsigned char const *f_d)
{
	unsigned char di;
	for(di=1;di<=f_d[0];di++)
	{
		info[k++]=f_d[di];
	}
}
//void zero_flash_opr_data(void)
//{
//	u16 ifod;
//	for(ifod=0;ifod<300;ifod++)
//		flash_opr_data[ifod]=0;
//}


void sr_no_ascii(void)
{
	u8 dumy_p;

	info[k]=0x0a;
	Eprom_Read(SerialNo_UtilityID_add);
	if(EPROM_bChksum_ok==0)
	{
		Eprom_Read(Dupli_SerialNo_UtilityID_add);
		if(EPROM_bChksum_ok==1)
		Eprom_Write(SerialNo_UtilityID_add);
	}
	dumy_p=k+2;
	info[dumy_p++]=opr_data[10];
	info[dumy_p++]=opr_data[11];
	info[dumy_p++]=opr_data[12];
	info[dumy_p++]=opr_data[13];
	info[dumy_p++]=opr_data[14];

	if(opr_data[0]!=0x20)
	info[dumy_p++]=opr_data[0];
	if(opr_data[1]!=0x20)
	info[dumy_p++]=opr_data[1];
	if(opr_data[2]!=0x20)
	info[dumy_p++]=opr_data[2];
	if(opr_data[3]!=0x20)
	info[dumy_p++]=opr_data[3];
	if(opr_data[4]!=0x20)
	info[dumy_p++]=opr_data[4];
	if(opr_data[5]!=0x20)
	info[dumy_p++]=opr_data[5];
	if(opr_data[6]!=0x20)
	info[dumy_p++]=opr_data[6];
	if(opr_data[7]!=0x20)
	info[dumy_p++]=opr_data[7];

	info[k+1]=dumy_p-2-k;
	k=dumy_p;
}

void get_resp1(void)
{
	unsigned int classatt;
	unsigned long int obis1;
	unsigned char cl_ob[4];
	unsigned char class_att[2];
	static union
	{
		u8 a8Temp[2];
		u16 u16Temp;
	}unionTemp;
	//unsigned char i_g;
	class_att[0]=info[7];
	class_att[1]=attribute_id;
	//	if(attribute_id == 1)
	//		class_att[0]=0;
	classatt=a8_to_u16(&class_att[0]);
	cl_ob[0]=info[8];
	cl_ob[1]=info[10];
	cl_ob[2]=info[11];
	cl_ob[3]=info[12];
	obis1=a8_to_u32(&cl_ob[0]); 
	switch(classatt)
	{
	case 0x0801:
		if(obis1== 0x00010000)
		log_name2(0,0,1,0,0);
		else
		fill_0b();
		break;
	case 0x0802:
		if(obis1== 0x00010000)
		date_time(dt.day,dt.month,dt.year,dt.hour,dt.min,dt.sec,0x03);
		else
		fill_0b();
		break;
		
	case 0x0803:
		if(obis1== 0x00010000)
		integer16(0x01,0x4a);
		else
		fill_0b();
		break;
		
	case 0x0804:
		if(obis1== 0x00010000)
		unsigned8(rtc_status_byte,1);
		else
		fill_0b();
		break;            
	case 0x0805:            
	case 0x0806:
	case 0x0807:
	case 0x0808:                    
		if(obis1== 0x00010000)
		fill_0d();
		else
		fill_0b();
		break;
	case 0x0809:
		if(obis1== 0x00010000)
		enum_d2(0x01);
		else
		fill_0b();
		break;
		
	case 0x0f01:   //logical_name				
		if(obis1 == 0x00280000)
		log_name2(0,0,40,0,0);
		else if(obis1 == 0x00280001)
		log_name2(0,0,40,0,1);
		else
		fill_0b();
		break;
	case 0x0f02://object_list
		if((obis1 == 0x00280000)||(obis1 == 0x00280001))
		{
			i_dlms=1;
			object_list();
		}
		else
		fill_0b();
		break;
	case 0x0f03://partners_id
		if((obis1 == 0x00280000)||(obis1 == 0x00280001))
		{
			info[k++]=0;
			structure(2);
			if(asso3_flag==1)
			integer8(0x40);  //info[k++]=0x40;
			else if(asso2_flag==1)
			integer8(0x30);  //  info[k++]=0x30;
			else if(asso1_flag==1)
			integer8(0x20);  //  info[k++]=0x20;
			else
			integer8(0x10);  //  info[k++]=0x10;
			
			val_2byt(0x00,0x01);
		}
		else
		fill_0b();
		break;
	case 0x0f04://application_context_name
		if((obis1 == 0x00280000)||(obis1 == 0x00280001))
		{
			fill_info(auth_fill);
			fill_info(app_con);
		}
		else
		fill_0b();
		break;
	case 0x0f05://xDLMS_context_info
		switch(obis_short)
		{	
		  case 0x00280000:
            fill_info(XDLMS_TYPE_CONST);
            info[k-15]=conf_blk[0]; //0x00;//conformance[0]
            info[k-14]=conf_blk[1]; //0x10;//conformance[1]
            info[k-13]=conf_blk[2]; //0x10;//conformance[3]
            unionTemp.u16Temp=max_info_rec;
            info[k-11]=unionTemp.a8Temp[1];
            info[k-10]=unionTemp.a8Temp[0];
            unionTemp.u16Temp=max_info_tra;
            info[k-8]=unionTemp.a8Temp[1];
            info[k-7]=unionTemp.a8Temp[0];
			break;
		  case 0x00280001:	//PC Association
			fill_info(XDLMS_TYPE_CONST);
			info[k-15]=0x00;
			info[k-14]=0x00;
			info[k-13]=0x10;
			unionTemp.u16Temp=DLMS_MAX_BUFF_SIZE;
			info[k-11]=unionTemp.a8Temp[1];
			info[k-10]=unionTemp.a8Temp[0];
			unionTemp.u16Temp=DLMS_MAX_BUFF_SIZE;
			info[k-8]=unionTemp.a8Temp[1];
			info[k-7]=unionTemp.a8Temp[0];
			break;
		  default:
			fill_0b();
			break;
		}	
		break;		
		
	case 0x0f06://authentication mech_name
		if((obis1 == 0x00280000)||(obis1 == 0x00280001))
		{
			auth_name();
		}
		else
		fill_0b();
		break;
	case 0x0f07:
		if((obis1 == 0x00280000)||(obis1 == 0x00280001))
		{
			fill_0d();
		}
		else
		fill_0b();
		break;
	case 0x0f08://association status
		if((obis1 == 0x00280000)||(obis1 == 0x00280001))
		{
			asso_status();
		}
		else
		fill_0b();
		break;
	case 0x0101:
		switch(obis1)
		{
		case 0x002a0000:	//logical device name
			log_name2(0,0,42,0,0);
			break;
		case 0x00600100:	 	// meter serial number
			log_name2(0,0,96,1,0);
			break;
		default:
			fill_0b();
			break;
		}
		break;
	case 0x0102:
		switch(obis1)
		{
		case 0x002a0000:	//logical device name
#if LDN == 0
			octet_s(LOGICAL_DEVICE_NAME[0],1);
			fill_info(LOGICAL_DEVICE_NAME);  
#else
            octet_s(LOGICAL_DEVICE_NAME_GIL[0],1);			
            fill_info(LOGICAL_DEVICE_NAME_GIL);  
#endif
			break;
		case 0x00600100:	 	// meter serial number
			info[k++]=0;
			sr_no_ascii();
			break;
		case 0x01608009:
			if(asso3_flag==1)
			{
				info[k++]=0;
				info[k++]=0x06;
				Eprom_Read(PCBSerialNo_add);
				if(EPROM_bChksum_ok == 0)
				{
					Eprom_Read(Dupli_PCBSerialNo_add);
				}
				
				memcpy(info+k,opr_data,4);
				k+=4;
			}
			else
			fill_0d();
			break; 
		case 0x0160800E:                              //firmware no and calibration status
			if(asso3_flag==1)
			{
				info[k++]=0;
				structure(2);
				val_2byt(FirmId[0],FirmId[1]);
				Eprom_Read(LagCalibrationCoffadd);
				if(opr_data[11]==0x33)
				{
					opr_data[11]=0x03;
				}
				else
				{
					opr_data[11]=0x00;
				}
				val_2byt(opr_data[10],opr_data[11]);     // two bytes calibration data upf,lag (12(integer)-XX(upf)-XX(lag))
				
				//                if(clib_lag_status==0x03&&clib_upf_status==0x03)
				//                    unsigned8(0x03,0);
				//                else
				//                    unsigned8(0x00,0);
			}
			else
			fill_0d();
			break; 
		case 0x01600A01:                                  //instant tamper status
//			if(asso3_flag==1)
//			{
				tamper_status();
//			}
//			else
//			fill_0d();
			break; 
		  case 0x0160803E:                                  //instant Eprom testing
			if(asso3_flag==1)// && (0==fg_done_f))
			{
				// structure--- No of Memories
				// MemoryStatus1,MemoryStatus2--- Memory Type	1:16K, 2:256K, 3:512K, 4:1Mb
				info[k++]=0x00;
				structure(NoOfMemoryPresent);
				unsigned8(MemoryStatus1,0);
#if NoOfMemoryPresent ==2				
				unsigned8(MemoryStatus2,0);
#endif				
			}
			else
			{
				fill_0d();
			}
			break;			
		case 0x01608011:                                  //RTC calibration status
			if(asso3_flag==1)
			{
				info[k++]=0;
				structure(2);
				Eprom_Read(RTC_Calibration_add);
				if(opr_data[2] == 1)
				{
					unsigned8(0x80,0);
				}
				else
				{
					unsigned8(0x00,0);
				}
				unsigned8(opr_data[3],0);
			}
			else
			fill_0d();
			break; 
			//          case 0x00028002://IMEI no
			//            if(asso3_flag==1)
			//            {
			//                Eprom_Read(GPR_ModuleIMEIAddr);
			//                if(gpr_a8IMEIno[0]>16)
			//                    gpr_a8IMEIno[0]=0;
			//                octet_s(gpr_a8IMEIno[0],1);
			//                memcpy(info+k,gpr_a8IMEIno+1,gpr_a8IMEIno[0]);
			//                k+=gpr_a8IMEIno[0];
			//            }
			//            else
			//                fill_0d();
			//            break; 
		default:
			fill_0b();
			break;
		}
		break;
	default:
		fill_0b();
		break;
	}
}


void get_resp(void)
{
	unsigned int classatt,i_g,i;
	unsigned long int obis1;
	unsigned char cl_ob[4],class_att[2],ident=1,not_found_f=1,sec=0,j_g,addpg;
	signed char temp1;
	static union
	{
		u8 a8Temp[2];
		u16 u16Temp;
	}unionTemp;

	class_att[0]=info[7];
	class_att[1]=attribute_id;
	if(attribute_id == 1)
	class_att[0]=0;
	classatt=a8_to_u16(&class_att[0]);
	cl_ob[0]=info[8];
	cl_ob[1]=info[10];
	cl_ob[2]=info[11];
	cl_ob[3]=info[12];
	obis1=a8_to_u32(&cl_ob[0]);
	
	switch(classatt)
	{
	case 0x0001:
		if(((asso1_flag)||(asso2_flag))&&(obis1==0x00280001))
			fill_0b();
			if(obis1==0x00280000)
			log_name2(info[8],info[9],info[10],info[11],info[12]);
			else
			{
		for(i_g=3;i_g<=((u16)OBJ_LIST[2]*(u16)9 + (u16)3);i_g+=9)
		{
			if((OBJ_LIST[i_g+1] == info[7]) && (OBJ_LIST[i_g+3] == info[8]) && (OBJ_LIST[i_g+4] == info[9]) && (OBJ_LIST[i_g+5] == info[10]) && (OBJ_LIST[i_g+6] == info[11]) && (OBJ_LIST[i_g+7] == info[12]))
			{
				not_found_f=0;
				if(obis_code[2] == 40 && 
				   (((obis_code[4]==3) && asso2_flag ==1) || 
					((obis_code[4]==2) && asso1_flag ==1) || 
				    ((obis_code[4]==1) || (obis_code[4]==0))))
				log_name2(info[8],info[9],info[10],info[11],info[12]);
				else if(obis_code[2] != 40)
				log_name2(info[8],info[9],info[10],info[11],info[12]);
				
				break;
				
			}
			
		}
		
		if((i_g > (OBJ_LIST[2] * 9)) && (not_found_f ==1))
		fill_0b();
			}
		break;
	case 0x0102:
		switch(obis1)
		{
		case 0x002A0000:
#if LDN == 0
			// info[k++]=0;
			octet_s(LOGICAL_DEVICE_NAME[0],1);
			fill_info(LOGICAL_DEVICE_NAME);
#else
			octet_s(LOGICAL_DEVICE_NAME_GIL[0],1);
			fill_info(LOGICAL_DEVICE_NAME_GIL);
#endif
			break;
		case 0x00600100:
			info[k++]=0;
			sr_no_ascii();
			break;
		case 0x00600700:
			
			val_2byt2(0,PWR_u8on_off_cnt);
			break;
			
			
		case 0x00603300://daily on-off
			unsigned8(daily_on_off,1);
			break;
			
		case 0x005e5b00:
			FUN_vfill_2byteR(TPR_u16cum_tpr_c,&opr_data[1]);
			val_2byt2(opr_data[0],opr_data[1]);
			break;
		case 0x005e5b01:
			unsigned8(BILL_u8btpr_c,1);
			break;
		case 0x00000100:
			//unsigned8(BILL_u8md_count,1);
			val_4byt2(0x00,0x00,0x00,BILL_u8md_count);
			break;
		case 0x00000101:
			if(BILL_u8md_count>=MaxBillDate)
			unsigned8(MaxBillDate,1);
			else
			unsigned8(BILL_u8md_count,1);
			break;
		case 0x00600200:
			val_2byt2(0,trans_count);
			break;
		case 0x005e5b09:
			unsigned8(5,1);
			break;
		case 0x00600101:
#if LDN == 0
			fill_info(manufacturer_name);
#else
			fill_info(manufacturer_name_GIL);
#endif
			break;
		case 0x01000200:
			info[k++]=0x00;
			fill_firmware_version();
			break;
		case 0x01000402:
		case 0x01000403:
			val_2byt2(0,1);
			break;
		case 0x00600104: 
			
			
			Eprom_Read(FG_DateTimeAdd);
			unionTemp.u16Temp=2000+bcd_to_hex(opr_data[2]);
			info[k++]=0x00;
			info[k++]=0x12;
			info[k++]=unionTemp.a8Temp[1];
			info[k++]=unionTemp.a8Temp[0];         //2012 year of manufacturing
			
			
			
			break;
		case 0x01000800:
			unionTemp.u16Temp=(u16)md_ip*(u16)60;
			val_2byt2(unionTemp.a8Temp[1],unionTemp.a8Temp[0]);
			break;
			
		case 0x01000804:
			unionTemp.u16Temp=(u16)ls_ip*(u16)60;
			val_2byt2(unionTemp.a8Temp[1],unionTemp.a8Temp[0]);
			break;
          case 0x00600B00:
			Eprom_Read(volt_tamp_status);
			val_2byt2(0,opr_data[14]);
			break;
		  case 0x00600B01:
			Eprom_Read(curr_tamp_status); //cngd
			val_2byt2(0,opr_data[14]);
			break;
			
		  case 0x00600B02:
			Eprom_Read(pwr_tamp_status); //cngd
			val_2byt2(0,opr_data[14]);
			break;
		  case 0x00600B03:
			Eprom_Read(trans_tamp_status);
			val_2byt2(0,opr_data[14]);
			break;
			
		  case 0x00600B04:
			Eprom_Read(other_tamp_status); //cngd
			val_2byt2(0,opr_data[14]);
			break;
			
		  case 0x00600B05:
			Eprom_Read(nonroll_status);
			val_2byt2(0,opr_data[14]);
			break;
			
		  case 0x00600B06:
			Eprom_Read(control_tamp_status);
			if(opr_data[14])
			{
				unionTemp.u16Temp=300+opr_data[14];
			}
			else
			unionTemp.u16Temp=0;
			val_2byt2( unionTemp.a8Temp[1], unionTemp.a8Temp[0]);       ///connect event
			break;
		  case 0x00600B07:			//
			Eprom_Read(Diagnostics_status);
			if(opr_data[14])
			{
				unionTemp.u16Temp=300+opr_data[14];
			}
			else
			unionTemp.u16Temp=0;
			val_2byt2( unionTemp.a8Temp[1], unionTemp.a8Temp[0]); 
			break;
			
		case 0x0060020d://0.0.96.2.13.255:
//			Eprom_Read(0x2640);
//			date_time(opr_data[4],opr_data[3],opr_data[2],opr_data[1],opr_data[0],0,1);
			break;
			
			//		  case 0x00600c05:
			//			unsigned8(GPR_u8RSSI,1);
			//			break;
			
			//		  case 0x01608005:
			//			unsigned8(rly_discnt,1);
			//            break;
			
		case 0x01608009://pcb tracking
			Eprom_Read(PCBSerialNo_add);
			if(EPROM_bChksum_ok==0)
			Eprom_Read(Dupli_PCBSerialNo_add);
			info[k++]=0;
			info[k++]=0x06;
			memcpy(info+k,opr_data,4);
			k+=4;
			break;
			
		case 0x01603200://        TPRCNT_u8NeuMiss    =opr_data[0];
			Eprom_Read(TPRCNT_Addr);
			unsigned8(opr_data[0],1);
			break;
		case 0x01603300://        TPRCNT_u8VHigh      =opr_data[1];
			Eprom_Read(TPRCNT_Addr);
			unsigned8(opr_data[1],1);
			break;
		case 0x01603400://        TPRCNT_u8VLow       =opr_data[2];
			Eprom_Read(TPRCNT_Addr);
			unsigned8(opr_data[2],1);
			break;
		case 0x01603500://        TPRCNT_u8Rev        =opr_data[3];
			Eprom_Read(TPRCNT_Addr);
			unsigned8(opr_data[3],1);
			break;
		case 0x01603600://        TPRCNT_u8EL         =opr_data[4];
			Eprom_Read(TPRCNT_Addr);
			unsigned8(opr_data[4],1);
			break;
		case 0x01603700://        TPRCNT_u8OC         =opr_data[5];
			Eprom_Read(TPRCNT_Addr);
			unsigned8(opr_data[5],1);
			break;
		case 0x01603800://        TPRCNT_u8OverLoad   =opr_data[6];
			Eprom_Read(TPRCNT_Addr);
			unsigned8(opr_data[6],1);
			break;
		case 0x01603900://        TPRCNT_u8Mag        =opr_data[7];
			Eprom_Read(TPRCNT_Addr);
			unsigned8(opr_data[7],1);
			break;
		case 0x01603a00://        TPRCNT_u8NeuDis     =opr_data[8];
			Eprom_Read(TPRCNT_Addr);
			unsigned8(opr_data[8],1);
			break;
		case 0x01603b00://        TPRCNT_u8LowPF      =opr_data[9];
			Eprom_Read(TPRCNT_Addr);
			unsigned8(opr_data[9],1);
			break;
		case 0x01603c00://        TPRCNT_u835kv       =opr_data[10];
			Eprom_Read(TPRCNT_Addr);
			unsigned8(opr_data[10],1);
			break;
		case 0x01603d00://        TPRCNT_u8FreqTamp   =opr_data[11];
			Eprom_Read(TPRCNT_Addr);
			unsigned8(opr_data[11],1);
			break;
		case 0x01603e00://        TPRCNT_u8TC         =opr_data[12];
			Eprom_Read(TPRCNT_Addr);
			unsigned8(opr_data[12],1);
			break;
		case 0x0160800e:
			info[k++]=0;
			structure(2);
			val_2byt(FirmId[0],FirmId[1]);
			Eprom_Read(LagCalibrationCoffadd);
			if(opr_data[11]==0x33)
			{
				opr_data[11]=0x03;
			}
			else
			{
				opr_data[11]=0x00;
			}
			val_2byt(opr_data[10],opr_data[11]);     // two bytes calibration data upf,lag (12(integer)-XX(upf)-XX(lag))
			
			//            if(clib_lag_status==0x03&&clib_upf_status==0x03)
			//                unsigned8(0x03,0);
			//            else
			//                unsigned8(0x00,0);
			break;
			//          case 0x00028002://IMEI no
			//            Eprom_Read(GPR_ModuleIMEIAddr);
			//            if(gpr_a8IMEIno[0]>16)
			//                gpr_a8IMEIno[0]=0;
			//            octet_s(gpr_a8IMEIno[0],1);
			//            memcpy(info+k,gpr_a8IMEIno+1,gpr_a8IMEIno[0]);
			//            k+=gpr_a8IMEIno[0];
			//            break;
			
		case 0x005e5b0c:
			info[k++] = 0x00;
			info[k++] = 0x0a;
			
			Eprom_Read(CurrentRating_addrs); 
			info[k++] = opr_data[0];
			for(i=1; i<=opr_data[0];++i)
			{
				info[k++] = opr_data[i];  
			}
			break;
		case 0x005e5b0b:
			info[k++] = 0x00;//unsigned8(0xc3,1);//    meter_catagory()
			info[k++] = 0x0a;
			info[k++] = 0x02;
			info[k++] = 0x43;//ascii for 'C'
			info[k++] = 0x33;//ascii for '3'
			break; 
			
		case 0x01608012:          //utility id as visible string
			info[k++] = 0x00;//unsigned8(0xc3,1);//    meter_catagory()
			info[k++] = 0x0a;
			
			Eprom_Read(UtilityIDAdd); 
			info[k++] = opr_data[0];
			memcpy(info+k,opr_data+1,opr_data[0]);
			k+=opr_data[0]; 

			break; 
			
		default:
			fill_0b();  
			break;
		}
		break;
	case 0x0302:
		switch(obis1)
		{
		case 0x00000102://bill date
			temp1=0;
			temp1 =BILL_u8mdmonth-1;
			if(temp1<=0)
			temp1=temp1+MaxBillDate+1;                
			temp1=calmdpg(temp1);                
			
			Eprom_Read(billkwh_data_addrs+(unsigned char)temp1);
			date_time(opr_data[10],opr_data[9],opr_data[8],opr_data[7],opr_data[6],0,1); //d&t of last MD reset    
			break;
			
		case 0x015b0700:
			if((MET_u32in_rms<=9)&&(test_mode==0))
			MET_u32in_rms=0;
			FUN_vfill_3byteR(MET_u32in_rms,&(opr_data[2]));
			val_4byt2(0x00,opr_data[0],opr_data[1],opr_data[2]);
			break;
			
		case 0x010b0700:
			if((MET_u32ip_rms<=9)&&(test_mode==0))
			{
				MET_u32ip_rms=0;
				MET_u32ip_rms_tamp=0;
			}
			FUN_vfill_3byteR(MET_u32ip_rms,&(opr_data[2]));
			val_4byt2(0x00,opr_data[0],opr_data[1],opr_data[2]);
			break;
			
		case 0x010c0700:
			unionTemp.u16Temp=MET_u16v_rms;
			val_2byt2( unionTemp.a8Temp[1], unionTemp.a8Temp[0]);
			break;
			
		case 0x010D0700:
			info[k++]=0x00;
			unionTemp.u16Temp=MET_u16sign_net_pf;            
			val_2byt_signed( unionTemp.a8Temp[1], unionTemp.a8Temp[0]);
			break;
			
		case 0x010E0700:
			unionTemp.u16Temp=MET_u16freq;            
			val_2byt2( unionTemp.a8Temp[1], unionTemp.a8Temp[0]);
			
			break;
			
		case 0x01010700:
			info[k++]=0x00;
			if(MET_bph_ct_f1 == 0)
			{
				unionTemp.u16Temp=MET_u16signed_Kw; 
			}
			else
			{
				unionTemp.u16Temp=MET_u16signed_ELKw; 
			}
			           
			val_2byt_signed(unionTemp.a8Temp[1], unionTemp.a8Temp[0]);
			
			break;
			
		case 0x01090700:
			unionTemp.u16Temp=MET_u16Kva;            
			val_2byt2(unionTemp.a8Temp[1], unionTemp.a8Temp[0]);
			
			break;
			
	    case 0x005e5b0e:
			FUN_vfill_3byteR(cum_pow_on,&opr_data[2]);//cumulative power failure duration
			val_4byt2(0,opr_data[0],opr_data[1],opr_data[2]);
			break;
			
		case 0x01010800:
			FUN_vfill_3byteR(MET_u32Cum_kwh,&(opr_data[2]));
			val_4byt2(0x00,opr_data[0],opr_data[1],opr_data[2]);
			break;
			
		case 0x01090800:
			FUN_vfill_3byteR(MET_u32Cum_kvah,&(opr_data[2]));
			val_4byt2(0x00,opr_data[0],opr_data[1],opr_data[2]);
			break;
		case 0x01B00800://defraud mag 1.0.176.8.0.255
			FUN_vfill_3byteR(TPR_u32cum_mag_defraud,&(opr_data[2]));
			val_4byt2(0x00,opr_data[0],opr_data[1],opr_data[2]);
			break;
		case 0x01B10800://defraud neutral
			FUN_vfill_3byteR(TPR_u32cum_neutemp_defraud,&(opr_data[2]));
			val_4byt2(0x00,opr_data[0],opr_data[1],opr_data[2]);
			break;
			
			case 0x01050800:  //Q1loadsur_kvarh stamp block  ====  Q1  //1,0,5,8,0,255,
			FUN_vfill_3byteR(MET_u32Cum_kvarh_Lag,&(opr_data[2]));
			val_4byt2(0x00,opr_data[0],opr_data[1],opr_data[2]);
			break;
			
			case 0x01080800:  //Q2loadsur_kvarh stamp block    ===== Q4 as per BLA sir.   1,0,8,8,0,255,
			FUN_vfill_3byteR(MET_u32Cum_kvarh_Lead,&(opr_data[2]));
			val_4byt2(0x00,opr_data[0],opr_data[1],opr_data[2]);
			break;
			
			
			case 0x01030800:   //Q1+Q2 cumulative lag+lead energy  1,0,3,8,0,255,
			FUN_vfill_3byteR(MET_u32Cum_kvarh,&(opr_data[2]));
			val_4byt2(0x00,opr_data[0],opr_data[1],opr_data[2]);
			break;
			
			
			
		 case 0x018E0800://High Resolution kWh
		    MET_vcheck_kwh_kvah();
	        hi_tempkwh = Compute_EnergyHighres(1);
//			hi_tempkwh=((MET_u16kw_pulse_cntr*(unsigned long)3125)/1000);
//	        hi_tempkwh = (MET_u32Cum_kwh*1000)+hi_tempkwh;
			FUN_vfill_4byteR(hi_tempkwh,&(opr_data[3]));
			val_4byt2(opr_data[0],opr_data[1],opr_data[2],opr_data[3]);
			break;
		case 0x01910800://High Resolution kvah
		    MET_vcheck_kwh_kvah();
	        hi_tempkvah = Compute_EnergyHighres(2);		
//			hi_tempkvah=((MET_u16kva_pulse_cntr*(unsigned long)3125)/1000);
//	        hi_tempkvah = (MET_u32Cum_kvah*1000)+hi_tempkvah;
			FUN_vfill_4byteR(hi_tempkvah,&(opr_data[3]));
			val_4byt2(opr_data[0],opr_data[1],opr_data[2],opr_data[3]);
			break;
		case 0x018E0700://High Resolution kw
			FUN_vfill_4byteR(MET_u32kwHires_1s,&(opr_data[3]));
			val_4byt2(opr_data[0],opr_data[1],opr_data[2],opr_data[3]);
			break;
		case 0x01910700://High Resolution kVA
			FUN_vfill_4byteR(MET_u32kVAHires_1s,&(opr_data[3]));
			val_4byt2(opr_data[0],opr_data[1],opr_data[2],opr_data[3]);
			break;
		case 0x01A90700://High Resolution power kW (CT element)  //kvar Lag 
			FUN_vfill_4byteR(MET_u32kwHires_el_1s,&(opr_data[3]));
			val_4byt2(opr_data[0],opr_data[1],opr_data[2],opr_data[3]);
			break;
			
//		case 0x00600982://Instant Temperature
//			unionTemp.u16Temp=MET_u16Temprature_MAX;
//			val_2byt2( unionTemp.a8Temp[1], unionTemp.a8Temp[0]);
//			//unsigned8(MET_u8Temprature,1);
//			break;
			
//		case 0x01010200:                              
//			Eprom_Read(CumMDAddr);
//			val_4byt2(opr_data[0],opr_data[1],opr_data[2],opr_data[3]); //cum md kW
//			break;      
		default:
			fill_0b();
			
		}
		break;
	case 0x0303:
		switch(obis1)
		{
		case 0x00000102:
			sca_unit(0,0xff,1);
break;
		case 0x015b0700:
			sca_unit(0xfd,33,1);
			break;
		case 0x010b0700:
			sca_unit(0xfd,33,1);
			break;
		case 0x010c0700:
			sca_unit(0xfe,35,1);
			break;
		case 0x010D0700:
			sca_unit(0xfd,0xff,1);
			break;
		case 0x010E0700:
			sca_unit(0xfd,44,1);
			break;
		case 0x01010700:
			sca_unit(0,27,1);
			break;
		case 0x01090700:
			sca_unit(0,28,1);
			break;
	   case 0x005e5b0e:
			sca_unit(0,0x06,1);
			break;

		case 0x01010800:
			sca_unit(2,30,1);
			break;
		case 0x01090800:
			sca_unit(2,31,1);
			break;
		case 0x01B00800:
			sca_unit(2,30,1);
			break;
		case 0x01B10800:
			sca_unit(2,30,1);
			break;
			
		case 0x01050800:  //Q1loadsur_kvarh stamp block  ====  Q1  //1,0,5,8,0,255,
			sca_unit(2,32,1);
		break;
			
		case 0x01080800:  //Q2loadsur_kvarh stamp block    ===== Q4 as per BLA sir.   1,0,8,8,0,255,
			sca_unit(2,32,1);
		break;
			
			
		case 0x01030800:   //Q1+Q2 cumulative lag+lead energy  1,0,3,8,0,255,
			sca_unit(2,32,1);
		break;
			
		case 0x018E0800://High Resolution kWh
		    sca_unit(0xff,30,1);
			break;
		case 0x01910800://High Resolution kvah
			sca_unit(0xff,31,1);
			break;
		case 0x018E0700://High Resolution power kW (main element)
			sca_unit(0xfc,27,1);
			break;
		case 0x01910700://High Resolution kVA
		    sca_unit(0xfc,28,1); 			
			break;
		case 0x01A90700://High Resolution power kW (CT element)
			sca_unit(0xfc,27,1);
			break;
			
//			case 0x00600982://Instant Temperature
//			sca_unit(0x00,9,1);
//			break;
                case 0x01010200:
                  sca_unit(0,27,1);
                  break;
		default:
			fill_0b();
			
		}
		break;
		
	case 0x0402:
		switch(obis1)
		{
		case 0x01010600:
			temp1=calmdpg(BILL_u8mdmonth);                     
			Eprom_Read(billmd_data_addrs+(unsigned char)temp1);
			val_4byt2(0x00,0x00,opr_data[0],opr_data[1]); //md kW      //current avg value
			break;
		case 0x01090600:                              
			temp1=calmdpg(BILL_u8mdmonth);                     
			Eprom_Read(billmd_data_addrs+(unsigned char)temp1);
			val_4byt2(0x00,0x00,opr_data[7],opr_data[8]); //md kva      //current avg value
			break;
			//          case 0x01011100:
			//			temp1=calmdpg(mdmonth);                     
			//			Eprom_Read(Universalmd_data_addrs);
			//			val_4byt2(0x00,0x00,opr_data[0],opr_data[1]); //Universal md kW      //current avg value
			//			break;
			//		  case 0x01091100:                              
			//			temp1=calmdpg(mdmonth);                     
			//			Eprom_Read(Universalmd_data_addrs);
			//			val_4byt2(0x00,0x00,opr_data[7],opr_data[8]); //Universal md kva      //current avg value
			//			break;
		default:
			fill_0b();
			
		}
		break;
	case 0x0403:
		switch(obis1)
		{
		case 0x01010600:
			sca_unit(0,27,1);
			break;
		case 0x01090600:
			sca_unit(0,28,1);
			break;
			//          case 0x01011100:
			//			sca_unit(0,27,1);
			//			break;
			//		  case 0x01091100:
			//			sca_unit(0,28,1);
			//			break;
		default:
			fill_0b();
			
		}
		break;
	case 0x0404:
		fill_0d();
		break;
		
	case 0x0405:
		switch(obis1)
		{
		case 0x01010600:                         
			temp1=calmdpg(BILL_u8mdmonth);                     
			Eprom_Read(billmd_data_addrs+(unsigned char)temp1);
			sec=0;
			if((opr_data[2]==0xff)&&(opr_data[3]==0xff))
			{
				sec=0xff;
			}
			
			date_time(opr_data[6],opr_data[5],opr_data[4],opr_data[3],opr_data[2],sec,1);
			
			break;                     
		case 0x01090600:
			temp1=calmdpg(BILL_u8mdmonth);                   
			Eprom_Read(billmd_data_addrs+(unsigned char)temp1);
			
sec=0;			
if((opr_data[9]==0xff)&&(opr_data[10]==0xff))
			{
				sec=0xff;
			}
			
			date_time(opr_data[13],opr_data[12],opr_data[11],opr_data[10],opr_data[9],sec,1);
			break;
			//          case 0x01011100:                         
			//			temp1=calmdpg(mdmonth);                     
			//			Eprom_Read(Universalmd_data_addrs);
			//			date_time(opr_data[6],opr_data[5],opr_data[4],opr_data[3],opr_data[2],0,1);
			//			break;                     
			//		  case 0x01091100:
			//			temp1=calmdpg(mdmonth);                   
			//			Eprom_Read(Universalmd_data_addrs);
			//			date_time(opr_data[13],opr_data[12],opr_data[11],opr_data[10],opr_data[9],0,1);
			//			break;
		default:
			fill_0b();
			
		}
		break;
		
	case 0x0702:
		sel_access_flag1=0;
		i_dlms=0;
		switch(obis1)
		{
		case 0x015e5b00:
			buffer_instantaneous_parameter();
			break;
		case 0x015e5b03:
			buffer_scaler_filler(instantaneous_parameter_scaler_buffer);
			break;
		case 0x015e5b04:
			buffer_scaler_filler(blockload_survey_parameter_scaler_buffer);
			break;
#if DailyEnergy == 2            
		case 0x015e5b05:
			buffer_scaler_filler(dailyload_profile_parameter_scaler_buffer);
			break;
#endif            
		case 0x015e5b06:
			buffer_scaler_filler(bill_profile_parameter_scaler_buffer);
			break;
		case 0x015e5b07:
			buffer_scaler_filler(event_log_profile_scaler_buffer);
			break;
		case 0x01630100:
			//			Data_block = 0x01;
			ls_count_local = 0x00;
			
			//Eprom_Read(load_survey_status);
			ls_count_dlms=load_survey_cnt+1;//a8_to_u16(&opr_data[0]);//no. of load survey
			//LS.b.ls_of=opr_data[2];
			
			if(LS.b.ls_of==1)
			{
				UintLoadSurptr=ls_count_dlms;
				ls_count_dlms= max_loadsurvey;
			}
			else
			{
				UintLoadSurptr=0;
			}
			Sel_Loadsurvey_buffer();
			break;
#if DailyEnergy == 2            
		case 0x01630200:////daily load profile
			dls_count_local = 0x00;
			
			//Eprom_Read(DailyEnergyStatus);
			dls_count_dlms=daily_enrcount;//a8_to_u16(&opr_data[0]);//no. of load survey
			//DE.b.dls_of=opr_data[5];
			
			UintLoadSurptr1=dloadsurvey_init_add;
			if(DE.b.dls_of == 1 )// && sel_access_flag != 1)
			{ 
				UintLoadSurptr1+=(u16)((u16)daily_enrcount*0x0020);
				dls_count_dlms=maxday_counter_d;
			}
			
			
			
			Sel_DailyLoadsurvey_buffer();
			break;
#endif
			
		case 0x01620100:
			bill_buffer();
			break;
			
		  case 0x00636200:
			Eprom_Read(volt_tamp_status);
			tamper_compart(volt_max_loc,volt_init_add,volt_max_add);
			break;
		  case 0x00636201:
			Eprom_Read(curr_tamp_status);
			tamper_compart(curr_max_loc,curr_init_add,curr_max_add);
			break;
		  case 0x00636202:
//			sel_obj_tamper[2]=0;
//			sel_obj_tamper[3]=0;
//			sel_obj_tamper[4]=0;
//			sel_obj_tamper[5]=0;
			
			for(i=2;i<7;i++)
				sel_obj_tamper[i]=0;

			if(sel_access_flag!=1)
			{
				sel_obj_tamper[0]=1;
				sel_obj_tamper[1]=1;
				sel_access_flag1=1;
				no_obj=2;
			}
			else
			{
				sel_obj_tamper[0]=1;
				if(no_obj>=2)
				{
					sel_obj_tamper[1]=1;
					no_obj=2;
				}
				
			}
			
			Eprom_Read(pwr_tamp_status);
			tamper_compart(pwr_max_loc,pwr_init_add,pwr_max_add);
			break;
		  case 0x00636203:
//			sel_obj_tamper[2]=0;
//			sel_obj_tamper[3]=0;
//			sel_obj_tamper[4]=0;
//			sel_obj_tamper[5]=0;

			for(i=2;i<7;i++)
				sel_obj_tamper[i]=0;

			if(sel_access_flag!=1)
			{
				sel_obj_tamper[0]=1;
				sel_obj_tamper[1]=1;
				sel_access_flag1=1;
				no_obj=2;
			}
			else
			{
				sel_obj_tamper[0]=1;
				if(no_obj>=2)
				{
					sel_obj_tamper[1]=1;
					no_obj=2;
				}
				
			}
			Eprom_Read(trans_tamp_status);
			tamper_compart(trans_max_loc,trans_init_add,trans_max_add);
			break;
		  case 0x00636204:
			Eprom_Read(other_tamp_status);
			tamper_compart(other_max_loc,other_init_add,other_max_add);
			break;
		  case 0x00636205:
//			sel_obj_tamper[2]=0;
//			sel_obj_tamper[3]=0;
//			sel_obj_tamper[4]=0;
//			sel_obj_tamper[5]=0;

			for(i=2;i<7;i++)
				sel_obj_tamper[i]=0;

			if(sel_access_flag!=1)
			{
				sel_obj_tamper[0]=1;
				sel_obj_tamper[1]=1;
				sel_access_flag1=1;
				no_obj=2;
			}
			else
			{
				sel_obj_tamper[0]=1;
				if(no_obj>=2)
				{
					sel_obj_tamper[1]=1;
					no_obj=2;
				}
				
			}
			Eprom_Read(nonroll_status);
			tamper_compart(nonroll_max_loc,nonroll_init_add,nonroll_max_add);
			break;
		  case 0x00636206:
			sel_obj_tamper[2]=0;
			sel_obj_tamper[3]=0;
			sel_obj_tamper[4]=0;
			sel_obj_tamper[5]=0;
			
			if(sel_access_flag!=1)
			{
				sel_obj_tamper[0]=1;
				sel_obj_tamper[1]=1;
				sel_access_flag1=1;
				no_obj=2;
			}
			else
			{
				sel_obj_tamper[0]=1;
				if(no_obj>=2)
				{
					sel_obj_tamper[1]=1;
					no_obj=2;
				}
				
			}
			Eprom_Read(control_tamp_status);
			tamper_compart((control_max_loc),control_init_add,control_max_add);
			break;
		  case 0x00636207:
			sel_obj_tamper[2]=0;
			sel_obj_tamper[3]=0;
			sel_obj_tamper[4]=0;
			sel_obj_tamper[5]=0;
			
			if(sel_access_flag!=1)
			{
				sel_obj_tamper[0]=1;
				sel_obj_tamper[1]=1;
				sel_access_flag1=1;
				no_obj=2;
			}
			else
			{
				sel_obj_tamper[0]=1;
				if(no_obj>=2)
				{
					sel_obj_tamper[1]=1;
					no_obj=2;
				}
				
			}
			Eprom_Read(Diagnostics_status);
			tamper_compart((Diagnostics_max_loc),Diagnostics_init_add,Diagnostics_max_add);
			break;
			
		case 0x005e5b0a:
			buffer_NamePlateDetails_parameter();
			break;
			
		default:
			fill_0b();
			
		}
		break;
	case 0x0703:
		i_dlms=0;
		switch(obis1)
		{
		case 0x015e5b00:
			capture_objects_filler(instantaneous_parameter_cap_obj);
			break;
		case 0x015e5b03:
			capture_objects_filler(instantaneous_parameter_scaler_cap_obj);   // capture object attribute for billing
			break;
		case 0x015e5b04:
			capture_objects_filler(blockload_survey_parameter_scaler_cap_obj);
			break;
#if DailyEnergy == 2
		case 0x015e5b05:
			capture_objects_filler(dailyload_profile_parameter_scaler_cap_obj);
			break;
#endif            
		case 0x015e5b06:
			capture_objects_filler(bill_profile_parameter_scaler_cap_obj); // capture object attribute for billing
			break;
		case 0x015e5b07:
			capture_objects_filler(event_log_profile_scaler_cap_obj);
			break;
		case 0x01630100:
			capture_objects_filler(load_survey_parameter_cap_obj);// capture object attribute for load survey
			break;
#if DailyEnergy == 2
		case 0x01630200:
			capture_objects_filler(dailyload_profile_parameter_cap_obj);// capture object attribute for load survey
			break;
#endif            
		case 0x01620100:
			capture_objects_filler(bill_profile_parameter_cap_obj); // capture object attribute for billing
			break;
		  case 0x00636200:
			capture_objects_filler(voltage_event_capture_obj);
			break;
		  case 0x00636201:
			capture_objects_filler(current_event_capture_obj);
			break;
		  case 0x00636202:
			capture_objects_filler(power_event_capture_obj);
			break;
		  case 0x00636203:
			capture_objects_filler(transaction_event_capture_obj);
			break;
		  case 0x00636204:
			capture_objects_filler(other_event_capture_obj);
			break;
		  case 0x00636205:
			capture_objects_filler(non_rollover_event_capture_obj);
			break;
		  case 0x00636206:
			capture_objects_filler(control_event_capture_obj);
			break;
		  case 0x00636207:
			capture_objects_filler(Diagnostics_event_capture_obj);
			break;
		case 0x005e5b0a:
			capture_objects_filler(NamePlateDetails_parameter_cap_obj);     //name plate profile cap obj.
			break;
			
		default:
			fill_0b();
			
		}
		break;
	case 0x0704:
		switch(obis1)
		{
		case 0x015e5b00:
		case 0x005e5b0a:
		case 0x015e5b03:
		case 0x015e5b04:
#if DailyEnergy == 2            
		case 0x015e5b05:
#endif            
		case 0x015e5b06:
		case 0x015e5b07:
		case 0x015e5b0a:
		case 0x01620100:
		case 0x00636200:
		case 0x00636201:
		case 0x00636202:
		case 0x00636203:
		case 0x00636204:
		case 0x00636205:
		case 0x00636206:
		  case 0x00636207:
			val_4byt2(0x00,0x00,0x00,0x00);
			break;
		case 0x01630100:
			unionTemp.u16Temp=(u16)ls_ip*60;            	   
			val_4byt2(0x00,0x00,unionTemp.a8Temp[1], unionTemp.a8Temp[0]);
			break;
#if DailyEnergy == 2 
		case 0x01630200:
			val_4byt2(0x00,0x01,0x51,0x80);//////capture period
			break;
#endif            
		default:
			fill_0b();
		}
		break;
	case 0x0705:
		switch(obis1)
		{
		case 0x015e5b00:
		case 0x005e5b0a:
		case 0x015e5b03:
		case 0x015e5b04:
		case 0x015e5b05:
		case 0x015e5b06:
		case 0x015e5b07:
		case 0x015e5b0a:
		case 0x01630100:
		case 0x01630200:
		case 0x01620100:
		case 0x00636200:
		case 0x00636201:
		case 0x00636202:
		case 0x00636203:
		case 0x00636204:
		case 0x00636205:
		case 0x00636206:
		  case 0x00636207:
			enum_d2(0x01);				//00
			break;
		default:
			fill_0b();
			
			
		}
		break;
	case 0x0706:
		switch(obis1)
		{
		case 0x015e5b00:
		case 0x005e5b0a:
		case 0x015e5b03:
		case 0x015e5b04:
#if DailyEnergy == 2            
		case 0x015e5b05:
#endif
		case 0x015e5b06:
		case 0x015e5b07:
		case 0x015e5b0a:
		case 0x01620100:
		case 0x00636200:
		case 0x00636201:
		case 0x00636202:
		case 0x00636203:
		case 0x00636204:
		case 0x00636205:
		case 0x00636206:
		  case 0x00636207:
			fill_info(sort_obj);
			//			info_len=k;
			//sort_object();
			break;
		case 0x01630100:
#if DailyEnergy == 2 
		case 0x01630200:
#endif
			fill_info(sort_obj1);
			break;
		default:
			fill_0b();
			
		}
		break;
	case 0x0707:
		switch(obis1)
		{
		case 0x015e5b00:
		case 0x005e5b0a:
		case 0x015e5b03:
		case 0x015e5b04:
#if DailyEnergy == 2            
		case 0x015e5b05:
#endif
		case 0x015e5b06:
		case 0x015e5b07:
		case 0x015e5b0a:
			val_4byt2(0x00,0x00,0x00,1);
			break;
		case 0x01630100:
			if(LS.b.ls_of != 1)
			{
				//Eprom_Read(load_survey_status);
				FUN_vfill_2byteR((load_survey_cnt+1),&opr_data[1]);
				val_4byt2(0,0,opr_data[0],opr_data[1]);
			}
			
			else
			{
				val_4byt2(0x00,0x00,max_loadsurvey/0x100,max_loadsurvey%0x100);
			}
			break;
#if DailyEnergy == 2            
		case 0x01630200:
			if(DE.b.dls_of != 1)
			{
				//Eprom_Read(DailyEnergyStatus);
				val_4byt2(0,0,0,daily_enrcount);
			}
			
			else
			{
				val_4byt2(0x00,0x00,maxday_counter_d/0x100,maxday_counter_d%0x100);
			}
			break;
#endif            
			
			
		case 0x01620100:
			if(BILL_u8md_count>=MaxBillDate)
			val_4byt2(0x00,0x00,0x00,MaxBillDate+1);
			else
			val_4byt2(0x00,0x00,0x00,BILL_u8md_count+1);
			break;
		  case 0x00636200:
			if(volt_of==0x01)
			val_4byt2(0,0,0,volt_max_loc);
			else
			val_4byt2(0,0,0,(volt_count));
			break;
		  case 0x00636201:
			if(curr_of==0x01)
			val_4byt2(0,0,0,curr_max_loc);
			else
			val_4byt2(0,0,0,(curr_count));
			break;
		  case 0x00636202:
			if(on_off_of==0x01)
			val_4byt2(0,0,0,pwr_max_loc);
			else
			val_4byt2(0,0,0,(on_off_loc));
			break;
		  case 0x00636203:
			if(trans_of==0x01)
			val_4byt2(0,0,0,trans_max_loc);
			else
			val_4byt2(0,0,0,(trans_count));
			break;
		  case 0x00636204:
			if(others_of==0x01)
			val_4byt2(0,0,0,other_max_loc);
			else
			val_4byt2(0,0,0,(others_count));
			break;
		  case 0x00636205:
			if(nonroll_of == 0x01)
			val_4byt2(0,0,0,nonroll_max_loc);
			else
			val_4byt2(0,0,0,nonroll_count);
			break;
//		  case 0x00636206:
//			if(control_of==0x01)
//				val_4byt2(0,0,0,control_max_loc);
//			else
//				val_4byt2(0,0,0,(control_count));
//			break;
		  case 0x00636207:
			if(Diagnostics_of==0x01)
			val_4byt2(0,0,0,Diagnostics_max_loc);
			else
			val_4byt2(0,0,0,(Diagnostics_count));
			break;
			
		default:
			fill_0b();
			
		}
		break;
	case 0x0708:
		switch(obis1)
		{
		case 0x015e5b00:
		case 0x005e5b0a:
		case 0x015e5b03:
		case 0x015e5b04:
#if DailyEnergy == 2            
		case 0x015e5b05:
#endif
		case 0x015e5b06:
		case 0x015e5b07:
			val_4byt2(0x00,0x00,0x00,0x01);
			break;
		case 0x01630100:
			val_4byt2(0x00,0x00,max_loadsurvey/0x100,max_loadsurvey%0x100);
			break;
#if DailyEnergy == 2            
		case 0x01630200:
			val_4byt2(0x00,0x00,maxday_counter_d/0x100,maxday_counter_d%0x100);
			break;
#endif            
		case 0x01620100:
			val_4byt2(0x00,0x00,0x00,MaxBillDate+1);
			break;
		  case 0x00636200:
			val_4byt2(0,0,0,volt_max_loc);
			break;
		  case 0x00636201:
			val_4byt2(0,0,0,curr_max_loc);
			break;
		  case 0x00636202:
			val_4byt2(0,0,0,pwr_max_loc);
			break;
		  case 0x00636203:
			val_4byt2(0,0,0,trans_max_loc);
			break;
		  case 0x00636204:
			val_4byt2(0,0,0,other_max_loc);
			break;
		  case 0x00636205:
			val_4byt2(0,0,0,nonroll_max_loc);
			break;
		  case 0x00636206:
			val_4byt2(0,0,0,control_max_loc);
			break;
		  case 0x00636207:
			val_4byt2(0,0,0,Diagnostics_max_loc);
			break;
		default:
			fill_0b();
			
		}
		
		break;
		
	case 0x0802:
		if(obis_code[0]== 0 && obis_code[1]==0  && obis_code[2]==1 && obis_code[3]==0  && obis_code[4]==0)
		date_time(dt.day,dt.month,dt.year,dt.hour,dt.min,dt.sec,0x03);
		else
		fill_0b();
		break;
		
	case 0x0803:
		if(obis_code[0]== 0 && obis_code[1]==0  && obis_code[2]==1 && obis_code[3]==0  && obis_code[4]==0)
		integer16(0x01,0x4a);
		else
		fill_0b();
		break;
		
	case 0x0804:
		if(obis_code[0]== 0 && obis_code[1]==0  && obis_code[2]==1 && obis_code[3]==0  && obis_code[4]==0)
		unsigned8(rtc_status_byte,1);
		else
		fill_0b();
		break;
	case 0x0805:
	case 0x0806:
	case 0x0807:
	case 0x0808:
		fill_0d();
		break;
	case 0x0809:
		if(obis_code[0]== 0 && obis_code[1]==0  && obis_code[2]==1 && obis_code[3]==0  && obis_code[4]==0)
		enum_d2(0x01);
		else
		fill_0b();
		break;
		
	case 0x0f02:
	case 0x0f03:
	case 0x0f04:
	case 0x0f05:
	case 0x0f06:
	case 0x0f07:
	case 0x0f08:
	case 0x0f09:
		if(obis_code[4] == 3 && asso2_flag != 1 )
		ident=0;
		if(obis_code[4] == 2 && asso1_flag != 1)
		ident=0;
		
		switch(obis1)
		{
		case 0x00280000:
			//		  case 0x00280001:
		case 0x00280002:
		case 0x00280003:
			
			if(ident == 1)
			{
				switch(attribute_id)
				{
				case 2://object_list
					i_dlms=1;
					object_list();
					break;
					
				case 3://partners_id
					info[k++]=0;
					structure(2);
					switch(obis_code[4])
					{
					case 0:
						if(asso3_flag==1)
						integer8(0x40);
						else if(asso2_flag==1)
						integer8(0x30);
						else if(asso1_flag==1)
						integer8(0x20);
						else
						integer8(0x10);
						break;
						
					case 1:
						integer8(0x10);
						break;
						
					case 2:
						integer8(0x20);
						break;
						
					case 3:
						integer8(0x30);
						break;
						
					}
					val_2byt(0x00,0x01);
					break;
					
				case 4://application_context_name
					fill_info(auth_fill);
					fill_info(app_con);
					break;
					
				case 5://xDLMS_context_info
					fill_info(XDLMS_TYPE_CONST);
					switch(obis_code[4])
					{
					  case 0:	// Current Association
						info[k-15]=conf_blk[0]; //0x00;//conformance[0]
						info[k-14]=conf_blk[1]; //0x10;//conformance[1]
						info[k-13]=conf_blk[2]; //0x10;//conformance[3]
						unionTemp.u16Temp=max_info_rec;
						info[k-11]=unionTemp.a8Temp[1];
						info[k-10]=unionTemp.a8Temp[0];
						unionTemp.u16Temp=max_info_tra;
						info[k-8]=unionTemp.a8Temp[1];
						info[k-7]=unionTemp.a8Temp[0];
						break;
						
					  case 2:	// MR Association
							info[k-15]=0x00;
							info[k-14]=0x10;
							info[k-13]=0x14;
						break;							
					  case 3:	// US Association
							info[k-15]=0x00;
							info[k-14]=0x18;
							info[k-13]=0x1d;
						break;

					}
					if(obis_code[4]!=0)
					{
						unionTemp.u16Temp=DLMS_MAX_BUFF_SIZE;
						info[k-11]=unionTemp.a8Temp[1];
						info[k-10]=unionTemp.a8Temp[0];
						unionTemp.u16Temp=DLMS_MAX_BUFF_SIZE;
						info[k-8]=unionTemp.a8Temp[1];
						info[k-7]=unionTemp.a8Temp[0];
					}
					break;					
				case 6://authentication mech_name
					auth_name();
					break;
					
				case 7:
					fill_0d();
					break;
					
				case 8://association status
					asso_status();
					break;
					
				case 9: /* security_setup_reference */
					octet_s(6,1);
					info[k++]=0x00;
					info[k++]=0x00;
					info[k++]=0x2B;
					info[k++]=0x00;
                    if(0x00280000==obis_short)
                    {					
						if(asso1_flag==1)
						{
							info[k++]=0x02;
						}
						else if(asso2_flag==1)
						{
							info[k++]=0x03;
						}
					}
					else 
                    {
                        info[k++]=obis_code[4];
                    }					
					info[k++]=0xFF;
					break;
					
				default:
					fill_0b();
					break;
				}
				
			}
			else
			{
				if((attribute_id<9)&&(attribute_id>0))
				fill_0d();
				else
				fill_0b();
			}
			break;
		default:
			fill_0b();
			
		}
		break;
	case 0x1102:
		switch (obis1)
		{
		case 0x00290000:
#if LDN == 0
			fill_info(sap_assgn_list);// sap__assg_list();//
#else
			fill_info(sap_assgn_list_GIL);// sap__assg_list();//			
#endif
			break;
		default:
			fill_0b();
			
		}
		break;
	case 0x1402:
	case 0x1406:
		switch (obis1)
		{
		case 0x000D0000:
			if(classatt == 0x1402)
			{
				if(TOD_bActive_Calendar == 0)
				Eprom_Read(TOU_CAL_ACTIVE_ADD);
				else if(TOD_bActive_Calendar == 1)
				Eprom_Read(TOU_CAL_PASSIVE_ADD);
			}
			else if(classatt == 0x1406)
			{
				if(TOD_bActive_Calendar == 0)
				Eprom_Read(TOU_CAL_PASSIVE_ADD);
				else if(TOD_bActive_Calendar == 1)
				Eprom_Read(TOU_CAL_ACTIVE_ADD);
			}
			
			octet_s(opr_data[0],1);
			memcpy(&info[k],opr_data+1,opr_data[0]);
			k+=opr_data[0];
			break;
		default:
			fill_0b();
			
		}
		break;
	case 0x1403:
	case 0x1407:
		switch (obis1)
		{
		case 0x000D0000:
			if(classatt == 0x1403)
			{
				addpg=0x00+0x40*TOD_bActive_Calendar;
			}
			else if(classatt == 0x1407)
			{
				if(TOD_bActive_Calendar == 0)
				addpg=0x40;
				else if(TOD_bActive_Calendar == 1)
				addpg=0x00;
			}
			Eprom_Read(TOU_CAL_ACTIVE_ADD+0x10+addpg);
			array(opr_data[0],1);
			if(opr_data[0])
			{
				structure(3);	//SEASON 1
				octet_s(7,0);
				memcpy(&info[k],opr_data+1,7);
				k+=7;
				Eprom_Read(TOU_CAL_ACTIVE_ADD+0x20+addpg);
				date_time(opr_data[4],opr_data[3],opr_data[2],opr_data[1],opr_data[0],0x00,0);
				Eprom_Read(TOU_CAL_ACTIVE_ADD+0x30+addpg);
				octet_s(7,0);
				memcpy(&info[k],opr_data,7);
				k+=7;
				Eprom_Read(TOU_CAL_ACTIVE_ADD+0x10+addpg);
				if(opr_data[0]==2)
				{
					structure(3);	//SEASON2
					octet_s(7,0);
					memcpy(&info[k],&opr_data[8],7);
					k+=7;
					Eprom_Read(TOU_CAL_ACTIVE_ADD+0x20+addpg);
					date_time(opr_data[9],opr_data[8],opr_data[7],opr_data[6],opr_data[5],0x00,0);
					Eprom_Read(TOU_CAL_ACTIVE_ADD+0x30+addpg);
					octet_s(7,0);
					memcpy(&info[k],&opr_data[7],7);
					k+=7;
				}
			}
			break;
		default:
			fill_0b();
		}
		break;
	case 0x1404:
	case 0x1408:
		switch (obis1)
		{
		case 0x000D0000:
			if(classatt == 0x1404)
			{
				addpg=0x00+0x40*TOD_bActive_Calendar;
			}
			else if(classatt == 0x1408)
			{
				if(TOD_bActive_Calendar == 0)
				addpg=0x40;
				else if(TOD_bActive_Calendar == 1)
				addpg=0x00;
			}
			Eprom_Read(TOU_WEEK_ACTIVE_ADD+addpg);
			array(opr_data[14],1);
			if(opr_data[14])
			{
				structure(8);
				octet_s(7,0);
				memcpy(&info[k],opr_data,7);
				k+=7;
				Eprom_Read(TOU_WEEK_ACTIVE_ADD+0X10+addpg);
				for(j_g=0;j_g<7;j_g++)
				{
					unsigned8(opr_data[j_g],0);
				}
				Eprom_Read(TOU_WEEK_ACTIVE_ADD+0X20+addpg);
				if(opr_data[14]==2)
				{
					
					structure(8);
					octet_s(7,0);
					memcpy(&info[k],opr_data,7);
					k+=7;
					Eprom_Read(TOU_WEEK_ACTIVE_ADD+0X30+addpg);
					for(j_g=0;j_g<7;j_g++)
					{
						unsigned8(opr_data[j_g],0);
					}
				}
			}
			break;
		default:
			fill_0b();
		}
		break;
	case 0x1405:
	case 0x1409:
		switch (obis1)
		{
		case 0x000D0000:
			if(classatt == 0x1405)
			{
				day_profile(TOD_bActive_Calendar);
			}
			else if(classatt == 0x1409)
			{
				if(TOD_bActive_Calendar)
				day_profile(0);
				else
				day_profile(1);
			}
			break;
		default:
			fill_0b();
		}
		break;
	case 0x140A:
		if(obis_code[0]== 0 && obis_code[1]==0  && obis_code[2]==13  && obis_code[3]==0  && obis_code[4]==0)
		{
			Eprom_Read(TOU_PassiveApliDate);
			date_time(opr_data[2],opr_data[1],opr_data[0],opr_data[3],opr_data[4],0,1);
		}
		else
		fill_0b();
		break;
	case 0x1602:
		switch(obis1)
		{
		case 0x000f0000:
			fill_info(SINGLE_ACTION_BILL_ATT2);
			break;
		case 0x000f0002:
			fill_info(SINGLE_ACTION_FW_ATT2);
			break;
		default:
			fill_0b();
		}
		break;
	case 0x1603:
		switch(obis1)
		{
		case 0x000f0000:
			enum_d2(0x01);
			break;
		case 0x000f0002:
			enum_d2(0x01);
			break;
			
		default:
			fill_0b();
		}
		break;
		
	case 0x1604:
		switch(obis1)
		{
		case 0x000f0000:
			fill_info(SINGLE_ACTION_ATT4);
			octet_s(4,0);
			load_time(BILL_a8date_array[1],BILL_a8date_array[0],0);
			octet_s(5,0);
			load_date(BILL_a8date_array[4],0xff,0xff,0xff);
			info_l5();
			break;
		case 0x000f0002:
			fill_info(SINGLE_ACTION_ATT4);
			Eprom_Read(0x2690);
			octet_s(4,0);
			load_time(0,0,0);
			octet_s(5,0);
			load_date(opr_data[3],opr_data[2],opr_data[1],0xff);
			info_l5();
			break;
		default:
			fill_0b();
		}
		break;
		
		//	  case 0x1b02:
		//		switch(obis1)
		//		{
		//		  case 0x00020000:
		//			enum_d2(0x05);
		//			break;
		//		  default:
		//			fill_0b();
		//		}
		//		break;
		//        
		//	  case 0x1b03:
		//		switch(obis1)
		//		{
		//		  case 0x00020000:
		//			array(1,1);
		//			structure(3);
		//			octet_s(1,0);
		//			info[k++]=0;
		//			octet_s(1,0);
		//			info[k++]=0;
		//			val_2byt(0,0);
		//			break;
		//		  default:
		//			fill_0b();
		//		}
		//		break;
		//	  case 0x1b04:
		//		switch(obis1)
		//		{
		//		  case 0x00020000:
		//			array(1,1);
		//			octet_s(1,0);
		//			info[k++]=0;
		//			break;
		//		  default:
		//			fill_0b();
		//		}
		//		break;
		
		//	  case 0x1c02:
		//		switch(obis1)
		//		{
		//		  case 0x00020200:
		//			enum_d2(0);
		//			break;
		//		  default:
		//			fill_0b();
		//		}
		//		break;
		
		//	  case 0x2802:
		//		switch(obis1)
		//		{
		//		  case 0x00190900:
		//			//Meter Sr. No, Date, Daily Cum kWh & kVAh, Daily MD kW, No of Power failure,
		//			//Tamper occurred status byte, Alerts occurred status byte, Transaction status,
		//			//load survey kW, kVA, V & I with D&T, Relay Status
		//			array(4,1);
		//            
		//			structure(4);
		//			val_2byt(0x00,0x01);//Meter Sr. No
		//			obiscode(0,0,96,1,0,255);
		//			integer8(2);
		//			val_2byt(0x00,0x00);
		//            
		//			structure(4);
		//			val_2byt(0x00,0x3f);//firmware ver
		//			obiscode(1,0,0,2,0,255);
		//			integer8(2);
		//			val_2byt(0x00,0x00);
		//            
		//			structure(4);
		//			val_2byt(0x00,0x07);//Daily energy
		//			obiscode(1,0,99,2,0,255);
		//			integer8(2);
		//			val_2byt(0xE0,0x03);
		//            
		//			structure(4);
		//			val_2byt(0x00,0x07);//load survey
		//			obiscode(1,0,99,1,0,255);
		//			integer8(2);
		//			val_2byt(0xE0,0x03);
		//			break;
		//		  case 0x00190901:
		//			//Meter Sr. No, Date, Cum kWh & kVAh, MD kW & kVA with D&T, Avg PF,
		//			//TOU wise Cum kWh & kVAh, TOU wise MD kW & kVA with D&T,
		//			//Billing period tamper count, billing Reset count,
		//			//billing POH
		//			array(3,1);
		//            
		//			structure(4);
		//			val_2byt(0x00,0x01);//Meter Sr. No
		//			obiscode(0,0,96,1,0,255);
		//			integer8(2);
		//			val_2byt(0x00,0x00);
		//            
		//			structure(4);
		//			val_2byt(0x00,0x3f);//firmware ver
		//			obiscode(1,0,0,2,0,255);
		//			integer8(2);
		//			val_2byt(0x00,0x00);
		//            
		//			structure(4);
		//			val_2byt(0x00,0x07);
		//			obiscode(1,0,98,1,0,255);
		//			integer8(2);
		//			val_2byt(0x10,0x01);
		//			break;
		//		  case 0x00190902:
		//			///Meter Sr No, RTC, Cum kWh, Inst V & I, Tamper occurred status byte, Alerts occurred status byte, Transaction status
		//			array(6,1);
		//            
		//			structure(4);
		//			val_2byt(0x00,0x01);//Meter Sr. No
		//			obiscode(0,0,96,1,0,255);
		//			integer8(2);
		//			val_2byt(0x00,0x00);
		//            
		//			structure(4);
		//			val_2byt(0x00,0x3f);//firmware ver
		//			obiscode(1,0,0,2,0,255);
		//			integer8(2);
		//			val_2byt(0x00,0x00);
		//            
		//			structure(4);
		//			val_2byt(0x00,0x03);
		//			obiscode(1,0,1,8,0,255);
		//			integer8(2);
		//			val_2byt(0x00,0x00);
		//            
		//			structure(4);
		//			val_2byt(0x00,0x03);//phase voltage
		//			obiscode(1,0,12,7,0,255);
		//			integer8(2);
		//			val_2byt(0x00,0x00);
		//            
		//			structure(4);
		//			val_2byt(0x00,0x03);//current
		//			obiscode(1,0,11,7,0,255);
		//			integer8(2);
		//			val_2byt(0x00,0x00);
		//            
		//            
		//			structure(4);
		//			val_2byt(0x00,0x3f);//tamper,alert,transaction status
		//			obiscode(0,0,96,50,3,255);
		//			integer8(2);
		//			val_2byt(0x00,0x00);
		//			break;
		//		  case 0x00190903://test GPRS connection
		//		    //Meter Sr. No., RTC, Cum kWh, RSSI
		//			array(4,1);
		//            
		//			structure(4);
		//			val_2byt(0x00,0x01);//Meter Sr. No
		//			obiscode(0,0,96,1,0,255);
		//			integer8(2);
		//			val_2byt(0x00,0x00);
		//            
		//			structure(4);
		//			val_2byt(0x00,0x3f);//firmware ver
		//			obiscode(1,0,0,2,0,255);
		//			integer8(2);
		//			val_2byt(0x00,0x00);
		//            
		//			structure(4);
		//			val_2byt(0x00,0x03);
		//			obiscode(1,0,1,8,0,255);
		//			integer8(2);
		//			val_2byt(0x00,0x00);
		//            
		//			structure(4);
		//			val_2byt(0x00,0x03);
		//			obiscode(0,0,96,12,5,255);//RSSI
		//			integer8(2);
		//			val_2byt(0x00,0x00);
		//            
		//			break;
		//		  case 0x00190904://test on SMS connection
		//			//Meter Sr. No., RTC, Cum kWh, RSSI
		//			array(4,1);
		//            
		//			structure(4);
		//			val_2byt(0x00,0x01);//Meter Sr. No
		//			obiscode(0,0,96,1,0,255);
		//			integer8(2);
		//			val_2byt(0x00,0x00);
		//            
		//			structure(4);
		//			val_2byt(0x00,0x3f);//firmware ver
		//			obiscode(1,0,0,2,0,255);
		//			integer8(2);
		//			val_2byt(0x00,0x00);
		//            
		//			structure(4);
		//			val_2byt(0x00,0x03);
		//			obiscode(1,0,1,8,0,255);
		//			integer8(2);
		//			val_2byt(0x00,0x00);
		//            
		//			structure(4);
		//			val_2byt(0x00,0x03);
		//			obiscode(0,0,96,12,5,255);//RSSI
		//			integer8(2);
		//			val_2byt(0x00,0x00);
		//			break;
		//            
		//			//		  case 0x00190905:
		//			//			///Others
		//			// 			//Relay  Malfunction, Memory Fail, RTC Fail,Tamper Condition
		//			//			array(1,1);
		//			//			structure(4);
		//			//			val_2byt(0x00,0x03);
		//			//			obiscode(1,0,15,8,0,255);
		//			//			integer8(2);
		//			//			val_2byt(0x00,0x00);
		//			//			break;
		//			//		  case 0x00190906:
		//			//			//Overload Lockout occurrence (TBD)
		//			//			array(1,1);
		//			//			structure(4);
		//			//			val_2byt(0x00,0x03);
		//			//			obiscode(1,0,15,8,0,255);
		//			//			integer8(2);
		//			//			val_2byt(0x00,0x00);
		//			//			break;
		//            
		//		  default:
		//			fill_0b();
		//		}
		//		break;
		
		//	  case 0x2803:
		//		switch(obis1)
		//		{
		//		  case 0x00190900:
		//			info[k++]=0;
		//			structure(3);
		//			enum_d(0);  //connection type
		//			octet_s(6,0); //Dest IP address
		//			Eprom_Read(PUS_AuditIPPortAddr);
		//			info[k++]=opr_data[0];
		//			info[k++]=opr_data[1];
		//			info[k++]=opr_data[2];
		//			info[k++]=opr_data[3];
		//			info[k++]=opr_data[4];
		//			info[k++]=opr_data[5];
		//			enum_d(0);  //message type
		//			break;
		//		  case 0x00190901:
		//			info[k++]=0;
		//			structure(3);
		//			enum_d(0);  //connection type
		//			octet_s(6,0); //Dest IP address
		//			Eprom_Read(PUS_BillIPPortAddr);
		//			info[k++]=opr_data[0];
		//			info[k++]=opr_data[1];
		//			info[k++]=opr_data[2];
		//			info[k++]=opr_data[3];
		//			info[k++]=opr_data[4];
		//			info[k++]=opr_data[5];
		//			enum_d(0);  //message type
		//			break;
		//		  case 0x00190902:
		//			info[k++]=0;
		//			structure(3);
		//			enum_d(0);  //connection type
		//			octet_s(6,0); //Dest IP address
		//			Eprom_Read(PUS_AlertIPPortAddr);
		//			info[k++]=opr_data[0];
		//			info[k++]=opr_data[1];
		//			info[k++]=opr_data[2];
		//			info[k++]=opr_data[3];
		//			info[k++]=opr_data[4];
		//			info[k++]=opr_data[5];
		//			enum_d(0);  //message type
		//			break;
		//		  case 0x00190903:
		//			info[k++]=0;
		//			structure(3);
		//			enum_d(0);  //connection type
		//			octet_s(6,0); //Dest IP address
		//			Eprom_Read(PUS_RegIPPortAddr);
		//			info[k++]=opr_data[0];
		//			info[k++]=opr_data[1];
		//			info[k++]=opr_data[2];
		//			info[k++]=opr_data[3];
		//			info[k++]=opr_data[4];
		//			info[k++]=opr_data[5];
		//			enum_d(0);  //message type
		//			break;
		//		  case 0x00190904:
		//			info[k++]=0;
		//			structure(3);
		//			enum_d(0);  //connection type
		//			octet_s(13,0); //Dest IP address
		//			Eprom_Read(0x2960);
		//			//for(i_g=0;i_g<13;i_g++)
		//            //info[k++]=opr_data[i_g];
		//			memcpy(&info[k],opr_data,13);
		//			k+=13;
		//			enum_d(1);  //message type
		//			break;
		//      case 0x00190905:
		//			info[k++]=0;
		//			structure(3);
		//			enum_d(0);  //connection type
		//			octet_s(6,0); //Dest IP address
		//			Eprom_Read(PUS_InstantIPPortAddr);
		//			info[k++]=opr_data[0];
		//			info[k++]=opr_data[1];
		//			info[k++]=opr_data[2];
		//			info[k++]=opr_data[3];
		//			info[k++]=opr_data[4];
		//			info[k++]=opr_data[5];
		//			enum_d(0);  //message type
		//			break;
		//            
		//		  default:
		//			fill_0b();
		//		}
		//		break;
		
		//	  case 0x2804:
		//		switch(obis1)
		//		{
		//		  case 0x00190900:
		//			Eprom_Read(0x2910);
		//			array(opr_data[0],1);//communication window
		//			if(opr_data[0]!=0)
		//			{
		//				structure(2);
		//				date_time(0xff,0xff,0xff,opr_data[2],opr_data[3],0x00,0);
		//				if(opr_data[0]==2)
		//				{
		//					structure(2);
		//					date_time(0xff,0xff,0xff,opr_data[4],opr_data[5],0x00,0);
		//				}
		//			}
		//			break;
		//		  case 0x00190901:
		//			array(0,1);//communication window
		//			break;
		//		  case 0x00190902:
		//			array(0,1);//communication window
		//			break;
		//		  case 0x00190903:
		//			array(0,1);//communication window
		//			break;
		//		  case 0x00190904:
		//			array(0,1);//communication window
		//			break;
		//            
		//		  default:
		//			fill_0b();
		//		}
		//		break;
		//        
		//	  case 0x2805:
		//		switch(obis1)
		//		{
		//		  case 0x00190900:
		//			val_2byt2(0,0);//randomisation interval
		//			break;
		//		  case 0x00190901:
		//			val_2byt2(0,0);//randomisation interval
		//			break;
		//		  case 0x00190902:
		//			val_2byt2(0,0);//randomisation interval
		//			break;
		//		  case 0x00190903:
		//			val_2byt2(0,0);//randomisation interval
		//			break;
		//		  case 0x00190904:
		//			val_2byt2(0,0);//randomisation interval
		//			break;
		//            
		//            
		//		  default:
		//			fill_0b();
		//		}
		//		break;
		//        
		//	  case 0x2806:
		//		switch(obis1)
		//		{
		//		  case 0x00190900:
		//		  case 0x00190901:
		//		  case 0x00190902:
		//		  case 0x00190903:
		//			val_2byt2(0,3);  //no. of retries
		//			break;
		//		  default:
		//			fill_0b();
		//		}
		//		break;
		//        
		//	  case 0x2807:
		//		switch(obis1)
		//		{
		//		  case 0x00190900:
		//		  case 0x00190901:
		//		  case 0x00190902:
		//		  case 0x00190903:
		//		  case 0x00190904:
		//			val_2byt2(0,0);//repetition delay
		//			break;
		//		  default:
		//			fill_0b();
		//		}
		//		break;
		
		//	  case 0x3c02:
		//		switch(obis1)
		//		{
		//		  case 0x00000203://wake up message
		//			array(1,1);
		//			structure(2);
		//			date_time(0xff,0xff,0xff,0xff,0xff,0,0);
		//			date_time(0xff,0xff,0xff,0xff,0xff,0,0);
		//			break;
		//		  default:
		//			fill_0b();
		//		}
		//		break;
		//	  case 0x3c03:
		//		switch(obis1)
		//		{
		//		  case 0x00000203://wake up message
		//			Eprom_Read(0x2a20);
		//            
		//			array(opr_data[0],1);//no. of senders
		//			if(opr_data[0]==2)
		//			{
		//				Eprom_Read(0x2a00);
		//				octet_s(opr_data[0],0);//
		//				memcpy(&info[k],opr_data+1,opr_data[0]);
		//				k+=opr_data[0];
		//				Eprom_Read(0x2a10);
		//				octet_s(opr_data[0],0);//
		//				memcpy(&info[k],opr_data+1,opr_data[0]);
		//				k+=opr_data[0];
		//			}
		//			else if(opr_data[0]==1)
		//			{
		//				Eprom_Read(0x2a00);
		//				octet_s(opr_data[0],0);//
		//				memcpy(&info[k],opr_data+1,opr_data[0]);
		//				k+=opr_data[0];
		//			}
		//			break;
		//		  default:
		//			fill_0b();
		//		}
		//		break;
		//	  case 0x3c04:
		//		switch(obis1)
		//		{
		//		  case 0x00000203:
		//FIXME
		//sender id octet string
		//script
		//script ::= structure
		//{
		//  script_identifier: long-unsigned,
		//  actions: array action_specification
		//}
		
		//			Eprom_Read(0x2a10);
		//			array(opr_data[0],1);
		//			structure(2);
		//
		//			octet_s(5,0);//
		//			info[k++]=opr_data[1];
		//			info[k++]=opr_data[2];
		//			info[k++]=opr_data[3];
		//			info[k++]=opr_data[4];
		//			info[k++]=opr_data[5];
		//
		//			structure(2);
		//			val_2byt();//script identifier
		//
		//			structure(5);//action specification
		//			info[k++]=;
		//			info[k++]=;
		//			info[k++]=;
		//			info[k++]=;
		//			info[k++]=;
		//
		//			octet_s(5,0);//
		//			info[k++]=opr_data[6];
		//			info[k++]=opr_data[7];
		//			info[k++]=opr_data[8];
		//			info[k++]=opr_data[9];
		//			info[k++]=opr_data[10];
		//
		//			structure(2);
		//			val_2byt();//script identifier
		//
		//			structure(5);//action specification
		//			info[k++]=;
		//			info[k++]=;
		//			info[k++]=;
		//			info[k++]=;
		//			info[k++]=;
		//			break;
		//		  default:
		//			fill_0b();
		//		}
		//		break;
		
	case 0x0902:
		{
			switch(obis1)
			{
				//			  case 0x000a006a:
				//                Disconnect_script();
				//				break;
				//			  case 0x000a006b:
				//                Image_script();
				//				break;
				//			  case 0x000a006c:
				//				Push_script();
				//				break;
			case 0x000a0064:
				Tarrif_script();
				break;
				
			}
			
			
			
		}
		break;
		
		
		      case 0x3f02:
				switch(obis1)
				{
		//		  case 0x00603200:
		//			bit_string(0x60,1);
		//			for( i = 14; i > 2; --i)
		//			{
		//				info[k++]=opr_data[i];
		//			}
		//			break;
		          case 0x00600A01:
		            tamper_status();
		            break; 
				  default:
					fill_0b();
				}
				break;
		//	  case 0x3f03:
		//		switch(obis1)
		//		{
		//		  case 0x00603200:
		//			info[k++]=0;
		//			structure(2);
		//			if(TOD_bSeason_f == 0)
		//				unsigned8(0,0);
		//			else
		//				unsigned8(1,0);
		//			array(8,0);
		//			for(j_g=1;j_g<9;j_g++)
		//				val_2byt(0,j_g);
		//		}
		//		break;
	default:
		fill_0b();
		break;
	}
}


void object_list(void)
{
	unsigned int buffer_filled_u16=0,m_element_2fill=0 ; //u16temp;
	u8 u8temp=0;

	if(GPR_u8TCPIP_F==1)
	{
		k=5;
		Start_Info2();
		k=20;
	}
	else
	{
		if((asso0_flag==1))
		{
			k=0;
			Start_Info();
			k=7;
		}
		else
		{
			k=0;
			Start_Info2();
			k=15;
		}
	}
	buffer_filled_u16=k;
	if(buffer_first_not_fill_f==0)
	{
		if((asso2_flag==1))
		{
		array(OBJ_LIST[2],0);
			element_filled=1;
		}
		else if((asso1_flag==1))
		{
		array(OBJ_LIST[1],0);
			element_filled=1;
		}
		else
		{
		array(OBJ_LIST[0],0);
			element_filled=0;
		}
		block_no=1;

		
		multi_filling_f=1;
		buffer_first_not_fill_f=1;
	}

	if(asso1_flag==1)
		m_element_2fill=OBJ_LIST[1]+1;
	else if(asso2_flag==1)
		m_element_2fill=OBJ_LIST[2]+1;
	else
	m_element_2fill=OBJ_LIST[0];

	for(;element_filled<m_element_2fill;element_filled++)
	{
		buffer_filled_u16+=OBJ_LIST[3+element_filled*9];

		class_sel(OBJ_LIST[4+element_filled*9],OBJ_LIST[4+element_filled*9+1],
				  OBJ_LIST[4+element_filled*9+2],OBJ_LIST[4+element_filled*9+3],
				  OBJ_LIST[4+element_filled*9+4],OBJ_LIST[4+element_filled*9+5],
				  OBJ_LIST[4+element_filled*9+6],OBJ_LIST[4+element_filled*9+7]);
		if(DLMS_MAX_BUFF_SIZE<(buffer_filled_u16+OBJ_LIST[3+(element_filled+1)*9]))
		{
			break;
		}
	}
	element_filled++;

	if(element_filled>=(m_element_2fill))
	{
		multi_filling_f=0;
		buffer_first_not_fill_f=0;
		u8temp=1;
	}
	if((asso0_flag==1))
	{
		frame_type=0;//((rrr_s<<5)|(0x10)|(sss_s<<1));
		info_send=k;
		info_sended=0;
		send_type_multi();
	}
	else
	{
	send_data(u8temp);
	}
}
void capture_objects_filler(unsigned char const *p_f)
{
	unsigned int buffer_filled_u16=0;
	u8 u8temp=0;
	if(GPR_u8TCPIP_F==1)
	{
		k=5;
		Start_Info2();
		k=20;
	}
	else
	{
		k=0;
		Start_Info2();
		k=15;
	}
	buffer_filled_u16=k;
	if(buffer_first_not_fill_f==0)
	{
		//		Start_Info();
		array(p_f[0],0);
		block_no=1;

		element_filled=0;
		multi_filling_f=1;
		buffer_first_not_fill_f=1;
	}
	for(;element_filled<p_f[0];element_filled++)
	{
		buffer_filled_u16+=18;

		profile_sel(p_f[1+(element_filled*8)],p_f[1+(element_filled*8)+1],p_f[1+(element_filled*8)+2],
		p_f[1+(element_filled*8)+3],p_f[1+(element_filled*8)+4],p_f[1+(element_filled*8)+5],
		p_f[1+(element_filled*8)+6],p_f[1+(element_filled*8)+7]);
		if(DLMS_MAX_BUFF_SIZE<(buffer_filled_u16+18))
		{
			break;
		}
	}
	element_filled++;
	if(element_filled>=p_f[0])
	{
		multi_filling_f=0;
		buffer_first_not_fill_f=0;
		u8temp=1;
	}
	send_data(u8temp);
}
void buffer_scaler_filler(unsigned char const *s_f)
{
	unsigned int buffer_filled_u16=0;
	u8 u8temp=0;
	if(GPR_u8TCPIP_F==1)
	{
		k=5;
		Start_Info2();
		k=20;
	}
	else
	{
		k=0;
		Start_Info2();
		k=15;
	}
	buffer_filled_u16=k;
	if(buffer_first_not_fill_f==0)
	{
		array(0x01,0);//s_f[0];
		structure(s_f[0]);

		block_no=1;
		element_filled=0;
		multi_filling_f=1;
		buffer_first_not_fill_f=1;
	}
	for(;element_filled<s_f[0];element_filled++)
	{
		buffer_filled_u16+=6;

		sca_unit(s_f[1+element_filled*2],s_f[1+element_filled*2+1],0);
		if(DLMS_MAX_BUFF_SIZE<(buffer_filled_u16+6))
		{
			break;
		}
	}
	element_filled++;
	if(element_filled>=s_f[0])
	{
		multi_filling_f=0;
		buffer_first_not_fill_f=0;
		u8temp=1;
	}
	send_data(u8temp);
}
void tamper_compart(unsigned char event,unsigned int init_add,unsigned int max_add)
{
	unsigned int buffer_filled_u16=0;
	u8 u8temp=0;

	if(GPR_u8TCPIP_F==1)
	{
		k=5;
		Start_Info2();
		k=20;
	}
	else
	{
		k=0;
		Start_Info2();
		k=15;
	}
	buffer_filled_u16=k;


	if(buffer_first_not_fill_f==0)
	{
		tamper_data=opr_data[2];
		if(opr_data[1]==1)
		{
			compart1=event;
			tamper_data=opr_data[2];
		}
		else
		{
			compart1 = tamper_data;
			tamper_data = 0;
		}

		if(sel_access_flag==1)
		{
			if((compart1<to_ptr)||(compart1<from_ptr) || access_selector != 2)
			compart1=0;
			else
			{
				if(to_ptr==0)
				compart1=compart1-from_ptr+1;
				else
				compart1= to_ptr-from_ptr+1;
				if(from_ptr>to_ptr && to_ptr!=0)
				compart1=0;
				if(from_ptr==0)
				compart1=0;

				tamper_data = tamper_data + from_ptr-1;

				if(tamper_data>event)
				tamper_data = tamper_data-event;
			}
		}

		//		Start_Info();
		dlms_address = init_add+((u16)tamper_data*0x0020);
		//array(compart1,0);
		info[k++]=0x01;
		if(compart1>128)
		{
			info[k++]=0x82;
			info[k++]=0x00;
		}
		info[k++]=compart1;
		block_no=1;

		element_filled=0;
		multi_filling_f=1;
		buffer_first_not_fill_f=1;

		//		if(obis_code[4] != 6)
		//		{
		if(sel_access_flag==1)
		{
			selective_values_byte=2+sel_obj_tamper[0]*14+
			sel_obj_tamper[1]*3+
			sel_obj_tamper[2]*5+
			sel_obj_tamper[3]*3+
			sel_obj_tamper[4]*3+
			sel_obj_tamper[5]*5+
			sel_obj_tamper[6]*5;
		}
		else if(sel_access_flag1 == 1)
		{
			selective_values_byte=2+sel_obj_tamper[0]*14+
			sel_obj_tamper[1]*3;
		}
		else
		{
			selective_values_byte=40;
		}

		if(compart1==0)
		goto comp1;
	}
	for(;element_filled<compart1;element_filled++)
	{
		buffer_filled_u16+=selective_values_byte;

		if(dlms_address == max_add)
		dlms_address = init_add;

		tpr_fill(dlms_address);
		dlms_address = dlms_address +0x20;
		if(DLMS_MAX_BUFF_SIZE<(buffer_filled_u16+selective_values_byte))
		{
			break;
		}
	}

comp1:
	element_filled++;
	if(element_filled>=compart1)
	{
		multi_filling_f=0;
		buffer_first_not_fill_f=0;
		u8temp=1;
	}
	send_data(u8temp);
}

void buffer_NamePlateDetails_parameter(void)
{
	static union
	{
		u8 a8Temp[2];
		u16 u16Temp;
	}unionTemp;
	u16 i;
	u8 u8temp=0;
	//    signed char temp1;
	if(GPR_u8TCPIP_F==1)
	{
		k=5;
		Start_Info2();
		k=20;
	}
	else
	{
		k=0;
		Start_Info2();
		k=15;
	}
	block_no=1;
	array(1,0);
	
	structure(NamePlateDetails_parameter_cap_obj[0]);
	
	sr_no_ascii();              //serial no

#if LDN == 0
	for(i =2; i <=manufacturer_name[0]; ++i)                    //manufacturer name. 
	{
		info[k++] = manufacturer_name[i];
	}
#else
	for(i =2; i <=manufacturer_name_GIL[0]; ++i)                    //manufacturer name. 
	{
		info[k++] = manufacturer_name_GIL[i];
	}
#endif
	fill_firmware_version();            //firmware version
	
	unsigned8(5,0);//    meter_type()                  //meter type
	
	info[k++] = 0x0a;           //unsigned8(0xc3,0);//    meter_catagory()                 //meter catagory
	info[k++] = 0x02;
	info[k++] = 0x43;//ascii for 'C'
	info[k++] = 0x33;//ascii for '3'
	
	info[k++] = 0x0a;
	Eprom_Read(CurrentRating_addrs); 
	info[k++] = opr_data[0];
	for(i=1; i<=opr_data[0];++i)
	{
		info[k++] = opr_data[i];  
	}

//	info[k++] = 0x04;
//	info[k++] = 0x35;
//	info[k++] = 0x2d;
//	info[k++] = 0x33;
//	info[k++] = 0x30;
	
	Eprom_Read(FG_DateTimeAdd);
	unionTemp.u16Temp=2000+bcd_to_hex(opr_data[2]);
	info[k++]=0x12;
	info[k++]=unionTemp.a8Temp[1];
	info[k++]=unionTemp.a8Temp[0];         //2012 year of manufacturing
	
	
	multi_filling_f=0;
	buffer_first_not_fill_f=0;
	u8temp=1;
	send_data(u8temp);
	
	
}
void buffer_instantaneous_parameter(void)
{
	u8 u8temp=0,sec=0;
	signed char temp1;
	if(GPR_u8TCPIP_F==1)
	{
		k=5;
		Start_Info2();
		k=20;
	}
	else
	{
		k=0;
		Start_Info2();
		k=15;
	}
	block_no=1;
	array(1,0);

	structure(instantaneous_parameter_cap_obj[0]);
	date_time(dt.day,dt.month,dt.year,dt.hour,dt.min,dt.sec,2);
	
	if(MET_u32ip_rms<=9)
	{
		MET_u32ip_rms=0;
		MET_u32ip_rms_tamp=0;
	}
	
	if(MET_u32in_rms<=9)
	MET_u32in_rms=0;

	val_2byt(MET_u16v_rms  /256,MET_u16v_rms  %256);    

	FUN_vfill_3byteR(MET_u32ip_rms,&(opr_data[2]));
	val_4byt(0x00,opr_data[0],opr_data[1],opr_data[2]);

	FUN_vfill_3byteR(MET_u32in_rms,&(opr_data[2]));
	val_4byt(0x00,opr_data[0],opr_data[1],opr_data[2]);

	

	val_2byt_signed(MET_u16sign_net_pf/256,MET_u16sign_net_pf%256); //signed pf

	val_2byt(MET_u16freq/256,MET_u16freq%256);

	val_2byt(MET_u16Kva/256,MET_u16Kva%256);//apparent power


	if(MET_bph_ct_f1 == 0)
	val_2byt_signed(MET_u16signed_Kw/256,MET_u16signed_Kw%256);//signed active power
	else
	val_2byt_signed(MET_u16signed_ELKw/256,MET_u16signed_ELKw%256);//signed active power

	FUN_vfill_3byteR(MET_u32Cum_kwh,&(opr_data[2]));
	val_4byt(0x00,opr_data[0],opr_data[1],opr_data[2]);  //kWh

	FUN_vfill_3byteR(MET_u32Cum_kvah,&(opr_data[2]));
	val_4byt(0x00,opr_data[0],opr_data[1],opr_data[2]);  //kVAh	
	
	
	temp1=calmdpg(BILL_u8mdmonth);   
	Eprom_Read(billmd_data_addrs+(unsigned char)temp1);
	val_4byt(0x00,0x00,opr_data[0],opr_data[1]); //md kW 
	
     if((opr_data[2]==0xff)&&(opr_data[3]==0xff))
     {
	     sec=0xff;
     }
	 
	date_time(opr_data[6],opr_data[5],opr_data[4],opr_data[3],opr_data[2],sec,0);//md d&t

	val_4byt(0x00,0x00,opr_data[7],opr_data[8]); //md kVA
	
	sec=0;
	
	if((opr_data[9]==0xff)&&(opr_data[10]==0xff))
	{
		sec=0xff;
	}
	
	date_time(opr_data[13],opr_data[12],opr_data[11],opr_data[10],opr_data[9],sec,0);//md kVA d&t   


	//    val_2byt(0,on_off_cnt);//no. of power failures

	FUN_vfill_3byteR(cum_pow_on,&opr_data[2]);//cumulative power failure duration
	val_4byt(0,opr_data[0],opr_data[1],opr_data[2]);

	FUN_vfill_2byteR(TPR_u16cum_tpr_c,&opr_data[1]);
	val_2byt(opr_data[0],opr_data[1]);// cumulative tamper count
	
	//unsigned8(BILL_u8md_count,0);// billing count
	val_4byt(0x00,0x00,0x00,BILL_u8md_count);
	
	val_2byt(0,trans_count); //cumulative prog count
	
	FUN_vfill_3byteR(MET_u32Cum_kvarh_Lag,&(opr_data[2]));
	val_4byt(0x00,opr_data[0],opr_data[1],opr_data[2]);  //Q1-Kvarh
	
	FUN_vfill_3byteR(MET_u32Cum_kvarh_Lead,&(opr_data[2]));
	val_4byt(0x00,opr_data[0],opr_data[1],opr_data[2]);  //Q2-Kvarh
	
	
	        hi_tempkwh = Compute_EnergyHighres(1);
			FUN_vfill_4byteR(hi_tempkwh,&(opr_data[3]));
			val_4byt(opr_data[0],opr_data[1],opr_data[2],opr_data[3]);
	        hi_tempkvah = Compute_EnergyHighres(2);		
			FUN_vfill_4byteR(hi_tempkvah,&(opr_data[3]));
			val_4byt(opr_data[0],opr_data[1],opr_data[2],opr_data[3]);	
//	FUN_vfill_3byteR(MET_u32Cum_kvarh_Lag,&(opr_data[2]));
//	val_4byt(0x00,opr_data[0],opr_data[1],opr_data[2]);  //kVAh

//	FUN_vfill_3byteR(MET_u32Cum_kvarh_Lead,&(opr_data[2]));
//	val_4byt(0x00,opr_data[0],opr_data[1],opr_data[2]);  //kVAh	
	//    Eprom_Read(Universalmd_data_addrs);
	//    val_4byt(0x00,0x00,opr_data[0],opr_data[1]); //universal md kW 
	//    
	//   
	//    date_time(opr_data[6],opr_data[5],opr_data[4],opr_data[3],opr_data[2],0,0);//universal md d&t
	
	//    val_4byt(0x00,0x00,opr_data[7],opr_data[8]); //universal md kVA
	//    
	//	
	//    date_time(opr_data[13],opr_data[12],opr_data[11],opr_data[10],opr_data[9],0,0);//universal md kVA d&t   

	temp1 =BILL_u8mdmonth-1;
	if(temp1<=0)
	temp1=temp1+MaxBillDate+1;                               
	
	//    temp1=calmdpg(temp1);    
	//    Eprom_Read(billkwh_data_addrs+(unsigned char)temp1);
	//    date_time(opr_data[10],opr_data[9],opr_data[8],opr_data[7],opr_data[6],0,0); //d&t of last MD reset    
	
	multi_filling_f=0;
	buffer_first_not_fill_f=0;
	u8temp=1;
	send_data(u8temp);

}

/*void fill_scalar(unsigned char const *s_f)
{
unsigned char j;

k=0;
seg_type=0xA8;

if(long_data==0)
{
	Start_Info();
	info[8]=0x01;
	k=9;
	structure(s_f[0]);
	sca_fill=0;
	i_dlms=1;
}

for(j=0;(j<8)&&(sca_fill<s_f[0]);i_dlms+=2)
{
	sca_unit(s_f[i_dlms],s_f[i_dlms+1],0);
	j++;
	sca_fill++;
}

if(sca_fill>=s_f[0])
{
	seg_type=0xA0;
	seg_flagsd=0;
}

frame_type=((rrr_s<<5)|(0x10)|(sss_s<<1));
info_len=k;
send_type();
long_data++;
if(seg_type==0xA0)
	long_data=0;
}*/

/*void app_con(void)
{//app_con
	unsigned char i;
	for(i=0;i<14;i++)
			info[k++] = *(auth_fill + i);
	info[k]=0x11;//app_con_ele(unsigned)
	info[k+1]=0x01;//app_con_ele[0]
	info[k+2]=0x11;//context_id_ele(unsigned)
	info[k+3]=0x01;//context_id_ele[0]
	info_l();
}//app_con*/

/*void xdlms_type(void)
{//xdlms_type
unsigned char i;
for(i=0;i<20;i++)
{
	info[k++]=XDLMS_TYPE_CONST[i];
}
info[k++]=0x00;
	info[k++]=0x02;//structure
	info[k++]=0x06;//length of structure
	info[k++]=0x04;//conformance(bit string(24))
	info[k++]=0x18;//length 3
	info[k++]=0x00;//conformance[0]
	info[k++]=0x00;//conformance[1]
	info[k++]=0x18;//conformance[3]
	info[k++]=0x12;//max-rec-PDU-size(long-unsigned)
	info[k++]=0x01;//max-rec-PDU-size[0]
	info[k++]=0xFF;//max-rec-PDU-size[1]
	info[k++]=0x12;//max-send-PDU-size(long-unsigned)
	info[k++]=0x01;//max-send-PDU-size[0]
	info[k++]=0xFF;//max-send-PDU-size[1]
	info[k++]=0x11;//DLMS-version-number(unsigned)
	info[k++]=0x06;//DLMS-version-number[0]
//	info[k++]=0x00;//quality-of-service(null)//optional
	info[k++]=0x0F;//integer8
	info[k++]=0x00;//value
//	info[k++]=0x00;//chypering_info(null)//optional
	info[k++]=0x09;//chypering_info(octet-string)
	info[k++]=0x00;//length

}//xdlms_type*/

void auth_name(void)
{//auth_name
    fill_info(auth_fill);
    unsigned8(0x02,0);
	if(obis_code[4]==0)	// Current Association
	{
		if(asso1_flag==1)	// MR Association
		{
			unsigned8(0x01,0);	//COSEM_low_level_security_mechanism_name ::= mechanism_id(1)
		}
		else if(asso2_flag==1)	// US Association
		{
			unsigned8(0x02,0);		// COSEM_high_level_security_mechanism_name ::= mechanism_id(2)
		}
		else
		{
			unsigned8(0x00,0);	//COSEM_lowest_level_security_mechanism_name ::= mechanism_id(0)
		}
	}
	if(obis_code[4]==2)	// MR Association
	{
		unsigned8(0x01,0);	//COSEM_low_level_security_mechanism_name ::= mechanism_id(1)
	}
	if(obis_code[4]==3)	// US Association
	{
		unsigned8(0x02,0);	// COSEM_high_level_security_mechanism_name ::= mechanism_id(2)
	}
	if(obis_code[4]==1)		// PC Association
	{
		unsigned8(0x00,0);	//COSEM_lowest_level_security_mechanism_name ::= mechanism_id(0)
	}
}//auth_name


void asso_status(void)
{//asso_status
	/*info[k++]=0x00;
	info[k++]=0x16;//enum(0->non_associated,1->association_pending,2->associated)*/


	if(obis_code[4]==0)
	{//ifobis0
		if(asso0_flag==1)
		enum_d2(0x02);//  info[k++]=0x02;
		else
		enum_d2(0x00);//info[k++]=0x00;
	}//ifobis0

	if(obis_code[4]==1)
	{//ifobis0
		if(asso0_flag==1)
		enum_d2(0x02);//  info[k++]=0x02;
		else
		enum_d2(0x00);// info[k++]=0x00;
	}//ifobis0

	if(obis_code[4]==2)
	{//ifobis0
		if(asso1_flag==1)
		enum_d2(0x02);
		else
		enum_d2(0x00);
	}//ifobis0
	if(obis_code[4]==3)
	{//ifobis0
		if(asso2_flag==1)
		enum_d2(0x02);
		else
		enum_d2(0x00);
	}//ifobis0
}//asso_stat`s



void log_name2(unsigned char a,unsigned char b,unsigned char c,unsigned char d,unsigned char e)
{//log_name2
	info[k++]=0x00;
	obiscode(a,b,c,d,e,255);
}//log_name2



/*void sap__assg_list(void)
{				//sap__assg_list
// unsigned char i;
	array(0x03,1);		//length of array
	structure(0x02);	//length of structure
	long_unsign();		//SAP address(long_unsigned)
	info[k++]=0x01;		//SAP address[1]

	octet_s(0x07);			//logical_device_name(octet_string)
	fill_info(LOGICAL_DEVICE_NAME);

	structure(0x02);
	long_unsign();
	info[k++]=0x10;			//SAP address[1]
	octet_s(0x07);
	fill_info(LOGICAL_DEVICE_NAME);
	k=k-1;
	info[k++]='1';

	octet_s(0x07);			//logical_device_name(octet_string)
	fill_info(LOGICAL_DEVICE_NAME);

	structure(0x02);
	long_unsign();
	info[k++]=0x10;			//SAP address[1]
	octet_s(0x07);
	fill_info(LOGICAL_DEVICE_NAME);
	k=k-1;
	info[k++]='2';


	}*/
void unsigned8(unsigned char value,unsigned char flag)/********/
{
	if(flag==1)
	info[k++]=0x00;//data result
	info[k++]=0x11;//object avaliable
	info[k++]=value;//object avaliable
}

/*void unsigned16(unsigned char value1,unsigned char value2,unsigned char flag)
{
		if(flag==1)
		info[k++]=0x00;//data result
	info[k++]=0x12;//object avaliable
	info[k++]=value1;//object avaliable
		info[k++]=value2;

}*/
/*void sort_object()
{
	info[k++]=0x00;
	structure(0x04);
	val_2byt(0x00,0x00);
	obiscode(0x00,0x00,0x00,0x00,0x00,0x00);
	cminfo[k++]=0x0F;
	info[k++]=0x00;cm
		integer8(0x00);
	val_2byt(0x00,0x00);
	info_len=k;
}*/

/*void sort_object()
{
	info[k++]=0x00;
	structure(0x04);
	if(obis_code[2]==99)
	{
			val_2byt2(0x00,0x08);
		obiscode(0x00,0x00,0x01,0x00,0x00,0xff);
		integer8(0x03);
	}
	else
	{
		val_2byt2(0x00,0x00);
		obiscode(0x00,0x00,0x00,0x00,0x00,0x00);
		integer8(0x00);
	}
	val_2byt2(0x00,0x00);
	info_len=k;
}*/
/*void  event_log_profile_buffer_s()
{
		k=0;
	//seg_type=0xA0;
		Start_Info();
		info[8]=0x04;			//page 18 of Kepco doccument
		k=9;
		//sca_unit(0,255,0);
		sca_unit(0xfd,33,0);
		sca_unit(0xfe,35,0);
		sca_unit(0xfd,255,0);
		sca_unit(2,30,0);

		seg_type=0xA0;
		seg_flagsd=0;
	frame_type=((rrr_s<<5)|(0x10)|(sss_s<<1));
	info_len=k;
	send_type();
	long_data++;
	if(seg_type==0xA0)
		long_data=0;
}*/

void class_sel(unsigned int ic,unsigned char ver,unsigned char obis_a,unsigned char obis_b,unsigned char obis_c,unsigned char obis_d,unsigned char obis_e,unsigned char obis_f)
{//class_sel
	structure(0x04);
	info[k]=0x12;//long_unsigned16(Interface Class) structure element 1
	info[k+1]=ic/256;//4
	info[k+2]=ic%256;//5
	info[k+3]=0x11;//unsigned8(version)//structure element 2
	info[k+4]=ver;//7
	info_l5();
	obiscode(obis_a,obis_b,obis_c,obis_d,obis_e,obis_f);
	structure(0x02);
	info[k++]=0x01;//array(attribute access descriptor)

	switch(ic)
	{//swic
	case 1://data class
		if(asso2_flag==1 &&((obis_a == 0x01)&&(obis_c == 0x00)&&(obis_d == 0x08)&&((obis_e == 0x00)||(obis_e == 0x04))))
		access_rights(2,1,3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);//remove access r/w in mdip lsip
		else
		access_rights(2,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
		break;
	case 3://register class
//		if((obis_a == 0x00)&&(obis_c == 0x00)&&(obis_d == 0x01)&&(obis_e == 0x02))
//		access_rights(3,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
//		else
		access_rights(3,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
		break;
	case 4://extended register class
		access_rights(5,1,1,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0);
		break;
		//	  case 5://Demand Register
		//		access_rights(9,1,1,1,1,0,1,1,1,1,0,0,0,0,0,0,0,0);
		//		break;
		//	  case 6://register activation class
		//		//cnt_att=4;
		//		access_rights(4,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0);
		//		break;
	case 7://profile generic class
		if(asso1_flag == 1 || asso2_flag == 1)
		{
			if((obis_a == 0x01)&&(obis_c == 0x63)&&((obis_d == 0x01)||(obis_d == 0x02)))
			access_rights(8,1,1,1,1,1,1,1,1,0,0,0,1,0,0,0,0,0);
			else if((obis_c == 0x63)&&(obis_d == 0x62) && (obis_e != 0x05))
			access_rights(8,1,1,1,1,1,1,1,1,0,0,0,2,0,0,0,0,0);
			else if((obis_a == 0x01)&&(obis_c == 0x62)&&(obis_d == 0x01))
			access_rights(8,1,1,1,1,1,1,1,1,0,0,0,2,0,0,0,0,0);
			else
			access_rights(8,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0);
		}
		else
		access_rights(8,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0);
		break;
	case 8://clock class
		if(asso2_flag==1)
		access_rights(9,1,3,1,1,0,0,0,0,1,0,0,0,0,0,0,0,0);//clock
		else
		access_rights(9,1,1,1,1,0,0,0,0,1,0,0,0,0,0,0,0,0);//clock
		break;
	case 9://script table class
		access_rights(2,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
		break;
		//	  case 11://special days table class
		//		if(asso0_flag==1)
		//			access_rights(2,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);//
		//		else
		//			access_rights(2,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);//
		//		break;
	case 15://association LN class
		if(((asso1_flag == 1)||(asso2_flag == 1 )) && (obis_a == 0x00)&&(obis_c == 0x28)&&(obis_e == 0x01))
		access_rights(9,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
		else if(asso2_flag == 1 && (obis_a == 0x00)&&(obis_c == 0x28)&&(obis_e == 0x02))
		access_rights(9,0,0,0,0,0,0,2,0,0,0,0,0,0,0,0,0,0);
		else if(asso2_flag == 1 && (obis_a == 0x00)&&(obis_c == 0x28)&&(obis_e == 0x03))
		access_rights(9,1,1,1,1,1,1,0,1,1,0,0,0,4,1,1,0,0);
		else
		{
			if((ver==1)&&(asso0_flag!=1))
			{
				access_rights(9,1,1,1,1,1,1,0,1,1,0,0,0,0,0,0,0,0);
			}
			else
			{
				access_rights(9,1,1,1,1,1,1,0,1,0,0,0,0,0,0,0,0,0);
			}
		}
		
		break;
	case 17://SAP assignment class
		access_rights(2,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
		break;
	case 18://image transfer class
		if(asso2_flag == 1 )
		access_rights(7,1,1,1,0,1,1,1,0,0,0,0,0,4,1,1,1,1);
		else
		access_rights(7,1,1,1,0,1,1,1,0,0,0,0,0,0,0,0,0,0);
		break;
		//	  case 19://IEC local port setup
		//		access_rights(9,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0);
		//		break;
	case 20://activity calender class
		if(asso2_flag==1)
		access_rights(10,1,1,1,1,1,3,3,3,3,3,0,0,1,1,0,0,0);
		else
		access_rights(10,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0);
		break;
	case 22://single action schedule class
		if(asso2_flag==1)
		access_rights(4,1,1,1,3,0,0,0,0,0,0,0,0,0,0,0,0,0);
		else
		access_rights(4,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0);
		break;
		//	  case 23://IEC HDLC setup class
		//		if(asso0_flag==1)
		//			access_rights(9,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0);
		//		else
		//			access_rights(9,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0);
		//		break;
		//	  case 27://MODEM configuration
		//		access_rights(4,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0);
		//		break;
		//	  case 28://Auto answer
		//		access_rights(6,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0);
		//		break;
		//	  case 29://Auto connect
		//		if(asso2_flag == 1)
		//			access_rights(6,1,1,3,3,3,3,0,0,0,0,0,0,0,0,0,0,0);
		//		else
		//			access_rights(6,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0);
		//		break;
	case 40://push set up class
		if(asso2_flag == 1)
		access_rights(7,1,3,3,3,3,3,3,0,0,0,0,0,0,0,0,0,0);
		else
		access_rights(7,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0);
		break;
	case 41://TCP/UDP setup class
		if(asso2_flag == 1)
		access_rights(6,1,3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
		else
		access_rights(6,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
		break;
	case 42://IPv4 setup class
		if(asso2_flag == 1)
		access_rights(10,1,0,3,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
		else
		access_rights(10,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
		break;
	case 44://PPP setup class
		if(asso2_flag == 1)
		access_rights(5,1,0,0,0,3,0,0,0,0,0,0,0,0,0,0,0,0);
		else
		access_rights(5,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
		break;
	case 45://GPRS modem setup class
		if(asso2_flag == 1)
		access_rights(4,1,3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
		else
		access_rights(4,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
		break;
	case 60: // Wake-Up using Messages
		if(asso2_flag == 1)
		access_rights(4,1,0,3,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
		else
		access_rights(4,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
		break;
	case 63: // status mapping
		access_rights(3,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
		break;
	case 70://disconnect control
		if(asso2_flag == 1)
		access_rights(4,1,1,0,0,0,0,0,0,0,0,0,0,4,1,1,0,0);
		else
		access_rights(4,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
		break;
	case 71://limiter class
		if((obis_c == 0x11)&&(obis_e == 0x03))//load
		{
			if(asso2_flag == 1)
			access_rights(11,1,1,1,3,3,3,3,3,3,1,1,0,0,0,0,0,0);
			else
			access_rights(11,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0);
		}
		else
		{
			if(asso2_flag == 1)
			access_rights(11,1,1,1,3,0,3,3,0,0,0,0,0,0,0,0,0,0);
			else
			access_rights(11,1,1,1,1,0,1,1,0,0,0,0,0,0,0,0,0,0);
		}
		break;
	}//swic
}//class_sel

void init_dlmsvar(unsigned char flag)
{
	if(flag==1)
	goto init_dlms;
	Cntr_2Min=0;
	p_fbit=0;
	decerr_flag=0;

	seg_flag=0;
	infose_flag=0;

	asserr_flag=0;
	max_info_tra=DLMS_MAX_BUFF_SIZE;
	max_info_rec=DLMS_MAX_BUFF_SIZE;
	max_win_tra=0x01;
	max_win_rec=0x01;
	Data_block=0x01;
	conf_ser_flag=0;
	conf_err_flag=0;
	conf_type_flag=0;
	conf_serror_flag=0;
	conf_blk[0]=0x00;
	conf_blk[1]=0x00;
	conf_blk[2]=0x10;
	seg_flagsd=0;
	Eprom_Read(DLMS_LLS_PassAdd);//cngd
	if((EPROM_bChksum_ok)&&(opr_data[0]!=0))
	{
		memcpy(aut_pswd1,opr_data,15);

		Eprom_Read(DLMS_HLS_Pass1Add);//cngd
		if((EPROM_bChksum_ok)&&(opr_data[0]!=0))
		{
			aut_pswd1[15]=opr_data[0];
		}
		else
		{
			memcpy(aut_pswd1,pswrd1,15);
		}
	}
	else
	{
		memcpy(aut_pswd1,pswrd2,16);
	}
	Eprom_Read(DLMS_HLS_Pass2Add);//cngd
	if((EPROM_bChksum_ok)&&(opr_data[0]!=0))
	{
		memcpy(aut_pswd,opr_data,8);
	}
	else
	{
		memcpy(aut_pswd,pswrd1,8);
	}

	init_dlms:

	multi_filling_f=0;
	send_type_multi_f=0;
	buffer_first_not_fill_f=0;
	element_filled=0;
	long_data=0;
	rrr_c=0;
	rrr_s=0;
	rrr_c1=0;
	sss_c=0;
	sss_s=0;
	sss_c1=0;
	asso0_flag=0;
	asso1_flag=0;
	asso2_flag=0;
	asso3_flag=0;
	infore_flag=0;
	nrm_flag=0;
	cosem_flag=0;
}

void integer8(char value)
{
	info[k++]=0x0F;
	info[k++]=value;
}
void integer16(unsigned char byte1,unsigned char byte2)
{
	info[k++]=0x00;
	info[k++]=0x10;
	info[k++]=byte1;
	info[k++]=byte2;
}


void MASS_READ1(unsigned char eprom_no,unsigned char block_no, unsigned char no_of_block)
{

	unsigned int address1,max_add,epp_page,start_add;

	start_add=block_no;//*0x100;
	max_add=start_add+(no_of_block);//*0x100);

	for(;start_add<max_add;)
	{
//		if(eprom_no == 1)
//		{
			epp_page=0x00;

			for(;epp_page<0x100;)
			{
				if(req_cnt == 0)
				{
					address1=start_add*0x100+epp_page;
					if(eprom_no == 2)
					{
						MEM_Device_add=1;
					}
					else if(eprom_no == 1)
					{
						MEM_Device_add=0;
					}
					
					Eprom_Read(address1);
					trn_buf[0]=address1/256;
					trn_buf[1]=address1%256;

					memcpy(&trn_buf[2],opr_data,16);
					req_cnt=18;
					trn_cnt=0;

					if(IrDA_f==1)
					{
						TXD2 = trn_buf[trn_cnt];// Set transmission data
					}
					else if (optical_f==1)
					{
						TXD0 = trn_buf[trn_cnt];// Set transmission data
					}
					else if(rj_f==1)
					{
						TXD1 = trn_buf[trn_cnt];
					}

					epp_page+=0x10;   

					if(epp_page == 0x100)// && address1 != start_add)
					start_add+=0x01;
					
					R_WDT_Restart();
				}
			}
//		}
//		if(eprom_no == 2)
//		{
//			//			start_add = start_add+(no_of_block-1)*256;
//			//			Flash_read(start_add);
//			//			//for(epp_page=0;epp_page<264;epp_page++)
//			//				//trn_buf[epp_page]=flash_opr_data[epp_page];
//			//			memcpy(trn_buf,flash_opr_data,264);
//			//			req_cnt=264;
//			//			trn_cnt=0;
//			//			UCA0IFG &=~(UCRXIFG);//IFG2 &= 0xfe;
//			//			// P3OUT &= 0x7f;
//			//			UCA0TXBUF = trn_buf[trn_cnt];// Set transmission data
//			//			UCA0IE|=(UCTXIE); //IE2 |= 0x02;  //for optical comm.DOUBT
//			//			max_add=start_add;
//			//			WDTCTL = WDT_ARST_1000;
//		}
	}
	while(1)
	{
		if(req_cnt == 0)
		break;
	}
}
void log_trans_event(unsigned int event)
{
	event_id=event;
	TP.b.TransactionEvent=1;
	trans_count++;
	cum_prog_count++;
	check_tpr();
}

void UART_vResetDlmsData(void)
{
	memcpy(opr_data,pswrd2,15);
	memcpy(aut_pswd1,pswrd2,15);

	Eprom_Write(DLMS_LLS_PassAdd);//cngd
	R_WDT_Restart();

	fill_oprzero();

	opr_data[0]=pswrd2[15];
	aut_pswd1[15]=pswrd2[15];
	Eprom_Write(DLMS_HLS_Pass1Add);//cngd

	memcpy(opr_data,pswrd1,8);
	memcpy(aut_pswd,pswrd1,8);
	Eprom_Write(DLMS_HLS_Pass2Add);//cngd
	R_WDT_Restart();
	
	cum_prog_count=0;
	R_WDT_Restart();
}


void value_monitored(unsigned char identifier)
{

	info[k++]=0;
	structure(3);
	long_unsign();
	info[k++]=0x03;//class id

	if(identifier == 0 || identifier == 1)
	obiscode(1,0,12,7,0,255);//logical name voltage

	if(identifier == 2)
	obiscode(1,0,11,7,0,255);//logical name current

	if(identifier == 3 )
	obiscode(1,0,1,7,0,255);//logical Name load


	info[k++]=0x0f;
	info[k++]=0x02;//attribute monitored

}

void Uart_CurrentBillFill(void)
{
	u8 bill_LoopIndex,sec=0;
	structure(bill_profile_parameter_cap_obj[0]);//41
	
	date_time(dt.day,dt.month,dt.year,dt.hour,dt.min,0x00,0x00);//date_time(255,255,255,0,0,0,0);  //current billing date
	cal_avg_pf();
	
	val_2byt(MET_u16Avg_pf/256, MET_u16Avg_pf%256);   //avg pf
	
	FUN_vfill_3byteR(MET_u32Cum_kwh,&(opr_data[2])); //MET_u32Cum_kwh

	val_4byt(0x00,opr_data[0],opr_data[1],opr_data[2]);

	for(bill_LoopIndex=0;bill_LoopIndex<Tarriff_slots;bill_LoopIndex++)
	{
		if((bill_LoopIndex+1) == TOD_u8zone_index)
		{
			fill_oprzero();
			FUN_vfill_3byteR(TOD_u32cum_zkwh,&opr_data[2]);
			val_4byt(0,opr_data[0],opr_data[1],opr_data[2]);//zkwh

		}
		else
		{
			Eprom_Read(Zone_energy_data_Add+(u16)((u16)0x0010*(u16)bill_LoopIndex));
			val_4byt(0,opr_data[0],opr_data[1],opr_data[2]);//zkwh

		}

	}

	FUN_vfill_3byteR(MET_u32Cum_kvah,&(opr_data[2])); //cum_kva_dlms

	val_4byt(0x00,opr_data[0],opr_data[1],opr_data[2]);

	for(bill_LoopIndex=0;bill_LoopIndex<Tarriff_slots;bill_LoopIndex++)
	{
		if((bill_LoopIndex+1) == TOD_u8zone_index)
		{
			fill_oprzero();
			FUN_vfill_3byteR(TOD_u32cum_zkvah,&opr_data[2]);
			val_4byt(0,opr_data[0],opr_data[1],opr_data[2]);//zkvah
		}
		else
		{
			Eprom_Read(Zone_energy_data_Add+(u16)((u16)0x0010*(u16)bill_LoopIndex));
			
			val_4byt(0,opr_data[3],opr_data[4],opr_data[5]);//zkvah
		}

	}


	read_add(billmd_data_addrs,long_data);

	val_2byt(opr_data[0], opr_data[1]);//md kw
    sec=0;

	if((opr_data[2]==0xff)&&(opr_data[3]==0xff))
	{
		sec=0xff;
	}

	date_time(opr_data[6],opr_data[5],opr_data[4],opr_data[3],opr_data[2],sec,0);//md kw D&T
	
	for(bill_LoopIndex=0;bill_LoopIndex<Tarriff_slots;bill_LoopIndex++)
	{
		Eprom_Read(Zone_MD_energy_data_Add+(bill_LoopIndex*0x10));
		val_2byt(opr_data[0],opr_data[1]);
        sec=0;		
        if((opr_data[2]==0xff)&&(opr_data[3]==0xff))
		{
			sec=0xff;
		}
		date_time(opr_data[6],opr_data[5],opr_data[4],opr_data[3],opr_data[2],sec,0);//zmd-kw D&T
	}

	read_add(billmd_data_addrs,long_data);
	
	val_2byt(opr_data[7], opr_data[8]);//md kva
    sec=0;
	
	if((opr_data[9]==0xff)&&(opr_data[10]==0xff))
	{
		sec=0xff;
	}
	date_time(opr_data[13],opr_data[12],opr_data[11],opr_data[10],opr_data[9],sec,0);//md kVA D&T
	
	for(bill_LoopIndex=0;bill_LoopIndex<Tarriff_slots;bill_LoopIndex++)
	{
		Eprom_Read(Zone_MD_energy_data_Add+(bill_LoopIndex*0x10));
		val_2byt(opr_data[7],opr_data[8]);
        sec=0;		
        if((opr_data[9]==0xff)&&(opr_data[10]==0xff))
		{
			sec=0xff;
		}
		date_time(opr_data[13],opr_data[12],opr_data[11],opr_data[10],opr_data[9],sec,0);//zmd-kw D&T
		
	}

//	unsigned8(btpr_c,0);
//  calc_bill_poff(1);            

	FUN_vfill_2byteR(u16_bill_pow_on,&opr_data[1]);

	val_2byt(opr_data[0],opr_data[1]);//bill power on minutes
	
	
	FUN_vfill_3byteR(MET_u32Cum_kvarh,&(opr_data[2])); //cum_kvarh (Q+Q2)

	val_4byt(0x00,opr_data[0],opr_data[1],opr_data[2]);


//			for(bill_LoopIndex=0;bill_LoopIndex<8;bill_LoopIndex++)
//			{
//				if((bill_LoopIndex+1) == zone_index)
//				{
//					fill_oprzero();
//					fill_data(cum_zkwh,&opr_data[2]);
//					val_4byt(0,opr_data[0],opr_data[1],opr_data[2]);//zkwh
//					fill_oprzero();
//					fill_data(cum_zkvah,&opr_data[2]);
//					val_4byt(0,opr_data[0],opr_data[1],opr_data[2]);//zkvah
//				}
//				else
//				{
//					Eprom_Read(Zone_energy_data_Add+(0x10*bill_LoopIndex));
//					val_4byt(0,opr_data[0],opr_data[1],opr_data[2]);//zkwh
//					val_4byt(0,opr_data[3],opr_data[4],opr_data[5]);//zkvah
//				}
//
//			}
//			for(bill_LoopIndex=0;bill_LoopIndex<8;bill_LoopIndex++)
//			{
//				Eprom_Read(Zone_MD_energy_data_Add+(bill_LoopIndex*0x10));
//				val_2byt(opr_data[0],opr_data[1]);
//				date_time(opr_data[6],opr_data[5],opr_data[4],opr_data[3],opr_data[2],0,0);//zmd-kw D&T
//			}
//			for(bill_LoopIndex=0;bill_LoopIndex<4;bill_LoopIndex++)
//			{
//				Eprom_Read(Zone_MD_energy_data_Add+(bill_LoopIndex*0x10));
//				val_2byt(opr_data[7],opr_data[8]);
//				date_time(opr_data[13],opr_data[12],opr_data[11],opr_data[10],opr_data[9],0,0);//zmd-kw D&T
//			}
}

void Uart_BillFill(unsigned char month)
{
	u16 todbill_add;
	u8 bill_LoopIndex1,sec=0;
	static union
	{
		u8 a8Temp[2];
		u16 u16Temp;
	}bill_pf;

	structure(bill_profile_parameter_cap_obj[0]);		//43

	read_add(billkwh_data_addrs,month);

	date_time(opr_data[10],opr_data[9],opr_data[8],opr_data[7],opr_data[6],0,0); //bill_date12

	bill_pf.u16Temp=(u16)opr_data[14]*(u16)4;

	val_2byt(bill_pf.a8Temp[1],bill_pf.a8Temp[0]);//pf   2

	val_4byt(0x00,opr_data[0],opr_data[1],opr_data[2]); //kwh4

	todbill_add=read_todadd(month);//billtod_data_addrs+month*0x0100;

	for(bill_LoopIndex1=0;bill_LoopIndex1<Tarriff_slots;bill_LoopIndex1++)
	{
		Eprom_Read(todbill_add+(bill_LoopIndex1<<4));

		val_4byt(0,opr_data[0],opr_data[1],opr_data[2]);//zkwh

	}
	
	read_add(billkwh_data_addrs,month);
	val_4byt(0x00,opr_data[3],opr_data[4],opr_data[5]); //kva


	todbill_add=read_todadd(month);//billtod_data_addrs+month*0x0100;

	for(bill_LoopIndex1=0;bill_LoopIndex1<Tarriff_slots;bill_LoopIndex1++)
	{
		Eprom_Read(todbill_add+(bill_LoopIndex1<<4));

		
		val_4byt(0,opr_data[3],opr_data[4],opr_data[5]);//zkvah


	}	

	read_add(billmd_data_addrs,month);

	val_2byt(opr_data[0], opr_data[1]); //md-kw
	
    sec=0;	
    if((opr_data[2]==0xff)&&(opr_data[3]==0xff))
	{
		sec=0xff;
	}

	date_time(opr_data[6],opr_data[5],opr_data[4],opr_data[3],opr_data[2],sec,0); //md-kw,date and time12
	
	todbill_add=read_todMDadd(month);
	
	for(bill_LoopIndex1=0;bill_LoopIndex1<Tarriff_slots;bill_LoopIndex1++)
	{
		Eprom_Read(todbill_add + (bill_LoopIndex1<<4));

		val_2byt(opr_data[0],opr_data[1]);
		
		sec=0;		
		if((opr_data[2]==0xff)&&(opr_data[3]==0xff))
		{
			sec=0xff;
		}

		date_time(opr_data[6],opr_data[5],opr_data[4],opr_data[3],opr_data[2],sec,0);//zmd-kw D&T
	}
	
	read_add(billmd_data_addrs,month);

	val_2byt(opr_data[7], opr_data[8]);//md-kva2
	
    sec=0;	
    if((opr_data[10]==0xff)&&(opr_data[9]==0xff))
	{
		sec=0xff;
	}
	
	date_time(opr_data[13],opr_data[12],opr_data[11],opr_data[10],opr_data[9],sec,0);//md-kVA D&T12

	
	todbill_add=read_todMDadd(month);
	
	for(bill_LoopIndex1=0;bill_LoopIndex1<Tarriff_slots;bill_LoopIndex1++)
	{
		Eprom_Read(todbill_add + (bill_LoopIndex1<<4));

		val_2byt(opr_data[7],opr_data[8]);
		
		sec=0;		
		if((opr_data[10]==0xff)&&(opr_data[9]==0xff))
		{
			sec=0xff;
		}

		date_time(opr_data[13],opr_data[12],opr_data[11],opr_data[10],opr_data[9],sec,0);//zmd-kw D&T

	}

	read_add(billkwh_data_addrs,month);

	val_2byt(opr_data[12],opr_data[13]);//bill power off minutes
	
	read_add(billkvarh_data_addrs,month);    //Cumulative Kvarh (Q1+Q2)
	val_4byt(0x00,opr_data[0],opr_data[1],opr_data[2]); //kvarh
	

	//	unsigned8(opr_data[11],0);//bill count

//		todbill_add=read_todadd(month);//billtod_data_addrs+month*0x0100;
//	
//		for(bill_LoopIndex1=0;bill_LoopIndex1<8;bill_LoopIndex1++)
//		{
//			Eprom_Read(todbill_add+(bill_LoopIndex1<<4));
//	
//			val_4byt(0,opr_data[0],opr_data[1],opr_data[2]);//zkwh
//	
//			val_4byt(0,opr_data[3],opr_data[4],opr_data[5]);//zkvah
//	
//	
//		}
//		for(bill_LoopIndex1=0;bill_LoopIndex1<8;bill_LoopIndex1++)
//		{
//			Eprom_Read(todbill_add+0x80+(bill_LoopIndex1<<4));
//	
//			val_2byt(opr_data[0],opr_data[1]);
//	
//			date_time(opr_data[6],opr_data[5],opr_data[4],opr_data[3],opr_data[2],0,0);//zmd-kw D&T
//		}
//		for(bill_LoopIndex1=0;bill_LoopIndex1<8;bill_LoopIndex1++)
//		{
//			Eprom_Read(todbill_add+0x80+(bill_LoopIndex1<<4));
//	
//			val_2byt(opr_data[7],opr_data[8]);
//	
//			date_time(opr_data[13],opr_data[12],opr_data[11],opr_data[10],opr_data[9],0,0);//zmd-kw D&T
//		}
}

void bill_buffer(void)
{
	u16 buffer_filled_u16=0;
	u8 u8tempbill;
	u8 u8temp=0;
	u8 ctt_pass_f=0;
	u16 const BILL_FillWithOneBill = 420;

	//	if(sel_access_flag==1)
	//	{
	//		if(buffer_first_not_fill_f == 0)
	//		{
	//			dlms_address=(from_ptr-1);
	//
	//            if(to_ptr ==0)
	//				to_ptr1=12;
	//			else
	//				to_ptr1=to_ptr;
	//
	//			no_bills=to_ptr1-from_ptr+1;
	//
	//		}
	//	}
	if(GPR_u8TCPIP_F==1)
	{
		k=5;
		Start_Info2();
		k=20;
	}
	else
	{
		k=0;
		Start_Info2();
		k=15;
	}
	buffer_filled_u16=k;
	if(buffer_first_not_fill_f == 0)
	{
		if(sel_access_flag == 0)
		{
			dlms_address=0;
			if(BILL_u8md_count > MaxBillDate)
			no_bills=MaxBillDate+1;//initially 1 max. 13
			else
			no_bills=BILL_u8md_count+1;

			to_ptr=no_bills;
		}
		else
		{
			u8tempbill= BILL_u8md_count+1;
			if(u8tempbill>(MaxBillDate+1))
			{
				u8tempbill=MaxBillDate+1;
			}
			if((u8tempbill<to_ptr)||(u8tempbill<from_ptr)||((from_ptr>to_ptr)&&(to_ptr!=0))||(to_ptr>(MaxBillDate+1))||(from_ptr==0)||(access_selector==1))
			{
				no_bills=0;
			}
			else
			{
				if(0==from_ptr)
				{
			from_ptr=1;
				}
			if(to_ptr ==0 || to_ptr > (MaxBillDate+1))
			{
				to_ptr=MaxBillDate+1;
			}
			if(to_ptr>(BILL_u8md_count+1))    
			{
			     to_ptr =BILL_u8md_count+1;
			}

				dlms_address=(from_ptr-1);
			//to_ptr1=to_ptr;
			no_bills=to_ptr-from_ptr+1;
				if((no_obj==1))
				{
					ctt_pass_f=1;
				}
			}
		}
		array(no_bills,0);
		buffer_filled_u16+=2;
		block_no=1;
		if(0==no_bills)
		{
			u8temp=1;
			buffer_first_not_fill_f=0;
			multi_filling_f=0;
		}
		else
		{
		buffer_first_not_fill_f=1;
		multi_filling_f=1;
		}
		element_filled=0;
	}
	if(ctt_pass_f==0)
	{
    for(;element_filled<(no_bills);element_filled++)
	{
		buffer_filled_u16+=BILL_FillWithOneBill;//420;
		switch(dlms_address)
		{
		case 0:
			if(BILL_u8md_count==0)					//for check no bill is present
			{
				Uart_CurrentBillFill();

			}
			else
			{
				if(BILL_u8md_count>MaxBillDate)    // if roll over then send first bill by pass max value
				{
					Uart_BillFill(MaxBillDate);
				}
				else
				{
					Uart_BillFill(BILL_u8md_count);
					
				}

			}
	
			break;
			
		case 1:
		case 2:
		case 3:
		case 4:
		case 5:
		case 6:
		case 7:
		case 8:
		case 9:
		case 10:
		case 11:
		case 12:
			if(dlms_address >= MaxBillDate)	// for current bill after sending max stored bill
			{
				Uart_CurrentBillFill();
			}
			else if(BILL_u8md_count>MaxBillDate)	 // if roll over then send bills by oldest bill first
			{
				Uart_BillFill(MaxBillDate-dlms_address);
			}
			else if(BILL_u8md_count > dlms_address)	// before roll over send bills by oldest bill first
			{
				Uart_BillFill(BILL_u8md_count-dlms_address);
			}
			else							// before roll over for current bill after sending all stored bill 
			{
				Uart_CurrentBillFill();
			}
			break;
		}
		dlms_address++;
		if(buffer_filled_u16 !=k)
		{
			__nop();
		}
		if(DLMS_MAX_BUFF_SIZE<(buffer_filled_u16+BILL_FillWithOneBill))
		{
			break;
		}
		if(dlms_address >= to_ptr)
		{
			break;
		}	
   	}
	}
	else
	{
		structure(1);
		date_time(dt.day,dt.month,dt.year,dt.hour,dt.min,0x00,0x00);
		dlms_address++;
   }
	if(dlms_address >= to_ptr)
	{
		multi_filling_f=0;
		buffer_first_not_fill_f=0;
		u8temp=1;
	}
	send_data(u8temp);
}
void fill_firmware_version(void)
{
#if LDN == 0
	fill_info(firmware_ver);
#else
	fill_info(firmware_ver_GIL);
#endif	
	Eprom_Read(SerialNo_UtilityID_add);
	if(EPROM_bChksum_ok==0)
	{
		Eprom_Read(Dupli_SerialNo_UtilityID_add);
		if(EPROM_bChksum_ok==1)
		Eprom_Write(SerialNo_UtilityID_add);
	}
	info[k-11]=hex_in_ascii[(opr_data[8]>>4)&0x0F];
	info[k-10]=hex_in_ascii[(opr_data[8])&0x0F];
	info[k-9]=hex_in_ascii[(opr_data[9]>>4)&0x0F];
	info[k-8]=hex_in_ascii[(opr_data[9])&0x0F];
}
unsigned int fcs_cal(unsigned int fcs,unsigned char *cp,unsigned int len)
{//fcs_calapplication layer
	while(len--)
	fcs=(fcs >> 8)^fcstab[(fcs^*cp++)&0xFF];
	return(fcs);
}//end of fcs_cal
unsigned char fcs(unsigned char *cp,unsigned int len,unsigned char flag) //(Frame Check Sequence)
{//fcsapplication layer
	unsigned int final_fcs = 0x0000;
	if(flag == 1)
	{//if1
		final_fcs = fcs_cal(PPPINITFCS16,cp,len);
		final_fcs ^= 0xFFFF;
		cp[len]= (final_fcs & 0x00FF);
		dlms_x = (u8)final_fcs;//cp[len];
		cp[len+1]= ((final_fcs >> 8) & 0x00FF);
		dlms_y = (u8)(final_fcs >> 8);//cp[len+1];
	}//if1
	else if(flag == 2)
	{//else2
		final_fcs = fcs_cal(PPPINITFCS16,cp,len);
		if(final_fcs == PPPGOODFCS16 )
		return(1);//  //printf("\ngood FCS");
		else
		return(0);//  //printf("\nERROR in FCS");
	}//else2
	return(1);
}//end of fcs


u8 set_resp(void)
{
	rtc_counter_value_t time2;
	u16 i;
	unsigned int  di,diff1;
	u8 addpg;
	u8 u8tem_mm;
	u8 u8tem_dd;
	u8 u8tem_h;
	u8 u8tem_m;
	u8 u8tem_dow;
	u8 u8tem_s;
	u8 count;
	u8 noweek;
	u16 u16tem_yy;
	unsigned char i_data=0x00;
	u8 UcharTemp=0x00;
	unsigned int uint_temp=0x00;
	u8 DevAddr=0x00;

	switch(obis_short)
	{//swa
	case 0x00010000:
		if(attribute_id==2 && obis_code[3] == 0x00)//RTC Set(0,0,1,0,0,255)
		{//ifattribute_id
			if(info[k]!=0x09 || info[k+1]!=0x0C)
			{
				conf_err(0x0C);
				return 0;
			}
			k++;
			k++;
			
			u16tem_yy=(unsigned int)info[k]*256+info[k+1];
			u8tem_mm=info[k+2];
			u8tem_dd=info[k+3];
			uint_temp=isdatevalid(u8tem_mm, u8tem_dd, u16tem_yy);
			
			if(uint_temp==0)
			{
				conf_err(250);
				return 0;
			}
			u8tem_dow=info[k+4];
			
			if(((u8tem_dow>7)||(u8tem_dow<1))&&(u8tem_dow!=255))
			{
				conf_err(250);
				return 0;
			}
			
			u8tem_h=info[k+5];
			u8tem_m=info[k+6];
			u8tem_s=info[k+7];
			
			if((u8tem_h>23)||(u8tem_m>59)||(u8tem_s>59))
			{
				conf_err(250);
				return 0;
			}
			/////////////////////////////////////////////
			log_trans_event(151);
			uint_temp = u16tem_yy- 0x07D0;
			UcharTemp = (unsigned char)(uint_temp % 256);
			
			u16tem_yy =hex_to_bcd(UcharTemp);
			u8tem_mm=hex_to_bcd(u8tem_mm);
			u8tem_dd=hex_to_bcd(u8tem_dd);
			u8tem_h=hex_to_bcd(u8tem_h);
			u8tem_m=hex_to_bcd(u8tem_m);
			u8tem_s=hex_to_bcd(u8tem_s);
			
			if(u16tem_yy> dt.year ||(u16tem_yy==dt.year && u8tem_mm> dt.month)
					||(u16tem_yy==dt.year && u8tem_mm==dt.month && u8tem_dd>= dt.day))
			{
				Eprom_Read(PowerDownTimeAdd);
				time_stamp(&opr_data[0]);
				if((opr_data[2]!=u16tem_yy)||(opr_data[3]!=u8tem_mm)||(opr_data[4]!=u8tem_dd))
				{
					//                    pow_on+=(1440)-(bcd_to_hex(opr_data[1])*60)-bcd_to_hex(opr_data[0]);
                    pow_nextday_f=1;
				}
				else
				{
					//                    pow_on += (bcd_to_hex(u8tem_h)*60)+bcd_to_hex(u8tem_m)-(bcd_to_hex(opr_data[1])*60)-bcd_to_hex(opr_data[0]);
				}
				//                FUN_vfill_2byteR(pow_on,&opr_data[6]);
				Eprom_Write(PowerDownTimeAdd);
				
				rtc_set_flag=1;
				bill_f=1;
				if(u16tem_yy==dt.year && u8tem_mm==dt.month && u8tem_dd== dt.day)
				{
					if(u8tem_h== dt.hour  && (bcd_to_hex(u8tem_m)/(ls_ip)== (bcd_to_hex(dt.min)/(ls_ip))))
					{
						LS.b.ls_rtc_fill=0;
					}
					else if(u8tem_h>dt.hour || (u8tem_h==dt.hour && u8tem_m>=dt.min))
					{
						LS.b.ls_rtc_fill=1;
						ls_timestamp(ls_ip);
					}
					else
					{
						FUN_vfill_2byteR(load_survey_cnt,&opr_data[1]);
						opr_data[2]=LS.b.ls_of=0;
						opr_data[10]=day_counter_ls;
						Eprom_Write(load_survey_status);
						
						LS.b.ls_rtc_fill=1;
						ls_timestamp(ls_ip);
					}
				}
				else
				{
					LS.b.ls_rtc_fill=1;
					ls_timestamp(ls_ip);
				}
			}
			else
			{
				uint_temp=sel_datediff(u8tem_dd,u8tem_mm,u16tem_yy);
				UcharTemp=250;
				for(u8tem_m=day_counter_ls,u8tem_s=0;u8tem_s<maxday_counter_l;u8tem_s++)
				{
					if(uint_temp >= d_array[u8tem_m] && d_array[u8tem_m] !=0)// || to_days > d_array[i])
					{
						UcharTemp=u8tem_m;
						break;
					}
					u8tem_m = u8tem_m-1;
					if(u8tem_m==255)
					{
						UcharTemp=250;
						break;
					}
				}
				if(UcharTemp == 250)
				{
					day_counter_ls=0;
					LS.b.ls_fg_f=1;
					LS.b.ls_rtc_fill=1;
					LS.b.ls_rev_fill=1;
				}
				else if(uint_temp > d_array[u8tem_m])
				{
					day_counter_ls=UcharTemp+1;
					LS.b.ls_fg_f=1;
					LS.b.ls_rtc_fill=1;
					LS.b.ls_rev_fill=1;
				}
				else
				{
					day_counter_ls=UcharTemp;
					
				}
				UcharTemp=250;
				uint_temp=sel_datediff(u8tem_dd,u8tem_mm,u16tem_yy);
				//dcounter=daily_enrcount;//a8_to_u16(&opr_data[0]);
				R_WDT_Restart();
				for(u8tem_m=daily_enrcount-1,u8tem_s=0;u8tem_s<maxday_counter_d;u8tem_s++)
				{
					Eprom_Read(dloadsurvey_init_add+(u16)((u16)u8tem_m * (u16)0x0020));
					diff1=sel_datediff(opr_data[4],opr_data[3],opr_data[2]);
					if(uint_temp >=diff1 && diff1!=0)
					{
						UcharTemp=u8tem_m;
						break;
					}
					u8tem_m = u8tem_m-1;
					if(u8tem_m==255)
					{
						UcharTemp=250;
						break;
					}
					if(u8tem_s%20==0)
					{
						R_WDT_Restart();
					}
				}
				Eprom_Read(dloadsurvey_init_add+(u16)((u16)u8tem_m * 0x0020));
				diff1=sel_datediff(opr_data[4],opr_data[3],opr_data[2]);
				R_WDT_Restart();
				if(UcharTemp == 250)
				{
					daily_enrcount=0;
				}
				else if(uint_temp >= diff1)
				{
					daily_enrcount=UcharTemp+1;
				}
				
				Eprom_Read(DailyEnergyStatus);
				opr_data[0]=daily_enrcount;
				opr_data[5]=DE.b.dls_of=0;
				Eprom_Write(DailyEnergyStatus);
				DE.b.daily_rev_fill=1;
				rtc_set_flag=1;
				
				load_ls_cnt();
				FUN_vfill_2byteR(load_survey_cnt,&opr_data[1]);
				opr_data[2]=LS.b.ls_of=0;
				opr_data[10]=day_counter_ls;
				Eprom_Write(load_survey_status);
			}
			if(LS.b.ls_of == 0)
			{
				for(i_data = day_counter_ls+1;i_data<maxday_counter_l;i_data++)
				{
					d_array[i_data]=0;
					fill_zerodarray(i_data);
					R_WDT_Restart();
				}
			}
			if(rtc_set_flag == 1)
			{
				BILL_u16mdkw_c = 0;
				BILL_u16mdkva_c = 0;
				BILL_u32mdkw_hires_c=0;
				BILL_u32mdkva_hires_c=0;
				
				UcharTemp = info[k++];
				uint_temp = (unsigned int)UcharTemp*256;
				uint_temp = uint_temp + info[k++] - 0x07D0;
				UcharTemp = (unsigned char)(uint_temp % 256);
				
				time2.year = hex_to_bcd(UcharTemp);
				UcharTemp = info[k++];
				time2.month=hex_to_bcd(UcharTemp);
				UcharTemp = info[k++];
				time2.day =hex_to_bcd(UcharTemp);
				UcharTemp = info[k++];
				if(UcharTemp==7)
				time2.week = 0;
				else
				time2.week = (UcharTemp);
				UcharTemp = info[k++];
				time2.hour = hex_to_bcd(UcharTemp);
				UcharTemp = info[k++];
				time2.min = hex_to_bcd(UcharTemp);
				UcharTemp = info[k++];
				time2.sec = hex_to_bcd(UcharTemp);
				R_RTC_Set_CounterValue(time2);
				rtc_status_byte=0;	
				save_rtc_status();
				TOU_vDeter_Season();
//BLA SIR START
					 TOD_u8zmd_index=TOD_u8vmain0;
				if(TOD_bzonecng_f0==1)
				{
 R_WDT_Restart();
						 TOD_bzonecng_f0=0;
					storezkwh();
					TOD_vloadzkwh();
				}
			}
			else
			{
				conf_err(250);
				return 0;
			}
		}//ifattribute_id
		else
		{
		  conf_err(250);
		  return 0;	
		}

		break;
	case 0x000d0000://TOU write (0,0,13,0,0,255)
		if(attribute_id == 6)
		{
			if((info[16]!=0x09) || (info[17]==0) || (info[17]>14))
			{
				conf_err(0x0C);
				return 0;
			}
			fill_oprzero();
			memcpy(opr_data,&info[17],info[17]+1);
			if(TOD_bActive_Calendar == 0)
			Eprom_Write(TOU_CAL_PASSIVE_ADD);
			else if(TOD_bActive_Calendar == 1)
			Eprom_Write(TOU_CAL_ACTIVE_ADD);
			//            a8Daily_alert_status[3]|=DES_TOU_cng;
			log_trans_event(155);
		}
		else if(attribute_id == 7)
		{
			if((info[k]!=0x01) || (info[k+2] != 0x02) || (info[k+3] != 0x03) || (info[k+4] != 0x09) || (info[k+5] != 0x07))
			{
				conf_err(0x0C);
				return 0;
			}
			if((info[k+1] > 2)||(info[k+1] == 0))//no of season
			{
				conf_err(0x0C);
				return 0;
			}
			if(info[k+13]!=0x09 || info[k+14]!=0x0C )//|| info[k+15]!=0xff || info[k+16]!=0xff )//rtc of season 1
			{
				conf_err(0x0C);
				return 0;
			}
			uint_temp=isdatevalid(info[k+17], info[k+18], bcd_to_hex(dt.year)+2000);//2012);//(unsigned int)info[k+15]*256+info[k+16]);
			if(uint_temp==0)
			{
				conf_err(250);
				return 0;
			}
			if(((info[k+20]>23)&&(info[k+20]!=0xff))||((info[k+21]>59)&&(info[k+21]!=0xff)))
			{
				conf_err(250);
				return 0;
			}
			if((info[k+27]!=0x09)||(info[k+28]!=0x07))// season week 1 name
			{
				conf_err(0x0C);
				return 0;
			}
			if(info[k+1]==2)
			{
				if((info[k+36] != 0x02) || (info[k+37] != 0x03) || (info[k+38] != 0x09) || (info[k+39] != 0x07))
				{
					conf_err(0x0C);
					return 0;
				}
				if(info[k+47]!=0x09 || info[k+48]!=0x0C)//|| info[k+49]!=0xff || info[k+50]!=0xff )//rtc of season 2
				{
					conf_err(0x0C);
					return 0;
				}
				uint_temp=isdatevalid(info[k+51], info[k+52], bcd_to_hex(dt.year)+2000);//(unsigned int)info[k+49]*256+info[k+50]);
				if(uint_temp==0)
				{
					conf_err(250);
					return 0;
				}
				if(((info[k+54]>23)&&(info[k+54]!=0xff))||((info[k+55]>59)&&(info[k+55]!=0xff)))
				{
					conf_err(250);
					return 0;
				}
				if((info[k+61]!=0x09)||(info[k+62]!=0x07))// season week 2 name
				{
					conf_err(0x0C);
					return 0;
				}
			}
			if(TOD_bActive_Calendar == 0)
			addpg=0x40;
			else
			addpg=0x00;
			fill_oprzero();
			Eprom_Write(TOU_CAL_ACTIVE_ADD+0X10+addpg);
			Eprom_Write(TOU_CAL_ACTIVE_ADD+0X20+addpg);
			Eprom_Write(TOU_CAL_ACTIVE_ADD+0X30+addpg);
			opr_data[0]=info[k+1];//no. of seasons
			memcpy(opr_data+1,&info[k+6],7);
			if(info[k+1]==2)
			{
				memcpy(opr_data+8,&info[k+40],7);
			}
			Eprom_Write(TOU_CAL_ACTIVE_ADD+0X10+addpg);// season names
			fill_oprzero();
			uint_temp = ((unsigned int)info[k+15]*256+info[k+16]) - 0x07D0;
			opr_data[0]=0xff;//hex_to_bcd(info[k+21]);//min
			opr_data[1]=0xff;//hex_to_bcd(info[k+20]);//hour
			opr_data[2]=0xff;//hex_to_bcd((unsigned char)uint_temp) ;
			
			if(info[k+1]==2)
			{
				opr_data[3]=hex_to_bcd(info[k+17]);//month
				opr_data[4]=hex_to_bcd(info[k+18]);//date
				uint_temp = ((unsigned int)info[k+49]*256+info[k+50]) - 0x07D0;
				opr_data[5]=0xff;//hex_to_bcd(info[k+55]);//min
				opr_data[6]=0xff;//hex_to_bcd(info[k+54]);//hour
				opr_data[7]=0xff;//hex_to_bcd((unsigned char)uint_temp) ;
				opr_data[8]=hex_to_bcd(info[k+51]);//month
				opr_data[9]=hex_to_bcd(info[k+52]);//date
			}
			else
			{
				opr_data[3]=0xff;
				opr_data[4]=0xff;
			}
			Eprom_Write(TOU_CAL_ACTIVE_ADD+0X20+addpg);
			fill_oprzero();
			memcpy(opr_data,&info[k+29],7);
			if(info[k+1]==2)
			{
				memcpy(opr_data+7,&info[k+63],7);
			}
			Eprom_Write(TOU_CAL_ACTIVE_ADD+0X30+addpg);// season week names
			log_trans_event(155);
		}
		else if(attribute_id == 8)///week profile
		{
			if(info[k] != 0x01 || info[k+2] != 0x02 || info[k+3] != 0x08)
			{
				conf_err(0x0C);
				return 0;
			}
			noweek=info[k+1];
			if(noweek > 2)
			{
				conf_err(0x0C);
				return 0;
			}
			if(TOD_bActive_Calendar == 0)
			addpg=0x40;
			else
			addpg=0x00;
			k+=4;
			if(info[k] != 0x09)
			{
				conf_err(0x0C);
				return 0;
			}
			k+=2;
			fill_oprzero();
			Eprom_Write(TOU_WEEK_ACTIVE_ADD+addpg);
			Eprom_Write(TOU_WEEK_ACTIVE_ADD+0x10+addpg);
			Eprom_Write(TOU_WEEK_ACTIVE_ADD+0x20+addpg);
			Eprom_Write(TOU_WEEK_ACTIVE_ADD+0x30+addpg);
			for(count=0;count<noweek;count++)
			{
				memcpy(opr_data,&info[k],7);
				k=k+7;
				opr_data[14]=noweek;
				Eprom_Write(TOU_WEEK_ACTIVE_ADD+addpg+(count*0x20));
				fill_oprzero();
				di=k+1;
				for(i_data=0;i_data<7;di+=2,i_data++)
				{
					opr_data[i_data]=info[di];
					k+=2;
				}
				
				Eprom_Write(TOU_WEEK_ACTIVE_ADD+0x10+addpg+(count*0x20));
				fill_oprzero();
				k+=4;
				
			}
			log_trans_event(155);
		}
		
		else if(attribute_id == 9)
		{//ifattribute_id
			u8tem_m=tou_pssv_store();
			if(last_block==1)
			{
				log_trans_event(155);
			}
			if(u8tem_m==0)
			{
				conf_err(250);
				return 0;
			}
			else if(u8tem_m == 1)
			return 1;
			else
			return 2;

		}//ifattribute_id
		
		else if (attribute_id==10)
		{
			if(info[k]!=0x09 || info[k+1]!=0x0C)
			{
				conf_err(0x0C);
				return 0;
			}
			
			k++;
			
			k++;
			
			u16tem_yy=(unsigned int)info[k]*256+info[k+1];
			
			u8tem_mm=info[k+2];
			
			u8tem_dd=info[k+3];
			
			uint_temp=isdatevalid(u8tem_mm, u8tem_dd, u16tem_yy);
			
			if(uint_temp==0)
			{
				conf_err(250);
				return 0;
			}
			
			u8tem_h=info[k+5];
			u8tem_m=info[k+6];
			u8tem_s=info[k+7];
			
			if((u8tem_h>23)||(u8tem_m>59)||(u8tem_s>59))
			{
				conf_err(250);
				return 0;
			}
			fill_oprzero();
			UcharTemp = info[k++];
			uint_temp = (unsigned int)UcharTemp*256;
			uint_temp = uint_temp + info[k++] - 0x07D0;
			UcharTemp = (unsigned char)(uint_temp % 256);
			opr_data[0] = hex_to_bcd(UcharTemp);//yr
			
			
			UcharTemp = info[k++];
			opr_data[1]=hex_to_bcd(UcharTemp);////month
			UcharTemp = info[k++];
			opr_data[2]=hex_to_bcd(UcharTemp);////day
			//dow skipped
			k++;
			UcharTemp = info[k++];
			opr_data[3]=hex_to_bcd(UcharTemp);////hour
			UcharTemp = info[k++];
			opr_data[4]=hex_to_bcd(UcharTemp);////min
			
			
			for(i_data=5;i_data<15;i_data++)
			opr_data [i_data]=0;
			
			Eprom_Write(TOU_PassiveApliDate);
			fill_oprzero();
			TOD_bcalendar_change_f=1;
			TOD_bcalendar_change_FG_f=0;
			opr_data[0]=TOD_bcalendar_change_f;
			
			fill_ff(1);
			opr_data[1]=TOD_bcalendar_change_FG_f;
			Eprom_Write(TOU_CheckPassiveApli);
			
			TOU_vCheck_Active_Calendar();
			if(TOD_bActive_f != TOD_bActive_Calendar)
			{
				TOU_vDeter_Season();
				TOD_bzonecng_f0=0;
				storezkwh();
				TOD_vloadzkwh();
			}
			
			//            a8Daily_alert_status[3]|=DES_TOU_cng;
			
			log_trans_event(155);
		}
		else
		{
			conf_err(0x0b);
			return 0;
		}
		break;
		
	case 0x000f0000://Regular/Irragular Billing Date  (0,0,15,0,0,255)
		if(attribute_id == 4)//billing date
		{
			
			if((info[k+2]==0x09)&&(info[k+3]==0x0c))
			{
				opr_data[1] = hex_to_bcd(info[k+9]);
				opr_data[0] = hex_to_bcd(info[k+10]);
				opr_data[4] = hex_to_bcd(info[k+7]);
			}
			else
			{
				if((info[k]!=0x01)||(info[k+1]!=0x01)||(info[k+2]!=0x02)||(info[k+3]!=0x02))
				{
					conf_err(0x0C);
					return 0;
				}

				if(info[k+4]!=0x09 || info[k+5]!=0x04)
				{
					conf_err(0x0C);
					return 0;
				}
				
				if(info[k+10]!=0x09 || info[k+11]!=0x05)
				{
					conf_err(0x0C);
					return 0;
				}
				
				opr_data[1] = hex_to_bcd(info[k+6]);
				opr_data[0] = hex_to_bcd(info[k+7]);
				opr_data[4] = hex_to_bcd(info[k+15]);
			}
			if((opr_data[1]>0x23)||(opr_data[0]>0x59))
			{
				conf_err(250);
				return 0;
			}
			else if((opr_data[4]>0x28)||(opr_data[4]==0x0))
			{
				conf_err(250);
				return 0;
			}
			else
			{
				Eprom_Write(NextBillDateAdd);
				BILL_a8date_array[0]=opr_data[0];
				BILL_a8date_array[1]=opr_data[1];
				BILL_a8date_array[4]=opr_data[4];
				GetNextDate(dt.min, dt.hour, dt.day,dt.month, dt.year, BILL_a8date_array[0], BILL_a8date_array[1], BILL_a8date_array[4], 0);
				//            a8Daily_alert_status[3]|=DES_bill_date;
				log_trans_event(154);
			}
		}
		else
		{
			conf_err(0x0b);
			return 0;
		}
		break;
		//      case 0x000f0002://Image Activation Date  (0,0,15,0,2,255)	
		//        if(attribute_id == 4)   //Image Activation date
		//        {
		//            if((info[k]!=0x01)&&(info[k+1]!=0x01)&&(info[k+2]!=0x02)&&(info[k+3]!=0x02))
		//            {
		//                conf_err(0x0C);
		//                return 0;
		//            }
		//            k=20;
		//            if(info[k]!=0x09 || info[k+1]!=0x04)
		//            {
		//                conf_err(0x0C);
		//                return 0;
		//            }
		//            if(info[k+6]!=0x09 || info[k+7]!=0x05)
		//            {
		//                conf_err(0x0C);
		//                return 0;
		//            }
		//            u16tem_yy=(unsigned int)info[k+8]*256+info[k+9];
		//            u8tem_mm=info[k+10];
		//            u8tem_dd=info[k+11];
		//            uint_temp=isdatevalid(u8tem_mm, u8tem_dd, u16tem_yy);
		//            fill_oprzero();
		//            Eprom_Read(0x2670);
		//            if(opr_data[0]!=3)
		//            {
		//                conf_err(250);//other-reason
		//                return 0;
		//            }
		//            if(uint_temp==0)
		//            {
		//                conf_err(250);
		//                return 0;
		//            }
		//            opr_data[0] = 1;
		//            opr_data[1] = hex_to_bcd(u16tem_yy-2000);
		//            opr_data[2] = hex_to_bcd(u8tem_mm);
		//            opr_data[3] = hex_to_bcd(u8tem_dd);
		//            Eprom_Write(0x2690);	
		//            a8Daily_alert_status[4]|=DES_image_active;
		//            log_trans_event(161); 
		//        }
		//        break;
	case 0x00600100://(0,0,96,1,0,255)//sr no genus asso	
		if((asso0_flag == 1)||(asso1_flag == 1) || (asso2_flag == 1))
		{
			conf_err(0x0D);
			return 0;
		}
		
		if(attribute_id==2)
		{
			if(info[k]!=0x02 || info[k+1]!=0x02 || info[k+2]!=0x09 || info[k+3]!=0x0d)
			{//if9
				conf_err(0x0C);
				return 0;
			}//if9
			
			k=k+4;
			memcpy(&opr_data[10],&info[k],5);
			k+=5;
			memcpy(opr_data,&info[k],8);
			k+=8;
			k=k+2;
			memcpy(&opr_data[8],&info[k],2);
			k+=2;
			Eprom_Write(SerialNo_UtilityID_add);	//cngd	
			Eprom_Write(Dupli_SerialNo_UtilityID_add);
			if(fg_done_f==1)
			{
				Disable_SingleWireBckup=1; //it will disable single wire feature till next pwr up
			}
			

		}
		break;
	case 0x00280002:   ///(0,0,40,0,2,255)
		if((asso0_flag == 1)||(asso1_flag == 1))
		{
			conf_err(0x0D);
			return 0;
		}
		if(attribute_id==7)
		{
			if(info[k]!=0x09 || info[k+1]!=0x08)
			{
				conf_err(0x0C);
				return 0;
			}
			
			k++;
			k++;
			memcpy(aut_pswd,&info[k],8);
			k+=8;
			memcpy(opr_data,aut_pswd,8);
			
			Eprom_Write(DLMS_HLS_Pass2Add);
		}
		break;
	  case 0x0160803E:         //instant Eprom testing	
		if((asso0_flag == 1)||(asso1_flag == 1) || (asso2_flag == 1))
		{
			conf_err(0x0D);
			return 0;
		}
		
		if(attribute_id==2)//&&(0==fg_done_f))
		{
			// info[17]--- No of Memories
			// info[19]--- Memory Type	1:16K, 2:256K, 3:512K, 4:1Mb
			// info[21]--- Memory Type
#if NoOfMemoryPresent ==2
			if((info[16]!=0x02)||(info[17]<=0x01)||(info[17]>0x02)||(info[18]!=0x11)||(info[20]!=0x11)||(info[19]<=0x01)||(info[21]==0x01)||(info[19]>0x04)||(info[21]>0x04))
#elif NoOfMemoryPresent ==1
			if((info[16]!=0x02)||(info[17]!=0x01)||(info[18]!=0x11)||(info[19]<=0x01)||(info[19]>0x04))
#endif			
			{
				conf_err(0x0C);
				return 0;
			}
			
			DevAddr=findDeviceAddress(info[19],1);
			MemoryStatus1=Eprom_dignostic(info[19],DevAddr);
#if NoOfMemoryPresent ==2			
			DevAddr=findDeviceAddress(info[21], 2);
			MemoryStatus2=Eprom_dignostic(info[21],DevAddr);
#endif			

		}
		else
		{
			fill_0d();
		}		
		break;		
	case 0x01000800://md_ip(1.0.0.8.0.255)
		if(info[k]!=0x12)
		{
			conf_err(0x0C);
			return 0;
		}
		k++;
		uint_temp=256*info[k]+info[k+1];
		if((uint_temp!=900)&&(uint_temp!=1800)&&(uint_temp!=3600))
		{
			conf_err(250);
			return 0;
		}
		md_ip_new=((256*info[k])+info[k+1])/60;   //10800/(256*info[k]+info[k+1]);
		if(md_ip!=md_ip_new)
		{
			mdfun();
			md_ip=md_ip_new;
			opr_data[0]=md_ip;
			Eprom_Write(MD_IP_Add);
		}

		log_trans_event(152);
		break;
	case 0x01000804://ls_ip(1.0.0.8.4.255)
		if(info[k]!=0x12)
		{
			conf_err(0x0C);
			return 0;
		}
		k++;
		uint_temp=256*info[k]+info[k+1];
		if((uint_temp!=900)&&(uint_temp!=1800)&&(uint_temp!=3600))
		{
			conf_err(250);
			return 0;
		}
		ls_ip_new= (256*info[k]+info[k+1])/60;
		
		if(ls_ip_new != 0x1e &&  ls_ip_new != 0x0f && ls_ip_new != 0x3c)     //improvement===nnnnnnnnnnnnnnnnnnnn
		{
			ls_ip_new = 0x1e;
		}
		
		k=k+2;
		if(ls_ip_new != ls_ip)
		{
			//****load survey integration change avg voltages zero and current energy load in last energy registers*************
			//load_survey();
			LS_vValueReset();

			maxday_counter_l=((max_loadsurvey)/(24*(60/ls_ip_new)));
			
			if(maxday_counter_l>MaxDarraySize)
			{
				maxday_counter_l=MaxDarraySize;
			}
			ls_ip=ls_ip_new;
			opr_data[0]=ls_ip;
			opr_data[1]=maxday_counter_l;
			Eprom_Write(LS_IP_Add);
			
			for(i_data=0;i_data<maxday_counter_l;i_data++)
			{
				d_array[i_data]=0;
				fill_zerodarray(i_data);
				R_WDT_Restart();
			}
			
			day_counter_ls=0;
			load_survey_cnt=0;
			LS.b.ls_of=0;
			//daily_enrcount=0;
			//DE.b.dls_of=0;
			fill_oprzero();
			Eprom_Write(load_survey_status);
            //Eprom_Write(DailyEnergyStatus);
			d_array[day_counter_ls]=sel_datediff((dt.day),(dt.month),(dt.year));
			fill_darray();
			
			LS.b.ls_fg_f=1;var5=0;var4=0;
			LS.b.ls_rtc_fill=0;
			LS.b.ls_rev_fill=0;
			ls_miss_fill();
		}
		log_trans_event(153);
		break;
		
	case 0x01608009:            //pcb tracking
		if(attribute_id == 2)
		{
			fill_oprzero();
			if(info[k]!=0x06)
			{
				conf_err(0x0C);
				return 0;
			}
			memcpy(opr_data,&info[k+1],4);
			opr_data[6]=1;
			Eprom_Write(PCBSerialNo_add);
			Eprom_Write(Dupli_PCBSerialNo_add);
		}			
		break;	
	case 0x01608000:
		if((asso0_flag == 1)||(asso1_flag == 1)||(asso2_flag == 1))
		{
			conf_err(0x0D);
			return 0;
		}
		if(attribute_id == 2)
		{
			if(info[k] != 0x09)
			{
				conf_err(0x0C);
				return 0;
			}
			if(info[k+1]>240)
			{
				conf_err(0x0C);
				return 0;
			}
			SaveDisplayList(lcd_Auto_Addr,info[k+1],'a');
			LCD_vLCDRamInit();
		}
		break;
	case 0x01608001:
		if((asso0_flag == 1)||(asso1_flag == 1)||(asso2_flag == 1))
		{
			conf_err(0x0D);
			return 0;
		}
		if(attribute_id == 2)
		{
			if(info[k] != 0x09)
			{
				conf_err(0x0C);
				return 0;
			}
			if(info[k+1]>240)
			{
				conf_err(0x0C);
				return 0;
			}
			SaveDisplayList(lcd_Push_Addr,info[k+1],'p');
			LCD_vLCDRamInit();
		}
		break;
		
	case 0x01608002:
		if((asso0_flag == 1)||(asso1_flag == 1)||(asso2_flag == 1))
		{
			conf_err(0x0D);
			return 0;
		}
		if(attribute_id == 2)
		{
			if(info[k] != 0x11)
			{
				conf_err(0x0C);
				return 0;
			}
			if(info[k+1]>30)
			{
				conf_err(0x0C);
				return 0;
			}
			//SaveDisplayList(lcd_Push_Addr,info[k+1],'p');
			Eprom_Read(lcd_Auto_Push_NO_Addr);
			opr_data[14]=info[k+1];
			lcd_scroll_time = opr_data[14];
			Eprom_Write(lcd_Auto_Push_NO_Addr);
		}
	break;
	
	case 0x01608003://LED Configuration set
        if((asso0_flag == 1)||(asso1_flag == 1))
        {
            conf_err(0x0D);
            return 0;
        }
        if(attribute_id == 2)
        {
            if((info[k] != 0x11)||(info[k+1]>0x04))
            {
                conf_err(0x0C);
                return 0;
            }

            CAL_u8LED_config = info[k+1];
            opr_data[0]= CAL_u8LED_config;
            opr_data[1]= 1;
            Eprom_Write(CalLedFunctionAddr);
        }
        break;
		
	case 0x01608007://tamper selection byte
		if((asso0_flag == 1)||(asso1_flag == 1)||(asso2_flag == 1))
		{
			conf_err(0x0D);
			return 0;
		}
		if(attribute_id == 2)
		{
			if((info[k] != 0x09)&&(info[k+1]!=0x04))
			{
				conf_err(0x0C);
				return 0;
			}
			memcpy(opr_data,info+k+2,4);
			Eprom_Write(TamperSelByteAdd);
		}
		break;
	case 0x0160800b://tamper theshrold list
		if((asso0_flag == 1)||(asso1_flag == 1)||(asso2_flag == 1))
		{
			conf_err(0x0D);
			return 0;
		}
		if(attribute_id == 2)
		{
			if((info[k] != 0x09)&&(info[k+1]!=84))//84 no of byte fix for single phase
			{
				conf_err(0x0C);
				return 0;
			}
			k=k+2;
			fill_oprzero();
			memcpy(opr_data,info+k,14);
			Eprom_Write(ThresholdAdd);
			k=k+14;
			fill_oprzero();
			memcpy(opr_data,info+k,14);
			Eprom_Write(ThresholdAdd+0x10);
			k=k+14;
			fill_oprzero();
			memcpy(opr_data,info+k,14);
			Eprom_Write(ThresholdAdd+0x20);
			k=k+14;
			fill_oprzero();
			memcpy(opr_data,info+k,14);
			Eprom_Write(ThresholdAdd+0x30);
			k=k+14;
			fill_oprzero();
			memcpy(opr_data,info+k,14);
			Eprom_Write(ThresholdAdd+0x40);
			k=k+14;
			fill_oprzero();
			memcpy(opr_data,info+k,14);
			Eprom_Write(ThresholdAdd+0x50);
			k=k+14;
			
		}
		break;
	case 0x00600104:    
		{
			if(asso3_flag == 1)
			{
				if(info[k]!=0x12 )
				{
					conf_err(0x0C);
					return 0;
				}
				k++;
				
				u16tem_yy =(unsigned int)info[k]*256+info[k+1];
				uint_temp = u16tem_yy- 0x07D0;
				UcharTemp = (unsigned char)(uint_temp % 256);
				
				Eprom_Read(FG_DateTimeAdd); 
				opr_data[2]=hex_to_bcd(UcharTemp);
				
				Eprom_Write(FG_DateTimeAdd);
			}
			else
			{
				//fill_0b();
				conf_type_flag = 0x0B; // not present obis case error.
				return 0;
			}
		}
		break;
	case 0x01608012:    
		{
			if(asso3_flag == 1)
			{
				if(info[k]!=0x0a || info[k+1]>14 )
				{
					conf_err(0x0C);
					return 0;
				}
				k++;
				opr_data[0]=info[k++];
				memcpy(opr_data+1,info+k,opr_data[0]);
				Eprom_Write(UtilityIDAdd);
			}
			else
			{
				//fill_0b();
				conf_type_flag = 0x0B; // not present obis case error.
				return 0;
			}
		}
		break;
	case 0x005e5b0c:
		{
			if(asso3_flag == 1)
			{
				if(info[k]!=0x0a || info[k+1]>15 )
				{
					conf_err(0x0C);
					return 0;
				}
				k++;
				k++;
				
				fill_oprzero();
				opr_data[0]=info[k++];
				for( i=1; i<=opr_data[0];++i)
				{
					opr_data[i]  = info[k++] ;  
				}
				Eprom_Write(CurrentRating_addrs); 
			}
			else
			{
				//fill_0b();
				conf_type_flag = 0x0B; // not present obis case error.
				return 0;
			}
		}
		
		break;
		
	default:
		//fill_0b();
		conf_type_flag = 0x0B; // not present obis case error.
		return 0;
	}//swa
	return 1;
}
u8 action_resp(void)
{
	switch(obis_short)
	{//swa

	case 0x000a0000: //global reset script execution
		if((attribute_id==1)&&(asso3_flag==1))
		{
			reset_meter_data_new();
			GetNextDate(dt.min,dt.hour,dt.day,dt.month,dt.year, BILL_a8date_array[0], BILL_a8date_array[1], BILL_a8date_array[4], 0);
		}
		else if((attribute_id == 3) && (asso3_flag==1))
		{
			MASS_READ1(info[16],info[17],info[18]);
		}
		else if((attribute_id == 2) && (asso3_flag==1))
		{
			fill_oprzero();
			Eprom_Write(nonroll_status);
			TP5.b.tc_tpr_f=0;
			TP5.b.tc_tprstr_f=0;
			TP.b.NonRollEvent=0;
			nonroll_of=0;
			if(nonroll_count)
			{
				nonroll_count--;                //if rly malfunction is restored
			}
		}
		if((attribute_id==5)&&(asso3_flag==1))
		{
			if(info[16]!=0x02 || info[17]!=0x02)
			{
				conf_err(0x0C);
				return 0;
			}
			if(info[18]!=0x09 || info[19]!=24)
			{
				conf_err(0x0C);
				return 0;
			}
			
			conf_err(0x0C);
		}
		if((attribute_id==6)&&(asso3_flag==1))
		{
			if(info[16]!=0x09 || info[17]!=24)
			{
				conf_err(0x0C);
				return 0;
			}
			if(strncmp((char *)FSL_Key,(char *)&info[18],24)==0)
			{
					//TXD0 = 0x06;
					PDwrite_detect_f=1;
					StorePDTime();
					save_pd();
					MET_u8CircularPtr=0;
					store_energy();
					stopmode_backup();
					bat_enable;
					disp_vDispString(LCD_BOOT,7);
					Self_Programming_main();
			}
			else
			{
				conf_err(0x0C);
				return 0;
			}

		}

		if((attribute_id == 9) && (asso3_flag==1))
		{
			reset_tamper();           //tamper status 9 pages cleared
		}
		break;


	case 0x000a006c://push object
		// push_ob();
		break;


	case 0x000d0000://activating passive calender by action
		if(attribute_id==1)
		{
			u8ChangeCalnder_flag = 1;
			mri_bill_f = 1;
			//            if(TOD_bActive_Calendar==0)
			//                TOD_bActive_Calendar=1;
			//            else
			//                TOD_bActive_Calendar=0;
			//
			//            TOU_vDeter_Season();
			//
			//            TOD_bzonecng_f0=0;
			//            storezkwh();
			//            TOD_vloadzkwh();
			//            opr_data[0]= TOD_bActive_Calendar;
			//            Eprom_Write(TOU_ActiveCalPtr);
			//            a8Daily_alert_status[3]|=DES_TOU_cng;
			log_trans_event(155);
		}
		break;


	case 0x00280003://HLS set
		if((asso0_flag != 0)||(asso1_flag != 0))// || (asso3_flag != 0))
		{//if9
			conf_err(0x0D);
			return 0;
		}//if9
		if(attribute_id==2)
		{
			if((info[16]!=0x09)||(info[17]!=0x10))
			{
				conf_err(0x0C);
				return 0;
			}

			memcpy(aut_pswd1,&info[18],16);

			memcpy(opr_data,aut_pswd1,15);

			Eprom_Write(DLMS_LLS_PassAdd);//cngd
			opr_data[0]=aut_pswd1[15];
			Eprom_Write(DLMS_HLS_Pass1Add);//cngd
		}
		break;
		
	case 0x01620100:

		if(attribute_id==2)
		{
			mri_bill_f=1;
			//            a8Daily_alert_status[5]|=DES_Bill_MRI;
			log_trans_event(181);// changed to 181 as per BLA
		}
		break;

	default:
		return 0;
	}
	return 1;
}
void SaveDisplayList(u16 addrr,u8 num,u8 auto_push)
{
	u8 tem;
	u8 tem1;
	k=k+2;
	for(tem=0;tem<num;tem=tem+15)
	{
		fill_oprzero();
		for(tem1=0;tem1<15;tem1++)
		{
			opr_data[tem1]=info[k++];
		}
		Eprom_Write(addrr);
		addrr=addrr+0x10;
	}
	Eprom_Read(lcd_Auto_Push_NO_Addr);
	if(auto_push=='p') 
	{
		opr_data[1]=num;
	}else
	{
		opr_data[0]=num;
	}
	Eprom_Write(lcd_Auto_Push_NO_Addr);
	
}

u8 save_tou_pass_data(u16 u8index_t, u8 *info_t, u8 buffer_trace)
{
	u8 u8loop_tou;
	u16 u16addr;
	u8 temp_buf_trace;
	u16 i;
	//u16 temp_tariff;
	
	//    if(tou_u8pssv_no_days == 1)
	//        week1_set = 0;
	
	if(buffer_trace == 1)                                           //doubt in case if buffer value is less than 19 in starting
	if(tou_u8pssv_buffer_traced>0)
	temp_buf_trace = 1;
	else 
	temp_buf_trace = 0;
	else
	temp_buf_trace = 1;
	
//	if(tou_u8pssv_no_zone>Tarriff_slots)
//	return 0;
	
	for(u8loop_tou=tou_u8pssv_up_zone;((u8loop_tou<tou_u8pssv_no_zone)&& temp_buf_trace);u8loop_tou++)// 
	{
		if((info_t[u8index_t]==0x02)&&(info_t[u8index_t+1]==0x03)&&(info_t[u8index_t+2]==0x09)&&(info_t[u8index_t+3]==0x04)
				&&(info_t[u8index_t+8]==0x09)&&(info_t[u8index_t+9]==0x06)&&(info_t[u8index_t+16]==0x12))
		{
			if(info_t[u8index_t+4]>23)//hour
			{
				return 0; 
			}
			if(info_t[u8index_t+5]>59)//min
			{
				return 0; 
			}
			
			if(info_t[u8index_t+18]>Tarriff_slots)
			{
				return 0;
			}
					
			tou_a8pssv_zone_time[tou_u8pssv_ptr]=info_t[u8index_t+4];//hour
			tou_a8pssv_zone_time[tou_u8pssv_ptr+1]=info_t[u8index_t+5];//min
			tou_a8pssv_traiff[tou_u8pssv_up_zone]=info_t[u8index_t+18];//tariff
			if(buffer_trace == 1)
			{
				tou_u8pssv_buffer_traced-=19;
				if(tou_u8pssv_buffer_traced>0)
				temp_buf_trace = 1;
				else 
				temp_buf_trace = 0;
			}
			else
			{
				temp_buf_trace = 1;
			}
			u8index_t+=19;
			tou_u8pssv_ptr+=2;
			tou_u8pssv_up_zone++;
			week1_set = 0;
		}
		else
		{
			return 0;
		}
		if(buffer_trace == 1)
		{
			if(((tou_u8pssv_buffer_traced<19)||(u8loop_tou == (tou_u8pssv_no_zone-1)))&& (tou_u8pssv_buffer_traced!=0))
			{
				memcpy(info2,&info_t[u8index_t],tou_u8pssv_buffer_traced);
				u8index_t = 0;
				if(u8loop_tou == (tou_u8pssv_no_zone-1))
				u8loop_tou++;
				
				break;
			}
			
			
		}
	}
	if((u8loop_tou==tou_u8pssv_no_zone) && (week1_set == 0))
	{
		//save tod
		if(TOD_bActive_Calendar==1)
		u16addr=TOU_DAY_ACTIVE_ADD+tou_u8pssv_day*0x30;
		else
		u16addr=TOU_DAY_ACTIVE_ADD+0X60+tou_u8pssv_day*0x30;
		fill_oprzero();
		opr_data[0]=tou_u8pssv_no_zone;
		memcpy(opr_data+1,tou_a8pssv_traiff,tou_u8pssv_no_zone);
		opr_data[13]=tou_u8pssv_no_days;
		opr_data[14]=tou_u8pssv_dayid;
		Eprom_Write(u16addr);
		fill_oprzero();
		for( i = 0; i < 16; ++i)
		{
			tou_a8pssv_zone_time[i]=hex_to_bcd(tou_a8pssv_zone_time[i]);
		}
		memcpy(opr_data,tou_a8pssv_zone_time,14);
		Eprom_Write(u16addr+0x10);
		fill_oprzero();
		memcpy(opr_data,tou_a8pssv_zone_time+14,2);
		Eprom_Write(u16addr+0x20);
		u8loop_tou = 0;
		week1_set=1;
		//save tod
	}
	if(buffer_trace == 1)
	{
		if(tou_u8pssv_buffer_traced>24)
		temp_buf_trace = 1;
		else
		{ 
			temp_buf_trace = 0;
			//            u8index_t = 0;
		}
	}
	else
	temp_buf_trace = 1;
	
	if((tou_u8pssv_no_days>1)&& temp_buf_trace )
	{
		if((info_t[u8index_t]==0x02)&&(info_t[u8index_t+1]==0x02)&&(info_t[u8index_t+2]==0x11)&&(info_t[u8index_t+4]==0x01))
		{
			tou_u8pssv_dayid=info_t[u8index_t+3];
			tou_u8pssv_no_zone=info_t[u8index_t+5];
			
//			if(tou_u8pssv_no_zone>Tarriff_slots)
//			return 0;
	
			if(buffer_trace == 1)
			tou_u8pssv_buffer_traced-=6;
			u8index_t+=6;
			tou_u8pssv_ptr=0;
			tou_u8pssv_up_zone=0;
			tou_u8pssv_day=1;
			memzero(tou_a8pssv_zone_time,16);
			memzero(tou_a8pssv_traiff,8);
			
			if(buffer_trace == 1)                                           //doubt in case if buffer value is less than 19 in starting
			if(tou_u8pssv_buffer_traced>0)
			temp_buf_trace = 1;
			else 
			temp_buf_trace = 0;
			else
			temp_buf_trace = 1;
			for(u8loop_tou=0;((u8loop_tou<tou_u8pssv_no_zone));u8loop_tou++)// && temp_buf_trace
			{
				if((info_t[u8index_t]==0x02)&&(info_t[u8index_t+1]==0x03)&&(info_t[u8index_t+2]==0x09)&&(info_t[u8index_t+3]==0x04)
						&&(info_t[u8index_t+8]==0x09)&&(info_t[u8index_t+9]==0x06)&&(info_t[u8index_t+16]==0x12))
				{
					if(info_t[u8index_t+4]>23)//hour
					{
						return 0; 
					}
					if(info_t[u8index_t+5]>59)//min
					{
						return 0; 
					}
					if(info_t[u8index_t+18]>Tarriff_slots)
					{
						return 0;
					}
					
					tou_a8pssv_zone_time[tou_u8pssv_ptr]=info_t[u8index_t+4];//hour
					tou_a8pssv_zone_time[tou_u8pssv_ptr+1]=info_t[u8index_t+5];//min
					tou_a8pssv_traiff[tou_u8pssv_up_zone]=info_t[u8index_t+18];//tariff
					if(buffer_trace == 1)
					{
						tou_u8pssv_buffer_traced-=19;                                                                  //doubt in case if buffer value is less than 19 in starting
						if(tou_u8pssv_buffer_traced>0)
						temp_buf_trace = 1;
						else 
						temp_buf_trace = 0;
					}
					else
					temp_buf_trace = 1;
					
					u8index_t+=19;
					tou_u8pssv_ptr+=2;
					tou_u8pssv_up_zone++;
				}
				else
				{
					return 0;
				}
				if(buffer_trace == 1)
				if((tou_u8pssv_buffer_traced<19) && (tou_u8pssv_buffer_traced!=0))////day id 2 is last day so (u8loop_tou == (tou_u8pssv_no_zone-1) not required
				{
					memcpy(info2,&info_t[u8index_t],tou_u8pssv_buffer_traced);
					u8index_t = 0;
					break;
				}
			}
			if(u8loop_tou==tou_u8pssv_no_zone)
			{
				//save tod
				if(TOD_bActive_Calendar==1)
				u16addr=TOU_DAY_ACTIVE_ADD+tou_u8pssv_day*0x30;
				else
				u16addr=TOU_DAY_ACTIVE_ADD+0X60+tou_u8pssv_day*0x30;
				fill_oprzero();
				opr_data[0]=tou_u8pssv_no_zone;
				memcpy(opr_data+1,tou_a8pssv_traiff,tou_u8pssv_no_zone);
				opr_data[13]=tou_u8pssv_no_days;
				opr_data[14]=tou_u8pssv_dayid;
				Eprom_Write(u16addr);
				fill_oprzero();
				for( i = 0; i < 16; ++i)
				{
					tou_a8pssv_zone_time[i]=hex_to_bcd(tou_a8pssv_zone_time[i]);
				}
				memcpy(opr_data,tou_a8pssv_zone_time,14);
				Eprom_Write(u16addr+0x10);
				fill_oprzero();
				memcpy(opr_data,tou_a8pssv_zone_time+14,2);
				Eprom_Write(u16addr+0x20);
				
				//save tod
			}
		}
	}
	
	if(buffer_trace == 1)
	return 2;
	else
	return 1;
}


