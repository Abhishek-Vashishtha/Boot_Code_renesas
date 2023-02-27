/****************************************************************************************************************************



*****************************************************************************************************************************/
#include "iodefine.h"
#include "r_cg_serial.h"
#include "fsl_types.h"
#include "fsl.h"
#include "Flash_Protocol.h"
#include "ClockSetting.h"
#include "lcd.h"

#define FLASH_SIZE	128
#define X1_FREQ		3
#define LCD_TYPE	3

#define LCD_b       0xf2
#define LCD_o       0xcc
#define LCD_t       0xf0

/*********** Hard Coded Constants ************************/
#define BIT0					0x01
#define BIT1					0x02
#define BIT2					0x04
#define BIT3					0x08
#define BIT4					0x10
#define BIT5					0x20
#define BIT6					0x40
#define BIT7					0x80

typedef unsigned long int u32;
typedef signed long int s32;
typedef unsigned int u16;
typedef int s16;
typedef unsigned char u8;
typedef char s8;
typedef signed char sc8;
typedef double f32;

u8 FSL_Blank[1034];
u8 FSL_Serial_counter;
u8 FSL_TX_Data_Pkt_length;
u8 Flash_write_flag;
u8 FSL_Analyse_Data_Pkt_f;
u8 flash_complete;
u8 WriteBlock1;
u8 FSL_Reset_f;
u16 increment;
u16 Reset_Add ;
u16 FSL_RX_Data_Pkt_length;
u16 FSL_Serial_rxcounter;
u32 Write_addr;
u32 Write_addr_temp;
u8 FSL_data_array[262];

//#pragma section text ram_text

__far void Self_Programming_main(void);
__far void Self_Programming_Read(void);
__far void Self_Programming_Verify(void);
__far void Self_Programming_SwapCluster(void);
__far void Self_Programming_Reset(void);
__far void Self_Programming_UART_Init(void);
__far void Self_Programming_UART_Data_Recieve(void);
__far void Self_Programming_UART_Data_Transmit(void);
__far void Self_Programming_Protocol_Analyse_Pkt(void);
__far void Self_Programming_Protocol_Prepare_Pkt(unsigned char Pkt_ID);
__far void FSL_delay(unsigned int count);
__far unsigned char Self_Programming_Init(void);
__far unsigned char Self_Programming_Write(unsigned long int Write_data);


void clear_ram(void)
{
	Write_addr=0;
	Write_addr_temp=0;
	increment=0;
	Reset_Add=0;
	FSL_RX_Data_Pkt_length=0;
	FSL_Serial_rxcounter=0;
	WriteBlock1=0;
	FSL_Serial_counter=0;
	FSL_TX_Data_Pkt_length=0;
	Flash_write_flag=0;
	FSL_Analyse_Data_Pkt_f=0;
	flash_complete=0;
}

void CLK_vClockSet(void)
{
    volatile u16 w_count;

    /* Set fMX */
   	CMC = _40_CGC_HISYS_OSC | _10_CGC_SUB_OSC | _01_CGC_SYSOSC_OVER10M | _04_CGC_SUBMODE_ULOW;//_00_CGC_SUBMODE_LOW;//_02_CGC_SUBMODE_NORMAL;
    XTSTOP = 0U;
    /* Change the waiting time according to the system */
    for (w_count = 0U; w_count <= CGC_SUBWAITTIME; w_count++)
    {
        __nop();
    }    
    
    OSMC = _00_CGC_SUBINHALT_ON | _00_CGC_RTC_CLK_FSUB;

	HIOSTOP = 0U;
	HOCODIV = X1_FREQ;//0x03;//0 = 24,1=12,2=6,3=3
	/* Set fCLK */
	CSS = 0U;//Main system clock (fMAIN)
	MCM0 = 0U;//Selects the high-speed system clock (fMX) as the main system clock (fMAIN)
	/* Stop fMX */
	MSTOP = 1U;
}

void hdwinit(void)
{
    CRC0CTL = 0x80U;
    IAWCTL = 0x80U;
}

void R_WDT_Restart(void)
{
	WDTE = 0xACU;
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
* Function Name: R_LCD_Set_VoltageOn
* Description  : This function enables voltage boost circuit or capacitor split circuit.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void LCD_vSetVoltageOn(void)
{
    volatile u32 wt_count1;
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
* Function Name: R_LCD_Create
* Description  : This function initializes the LCD module.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void LCD_vInit(void)
{
    volatile u32 wt_count;
    
 	OSMC |= 0x80;  /* Supply lock to LCD */
    LCDON = 0U;    /* disable LCD clock operation */
    LCDM1 |= _00_LCD_VOLTAGE_HIGH;
    LCDM0 = _00_LCD_DISPLAY_WAVEFORM_A | _0D_LCD_DISPLAY_MODE1;
    LCDM0 |= _40_LCD_VOLTAGE_MODE_INTERNAL;
    /* Set CAPL and CAPH pins */
    ISCLCD &= (u8)~_01_LCD_CAPLH_BUFFER_VALID;
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
	
    LCDC0 = _00_LCD_SOURCE_CLOCK_FSUB | _05_LCD_CLOCK_FLCD_64;

    VLCD = _04_LCD_BOOST_VOLTAGE_100V;
	R_WDT_Restart();
	for (wt_count = 0U; wt_count <= LCD_REFVOLTAGE_WAITTIME; wt_count++)
	{
		__nop();
	}    
    LCDVLM = 1;
	LCD_vSetVoltageOn();
	LCD_vStart();
}

#if LCD_TYPE == 1
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
#endif

#if LCD_TYPE == 2
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
#endif

#if LCD_TYPE == 3
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
#endif

void Self_Programming_LCD_Init(void)
{
	LCD_vInit();
	
	LCD_vDispChar(LCD_b,6);
	LCD_vDispChar(LCD_o,5);
	LCD_vDispChar(LCD_o,4);
	LCD_vDispChar(LCD_t,3);
}


int main(void)
{
	__DI();
	
	/* Disables RAM Parity Resets */
	RPERDIS = 1;
	
	/* Initialise Clock Frequency */
	CLK_vClockSet();
	
	/* Clear ram Variables */
	clear_ram();
	
	/* Initialise LCD and display Boot */
	Self_Programming_LCD_Init();

	/* Switch to Boot Mode */
	Self_Programming_main();
}


__far void Self_Programming_UART_Init(void)
{
	/* Configure LCD Segment Port for Input/Output*/
	PFSEG4 &=0xc0;
    PFSEG3 &= 0xF0;
	
	SAU0EN = 1;//enable module     
	__nop();
	__nop();
	__nop();
	__nop();
	
#if X1_FREQ == 0	
	SPS0 =  _0004_SAU_CK00_FCLK_4 | _0070_SAU_CK01_FCLK_7;//_000A_SAU_CK00_FCLK_10;	
#elif X1_FREQ == 1	
	SPS0 =  _0003_SAU_CK00_FCLK_3 | _0070_SAU_CK01_FCLK_7;//_000A_SAU_CK00_FCLK_10;
#elif X1_FREQ == 2
	SPS0 =  _0002_SAU_CK00_FCLK_2 | _0070_SAU_CK01_FCLK_7;//_000A_SAU_CK00_FCLK_10;
#elif X1_FREQ == 3
	SPS0 =  _0001_SAU_CK00_FCLK_1 | _0070_SAU_CK01_FCLK_7;//_000A_SAU_CK00_FCLK_10;
#endif
	ST0 |= _0002_SAU_CH1_STOP_TRG_ON | _0001_SAU_CH0_STOP_TRG_ON | _0004_SAU_CH2_STOP_TRG_ON | _0008_SAU_CH3_STOP_TRG_ON ;	/* disable UART2 receive and transmit */
	
	
	STMK0 = 1U;	/* disable INTST0 interrupt TX*/
	STIF0 = 0U;	/* clear INTST0 interrupt flag TX*/
	SRMK0 = 1U;	/* disable INTSR0 interrupt RX*/
	SRIF0 = 0U;	/* clear INTSR0 interrupt flag RX*/
	SREMK0 = 1U;	/* disable INTSRE0 interrupt Err*/
	SREIF0 = 0U;	/* clear INTSRE0 interrupt flag Err*/
	
	/* Set INTST0 low priority */
	STPR00 = 1U;
	STPR10 = 1U;
	/* Set INTSR0 low priority */
	SRPR00 = 1U;
	SRPR10 = 1U;
	/* Set INTSRE0 low priority */
	SREPR00 = 1U;
	SREPR10 = 1U;
	
	SMR00 = _0020_SAU_SMRMN_INITIALVALUE | _0000_SAU_CLOCK_SELECT_CK00 | _0000_SAU_TRIGGER_SOFTWARE | _0002_SAU_MODE_UART | _0000_SAU_TRANSFER_END;
	SCR00 = _8000_SAU_TRANSMISSION | _0000_SAU_INTSRE_MASK | _0000_SAU_PARITY_NONE | _0080_SAU_LSB + _0010_SAU_STOP_1 + _0007_SAU_LENGTH_8;
	SDR00 = _9A00_UART2_TRANSMIT_DIVISOR;
	NFEN0 |= _01_SAU_RXD0_FILTER_ON;
	SIR01 = _0004_SAU_SIRMN_FECTMN | _0002_SAU_SIRMN_PECTMN | _0001_SAU_SIRMN_OVCTMN;	/* clear error flag */
	SMR01 = _0020_SAU_SMRMN_INITIALVALUE | _0000_SAU_CLOCK_SELECT_CK00 | _0100_SAU_TRIGGER_RXD | _0000_SAU_EDGE_FALL | _0002_SAU_MODE_UART | _0000_SAU_TRANSFER_END;
	SCR01 = _4000_SAU_RECEPTION | _0400_SAU_INTSRE_ENABLE | _0000_SAU_PARITY_NONE | _0080_SAU_LSB | _0010_SAU_STOP_1 | _0007_SAU_LENGTH_8;
	SDR01 = _9A00_UART2_RECEIVE_DIVISOR;	
	
	SO0 |= _0001_SAU_CH0_DATA_OUTPUT_1;	
	SOL0 |= _0000_SAU_CHANNEL0_NORMAL;	/* output level normal */
	SOE0 |= _0001_SAU_CH0_OUTPUT_ENABLE;	/* enable UART2 output */
		
	/* Set RxD0 pin */
	PM0 |= BIT6;    //set 1 for input and 0 for output
	PIM0 |= BIT6;
	PM3 &= ~BIT0;
	P3 &= ~BIT0;
	/* Set TxD0 pin */
	P0 |= BIT7;
	PM0 &= ~BIT7;

	STIF0 = 0U;	/* clear INTST2 interrupt flag */
	STMK0 = 1U;	/* enable INTST2 interrupt */
	SRIF0 = 0U;	/* clear INTSR2 interrupt flag */
	SRMK0 = 1U;	/* enable INTSR2 interrupt */
	SREIF0 = 0U;	/* clear INTSRE2 interrupt flag */
	SREMK0 = 1U;	/* enable INTSRE2 interrupt */
	SO0 |= _0001_SAU_CH0_DATA_OUTPUT_1;	/* output level normal */
	SOE0 |= _0001_SAU_CH0_OUTPUT_ENABLE;	/* enable UART2 output */
	SS0 |= _0002_SAU_CH1_START_TRG_ON | _0001_SAU_CH0_START_TRG_ON;	/* enable UART2 receive and transmit */	
}

__far void Self_Programming_main(void)
{
	unsigned char FSL_Status_byte=0;
	
	/* Disable Interrupt Routines */
	__DI();
	
	/* Initialize UART Channel for Communication */
	Self_Programming_UART_Init();

	/* Intialize Self Programming */
	FSL_Status_byte = Self_Programming_Init();
	
	if(FSL_Status_byte != 0)
	{
		return;
	}
	
	while(1)
    {
		 WDTE = 0xACU;
		/* Check for Data Reception */
		Self_Programming_UART_Data_Recieve();
		
		if(FSL_Analyse_Data_Pkt_f==1)
		{
			Self_Programming_Protocol_Analyse_Pkt();
		}
	}
}

__far unsigned char Self_Programming_Init(void)
{
    unsigned char Init_Status=1; 
	
	__far fsl_descriptor_t InitArg;
    //fsl_fsw_t ShieldWindow;

    /* Set argument of FSL_Init()	 */
    InitArg.fsl_flash_voltage_u08     = 0x00; //FULL_SPEED_MODE;
    InitArg.fsl_frequency_u08         = 0x03;  //FREQUENCY_32M;
    InitArg.fsl_auto_status_check_u08 = 0x01; //INTERNAL_MODE;
	
    /* Initialize Flash Library Functions **/
    Init_Status = FSL_Init(&InitArg);  

    if(Init_Status == FSL_OK)
    {  
		FSL_Open();                                                   
		FSL_PrepareFunctions();                                              
		FSL_PrepareExtFunctions();  
    }
    
	return Init_Status;
}

__far void Self_Programming_UART_Data_Recieve(void)
{
	unsigned long int count = 0;
	
	while(SRIF0==0)
	{
		 count++;
		 if(count > 1000000)
		 {
		 	FSL_Serial_rxcounter = 0;
			count = 0;
			while(1);
		 }
		 else
		 {
			 WDTE = 0xACU;
		 }
	}
	SRIF0=0;
	SREIF0=0;
	count=0;
	
	FSL_data_array[FSL_Serial_rxcounter] =RXD0;
	FSL_Serial_rxcounter++ ;
	
	if(FSL_data_array[0]==0x01)
	{
		FSL_RX_Data_Pkt_length=4;
		
		if(FSL_data_array[3]==0x05)
		{
			FSL_RX_Data_Pkt_length=11;
		}
		else if((FSL_data_array[3]==0x0b) || (FSL_data_array[3]==0x08) || (FSL_data_array[3]==0x07))
		{
			FSL_RX_Data_Pkt_length=6;
		}
		else if(FSL_data_array[3]==0x06)
		{
			FSL_RX_Data_Pkt_length=261;
		}
	}
	else
	{
		FSL_RX_Data_Pkt_length = 0;
	}
	
	if(FSL_Serial_rxcounter>=FSL_RX_Data_Pkt_length)
  	{	 						
        FSL_Serial_rxcounter=0;
		
		if((FSL_RX_Data_Pkt_length==6) || (FSL_RX_Data_Pkt_length==11) || (FSL_RX_Data_Pkt_length==261)) // || (FSL_RX_Data_Pkt_length==133) || (FSL_RX_Data_Pkt_length==16))
		{
			FSL_Analyse_Data_Pkt_f=1;
			//FSL_RX_Data_Pkt_length = 0 ;
		}
		else
		{
			FSL_RX_Data_Pkt_length=0;  
		}
  	}
}

__far unsigned char Verify_ChkSum(unsigned char *chkdata,unsigned int nob)
{
	unsigned int i,chksum=0;
  
	for(i=0;i<nob;i++)
	{
		chksum += *chkdata++;
	}
	
	Write_addr_temp = nob;
	return(chksum);
}

__far void Self_Programming_Protocol_Analyse_Pkt(void)
{
	u8 Command_key=0;
	
	FSL_Analyse_Data_Pkt_f = 0 ;
	
	if(FSL_data_array[3] == 0x06)
	{
		__nop();
	}
	
	if(Verify_ChkSum(&FSL_data_array[0],FSL_RX_Data_Pkt_length-1) != FSL_data_array[FSL_RX_Data_Pkt_length-1])
	{
		FSL_RX_Data_Pkt_length = 0 ;
		return;
	}
	Command_key = FSL_data_array[3];
	
	Self_Programming_Protocol_Prepare_Pkt(Command_key);
}

__far void Self_Programming_Protocol_Prepare_Pkt(unsigned char data_ID)
{
	unsigned int Satus_check = 0 ; 
	
	FSL_Reset_f = 0;
	
	FSL_data_array[0] = 0x01;
	FSL_data_array[1] = 0x00;
	FSL_data_array[2] = 0x03;

	switch(data_ID)
	{
		case 0x05:
			WriteBlock1 = FSL_data_array[4]; 
			//Write_addr = (Write_addr<<8) + FSL_data_array[7];
			//WriteBlock1 = Write_addr;	///0x400 ;
			
			if(WriteBlock1 < FLASH_SIZE)
			{
				Satus_check = FSL_Erase(WriteBlock1); 
				if(Satus_check == FSL_OK)
				{
					Flash_write_flag =1;
					increment = 0;
				}
			}
			
			FSL_data_array[3]=data_ID ;
			FSL_data_array[4]=Satus_check;
			FSL_data_array[5]=0x09;
			FSL_TX_Data_Pkt_length=6;
		break;
		
		case 0x0b: 
			Flash_write_flag = 0;
			//WriteBlock1 = Write_addr; ///0x400;
			Satus_check = FSL_IVerify(WriteBlock1);
		    FSL_data_array[3]=data_ID ;
			FSL_data_array[4]=Satus_check;
			FSL_data_array[5]=0x0f;
			FSL_TX_Data_Pkt_length=6;
		break;
		
		case 0x06:
			if(WriteBlock1 < FLASH_SIZE)
			{
				Write_addr = (u32)WriteBlock1*0x0400;
				Satus_check = Self_Programming_Write(Write_addr);
			}
			FSL_delay(50);
			FSL_data_array[3]=data_ID ;
			FSL_data_array[4]=Satus_check;
			if(Satus_check	== 0x05)
			{
				__nop();
			}
			FSL_data_array[5]=0x0a;
			FSL_TX_Data_Pkt_length=6;
		break;
		
		case 0x08: 
			Satus_check = FSL_InvertBootFlag(); 
			FSL_delay(100);
			FSL_Reset_f=1;
		    FSL_data_array[3]=data_ID ;
			FSL_data_array[4]=Satus_check;
			FSL_data_array[5]=0x0c;
			FSL_TX_Data_Pkt_length=6;
		break;
		
		
		case 0x07: 
			FSL_Reset_f=1;
		    FSL_data_array[3]=data_ID ;
			FSL_data_array[4]=0x00;
			FSL_data_array[5]=0x0b;
			FSL_TX_Data_Pkt_length=6;
		break;
		
		default:  
		     __nop();
		break;
	}
	 
	FSL_Serial_counter=0;
	Self_Programming_UART_Data_Transmit();
	
	if(FSL_Reset_f==1)
	{
		FSL_Reset_f = 0;
		FSL_delay(100);
		FSL_ForceReset();
	}
}

__far void Self_Programming_UART_Data_Transmit(void)
{	
	unsigned long int count = 0;
	
	while(FSL_Serial_counter<FSL_TX_Data_Pkt_length)
	{
       TXD0 = FSL_data_array[FSL_Serial_counter];
       while(STIF0 == 0)
	   {
		 WDTE = 0xACU;
		 count++;
		 if(count > 400000)
		 {
		 	FSL_Serial_rxcounter = 0;
			count = 0;
			while(1);
		 }
	   }
       STIF0=0;
       FSL_Serial_counter++;
  	}
	
	FSL_Serial_counter=0; 
}


__far unsigned char Self_Programming_Write(unsigned long int Write_data)
{
    unsigned char status, count;
    fsl_write_t WriteArg;

    if(Flash_write_flag == 1)
    {
		count = 0x40;
		Write_addr_temp = Write_addr + increment;
		/* ---- Set argument of FSL_Write() ---- */
		WriteArg.fsl_data_buffer_p_u08       = &FSL_data_array[4];
		WriteArg.fsl_destination_address_u32 = Write_addr + increment;
		WriteArg.fsl_word_count_u08          = count;

		status = FSL_Write(&WriteArg);    /* Write execution */

		if(status == FSL_OK)
		{
			increment= increment + 0x100 ;
			if(increment>0x100)
			{
				__nop();
			}
		}
	}
	else
	{
	   	__nop();
	}
    
    return status;
}


/**** 1 for 1 ms ****/
__far void FSL_delay(unsigned int count)
{
	unsigned int i,k;
	
	for(i=0; i<count; i++)
	{
		for(k=0;k<510;k++)
		{
			__nop();
		}
	}
	
}


