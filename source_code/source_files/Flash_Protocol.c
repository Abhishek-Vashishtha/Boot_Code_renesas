/***********************************************************************************************************************
* File Name       : Flash_Protocol.c
* Current Version : rev_01  
* Tool-Chain      : IAR Systems RL78
* Description     : this file will have all the routines related to tou
* Creation Date   : 18-12-2020
* Company         : Genus Power Infrastructures Limited, Jaipur
* Author          : dheeraj.singhal
* Version History : rev_01 :
***********************************************************************************************************************/
/************************************ Includes **************************************/
#include "Flash_Protocol.h"
/************************************ Local Variables *****************************************/
/************************************ Extern Variables *****************************************/
/************************************ Local Functions *******************************/
/************************************ Extern Functions ******************************/


#define FLASH_SIZE	256

#define RJ_45          (0u)
#define OPTICAL_PORT   (1u)
#define UART_TYPE      OPTICAL_PORT

us8 FSL_data_array[262];
us32 Write_addr;
us32 Write_addr_temp;
us16 increment;
us16 Reset_Add ;
us16 FSL_RX_Data_Pkt_length;
us16 FSL_Serial_rxcounter;
us16 WriteBlock1;
us8 FSL_Serial_counter;
us8 FSL_TX_Data_Pkt_length;
us8 Flash_write_flag;
us8 FSL_Analyse_Data_Pkt_f;
us8 flash_complete;
us8 FSL_Reset_f;



//#pragma section text ram_text

__near_func void Self_Programming_main(void);
__near_func void Self_Programming_Read(void);
__near_func void Self_Programming_Verify(void);
__near_func void Self_Programming_SwapCluster(void);
__near_func void Self_Programming_Reset(void);
__near_func void Self_Programming_UART_Init(void);
__near_func void Self_Programming_UART_Data_Recieve(void);
__near_func void Self_Programming_UART_Data_Transmit(void);
__near_func void Self_Programming_Protocol_Analyse_Pkt(void);
__near_func void Self_Programming_Protocol_Prepare_Pkt(unsigned char Pkt_ID);
__near_func void FSL_delay(unsigned int count);
__near_func unsigned char Self_Programming_Init(void);
__near_func unsigned char Self_Programming_Write(unsigned long int Write_data);


__near_func void Self_Programming_UART_Init(void)
{
#if UART_TYPE == RJ_45
  SAU0EN = 1U;    /* enables input clock supply */
  NOP();
  NOP();
  NOP();
  NOP();
  if(clock_select == CLOCK_24MHZ)
  {
    SPS0 = _0040_SAU_CK01_fCLK_4 | _0004_SAU_CK00_fCLK_4;
  }
  else if(clock_select == CLOCK_12MHZ)
  {
    SPS0 = _0030_SAU_CK01_fCLK_3 | _0003_SAU_CK00_fCLK_3;
  }
  else if(clock_select == CLOCK_6MHZ)
  {
    SPS0 = _0020_SAU_CK01_fCLK_2 | _0002_SAU_CK00_fCLK_2;
  }
  else if(clock_select == CLOCK_1_5MHZ)
  {
    SPS0 = _0000_SAU_CK01_fCLK_0 | _0000_SAU_CK00_fCLK_0;
  }


  ST0 |= _0008_SAUm_CH3_STOP_TRG_ON | _0004_SAUm_CH2_STOP_TRG_ON;
  STMK1 = 1U;     /* disable INTST1 interrupt */
  STIF1 = 0U;     /* clear INTST1 interrupt flag */
  SRMK1 = 1U;     /* disable INTSR1 interrupt */
  SRIF1 = 0U;     /* clear INTSR1 interrupt flag */
  SREMK1 = 1U;    /* disable INTSRE1 interrupt */
  SREIF1 = 0U;    /* clear INTSRE1 interrupt flag */
  /* Set INTSR1 low priority */
  SRPR11 = 1U;
  SRPR01 = 1U;
  /* Set INTSRE1 low priority */
  SREPR11 = 1U;
  SREPR01 = 1U;
  /* Set INTST1 low priority */
  STPR11 = 1U;
  STPR01 = 1U;
  SMR02 = _0020_SMR02_DEFAULT_VALUE | _0000_SAU_CLOCK_SELECT_CK00 | _0000_SAU_CLOCK_MODE_CKS | 
    _0002_SAU_MODE_UART | _0000_SAU_TRANSFER_END;
  SCR02 = _0004_SCR02_DEFAULT_VALUE | _8000_SAU_TRANSMISSION | _0000_SAU_TIMING_1 | _0000_SAU_INTSRE_MASK | 
    _0000_SAU_PARITY_NONE | _0080_SAU_LSB | _0010_SAU_STOP_1 | _0003_SAU_LENGTH_8;
  SDR02 = _9A00_SAU0_CH2_BAUDRATE_DIVISOR;
  NFEN0 |= _04_SAU_RXD1_FILTER_ON;
  SIR03 = _0004_SAU_SIRMN_FECTMN | _0002_SAU_SIRMN_PECTMN | _0001_SAU_SIRMN_OVCTMN;
  SMR03 = _0020_SMR03_DEFAULT_VALUE | _0000_SAU_CLOCK_SELECT_CK00 | _0000_SAU_CLOCK_MODE_CKS | 
    _0100_SAU_TRIGGER_RXD | _0000_SAU_EDGE_FALL | _0002_SAU_MODE_UART | _0000_SAU_TRANSFER_END;
  SCR03 = _0004_SCR03_DEFAULT_VALUE | _4000_SAU_RECEPTION | _0000_SAU_TIMING_1 | _0400_SAU_INTSRE_ENABLE | 
    _0000_SAU_PARITY_NONE | _0080_SAU_LSB | _0010_SAU_STOP_1 | _0003_SAU_LENGTH_8;
  SDR03 = _9A00_SAU0_CH3_BAUDRATE_DIVISOR;
  SO0 |= _0004_SAUm_CH2_DATA_OUTPUT_1;
  SOL0 &= (uint16_t)~_0004_SAUm_CHANNEL2_INVERTED;
  SOE0 |= _0004_SAUm_CH2_OUTPUT_ENABLE;
  
  
  SO0 |= _0004_SAUm_CH2_DATA_OUTPUT_1;
  SOE0 |= _0004_SAUm_CH2_OUTPUT_ENABLE;
  SS0 |= _0008_SAUm_CH3_START_TRG_ON | _0004_SAUm_CH2_START_TRG_ON;
  STIF1 = 0U;     /* clear INTST1 interrupt flag */
  SRIF1 = 0U;     /* clear INTSR1 interrupt flag */
  SREIF1 = 0U;    /* clear INTSRE1 interrupt flag */
  STMK1 = 0U;     /* enable INTST1 interrupt */
  SRMK1 = 0U;     /* enable INTSR1 interrupt */
  SREMK1 = 0U;    /* enable INTSRE1 interrupt */
    /* pending, there is slight vairation in uart code. check it if dont work */	
#else
  SAU1EN = 1U;    /* enables input clock supply */
  NOP();
  NOP();
  NOP();
  NOP();
  if(clock_select == CLOCK_24MHZ)
  {
    SPS1 = _0040_SAU_CK01_fCLK_4 | _0004_SAU_CK00_fCLK_4;
  }
  else if(clock_select == CLOCK_12MHZ)
  {
    SPS1 = _0030_SAU_CK01_fCLK_3 | _0003_SAU_CK00_fCLK_3;
  }
  else if(clock_select == CLOCK_6MHZ)
  {
    SPS1 = _0020_SAU_CK01_fCLK_2 | _0002_SAU_CK00_fCLK_2;
  }
  else if(clock_select == CLOCK_1_5MHZ)
  {
    SPS1 = _0000_SAU_CK01_fCLK_0 | _0000_SAU_CK00_fCLK_0;
  }


  ST1 |= _0002_SAUm_CH1_STOP_TRG_ON | _0001_SAUm_CH0_STOP_TRG_ON;
  STMK2 = 1U;     /* disable INTST2 interrupt */
  STIF2 = 0U;     /* clear INTST2 interrupt flag */
  SRMK2 = 1U;     /* disable INTSR2 interrupt */
  SRIF2 = 0U;     /* clear INTSR2 interrupt flag */
  SREMK2 = 1U;    /* disable INTSRE2 interrupt */
  SREIF2 = 0U;    /* clear INTSRE2 interrupt flag */
  /* Set INTSR2 low priority */
  SRPR12 = 1U;
  SRPR02 = 1U;
  /* Set INTSRE2 low priority */
  SREPR12 = 1U;
  SREPR02 = 1U;
  /* Set INTST2 low priority */
  STPR12 = 1U;
  STPR02 = 1U;
  SMR10 = _0020_SMR10_DEFAULT_VALUE | _0000_SAU_CLOCK_SELECT_CK00 | _0000_SAU_CLOCK_MODE_CKS | 
    _0002_SAU_MODE_UART | _0002_SAU_MODE_UART | _0000_SAU_TRANSFER_END;
  SCR10 = _0004_SCR10_DEFAULT_VALUE | _8000_SAU_TRANSMISSION | _0000_SAU_TIMING_1 | _0000_SAU_INTSRE_MASK | 
    _0000_SAU_PARITY_NONE | _0080_SAU_LSB | _0010_SAU_STOP_1 | _0003_SAU_LENGTH_8;
  SDR10 = _9A00_SAU1_CH0_BAUDRATE_DIVISOR;
  NFEN0 |= _10_SAU_RXD2_FILTER_ON;
  SIR11 = _0004_SAU_SIRMN_FECTMN | _0002_SAU_SIRMN_PECTMN | _0001_SAU_SIRMN_OVCTMN;
  SMR11 = _0020_SMR11_DEFAULT_VALUE | _0000_SAU_CLOCK_SELECT_CK00 | _0000_SAU_CLOCK_MODE_CKS | 
    _0100_SAU_TRIGGER_RXD | _0000_SAU_EDGE_FALL | _0002_SAU_MODE_UART | _0000_SAU_TRANSFER_END;
  SCR11 = _0004_SCR11_DEFAULT_VALUE | _4000_SAU_RECEPTION | _0000_SAU_TIMING_1 | _0400_SAU_INTSRE_ENABLE | 
    _0000_SAU_PARITY_NONE | _0080_SAU_LSB | _0010_SAU_STOP_1 | _0003_SAU_LENGTH_8;
  SDR11 = _9A00_SAU1_CH1_BAUDRATE_DIVISOR;
  SO1 |= _0001_SAUm_CH0_DATA_OUTPUT_1;
  SOL1 &= (uint16_t)~_0001_SAUm_CHANNEL0_INVERTED;
  SOE1 |= _0001_SAUm_CH0_OUTPUT_ENABLE;
  
  
  SO1 |= _0001_SAUm_CH0_DATA_OUTPUT_1;
  SOE1 |= _0001_SAUm_CH0_OUTPUT_ENABLE;
  SS1 |= _0002_SAUm_CH1_START_TRG_ON | _0001_SAUm_CH0_START_TRG_ON;
  STIF2 = 0U;     /* clear INTST2 interrupt flag */
  SRIF2 = 0U;     /* clear INTSR2 interrupt flag */
  SREIF2 = 0U;    /* clear INTSRE2 interrupt flag */
  STMK2 = 0U;     /* enable INTST2 interrupt */
  SRMK2 = 0U;     /* enable INTSR2 interrupt */
  SREMK2 = 0U;    /* enable INTSRE2 interrupt */  
#endif
}

__near_func void Self_Programming_main(void)
{
    unsigned char FSL_Status_byte=0;
    
    /* Disable Interrupt Routines */
    __disable_interrupt();
    
    /* Initialize UART Channel for Communication */
    Self_Programming_UART_Init();
    #if UART_TYPE == RJ_45
    SendRJ45(0x06);
    #else
    SendOptical(0x06);
    #endif
    /* Intialize Self Programming */
    FSL_Status_byte = Self_Programming_Init();
    
    if(FSL_Status_byte != 0)
    {
        return ;
    }
    
    while(1)
    {
        WDTE = 0xACU;
        backlight_operation();
        /* Check for Data Reception */
        Self_Programming_UART_Data_Recieve();
        
        if(FSL_Analyse_Data_Pkt_f==1)
        {
            Self_Programming_Protocol_Analyse_Pkt();
        }
    }
}

__near_func unsigned char Self_Programming_Init(void)
{
    unsigned char Init_Status=1; 
    
    fsl_descriptor_t InitArg;
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

__near_func void Self_Programming_UART_Data_Recieve(void)
{
    unsigned long int count = 0;
#if UART_TYPE == RJ_45    
    while(SRIF1==0)
#else
    while(SRIF2==0)  
#endif
    {
        count++;
        if(count > 10000000)
        {
            FSL_Serial_rxcounter = 0;
            count = 0;
            //SendOptical(0x05);
            while(1);
        }
        else
        {
            WDTE = 0xACU;
        }
        if(count % 100000 == 0)
        {
            backlight_operation();
        }
    }
#if UART_TYPE == RJ_45
    SRIF1=0;
    SREIF1=0;
#else
    SRIF2=0;
    SREIF2=0;
#endif
    count = 0;
#if UART_TYPE == RJ_45
    FSL_data_array[FSL_Serial_rxcounter] =RXD1;
#else
    FSL_data_array[FSL_Serial_rxcounter] =RXD2;
#endif
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
        
        if((FSL_RX_Data_Pkt_length==6) || (FSL_RX_Data_Pkt_length==11)||(FSL_RX_Data_Pkt_length==261)) // || (FSL_RX_Data_Pkt_length==133) || (FSL_RX_Data_Pkt_length==16))
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

__near_func unsigned char Verify_ChkSum(unsigned char *chkdata,unsigned int nob)
{
    unsigned int i,chksum=0;
    
    for(i=0;i<nob;i++)
    {
        chksum += *chkdata++;
    }
    
    Write_addr_temp = nob;
    return(chksum);
}

__near_func void Self_Programming_Protocol_Analyse_Pkt(void)
{
    us8 Command_key=0;
    
    FSL_Analyse_Data_Pkt_f = 0 ;
    
    if(FSL_data_array[3] == 0x06)
    {
        NOP();
    }
    
    if(Verify_ChkSum(&FSL_data_array[0],FSL_RX_Data_Pkt_length-1) != FSL_data_array[FSL_RX_Data_Pkt_length-1])
    {
        FSL_RX_Data_Pkt_length = 0 ;
        return;
    }
    Command_key = FSL_data_array[3];
    
    Self_Programming_Protocol_Prepare_Pkt(Command_key);
}

__near_func void Self_Programming_Protocol_Prepare_Pkt(unsigned char data_ID)
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
        //    lcd_clear_var();
        //    lcd_map[6] |= LCD_7b;
        //    lcd_map[7] |= LCD_7O;
        //    lcd_map[8] |= LCD_7O;
        //    lcd_map[9] |= LCD_7t;
        //    lcd_map[5] |= 0x7A; // reverse_8bits(LCD_7b);
        //    lcd_map[11] |= lcd_7digit[WriteBlock1/0x10];
        //    lcd_map[12] |= lcd_7digit[WriteBlock1%0x10];
        //    lcd_write();
        
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
        
    case 0x06:
        if(WriteBlock1 < FLASH_SIZE)
        {
            Write_addr = (us32)WriteBlock1*0x0400;
            Satus_check = Self_Programming_Write(Write_addr);
        }
        FSL_delay(100);
        FSL_data_array[3]=data_ID ;
        FSL_data_array[4]=Satus_check;
        if(Satus_check	== 0x05)
        {
            NOP();
        }
        FSL_data_array[5]=0x0a;
        FSL_TX_Data_Pkt_length=6;
        break;
        
    case 0x07: 
        FSL_Reset_f = 1; //FSL_ForceReset();
        FSL_data_array[3]=data_ID ;
        FSL_data_array[4]=0x00;
        FSL_data_array[5]=0x0b;
        FSL_TX_Data_Pkt_length=6;
        break;
        
    case 0x08: 
        Satus_check = FSL_InvertBootFlag(); 
        FSL_delay(200);
        FSL_Reset_f=1;
        FSL_data_array[3]=data_ID ;
        FSL_data_array[4]=Satus_check;
        FSL_data_array[5]=0x0c;
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
    default:  
        NOP();
        break;
    }
    
    FSL_Serial_counter=0;
    Self_Programming_UART_Data_Transmit();
    
    if(FSL_Reset_f==1)
    {
        FSL_Reset_f = 0;
        FSL_delay(200);
        FSL_ForceReset();
    }
}

__near_func void Self_Programming_UART_Data_Transmit(void)
{	
    unsigned long int count = 0;
    while(FSL_Serial_counter<FSL_TX_Data_Pkt_length)
    {
#if UART_TYPE == RJ_45
      TXD1 = FSL_data_array[FSL_Serial_counter];
#else
      TXD2 = FSL_data_array[FSL_Serial_counter];
#endif
#if UART_TYPE == RJ_45
      while(STIF1 == 0) 
#else 
      while(STIF2 == 0)
#endif
        {
            WDTE = 0xACU;
            count++;
            if(count > 1000000)
            {
                FSL_Serial_rxcounter = 0;
                count = 0;
                while(1);
            }
        }
#if UART_TYPE == RJ_45
        STIF1=0;
#else
        STIF2=0;
#endif
        FSL_Serial_counter++;
    }
    
    FSL_Serial_counter=0; 
}


__near_func unsigned char Self_Programming_Write(unsigned long int Write_data)
{
    unsigned char status=0, count;
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
                NOP();
            }
        }
    }
    else
    {
        NOP();
    }
    
    return status;
}


/**** 1 for 1 ms ****/
__near_func void FSL_delay(unsigned int count)
{
    unsigned int i,k;
    
    for(i=0; i<count; i++)
    {
        for(k=0;k<510;k++)
        {
            NOP();
        }
    }
}


