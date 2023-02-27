/************************************************************************************
 *																					*
 *  File Name	: Eprom_i2c.c												
 *  Contents	: IIC Bus file														
 *  Copyright	: Genus Overseas Electronics Ltd.
 *  Version	: 1.0																
 *  note        :																	
 *  delay changed 																					*
 **************
TPAEC FILE FOR EPROM
*********************************************************************/
#include "r_cg_macrodriver.h"
#include "Eprom_i2c.h"
#include  "variable.h"
#include  "function.h"
#include  "tamper.h"


#define scle                (BIT0)
#define sdae                (BIT1)
#define EP_P_OUT		P6
#define EP_P_IN			P6
#define EP_P_DIR		PM6
#define ACK                     0
#define NOACK                   1
//#define WaitTime2us()		Wait2us()

void StartCondition_mem(void);
void StopCondition_mem(void);
void EPR_vReset_Mem(void);
unsigned char ByteWrite(unsigned char);
void ByteRead(unsigned char *, unsigned char);
void WaitTime2us(void);
//extern void delayw(void);

//#pragma data_alignment = 16
//extern unsigned char opr_data[16];
//extern flag_union  flag0;

//extern  struct bit  flag1;
//extern  flag_union  flag1;
//extern	IicPack IicData_r;
//extern  unsigned char opr_data[16];

//extern void _WaitTime1us(void);
//extern void WaitTime2us(void);
void EPR_vInit(void)
{
  // Select P3.0 & P3.1 as SCL & SDA pins
  //P3SEL =0x00;

  //Set SDA N SCL as output pins
  EP_P_DIR &= ~(sdae+scle);

  //Pull SDA n SCL pins high
  EP_P_OUT |= sdae+scle;
}
void __delay_cycles(u32 cyc)
{
	while(cyc--)
	{ __nop(); }
}
/************************************************************************************
 Name         : Eprom_Read
 Parameters   : structure IicPack pointer
 Returns      : Acknowledge
 Description  : Sequential Ramdom Read Cycle (I2C-BUS)
 Note         :
************************************************************************************/
//unsigned char Eprom_Read(IicPack *IicData, unsigned char mem_id)
//void Eprom_Read(unsigned char Block_Add, unsigned char Page_Add)
void Eprom_Read(unsigned int Add)
{
	unsigned char *data;
	unsigned char i,iic_DeviceAddress;
	unsigned char repeat_cnt;
	
 for(repeat_cnt=0; repeat_cnt<3; repeat_cnt++)             // retry to retrieve checksum for three times
 {
		 
	//if(MEM2_256_f==0)
	{
		iic_DeviceAddress=0xa0;
	}
	//else
	//{
	//	iic_DeviceAddress=0xa2;
	//}
	
	data = opr_data;
	
	/*  if(mem_id == 0x01)
	iic_DeviceAddress = 0xA0;
	if(mem_id == 0x02)
	iic_DeviceAddress = 0xA1;
	if(mem_id == 0x03)
	iic_DeviceAddress = 0xA2;
	*/
	
	EP_P_OUT |= sdae+scle;
	WaitTime2us();//8mhz
	
	if((EP_P_OUT & sdae) == 0)
		EPR_vReset_Mem();	// reset if sda=0
		
	// Ramdom Read Cycle / Sequential Ramdom Read Cycle
	//  iic_DeviceAddress &= 0xFE;	// WRITE Setting Device Address
	//iic_DeviceAddress += Block_Add;	// WRITE Setting Device Address  
	StartCondition_mem();	// Start Condition
	if(busbusy_f0 == 1)
		return;																			// exit if Bus is Busy
		
	while(1)
	{
		if((ByteWrite(iic_DeviceAddress)) == NOACK)   // WRITE Device Address
			break;			        // NoAck Detect
		if((ByteWrite((u8)(Add/0x0100))) == NOACK)	        // WRITE Memory Address
			break;			        // NoAck Detect
		if((ByteWrite((u8)(Add%0x0100))) == NOACK)	        // WRITE Memory Address
			break;			        // NoAck Detect
		
		iic_DeviceAddress |= 0x01;	        // READ Setting Device Address
		// iic_DeviceAddress += Block_Add;
		StartCondition_mem();		        // ReStart Condition
		if(busbusy_f0 == 1)		
			return;                                         // exit if Bus is Busy
		
		if((ByteWrite(iic_DeviceAddress)) == NOACK)   // WRITE Device Address
			break;			        // NoAck Detect
		
		for (i=1; i<16; i++)                             // specified bytes as loop   NOB
		{		
			//  ByteRead(IicData->iic_Data, ACK);	        // Read data (Ack output)
			ByteRead(data, ACK);                          // Read data (Ack output)
			data++;	                        //IicData->iic_Data++;							
		}
		
		//    ByteRead(IicData->iic_Data, NOACK);	        // Read data (NoAck output)
		ByteRead(data, NOACK);		        // Read data (NoAck output)
		break;
	} // while
	
	StopCondition_mem();		        // Stop Condition

	if((opr_data[15] == chksumsr(opr_data,15))&&(acknotr_f0==0))
	{
		EPROM_bChksum_ok =1;
		repeat_cnt = 3 ;
	}
	else
	{
		EPROM_bChksum_ok =0;
		fill_oprzero();
	}
  }
	//MEM2_256_f=0;
}


/************************************************************************************
 Name           : Eprom_Write
 Parameters	: structure IicPack pointer
 Returns	: Acknowledge
 Description	: Byte Write or Page Write Cycle (I2C-BUS)
Note	:
************************************************************************************/
//unsigned char Eprom_Write(IicPack *IicData, unsigned char mem_id)
//void Eprom_Write(unsigned char Block_Add, unsigned char Page_Add)
void Eprom_Write(unsigned int Add)
{
  
  unsigned char i,iic_DeviceAddress;
  unsigned char *data;
  data = opr_data;
	
  	//if(MEM2_256_f==0)
	iic_DeviceAddress=0xa0;
	//else
	//iic_DeviceAddress=0xa2;
  
		 
  
  //  __disable_interrupt();


//  if(chk_sum8b_f==1)
//     opr_data[7] = chksumsr(opr_data,7);
//  else
    opr_data[15] = chksumsr(opr_data,15);
  
  EP_P_OUT |= sdae+scle;
//  P1OUT |= scle; 
 __nop();//asm("nop");
//   _WaitTime2us();//8mhz	// wait
  if((EP_P_OUT & sdae) == 0)
    EPR_vReset_Mem();	// reset if sda=0
// Ramdom Read Cycle / Sequential Ramdom Read Cycle
//  iic_DeviceAddress &= 0xFE;    // WRITE Setting Device Address
  //iic_DeviceAddress += Block_Add;    // WRITE Setting Device Address  
  StartCondition_mem();	// Start Condition
  if(busbusy_f0 == 1)
    return;	                // exit if Bus is Busy

  while(1)
    {
      if((ByteWrite(iic_DeviceAddress)) == NOACK)     // Write Device Address
        break;			          // NoAck Detect
      if((ByteWrite((u8)(Add/0x0100))) == NOACK)	          // Write Memory Addreess
        break;			          // NoAck Detect
      if((ByteWrite((u8)(Add%0x0100))) == NOACK)	          // Write Memory Addreess
        break;			          // NoAck Detect
      for(i=0; i<16; i++)                                // specified bytes as loop  NOB
        {			
          if((ByteWrite(*data)) == NOACK)	          // Write Data
            break;		          // NoAck Detect
          data++;	
//          if(i==7 && chk_sum8b_f==1)// && even_f == 0)
//            break;  
          
        }
      break;
    }   // while

  StopCondition_mem();		          // Stop Condition
//  chk_sum8b_f=0;
// delay_5ms();
//EP_P_OUT |= scle;
//EP_P_OUT &=~(scle);
 __delay_cycles(800);//approx 5ms = 40>3mhz
//EP_P_OUT |= scle;
  
	MEM2_256_f=0;
   // __enable_interrupt();

  return;
}

/************************************************************************************
 Name	: ByteWrite
 Parameters	: Write data
 Returns	: Acknowledge
 Description	: byte data Output (I2C-BUS)
 Note	: *1 adjust a wait time
************************************************************************************/
unsigned char ByteWrite(unsigned char iic_writeData)
{
	unsigned char maskData=0x80;	// MSB first
	unsigned char ret=ACK,i;      // Ack/NoAck
	
	EP_P_OUT &=~(scle);//0xfe;	//0x7f	
	__nop();//asm("nop");
	//  WaitTime2us();//8mhz               // wait *2
	//  _WaitTime1us();               // wait *2
	
  while(maskData  )                 // 8times as loop
    {	

      //  iic_sda = 0;	    // initialize port-latch Set as o/p
      if(iic_writeData & maskData)  // "H" output ?
        EP_P_OUT |= sdae;	    // Yes SDA="H"
      else
        EP_P_OUT &= ~(sdae);//0xfd;	    // No  SDA="L"//0xbf


//    _Wait_tSU_DAT;	    // wait
      //WaitTime2us();	    // wait *2
      __nop();//asm("nop");
//      WaitTime2us();//8mhz	    // wait *2
      EP_P_OUT |= scle;	    // SCL="H"
      //WaitTime2us();	    // wait *2
      __nop();//asm("nop");
      __nop();
      __nop();
//      _WaitTime2us();//8mhz	    // wait *2
      EP_P_OUT &=~(scle);// 0xfe;	    // SCL="L"
      maskData >>= 1;	    // change mask data
////      __nop();//asm("nop");
////      __nop();//asm("nop");
//      WaitTime2us();//8mhz	    // wait *2
//      WaitTime2us();//8mhz	    // wait *2

    } // End while

// Get ACK from slave at 9th pulse
  EP_P_OUT |= sdae;
  EP_P_DIR |= (sdae);//0xfd;	    // SDA input//0xbf
  //WaitTime2us();	    // wait *2
  //asm("nop");
  //asm("nop");
  //WaitTime2us();	    // wait *2
//  P1OUT |= scle;
  for(i=0;i<255;i++)
    {
      //_WaitTime2us();	    // wait *2
    //  asm("nop");
      //asm("nop");
      //_WaitTime2us();	    // wait *2
      EP_P_OUT |= scle;	    // SCL="H"
      if((EP_P_IN & sdae)==0) {
          ret = ACK;
          break;
        }				
      ret = NOACK;	    // NoAck Detect
//      EP_P_OUT &=~(scle);
    }
//  _WaitTime2us();//8mhz	    // wait *2
//  __nop();//asm("nop");
//  __nop();//asm("nop");
//  _WaitTime2us();//8mhz	    // wait *2
  EP_P_OUT &= ~(scle);//0xfe;		    // SCL="L"
  //_WaitTime2us();	    // wait *2
  __nop();//asm("nop");
  __nop();
//  _WaitTime2us();//8mhz	    // wait *2
  EP_P_DIR &= ~sdae;	    // SDA output
  
  if(ret==NOACK)
  {
	  if(acknotr_f0==0)
	  {
//		  MemFail_tprstr_f=1;
//		  TP.b.DiagnosticsEvent=1;
//		  Diagnostics_count++;
//		  check_tpr();
	  }
	  acknotr_f0=1;
//	  a8Daily_alert_status[11]|=DES_mem_fail;
  }
  
  return(ret);

}


/************************************************************************************
 Name	: ByteRead
 Parameters	: Read data strage location pointer, Select Ack/NoAck
 Returns	: None
 Description	: byte data input with Ack output (I2C-BUS)
 Note	: *1 adjust a wait time
************************************************************************************/
void ByteRead(unsigned char *iic_readData, unsigned char ackData)
{
	unsigned char maskData=0x80;  	// MSB first
	unsigned char readData;
	
	EP_P_DIR |= (sdae);//0xfd;              	// initialize port-latch//0xbf
	WaitTime2us();//8mhz
	*iic_readData = 0;	

	while(maskData)             // 8times as loop
	{	
		readData = *iic_readData | maskData;	
		EP_P_OUT |= scle;		// SCL="H"
//		WaitTime2us();//8mhz
		__nop();
		if((EP_P_IN & sdae) == sdae)
			*iic_readData = readData;	
//		WaitTime2us();//8mhz
		__nop();
		EP_P_OUT &=~(scle);// 0xfe;	//0x7f
		maskData >>= 1;	                // Change mask data
//		WaitTime2us();//8mhz
		//_WaitTime2us();
	}
	
	EP_P_DIR &= ~sdae;		// SDA output
	__nop();
	EP_P_OUT |= sdae;

	if(!ackData)			// Ack output ?
		EP_P_OUT &=~(sdae);// 0xfd;		// Yes SDA="L"//0xbf
	else			// NoAck output
		EP_P_OUT |= sdae;		// No  SDA="H"

//	WaitTime2us();//8mhz
	__nop();
	EP_P_OUT |= scle;			// SCL="H"
	
//	WaitTime2us();//8mhz
	__nop();
	__nop();
	__nop();
	EP_P_OUT &=~(scle);// 0xfe;			// SCL="L"//0x7f
//	WaitTime2us();//8mhz
	__nop();
	__nop();
	EP_P_OUT |= sdae;

}

/************************************************************************************
 Name	: StartCondition
 Parameters	: None
 Returns	: None
 Description	: Output Start Condition (I2C-BUS)
 Note	: *1 adjust a wait time
************************************************************************************/
void StartCondition_mem(void)
{ 
	busbusy_f0 = 0;
	acknotr_f0 = 0;
//	EP_P_DIR &= ~sdae;
	EP_P_OUT |= sdae;
//	WaitTime2us();
	EP_P_OUT |= scle;	
	WaitTime2us();//8mhz
//	__nop();//asm("nop");
	if ( (EP_P_OUT & sdae) == 0 || (EP_P_OUT & scle) ==0 )
		busbusy_f0 = 1;
	else
	{
		EP_P_OUT &=~(sdae);// 0xfd;//0xbf
//		WaitTime2us();//8mhz
		WaitTime2us();//8mhz
	} // else
}		

/************************************************************************************
 Name	: StopCondition
 Parameters	: None
 Returns	: None
 Description	: Output Stop Condition (I2C-BUS)
 Note	: *1 adjust a wait time
************************************************************************************/
void StopCondition_mem(void)
{
	EP_P_OUT &=~(sdae);// clear SDA
	WaitTime2us();//8mhz
	
	EP_P_OUT |= scle;			// Make SCLK high
	WaitTime2us();//8mhz	
	
	EP_P_OUT |= sdae;			// Make SDA high
}	

/************************************************************************************
 Name         : _WaitTime1us
 Parameters   : None
Returns       : None
 Description  : a 1us wait
************************************************************************************/
void WaitTime1us(void)
{
	__nop();//asm("nop");		
	__nop();
}


/************************************************************************************
Name          : WaitTime2us
 Parameters   : None
Returns       : None
 Description  : a 2us wait
************************************************************************************/
void WaitTime2us(void)
{
	__nop();//asm("nop");		/* +1cycle */
	__nop();//asm("nop");
//	__nop();//  asm("nop");//8mhz
//	__nop();//  asm("nop");//8mhz
}


/************************************************************************************
Name          : delayw
 Parameters   : None
Returns       : None
 Description  : a 10ms wait at 1MHz i=16650
************************************************************************************/
void delayw(void)
{
  unsigned int  i;
  
  for(i=0;i<8000;i++)
  {
    __nop();
  }
}

/************************************************************************************
 Name	: reset_mem
 Parameters	: None
 Returns	: None
 Description	: a 2us wait
************************************************************************************/
void EPR_vReset_Mem(void)
{

  unsigned char i,j;
//  EP_P_DIR &=~(scle) ;
  for(j=0;j<5;j++)
    {
      for(i=0;i<9;i++)
        {
          EP_P_OUT &=~(scle);// 0xfe;//0x7f
//          __nop();//asm("nop"); 
          WaitTime2us();//8mhz
          EP_P_OUT |= scle;
          __nop();//asm("nop");
//          WaitTime2us();//8mhz
          if((EP_P_OUT & sdae)==sdae)
            return;
          /*else
            
            asm("nop");//WaitTime2us();*/
        }

//      WaitTime2us();
//      WaitTime2us();
//      WaitTime2us();
      //asm("nop");
      //asm("nop");//WaitTime2us();
    //  asm("nop");//WaitTime2us();

    }

}


