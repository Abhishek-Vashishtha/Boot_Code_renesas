/***********************************************************************************************************************
* File Name       : system.c
* Current Version : rev_  
* Tool-Chain      : IAR Systems
* Description     : This file contains routines for system operation
* Creation Date   : 30-12-2019
* Company         : Genus Power Infrastructures Limited, Jaipur
* Author          : dheeraj.singhal
* Version History : 
***********************************************************************************************************************/
/************************************ Includes **************************************/
#include "system.h"

/************************************ Local Variables *****************************************/
/************************************ Extern Variables *****************************************/

flag_union flag1;

us8 last_interrupt;

/************************************ Local Functions *******************************/
void system_reset_source();

/************************************ Extern Functions ******************************/
void clear_ram();
void system_init();
void lvd_start();

us8 read_operating_mode();
void backlight_operation();
/* Set option bytes */
#pragma location = "OPTBYTE"
#if WDT_TIME == WDT_TIME_474MS
    __root const uint8_t opbyte0 = 0x7AU;
#elif WDT_TIME == WDT_TIME_949MS
    __root const uint8_t opbyte0 = 0x7CU;
#else                                                                           //WDT_TIME_3799MS
    __root const uint8_t opbyte0 = 0x7EU;
#endif

#pragma location = "OPTBYTE"
__root const uint8_t opbyte1 = 0x36U;
#pragma location = "OPTBYTE"
__root const uint8_t opbyte2 = 0xE0U;
#pragma location = "OPTBYTE"
__root const uint8_t opbyte3 = 0x84U;

/* Set security ID */
#pragma location = "SECUID"
__root const uint8_t secuid[10] = 
    {0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U};



/***********************************************************************************************************************
* Function Name: system_init
* Description  : initialises system settings which are not relevent to particular application
* Arguments    : nothing
* Return Value : nothing
***********************************************************************************************************************/
void system_init()
{
    //system_reset_source();
    
    /* Reset through ram parity error are disabled */
    RPERDIS = 1;
    
    pcb_io_redirection();

    /* Enabling CRC checking of flash */
    //CRC0CTL = 0x80U;
    
    /* Illegal writing access reset enabled */
    //IAWCTL = 0x80U;
    
    clock_init();
    //clock_change(CLOCK_1_5MHZ);
    
    R_LVD_Create();
    
    wdt_init();
}


void clear_ram()
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
            
us8 read_operating_mode()
{
  if(flag_lvd_exlvd_instant == 1)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

void lvd_start()
{
  //R_LVD_InterruptMode_Start();
  //R_LVD_Start_VDD();
  //R_LVD_Start_VRTC();
  R_LVD_Start_EXLVD();
}
void backlight_operation()
{
  if(battery_mode_f == 1 || flag_lvd_exlvd_instant == 1)
  {
    LED_BACKLIGHT_LOW;
  }
  else
  {
    LED_BACKLIGHT_HIGH;
  }
}
