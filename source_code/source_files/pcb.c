/***********************************************************************************************************************
* File Name       : pcb.c
* Current Version : rev_01  
* Tool-Chain      : IAR Systems
* Description     : Functionality related to Meter operation
* Creation Date   : 30-12-2019
* Company         : Genus Power Infrastructures Limited, Jaipur
* Author          : dheeraj.singhal
* Version History : rev_01 : initial release
***********************************************************************************************************************/
/************************************ Includes **************************************/
#include "pcb.h"
#include "system.h"
#include "lcd.h"
/************************************ Local Variables *****************************************/
/************************************ Extern Variables *****************************************/

/************************************ Local Functions *******************************/
/************************************ Extern Functions ******************************/
void pcb_io_redirection();
void pcb_init_universal();


void pcb_io_redirection()
{
  /* IO Redirection register */
  PIOR0 = bit3; /* As RTCOUT has been used on alternative pin */
}
void pcb_init_universal()
{
  delay_ms(1); /* 300 us time is needed for EXLVD pin stabilisation */
  
   /* BAT_CTRL P74*/
    PFSEG2 &= ~bit4;
    PM7 &= ~BAT_CTRL;
    BAT_CTRL_ENABLE;      
    
    /* TXD2 P56*/
    /* Set TxD1 UART Mode P04*/
//    PM0 &= ~TXD1_MISO;
//    POM0 &= ~TXD1_MISO;
//    TXD1_MISO_HIGH;
    
    /* TXD2 P56*/
    PFSEG4 &= ~bit6;
    PM5 &= ~TXD2_PIN;
    POM5 &= ~TXD2_PIN;
    TXD2_PIN_HIGH;
    
    /* RXD2 P55*/
    PFSEG4 &= ~bit5;
    PM5 |= RXD2_PIN;
    
    /* SD P51 */
    bitClear(PFSEG4,bit1);
    bitClear(PM5,SD);
    bitClear(P5,SD); 
    
    
    /* RXD2 P55*/
//    PM0 |= RXD1_MOSI;
    
    /* E-CAL P43*/
    PM4 &= ~LED_ECAL;
    LED_ECAL_LOW; 
    
    
    /* BACKLIGHT_CTRL P53 */
    bitClear(PFSEG4,bit3);
    bitClear(PM5,LED_BACKLIGHT);
    LED_BACKLIGHT_LOW;     
    
    /* R-CAL P50 */
    bitClear(PFSEG4,bit0);
    bitClear(PM5,LED_RCAL);
    LED_RCAL_LOW;
}