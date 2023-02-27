/***********************************************************************************************************************
* File Name       : lcd.c
* Current Version : rev_01  
* Tool-Chain      : IAR Systems
* Description     : This file comprises all the routines and variables required for the operation of LCD Module
* Creation Date   : 26-12-2019
* Company         : Genus Power Infrastructures Limited, Jaipur
* Author          : dheeraj.singhal
* Version History : rev_01 - new source file created with routine to operate LCD module 
***********************************************************************************************************************/

/************************************ Includes **************************************/
#include "lcd.h"

/************************************ Local Variables *****************************************/
//const us8 lcd_alphabeat[58] = {LCD_7A,LCD_7B,LCD_7C,LCD_7D,LCD_7E,LCD_7F,LCD_7G,LCD_7H,LCD_7I,LCD_7J,LCD_7K,LCD_7L,LCD_7M,LCD_7N,LCD_7O,LCD_7P,LCD_7Q,LCD_7R,LCD_7S,LCD_7T,LCD_7U,LCD_7V,LCD_7W,LCD_7X,LCD_7Y,LCD_7Z,LCD_7dash,LCD_7dash,LCD_7dash,LCD_7dash,LCD_7dash,LCD_7dash,LCD_7a,LCD_7b,LCD_7c,LCD_7d,LCD_7e,LCD_7f,LCD_7g,LCD_7h,LCD_7i,LCD_7j,LCD_7k,LCD_7l,LCD_7m,LCD_7n,LCD_7o,LCD_7p,LCD_7q,LCD_7r,LCD_7s,LCD_7t,LCD_7u,LCD_7v,LCD_7w,LCD_7x,LCD_7y,LCD_7z};
//const us8 lcd_7digit[16] = {LCD_70,LCD_71,LCD_72,LCD_73,LCD_74,LCD_75,LCD_76,LCD_77,LCD_78,LCD_79,LCD_7A,LCD_7B,LCD_7C,LCD_7D,LCD_7E,LCD_7F};

/************************************ Global Variables *****************************************/

us16 lcd_map[13];

/************************************ Local Functions *******************************/

/************************************ Extern Functions ******************************/
void lcd_clear_var();
void lcd_disp_off();
void lcd_init();
void lcd_stop();
void lcd_start();
void lcd_write();


/***********************************************************************************************************************
* Function Name: lcd_init
* Description  : Initialises driver and displays welcome text
* Arguments    : nothing
* Return Value : nothing
***********************************************************************************************************************/
void lcd_init()
{
  R_LCD_Create();
  R_LCD_Voltage_On();
}
void lcd_start()
{
  R_LCD_Start();
  lcd_disp_off();
}
void lcd_stop()
{
  R_LCD_Voltage_Off();
  R_LCD_Stop();
}
void lcd_disp_off()
{
    for(us8 index = 0; index <=12; index++)
    {
	U8LCDMEM[2 * index] = 0;
	U8LCDMEM[2 * index + 1] = 0;
    }
}

void lcd_write()
{
    for(us8 index = 0; index <=12; index++)
    {
	U8LCDMEM[2 * index] = reverse_8bits(lowByte(lcd_map[12 - index]));
	U8LCDMEM[2 * index + 1] = reverse_8bits(highByte(lcd_map[12 - index]));
    }
}

void lcd_clear_var()
{
  us8 index = 0;
  while(index <= 12)
  {
    lcd_map[index]=0;
    index++;
  }
}
