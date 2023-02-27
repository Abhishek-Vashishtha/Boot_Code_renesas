/***********************************************************************************************************************
* File Name       : lcd.h
* Current Version : rev_01  
* Tool-Chain      : IAR Systems
* Description     : this is a header file of lcd.c
* Creation Date   : 26-12-2019
* Company         : Genus Power Infrastructures Limited, Jaipur
* Author          : dheeraj.singhal
***********************************************************************************************************************/
#pragma once
/************************************ Includes **************************************/
#include "system.h"
#include "r_cg_lcd.h"
#include "pcb.h"
/************************************ Macro *****************************************/
/* Configurations */

/* LCD related */
#define U8LCDMEM                                        ((us8*)&SEG4)
#define LCD_7A                                          (0xFCu)
#define LCD_7a                                          (0xFCu)
//#define LCD_7B                                          (0x5Eu)
#define LCD_7B                                          (0xe030)
#define LCD_7C                                          (0xCAu)
#define LCD_7c                                          (0x1Au)
#define LCD_7D                                          (0x3Eu)
#define LCD_7d                                          (0x3Eu)
#define LCD_7E                                          (0xDAu)
#define LCD_7e                                          (0xFAu)
#define LCD_7F                                          (0xD8u)
#define LCD_7f                                          (0xD8u)
#define LCD_7G                                          (0xCEu)
#define LCD_7g                                          (0xCEu)
#define LCD_7H                                          (0x7Cu)
#define LCD_7h                                          (0x5Cu)
#define LCD_7I                                          (0x24u)
#define LCD_7i                                          (0x24u)
#define LCD_7J                                          (0x2Eu)
#define LCD_7j                                          (0x2Eu)
#define LCD_7K                                          (0x7Cu)
#define LCD_7k                                          (0x7Cu)
#define LCD_7L                                          (0x4Au)
#define LCD_7l                                          (0x4Au)
#define LCD_7M                                          (0x8Cu)
#define LCD_7m                                          (0x8Cu)
#define LCD_7N                                          (0x1Cu)
#define LCD_7n                                          (0x1Cu)
#define LCD_7O                                          (0xa0f0)
#define LCD_7o                                          (0x1Eu)
#define LCD_7P                                          (0xF8u)
#define LCD_7p                                          (0xF8u)
#define LCD_7Q                                          (0xF4u)
#define LCD_7q                                          (0xF4u)
#define LCD_7R                                          (0x18u)
#define LCD_7r                                          (0x18u)
#define LCD_7S                                          (0xD6u)
#define LCD_7s                                          (0xD6u)
#define LCD_7T                                          (0xe010)
#define LCD_7t                                          (0x5Au)
#define LCD_7U                                          (0x6Eu)
#define LCD_7u                                          (0x6Eu)
#define LCD_7V                                          (0x0Eu)
#define LCD_7v                                          (0x0Eu)
#define LCD_7W                                          (0x62u)
#define LCD_7w                                          (0x62u)
#define LCD_7X                                          (0x7Cu)
#define LCD_7x                                          (0x7Cu)
#define LCD_7Y                                          (0x76u)
#define LCD_7y                                          (0x76u)
#define LCD_7Z                                          (0xBAu)
#define LCD_7z                                          (0xBAu)
#define LCD_70                                          (0xEEu)
#define LCD_71                                          (0x24u)
#define LCD_72                                          (0xBAu)
#define LCD_73                                          (0xB6u)
#define LCD_74                                          (0x74u)
#define LCD_75                                          (0xD6u)
#define LCD_76                                          (0xDEu)
#define LCD_77                                          (0xA4u)
#define LCD_78                                          (0xFEu)
#define LCD_79                                          (0xF6u)
#define LCD_7ALL                                        (0xFEu)
#define LCD_7dash                                       (0x10u)

#define LCD_ALL_SEG                                     {lcd_map[0] |= 0xFF;lcd_map[1] |= 0xFF;lcd_map[2] |= 0xFF;lcd_map[3] |= 0xFF;lcd_map[4] |= 0xFF;lcd_map[5] |= 0xFF;lcd_map[6] |= 0xFF;lcd_map[7] |= 0xFF;lcd_map[8] |= 0xFF;lcd_map[9] |= 0xFF;lcd_map[10] |= 0xFF;lcd_map[11] |= 0xFF;lcd_map[12] |= 0xFF;lcd_map[13] |= 0xFF;lcd_map[14] |= 0xFF;lcd_map[15] |= 0xFF;}

/* segments */    
#define LCD_SEG_ONE                                     {lcd_map[0] |= bit7;}
#define LCD_SEG_B                                       {lcd_map[1] |= bit7;} 
#define LCD_SEG_C                                       {lcd_map[2] |= bit6;}                
#define LCD_SEG_1                                       {lcd_map[2] |= bit5;}                
#define LCD_SEG_2                                       {lcd_map[2] |= bit4;}                
#define LCD_SEG_3                                       {lcd_map[2] |= bit3;}                
#define LCD_SEG_X                                       {lcd_map[2] |= bit2;}                
#define LCD_SEG_TC                                      {lcd_map[2] |= bit1;}
#define LCD_SEG_C1                                      {lcd_map[2] |= bit0;}
#define LCD_SEG_L                                       {lcd_map[3] |= bit7;} 
#define LCD_SEG_T                                       {lcd_map[4] |= bit7;} 
#define LCD_SEG_T_Clear                                 {lcd_map[4] &= ~bit7;}
#define LCD_SEG_EL                                      {lcd_map[5] |= bit7;}                        
#define LCD_SEG_MAG                                     {lcd_map[5] |= bit6;}
#define LCD_SEG_RLY1                                    {lcd_map[5] |= bit5;}                        
#define LCD_SEG_RLY2                                    {lcd_map[5] |= bit4;}                
#define LCD_SEG_BAT                                     {lcd_map[5] |= bit3;} 
#define LCD_SEG_P1                                      {lcd_map[7] |= bit0;}                        
#define LCD_SEG_P2                                      {lcd_map[8] |= bit0;} 
#define LCD_SEG_COL                                     {lcd_map[9] |= bit0;}   
#define LCD_SEG_P3                                      {lcd_map[10] |= bit0;}                        
#define LCD_SEG_P4                                      {lcd_map[11] |= bit0;}                        
#define LCD_SEG_P5                                      {lcd_map[12] |= bit0;}                        
#define LCD_SEG_P6                                      {lcd_map[13] |= bit0;} 
#define LCD_SEG_h                                       {lcd_map[14] |= bit7;}                
#define LCD_SEG_r                                       {lcd_map[14] |= bit6;}             
#define LCD_SEG_A                                       {lcd_map[14] |= bit5;}                      
#define LCD_SEG_V1                                      {lcd_map[14] |= bit4;} 
#define LCD_SEG_V                                       {lcd_map[14] |= bit3;} 
#define LCD_SEG_K                                       {lcd_map[14] |= bit2;}
#define LCD_SEG_COM                                     {lcd_map[15] |= bit7;} 
#define LCD_SEG_I                                       {lcd_map[15] |= bit6;} 
#define LCD_SEG_E                                       {lcd_map[15] |= bit5;}                        
#define LCD_SEG_EP                                      {lcd_map[15] |= bit4;}                
#define LCD_SEG_RUPEE                                   {lcd_map[15] |= bit3;}
#define LCD_SEG_FWD                                     {lcd_map[15] |= bit2;}                        
#define LCD_SEG_REV                                     {lcd_map[15] |= bit1;}               
#define LCD_SEG_MD                                      {lcd_map[15] |= bit0;}                


/************************************ Global Variables *****************************************/


extern const us8 lcd_7digit[16];

extern us16 lcd_map[13];

/************************************ Extern Functions ******************************/
extern void lcd_init();
extern void lcd_start();
extern void lcd_stop();
extern void lcd_write();
extern void lcd_clear_var();

