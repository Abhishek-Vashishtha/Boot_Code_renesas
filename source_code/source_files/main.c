/***********************************************************************************************************************
* File Name       : main.c  
* Tool-Chain      : IAR Systems
* Creation Date   : 03-01-2020
* Company         : Genus Power Infrastructures Limited, Jaipur
* Author          : dheeraj.singhal
***********************************************************************************************************************/
/************************************ Includes **************************************/
#include "main.h"
#include "system.h"
#include "clock.h"
#include "variables.h"

/************************************ Local Variables *****************************************/
/************************************ Extern Variables *****************************************/
/************************************ Local Functions *******************************/
/************************************ Extern Functions ******************************/

void main()
{
  /* Reset through ram parity error are disabled */
  __disable_interrupt();
  RPERDIS = 1;
  pcb_init_universal();
  clock_init();
  wdt_init();
  wdt_restart();
  R_LVD_Create();
  lvd_start();
  
  /* waiting for 0.5 Sec */
  for(us16 index = 0; index <5; index++)
  {
    wdt_restart();
    delay_ms(100);
  }
  clear_ram();

  lcd_init();
  lcd_start();
  
  lcd_clear_var();
  lcd_map[3] |= 0x0009;   // Capital B
  lcd_map[4] |= 0x0207;
  lcd_map[5] |= LCD_7B;
  lcd_map[6] |= LCD_7O;
  lcd_map[7] |= LCD_7O;
  lcd_map[8] |= LCD_7T;
//  lcd_map[1] |= 0x7A; // reverse_8bits(LCD_7b);
  lcd_write();
  
  battery_mode_f = read_operating_mode();
  
  backlight_operation();
  
  wdt_restart();
  Self_Programming_main();
  
  while(1)
  {
    NOP();
  }
}
