/***********************************************************************************************************************
* File Name       : pcb.h
* Current Version : rev_01  
* Tool-Chain      : IAR Systems
* Description     : this is a header file of
* Creation Date   : 30-12-2019
* Company         : Genus Power Infrastructures Limited, Jaipur
* Author          : dheeraj.singhal
***********************************************************************************************************************/
#pragma once
/************************************ Includes **************************************/
#include "variables.h"
#include "r_cg_port.h"

/************************************ Macro *****************************************/
/* Pin Configurations */
#define TXD2_PIN                                                (bit6)                                  /* P56 */
#define TXD2_PIN_HIGH                                           (bitSet(P5,TXD2_PIN))
#define TXD2_PIN_LOW                                            (bitClear(P5,TXD2_PIN))

#define RXD2_PIN                                                (bit5)                                  /* P55 */
#define RXD2_PIN_HIGH                                           (bitSet(P5,RXD2_PIN))
#define RXD2_PIN_LOW                                            (bitClear(P5,RXD2_PIN))

#define TXD1_MISO                                               (bit4)                                  /* P04 */
#define TXD1_MISO_HIGH                                          (bitSet(P0,TXD1_MISO))
#define TXD1_MISO_LOW                                           (bitClear(P0,TXD1_MISO))

#define RXD1_MOSI                                               (bit3)                                  /* P03 */
#define RXD1_MOSI_HIGH                                          (bitSet(P0,RXD1_MOSI))
#define RXD1_MOSI_LOW                                           (bitClear(P0,RXD1_MOSI))

#define LED_BACKLIGHT                                           (bit3)                                  /* P53 */  
#define LED_BACKLIGHT_HIGH                                      (bitSet(P5,LED_BACKLIGHT))
#define LED_BACKLIGHT_LOW                                       (bitClear(P5,LED_BACKLIGHT))
#define LED_BACKLIGHT_TOGGLE                                    (bitToggle(P5,LED_BACKLIGHT))


#define SD                                                      (bit1)                                  /* P51 */
#define SD_HIGH                                                 (bitSet(P5,SD))  
#define SD_LOW                                                  (bitClear(P5,SD)) 

#define LED_RCAL                                                (bit0)                                  /* P50 */  
#define LED_RCAL_HIGH                                           (bitSet(P5,LED_RCAL))
#define LED_RCAL_LOW                                            (bitClear(P5,LED_RCAL))
#define LED_RCAL_TOGGLE                                         (bitToggle(P5,LED_RCAL))

#define LED_ECAL                                                (bit3)                                  /* P43 */  
#define LED_ECAL_HIGH                                           (bitSet(P4,LED_ECAL))
#define LED_ECAL_LOW                                            (bitClear(P4,LED_ECAL))
#define LED_ECAL_TOGGLE                                         (bitToggle(P4,LED_ECAL))

/* BAT_CTRL */
#define BAT_CTRL                                                (bit4)  /* P74 */
#define BAT_CTRL_DISABLE                                        (bitSet(P7, BAT_CTRL))
#define BAT_CTRL_ENABLE                                         (bitClear(P7, BAT_CTRL))

#define BOOT_MODE                                               (1u)
#define BOOT_MODE_SLEEP                                         (2u)

/************************************ Extern Variables *****************************************/


/************************************ Extern Functions ******************************/
extern void pcb_io_redirection();
extern void pcb_init_universal();


