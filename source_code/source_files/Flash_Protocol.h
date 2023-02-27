/***********************************************************************************************************************
* File Name       : Flash_Protocol.h
* Current Version : rev_01  
* Tool-Chain      : IAR Systems
* Description     : this is a header file of Flash_Protocol.c
* Creation Date   : 18-12-2020
* Company         : Genus Power Infrastructures Limited, Jaipur
* Author          : dheeraj.singhal
***********************************************************************************************************************/
#pragma once
/************************************ Includes **************************************/
#include "variables.h"
/************************************ Macro *****************************************/
/* Configurations */
/* Thresholds */
/************************************ Extern Variables *****************************************/
/************************************ Extern Functions ******************************/

extern us8 FSL_data_array[262];
extern us32 Write_addr;
extern us32 Write_addr_temp;
extern us16 increment;
extern us16 Reset_Add ;
extern us16 FSL_RX_Data_Pkt_length;
extern us16 FSL_Serial_rxcounter;
extern us16 WriteBlock1;
extern us8 FSL_Serial_counter;
extern us8 FSL_TX_Data_Pkt_length;
extern us8 Flash_write_flag;
extern us8 FSL_Analyse_Data_Pkt_f;
extern us8 flash_complete;
extern us8 FSL_Reset_f;


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
__near_func void Self_Programming_LED_Init(void);
__near_func void FSL_delay(unsigned int count);
__near_func unsigned char Self_Programming_Init(void);
__near_func unsigned char Self_Programming_Write(unsigned long int Write_data);
