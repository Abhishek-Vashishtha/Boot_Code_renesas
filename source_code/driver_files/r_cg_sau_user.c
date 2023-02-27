/***********************************************************************************************************************
* DISCLAIMER
* This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products.
* No other uses are authorized. This software is owned by Renesas Electronics Corporation and is protected under all
* applicable laws, including copyright laws. 
* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING THIS SOFTWARE, WHETHER EXPRESS, IMPLIED
* OR STATUTORY, INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NON-INFRINGEMENT.  ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED.TO THE MAXIMUM EXTENT PERMITTED NOT PROHIBITED BY
* LAW, NEITHER RENESAS ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES SHALL BE LIABLE FOR ANY DIRECT,
* INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON RELATED TO THIS SOFTWARE, EVEN IF RENESAS OR
* ITS AFFILIATES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* Renesas reserves the right, without notice, to make changes to this software and to discontinue the availability 
* of this software. By using this software, you agree to the additional terms and conditions found by accessing the 
* following link:
* http://www.renesas.com/disclaimer
*
* Copyright (C) 2015, 2019 Renesas Electronics Corporation. All rights reserved.
***********************************************************************************************************************/

/***********************************************************************************************************************
* File Name    : r_cg_sau_user.c
* Version      : Applilet4 for RL78/I1C V1.01.04.02 [20 Nov 2019]
* Device(s)    : R5F10NPJ
* Tool-Chain   : IAR Systems icc78k0r
* Description  : This file implements device driver for SAU module.
* Creation Date: 06/05/2020
***********************************************************************************************************************/

/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "r_cg_macrodriver.h"
#include "r_cg_sau.h"
/* Start user code for include. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
Pragma directive
***********************************************************************************************************************/
/* Start user code for pragma. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
extern uint8_t * gp_uart1_tx_address;         /* uart1 send buffer address */
extern uint16_t  g_uart1_tx_count;            /* uart1 send data number */
extern uint8_t * gp_uart1_rx_address;         /* uart1 receive buffer address */
extern uint16_t  g_uart1_rx_count;            /* uart1 receive data number */
extern uint16_t  g_uart1_rx_length;           /* uart1 receive data length */
extern uint8_t * gp_uart2_tx_address;         /* uart2 send buffer address */
extern uint16_t  g_uart2_tx_count;            /* uart2 send data number */
extern uint8_t * gp_uart2_rx_address;         /* uart2 receive buffer address */
extern uint16_t  g_uart2_rx_count;            /* uart2 receive data number */
extern uint16_t  g_uart2_rx_length;           /* uart2 receive data length */
/* Start user code for global. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
* Function Name: r_uart1_interrupt_receive
* Description  : RJ45 Port
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
#pragma vector = INTSR1_vect
__interrupt static void r_uart1_interrupt_receive(void)
{
  volatile uint8_t rx_data;
  last_interrupt = 25;
  rx_data = ReceiveRJ45();
  //rcv_rj45(rx_data);
}
/***********************************************************************************************************************
* Function Name: r_uart1_interrupt_error
* Description  : None
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
#pragma vector = INTSRE1_vect
__interrupt static void r_uart1_interrupt_error(void)
{
    volatile uint8_t err_type;
    last_interrupt = 26;
    *gp_uart1_rx_address = RXD1;
    err_type = (uint8_t)(SSR03 & 0x0007U);
    SIR03 = (uint16_t)err_type;
    
    if((err_type & BIT2)!=0)
    {
        ST0 |= _0008_SAUm_CH3_STOP_TRG_ON | _0004_SAUm_CH2_STOP_TRG_ON;
        NOP();
        NOP();
        SS0 |= _0008_SAUm_CH3_START_TRG_ON | _0004_SAUm_CH2_START_TRG_ON;
    }
}
/***********************************************************************************************************************
* Function Name: r_uart1_interrupt_send
* Description  : None
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
#pragma vector = INTST1_vect
__interrupt static void r_uart1_interrupt_send(void)
{
  last_interrupt = 27;
  //transmit_rj45();
}
/***********************************************************************************************************************
* Function Name: r_uart1_callback_receiveend
* Description  : This function is a callback function when UART1 finishes reception.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
//static void r_uart1_callback_receiveend(void)
//{
//  /* Start user code. Do not edit comment generated here */
//  /* End user code. Do not edit comment generated here */
//}
/***********************************************************************************************************************
* Function Name: r_uart1_callback_softwareoverrun
* Description  : This function is a callback function when UART1 receives an overflow data.
* Arguments    : rx_data -
*                    receive data
* Return Value : None
***********************************************************************************************************************/
//static void r_uart1_callback_softwareoverrun(uint16_t rx_data)
//{
//  /* Start user code. Do not edit comment generated here */
//  /* End user code. Do not edit comment generated here */
//}
/***********************************************************************************************************************
* Function Name: r_uart1_callback_sendend
* Description  : This function is a callback function when UART1 finishes transmission.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
//static void r_uart1_callback_sendend(void)
//{
//  /* Start user code. Do not edit comment generated here */
//  /* End user code. Do not edit comment generated here */
//}
/***********************************************************************************************************************
* Function Name: r_uart1_callback_error
* Description  : This function is a callback function when UART1 reception error occurs.
* Arguments    : err_type -
*                    error type value
* Return Value : None
***********************************************************************************************************************/
//static void r_uart1_callback_error(uint8_t err_type)
//{
//  /* Start user code. Do not edit comment generated here */
//  /* End user code. Do not edit comment generated here */
//}
/***********************************************************************************************************************
* Function Name: r_uart2_interrupt_receive
* Description  : Optical port
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
#pragma vector = INTSR2_vect
__interrupt static void r_uart2_interrupt_receive(void)
{
  last_interrupt = 28;
//  rcv_optical(ReceiveOptical());  
}
/***********************************************************************************************************************
* Function Name: r_uart2_interrupt_error
* Description  : None
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
#pragma vector = INTSRE2_vect
__interrupt static void r_uart2_interrupt_error(void)
{
    volatile uint8_t err_type;
    last_interrupt = 29;
    *gp_uart2_rx_address = RXD2;
    err_type = (uint8_t)(SSR11 & 0x0007U);
    SIR11 = (uint16_t)err_type;

    if((err_type & BIT2)!=0)
    {
        ST1 |= _0002_SAUm_CH1_STOP_TRG_ON | _0001_SAUm_CH0_STOP_TRG_ON;
        NOP();
        NOP();
        SS1 |= _0002_SAUm_CH1_START_TRG_ON | _0001_SAUm_CH0_START_TRG_ON;
    }
}
/***********************************************************************************************************************
* Function Name: r_uart2_interrupt_send
* Description  : None
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
#pragma vector = INTST2_vect
__interrupt static void r_uart2_interrupt_send(void)
{
  last_interrupt = 30;
//  transmit_optical();
}
/***********************************************************************************************************************
* Function Name: r_uart2_callback_receiveend
* Description  : This function is a callback function when UART2 finishes reception.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
//static void r_uart2_callback_receiveend(void)
//{
//  /* Start user code. Do not edit comment generated here */
//  /* End user code. Do not edit comment generated here */
//}
/***********************************************************************************************************************
* Function Name: r_uart2_callback_softwareoverrun
* Description  : This function is a callback function when UART2 receives an overflow data.
* Arguments    : rx_data -
*                    receive data
* Return Value : None
***********************************************************************************************************************/
//static void r_uart2_callback_softwareoverrun(uint16_t rx_data)
//{
//  /* Start user code. Do not edit comment generated here */
//  /* End user code. Do not edit comment generated here */
//}
/***********************************************************************************************************************
* Function Name: r_uart2_callback_sendend
* Description  : This function is a callback function when UART2 finishes transmission.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
//static void r_uart2_callback_sendend(void)
//{
//  /* Start user code. Do not edit comment generated here */
//  flag_optical_sending = 0;
//  /* End user code. Do not edit comment generated here */
//}
/***********************************************************************************************************************
* Function Name: r_uart2_callback_error
* Description  : This function is a callback function when UART2 reception error occurs.
* Arguments    : err_type -
*                    error type value
* Return Value : None
***********************************************************************************************************************/
//static void r_uart2_callback_error(uint8_t err_type)
//{
//  /* Start user code. Do not edit comment generated here */
//  /* End user code. Do not edit comment generated here */
//}

/* Start user code for adding. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
