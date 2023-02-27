/***********************************************************************************************************************
* File Name    : variable.h
* Version      : CodeGenerator for RL78/L12 V2.00.00.07 [22 Feb 2013]
* Device(s)    : R5F10RLC
* Tool-Chain   : CA78K0R
* Description  : This file implements device driver for CGC module.
* Creation Date: 7/2/2013
***********************************************************************************************************************/

#ifndef GenusProtocol_H
#define GenusProtocol_H


/***********************************************************************************************************************
Global functions
***********************************************************************************************************************/

void transmit_byte(unsigned char);
void analyse_pkt(void);
void brcast_response(void);
void decrypt_password(void);
//unsigned char chksumsr(unsigned char*,unsigned char)
void transmit_byte(unsigned char);
unsigned int char_array_to_int(unsigned char *);
void data_dump(unsigned char ,unsigned char, unsigned char );
void prepare_ins_pkt(unsigned char);

//void write_dmd(unsigned int daily_md);
/* Start user code for function. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
#endif