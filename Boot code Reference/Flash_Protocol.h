#ifndef Flash_Protocol
#define Flash_Protocol

__far void Self_Programming_main(void);
__far void Self_Programming_Read(void);
__far void Self_Programming_Verify(void);
__far void Self_Programming_SwapCluster(void);
__far void Self_Programming_Reset(void);
__far void Self_Programming_UART_Init(void);
__far void Self_Programming_UART_Data_Recieve(void);
__far void Self_Programming_UART_Data_Transmit(void);
__far void Self_Programming_Protocol_Analyse_Pkt(void);
__far void Self_Programming_Protocol_Prepare_Pkt(unsigned char Pkt_ID);
__far void Self_Programming_LED_Init(void);
__far void FSL_delay(unsigned int count);
__far unsigned char Self_Programming_Init(void);
__far unsigned char Self_Programming_Write(unsigned long int Write_data);

#endif
