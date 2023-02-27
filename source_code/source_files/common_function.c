/***********************************************************************************************************************
* File Name       : common_function.c
* Current Version : rev_01  
* Tool-Chain      : IAR Systems
* Description     : Common functions library 
* Creation Date   : 30-12-2019
* Company         : Genus Power Infrastructures Limited, Jaipur
* Author          : dheeraj.singhal
* Version History : rev_01 : Routines added for general applications
***********************************************************************************************************************/
/************************************ Includes **************************************/
#include "common_function.h"

const us8 dec2hexAscii[16] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

/************************************ Local Functions *******************************/
void delay(us16 cntr);

/************************************ Extern Functions ******************************/

us8 reverse_8bits(us8 data);
void delay_ms(us16 count);
void delay_us(us16 us);

us8 reverse_8bits(us8 data)
{
    us8 temp=0;
    for(us8 index=0; index<8; index++)
    {
	if(data & 0x01)
	{
	    temp |= (1<<(7-index)); 
	}
	data = data>>1;
    }
    return(temp);
}

void delay_ms(us16 count)
{
	us16 index = 0;
        if(clock_select == CLOCK_24MHZ)
        {
            for (index = 0; index < count; index++)
            {
                delay(3416);
            }
        }
        else if(clock_select == CLOCK_12MHZ)
        {
            for (index = 0; index < count; index++)
            {
                delay(1708);
            }
        }
        else if(clock_select == CLOCK_6MHZ)
        {
            for (index = 0; index < count; index++)
            {
                delay(854);
            }
        }
        else if(clock_select == CLOCK_1_5MHZ)
        {
            for (index = 0; index < count; index++)
            {
                delay(214);
            }
        }
        else
        {
            for (index = 0; index < count; index++)
            {
                delay(3416);
            }
        }
}
inline void delay(us16 cntr)
{
    while(cntr != 0)
    {
        cntr--;
    }
}
void delay_us(us16 us)
{
    if(clock_select == CLOCK_24MHZ)
    {
        /* Compensate -1us for:
        *  - Copy param to AX (input params)
        *  - CALL _MCU_Delay 
        *  - Begin & End function
        * See "ASM Code Summary Table" for more detail
        */
        if (us < 2)
        {
            return;
        }
        us--;
        us--;
        
        /* NOP compensated for t2 */
        NOP();
        NOP();
        NOP();
        NOP();
        NOP();
        NOP();
        NOP();
        NOP();
        NOP();
        NOP();
        NOP();
        NOP();
        NOP();      
        NOP();
        NOP();
        NOP();
        
        /* Implementation */
        while (us)  /* Each loop must elapse 1us */
        {
            /* Put ASM code (NOP instruction here) */
            /* 12 NOP Compensated t3 */
            NOP();
            NOP();
            NOP();
            NOP();
            NOP();
            NOP();
            NOP();
            NOP();
            NOP();
            NOP();
            NOP();
            NOP();
            
            us--;   /* count down number of us */
        }
        
        /*
        *-----------------------------------------------------------------------------------------------
        *| ASM Code Summary Table                                                                      |
        *| . Setting of RL78I1B                                                                        |
        *|   fCPU = 20MHz                                                                              |
        *|   -->  1Clock = 50ns , 1us = 20 clock                                                       |
        *| . Requirement                                                                               |
        *|   > t1 + t2 + t4 = 2us (40 clocks)                                                          |
        *|   > t3           = 1us                                                                      |
        *| . Actual                                                                                    |
        *|   > t1 + t4 = 19 clock                                                                      |
        *|   > t2 = 21 clock compensated                                                                |
        *|   > t1 + t2 + t4 = 20 * 50 = 2us (Passed)                                                   |
        *|   > t3           = 8clock + 12clock compensated = 20 * 50 = 1us (Passed)                    |
        *-----------------------------------------------------------------------------------------------
        *| ASM Code                                        | Description          | Summary (by time)  |
        *-----------------------------------------------------------------------------------------------
        * _MCU_Delay:                                      |                      |                    |
        *                                                  |                      |                    |
        *                                                  |                      |                    |
        * # <<Begin Function>>                             |                      | <<Begin Function>> |
        * # MOVW Input Parameter to AX                     | 1clock            t1 | t1: 6 clocks       |
        * # CALL _MCU_Delay                                | 3clocks           t1 |                    |
        * #                                                |                      |                    |
        * #   Backup Register                              |                      |                    |
        * #   Variable Stack Allocation                    |                      |                    |
        *                                                  |                      |                    |
        *    c7          PUSH            HL                | 1clock            t1 |                    |
        *    16          MOVW            HL,AX             | 1clock            t1 |                    |
        *                                                  |                      |                    |
        * # <<Function Body>>                              |                      | <<Function Body>>  |
        * # <<Compensate>>                                 |                      | <<Compensate>>     |
        *    b7          DECW            HL                | 1clock            t2 | t2: 21 clock       |
        *    b7          DECW            HL                | 1clock            t2 |                    |
        *    f6          CLRW            AX                | 1clock            t2 |                    |
        *    47          CMPW            AX,HL             | 1clock            t2 |                    |
        *    61f8        SKNZ                              | 1clock            t2 |                    |
        *    00          NOP x 16                          | 16clock           t2 |                    |        
        *                                                  |                      |                    |
        * # while (us)                                     |                      | << 1 Whill Loop>>  |
        * # {                                              |                      | t3: 20 clocks      |
        *    f6          CLRW            AX                | 1clock         t3 t4 |                    |
        *    47          CMPW            AX,HL             | 1clock         t3 t4 |                    |
        *    xxxx        BZ              $[End Function]   | 2clocks (NM)   t3    |                    |
        *                                                  | 4clocks ( M)      t4 |                    |
        * # NOP(); 12 NOP                                  |                      |                    |
        *    00          NOP x 12                          | 12clock        t3    |                    |
        *                                                  |                      |                    |
        * #us--;                                           |                      |                    |
        *    b7          DECW            HL                | 1clock         t3    |                    |
        *                                                  |                      |                    |
        * # }                                              |                      |                    |
        *    xxxx        BR              $[# while (us)]   | 3clocks        t3    |                    |
        *                                                  |                      |                    |
        * # <<End Function>>                               |                      | <<End Function>>   |
        *    c6          POP             HL                | 1clock            t4 | t4: 13 clocks      |
        *    d7          RET                               | 6clocks           t4 |                    |
        *                                                  |                      |                    |
        *-----------------------------------------------------------------------------------------------
        */
        
    }
    else if(clock_select == CLOCK_12MHZ)
    {
        
        /* Compensate -1us for:
        *  - Copy param to AX (input params)
        *  - CALL _MCU_Delay 
        *  - Begin & End function
        * See "ASM Code Summary Table" for more detail
        */
        if (us < 2)
        {
            return;
        }
        us--;
        us--;
        
        /* No NOP compensated for t2 */
        
        /* Implementation */
        while (us)  /* Each loop must elapse 1us */
        {
            /* Put ASM code (NOP instruction here) */
            /* 16 NOP Compensated t3 */
            NOP();  /*  01  */
            NOP();  /*  02  */
            NOP();  /*  03  */
            NOP();  /*  04  */
            
            us--;   /* count down number of us */
        }
        
        /*
        *-----------------------------------------------------------------------------------------------
        *| ASM Code Summary Table                                                                      |
        *| . Setting of RL78I1B                                                                        |
        *|   fCPU = 12MHz                                                                              |
        *|   -->  1Clock = (1/12)us ~ 0.08333us --> 1us = 12clock                                      |
        *| . Requirement                                                                               |
        *|   > t1 + t2 + t4 = 2us (24 clock)                                                           |
        *|   > t3           = 1us (12 clock)                                                           |
        *| . Actual                                                                                    |
        *|   > t1 + t4      = 19 clock                                                                 |
        *|   > t2           = 5  clock                                                                 |
        *|   > t1 + t2 + t4 = 24 clock = 2us                                                           |
        *|   > t3           = 8  clock + 4clock compensated = 12clock = 1us (Passed)                   |
        *-----------------------------------------------------------------------------------------------
        *| ASM Code                                        | Description          | Summary (by time)  |
        *-----------------------------------------------------------------------------------------------
        * _MCU_Delay:                                      |                      |                    |
        *                                                  |                      |                    |
        *                                                  |                      |                    |
        * # <<Begin Function>>                             |                      | <<Begin Function>> |
        * # MOVW Input Parameter to AX                     | 1clock            t1 | t1: 6 clocks       |
        * # CALL _MCU_Delay                                | 3clocks           t1 |                    |
        * #                                                |                      |                    |
        * #   Backup Register                              |                      |                    |
        * #   Variable Stack Allocation                    |                      |                    |
        *                                                  |                      |                    |
        *    c7          PUSH            HL                | 1clock            t1 |                    |
        *    16          MOVW            HL,AX             | 1clock            t1 |                    |
        *                                                  |                      |                    |
        * # <<Function Body>>                              |                      | <<Function Body>>  |
        * # <<Compensate>>                                 |                      | <<Compensate>>     |
        *    b7          DECW            HL                | 1clock            t2 | t2: 5 clock        |
        *    b7          DECW            HL                | 1clock            t2 |                    |
        *    f6          CLRW            AX                | 1clock            t2 |                    |
        *    47          CMPW            AX,HL             | 1clock            t2 |                    |
        *    61f8        SKNZ                              | 1clock            t2 |                    |
        *                                                  |                      |                    |
        * # while (us)                                     |                      | << 1 Whill Loop>>  |
        * # {                                              |                      | t3: 12 clocks      |
        *    f6          CLRW            AX                | 1clock         t3 t4 |                    |
        *    47          CMPW            AX,HL             | 1clock         t3 t4 |                    |
        *    xxxx        BZ              $[End Function]   | 2clocks (NM)   t3    |                    |
        *                                                  | 4clocks ( M)      t4 |                    |
        * # NOP(); 4 NOP                                   |                      |                    |
        *    00          NOP x 4                           | 4clock         t3    |                    |
        *                                                  |                      |                    |
        * #us--;                                           |                      |                    |
        *    b7          DECW            HL                | 1clock         t3    |                    |
        *                                                  |                      |                    |
        * # }                                              |                      |                    |
        *    xxxx        BR              $[# while (us)]   | 3clocks        t3    |                    |
        *                                                  |                      |                    |
        * # <<End Function>>                               |                      | <<End Function>>   |
        *    c6          POP             HL                | 1clock            t4 | t4: 13 clocks      |
        *    d7          RET                               | 6clocks           t4 |                    |
        *                                                  |                      |                    |
        *-----------------------------------------------------------------------------------------------
        */
        
    }
    else if(clock_select == CLOCK_6MHZ)
    {
        
        /* Compensate -4us for:
        *  - Copy param to AX (input params)
        *  - CALL _MCU_Delay 
        *  - Begin & End function
        * See "ASM Code Summary Table" for more detail
        */
        if (us <= 4)
        {
            us = 1;
        }
        else
        {
            us = us / 2;
        }
        us--;
        
        /* 1 NOP Compensated t2 */
        NOP();  
        
        /* Implementation */
        while (us)  /* Each loop must elapse 2us */
        {
            /* Put ASM code (NOP instruction here) */
            /* 4 NOP Compensated t3 */
            NOP();  /*  01  */
            NOP();  /*  02  */
            NOP();  /*  03  */
            NOP();  /*  04  */
            
            us--;   /* count down number of us */
        }
        
        /*  
        *-----------------------------------------------------------------------------------------------   
        *| ASM Code Summary Table                                                                      |   
        *| . Setting of RL78I1B                                                                        |   
        *|   fCPU = 6MHz                                                                               |   
        *|   -->  1Clock = (1/6)us ~ 0.1666667us --> 1us = 6clock                                      |   
        *| . Requirement                                                                               |   
        *|   > t1 + t2 + t4 = 4us (24 clocks)                                                          |   
        *|   > t3           = 2us (12 clocks)                                                          |   
        *| . Actual                                                                                    |   
        *|   > t1 + t4      = 19clock                                                                  |   
        *|   > t2 = 24-19   = 5clock compensated                                                       |   
        *|   > t3           = 8clock + 4clock compensated = 12 / 6 = 2us (Passed)                     |    
        *-----------------------------------------------------------------------------------------------   
        *| ASM Code                                        | Description          | Summary (by time)  |   
        *-----------------------------------------------------------------------------------------------   
        * _MCU_Delay:                                      |                      |                    |   
        *                                                  |                      |                    |   
        *                                                  |                      |                    |   
        * # <<Begin Function>>                             |                      | <<Begin Function>> |   
        * # MOVW Input Parameter to AX                     | 1clock            t1 | t1: 6 clocks       |   
        * # CALL _MCU_Delay                                | 3clocks           t1 |                    |   
        * #                                                |                      |                    |   
        * #   Backup Register                              |                      |                    |   
        * #   Variable Stack Allocation                    |                      |                    |   
        *                                                  |                      |                    |   
        *    c7          PUSH            HL                | 1clock            t1 |                    |   
        *    16          MOVW            HL,AX             | 1clock            t1 |                    |   
        *                                                  |                      |                    |   
        * # <<Function Body>>                              |                      | <<Function Body>>  |   
        * # <<Compensate>>                                 |                      | <<Compensate>>     |   
        *    17          MOVW            AX,HL             | 1clock            t2 | t2: 5 clocks       |   
        *    312e        SHRW            AX,2H             | 1clock            t2 |                    |   
        *    16          MOVW            HL,AX             | 1clock            t2 |                    |   
        *    b7          DECW            HL                | 1clock            t2 |                    |   
        *    00          NOP                               | 1clock            t2 |                    |   
        *                                                  |                      |                    |   
        * # while (us)                                     |                      | << 1 Whill Loop>>  |   
        * # {                                              |                      | t3: 12 clocks      |   
        *    f6          CLRW            AX                | 1clock         t3 t4 |                    |   
        *    47          CMPW            AX,HL             | 1clock         t3 t4 |                    |   
        *    xxxx        BZ              $[End Function]   | 2clocks (NM)   t3    |                    |   
        *                                                  | 4clocks ( M)      t4 |                    |   
        * # NOP(); 16 NOP                                  |                      |                    |   
        *    00          NOP x 4                           | 4clock         t3    |                    |   
        *                                                  |                      |                    |   
        * #us--;                                           |                      |                    |   
        *    b7          DECW            HL                | 1clock         t3    |                    |   
        *                                                  |                      |                    |   
        * # }                                              |                      |                    |   
        *    xxxx        BR              $[# while (us)]   | 3clocks        t3    |                    |   
        *                                                  |                      |                    |   
        * # <<End Function>>                               |                      | <<End Function>>   |   
        *    c6          POP             HL                | 1clock            t4 | t4: 13 clocks      |   
        *    d7          RET                               | 6clocks           t4 |                    |   
        *                                                  |                      |                    |   
        *-----------------------------------------------------------------------------------------------}  
        */
        
    }
    else if(clock_select == CLOCK_1_5MHZ)
    {
        
        /* Compensate -8us for:
        *  - Copy param to AX (input params)
        *  - CALL _MCU_Delay 
        *  - Begin & End function
        * See "ASM Code Summary Table" for more detail
        */
        if (us <= 16)
        {
            us = 1;
        }
        else
        {
            us = us / 8;
        }
        us--;
        
        /* 1 NOP Compensated t2 */
        NOP();  
        
        /* Implementation */
        while (us)  /* Each loop must elapse 2us */
        {
            /* Put ASM code (NOP instruction here) */
            /* 4 NOP Compensated t3 */
            NOP();  /*  01  */
            NOP();  /*  02  */
            NOP();  /*  03  */
            NOP();  /*  04  */
            
            us--;   /* count down number of us */
        }
        
        /*  
        *-----------------------------------------------------------------------------------------------   
        *| ASM Code Summary Table                                                                      |   
        *| . Setting of RL78I1B                                                                        |   
        *|   fCPU = 6MHz                                                                               |   
        *|   -->  1Clock = (1/3)us ~ 0.3333333333333333us --> 1us = 3clock                             |   
        *| . Requirement                                                                               |   
        *|   > t1 + t2 + t4 = 8us (24 clocks)                                                          |   
        *|   > t3           = 4us (12 clocks)                                                          |   
        *| . Actual                                                                                    |   
        *|   > t1 + t4      = 19clock                                                                  |   
        *|   > t2 = 24-19   = 5clock compensated                                                       |   
        *|   > t3           = 8clock + 4clock compensated = 12 / 3 = 4us (Passed)                     |    
        *-----------------------------------------------------------------------------------------------   
        *| ASM Code                                        | Description          | Summary (by time)  |   
        *-----------------------------------------------------------------------------------------------   
        * _MCU_Delay:                                      |                      |                    |   
        *                                                  |                      |                    |   
        *                                                  |                      |                    |   
        * # <<Begin Function>>                             |                      | <<Begin Function>> |   
        * # MOVW Input Parameter to AX                     | 1clock            t1 | t1: 6 clocks       |   
        * # CALL _MCU_Delay                                | 3clocks           t1 |                    |   
        * #                                                |                      |                    |   
        * #   Backup Register                              |                      |                    |   
        * #   Variable Stack Allocation                    |                      |                    |   
        *                                                  |                      |                    |   
        *    c7          PUSH            HL                | 1clock            t1 |                    |   
        *    16          MOVW            HL,AX             | 1clock            t1 |                    |   
        *                                                  |                      |                    |   
        * # <<Function Body>>                              |                      | <<Function Body>>  |   
        * # <<Compensate>>                                 |                      | <<Compensate>>     |   
        *    17          MOVW            AX,HL             | 1clock            t2 | t2: 5 clocks       |   
        *    312e        SHRW            AX,2H             | 1clock            t2 |                    |   
        *    16          MOVW            HL,AX             | 1clock            t2 |                    |   
        *    b7          DECW            HL                | 1clock            t2 |                    |   
        *    00          NOP                               | 1clock            t2 |                    |   
        *                                                  |                      |                    |   
        * # while (us)                                     |                      | << 1 Whill Loop>>  |   
        * # {                                              |                      | t3: 12 clocks      |   
        *    f6          CLRW            AX                | 1clock         t3 t4 |                    |   
        *    47          CMPW            AX,HL             | 1clock         t3 t4 |                    |   
        *    xxxx        BZ              $[End Function]   | 2clocks (NM)   t3    |                    |   
        *                                                  | 4clocks ( M)      t4 |                    |   
        * # NOP(); 16 NOP                                  |                      |                    |   
        *    00          NOP x 4                           | 4clock         t3    |                    |   
        *                                                  |                      |                    |   
        * #us--;                                           |                      |                    |   
        *    b7          DECW            HL                | 1clock         t3    |                    |   
        *                                                  |                      |                    |   
        * # }                                              |                      |                    |   
        *    xxxx        BR              $[# while (us)]   | 3clocks        t3    |                    |   
        *                                                  |                      |                    |   
        * # <<End Function>>                               |                      | <<End Function>>   |   
        *    c6          POP             HL                | 1clock            t4 | t4: 13 clocks      |   
        *    d7          RET                               | 6clocks           t4 |                    |   
        *                                                  |                      |                    |   
        *-----------------------------------------------------------------------------------------------}  
        */
        
    }
}
