/************************************************************************************
 *																					*
 *  File Name	: uart.c												
 *  Contents	: serial interrupt subroutines and other communication related subroutines
 *  Copyright	: Genus Overseas Electronics Ltd.
 *  Version	: 1.0																
 *  note        :																	
 *  Author      : Manu Mishra

* reset meter data: watch dog feed is added for FG
* reset meter data: todmonth writing to eprom is added for FG
* reset meter data: power on reset added;
* instant data: present month power on hrs sended
* config_data: push display configuration added
              :Bill_date Config added
***********************************************************************************
Pragma directive
***********************************************************************************************************************/
//#pragma interrupt INTSR0 UART_iOpticalRx_Interrupt
//#pragma interrupt INTST0 UART_iOpticalTx_Interrupt
//#pragma interrupt INTSRE0 UART_iOpticalInterrupt_Error
//
//#pragma interrupt INTSR2 UART_IRDA_Rx_Interrupt
//#pragma interrupt INTST2 UART_IRDA_Tx_Interrupt
//#pragma interrupt INTSRE2 UART_IRDA_Interrupt_Error

#include "r_cg_macrodriver.h" 
#include "variable.h"
#include "r_cg_serial.h"
#include "r_cg_wdt.h"
#include "function.h"
#include "Genus_Protocol.h"
#include "Eprom_i2c.h"
/*
#define FW_id   0x11
#define FW_rev  0xC0
unsigned char pkt_length,signout_time,data_array1[21],serial_rxcounter,serial_counter,length_trans_pkt; unsigned int timeout;
unsigned char meter_id[4],utility_id[4],pkt_ID,deviceID,addressMSB,addressLSB,no_of_pkts;
unsigned char rtcfail_cntr,bat_disp_c,disp_cntr;
unsigned char data_pkts,receive_data_array1[20],ID[2],data_corrupt,opr_data[16];
unsigned char zone_index,*ch_ptr,tcount1,tcount32,btpr_c,todmonth,bill_hour,bill_min,bill_date,bill_month,md_ptr,md_ip,scroll_time,mepromloc,mepromblk,trans_id;
unsigned int  act_power,pf,tct_cnt,daily_noload_min,no_supply_prd;
extern unsigned int  act_el_power;
extern unsigned int  app_power,v_rms,mdkw_c,mdkva_c,ls_mdkw_c;
extern unsigned int freq,cum_tpr_c,pow_on,str_type1,restr_type1,str_type2,restr_type2,str_type3,restr_type3;
extern unsigned int v_cal_coff,i1_cal_coff,i2_cal_coff,kw_cal,kw_el_cal;
extern unsigned int temp_pulse_cntr;
extern unsigned int tempkva_pulse_cntr;

unsigned char PI_ID;
char  password[8];
extern flag_union flag0,flag2,flag3,flag4,flag8,flag6,flag7,flag5,flag1,flag9,flag10,flag11,flag12,flag13,flag14,flag15,flag16,flag17,flag18;
//extern struct  rtcpack rtc;

extern unsigned char md_count,mdmonth,phcorr_CT1,phcorr_CT2,freq_count,neu_count,neud_count,power_fail_cnt,tpc_count,el_count,rev_count,mag_count,el_sign,rev_el_tpr_sign,el_glow,rev_tpr_sign,pfsign;
extern unsigned long int bp_kwh_import,bp_kvah_import,bp_kwh_export,bp_kvah_export,long_int,bp_kvarh_lag,bp_kvarh;
extern unsigned long int cum_kvah_import,cum_kwh_import,cum_zkwh_import,cum_zkvah_import,cum_zkvah_export,cum_kvah_export,cum_zvawh_export,cum_kwh_export,cum_zkwh_export,dkvah_import,dkvah_export,cum_pow_off,cum_pow_on,cum_mdkw_import,cum_mdkva_import,cum_mdkw_export,cum_mdkva_export;//,dkwh;
extern unsigned long int ip_rms,in_rms,kw_1s,kw_el_1s,kva_1s;
extern unsigned int mdkw_c_import,mdkva_c_import,rising_mdkw_import,rising_mdkw_export,mdkw_c_export,mdkva_c_export;
extern unsigned int kw_pulse_cntr_import,kva_pulse_cntr_import,kw_pulse_cntr_export,kva_pulse_cntr_export,kw_cnt_import,kva_cnt_import,kw_cnt_export,kva_cnt_export;
extern unsigned char rtc_cal_sign,charge_c,discharge_c,pf_sign,md_config;
extern unsigned char cal_second,neu_test_c,LED_config,tpr_disp,rtc_fail_c;
extern unsigned char para_nos_auto,para_nos_push,hv_count,daily_flag1;
//extern unsigned char push_disp_array[26];
extern unsigned char TI_year;            //  DS 1
extern unsigned char TI_second;          //  DS 1
extern unsigned char TI_minute;          //  DS 1/
extern unsigned char TI_hour;            //  DS 1
extern unsigned char TI_day;             //  DS 1
extern unsigned char TI_month;           //  DS 1
extern unsigned char TI_dayOfWeek;           //  DS 1
extern unsigned char TI_FebDays;
extern unsigned char persist_c;
extern unsigned long int random_no;//,cum_kwh_ndpl;
extern unsigned int temp_var_int1,temp_var_int2,sd16_cnts,bill_pow_off;
const  char hcpassword[8]={'s','i','n','e','r','g','i','\0'};

//extern  Eprom_Read;*/

//====================================================================
/* Interrupt service routine for Serial Data Transmitter  */
__interrupt void UART_iOpticalInterrupt_Error(void)
{
	uint8_t err_type;
	err_type = (uint8_t)(SSR01 & 0x0007U);
	SIR01 = (uint16_t)err_type;	
}

__interrupt void UART_iOpticalTx_Interrupt(void)
{	
	time_out_f=0;
	timeout=0;
	signout_time=0;
	serial_counter++;
	if(serial_counter>=length_trans_pkt)
	{
      		serial_counter=0;
      		data_transmit_flag=0;     
  	}

  	if(data_transmit_flag==1)
      		TXD0 = data_array1[serial_counter];  
}


//====================================================================
/* Interrupt service routine for Serial Data Receiver */
__interrupt void UART_iOpticalRx_Interrupt(void)
{
	time_out_f=0;
	timeout=0;
	signout_time=0;

	data_array1[serial_rxcounter] = RXD0; //the contents of SBUF are stored in subsequent locations of data_array1
	serial_rxcounter++;

  	if (data_array1[0]==0x01 && data_transmit_flag==0)
  	{
      		pkt_length=10;							
      		command_pkt_flag=1;
      		data_pkt_flag=0;	
      	}
/*  	else if(data_array1[0]==0x02 && data_transmit_flag==0)     
      	{                     
		pkt_length=19;    
		data_pkt_flag=1;  
        	command_pkt_flag=0;
      	}
  	else if (data_array1[0]==0x41 && data_transmit_flag==0)
      	{
        	pkt_length=10;							
        	amr_cmd_f=1;
        	data_pkt_flag=0;	
        	command_pkt_flag=0;
      	}  
  	else
  	{
		pkt_length=0;	
		serial_rxcounter=0;
  	}*/
  

	if (serial_rxcounter>=pkt_length)
  	{	 						//if 8 bytes of data are received, the analyse the packet
        	serial_rxcounter=0;							
      		if((pkt_length==10)||(pkt_length==19))           //||(pkt_length==1))  
        		analyse_pkt_flag=1;      
      		else                                        
		pkt_length=0;   

      		data_array1[pkt_length]='\0';
  	}
}

void transmit_byte(unsigned char byte)
{            
  length_trans_pkt=1;
  TXD0= byte; // transmit G  
}

//unsigned char chksumsr(unsigned char *chkdata,unsigned char nob)
//{
//  unsigned char i,chksum=0;
//  
//  for(i=0;i<nob;i++)
//    chksum += *chkdata++;
//    chksum = chksum ^ 0xff;
//    chksum += 1;
//    return(chksum);
//}
//**************************Serial Routines as per new protocol********

void analyse_pkt()
{
	unsigned int i;
  	unsigned char main2;    //main1,
  	analyse_pkt_flag=0;
	
  
 /*	if(data_pkt_flag==1)    
	{
		if(chksumsr(&data_array1[3],pkt_length-4)!=data_array1[pkt_length-1])   // if checksum is not correcet
	 	{
//	    		 analyse_pkt_flag=0;     
           		 data_pkt_flag=0;
	   		 serial_rxcounter=0;
	    		 return;
		 }
      	}*/
   	//else 
	if(command_pkt_flag==1)
      	{
      	 	if(chksumsr(&data_array1[1],pkt_length-2)!=data_array1[pkt_length-1])    // if checksum is not correcet
		{
		//			analyse_pkt_flag=0;
	   		 command_pkt_flag=0;
           		 serial_rxcounter=0;
	   		 return;
		}
    	}
  	
   /*	else if(amr_cmd_f==1)
      	{
       		if(chksumsr(&data_array1[1],pkt_length-2)!=data_array1[pkt_length-1])    // if checksum is not correcet
		{
		//			analyse_pkt_flag=0;
	    		amr_cmd_f=0;
            		serial_rxcounter=0;
	    		return;
		}
     	}*/
   
    	if (command_pkt_flag==1)
  	{
		command_pkt_flag=0;
		if (data_array1[1]==0x3f)
        	{ 

          		if(((data_array1[2]=='G')&&(data_array1[3]=='S')&&(data_array1[4]=='T')&&(data_array1[5]=='G'))               //Packet giving meter ID/Utility code for signon
			||((utility_id[0]==data_array1[2])&&(utility_id[1]==data_array1[3])&&(utility_id[2]==data_array1[4])&&(utility_id[3]==data_array1[5]))
			||((meter_id[0]==data_array1[2])&&(meter_id[1]==data_array1[3])&&(meter_id[2]==data_array1[4])&&(meter_id[3]==data_array1[5])))
          		{                                
                                             
            			if((utility_id[0]==data_array1[2])&&(utility_id[1]==data_array1[3])&&(utility_id[2]==data_array1[4])&&(utility_id[3]==data_array1[5]))
              			{
					event_log_f=1;
				}
				
                		data_array1[0]=0x06;
                		data_array1[1]= 0x02;
				data_array1[2]= 0x00; 
				data_array1[3]= 0x00; 
				data_array1[4]= 'G'; 
				data_array1[5]= 'E'; 
				data_array1[6]= 'N'; 
				data_array1[7]= 'U'; 
				data_array1[8]= 'S'; 
				data_array1[9]= 'E'; 
		
             			temp_var_int1=0x5030;
           		     	temp_var_int2=DSADCR1;
                
           		     	FUN_vfill_2byteR(temp_var_int1,&data_array1[11]); 
           	     		FUN_vfill_2byteR(temp_var_int2,&data_array1[13]);
  
				random_no=temp_var_int1;
                		random_no= ((random_no <<16)| (temp_var_int2));
               
                		for(i=14;i<=18;i++)
                  			data_array1[i]=0xff;
       
				data_array1[19]= chksumsr(&data_array1[4],15); 
				data_array1[20]= '\0';
				serial_counter=0; 
                
				length_trans_pkt=20;
                		signoncmd_f=1;
				data_transmit_flag=1;
                		//__enable_interrupt();
               			// IFG2 &= 0xfd;
				TXD0=data_array1[serial_counter];
                		//IE2 |= UCA0TXIE;    //Enable Tx interrupt                                 
            		}
        	}
		else if(data_array1[1]=='B')// baud rate switching response
 		{ 
            		if(signoncmd_f==1)
            		{
              			//length_trans_pkt=1;
	      			if(data_array1[2]=='E')
              			{
                			i=char_array_to_int(&data_array1[3]);   //(int)long_int;
                			if(i==encode(random_no))
                			{
       	        				signoncmd_f=0;
                				signon_flag=1;
      	        				transmit_byte(0x06);// sign on successfully done
                			}
                			else
                  			transmit_byte(0x15);
              		 	}
	      			else
	        		transmit_byte(0x15);// U0TXBUF=0x15;            
	    		}
              		return;
		}
		else if(data_array1[1]==0x1b) //break command
		{
	    		//U0TXBUF= 0x06; 
            		signoncmd_f=0;
            		serial_counter=0;
	    		signon_flag=0;
            		event_log_f=0;
            		transmit_byte(0x06);// U0TXBUF=0x15;
            		return;
		}
        	else if(data_array1[1]==0x63)
        	{
          		if(data_array1[2]==0x01)
          		{
            			brcast_cmd1_f=1;
            			temp_var_int1=meter_id[3]*4+4;
          		}          
          		else if(data_array1[2]==0x02)
          		{
            			brcast_cmd2_f=1;              
            			temp_var_int1=meter_id[2]*4+4;
          		}
         	}
        	else if(data_array1[1]==0x64)
        	{   
          		if(data_array1[2]==0x01)
          		{
           	 		brcast_cmd1_f=1;
            			temp_var_int1=meter_id[3]*20+20;
          		}
          		else if(data_array1[2]==0x02)
          		{
            			brcast_cmd2_f=1;                
            			temp_var_int1=meter_id[2]*20+20;
          		}
        	}
		
    		if(signon_flag==1)
		{	
            		if (data_array1[1]=='r')
	    		{
//				if(data_array1[2]== 1) //device ID for instant data
//				{
//		    			length_trans_pkt=20;
//		    			pkt_ID= data_array1[3];
//		    			prepare_ins_pkt_flag=1;
//		   			// serial_counter=0;
//                   			// serial_rxcounter=0;
//                    			//prepare_ins_pkt(pkt_ID);
//				}
//				
//				else 
				if(data_array1[2] ==2)
				{                   		                   	
                      			deviceID=data_array1[2];
		      			addressMSB=data_array1[3];
		      			addressLSB=data_array1[4];
		      			no_of_pkts=data_array1[6];
                      			no_of_pkts++;
                      			data_dump_flag=1;

					
					if(addressMSB >=0x80)
                        		{
                          			//length_trans_pkt=1;
                          			data_dump_flag=0;      // added for 24C256 only
		          			//data_transmit_flag=1;                   //no of packets is of 1 byte only, protocol has provision for 2 Bytes
	                 			transmit_byte(0x17);// U0TXBUF=0x17;
                          			return;
                        		}

                     			transmit_byte(0x06);                    
				}
				else if(data_array1[2] ==4)
				{        
                      			deviceID=data_array1[2];
		      			addressMSB=data_array1[3];
		      			addressLSB=data_array1[4];
		      			no_of_pkts=data_array1[6];
                      			no_of_pkts++;
                      			data_dump_flag=1;		      

		      			if(addressMSB >=0x80)
                        		{
                          			//length_trans_pkt=1;
                          			data_dump_flag=0;      // added for 24C256 only
		          			//data_transmit_flag=1;                   //no of packets is of 1 byte only, protocol has provision for 2 Bytes
	                 			transmit_byte(0x17);// U0TXBUF=0x17;
                          			return;
                        		}
               
                     			transmit_byte(0x06);
                    		}    	
				else
				{
                    			//length_trans_pkt=1;
		   			// serial_counter=0;
                   			// data_transmit_flag=1;
                   			transmit_byte(0x17);//  U0TXBUF=0x17;
		    			//pkt_length=0;
                   			// data_array1[2]=0;
				}
	    		}     //read ends

//	    		else if(data_array1[1]=='w')
//	    		{
//				if(data_array1[2]=='p')                              //Password Check
//				{
//		    			//length_trans_pkt=1;    
//		    			password_check_flag=1;
//                   			transmit_byte(0x06);// U0TXBUF=0x06;
//				}
//              
//				else if((data_array1[2]=='c')&&(write_enable_flag==1))  //Data configuration Command
//				{
//		    			//length_trans_pkt=1;
//		    			ID[1]=data_array1[3];
//		    			data_pkts=data_array1[4];
//                   			transmit_byte(0x06);
//				}
//                		
//				else if((data_array1[2]=='R')&&(write_enable_flag==1))  //Data Reset Command
//				{
//		    			//length_trans_pkt=1;
//		    			data_reset_flag=1;
//		    			ID[1]=data_array1[3];
//                   			transmit_byte(0x06);// U0TXBUF=0x06;
//
//	        		}
//	    		}   //write ends
//
//	    		else
//	    		{
////				serial_counter=0;
////				length_trans_pkt=0;
////				data_array1[1]=0;
//				pkt_length=0;
//	    		}
		}   // sign on ends
    	}   //command pkt ends here
//		
//    	else if ((data_pkt_flag==1)&&(signon_flag==1))
//    	{
//		data_pkt_flag=0;
//		for(main2=0;main2<18;main2++)
//	  		receive_data_array1[main2]=data_array1[main2+1];
//
//		if(receive_data_array1[0]==0)
//		{
//	    		if(password_check_flag==1)
//	    		decrypt_password();
//		}
//		else if(receive_data_array1[0]==ID[1])
//            	{
//	        	config_data_flag=1;
//                	//length_trans_pkt=1;
//                	//U0TXBUF=0x06;
//            	}	
//    	}
//	
//  	else if(amr_cmd_f==1)  
//  	{
//    		amr_cmd_f=0;
//    		if((meter_id[0]==data_array1[1])&&(meter_id[1]==data_array1[2])&&(meter_id[2]==data_array1[3])&&(meter_id[3]==data_array1[4]) && (data_array1[5]==0x27))
//    		{
//      			main2=0;
//      			temp_var_int1=(unsigned int)data_array1[6]*256+data_array1[7];
//        		if(temp_var_int1 & 0x0002)
//          	 		main2 = 1;
//        		if(temp_var_int1 & 0x0008)
//          			main2 +=2;
//        		if(temp_var_int1 & 0x0020)
//          			main2 +=4;
//        		if(temp_var_int1 & 0x0080)
//          			main2 +=8;
//        		if(temp_var_int1 & 0x0800)
//          			main2 +=16;
//        		if(temp_var_int1 & 0x2000)
//          			main2 +=32;
//
//        		if(main2==dt.day)
//        		{
//            			data_array1[0]= 0x41;            
//            			data_array1[1]= 0x27;
//            			data_array1[2]= 0x00; 
//
//            			Eprom_Read(0x08f0);
//              			for(i=0;i<16;i++)
//                			data_array1[i+3]= opr_data[i]; 
//				serial_counter=0; 
//                
//				length_trans_pkt=19;
//				data_transmit_flag=1;
//                		//IFG2 &= 0xfd;  
//				TXD0=data_array1[serial_counter];
//                		//IE2 |= UCA0TXIE; 
//        		}
//    		}
//  	}

}

//void brcast_response(void)
//{
//      data_array1[0]=0x09;
//      
//      data_array1[1]= meter_id[0];
//      data_array1[2]= meter_id[1]; 
//      data_array1[3]= meter_id[2]; 
//      data_array1[4]= meter_id[3]; 
//      data_array1[5]= 0; 
//      data_array1[6]= chksumsr(&data_array1[1],5); 
//  
//      length_trans_pkt=7;
//      data_transmit_flag=1;
//
//      //IFG2 &= 0xfd;  
//      TXD0=data_array1[serial_counter];
//      //IE2 |= UCA0TXIE; 
//}
//
///*void delay_2()
//{
//  asm("nop");
//  asm("nop");
//}
//*/
//
//
//void decrypt_password()
//{
//	unsigned char main12,main13,swaprnd1,swaprnd2; //,main1;
//	
//	password_check_flag=0;
//         
//	main12=data_array1[10] & 0x0f;
//	main13=data_array1[10] & 0xf0;
//	main12=main12<<4;
//	main13=main13>>4;
//	swaprnd1=main12 | main13;
//
//	main12=data_array1[11] & 0x0f;
//	main13=data_array1[11] & 0xf0;
//	main12=main12<<4;
//	main13=main13>>4;
//	swaprnd2=main12 | main13;
//
//	password[0]=receive_data_array1[2] ^ data_array1[11];
//	password[1]=receive_data_array1[3] ^ swaprnd1;
//	password[2]=receive_data_array1[4] ^ data_array1[10];
//	password[3]=receive_data_array1[5] ^ swaprnd2;
//	password[4]=receive_data_array1[6] ^ data_array1[11];
//	password[5]=receive_data_array1[7] ^ swaprnd1;
//	password[6]=receive_data_array1[8] ^ data_array1[10];
//	password[7]='\0';
//
//	if(change_pw_f==1)
//	{
//		change_pw_f=0;  
//	}
//	else
//	{
//	  	Eprom_Read(0x0520);
//	  	//main1=chksumsr(opr_data,15);
//	  	//if(main1!=opr_data[15])
//	  	//if(chksum_ok ==0)   
//	  	//data_corrupt=1;
//	  	opr_data[7]='\0';
//
//		if((strcmp(password,hcpassword)==0)||(strcmp(password,(char*)opr_data)==0))
//		{
//	    	//length_trans_pkt=1;
//	    	write_enable_flag=1;
//	   	transmit_byte(0x06);// U0TXBUF=0x06;
//	    
//		}
//		else
//		{
//	  		signon_flag=0; 
//	  		write_enable_flag=0;
//	 		 //sign off
//		}
//	}  
//}
//
void prepare_ins_pkt(unsigned char pkt_ID)
{
    	unsigned char i;
   	unsigned long int energy_import,energy_export,energy_net,hi_kwh;
    
    	data_array1[0]=0x06;
    	data_array1[1]=0x02;
    	data_array1[2]=pkt_ID;
    	data_array1[3]=0;
	
    	for(i=0;i<16;i++)
	{
       		data_array1[i+4] = 0xff;
	}	
	
    	switch(pkt_ID)
    	{
        case 1:
               // Eprom_Read(0x0500);
                for(i=0;i<16;i++)
                  data_array1[i+4] = opr_data[i];                              
        	break;
              
                
        case 0x23:    
        	
//                fill_4bdata(energy_net ,&data_array1[7]);									
//               
//                fill_4bdata(energy_net ,&data_array1[11]);   							
        	break;						 
						 
	case 0xc8:
		data_array1[4]=dt.sec;
                time_stamp(&data_array1[5]);
              //  data_array1[10]=TI_dayOfWeek;
		
               // if(rtcfail_f==1)
               //   data_array1[11]=rtcfail_cntr;
                //else
                //  data_array1[11]=0;
		data_array1[12]=TOD_u8zone_index;
                data_array1[18]=md_ip;
                //ch_ptr = (unsigned char*)&pow_on;
                //data_array1[14] =0xaa;// *ch_ptr++;       
                //data_array1[13] =0xaa;// *ch_ptr;               //msb
        	break;
	
	case 0xc9:
                FUN_vfill_2byteR(MET_u16pf,&data_array1[5]);
                FUN_vfill_2byteR(MET_u16freq,&data_array1[7]);             
                data_array1[8] = MET_u8pfsign;
               // data_array1[9] = rev_tpr_sign;
               // data_array1[10]= el_sign;
                FUN_vfill_3byteR(MET_u32ip_rms,&data_array1[13]);

                FUN_vfill_3byteR(MET_u32in_rms,&data_array1[16]);
                FUN_vfill_2byteR(MET_u16v_rms,&data_array1[18]);
          
                break;

	 case 0xca:
	 
                FUN_vfill_2byteR(MET_u16Kva,&data_array1[5]);
                FUN_vfill_2byteR(MET_u16Kw,&data_array1[7]);
                FUN_vfill_2byteR(MET_u16ELKw,&data_array1[9]);
                if(TP1.b.mag_tpr_f==0)
                {
                  data_array1[10]=MET_bkw_negative_f;
                  data_array1[11]=MET_bkw_el_negative_f;
                }
                else
                {
                  data_array1[10]=0;
                  data_array1[11]=0;                  
                }
                                  
        
                //FUN_vfill_3byteR(cum_neutemp_defraud,&data_array1[18]);

                break;

	 case 0xcb:
//                FUN_vfill_3byteR(cum_kvah_import,&data_array1[6]);
//             //  FUN_vfill_3byteR(cum_zkvah_import,&data_array1[9]);
//                FUN_vfill_3byteR(cum_kwh_import,&data_array1[12]);
//             //   FUN_vfill_3byteR(cum_zkwh_import,&data_array1[15]);             
                break;

	case 0xcc:
	
//		data_array1[4] = mag_count;
//		data_array1[5] = rev_count;
//		data_array1[6] = el_count;
//		data_array1[7] = tpc_count;
//		data_array1[8] = neu_count;
//		data_array1[9] = freq_count;
//                data_array1[17] = neud_count;//current releated and voltage releated tamper in DLMS 
                FUN_vfill_2byteR(TPR_u16cum_tpr_c,&data_array1[13]);
     
                data_array1[14]=0;
                if(TP1.b.mag_tpr_f==1)
                  data_array1[14] |=0x80;
//                if(TP1.b.rev_tpr_f==1)
//                  data_array1[14] |=0x40;
                if(TP2.b.el_tpr_f==1)
                  data_array1[14] |=0x20;
                if(TP5.b.tc_tpr_f==1)
                  data_array1[14]|=0x10;
              
                if(TP3.b.neu_tpr_f==1)
                  data_array1[14] |=0x04;
              
//                if(rtcfail_f==1)
//                  data_array1[14]|=0x02;
//                if(acknotr_f0==1)
//                  data_array1[14]|=0x01;  // 0th bit
                
                data_array1[18]=0;
                if(TP6.b.bat_discharge_f ==1)
                  data_array1[18] |=0x40;
                if(TP4.b.AbFreq_tpr_f ==1)
                  data_array1[18] |=0x20;
             //   if(el_tpr_f==1)
               //   data_array1[18] |=0x10;
                if(TP3.b.neu_tpr_f==1)
                  data_array1[18] |=0x02;       
                
                                              // tamper flags not added
		break;

	case 0xcd:
//                
//                i = mdmonth-1;
//                if(i == 0xff)
//                {
//                 // if(bill_month==0)
//                    i = 0x0c;                         // no. of billing month
//                 // else
//                  //  i = 0x06;                         // no. of billing month
//                }
//                i = i << 4;
//                Eprom_Read(0x0400+i);
//                data_array1[4] = opr_data[0];
//                data_array1[5] = opr_data[1];
//                for(i=0;i<5;i++)
//                  data_array1[i+6] = opr_data[i+3];
//                
//                
//                i = mdmonth-1;
//                if(i == 0xff)                
//                {
//                  //if(bill_month==0)
//                    i = 0x0c;                         // no. of billing month
//                  //else
//                   // i = 0x06;                         // no. of billing month
//                }
//                i = i << 4;
//            ////    i += 0x80;
//                Eprom_Read(0x0600+i);
//                data_array1[11] = opr_data[0];
//                data_array1[12] = opr_data[1];
//                for(i=0;i<5;i++)
//                  data_array1[i+13] = opr_data[i+3];
//                
//                
//                data_array1[18] = md_count;
          
		break;

	case 0xce:
//          
//                i = mdmonth-2;
//                if(i == 0xff)
//                 {
//                  //if(bill_month==0)
//                    i = 0x0c;                         // no. of billing month
//                  //else
//                  //  i = 0x06;                         // no. of billing month
//                }
//                i = i << 4;
//                Eprom_Read(0x0400+i);
//                data_array1[4] = opr_data[0];
//                data_array1[5] = opr_data[1];
//                for(i=0;i<5;i++)
//                  data_array1[i+6] = opr_data[i+3];
//               
//                i = mdmonth-2;
//                if(i == 0xff)
//                {
//                  //if(bill_month==0)
//                    i = 0x0c;                         // no. of billing month
//                 // else
//                  //  i = 0x06;                         // no. of billing month
//                }               
//								i = i << 4;
//              ////  i += 0x80;
//                Eprom_Read(0x0600+i);
//                data_array1[11] = opr_data[0];
//                data_array1[12] = opr_data[1];
//                for(i=0;i<5;i++)
//                  data_array1[i+13] = opr_data[i+3];
//                
//                data_array1[18] = 0x0c;              //no. of billing month
//            
		break;

	case 0xcf:
/*		ch_ptr = (unsigned char*)&bp_kwh;
                data_array1[4] = *ch_ptr++;                //LSB
                data_array1[5] = *ch_ptr++;
                data_array1[6] = *ch_ptr;
  */
                //fill_data1(bp_kwh,&data_array1[4]);
               // Eprom_Read(0x09,0xd0);   // billing Energy
             //   for(i=0;i<16;i++)
                //  data_array1[i+4] = opr_data[i];
          
		break;
	case 0xdb:
//                FUN_vfill_3byteR(cum_kvah_export,&data_array1[6]);
//              //  FUN_vfill_3byteR(cum_zkvah_export,&data_array1[9]);
//                FUN_vfill_3byteR(cum_kwh_export,&data_array1[12]);
//              //  FUN_vfill_3byteR(cum_zkwh_export,&data_array1[15]);
//                
            break;      
                
case 0xdd:
                
//               i = mdmonth-1;
//                if(i == 0xff)
//                 {
//                  //if(bill_month==0)
//                    i = 0x0c;                         // no. of billing month
//                  //else
//                  //  i = 0x06;                         // no. of billing month
//                }
//                i = i << 4;
//                Eprom_Read1(0x0400+i);
//                data_array1[4] = opr_data[0];
//                data_array1[5] = opr_data[1];
//                for(i=0;i<5;i++)
//                  data_array1[i+6] = opr_data[i+3];
//								
//                i = mdmonth-1;
//                if(i == 0xff)
//                 {
//                  //if(bill_month==0)
//                    i = 0x0c;                         // no. of billing month
//                  //else
//                  //  i = 0x06;                         // no. of billing month
//                }
//                i = i << 4;
//                
//                Eprom_Read1(0x0600+i);
//                data_array1[11] = opr_data[0];
//                data_array1[12] = opr_data[1];
//                for(i=0;i<5;i++)
//                  data_array1[i+13] = opr_data[i+3];
//                
//                
//                //data_array1[18] = md_count;
//          
		break;                
                
	case 0xde:
          
//                i = mdmonth-2;
//                if(i == 0xff)
//                 {
//                  //if(bill_month==0)
//                    i = 0x0c;                         // no. of billing month
//                  //else
//                  //  i = 0x06;                         // no. of billing month
//                }
//                i = i << 4;
//                Eprom_Read1(0x0400+i);
//                data_array1[4] = opr_data[0];
//                data_array1[5] = opr_data[1];
//                for(i=0;i<5;i++)
//                  data_array1[i+6] = opr_data[i+3];
//               
//                i = mdmonth-2;
//                if(i == 0xff)
//                {
//                  //if(bill_month==0)
//                    i = 0x0c;                         // no. of billing month
//                 // else
//                  //  i = 0x06;                         // no. of billing month
//                }               
//								i = i << 4;
//              ////  i += 0x80;
//                Eprom_Read1(0x0600+i);
//                data_array1[11] = opr_data[0];
//                data_array1[12] = opr_data[1];
//                for(i=0;i<5;i++)
//                  data_array1[i+13] = opr_data[i+3];
//                
//                data_array1[18] = 0x0c;              //no. of billing month
//            
		break;  
		
	case 0x27:
//                Eprom_Read(0x08f0);
//                for(i=0;i<16;i++)
//                  data_array1[i+4] = opr_data[i];
               break;
                
	case 0x28:
                  
                  data_array1[4]=3;
                 for(i=0;i<9;i++)
                  data_array1[i+5] =0;
               break;
	       
	case 0x30:           
	
//                Eprom_Read(0x0890);
//                for(i=0;i<4;i++)
//                  data_array1[i+4] = opr_data[i];
//                Eprom_Read(0x08a0); // meter mfd. date 
//                for(i=3;i<6;i++)
//                  data_array1[i+5] = opr_data[i];
//                if(rtc_calib_f==0)
//                {
//                  data_array1[11] = rtc_cal_sign;
//                  data_array1[12] = cal_second;
//                }
//                else
//                {
//                  data_array1[11] = 0xff;
//                  data_array1[12] = 0xff;
//                } 
//                data_array1[13]=0;
//                if(mag_tpr_f==1)
//                  data_array1[13] |=0x80;   
////                if((rev_tpr_sign == 1) || (rev_el_tpr_sign == 1))
////                  data_array1[13] |=0x40; 
//                if((el_glow == 1)  && (mag_tpr_f==0))
//                  data_array1[13] |=0x20; 
//                if(tc_tpr_f==1)
//                  data_array1[13]|=0x10;                
//              //  if(hv_tamper_f==1)
//                //  data_array1[13] |=0x08;
//                if(neu_fea_f5==1)
//                  data_array1[13] |=0x04;                   
//                if(rtcfail_f==1)
//                  data_array1[13]|=0x02;
//                if(acknotr_f0==1)
//                  data_array1[13]|=0x01;    
//                
//                data_array1[14]=0;
//              //  if(mag_tpr_f==1)
//                 // data_array1[14] |=0x80;   
//                if(bat_discharge_f == 1)
//                  data_array1[14] |=0x40; 
//               // if(el_glow == 1 && ph_ct_f1==1)
//                //  data_array1[14] |=0x20; 
//               // if(tc_tpr_f==1)
//                 // data_array1[14]|=0x10;                
//               // if((v_rms < 16000) && v_tpr_f==0 && v_rms >= 9000)
//                //  data_array1[14] |=0x04;
//               // if(neu_fea_f5==1)
//                 // data_array1[14] |=0x04;                   
////                if(neu_fea_f5==1)
////                  data_array1[14]|=0x02;
//               // if(acknotr_f0==1)
//                  //data_array1[14]|=0x01;  
//                data_array1[17] = 0x0c;// Meter constant 3200
//                data_array1[18] = 0x80;
//                
               break;                 
//   case 0xa0:                
//                hi_kwh=((kw_pulse_cntr_import*(unsigned long)3125)/1000); 
//                hi_kwh = (cum_kwh_import*1000)+hi_kwh;
//                fill_4bdata(hi_kwh,&data_array1[7]);
//                hi_kwh=((kva_pulse_cntr_import*(unsigned long)3125)/1000); 
//                hi_kwh = (cum_kvah_import*1000)+hi_kwh;
//                fill_4bdata(hi_kwh,&data_array1[11]);
//		break;  
//	case 0xa1:
//                fill_4bdata(kw_1s,&data_array1[7]);
//                fill_4bdata(kw_el_1s,&data_array1[11]);
//		break;  
//	case 0xa2:
//                if(ph_ct_f1==0)
//                {
//                  if(kw_1s>kva_1s)
//                    fill_4bdata(kw_1s,&data_array1[7]);
//                  else
//                    fill_4bdata(kva_1s,&data_array1[7]);
//                }
//                else
//                {
//                  if(kw_el_1s>kva_1s)
//                    fill_4bdata(kw_el_1s,&data_array1[7]);
//                  else
//                    fill_4bdata(kva_1s,&data_array1[7]);                
//                }
//		break;                     
//
//		 case 0xe0:    
//                energy_import = ((cum_kvah_import*100) + ((kva_pulse_cntr_import*(unsigned long)3125)/1000));
//                fill_4bdata(energy_import,&data_array1[7]);	
//								
//                energy_import = ((cum_kwh_import*100) + ((kw_pulse_cntr_import*(unsigned long)3125)/1000));
//                fill_4bdata(energy_import ,&data_array1[11]);   		
//               break;	
//							 
//			case 0xe1:    
//                
//								energy_export = ((cum_kvah_export*100) + ((kva_pulse_cntr_export*(unsigned long)3125)/1000));							
//                fill_4bdata(energy_export,&data_array1[7]);	
//								energy_export = ((cum_kwh_export*100) + ((kw_pulse_cntr_export*(unsigned long)3125)/1000));							
//                fill_4bdata(energy_export,&data_array1[11]);   		
//              break;					 
//							 
							 
							 
 /*       case 0xd0:
	 	ch_ptr = (unsigned char*)&v_rms;
                data_array1[5] = *ch_ptr++;
                data_array1[4] = *ch_ptr++;
                ch_ptr = (unsigned char*)&v_cal_coff;
                data_array1[7] = *ch_ptr++;
                data_array1[6] = *ch_ptr++;
                ch_ptr = (unsigned char*)&ip_rms;
                data_array1[10] = *ch_ptr++;
                data_array1[9] = *ch_ptr++;
                data_array1[8] = *ch_ptr++;
                ch_ptr = (unsigned char*)&i1_cal_coff;
                data_array1[12] = *ch_ptr++;
                data_array1[11] = *ch_ptr++;
                ch_ptr = (unsigned char*)&in_rms;
                data_array1[15] = *ch_ptr++;
                data_array1[14] = *ch_ptr++;
                data_array1[13] = *ch_ptr++;
                ch_ptr = (unsigned char*)&i2_cal_coff;
                data_array1[17] = *ch_ptr++;
                data_array1[16] = *ch_ptr++;
                
                break;


	case 0xd1:
                ch_ptr = (unsigned char*)&kw_1s;
                data_array1[4] = *ch_ptr++;                //LSB
                data_array1[5] = *ch_ptr++;
                data_array1[6] = *ch_ptr++;
                data_array1[7] = *ch_ptr++;
                ch_ptr = (unsigned char*)&kw_cal;
                data_array1[8] = *ch_ptr++;
                data_array1[9] = *ch_ptr++;
                data_array1[10] = phcorr_CT1;
                ch_ptr = (unsigned char*)&kw_el_1s;
                data_array1[11] = *ch_ptr++;                //LSB
                data_array1[12] = *ch_ptr++;
                data_array1[13] = *ch_ptr++;
                data_array1[14] = *ch_ptr++;
                ch_ptr = (unsigned char*)&kw_el_cal;
                data_array1[15] = *ch_ptr++;
                data_array1[16] = *ch_ptr++;
                data_array1[17] = phcorr_CT2;
          
		break;

*/
	//default:	
	//       transmit_byte(0x17);
               // length_trans_pkt=1;
	       // data_array1[0]=0x17;
							

	}

        data_array1[19] = chksumsr(&data_array1[4],15);

	data_transmit_flag=1;
	prepare_ins_pkt_flag=0;
	serial_counter=0;
	TXD0=data_array1[serial_counter];
}

//*************************************************//
void data_dump(unsigned char deviceID1,unsigned char addMSB,unsigned char addLSB)
{
	unsigned char i; 
  	if(data_transmit_flag==1)
  	return;
 	          
  
  	serial_counter=0;         
  	if(deviceID1==3)
     		Eprom_Read(addMSB*0x100+addLSB);
//  	else
//     		Eprom_Read1(addMSB*0x100+addLSB);
    
   	data_array1[0]=0x02;
    	data_array1[1]=addMSB;
    	data_array1[2]=addLSB;
    	for(i=3;i<=17;i++)
      		*( data_array1 +i) = *(opr_data+i-3);
        data_array1[18]=chksumsr(&data_array1[3],15);
    	data_array1[19]='\0';
							
    	if((addressLSB)>=0xf0)//if(eepg+16>=0xf0)
       	{
          	addressLSB=0;
          	addressMSB+=0x01;
        }
    	else
        {
          	addressLSB+=16;
        }

  
	no_of_pkts--;
  
      	length_trans_pkt=19;
      	data_transmit_flag=1;
      	if(no_of_pkts==0)
        {
        	length_trans_pkt=0;
		data_transmit_flag=0;
        	data_dump_flag=0;
          	return;
        }
      	TXD0=data_array1[serial_counter];
}



//void reset_meter_data(void)
//{
//  unsigned char i,j;
//  
//  //  for(i=0;i<16;i++)
//  //  *(opr_data+i)=0;
//  fill_oprzero();
//  switch(ID[1])
//  {
//    case 1:
//      
//      for(i=0;i<=2;i++)   //Energies up to block 0x04
//      {
//        for(j=0;j<16;j++)
//          Eprom_Write(i*0x100+(j*16));   // block 00,01,02 is reset
//          //if(batt_disp_f5==0 )
//            WDTCTL = WDT_ARST_1000+WDTNMI+WDTNMIES;              
//
//      }  
//	   for(i=0;i<3;i++)   //Energies up to block 0x04
//      {
//        for(j=0;j<16;j++)
//          Eprom_Write1(i*0x100+(j*16));   // block 00,01,02 is reset
//          //if(batt_disp_f5==0 )
//            WDTCTL = WDT_ARST_1000+WDTNMI+WDTNMIES;              
//
//      }
//	  
//      
//      Eprom_Write(0x0560);     //kw_pulse_cntr location is reset
//			Eprom_Write(0x45f0);     //rollover flags clear
//      opr_data[0]=TI_year;
//      opr_data[1]=TI_month;
//      opr_data[2]=TI_day;
//      opr_data[14]=0x01;
//      Eprom_Write(0x08f0);
//      //cum_neutemp_defraud=0;
////      cum_kwh = 0;
////      
////      //cum_kwh_ndpl = 0;
////      cum_kvah = 0;
////    //  cum_zkvah = 0;
////     // cum_kvarh_lag = 0;      
////     // cum_zkvarh_lag = 0;
////      cum_zkwh = 0;
//////      cum_kvarh_lag_lag_lg = 0;
////      ////dkwh = 0;
////      dkvah=0;
////  //    cum_kvarh_lag_lag_ld = 0;
////      //delta_kwh=0;
////      
////    //  md_ptr=0x70;
////      //fill_oprzero();
////   //   Eprom_Write(0x07,0x70);
////  //    Eprom_Write(0x07,0x80);
////  //    Eprom_Write(0x07,0x90);
////    //  Eprom_Write(0x07,0x60);    
////    //  Eprom_Write(0x09,0xe0);  // delta_kwh
////      mdkw_c = 0;
////      mdkva_c=0;
////      ls_mdkw_c=0;
////      tcount1 = 0;
//			cum_kwh_import = 0;
//      //cum_kwh_ndpl=0;
//      cum_kvah_import = 0;
//      cum_zkvah_import = 0;
//      cum_zkwh_import = 0;
//      
//      cum_kwh_export = 0;
//      cum_kvah_export = 0;
//      cum_zkvah_export = 0;
//      cum_zkwh_export = 0;      
////      cum_kvarh_lg = 0;
////      dkwh = 0;      
//      dkvah_import = 0;
//      dkvah_export = 0;
//  //    cum_kvarh_ld = 0;
//      mdkw_c_import = 0;
//      mdkva_c_import = 0;
//      
//      mdkw_c_export = 0;
//      mdkva_c_export = 0;   
//      
////      rising_mdkw_import=0;
////      rising_mdkw_export=0;
//      
//      tcount1 = 0;
//      tcount32 = 0;
//      kvah_rollover_import_f=0;
//      zkvah_rollover_import_f=0;
//      kvah_rollover_export_f=0;
//      zkvah_rollover_export_f=0;      
//      kw_pulse_cntr_import=0;
//      temp_pulse_cntr=0;
//      tempkva_pulse_cntr=0;        
//      kva_pulse_cntr_import=0;  
//      kw_pulse_cntr_export=0;
//      kva_pulse_cntr_export=0;        
//      kw_cnt_import=0;
//      //kva_cnt_import=0;
//      kw_cnt_export=0;
//  //    kvah_rollover_f=0;
// //     opr_data[0]=0x00;
//   //   kw_pulse_cntr=0;
//   //   kva_pulse_cntr=0;
//      md_ip=0x1e;
//      opr_data[0]=0x1e;
//      Eprom_Write(0x06e0);
//      
//  //    length_trans_pkt=1;
//    //  U0TXBUF=0x06;
//      break;
//		
//    case 2:
//      for(j=0;j<16;j++)            ////        // Billing Energy Block 08
//      {
//        Eprom_Write(0x0400+(j*16));
//        Eprom_Write1(0x0400+(j*16));
//		    Eprom_Write1(0x0600+(j*16)); 
//				WDTCTL = WDT_ARST_1000+WDTNMI+WDTNMIES;      
//      }            
//
//      for(i=0x06;i<0x08;i++)   
//      {
//        for(j=0;j<13;j++)
//          Eprom_Write(i*0x100+(j*16));   // block 06,07 is reset
//          
//          WDTCTL = WDT_ARST_1000+WDTNMI+WDTNMIES;      
//		          
//      } 
//			
////			for(i=0x38;i<0x45;i++)   //TOD Billing reset
////      {
////        for(j=0;j<16;j++)
////          Eprom_Write(i*0x100+(j*16));   
////          WDTCTL = WDT_ARST_1000+WDTNMI+WDTNMIES;              
////      
////      } 
//	  
////	  for(i=0x08;i<0x15;i++)   //TOD Billing export reset
////      {
////        for(j=0;j<16;j++)
////          Eprom_Write1(i*0x100+j*16);   // block 00,01,02 is reset
////          //if(batt_disp_f5==0 )
////          WDTCTL = WDT_ARST_1000+WDTNMI+WDTNMIES;                    
////      } 
//			
//          
//      mdmonth = 1;
//      todmonth=1;
//      bp_kwh_import = 0;
//			bp_kwh_export=0;
//			bp_kvah_import = 0;
//			bp_kvah_export=0;
//     //// bp_kvah=0;
//    ////  bp_kvarh_lag=0;
//      md_count = 0;
//      //opr_data[0] = 0;
//      //opr_data[1] = 0;
//      // opr_data[1] = 0; 
//      md_config=0;                       // for test MD is configured to MD kva display
//      //Eprom_Write(0x04,0xd0);      
//      Eprom_Write(0x08d0); 
//			Eprom_Write(0x0900); 
//      Eprom_Write(0x0920); 
//			
//      bill_date = 0x01;
//      bill_month = 0;
//      bill_hour = 0;
//      bill_min = 0;      
//      opr_data[0] = 0x01;
////      opr_data[1] = 0;
////      opr_data[2] = 0;
////      opr_data[3] = 0;
//      Eprom_Write(0x07d0);
//      
//			fill_oprzero();
//      time_stamp(&opr_data[2]);
//      opr_data[7] = 1;
//      opr_data[0] = 0;
//      opr_data[14] = todmonth;
//      Eprom_Write(0x0570);
//     // fill_oprzero();
//     // Eprom_Write(0x09,0xd0); // billing energy
//
//      
//     // scroll_time = 10;  // by default 6 sec.
//     // opr_data[0]=scroll_time;
//     // Eprom_Write(0x08,0xf0);
//      cum_mdkw_import=0;
//      cum_mdkva_import=0;
//			cum_mdkw_export=0;
//			cum_mdkva_export=0;
//      cum_pow_off=0;
//			cum_pow_on=0;
//      bill_pow_off=0;
//      
//     // LED_config=0x04;
//      
//     // opr_data[0] = 0x04;
//      
//      
//    /*  if(event_log_f==1)
//      {
//       trans_id=22;
//       store_event();
//      }
//      */
//      //length_trans_pkt=1;
////      U0TXBUF=0x06;
//      break;
//
//    case 3:
//      for(j=0;j<16;j++)                   // Power on hours
//        Eprom_Write(0x0300+(j*16));
//
//      //pow_on = 0;
//     // length_trans_pkt=1;
//     // U0TXBUF=0x06;
//      pon_reset();
//      
//      break;
//
//    case 5:
//      for(i=0x0a;i<0x28;i++)   //tamper
//      {
//        for(j=0;j<16;j++)
//          Eprom_Write(i*0x100+(j*16));
//        WDTCTL = WDT_ARST_1000+WDTNMI+WDTNMIES;                  
//      }
//			for(i=0x2b;i<0x2c;i++)   //tamper
//      {
//        for(j=0;j<16;j++)
//          Eprom_Write(i*0x100+(j*16));
//        WDTCTL = WDT_ARST_1000+WDTNMI+WDTNMIES;                  
//      }
//      
//      for(j=0;j<16;j++)                   
//        Eprom_Write(0x0900+(j*16));
//      for(j=11;j<16;j++)                   
//        Eprom_Write(0x0500+(j*16));   
//      for(j=6;j<9;j++)                   
//        Eprom_Write(0x0800+(j*16));      
//      WDTCTL = WDT_ARST_1000+WDTNMI+WDTNMIES; 
//      for(j=10;j<15;j++)                   
//        Eprom_Write(0x0800+(j*16));       
//      for(j=0;j<3;j++)                   
//        Eprom_Write(0x0800+(j*16));
//       neu_test_c=0;
//      // opr_data[0]=0;
//       Eprom_Write(0x04f0);
//      //Eprom_Write(0x05,0xe0);     //mag
//      Eprom_Write(0x05b0);
//      Eprom_Write(0x07f0);     //tpc
//      //Eprom_Write(0x05,0xf0);     //cum tpr count reset
//      //Eprom_Write(0x08,0xb0);
//      //Eprom_Write(0x08,0x60);
//      //Eprom_Write(0x08,0x80);  
//      //Eprom_Write(0x05,0xb0);
//      
//     
//      opr_data[1]=0x0a;
//      Eprom_Write(0x05d0);     //mag
////      opr_data[1]=0x0c;
////      Eprom_Write(0x0910);     //rev
//      opr_data[1]=0x16;
//      Eprom_Write(0x0940);     //neud 
//      opr_data[1]=0x12;
//      Eprom_Write(0x0970);     //neu
//      opr_data[1]=0x1a;
//      Eprom_Write(0x09a0);     //freq
//			opr_data[1]=0x2b;        //earth
//      Eprom_Write(0x0810);
//			
////      opr_data[1]=0x1b;
////      Eprom_Write(0x08,0x70);     //top 
//
//      
//      opr_data[1]=0x1c;
//      Eprom_Write(0x08c0);       
//      
//     // Eprom_Write(0x05,0xc0);     //mag
//      
//     // flag9.all=0;
//     // WDTCTL = WDT_ARST_1000+WDTNMI+WDTNMIES;  
//            
//      rtc_fail_c=0;
//      mag_tpr_f=0;
//      //rev_tpr_f=0;
//			el_tpr_f=0;
//      neu_tpr_f=0;
//      tc_tpr_f=0;
//      //hv_fast_f=0;
//      //v_tpr_f=0;
//      flag13.all=0;
//      freq_tpr_f=0;
//      //flag14.all=0;
//      flag15.all &=0xf1;
//      persist_c=0;
////      neud_tpr_f=0;
////      neud_tprstr_f=0;
////      neud_tprrestr_f=0;
//     // flag15.all=0;
//      daily_noload_min=0;
//      no_supply_prd=0;
//      cum_tpr_c =0;
//      btpr_c=0;
//      mag_count=0;
//      daily_flag1=0;
//      rev_count=0;
//      tpc_count=0;
//      neu_count=0;
//      neud_count=0;
//      freq_count=0;
//      power_fail_cnt=0;
//      el_count=0;
//      //v_count=0;
//     
////      fill_oprzero();
////       // str_type1 = 10;
////        opr_data[1]=10;
////        //FUN_vfill_2byteR(str_type1 ,&opr_data[1]);
////        //restr_type1 = 10;    
////        opr_data[3]=10;
////        //FUN_vfill_2byteR(restr_type1 ,&opr_data[3]);
////       // str_type2 = 2100; 
////        opr_data[4]=0x08;
////        opr_data[5]=0x34;
////        //FUN_vfill_2byteR(str_type2 ,&opr_data[5]);
////        //restr_type2 = 180;   
////        opr_data[7]=180;
////        //FUN_vfill_2byteR(restr_type2 ,&opr_data[7]);        
////       // str_type3 = 120;
////        opr_data[8]=0x08;
////        opr_data[9]=0x34;
////        //FUN_vfill_2byteR(str_type3 ,&opr_data[9]); 
////        //restr_type3 = 180;
////        opr_data[11]=180;
////        //FUN_vfill_2byteR(str_type3 ,&opr_data[11]);
////
////        Eprom_Write(0x0850);      
//
//   //  WDTCTL = WDT_ARST_1000+WDTNMI+WDTNMIES;  
//      /***********calibration configuration, by default act_power****************/
//      //calib_ract_power_f=0;
//      //calib_act_power_f=1;    
//
//      /*******************************************/
//           
//    
//
//      break;
//
//    case 6:						//load survey
//      for(i=0x50;i<0xff;i++)   //Daily data reset
//      {
//        for(j=0;j<16;j++)
//          Eprom_Write(i*0x100+(j*16));
//        WDTCTL = WDT_ARST_1000+WDTNMI+WDTNMIES;              
//      }
////      Eprom_Write(0x08,0x00);
////      Eprom_Write(0x08,0x10);
//      
////      opr_data[6]=30;
////      Eprom_Write(0x08,0x20);  
//      
//     // opr_data[0]=0x00;
//      opr_data[1]=0x50;
//      Eprom_Write(0x0590);
//
//     // opr_data[0]=0x00;
//      opr_data[1]=0x60;
//      opr_data[2]=90;// default no of days LS
//      Eprom_Write(0x0580);      
//      
//      fg_done_f=1;
//      opr_data[0]=0x02;                //// use for set mag_tpr str/restr time.
//      time_stamp(&opr_data[1]);
//      
////      opr_data[6] = 'G';
////      opr_data[7] = 'E';
////      opr_data[8] = 'N';
////      opr_data[9] = 'U';
////      opr_data[10] = 'S';     
//      Eprom_Write(0x08a0);
//      
///**********Display Configure************/
//      default_disp();
//           
//      break;
//      
//      case 7:
//    
//        for(i=0x28;i<0x2b;i++)   
//        {
//          for(j=0;j<16;j++)
//            Eprom_Write(i*0x100+(j*16));
//          WDTCTL = WDT_ARST_1000+WDTNMI+WDTNMIES;                  
//        }
//        
//        opr_data[1]=0x28;
//        Eprom_Write(0x09c0); 
//     
//      break;
//
//    case 11:
//			    bill_resetmode=1;
//          save_billdata();
//					bill_resetmode=0;
////          if(event_log_f==1)
////          {
////           trans_id=19;
//           store_event(19);
//         // }
//          
//          break;
////    case 12:      
////      tct_cnt=0;
////      tc_tprrestr_f=1;
////      tc_tpr_f=0;
////	
////    /*  if(event_log_f==1)
////      {
////       trans_id=21;
////       store_event();
////      }
////      */
////  //    length_trans_pkt=1;
////    //   U0TXBUF=0x06;
////       break;
//       
//   // case 13:      
//     // hv_icon_f=0;
//    //  hv_tprrestr_f=1;
//    //  fill_oprzero();
//    //  Eprom_Write(0x07,0x40);     //neu
//	
//      // break;
//             
//       
//    default:
//              //length_trans_pkt=1;
//              serial_counter=0;
//              data_transmit_flag=1;
////              U0TXBUF=0x17;
//              default_tx_f=1;
//				
//  }
//  //length_trans_pkt=1;
//  if(default_tx_f==1)
//  {
//    default_tx_f=0;
//   transmit_byte(0x17);// U0TXBUF=0x17;
//  }
//  else
//    transmit_byte(0x06);//U0TXBUF=0x06;
//    
//}
//
//void config_meter_data(void)
//{
//  unsigned char i;
//  
//  switch(ID[1])
//  {
//    case 1:
////       if(event_log_f==1)
////      {
////       trans_id=1;
//       store_event(1);
//    //  }
//            
//      TI_second = receive_data_array1[2];
//      TI_minute = receive_data_array1[3];
//      TI_hour = receive_data_array1[4];
//      TI_year = receive_data_array1[5];
//      TI_month = receive_data_array1[6];
//      TI_day = receive_data_array1[7];
//      TI_dayOfWeek = receive_data_array1[8];
//      rtcfail_f=0;
//      //cum_kwh_ndpl = cum_kwh;
//      if((bcd_to_hex2(TI_year))%4 == 0) //|| TI_year == 0x12) //|| TI_year == 0x16 || TI_year == 0x20)
//        TI_FebDays=0x29;
//      else
//        TI_FebDays=0x28;
//      
//      if(receive_data_array1[11]==85)
//      {
//        rtc_cal_sign = receive_data_array1[9];
//        cal_second = receive_data_array1[10];
//        /*write in memory also*/
//        
//        opr_data[0]=rtc_cal_sign;
//        opr_data[1]=cal_second;        
//        Eprom_Write(0x06f0);
//        rtc_calib_f=0;
//      }
//      else
//      {  //rtc configure
//      /*        TI_second = receive_data_array1[2];
//              TI_minute = receive_data_array1[3];
//              TI_hour = receive_data_array1[4];
//              TI_year = receive_data_array1[5];
//              TI_month = receive_data_array1[6];
//              TI_day = receive_data_array1[7];
//              TI_dayOfWeek = receive_data_array1[8];
//    */    
//              BTCTL |=BT_ADLY_1000;
//              //rtcwr_f=1;
//              //RTC_Write();
//               // TI_second =rtc.seconds;         
//             //  time_stamp(&TI_minute);
//                /*TI_minute =rtc.min;         
//                TI_hour =rtc.hrs;           
//                TI_day =rtc.date;            
//                TI_month=rtc.month;           
//                TI_year =rtc.year;           
//                */
////              mdkw_c = 0;
////              mdkva_c = 0;
//							mdkw_c_import = 0;
//              mdkva_c_import = 0;
//              mdkw_c_export = 0;
//              mdkva_c_export = 0;              
//              kw_cnt_import=0;
//              //kva_cnt_import=0;
//              kw_cnt_export=0;
//              //kva_cnt_export=0;    
//              //miss_latenttable();
//        
//        //      delay_2();
//              //deter_season();
////              deter_zone();
////              if(zonecng_f0==1)
////                { 
////                  storezkwh();
////                  loadzkwh();
////                }
//              
//        //      U0TXBUF=0x06;
//      GetNextDate();        
//      }       
//      break;
//		
//    case 2:
//      for(i=0;i<8;i++)
//        *(opr_data + i) = receive_data_array1[2+i];
//                                                            //serial number configure
//      opr_data[8] = FW_id;
//      opr_data[9] = FW_rev;
//      opr_data[10] = 0x0d;
//      opr_data[11] = 0x10;
//      opr_data[12] = 0x42;   //version changed
//      opr_data[13] = receive_data_array1[10];//(0x37 & (receive_data_array1[10] | 0xfc));
//      opr_data[14] = receive_data_array1[11];
//      Eprom_Write(0x0500);
////      opr_data[15] = chksumsr(opr_data,15);
//
//        i=0;
//       do 
//       {
//         utility_id[i]=opr_data[i+4]; 
//         meter_id[i]=opr_data[i];  
//         i++;
//       }
//       while(i<4);      
//    //  serial_counter=0;
//  //    length_trans_pkt=1;
//    //  U0TXBUF=0x06;
//     
//      break;
//
//  case 3:
//    
//    md_ip=receive_data_array1[2];
//    
//  //  if(md_ip != 0x1e && md_ip != 0x0f )
//   //   md_ip=0x1e;
//    
//    opr_data[0]=md_ip;
//    Eprom_Write(0x06e0);
//    
////      if(event_log_f==1)
////      {
////       trans_id=2;
//       store_event(2);
//      //}
//    
//    break;
////  case 4:   
////        PI_ID=receive_data_array1[10];
////        Eprom_Read(0x0850);
//////        if(PI_ID==0x00)
//////        {
//////          opr_data[0]=receive_data_array1[2];
//////          opr_data[1]=receive_data_array1[3];
//////          str_type1 = char_array_to_int(&opr_data[0]); // value in seconds
//////          
//////          opr_data[2]=receive_data_array1[4];
//////          opr_data[3]=receive_data_array1[5];
//////          restr_type1 = char_array_to_int(&opr_data[2]); // value in seconds
//////          
//////          opr_data[4]=receive_data_array1[6];
//////          opr_data[5]=receive_data_array1[7];
//////          str_type2 = char_array_to_int(&opr_data[4]);   // value in seconds 
//////          
//////          opr_data[6]=receive_data_array1[8];
//////          opr_data[7]=receive_data_array1[9];
//////          restr_type2 = char_array_to_int(&opr_data[6]); 
//////           
//////        }
//////        else if(PI_ID==0x01)
//////        {
//////          opr_data[0]=receive_data_array1[2];
//////          opr_data[1]=receive_data_array1[3];
//////          str_type1 = char_array_to_int(&opr_data[0]); // value in seconds
//////        }
//////        else if(PI_ID==0x02)
//////        {
//////          opr_data[2]=receive_data_array1[4];
//////          opr_data[3]=receive_data_array1[5];          
//////          restr_type1 = char_array_to_int(&opr_data[2]); // value in seconds       
//////        } 
//////        else if(PI_ID==0x03)
//////        {
//////          opr_data[4]=receive_data_array1[6];
//////          opr_data[5]=receive_data_array1[7];
//////          str_type2 = char_array_to_int(&opr_data[4]);   // value in seconds 
//////        }
//////        else if(PI_ID==0x04)
//////        {
//////          opr_data[6]=receive_data_array1[8];
//////          opr_data[7]=receive_data_array1[9];
//////          restr_type2 = char_array_to_int(&opr_data[6]); 
//////        }  
////        //else 
////        if(PI_ID==0x05)
////        {
////          opr_data[0]=receive_data_array1[2];
////          opr_data[1]=receive_data_array1[3];
////          str_type1 = char_array_to_int(&opr_data[0]); // value in seconds
////        
////          opr_data[2]=receive_data_array1[4];
////          opr_data[3]=receive_data_array1[5];          
////          restr_type1 = char_array_to_int(&opr_data[2]); // value in seconds   
////        }         
////        else if(PI_ID==0x06)
////        {
////          opr_data[4]=receive_data_array1[6];
////          opr_data[5]=receive_data_array1[7];
////          str_type2 = char_array_to_int(&opr_data[4]);   // value in seconds 
////          opr_data[6]=receive_data_array1[8];
////          opr_data[7]=receive_data_array1[9];
////          restr_type2 = char_array_to_int(&opr_data[6]);           
////        } 
////       
////        else if(PI_ID==0x07)
////        {
////          opr_data[8]=receive_data_array1[11];
////          opr_data[9]=receive_data_array1[12];
////          str_type3 = char_array_to_int(&opr_data[8]);
////          if(str_type3<60)
////            str_type3=60;
////          opr_data[10]=receive_data_array1[13];
////          opr_data[11]=receive_data_array1[14];
////          restr_type3 = char_array_to_int(&opr_data[10]);              
////        }  
//////        else if(PI_ID==0x08)
//////        {
//////          opr_data[8]=receive_data_array1[11];
//////          opr_data[9]=receive_data_array1[12];
//////          str_type3 = char_array_to_int(&opr_data[8]);
//////        }
//////        else if(PI_ID==0x09)
//////        {
//////          opr_data[10]=receive_data_array1[13];
//////          opr_data[11]=receive_data_array1[14];
//////          restr_type3 = char_array_to_int(&opr_data[10]);              
//////        } 
////
////        Eprom_Write(0x0850);        
//////        if(event_log_f==1)
//////        {
//////         trans_id=3;
////         store_event(3);
////        //}        
////
////        break;        
//
//    
//    
//      
//  /*case 5:
//        scroll_time=receive_data_array1[2];
//        opr_data[0]=scroll_time;
//        Eprom_Write(0x08,0xf0);
//        break; */ 
//  case 9:
//        bill_month=receive_data_array1[2];
//        Eprom_Read(0x07d0);
//        opr_data[3]=bill_month;
//        Eprom_Write(0x07d0);
//        GetNextDate(); 
////        if(event_log_f==1)
////        {
////         trans_id=7;
//         store_event(7);
//       // }   
//        break;        
//    
//  case 10:
//        bill_date=receive_data_array1[2];
//        bill_hour = receive_data_array1[3];
//        bill_min = receive_data_array1[4];
//        Eprom_Read(0x07d0);
//        opr_data[0]=bill_date;
//        opr_data[1]=bill_hour;
//        opr_data[2]=bill_min;
//        Eprom_Write(0x07d0);
//        
//        GetNextDate();        
////        if(event_log_f==1)
////        {
////         trans_id=8;
//         store_event(8);
//        //}
//        
//        break;
//        
//  case 11:       
//    
//        for(i=0;i<15;i++)
//       *(opr_data + i) = receive_data_array1[2+i];
////        opr_data[15] = chksumsr(opr_data,15);
//        
//        if(receive_data_array1[1]==01)
//        {
//          Eprom_Write(0x0530);
//          if(event_log_f==1)
//          {
//           store_event(9);
//          }
//        }
//        else if(receive_data_array1[1]==02)
//           Eprom_Write(0x0510);
//   
//    
//    /*
//        fill_tarifftable(0x20);
//        fill_tarifftable(0x50);
//        fill_tarifftable(0x80);
//        fill_tarifftable(0xb0);
//        
//        WDTCTL = WDT_ARST_1000+WDTNMI+WDTNMIES;              
//
//        fill_oprzero();
//        opr_data[1]=TI_day;
//        opr_data[2]=TI_month;
//        opr_data[3]=TI_year;               
//        opr_data[4]=1; 
//        Eprom_Write(0x08,0xe0);
//        */
//        break;
//        
//        
//  case 13:
////        LED_config=receive_data_array1[2];
////        Eprom_Read(0x04,0xd0);
////        opr_data[0]=LED_config;
////        Eprom_Write(0x04,0xd0);
////        if(event_log_f==1)
////        {
////         trans_id=11;
////         store_event();
////        }        
//        break;  
//        
//  case 16:  
//       for(i=0;i<15;i++)
//       *(opr_data + i) = receive_data_array1[2+i];
//       
//        if(receive_data_array1[1]==32)   
//        {
//          Eprom_Write(0x05a0);
//          para_nos_auto=opr_data[5];
//          para_nos_push=opr_data[6];
//        }        
//        else if(receive_data_array1[1]==31)     // AUTO MODE   
//             Eprom_Write(0x4500);
//        else if(receive_data_array1[1]==30)         
//             Eprom_Write(0x4510);
//        else if(receive_data_array1[1]==29)         
//             Eprom_Write(0x4520);
///*        else if(receive_data_array1[1]==28)         
//             Eprom_Write(0x51,0x30);
//        else if(receive_data_array1[1]==27)         
//             Eprom_Write(0x51,0x40);
//        else if(receive_data_array1[1]==26)         
//             Eprom_Write(0x51,0x50);
//        else if(receive_data_array1[1]==25)         
//             Eprom_Write(0x51,0x60);
//        else if(receive_data_array1[1]==24)         
//             Eprom_Write(0x51,0x70);
//        else if(receive_data_array1[1]==23)         
//             Eprom_Write(0x51,0x80);
//        else if(receive_data_array1[1]==22)         
//             Eprom_Write(0x51,0x90);
//        else if(receive_data_array1[1]==21)         
//             Eprom_Write(0x51,0xa0);
//        else if(receive_data_array1[1]==20)         
//             Eprom_Write(0x51,0xb0);
//        else if(receive_data_array1[1]==19)         
//             Eprom_Write(0x51,0xc0);
//        else if(receive_data_array1[1]==18)         
//             Eprom_Write(0x51,0xb0);
//        else if(receive_data_array1[1]==17)         
//             Eprom_Write(0x51,0xe0);
//        else if(receive_data_array1[1]==16)         
//             Eprom_Write(0x51,0xf0);*/
//        else if(receive_data_array1[1]==15)    // PUSH MODE
//             Eprom_Write(0x4540);           
//        else if(receive_data_array1[1]==14)   
//             Eprom_Write(0x4550);           
//        else if(receive_data_array1[1]==13)   
//             {
//               Eprom_Write(0x4560);       
////               if(event_log_f==1)
////               {
////                 trans_id=20;
//                 store_event(20);
//               //}        
//             }
//     /*   else if(receive_data_array1[1]==12)   
//             Eprom_Write(0x52,0x30);           
//        else if(receive_data_array1[1]==11)   
//             Eprom_Write(0x52,0x40);           
//        else if(receive_data_array1[1]==10)   
//             Eprom_Write(0x52,0x50);           
//        else if(receive_data_array1[1]==9)   
//             Eprom_Write(0x52,0x60);           
//        else if(receive_data_array1[1]==8)   
//             Eprom_Write(0x52,0x70);           
//        else if(receive_data_array1[1]==7)   
//             Eprom_Write(0x52,0x80);           
//        else if(receive_data_array1[1]==6)   
//             Eprom_Write(0x52,0x90);           
//        else if(receive_data_array1[1]==5)   
//             Eprom_Write(0x52,0xa0);           
//        else if(receive_data_array1[1]==4)   
//             Eprom_Write(0x52,0xb0);           
//        else if(receive_data_array1[1]==3)   
//             Eprom_Write(0x52,0xc0);           
//        else if(receive_data_array1[1]==2)   
//             Eprom_Write(0x52,0xd0);           
//        else if(receive_data_array1[1]==1)   
//             Eprom_Write(0x52,0xe0);           
//        else if(receive_data_array1[1]==0)   
//        {
//          Eprom_Write(0x52,0xf0);           
//          if(event_log_f==1)
//          {
//           trans_id=20;
//           store_event();
//          }
//        }                        */
//        bat_disp_c=0;
//        switch_disp_f4=0;
//        disp_cntr=0;
//         break;
//        
//  case 34:                                                  // config md display;
//      md_config = receive_data_array1[2];
//     // Eprom_Read(0x04,0xd0);
//      opr_data[1] = md_config;
//      Eprom_Write(0x04d0);
////        if(event_log_f==1)
////        {
////         trans_id=15;
//         store_event(15);
//       // }      
//      //length_trans_pkt=1;
////      U0TXBUF=0x06;
//      break;        
//
//        
//    case 36:                                                  // password configure
//        /*opr_data[0]=receive_data_array1[2];
//        if ( receive_data_array1[2]==0xc0)
//           test_mode_f=1;
//        else if(receive_data_array1[2]==0x80) 
//            test_mode_f=0;
//        else
//        {
//          test_mode_f=0;
//          opr_data[0]=0x80;        
//        }
//        Eprom_Write(0x08,0x30);
//        charge_c=0;
//        discharge_c=0;
//        
//        opr_data[0]=1;
//        Eprom_Write(0x08,0x40);
//        bat_discharge_f=0;*/
//        
//       // tamper_reset();
//        
//        
//      break;
//      
////    case 38:
////      for(i=0;i<15;i++)
////        *(opr_data + i) = receive_data_array1[2+i];                                                          //serial number configure 
////         Eprom_Write(0x0870);
////         break;      
//      
//    case 42:                  
//         for(i=0;i<4;i++)
//          *(opr_data + i) = receive_data_array1[2+i]; // PCB Serial No Configuration
//         Eprom_Write(0x0890);
//  
//         break;        
//    case 43:      
//         Eprom_Read(0x08a0);
//         for(i=1;i<6;i++)
//          *(opr_data + i) = receive_data_array1[1+i]; // Meter Serial No Configuration
//         Eprom_Write(0x08a0);
//         break;   
//         
//    case 47:      
//         Eprom_Read(0x0580);// no of LS Days
//         opr_data[2] = receive_data_array1[2];
//         Eprom_Write(0x0580);
//         break;  
//         
////    case 48:      
////         Eprom_Read(0x08,0x20);
////         opr_data[6] = receive_data_array1[2];
////         Eprom_Write(0x08,0x20);
////         break;  
//         
//    /*
//  case 48:  //tod cinfigure(for default and SEASON 1 TOD)
//      fill_tarifftable(0x90);
//      
//        //length_trans_pkt=1;      
//        //U0TXBUF=0x06;
//       break;
//  case 49:  //tod cinfigure(for season2 TOD)
//            fill_tarifftable(0xc0);
//
//      
//        //length_trans_pkt=1;      
//        //U0TXBUF=0x06;
//       break;*/
//       
///*  case 50:                            //Latent season1
//             fill_tarifftable(0x80);
//              break;
//        
//  case 51:                           //Latent season2     
//             fill_tarifftable(0xb0);
//              break;
//
//  case 52:                            //date of application
//              for(i=0;i<15;i++)
//       *(opr_data + i) = receive_data_array1[2+i];
//              Eprom_Write(0x08,0xe0);
//              break;
//    
//  case 16:  
//              for(i=0;i<15;i++)
//       *(opr_data + i) = receive_data_array1[2+i];
//        if(receive_data_array1[1]==01)
//        {
//          Eprom_Write(0x06,0x00);
//          para_nos=opr_data[0];
//          //for(i=0;i<14;i++)
//          //  push_disp_array[i]=opr_data[i+1];
//          para_nos--;
//        }
//        else if(receive_data_array1[1]==00)
//        {
//           Eprom_Write(0x06,0x10);
//          // for(i=14;i<26;i++)
//          //  push_disp_array[i]=opr_data[i-14];
//        }
//        bat_disp_c=0;
//        disp_nconfig_f=0;
//        switch_disp_f4=0;
//         break;
//
//      */
//      
//    case 17:                                                  // password configure
//      change_pw_f=1;
//      decrypt_password();
//      for(i=0;i<8;i++)
//        *(opr_data + i)=password[i];
//
////      opr_data[15] = chksumsr(opr_data,15);
//
//      Eprom_Write(0x0520);
//      //length_trans_pkt=1;
////      U0TXBUF=0x06;
//      break;
//      
//      
//      
//       default:
//  //            length_trans_pkt=1;
//             // serial_counter=0;
//             // data_transmit_flag=1;
//    //          U0TXBUF=0x17;
//              default_tx_f=1;
//
//		
//  }
//  //length_trans_pkt=1;
//  if(default_tx_f==1)
//  {
//    default_tx_f=0;
//    transmit_byte(0x17);//U0TXBUF=0x06;
//  }
//  else
//    transmit_byte(0x06);//U0TXBUF=0x06;
//}
//
//
//void store_event(unsigned char id)
//{ 
//  if(event_log_f==0)
//    return;
//  Eprom_Read(0x09c0);
//  if(chksum_ok==1)
//  {
//      mepromblk=opr_data[1];
//      mepromloc=opr_data[0];
//  
//  if(mepromblk > 0x2a )         //for 24 events
//    {
//        mepromblk=0x28;
//        mepromloc=0x00;
//    }
//  }
//  else
//   {
//      mepromblk=0x28;
//      mepromloc=0x00;
//   }
//    if(mepromblk==0)
//      mepromblk=0x28;
//  
//  opr_data[0]=id;
//  time_stamp_withsec(&opr_data[1]);
////    FUN_vfill_3byteR(cum_kwh,&opr_data[8]);
////
////    FUN_vfill_2byteR(v_rms,&opr_data[11]);
////    if(ph_ct_f1==0)
////      FUN_vfill_3byteR(ip_rms,&opr_data[14]);
////    else
////      FUN_vfill_3byteR(in_rms,&opr_data[14]);  
//  Eprom_Write(mepromblk*0x100+mepromloc);
//  
//  opr_data[0]=mepromloc+0x10;
//  
//  if(opr_data[0]==0x00)
//     opr_data[1]=mepromblk+1;
//  else
//     opr_data[1]=mepromblk;
//   Eprom_Write(0x09c0);  
//
//
//}


/*
void calib_meter(void)
{
 switch(ID[1])
  {
    case 1:
      char_array_to_int(&receive_data_array1[2]);
      v_cal_coff = (int)long_int;
      if(receive_data_array1[2] == 's')
      {
        Eprom_Read(0x0a,0x40);
        opr_data[0] = receive_data_array1[2];
        opr_data[1] = receive_data_array1[3];
        opr_data[15] = chksumsr(opr_data,15);
        Eprom_Write(0x0a,0x40);
      }
      
      U0TXBUF=0x06;
      break;
		
    case 2:
      char_array_to_int(&receive_data_array1[2]);
      //i1_cal_coff = (int)long_int;
      if(receive_data_array1[2] == 's')
      {
        Eprom_Read(0x0a,0x40);
        opr_data[2] = receive_data_array1[2];
        opr_data[3] = receive_data_array1[3];
        opr_data[15] = chksumsr(opr_data,15);
        Eprom_Write(0x0a,0x40);
      }
      
      U0TXBUF=0x06;
      break;

    case 3:
      char_array_to_int(&receive_data_array1[2]);
      //i2_cal_coff = (int)long_int;
      if(receive_data_array1[2] == 's')
      {
        Eprom_Read(0x0a,0x40);
        opr_data[4] = receive_data_array1[2];
        opr_data[5] = receive_data_array1[3];
        opr_data[15] = chksumsr(opr_data,15);
        Eprom_Write(0x0a,0x40);
      }
      
      U0TXBUF=0x06;
      break;

    case 4:
      char_array_to_int(&receive_data_array1[2]);
      kw_cal = (int)long_int;
      if(receive_data_array1[2] == 's')
      {
        Eprom_Read(0x0a,0x40);
        opr_data[6] = receive_data_array1[2];
        opr_data[7] = receive_data_array1[3];
        opr_data[15] = chksumsr(opr_data,15);
        Eprom_Write(0x0a,0x40);
      }
      
      U0TXBUF=0x06;
      break;

    case 5:						//load survey
      char_array_to_int(&receive_data_array1[2]);
      kw_el_cal = (int)long_int;
      if(receive_data_array1[2] == 's')
      {
        Eprom_Read(0x0a,0x40);
        opr_data[8] = receive_data_array1[2];
        opr_data[9] = receive_data_array1[3];
        opr_data[15] = chksumsr(opr_data,15);
        Eprom_Write(0x0a,0x40);
      }
      
      U0TXBUF=0x06;
      break;
	
  }
}*/
unsigned int char_array_to_int(unsigned char *char_array_ptr)
{
  unsigned char dumy1;
unsigned int long_int=0; 
    //long_int=0;
  
//  long_int = 0;
  long_int = *(char_array_ptr);
  long_int = long_int << 8;
  dumy1 = *(char_array_ptr+1);
  long_int = long_int | dumy1;
  return(long_int);
}

