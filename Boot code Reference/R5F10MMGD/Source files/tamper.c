/***********************************************************************************************************************
* File Name    : tamper.c
* Version      : 
* Device(s)    : R5F10MMG
* Tool-Chain   : CC-RL
* Description  : This file implements device driver for CGC module.
* Creation Date: 
***********************************************************************************************************************/


/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "r_cg_macrodriver.h"
#include "variable.h"
#include "tamper.h"
#include "bill.h"
#include "function.h"
#include "Eprom_i2c.h"
#include "r_cg_wdt.h"

/***********************************************************************************************************************
Pragma directive
***********************************************************************************************************************/
#pragma interrupt LPM_PORT42_Interrupt(vect = INTP7)

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/

/***********************************************************************************************************************
* Function Name: fill_oprzero
* Description  : This function initializes the clock generator.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void check_tpr(void)
{
	/***************voltage related event***************/
		if(TP.b.VoltageRelatedEvent == 1)  
		{
			TP.b.VoltageRelatedEvent=0;
			save_cumtpr();  
//			if(TP3.b.neu_miss_tprstr_f==1)
//			{  
//				TP3.b.neu_miss_tprstr_f=0; 
//	            TPRCNT_u8NeuMiss+=1;  
//				event_id=13;          
//				volt_of=TPR_vSnapshotStore(volt_tamp_status,volt_init_add,volt_count,3,volt_max_loc,1);
//				//read location addr , compart init addr , count of compart , location of flag , max compartment size , 1-store 0-restore
//	            TPR_vSaveFirstStrTpr();
//	            TPR_vSaveLastStrTpr();
//			}
			if(TP7.b.highv_tprstr_f==1) 
			{ 
				TP7.b.highv_tprstr_f=0; 
	            TPRCNT_u8VHigh+=1;  
				event_id=7;          
				volt_of=TPR_vSnapshotStore(volt_tamp_status,volt_init_add,volt_count,4,volt_max_loc,1);
				//read location addr , compart init addr , count of compart , location of flag , max compartment size , 1-store 0-restore
	            TPR_vSaveFirstStrTpr();
	            TPR_vSaveLastStrTpr();
			}
//			if(TP7.b.LowVolt_tprstr_f==1)
//			{ 
//				TP7.b.LowVolt_tprstr_f=0;  
//	            TPRCNT_u8VLow    +=1;
//				event_id=9;          
//				volt_of=TPR_vSnapshotStore(volt_tamp_status,volt_init_add,volt_count,5,volt_max_loc,1);
//				//read location addr , compart init addr , count of compart , location of flag , max compartment size , 1-store 0-restore
//	            TPR_vSaveFirstStrTpr();
//	            TPR_vSaveLastStrTpr();
//			}
			
//			if(TP3.b.neu_miss_tprrestr_f==1)
//			{
//				TP3.b.neu_miss_tprrestr_f=0;
//				event_id=14;          
//				volt_of=TPR_vSnapshotStore(volt_tamp_status,volt_init_add,volt_count,3,volt_max_loc,0);
//				//read location addr , compart init addr , count of compart , location of flag , max compartment size , 1-store 0-restore
//	            TPR_vSaveFirstRestrTpr();
//	            TPR_vSaveLastRestrTpr();
//			}  
			if(TP7.b.highv_tprrestr_f==1) 
			{   
				TP7.b.highv_tprrestr_f=0;   
				event_id=8;          
				volt_of=TPR_vSnapshotStore(volt_tamp_status,volt_init_add,volt_count,4,volt_max_loc,0);
				//read location addr , compart init addr , count of compart , location of flag , max compartment size , 1-store 0-restore
	            TPR_vSaveFirstRestrTpr();
	            TPR_vSaveLastRestrTpr();
	            
			}
			
//			if(TP7.b.LowVolt_tprrestr_f==1)
//			{ 
//				TP7.b.LowVolt_tprrestr_f=0;   
//				event_id=10;          
//				volt_of=TPR_vSnapshotStore(volt_tamp_status,volt_init_add,volt_count,5,volt_max_loc,0);
//				//read location addr , compart init addr , count of compart , location of flag , max compartment size , 1-store 0-restore
//	            TPR_vSaveFirstRestrTpr();
//	            TPR_vSaveLastRestrTpr();
//	            
//			}
		}

	/***************current related event***************/
	if(TP.b.CurrentRelatedEvent==1)
	{
		TP.b.CurrentRelatedEvent=0;
		save_cumtpr();        
		if(TP1.b.rev_tprstr_f==1)
		{
			TP1.b.rev_tprstr_f=0;   
			TPRCNT_u8Rev     +=1;
			event_id=51;          
			curr_of=TPR_vSnapshotStore(curr_tamp_status,curr_init_add,curr_count,3,curr_max_loc,1);
			//read location addr , compart init addr , count of compart , location of flag , max compartment size , 1-store 0-restore
			TPR_vSaveFirstStrTpr();
			TPR_vSaveLastStrTpr();
		}
		if(TP2.b.el_tprstr_f==1)
		{
			TP2.b.el_tprstr_f=0;
			TPRCNT_u8EL     +=1;
			event_id=69;          
			curr_of=TPR_vSnapshotStore(curr_tamp_status,curr_init_add,curr_count,4,curr_max_loc,1);
			//read location addr , compart init addr , count of compart , location of flag , max compartment size , 1-store 0-restore
			TPR_vSaveFirstStrTpr();
			TPR_vSaveLastStrTpr();
		}
		if(TP2.b.oc_tprstr_f==1)
		{
			TP2.b.oc_tprstr_f=0;
			TPRCNT_u8OC     +=1;
			event_id=67;   
			curr_of=TPR_vSnapshotStore(curr_tamp_status,curr_init_add,curr_count,6,curr_max_loc,1);
			//read location addr , compart init addr , count of compart , location of flag , max compartment size , 1-store 0-restore
			TPR_vSaveFirstStrTpr();
			TPR_vSaveLastStrTpr();
		}
		if(TP2.b.ol_tprstr_f==1)
		{
			TP2.b.ol_tprstr_f=0;
			TPRCNT_u8OverLoad+=1;
			event_id=71;          
			curr_of=TPR_vSnapshotStore(curr_tamp_status,curr_init_add,curr_count,5,curr_max_loc,1);
			//read location addr , compart init addr , count of compart , location of flag , max compartment size , 1-store 0-restore
			TPR_vSaveFirstStrTpr();
			TPR_vSaveLastStrTpr();
		}
		if(TP1.b.rev_tprrestr_f==1)
		{
			TP1.b.rev_tprrestr_f=0;
			event_id=52;          
			curr_of=TPR_vSnapshotStore(curr_tamp_status,curr_init_add,curr_count,3,curr_max_loc,0);
			//read location addr , compart init addr , count of compart , location of flag , max compartment size , 1-store 0-restore
			TPR_vSaveFirstRestrTpr();
			TPR_vSaveLastRestrTpr();
			
		}
		if(TP2.b.el_tprrestr_f==1)
		{
			TP2.b.el_tprrestr_f=0;         
			event_id=70;          
			curr_of=TPR_vSnapshotStore(curr_tamp_status,curr_init_add,curr_count,4,curr_max_loc,0);
			//read location addr , compart init addr , count of compart , location of flag , max compartment size , 1-store 0-restore
			TPR_vSaveFirstRestrTpr();
			TPR_vSaveLastRestrTpr();            
		}
		
		if(TP2.b.oc_tprrestr_f==1)
		{
			TP2.b.oc_tprrestr_f=0;
			event_id=68;          
			curr_of=TPR_vSnapshotStore(curr_tamp_status,curr_init_add,curr_count,6,curr_max_loc,0);
			//read location addr , compart init addr , count of compart , location of flag , max compartment size , 1-store 0-restore
			TPR_vSaveFirstRestrTpr();
			TPR_vSaveLastRestrTpr();
			
		}
		
		if(TP2.b.ol_tprrestr_f==1)
		{
			TP2.b.ol_tprrestr_f=0;
			event_id=72;
			curr_of=TPR_vSnapshotStore(curr_tamp_status,curr_init_add,curr_count,5,curr_max_loc,0);
			//read location addr , compart init addr , count of compart , location of flag , max compartment size , 1-store 0-restore
			TPR_vSaveFirstRestrTpr();
			TPR_vSaveLastRestrTpr();
			
		}
	}     
	
	/***************transaction related event***************/
	if(TP.b.TransactionEvent==1)
	{
		TP.b.TransactionEvent=0;  
		trans_of=TPR_vSnapshotStore(trans_tamp_status,trans_init_add,trans_count,3,trans_max_loc,1);
		//read location addr , compart init addr , count of compart , location of flag , max compartment size , 1-store 0-restore
	}
	/***************other event***************/  
	if(TP.b.OtherEvent==1)
	{
		TP.b.OtherEvent=0;
		save_cumtpr();       
		if(TP1.b.mag_tprstr_f==1)
		{
			TP1.b.mag_tprstr_f=0;
			TPRCNT_u8Mag     +=1;
			event_id=201;   
			others_of=TPR_vSnapshotStore(other_tamp_status,other_init_add,others_count,3,other_max_loc,1);
			//read location addr , compart init addr , count of compart , location of flag , max compartment size , 1-store 0-restore
			TPR_vSaveFirstStrTpr();
			TPR_vSaveLastStrTpr();
		}
		
		if (TP3.b.neu_dis_tprstr_f==1)       
		{
			TP3.b.neu_dis_tprstr_f=0;
			TPRCNT_u8NeuDis  +=1;
			event_id=203;
			others_of=TPR_vSnapshotStore(other_tamp_status,other_init_add,others_count,4,other_max_loc,1);
			//read location addr , compart init addr , count of compart , location of flag , max compartment size , 1-store 0-restore
			TPR_vSaveFirstStrTpr();
			TPR_vSaveLastStrTpr();
		}
		if (TP5.b.kv35_tprstr_f==1)       
		{
			TP5.b.kv35_tprstr_f=0;
			TPRCNT_u835kv     +=1;
			event_id=213;
			others_of=TPR_vSnapshotStore(other_tamp_status,other_init_add,others_count,5,other_max_loc,1);
			//read location addr , compart init addr , count of compart , location of flag , max compartment size , 1-store 0-restore
			TPR_vSaveFirstStrTpr();
			TPR_vSaveLastStrTpr();
		}
		if (TP4.b.LowPF_tprstr_f==1)       
		{
			TP4.b.LowPF_tprstr_f=0;
			TPRCNT_u8LowPF     +=1;
			event_id=205;
			others_of=TPR_vSnapshotStore(other_tamp_status,other_init_add,others_count,6,other_max_loc,1);
			//read location addr , compart init addr , count of compart , location of flag , max compartment size , 1-store 0-restore
			TPR_vSaveFirstStrTpr();
			TPR_vSaveLastStrTpr();
		}
		if (TP4.b.AbFreq_tprstr_f==1)       
		{
			TP4.b.AbFreq_tprstr_f=0;
			TPRCNT_u8FreqTamp     +=1;
			event_id=211;
			others_of=TPR_vSnapshotStore(other_tamp_status,other_init_add,others_count,7,other_max_loc,1);
			//read location addr , compart init addr , count of compart , location of flag , max compartment size , 1-store 0-restore
			TPR_vSaveFirstStrTpr();
			TPR_vSaveLastStrTpr();
			
		}
		
		if(TP3.b.neu_miss_tprstr_f==1)
		{  
			TP3.b.neu_miss_tprstr_f=0; 
            TPRCNT_u8NeuMiss+=1;  
			event_id=207;          
			others_of=TPR_vSnapshotStore(other_tamp_status,other_init_add,others_count,8,other_max_loc,1);
			//read location addr , compart init addr , count of compart , location of flag , max compartment size , 1-store 0-restore
            TPR_vSaveFirstStrTpr();
            TPR_vSaveLastStrTpr();
		}
		
		if(TP1.b.mag_tprrestr_f==1)
		{
			TP1.b.mag_tprrestr_f=0;
			event_id=202; 
			others_of=TPR_vSnapshotStore(other_tamp_status,other_init_add,others_count,3,other_max_loc,0);
			//read location addr , compart init addr , count of compart , location of flag , max compartment size , 1-store 0-restore
			TPR_vSaveFirstRestrTpr();
			TPR_vSaveLastRestrTpr();
			
		}
		if(TP3.b.neu_dis_tprrestr_f==1)
		{
			TP3.b.neu_dis_tprrestr_f=0;
			event_id=204; 
			others_of=TPR_vSnapshotStore(other_tamp_status,other_init_add,others_count,4,other_max_loc,0);
			//read location addr , compart init addr , count of compart , location of flag , max compartment size , 1-store 0-restore
			TPR_vSaveFirstRestrTpr();
			TPR_vSaveLastRestrTpr();
			
		}
			
		if(TP5.b.kv35_tprrestr_f==1)
		{
			TP5.b.kv35_tprrestr_f=0;
			event_id=214; 
			others_of=TPR_vSnapshotStore(other_tamp_status,other_init_add,others_count,5,other_max_loc,0);
			//read location addr , compart init addr , count of compart , location of flag , max compartment size , 1-store 0-restore
			TPR_vSaveFirstRestrTpr();
			TPR_vSaveLastRestrTpr();
			
		}
		if(TP4.b.LowPF_tprrestr_f==1)
		{
			TP4.b.LowPF_tprrestr_f=0;
			event_id=206; 
			others_of=TPR_vSnapshotStore(other_tamp_status,other_init_add,others_count,6,other_max_loc,0);
			//read location addr , compart init addr , count of compart , location of flag , max compartment size , 1-store 0-restore
			TPR_vSaveFirstRestrTpr();
			TPR_vSaveLastRestrTpr();
			
		}
		if(TP4.b.AbFreq_tprrestr_f==1)
		{
			TP4.b.AbFreq_tprrestr_f=0;
			event_id=212; 
			others_of=TPR_vSnapshotStore(other_tamp_status,other_init_add,others_count,7,other_max_loc,0);
			//read location addr , compart init addr , count of compart , location of flag , max compartment size , 1-store 0-restore
			TPR_vSaveFirstRestrTpr();
			TPR_vSaveLastRestrTpr();
			
		}
		
		if(TP3.b.neu_miss_tprrestr_f==1)
		{
			TP3.b.neu_miss_tprrestr_f=0;
			event_id=208;          
			others_of=TPR_vSnapshotStore(other_tamp_status,other_init_add,others_count,8,other_max_loc,0);
			//read location addr , compart init addr , count of compart , location of flag , max compartment size , 1-store 0-restore
            TPR_vSaveFirstRestrTpr();
            TPR_vSaveLastRestrTpr();
		}
 		
	}
	//	
	//	/***************non rollover event***************/
	if(TP.b.NonRollEvent)
	{
		TP.b.NonRollEvent=0;
		if(TP5.b.tc_tprstr_f==1)
		{ 
			TP5.b.tc_tprstr_f=0; 
			TPRCNT_u8TC      +=1;
			save_cumtpr();
			event_id=251;
			nonroll_of=TPR_vSnapshotStore(nonroll_status,nonroll_init_add,nonroll_count,3,nonroll_max_loc,1);
			//read location addr , compart init addr , count of compart , location of flag , max compartment size , 1-store 0-restore
			TPR_vSaveFirstStrTpr();
			TPR_vSaveLastStrTpr();
			
		}
		//		if(rlymal_tprstr_f == 1)
		//		{
		//			rlymal_tprstr_f=0;     
		//			a8Daily_alert_status[8]|=DES_rly_mal;
		////			man_a8Alert_array[4]|=MAA_rly_mal;
		//			event_id=253 ;      
		//			nonroll_of=TPR_vSnapshotStore(nonroll_status,nonroll_init_add,nonroll_count,4,nonroll_max_loc,1);
		//			//read location addr , compart init addr , count of compart , location of flag , max compartment size , 1-store 0-restore
		////			PUS_vAlertBuffUpdate();
		//		}  
	}	
	//	/***************non rollover event***************/
	//	
	//    //	if(TPR_bcontrolEvent == 1)
	//    //	{
	//    //		TPR_bcontrolEvent=0;
	//    //		if(event_id==301)
	//    //		{
	//    ////			a8Daily_alert_status[9]|=DES_remote_disO;   
	//    ////			man_a8Alert_array[4]|=MAA_remote_dis;
	//    ////			man_a8Alert_array[0]&=~MAA_remote_dis;
	//    //		}
	//    //		else if(event_id==302)
	//    //		{
	//    //			a8Daily_alert_status[9]|=DES_remote_disR;
	//    ////			man_a8Alert_array[0]|=MAA_remote_dis;
	//    ////			man_a8Alert_array[4]&=~MAA_remote_dis;
	//    //		}
	//    //		if(event_id==303)
	//    //		{
	//    //			a8Daily_alert_status[9]|=DES_lock_outO; 
	//    ////			PUS_vAlertBuffUpdate();
	//    ////			man_a8Alert_array[4]|=MAA_lock_out;
	//    ////			man_a8Alert_array[0]&=~MAA_lock_out;
	//    //		}
	//    //		else if(event_id==304)
	//    //		{
	//    //			a8Daily_alert_status[9]|=DES_lock_outR; 
	//    ////			man_a8Alert_array[0]|=MAA_lock_out;
	//    ////			man_a8Alert_array[4]&=~MAA_lock_out;
	//    //		}
	//    //		
	//    //		event_id-=300;
	//    //		control_of=TPR_vSnapshotStore(control_tamp_status,control_init_add,control_count,4,control_max_loc,1);
	//    //		//read location addr , compart init addr , count of compart , location of flag , max compartment size , 1-store 0-restore
	//    //		
	//    ////		if(event_id==3)
	//    ////			PUS_vAlertBuffUpdate();		
	//    //	}

//	/***************DiagnosticsEvent***************/
//		if(TPR_bDiagnosticsEvent)
//		{
//			TPR_bDiagnosticsEvent=0;
//			if(rtcfail_tprstr_f==1)
//			{ 
//				rtcfail_tprstr_f=0; 
//	            //			a8Daily_alert_status[11]|=DES_rtc_fail; 
//	            //			save_cumtpr();
//	            //			man_a8Alert_array[6]|=MAA_tc_tpr;
//				event_id=51;
//				Diagnostics_of=TPR_vSnapshotStore(Diagnostics_status,Diagnostics_init_add,Diagnostics_count,3,Diagnostics_max_loc,1);
//				//read location addr , compart init addr , count of compart , location of flag , max compartment size , 1-store 0-restore
//	            //			PUS_vAlertBuffUpdate();
//			}
//			if(MemFail_tprstr_f == 1)
//			{
//				MemFail_tprstr_f=0;  
//	            //			a8Daily_alert_status[11]|=DES_mem_fail;
//	            //			man_a8Alert_array[4]|=MAA_rly_mal;
//				event_id=52 ;      
//				Diagnostics_of=TPR_vSnapshotStore(Diagnostics_status,Diagnostics_init_add,Diagnostics_count,4,Diagnostics_max_loc,1);
//				//read location addr , compart init addr , count of compart , location of flag , max compartment size , 1-store 0-restore
//	            //			PUS_vAlertBuffUpdate();
//			}  
//			if(bat_discharge_tprstr_f == 1)
//			{
//	            //			a8Daily_alert_status[11]|=DES_rtc_bat_lo;
//				bat_discharge_tprstr_f=0;     
//	            //			u16Daily_alert_status|=DAS_rly_mal;
//	            //			man_a8Alert_array[4]|=MAA_rly_mal;
//				event_id=53 ;      
//				Diagnostics_of=TPR_vSnapshotStore(Diagnostics_status,Diagnostics_init_add,Diagnostics_count,5,Diagnostics_max_loc,1);
//				//read location addr , compart init addr , count of compart , location of flag , max compartment size , 1-store 0-restore
//	            //			PUS_vAlertBuffUpdate();
//			}  
//		}	
	//	/***************DiagnosticsEvent***************/
	//    //	memcpy(opr_data,a8Daily_alert_status,12);
	//    //	Eprom_Write(0x08f0);
	    opr_data[0] = TPRCNT_u8NeuMiss ;
	    opr_data[1] = TPRCNT_u8VHigh   ;
	    opr_data[2] = TPRCNT_u8VLow    ;
	    opr_data[3] = TPRCNT_u8Rev     ;
	    opr_data[4] = TPRCNT_u8EL      ;
	    opr_data[5] = TPRCNT_u8OC      ;
	    opr_data[6] = TPRCNT_u8OverLoad;
	    opr_data[7] = TPRCNT_u8Mag     ;
	    opr_data[8] = TPRCNT_u8NeuDis  ;
	    opr_data[9]= TPRCNT_u8LowPF   ;
	    opr_data[10] = TPRCNT_u835kv    ;
	    opr_data[11]= TPRCNT_u8FreqTamp;
	    opr_data[12]= TPRCNT_u8TC      ;
	    Eprom_Write(TPRCNT_Addr);
	
}
void TPR_vInitRam(void)
{
	//	u16 u16Tempp;
	//	u16 a16tempp[7];
	Eprom_Read(TamperSelByteAdd);
	if(EPROM_bChksum_ok == 1)
	{
		TamperSelByte=a8_to_u32(&opr_data[0]);
	}
	if(TamperSelByte==0)
	{
		TamperSelByte=0x000007ff;
	}
	Eprom_Read(ThresholdAdd);
	if(EPROM_bChksum_ok == 1)
	{
		tpr_u16NeuDStrTime=a8_to_u16(&opr_data[0]);
		tpr_u16NeuDRestrTime=a8_to_u16(&opr_data[2]);
		tpr_u16NeuD_Thrshld=a8_to_u16(&opr_data[4]);
		tpr_u16NeuD_ThrshldRest=a8_to_u16(&opr_data[6]);
		
		tpr_u16OverVStrTime=a8_to_u16(&opr_data[8]);
		tpr_u16OverVRestrTime=a8_to_u16(&opr_data[10]);
		tpr_u16OverV_Thrshld=a8_to_u16(&opr_data[12]);
	}
	Eprom_Read(ThresholdAdd+0x10);
	if(EPROM_bChksum_ok == 1)
	{
		tpr_u16OverV_ThrshldRest=a8_to_u16(&opr_data[0]);
		
		tpr_u16LowVStrTime=a8_to_u16(&opr_data[2]);
		tpr_u16LowVRestrTime=a8_to_u16(&opr_data[4]);
		tpr_u16LowV_Thrshld=a8_to_u16(&opr_data[6]);
		tpr_u16LowV_ThrshldRest=a8_to_u16(&opr_data[8]);
		
		tpr_u16RevStrTime=a8_to_u16(&opr_data[10]);
		tpr_u16RevRestrTime=a8_to_u16(&opr_data[12]);
	}
	Eprom_Read(ThresholdAdd+0x20);
	if(EPROM_bChksum_ok == 1)
	{
		//Rev Threshold Occur
		//Rev Threshold Restore
		
		tpr_u16ELStrTime=a8_to_u16(&opr_data[4]);
		tpr_u16ELRestrTime=a8_to_u16(&opr_data[6]);
		//EL Threshold Occur
		//EL Threshold Restore
		
		tpr_u16OverCStrTime=a8_to_u16(&opr_data[12]);
	}
	Eprom_Read(ThresholdAdd+0x30);
	if(EPROM_bChksum_ok == 1)
	{
		tpr_u16OverCRestrTime=a8_to_u16(&opr_data[0]); 
		tpr_u32OverC_Thrshld=a8_to_u24(&opr_data[2]);
		tpr_u32OverC_ThrshldRest=a8_to_u24(&opr_data[5]);
		
		tpr_u16OverLStrTime=a8_to_u16(&opr_data[8]);
		tpr_u16OverLRestrTime=a8_to_u16(&opr_data[10]);
		tpr_u16OverL_Thrshld=a8_to_u16(&opr_data[12]);
	}
	Eprom_Read(ThresholdAdd+0x40);
	if(EPROM_bChksum_ok == 1)
	{
		tpr_u16OverL_ThrshldRest=a8_to_u16(&opr_data[0]);
		tpr_u16MagStrTime=a8_to_u16(&opr_data[2]);
		tpr_u16MagRestrTime=a8_to_u16(&opr_data[4]);        
		
		tpr_u16NeuMissStrTime=a8_to_u16(&opr_data[6]);
		tpr_u16NeuMissRestrTime=a8_to_u16(&opr_data[8]);
		
		tpr_u16KV35RestrTime=a8_to_u16(&opr_data[10]);
		tpr_u16LowPFStrTime=a8_to_u16(&opr_data[12]);
	}  
	Eprom_Read(ThresholdAdd+0x50);
	if(EPROM_bChksum_ok == 1)
	{
		tpr_u16LowPFRestrTime=a8_to_u16(&opr_data[0]);
		tpr_u16LowPF_Thrshld=a8_to_u16(&opr_data[2]);
		tpr_u16LowPF_ThrshldRest=a8_to_u16(&opr_data[4]);
		
		tpr_u16AbFreqStrTime=a8_to_u16(&opr_data[6]);
		tpr_u16AbFreqRestrTime=a8_to_u16(&opr_data[8]);
		tpr_u16AbFreq_Thrshldhigh=a8_to_u16(&opr_data[10]);
		tpr_u16AbFreq_Thrshldlow=a8_to_u16(&opr_data[12]);
	}  
	
	
	if(tpr_u16NeuDStrTime==0)
	tpr_u16NeuDStrTime=60;
	if(tpr_u16NeuDRestrTime==0)
	tpr_u16NeuDRestrTime=60;
	if(tpr_u16NeuD_Thrshld==0)
	tpr_u16NeuD_Thrshld=10000;
	if(tpr_u16NeuD_ThrshldRest==0)
	tpr_u16NeuD_ThrshldRest=9000;
	
	if(tpr_u16OverVStrTime==0)
	tpr_u16OverVStrTime=30;
	if(tpr_u16OverVRestrTime==0)
	tpr_u16OverVRestrTime=30;
	if(tpr_u16OverV_Thrshld==0)
	tpr_u16OverV_Thrshld=28000;
	if(tpr_u16OverV_ThrshldRest==0)
	tpr_u16OverV_ThrshldRest=27000;
	
	if(tpr_u16LowVStrTime==0)
	tpr_u16LowVStrTime=30;
	if(tpr_u16LowVRestrTime==0)
	tpr_u16LowVRestrTime=30;
	if(tpr_u16LowV_Thrshld==0)
	tpr_u16LowV_Thrshld=16800;
	if(tpr_u16LowV_ThrshldRest==0)
	tpr_u16LowV_ThrshldRest=18000;
	
	if(tpr_u16RevStrTime==0)
	tpr_u16RevStrTime=30;
	if(tpr_u16RevRestrTime==0)
	tpr_u16RevRestrTime=30;
	//Rev Threshold Occur
	//Rev Threshold Restore
	
	if(tpr_u16ELStrTime==0)
	tpr_u16ELStrTime=30;
	if(tpr_u16ELRestrTime==0)
	tpr_u16ELRestrTime=30;
	//EL Threshold Occur
	//EL Threshold Restore
	
	if(tpr_u16OverCStrTime==0)
	tpr_u16OverCStrTime=30;
	if(tpr_u16OverCRestrTime==0)
	tpr_u16OverCRestrTime=30; 
	if(tpr_u32OverC_Thrshld==0)
	tpr_u32OverC_Thrshld=60000;
	if(tpr_u32OverC_ThrshldRest==0)
	tpr_u32OverC_ThrshldRest=55000;
	
	if(tpr_u16OverLStrTime==0)
	tpr_u16OverLStrTime=30;
	if(tpr_u16OverLRestrTime==0)
	tpr_u16OverLRestrTime=30;
	if(tpr_u16OverL_Thrshld==0)
	tpr_u16OverL_Thrshld=14400;
	if(tpr_u16OverL_ThrshldRest==0)
	tpr_u16OverL_ThrshldRest=13000;
	
	if(tpr_u16MagStrTime==0)
	tpr_u16MagStrTime=10;
	if(tpr_u16MagRestrTime==0)
	tpr_u16MagRestrTime=10;
	
	if(tpr_u16NeuMissStrTime==0)
	tpr_u16NeuMissStrTime=20;
	if(tpr_u16NeuMissRestrTime==0)
	tpr_u16NeuMissRestrTime=20;
	
	if(tpr_u16KV35RestrTime==0)
	tpr_u16KV35RestrTime=20;
	
	R_WDT_Restart();//	WDTCTL = WDT_ARST_1000;
	
	load_pon();
	
	if(test_mode ==0)
	TPR_vPowerEventLog();
	
	Eprom_Read(cum_tpr_addrs);//cumulative count
	if(EPROM_bChksum_ok==1)
	{
		TPR_u16cum_tpr_c =a8_to_u16(&opr_data[0]);
		BILL_u8btpr_c=opr_data[2];
	}    
	
	Eprom_Read(volt_tamp_status);/////voltage event
	if(EPROM_bChksum_ok==1)
	{
		volt_count=opr_data[0];
		volt_of = opr_data[1];
		//		volt_loc=opr_data[2];
		//		if(opr_data[3] == 1)
		//			neu_tpr_f=1;		
		if(opr_data[4] == 1)
			TP7.b.high_v_f=1;
//		if(opr_data[5] == 1)
//		TP7.b.LowVolt_tpr_f=1;
	}
	
	R_WDT_Restart();//WDTCTL = WDT_ARST_1000;
	
	Eprom_Read(curr_tamp_status);////current event
	if(EPROM_bChksum_ok==1)
	{
		curr_count=opr_data[0]; 
		curr_of=opr_data[1];
		//		curr_loc=opr_data[2];
		
		if(opr_data[3]==1)
		TP1.b.rev_tpr_f=1;
		if(opr_data[4]==1)
		TP2.b.el_tpr_f=1;		
		if(opr_data[5]==1)
		TP1.b.ol_tpr_f=1;
		if(opr_data[6] == 1)
		TP2.b.oc_tpr_f=1;
	}   
	
	R_WDT_Restart();//WDTCTL = WDT_ARST_1000;
	
	Eprom_Read(other_tamp_status);//other events
	
	if(EPROM_bChksum_ok==1)
	{
		others_count=opr_data[0];
		others_of = opr_data[1];
		//		others_loc=opr_data[2];
		if(opr_data[3]==1)
		TP1.b.mag_tpr_f=1;
		if(opr_data[4]==1)
		TP3.b.neu_dis_tpr_f=1;	
		
		if(opr_data[5]==1)
		TP5.b.kv35_tpr_f=1;
		if(opr_data[6]==1)
		TP4.b.LowPF_tpr_f=1;
		if(opr_data[7]==1)
		TP4.b.AbFreq_tpr_f=1;
		if(opr_data[8]==1)
		TP3.b.neu_miss_tpr_f=1;
		
		
	}
	Eprom_Read(trans_tamp_status);///////trnasition event
	if(EPROM_bChksum_ok==1)
	{
		trans_count=opr_data[0];
		trans_of = opr_data[1];
		//		trans_loc=opr_data[2];
	}
	
	//	Eprom_Read(control_tamp_status);///////control event
	//	if(EPROM_bChksum_ok==1)
	//	{
	//		control_count=opr_data[0];
	//		control_of = opr_data[1];
	//        //		conct_loc=opr_data[2];
	//	}  
	Eprom_Read(Diagnostics_status);///////Diagnostics_status event
	if(EPROM_bChksum_ok==1)
	{
		Diagnostics_count=opr_data[0];
		Diagnostics_of = opr_data[1];
		//		conct_loc=opr_data[2];
	}  
	
	
	Eprom_Read(nonroll_status); 
	if(EPROM_bChksum_ok==1)
	{
		nonroll_count = opr_data[0];
		nonroll_of = opr_data[1];
		if(opr_data[3] != 0)
		{        
			TP5.b.tc_tpr_f=1;
		}
		//		if(opr_data[4] != 0)
		//		{
		//			rly_malfun_f=1;			
		//		}
	}
	//    Neutral Miss count	High Voltage Count	Low Voltage Count	CT Reverse Count	Over Load Count	Magnetic Tamper count	Neutral  Disturbance Count	Frequency Variation Count	low pf	TC

	Eprom_Read(TPRCNT_Addr);
	if(EPROM_bChksum_ok==1)
	{
		TPRCNT_u8NeuMiss    =opr_data[0];
		TPRCNT_u8VHigh      =opr_data[1];
		TPRCNT_u8VLow       =opr_data[2];
		TPRCNT_u8Rev        =opr_data[3];
		TPRCNT_u8EL         =opr_data[4];
		TPRCNT_u8OC         =opr_data[5];
		TPRCNT_u8OverLoad   =opr_data[6];
		TPRCNT_u8Mag        =opr_data[7];
		TPRCNT_u8NeuDis     =opr_data[8];
		TPRCNT_u8LowPF      =opr_data[9];
		TPRCNT_u835kv       =opr_data[10];
		TPRCNT_u8FreqTamp   =opr_data[11];
		TPRCNT_u8TC         =opr_data[12];
	}
	Eprom_Read(FirstStrAddr);
	if(EPROM_bChksum_ok==1)
	{
		//        TPR_bFirstStrTpr=opr_data[0];
	}    
	Eprom_Read(FirstRestrAddr);
	if(EPROM_bChksum_ok==1)
	{
		//        TPR_bFirstRestrTpr=opr_data[0];
	} 
	Eprom_Read(NeutralTestCountAdd);
	if(EPROM_bChksum_ok==1)
	neu_test_c=opr_data[0];
	else 
	neu_test_c=0;
	
	Eprom_Read(BatteryStatus);
	if(EPROM_bChksum_ok ==1)
	{
		if(opr_data[5]==1)
		{
			TP6.b.bat_discharge_f=1;
			disable_bkup_f=1;
		}
		else if(opr_data[5]==0)
		{
			TP6.b.bat_discharge_f=0;
			disable_bkup_f=0;
		}
	}
	else
	{
		TP6.b.bat_discharge_f=0;
		disable_bkup_f=0;
	}
	
	if((TamperSelByte & TprSel_35KV)==TprSel_35KV)	//if(test_mode==0)
	{
		if(TP6.b.bat_discharge_f==0)
		{
			Eprom_Read(PowerDownByteAdd);
			if(EPROM_bChksum_ok ==1)
			{
				if(opr_data[3] != 0x01)
				{
					abnrm_c++;
					if(abnrm_c>=2)
					{
						abnrm_c=0;
						if((hv_persis_c<300) && (TP5.b.kv35_tpr_f ==0))
						{
							TP5.b.kv35_tpr_f=1;
							TP5.b.kv35_tprstr_f=1;
							TP.b.OtherEvent=1;
							others_count++;
							TPR_u16cum_tpr_c++;
							BILL_u8btpr_c++;
							check_tpr();
						}
					}
					hv_persis_c=0;
				}
				FUN_vfill_3byteR(MET_u32Cum_kwh,&opr_data[2]);
				opr_data[3]=0x02;
				time_stamp(&opr_data[4]);
				Eprom_Write(PowerDownByteAdd);
			}
			else
			{
				abnrm_c=0;
				hv_persis_c=0;
			}
		}
		else
		{
			abnrm_c=0;
			hv_persis_c=0;
		}
	}
}

u8 TPR_vSnapshotStore(u16 u16Loc_addr,u16 u16init_addr,u8 u8count,u8 u8flag_loc,u8 u8max_compart_size,u8 u8occur)
{
	u16 u16addr;
	u8 over_flow=0;
	Eprom_Read(u16Loc_addr);
	opr_data[u8flag_loc]=u8occur;
	u16addr=u16init_addr+((u16)(opr_data[2])*0x10);
	tpr_u16addr=u16addr;
	opr_data[0]=u8count;
	opr_data[14]=event_id;
	over_flow=opr_data[1];
	opr_data[2]++;
	if(opr_data[2]>=u8max_compart_size)
	{
		opr_data[2]=0;
		opr_data[1]=over_flow=1;
	}
	Eprom_Write(u16Loc_addr);
	
	if(event_id==101)
	Eprom_Read(PowerDownTimeAdd);
	else
	time_stamp(&opr_data[0]);
	opr_data[5]=event_id;
	
	if(MET_bph_ct_f1==0)
	FUN_vfill_3byteR(MET_u32ip_rms,&opr_data[8]);
	else
	FUN_vfill_3byteR(MET_u32in_rms,&opr_data[8]);

	FUN_vfill_2byteR(MET_u16v_rms,&opr_data[10]);
	
	opr_data[11]=MET_u16pf/10;
	FUN_vfill_3byteR(MET_u32Cum_kwh,&opr_data[14]);
	
	Eprom_Write(u16addr);
	return over_flow;
}
void TPR_vSaveFirstStrTpr(void)
{
	//    if(TPR_bFirstStrTpr==0)
	{
		//        opr_data[0]=TPR_bFirstStrTpr=1;
		opr_data[1]=(u8)event_id;
		time_stamp(&opr_data[2]); 
		Eprom_Write(FirstStrAddr);
	}
}
void TPR_vSaveFirstRestrTpr(void)
{
	//    if(TPR_bFirstRestrTpr==0)
	{
		//        opr_data[0]=TPR_bFirstRestrTpr=1;
		opr_data[1]=(u8)event_id;
		time_stamp(&opr_data[2]); 
		Eprom_Write(FirstRestrAddr);
	}
}
void TPR_vSaveLastStrTpr(void)
{
	opr_data[0]=(u8)event_id;
	time_stamp(&opr_data[1]); 
	Eprom_Write(LastStrAddr);
}
void TPR_vSaveLastRestrTpr(void)
{
	opr_data[0]=(u8)event_id;
	time_stamp(&opr_data[1]); 
	Eprom_Write(LastRestrAddr);
}
void TPR_vCurrentRev_Detection(void)
{
	
	if((MET_u16Kw < 60 && MET_u16ELKw < 60)|| TP1.b.mag_tpr_f==1 )
	return;
	
	if(MET_bkw_negative_f==0 && MET_bkw_el_negative_f==0)
	{
		TPR_u16rev_per_c=0;
		if(TP1.b.rev_tpr_f==1)
		{
			TPR_u16rev_per_c1++;
			if(TPR_u16rev_per_c1 >= tpr_u16RevRestrTime)            //12c
			{
				TPR_u16rev_per_c1=0;               
				TP1.b.rev_tprrestr_f=1;
				TP.b.CurrentRelatedEvent=1;
				TP1.b.rev_tpr_f=0;             
				curr_count++;
			}
		}
	}
	else
	{
		TPR_u16rev_per_c1=0x00;
		if(TP1.b.rev_tpr_f==0)
		{
			TPR_u16rev_per_c++;
			if(TPR_u16rev_per_c >= tpr_u16RevStrTime)            //12c
			{
				TPR_u16rev_per_c=0;
				TP1.b.rev_tprstr_f=1;
				TP.b.CurrentRelatedEvent=1;              
				TP1.b.rev_tpr_f=1;              
				curr_count++;
				TPR_u16cum_tpr_c++;
				BILL_u8btpr_c++;
			}
		}
	}    
	
}
void TPR_vCurrentEL_Detection(void)
{
	if((MET_u16Kw <20 && MET_u16ELKw <20) || (pow_up_f==1) || (TP1.b.mag_tpr_f==1) || (test_mode == 1) )
	return;   
	
	if(MET_bEl_glow==0)
	{
		TPR_u16el_per_c=0;
		if(TP2.b.el_tpr_f==1)
		{
			TPR_u16el_per_c1++;
			if(TPR_u16el_per_c1 >= tpr_u16ELRestrTime)            //12c
			{
				TPR_u16el_per_c1=0;
				TP2.b.el_tprrestr_f=1;
				TP.b.CurrentRelatedEvent=1;
				TP2.b.el_tpr_f=0;
				curr_count++;
			}
		}
	}
	else 
	{
		TPR_u16el_per_c1=0x00;
		if(TP2.b.el_tpr_f==0)
		{
			TPR_u16el_per_c++;
			if(TPR_u16el_per_c >= tpr_u16ELStrTime)            //12c
			{
				TPR_u16el_per_c=0;
				TP2.b.el_tprstr_f=1;
				TP2.b.el_tpr_f=1;
				TP.b.CurrentRelatedEvent=1;
				curr_count++;				
				TPR_u16cum_tpr_c++;
				BILL_u8btpr_c++;
			}
		}
	}
}
void TPR_vCurrentOver_Detection(void)
{                            
	if((MET_u16Kw < 20 && MET_u16ELKw < 20)|| (TP1.b.mag_tpr_f==1))
	return;
	
	if((MET_u32ip_rms <= tpr_u32OverC_ThrshldRest) && (MET_u32in_rms <= tpr_u32OverC_ThrshldRest))
	{
		TPR_u16oc_per_c=0;
		if(TP2.b.oc_tpr_f==1)
		{
			TPR_u16oc_per_c1++;
			if(TPR_u16oc_per_c1 >= tpr_u16OverCRestrTime)            //12c
			{
				TPR_u16oc_per_c1=0;
				TP2.b.oc_tprrestr_f=1;
				TP2.b.oc_tpr_f=0;
				TP.b.CurrentRelatedEvent=1;
				curr_count++;
			}
		}
	}
	else if((MET_u32ip_rms > tpr_u32OverC_Thrshld) || (MET_u32in_rms > tpr_u32OverC_Thrshld))
	{
		TPR_u16oc_per_c1=0x00;
		if(TP2.b.oc_tpr_f==0)
		{
			TPR_u16oc_per_c++;
			if(TPR_u16oc_per_c >= tpr_u16OverCStrTime)            //12c
			{
				TPR_u16oc_per_c=0;
				TP2.b.oc_tprstr_f=1;
				TP2.b.oc_tpr_f=1;              
				TP.b.CurrentRelatedEvent=1;
				curr_count++;
				TPR_u16cum_tpr_c++;
				BILL_u8btpr_c++;               
			}
		}
	}
}
void TPR_vCurrentOLoad_Detection(void)
{                                  
	if((MET_u16Kw < 20 && MET_u16ELKw < 20)|| TP1.b.mag_tpr_f==1 )
	return;
	
	if((MET_u16Kw <= tpr_u16OverL_ThrshldRest) && (MET_u16ELKw <= tpr_u16OverL_ThrshldRest))
	{
		TPR_u16ol_per_c=0;
		if(TP1.b.ol_tpr_f==1)
		{
			TPR_u16ol_per_c1++;
			if(TPR_u16ol_per_c1 >= tpr_u16OverLRestrTime)            //12c
			{
				TPR_u16ol_per_c1=0;
				TP1.b.ol_tpr_f=0;  
				//				TPR_u8LockoutCutCnt=0;
				TP2.b.ol_tprrestr_f=1;
				TP.b.CurrentRelatedEvent=1;
				curr_count++;
			}
		}
	}
	else if((MET_u16Kw > tpr_u16OverL_Thrshld) || (MET_u16ELKw > tpr_u16OverL_Thrshld))
	{
		TPR_u16ol_per_c1=0x00;
		if(TP1.b.ol_tpr_f==0)
		{
			TPR_u16ol_per_c++;
			if(TPR_u16ol_per_c >= tpr_u16OverLStrTime)            //12c
			{
				TPR_u16ol_per_c=0;
				TP1.b.ol_tpr_f=1;
				//				ol_tpr_f1=1;
				TP2.b.ol_tprstr_f=1;
				TP.b.CurrentRelatedEvent=1;
				curr_count++;
				TPR_u16cum_tpr_c++;
				BILL_u8btpr_c++;
			}
		}
	}
}





void TPR_vNeutralMissDisturbanceDetection(void)
{
	if((MET_u16Kw < 20 && MET_u16ELKw < 20)|| TP1.b.mag_tpr_f==1 )
	return;
	
	if((TamperSelByte & TprSel_NeuMiss)==TprSel_NeuMiss)
	{
		if(TP3.b.neu_miss_f==0)         
		{
			TPR_u16neu_per_c=0;
			if(TP3.b.neu_miss_tpr_f==1)
			{
				TPR_u16neu_per_c1++;
				if(TPR_u16neu_per_c1 >= tpr_u16NeuMissRestrTime)
				{
					TPR_u16neu_per_c1=0;
					TP3.b.neu_miss_tprrestr_f=1;
//					TP.b.VoltageRelatedEvent=1;
//					volt_count++;
					TP.b.OtherEvent=1;
					others_count++;
					TP3.b.neu_miss_tpr_f=0;
					//                normal_index();
				}
			}
		}
		else 
		{
			TPR_u16neu_per_c1=0x00;
			if(TP3.b.neu_miss_tpr_f==0)
			{
				TPR_u16neu_per_c++;
				if(TPR_u16neu_per_c >= tpr_u16NeuMissStrTime)
				{
					TPR_u16neu_per_c=0;
					//                    neu_tpr();
					TP3.b.neu_miss_tprstr_f=1;
					TP3.b.neu_miss_tpr_f=1;
//					TP.b.VoltageRelatedEvent=1;
//					volt_count++;
					TP.b.OtherEvent=1;
					others_count++;
					TPR_u16cum_tpr_c++;
					BILL_u8btpr_c++;
				}
			}
		}
	}
	
	if((TamperSelByte & TprSel_NeutralDis)==TprSel_NeutralDis)
	{
		
		if((TP4.b.dcchop_f==0) && (TP3.b.neu_dis_f==0) && (TP5.b.kv35_tpr_f ==0) && (batt_disp_f==0) && (test_mode==0)&&(MET_u16v_rms >= tpr_u16NeuD_ThrshldRest)) 
		{
			TPR_u16neud_per_c=0;
			if(TP3.b.neu_dis_tpr_f==1)
			{
				TPR_u16neud_per_c1++;
				if(TPR_u16neud_per_c1>=tpr_u16NeuDRestrTime)
				{
					TPR_u16neud_per_c1=0;
					TP3.b.neu_dis_tprrestr_f=1;
					TP.b.OtherEvent=1;
					TP3.b.neu_dis_tpr_f=0;
					others_count++;
				}
			}
		}
		else if(((TP4.b.dcchop_f==1) || (TP3.b.neu_dis_f==1)) && TP5.b.kv35_tpr_f == 0)
		{
			TPR_u16neud_per_c1=0x00;
			if(TP3.b.neu_dis_tpr_f==0)
			{
				TPR_u16neud_per_c++;
				if(TPR_u16neud_per_c>=tpr_u16NeuDStrTime)
				{
					TPR_u16neud_per_c=0;
					TP3.b.neu_dis_tprstr_f=1;
					TP3.b.neu_dis_tpr_f=1;
					TP.b.OtherEvent=1;
					TPR_u16cum_tpr_c++;
					BILL_u8btpr_c++;
					others_count++;
				}
			}
		}
	}
}
void TPR_vLowPF_Detection(void)
{
	if(MET_u16Kw < 40 && MET_u16ELKw < 40)
	return;
	if((MET_u16pf > tpr_u16LowPF_ThrshldRest))
	{
		TPR_u16LowPF_per_c=0;
		if(TP4.b.LowPF_tpr_f==1)
		{
			TPR_u16LowPF_per_c1++;
			if(TPR_u16LowPF_per_c1 >= tpr_u16LowPFRestrTime)            //12c
			{
				TPR_u16LowPF_per_c1=0;
				TP4.b.LowPF_tprrestr_f=1;
				TP4.b.LowPF_tpr_f=0;
				TP.b.OtherEvent=1;
				others_count++;
			}
		}
	}
	else if((MET_u16pf < tpr_u16LowPF_Thrshld) )
	{
		TPR_u16LowPF_per_c1=0x00;
		if(TP4.b.LowPF_tpr_f==0)
		{
			TPR_u16LowPF_per_c++;
			if(TPR_u16LowPF_per_c >= tpr_u16LowPFStrTime)            //12c
			{
				TPR_u16LowPF_per_c=0;
				TP4.b.LowPF_tprstr_f=1;
				TP4.b.LowPF_tpr_f=1;              
				TP.b.OtherEvent=1;
				others_count++;
				TPR_u16cum_tpr_c++;
				BILL_u8btpr_c++;               
			}
		}
	}
}
void TPR_vAbFreq_Detection(void)
{
	if((MET_u16freq >= tpr_u16AbFreq_Thrshldlow)&&(MET_u16freq <= tpr_u16AbFreq_Thrshldhigh))
	{
		TPR_u16AbFreq_per_c=0;
		if(TP4.b.AbFreq_tpr_f==1)
		{
			TPR_u16AbFreq_per_c1++;
			if(TPR_u16AbFreq_per_c1 >= tpr_u16AbFreqRestrTime)            //12c
			{
				TPR_u16AbFreq_per_c1=0;
				TP4.b.AbFreq_tprrestr_f=1;
				TP4.b.AbFreq_tpr_f=0;
				TP.b.OtherEvent=1;
				others_count++;
			}
		}
	}
	else if((MET_u16freq < tpr_u16AbFreq_Thrshldlow)||(MET_u16freq > tpr_u16AbFreq_Thrshldhigh))
	{
		TPR_u16AbFreq_per_c1=0x00;
		if(TP4.b.AbFreq_tpr_f==0)
		{
			TPR_u16AbFreq_per_c++;
			if(TPR_u16AbFreq_per_c >= tpr_u16AbFreqStrTime)            //12c
			{
				TPR_u16AbFreq_per_c=0;
				TP4.b.AbFreq_tprstr_f=1;
				TP4.b.AbFreq_tpr_f=1;              
				TP.b.OtherEvent=1;
				others_count++;
				TPR_u16cum_tpr_c++;
				BILL_u8btpr_c++;               
			}
		}
	}
}
void check_neu_feature(void)
{
	TP4.b.neu_fea_f1 = 0;
	TP3.b.neu_miss_f=0;
	TP3.b.neu_dis_f=0;
	if(pow_up_f==0)
	{
		//    if(v_rms < 10000)
		//    {
		//		TP4.b.neu_fea_f1 = 1;
		//		neu_miss_f=1;
		//    }
#if NEUTRAL_ON_POWERCT
		if((P1IN & PCTpres) == 0)
#endif
		
		if((MET_u16v_rms < 3000) ) //11600
		{
			TP4.b.neu_fea_f1 = 1;
			//            dcchop2_f=1;
			TP3.b.neu_miss_f=1;
			//            v_cnt=0x0000;
			
		}
		else if((MET_u16v_rms <= tpr_u16NeuD_Thrshld) && (TP1.b.mag_tpr_f == 0) && MET_bEl_glow == 1 && ((MET_u32ip_rms > 500) || (MET_u32in_rms > 500)))
		{
				TP4.b.neu_fea_f1 = 1;
				TP3.b.neu_dis_f=1;
		}
		else
		{
			if(((((MET_u16v_rms <23500) || (MET_u16v_rms >=24500))  && (MET_u32ip_rms > 900) && (MET_u32in_rms>10) && (MET_u32in_rms<70)) || (TP4.b.dcchop_f==1))&&(TP1.b.mag_tpr_f==0))//v_rms <22000 && ip_rms >=250 &&in_rms>13 && in_rms<32 )//by manoj.dhatterwal sir 8-11-2012
			{
				TP4.b.neu_fea_f1 = 1;
				TP3.b.neu_dis_f=1;
			}
			
			if(MET_u16freq <45000 || MET_u16freq >54000)
			{
				TP4.b.neu_fea_f1 = 1;
				TP3.b.neu_dis_f=1;
			}
			
			if(MET_u16POS_MAX_1s > MET_u16NEG_MAX_1s)
			{
				if((MET_u16POS_MAX_1s- MET_u16NEG_MAX_1s) > 2000)
				{ 
					TP4.b.neu_fea_f1 = 1;
					TP3.b.neu_dis_f=1;
				}
			}
			else
			{
				if((MET_u16NEG_MAX_1s- MET_u16POS_MAX_1s) > 2000)
				{ 
					TP4.b.neu_fea_f1 = 1;
					TP3.b.neu_dis_f=1;
				}
			}
		}
		if((MET_bph_ct_f1==1)&&(TP4.b.neu_fea_f1==0)&&(TP1.b.mag_tpr_f == 0))
		{
			if(MET_u16POS_MAX_C_1s > MET_u16NEG_MAX_C_1s)//dc immunity in earthload //detect of el
			{
				if((MET_u16POS_MAX_C_1s - MET_u16NEG_MAX_C_1s) > 1000)
				{ 
					TPR_u8dc_counter++;
					if(TPR_u8dc_counter > 10)
					TP5.b.dcinac_f=1;
					//    TP4.b.neu_fea_f1 = 1;
					//  diode_f=1;
				}
				else
				{
					TPR_u8dc_counter=0;
					TP5.b.dcinac_f=0;
				}
			}
			else
			{
				if((MET_u16NEG_MAX_C_1s - MET_u16POS_MAX_C_1s) > 1000)
				{ 
					TPR_u8dc_counter++;
					if(TPR_u8dc_counter > 10)
					TP5.b.dcinac_f=1;
					//  TP4.b.neu_fea_f1 = 1;
					//diode_f=1;
				}
				else
				{
					TPR_u8dc_counter=0;
					TP5.b.dcinac_f=0;
				}
				
			} 
		}     
	}
	//     if(MET_u16POS_MAX_1s > MET_u16NEG_MAX_1s)
	//     {
	//       if((MET_u16POS_MAX_1s- MET_u16NEG_MAX_1s) > 2000)
	//       { 
	//         dcchop2_f=1;
	//         TP4.b.neu_fea_f1 = 1;
	//         TP3.b.neu_dis_f=1;
	//       }
	//     }
	//    else
	//      {
	//       if((MET_u16NEG_MAX_1s- MET_u16POS_MAX_1s) > 2000)
	//       { 
	//         dcchop2_f=1;
	//         TP4.b.neu_fea_f1 = 1;
	//         TP3.b.neu_dis_f=1;
	//       }
	//     }   
}

/*void neu_tpr(void)
{
	TP3.b.neu_miss_tprstr_f=1;
	TP3.b.neu_miss_tpr_f=1;
	//  TPR_bVoltageRelatedEvent=1;
	TP.b.OtherEvent = 1;
	//defraud_t1();
	//  volt_count++;
	others_count++;
	TPR_u16cum_tpr_c++;
	BILL_u8btpr_c++;


}*/

void TPR_vVoltageOver_Detection(void)
{
	if(MET_u16v_rms < tpr_u16OverV_ThrshldRest) 
	{
		TPR_u16HighVolt_per_c=0; 
		if(TP7.b.high_v_f == 1)
		{
			TPR_u16HighVolt_per_c1++;
			if(TPR_u16HighVolt_per_c1 >=tpr_u16OverVRestrTime)
			{
				TPR_u16HighVolt_per_c1=0;      
				TP7.b.highv_tprrestr_f=1;
				TP.b.VoltageRelatedEvent=1;
				volt_count++;
				TP7.b.high_v_f=0; 	
			}
		}
	}
    else if((MET_u16v_rms >=tpr_u16OverV_Thrshld) && (TP4.b.neu_fea_f1==0))     
	{
        TPR_u16HighVolt_per_c1=0;
        if(TP7.b.high_v_f == 0)
        {
			TPR_u16HighVolt_per_c++;
			if(TPR_u16HighVolt_per_c >=tpr_u16OverVStrTime)
            {
				TPR_u16HighVolt_per_c=0;
				TP7.b.highv_tprstr_f=1; 
				TP7.b.high_v_f=1;  
				TP.b.VoltageRelatedEvent=1;
				TPR_u16cum_tpr_c++;
				BILL_u8btpr_c++;
				volt_count++;
            }
        }
	}
}

void TPR_vVoltageLow_Detection(void)
{
	if(( MET_u16v_rms > tpr_u16LowV_ThrshldRest) && (TP1.b.mag_tpr_f == 0))
	{
		TPR_u16LowVolt_per_c=0;
		if(TP7.b.LowVolt_tpr_f == 1)
		{
			TPR_u16LowVolt_per_c1++;
			if(TPR_u16LowVolt_per_c1 >= tpr_u16LowVRestrTime)    
			{
				TPR_u16LowVolt_per_c1=0;
				TP7.b.LowVolt_tpr_f=0;
				TP7.b.LowVolt_tprrestr_f=1;                   
				TP.b.VoltageRelatedEvent=1;   
				volt_count++;
			}
		}
	}
	else if((MET_u16v_rms <= tpr_u16LowV_Thrshld) && (MET_u16v_rms > 11000)  && (TP1.b.mag_tpr_f==0) && (TP4.b.neu_fea_f1==0))
	{
		TPR_u16LowVolt_per_c1=0;
		if(TP7.b.LowVolt_tpr_f==0)
		{
			TPR_u16LowVolt_per_c++;
			if(TPR_u16LowVolt_per_c >= tpr_u16LowVStrTime)       
			{
				TPR_u16LowVolt_per_c=0;
				TP7.b.LowVolt_tpr_f=1;
				TP7.b.LowVolt_tprstr_f=1;               
				TP.b.VoltageRelatedEvent=1;                 
				TPR_u16cum_tpr_c++;
				BILL_u8btpr_c++;
				volt_count++;
			}
		}
	}
}


void TPR_vPowerEventLog(void)
{
	Eprom_Read(0x28f0);
	if(EPROM_bChksum_ok == 1)
	{
		tpr_u16pwr_time_thrshld=a8_to_u16(&opr_data[0]);
	}
	if(tpr_u16pwr_time_thrshld == 0)
	tpr_u16pwr_time_thrshld=0x01;
	
	Eprom_Read(pwr_tamp_status);
	if(EPROM_bChksum_ok ==1)
	{
		on_off_cnt = opr_data[0];
		on_off_loc = opr_data[2];
		on_off_of=opr_data[1];
		//		lon_off_cnt=opr_data[3];
		//		son_off_cnt=opr_data[4];
	}
	
	//	Eprom_Read(BILL_POff_minAdd);
	//	if(EPROM_bChksum_ok == 1)
	//	{
	//		poff_mins=a8_to_u16(&opr_data[0]);			
	//	}
	Eprom_Read(PowerDownTimeAdd);
	
	time_diff();
	
	////power off min calculation
	
	//	poff_mins+=bat_on_secs;
	//	fill_oprzero();
	//	fill_2data(poff_mins,&opr_data[1]);		
	//	Eprom_Write(BILL_POff_minAdd);
	
	//	Daily_u16Poff_mins+=bat_on_secs;
	//    Eprom_Read(DailyOffCount);		
	//    fill_2data(Daily_u16Poff_mins,&opr_data[2]);	    
	//	Eprom_Write(DailyOffCount);
	
	if(bat_on_secs>=15)
	{		
		on_off_cnt++;
		event_id=101;
		on_off_of=TPR_vSnapshotStore(pwr_tamp_status,pwr_init_add,on_off_cnt,5,pwr_max_loc,1);
		event_id=102;
		on_off_of=TPR_vSnapshotStore(pwr_tamp_status,pwr_init_add,on_off_cnt,5,pwr_max_loc,1);

		daily_on_off++;
		
		Eprom_Read(DailyOffCount);		
		opr_data[0]=daily_on_off;
		Eprom_Write(DailyOffCount);
		//	  a8Daily_alert_status[2]|=DES_power_outageO;
		//	  a8Daily_alert_status[2]|=DES_power_outageR;
	}
	Eprom_Read(pwr_tamp_status);	
	if(EPROM_bChksum_ok ==1)
	{
		on_off_loc=opr_data[2];
		on_off_of=opr_data[1];
	}
	//	Eprom_Read(cum_poff_addrs);    
	//	if(EPROM_bChksum_ok == 1)
	//	{
	//		cum_poff_min = char_array_to_long1(&opr_data[0]);		
	//		cum_poff_min += bat_on_secs;
	//		if(cum_poff_min >= 1000000000)
	//			cum_poff_min=0;
	//		FUN_vfill_3byteR(cum_poff_min,&opr_data[3]);
	//		Eprom_Write(cum_poff_addrs);//
	//	}
}
void load_pon(void)
{
	unsigned char i;
	u32 tempint;
	u8 mem16_ptr1;
	pon_ptr = 0;
	
	for(i=0;i<16;i++)            // since 8 pages are used for energy write
	{
		Eprom_Read(CircularBufferMDLSkwh+pon_ptr);
		if(EPROM_bChksum_ok==1)
		{
			tempint = a8_to_u32(&opr_data[0]); //msb
			if(cum_pow_on <= tempint)
			{
				cum_pow_on = tempint;
				pow_on = a8_to_u16(&opr_data[4]); //msb
				u16_bill_pow_on = a8_to_u16(&opr_data[6]); //msb
				mem16_ptr1 = pon_ptr;
				
			}
		}
		pon_ptr += 0x10;
		
	}
	
	pon_ptr = mem16_ptr1 + 0x10;
}
/***********************************************************************************************************************
* Function Name: port42
* Description  : This function initializes the clock generator.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
static void __near LPM_PORT42_Interrupt(void)
{
	if((TC_OPEN_Port & TC_OPEN_Bit) == TC_OPEN_Bit && (TP5.b.tc_tpr_f==0))
	{
		
		if((TamperSelByte & TprSel_TCOpen)==TprSel_TCOpen)
		{
			if(TP6.b.sleep_f)
			{
				MET_u16v_rms=0;
//	          	TP5.b.tc_tpr_f=1;
//			    TP5.b.tc_tprstr_f=1;
//			    nonroll_count++;
//			    TP.b.NonRollEvent=1;
//			    //tpc_count++;
//			    TPR_u16cum_tpr_c++;
//			    BILL_u8btpr_c++;	
				TP6.b.tc_exit_f=1;
				TC_u16sleepcounter=0;
				TC_u8exitcounter=0;
			}
		}
//		else
//		{
//				v_rms=0;
//				tc_tpr_f=1;
//				tc_tprstr_f=1;
//				//tpc_count++;
//				cum_tpr_c++;
//				btpr_c++;
//				tc_transit_f=1;
//
//		}
	}
	TP5.b.tc_tprstopmode=1;
	PIF7 = 0;//P1IFG &= ~(TC_OPEN_Bit);
}
/* Start user code for adding. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
