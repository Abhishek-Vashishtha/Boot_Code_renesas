/***********************************************************************************************************************
* File Name    : variable.h
* Version      : CodeGenerator for RL78/L12 V2.00.00.07 [22 Feb 2013]
* Device(s)    : R5F10MMG
* Tool-Chain   : CC-RL
* Description  : This file implements device driver for CGC module.
* Creation Date: 7/2/2013
***********************************************************************************************************************/

#ifndef TPR_H
#define TPR_H

/***********************************************************************************************************************
Macro definitions (Register bit)
***********************************************************************************************************************/
/***********************************************************************************************************************
Macro definitions
***********************************************************************************************************************/
/***********************************************************************************************************************
Typedef definitions
***********************************************************************************************************************/

/***********************************************************************************************************************
Global functions
***********************************************************************************************************************/
u8 TPR_vSnapshotStore(u16 ,u16 ,u8 ,u8 ,u8 ,u8 );
void check_tpr(void);
void TPR_vSaveFirstStrTpr(void);
void TPR_vSaveFirstRestrTpr(void);
void TPR_vSaveLastStrTpr(void);
void TPR_vSaveLastRestrTpr(void);
void TPR_vCurrentRev_Detection(void);
void TPR_vCurrentEL_Detection(void);
void TPR_vAbFreq_Detection(void);
void check_neu_feature(void);
void TPR_vLowPF_Detection(void);
void TPR_vNeutralMissDisturbanceDetection(void);
void TPR_vCurrentOLoad_Detection(void);
void TPR_vCurrentOver_Detection(void);
void TPR_vInitRam(void);
void load_pon(void);
void TPR_vPowerEventLog(void);
void TPR_vVoltageLow_Detection(void);
void TPR_vVoltageOver_Detection(void);
/* Start user code for function. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
#endif