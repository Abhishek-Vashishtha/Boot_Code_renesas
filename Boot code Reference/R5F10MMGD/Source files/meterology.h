/***********************************************************************************************************************
* File Name    : variable.h
* Version      : CodeGenerator for RL78/L12 V2.00.00.07 [22 Feb 2013]
* Device(s)    : R5F10MMG
* Tool-Chain   : CC-RL
* Description  : This file implements device driver for CGC module.
* Creation Date: 7/2/2013
***********************************************************************************************************************/

#ifndef MET_H
#define MET_H

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
void cal_avg_pf(void);
void MET_vcheck_kwh_kvah(void);
void MET_vcheck_coffs(void);
void MET_vSD24Init(void);
void MET_vInit(void);
void calibration(void);
void lag_calibration(void);
void MET_vSampleReady(void);
void calculate_kwh(void);
void cal_app_power_vi(void);
void cal_pf(void);
void compute_energy(void);
void store_energy(void);
void roll_over_kvah(void);
void roll_over(void);
void MET_vCal_freq(void);
void pfsigncal(void);
void save_pd(void);
void save_pulse_cntrs(void);
void write_energy(void);
void write_kvaenergy(void);
void meterology(void);
void CopyBits(void);
void check_MET_s32IpDC_acc(void);
void check_MET_s32InDC_acc(void);
void INVERSEIpDCcon(void);
void INVERSEInDCcon(void);
void ClearRawNew(void);
void ClearRaw(void);
void clear_ram(void);
void log_battery_status(void);
/* Start user code for function. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
#endif