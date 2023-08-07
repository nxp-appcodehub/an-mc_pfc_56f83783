#ifndef _PMSM_SPEEDVECTORCTRL_H_
#define _PMSM_SPEEDVECTORCTRL_H_

/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * */

/*.*************************************************************************
*
*  Application configuration file
*
****************************************************************************.*/

/*=============================================================   Manually input Macros ================================================================*/
//-------------------------------------------------------------------------------------------------------------------------------------------------------


/* FreeMASTER enable control */
#define FMSTR_ON					           1	 /* 1:Enable  0:Disable */

#define DEADTIME_COM						   0

/* Dead time compensation */
#define DTC_I_THRESHOLD						   0.05  /* [A]; DTC is enabled only when current value is larger than this number */
#define DTC_DUTY							   0.024 /* [n/a]; Compensated duty. e.g. 1.5us/62.5us = 0.024 */


/* Pressure relax waiting time before next start up */
#define MC_PRESSURE_RELAX_DURATION	           		0.01  /* [min]; default waiting time before next start up */
#define MC_PRESSURE_RELAX_DURATION_RUNTOFAULT  		0.5   /* [min]; waiting time before next start up when fault occurs */
#define MC_PRESSURE_RELAX_DURATION_ATTEMPT_STARTUP  15.0  /* [s]; waiting time before next startup when last startup fails (Not fault); MR20150331_B */
#define MC_START_UP_ATTEMPTS		            	3     /* Number of start-up attempts before it goes to startup failure fault */


/* Over voltage protection threshold */
#define MC_OVERVOLT_LIMIT                      410.0 /* [Volt] */

/* Under voltage protection threshold */
#define MC_UNDERVOLT_LIMIT                     260.0  /* [Volt] */
#define UNDER_VOLT_DELAY_PROTECT_TIME          0.125 /* [s]; maximum value = 0xFFFFFFFF/MC_FAST_CONTROL_LOOP_FREQ */

/* Software over current threshold */
#define SOFTWARE_OC_TH_SHRK					   3.3 	 /* [A]; PWM is disabled when phase current reaches this value */
#define I_LENGTH_MA32_NUMBER				   4




/* Led indications for faults */
/*--------------------------------------------------------------------------------------
 *  Take HOC for instance:
 *  
 *  	Total indication time is 
 *  
 *  PATTERN_TIMES_HOC*(BLINK_TIMES_HOC*BLINK_DURATION_HOC+PATTERN_SPACE_HOC)
 *  
 *  Make sure this value is less than MC_PRESSURE_RELAX_DURATION_RUNTOFAULT*60, so
 *  the maximal pattern times is:
 *  
 *   MC_PRESSURE_RELAX_DURATION_RUNTOFAULT*60/(BLINK_TIMES_HOC*BLINK_DURATION_HOC+PATTERN_SPACE_HOC)
 * 
 * -------------------------------------------------------------------------------------
 */

// Hardware Over Current
#define BLINK_TIMES_HOC						   3	/* [n/a]; Led blinking times */
#define BLINK_DURATION_HOC					   0.5  /* [S]; Led on/off duration during blinking */
#define PATTERN_SPACE_HOC					   2.0  /* [S]; Time Duration between patterns */
#define PATTERN_TIMES_HOC					   (uint16_t)(MC_PRESSURE_RELAX_DURATION_RUNTOFAULT*60/(BLINK_TIMES_HOC*BLINK_DURATION_HOC+PATTERN_SPACE_HOC))	/* [n/a]; Led blinking pattern repetition times */

#define BLINK_TIMES_SOC						   4	/* [n/a]; Led blinking times */
#define BLINK_DURATION_SOC					   0.5  /* [S]; Led on/off duration during blinking */
#define PATTERN_SPACE_SOC					   2.0  /* [S]; Time Duration between patterns */
#define PATTERN_TIMES_SOC					   (uint16_t)(MC_PRESSURE_RELAX_DURATION_RUNTOFAULT*60/(BLINK_TIMES_SOC*BLINK_DURATION_SOC+PATTERN_SPACE_SOC))	/* [n/a]; Led blinking pattern repetition times */

#define BLINK_TIMES_UV						   5	/* [n/a]; Led blinking times */
#define BLINK_DURATION_UV					   0.5  /* [S]; Led on/off duration during blinking */
#define PATTERN_SPACE_UV					   2.0  /* [S]; Time Duration between patterns */
#define PATTERN_TIMES_UV					   (uint16_t)(MC_PRESSURE_RELAX_DURATION_RUNTOFAULT*60/(BLINK_TIMES_UV*BLINK_DURATION_UV+PATTERN_SPACE_UV))	/* [n/a]; Led blinking pattern repetition times */

#define BLINK_TIMES_OV						   6	/* [n/a]; Led blinking times */
#define BLINK_DURATION_OV					   0.5  /* [S]; Led on/off duration during blinking */
#define PATTERN_SPACE_OV					   2.0  /* [S]; Time Duration between patterns */
#define PATTERN_TIMES_OV					   (uint16_t)(MC_PRESSURE_RELAX_DURATION_RUNTOFAULT*60/(BLINK_TIMES_OV*BLINK_DURATION_OV+PATTERN_SPACE_OV))	/* [n/a]; Led blinking pattern repetition times */

#define BLINK_TIMES_OP						   7	/* [n/a]; Led blinking times */
#define BLINK_DURATION_OP					   0.5  /* [S]; Led on/off duration during blinking */
#define PATTERN_SPACE_OP					   2.0  /* [S]; Time Duration between patterns */
#define PATTERN_TIMES_OP					   (uint16_t)(MC_PRESSURE_RELAX_DURATION_RUNTOFAULT*60/(BLINK_TIMES_OP*BLINK_DURATION_OP+PATTERN_SPACE_OP))	/* [n/a]; Led blinking pattern repetition times */

#define BLINK_TIMES_OL						   8	/* [n/a]; Led blinking times */
#define BLINK_DURATION_OL					   0.5  /* [S]; Led on/off duration during blinking */
#define PATTERN_SPACE_OL					   2.0  /* [S]; Time Duration between patterns */
#define PATTERN_TIMES_OL					   (uint16_t)(MC_PRESSURE_RELAX_DURATION_RUNTOFAULT*60/(BLINK_TIMES_OL*BLINK_DURATION_OL+PATTERN_SPACE_OL))	/* [n/a]; Led blinking pattern repetition times */

#define BLINK_TIMES_STALL					   2	/* [n/a]; Led blinking times */
#define BLINK_DURATION_STALL				   0.5  /* [S]; Led on/off duration during blinking */
#define PATTERN_SPACE_STALL					   2.0  /* [S]; Time Duration between patterns */
#define PATTERN_TIMES_STALL					   (uint16_t)(MC_PRESSURE_RELAX_DURATION_RUNTOFAULT*60/(BLINK_TIMES_STALL*BLINK_DURATION_STALL+PATTERN_SPACE_STALL))	/* [n/a]; Led blinking pattern repetition times */

#define BLINK_TIMES_OTHER					   1	/* [n/a]; Led blinking times */
#define BLINK_DURATION_OTHER				   0.15  /* [S]; Led on/off duration during blinking */
#define PATTERN_SPACE_OTHER					   0.01  /* [S]; Time Duration between patterns */
#define PATTERN_TIMES_OTHER					   (uint16_t)(MC_PRESSURE_RELAX_DURATION_RUNTOFAULT*60/(BLINK_TIMES_OTHER*BLINK_DURATION_OTHER+PATTERN_SPACE_OTHER))	/* [n/a]; Led blinking pattern repetition times */

/* Bootstrap capacitor charging duty and duration */
#define BOOTSTRAP_CHARGE_DUTY	               0.95  /* This duty is used to charge bootstrap capacitors before alignment */
#define MC_BOOTSTRAP_CHARGE					   0.1//0.2	 /* [s];Duration of charging bootstrap, must be less than alignment time */

/* Alignment  */
#define MC_ALIGN_CURRENT                  	   0.5 	  /* [A]; Alignment current */
#define MC_ALIGN_SPEED				  	  	   0.0    /* [RPM]; Alignment speed in RPM */
#define VOLTAGE_ALIGN_MAX					   60.0   /* [Volt]; Output limit of current controllers during alignment */
#define MC_ALIGN_VOLT_MAX                      15.0   /* [Volt]; Max. voltage for alignment, NOT USED */
#define MC_ALIGN_VOLT_RAMP				  	   4.0    /* [Volt/s]; Alignment voltage ramp, NOT USED */
#define MC_ALIGN_CURRENT_RAMP				   8.0 	  /* [A/S]; */
#define MC_ALIGN_POSITION_1					   0.0    /* [Degree] */
#define MC_ALIGN_POSITION_2					   -90.0  /* [Degree]; Aligned position where current is put on D-axis */
#define MC_ALIGN_STAGE_1_TIME				   0.25   /* [s]; The time duration for MC_ALIGN_POSITION_1.This value must be smaller than MC_DURATION_TASK_ALIGN */

/* Start up speed and current parameters */
#define MC_START_UP_SPEED_MAX				   500.0   /* [RPM]; Desired final speed in open-loop start up, then merge into speed closed loop */
#define MC_START_UP_SPEED_RAMP				   400.0  /* [RPM/s]; Start-up speed ramp */
#define MC_START_UP_CURRENT_RAMP			   8.0     /* [A/s]; Start-up current ramp */
#define MC_START_UP_CURRENT_PULL_OUT		   0.30    /* [A]; Speed starts to increase when current reaches this value */
#define MC_START_UP_CURRENT_MAX				   0.5     /* [A]; Start-up current max value */
#define MC_START_UP_MERGE_STEP				   0.01    /* [N/A]; Step used during merging to calculate merging coefficient */

/* Speed ramp during closed loop running */
#define SPEED_UP_RAMP_ASR                      1000.0    /* [RPM/s]; Speed ramp for speed up */
#define SPEED_DOWN_RAMP_ASR                    1000.0    /* [RPM/s]; Speed ramp for speed down */
#define SPEED_STOP_THRHLD					   2000.0     /* [RPM]; Stop the motor when speed ramp reaches this value and speed command is set to 0 */

/* State duration */
#define MC_DURATION_TASK_DEFAULT        		0
#define MC_DURATION_TASK_ALIGN           		0.8	  /* [s]; Duration of alignment */
#define MC_DURATION_TASK_CALIB           		0.2   /* [s]; Duration of current calibration */
#define MC_DURATION_TASK_INTER_RUN				2.0	  /* [s]; Duration of free-wheel when it's transferred from Start-up state */
#define MC_DURATION_TASK_FREE_WHEEL				5.0	  /* [s]; Maximal duration of free-wheel when it's transferred from Spin state */
#define MC_DURATION_TASK_FAULT_RELEASE			2.0	  /* [s]; Extra duration of fault state after fault disappears */                    


/* Relevant Field-weakening parameters */
#define MC_SPEED_FW_ON                      2000.0 /* Speed when the field weakening is allowed */
// Field-weakening PI controller 
#define MC_PI_FW_P_GAIN                     0.0125
#define MC_PI_FW_I_GAIN                     0.00375
#define MC_PI_FW_OUTPUT_LIMIT               2.0 	/* Maximal d-axis current in [A] */

/* Speed controller parameters;Parallel type */    
#define MC_PI_SPEED_P_GAIN					0.3 		  /* Speed controller proportional gain */
#define MC_PI_SPEED_I_GAIN					0.0175  	 	  /* Speed controller integral gain */
#define MC_PI_SPEED_OUTPUT_LIMIT			3.0           /* [A]; Speed controller output limit in Amps, used in initialisation only */
#define MC_I_MAX							3.0		      /* [A]; Max current used for flux weakening, used as speed controller output limit during running */

/* ACR output limit (Voltage limit) */
#define MC_PI_D_ACR_OUT_LIMIT		        170.0	     /* [Volt]; D current controller output limit in Volt */
#define MC_PI_Q_ACR_OUT_LIMIT		        170.0	     /* [Volt]; Q current controller output limit in Volt */
#define CLOOP_LIMIT                     	FRAC16(0.96) /* Maximal duty output */

/* Constant */
#define PI           			           		3.1415926 /* [N/A] */

/* Current Channels */
#define IA_ANA					           		1     /* Channel number in CLIST register */
#define IB_ANB				               		10     /* Channel number in CLIST register */
#define IC_ANA					           		6     /* Channel number in CLIST register */
#define IC_ANB				               		15    /* Channel number in CLIST register */
#define UDC_ANA									3
#define IPM_TEMP_ANB							8

/*=============================================================   Auto-generated Macros ================================================================*/
//-------------------------------------------------------------------------------------------------------------------------------------------------------
					
					
/* Motor basic parameters */					
//------------------------------------------------------------------------------------------------------------------------------------------					
#define	STATOR_R		1.2	 /* [ohm] */	
#define	LD_INDUCTANCE		0.00376	 /* [Henry] */	
#define	LQ_INDUCTANCE		0.00385	 /* [Henry] */	
#define	MC_POLE_PAIRS		4	 /* [pairs] */	
					
/* Based value */					
//------------------------------------------------------------------------------------------------------------------------------------------					
#define	U_DCB_MAX		433	 /* [V] */	
#define	U_MAX		249.9926666	 /* [V] */	
#define	I_MAX		8	 /* [A] */	
#define	E_MAX		250	 /* [V] */	
#define	N_MAX		8000	 /* [RPM] */	
#define	POWER_MAX		2999.911999	 /* [Watt] */	
					
/* Time scaling */					
//------------------------------------------------------------------------------------------------------------------------------------------					
#define 	MC_PWM_CLK_FREQ		100000000	 /*  [Hz] */	
#define 	MC_FAST_CONTROL_LOOP_FREQ		16000	 /*  [Hz] */	
#define 	MC_SLOW_CONTROL_LOOP_FREQ		1000	 /*  [Hz] */	
#define 	SPEED_LOOP_CNTR		16		
					
/*ACR parameter*/					
//------------------------------------------------------------------------------------------------------------------------------------------					
/*Damping Coefficient_D&Q_ACR=	0.85	*/			
/*bandWidth=	400	[Hz]*/			
#define	D_KP_GAIN	ACC32(	0.475689087	)	
#define	D_KI_GAIN	ACC32(	0.047501824	)	
					
#define	Q_KP_GAIN	ACC32(	0.487994438	)	
#define	Q_KI_GAIN	ACC32(	0.048638836	)	
					
/*DQ Observer parameters*/					
//------------------------------------------------------------------------------------------------------------------------------------------					
// Damping Coefficient =	0.85				
// BandWidth =	500	[Hz]			
#define	I_SCALE		ACC32(	0.980443286	)
#define	U_SCALE		ACC32(	0.5092745	)
#define	E_SCALE		ACC32(	0.509289439	)
#define	WI_SCALE		ACC32(	0.210258696	)
#define	BEMF_DQ_KP_GAIN	ACC32(	0.604193917	)	
#define	BEMF_DQ_KI_GAIN	ACC32(	0.074219423	)	
					
/* Tracking Observer for DQ -observer */					
//------------------------------------------------------------------------------------------------------------------------------------------					
// Damping Coefficient =	0.85				
// Bandwidth =	40	[Hz]			
#define	TO_KP_GAIN	FRAC16(	0.51	)	
#define	TO_KP_SHIFT	(	-2	)	
#define	TO_KI_GAIN	FRAC16(	0.603185779	)	
#define	TO_KI_SHIFT	(	-9	)	
#define	TO_THETA_GAIN	FRAC16(	0.533333333	)	
#define	TO_THETA_SHIFT	(	-3	)	
					
					
/* Filters */					
//------------------------------------------------------------------------------------------------------------------------------------------					
// Udc bus voltage, cutoff freq = 	100	[Hz]	Ts =	0.0000625	[s]
#define	MC_FILTER_UDCBUS_B0	0.019249696			
#define	MC_FILTER_UDCBUS_B1	0.019249696			
#define	MC_FILTER_UDCBUS_A1	-0.961500608			
// Speed_DQ obsrv , cutoff freq = 	30	[Hz]	Ts =	0.0000625	[s]
#define	MC_FILTER_SPEED_EST_B0	0.005853107			
#define	MC_FILTER_SPEED_EST_B1	0.005853107			
#define	MC_FILTER_SPEED_EST_A1	-0.988293785			
// FW error , cutoff freq = 	50	[Hz]	Ts =	0.001	[s]
#define	MC_FILTER_FW_B0	0.136667899			
#define	MC_FILTER_FW_B1	0.136667899			
#define	MC_FILTER_FW_A1	-0.726664203			



#endif /* _PMSM_SPEEDVECTORCTRL_H_ */
