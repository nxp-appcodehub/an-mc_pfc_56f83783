
/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


#ifndef PFC_DEF_H_
#define PFC_DEF_H_

/******************************************************************************
* Includes
******************************************************************************/
#include "PFC_structure.h"

/******************************************************************************
* Constants
******************************************************************************/

/******************************************************************************
* Macros 
******************************************************************************/
#define PLL                                 1
#define NANO_EDGE_EN                        1
#define BURST_MODE_EN                       1

/*************************************** AC input voltage filter ************************************************/
/* input voltage filter, Fs = 32k, Fc = 2kHz */
#define PFC_U_IN_IIR_B0                     FRAC32(0.16/2)
#define PFC_U_IN_IIR_B1                     FRAC32(0.16/2)
#define PFC_U_IN_IIR_A1                     FRAC32(-0.68/-2)

/*************************************** AC input voltage peak detection ************************************************/
#define PEAK_DET_INPUT_VOLTAGE_RISE_NUM               8       /* During input voltage peak detection, voltage is considered to be rising when it's been detected rising at least
                                                                 this value of times */
#define PEAK_DET_INPUT_VOLTAGE_FALL_NUM               8       /* During input voltage peak detection, voltage is considered to be falling when it's been detected falling at least
                                                                 this value of times */
#define PEAK_DET_INPUT_VOLTAGE_PEAK_NUM				  8       /* PFC starts to work at least this number of peaks have been detected */

/*************************************** AC input voltage phase detection ************************************************/
#define PHASE_DET_INPUT_VOLTAGE_PERIOD_FILTER_N       2       /* Input AC voltage period MA filter number, which represents 2^number averaging points */
#define PHASE_DET_INPUT_VOLTAGE_PERIOD_FILTER_POINTS  4       /* Input AC voltage period MA filter points = 2^PHASE_DET_INPUT_VOLTAGE_PERIOD_FILTER_N */
#define PHASE_DET_INPUT_VOLTAGE_FALL_CNT_THRSD        3       /* When input AC voltage is falling and the voltage is below threshold, a counter increases in every fast loop. 
                                                               The falling edge input voltage threshold is considered detected at this value of times. Must be larger than 1 */
#define PHASE_DET_INPUT_VOLTAGE_RISE_CNT_THRSD        PHASE_DET_INPUT_VOLTAGE_FALL_CNT_THRSD     
#define PFC_VIN_PHASEDETECT_TH                        65.0    /* Input voltage threshold for phase detection [V] */

/*************************************** Voltage&Current Scales ************************************************/
#define PFC_V_DCB_SCALE						433.0	/* MAX measurable DCB voltage [V] */
#define PFC_V_IN_SCALE						433.0	/* MAX measurable input voltage [V] */
#define PFC_I_SCALE							8.0  	/* MAX measurable current [A] */

/*************************************** Fault thresholds ************************************************/
#define PFC_VIN_OVERVOLT_LIMIT              375.0 	/* Max input voltage peak value [V] */
#define PFC_VIN_UNDERVOLT_LIMIT             120.0 	/* Min input voltage peak value [V] */
#define PFC_OVERVOLT_LIMIT               	420.0 	/* Max DC bus voltage [V] */
#define PFC_UNDERVOLT_LIMIT              	350.0 	/* Min DC bus voltage [V] */
#define PFC_IOVER_LIMIT                     4.5 	/* Max input current [A] */
#define PFC_VIN_FREQOVER_LIMIT              65.0    /* Max frequency on input voltage [Hz] */
#define PFC_VIN_FREQUNDER_LIMIT             45.0    /* Min frequency on input voltage [Hz] */

/*************************************** Relay control ************************************************/
#define PFC_U_DCB_RELAY_OFF                 110.0   /* Relay is turned off when DC bus voltage is below this value [V] */
#define PFC_COEFF_U_DCB_RELAY_ON            0.8     /* Relay can be turned on when DC bus voltage is above Vin_max multiplied with this coefficient */
#define PFC_VIN_THRESHOLD_RELAY_ON          30.0    /* After DC bus reaches a certain value, relay is turned on when AC input voltage is below this value [V] */

/*************************************** Voltage(DC bus) control ************************************************/
#define PFC_U_DCB_REF					    400.0   /* Required DC bus voltage [V] */
#define PFC_U_SOFTSTART_STEP                0.1     /* DC bus reference change step in soft start [V] */
#define PFC_U_DCB_BURSTON                   (PFC_U_DCB_REF-8.0)   /* In light load mode, current control is enabled when DC bus voltage falls below this value */
#define PFC_U_DCB_BURSTOFF                  (PFC_U_DCB_REF-1.0)   /* In light load mode, current control is disabled when DC bus voltage rises above this value */
//--------------------------------------------------------------------
//loop sample time                          = 0.0001[sec](10kHz)
//--------------------------------------------------------------------
#define PFC_U_DCB_IIR_B0                    FRAC32(0.42 / 2.0)//fc = 2kHz
#define PFC_U_DCB_IIR_B1                    FRAC32(0.42 / 2.0)
#define PFC_U_DCB_IIR_A1                    FRAC32(-0.16 / -2.0)

#define PFC_U_P_GAIN						ACC32(1.0)		/* Voltage controller proportional gain */
#define PFC_U_I_GAIN						ACC32(0.005)	/* Voltage controller integral gain */
#define PFC_U_UPPER_LIMIT   				FRAC16(1.0)  	/* Voltage controller upper limit, used in initialization */
#define PFC_U_LOWER_LIMIT    				FRAC16(0.0) 	/* Voltage controller lower limit, used in initialization */
#define LOOP_SCALE           				0.177 			/* 1.414/PFC_I_SCALE */     
#define LOW_CURRENT          				0.05  			/* Lower rms limit of input current reference, [A] */
#define HIGH_CURRENT         				3.0   			/* Upper rms limit of input current reference, [A] */
#define PFC_U_ERROR_LIMIT               	FRAC16(50.0/PFC_V_DCB_SCALE)  /* Output voltage error limit */

/**************************************** Current control ***********************************************/
#define PFC_I_OFFSET_MA_WINDOW              3           /* MA filter number for current offset, number of samples for averaging = 2^PFC_I_OFFSET_MA_WINDOW */ 
//---------------------------------------------------------------------
//loop sample time                          = 0.00003125[sec](32kHz)
//---------------------------------------------------------------------
#define PFC_I_P_GAIN1						ACC32(0.4) 		/* Current controller proportional gain */
#define PFC_I_I_GAIN1						ACC32(0.3) 		/* Current controller integral gain */
#define PFC_I_UPPER_LIMIT   				FRAC16(0.99) 	/* Current controller upper limit */
#define PFC_I_LOWER_LIMIT    				FRAC16(-0.99)   /* Current controller lower limit */
#define PFC_DCM_DUTY_COEFF   				ACC32(2.306)    /* (2*L*PFC_I_SCALE)/(Ts*V_DCB_SCALE) */

/**************************************** Application time base ***********************************************/
#define PFC_PWM_FREQ                        95.969                  /* PFC switch frequency [kHz] */
#define PWM_FREQ_VS_CTR_FREQ                3                       /* PWM frequency vs fast control frequency */
#define PFC_SLOW_LOOP_FREQ                  10                      /* PFC slow control loop frequency [kHz] */
#define PFC_FASTLOOP_FREQ                   (PFC_PWM_FREQ/PWM_FREQ_VS_CTR_FREQ*1000.0)  /* PFC fastloop frequency [Hz] */
#define TIMEBASE_1MS_FASTLOOP_CNTR          (PFC_PWM_FREQ/PWM_FREQ_VS_CTR_FREQ) 		/* Obtain 1ms in fast loop */
#define TIMEBASE_1MS_SLOWLOOP_CNTR          (PFC_SLOW_LOOP_FREQ)    /* Obtain 1ms in slow loop */
/* State duration based on time base 1ms*/
#define FAULT_RELEASE_DURATION              3000  /* [ms] */

#define PFC_IOFFSET_CALIB_DURATION          200   /* [ms] */
#define UDCBUS_RAMPUP_DURATION              200   /* [ms] When Dc bus voltage reaches certain value, delay this amount of time to turn on relay */
#define DELAY_BEFORE_CURRENT_CTRL           200   /* [ms] After relay is turned on, delay this amount of time before switching to RUN state */
#define BURST_OFF_MIN_DURATION              30    /* [ms] */

/* Feed forward compensation */
#define PFC_DC_BUS_FLUCTUATION               10.0   /* When DC bus voltage fluctuation is larger than this value, current reference will be compensated  */

/* Burst mode voltage thresholds */
#define PFC_DC_BUS_NORMAL_OVERSHOOT_DELTA_LOW     8.0   /* Enter burst mode when DC bus voltage is larger than command plus this value and voltage controller outputs lower limit [V] */
#define PFC_DC_BUS_NORMAL_OVERSHOOT_DELTA_HIGH    15.0  /* Enter burst mode when DC bus voltage is larger than command plus this value [V] */
#define PFC_DC_BUS_BURST_UNDER_DELTA              15.0  /* During burst mode, when DC bus voltage is lesser than (PFC_U_DCB_BURSTON-PFC_DC_BUS_BURST_UNDER_DELTA), go to normal sub-state */



/* Sets the fault defined by faultid in the faults variable */
#define PFC_FAULT_SET(faults, faultid) (faults |= faultid)

/* Clears the fault defined by faultid in the faults variable */
#define PFC_FAULT_CLEAR(faults, faultid) (faults &= ~faultid))

/* Check the fault defined by faultid in the faults variable, returns 1 or 0 */
#define PFC_FAULT_CHECK(faults, faultid) ((faults &  faultid)

/* Clears all faults in the faults variable */
#define PFC_FAULT_CLEAR_ALL(faults) (faults = 0)

/* Check if a fault is set in the faults variable, 0 = no fault */
#define PFC_FAULT_ANY(faults) (faults > 0)

#endif /* PFC_DEF_H_ */
