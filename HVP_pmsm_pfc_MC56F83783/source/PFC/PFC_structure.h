/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef PFC_STRUCTURE_H_
#define PFC_STRUCTURE_H_

#include "gflib.h"
#include "gdflib.h"
#include "mlib.h"

/******************************************************************************
* Types
******************************************************************************/


typedef struct
{
    GFLIB_CTRL_PI_P_AW_T_A32        sIPiParams;     	/* Current PI controller parameters */
    GDFLIB_FILTER_MA_T_A32          sIOffsetFilter;     /* current offset filter */
    frac16_t              			f16Duty;			/* Applied duty cycle */
	frac16_t                        f16IFdbck; 	       	/* Real current on the MOS */
	frac16_t                        f16ICorrect;        /* Correct current(actual average current) on the MOS */
	frac16_t						f16IRef;  		    /* Required current reference */
	frac16_t						f16IError;			/* Current error */
	frac16_t                        f16IOffset;         /* current offset */
	frac16_t						f16CompCoeff;       /* Current compensation for DCM */
	bool_t                          bStopIntegFlag;     /* PI controller integration stop flag */ 
} PFCSTRUC_CURRENT_CTRL_T;

typedef struct
{			
	GFLIB_CTRL_PI_P_AW_T_A32        sUDcBusPiParams;	/* Dc bus voltage PI controller parameters */
	GFLIB_RAMP_T_F16				sUDcBusRampParams; 	/* Dc bus voltage ramp parameters */
	GDFLIB_FILTER_IIR1_T_F32 		sUDcBusFilter;		/* Dc bus voltage filter */
	frac16_t						f16UDcBusReq;		/* Required DC bus voltage (ramp output) */
	frac16_t						f16UDcBusCmd;		/* DC bus voltage command (entered by user or master layer) */
	frac16_t						f16UDcBus;			/* Raw Sampled DC bus voltage */
	frac16_t						f16UDcBusFilt;		/* Filtered DC bus voltage */
	frac16_t						f16UDcBusError;		/* DC bus voltage error */
	frac16_t                        f16UDcBusErrLim;    /* DC bus voltage error limit */
	frac16_t						f16VoltDiff_UDc_UIn;/* Voltage difference between DC bus voltage and input AC voltage */

	frac16_t						f16UDcBusCtrlOut;   /* DC bus voltage PI controller output */
	acc32_t 						a32IRef;  		    /* Required current reference */
	frac16_t                        f16UDcBusRelayOn;   /* DC bus voltage threshold to turn on the relay */
	frac16_t                        f16UDcBusRelayOff;  /* DC bus voltage threshold to turn off the relay */
	frac16_t                        f16UDcBusBurstOn;   /* Start PFC operation when output voltage is less than this value in light-load mode */
	frac16_t                        f16UDcBusBurstOff;  /* Stop PFC operation when output voltage is larger than this value in light-load mode */
	bool_t                          bStopIntegFlag;     /* PI controller integration stop flag */ 
} PFCSTRUC_VOLTAGE_CTRL_T;

typedef struct
{
	frac16_t						f16Sine; 					/* Generated sine wave based on input voltage phase */ 
	frac16_t						f16Phase; 					/* Electrical phase of the voltage */ 
	uint16_t                        u16FallOkCnt;               /* A counter that increases when input voltage is below threshold when the voltage is falling */
	uint16_t                        u16RiseOkCnt;               /* A counter that increases when input voltage is above threshold when the voltage is rising */
	uint16_t						u16CntThrsd2Zero;		    /* A counter that records the time when input voltage falls from threshold to zero */
	uint16_t                        u16TimeCnt;			        /* A counter to record the elapsed time within an input voltage period */
	uint16_t						u16Period;					/* Input voltage period duration, which is derived from u16TimeCnt */
	uint16_t						u16PeriodFilt;				/* Filtered period duration */
	frac16_t						f16Freq;					/* Reciprocal of filtered period,Q1.15 format,which represents the value 1 divided by filtered period */
	GDFLIB_FILTER_MA_T_A32          sPeriodFilter;              /* A filter for the period */
	
	frac16_t						f16UInThreshold;			/* Rectified voltage threshold for phase detect, which is a value near zero */
	bool_t							bFallThrsdDetEn;			/* A control switch to enable searching for threshold when input voltage is falling */
	bool_t							bRiseThrsdDetEn;			/* A control switch to enable searching for threshold when input voltage is rising */
} PFCSTRUC_PHASE_DETECT_T;

typedef struct
{
	GDFLIB_FILTER_IIR1_T_F32   		sFilter;     			/* Input voltage filter */
	frac16_t						f16UIn;					/* Input voltage */
	frac16_t                        f16UInFilt;             /* Filtered input voltage */
	frac16_t                        f16UInFilt_1;           /* Filtered input voltage of last step */
	frac16_t                        f16UInMaxTemp;          /* Temporary max value of input during peak value detection */
	frac16_t                        f16UInMax;              /* Final detected input voltage max value */

	frac16_t                        f16UInMaxSquare;        /* Square of max input voltage */
	frac16_t                        f16UInMaxSquInv;        /* Reciprocal of the square of max input voltage */
	frac16_t                        f16UInMaxInv;           /* Reciprocal of max input voltage */
	
	uint16_t                        u16UpCnt;
	uint16_t                        u16DownCnt;
	uint16_t                        u16VoltagePeakCnt;		/* A counter recording how many times peak voltage has been detected.
	                                                           It stops increasing after it reaches certain value. */
	bool_t                          bSeveralPeaksDetectedFlag;        /* This flag will set after peak voltage has been detected for certain times */  
	bool_t                          bPeakDetectEn;      	/* A control switch that decides whether to detect the peak voltage or not */
	bool_t                          bPeakFlag;              /* A flag indicating that a peak value has been detected */
} PFCSTRUC_INPUT_VOLTAGE_T;

typedef struct
{
	uint16_t             RelayFlag:1;
	uint16_t             InterleaveFlag:1;
	uint16_t             BrakeFlag:1;
	uint16_t             PWM_enable:1;
	uint16_t             DCMFlag:1;
	uint16_t             Reserved:11;
} PFCSTRUC_FLAG_T;

typedef union
{
    uint16_t R;
    struct
    {
    	uint16_t UDcBusOver            	: 1;   /* DC bus over voltage */
    	uint16_t UDcBusUnder            : 1;   /* DC bus under voltage */
    	uint16_t UInOver       			: 1;   /* AC input over voltage */
    	uint16_t UInUnder      			: 1;   /* AC input under voltage */
    	uint16_t IPh1Over       		: 1;   /* Phase 1 PFC over current */
    	uint16_t IPh2Over				: 1;   /* Phase 2 PFC over current */	
    	uint16_t UInFreqOver            : 1;   /* AC input over frequency */
    	uint16_t UInFreqUnder           : 1;   /* AC input under frequency */
        uint16_t HW_UDcBusOver			: 1;   /* DC bus over voltage - HW protection */
        uint16_t HW_IOver			    : 1;   /* Phase 1or2 over current - HW protection */
    } B;
} PFCSTRUC_FAULT_STATUS_T;    /* Application fault status user type*/

typedef struct
{
	frac16_t					f16UDcBusOver;		/* DC bus over voltage level */
	frac16_t					f16UDcBusUnder;		/* DC bus under voltage level */
	frac16_t					f16UInOver;			/* Input over voltage level */
	frac16_t					f16UInUnder;		/* Input under voltage level */
	frac16_t					f16FreqUInOver;		/* Input voltage over frequency  level */
	frac16_t					f16FreqUInUnder;	/* Input voltage under frequency level */
	frac16_t                    f16IInOver;         /* Input over current level */
} PFCDEF_FAULT_THRESHOLDS_T;

typedef struct
{	
	PFCDEF_FAULT_THRESHOLDS_T	sFaultThresholds;
	PFCSTRUC_CURRENT_CTRL_T		sICtrlPh1;
	PFCSTRUC_CURRENT_CTRL_T		sICtrlPh2;
	PFCSTRUC_VOLTAGE_CTRL_T		sUCtrl;
	PFCSTRUC_PHASE_DETECT_T		sPhaseDetector;
	PFCSTRUC_INPUT_VOLTAGE_T    sUInPeakDetection;
	PFCSTRUC_FLAG_T             sFlag;

	frac16_t                    f16DutyCCM;
	frac16_t                    f16DutyDCM;
	frac16_t                    f16DutyComp;
	PFCSTRUC_FAULT_STATUS_T 	FaultId; 	    	   /* Fault identification */ 
	PFCSTRUC_FAULT_STATUS_T 	FaultIdPending;    	   /* Fault identification pending*/ 
	uint16_t 					u16CounterTimeBase;   /* A counter to obtain a time base, such as 1ms */   

	uint16_t 					ui16CounterState;      /* A counter to time the duration of each necessary state */

} PFCDEF_DRIVE_T;


#endif /* PFC_STRUCTURE_H_ */
