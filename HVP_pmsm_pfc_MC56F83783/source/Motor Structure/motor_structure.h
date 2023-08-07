/******************************************************************************
* 
 * Copyright (c) 2008 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause                     
*
******************************************************************************* 
*
* Any support, information, and technology (¡°Materials¡±) provided by NXP are provided AS IS, 
* without any warranty express or implied, and NXP disclaims all direct and indirect liability 
* and damages in connection with the Material to the maximum extent permitted by the applicable law. 
* NXP accepts no liability for any assistance with applications or product design. Materials may 
* only be used in connection with NXP products. Any feedback provided to NXP regarding the Materials 
* may be used by NXP without restriction.
*
***************************************************************************//*!


*******************************************************************************
*
* Motor control structure.
*
******************************************************************************/
#ifndef _MCSTRUC_H_
#define _MCSTRUC_H_

/******************************************************************************
* Includes
******************************************************************************/
#include "cpu.h"
#include "gflib.h"
#include "gmclib.h"
#include "gdflib.h"
#include "amclib.h"
#include "mlib.h"/*MR20150306_A*/
#include "PMSM_SpeedVectorCtrl.h"
/******************************************************************************
* Constants
******************************************************************************/

/******************************************************************************
* Macros 
******************************************************************************/

/******************************************************************************
* Types
******************************************************************************/
typedef enum {
    FAST           = 0,
    SLOW           = 1,
} MCSTRUC_CONTROL_LOOP_T;         /* Loop */

//typedef struct
//{
//	frac32_t f32I_1;
//	frac16_t f16IGain;
//	int16_t  i16IGainSh;
//}GFLIB_INTEGRATOR_T;

typedef union
{
    uint32_t R;
    struct
    {
        uint32_t StartUpFail            	: 1;   /* Start-up fail */
        uint32_t OverLoad               	: 1;   /* Overload Flag */
        uint32_t IDcBusOver       		: 1;   /* OverCurrent fault flag */
        uint32_t UDcBusUnder      		: 1;   /* Under voltage fault flag */
        uint32_t UDcBusOver       		: 1;   /* Over voltage fault flag */
        uint32_t IPMTempOver				: 1;   /* IPM over temperature */	
        uint32_t PowerOver               : 1;   /* Motor over power flag; MR20150324_A */
        uint32_t OpenPhase               : 1;   /* Open phase fault; MR20150331_A */
        uint32_t OverCurrentSoft			: 1;   /* Over current software protection flag */

    } B;
} MCSTRUC_FAULT_STATUS_T;    /* Application fault status user type*/


typedef union
{
    uint32_t R;
    struct
    {
        uint32_t StartUpFailRecord           : 1;   /* Start-up fail */
        uint32_t OverLoadRecord              : 1;   /* Overload Flag */
        uint32_t IDcBusOverRecord       		: 1;   /* OverCurrent fault flag */
        uint32_t UDcBusUnderRecord      		: 1;   /* Under voltage fault flag */
        uint32_t UDcBusOverRecord       		: 1;   /* Over voltage fault flag */
        uint32_t IPMTempOverRecord			: 1;   /* IPM over temperature */	
        uint32_t PowerOverRecord             : 1;   /* Motor over power flag; MR20150324_A */
        uint32_t OpenPhaseRecord             : 1;   /* Open phase fault; MR20150331_A */
        uint32_t OverCurrentSoftRecord		: 1;   /* Over current software protection flag */
        
    } B;
} MCSTRUC_FAULT_RECORD_T;    /* Application fault status user type*/

typedef struct
{
	frac16_t					f16UDcBusOver;		/* DC bus over voltage level */
	frac16_t					f16UDcBusUnder;		/* DC bus under voltage level */
	frac16_t					f16SpeedOver;		/* Over speed level, not used */
	frac16_t					f16SpeedUnder;		/* Under speed level, not used */
	frac16_t					f16IPMTempOver;		/* IPM over temperature level */
	frac16_t                  f16PowerOver;       /* Motor over power threshold MR20150324_A */
	uint16_t 				uw16OpenPhaseStartupAttempCntr;		/* Counter to count times of start-up when open-phase occurs; MR20150427_E */
	frac16_t					f16SoftwareOC;		/* Software over current threshold */
} MCSTRUC_FAULT_THRESHOLDS_T;


typedef struct
{
    uint16_t					uw16Ph0;		/* Phase 0 channel number */
    uint16_t					uw16Ph1;		/* Phase 1 channel number */
} MCSTRUC_ADC_2_PHASE_ASSIGNMENT_T;


typedef struct
{
    frac32_t						f32Position;			/* Position of field at alignment */
    frac32_t						f32U;					/* D voltage at alignment, not used */
    frac32_t						f32Speed;				/* Speed of field at alignment */
    frac32_t						f32UStep;				/* D voltage ramp at alignment, not used */    
    frac16_t						f16UMax;				/* Max D voltage at alignment, not used */
    uint16_t						uw16TimeAlignment;		/* Alignment time duration */
    //--------------------- Current ramp variables -----------------------------
    frac16_t						f16IMax;				/* Max D current at alignment */
    GFLIB_RAMP_T_F32					sIRamp;
    frac32_t						f32IAlignment;			/* Ramp output */
    frac16_t					    f16IAlignment;			/* Not used */
    //--------------------------------------------------------------------------
} MCSTRUC_ALIGNMENT_T;

typedef struct
{
    frac16_t    	f16PhA_ANA;     /* DC offset phase A from converter A */
    frac16_t    	f16PhB_ANB;     /* DC offset phase B from converter B */
    frac16_t    	f16PhC_ANA;     /* DC offset phase C from converter A */
    frac16_t    	f16PhC_ANB;     /* DC offset phase C from converter B */
    GMCLIB_2COOR_T_F16 				sIRaw;							  /* Stores the latest raw current values from ADC */
    GMCLIB_2COOR_T_F16				sIOffsets; /* Store the offsets of sampled channels in current PWM period */
    GDFLIB_FILTER_MA_T_A32 sFilterMA_phA_ANA;
    GDFLIB_FILTER_MA_T_A32 sFilterMA_phB_ANB;
    GDFLIB_FILTER_MA_T_A32 sFilterMA_phC_ANA;
    GDFLIB_FILTER_MA_T_A32 sFilterMA_phC_ANB;
    MCSTRUC_ADC_2_PHASE_ASSIGNMENT_T sIChnl; /* The updated two channels for next PWM period current sampling */
} MCSTRUC_ADC_CHN_OFFSET_T;

typedef struct
{
    GFLIB_CTRL_PI_P_AW_T_A32  sIdPiParams;		/* Id PI controller parameters */
    GFLIB_CTRL_PI_P_AW_T_A32  sIqPiParams;		/* Iq PI controller parameters */
    bool_t 					        i16IdPiSatFlag;		/* Id PI controeller saturation flag */
	bool_t 					        i16IqPiSatFlag;		/* Iq PI controeller saturation flag */
	
    GDFLIB_FILTER_IIR1_T_F32 			sUDcBusFilter;		/* Dc bus voltage filter */
	volatile frac16_t							f16UDcBus;			/* DC bus votlage */
	volatile frac16_t							f16UDcBusFilt;		/* Filtered DC bus votlage */
	
    GMCLIB_3COOR_T_F16    			sIABC;				/* Measured 3-phase current */
    GMCLIB_2COOR_ALBE_T_F16	sIAlBe;   			/* Alpha/Beta current */
    GMCLIB_2COOR_DQ_T_F16			sIDQ;     			/* DQ current */
    GMCLIB_2COOR_DQ_T_F16			sIDQReq;  			/* DQ required current */
    GMCLIB_2COOR_DQ_T_F16			sIDQReqZc;  		/* DQ required current after zero cancelation */
    GMCLIB_2COOR_DQ_T_F16			sIDQError;  		/* DQ current error */
    GMCLIB_3COOR_T_F16    			sDutyABC;			/* Applied duty cycles ABC */
    GMCLIB_2COOR_ALBE_T_F16	sUAlBeReq;   		/* Required Alpha/Beta voltage */
    GMCLIB_2COOR_ALBE_T_F16	sUAlBeComp;	 		/* Compensated to DC bus Alpha/Beta volate */
    GMCLIB_2COOR_DQ_T_F16			sUDQReq;     		/* Required DQ voltage */
    GMCLIB_2COOR_DQ_T_F16			sUDQController; 	/* Required DQ voltage */
	GMCLIB_2COOR_SINCOS_T_F16					sAnglePosEl; 		/* Electrical position sin/cos (at the moment of PWM current reading) */
	GMCLIB_2COOR_SINCOS_T_F16					sAnglePosElUpdate;	/* Compensated electrical position sin/cos (at the moment of PWM update) */

	uint16_t 						uw16SectorSVM;		/* SVM sector */
	frac16_t							f16DutyCycleLimit;	/* Max. allowable dutycycle in frac */

    MCSTRUC_ALIGNMENT_T				sAlignment;			/* Alignment struction params */
    MCSTRUC_ADC_CHN_OFFSET_T        sAllChnlOffsets;
    
	bool_t							bOpenLoop;			/* Current control loop is open */
	bool_t							bUseMaxBus;			/* Indicate if calculating the max. possible DQ current controllers' output limits based on dc bus voltage ,not used */
} MCSTRUC_FOC_PMSM_T;

typedef struct
{
    GDFLIB_FILTER_IIR1_T_F32 			sFwErrorFilter;		/* Field weakening error filter */				
	frac16_t							f16FwError;			/* Raw field weakening error */				
	frac16_t							f16FwErrorFilt;		/* Filtered field weakening error */	   
	frac16_t							f16UFwError;		/* Voltage error */
	frac16_t							f16IFwError;		/* Current error */				
				
	frac16_t							f16SpeedFwOn;		/* Speed when field weakening is turned on */
	frac16_t							f16ILimit;			/* Max. allowable -D current (output from the PI controller) */
	
	GFLIB_CTRL_PI_P_AW_T_A32  sFwPiParams;		/* FW PI controller parameters */
	bool_t 					        i16FwPiSatFlag;		/* Field weakening PI controller saturation flag */
} MCSTRUC_FOC_FW_T;

typedef struct
{
    frac16_t							f16Speed;			/* Speed */
    GFLIB_RAMP_T_F32                  s32SpeedRampParams; /* 32bit speed ramp structure MR20150317_A*/
    frac32_t                          f32SpeedRamp;       /* Required speed (ramp output,32bit )MR20150317_A*/
	frac16_t							f16SpeedRamp;		/* Required speed (ramp output,16bit ) */
	frac16_t							f16SpeedReq;		/* Required speed (ramp input) */
	frac16_t							f16SpeedCmd;		/* Speed command (entered by user or from control board) */
	frac16_t							f16SpeedError;		/* Speed error */
	
    GFLIB_CTRL_PI_P_AW_T_A32  sSpeedPiParams;		/* Speed PI controller parameters */
    bool_t 					        i16SpeedPiSatFlag;	/* Speed PI controller saturation flag */

	bool_t							bOpenLoop;			/* Flag indicating speed control loop is open */	
	bool_t	 						bStopFlag;	 /* Flag indicating whether motor is stopped immediately or stopped after a delay */
	uint16_t	 						uw16StopDelayCnt;   /* A counter to count the delay time to stop */
} MCSTRUC_SPEED_T;



typedef struct
{
	GFLIB_INTEGRATOR_T_A32				sSpeedIntegrator;			/* Speed integrator structure to get position */
	frac16_t							f16PositionPredicted;		/* Position generated in open-loop */ 
	frac16_t                          f16PositionPredictedOld;    /* Position generated in last step */
	
	frac16_t							f16IStartUpRamp;			/* Start up ramp current, 16bit */
	frac32_t                          f32IStartUpRamp;            /* Start up ramp current, 32bit */
	frac32_t							f32IStartUpDesired;
	GFLIB_RAMP_T_F32					sIRamp32;
	frac16_t							f16IStartUpPullOut;			/* Pull-out current. Position is not accumulated until start up current exceeds this value */
	
	frac16_t							f16SpeedPredictedRamp;		/* Speed generated in open-loop, 16bit */
	frac32_t							f32SpeedPredictedRamp;		/* Speed generated in open-loop, 32bit */
	frac32_t							f32SpeedStartUpDesired;
	GFLIB_RAMP_T_F32					sSpdRamp32;
	
	uint16_t							uw16AttemptCntr;			/* Start-up attempt counter */
	uint16_t							uw16AttemptCntrMax;			/* Max permitted no. of start-up attempts */
	uint16_t							uw16TimeStartUpFreeWheel;	/* Free-wheel duration when sub-state transfers from start-up to freewheel */
	frac16_t							f16UdStartUpLimit;			/* Start-up D voltage limit */
	frac16_t							f16UqStartUpLimit;			/* Start-up Q voltage limit */
	uint16_t                         uw16CntrForOLStartup;		/* counter to define how many electrical revolutions during start up */	
	
	GMCLIB_2COOR_SINCOS_T_F16					sAngleMerge;				/* Sin&Cos during startup and merging */
	frac16_t							f16MergeCoeff;				/* Merging coefficient which varies from 0 to 1 during merging */
	frac16_t							f16MergeStep;				/* Step used to alter merging coefficient */
	frac16_t							f16PosMerge;				/* Position used during merging */
				
} MCSTRUC_EST_STARTUP_T;


typedef struct
{
	AMCLIB_BEMF_OBSRV_DQ_T_A32			sBemfObsrv;			/* BEMF observer in DQ */
    AMCLIB_TRACK_OBSRV_T_F32				sTo;				/* Tracking observer */
} MCSTRUC_BEMF_OBS_DQ_T;


typedef struct
{
	GMCLIB_2COOR_SINCOS_T_F16					sAnglePosEl; 			/* Electrical position sin/cos (at the moment of PWM current reading), not used */
	GMCLIB_2COOR_SINCOS_T_F16					sAnglePosElUpdate;		/* Compensated electrical position sin/cos (at the moment of PWM update) */
	GDFLIB_FILTER_IIR1_T_F32 			sSpeedEstFilter;		/* Estimated speed filter */
	MCSTRUC_EST_STARTUP_T			sStartUp;				/* Start-up structure */
    frac16_t							f16PositionEl;			/* Fractional electrical position */
    frac16_t							f16PositionElUpdate;	/* Compensated Fractional electrical position */
	frac16_t							f16SpeedEstimated;		/* Speed by BEMF and ATO */
	frac16_t							f16SpeedEstimatedFilt;	/* Speed by BEMF and ATO filtered */
	bool_t							bStartUp;				/* Start-up mode, indicating whether it's start-up or not */
	bool_t							bStartUpFail;			/* Start-up fail flag */
} MCSTRUC_POS_SPEED_EST_T;



/* PMSM FOC with BEMF observer in DQ */
typedef struct
{
	MCSTRUC_FAULT_STATUS_T				sFaultId;
	MCSTRUC_FAULT_STATUS_T				sFaultIdPending;
	MCSTRUC_FAULT_THRESHOLDS_T			sFaultThresholds;
	MCSTRUC_FOC_PMSM_T					sFocPMSM;
	MCSTRUC_FOC_FW_T					sFocFw;
	MCSTRUC_BEMF_OBS_DQ_T				sObserverDQ;
	MCSTRUC_POS_SPEED_EST_T				sPositionObsDQ;
	MCSTRUC_SPEED_T						sSpeed;
				  /* Stores the four original current offsets of ADC channels */
	uint16_t 							uw16CounterSlowLoop;		 /* Counter used to realize slow loop */
	uint16_t 							uw16DividerSlowLoop;		 /* Maximal value for the counter used to realize slow loop */
	uint16_t 							uw16CounterState;			 /* Counter used to time every sub-state */
	uint16_t 							uw16TimeFullSpeedFreeWheel;  /* Maximal duration of free-wheel when transferred from spin state */
	uint16_t								uw16TimeCalibration;		 /* Duration of calibration sub-state */
	uint16_t								uw16TimeFaultRelease;		 /* Extra duration of fault state when fault disappears */	
	frac16_t								f16IPMTemp;					 /* Temperature of IPM module */
	frac16_t                                f16Uac;
} MCSTRUC_FOC_PMSM_OBS_DQ_T;


typedef struct
{
	frac16_t f16BoundarySpeed; /* No over-load protection above this speed */
	frac16_t f16MinSpeed;      /* Overload occurs if real speed is below this speed */
	frac16_t f16MaxCurr;       /* Not used */
}MCSTRUC_OVER_LOAD_PROTECT_T; /* MR20150326_A */


/******************************************************************************
* Global variables
******************************************************************************/

/******************************************************************************
* Global functions
******************************************************************************/

extern void MCSTRUC_FocPMSMCurrentCtrl(MCSTRUC_FOC_PMSM_T *psFocPMSM);
extern void MCSTRUC_PMSMPositionObsDQ(MCSTRUC_FOC_PMSM_T *psFocPMSM, MCSTRUC_BEMF_OBS_DQ_T *psObserver, MCSTRUC_POS_SPEED_EST_T *psPosition);
extern void MCSTRUC_FocFieldWeakeningCtrl(MCSTRUC_FOC_PMSM_T *psFocPMSM, MCSTRUC_FOC_FW_T *psFocFw, MCSTRUC_SPEED_T *psSpeed);
extern void MCSTRUC_PMSMOpenLoopStartUp(MCSTRUC_FOC_PMSM_T *psFocPMSM, MCSTRUC_POS_SPEED_EST_T *psPosition, MCSTRUC_SPEED_T *psSpeed);
extern void MCSTRUC_AlignmentPMSM(MCSTRUC_FOC_PMSM_T *psFocPMSM);
extern void MCSTRUC_ChlOffsetCalib(MCSTRUC_FOC_PMSM_T *psFocPMSM);

/******************************************************************************
* Inline functions
******************************************************************************/

#endif /* _MCSTRUC_H_ */
