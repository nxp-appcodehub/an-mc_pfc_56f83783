
/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdint.h>
#include "fsl_device_registers.h"
#include "PFC_statemachine.h"
#include "hwcontrol.h"
#include "common_func.h"
/******************************************************************************
* Global variables
******************************************************************************/
/* PFC object including both hardware and software content */
PFCDEF_DRIVE_T     	 gsPFC_Drive;
PFC_RUN_SUBSTATE_T	 ePFC_StateRunSub;
PFCDRV_PWMVAL        gsPFC_PwmVal;


/******************************************************************************
* Static variables
******************************************************************************/
bool_t bPFC_RUN; /* PFC run/stop command */
static  bool_t bEnableCheckDCBusVunder;  /* enabling flag for checking Output voltage under*/
static  bool_t bEnableCheckInputVunder;  /* enabling flag for checking Input voltage under*/
bool_t  bPFCVarInit = 0; /* PFC variables init status flag */
/******************************************************************************
* Local functions
******************************************************************************/

/*------------------------------------
 * User state machine functions
 * ----------------------------------*/
static void PFC_StateFault(void);
static void PFC_StateInit(void);
static void PFC_StateStop(void);
static void PFC_StateRun(void);

/*------------------------------------
 * User state-transition functions
 * ----------------------------------*/
static void PFC_TransFaultInit(void);
static void PFC_TransInitFault(void);
static void PFC_TransInitStop(void);
static void PFC_TransStopFault(void);
static void PFC_TransStopInit(void);
static void PFC_TransStopRun(void);
static void PFC_TransRunFault(void);
static void PFC_TransRunStop(void);

/* State machine functions field */
static const SM_APP_STATE_FCN_T msSTATE = {PFC_StateFault, PFC_StateInit, PFC_StateStop, PFC_StateRun};


/* State-transition functions field */
static const SM_APP_TRANS_FCN_T msTRANS = {PFC_TransFaultInit, PFC_TransInitFault, PFC_TransInitStop, PFC_TransStopFault, PFC_TransStopInit, PFC_TransStopRun, PFC_TransRunFault, PFC_TransRunStop};

/* State machine structure declaration and initialization */
SM_APP_CTRL_T gsPFC_Ctrl = 
{
	/* gsPFC_Ctrl.psState, User state functions  */
	&msSTATE,
 	
 	/* gsPFC_Ctrl.psTrans, User state-transition functions */
 	&msTRANS,
 
  	/* gsPFC_Ctrl.uiCtrl, Default no control command */
  	SM_CTRL_NONE,
  	
  	/* gsPFC_Ctrl.eState, Default state after reset */
  	INIT 	
};


/*------------------------------------
 * User sub-state machine functions
 * ----------------------------------*/
static void PFC_StateRunSoftstart(void);
static void PFC_StateRunNormal(void);
static void PFC_StateRunLightload(void);

/*------------------------------------
 * User sub-state-transition functions
 * ----------------------------------*/

/* Sub-state machine functions field  */
const PFCN_VOID_VOID mPFC_STATE_RUN_TABLE[3] = 
{
		PFC_StateRunSoftstart,
		PFC_StateRunNormal,
		PFC_StateRunLightload
};


void PFC_FaultDetection(void);

/***************************************************************************//*!
*
* @brief   FAULT detection function
*
* @param   void
*
* @return  none
*
******************************************************************************/
#pragma section CODES_IN_RAM begin
void PFC_FaultDetection(void)
{
	/* Clearing actual faults before detecting them again  */         
	PFC_FAULT_CLEAR_ALL(gsPFC_Drive.FaultIdPending.R);
	
	/* DC bus voltage detection */
	if(gsPFC_Drive.sUCtrl.f16UDcBusFilt > gsPFC_Drive.sFaultThresholds.f16UDcBusOver)
	{ 
		gsPFC_Drive.FaultIdPending.B.UDcBusOver = 1;
	}
	else if((gsPFC_Drive.sUCtrl.f16UDcBusFilt < gsPFC_Drive.sFaultThresholds.f16UDcBusUnder)&&(bEnableCheckDCBusVunder))
	{ 
		gsPFC_Drive.FaultIdPending.B.UDcBusUnder = 1;
	}

	
	if(gsPFC_Drive.sUInPeakDetection.bSeveralPeaksDetectedFlag)
	{
		/* AC input voltage detection */
	    if(gsPFC_Drive.sUInPeakDetection.f16UInMax > gsPFC_Drive.sFaultThresholds.f16UInOver)
	    { 
	    	gsPFC_Drive.FaultIdPending.B.UInOver = 1;
	    }
	    else if((gsPFC_Drive.sUInPeakDetection.f16UInMax < gsPFC_Drive.sFaultThresholds.f16UInUnder)&&(bEnableCheckInputVunder))
	    { 
	    	gsPFC_Drive.FaultIdPending.B.UInUnder = 1;
	    }
	    
#if PLL
	    /* AC input frequency detection */
	    if(gsPFC_Ctrl.eState == RUN)
	    {
			if(gsPFC_Drive.sPhaseDetector.f16Freq > gsPFC_Drive.sFaultThresholds.f16FreqUInOver)
			{ 
				gsPFC_Drive.FaultIdPending.B.UInFreqOver = 1;
			}
			else if(gsPFC_Drive.sPhaseDetector.f16Freq < gsPFC_Drive.sFaultThresholds.f16FreqUInUnder)
			{ 
				gsPFC_Drive.FaultIdPending.B.UInFreqUnder = 1;
			}
	    }
#endif
	}
	/* Phase current detection */
	if(gsPFC_Drive.sICtrlPh1.f16IFdbck > gsPFC_Drive.sFaultThresholds.f16IInOver)
	{
		gsPFC_Drive.FaultIdPending.B.IPh1Over = 1;
	}
	if(gsPFC_Drive.sICtrlPh2.f16IFdbck > gsPFC_Drive.sFaultThresholds.f16IInOver)
	{
		gsPFC_Drive.FaultIdPending.B.IPh2Over = 1;
	}
	
	/* HW Over current detection */
	if(PFC_OVERCURRENT_FAULT())
	{
		gsPFC_Drive.FaultIdPending.B.HW_IOver = 1;
	}
	
	/* HW DC bus over voltage detection */
	if(PFC_OVERVOLTAGE_FAULT())
	{
		gsPFC_Drive.FaultIdPending.B.HW_UDcBusOver = 1;
	}
	
	/* pass fault to FaultId for recording */
	gsPFC_Drive.FaultId.R |= gsPFC_Drive.FaultIdPending.R;		
}

/***************************************************************************//*!
*
* @brief   FAULT state
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void PFC_StateFault()
{
	PH1_PWM_DIS();
	PH2_PWM_DIS();
		
	if(gsPFC_Drive.sUCtrl.f16UDcBusFilt < gsPFC_Drive.sUCtrl.f16UDcBusRelayOff)
	{
		RELAY_OFF();
		gsPFC_Drive.sFlag.RelayFlag = 0;
	}
	/* detect faults */
	PFC_FaultDetection();
	if(!gsPFC_Drive.FaultIdPending.R) // No fault detected for a certain time interval, recovery
	{
		if(--gsPFC_Drive.u16CounterTimeBase == 0)
		{
			gsPFC_Drive.u16CounterTimeBase = TIMEBASE_1MS_FASTLOOP_CNTR;
			if(--gsPFC_Drive.ui16CounterState == 0)
			{
				gsPFC_Ctrl.uiCtrl |= SM_CTRL_FAULT_CLEAR;  // clear fault command
			}
		}
	}
	else 
	{
		gsPFC_Drive.u16CounterTimeBase = TIMEBASE_1MS_FASTLOOP_CNTR;
		gsPFC_Drive.ui16CounterState = FAULT_RELEASE_DURATION;
	}
}

/***************************************************************************//*!
*
* @brief   Init state
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void PFC_StateInit()
{
	if(!bPFCVarInit)
	{
	bPFC_RUN = 0;
//	PFC_FAULT_CLEAR_ALL(gsPFC_Drive.FaultId.R);
	PFC_FAULT_CLEAR_ALL(gsPFC_Drive.FaultIdPending.R);
	PFC_CLEAR_OVERCURRENT_FAULT();
	PFC_CLEAR_OVERVOLTAGE_FAULT();
	
	/* Fault thresholds */
	gsPFC_Drive.sFaultThresholds.f16IInOver = FRAC16(PFC_IOVER_LIMIT/PFC_I_SCALE);
	gsPFC_Drive.sFaultThresholds.f16UDcBusOver = FRAC16(PFC_OVERVOLT_LIMIT/PFC_V_DCB_SCALE); 
    gsPFC_Drive.sFaultThresholds.f16UDcBusUnder = FRAC16(PFC_UNDERVOLT_LIMIT/PFC_V_DCB_SCALE);
	gsPFC_Drive.sFaultThresholds.f16UInOver = FRAC16(PFC_VIN_OVERVOLT_LIMIT/PFC_V_IN_SCALE);
	gsPFC_Drive.sFaultThresholds.f16UInUnder = FRAC16(PFC_VIN_UNDERVOLT_LIMIT/PFC_V_IN_SCALE);
	gsPFC_Drive.sFaultThresholds.f16FreqUInOver =  FRAC16(PFC_VIN_FREQOVER_LIMIT*2/PFC_FASTLOOP_FREQ);
	gsPFC_Drive.sFaultThresholds.f16FreqUInUnder = FRAC16(PFC_VIN_FREQUNDER_LIMIT*2/PFC_FASTLOOP_FREQ);	
	
	gsPFC_Drive.sFlag.DCMFlag = 0;
	gsPFC_Drive.sFlag.InterleaveFlag = 1;
	gsPFC_Drive.sFlag.PWM_enable = 0;
	gsPFC_Drive.sFlag.RelayFlag = 0;  /* flag indicate relay is open */
	gsPFC_Drive.sFlag.BrakeFlag = 0;
	/*------------------------- DC Bus voltage parameters --------------------------*/
	gsPFC_Drive.sUCtrl.f16UDcBusCmd = FRAC16(PFC_U_DCB_REF/PFC_V_DCB_SCALE);
	gsPFC_Drive.sUCtrl.f16UDcBusFilt = 0;
	gsPFC_Drive.sUCtrl.f16UDcBusRelayOff  = FRAC16(PFC_U_DCB_RELAY_OFF/PFC_V_DCB_SCALE);
	gsPFC_Drive.sUCtrl.f16UDcBusBurstOff = FRAC16(PFC_U_DCB_BURSTOFF/PFC_V_DCB_SCALE);
	gsPFC_Drive.sUCtrl.f16UDcBusBurstOn = FRAC16(PFC_U_DCB_BURSTON/PFC_V_DCB_SCALE);
	
	/* softstart ramp parameters */
	gsPFC_Drive.sUCtrl.sUDcBusRampParams.f16RampUp = FRAC16(PFC_U_SOFTSTART_STEP/PFC_V_DCB_SCALE);

	
	/* DC Bus control parameters */
	gsPFC_Drive.sUCtrl.sUDcBusFilter.sFltCoeff.f32B0 = PFC_U_DCB_IIR_B0;
	gsPFC_Drive.sUCtrl.sUDcBusFilter.sFltCoeff.f32B1 = PFC_U_DCB_IIR_B1;
	gsPFC_Drive.sUCtrl.sUDcBusFilter.sFltCoeff.f32A1 = PFC_U_DCB_IIR_A1;
	GDFLIB_FilterIIR1Init_F16(&gsPFC_Drive.sUCtrl.sUDcBusFilter);
	
	gsPFC_Drive.sUCtrl.sUDcBusPiParams.a32PGain = PFC_U_P_GAIN;
	gsPFC_Drive.sUCtrl.sUDcBusPiParams.a32IGain = PFC_U_I_GAIN;
	gsPFC_Drive.sUCtrl.sUDcBusPiParams.f16LowerLim = PFC_U_LOWER_LIMIT;
	gsPFC_Drive.sUCtrl.sUDcBusPiParams.f16UpperLim = PFC_U_UPPER_LIMIT;
	gsPFC_Drive.sUCtrl.sUDcBusPiParams.f16InErrK_1 = 0;
	gsPFC_Drive.sUCtrl.sUDcBusPiParams.f32IAccK_1 = 0;
	gsPFC_Drive.sUCtrl.bStopIntegFlag = 0;
	gsPFC_Drive.sUCtrl.a32IRef  = 0;
	gsPFC_Drive.sUCtrl.f16UDcBusErrLim = PFC_U_ERROR_LIMIT;

	/*------------------------- input voltage parameters --------------------------*/
	gsPFC_Drive.sUInPeakDetection.sFilter.sFltCoeff.f32B0    = PFC_U_IN_IIR_B0;
	gsPFC_Drive.sUInPeakDetection.sFilter.sFltCoeff.f32B1    = PFC_U_IN_IIR_B1;
	gsPFC_Drive.sUInPeakDetection.sFilter.sFltCoeff.f32A1    = PFC_U_IN_IIR_A1;
	GDFLIB_FilterIIR1Init_F16(&gsPFC_Drive.sUInPeakDetection.sFilter);
	
	gsPFC_Drive.sUInPeakDetection.bSeveralPeaksDetectedFlag = 0; /* flag indicate input voltage rms calculation isn't complete*/
	gsPFC_Drive.sUInPeakDetection.bPeakFlag = 0;
	gsPFC_Drive.sUInPeakDetection.u16VoltagePeakCnt = 0;
	gsPFC_Drive.sUInPeakDetection.u16DownCnt = 0;
	gsPFC_Drive.sUInPeakDetection.u16UpCnt = 0;
	gsPFC_Drive.sUInPeakDetection.bPeakDetectEn = 0;
	gsPFC_Drive.sUInPeakDetection.f16UInMaxTemp = 0;
	gsPFC_Drive.sUInPeakDetection.f16UInMax = 0;
#if PLL
	gsPFC_Drive.sPhaseDetector.bFallThrsdDetEn = 0;
	gsPFC_Drive.sPhaseDetector.bRiseThrsdDetEn = 0;
	gsPFC_Drive.sPhaseDetector.f16UInThreshold = FRAC16(PFC_VIN_PHASEDETECT_TH/PFC_V_IN_SCALE);
	gsPFC_Drive.sPhaseDetector.f16Freq = 0;
	gsPFC_Drive.sPhaseDetector.f16Phase = 0;
	gsPFC_Drive.sPhaseDetector.u16CntThrsd2Zero = 0;
	gsPFC_Drive.sPhaseDetector.sPeriodFilter.u16Sh = PHASE_DET_INPUT_VOLTAGE_PERIOD_FILTER_N;		
#endif
	
	/*------------------------- PFC current parameters --------------------------*/	
	gsPFC_Drive.sICtrlPh1.f16Duty = 0;
	gsPFC_Drive.sICtrlPh2.f16Duty = 0;

	gsPFC_Drive.sICtrlPh1.sIPiParams.a32PGain = PFC_I_P_GAIN1;
	gsPFC_Drive.sICtrlPh1.sIPiParams.a32IGain = PFC_I_I_GAIN1;
	gsPFC_Drive.sICtrlPh1.sIPiParams.f16LowerLim = PFC_I_LOWER_LIMIT;
	gsPFC_Drive.sICtrlPh1.sIPiParams.f16UpperLim = PFC_I_UPPER_LIMIT;
	gsPFC_Drive.sICtrlPh1.sIPiParams.f16InErrK_1 = 0;
	gsPFC_Drive.sICtrlPh1.sIPiParams.f32IAccK_1 = 0;
	gsPFC_Drive.sICtrlPh1.bStopIntegFlag = 0;
		
	gsPFC_Drive.sICtrlPh2.sIPiParams.a32PGain = PFC_I_P_GAIN1;
	gsPFC_Drive.sICtrlPh2.sIPiParams.a32IGain = PFC_I_I_GAIN1;
	gsPFC_Drive.sICtrlPh2.sIPiParams.f16LowerLim = PFC_I_LOWER_LIMIT;
	gsPFC_Drive.sICtrlPh2.sIPiParams.f16UpperLim = PFC_I_UPPER_LIMIT;
	gsPFC_Drive.sICtrlPh2.sIPiParams.f16InErrK_1 = 0;
	gsPFC_Drive.sICtrlPh2.sIPiParams.f32IAccK_1 = 0;
	gsPFC_Drive.sICtrlPh2.bStopIntegFlag = 0;
	
	gsPFC_Drive.sICtrlPh1.f16IFdbck = 0;
	gsPFC_Drive.sICtrlPh2.f16IFdbck = 0;
	gsPFC_Drive.sICtrlPh1.f16ICorrect = 0;
	gsPFC_Drive.sICtrlPh2.f16ICorrect = 0;
	
	/* current offset parameters */
	gsPFC_Drive.sICtrlPh1.sIOffsetFilter.u16Sh = PFC_I_OFFSET_MA_WINDOW;
	gsPFC_Drive.sICtrlPh1.sIOffsetFilter.a32Acc = FRAC16(0);
	gsPFC_Drive.sICtrlPh2.sIOffsetFilter.u16Sh = PFC_I_OFFSET_MA_WINDOW;
	gsPFC_Drive.sICtrlPh2.sIOffsetFilter.a32Acc = FRAC16(0);
	
	/* flag variables */
	bEnableCheckDCBusVunder = 0;            /* disable flag for checking Output voltages under */
	bEnableCheckInputVunder = 0;            /* disable flag for checking Input voltages under */                  
	
	gsPFC_PwmVal.pui32PwmFrac2Val2 = (uint32_t *)(&PWMA->SM[3].FRACVAL2);
	gsPFC_PwmVal.pui32PwmFrac3Val3 = (uint32_t *)(&PWMA->SM[3].FRACVAL3);
	gsPFC_PwmVal.pui32PwmFrac4Val4 = (uint32_t *)(&PWMA->SM[3].FRACVAL4);
	gsPFC_PwmVal.pui32PwmFrac5Val5 = (uint32_t *)(&PWMA->SM[3].FRACVAL5);
	
	/* timing control variables */
	gsPFC_Drive.u16CounterTimeBase = TIMEBASE_1MS_FASTLOOP_CNTR;
	gsPFC_Drive.ui16CounterState = PFC_IOFFSET_CALIB_DURATION; //200ms duration for current offset detection
	
	bPFCVarInit = 1;
	}
	  
	gsPFC_Drive.sICtrlPh1.f16IOffset = GDFLIB_FilterMA_F16(gsPFC_Drive.sICtrlPh1.f16IFdbck, &gsPFC_Drive.sICtrlPh1.sIOffsetFilter);
	gsPFC_Drive.sICtrlPh2.f16IOffset = GDFLIB_FilterMA_F16(gsPFC_Drive.sICtrlPh2.f16IFdbck, &gsPFC_Drive.sICtrlPh2.sIOffsetFilter);
	
	if(--gsPFC_Drive.u16CounterTimeBase == 0)
	{
		gsPFC_Drive.u16CounterTimeBase = TIMEBASE_1MS_FASTLOOP_CNTR;
		if(--gsPFC_Drive.ui16CounterState == 0)
		{
			ADC->OFFST[9] = gsPFC_Drive.sICtrlPh1.f16IOffset;
			ADC->OFFST[12] = gsPFC_Drive.sICtrlPh1.f16IOffset;
			ADC->OFFST[11] = gsPFC_Drive.sICtrlPh2.f16IOffset;
			ADC->OFFST[14] = gsPFC_Drive.sICtrlPh2.f16IOffset;
			gsPFC_Ctrl.uiCtrl |= SM_CTRL_INIT_DONE;
		}
	}
	PFC_FaultDetection();
	if(gsPFC_Drive.FaultIdPending.R)
	{
		gsPFC_Ctrl.uiCtrl |= SM_CTRL_FAULT; 
	}
	
}

/***************************************************************************//*!
*
* @brief   Stop state
*
* @param   void
*
* @return  none
*
******************************************************************************/

static void PFC_StateStop()
{
	
	/********* Relay control ***********/
	if(gsPFC_Drive.sUCtrl.f16UDcBusFilt < gsPFC_Drive.sUCtrl.f16UDcBusRelayOff)
	{
		RELAY_OFF();
		gsPFC_Drive.sFlag.RelayFlag = 0;
	}
	/********* brake control ***********/
	if(gsPFC_Drive.sUCtrl.f16UDcBusFilt > gsPFC_Drive.sFaultThresholds.f16UDcBusOver)
	{
		BRAKE_ON();
		gsPFC_Drive.sFlag.BrakeFlag = 1;
	}
	else   
	{
		BRAKE_OFF();
		gsPFC_Drive.sFlag.BrakeFlag = 0;
	}

    if(gsPFC_Drive.sUInPeakDetection.bSeveralPeaksDetectedFlag && gsPFC_Drive.sUInPeakDetection.f16UInMax > gsPFC_Drive.sFaultThresholds.f16UInUnder)
    {
    	gsPFC_Drive.sUCtrl.f16UDcBusRelayOn = MLIB_Mul_F16(FRAC16(PFC_COEFF_U_DCB_RELAY_ON), gsPFC_Drive.sUInPeakDetection.f16UInMax); 
    	
    	if(!gsPFC_Drive.sFlag.RelayFlag)
    	{
    		if(gsPFC_Drive.sUCtrl.f16UDcBusFilt > gsPFC_Drive.sUCtrl.f16UDcBusRelayOn) // Delay some time for diode rectifier output ramping up, then turn on the relay
    		{
    			if(--gsPFC_Drive.u16CounterTimeBase == 0)
    			{
    				gsPFC_Drive.u16CounterTimeBase = TIMEBASE_1MS_FASTLOOP_CNTR; 
    				if(gsPFC_Drive.ui16CounterState >=1)  
    				{
    					gsPFC_Drive.ui16CounterState--;   
    				} 				
    			}
    			if(gsPFC_Drive.ui16CounterState == 0 && gsPFC_Drive.sUInPeakDetection.f16UInFilt < FRAC16(PFC_VIN_THRESHOLD_RELAY_ON/PFC_V_IN_SCALE))
    			{
    			    RELAY_ON();
    			    gsPFC_Drive.sFlag.RelayFlag = 1;
    			    gsPFC_Drive.u16CounterTimeBase = TIMEBASE_1MS_FASTLOOP_CNTR;
    			    gsPFC_Drive.ui16CounterState = DELAY_BEFORE_CURRENT_CTRL;
    			}
    		}

    	}
    	else if(bPFC_RUN) // Relay has been turned on
    	{
    		bEnableCheckInputVunder = 1;  //Enable input under voltage detection when PFC is enabled
    		if(gsPFC_Drive.sUCtrl.f16UDcBusFilt > gsPFC_Drive.sUCtrl.f16UDcBusRelayOn) // delay 200ms after close relay
    		{
    			if(--gsPFC_Drive.u16CounterTimeBase == 0)
    			{
    			    gsPFC_Drive.u16CounterTimeBase = TIMEBASE_1MS_FASTLOOP_CNTR;
    			    if(gsPFC_Drive.ui16CounterState >=1)
    			    {
    			    	gsPFC_Drive.ui16CounterState--;
    			    }
    			}
    			// after the 200ms delay is over, start PFC when input voltage is low
    			if(gsPFC_Drive.ui16CounterState == 0 && gsPFC_Drive.sUInPeakDetection.f16UInFilt < FRAC16(PFC_VIN_THRESHOLD_RELAY_ON/PFC_V_IN_SCALE))
    			{
    			    gsPFC_Drive.u16CounterTimeBase = TIMEBASE_1MS_FASTLOOP_CNTR;
    			    gsPFC_Ctrl.uiCtrl |= SM_CTRL_START;
    			}  			
    		}

    	}
    }

    PFC_FaultDetection();
	if(gsPFC_Drive.FaultIdPending.R)
	{
		gsPFC_Ctrl.uiCtrl |= SM_CTRL_FAULT; 
	}
	
}
#pragma section CODES_IN_RAM end
/***************************************************************************//*!
*
* @brief   Run state
*
* @param   void
*
* @return  none
*
******************************************************************************/
#pragma section CODES_IN_RAM begin
static void PFC_StateRun()
{
	frac16_t f16Num;
	if(bPFC_RUN == 0)
	{
		gsPFC_Ctrl.uiCtrl |= SM_CTRL_STOP;
	}
	
	/* Compensate the feedback current for DCM */	
	gsPFC_Drive.sUCtrl.f16VoltDiff_UDc_UIn = MLIB_Sub_F16(gsPFC_Drive.sUCtrl.f16UDcBusFilt,gsPFC_Drive.sUInPeakDetection.f16UInFilt);
	if(gsPFC_Drive.sUCtrl.f16VoltDiff_UDc_UIn < 0 )
	{
		gsPFC_Drive.sUCtrl.f16VoltDiff_UDc_UIn = 0;
	}
	
	if(gsPFC_Drive.sFlag.DCMFlag)
	{
		f16Num = MLIB_Mul_F16(gsPFC_Drive.sUCtrl.f16UDcBusFilt, gsPFC_Drive.sICtrlPh1.f16Duty); // Duty_ph1 * Udc
		gsPFC_Drive.sICtrlPh1.f16CompCoeff = MLIB_Div1QSat_F16(f16Num, gsPFC_Drive.sUCtrl.f16VoltDiff_UDc_UIn);
		gsPFC_Drive.sICtrlPh1.f16ICorrect = MLIB_Mul_F16(gsPFC_Drive.sICtrlPh1.f16IFdbck, gsPFC_Drive.sICtrlPh1.f16CompCoeff); 

				
		f16Num = MLIB_Mul_F16(gsPFC_Drive.sUCtrl.f16UDcBusFilt, gsPFC_Drive.sICtrlPh2.f16Duty); // Duty_ph2 * Udc
		gsPFC_Drive.sICtrlPh2.f16CompCoeff = MLIB_Div1QSat_F16(f16Num, gsPFC_Drive.sUCtrl.f16VoltDiff_UDc_UIn);
		gsPFC_Drive.sICtrlPh2.f16ICorrect = MLIB_Mul_F16(gsPFC_Drive.sICtrlPh2.f16IFdbck, gsPFC_Drive.sICtrlPh2.f16CompCoeff);
	}
	else
	{
		gsPFC_Drive.sICtrlPh1.f16ICorrect = gsPFC_Drive.sICtrlPh1.f16IFdbck;
		gsPFC_Drive.sICtrlPh2.f16ICorrect = gsPFC_Drive.sICtrlPh2.f16IFdbck;
	}
	 
	
	if(gsPFC_Drive.sFlag.PWM_enable)
	{
		#if PLL
	        gsPFC_Drive.sICtrlPh1.f16IRef = MLIB_Mul_F16as(gsPFC_Drive.sUCtrl.a32IRef, gsPFC_Drive.sPhaseDetector.f16Sine);
        #else
	        gsPFC_Drive.sICtrlPh1.f16IRef = MLIB_Mul_F16as(gsPFC_Drive.sUCtrl.a32IRef, gsPFC_Drive.sUInPeakDetection.f16UInFilt);
        #endif
	   
	        gsPFC_Drive.sICtrlPh1.sIPiParams.f16LowerLim = MLIB_Neg_F16(gsPFC_Drive.f16DutyComp);
	        gsPFC_Drive.sICtrlPh1.f16IError =  MLIB_SubSat_F16(gsPFC_Drive.sICtrlPh1.f16IRef, gsPFC_Drive.sICtrlPh1.f16ICorrect);
	        gsPFC_Drive.sICtrlPh1.f16Duty = GFLIB_CtrlPIpAW_F16(gsPFC_Drive.sICtrlPh1.f16IError, &gsPFC_Drive.sICtrlPh1.bStopIntegFlag, &gsPFC_Drive.sICtrlPh1.sIPiParams);
	        gsPFC_Drive.sICtrlPh1.f16Duty = MLIB_AddSat_F16(gsPFC_Drive.sICtrlPh1.f16Duty, gsPFC_Drive.f16DutyComp);
	       
	        if(gsPFC_Drive.sFlag.InterleaveFlag)
	        {
	        	gsPFC_Drive.sICtrlPh2.sIPiParams.f16LowerLim = MLIB_Neg_F16(gsPFC_Drive.f16DutyComp);
	        	gsPFC_Drive.sICtrlPh2.f16IError =  MLIB_SubSat_F16(gsPFC_Drive.sICtrlPh1.f16IRef, gsPFC_Drive.sICtrlPh2.f16ICorrect);
	        	gsPFC_Drive.sICtrlPh2.f16Duty = GFLIB_CtrlPIpAW_F16(gsPFC_Drive.sICtrlPh2.f16IError, &gsPFC_Drive.sICtrlPh2.bStopIntegFlag, &gsPFC_Drive.sICtrlPh2.sIPiParams);
	            gsPFC_Drive.sICtrlPh2.f16Duty = MLIB_AddSat_F16(gsPFC_Drive.sICtrlPh2.f16Duty, gsPFC_Drive.f16DutyComp);
		     }
	    PFCDRV_PWMA3_update(gsPFC_Drive.sICtrlPh1.f16Duty,gsPFC_Drive.sICtrlPh2.f16Duty);
	}
}
#pragma section CODES_IN_RAM end

/***************************************************************************//*!
*
* @brief   FAULT to INIT transition
*
* @param   void
*
* @return  none
*
******************************************************************************/
#pragma section CODES_IN_RAM begin
static void PFC_TransFaultInit()
{
	bPFCVarInit = 0;
	gsPFC_Drive.u16CounterTimeBase = TIMEBASE_1MS_FASTLOOP_CNTR;
	gsPFC_Drive.ui16CounterState = PFC_IOFFSET_CALIB_DURATION; //Time duration for current offset detection
}

/***************************************************************************//*!
*
* @brief   INIT to FAULT transition
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void PFC_TransInitFault()
{
	gsPFC_Drive.u16CounterTimeBase = TIMEBASE_1MS_FASTLOOP_CNTR;
	gsPFC_Drive.ui16CounterState = FAULT_RELEASE_DURATION; //Time duration for fault release
}

/***************************************************************************//*!
*
* @brief   INIT to STOP transition
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void PFC_TransInitStop()
{
	gsPFC_Drive.u16CounterTimeBase = TIMEBASE_1MS_FASTLOOP_CNTR;
	gsPFC_Drive.ui16CounterState = UDCBUS_RAMPUP_DURATION; // Time duration for bus voltage rise 
}

/***************************************************************************//*!
*
* @brief   STOP to INIT transition
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void PFC_TransStopInit(void)
{

}
/***************************************************************************//*!
*
* @brief   STOP to RUN transition
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void PFC_TransStopRun()
{
	ePFC_StateRunSub = SOFTSTART;
	gsPFC_Drive.sUCtrl.f16UDcBusReq = gsPFC_Drive.sUCtrl.f16UDcBusFilt; // Set current real DC bus voltage to voltage reference
	GFLIB_RampInit_F16(gsPFC_Drive.sUCtrl.f16UDcBusFilt, &gsPFC_Drive.sUCtrl.sUDcBusRampParams);
	gsPFC_Drive.u16CounterTimeBase = TIMEBASE_1MS_SLOWLOOP_CNTR;
	gsPFC_Drive.sFlag.PWM_enable = 1;
	PH1_PWM_EN();
	if(gsPFC_Drive.sFlag.InterleaveFlag)
	{
		PH2_PWM_EN();
	}
	gsPFC_Ctrl.uiCtrl |= SM_CTRL_RUN_ACK;
}

/***************************************************************************//*!
*
* @brief   RUN to STOP transition
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void PFC_TransRunStop()
{
	PH1_PWM_DIS();
	PH2_PWM_DIS();
	ePFC_StateRunSub = SOFTSTART;
	bEnableCheckDCBusVunder = 0; //Disable bus voltage under detection when PFC is not working
	bEnableCheckInputVunder = 0; //Disable input voltage under detection when PFC is not working
	BRAKE_OFF(); //Disconnect load
	gsPFC_Drive.sFlag.BrakeFlag = 0;
	gsPFC_Drive.sICtrlPh1.f16Duty = 0;
	gsPFC_Drive.sICtrlPh2.f16Duty = 0;
	gsPFC_Drive.sICtrlPh1.sIPiParams.f32IAccK_1 = 0;
	gsPFC_Drive.sICtrlPh2.sIPiParams.f32IAccK_1 = 0;
	gsPFC_Drive.sUCtrl.sUDcBusPiParams.f32IAccK_1 = MLIB_Conv_F32s(gsPFC_Drive.sUCtrl.sUDcBusPiParams.f16LowerLim);
	
	gsPFC_Ctrl.uiCtrl |= SM_CTRL_STOP_ACK;
}

/***************************************************************************//*!
*
* @brief   STOP to FAULT transition
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void PFC_TransStopFault()
{
	bEnableCheckInputVunder = 0; 
	 if(gsPFC_Drive.sFlag.BrakeFlag)
	 {
		 BRAKE_OFF();
		 gsPFC_Drive.sFlag.BrakeFlag = 0;
	 }
	 gsPFC_Drive.u16CounterTimeBase = TIMEBASE_1MS_FASTLOOP_CNTR; 
	 gsPFC_Drive.ui16CounterState = FAULT_RELEASE_DURATION; 
}

/***************************************************************************//*!
*
* @brief   RUN to FAULT transition
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void PFC_TransRunFault()
{
	PH1_PWM_DIS();
	PH2_PWM_DIS();
	bEnableCheckDCBusVunder = 0;
	bEnableCheckInputVunder = 0; 
	gsPFC_Drive.sFlag.PWM_enable = 0;
	BRAKE_OFF();
	gsPFC_Drive.sFlag.BrakeFlag = 0;
	gsPFC_Drive.u16CounterTimeBase = TIMEBASE_1MS_FASTLOOP_CNTR;
	gsPFC_Drive.ui16CounterState = FAULT_RELEASE_DURATION;
}

/***************************************************************************//*!
*
* @brief   softstart sub-state, reference ramp up
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void PFC_StateRunSoftstart(void)
{
	// Real DC bus voltage is already larger than command, go to NORMAL sub-state
	if(gsPFC_Drive.sUCtrl.f16UDcBusFilt > gsPFC_Drive.sUCtrl.f16UDcBusCmd)
	{
		ePFC_StateRunSub = NORMAL;
		bEnableCheckDCBusVunder = 1; //Enable bus voltage under detection when PFC is working and soft-start state has finished
		gsPFC_Drive.u16CounterTimeBase = TIMEBASE_1MS_SLOWLOOP_CNTR;
		
		if(!gsPFC_Drive.sFlag.BrakeFlag)
		{
		    BRAKE_ON(); // To add loading on DC bus
		    gsPFC_Drive.sFlag.BrakeFlag = 1;
		}
		gsPFC_Drive.sUCtrl.f16UDcBusReq = gsPFC_Drive.sUCtrl.f16UDcBusCmd; 
	}
	// DC bus voltage request is behind command, increase the request voltage
	else if(gsPFC_Drive.sUCtrl.f16UDcBusReq < gsPFC_Drive.sUCtrl.f16UDcBusCmd)
	{
		if(--gsPFC_Drive.u16CounterTimeBase == 0)
		{
			gsPFC_Drive.sUCtrl.f16UDcBusReq = GFLIB_Ramp_F16(gsPFC_Drive.sUCtrl.f16UDcBusCmd, &gsPFC_Drive.sUCtrl.sUDcBusRampParams);
			gsPFC_Drive.u16CounterTimeBase = TIMEBASE_1MS_SLOWLOOP_CNTR;
		}
	}
	// DC bus voltage request has reached command, but the real DC bus voltage is still behind command, go to NORMAL sub-state
	else
	{
		ePFC_StateRunSub = NORMAL;
		bEnableCheckDCBusVunder = 1;
		gsPFC_Drive.u16CounterTimeBase = TIMEBASE_1MS_SLOWLOOP_CNTR;

		if(!gsPFC_Drive.sFlag.BrakeFlag)
		{
			BRAKE_ON();
			gsPFC_Drive.sFlag.BrakeFlag = 1;
		}
	    gsPFC_Drive.sUCtrl.f16UDcBusReq = gsPFC_Drive.sUCtrl.f16UDcBusCmd;
	}
}

/***************************************************************************//*!
*
* @brief   Normal sub-state, interleave or not control and normal to lightload judgement
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void PFC_StateRunNormal(void)
{ 
	
	if(gsPFC_Drive.sUCtrl.f16UDcBusFilt > (gsPFC_Drive.sUCtrl.f16UDcBusCmd + FRAC16(PFC_DC_BUS_FLUCTUATION/PFC_V_DCB_SCALE)))
	{
		gsPFC_Drive.sUCtrl.sUDcBusPiParams.f32IAccK_1 -= (((frac32_t)5)<<16);   // approximate output voltage feed forward to restrain the overshoot
	}
	else if(gsPFC_Drive.sUCtrl.f16UDcBusFilt < (gsPFC_Drive.sUCtrl.f16UDcBusCmd - FRAC16(PFC_DC_BUS_FLUCTUATION/PFC_V_DCB_SCALE)))
	{
		gsPFC_Drive.sUCtrl.sUDcBusPiParams.f32IAccK_1 += (((frac32_t)5)<<16);   // approximate output voltage feed forward to restrain the overshoot
	}
	
	
	
#if BURST_MODE_EN
	/* Enter burst mode when DC bus voltage is larger than low overshoot command and voltage controller outputs lower limit
	 * or when DC bus voltage is larger than high overshoot command
	 * */
	if((gsPFC_Drive.sUCtrl.f16UDcBusCtrlOut == gsPFC_Drive.sUCtrl.sUDcBusPiParams.f16LowerLim && \
			gsPFC_Drive.sUCtrl.f16UDcBusFilt > (gsPFC_Drive.sUCtrl.f16UDcBusCmd + FRAC16(PFC_DC_BUS_NORMAL_OVERSHOOT_DELTA_LOW/PFC_V_DCB_SCALE))) ||\
			gsPFC_Drive.sUCtrl.f16UDcBusFilt > (gsPFC_Drive.sUCtrl.f16UDcBusCmd + FRAC16(PFC_DC_BUS_NORMAL_OVERSHOOT_DELTA_HIGH/PFC_V_DCB_SCALE)))
	{
		ALL_PFC_PWM_DIS();
		gsPFC_Drive.sFlag.PWM_enable = 0;
		ePFC_StateRunSub = LIGHTLOAD;
		gsPFC_Drive.u16CounterTimeBase = TIMEBASE_1MS_SLOWLOOP_CNTR;
		gsPFC_Drive.ui16CounterState = BURST_OFF_MIN_DURATION;
	}
#endif
}

/***************************************************************************//*!
*
* @brief   lightload sub-state, lightload to normal judgement
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void PFC_StateRunLightload(void)
{
#if BURST_MODE_EN
	if(!gsPFC_Drive.sFlag.PWM_enable)
	{
		if(--gsPFC_Drive.u16CounterTimeBase == 0)
		{
			gsPFC_Drive.u16CounterTimeBase = TIMEBASE_1MS_SLOWLOOP_CNTR;
			if(gsPFC_Drive.ui16CounterState >=1) 
			{
				gsPFC_Drive.ui16CounterState--;
			}
		}     		
	}
	else
	{
		gsPFC_Drive.u16CounterTimeBase = TIMEBASE_1MS_SLOWLOOP_CNTR;
		gsPFC_Drive.ui16CounterState = BURST_OFF_MIN_DURATION; //minimum burst off time duration
	}
	
	// When DC bus reaches a higher threshold, turn off the switch (burst off)
	if(gsPFC_Drive.sUCtrl.f16UDcBusFilt > gsPFC_Drive.sUCtrl.f16UDcBusBurstOff)
	{
		gsPFC_Drive.sFlag.PWM_enable = 0;
		ALL_PFC_PWM_DIS();
		gsPFC_Drive.sICtrlPh1.sIPiParams.f32IAccK_1 = 0;
		gsPFC_Drive.sICtrlPh2.sIPiParams.f32IAccK_1 = 0;
	}
	else if(gsPFC_Drive.sUCtrl.f16UDcBusFilt < gsPFC_Drive.sUCtrl.f16UDcBusBurstOn && !gsPFC_Drive.sFlag.PWM_enable)
	{
		if(gsPFC_Drive.ui16CounterState != 0)//burst off time is less than the minimum duration, 
			                                 //voltage drop too fast, directly return to normal mode
		{
			gsPFC_Drive.sUCtrl.sUDcBusPiParams.f16InErrK_1 = 0;
			gsPFC_Drive.sUCtrl.sUDcBusPiParams.f32IAccK_1 = MLIB_Conv_F32s(gsPFC_Drive.sUCtrl.sUDcBusPiParams.f16LowerLim);	
			ePFC_StateRunSub = NORMAL;	
		}
		// if burst off time is longer than the minimum duration, stay in light load mode and burst on
		gsPFC_Drive.sFlag.PWM_enable = 1;
		if(gsPFC_Drive.sFlag.InterleaveFlag)  ALL_PFC_PWM_EN();
	    else PH1_PWM_EN();
	}
	
	// output voltage drop too low , directly return to normal mode
	if(gsPFC_Drive.sUCtrl.f16UDcBusFilt < (gsPFC_Drive.sUCtrl.f16UDcBusCmd - FRAC16(PFC_DC_BUS_BURST_UNDER_DELTA/PFC_V_DCB_SCALE)))
	{
		gsPFC_Drive.sUCtrl.sUDcBusPiParams.f16InErrK_1 = 0;
		gsPFC_Drive.sUCtrl.sUDcBusPiParams.f32IAccK_1 = MLIB_Conv_F32s(gsPFC_Drive.sUCtrl.sUDcBusPiParams.f16LowerLim);
		ePFC_StateRunSub = NORMAL;
		gsPFC_Drive.sFlag.PWM_enable = 1;
		if(gsPFC_Drive.sFlag.InterleaveFlag)  ALL_PFC_PWM_EN();
		else PH1_PWM_EN();
	}	
#endif
}
#pragma section CODES_IN_RAM end
