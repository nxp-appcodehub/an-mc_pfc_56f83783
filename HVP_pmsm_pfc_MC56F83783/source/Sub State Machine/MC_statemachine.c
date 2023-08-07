/*
 * MC_statemachine.c
 *
 *  Created on: Jun 29, 2013
 *      Author: B45091
 */
/******************************************************************************
* 
 * Copyright (c) 2012 - 2015, Freescale Semiconductor, Inc.
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
* Main state machine realization
* Motor sub state machine.
*
******************************************************************************/

/******************************************************************************
* Includes
******************************************************************************/
#include "MC_statemachine.h"
#include "hwcontrol.h"
#include "peripherals.h"
#include "PFC_statemachine.h"
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
    CALIB           = 0,
    READY           = 1,
    ALIGN          	= 2,
	STARTUP			= 3,
	SPIN			= 4,
	FREEWHEEL		= 5,
} MC_RUN_SUBSTATE_T;         /* Run MLIB_Sub_F16-states */

/******************************************************************************
* Global variables
******************************************************************************/
/*convenience for changing Free-Master based value*/
/*if based value is fractional, in order to keep precision the best method is manual input
 * or select milli unit as based value*/
/*define the base value used for FreeMaster*/
float fmBaseValue_I;
float fmBaseValue_U;
float fmBaseValue_E;
float fmBaseValue_UDC;
float fmBaseValue_N;
float fmBaseValue_Flux;
float fmBaseValue_Power;/* MR20150324_A */
/*convenience for speed command input*/
unsigned int ActionOnceFlag = 1;


/******************** Led indication variables *****************/
uint16_t								uw16LedCntBlkTimes;
uint16_t								uw16LedCntBlkDuration;
uint16_t								uw16LedCntPatternTimes;
uint16_t								uw16LedCntPatternSpace;
uint16_t								uw16LedCntBlkTimesVal;
uint16_t								uw16LedCntBlkDurationVal;
uint16_t								uw16LedCntPatternTimesVal;
uint16_t								uw16LedCntPatternSpaceVal;

/******************** Power down mode variables ****************/
uint16_t 							uw16WaitForWakeUpFlag  = 0; 	  /* Indicates whether the system is waiting for waking up */
uint16_t 							uw16DelayEnterStopCntr = 0;       /* Used to delay couple of seconds before entering stop mode */

/************* Under voltage delay protection *****************/
uint32_t 							uw32DelayUnderVoltProtectCntr = 0;/* MR20140910_D; MR20150324_B, voltage must be low for this period of time before protection */

/************* Over Current software protection **************/
frac16_t								f16ILength = 0;					  /* Current vector length */
frac16_t								f16ILengthFilt = 0;
GDFLIB_FILTER_MA_T_A32				sILengthFilter;

/*********************** MC structure *************************/
MCSTRUC_FOC_PMSM_OBS_DQ_T			gsMC_Drive;						  /* Variable for FOC */
MCSTRUC_CONTROL_LOOP_T				geMC_StateRunLoop;				  /* Fast and slow loop index */
MCSTRUC_ADC_2_PHASE_ASSIGNMENT_T 	gsMC_IChannels;					  /* Indicates the latest 2 current channels for ADC sampling */
MCSTRUC_FAULT_RECORD_T              faultRecordFlag;				  /* Stores the historical fault types */
static uint32_t 						uw32CounterPressureRelax; 		  /* Counter used to MLIB_Add_F16 time delays between each start up */

GDFLIB_FILTER_IIR1_T_F32 			sHallSpdFilter;
frac16_t f16SpeedHall, f16SpeedHallFilt;

/******************************************************************************
* Local variables
******************************************************************************/
static bool_t                 	mbMC_SwitchAppOnOff;				  /* Switch between RUN and STOP state */
static MC_RUN_SUBSTATE_T		meMC_StateRun;						  /* Sub-state index */		

/* MC current phase channels */
static MCSTRUC_ADC_2_PHASE_ASSIGNMENT_T	msChannels[8] =
{{0,  0}, {IC_ANA, IB_ANB},\
		  /* Sector1 */  
		  {IA_ANA, IC_ANB},\
		  /* Sector2 */
		  {IA_ANA, IC_ANB},\
		  /* Sector3 */
		  {IA_ANA, IB_ANB},\
		  /* Sector4 */
		  {IA_ANA, IB_ANB},\
		  /* Sector5 */
		  {IC_ANA, IB_ANB},\
		  /* Sector6 */
		  {0, 0}};

int16_t	w16PWMTestFlag;
int16_t  w16OpenLpTstFlag;

/******************************************************************************
* Local functions
******************************************************************************/

/*------------------------------------
 * User state machine functions
 * ----------------------------------*/
static void MC_StateFault(void);
static void MC_StateInit(void);
static void MC_StateStop(void);
static void MC_StateRun(void);

/*------------------------------------
 * User state-transition functions
 * ----------------------------------*/
static void MC_TransFaultInit(void);
static void MC_TransInitFault(void);
static void MC_TransInitStop(void);
static void MC_TransStopFault(void);
static void MC_TransStopInit(void);
static void MC_TransStopRun(void);
static void MC_TransRunFault(void);
static void MC_TransRunStop(void);

/* State machine functions field (in pmem) */
static const SM_APP_STATE_FCN_T msSTATE = {MC_StateFault, MC_StateInit, MC_StateStop, MC_StateRun};

/* State-transition functions field (in pmem) */
static const SM_APP_TRANS_FCN_T msTRANS = {MC_TransFaultInit, MC_TransInitFault, MC_TransInitStop, MC_TransStopFault, MC_TransStopInit, MC_TransStopRun, MC_TransRunFault, MC_TransRunStop};

/* State machine structure declaration and initialization */
SM_APP_CTRL_T gsMC_Ctrl = 
{
	/* gsMC_Ctrl.psState, User state functions  */
	&msSTATE,
 	
 	/* gsMC_Ctrl.psTrans, User state-transition functions */
 	&msTRANS,
 
  	/* gsMC_Ctrl.uiCtrl, Deafult no control command */
  	SM_CTRL_NONE,
  	
  	/* gsMC_Ctrl.eState, Default state after reset */
  	INIT 	
};


/*------------------------------------
 * User MLIB_Sub_F16-state machine functions
 * ----------------------------------*/
static void MC_StateRunCalib(void);
static void MC_StateRunReady(void);
static void MC_StateRunAlign(void);
static void MC_StateRunStartup(void);
static void MC_StateRunSpin(void);
static void MC_StateRunFreewheel(void);

static void MC_StateRunCalibSlow(void);
static void MC_StateRunReadySlow(void);
static void MC_StateRunAlignSlow(void);
static void MC_StateRunStartupSlow(void);
static void MC_StateRunSpinSlow(void);
static void MC_StateRunFreewheelSlow(void);

/*------------------------------------
 * User MLIB_Sub_F16-state-transition functions
 * ----------------------------------*/
static void MC_TransRunCalibReady(void);
static void MC_TransRunReadyAlign(void);
static void MC_TransRunAlignStartup(void);
static void MC_TransRunAlignReady(void);
static void MC_TransRunStartupSpin(void);
static void MC_TransRunStartupFreewheel(void);
static void MC_TransRunSpinFreewheel(void);
static void MC_TransRunFreewheelAlign(void);
static void MC_TransRunFreewheelReady(void);

/* Sub-state machine functions field (in pmem) */
__pmem static const PFCN_VOID_VOID mMC_STATE_RUN_TABLE[6][2] = 
{
		{MC_StateRunCalib, MC_StateRunCalibSlow},
		{MC_StateRunReady, MC_StateRunReadySlow},
		{MC_StateRunAlign, MC_StateRunAlignSlow},
		{MC_StateRunStartup, MC_StateRunStartupSlow}, 
		{MC_StateRunSpin, MC_StateRunSpinSlow},
		{MC_StateRunFreewheel, MC_StateRunFreewheelSlow}
};


static void MC_FaultDetection(void);
static void ADCChannelMapping(MCSTRUC_FOC_PMSM_T *ptr);
static void ADCOffsetConfig(MCSTRUC_FOC_PMSM_T *ptr);
static void GetCurrent(MCSTRUC_FOC_PMSM_T *ptr);
static void LED_Indication(void);
static void LED_Indication_In_Fault(void);
/******************************************************************************
* Global functions
******************************************************************************/

#pragma section CODES_IN_RAM begin
/***************************************************************************//*!
*
* @brief   FAULT state
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void MC_StateFault(void)
{
	/* Disables PWM outputs */
	MC_DISABLE_PWM_OUTPUT();
	
	/* DC bus voltage filter */
	gsMC_Drive.sFocPMSM.f16UDcBusFilt = gsPFC_Drive.sUCtrl.f16UDcBusFilt;
	
    /* Enable user application switch, so that it goes to ready state directly after exiting fault state */
    mbMC_SwitchAppOnOff = true;

	/* Slow loop control */
    if ( SLOW == geMC_StateRunLoop)
    {
    	if (--gsMC_Drive.uw16CounterSlowLoop == 0)
    	{
    		gsMC_Drive.uw16CounterSlowLoop = gsMC_Drive.uw16DividerSlowLoop;
    		
    		MC_CLEAR_OVERCURRENT_FAULT();
        	
        	MC_FaultDetection();
        	
        	if (gsMC_Drive.sFaultIdPending.R == 0)
        	{
        		if (gsMC_Drive.uw16CounterState == 0)
    			{
    				gsMC_Ctrl.uiCtrl |= SM_CTRL_FAULT_CLEAR; 
    			}
    			else
    			{
    				gsMC_Drive.uw16CounterState--;
    			}
        	}
    		else
    		{
    			gsMC_Drive.uw16CounterState = gsMC_Drive.uw16TimeFaultRelease;// 2s
    		}
        	
        	LED_Indication_In_Fault();
    	}

    }
    
//    TestPin_NegVal();

}
#pragma section CODES_IN_RAM end
/***************************************************************************//*!
*
* @brief   INIT state
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void MC_StateInit(void)
{
	/* common used Flag */
	mbMC_SwitchAppOnOff	= false;
	
	sHallSpdFilter.f16FltBfrX[0] = 0;
	sHallSpdFilter.f32FltBfrY[0] = 0;
	sHallSpdFilter.sFltCoeff.f32A1 = FRAC32(MC_FILTER_SPEED_EST_A1/-2.0);
	sHallSpdFilter.sFltCoeff.f32B0 = FRAC32(MC_FILTER_SPEED_EST_B0/2.0);
	sHallSpdFilter.sFltCoeff.f32B1 = FRAC32(MC_FILTER_SPEED_EST_B1/2.0);

	/* define the base value used for FreeMaster */
	fmBaseValue_I = I_MAX;
	fmBaseValue_U = U_MAX;
	fmBaseValue_E = E_MAX;
	fmBaseValue_UDC = U_DCB_MAX;
	fmBaseValue_N = N_MAX;
	fmBaseValue_Power = POWER_MAX;

	/* channel configuration */
 	gsMC_IChannels.uw16Ph0 = 0;
 	gsMC_IChannels.uw16Ph1 = 0;

 	/* fault state initialisation */
	gsMC_Drive.sFaultIdPending.R = 0x0;
	gsMC_Drive.sFaultThresholds.f16UDcBusOver = FRAC16(MC_OVERVOLT_LIMIT / U_DCB_MAX);
	gsMC_Drive.sFaultThresholds.f16UDcBusUnder = FRAC16(MC_UNDERVOLT_LIMIT / U_DCB_MAX);
	
	/* PMSM FOC params */	
	gsMC_Drive.sFocPMSM.f16DutyCycleLimit = CLOOP_LIMIT;

	// 100Hz bandwidth for D-axis current controller
  	gsMC_Drive.sFocPMSM.sIdPiParams.a32PGain = D_KP_GAIN;
   	gsMC_Drive.sFocPMSM.sIdPiParams.a32IGain = D_KI_GAIN;
   	gsMC_Drive.sFocPMSM.sIdPiParams.f32IAccK_1 = 0;
   	gsMC_Drive.sFocPMSM.sIdPiParams.f16UpperLim = FRAC16(MC_PI_D_ACR_OUT_LIMIT / U_MAX);  
   	gsMC_Drive.sFocPMSM.sIdPiParams.f16LowerLim = FRAC16(-MC_PI_D_ACR_OUT_LIMIT / U_MAX);
   	gsMC_Drive.sFocPMSM.sIdPiParams.bLimFlag = 0;

	gsMC_Drive.sFocPMSM.sIqPiParams.a32PGain = Q_KP_GAIN;
   	gsMC_Drive.sFocPMSM.sIqPiParams.a32IGain = Q_KI_GAIN;
   	gsMC_Drive.sFocPMSM.sIqPiParams.f32IAccK_1 = 0;
   	gsMC_Drive.sFocPMSM.sIqPiParams.f16UpperLim = FRAC16(MC_PI_Q_ACR_OUT_LIMIT / U_MAX);
   	gsMC_Drive.sFocPMSM.sIqPiParams.f16LowerLim = FRAC16(-MC_PI_Q_ACR_OUT_LIMIT / U_MAX);    
   	gsMC_Drive.sFocPMSM.sIqPiParams.bLimFlag = 0;

    gsMC_Drive.sFocPMSM.i16IdPiSatFlag = 0;
	gsMC_Drive.sFocPMSM.i16IqPiSatFlag = 0;

	gsMC_Drive.sFocPMSM.sUDcBusFilter.sFltCoeff.f32B0 = FRAC32(MC_FILTER_UDCBUS_B0 / 2.0);
	gsMC_Drive.sFocPMSM.sUDcBusFilter.sFltCoeff.f32B1 = FRAC32(MC_FILTER_UDCBUS_B1 / 2.0);
	gsMC_Drive.sFocPMSM.sUDcBusFilter.sFltCoeff.f32A1 = FRAC32(MC_FILTER_UDCBUS_A1 / -2.0);
	GDFLIB_FilterIIR1Init_F16(&gsMC_Drive.sFocPMSM.sUDcBusFilter);	
	gsMC_Drive.sFocPMSM.sUDcBusFilter.f16FltBfrX[0] = FRAC16((MC_OVERVOLT_LIMIT + MC_UNDERVOLT_LIMIT) / 2.0 / U_DCB_MAX); /* Filter init not to enter to fault */
	gsMC_Drive.sFocPMSM.sUDcBusFilter.f32FltBfrY[0] = FRAC32((MC_OVERVOLT_LIMIT + MC_UNDERVOLT_LIMIT) / 2.0 / U_DCB_MAX); /* Filter init not to enter to fault */

	gsMC_Drive.sFocPMSM.f16UDcBus = 0;
	gsMC_Drive.sFocPMSM.f16UDcBusFilt = 0;

	gsMC_Drive.sFocPMSM.sAnglePosEl.f16Sin = 0;
	gsMC_Drive.sFocPMSM.sAnglePosEl.f16Cos = 0;
	gsMC_Drive.sFocPMSM.sAnglePosElUpdate.f16Sin = 0;
	gsMC_Drive.sFocPMSM.sAnglePosElUpdate.f16Cos = 0;

	gsMC_Drive.sFocPMSM.sIABC.f16A = 0;
	gsMC_Drive.sFocPMSM.sIABC.f16B = 0;
	gsMC_Drive.sFocPMSM.sIABC.f16C = 0;

	gsMC_Drive.sFocPMSM.sIAlBe.f16Alpha = 0;
	gsMC_Drive.sFocPMSM.sIAlBe.f16Beta = 0;

	gsMC_Drive.sFocPMSM.sIDQ.f16D = 0;
	gsMC_Drive.sFocPMSM.sIDQ.f16Q = 0;

	gsMC_Drive.sFocPMSM.sIDQReq.f16D = 0;
	gsMC_Drive.sFocPMSM.sIDQReq.f16Q = 0;

	gsMC_Drive.sFocPMSM.sIDQError.f16D = 0;
	gsMC_Drive.sFocPMSM.sIDQError.f16Q = 0;
	
	gsMC_Drive.sFocPMSM.sDutyABC.f16A = 0;
	gsMC_Drive.sFocPMSM.sDutyABC.f16B = 0;
	gsMC_Drive.sFocPMSM.sDutyABC.f16C = 0;

	gsMC_Drive.sFocPMSM.sUAlBeReq.f16Alpha = 0;
	gsMC_Drive.sFocPMSM.sUAlBeReq.f16Beta = 0;

	gsMC_Drive.sFocPMSM.sUAlBeComp.f16Alpha = 0;
	gsMC_Drive.sFocPMSM.sUAlBeComp.f16Beta = 0;

	gsMC_Drive.sFocPMSM.sUDQReq.f16D = 0;
	gsMC_Drive.sFocPMSM.sUDQReq.f16Q = 0;

	gsMC_Drive.sFocPMSM.sUDQController.f16D = 0;
	gsMC_Drive.sFocPMSM.sUDQController.f16Q = 0;

	gsMC_Drive.sFocPMSM.sAlignment.f32Position = 0;
	gsMC_Drive.sFocPMSM.sAlignment.f32U = 0;
	gsMC_Drive.sFocPMSM.sAlignment.f32Speed = FRAC32(MC_ALIGN_SPEED/N_MAX *2.0*N_MAX/60.0*MC_POLE_PAIRS/MC_FAST_CONTROL_LOOP_FREQ );
	gsMC_Drive.sFocPMSM.sAlignment.f32UStep = FRAC32(MC_ALIGN_VOLT_RAMP / MC_FAST_CONTROL_LOOP_FREQ / U_MAX);
	gsMC_Drive.sFocPMSM.sAlignment.f16IMax = FRAC16(MC_ALIGN_CURRENT / I_MAX);
	gsMC_Drive.sFocPMSM.sAlignment.f16UMax = FRAC16(MC_ALIGN_VOLT_MAX / U_MAX);
	gsMC_Drive.sFocPMSM.sAlignment.uw16TimeAlignment = (uint16_t)(MC_DURATION_TASK_ALIGN * MC_SLOW_CONTROL_LOOP_FREQ);
	gsMC_Drive.sFocPMSM.sAlignment.sIRamp.f32RampUp = FRAC32(MC_ALIGN_CURRENT_RAMP/(1.0L*I_MAX * MC_FAST_CONTROL_LOOP_FREQ));
	gsMC_Drive.sFocPMSM.sAlignment.sIRamp.f32RampDown = FRAC32(MC_ALIGN_CURRENT_RAMP/(1.0L*I_MAX * MC_FAST_CONTROL_LOOP_FREQ));
	gsMC_Drive.sFocPMSM.sAlignment.f32IAlignment = 0;
	gsMC_Drive.sFocPMSM.sAlignment.f16IAlignment = 0;
	
	gsMC_Drive.sFocPMSM.uw16SectorSVM = 2;

	/* Field weakening params */
	gsMC_Drive.sFocFw.sFwErrorFilter.sFltCoeff.f32B0 = FRAC32(MC_FILTER_FW_B0 / 2.0);
	gsMC_Drive.sFocFw.sFwErrorFilter.sFltCoeff.f32B1 = FRAC32(MC_FILTER_FW_B1 / 2.0);
	gsMC_Drive.sFocFw.sFwErrorFilter.sFltCoeff.f32A1 = FRAC32(MC_FILTER_FW_A1 / -2.0);
	GDFLIB_FilterIIR1Init_F16(&gsMC_Drive.sFocFw.sFwErrorFilter);

	gsMC_Drive.sFocFw.sFwPiParams.a32PGain = ACC32(MC_PI_FW_P_GAIN);
	gsMC_Drive.sFocFw.sFwPiParams.a32IGain = ACC32(MC_PI_FW_I_GAIN);
	gsMC_Drive.sFocFw.sFwPiParams.f32IAccK_1 = FRAC32(0.0);
	gsMC_Drive.sFocFw.sFwPiParams.f16UpperLim = 0;
	gsMC_Drive.sFocFw.sFwPiParams.f16LowerLim = FRAC16(-MC_PI_FW_OUTPUT_LIMIT / I_MAX);	

	gsMC_Drive.sFocFw.f16UFwError = FRAC16(0.0);
	gsMC_Drive.sFocFw.f16IFwError = FRAC16(0.0);
	gsMC_Drive.sFocFw.f16FwError = FRAC16(0.0);
	gsMC_Drive.sFocFw.f16FwErrorFilt = FRAC16(0.0);
	
	gsMC_Drive.sFocFw.i16FwPiSatFlag = 0;
	gsMC_Drive.sFocFw.f16SpeedFwOn = FRAC16(MC_SPEED_FW_ON / N_MAX);
	gsMC_Drive.sFocFw.f16ILimit = FRAC16(MC_I_MAX / I_MAX);
	
	/* Observer DQ params */
	gsMC_Drive.sObserverDQ.sBemfObsrv.sIObsrv.f32D = FRAC32(0.0);
	gsMC_Drive.sObserverDQ.sBemfObsrv.sIObsrv.f32Q = FRAC32(0.0);	
	gsMC_Drive.sObserverDQ.sBemfObsrv.sCtrl.f32ID_1 = FRAC32(0.0);
	gsMC_Drive.sObserverDQ.sBemfObsrv.sCtrl.f32IQ_1 = FRAC32(0.0);	
	gsMC_Drive.sObserverDQ.sBemfObsrv.sEObsrv.f32D = FRAC32(0.0);
	gsMC_Drive.sObserverDQ.sBemfObsrv.sEObsrv.f32Q = FRAC32(0.0);	
	
	gsMC_Drive.sObserverDQ.sBemfObsrv.sCtrl.a32PGain	= BEMF_DQ_KP_GAIN;
	gsMC_Drive.sObserverDQ.sBemfObsrv.sCtrl.a32IGain = BEMF_DQ_KI_GAIN;
	
	gsMC_Drive.sObserverDQ.sBemfObsrv.a32IGain = I_SCALE;
	gsMC_Drive.sObserverDQ.sBemfObsrv.a32UGain = U_SCALE;	
	gsMC_Drive.sObserverDQ.sBemfObsrv.a32EGain = E_SCALE;	
	gsMC_Drive.sObserverDQ.sBemfObsrv.a32WIGain = WI_SCALE;	

	gsMC_Drive.sObserverDQ.sTo.f32Theta = FRAC32(0.0);
	gsMC_Drive.sObserverDQ.sTo.f32Speed = FRAC32(0.0);
	gsMC_Drive.sObserverDQ.sTo.f32I_1 = FRAC32(0.0);

	gsMC_Drive.sObserverDQ.sTo.f16PGain = TO_KP_GAIN;
	gsMC_Drive.sObserverDQ.sTo.i16PGainSh = TO_KP_SHIFT;
	gsMC_Drive.sObserverDQ.sTo.f16IGain = TO_KI_GAIN;
	gsMC_Drive.sObserverDQ.sTo.i16IGainSh = TO_KI_SHIFT;
	gsMC_Drive.sObserverDQ.sTo.f16ThGain = TO_THETA_GAIN;
	gsMC_Drive.sObserverDQ.sTo.i16ThGainSh = TO_THETA_SHIFT;
	
	/* Position DQ params */
	gsMC_Drive.sPositionObsDQ.sAnglePosEl.f16Sin = 0;
	gsMC_Drive.sPositionObsDQ.sAnglePosEl.f16Cos = 0;
	gsMC_Drive.sPositionObsDQ.sAnglePosElUpdate.f16Sin = 0;
	gsMC_Drive.sPositionObsDQ.sAnglePosElUpdate.f16Cos = 0;

	gsMC_Drive.sPositionObsDQ.sSpeedEstFilter.sFltCoeff.f32B0 = FRAC32(MC_FILTER_SPEED_EST_B0 / 2.0);
	gsMC_Drive.sPositionObsDQ.sSpeedEstFilter.sFltCoeff.f32B1 = FRAC32(MC_FILTER_SPEED_EST_B1 / 2.0);
	gsMC_Drive.sPositionObsDQ.sSpeedEstFilter.sFltCoeff.f32A1 = FRAC32(MC_FILTER_SPEED_EST_A1 / -2.0);
	GDFLIB_FilterIIR1Init_F16(&gsMC_Drive.sPositionObsDQ.sSpeedEstFilter);

	gsMC_Drive.sPositionObsDQ.f16PositionEl = 0;
	gsMC_Drive.sPositionObsDQ.f16SpeedEstimated = 0;
	gsMC_Drive.sPositionObsDQ.f16SpeedEstimatedFilt = 0;
	
	/* sPositionObsDQ Flag */
	gsMC_Drive.sPositionObsDQ.bStartUp = true;
	gsMC_Drive.sPositionObsDQ.bStartUpFail = false;

	/* Speed params */
	gsMC_Drive.sSpeed.sSpeedPiParams.a32PGain = ACC32(MC_PI_SPEED_P_GAIN);
   	gsMC_Drive.sSpeed.sSpeedPiParams.a32IGain = ACC32(MC_PI_SPEED_I_GAIN);
   	gsMC_Drive.sSpeed.sSpeedPiParams.f32IAccK_1 = 0;
   	gsMC_Drive.sSpeed.sSpeedPiParams.f16UpperLim = FRAC16(MC_PI_SPEED_OUTPUT_LIMIT/I_MAX);
   	gsMC_Drive.sSpeed.sSpeedPiParams.f16LowerLim = FRAC16(-MC_PI_SPEED_OUTPUT_LIMIT/I_MAX); 
   	gsMC_Drive.sSpeed.sSpeedPiParams.bLimFlag = 0;
	gsMC_Drive.sSpeed.i16SpeedPiSatFlag = 0;

	/* 32bit speed ramp */
	gsMC_Drive.sSpeed.s32SpeedRampParams.f32RampUp = FRAC32(SPEED_UP_RAMP_ASR/MC_SLOW_CONTROL_LOOP_FREQ/N_MAX);
	gsMC_Drive.sSpeed.s32SpeedRampParams.f32RampDown = FRAC32(SPEED_DOWN_RAMP_ASR/MC_SLOW_CONTROL_LOOP_FREQ/N_MAX);
	gsMC_Drive.sSpeed.f32SpeedRamp = 0;
	gsMC_Drive.sSpeed.f16SpeedRamp = 0;
	
	gsMC_Drive.sSpeed.f16Speed = 0;
	gsMC_Drive.sSpeed.f16SpeedError = 0;
	gsMC_Drive.sSpeed.f16SpeedReq = 0;
	gsMC_Drive.sSpeed.f16SpeedCmd = 0;
	/* sSpeed Flag */
	gsMC_Drive.sSpeed.bOpenLoop = true;/* MR20150318_A */


	/* Start-up DQ params */
	gsMC_Drive.sPositionObsDQ.sStartUp.uw16CntrForOLStartup= 0; 
	gsMC_Drive.sPositionObsDQ.sStartUp.sSpeedIntegrator.f32IAccK_1 = 0;
	gsMC_Drive.sPositionObsDQ.sStartUp.sSpeedIntegrator.a32Gain = ACC32((2.0*(N_MAX/60.0*MC_POLE_PAIRS))/MC_FAST_CONTROL_LOOP_FREQ);
	gsMC_Drive.sPositionObsDQ.sStartUp.f16IStartUpRamp = 0;
	gsMC_Drive.sPositionObsDQ.sStartUp.f32IStartUpRamp = 0;
	gsMC_Drive.sPositionObsDQ.sStartUp.sIRamp32.f32RampUp =  FRAC32(MC_START_UP_CURRENT_RAMP / MC_FAST_CONTROL_LOOP_FREQ / I_MAX);
	gsMC_Drive.sPositionObsDQ.sStartUp.sIRamp32.f32RampDown =  FRAC32(MC_START_UP_CURRENT_RAMP / MC_FAST_CONTROL_LOOP_FREQ / I_MAX);
	gsMC_Drive.sPositionObsDQ.sStartUp.f32IStartUpDesired =   FRAC32(-MC_START_UP_CURRENT_MAX / I_MAX);
	gsMC_Drive.sPositionObsDQ.sStartUp.f16IStartUpPullOut = FRAC16(MC_START_UP_CURRENT_PULL_OUT / I_MAX);
	gsMC_Drive.sPositionObsDQ.sStartUp.f16PositionPredicted = 0;
	gsMC_Drive.sPositionObsDQ.sStartUp.f16SpeedPredictedRamp = 0;
	gsMC_Drive.sPositionObsDQ.sStartUp.f32SpeedPredictedRamp = 0;
	gsMC_Drive.sPositionObsDQ.sStartUp.f32SpeedStartUpDesired =  FRAC32(-MC_START_UP_SPEED_MAX / N_MAX);
	gsMC_Drive.sPositionObsDQ.sStartUp.sSpdRamp32.f32RampUp = FRAC32(MC_START_UP_SPEED_RAMP / MC_FAST_CONTROL_LOOP_FREQ / N_MAX);
	gsMC_Drive.sPositionObsDQ.sStartUp.sSpdRamp32.f32RampDown = FRAC32(MC_START_UP_SPEED_RAMP / MC_FAST_CONTROL_LOOP_FREQ / N_MAX);
	gsMC_Drive.sPositionObsDQ.sStartUp.f16MergeCoeff = 0;
	gsMC_Drive.sPositionObsDQ.sStartUp.f16MergeStep = FRAC16(MC_START_UP_MERGE_STEP);
	
	gsMC_Drive.sPositionObsDQ.sStartUp.uw16AttemptCntr = 0;
	gsMC_Drive.sPositionObsDQ.sStartUp.uw16AttemptCntrMax = (uint16_t)MC_START_UP_ATTEMPTS;	
	gsMC_Drive.sPositionObsDQ.sStartUp.uw16TimeStartUpFreeWheel = (uint16_t)(MC_DURATION_TASK_INTER_RUN * MC_SLOW_CONTROL_LOOP_FREQ);
	gsMC_Drive.sPositionObsDQ.sStartUp.f16UdStartUpLimit = FRAC16(MC_PI_D_ACR_OUT_LIMIT / U_MAX);
	gsMC_Drive.sPositionObsDQ.sStartUp.f16UqStartUpLimit = FRAC16(MC_PI_Q_ACR_OUT_LIMIT / U_MAX);

	/* state duration */
	gsMC_Drive.uw16CounterSlowLoop = 1;
	gsMC_Drive.uw16DividerSlowLoop = SPEED_LOOP_CNTR;
	gsMC_Drive.uw16CounterState = 0;
	gsMC_Drive.uw16TimeFullSpeedFreeWheel = (uint16_t)(MC_DURATION_TASK_FREE_WHEEL * MC_SLOW_CONTROL_LOOP_FREQ);
	gsMC_Drive.uw16TimeCalibration = (uint16_t)(MC_DURATION_TASK_CALIB * MC_SLOW_CONTROL_LOOP_FREQ);
	gsMC_Drive.uw16TimeFaultRelease = (uint16_t)(MC_DURATION_TASK_FAULT_RELEASE * MC_SLOW_CONTROL_LOOP_FREQ);// 2s

	/* MR20140910_D; MR20150324_B */
	uw32DelayUnderVoltProtectCntr = 0;
	
	
	/* Current vector length filter */
	f16ILength = 0;
	f16ILengthFilt = 0;
	sILengthFilter.a32Acc = 0;
	sILengthFilter.u16Sh = I_LENGTH_MA32_NUMBER;
	GDFLIB_FilterMAInit_F16(0,&sILengthFilter);
	
	/* Software over current protection threshold */
	gsMC_Drive.sFaultThresholds.f16SoftwareOC = MLIB_Mul_F16(FRAC16(SOFTWARE_OC_TH_SHRK/I_MAX),\
													FRAC16(SOFTWARE_OC_TH_SHRK/I_MAX));

		
	/* INIT_DONE command */
	gsMC_Ctrl.uiCtrl |= SM_CTRL_INIT_DONE;
}

#pragma section CODES_IN_RAM begin
/***************************************************************************//*!
*
* @brief   STOP state
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void MC_StateStop(void)
{
//	GPIO_PinClear(BOARD_TP7_GPIO,BOARD_TP7_PIN_MASK);
	/* Disable PWM output */
	MC_DISABLE_PWM_OUTPUT();
	
	/* DC bus voltage filter */
	gsMC_Drive.sFocPMSM.f16UDcBusFilt = gsPFC_Drive.sUCtrl.f16UDcBusFilt;
	
	/* Slow loop control */
	if (--gsMC_Drive.uw16CounterSlowLoop == 0)
	{	
		gsMC_Drive.uw16CounterSlowLoop = gsMC_Drive.uw16DividerSlowLoop;

		if (mbMC_SwitchAppOnOff)
		{
			/* Start command */
			gsMC_Ctrl.uiCtrl |= SM_CTRL_START;
		}

	}

	MC_FaultDetection();
	
	if (gsMC_Drive.sFaultIdPending.R > 0)
	{
		/* Fault state */
		gsMC_Ctrl.uiCtrl |= SM_CTRL_FAULT;
	}
	
	gsMC_Drive.sFocPMSM.uw16SectorSVM = 1;
	/* ADC set up for the next step SVM sector */
	ADCChannelMapping(&gsMC_Drive.sFocPMSM);
}

/***************************************************************************//*!
*
* @brief   RUN state
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void MC_StateRun(void)
{
	/* Run MLIB_Sub_F16-state function */
	mMC_STATE_RUN_TABLE[meMC_StateRun][geMC_StateRunLoop]();

	if ( FAST == geMC_StateRunLoop) /* MR20150324_C */
	{
		/* Detects faults */
		MC_FaultDetection();
	    
	}

	if (!mbMC_SwitchAppOnOff)
	{
		/* Stop command */
		gsMC_Ctrl.uiCtrl |= SM_CTRL_STOP;	
	}

	if (gsMC_Drive.sFaultIdPending.R > 0)
	{
		/* Fault state */
		gsMC_Ctrl.uiCtrl |= SM_CTRL_FAULT;
	}
}

/***************************************************************************//*!
*
* @brief   FAULT to INIT transition
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void MC_TransFaultInit(void)
{

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
static void MC_TransInitFault(void)
{
	MC_DISABLE_PWM_OUTPUT();

	gsMC_Drive.uw16CounterState = gsMC_Drive.uw16TimeFaultRelease;	
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
static void MC_TransInitStop(void)
{

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
static void MC_TransStopFault(void)
{
	/* Disables PWM outputs */
	MC_DISABLE_PWM_OUTPUT();
	
	gsMC_Drive.uw16CounterState = gsMC_Drive.uw16TimeFaultRelease;
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
static void MC_TransStopInit(void)
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
static void MC_TransStopRun(void)
{
	gsMC_Drive.sFocPMSM.uw16SectorSVM = 1;
	
	gsMC_Drive.sFocPMSM.sAllChnlOffsets.f16PhA_ANA = 0;
	gsMC_Drive.sFocPMSM.sAllChnlOffsets.f16PhB_ANB = 0;
	gsMC_Drive.sFocPMSM.sAllChnlOffsets.f16PhC_ANA = 0;
	gsMC_Drive.sFocPMSM.sAllChnlOffsets.f16PhC_ANB = 0;
	gsMC_Drive.sFocPMSM.sAllChnlOffsets.sFilterMA_phA_ANA.a32Acc = 0;
	gsMC_Drive.sFocPMSM.sAllChnlOffsets.sFilterMA_phA_ANA.u16Sh = 6;
	gsMC_Drive.sFocPMSM.sAllChnlOffsets.sFilterMA_phB_ANB.a32Acc = 0;
	gsMC_Drive.sFocPMSM.sAllChnlOffsets.sFilterMA_phB_ANB.u16Sh = 6;
	gsMC_Drive.sFocPMSM.sAllChnlOffsets.sFilterMA_phC_ANA.a32Acc = 0;
	gsMC_Drive.sFocPMSM.sAllChnlOffsets.sFilterMA_phC_ANA.u16Sh = 6;
	gsMC_Drive.sFocPMSM.sAllChnlOffsets.sFilterMA_phC_ANB.a32Acc = 0;
	gsMC_Drive.sFocPMSM.sAllChnlOffsets.sFilterMA_phC_ANB.u16Sh = 6;

    
	gsMC_Drive.sFocPMSM.sAllChnlOffsets.sIChnl.uw16Ph0 = msChannels[gsMC_Drive.sFocPMSM.uw16SectorSVM].uw16Ph0;
	gsMC_Drive.sFocPMSM.sAllChnlOffsets.sIChnl.uw16Ph1 = msChannels[gsMC_Drive.sFocPMSM.uw16SectorSVM].uw16Ph1;

	/* Set duty cycle to charge bootstrap capacitors */
	gsMC_Drive.sFocPMSM.sDutyABC.f16A = FRAC16(0.5);
	gsMC_Drive.sFocPMSM.sDutyABC.f16B = FRAC16(0.5);
	gsMC_Drive.sFocPMSM.sDutyABC.f16C = FRAC16(0.5);

	sHallSpdFilter.f16FltBfrX[0] = 0;
	sHallSpdFilter.f32FltBfrY[0] = 0;
	
	/* PWM update */
	MC_PWM_UPDATE(&gsMC_Drive.sFocPMSM.sDutyABC);
	
	/* Enable PWM output */
//	MC_ENABLE_PWM_OUTPUT();

	/* Required time for ADC calibration */
	gsMC_Drive.uw16CounterState = gsMC_Drive.uw16TimeCalibration;

	/* Init MLIB_Sub_F16-state when transition to RUN */
	meMC_StateRun = CALIB;

	/* Acknowledge that the system can proceed into the RUN state */
	gsMC_Ctrl.uiCtrl |= SM_CTRL_RUN_ACK;
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
static void MC_TransRunFault(void)
{
	/* Disables PWM outputs */
	MC_DISABLE_PWM_OUTPUT();

	gsMC_Drive.sFocPMSM.sIABC.f16A = 0;
	gsMC_Drive.sFocPMSM.sIABC.f16B = 0;
	gsMC_Drive.sFocPMSM.sIABC.f16C = 0;
				
	gsMC_Drive.sFocPMSM.sIDQReq.f16D = 0;
	gsMC_Drive.sFocPMSM.sIDQReq.f16Q = 0;
	gsMC_Drive.sFocPMSM.sUDQReq.f16D = 0;
	gsMC_Drive.sFocPMSM.sUDQReq.f16Q = 0;	

	gsMC_Drive.sFocPMSM.sIAlBe.f16Alpha = 0;
	gsMC_Drive.sFocPMSM.sIAlBe.f16Beta = 0;
	gsMC_Drive.sFocPMSM.sUAlBeReq.f16Alpha = 0;
	gsMC_Drive.sFocPMSM.sUAlBeReq.f16Beta = 0;	
	gsMC_Drive.sFocPMSM.sUAlBeComp.f16Alpha = 0;
	gsMC_Drive.sFocPMSM.sUAlBeComp.f16Beta = 0;	
	gsMC_Drive.uw16CounterState = gsMC_Drive.uw16TimeFaultRelease;	

	gsMC_Drive.sFocPMSM.sIDQ.f16D = 0;
	gsMC_Drive.sFocPMSM.sIDQ.f16Q = 0;
	gsMC_Drive.sSpeed.f32SpeedRamp = 0;/*MR20150317_A*/
	gsMC_Drive.sSpeed.f16SpeedRamp = 0;
	gsMC_Drive.sSpeed.f16Speed = 0;
	gsMC_Drive.sPositionObsDQ.f16SpeedEstimatedFilt	= 0;
	
	/* MR20150324_D */
	gsMC_Drive.sFocFw.f16ILimit = FRAC16(MC_I_MAX / I_MAX);
   	gsMC_Drive.sSpeed.sSpeedPiParams.f16UpperLim = FRAC16(MC_I_MAX/I_MAX);
   	gsMC_Drive.sSpeed.sSpeedPiParams.f16LowerLim = FRAC16(-MC_I_MAX/I_MAX);
    /* MR20150326_A */
   	gsMC_Drive.sFaultIdPending.B.OverLoad = 0; /* clear the overload flag after disable the PWM output, avoid stop at fault state forever */

	gsMC_Drive.sFaultIdPending.B.StartUpFail = 0; 
	
	/* Current length initialization for software over current protection */
	f16ILength = 0;
	f16ILengthFilt = 0;
	GDFLIB_FilterMAInit_F16(0, &sILengthFilter);
	
	/* Pressure relaxation time */
    uw32CounterPressureRelax = (uint32_t)(MC_PRESSURE_RELAX_DURATION_RUNTOFAULT * 60.0 * MC_SLOW_CONTROL_LOOP_FREQ);

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
static void MC_TransRunStop(void)
{
	/* Pressure relaxation time */
	uw32CounterPressureRelax = (uint32_t)(MC_PRESSURE_RELAX_DURATION * 60.0 * MC_SLOW_CONTROL_LOOP_FREQ);

	gsMC_Drive.sSpeed.f16SpeedCmd = 0;//FRAC16(500 * 1.0L / N_MAX);
	
    switch(meMC_StateRun)
    {
        case (CALIB):
		{
			/* Acknowledge that the system can proceed into the STOP state */
			gsMC_Ctrl.uiCtrl |= SM_CTRL_STOP_ACK;		
			ActionOnceFlag = 1;// if no this set, next time speed cmd will 0 ,need to manual input 
			break;
		}
        case (READY):
        {
			/* Acknowledge that the system can proceed into the STOP state */
        	gsMC_Ctrl.uiCtrl |= SM_CTRL_STOP_ACK;			
        	ActionOnceFlag = 1;// if no this set, next time speed cmd will 0 ,need to manual input
			break;
        }
        case (ALIGN):
        {
			/* Disables PWM outputs */
			MC_DISABLE_PWM_OUTPUT();

			gsMC_Drive.sFocPMSM.sIABC.f16A = 0;
			gsMC_Drive.sFocPMSM.sIABC.f16B = 0;
			gsMC_Drive.sFocPMSM.sIABC.f16C = 0;

			gsMC_Drive.sFocPMSM.sIDQReq.f16D = 0;
			gsMC_Drive.sFocPMSM.sIDQReq.f16Q = 0;
			gsMC_Drive.sFocPMSM.sUDQReq.f16D = 0;
			gsMC_Drive.sFocPMSM.sUDQReq.f16Q = 0;	

			gsMC_Drive.sFocPMSM.sIAlBe.f16Alpha = 0;
			gsMC_Drive.sFocPMSM.sIAlBe.f16Beta = 0;
			gsMC_Drive.sFocPMSM.sUAlBeReq.f16Alpha = 0;
			gsMC_Drive.sFocPMSM.sUAlBeReq.f16Beta = 0;	
			gsMC_Drive.sFocPMSM.sUAlBeComp.f16Alpha = 0;
			gsMC_Drive.sFocPMSM.sUAlBeComp.f16Beta = 0;	
			
			
			/* Acknowledge that the system can proceed into the STOP state */
			gsMC_Ctrl.uiCtrl |= SM_CTRL_STOP_ACK;			
			ActionOnceFlag = 1;// if no this set, next time speed cmd will 0 ,need to manual input
			break;
        }

        case (STARTUP):
		{
			/* Generates a time gap before the alignment to assure the rotor is not rotating */
			gsMC_Drive.uw16CounterState = gsMC_Drive.sPositionObsDQ.sStartUp.uw16TimeStartUpFreeWheel;

			/* Disables PWM outputs */
			MC_DISABLE_PWM_OUTPUT();

			gsMC_Drive.sFocPMSM.sIABC.f16A = 0;
			gsMC_Drive.sFocPMSM.sIABC.f16B = 0;
			gsMC_Drive.sFocPMSM.sIABC.f16C = 0;
						
			gsMC_Drive.sFocPMSM.sIDQReq.f16D = 0;
			gsMC_Drive.sFocPMSM.sIDQReq.f16Q = 0;
			gsMC_Drive.sFocPMSM.sUDQReq.f16D = 0;
			gsMC_Drive.sFocPMSM.sUDQReq.f16Q = 0;	

			gsMC_Drive.sFocPMSM.sIAlBe.f16Alpha = 0;
			gsMC_Drive.sFocPMSM.sIAlBe.f16Beta = 0;
			gsMC_Drive.sFocPMSM.sUAlBeReq.f16Alpha = 0;
			gsMC_Drive.sFocPMSM.sUAlBeReq.f16Beta = 0;	
			gsMC_Drive.sFocPMSM.sUAlBeComp.f16Alpha = 0;
			gsMC_Drive.sFocPMSM.sUAlBeComp.f16Beta = 0;	
			ActionOnceFlag = 1;// if no this set, next time speed cmd will 0 ,need to manual input
			meMC_StateRun = FREEWHEEL;
			
			break;
		}

        case (SPIN):
		{
			/* Generates a time gap before the alignment to assure the rotor is not rotating */
			gsMC_Drive.uw16CounterState = (uint16_t)MLIB_MulSat_F16(MLIB_Abs_F16(gsMC_Drive.sSpeed.f16Speed), (frac16_t)gsMC_Drive.uw16TimeFullSpeedFreeWheel);				

			/* Disables PWM outputs */
			MC_DISABLE_PWM_OUTPUT();

			gsMC_Drive.sFocPMSM.sIABC.f16A = 0;
			gsMC_Drive.sFocPMSM.sIABC.f16B = 0;
			gsMC_Drive.sFocPMSM.sIABC.f16C = 0;
						
			gsMC_Drive.sFocPMSM.sIDQReq.f16D = 0;
			gsMC_Drive.sFocPMSM.sIDQReq.f16Q = 0;
			gsMC_Drive.sFocPMSM.sUDQReq.f16D = 0;
			gsMC_Drive.sFocPMSM.sUDQReq.f16Q = 0;	

			gsMC_Drive.sFocPMSM.sIAlBe.f16Alpha = 0;
			gsMC_Drive.sFocPMSM.sIAlBe.f16Beta = 0;
			gsMC_Drive.sFocPMSM.sUAlBeReq.f16Alpha = 0;
			gsMC_Drive.sFocPMSM.sUAlBeReq.f16Beta = 0;	
			gsMC_Drive.sFocPMSM.sUAlBeComp.f16Alpha = 0;
			gsMC_Drive.sFocPMSM.sUAlBeComp.f16Beta = 0;	
			ActionOnceFlag = 1;
			meMC_StateRun = FREEWHEEL;
			
			break;
		}

        case (FREEWHEEL):
		{
			if (gsMC_Drive.uw16CounterState == 0)
			{
				/* Disables PWM outputs */
				MC_DISABLE_PWM_OUTPUT();

				gsMC_Drive.sFocPMSM.sIABC.f16A = 0;
				gsMC_Drive.sFocPMSM.sIABC.f16B = 0;
				gsMC_Drive.sFocPMSM.sIABC.f16C = 0;
				gsMC_Drive.sFocPMSM.sIDQReq.f16D = 0;
				gsMC_Drive.sFocPMSM.sIDQReq.f16Q = 0;
				gsMC_Drive.sFocPMSM.sUDQReq.f16D = 0;
				gsMC_Drive.sFocPMSM.sUDQReq.f16Q = 0;	

				gsMC_Drive.sFocPMSM.sIAlBe.f16Alpha = 0;
				gsMC_Drive.sFocPMSM.sIAlBe.f16Beta = 0;
				gsMC_Drive.sFocPMSM.sUAlBeReq.f16Alpha = 0;
				gsMC_Drive.sFocPMSM.sUAlBeReq.f16Beta = 0;	
				gsMC_Drive.sFocPMSM.sUAlBeComp.f16Alpha = 0;
				gsMC_Drive.sFocPMSM.sUAlBeComp.f16Beta = 0;	
				ActionOnceFlag = 1;
				/* Acknowledge that the system can proceed into the STOP state */
				gsMC_Ctrl.uiCtrl |= SM_CTRL_STOP_ACK;			

				break;
			}
		}
    }
}

/***************************************************************************//*!
*
* @brief   RUN CALIB MLIB_Sub_F16-state
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void MC_StateRunCalib(void)
{
	/* DC bus voltage filter */
	gsMC_Drive.sFocPMSM.f16UDcBusFilt = gsPFC_Drive.sUCtrl.f16UDcBusFilt;
	
	/* Filter the offsets of each ADC Channel */
	MCSTRUC_ChlOffsetCalib(&gsMC_Drive.sFocPMSM);

	
	/* ADC set up for the next step SVM sector */
	ADCChannelMapping(&gsMC_Drive.sFocPMSM);

}

/***************************************************************************//*!
*
* @brief   RUN CALIB MLIB_Sub_F16-state for Slow loop
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void MC_StateRunCalibSlow(void)
{

	/* Slow loop control */
	if (--gsMC_Drive.uw16CounterSlowLoop == 0)
	{
		gsMC_Drive.uw16CounterSlowLoop = gsMC_Drive.uw16DividerSlowLoop;
		
		if(++gsMC_Drive.sFocPMSM.uw16SectorSVM > 6)
			gsMC_Drive.sFocPMSM.uw16SectorSVM = 1;

		if (--gsMC_Drive.uw16CounterState == 0)
		{
			/* Transition to the RUN READY MLIB_Sub_F16-state */
			MC_TransRunCalibReady();	
		}
		
	}
}

/***************************************************************************//*!
*
* @brief   RUN READY MLIB_Sub_F16-state
*
* @param   void
*
* @return  none
*
******************************************************************************/

static void MC_StateRunReady(void)
{
	/* DC bus voltage filter */
	gsMC_Drive.sFocPMSM.f16UDcBusFilt = gsPFC_Drive.sUCtrl.f16UDcBusFilt;

	/* Configure offsets for current measurements of last step */
	ADCOffsetConfig(&gsMC_Drive.sFocPMSM); 

	/* Reads the 3-phase current */
	GetCurrent(&gsMC_Drive.sFocPMSM);
	
	gsMC_Drive.sFocPMSM.uw16SectorSVM = 2;

	/* ADC set up for the next step SVM sector */
	ADCChannelMapping(&gsMC_Drive.sFocPMSM);

	gsMC_Drive.sFocPMSM.sIDQReq.f16D = 0;
	gsMC_Drive.sFocPMSM.sIDQReq.f16Q = 0;
	gsMC_Drive.sFocPMSM.sUDQReq.f16D = 0;
	gsMC_Drive.sFocPMSM.sUDQReq.f16Q = 0;

	gsMC_Drive.sPositionObsDQ.bStartUp = true;

	gsMC_Drive.sSpeed.f16SpeedReq = 0;
	gsMC_Drive.sSpeed.f16Speed = 0;
	
	if(w16OpenLpTstFlag == 1)
	{
		MCSTRUC_PMSMOpenLoopStartUp(&gsMC_Drive.sFocPMSM, &gsMC_Drive.sPositionObsDQ, &gsMC_Drive.sSpeed);	
	}
	
	if(w16PWMTestFlag == 1)
	{
		MC_ENABLE_PWM_OUTPUT();
	}
	else
	{
		MC_DISABLE_PWM_OUTPUT();
	}
	
	/* PWM update */
	MC_PWM_UPDATE(&gsMC_Drive.sFocPMSM.sDutyABC);

	f16SpeedHallFilt = GDFLIB_FilterIIR1_F16(f16SpeedHall, &sHallSpdFilter);
}

/***************************************************************************//*!
*
* @brief   RUN READY MLIB_Sub_F16-state for Slow loop
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void MC_StateRunReadySlow(void)
{
	/* Slow loop control */
	if (--gsMC_Drive.uw16CounterSlowLoop == 0)
	{
		gsMC_Drive.uw16CounterSlowLoop = gsMC_Drive.uw16DividerSlowLoop;

		/* Waits until the pressure gets low */
		if (uw32CounterPressureRelax > 0)
		{
			uw32CounterPressureRelax--;
		}
		
		/* Enter stop state or start up */
		if(faultRecordFlag.R == 0)
		{
			if ((!(0 == gsMC_Drive.sSpeed.f16SpeedCmd)) && (0 == uw32CounterPressureRelax))
			{
				
				/* Transition to the RUN ALIGN state */
				MC_TransRunReadyAlign();
			}
		}
		else
		{
			LED_Indication();
		}
		 
	}
	
}

/***************************************************************************//*!
*
* @brief   RUN ALIGN MLIB_Sub_F16-state
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void MC_StateRunAlign(void)
{
	/* DC bus voltage filter */
	gsMC_Drive.sFocPMSM.f16UDcBusFilt = gsPFC_Drive.sUCtrl.f16UDcBusFilt;

	/* Configure offsets for current measurements of last step */
	ADCOffsetConfig(&gsMC_Drive.sFocPMSM);

	/* Reads the 3-phase current */
	GetCurrent(&gsMC_Drive.sFocPMSM);
   
	/* Charge bootstrap capacitor with small current before alignment *//*MR20150312_B*/
	if ( gsMC_Drive.uw16CounterState >= MLIB_Sub_F16(gsMC_Drive.sFocPMSM.sAlignment.uw16TimeAlignment,\
			(uint16_t)(MC_BOOTSTRAP_CHARGE * MC_SLOW_CONTROL_LOOP_FREQ)) )
	{
		gsMC_Drive.sFocPMSM.sDutyABC.f16A = FRAC16(BOOTSTRAP_CHARGE_DUTY);
		gsMC_Drive.sFocPMSM.sDutyABC.f16B = FRAC16(BOOTSTRAP_CHARGE_DUTY);
		gsMC_Drive.sFocPMSM.sDutyABC.f16C = FRAC16(BOOTSTRAP_CHARGE_DUTY);
	}
	else
	{
		if ( gsMC_Drive.uw16CounterState >= MLIB_Sub_F16(gsMC_Drive.sFocPMSM.sAlignment.uw16TimeAlignment,\
					(uint16_t)(MC_ALIGN_STAGE_1_TIME * MC_SLOW_CONTROL_LOOP_FREQ)) )
		{
			gsMC_Drive.sFocPMSM.sAlignment.f32Position = FRAC32(MC_ALIGN_POSITION_1/180.0);
		}
		else
		{
			gsMC_Drive.sFocPMSM.sAlignment.f32Position = FRAC32(MC_ALIGN_POSITION_2/180.0);
		}
		/* Alignment */
		gsMC_Drive.sFocPMSM.sAlignment.f32IAlignment = \
				GFLIB_Ramp_F32(MLIB_Conv_F32s(gsMC_Drive.sFocPMSM.sAlignment.f16IMax),&gsMC_Drive.sFocPMSM.sAlignment.sIRamp);
		gsMC_Drive.sFocPMSM.sIDQReq.f16Q = MLIB_Conv_F16l(gsMC_Drive.sFocPMSM.sAlignment.f32IAlignment);
		gsMC_Drive.sFocPMSM.sIDQReq.f16D = 0;
		MCSTRUC_AlignmentPMSM(&gsMC_Drive.sFocPMSM);
	}
	
	
	/* PWM update */
	MC_PWM_UPDATE(&gsMC_Drive.sFocPMSM.sDutyABC);
	
	/* ADC set up for the next step SVM sector */
	ADCChannelMapping(&gsMC_Drive.sFocPMSM);

	
	DACA->FMT0.DATAREG_FMT0 = MLIB_Conv_F16l(gsMC_Drive.sFocPMSM.sAlignment.f32Position)>>4;
//	DACA->FMT0.DATAREG_FMT0 = (gsMC_Drive.sFocPMSM.sDutyABC.f16A >> 4);
}

/***************************************************************************//*!
*
* @brief   RUN ALIGN MLIB_Sub_F16-state for Slow loop
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void MC_StateRunAlignSlow(void)
{
	/* Slow loop control */
	if (--gsMC_Drive.uw16CounterSlowLoop == 0)
	{
		gsMC_Drive.uw16CounterSlowLoop = gsMC_Drive.uw16DividerSlowLoop;


		if (--gsMC_Drive.uw16CounterState == 0)
		{
			/* Transition to the RUN STARTUP MLIB_Sub_F16-state */
			MC_TransRunAlignStartup();
			
			//================ Alignment Test =====================
//			MC_TransRunAlignReady();
//			gsMC_Drive.sSpeed.f16SpeedCmd = 0;
			//================ Alignment Test End =====================
			
		}

		if (gsMC_Drive.sSpeed.f16SpeedCmd == 0)
		{
			/* Transition to the RUN READY MLIB_Sub_F16-state */
			MC_TransRunAlignReady();
		}
	}
}

/***************************************************************************//*!
*
* @brief   RUN STARTUP MLIB_Sub_F16-state
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void MC_StateRunStartup(void)
{
	/* DC bus voltage filter */
	gsMC_Drive.sFocPMSM.f16UDcBusFilt = gsPFC_Drive.sUCtrl.f16UDcBusFilt;

	/* Configure offsets for current measurements of last step */
	ADCOffsetConfig(&gsMC_Drive.sFocPMSM);

	/* Reads the 3-phase current */
	GetCurrent(&gsMC_Drive.sFocPMSM);
	
	if(MLIB_Abs_F32(gsMC_Drive.sPositionObsDQ.sStartUp.f32SpeedPredictedRamp) > MLIB_Abs_F32(gsMC_Drive.sPositionObsDQ.sStartUp.f32SpeedStartUpDesired>>1))
	{
		/* DQ observer */
		MCSTRUC_PMSMPositionObsDQ(&gsMC_Drive.sFocPMSM, &gsMC_Drive.sObserverDQ, &gsMC_Drive.sPositionObsDQ);
		gsMC_Drive.sSpeed.f16Speed = gsMC_Drive.sPositionObsDQ.f16SpeedEstimatedFilt;
	}


	/* open loop startup */
	MCSTRUC_PMSMOpenLoopStartUp(&gsMC_Drive.sFocPMSM, &gsMC_Drive.sPositionObsDQ, &gsMC_Drive.sSpeed);	

    /* MR20150316_B */
	if ( false == gsMC_Drive.sPositionObsDQ.bStartUp)
	{
		/* Sub-state RUN Spin */
		MC_TransRunStartupSpin();
		
		if (gsMC_Drive.sPositionObsDQ.bStartUpFail)
		{
			/* Sub-state RUN Freewheel */
			MC_TransRunStartupFreewheel(); // gsMC_Drive.sPositionObsDQ.bStartUpFail won't be set during start-up.
		}
	}

	/* FOC */
	MCSTRUC_FocPMSMCurrentCtrl(&gsMC_Drive.sFocPMSM);

	/* PWM update */
	MC_PWM_UPDATE(&gsMC_Drive.sFocPMSM.sDutyABC);

	/* ADC set up for the next step SVM sector */
	ADCChannelMapping(&gsMC_Drive.sFocPMSM);
	
	DACA->FMT0.DATAREG_FMT0 = gsMC_Drive.sPositionObsDQ.sStartUp.f16PositionPredicted>>4;
	
	f16SpeedHallFilt = GDFLIB_FilterIIR1_F16(f16SpeedHall, &sHallSpdFilter);
}

/***************************************************************************//*!
*
* @brief   RUN STARTUP MLIB_Sub_F16-state for Slow loop
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void MC_StateRunStartupSlow(void)
{
	  /* Slow loop control */
	if (--gsMC_Drive.uw16CounterSlowLoop == 0)
	{
		gsMC_Drive.uw16CounterSlowLoop = gsMC_Drive.uw16DividerSlowLoop;

		if (gsMC_Drive.sSpeed.f16SpeedCmd == 0)
		{
			/* Sub-state RUN Freewheel */
			MC_TransRunStartupFreewheel();
		}

	}
}

/***************************************************************************//*!
*
* @brief   RUN SPIN MLIB_Sub_F16-state
*
* @param   void
*
* @return  none
*
******************************************************************************/

static void MC_StateRunSpin(void)
{
    frac16_t f16Temp;

	/* DC bus voltage filter */
    gsMC_Drive.sFocPMSM.f16UDcBusFilt = gsPFC_Drive.sUCtrl.f16UDcBusFilt;

	/* Configure offsets for current measurements of last step */
	ADCOffsetConfig(&gsMC_Drive.sFocPMSM);

	/* Reads the 3-phase current */
	GetCurrent(&gsMC_Drive.sFocPMSM);
	
	/* Current length calculation for over current software protection */
	f16ILength = MLIB_Add_F16(MLIB_Mul_F16(gsMC_Drive.sFocPMSM.sIAlBe.f16Alpha,gsMC_Drive.sFocPMSM.sIAlBe.f16Alpha),\
				     MLIB_Mul_F16(gsMC_Drive.sFocPMSM.sIAlBe.f16Beta,gsMC_Drive.sFocPMSM.sIAlBe.f16Beta));
	
	f16ILengthFilt = GDFLIB_FilterMA_F16(f16ILength,&sILengthFilter);

	/* DQ observer */
	MCSTRUC_PMSMPositionObsDQ(&gsMC_Drive.sFocPMSM, &gsMC_Drive.sObserverDQ, &gsMC_Drive.sPositionObsDQ);
	gsMC_Drive.sSpeed.f16Speed = gsMC_Drive.sPositionObsDQ.f16SpeedEstimatedFilt;
	
	/* Pass the estimated position to current loop */
	gsMC_Drive.sFocPMSM.sAnglePosElUpdate = gsMC_Drive.sPositionObsDQ.sAnglePosElUpdate;
	gsMC_Drive.sFocPMSM.sAnglePosEl = gsMC_Drive.sPositionObsDQ.sAnglePosEl;
		
	/*Call FOC function*/
	MCSTRUC_FocPMSMCurrentCtrl(&gsMC_Drive.sFocPMSM);

#if DEADTIME_COM == 1	
	if(gsMC_Drive.sFocPMSM.sDutyABC.f16A > 0)
	{
		if(MLIB_Abs_F16(gsMC_Drive.sFocPMSM.sIABC.f16A) > FRAC16(DTC_I_THRESHOLD/I_MAX))
		{
			if(gsMC_Drive.sFocPMSM.sIABC.f16A > 0)
			{
				gsMC_Drive.sFocPMSM.sDutyABC.f16A = MLIB_Add_F16(gsMC_Drive.sFocPMSM.sDutyABC.f16A,FRAC16(DTC_DUTY));
			}
			else
			{
				if(gsMC_Drive.sFocPMSM.sDutyABC.f16A > FRAC16(DTC_DUTY))
				{
					gsMC_Drive.sFocPMSM.sDutyABC.f16A = MLIB_Sub_F16(gsMC_Drive.sFocPMSM.sDutyABC.f16A,FRAC16(DTC_DUTY));
				}
				else
				{
					gsMC_Drive.sFocPMSM.sDutyABC.f16A = 0;
				}
			}
		}
	}
	if(gsMC_Drive.sFocPMSM.sDutyABC.f16B > 0)
	{
		if(MLIB_Abs_F16(gsMC_Drive.sFocPMSM.sIABC.f16B) > FRAC16(DTC_I_THRESHOLD/I_MAX))
		{
			if(gsMC_Drive.sFocPMSM.sIABC.f16B > 0)
			{
				gsMC_Drive.sFocPMSM.sDutyABC.f16B = MLIB_Add_F16(gsMC_Drive.sFocPMSM.sDutyABC.f16B,FRAC16(DTC_DUTY));
			}
			else
			{
				if(gsMC_Drive.sFocPMSM.sDutyABC.f16B > FRAC16(DTC_DUTY))
				{
					gsMC_Drive.sFocPMSM.sDutyABC.f16B = MLIB_Sub_F16(gsMC_Drive.sFocPMSM.sDutyABC.f16B,FRAC16(DTC_DUTY));
				}
				else
				{
					gsMC_Drive.sFocPMSM.sDutyABC.f16B = 0;
				}
			}
		}
	}
	if(gsMC_Drive.sFocPMSM.sDutyABC.f16C > 0)
	{
		if(MLIB_Abs_F16(gsMC_Drive.sFocPMSM.sIABC.f16C) > FRAC16(DTC_I_THRESHOLD/I_MAX))
		{
			if(gsMC_Drive.sFocPMSM.sIABC.f16C > 0)
			{
				gsMC_Drive.sFocPMSM.sDutyABC.f16C = MLIB_Add_F16(gsMC_Drive.sFocPMSM.sDutyABC.f16C,FRAC16(DTC_DUTY));
			}
			else
			{
				if(gsMC_Drive.sFocPMSM.sDutyABC.f16C > FRAC16(DTC_DUTY))
				{
					gsMC_Drive.sFocPMSM.sDutyABC.f16C = MLIB_Sub_F16(gsMC_Drive.sFocPMSM.sDutyABC.f16C,FRAC16(DTC_DUTY));
				}
				else
				{
					gsMC_Drive.sFocPMSM.sDutyABC.f16C = 0;
				}
			}
		}
	}
#endif
	
	/* PWM update */
	MC_PWM_UPDATE(&gsMC_Drive.sFocPMSM.sDutyABC);

	/* ADC set up for the next step SVM sector */
	ADCChannelMapping(&gsMC_Drive.sFocPMSM);
	
	f16SpeedHallFilt = GDFLIB_FilterIIR1_F16(f16SpeedHall, &sHallSpdFilter);
}

/***************************************************************************//*!
*
* @brief   RUN SPIN MLIB_Sub_F16-state for Slow loop
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void MC_StateRunSpinSlow(void)
{
	/* begin of Slow loop control */
	if (--gsMC_Drive.uw16CounterSlowLoop == 0)
	{
		gsMC_Drive.uw16CounterSlowLoop = gsMC_Drive.uw16DividerSlowLoop;


		/*========================================== Field weakening control ======================================*/
		
		MCSTRUC_FocFieldWeakeningCtrl(&gsMC_Drive.sFocPMSM, &gsMC_Drive.sFocFw, &gsMC_Drive.sSpeed);

		/*============================================  Speed closed loop =========================================*/
		
		/* Speed ramp generation */
	    gsMC_Drive.sSpeed.f32SpeedRamp = GFLIB_Ramp_F32((frac32_t)gsMC_Drive.sSpeed.f16SpeedReq<<16, &gsMC_Drive.sSpeed.s32SpeedRampParams);
	    gsMC_Drive.sSpeed.f16SpeedRamp = MLIB_Conv_F16l(gsMC_Drive.sSpeed.f32SpeedRamp);
	    /* speed error */
	    gsMC_Drive.sSpeed.f16SpeedError = MLIB_Sub_F16(gsMC_Drive.sSpeed.f16SpeedRamp, gsMC_Drive.sSpeed.f16Speed);
		/* ASR */
		gsMC_Drive.sFocPMSM.sIDQReq.f16Q = GFLIB_CtrlPIpAW_F16(gsMC_Drive.sSpeed.f16SpeedError, &gsMC_Drive.sSpeed.i16SpeedPiSatFlag, &gsMC_Drive.sSpeed.sSpeedPiParams);			
		/* PI controller saturation control */
		gsMC_Drive.sSpeed.i16SpeedPiSatFlag = gsMC_Drive.sSpeed.sSpeedPiParams.bLimFlag;
		
        /* transfer from spin to free-wheel */
		if ((0 == gsMC_Drive.sSpeed.f16SpeedCmd)||MLIB_Mul_F16(gsMC_Drive.sSpeed.f16SpeedCmd,gsMC_Drive.sSpeed.f16SpeedRamp)<0)
	    {
			if(MLIB_Abs_F16(gsMC_Drive.sSpeed.f16SpeedRamp) <= FRAC16(SPEED_STOP_THRHLD / N_MAX)) // Stop immediately
			{
				MC_TransRunSpinFreewheel(); // transfer from spin to free-wheel 
			}
	    }
	}
	/*end of Slow loop control*/
}

/***************************************************************************//*!
*
* @brief   RUN FREEWHEEL MLIB_Sub_F16-state
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void MC_StateRunFreewheel(void)
{
	/* DC bus voltage filter */
	gsMC_Drive.sFocPMSM.f16UDcBusFilt = gsPFC_Drive.sUCtrl.f16UDcBusFilt;

	/* Configure offsets for current measurements of last step */
	ADCOffsetConfig(&gsMC_Drive.sFocPMSM);
	
	/* Reads the 3-phase current */
	GetCurrent(&gsMC_Drive.sFocPMSM);

	gsMC_Drive.sFocPMSM.sDutyABC.f16A = 0;
	gsMC_Drive.sFocPMSM.sDutyABC.f16B = 0;
	gsMC_Drive.sFocPMSM.sDutyABC.f16C = 0;

	/* PWM update */
	MC_PWM_UPDATE(&gsMC_Drive.sFocPMSM.sDutyABC);

	gsMC_Drive.sFocPMSM.uw16SectorSVM = 2;

	/* ADC set up for the next step SVM sector */
	ADCChannelMapping(&gsMC_Drive.sFocPMSM);

}

/***************************************************************************//*!
*
* @brief   RUN FREEWHEEL MLIB_Sub_F16-state for Slow loop
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void MC_StateRunFreewheelSlow(void)
{
	/* Slow loop control */
	if (--gsMC_Drive.uw16CounterSlowLoop == 0)
	{
		gsMC_Drive.uw16CounterSlowLoop = gsMC_Drive.uw16DividerSlowLoop;

		if (--gsMC_Drive.uw16CounterState == 0)
		{
			gsMC_Drive.sSpeed.f16Speed = 0;
			
			/* If app switch is on */
			if (mbMC_SwitchAppOnOff)
			{
				/* If speed command is non-zero */ 
				if (MLIB_Abs_F16(gsMC_Drive.sSpeed.f16SpeedCmd > 0))
				{
					/* If the motor has been in the start-up phase */
					if (gsMC_Drive.sPositionObsDQ.bStartUpFail)
					{
						/* Check the start-up attempt counter */
						if (gsMC_Drive.sPositionObsDQ.sStartUp.uw16AttemptCntr++ >= gsMC_Drive.sPositionObsDQ.sStartUp.uw16AttemptCntrMax) /* enter into Fault */
						{
							/* If exceeds the allowed sequence of start-ups goes to fault */
							gsMC_Drive.sFaultIdPending.B.StartUpFail = 1;
						    faultRecordFlag.B.StartUpFailRecord = 1;
							/* Pressure relaxation time */
//							uw32CounterPressureRelax = (uint32_t)(MC_PRESSURE_RELAX_DURATION_REPSTARTUP * 60.0 * MC_SLOW_CONTROL_LOOP_FREQ);
						}
						else /* enter into attempting to startup */
						{
							/* Pressure relaxation time */
							uw32CounterPressureRelax = (uint32_t)(MC_PRESSURE_RELAX_DURATION_ATTEMPT_STARTUP * MC_SLOW_CONTROL_LOOP_FREQ);/* MR20150331_B */
							/* Sub-state RUN READY */
							MC_TransRunFreewheelReady();/* MR20150331_B */
						}
					}
					/* If the motor has been in the run phase */
					else
					{
						/* Sub-state RUN READY , this path seems to be invalid  */
						MC_TransRunFreewheelReady();
					}
				}
				/* If speed command is zero */
				else
				{
					/* Sub-state RUN READY */
					MC_TransRunFreewheelReady();	
				}
			}
		}
	}
	
}

/***************************************************************************//*!
*
* @brief   RUN CALIB to READY transition
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void MC_TransRunCalibReady(void)
{
	/* Disable PWM output */
	MC_DISABLE_PWM_OUTPUT();

	/* Sub-state RUN READY */
	meMC_StateRun = READY;
}

/***************************************************************************//*!
*
* @brief   RUN READY to ALIGN transition
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void MC_TransRunReadyAlign(void)
{
	
	/* set up the Id for alignment   */
	gsMC_Drive.sFocPMSM.sIDQReq.f16D = 0;
	gsMC_Drive.sFocPMSM.sIDQReq.f16Q = 0;
	gsMC_Drive.sFocPMSM.sAlignment.f32IAlignment = 0; // ramp value
	gsMC_Drive.sFocPMSM.sAlignment.f16IAlignment = 0;
	gsMC_Drive.sFocPMSM.sAlignment.f16IMax = FRAC16(MC_ALIGN_CURRENT/I_MAX); // desired value
	gsMC_Drive.sFocPMSM.sAlignment.sIRamp.f32State = 0;
	
	/* Alignment voltage init point */
	gsMC_Drive.sFocPMSM.sAlignment.f32U = 0;
	
	/* Alignment voltage init position */
	gsMC_Drive.sFocPMSM.sAlignment.f32Position = FRAC32(MC_ALIGN_POSITION_1/180.0);

	/* Alignment duration set-up */
	gsMC_Drive.uw16CounterState = gsMC_Drive.sFocPMSM.sAlignment.uw16TimeAlignment;
	gsMC_Drive.sFocPMSM.sDutyABC.f16A = FRAC16(BOOTSTRAP_CHARGE_DUTY);
	gsMC_Drive.sFocPMSM.sDutyABC.f16B = FRAC16(BOOTSTRAP_CHARGE_DUTY);
	gsMC_Drive.sFocPMSM.sDutyABC.f16C = FRAC16(BOOTSTRAP_CHARGE_DUTY);

	gsMC_Drive.sFocPMSM.sIABC.f16A = 0;
	gsMC_Drive.sFocPMSM.sIABC.f16B = 0;
	gsMC_Drive.sFocPMSM.sIABC.f16C = 0;
	
	gsMC_Drive.sSpeed.f32SpeedRamp = 0; /*MR20150317_A*/
	gsMC_Drive.sSpeed.f16SpeedRamp = 0;
	gsMC_Drive.sSpeed.s32SpeedRampParams.f32State = 0;
	
	/* PMSM FOC params for alignment only because of Fault*/
  	gsMC_Drive.sFocPMSM.sIdPiParams.a32PGain = D_KP_GAIN;
   	gsMC_Drive.sFocPMSM.sIdPiParams.a32IGain = D_KI_GAIN;
   	gsMC_Drive.sFocPMSM.sIdPiParams.f32IAccK_1 = 0;
   	gsMC_Drive.sFocPMSM.sIdPiParams.f16UpperLim = FRAC16(VOLTAGE_ALIGN_MAX / U_MAX);  
   	gsMC_Drive.sFocPMSM.sIdPiParams.f16LowerLim = FRAC16(-VOLTAGE_ALIGN_MAX / U_MAX);
   	gsMC_Drive.sFocPMSM.sIdPiParams.bLimFlag = 0;

	gsMC_Drive.sFocPMSM.sIqPiParams.a32PGain = (Q_KP_GAIN);
   	gsMC_Drive.sFocPMSM.sIqPiParams.a32IGain = (Q_KI_GAIN);
   	gsMC_Drive.sFocPMSM.sIqPiParams.f32IAccK_1 = 0;
   	gsMC_Drive.sFocPMSM.sIqPiParams.f16UpperLim = FRAC16(VOLTAGE_ALIGN_MAX / U_MAX);  
   	gsMC_Drive.sFocPMSM.sIqPiParams.f16LowerLim = FRAC16(-VOLTAGE_ALIGN_MAX / U_MAX);    
   	gsMC_Drive.sFocPMSM.sIqPiParams.bLimFlag = 0;
	
   	gsMC_Drive.sPositionObsDQ.sStartUp.sSpdRamp32.f32State = 0;

	gsMC_Drive.sPositionObsDQ.sStartUp.f16PositionPredictedOld = 0;
	uw32DelayUnderVoltProtectCntr = 0;/* MR20140910_D; MR20150324_B */

	/* Enable PWM output */
	MC_ENABLE_PWM_OUTPUT();
	
	/* Sub-state RUN ALIGN */
	meMC_StateRun = ALIGN;
	

}

/***************************************************************************//*!
*
* @brief   RUN ALIGN to STARTUP transition
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void MC_TransRunAlignStartup(void)
{
	gsMC_Drive.sFocPMSM.i16IdPiSatFlag = 0;
	gsMC_Drive.sFocPMSM.i16IqPiSatFlag = 0;

   	gsMC_Drive.sFocPMSM.sIdPiParams.f16UpperLim = gsMC_Drive.sPositionObsDQ.sStartUp.f16UdStartUpLimit;
   	gsMC_Drive.sFocPMSM.sIdPiParams.f16LowerLim = MLIB_Neg_F16(gsMC_Drive.sPositionObsDQ.sStartUp.f16UdStartUpLimit);
   	
   	gsMC_Drive.sFocPMSM.sIqPiParams.f16UpperLim = gsMC_Drive.sPositionObsDQ.sStartUp.f16UqStartUpLimit;
   	gsMC_Drive.sFocPMSM.sIqPiParams.f16LowerLim = MLIB_Neg_F16(gsMC_Drive.sPositionObsDQ.sStartUp.f16UqStartUpLimit);
	
	gsMC_Drive.sPositionObsDQ.sStartUp.f16IStartUpRamp = 0;
   	gsMC_Drive.sPositionObsDQ.sStartUp.f32IStartUpRamp = 0;
	
	gsMC_Drive.sPositionObsDQ.f16SpeedEstimated = 0;
	gsMC_Drive.sPositionObsDQ.sStartUp.f16SpeedPredictedRamp = 0;
	gsMC_Drive.sPositionObsDQ.sStartUp.f32SpeedPredictedRamp = 0;
	gsMC_Drive.sPositionObsDQ.sStartUp.sSpdRamp32.f32State = 0;

	gsMC_Drive.sPositionObsDQ.sStartUp.uw16CntrForOLStartup= 0;

	gsMC_Drive.sObserverDQ.sBemfObsrv.sIObsrv.f32D = FRAC32(0.0);
	gsMC_Drive.sObserverDQ.sBemfObsrv.sIObsrv.f32Q = FRAC32(0.0);
			
	gsMC_Drive.sObserverDQ.sBemfObsrv.sCtrl.f32ID_1 = FRAC32(0.0);
	gsMC_Drive.sObserverDQ.sBemfObsrv.sCtrl.f32IQ_1 = FRAC32(0.0);
	gsMC_Drive.sObserverDQ.sBemfObsrv.sEObsrv.f32D = FRAC32(0.0);
	gsMC_Drive.sObserverDQ.sBemfObsrv.sEObsrv.f32Q = FRAC32(0.0);	

	gsMC_Drive.sObserverDQ.sTo.f32Theta = FRAC32(0.0);
	gsMC_Drive.sObserverDQ.sTo.f32Speed = FRAC32(0.0);
	gsMC_Drive.sObserverDQ.sTo.f32I_1 = FRAC32(0.0);
	
	GDFLIB_FilterIIR1Init_F16(&gsMC_Drive.sPositionObsDQ.sSpeedEstFilter);

	gsMC_Drive.sPositionObsDQ.f16SpeedEstimatedFilt = FRAC16(0.0);

	gsMC_Drive.sSpeed.f32SpeedRamp = 0;/*MR20150317_A*/
	gsMC_Drive.sSpeed.f16SpeedRamp = FRAC16(0.0);
	gsMC_Drive.sSpeed.f16SpeedReq = FRAC16(0.0);
	gsMC_Drive.sSpeed.f16SpeedError = FRAC16(0.0);
	gsMC_Drive.sSpeed.f16Speed = FRAC16(0.0);
	gsMC_Drive.sSpeed.i16SpeedPiSatFlag = 0;
	gsMC_Drive.sSpeed.bOpenLoop = true;/*MR20150318_A*/
	gsMC_Drive.sSpeed.sSpeedPiParams.f16UpperLim = FRAC16(MC_PI_SPEED_OUTPUT_LIMIT/I_MAX);
	gsMC_Drive.sSpeed.sSpeedPiParams.f16LowerLim = FRAC16(-MC_PI_SPEED_OUTPUT_LIMIT/I_MAX);

	gsMC_Drive.sFocFw.f16FwError = FRAC16(0.0);
	gsMC_Drive.sFocFw.f16FwErrorFilt = FRAC16(0.0);
	gsMC_Drive.sFocFw.f16UFwError = FRAC16(0.0);
	gsMC_Drive.sFocFw.f16IFwError = FRAC16(0.0);
	GDFLIB_FilterIIR1Init_F16(&gsMC_Drive.sFocFw.sFwErrorFilter);
	gsMC_Drive.sFocFw.i16FwPiSatFlag = 0;

	gsMC_Drive.sPositionObsDQ.bStartUp     = true;
	gsMC_Drive.sPositionObsDQ.bStartUpFail = false;
	gsMC_Drive.sPositionObsDQ.sStartUp.f16MergeCoeff = 0;

	gsMC_Drive.sPositionObsDQ.sStartUp.f16IStartUpRamp = FRAC16(MC_ALIGN_CURRENT/I_MAX);
	gsMC_Drive.sPositionObsDQ.sStartUp.f32IStartUpRamp = FRAC32(MC_ALIGN_CURRENT/I_MAX);
	gsMC_Drive.sPositionObsDQ.sStartUp.sIRamp32.f32State = FRAC32(MC_ALIGN_CURRENT/I_MAX);
	gsMC_Drive.sPositionObsDQ.sStartUp.f16SpeedPredictedRamp = 0;
	gsMC_Drive.sPositionObsDQ.sStartUp.f32SpeedPredictedRamp = 0;
	gsMC_Drive.sPositionObsDQ.sStartUp.sSpeedIntegrator.f32IAccK_1 = gsMC_Drive.sFocPMSM.sAlignment.f32Position; // Pass the position in alignment to the integrator
	
	gsMC_Drive.sObserverDQ.sTo.f32I_1 = 0; // MLIB_Add_F16ed @2018.4.26, estimated position starts from 0

	if(gsMC_Drive.sSpeed.f16SpeedCmd > 0)
	{
		gsMC_Drive.sPositionObsDQ.sStartUp.f32SpeedStartUpDesired =  FRAC32(MC_START_UP_SPEED_MAX / N_MAX);
		gsMC_Drive.sPositionObsDQ.sStartUp.f32IStartUpDesired =   FRAC32(MC_START_UP_CURRENT_MAX / I_MAX);
	}
	else if(gsMC_Drive.sSpeed.f16SpeedCmd < 0)
	{
		gsMC_Drive.sPositionObsDQ.sStartUp.f32SpeedStartUpDesired =  FRAC32(-MC_START_UP_SPEED_MAX / N_MAX);
		gsMC_Drive.sPositionObsDQ.sStartUp.f32IStartUpDesired =   FRAC32(-MC_START_UP_CURRENT_MAX / I_MAX);
	}
	
	/* PWM update */
	MC_PWM_UPDATE(&gsMC_Drive.sFocPMSM.sDutyABC);

   /* Sub-state RUN STARTUP */
	meMC_StateRun = STARTUP;
}

/***************************************************************************//*!
*
* @brief   RUN ALIGN to READY transition
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void MC_TransRunAlignReady(void)
{
	/* Disable PWM output */
	MC_DISABLE_PWM_OUTPUT();
	
	gsMC_Drive.sFocPMSM.sIDQReq.f16D = 0;
	gsMC_Drive.sFocPMSM.sIDQReq.f16Q = 0;
	gsMC_Drive.sFocPMSM.sUDQReq.f16D = 0;
	gsMC_Drive.sFocPMSM.sUDQReq.f16Q = 0;	

	gsMC_Drive.sFocPMSM.sIAlBe.f16Alpha = 0;
	gsMC_Drive.sFocPMSM.sIAlBe.f16Beta = 0;
	gsMC_Drive.sFocPMSM.sUAlBeReq.f16Alpha = 0;
	gsMC_Drive.sFocPMSM.sUAlBeReq.f16Beta = 0;	
	gsMC_Drive.sFocPMSM.sUAlBeComp.f16Alpha = 0;
	gsMC_Drive.sFocPMSM.sUAlBeComp.f16Beta = 0;	

	/* 50% duty cycle */
	gsMC_Drive.sFocPMSM.sDutyABC.f16A = FRAC16(BOOTSTRAP_CHARGE_DUTY);
	gsMC_Drive.sFocPMSM.sDutyABC.f16B = FRAC16(BOOTSTRAP_CHARGE_DUTY);
	gsMC_Drive.sFocPMSM.sDutyABC.f16C = FRAC16(BOOTSTRAP_CHARGE_DUTY);

	/* PWM update */
	MC_PWM_UPDATE(&gsMC_Drive.sFocPMSM.sDutyABC);

	
	/* Sub-state RUN READY */
	meMC_StateRun = READY;
}

/***************************************************************************//*!
*
* @brief   RUN STARTUP to SPIN transition
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void MC_TransRunStartupSpin(void)
{	
	gsMC_Drive.uw16CounterSlowLoop = 1;

	gsMC_Drive.sFocPMSM.sIDQReq.f16D = 0;
	
	/* ready to speed closed loop */
	gsMC_Drive.sSpeed.s32SpeedRampParams.f32State = gsMC_Drive.sPositionObsDQ.sStartUp.f32SpeedPredictedRamp;
	gsMC_Drive.sSpeed.sSpeedPiParams.f32IAccK_1 = MLIB_Conv_F32s(gsMC_Drive.sFocPMSM.sIDQ.f16Q);
	
	/* Speed closed loop */
	gsMC_Drive.sSpeed.bOpenLoop = false;

	/* Sub-state RUN SPIN */
	meMC_StateRun = SPIN;
}

/***************************************************************************//*!
*
* @brief   RUN STARTUP to FREEWHEEL transition
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void MC_TransRunStartupFreewheel(void)
{
	/* Disable PWM output */
	MC_DISABLE_PWM_OUTPUT();

	gsMC_Drive.sFocPMSM.sIDQReq.f16D = 0;
	gsMC_Drive.sFocPMSM.sIDQReq.f16Q = 0;
	gsMC_Drive.sFocPMSM.sUDQReq.f16D = 0;
	gsMC_Drive.sFocPMSM.sUDQReq.f16Q = 0;
	
	gsMC_Drive.sFocPMSM.sIAlBe.f16Alpha = 0;
	gsMC_Drive.sFocPMSM.sIAlBe.f16Beta = 0;
	gsMC_Drive.sFocPMSM.sUAlBeReq.f16Alpha = 0;
	gsMC_Drive.sFocPMSM.sUAlBeReq.f16Beta = 0;	
	gsMC_Drive.sFocPMSM.sUAlBeComp.f16Alpha = 0;
	gsMC_Drive.sFocPMSM.sUAlBeComp.f16Beta = 0;	
	
	/* Generates a time gap before the alignment to assure the rotor is not rotating */
	gsMC_Drive.uw16CounterState = gsMC_Drive.sPositionObsDQ.sStartUp.uw16TimeStartUpFreeWheel;
	
	/* Default Pressure relaxation time before next start up when there's no start up fail */
	uw32CounterPressureRelax = (uint32_t)(MC_PRESSURE_RELAX_DURATION * 60.0 * MC_SLOW_CONTROL_LOOP_FREQ);
	
	/* Sub-state RUN FREEWHEEL */
	meMC_StateRun = FREEWHEEL;
}

/***************************************************************************//*!
*
* @brief   RUN SPIN to FREEWHEEL transition
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void MC_TransRunSpinFreewheel(void)
{
	/* Disable PWM output */
	MC_DISABLE_PWM_OUTPUT();

	/* Generates a time gap before the alignment to assure the rotor is not rotating */
	gsMC_Drive.uw16CounterState = (uint16_t)MLIB_MulSat_F16(MLIB_Abs_F16(gsMC_Drive.sSpeed.f16Speed), (frac16_t)gsMC_Drive.uw16TimeFullSpeedFreeWheel);		
	gsMC_Drive.uw16CounterState += 1;

	gsMC_Drive.sFocPMSM.sIDQReq.f16D = 0;
	gsMC_Drive.sFocPMSM.sIDQReq.f16Q = 0;
	gsMC_Drive.sFocPMSM.sUDQReq.f16D = 0;
	gsMC_Drive.sFocPMSM.sUDQReq.f16Q = 0;	
	
	gsMC_Drive.sFocPMSM.sIAlBe.f16Alpha = 0;
	gsMC_Drive.sFocPMSM.sIAlBe.f16Beta = 0;
	gsMC_Drive.sFocPMSM.sUAlBeReq.f16Alpha = 0;
	gsMC_Drive.sFocPMSM.sUAlBeReq.f16Beta = 0;	
	gsMC_Drive.sFocPMSM.sUAlBeComp.f16Alpha = 0;
	gsMC_Drive.sFocPMSM.sUAlBeComp.f16Beta = 0;

	/* MR20150324_D */
	gsMC_Drive.sFocFw.f16ILimit = FRAC16(MC_I_MAX / I_MAX);
   	gsMC_Drive.sSpeed.sSpeedPiParams.f16UpperLim = FRAC16(MC_I_MAX/I_MAX);
   	gsMC_Drive.sSpeed.sSpeedPiParams.f16LowerLim = FRAC16(-MC_I_MAX/I_MAX);


	/* Default Pressure relaxation time before next start up when there's no start up fail */
	uw32CounterPressureRelax = (uint32_t)(MC_PRESSURE_RELAX_DURATION * 60.0 * MC_SLOW_CONTROL_LOOP_FREQ);
	
	/* Sub-state RUN FREEWHEEL */
	meMC_StateRun = FREEWHEEL;	
}

/***************************************************************************//*!
*
* @brief   RUN FREEWHEEL to ALIGN transition
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void MC_TransRunFreewheelAlign(void)
{
	/* ENable PWM output */
	MC_ENABLE_PWM_OUTPUT();

	/* set up the Id for alignment   */
	gsMC_Drive.sFocPMSM.sIDQReq.f16D = gsMC_Drive.sFocPMSM.sAlignment.f16IMax;
	gsMC_Drive.sFocPMSM.sIDQReq.f16Q = 0;
	
	/* Alignment voltage init point */
	gsMC_Drive.sFocPMSM.sAlignment.f32U = 0;
	
	/* Alignment voltage init position */
	gsMC_Drive.sFocPMSM.sAlignment.f32Position = 0;
	
	/* Alignment duration set-up */
	gsMC_Drive.uw16CounterState = gsMC_Drive.sFocPMSM.sAlignment.uw16TimeAlignment;
	gsMC_Drive.sFocPMSM.sDutyABC.f16A = FRAC16(BOOTSTRAP_CHARGE_DUTY);
	gsMC_Drive.sFocPMSM.sDutyABC.f16B = FRAC16(BOOTSTRAP_CHARGE_DUTY);
	gsMC_Drive.sFocPMSM.sDutyABC.f16C = FRAC16(BOOTSTRAP_CHARGE_DUTY);
	
	gsMC_Drive.sSpeed.f32SpeedRamp = 0;/*MR20150317_A*/
	gsMC_Drive.sSpeed.f16SpeedRamp = 0;
	
	/* PMSM FOC params for alignment only because of Fault*/

  	gsMC_Drive.sFocPMSM.sIdPiParams.a32PGain = D_KP_GAIN;
   	gsMC_Drive.sFocPMSM.sIdPiParams.a32IGain = D_KI_GAIN;
   	gsMC_Drive.sFocPMSM.sIdPiParams.f32IAccK_1 = 0;
   	gsMC_Drive.sFocPMSM.sIdPiParams.f16UpperLim = FRAC16(VOLTAGE_ALIGN_MAX / U_MAX);  
   	gsMC_Drive.sFocPMSM.sIdPiParams.f16LowerLim = FRAC16(-VOLTAGE_ALIGN_MAX / U_MAX);
   	gsMC_Drive.sFocPMSM.sIdPiParams.bLimFlag = 0;

	gsMC_Drive.sFocPMSM.sIqPiParams.a32PGain = Q_KP_GAIN;
   	gsMC_Drive.sFocPMSM.sIqPiParams.a32IGain = Q_KI_GAIN;
   	gsMC_Drive.sFocPMSM.sIqPiParams.f32IAccK_1 = 0;
   	gsMC_Drive.sFocPMSM.sIqPiParams.f16UpperLim = FRAC16(VOLTAGE_ALIGN_MAX / U_MAX);  
   	gsMC_Drive.sFocPMSM.sIqPiParams.f16LowerLim = FRAC16(-VOLTAGE_ALIGN_MAX / U_MAX);    
   	gsMC_Drive.sFocPMSM.sIqPiParams.bLimFlag = 0;
   	
   	gsMC_Drive.sPositionObsDQ.sStartUp.f32SpeedPredictedRamp = 0;

	gsMC_Drive.sPositionObsDQ.sStartUp.f16PositionPredictedOld = 0;
	
	uw32DelayUnderVoltProtectCntr = 0; /* MR20140910_D; MR20150324_B */

   	/* Sub-state RUN ALIGN */
	meMC_StateRun = ALIGN;
	
}

/***************************************************************************//*!
*
* @brief   RUN FREEWHEEL to READY transition
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void MC_TransRunFreewheelReady(void)
{
	/* Disable PWM output */
	MC_DISABLE_PWM_OUTPUT();

	/* Sub-state RUN READY */
	meMC_StateRun = READY;
}



/***************************************************************************//*!
*
* @brief   Fault Detection function
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void MC_FaultDetection(void)
{
	gsMC_Drive.sFaultIdPending.R = 0;
	//-----------------------------
    // Actual Faults
    //-----------------------------
	
	//======================================  Fault:   Software over current  ======================================
	gsMC_Drive.sFaultIdPending.B.OverCurrentSoft = (f16ILengthFilt >= gsMC_Drive.sFaultThresholds.f16SoftwareOC);
	
    //======================================  Fault:   DC-bus over-voltage ======================================
    gsMC_Drive.sFaultIdPending.B.UDcBusOver = (gsMC_Drive.sFocPMSM.f16UDcBusFilt >= gsMC_Drive.sFaultThresholds.f16UDcBusOver);

    //======================================  Fault:   DC-bus over-current ======================================
    gsMC_Drive.sFaultIdPending.B.IDcBusOver = MC_OVERCURRENT_FAULT();

    //====================================== Fault: under voltage ====================================== 
    if (gsMC_Drive.sFocPMSM.f16UDcBusFilt <= gsMC_Drive.sFaultThresholds.f16UDcBusUnder)
    {
   	
    	if (uw32DelayUnderVoltProtectCntr < (uint32_t)(UNDER_VOLT_DELAY_PROTECT_TIME*MC_FAST_CONTROL_LOOP_FREQ)) 
    	{
    		uw32DelayUnderVoltProtectCntr++;  
    	}
    	else
    	{
    		gsMC_Drive.sFaultIdPending.B.UDcBusUnder = 1;
    	}
    }
    else
    {
    	uw32DelayUnderVoltProtectCntr = 0;
    	gsMC_Drive.sFaultIdPending.B.UDcBusUnder = 0;
    }

    gsMC_Drive.sFaultId.R |= gsMC_Drive.sFaultIdPending.R;    

}

/******************************************************************************
*
* 		   Sampled motor phases are updated per SVM sector
*
******************************************************************************/

/* Fetch the next channels to be sampled based on updated SVM number */
static void ADCChannelMapping(MCSTRUC_FOC_PMSM_T *ptr)
{
 	ptr->sAllChnlOffsets.sIChnl.uw16Ph0 = msChannels[ptr->uw16SectorSVM].uw16Ph0;
 	ptr->sAllChnlOffsets.sIChnl.uw16Ph1 = msChannels[ptr->uw16SectorSVM].uw16Ph1;
// 	ptr->sAllChnlOffsets.sIChnl.uw16Ph0 = msChannels[2].uw16Ph0;
// 	ptr->sAllChnlOffsets.sIChnl.uw16Ph1 = msChannels[2].uw16Ph1;
}


/* Configure the correct channel offsets based on the SVM number used in last PWM period, so that the sampled currents in this PWM period can be calculated */
static void ADCOffsetConfig(MCSTRUC_FOC_PMSM_T *ptr)
{
	switch (ptr->uw16SectorSVM) 
	{
	 case 2:
	 case 3:
		 	// phase A,C 
		 ptr->sAllChnlOffsets.sIOffsets.f16A = ptr->sAllChnlOffsets.f16PhA_ANA;
		 ptr->sAllChnlOffsets.sIOffsets.f16B = ptr->sAllChnlOffsets.f16PhC_ANB;
	        break;

	 case 4:
	 case 5:
		 	// phase A,B 
		 ptr->sAllChnlOffsets.sIOffsets.f16A = ptr->sAllChnlOffsets.f16PhA_ANA;
		 ptr->sAllChnlOffsets.sIOffsets.f16B = ptr->sAllChnlOffsets.f16PhB_ANB;
	        break;

	 case 6: 
	 case 1:
	 default:
		 	// phase C,B 
		 ptr->sAllChnlOffsets.sIOffsets.f16A = ptr->sAllChnlOffsets.f16PhC_ANA;
		 ptr->sAllChnlOffsets.sIOffsets.f16B = ptr->sAllChnlOffsets.f16PhB_ANB;
	        break;
	}
//	 ptr->sAllChnlOffsets.sIOffsets.f16A = ptr->sAllChnlOffsets.f16PhA_ANA;
//	 ptr->sAllChnlOffsets.sIOffsets.f16B = ptr->sAllChnlOffsets.f16PhC_ANB;
}
/***************************************************************************//*!
*
* @brief   Measure phase current
*
* @param   void
*	
* @return  none
*
******************************************************************************
*
* 		   In every sector, two motor phase currents sensed, the third calculated
*
******************************************************************************/
static void GetCurrent(MCSTRUC_FOC_PMSM_T *ptr)
{ 	
	//*************** 3-PHASE CURRENT RECONSTRUCTION BEGIN **************
    switch (ptr->uw16SectorSVM)
    {
    case 2:
    case 3:
    		{	//direct sensing of A, C, calculation of B 
    			ptr->sIABC.f16A = (MLIB_Sub_F16(ptr->sAllChnlOffsets.sIRaw.f16A, ptr->sAllChnlOffsets.sIOffsets.f16A) << 1);
    			ptr->sIABC.f16C = (MLIB_Sub_F16(ptr->sAllChnlOffsets.sIRaw.f16B, ptr->sAllChnlOffsets.sIOffsets.f16B) << 1);
    			ptr->sIABC.f16B = MLIB_Neg_F16(MLIB_AddSat_F16(ptr->sIABC.f16A, ptr->sIABC.f16C));
				
			return;
	       	}
	       	break;
    case 4: 
    case 5: 
    		{	//direct sensing of A, B, calculation of C 
    			ptr->sIABC.f16A = (MLIB_Sub_F16(ptr->sAllChnlOffsets.sIRaw.f16A, ptr->sAllChnlOffsets.sIOffsets.f16A) << 1);
    			ptr->sIABC.f16B = (MLIB_Sub_F16(ptr->sAllChnlOffsets.sIRaw.f16B, ptr->sAllChnlOffsets.sIOffsets.f16B) << 1);
    			ptr->sIABC.f16C = MLIB_Neg_F16(MLIB_AddSat_F16(ptr->sIABC.f16A, ptr->sIABC.f16B));
                
			return;
	        }		        
            break;
    case 1:
    case 6:
    default:
	        {	//direct sensing of C, B, calculation of A  
    			ptr->sIABC.f16C = (MLIB_Sub_F16(ptr->sAllChnlOffsets.sIRaw.f16A, ptr->sAllChnlOffsets.sIOffsets.f16A) << 1);
    			ptr->sIABC.f16B = (MLIB_Sub_F16(ptr->sAllChnlOffsets.sIRaw.f16B, ptr->sAllChnlOffsets.sIOffsets.f16B) << 1);
    			ptr->sIABC.f16A = MLIB_Neg_F16(MLIB_AddSat_F16(ptr->sIABC.f16C, ptr->sIABC.f16B));
	    		
            return;
	        }
    break;
    }
	//*************** 3-PHASE CURRENT RECONSTRUCTION -END- **************
	
//	ptr->sIABC.f16A = (MLIB_Sub_F16(ptr->sAllChnlOffsets.sIRaw.f16A, ptr->sAllChnlOffsets.sIOffsets.f16A) << 1);
//	ptr->sIABC.f16C = (MLIB_Sub_F16(ptr->sAllChnlOffsets.sIRaw.f16B, ptr->sAllChnlOffsets.sIOffsets.f16B) << 1);
//	ptr->sIABC.f16B = MLIB_Neg_F16(MLIB_AddSat_F16(ptr->sIABC.f16A, ptr->sIABC.f16C));
	
}



/***************************************************************************//*!
*
* @brief   Fault function
*
* @param   void
*
* @return  none
*
******************************************************************************/
void MC_Fault(void)
{
	MC_FaultDetection();

	if (gsMC_Drive.sFaultIdPending.R > 0)
	{
		/* Fault state */
		gsMC_Ctrl.uiCtrl |= SM_CTRL_FAULT;
	}
}


/***************************************************************************//*!
*
* @brief   LED indication function for faults
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void LED_Indication(void)
{
	switch(faultRecordFlag.R)
	{
	case 0x01: // startup fail
		uw16LedCntBlkTimesVal = 2*BLINK_TIMES_STALL;
		uw16LedCntPatternTimesVal = PATTERN_TIMES_STALL;
		uw16LedCntBlkDurationVal = (uint16_t)(MC_SLOW_CONTROL_LOOP_FREQ*BLINK_DURATION_STALL);
		uw16LedCntPatternSpaceVal = (uint16_t)(MC_SLOW_CONTROL_LOOP_FREQ*PATTERN_SPACE_STALL);
		break;
	case 0x02: // over load
		uw16LedCntBlkTimesVal = 2*BLINK_TIMES_OL;
		uw16LedCntPatternTimesVal = PATTERN_TIMES_OL;
		uw16LedCntBlkDurationVal = (uint16_t)(MC_SLOW_CONTROL_LOOP_FREQ*BLINK_DURATION_OL);
		uw16LedCntPatternSpaceVal = (uint16_t)(MC_SLOW_CONTROL_LOOP_FREQ*PATTERN_SPACE_OL);		
		break;
	case 0x04: // hardware over current
		uw16LedCntBlkTimesVal = 2*BLINK_TIMES_HOC;
		uw16LedCntPatternTimesVal = PATTERN_TIMES_HOC;
		uw16LedCntBlkDurationVal = (uint16_t)(MC_SLOW_CONTROL_LOOP_FREQ*BLINK_DURATION_HOC);
		uw16LedCntPatternSpaceVal = (uint16_t)(MC_SLOW_CONTROL_LOOP_FREQ*PATTERN_SPACE_HOC);
		break;
	case 0x08: // under voltage
		uw16LedCntBlkTimesVal = 2*BLINK_TIMES_UV;
		uw16LedCntPatternTimesVal = PATTERN_TIMES_UV;
		uw16LedCntBlkDurationVal = (uint16_t)(MC_SLOW_CONTROL_LOOP_FREQ*BLINK_DURATION_UV);
		uw16LedCntPatternSpaceVal = (uint16_t)(MC_SLOW_CONTROL_LOOP_FREQ*PATTERN_SPACE_UV);
		break;
	case 0x10: // over voltage
		uw16LedCntBlkTimesVal = 2*BLINK_TIMES_OV;
		uw16LedCntPatternTimesVal = PATTERN_TIMES_OV;
		uw16LedCntBlkDurationVal = (uint16_t)(MC_SLOW_CONTROL_LOOP_FREQ*BLINK_DURATION_OV);
		uw16LedCntPatternSpaceVal = (uint16_t)(MC_SLOW_CONTROL_LOOP_FREQ*PATTERN_SPACE_OV);
		break;
	case 0x80: // open phase
		uw16LedCntBlkTimesVal = 2*BLINK_TIMES_OP;
		uw16LedCntPatternTimesVal = PATTERN_TIMES_OP;
		uw16LedCntBlkDurationVal = (uint16_t)(MC_SLOW_CONTROL_LOOP_FREQ*BLINK_DURATION_OP);
		uw16LedCntPatternSpaceVal = (uint16_t)(MC_SLOW_CONTROL_LOOP_FREQ*PATTERN_SPACE_OP);
		break;
	case 0x100: // software over current
		uw16LedCntBlkTimesVal = 2*BLINK_TIMES_SOC;
		uw16LedCntPatternTimesVal = PATTERN_TIMES_SOC;
		uw16LedCntBlkDurationVal = (uint16_t)(MC_SLOW_CONTROL_LOOP_FREQ*BLINK_DURATION_SOC);
		uw16LedCntPatternSpaceVal = (uint16_t)(MC_SLOW_CONTROL_LOOP_FREQ*PATTERN_SPACE_SOC);
		break;
	default:	// compound error or other error
		uw16LedCntBlkTimesVal = 2*BLINK_TIMES_OTHER;
		uw16LedCntPatternTimesVal = PATTERN_TIMES_OTHER;
		uw16LedCntBlkDurationVal = (uint16_t)(MC_SLOW_CONTROL_LOOP_FREQ*BLINK_DURATION_OTHER);
		uw16LedCntPatternSpaceVal = (uint16_t)(MC_SLOW_CONTROL_LOOP_FREQ*PATTERN_SPACE_OTHER);
		break;			
	}
	
	if(uw16LedCntPatternTimes <= uw16LedCntPatternTimesVal)
	{
		if(uw16LedCntBlkTimes <= uw16LedCntBlkTimesVal)
		{
			GPIO_PinWrite(BOARD_USER_LED_GPIO, BOARD_USER_LED_PIN_MASK, (gpio_output_level_t)(uw16LedCntBlkTimes & 0x0001));
			if(++uw16LedCntBlkDuration > uw16LedCntBlkDurationVal)
			{
				uw16LedCntBlkDuration = 0;
				uw16LedCntBlkTimes++;
			}				
		}
		else
		{
			if(++uw16LedCntPatternSpace > uw16LedCntPatternSpaceVal)
			{
				uw16LedCntPatternSpace = 0;
				uw16LedCntPatternTimes++;
				uw16LedCntBlkTimes = 1;
			}
		}
	}
	else
	{
		faultRecordFlag.R = 0;
		uw16LedCntPatternSpace = 0;
		uw16LedCntPatternTimes = 1;
		uw16LedCntBlkDuration = 0;
		uw16LedCntBlkTimes = 1;
	}
}

/***************************************************************************//*!
*
* @brief   LED indication function invoked by Fault state
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void LED_Indication_In_Fault(void)
{
	switch(faultRecordFlag.R)
	{
	case 0x01: // startup fail
		uw16LedCntBlkTimesVal = 2*BLINK_TIMES_STALL;
		uw16LedCntPatternTimesVal = PATTERN_TIMES_STALL;
		uw16LedCntBlkDurationVal = (uint16_t)(MC_SLOW_CONTROL_LOOP_FREQ*BLINK_DURATION_STALL);
		uw16LedCntPatternSpaceVal = (uint16_t)(MC_SLOW_CONTROL_LOOP_FREQ*PATTERN_SPACE_STALL);
		break;
	case 0x02: // over load
		uw16LedCntBlkTimesVal = 2*BLINK_TIMES_OL;
		uw16LedCntPatternTimesVal = PATTERN_TIMES_OL;
		uw16LedCntBlkDurationVal = (uint16_t)(MC_SLOW_CONTROL_LOOP_FREQ*BLINK_DURATION_OL);
		uw16LedCntPatternSpaceVal = (uint16_t)(MC_SLOW_CONTROL_LOOP_FREQ*PATTERN_SPACE_OL);		
		break;
	case 0x04: // hardware over current
		uw16LedCntBlkTimesVal = 2*BLINK_TIMES_HOC;
		uw16LedCntPatternTimesVal = PATTERN_TIMES_HOC;
		uw16LedCntBlkDurationVal = (uint16_t)(MC_SLOW_CONTROL_LOOP_FREQ*BLINK_DURATION_HOC);
		uw16LedCntPatternSpaceVal = (uint16_t)(MC_SLOW_CONTROL_LOOP_FREQ*PATTERN_SPACE_HOC);
		break;
	case 0x08: // under voltage
		uw16LedCntBlkTimesVal = 2*BLINK_TIMES_UV;
		uw16LedCntPatternTimesVal = PATTERN_TIMES_UV;
		uw16LedCntBlkDurationVal = (uint16_t)(MC_SLOW_CONTROL_LOOP_FREQ*BLINK_DURATION_UV);
		uw16LedCntPatternSpaceVal = (uint16_t)(MC_SLOW_CONTROL_LOOP_FREQ*PATTERN_SPACE_UV);
		break;
	case 0x10: // over voltage
		uw16LedCntBlkTimesVal = 2*BLINK_TIMES_OV;
		uw16LedCntPatternTimesVal = PATTERN_TIMES_OV;
		uw16LedCntBlkDurationVal = (uint16_t)(MC_SLOW_CONTROL_LOOP_FREQ*BLINK_DURATION_OV);
		uw16LedCntPatternSpaceVal = (uint16_t)(MC_SLOW_CONTROL_LOOP_FREQ*PATTERN_SPACE_OV);
		break;
	case 0x80: // open phase
		uw16LedCntBlkTimesVal = 2*BLINK_TIMES_OP;
		uw16LedCntPatternTimesVal = PATTERN_TIMES_OP;
		uw16LedCntBlkDurationVal = (uint16_t)(MC_SLOW_CONTROL_LOOP_FREQ*BLINK_DURATION_OP);
		uw16LedCntPatternSpaceVal = (uint16_t)(MC_SLOW_CONTROL_LOOP_FREQ*PATTERN_SPACE_OP);
		break;
	case 0x100: // software over current
		uw16LedCntBlkTimesVal = 2*BLINK_TIMES_SOC;
		uw16LedCntPatternTimesVal = PATTERN_TIMES_SOC;
		uw16LedCntBlkDurationVal = (uint16_t)(MC_SLOW_CONTROL_LOOP_FREQ*BLINK_DURATION_SOC);
		uw16LedCntPatternSpaceVal = (uint16_t)(MC_SLOW_CONTROL_LOOP_FREQ*PATTERN_SPACE_SOC);
		break;
	default:	// compound error or other error
		uw16LedCntBlkTimesVal = 2*BLINK_TIMES_OTHER;
		uw16LedCntPatternTimesVal = PATTERN_TIMES_OTHER;
		uw16LedCntBlkDurationVal = (uint16_t)(MC_SLOW_CONTROL_LOOP_FREQ*BLINK_DURATION_OTHER);
		uw16LedCntPatternSpaceVal = (uint16_t)(MC_SLOW_CONTROL_LOOP_FREQ*PATTERN_SPACE_OTHER);
		break;			
	}
	
	if(uw16LedCntBlkTimes <= uw16LedCntBlkTimesVal)
	{
		GPIO_PinWrite(BOARD_USER_LED_GPIO, BOARD_USER_LED_PIN_MASK, (gpio_output_level_t)(uw16LedCntBlkTimes & 0x0001));
		if(++uw16LedCntBlkDuration > uw16LedCntBlkDurationVal)
		{
			uw16LedCntBlkDuration = 0;
			uw16LedCntBlkTimes++;
		}				
	}
	else
	{
		if(++uw16LedCntPatternSpace > uw16LedCntPatternSpaceVal)
		{
			uw16LedCntPatternSpace = 0;
			uw16LedCntBlkTimes = 1;
		}
	}
}

#pragma section CODES_IN_RAM begin
/******************************************************************************
* Inline functions
******************************************************************************/




