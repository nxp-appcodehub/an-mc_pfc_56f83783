
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
* Motor control structure.
*
******************************************************************************/

/******************************************************************************
* Includes
******************************************************************************/
#include "motor_structure.h"
#include "cpu.h"
/******************************************************************************
* Constants
******************************************************************************/

/******************************************************************************
* Macros 
******************************************************************************/


/******************************************************************************
* Types
******************************************************************************/

#define OPENLOOP 0 // for openloop test
/******************************************************************************
* Global variables
******************************************************************************/


/******************************************************************************
* Global functions
******************************************************************************/
/***************************************************************************//*!
*
* @brief   PMSM field oriented control
*
* @param   MCSTRUC_FOC_PMSM_T *psFocPMSM
*			- structure of PMSM FOC parameters
*			IN:->sIABC - input ABC phases currents
*			IN:->sAnglePosEl - angle where the currents were measured
*			IN:->sAnglePosElUpdate - angle where the next PWM reload
*			IN:->f16UDcBusFilt - DC bus voltage
*			IN:->bUseMaxBus - true to calculate max. possible output DQ voltage
*					limits in dependence on the dc bus voltage. False to keep the
*					output DQ voltage limits intacts.
*			IN:->f16DutyCycleLimit - determins the max. value of duty cycle
*			IN/OUT->sIdPiParams - D current controller structure
*			IN/OUT->sIqPiParams - Q current controller structure
*			IN/OUT->i16IdPiSatFlag - D current controller saturation flag
*			IN/OUT->i16IqPiSatFlag - Q current controller saturation flag
*			OUT->sDutyABC - ABC duty cycles
*			OUT->uw16SectorSVM - Next step SVM sector
*
* @return  N/A
*
******************************************************************************/
#pragma section CODES_IN_RAM begin
void MCSTRUC_FocPMSMCurrentCtrl(MCSTRUC_FOC_PMSM_T *psFocPMSM)
{
	/* 3-phase to 2-phase transformation to stationary ref. frame */
	GMCLIB_Clark_F16(&psFocPMSM->sIABC,&psFocPMSM->sIAlBe);
	
	/* 2-phase to 2-phase transformation to rotary ref. frame */
	GMCLIB_Park_F16(&psFocPMSM->sIAlBe, &psFocPMSM->sAnglePosEl,&psFocPMSM->sIDQ);

	/* D current error calculation */
    psFocPMSM->sIDQError.f16D = MLIB_SubSat_F16(psFocPMSM->sIDQReq.f16D, psFocPMSM->sIDQ.f16D);/*MR20150306_A*/
	/* Q current error calculation */
	psFocPMSM->sIDQError.f16Q = MLIB_SubSat_F16(psFocPMSM->sIDQReq.f16Q, psFocPMSM->sIDQ.f16Q);/*MR20150306_A*/
	
    /* D - controller */
	psFocPMSM->sIdPiParams.f16UpperLim = MLIB_MulSat_F16(psFocPMSM->f16DutyCycleLimit, psFocPMSM->f16UDcBusFilt);
	psFocPMSM->sIdPiParams.f16LowerLim = MLIB_Neg_F16(psFocPMSM->sIdPiParams.f16UpperLim);
    /* D current PI controller */
    psFocPMSM->sUDQController.f16D = GFLIB_CtrlPIpAW_F16(psFocPMSM->sIDQError.f16D,&psFocPMSM->i16IdPiSatFlag, &psFocPMSM->sIdPiParams);
	/* D current controller saturation flag */
	psFocPMSM->i16IdPiSatFlag = psFocPMSM->sIdPiParams.bLimFlag;

    /* Q - controller */
	psFocPMSM->sIqPiParams.f16UpperLim = GFLIB_Sqrt_F16l(MLIB_Sub_F32( \
		MLIB_Mul_F32ss(psFocPMSM->sIdPiParams.f16UpperLim, psFocPMSM->sIdPiParams.f16UpperLim), \
	    MLIB_Mul_F32ss(psFocPMSM->sUDQController.f16D, psFocPMSM->sUDQController.f16D)));
    psFocPMSM->sIqPiParams.f16LowerLim = MLIB_Neg_F16(psFocPMSM->sIqPiParams.f16UpperLim);
    /* Q current PI controller */
    psFocPMSM->sUDQController.f16Q = GFLIB_CtrlPIpAW_F16(psFocPMSM->sIDQError.f16Q, &psFocPMSM->i16IqPiSatFlag,&psFocPMSM->sIqPiParams);
	/* Q current controller saturation flag */
	psFocPMSM->i16IqPiSatFlag = psFocPMSM->sIqPiParams.bLimFlag;

	/* D, Q voltage application */
	psFocPMSM->sUDQReq.f16D = psFocPMSM->sUDQController.f16D;
	psFocPMSM->sUDQReq.f16Q = psFocPMSM->sUDQController.f16Q;	
    
 	/* 2-phase to 2-phase transformation to stationary ref. frame */
 	GMCLIB_ParkInv_F16(&psFocPMSM->sUDQReq, &psFocPMSM->sAnglePosElUpdate,&psFocPMSM->sUAlBeReq);

    /* Begin - voltage control */
    GMCLIB_ElimDcBusRipFOC_F16(psFocPMSM->f16UDcBusFilt, &psFocPMSM->sUAlBeReq, &psFocPMSM->sUAlBeComp);
    
    psFocPMSM->uw16SectorSVM = GMCLIB_SvmStd_F16(&psFocPMSM->sUAlBeComp, &psFocPMSM->sDutyABC);
  
    /* End - voltage control */
}




/***************************************************************************//*!
*
* @brief   Position estimation using BEMF observer in DQ
*
* @param   MCSTRUC_FOC_PMSM_T *psFocPMSM
*			- structure of PMSM FOC parameters
*			IN:->sIABC - input ABC phases currents
*			IN:->sUAlBeReq - required voltage alpha/beta
*
*		   MCSTRUC_BEMF_OBS_DQ_T *psObserver
*			- structure of BEMF observer to dermine speed and position
*			IN/OUT->sBemfObsrv - BEMF observer params
*			IN/OUT->sTo - Tracking observer params
*
*		   MCSTRUC_POS_SPEED_EST_T *psPosition
*			- structure of position and speed
*			IN:->bObserver - true to turn on the observer
*			IN:->bStartUp - true to motor open-loop start-up mode
*			IN:->bOpenLoop - true not to copy the estimated position to the finale structure
*			IN:->bUseMaxBus - true to calculate max. possible output DQ voltage
*					limits in dependence on the dc bus voltage. False to keep the
*					output DQ voltage limits intacts.
*			IN->sStartUp.f16SpeedPredicted - Predicted speed at open-loop start-up
*			IN/OUT->f16SpeedEstimatedFilt - Estimated filtered speed
*			IN/OUT->sSpeedEstFilter - Speed 1st order IIR filter
*			IN/OUT->f16PositionEl - Estimated position
*			IN/OUT->sAnglePosElUpdate - next step position estimation
*			OUT->sAnglePosEl - actual step position estimation (from previous step calculation)
*
* @return  N/A
*
******************************************************************************/
void MCSTRUC_PMSMPositionObsDQ(MCSTRUC_FOC_PMSM_T *psFocPMSM, MCSTRUC_BEMF_OBS_DQ_T *psObserver, MCSTRUC_POS_SPEED_EST_T *psPosition)
{
	GMCLIB_2COOR_ALBE_T_F16 sIAlBe;
	GMCLIB_2COOR_DQ_T_F16	sIDQ;
	GMCLIB_2COOR_DQ_T_F16 sUDQ;

	/* 3-phase to 2-phase transformation to stationary ref. frame */
	GMCLIB_Clark_F16(&psFocPMSM->sIABC, &sIAlBe);
	/* 2-phase to 2-phase transformation to rotary ref. frame */
	GMCLIB_Park_F16(&sIAlBe, &psPosition->sAnglePosElUpdate,&sIDQ);
	
//	sIDQ.f16D = sIDQ.f16D>>0;//2;
//	sIDQ.f16Q = sIDQ.f16Q>>0;//2;
	
	/* 2-phase to 2-phase transformation to rotary ref. frame */
	GMCLIB_Park_F16(&psFocPMSM->sUAlBeReq, &psPosition->sAnglePosElUpdate,&sUDQ);
	AMCLIB_PMSMBemfObsrvDQ_F16(&sIDQ, &sUDQ, psPosition->f16SpeedEstimatedFilt, &psObserver->sBemfObsrv);
	
	/* Tracking observer calculation */
	psPosition->f16PositionEl = AMCLIB_TrackObsrv_F16(psObserver->sBemfObsrv.f16Error, &psObserver->sTo);
	
	/* Speed estimation by the tracking observer */
	psPosition->f16SpeedEstimated = MLIB_Conv_F16l(psObserver->sTo.f32Speed);
	
	/* Speed filter */
	psPosition->f16SpeedEstimatedFilt = GDFLIB_FilterIIR1_F16(psPosition->f16SpeedEstimated, &psPosition->sSpeedEstFilter);

	psPosition->f16PositionElUpdate = MLIB_Add_F16(psPosition->f16PositionEl, MLIB_Mul_F16(FRAC16(2.0*N_MAX*MC_POLE_PAIRS/(60.0*MC_FAST_CONTROL_LOOP_FREQ)),psPosition->f16SpeedEstimatedFilt)); 

	/* Estimated angle calculation */
   	psPosition->sAnglePosEl.f16Sin = GFLIB_Sin_F16(psPosition->f16PositionEl);
	psPosition->sAnglePosEl.f16Cos = GFLIB_Cos_F16(psPosition->f16PositionEl);
   	psPosition->sAnglePosElUpdate.f16Sin = GFLIB_Sin_F16(psPosition->f16PositionElUpdate);
	psPosition->sAnglePosElUpdate.f16Cos = GFLIB_Cos_F16(psPosition->f16PositionElUpdate);
}



/***************************************************************************//*!
*
* @brief   PMSM field weakening
*
* @param   MCSTRUC_FOC_PMSM_T *psFocPMSM
*			- structure of PMSM FOC parameters
*			IN:->sIqPiParams.f16UpperLim - Iq controller output limit
*			IN:->sUDQReq.f16Q - Required Q voltage
*			IN:->sIDQReq.f16Q - Required Q current
*			IN:->sIDQ.f16Q - Measured Q current
*			IN:->i16IdPiSatFlag - D current controller saturation flag
*
*		   MCSTRUC_FOC_FW_T *psFocFw
*			- field weakening structure
*			IN:->f16SpeedFwOn - speed threshold where the FW is turned on
*			IN/OUT:->sFwErrorFilter - Fw error 1st order IIR filter structure
*			IN/OUT:->sFwPiParams - Fw PI controller params
*			IN/OUT:->i16FwPiSatFlag - Fw PI controller saturation flag
*			OUT->sSpeedPiParams.f16UpperLim - speed controller output current limit
*			OUT->sSpeedPiParams.f16LowerLim - speed controller output current limit
*
*		   MCSTRUC_SPEED_T *psSpeed
*			- speed structure
*			IN:->f16Speed - actual speed
*			OUT->sSpeedPiParams.f16UpperLim - speed controller output current limit
*			OUT->sSpeedPiParams.f16LowerLim - speed controller output current limit
*
* @return  N/A
*
******************************************************************************/

void MCSTRUC_FocFieldWeakeningCtrl(MCSTRUC_FOC_PMSM_T *psFocPMSM, MCSTRUC_FOC_FW_T *psFocFw, MCSTRUC_SPEED_T *psSpeed)
{
	if (MLIB_Abs_F16(psSpeed->f16Speed) > psFocFw->f16SpeedFwOn)
	{
		/* Begin - field weakening controller */    
	    psFocFw->f16UFwError = MLIB_Sub_F16(psFocPMSM->sIqPiParams.f16UpperLim, MLIB_Abs_F16(psFocPMSM->sUDQReq.f16Q));
	    psFocFw->f16IFwError = MLIB_Abs_F16(MLIB_Sub_F16(psFocPMSM->sIDQReq.f16Q, psFocPMSM->sIDQ.f16Q));
	    psFocFw->f16FwError = MLIB_Sub_F16(psFocFw->f16UFwError, psFocFw->f16IFwError);
	    psFocFw->f16FwErrorFilt = GDFLIB_FilterIIR1_F16(psFocFw->f16FwError, &psFocFw->sFwErrorFilter);

		psFocFw->i16FwPiSatFlag = psFocFw->sFwPiParams.bLimFlag | psFocPMSM->i16IdPiSatFlag;
	    psFocPMSM->sIDQReq.f16D = GFLIB_CtrlPIpAW_F16(psFocFw->f16FwErrorFilt, &psFocFw->i16FwPiSatFlag, &psFocFw->sFwPiParams);
		/* End - field weakening controller */                        

		
		/* Speed PI controller current (torque) limitation */
		psSpeed->sSpeedPiParams.f16UpperLim = GFLIB_Sqrt_F16l(MLIB_Sub_F32( \
		MLIB_Mul_F32ss(psFocFw->f16ILimit, psFocFw->f16ILimit), \
		MLIB_Mul_F32ss(psFocPMSM->sIDQReq.f16D, psFocPMSM->sIDQReq.f16D)));
		
		psSpeed->sSpeedPiParams.f16LowerLim = MLIB_Neg_F16(psSpeed->sSpeedPiParams.f16UpperLim);

	}
	else
	{
		psSpeed->sSpeedPiParams.f16UpperLim = psFocFw->f16ILimit;
		psSpeed->sSpeedPiParams.f16LowerLim = MLIB_Neg_F16(psFocFw->f16ILimit);
		
		psFocFw->i16FwPiSatFlag = 0;
		psFocPMSM->sIDQReq.f16D  = 0;
		psFocFw->sFwPiParams.f32IAccK_1 = 0;
	}
}

/***************************************************************************//*!
*
* @brief   PMSM open-loop start-up
*
* @param   MCSTRUC_FOC_PMSM_T *psFocPMSM
*			- structure of PMSM FOC parameters


******************************************************************************/


void MCSTRUC_PMSMOpenLoopStartUp(MCSTRUC_FOC_PMSM_T *psFocPMSM, MCSTRUC_POS_SPEED_EST_T *psPosition, MCSTRUC_SPEED_T *psSpeed)
{	
	/* current ramp command calculation */
	psPosition->sStartUp.f32IStartUpRamp = GFLIB_Ramp_F32(psPosition->sStartUp.f32IStartUpDesired,&psPosition->sStartUp.sIRamp32);
	psPosition->sStartUp.f16IStartUpRamp = MLIB_Conv_F16l(psPosition->sStartUp.f32IStartUpRamp);	
	psFocPMSM->sIDQReq.f16Q = psPosition->sStartUp.f16IStartUpRamp;
	psFocPMSM->sIDQReq.f16D = 0;

	/* speed command is given after the current reaches to pre-defined pull-out current */ 
	if (MLIB_Abs_F16(psPosition->sStartUp.f16IStartUpRamp) >= psPosition->sStartUp.f16IStartUpPullOut)
	{
		psPosition->sStartUp.f32SpeedPredictedRamp = GFLIB_Ramp_F32(psPosition->sStartUp.f32SpeedStartUpDesired,&psPosition->sStartUp.sSpdRamp32);
		psPosition->sStartUp.f16SpeedPredictedRamp = MLIB_Conv_F16l(psPosition->sStartUp.f32SpeedPredictedRamp);
	}

	/* Get Position from speed ramp command */
	psPosition->sStartUp.f16PositionPredicted = GFLIB_Integrator_F16(psPosition->sStartUp.f16SpeedPredictedRamp, &psPosition->sStartUp.sSpeedIntegrator);
#if OPENLOOP == 0 
	if(MLIB_Abs_F32(psPosition->sStartUp.f32SpeedPredictedRamp) < MLIB_Abs_F32(psPosition->sStartUp.f32SpeedStartUpDesired))
	{
#endif
		psPosition->sStartUp.sAngleMerge.f16Sin = GFLIB_Sin_F16(psPosition->sStartUp.f16PositionPredicted);
		psPosition->sStartUp.sAngleMerge.f16Cos = GFLIB_Cos_F16(psPosition->sStartUp.f16PositionPredicted);
#if OPENLOOP == 0
	}
#endif
#if OPENLOOP == 0
	else if (MLIB_Abs_F32(psPosition->sStartUp.f32SpeedPredictedRamp) >= MLIB_Abs_F32(psPosition->sStartUp.f32SpeedStartUpDesired))
	{
		psPosition->sStartUp.f16MergeCoeff = MLIB_AddSat_F16(psPosition->sStartUp.f16MergeCoeff, psPosition->sStartUp.f16MergeStep);       
	        
	    /* merging equation */
		psPosition->sStartUp.f16PosMerge = MLIB_Add_F16(psPosition->sStartUp.f16PositionPredicted,\
				MLIB_Mul_F16(MLIB_Sub_F16(psPosition->f16PositionEl, psPosition->sStartUp.f16PositionPredicted), psPosition->sStartUp.f16MergeCoeff));
		psPosition->sStartUp.sAngleMerge.f16Sin = GFLIB_Sin_F16(psPosition->sStartUp.f16PosMerge);
		psPosition->sStartUp.sAngleMerge.f16Cos = GFLIB_Cos_F16(psPosition->sStartUp.f16PosMerge);
    }
#endif
	if(psPosition->sStartUp.f16MergeCoeff == FRAC16(1.0))
	{
		/* startup mode is finished, switch to spin */
		psPosition->bStartUp = FALSE;
	}
	
	psFocPMSM->sAnglePosElUpdate = psPosition->sStartUp.sAngleMerge;
	psFocPMSM->sAnglePosEl = psPosition->sStartUp.sAngleMerge;
}

/***************************************************************************//*!
*
* @brief   PMSM Alignment with rotation
*
* @param   MCSTRUC_FOC_PMSM_T *psFocPMSM
*			- structure of PMSM FOC parameters
*			IN/OUT:->sAlignment.f32Position - position of the field
*			IN:->sAlignment.f32Speed - speed of the field
*			IN:->sIDQ.f16D - measured D current
*			IN:->sAlignment.f16UMax - max D voltage at alignment
*			IN:->sAlignment.f16IMax - max D current at alignment
*			IN/OUT:->sAlignment.f32U - alignment D voltage which is ramped
*			IN:->sAlignment.f32UStep - voltage step to ramp the voltage
*			OUT:->psFocPMSM->sDutyABC - duty cycles ABC
*
*
* @return  N/A
*
******************************************************************************/
void MCSTRUC_AlignmentPMSM(MCSTRUC_FOC_PMSM_T *psFocPMSM)
{
	GMCLIB_2COOR_SINCOS_T_F16 sAngle;

   	sAngle.f16Sin = GFLIB_Sin_F16(MLIB_Conv_F16l(psFocPMSM->sAlignment.f32Position));
	sAngle.f16Cos = GFLIB_Cos_F16(MLIB_Conv_F16l(psFocPMSM->sAlignment.f32Position));

  	/* 3-phase to 2-phase transformation to stationary ref. frame */
	GMCLIB_Clark_F16(&psFocPMSM->sIABC, &psFocPMSM->sIAlBe);

	/* 2-phase to 2-phase transformation to rotary ref. frame */
	GMCLIB_Park_F16(&psFocPMSM->sIAlBe, &sAngle, &psFocPMSM->sIDQ);
	
	/* D current error calculation */
	psFocPMSM->sIDQError.f16D = MLIB_Sub_F16(psFocPMSM->sIDQReq.f16D, psFocPMSM->sIDQ.f16D);
	/* Q current error calculation */
	psFocPMSM->sIDQError.f16Q = MLIB_Sub_F16(psFocPMSM->sIDQReq.f16Q, psFocPMSM->sIDQ.f16Q);
	
	/* D current PI controller */
	psFocPMSM->sUDQController.f16D = GFLIB_CtrlPIpAW_F16(psFocPMSM->sIDQError.f16D, &psFocPMSM->i16IdPiSatFlag, &psFocPMSM->sIdPiParams);
	/* D current controller saturation flag */
	psFocPMSM->i16IdPiSatFlag = psFocPMSM->sIdPiParams.bLimFlag;
	
	/* Q current PI controller */
	psFocPMSM->sUDQController.f16Q = GFLIB_CtrlPIpAW_F16(psFocPMSM->sIDQError.f16Q, &psFocPMSM->i16IqPiSatFlag, &psFocPMSM->sIqPiParams);
	/* Q current controller saturation flag */
	psFocPMSM->i16IqPiSatFlag = psFocPMSM->sIqPiParams.bLimFlag;
	
	/* D, Q voltage application */
	psFocPMSM->sUDQReq.f16D = psFocPMSM->sUDQController.f16D;
	psFocPMSM->sUDQReq.f16Q = psFocPMSM->sUDQController.f16Q;	
	
   	/* 2-phase to 2-phase transformation to stationary ref. frame */
 	GMCLIB_ParkInv_F16(&psFocPMSM->sUDQReq, &sAngle, &psFocPMSM->sUAlBeReq);

	/* Begin - voltage control */ 	
 	GMCLIB_ElimDcBusRipFOC_F16(psFocPMSM->f16UDcBusFilt, &psFocPMSM->sUAlBeReq, &psFocPMSM->sUAlBeComp);
	psFocPMSM->uw16SectorSVM = GMCLIB_SvmStd_F16(&psFocPMSM->sUAlBeComp, &psFocPMSM->sDutyABC);
	/* End - voltage control */

	psFocPMSM->sAlignment.f32Position += psFocPMSM->sAlignment.f32Speed;
}


void MCSTRUC_ChlOffsetCalib(MCSTRUC_FOC_PMSM_T *psFocPMSM)
{
	switch(psFocPMSM->uw16SectorSVM)
	{
	 case 2:
	 case 3:
		 	// phase A,C is sampled based on current SVM number
		 psFocPMSM->sAllChnlOffsets.f16PhA_ANA = GDFLIB_FilterMA_F16(psFocPMSM->sAllChnlOffsets.sIRaw.f16A, &psFocPMSM->sAllChnlOffsets.sFilterMA_phA_ANA);
		 psFocPMSM->sAllChnlOffsets.f16PhC_ANB = GDFLIB_FilterMA_F16(psFocPMSM->sAllChnlOffsets.sIRaw.f16B, &psFocPMSM->sAllChnlOffsets.sFilterMA_phC_ANB);
	        break;

	 case 4:
	 case 5:
		 	// phase A,B is sampled based on current SVM number  
		 psFocPMSM->sAllChnlOffsets.f16PhA_ANA = GDFLIB_FilterMA_F16(psFocPMSM->sAllChnlOffsets.sIRaw.f16A, &psFocPMSM->sAllChnlOffsets.sFilterMA_phA_ANA);
		 psFocPMSM->sAllChnlOffsets.f16PhB_ANB = GDFLIB_FilterMA_F16(psFocPMSM->sAllChnlOffsets.sIRaw.f16B, &psFocPMSM->sAllChnlOffsets.sFilterMA_phB_ANB);
	        break;

	 case 6: 
	 case 1:
	 default:
		 	// phase C,B is sampled based on current SVM number  
		 psFocPMSM->sAllChnlOffsets.f16PhC_ANA = GDFLIB_FilterMA_F16(psFocPMSM->sAllChnlOffsets.sIRaw.f16A, &psFocPMSM->sAllChnlOffsets.sFilterMA_phC_ANA);
		 psFocPMSM->sAllChnlOffsets.f16PhB_ANB = GDFLIB_FilterMA_F16(psFocPMSM->sAllChnlOffsets.sIRaw.f16B, &psFocPMSM->sAllChnlOffsets.sFilterMA_phB_ANB);
	        break;
	}
}

#pragma section CODES_IN_RAM end

/******************************************************************************
* Inline functions
******************************************************************************/




