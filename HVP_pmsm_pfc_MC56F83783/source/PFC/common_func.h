
/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef COMMON_FUNC_H_
#define COMMON_FUNC_H_

#include "PFC_statemachine.h"
#include "Peripherals.h"
#include "mlib.h"
#include "gdflib.h"

inline void PFCDRV_PWMA3_update(frac16_t f16Duty1, frac16_t f16Duty2)
{
	frac16_t f16duty_temp;
	frac16_t f16val1, f16val2;
	frac32_t f32val1, f32val2;

	f16duty_temp = MLIB_Sub_F16(FRAC16(1.0), f16Duty2);

#if NANO_EDGE_EN  
	/*****Nano edge placement is enabled, VALx register and 
	 its corresponding FRACVALx register are written together *****/
	f32val1 = MLIB_Mul_F32ss(PWMA->SM[3].VAL1, f16duty_temp);
	f32val2 = MLIB_Mul_F32ss(PWMA->SM[3].VAL1, f16Duty1);
	PWMA->MCTRL |= PWM_MCTRL_CLDOK(0x8); // LDOK = 0
	*gsPFC_PwmVal.pui32PwmFrac3Val3 = f32val2;
	*gsPFC_PwmVal.pui32PwmFrac2Val2 = MLIB_Neg_F32(f32val2);
    if(gsPFC_Drive.sFlag.InterleaveFlag)
    {
	    *gsPFC_PwmVal.pui32PwmFrac4Val4 = f32val1;
	    *gsPFC_PwmVal.pui32PwmFrac5Val5 = MLIB_Neg_F32(f32val1);
    }
#else
    /*****Nano edge placement is disabled *****/
	f16val1 = MLIB_Mul_F16(PWMA->SM[3].VAL1, f16duty_temp);
	f16val2 = MLIB_Mul_F16(PWMA->SM[3].VAL1, f16Duty1);
	PWMA->MCTRL |= PWM_MCTRL_CLDOK(0x8); // LDOK = 0
	if(gsPFC_Drive.sFlag.InterleaveFlag)
	{
	    PWMA->SM[3].VAL4 = f16val1;
	    PWMA->SM[3].VAL5 = MLIB_Neg_F16(f16val1);
	}
	PWMA->SM[3].VAL3 = f16val2;
	PWMA->SM[3].VAL2 = MLIB_Neg_F16(f16val2);
#endif
	PWMA->MCTRL |= PWM_MCTRL_LDOK(0x8); // LDOK = 1
}

inline static void PFC_UInPeak_detect(PFCDEF_DRIVE_T *ptr)
{
	if((ptr->sUInPeakDetection.f16UInFilt > ptr->sUInPeakDetection.f16UInMaxTemp) && ptr->sUInPeakDetection.bPeakDetectEn)
	{
		gsPFC_Drive.sUInPeakDetection.f16UInMaxTemp = gsPFC_Drive.sUInPeakDetection.f16UInFilt;
	}
	
	// Voltage rising detection, enable peak detection when voltage is continuously rising 
	if(ptr->sUInPeakDetection.f16UInFilt > ptr->sUInPeakDetection.f16UInFilt_1)
	{
		ptr->sUInPeakDetection.u16UpCnt++;
		ptr->sUInPeakDetection.u16DownCnt = 0;
		if((ptr->sUInPeakDetection.u16UpCnt >= PEAK_DET_INPUT_VOLTAGE_RISE_NUM) && !ptr->sUInPeakDetection.bPeakDetectEn)
		{
			ptr->sUInPeakDetection.bPeakDetectEn = 1; //start peak detection of this cycle when confirmed that the voltage is rising
#if PLL
			ptr->sPhaseDetector.bRiseThrsdDetEn = 1; /*for phase detect, start rising edge threshold value
			                                         detection when confirmed that the voltage is rising*/
#endif
		}
		
	}
	// Voltage falling detection, disable peak detection when voltage is continuously falling
	// Peak voltage is detected in this procedure
	else if(ptr->sUInPeakDetection.f16UInFilt < ptr->sUInPeakDetection.f16UInFilt_1)
	{
		ptr->sUInPeakDetection.u16UpCnt = 0;
		ptr->sUInPeakDetection.u16DownCnt++;
		
		/*disable peak detection when confirmed that the voltage is falling, the stored peak value f16UInMaxTemp is the peak voltage of this cycle. */
		if((ptr->sUInPeakDetection.u16DownCnt >= PEAK_DET_INPUT_VOLTAGE_FALL_NUM) && ptr->sUInPeakDetection.bPeakDetectEn)
		{
			ptr->sUInPeakDetection.bPeakDetectEn = 0; 
#if PLL		
			ptr->sPhaseDetector.bFallThrsdDetEn = 1; /*for phase detect, start falling edge threshold value
			                                         detection when confirmed that the voltage is falling*/
#endif
			ptr->sUInPeakDetection.bPeakFlag = 1; 
			ptr->sUInPeakDetection.f16UInMax = ptr->sUInPeakDetection.f16UInMaxTemp;
			ptr->sUInPeakDetection.f16UInMaxSquare = MLIB_Mul_F16(ptr->sUInPeakDetection.f16UInMax, ptr->sUInPeakDetection.f16UInMax);

			// adjust bus voltage controller output according to the input voltage to keep the same minimum and maximum allowable current 
			if(ePFC_StateRunSub == SOFTSTART) 
			{
				ptr->sUCtrl.sUDcBusPiParams.f16LowerLim = FRAC16(0.0);	
			}
			else
			{
				ptr->sUCtrl.sUDcBusPiParams.f16LowerLim = MLIB_Mul_F16as(ACC32(LOW_CURRENT*LOOP_SCALE), ptr->sUInPeakDetection.f16UInMax);
			}
			ptr->sUCtrl.sUDcBusPiParams.f16UpperLim = MLIB_MulSat_F16as(ACC32(HIGH_CURRENT*LOOP_SCALE), ptr->sUInPeakDetection.f16UInMax);
			ptr->sUInPeakDetection.f16UInMaxTemp = 0;		
		}
	}	
	ptr->sUInPeakDetection.f16UInFilt_1 = ptr->sUInPeakDetection.f16UInFilt;
	
	/* PFC starts to work at least PEAK_DET_INPUT_VOLTAGE_PEAK_NUM number of peaks have been detected */
	if(!ptr->sUInPeakDetection.bSeveralPeaksDetectedFlag && ptr->sUInPeakDetection.bPeakFlag)
	{
		ptr->sUInPeakDetection.bPeakFlag = 0;
		ptr->sUInPeakDetection.u16VoltagePeakCnt ++;
		if(ptr->sUInPeakDetection.u16VoltagePeakCnt > PEAK_DET_INPUT_VOLTAGE_PEAK_NUM)
		{
			ptr->sUInPeakDetection.bSeveralPeaksDetectedFlag = 1;
		}
	}
}

/* PFC_Phase_detect is called in current control loop of PFC when compilation condition "PLL" is not zero*/
inline static void PFC_Phase_detect(PFCDEF_DRIVE_T *ptr)
{	
	ptr->sPhaseDetector.u16TimeCnt++;
	
	// Voltage zero-crossing detection when input AC voltage is indeed falling down
	if(ptr->sPhaseDetector.bFallThrsdDetEn)
	{
		if(ptr->sUInPeakDetection.f16UInFilt < ptr->sPhaseDetector.f16UInThreshold)
		{
			ptr->sPhaseDetector.u16FallOkCnt++;
		}
		else
		{
			ptr->sPhaseDetector.u16FallOkCnt = 0;	
		}
		if(ptr->sPhaseDetector.u16FallOkCnt >= PHASE_DET_INPUT_VOLTAGE_FALL_CNT_THRSD)
		{
			ptr->sPhaseDetector.bFallThrsdDetEn = 0; // Falling voltage threshold has been detected, disable the detection
			ptr->sPhaseDetector.u16FallOkCnt = 0;
			
			if(ptr->sUInPeakDetection.u16VoltagePeakCnt > 2)
			{
				ptr->sPhaseDetector.u16Period = ptr->sPhaseDetector.u16TimeCnt;
				ptr->sPhaseDetector.u16PeriodFilt = GDFLIB_FilterMA_F16(ptr->sPhaseDetector.u16Period, &ptr->sPhaseDetector.sPeriodFilter);
				ptr->sPhaseDetector.f16Freq = MLIB_Div_F16(1, ptr->sPhaseDetector.u16PeriodFilt);					
			}
			else if(ptr->sUInPeakDetection.u16VoltagePeakCnt == 2) //Consider different starting voltage point, discard the first period value
			{
				ptr->sPhaseDetector.sPeriodFilter.a32Acc = ptr->sPhaseDetector.u16TimeCnt * (PHASE_DET_INPUT_VOLTAGE_PERIOD_FILTER_POINTS-1);
				
			}
			ptr->sPhaseDetector.u16TimeCnt = 0;
		}
	}
	// Voltage zero-crossing detection when input AC voltage is rising up
	else if(ptr->sPhaseDetector.bRiseThrsdDetEn)
	{
		if(ptr->sUInPeakDetection.f16UInFilt > ptr->sPhaseDetector.f16UInThreshold)
		{
			ptr->sPhaseDetector.u16RiseOkCnt ++;
		}
		else 
		{
			ptr->sPhaseDetector.u16RiseOkCnt = 0;
		}
		if(ptr->sPhaseDetector.u16RiseOkCnt >= PHASE_DET_INPUT_VOLTAGE_RISE_CNT_THRSD)
		{
			ptr->sPhaseDetector.bRiseThrsdDetEn = 0; // Rising voltage threshold has been detected, disable the detection
			ptr->sPhaseDetector.u16RiseOkCnt = 0;
			ptr->sPhaseDetector.u16CntThrsd2Zero = (ptr->sPhaseDetector.u16TimeCnt >> 1) - PHASE_DET_INPUT_VOLTAGE_FALL_CNT_THRSD;
		}
	}
	
	if(ptr->sPhaseDetector.u16TimeCnt == ptr->sPhaseDetector.u16CntThrsd2Zero)
	{
		ptr->sPhaseDetector.f16Phase = 0;
	}
	else
	{
		ptr->sPhaseDetector.f16Phase = MLIB_AddSat_F16(ptr->sPhaseDetector.f16Phase, ptr->sPhaseDetector.f16Freq);
	}
	
	ptr->sPhaseDetector.f16Sine = GFLIB_Sin_F16(ptr->sPhaseDetector.f16Phase);

}
#endif /* COMMON_FUNC_H_ */
