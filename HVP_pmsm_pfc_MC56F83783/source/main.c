
/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_device_registers.h"
#include "board.h"

#include "pin_mux.h"
#include "clock_config.h"
#include "peripherals.h"

#include "MC_statemachine.h"
#include "hwcontrol.h"
#include "mlib.h"

#include "common_func.h"
#include "PFC_statemachine.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
volatile uint16_t u16CntTst,u16CntPitTst;
volatile uint32_t u32CntTst;
volatile uint16_t w16ExeTimeMotor,w16ExeTimePfc1,w16ExeTimePfc_slowloop;
extern MCSTRUC_FAULT_RECORD_T	faultRecordFlag;		// Fault recorder variable
extern uint16_t								uw16LedCntBlkTimes;
extern uint16_t								uw16LedCntBlkDuration;
extern uint16_t								uw16LedCntPatternTimes;
extern uint16_t								uw16LedCntPatternSpace;
/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Main function
 */


int main(void)
{
    /* Init board hardware. */    
	BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();

    
    /* Enable the counter of PWMA to start the state machine */
    PWMA_SM0123_RUN();
    PIT0_RUN();
    
    /* Set speed command to zero */
    gsMC_Drive.sSpeed.f16SpeedCmd = 0;

    /* Led indication variables */
    uw16LedCntPatternSpace = 0;
    uw16LedCntBlkDuration = 0;
    uw16LedCntPatternTimes = 1;
    uw16LedCntBlkTimes = 1;
       
    
    while (1)
    {
  	
    	FMSTR_Poll();
    	gsMC_Drive.sSpeed.f16SpeedReq = gsMC_Drive.sSpeed.f16SpeedCmd;

    }
}


#pragma section CODES_IN_RAM begin
/* kADC12_CC0_VECTORn interrupt handler */
#pragma interrupt saveall
void ADC_A_IRQHANDLER(void) {
  /*  Place your code here */
	
	TMRA->CHANNEL[1].CNTR = 0x0;
	
	/* read ADC results */
	if(ADC->RDY & 0x0800)
	{
		gsPFC_Drive.sICtrlPh1.f16IFdbck = ADC->RSLT[9];
	    gsPFC_Drive.sICtrlPh2.f16IFdbck = ADC->RSLT[11];
		gsPFC_Drive.sUInPeakDetection.f16UIn = ADC->RSLT[10];
	}

	if(ADC->RDY & 0x4000)
	{
		gsPFC_Drive.sICtrlPh1.f16IFdbck = ADC->RSLT[12];
	    gsPFC_Drive.sICtrlPh2.f16IFdbck = ADC->RSLT[14];
		gsPFC_Drive.sUInPeakDetection.f16UIn = ADC->RSLT[13];		  
	}
	gsPFC_Drive.sUInPeakDetection.f16UInFilt = GDFLIB_FilterIIR1_F16(gsPFC_Drive.sUInPeakDetection.f16UIn, &gsPFC_Drive.sUInPeakDetection.sFilter);

#if PLL
	PFC_Phase_detect(&gsPFC_Drive);
#endif

	/* State machine call */
	SM_StateMachine(&gsPFC_Ctrl);
	
	w16ExeTimePfc1 = TMRA->CHANNEL[1].CNTR;

	if(++u32CntTst == 16000)
	{
		u32CntTst = 0;
		GPIO_PinToggle(BOARD_USER_LED_2_GPIO, BOARD_USER_LED_2_PIN_MASK);
	}
}
#pragma interrupt off



/* keFlexPWMA_CMP0_VECTORn interrupt handler */
#pragma interrupt alignsp saveall
void PWMA_COMPARE_0_IRQHANDLER(void) {
  /*  Place your code here */
	GPIO_PinSet(BOARD_TP22_GPIO,BOARD_TP22_PIN_MASK);
	
	TMRA->CHANNEL[0].CNTR = 0x0;
	while(!(ADC->RDY & 0x0001)); // Make sure the ADC results are ready
	
	
    
	
    /*========================= Read currents begin =========================*/
    gsMC_Drive.sFocPMSM.sAllChnlOffsets.sIRaw.f16A = ADC->RSLT[0];
    gsMC_Drive.sFocPMSM.sAllChnlOffsets.sIRaw.f16B = ADC->RSLT[8];
	/*========================= Read currents end  =========================*/
	
    /*========================= State machine begin========================================================*/
 
	/* Slow loop calculation */
	geMC_StateRunLoop = SLOW;
	
	/*StateMachine call */
	SM_StateMachine(&gsMC_Ctrl);
	
	/* Fast loop calculation */
	geMC_StateRunLoop = FAST;
	
	/* StateMachine call */
	SM_StateMachine(&gsMC_Ctrl);

    /*use ADC channel mapping After calibration*/
	/* Channel update */
	/* S0  S1  S2  S3   S4  S5  S6  S7 */
	/* IX  Udc                     */
	ADC->CLIST1 = (ADC->CLIST1 & ~ADC_CLIST1_SAMPLE0_MASK)|ADC_CLIST1_SAMPLE0(gsMC_Drive.sFocPMSM.sAllChnlOffsets.sIChnl.uw16Ph0);
	/* S8  S9  S10 S11  S12 S13 S14 S15 */
	/* IX  RTH                      */   
    ADC->CLIST3 = (ADC->CLIST3 & ~ADC_CLIST3_SAMPLE8_MASK)|ADC_CLIST3_SAMPLE8(gsMC_Drive.sFocPMSM.sAllChnlOffsets.sIChnl.uw16Ph1);
     
	
	if(++u16CntTst == 8000)
	{
		u16CntTst = 0;
	}
		
	
	PWMA->SM[0].STS = PWM_STS_CMPF(0x1); // Clear flag of Val0 compare 	
	GPIO_PinClear(BOARD_TP22_GPIO,BOARD_TP22_PIN_MASK);
	w16ExeTimeMotor = TMRA->CHANNEL[0].CNTR;
}
#pragma interrupt off



/* kPIT0_ROLLOVR_VECTORn interrupt handler */
#pragma interrupt alignsp saveall
void PIT0_IRQHANDLER(void) {
  /*  Place your code here */
	
	frac16_t f16PFCDCMRatioNum, f16PFCDCMRatioDen;
	
	TMRA->CHANNEL[2].CNTR = 0;
	
	gsPFC_Drive.sUCtrl.f16UDcBus = ADC->RSLT[3];
	gsPFC_Drive.sUCtrl.f16UDcBusFilt = GDFLIB_FilterIIR1_F16(gsPFC_Drive.sUCtrl.f16UDcBus, &gsPFC_Drive.sUCtrl.sUDcBusFilter);
	
	PFC_UInPeak_detect(&gsPFC_Drive);
		
	if(gsPFC_Ctrl.eState == RUN)
	{
		mPFC_STATE_RUN_TABLE[ePFC_StateRunSub]();

		if(gsPFC_Drive.sFlag.PWM_enable)
		{
			if(ePFC_StateRunSub == LIGHTLOAD) // constant current reference in light-load mode
			{
				gsPFC_Drive.sUCtrl.f16UDcBusCtrlOut = gsPFC_Drive.sUCtrl.sUDcBusPiParams.f16LowerLim;
			}
			else
			{
				gsPFC_Drive.sUCtrl.f16UDcBusError = MLIB_SubSat_F16(gsPFC_Drive.sUCtrl.f16UDcBusReq, gsPFC_Drive.sUCtrl.f16UDcBusFilt);
				gsPFC_Drive.sUCtrl.f16UDcBusError = GFLIB_Limit_F16(gsPFC_Drive.sUCtrl.f16UDcBusError,-gsPFC_Drive.sUCtrl.f16UDcBusErrLim,gsPFC_Drive.sUCtrl.f16UDcBusErrLim);
				gsPFC_Drive.sUCtrl.f16UDcBusCtrlOut = GFLIB_CtrlPIpAW_F16(gsPFC_Drive.sUCtrl.f16UDcBusError, &gsPFC_Drive.sUCtrl.bStopIntegFlag, &gsPFC_Drive.sUCtrl.sUDcBusPiParams);
			}
	   #if PLL
			gsPFC_Drive.sUCtrl.a32IRef = MLIB_Div1Q_A32ss(gsPFC_Drive.sUCtrl.f16UDcBusCtrlOut,gsPFC_Drive.sUInPeakDetection.f16UInMax);
	   #else
			gsPFC_Drive.sUCtrl.a32IRef = MLIB_Div1Q_A32ss(gsPFC_Drive.sUCtrl.f16UDcBusCtrlOut,gsPFC_Drive.sUInPeakDetection.f16UInMaxSquare);
	   #endif
			
			/* theoretical duty cycle calculation for compensation */
			gsPFC_Drive.f16DutyCCM = MLIB_Div_F16(gsPFC_Drive.sUCtrl.f16VoltDiff_UDc_UIn , gsPFC_Drive.sUCtrl.f16UDcBusFilt);
			f16PFCDCMRatioNum = MLIB_MulSat_F16as(PFC_DCM_DUTY_COEFF, gsPFC_Drive.sUCtrl.f16UDcBusCtrlOut);
			f16PFCDCMRatioNum = MLIB_Mul_F16(f16PFCDCMRatioNum,gsPFC_Drive.f16DutyCCM);
			f16PFCDCMRatioDen = gsPFC_Drive.sUInPeakDetection.f16UInMaxSquare;
			gsPFC_Drive.f16DutyDCM = GFLIB_Sqrt_F16(MLIB_DivSat_F16(f16PFCDCMRatioNum,f16PFCDCMRatioDen));
			if(gsPFC_Drive.f16DutyDCM > gsPFC_Drive.f16DutyCCM) // when calculated DCM duty is larger than CCM duty, it should be under CCM 
			{
				gsPFC_Drive.f16DutyComp = gsPFC_Drive.f16DutyCCM;
				gsPFC_Drive.sFlag.DCMFlag = 0;
			}
			else // when calculated CCM duty is larger than DCM duty, it should be under DCM 
			{
				gsPFC_Drive.f16DutyComp = gsPFC_Drive.f16DutyDCM;
				gsPFC_Drive.sFlag.DCMFlag = 1;
			}
		}
		
		PFC_FaultDetection();
		if(gsPFC_Drive.FaultIdPending.R)
		{
			gsPFC_Ctrl.uiCtrl |= SM_CTRL_FAULT;
		}
	}
	// Clear flag
	PIT0->CTRL &= ~PIT_CTRL_PRF_MASK;
	
	
	if(++u16CntPitTst == 10000)
	{
		u16CntPitTst = 0;
	}
	
	w16ExeTimePfc_slowloop = TMRA->CHANNEL[2].CNTR;
	
	PIT0->CTRL &= ~PIT_CTRL_PRF_MASK;
}
#pragma interrupt off
#pragma section CODES_IN_RAM end
