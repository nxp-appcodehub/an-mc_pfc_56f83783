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
* Peripheral and hw control macros.
*
******************************************************************************/
#ifndef _HWCONTROL_H_
#define _HWCONTROL_H_
#include "cpu.h"
#include "gmclib.h"
#include "PMSM_SpeedVectorCtrl.h"
#include "fsl_device_registers.h"
#include "pin_mux.h"
#include "fsl_gpio.h"

/* Constant definition*/
#define PWMA_SM012_RUN() PWMA->MCTRL |= PWM_MCTRL_RUN(0x07)
#define PWMA_SM0123_RUN() PWMA->MCTRL |= PWM_MCTRL_RUN(0x0F)
#define PIT0_RUN() PIT0->CTRL |= PIT_CTRL_CNT_EN_MASK
#define RELAY_ON() GPIO_PinSet(BOARD_RELAY_GPIO, BOARD_RELAY_PIN_MASK)
#define RELAY_OFF() GPIO_PinClear(BOARD_RELAY_GPIO, BOARD_RELAY_PIN_MASK)
#define BRAKE_ON() GPIO_PinSet(BOARD_BRAKE_GPIO, BOARD_BRAKE_PIN_MASK)
#define BRAKE_OFF() GPIO_PinClear(BOARD_BRAKE_GPIO, BOARD_BRAKE_PIN_MASK)
#define PH1_PWM_DIS() PWMA->OUTEN &= ~PWM_OUTEN_PWMA_EN(8);
#define PH2_PWM_DIS() PWMA->OUTEN &= ~PWM_OUTEN_PWMB_EN(8);
#define PH1_PWM_EN() PWMA->OUTEN |= PWM_OUTEN_PWMA_EN(8);
#define PH2_PWM_EN() PWMA->OUTEN |= PWM_OUTEN_PWMB_EN(8);

#define MC_DISABLE_PWM_OUTPUT() PWMA->OUTEN &= ~(PWM_OUTEN_PWMA_EN(0x7)|PWM_OUTEN_PWMB_EN(0x7))
#define MC_ENABLE_PWM_OUTPUT() PWMA->OUTEN |= (PWM_OUTEN_PWMA_EN(0x7)|PWM_OUTEN_PWMB_EN(0x7))
#define MC_OVERCURRENT_FAULT() (PWMA->FAULT[0].FSTS & PWM_FSTS_FFLAG(1))
#define MC_CLEAR_OVERCURRENT_FAULT() PWMA->FAULT[0].FSTS |= PWM_FSTS_FFLAG(1)
#define ALL_PFC_PWM_DIS() PWMA->OUTEN &= ~(PWM_OUTEN_PWMA_EN(0x8)|PWM_OUTEN_PWMB_EN(0x8))
#define ALL_PFC_PWM_EN()  PWMA->OUTEN |= (PWM_OUTEN_PWMA_EN(0x8)|PWM_OUTEN_PWMB_EN(0x8))

#define PFC_OVERCURRENT_FAULT() (PWMA->FAULT[0].FSTS & PWM_FSTS_FFLAG(1))
#define PFC_OVERVOLTAGE_FAULT() (PWMA->FAULT[0].FSTS & PWM_FSTS_FFLAG(2))
#define PFC_CLEAR_OVERCURRENT_FAULT() PWMA->FAULT[0].FSTS |= PWM_FSTS_FFLAG(1)
#define PFC_CLEAR_OVERVOLTAGE_FAULT() PWMA->FAULT[0].FSTS |= PWM_FSTS_FFLAG(2)

//#define MC_MODULO_HALF  (uint16_t)((MC_PWM_CLK_FREQ / MC_FAST_CONTROL_LOOP_FREQ)/2.0)
#define MC_MODULO_HALF  PWMA->SM[0].VAL1

/*MR20141208_C*/
#define DISABLE_WATCHDOG()                      ( COP->CTRL &= ~(0x0002|0x0008) )
#define ENABLE_WATCHDOG()                       ( COP->CTRL |= 0x0002 )
#define KICK_THE_WATCHDOG()                     do{ COP->CNTR = 0x5555; COP->CNTR = 0xAAAA; }while(0)


inline void MC_PWM_UPDATE(GMCLIB_3COOR_T_F16 *psDuty)
{
	int16_t w16ModuloHalf = MC_MODULO_HALF;
	int16_t w16Result;
	
	/* PWM channels 0&1  */ 
	w16Result = MLIB_MulRnd_F16(psDuty->f16A, w16ModuloHalf);

	PWMA->SM[0].VAL2 = -w16Result; 
	PWMA->SM[0].VAL3 = w16Result; 

	/* PWM channels 2&3 */								 
	w16Result = MLIB_MulRnd_F16(psDuty->f16B, w16ModuloHalf);
	
	PWMA->SM[1].VAL2 = -w16Result; 
	PWMA->SM[1].VAL3 = w16Result; 
														 
	/* PWM channels 4&5 */								 
	w16Result = MLIB_MulRnd_F16(psDuty->f16C, w16ModuloHalf);

	PWMA->SM[2].VAL2 = -w16Result; 
	PWMA->SM[2].VAL3 = w16Result; 
														 
	/* load new values to PWM registers */				 
	//PWMA_MCTRL |= PWM_MCTRL_LDOK(1) | PWM_MCTRL_LDOK(2) | PWM_MCTRL_LDOK(4); /* LDOK */ 
	PWMA->MCTRL |= PWM_MCTRL_LDOK(0x7); /* LDOK */ 
	
}

#endif /* _HWCONTROL_H_ */

