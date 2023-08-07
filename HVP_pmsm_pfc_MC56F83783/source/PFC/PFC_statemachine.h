
/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef PFC_STATEMACHINE_H_
#define PFC_STATEMACHINE_H_

#include "state_machine.h"
#include "PFC_def.h"
#include "Cpu.h"

/******************************************************************************
* Types
******************************************************************************/
typedef enum {
    SOFTSTART          = 0,
    NORMAL             = 1,
    LIGHTLOAD          = 2
} PFC_RUN_SUBSTATE_T;         /* Run sub-states */

extern SM_APP_CTRL_T  gsPFC_Ctrl;
extern PFCDEF_DRIVE_T  gsPFC_Drive;
extern PFC_RUN_SUBSTATE_T  ePFC_StateRunSub; 
extern const PFCN_VOID_VOID mPFC_STATE_RUN_TABLE[3];

typedef struct
{
	uint32_t     *pui32PwmFrac2Val2;
	uint32_t     *pui32PwmFrac3Val3;
	uint32_t     *pui32PwmFrac4Val4;
	uint32_t     *pui32PwmFrac5Val5;
} PFCDRV_PWMVAL;
extern PFCDRV_PWMVAL        gsPFC_PwmVal;

extern void PFC_FaultDetection(void);
#endif /* PFC_STATEMACHINE_H_ */
