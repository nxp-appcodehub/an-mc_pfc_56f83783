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
* Main state machine realization
* Motor sub state machine.
*
******************************************************************************/
#ifndef _MC_STATEMACHINE_H_
#define _MC_STATEMACHINE_H_

/******************************************************************************
* Includes
******************************************************************************/
#include "cpu.h"
/* library headers */
#include "gmclib.h"
#include "gflib.h"
#include "gdflib.h"
#include "amclib.h"

/* application constants */
#include "PMSM_SpeedVectorCtrl.h"
#include "state_machine.h"
#include "motor_structure.h"


/******************************************************************************
* Constants
******************************************************************************/

/******************************************************************************
* Macros 
******************************************************************************/

/******************************************************************************
* Types
******************************************************************************/

/******************************************************************************
* Global variables
******************************************************************************/
extern SM_APP_CTRL_T 					gsMC_Ctrl;
extern MCSTRUC_FOC_PMSM_OBS_DQ_T		gsMC_Drive;
extern MCSTRUC_ADC_2_PHASE_ASSIGNMENT_T gsMC_IChannels;
extern GMCLIB_2COOR_T_F16 				gsMC_I;
extern GMCLIB_2COOR_T_F16 				gsMC_IOffset;
extern MCSTRUC_CONTROL_LOOP_T 			geMC_StateRunLoop;

extern MCSTRUC_FAULT_RECORD_T 			faultRecordFlag;

extern uint16_t 							uw16WaitForWakeUpFlag;
extern uint16_t 							uw16DelayEnterStopCntr;

/******************************************************************************
* Global functions
******************************************************************************/

extern void MC_Fault(void);

/******************************************************************************
* Inline functions
******************************************************************************/

#endif /* _MC_STATEMACHINE_H_ */

