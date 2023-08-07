
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
* Main state machine frame.
*
******************************************************************************/


 
/******************************************************************************
* Includes
******************************************************************************/
#include "state_machine.h"

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

/******************************************************************************
* Local variables
******************************************************************************/

/******************************************************************************
* Local functions
******************************************************************************/

/*------------------------------------
 * Application state machine functions
 * ----------------------------------*/
static void SM_StateFault(SM_APP_CTRL_T *psAppCtrl);
static void SM_StateInit(SM_APP_CTRL_T *psAppCtrl);
static void SM_StateStop(SM_APP_CTRL_T *psAppCtrl);
static void SM_StateRun(SM_APP_CTRL_T *psAppCtrl);

/******************************************************************************
* Global functions
******************************************************************************/
/* State machine functions field (in pmem) */
PFCN_VOID_PSM gSM_STATE_TABLE[4] = {SM_StateFault, SM_StateInit, SM_StateStop, SM_StateRun};


/***************************************************************************//*!
*
* @brief   FAULT state
*
* @param   void
*
* @return  none
*
******************************************************************************/
#pragma section CODES_IN_RAM begin
static void SM_StateFault(SM_APP_CTRL_T *psAppCtrl)
{
    /* User Fault function */
    psAppCtrl->psState->Fault();
    
    /* if clear fault command flag */
    if ((psAppCtrl->uiCtrl & SM_CTRL_FAULT_CLEAR) > 0 )
    {
	    /* Clear INIT_DONE, FAULT, FAULT_CLEAR flags */
	   psAppCtrl->uiCtrl &= ~(SM_CTRL_INIT_DONE | SM_CTRL_FAULT | SM_CTRL_FAULT_CLEAR);
    	
       /* User Fault to Init transition function */
       psAppCtrl->psTrans->FaultInit();

	   /* Init state */
	   psAppCtrl->eState = INIT;
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
static void SM_StateInit(SM_APP_CTRL_T *psAppCtrl)
{
    /* User Init function */
    psAppCtrl->psState->Init();

	/* if fault flag */
	if ((psAppCtrl->uiCtrl & SM_CTRL_FAULT) > 0)
	{
	    /* User Init to Fault transition function */
	    psAppCtrl->psTrans->InitFault();
	    
		/* Fault state */
		psAppCtrl->eState = FAULT;
	}
	/* if INIT_DONE flag */
	else if ((psAppCtrl->uiCtrl & SM_CTRL_INIT_DONE) > 0)
	{
	    /* Clear INIT_DONE, START_STOP, OM_CHANGE, STOP_ACK, RUN_ACK flags */
		psAppCtrl->uiCtrl &= ~(SM_CTRL_INIT_DONE | SM_CTRL_STOP | SM_CTRL_START | SM_CTRL_STOP_ACK | SM_CTRL_RUN_ACK);
		
 		/* User Init to Stop transition function */
		psAppCtrl->psTrans->InitStop();

		/* Stop state */
		psAppCtrl->eState = STOP;
	}
}

/***************************************************************************//*!
*
* @brief   STOP state
*
* @param   void
*
* @return  none
*
******************************************************************************/
static void SM_StateStop(SM_APP_CTRL_T *psAppCtrl)
{
    /* User Stop function */
    psAppCtrl->psState->Stop();

    /* if fault */
    if ((psAppCtrl->uiCtrl & SM_CTRL_FAULT) > 0)
    {
	    /* User Stop to Fault transition function */
	    psAppCtrl->psTrans->StopFault();

		/* Fault state */
		psAppCtrl->eState = FAULT;
    }
	else if ((psAppCtrl->uiCtrl & SM_CTRL_START) > 0)
	{
		/* User Stop to Run transition function, user must set up the SM_CTRL_RUN_ACK
		flag to allow the RUN state */
		psAppCtrl->psTrans->StopRun();
		
		/* Clears the START command */
//		psAppCtrl->uiCtrl &= ~SM_CTRL_START;		

		if ((psAppCtrl->uiCtrl & SM_CTRL_RUN_ACK) > 0)
		{
			/* Clears the RUN_ACK flag */
			psAppCtrl->uiCtrl &= ~(SM_CTRL_RUN_ACK | SM_CTRL_START);
			
			/* Run state */
			psAppCtrl->eState = RUN;	
		}
	}
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

static void SM_StateRun(SM_APP_CTRL_T *psAppCtrl)
{
    psAppCtrl->psState->Run();


	if ((psAppCtrl->uiCtrl & SM_CTRL_FAULT) > 0)
    {
	    /* User Run to Fault transition function */
	    psAppCtrl->psTrans->RunFault();

		/* Fault state */
	    psAppCtrl->eState = FAULT; 	
    }
    else if ((psAppCtrl->uiCtrl & SM_CTRL_STOP) > 0)
	{
		/* User Run to Stop transition function, user must set up the SM_CTRL_STOP_ACK
		flag to allow the STOP state */
		psAppCtrl->psTrans->RunStop();
		
		/* Clears the STOP command */
//		psAppCtrl->uiCtrl &= ~SM_CTRL_STOP;

		if ((psAppCtrl->uiCtrl & SM_CTRL_STOP_ACK) > 0)
		{
			/* Clears the STOP_ACK flag */
			psAppCtrl->uiCtrl &= ~(SM_CTRL_STOP_ACK | SM_CTRL_STOP);
			
			/* Run state */
			psAppCtrl->eState = STOP;	
		}
	}
}
#pragma section CODES_IN_RAM end
/******************************************************************************
* Inline functions
******************************************************************************/




