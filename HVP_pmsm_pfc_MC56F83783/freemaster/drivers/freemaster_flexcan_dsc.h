/*
 * Copyright (c) 2007-2015 Freescale Semiconductor, Inc.
 * Copyright 2018-2020 NXP
 *
 * License: NXP LA_OPT_NXP_Software_License
 *
 * NXP Confidential. This software is owned or controlled by NXP and may
 * only be used strictly in accordance with the applicable license terms.
 * By expressly accepting such terms or by downloading, installing,
 * activating and/or otherwise using the software, you are agreeing that
 * you have read, and that you agree to comply with and are bound by,
 * such license terms.  If you do not agree to be bound by the applicable
 * license terms, then you may not retain, install, activate or otherwise
 * use the software.  This code may only be used in a microprocessor,
 * microcontroller, sensor or digital signal processor ("NXP Product")
 * supplied directly or indirectly from NXP.  See the full NXP Software
 * License Agreement in license/LA_OPT_NXP_Software_License.pdf
 *
 * FreeMASTER Communication Driver - DSC FlexCAN low-level driver
 */

#ifndef __FREEMASTER_FLEXCAN_DSC_H
#define __FREEMASTER_FLEXCAN_DSC_H

/******************************************************************************
* Required header files include check
******************************************************************************/
#ifndef __FREEMASTER_H
#error Please include the freemaster.h master header file before the freemaster_flexcan_dsc.h
#endif

/* This low-level driver uses the MCUXpresso SDK peripheral structure types. */
#include "fsl_device_registers.h"

/******************************************************************************
* Adapter configuration
******************************************************************************/

/* flexcan needs to know the transmit and receive MB number */
#if ((FMSTR_USE_FLEXCAN) || (FMSTR_USE_FLEXCAN32))
    /* Flexcan TX message buffer must be defined */
    #ifndef FMSTR_FLEXCAN_TXMB
    /* #error FlexCAN transmit buffer needs to be specified (use FMSTR_FLEXCAN_TXMB) */
    #warning "FlexCAN Message Buffer 0 is used for transmitting messages"
    #define FMSTR_FLEXCAN_TXMB 0
    #endif
    /* Flexcan RX message buffer must be defined */
    #ifndef FMSTR_FLEXCAN_RXMB
    /* #error FlexCAN receive buffer needs to be specified (use FMSTR_FLEXCAN_RXMB) */
    #warning "FlexCAN Message Buffer 1 is used for receiving messages"
    #define FMSTR_FLEXCAN_RXMB 1
    #endif
#endif


#ifdef __cplusplus
  extern "C" {
#endif
      
/******************************************************************************
* Types definition
******************************************************************************/

/******************************************************************************
* inline functions
******************************************************************************/

/******************************************************************************
* Global API functions
******************************************************************************/
      
void FMSTR_CanSetBaseAddress(CAN_Type *base);
void FMSTR_CanIsr(void);

#ifdef __cplusplus
  }
#endif

#endif /* __FREEMASTER_FLEXCAN_DSC_H */

