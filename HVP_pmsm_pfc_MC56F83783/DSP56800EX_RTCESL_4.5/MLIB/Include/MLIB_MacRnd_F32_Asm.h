/*******************************************************************************
*
 * Copyright (c) 2013 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2019 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
* 
*
****************************************************************************//*!
*
* @brief  Multiply accumulate functions with 32-bit rounded fractional 
* 		  output in assembler
* 
*******************************************************************************/
#ifndef _MLIB_MACRND_F32_ASM_H_
#define _MLIB_MACRND_F32_ASM_H_

#if defined(__cplusplus) 
extern "C" { 
#endif 
/******************************************************************************
* Includes
******************************************************************************/
#include "mlib_types.h"
#include "MLIB_MacRnd_F32_Asmi.h"

/******************************************************************************
* Constants
******************************************************************************/

/******************************************************************************
* Macros 
******************************************************************************/
#define MLIB_MacRnd_F32lls_Asmi(f32Accum, f32Mult1, f16Mult2) MLIB_MacRnd_F32lls_FAsmi(f32Accum, f32Mult1, f16Mult2)
#define MLIB_MacRndSat_F32lls_Asmi(f32Accum, f32Mult1, f16Mult2) MLIB_MacRndSat_F32lls_FAsmi(f32Accum, f32Mult1, f16Mult2)

#define MLIB_MacRnd_F32_Asmi(f32Accum, f32Mult1, f32Mult2) MLIB_MacRnd_F32_FAsmi(f32Accum, f32Mult1, f32Mult2)
#define MLIB_MacRndSat_F32_Asmi(f32Accum, f32Mult1, f32Mult2) MLIB_MacRndSat_F32_FAsmi(f32Accum, f32Mult1, f32Mult2)

/******************************************************************************
* Types
******************************************************************************/

/******************************************************************************
* Global variables
******************************************************************************/

/******************************************************************************
* Global functions
******************************************************************************/

/******************************************************************************
* Inline functions
******************************************************************************/
#if defined(__cplusplus) 
} 
#endif 

#endif /* MLIB_MACRND_F32_ASM_H_ */
