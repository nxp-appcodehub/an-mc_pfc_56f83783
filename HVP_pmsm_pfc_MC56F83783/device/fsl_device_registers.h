/*
 * Copyright 2014-2016 Freescale Semiconductor, Inc.
 * Copyright 2016-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#ifndef __FSL_DEVICE_REGISTERS_H__
#define __FSL_DEVICE_REGISTERS_H__

/*
 * Include the cpu specific register header files.
 *
 * The CPU macro should be declared in the project or makefile.
 */
#if (defined(CPU_MC56F83783AVLH) || defined(CPU_MC56F83783VLH))

#define MC56F83783_SERIES

/* CMSIS-style register definitions */
#include "MC56F83783.h"
/* CPU specific feature definitions */
#include "MC56F83783_features.h"

#else
#error "No valid CPU defined!"
#endif

#endif /* __FSL_DEVICE_REGISTERS_H__ */

/*******************************************************************************
 * EOF
 ******************************************************************************/
