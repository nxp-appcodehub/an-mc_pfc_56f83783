/*
 * Copyright 2020 NXP
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* ###################################################################
**     Filename    : Cpu.h
**     Processor   : MC56F83xxx
**     Component   : MC56F83xxx
**     Version     : 0.1
**     Datasheet   : MC56F83xxx Reference Manual
**     Compiler    : CodeWarrior DSP C Compiler
**     Date/Time   : 2019-09-12, 18:33
*/

#ifndef __Cpu_H
#define __Cpu_H

#ifdef SUPPORT_64BIT_DATA_TYPE
#pragma slld on
#endif

#include <stdint.h>

#pragma define_section CODES_IN_RAM "codesInRam.text" RX
#pragma define_section NVM_DATA "nvm.data" RW

#define __EI0()                 \
    do                          \
    {                           \
        asm {                   \
        .iasm_volatile on;      \
        bfclr #0x0300, SR;      \
        .iasm_volatile off;     \
        };                      \
    } while (0) /* Enable interrupts of level 0,1,2,3 */
#define __EI1()                 \
    do                          \
    {                           \
        asm {                   \
        .iasm_volatile on;      \
        bfset #0x0100, SR;      \
        bfclr #0x0200, SR;      \
        .iasm_volatile off;     \
        };                      \
    } while (0) /* Enable interrupts of level 1,2,3 */
#define __EI2()                 \
    do                          \
    {                           \
        asm {                   \
        .iasm_volatile on;      \
        bfset #0x0200, SR;      \
        bfclr #0x0100, SR;      \
        .iasm_volatile off;     \
        };                      \
    } while (0) /* Enable interrupts of level 2,3 */
#define __EI3()                 \
    do                          \
    {                           \
        asm {                   \
        .iasm_volatile on;      \
        bfset #0x0300, SR;      \
        .iasm_volatile off;     \
        };                      \
    } while (0) /* Enable interrupts of level 3 */
/*lint -save  -esym(960,14.3) Disable MISRA rule (14.3) checking. */
#define __EI(level) __EI##level() /* Enable interrupts of the given level */
/*lint -restore  +esym(960,14.3) Enable MISRA rule (14.3) checking. */
#define __DI() __EI3() /* Disable interrupts, only level 3 is allowed */

#define Cpu_SetStopMode() asm(STOP) /* Set STOP mode */
#define __NOP()    asm(nop)
#define __DSB()    asm(nop)
#define __ISB()    asm(nop)
#define __WFI()    asm(WAIT)

/*
** ===================================================================
**     Method      :  Cpu_SetStopMode
**     Description :
**         Sets low power mode - Stop mode.
**         For more information about the stop mode see this CPU
**         documentation.
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/

#define Cpu_SetWaitMode() asm(WAIT) /* Set WAIT mode */
/*
** ===================================================================
**     Method      :  Cpu_SetWaitMode
**     Description :
**         Sets low power mode - Wait mode.
**         For more information about the wait mode see this CPU
**         documentation.
**         Release from wait mode: Reset or interrupt
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/

#define Cpu_DisableInt() __DI() /* Disable interrupts */
/*
** ===================================================================
**     Method      :  Cpu_DisableInt
**     Description :
**         Disables all maskable interrupts
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/

#define Cpu_EnableInt() __EI(0) /* Enable interrupts */
/*
** ===================================================================
**     Method      :  Cpu_EnableInt
**     Description :
**         Enables all maskable interrupts
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/

/* END Cpu. */

#endif
/* __Cpu_H */
