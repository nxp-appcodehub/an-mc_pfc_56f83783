/*
 * Copyright (c) 2015-2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_common.h"
#define SDK_MEM_MAGIC_NUMBER 12345U

typedef struct _mem_align_control_block
{
    uint16_t identifier; /*!< Identifier for the memory control block. */
    uint16_t offset;     /*!< offset from aligned address to real address */
} mem_align_cb_t;

/* Component ID definition, used by tools. */
#ifndef FSL_COMPONENT_ID
#define FSL_COMPONENT_ID "platform.drivers.common"
#endif

#if ((!defined(__DSC__)) || (!defined(__CW__)))

#ifndef __GIC_PRIO_BITS
#if defined(ENABLE_RAM_VECTOR_TABLE)
uint32_t InstallIRQHandler(IRQn_Type irq, uint32_t irqHandler)
{
/* Addresses for VECTOR_TABLE and VECTOR_RAM come from the linker file */
#if defined(__CC_ARM) || defined(__ARMCC_VERSION)
    extern uint32_t Image$$VECTOR_ROM$$Base[];
    extern uint32_t Image$$VECTOR_RAM$$Base[];
    extern uint32_t Image$$RW_m_data$$Base[];

#define __VECTOR_TABLE Image$$VECTOR_ROM$$Base
#define __VECTOR_RAM Image$$VECTOR_RAM$$Base
#define __RAM_VECTOR_TABLE_SIZE (((uint32_t)Image$$RW_m_data$$Base - (uint32_t)Image$$VECTOR_RAM$$Base))
#elif defined(__ICCARM__)
    extern uint32_t __RAM_VECTOR_TABLE_SIZE[];
    extern uint32_t __VECTOR_TABLE[];
    extern uint32_t __VECTOR_RAM[];
#elif defined(__GNUC__)
    extern uint32_t __VECTOR_TABLE[];
    extern uint32_t __VECTOR_RAM[];
    extern uint32_t __RAM_VECTOR_TABLE_SIZE_BYTES[];
    uint32_t __RAM_VECTOR_TABLE_SIZE = (uint32_t)(__RAM_VECTOR_TABLE_SIZE_BYTES);
#endif /* defined(__CC_ARM) || defined(__ARMCC_VERSION) */
    uint32_t n;
    uint32_t ret;
    uint32_t irqMaskValue;

    irqMaskValue = DisableGlobalIRQ();
    if (SCB->VTOR != (uint32_t)__VECTOR_RAM)
    {
        /* Copy the vector table from ROM to RAM */
        for (n = 0; n < ((uint32_t)__RAM_VECTOR_TABLE_SIZE) / sizeof(uint32_t); n++)
        {
            __VECTOR_RAM[n] = __VECTOR_TABLE[n];
        }
        /* Point the VTOR to the position of vector table */
        SCB->VTOR = (uint32_t)__VECTOR_RAM;
    }

    ret = __VECTOR_RAM[irq + 16];
    /* make sure the __VECTOR_RAM is noncachable */
    __VECTOR_RAM[irq + 16] = irqHandler;

    EnableGlobalIRQ(irqMaskValue);

/* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
  exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif

    return ret;
}
#endif /* ENABLE_RAM_VECTOR_TABLE. */
#endif /* __GIC_PRIO_BITS. */

#if (defined(FSL_FEATURE_SOC_SYSCON_COUNT) && (FSL_FEATURE_SOC_SYSCON_COUNT > 0))
#if !(defined(FSL_FEATURE_SYSCON_STARTER_DISCONTINUOUS) && FSL_FEATURE_SYSCON_STARTER_DISCONTINUOUS)

void EnableDeepSleepIRQ(IRQn_Type interrupt)
{
    uint32_t intNumber = (uint32_t)interrupt;

    uint32_t index = 0;

    while (intNumber >= 32u)
    {
        index++;
        intNumber -= 32u;
    }

    SYSCON->STARTERSET[index] = 1u << intNumber;
    EnableIRQ(interrupt); /* also enable interrupt at NVIC */
}

void DisableDeepSleepIRQ(IRQn_Type interrupt)
{
    uint32_t intNumber = (uint32_t)interrupt;

    DisableIRQ(interrupt); /* also disable interrupt at NVIC */
    uint32_t index = 0;

    while (intNumber >= 32u)
    {
        index++;
        intNumber -= 32u;
    }

    SYSCON->STARTERCLR[index] = 1u << intNumber;
}
#endif /* FSL_FEATURE_SYSCON_STARTER_DISCONTINUOUS */
#endif /* FSL_FEATURE_SOC_SYSCON_COUNT */

void *SDK_Malloc(size_t size, size_t alignbytes)
{
    mem_align_cb_t *p_cb = NULL;
    uint32_t alignedsize = SDK_SIZEALIGN(size, alignbytes) + alignbytes + sizeof(mem_align_cb_t);
    union
    {
        void *pointer_value;
        uint32_t unsigned_value;
    } p_align_addr, p_addr;

    p_addr.pointer_value = malloc(alignedsize);

    if (p_addr.pointer_value == NULL)
    {
        return NULL;
    }

    p_align_addr.unsigned_value = SDK_SIZEALIGN(p_addr.unsigned_value + sizeof(mem_align_cb_t), alignbytes);

    p_cb             = (mem_align_cb_t *)(p_align_addr.unsigned_value - 4U);
    p_cb->identifier = SDK_MEM_MAGIC_NUMBER;
    p_cb->offset     = (uint16_t)(p_align_addr.unsigned_value - p_addr.unsigned_value);

    return p_align_addr.pointer_value;
}

void SDK_Free(void *ptr)
{
    union
    {
        void *pointer_value;
        uint32_t unsigned_value;
    } p_free;
    p_free.pointer_value = ptr;
    mem_align_cb_t *p_cb = (mem_align_cb_t *)(p_free.unsigned_value - 4U);

    if (p_cb->identifier != SDK_MEM_MAGIC_NUMBER)
    {
        return;
    }

    p_free.unsigned_value = p_free.unsigned_value - p_cb->offset;

    free(p_free.pointer_value);
}

/*!
 * @brief Delay function bases on while loop, every loop includes three instructions.
 *
 * @param count  Counts of loop needed for dalay.
 */
#if defined(SDK_DELAY_USE_DWT) && defined(DWT)
void enableCpuCycleCounter(void)
{
    /* Make sure the DWT trace fucntion is enabled. */
    if (CoreDebug_DEMCR_TRCENA_Msk != (CoreDebug_DEMCR_TRCENA_Msk & CoreDebug->DEMCR))
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    }

    /* CYCCNT not supported on this device. */
    assert(DWT_CTRL_NOCYCCNT_Msk != (DWT->CTRL & DWT_CTRL_NOCYCCNT_Msk));

    /* Read CYCCNT directly if CYCCENT has already been enabled, otherwise enable CYCCENT first. */
    if (DWT_CTRL_CYCCNTENA_Msk != (DWT_CTRL_CYCCNTENA_Msk & DWT->CTRL))
    {
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }
}

uint32_t getCpuCycleCount(void)
{
    return DWT->CYCCNT;
}
#elif defined __XCC__
extern uint32_t xthal_get_ccount(void);
void enableCpuCycleCounter(void)
{
    /* do nothing */
}

uint32_t getCpuCycleCount(void)
{
    return xthal_get_ccount();
}
#endif

#ifndef __XCC__
#if (!defined(SDK_DELAY_USE_DWT)) || (!defined(DWT))
#if defined(__CC_ARM) /* This macro is arm v5 specific */
/* clang-format off */
__ASM static void DelayLoop(uint32_t count)
{
loop
    SUBS R0, R0, #1
    CMP  R0, #0
    BNE  loop
    BX   LR
}
/* clang-format on */
#elif defined(__ARMCC_VERSION) || defined(__ICCARM__) || defined(__GNUC__)
/* Cortex-M0 has a smaller instruction set, SUBS isn't supported in thumb-16 mode reported from __GNUC__ compiler,
 * use SUB and CMP here for compatibility */
static void DelayLoop(uint32_t count)
{
    __ASM volatile("    MOV    R0, %0" : : "r"(count));
    __ASM volatile(
        "loop:                          \n"
#if defined(__GNUC__) && !defined(__ARMCC_VERSION)
        "    SUB    R0, R0, #1          \n"
#else
        "    SUBS   R0, R0, #1          \n"
#endif
        "    CMP    R0, #0              \n"

        "    BNE    loop                \n");
}
#endif /* defined(__CC_ARM) */
#endif /* (!defined(SDK_DELAY_USE_DWT)) || (!defined(DWT)) */
#endif /* __XCC__ */
/*!
 * @brief Delay at least for some time.
 *  Please note that, if not uses DWT, this API will use while loop for delay, different run-time environments have
 *  effect on the delay time. If precise delay is needed, please enable DWT delay. The two parmeters delay_us and
 *  coreClock_Hz have limitation. For example, in the platform with 1GHz coreClock_Hz, the delay_us only supports
 *  up to 4294967 in current code. If long time delay is needed, please implement a new delay function.
 *
 * @param delay_us  Delay time in unit of microsecond.
 * @param coreClock_Hz  Core clock frequency with Hz.
 */
void SDK_DelayAtLeastUs(uint32_t delay_us, uint32_t coreClock_Hz)
{
    assert(0U != delay_us);
    uint64_t count = USEC_TO_COUNT(delay_us, coreClock_Hz);
    assert(count <= UINT32_MAX);

#if defined(SDK_DELAY_USE_DWT) && defined(DWT) || (defined __XCC__) /* Use DWT for better accuracy */

    enableCpuCycleCounter();
    /* Calculate the count ticks. */
    count += getCpuCycleCount();

    if (count > UINT32_MAX)
    {
        count -= UINT32_MAX;
        /* Wait for cyccnt overflow. */
        while (count < getCpuCycleCount())
        {
        }
    }

    /* Wait for cyccnt reach count value. */
    while (count > getCpuCycleCount())
    {
    }
#else
    /* Divide value may be different in various environment to ensure delay is precise.
     * Every loop count includes three instructions, due to Cortex-M7 sometimes executes
     * two instructions in one period, through test here set divide 1.5. Other M cores use
     * divide 4. By the way, divide 1.5 or 4 could let the count lose precision, but it does
     * not matter because other instructions outside while loop is enough to fill the time.
     */
#if (__CORTEX_M == 7)
    count = count / 3U * 2U;
#else
    count = count / 4U;
#endif
    DelayLoop((uint32_t)count);
#endif /* defined(SDK_DELAY_USE_DWT) && defined(DWT) || (defined __XCC__) */
}

#else /* ( (!defined(__DSC__)) || (!defined(__CW__)) ) */

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Interrupt priority table. */
static uint16_t s_intPrioTable[(NUMBER_OF_INT_IRQ - 1U) / 8U + 1U] = {0U};

/*******************************************************************************
 * Code
 ******************************************************************************/

status_t EnableIRQWithPriority(IRQn_Type irq, uint8_t priNum)
{
    uint8_t regIndex;
    uint8_t bitIndex;
    uint16_t prioMask;
    uint16_t reg;
    uint16_t intcCtrl;

    regIndex = INTC_TYPE_REG_INDEX(irq);
    bitIndex = INTC_TYPE_BIT_INDEX(irq);
    prioMask = (3UL << bitIndex);
    /* Valid priority number is 0-3 */
    priNum = priNum & 0x03U;

    if (0U == priNum)
    {
        priNum = SDK_DSC_DEFAULT_INT_PRIO;
    }

    /* Disable global interrupt for atomic change. */
    intcCtrl   = INTC->CTRL;
    INTC->CTRL = intcCtrl | INTC_CTRL_INT_DIS_MASK;

    /* Save the priority in s_intPrioTable */
    reg = s_intPrioTable[regIndex];
    reg = (reg & ~prioMask) | (priNum << bitIndex);

    s_intPrioTable[regIndex] = reg;

    /* Set new priority in interrupt controller register. */
    reg = ((volatile uint16_t *)&(INTC->IPR0))[regIndex];
    reg = (reg & ~prioMask) | (priNum << bitIndex);

    ((volatile uint16_t *)&(INTC->IPR0))[regIndex] = reg;

    INTC->CTRL = intcCtrl;

    return kStatus_Success;
}

status_t DisableIRQ(IRQn_Type irq)
{
    uint8_t regIndex;
    uint8_t bitIndex;
    uint16_t reg;
    uint16_t intcCtrl;

    regIndex = INTC_TYPE_REG_INDEX(irq);
    bitIndex = INTC_TYPE_BIT_INDEX(irq);

    /* Disable global interrupt for atomic change. */
    intcCtrl   = INTC->CTRL;
    INTC->CTRL = intcCtrl | INTC_CTRL_INT_DIS_MASK;

    reg = ((volatile uint16_t *)&(INTC->IPR0))[regIndex];
    reg = reg & (~(3UL << bitIndex));

    ((volatile uint16_t *)&(INTC->IPR0))[regIndex] = reg;

    INTC->CTRL = intcCtrl;

    return kStatus_Success;
}

status_t EnableIRQ(IRQn_Type irq)
{
    uint8_t regIndex;
    uint8_t bitIndex;
    uint16_t prioMask;
    uint16_t reg;
    uint16_t intcCtrl;

    regIndex = INTC_TYPE_REG_INDEX(irq);
    bitIndex = INTC_TYPE_BIT_INDEX(irq);
    prioMask = (3UL << bitIndex);

    /* Disable global interrupt for atomic change. */
    intcCtrl   = INTC->CTRL;
    INTC->CTRL = intcCtrl | INTC_CTRL_INT_DIS_MASK;

    /* If priority in s_intPrioTable is 0, use SDK_DSC_DEFAULT_INT_PRIO. */
    if (0U == (s_intPrioTable[regIndex] & prioMask))
    {
        s_intPrioTable[regIndex] = (s_intPrioTable[regIndex] & ~prioMask) | (SDK_DSC_DEFAULT_INT_PRIO << bitIndex);
    }

    /* Set the interrupt priority with the priority in s_intPrioTable. */
    reg = ((volatile uint16_t *)&(INTC->IPR0))[regIndex];
    reg = (reg & ~prioMask) | (s_intPrioTable[regIndex] & prioMask);

    ((volatile uint16_t *)&(INTC->IPR0))[regIndex] = reg;

    INTC->CTRL = intcCtrl;

    return kStatus_Success;
}

/*
 * brief Set the IRQ priority.
 *
 * note The parameter priNum is range in 1~3, and its value is **NOT**
 * directly map to interrupt priority.
 *
 * - Some IPs maps 1 to priority 1, 2 to priority 2, 3 to priority 3
 * - Some IPs maps 1 to priority 0, 2 to priority 1, 3 to priority 2
 *
 * User should check chip's RM to get its corresponding interrupt priority
 */
status_t IRQ_SetPriority(IRQn_Type irq, uint8_t priNum)
{
    /*
     * If the interrupt is already enabled, the new priority will be set
     * to the register. If interrupt is not enabled, the new priority is
     * only saved in priority table s_intPrioTable, when interrupt enabled,
     * the priority value is set to register.
     */
    uint8_t regIndex;
    uint8_t bitIndex;
    uint16_t prioMask;
    uint16_t reg;
    uint16_t intcCtrl;

    regIndex = INTC_TYPE_REG_INDEX(irq);
    bitIndex = INTC_TYPE_BIT_INDEX(irq);
    prioMask = (3UL << bitIndex);

    /* Valid priority number is 0-3 */
    priNum = priNum & 0x03U;

    if (0U == priNum)
    {
        priNum = SDK_DSC_DEFAULT_INT_PRIO;
    }

    /* Disable global interrupt for atomic change. */
    intcCtrl   = INTC->CTRL;
    INTC->CTRL = intcCtrl | INTC_CTRL_INT_DIS_MASK;

    /* Save the priority in s_intPrioTable */
    reg = s_intPrioTable[regIndex];
    reg = (reg & ~prioMask) | (priNum << bitIndex);

    s_intPrioTable[regIndex] = reg;

    /*
     * If interrupt already enabled, set new priority
     * in interrupt controller register.
     */
    reg = ((volatile uint16_t *)&(INTC->IPR0))[regIndex];

    if (0U != (reg & prioMask))
    {
        reg = (reg & (~prioMask)) | (priNum << bitIndex);

        ((volatile uint16_t *)&(INTC->IPR0))[regIndex] = reg;
    }

    INTC->CTRL = intcCtrl;

    return kStatus_Success;
}

void *SDK_Malloc(size_t size, size_t alignbytes)
{
    mem_align_cb_t *p_cb = NULL;
    uint32_t alignedsize = SDK_SIZEALIGN(size, alignbytes) + alignbytes + sizeof(mem_align_cb_t);
    union
    {
        void *pointer_value;
        uint32_t unsigned_value;
    } p_align_addr, p_addr;

    p_addr.pointer_value = malloc(alignedsize);

    if (p_addr.pointer_value == NULL)
    {
        return NULL;
    }

    p_align_addr.unsigned_value = SDK_SIZEALIGN(p_addr.unsigned_value + sizeof(mem_align_cb_t), alignbytes);

    p_cb             = (mem_align_cb_t *)(p_align_addr.unsigned_value - 4U);
    p_cb->identifier = SDK_MEM_MAGIC_NUMBER;
    p_cb->offset     = (uint16_t)(p_align_addr.unsigned_value - p_addr.unsigned_value);

    return p_align_addr.pointer_value;
}

void SDK_Free(void *ptr)
{
    union
    {
        void *pointer_value;
        uint32_t unsigned_value;
    } p_free;
    p_free.pointer_value = ptr;
    mem_align_cb_t *p_cb = (mem_align_cb_t *)(p_free.unsigned_value - 4U);

    if (p_cb->identifier != SDK_MEM_MAGIC_NUMBER)
    {
        return;
    }

    p_free.unsigned_value = p_free.unsigned_value - p_cb->offset;

    free(p_free.pointer_value);
}

/*!
 * brief Delay core cycles.
 *  Please note that, this API uses software loop for delay, the actual delayed
 *  time depends on core clock frequency, where the function is located (ram or flash),
 *  flash clock, possible interrupt.
 *
 * param u32Num  Number of core clock cycle which needs to be delayed.
 */
void SDK_DelayCoreCycles(uint32_t u32Num)
{
    /*
     *  if(u32Num < 22)
     *  {
     *      ActualDelayCycle = 21;
     *  }
     *  else
     *  {
     *      ActualDelayCycle = 35 + ((u32Num-22)/8) * 8 = 13 + u32Num - ((u32Num-22)%8)
     *  }
     */

    /*  JSR - 4 cycles
     *  RTS - 8 cycles
     */

    asm {
       cmp.l #21,A // 2 cycle
       bls   ret // 5 cycles when jump occurs. 3 cycles when jump doesn't occur
       nop // 1 cycle
       nop // 1 cycle
       sub.l  #22, A // 2 cycle
       asrr.l #3, A // 2 cycle
       bra   test // 5 cycle

       loop:
       dec.l A // 1 cycle

       test:
       tst.l A // 1 cycle
       nop // 1 cycle
       bne   loop // 5 cycles when jump occurs. 3 cycles when jump doesn't occur

       ret:
       nop // 1 cycle
       nop // 1 cycle
    }
}

/*!
 * brief Delay at least for some time.
 *  Please note that, this API uses while loop for delay, different run-time environments make the time not precise,
 *  if precise delay count was needed, please implement a new delay function with hardware timer.
 *  The maximum delay time is limited with MCU core clock, for example CPU core clock = 100MHz, then the maximum delay
 *  time should be less than 0xFFFFFFFF/100 = 42,949,672us
 *
 * param delay_us  Delay time in unit of microsecond.
 * param coreClock_Hz  Core clock frequency with Hz.
 */
void SDK_DelayAtLeastUs(uint32_t delay_us, uint32_t coreClock_Hz)
{
    assert(0U != delay_us);
    uint32_t count = USEC_TO_COUNT(delay_us, coreClock_Hz);

    SDK_DelayCoreCycles(count);
}

#endif /* ( (!defined(__DSC__)) || (!defined(__CW__)) ) */
