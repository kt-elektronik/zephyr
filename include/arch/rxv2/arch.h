/*
 * Copyright (c) 2013-2014 Wind River Systems, Inc.
 * Copyright (c) 2020 KT-Elektronik, Klaucke und Partner GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */
//
/**
 * @file
 * @brief Renesas RX65N specific kernel interface header
 *
 * This header contains the Renesas RX65N specific kernel interface. It is
 * included by the kernel interface architecture-abstraction header
 * (include/arm/cpu.h) check if: ???
 */
//

#ifndef ZEPHYR_INCLUDE_ARCH_RXV2_ARCH_H_
#define ZEPHYR_INCLUDE_ARCH_RXV2_ARCH_H_

/* Add include for DTS generated information */
#include <devicetree.h>

#include <arch/rxv2/thread.h>
#include <arch/rxv2/misc.h>
#include <arch/rxv2/asm_inline_gcc.h>
#include <arch/common/sys_bitops.h>
#include <arch/common/sys_io.h>
#include <arch/common/ffs.h>
#include <sw_isr_table.h>
/* Common mstatus bits. All supported cores today have the same
 * layouts.
 */

#define MSTATUS_IEN     (1UL << 3)
#define MSTATUS_MPP_M   (3UL << 11)
#define MSTATUS_MPIE_EN (1UL << 7)
#define MSTATUS_FS_INIT (1UL << 13)
#define MSTATUS_FS_MASK ((1UL << 13) | (1UL << 14))

#ifdef __cplusplus
extern "C" {
#endif

/* internal routine documented in C file, needed by IRQ_CONNECT() macro */
extern void z_irq_priority_set(uint32_t irq, uint32_t prio, uint32_t flags);

/* There is no notion of priority with the Nios II internal interrupt
 * controller and no flags are currently supported.
 */
#define ARCH_IRQ_CONNECT(irq_p, priority_p, isr_p, isr_param_p, flags_p) \
{ \
	Z_ISR_DECLARE(irq_p, 0, isr_p, isr_param_p); \
}

#ifdef CONFIG_64BIT
#define ARCH_STACK_PTR_ALIGN 8
#else
#define ARCH_STACK_PTR_ALIGN 4
#endif

// extern void arch_irq_enable(unsigned int irq); // check if: required here
// extern void arch_irq_disable(unsigned int irq); // check if: required here

//#ifdef _ASMLANGUAGE

struct esf{
	uint32_t dummy; // maybe we will want to add something someday
};

typedef struct esf z_arch_esf_t;

static ALWAYS_INLINE unsigned int arch_irq_lock(void){
	return 0;
}

static inline void arch_irq_unlock(unsigned int key){
	ARG_UNUSED(key);
}

static inline bool arch_irq_unlocked(unsigned int key){
	return 0;
}

//#endif /* _ASMLANGUAGE */

#ifdef __cplusplus
}
#endif

#endif // ZEPHYR_INCLUDE_ARCH_RXV2_ARCH_H_