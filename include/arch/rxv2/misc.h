/*
 * Copyright (c) 2021 KT-Elektronik, Klaucke und Partner GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Renesas RXv2 public kernel miscellaneous
 *
 *  Renesas RXv2-specific kernel miscellaneous interface. Included by arch/rxv2/arch.h.
 */

#ifndef ZEPHYR_INCLUDE_ARCH_RXV2_MISC_H_
#define ZEPHYR_INCLUDE_ARCH_RXV2_MISC_H_

#ifdef __cplusplus
extern "C" {
#endif

#ifndef _ASMLANGUAGE
extern uint32_t z_timer_cycle_get_32(void);

static inline uint32_t arch_k_cycle_get_32(void)
{
	return z_timer_cycle_get_32();
}

static ALWAYS_INLINE void arch_nop(void)
{
	__asm__ volatile("nop;");
}

#define arch_brk()  __asm__ volatile("brk;")
#define arch_wait() __asm__ volatile("wait;")

#endif

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_ARCH_RXV2_MISC_H_ */
