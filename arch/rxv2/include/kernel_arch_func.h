/*
 * Copyright (c) 2021 KT-Elektronik, Klaucke und Partner GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_ARCH_RXV2_INCLUDE_KERNEL_ARCH_FUNC_H_
#define ZEPHYR_ARCH_RXV2_INCLUDE_KERNEL_ARCH_FUNC_H_

#ifndef _ASMLANGUAGE
#include <kernel_internal.h>
#include <kernel_arch_data.h>
#include <kernel_structs.h>
#include <string.h>
#include <stddef.h> // for size_t

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

static ALWAYS_INLINE void arch_kernel_init(void)
{
	// check if: further device initialization functions must be called here
}

extern void z_rxv2_arch_switch(void *switch_to, void **switched_from);

static inline void arch_switch(void *switch_to, void **switched_from)
{
	z_rxv2_arch_switch(switch_to, switched_from);
}

#ifdef __cplusplus
}
#endif // __cplusplus

#endif /* _ASMLANGUAGE */

#endif /* ZEPHYR_ARCH_RXV2_INCLUDE_KERNEL_ARCH_FUNC_H_ */
