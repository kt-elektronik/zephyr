/*
 * Copyright (c) 2016 Wind River Systems, Inc.
 * Copyright (c) 2017 Oticon A/S
 * Copyright (c) 2020 KT-Elektronik, Klaucke und Partner GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* This file is only meant to be included by kernel_structs.h */

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

static ALWAYS_INLINE void arch_kernel_init(void){
    // check if: further device initialization functions must be called here
}

static ALWAYS_INLINE void arch_thread_return_value_set(struct k_thread *thread, unsigned int value){
	thread->callee_saved.retval = value;
//	thread->arch.swap_return_value = value;
}

static inline bool arch_is_in_isr(void){
	return _kernel.cpus[0].nested != 0U;
}

#ifdef __cplusplus
}
#endif // __cplusplus

#endif /* _ASMLANGUAGE */

#endif /* ZEPHYR_ARCH_RXV2_INCLUDE_KERNEL_ARCH_FUNC_H_ */
