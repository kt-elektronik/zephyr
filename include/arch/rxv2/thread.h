/*
 * Copyright (c) 2017 Intel Corporation
 * Copyright (c) 2021 KT-Elektronik, Klaucke und Partner GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Per-arch thread definition
 *
 * This file contains definitions for
 *
 *  struct _thread_arch
 *  struct _callee_saved
 *
 * necessary to instantiate instances of struct k_thread.
 */

#ifndef ZEPHYR_INCLUDE_ARCH_RXV2_THREAD_H_
#define ZEPHYR_INCLUDE_ARCH_RXV2_THREAD_H_

#ifndef _ASMLANGUAGE
#include <zephyr/types.h>

#ifdef __cplusplus
extern "C" {
#endif

struct _callee_saved {
	/* General purpose callee-saved registers */
	uint32_t r0; // check if: a) r0 exists & b) name adaptation according to RX65N spec
	uint32_t r1; // check if: name adaptation according to RX65N spec
	uint32_t r2; // check if: name adaptation according to RX65N spec
	uint32_t r3; // check if: name adaptation according to RX65N spec
	uint32_t r4; // check if: name adaptation according to RX65N spec
	uint32_t r5; // check if: name adaptation according to RX65N spec
	uint32_t r6; // check if: name adaptation according to RX65N spec
	uint32_t r7; // check if: name adaptation according to RX65N spec

	/* processor status register */
	uint32_t psr;	

	/* Return address */
	uint32_t ra;

	/* Stack pointer */
	uint32_t sp;

	/* IRQ status before irq_lock() and call to z_swap() */
	uint32_t key;

	/* Return value of z_swap() */
	uint32_t retval;

	/* Thread status pointer */
	void *thread_status;
};

typedef struct _callee_saved _callee_saved_t;

struct _thread_arch { // check if: further declarations are required
	uint32_t flags;
	uint32_t swap_return_value; /* Return value of z_swap() */	
};

typedef struct _thread_arch _thread_arch_t;

#ifdef __cplusplus
}
#endif

#endif /* _ASMLANGUAGE */

#endif /* ZEPHYR_INCLUDE_ARCH_RXV2_THREAD_H_ */