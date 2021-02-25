/*
 * Copyright (c) 2010-2014 Wind River Systems, Inc.
 * Copyright (c) 2017 Intel Corporation
 * Copyright (c) 2021 KT-Elektronik, Klaucke und Partner GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief RXV2 Kernel structure member offset definition file
 *
 * This module is responsible for the generation of the absolute symbols whose
 * value represents the member offsets for various structures.
 *
 * All of the absolute symbols defined by this module will be present in the
 * final kernel ELF image (due to the linker's reference to the _OffsetAbsSyms
 * symbol).
 *
 * INTERNAL
 * It is NOT necessary to define the offset for every member of a structure.
 * Typically, only those members that are accessed by assembly language routines
 * are defined; however, it doesn't hurt to define all fields for the sake of
 * completeness.
 */

#ifndef _RXV2_OFFSETS_INC_
#define _RXV2_OFFSETS_INC_

#include <gen_offset.h>
#include <kernel.h>
#include <kernel_arch_data.h>
#include <kernel_offsets.h>

GEN_OFFSET_SYM(_callee_saved_t, r0);
GEN_OFFSET_SYM(_callee_saved_t, r1);
GEN_OFFSET_SYM(_callee_saved_t, r2);
GEN_OFFSET_SYM(_callee_saved_t, r3);
GEN_OFFSET_SYM(_callee_saved_t, r4);
GEN_OFFSET_SYM(_callee_saved_t, r5);
GEN_OFFSET_SYM(_callee_saved_t, r6);
GEN_OFFSET_SYM(_callee_saved_t, r7);
GEN_OFFSET_SYM(_callee_saved_t, psr);
GEN_OFFSET_SYM(_callee_saved_t, ra);
GEN_OFFSET_SYM(_callee_saved_t, sp);
GEN_OFFSET_SYM(_callee_saved_t, key);
GEN_OFFSET_SYM(_callee_saved_t, retval);
GEN_ABSOLUTE_SYM(__callee_saved_t_SIZEOF, sizeof(_callee_saved_t));

GEN_OFFSET_SYM(_thread_arch_t, flags);
GEN_OFFSET_SYM(_thread_arch_t, swap_return_value);
GEN_ABSOLUTE_SYM(__thread_arch_t_SIZEOF, sizeof(_thread_arch_t));

GEN_ABS_SYM_END


#endif // _RXV2_OFFSETS_INC_