/*
 * Copyright (c) 2013-2016 Wind River Systems, Inc.
 * Copyright (c) 2020 KT-Elektronik, Klaucke und Partner GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Private kernel definitions (rxv2)
 *
 * This file contains private kernel structures definitions and various
 * other definitions for the Renesas rxv2 architecture (e.g. RX65N).
 *
 * This file is also included by assembly language files which must #define
 * _ASMLANGUAGE before including this header file.  Note that kernel
 * assembly source files obtains structure offset values via "absolute symbols"
 * in the offsets.o module.
 */

#ifndef ZEPHYR_ARCH_RXV2_INCLUDE_KERNEL_ARCH_DATA_H_
#define ZEPHYR_ARCH_RXV2_INCLUDE_KERNEL_ARCH_DATA_H_

#include <toolchain.h>
#include <linker/sections.h>
#include <arch/cpu.h>

#ifndef _ASMLANGUAGE
#include <kernel.h>
#include <zephyr/types.h>
#include <sys/dlist.h>
#include <sys/util.h>

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

// place C-code here

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _ASMLANGUAGE

#endif // ZEPHYR_ARCH_RXV2_INCLUDE_KERNEL_ARCH_DATA_H_