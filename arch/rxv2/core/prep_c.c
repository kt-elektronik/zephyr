/*
 * Copyright (c) 2019 Intel Corporation
 * Copyright (c) 2020 KT-Elektronik, Klaucke und Partner GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Full C support initialization
 *
 *
 * Initialization of full C support: zero the .bss and call z_cstart().
 *
 * Stack is available in this module, but not the global data/bss until their
 * initialization is performed.
 */

#include <kernel_internal.h>
#include <kernel.h>
#include <logging/log.h>
#include <toolchain.h>
#include <linker/sections.h>

 /**
 *
 * @brief Prepare to and run C code
 *
 * This routine prepares for the execution of and runs C code.
 *
 * @return N/A
 */

 void _PrepC(void)
{

	z_cstart();
	CODE_UNREACHABLE;
} // _PrepC
