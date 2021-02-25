/*
 * Copyright (c) 2019 Carlo Caione <ccaione@baylibre.com>
 * Copyright (c) 2020 KT-Elektronik, Klaucke und Partner GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Software interrupts utility code - Renesas rxv2 architecture (e.g. RX65N) implementation.
 */

#include <kernel.h>
#include <irq_offload.h>

static irq_offload_routine_t offload_routine;
static const void *offload_param;

// Called by ISR dispatcher
void z_irq_do_offload(const void *unused){
// check if: TODO: find the right ISR dispatcher where to call z_irq_do_offload, perhaps in an *.S-file?
	ARG_UNUSED(unused);
	offload_routine(offload_param);
}

void arch_irq_offload(irq_offload_routine_t routine, const void *parameter){
// check if: TODO: define IRQ_OFFLOAD_INTNUM in arch/rxv2/Kconfig and rename XCHAL_EXCM_LEVEL appropriately
	IRQ_CONNECT(CONFIG_IRQ_OFFLOAD_INTNUM, XCHAL_EXCM_LEVEL, z_irq_do_offload, NULL, 0);
	arch_irq_disable(CONFIG_IRQ_OFFLOAD_INTNUM);
	offload_routine = routine;
	offload_param = parameter;
	// z_xt_set_intset(BIT(CONFIG_IRQ_OFFLOAD_INTNUM)); // check if: TODO: needs own Renesas function
	/*
	 * Enable the software interrupt, in case it is disabled, so that IRQ
	 * offload is serviced.
	 */
	arch_irq_enable(CONFIG_IRQ_OFFLOAD_INTNUM);
}
